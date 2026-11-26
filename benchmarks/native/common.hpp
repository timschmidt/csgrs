#pragma once

/// Shared native benchmark fixture parsing, winding normalization and CSV timing.
/// This header defines a binary64 input boundary and diagnostic measurements, not
/// a geometry kernel or exact certification layer. Callers own semantic validation.
/// Environment parsing and counter overflow differ from the Rust benchmark harness.

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

/// Helpers shared by the CGAL and OpenCascade benchmark executables.
namespace csgrs_bench {

/// Indexed binary64 OBJ geometry after polygon fans and optional winding changes.
struct ObjTriangleSoup {
  /// Parsed positions; no exact decimal source or coordinate-error receipt survives.
  std::vector<std::array<double, 3>> points;
  /// Zero-based fan triangles, potentially flipped by component normalization.
  std::vector<std::array<std::size_t, 3>> triangles;
  /// Number of authored face records, before fan triangulation; defaults to zero.
  std::size_t source_faces{};
};

/// Recognizes case-folded 1/true/yes/on for YEAHRIGHT_BENCH, without trimming.
/// Missing/other values disable the optional fixture; conversion uses the C locale.
inline bool yeahright_enabled() {
  const char *raw = std::getenv("YEAHRIGHT_BENCH");
  if (raw == nullptr) {
    return false;
  }
  std::string value(raw);
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

/// Returns the compiled control path or a cwd-relative default; performs no acquisition.
inline std::filesystem::path yeahright_control_path() {
#ifdef CSGRS_YEAHRIGHT_CONTROL_OBJ
  return CSGRS_YEAHRIGHT_CONTROL_OBJ;
#else
  return "target/benchmark-fixtures/yeahright/controlmesh.obj";
#endif
}

/// Returns the compiled proxy path or hull default without checking content/source.
inline std::filesystem::path yeahright_boolean_proxy_path() {
#ifdef CSGRS_YEAHRIGHT_BOOLEAN_PROXY_OBJ
  return CSGRS_YEAHRIGHT_BOOLEAN_PROXY_OBJ;
#else
  return "target/benchmark-fixtures/yeahright/yeahright_boolean_hull.obj";
#endif
}

/// Returns the compiled hull path or cwd-relative default without generation/validation.
inline std::filesystem::path yeahright_boolean_hull_path() {
#ifdef CSGRS_YEAHRIGHT_BOOLEAN_HULL_OBJ
  return CSGRS_YEAHRIGHT_BOOLEAN_HULL_OBJ;
#else
  return "target/benchmark-fixtures/yeahright/yeahright_boolean_hull.obj";
#endif
}

/// Names the generated labyrinth OBJ; the Rust generator must have created it first.
inline std::filesystem::path generated_concave_path() {
#ifdef CSGRS_GENERATED_CONCAVE_OBJ
  return CSGRS_GENERATED_CONCAVE_OBJ;
#else
  return "target/benchmark-fixtures/generated/"
         "deterministic_concave_labyrinth_31x31x6.obj";
#endif
}

/// Names the generated foam OBJ; this helper neither creates nor hashes the asset.
inline std::filesystem::path generated_sierpinski_foam_path() {
#ifdef CSGRS_GENERATED_SIERPINSKI_FOAM_OBJ
  return CSGRS_GENERATED_SIERPINSKI_FOAM_OBJ;
#else
  return "target/benchmark-fixtures/generated/sierpinski_foam_level3.obj";
#endif
}

/// Parses the integer before the first slash, rejecting zero, trailing junk and bounds.
/// Positive indices are one-based; negatives are relative to vertices already seen.
/// Forward references are rejected. Other slash fields are ignored. Throws on parse
/// or bounds failure; assumes the fixture's vertex count fits in signed long long.
inline std::size_t obj_vertex_index(std::string_view token,
                                    std::size_t vertex_count) {
  const auto slash = token.find('/');
  const std::string index_text(token.substr(0, slash));
  std::size_t consumed{};
  const long long parsed = std::stoll(index_text, &consumed);
  if (consumed != index_text.size() || parsed == 0) {
    throw std::runtime_error("invalid OBJ vertex index: " + index_text);
  }
  const long long resolved =
      parsed > 0 ? parsed - 1 : static_cast<long long>(vertex_count) + parsed;
  if (resolved < 0 || static_cast<std::size_t>(resolved) >= vertex_count) {
    throw std::runtime_error("OBJ vertex index is out of range: " + index_text);
  }
  return static_cast<std::size_t>(resolved);
}

/// Makes indexed face components consistently oriented, then flips negative volume sums.
/// Requires paired undirected edges and consistent BFS parity, throwing otherwise.
/// The final signed six-volume is ordinary double arithmetic: zero, NaN and uncertain
/// signs are not rejected. Unlike the Rust helper, this is not a certified sign test.
/// It assumes valid indices, does not prove embeddedness/vertex-link manifoldness,
/// and positive orientation for every shell does not preserve nested cavity semantics.
/// Mutates per-triangle vertex order in place; points/source-face count remain unchanged.
inline void orient_closed_boolean_mesh(ObjTriangleSoup &soup) {
  /// Face index and whether an edge traverses from smaller to larger vertex index.
  using Incidence = std::pair<std::size_t, bool>;
  std::map<std::pair<std::size_t, std::size_t>, std::vector<Incidence>> edges;
  std::vector<std::vector<std::pair<std::size_t, bool>>> adjacent(
      soup.triangles.size());
  for (std::size_t triangle_index = 0;
       triangle_index < soup.triangles.size(); ++triangle_index) {
    const auto &triangle = soup.triangles[triangle_index];
    for (const auto [a, b] : {std::array{triangle[0], triangle[1]},
                              std::array{triangle[1], triangle[2]},
                              std::array{triangle[2], triangle[0]}}) {
      edges[std::minmax(a, b)].emplace_back(triangle_index, a < b);
    }
  }
  for (const auto &[edge, incidence] : edges) {
    (void)edge;
    if (incidence.size() != 2) {
      throw std::runtime_error(
          "YeahRight control mesh is not a closed triangle soup");
    }
    const auto [left, left_forward] = incidence[0];
    const auto [right, right_forward] = incidence[1];
    const bool differs = left_forward == right_forward;
    adjacent[left].emplace_back(right, differs);
    adjacent[right].emplace_back(left, differs);
  }

  std::vector<int> flipped(soup.triangles.size(), -1);
  std::vector<std::vector<std::size_t>> components;
  for (std::size_t seed = 0; seed < soup.triangles.size(); ++seed) {
    if (flipped[seed] >= 0) {
      continue;
    }
    flipped[seed] = 0;
    std::queue<std::size_t> pending;
    pending.push(seed);
    std::vector<std::size_t> component;
    while (!pending.empty()) {
      const std::size_t current = pending.front();
      pending.pop();
      component.push_back(current);
      for (const auto [neighbor, differs] : adjacent[current]) {
        const int required = flipped[current] ^ static_cast<int>(differs);
        if (flipped[neighbor] >= 0 && flipped[neighbor] != required) {
          throw std::runtime_error("YeahRight control mesh is not orientable");
        }
        if (flipped[neighbor] < 0) {
          flipped[neighbor] = required;
          pending.push(neighbor);
        }
      }
    }
    components.push_back(std::move(component));
  }
  for (std::size_t index = 0; index < soup.triangles.size(); ++index) {
    if (flipped[index] != 0) {
      std::swap(soup.triangles[index][1], soup.triangles[index][2]);
    }
  }
  for (const auto &component : components) {
    double signed_volume{};
    for (const std::size_t triangle_index : component) {
      const auto &triangle = soup.triangles[triangle_index];
      const auto &a = soup.points[triangle[0]];
      const auto &b = soup.points[triangle[1]];
      const auto &c = soup.points[triangle[2]];
      signed_volume += a[0] * (b[1] * c[2] - b[2] * c[1]) +
                       a[1] * (b[2] * c[0] - b[0] * c[2]) +
                       a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    if (signed_volume < 0.0) {
      for (const std::size_t triangle_index : component) {
        std::swap(soup.triangles[triangle_index][1],
                  soup.triangles[triangle_index][2]);
      }
    }
  }
}

/// Reads v/f records into doubles and fans, then applies component winding normalization.
/// Uses only the first three vertex fields and position part of face tokens; ignores
/// other directives. Does not test polygon planarity/convexity, coordinate finiteness
/// or source fidelity. Open/record/index/orientation errors throw; post-open stream
/// failures are not distinguished from end of input. Empty input yields an empty soup.
inline ObjTriangleSoup read_obj_boolean_mesh(
    const std::filesystem::path &path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open OBJ: " + path.string());
  }
  ObjTriangleSoup soup;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream input(line);
    std::string directive;
    input >> directive;
    if (directive == "v") {
      std::array<double, 3> point{};
      if (!(input >> point[0] >> point[1] >> point[2])) {
        throw std::runtime_error("invalid OBJ vertex in " + path.string());
      }
      soup.points.push_back(point);
    } else if (directive == "f") {
      std::vector<std::size_t> face;
      std::string token;
      while (input >> token) {
        face.push_back(obj_vertex_index(token, soup.points.size()));
      }
      if (face.size() < 3) {
        throw std::runtime_error("invalid OBJ face in " + path.string());
      }
      ++soup.source_faces;
      for (std::size_t corner = 1; corner + 1 < face.size(); ++corner) {
        soup.triangles.push_back({face[0], face[corner], face[corner + 1]});
      }
    }
  }
  orient_closed_boolean_mesh(soup);
  return soup;
}

/// Per-iteration counters whose meanings are supplied by each benchmark closure.
/// All counters start at zero; none independently establishes completed geometry.
struct Measurement {
  /// Nominal work count, often input facets; arithmetic wraps modulo 2^64.
  std::uint64_t work_units{};
  /// Nominal output size/count; arithmetic wraps modulo 2^64.
  std::uint64_t output_size{};
  /// Caller-defined fingerprint, accumulated with wrapping addition.
  std::uint64_t checksum{};

  /// Adds all three counters with unsigned wraparound, returning this aggregate.
  Measurement &operator+=(const Measurement &other) {
    work_units += other.work_units;
    output_size += other.output_size;
    checksum += other.checksum;
    return *this;
  }
};

/// Extracts an unsigned integer prefix with stream rules, or returns the fallback.
/// Does not require whole-string consumption or reject a minus sign explicitly;
/// this is not the Rust harness's whole-string usize parser. No budget is enforced.
inline std::size_t env_size(const char *name, std::size_t fallback) {
  const char *raw = std::getenv(name);
  if (raw == nullptr) {
    return fallback;
  }
  std::istringstream input(raw);
  std::size_t value{};
  return input >> value ? value : fallback;
}

/// Accepts exact cold/warm labels, defaults an absent value to warm, otherwise throws.
/// The label does not flush geometry caches or establish process temperature.
inline std::string env_temperature() {
  const char *raw = std::getenv("CSGRS_BENCH_TEMPERATURE");
  if (raw == nullptr || std::string_view(raw) == "warm") {
    return "warm";
  }
  if (std::string_view(raw) == "cold") {
    return "cold";
  }
  throw std::runtime_error(
      "CSGRS_BENCH_TEMPERATURE must be 'cold' or 'warm'");
}

/// Applies corpus-Boolean skip, dangerous opt-in and stress opt-in/skip gates in order.
/// Opt-in requires exactly 1; a present skip value other than 0 enables skipping.
/// Remaining comma-separated substrings match suite/benchmark/case without trimming.
/// Missing or empty filter selects all gated cases, unlike the Rust empty-filter rule.
inline bool selected(std::string_view suite, std::string_view benchmark,
                     std::string_view benchmark_case) {
  if (suite == "corpus" && benchmark == "boolean_all") {
    const char *skip = std::getenv("CSGRS_BENCH_SKIP_CORPUS_BOOLEANS");
    if (skip != nullptr && std::string_view(skip) != "0") {
      return false;
    }
  }
  if (suite == "dangerous") {
    const char *enabled = std::getenv("CSGRS_BENCH_ENABLE_DANGEROUS");
    if (enabled == nullptr || std::string_view(enabled) != "1") {
      return false;
    }
  }
  if (suite == "stress") {
    const char *enabled = std::getenv("CSGRS_BENCH_ENABLE_STRESS");
    if (enabled == nullptr || std::string_view(enabled) != "1") {
      return false;
    }
    const char *skip_stress = std::getenv("CSGRS_BENCH_SKIP_STRESS");
    if (skip_stress != nullptr && std::string_view(skip_stress) != "0") {
      return false;
    }
  }
  const char *raw = std::getenv("CSGRS_BENCH_FILTER");
  if (raw == nullptr || *raw == '\0') {
    return true;
  }
  const std::string qualified = std::string(suite) + "/" +
                                std::string(benchmark) + "/" +
                                std::string(benchmark_case);
  std::istringstream filters(raw);
  std::string filter;
  while (std::getline(filters, filter, ',')) {
    if (!filter.empty() && qualified.find(filter) != std::string::npos) {
      return true;
    }
  }
  return false;
}

/// Captured repetitions/labels with live environment filtering and per-sample CSV output.
/// Warmups, earlier cases and shared sources can warm caches. Labels are not escaped.
class Harness {
public:
  /// Captures environment counts, emits opt-in warnings and prints the CSV header.
  /// Samples/scale clamp to one; warmups may be zero. A zero iteration override means
  /// use scaling. Invalid temperature throws. Counts impose no total resource bound.
  explicit Harness(std::string engine)
      : engine_(std::move(engine)),
        samples_(std::max<std::size_t>(1, env_size("CSGRS_BENCH_SAMPLES", 10))),
        warmup_(env_size("CSGRS_BENCH_WARMUP", 2)),
        stress_samples_(std::max<std::size_t>(
            1, env_size("CSGRS_BENCH_STRESS_SAMPLES", 1))),
        stress_warmup_(env_size("CSGRS_BENCH_STRESS_WARMUP", 0)),
        scale_(std::max<std::size_t>(1, env_size("CSGRS_BENCH_SCALE", 1))),
        iterations_override_(env_size("CSGRS_BENCH_ITERATIONS", 0)),
        sample_offset_(env_size("CSGRS_BENCH_SAMPLE_OFFSET", 0)),
        temperature_(env_temperature()) {
    if (const char *enabled = std::getenv("CSGRS_BENCH_ENABLE_DANGEROUS");
        enabled != nullptr && std::string_view(enabled) == "1") {
      std::cerr << "WARNING: dangerous benchmark workloads are enabled and may "
                   "exhaust system memory\n";
    }
    if (const char *enabled = std::getenv("CSGRS_BENCH_ENABLE_STRESS");
        enabled != nullptr && std::string_view(enabled) == "1") {
      std::cerr << "WARNING: stress benchmark workloads are enabled and may "
                   "consume tens of GiB\n";
    }
    std::cout << "engine,temperature,suite,benchmark,case,sample,iterations,elapsed_ns,"
                 "work_units,output_size,checksum\n";
  }

  /// Runs selected warmups and measured repetitions of a callback returning Measurement.
  /// Corpus/stress/dangerous use stress counts. Scaling and sample offsets use size_t
  /// arithmetic and may wrap; aggregate counters also wrap. The timed interval includes
  /// callback work/destruction and accumulation, excluding CSV output and sink updates.
  /// Exceptions propagate, leaving any prior rows intact; there is no typed failure row
  /// or independent correctness check. Callback is copied into this invocation.
  template <typename Operation>
  void run(std::string_view suite, std::string_view benchmark,
           std::string_view benchmark_case, std::size_t base_iterations,
           Operation operation) const {
    if (!selected(suite, benchmark, benchmark_case)) {
      return;
    }
    const std::size_t iterations = iterations_override_ == 0
                                       ? std::max<std::size_t>(
                                             1, base_iterations * scale_)
                                       : iterations_override_;
    const bool is_stress =
        suite == "corpus" || suite == "stress" || suite == "dangerous";
    const std::size_t samples = is_stress ? stress_samples_ : samples_;
    const std::size_t warmups = is_stress ? stress_warmup_ : warmup_;
    volatile std::uint64_t sink = 0;
    for (std::size_t warmup = 0; warmup < warmups; ++warmup) {
      for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        sink = sink ^ operation().checksum;
      }
    }
    for (std::size_t sample = 0; sample < samples; ++sample) {
      Measurement aggregate;
      const auto start = std::chrono::steady_clock::now();
      for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        aggregate += operation();
      }
      const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
      sink = sink ^ aggregate.checksum;
      std::cout << engine_ << ',' << temperature_ << ',' << suite << ','
                << benchmark << ',' << benchmark_case << ','
                << sample_offset_ + sample << ',' << iterations << ',' << elapsed
                << ',' << aggregate.work_units << ',' << aggregate.output_size
                << ',' << aggregate.checksum << '\n';
    }
    (void)sink;
  }

private:
  /// Engine label emitted verbatim, without CSV escaping.
  std::string engine_;
  /// Ordinary timed sample count, at least one.
  std::size_t samples_;
  /// Ordinary untimed repetition batches, permitting zero.
  std::size_t warmup_;
  /// Timed samples for corpus/stress/dangerous suites, at least one.
  std::size_t stress_samples_;
  /// Untimed batches for corpus/stress/dangerous suites, permitting zero.
  std::size_t stress_warmup_;
  /// Multiplier for caller base iterations when no override is selected.
  std::size_t scale_;
  /// Explicit iterations per sample, with zero meaning no override.
  std::size_t iterations_override_;
  /// Added to each emitted zero-based sample index using ordinary unsigned arithmetic.
  std::size_t sample_offset_;
  /// Validated display label only; no cache-reset behavior.
  std::string temperature_;
};

/// Combines facet/corner counts with a 17-bit shift and XOR; not a geometric hash.
inline std::uint64_t checksum(std::size_t facets, std::size_t corners) {
  return (static_cast<std::uint64_t>(facets) << 17U) ^
         static_cast<std::uint64_t>(corners);
}

} // namespace csgrs_bench
