#pragma once

#include <algorithm>
#include <array>
#include <chrono>
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

namespace csgrs_bench {

struct ObjTriangleSoup {
  std::vector<std::array<double, 3>> points;
  std::vector<std::array<std::size_t, 3>> triangles;
  std::size_t source_faces{};
};

inline std::filesystem::path yeahright_control_path() {
#ifdef CSGRS_YEAHRIGHT_CONTROL_OBJ
  return CSGRS_YEAHRIGHT_CONTROL_OBJ;
#else
  return "../data/yeahright/controlmesh.obj";
#endif
}

inline std::filesystem::path yeahright_boolean_proxy_path() {
#ifdef CSGRS_YEAHRIGHT_BOOLEAN_PROXY_OBJ
  return CSGRS_YEAHRIGHT_BOOLEAN_PROXY_OBJ;
#else
  return "../data/yeahright/controlmesh_boolean_proxy.obj";
#endif
}

inline std::filesystem::path yeahright_boolean_hull_path() {
#ifdef CSGRS_YEAHRIGHT_BOOLEAN_HULL_OBJ
  return CSGRS_YEAHRIGHT_BOOLEAN_HULL_OBJ;
#else
  return "../data/yeahright/yeahright_boolean_hull.obj";
#endif
}

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

inline void orient_closed_triangle_soup(ObjTriangleSoup &soup) {
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

inline ObjTriangleSoup read_obj_triangle_soup(
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
  orient_closed_triangle_soup(soup);
  return soup;
}

struct Measurement {
  std::uint64_t work_units{};
  std::uint64_t output_size{};
  std::uint64_t checksum{};

  Measurement &operator+=(const Measurement &other) {
    work_units += other.work_units;
    output_size += other.output_size;
    checksum += other.checksum;
    return *this;
  }
};

inline std::size_t env_size(const char *name, std::size_t fallback) {
  const char *raw = std::getenv(name);
  if (raw == nullptr) {
    return fallback;
  }
  std::istringstream input(raw);
  std::size_t value{};
  return input >> value ? value : fallback;
}

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

class Harness {
public:
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
  std::string engine_;
  std::size_t samples_;
  std::size_t warmup_;
  std::size_t stress_samples_;
  std::size_t stress_warmup_;
  std::size_t scale_;
  std::size_t iterations_override_;
  std::size_t sample_offset_;
  std::string temperature_;
};

inline std::uint64_t checksum(std::size_t facets, std::size_t corners) {
  return (static_cast<std::uint64_t>(facets) << 17U) ^
         static_cast<std::uint64_t>(corners);
}

} // namespace csgrs_bench
