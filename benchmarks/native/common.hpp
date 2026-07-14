#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>

namespace csgrs_bench {

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

inline bool selected(std::string_view suite, std::string_view benchmark,
                     std::string_view benchmark_case) {
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
        scale_(std::max<std::size_t>(1, env_size("CSGRS_BENCH_SCALE", 1))) {
    std::cout << "engine,suite,benchmark,case,sample,iterations,elapsed_ns,"
                 "work_units,output_size,checksum\n";
  }

  template <typename Operation>
  void run(std::string_view suite, std::string_view benchmark,
           std::string_view benchmark_case, std::size_t base_iterations,
           Operation operation) const {
    if (!selected(suite, benchmark, benchmark_case)) {
      return;
    }
    const std::size_t iterations =
        std::max<std::size_t>(1, base_iterations * scale_);
    volatile std::uint64_t sink = 0;
    for (std::size_t warmup = 0; warmup < warmup_; ++warmup) {
      for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        sink = sink ^ operation().checksum;
      }
    }
    for (std::size_t sample = 0; sample < samples_; ++sample) {
      Measurement aggregate;
      const auto start = std::chrono::steady_clock::now();
      for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        aggregate += operation();
      }
      const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
      sink = sink ^ aggregate.checksum;
      std::cout << engine_ << ',' << suite << ',' << benchmark << ','
                << benchmark_case << ',' << sample << ',' << iterations << ','
                << elapsed << ',' << aggregate.work_units << ','
                << aggregate.output_size << ',' << aggregate.checksum << '\n';
    }
    (void)sink;
  }

private:
  std::string engine_;
  std::size_t samples_;
  std::size_t warmup_;
  std::size_t scale_;
};

inline std::uint64_t checksum(std::size_t facets, std::size_t corners) {
  return (static_cast<std::uint64_t>(facets) << 17U) ^
         static_cast<std::uint64_t>(corners);
}

} // namespace csgrs_bench
