#include <CGAL/Gmpq.h>
#include <CGAL/Root_of_traits.h>
#include <CGAL/version.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <vector>

namespace {

using Coefficient = CGAL::Gmpq;
using Root = CGAL::Root_of_traits<Coefficient>::Root_of_2;

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
std::size_t solve_once(const Coefficient& a, const Coefficient& b, const Coefficient& c) {
    std::vector<Root> roots;
    roots.reserve(2);
    CGAL::compute_roots_of_2(a, b, c, std::back_inserter(roots));
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "g"(roots.data()), "g"(roots.size()) : "memory");
#endif
    return roots.size();
}

}  // namespace

int main() {
    constexpr std::size_t samples = 50;
    constexpr std::size_t iterations_per_sample = 10'000;
    const Coefficient a(1);
    const Coefficient b(0);
    const Coefficient c(-2);

    std::size_t checksum = 0;
    for (std::size_t index = 0; index < iterations_per_sample; ++index) {
        checksum += solve_once(a, b, c);
    }

    std::vector<double> nanoseconds;
    nanoseconds.reserve(samples);
    for (std::size_t sample = 0; sample < samples; ++sample) {
        const auto start = std::chrono::steady_clock::now();
        for (std::size_t index = 0; index < iterations_per_sample; ++index) {
            checksum += solve_once(a, b, c);
        }
        const auto end = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration<double, std::nano>(end - start).count();
        nanoseconds.push_back(elapsed / static_cast<double>(iterations_per_sample));
    }

    std::sort(nanoseconds.begin(), nanoseconds.end());
    const double median = nanoseconds[nanoseconds.size() / 2];
    const double lower = nanoseconds[nanoseconds.size() / 20];
    const double upper = nanoseconds[nanoseconds.size() - 1 - nanoseconds.size() / 20];
    std::cout << "competitor_exact_quadratic_roots/cgal"
              << " cgal=" << CGAL_VERSION_STR
              << " median_ns=" << median
              << " p05_ns=" << lower
              << " p95_ns=" << upper
              << " checksum=" << checksum << '\n';
}
