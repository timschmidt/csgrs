#include <real/real.hpp>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

using Clock = std::chrono::steady_clock;
using Real = boost::real::real<int>;
static volatile std::uint64_t sink;

static Real make_expression() {
    Real a("0.12345678901234567890123456789");
    Real b("0.98765432109876543210987654321");
    return ((a + b) * (a - b)) / (a * b);
}

template<class Work>
static double sample(int iterations, Work work) {
    const auto start = Clock::now();
    std::uint64_t local = 0;
    for (int i = 0; i < iterations; ++i) local += work();
    sink = local;
    const auto stop = Clock::now();
    return std::chrono::duration<double, std::micro>(stop - start).count()
           / iterations;
}

template<class Work>
static void report(const char* name, int iterations, Work work) {
    std::vector<double> samples;
    for (int warm = 0; warm < 3; ++warm) (void)sample(iterations, work);
    for (int i = 0; i < 21; ++i) samples.push_back(sample(iterations, work));
    std::sort(samples.begin(), samples.end());
    std::cout << name << " median_us=" << std::fixed << std::setprecision(3)
              << samples[samples.size()/2] << " min_us=" << samples.front()
              << " max_us=" << samples.back() << '\n';
}

int main() {
    report("construct_and_initial_interval", 20, [] {
        Real value = make_expression();
        return value.get_real_itr().get_interval().lower_bound.digits.size();
    });
    report("fresh_refine_to_four", 10, [] {
        Real value = make_expression();
        auto it = value.get_real_itr();
        for (int p = 1; p < 4; ++p) ++it;
        return it.get_interval().lower_bound.digits.size();
    });
    Real shared = make_expression();
    report("stateful_replay_to_four", 10, [&] {
        auto it = shared.get_real_itr();
        for (int p = 1; p < 4; ++p) ++it;
        return it.get_interval().lower_bound.digits.size();
    });
}
