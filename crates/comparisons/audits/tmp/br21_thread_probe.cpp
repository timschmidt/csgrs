#include <real/real.hpp>
#include <atomic>
#include <iostream>
#include <thread>

using Real = boost::real::real<int>;

static int digit(unsigned int n) {
    return static_cast<int>((n * 2654435761u) % 1000003u);
}

int main() {
    Real source(digit, 0);
    Real expression = source * source + source;
    std::atomic<unsigned long long> checksum{0};
    auto work = [&] {
        for (int pass = 0; pass < 20; ++pass) {
            auto it = expression.get_real_itr();
            for (int p = 0; p < 12; ++p) {
                const auto interval = it.get_interval();
                checksum.fetch_add(interval.lower_bound.digits.size(),
                                   std::memory_order_relaxed);
                ++it;
            }
        }
    };
    std::thread a(work), b(work);
    a.join();
    b.join();
    std::cout << "checksum=" << checksum.load() << '\n';
}
