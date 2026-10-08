#include <real/real.hpp>
#include <iostream>

using Real = boost::real::real<int>;

int main() {
    auto numeric = 0.123456789012345678_r;
    auto string = "0.123456789012345678"_r;
    auto rounded = "0.123457"_r;
    std::cout << "numeric=" << numeric << '\n'
              << "string=" << string << '\n'
              << "rounded=" << rounded << '\n';
}
