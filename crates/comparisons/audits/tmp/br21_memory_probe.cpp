#include <real/real.hpp>
#include <iostream>

using Real = boost::real::real<int>;

int main() {
    std::size_t checksum = 0;
    for (int pass = 0; pass < 3; ++pass) {
        Real a("0.12345678901234567890123456789");
        Real b("0.98765432109876543210987654321");
        Real value = ((a + b) * (a - b)) / (a * b);
        auto it = value.get_real_itr();
        for (int p = 0; p < 4; ++p) {
            checksum += it.get_interval().lower_bound.digits.size();
            ++it;
        }
    }
    std::cout << "checksum=" << checksum << '\n';
}
