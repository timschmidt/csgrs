#include <real/real.hpp>
#include <cstdlib>
#include <iostream>
#include <string>

using Real = boost::real::real<int>;

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const std::string mode = argv[1];
    try {
        Real result;
        if (mode == "sqrt_neg") result = Real::sqrt(Real("-1"));
        else if (mode == "log_zero") result = Real::log(Real("0"));
        else if (mode == "log_neg") result = Real::log(Real("-1"));
        else if (mode == "asin_hi") result = Real::asin(Real("1.1"));
        else if (mode == "asin_lo") result = Real::asin(Real("-1.1"));
        else if (mode == "acos_hi") result = Real::acos(Real("1.1"));
        else if (mode == "acos_lo") result = Real::acos(Real("-1.1"));
        else if (mode == "atan2_origin") result = Real::atan2(Real("0"), Real("0"));
        else if (mode == "bad_type") result = Real("1", "bogus");
        else return 3;
        auto it = result.get_real_itr();
        for (int p = 0; p < 6; ++p) {
            std::cout << p << ':' << it.get_interval() << '\n';
            ++it;
        }
        std::cout << "completed\n";
    } catch (const std::exception& e) {
        std::cout << "exception=" << e.what() << '\n';
        return 4;
    }
}
