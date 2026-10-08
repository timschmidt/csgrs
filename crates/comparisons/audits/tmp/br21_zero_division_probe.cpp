#include <real/real.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc != 2) return 64;
    const std::string mode = argv[1];
    try {
        if (mode == "rational") {
            boost::real::real_rational<> one("1/2"), zero("0");
            auto result = one / zero;
            std::cout << result.a.digits.size() << '/' << result.b.digits.size() << '\n';
        } else if (mode == "real-rational") {
            boost::real::real<> one("1/2", "rational"), zero("0", "rational");
            auto result = one / zero;
            std::cout << result.get_real_itr().get_interval() << '\n';
        } else if (mode == "real-explicit") {
            boost::real::real<> one("1"), zero("0");
            auto result = one / zero;
            std::cout << result.get_real_itr().get_interval() << '\n';
        }
    } catch (const std::exception& error) {
        std::cout << "exception: " << error.what() << '\n';
        return 2;
    }
}
