#include <real/real.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) return 64;
    const std::string mode = argv[1];
    const std::string input = argc >= 3 ? argv[2] : std::string();
    try {
        if (mode == "parse") {
            auto [integer, decimal, exponent, positive] =
                boost::real::exact_number<>::number_from_string(input);
            std::cout << "ok integer='" << integer << "' decimal='" << decimal
                      << "' exponent=" << exponent << " positive=" << positive << '\n';
        } else if (mode == "exact") {
            boost::real::exact_number<> value(input);
            std::cout << "ok digits=" << value.digits.size()
                      << " exponent=" << value.exponent
                      << " positive=" << value.positive
                      << " text='" << value.as_string() << "'\n";
        } else if (mode == "integer") {
            boost::real::integer_number<> value(input);
            std::cout << "ok digits=" << value.digits.size()
                      << " positive=" << value.positive << '\n';
        } else if (mode == "real") {
            boost::real::real<> value(input);
            auto it = value.get_real_itr().cbegin();
            std::cout << "ok " << it.get_interval() << '\n';
        } else if (mode == "default-exact") {
            boost::real::exact_number<> value;
            std::cout << value.as_string() << '\n';
        } else if (mode == "default-real") {
            boost::real::real<> value;
            auto it = value.get_real_itr().cbegin();
            std::cout << it.get_interval() << '\n';
        } else {
            return 65;
        }
    } catch (const std::exception& error) {
        std::cout << "exception: " << error.what() << '\n';
        return 2;
    }
}
