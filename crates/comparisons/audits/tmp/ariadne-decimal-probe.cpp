#include <iostream>
#include <limits>
#include <string>

#include "utility/string.hpp"
#include "numeric/decimal.hpp"

using namespace Ariadne;

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "infinity") {
        Decimal d(std::numeric_limits<double>::infinity());
        std::cout << d << '\n';
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "nan") {
        Decimal d(std::numeric_limits<double>::quiet_NaN());
        std::cout << d << '\n';
        return 0;
    }

    Decimal one(1);
    Decimal two(2);
    std::cout << std::boolalpha
              << "one_lt_two=" << (one < two) << '\n'
              << "one_le_two=" << (one <= two) << '\n'
              << "two_gt_one=" << (two > one) << '\n'
              << "two_ge_one=" << (two >= one) << '\n';

    for (char const* text : {"", "+", "-", ".", "+.", "-."}) {
        Decimal d{String(text)};
        std::cout << "parsed[" << text << "]=" << d << '\n';
    }

    Decimal noncanonical(10, 1u);
    Decimal product = noncanonical * noncanonical;
    std::cout << "noncanonical_q=" << noncanonical._q
              << " product_q=" << product._q << '\n';
}
