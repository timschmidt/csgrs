#include "numeric/number.hpp"
#include "numeric/upper_number.hpp"
#include "numeric/lower_number.hpp"
#include "numeric/floatdp.hpp"
#include "numeric/float_upper_bound.hpp"
#include "numeric/float_bounds.hpp"
#include "numeric/rational.hpp"

#include <iostream>
#include <string>

using namespace Ariadne;

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "cast";
    if (mode == "cast") {
        FloatDPUpperBound upper(Rational(5, 2), dp);
        ValidatedUpperNumber validated = upper.operator ValidatedUpperNumber();
        ExactNumber exact = cast_exact(validated);
        std::cout << "validated_class=" << validated.class_name() << "\n";
        std::cout << "exact_class=" << exact.class_name() << "\n";
        std::cout << "exact_paradigm_value=" << exact << "\n";
        try {
            std::cout << "exact_get=" << exact.get(dp) << "\n";
        } catch (std::exception const& e) {
            std::cout << "exact_get_exception=" << e.what() << "\n";
        }
        return 0;
    }
    if (mode == "sqrt_bounds") {
        RationalBounds bounds(Rational(1), Rational(4));
        ValidatedNumber validated = bounds.operator ValidatedNumber();
        std::cout << "before_sqrt=" << validated << std::endl;
        auto result = sqrt(validated);
        std::cout << "after_sqrt=" << result << "\n";
        return 0;
    }
    if (mode == "kind") {
        for (OperatorCode op : {OperatorCode::FMA, OperatorCode::NUL,
                                OperatorCode::HLF, OperatorCode::ASIN,
                                OperatorCode::ACOS, OperatorCode::XOR,
                                OperatorCode::SGN}) {
            try {
                std::cout << name(op) << '=' << get_kind(op) << "\n";
            } catch (std::exception const& e) {
                std::cout << name(op) << " exception=" << e.what() << "\n";
            }
        }
        return 0;
    }
    return 2;
}
