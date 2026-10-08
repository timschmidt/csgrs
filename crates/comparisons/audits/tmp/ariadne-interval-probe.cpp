#include <iostream>

#include "geometry/interval.hpp"

using namespace Ariadne;

int main() {
    ExactIntervalType a(0.0_x, 1.0_x);
    ExactIntervalType b(-1.0_x, 1.0_x);
    std::cout << "equal_a_b=" << static_cast<bool>(equal(a, b)) << '\n';
    std::cout << "equal_b_a=" << static_cast<bool>(equal(b, a)) << '\n';

    UpperIntervalType positive(1.0_x, 1.0_x);
    UpperIntervalType negative(-1.0_x, -1.0_x);
    UpperIntervalType zero(0.0_x, 0.0_x);
    std::cout << "widen_positive=" << widen_domain(positive) << '\n';
    std::cout << "widen_negative=" << widen_domain(negative) << '\n';
    std::cout << "widen_zero=" << widen_domain(zero) << '\n';
    std::cout << "approx_zero=" << approximate_domain(zero) << '\n';
    std::cout << "floatmp_name=" << class_name<Interval<FloatMP>>() << '\n';
}
