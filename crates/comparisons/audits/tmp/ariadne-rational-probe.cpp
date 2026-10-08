#include <iostream>

#include "numeric/rational.hpp"

using namespace Ariadne;

int main() {
    std::cout << std::unitbuf;
    RationalBounds x(Rational(2), Rational(4));
    RationalBounds neg(Rational(-2), Rational(-1));
    RationalBounds cross(Rational(-1), Rational(2));
    std::cout << "hlf([2,4])=" << hlf(x) << '\n';
    std::cout << "pow([-2,-1],2)=" << pow(neg, 2u) << '\n';
    std::cout << "pow([-1,2],2)=" << pow(cross, 2u) << '\n';

    Rational nan = Rational::nan();
    std::cout << "cmp(nan,0)=" << cmp(nan, Rational(0)) << '\n';
    std::cout << "nan<0=" << (nan < Rational(0))
              << " nan>0=" << (nan > Rational(0))
              << " nan==nan=" << (nan == nan) << '\n';

    Rational huge(pow(Integer(10), 600u));
    std::cout << "huge=" << huge << '\n';
}
