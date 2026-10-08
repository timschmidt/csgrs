#include <iostream>

#include "numeric/floatdp.hpp"
#include "numeric/float_bounds.hpp"
#include "numeric/real.hpp"

using namespace Ariadne;

int main() {
    DoublePrecision dp;
    Real p = 4 * atan(Real(1));
    Real q = p;
    Real p2 = 4 * atan(Real(1));
    Real s = sqrt(Real(2));

    std::cout << "same(1,1)=" << same(Real(1), Real(1)) << '\n';
    std::cout << "p.bounds=" << p.get(dp) << '\n';
    std::cout << "same(p,p)=" << same(p, p) << '\n';
    std::cout << "same(p,q-copy)=" << same(p, q) << '\n';
    std::cout << "same(p,p2-equivalent-expression)=" << same(p, p2) << '\n';
    std::cout << "s.bounds=" << s.get(dp) << '\n';
    std::cout << "same(s,s)=" << same(s, s) << '\n';
}
