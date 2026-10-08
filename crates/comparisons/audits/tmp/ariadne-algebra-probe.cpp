#include <cstdlib>
#include <iostream>
#include <string>

#include "config.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/symmetric_matrix.hpp"
#include "algebra/differential.hpp"
#include "algebra/differential.tpl.hpp"
#include "algebra/univariate_differential.hpp"
#include "algebra/univariate_differential.tpl.hpp"
#include "algebra/fixed_differential.hpp"
#include "algebra/fixed_univariate_differential.hpp"

using namespace Ariadne;

int main(int argc, char** argv) {
    if (argc != 2) return 64;
    const std::string which(argv[1]);
    using X = FloatDPApproximation;

    if (which == "vector-size") {
        Vector<X> a({1.0_x, 2.0_x}, dp);
        Vector<X> b({1.0_x}, dp);
        std::cout << "equal=" << (a == b) << "\n";
        return 0;
    }
    if (which == "symmetric-resize") {
        SymmetricMatrix<X> a(2u, dp);
        a.resize(3u);
        a.at(2u, 2u) = X(7.0_x, dp);
        std::cout << "shape=" << a.row_size() << 'x' << a.column_size()
                  << " value=" << a.at(2u, 2u) << "\n";
        return 0;
    }
    if (which == "univariate-derivative") {
        UnivariateDifferential<X> a(2u, {1.0_x, 2.0_x, 3.0_x}, dp);
        auto d = derivative(a);
        std::cout << "degree=" << d.degree() << " c0=" << d[0]
                  << " c1=" << d[1] << "\n";
        return 0;
    }
    if (which == "first-quotient") {
        FirstDifferential<X> a(1u, dp), b(1u, dp);
        a._value = X(2.0_x, dp); a._gradient[0] = X(3.0_x, dp);
        b._value = X(4.0_x, dp); b._gradient[0] = X(5.0_x, dp);
        const auto q = a / b;
        std::cout << "value=" << q.value() << " derivative=" << q.gradient(0u)
                  << " expected=0.125\n";
        return 0;
    }
    if (which == "second-square") {
        UnivariateSecondDifferential<X> a(X(2.0_x, dp), X(3.0_x, dp), X(4.0_x, dp));
        const auto q = sqr(a);
        std::cout << "value=" << q.value() << " derivative=" << q.gradient()
                  << " half_hessian=" << q.half_hessian() << " expected=25\n";
        return 0;
    }
    if (which == "sparse-affine") {
        Vector<X> v({0.0_x, 0.0_x}, dp);
        Matrix<X> g({{1.0_x, 2.0_x}, {3.0_x, 4.0_x}}, dp);
        auto a = Differential<X>::affine(1u, v, g);
        std::cout << "jacobian=" << a.jacobian() << "\n";
        return 0;
    }
    if (which == "sparse-check") {
        auto a = Differential<X>::variable(2u, 2u, X(1.0_x, dp), 0u);
        a.check();
        std::cout << "checked\n";
        return 0;
    }
    return 65;
}
