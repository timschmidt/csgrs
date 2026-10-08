#include "ariadne.hpp"
#include "dynamics/2D_pde.hpp"

#include <iostream>

int main() {
    using namespace Ariadne;
    Parameter2D<DoublePrecision> x(double_precision);
    Parameter2D<DoublePrecision> y(double_precision);
    x.length = 3;
    y.length = 3;
    x.x0 = 1;
    y.x0 = 1;
    const auto result = pde_2d_solver(x, y, 3u, 3u, double_precision);
    const auto expected = exp(FloatDP(-1, double_precision)).value();
    const auto observed = result[{1u, 2u, 0u}];
    std::cout << "shape=" << result.size(0) << 'x' << result.size(1)
              << 'x' << result.size(2)
              << " boundary_initial_expected=" << expected
              << " boundary_initial_observed=" << observed
              << " changed=" << decide(observed != expected) << '\n';
    return decide(observed != expected) ? 0 : 1;
}
