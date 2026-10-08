#include <iostream>
#include <sstream>

#define private public
#include "solvers/bounder.hpp"
#undef private

#include "function/function.hpp"

using namespace Ariadne;

int main() {
    const EffectiveVectorMultivariateFunction zero =
        EffectiveVectorMultivariateFunction::zeros(1, 1);
    const ValidatedVectorMultivariateFunction f = zero;
    const ExactBoxType D = {{-1.0_x, 1.0_x}};
    const ExactBoxType A(0u);
    const UpperBoxType B = D;
    const IntervalDomainType T = to_time_bounds(Dyadic(0), Dyadic(0));
    const PositiveFloatDP one = cast_positive(1.0_exact);
    const PositiveFloatDP two = cast_positive(2.0_exact);
    EulerBounder bounder;
    std::cout << "formula_widen_1="
              << bounder._formula(f, D, T, A, B, one, one) << std::endl;
    std::cout << "formula_widen_2="
              << bounder._formula(f, D, T, A, B, two, one) << std::endl;
}
