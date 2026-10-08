#include "ariadne.hpp"
#include "dynamics/reachability_analyser.tpl.hpp"

#include <iostream>

int main() {
    using namespace Ariadne;

    const Real time(Rational(3, 5));
    const Real lock(1);
    const Natural steps = compute_time_steps(time, lock);
    const Real remainder = time - steps * lock;

    std::cout << "time=" << time
              << " lock=" << lock
              << " steps=" << steps
              << " remainder=" << remainder
              << " remainder_bounds="
              << remainder.compute_get(Effort(0u), double_precision)
              << " negative=" << decide(remainder < Real(0)) << '\n';
    return steps == 1u && decide(remainder < Real(0)) ? 0 : 1;
}
