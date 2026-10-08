#include <iostream>

#include "config.hpp"
#include "function/functional.hpp"
#include "function/formula.hpp"
#include "function/taylor_model.hpp"
#include "hybrid/hybrid_automata.hpp"
#include "hybrid/hybrid_orbit.hpp"
#include "hybrid/hybrid_simulator.hpp"
#include "hybrid/hybrid_time.hpp"

using namespace Ariadne;

int main() {
    DiscreteLocation q;
    RealVariable x("x");
    HybridAutomaton system;
    system.new_mode(q, {dot(x)=1});

    HybridSimulator simulator(system);
    simulator.configuration().set_step_size(0.4);
    Point<FloatDPApproximation> point({FloatDPApproximation(0.0, dp)});
    HybridApproximatePoint initial(q, RealSpace({x}), point);
    auto orbit = simulator.orbit(initial, HybridTime(1.0_x, 10));
    auto const& curve = orbit.curve(0);
    std::cout << "curve_points=" << curve.size()
              << " last_time=" << curve.end()->first
              << " last_state=" << curve.end()->second << "\n";
}
