#include <iostream>
#include <stdexcept>

#include "config.hpp"
#include "algebra/algebra.hpp"
#include "function/functional.hpp"
#include "function/formula.hpp"
#include "function/taylor_model.hpp"
#include "geometry/grid.hpp"
#include "hybrid/hybrid_automata.hpp"
#include "hybrid/hybrid_grid.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_space.hpp"

using namespace Ariadne;

int main(int argc, char**) {
    StringVariable a("a"), b("b");
    DiscreteLocation full({a|"one", b|"two"});
    try {
        DiscreteLocation part = restrict(full, Set<Identifier>{a.name()});
        std::cout << "restrict_succeeded=" << part << "\n";
    } catch (std::exception const& e) {
        std::cout << "restrict_threw=" << e.what() << "\n";
    }

    RealVariable x("x"), y("y"), z("z");
    DiscreteLocation q1(1), q2(2), q3(3);
    MonolithicHybridSpace small;
    small.new_location(q1, RealSpace({x}));
    MonolithicHybridSpace large;
    large.new_location(q1, RealSpace({x}));
    large.new_location(q2, RealSpace({x}));
    large.new_location(q3, RealSpace({x}));
    HybridSpace small_space(small), large_space(large);
    std::cout << "small_eq_large=" << small_space.operator==(large_space)
              << " large_eq_small=" << large_space.operator==(small_space) << "\n";

    HybridAutomaton resets;
    DiscreteEvent jump("jump");
    resets.new_mode(q1, {dot(x)=0, dot(y)=0});
    resets.new_mode(q2, {dot(x)=0, dot(y)=0});
    resets.new_transition(q1, jump, q2,
                          {next(y)=x+10, next(x)=y+20});
    auto reset = resets.reset_function(q1, jump);
    Point<FloatDPApproximation> reset_input(
        {FloatDPApproximation(1.0, dp), FloatDPApproximation(2.0, dp)});
    auto reset_value = evaluate(reset, reset_input);
    std::cout << "reordered_reset_value=" << reset_value << "\n";

    try {
        HybridAutomaton cyclic;
        cyclic.new_mode(q1, {let(z)=1, let(x)=y, let(y)=x});
        std::cout << "partial_cycle_accepted=true auxiliary_space="
                  << cyclic.continuous_auxiliary_space(q1) << "\n";
    } catch (std::exception const& e) {
        std::cout << "partial_cycle_threw=" << e.what() << "\n";
    }

    HybridGrid grid(large_space);
    HybridGridTreePaving paving(grid);
    paving[q1].adjoin_outer_approximation(ExactBoxType{{0,1}}, 0);
    paving[q2].adjoin_outer_approximation(ExactBoxType{{10,11}}, 0);
    HybridExactBox overlapping(q1, RealSpace({x}), ExactBoxType{{0,1}});
    std::cout << "separated_from_overlap=" << paving.separated(overlapping) << "\n";

    HybridExactBoxes bounds;
    bounds.insert(q1, RealSpace({x}), ExactBoxType{{-100,100}});
    bounds.insert(q2, RealSpace({x}), ExactBoxType{{-1,1}});
    std::cout << "two_location_inside=" << paving.inside(bounds) << "\n";

    if (argc > 1) {
        HybridExactBox absent(q3, RealSpace({x}), ExactBoxType{{0,1}});
        std::cout << "separated_from_absent=" << paving.separated(absent) << "\n";
    }
}
