/***************************************************************************
 *            test_hybrid_evolver.cpp
 *
 *  Copyright  2006-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <iostream>

#include "config.hpp"
#include "utility/tuple.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "function/function.hpp"
#include "geometry/box.hpp"
#include "geometry/list_set.hpp"
#include "geometry/affine_set.hpp"
#include "solvers/integrator.hpp"
#include "dynamics/orbit.hpp"
#include "io/graphics_interface.hpp"
#include "io/figure.hpp"
#include "hybrid/hybrid_automata.hpp"
#include "hybrid/hybrid_time.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "hybrid/hybrid_evolver.hpp"
#include "hybrid/hybrid_graphics.hpp"
#include "hybrid/hybrid_automaton-composite.hpp"
#include "io/command_line_interface.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

Dyadic half(0.5_x);
Dyadic quarter(0.25_x);
Dyadic eighth(0.125_x);

Colour reach_set_colour(0.25,0.25,0.50);
Colour intermediate_set_colour(0.50,0.50,0.75);
Colour final_set_colour(0.75,0.75,1.00);
Colour initial_set_colour(0.75,0.75,1.00);
Colour guard_set_colour(0.75,0.75,0.75);

class TestHybridEvolver
{
    static RealVariable x,y;
    static DiscreteLocation q;
    static DiscreteEvent e;
    static Real zero,one;
  private:
    string evolver_name;
    std::shared_ptr<GradedTaylorSeriesIntegrator> evolver_integrator;
    mutable std::shared_ptr<HybridEvolverBase> evolver_ptr;
  private:
    Void _set_evolver(const HybridAutomatonInterface& system) const;
  public:
    TestHybridEvolver(
            const string evolver_name,
            const GradedTaylorSeriesIntegrator& evolver_integrator);
    Void test_all() const;
    Void test_flow() const;
    Void test_affine_flow() const;
    Void test_exact_final_time() const;
    Void test_maximum_steps() const;
    Void test_urgent_event() const;
    Void test_empty_interior() const;
    Void test_partial_event() const;
    Void test_step_size_event() const;
    Void test_initially_active_event() const;
    Void test_initially_active_attracting_event() const;
    Void test_initially_active_repelling_event() const;
    Void test_impact() const;
    Void test_tangency() const;
    Void test_simultaneous_events() const;
    Void test_creep() const;
    Void test_unwind() const;
    Void test_permissive() const;

    Void test_splitting_on_urgent_event() const;
    Void test_affine_hysteresis() const;
    Void test_transverse_linear_crossing() const;
    Void test_transverse_cubic_crossing() const;
    Void test_transverse_cube_root_crossing() const;
    Void test_constant_derivative_system() const;

};

TestHybridEvolver::TestHybridEvolver(
        const string ev_name,
        const GradedTaylorSeriesIntegrator& ev_integrator)
    : evolver_name(ev_name)
    , evolver_integrator(ev_integrator.clone())
{
}

Void TestHybridEvolver::_set_evolver(const HybridAutomatonInterface& system) const
{
    evolver_ptr.reset(new GeneralHybridEvolver(system));
    evolver_ptr->set_integrator(*evolver_integrator);
}

Void TestHybridEvolver::test_all() const {
    ARIADNE_TEST_CALL(test_flow());
    ARIADNE_TEST_CALL(test_affine_flow());
    ARIADNE_TEST_CALL(test_exact_final_time());
    ARIADNE_TEST_CALL(test_maximum_steps());
    ARIADNE_TEST_CALL(test_urgent_event());
    ARIADNE_TEST_CALL(test_step_size_event());
    ARIADNE_TEST_CALL(test_empty_interior());
    ARIADNE_TEST_CALL(test_partial_event());
    ARIADNE_TEST_CALL(test_initially_active_event());
    ARIADNE_TEST_CALL(test_initially_active_attracting_event());
    ARIADNE_TEST_CALL(test_initially_active_repelling_event());
    ARIADNE_TEST_CALL(test_simultaneous_events());
    ARIADNE_TEST_CALL(test_impact());
    ARIADNE_TEST_CALL(test_creep());
    ARIADNE_TEST_CALL(test_unwind());
    ARIADNE_TEST_CALL(test_tangency());
    ARIADNE_TEST_CALL(test_permissive());

    ARIADNE_TEST_CALL(test_splitting_on_urgent_event());
    ARIADNE_TEST_CALL(test_affine_hysteresis());
    ARIADNE_TEST_CALL(test_transverse_linear_crossing());
    ARIADNE_TEST_CALL(test_transverse_cubic_crossing());
    ARIADNE_TEST_CALL(test_transverse_cube_root_crossing());
    ARIADNE_TEST_CALL(test_constant_derivative_system());
}

RealVariable TestHybridEvolver::x("x");
RealVariable TestHybridEvolver::y("y");
DiscreteEvent TestHybridEvolver::e("e");
DiscreteLocation TestHybridEvolver::q;
Real TestHybridEvolver::zero(0);
Real TestHybridEvolver::one(1);

Void TestHybridEvolver::test_flow() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    RealSpace space=automaton.continuous_state_space(q);

    HybridBoxSet initial(q,{-eighth<=x<=eighth,-eighth<=y<=eighth});
    HybridTime time(2.5_x,3);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(1.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),3u);
    ARIADNE_TEST_CHECK_WARN(orbit.intermediate().size(),2u);

    ARIADNE_TEST_PRINT(orbit);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-flow"),Axes2d(-0.5_x,x,+3.5_x, -1.0_x,y, +3.0_x),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}


Void TestHybridEvolver::test_exact_final_time() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    RealSpace space=automaton.continuous_state_space(q);

    HybridBoxSet initial(q,{-eighth<=x<=eighth,-eighth<=y<=eighth});
    HybridTime time(2.0_x,1);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(1.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);

    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),2u);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-exact_final_time"),Axes2d(-0.5_x,x,+3.5_x, -1.0_x,y,+3.0_x),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}


//! A simple test for the maximum number of steps
Void TestHybridEvolver::test_maximum_steps() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    const Dyadic c1(1.5); const Dyadic c2(1.0/16); const Dyadic c3(0.5);
    automaton.new_transition(q,e,q,{next(x)=x-c1,next(y)=y},x+c2*y-c3>=0,EventKind::URGENT);
    //automaton.new_transition(e,q,q,(x-1.5_x,y),x-0.5_x,EventKind::URGENT);
    // FIXME: Change so that hitting coordinate guard is not an error.

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=eighth,-eighth<=y<=eighth});
    HybridTime time(1.0_x,1);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(1.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);

    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),1u);

    Axes2d axes(-1.5_x,space[0],+2.5_x, -1.0_x,space[1],+3.0_x);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-maximum_steps"),axes,
         //guard_set_colour,BoundedConstraintSet(bounding_box,x+y/16-0.5>=0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}

//! A simple test for an urgent event
Void TestHybridEvolver::test_urgent_event() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    const Dyadic c1(1.5); const Dyadic c2(1.0/16); const Dyadic c3(0.5);
    automaton.new_transition(q,e,q,{next(x)=x-c1,next(y)=y},x+c2*y-c3>=0,EventKind::URGENT);
    //automaton.new_transition(e,q,q,(x-1.5_x,y),x-0.5_x,urgent);
    // FIXME: Change so that hitting coordinate guard is not an error.

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=eighth,-eighth<=y<=eighth});
    HybridTime time(1.0_x,2);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(1.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);

    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),2u);

    Axes2d axes(-1.5_x,space[0],+2.5_x, -1.0_x,space[1],+3.0_x);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-urgent_event"),axes,
         //guard_set_colour,BoundedConstraintSet(bounding_box,x+y/16-0.5>=0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}


// A test to ensure that an event which is active at the final time does actually occur
Void TestHybridEvolver::test_empty_interior() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    automaton.new_transition(q,e,q,{next(x)=x-2,next(y)=y},x-1>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=quarter,-eighth<=y<=eighth});
    HybridTime time(2.0_x,3);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);

    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),2u);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-empty_interior"),Axes2d(-1.5_x,space[0],+2.5_x, -1.0_x,space[1],+3.0_x),
         //guard_set_colour,ExactBoxType(2,1.0,8.0,-8.0,+8.0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}

// A test to ensure that an event which is active at the final time does actually occur
Void TestHybridEvolver::test_partial_event() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    automaton.new_transition(q,e,q,{next(x)=x-2,next(y)=y},x-y/16-2>=0,EventKind::URGENT);
    //automaton.new_transition(q,e,q,(x-2,y),x-2>=0,EventKind::URGENT);
    //FIXME: Need to allow domain of TaylorFunction to have empty interior

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=quarter,-eighth<=y<=eighth});
    HybridTime time(2.0_x,3);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);

    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),2u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),2u);

    Axes2d axes(-1.5_x,x,+2.5_x, -1.0_x,y,+3.0_x);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-partial_event"),axes,
         //guard_set_colour,BoundedConstraintSet(bounding_box,x-y/16-2>=0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}


// A test to ensure that an event which would be active after a time step
// but is avoided because the evolution is completed before the event would occur,
// does not occur
Void TestHybridEvolver::test_step_size_event() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    automaton.new_transition(q,e,q,{next(x)=x-2,next(y)=y},x-2>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=eighth,-eighth<=y<=eighth});
    HybridTime time(1.0_x,3);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);

    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),1u);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-step_size_event"),Axes2d(-0.5_x,x,+2.5_x, -1.0_x,y,+3.0_x),
         //guard_set_colour,ExactBoxType(2,2.0,8.0,-8.0,+8.0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}


Void TestHybridEvolver::test_initially_active_event() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one});
    automaton.new_transition(q,e,q,{next(x)=x+1,next(y)=y},-x>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-1.625_dy<=x<=-1.375_dy,-0.125_dy<=y<=0.125_dy});
    HybridTime time(1.0_x,4);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);

    // There should be two components of the reachable set which come from initially active events, and one from flowing
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),3u);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-initially_active"),Axes2d(-2.0_x,x,+2.0_x, -1.0_x,y,+2.0_x),
         //guard_set_colour,ExactBoxType(2,-8.0,0.0,-8.0,+8.0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());


}

Void TestHybridEvolver::test_initially_active_attracting_event() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=-one/2,dot(y)=one});
    automaton.new_transition(q,e,q,{next(x)=x+1,next(y)=y},-x-y/256>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=quarter,-eighth<=y<=eighth});
    HybridTime time(1.0_x,4);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-initially_active_attracting"),Axes2d(-1.0_x,x,+2.0_x, -1.0_x,y,+2.0_x),
         //guard_set_colour,ExactBoxType(2,-1.0,0.0,-8.0,+8.0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());


}

Void TestHybridEvolver::test_initially_active_repelling_event() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=+one/2,dot(y)=one});
    automaton.new_transition(q,e,q,{next(x)=x+1,next(y)=y},-x>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=quarter,-eighth<=y<=eighth});
    HybridTime time(1.0_x,2);

    _set_evolver(automaton);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-initially_active_repelling"),Axes2d(-1.0_x,x,+2.0_x, -1.0_x,y,+2.0_x),
         guard_set_colour,HybridRealBox(q,{-1<=x<=0,-8<=y<=+8}),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}



Void TestHybridEvolver::test_impact() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=y,dot(y)=zero});
//     automaton.new_transition(q,e,q,{x,y-2},x-1,impact);
    //automaton.new_transition(q,e,q,(x+0.001_x*y-0.0004_x,y-2),x-1>=0,urgent);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{0.4375_dy<=x<=0.5625_dy,0.9375_dy<=y<=1.0625_dy});
    HybridTime time(2.0_x,3);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    //ARIADNE_TEST_CHECK(orbit.final().size(),2u);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-impact"),Axes2d(-3.0_x,x,+2.0_x, -4.0_x,y,+2.0_x),
         //guard_set_colour,ExactBoxType(2,1.0,8.0,-8.0,+8.0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}

Void TestHybridEvolver::test_tangency() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=zero});
    automaton.new_transition(q,e,q,{next(x)=x,next(y)=y-1},y-sqr(x)>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-1-eighth<=x<=-1+eighth,-quarter<=y<=quarter});
    HybridTime time(2.0_x,3);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),2u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),3u);
    ARIADNE_TEST_PRINT(orbit);

    Axes2d axes(-2.0_x,x,+2.0_x, -2.0_x,y,+1.0_x);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-tangency"),axes,
         //guard_set_colour,BoundedConstraintSet(bounding_box,y-x*x>=0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}

Void TestHybridEvolver::test_simultaneous_events() const {
    HybridAutomaton automaton;
    DiscreteEvent e1("e1");
    DiscreteEvent e2("e2");
    automaton.new_mode(q,{dot(x)=one,dot(y)=one});
    automaton.new_transition(q,e1,q,{next(x)=x-1,next(y)=y-2},x-1>=0,EventKind::URGENT);
    automaton.new_transition(q,e2,q,{next(x)=x-2,next(y)=y-1},y-1>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-quarter<=x<=eighth,-eighth<=y<=quarter});
    HybridTime time(2.5_x,4);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(4.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),2u);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-simultaneous_events"),Axes2d(-3.0_x,x,+2.0_x, -3.0_x,y,+2.0_x),
         //guard_set_colour,ExactBoxType(2,1.0,8.0,-8.0,+8.0),
         //guard_set_colour,ExactBoxType(2,-8.0,+8.0,1.0,8.0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}


Void TestHybridEvolver::test_creep() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one});
    automaton.new_transition(q,e,q,{next(x)=x-1,next(y)=y},x-1>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-quarter<=x<=eighth,-eighth<=y<=quarter});
    HybridTime time(1.5_x,4);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(1.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_PRINT(*orbit.reach().begin());
    ARIADNE_TEST_PRINT(HybridEnclosure(*orbit.final().begin()).bounding_box());
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),3u);
    ARIADNE_TEST_BINARY_PREDICATE(inside,HybridEnclosure(*orbit.final().begin()),HybridRealBox(q,{0.24_dy<=x<=0.635_dy,1.365_dy<=y<=1.76_dy}));

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-creep"),Axes2d(-1.5_x,x,+1.5_x, -0.5_x,y,+3.5_x),
         //guard_set_colour,ExactBoxType(2,1.0,8.0,-8.0,+8.0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}

Void TestHybridEvolver::test_unwind() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one});
    //automaton.new_transition(q,e,q,{x-3,y-1},x-1>=0,urgent);
    automaton.new_transition(q,e,q,{next(x)=x-3,next(y)=y-1},x-y/16-1>=0,EventKind::URGENT);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-quarter<=x<=eighth,-eighth<=y<=quarter});
    HybridTime time(3.0_x,4);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);

    Axes2d axes(-2.5_x,x,+1.5_x, -0.5_x,y,+2.5_x);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-unwind"),axes,
         //guard_set_colour,BoundedConstraintSet(bounding_box,x-y/16-1>=0),
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}

Void TestHybridEvolver::test_permissive() const {
    HybridAutomaton automaton;
    automaton.new_mode(q,{dot(x)=one,dot(y)=one/2});
    DiscreteEvent i("i");
    // Invariant has form f(x)<=0
    const Dyadic eps(0.125_x);
    automaton.new_invariant(q,x-2+eps<=0,i);
    automaton.new_transition(q,e,q,{next(x)=x-3,next(y)=y},x-2-eps>=0,EventKind::PERMISSIVE);
    ARIADNE_TEST_PRINT(automaton);

    RealSpace space=automaton.continuous_state_space(q);
    HybridBoxSet initial(q,{-eighth<=x<=eighth,-eighth<=y<=eighth});
    HybridTime time(3.0_x,4);

    _set_evolver(automaton);
    evolver_ptr->configuration().set_maximum_step_size(1.0_x);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial,time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);

    HybridRealBox guard_set(q,{2-eighth<=x<=2+eighth,-half<=y<=+2+half});
    Axes2d axes(-4.5_x,x,+2.5_x, -0.5_x,y,+2.5_x);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-permissive"),axes,
         guard_set_colour,guard_set,
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}







Void
TestHybridEvolver::test_affine_flow() const
{
    HybridAutomaton system;
    system.new_mode(q,{dot(x)=1-x-2*y,dot(y)=+2*x-y});

    _set_evolver(system);
    evolver_ptr->configuration().set_maximum_step_size(0.5_x);

    const Real r(0.125_x);
    HybridSet initial_set(q,{2-r<=x<=2+r,-r<=y<=r});
    HybridEnclosure initial_enclosure=this->evolver_ptr->enclosure(initial_set);

    HybridTime evolution_time=HybridTime(1.5_x,2);
    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial_enclosure,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_EQUAL(orbit.final().size(),1);
    ARIADNE_TEST_COMPARE(orbit.reach().size(),>=,3);
    if(orbit.reach().size()>12) { ARIADNE_TEST_WARN("Continuous evolution may use very short step size."); }
    else if(orbit.reach().size()!=3u) { ARIADNE_TEST_WARN("Continuous evolution may use shorter step size than necessary."); }

    Axes2d axes(-1.0_x,x,3.0_x, -2.0_x,y,2.0_x);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-affine_flow"), axes,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());
}


Void
TestHybridEvolver::test_splitting_on_urgent_event() const
{
    StringVariable loc("");
    DiscreteLocation upwards(loc|"upwards");
    DiscreteLocation downwards(loc|"downwards");
    DiscreteEvent changeup("changeup");
    DiscreteEvent changedown("changedown");
    DiscreteEvent block("block");

    HybridAutomaton system;
    system.new_mode(upwards,{dot(x)=1,dot(y)=1});
    system.new_mode(downwards,{dot(x)=1,dot(y)=-1});

    system.new_guard(upwards,changedown,y>=2,EventKind::URGENT);
    system.new_transition(upwards,changedown,downwards,{next(x)=x+1,next(y)=y});

    ARIADNE_TEST_PRINT(system);

    _set_evolver(system);
    evolver_ptr->configuration().set_maximum_step_size(2.0_x);

    DiscreteLocation initial_location(upwards);
    const Real r(0.125_x);
    RealVariablesBox initial_box({-r<=x<=+r, -r<=y<=+r});
    HybridEnclosure initial_enclosure=this->evolver_ptr->enclosure(HybridSet(initial_location,initial_box));
    HybridTime evolution_time=HybridTime(4.0_x,2);
    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial_enclosure,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_CHECK_WARN(orbit.final().size(),1u);
    ARIADNE_TEST_CHECK_WARN(orbit.reach().size(),4u);

    Axes2d axes(-1.0_x,x,11.0_x, -6.0_x,y,6.0_x);
    HybridSet guard_set(upwards,{-1<=x<=11,2<=y<=6});
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-splitting_on_urgent_event"), axes, guard_set_colour,guard_set,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());
}


Void
TestHybridEvolver::test_affine_hysteresis() const
{
    // A hybrid automaton with a single component and two locations,
    // with an affine flow in each, and affine guards and resets.
    // This should be very easy to analyse numerically, and is there to test
    // switching logic
    FloatDP::set_output_places(2);

    StringVariable loc("");
    DiscreteLocation upwards(loc|"upwards");
    DiscreteLocation downwards(loc|"downwards");
    DiscreteEvent changeup("changeup");
    DiscreteEvent changedown("changedown");
    DiscreteEvent block("block");

    HybridAutomaton system;
    system.new_mode(upwards,{dot(x)=1,dot(y)=1});
    system.new_mode(downwards,{dot(x)=1,dot(y)=-1});

    Real changedown_theshold=Dyadic(2);
    Real changeup_lower=Dyadic(-2);
    Real changeup_upper=Dyadic(-1.5_x);

    system.new_guard(upwards,changedown,y>=changedown_theshold,EventKind::URGENT);
    system.new_invariant(downwards,y>=changeup_lower,block);
    system.new_guard(downwards,changeup,y<=changeup_upper,EventKind::PERMISSIVE);
    system.new_transition(upwards,changedown,downwards,{next(x)=x+1,next(y)=y});
    system.new_transition(downwards,changeup,upwards,{next(x)=x+1,next(y)=y});

    ARIADNE_TEST_PRINT(system);

    _set_evolver(system);
    evolver_ptr->configuration().set_maximum_step_size(8.0_x);

    DiscreteLocation initial_location(upwards);
//    DiscreteLocation initial_location(downwards);
    Dyadic r(0.125_x);
    RealVariablesBox initial_box({-r<=x<=+r, -r<=y<=+r});
    HybridEnclosure initial_enclosure=this->evolver_ptr->enclosure(HybridSet(initial_location,initial_box));
    HybridTime evolution_time(0.0_x,0u);

    Axes2d axes(-1.0_x,x,11.0_x, -3.0_x,y,3.0_x);
    HybridSet guard_set_changeup(downwards,{-1<=x<=11,-2<=y<=-Dyadic(1.5_x)});
    Orbit<HybridEnclosure> orbit;

    evolution_time=HybridTime(1.0_x,3);
    ARIADNE_TEST_PRINT(evolution_time);
    orbit=evolver_ptr->orbit(initial_enclosure,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_EQUAL(orbit.final().size(),1u);
    ARIADNE_TEST_EQUAL(orbit.reach().size(),1u);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-affine_hysteresis-t=1"), axes, guard_set_colour,guard_set_changeup,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());

    evolution_time=HybridTime(2.0_x,3);
    ARIADNE_TEST_PRINT(evolution_time);
    orbit=evolver_ptr->orbit(initial_enclosure,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_EQUAL(orbit.reach().size(),2u);
    ARIADNE_TEST_EQUAL(orbit.final().size(),2u);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-affine_hysteresis-t=2"), axes, guard_set_colour,guard_set_changeup,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());

    evolution_time=HybridTime(4.0_x,3);
    ARIADNE_TEST_PRINT(evolution_time);
    orbit=evolver_ptr->orbit(initial_enclosure,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_EQUAL(orbit.reach().size(),2u);
    ARIADNE_TEST_EQUAL(orbit.final().size(),1u);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-affine_hysteresis-t=4"), axes, guard_set_colour,guard_set_changeup,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());

    evolution_time=HybridTime(6.0_x,3);
    ARIADNE_TEST_PRINT(evolution_time);
    orbit=evolver_ptr->orbit(initial_enclosure,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_EQUAL(orbit.reach().size(),3u);
    ARIADNE_TEST_EQUAL(orbit.final().size(),2u);

    plot(c_str("test_hybrid_evolver-"+evolver_name+"-affine_hysteresis-t=6"), axes, guard_set_colour,guard_set_changeup,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());

    evolution_time=HybridTime(8.0_x,3);
    ARIADNE_TEST_PRINT(evolution_time);
    orbit=evolver_ptr->orbit(initial_enclosure,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);
    ARIADNE_TEST_EQUAL(orbit.reach().size(),3u);
    ARIADNE_TEST_EQUAL(orbit.final().size(),1u);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-affine_hysteresis-t=8"), axes, guard_set_colour,guard_set_changeup,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());
}




Void TestHybridEvolver::test_constant_derivative_system() const
{
    // Test the system (d(x),d(y))=(1,0) with reset (x',y')=(x-2,y) when x+y>0
    // Starting in a small box near the origin, the system should return to
    // the initial condition after time 2
    DiscreteLocation q1(1), q2(2);

    HybridAutomaton system;
    system.new_mode(q1,{dot(x)=1,dot(y)=0});
    system.new_mode(q2,{dot(x)=1,dot(y)=0});
    system.new_transition(q1,e,q2,{prime(x)=x-2,prime(y)=y},x+y>=0,EventKind::URGENT);

    Real r(0.0625_x);
    RealVariablesBox initial_box({-r<=x<=+r,-r<=y<=+r});
    HybridSet initial_set(q1,initial_box);

    _set_evolver(system);

    ARIADNE_TEST_PRINT(system);
    ARIADNE_TEST_PRINT(initial_set);

    // Test continuous evolution with a single transverse jump
    HybridTime evolution_time(2.0_x,2);
    ARIADNE_TEST_PRINT(evolution_time);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial_set,evolution_time,Semantics::UPPER);
    ARIADNE_TEST_PRINT(orbit);

    ListSet<HybridEnclosure> final_set=orbit.final();
    ARIADNE_TEST_PRINT(final_set);
    HybridEnclosure expected_final_set=evolver_ptr->enclosure(HybridSet(q2,initial_box));
    ARIADNE_TEST_PRINT(expected_final_set);

    ARIADNE_TEST_COMPARE(norm(final_set[0].state_function()-expected_final_set.state_function()),<,1e-14_pr);

}



Void TestHybridEvolver::test_transverse_linear_crossing() const
{
    DiscreteLocation q1(1), q2(2);
    Real r(0.125_x);
    ExactDouble tol=1e-5_pr;
    RealExpression guard=x+y/2-1;

    HybridAutomaton system;
    system.new_mode(q1,{dot(x)=1,dot(y)=0});
    system.new_mode(q2,{dot(x)=0,dot(y)=1});
    system.new_transition(q1,e,q2,{prime(x)=x,prime(y)=y},guard>=0,EventKind::URGENT);
    HybridSet initial_set(q1,{-r<=x<=r,-r<=y<=r});
    HybridTime evolution_time(2.0_x,3);

    _set_evolver(system);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial_set,evolution_time,Semantics::UPPER);
    Enclosure final_enclosure=orbit.final()[0].continuous_set();

    RealSpace final_space=system.continuous_state_space(q2);
    RealExpression ct=-guard; // Crossing time
    //EffectiveVectorMultivariateFunction function=make_function((x+ct,y+2-ct),final_space);
    EffectiveVectorMultivariateFunction function=join(make_function(final_space,x+ct),make_function(final_space,y+2-ct));
    LabelledEnclosure expected_final_enclosure(initial_set.euclidean_set(q1,final_space),system.continuous_state_space(q1),EnclosureConfiguration(evolver_ptr->function_factory()));
    expected_final_enclosure.apply_map(function);

    Vector<FloatDPBounds> tolerance(2,FloatDPBounds(-tol,+tol,dp));
    ARIADNE_TEST_BINARY_PREDICATE(refines,expected_final_enclosure.state_function()[0],final_enclosure.state_function()[0]);
    ARIADNE_TEST_BINARY_PREDICATE(refines,final_enclosure.state_function()[0],expected_final_enclosure.state_function()[0]+tolerance[0]);

    Real xl(-1.0_x), xu(3.0_x), yl(-1.0_x), yu(3.0_x);
    Axes2d axes(xl,x,xu, yl,y,yu);
    HybridSet guard_set(q1,{xl<=x<=xu,yl<=y<=yu},{guard>=0});
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-transverse_linear_crossing"), axes, guard_set_colour,guard_set,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());
}

Void TestHybridEvolver::test_transverse_cubic_crossing() const
{
    DiscreteLocation q1(1), q2(2);
    Real r(1.0_x/pow(two,3));
    RealExpression guard=x-(1+y/2+y*y*y);
    HybridAutomaton system;
    system.new_mode(q1,{dot(x)=1,dot(y)=0});
    system.new_mode(q2,{dot(x)=0,dot(y)=1});
    system.new_transition(q1,e,q2,{prime(x)=x,prime(y)=y},guard>=0,EventKind::URGENT);
    HybridSet initial_set(q1,{-r<=x<=r,-r<=y<=r});
    HybridTime evolution_time(2.0_x,3);

    _set_evolver(system);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial_set,evolution_time,Semantics::UPPER);

    Real xl(-1.0_x), xu(3.0_x), yl(-1.0_x), yu(3.0_x);
    Axes2d axes(xl,x,xu, yl,y,yu);
    HybridSet guard_set(q1,{xl<=x<=xu,yl<=y<=yu},{guard>=0});
    HybridEnclosure guard_enclosure=evolver_ptr->enclosure(guard_set);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-transverse_cubic_crossing"), axes, guard_set_colour,guard_enclosure,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());
}

Void TestHybridEvolver::test_transverse_cube_root_crossing() const
{
    DiscreteLocation q1(1), q2(2);
    Real r=Real(1.0_x/pow(two,5));
    Real eps(1.0_x/pow(two,5));
    Real a(1);
    RealExpression guard=((x-a)*(x-a)+1)*(x-a)-y-eps;
    HybridAutomaton system;
    system.new_mode(q1,{dot(x)=1,dot(y)=0});
    system.new_mode(q2,{dot(x)=0,dot(y)=1});
    system.new_transition(q1,e,q2,{prime(x)=x,prime(y)=y},guard>=0,EventKind::URGENT);
    HybridSet initial_set(q1,{-r<=x<=r,-r<=y<=r});
    HybridTime evolution_time(2.0_x,3);

    _set_evolver(system);

    Orbit<HybridEnclosure> orbit=evolver_ptr->orbit(initial_set,evolution_time,Semantics::UPPER);

    Real xl(-1.0_x), xu(3.0_x), yl(-1.0_x), yu(3.0_x);
    //Axes2d axes={xl<=x<=xu, yl<=y<=yu};
    Axes2d axes(xl,x,xu, yl,y,yu);
    HybridSet guard_set(q1,{xl<=x<=xu,yl<=y<=yu},{guard>=0});
    HybridEnclosure guard_enclosure=evolver_ptr->enclosure(guard_set);
    plot(c_str("test_hybrid_evolver-"+evolver_name+"-transverse_cube_root_crossing"), axes, guard_set_colour,guard_enclosure,
         reach_set_colour,orbit.reach(), intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(), initial_set_colour,orbit.initial());

}




Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    GradedTaylorSeriesIntegrator evolver_integrator(1e-3);
    ARIADNE_TEST_CALL(TestHybridEvolver("general",evolver_integrator).test_all());

    std::cerr<<"INCOMPLETE ";
    return ARIADNE_TEST_FAILURES;
}

