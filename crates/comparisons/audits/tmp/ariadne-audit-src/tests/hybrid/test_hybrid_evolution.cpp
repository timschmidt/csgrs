/***************************************************************************
 *            test_hybrid_evolution.cpp
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
#include "solvers/integrator.hpp"
#include "dynamics/orbit.hpp"
#include "hybrid/hybrid_automata.hpp"
#include "hybrid/hybrid_time.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "hybrid/hybrid_evolver.hpp"
#include "io/graphics_interface.hpp"
#include "io/figure.hpp"
#include "hybrid/hybrid_graphics.hpp"
#include "io/command_line_interface.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;


Colour reach_set_colour(0.25,0.25,0.50);
Colour intermediate_set_colour(0.50,0.50,0.75);
Colour final_set_colour(0.75,0.75,1.00);
Colour initial_set_colour(0.75,0.75,1.00);
Colour guard_set_colour(0.75,0.75,0.75);

// Test evolution of realistic hybrid systems
class TestHybridEvolution
{
    mutable shared_ptr<HybridEvolverBase> evolver;
  private:
    Void _set_evolver(const HybridAutomatonInterface& system) const;
  public:
    Void test() const;
    Void test_bouncing_ball() const;
    Void test_water_tank() const;
};

Void TestHybridEvolution::_set_evolver(const HybridAutomatonInterface& system) const
{
    evolver.reset(new GeneralHybridEvolver(system));
    evolver->set_integrator(TaylorPicardIntegrator(StepMaximumError(1e-5)));
    evolver->configuration().set_maximum_step_size(0.25);
    evolver->configuration().set_maximum_enclosure_radius(0.125);
    evolver->configuration().set_maximum_enclosure_radius(0.5);
}

Void TestHybridEvolution::test() const {
    ARIADNE_TEST_CALL(test_bouncing_ball());
    ARIADNE_TEST_CALL(test_water_tank());
}

Void TestHybridEvolution::test_bouncing_ball() const {
    HybridAutomaton bouncing_ball;
    Real one(1);
    DiscreteLocation q;
    DiscreteEvent e("e");
    RealVariable x("x");
    RealVariable v("v");
    TimeVariable t;

    Real lambda(0.5_x);
    bouncing_ball.new_mode(q,{dot(x)=v,dot(v)=-one});
    bouncing_ball.new_transition(q,e,q,{next(x)=x,next(v)=-lambda*v},x<=0,EventKind::IMPACT);
    ARIADNE_TEST_PRINT(bouncing_ball);

    Dyadic height=2;
    Dyadic radius=1/pow(two,6);
    HybridExactBox initial(q,bouncing_ball.continuous_state_space(q),ExactBoxType{{height-radius,height+radius},{-radius,+radius}});
    Decimal tmax(4.5);
    Natural maxsteps=5u;
    HybridTime time(tmax,maxsteps);

    this->_set_evolver(bouncing_ball);

    Orbit<HybridEnclosure> orbit=evolver->orbit(initial,time,Semantics::UPPER);
    ListSet<HybridEnclosure> const& orbit_final=orbit.final();

    //ARIADNE_TEST_PRINT(orbit);
    if(orbit_final.size()!=1u) {
        ARIADNE_TEST_WARN("orbit.final().size()="<<orbit_final.size()<<"; expected 1. "
                          "This may indicate over-zealous splitting, and/or errors in detecting the end conditions.");
    }

    FloatDP exl(0.12_pr,dp), exu(+0.13_pr,dp), evl(-0.04_pr,dp), evu(+0.04_pr,dp); // Expected bounds
    HybridExactBox expected_orbit_final_bounding_box=HybridExactBox(q,{x.in(exl,exu),v.in(evl,evu)});
    ARIADNE_TEST_BINARY_PREDICATE(inside,orbit_final,expected_orbit_final_bounding_box);

    Dyadic xl(-0.5), xu(+2.5), vl(-4.0), vu(+4.0);
    Axes2d bounding_box={xl<=x<=xu,vl<=v<=vu};
    plot("test_hybrid_evolution-bouncing_ball",bounding_box,
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
    Axes2d time_bounding_box={0<=t<=tmax,xl<=x<=xu};
    plot("test_hybrid_evolution-bouncing_ball-time",time_bounding_box,
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
    Axes2d velocity_bounding_box={0<=t<=tmax,vl<=v<=vu};
    plot("test_hybrid_evolution-bouncing_ball-velocity",velocity_bounding_box,
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());
}

Void TestHybridEvolution::test_water_tank() const {
    // Declare some constants
    Real T(Rational(4));
    Real hmin(Rational(11,2));
    Real hmax(Rational(8));
    Real delta(Rational(1,20));
    Real lambda(Rational(1,50));
    Real rate(Rational(3,10));

    RealVariable height("height");
    RealVariable aperture("aperture");
    Real zero(0);
    Real one(1);

    StringVariable valve("valve");
    DiscreteLocation open(valve|"open");
    DiscreteLocation opening(valve|"opening");
    DiscreteLocation closed(valve|"closed");
    DiscreteLocation closing(valve|"closing");

    DiscreteEvent start_opening("start_opening");
    DiscreteEvent start_closing("start_closing");
    DiscreteEvent finished_opening("finished_opening");
    DiscreteEvent finished_closing("finished_closing");

    // Create the tank object
    HybridAutomaton watertank;

    watertank.new_mode(open,dot({height,aperture})={-lambda*height+rate*aperture,zero});
    watertank.new_mode(closed,dot({height,aperture})={-lambda*height+rate*aperture,zero});
    watertank.new_mode(opening,dot({height,aperture})={-lambda*height+rate*aperture,1/T});
    watertank.new_mode(closing,dot({height,aperture})={-lambda*height+rate*aperture,-1/T});

    watertank.new_transition(closed,start_opening,opening,next({height,aperture})={height,aperture},height<=hmin,EventKind::URGENT);
    watertank.new_transition(open,start_closing,closing,next({height,aperture})={height,aperture},height>=hmax,EventKind::URGENT);
    watertank.new_transition(opening,finished_opening,open,next({height,aperture})={height,aperture},aperture>=1,EventKind::URGENT);
    watertank.new_transition(closing,finished_closing,closed,next({height,aperture})={height,aperture},aperture<=0,EventKind::URGENT);

    ARIADNE_TEST_PRINT(watertank);

    DiscreteLocation initial_location=opening;
    HybridRealBox initial_box(initial_location,{0<=height<=one/16,0<=aperture<=one/64});
    HybridSet initial(initial_location,{0<=height<=one/16,0<=aperture<=one/64});

    //HybridTime evolution_time(80.0,5);
    HybridTime evolution_time(80.0_x,8);

    _set_evolver(watertank);

    evolver->configuration().set_maximum_step_size(1.0);
    Orbit<HybridEnclosure> orbit = evolver->orbit(initial,evolution_time,Semantics::UPPER);
    if(orbit.final().size()!=1u) {
        ARIADNE_TEST_WARN("orbit.final().size()="<<orbit.final().size()<<"; expected 1. "
                          "This may indicate over-zealous splitting, and/or errors in detecting the end conditions.");
    }
    HybridEnclosure final_enclosure=HybridEnclosure(orbit.final()[0]);
    ARIADNE_TEST_PRINT(final_enclosure.bounding_box());
    Dyadic ehl(7.6875), ehu(8.0); Decimal eal(0.999), eau(1.001); // Expected bounds
    ARIADNE_TEST_BINARY_PREDICATE(inside,final_enclosure,HybridRealBox(open,{height.in(ehl,ehu),aperture.in((Rational)eal,(Rational)eau)}));

    Decimal hl(-0.1), hu(+9.1), al(-0.3), au(+1.3);
    Axes2d bounding_box={hl<=height<=hu, al<=aperture<=au};
    plot("test_hybrid_evolution-water_tank",bounding_box,
         reach_set_colour,orbit.reach(),
         intermediate_set_colour,orbit.intermediate(),
         final_set_colour,orbit.final(),
         initial_set_colour,orbit.initial());

}


Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    TestHybridEvolution().test();
    return ARIADNE_TEST_FAILURES;
}

