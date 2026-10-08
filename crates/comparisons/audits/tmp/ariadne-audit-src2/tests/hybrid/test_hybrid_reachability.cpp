/***************************************************************************
 *            test_hybrid_reachability.cpp
 *
 *  Copyright  2006-20  Luca Geretti
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
#include "hybrid/hybrid_reachability_analyser.hpp"
#include "io/graphics_manager.hpp"
#include "io/drawer.hpp"
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
class TestHybridReachability
{
    mutable shared_ptr<HybridReachabilityAnalyser> analyser;
  private:
    Void _set_analyser(const HybridAutomatonInterface& system) const;
  public:
    Void test() const;
    Void test_bouncing_ball() const;
};

Void TestHybridReachability::_set_analyser(const HybridAutomatonInterface& system) const
{
    GeneralHybridEvolver evolver(system);
    evolver.set_integrator(TaylorPicardIntegrator(StepMaximumError(1e-5)));
    evolver.configuration().set_maximum_step_size(0.25);
    evolver.configuration().set_maximum_enclosure_radius(0.125);
    evolver.configuration().set_maximum_enclosure_radius(0.5);
    analyser.reset(new HybridReachabilityAnalyser(evolver));
    analyser->configuration().set_maximum_grid_fineness(4);
    analyser->configuration().set_lock_to_grid_time(5);
    ARIADNE_TEST_PRINT(analyser->configuration());
}

Void TestHybridReachability::test() const {
    ARIADNE_TEST_CALL(test_bouncing_ball());
}

Void TestHybridReachability::test_bouncing_ball() const {
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
    HybridBoundedConstraintSet initial(q,{height-radius<=x<=height+radius,-radius<=v<=radius});
    Decimal tmax(6.5);
    Natural maxsteps=5u;
    HybridTime time(tmax,maxsteps);

    this->_set_analyser(bouncing_ball);

    auto reach_evolve=analyser->upper_reach_evolve(initial,time);

    Dyadic xl(-0.5), xu(+2.5), vl(-4.0), vu(+4.0);
    Axes2d bounding_box={xl<=x<=xu,vl<=v<=vu};
    plot("test_hybrid_reachability-bouncing_ball",bounding_box,reach_evolve.first);
}


Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    GraphicsManager::instance().set_drawer(GridDrawer(2));

    TestHybridReachability().test();
    return ARIADNE_TEST_FAILURES;
}

