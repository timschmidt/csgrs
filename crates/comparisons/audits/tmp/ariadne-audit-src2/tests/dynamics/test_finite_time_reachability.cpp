/***************************************************************************
 *            test_finite_time_reachability.cpp
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

#include <iostream>
#include <fstream>
#include <string>

#include "config.hpp"
#include "function/taylor_function.hpp"
#include "function/formula.hpp"
#include "algebra/algebra.hpp"
#include "geometry/function_set.hpp"
#include "geometry/grid_paving.hpp"
#include "dynamics/enclosure.hpp"
#include "dynamics/orbit.hpp"
#include "dynamics/vector_field_evolver.hpp"
#include "dynamics/reachability_analyser.hpp"
#include "symbolic/expression_set.hpp"
#include "solvers/integrator.hpp"
#include "io/figure.hpp"
#include "io/command_line_interface.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;

Colour reach_set_colour(0.25,0.25,0.50);
Colour intermediate_set_colour(0.50,0.50,0.75);
Colour final_set_colour(0.75,0.75,1.00);
Colour evolve_set_colour(0.75,0.75,1.00);
Colour initial_set_colour(0.75,0.75,1.00);
Colour guard_set_colour(0.75,0.75,0.75);
Colour bounding_box_colour(0.75,0.75,0.875);
Colour safe_set_colour(0.50,1.00,0.50);

class TestFiniteTimeReachability
{

    typedef VectorFieldEvolver EvolverType;
    typedef EvolverType::SystemType SystemType;
    typedef SystemType::TimeType TimeType;
    typedef SystemType::StateSpaceType StateSpaceType;
    typedef EvolverType::EnclosureType EnclosureType;
    typedef RealExpressionBoundedConstraintSet SymbolicSetType;
    typedef BoundedConstraintSet SetType;
    typedef ReachabilityAnalyser<SystemType> AnalyserType;
    typedef AnalyserType::BoundingDomainType BoundingDomainType;
    typedef AnalyserType::StorageType StorageType;

    SystemType system;
    AnalyserType analyser;
    Grid grid;
    RealVariable x;
    RealVariable y;
    ExactBoxType graphics_box;
    BoundingDomainType bounding;
    SymbolicSetType symbolic_initial_set;
    SetType initial_set;
    SymbolicSetType symbolic_safe_set;
    SetType safe_set;
    TimeType reach_time;
  public:

    static SystemType build_system()
    {
        std::cout<<std::setprecision(20);
        std::cerr<<std::setprecision(20);
        std::clog<<std::setprecision(20);
        RealVariable x("x");
        RealVariable y("y");
        Real a(-0.5_x); Real b(1.0_x);
        SystemType sys({dot(x)=-a*x-b*y,dot(y)=b*x+2*a*y});
        cout << "Done building system\n";
        return sys;
    }

    static AnalyserType build_analyser(const SystemType& system)
    {
        GradedTaylorSeriesIntegrator integrator(StepMaximumError(1e-2_pr));

        EvolverType evolver(system,integrator);

        AnalyserType analyser(evolver);
        analyser.configuration().set_maximum_grid_fineness(3);
        cout << "Done building analyser\n";
        return analyser;
    }

    TestFiniteTimeReachability()
        : system(build_system()),
          analyser(build_analyser(system)),
          grid(2),
          x("x"),
          y("y"),
          graphics_box({{-3,3},{-3,3}}),
          bounding({{-4,4},{-4,4}}),
          symbolic_initial_set({1.98_dec<=x<=1.99_dec,0.01_dec<=y<=0.02_dec}),
          initial_set(symbolic_initial_set.euclidean_set(system.state_space())),
          symbolic_safe_set({-2_dec<=x<=3_dec,-2_dec<=y<=3_dec},{sqr(x)+sqr(y)<=sqr((Real)3)}),
          safe_set(symbolic_safe_set.euclidean_set(system.state_space())),
          reach_time(3.0_x)
    {
        cout << "Done creating initial and safe sets\n" << endl;

        cout << "system=" << system << endl;
        cout << "initial_set=" << initial_set << endl;
        cout << "safe_set=" << safe_set << endl;
    }

    Void test_lower_reach_lower_evolve() {
        cout << "Computing timed reachable set" << endl;
        auto lower_reach=analyser.lower_reach(initial_set,reach_time);
        ARIADNE_TEST_ASSERT(lower_reach.size() > 0);

        cout << "Computing timed evolve set" << endl;
        auto lower_evolve=analyser.lower_evolve(initial_set,reach_time);
        ARIADNE_TEST_ASSERT(lower_evolve.size() > 0);

        cout << "Reached " << lower_reach.size() << " cells " << endl;
        cout << "Evolved to " << lower_evolve.size() << " cells " << endl << endl;

        plot("test_reachability_analyser-lower_reach_lower_evolve.png",Projection2d(2,0,1),graphics_box,
             reach_set_colour,lower_reach,evolve_set_colour,lower_evolve);
    }

    Void test_lower_reach_evolve() {
        cout << "Computing timed reach-evolve set" << endl;
        Pair<StorageType,StorageType> reach_evolve_set = analyser.lower_reach_evolve(initial_set,reach_time);
        StorageType& lower_reach=reach_evolve_set.first;
        StorageType& lower_evolve=reach_evolve_set.second;
        cout << "Reached " << lower_reach.size() << " cells " << endl;
        cout << "Evolved to " << lower_evolve.size() << " cells " << endl << endl;

        ARIADNE_TEST_ASSERT(lower_reach.size() > 0);
        ARIADNE_TEST_ASSERT(lower_evolve.size() > 0);

        plot("test_reachability_analyser-map_lower_reach_evolve.png",Projection2d(2,0,1),graphics_box,
             reach_set_colour,lower_reach,evolve_set_colour,lower_evolve);
    }

    Void test_upper_reach_upper_evolve() {
        cout << "Computing timed reachable set" << endl;
        StorageType upper_reach_set=analyser.upper_reach(initial_set,reach_time);
        ARIADNE_TEST_ASSERT(upper_reach_set.size() > 0);

        cout << "Computing timed evolve set" << endl;
        StorageType upper_evolve_set=analyser.upper_evolve(initial_set,reach_time);
        ARIADNE_TEST_ASSERT(upper_evolve_set.size() > 0);

        cout << "Reached " << upper_reach_set.size() << " cells " << endl;
        cout << "Evolved to " << upper_evolve_set.size() << " cells " << endl << endl;

        plot("test_reachability_analyser-map_upper_reach_upper_evolve.png",Projection2d(2,0,1),graphics_box,
             reach_set_colour,upper_reach_set,evolve_set_colour,upper_evolve_set);
    }

    Void test_upper_reach_evolve() {
        cout << "Computing timed reach-evolve set" << endl;
        Pair<StorageType,StorageType> reach_evolve_set = analyser.upper_reach_evolve(initial_set,reach_time);

        ARIADNE_TEST_ASSERT(reach_evolve_set.first.size() > 0);
        ARIADNE_TEST_ASSERT(reach_evolve_set.second.size() > 0);

        cout << "Reached " << reach_evolve_set.first.size() << " cells " << endl;
        cout << "Evolved to " << reach_evolve_set.second.size() << " cells " << endl << endl;

        plot("test_reachability_analyser-map_upper_reach_evolve.png",Projection2d(2,0,1),graphics_box,
             reach_set_colour,reach_evolve_set.first,final_set_colour,reach_evolve_set.second);
    }

    Void test() {
        ARIADNE_TEST_CALL(test_lower_reach_lower_evolve());
        ARIADNE_TEST_CALL(test_lower_reach_evolve());
        ARIADNE_TEST_CALL(test_upper_reach_upper_evolve());
        ARIADNE_TEST_CALL(test_upper_reach_evolve());
    }

};


Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    TestFiniteTimeReachability().test();
    return ARIADNE_TEST_FAILURES;
}

