/***************************************************************************
 *            test_verify.cpp
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

class TestVerifySafety
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
    SymbolicSetType symbolic_initial_set;
    SetType initial_set;
    SymbolicSetType symbolic_safe_set;
    SetType safe_set;
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

    TestVerifySafety()
        : system(build_system()),
          analyser(build_analyser(system)),
          grid(2),
          x("x"),
          y("y"),
          graphics_box({{-3,3},{-3,3}}),
          symbolic_initial_set({1.98_dec<=x<=1.99_dec,0.01_dec<=y<=0.02_dec}),
          initial_set(symbolic_initial_set.euclidean_set(system.state_space())),
          symbolic_safe_set({-2_dec<=x<=3_dec,-2_dec<=y<=3_dec},{sqr(x)+sqr(y)<=sqr((Real)3)}),
          safe_set(symbolic_safe_set.euclidean_set(system.state_space()))
    {
        cout << "Done creating initial and safe sets\n" << endl;

        cout << "system=" << system << endl;
        cout << "initial_set=" << initial_set << endl;
        cout << "safe_set=" << safe_set << endl;
    }

    Void test_verify_safety_no_bounding() {
        analyser.configuration().set_bounding_domain_ptr(nullptr);
        auto safety_certificate=analyser.verify_safety(initial_set,safe_set);
        ARIADNE_TEST_ASSERT(definitely(safety_certificate.is_safe));

        auto safe_cells=inner_approximation(safe_set, grid, analyser.configuration().maximum_grid_fineness());
        plot("test_verify_safety-safe.png",Projection2d(2,0,1),graphics_box,
             safe_set_colour,safety_certificate.safe_set,
             reach_set_colour,safety_certificate.chain_reach_set,
             initial_set_colour,initial_set);
    }

    Void test_verify_safety_crossing_bounding() {
        analyser.configuration().set_bounding_domain(BoundingDomainType(analyser.system().dimension(),{-1,+1}));
        ARIADNE_TEST_FAIL(analyser.verify_safety(initial_set,safe_set));
    }

    Void test_verify_safety_crossing_safe() {
        analyser.configuration().set_bounding_domain_ptr(nullptr);
        SymbolicSetType smaller_symbolic_safe_set({-2_dec<=x<=2.5_dec,-2_dec<=y<=2.5_dec},{sqr(x)+sqr(y)<=sqr(2.5_dec)});
        auto smaller_safe_set = smaller_symbolic_safe_set.euclidean_set(system.state_space());

        auto safety_certificate=analyser.verify_safety(initial_set,smaller_safe_set);
        ARIADNE_TEST_ASSERT(is_indeterminate(safety_certificate.is_safe));

        auto safe_cells=inner_approximation(safe_set, grid, analyser.configuration().maximum_grid_fineness());
        plot("test_verify_safety-possibly-unsafe.png",Projection2d(2,0,1),graphics_box,
             safe_set_colour,safety_certificate.safe_set,
             reach_set_colour,safety_certificate.chain_reach_set,
             initial_set_colour,initial_set);
    }


    Void test() {
        ARIADNE_TEST_CALL(test_verify_safety_no_bounding());
        ARIADNE_TEST_CALL(test_verify_safety_crossing_bounding());
        ARIADNE_TEST_CALL(test_verify_safety_crossing_safe());
    }

};


Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    TestVerifySafety().test();
    return ARIADNE_TEST_FAILURES;
}

