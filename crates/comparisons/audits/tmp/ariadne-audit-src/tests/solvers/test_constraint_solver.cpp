/***************************************************************************
 *            test_constraint_solver.cpp
 *
 *  Copyright  2010-20  Pieter Collins
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

#include "config.hpp"
#include "../test.hpp"

#include "numeric/numeric.hpp"
#include "foundations/logical.hpp"
#include "algebra/vector.hpp"
#include "function/function.hpp"
#include "function/constraint.hpp"
#include "function/procedure.hpp"
#include "solvers/constraint_solver.hpp"
#include "geometry/box.hpp"
#include "io/command_line_interface.hpp"

using namespace std;
using namespace Ariadne;

class TestConstraintSolver
{
  public:
    Void test() {
        ARIADNE_TEST_CALL(test_empty_reduce_inequality());
        ARIADNE_TEST_CALL(test_empty_reduce_equality());
        ARIADNE_TEST_CALL(test_empty_reduce_mixed());
        ARIADNE_TEST_CALL(test_empty_hull_reduce());
        ARIADNE_TEST_CALL(test_empty_box_reduce());
        ARIADNE_TEST_CALL(test_hull_reduce());
        ARIADNE_TEST_CALL(test_box_reduce());
        ARIADNE_TEST_CALL(test_monotone_reduce());
        ARIADNE_TEST_CALL(test_feasible());
        ARIADNE_TEST_CALL(test_split());
    }

    Void test_empty_reduce_inequality() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,1.0_x},{0.0_x,1.0_x}};
        List<EffectiveConstraint> c = {4<=2*x[0]+x[1]};

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.reduce(D,c));
        ARIADNE_TEST_PRINT(D);
        ARIADNE_TEST_ASSERT(D.is_empty());
    }

    Void test_empty_reduce_equality() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,1.0_x},{0.0_x,1.0_x}};
        List<EffectiveConstraint> c = {2*x[0]+x[1]==4};

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.reduce(D,c));
        ARIADNE_TEST_PRINT(D);
        ARIADNE_TEST_ASSERT(D.is_empty());
    }

    Void test_empty_reduce_mixed() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,0.25_x},{0.0_x, 2.0_x}};
        List<EffectiveConstraint> c = {x[1]<=1,x[0]+x[1]==2};

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.reduce(D,c));
        ARIADNE_TEST_PRINT(D);
        ARIADNE_TEST_ASSERT(D.is_empty());
    }

    Void test_empty_hull_reduce() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,0.25_x},{0.0_x,2.0_x}};
        List<EffectiveConstraint> c = {x[1]<=1, x[0]+x[1]==2};

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.hull_reduce(D,c[0]));
        ARIADNE_TEST_EXECUTE(propagator.hull_reduce(D,c[1]));
        ARIADNE_TEST_PRINT(D);
        ARIADNE_TEST_ASSERT(D.is_empty());
    }

    Void test_empty_box_reduce() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,0.25_x},{0.0_x, 2.0_x}};
        List<EffectiveConstraint> c = {x[1]<=1,x[0]+x[1]==2};

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c[0],0));
        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c[1],0));
        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c[0],1));
        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c[1],1));
        ARIADNE_TEST_EXECUTE(propagator.hull_reduce(D,c[0]));
        ARIADNE_TEST_EXECUTE(propagator.hull_reduce(D,c[1]));
        ARIADNE_TEST_PRINT(D);
        ARIADNE_TEST_ASSERT(D.is_empty());
    }

    Void test_hull_reduce() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,2.0_x},{0.0_x,2.0_x}};
        List<EffectiveConstraint> c = {-2<=2*x[0]+x[1]<=1};

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.hull_reduce(D,c[0]));
        ARIADNE_TEST_SAME(D,UpperBoxType({{0.0_x,0.5_x},{0.0_x,1.0_x}}));
    }

    Void test_box_reduce() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,2.0_x},{0.0_x,2.0_x}};
        EffectiveConstraint c = (-2<=2*x[0]+x[1]<=1);

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c,0));
        ARIADNE_TEST_SAME(D,UpperBoxType({{0.0_x,0.75_x},{0.0_x,2.0_x}}));
        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c,1));
        ARIADNE_TEST_SAME(D,UpperBoxType({{0.0_x,0.75_x},{0.0_x,1.25_x}}));
    }


    Void test_monotone_reduce() {
        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(2);
        UpperBoxType D = ExactBoxType{{0.0_x,2.0_x},{0.0_x,2.0_x}};
        EffectiveConstraint c = (-2<=2*x[0]+x[1]<=1);

        ConstraintSolver propagator;

        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c,0));
        ARIADNE_TEST_SAME(D,UpperBoxType({{0.0_x,0.75_x},{0.0_x,2.0_x}}));
        ARIADNE_TEST_EXECUTE(propagator.box_reduce(D,c,1));
        ARIADNE_TEST_SAME(D,UpperBoxType({{0.0_x,0.75_x},{0.0_x,1.25_x}}));
    }

    Void test_split() {
        ARIADNE_TEST_WARN("test_split: Not implemented");
    }

    Void test_feasible() {

        List<EffectiveScalarMultivariateFunction> x=EffectiveScalarMultivariateFunction::coordinates(1);
        EffectiveConstraint c = (x[0]-2<=0);

        List<ValidatedConstraint> constraints;
        constraints.append(c);

        ConstraintSolver contractor;

        ARIADNE_TEST_PRINT(constraints);

        ExactBoxType domain1({{1.9375_x,2.0_x}});
        ARIADNE_TEST_ASSERT(definitely(contractor.feasible(domain1,constraints).first));

        ExactBoxType domain2({{2.015625_x,2.5_x}});
        ARIADNE_TEST_ASSERT(!possibly(contractor.feasible(domain2,constraints).first));

        ExactBoxType domain3({{2.0_x,2.015625_x}});
        ARIADNE_TEST_ASSERT(is_indeterminate(contractor.feasible(domain3,constraints).first));
    }
};

Int main(Int argc, const char* argv[]) {
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;
    TestConstraintSolver().test();
    return ARIADNE_TEST_FAILURES;
}

