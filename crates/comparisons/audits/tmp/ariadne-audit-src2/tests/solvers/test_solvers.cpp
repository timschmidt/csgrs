/***************************************************************************
 *            test_solvers.cpp
 *
 *  Copyright  2008-20  Pieter Collins
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
#include <sstream>
#include <string>

#include "config.hpp"

#include "solvers/solver.hpp"
#include "function/function.hpp"
#include "function/function_patch.hpp"
#include "function/taylor_function.hpp"
#include "algebra/vector.hpp"
#include "algebra/algebra.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/space.hpp"
#include "function/formula.hpp"
#include "io/command_line_interface.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

typedef Vector<DyadicInterval> DyadicIntervalVector;

class TestSolver
{
  private:
    std::unique_ptr<SolverInterface> solver;
    StringType solver_class_name;
  public:
    TestSolver(const SolverInterface& s,const char* n)
        : solver(s.clone()), solver_class_name(n) { }

    Int test() {
        ARIADNE_TEST_PRINT(*solver);
        ARIADNE_TEST_CALL(test_solve());
        ARIADNE_TEST_CALL(test_implicit());
        ARIADNE_TEST_CALL(test_scalar_implicit());
        return 0;
    }

    Void test_solve() {
        EffectiveScalarMultivariateFunction x=EffectiveScalarMultivariateFunction::coordinate(1,0);
        ExactBoxType d({{0.0_x,1.0_x}});
        EffectiveVectorMultivariateFunction f({(x*x+1)*x-1});
        FloatDPBoundsVector p=solver->solve(f,d);
        ARIADNE_TEST_BINARY_PREDICATE(contains,ExactIntervalType(0.6823_pr,0.6824_pr),p[0]);
    }

    Void test_implicit() {
        //TaylorModelAccuracy::set_default_sweep_threshold(1e-12);

        // FIXME: Should be able to use numbers yielding exact results for p,r
        EffectiveScalarMultivariateFunction aa=EffectiveScalarMultivariateFunction::coordinate(1,0);
        EffectiveScalarMultivariateFunction a=EffectiveScalarMultivariateFunction::coordinate(2,0);
        EffectiveScalarMultivariateFunction x=EffectiveScalarMultivariateFunction::coordinate(2,1);
        EffectiveScalarMultivariateFunction bb;
        ExactBoxType p,r;
        EffectiveVectorMultivariateFunction f;
        ValidatedVectorMultivariateFunctionPatch h;
        EffectiveVectorMultivariateFunction e;

        // Test solution of x-a=0. This should be very easy to solve.
        p=ExactBoxType({{-0.25_x,0.25_x}});
        r=ExactBoxType({{-2.0_x,2.0_x}});
        f=EffectiveVectorMultivariateFunction({x-a});
        ARIADNE_TEST_PRINT(f);
        h=solver->implicit(f,p,r);
        ARIADNE_TEST_PRINT(h);
        e=EffectiveVectorMultivariateFunction(1u,aa);
        ARIADNE_TEST_PRINT(e);
        ARIADNE_TEST_COMPARE(norm(h-e),<,1e-8_pr);

        // Test solution of 4x^2+x-4-a=0 on [0.875_x,1.125_x]. There is a unique solution with positive derivative.
        p=ExactBoxType({ExactIntervalType(0.875_x,1.125_x)});
        r=ExactBoxType({ExactIntervalType(0.25_x,1.25_x)});
        f=EffectiveVectorMultivariateFunction({(x*x+1)*x-a});
        ARIADNE_TEST_PRINT(f);
        h=solver->implicit(f,p,r);
        ARIADNE_TEST_PRINT(h);
        bb=EffectiveScalarMultivariateFunction(aa-DyadicInterval(p[0]).midpoint())/DyadicInterval(p[0]).radius();
        Decimal a0(0.682328_dec), a1(0.0521547_dec), a2(-0.0023232_dec), a3(0.000147778_dec);
        e=EffectiveVectorMultivariateFunction( { a0+bb*(a1+bb*(a2+bb*a3)) } );
        ARIADNE_TEST_PRINT(e);
        ARIADNE_TEST_COMPARE(norm(h-e),<,1e-4_pr);

        // Test solution of 4x^2+x-4-a=0 on [-0.25_x,0.25_x]. There is a unique solution with positive derivative.
        p=ExactBoxType({ExactIntervalType(-0.25_x,0.25_x)});
        r=ExactBoxType({ExactIntervalType(0.25_x,2.0_x)});
        f=EffectiveVectorMultivariateFunction({4*x+x*x-a-4});
        ARIADNE_TEST_PRINT(f);
        h=solver->implicit(f,p,r);
        ARIADNE_TEST_PRINT(h);
        bb=EffectiveScalarMultivariateFunction(aa-DyadicInterval(p[0]).midpoint())/DyadicInterval(p[0]).radius();
        Decimal c0(0.828427_dec), c1(0.0441942_dec), c2(-0.000345267_dec), c3(0.00000539468_dec);
        e=EffectiveVectorMultivariateFunction( { c0+bb*(c1+bb*(c2+bb*c3)) } );
        ARIADNE_TEST_PRINT(e);
        ARIADNE_TEST_COMPARE(norm(h-e),<,1e-4_pr);

        // Test solution of x-2*a=0 on [-1,+1], taking values in [-1,+1].
        // There is at most one solution, but this lies partially outside the range.
        // Should obtain PartialSolutionException
        p=ExactBoxType({ExactIntervalType(-1,1)});
        r=ExactBoxType({ExactIntervalType(-1,1)});
        f=EffectiveVectorMultivariateFunction({x-2*a});
        ARIADNE_TEST_PRINT(f);
        try {
            h=solver->implicit(f,p,r);
            ARIADNE_TEST_NOTIFY(solver_class_name<<" silently returns partially defined vector implicit function.");
        }
        catch(const SolverException& exc) {
            ARIADNE_TEST_THROWS(solver->implicit(f,p,r),SolverException);
            ARIADNE_TEST_NOTIFY(solver_class_name<<" throws error on partially defined vector implicit function.");
        }

    }

    Void test_scalar_implicit() {
        //TaylorModelAccuracy::set_default_sweep_threshold(1e-12);

        EffectiveScalarMultivariateFunction aa=EffectiveScalarMultivariateFunction::coordinate(1,0);
        EffectiveScalarMultivariateFunction a=EffectiveScalarMultivariateFunction::coordinate(2,0);
        EffectiveScalarMultivariateFunction x=EffectiveScalarMultivariateFunction::coordinate(2,1);
        // Test solution of x-2*a=0 on [-1,+1], taking values in [-1,+1]. There is at most one solution.
        // Uses scalar implicit
        ExactBoxType p; ExactIntervalType r;
        EffectiveScalarMultivariateFunction e,f,s; // s is unscaling functions
        ValidatedScalarMultivariateFunctionPatch h;

        ARIADNE_TEST_PRINT(*solver);

        // Test solution of 4x^2+x-4-a=0 on [0.875_x,1.125_x]. There is a unique solution with positive derivative.
        p=ExactBoxType({ExactIntervalType(0.875_x,1.125_x)});
        r=ExactIntervalType(0.25_x,1.25_x);
        f=EffectiveScalarMultivariateFunction((x*x+1)*x-a);
        ARIADNE_TEST_PRINT(f);
        h=solver->implicit(f,p,r);
        ARIADNE_TEST_PRINT(h);
        s=EffectiveScalarMultivariateFunction(aa-DyadicInterval(p[0]).midpoint())/DyadicInterval(p[0]).radius();
        Decimal a0(0.682328_dec), a1(0.0521547_dec), a2(-0.0023232_dec), a3(0.000147778_dec);
        e=EffectiveScalarMultivariateFunction( a0+s*(a1+s*(a2+s*a3)) );
        ARIADNE_TEST_PRINT(e);
        ARIADNE_TEST_COMPARE(mag((h-e).range()),<,1e-4_pr);

        // Test solution of x-2*a=0 on [-1,+1], taking values in [-1,+1].
        // There is at most one solution, but this lies partially outside the range.
        // Should obtain PartialSolutionException
        p=ExactBoxType({ExactIntervalType(-1,1)});
        r=ExactIntervalType(-1,1);
        f=EffectiveScalarMultivariateFunction(x-2*a);
        ARIADNE_TEST_PRINT(f);
        ValidatedScalarMultivariateFunctionPatch g=ValidatedScalarMultivariateTaylorFunctionModelDP(product(p,r),f,ThresholdSweeper<FloatDP>(dp,1e-12));
        ARIADNE_TEST_PRINT(g);
        try {
            h=solver->implicit(g,p,r);
            ARIADNE_TEST_NOTIFY(solver_class_name<<" silently returns partially defined scalar implicit function.");
        }
        catch(const SolverException& exc) {
            ARIADNE_TEST_THROWS(solver->implicit(g,p,r),SolverException);
            ARIADNE_TEST_NOTIFY(solver_class_name<<" throws error on partially defined scalar implicit function.");
        }
        catch(const DomainException& exc) {
            ARIADNE_TEST_THROWS(solver->implicit(g,p,r),DomainException);
            ARIADNE_TEST_NOTIFY(solver_class_name<<" throws DomainException on partially defined scalar implicit function.");
        }
    }

};

#include "algebra/differential.hpp"

Int main(Int argc, const char **argv) {

    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    IntervalNewtonSolver interval_newton_solver(maximum_error=1e-5_pr,maximum_number_of_steps=12);
    TestSolver(interval_newton_solver,"IntervalNewtonSolver").test();
    KrawczykSolver krawczyk_solver(maximum_error=1e-5_pr,maximum_number_of_steps=12);
    ARIADNE_TEST_PRINT(krawczyk_solver.function_factory());
    TestSolver(krawczyk_solver,"KrawczykSolver").test();

    FactoredKrawczykSolver factored_krawczyk_solver(maximum_error=1e-5_pr,maximum_number_of_steps=12);
    TestSolver(factored_krawczyk_solver,"FactoredKrawczykSolver").test();

    std::cerr<<"INCOMPLETE "<<std::flush;
    return ARIADNE_TEST_FAILURES;
}
