/***************************************************************************
 *            test_procedure.cpp
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

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <stdexcept>

#include "config.hpp"

#include "function/procedure.hpp"
#include "function/procedure.tpl.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/expansion.hpp"
#include "algebra/evaluate.hpp"
#include "function/formula.hpp"

#include "../test.hpp"

#include "algebra/covector.hpp"
#include "function/function.hpp"
#include "algebra/expansion.inl.hpp"

using namespace std;
using namespace Ariadne;

template<class X> decltype(auto) mag(Covector<X> const& u) { return norm(transpose(u)); }

class TestProcedure
{
    DoublePrecision pr;
  public:
    TestProcedure();
    Void test();
  private:
    Void test_construct_from_formula();
    Void test_construct_from_expansion();
    Void test_evaluate();
    Void test_propagate();
    Void test_derivative();
};

TestProcedure::TestProcedure()
{
}

Void TestProcedure::test()
{
    ARIADNE_TEST_CALL(test_construct_from_formula());
    ARIADNE_TEST_CALL(test_construct_from_expansion());
    ARIADNE_TEST_CALL(test_evaluate());
    ARIADNE_TEST_CALL(test_propagate());
    ARIADNE_TEST_CALL(test_derivative());
}


Void TestProcedure::test_construct_from_formula()
{
    ApproximateNumber c(2.0_x);
    ApproximateFormula o(ApproximateFormula::constant(1.0_x));
    ApproximateFormula x(ApproximateFormula::coordinate(0));
    ApproximateFormula y(ApproximateFormula::coordinate(1));

    ApproximateFormula xs=pow(x,2);
    ApproximateFormula ys=pow(y,2);

    ARIADNE_TEST_PRINT(y*c);
    ARIADNE_TEST_PRINT(c*y);
    Vector<ApproximateFormula> f={sqrt(xs+c*ys), atan(y/x), xs*c-ys};
    ARIADNE_TEST_PRINT(f);

    ApproximateProcedure p0(argument_size=2u,f[0]);
    ARIADNE_TEST_EQUALS(p0.argument_size(),2u);
    ARIADNE_TEST_PRINT(p0);
    Vector<ApproximateProcedure> p(argument_size=2u,f);
    ARIADNE_TEST_PRINT(p);
    ARIADNE_TEST_EQUALS(p.result_size(),3u);
    ARIADNE_TEST_EQUALS(p.argument_size(),2u);
}


Void TestProcedure::test_construct_from_expansion()
{
    {
        Expansion<MultiIndex,FloatDPApproximation> e({ {{0,0},1.0_x}, {{1,0},2.0_x}, {{0,2},3.0_x}, {{1,4},4.0_x} },pr);
        ARIADNE_TEST_PRINT(e);
        e.reverse_lexicographic_sort();
        ARIADNE_TEST_PRINT(e);
        Procedure<ApproximateNumber> p(e);
        ARIADNE_TEST_PRINT(p);
        Vector<FloatDPApproximation> x({2.0_x,3.0_x},pr);
        ARIADNE_TEST_PRINT(simple_evaluate(e,x));
        ARIADNE_TEST_PRINT(evaluate(p,x));
        ARIADNE_TEST_EQUAL(evaluate(p,x),simple_evaluate(e,x));
    }

    {
        Expansion<MultiIndex,FloatDPApproximation> e({ {{0,0},1.0_x}, {{1,0},2.0_x}, {{0,1},3.0_x}, {{2,0},4.0_x}, {{1,1},5.0_x}, {{0,2},6.0_x} },pr);
        e.reverse_lexicographic_sort();
        Procedure<ApproximateNumber> p(e);
        ARIADNE_TEST_PRINT(p);
        Vector<FloatDPApproximation> x({2.0_x,3.0_x},pr);
        ARIADNE_TEST_EQUAL(evaluate(p,x),simple_evaluate(e,x));
    }
}


Void TestProcedure::test_evaluate()
{
    ApproximateProcedure p(2);
    p.new_instruction(Var(),0ul);
    p.new_instruction(Var(),1ul);
    p.new_instruction(Pow(),0ul,2);
    p.new_instruction(Sqr(),1ul);
    p.new_instruction(Add(),2ul,3ul);
    p.new_constant(9.0_x);
    p.new_instruction_scalar(Mul(),0ul,4ul);
    p.new_instruction(Sqrt(),5ul);
    ARIADNE_TEST_PRINT(p);

    Vector<FloatDPApproximation> x({3.0_x,4.0_x},pr);
    ARIADNE_TEST_PRINT(x);

    ARIADNE_TEST_EQUALS(evaluate(p,x),15.0_x);
}

Void TestProcedure::test_propagate()
{
    {
        ValidatedProcedure p(2);
        p.new_instruction(Var(),0u);
        p.new_instruction(Var(),1u);
        p.new_instruction(Sqr(),0u);
        p.new_instruction(Sqr(),1u);
        p.new_instruction(Add(),2u,3u);
        p.new_constant(1.125_decimal);
        p.new_instruction_scalar(Mul(),0ul,4ul);
        p.new_instruction(Sqrt(),5u);
        ARIADNE_TEST_PRINT(p);

        UpperBoxType x=ExactBoxType{ {0.25_x,2.0_x}, {0.5_x,3.0_x} };
        ARIADNE_TEST_PRINT(x);

        simple_hull_reduce(x,p,ExactIntervalType(1,1));
        ARIADNE_TEST_PRINT(x);
        simple_hull_reduce(x,p,ExactIntervalType(1,1));
        ARIADNE_TEST_PRINT(x);
    }

    ValidatedFormula x(ValidatedFormula::coordinate(0));
    ValidatedFormula y(ValidatedFormula::coordinate(1));

    Vector<ValidatedFormula> ff={sqrt(sqr(x)+sqr(y)),2*x-y};
    ARIADNE_TEST_PRINT(ff);
    Vector<ValidatedProcedure> pp(argument_size=2,ff);
    ARIADNE_TEST_PRINT(pp);
    UpperBoxType xx=ExactBoxType{ {0.125_x,2.0_x}, {0.25_x,3.0_x} };
    ExactBoxType cc=ExactBoxType{ {1.0_x,1.0_x}, {1.0_x,1.0_x} };
    ARIADNE_TEST_PRINT(xx);
    ARIADNE_TEST_PRINT(evaluate(pp,cast_vector(xx)));
    simple_hull_reduce(xx,pp,cc);
    ARIADNE_TEST_PRINT(xx);
}

Void TestProcedure::test_derivative()
{
    typedef ApproximateTag P;
    typedef Number<P> Y;
    typedef FloatDPApproximation X;

    Y c(2);
    Formula<Y> o(Formula<Y>::constant(1));
    Formula<Y> x(Formula<Y>::coordinate(0));
    Formula<Y> y(Formula<Y>::coordinate(1));

    auto xs=sqr(x); auto ys=sqr(y);
    Formula<Y> e=sqrt(xs+c*ys)+sin(y/x)*xs*c-ys;
    ARIADNE_TEST_PRINT(e);
    ScalarMultivariateFunction<P> f(EuclideanDomain(2),e);
    ARIADNE_TEST_PRINT(f);
    Procedure<Y> p(f);
    ARIADNE_TEST_PRINT(p);

    X zero(0,dp);

    Vector<X> q({2,1},dp);
    Vector<X> s({-1,3},dp);
    Vector<Differential<X>> ds=Differential<X>::variable(1,2,zero,0)*s+q;
    ARIADNE_TEST_WITHIN(gradient(p,q),f.gradient(q),8e-16);
    ARIADNE_TEST_EQUALS(hessian(p,q,s),f(ds).hessian().get(0,0));
}

Int main() {
    TestProcedure().test();
    return ARIADNE_TEST_FAILURES;
}

