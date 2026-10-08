/***************************************************************************
 *            test_taylor_function.cpp
 *
 *  Copyright  2009-20  Pieter Collins
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
#include <iomanip>
#include "config.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/expansion.hpp"
#include "algebra/algebra.hpp"
#include "algebra/sweeper.hpp"
#include "function/taylor_model.hpp"
#include "function/taylor_function.hpp"
#include "algebra/differential.hpp"
#include "function/polynomial.hpp"
#include "function/function.hpp"

#include "function/formula.hpp"
#include "function/symbolic_function.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;

extern template Ariadne::Nat Ariadne::Error<Ariadne::FloatDP>::output_places;
extern template Ariadne::Nat Ariadne::Error<Ariadne::FloatMP>::output_places;

inline Vector<Real> e(Nat n, Nat i) { return Vector<Real>::unit(n,i); }
inline MultivariatePolynomial<FloatDP> p(Nat n, Nat j) { return MultivariatePolynomial<FloatDP>::variable(n,j,dp); }
inline ValidatedScalarMultivariateTaylorFunctionModelDP t(ExactBoxType d, Nat j,Sweeper<FloatDP> swp) { return ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(d,j,swp); }

template<class X> Vector< Expansion<MultiIndex,X> > operator*(const Expansion<MultiIndex,X>& e, const Vector<FloatDP> v) {
    Vector< Expansion<MultiIndex,X> > r(v.size(),Expansion<MultiIndex,X>(e.argument_size(),e.coefficient_characteristics()));
    for(Nat i=0; i!=r.size(); ++i) { ARIADNE_ASSERT(v[i]==0.0_x || v[i]==1.0_x); if(v[i]==1.0_x) { r[i]=e; } }
    return r;
}

class TestScalarTaylorFunction
{
    DoublePrecision pr;
    Sweeper<FloatDP> swp;
  public:
    TestScalarTaylorFunction(Sweeper<FloatDP> sweeper);
    Void test();
  private:
    Void test_concept();
    Void test_create();
    Void test_constructors();
    Void test_predicates();
    Void test_approximation();
    Void test_evaluate();
    Void test_gradient();
    Void test_arithmetic();
    Void test_functions();
    Void test_compose();
    Void test_antiderivative();
    Void test_conversion();
    Void test_generic();
  private:
    ExactBoxType d(SizeType n) { return Vector<ExactIntervalType>(n,ExactIntervalType(-1,+1)); }
    typedef Expansion<MultiIndex,RawFloatDP> e;
    typedef TaylorModel<ValidatedTag,FloatDP> TM;
};


TestScalarTaylorFunction::TestScalarTaylorFunction(Sweeper<FloatDP> sweeper)
    : swp(sweeper)
{
}


Void TestScalarTaylorFunction::test()
{
    FloatDP::set_output_places(18);
    FloatDPError::set_output_places(18);
    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_create());
    ARIADNE_TEST_CALL(test_predicates());
    ARIADNE_TEST_CALL(test_approximation());
    ARIADNE_TEST_CALL(test_arithmetic());
    ARIADNE_TEST_CALL(test_functions());
    ARIADNE_TEST_CALL(test_evaluate());
    ARIADNE_TEST_CALL(test_gradient());
    ARIADNE_TEST_CALL(test_compose());
    ARIADNE_TEST_CALL(test_antiderivative());
    ARIADNE_TEST_CALL(test_conversion());
    ARIADNE_TEST_CALL(test_generic());
}


Void TestScalarTaylorFunction::test_concept()
{
    SizeType k=0;
    const FloatDP w={0,pr};
    const FloatDPBounds x(pr);
    const ValidatedNumber y;
    const Vector<FloatDP> vw;
    const Vector<FloatDPBounds> vx;
    const Vector<ValidatedNumber> vy;
    const BoxDomainType dom;
    const SweeperDP swp;
    ValidatedScalarMultivariateTaylorFunctionModelDP stf(dom,swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP stfr(dom,swp);

    stfr=stf+w; stfr=stf-w; stfr=stf*w; stfr=stf/w;
    stfr=w+stf; stfr=w-stf; stfr=w*stf; stfr=w/stf;
    stfr=stf+x; stfr=stf-x; stfr=stf*x; stfr=stf/x;
    stfr=x+stf; stfr=x-stf; stfr=x*stf; stfr=x/stf;
    stfr=stf+y; stfr=stf-y; stfr=stf*y; stfr=stf/y;
    stfr=y+stf; stfr=y-stf; stfr=y*stf; stfr=y/stf;
    stfr=stf+stf; stfr=stf-stf; stfr=stf*stf; stfr=stf/stf;

    stfr+=w; stfr-=w; stfr*=w; stfr/=w;
    stfr+=x; stfr-=x; stfr*=x; stfr/=x;
    stfr+=y; stfr-=y; stfr*=y; stfr/=y;
    stfr+=stf; stfr-=stf;

    stfr=pos(stf); stfr=neg(stf); stfr=sqr(stf);
    stfr=rec(stf); stfr=pow(stf,1); stfr=pow(stf,-1);
    stfr=exp(stf); stfr=log(stf); stfr=sqrt(stf);
    stfr=sin(stf); stfr=cos(stf); stfr=tan(stf);
    //stfr=asin(stf); stfr=acos(stf); stfr=atan(stf);
    stfr=max(stf,stfr); stfr=min(stf,stfr); stfr=abs(stfr);

    stfr.simplify(); stfr.clobber();

    stf(vx); stf(vy); evaluate(stf,vx); unchecked_evaluate(stf,vx); partial_evaluate(stf,k,x);
    stf(vy); stf(vy); evaluate(stf,vy); unchecked_evaluate(stf,vy); partial_evaluate(stf,k,y);
    stf.domain(); stf.range(); stf.expansion(); stf.error();

    ScaledFunctionPatch<ValidatedTaylorModelDP> vstfp(dom,swp);

}

Void TestScalarTaylorFunction::test_constructors()
{
    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP,tv1,({{-1,+1},{-1,+1}},{{{0,0},1.0_x},{{1,0},2.0_x},{{0,1},3.0_x},{{2,0},4.0_x},{{1,1},5.0_x},{{0,2},6.0_x},{{3,0},7.0_x},{{2,1},8.0_x},{{1,2},9.0_x},{{0,3},10.0_x}},0.25_x,swp));

    ARIADNE_ASSERT_EQUAL(tv1.domain(),Box<ExactIntervalType>({{-1,+1},{-1,+1}}));
    ARIADNE_ASSERT_EQUAL(tv1.argument_size(),2);
    ARIADNE_ASSERT_EQUAL(tv1.number_of_nonzeros(),10);
    ARIADNE_ASSERT_EQUAL(tv1.value().raw(),1.0_x);
    ARIADNE_ASSERT_EQUAL(tv1.error().raw(),0.25_x);
}

Void TestScalarTaylorFunction::test_create()
{
    ExactBoxType D={{-0.5_x,0.5_x},{-1.0_x,2.0_x}};
    FloatDP c1={1,pr};
    FloatDP c2={1,pr};
    ValidatedScalarMultivariateTaylorFunctionModelDP pf1=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(D,c1,swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP pf2=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(D,c2,swp);
    ValidatedScalarMultivariateFunction f2=pf2;

    ValidatedScalarMultivariateFunctionPatch fp1=pf1;
    ValidatedScalarMultivariateFunctionPatch fp2=pf2;
    ARIADNE_TEST_EQUAL(factory(pf1).create(pf2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(pf1).create(fp2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(pf1).create(f2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(fp1).create(fp2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(fp1).create(f2).range(),c2);

    ValidatedScalarMultivariateFunctionModelDP fm1=pf1;
    ValidatedScalarMultivariateFunctionModelDP fm2=pf2;
    f2=fm2;
    ARIADNE_TEST_EQUAL(factory(pf1).create(pf2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(pf1).create(fm2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(pf1).create(f2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(fm1).create(fm2).range(),c2);
    ARIADNE_TEST_EQUAL(factory(fm1).create(f2).range(),c2);
}

Void TestScalarTaylorFunction::test_predicates()
{
    ValidatedScalarMultivariateTaylorFunctionModelDP tv1(d(1),{{{0},1.00_x},{{1},2.00_x},{{2},3.00_x}}, 0.75_x, swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP tv2(d(1),{{{0},1.00_x},{{1},1.75_x},{{2},3.25_x}}, 0.25_x, swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP tv3(d(1),{{{0},1.125_x},{{1},1.75_x},{{2},3.25_x}}, 0.25_x, swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP tv4(d(1),{{{0},1.00_x},{{1},2.25_x},{{2},3.00_x},{{3},-0.25_x}}, 0.25_x, swp);

    ARIADNE_TEST_BINARY_PREDICATE(refines,tv1,tv1);
    ARIADNE_TEST_BINARY_PREDICATE(refines,tv2,tv1);
    ARIADNE_TEST_BINARY_PREDICATE(!refines,tv3,tv1);
    ARIADNE_TEST_BINARY_PREDICATE(refines,tv4,tv1);
}

Void TestScalarTaylorFunction::test_approximation()
{
//    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP,tv1,(d(2),e(2,3,{1.0_x,2.0_x,3.0_x,4.0_x,5.0_x,6.0_x,7.0_x,8.0_x,9.0_x,10.0_x}),0.25_x,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP,tv2,(d(1),{{{0},1.0_x},{{1},2.0_x},{{2},3.0_x}},0.25_x,swp));
}

Void TestScalarTaylorFunction::test_evaluate()
{
    Vector<FloatDPBounds> iv({{0.25_x,0.5_x},{-0.75_x,-0.5_x}},pr);
    ValidatedScalarMultivariateTaylorFunctionModelDP tv(d(2),{{{0,0},1.0_x},{{1,0},2.0_x},{{0,1},3.0_x},{{2,0},4.0_x},{{1,1},5.0_x},{{0,2},6.0_x}},0.25_x,swp);
    ARIADNE_TEST_SAME(evaluate(tv,iv),FloatDPBounds(-0.375000_x,3.43750_x,pr));
}

Void TestScalarTaylorFunction::test_gradient()
{
/*
    EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(2);
    Real a(1.5_x); Real b(0.25_x);
    EffectiveScalarMultivariateFunction quadratic = a-x[0]*x[0]+b*x[1];

    ExactBoxType domain1={{-1.0_x,+1.0_x},{-1.0_x,+1.0_x}};
    ExactBoxType domain2={{-0.5_x,+0.5_x},{-0.25_x,+0.25_x}};
    ExactBoxType domain3={{-0.25_x,+0.75_x},{0.0_x,+0.50_x}};

    Vector<FloatDPBounds> point1=Vector<FloatDPBounds>{0.0_x,0.0_x};
    Vector<FloatDPBounds> point2=Vector<FloatDPBounds>{0.0_x,0.25_x};

    ARIADNE_TEST_EQUAL(ValidatedScalarMultivariateTaylorFunctionModelDP(domain1,quadratic,swp).gradient(point1),quadratic.gradient(point1));
    ARIADNE_TEST_EQUAL(ValidatedScalarMultivariateTaylorFunctionModelDP(domain1,quadratic,swp).gradient(point2),quadratic.gradient(point2));
    ARIADNE_TEST_EQUAL(ValidatedScalarMultivariateTaylorFunctionModelDP(domain2,quadratic,swp).gradient(point1),quadratic.gradient(point1));
    ARIADNE_TEST_EQUAL(ValidatedScalarMultivariateTaylorFunctionModelDP(domain2,quadratic,swp).gradient(point2),quadratic.gradient(point2));
    ARIADNE_TEST_EQUAL(ValidatedScalarMultivariateTaylorFunctionModelDP(domain3,quadratic,swp).gradient(point1),quadratic.gradient(point1));
    ARIADNE_TEST_EQUAL(ValidatedScalarMultivariateTaylorFunctionModelDP(domain3,quadratic,swp).gradient(point2),quadratic.gradient(point2));
*/
}


Void TestScalarTaylorFunction::test_arithmetic()
{
    //Operations which can be performed exactly with floating-point arithmetic.
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)+(-3), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},-2.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)-(-3), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},4.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)*(-3), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},-3.0_x},{{1},6.0_x},{{2},-9.0_x}}, 2.25_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)/(-4), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},-0.25_x},{{1},0.5_x},{{2},-0.75_x}}, 0.1875_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)+FloatDPBounds(-1,2,pr), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.5_x},{{1},-2.0_x},{{2},3.0_x}}, 2.25_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)-FloatDPBounds(-1,2,pr), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},0.5_x},{{1},-2.0_x},{{2},3.0_x}}, 2.25_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)*FloatDPBounds(-1,2,pr), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},0.5_x},{{1},-1.0_x},{{2},1.5_x}}, 10.5_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)/FloatDPBounds(0.25_x,2.0_x,pr), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},2.25_x},{{1},-4.5_x},{{2},6.75_x}}, 13.5_x, swp));
    ARIADNE_TEST_SAME(+ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp));
    ARIADNE_TEST_SAME(-ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},-1.0_x},{{1},2.0_x},{{2},-3.0_x}}, 0.75_x, swp));

    // Regression test to check subtraction yielding zero coefficients
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)+ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},3.0_x},{{1},2.0_x},{{2},-4.0_x}}, 0.5_x, swp), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},4.0_x},{{1},0.0_x},{{2},-1.0_x}}, 1.25_x, swp));

    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)-ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},3.0_x},{{1},2.0_x},{{2},-4.0_x}}, 0.5_x, swp), ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},-2.0_x},{{1},-4.0_x},{{2},7.0_x}}, 1.25_x, swp));
    ARIADNE_TEST_SAME(ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x, swp)*ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},3.0_x},{{1},2.0_x},{{2},-4.0_x}}, 0.5_x, swp),
                       ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},3.0_x},{{1},-4.0_x},{{2},1.0_x},{{3},14.0_x},{{4},-12.0_x}}, 10.125_x, swp));

}

Void TestScalarTaylorFunction::test_functions()
{
    ValidatedScalarMultivariateTaylorFunctionModelDP xz(d(1),{{{0},0.0_x},{{1}, 0.5_x}}, 0.0_x, swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP xo(d(1),{{{0},1.0_x},{{1}, 0.5_x}}, 0.0_x, swp);

    //Functions based on their natural defining points
    ARIADNE_TEST_BINARY_PREDICATE(refines,exp(xz),ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.00000_pr},{{1},0.50000_pr},{{2},0.12500_pr},{{3},0.02083_pr},{{4},0.00260_pr},{{5},0.00026_pr},{{6},0.00002_pr}}, 0.00003_pr, swp));
    ARIADNE_TEST_BINARY_PREDICATE(refines,sin(xz),ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},0.00000_pr},{{1},0.50000_pr},{{2},0.0000_pr},{{3},-0.02083_pr},{{4},0.00000_pr},{{5},0.00026_pr},{{6},0.00000_pr}}, 0.00003_pr, swp));
    ARIADNE_TEST_BINARY_PREDICATE(refines,cos(xz),ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.00000_pr},{{1},0.0000_pr},{{2},-0.12500_pr},{{3},0.00000_pr},{{4},0.00260_pr},{{5},0.0000_pr},{{6},-0.00002_pr}}, 0.00003_pr, swp));

    ARIADNE_TEST_BINARY_PREDICATE(refines,rec(xo),ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.000000_pr},{{1},-0.500000_pr},{{2},0.250000_pr},{{3},-0.125000_pr},{{4}, 0.062500_pr},{{5},-0.031250_pr},{{6},0.015625_pr}}, 0.018_pr, swp));
    ARIADNE_TEST_BINARY_PREDICATE(refines,sqrt(xo),ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},1.000000_pr},{{1}, 0.250000_pr},{{2},-0.031250_pr},{{3}, 0.007813_pr},{{4},-0.002441_pr},{{5}, 0.000854_pr},{{6},-0.000320_pr}}, 0.0003_pr, swp));
    ARIADNE_TEST_BINARY_PREDICATE(refines,log(xo),ValidatedScalarMultivariateTaylorFunctionModelDP(d(1),{{{0},0.000000_pr},{{1},0.500000_pr},{{2},-0.125000_pr},{{3}, 0.041667_pr},{{4},-0.015625_pr},{{5}, 0.006250_pr},{{6},-0.002604_pr}}, 0.003_pr, swp));

}


Void TestScalarTaylorFunction::test_compose()
{
}


Void TestScalarTaylorFunction::test_antiderivative()
{
    ValidatedScalarMultivariateTaylorFunctionModelDP tm=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(d(2),1.0_x,swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP atm=antiderivative(tm,1u);

    ARIADNE_TEST_CONSTRUCT(ExactBoxType,dom,({{1.0_x, 4.0_x}, {0.0_x, 2.0_x}}));
    ValidatedVectorMultivariateTaylorFunctionModelDP x = ValidatedVectorMultivariateTaylorFunctionModelDP::identity(dom,swp);

    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP, f, (1 + 2*x[0]+3*x[1]+5*x[0]*x[0]+4*x[0]*x[1]));
    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP, g, (antiderivative(f,0,2.0_x)) );
    ARIADNE_TEST_PRINT( derivative(g,0) );
    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP, dg, (derivative(g,0)) );
    ARIADNE_TEST_LESS(norm(dg-f),1e-8);

    // We should have f(c,s)=0 for all x1
    ValidatedScalarMultivariateTaylorFunctionModelDP s = ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate({dom[1]},0u,swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP c = ValidatedScalarMultivariateTaylorFunctionModelDP::constant(s.domain(),2.0_x,swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP h=compose(g,ValidatedVectorMultivariateTaylorFunctionModelDP({c,s}));

    ExactBoxType hdom=h.domain();
    Vector<FloatDPBounds> domv(cast_singleton(hdom));
    ARIADNE_ASSERT(definitely(mag(h(domv))<1e-8_pr));

}

Void TestScalarTaylorFunction::test_conversion() {
    {
        SizeType as=2;
        ValidatedTaylorModelDP ty=ValidatedTaylorModelDP::coordinate(as,1,swp);
        ValidatedTaylorModelDP tc=ValidatedTaylorModelDP::constant(as, 2, swp);
        ARIADNE_TEST_PRINT(ty);
        ARIADNE_TEST_PRINT(tc);
        ARIADNE_TEST_PRINT(rec(tc));
        return;
        ARIADNE_TEST_PRINT(ty/tc);
        ARIADNE_TEST_PRINT(rec(tc+ty));

    }


    // Test conversion between ordinary functions and Taylor functions.
    ExactBoxType D={{-0.5_x,0.5_x},{-1.0_x,2.0_x}};
    Vector<FloatDP> pt({-0.25_x,0.25_x},pr);
    Vector<FloatDPBounds> ipt(pt);
    EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(2);
    ARIADNE_TEST_PRINT(x);
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,tx,(D,x,swp));

    ARIADNE_TEST_CONSTRUCT(EffectiveScalarMultivariateFunction,f,(1-x[0]*x[0]-x[1]/2));
    ARIADNE_TEST_PRINT(compose(f,tx));
    ARIADNE_TEST_CONSTRUCT(EffectiveScalarMultivariateFunction,y,(x[1]/2));
    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP,ty,(D,x[1]/2,swp));
    ARIADNE_TEST_PRINT(ValidatedScalarMultivariateTaylorFunctionModelDP(D,x[1],swp)/2);
    ARIADNE_TEST_CONSTRUCT(ValidatedScalarMultivariateTaylorFunctionModelDP,tf,(D,f,swp));

    // Conversion to TaylorFunction should be exact in second component
    ARIADNE_TEST_BINARY_PREDICATE(refines,f(ipt),tf(ipt));
    ARIADNE_TEST_BINARY_PREDICATE(refines,tf(ipt),f(ipt)+FloatDPBounds(-1e-15_pr,1e-15_pr,pr));
}

Void TestScalarTaylorFunction::test_generic() {
    ExactBoxType D={{-0.5_x,0.5_x},{-1.0_x,2.0_x}};
    FloatDP c0={0,pr};
    FloatDP c1={0,pr};
    FloatDP c2={0,pr};
    ValidatedScalarMultivariateTaylorFunctionModelDP pf0=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(D,c0,swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP pf1=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(D,c1,swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP pf2=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(D,c2,swp);

    ValidatedScalarMultivariateFunction f1=pf1;
    ValidatedScalarMultivariateFunction f2=pf2;

    ValidatedScalarMultivariateFunctionPatch fp1=pf1;
    ValidatedScalarMultivariateFunctionPatch fp2=pf2;
    ValidatedScalarMultivariateFunctionPatch fp4=pf0;
    ARIADNE_TEST_EQUAL((pf1+pf2).range(),c1+c2);
    //ARIADNE_TEST_EQUAL((pf1+fp2).range(),c1+c2);
    ARIADNE_TEST_EQUAL((pf1+ f2).range(),c1+c2);
    //ARIADNE_TEST_EQUAL((fp1+pf2).range(),c1+c2);
    ARIADNE_TEST_EQUAL((fp1+fp2).range(),c1+c2);
    ARIADNE_TEST_EQUAL((fp1+ f2).range(),c1+c2);
    ARIADNE_TEST_EQUAL(( f1+pf2).range(),c1+c2);
    ARIADNE_TEST_EQUAL(( f1+fp2).range(),c1+c2);
    ARIADNE_TEST_EQUAL(( f1+ f2)(D.centre()),c1+c2);

    ValidatedScalarMultivariateFunctionModelDP fm1=pf1;
    ValidatedScalarMultivariateFunctionModelDP fm2=pf2;
    ValidatedScalarMultivariateFunctionModelDP fm4=pf0;
    f1=fm1;
    f2=fm2;
    ARIADNE_TEST_EQUAL((pf1+pf2).range(),c1+c2);
    //ARIADNE_TEST_EQUAL((pf1+fm2).range(),c1+c2);
    ARIADNE_TEST_EQUAL((pf1+ f2).range(),c1+c2);
    //ARIADNE_TEST_EQUAL((fm1+pf2).range(),c1+c2);
    ARIADNE_TEST_EQUAL((fm1+fm2).range(),c1+c2);
    ARIADNE_TEST_EQUAL((fm1+ f2).range(),c1+c2);
    ARIADNE_TEST_EQUAL(( f1+pf2).range(),c1+c2);
    ARIADNE_TEST_EQUAL(( f1+fm2).range(),c1+c2);
    ARIADNE_TEST_EQUAL(( f1+ f2)(D.centre()),c1+c2);
}

/*
ValidatedVectorMultivariateTaylorFunctionModelDP henon(const ValidatedVectorMultivariateTaylorFunctionModelDP& x, const Vector<FloatDP>& p)
{
    ValidatedVectorMultivariateTaylorFunctionModelDP r(2,2,x.degree()); henon(r,x,p); return r;
}
*/

class TestVectorTaylorFunction
{
    DoublePrecision pr;
    Sweeper<FloatDP> swp;
  public:
    TestVectorTaylorFunction(Sweeper<FloatDP> sweeper);
    Void test();
  private:
    Void test_constructors();
    Void test_restrict();
    Void test_jacobian();
    Void test_compose();
    Void test_antiderivative();
    Void test_join();
    Void test_combine();
    Void test_conversion();
    Void test_domain();
};


TestVectorTaylorFunction::TestVectorTaylorFunction(Sweeper<FloatDP> sweeper)
    : swp(sweeper)
{
  std::cout<<std::setprecision(17);
  std::cerr<<std::setprecision(17);
}


Void
TestVectorTaylorFunction::test()
{
    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_restrict());
    ARIADNE_TEST_CALL(test_jacobian());
    ARIADNE_TEST_CALL(test_antiderivative());
    ARIADNE_TEST_CALL(test_compose());
    ARIADNE_TEST_CALL(test_join());
    ARIADNE_TEST_CALL(test_combine());
    ARIADNE_TEST_CALL(test_conversion());
    ARIADNE_TEST_CALL(test_domain());
}

template<class I, class F> Expansion<I,F>const& cast_exact(Expansion<I,F> const& e) {
    return reinterpret_cast<Expansion<I,F>const&>(e); }

Void TestVectorTaylorFunction::test_constructors()
{
    Vector< Expansion<MultiIndex,RawFloatDP> > expansion(2, Expansion<MultiIndex,RawFloatDP>(2,pr));
    expansion[0]=Expansion<MultiIndex,RawFloatDP>({ {{0,0},1.125_x}, {{1,0},-0.75_x}, {{0,1},0.0625_x}, {{2,0},-0.25_x} },pr);
    expansion[1]=Expansion<MultiIndex,RawFloatDP>({ {{0,0},0.750_x}, {{1,0},0.50_x} },pr);
    expansion[0].reverse_lexicographic_sort(); expansion[1].reverse_lexicographic_sort();
    Vector< RawFloatDP > errors(2,dp);

    ExactBoxType domain={{0.25_x,1.25_x},{0.5_x,1.0_x}};
    EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(2);
    Real a(1.5_x); Real b(0.25_x);
    EffectiveVectorMultivariateFunction henon_function={a-x[0]*x[0]+b*x[1], x[0]*1};
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,henon_model,(domain,henon_function,swp));
    ARIADNE_TEST_SAME(henon_model.models()[0].expansion(),cast_exact(expansion[0]))
    ARIADNE_TEST_SAME(henon_model.models()[1].expansion(),cast_exact(expansion[1]))

    Vector<FloatDPBounds> e0(e(2,0),pr); Vector<FloatDPBounds> e1(e(2,1),pr);

    ValidatedVectorMultivariateTaylorFunctionModelDP t=ValidatedVectorMultivariateTaylorFunctionModelDP::identity(domain,swp);
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,variables_model,((1.5_x-t[0]*t[0]+0.25_x*t[1])*e0+t[0]*e1));
    ARIADNE_TEST_EXECUTE(variables_model.simplify());
    ARIADNE_TEST_SAME(variables_model,ValidatedVectorMultivariateTaylorFunctionModelDP(domain,expansion,errors,swp));

}

Void TestVectorTaylorFunction::test_restrict()
{
    Vector<RawFloatDP> unit0=Vector<RawFloatDP>::unit(1,0,pr);

    ExactBoxType domain1={{-1.0_x,+1.0_x},{-1.0_x,+1.0_x}};
    Expansion<MultiIndex,RawFloatDP> expansion1({{{0,0},1.0_x}, {{1,0},2.0_x},{{0,1},3.0_x}, {{2,0},4.0_x},{{1,1},5.0_x},{{0,2},6.0_x}, {{3,0},7.0_x},{{2,1},8.0_x},{{1,2},9.0_x},{{0,3},10.0_x}},pr);
    Vector<ExactIntervalType> subdomain1={{-0.25_x,0.75_x},{-0.5_x,0.0_x}};
    Expansion<MultiIndex,RawFloatDP> subexpansion1({{{0,0},1.031250_x},{{1,0},1.812500_x},{{0,1},0.625000_x}, {{2,0},1.812500_x},{{1,1},0.562500_x},{{0,2},0.0468750_x},
                                        {{3,0},0.875000_x},{{2,1},0.500000_x},{{1,2},0.281250_x},{{0,3},0.156250_x}},pr);
    Vector<RawFloatDP> error1({0.0_x},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP function1(domain1,expansion1*unit0,error1,swp);
    ValidatedVectorMultivariateTaylorFunctionModelDP restricted_function1(subdomain1,subexpansion1*unit0,error1,swp);
    ARIADNE_TEST_SAME(restriction(function1,subdomain1),restricted_function1);

    ExactBoxType domain2={{-1.0_x,+1.0_x}};
    Expansion<MultiIndex,RawFloatDP> expansion2({{{0},0.0_x},{{1},1.0_x}},pr);
    Vector<RawFloatDP> error2({0.125_x},pr);
    Vector<ExactIntervalType> subdomain2={{3e-16_pr,1.0_x}};
    Expansion<MultiIndex,RawFloatDP> subexpansion2({{{0},0.50000000000000022_pr},{{1},0.49999999999999989_pr}},pr);
    Vector<RawFloatDP> suberror2({0.125000000000000028_pr},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP function2(domain2,expansion2*unit0,error2,swp);
    ValidatedVectorMultivariateTaylorFunctionModelDP restricted_function2(subdomain2,subexpansion2*unit0,suberror2,swp);
    FloatDP::set_output_places(18);
    FloatDPError::set_output_places(18);

    ARIADNE_TEST_SAME(restriction(function2,subdomain2),restricted_function2);
    ARIADNE_TEST_SAME(restriction(function2,subdomain2).domain(),restricted_function2.domain());
    ARIADNE_TEST_SAME(restriction(function2,subdomain2).error(),restricted_function2.error());
}

Void TestVectorTaylorFunction::test_jacobian()
{
    EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(2);
    Real a(1.5_x); Real b(0.25_x);
    EffectiveVectorMultivariateFunction henon={a-x[0]*x[0]+b*x[1], x[0]*1};
    ExactBoxType domain1={{-1.0_x,+1.0_x},{-1.0_x,+1.0_x}};
    ExactBoxType domain2={{-0.5_x,+0.5_x},{-0.25_x,+0.25_x}};
    ExactBoxType domain3={{-0.25_x,+0.75_x},{0.0_x,+0.50_x}};
    //Vector<FloatDPApproximation> point1={0.0_x,0.0_x};
    //Vector<FloatDPApproximation> point2={0.5_x,0.25_x};
    Vector<FloatDPBounds> point1=Vector<FloatDPBounds>({0.0_x,0.0_x},pr);
    Vector<FloatDPBounds> point2=Vector<FloatDPBounds>({0.0_x,0.25_x},pr);
    //Vector<FloatDP> point1=Vector<FloatDP>({0.0_x,0.0_x},pr);
    //Vector<FloatDP> point2=Vector<FloatDP>({0.5_x,0.25_x},pr);
    ARIADNE_TEST_EQUAL(ValidatedVectorMultivariateTaylorFunctionModelDP(domain1,henon,swp).jacobian(point1),henon.jacobian(point1));
    ARIADNE_TEST_EQUAL(ValidatedVectorMultivariateTaylorFunctionModelDP(domain1,henon,swp).jacobian(point2),henon.jacobian(point2));
    ARIADNE_TEST_EQUAL(ValidatedVectorMultivariateTaylorFunctionModelDP(domain2,henon,swp).jacobian(point1),henon.jacobian(point1));
    ARIADNE_TEST_EQUAL(ValidatedVectorMultivariateTaylorFunctionModelDP(domain2,henon,swp).jacobian(point2),henon.jacobian(point2));
    ARIADNE_TEST_EQUAL(ValidatedVectorMultivariateTaylorFunctionModelDP(domain3,henon,swp).jacobian(point1),henon.jacobian(point1));
    ARIADNE_TEST_EQUAL(ValidatedVectorMultivariateTaylorFunctionModelDP(domain3,henon,swp).jacobian(point2),henon.jacobian(point2));
}

Void TestVectorTaylorFunction::test_compose()
{
    Real a(1.5_x); Real b(0.25_x);
    EffectiveScalarMultivariateFunction x=EffectiveScalarMultivariateFunction::coordinate(2,0);
    EffectiveScalarMultivariateFunction y=EffectiveScalarMultivariateFunction::coordinate(2,1);
    EffectiveVectorMultivariateFunction henon_polynomial=(a-x*x+b*y)*e(2,0)+x*e(2,1);
    EffectiveVectorMultivariateFunction henon_square_polynomial=
        {a*(1-a)+b*x-2*a*b*y+2*a*x*x-b*b*y*y+2*b*x*x*y-x*x*x*x, a-x*x+b*y};
    //    compose(henon_polynomial,henon_polynomial);
    ExactBoxType domain1={{0.25_x,1.25_x},{0.5_x,1.0_x}};
    ExactBoxType domain2={{-1.5_x,2.5_x},{0.25_x,1.25_x}};
    ARIADNE_TEST_PRINT((a-x*x+b*y));
    ARIADNE_TEST_PRINT(e(2,0));
    ARIADNE_TEST_PRINT((a-x*x+b*y)*e(2,0));
    ARIADNE_TEST_PRINT(henon_polynomial);
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,function1,(domain1,henon_polynomial,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,function2,(domain2,henon_polynomial,swp));

    ValidatedVectorMultivariateTaylorFunctionModelDP composition1(domain1,henon_square_polynomial,swp);
    ARIADNE_TEST_SAME(compose(function2,function1),composition1);
}


Void TestVectorTaylorFunction::test_antiderivative()
{
    SizeType index0=0;
    SizeType index1=1;

    Vector<RawFloatDP> unit0=Vector<RawFloatDP>::unit(1,0,pr);
    ExactBoxType domain1={{-1,+1},{-1,+1}};
    Expansion<MultiIndex,RawFloatDP> expansion1({{{0,0},3.0_x}},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP function1(domain1,expansion1*unit0,swp);
    Expansion<MultiIndex,RawFloatDP> aexpansion1({{{0,1},3.0_x}},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP antiderivative1(domain1,aexpansion1*unit0,swp);
    ARIADNE_TEST_SAME(antiderivative(function1,index1),antiderivative1);

    ExactBoxType domain2={{-0.25_x,0.75_x},{0.0_x,0.5_x}};
    Expansion<MultiIndex,RawFloatDP> expansion2({{{0,0},3.0_x}},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP function2(domain2,expansion2*unit0,swp);
    Expansion<MultiIndex,RawFloatDP> aexpansion2({{{0,1},0.75_x}},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP antiderivative2(domain2,aexpansion2*unit0,swp);
    ARIADNE_TEST_SAME(antiderivative(function2,index1),antiderivative2);

    ExactBoxType domain3={{-0.25_x,0.75_x},{0.0_x,0.5_x}};
    Expansion<MultiIndex,RawFloatDP> expansion3({{{0,0},1.0_x},{{1,0},2.0_x},{{0,1},3.0_x},{{2,0},4.0_x},{{1,1},5.0_x},{{0,2},6.0_x}},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP function3(domain3,expansion3*unit0,swp);
    Expansion<MultiIndex,RawFloatDP> aexpansion30({{{1,0},0.5_x},{{2,0},0.5_x},{{1,1},1.5_x},{{3,0},0.66666666666666663_pr},{{2,1},1.25_x},{{1,2},3.0_x}},pr);
    Vector<FloatDP> aerror30({5.5511151231257827e-17_pr},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP antiderivative30(domain3,aexpansion30*unit0,aerror30,swp);
    ARIADNE_TEST_SAME(antiderivative(function3,index0),antiderivative30);
    Expansion<MultiIndex,RawFloatDP> aexpansion31({{{0,1},0.25_x},{{1,1},0.5_x},{{0,2},0.375_x},{{2,1},1.0_x},{{1,2},0.625_x},{{0,3},0.5_x}},pr);
    ValidatedVectorMultivariateTaylorFunctionModelDP antiderivative31(domain3,aexpansion31*unit0,swp);
    ARIADNE_TEST_SAME(antiderivative(function3,index1),antiderivative31);

}

Void TestVectorTaylorFunction::test_join()
{
    ExactBoxType domain={{-0.25_x,+0.25_x},{-0.5_x,+0.5_x}};
    EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(2);
    EffectiveVectorMultivariateFunction function1 = (x[0]*x[0]+2*x[0]*x[1]+3*x[1]*x[1])*e(1,0);
    EffectiveVectorMultivariateFunction function2 = (4*x[0]*x[0]+5*x[0]*x[1]+6*x[1]*x[1])*e(2,1);
    EffectiveVectorMultivariateFunction function3 = (x[0]*x[0]+2*x[0]*x[1]+3*x[1]*x[1])*e(3,0)
        + (4*x[0]*x[0]+5*x[0]*x[1]+6*x[1]*x[1])*e(3,2);

    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,taylorfunction1,(domain,function1,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,taylorfunction2,(domain,function2,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,taylorfunction3,(domain,function3,swp));
    ARIADNE_TEST_SAME(join(taylorfunction1,taylorfunction2),taylorfunction3);

}

Void TestVectorTaylorFunction::test_combine()
{
    // This test contains a regression test to check correct behaviour for a zero component.
    ExactBoxType domain1={{-0.25_x,+0.25_x},{-0.5_x,+0.5_x}};
    ExactBoxType domain2={{-0.75_x,+0.75_x},{-1.0_x,+1.0_x},{-1.25_x,+1.25_x}};
    ExactBoxType domain3={{-0.25_x,+0.25_x},{-0.5_x,+0.5_x},{-0.75_x,+0.75_x},{-1.0_x,+1.0_x},{-1.25_x,+1.25_x}};
    EffectiveVectorMultivariateFunction x;
    x=EffectiveVectorMultivariateFunction::identity(2);
    EffectiveVectorMultivariateFunction function1 = (x[0]*x[0]+2*x[0]*x[1]+3*x[1]*x[1])*e(1,0);
    x=EffectiveVectorMultivariateFunction::identity(3);
    EffectiveVectorMultivariateFunction function2 = (4*x[0]*x[0]+5*x[0]*x[1]+6*x[1]*x[2])*e(2,1);
    x=EffectiveVectorMultivariateFunction::identity(5);
    EffectiveVectorMultivariateFunction function3 = (x[0]*x[0]+2*x[0]*x[1]+3*x[1]*x[1])*e(3,0)
        + (4*x[2]*x[2]+5*x[2]*x[3]+6*x[3]*x[4])*e(3,2);
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,taylorfunction1,(domain1,function1,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,taylorfunction2,(domain2,function2,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedVectorMultivariateTaylorFunctionModelDP,taylorfunction3,(domain3,function3,swp));
    ARIADNE_TEST_SAME(combine(taylorfunction1,taylorfunction2),taylorfunction3);

}

Void TestVectorTaylorFunction::test_conversion()
{
    // Test conversion between ordinary functions and Taylor functions.
    ExactBoxType D={{-0.5_x,0.5_x},{-1.0_x,2.0_x}};
    Vector<RawFloatDP> pt({-0.25_x,0.25_x},pr);
    Vector<FloatDPBounds> ipt(pt);
    Vector<FloatDPApproximation> apt(pt);
    EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(2);

    EffectiveVectorMultivariateFunction h={1-x[0]*x[0]-x[1]/2,x[0]+Real(0)};
    ValidatedVectorMultivariateTaylorFunctionModelDP th(D,h,swp);

    ARIADNE_TEST_PRINT(h);
    ARIADNE_TEST_PRINT(th);

    // Conversion to TaylorFunction should be exact in second component
    ARIADNE_TEST_SAME(th(apt)[1],h(apt)[1]);
    ARIADNE_TEST_SAME(th(ipt)[1],h(ipt)[1]);
    ARIADNE_TEST_BINARY_PREDICATE(refines,h[0](ipt),th[0](ipt));


}

// Regression test for domain with empty interior
Void TestVectorTaylorFunction::test_domain()
{
    EffectiveScalarMultivariateFunction z=EffectiveScalarMultivariateFunction::constant(2,0);
    EffectiveScalarMultivariateFunction o=EffectiveScalarMultivariateFunction::constant(2,1);
    EffectiveScalarMultivariateFunction x0=EffectiveScalarMultivariateFunction::coordinate(2,0);
    EffectiveScalarMultivariateFunction x1=EffectiveScalarMultivariateFunction::coordinate(2,1);

    ExactBoxType D1={{-1.0_x,1.0_x},{-1.0_x,1.0_x}};
    ValidatedVectorMultivariateTaylorFunctionModelDP t1(D1, {o,x0+x1}, swp);
    ARIADNE_TEST_PRINT(t1);
    ARIADNE_TEST_PRINT(t1.codomain());
    ExactBoxType D2={{1.0_x,1.0_x},{-2.0_x,2.0_x}};
    ValidatedScalarMultivariateTaylorFunctionModelDP t2(D2,2*x0+x1*x1,swp);
    ARIADNE_TEST_PRINT(t2.domain());
    ARIADNE_TEST_PRINT(t2.model());
    ARIADNE_TEST_PRINT(t2.codomain());

    ARIADNE_TEST_PRINT(t2);
    ARIADNE_TEST_PRINT(compose(t2,t1));
    ValidatedScalarMultivariateTaylorFunctionModelDP t3(D1,2+(x0+x1)*(x0+x1),swp);
    ARIADNE_TEST_SAME(compose(t2,t1),t3);

    Vector<FloatDPBounds> x({{1.0_x,1.0_x},{0.5_x,1.5_x}},pr);
    ARIADNE_TEST_PRINT(x);
    ARIADNE_TEST_SAME(evaluate(t2,x),FloatDPBounds(2.25_x,4.25_x,pr));

    // Ensure evaluation and composition throw errors when expected
    Vector<FloatDPBounds> xe({{0.875_x,1.125_x},{0.5_x,1.5_x}},pr);
    ARIADNE_TEST_THROWS(t2(xe),DomainException);
    ARIADNE_TEST_THROWS(evaluate(t2,xe),DomainException);

    // Ensure evaluation and composition throw errors when expected
    ValidatedVectorMultivariateTaylorFunctionModelDP vt2={t2};
    ARIADNE_TEST_THROWS(vt2(xe),DomainException);
    ARIADNE_TEST_THROWS(evaluate(vt2,xe),DomainException);

    ValidatedVectorMultivariateTaylorFunctionModelDP te1=t1; te1[0]=te1[0]+FloatDPBounds(-0.125_x,+0.125_x,pr);
    ARIADNE_TEST_THROWS(compose(t2,te1),DomainException);
    ARIADNE_TEST_THROWS(compose(vt2,te1),DomainException);

    ARIADNE_TEST_SAME(unchecked_evaluate(t2,xe),FloatDPBounds(2.25_x,4.25_x,pr));

    // Regression test for printing functions with trivial domain component
    ExactBoxType D4={{1.0_x,1.0_x},{0.0_x,2.0_x}};
    ValidatedScalarMultivariateTaylorFunctionModelDP st40(D4, x0, swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP st41(D4, x1, swp);
    ARIADNE_TEST_PRINT(st40);
    ARIADNE_TEST_PRINT(st41);
    ValidatedVectorMultivariateTaylorFunctionModelDP t4(D4, {x0,x1}, swp);
    ARIADNE_TEST_PRINT(t4);
}


class TestTaylorFunctionFactory
{
  public:
    TestTaylorFunctionFactory();
    Void test();
  private:
    Void test_create();
};

TestTaylorFunctionFactory::TestTaylorFunctionFactory()
{
}

Void TestTaylorFunctionFactory::test()
{
    ARIADNE_TEST_CALL(test_create());
}

Void TestTaylorFunctionFactory::test_create()
{
    DP pr;

    ThresholdSweeper<FloatDP> threshold_sweeper(pr,1e-4);
    Sweeper<FloatDP> sweeper(threshold_sweeper);
    TaylorFunctionFactory factory(sweeper);

    Vector<ExactIntervalType> dom={{-1,+1},{0.5_x,3.5_x}};
    Vector<FloatDPBounds> args=reinterpret_cast<Vector<FloatDPBounds>const&>(dom);

    ValidatedScalarMultivariateTaylorFunctionModelDP stf=factory.create(dom, EffectiveScalarMultivariateFunction::zero(dom.size()) );
    ARIADNE_TEST_PRINT(sweeper);
    ARIADNE_TEST_PRINT(stf);
    ARIADNE_TEST_PRINT(stf.properties());

    ARIADNE_TEST_EQUALS(dynamic_handle_extract<ThresholdSweeper<FloatDP>>(stf.properties()).sweep_threshold(),threshold_sweeper.sweep_threshold());
    ARIADNE_TEST_EQUALS(stf(args),FloatDPBounds(0.0_x,pr));
    ARIADNE_TEST_EQUALS(evaluate(stf,args),FloatDPBounds(0.0_x,pr));

    ValidatedVectorMultivariateTaylorFunctionModelDP vtf=factory.create(dom, EffectiveVectorMultivariateFunction::identity(dom.size()) );
    ARIADNE_TEST_PRINT(vtf);

    // Test evaluation gives a superset with small additional error
    Vector<FloatDPBounds> errs(2,FloatDPBounds(-1e-15_pr,+1e-15_pr,pr));
    ARIADNE_TEST_BINARY_PREDICATE(refines,args,vtf(args));
    ARIADNE_TEST_BINARY_PREDICATE(refines,vtf(args),Vector<FloatDPBounds>(args+errs));
    Vector<FloatDPBounds> pt(2,pr); pt[0]=FloatDPBounds(0.2_dec,pr); pt[1]=FloatDPBounds(1.25_x,pr);
    ARIADNE_TEST_BINARY_PREDICATE(refines,pt,vtf(pt));
}



Int main() {
    ThresholdSweeper<FloatDP> sweeper(dp,std::numeric_limits<float>::epsilon());
    TestScalarTaylorFunction(sweeper).test();
    TestVectorTaylorFunction(sweeper).test();
    TestTaylorFunctionFactory().test();
    std::cerr<<"INCOMPLETE "<<std::flush;
    return ARIADNE_TEST_FAILURES;
}

