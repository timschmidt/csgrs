/***************************************************************************
 *            test_float.cpp
 *
 *  Copyright  2006-20  Alberto Casagrande, Pieter Collins
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
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>

#include "config.hpp"
#include "numeric/builtin.hpp"
#include "numeric/decimal.hpp"
#include "numeric/rational.hpp"
#include "numeric/number.hpp"
#include "numeric/upper_number.hpp"
#include "numeric/lower_number.hpp"
#include "numeric/float.decl.hpp"


#include "numeric/float_ball.hpp"
#include "numeric/float_bounds.hpp"
#include "numeric/float_lower_bound.hpp"
#include "numeric/float_upper_bound.hpp"
#include "numeric/float_error.hpp"

#include "../test.hpp"
#include "test_floats.hpp"

using namespace Ariadne;
using namespace std;


template<class PR>
class TestFloat
{
    using PRE=DoublePrecision;
    typedef RawFloat<PR> RawFloatType;
    typedef FloatBounds<PR> FloatBoundsType;
    typedef FloatBall<PR,PRE> FloatBallType;
    typedef Float<PR> FloatType;
  private:
    PR precision;
  public:
    TestFloat(PR prec) : precision(prec) { }
    Void test();
  private:
    Void test_concept();
    Void test_conversions();
    Void test_operations();
    Void test_predicates();
};

template<class PR> Void
TestFloat<PR>::test()
{
    ARIADNE_TEST_CALL(test_conversions());
    ARIADNE_TEST_CALL(test_operations());
    ARIADNE_TEST_CALL(test_predicates());
}

template<class PR> Void
TestFloat<PR>::test_concept()
{
    Float<DoublePrecision>::set_output_places(17);

    PR pr=precision;
    PRE pre;

    Boolean b;
    Nat m=1u;
    Int n=1;
    Integer z=1;
    Dyadic w=1;
    ExactDouble d(1.0);
    TwoExp t(0);
    RawFloatType f(pr);
    FloatType vx(pr);
    FloatType rx(pr);
    FloatBoundsType rbx(pr);

    // Constructors
    rx=FloatType(m,pr); rx=FloatType(n,pr); rx=FloatType(z,pr); rx=FloatType(w,pr);
    rx=FloatType(d,pr); rx=FloatType(t,pr);
    rx=FloatType(pr); rx=FloatType(f);

    // Assignment
    rx=m; rx=n; rx=z; rx=w; rx=d; rx=t;

    // Arithmetic operators
    rx=operator+(vx); rx=operator-(vx); rx=t*vx; rx=vx*t; rx=vx/t;
    rbx=operator+(vx,vx); rbx=operator-(vx,vx); rbx=operator*(vx,vx); rbx=operator/(vx,vx);

    // Exact operations
    rx=nul(vx); rx=pos(vx); rx=neg(vx); rx=hlf(vx);
    rx=mul(vx,t); rx=div(vx,t);

    FloatBall<PR,PRE> rmx=add(vx,vx,pre); rmx=sub(vx,vx,pre); rmx=mul(vx,vx,pre); rmx=div(vx,vx,pre);

    // Arithmetic
    rbx=add(vx,vx); rbx=sub(vx,vx); rbx=mul(vx,vx); rbx=div(vx,vx);
    rbx=sqr(vx); rbx=rec(vx); rbx=pow(vx,m); rbx=pow(vx,n);

    // Order
    rx=max(vx,vx); rx=min(vx,vx); rx=abs(vx);

    // Comparisons
    b=(vx==vx); b=(vx!=vx); b=(vx<=vx); b=(vx>=vx); b=(vx< vx); b=(vx> vx);
}

template<class PR> Void
TestFloat<PR>::test_conversions()
{
    ARIADNE_TEST_EQUALS(cast_integer(Float<PR>(Dyadic(2),precision)),Integer(2));
    ARIADNE_TEST_FAIL(cast_integer(Float<PR>(Dyadic(7,2u),precision)));
}

template<class PR> Void
TestFloat<PR>::test_operations()
{
    PR pr=precision;
    DoublePrecision pre;
    FloatType vr(pr);

    Nat m(5);
    Int n(-3);
    TwoExp t(-12);
    Dyadic w(-3,1u);
    FloatType vx(w,pr);
    Dyadic w1(-3,1u), w2(5,2u);
    FloatType vx1(w1,pr), vx2(w2,pr);

    ARIADNE_TEST_EQUALS(vx,w);

    ARIADNE_TEST_EQUALS(FloatType(RawFloatType(1.25_x,pr)),1.25_dy);

    ARIADNE_TEST_EQUALS(FloatType(3u,pr),3.0_dy);
    ARIADNE_TEST_EQUALS(FloatType(-5,pr),-5.0_dy);
    ARIADNE_TEST_EQUALS(FloatType(ExactDouble(1.25),pr),1.25_dy);
    ARIADNE_TEST_EQUALS(FloatType(TwoExp(-3),pr),0.125_dy);
    ARIADNE_TEST_EQUALS(FloatType(Integer(-23),pr),-23.0_dy);
    ARIADNE_TEST_EQUALS(FloatType(Dyadic(-23,3u),pr),-2.875_dy);


    ARIADNE_TEST_EQUALS((vr=3u),3.0_dy);
    ARIADNE_TEST_EQUALS((vr=-5),-5.0_dy);
    ARIADNE_TEST_EQUALS((vr=Integer(-23)),-23.0_dy);
    ARIADNE_TEST_EQUALS((vr=TwoExp(-3)),0.125_dy);
    ARIADNE_TEST_EQUALS((vr=Dyadic(-23,3u)),-2.875_dy);

    ARIADNE_TEST_EQUALS(FloatType(w,pr).operator Dyadic(),w);
    ARIADNE_TEST_EQUALS(FloatType(w,pr).operator Rational(),w);

    ARIADNE_TEST_EQUALS(nul(vx),nul(w));
    ARIADNE_TEST_EQUALS(pos(vx),pos(w));
    ARIADNE_TEST_EQUALS(neg(vx),neg(w));
    ARIADNE_TEST_EQUALS(hlf(vx),hlf(w));

    ARIADNE_TEST_EQUALS(mul(vx,t),mul(w,t));
    ARIADNE_TEST_EQUALS(div(vx,t),w/t);

    ARIADNE_TEST_BINARY_PREDICATE(models,sqr(vx),sqr(w));
    ARIADNE_TEST_BINARY_PREDICATE(models,rec(vx),rec(w));

//    ARIADNE_TEST_BINARY_PREDICATE(models,sqr(vx,pre),sqr(w));
//    ARIADNE_TEST_BINARY_PREDICATE(models,rec(vx,pre),rec(w));

    ARIADNE_TEST_EQUALS(max(vx1,vx2),max(w1,w2));
    ARIADNE_TEST_EQUALS(min(vx1,vx2),min(w1,w2));
    ARIADNE_TEST_EQUALS(abs(vx),abs(w));
    ARIADNE_TEST_EQUALS(mig(vx).raw(),abs(w));
    ARIADNE_TEST_EQUALS(mag(vx).raw(),abs(w));
    ARIADNE_TEST_SAME(mag(vx),PositiveFloatUpperBound<PR>(abs(w),pr));
    ARIADNE_TEST_SAME(mig(vx),PositiveFloatLowerBound<PR>(abs(w),pr));

    ARIADNE_TEST_EQUALS(t*vx,t*w);
    ARIADNE_TEST_EQUALS(vx*t,w*t);
    ARIADNE_TEST_EQUALS(vx/t,w/t);

    ARIADNE_TEST_BINARY_PREDICATE(models,vx1+vx2,w1+w2);
    ARIADNE_TEST_BINARY_PREDICATE(models,vx1-vx2,w1-w2);
    ARIADNE_TEST_BINARY_PREDICATE(models,vx1*vx2,w1*w2);
    ARIADNE_TEST_BINARY_PREDICATE(models,vx1/vx2,w1/w2);

    ARIADNE_TEST_BINARY_PREDICATE(models,add(vx1,vx2),add(w1,w2));
    ARIADNE_TEST_BINARY_PREDICATE(models,sub(vx1,vx2),sub(w1,w2));
    ARIADNE_TEST_BINARY_PREDICATE(models,mul(vx1,vx2),mul(w1,w2));
    ARIADNE_TEST_BINARY_PREDICATE(models,div(vx1,vx2),div(w1,w2));

    ARIADNE_TEST_BINARY_PREDICATE(models,add(vx1,vx2,pre),add(w1,w2));
    ARIADNE_TEST_BINARY_PREDICATE(models,sub(vx1,vx2,pre),sub(w1,w2));
    ARIADNE_TEST_BINARY_PREDICATE(models,mul(vx1,vx2,pre),mul(w1,w2));
    ARIADNE_TEST_BINARY_PREDICATE(models,div(vx1,vx2,pre),div(w1,w2));


    ARIADNE_TEST_SAME(add(vx1,vx2,pre),FloatBallType(add(w1,w2),pr,pre));

    ARIADNE_TEST_BINARY_PREDICATE(models,pow(vx,m),pow(w,m));
    ARIADNE_TEST_BINARY_PREDICATE(models,pow(vx,n),pow(Rational(w),n));

//    friend Bounds<F> med(F const& x1, F const& x2);
//    friend Bounds<F> rad(F const& x1, F const& x2);

    ARIADNE_TEST_BINARY_PREDICATE(models,sqr(sqrt(abs(vx))),abs(w));
    ARIADNE_TEST_BINARY_PREDICATE(models,log(exp(vx)),w);
    ARIADNE_TEST_BINARY_PREDICATE(models,exp(log(abs(vx))),abs(w));
    ARIADNE_TEST_BINARY_PREDICATE(models,atan(tan(vx)),w);
    ARIADNE_TEST_BINARY_PREDICATE(models,tan(atan(vx)),w);

    ARIADNE_TEST_SAME(sin(vx),sin(FloatBounds<PR>(vx)));
    ARIADNE_TEST_SAME(cos(vx),cos(FloatBounds<PR>(vx)));
    ARIADNE_TEST_SAME(tan(vx),tan(FloatBounds<PR>(vx)));
    ARIADNE_TEST_SAME(atan(vx),atan(FloatBounds<PR>(vx)));

//    ARIADNE_TEST_SAME(shft(vx,n),shft(w,n));
}

template<class PR> Void
TestFloat<PR>::test_predicates()
{
    PR pr=precision;
    ARIADNE_TEST_BINARY_PREDICATE(eq,Float<PR>(-1,pr),Float<PR>(-1,pr));
    ARIADNE_TEST_BINARY_PREDICATE(not eq,Float<PR>(-1,pr),Float<PR>(-2,pr));
    ARIADNE_TEST_BINARY_PREDICATE(lt,Float<PR>(-2,pr),Float<PR>(-1,pr));
    ARIADNE_TEST_BINARY_PREDICATE(not lt,Float<PR>(-2,pr),Float<PR>(-2,pr));
    ARIADNE_TEST_BINARY_PREDICATE(not lt,Float<PR>(-1,pr),Float<PR>(-2,pr));

    Dyadic w(3,1u);
    Integer zl(1), zu(2);
    Dyadic wl(5,2u), wu(7,2u);
    Rational ql(4,3), qu(5,3);

    ARIADNE_TEST_EQUALS(cmp(Float<PR>(w,pr),w),cmp(w,w));
    ARIADNE_TEST_EQUALS(cmp(Float<PR>(w,pr),zl),cmp(w,zl));
    ARIADNE_TEST_EQUALS(cmp(Float<PR>(w,pr),zu),cmp(w,zu));
    ARIADNE_TEST_EQUALS(cmp(Float<PR>(w,pr),wl),cmp(w,wl));
    ARIADNE_TEST_EQUALS(cmp(Float<PR>(w,pr),wu),cmp(w,wu));
    ARIADNE_TEST_EQUALS(cmp(Float<PR>(w,pr),ql),cmp(w,ql));
    ARIADNE_TEST_EQUALS(cmp(Float<PR>(w,pr),qu),cmp(w,qu));
}


Int main() {
    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);
    
    TestFloat<DoublePrecision>(dp).test();
    TestFloat<MultiplePrecision>(MultiplePrecision(128_bits)).test();

    return ARIADNE_TEST_FAILURES;
}

