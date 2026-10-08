/***************************************************************************
 *            test_number.cpp
 *
 *  Copyright  2016-20  Alberto Casagrande, Pieter Collins
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

#include "numeric/builtin.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/rational.hpp"
#include "numeric/number.hpp"
#include "numeric/upper_number.hpp"
#include "numeric/lower_number.hpp"

#include "numeric/floatdp.hpp"
#include "numeric/float_ball.hpp"
#include "numeric/float_bounds.hpp"
#include "numeric/float_upper_bound.hpp"
#include "numeric/float_lower_bound.hpp"
#include "numeric/float_approximation.hpp"
#include "numeric/float_error.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;


template<class F, class FE> Bool models(Ball<F,FE> const& x, Rational const& q) {
    return abs(Dyadic(x.value().raw())-q)<=Dyadic(x.error().raw()); }
template<class F> Bool models(Bounds<F> const& x, Rational const& q) {
    return Dyadic(x.lower().raw())<=q && q <= Dyadic(x.upper().raw()); }
template<class F> Bool models(LowerBound<F> const& x, Rational const& q) {
    return Dyadic(x.raw())<=q; }
template<class F> Bool models(UpperBound<F> const& x, Rational const& q) {
    return q<=Dyadic(x.raw()); }


class TestNumbers
{
  public:
    Void test();
    Void test_dyadic_behaviour();
    Void test_float_value_behaviour();
    Void test_operations();
    Void test_misc();
};

Void TestNumbers::test()
{
    ARIADNE_TEST_CALL(test_operations());
    ARIADNE_TEST_CALL(test_dyadic_behaviour());
    ARIADNE_TEST_CALL(test_float_value_behaviour());
    ARIADNE_TEST_CALL(test_misc());
}

Void
TestNumbers::test_dyadic_behaviour()
{
    ExactNumber y(Dyadic(1));
    try {
        ExactNumber r = y+y;
        ARIADNE_TEST_NOTIFY("Binary addition on Dyadic within ExactNumber yields "<<r.class_name());
        r = y/y;
        ARIADNE_TEST_NOTIFY("Binary division on Dyadic within ExactNumber yields "<<r.class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary operations on Dyadic give error:\n    "<<e.what());
    }
}

Void
TestNumbers::test_float_value_behaviour()
{
    ExactNumber n(Int(1));
    ExactNumber z(Integer(1));
    ExactNumber w(Dyadic(1));
    ExactNumber q(Rational(1));
    ExactNumber y(FloatDP(1,dp));
    ExactNumber r;
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> within ExactNumber yields "<<(y+y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary division on Float<DP> within ExactNumber yields "<<(y/y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary division on Float<DP> within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary maximum on Float<DP> within ExactNumber yields "<<max(y,y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary maximum on Float<DP> within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Int within ExactNumber yields "<<(y+n).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Int within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Integer within ExactNumber yields "<<(y+z).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Integer within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Dyadic within ExactNumber yields "<<(y+w).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Dyadic within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Rational within ExactNumber yields "<<(y+q).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<DP> and Rational within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Dyadic and Float<DP> within ExactNumber yields "<<(w+y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Dyadic and Float<DP> within ExactNumber gives error:\n    "<<e.what());
    } 
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Rational and Float<DP> within ExactNumber yields "<<(q+y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Rational and Float<DP> within ExactNumber gives error:\n    "<<e.what());
    } 
    y=ExactNumber(FloatMP(1,MP(64)));
    try {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<MP> within ExactNumber yields "<<(y+y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary addition on Float<MP> within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary division on Float<MP> within ExactNumber yields "<<(y/y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary division on Float<MP> within ExactNumber gives error:\n    "<<e.what());
    }
    try {
        ARIADNE_TEST_NOTIFY("Binary maximum on Float<MP> within ExactNumber yields "<<max(y,y).class_name());
    } catch (const DispatchException& e) {
        ARIADNE_TEST_NOTIFY("Binary maximum on Float<MP> within ExactNumber gives error:\n    "<<e.what());
    }
}

Void
TestNumbers::test_operations()
{
    Nat m=1u; Int n=1; Integer z=1; FloatDP v(3,dp); FloatDPBounds b(3,dp);
    ExactNumber yn=n; ExactNumber yz=z; ExactNumber yv=v; ValidatedNumber yb=b;
    ValidatedErrorNumber en=m; ValidatedErrorNumber ev=v;

    Dyadic w2(2);
    Dyadic w3(3);
    Rational q2(2);
    Rational q3(3);
    FloatDP x2(2,dp);
    FloatDP x3(3,dp);
    ExactNumber y2(x2);
    ExactNumber y3(x3);

    ARIADNE_TEST_THROWS(y2/y3, DispatchException);
    ARIADNE_TEST_THROWS(q2/y3, DispatchException);
    ARIADNE_TEST_THROWS(y2/q3, DispatchException);

    ARIADNE_TEST_FAIL(add(yv,yv));
    ARIADNE_TEST_PRINT(add(yb,yb));
    ARIADNE_TEST_PRINT(add(yb,yv));
    ARIADNE_TEST_PRINT(add(b,yb));
    ARIADNE_TEST_PRINT(add(yb,yn));

    ValidatedErrorNumber ym=max(en,ev);
    ARIADNE_TEST_PRINT(max(en,ev));
    ARIADNE_TEST_PRINT(max(en,ev).handle().pointer());
    ARIADNE_TEST_PRINT(max(en,ev).class_name());
    ARIADNE_TEST_PRINT(max(en,ev));

    ARIADNE_TEST_FAIL(add(ExactNumber(1),ExactNumber(FloatDP(2,dp))));
    ARIADNE_TEST_FAIL(add(ExactNumber(FloatDP(1,dp)),ExactNumber(FloatDP(2,dp))));

    ARIADNE_TEST_PRINT(max(ExactNumber(1),ExactNumber(2)));
    ARIADNE_TEST_PRINT(max(ExactNumber(FloatDP(1,dp)),ExactNumber(FloatDP(2,dp))));
    ARIADNE_TEST_PRINT(max(ExactNumber(1),ExactNumber(FloatDP(2,dp))));

    ARIADNE_TEST_PRINT(max(ExactNumber(1),ExactNumber(FloatDP(2,dp))));

    ARIADNE_TEST_PRINT(max(ExactNumber(Dyadic(1)),ExactNumber(FloatDP(2,dp))));
    ARIADNE_TEST_PRINT(max(ExactNumber(FloatDP(1,dp)),ExactNumber(Dyadic(2))));

}


Void
TestNumbers::test_misc()
{
    ARIADNE_TEST_CONSTRUCT(Bounds<FloatDP>,x,(2,3,dp));
//    ARIADNE_TEST_ASSIGN_CONSTRUCT(ValidatedNumber,y,x);
    ARIADNE_TEST_ASSIGN_CONSTRUCT(ValidatedNumber,y,x.operator ValidatedNumber());
    ARIADNE_TEST_CONSTRUCT(ValidatedLowerNumber,yyl,(y));

    ARIADNE_TEST_CONSTRUCT(LowerBound<FloatDP>,xl,(2u,dp));
//    ARIADNE_TEST_ASSIGN_CONSTRUCT(ValidatedLowerNumber,yl,xl);
    ARIADNE_TEST_ASSIGN_CONSTRUCT(ValidatedLowerNumber,yl,xl.operator ValidatedLowerNumber());
    ARIADNE_TEST_ASSIGN(yl,xl+xl);
    ARIADNE_TEST_ASSIGN_CONSTRUCT(ValidatedLowerNumber,z,yl);
    ARIADNE_TEST_ASSIGN(z,yl+yl);
}

template<class Y> class TestNumber
{
  public:
    Void test();
  private:
    Void test_concept();
    Void test_class();
    Void test_get();
    Void test_operations();
    Void test_comparisons();
};



template<class Y> Void
TestNumber<Y>::test()
{
    ARIADNE_TEST_CALL(test_class());
    ARIADNE_TEST_CALL(test_get());
    ARIADNE_TEST_CALL(test_comparisons());
}

template<class Y> Void
TestNumber<Y>::test_concept()
{
    Int n;
    Y y;
    y=Y(2);
    y=3;
    Y y2(1);

    y=y+y; y=y-y; y=y*y; y=y/y;
    y+=y2; y-=y2; y*=y2; y/=y2;
    y=abs(y); y=max(y,y); y=min(y,y);
    y=add(y,y); y=sub(y,y); y=mul(y,y); y=div(y,y);
    y=sqrt(y); y=exp(y); y=log(y); y=atan(y);
    y=pos(y); y=neg(y); y=sqr(y); y=rec(y);
    y=sin(y); y=cos(y); y=tan(y);

    y==y; y!=y; y<=y; y>=y; y<y; y>y;
    y==n; y!=n; y<=n; y>=n; y<n; y>n;
}


template<class Y> void test_number_get() {
    Rational q=3/5_q;
    Real r(q);
    Y y(r);
    MultiplePrecision mp(192);
    MultiplePrecision mpe(128);

    ARIADNE_TEST_WITHIN(ApproximateNumber(y).get(dp),q,Dyadic(TwoExp(-52)));
    ARIADNE_TEST_WITHIN(ApproximateNumber(y).get(mp),q,Dyadic(TwoExp(-128)));

    ARIADNE_TEST_BINARY_PREDICATE(models,ValidatedLowerNumber(y).get(dp),q);
    ARIADNE_TEST_BINARY_PREDICATE(models,ValidatedLowerNumber(y).get(mp),q);

    ARIADNE_TEST_BINARY_PREDICATE(models,ValidatedUpperNumber(y).get(dp),q);
    ARIADNE_TEST_BINARY_PREDICATE(models,ValidatedUpperNumber(y).get(mp),q);

    ARIADNE_TEST_BINARY_PREDICATE(models,y.get(dp),q);
    ARIADNE_TEST_BINARY_PREDICATE(models,y.get(mp),q);

    ARIADNE_TEST_BINARY_PREDICATE(models,y.get(dp,dp),q);
    ARIADNE_TEST_BINARY_PREDICATE(models,y.get(mp,dp),q);
    ARIADNE_TEST_BINARY_PREDICATE(models,y.get(mp,mpe),q);

    ARIADNE_TEST_SAME_TYPE(decltype(y.get(dp,dp)),FloatDPBall);
    ARIADNE_TEST_SAME_TYPE(decltype(y.get(mp,dp)),FloatMPDPBall);
    ARIADNE_TEST_SAME_TYPE(decltype(y.get(mp,mpe)),FloatMPBall);
}

template<> void test_number_get<ExactNumber>() { }

template<> void test_number_get<ApproximateNumber>() {
    Rational q=3/5_q;
    ApproximateNumber y(q);
    MultiplePrecision mp(128);
    ARIADNE_TEST_WITHIN(y.get(dp),q,3e-16);
    ARIADNE_TEST_WITHIN(y.get(mp),q,3e-16);

    // Regression tests for DP/MP conversion.
    Dbl d=-0.6;
    ARIADNE_TEST_ASSIGN(y,d);
    ARIADNE_TEST_EXECUTE(y.get(dp));
    ARIADNE_TEST_EXECUTE(y.get(mp));

    FloatDPApproximation x(-2.5,double_precision);
    ARIADNE_TEST_ASSIGN(y,x);
    ARIADNE_TEST_EXECUTE(y.get(dp));
    ARIADNE_TEST_EXECUTE(y.get(mp));

}

template<class Y> Void
TestNumber<Y>::test_get()
{
    test_number_get<Y>();

}

template<class Y> Void
TestNumber<Y>::test_class()
{
    Int n=1; Integer z=1; FloatDP v(3,dp); FloatDPBounds b(3,dp);
    ExactNumber yn=n; ExactNumber yz=z; ExactNumber yv=v; ValidatedNumber yb=b;

    ARIADNE_TEST_EQUAL(yz.class_name(),"Integer");
    ARIADNE_TEST_EQUAL(yv.class_name(),"FloatDP");
    ARIADNE_TEST_EQUAL(yb.class_name(),"FloatDPBounds");
}


template<class Y> Void
TestNumber<Y>::test_comparisons() {
}

template<> Void
TestNumber<ExactNumber>::test_comparisons() {
    ARIADNE_TEST_CONSTRUCT(ExactNumber,y1,(Rational(2,3)));
    ARIADNE_TEST_CONSTRUCT(ExactNumber,y2,(Rational(683,1024)));
    ARIADNE_TEST_CONSTRUCT(ExactNumber,pinf,(+ExactDouble::inf()));
    ARIADNE_TEST_CONSTRUCT(ExactNumber,ninf,(-ExactDouble::inf()));

    ARIADNE_TEST_BINARY_PREDICATE( operator==,y1,y1);
    ARIADNE_TEST_BINARY_PREDICATE(!operator!=,y1,y1);
    ARIADNE_TEST_BINARY_PREDICATE( operator<=,y1,y1);
    ARIADNE_TEST_BINARY_PREDICATE( operator>=,y1,y1);
    ARIADNE_TEST_BINARY_PREDICATE(!operator< ,y1,y1);
    ARIADNE_TEST_BINARY_PREDICATE(!operator> ,y1,y1);

    ARIADNE_TEST_BINARY_PREDICATE(!operator==,y1,y2);
    ARIADNE_TEST_BINARY_PREDICATE( operator!=,y1,y2);
    ARIADNE_TEST_BINARY_PREDICATE( operator<=,y1,y2);
    ARIADNE_TEST_BINARY_PREDICATE(!operator>=,y1,y2);
    ARIADNE_TEST_BINARY_PREDICATE( operator< ,y1,y2);
    ARIADNE_TEST_BINARY_PREDICATE(!operator> ,y1,y2);

    ARIADNE_TEST_BINARY_PREDICATE(!operator==,y2,y1);
    ARIADNE_TEST_BINARY_PREDICATE( operator!=,y2,y1);
    ARIADNE_TEST_BINARY_PREDICATE(!operator<=,y2,y1);
    ARIADNE_TEST_BINARY_PREDICATE( operator>=,y2,y1);
    ARIADNE_TEST_BINARY_PREDICATE(!operator< ,y2,y1);
    ARIADNE_TEST_BINARY_PREDICATE( operator> ,y2,y1);

    ARIADNE_TEST_BINARY_PREDICATE( operator==,y2,y2);
    ARIADNE_TEST_BINARY_PREDICATE(!operator< ,y2,y2);

    ARIADNE_TEST_BINARY_PREDICATE(operator==,y1,y1);

    ARIADNE_TEST_BINARY_PREDICATE(operator==,pinf,pinf);
    ARIADNE_TEST_BINARY_PREDICATE(operator!=,ninf,pinf);
    ARIADNE_TEST_BINARY_PREDICATE(operator< ,ninf,pinf);
    ARIADNE_TEST_BINARY_PREDICATE(operator!=,y1,pinf);
    ARIADNE_TEST_BINARY_PREDICATE(operator!=,pinf,y1);
    ARIADNE_TEST_BINARY_PREDICATE(operator<,y1,pinf);
    ARIADNE_TEST_BINARY_PREDICATE(operator<,ninf,y1);

    ARIADNE_TEST_ASSERT(definitely(ValidatedUpperNumber(3)<ValidatedLowerNumber(4)));
    ARIADNE_TEST_ASSERT(possibly(ValidatedUpperNumber(4)>ValidatedLowerNumber(3)));
    ARIADNE_TEST_ASSERT(not definitely(ValidatedUpperNumber(4)>ValidatedLowerNumber(3)));

    ARIADNE_TEST_CONSTRUCT(Dyadic,w1,(1.25_x));
    ARIADNE_TEST_CONSTRUCT(Dyadic,w2,(0.875_x));
    ARIADNE_TEST_CONSTRUCT(FloatDP,d1,(w1,dp));
    ARIADNE_TEST_CONSTRUCT(FloatDP,d2,(w2,dp));
    ARIADNE_TEST_CONSTRUCT(ExactNumber,yw1,(w1));
    ARIADNE_TEST_CONSTRUCT(ExactNumber,yw2,(w2));
    ARIADNE_TEST_CONSTRUCT(ExactNumber,yd1,(d1));
    ARIADNE_TEST_CONSTRUCT(ExactNumber,yd2,(d2));

    ARIADNE_TEST_BINARY_PREDICATE(operator==,yw1,yd1);
    ARIADNE_TEST_BINARY_PREDICATE(operator!=,yw1,yd2);
    ARIADNE_TEST_BINARY_PREDICATE(operator> ,yw1,yd2);
    ARIADNE_TEST_BINARY_PREDICATE(operator< ,yw2,yd1);
    ARIADNE_TEST_BINARY_PREDICATE(operator>=,yw1,yd2);
    ARIADNE_TEST_BINARY_PREDICATE(operator<=,yw2,yd1);
    
    ARIADNE_TEST_BINARY_PREDICATE(operator==,yd1,yd1);
    ARIADNE_TEST_BINARY_PREDICATE(operator!=,yd1,yd2);
    ARIADNE_TEST_BINARY_PREDICATE(operator> ,yd1,yd2);
    ARIADNE_TEST_BINARY_PREDICATE(operator< ,yd2,yd1);
    ARIADNE_TEST_BINARY_PREDICATE(operator>=,yd1,yd2);
    ARIADNE_TEST_BINARY_PREDICATE(operator<=,yd2,yd1);
    
}

template<> Void
TestNumber<EffectiveNumber>::test_comparisons() {
    ARIADNE_TEST_ASSERT(definitely(EffectiveNumber(Real(-inf))<EffectiveNumber(Real(0))));
    ARIADNE_TEST_ASSERT(definitely(EffectiveNumber(Real(0))>EffectiveNumber(Real(-inf))));
    ARIADNE_TEST_ASSERT(not possibly(EffectiveNumber(Real(-inf))>EffectiveNumber(Real(0))));
    ARIADNE_TEST_ASSERT(not possibly(EffectiveNumber(Real(0))<EffectiveNumber(Real(-inf))));
    ARIADNE_TEST_ASSERT(definitely(not (EffectiveNumber(Real(-inf))>EffectiveNumber(Real(0)))));
    ARIADNE_TEST_ASSERT(definitely(not (EffectiveNumber(Real(0))<EffectiveNumber(Real(-inf)))));
    ARIADNE_TEST_ASSERT(definitely(EffectiveNumber(Real(-inf))<=EffectiveNumber(Real(0))));

    ARIADNE_TEST_CONSTRUCT(Real,r1,(1/3_q));
    ARIADNE_TEST_CONSTRUCT(Real,r2,(ExactDouble(0.333333333333333333)));
    ARIADNE_TEST_ASSERT(not definitely((r1>r2).check(Effort(0))));
    ARIADNE_TEST_ASSERT(possibly(not (r1>r2).check(Effort(0))));
    ARIADNE_TEST_ASSERT(definitely((r1>r2).check(Effort(64))));

    ARIADNE_TEST_CONSTRUCT(EffectiveNumber,y1,(r1));
    ARIADNE_TEST_CONSTRUCT(EffectiveNumber,y2,(r2));
    ARIADNE_TEST_ASSERT(not definitely((y1>y2).check(Effort(0))));
    ARIADNE_TEST_ASSERT(possibly(not (y1>y2).check(Effort(0))));
    ARIADNE_TEST_ASSERT(definitely((y1>y2).check(Effort(64))));
}

template<class Y> class TestDirectedNumber
{
  public:
    Void test();
  private:
    Void test_concept();
    Void test_operations();
};

template<class Y> Void
TestDirectedNumber<Y>::test() {
    test_operations();
}

template<class Y> Void
TestDirectedNumber<Y>::test_concept() {
    static_assert(Same<NegationType<NegationType<Y>>,Y>);
    typedef NegationType<Y> NY;
    Y y; NY ny;
    y=+y; ny=-y; y=-ny;
    y=y+y; y=y-ny; y+=y; y+=ny;
    y=add(y,y); y=sub(y,ny); ny=sub(ny,y);
    y=sqrt(y); y=exp(y); y=log(y); y=tan(y);

    y==ny; y!=ny; y<ny; y>ny;
    ny==y; ny!=y; ny<y; ny>y;
}


template<class Y> Void
TestDirectedNumber<Y>::test_operations() {
    if constexpr (Same<Y,ValidatedLowerNumber>) {
        typedef DoublePrecision PR;
        PR pr;
        Rational q(1,3);
        FloatBounds<PR> x(q,pr);
        FloatLowerBound<PR> xl=x;
        FloatUpperBound<PR> xu=x*7/8;

        ValidatedNumber y(x);
        ValidatedLowerNumber yl(xl);
        ValidatedUpperNumber yu(xu);

        // Tests
        ARIADNE_TEST_ASSERT(not possibly (yu == yl));
        ARIADNE_TEST_ASSERT(definitely (yu != yl));
        ARIADNE_TEST_ASSERT(not definitely (yl > q));
        ARIADNE_TEST_ASSERT(not definitely (-yl < -q));
        ARIADNE_TEST_ASSERT(not definitely (yl+yl+yl > 1));
        ARIADNE_TEST_ASSERT(not definitely (yl+yl+yl > 1));
        ARIADNE_TEST_ASSERT(definitely (yu < yl));
        ARIADNE_TEST_ASSERT(definitely (yl > yu));

        ARIADNE_TEST_ASSERT(not definitely (yl > q));

        static_assert(Same<DifferenceType<ExactNumber,ValidatedUpperNumber>,ValidatedLowerNumber>);
        static_assert(Same<DifferenceType<ValidatedLowerNumber,ValidatedUpperNumber>,ValidatedLowerNumber>);
        static_assert(Same<DifferenceType<ValidatedLowerNumber,ValidatedNumber>,ValidatedLowerNumber>);

        static_assert(Same<DifferenceType<Integer,FloatDPUpperBound>,FloatDPLowerBound>);
        static_assert(Same<DifferenceType<Dyadic,FloatDPUpperBound>,FloatDPLowerBound>);
        static_assert(Same<DifferenceType<FloatDPUpperBound,Integer>,FloatDPUpperBound>);
        static_assert(Same<DifferenceType<FloatDPUpperBound,Dyadic>,FloatDPUpperBound>);

        static_assert(Same<decltype(Add().operator()(xu,xu)),FloatDPUpperBound>);
        static_assert(Same<decltype(Sub().operator()(xu,xl)),FloatDPUpperBound>);
        static_assert(Same<decltype(BinaryElementaryOperator(Add()).operator()(xu,xu)),FloatDPApproximation>);
        static_assert(Same<decltype(BinaryElementaryOperator(Sub()).operator()(xu,xl)),FloatDPApproximation>);

        ARIADNE_TEST_NOTIFY("BinaryElementaryOperator returns common arithmetical type, which may differ from type of actual operation.");

        Integer z(5);
        Dyadic w(1.375_x);
        ARIADNE_TEST_PRINT(w<yu);
        ARIADNE_TEST_PRINT(ExactNumber(w).class_name());
        ARIADNE_TEST_PRINT(ValidatedLowerNumber(w).class_name());

        ARIADNE_TEST_EQUALS(ExactNumber(w).class_name(),"Dyadic");
        ARIADNE_TEST_EQUALS((yl+yl).class_name(),"FloatDPLowerBound");
        ARIADNE_TEST_EQUALS((yl-yu).class_name(),"FloatDPLowerBound");
        ARIADNE_TEST_EQUALS((yu+yu).class_name(),"FloatDPUpperBound");
        ARIADNE_TEST_EQUALS((yu-yl).class_name(),"FloatDPUpperBound");
        ARIADNE_TEST_EQUALS((ExactNumber(z)-yu).class_name(),"FloatDPLowerBound");

        ARIADNE_TEST_ASSERT(not possibly(w<yu));

    }
}


Int main() {
    TestNumbers().test();

    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);

    TestNumber<ApproximateNumber>().test();
    TestNumber<ValidatedNumber>().test();
    TestNumber<EffectiveNumber>().test();
    TestNumber<ExactNumber>().test();

    TestDirectedNumber<ValidatedLowerNumber>().test();
    TestDirectedNumber<ValidatedUpperNumber>().test();
    TestDirectedNumber<EffectiveLowerNumber>().test();
    TestDirectedNumber<EffectiveUpperNumber>().test();

    std::cerr<<"INCOMPLETE ";
    return ARIADNE_TEST_FAILURES;
//    TestNumber<ValidatedUpperNumber>().test();
//    TestNumber<ValidatedLowerNumber>().test();
}

