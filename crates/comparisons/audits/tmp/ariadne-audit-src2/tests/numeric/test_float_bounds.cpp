/***************************************************************************
 *            test_float_bounds.cpp
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
#include "numeric/float.decl.hpp"

#include "numeric/float_bounds.hpp"

#include "numeric/float_lower_bound.hpp"
#include "numeric/float_upper_bound.hpp"
#include "numeric/float_error.hpp"

#include "../test.hpp"
#include "test_floats.hpp"

using namespace Ariadne;
using namespace std;

template<class PR>
class TestFloatBounds
{
    typedef RawFloat<PR> RawFloatType;
    typedef FloatType<BoundedTag,PR> FloatBoundsType;
    typedef FloatType<ExactTag,PR> FloatValueType;
  private:
    PR pr;
  public:
    TestFloatBounds(PR prec) : pr(prec) { }
    Void test();
  private:
    inline FloatBoundsType make_float_bounds(ExactDouble l, ExactDouble u) { return FloatBoundsType(l,u,pr); }

    Void test_concept();
    Void test_constructors();
    Void test_conversions();
    Void test_input();
    Void test_class();
    Void test_comparison();
    Void test_precision();
    Void test_correct_rounded_arithmetic();
    Void test_accurate_rounded_arithmetic();
    Void test_exact_rounded_arithmetic();
    Void test_aliasing();
    Void test_monotone_functions();
    Void test_trigonometric_functions();
    Void regression_tests();
};


template<class PR> Void
TestFloatBounds<PR>::test()
{
    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_input());
    ARIADNE_TEST_CALL(test_class());
    ARIADNE_TEST_CALL(test_conversions());
    ARIADNE_TEST_CALL(test_comparison());
    ARIADNE_TEST_CALL(test_precision());
    ARIADNE_TEST_CALL(test_correct_rounded_arithmetic());
    ARIADNE_TEST_CALL(test_accurate_rounded_arithmetic());
    ARIADNE_TEST_CALL(test_exact_rounded_arithmetic());
    ARIADNE_TEST_CALL(test_monotone_functions());
    ARIADNE_TEST_CALL(test_trigonometric_functions());
    ARIADNE_TEST_CALL(regression_tests());
}

template<class PR> Void
TestFloatBounds<PR>::test_concept()
{
    FloatBoundsType::set_output_places(17);

    Nat m=1;
    Int n=1;
    double d=1;
    FloatValueType x(1);
    RawFloatType a,b;
    FloatBoundsType vx(1);
    FloatBoundsType rx(1);

    // Constructors
    rx=FloatBoundsType(); rx=FloatBoundsType(n); rx=FloatBoundsType(m); rx=FloatBoundsType(d); rx=FloatBoundsType(x); rx=FloatBoundsType(vx);
    rx=FloatBoundsType(n,n); rx=FloatBoundsType(m,m); rx=FloatBoundsType(d,d); rx=FloatBoundsType(a,b);
    rx=FloatBoundsType(n,m); rx=FloatBoundsType(m,d); rx=FloatBoundsType(d,n);
//
    // Assignment
    rx=n; rx=m; rx=x; rx=vx;

    // ExactTag operations
    rx=nul(vx); rx=pos(vx); rx=neg(vx); rx=hlf(vx); rx=sqr(vx); rx=rec(vx);

    rx=operator+(x,x); rx=operator+(x,vx); rx=operator+(vx,x); rx=operator+(vx,vx);
    rx=operator-(x,x); rx=operator-(x,vx); rx=operator-(vx,x); rx=operator-(vx,vx);
    rx=operator*(x,x); rx=operator*(x,vx); rx=operator*(vx,x); rx=operator*(vx,vx);
    rx=operator/(x,x); rx=operator/(x,vx); rx=operator/(vx,x); rx=operator/(vx,vx);

    // Arithmetic
    rx=add(x,x); rx=add(vx,vx);
    rx=sub(x,x); rx=sub(vx,vx);
    rx=mul(x,x); rx=mul(vx,vx);
    rx=div(x,x); rx=div(vx,vx);
    rx=pow(x,m); rx=pow(x,m);
    rx=pow(x,n); rx=pow(x,n);

    // Order
    rx=max(vx,vx); rx=min(vx,vx); rx=abs(vx);

    // Transcendental functions
    rx=sqrt(vx);
    rx=exp(vx);
    rx=log(vx);
    rx=sin(vx);
    rx=cos(vx);
    rx=tan(vx);
    rx=atan(vx);
}

template<class PR> Void
TestFloatBounds<PR>::test_precision()
{

    FloatBoundsType bx(Rational(1),pr);
    ARIADNE_TEST_EQUALS(bx.lower().precision(),pr);
    ARIADNE_TEST_EQUALS(bx.upper().precision(),pr);
    ARIADNE_TEST_EQUALS(max(bx,bx).precision(),pr);
    ARIADNE_TEST_EQUALS(min(bx,bx).precision(),pr);
    ARIADNE_TEST_EQUALS(abs(bx).precision(),pr);
    ARIADNE_TEST_EQUALS((bx+bx).precision(),pr);
    ARIADNE_TEST_EQUALS((bx-bx).precision(),pr);
    ARIADNE_TEST_EQUALS((bx*bx).precision(),pr);
    ARIADNE_TEST_EQUALS((bx/bx).precision(),pr);
    ARIADNE_TEST_EQUALS(sqrt(bx).precision(),pr);
    ARIADNE_TEST_EQUALS(exp(bx).precision(),pr);
    ARIADNE_TEST_EQUALS(log(bx).precision(),pr);
    ARIADNE_TEST_EQUALS(sin(bx).precision(),pr);
    ARIADNE_TEST_EQUALS(cos(bx).precision(),pr);
    ARIADNE_TEST_EQUALS(tan(bx).precision(),pr);
    ARIADNE_TEST_EQUALS(atan(bx).precision(),pr);
    ARIADNE_TEST_EQUALS((bx+2u).precision(),pr);
    ARIADNE_TEST_EQUALS((bx-2u).precision(),pr);
    ARIADNE_TEST_EQUALS((bx*2u).precision(),pr);
    ARIADNE_TEST_EQUALS((bx/2u).precision(),pr);
    ARIADNE_TEST_EQUALS((bx+2).precision(),pr);
    ARIADNE_TEST_EQUALS((bx-2).precision(),pr);
    ARIADNE_TEST_EQUALS((bx*2).precision(),pr);
    ARIADNE_TEST_EQUALS((bx/2).precision(),pr);

    ARIADNE_TEST_EQUALS(abs(FloatBoundsType(-1,2,pr)).lower().precision(),pr);

}

template<class PR> Void
TestFloatBounds<PR>::test_conversions()
{
    ARIADNE_TEST_EQUALS(cast_integer(FloatBounds<PR>(Dyadic(2),pr)),Integer(2));
    ARIADNE_TEST_EQUALS(cast_integer(FloatBounds<PR>(Dyadic(1),Dyadic(2),pr)),Integer(2));
    ARIADNE_TEST_EQUALS(cast_integer(FloatBounds<PR>(Dyadic(5,2u),Dyadic(2),pr)),Integer(2));
    ARIADNE_TEST_EQUALS(cast_integer(FloatBounds<PR>(Dyadic(7,2u),Dyadic(9,2u),pr)),Integer(2));
    ARIADNE_TEST_FAIL(cast_integer(FloatBounds<PR>(Dyadic(9,2u),Dyadic(11,2u),pr)));
}



// Test that interval arithmetic is rounded correctly,
// without paying attention to accuracy issues.
template<class PR> Void
TestFloatBounds<PR>::test_correct_rounded_arithmetic()
{
    FloatBoundsType onethird=FloatBoundsType(1,pr)/FloatBoundsType(3,pr);
    ARIADNE_TEST_COMPARE( onethird.lower_raw(), < , onethird.upper_raw() );
    FloatBoundsType one_approx=onethird*FloatBoundsType(3,pr);
    ARIADNE_TEST_COMPARE( one_approx.lower_raw(), < , 1.0_x );
    ARIADNE_TEST_COMPARE( one_approx.upper_raw(), > , 1.0_x );
}


// Test that interval arithmetic gives the most accurate rounded values
template<class PR> Void
TestFloatBounds<PR>::test_accurate_rounded_arithmetic()
{
    if constexpr (Same<PR,DoublePrecision>) {
        ARIADNE_TEST_SAME(FloatBoundsType(1,pr)/FloatBoundsType(3,pr),FloatBoundsType(0.33333333333333331_pr,0.33333333333333337_pr,pr));
        ARIADNE_TEST_SAME(FloatBoundsType(2,pr)/FloatBoundsType(5,pr),FloatBoundsType(0.39999999999999997_pr,0.40000000000000002_pr,pr));

        ARIADNE_TEST_SAME(pow(FloatBoundsType(3,5,pr),-1),FloatBoundsType(0.19999999999999998_pr,0.33333333333333337_pr,pr));
        ARIADNE_TEST_SAME(pow(FloatBoundsType(3,5,pr),-2),FloatBoundsType(0.039999999999999986955_pr,0.11111111111111114658_pr,pr));
    }
}


// Test that interval arithmetic gives exact values if possible
template<class PR> Void
TestFloatBounds<PR>::test_exact_rounded_arithmetic()
{
    ARIADNE_TEST_SAME(FloatBoundsType(5,7,pr)+FloatBoundsType(2,4,pr),FloatBoundsType(7,11,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(5,7,pr)-FloatBoundsType(2,6,pr),FloatBoundsType(-1,5,pr));

    ARIADNE_TEST_SAME(FloatBoundsType(5,7,pr)*FloatBoundsType(2,4,pr),FloatBoundsType(10,28,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(5,7,pr)*FloatBoundsType(-2,4,pr),FloatBoundsType(-14,28,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(5,7,pr)*FloatBoundsType(-4,-2,pr),FloatBoundsType(-28,-10,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,5,pr)*FloatBoundsType(2,4,pr),FloatBoundsType(-28,20,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,5,pr)*FloatBoundsType(-2,4,pr),FloatBoundsType(-28,20,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,5,pr)*FloatBoundsType(-4,-2,pr),FloatBoundsType(-20,28,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,-5,pr)*FloatBoundsType(2,4,pr),FloatBoundsType(-28,-10,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,-5,pr)*FloatBoundsType(-2,4,pr),FloatBoundsType(-28,14,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,-5,pr)*FloatBoundsType(-4,-2,pr),FloatBoundsType(10,28,pr));

    ARIADNE_TEST_SAME(FloatBoundsType(5,7,pr)/FloatBoundsType(2,4,pr),FloatBoundsType(1.25_x,3.50_x,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(5,7,pr)/FloatBoundsType(-4,-2,pr),FloatBoundsType(-3.50_x,-1.25_x,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,5,pr)/FloatBoundsType(2,4,pr),FloatBoundsType(-3.50_x,2.50_x,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,5,pr)/FloatBoundsType(-4,-2,pr),FloatBoundsType(-2.50_x,3.5_x,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,-5,pr)/FloatBoundsType(2,4,pr),FloatBoundsType(-3.50_x,-1.25_x,pr));
    ARIADNE_TEST_SAME(FloatBoundsType(-7,-5,pr)/FloatBoundsType(-4,-2,pr),FloatBoundsType(1.25_x,3.50_x,pr));

    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),0u),FloatBoundsType(1,1,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),0u),FloatBoundsType(1,1,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),0u),FloatBoundsType(1,1,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),0u),FloatBoundsType(1,1,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),1u),FloatBoundsType(5,7,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),1u),FloatBoundsType(-5,7,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),1u),FloatBoundsType(-7,5,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),1u),FloatBoundsType(-7,-5,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),2u),FloatBoundsType(25,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),2u),FloatBoundsType(0,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),2u),FloatBoundsType(0,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),2u),FloatBoundsType(25,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),3u),FloatBoundsType(125,343,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),3u),FloatBoundsType(-125,343,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),3u),FloatBoundsType(-343,125,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),3u),FloatBoundsType(-343,-125,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),4u),FloatBoundsType(625,2401,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),4u),FloatBoundsType(0,2401,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),4u),FloatBoundsType(0,2401,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),4u),FloatBoundsType(625,2401,pr));

    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),0),FloatBoundsType(1,1,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),0),FloatBoundsType(1,1,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),0),FloatBoundsType(1,1,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),1),FloatBoundsType(5,7,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),1),FloatBoundsType(-5,7,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),1),FloatBoundsType(-7,5,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),1),FloatBoundsType(-7,-5,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),2),FloatBoundsType(25,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),2),FloatBoundsType(0,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),2),FloatBoundsType(0,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),2),FloatBoundsType(25,49,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),3),FloatBoundsType(125,343,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),3),FloatBoundsType(-125,343,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),3),FloatBoundsType(-343,125,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),3),FloatBoundsType(-343,-125,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),4),FloatBoundsType(625,2401,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),4),FloatBoundsType(0,2401,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),4),FloatBoundsType(0,2401,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),4),FloatBoundsType(625,2401,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),5),FloatBoundsType(3125,16807,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),5),FloatBoundsType(-3125,16807,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),5),FloatBoundsType(-16807,3125,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),5),FloatBoundsType(-16807,-3125,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(5,7,pr),7),FloatBoundsType(78125,823543,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-5,7,pr),7),FloatBoundsType(-78125,823543,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,5,pr),7),FloatBoundsType(-823543,78125,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-7,-5,pr),7),FloatBoundsType(-823543,-78125,pr));

    ARIADNE_TEST_SAME(pow(FloatBoundsType(2,4,pr),-1),FloatBoundsType(0.25_x,0.5_x,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-4,-2,pr),-1),FloatBoundsType(-0.5_x,-0.25_x,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(2,4,pr),-2),FloatBoundsType(0.0625_x,0.25_x,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-4,-2,pr),-2),FloatBoundsType(0.0625_x,0.25_x,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(2,4,pr),-3),FloatBoundsType(0.015625_x,0.125_x,pr));
    ARIADNE_TEST_SAME(pow(FloatBoundsType(-4,-2,pr),-3),FloatBoundsType(-0.125_x,-0.015625_x,pr));

    ARIADNE_TEST_SAME(rec(FloatBoundsType(2,4,pr)),FloatBoundsType(0.25_x,0.50_x,pr));
    ARIADNE_TEST_SAME(rec(FloatBoundsType(-4,-2,pr)),FloatBoundsType(-0.50_x,-0.25_x,pr));
}



template<> Void
TestFloatBounds<DoublePrecision>::test_constructors()
{
    FloatDP zero(0,pr);

    // Construct from pair
    FloatBoundsType xd1(FloatDP(1.125_x,pr),FloatDP(2.25_x,pr));
    ARIADNE_TEST_ASSERT(xd1.lower_raw()==1.125_x); ARIADNE_TEST_ASSERT(xd1.upper_raw()==2.25_x);

    // Default constructor
    FloatBoundsType xd2(pr);
    if(xd2.lower_raw()>xd2.upper_raw()) {
        ARIADNE_TEST_WARN("FloatBoundsType default constructor returns an empty set.");
    } else {
        ARIADNE_TEST_BINARY_PREDICATE(same,xd2,FloatBoundsType(zero,zero));
    }

    // Constructor from Rational with approximations
    FloatBoundsType xd3(Rational(21,10),pr);
    ARIADNE_TEST_COMPARE(Rational(xd3.lower_raw()),<,Rational(21,10));
    ARIADNE_TEST_COMPARE(Rational(xd3.upper_raw()),>,Rational(21,10));

    // Constructor from rational bounds
    FloatBoundsType xd4(2.1_q,3.2_q,pr);
    ARIADNE_TEST_COMPARE(xd4.lower_raw(),<=,2.1_pr);
    ARIADNE_TEST_COMPARE(xd4.upper_raw(),>=,3.2_pr);

    // Approximate constructor from a single value
    FloatBoundsType xd5(Rational(1,3),pr);
    ARIADNE_TEST_COMPARE(Rational(xd5.lower_raw()),<,Rational(1,3));
    ARIADNE_TEST_COMPARE(Rational(xd5.upper_raw()),>,Rational(1,3));

    // ExactTag constructor from a single value
    FloatBoundsType xd6(FloatDP(1.25_x,pr));
    ARIADNE_TEST_EQUAL(xd6.lower_raw(),FloatDP(1.25_x,pr));
    ARIADNE_TEST_EQUAL(xd6.upper_raw(),FloatDP(1.25_x,pr));
}

template<class PR> Void
TestFloatBounds<PR>::test_constructors()
{
    RawFloatType zero(0,pr);

    // Construct from pair
    FloatBoundsType xd1(RawFloatType(1.125_x,pr),RawFloatType(2.25_x,pr));
    ARIADNE_TEST_ASSERT(xd1.lower_raw()==1.125_x); ARIADNE_TEST_ASSERT(xd1.upper_raw()==2.25_x);

    // Default value constructor
    FloatBoundsType xd2(pr);
    if(xd2.lower_raw()>xd2.upper_raw()) {
        ARIADNE_TEST_WARN("FloatBoundsType default constructor returns an empty set.");
    } else {
        ARIADNE_TEST_BINARY_PREDICATE(same,xd2,FloatBoundsType(zero,zero));
    }

    // Constructor with approximations
    FloatBoundsType xd3(Rational(21,10),pr);
    ARIADNE_TEST_COMPARE(Rational(xd3.lower_raw()),<,Rational(21,10));
    ARIADNE_TEST_COMPARE(Rational(xd3.upper_raw()),>,Rational(21,10));

    // Constructor from approximate values
    FloatBoundsType xd4(2.1_q,3.2_q,pr);
    ARIADNE_TEST_COMPARE(xd4.lower_raw(),<=,Rational(21,10));
    ARIADNE_TEST_COMPARE(xd4.upper_raw(),>=,Rational(32,10));

    // ApproximateTag constructor from a single value
    FloatBoundsType xd5(Rational(1,3),pr);
    ARIADNE_TEST_COMPARE(xd5.lower_raw(),<,Rational(1,3));
    ARIADNE_TEST_COMPARE(xd5.upper_raw(),>,Rational(1,3));

    // ExactTag constructor from a single value
    FloatBoundsType xd6(RawFloatType(1.25_x,pr));
    ARIADNE_TEST_EQUAL(xd6.lower_raw(),RawFloatType(1.25_x,pr));
    ARIADNE_TEST_EQUAL(xd6.upper_raw(),RawFloatType(1.25_x,pr));
}

template<class PR> Void TestFloatBounds<PR>::test_class()
{
    // Test lower, upper, midpoint, radius, width
    RawFloatType one_(1.0_x,pr);
    RawFloatType two_(2.0_x,pr);
    RawFloatType three_(3.0_x,pr);

    // Tests for exact operations
    ARIADNE_TEST_EQUAL(FloatBoundsType(-0.25_x,0.50_x,pr).lower().raw(),-0.25_x);
    ARIADNE_TEST_EQUAL(FloatBoundsType(-0.25_x,0.50_x,pr).upper().raw(),0.5_x);
    ARIADNE_TEST_EQUAL(FloatBoundsType(-0.25_x,0.50_x,pr).value(),0.125_x);
    ARIADNE_TEST_EQUAL(FloatBoundsType(-0.25_x,0.50_x,pr).error().raw(),0.375_x)

    // Tests for inexact operations
    ARIADNE_TEST_EQUAL((FloatBoundsType(-1,2,pr)/3).lower().raw(),div(down,-one_,three_));
    ARIADNE_TEST_EQUAL((FloatBoundsType(-1,2,pr)/3).upper().raw(),div(up,two_,three_));
    ARIADNE_TEST_EQUAL((FloatBoundsType(-1,2,pr)/3).value(),hlf(add(up,div(down,-one_,three_),div(up,two_,three_))))
    ARIADNE_TEST_EQUAL((FloatBoundsType(-1,2,pr)/3).error().raw(),hlf(sub(up,div(up,two_,three_),div(down,-one_,three_))));
}

template<class PR> Void TestFloatBounds<PR>::test_input()
{
    FloatBoundsType x(pr);

    string input("[1.125,2.75] [0.4,0.6]");
    stringstream iss(input);

    iss >> x;
    ARIADNE_TEST_EQUALS(x.lower_raw(),Rational(9,8))
    ARIADNE_TEST_EQUALS(x.upper_raw(),Rational(11,4))

    iss >> x;
    // TODO: Make stream input produce rounded values
    //ARIADNE_TEST_COMPARE(x.lower_raw(),<,Rational(2,5))
    // ARIADNE_TEST_COMPARE(x.upper_raw(),>,Rational(3,5))
    if(not(x.lower_raw()<=Rational(2,5) and x.upper_raw()>=Rational(3,5))) {
        ARIADNE_TEST_WARN("FloatType<BoundedTag,"<<class_name<PR>()<<"> string constructor returns an approximate interval, not an outwardly rounded interval.");
    }
}

template<class PR> Void TestFloatBounds<PR>::test_comparison() {
    // FIXME: If using Boost style interval tests, uncomment the line below
    // and comment out the line after
    //ARIADNE_TEST_ASSERT(indeterminate(ivld1==ivld2));
    FloatBoundsType ivl1(1.125_x,2.25_x,pr);
    FloatBoundsType ivl2=ivl1;

    ARIADNE_TEST_ASSERT(!definitely(ivl1==ivl2));
    ARIADNE_TEST_ASSERT(possibly(ivl1==ivl2));
    FloatBoundsType& ivl1ref=ivl1;
    ivl1ref=FloatBoundsType(5.25_x,7.375_x,pr);
    cout << "ivl1ref=" << ivl1ref << endl;
    ARIADNE_TEST_ASSERT(ivl1ref.lower_raw()==5.25_x);
}

template<class PR> Void TestFloatBounds<PR>::test_aliasing() {
    FloatValueType ex2(1.5_x,pr);
    FloatValueType ex3(2.25_x,pr);

    FloatBoundsType vx1;
    FloatBoundsType vx2(1.5_x,2.25_x,pr);
    FloatBoundsType vx3(3.125_x,4.0625_x,pr);

    // Check to make sure aliases are handled correctly
    vx1=vx3; vx1=vx2-vx1; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(vx2-vx3));
    vx1=vx3; vx1=vx2*vx1; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(vx2*vx3));
    vx1=vx2; vx1=vx1*vx3; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(vx2*vx3));
    vx1=vx2; vx1=vx1*ex3; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(vx2*ex3));
    vx1=vx3; vx1=ex2*vx1; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(ex2*vx3));
    vx1=vx2; vx1=vx1/vx3; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(vx2/vx3));
    vx1=vx2; vx1=vx1/ex3; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(vx2/ex3));
    vx1=vx3; vx1=ex2/vx1; ARIADNE_TEST_BINARY_PREDICATE(same,vx1,FloatBoundsType(ex2/vx3));
}

template<class PR> Void TestFloatBounds<PR>::test_monotone_functions()
{
    FloatBoundsType two_(2.0_x,pr);
    FloatBoundsType sqrt_two=sqrt(two_);
    FloatBoundsType sqr_sqrt_two=sqr(sqrt_two);
    ARIADNE_TEST_PRINT(sqrt_two);
    ARIADNE_TEST_PRINT(sqr_sqrt_two);
    if (Same<PR,DoublePrecision>) {
        ARIADNE_TEST_EQUAL(sqrt_two.lower_raw(),1.4142135623730949_pr);
        ARIADNE_TEST_EQUAL(sqrt_two.upper_raw(),1.4142135623730951_pr);
        ARIADNE_TEST_COMPARE(sqrt_two.lower_raw(),<=,1.4142135623730949_pr);
        ARIADNE_TEST_COMPARE(sqrt_two.lower_raw(),> ,1.4142135623730947_pr);
        ARIADNE_TEST_COMPARE(sqrt_two.upper_raw(),>=,1.4142135623730951_pr);
        ARIADNE_TEST_COMPARE(sqrt_two.upper_raw(),< ,1.4142135623730954_pr);
    }
    ARIADNE_TEST_EQUALS(sqrt_two.lower_raw(),sqrt(down,RawFloatType(2.0_x,pr)));
    ARIADNE_TEST_EQUALS(sqrt_two.upper_raw(),sqrt(up,RawFloatType(2.0_x,pr)));
    ARIADNE_TEST_ASSERT(sqrt_two.lower_raw()<sqrt_two.upper_raw());
    ARIADNE_TEST_COMPARE(sqr_sqrt_two.lower_raw(),< ,2.0_x);
    ARIADNE_TEST_COMPARE(sqr_sqrt_two.upper_raw(),> ,2.0_x);

    FloatBoundsType one(1.0_x,pr);
    FloatBoundsType exp_one=exp(one);
    FloatBoundsType log_exp_one=log(exp_one);
    ARIADNE_TEST_PRINT(exp_one);
    ARIADNE_TEST_PRINT(log_exp_one);
    if (Same<PR,DoublePrecision>) {
        ARIADNE_TEST_COMPARE(exp_one.lower_raw(),<,2.71828182845905_pr);
        ARIADNE_TEST_COMPARE(exp_one.lower_raw(),>,2.71828182845903_pr);
        ARIADNE_TEST_COMPARE(exp_one.upper_raw(),>,2.71828182845904_pr);
        ARIADNE_TEST_COMPARE(exp_one.upper_raw(),<,2.71828182845906_pr);
    }
    ARIADNE_TEST_EQUALS(exp_one.lower_raw(),exp(down,RawFloatType(1.0_x,pr)));
    ARIADNE_TEST_EQUALS(exp_one.upper_raw(),exp(up,RawFloatType(1.0_x,pr)));
    ARIADNE_TEST_ASSERT(exp_one.lower_raw()<exp_one.upper_raw());
    ARIADNE_TEST_COMPARE(log_exp_one.lower_raw(),<,1.0_x);
    ARIADNE_TEST_COMPARE(log_exp_one.lower_raw(),<,1.0_x);

    if (Same<PR,DoublePrecision>) {
        FloatBoundsType e(2.7182818284590451_pr,2.7182818284590455_pr,pr);
        FloatBoundsType loge=log(e);
        ARIADNE_TEST_PRINT(e);
        ARIADNE_TEST_COMPARE(loge.lower_raw(),<,1.0_x);
        ARIADNE_TEST_COMPARE(loge.lower_raw(),>,0.9999999999998_pr);
        ARIADNE_TEST_COMPARE(loge.upper_raw(),>,1.0_x);
        ARIADNE_TEST_COMPARE(loge.upper_raw(),<,1.000000000002_pr);
    }

}

template<class PR> Void TestFloatBounds<PR>::test_trigonometric_functions()
{
    try {
        FloatBoundsType x(6.283185307179586_pr,6.283185307179587_pr,pr);
        FloatBoundsType sinx=sin(x);
        ARIADNE_TEST_PRINT(x);
        ARIADNE_TEST_COMPARE(sinx.lower_raw(),<,0.0_x);
        ARIADNE_TEST_COMPARE(sinx.lower_raw(),>,-1e-14_pr);
        ARIADNE_TEST_COMPARE(sinx.upper_raw(),>,0.0_x);
        ARIADNE_TEST_COMPARE(sinx.upper_raw(),<,+1e-14_pr);
        ARIADNE_TEST_ASSERT(sinx.lower_raw()<sinx.upper_raw());
    }
    catch(...) { }

    try {
        FloatBoundsType x(7.0685834705770345_pr,pr);
        FloatBoundsType sinx=sin(x);
        ARIADNE_TEST_PRINT(x);
        ARIADNE_TEST_COMPARE(sinx.lower_raw(),<,0.7071067811866_pr);
        ARIADNE_TEST_COMPARE(sinx.upper_raw(),>,0.7071067811865_pr);
        ARIADNE_TEST_ASSERT(sinx.lower_raw()<sinx.upper_raw());
    }
    catch(...) { }

}

template<class PR> Void TestFloatBounds<PR>::regression_tests() {
    RawFloatType inf_=RawFloatType::inf(pr);

    // Regression test; fails dramatically on certain types of rounding
    {
        FloatBoundsType x(1.5707963267948966_pr,1.5707963267948968_pr,pr);
        FloatBoundsType cosx=cos(x);
        ARIADNE_TEST_PRINT(x);
        ARIADNE_TEST_COMPARE(cosx.lower_raw(),<,0.0_x);
        ARIADNE_TEST_COMPARE(cosx.lower_raw(),>,-1e-14_pr);
        ARIADNE_TEST_COMPARE(cosx.upper_raw(),>,0.0_x);
        ARIADNE_TEST_COMPARE(cosx.upper_raw(),<,+1e-14_pr);
        ARIADNE_TEST_ASSERT(cosx.lower_raw()<cosx.upper_raw());
    }

    // Regression test for dividing by interval with lower endpoint -0.0 or upper endpoint +0.0

    ARIADNE_TEST_EQUAL((FloatBoundsType(1.0_x,2.0_x,pr)/FloatBoundsType(-0.0_x,1.0_x,pr)).upper_raw(),+inf_);
    ARIADNE_TEST_EQUAL((FloatBoundsType(1.0_x,2.0_x,pr)/FloatBoundsType(-1.0_x,+0.0_x,pr)).lower_raw(),-inf_);

    ARIADNE_TEST_EQUAL(rec(FloatBoundsType(-0.0_x,+1.0_x,pr)).upper_raw(),+inf_);
    ARIADNE_TEST_EQUAL(rec(FloatBoundsType(-1.0_x,+0.0_x,pr)).lower_raw(),-inf_);
}


Int main() {
    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);

    TestFloatBounds<DoublePrecision>(dp).test();
    TestFloatBounds<MultiplePrecision>(MultiplePrecision(53_bits)).test();
    TestFloatBounds<MultiplePrecision>(MultiplePrecision(128_bits)).test();

    return ARIADNE_TEST_FAILURES;
}

