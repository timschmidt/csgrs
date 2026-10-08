/***************************************************************************
 *            test_float_ball.cpp
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

#include "numeric/float_ball.hpp"

#include "numeric/float_error.hpp"
#include "numeric/float_bounds.hpp"

#include "../test.hpp"
#include "test_floats.hpp"

using namespace Ariadne;
using namespace std;

template<class PR, class PRE=PR>
class TestFloatBall
    : public TestFloats<PR>
{
    typedef RawFloat<PR> RawFloatType;
    typedef FloatType<ApproximateTag,PR> FloatApproximationType;
    typedef FloatType<LowerTag,PR> FloatLowerBoundType;
    typedef FloatType<UpperTag,PR> FloatUpperBoundType;
    typedef FloatType<BoundedTag,PR> FloatBoundsType;
    typedef FloatBall<PR,PRE> FloatBallType;
    typedef FloatType<ExactTag,PR> FloatValueType;
  private:
    PR precision; PRE error_precision;
    static PRE _make_error_precision(PR const& pr) { if constexpr (Same<PR,PRE>) { return pr; } else { return PRE(); } }
  public:
    TestFloatBall(PR prec) : precision(prec), error_precision(_make_error_precision(prec)) { };
    TestFloatBall(PR prec, PRE err_prec) : precision(prec), error_precision(err_prec) { };
    Void test();
  private:
    using TestFloats<PR>::to_rational;
    Void test_concept();
    Void test_precision();
    Void test_conversions();
    Void test_validation();
    Void test_rounded_arithmetic();
};

template<class PR, class PRE> Void
TestFloatBall<PR,PRE>::test()
{
    ARIADNE_TEST_CALL(test_precision());
    ARIADNE_TEST_CALL(test_conversions());
    ARIADNE_TEST_CALL(test_rounded_arithmetic());
}

template<class PR, class PRE> Void
TestFloatBall<PR,PRE>::test_conversions()
{
    Rational one=1;
    Rational third=one/3;

    ARIADNE_TEST_EQUALS(cast_integer(FloatBall<PR,PRE>(FloatBounds<PR>(2,3,precision))),Integer(3));
    ARIADNE_TEST_EQUALS(cast_integer(FloatBall<PR,PRE>(FloatBounds<PR>(2.25_dy,3.25_dy,precision))),Integer(3));
    ARIADNE_TEST_FAIL(cast_integer(FloatBall<PR,PRE>(FloatBounds<PR>(2.625_dy,2.875_dy,precision))));
}

template<class PR, class PRE> Void
TestFloatBall<PR,PRE>::test_precision()
{
    FloatBallType mx(Rational(1),precision);
    ARIADNE_TEST_EQUALS(mx.value().precision(),precision);
    ARIADNE_TEST_EQUALS(mx.error().precision(),error_precision);
    ARIADNE_TEST_EQUALS(max(mx,mx).precision(),precision);
    ARIADNE_TEST_EQUALS(min(mx,mx).precision(),precision);
    ARIADNE_TEST_EQUALS(abs(mx).precision(),precision);
    ARIADNE_TEST_EQUALS((mx+mx).precision(),precision);
    ARIADNE_TEST_EQUALS((mx-mx).precision(),precision);
    ARIADNE_TEST_EQUALS((mx*mx).precision(),precision);
    ARIADNE_TEST_EQUALS((mx/mx).precision(),precision);
    ARIADNE_TEST_EQUALS(sqrt(mx).precision(),precision);
    ARIADNE_TEST_EQUALS(exp(mx).precision(),precision);
    ARIADNE_TEST_EQUALS(log(mx).precision(),precision);
    ARIADNE_TEST_EQUALS(sin(mx).precision(),precision);
    ARIADNE_TEST_EQUALS(cos(mx).precision(),precision);
    ARIADNE_TEST_EQUALS(tan(mx).precision(),precision);
    ARIADNE_TEST_EQUALS(atan(mx).precision(),precision);
    ARIADNE_TEST_EQUALS((mx+2u).precision(),precision);
    ARIADNE_TEST_EQUALS((mx-2u).precision(),precision);
    ARIADNE_TEST_EQUALS((mx*2u).precision(),precision);
    ARIADNE_TEST_EQUALS((mx/2u).precision(),precision);
    ARIADNE_TEST_EQUALS((mx+2).precision(),precision);
    ARIADNE_TEST_EQUALS((mx-2).precision(),precision);
    ARIADNE_TEST_EQUALS((mx*2).precision(),precision);
    ARIADNE_TEST_EQUALS((mx/2).precision(),precision);
}

template<class PR, class PRE> Void
TestFloatBall<PR,PRE>::test_rounded_arithmetic()
{
    Rational one=1;
    Rational three=3;
    Rational five=5;
    Rational six=6;
    Rational third=one/three;
    Rational fifth=one/five;

    ARIADNE_TEST_BINARY_PREDICATE(models,FloatBallType(third,precision)+FloatBallType(fifth,precision),third+fifth);
    ARIADNE_TEST_BINARY_PREDICATE(models,FloatBallType(third,precision)-FloatBallType(fifth,precision),third-fifth);
    ARIADNE_TEST_BINARY_PREDICATE(models,FloatBallType(third,precision)*FloatBallType(fifth,precision),third*fifth);
    ARIADNE_TEST_BINARY_PREDICATE(models,FloatBallType(third,precision)/FloatBallType(fifth,precision),third/fifth);
//    ARIADNE_TEST_BINARY_PREDICATE(models,1/FloatBallType(three,precision)/FloatBallType(fifth,precision),1/three);

    ARIADNE_TEST_BINARY_PREDICATE(models,pow(FloatBallType(three/five,precision),4u),pow(three/five,4u));
    ARIADNE_TEST_BINARY_PREDICATE(models,pow(FloatBallType(three/five,precision),4),pow(three/five,4));
    ARIADNE_TEST_BINARY_PREDICATE(models,pow(FloatBallType(three/five,precision),-4),pow(three/five,-4));
    ARIADNE_TEST_BINARY_PREDICATE(models,pow(FloatBallType(three/five,precision),-7),pow(three/five,-7));
}





Int main() {
    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);

    TestFloatBall<DoublePrecision,DoublePrecision>(dp).test();
    TestFloatBall<MultiplePrecision>(MultiplePrecision(128_bits)).test();
    TestFloatBall<MultiplePrecision,DoublePrecision>(MultiplePrecision(128_bits)).test();

    return ARIADNE_TEST_FAILURES;
}

