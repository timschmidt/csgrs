/***************************************************************************
 *            test_float_approximation.cpp
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
#include "numeric/float_approximation.hpp"

#include "../test.hpp"
#include "test_floats.hpp"

using namespace Ariadne;
using namespace std;

template<class PR>
class TestFloatApproximation
    : public TestFloats<PR>
{
    typedef FloatType<ApproximateTag,PR> FloatApproximationType;

    using TestFloats<PR>::m,TestFloats<PR>::n,TestFloats<PR>::ad,TestFloats<PR>::ed;
    using TestFloats<PR>::z,TestFloats<PR>::w,TestFloats<PR>::d,TestFloats<PR>::q;
  private:
    PR precision;
  public:
    TestFloatApproximation(PR prec) : precision(prec) { };
    Void test();
  private:
    Void test_concept();
    Void test_conversions();
    Void test_arithmetic();
    Void test_comparison();
};


template<class PR> Void
TestFloatApproximation<PR>::test()
{
    ARIADNE_TEST_CALL(test_conversions());
    ARIADNE_TEST_CALL(test_arithmetic());
    ARIADNE_TEST_CALL(test_comparison());
}

template<class PR> Void
TestFloatApproximation<PR>::test_concept()
{
    typedef DoublePrecision PRE; PRE pre;
    PR pr=precision;

    FloatApproximation<PR>::set_output_places(17);
    FloatApproximationType ax(pr);
    FloatApproximationType rx(pr);

    // Constructors
    FloatApproximationType(0u,pr); FloatApproximationType(0,pr); FloatApproximationType(0.0,pr);
    FloatApproximationType(Integer(0),pr); FloatApproximationType(Dyadic(0),pr); FloatApproximationType(Rational(0),pr);

    rx=FloatApproximationType(RawFloat<PR>(pr));
    rx=FloatApproximationType(Float<PR>(pr)); FloatApproximationType(FloatBall<PR,PRE>(pr,pre)); FloatApproximationType(FloatBounds<PR>(pr));
    rx=FloatApproximationType(FloatUpperBound<PR>(pr)); rx=FloatApproximationType(FloatLowerBound<PR>(pr));
    rx=FloatApproximationType(FloatApproximation<PR>(pr));

    // Assignment
    rx=m; rx=n; rx=ad; rx=ed; rx=z; rx=w; rx=q;

    // ExactTag operations
    rx=nul(ax); rx=pos(ax); rx=neg(ax); rx=hlf(ax); rx=sqr(ax); rx=rec(ax);

    rx=operator+(ax); rx=operator-(ax); rx=operator+(ax,ax); rx=operator-(ax,ax); rx=operator*(ax,ax); rx=operator/(ax,ax);

    // Arithmetic
    rx=add(ax,ax);
    rx=sub(ax,ax);
    rx=mul(ax,ax);
    rx=div(ax,ax);
    rx=pow(ax,m);
    rx=pow(ax,n);

    // Order
    rx=max(ax,ax); rx=min(ax,ax); rx=abs(ax);

    // Transcendental functions
    rx=sqrt(ax);
    rx=exp(ax);
    rx=log(ax);
    rx=sin(ax);
    rx=cos(ax);
    rx=tan(ax);
    rx=atan(ax);
}

template<class PR> Void
TestFloatApproximation<PR>::test_conversions()
{
    PR pr=precision;
    ARIADNE_TEST_EQUALS(cast_integer(FloatApproximation<PR>(Dyadic(-3,1u),pr)),Integer(-2));
    ARIADNE_TEST_EQUALS(cast_integer(FloatApproximation<PR>(Dyadic(2),pr)),Integer(2));
    ARIADNE_TEST_EQUALS(cast_integer(FloatApproximation<PR>(Dyadic(3,1u),pr)),Integer(2));
    ARIADNE_TEST_EQUALS(cast_integer(FloatApproximation<PR>(Dyadic(7,2u),pr)),Integer(2));
}

template<class PR> Void
TestFloatApproximation<PR>::test_arithmetic()
{
    RawFloat<PR> third(Rational(1,3),near,precision);
    RawFloat<PR> fifth(Rational(1,5),near,precision);
    RawFloat<PR> seventh(Rational(1,7),near,precision);

    ARIADNE_TEST_SAME(max(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(third));
    ARIADNE_TEST_SAME(min(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(fifth));
    ARIADNE_TEST_SAME(abs(FloatApproximation<PR>(neg(third))),FloatApproximation<PR>(third));

    ARIADNE_TEST_SAME(mag(FloatApproximation<PR>(neg(third))),PositiveFloatApproximation<PR>(third));
    ARIADNE_TEST_SAME(mig(FloatApproximation<PR>(neg(third))),PositiveFloatApproximation<PR>(third));

    ARIADNE_TEST_SAME(operator+(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(add(near,third,fifth)));
    ARIADNE_TEST_SAME(operator-(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(sub(near,third,fifth)));
    ARIADNE_TEST_SAME(operator*(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(mul(near,third,fifth)));
    ARIADNE_TEST_SAME(operator/(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(div(near,third,fifth)));

    ARIADNE_TEST_SAME(nul(FloatApproximation<PR>(third)),FloatApproximation<PR>(nul(third)));
    ARIADNE_TEST_SAME(pos(FloatApproximation<PR>(third)),FloatApproximation<PR>(pos(third)));
    ARIADNE_TEST_SAME(neg(FloatApproximation<PR>(third)),FloatApproximation<PR>(neg(third)));
    ARIADNE_TEST_SAME(hlf(FloatApproximation<PR>(third)),FloatApproximation<PR>(hlf(third)));
    ARIADNE_TEST_SAME(add(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(add(near,third,fifth)));
    ARIADNE_TEST_SAME(sub(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(sub(near,third,fifth)));
    ARIADNE_TEST_SAME(mul(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(mul(near,third,fifth)));
    ARIADNE_TEST_SAME(div(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth)),FloatApproximation<PR>(div(near,third,fifth)));
    ARIADNE_TEST_SAME(fma(FloatApproximation<PR>(third),FloatApproximation<PR>(fifth),FloatApproximation<PR>(seventh)),
                          FloatApproximation<PR>(fma(near,third,fifth,seventh)));
    ARIADNE_TEST_SAME(pow(FloatApproximation<PR>(fifth),3),FloatApproximation<PR>(pow(near,fifth,3)));
}

template<class PR> Void
TestFloatApproximation<PR>::test_comparison()
{
    PR pr=precision;
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-2,pr)==FloatApproximation<PR>(-1,pr),ApproximateKleenean(false));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-2,pr)!=FloatApproximation<PR>(-1,pr),ApproximateKleenean(true));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-2,pr)< FloatApproximation<PR>(-1,pr),ApproximateKleenean(true));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-2,pr)> FloatApproximation<PR>(-1,pr),ApproximateKleenean(false));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-2,pr)<=FloatApproximation<PR>(-1,pr),ApproximateKleenean(true));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-2,pr)>=FloatApproximation<PR>(-1,pr),ApproximateKleenean(false));

    ARIADNE_TEST_SAME(FloatApproximation<PR>(-1,pr)==FloatApproximation<PR>(-1,pr),ApproximateKleenean(true));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-1,pr)!=FloatApproximation<PR>(-1,pr),ApproximateKleenean(false));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-1,pr)< FloatApproximation<PR>(-1,pr),ApproximateKleenean(false));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-1,pr)> FloatApproximation<PR>(-1,pr),ApproximateKleenean(false));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-1,pr)<=FloatApproximation<PR>(-1,pr),ApproximateKleenean(true));
    ARIADNE_TEST_SAME(FloatApproximation<PR>(-1,pr)>=FloatApproximation<PR>(-1,pr),ApproximateKleenean(true));
}


Int main() {
    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);

    TestFloatApproximation<DoublePrecision>(dp).test();
    TestFloatApproximation<MultiplePrecision>(MultiplePrecision(128_bits)).test();

    return ARIADNE_TEST_FAILURES;
}

