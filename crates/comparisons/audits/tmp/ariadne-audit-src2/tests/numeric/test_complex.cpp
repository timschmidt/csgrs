/***************************************************************************
 *            test_complex.cpp
 *
 *  Copyright  2019-20  Pieter Collins
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

#include "utility/module.hpp"
#include "config.hpp"

#include "foundations/logical.hpp"
#include "numeric/builtin.hpp"
#include "numeric/real.hpp"

#include "numeric/integer.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/decimal.hpp"
#include "numeric/rational.hpp"
#include "numeric/real.hpp"

#include "numeric/float_approximation.hpp"
#include "numeric/float_upper_bound.hpp"
#include "numeric/float_lower_bound.hpp"
#include "numeric/float_bounds.hpp"
#include "numeric/float_error.hpp"

#include "numeric/complex.hpp"

#include "../test.hpp"

using namespace Ariadne;

template<class X> class TestComplex
{
    X _one;
    typedef Complex<X> ComplexType;
  public:
    TestComplex(X const& one) : _one(one) { }
    void test();
  private:
    void test_concept();
    void test_constructors();
    void test_conversions();
    void test_arithmetic();
    void test_polar();
    void test_transcendental();
    void test_comparison();
};

template<class X> void TestComplex<X>::test()
{
    FloatDPApproximation::set_output_places(18);
    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_conversions());
    ARIADNE_TEST_CALL(test_arithmetic());
    ARIADNE_TEST_CALL(test_transcendental());
    ARIADNE_TEST_CALL(test_polar());
    ARIADNE_TEST_CALL(test_comparison());
}

template<class X> void TestComplex<X>::test_concept() {
    X* xp; X& x=*xp;
    Complex<X> z;
    x=Complex<X>(); x=Complex<X>(x);
    x=Complex<X>(x,x);
    z=+z; z=-z; z=z+z; z=z-z; z=z*z; z=z/z;
    z=pow(z,2u); z=pow(z,2);
    z=sqr(z); z=rec(z); z=sqrt(z);
    z=ezp(z); z=log(z);
    z=sin(z); z=cos(z); z=tan(z);
}

template<class X> void TestComplex<X>::test_conversions() {
//    auto one_dp=one.get(dp);
//    auto pi_mp=pi(MultiplePrecision(2_bits));
//    ARIADNE_TEST_PRINT(one_dp);
//    ARIADNE_TEST_PRINT(pi_mp);

}

template<class X> void TestComplex<X>::test_constructors() {
/*
    ARIADNE_TEST_CONSTRUCT(ComplexType,zz,);
    ARIADNE_TEST_EQUALS(zz.get(dp),0);
    ARIADNE_TEST_CONSTRUCT(ComplexType,zr,(1));
    ARIADNE_TEST_EQUALS(zr.get(dp),1);
    ARIADNE_TEST_CONSTRUCT(ComplexType,zc,(4,3));
    ARIADNE_TEST_EQUALS(zc.get(dp).real_part(),4);
*/
}

template<class X> void TestComplex<X>::test_arithmetic() {
    FloatDPApproximation::set_output_places(18);
    X x(2.5_dyadic*_one);
    X y(4.0_dyadic*_one);

    using Constants::i;

    ARIADNE_TEST_EQUALS(x+i*y, ComplexType(x,y));

    ARIADNE_TEST_EQUALS(x, 2.5_dy);
    ARIADNE_TEST_EQUALS(y, 4.0_dy);
    ARIADNE_TEST_EQUALS(+x, 2.5_dy);
    ARIADNE_TEST_EQUALS(-x,-2.5_dy);
    ARIADNE_TEST_EQUALS(x+y, 6.5_dy);
    ARIADNE_TEST_EQUALS(x-y,-1.5_dy);
    ARIADNE_TEST_EQUALS(x*y,10.0_dy);
    ARIADNE_TEST_EQUALS(x/y,0.625_dy);
    ARIADNE_TEST_EQUALS(add(x,y), 6.5_dy);
    ARIADNE_TEST_EQUALS(sub(x,y),-1.5_dy);
    ARIADNE_TEST_EQUALS(mul(x,y),10.0_dy);
    ARIADNE_TEST_EQUALS(div(x,y),0.625_dy);
    ARIADNE_TEST_EQUALS(pow(x,3u),15.625_dy);
    ARIADNE_TEST_EQUALS(pow(x,3),15.625_dy);
    ARIADNE_TEST_EQUALS(pos(x),+2.5_dy);
    ARIADNE_TEST_EQUALS(neg(x),-2.5_dy);
    ARIADNE_TEST_EQUALS(hlf(x),1.25_dy);
    ARIADNE_TEST_EQUALS(sqr(x),6.25_dy);
    ARIADNE_TEST_EQUALS(rec(y),0.25_dy);
}

template<class X> void TestComplex<X>::test_transcendental() {
    using Constants::i;
    Dyadic eps{FloatDP::eps(dp)};
    Dyadic tol=1024*eps;
    Complex<X> x(3.0_dyadic*_one,4.0_dyadic*_one);

    ARIADNE_TEST_WITHIN(sqrt(sqr(x)),x,tol);
    ARIADNE_TEST_WITHIN(sqr(sqrt(x)),x,tol);
    ARIADNE_TEST_WITHIN(log(exp(x))+2*pi*i,x,tol);
    ARIADNE_TEST_WITHIN(exp(log(x)),x,tol);

    ARIADNE_TEST_ASSERT(possibly(sqrt(Complex<X>(4*_one))==2.0_dy));
    ARIADNE_TEST_ASSERT(possibly(log(Complex<X>(1*_one))==0.0_dy));
    ARIADNE_TEST_ASSERT(possibly(exp(Complex<X>(0*_one))==1.0_dy));
    ARIADNE_TEST_ASSERT(possibly(log(exp(Complex<X>(1*_one)))==1.0_dy));
}


template<class X> void TestComplex<X>::test_polar() {
    Dyadic eps{FloatDP::eps(dp)};
    Dyadic tol=16*eps;
    X zero=nul(_one);
    Complex<X> i=Complex<X>(zero,_one);
    ARIADNE_TEST_WITHIN(arg(exp(-3*i)),-3,tol);
    ARIADNE_TEST_WITHIN(arg(exp(-2*i)),-2,tol);
    ARIADNE_TEST_WITHIN(arg(exp(-1*i)),-1,tol);
    ARIADNE_TEST_WITHIN(arg(exp(0*i)),0,tol);
    ARIADNE_TEST_WITHIN(arg(exp(1*i)),1,tol);
    ARIADNE_TEST_WITHIN(arg(exp(2*i)),2,tol);
    ARIADNE_TEST_WITHIN(arg(exp(3*i)),3,tol);
    ARIADNE_TEST_WITHIN(arg(exp(-pi/2*i)),-pi/2,tol);
    ARIADNE_TEST_WITHIN(arg(exp(+pi/2*i)),+pi/2,tol);
}

template<class X> void TestComplex<X>::test_comparison() {
    Effort effort(0);
    Complex<X> one=_one;
    Complex<X> e=exp(one);
//    Complex<X> elogpi=e*log(pi);
//    ARIADNE_TEST_SAME(e,e);
//    ARIADNE_TEST_EQUALS(log(e),one);

    ARIADNE_TEST_PRINT(pi.get(dp));
    ARIADNE_TEST_PRINT(e*log(pi).get(dp));

//    ARIADNE_TEST_ASSERT(not check(pi==elogpi,effort));
//    ARIADNE_TEST_ASSERT(check(pi!=elogpi,effort));
}


void test_fourier() {
    typedef FloatDPBounds X;
    auto pr=dp;

    SizeType N=32;
    Array<X> cs(N,[&](SizeType i){if (i==0) return X(1,pr); return X(1,pr)/(i*i);});

    using Constants::i;

    X x=X(1,pr)/2;

    Complex<X> r(0,0,pr);
    for(SizeType k=0; k!=32; ++k) {
        r=r+cs[k]*exp(i*k*x);
    }
}


int main() {
    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);
    using Constants::i;

    typedef FloatDPApproximation XA;
    Complex<XA> x(4,3,dp);

    FloatDP one(1,dp);
//    TestComplex<Real>().test();
//    TestComplex<FloatDPApproximation>(one).test();
    TestComplex<FloatDPBounds>(one).test();
    test_fourier();

    return ARIADNE_TEST_FAILURES;
}
