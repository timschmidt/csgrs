/***************************************************************************
 *            test_chebyshev_polynomial.cpp
 *
 *  Copyright  2008-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <iostream>
#include <iomanip>
#include "config.hpp"

#include "utility/metaprogramming.hpp"
#include "utility/typedefs.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/differential.hpp"
#include "function/function.hpp"
#include "function/polynomial.hpp"

#include "function/chebyshev_polynomial.hpp"
#include "function/chebyshev_polynomial.tpl.hpp"

#include "algebra/sweeper.hpp"

#include "../test.hpp"

using std::cout; using std::cerr; using std::endl;
using namespace Ariadne;

extern template Ariadne::Nat Ariadne::Error<Ariadne::FloatMP>::output_places;
extern template Ariadne::Nat Ariadne::Approximation<Ariadne::FloatDP>::output_places;
extern template Ariadne::Nat Ariadne::Approximation<Ariadne::FloatMP>::output_places;

template<class I, class X> decltype(auto) sup_norm(Polynomial<I,X> const& p) {
    auto r=mag(p.expansion().zero_coefficient());
    for (auto term : p) { r += mag(term.coefficient()); }
    return r;
}

template<class X> class TestChebyshevPolynomial
{
    typedef PrecisionType<X> PR;
    PR pr;
  public:
    TestChebyshevPolynomial(PrecisionType<X> prec);
    Void test();
  private:
    Void test_concept();
    Void test_univariate();
    Void test_multivariate();
};

template<class X> TestChebyshevPolynomial<X>::TestChebyshevPolynomial(PrecisionType<X> prec)
    : pr(prec)
{
}

template<class X> Void TestChebyshevPolynomial<X>::test()
{
    Float<PR>::set_output_places(18);
    FloatBounds<PR>::set_output_places(18);
    FloatApproximation<PR>::set_output_places(8);

    ARIADNE_TEST_CALL(test_univariate());
    ARIADNE_TEST_CALL(test_multivariate());
}

template<class X> Void TestChebyshevPolynomial<X>::test_concept()
{
    X c;
    UnivariateChebyshevPolynomial<X>* xp; UnivariateChebyshevPolynomial<X>& x=*xp;
    x=+x; x=-x; x=x+x; x=x-x; x=x*x;
    x=x+c; x=c+x; x=x-c; x=c-x; x=x*c; x=c*x; x=x/c;
    // x=sqr(x);
    x=pow(x,2u);


}


template<class X> Void TestChebyshevPolynomial<X>::test_univariate() {
    ARIADNE_TEST_NAMED_CONSTRUCT(UnivariateChebyshevPolynomial<X>,one,constant(1,pr));
    ARIADNE_TEST_NAMED_CONSTRUCT(UnivariateChebyshevPolynomial<X>,x,coordinate(pr));

    ARIADNE_TEST_PRINT(x);
    ARIADNE_TEST_PRINT(x*x);
    ARIADNE_TEST_PRINT(x*x*x);
    ARIADNE_TEST_PRINT(x*x*x*x);

    ARIADNE_TEST_CONSTRUCT(X,y,(-0.75_dy,pr));
    ARIADNE_TEST_EQUALS(x(y),(y));
    ARIADNE_TEST_EQUALS((x*x)(y),(y*y));
    ARIADNE_TEST_EQUALS((x*x*x)(y),(y*y*y));
    ARIADNE_TEST_EQUALS((x*x*x*x)(y),(y*y*y*y));

    ARIADNE_TEST_EQUALS(UnivariateChebyshevPolynomial<X>::basis(0u,pr),one);
    ARIADNE_TEST_EQUALS(UnivariateChebyshevPolynomial<X>::basis(1u,pr),x);
    ARIADNE_TEST_EQUALS(UnivariateChebyshevPolynomial<X>::basis(2u,pr),x*x*2-1);
    ARIADNE_TEST_EQUALS(UnivariateChebyshevPolynomial<X>::basis(3u,pr),x*x*x*4-x*3);
    ARIADNE_TEST_EQUALS(UnivariateChebyshevPolynomial<X>::basis(4u,pr),x*x*x*x*8-x*x*8+1);

    UnivariatePolynomial<X> p=UnivariatePolynomial<X>::coordinate(pr);
    ARIADNE_TEST_PRINT(p);

    std::cout << "pnorm(x^4-x^2)=" << sup_norm(p*p*p*p-p*p) << "\n";
    ARIADNE_TEST_EQUALS(sup_norm(x*x),1.0_dy);
    ARIADNE_TEST_EQUALS(sup_norm(pow(x,4u)),1.0_dy);
    ARIADNE_TEST_EQUALS(sup_norm(pow(x,4)-pow(x,2)),0.25_dy);
    ARIADNE_TEST_EQUALS(sup_norm(x*x*x*x-x*x),0.25_dy);
    ARIADNE_TEST_EQUALS(sup_norm(3*x*x*x*x-2*x*x-1),1.75_dy);
    ARIADNE_TEST_COMPARE(sup_norm(3*x*x*x*x-2*x*x-1),<=,sup_norm(3*p*p*p*p-2*p*p-1));
//    std::cout << "pnorm(3x^4-2x^2-1)=sup_norm("<<(X(3,pr)*p*p*p*p-X(2,pr)*p*p-X(1,pr)) << ")=" << sup_norm(X(3,pr)*p*p*p*p-X(2,pr)*p*p-X(1,pr)) << "\n";
    std::cout << std::endl;
}


template<class X> Void TestChebyshevPolynomial<X>::test_multivariate() {
    ARIADNE_TEST_NAMED_CONSTRUCT(MultivariateChebyshevPolynomial<X>,one,constant(2u,1,pr));
    ARIADNE_TEST_NAMED_CONSTRUCT(MultivariateChebyshevPolynomial<X>,x,coordinate(2u,0u,pr));
    ARIADNE_TEST_NAMED_CONSTRUCT(MultivariateChebyshevPolynomial<X>,y,coordinate(2u,1u,pr));

    Vector<X> v({0.5,-0.75},pr);
    ARIADNE_TEST_EQUALS(x(v),v[0]);
    ARIADNE_TEST_EQUALS((x*x)(v),v[0]*v[0]);
    ARIADNE_TEST_EQUALS((x*x*x)(v),v[0]*v[0]*v[0]);
    ARIADNE_TEST_EQUALS((x*x*x*x)(v),v[0]*v[0]*v[0]*v[0]);
    ARIADNE_TEST_EQUALS(pow(x,3)(v),pow(v[0],3));
    ARIADNE_TEST_EQUALS(pow(x,4)(v),pow(v[0],4));
    ARIADNE_TEST_EQUALS((x*x*x*x-x*x)(v),v[0]*v[0]*v[0]*v[0]-v[0]*v[0]);
    ARIADNE_TEST_EQUALS((pow(x,4)-pow(x,2))(v),pow(v[0],4)-pow(v[0],2));

    ARIADNE_TEST_EQUALS(y(v),v[1]);
    ARIADNE_TEST_EQUALS((y*y)(v),v[1]*v[1]);
    ARIADNE_TEST_EQUALS((y*y*y)(v),v[1]*v[1]*v[1]);
    ARIADNE_TEST_EQUALS((y*y*y*y)(v),v[1]*v[1]*v[1]*v[1]);

    assert(v.size()==2);
    ARIADNE_TEST_EQUALS((x*y)(v),v[0]*v[1]);
    ARIADNE_TEST_EQUALS((x*x*y)(v),v[0]*v[0]*v[1]);
    ARIADNE_TEST_EQUALS((x*y*y)(v),v[0]*v[1]*v[1]);
}



Int main() {
    MultiplePrecision mp(128);
    TestChebyshevPolynomial<FloatDPApproximation>(dp).test();
    TestChebyshevPolynomial<FloatMPApproximation>(mp).test();

    ThresholdSweeper<FloatDP> sweeper_dp(dp,1e-8);
    ThresholdSweeper<FloatMP> sweeper_mp(mp,std::pow(2.0,-64));
    ARIADNE_TEST_PRINT(sweeper_mp.precision());
    ARIADNE_TEST_PRINT(sweeper_mp);

    return ARIADNE_TEST_FAILURES;
}



