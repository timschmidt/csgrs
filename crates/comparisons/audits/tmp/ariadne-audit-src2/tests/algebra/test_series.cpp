/***************************************************************************
 *            test_series.cpp
 *
 *  Copyright  2015-20  Pieter Collins
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

#include "numeric/numeric.hpp"
#include "algebra/series.hpp"
#include "function/taylor_series.hpp"

#include "../test.hpp"

namespace Ariadne {

template<class F> decltype(auto) operator==(Rounded<F> const& x, Rational const& q) { return x.raw()==q; }

//template<class Iter> container(Iter b, Iter e) -> container<typename std::iterator_traits<Iter>::value_type>;
template<class X> List(InitializerList<X>) -> List<X>;

template<class T1, class T2> EqualsType<T1,T2> operator==(List<T1> const& lst1, List<T2> const& lst2) {
    if(lst1.size()!=lst2.size()) { return false; }
    EqualsType<T1,T2> r=true;
    for(SizeType i=0; i!=lst1.size(); ++i) {
        r = r && (lst1[i]==lst2[i]);
    }
    return r;
}

template<class T> Bool same(List<T> const& lst1, List<T> const& lst2) {
    if(lst1.size()!=lst2.size()) { return false; }
    for(SizeType i=0; i!=lst1.size(); ++i) {
        if(!same(lst1[i],lst2[i])) { return false; }
    }
    return true;
    auto iter1=lst1.begin();
    auto iter2=lst2.begin();
    for(; iter1!=lst1.end(); ++iter1, ++iter2) {
        if(not same(*iter1,*iter2)) { return false; }
    }
    return true;
}
} // namespace Ariadne

using namespace Ariadne;

template<class X> class TestSeries
{
    typedef Rational Q;
    typedef typename X::PrecisionType PR;
    PR pr;
  public:
    TestSeries(PR prec) : pr(prec) { }

    void test() {
        ARIADNE_TEST_CALL(test_class());
        ARIADNE_TEST_CALL(test_rec());
        ARIADNE_TEST_CALL(test_sqrt());
        ARIADNE_TEST_CALL(test_exp());
        ARIADNE_TEST_CALL(test_log());
        ARIADNE_TEST_CALL(test_sin());
        ARIADNE_TEST_CALL(test_cos());
    }
  private:
    void test_class() {
        ARIADNE_TEST_CONSTRUCT( Series<X>, series, (Rec(),X(1.0_x,pr)) );
        ARIADNE_TEST_EQUALS(series[0],1.0_x);
        ARIADNE_TEST_EQUALS(series[10],1.0_x);
        ARIADNE_TEST_EQUALS(series[1],-1.0_x);
        ARIADNE_TEST_EQUALS(series[127],-1.0_x);
        ARIADNE_TEST_EQUALS(series[255],-1.0_x);
//        ARIADNE_TEST_EQUALS(series[32767],1.0);
        std::cerr<<std::setprecision(18);
    }
    void test_rec() {
        ARIADNE_TEST_EQUALS(Series<X>(Rec(),X(2.0_x,pr)).coefficients(5), (List<Q>{0.5_q,-0.25_q,0.125_q,-0.0625_q,0.03125_q,-0.015625_q}) );
        ARIADNE_TEST_EQUALS(Series<X>(Rec(),X(1.0_x,pr)).coefficients(5), (List<Q>{1.0_q,-1.0_q,1.0_q,-1.0_q,1.0_q,-1.0_q}) );
        ARIADNE_TEST_EQUALS(Series<X>(Rec(),X(0.5_x,pr)).coefficients(5), (List<Q>{2.0_q,-4.0_q,8.0_q,-16.0_q,32.0_q,-64.0_q}) );
        ARIADNE_TEST_EQUALS(Series<X>(Rec(),X(-1.0_x,pr)).coefficients(5), (List<Q>{-1.0_q,-1.0_q,-1.0_q,-1.0_q,-1.0_q,-1.0_q}) );
    }

    void test_sqrt() {
        ARIADNE_TEST_EQUALS(Series<X>(Sqrt(),X(4.0_x,pr)).coefficients(5),
                             (List<Q>{2.0_q,0.25_q,-0.015625_q,0.001953125_q,-0.00030517578125_q,0.00005340576171875_q}) );
        ARIADNE_TEST_EQUALS(Series<X>(Sqrt(),X(1.0_x,pr)).coefficients(5), (List<Q>{1.0_q,0.5_q,-0.125_q,0.0625_q,-0.0390625_q,0.02734375_q}) );
        ARIADNE_TEST_EQUALS(Series<X>(Sqrt(),X(0.25_x,pr)).coefficients(5), (List<Q>{0.5_q,1.0_q,-1.0_q,2.0_q,-5.0_q,14.0_q}) );
    }

    void test_exp() {
        const X zero(0,pr), one(1,pr);
        const X exp1=exp(one);
        ARIADNE_TEST_EQUALS( Series<X>(Exp(),zero).coefficients(5),
                             (List<X>{one,one/1,one/2,one/6,one/24,one/120}) );
        ARIADNE_TEST_EQUALS( Series<X>(Exp(),one).coefficients(5),
                             (List<X>{exp1,exp1/1,exp1/2,exp1/6,exp1/24,exp1/120}) );
    }

    void test_log() {
        const X zero(0,pr), one(1,pr), two(2,pr);
        const X log2=log(two);
        ARIADNE_TEST_EQUALS( Series<X>(Log(),one).coefficients(5),
                             (List<X>{zero,one,-one/2,one/3,-one/4,one/5}) );
        ARIADNE_TEST_EQUALS( Series<X>(Log(),two).coefficients(5),
                             (List<X>{log2,one/2,-one/8,one/24,-one/64,one/160}) );
    }

    void test_sin() {
        const X zero(0,pr), one(1,pr);
        const X sin1=sin(one);
        const X cos1=cos(one);

        ARIADNE_TEST_EQUALS( Series<X>(Sin(),zero).coefficients(5),
                             (List<X>{zero,one,zero,-one/6,zero,one/120}) );
        ARIADNE_TEST_EQUAL ( Series<X>(Sin(),one).coefficients(5),
                             (List<X>{sin1,cos1,-sin1/2,-cos1/6,sin1/24,cos1/120}) );
    }

    void test_cos() {
        const X zero(0,pr), one(1,pr);
        const X sin1=sin(one);
        const X cos1=cos(one);
        ARIADNE_TEST_EQUALS( Series<X>(Cos(),zero).coefficients(5),
                             (List<X>{one,zero,-one/2,zero,one/24,zero}) );
        ARIADNE_TEST_EQUAL ( Series<X>(Cos(),one).coefficients(4),
                             (List<X>{cos1,-sin1,-cos1/2,sin1/6,cos1/24}) );
    }
};


Int main() {
    TestSeries<RoundedFloatDP>(dp).test();

    return ARIADNE_TEST_FAILURES;
}
