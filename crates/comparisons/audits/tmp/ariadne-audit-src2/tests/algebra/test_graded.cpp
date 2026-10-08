/***************************************************************************
 *            test_graded.cpp
 *
 *  Copyright  2018-20  Pieter Collins
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
#include "algebra/graded.hpp"

#include "../test.hpp"

namespace Ariadne {

template<class T1, class T2> decltype(declval<T1>()==declval<T2>()) operator==(List<T1> const& lst1, List<T2> const& lst2) {
    if(lst1.size()!=lst2.size()) { return false; }
    if(lst1.size()==0) { return true; }
    auto r=lst1[0]==lst2[0];
    for(SizeType i=1; i!=lst1.size(); ++i) {
        r=r && (lst1[i]==lst2[i]);
    }
    return r;
}
template<class T1, class T2> decltype(declval<T1>()==declval<T2>())  same(List<T1> const& lst1, List<T2> const& lst2) {
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



template<class X>
class TestGraded
{
    typedef typename X::PrecisionType PR;

    static Graded<X> variable(Dbl v0, DegreeType deg) {
        assert(deg>=1); X z(0.0); X x0(v0); Graded<X> r(x0); r.append(z+1);
        for(DegreeType d=2; d<=deg; ++d) { r.append(z); }
        return r; }

    static Graded<X> graded(InitializerList<Dbl> lst) {
        assert(lst.size()>=2); Graded<X> r;
        for(Double x : lst) { r.append(X(x)); }
        return r; }

    static Graded<X> graded(InitializerList<X> lst) {
        assert(lst.size()>=2); Graded<X> r;
        for(X x : lst) { r.append(x); }
        return r; }

    template<class OP> Graded<X> apply(OP op, Graded<X> const& a) {
        Graded<X> r(a.element_characteristics());
        while(r.size()!=a.size()) { compute(r,op,a); }
        return r;
    }

    template<class OP> Graded<X> apply(OP op, Graded<X> const& a, Int n) {
        Graded<X> r(a.element_characteristics());
        while(r.size()!=a.size()) { compute(r,op,a,n); }
        return r;
    }

    PR pr;
  public:
    TestGraded(PR prec) : pr(prec) { }

    void test() {
        ARIADNE_TEST_CALL(test_class());
        ARIADNE_TEST_CALL(test_sqr());
        ARIADNE_TEST_CALL(test_rec());
        ARIADNE_TEST_CALL(test_pow());
        ARIADNE_TEST_CALL(test_sqrt());
        ARIADNE_TEST_CALL(test_exp());
        ARIADNE_TEST_CALL(test_log());
        ARIADNE_TEST_CALL(test_sin());
        ARIADNE_TEST_CALL(test_cos());
    }
  private:
    void test_class() {
    }

    void test_sqr() {
        ARIADNE_TEST_EQUALS(apply(Sqr(),Graded<X>::variable(X(2.0_x,pr),5u)), (Graded<X>({4.0_x,4.0_x,1.0_x,0.0_x,0.0_x,0.0_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Sqr(),Graded<X>({3.0_x,2.0_x,1.0_x,0.0_x,0.0_x,0.0_x},pr)), (Graded<X>({9.0_x,12.0_x,10.0_x,4.0_x,1.0_x,0.0_x},pr)) );
    }

    void test_rec() {
        ARIADNE_TEST_EQUALS(apply(Rec(),Graded<X>::variable(X(2.0_x,pr),5u)), (Graded<X>({0.5_x,-0.25_x,0.125_x,-0.0625_x,0.03125_x,-0.015625_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Rec(),Graded<X>::variable(X(1.0_x,pr),5u)), (Graded<X>({1.0_x,-1.0_x,1.0_x,-1.0_x,1.0_x,-1.0_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Rec(),Graded<X>::variable(X(0.5_x,pr),5u)), (Graded<X>({2.0_x,-4.0_x,8.0_x,-16.0_x,32.0_x,-64.0_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Rec(),Graded<X>::variable(X(-1.0_x,pr),5u)), (Graded<X>({-1.0_x,-1.0_x,-1.0_x,-1.0_x,-1.0_x,-1.0_x},pr)) );
    }

    void test_pow() {
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),2), apply(Sqr(),Graded<X>::variable(X(3.0_x,pr),5u)) );
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),4), apply(Sqr(),apply(Sqr(),Graded<X>::variable(X(3.0_x,pr),5u))) );
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),4), apply(Sqr(),apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),2)) );
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),4), apply(Pow(),apply(Sqr(),Graded<X>::variable(X(3.0_x,pr),5u)),2) );

        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(2.0_x,pr),7u),6), (apply(Pow(),apply(Sqr(),Graded<X>::variable(X(2,pr),7u)),3)) )
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(2.0_x,pr),7u),6), (apply(Sqr(),apply(Pow(),Graded<X>::variable(X(2,pr),7u),3))) )

        ARIADNE_TEST_EQUALS(apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),0), (Graded<X>({1,0,0,0,0,0},pr)) );
        ARIADNE_TEST_EQUALS(apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),1), (Graded<X>({3.0_x,1.0_x,0.0_x,0.0_x,0.0_x,0.0_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),2), (Graded<X>({9.0_x,6.0_x,1.0_x,0.0_x,0.0_x,0.0_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Pow(),Graded<X>::variable(X(1.0_x,pr),5u),5), (Graded<X>({1.0_x,5.0_x,10.0_x,10.0_x,5.0_x,1.0_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Pow(),Graded<X>::variable(X(3.0_x,pr),5u),5), (Graded<X>({243.0_x,405.0_x,270.0_x,90.0_x,15.0_x,1.0_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Pow(),Graded<X>::variable(X(-3.0_x,pr),5u),5), (Graded<X>({-243.0_x,405.0_x,-270.0_x,90.0_x,-15.0_x,1.0_x},pr)) );
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(-3.0_x,pr),7u),2), (apply(Sqr(),Graded<X>::variable(X(-3,pr),7u))) )
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(2.0_x,pr),7u),6), (apply(Sqr(),apply(Pow(),Graded<X>::variable(X(2,pr),7u),3))) )
        ARIADNE_TEST_EQUALS(apply(Pow(),Graded<X>({2.0_x,0.0_x,1.0_x,0.0_x,0.0_x,0.0_x,0.0_x,0.0_x},pr),3), (Graded<X>({8.0_x,0.0_x,12.0_x,0.0_x,6.0_x,0.0_x,1.0_x,0.0_x},pr)) );

        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>({4.0_x,4.0_x,1.0_x, 0,0,0,0,0},pr),3), (Graded<X>({64.0_x,192.0_x,240.0_x, 160.0_x,60.0_x,12.0_x,1.0_x,0.0_x},pr)) );

        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(2.0_x,pr),5u),-1), apply(Rec(),Graded<X>::variable(X(2.0_x,pr),5u)) );
        ARIADNE_TEST_EQUAL (apply(Pow(),Graded<X>::variable(X(2.0_x,pr),5u),-6), apply(Rec(),apply(Pow(),Graded<X>::variable(X(2.0_x,pr),5u),6)) );
    }

    void test_sqrt() {
        ARIADNE_TEST_EQUALS( apply(Sqrt(),Graded<X>::variable(X(4.0_x,pr),5u)),
                             (Graded<X>({2.0_x,0.25_x,-0.015625_x,0.001953125_x,-0.00030517578125_x,0.00005340576171875_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Sqrt(),Graded<X>::variable(X(1.0_x,pr),5u)), (Graded<X>({1.0_x,0.5_x,-0.125_x,0.0625_x,-0.0390625_x,0.02734375_x},pr)) );
        ARIADNE_TEST_EQUALS(apply(Sqrt(),Graded<X>::variable(X(0.25_x,pr),5u)), (Graded<X>({0.5_x,1.0_x,-1.0_x,2.0_x,-5.0_x,14.0_x},pr)) );
    }

    void test_exp() {
        const X zero(0,pr), one(1,pr);
        const X exp1=exp(one);
        ARIADNE_TEST_EQUALS( apply(Exp(),Graded<X>::variable(zero,5u)),
                             (Graded<X>{one,one,one/2,one/6,one/24,one/120}) );
        ARIADNE_TEST_EQUALS( apply(Exp(),Graded<X>::variable(one,5u)),
                             (Graded<X>{exp1,exp1/1,exp1/2,exp1/6,exp1/24,exp1/120}) );
    }

    void test_log() {
        const X zero(0,pr), one(1,pr), two(2,pr);
        const X log2=log(two);
        ARIADNE_TEST_EQUALS( apply(Log(),Graded<X>::variable(one,5u)),
                             (Graded<X>{zero,one,-one/2,one/3,-one/4,one/5}) );
        ARIADNE_TEST_EQUALS( apply(Log(),Graded<X>::variable(two,5u)),
                             (Graded<X>{log2,one/2,-one/8,one/24,-one/64,one/160}) );
    }

    void test_sin() {
        const X zero(0,pr), one(1,pr);
        const X sin1=sin(one), cos1=cos(one);
        ARIADNE_TEST_EQUALS( apply(Sin(),Graded<X>::variable(zero,5u)),
                             (Graded<X>{zero,one,zero,-one/6,zero,one/120}) );
        ARIADNE_TEST_EQUALS ( apply(Sin(),Graded<X>::variable(one,5u)),
                             (Graded<X>{sin1,cos1,-sin1/2,-cos1/6,sin1/24,cos1/120}) );
    }

    void test_cos() {
        const X zero(0,pr), one(1,pr);
        const X sin1=sin(one), cos1=cos(one);
        ARIADNE_TEST_EQUALS( apply(Cos(),Graded<X>::variable(zero,5u)),
                             (Graded<X>{one,zero,-one/2,zero,one/24,zero}) );
        // Only go to degree 4 here due to roundoff error
        ARIADNE_TEST_EQUALS ( apply(Cos(),Graded<X>::variable(one,4u)),
                             (Graded<X>{cos1,-sin1,-cos1/2,sin1/6,cos1/24}) );
    }
};


Int main() {
    TestGraded<RoundedFloatDP>(dp).test();

    return ARIADNE_TEST_FAILURES;
}
