/***************************************************************************
 *            test_taylor_model.cpp
 *
 *  Copyright  2008-20  Pieter Collins
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
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "function/taylor_model.hpp"
#include "algebra/differential.hpp"
#include "function/function.hpp"
#include "function/polynomial.hpp"

#include "../test.hpp"

using std::cout; using std::cerr; using std::endl;
using namespace Ariadne;

extern template Ariadne::Nat Ariadne::Error<Ariadne::FloatDP>::output_places;
extern template Ariadne::Nat Ariadne::Error<Ariadne::FloatMP>::output_places;

inline Dyadic operator"" _exd (long double x) { return Dyadic(x); }

template<class T> concept HasClobber = requires(T& t) { t.clobber(); };

template<class T> void do_clobber(T& t) {
    if constexpr (HasClobber<T>) { t=t.clobber(); }
}

template<class T, class = decltype(declval<T>().norm())> decltype(declval<T>().norm()) norm(T& t, int=0) { return t.norm(); }
template<class T> T norm(T const& t) { return t; }


#define ARIADNE_TEST_REFINES(expression,expected)                         \
    {                                                                   \
        std::cout << "refines(" << #expression << "," << #expected << "): " << std::flush; \
        auto result = (expression); \
        Bool ok = decide(refines(result,(expected)));                       \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++ARIADNE_TEST_FAILURES;                                    \
            std::cout << "false\nERROR: refines(" << #expression << "," << #expected << "):\n"; \
            std::cout << "    " << #expression << "=" << (result) << "\n    #expected="<< (expected) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __PRETTY_FUNCTION__ << ": "; \
            std::cerr << "`refines(" << #expression << "," << #expected << ")' failed;\n"; \
            std::cerr << "result=" << (result) << "\nexpected="<< (expected) << std::endl; \
            auto exact_result=result; auto exact_expected=expected; clobber(exact_expected); clobber(exact_result); \
            auto difference=(exact_result-exact_expected); \
            std::cerr << "difference=" << difference << std::endl; \
            std::cerr<<"  norm(difference)="<<norm(difference)<<" result.error="<<(result).error()<<" expected.error="<<(expected).error()<<"\n"; \
        }                                                               \
    }                                                                   \

inline Vector<Dyadic> v(Nat n, Nat i) { return Vector<Dyadic>::unit(n,i); }
template<class F> ValidatedTaylorModel<F> ctm(Nat m, double c, Sweeper<F> swp) {
    typedef typename F::PrecisionType PR; return ValidatedTaylorModel<F>::constant(m,Float<PR>(c),swp); }
template<class F> ValidatedTaylorModel<F> ctm(Nat m, Sweeper<F> swp) {
    typedef typename F::PrecisionType PR; return ValidatedTaylorModel<F>::constant(m,Float<PR>(1.0_x),swp); }
//template<class F> ValidatedTaylorModel<F> tm(Nat m, Nat i, Sweeper swp) { return ValidatedTaylorModelType::coordinate(m,i,swp); }

// Allow addition and multiplication by double
template<class F, SameAs<double> D> ValidatedTaylorModel<F> operator+(D d, ValidatedTaylorModel<F> tm) {
    typedef typename F::PrecisionType PR; return Float<PR>(d)+tm; }
template<class F, SameAs<double> D> ValidatedTaylorModel<F> operator-(D d, ValidatedTaylorModel<F> tm) {
    typedef typename F::PrecisionType PR; return Float<PR>(d)-tm; }
template<class F, SameAs<double> D> ValidatedTaylorModel<F> operator*(D d, ValidatedTaylorModel<F> tm) {
    typedef typename F::PrecisionType PR; return Float<PR>(d)*tm; }

// Allow carat for power
template<class F> ValidatedTaylorModel<F> operator^(ValidatedTaylorModel<F> tm, Nat m) { return pow(tm,m); }
template<class F> ValidatedTaylorModel<F> clobber(ValidatedTaylorModel<F> tm) { tm.clobber(); return tm; }

template<class A> void display_difference(A const& res, A const& exp) {
    auto tol=exp.error();
    const_cast<A&>(exp).clobber();
    auto dif=res-exp;
    auto err=norm(dif);
    std::cerr<<"\nres="<<res<<"\nexp="<<exp<<"\ndif="<<dif<<"\nerr="<<err<<"\ntol="<<tol<<"\n";
}

template<class F> class TestTaylorModel
{
    typedef PrecisionType<F> PR;
    typedef MultiIndex MI;
    typedef Expansion<MI,F> E;
    typedef MultivariatePolynomial<F> P;
    typedef ValidatedTaylorModel<F> T;
    typedef Expansion<MI,F> ExpansionType;
  public:
    typedef TaylorModel<ValidatedTag,UpperInterval<F>> IntervalTaylorModelType;
    typedef TaylorModel<ValidatedTag,F> ValidatedTaylorModelType;
    typedef TaylorModel<ApproximateTag,F> ApproximateTaylorModelType;
    typedef F FloatType;
  public:
    Sweeper<F> swp; PR pr;
    ValidatedTaylorModelType x0,x1,x2,r,o;
  public:
    TestTaylorModel(Sweeper<F> swp);
    Void test();
  private:
    Void test_concept();
    Void test_constructors();
    Void test_assign();
    Void test_predicates();
    Void test_approximation();
    Void test_unscale();
    Void test_evaluate();
    Void test_arithmetic();
    Void test_interval_arithmetic();
    Void test_range();
    Void test_functions();
    Void test_rescale();
    Void test_restrict();
    Void test_refinement();
    Void test_split();
    Void test_antiderivative();
    Void test_compose();
    Void test_recondition();
};


template<class F> TestTaylorModel<F>::TestTaylorModel(Sweeper<F> sweeper)
    : swp(sweeper), pr(sweeper.precision())
    , x0(ValidatedTaylorModelType::coordinate(2,0,swp))
    , x1(ValidatedTaylorModelType::coordinate(2,1,swp))
    , x2(ValidatedTaylorModelType::zero(2,swp))
    , r(ValidatedTaylorModelType::unit_ball(2,swp))
    , o(ValidatedTaylorModelType::constant(2,Dyadic(1),swp))
{
}

template<class F> Void TestTaylorModel<F>::test()
{
    std::cerr<<std::setprecision(18);
    std::cout<<std::setprecision(18);
    std::clog<<std::setprecision(18);
    Float<PR>::set_output_places(18);
    FloatBounds<PR>::set_output_places(18);

    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_assign());
    ARIADNE_TEST_CALL(test_predicates());
    ARIADNE_TEST_CALL(test_approximation());
    ARIADNE_TEST_CALL(test_unscale());
    ARIADNE_TEST_CALL(test_arithmetic());
    ARIADNE_TEST_CALL(test_interval_arithmetic());
    ARIADNE_TEST_CALL(test_range());
    ARIADNE_TEST_CALL(test_functions());
    ARIADNE_TEST_CALL(test_rescale());
    ARIADNE_TEST_CALL(test_restrict());
    ARIADNE_TEST_CALL(test_refinement());
    ARIADNE_TEST_CALL(test_split());
    ARIADNE_TEST_CALL(test_antiderivative());
    ARIADNE_TEST_CALL(test_compose());
    ARIADNE_TEST_CALL(test_recondition());
}


template<class F> Void TestTaylorModel<F>::test_concept()
{
    const Float<PR> f={0,pr};
    const FloatBounds<PR> i;
    const Vector<Float<PR>> vf;
    const Vector<FloatBounds<PR>> vi;
    const ValidatedTaylorModelType  t(0,swp);
    ValidatedTaylorModelType tr(0,swp);

    tr=t+f; tr=t-f; tr=t*f; tr=t/f;
    tr=f+t; tr=f-t; tr=f*t; tr=f/t;
    tr=t+i; tr=t-i; tr=t*i; tr=t/i;
    tr=i+t; tr=i-t; tr=i*t; tr=i/t;
    tr=t+t; tr=t-t; tr=t*t; tr=t/t;

    tr+=f; tr-=f; tr*=f; tr/=f;
    tr+=i; tr-=i; tr*=i; tr/=i;
    tr+=t; tr-=t;

    tr=exp(t); tr=log(t); tr=sqrt(t);
    tr=sin(t); tr=cos(t); tr=tan(t);
    //tr=asin(t); tr=acos(t); tr=atan(t);

    tr.sweep(); tr.clobber();

    evaluate(t,vi);
    t.domain(); t.range(); t.expansion(); t.error();

}

template<class F> Void TestTaylorModel<F>::test_constructors()
{
    ARIADNE_TEST_CONSTRUCT(ExpansionType,raw_expansion,({ {{0,0},1.0_x}, {{1,0},2.0_x}, {{0,1},3.0_x}, {{2,0},4.0_x}, {{1,1},5.0_x}, {{0,2},6.0_x}, {{3,0},7.0_x}, {{2,1},8.0_x}, {{1,2},9.0_x}, {{0,3},10.0_x} },pr));
    ARIADNE_TEST_CONSTRUCT(F,raw_error,(0.25_x,pr));
    ARIADNE_TEST_CONSTRUCT(ExpansionType,expansion,(raw_expansion));
    ARIADNE_TEST_CONSTRUCT(FloatError<PR>,error,(raw_error));

    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tv,(raw_expansion, raw_error, swp));
    ARIADNE_TEST_PRINT(tv.expansion());
    ARIADNE_TEST_PRINT(tv.error());
    ARIADNE_TEST_EQUAL(tv.value().precision(),swp.precision());

    ARIADNE_TEST_EQUALS(tv.value(),1.0_x);
    ARIADNE_TEST_EQUALS(tv.error().raw(),0.25_x);
    ARIADNE_TEST_EQUALS(tv.norm().raw(),55.25_x);

    ARIADNE_TEST_EQUALS((tv[{0,0}]),1.0_x);
    ARIADNE_TEST_EQUALS((tv[{1,0}]),2.0_x);
    ARIADNE_TEST_EQUALS((tv[{0,1}]),3.0_x);
    ARIADNE_TEST_EQUALS((tv[{2,1}]),8.0_x);

}

template<class F> Void TestTaylorModel<F>::test_assign()
{
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tv,(1u,swp));
    ARIADNE_TEST_ASSIGN(tv,Bounds<F>(1,3,pr));
    ARIADNE_TEST_EQUALS((tv[{0}]),2);
    ARIADNE_TEST_SAME(tv.error(),1);
    ARIADNE_TEST_ASSIGN(tv,Bounds<F>(5,pr));
    ARIADNE_TEST_EQUALS((tv[{0}]),5);
    ARIADNE_TEST_SAME(tv.error(),0);
}


template<class F> Void TestTaylorModel<F>::test_predicates()
{
    ARIADNE_TEST_REFINES(1+x0+(x0^2)/2+(x0^3)/4,1+x0+(x0^2)/2+r/4);
    ARIADNE_TEST_BINARY_PREDICATE(not refines,1+x0+(x0^2)/2+(x0^3)/6+r/1048576,1+x0+(x0^2)/2+r/6);

    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tv1,(1+2*x0+3*(x0^2)+r*3/4));
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tv2,(1+x0*7/4+(x0^2)*13/4+r/4));
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tv3,(o*9/8+x0*7/4+(x0^2)*13/4+r/4));
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tv4,(1+x0*9/4+(x0^2)*3-(x0^4)/4+r/4));

    ARIADNE_TEST_REFINES(tv1,tv1);
    ARIADNE_TEST_REFINES(tv2,tv1);
    ARIADNE_TEST_BINARY_PREDICATE(not refines,tv3,tv1);
    ARIADNE_TEST_REFINES(tv4,tv1);

    Float<PR> h(Dyadic(0.5_x),pr);
    ARIADNE_TEST_BINARY_PREDICATE(consistent,1+2*x0+3*x1+r*3/4,1+2*x0+3*x1+r*3/4);
    ARIADNE_TEST_BINARY_PREDICATE(consistent,1+r*3/4,2+r/4);
    ARIADNE_TEST_BINARY_PREDICATE(not consistent,x0-(x0^3)*2/3+r/(3.0_x+pow(h,20)),x2);
    ARIADNE_TEST_BINARY_PREDICATE(inconsistent,1-2*(x0^2),r*7/8);
    ARIADNE_TEST_BINARY_PREDICATE(not inconsistent,1+2*x0+3*x1+r*3/4,1+2*x0+3*x1+r*3/4);
    ARIADNE_TEST_BINARY_PREDICATE(not inconsistent,1+r*3/4,2+r/4);

    if(not refines(1-(x0^2)/2,r/2)) {
        ARIADNE_TEST_WARN("refines(ValidatedTaylorModelType,ValidatedTaylorModelType) may return false even if the first model refines the second.");
    }

    if(not consistent(x0-(x0^3)*2/3+r/3,x2)) {
        ARIADNE_TEST_WARN("consistent(ValidatedTaylorModelType,ValidatedTaylorModelType) may return false even if models are consistent with representing the same function.");
    }

    if(not inconsistent(x0-(x0^3)*2/3+r/4,x2)) {
        ARIADNE_TEST_WARN("inconsistent(ValidatedTaylorModelType,ValidatedTaylorModelType) may return false even if models are consistent with representing the same function.");
    }

}


template<class F> Void TestTaylorModel<F>::test_approximation()
{
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tv2,({{{0,0},1.0_x},{{1,0},2.0_x},{{0,1},3.0_x}},0.25_x, swp));
}


template<class F> Void TestTaylorModel<F>::test_unscale()
{

    ExactIntervalType is_singleton(1.0_x,1.0_x);
    if(unscale(3*o,is_singleton).codomain()!=is_singleton) {
        ARIADNE_TEST_WARN("Unscaling over singleton domain does not yield constant");
    }
}

template<class F> Void TestTaylorModel<F>::test_evaluate()
{
    Vector<FloatBounds<PR>> iv={{0.25_x,0.5_x},{-0.75_x,-0.5_x}};
    ValidatedTaylorModelType tv=1+2*x0+3*x1+4*(x0^2)+5*(x0*x1)+6*(x1^2)+r/2;
    ARIADNE_TEST_EQUAL(evaluate(tv,iv),FloatBounds<PR>(-1,1));
}


template<class F> Void TestTaylorModel<F>::test_arithmetic()
{
    //Operations which can be performed exactly with floating-point arithmetic.
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)+(-3), ValidatedTaylorModelType({{{0},-2.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)-(-3), ValidatedTaylorModelType({{{0},4.0_x},{{1},-2.0_x},{{2},3.0_x}}, 0.75_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)*(-3), ValidatedTaylorModelType({{{0},-3.0_x},{{1},6.0_x},{{2},-9.0_x}}, 2.25_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)/(-4), ValidatedTaylorModelType({{{0},-0.25_x},{{1},0.5_x},{{2},-0.75_x}}, 0.1875_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)+FloatBounds<PR>(-1,2,pr), ValidatedTaylorModelType({{{0},1.5_x},{{1},-2.0_x},{{2},3.0_x}}, 2.25_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)-FloatBounds<PR>(-1,2,pr), ValidatedTaylorModelType({{{0},0.5_x},{{1},-2.0_x},{{2},3.0_x}}, 2.25_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)*FloatBounds<PR>(-1,2,pr), ValidatedTaylorModelType({{{0},0.5_x},{{1},-1.0_x},{{2},1.5_x}}, 10.5_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)/FloatBounds<PR>(0.25_x,2.0_x,pr), ValidatedTaylorModelType({{{0},2.25_x},{{1},-4.5_x},{{2},6.75_x}}, 13.5_x,swp));
    ARIADNE_TEST_SAME(+ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp), ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp));
    ARIADNE_TEST_SAME(-ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp), ValidatedTaylorModelType({{{0},-1.0_x},{{1},2.0_x},{{2},-3.0_x}}, 0.75_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)+ValidatedTaylorModelType({{{0},3.0_x},{{1},2.0_x},{{2},-4.0_x}},0.5_x,swp),
                      ValidatedTaylorModelType({{{0},4.0_x},{{1},0.0_x},{{2},-1.0_x}}, 1.25_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)-ValidatedTaylorModelType({{{0},3.0_x},{{1},2.0_x},{{2},-4.0_x}},0.5_x,swp),
                      ValidatedTaylorModelType({{{0},-2.0_x},{{1},-4.0_x},{{2},7.0_x}}, 1.25_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},0.0_x},{{1},0.0_x},{{2},3.0_x}}, 0.75_x,swp)*ValidatedTaylorModelType({{{0},3.0_x},{{1},2.0_x},{{2},-4.0_x}},0.5_x,swp),
                      ValidatedTaylorModelType({{{0},0.0_x},{{1},0.0_x},{{2},9.0_x},{{3},6.0_x},{{4},-12.0_x}}, 8.625_x,swp));
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},1.0_x},{{1},-2.0_x},{{2},3.0_x}},0.75_x,swp)*ValidatedTaylorModelType({{{0},3.0_x},{{1},2.0_x},{{2},-4.0_x}},0.5_x,swp),
                      ValidatedTaylorModelType({{{0},3.0_x},{{1},-4.0_x},{{2},1.0_x},{{3},14.0_x},{{4},-12.0_x}}, 10.125_x,swp));

    ARIADNE_TEST_SAME(1-2*x0+3*(x0^2)+r*3/4, ValidatedTaylorModelType({{{0,0},1.0_x},{{1,0},-2.0_x},{{2,0},3.0_x}}, 0.75_x,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,t,(1-2*x0+3*(x0^2)+r*3/4));
    ARIADNE_TEST_SAME(t,ValidatedTaylorModelType({{{0,0},1.0_x},{{1,0},-2.0_x},{{2,0},3.0_x}},0.75_x,swp));

    // Reciprocal of a constant
    ARIADNE_TEST_SAME(rec(o*4),o/4);

    ARIADNE_TEST_SAME((1-2*x0+3*x1)*(3+2*x0-4*x1),3-4*x0+5*x1-4*(x0^2)+14*(x0*x1)-12*(x1^2));
    ARIADNE_TEST_SAME(sqr(t),t*t);
    ARIADNE_TEST_SAME(pow(t,0),o);
    ARIADNE_TEST_SAME(pow(t,1),t);
    ARIADNE_TEST_SAME(pow(t,2),t*t);
    ARIADNE_TEST_SAME(pow(t,3),t*t*t);

    F inf_ = F::inf(pr);
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tm_inf,(Expansion<MultiIndex,FloatType>(2,pr),+inf_,swp));
    ARIADNE_TEST_CONSTRUCT(ValidatedTaylorModelType,tm_zero_times_inf,(0*tm_inf));
    if(is_nan(tm_zero_times_inf.error().raw())) {
        ARIADNE_TEST_WARN("Multiplying 0+/-inf by 0 yields 0+/-NaN");
    } else if(tm_zero_times_inf.error().raw()==+inf_) {
        ARIADNE_TEST_WARN("Multiplying 0+/-inf by 0 yields 0+/-inf");
    } else if(tm_zero_times_inf.error().raw()==0.0_x) {
        ARIADNE_TEST_WARN("Multiplying 0+/-inf by 0 yields 0+/-0");
    }


    // Regression test for multiplication
    //   (5+7*x+11*x^2±2) * (13+17*x+19*x^2±3) = (65+176*x+357*x^2+320*x^3+209*x^4 ± 69 ± 98 ± 6)
    //        = (65+176*x+357*x^2+320*x^3+209*x^4 ± 173)
    //        = (65+176*x+357*x^2) ± 529 ± 173
    //        = (65+176*x+357*x^2) ± 702
    GradedSweeper<F> gswp4(pr,4);
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},5.0_x},{{1},7.0_x},{{2},11.0_x}},2.0_x,gswp4)*ValidatedTaylorModelType({{{0},13.0_x},{{1},17.0_x},{{2},19.0_x}},3.0_x,gswp4),
                    ValidatedTaylorModelType({{{0},65.0_x},{{1},176.0_x},{{2},357.0_x},{{3},320.0_x},{{4},209.0_x}}, 173.0_x,gswp4));
    GradedSweeper<F> gswp2(pr,2);
    ARIADNE_TEST_SAME(ValidatedTaylorModelType({{{0},5.0_x},{{1},7.0_x},{{2},11.0_x}},2.0_x,gswp2)*ValidatedTaylorModelType({{{0},13.0_x},{{1},17.0_x},{{2},19.0_x}},3.0_x,gswp2),
                    ValidatedTaylorModelType({{{0},65.0_x},{{1},176.0_x},{{2},357.0_x}}, 702.0_x,gswp2));
}


template<class F> Void TestTaylorModel<F>::test_interval_arithmetic()
{
    IntervalTaylorModelType xi0=IntervalTaylorModelType::coordinate(2,0,swp);
    IntervalTaylorModelType xi1=IntervalTaylorModelType::coordinate(2,1,swp);
    IntervalTaylorModelType ri=IntervalTaylorModelType::unit_ball(2,swp);
    IntervalTaylorModelType oi=IntervalTaylorModelType::constant(2,1,swp);

    typedef typename IntervalTaylorModelType::CoefficientType CoefficientType;
    typedef typename IntervalTaylorModelType::ErrorType ErrorType;

    CoefficientType c(-1,2,pr);
    CoefficientType d1(2,3,pr);
    CoefficientType d2(5,7,pr);

    auto rd1=rec(d1);
    auto rd2=rec(d2);

    InitializerList<DegreeType> i0={0}, i1={1}, i2={2}, i3={3};
    CoefficientType a0(1,pr), a1(-2,pr), a2(3,pr);
    CoefficientType b0(4,pr), b1(-2,pr), b2(3,pr), b3(5,pr);

    ARIADNE_TEST_CONSTRUCT(ErrorType,e,(pr));
    IntervalTaylorModelType tm1({{i0,a0},{i1,a1},{i2,a2}},e,swp);
    ARIADNE_TEST_PRINT(tm1);
    IntervalTaylorModelType tm1pc=tm1+c;
    ARIADNE_TEST_PRINT(tm1pc);

    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp)+c, IntervalTaylorModelType({{i0,a0+c},{i1,a1},{i2,a2}},e,swp));
    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp)-c, IntervalTaylorModelType({{i0,a0-c},{i1,a1},{i2,a2}},e,swp));
    ARIADNE_TEST_SAME(c-IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp), IntervalTaylorModelType({{i0,c-a0},{i1,-a1},{i2,-a2}},e,swp));
    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp)*c, IntervalTaylorModelType({{i0,a0*c},{i1,a1*c},{i2,a2*c}},e,swp));
    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp)/d1, IntervalTaylorModelType({{i0,a0*rd1},{i1,a1*rd1},{i2,a2*rd1}},e,swp));
    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp)/d2, IntervalTaylorModelType({{i0,a0*rd2},{i1,a1*rd2},{i2,a2*rd2}},e,swp));

    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp) + IntervalTaylorModelType({{i0,b0},{i1,b1}},e,swp),
                      IntervalTaylorModelType({{i0,a0+b0},{i1,a1+b1},{i2,a2}},e,swp));
    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1}},e,swp) - IntervalTaylorModelType({{i0,b0},{i1,b1},{i2,b2}},e,swp),
                      IntervalTaylorModelType({{i0,a0-b0},{i1,a1-b1},{i2,-b2}},e,swp));
    ARIADNE_TEST_SAME(IntervalTaylorModelType({{i0,a0},{i1,a1},{i2,a2}},e,swp) * IntervalTaylorModelType({{i0,b0},{i1,b1}},e,swp),
                      IntervalTaylorModelType({{i0,a0*b0},{i1,a0*b1+a1*b0},{i2,a1*b1+a2*b0},{i3,a2*b1}},e,swp));

    ARIADNE_TEST_CONSTRUCT(IntervalTaylorModelType,ti,(1-2*xi0+3*(xi0^2)+ri*3/4));
    ARIADNE_TEST_SAME(ti,IntervalTaylorModelType({{{0,0},1.0_x},{{1,0},-2.0_x},{{2,0},3.0_x}},0.75_x,swp));
    // Reciprocal of a constant
    ARIADNE_TEST_SAME(rec(oi*4),oi/4);

//    ARIADNE_TEST_SAME((1-2*x0+3*x1)*(3+2*x0-4*x1),3-4*x0+5*x1-4*(x0^2)+14*(x0*x1)-12*(x1^2));
    ARIADNE_TEST_SAME(sqr(ti),ti*ti);
    ARIADNE_TEST_SAME(pow(ti,0),oi);
    ARIADNE_TEST_SAME(pow(ti,1),ti);
    ARIADNE_TEST_SAME(pow(ti,2),ti*ti);
    ARIADNE_TEST_SAME(pow(ti,3),ti*ti*ti);
}

template<class F> Void TestTaylorModel<F>::test_range()
{
    typedef Interval<Float<PR>> ExactRangeType;

    // Test range of cubic, which should be exact
    ValidatedTaylorModelType t1 = x0*x0*x0+x0;
    ARIADNE_TEST_SAME(t1.range(),ExactRangeType(-2,+2,pr));

    //x^2+x = (x+1/2)^2-1/4; Range [-0.25_x,+2.0_x]
    // Test range of quadratic, which could be exact, but need not be
    ValidatedTaylorModelType t2 = x0*x0+x0;
    ARIADNE_TEST_BINARY_PREDICATE(refines,t2.range(),ExactRangeType(-2,+2,pr));
    ARIADNE_TEST_BINARY_PREDICATE(refines,ExactRangeType(-0.25_x,+2,pr),t2.range());
    if(cast_exact_interval(t2.range())!=ExactRangeType(-0.25_x,2.0_x,pr)) {
        ARIADNE_TEST_WARN("ValidatedTaylorModelType::range() not exact for quadratic functions."); }
}


template<class F> Void TestTaylorModel<F>::test_functions()
{
    ValidatedTaylorModelType x=ValidatedTaylorModelType::coordinate(1,0,swp);
    ValidatedTaylorModelType hx=x/2;
    ValidatedTaylorModelType ophx=1+x/2;
    FloatBounds<PR> e(-1,+1,pr);
    FloatError<PR>::set_output_places(8);

    ARIADNE_TEST_PRINT(exp(x));
    ARIADNE_TEST_PRINT(sin(x));
    ARIADNE_TEST_PRINT(cos(x));
    ARIADNE_TEST_PRINT(x.tolerance());

    const ExactDouble LAXITY=128;
    // Threshold based on sweeper characteristics
    static const Float<PR> threshold=Float<PR>(x.tolerance());
    static const FloatBounds<PR> tolerance=threshold*FloatBounds<PR>(-1,+1,pr)*LAXITY;

    // exp, sin and cos have error bound e^N/N!*(1+1/N), where e is bound for |x|, N is the first term omitted
    // ErrorTag bound for rec is e^(N-1); log is e^(N-1)/N; sqrt is ???, where e is bound for |x-1|

    //Functions based on their natural defining points with variable dependence 1.0_x
    ValidatedTaylorModelType expected_exp_x
        = 1+x+(x^2)/2+(x^3)/6+(x^4)/24+(x^5)/120+(x^6)/720+(x^7)/5040+(x^8)/40320+(x^9)/362880+(x^10)/3628800+(x^11)/39916800+(x^12)/479001600+e/5782233600;
    ValidatedTaylorModelType expected_sin_x
        = x-(x^3)/6+(x^5)/120-(x^7)/5040+(x^9)/362880-(x^11)/39916800+e/5782233600;
    ValidatedTaylorModelType expected_cos_x
        = 1-(x^2)/2+(x^4)/24-(x^6)/720+(x^8)/40320-(x^10)/3628800+(x^12)/479001600+e/81366405120;
    ARIADNE_TEST_REFINES(exp(x),expected_exp_x+tolerance);
    ARIADNE_TEST_REFINES(sin(x),expected_sin_x+tolerance);
    ARIADNE_TEST_REFINES(cos(x),expected_cos_x+tolerance);

    //Functions based on their natural defining points with variable dependence 0.5_x
    //ValidatedTaylorModelType expected_exp_hx = 1+hx+(hx^2)/2+(hx^3)/6+(hx^4)/24+(hx^5)/120+(hx^6)/720+(hx^7)/5040+e/128/4410+tol;
    ValidatedTaylorModelType expected_exp_hx
        = 1+x/2+(x^2)/8+(x^3)/48+(x^4)/384+(x^5)/3840+(x^6)/46080+(x^7)/645120+(x^8)/10321920+(x^9)/185794560+(x^10)/3715891200+e/2048/36590400;
    ValidatedTaylorModelType expected_sin_hx
        = x/2-(x^3)/48+(x^5)/3840-(x^7)/645120+(x^9)/185794560+e/2048/36590400;
    ValidatedTaylorModelType expected_cos_hx
        = 1-(x^2)/8+(x^4)/384-(x^6)/46080+(x^8)/10321920-(x^10)/3715891200+e/4096/5748019200*13;
    // Uncomment the last line so that expected_cos_hx contains error term base on x^8 term rather than x^7 term.
    //ValidatedTaylorModelType expected_cos_hx = 1-(hx^2)/2+(hx^4)/24-(hx^6)/720+e/256/35840+tol;

    ARIADNE_TEST_REFINES(exp(hx),expected_exp_hx+tolerance);
    ARIADNE_TEST_REFINES(sin(hx),expected_sin_hx+tolerance);
    ARIADNE_TEST_REFINES(cos(hx),expected_cos_hx+tolerance);

    // Series for sqrt(1+x) is
    // 1, 1/2, -1/8, 1/16, -5/128, 7/256, -21/1024, 33/2048, -273/16384, 35/2048, -357/20480
    // Series for sqrt(1+x/2) is
    // 1, 1/4, -1/32, 1/128, -5/2048, 7/8192, -21/65536, 33/262144, 429/8388608, 715/33554432, 2431/268435456
    ValidatedTaylorModelType expected_rec_ophx
        = 1-x/2+(x^2)/4-(x^3)/8+(x^4)/16-(x^5)/32+(x^6)/64-(x^7)/128+(x^8)/256-(x^9)/512+(x^10)/1024+e/1024;
    ValidatedTaylorModelType expected_rec_ophx_coarse_error
        = 1-x/2+(x^2)/4-(x^3)/8+(x^4)/16-(x^5)/32+(x^6)/64-(x^7)/128+(x^8)/256-(x^9)/512+(x^10)/1024+e/1024*2;
    ValidatedTaylorModelType expected_sqrt_ophx
        = 1+x/4-(x^2)/32+(x^3)/128-(x^4)*5/2048+(x^5)*7/8192-(x^6)*21/65536+(x^7)*33/262144-(x^8)*429/8388608+(x^9)*715/33554432+e*2431/268435456;
    ValidatedTaylorModelType expected_sqrt_ophx_coarse_error
        = 1+x/4-(x^2)/32+(x^3)/128-(x^4)*5/2048+(x^5)*7/8192-(x^6)*21/65536+(x^7)*33/262144-(x^8)*429/8388608+(x^9)*715/33554432+e*2431/268435456*2;
    ValidatedTaylorModelType expected_log_ophx
        = x/2-(x^2)/4/2+(x^3)/8/3-(x^4)/16/4+(x^5)/32/5-(x^6)/64/6+(x^7)/128/7-(x^8)/256/8+(x^9)/512/9-(x^10)/1024/10+e/1024/10;
    ValidatedTaylorModelType expected_log_ophx_coarse_error
        = x/2-(x^2)/4/2+(x^3)/8/3-(x^4)/16/4+(x^5)/32/5-(x^6)/64/6+(x^7)/128/7-(x^8)/256/8+(x^9)/512/9-(x^10)/1024/10+e/1024/10;
    ARIADNE_TEST_REFINES(rec(ophx),expected_rec_ophx_coarse_error+tolerance);
    ARIADNE_TEST_REFINES(sqrt(ophx),expected_sqrt_ophx_coarse_error+tolerance);
    ARIADNE_TEST_REFINES(log(ophx),expected_log_ophx_coarse_error+tolerance);

    ARIADNE_TEST_REFINES(sqrt(ophx)*sqrt(ophx),ophx+tolerance*2);

    // Doubling formulae
    ARIADNE_TEST_REFINES(exp(2*x),(expected_exp_x^2)+tolerance);
    ARIADNE_TEST_REFINES(sin(2*x),2*expected_sin_x*expected_cos_x+tolerance);
    ARIADNE_TEST_REFINES(cos(2*x),2*(expected_cos_x^2)-1+tolerance);

    ARIADNE_TEST_REFINES(3*rec(3*ophx),expected_rec_ophx_coarse_error+tolerance);
    ARIADNE_TEST_REFINES(sqrt(9*ophx)/3,expected_sqrt_ophx_coarse_error+tolerance);
    ARIADNE_TEST_REFINES(log(3*ophx),expected_log_ophx_coarse_error+log(Float<PR>(3,pr))+tolerance);

    Nat rn=3; Float<PR> c(2,pr); Float<PR> frn(rn,pr);
    ARIADNE_TEST_PRINT(c);
    ARIADNE_TEST_PRINT(frn);
    ARIADNE_TEST_REFINES(exp(hx),expected_exp_hx+tolerance);
    ARIADNE_TEST_REFINES(exp(c+frn*hx),exp(c)*pow(expected_exp_hx,rn)+tolerance);
    ARIADNE_TEST_REFINES(sin(c+hx),sin(c)*expected_cos_hx+cos(c)*expected_sin_hx+tolerance);
    ARIADNE_TEST_REFINES(cos(c+hx),cos(c)*expected_cos_hx-sin(c)*expected_sin_hx+tolerance);

    c=Float<PR>(10,pr);
    ARIADNE_TEST_REFINES(sin(c+hx),sin(c)*expected_cos_hx+cos(c)*expected_sin_hx+tolerance);
    ARIADNE_TEST_REFINES(cos(c+hx),cos(c)*expected_cos_hx-sin(c)*expected_sin_hx+tolerance);

    // Test exponential based at log2; exp(log(2)+x/2)=2*exp(x/2)
    Float<PR> log2_apprx(0.693147_pr,pr);
    ARIADNE_TEST_REFINES(exp(log2_apprx+x/2),
                                  2*(1+hx+(hx^2)/2+(hx^3)/6+(hx^4)/24+(hx^5)/120+(hx^6)/720)+e*6/100000);
}


template<class F> Void TestTaylorModel<F>::test_rescale()
{
}

template<class F> Void TestTaylorModel<F>::test_restrict()
{
}

template<class F> Void TestTaylorModel<F>::test_refinement()
{
    ValidatedTaylorModelType x=ValidatedTaylorModelType::coordinate(2,0,swp);
    ValidatedTaylorModelType y=ValidatedTaylorModelType::coordinate(2,1,swp);
    ValidatedTaylorModelType e=ValidatedTaylorModelType::unit_ball(2,swp);

    ARIADNE_TEST_REFINES(1+x+(x^2)/2+(x^3)/4,1+x+(x^2)/2+e/4);
    ARIADNE_TEST_BINARY_PREDICATE(not refines,1+x+(x^2)/2+(x^3)/6+e/static_cast<Dyadic>(pow(two,31)),1+x+(x^2)/2+e/6);

    // Test refinement with no roundoff errors
    ARIADNE_TEST_SAME(refinement(1-x*3/4+(x^3)*3+(x^4)*13/4+e/2,1+(x^2)/4+(x^3)*2+(x^4)*3+e),1-x*5/8+(x^3)*11/4+(x^4)*13/4+e/2);

    if (Same<F,FloatDP>) {
        // Test refinement with roundoff errors
        ARIADNE_TEST_SAME(refinement(0.66666666666666663_pr*x+e/2,1.19999999999999996_pr*x+e/4),
                        1.05833333333333335_pr*x+0.108333333333333393_pr*e);
    }

    // Code below computes expected values for second test
    // FloatDP xv=2./3; FloatDP xe=1./2; FloatDP yv=6./5; FloatDP ye=1./4;
    // FloatDP rl=sub(down,yv,ye); FloatDP ru=add(up,xv,xe); FloatDP rv=add(near,rl,ru)/2; FloatDP re=sub(up,ru,rl)/2;
    // std::cerr << std::setprecision(18) << "xv="<<xv<<" yv="<<yv<<" rl="<<rl<<" ru="<<ru<<" rv="<<rv<<" re="<<re<<"\n";
}

template<class F> Void TestTaylorModel<F>::test_split()
{
    ValidatedTaylorModelType x=ValidatedTaylorModelType::coordinate(2,0,swp);
    ValidatedTaylorModelType y=ValidatedTaylorModelType::coordinate(2,1,swp);
    ValidatedTaylorModelType z=ValidatedTaylorModelType::zero(2,swp);
    ValidatedTaylorModelType t=1+3*x+2*y-5*x*x-7*x*y+11*y*y;
    ValidatedTaylorModelType es1=-1.75_x+4*x+5.5_x*y-1.25_x*x*x-3.5_x*x*y+11*y*y;
    ValidatedTaylorModelType es2=1+1.5_x*x+2*y-1.25_x*x*x-3.5_x*x*y+11*y*y;
    ValidatedTaylorModelType es3=1.25_x-1*x-1.5_x*y-1.25_x*x*x-3.5_x*x*y+11*y*y;

    ARIADNE_TEST_PRINT(t);
    ARIADNE_TEST_SAME(split(t,0,SplitPart::LOWER),es1);
    ARIADNE_TEST_SAME(split(t,0,SplitPart::MIDDLE),es2);
    ARIADNE_TEST_SAME(split(t,0,SplitPart::UPPER),es3);
}


template<class F> Void TestTaylorModel<F>::test_antiderivative()
{
    FloatBounds<PR> unit_interval(-1,+1,pr);
    ValidatedTaylorModelType tm=ValidatedTaylorModelType::constant(2,1,swp);
    ValidatedTaylorModelType atm=antiderivative(tm,1);

    ARIADNE_TEST_SAME(antiderivative(2*o,0),2*x0);
    ARIADNE_TEST_SAME(antiderivative(2*o,1),2*x1);
    ARIADNE_TEST_SAME(antiderivative(3*x0,0),(x0^2)*3/2);
    ARIADNE_TEST_SAME(antiderivative(2*o+3*x0,0),x0*2+(x0^2)*3/2);
    ARIADNE_TEST_SAME(antiderivative((x0^2)*15/2,0),(x0^3)*5/2);
    ARIADNE_TEST_SAME(antiderivative((x0^2)*(x1^4)*15/2,0),(x0^3)*(x1^4)*5/2);
    ARIADNE_TEST_SAME(antiderivative((x0^2)*(x1^4)*15/2,1),(x0^2)*(x1^5)*3/2);

    if constexpr (Same<F,FloatDP>) {
        ValidatedTaylorModelType x=ValidatedTaylorModelType::coordinate(1,0,swp);
        ValidatedTaylorModelType e=ValidatedTaylorModelType::zero(1,swp)+FloatBounds<PR>(-1,+1,pr);
        ARIADNE_TEST_SAME(antiderivative(2*x*x,0),0.66666666666666663_pr*x*x*x+5.5511151231257827021e-17_pr*e);
        ARIADNE_TEST_SAME(antiderivative(2*x*x+e,0),0.66666666666666663_pr*x*x*x+1.0000000000000002_pr*e);
        ARIADNE_TEST_SAME(antiderivative(2*(x^2),0),Float<PR>(0.66666666666666663_pr,pr)*(x^3)+Float<PR>(5.5511151231257827021e-17_pr,pr)*e);
        ARIADNE_TEST_SAME(antiderivative(2*(x^2)+e,0),Float<PR>(0.66666666666666663_pr,pr)*(x^3)+Float<PR>(1.0000000000000002_pr,pr)*e);

        // Regression test
        ValidatedTaylorModelType t1({ {{0,0},1.0_x}, {{1,0},2.0_x}, {{0,1},3.0_x}, {{2,0},4.0_x}, {{1,1},5.0_x}, {{0,2},6.0_x} }, 0.0_x, swp);
        ValidatedTaylorModelType at1({ {{1,0},1.0_x}, {{2,0},1.0_x}, {{1,1},3.0_x}, {{3,0},1.33333333333333333_pr}, {{2,1},2.5_x}, {{1,2},6.0_x} }, 1.1102230246251565404e-16_pr, swp);
        ARIADNE_TEST_SAME(antiderivative(t1,0),at1);
    }
}


template<class F> Void TestTaylorModel<F>::test_compose()
{
    ARIADNE_TEST_SAME(compose(2-x0*x0-x1/4,{2-x0*x0-x1/4,x0}),-2-(x0^4)-(x1^2)/16+4*(x0^2)+x1-(x0^2)*x1/2-x0/4);

    // Regression test from failure in integration routing
    if constexpr (Same<F,FloatDP>) {
        auto id=ValidatedTaylorModelType::coordinates(2,ThresholdSweeper<FloatDP>(double_precision,1e-10));
        auto s0=id[0];
        auto s1=id[1];
        auto x=( 0.3750000000000000_pr +0.1250000000000000_pr*s0 +0.0292968750000000_pr*s1 +0.0039062500000000_pr*s0*s1 +0.0004577636718750_pr*(s1^2) -0.0019531250000000_pr*(s0^2)*s1 -0.0003967285156250_pr*s0*(s1^2) -0.0000309944152832_pr*(s1^3) -0.0000915527343750_pr*(s0^2)*(s1^2) -0.0000184377034505_pr*s0*(s1^3) -0.0000010803341866_pr*(s1^4) +0.0000305175781250_pr*(s0^3)*(s1^2) +0.0000073115030924_pr*(s0^2)*(s1^3) +0.0000007127722104_pr*s0*(s1^4) +0.0000000334111974_pr*(s1^5) +0.0000019073486328_pr*(s0^3)*(s1^3) +0.0000005215406418_pr*(s0^2)*(s1^4) +0.0000000533492615_pr*s0*(s1^5) +0.0000000020839555_pr*(s1^6) -0.0000004768371582_pr*(s0^4)*(s1^3) -0.0000001241763433_pr*(s0^3)*(s1^4) -0.0000000131704534_pr*(s0^2)*(s1^5) -0.0000000007510809_pr*s0*(s1^6) -0.0000000372529030_pr*(s0^4)*(s1^4) -0.0000000125728548_pr*(s0^3)*(s1^5) -0.0000000017495040_pr*(s0^2)*(s1^6) -0.0000000001205073_pr*s0*(s1^7) +0.0000000074505806_pr*(s0^5)*(s1^4) +0.0000000019790605_pr*(s0^4)*(s1^5) +0.0000000001813735_pr*(s0^3)*(s1^6) +0.0000000006984919_pr*(s0^5)*(s1^5) +0.0000000002758801_pr*(s0^4)*(s1^6) -0.0000000001164153_pr*(s0^6)*(s1^5));
        auto y=id;
        y[1]=(y[1]+1)/2;

        ARIADNE_TEST_COMPARE(compose(x,y).error().raw(),<=,1e-8_pr);
    }
}

template<class F> Void TestTaylorModel<F>::test_recondition()
{
    ARIADNE_TEST_CONSTRUCT( ValidatedTaylorModelType, tm1, ({ {{0,0},2.0_x}, {{1,0},3.0_x}, {{0,1},5.0_x} }, 0.5_x, swp) );
    ARIADNE_TEST_CONSTRUCT( ValidatedTaylorModelType, tm2, ({ {{0,0,1},0.5_x}, {{0,0,0},2.0_x}, {{1,0,0},3.0_x}, {{0,1,0},5.0_x} }, 0.0_x, swp) );
    ARIADNE_TEST_CONSTRUCT( ValidatedTaylorModelType, tm3, ({ {{0},2.0_x}, {{1},5.0_x} }, 3.5_x, swp) );
    ARIADNE_TEST_SAME(embed_error(tm1),tm2);
    ARIADNE_TEST_SAME(discard_variables(tm2,{2}),tm1);
    ARIADNE_TEST_SAME(discard_variables(tm1,{0}),tm3);
}




Int main() {
    ThresholdSweeper<FloatDP> sweeper_dp(dp,1e-8);
    TestTaylorModel<FloatDP>(sweeper_dp).test();
    MultiplePrecision mp(128);
    ThresholdSweeper<FloatMP> sweeper_mp(mp,FloatMP(pow(two,-64),mp));
    ARIADNE_TEST_PRINT(sweeper_mp.precision());
    ARIADNE_TEST_PRINT(sweeper_mp);
    TestTaylorModel<FloatMP>(sweeper_mp).test();

    RelativeThresholdSweeper<FloatMP> relative_sweeper_mp(mp,FloatMP(pow(two,-64),mp));
    TestTaylorModel<FloatMP>(relative_sweeper_mp).test();

    return ARIADNE_TEST_FAILURES;
}



