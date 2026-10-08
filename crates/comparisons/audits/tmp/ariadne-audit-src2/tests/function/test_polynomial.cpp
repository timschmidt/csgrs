/***************************************************************************
 *            test_polynomial.cpp
 *
 *  Copyright  2009-20  Pieter Collins
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
#include "config.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/expansion.hpp"
#include "algebra/expansion.inl.hpp"
#include "algebra/algebra.hpp"
#include "function/polynomial.hpp"
#include "function/polynomial.tpl.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;

class TestPolynomial
{
    typedef MultiIndex MI;
    typedef MultivariatePolynomial<RoundedFloatDP> P;
  public:
    Void test();
  private:
    Void test_concept();
    Void test_cleanup();
    Void test_constructors();
    Void test_indexing();
    Void test_arithmetic();
    Void test_variables();
    Void test_find();
    Void test_flow();
};


Void TestPolynomial::test()
{
    ARIADNE_TEST_CALL(test_cleanup())
    ARIADNE_TEST_CALL(test_constructors())
    ARIADNE_TEST_CALL(test_indexing())
    ARIADNE_TEST_CALL(test_arithmetic())
    ARIADNE_TEST_CALL(test_variables())

    ARIADNE_TEST_CALL(test_find())
    ARIADNE_TEST_CALL(test_flow())
}


Void TestPolynomial::test_concept()
{
    RoundedFloatDP x(0,dp);
    Vector<RoundedFloatDP> v(3u,dp);
    MultiIndex a(3u);
    MultivariatePolynomial<RoundedFloatDP> p(3u,dp);
    const MultivariatePolynomial<RoundedFloatDP> cp(3u,dp);
    Vector< MultivariatePolynomial<RoundedFloatDP> > pv(2u,cp);

    p=MultivariatePolynomial<RoundedFloatDP>(3u,dp);
    p=MultivariatePolynomial<RoundedFloatDP>(cp);

    p=MultivariatePolynomial<RoundedFloatDP>({ {{0,0,0},1.0_x}, {{1,0,0},2.0_x}, {{0,0,0},3.0_x}, {{0,0,1},5.0_x} },dp);

    //p=MultivariatePolynomial<ApproximateDouble>::variable(3u,0u);
    //p=MultivariatePolynomial<ApproximateDouble>::variables(3u)[0u];

    p=x;

    p.reserve(2u);
    p.insert(a,x);

    x=cp[a];
    p[a]=1.0_x;

    cp.argument_size();

    p.erase(p.begin());

    p.check();
    p.cleanup();
    p.clear();

    evaluate(p,v);
    compose(p,pv);

}

Void TestPolynomial::test_cleanup()
{
/*
    {
        MultiIndex a(3);
        MultiIndex b(3); ++b;
        std::vector<ValueType> v;

        for(Nat i=0; i!=20; ++i) {
            if(i%2) { v.push_back(ValueType(a,1/(1.+i)));  ++b; ++b; a=b; ++b; } else { v.push_back(ValueType(b,1/(1.+i)));}
        }
        std::sort(v.begin(),v.end());
    }
    {
        std::vector<Int> v;
        const Int element_size=3;
        const Int index_size=1;
        const Int data_size=2;
        for(Nat i=0; i!=20; ++i) {
            Int word= ( i%2 ? (3*i-2)/2 : (3*i+2)/2 );
            double value=1/(2.+i);
            v.resize(v.size()+3);
            v[v.size()-3]=word;
            reinterpret_cast<double&>(v[v.size()-2])=value;
        }
        for(Nat i=0; i!=v.size()/3; ++i) {
            Int word; double value;
            word=v[3*i];
            value=reinterpret_cast<double&>(v[3*i+1]);
        }

        typedef Expansion<ApproximateDouble>::Iterator Iterator;
        Iterator iter1(3,&*v.begin());
        Iterator iter2(3,&*v.end());
        std::sort(iter1,iter2);

        for(Nat i=0; i!=v.size()/3; ++i) {
            Int word; double value;
            word=v[3*i];
            value=reinterpret_cast<double&>(v[3*i+1]);
        }

    }
*/

    // Test to see if the cleanup/sort operations work.
    // Since these are used in the constructors, we can't use the main constructors to test this
    MultiIndex a(3);
    MultiIndex b(3); ++b;
    MultivariatePolynomial<RoundedFloatDP> p(3,dp);
    RoundedFloatDP c(1,dp);
    for(Nat i=0; i!=2; ++i) {
        if(i%2) { p.expansion().append(a,c); ++b; ++b; a=b; ++b; } else { p.expansion().append(b,c);}
        c=hlf(c);
    }
    ARIADNE_TEST_PRINT(p)
    ARIADNE_TEST_EXECUTE(p.cleanup())
    ARIADNE_TEST_PRINT(p)

}

Void TestPolynomial::test_constructors()
{
    // Empty polynomial
    ARIADNE_TEST_CONSTRUCT(UnivariatePolynomial<RoundedFloatDP>,q1,(SizeOne(),dp))
    // Dense polynomial
    ARIADNE_TEST_CONSTRUCT(UnivariatePolynomial<RoundedFloatDP>,q2,({ {0,0.0_x}, {1,0.0_x},{2,5.0_x},{3,2.0_x} },dp))
    ARIADNE_TEST_EQUAL(q2[1],0.0_x)
    ARIADNE_TEST_EQUAL(q2[2],5.0_x)
    ARIADNE_TEST_EQUAL(q2[3],2.0_x)

    // Empty polynomial
    ARIADNE_TEST_CONSTRUCT(MultivariatePolynomial<RoundedFloatDP>,p1,(3,dp))
    // Dense polynomial
    ARIADNE_TEST_CONSTRUCT(MultivariatePolynomial<RoundedFloatDP>,p2,({ {{0,0,0},0.0_x}, {{1,0,0},0.0_x},{{0,1,0},0.0_x},{{0,0,1},0.0_x}, {{2,0,0},5.0_x},{{1,1,0},2.0_x},{{1,0,1},0.0_x},{{0,2,0},0.0_x},{{0,1,2},3.0_x},{{0,0,2},0.0_x} },dp))
    ARIADNE_TEST_EQUAL(p2[MultiIndex({2,0,0})],5.0_x)
    // Sparse polynomial with unordered indiced
    ARIADNE_TEST_CONSTRUCT(MultivariatePolynomial<RoundedFloatDP>,p3,({ {{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x} },dp))
    ARIADNE_TEST_EQUAL(p3[MultiIndex({1,2})],5.0_x)
    ARIADNE_TEST_EQUAL(p3[MultiIndex({0,0})],2.0_x)

    // Unordered indices
    ARIADNE_TEST_EQUAL(MultivariatePolynomial<RoundedFloatDP>({ {{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x} },dp), MultivariatePolynomial<RoundedFloatDP>({ {{0,0},2.0_x}, {{1,0},3.0_x}, {{0,1},11.0_x}, {{3,0},7.0_x}, {{1,2},5.0_x} },dp))
    // Repeated indices
    ARIADNE_TEST_EQUAL(MultivariatePolynomial<RoundedFloatDP>({ {{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{1,0},7.0_x}, {{1,2},11.0_x} },dp), MultivariatePolynomial<RoundedFloatDP>({ {{0,0},2.0_x}, {{1,0},10.0_x}, {{1,2},16.0_x} },dp))

}

Void TestPolynomial::test_indexing()
{
    MultivariatePolynomial<RoundedFloatDP> p({ {{0,0,0},2.0_x},  {{1,0,0},3.0_x}, {{1,0,1},5.0_x}, {{2,1,0},7.0_x} },dp);
    const MultivariatePolynomial<RoundedFloatDP>& pc=p;
    ARIADNE_TEST_EQUAL(p[MultiIndex({1,0,0})],3.0_x)

    p[MultiIndex({1,0,0})]-=0.5_x;
    ARIADNE_TEST_EQUAL(p[MultiIndex({1,0,0})],2.5_x)

    p[MultiIndex({1,1,0})]=11.0_x;
    ARIADNE_TEST_EQUAL(p[MultiIndex({1,1,0})],11.0_x)

    MultivariatePolynomial<RoundedFloatDP> q(3,dp);
    q[MultiIndex({0,0,0})]=2.0_x;
    q[MultiIndex({0,1,0})]=3.0_x;
    ARIADNE_TEST_EQUALS(q.number_of_terms(),2)
    ARIADNE_TEST_EQUALS(q[MultiIndex({0,0,0})],2.0_x)
    ARIADNE_TEST_EQUALS(q[MultiIndex({0,1,0})],3.0_x)
    q[MultiIndex({1,0,0})]=5.0_x;
    ARIADNE_TEST_EQUALS(q[MultiIndex({1,0,0})],5.0_x)
    ARIADNE_TEST_EQUALS(q[MultiIndex({0,1,0})],3.0_x)
    q[MultiIndex({0,0,1})]=7.0_x;
    ARIADNE_TEST_EQUALS(q[MultiIndex({0,0,1})],7.0_x)

    // Test insert at beginning
    p.clear();
    ARIADNE_TEST_PRINT(p.expansion())
    p[MultiIndex({0,1,0})]=2.0_x;
    p[MultiIndex({0,0,1})]=3.0_x;
    p[MultiIndex({2,1,0})]=5.0_x;
    ARIADNE_TEST_PRINT(p.expansion());
    ARIADNE_TEST_EQUALS(pc[MultiIndex({0,0,0})],0.0_x)
    ARIADNE_TEST_EQUALS(p.number_of_terms(),3)
    ARIADNE_TEST_PRINT(p.expansion())
    ARIADNE_TEST_EXECUTE(p[MultiIndex({0,0,0})]=7)
    ARIADNE_TEST_PRINT(p.expansion())
    ARIADNE_TEST_EQUALS(p.number_of_terms(),4)
    p.expansion().graded_sort();
    ARIADNE_TEST_PRINT(p.expansion())

    MultivariatePolynomial<RoundedFloatDP>::ConstIterator iter=p.begin();
    ARIADNE_TEST_EQUALS(iter->index(),MultiIndex({0,0,0}))
    ARIADNE_TEST_EQUALS(iter->coefficient(),7.0_x)
    ++iter;
    ARIADNE_TEST_EQUALS(iter->index(),MultiIndex({0,1,0}))
    ARIADNE_TEST_EQUALS(iter->coefficient(),2.0_x)
    ++iter;
    ARIADNE_TEST_EQUALS(iter->index(),MultiIndex({0,0,1}))
    ARIADNE_TEST_EQUALS(iter->coefficient(),3.0_x)
    ++iter;
    ARIADNE_TEST_EQUALS(iter->index(),MultiIndex({2,1,0}))
    ARIADNE_TEST_EQUALS(iter->coefficient(),5.0_x)
}

Void TestPolynomial::test_arithmetic()
{
    ARIADNE_TEST_EQUAL(P(3,dp)+P(3,dp),P(3,dp))
    ARIADNE_TEST_EQUAL(P(3,dp)+P({ {{2,1,0},2.0_x} },dp),P({ {{2,1,0},2.0_x} },dp))
    ARIADNE_TEST_EQUAL(P(3,dp)+P({ {{2,1,0},2.0_x}, {{0,1,0},3.0_x}, {{1,1,0},5.0_x} },dp), P({ {{0,1,0},3.0_x}, {{1,1,0},5.0_x}, {{2,1,0},2.0_x} },dp))

    MultivariatePolynomial<RoundedFloatDP> x0(3,dp); x0[MultiIndex({1,0,0})]=1.0_x;
    MultivariatePolynomial<RoundedFloatDP> x1(3,dp); x1[MultiIndex({0,1,0})]=1.0_x;
    MultivariatePolynomial<RoundedFloatDP> x2=MultivariatePolynomial<RoundedFloatDP>::coordinate(3,2,dp);
    UnivariatePolynomial<RoundedFloatDP> y=UnivariatePolynomial<RoundedFloatDP>::coordinate(SizeOne(),IndexZero(),dp);
    y=UnivariatePolynomial<RoundedFloatDP>::coordinate(dp);

    RoundedFloatDP w(3,dp);
    Vector<RoundedFloatDP> v({3,5,2},dp);

    ARIADNE_TEST_EQUALS(evaluate(2*x0*x0-1,v),2*v[0]*v[0]-1)
    ARIADNE_TEST_EQUALS(evaluate(2*x1*x1-1,v),2*v[1]*v[1]-1)
    /* Failing with UnivariatePolynomial
    ARIADNE_TEST_EQUALS(evaluate(2*y*y-1,w),2*w*w-1);
    ARIADNE_TEST_EQUALS(evaluate(8*y*y*(y*y-1)+1,w),8*w*w*(w*w-1)+1);
    ARIADNE_TEST_EQUALS(compose(2*y*y-1,x0),2*x0*x0-1);
     */
}

Void TestPolynomial::test_variables()
{
    Vector< MultivariatePolynomial<RoundedFloatDP> > x=MultivariatePolynomial<RoundedFloatDP>::variables(3,dp);
    Array< Vector<RoundedFloatDP> > e=Vector<RoundedFloatDP>::basis(2,dp);

    MultivariatePolynomial<RoundedFloatDP> p1=x[1]*3.0_x;
    MultivariatePolynomial<RoundedFloatDP> p2=p1+x[0]; p2=x[1]*3; p2=x[0]+0;
    MultivariatePolynomial<RoundedFloatDP> p3=x[0]*p2; p3=x[0]*(x[1]*3.0_x+x[0]);
    MultivariatePolynomial<RoundedFloatDP> p4=x[1]*x[2];
    MultivariatePolynomial<RoundedFloatDP> p5=p3+p4;
    MultivariatePolynomial<RoundedFloatDP> p=x[0]*(x[1]*3.0_x+x[0])+x[1]*x[2];

    ARIADNE_TEST_EQUAL(x[0], MultivariatePolynomial<RoundedFloatDP>({ {{1,0,0},1.0_x} },dp))
    ARIADNE_TEST_EQUAL(x[1], MultivariatePolynomial<RoundedFloatDP>({ {{0,1,0},1.0_x} },dp))
    ARIADNE_TEST_EQUAL(x[2], MultivariatePolynomial<RoundedFloatDP>({ {{0,0,1},1.0_x} },dp))
    ARIADNE_TEST_EQUAL(x[0]+x[1], MultivariatePolynomial<RoundedFloatDP>({ {{1,0,0},1.0_x}, {{0,1,0},1.0_x} },dp))
    ARIADNE_TEST_EQUAL(x[0]*x[1], MultivariatePolynomial<RoundedFloatDP>({ {{1,1,0},1.0_x} },dp))
    ARIADNE_TEST_EVALUATE(x[0]*(x[1]*3.0_x+x[0])+x[1]*x[2])
    ARIADNE_TEST_EQUAL((x[0]*(x[1]*3.0_x+x[0])+x[1]*x[2]), MultivariatePolynomial<RoundedFloatDP>({ {{1,1,0},3.0_x}, {{2,0,0},1.0_x}, {{0,1,1},1.0_x} },dp))
    ARIADNE_TEST_EQUAL((e[1]*(x[0]*(x[1]*3.0_x+x[0])+x[1]*x[2]))[1], MultivariatePolynomial<RoundedFloatDP>({ {{1,1,0},3.0_x}, {{2,0,0},1.0_x}, {{0,1,1},1.0_x} },dp))
    ARIADNE_TEST_PRINT((e[1]*(x[0]*(x[1]*3.0_x+x[0])+x[1]*x[2]))[0])
    ARIADNE_TEST_EQUAL((e[1]*(x[0]*(x[1]*3.0_x+x[0])+x[1]*x[2]))[0], MultivariatePolynomial<RoundedFloatDP>(3,dp))
    ARIADNE_TEST_EQUAL((e[1]*(x[0]*(x[1]*3.0_x+x[0])+x[1]*x[2]))[0], MultivariatePolynomial<RoundedFloatDP>({ {{3,0,0},0.0_x} },dp))
}

Void TestPolynomial::test_find()
{
    MultivariatePolynomial<RoundedFloatDP> p({ {{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x} },dp);
    MultiIndex a(2);
    a[0]=1; a[1]=2;
    ARIADNE_TEST_PRINT(p)
    ARIADNE_TEST_PRINT(p.find(a)-p.begin())
    ARIADNE_TEST_COMPARE(p.find(a),!=,p.end())
    ARIADNE_TEST_EQUAL(p.find(a)->index(),a)
    ARIADNE_TEST_EQUAL(p.find(a)->coefficient(),5.0_x)
    a[1]=1;
    ARIADNE_TEST_EQUAL(p.find(a),p.end())
}

Void TestPolynomial::test_flow()
{
    static_assert(not DefaultConstructible<MultivariatePolynomial<FloatDPBounds>>);
    using PolynomialType = MultivariatePolynomial<FloatDPBounds>;
    PolynomialType f0({ {{0,0,0},1.0_x}, {{0,1,0},1.0_x} },dp);
    PolynomialType f1({ {{2,0,0},-1.0_x}},dp);
    Vector<PolynomialType> f={f0,f1};
    ARIADNE_TEST_PRINT(f[0].argument_size())
    ARIADNE_TEST_PRINT(characteristics(f[0]))
    ARIADNE_TEST_PRINT(f.array().element_characteristics())
    ARIADNE_TEST_PRINT(f)

    auto phi_picard = flow_polynomial_picard_iteration(f,4);
    ARIADNE_TEST_PRINT(phi_picard)
    auto phi_lie = flow_polynomial_lie_derivative(f,4);
    ARIADNE_TEST_PRINT(phi_lie)
    ARIADNE_TEST_ASSERT(possibly(phi_picard == phi_lie))
}

Int main() {
    TestPolynomial().test();
    return ARIADNE_TEST_FAILURES;
}
