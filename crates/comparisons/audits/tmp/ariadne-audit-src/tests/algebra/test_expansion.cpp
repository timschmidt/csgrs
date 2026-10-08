/***************************************************************************
 *            test_expansion.cpp
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
#include <vector>
#include "config.hpp"
#include "numeric/numeric.hpp"
#include "algebra/expansion.hpp"
#include "algebra/expansion.inl.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;

template<class K, class V> struct MapValue {
    typedef K key_type;
    K first; V second;
    MapValue(const K& k, const V& v) : first(k), second(v) { }
    const K& index() const { return first; }
    const V& coefficient() const { return second; }
};


template<class F> class TestExpansion
{
    typedef PrecisionType<F> PR;
    typedef MultiIndex MI;
    typedef Expansion<MI,F> ExpansionType;

    typedef typename Expansion<MI,F>::ValueType ExpansionValueType;
    typedef typename Expansion<MI,F>::Reference ExpansionReference;
    typedef typename Expansion<MI,F>::ConstReference ExpansionConstReference;
    typedef typename Expansion<MI,F>::Pointer ExpansionExpansionPointer;
    typedef typename Expansion<MI,F>::ConstPointer ExpansionExpansionConstPointer;
    typedef typename Expansion<MI,F>::Iterator ExpansionIterator;
    typedef typename Expansion<MI,F>::ConstIterator ExpansionConstIterator;

    typedef typename Expansion<MI,F>::IndexReference ExpansionIndexReference;
    typedef typename Expansion<MI,F>::IndexConstReference ExpansionIndexConstReference;
    typedef typename Expansion<MI,F>::CoefficientReference ExpansionCoefficientReference;
    typedef typename Expansion<MI,F>::CoefficientConstReference ExpansionCoefficientConstReference;
  public:
    PR prec; F zero;
    GradedLess graded_less;
    LexicographicLess lexicographic_less;
    ReverseLexicographicLess reverse_lexicographic_less;
  public:
    TestExpansion(F const& z);
    Void test();
  private:
    Void test_working();
    Void test_concept();
    Void test_iterator_concept();
    Void test_data_access();
    Void test_equality();
    Void test_sort();
    Void test_cleanup();
    Void test_constructors();
    //Void test_indexing();
    Void test_find();
    Void test_embed();
};


template<class F> TestExpansion<F>::TestExpansion(F const& z)
    : prec(z.precision()), zero(z)
{
}

template<class F> Void TestExpansion<F>::test()
{
    ARIADNE_TEST_CALL(test_working());
    ARIADNE_TEST_CALL(test_data_access());
    ARIADNE_TEST_CALL(test_equality());
    ARIADNE_TEST_CALL(test_sort());
    ARIADNE_TEST_CALL(test_cleanup());
    ARIADNE_TEST_CALL(test_constructors());
    //ARIADNE_TEST_CALL(test_indexing());
    ARIADNE_TEST_CALL(test_find());
    ARIADNE_TEST_CALL(test_embed());
}


template<class F> Void TestExpansion<F>::test_working()
{
    ARIADNE_TEST_PRINT(zero);
    ARIADNE_TEST_CONSTRUCT(MultiIndexList,as,(3u));
    ARIADNE_TEST_CONSTRUCT(ExpansionType,e,(3u,prec));
    ARIADNE_TEST_EQUALS(e.size(),0u);
    ARIADNE_TEST_EQUALS(e.argument_size(),3u);
    // Append values
    ARIADNE_TEST_EXECUTE(e.append({0,0,0},2.0_x));
    ARIADNE_TEST_EQUALS(e.begin()->index().number_of_variables(),3);
    ARIADNE_TEST_EXECUTE(e.append({1,0,0},3.0_x));
    ARIADNE_TEST_EXECUTE(e.append({0,1,0},5.0_x));
    ARIADNE_TEST_EXECUTE(e.append({1,0,1},7.0_x));
    ARIADNE_TEST_PRINT(e);

    ARIADNE_TEST_EQUAL(e.find({0,0,0}),e.begin());
    //assert(false);

    // Regression test to ensure that indices and coefficients always have same capacity
    ARIADNE_TEST_ASSERT(e._indices.capacity()==e._coefficients.capacity());
    ExpansionType ce=e;
    ARIADNE_TEST_ASSERT(ce._indices.capacity()==ce._coefficients.capacity());

}


template<class F> Void TestExpansion<F>::test_concept()
{
    F x(5,prec);
    SizeType as(3);

    ExpansionType e(as,prec);
    const ExpansionType ce(as,prec);

    e=ExpansionType(as,zero);
    e=ExpansionType(ce);

    //e=ExpansionType(3,1, {0.0_x, 0.0_x,0.0_x,0.0_x}, prec);
    //e=ExpansionType(3,1, {1, 2,3,5.0_x}, prec);
    e=ExpansionType({ {{0,0},1}, {{1,0,0},2}, {{0,1,0},3}, {{0,0,1},5.0_x} }, prec);
    e=ExpansionType({ {{0,0},1}, {{1,0,0},2}, {{0,1,0},3}, {{0,0,1},5.0_x} }, prec);

    MultiIndex a(as);
    e.reserve(2u);
    e.set(a,x);
    e.prepend(a,x);
    e.append(a,x);
    e.append_sum(a,a,x);
    e.clear();

    e.index_sort(GradedLess());
    e.index_sort(LexicographicLess());
    e.index_sort(GradedIndexLess());
    e.sort(ReverseLexicographicIndexLess());

    x=ce[a];

    ce.number_of_terms();
    ce.argument_size();

    e.erase(e.begin());

    ce.check();
}

template<class F> Void TestExpansion<F>::test_iterator_concept()
{
    MultiIndex a(3);
    ExpansionType e(3,zero);
    const ExpansionType cp(3,zero);

    ExpansionIterator iter=e.begin(); iter=e.end(); iter=e.find(a);
    ExpansionConstIterator citer=e.begin(); citer=e.end(); citer=e.find(a);
    citer=e.begin(); citer=cp.end(); citer=cp.find(a);

    ExpansionValueType val=*iter;
    ExpansionReference ref=*iter;
    ExpansionConstReference ncref=*iter;

    ExpansionValueType cval=*citer;
    ExpansionConstReference cref=*citer;

    Bool res; if (res) return; // To avoid compiler warning

    ++iter; --iter;
    ++citer; --citer;

    res=(iter==iter); res=(iter!=iter); res=(citer==citer); res=(citer!=citer);
    res=(citer==iter); res=(citer!=iter); res=(iter==citer); res=(iter!=citer);

    ref=cref; ref=ncref;
}

// Test dereferencing of iterators
template<class F> Void TestExpansion<F>::test_data_access()
{
    ExpansionType e(3,prec);
    ExpansionType const& ce=e;
    MultiIndex a(3);
    ARIADNE_TEST_EXECUTE(e[a]=11);
    ARIADNE_TEST_EQUALS(e[a],11);
    ARIADNE_TEST_EQUALS(ce[a],11);
    ARIADNE_TEST_EQUALS(e.at(a),11);
    e.clear();

    ARIADNE_TEST_EXECUTE(e.at(a)=17);
    ARIADNE_TEST_EQUALS(e[a],17);
    e.clear();

    SortedExpansion<MultiIndex,F,GradedIndexLess> se(3,prec);
    ARIADNE_TEST_EXECUTE(se[a]=11);
    ARIADNE_TEST_EQUALS(se[a],11);
    se.clear();


    // Append values
    ARIADNE_TEST_EXECUTE(e.append({0,0,0},F(2.0_x,prec)));
    ARIADNE_TEST_EXECUTE(e.append({1,0,0},F(3.0_x,prec)));
    ARIADNE_TEST_EXECUTE(e.append({0,1,0},F(5.0_x,prec)));
    ARIADNE_TEST_EXECUTE(e.append({1,0,1},F(7.0_x,prec)));

    // Test ExpansionIterator difference
    ARIADNE_TEST_PRINT(e.begin());
    ARIADNE_TEST_PRINT(e.end());
    ARIADNE_TEST_EQUAL(e.begin()+static_cast<PointerDifferenceType>(e.number_of_terms()),e.end());
    ARIADNE_TEST_EQUAL(e.end()-static_cast<PointerDifferenceType>(e.number_of_terms()),e.begin());
    ARIADNE_TEST_EQUAL(e.end()-e.begin(),static_cast<PointerDifferenceType>(e.number_of_terms()));

    // Test derefencing of iterators
    ARIADNE_TEST_PRINT(e.begin());
    ARIADNE_TEST_PRINT(*e.begin());
    ARIADNE_TEST_EQUAL(e.begin()->index(),MultiIndex({0,0,0}));
    ARIADNE_TEST_EQUAL(e.begin()->coefficient(),2.0_x);

    ARIADNE_TEST_PRINT(ce.begin());
    ARIADNE_TEST_PRINT(*ce.begin());
    ARIADNE_TEST_EQUAL(ce.begin()->index(),MultiIndex({0,0,0}));
    ARIADNE_TEST_EQUAL(ce.begin()->coefficient(),2.0_x);


    // The behaviour of iterators is rather odd and not what might be expected
    // A MultiIndex reference assigned to by iter->index() changes its value
    // when the ExpansionIterator is incremented, but a F reference does not.
    // This behaviour should be changed in future versions if technologically
    // feasible.
    {
        ExpansionIterator iter=e.begin();
        ExpansionIndexReference aref=iter->index();
        ExpansionCoefficientConstReference xref=iter->coefficient();
        MultiIndex a1=iter->index();
        F x1=iter->coefficient();
        ++iter;
        MultiIndex a2=iter->index();
        F x2=iter->coefficient();
        ARIADNE_TEST_ASSERT(a1==aref);
        ARIADNE_TEST_ASSERT(x1==xref);
        ARIADNE_TEST_ASSERT(a2!=aref);
        ARIADNE_TEST_ASSERT(x2!=xref);
    }

    {
        ExpansionConstIterator citer=e.begin();
        ExpansionIndexConstReference caref=citer->index();
        ExpansionCoefficientConstReference cxref=citer->coefficient();
        MultiIndex ca1=citer->index();
        F cx1=citer->coefficient();
        ++citer;
        MultiIndex ca2=citer->index();
        F cx2=citer->coefficient();
        ARIADNE_TEST_ASSERT(ca1==caref);
        ARIADNE_TEST_ASSERT(cx1==cxref);
        ARIADNE_TEST_ASSERT(ca2!=caref);
        ARIADNE_TEST_ASSERT(cx2!=cxref);
    }

    {
        ExpansionIterator iter=e.begin()+2;
        ExpansionPointer mptr=iter.operator->();
        ExpansionReference mref=iter.operator*();
        ExpansionValueType m1val=*iter;
        ++iter;
        ExpansionValueType m2val=*iter;
        ARIADNE_TEST_ASSERT(m1val==mref);
        ARIADNE_TEST_ASSERT(m1val==*mptr);
        ARIADNE_TEST_ASSERT(m2val!=mref);
        ARIADNE_TEST_ASSERT(m2val!=*mptr);
    }

    {
        ExpansionConstIterator iter=e.end()-1;
        ExpansionConstPointer mptr=iter.operator->();
        ExpansionConstReference mref=iter.operator*();
        ExpansionValueType m1val=*iter;
        ++iter;
        ARIADNE_TEST_ASSERT(m1val==mref);
        ARIADNE_TEST_ASSERT(m1val==*mptr);
    }

    // Test finding of values of iterators
    ExpansionIterator iter=e.begin();
    ARIADNE_TEST_PRINT(e);
    ARIADNE_TEST_EXECUTE(iter=e.find(MultiIndex({1,0,0})));
    ARIADNE_TEST_EQUAL(iter->index(),MultiIndex({1,0,0}));
    ARIADNE_TEST_EQUAL(iter->coefficient(),3.0_x);



    // Test hand-coded swap of values
    ARIADNE_TEST_CONSTRUCT(ExpansionIterator,iter1,(e.begin()+1));
    ARIADNE_TEST_CONSTRUCT(ExpansionIterator,iter2,(e.begin()+3));

    // Perform swap
    ARIADNE_TEST_CONSTRUCT(ExpansionValueType,tmp,(*iter2));
    ARIADNE_TEST_ASSERT(tmp.index()==MultiIndex({1,0,1}));
    ARIADNE_TEST_ASSERT(tmp.coefficient()==7.0_x);
    ARIADNE_TEST_EXECUTE(*iter2=*iter1);
    ARIADNE_TEST_ASSERT(iter2->index()==MultiIndex({1,0,0}));
    ARIADNE_TEST_ASSERT(iter2->coefficient()==3.0_x);
    ARIADNE_TEST_EXECUTE(*iter1=tmp);
    ARIADNE_TEST_ASSERT(iter1->index()==MultiIndex({1,0,1}));
    ARIADNE_TEST_ASSERT(iter1->coefficient()==7.0_x);


}

template<class F> Void TestExpansion<F>::test_equality()
{
    MultiIndex a(2);
    MultiIndex b(2); ++b;
    Expansion<MI,F> e1(2,prec),e2(2,prec);
    e1.append(a,1.0_x); e1.append(b,2.0_x);
    e2.append(a,1.0_x); e2.append(b,3.0_x);
    ARIADNE_TEST_BINARY_PREDICATE(!same, e1,e2);
    e2.clear(); e2.append(a,1.0_x); e2.append(b,2.0_x);
    ARIADNE_TEST_BINARY_PREDICATE(same, e1,e2);
    e1.clear(); e1.append(b,2.0_x);
    e2.clear(); e2.append(a,0.0_x); e2.append(b,2.0_x);
    if(!same(e1,e2)) { ARIADNE_TEST_NOTIFY("Expansion<MI,F> objects differing by explicit zeros are considered not same."); }
    e1.clear(); e1.append(a,-0.0_x);
    e1.clear(); e1.append(a,+0.0_x);
    if(!same(e1,e2)) { ARIADNE_TEST_NOTIFY("Expansion<MI,F> objects differing by +0 versus -0 coefficients are considered not same."); }
    e1.clear(); e1.append(a,1.0_x); e1.append(b,2.0_x);
    e2.clear(); e2.append(b,2.0_x); e2.append(a,1.0_x);
    if(!same(e1,e2)) { ARIADNE_TEST_NOTIFY("Expansion<MI,F> objects differing by order of set operators are considered not same."); }
}

template<class F> Void TestExpansion<F>::test_sort()
{
    MultiIndexList as1({{0,0},{1,0},{2,0}});
    ARIADNE_TEST_PRINT(as1);
    MultiIndexList as2({{2,0},{0,0},{1,0}});
    ARIADNE_TEST_PRINT(as1);

    GradedIndexLess graded_index_less;
    Expansion<MI,F> e2s({{{0,0},5.0_x},{{1,0},11.0_x}},prec);
    ARIADNE_TEST_PRINT(e2s)
    ARIADNE_TEST_ASSERT(e2s.is_sorted(graded_index_less));
    Expansion<MI,F> e2u({{{1,0},2.0_x},{{0,0},5.0_x}},prec);
    ARIADNE_TEST_PRINT(e2u)
    ARIADNE_TEST_ASSERT(not e2u.is_sorted(graded_index_less));

//    ARIADNE_TEST_CONSTRUCT( Expansion<MI,F>, e1, ({{{0,1},1.0_x},{{1,1},1.0_x},{{2,1},1.0_x},{{0,1},11.0_x},{{0,1},11.0_x},{{1,1},11.0_x}},prec) );
    Expansion<MI,F> e1({{{0,1},1.0_x},{{1,1},1.0_x},{{2,1},1.0_x},{{0,1},11.0_x},{{0,1},11.0_x},{{1,1},11.0_x}},prec);
    ARIADNE_TEST_PRINT(e1);
    ARIADNE_TEST_ASSERT(not e1.is_sorted(graded_index_less));
    ARIADNE_TEST_EXECUTE(e1.sort(graded_index_less));
    ARIADNE_TEST_ASSERT(e1.is_sorted(graded_index_less));
}

template<class F> Void TestExpansion<F>::test_cleanup()
{
    // Test to see if the cleanup/sort operations work.
    // Since these are used in the constructors, we can't use the main constructors to test this
    MultiIndex a(3);
    MultiIndex b(3); ++b;
    ARIADNE_TEST_PRINT(a);
    ARIADNE_TEST_PRINT(b);

    Expansion<MI,F> e(3,prec);
    F c(1,prec);
    ARIADNE_TEST_PRINT(e);
    for(Nat i=0; i!=2; ++i) {
        c=hlf(c);
        if(i%2) { e.append(a,c); ++b; ++b; a=b; ++b; } else { e.append(b,c);}
        ARIADNE_TEST_PRINT(e);
    }

    ARIADNE_TEST_PRINT(e);
    ARIADNE_TEST_EXECUTE(e.index_sort(graded_less));
    ARIADNE_TEST_PRINT(e);
    ARIADNE_TEST_EXECUTE(e.combine_terms());
    ARIADNE_TEST_PRINT(e);
    ARIADNE_TEST_EXECUTE(e.remove_zeros());
    ARIADNE_TEST_PRINT(e);

}

template<class F> Void TestExpansion<F>::test_constructors()
{
    // Empty initialiser list causes failure
    ARIADNE_TEST_FAIL(ExpansionType e0({}));

    // Empty expansion
    ARIADNE_TEST_CONSTRUCT(ExpansionType,e1,(3,prec));
    // Expansion with all entries; useful for checking ordering of indices
    //ARIADNE_TEST_CONSTRUCT(Expansion<MI,F>,e2,(3,4, {1., 2.,3.,4., 5.,6.,7.,8.,9.,10.,
    //    11.,12.,13.,14.,15.,16.,17.,18.,19.,20., 21.,22.,23.,24.,25.,26.,27.,28.,29.,30.,31.,32.,33.,34.,35}));

    // Dense expansion
    ARIADNE_TEST_CONSTRUCT(ExpansionType,e3,({{{2,0,0},5.0_x},{{1,1,0},2.0_x},{{1,0,1},0.0_x},{{0,2,0},0.0_x},{{0,1,1},3.0_x},{{0,2,0},0.0_x}}, prec));
    //ARIADNE_TEST_CONSTRUCT(Expansion<MI,F>,e3,(3,2, {0., 0.,0.,0., 5.,2.,0.,0.,3.,0.}));
    ARIADNE_TEST_PRINT(e3);
    ARIADNE_TEST_COMPARE(e3.find(MultiIndex({2,0,0})),!=,e3.end());
    ARIADNE_TEST_EQUAL(e3[MultiIndex({2,0,0})],5.0_x);

    // Sparse expansion with unordered indiced
    Expansion<MI,F> pp3({{{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x}}, prec);
    ARIADNE_TEST_CONSTRUCT(ExpansionType,p3,({{{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x}}, prec));
    ARIADNE_TEST_EQUAL(p3[MultiIndex({1,2})],5.0_x);
    ARIADNE_TEST_EQUAL(p3[MultiIndex({0,0})],2.0_x);

    // Unordered indices
    ARIADNE_TEST_BINARY_PREDICATE(!same,ExpansionType({{{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x}}, prec),ExpansionType({{{0,0},2.0_x}, {{1,0},3.0_x}, {{0,1},11.0_x}, {{3,0},7.0_x}, {{1,2},5.0_x}}, prec));
    // Repeated indices; do not sum in expansion class
    ARIADNE_TEST_BINARY_PREDICATE(!same,ExpansionType({{{1,0},2.0_x}, {{1,0},3.0_x}, {{0,2},7.0_x}}, prec),ExpansionType({{{1,0},5.0_x}, {{0,2},7.0_x}}, prec));

    // Regression tests for expansions with only preces
    ARIADNE_TEST_CONSTRUCT(ExpansionType,pr2,({{{},0.0_x}},prec));
    ARIADNE_TEST_CONSTRUCT(ExpansionType,pr1,({{{3,0,0},0.0_x}},prec));

    // Regression tests for higher-order expansions
    ARIADNE_TEST_CONSTRUCT(ExpansionType,ho1,({{{0,1,0,0,0},2.0_x}, {{0,1,0,0,1},3.0_x}, {{2,0,1,0,0},5.0_x}, {{0,0,0,0,0},7.0_x}},prec));
}


template<class F> Void TestExpansion<F>::test_find()
{
    ExpansionType e({ {{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x} }, prec);
    MultiIndex a(2);
    a[0]=1; a[1]=2;
    ARIADNE_TEST_PRINT(e);
    ARIADNE_TEST_PRINT(e.find(a)-e.begin());
    ARIADNE_TEST_COMPARE(e.find(a),!=,e.end());
    ARIADNE_TEST_EQUAL(e.find(a)->index(),a);
    ARIADNE_TEST_EQUAL(e.find(a)->coefficient(),5.0_x);
    a[1]=1;
    ARIADNE_TEST_EQUAL(e.find(a),e.end());

}

template<class F> Void TestExpansion<F>::test_embed()
{
    ARIADNE_TEST_CONSTRUCT(ExpansionType,e,({ {{1,2},5.0_x}, {{0,0},2.0_x}, {{1,0},3.0_x}, {{3,0},7.0_x}, {{0,1},11.0_x} },prec));
    ARIADNE_TEST_SAME_AS(embed(0,e,2),ExpansionType({ {{1,2,0,0},5.0_x}, {{0,0,0,0},2.0_x}, {{1,0,0,0},3.0_x}, {{3,0,0,0},7.0_x}, {{0,1,0,0},11.0_x} },prec));
    ARIADNE_TEST_SAME_AS(embed(1,e,0),ExpansionType({ {{0,1,2},5.0_x}, {{0,0,0},2.0_x}, {{0,1,0},3.0_x}, {{0,3,0},7.0_x}, {{0,0,1},11.0_x} },prec));
    ARIADNE_TEST_SAME_AS(embed(1,e,2),ExpansionType({ {{0,1,2,0,0},5.0_x}, {{0,0,0,0,0},2.0_x}, {{0,1,0,0,0},3.0_x}, {{0,3,0,0,0},7.0_x}, {{0,0,1,0,0},11.0_x} },prec));

}

Int main() {
    RoundedFloatDP zero_dp{0,double_precision};
    RoundedFloatMP zero_mp{0,precision(128_bits)};
    TestExpansion<RoundedFloatDP>(zero_dp).test();
    TestExpansion<Rounded<FloatMP>>(zero_mp).test();
    return ARIADNE_TEST_FAILURES;
}
