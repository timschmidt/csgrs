/***************************************************************************
 *            test_box.cpp
 *
 *  Copyright  2005-20  Alberto Casagrande, Pieter Collins
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

#include "numeric/module.hpp"
#include "geometry/box.hpp"

#include "../test.hpp"

using namespace Ariadne;
using std::cout; using std::endl;

template<class I> Box<I> make_box(InitializerList<Interval<RawFloatDP>> lst) {
    return Box<I>(Vector<I>(lst));
}

template<class B> inline auto possibly_not(B b) -> decltype(possibly(not b)) { return possibly(not b); }
template<class B> inline auto definitely_not(B b) -> decltype(definitely(not b)) { return definitely(not b); }

template<class BX> class TestBoxType {
    typedef typename BX::IntervalType I;
    typedef typename I::LowerBoundType L;
    typedef typename I::UpperBoundType U;
    typedef typename I::MidpointType X;
    typedef I IntervalType;
    typedef Point<X> PointType;
    typedef BX BoxType;
  public:
    void test() {
        ARIADNE_TEST_CALL(test_geometric_predicates());
    }

    void test_geometric_predicates() {

    PointType s1({1.0_x,1.0_x},dp);
    PointType s2({1.5_x,1.5_x},dp);
    PointType s3({1.375_x,1.375_x},dp);
    PointType s4({2.0_x,2.0_x},dp);
    PointType s5({0.75_x,0.125_x},dp);

    ARIADNE_TEST_CONSTRUCT( BoxType, r0, (2) );
    ARIADNE_TEST_CONSTRUCT( BoxType, r1, ({{0.0_x,1.0_x},{0.0_x,1.0_x}}) );
    ARIADNE_TEST_CONSTRUCT( BoxType, r2, ({{-0.5_x,1.5_x},{-0.375_x,0.5_x}}) );
    ARIADNE_TEST_CONSTRUCT( BoxType, r3, ({{-0.25_x,0.625_x},{0.375_x,1.5_x}}) );
    ARIADNE_TEST_CONSTRUCT( BoxType, r4, ({{0.5625_x,1.125_x},{0.4375_x,1.375_x}}) );
    ARIADNE_TEST_CONSTRUCT( BoxType, r5, ({{0.0_x,1.1875_x},{0.4375_x,1.0_x}}) );
    ARIADNE_TEST_CONSTRUCT( BoxType, r6, ({{0.0_x,1.0_x},{0.0_x,0.5_x}}) );

    ARIADNE_TEST_EQUALS(r2.volume(),1.75_x);
    ARIADNE_TEST_EQUALS(r2.semiperimeter(),2.875_x);

    ARIADNE_TEST_EQUALS(r2.widths()[0],2.0_x);
    ARIADNE_TEST_EQUALS(r2.widths()[1],0.875_x);
    ARIADNE_TEST_EQUALS(r2.radii()[0],1.0_x);
    ARIADNE_TEST_EQUALS(r2.radii()[1],0.4375_x);

    ARIADNE_TEST_EQUALS(r2.measure(), 1.75_x);

    ARIADNE_TEST_ASSERT(r2.midpoint()==PointType({0.5_x,0.0625_x},dp));

    ARIADNE_TEST_ASSERT(decide(!disjoint(r1,r1)));
    ARIADNE_TEST_ASSERT(definitely(subset(r1,r1)));

    ARIADNE_TEST_ASSERT(definitely(contains(r1,s1)));
    ARIADNE_TEST_ASSERT(decide(!contains(r1,s3)));
    ARIADNE_TEST_ASSERT(decide(contains(r1,s5)));
    //ARIADNE_TEST_ASSERT(!subset_of_interior(r1,r1));
    //ARIADNE_TEST_ASSERT(subset_of_open_cover(r1,cover1));
    //ARIADNE_TEST_ASSERT(!subset_of_open_cover(r1,cover2));

    ARIADNE_TEST_EXECUTE( r4=intersection(r1,r2) );
    ARIADNE_TEST_ASSERT(r4==r6);

    ARIADNE_TEST_EXECUTE( r1[1]=IntervalType(0.0_x,0.5_x) );
    ARIADNE_TEST_ASSERT(r1==r6);

    ARIADNE_TEST_ASSERT(definitely(intersect(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{1.0_x,2.0_x},{1.5_x,2.5_x}})));
    ARIADNE_TEST_ASSERT(definitely_not(intersect(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{1.5_x,2.0_x},{1.5_x,2.5_x}})));

    ARIADNE_TEST_ASSERT(definitely(disjoint(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{1.5_x,2.0_x},{1.5_x,2.5_x}})));
    ARIADNE_TEST_ASSERT(definitely_not(disjoint(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{1.0_x,2.0_x},{1.5_x,2.5_x}})));

    ARIADNE_TEST_ASSERT(definitely(subset(BoxType{{0.5_x,1.25_x},{1.5_x,2.0_x}},BoxType{{0.0_x,1.25_x},{1.0_x,2.5_x}})));
    ARIADNE_TEST_ASSERT(definitely_not(subset(BoxType{{0.5_x,1.5_x},{1.5_x,2.0_x}},BoxType{{0.0_x,1.0_x},{1.0_x,2.5_x}})));

    ARIADNE_TEST_ASSERT(definitely(superset(BoxType{{0.0_x,1.25_x},{1.0_x,2.5_x}},BoxType{{0.5_x,1.25_x},{1.5_x,2.0_x}})));
    ARIADNE_TEST_ASSERT(definitely_not(superset(BoxType{{0.0_x,1.0_x},{1.0_x,2.5_x}},BoxType{{0.5_x,1.5_x},{1.5_x,2.0_x}})));

    ARIADNE_TEST_ASSERT(definitely(covers(BoxType{{0.0_x,1.5_x},{1.0_x,2.5_x}},BoxType{{0.5_x,1.0_x},{1.5_x,2.0_x}})));
    ARIADNE_TEST_ASSERT(possibly_not(covers(BoxType{{0.0_x,1.25_x},{1.0_x,2.5_x}},BoxType{{0.5_x,1.25_x},{1.5_x,2.0_x}})));

    ARIADNE_TEST_ASSERT(definitely(overlap(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{0.5_x,1.5_x},{1.5_x,2.5_x}})));
    ARIADNE_TEST_ASSERT(possibly_not(overlap(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{1.0_x,2.0_x},{1.5_x,2.5_x}})));

    ARIADNE_TEST_ASSERT(definitely(inside(BoxType{{0.5_x,1.0_x},{1.5_x,2.0_x}},BoxType{{0.0_x,1.5_x},{1.0_x,2.5_x}})));
    ARIADNE_TEST_ASSERT(possibly_not(inside(BoxType{{0.5_x,1.25_x},{1.5_x,2.0_x}},BoxType{{0.0_x,1.25_x},{1.0_x,2.5_x}})));

    ARIADNE_TEST_ASSERT(definitely(separated(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{1.5_x,2.5_x},{1.5_x,2.5_x}})));
    ARIADNE_TEST_ASSERT(possibly_not(separated(BoxType{{0.0_x,1.0_x},{1.0_x,2.0_x}},BoxType{{1.0_x,2.0_x},{1.5_x,2.5_x}})));



    }
};


int main() {
    TestBoxType<FloatDPExactBox>().test();

    return ARIADNE_TEST_FAILURES;
}
