/***************************************************************************
 *            test_point_curve.cpp
 *
 *  Copyright  2005-21  Luca Geretti
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
#include "geometry/point.hpp"
#include "geometry/curve.hpp"
#include "symbolic/space.hpp"
#include "symbolic/assignment.hpp"
#include "io/figure.hpp"

#include "../test.hpp"

using namespace Ariadne;

template<class X> class TestPoint {
    typedef LabelledPoint<X> PointType;
  public:
    void test() {
        ARIADNE_TEST_CALL(test_creation());
        ARIADNE_TEST_CALL(test_projection());
    }

    void test_creation() {
        RealVariable x("x"), y("y");
        LabelledPoint<Real> rpt({x=3,y=2.1_dec});
        PointType pt(rpt,dp);

        ARIADNE_TEST_EQUALS(pt.dimension(),2);
        ARIADNE_TEST_PRINT(pt.bounding_box());
    }

    void test_projection() {
        RealVariable x("x"), y("y"), z("z");
        LabelledPoint<Real> rpt({x=3,y=2.1_dec,z=1/5_q});
        PointType pt(rpt,dp);
        ARIADNE_TEST_PRINT(pt);
        ARIADNE_TEST_EQUALS(pt.dimension(),3);

        auto pt_pr = project(pt,Variables2d(z,x));
        ARIADNE_TEST_PRINT(pt_pr);
        ARIADNE_TEST_EQUALS(pt_pr.dimension(),2);
        ARIADNE_TEST_EQUAL(pt.at(0),pt_pr.at(1));
        ARIADNE_TEST_EQUAL(pt.at(2),pt_pr.at(0));
    }
};

class TestCurve {
    typedef LabelledInterpolatedCurve CurveType;
    typedef CurveType::PointType PointType;
  public:
    void test() {
        ARIADNE_TEST_CALL(test_creation());
    }

    void test_creation() {
        RealVariable x("x"), y("y");
        LabelledPoint<Real> rpt({x=3,y=2.1_dec});
        PointType pt(rpt,dp);
        CurveType curve(pt);
        ARIADNE_TEST_EQUALS(curve.dimension(),2);
        ARIADNE_TEST_EQUALS(curve.size(),1);
        curve.insert(FloatDP(1.0_x,dp),pt);
        ARIADNE_TEST_EQUALS(curve.size(),2);
    }
};


int main() {
    TestPoint<FloatDPApproximation>().test();
    TestCurve().test();

    return ARIADNE_TEST_FAILURES;
}
