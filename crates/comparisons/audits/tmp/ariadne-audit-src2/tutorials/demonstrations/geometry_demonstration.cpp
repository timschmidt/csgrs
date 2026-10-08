/***************************************************************************
 *            geometry_demonstration.cpp
 *
 *  Copyright  2009-21  Pieter Collins
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

#include "ariadne.hpp"

using namespace Ariadne;

void print() { CONCLOG_PRINTLN(""); }
template<class T> void print(const char* label, T const& expr) { CONCLOG_PRINTLN(label << ": " << (expr)) }


void geometry_demonstration() {
    //! [Geometry demonstration]

    // Create intervals with different endpoint types
    auto ivlq = RationalInterval(4/7_q,3/5_q);
    print("ivlq:",ivlq);
    auto ivlu = FloatDPUpperInterval({1.5_x,1.5_x});
    print("ivlu:",ivlu);

    // Create a box from a list of intervals
    // Interval literals can be given as {l,u}
    auto bx = RealBox({ {4/7_q,3/5_q} , {1.5_x,2.5_x} });
    print("bx",bx);

    // Test geometric predicates
    subset(RationalInterval{2,3},RationalInterval{1,4});
    disjoint(RealInterval{2,3},RealInterval{4,5});

    // Create sets based on constraints
    auto x = EffectiveScalarMultivariateFunction::coordinate(2,0);
    auto y = EffectiveScalarMultivariateFunction::coordinate(2,1);

    // Create a set based on constraints
    auto cs = ConstraintSet({sqr(x)+2*sqr(y)<=1,3*x+2*y>=1});
    print("cs:",cs);

    // Compute the intersection of a contraint set with a box, obtaining a bounded set
    auto bcs = intersection(bx,cs);
    print("bcs:",bcs);

    // Compute the image of a bounded contraint set under a continuous function
    auto h=EffectiveVectorMultivariateFunction({1.5_dec-x*x-y/3,y});
    auto cis = image(bcs,h);
    print("cis:",cis);

    // Discretise the constrained image set on a grid
    auto g=Grid(2);
    auto dpth=6u;
    auto gtp = outer_approximation(cis,g,dpth);
    print("gtp:",gtp);

    // Plot an approximation to the set
    auto prj=Projection2d(2,0,1);  // The identity projection
    auto wnd=ApproximateBoxType({{0,3},{-2,+2}});  // The view window
    auto green=Colour(0,1,0);
    plot("geometry_demonstration", prj, wnd, { {green,cis} });

    //! [Geometry demonstration]
}


int main(int argc, const char* argv[]) {
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    geometry_demonstration();
}



