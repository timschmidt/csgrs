/***************************************************************************
 *            algebra_demonstration.cpp
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


void linear_algebra_demonstration() {
    //! [Linear Algebra demonstration]

    // Create an interval vector
    auto b=FloatDPBoundsVector({{1,1},{2,3},{3.875_x,4.125_x}},double_precision);
    print("b:",b);

    auto Aq=RationalMatrix({{1,2,4},{3,1,2},{0,0,1}});
    print("Aq:",Aq);

    // Create an interval matrix
    auto A=FloatDPBoundsMatrix({{1,2,4},{3,1.5_x,2},{0,0,1}},dp);
    print("A:",A);

    // Solve the linear equation Ax=b
    auto x=solve(A,b);
    print("A\\b:",x);
    //! [Linear Algebra demonstration]
}


int main(int argc, const char* argv[]) {
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    linear_algebra_demonstration();
}



