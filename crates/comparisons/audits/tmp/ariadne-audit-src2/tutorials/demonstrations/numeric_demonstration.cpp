/***************************************************************************
 *            numeric_demonstration.cpp
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


void numeric_demonstration() {
    //! [Numeric demonstration]

    // Number classes and their constructors

    // Create an integer
    auto z=Integer(5);
    print("z:",z);

    // Create a dyadic; can convert from Integer
    auto w=Dyadic(z);
    w=Dyadic(5);
    w=Dyadic(11,3u); // 11/2^3; Dyadic(11,3) is deleted since second argument must be positive (exponent of denominator)
    w=11/(two^3u); // ^ in C++ is bitwise-xor, not power, and has lower precedence
    print("w:",w);

    // Create a decimal number; can convert from Dyadic
    auto g=Decimal(w);
    g=Decimal(9.81);
    g=Decimal("9.81");
    print("g:",g);

    // Create a rational; can convert from Dyadic, Decimal
    auto q=Rational(w);
    q=Rational(5);
    q=Rational(11,8);
    print("q:",q);

    // Create a real number
    auto r=Real(q);
    print("r:",r);

    // Operations on real numbers
    // Arithmetic operators
    +r; -r; r+r; r-r; r*r; r/r;
    // Arithmetic operations
    neg(r); sqr(r); hlf(r); rec(r); pow(r,-3);
    // Algebraic and transcendental operations
    sqrt(r); exp(r); log(r); sin(r); cos(r); tan(r); atan(r);
    // Lattice operations
    abs(r); max(r,r); min(r,r);
    // Comparison operators
    auto k = r<=r;
    // Metric
    dist(r,r);

    // Store a double-precision floating-point number
    auto d=ExactDouble(1.375);
    d=1.375_x;
    print("d:",d);
    // Can convert an ExactDouble to a Dyadic number.
    w=Dyadic(d);

    // Specify precisions of floating-point number types
    auto dp=DoublePrecision();
    dp=double_precision;
    auto mp=MultiplePrecision(128);

    // Create a raw double-precision number
    auto xdp=FloatDP(1.75_x,dp);
    print("FloatDP(1.75_x):",xdp);
    // Create a raw multiple-precision number
    auto xmp=FloatMP(1.75_x,mp);
    print("FloatMP(1.75_x):",xmp);

    // Create double-precision bounds for a value
    auto xdpb=FloatDPBounds(Decimal(1.2),dp); // Creates the interval [1.19999999999999996:1.20000000000000018]
    print("FloatDPBounds(1.2):",xdpb);

    // Create double-precision bounds for a range of values
    auto xmpb=FloatDPBounds(Rational(11,10),Rational(14,10),dp); // Creates the interval [1.09999999999999987:1.40000000000000013]
    print("FloatDPBounds(11/10,14/10,dp):",xmpb);

    // Create multiple-precision bounds for a value
    xmpb=FloatMPBounds("1.2"_dec,mp); // Creates the interval [1.19999999999999996:19999999999999996]
    print("FloatMPBounds(1.2,mp):",xmpb);

    // Create multiple-precision bounds for a range of values
    xmpb=FloatMPBounds(1.5_dec,2.25_dec,mp); // Creates the interval [1.5,2.25]
    print("FloatMPBounds(1.5,2.25,mp):",xmpb);

    // Create multiple-precision bounds for a range of values
    xmpb=FloatMPBounds(11/10_q,14/10_q,mp); // Creates the interval [1.10000000000000009:1.39999999999999991]
    print("FloatMPBounds(11/10,14/10,mp):",xmpb);

    // Create a double-precision approximation
    auto xdpa=FloatDPApproximation(1.23,dp);
    print("FloatDPApproximation(1.23,dp):",xdpa);

    auto xmpa=FloatMPApproximation(1.23,mp);
    print("FloatMPApproximation(1.23,dp):",xmpa);
    xmpa=FloatMPApproximation("1.23"_dec,mp);
    print("FloatMPApproximation(\"1.23\"_dec,dp):",xmpa);
    //! [Numeric demonstration]
}


int main(int argc, const char* argv[]) {
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    numeric_demonstration();
}



