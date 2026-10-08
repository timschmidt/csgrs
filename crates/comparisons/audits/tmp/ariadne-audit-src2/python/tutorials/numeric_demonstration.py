#!/usr/bin/python3

##############################################################################
#            numeric_demonstration.py
#
#  Copyright  2009-21  Pieter Collins
##############################################################################

# This file is part of Ariadne.

# Ariadne is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Ariadne is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with Ariadne. If not, see <https://www.gnu.org/licenses/>.

# Import all classes in the ariadne module
from pyariadne import *

# Print a list of all available classes and functions
print(dir())


def numeric_demonstration():
    #! [Numeric demonstration]

    # Number classes and their constructors

    # Create an integer
    z=Integer(5)
    print("Integer(5):",z)

    # Create a dyadic; can convert from Integer
    w=Dyadic(z)
    w=Dyadic(5)
    w=Dyadic(11,3) # Constructs the number 11/2^3=11/8
    w=11/two**3    # Alternative syntax for 11/2^3
    print("Dyadic(11,3):",w)

    # Create a decimal number; can convert from Dyadic
    g=Decimal(w)
    g=Decimal(9.81)
    g=Decimal("9.81")
    print("Decimal(9.81):",g,type(g))

    # Create a rational; can convert from Dyadic, Decimal
    q=Rational(w);
    q=Rational(5);
    q=Rational(11,8);
    print("Rational(11,8):",q,type(q))

    # Create a real number
    r=Real(q)
    print("Real(4*atan(1)):",r)
    print()

    # Operations on real numbers
    # Arithmetic operators
    +r; -r; r+r; r-r; r*r; r/r
    # Aliases for arithmetic operators (currently not fully supported)
    pos(r); neg(r); # add(r,r); sub(r,r); mul(r,r); div(r,r);
    # Other arithmetic operations
    nul(r); sqr(r); hlf(r); rec(r); pow(r,-3)
    # Algebraic and transcendental operations
    sqrt(r); exp(r); log(r); sin(r); cos(r); tan(r); atan(r)
    # Lattice operations
    abs(r); max(r,r); min(r,r)
    # Comparison operators
    k=(r<=r)
    # Metric
    dist(r,r)


    # Store a double-precision floating-point number
    d=ExactDouble(1.375)
    d=cast_exact(1.375)
    d=x_(1.375)
    print("ExactDouble(1.375):",d)
    print("cast_exact(1.375):",d)
    print("x_(1.375):",d)
    # Can convert an ExactDouble to a Dyadic number.
    w=Dyadic(d)
    print("Dyadic(ExactDouble(1.375))",w)
    print()

    # Define shorthands for defining Ariadne values from input
    def dec(x): return Decimal(x)
    def ex(x): return ExactDouble(x)
    def dy(x): return Dyadic(ExactDouble(x))


    # Specify precisions of floating-point number types
    dp=DoublePrecision()
    dp=double_precision
    mp=MultiplePrecision(128)

    # Create a raw double-precision number
    xdp=FloatDP(exact(1.75),dp)
    # Create a raw double-precision number using generic Float type
    xdp=Float[DP](exact(1.75),dp)
    # Create a raw multiple-precision number
    xmp=FloatMP(exact(1.75),mp)
    xmp=Float[MP](exact(1.75),mp)

    # Create double-precision bounds for a value
    xdpb=FloatDPBounds(Decimal(1.2),dp) # Creates the interval [1.19999999999999996:1.20000000000000018]
    print("FloatDPBounds(1.2):",xdpb)
    # Create double-precision bounds for a value using generic Bounds type
    xdpb=Bounds[FloatDP](Decimal(1.2),dp) # Creates the interval [1.19999999999999996:1.20000000000000018]
    print("Bounds[FloatDP](1.2):",xdpb)
    xdpb=Bounds[Float[DP]](Decimal(1.2),dp) # Creates the interval [1.19999999999999996:1.20000000000000018]
    print("Bounds[Float[DP]](1.2):",xdpb)

    # Create double-precision bounds for a range of values
    xdpb=FloatDPBounds(Rational(11,10),Rational(14,10),dp) # Creates the interval [1.09999999999999987:1.40000000000000013]
    print("FloatDPBounds(11/10,14/10,dp):",xdpb)

    # Create multiple-precision bounds for a value
    xmpb=FloatMPBounds(dec("1.2"),mp) # Creates the interval [1.19999999999999996:19999999999999996]
    print("FloatMPBounds(1.2,mp):",xmpb)

    # Create multiple-precision bounds for a range of values
    xmpb=FloatMPBounds(dec(1.5),dec(2.25),mp) # Creates the interval [1.5,2.25]
    print("FloatMPBounds(1.5,2.25,mp):",xmpb)

    # Create multiple-precision bounds for a range of values
    xmpb=FloatMPBounds(Rational(11,10),Rational(14,10),mp) # Creates the interval [1.10000000000000009:1.39999999999999991]
    print("FloatMPBounds(11/10,14/10,mp):",xmpb)

    # Create a double-precision approximation
    xdpa=FloatDPApproximation(1.23,dp)
    print("FloatDPApproximation(1.23,dp):",xdpa)
    # Create a double-precision approximation using generic Approximation type
    xdpa=Approximation[FloatDP](1.23,dp)
    print("Approximation[FloatDP](1.23,dp):",xdpa)
    xdpa=Approximation[Float[DP]](1.23,dp)
    print("Approximation[Float[DP]](1.23,dp):",xdpa)

    xmpa=FloatMPApproximation(1.23,mp)
    print("FloatMPApproximation(1.23,dp):",xmpa)
    xmpa=FloatMPApproximation(Decimal("1.23"),mp)
    print("FloatMPApproximation(Decimal(\"1.23\"),dp):",xmpa)
    #! [Numeric demonstration]




if __name__=='__main__':
    numeric_demonstration()
