#!/usr/bin/python3

##############################################################################
#            computing_pi.py
#
#  Copyright  2021  Pieter Collins
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


def compute_pi():
    # There are many formulae for pi.
    # We will use the identity tan(pi/6)=1/sqrt(3),
    # yielding pi=6*atan(1/sqrt(3))

    r = 6*atan(rec(sqrt(3))) # rec(x) is the reciprocal 1/x

    print("r:",r,type(r))    # r has type Real, and is represented by a symbolic formula

    # Compute r to an accuracy of 20 binary digits of precision (approximately 6 decimal places)
    acc = Accuracy(1/two**20)
    print("acc:",acc)
    v=r.compute(acc)
    print("v:",v,type(v))

    # The result is stored as a Dyadic lower and upper bound for r.
    # To output  a double-precision approversion
    pr=double_precision
    pr=DoublePrecision() # Alternative syntax using constructor
    pr=dp                # Alternative syntax using abbreviation
    x=v.get(pr)
    print("x:",x,type(x))

    print("sin(x/6):",sin(x/6),type(sin(x/6)))
    print("sin(x/4):",sin(x/4))

    print()

    # Compute r to an accuracy of 1/2^120 binary digits of precision (approximately 40 decimal places)
    acc=Accuracy(Dyadic(1,120))
    print("acc:",acc)
    v=r.compute(acc)
    print("v:",v,type(v))

    # To output double-precision approximation
    pr=double_precision
    x=v.get(pr)
    print("x:",x,type(x))

    # To output multiple-precision approversion with 128bits
    pr=precision(128)
    pr=multiple_precision(128)
    pr=MultiplePrecision(128)
    pr=MP(128)
    x=v.get(pr)
    print("x:",x,type(x))
    print("repr(x):",repr(x),type(x))
    print("x.precision():",x.precision())
    print("sin(x/6):",sin(x/6))
    print("sin(x/4):",sin(x/4))

    print()

    three=Bounds[FloatMP](3,precision(128))
    x=6*atan(1/sqrt(three))
    print("x:",x,type(x))

    print()

    acc=Accuracy(1/two**20)
    print("pi:",pi,type(pi))
    print("pi.compute(acc):",pi.compute(acc))

if __name__=='__main__':
    compute_pi()
