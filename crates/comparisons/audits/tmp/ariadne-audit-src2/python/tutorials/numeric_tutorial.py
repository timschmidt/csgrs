#!/usr/bin/python3

##############################################################################
#            numeric_tutorial.py
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

#! \file numeric_tutorial.py


# Import all classes in the ariadne module
from pyariadne import *

def compute_iterates(f,x0,n):
    xs=[x0]
    for i in range(n):
        xs.append(f(xs[-1]))
    return xs

def print_iterates(xs,str="x"):
    for i in range(len(xs)):
        print(str,'[',i,']:',xs[i])


def numeric_tutorial():
    #! [Exact arithmetic]


    #! Create an integer
    b=Integer(1)
    b=z_(1)

    #! Create the dyadic number 3+5/8=29/8=29/2^3
    a=Dyadic(29,3)  # Constructs the number 3.625=29/2^3.
    a=29/two**3     # Alternative syntax for 3.625=29/2^3
    a=3+5/two**3     # Alternative formula for 3.625=3+5/2^3

    #! Define the map f(x) = a * x * (b-x) with a=3.625 and b=1
    def f(x):
        return a*x*(b-x)

    print("Computing iterates using dyadic arithmetic")
    x0=1/two
    xs=compute_iterates(f,x0,8)
    print_iterates(xs)

    print("Computing iterates using rational arithmetic")
    x0=Rational(1,2)
    xs=compute_iterates(f,x0,8)
    print_iterates(xs)




    #! [The problem with float]

    a=ExactDouble(3.625)
    a=x_(3.625)

    a=Decimal(3.625)
    a=Decimal("3.625")

    a=dy_(3.625)
    a=dec_(3.625)
    a=q_(3.625)
    b=z_(1)

    a=Decimal(3.625)
    b=Integer(1)
    def f(x):
        return a*x*(b-x)

    def f(x):
        return dec_(3.625)*x*(1-x)


    #! [Floating-point arithmetic]

    print("Computing iterates using double-precision bounds")
    x0=FloatDPBounds(q_(1/2),double_precision)
    x0=Bounds[FloatDP](q_(1/2),double_precision)
    xs=compute_iterates(f,x0,16)
    print_iterates(xs)

    print("Computing iterates using multiple-precision bounds")
    x0=Bounds[FloatMP](q_(1/2),precision(bits=192))
    xs=compute_iterates(f,x0,16)
    print_iterates(xs)

    print("Computing iterates using multiple-precision ball with double-precision error")
    x0=Ball[FloatMP,FloatDP](q_(1/2),precision(bits=192))
    xs=compute_iterates(f,x0,16)
    print_iterates(xs)

    print("Computing iterates using double-precision approximate arithmetic")
    x0=Approximation[FloatDP](0.5,double_precision)
    xs=compute_iterates(f,x0,16)
    print_iterates(xs)



    #! [Rounded arithmetic]

    a=FloatDP(exact(3.625),double_precision)

    a=FloatDP(Rational(38,10),to_nearest,double_precision)
    b=FloatDP(1,double_precision)
    x=FloatDP(Dyadic(1/two),double_precision)
    fx=mul(near,a,mul(near,x,sub(near,b,x)))



    #! [Real arithmetic]

    a=Decimal(3.625)
    b=Integer(1)
    def f(x): return a*x*(b-x)

    x=Real(q_(3/5))
    for i in range(8): x=f(x)
    print("x:",x)
    print("Computing 8th iterate to 128 binary places of accuracy.")
    y=x.compute(Accuracy(bips=128))
    print("y:",y)
    w=y.get()
    print("y.get():",y.get())
    w=y.get(double_precision)
    print("y.get(dp):",y.get(double_precision))
    w=y.get(precision(bits=128))
    print("y.get(mp):",w)

    print("Computing 8th iterate using effort of '2':")
    y=x.compute(Effort(2))
    print("x.compute(Effort(2)):",y)
    print("Computing iterate using 128 bits precision:")
    w=x.compute_using(precision(bits=128))
    print("x.compute_using(precision(bits=128)):",w)



if __name__=='__main__':
    numeric_tutorial()
