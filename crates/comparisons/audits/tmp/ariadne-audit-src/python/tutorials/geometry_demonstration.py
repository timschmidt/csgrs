#!/usr/bin/python3

##############################################################################
#            geometry_demonstration.py
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


def geometry_demonstration():
    #! [Geometry demonstration]

    # Create intervals with different endpoint types
    ivlq = RationalInterval(Rational(4,7),Rational(3,5))
    ivlq = Interval[Rational](Rational(4,7),Rational(3,5))
    print("ivlq:",ivlq,type(ivlq))
    ivlu = FloatDPUpperInterval({exact(1.5):exact(2.5)})
    ivlu = Interval[FloatDPUpperBound]({exact(1.5):exact(2.5)})
    print("ivlu:",ivlu,type(ivlu))

    # Create a box from a list of intervals
    # Interval literals can be given as [l,u] or (for int/float values) as {l:u}
    bx = RealBox([ [Rational(4,7),Rational(3,5)] , {exact(1.5):exact(2.5)} ])
    bx = Box[Real]([ [Rational(4,7),Rational(3,5)] , {exact(1.5):exact(2.5)} ])
    print("bx",bx,type(bx))

    # Test geometric predicates
    subset({2:3},{1:4})
    disjoint({2:3},{4:5})

    # Create sets based on constraints
    x = EffectiveScalarMultivariateFunction.coordinate(2,0)
    y = EffectiveScalarMultivariateFunction.coordinate(2,1)
    cs = ConstraintSet([sqr(x)+2*sqr(y)<=1,3*x+2*y>=1])
    print("cs:",cs)

    bx = Box[Real]([[-1,+1],[-1,+1]])
    # Compute the intersection of a contraint set with a box, obtaining a bounded set
    bcs = intersection(bx,cs)
    print("bcs:",bcs,type(bcs))

    # Compute the image of a bounded contraint set under a continuous function
    h=EffectiveVectorMultivariateFunction([dec_(1.5)-x*x-y/3,y])
    cis = image(bcs,h)
    print("cis:",cis,type(cis))

    # Discretise the constrained image set on a grid
    g=Grid(2)
    dpth=4
    gtp = outer_approximation(cis,g,dpth)
    print("gtp:",gtp,type(gtp))


    (cis1,cis2)=cis.split()
    (cis11,cis12)=cis1.split()
    (cis21,cis22)=cis2.split()

    # Plot an approximation to the set
    prj=Projection2d(2,0,1) # The identity projection
    wnd=[{0:3},{-2:+2}] # The view window
    green=Colour(0,1,0)
    plot("geometry_demonstration", prj, wnd, [(green,cis)])
    #! [Geometry demonstration]


if __name__=='__main__':
    geometry_demonstration()
