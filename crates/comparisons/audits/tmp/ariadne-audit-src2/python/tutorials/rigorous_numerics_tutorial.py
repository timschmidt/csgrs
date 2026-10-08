#!/usr/bin/python3

##############################################################################
#            rigorous_numerics_tutorial.py
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

def bits(n):
    return n

def PRINT(expr):
    print(expr,":",eval(expr),type(eval(expr)))

if __name__=='__main__':

    from sys import argv
    if not CommandLineInterface.instance().acquire(argv):
        exit()

#! [numeric_demonstration]
    print("Numeric")
    r = 6 * atan(1/sqrt(Real(3)))
    print(r)
    xdp=r.get(double_precision)
    print("xdp=",xdp,type(xdp))
    PRINT("xdp")
    xmp=r.get(precision(128))
    print("xmp=",xmp,type(xmp))
    print(xmp.error()); print(xmp-xmp); print(xmp-xmp+1)
    ymp=r.compute(Effort(128))
    print("ymp=",ymp,type(ymp))
    zmp=Ball[FloatMP](r.compute(Accuracy(128)).get(precision(128)))
    print("zmp=",zmp,type(zmp))
    print(zmp.error()); print(zmp-zmp); print(zmp-zmp+1)

    y=EffectiveNumber(r)
    print("y:",y,type(y))
#! [numeric_demonstration]


#! [expression_demonstration]
    print("Expression")
    x=Variable[Real]("x")
    y=Variable[Real]("y")
    c=Constant[Real]("c",dy_(3.75))
    e = c * x * (1-x)
    print(x); print(c); print(e)
    x0=Rational(1,2)
    y0=Rational(-2,3)
    v=Valuation[Real]({x:x0,y:y0})
    print(v)
    x1=evaluate(e,v)
    print("x1:",x1,"=",x1.get(double_precision))
#! [expression_demonstration]

#! [linear_algebra_demonstration]
    print("Linear Algebra")
    A=FloatMPApproximationMatrix([[4,1,0],[1,4,1],[0,1,4]],precision(bits(128)))
    v=FloatMPApproximationVector([2.0,3,5],precision(bits(128)))
    print(inverse(A))
    FloatMPApproximation.set_output_places(30)
    print(inverse(A))
    print(solve(A,v))

    A_approx=FloatDPBoundsMatrix([[4,1,0],[1,4,1],[0,1,4]],double_precision)
    v_approx=FloatDPBoundsVector([exact(2.0),3,5],double_precision)
    print(A_approx)
    print(inverse(A_approx))
    print(solve(A_approx,v_approx))
    print(gs_solve(A_approx,v_approx))
#! [linear_algebra_demonstration]

#! [function_demonstration]
    print("Function")
    a=Real(dy_(1.875))
    b=Real(dec_(0.3))
    id=EffectiveVectorMultivariateFunction.identity(EuclideanDomain(2))
    x=id[0]
    y=id[1]
    h=EffectiveVectorMultivariateFunction([a-x*x-b*y,x])
    v=FloatDPVector([exact(0.5),exact(1.0)],double_precision)
    print(v)
    print(h(v))
    print(evaluate(h,v))
    print(h.jacobian(v))
    print(jacobian(h,v))
    print(h.differential(v,3))

    dom=BoxDomainType([(0,1),(exact(0.5),exact(1.5))])
    th = ValidatedVectorMultivariateTaylorFunctionModelDP(dom,h,ThresholdSweeperDP(double_precision,1e-4))
    print(th)
    thh=compose(h,th)
    print(thh)
    print(evaluate(thh,v))
    print(h(h(v)))
#! [function_demonstration]

#! [geometry_demonstration]
    print("Geometry")
    x=EffectiveScalarMultivariateFunction.coordinate(EuclideanDomain(2),0)
    y=EffectiveScalarMultivariateFunction.coordinate(EuclideanDomain(2),1)
    g = sqr(x)+4*sqr(y)
    h = EffectiveVectorMultivariateFunction([1+x+y*y,2+x-y])
    c=EffectiveConstraint(g<=1)
    cs=ConstraintSet([c])
    bx=RealBox([(-2,+2),(-2,+2)])

    assert(type(g<=1)==EffectiveConstraint)

    bbx=FloatDPApproximateBox([(-2,+4),(-2,+4)])
    bcs=BoundedConstraintSet([(-2,+2),(-2,+2)],[g<=1])
    bcs=intersection(bx,cs)
    cis=ConstrainedImageSet=image(bcs,h)
    print(cis)
    fig=Figure(bbx,Projection2d(2,0,1)) # Set up a figure for a space restricted to the bounding box, drawing first two coordinates
    fig.set_fill_colour(0.0,0.5,0.5)
    fig.draw(bbx)
    fig.set_fill_colour(0,1,1)
    fig.draw(cis) # Use chained methods to control graphics
    fig.write("rigorous_numerics_tutorial") #
#    plot("rigorous_numerics_tutorial",Projection2d(2,0,1),bbx,[(Colour(1,0.5,0.5),bbx),(Colour(0,1,1),cis)]) # Plot in one command
    plot("rigorous_numerics_tutorial",Projection2d(2,0,1),bbx,[(Colour(0,1,1),cis)]) # Plot in one command
#! [geometry_demonstration]

