#!/usr/bin/python3

##############################################################################
#            function_tutorial.py
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


def function_tutorial():
    #! [Univariate function tutorial]

    id=EffectiveScalarUnivariateFunction.coordinate()
    a=dec_(3.712)
    b=z_(1)
    f=a*id*(b-id)
    print("f:",f)

    x=RealVariable("x")
    f=make_function(x,a*x*(b-x))
    print("f:",f)

    f2=compose(f,f)
    f3=compose(f,f2)

    x0=q_(3/5)
    x=Approximation(x0,double_precision)
    fffx=f(f(f(x)))
    print("fffx:",fffx)
    f3x=f3(x)
    print("f3x:",f3x)
    f3x=evaluate(f3,x)
    print("evaluate(f3,x):",f3x)

    x=Bounds(x0,precision(bits=128))
    fffx=f(f(f(x)))
    print("fffx:",fffx)
    f3x=f3(x)
    print("f3x:",f3x)

    df3=f3.derivative()
    print("df3:",df3)
    print("df3(x):",df3(x))
    df3x=slope(f3,x)
    print("slope(f3,x):",df3x)


    #! [Multivariate function tutorial]

    a=dec_(0.7)
    b=dec_(0.4)
    c=6

    id=EffectiveVectorMultivariateFunction.identity(2)
    x=id[0]
    y=id[1]
    t=EffectiveScalarMultivariateFunction(b-c/(1+sqr(x)+sqr(y)))
    f=EffectiveVectorMultivariateFunction([1+a*(x*cos(t)-y*sin(t)),a*(x*sin(t)+y*cos(t))])

    x=EffectiveScalarMultivariateFunction.coordinate(2,0)
    y=EffectiveScalarMultivariateFunction.coordinate(2,1)
    t=EffectiveScalarMultivariateFunction(b-c/(1+sqr(x)+sqr(y)))
    f=EffectiveVectorMultivariateFunction([1+a*(x*cos(t)-y*sin(t)),a*(x*sin(t)+y*cos(t))])

    x=RealVariable("x")
    y=RealVariable("y")
    t=b-c/(1+x*x+y*y)
    f=make_function([x,y],[1+a*(x*cos(t)-y*sin(t)), a*(x*sin(t)+y*cos(t))])

    f2=compose(f,f)
    df0dx1=derivative(f[0],1)

    x=Vector[Approximation[FloatDP]]([0.1,0.2],dp)
    fx=f(x)
    print("f(x):",fx)
    fx=evaluate(f,x)
    print("evaluate(f,x):",fx)

    mp=precision(128)
    x=Vector[Bounds[FloatMP]]([dec_(0.1),dec_(0.2)],mp)
    fx=f(x)
    print("f(x):",fx)
    fx=evaluate(f,x)
    print("evaluate(f,x):",fx)

    jfx=jacobian(f,x)
    print("jacobian(f,x):",jfx)

    ddfx=differential(f,x,2)
    print("differential(f,x,2):",ddfx)


    #! [Function model]

    dom=BoxDomainType([[x_(-0.25),x_(0.25)],[0,x_(0.5)]])
    swp=ThresholdSweeper[FloatDP](dp,1e-3)
    p=ValidatedVectorMultivariateTaylorFunctionModelDP(dom,f,swp)
    print("p:",p)

    fp=compose(f,p)
    print("compose(f,p):",fp)
    x=Vector[Bounds[FloatDP]]([dec_(0.1),dec_(0.2)],dp)
    px=p(x)
    print("p(x):",p)
    #px=evaluate(p,x)
    #print("evaluate(p,x):",p)
    If0x1=antiderivative(p[0],1)
    print("antiderivative(p[0],1):",If0x1)


if __name__=='__main__':
    function_tutorial()
