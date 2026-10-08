#!/usr/bin/python3

##############################################################################
#            function_demonstration.py
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


def function_demonstration():
    #! [Function demonstration]

    # Create a user-defined scalar-valued function
    x=[EffectiveScalarMultivariateFunction.coordinate(2,index) for index in range(0,2)]
    sf=sqrt(sqr(x[0])+sqr(x[1]))
    print("sf:",sf)
    a=FloatDPApproximationVector([4,3],dp)
    sfa=sf(a)
    print("sf(a):",sfa)
    b=FloatDPBoundsVector([4,3],dp)
    sfb=sf(b)
    print("sf(b):",sfb)

    x=EffectiveVectorMultivariateFunction.identity(3)
    vf=EffectiveVectorMultivariateFunction([sqrt(sqr(x[0])+sqr(x[1])),x[1]+x[2]])
    print("vf:",vf)
    a=FloatDPApproximationVector([4,3,0],dp)
    vfa=vf(a)
    print("vf(a):",vfa)
    b=FloatDPBoundsVector([4,3,0],dp)
    vfb=evaluate(vf,b)
    print("evaluate(vf,b):",vfb)

    dsf0=derivative(sf,0);
    print("derivative(sf,0):",dsf0);

    p0=FloatDPApproximationMultivariatePolynomial.coordinate(2,0,dp);
    p1=FloatDPApproximationMultivariatePolynomial.coordinate(2,1,dp);
    print("p1:",p1)
    a3=FloatDPApproximation(3,dp)
    a5=FloatDPApproximation(5,dp)
    q=p0*(p0+p1*a3)+a5
    print("q:",q)
    #! [Function demonstration]


def calculus_demonstration():
    #! [Calculus demonstration]

    # Create a box to act as the domain of a Taylor function
    dom=BoxDomainType([{4:7},{1:6},{-1:+1}])
    print("dom:",dom)

    # Create a sweeper to control the accuracy of a Taylor function
    swp=ThresholdSweeperDP(dp,1e-8)
    #swp=GradedSweeper(dp,6)
    print("swp:",swp)

    # Create the scalar Taylor model representing a constant function with given value on the domain dom
    tc=ValidatedScalarMultivariateTaylorFunctionModelDP.constant(dom,dec_(4.2),swp)
    print("tc:",tc,"\n")

    # Create the scalar Taylor model representing the function x1 on the domain dom
    t1=ValidatedScalarMultivariateTaylorFunctionModelDP.coordinate(dom,1,swp)
    print("t1:",t1,"\n")

    # Create all Taylor variables on the domain dom
    t=[ValidatedScalarMultivariateTaylorFunctionModelDP.coordinate(dom,i,swp) for i in range(0,3)]
    print("t[0]:",t[0])
    print("t[1]:",t[1])
    print("t[2]:",t[2])

    # Make shorthands for the variable names
    tx=t[0]
    ty=t[1]
    tz=t[2]
    print("tx:",tx)
    print("ty:",ty)
    print("tz:",tz)

    #Create a ValidatedScalarMultivariateTaylorFunctionModelDP from a Function
    p=EffectiveScalarMultivariateFunction.coordinate(3,0)
    tp=ValidatedScalarMultivariateTaylorFunctionModelDP(dom,p,swp)
    print("p:",p)
    print("tp:",tp)

    # The domain D of tx.
    print("tx.domain():",tx.domain())
    # A not-necessarily tight over-approximation to p(D)+/-e.
    print("tx.codomain():",tx.codomain())
    # An over-approximation to p(D)+/-e.
    print("tx.range():",tx.range())

    # Define some constants
    n=int(3)
    c=FloatDP(exact(0.75),dp)
    b=FloatDPBounds(exact(0.625),exact(0.875),dp);

    # Arithmetic on Taylor models
    +tx; -tx; tx+ty; tx-ty; tx*ty; tx/ty;

    # Mixed arithmetic
    tx+c; tx-c; tx*c; tx/c;
    c+tx; c-tx; c*tx; c/tx;

    tx+b; tx-b; tx*b; tx/b;
    b+tx; b-tx; b*tx; b/tx;

    # Lattice functions
    min(tx,ty); max(tx,ty); abs(tx);

    # Univariate arithmetical functions
    neg(tx);
    rec(tx);
    sqr(tx);
    pow(tx,n);

    # Algebraic and transcendental functions
    sqrt(tx);
    exp(tx); log(tx);
    sin(tx), cos(tx), tan(tx/2)

    # Inplace operations
    tx+=ty; tx-=ty;
    tx+=c; tx-=c; tx*=c; tx/=c;
    tx+=b; tx-=b; tx*=b; tx/=b;

    # Set coordinate functions tx0, tx1
    dom=BoxDomainType([{-1:+1},{-1:+1}])
    tx0=ValidatedScalarMultivariateTaylorFunctionModelDP.coordinate(dom,0,swp);
    tx1=ValidatedScalarMultivariateTaylorFunctionModelDP.coordinate(dom,1,swp);

    tf=ValidatedScalarMultivariateTaylorFunctionModelDP(1+2*tx0+3*tx0*tx1+tx0*tx0)
    # Compute an antiderivative of f with respect to x[0] whose value is zero at the midpoint of the domain of x[0]
    Itf0=antiderivative(tf,0)
    # Compute the antiderivative of f with respect to x[1] whose value is zero when x[1]=c
    Itf1=antiderivative(tf,1,c)
    print("antiderivative(tf,0):",Itf0)
    print("antiderivative(tf,1):",Itf1)

    # Restrict to a subdomain
    subdom=BoxDomainType([{exact(-0.125):exact(0.125)},{exact(0.5):exact(0.75)}])
    w=tx0*tx1
    rw=restriction(w,subdom)
    print("restriction(w,subdom):",rw)

    #Join tf and tg into a VectorTaylorFunction x:->[tf(x),tg(x)]
    tg=ValidatedVectorMultivariateTaylorFunctionModelDP([exp(tx0)*cos(tx1)])
    tfg=join(tf,tg)
    print("join(tf,tg):",tfg)

    # Combine tf and tg into a VectorTaylorFunction (x,y):->[tf(x),tg(y)]
    tfg=combine(tf,tg)
    print("combine(tf,tg):",tfg)

    # Function composition
    dom=BoxDomainType([{4:7},{1:6},{-1:1}])
    th=ValidatedVectorMultivariateTaylorFunctionModelDP.identity(dom,swp)
    cd=th.codomain()

    f=EffectiveScalarMultivariateFunction.coordinate(3,0)
    g=EffectiveVectorMultivariateFunction.zeros(2,3)

    tf=ValidatedScalarMultivariateTaylorFunctionModelDP(cd,f,swp)
    tg=ValidatedVectorMultivariateTaylorFunctionModelDP(cd,g,swp)

    # Compose an scalar multivariate function and a vector Taylor function
    fth=compose(f,th)
    print("compose(f,th):",fth)

    # Compose a vector multivariate function and a vector Taylor function
    gth=compose(g,th)
    print("compose(g,th):",gth)

    # Compose a scalar and a vector Taylor function
    assert(subset(th.codomain(),tf.domain()))
    tfh=compose(tf,th)
    print("compose(tf,th):",tfh)

    # Compose two vector Taylor functions
    assert(subset(th.codomain(),tg.domain()))
    tgh=compose(tg,th)
    print("compose(tg,th):",tgh)


    # Consistency and refinement checking
    dom=BoxDomainType([{-1:+1},{-1:+1}]);
    tf1=tx0+tx0*tx0/2
    tf2=tx0+FloatDPBounds(-1,1,dp)
    print("tf1:",tf1)
    print("tf2:",tf1)
    ic=inconsistent(tf1,tf2)
    print("inconsistent(tf1,tf2):",ic)
    rf=refines(tf1,tf2)
    print("refines(tf1,tf2):",rf)
    tf=refinement(tf1,tf2)
    print("refinement(tf1,tf2):",tf)
    #! [Calculus demonstration]


if __name__=='__main__':
    function_demonstration()
    print()
    calculus_demonstration()
