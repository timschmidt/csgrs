#!/usr/bin/python3

##############################################################################
#            neural_network.py
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

## \file neural_network.py

def create_neural_network():
    """! Create a neural network.
    """

    #! Create activation function from the identity function
    y=EffectiveScalarUnivariateFunction.identity()
    sigma=1/(1+exp(-y))
    sigma=rec(1+exp(-y))

    #! Create activation function using an argument variable
    y=Variable[Real]("y")
    sigma=make_function(y,rec(1+exp(-y)))
    print(sigma)

    Vector[Decimal]([dec_(1.2),3])
    #! Create the first layer
    A0=Matrix[Real]([[dec_(0.884),dec_(-1.097)],[dec_(1.586),dec_(0.380)],[dec_(-0.005),dec_(0.677)]])
    b0=Vector[Real]([dec_(-0.127),dec_(2.747),dec_(0.378)])
        # b0=Vector[Decimal]([-0.127,2.747,0.378]) # Proposed future construtor
    print(A0,b0)
    x0=EffectiveVectorMultivariateFunction.identity(2)
    f0=EffectiveVectorMultivariateFunction([compose(sigma,A0[0,0]*x0[0]+A0[0,1]*x0[1]+b0[0]),
                                            compose(sigma,A0[1,0]*x0[0]+A0[1,1]*x0[1]+b0[1]),
                                            compose(sigma,A0[2,0]*x0[0]+A0[2,1]*x0[1]+b0[2])])
    print("f0:",f0)

    #! Create second first layer
    A1=RealCovector([dec_(-0.541),dec_(-0.535),dec_(-0.255)])
    b1=Real(dec_(0.244))
    print(A1,b1)
    x1=EffectiveVectorMultivariateFunction.identity(3)
    f1=EffectiveScalarMultivariateFunction(compose(sigma,A1[0]*x1[0]+A1[1]*x1[1]+A1[2]*x1[2]+b1))
    print("f1:",f1)

    #! Combine the layers
    f=compose(f1,f0)
    print("f:",f)
    print()

    return f


def approximate_function(f):

    dom=BoxDomainType([[dy_(0.375),dy_(0.5)],[dy_(0.625),dy_(0.75)]])
    swp=ThresholdSweeperDP(double_precision,1e-9)
    model_f=ValidatedScalarMultivariateTaylorFunctionModelDP(dom,f,swp)
    print("model_f:",model_f)

    x=Vector[FloatDPBounds]([dy_(0.4375),dy_(0.6875)],dp)
    print("f(x):",f(x))
    print("model_f(x):",model_f(x))

    return model_f


def compute_derivatives(f):

    #! Define a point x at which to compute the derivatives of f
    x=Vector[FloatDPBounds]([dy_(0.4375),dy_(0.6875)],dp)
    #! Specify the degree of the highest derivative required
    deg=3

    #! Compute the derivates of f at x up to the requested degree
    dfx=f.differential(x,deg)
    dfx=differential(f,x,deg)
    print("dfx:",dfx,type(dfx))

    #! Extract the gradient of f from the differential:
    gfx=dfx.gradient()
    #! Or compute the gradient directly from f:
    gfx=gradient(f,x)
    print("gfx:",gfx,type(gfx))

    #! Extract the partial derivatve df/dx0.
    #! We can either extract from the differential object by saying we want to
    #! differentiate 1 time with respect to x0, and 0 times with respect to x1,
    #! or extract the 0-th component of the gradient covector
    dfx1=dfx[(1,0)]
    dfx1=gfx[0]

    #! Extract the Hessian of f from the differential:
    hfx=dfx.hessian()
    print("hfx:",hfx,type(hfx))

    #! Extract the partial derivatve ddf/dx0dx0.
    #! We can either extract from the differential object by saying we want to
    #! differentiate 2 times with respect to x0, and 0 times with respect to x1,
    #! or extract the (0,0)-th component of the hessian symmetric matrix
    #! Note that due to the scaling of the Differential class, we need to divide the raw element by 2!*0!.
    ddfx0x0=dfx[(2,0)]/2
    ddfx0x0=hfx[0,0]

    #! Extract the partial derivatve ddf/dx0dx1.
    #! We can either extract from the differential object by saying we want to
    #! differentiate 1 time with respect to x0 and with respect to x1,
    #! or extract the (0,1)-th or (1,0)-component of the gradient symmetric matrix
    #! Due to the scaling we need to divide the raw element by 1!*1!, but this is 1 in this case.
    ddfx0x1=dfx[(1,1)]
    ddfx0x1=hfx[0,1]
    ddfx0x1=hfx[1,0]

    #! Extract the partial derivatve dddf/dx0dx0dx0.
    dddfx0x0x0=dfx[(3,0)]/6
    #! Extract the partial derivatve ddf/dx0dx0dx1.
    dddfx0x0x1=dfx[(2,1)]/2

    return

    #! We can also differential the entire function \f$f\f$ with respect to one of the argument variables.
    #! \paragraph unimplemented_composed_function_derivative_note Note: The derivative is not currently implemented for composed functions.
    df0=derivative(f,0)
    #! Evaluate the derivative function at x.
    df0x=df0(x)
    #! Compare results
    print("dfx0:",dfx0)
    print("df0x:",df0x)


def compute_bounds(f):

    # Compute over-approximation to range of possible values over box [{0:3},{1:2}]
    dom=BoxDomainType([{0:3},{1:2}])
    print("dom:",dom,type(dom))
    rng=image(dom,f)
    print("rng:",rng,type(rng))

    (dom0,dom1)=dom.split()
    print(dom0,dom1)
    print("image(",dom,",f): ",image(dom,f),sep='')
    print("  image(",dom0,",f): ",image(dom0,f),sep='')
    (dom00,dom01)=dom0.split()
    print("    image(",dom00,",f): ",image(dom00,f),sep='')
    print("    image(",dom01,",f): ",image(dom01,f),sep='')
    print("  image(",dom1,",f): ",image(dom1,f),sep='')

    expected=IntervalDomainType([dy_(0.25),dy_(0.4375)])
    img=ValidatedConstrainedImageSet(dom,EffectiveVectorMultivariateFunction([f]))
    print("subset?:",img.inside(BoxDomainType([expected])))


if __name__=='__main__':
    f=create_neural_network()
    approximate_function(f)
    compute_derivatives(f)
    compute_bounds(f)
