#!/usr/bin/python3

##############################################################################
#            algebraic_equation_tutorial.py
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

def fixed_point():
    #! Make a function and compute its fixed points
    x=RealVariable("x")
    f=make_function([x],[x**3+x/3])
    print("f:",f)

    #! To compute a fixed-point of f, need to solve the equation f(x)-x=0
    g=make_function([x],[(x**3+x/3)-x])
    print("g:",g)

    #! Since f(1/2)=7/24<1/2 and f(1)=4/3>1, look for fixed-point in [1/2:1]
    region=BoxDomainType([ [x_(0.5),x_(1)] ])
        #!< Specify that the bounds as written are exact double-precision numbers
    region=BoxDomainType([ [pr_(0.5),pr_(1)] ])
        #!< Specify that the bounds as written can be safely interpreted as the nearest double-precision number.
    print("region:",region)

    #! Solve to a tolerace of 10^-9; time-out after 32 steps
    tolerance=1e-9
    max_steps=32

    #! Solve for the root of g in the given region using the interval Newton method
    solver=IntervalNewtonSolver(tolerance,max_steps)
    p=solver.solve(g,region)
    print("p=fix(f):",p)
    print("f(p):",f(p))
    print()

    #! Solve using the Krawczyk method
    solver=KrawczykSolver(tolerance,max_steps)
    p=solver.solve(g,region)
    print("p=fix(f):",p)
    print("f(p):",f(p))
    print("\n")


def inverse_function():
    #! Make a function and compute its inverse over a subdomain

    x=RealVariable("x")
    y=RealVariable("y")
    f=make_function([x],[x**3+x/3])

    #! A function h(x) is an inverse of f in a domain D if f(h(x))=x for all x in D.
    #! To compute the fixed-point, define the function g(x,y)=f(y)-x,
    #! and solve the implicit function problem g(x,h(x))=0.
    x=RealVariable("x")
    g=make_function([x,y],[(y**3+y/3)-x])
    print(g)


    #! Look for the inverse over the interval [0:1].
    #! Since f is increasing, f(0)=0 and f(1)=4/3>1, the inverse maps [0:1] into [0:1]
    domain=BoxDomainType([[0,1]])
    codomain=BoxDomainType([[0,1]])

    tolerance=1e-6
    max_steps=32
    solver=KrawczykSolver(tolerance,max_steps)

    #! Unfortunately, trying to compute the inverse over [0:1] using this solver does not converge,
    #! and gives an error
    try:
        invf=solver.implicit(g,domain,codomain)
        print("invf:",invf)
    except:
        print("Caught exception (method does not converge over domain ",domain,")",sep='')
    print()

    #! Instead, look for inverse over smaller domains.
    domains=[BoxDomainType([[x_(0.0),x_(0.125)]]),BoxDomainType([[x_(0.125),x_(0.375)]]),BoxDomainType([[x_(0.375),x_(1.0)]])]
    codomains=[BoxDomainType([[x_(-0.125),x_(0.5)]]),BoxDomainType([[x_(0.0),x_(0.75)]]),BoxDomainType([[x_(0.25),x_(1.25)]])]

    for i in range(3):
        invf=solver.implicit(g,domains[i],codomains[i])
        print("invf",i,": ",invf,sep='')

    print()


    #! The IntervalNewtonSolver is rather more sensitive,
    #! but we can still compute the inverse given sufficiently tight bounds
    newton_solver=IntervalNewtonSolver(tolerance,max_steps)
    domain=BoxDomainType([[dy_(0.125),dy_(0.375)]])
    codomain=BoxDomainType([[dy_(0.25),dy_(0.625)]])
    invf=newton_solver.implicit(g,domain,codomain)
    print("invf:",invf)



if __name__=='__main__':
    fixed_point()
    inverse_function()
