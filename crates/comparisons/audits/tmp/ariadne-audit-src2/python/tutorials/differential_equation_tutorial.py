#!/usr/bin/python3

##############################################################################
#            differential_equation_tutorial.py
#
#  Copyright  2022  Pieter Collins
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

def flow():
    #! Make a function f and compute the flow of the differential equation dx/dt=f(x)
    x=RealVariable("x")
    f=make_function([x],[-x+sin(x)])
    print("f:",f)


    domain=BoxDomainType([ (x_(0),x_(2)) ])
        #!< Specify that the bounds as written are exact double-precision numbers
    step=Dyadic(x_(1.5))
        #!< Specify that the bounds as written are exact double-precision numbers

    #! Solve to a tolerace of 10^-9; time-out after 32 steps
    tolerance=1e-9
    order=12

    integrator = TaylorSeriesIntegrator(tolerance, order)

    #! Solve for the root of g in the given region using the interval Newton method
    phi=integrator.flow_step(f,domain,step)
    print("phi=flow(f):",phi)
    print()



if __name__=='__main__':
    flow()
