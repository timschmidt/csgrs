#!/usr/bin/python3

##############################################################################
#            test_solvers.py
#
#  Copyright  2012-20  Pieter Collins
##############################################################################

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

from pyariadne import *

fo=EffectiveScalarMultivariateFunction(2)
fx=EffectiveScalarMultivariateFunction.coordinate(2,0)
fy=EffectiveScalarMultivariateFunction.coordinate(2,1)

def test_solvers():
    f=ValidatedVectorMultivariateFunction([fx+fy,fy])
    b=BoxDomainType([{-1:+1},{-1:+1}])

    solver=IntervalNewtonSolver(1e-8,12)
    solver.solve(f,b)

    solver=KrawczykSolver(1e-8,12)
    solver.solve(f,b)

def test_integrators():
    d=BoxDomainType([{-1:+1},{-1:+1}])
    h=Dyadic(exact(0.25))

    vf=EffectiveVectorMultivariateFunction([fo,fx])
    integrator=TaylorPicardIntegrator(1e-8)
    integrator.flow_step(vf,d,h)

    vf=ValidatedVectorMultivariateFunction([fo,fx])
    integrator=GradedTaylorSeriesIntegrator(1e-8)
    integrator.flow_step(vf,d,h)

