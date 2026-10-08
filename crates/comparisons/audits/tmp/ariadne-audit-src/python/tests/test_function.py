#!/usr/bin/python3

##############################################################################
#            test_function.py
#
#  Copyright  2009-20  Pieter Collins
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

def test_function():

    dp = DoublePrecision()
    c=Real(Decimal(1.75))

    fc=EffectiveScalarMultivariateFunction.constant(3,c)
    fx=EffectiveScalarMultivariateFunction.coordinate(3,0)
    fy=EffectiveScalarMultivariateFunction.coordinate(3,1)
    fid=EffectiveVectorMultivariateFunction.identity(3)

    sf=fx+fy
    vf=fid
    
    +sf; -sf; sf+sf; sf-sf; sf*sf; sf/sf;
    sf+c; sf-c; sf*c; sf/c;
    c+sf; c-sf; c*sf;

    +vf; -vf; vf+vf; vf-vf; sf*vf; vf*sf; vf/sf;
    c*vf; vf*c; vf/c;

    derivative(sf,0)

    va=FloatDPApproximationVector([1,2,3],dp)
    vb=FloatDPBoundsVector([1,2,3],dp)
    sf=EffectiveScalarMultivariateFunction(fc+fx*fy)
    vf=EffectiveVectorMultivariateFunction([fc,fx,fy])

    sf(va), sf(vb), evaluate(sf,va), evaluate(sf,vb)
    vf(va), vf(vb), evaluate(vf,va), evaluate(vf,vb)
    compose(sf,vf); compose(vf,vf)
    join(sf,sf); join(vf,sf); join(sf,vf); join(vf,vf)

    
