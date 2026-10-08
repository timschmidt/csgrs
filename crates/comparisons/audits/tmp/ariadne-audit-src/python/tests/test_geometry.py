#!/usr/bin/python3

##############################################################################
#            test_geometry.py
#
#  Copyright  2020  Pieter Collins
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

IntervalDomainType = FloatDPExactInterval
IntervalRangeType = FloatDPUpperInterval
BoxDomainType = FloatDPExactBox
BoxRangeType = FloatDPUpperBox

def test_generics():
    assert(Interval[FloatDP]==FloatDPExactInterval)
    assert(Interval[FloatDPUpperBound]==FloatDPUpperInterval)
    assert(Interval[FloatDPLowerBound]==FloatDPLowerInterval)
    assert(Interval[FloatDPApproximation]==FloatDPApproximateInterval)

    assert(Box[FloatDP]==FloatDPExactBox)
    assert(Box[FloatDPUpperBound]==FloatDPUpperBox)
    assert(Box[FloatDPLowerBound]==FloatDPLowerBox)
    assert(Box[FloatDPApproximation]==FloatDPApproximateBox)

def test_interval():

    dp = DoublePrecision()
    mp = MultiplePrecision(64)

    xd=ExactDouble(0)
    w=Dyadic(0)

    wivl=DyadicInterval(-w,w)
    rivl=RealInterval(wivl)

    x=FloatDP(0,dp)
    u=FloatDPUpperBound(0,dp)
    l=FloatDPLowerBound(0,dp)
    a=FloatDPApproximation(0,dp)

    xivl=FloatDPExactInterval({-xd:xd})
    xivl=FloatDPExactInterval(-x,x)
    uivl=FloatDPUpperInterval(-u,u)
    livl=FloatDPLowerInterval(-l,l)
    aivl=FloatDPApproximateInterval(-a,a)

    xivl=FloatDPExactInterval(wivl)
    uivl=FloatDPUpperInterval(rivl)

    xivl.lower_bound()
    xivl.upper_bound()
    xivl.midpoint()
#    xivl.radius()
#    xivl.width()
    xivl.contains(x)
    xivl.empty()

    contains(xivl,x)
    disjoint(xivl,xivl)
    subset(xivl,xivl)
    intersection(xivl,xivl)
    hull(xivl,xivl)
    (xivl,xivl)=split(xivl)

    assert(type(subset(xivl,xivl))==Boolean)
    assert(type(subset(livl,uivl))==ValidatedUpperKleenean)
    assert(type(subset(uivl,livl))==ValidatedLowerKleenean)
    assert(type(subset(aivl,aivl))==ApproximateKleenean)

    assert(IntervalDomainType==FloatDPExactInterval)
    assert(IntervalValidatedRangeType==FloatDPUpperInterval)
    assert(IntervalApproximateRangeType==FloatDPApproximateInterval)


def test_box():
    dp = DoublePrecision()
    mp = MultiplePrecision(64)

    ad=ApproximateDouble(0)
    xd=ExactDouble(0)
    w=Dyadic(0)

    wbx=DyadicBox([{-xd:xd},{-xd:xd}])
    wbx=DyadicBox([(-xd,xd),(-xd,xd)])
    wbx=DyadicBox([(-w,w),(-w,w)])
    rbx=RealBox(wbx)

    x=FloatDP(0,dp)
    u=FloatDPUpperBound(0,dp)
    l=FloatDPLowerBound(0,dp)
    a=FloatDPApproximation(0,dp)

    xbx=FloatDPExactBox([{-xd:xd},{-xd:xd}])
    ubx=FloatDPUpperBox([{-xd:xd},{-xd:xd}])
    lbx=FloatDPLowerBox([{-xd:xd},{-xd:xd}])
    abx=FloatDPApproximateBox([{-xd:xd},{-xd:xd}])
#    abx=FloatDPApproximateBox([{-ad:ad},{-ad:ad}])

    xbx=FloatDPExactBox([(-x,x),(-x,x)])
    ubx=FloatDPUpperBox([(-u,u),(-u,u)])
    lbx=FloatDPLowerBox([(-l,l),(-l,l)])
    abx=FloatDPApproximateBox([(-a,a),(-a,a)])

    xbx=FloatDPExactBox(wbx)
    ubx=FloatDPUpperBox(rbx)

    xpt=FloatDPPoint([x,x])
    xivl=FloatDPExactInterval(-x,x)

    xbx.dimension()
    xbx.centre()
#    xbx.radius()
    xbx.contains(xpt)
    xbx.is_empty()

    xbx.separated(xbx)
    xbx.overlaps(xbx)
    xbx.covers(xbx)
    xbx.inside(xbx)
    (xbx,xbx)=xbx.split()
    (xbx,xbx)=xbx.split(0)

    contains(xbx,xpt)
    disjoint(xbx,xbx)
    subset(xbx,xbx)
    product(xbx,xivl)
    product(xbx,xbx)
    intersection(xbx,xbx)
    hull(xbx,xbx)
    (xbx,xbx)=split(xbx)

    assert(type(subset(xbx,xbx))==Boolean)
    assert(type(subset(lbx,ubx))==ValidatedUpperKleenean)
    assert(type(subset(ubx,lbx))==ValidatedLowerKleenean)
    assert(type(subset(abx,abx))==ApproximateKleenean)

    assert(BoxDomainType==FloatDPExactBox)
    assert(BoxValidatedRangeType==FloatDPUpperBox)
    assert(BoxApproximateRangeType==FloatDPApproximateBox)


def test_geometry():
    test_interval()
    test_box()



