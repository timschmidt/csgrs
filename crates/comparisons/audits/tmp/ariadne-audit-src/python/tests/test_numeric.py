#!/usr/bin/python3

##############################################################################
#            test_numeric.py
#
#  Copyright  2007-20  Pieter Collins
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

def test_generics():
    assert(DP==DoublePrecision)
    assert(MP==MultiplePrecision)
    assert(Float[DP]==FloatDP)
    assert(Float[MP]==FloatMP)
    assert(Ball[FloatDP]==FloatDPBall)
    assert(Ball[FloatMP]==FloatMPBall)
    assert(Bounds[FloatDP]==FloatDPBounds)
    assert(Bounds[FloatMP]==FloatMPBounds)
    assert(UpperBound[FloatDP]==FloatDPUpperBound)
    assert(UpperBound[FloatMP]==FloatMPUpperBound)
    assert(LowerBound[FloatDP]==FloatDPLowerBound)
    assert(LowerBound[FloatMP]==FloatMPLowerBound)
    assert(Approximation[FloatDP]==FloatDPApproximation)
    assert(Approximation[FloatMP]==FloatMPApproximation)
    assert(Ball[FloatDP,FloatDP]==FloatDPBall)
    assert(Ball[FloatMP,FloatDP]==FloatMPDPBall)
    assert(Ball[FloatMP,FloatMP]==FloatMPBall)

def test_regression():
    # New regression tests
    dp = DoublePrecision()
    mp = MultiplePrecision(52)
    # Questions: Should the following conversions from double be allowed?
    ExactDouble(1.3) # Can we construct ExactDouble when input probably not exact?
    Dyadic(exact(1.375))
    FloatDP("1.375",dp)
    FloatMP("1.375",mp)

    z=Integer(1)
    w=Dyadic(1)
    q=Rational(1)

    # Check application of overloaded operators, especially on builtin types, returns the correct value
    assert(type(hlf(1))==Dyadic)
    print("type(rec(1)):",type(rec(1)))
    print("type(rec(z)):",type(rec(z)))

    assert(type(rec(1))==Rational)
    assert(type(rec(z))==Rational)
    assert(type(rec(z))==Rational)
    print("type(exp(1)):",type(exp(1)))
    assert(type(exp(1))==Real)
    assert(type(exp(z))==Real)

    ValidatedNumber({1:2})

    0*Dyadic(0)

    # Check construction from builtin integers
    assert(type(FloatDPBounds(1,2,dp))==FloatDPBounds);

    # Check construction from builtin floats
    assert(type(FloatDPBounds(1,2,dp))==FloatDPBounds);

    assert(type(FloatDPBounds({1:2},dp))==FloatDPBounds);


def check_directed_arithmetic(x1,x2,r):
    nx2=-x2;
    assert(type(-nx2)==type(x1));
    assert(type(x1+x2)==type(r));
    assert(type(x1-nx2)==type(r));
#    assert(type(exp(x1))==type(r));
#    assert(type(log(x1))==type(r));
    assert(type(atan(x1))==type(r));
    assert(type(max(x1,x2))==type(r))
    assert(type(min(x1,x2))==type(r))
    assert(type(max(x2,x1))==type(r))
    assert(type(min(x2,x1))==type(r))


def check_arithmetic(x1,x2,r,xr=None,qr=None):
    if qr==None: qr=r;
    if xr==None: xr=r;
#    assert(type(+x1)==type(xr));
#    assert(type(-x1)==type(xr));
    assert(type(x1+x2)==type(r));
    assert(type(x1-x2)==type(r));
    assert(type(x1*x2)==type(r));
    assert(type(x1/x2)==type(qr));
    assert(type(x2+x1)==type(r));
    assert(type(x2-x1)==type(r));
    assert(type(x2*x1)==type(r));
    assert(type(x2/x1)==type(qr));
    assert(type(max(x1,x2))==type(xr))
    assert(type(min(x1,x2))==type(xr))
    assert(type(max(x2,x1))==type(xr))
    assert(type(min(x2,x1))==type(xr))

def check_directed_arithmetic(x1,x2,r,xr=None,qr=None):
    if qr==None: qr=r;
    if xr==None: xr=r;
    nx1=-x1
    nx2=-x2
    assert(type(-nx1)==type(x1));
    assert(type(-nx2)==type(x2));
    assert(type(x1+x2)==type(r));
    assert(type(x1-nx2)==type(r));
    assert(type(x2+x1)==type(r));
    assert(type(x2-nx1)==type(r));
    assert(type(max(x1,x2))==type(xr))
    assert(type(min(x1,x2))==type(xr))
    assert(type(max(x2,x1))==type(xr))
    assert(type(min(x2,x1))==type(xr))

def check_elementary(x,r,xr=None,qr=None):
    if qr==None: qr=r;
    if xr==None: xr=r;
#    assert(type(add(x,x))==type(r));
#    assert(type(sub(x,x))==type(r));
#    assert(type(mul(x,x))==type(r));
#    assert(type(div(x,x))==type(r));
    assert(type(x+x)==type(r));
    assert(type(x-x)==type(r));
    assert(type(x*x)==type(r));
    assert(type(x/x)==type(qr));
    assert(type(pow(x,-3))==type(r));
    assert(type(neg(x))==type(xr));
    assert(type(hlf(x))==type(xr));
    assert(type(sqr(x))==type(r));
    assert(type(rec(x))==type(r));
    assert(type(abs(x))==type(xr))
    assert(type(max(x,x))==type(xr))
    assert(type(min(x,x))==type(xr))

def check_directed_elementary(x,r,xr=None,qr=None):
    if qr==None: qr=r;
    if xr==None: xr=r;
    nx=-x;
    assert(type(-nx)==type(x));
    assert(type(x+x)==type(r));
    assert(type(x-nx)==type(r));
#    assert(type(hlf(x))==type(xr));
#    assert(type(sqr(x))==type(r));
#    assert(type(rec(x))==type(r));
#    assert(type(abs(x))==type(xr))
    assert(type(atan(x))==type(r));
    assert(type(max(x,x))==type(xr))
    assert(type(min(x,x))==type(xr))

def check_constructible_from(cls,val):
    cls(val)

def check_not_constructible_from(cls,val):
    try:
        cls(val)
        assert(False)
    except (TypeError):
        pass


def test_algebraic():
    n=1
    x=ExactDouble(1.0)
    z=Integer(n)
    w=Dyadic(z)
    d=Decimal(w)
    q=Rational(d)
    r=Real(q)

    assert(type(exact(0.0))==ExactDouble)

    check_not_constructible_from(Integer,0.0)
    check_not_constructible_from(Dyadic,0.0)
    check_not_constructible_from(Rational,0.0)
    check_not_constructible_from(Real,0.0)

    check_constructible_from(Decimal,0.0)

    check_arithmetic(z,n,r=z,qr=q)
    check_arithmetic(w,n,r=w,qr=q)
    check_arithmetic(q,n,q)
    check_arithmetic(r,n,r)

    check_arithmetic(z,z,r=z,qr=q)
    check_arithmetic(z,w,r=w,qr=q)
    check_arithmetic(z,q,q)
    check_arithmetic(z,r,r)
    check_arithmetic(w,w,r=w,qr=q)
    check_arithmetic(w,q,q)
    check_arithmetic(w,r,r)
    check_arithmetic(q,q,q)
    check_arithmetic(q,r,r)
    check_arithmetic(r,r,r)


def check_rounded(x):
    add(up,x,x); add(down,x,x); add(near,x,x);
    sub(up,x,x); sub(down,x,x); sub(near,x,x);
    mul(up,x,x); mul(down,x,x); mul(near,x,x);
    div(up,x,x); div(down,x,x); div(near,x,x);

    nul(up,x); nul(down,x); nul(near,x);
    pos(up,x); pos(down,x); pos(near,x);
    neg(up,x); neg(down,x); neg(near,x);
    hlf(up,x); hlf(down,x); hlf(near,x);
    sqr(up,x); sqr(down,x); sqr(near,x);
    rec(up,x); rec(down,x); rec(near,x);
    fma(up,x,x,x); fma(down,x,x,x); fma(near,x,x,x);
    pow(up,x,1); pow(down,x,1); pow(near,x,1);
    sqrt(up,x); sqrt(down,x); sqrt(near,x);
    exp(up,x); exp(down,x); exp(near,x);
    log(up,x); log(down,x); log(near,x);
    sin(up,x); sin(down,x); sin(near,x);
    cos(up,x); cos(down,x); cos(near,x);
    tan(up,x); tan(down,x); tan(near,x);
    atan(up,x); atan(down,x); atan(near,x);

    nul(x); pos(x); neg(x); hlf(x);
    abs(x); max(x,x); min(x,x);
    x==x; x!=x; x< x; x> x; x<=x; x>=x;


def test_rounded():
    w=Dyadic(3,1); q=Rational(1,3);
    dp=DoublePrecision(); RoundedFloatDP(w,dp); RoundedFloatDP(q,dp); 
    mp=MultiplePrecision(128); RoundedFloatMP(w,mp); RoundedFloatMP(q,mp); 
    dp=DoublePrecision(); FloatDP(w,dp); FloatDP(q,up,dp); FloatDP.eps(dp);
    mp=MultiplePrecision(128); FloatMP(w,mp); FloatMP(q,up,mp); FloatMP.eps(mp);
    check_rounded(FloatDP(w,dp))
    check_rounded(FloatMP(w,mp))


def test_concrete():
    n=2
    d=2.125
    xd=exact(d)
    z=Integer(3)
    w=Dyadic(9,2)
    q=Rational(9,4)
    r=Real(q)

    dp=DoublePrecision()
    mp=MultiplePrecision(64)
    ax=FloatDPApproximation(2.25,dp)
    bx=FloatDPBounds(dp)
    bx=FloatDPBounds(5,dp)
    bx=FloatDPBounds(ExactDouble(2.25),dp)
    bx=FloatDPBounds(exact(2.25),dp)
    bx=FloatDPBounds(exact(2.00),exact(2.25),dp)
#    bx=FloatDPBounds({exact(2.00):exact(2.25)},dp)

    vy=ValidatedNumber(0)
    uy=ValidatedUpperNumber(0)
    ly=ValidatedLowerNumber(0)
    ay=ApproximateNumber(0)

    ax=FloatDPApproximation(xd,dp)
    lx=FloatDPLowerBound(xd,dp)
    ux=FloatDPUpperBound(xd,dp)
    bx=FloatDPBounds(xd,dp)
    mx=FloatDPBall(xd,dp)
    vx=FloatDP(xd,dp)

    check_elementary(vx,r=bx,xr=vx)
    check_elementary(mx,r=mx)
    check_elementary(bx,r=bx)
    check_directed_elementary(ux,r=ux)
    check_directed_elementary(lx,r=lx)
    check_elementary(ax,r=ax)

    check_arithmetic(mx,bx,r=bx)
    check_arithmetic(bx,mx,r=bx)

    check_arithmetic(vx,ax,r=ax)
    check_arithmetic(ax,vx,r=ax)
    check_arithmetic(mx,ax,r=ax)
    check_arithmetic(ax,mx,r=ax)
    check_directed_arithmetic(bx,ux,r=ux)
    check_directed_arithmetic(ux,bx,r=ux)
    check_directed_arithmetic(bx,lx,r=lx)
    check_directed_arithmetic(lx,bx,r=lx)
    check_arithmetic(bx,ax,r=ax)
    check_arithmetic(ax,bx,r=ax)


    check_arithmetic(vx,n,r=bx,xr=vx)
    check_arithmetic(vx,w,r=bx,xr=vx)
    check_arithmetic(mx,n,r=mx)
    check_arithmetic(mx,w,r=mx)
    check_arithmetic(mx,vy,r=mx)
    check_arithmetic(bx,n,r=bx,xr=bx)
    check_arithmetic(bx,vy,r=bx)
    check_directed_arithmetic(ux,uy,r=ux)
    check_directed_arithmetic(lx,ly,r=lx)
    check_arithmetic(ax,n,r=ax)
    check_arithmetic(ax,d,r=ax)
    check_arithmetic(ax,vy,r=ax)
    check_arithmetic(ax,ay,r=ax)

    mx=FloatMPDPBall(xd,mp)
    check_arithmetic(mx,mx,r=mx)
    check_arithmetic(mx,n,r=mx)
    check_arithmetic(mx,w,r=mx)
