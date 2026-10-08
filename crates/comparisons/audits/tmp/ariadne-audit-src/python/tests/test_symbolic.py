#!/usr/bin/python3

##############################################################################
#            test_symbolic.py
#
#  Copyright  2019-20  Pieter Collins
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

def test_symbolic():

    n=5
    q=Rational(1,5);
    r=Real(sin(5))
    c=RealConstant("five",5)
    x=RealVariable("x")
    y=RealVariable("y")
    e=RealExpression(0);

    x+n; x-n; x*n; x/n; max(x,n); min(x,n);
    n+x; n-x; n*x; n/x; max(n,x); min(n,x);
    x+q; x-q; x*q; x/q; max(x,q); min(x,q);
    q+x; q-x; q*x; q/x; max(q,x); min(q,x);
    x+r; x-r; x*r; x/r; max(x,r); min(x,r);
    r+x; r-x; r*x; r/x; max(r,x); min(r,x);
    x+c; x-c; x*c; x/c; max(x,c); min(x,c);
    c+x; c-x; c*x; c/x; max(c,x); min(c,x);
    x+x; x-x; x*x; x/x; max(x,x); min(x,x);

    e+n; e-n; e*n; e/n; max(e,n); min(e,n);
    n+e; n-e; n*e; n/e; max(n,e); min(n,e);
    e+q; e-q; e*q; e/q; max(e,q); min(e,q);
    q+e; q-e; q*e; q/e; max(q,e); min(q,e);
    e+r; e-r; e*r; e/r; max(e,r); min(e,r);
    r+e; r-e; r*e; r/e; max(r,e); min(r,e);
    e+c; e-c; e*c; e/c; max(e,c); min(e,c);
    c+e; c-e; c*e; c/e; max(c,e); min(c,e);
    e+x; e-x; e*x; e/x; max(e,x); min(e,x);
    x+e; x-e; x*e; x/e; max(x,e); min(x,e);
    e+e; e-e; e*e; e/e; max(e,e); min(e,e);

    +x; -x; neg(x); sqr(x); hlf(x); rec(x);
    sqrt(x); exp(x); log(x); sin(x); cos(x); tan(x); atan(x);

    +e; -e; neg(e); sqr(e); hlf(e); rec(e);
    sqrt(e); exp(e); log(e); sin(e); cos(e); tan(e); atan(e);

    derivative(e,x)

    xs=RealVariables("x",3)

    xy=RealSpace([x,y])
    ev=RealExpressionVector([n,q,r,c,x,x+x,x*x]);

    f=make_function(x,x*x)
    f=make_function([x,y],x*x+y)
    f=make_function(x,RealExpressionVector([n,q,r,c,x,x+x,x*x]))
    f=make_function(x,[n,q,r,c,x,x+x,x*x])
    f=make_function(x,[n,q,r,c,x]) # Check construction from a list of expressions without and RealExpression class
    f=make_function([x,y],[n,q,r,c,x,x+y])

def test_predicates():

    c001=Real(dec_(0.01))
    five=RealConstant("five",5)
    x=RealVariable("x")
    y=RealVariable("y")

    0<=x
    0>=x
    0<=x-y
    0>=x-y

    c001<=x
    c001>=x
    c001<=x-y
    c001>=x-y

    five<=x
    five>=x
    five<=x-y
    five>=x-y

    x<=0
    x>=0
    x<=c001
    x>=c001
    x<=five
    x>=five
    #x<=x-y
    #x>=x-y

    x-y<=0
    x-y>=0
    x-y<=c001
    x-y>=c001
    x-y<=x
    x-y>=x
    x-y<=sqr(x)
    x-y>=sqr(x)

def test_variable_interval():

    x=RealVariable("x")
    c = Real(dec_(0.1))

    [(0<=x)&(x<=1)]
    [x<<c]

if __name__  == '__main__':
    test_symbolic()
    test_predicates()
    test_variable_interval()
