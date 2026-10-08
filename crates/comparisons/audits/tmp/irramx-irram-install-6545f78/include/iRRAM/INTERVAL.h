/*

iRRAM_INTERVAL.h -- header file for INTERVAL class of the iRRAM library
 
Copyright (C) 2003-2004 Norbert Mueller
 
This file is part of the iRRAM Library.
 
The iRRAM Library is free software; you can redistribute it and/or modify
it under the terms of the GNU Library General Public License as published by
the Free Software Foundation; either version 2 of the License, or (at your
option) any later version.
 
The iRRAM Library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library General Public
License for more details.
 
You should have received a copy of the GNU Library General Public License
along with the iRRAM Library; see the file COPYING.LIB.  If not, write to
the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
MA 02111-1307, USA. 
*/

/****************************************************************************/
// The syntax of the functions is chosen similar to 
//    SUN C++ Interval Arithmetic Programming Reference
// The semantics must of course is according to the neccessities of TTE
// Not all functions have been implemented!
/****************************************************************************/

#ifndef iRRAM_INTERVAL_H
#define iRRAM_INTERVAL_H

#include <iRRAM/REAL.h>

namespace iRRAM {

/*! \ingroup types */
class INTERVAL 
{
public:

// Constructors: -------------------------------

// value: point interval [0,0]
INTERVAL();
INTERVAL(const INTERVAL &x) = default;
INTERVAL(INTERVAL &&x) noexcept = default;

// value: point interval [x,x]
INTERVAL(const REAL& x) : low(x), upp(x) {}

// value: interval [ min(x,y), max(x,y) ]
INTERVAL(const REAL& x, const REAL& y);

//  Fast construction of an interval from an ordered(!) pair of reals
INTERVAL(REAL l, REAL r, bool) : low(std::move(l)), upp(std::move(r)) {}


// Standard Arithmetic: ------------------------
INTERVAL & operator=(const INTERVAL &x) = default;
INTERVAL & operator=(INTERVAL &&x) noexcept = default;

friend INTERVAL  operator +  (      INTERVAL  x, const INTERVAL& y) { x += y; return x; }
friend INTERVAL& operator += (      INTERVAL& x, const INTERVAL& y);

friend INTERVAL  operator -  (      INTERVAL  x, const INTERVAL& y) { x -= y; return x; }
friend INTERVAL& operator -= (      INTERVAL& x, const INTERVAL& y);
friend INTERVAL  operator -  (const INTERVAL& x);

friend INTERVAL  operator *  (const INTERVAL& x, const INTERVAL& y);
friend INTERVAL& operator *= (      INTERVAL& x, const INTERVAL& y) { return x=x*y; }

friend INTERVAL  operator /  (const INTERVAL& x, const INTERVAL& y);
friend INTERVAL& operator /= (      INTERVAL& x, const INTERVAL& y) { return x=x/y; }

friend REAL wid(const INTERVAL& x);
friend REAL inf(const INTERVAL& x);
friend REAL sup(const INTERVAL& x);
friend REAL mid(const INTERVAL& x);
friend REAL mag(const INTERVAL& x);
friend REAL mig(const INTERVAL& x);

friend LAZY_BOOLEAN superset (const INTERVAL& x,
                             const INTERVAL& y);
friend LAZY_BOOLEAN proper_superset (const INTERVAL& x,
                             const INTERVAL& y);
friend LAZY_BOOLEAN subset (const INTERVAL& x,
                             const INTERVAL& y);
friend LAZY_BOOLEAN proper_subset (const INTERVAL& x,
                             const INTERVAL& y);
friend LAZY_BOOLEAN in_interior (const INTERVAL& x,
                             const INTERVAL& y);
friend LAZY_BOOLEAN disjoint (const INTERVAL& x,
                             const INTERVAL& y);
friend LAZY_BOOLEAN in (const REAL& x,
                             const INTERVAL& y);

friend INTERVAL interval_hull (const INTERVAL& x,
                             const INTERVAL& y);
friend INTERVAL intersect (const INTERVAL& x,
                             const INTERVAL& y);

friend INTERVAL fabs(const INTERVAL& x);

friend INTERVAL power(const INTERVAL& x,int n);
friend INTERVAL square(const INTERVAL& x);
friend INTERVAL exp(const INTERVAL& x);
friend INTERVAL log(const INTERVAL& x);
friend INTERVAL sin(const INTERVAL& x);
friend INTERVAL cos(const INTERVAL& x);
friend INTERVAL tan(const INTERVAL& x);
friend INTERVAL asin(const INTERVAL& x);
friend INTERVAL acos(const INTERVAL& x);
friend INTERVAL atan(const INTERVAL& x);

//private :
REAL low;
REAL upp;
};

} /* ! namespace iRRAM */

#endif
