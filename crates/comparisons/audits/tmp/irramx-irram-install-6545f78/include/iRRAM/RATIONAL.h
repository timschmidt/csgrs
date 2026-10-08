/*

iRRAM_RATIONAL.h -- declaration of the interface to rational numbers for the iRRAM library
 
Copyright (C) 2005 Norbert Mueller
Copyright     2016 Franz Brausse
 
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

#ifndef iRRAM_RATIONAL_H
#define iRRAM_RATIONAL_H

#include <iRRAM/core.h>

namespace iRRAM {

/*! \ingroup types */
class RATIONAL
{
	/****** Private ******/
	MP_rat_type value;

public:

friend class INTEGER;
friend class DYADIC;
friend class REAL;

/****** Constructors ******/

RATIONAL(int i=0);
RATIONAL(double d);
RATIONAL(const char* s);
RATIONAL(const INTEGER& y);
//RATIONAL(const DYADIC& y); 
RATIONAL(const RATIONAL& y);
RATIONAL(RATIONAL &&y) : value(y.value) { y.value = nullptr; }

RATIONAL(int i, int j);
RATIONAL(const INTEGER& x, const INTEGER& y);

#if iRRAM_HAVE_GMP_C
	explicit RATIONAL(mpq_srcptr);

	explicit operator mpq_srcptr() const { return value; }
#endif

/****** Copy constructor ******/

RATIONAL & operator=(RATIONAL y);
RATIONAL & operator=(int y);

/****** Destructor ******/

~RATIONAL();

/****** Standard arithmetic ******/

friend RATIONAL  operator +  (RATIONAL  x, const RATIONAL& y) { x += y; return x; }
friend RATIONAL  operator +  (RATIONAL  x,       int       y) { x += y; return x; }
friend RATIONAL  operator +  (int       x,       RATIONAL  y) { y += x; return y; }
friend RATIONAL& operator += (RATIONAL& x, const RATIONAL& y);
friend RATIONAL& operator += (RATIONAL& x,       int       y);

friend RATIONAL  operator -  (RATIONAL  x, const RATIONAL& y) { x -= y; return x; }
friend RATIONAL  operator -  (RATIONAL  x,       int       y) { x += -y; return x; }
friend RATIONAL  operator -  (int       x,       RATIONAL  y);
friend RATIONAL& operator -= (RATIONAL& x, const RATIONAL& y);
friend RATIONAL& operator -= (RATIONAL& x,       int       y) { x += -y; return x; }

friend RATIONAL  operator -  (RATIONAL  x);

friend RATIONAL  operator *  (RATIONAL  x, const RATIONAL& y) { x *= y; return x; }
friend RATIONAL  operator *  (RATIONAL  x,       int       y) { x *= y; return x; }
friend RATIONAL  operator *  (int       x,       RATIONAL  y) { y *= x; return y; }
friend RATIONAL& operator *= (RATIONAL& x, const RATIONAL& y);
friend RATIONAL& operator *= (RATIONAL& x,       int       y);

friend RATIONAL  operator /  (RATIONAL  x, const RATIONAL& y) { x /= y; return x; }
friend RATIONAL  operator /  (RATIONAL  x, const int       y) { x /= y; return x; }
friend RATIONAL  operator /  (int       x,       RATIONAL  y);
friend RATIONAL& operator /= (RATIONAL& x, const RATIONAL& y);
friend RATIONAL& operator /= (RATIONAL& x,       int       y);

friend RATIONAL scale        (RATIONAL x, const int k);
friend RATIONAL abs          (RATIONAL x);

friend INTEGER  numerator	 (const RATIONAL& x);
friend INTEGER  denominator	 (const RATIONAL& x);
friend int 	sign	 	 (const RATIONAL& x);
friend RATIONAL power        (RATIONAL x, unsigned long n);
friend RATIONAL inv          (RATIONAL x);

/****** Comparisons ******/

friend bool operator< (const RATIONAL& x, const RATIONAL& y);
friend bool operator<=(const RATIONAL& x, const RATIONAL& y) { return !(y< x); }
friend bool operator> (const RATIONAL& x, const RATIONAL& y) { return   y< x ; }
friend bool operator>=(const RATIONAL& x, const RATIONAL& y) { return !(x< y); }
friend bool operator==(const RATIONAL& x, const RATIONAL& y);
friend bool operator!=(const RATIONAL& x, const RATIONAL& y) { return !(x==y); }

/****** String conversion ******/

friend std::string    swrite          (const RATIONAL& x);
friend std::string    swrite          (const RATIONAL& x, const int w);
};

} /* ! namespace iRRAM */

#endif
