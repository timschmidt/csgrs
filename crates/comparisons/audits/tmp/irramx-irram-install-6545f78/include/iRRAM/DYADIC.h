/*

iRRAM_DYADIC.h -- header file for the DYADIC part of the iRRAM library
 
Copyright (C) 2001-2004 Norbert Mueller
 
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

#ifndef iRRAM_DYADIC_H
#define iRRAM_DYADIC_H

#include <iRRAM/helper-templates.hh>

namespace iRRAM {

/*! \ingroup types */
class DYADIC : internal::conditional_comparison_overloads<DYADIC>
{
public:
	static int getprec() { return state->DYADIC_precision; };

	// Constructors: -------------------------------

	DYADIC();
	DYADIC(const int i);
	DYADIC(const double y);
	DYADIC(const INTEGER & y);
	DYADIC(const DYADIC & y);
	DYADIC(      DYADIC &&y) noexcept : DYADIC(y.value) { y.value = nullptr; }

	// Copy/Move assignment: -----------------------

	DYADIC & operator=(const DYADIC & y);
	DYADIC & operator=(      DYADIC &&y) noexcept { using std::swap; swap(value, y.value); return *this; }

	// Destructor: ---------------------------------

	~DYADIC();

	// Standard Arithmetic: ------------------------

	friend DYADIC ADD (const DYADIC& x, const DYADIC& y, int p);
	friend DYADIC SUB (const DYADIC& x, const DYADIC& y, int p);
	friend DYADIC MULT(const DYADIC& x, const DYADIC& y, int p);
	friend DYADIC DIV (const DYADIC& x, const DYADIC& y, int p);

	friend DYADIC operator+(const DYADIC& x, const DYADIC& y) { return ADD(x, y, getprec()); }
	friend DYADIC operator-(const DYADIC& x, const DYADIC& y) { return SUB(x, y, getprec()); }
	friend DYADIC operator-(const DYADIC& x) { return DYADIC()-x; }
	friend DYADIC operator*(const DYADIC& x, const DYADIC& y) { return MULT(x, y, getprec()); }
	friend DYADIC operator/(const DYADIC& x, const DYADIC& y) { return DIV(x, y, getprec()); }

	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator+(const A &a, const B &b) { return a+DYADIC(b); }
	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator+(const B &b, const A &a) { return a+b; }

	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator-(const A &a, const B &b) { return a-DYADIC(b); }
	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator-(const B &b, const A &a) { return DYADIC(b)-a; }

	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator*(const A &a, const B &b) { return a*DYADIC(b); }
	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator*(const B &b, const A &a) { return a*b; }

	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator/(const A &a, const B &b) { return a/DYADIC(b); }
	template <typename A,typename B> friend internal::enable_if_compat<DYADIC,A,B> operator/(const B &b, const A &a) { return DYADIC(b)/a; }

	// Comparisons: --------------------------------

	friend bool operator<(const DYADIC& x, const DYADIC& y);
	friend bool operator>(const DYADIC& x, const DYADIC& y);
	friend bool operator<=(const DYADIC& x, const DYADIC& y);
	friend bool operator>=(const DYADIC& x, const DYADIC& y);
	friend bool operator==(const DYADIC& x, const DYADIC& y);
	friend bool operator!=(const DYADIC& x, const DYADIC& y);

	// Output: -------------------------------------

	friend std::string  swrite      (const DYADIC& x, const int w);

	// miscellaneous: ------------------------------

	friend int     size        (const DYADIC& x); 
	friend DYADIC  abs         (const DYADIC& x); 
	friend DYADIC  scale       (const DYADIC& x, const int k);

	// coexistence with other classes: -------------

	friend class REAL;
	friend class INTEGER;
	friend class RATIONAL;

	friend DYADIC  approx      (const REAL& x, const int p);

	INTEGER as_INTEGER() const;

	// implementational issues: --------------------

	MP_type    value;

private:

	DYADIC(MP_type  y);
};

/*! \ingroup switches */
class DYADIC_precision
{
	int precision;
public:
	DYADIC_precision(int p)
	{
		precision = state->DYADIC_precision;
		state->DYADIC_precision = p;
	}
	~DYADIC_precision() { state->DYADIC_precision = precision; }
};

} /* ! namespace iRRAM */

#endif
