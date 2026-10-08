/*

iRRAM_INTEGER.h -- header file for the INTEGER part of the iRRAM library
 
Copyright (C) 2001-2003 Norbert Mueller
 
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

#ifndef iRRAM_INTEGER_H
#define iRRAM_INTEGER_H

#include <string>

#include <iRRAM/core.h>

namespace iRRAM {

/*! \ingroup types */
class INTEGER 
{
	MP_int_type value;

	friend INTEGER numerator(const RATIONAL &);
	friend INTEGER denominator(const RATIONAL &);

	friend class RATIONAL;
	friend class REAL;
	friend class DYADIC;

	/* internal tag type to indicate the MP_int_type passed to INTEGER's
	 * constructor is supposed to be moved and not copied (in case MP_int_type
	 * matches one of the other conversion sources like e.g. mpz_srcptr). */
	struct move_t {};

	INTEGER(MP_int_type y, move_t);

public:

#if iRRAM_HAVE_GMP_C
	explicit INTEGER(mpz_srcptr);

	explicit operator mpz_srcptr() const { return value; }
#endif

/****** Constructors ******/

INTEGER(int i = 0);
INTEGER(const std::string &s);
INTEGER(const char* s);
INTEGER(const INTEGER& y);
INTEGER(INTEGER &&o) noexcept : value(o.value) { o.value = nullptr; }
INTEGER(double d);

/****** Copy/move assignment ******/

INTEGER& operator = (const int y);
INTEGER& operator = (const INTEGER& y);
INTEGER & operator=(INTEGER &&o) noexcept { using std::swap; swap(value, o.value); return *this; }

	/****** Destructor ******/
	
~INTEGER();

	/****** Standard arithmetic ******/

friend INTEGER 	operator +  (const INTEGER& x, const INTEGER& y);
friend INTEGER 	operator +  (const INTEGER& x, const int      y);
friend INTEGER 	operator +  (const int      x, const INTEGER& y);

friend INTEGER 	operator -  (const INTEGER& x, const INTEGER& y);
friend INTEGER 	operator -  (const INTEGER& x, const int      y);
friend INTEGER 	operator -  (const int      x, const INTEGER& y);

friend INTEGER 	operator -  (const INTEGER& x);

friend INTEGER 	operator *  (const INTEGER& x, const INTEGER& y);
friend INTEGER	operator *  (const INTEGER& x, const int      y);
friend INTEGER	operator *  (const int      x, const INTEGER& y);

friend INTEGER 	operator /  (const INTEGER& x, const INTEGER& y);
friend INTEGER 	operator /  (const INTEGER& x, const int      y);
friend INTEGER 	operator /  (const int      x, const INTEGER& y);

friend INTEGER 	operator << (const INTEGER& x, const int y);
friend INTEGER 	operator >> (const INTEGER& x, const int y);
friend INTEGER	operator %  (const INTEGER& x, const INTEGER& y);

friend INTEGER&	operator += (	   INTEGER& x, const INTEGER& y);
friend INTEGER& operator += (      INTEGER& x,       int      y);

friend INTEGER&	operator -= (	   INTEGER& x, const INTEGER& y);
friend INTEGER& operator -= (      INTEGER& x,       int      y);

friend INTEGER&	operator *= (	   INTEGER& x, const INTEGER& y);
friend INTEGER& operator *= (      INTEGER& x,       int      y);

friend INTEGER&	operator /= (	   INTEGER& x, const INTEGER& y);
friend INTEGER& operator /= (      INTEGER& x,       int      y);

friend INTEGER & operator%=(INTEGER &x, const INTEGER &y);


friend INTEGER  power(const INTEGER& x,       unsigned y);
friend INTEGER 	sqrt	(const INTEGER& x);
friend INTEGER		scale	(const INTEGER& x, const int k);
friend INTEGER		abs	(const INTEGER& x);

/****** Comparisons ******/

friend bool operator <  (const INTEGER& x, const INTEGER& y);
friend bool operator <  (const INTEGER& x, const int      y);
friend bool operator <  (const int      x, const INTEGER& y);

friend bool operator <= (const INTEGER& x, const INTEGER& y);
friend bool operator <= (const INTEGER& x, const int      y);
friend bool operator <= (const int      x, const INTEGER& y);

friend bool operator >  (const INTEGER& x, const INTEGER& y);
friend bool operator >  (const INTEGER& x, const int      y);
friend bool operator >  (const int      x, const INTEGER& y);

friend bool operator >= (const INTEGER& x, const INTEGER& y);
friend bool operator >= (const INTEGER& x, const int      y);
friend bool operator >= (const int      x, const INTEGER& y);

friend bool operator == (const INTEGER& x, const INTEGER& y);
friend bool operator == (const INTEGER& x, const int      y);
friend bool operator == (const int      x, const INTEGER& y);

friend bool operator != (const INTEGER& x, const INTEGER& y);
friend bool operator != (const INTEGER& x, const int      y);
friend bool operator != (const int      x, const INTEGER& y);
	
friend int size(const INTEGER& x);
friend int sign(const INTEGER& x);

/* conversion */
friend std::string    swrite  (const INTEGER& x, const int w);
friend std::string    swrite  (const INTEGER& x);

explicit operator int()  const ;
};

inline INTEGER::~INTEGER() { if (value) MP_int_clear(value); }

inline INTEGER::INTEGER(MP_int_type y, INTEGER::move_t) : value(y) {}

#if iRRAM_HAVE_GMP_C
inline INTEGER::INTEGER(mpz_srcptr p)
{
	MP_int_duplicate_w_init(p, value);
}
#endif

inline INTEGER::INTEGER(int i){
	MP_int_init(value);
	MP_int_to_INTEGER(i,value);
}

inline INTEGER::INTEGER(double d){
  MP_int_init(value);
  MP_double_to_INTEGER(d,value);
}

inline INTEGER::INTEGER(const INTEGER& i){
  MP_int_init(value);
  MP_int_duplicate_wo_init(i.value, value);
}


//****************************************************************************************
// Constructing INTEGER from character string
// The base is set to 10 (decimal)
//****************************************************************************************

inline INTEGER::INTEGER(const char* s){
  MP_int_init(value);
  MP_string_to_INTEGER(s,value,10);
}

inline INTEGER::INTEGER(const std::string &s) : INTEGER(s.c_str()) {}


inline INTEGER& INTEGER::operator = (const INTEGER& y){
  MP_int_duplicate_wo_init(y.value, value);
  return (*this);
}

inline INTEGER& INTEGER::operator = (const int y){
  MP_int_to_INTEGER(y,value);
  return (*this);
}


inline INTEGER scale(const INTEGER& x, const int n)
{
	MP_int_type zvalue;
	MP_int_init(zvalue);
	MP_int_shift(x.value,zvalue,n);
	return { zvalue, INTEGER::move_t{} };
}


//****************************************************************************************
// Returns the sign of INTEGER objects (-1/0/1): 
// sign(i)= -1, iff i<0,   sign(i)=1 iff i>0,  and sign(0)=0
//****************************************************************************************
inline int sign(const INTEGER& x){  return MP_int_sign(x.value); }

//****************************************************************************************
// Returns the size of INTEGER objects: The smallest i>0 such that |x| < 2^i,
// e.g. size(0)=size(1)=1;size(2)=size(3)=2,size(4)=...size(7)=3
//****************************************************************************************

inline int size(const INTEGER& x){ return MP_int_size(x.value);}

//****************************************************************************************
// Power: return = x**n
//****************************************************************************************

inline INTEGER power(const INTEGER& x, unsigned n)
{
	if (n == 0 || x == 1) return 1;
	if (n == 1)           return x;
	MP_int_type zvalue;
	MP_int_init(zvalue);
	MP_int_power_i(x.value,n,zvalue);
	return { zvalue, INTEGER::move_t{} };
}

//****************************************************************************************
// Shifter: returns x << n
// shifts n bits to the left
//****************************************************************************************

inline INTEGER operator << (const INTEGER& x, const int n){
  return scale(x,n);
}

//****************************************************************************************
// Shifter: returns x >> n
// shifts n bits to the right
//****************************************************************************************

inline INTEGER operator >> (const INTEGER& x, const int n){
  return scale(x,-n);
}




//****************************************************************************************
// Addition: returns x + y
//****************************************************************************************

inline INTEGER operator + (const INTEGER& x, const INTEGER& y){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_add(x.value,y.value,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

inline INTEGER operator + (const INTEGER& x, const int y){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  if (y<0) MP_int_sub_ui(x.value,-y,zvalue);
  else     MP_int_add_ui(x.value,y,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

inline INTEGER operator + (const int     x, const INTEGER& y) {return y+x;}

//****************************************************************************************
// Addition: returns x += y
//****************************************************************************************

inline INTEGER& operator += (INTEGER& x, const INTEGER& y){
  MP_int_add(x.value,y.value,x.value);
  return x;
}

inline INTEGER& operator += (INTEGER& x, int n)
{
	if (n<0) MP_int_sub_ui(x.value,-n,x.value);
	else     MP_int_add_ui(x.value, n,x.value);
	return x;
}

//****************************************************************************************
// Subtraction: returns x - y
//****************************************************************************************

inline INTEGER operator - (const INTEGER& x, const INTEGER& y){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_sub(x.value,y.value,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

inline INTEGER operator - (const INTEGER& x, const int y)
{
	return x + (-y);
}

inline INTEGER operator - (const int x, const INTEGER& y){
  return -(y-x);
}

//****************************************************************************************
// Subtraction: returns x -= y
//****************************************************************************************

inline INTEGER& operator -= (INTEGER& x, const INTEGER& y){
  MP_int_sub(x.value,y.value,x.value);
  return x;
}

inline INTEGER& operator -= (INTEGER& x, int n) { return x += -n; }

//****************************************************************************************
// Negative: returns -x
//****************************************************************************************

inline INTEGER operator - (const INTEGER& x){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_neg(x.value,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

//****************************************************************************************
// Multiplication: returns x * y
//****************************************************************************************

inline INTEGER operator * (const INTEGER& x, const INTEGER& y){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_mul(x.value,y.value,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

inline INTEGER operator * (const INTEGER& x, const int y){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_mul_si(x.value,y,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

inline INTEGER operator * (const int x, const INTEGER& y)
{
	return y * x;
}

//****************************************************************************************
// Multiplication: returns x *= y
//****************************************************************************************

inline INTEGER& operator *= (INTEGER& x, const INTEGER& y){
  MP_int_mul(x.value,y.value,x.value);
  return x;
}

inline INTEGER& operator *= (INTEGER& x, int n) {
  MP_int_mul_si(x.value,n,x.value);
  return x;
}

//****************************************************************************************
// Division: returns x / y
//****************************************************************************************


inline INTEGER operator / (const INTEGER& x, const INTEGER& y){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_div(x.value,y.value,zvalue);
  return { zvalue, INTEGER::move_t{} };
}


inline INTEGER operator / (const INTEGER& x, const int n){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  if(n<0){
    MP_int_div_ui(x.value,-n,zvalue);
    MP_int_neg(zvalue,zvalue);
  }
  else
    MP_int_div_ui(x.value,n,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

inline INTEGER operator / (const int      x, const INTEGER& y) {return INTEGER(x)/y;}

//****************************************************************************************
// Division: returns x /= y
//****************************************************************************************

inline INTEGER& operator /= (INTEGER& x, const INTEGER& y){
  MP_int_div(x.value,y.value,x.value);
  return x;
}

inline INTEGER& operator /= (INTEGER& x, int n)
{
	if (n<0) {
		MP_int_div_ui(x.value,-n,x.value);
		MP_int_neg(x.value,x.value);
	} else
		MP_int_div_ui(x.value,n,x.value);
	return x;
}

//****************************************************************************************
// Modulo: returns x % y
//****************************************************************************************

inline INTEGER operator % (const INTEGER& x, const INTEGER& y){
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_modulo(x.value,y.value,zvalue);
  return { zvalue, INTEGER::move_t{} };
}

inline INTEGER & operator%=(INTEGER &x, const INTEGER &y)
{
	MP_int_modulo(x.value, y.value, x.value);
	return x;
}

//****************************************************************************************
// Square-root: returns SQRT(x)
// Returns the truncated square-root of the given argument
//****************************************************************************************
/*! \ingroup maths */
inline INTEGER sqrt (const INTEGER& x)
{
	MP_int_type zvalue;
	MP_int_init(zvalue);
	MP_int_sqrt(x.value, zvalue);
	return { zvalue, INTEGER::move_t{} };
}

//****************************************************************************************
// Absolute value: |x|
// returns the positive part of the object
// 1. argument: INTEGER
// return value: INTEGER
//****************************************************************************************
/*! \ingroup maths */
inline INTEGER abs (const INTEGER& x)
{
  MP_int_type zvalue;
  MP_int_init(zvalue);
  MP_int_abs(x.value,zvalue);
  return { zvalue, INTEGER::move_t{} };
}


//****************************************************************************************
// boolean operators
// Last change: 2004-12-20
//****************************************************************************************

//****************************************************************************************
// Less than: x < y
//****************************************************************************************

inline bool operator < (const INTEGER& x, const INTEGER& y){
  return (MP_int_compare(x.value,y.value) < 0 );
}
inline bool operator <  (const INTEGER& x, const int      y){ return x<INTEGER(y) ;}
inline bool operator <  (const int      x, const INTEGER& y){ return INTEGER(x)<y ;}

inline bool operator <= (const INTEGER& x, const INTEGER& y){ return !(y<x) ;}
inline bool operator <= (const INTEGER& x, const int      y){ return !(y<x) ;}
inline bool operator <= (const int      x, const INTEGER& y){ return !(y<x) ;}

inline bool operator >  (const INTEGER& x, const INTEGER& y){ return y<x ;}
inline bool operator >  (const INTEGER& x, const int      y){ return y<x ;}
inline bool operator >  (const int      x, const INTEGER& y){ return y<x ;}

inline bool operator >= (const INTEGER& x, const INTEGER& y){ return !(x<y) ;}
inline bool operator >= (const INTEGER& x, const int      y){ return !(x<y) ;}
inline bool operator >= (const int      x, const INTEGER& y){ return !(x<y) ;}



inline bool operator == (const INTEGER& x, const INTEGER& y){
  return (MP_int_compare(x.value,y.value)==0);
}

inline bool operator == (const INTEGER& x, const int  i){ return x==INTEGER(i);}
inline bool operator == (const int      x, const INTEGER& y){ return (y==x) ;}

inline bool operator != (const INTEGER& x, const INTEGER& y){ return !(x==y) ;} 
inline bool operator != (const INTEGER& x, const int      y){ return !(x==y) ;}
inline bool operator != (const int      x, const INTEGER& y){ return !(x==y) ;}


//*****************************************************************************
// Function swrite
// output function for integer 
// 1. argument: INTEGER
// 2. argument: int integer : width
//*****************************************************************************

inline std::string swrite(const INTEGER& x, const int w){
     char* erg= MP_int_swritee(x.value,w);
     std::string result=erg;
     free(erg);
     return result;
}

//*****************************************************************************
// Function swrite
// output function for integer 
// 1. argument: INTEGER
//*****************************************************************************

inline std::string swrite(const INTEGER& x){
     char* erg= MP_int_sprintf(x.value);
     std::string result=erg;
     free(erg);
     return result;
}

// Conversion from INTEGER to smaller types
inline INTEGER::operator int() const { return MP_INTEGER_to_int(value); }

} /* ! namespace iRRAM */

#endif /* ! iRRAM_INTEGER_H */
