/*

iRRAM_REALMATRIX.h -- header file for the REALMATRIX class of the iRRAM library
 
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

#ifndef iRRAM_REALMATRIX_H
#define iRRAM_REALMATRIX_H

#include <iRRAM/REAL.h>

namespace iRRAM {

class SPARSEREALMATRIX;

/*! \ingroup types */
class REALMATRIX
{
public:
// Constructors: -------------------------------

REALMATRIX(unsigned int rows,unsigned int columns);
REALMATRIX();
REALMATRIX(const REALMATRIX& y);
// Copy Constructor: ---------------------------

REALMATRIX&   operator = (const REALMATRIX& y);

// Destructor: ---------------------------------

~REALMATRIX();

// Access to matrix elements: ------------------

REAL&   operator ()           (unsigned int row,
                               unsigned int column) const;

REAL&   element               (unsigned int row,
                               unsigned int column) const;

// Standard Arithmetic: ------------------------

friend REALMATRIX  operator + (const REALMATRIX& x,
                               const REALMATRIX& y);
friend REALMATRIX  operator - (const REALMATRIX& x,
                               const REALMATRIX& y);
friend REALMATRIX  operator * (const REALMATRIX& x,
                               const REALMATRIX& y);
friend REALMATRIX  operator / (const REALMATRIX& x,
                               const REALMATRIX& y);

// Arithmetic with Scalar: ---------------------
friend REALMATRIX  operator * (const REALMATRIX& x,
                               const REAL& y);
inline friend REALMATRIX operator * (const REAL& y,
                               const REALMATRIX& x)
       {return x*y;};

friend REALMATRIX  operator / (const REALMATRIX& x,
                               const REAL& y);

// Information on Dimensions: ------------------

friend inline unsigned int rows (const REALMATRIX& x)
                {return x.maxrow;};
friend inline unsigned int columns (const REALMATRIX& x)
                {return x.maxcolumn;};

friend int bound (const REALMATRIX& x, 
                  const int k);

// Linear Algebra: -----------------------------

friend REALMATRIX eye     (unsigned int rows);
friend REALMATRIX zeroes  (unsigned int rows,
                           unsigned int columns);
friend REALMATRIX ones    (unsigned int rows,
                           unsigned int columns);
friend REAL       maxnorm (const REALMATRIX& x);
friend REAL       rowsumnorm (const REALMATRIX& x);
friend REAL       colsumnorm (const REALMATRIX& x);

friend REALMATRIX solve (
       REALMATRIX& lside,
       REALMATRIX& rside,
       int use_pivot);

// Limits: --------------------------

friend REALMATRIX limit_lip (REALMATRIX f(int, const REALMATRIX&),
                           int lip,
                           bool on_domain(const REALMATRIX&),
                           const REALMATRIX& x);


public:
REAL*  values; 
unsigned int    maxrow,maxcolumn;
void adderror (sizetype error);
void seterror (sizetype error);
void geterror (sizetype& error) const;
sizetype geterror() const { sizetype t; geterror(t); return t; }
};

/*! \relates REALMATRIX */
inline sizetype geterror(const REALMATRIX &r) { return r.geterror(); }
/*! \relates REALMATRIX */
inline void     seterror(REALMATRIX &r, const sizetype &err) { r.seterror(err); }

REALMATRIX eye     (unsigned int rows);
REALMATRIX zeroes  (unsigned int rows,
                           unsigned int columns);
REALMATRIX ones    (unsigned int rows,
                           unsigned int columns);
REAL       maxnorm (const REALMATRIX& x);
REAL       rowsumnorm (const REALMATRIX& x);
REAL       colsumnorm (const REALMATRIX& x);
REALMATRIX solve (
       REALMATRIX& lside,
       REALMATRIX& rside,
       int use_pivot);
REALMATRIX limit_lip (REALMATRIX f(int, const REALMATRIX&),
                           int lip,
                           bool on_domain(const REALMATRIX&),
                           const REALMATRIX& x);

/****************************************************************************/
//  matrix functions
/****************************************************************************/

/*! \addtogroup maths
 * @{ */
REALMATRIX exp    (const REALMATRIX& x);
//! @}

REALMATRIX steady_state (const REALMATRIX& x);

REALMATRIX steady_state (const SPARSEREALMATRIX& x);
REALMATRIX equilib (const SPARSEREALMATRIX& x);
void equilib_del (SPARSEREALMATRIX& x,REALMATRIX& z);

} // namespace iRRAM

#endif
