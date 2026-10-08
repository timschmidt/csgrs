/*

iRRAM_SPARSEREALMATRIX.h -- header file for the SPARSEREALMATRIX class 
                                                  of the iRRAM library
 
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

#ifndef iRRAM_SPARSEREALMATRIX_H
#define iRRAM_SPARSEREALMATRIX_H

#include <iRRAM/REAL.h>

namespace iRRAM {

class SPM_ELEMENT
{ public:
REAL value;
unsigned int rowindex;
unsigned int colindex;
SPM_ELEMENT* nextcol;
SPM_ELEMENT* nextrow; 
};


/*! \ingroup types */
class SPARSEREALMATRIX
{
public:
// Constructors: -------------------------------

SPARSEREALMATRIX(unsigned int rows,unsigned int columns);
SPARSEREALMATRIX();
SPARSEREALMATRIX(const SPARSEREALMATRIX& y);
// Copy Constructor: ---------------------------

SPARSEREALMATRIX&   operator = (const SPARSEREALMATRIX& y);

// Destructor: ---------------------------------

~SPARSEREALMATRIX();
void clear();
void free();

// Access to matrix elements: ------------------

friend void sparse_set(SPARSEREALMATRIX& m,unsigned int i,unsigned int j,const REAL& x);
friend void sparse_unset(SPARSEREALMATRIX& m,unsigned int i,unsigned int j);
friend SPM_ELEMENT* sparse_get_ptr(SPARSEREALMATRIX& m,unsigned int i,unsigned int j);

REAL    operator ()           (unsigned int row,
                               unsigned int column);


// Standard Arithmetic: ------------------------

friend SPARSEREALMATRIX  operator + (const SPARSEREALMATRIX& x,
                               const SPARSEREALMATRIX& y);
friend SPARSEREALMATRIX  operator - (const SPARSEREALMATRIX& x,
                               const SPARSEREALMATRIX& y);
friend SPARSEREALMATRIX  operator * (const SPARSEREALMATRIX& x,
                               const SPARSEREALMATRIX& y);
friend SPARSEREALMATRIX  operator / (const SPARSEREALMATRIX& x,
                               const SPARSEREALMATRIX& y);

// Arithmetic with Scalar: ---------------------
friend SPARSEREALMATRIX  operator * (const SPARSEREALMATRIX& x,
                               const REAL& y);
inline friend SPARSEREALMATRIX operator * (const REAL& y,
                               const SPARSEREALMATRIX& x)
       {return x*y;};

friend SPARSEREALMATRIX  operator / (const SPARSEREALMATRIX& x,
                               const REAL& y);

// Information on Dimensions: ------------------

friend inline unsigned int rows (const SPARSEREALMATRIX& x)
                {return x.maxrow;};
friend inline unsigned int columns (const SPARSEREALMATRIX& x)
                {return x.maxcolumn;};

friend int bound (const SPARSEREALMATRIX& x, 
                  const int k);

// Linear Algebra: --------------------------

friend SPARSEREALMATRIX sparse_eye     (unsigned int rows);
friend SPARSEREALMATRIX sparse_zeroes  (unsigned int rows,
                           unsigned int columns);
friend SPARSEREALMATRIX sparse_ones    (unsigned int rows,
                           unsigned int columns);
friend REAL       maxnorm (const SPARSEREALMATRIX& x);
friend REAL       rowsumnorm (const SPARSEREALMATRIX& x);
friend REAL       colsumnorm (const SPARSEREALMATRIX& x);

friend SPARSEREALMATRIX solve (
       SPARSEREALMATRIX& lside,
       SPARSEREALMATRIX& rside,
       int use_pivot);

// Limits: --------------------------

friend SPARSEREALMATRIX limit_lip (SPARSEREALMATRIX f(int, const SPARSEREALMATRIX&),
                           int lip,
                           const SPARSEREALMATRIX& x);

//private:
SPM_ELEMENT**  rowvector; 
SPM_ELEMENT**  colvector; 
SPM_ELEMENT* hotspot;
unsigned int    maxrow,maxcolumn,filled;
void adderror (sizetype error);
void seterror (sizetype error);
void geterror (sizetype& error) const;
sizetype geterror() const { sizetype r; geterror(r); return r; }
};

/*! \relates SPARSEREALMATRIX */
inline sizetype geterror(const SPARSEREALMATRIX &r) { return r.geterror(); }
/*! \relates SPARSEREALMATRIX */
inline void     seterror(SPARSEREALMATRIX &r, const sizetype &err) { r.seterror(err); }

} // namespace iRRAM

#endif
