/* mpfr_array -- initialize an array of floating-point number

Modified from mpfr_init -- Copyright 1999, 2001, 2002, 2004, 2006 Free Software Foundation, Inc.

Copyright  2007-20  Pieter Collins, Maastricht University

This file is intended to become part of the MPFR Library.

The MPFR Library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or (at your
option); any later version.

The MPFR Library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
License for more details.

You should have received a copy of the GNU Lesser General Public License
along with the MPFR Library; see the file COPYING.LIB.  If not, write to
the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
MA 02110-1301, USA.
*/

#include <cstddef>
#include <cstdlib>
#include <mpfr.h>

/*! \defgroup mpfr_array MPFR Array
 *  \ingroup ExternalModules
 *  \brief Arrays of mpfr objects of the same precision.
 */
/**@{*/
/*! \brief Allocate an array of \a n mpfr objects and their limbs with default precision, and initialize to NaN. */
mpfr_ptr mpfr_array_alloc_init (size_t n);
/*! \brief Allocate an array of \a n mpfr objects and their limbs with precision \a p, and initialize to NaN.
 *  Note that this function allocates array and limbs in a single memory block, so is more efficient (but less flexible)
 *  then mpfr_array_alloc(n) followed by mpfr_array_init2(a,n,p). */
mpfr_ptr mpfr_array_alloc_init2 (size_t n, mpfr_prec_t p);
/*! \brief Clear limbs and deallocate the array \a a of \a n mpfr objects created with \c mpfr_array_alloc_init. */
void mpfr_array_clear_free (mpfr_ptr a, size_t n);
/*! \brief Reallocate the array \a a of \a n mpfr objects to have precision \a p. */
mpfr_ptr mpfr_array_realloc_set_prec (mpfr_ptr a, size_t n, mpfr_prec_t p);

/*! \brief Allocate an array of \a n mpfr objects. */
mpfr_ptr mpfr_array_alloc (size_t n);
/*! \brief Allocate the limbs of the array \a a of \a n mpfr objects with default precision and set to NaN. */
void mpfr_array_init (mpfr_ptr a, size_t n);
/*! \brief Allocate the limbs of the array \a a of \a n mpfr objectswith precision \a p and set to NaN. */
void mpfr_array_init2 (mpfr_ptr a, size_t n, mp_prec_t p);
/*! \brief Allocate the limbs of the array \a a of \a n mpfr objects with default precision and set to 0. */
void mpfr_array_init_zero (mpfr_ptr a, size_t n);
/*! \brief Allocate the limbs of the array \a a of \a n mpfr objectswith precision \a p and set to 0. */
void mpfr_array_init_zero2 (mpfr_ptr a, size_t n, mpfr_prec_t p);
/*! \brief Deallocate the limbs of the array \a a of \a n mpfr objects. */
void mpfr_array_clear (mpfr_ptr a, size_t n);
/*! \brief Free the array \a a of \a n mpfr objects, for which the limbs have already by deallocated. */
void mpfr_array_free (mpfr_ptr a, size_t n);

/*! \brief Get the precision of the mpfr array \a a of size \a n, which is equal to the precision of the first element. */
mpfr_prec_t mpfr_array_get_prec (mpfr_t a, size_t n);
/*! \brief Set the precision of the mpfr array \a a of size \a n to \a p, reallocating the array and destroying the original values if the precision changes. */
void mpfr_array_set_prec (mpfr_ptr a, size_t n, mpfr_prec_t p);

/*! \brief Set the elements of the mpfr array \a a of size \a n to the values starting at \a x with rounding \a r. */
void mpfr_array_copy (mpfr_ptr a, size_t n, mpfr_srcptr x, mp_rnd_t r);
/*! \brief Set the elements of the mpfr array \a a of size \a n to \a x with rounding \a r. */
void mpfr_array_set (mpfr_ptr a, size_t n, mpfr_t x, mp_rnd_t r);


/* Set values of an mpfr array of size n */
void mpfr_array_set_ui (mpfr_ptr rop, size_t n, unsigned long int op, mp_rnd_t rnd);
void mpfr_array_set_si (mpfr_ptr rop, size_t n, long int op, mp_rnd_t rnd);
#ifdef _MPFR_H_HAVE_INTMAX_T
void mpfr_array_set_uj (mpfr_ptr rop, size_t n, uintmax_t op, mp_rnd_t rnd);
void mpfr_array_set_sj (mpfr_ptr rop, size_t n, intmax_t op, mp_rnd_t rnd);
#endif
void mpfr_array_set_d (mpfr_ptr rop, size_t n, double op, mp_rnd_t rnd);
void mpfr_array_set_ld (mpfr_t rop, size_t n, long double op, mp_rnd_t rnd);
void mpfr_array_set_z (mpfr_t rop, size_t n, mpz_t op, mp_rnd_t rnd);
void mpfr_array_set_q (mpfr_t rop, size_t n, mpq_t op, mp_rnd_t rnd);
void mpfr_array_set_f (mpfr_t rop, size_t n, mpf_t op, mp_rnd_t rnd);

/**@}*/
