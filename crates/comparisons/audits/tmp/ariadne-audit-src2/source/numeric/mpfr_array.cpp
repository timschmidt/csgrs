/* mpfr_array -- initialize an array of floating-point number

Modified from mpfr_init -- Copyright 1999, 2001, 2002, 2004, 2006 Free Software Foundation, Inc.

Copyright  2007-20  Pieter Collins, Maastricht University

This file is intended to become part of the MPFR Library.

The MPFR Library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or (at your
option) any later version.

The MPFR Library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
License for more details.

You should have received a copy of the GNU Lesser General Public License
along with the MPFR Library; see the file COPYING.LIB.  If not, write to
the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
MA 02110-1301, USA. */

#include "mpfr_array.hpp"

mpfr_ptr
mpfr_array_alloc_init (size_t n)
{
  return mpfr_array_alloc_init2 (n, mpfr_get_default_prec());
}

mpfr_ptr
mpfr_array_alloc_init2 (size_t n, mpfr_prec_t p)
{
  static const mpfr_exp_t exp = 0;

  const size_t element_size = mpfr_custom_get_size(p);

  mpfr_ptr a = (mpfr_ptr) malloc( n * (sizeof(mpfr_t)+element_size) );
  char* l = (char*) (a + n);

  for(size_t i=0; i!=n; ++i) {
    mpfr_ptr x=a+i;
    mp_limb_t* s = (mp_limb_t*) (l + i * element_size);
    mpfr_custom_init_set (x, MPFR_NAN_KIND, exp, p, x);
  }
  return a;
}

void
mpfr_array_clear_free (mpfr_ptr a, size_t n)
{
  free(a);
}



mpfr_ptr
mpfr_array_realloc_set_prec (mpfr_ptr a, size_t n, mpfr_prec_t p)
{
  mpfr_array_clear_free(a,n);
  return mpfr_array_alloc_init2(n,p);
}




mpfr_ptr
mpfr_array_alloc (size_t n)
{
  return (mpfr_ptr) malloc(n*sizeof(mpfr_t));
}

void
mpfr_array_init (mpfr_ptr a, size_t n)
{
  mpfr_array_init2 (a, n, mpfr_get_default_prec());
}

void
mpfr_array_init2 (mpfr_ptr a, size_t n, mp_prec_t p)
{
  static const mpfr_exp_t exp = 0;
  const size_t element_size = mpfr_custom_get_size(p);

  char* l = (char*) malloc(n*element_size);
  for(size_t i=0; i!=n; ++i) {
    mpfr_ptr x=a+i;
    mp_limb_t* s = (mp_limb_t*) (l+i*element_size);
    mpfr_custom_init_set (x, MPFR_NAN_KIND, exp, p, s);
  }
}

void
mpfr_array_clear (mpfr_ptr a, size_t n)
{
  free(a->_mpfr_d);
  for(size_t i=0u; i!=n; ++i) {
    (a+i)->_mpfr_d = nullptr;
  }
}

void
mpfr_array_free (mpfr_ptr a, size_t n)
{
  free(a);
}


mpfr_prec_t
mpfr_array_get_prec (mpfr_t a, size_t n)
{
  return mpfr_get_prec(a);
}

void
mpfr_array_set_prec (mpfr_ptr a, size_t n, mpfr_prec_t p)
{
  mpfr_prec_t o = mpfr_array_get_prec(a,n);

  if(mpfr_custom_get_size(o) != mpfr_custom_get_size(p)) {
    free(a->_mpfr_d);
    mpfr_array_init2(a,n,p);
  }
}




void
mpfr_array_init_zero (mpfr_ptr a, size_t n)
{
  mpfr_array_init(a,n);
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_si(a+i,0,GMP_RNDN);
  }
}


void
mpfr_array_init_zero2 (mpfr_ptr a, size_t n, mpfr_prec_t p)
{
  mpfr_array_init2(a,n,p);
  for(size_t i=0; i!=n; ++i) {
      mpfr_set_si(a+i,0,GMP_RNDN);
  }
}



void
mpfr_array_copy (mpfr_ptr a, size_t n, mpfr_srcptr x, mp_rnd_t rnd_mode)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set(a+i,x+i,rnd_mode);
  }
}


void
mpfr_array_set (mpfr_ptr a, size_t n, mpfr_t x, mp_rnd_t rnd_mode)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set(a+i,x,rnd_mode);
  }
}



void
mpfr_array_set_ui (mpfr_ptr rop, size_t n, unsigned long int op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_ui(rop+i,op,rnd);
  }
}

void
mpfr_array_set_si (mpfr_ptr rop, size_t n, long int op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i)  {
    mpfr_set_si(rop+i,op,rnd);
  }
}


#ifdef _MPFR_H_HAVE_INTMAX_T

void mpfr_array_set_uj (mpfr_ptr rop, size_t n, uintmax_t op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_uj(rop+i,op,rnd);
  }
}

void mpfr_array_set_sj (mpfr_ptr rop, size_t n, intmax_t op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_sj(rop+i,op,rnd);
  }
}

#endif


void mpfr_array_set_d (mpfr_ptr rop, size_t n, double op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_d(rop+i,op,rnd);
  }
}


void mpfr_array_set_ld (mpfr_t rop, size_t n, long double op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_ld(rop+i,op,rnd);
  }
}


void mpfr_array_set_z (mpfr_t rop, size_t n, mpz_t op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_z(rop+i,op,rnd);
  }
}


void mpfr_array_set_q (mpfr_t rop, size_t n, mpq_t op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_q(rop+i,op,rnd);
  }
}


void mpfr_array_set_f (mpfr_t rop, size_t n, mpf_t op, mp_rnd_t rnd)
{
  for(size_t i=0; i!=n; ++i) {
    mpfr_set_f(rop+i,op,rnd);
  }
}



