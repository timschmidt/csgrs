/*

GMP_intrat.h -- extension to the GMP integer/rationals for the iRRAM library
 
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

/*
Changelog: (initial version by Tom van Diessen)  

  2001-09-10 removal of several unneeded functions by Norbert
  2004-12-20 transformation to inline by Norbert
*/

#ifndef iRRAM_GMP_INT_RAT_H
#define iRRAM_GMP_INT_RAT_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iRRAM/version.h>
#include <gmp.h>


#ifdef __cplusplus
extern "C" {
#endif

/********** caching vars **********/

#define iRRAM_MPZ_CACHE_SIZE 1000

struct iRRAM_mpz_cache_t {
	int free_var_count;
	size_t int_gmp_var_count;
	mpz_ptr free_vars[iRRAM_MPZ_CACHE_SIZE];
};

#define iRRAM_MPZ_CACHE_INIT	{ 0, 0, {0}, }

/********** initialization function **********/

static inline mpz_ptr int_gmp_init(struct iRRAM_mpz_cache_t *cache)
{
	mpz_ptr z;
	if (cache->free_var_count > 0) {
		cache->free_var_count--;
		z = cache->free_vars[cache->free_var_count];
	} else {
		z = (mpz_ptr)malloc(sizeof(mpz_t));
		mpz_init(z);
	}
	cache->int_gmp_var_count++;
	return z;
}

static inline void int_gmp_free(struct iRRAM_mpz_cache_t *cache, mpz_ptr z)
{
	if (cache->free_var_count < iRRAM_MPZ_CACHE_SIZE) {
		cache->free_vars[cache->free_var_count] = z;
		cache->free_var_count++;
	} else {
		mpz_clear(z);
		free(z);
	}
	cache->int_gmp_var_count--;
}

static inline void int_gmp_finalize(struct iRRAM_mpz_cache_t *cache)
{
	for (int i=cache->free_var_count; i; i--) {
		mpz_clear(cache->free_vars[i-1]);
		free(cache->free_vars[i-1]);
	}
	cache->free_var_count = 0;
}

/********** Conversion functions **********/

static inline void int_gmp_int2integer(const int i, mpz_t z){mpz_set_si(z,i);}
static inline void int_gmp_double2int(const double d, mpz_t z){mpz_set_d(z,d);}
static inline void int_gmp_string2int(const char* s, mpz_t z, int b){mpz_set_str(z,s,b);}
static inline int int_gmp_integer2int(const mpz_t z){return mpz_get_si(z);}

/********** standard arithmetic functions for MP integer **********/

static inline void int_gmp_add(const mpz_t z1, const mpz_t z2, mpz_t z){mpz_add(z,z1,z2);}
static inline void int_gmp_sub(const mpz_t z1, const mpz_t z2, mpz_t z){mpz_sub(z,z1,z2);}
static inline void int_gmp_mul(const mpz_t z1, const mpz_t z2, mpz_t z){mpz_mul(z,z1,z2);}
static inline void int_gmp_div(const mpz_t z1, const mpz_t z2, mpz_t z){mpz_tdiv_q(z,z1,z2);}

static inline void int_gmp_add_ui(const mpz_t z1, const unsigned int z2, mpz_t z){mpz_add_ui(z,z1,z2);}
static inline void int_gmp_sub_ui(const mpz_t z1, const unsigned int z2,mpz_t z){mpz_sub_ui(z,z1,z2);}
static inline void int_gmp_mul_si(const mpz_t z1, const int z2, mpz_t z){mpz_mul_si(z,z1,z2);}
static inline void int_gmp_div_ui(const mpz_t z1, const unsigned int z2, mpz_t z){mpz_tdiv_q_ui(z,z1,z2);}

static inline void int_gmp_abs(const mpz_t z1, mpz_t z){mpz_abs(z, z1);}
static inline void int_gmp_neg(const mpz_t z1, mpz_t z){mpz_neg(z, z1);}



/********** more MP integer functions *********/

static inline void int_gmp_root(const mpz_t z1, unsigned int z2, mpz_t z){mpz_root(z, z1, z2);}
static inline void int_gmp_power_i(const mpz_t z1, unsigned int z2, mpz_t z){mpz_pow_ui(z, z1, z2);}
static inline void int_gmp_power_ii(unsigned int z1, unsigned int z2, mpz_t z){mpz_ui_pow_ui(z, z1, z2);}
static inline void int_gmp_fac(unsigned int z1, mpz_t z){mpz_fac_ui(z, z1);}
static inline void int_gmp_modulo(const mpz_t z1, const mpz_t z2, mpz_t z){mpz_mod(z, z1, z2);}
static inline void int_gmp_sqrt(const mpz_t z1, mpz_t z){mpz_sqrt(z,z1);}
void int_gmp_shift(const mpz_t z1, mpz_t z, int p);
//int int_gmp_log(mpz_t z);


/********** output functions for MP integers **********/

//void int_gmp_writee(const mpz_t z, int w);
char* int_gmp_swritee(const mpz_t z, int w);
//void int_gmp_write(const mpz_t z, int w);
//void int_gmp_outstr(const mpz_t z, int w);
//void int_gmp_printf(const mpz_t z);
static inline char* int_gmp_sprintf(const mpz_t z){return mpz_get_str(NULL,10,z);}


/********** copying MP integers with/without initializing **********/

static inline void int_gmp_duplicate_wo_init(const mpz_t z1, mpz_t z2){mpz_set(z2,z1);}

/********* sign, size, and comparison of integer*/
static inline int int_gmp_sgn(const mpz_t z){return mpz_sgn(z);}
static inline int int_gmp_cmp(const mpz_t z1,const  mpz_t z2){return mpz_cmp(z1,z2);}
static inline int int_gmp_size(const mpz_t z){
	if (mpz_sgn(z) == 0 )  return 0; else   return mpz_sizeinbase(z,2);}

/********** counting vars **********/

#define iRRAM_MPQ_CACHE_SIZE 1000

struct iRRAM_mpq_cache_t {
	int free_var_count;
	size_t rat_gmp_var_count;
	mpq_ptr free_vars[iRRAM_MPQ_CACHE_SIZE];
};

#define iRRAM_MPQ_CACHE_INIT	{ 0, 0, {0}, }

/********** initialization function **********/

static inline mpq_ptr rat_gmp_init(struct iRRAM_mpq_cache_t *cache)
{
	mpq_ptr z;
	if (cache->free_var_count > 0) {
		cache->free_var_count--;
		z = cache->free_vars[cache->free_var_count];
	}else{
		z = (mpq_ptr)malloc(sizeof(mpq_t));
		mpq_init(z);
	}
	cache->rat_gmp_var_count++;
	return z;
}

static inline void rat_gmp_free(struct iRRAM_mpq_cache_t *cache, mpq_ptr z)
{
	if (cache->free_var_count < iRRAM_MPQ_CACHE_SIZE) {
		cache->free_vars[cache->free_var_count] = z;
		cache->free_var_count++;
	}else{
		mpq_clear(z);
		free(z);
	}
	cache->rat_gmp_var_count--;
}

static inline void rat_gmp_finalize(struct iRRAM_mpq_cache_t *cache)
{
	for (int i=cache->free_var_count; i; i--) {
		mpq_clear(cache->free_vars[i-1]);
		free(cache->free_vars[i-1]);
	}
	cache->free_var_count = 0;
}

/* canonicalize the given rational, used for used defined rationals */
static inline void rat_gmp_canon(mpq_t z){mpq_canonicalize(z);}


/********** standard arithmetic functions for MP rational **********/
/********** + - * / *********/

static inline void rat_gmp_add(const mpq_t z1, const mpq_t z2, mpq_t z){mpq_add(z,z1,z2);}
static inline void rat_gmp_sub(const mpq_t z1, const mpq_t z2, mpq_t z){mpq_sub(z,z1,z2);}
static inline void rat_gmp_mul(const mpq_t z1, const mpq_t z2, mpq_t z){mpq_mul(z,z1,z2);}
static inline void rat_gmp_div(const mpq_t z1, const mpq_t z2, mpq_t z){mpq_div(z,z1,z2);}
static inline void rat_gmp_add_ui_inplace(unsigned v, mpq_t r)
{
	mpz_addmul_ui(mpq_numref(r), mpq_denref(r), v);
	/* integer addition, no need to canonicalize since r was canonical
	 * before */
}
static inline void rat_gmp_sub_ui_inplace(unsigned v, mpq_t r)
{
	mpz_submul_ui(mpq_numref(r), mpq_denref(r), v);
	/* integer subtraction, no need to canonicalize since r was canonical
	 * before */
}
static inline void rat_gmp_add_si_inplace(int v, mpq_t r)
{
	if (v > 0) {
		rat_gmp_add_ui_inplace((unsigned)v, r);
	} else {
		/* (unsigned)-v has the absolute value of v in any's complement,
		 * also for v == INT_MIN */
		rat_gmp_sub_ui_inplace((unsigned)-v, r);
	}
}

static inline void rat_gmp_add_si(const mpq_t z1, int z2, mpq_t z)
{
	mpq_set(z, z1);
	rat_gmp_add_si_inplace(z2, z);
}

/*****************************************************************************/
/* adding rational and int in GMP                                            */
/* added: 18.06.2001                                                         */
/* Last change: 09.08.2017                                                   */
/*****************************************************************************/
static inline void rat_gmp_add_ui(const mpq_t z1, const unsigned int z2, mpq_t z)
{
	mpq_set(z, z1);
	rat_gmp_add_ui_inplace(z2, z);
}

/*****************************************************************************/
/* subtract rational and int in GMP                                          */
/* added: 18.06.2001                                                         */
/* Last change: 09.08.2017                                                   */
/*****************************************************************************/
static inline void rat_gmp_sub_ui(const mpq_t z1, const unsigned int z2, mpq_t z)
{
	mpq_set(z, z1);
	rat_gmp_sub_ui_inplace(z2, z);
}

/*****************************************************************************/
/* multiplying rational and int in GMP                                       */
/* added: 18.06.2001                                                         */
/* Last change: 09.08.2017                                                   */
/*****************************************************************************/
static inline void rat_gmp_mul_si(const mpq_t z1, const int z2, mpq_t z)
{
	mpz_mul_si(mpq_numref(z), mpq_numref(z1), z2);
	mpq_set_den(z, mpq_denref(z1));
	mpq_canonicalize(z);
}

/*****************************************************************************/
/* dividing rational by int in GMP                                           */
/* added: 18.06.2001                                                         */
/* Last change: 09.08.2017                                                   */
/*****************************************************************************/
static inline void rat_gmp_div_si(const mpq_t z1, const int z2, mpq_t z)
{
	mpz_mul_si(mpq_denref(z), mpq_denref(z1), z2);
	mpq_set_num(z, mpq_numref(z1));
	mpq_canonicalize(z);
}

/*****************************************************************************/
/* dividing int by rational in GMP                                           */
/* added: 18.06.2001                                                         */
/* Last change: 09.08.2017                                                   */
/* ***************************************************************************/
static inline void rat_gmp_si_div(const int z1, const mpq_t z2, mpq_t z)
{
	mpq_set_num(z, mpq_numref(z2));
	mpz_mul_si(mpq_denref(z), mpq_denref(z2), z1);
	mpz_swap(mpq_numref(z), mpq_denref(z));
	mpq_canonicalize(z);
}

static inline void rat_gmp_abs(const mpq_t z1, mpq_t z){mpq_abs(z,z1);}
static inline void rat_gmp_neg(const mpq_t z1, mpq_t z){mpq_neg(z,z1);}


/********** more MP rational functions *********/
/********** ^!%>>... **********/

/*****************************************************************************/
/* power function in GMP                                                     */
/* added: 19.02.2001                                                         */
/* Last change: 09.08.2017                                                   */
/* Comment: base=MP exponent=int                                             */
/*****************************************************************************/
static inline void rat_gmp_power(const mpq_t z1, unsigned long int z2, mpq_t z)
{
	mpz_pow_ui(mpq_numref(z), mpq_numref(z1), z2);
	mpz_pow_ui(mpq_denref(z), mpq_denref(z1), z2);
	mpq_canonicalize(z);
}

void rat_gmp_powerr(const mpq_t z1, const mpq_t z2, mpq_t z);
void rat_gmp_shift(const mpq_t z1, mpq_t z, int p);


/********** output functions for MP rationals ************/

char* rat_gmp_swritee(const mpq_t z, const int w);
static inline char* rat_gmp_sprintf(const mpq_t z){return mpq_get_str(NULL, 10, z);}


void rat_gmp_string_2_rat(mpq_t z, const char* s);
void rat_gmp_double_2_rat(mpq_t z, const double d);

static inline void rat_gmp_get_numerator(mpz_t z, const mpq_t z1){mpq_get_num(z,z1);}
static inline void rat_gmp_get_denominator(mpz_t z, const mpq_t z1){mpq_get_den(z,z1);}


/********** copying MP rationals with/without initializing ***********/
static inline void rat_gmp_duplicate_wo_init(const mpq_t z1, mpq_t z2){mpq_set(z2,z1);}

/********* sign and comparison of integer and rational */
static inline int rat_gmp_sgn(const mpq_t z){return mpq_sgn(z);}
static inline int rat_gmp_cmp(const mpq_t z1,const mpq_t z2){return mpq_cmp(z1,z2);}

static inline void rat_gmp_inv(const mpq_t x, mpq_t r) { mpq_inv(r, x); }

#ifdef __cplusplus
}
#endif

#endif /* ifndef iRRAM_GMP_INT_RAT_H */
