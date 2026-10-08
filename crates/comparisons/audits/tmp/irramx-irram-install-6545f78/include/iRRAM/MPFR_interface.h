/*

MPFR_interface.h -- interface to the MPFR float functions for the iRRAM library
 
Copyright (C) 2002-2004 Norbert Mueller
 
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
#ifndef MPFR_INTERFACE_H
#define MPFR_INTERFACE_H

#include <iRRAM/GMP_intrat.h>
#include <mpfr.h>

# ifndef BITS_PER_MP_LIMB
#  define BITS_PER_MP_LIMB GMP_LIMB_BITS
# endif


#define GMP_min -1000000000
#define GMP_max 1000000000

#define MP_min    GMP_min
#define MP_max    GMP_max


/****** Type definitions ******/

#define MP_type         mpfr_ptr
#define MP_int_type     mpz_ptr
#define MP_rat_type     mpq_ptr


/****** Initialization functions ******/

/* Backend initialization (if necessary) */
#define MP_initialize(st)   ext_mpfr_initialize(&(st)->ext_mpfr_cache)
#define MP_finalize(st)     ext_mpfr_finalize(&(st)->ext_mpfr_cache)

/* Initialization of MP/integer/rational variables */
#define MP_init(z)      do { z = ext_mpfr_init(&state->ext_mpfr_cache); } while (0)
#define MP_int_init(z)  do { z = int_gmp_init(&state->mpz_cache); } while (0)
#define MP_rat_init(z)  do { z = rat_gmp_init(&state->mpq_cache); } while (0)

/* Deletion of MP/integer/rational variables */
#define MP_clear(z)     ext_mpfr_free(&state->ext_mpfr_cache, z)
#define MP_int_clear(z) int_gmp_free(&state->mpz_cache, z)
#define MP_rat_clear(z) rat_gmp_free(&state->mpq_cache, z)


/****** typechanging functions ******/
#define MP_mp_to_double(z)	mpfr_get_d(z,GMP_RNDN)
#define MP_double_to_mp(d,z)                                                   \
	do {   	                                                               \
		mpfr_set_prec(z,53);                                           \
		mpfr_set_d(z,d,iRRAM_mpfr_rounding_mode);                      \
	} while (0)

#define MP_int_to_mp(i,z)                                                      \
	do {                                                                   \
		mpfr_set_prec(z,32);                                           \
		mpfr_set_si(z,i,iRRAM_mpfr_rounding_mode);                     \
	} while (0)
#define MP_int_to_INTEGER(i,z)	mpz_set_si(z,i)

#define MP_INTEGER_to_mp(i,r)	ext_mpfr_set_z(r,i)
#define MP_INTEGER_to_int(z)	mpz_get_si(z)

#define MP_mp_to_INTEGER(r,i)	mpfr_get_z(i,r,MPFR_RNDZ)
/* here  MPFR_RNDZ is chosen as GMP does a truncation in the corresponding function */

#define MP_double_to_INTEGER(i,z)   mpz_set_d(z,i)
#define MP_string_to_INTEGER(s,z,b) mpz_set_str(z,s,b)

#define MP_int_to_RATIONAL(i,z)		mpq_set_si(z,i,1)
#define MP_intint_to_RATIONAL(i,j,z)                                           \
	do {                                                                   \
		mpq_set_si(z,i,j);                                             \
		mpq_canonicalize(z);                                           \
	} while (0)
#define MP_double_to_RATIONAL(d,z)	mpq_set_d(z,d)
#define MP_string_to_RATIONAL(s,z)	rat_gmp_string_2_rat(z,s)
#define MP_INTEGER_to_RATIONAL(i,r)	mpq_set_z(r,i)
#define MP_INTINTEGER_to_RATIONAL(i,j,r)                                       \
	do {                                                                   \
		mpq_set_num(r,i);                                              \
		mpq_set_den(r,j);                                              \
		mpq_canonicalize(r);                                           \
	} while (0)


/* duplicate value z1 to z2, with/without initialization of z2 */

#define MP_duplicate_w_init(z1,z2)	do { MP_init(z2); MP_duplicate_wo_init(z1, z2); } while (0)
#define MP_duplicate_wo_init(z1,z2)	ext_mpfr_duplicate_wo_init(z1,z2) 

#define MP_int_duplicate_w_init(z1,z2)	do { MP_int_init(z2); MP_int_duplicate_wo_init(z1, z2); } while (0)
#define MP_int_duplicate_wo_init(z1,z2)	int_gmp_duplicate_wo_init(z1,z2) 

#define MP_rat_duplicate_w_init(z1,z2)	do { MP_rat_init(z2); MP_rat_duplicate_wo_init(z1, z2); } while (0)
#define MP_rat_duplicate_wo_init(z1,z2)	rat_gmp_duplicate_wo_init(z1,z2) 

/* copy z1 to z2, but precision p is sufficient */
#define MP_copy(z1,z2,p)		ext_mpfr_duplicate_wo_init(z1,z2)


/* Multiple precision arithmetic, deterministic results */
#define MP_add(z1,z2,z,p)  ext_mpfr_add(z1,z2,z,p)
#define MP_sub(z1,z2,z,p)  ext_mpfr_sub(z1,z2,z,p)
#define MP_mul(z1,z2,z,p)  ext_mpfr_mul(z1,z2,z,p) 
#define MP_div(z1,z2,z,p)  ext_mpfr_div(z1,z2,z,p) 
#define MP_addi(z1,z2,z,p) ext_mpfr_add_i(z1,z2,z,p)
#define MP_subi(z1,z2,z,p) ext_mpfr_sub_i(z1,z2,z,p)
#define MP_isub(z1,z2,z,p) ext_mpfr_i_sub(z1,z2,z,p)
#define MP_muli(z1,z2,z,p) ext_mpfr_mul_i(z1,z2,z,p) 
#define MP_divi(z1,z2,z,p) ext_mpfr_div_i(z1,z2,z,p)
#define MP_idiv(z1,z2,z,p) ext_mpfr_i_div(z1,z2,z,p); 
#define MP_abs(z1,z)       ext_mpfr_abs(z1,z)

/* Multiple precision arithmetic with multi-valued results:
  The following versions of the arithmetic operations
  are allowed to be multi-valued, here in the sense that
  even for absolutely identical parameters there can be
  different results. 
*/    
#define MP_mv_add(z1,z2,z,p)  MP_add(z1,z2,z,p)  
#define MP_mv_sub(z1,z2,z,p)  MP_sub(z1,z2,z,p)  
#define MP_mv_mul(z1,z2,z,p)  MP_mul(z1,z2,z,p)  
#define MP_mv_div(z1,z2,z,p)  MP_div(z1,z2,z,p)  
#define MP_mv_addi(z1,z2,z,p) MP_addi(z1,z2,z,p) 
#define MP_mv_subi(z1,z2,z,p) MP_subi(z1,z2,z,p) 
#define MP_mv_isub(z1,z2,z,p) MP_isub(z1,z2,z,p) 
#define MP_mv_muli(z1,z2,z,p) MP_muli(z1,z2,z,p) 
#define MP_mv_divi(z1,z2,z,p) MP_divi(z1,z2,z,p) 
#define MP_mv_idiv(z1,z2,z,p) MP_idiv(z1,z2,z,p)  

/* INTEGER arithmetic, deterministic results */
#define MP_int_add(z1,z2,z)     int_gmp_add(z1,z2,z)
#define MP_int_sub(z1,z2,z)     int_gmp_sub(z1,z2,z)
#define MP_int_mul(z1,z2,z)     int_gmp_mul(z1,z2,z) 
#define MP_int_div(z1,z2,z)     int_gmp_div(z1,z2,z) 
#define MP_int_add_ui(z1,z2,z)	int_gmp_add_ui(z1,z2,z)
#define MP_int_sub_ui(z1,z2,z)	int_gmp_sub_ui(z1,z2,z)
#define MP_int_mul_si(z1,z2,z)	int_gmp_mul_si(z1,z2,z)
#define MP_int_div_ui(z1,z2,z)	int_gmp_div_ui(z1,z2,z)
#define MP_int_abs(z1,z)        int_gmp_abs(z1,z)
#define MP_int_neg(z1,z)        int_gmp_neg(z1,z)

/* RATIONAL arithmetic, deterministic results */
#define MP_rat_add(z1,z2,z) rat_gmp_add(z1,z2,z)
#define MP_rat_sub(z1,z2,z) rat_gmp_sub(z1,z2,z)
#define MP_rat_mul(z1,z2,z) rat_gmp_mul(z1,z2,z)
#define MP_rat_div(z1,z2,z) rat_gmp_div(z1,z2,z) 
#define MP_rat_add_ui_inplace(z1,z) rat_gmp_add_ui_inplace(z1,z)
#define MP_rat_add_si_inplace(z1,z) rat_gmp_add_si_inplace(z1,z)
#define MP_rat_sub_ui_inplace(z1,z) rat_gmp_sub_ui_inplace(z1,z)
#define MP_rat_add_si(z1,z2,z) rat_gmp_add_si(z1,z2,z)
#define MP_rat_add_ui(z1,z2,z) rat_gmp_add_ui(z1,z2,z)
#define MP_rat_sub_ui(z1,z2,z) rat_gmp_sub_ui(z1,z2,z)
#define MP_rat_mul_si(z1,z2,z) rat_gmp_mul_si(z1,z2,z)
#define MP_rat_div_si(z1,z2,z) rat_gmp_div_si(z1,z2,z)
#define MP_rat_si_div(z1,z2,z) rat_gmp_si_div(z1,z2,z)
#define MP_rat_abs(z1,z)       rat_gmp_abs(z1,z)
#define MP_rat_neg(z1,z)       rat_gmp_neg(z1,z)


/* Additional integer/rational arithmetic */
#define MP_int_sqrt(z1,z)        int_gmp_sqrt(z1,z);
#define MP_int_root(z1,z2,z)     int_gmp_root(z1,z2,z);
#define MP_int_power_i(z1,z2,z)  int_gmp_power_i(z1,z2,z);
#define MP_int_power_ii(z1,z2,z) int_gmp_power_ii(z1,z2,z);
#define MP_int_fac(z1,z)         int_gmp_fac(z1,z);
#define MP_int_modulo(z1,z2,z)   int_gmp_modulo(z1,z2,z);
#define MP_int_log(z)            int_gmp_log(z);
#define MP_rat_power(z1,z2,z)    rat_gmp_power(z1,z2,z);
#define MP_rat_inv(z1,z)         rat_gmp_inv(z1,z)
#define MP_rat_power2(z1,z2,z)   rat_gmp_power_ii(z1,z2,z);
#define MP_rat_get_numerator(z1,z) mpq_get_num(z,z1) 
#define MP_rat_get_denominator(z1,z) mpq_get_den(z,z1)
#define MP_rat_canon(z)          mpq_canonicalize(z) 

/* MP_compare: values -1 (lessthan), 0 (equal), or 1 (greater than) */
#define MP_compare(z1,z2)        mpfr_cmp(z1,z2)
#define MP_int_compare(z1,z2)    int_gmp_cmp(z1,z2)
#define MP_rat_compare(z1,z2)    rat_gmp_cmp(z1,z2)

/* MP_sign: values -1 (negative), 0 (zero), or 1 (positive) */
#define MP_sign(z)          mpfr_sgn(z)
#define MP_int_sign(z)      int_gmp_sgn(z)
#define MP_rat_sign(z)      rat_gmp_sgn(z)

/* MP_equality for rational values: 0 (not equal), or nonzero (equal) */
#define MP_rat_equal(z1,z2) mpq_equal (z1,z2)


/****** size information ******/
/* int MP_size : smallest(?) value k such that 2**k > |z|  */
#define MP_size(z)       ext_mpfr_size(z)
#define MP_getsize(z,s)  ext_mpfr_getsize(z,(ext_mpfr_sizetype*)&s)
#define MP_int_size(z)   int_gmp_size(z)

/* truncate z1 yielding z2 */ 
#define MP_truncate(z1,z)   ext_mpfr_truncate(z1,z)

/*Conversion to strings */
#define MP_swrite(z,w)      ext_mpfr_swritee(z,w)

/* In/Out */
#define MP_writee(z,w)      ext_mpfr_writee(z,w)
#define MP_writef(z,w,d)    mpfr_out_str(stdout,10,w,z,iRRAM_mpfr_rounding_mode)
#define MP_write(z,c)       ext_mpfr_write(z,c)

#define MP_int_writee(z,w)  int_gmp_writee(z,w)
#define MP_int_swritee(z,w) int_gmp_swritee(z,w)
#define MP_int_writef(z,w)  int_gmp_outstr(z,w)
#define MP_int_write(z,c)   int_gmp_write(z,c)
#define MP_int_printf(z)    int_gmp_printf(z)
#define MP_int_sprintf(z)   int_gmp_sprintf(z)

#define MP_rat_writee(z,w)  rat_gmp_writee(z,w)
#define MP_rat_writef(z,w)  rat_gmp_outstr(z,w)
#define MP_rat_swritee(z,w) rat_gmp_swritee(z,w)
#define MP_rat_sprintf(z)   rat_gmp_sprintf(z)

/* Statistics, optional */

#define MP_var_count        (state->ext_mpfr_cache.ext_mpfr_var_count)
/* #define MP_space_count  */
/* #define MP_max_space_count */


/* special functions, optional for the backends */
#define MP_sqrt(z1,z,p) ext_mpfr_sqrt(z1,z,p)
#define MP_mv_sqrt(z1,z,p)   MP_sqrt(z1,z,p)
#define MP_shift(z1,z,n) ext_mpfr_shift(z1,z,n)
#define MP_int_shift(z1,z,n) int_gmp_shift(z1,z,n)
#define MP_rat_shift(z1,z,n) rat_gmp_shift(z1,z,n)


/***********************************************************/
/* Declaration of those types/functions that are necessary */
/* for user programs:                                       */ 
/***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {unsigned int mantissa; int exponent; } ext_mpfr_sizetype;
typedef mpz_ptr  int_mpfr_type;

#define iRRAM_EXT_MPFR_CACHE_SIZE 1000 /* TODO: make adjustable during init() */

struct iRRAM_ext_mpfr_cache_t {
	int free_var_count;
	int ext_mpfr_var_count;
	size_t total_alloc_var_count;
	size_t total_freed_var_count;
	mpfr_ptr free_vars[iRRAM_EXT_MPFR_CACHE_SIZE];
};

#define iRRAM_EXT_MPFR_CACHE_INIT	{ 0, 0, 0, 0, {0} }

void ext_mpfr_remove_trailing_zeroes (mpfr_t x);

void ext_mpfr_initialize(struct iRRAM_ext_mpfr_cache_t *);
void ext_mpfr_finalize(struct iRRAM_ext_mpfr_cache_t *);

void ext_mpfr_getsize(const mpfr_t z,ext_mpfr_sizetype* s);

#ifdef __cplusplus
}
#endif


mpfr_ptr ext_mpfr_init(struct iRRAM_ext_mpfr_cache_t *);

inline mpfr_ptr ext_mpfr_init(struct iRRAM_ext_mpfr_cache_t *cache)
{
	mpfr_ptr z;
	if (cache->free_var_count > 0) {
		cache->free_var_count -= 1;
		z = cache->free_vars[cache->free_var_count];
	} else {
		z = (mpfr_ptr)malloc(sizeof(mpfr_t));
		mpfr_init(z);
		cache->total_alloc_var_count++;
	}
	cache->ext_mpfr_var_count += 1;

	/* fprintf(stderr,"create %x\n",z); */

	return z;
}

void ext_mpfr_free(struct iRRAM_ext_mpfr_cache_t *, mpfr_ptr z);

inline void ext_mpfr_free(struct iRRAM_ext_mpfr_cache_t *cache, mpfr_ptr z)
{
	/* fprintf(stderr,"delete %x\n",z); */
	if (cache->free_var_count < iRRAM_EXT_MPFR_CACHE_SIZE) {
		cache->free_vars[cache->free_var_count] = z;
		cache->free_var_count += 1;
	} else {
		mpfr_clear(z);
		free(z);
		cache->total_freed_var_count++;
	}
	cache->ext_mpfr_var_count -= 1;
}

#endif /*ifndef MPFR_INTERFACE_H */
