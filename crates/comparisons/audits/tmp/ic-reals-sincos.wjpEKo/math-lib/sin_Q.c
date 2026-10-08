/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdio.h>
#include "real.h"

Real
sin_QZ(mpz_t a, mpz_t b)
{
	mpz_t x;
	Real r;

	mpz_init(x);
	mpz_mul_ui(x, b, 2);
	r = tan_QZ(a, x);
	mpz_clear(x);
	r = tensor_Int(r, r, 0, 1, 1, 0, 1, 0, 0, 1);
	return r;
}

Real
sin_QInt(int a, int b)
{
	Real r;
	mpz_t ap, bp;

	/* check for overflow */
	if (b > 0x3FFFFFFF || b < -0x40000000) {
		mpz_init_set_si(ap, a);
		mpz_init_set_si(bp, b);
		mpz_mul_ui(bp, bp, 2);
		r = tan_QZ(ap, bp);
		mpz_clear(ap);
		mpz_clear(bp);
	}
	else
		r = tan_QInt(a, b * 2);

	r = tensor_Int(r, r, 0, 1, 1, 0, 1, 0, 0, 1);
	return r;
}
