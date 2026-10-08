/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdio.h>
#include "real.h"

/*
 * These will be specialized in a later version.
 */
Real
tan_QInt(int a, int b)
{
	Real r;

	r = vector_Int(a, b);
	return tan_R(r);
}

Real
tan_QZ(mpz_t a, mpz_t b)
{
	Real r;

	r = vector_Z(a, b);
	return tan_R(r);
}
