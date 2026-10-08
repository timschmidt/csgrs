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
 * These functions should probably be broken into separate files.
 */

/*
 * cosec x = 1 / (sin x)
 */
Real
cosec_R(Real x)
{
	return div_Int_R(1, sin_R(x));
}

Real
cosec_QInt(int a, int b)
{
	return div_Int_R(1, sin_QInt(a, b));
}

Real
cosec_QZ(mpz_t a, mpz_t b)
{
	return div_Int_R(1, sin_QZ(a, b));
}

/*
 * acosec x = asin (1/x)
 */
Real
acosec_R(Real x)
{
	return asin_R(div_Int_R(1, x));
}

Real
acosec_QInt(int a, int b)
{
	return asin_QInt(b, a);
}

Real
acosec_QZ(mpz_t a, mpz_t b)
{
	return asin_QZ(b, a);
}

/*
 * cosech x = 1 / (sinh x)
 */
Real
cosech_R(Real x)
{
	return div_Int_R(1, sinh_R(x));
}

Real
cosech_QInt(int a, int b)
{
	return div_Int_R(1, sinh_QInt(a, b));
}

Real
cosech_QZ(mpz_t a, mpz_t b)
{
	return div_Int_R(1, sinh_QZ(a, b));
}

/*
 * acosech x = asinh (1/x)
 */
Real
acosech_R(Real x)
{
	return asinh_R(div_Int_R(1, x));
}

Real
acosech_QInt(int a, int b)
{
	return asinh_QInt(b, a);
}

Real
acosech_QZ(mpz_t a, mpz_t b)
{
	return asinh_QZ(b, a);
}
