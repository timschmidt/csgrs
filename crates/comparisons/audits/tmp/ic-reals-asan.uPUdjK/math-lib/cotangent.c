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
 * cotan x = 1 / (tan x)
 */
Real
cotan_R(Real x)
{
	return div_Int_R(1, tan_R(x));
}

Real
cotan_QInt(int a, int b)
{
	return div_Int_R(1, tan_QInt(a, b));
}

Real
cotan_QZ(mpz_t a, mpz_t b)
{
	return div_Int_R(1, tan_QZ(a, b));
}

/*
 * acotan x = atan (1/x)
 */
Real
acotan_R(Real x)
{
	return atan_R(div_Int_R(1, x));
}

Real
acotan_QInt(int a, int b)
{
	return atan_QInt(b, a);
}

Real
acotan_QZ(mpz_t a, mpz_t b)
{
	return atan_QZ(b, a);
}

/*
 * cotanh x = 1 / (tanh x)
 */
Real
cotanh_R(Real x)
{
	return div_Int_R(1, tanh_R(x));
}

Real
cotanh_QInt(int a, int b)
{
	return div_Int_R(1, tanh_QInt(a, b));
}

Real
cotanh_QZ(mpz_t a, mpz_t b)
{
	return div_Int_R(1, tanh_QZ(a, b));
}

/*
 * acotanh x = atanh (1/x)
 */
Real
acotanh_R(Real x)
{
	return atanh_R(div_Int_R(1, x));
}

Real
acotanh_QInt(int a, int b)
{
	return atanh_QInt(b, a);
}

Real
acotanh_QZ(mpz_t a, mpz_t b)
{
	return atanh_QZ(b, a);
}
