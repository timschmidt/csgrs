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
 * sec x = 1 / (cos x)
 */
Real
sec_R(Real x)
{
	return div_Int_R(1, cos_R(x));
}

Real
sec_QInt(int a, int b)
{
	return div_Int_R(1, cos_QInt(a, b));
}

Real
sec_QZ(mpz_t a, mpz_t b)
{
	return div_Int_R(1, cos_QZ(a, b));
}

/*
 * asec x = acos (1/x)
 */
Real
asec_R(Real x)
{
	return acos_R(div_Int_R(1, x));
}

Real
asec_QInt(int a, int b)
{
	return acos_QInt(b, a);
}

Real
asec_QZ(mpz_t a, mpz_t b)
{
	return acos_QZ(b, a);
}

/*
 * sech x = 1 / (cosh x)
 */
Real
sech_R(Real x)
{
	return div_Int_R(1, cosh_R(x));
}

Real
sech_QInt(int a, int b)
{
	return div_Int_R(1, cosh_QInt(a, b));
}

Real
sech_QZ(mpz_t a, mpz_t b)
{
	return div_Int_R(1, cosh_QZ(a, b));
}

/*
 * asech x = acosh (1/x)
 */
Real
asech_R(Real x)
{
	return acosh_R(div_Int_R(1, x));
}

Real
asech_QInt(int a, int b)
{
	return acosh_QInt(b, a);
}

Real
asech_QZ(mpz_t a, mpz_t b)
{
	return acosh_QZ(b, a);
}
