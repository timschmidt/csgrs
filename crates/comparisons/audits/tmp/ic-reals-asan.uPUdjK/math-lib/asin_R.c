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
 * asin(x) = atan(x/sqrt(1-x^2))
 */
Real
asin_R(Real x)
{
	Real r, s;

	r = mul_R_R(x, x);
	r = sub_Int_R(1, r);
	r = sqrt_R(r);
	r = div_R_R(x, r);
	r = atan_R(r);
	return r;
}
