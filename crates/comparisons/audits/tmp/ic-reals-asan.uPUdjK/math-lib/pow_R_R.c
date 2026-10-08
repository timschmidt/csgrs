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
pow_R_R(Real x, Real y)
{
	Real r;

	r = log_R(x);
	r = mul_R_R(y, r);
	return exp_R(r);
}
