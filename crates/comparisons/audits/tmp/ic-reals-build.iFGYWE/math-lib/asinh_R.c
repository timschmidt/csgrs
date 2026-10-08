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
asinh_R(Real x)
{
	Real r;

	r = tensor_Int(x, x, 1, 0, 0, 0, 0, 0, 1, 1);
	r = sqrt_R(r);
	r = add_R_R(x, r);
	r = log_R(r);
	return r;
}
