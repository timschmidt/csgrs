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
tanh_R(Real x)
{
	Real r;

	r = exp_R(x);
	return tensor_Int(r, r, 1, 1, 0, 0, 0, 0, -1, 1);
}
