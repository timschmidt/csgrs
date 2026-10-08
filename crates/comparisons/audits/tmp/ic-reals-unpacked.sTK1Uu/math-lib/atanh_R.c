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
 * One could do this more efficiently, but it will have to wait.
 */
Real
atanh_R(Real x)
{
	Real r;

	r = matrix_Int(x, 1, -1, 1, 1);
	r = log_R(r);
	r = div_R_Int(r, 2);
	return r;
}
