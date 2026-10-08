/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdio.h>
#include "real.h"
#include "real-impl.h"

/*
 * A collection of convenient boolean predicates written in terms of
 * more primitive functions defined elsewhere.
 */

Bool
gt_R_QInt(Real x, int a, int b)
{
	return gt_R_0(sub_R_QInt(x, a, b));
}

Bool
ltEq_R_0(Real x)
{
	return not_B(gt_R_0(x));
}

Bool
ltEq_R_R(Real x, Real y)
{
	return ltEq_R_0(sub_R_R(x, y));
}

Bool
lt_R_R(Real x, Real y)
{
	return lt_R_0(sub_R_R(x, y));
}

Bool
lt_R_QInt(Real x, int a, int b)
{
	return gt_R_0(sub_QInt_R(a, b, x));
}

Bool
lt_R_0(Real x)
{
	return not_B(gtEq_R_0(x));
}

Bool
gtEq_R_QInt(Real x, int a, int b)
{
	return gtEq_R_0(sub_R_QInt(x, a, b));
}

Bool
ltEq_R_QInt(Real x, int a, int b)
{
	return gtEq_R_0(sub_QInt_R(a, b, x));
}

Bool
gtEq_R_R(Real x, Real y)
{
	return gtEq_R_0(sub_R_R(x, y));
}

Bool
gt_R_R(Real x, Real y)
{
	return gt_R_0(sub_R_R(x, y));
}

