/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

/*
 * This is the structure holding the closure data for those functions
 * which use the standard tensor continuation.
 */

typedef struct {
	int n;
	Real x;
	TenXY *(*nextTensor)(Real, Real, int);
} ClsData;
