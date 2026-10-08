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
 
void
force_B(Bool b, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		if (boolValue(b) != LAZY_UNKNOWN) break;
		PUSH_2 (b->gen.force, b);
		runStack();
	}
}
