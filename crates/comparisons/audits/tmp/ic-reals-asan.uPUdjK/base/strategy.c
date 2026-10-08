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
 * Functions related to ``information overlap'' tensor strategy. Can
 * probably be dropped and reky soley on epsilon-delta stuff.
 */
static mpz_t a_times_d, c_times_b;

void
initStrategy()
{
	mpz_init(a_times_d);
	mpz_init(c_times_b);
}

/*
 * Given vectors (a,b) and (c,d), this returns
 *  -1 if (a,b) < (c,d)
 *  =0 if (a,b) = (c,d)
 *  +1 if (a,b) > (c,d)
 */
static int
compareVectors(Vector v0, Vector v1)
{
	int tmp;

	mpz_mul(a_times_d, v0[0], v1[1]);
	mpz_mul(c_times_b, v1[0], v0[1]);
	tmp = mpz_cmp(a_times_d, c_times_b);
	return MPZ_SIGN(tmp);
}

/*
 * This is Peter's ``information overlap'' strategy. 
 *
 * If the structure of the code seems odd, it is because we want to avoid
 * doing the same operations more than once. C guarantees that when evaluating
 * a conjunction, if the first conjunct is false, then the second is
 * not evaluated.
 *
 * This returns 1 to select y (right) and 0 to select x (left).
 */
int
tensorStrategy(Tensor t)
{
	int v0cv1;

	/*
	 * we compare:
	 *  v0 <> v1
	 *  v0 <> v3
	 *  v2 <> v1
	 *  v2 <> v3
	 */
	v0cv1 = compareVectors(t[0], t[1]);
	if (v0cv1 > 0) {
		if ((compareVectors(t[0], t[3]) > 0)
			&& (compareVectors(t[2], t[1]) > 0)
			&& (compareVectors(t[2], t[3]) > 0)) {
			return 1;
		}
		else
			return 0;
	}
	else {
		if (v0cv1 < 0) {
			if ((compareVectors(t[0], t[3]) < 0)
				&& (compareVectors(t[2], t[1]) < 0)
				&& (compareVectors(t[2], t[3]) < 0)) {
				return 1;
			}
			else
				return 0;
		}
	}
	return 0;
}
