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
 * Functions for allocating and managing vector LFTs in the heap.
 */

Vec *
allocVec()
{
	Vec *vec;

	if ((vec = (Vec *) malloc (sizeof(Vec))) == NULL) 
		Error(FATAL, E_INT, "allocVec", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(vec);

	vec->tag.type = VECTOR;
	vec->tag.dumped = FALSE;
	vec->strm = (Real) NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(vec, VECTOR);
		endGraphUpdate();
	}

	return vec;
}

/*
 * The next family of functions are one level up. They allocate structures
 * in the heap for lfts and also initialize both the entries in the lft
 * as well as the pointers to the heap objects to which the lft is being
 * applied.
 */

/*
 * Create a vector with small entries. A vector is a rational. We might
 * wish to disallow 0/0.  
 */
Real
vector_Int(int a, int b)
{
	Vec *vec;

	if ((a == 0) && (b == 0))
		Error(FATAL, E_INT, "vector_Int", "zero column vector");

	vec = allocVec();
	mpz_init_set_si(vec->vec[0], a);
	mpz_init_set_si(vec->vec[1], b);
	canonVector(vec->vec);

	if (vectorSign(vec->vec) == 0)
		vec->tag.isSigned = TRUE;
	else
		vec->tag.isSigned = FALSE;

	return (Real) vec;
}

Real
vector_Z(mpz_t a, mpz_t b)
{
	Vec *vec;

	if ((mpz_sgn(a) == 0) && (mpz_sgn(b) == 0))
		Error(FATAL, E_INT, "vector_Z", "zero column vector");

	vec = allocVec();
	mpz_init_set(vec->vec[0], a);
	mpz_init_set(vec->vec[1], b);
	canonVector(vec->vec);

	if (vectorSign(vec->vec) == 0)
		vec->tag.isSigned = TRUE;
	else
		vec->tag.isSigned = FALSE;

	return (Real) vec;
}
