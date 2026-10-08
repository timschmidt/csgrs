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
 * Functions dealing with the epsilon-delta analysis for matrices and
 * tensors.
 */

/*
static mpz_t vec0_sum, vec1_sum, vec2_sum, vec3_sum;
*/

#define vec0_sum tmpc_z
#define vec1_sum tmpd_z
#define vec2_sum tmpe_z
#define vec3_sum tmpf_z

static int EpsDelVectorPair(Vector, Vector, mpz_t, mpz_t, int);

void
initEpsDel()
{
/*
 * These aren't needed anymore as temporary storage is shared amongst
 * the entire library
 *
	mpz_init(vec0_sum);
	mpz_init(vec1_sum);
	mpz_init(vec2_sum);
	mpz_init(vec3_sum);
*/
}

/*
 * Given a matrix and an epsilon (number of digits needed), this returns
 * the number of digits the matrix must consume from its argument
 */
int
epsDelMatrix(Matrix mat, int epsilon)
{
	mpz_add(vec0_sum, mat[0][0], mat[0][1]);
	mpz_add(vec1_sum, mat[1][0], mat[1][1]);
	return EpsDelVectorPair(mat[0], mat[1], vec0_sum, vec1_sum, epsilon);
}

/*
 * As for matrices above, but this gives the number of digits needed from
 * the two arguments of a tensor.
 */
void
epsDelTensor(Tensor ten, int epsilon, int *nLeft, int *nRight)
{
	int r, s, p, q;

	mpz_add(vec0_sum, ten[0][0], ten[0][1]);
	mpz_abs(vec0_sum, vec0_sum);
	mpz_add(vec1_sum, ten[1][0], ten[1][1]);
	mpz_abs(vec1_sum, vec1_sum);
	mpz_add(vec2_sum, ten[2][0], ten[2][1]);
	mpz_abs(vec2_sum, vec2_sum);
	mpz_add(vec3_sum, ten[3][0], ten[3][1]);
	mpz_abs(vec3_sum, vec3_sum);

	r = EpsDelVectorPair(ten[0], ten[2], vec0_sum, vec2_sum, epsilon);
	s = EpsDelVectorPair(ten[1], ten[3], vec1_sum, vec3_sum, epsilon);
	p = EpsDelVectorPair(ten[0], ten[1], vec0_sum, vec1_sum, epsilon);
	q = EpsDelVectorPair(ten[2], ten[3], vec2_sum, vec3_sum, epsilon);

	*nLeft = MIN(r,s);
	*nRight = MIN(p,q);
}

/*
 * As above for tensors, but we are interested only in the number
 * of arguments needed from the x (left) argument.
 */
int
epsDelTensorX(Tensor ten, int epsilon)
{
	int r, s, p, q;

	mpz_add(vec0_sum, ten[0][0], ten[0][1]);
	mpz_abs(vec0_sum, vec0_sum);
	mpz_add(vec1_sum, ten[1][0], ten[1][1]);
	mpz_abs(vec1_sum, vec1_sum);
	mpz_add(vec2_sum, ten[2][0], ten[2][1]);
	mpz_abs(vec2_sum, vec2_sum);
	mpz_add(vec3_sum, ten[3][0], ten[3][1]);
	mpz_abs(vec3_sum, vec3_sum);

	r = EpsDelVectorPair(ten[0], ten[2], vec0_sum, vec2_sum, epsilon);
	s = EpsDelVectorPair(ten[1], ten[3], vec1_sum, vec3_sum, epsilon);

	return MIN(r,s);
}

static int
EpsDelVectorPair(Vector vec0, Vector vec1,
	mpz_t vec0_sum,
	mpz_t vec1_sum,
	int epsilon)
{
	int logdet, logalsq, bitcount;
	
	mpz_mul(tmpa_z, vec0[0], vec1[1]);
	mpz_mul(tmpb_z, vec0[1], vec1[0]);
	mpz_sub(tmpa_z, tmpa_z, tmpb_z);
	mpz_abs(tmpa_z, tmpa_z);
	logdet = mpz_sizeinbase(tmpa_z, 2);

	if (mpz_cmp(vec0_sum, vec1_sum) >= 0) {
		bitcount = mpz_popcount(vec0_sum);
		mpz_mul(tmpa_z, vec0_sum, vec0_sum);
	}
	else {
		bitcount = mpz_popcount(vec1_sum);
		mpz_mul(tmpa_z, vec1_sum, vec1_sum);
	}
	logalsq = mpz_sizeinbase(tmpa_z, 2);
	if (bitcount == 1)
		return epsilon + logdet - logalsq;
	else
		return epsilon + logdet - logalsq - 1;
}
