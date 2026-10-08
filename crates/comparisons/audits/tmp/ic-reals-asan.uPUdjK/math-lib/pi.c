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

static mpz_t k;

Real Pi;

static void piCont();
static void nextMatrix(Tensor, int);

void
initPi()
{
	Real sq, cls;
	DigsX *digsX;
	TenXY *tenXY;
	static int doneInit;
	void force_To_DigsX_From_TenXY_Entry();

	if (!doneInit) {
		registerForceFunc(piCont, "piCont", 2);
		doneInit++;
	}

	mpz_init_set_str(k, "10939058860032000", 10);

	sq = sqrt_QInt(10005, 1);
	cls = (Real) allocCls(piCont, (void *) 0);
	cls->gen.tag.isSigned = FALSE;

	tenXY = (TenXY *) div_R_R(sq, cls);
	tenXY->forceY = piCont;

	digsX = allocDigsX();
	digsX->x = (Real) tenXY;
	digsX->force = force_To_DigsX_From_TenXY_Entry;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(digsX, tenXY);
		endGraphUpdate();
	}

	Pi = (Real) digsX;
}

/*
 * It might be possible compute the next matrix in the sequence
 * in terms of the previous matrix. This would mean having much more
 * state but presumably much quicker.
 */
static void
piCont()
{
	TenXY *tenXY;
	Cls *cls;
	int digitsNeeded;
	int n;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	cls = (Cls *) tenXY->y;
	n = (int) cls->userData;

	/*
	 * The two constants below have been determined empirically. The second
	 * may be off by one (ie 47) in some circumstances.
	 */
	while (digitsNeeded > 0) {
		nextMatrix(tenXY->ten, n);
		if (n == 0)
			digitsNeeded -= 27;
		else
			digitsNeeded -= 46;
		n +=1;
	}
	cls->userData = (void *) n;
}

static void
nextMatrix(Tensor ten, int n)
{
	SmallMatrix smallMat;

	if (n == 0) {
		smallMat[0][0] = 6795705;
		smallMat[0][1] = 213440;
		smallMat[1][0] = 6795704;
		smallMat[1][1] = 213440;
		multVectorPairTimesSmallMatrix(ten[0], ten[1], smallMat);
		multVectorPairTimesSmallMatrix(ten[2], ten[3], smallMat);
	}
	else {
		/* b = 2n - 1 */
		mpz_set_ui(tmpb_z, n);
		mpz_mul_2exp(tmpb_z, tmpb_z, 1);
		mpz_sub_ui(tmpb_z, tmpb_z, 1);
	
		/* tmp = 6n - 5, tmpf_z = 6n */
		mpz_set_ui(tmpa_z, n);
		mpz_mul_ui(tmpa_z, tmpa_z, 6);
		mpz_set(tmpf_z, tmpa_z);
		mpz_sub_ui(tmpa_z, tmpa_z, 5);
	
		/* tmpf_z = 6n - 1 */
		mpz_sub_ui(tmpf_z, tmpf_z, 1);
	
		/* b = (2n - 1) * (6n - 5) * (6n - 1) */
		mpz_mul(tmpb_z, tmpb_z, tmpa_z);
		mpz_mul(tmpb_z, tmpb_z, tmpf_z);
	
		/* c = b*(545140134n + 13591409) */
		mpz_set_ui(tmpa_z, 545140134);
		mpz_mul_ui(tmpa_z, tmpa_z, n);
		mpz_add_ui(tmpa_z, tmpa_z, 13591409);
		mpz_mul(tmpc_z, tmpb_z, tmpa_z);
	
		/* d = b * (n + 1) */
		mpz_set_ui(tmpd_z, n + 1);
		mpz_mul(tmpd_z, tmpd_z, tmpb_z);
	
		/* e = 10939058860032000*n^4 */
		mpz_set_ui(tmpe_z, n);
		mpz_mul(tmpe_z, tmpe_z, tmpe_z);
		mpz_mul(tmpe_z, tmpe_z, tmpe_z);
		mpz_mul(tmpe_z, tmpe_z, k);
	
		mpz_sub(bigTmpMat[0][0], tmpe_z, tmpd_z);
		mpz_set(bigTmpMat[1][1], bigTmpMat[0][0]);
		mpz_add(bigTmpMat[0][1], tmpe_z, tmpd_z);
		mpz_set(bigTmpMat[1][0], bigTmpMat[0][1]);
	
		mpz_sub(bigTmpMat[0][0], bigTmpMat[0][0], tmpc_z);
		mpz_add(bigTmpMat[0][1], bigTmpMat[0][1], tmpc_z);
		mpz_sub(bigTmpMat[1][0], bigTmpMat[1][0], tmpc_z);
		mpz_add(bigTmpMat[1][1], bigTmpMat[1][1], tmpc_z);
	
		multVectorPairTimesMatrix(ten[0], ten[1], bigTmpMat);
		multVectorPairTimesMatrix(ten[2], ten[3], bigTmpMat);
	}
}
