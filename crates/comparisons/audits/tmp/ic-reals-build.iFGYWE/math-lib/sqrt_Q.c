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
 * It would be better to use a union of three int's and three mpz_t's and
 * then go to the bignumns when we overflow a machine word.
 */
typedef struct {
	mpz_t a, b, c;
} RatSqrtData;

static int doneInit = 0;

static void ratSqrtCont();
static bool emitDigitFromRatSqrt(RatSqrtData *, Digit *);

Real
sqrt_QInt(int a, int b)
{
	RatSqrtData *rsd;
	Cls *cls;
	DigsX *digsX;

	if (!doneInit) {
		registerForceFunc(ratSqrtCont, "ratSqrtCont", 3);
		doneInit++;
	}

	if ((rsd = (RatSqrtData *) malloc(sizeof(RatSqrtData))) == NULL)
		Error(FATAL, E_INT, "sqrt_QInt", "malloc failed");

	mpz_init_set_ui(rsd->a, a);
	mpz_init_set_ui(rsd->b, b);
	mpz_init_set_ui(rsd->c, a - b);

	cls = allocCls(ratSqrtCont, (void *) rsd);
	cls->tag.isSigned = FALSE;

	digsX = allocDigsX();
	digsX->x = (Real) cls;
	digsX->force = ratSqrtCont;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(digsX, cls);
		endGraphUpdate();
	}

	return (Real) digsX;
}

Real
sqrt_QZ(mpz_t a, mpz_t b)
{
	RatSqrtData *rsd;
	Cls *cls;
	DigsX *digsX;

	if (!doneInit) {
		registerForceFunc(ratSqrtCont, "ratSqrtCont", 3);
		doneInit++;
	}

	if ((rsd = (RatSqrtData *) malloc(sizeof(RatSqrtData))) == NULL)
		Error(FATAL, E_INT, "sqrt_QZ", "malloc failed");

	mpz_init_set(rsd->a, a);
	mpz_init_set(rsd->b, b);
	mpz_init(rsd->c);
	mpz_sub(rsd->c, rsd->a, rsd->b);

	cls = allocCls(ratSqrtCont, (void *) rsd);
	cls->tag.isSigned = FALSE;

	digsX = allocDigsX();
	digsX->x = (Real) cls;
	digsX->force = ratSqrtCont;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(digsX, cls);
		endGraphUpdate();
	}

	return (Real) digsX;
}

static void
ratSqrtCont()
{
	DigsX *digsX;
	Cls *cls;
	int digitsNeeded;
	RatSqrtData *rsd;

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) digsX->x;

	rsd = (RatSqrtData *) cls->userData;

	emitDigits(digsX, (edf) emitDigitFromRatSqrt, (void *) rsd, digitsNeeded);
	newDigsX(digsX);
}

static bool
emitDigitFromRatSqrt(RatSqrtData *rsd, Digit *d)
{
	mpz_sub(tmpd_z, rsd->b, rsd->a);
	mpz_mul_2exp(tmpd_z, tmpd_z, 1);
	mpz_add(tmpd_z, tmpd_z, rsd->c);

	switch (mpz_sgn(tmpd_z)) {
		case 0 :
		case 1 :
			*d = DNEG;
			mpz_mul_2exp(rsd->a, rsd->a, 2);  /* a = 4a */
			mpz_set(rsd->b, tmpd_z);
			break;
		case -1 :
			*d = DPOS;
			mpz_neg(rsd->a, tmpd_z);
			mpz_mul_2exp(rsd->b, rsd->b, 2);  /* b = 4b */
			break;
		default :
			Error(FATAL, E_INT, "emitDigitFromRatSqrt",
							"bad value returned by mpz_sgn");
			break;
	}
	return TRUE;
}
