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
#include "math-lib.h"

static TenXY *nextTensor(Real, Real, int);
static void atanInside();
static void atanPositive();
static void atanNegative();

Real
atan_R(Real x)
{
	Real u, v, w;
	Bool xLtEq1, xGtEqNeg1, xLtEq1_and_GtEqNeg1;
	static int doneInit = 0;

	if (!doneInit) {
		registerForceFunc(atanInside, "atanInside", 2);
		registerForceFunc(atanPositive, "atanPositive", 2);
		registerForceFunc(atanNegative, "atanNegative", 2);
		doneInit++;
	}

	xLtEq1 = ltEq_R_QInt(x, 1, 1);
	xGtEqNeg1 = gtEq_R_QInt(x, -1, 1);
	xLtEq1_and_GtEqNeg1 = and_B_B(xLtEq1, xGtEqNeg1);
		
	u = (Real) allocCls(atanInside, (void *) x);
	u->cls.tag.isSigned = TRUE;
	v = (Real) allocCls(atanPositive, (void *) x);
	v->cls.tag.isSigned = TRUE;
	w = (Real) allocCls(atanNegative, (void *) x);
	w->cls.tag.isSigned = TRUE;
	
	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(u, x);
		newEdgeToOnlyChild(v, x);
		newEdgeToOnlyChild(w, x);
		endGraphUpdate();
	}

	/*
	 * The order of the tests in the alt is not semantically
	 * significant. The tests are applied in order, so there is a
	 * very modest performance improvement by putting the -1,1 tests
	 * before the 999/1000,-999/1000 tests.  Also there is a very
	 * small win by doing the negative test before the positive test.
	 */
	return realIf(3,
		xLtEq1_and_GtEqNeg1,			u,
		gtEq_R_QInt(x, 999, 1000),		v,
		ltEq_R_QInt(x, -999, 1000),		w);
}

static TenXY *
nextTensor(Real x, Real y, int n)
{
    return (TenXY *) tensor_Int(x, y, 2*n+1, 1, -1, -(2*n+1),
					2*n+1, 2*n+1, 2*n+1, 2*n+1);
}

static void
atanInside()
{
	Cls *cls, *newCls;
	Real x, atan_x;
	ClsData *data;
	void stdTensorCont();

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	if ((data = (ClsData *) malloc(sizeof(ClsData))) == NULL)
		Error(FATAL, E_INT, "atan_R", "malloc failed");

	data->n = 1;
	data->x = x;
	data->nextTensor = nextTensor;

	newCls = allocCls(stdTensorCont, (void *) data);
	newCls->tag.isSigned = FALSE;

	cls->redirect = tensor_Int(x, (Real) newCls, 1, 1, 1, -1, 0, 1, 0, 1);

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(newCls, x);
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}
}

/*
 * This implements the identity:
 *   x > 0 implies atan(x) = atan(szero * x) + \pi/4
 */
static void
atanPositive()
{
	Cls *cls, *newCls;
	Real x, y;

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	y = atan_R(matrix_Int(x, 1, 1, -1, 1));
	cls->redirect = add_R_R(y, div_R_Int(Pi, 4));
}

/*
 * This implements the identity:
 *   x < 0 implies atan(x) = atan(szero * (sneg^(-1) * x) - \pi/4
 */
static void
atanNegative()
{
	Cls *cls, *newCls;
	Real x, y;

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	y = atan_R(matrix_Int(x, 1, -1, 1, 1));
	cls->redirect = sub_R_R(y, div_R_Int(Pi, 4));
}
