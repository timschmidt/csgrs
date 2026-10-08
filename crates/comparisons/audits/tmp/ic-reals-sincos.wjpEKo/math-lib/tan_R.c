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
static void tanInside();
static void tanOutside();

Real
tan_R(Real x)
{
	Real u, v;
	Bool xLtEq1, xGtEqNeg1, xLtEq1_and_GtEqNeg1;
	static int doneInit = 0;

	if (!doneInit) {
		registerForceFunc(tanInside, "tanInside", 2);
		registerForceFunc(tanOutside, "tanOutside", 2);
		doneInit++;
	}

	/*
	if (x->gen.tag.type == VECTOR)
		return tan_QZ(x->vec.vec[0], x->vec.vec[1]);
	*/

	xLtEq1 = ltEq_R_QInt(x, 1, 1);
	xGtEqNeg1 = gtEq_R_QInt(x, -1, 1);
	xLtEq1_and_GtEqNeg1 = and_B_B(xLtEq1, xGtEqNeg1);
		
	u = (Real) allocCls(tanInside, (void *) x);
	u->cls.tag.isSigned = TRUE;
	v = (Real) allocCls(tanOutside, (void *) x);
	v->cls.tag.isSigned = TRUE;
	
	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(u, x);
		newEdgeToOnlyChild(v, x);
		endGraphUpdate();
	}

	/*
	 * The order of the tests in the alt is not semantically
	 * significant. The tests are applied in order, so there is a
	 * very modest performance improvement by putting the -1,1 tests
	 * before the 999/1000,-999/1000 tests.  Also there is a very
	 * small win by doing the negative test before the positive test.
	 */
	return realIf(4,
		not_B(xLtEq1_and_GtEqNeg1),   v,
		xLtEq1_and_GtEqNeg1,			u,
		gtEq_R_QInt(x, 999, 1000),		v,
		ltEq_R_QInt(x, -999, 1000),		v);
}

static TenXY *
nextTensor(Real x, Real y, int n)
{
    return (TenXY *) tensor_Int(x, y, 0, 2, -2, 0, 2*n+1, 2*n+1, 2*n+1, 2*n+1);
}

static void
tanInside()
{
	Cls *cls, *newCls;
	Real x, tan_x;
	ClsData *data;
	void stdTensorCont();

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	if ((data = (ClsData *) malloc(sizeof(ClsData))) == NULL)
		Error(FATAL, E_INT, "tan_R", "malloc failed");

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

static void
tanOutside()
{
	Cls *cls;
	Real w, x;

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	x = div_R_Int(x, 2);

	w = tan_R(x);

	cls->redirect = tensor_Int(w, w, 0, -1, 1, 0, 1, 0, 0, 1);

	if (DAVINCI) {
		beginGraphUpdate();
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}
}
