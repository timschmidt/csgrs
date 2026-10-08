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

void log2Cont();
void logInside();
void logLow();
void logHigh();

/*
 * This diverges when the argument is zero.
 */
Real
log_R(Real x)
{
	Bool xGtEq0, xLtEq2, xGtEqOneHalf, xLtEq2_and_GtEqOneHalf;
	Bool xLtEqOverOneHalf_and_GtEq0;
	Real in, low, high, ltZero;
	static int doneInit = 0;

	if (!doneInit) {
		registerForceFunc(logInside, "logInside", 2);
		registerForceFunc(logLow, "logLow", 2);
		registerForceFunc(logHigh, "logHigh", 2);
		registerForceFunc(log2Cont, "log2Cont", 3);
		doneInit++;
	}

	xGtEq0 = gtEq_R_0(x);
	xLtEq2 = ltEq_R_QInt(x, 2, 1);
	xGtEqOneHalf = gtEq_R_QInt(x, 1, 2);
	xLtEq2_and_GtEqOneHalf = and_B_B(xLtEq2, xGtEqOneHalf);
	xLtEqOverOneHalf_and_GtEq0 = and_B_B(ltEq_R_QInt(x, 1001, 2000), xGtEq0);

	in = (Real) allocCls(logInside, (void *) x);
	in->cls.tag.isSigned = TRUE;
	low = (Real) allocCls(logLow, (void *) x);
	low->cls.tag.isSigned = TRUE;
	high = (Real) allocCls(logHigh, (void *) x);
	high->cls.tag.isSigned = TRUE;
	ltZero = realError("(log_R x) and x < 0");

	return realIf(4,
			xLtEq2_and_GtEqOneHalf,			in,
			gtEq_R_QInt(x, 1999, 1000),		high,
			xLtEqOverOneHalf_and_GtEq0,		low,
			not_B(xGtEq0),				ltZero);
}

static TenXY *nextTensor(Real, Real, int);

void
logInside()
{
	Cls *cls, *newCls;
	ClsData *data;
	Real x;
	void stdTensorCont();

	cls = (Cls *) POP;
	x = (Real) cls->userData;

	if ((data = (ClsData *) malloc(sizeof(ClsData))) == NULL)
		Error(FATAL, E_INT, "logInside", "malloc failed");

	data->n = 1;
	data->x = x;
	data->nextTensor = nextTensor;

	newCls = allocCls(stdTensorCont, (void *) data);
	newCls->tag.isSigned = FALSE;

	cls->redirect = tensor_Int(x, (Real) newCls, 1, 0, 1, 1, -1, 1, -1, 0);

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(newCls, x);
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}
}

void
logHigh()
{
	Cls *cls, *newCls;
	Real w, x;

	cls = (Cls *) POP;
	x = (Real) cls->userData;

	w = div_R_Int(x, 2);
	w = log_R(w);
	newCls = allocCls(log2Cont, (void *) 0);
	newCls->tag.isSigned = FALSE;
	w = add_R_R(w, (Real) newCls);
	w->tenXY.forceY = log2Cont; /* see the note below */

	/* now guard tensor to prevent it being copied */
	cls->redirect = (Real) allocSignX(w, SIGN_UNKN);
}

void
logLow()
{
	Cls *cls, *newCls;
	Real w, x;

	cls = (Cls *) POP;
	x = (Real) cls->userData;

	w = mul_R_Int(x, 2);
	w = log_R(w);
	newCls = allocCls(log2Cont, (void *) 0);
	newCls->tag.isSigned = FALSE;
	w = sub_R_R(w, (Real) newCls);
	w->tenXY.forceY = log2Cont; /* see the note below */

	/* now guard tensor to prevent it being copied */
	cls->redirect = (Real) allocSignX(w, SIGN_UNKN);
}

/*
 * This closure works a little different from most. Most simply create
 * an lft and set the redirect field of the closure to point to the new
 * heap object. In this case, we can do a bit better. This continuation
 * represents (log 2). One option would be to follow the usual strategy
 * and to arrange for (log 2) to be shared whenever possible. However,
 * we know that this is used internally to log_R only and only in
 * those circumstances when the argument falls outside the working range
 * of the log tensor chain. Moreover, we know that the consumer of this
 * real is either an addition or substraction tensor. Whatever it is,
 * it is certain to be a tensor. The other argument of the tensor will
 * reduce at most to a matrix and hence the tensor itself will never
 * reduce.
 *
 * This property allows us to adopt a different strategy. Rather than create
 * the next matrix in the (log 2) chain, we put the matrix directly into
 * the consuming tensor. This avoids creating garbage in the stack and
 * avoids a separate reduction step.
 *
 * There is a slighlt better scheme than this. Rather than plunk in
 * each successive
 * matrix into the tensor (each providing 4 digits), it would be better
 * to accumulate a sequence of matrices (up to the capacity of a small matrix)
 * and then plunk it into the tensor. This would avoid using bignum stuff until
 * it becomes necessary. Perhaps I'll do this later.
 *
 * For this it is useful to bear in mind that when a sequence of matrices
 * starts from n = 0, then the largest entry is always d. When the sequence
 * starts with n > 0, then the largest entry is c. To decide if there
 * is going to be an overflow, it suffices to check if c (for example)
 * will overflow.
 */
static void nextMatrix(Tensor, int);

void
log2Cont()
{
	TenXY *tenXY;
	Cls *cls;
	int digitsNeeded;
	int n;

	tenXY = (TenXY *) POP;
	digitsNeeded = POP;
	cls = (Cls *) tenXY->y;

	n = (int) cls->userData;

	while (digitsNeeded > 0) {
		nextMatrix(tenXY->ten, n);
		if (n == 0)
			digitsNeeded -= 2;
		else
			digitsNeeded -= 4;
		n += 1;
	}
	cls->userData = (void *) n;
}

/*
 * We set a rather arbitrary (but large) limit on the value of n
 * for log 2. On a 32 bit machine it is 536,870,911. 
 * Larger than this and the matrix entries are no longer small.
 */
static void
nextMatrix(Tensor ten, int n)
{
	SmallMatrix smallMat;

	if (n == 0) {
		smallMat[0][0] = 1;
		smallMat[0][1] = 1;
		smallMat[1][0] = 1;
		smallMat[1][1] = 2;
	}
	else {
		if (n <= (MAXINT - 2) / 4) {
			smallMat[0][0] = 3 * n + 1;
			smallMat[0][1] = 2 * n + 1;
			smallMat[1][0] = 4 * n + 2;
			smallMat[1][1] = 3 * n + 2;
		}
		else
			Error(FATAL, E_INT, "nextMatrix (log2)",
									"n > %d", (MAXINT - 2) / 4);
	}
	multVectorPairTimesSmallMatrix(ten[0], ten[1], smallMat);
	multVectorPairTimesSmallMatrix(ten[2], ten[3], smallMat);
	normalizeTensor(ten);
}

static TenXY *
nextTensor(Real x, Real y, int n)
{
	return (TenXY *) tensor_Int(x, y, n, 0, 2*n+1, n+1, n+1, 2*n+1, 0, n);
}
