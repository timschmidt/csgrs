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

Real
sqrt_R(Real x)
{
	Bool xLtEq4, xGtEqOneFifth;
	Cls *in, *gtEq3, *ltEqOneQuarter;
	Real lt0;
	void force_To_DigsX_From_Sqrt_TenXY_Entry();
	void sqrtInside();
	void sqrtGtEq3();
	void sqrtLtEqOneQuarter();
	void sqrtLtEqOneQuarterCont();
	void force_To_DigsX_From_Sqrt_TenXY_Entry();
	void force_To_DigsX_From_Sqrt_TenXY_Cont();
	void force_To_DigsX_From_Sqrt_MatX_Entry();
	void force_To_DigsX_From_Sqrt_MatX_Cont();
	void force_To_DigsX_From_Sqrt_Reduce();
	void force_To_MatX_Until_Refining();
	static int doneInit = 0;
	Real sqrt_QZ(mpz_t, mpz_t);

	if (!doneInit) {
		registerForceFunc(force_To_DigsX_From_Sqrt_TenXY_Entry,
								"force_To_DigsX_From_Sqrt_TenXY_Entry", 3);
		registerForceFunc(force_To_DigsX_From_Sqrt_TenXY_Cont,
								"force_To_DigsX_From_Sqrt_TenXY_Cont", 3);
		registerForceFunc(force_To_DigsX_From_Sqrt_MatX_Entry,
								"force_To_DigsX_From_Sqrt_MatX_Entry", 3);
		registerForceFunc(force_To_DigsX_From_Sqrt_MatX_Cont,
								"force_To_DigsX_From_Sqrt_MatX_Cont", 3);
		registerForceFunc(force_To_DigsX_From_Sqrt_Reduce,
								"force_To_DigsX_From_Sqrt_Reduce", 2);
		registerForceFunc(force_To_MatX_Until_Refining,
								"force_To_MatX_Until_Refining", 2);
		registerForceFunc(sqrtInside, "sqrtInside", 2);
		registerForceFunc(sqrtGtEq3, "sqrtGtEq3", 2);
		registerForceFunc(sqrtLtEqOneQuarter, "sqrtLtEqOneQuarter", 2);
		registerForceFunc(sqrtLtEqOneQuarterCont, "sqrtLtEqOneQuarterCont", 2);
		doneInit++;
	}

	if (x->gen.tag.type == VECTOR)
		return sqrt_QZ(x->vec.vec[0], x->vec.vec[1]);

	xLtEq4 = ltEq_R_QInt(x, 4, 1);
	xGtEqOneFifth = gtEq_R_QInt(x, 1, 5);

	in = allocCls(sqrtInside, (void *) x);
	in->tag.isSigned = FALSE;

	gtEq3 = allocCls(sqrtGtEq3, (void *) x);
	gtEq3->tag.isSigned = FALSE;

	ltEqOneQuarter = allocCls(sqrtLtEqOneQuarter, (void *) x);
	ltEqOneQuarter->tag.isSigned = FALSE;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(in, x);
		newEdgeToOnlyChild(gtEq3, x);
		newEdgeToOnlyChild(ltEqOneQuarter, x);
		endGraphUpdate();
	}

	lt0 = realError("(sqrt_R x) and x < 0");

	/*
	 * The order of the tests in the realAlt is not semantically
	 * significant. Note that the sqrt_R is always unsigned since
	 * each case is unsigned.
	 */
	return realIf(4,
		and_B_B(xLtEq4, xGtEqOneFifth),	(Real) in,
		gtEq_R_QInt(x, 3, 1),			(Real) gtEq3,
		and_B_B(ltEq_R_QInt(x, 1, 4), gtEq_R_0(x)),	(Real) ltEqOneQuarter,
		lt_R_0(x),						lt0);
}

void
sqrtInside()
{
	Cls *cls;
	TenXY *tenXY;
	DigsX *digsX;
	void force_To_TenXY_X_Until_Refining();
	void force_To_DigsX_From_Sqrt_TenXY_Entry();
	Real x;

	cls = (Cls *) POP;
	x = (Real) cls->userData;

	digsX = allocDigsX();
	digsX->force = force_To_DigsX_From_Sqrt_TenXY_Entry;

	tenXY = (TenXY *) tensor_Int(x, (Real) digsX, 1, 0, 2, 1, 1, 2, 0, 1);
	tenXY->tag.isSigned = FALSE;

	digsX->x = (Real) tenXY;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(digsX, tenXY);
		endGraphUpdate();
	}
	
	/*
	 * We must still absorb the sign (if any) and enough information 
	 * to ensure the tensor is refining. Note that, if we get to
	 * this point, then we know the argument is within the interval
	 * 1/5 >= x <= 4.  I claim that the n'th tensor for n >= 2
	 * can always be made refining by absorbing
	 * information from the left.
	 */
	PUSH_2(force_To_TenXY_X_Until_Refining, tenXY);

	if (tenXY->x->gen.tag.isSigned) 
		PUSH_2(tenXY->forceX, tenXY);

	cls->redirect = (Real) digsX;
	cls->userData = NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(cls, x);
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}
}

void
sqrtGtEq3()
{
	Cls *cls;
	Real w, x;

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	w = div_R_Int(x, 4);
	w = sqrt_R(w);
	cls->redirect = mul_R_Int(w, 2);
	cls->userData = NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(cls, x);
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}
}

void
sqrtLtEqOneQuarter()
{
	Cls *cls;
	Real w, x;

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	w = mul_R_Int(x, 4);
	w = sqrt_R(w);
	cls->redirect = div_R_Int(w, 2);
	cls->userData = NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(cls, x);
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}
}

#ifdef LATER
void
sqrtLtEqOneQuarter()
{
	Cls *cls, *newCls;
	void sqrtLtEqOneQuarterCont();
	Real x;

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	newCls = allocCls(sqrtLtEqOneQuarterCont, (void *) x);
	newCls->tag.isSigned = FALSE;
	cls->redirect = matrix_Int((Real) newCls, 0, 1, 1, 2);
	cls->userData = NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(newCls, x);
		deleteOnlyEdge(cls, x);
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}
}
#endif

void
sqrtLtEqOneQuarterCont()
{
	Cls *cls;
	Real w, x;
	void force_To_MatX_Until_Refining();

	cls = (Cls *) POP;
	x = (Real) cls->userData;
	
	/* w = matrix_Int(x, 0, 1, 1, 0);	 reciprocal of x */
	w = mul_R_Int(x, 4);
	w = sqrt_R(w);
	/* cls->redirect = matrix_Int(w, 1, 0, -2, 1); */
	cls->redirect = w;
	cls->redirect->gen.tag.isSigned = FALSE;
	cls->userData = NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(cls, x);
		drawEqEdge(cls, cls->redirect);
		endGraphUpdate();
	}

	PUSH_2(force_To_MatX_Until_Refining, cls->redirect);
}
	
void
force_To_DigsX_From_Sqrt_TenXY_Entry()
{
	DigsX *digsX;
	TenXY *tenXY;
	int digitsNeeded;
	void force_To_DigsX_From_Sqrt_TenXY_Cont();
	void force_To_DigsX_From_DigsX_Entry();
	void force_To_DigsX_From_Sqrt_Reduce();

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	tenXY = (TenXY *) digsX->x;

	/*
	 * The current strategy is that when forced, we create a list
	 * of DigsX structures rather than a single struct with the
	 * requested number of digits. So, after we are done emitting,
	 * we arrange to reduce the list at the end.
	 *
	 * A better strategy would be to allocate two DigsX structures at the
	 * very start. The second one would be private and only visible to the sqrt
	 * closure. Then the cycle would be:
	 *	emit from tensor into DigsX-2
	 *	absorb DigsX-2 digits into tensor
	 *	absorb DigsX-2 digits into DigsX-1
	 *	clear DigsX-2 of all digits.
	 *
	 * This would avoid allocating a chain of DigsX structures and the
	 * need for reducing at the end.
	PUSH_3(force_To_DigsX_From_DigsX_Entry, digsX, digitsNeeded);
	 */
	PUSH_2(force_To_DigsX_From_Sqrt_Reduce, digsX);
	PUSH_3(force_To_DigsX_From_Sqrt_TenXY_Cont, digsX, digitsNeeded);
}

void
force_To_DigsX_From_Sqrt_TenXY_Cont()
{
	DigsX *digsX;
	TenXY *tenXY;
	int digitsNeeded;
	int nX;
	int digitsEmitted, bitsShifted;
	int epsDelTensorX(Tensor, int);
	bool emitDigitFromTensor(Tensor, Digit *);

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	tenXY = (TenXY *) digsX->x;

	digitsEmitted = emitDigits(digsX,
								(edf) emitDigitFromTensor,
								(void *) tenXY->ten,
								digitsNeeded);

	if (digitsEmitted > 0)
		bitsShifted = normalizeTensor(tenXY->ten);

	if (TRACE) {
		debugp("force_To_DigsX_From_Sqrt_TenXY_Cont",
				"%x %x emitted=%d shifted=%d\n",
				(unsigned) digsX,
				(unsigned) tenXY,
				digitsEmitted,
				bitsShifted);
	}

	digitsNeeded -= digitsEmitted;

	if (digsX->count > 0)
		newDigsX(digsX);

	if (digitsNeeded <= 0)
		return;

	/*
	 * So now we emitted what we can but still need more. First arrange
	 * to come back and try to emit again after forcing the necessary
	 * number of digits from the the argument.
	 */
	if (digsX->count > 0)
		PUSH_3(force_To_DigsX_From_Sqrt_TenXY_Cont, digsX->x, digitsNeeded);
	else
		PUSH_3(force_To_DigsX_From_Sqrt_TenXY_Cont, digsX, digitsNeeded);

	/*
	 * Now absorb everything emitted into the tensor.
	 */
	absorbDigsXIntoTenXY_Y(tenXY);

	nX = epsDelTensorX(tenXY->ten, digitsNeeded);

	if (TRACE) {
		debugp("force_To_DigsX_From_Sqrt_TenXY_Cont",
				"%x %x nX=%d\n",
				(unsigned) digsX,
				(unsigned) tenXY,
				nX);
	}

	if (nX > 0)
		PUSH_3(tenXY->forceX, tenXY, nX);
	else
		PUSH_3(tenXY->forceX, tenXY, 1);
}

void
force_To_DigsX_From_Sqrt_MatX_Entry()
{
	DigsX *digsX;
	MatX *matX;
	int digitsNeeded;
	void force_To_DigsX_From_Sqrt_MatX_Cont();
	void force_To_DigsX_From_DigsX_Entry();
	void force_To_DigsX_From_Sqrt_Reduce();

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	matX = (MatX *) digsX->x;

/*
	PUSH_3(force_To_DigsX_From_DigsX_Entry, digsX, digitsNeeded);
*/
	PUSH_2(force_To_DigsX_From_Sqrt_Reduce, digsX);
	PUSH_3(force_To_DigsX_From_Sqrt_MatX_Cont, digsX, digitsNeeded);
}

/*
 * This is not used at this stage. This will be used when reduction is
 * performed after the tensor is first allocated and when the tensor
 * reduces to a matrix.
 */
void
force_To_DigsX_From_Sqrt_MatX_Cont()
{
	DigsX *digsX;
	MatX *matX;
	int digitsNeeded;
	int nX;
	int digitsEmitted, bitsShifted;
	bool emitDigitFromMatrix(Matrix, Digit *);

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	matX = (MatX *) digsX->x;

	digitsEmitted = emitDigits(digsX,
								(edf) emitDigitFromMatrix,
								(void *) matX->mat,
								digitsNeeded);

	if (digitsEmitted > 0)
		bitsShifted = normalizeMatrix(matX->mat);

	if (TRACE) {
		debugp("force_To_DigsX_From_Sqrt_MatX_Cont",
				"%x %x emitted=%d shifted=%d\n",
				(unsigned) digsX,
				(unsigned) matX,
				digitsEmitted,
				bitsShifted);
	}

	digitsNeeded -= digitsEmitted;

	if (digsX->count > 0)
		newDigsX(digsX);

	if (digitsNeeded <= 0)
		return;

	/*
	 * Now absorb everything emitted into the matrix.
	 */
	absorbDigsXIntoMatX(matX);

	/*
	 * So now we emitted what we can but still need more. First arrange
	 * to come back and try to emit again after forcing the necessary
	 * number of digits from the the argument.
	 */
	if (digsX->count > 0)
		PUSH_3(force_To_DigsX_From_Sqrt_MatX_Cont, digsX->x, digitsNeeded);
	else
		PUSH_3(force_To_DigsX_From_Sqrt_MatX_Cont, digsX, digitsNeeded);
}

/*
 * In some cases when we generate a matrix which is not refining, but
 * where we no that the argument of the matrix is constrained in such a
 * way that after a finite amount of absorption, it will become refining.
 * What we do is force information from the argument until the matrix is
 * refining.
 */
void
force_To_MatX_Until_Refining()
{
	MatX *matX;
	int sgn;

	matX = (MatX *) POP;

	if (matX->tag.type != MATX)
		return;

	sgn = matrixSign(matX->mat);
	
	if (sgn > 0)		/* matrix is refining and entries positive */
		return;

	if (sgn < 0) {	  /* matrix is refining and entries negative */
		negateMatrix(matX->mat);
		return;
	}

	PUSH_2(force_To_MatX_Until_Refining, matX);
	PUSH_3(matX->force, matX, 1);
}

void
force_To_DigsX_From_Sqrt_Reduce()
{
	DigsX *digsX;

	digsX = (DigsX *) POP;
	reduceDigsXList(digsX);
}
