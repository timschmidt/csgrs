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
 * Functions for allocating and manipulating tensor LFTs in the heap.
 */

void setTenXY_Y_MethodSigned(TenXY *);
void setTenXY_X_MethodSigned(TenXY *);
void setTenXY_Y_MethodUnsigned(TenXY *);
void setTenXY_X_MethodUnsigned(TenXY *);

void absorbDigsXIntoTenXY_X(TenXY *);
void absorbDigsXIntoTenXY_Y(TenXY *);
void createUnsignedStreamForTenXY(TenXY *);

TenXY *
allocTenXY()
{
	TenXY *tenXY;

	if ((tenXY = (TenXY *) malloc (sizeof(TenXY))) == NULL) 
		Error(FATAL, E_INT, "allocTenXY", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(tenXY);

	tenXY->tag.type = TENXY;
	tenXY->tag.dumped = FALSE;
	tenXY->tag.isSigned = FALSE;
	tenXY->strm = (Real) NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(tenXY, TENXY);
		endGraphUpdate();
	}

	return tenXY;
}

/*
 * The following function allocates a tensor, fills in the tensor 
 * entries and sets the arguments to the tensors. 
 *
 * There is code here for eagerly reducing a tensor against vector and
 * matrix arguments. Eager reduction has the advantage that it reduces
 * the amount of garbage created in the heap. For the time being, however,
 * reduction is disabled since there are functions which call this function
 * and assume the result they get back is genuinely a tensor. With reduction
 * it may return a tensor, matrix or vector.
 */

Real
tensor_Int(Real x, Real y,
	int a, int b, int c, int d,
	int e, int f, int g, int h)
{
	TenXY *tenXY;

#ifdef LATER
	/*
	 * If either or both of the arguments are vectors, then there is room
	 * for reduction.
	 */
	if (x->gen.tag.type == VECTOR || y->gen.tag.type == VECTOR) {
		mpz_init_set_si(bigTmpTen[0][0], a);
		mpz_init_set_si(bigTmpTen[0][1], b);
		mpz_init_set_si(bigTmpTen[1][0], c);
		mpz_init_set_si(bigTmpTen[1][1], d);
		mpz_init_set_si(bigTmpTen[2][0], e);
		mpz_init_set_si(bigTmpTen[2][1], f);
		mpz_init_set_si(bigTmpTen[3][0], g);
		mpz_init_set_si(bigTmpTen[3][1], h);

		if (x->gen.tag.type == VECTOR) {
			multVectorPairTimesVector(bigTmpTen[0], bigTmpTen[2], x->vec.vec);
			multVectorPairTimesVector(bigTmpTen[1], bigTmpTen[3], x->vec.vec);
		}

		if (y->gen.tag.type == VECTOR) {
			multVectorPairTimesVector(bigTmpTen[0], bigTmpTen[1], y->vec.vec);
			multVectorPairTimesVector(bigTmpTen[2], bigTmpTen[3], y->vec.vec);
		}

		if (x->gen.tag.type == VECTOR) {
			if (y->gen.tag.type == VECTOR)
				return vector_Z(bigTmpTen[0][0], bigTmpTen[0][1]);
			else
				return matrix_Z(y, bigTmpTen[0][0], bigTmpTen[0][1],
									bigTmpTen[1][0], bigTmpTen[1][1]);
		}
		else { 		/* then y->gen.tag.type == VECTOR */
			return matrix_Z(x, bigTmpTen[0][0], bigTmpTen[0][1],
									bigTmpTen[2][0], bigTmpTen[2][1]);
		}
	}
#endif

	/*
	 * So now we know we will end up with a tensor, so we allocate one.
	 */
	tenXY = allocTenXY();

	mpz_init_set_si(tenXY->ten[0][0], a);
	mpz_init_set_si(tenXY->ten[0][1], b);
	mpz_init_set_si(tenXY->ten[1][0], c);
	mpz_init_set_si(tenXY->ten[1][1], d);
	mpz_init_set_si(tenXY->ten[2][0], e);
	mpz_init_set_si(tenXY->ten[2][1], f);
	mpz_init_set_si(tenXY->ten[3][0], g);
	mpz_init_set_si(tenXY->ten[3][1], h);

	/* remove powers of 2 from the tensor */
	normalizeTensor(tenXY->ten);

	/* make the tensor positive if it is negative (no entries > 0) */
	if (tensorSign(tenXY->ten) < 0)
		negateTensor(tenXY->ten);

	tenXY->x = x;
	tenXY->y = y;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToXChild(tenXY, x);
		newEdgeToYChild(tenXY, y);
		endGraphUpdate();
	}

#ifdef LATER
	/*
	 * Now we eagerly consume any matrix which follows. 
	 */
	if (x->gen.tag.type == MATX) {
		multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[2], x->matX.mat);
		multVectorPairTimesMatrix(tenXY->ten[1], tenXY->ten[3], x->matX.mat);
		if (DAVINCI) {
			beginGraphUpdate();
			deleteEdgeToXChild(tenXY, tenXY->x);
			newEdgeToXChild(tenXY, x->matX.x);
			endGraphUpdate();
		}
		tenXY->x = x->matX.x;
	}

	/*
	 * Do the same as the above on the y side now.
	 */
	if (y->gen.tag.type == MATX) {
		multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[1], y->matX.mat);
		multVectorPairTimesMatrix(tenXY->ten[2], tenXY->ten[3], y->matX.mat);
		if (DAVINCI) {
			beginGraphUpdate();
			deleteEdgeToYChild(tenXY, tenXY->y);
			newEdgeToYChild(tenXY, y->matX.x);
			endGraphUpdate();
		}
		tenXY->y = y->matX.x;
	}
#endif

	/*
	 * A TenXY is signed if the there are entries in the tensor of different
	 * signs or if either argument to the TenXY is signed.
	 */
	if (tenXY->x->gen.tag.isSigned
			|| tenXY->y->gen.tag.isSigned
			|| tensorSign(tenXY->ten) == 0)
		tenXY->tag.isSigned = TRUE;

	if (tenXY->x->gen.tag.isSigned)
		setTenXY_X_MethodSigned(tenXY);
	else
		setTenXY_X_MethodUnsigned(tenXY);

	if (tenXY->y->gen.tag.isSigned)
		setTenXY_Y_MethodSigned(tenXY);
	else
		setTenXY_Y_MethodUnsigned(tenXY);

	return (Real) tenXY;
}

Real
tensor_Z(Real x, Real y,
	mpz_t a, mpz_t b, mpz_t c, mpz_t d,
	mpz_t e, mpz_t f, mpz_t g, mpz_t h)
{
	TenXY *tenXY;

	/*
	 * So now we know we will end up with a tensor, so we allocate one.
	 */
	tenXY = allocTenXY();

	mpz_init_set(tenXY->ten[0][0], a);
	mpz_init_set(tenXY->ten[0][1], b);
	mpz_init_set(tenXY->ten[1][0], c);
	mpz_init_set(tenXY->ten[1][1], d);
	mpz_init_set(tenXY->ten[2][0], e);
	mpz_init_set(tenXY->ten[2][1], f);
	mpz_init_set(tenXY->ten[3][0], g);
	mpz_init_set(tenXY->ten[3][1], h);

	/* remove powers of 2 from the tensor */
	normalizeTensor(tenXY->ten);

	/* make the tensor positive if it is negative (no entries > 0) */
	if (tensorSign(tenXY->ten) < 0)
		negateTensor(tenXY->ten);

	tenXY->x = x;
	tenXY->y = y;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToXChild(tenXY, x);
		newEdgeToYChild(tenXY, y);
		endGraphUpdate();
	}

	/*
	 * A TenXY is signed if the there are entries in the tensor of different
	 * signs or if either argument to the TenXY is signed.
	 */
	if (tenXY->x->gen.tag.isSigned
			|| tenXY->y->gen.tag.isSigned
			|| tensorSign(tenXY->ten) == 0)
		tenXY->tag.isSigned = TRUE;

	if (tenXY->x->gen.tag.isSigned)
		setTenXY_X_MethodSigned(tenXY);
	else
		setTenXY_X_MethodUnsigned(tenXY);

	if (tenXY->y->gen.tag.isSigned)
		setTenXY_Y_MethodSigned(tenXY);
	else
		setTenXY_Y_MethodUnsigned(tenXY);

	return (Real) tenXY;
}

void
force_To_TenXY_X_From_DigsX_Entry()
{
	TenXY *tenXY;
	DigsX *digsX;
	int digitsNeeded;
	void force_To_TenXY_X_From_DigsX_Cont();

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;
	digsX = (DigsX *) tenXY->x;

	PUSH_3(force_To_TenXY_X_From_DigsX_Cont, tenXY, digitsNeeded);

	/*
	 * See if the source has the number of digits we need. If not,
	 * then force the remaining.
	 */
	if (digsX->count < digitsNeeded)
		PUSH_3(digsX->force, digsX, digitsNeeded - digsX->count);
}

void
force_To_TenXY_X_From_DigsX_Cont()
{
	TenXY *tenXY;
	DigsX *digsX;
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;
	digsX = (DigsX *) tenXY->x;

	absorbDigsXIntoTenXY_X(tenXY);
}

void
force_To_TenXY_X_From_DigsX_Signed()
{
	TenXY *tenXY;
	void force_To_TenXY_X_From_DigsX_Entry();

	tenXY = (TenXY *) POP;
	tenXY->forceX = force_To_TenXY_X_From_DigsX_Entry;
}

void
force_To_TenXY_Y_From_DigsX_Signed()
{
	TenXY *tenXY;
	void force_To_TenXY_Y_From_DigsX_Entry();

	tenXY = (TenXY *) POP;
	tenXY->forceY = force_To_TenXY_Y_From_DigsX_Entry;
}

/*
 * When a tensor absorbs a vector, the tensor reduces to a matrix. We do this
 * in place. That is, we overwrite the tensor with a matrix. We do this
 * since the tensor might be shared. It might waste a bit of space
 * but the garbage collector will deal with this (eventually).
 *
 * Note that the node drawn by daVinci still has type tensor but this
 * doesn't matter.
 */
void
force_To_TenXY_X_From_Vec()
{
	TenXY *tenXY;
	MatX *matX;
	mpz_t a, b, c, d;	/* temporary storage while we clobber the TenXY */
	Real strm;			/* temporary storage again */
	int digitsNeeded;
	int totalEmitted;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	multVectorPairTimesVector(tenXY->ten[0], tenXY->ten[2], tenXY->x->vec.vec);
	multVectorPairTimesVector(tenXY->ten[1], tenXY->ten[3], tenXY->x->vec.vec);

	a[0] = tenXY->ten[0][0][0];
	b[0] = tenXY->ten[0][1][0];
	c[0] = tenXY->ten[1][0][0];
	d[0] = tenXY->ten[1][1][0];
	strm = tenXY->strm;
	totalEmitted = tenXY->totalEmitted;

	mpz_clear(tenXY->ten[2][0]);
	mpz_clear(tenXY->ten[2][1]);
	mpz_clear(tenXY->ten[3][0]);
	mpz_clear(tenXY->ten[3][1]);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		deleteEdgeToYChild(tenXY, tenXY->y);
		endGraphUpdate();
	}

	matX = (MatX *) tenXY;
	matX->tag.type = MATX;
	matX->x = tenXY->y;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(matX, matX->x);
		endGraphUpdate();
	}

	matX->mat[0][0][0] = a[0];
	matX->mat[0][1][0] = b[0];
	matX->mat[1][0][0] = c[0];
	matX->mat[1][1][0] = d[0];
	normalizeMatrix(matX->mat);
	matX->strm = strm;
	matX->totalEmitted = totalEmitted;

	setMatXMethodUnsigned(matX);
}

/*
 * When a tensor absorbs a vector, the tensor reduces to a matrix. We do this
 * in place. That is, we overwrite the tensor with a matrix. We do this
 * since the tensor might be shared. It might waste a bit of space
 * but the garbage collector will deal with this (eventually).
 *
 * Note that the node drawn by daVinci still has type tensor but this
 * doesn't matter.
 */
void
force_To_TenXY_Y_From_Vec()
{
	TenXY *tenXY;
	MatX *matX;
	mpz_t a, b, c, d;	/* temporary storage while we clobber the TenXY */
	Real strm;			/* temporary storage again */
	int digitsNeeded;
	int totalEmitted;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	multVectorPairTimesVector(tenXY->ten[0], tenXY->ten[1], tenXY->y->vec.vec);
	multVectorPairTimesVector(tenXY->ten[2], tenXY->ten[3], tenXY->y->vec.vec);

	a[0] = tenXY->ten[0][0][0];
	b[0] = tenXY->ten[0][1][0];
	c[0] = tenXY->ten[2][0][0];
	d[0] = tenXY->ten[2][1][0];
	strm = tenXY->strm;
	totalEmitted = tenXY->totalEmitted;

	mpz_clear(tenXY->ten[1][0]);
	mpz_clear(tenXY->ten[1][1]);
	mpz_clear(tenXY->ten[3][0]);
	mpz_clear(tenXY->ten[3][1]);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		deleteEdgeToYChild(tenXY, tenXY->y);
		endGraphUpdate();
	}

	matX = (MatX *) tenXY;
	matX->tag.type = MATX;
	matX->x = tenXY->x;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(matX, matX->x);
		endGraphUpdate();
	}

	matX->mat[0][0][0] = a[0];
	matX->mat[0][1][0] = b[0];
	matX->mat[1][0][0] = c[0];
	matX->mat[1][1][0] = d[0];
	normalizeMatrix(matX->mat);
	matX->strm = strm;
	matX->totalEmitted = totalEmitted;

	setMatXMethodUnsigned(matX);
}

void
force_To_TenXY_X_From_MatX()
{
	TenXY *tenXY;
	int digitsNeeded;
	void force_To_TenXY_X_From_Vec();

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (tenXY->x->gen.tag.type == VECTOR) {
		PUSH_3(force_To_TenXY_X_From_Vec, tenXY, digitsNeeded);
		return;
	}

	multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[2], tenXY->x->matX.mat);
	multVectorPairTimesMatrix(tenXY->ten[1], tenXY->ten[3], tenXY->x->matX.mat);
	normalizeTensor(tenXY->ten);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->matX.x);
		endGraphUpdate();
	}
	tenXY->x = tenXY->x->matX.x;
	setTenXY_X_MethodUnsigned(tenXY);
}

void
force_To_TenXY_Y_From_MatX()
{
	TenXY *tenXY;
	int digitsNeeded;
	void force_To_TenXY_Y_From_Vec();

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (tenXY->y->gen.tag.type == VECTOR) {
		PUSH_3(force_To_TenXY_Y_From_Vec, tenXY, digitsNeeded);
		return;
	}

	multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[1], tenXY->y->matX.mat);
	multVectorPairTimesMatrix(tenXY->ten[2], tenXY->ten[3], tenXY->y->matX.mat);
	normalizeTensor(tenXY->ten);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->matX.x);
		endGraphUpdate();
	}
	tenXY->y = tenXY->y->matX.x;
	setTenXY_Y_MethodUnsigned(tenXY);
}

/*
 * This forces information (ie digits, not a sign) from one tensor into 
 * the x argument of another. It is assumed that the argument is unsigned,
 * otherwise it would already have changed into a SignX or DigsX.
 */
void
force_To_TenXY_X_From_TenXY()
{
	TenXY *tenXY;
	int digitsNeeded;
	void force_To_TenXY_X_From_DigsX_Entry();
	void force_To_TenXY_X_From_MatX();

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (tenXY->x->gen.tag.type != TENXY) {
		PUSH_3(force_To_TenXY_X_From_MatX, tenXY, digitsNeeded);
		return;
	}

	createUnsignedStreamForTenXY((TenXY *) tenXY->x);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->tenXY.strm);
		endGraphUpdate();
	}

	tenXY->x = tenXY->x->tenXY.strm;
	tenXY->forceX = force_To_TenXY_X_From_DigsX_Entry;
	PUSH_3(tenXY->forceX, tenXY, digitsNeeded);
}

/*
 * This forces information (ie digits, not a sign) from one tensor into 
 * the y argument of another. It is assumed that the argument is unsigned,
 * otherwise it would already have changed into a SignX or DigsX.
 */
void
force_To_TenXY_Y_From_TenXY()
{
	TenXY *tenXY;
	int digitsNeeded;
	void force_To_TenXY_Y_From_DigsX_Entry();
	void force_To_TenXY_Y_From_MatX();

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (tenXY->y->gen.tag.type != TENXY) {
		PUSH_3(force_To_TenXY_Y_From_MatX, tenXY, digitsNeeded);
		return;
	}

	createUnsignedStreamForTenXY((TenXY *) tenXY->y);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->tenXY.strm);
		endGraphUpdate();
	}

	tenXY->y = tenXY->y->tenXY.strm;
	tenXY->forceY = force_To_TenXY_Y_From_DigsX_Entry;
	PUSH_3(tenXY->forceY, tenXY, digitsNeeded);
}

void
force_To_TenXY_Y_From_DigsX_Entry()
{
	TenXY *tenXY;
	DigsX *digsX;
	int digitsNeeded;
	void force_To_TenXY_Y_From_DigsX_Cont();

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;
	digsX = (DigsX *) tenXY->y;

	PUSH_3(force_To_TenXY_Y_From_DigsX_Cont, tenXY, digitsNeeded);

	/*
	 * See if the source has the number of digits we need. If not,
	 * then force the remaining.
	 */
	if (digsX->count < digitsNeeded)
		PUSH_3(digsX->force, digsX, digitsNeeded - digsX->count);
}

void
force_To_TenXY_Y_From_DigsX_Cont()
{
	TenXY *tenXY;
	DigsX *digsX;
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;
	digsX = (DigsX *) tenXY->y;

	absorbDigsXIntoTenXY_Y(tenXY);
}

/*
 * In some cases when we generate a chain of tensors, the tensors
 * themselves are not refining. What we have to do is force information
 * from the argument until the tensor is refining.
 *
 * If the tensor is not refining, we force a digit from it's argument
 * and push a continuation to check again.
 */
void
force_To_TenXY_X_Until_Refining()
{
	TenXY *tenXY;
	int sgn;

	tenXY = (TenXY *) POP;

	if (tenXY->tag.type != TENXY)
		return;

	sgn = tensorSign(tenXY->ten);
	
	if (sgn > 0)		/* tensor is refining and entries positive */
		return;

	if (sgn < 0) {		/* tensor is refining and entries negative */
		negateTensor(tenXY->ten);
		return;
	}

	PUSH_2(force_To_TenXY_X_Until_Refining, tenXY);
	PUSH_3(tenXY->forceX, tenXY, defaultForceCount);
}

void
force_To_TenXY_X_From_Alt_Entry()
{
	TenXY *tenXY;
	void force_To_Alt_Entry();
	void force_To_TenXY_X_From_Alt_Cont();
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	PUSH_3(force_To_TenXY_X_From_Alt_Cont, tenXY, digitsNeeded);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (tenXY->x->alt.redirect == NULL)
		PUSH_2(force_To_Alt_Entry, tenXY->x);
}

void
force_To_TenXY_X_From_Alt_Cont()
{
	TenXY *tenXY;
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->alt.redirect);
		endGraphUpdate();
	}

	tenXY->x = tenXY->x->alt.redirect;
	setTenXY_X_MethodUnsigned(tenXY);
	PUSH_3(tenXY->forceX, tenXY, digitsNeeded);
}

void
force_To_TenXY_X_From_Cls_Entry()
{
	TenXY *tenXY;
	void force_To_TenXY_X_From_Cls_Cont();
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	PUSH_3(force_To_TenXY_X_From_Cls_Cont, tenXY, digitsNeeded);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to force it.
	 */
	if (tenXY->x->cls.redirect == NULL)
		PUSH_2(tenXY->x->cls.force, tenXY->x);
}

void
force_To_TenXY_X_From_Cls_Cont()
{
	TenXY *tenXY;
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->cls.redirect);
		endGraphUpdate();
	}

	tenXY->x = tenXY->x->cls.redirect;
	setTenXY_X_MethodUnsigned(tenXY);
	PUSH_3(tenXY->forceX, tenXY, digitsNeeded);
}

void
force_To_TenXY_X_From_Alt_Signed_Entry()
{
	TenXY *tenXY;
	void force_To_Alt_Entry();
	void force_To_TenXY_X_From_Alt_Signed_Cont();

	tenXY = (TenXY *) POP;

	PUSH_2(force_To_TenXY_X_From_Alt_Signed_Cont, tenXY);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (tenXY->x->alt.redirect == NULL)
		PUSH_2(force_To_Alt_Entry, tenXY->x);
}

void
force_To_TenXY_X_From_Alt_Signed_Cont()
{
	TenXY *tenXY;

	tenXY = (TenXY *) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->alt.redirect);
		endGraphUpdate();
	}
	tenXY->x = tenXY->x->alt.redirect;
	setTenXY_X_MethodSigned(tenXY);

	PUSH_2(tenXY->forceX, tenXY);
}

void
force_To_TenXY_X_From_Cls_Signed_Entry()
{
	TenXY *tenXY;
	void force_To_TenXY_X_From_Cls_Signed_Cont();

	tenXY = (TenXY *) POP;

	PUSH_2(force_To_TenXY_X_From_Cls_Signed_Cont, tenXY);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (tenXY->x->cls.redirect == NULL)
		PUSH_2(tenXY->x->cls.force, tenXY->x);
}

void
force_To_TenXY_X_From_Cls_Signed_Cont()
{
	TenXY *tenXY;

	tenXY = (TenXY *) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->cls.redirect);
		endGraphUpdate();
	}
	tenXY->x = tenXY->x->cls.redirect;
	setTenXY_X_MethodSigned(tenXY);

	PUSH_2(tenXY->forceX, tenXY);
}

void
force_To_TenXY_Y_From_Alt_Entry()
{
	TenXY *tenXY;
	void force_To_Alt_Entry();
	void force_To_TenXY_Y_From_Alt_Cont();
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	PUSH_3(force_To_TenXY_Y_From_Alt_Cont, tenXY, digitsNeeded);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (tenXY->y->alt.redirect == NULL)
		PUSH_2(force_To_Alt_Entry, tenXY->y);
}

void
force_To_TenXY_Y_From_Alt_Cont()
{
	TenXY *tenXY;
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->alt.redirect);
		endGraphUpdate();
	}

	tenXY->y = tenXY->y->alt.redirect;
	setTenXY_Y_MethodUnsigned(tenXY);
	PUSH_3(tenXY->forceY, tenXY, digitsNeeded);
}

void
force_To_TenXY_Y_From_Cls_Entry()
{
	TenXY *tenXY;
	void force_To_TenXY_Y_From_Cls_Cont();
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	PUSH_3(force_To_TenXY_Y_From_Cls_Cont, tenXY, digitsNeeded);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to force it.
	 */
	if (tenXY->y->cls.redirect == NULL)
		PUSH_2(tenXY->y->cls.force, tenXY->y);
}

void
force_To_TenXY_Y_From_Cls_Cont()
{
	TenXY *tenXY;
	int digitsNeeded;

	tenXY = (TenXY *) POP;
	digitsNeeded = (int) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->cls.redirect);
		endGraphUpdate();
	}

	tenXY->y = tenXY->y->cls.redirect;
	setTenXY_Y_MethodUnsigned(tenXY);
	PUSH_3(tenXY->forceY, tenXY, digitsNeeded);
}

void
force_To_TenXY_Y_From_Alt_Signed_Entry()
{
	TenXY *tenXY;
	void force_To_Alt_Entry();
	void force_To_TenXY_Y_From_Alt_Signed_Cont();

	tenXY = (TenXY *) POP;

	PUSH_2(force_To_TenXY_Y_From_Alt_Signed_Cont, tenXY);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (tenXY->y->alt.redirect == NULL)
		PUSH_2(force_To_Alt_Entry, tenXY->y);
}

void
force_To_TenXY_Y_From_Alt_Signed_Cont()
{
	TenXY *tenXY;

	tenXY = (TenXY *) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->alt.redirect);
		endGraphUpdate();
	}

	tenXY->y = tenXY->y->alt.redirect;
	setTenXY_Y_MethodSigned(tenXY);
	PUSH_2(tenXY->forceY, tenXY);
}

void
force_To_TenXY_Y_From_Cls_Signed_Entry()
{
	TenXY *tenXY;
	void force_To_TenXY_Y_From_Cls_Signed_Cont();

	tenXY = (TenXY *) POP;

	PUSH_2(force_To_TenXY_Y_From_Cls_Signed_Cont, tenXY);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to force it.
	 */
	if (tenXY->y->cls.redirect == NULL)
		PUSH_2(tenXY->y->cls.force, tenXY->y);
}

void
force_To_TenXY_Y_From_Cls_Signed_Cont()
{
	TenXY *tenXY;

	tenXY = (TenXY *) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->cls.redirect);
		endGraphUpdate();
	}

	tenXY->y = tenXY->y->cls.redirect;
	setTenXY_Y_MethodSigned(tenXY);
	PUSH_2(tenXY->forceY, tenXY);
}

void
absorbSignIntoTenXY_Y(TenXY *tenXY)
{
	absorbSignIntoVectorPair(tenXY->ten[0], tenXY->ten[1],
							tenXY->y->signX.tag.value);
	absorbSignIntoVectorPair(tenXY->ten[2], tenXY->ten[3],
							tenXY->y->signX.tag.value);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->signX.x);
		endGraphUpdate();
	}
	tenXY->y = tenXY->y->signX.x;
	setTenXY_Y_MethodUnsigned(tenXY);
}

void
absorbSignIntoTenXY_X(TenXY *tenXY)
{
	absorbSignIntoVectorPair(tenXY->ten[0], tenXY->ten[2],
							tenXY->x->signX.tag.value);
	absorbSignIntoVectorPair(tenXY->ten[1], tenXY->ten[3],
							tenXY->x->signX.tag.value);
	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->signX.x);
		endGraphUpdate();
	}
	tenXY->x = tenXY->x->signX.x;
	setTenXY_X_MethodUnsigned(tenXY);
}

void
absorbDigsXIntoTenXY_Y(TenXY *tenXY)
{
	DigsX *digsX;
	SmallMatrix smallAccumMat;

	digsX = (DigsX *) tenXY->y;

	/*
	 * Accumulate the digits into a matrix (large or small integers)
	 * and augment the tensor with the information.
	 */
	if (digsX->count > 0) {
#ifdef PACK_DIGITS
		if (digsX->count <= DIGITS_PER_WORD) {
			makeSmallMatrixFromDigits(smallAccumMat, digsX);
			multVectorPairTimesSmallMatrix(tenXY->ten[0], tenXY->ten[1],
													smallAccumMat);
			multVectorPairTimesSmallMatrix(tenXY->ten[2], tenXY->ten[3],
													smallAccumMat);
		}
		else {
#endif
			makeMatrixFromDigits(bigTmpMat, digsX);
			multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[1], bigTmpMat);
			multVectorPairTimesMatrix(tenXY->ten[2], tenXY->ten[3], bigTmpMat);
#ifdef PACK_DIGITS
		}
#endif
		normalizeTensor(tenXY->ten);
		tenXY->y = digsX->x;
		if (DAVINCI) {
			beginGraphUpdate();
			deleteEdgeToYChild(tenXY, digsX);
			newEdgeToYChild(tenXY, digsX->x);
			endGraphUpdate();
		}
	}

	if (TRACE) {
		debugp("absorbDigsXIntoTenXY_Y",
				"%x %x absorbed=%d\n",
				(unsigned) tenXY,
				(unsigned) digsX,
				digsX->count);
	}
}

void
absorbDigsXIntoTenXY_X(TenXY *tenXY)
{
	DigsX *digsX;
	SmallMatrix smallAccumMat;

	digsX = (DigsX *) tenXY->x;

	/*
	 * Now accumulate the digits into a matrix (large or small integers)
	 * and augment the tensor with the information.
	 */
	if (digsX->count > 0) {
#ifdef PACK_DIGITS
		if (digsX->count <= DIGITS_PER_WORD) {
			makeSmallMatrixFromDigits(smallAccumMat, digsX);
			multVectorPairTimesSmallMatrix(tenXY->ten[0], tenXY->ten[2],
													smallAccumMat);
			multVectorPairTimesSmallMatrix(tenXY->ten[1], tenXY->ten[3],
													smallAccumMat);
		}
		else {
#endif
			makeMatrixFromDigits(bigTmpMat, digsX);
			multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[2], bigTmpMat);
			multVectorPairTimesMatrix(tenXY->ten[1], tenXY->ten[3], bigTmpMat);
#ifdef PACK_DIGITS
		}
#endif
		normalizeTensor(tenXY->ten);
		tenXY->x = digsX->x;
		if (DAVINCI) {
			beginGraphUpdate();
			deleteEdgeToXChild(tenXY, digsX);
			newEdgeToXChild(tenXY, digsX->x);
			endGraphUpdate();
		}
	}

	if (TRACE) {
		debugp("absorbDigsXIntoTenXY_X",
			"%x %x absorbed=%d\n",
			(unsigned) tenXY,
			(unsigned) digsX,
			digsX->count);
	}
}

void
createSignedStreamForTenXY(TenXY *tenXY)
{
	Real r;
	void force_To_SignX_From_TenXY_Entry();

	if (tenXY->strm == NULL) {
		r = tensor_Z(tenXY->x, tenXY->y,
				tenXY->ten[0][0], tenXY->ten[0][1],
				tenXY->ten[1][0], tenXY->ten[1][1],
				tenXY->ten[2][0], tenXY->ten[2][1],
				tenXY->ten[3][0], tenXY->ten[3][1]);
		tenXY->strm = (Real) allocSignX(r, SIGN_UNKN);
		if (DAVINCI) {
			beginGraphUpdate();
			drawEqEdge(tenXY, tenXY->strm);
			endGraphUpdate();
		}
	}
}

void
createUnsignedStreamForTenXY(TenXY *tenXY)
{
	DigsX *digsX;
	Real r;
	void force_To_DigsX_From_TenXY_Entry();

	if (tenXY->strm == NULL) {
		if (tenXY->tag.isSigned)
			Error(FATAL, E_INT, "createUnsignedStreamForTenXY", 
				"creating unsigned stream for signed tensor");
		else {
			r = tensor_Z(tenXY->x, tenXY->y,
					tenXY->ten[0][0], tenXY->ten[0][1],
					tenXY->ten[1][0], tenXY->ten[1][1],
					tenXY->ten[2][0], tenXY->ten[2][1],
					tenXY->ten[3][0], tenXY->ten[3][1]);
			digsX = allocDigsX();
			digsX->x = (Real) r;
			digsX->force = force_To_DigsX_From_TenXY_Entry;
			tenXY->strm = (Real) digsX;
			if (DAVINCI) {
				beginGraphUpdate();
				newEdgeToOnlyChild(digsX, r);
				drawEqEdge(tenXY, tenXY->strm);
				endGraphUpdate();
			}
		}
	}
}

void
force_To_TenXY_X_From_SignX_Entry()
{
	TenXY *tenXY;
	SignX *signX;
	void force_To_TenXY_X_From_SignX_Cont();

	tenXY = (TenXY *) POP;
	signX = (SignX *) tenXY->x;

	PUSH_2(force_To_TenXY_X_From_SignX_Cont, tenXY);
	if (signX->tag.value == SIGN_UNKN)
		PUSH_2(signX->force, signX);
}

void
force_To_TenXY_X_From_SignX_Cont()
{
	TenXY *tenXY;

	tenXY = (TenXY *) POP;
	absorbSignIntoTenXY_X(tenXY);
}

void
force_To_TenXY_Y_From_SignX_Entry()
{
	TenXY *tenXY;
	SignX *signX;
	void force_To_TenXY_Y_From_SignX_Cont();

	tenXY = (TenXY *) POP;
	signX = (SignX *) tenXY->y;

	PUSH_2(force_To_TenXY_Y_From_SignX_Cont, tenXY);
	if (signX->tag.value == SIGN_UNKN)
		PUSH_2(signX->force, signX);
}

void
force_To_TenXY_Y_From_SignX_Cont()
{
	TenXY *tenXY;

	tenXY = (TenXY *) POP;
	absorbSignIntoTenXY_Y(tenXY);
}

/*
 * This handles the case when the x arg of a signed TenXY is a MatX.
 * This version is strict. It inspects its argument first. If it too is
 * signed, then it forces it before reducing the two matrices to one.
 */
void
force_To_TenXY_X_From_MatX_Signed_Entry()
{
	TenXY *tenXY;
	MatX *matX;
	void force_To_TenXY_X_From_MatX_Signed_Cont();
	void force_To_TenXY_X_From_Vec_Signed();

	tenXY = (TenXY *) POP;

	if (tenXY->x->gen.tag.type == VECTOR) {
		PUSH_2(force_To_TenXY_X_From_Vec_Signed, tenXY);
		return;
	}

	matX = (MatX *) tenXY->x;

	PUSH_2(force_To_TenXY_X_From_MatX_Signed_Cont, tenXY);
	if (matX->x->gen.tag.isSigned)
		PUSH_2(matX->force, matX);
}

/*
 * The following code is exactly the same as the unsigned case except there
 * are fewer things on the stack. The two can probably be reconciled
 * as the number of digits is irrelevant when reducing matrices. We leave
 * them separate in case one or other can be improved at a later time.
 */
void
force_To_TenXY_X_From_MatX_Signed_Cont()
{
	TenXY *tenXY;
	void force_To_TenXY_X_From_Vec_Signed();

	tenXY = (TenXY *) POP;

	if (tenXY->x->gen.tag.type == VECTOR) {
		PUSH_2(force_To_TenXY_X_From_Vec_Signed, tenXY);
		return;
	}

	multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[2], tenXY->x->matX.mat);
	multVectorPairTimesMatrix(tenXY->ten[1], tenXY->ten[3], tenXY->x->matX.mat);
	normalizeTensor(tenXY->ten);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->matX.x);
		endGraphUpdate();
	}

	tenXY->x = tenXY->x->matX.x;
	setTenXY_X_MethodUnsigned(tenXY);
}

void
force_To_TenXY_X_From_Vec_Signed()
{
	TenXY *tenXY;
	MatX *matX;
	mpz_t a, b, c, d;	/* temporary storage while we clobber the TenXY */
	Real strm;			/* temporary storage again */
	int totalEmitted;

	tenXY = (TenXY *) POP;

	multVectorPairTimesVector(tenXY->ten[0], tenXY->ten[2], tenXY->x->vec.vec);
	multVectorPairTimesVector(tenXY->ten[1], tenXY->ten[3], tenXY->x->vec.vec);

	a[0] = tenXY->ten[0][0][0];
	b[0] = tenXY->ten[0][1][0];
	c[0] = tenXY->ten[1][0][0];
	d[0] = tenXY->ten[1][1][0];
	strm = tenXY->strm;
	totalEmitted = tenXY->totalEmitted;

	mpz_clear(tenXY->ten[2][0]);
	mpz_clear(tenXY->ten[2][1]);
	mpz_clear(tenXY->ten[3][0]);
	mpz_clear(tenXY->ten[3][1]);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		deleteEdgeToYChild(tenXY, tenXY->y);
		endGraphUpdate();
	}

	matX = (MatX *) tenXY;
	matX->tag.type = MATX;
	matX->x = tenXY->y;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(matX, matX->x);
		endGraphUpdate();
	}

	matX->mat[0][0][0] = a[0];
	matX->mat[0][1][0] = b[0];
	matX->mat[1][0][0] = c[0];
	matX->mat[1][1][0] = d[0];
	normalizeMatrix(matX->mat);
	matX->strm = strm;
	matX->totalEmitted = totalEmitted;

/*
	setMatXMethodUnsigned(matX);
*/
	if (matX->x->gen.tag.isSigned)
		setMatXMethodSigned(matX);
	else
		setMatXMethodUnsigned(matX);
}

void
force_To_TenXY_X_From_TenXY_Signed_Entry()
{
	TenXY *tenXY, *arg;
	void force_To_TenXY_X_From_MatX_Signed_Entry();
	void force_To_TenXY_X_From_TenXY_Signed_Cont();
	void force_To_TenXY_X_From_TenXY_Signed_Cont_X();

	tenXY = (TenXY *) POP;
	arg = (TenXY *) tenXY->x;

	if (arg->tag.type != TENXY) {
		PUSH_2(force_To_TenXY_X_From_MatX_Signed_Entry, tenXY);
		return;
	}

	if (arg->x->gen.tag.isSigned) {
		if (arg->y->gen.tag.isSigned) {
			PUSH_2(force_To_TenXY_X_From_TenXY_Signed_Cont_X, tenXY);
			PUSH_2(arg->forceY, arg);
		}
		else {
			PUSH_2(force_To_TenXY_X_From_TenXY_Signed_Cont, tenXY);
			PUSH_2(arg->forceX, arg);
		}
	}
	else {
		if (arg->y->gen.tag.isSigned) {
			PUSH_2(force_To_TenXY_X_From_TenXY_Signed_Cont, tenXY);
			PUSH_2(arg->forceY, arg);
		}
		else
			PUSH_2(force_To_TenXY_X_From_TenXY_Signed_Cont, tenXY);
	}
}

void
force_To_TenXY_X_From_TenXY_Signed_Cont_X()
{
	TenXY *tenXY, *arg;
	void force_To_TenXY_X_From_TenXY_Signed_Cont();
	void force_To_TenXY_X_From_MatX_Signed_Entry();

	tenXY = (TenXY *) POP;
	arg = (TenXY *) tenXY->x;

	if (arg->tag.type != TENXY) {
		PUSH_2(force_To_TenXY_X_From_MatX_Signed_Entry, tenXY);
		return;
	}

	PUSH_2(force_To_TenXY_X_From_TenXY_Signed_Cont, tenXY);

	if (arg->x->gen.tag.isSigned)
		PUSH_2(arg->forceX, arg);
}

void
force_To_TenXY_X_From_TenXY_Signed_Cont()
{
	TenXY *tenXY;
	void force_To_TenXY_X_From_MatX_Signed_Cont();
	void force_To_TenXY_X_From_SignX_Entry();

	tenXY = (TenXY *) POP;

	if (tenXY->x->gen.tag.type != TENXY) {
		PUSH_2(force_To_TenXY_X_From_MatX_Signed_Cont, tenXY);
		return;
	}

	createSignedStreamForTenXY((TenXY *) tenXY->x);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		newEdgeToXChild(tenXY, tenXY->x->tenXY.strm);
		endGraphUpdate();
	}

	tenXY->x = tenXY->x->tenXY.strm;
	tenXY->forceX = force_To_TenXY_X_From_SignX_Entry;
	PUSH_2(tenXY->forceX, tenXY);
}

/*
 * This handles the case when the x arg of a signed TenXY is a MatX.
 * This version is strict. It inspects its argument first. If it too is
 * signed, then it forces it before reducing the two matrices to one.
 */
void
force_To_TenXY_Y_From_MatX_Signed_Entry()
{
	TenXY *tenXY;
	MatX *matX;
	void force_To_TenXY_Y_From_MatX_Signed_Cont();
	void force_To_TenXY_Y_From_Vec_Signed();

	tenXY = (TenXY *) POP;

	if (tenXY->y->gen.tag.type == VECTOR) {
		PUSH_2(force_To_TenXY_Y_From_Vec_Signed, tenXY);
		return;
	}

	matX = (MatX *) tenXY->y;

	PUSH_2(force_To_TenXY_Y_From_MatX_Signed_Cont, tenXY);
	if (matX->x->gen.tag.isSigned)
		PUSH_2(matX->force, matX);
}

/*
 * The following code is exactly the same as the unsigned case except there
 * are fewer things on the stack. The two can probably be reconciled
 * as the number of digits is irrelevant when reducing matrices. We leave
 * them separate in case one or other can be improved at a later time.
 */
void
force_To_TenXY_Y_From_MatX_Signed_Cont()
{
	TenXY *tenXY;
	void force_To_TenXY_Y_From_Vec_Signed();

	tenXY = (TenXY *) POP;

	if (tenXY->y->gen.tag.type == VECTOR) {
		PUSH_2(force_To_TenXY_Y_From_Vec_Signed, tenXY);
		return;
	}

	multVectorPairTimesMatrix(tenXY->ten[0], tenXY->ten[1], tenXY->y->matX.mat);
	multVectorPairTimesMatrix(tenXY->ten[2], tenXY->ten[3], tenXY->y->matX.mat);
	normalizeTensor(tenXY->ten);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->matX.x);
		endGraphUpdate();
	}

	tenXY->y = tenXY->y->matX.x;
	setTenXY_Y_MethodUnsigned(tenXY);
}

void
force_To_TenXY_Y_From_Vec_Signed()
{
	TenXY *tenXY;
	MatX *matX;
	mpz_t a, b, c, d;	/* temporary storage while we clobber the TenXY */
	Real strm;			/* temporary storage again */
	int totalEmitted;

	tenXY = (TenXY *) POP;

	multVectorPairTimesVector(tenXY->ten[0], tenXY->ten[1], tenXY->y->vec.vec);
	multVectorPairTimesVector(tenXY->ten[2], tenXY->ten[3], tenXY->y->vec.vec);

	a[0] = tenXY->ten[0][0][0];
	b[0] = tenXY->ten[0][1][0];
	c[0] = tenXY->ten[2][0][0];
	d[0] = tenXY->ten[2][1][0];
	strm = tenXY->strm;
	totalEmitted = tenXY->totalEmitted;

	mpz_clear(tenXY->ten[1][0]);
	mpz_clear(tenXY->ten[1][1]);
	mpz_clear(tenXY->ten[3][0]);
	mpz_clear(tenXY->ten[3][1]);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToXChild(tenXY, tenXY->x);
		deleteEdgeToYChild(tenXY, tenXY->y);
		endGraphUpdate();
	}

	matX = (MatX *) tenXY;
	matX->tag.type = MATX;
	matX->x = tenXY->x;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(matX, matX->x);
		endGraphUpdate();
	}

	matX->mat[0][0][0] = a[0];
	matX->mat[0][1][0] = b[0];
	matX->mat[1][0][0] = c[0];
	matX->mat[1][1][0] = d[0];
	normalizeMatrix(matX->mat);
	matX->strm = strm;
	matX->totalEmitted = totalEmitted;

/*
	setMatXMethodUnsigned(matX);
*/
	if (matX->x->gen.tag.isSigned)
		setMatXMethodSigned(matX);
	else
		setMatXMethodUnsigned(matX);
}

void
force_To_TenXY_Y_From_TenXY_Signed_Entry()
{
	TenXY *tenXY, *arg;
	void force_To_TenXY_Y_From_MatX_Signed_Entry();
	void force_To_TenXY_Y_From_TenXY_Signed_Cont();
	void force_To_TenXY_Y_From_TenXY_Signed_Cont_X();

	tenXY = (TenXY *) POP;
	arg = (TenXY *) tenXY->y;

	if (arg->tag.type != TENXY) {
		PUSH_2(force_To_TenXY_Y_From_MatX_Signed_Entry, tenXY);
		return;
	}

	if (arg->x->gen.tag.isSigned) {
		if (arg->y->gen.tag.isSigned) {
			PUSH_2(force_To_TenXY_Y_From_TenXY_Signed_Cont_X, tenXY);
			PUSH_2(arg->forceY, arg);
		}
		else {
			PUSH_2(force_To_TenXY_Y_From_TenXY_Signed_Cont, tenXY);
			PUSH_2(arg->forceX, arg);
		}
	}
	else {
		if (arg->y->gen.tag.isSigned) {
			PUSH_2(force_To_TenXY_Y_From_TenXY_Signed_Cont, tenXY);
			PUSH_2(arg->forceY, arg);
		}
		else
			PUSH_2(force_To_TenXY_Y_From_TenXY_Signed_Cont, tenXY);
	}
}

void
force_To_TenXY_Y_From_TenXY_Signed_Cont_X()
{
	TenXY *tenXY, *arg;
	void force_To_TenXY_Y_From_TenXY_Signed_Cont();
	void force_To_TenXY_Y_From_MatX_Signed_Entry();

	tenXY = (TenXY *) POP;
	arg = (TenXY *) tenXY->y;

	if (arg->tag.type != TENXY) {
		PUSH_2(force_To_TenXY_Y_From_MatX_Signed_Entry, tenXY);
		return;
	}

	PUSH_2(force_To_TenXY_Y_From_TenXY_Signed_Cont, tenXY);

	if (arg->x->gen.tag.isSigned)
		PUSH_2(arg->forceX, arg);
}

void
force_To_TenXY_Y_From_TenXY_Signed_Cont()
{
	TenXY *tenXY;
	void force_To_TenXY_Y_From_MatX_Signed_Cont();
	void force_To_TenXY_Y_From_SignX_Entry();

	tenXY = (TenXY *) POP;

	if (tenXY->y->gen.tag.type != TENXY) {
		PUSH_2(force_To_TenXY_Y_From_MatX_Signed_Cont, tenXY);
		return;
	}

	createSignedStreamForTenXY((TenXY *) tenXY->y);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteEdgeToYChild(tenXY, tenXY->y);
		newEdgeToYChild(tenXY, tenXY->y->tenXY.strm);
		endGraphUpdate();
	}

	tenXY->y = tenXY->y->tenXY.strm;
	tenXY->forceY = force_To_TenXY_Y_From_SignX_Entry;
	PUSH_2(tenXY->forceY, tenXY);
}

void
setTenXY_X_MethodUnsigned(TenXY *tenXY)
{
	void force_To_TenXY_X_From_SignX_Entry();
	void force_To_TenXY_X_From_DigsX_Entry();
	void force_To_TenXY_X_From_Vec();
	void force_To_TenXY_X_From_MatX();
	void force_To_TenXY_X_From_TenXY();
	void force_To_TenXY_X_From_Alt_Entry();
	void force_To_TenXY_X_From_Cls_Entry();

	switch (tenXY->x->gen.tag.type) {
	case SIGNX :
		Error(FATAL, E_INT, "setTenXY_X_MethodUnsigned", "x is signed");
		break;
	case DIGSX :
		tenXY->forceX = force_To_TenXY_X_From_DigsX_Entry;
		break;
	case ALT :
		tenXY->forceX = force_To_TenXY_X_From_Alt_Entry;
		break;
	case VECTOR :
		tenXY->forceX = force_To_TenXY_X_From_Vec;
		break;
	case MATX :
		tenXY->forceX = force_To_TenXY_X_From_MatX;
		break;
	case TENXY :
		tenXY->forceX = force_To_TenXY_X_From_TenXY;
		break;
	case CLOSURE :
		tenXY->forceX = force_To_TenXY_X_From_Cls_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setTenXY_X_MethodUnsigned",
					"something wrong with x");
		break;
	}
}

void
setTenXY_Y_MethodUnsigned(TenXY *tenXY)
{
	void force_To_TenXY_Y_From_SignX_Entry();
	void force_To_TenXY_Y_From_DigsX_Entry();
	void force_To_TenXY_Y_From_Vec();
	void force_To_TenXY_Y_From_MatX();
	void force_To_TenXY_Y_From_TenXY();
	void force_To_TenXY_Y_From_Alt_Entry();
	void force_To_TenXY_Y_From_Cls_Entry();

	switch (tenXY->y->gen.tag.type) {
	case SIGNX :
		Error(FATAL, E_INT, "setTenXY_Y_MethodUnsigned", "y is signed");
		break;
	case DIGSX :
		tenXY->forceY = force_To_TenXY_Y_From_DigsX_Entry;
		break;
	case ALT :
		tenXY->forceY = force_To_TenXY_Y_From_Alt_Entry;
		break;
	case VECTOR :
		tenXY->forceY = force_To_TenXY_Y_From_Vec;
		break;
	case MATX :
		tenXY->forceY = force_To_TenXY_Y_From_MatX;
		break;
	case TENXY :
		tenXY->forceY = force_To_TenXY_Y_From_TenXY;
		break;
	case CLOSURE :
		tenXY->forceY = force_To_TenXY_Y_From_Cls_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setTenXY_Y_MethodUnsigned",
						"something wrong with y");
		break;
	}
}

void
setTenXY_X_MethodSigned(TenXY *tenXY)
{
	void force_To_TenXY_X_From_SignX_Entry();
	void force_To_TenXY_X_From_DigsX_Signed();
	void force_To_TenXY_X_From_Vec_Signed();
	void force_To_TenXY_X_From_MatX_Signed_Entry();
	void force_To_TenXY_X_From_TenXY_Signed_Entry();
	void force_To_TenXY_X_From_Alt_Signed_Entry();
	void force_To_TenXY_X_From_Cls_Signed_Entry();

	switch (tenXY->x->gen.tag.type) {
	case SIGNX :
		tenXY->forceX = force_To_TenXY_X_From_SignX_Entry;
		break;
	case DIGSX :
		tenXY->forceX = force_To_TenXY_X_From_DigsX_Signed;
		break;
	case ALT :
		tenXY->forceX = force_To_TenXY_X_From_Alt_Signed_Entry;
		break;
	case VECTOR :
		tenXY->forceX = force_To_TenXY_X_From_Vec_Signed;
		break;
	case MATX :
		tenXY->forceX = force_To_TenXY_X_From_MatX_Signed_Entry;
		break;
	case TENXY :
		tenXY->forceX = force_To_TenXY_X_From_TenXY_Signed_Entry;
		break;
	case CLOSURE :
		tenXY->forceX = force_To_TenXY_X_From_Cls_Signed_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setTenXY_X_MethodSigned",
					"something wrong with x");
		break;
	}
}

void
setTenXY_Y_MethodSigned(TenXY *tenXY)
{
	void force_To_TenXY_Y_From_SignX_Entry();
	void force_To_TenXY_Y_From_DigsX_Signed();
	void force_To_TenXY_Y_From_Vec_Signed();
	void force_To_TenXY_Y_From_MatX_Signed_Entry();
	void force_To_TenXY_Y_From_TenXY_Signed_Entry();
	void force_To_TenXY_Y_From_Alt_Signed_Entry();
	void force_To_TenXY_Y_From_Cls_Signed_Entry();

	switch (tenXY->y->gen.tag.type) {
	case SIGNX :
		tenXY->forceY = force_To_TenXY_Y_From_SignX_Entry;
		break;
	case DIGSX :
		tenXY->forceY = force_To_TenXY_Y_From_DigsX_Signed;
		break;
	case ALT :
		tenXY->forceY = force_To_TenXY_Y_From_Alt_Signed_Entry;
		break;
	case VECTOR :
		tenXY->forceY = force_To_TenXY_Y_From_Vec_Signed;
		break;
	case MATX :
		tenXY->forceY = force_To_TenXY_Y_From_MatX_Signed_Entry;
		break;
	case TENXY :
		tenXY->forceY = force_To_TenXY_Y_From_TenXY_Signed_Entry;
		break;
	case CLOSURE :
		tenXY->forceY = force_To_TenXY_Y_From_Cls_Signed_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setTenXY_Y_MethodSigned",
					"something wrong with y");
		break;
	}
}
