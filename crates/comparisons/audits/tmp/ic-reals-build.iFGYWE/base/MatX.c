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
 * Functions for allocating and manipulating matrix LFTs in the heap.
 */

void setMatXMethodSigned(MatX *);
void setMatXMethodUnsigned(MatX *);
void absorbDigsXIntoMatX(MatX *);

MatX *
allocMatX()
{
	MatX *matX;
 
	if ((matX = (MatX *) malloc (sizeof(MatX))) == NULL) 
		Error(FATAL, E_INT, "allocMatX", "malloc failed");
 
	if (TRACE || DAVINCI)
		newNodeId(matX);

	matX->tag.type = MATX;
	matX->tag.dumped = FALSE;
	matX->tag.isSigned = FALSE;
	matX->strm = (Real) NULL;

	if (DAVINCI) {
    	beginGraphUpdate();
    	newNode(matX, MATX);
    	endGraphUpdate();
	}

	return matX;
}

/*
 * Allocates and fills a matrix object in the heap. Included below
 * is code for eagerly reducing a matrix against its argument. Eager
 * reduction reduces the amount of garbage in the heap. At this point,
 * eager reduction is disabled as some functions which call this expect
 * to get back a MatX object. With reduction they might get either
 * a MatX or a Vec.
 */
Real
matrix_Int(Real x, int a, int b, int c, int d)
{
	MatX *matX;

#ifdef LATER
	/*
	 * If the argument is a vector, then we eagerly reduce our given
	 * matrix to a vector. We do this in tmp storage to avoid creating
	 * garbage in the heap.
	 */
	if (x->gen.tag.type == VECTOR) {
		mpz_set_si(bigTmpMat[0][0], a);
		mpz_set_si(bigTmpMat[0][1], b);
		mpz_set_si(bigTmpMat[1][0], c);
		mpz_set_si(bigTmpMat[1][1], d);
		multVectorPairTimesVector(bigTmpMat[0], bigTmpMat[1], x->vec.vec);
		return vector_Z(bigTmpMat[0][0], bigTmpMat[0][1]);
	}
#endif

	/*
	 * So now we know we will end up with matrix, so we allocate one.
	 */
	matX = allocMatX();

	mpz_init_set_si(matX->mat[0][0], a);
	mpz_init_set_si(matX->mat[0][1], b);
	mpz_init_set_si(matX->mat[1][0], c);
	mpz_init_set_si(matX->mat[1][1], d);

	/*
	 * ### should perhaps check that there are no zero columns
	 */

	/* remove powers of 2 from the matrix */
	normalizeMatrix(matX->mat);

	/* make the matrix positive if it is negative (ie no entries > 0) */
	if (matrixSign(matX->mat) < 0)
		negateMatrix(matX->mat);

	matX->x = (Real) x;

	if (DAVINCI) {
    	beginGraphUpdate();
    	newEdgeToOnlyChild(matX, x);
    	endGraphUpdate();
	}

#ifdef LATER
	/*
	 * Now we eagerly consume any matrix which follows. This may be unwise.
	 */
	if (x->gen.tag.type == MATX) {
		multVectorPairTimesMatrix(matX->mat[0], matX->mat[1], x->matX.mat);
		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(matX, matX->x);
			newEdgeToOnlyChild(matX, x->matX.x);
			endGraphUpdate();
		}
		matX->x = x->matX.x;
	}
#endif

	/*
	 * A MatX is signed if the there are entries in the matrix of different
	 * signs or if the argument to the MatX is signed.
	 */
	if (matX->x->gen.tag.isSigned || matrixSign(matX->mat) == 0)
		matX->tag.isSigned = TRUE;

	if (matX->x->gen.tag.isSigned)
		setMatXMethodSigned(matX);
	else
		setMatXMethodUnsigned(matX);
	return (Real) matX;
}

/*
 * This is exactly as above but in this case the matrix is filled with
 * large (GMP) integers rather than machine integers.
 */
Real
matrix_Z(Real x, mpz_t a, mpz_t b, mpz_t c, mpz_t d)
{
	MatX *matX;

#ifdef LATER
	/*
	 * If the argument is a vector, then we eagerly reduce our given
	 * matrix to a vector. We do this in tmp storage to avoid creating
	 * garbage in the heap.
	 */
	if (x->gen.tag.type == VECTOR) {
		mpz_set(bigTmpMat[0][0], a);
		mpz_set(bigTmpMat[0][1], b);
		mpz_set(bigTmpMat[1][0], c);
		mpz_set(bigTmpMat[1][1], d);
		multVectorPairTimesVector(bigTmpMat[0], bigTmpMat[1], x->vec.vec);
		return vector_Z(bigTmpMat[0][0], bigTmpMat[0][1]);
	}
#endif

	/*
	 * So now we know we will end up with a matrix, so we allocate one.
	 */
	matX = allocMatX();

	mpz_init_set(matX->mat[0][0], a);
	mpz_init_set(matX->mat[0][1], b);
	mpz_init_set(matX->mat[1][0], c);
	mpz_init_set(matX->mat[1][1], d);

	/*
	 * ### should perhaps check that there are no zero columns
	 */

	/* remove powers of 2 from the matrix */
	normalizeMatrix(matX->mat);

	/* make the matrix positive if it is negative (ie no entries > 0) */
	if (matrixSign(matX->mat) < 0)
		negateMatrix(matX->mat);

	matX->x = (Real) x;

	if (DAVINCI) {
    	beginGraphUpdate();
    	newEdgeToOnlyChild(matX, x);
    	endGraphUpdate();
	}

#ifdef LATER
	/*
	 * Now we eagerly consume any matrix which follows. This may be unwise.
	 */
	if (x->gen.tag.type == MATX) {
		multVectorPairTimesMatrix(matX->mat[0], matX->mat[1], x->matX.mat);
		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(matX, matX->x);
			newEdgeToOnlyChild(matX, x->matX.x);
			endGraphUpdate();
		}
		matX->x = x->matX.x;
	}
#endif

	/*
	 * A MatX is signed if the there are entries in the matrix of different
	 * signs or if the argument to the MatX is signed.
	 */
	if (matX->x->gen.tag.isSigned || matrixSign(matX->mat) == 0)
		matX->tag.isSigned = TRUE;

	if (matX->x->gen.tag.isSigned)
		setMatXMethodSigned(matX);
	else
		setMatXMethodUnsigned(matX);
	return (Real) matX;
}

void
force_To_MatX_From_DigsX_Entry()
{
	MatX *matX;
	DigsX *digsX;
	int digitsNeeded;
	void force_To_MatX_From_DigsX_Cont();

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;
	digsX = (DigsX *) matX->x;

	PUSH_3(force_To_MatX_From_DigsX_Cont, matX, digitsNeeded);

	/*
	 * See if the source has the number of digits we need. If not,
	 * then force the remaining.
	 */
	if (digsX->count < digitsNeeded)
		PUSH_3(digsX->force, digsX, digitsNeeded - digsX->count);
}

void
force_To_MatX_From_DigsX_Cont()
{
	MatX *matX;
	int digitsNeeded;

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;

	absorbDigsXIntoMatX(matX);
}

/*
 * It can happen that, for example, an Alt is deemed signed, and yet
 * the value it ultimately yields is unsigned. In this case we need to
 * emit a sign from a DigsZ. This is little more than a no-op.
 */
void
force_To_MatX_From_DigsX_Signed()
{
	MatX *matX;
	void force_To_MatX_From_DigsX_Entry();

	matX = (MatX *) POP;
	matX->force = force_To_MatX_From_DigsX_Entry;
}

/*
 * When a matrix is applied to an vector, the matrix reduces to a vector.
 * The reduction happens in place. That is, we overwrite the matrix
 * with a vector. That way any other consumers which share the matrix
 * (now vector), end up pointing to the vector.
 */
void
force_To_MatX_From_Vec()
{
	MatX *matX;
	Vec *vec;
	int digitsNeeded;
	mpz_t a, b;		/* temporary storage while we clobber the MatX */
	Real strm;		/* temporary storage while we clobber the MatX */

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;

	multVectorPairTimesVector(matX->mat[0], matX->mat[1], matX->x->vec.vec);

	a[0] = matX->mat[0][0][0];
	b[0] = matX->mat[0][1][0];
	strm = matX->strm;

	mpz_clear(matX->mat[1][0]);
	mpz_clear(matX->mat[1][1]);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, matX->x);
		endGraphUpdate();
	}

	vec = (Vec *) matX;

	vec->tag.type = VECTOR;
	vec->vec[0][0] = a[0];
	vec->vec[1][0] = b[0];
	normalizeVector(vec->vec);
	vec->strm = strm;
}

/*
 * Same as the above, except for the signed case. The stack frame
 * is different.
 */
void
force_To_MatX_From_Vec_Signed()
{
	MatX *matX;
	Vec *vec;
	mpz_t a, b;		/* temporary storage while we clobber the MatX */
	Real strm;		/* temporary storage while we clobber the MatX */

	matX = (MatX *) POP;

	multVectorPairTimesVector(matX->mat[0], matX->mat[1], matX->x->vec.vec);

	a[0] = matX->mat[0][0][0];
	b[0] = matX->mat[0][1][0];
	strm = matX->strm;

	mpz_clear(matX->mat[1][0]);
	mpz_clear(matX->mat[1][1]);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, matX->x);
		endGraphUpdate();
	}

	vec = (Vec *) matX;

	vec->tag.type = VECTOR;
	vec->vec[0][0] = a[0];
	vec->vec[1][0] = b[0];
	normalizeVector(vec->vec);
	vec->tag.isSigned = TRUE;
	vec->strm = strm;
}

void
force_To_MatX_From_MatX()
{
	MatX *matX;
	int digitsNeeded;
	void force_To_MatX_From_Vec();

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;

	if (matX->x->gen.tag.type == VECTOR) {
		PUSH_3(force_To_MatX_From_Vec, matX, digitsNeeded);
		return;
	}

	multVectorPairTimesMatrix(matX->mat[0], matX->mat[1], matX->x->matX.mat);
	normalizeMatrix(matX->mat);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, matX->x);
		newEdgeToOnlyChild(matX, matX->x->matX.x);
		endGraphUpdate();
	}
	matX->x = matX->x->matX.x;
	setMatXMethodUnsigned(matX);
}

/*
 * This handles the case when the arg of a signed MatX is another MatX.
 * This version is strict. It inspects its argument first. If it too is
 * signed, then it forces it before reducing the two matrices to one.
 */
void
force_To_MatX_From_MatX_Signed_Entry()
{
	MatX *matX, *arg;
	void force_To_MatX_From_MatX_Signed_Cont();
	void force_To_MatX_From_Vec_Signed();

	matX = (MatX *) POP;

	if (matX->x->gen.tag.type == VECTOR) {
		PUSH_2(force_To_MatX_From_Vec_Signed, matX);
		return;
	}

	arg = (MatX *) matX->x;

	PUSH_2(force_To_MatX_From_MatX_Signed_Cont, matX);
	if (arg->x->gen.tag.isSigned)
		PUSH_2(arg->force, arg);
}

/*
 * The following code is exactly the same as the unsigned case except there
 * are fewer things on the stack. The two can probably be reconciled
 * as the number of digits is irrelevant when reducing matrices. We leave
 * them separate in case one or other can be improved at a later time.
 */
void
force_To_MatX_From_MatX_Signed_Cont()
{
	MatX *matX;
	void force_To_MatX_From_Vec_Signed();

	matX = (MatX *) POP;

	if (matX->x->gen.tag.type == VECTOR) {
		PUSH_2(force_To_MatX_From_Vec_Signed, matX);
		return;
	}

	multVectorPairTimesMatrix(matX->mat[0], matX->mat[1], matX->x->matX.mat);
	normalizeMatrix(matX->mat);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, matX->x);
		newEdgeToOnlyChild(matX, matX->x->matX.x);
		endGraphUpdate();
	}

	matX->x = matX->x->matX.x;
	setMatXMethodUnsigned(matX);
}

void
force_To_MatX_From_TenXY()
{
	MatX *matX;
	int digitsNeeded;
	void force_To_MatX_From_MatX();
	void force_To_MatX_From_DigsX_Entry();

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;

	if (matX->x->gen.tag.type != TENXY) {
		PUSH_3(force_To_MatX_From_MatX, matX, digitsNeeded);
		return;
	}

	createUnsignedStreamForTenXY(matX->x);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, matX->x);
		newEdgeToOnlyChild(matX, matX->x->tenXY.strm);
		endGraphUpdate();
	}

	matX->x = matX->x->tenXY.strm;
	matX->force = force_To_MatX_From_DigsX_Entry;
	PUSH_3(matX->force, matX, digitsNeeded);
}

void
force_To_MatX_From_TenXY_Signed_Entry()
{
	MatX *matX;
	TenXY *tenXY;
	void force_To_MatX_From_MatX_Signed_Entry();
	void force_To_MatX_From_TenXY_Signed_Cont();
	void force_To_MatX_From_TenXY_Signed_Cont_X();

	matX = (MatX *) POP;
	tenXY = (TenXY *) matX->x;

	if (matX->x->gen.tag.type != TENXY) {
		PUSH_2(force_To_MatX_From_MatX_Signed_Entry, matX);
		return;
	}

	if (tenXY->x->gen.tag.isSigned) {
		if (tenXY->y->gen.tag.isSigned) {
			PUSH_2(force_To_MatX_From_TenXY_Signed_Cont_X, matX);
			PUSH_2(tenXY->forceY, tenXY);
		}
		else {
			PUSH_2(force_To_MatX_From_TenXY_Signed_Cont, matX);
			PUSH_2(tenXY->forceX, tenXY);
		}
	}
	else {
		if (tenXY->y->gen.tag.isSigned) {
			PUSH_2(force_To_MatX_From_TenXY_Signed_Cont, matX);
			PUSH_2(tenXY->forceY, tenXY);
		}
		else
			PUSH_2(force_To_MatX_From_TenXY_Signed_Cont, matX);
	}
}

/*
 * Here we have already forced the y argument, and now we force the sign
 * from the x side of the tensor.
 */
void
force_To_MatX_From_TenXY_Signed_Cont_X()
{
	MatX *matX;
	TenXY *tenXY;
	void force_To_MatX_From_MatX_Signed_Entry();
	void force_To_MatX_From_TenXY_Signed_Cont();

	matX = (MatX *) POP;
	tenXY = (TenXY *) matX->x;

	if (matX->x->gen.tag.type != TENXY) {
		PUSH_2(force_To_MatX_From_MatX_Signed_Entry, matX);
		return;
	}

	PUSH_2(force_To_MatX_From_TenXY_Signed_Cont, matX);
	if (tenXY->x->gen.tag.isSigned)
		PUSH_2(tenXY->forceX, tenXY);
}

void
force_To_MatX_From_TenXY_Signed_Cont()
{
	MatX *matX;
	void force_To_MatX_From_MatX_Signed_Cont();
	void force_To_MatX_From_SignX_Entry();

	matX = (MatX *) POP;

	if (matX->x->gen.tag.type != TENXY) {
		PUSH_2(force_To_MatX_From_MatX_Signed_Cont, matX);
		return;
	}

	createSignedStreamForTenXY(matX->x);

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, matX->x);
		newEdgeToOnlyChild(matX, matX->x->tenXY.strm);
		endGraphUpdate();
	}

	matX->x = matX->x->tenXY.strm;
	matX->force = force_To_MatX_From_SignX_Entry;
	PUSH_2(matX->force, matX);
}

void
force_To_MatX_From_Alt_Entry()
{
	MatX *matX;
	Alt *alt;
	void force_To_Alt_Entry();
	void force_To_MatX_From_Alt_Cont();
	int digitsNeeded;

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;
	alt = (Alt *) matX->x;
	
	PUSH_3(force_To_MatX_From_Alt_Cont, matX, digitsNeeded);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (alt->redirect == NULL)
		PUSH_2(force_To_Alt_Entry, alt);
}

void
force_To_MatX_From_Alt_Cont()
{
	MatX *matX;
	Alt *alt;
	int digitsNeeded;

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;
	alt = (Alt *) matX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, alt);
		newEdgeToOnlyChild(matX, alt->redirect);
		endGraphUpdate();
	}
	matX->x = alt->redirect;
	setMatXMethodUnsigned(matX);

	PUSH_3(matX->force, matX, digitsNeeded);
}

void
force_To_MatX_From_Cls_Entry()
{
	MatX *matX;
	Cls *cls;
	void force_To_MatX_From_Cls_Cont();
	int digitsNeeded;

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) matX->x;
	
	PUSH_3(force_To_MatX_From_Cls_Cont, matX, digitsNeeded);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to force it.
	 */
	if (cls->redirect == NULL)
		PUSH_2(cls->force, cls);
}

void
force_To_MatX_From_Cls_Cont()
{
	MatX *matX;
	Cls *cls;
	int digitsNeeded;

	matX = (MatX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) matX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, cls);
		newEdgeToOnlyChild(matX, cls->redirect);
		endGraphUpdate();
	}
	matX->x = cls->redirect;
	setMatXMethodUnsigned(matX);

	PUSH_3(matX->force, matX, digitsNeeded);
}

void
force_To_MatX_From_Alt_Signed_Entry()
{
	MatX *matX;
	Alt *alt;
	void force_To_Alt_Entry();
	void force_To_MatX_From_Alt_Signed_Cont();

	matX = (MatX *) POP;
	alt = (Alt *) matX->x;
	
	PUSH_2(force_To_MatX_From_Alt_Signed_Cont, matX);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (alt->redirect == NULL)
		PUSH_2(force_To_Alt_Entry, alt);
}

void
force_To_MatX_From_Alt_Signed_Cont()
{
	MatX *matX;
	Alt *alt;

	matX = (MatX *) POP;
	alt = (Alt *) matX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, alt);
		newEdgeToOnlyChild(matX, alt->redirect);
		endGraphUpdate();
	}

	matX->x = alt->redirect;
	setMatXMethodSigned(matX);

	PUSH_2(matX->force, matX);
}

void
force_To_MatX_From_Cls_Signed_Entry()
{
	MatX *matX;
	Cls *cls;
	void force_To_MatX_From_Cls_Signed_Cont();

	matX = (MatX *) POP;
	cls = (Cls *) matX->x;
	
	PUSH_2(force_To_MatX_From_Cls_Signed_Cont, matX);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to force it.
	 */
	if (cls->redirect == NULL)
		PUSH_2(cls->force, cls);
}

void
force_To_MatX_From_Cls_Signed_Cont()
{
	MatX *matX;
	Cls *cls;

	matX = (MatX *) POP;
	cls = (Cls *) matX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, cls);
		newEdgeToOnlyChild(matX, cls->redirect);
		endGraphUpdate();
	}

	matX->x = cls->redirect;
	setMatXMethodSigned(matX);

	PUSH_2(matX->force, matX);
}

void
force_To_MatX_From_SignX_Entry()
{
	MatX *matX;
	SignX *signX;
	void force_To_MatX_From_SignX_Cont();

	matX = (MatX *) POP;
	signX = (SignX *) matX->x;

	PUSH_2(force_To_MatX_From_SignX_Cont, matX);
	if (signX->tag.value == SIGN_UNKN)
		PUSH_2(signX->force, signX);
}

void
force_To_MatX_From_SignX_Cont()
{
	MatX *matX;
	void absorbSignIntoMatX(MatX *);

	matX = (MatX *) POP;
	absorbSignIntoMatX(matX);
}

void
absorbSignIntoMatX(MatX *matX)
{
	absorbSignIntoVectorPair(matX->mat[0], matX->mat[1],
					matX->x->signX.tag.value);
	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(matX, matX->x);
		newEdgeToOnlyChild(matX, matX->x->signX.x);
		endGraphUpdate();
	}
	matX->x = matX->x->signX.x;
	setMatXMethodUnsigned(matX);
}

void
setMatXMethodSigned(MatX *matX)
{
	void force_To_MatX_From_SignX_Entry();
	void force_To_MatX_From_DigsX_Signed();
	void force_To_MatX_From_Vec_Signed();
	void force_To_MatX_From_MatX_Signed_Entry();
	void force_To_MatX_From_TenXY_Signed_Entry();
	void force_To_MatX_From_Alt_Signed_Entry();
	void force_To_MatX_From_Cls_Signed_Entry();

	switch (matX->x->gen.tag.type) {
	case SIGNX :
		matX->force = force_To_MatX_From_SignX_Entry;
		break;
	case DIGSX :
		matX->force = force_To_MatX_From_DigsX_Signed;
		break;
	case ALT :
		matX->force = force_To_MatX_From_Alt_Signed_Entry;
		break;
	case VECTOR :
		matX->force = force_To_MatX_From_Vec_Signed;
		break;
	case MATX :
		matX->force = force_To_MatX_From_MatX_Signed_Entry;
		break;
	case TENXY :
		matX->force = force_To_MatX_From_TenXY_Signed_Entry;
		break;
	case CLOSURE :
		matX->force = force_To_MatX_From_Cls_Signed_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setMatXMethodSigned", "something wrong with x");
		break;
	}
}

void
setMatXMethodUnsigned(MatX *matX)
{
	void force_To_MatX_From_SignX_Entry();
	void force_To_MatX_From_DigsX_Entry();
	void force_To_MatX_From_Vec();
	void force_To_MatX_From_MatX();
	void force_To_MatX_From_TenXY();
	void force_To_MatX_From_Alt_Entry();
	void force_To_MatX_From_Cls_Entry();

	switch (matX->x->gen.tag.type) {
	case SIGNX :
		Error(FATAL, E_INT, "setMatXMethodUnsigned", "x is signed");
		break;
	case DIGSX :
		matX->force = force_To_MatX_From_DigsX_Entry;
		break;
	case ALT :
		matX->force = force_To_MatX_From_Alt_Entry;
		break;
	case VECTOR :
		matX->force = force_To_MatX_From_Vec;
		break;
	case MATX :
		matX->force = force_To_MatX_From_MatX;
		break;
	case TENXY :
		matX->force = force_To_MatX_From_TenXY;
		break;
	case CLOSURE :
		matX->force = force_To_MatX_From_Cls_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setMatXMethodUnsigned", "something wrong with x");
		break;
	}
}

void
absorbDigsXIntoMatX(MatX *matX)
{
	DigsX *digsX;
	SmallMatrix smallAccumMat;

	digsX = (DigsX *) matX->x;

	/*
	 * Now accumulate the digits into a matrix (large or small integers)
	 * and augment the matrix (matX) with the information.
	 */
	if (digsX->count > 0) {
#ifdef PACK_DIGITS
		if (digsX->count <= DIGITS_PER_WORD) {
			makeSmallMatrixFromDigits(smallAccumMat, digsX);
			multVectorPairTimesSmallMatrix(matX->mat[0], matX->mat[1],
											smallAccumMat);
		}
		else {
#endif
			makeMatrixFromDigits(bigTmpMat, digsX);
			multVectorPairTimesMatrix(matX->mat[0], matX->mat[1], bigTmpMat);
#ifdef PACK_DIGITS
		}
#endif
		matX->x = digsX->x;
		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(matX, digsX);
			newEdgeToOnlyChild(matX, digsX->x);
			endGraphUpdate();
		}

		/*
		 * Now try to remove powers of 2 from the residual matrix.
		 */
		normalizeMatrix(matX->mat);

	}

	if (TRACE) {
		debugp("force_To_MatX_From_DigsX",
			"%x %x absorbed=%d\n",
			(unsigned) matX,
			(unsigned) digsX,
			digsX->count);
	}
}
