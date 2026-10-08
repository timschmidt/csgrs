/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "real.h"
#include "real-impl.h"

void initEmitDigit(void);
void initEmitSign(void);
void initEpsDel();
void initTmp();
void initStack();
void initForceMethodLookupTable();
void initStrategy();

int defaultForceCount = DEFAULT_FORCE_COUNT;
int forceDecUpperBound = FORCE_DEC_UPPER_BOUND;
int stackSize = STACK_SIZE * 1024;

/*
 * Initialization of the real library
 */
void
initRealBase()
{
	char *p;

	if ((p = getenv("ICR_DEFAULT_FORCE_COUNT")) != NULL)
		defaultForceCount = atoi(p);

	if ((p = getenv("ICR_FORCE_DEC_UPPER_BOUND")) != NULL)
		forceDecUpperBound = atoi(p);

	if ((p = getenv("ICR_STACK_SIZE")) != NULL)
		stackSize = atoi(p) * 1024;

	if (DAVINCI)
		initDaVinci();

	initStack();
	initForceMethodLookupTable();
	initEmitDigit();
	initEmitSign();
	initTmp();
	initEpsDel();
	initStrategy();
}

/*
 * For a real x, this returns a new real which denotes the same value as
 * x, but where the new real is a stream (ie prefixed by a SignX or DigsX).
 * The only interesting cases are when the expression rooted at x is an
 * lft. In this case we create a new real object but also record in x
 * a link to the new real to avoid doing it again.
 */
Real
makeStream(Real x)
{
	Real r;
	DigsX *digsX;
	void force_To_DigsX_From_Vec();
	void force_To_DigsX_From_MatX_Entry();
	void force_To_DigsX_From_TenXY_Entry();
	void force_To_DigsX_From_Cls_Entry();
	void force_To_DigsX_From_Alt_Entry();

	/*
	 * When we want to make a stream from a vector or matrix, we
	 * must make a copy of the lft first since the LFT might be shared 
	 * and emitting a sign or digit changes the lft. For tensors,
	 * this is unnecessary as, while the tensor might be shared, 
	 * any consumer of the tensor can consume only the stream
	 * and changes to the original tensor are acceptable.
	 */
	switch (x->gen.tag.type) {
	case VECTOR :
		/*
		 * Check that we haven't already made a stream for this lft
		 */
		if (x->vec.strm == NULL) {
			
			/* create a copy of the lft */
			r = vector_Z(x->vec.vec[0], x->vec.vec[1]);

			/* to make a signed lft into a stream, we prefix it by a SignX */
			if (r->gen.tag.isSigned)
				x->vec.strm = (Real) allocSignX(r, SIGN_UNKN);

			/* for an unsigned lft, we prefix it by a DigsX */
			else {
				digsX = allocDigsX();
				digsX->force = force_To_DigsX_From_Vec;
				digsX->x = r;
				if (DAVINCI) {
					beginGraphUpdate();
					newEdgeToOnlyChild(digsX, r);
					endGraphUpdate();
				}
				x->vec.strm = (Real) digsX;
			}
		}

		/*
		 * If daVinci is running then we draw a double line from the original
		 * real and its stream version to indicate that there are two
		 * expressions in the heap which point denote the same real value.
		 */
		if (DAVINCI) {
			beginGraphUpdate();
			drawEqEdge(x, x->vec.strm);
			endGraphUpdate();
		}
		return x->vec.strm;
		break;

	/* this case is exactly the same as the above */
	case MATX :
		if (x->matX.strm == NULL) {
			r = matrix_Z(x->matX.x, x->matX.mat[0][0], x->matX.mat[0][1],
									x->matX.mat[1][0], x->matX.mat[1][1]);
			if (r->gen.tag.isSigned)
				x->matX.strm = (Real) allocSignX(r, SIGN_UNKN);
			else {
				digsX = allocDigsX();
				digsX->force = force_To_DigsX_From_MatX_Entry;
				digsX->x = r;
				if (DAVINCI) {
					beginGraphUpdate();
					newEdgeToOnlyChild(digsX, r);
					endGraphUpdate();
				}
				x->matX.strm = (Real) digsX;
			}
		}
		if (DAVINCI) {
			beginGraphUpdate();
			drawEqEdge(x, x->matX.strm);
			endGraphUpdate();
		}
		return x->matX.strm;
		break;

	/* same as for vectors */
	case TENXY :
		if (x->tenXY.strm == NULL) {
			r = tensor_Z(x->tenXY.x, x->tenXY.y,
					x->tenXY.ten[0][0], x->tenXY.ten[0][1],
					x->tenXY.ten[1][0], x->tenXY.ten[1][1],
					x->tenXY.ten[2][0], x->tenXY.ten[2][1],
					x->tenXY.ten[3][0], x->tenXY.ten[3][1]);
			if (r->gen.tag.isSigned)
				x->tenXY.strm = (Real) allocSignX(r, SIGN_UNKN);
			else {
				digsX = allocDigsX();
				digsX->force = force_To_DigsX_From_TenXY_Entry;
				digsX->x = r;
				if (DAVINCI) {
					beginGraphUpdate();
					newEdgeToOnlyChild(digsX, r);
					endGraphUpdate();
				}
				x->tenXY.strm = (Real) digsX;
			}
			if (DAVINCI) {
				beginGraphUpdate();
				drawEqEdge(x, x->tenXY.strm);
				endGraphUpdate();
			}
		}
		return x->tenXY.strm;
		break;

	case ALT :
		if (x->alt.tag.isSigned)
			r = (Real) allocSignX(x, SIGN_UNKN);
		else {
			r = (Real) allocDigsX();
			r->digsX.force = force_To_DigsX_From_Alt_Entry;
			r->digsX.x = x;
			if (DAVINCI) {
				beginGraphUpdate();
				newEdgeToOnlyChild(r, x);
				endGraphUpdate();
			}
		}
		return r;
		break;

	case CLOSURE :
		if (x->cls.tag.isSigned)
			r = (Real) allocSignX(x, SIGN_UNKN);
		else {
			r = (Real) allocDigsX();
			r->digsX.force = force_To_DigsX_From_Cls_Entry;
			r->digsX.x = x;
			if (DAVINCI) {
				beginGraphUpdate();
				newEdgeToOnlyChild(r, x);
				endGraphUpdate();
			}
		}
		return r;
		break;

	case DIGSX :
	case SIGNX :
		return x;

	default :
		Error(FATAL, E_INT, "makeStream",
					"trying to make a stream from a non-real");
	}
}

/*
 * This creates a closure (typically a function which unfolds a tensor
 * chain).
 */
Cls *
allocCls(void (*force)(), void *userData)
{
	Cls *cls;

	if ((cls = (Cls *) malloc (sizeof(Cls))) == NULL) 
		Error(FATAL, E_INT, "allocCls", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(cls);

	cls->tag.type = CLOSURE;
	cls->tag.dumped = FALSE;
	cls->tag.isSigned = FALSE;
	cls->force = force;
	cls->userData = userData;
	cls->redirect = NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(cls, CLOSURE);
		endGraphUpdate();
	}

	return cls;
}

/*
 * The next family of functions are simple arithmetic operations.
 */
Real
add_R_R(Real x, Real y)
{
	return tensor_Int(x, y, 0, 0, 1, 0, 1, 0, 0, 1);
}

Real
add_R_Int(Real x, int y)
{
	return matrix_Int(x, 1, 0, y, 1);
}

Real
add_R_QInt(Real x, int a, int b)
{
	return matrix_Int(x, b, 0, a, b);
}

Real
sub_R_R(Real x, Real y)
{
	return tensor_Int(x, y, 0, 0, 1, 0, -1, 0, 0, 1);
}

Real
sub_R_Int(Real x, int y)
{
	return matrix_Int(x, 1, 0, -y, 1);
}

Real
sub_R_QInt(Real x, int a, int b)
{
	return matrix_Int(x, b, 0, -a, b);
}

Real
sub_Int_R(int x, Real y)
{
	return matrix_Int(y, -1, 0, x, 1);
}

Real
sub_QInt_R(int a, int b, Real y)
{
	return matrix_Int(y, -b, 0, a, b);
}

Real
mul_R_R(Real x, Real y)
{
	return tensor_Int(x, y, 1, 0, 0, 0, 0, 0, 0, 1);
}

Real
mul_R_Int(Real x, int y)
{
	return matrix_Int(x, y, 0, 0, 1);
}

Real
mul_R_QInt(Real x, int a, int b)
{
	return matrix_Int(x, a, 0, 0, b);
}

Real
div_R_R(Real x, Real y)
{
	return tensor_Int(x, y, 0, 0, 1, 0, 0, 1, 0, 0);
}

Real
div_R_Int(Real x, int y)
{
	return matrix_Int(x, 1, 0, 0, y);
}

Real
div_Int_R(int x, Real y)
{
	return matrix_Int(y, 0, 1, x, 0);
}

Real
div_R_QInt(Real x, int a, int b)
{
	return matrix_Int(x, b, 0, 0, a);
}

Real
div_QInt_R(int a, int b, Real x)
{
	return matrix_Int(x, 0, b, a, 0);
}


/*
 * This creates a vector (a,b) and then prepends it by the characteristic
 * pair (c,n) and finally sets the sign to sign. This is used only for
 * debugging .. to set up a real argument for a function.
 */
Real
makeRealSignCNQInt(Sign sign, char *c, int n, int a, int b)
{
	DigsX *digsX;
	Real y;
	void force_To_DigsX_From_Vec();
	void force_To_DigsX_From_DigsX_Entry();

	y = (Real) vector_Int(a, b);

	if (n > 0) {
		digsX = allocDigsX();
		digsX->x = y;
		digsX->force = force_To_DigsX_From_Vec;

		if (DAVINCI) {
			beginGraphUpdate();
			newEdgeToOnlyChild(digsX, y);
			endGraphUpdate();
		}

		y = (Real) digsX;

		digsX = allocDigsX();
		digsX->x = y;
		digsX->force = force_To_DigsX_From_DigsX_Entry;

		if (DAVINCI) {
			beginGraphUpdate();
			newEdgeToOnlyChild(digsX, y);
			endGraphUpdate();
		}

		digsX->count = n;
#ifdef PACK_DIGITS
		if (n <= DIGITS_PER_WORD)
			digsX->word.small = atoi(c);
		else
			mpz_init_set_str(digsX->word.big, c, 10);
#else
		mpz_set_str(digsX->word.big, c, 10);
#endif
		return (Real) allocSignX((Real) digsX, sign);
	}
	else {
		switch (sign) {
		case SZERO :
		case SPOS :
		case SNEG :
		case SINF :
			digsX = allocDigsX();
			digsX->x = y;
			digsX->force = force_To_DigsX_From_Vec;

			if (DAVINCI) {
				beginGraphUpdate();
				newEdgeToOnlyChild(digsX, y);
				endGraphUpdate();
			}

			digsX->count = 0;
			return (Real) allocSignX((Real) digsX, sign);

		case SIGN_UNKN :
			return (Real) allocSignX(y, sign);

		default :
			Error(FATAL, E_INT, "makeRealSignCNQInt", "invalid sign");
		}
	}
}

/*
 * These are the new preferred names for the basic lft functions.
 * The other names will disappear in a future release.
 */
Real
real_QInt(int a, int b)
{
	return vector_Int(a, b);
}

Real
real_QZ(mpz_t a, mpz_t b)
{
	return vector_Z(a, b);
}

/*
 * These are the same as matrix_ functions but take their arguments
 * in a different order (row order rather than column order).
 */
Real
lft_R_Int(Real x, int a, int b, int c, int d)
{
	return matrix_Int(x, a, c, b, d);
}

Real
lft_R_Z(Real x, mpz_t a, mpz_t b, mpz_t c, mpz_t d)
{
	return matrix_Z(x, a, c, b, d);
}

/*
 * These are the same as tensor_ functions but take their arguments
 * in a different order (row order rather than column order).
 */
Real
lft_R_R_Int(Real x, Real y, int a, int b, int c, int d,
                            int e, int f, int g, int h)
{
	return tensor_Int(x, y, a, e, b, f, c, g, d, h);
}

Real
lft_R_R_Z(Real x, Real y, mpz_t a, mpz_t b, mpz_t c, mpz_t d,
                          mpz_t e, mpz_t f, mpz_t g, mpz_t h)
{
	return tensor_Z(x, y, a, e, b, f, c, g, d, h);
}
