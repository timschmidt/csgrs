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
 * Functions for forcing signs from LFTs. Bear in mind that, unlike digits,
 * sign emission is strict. We cannot emit a sign from an lft
 * without first absorbing the sign(s) from the argument(s) of the lft.
 *
 * Unlike digit emission which is associated with edges, sign emission is
 * node based. 
 */

bool emitSignFromVector(Vector, Sign *);
bool emitSignFromMatrix(Matrix, Sign *);
bool emitSignFromTensor(Tensor, Sign *);

void redirectSignX(SignX *, Real);

/*
 * Allocate a structure in the heap for a SignX.
 */
SignX *
allocSignX(Real x, int sign)
{
	SignX *signX;
	void force_To_SignX_From_DigsX();
	void force_To_SignX_From_Vec();
	void force_To_SignX_From_MatX_Entry();
	void force_To_SignX_From_TenXY_Entry();
	void force_To_SignX_From_Cls_Entry();
	void force_To_SignX_From_Alt_Entry();

	if ((signX = (SignX *) malloc (sizeof(SignX))) == NULL) 
		Error(FATAL, E_INT, "allocSignX", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(signX);

	signX->tag.type = SIGNX;
	signX->tag.dumped = FALSE;
	signX->tag.isSigned = TRUE;
	signX->tag.value = sign;

	signX->x = x;

	/*
	 * Now set the method to retrieve the sign from the argument.
	 * We do a sanity check along the way. If the sign is specified, then
	 * the force method assigned here is irrelevant.
	 */
	switch (x->gen.tag.type) {
		case DIGSX :
			signX->force = force_To_SignX_From_DigsX;
			break;
		case ALT :
			signX->force = force_To_SignX_From_Alt_Entry;
			break;
		case CLOSURE :
			signX->force = force_To_SignX_From_Cls_Entry;
			break;
		case VECTOR :
			if (sign != SIGN_UNKN && !vectorIsPositive(x->vec.vec))
				Error(FATAL, E_INT, "allocSignX",
						"sign specified but vector not positive");
			signX->force = force_To_SignX_From_Vec;
			break;
		case MATX :
			if (sign != SIGN_UNKN && !matrixIsPositive(x->matX.mat))
				Error(FATAL, E_INT, "allocSignX",
						"sign specified but matrix not positive");
			signX->force = force_To_SignX_From_MatX_Entry;
			break;
		case TENXY :
			if (sign != SIGN_UNKN && !tensorIsPositive(x->tenXY.ten))
				Error(FATAL, E_INT, "allocSignX",
						"sign specified but tensor not positive");
			signX->force = force_To_SignX_From_TenXY_Entry;
			break;
		default :
			Error(FATAL, E_INT, "allocSignX", "bad argument type");
	}

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(signX, SIGNX);
		newEdgeToOnlyChild(signX, x);
		endGraphUpdate();
	}

	return signX;
}

void
setSignXMethods(SignX *signX)
{
	void force_To_SignX_From_Alt_Entry();
	void force_To_SignX_From_Cls_Entry();
	void force_To_SignX_From_SignX_Entry();
	void force_To_SignX_From_DigsX();
	void force_To_SignX_From_Vec();
	void force_To_SignX_From_MatX_Entry();
	void force_To_SignX_From_TenXY_Entry();

	switch (signX->x->gen.tag.type) {
	case ALT :
		signX->force = force_To_SignX_From_Alt_Entry;
		break;
	case SIGNX :
		signX->force = force_To_SignX_From_SignX_Entry;
		break;
	case DIGSX :
		signX->force = force_To_SignX_From_DigsX;
		break;
	case VECTOR :
		signX->force = force_To_SignX_From_Vec;
		break;
	case MATX :
		signX->force = force_To_SignX_From_MatX_Entry;
		break;
	case TENXY :
		signX->force = force_To_SignX_From_TenXY_Entry;
		break;
	case CLOSURE :
		signX->force = force_To_SignX_From_Cls_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setSignXMethod", "bad argument type");
	}
}

/*
 * When an lft is guarded by a SIGNX and we emit the sign then we introduce
 * an empty DigsX between the sign and the residual lft.
 */  
void
introDigsX(SignX *signX)
{
	DigsX *digsX;
	void force_To_SignX_From_DigsX();

	digsX = allocDigsX(signX->x);
	digsX->x = signX->x;
	setDigsXMethod(digsX);

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(digsX, signX->x);
		deleteOnlyEdge(signX, signX->x);
		newEdgeToOnlyChild(signX, digsX);
		endGraphUpdate();
	}

	signX->x = (Real) digsX;
	signX->force = force_To_SignX_From_DigsX;
}

/*
 * This is called when the sign is unknown. In fact, in the case
 * of vectors, it is always possible to determine the sign when the
 * vector is created. But we leave it for now.
 */
void
force_To_SignX_From_Vec()
{
	SignX *signX;
	Vec *vec;
	Sign s;

	signX = (SignX *) POP;
	vec = (Vec *) signX->x;

	if (emitSignFromVector(vec->vec, &s)) {
		signX->tag.value = s;
		introDigsX(signX);
	}
	else
		Error(FATAL, E_INT, "force_To_SignX_From_Vecr",
			"Failed to get sign from vector");
}

void
force_To_SignX_From_MatX_Entry()
{
	SignX *signX;
	MatX *matX;
	void force_To_SignX_From_MatX_Cont();
	void force_To_SignX_From_Vec();

	signX = (SignX *) POP;
	matX = (MatX *) signX->x;

	if (matX->tag.type == VECTOR) {
		PUSH_2(force_To_SignX_From_Vec, signX);
		return;
	}

	PUSH_2(force_To_SignX_From_MatX_Cont, signX);

	/*
	 * First check to see if the argument is signed. If
	 * so then we must force it.
	 */
	if (matX->x->gen.tag.isSigned)
		PUSH_2(matX->force, matX);
}

void
force_To_SignX_From_MatX_Cont()
{
	SignX *signX;
	MatX *matX;
	void force_To_SignX_From_Vec();
	Sign s;

	signX = (SignX *) POP;

	/*
	 * First of all, we need to check that the MatX argument hasn't been
	 * reduced to a vector.
	 */
	if (signX->x->gen.tag.type == VECTOR) {
		PUSH_2(force_To_SignX_From_Vec, signX);
		return;
	}

	matX = (MatX *) signX->x;

	/*
	 * Now try to emit our sign, and if we fail to do so, we get a digit
	 * from the argument.
	 */
	if (emitSignFromMatrix(matX->mat, &s)) {
		signX->tag.value = s;
		introDigsX(signX);
		return;
	}

	PUSH_2(force_To_SignX_From_MatX_Cont, signX);
	PUSH_3(matX->force, matX, defaultForceCount);
}

void
force_To_SignX_From_TenXY_Entry()
{
	SignX *signX;
	TenXY *tenXY;
	void force_To_SignX_From_TenXY_Cont();
	void force_To_SignX_From_MatX_Entry();
	void force_To_SignX_From_TenXY_Cont_X();

	signX = (SignX *) POP;
	tenXY = (TenXY *) signX->x;

	if (tenXY->tag.type != TENXY) {
		PUSH_2(force_To_SignX_From_MatX_Entry, signX);
		return;
	}

	tenXY->tensorFairness = 1;

	if (tenXY->x->gen.tag.isSigned) {
		if (tenXY->y->gen.tag.isSigned) {
			PUSH_2(force_To_SignX_From_TenXY_Cont_X, signX);
			PUSH_2(tenXY->forceY, tenXY);
		}
		else {
			PUSH_2(force_To_SignX_From_TenXY_Cont, signX);
			PUSH_2(tenXY->forceX, tenXY);
		}
	}
	else {
		if (tenXY->y->gen.tag.isSigned) {
			PUSH_2(force_To_SignX_From_TenXY_Cont, signX);
			PUSH_2(tenXY->forceY, tenXY);
		}
		else
			PUSH_2(force_To_SignX_From_TenXY_Cont, signX);
	}
}

void
force_To_SignX_From_TenXY_Cont_X()
{
	SignX *signX;
	TenXY *tenXY;
	void force_To_SignX_From_TenXY_Cont();
	void force_To_SignX_From_MatX_Entry();

	signX = (SignX *) POP;
	tenXY = (TenXY *) signX->x;

	if (tenXY->tag.type != TENXY) {
		PUSH_2(force_To_SignX_From_MatX_Entry, signX);
		return;
	}

	PUSH_2(force_To_SignX_From_TenXY_Cont, signX);

	if (tenXY->x->gen.tag.isSigned)
		PUSH_2(tenXY->forceX, tenXY);
}

void
force_To_SignX_From_TenXY_Cont()
{
	SignX *signX;
	TenXY *tenXY;
	Sign s;
	void force_To_SignX_From_MatX_Entry();
	void force_To_SignX_From_MatX_Cont();

	signX = (SignX *) POP;

	/*
	 * First of all, we need to check that the TenXY argument hasn't been
	 * reduced to a matrix or vector.
	 */
	if (signX->x->gen.tag.type != TENXY) {
		PUSH_2(force_To_SignX_From_MatX_Cont, signX);
		return;
	}

	tenXY = (TenXY *) signX->x;

	/*
	 * Now try to emit the sign, and if we fail to do so, we get information
	 * from one or other of the arguments.
	 */
	if (emitSignFromTensor(tenXY->ten, &s)) {
		signX->tag.value = s;
		introDigsX(signX);
		return;
	}

	/*
	 * If we get here, then we have failed to emit the sign.
	 * So we take turns absorbing digits from left and right until
	 * we succeed.
	 */
	PUSH_2(force_To_SignX_From_TenXY_Cont, signX);

	tenXY->tensorFairness = !tenXY->tensorFairness;
	if (tenXY->tensorFairness)
		PUSH_3(tenXY->forceX, tenXY, defaultForceCount);
	else
		PUSH_3(tenXY->forceY, tenXY, defaultForceCount);
}

void
force_To_SignX_From_DigsX()
{
	SignX *signX;

	signX = (SignX *) POP;
	signX->tag.value = SPOS;
}


/*
 * If we prefix an alt with a SignX, then it can happen that after reduction
 * we end up with a SignX consuming from another SignX. When this happens
 * then when we force the top level SignX, we must force the second
 * and then copy the sign from the second to the first.
 */
void
force_To_SignX_From_SignX_Entry()
{
	SignX *signX;
	void force_To_SignX_From_SignX_Cont();

	signX = (SignX *) POP;

	PUSH_2(force_To_SignX_From_SignX_Cont, signX);
	if (signX->x->signX.tag.value == SIGN_UNKN)
		PUSH_2(signX->x->signX.force, signX->x);
}

void
force_To_SignX_From_SignX_Cont()
{
	SignX *signX;

	signX = (SignX *) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(signX, signX->x);
		newEdgeToOnlyChild(signX, signX->x->signX.x);
		endGraphUpdate();
	}

	/* copy the sign from the second to the first */
	signX->tag.value = signX->x->signX.tag.value;
	signX->x = signX->x->signX.x;
}

/*
 * These functions are only used when an alt has been explicitly guarded
 * using makeStream();
 *
 * force_To_SignX_From_Alt(Alt *alt)
 * {
 *    if (alt->redirect == NULL)
 *      force_To_Alt_Entry(alt);
 *
 *    # we need to handle the special case when the alt evaluates to a real
 *    # prefixed by a SignX. We need to force the sign.
 *    
 *    if (alt->redirect->gen.tag.type == SIGNX
				&& alt->redirect->signX.value == SIGN_UNKN)
 *       *(alt->redirect->signX.force)(alt->redirect);
 *
 *    # now inspect the tag on alt->redirect to decide how to tranferi
 *    # or evaluate the sign.
 *    # Because of sharing and since emission has side-effects
 *    # then in the case of matrices and vectors, we must make a copy
 *    # of the lft. 
 *    ... see below for the cases.
 * }
 */
void
force_To_SignX_From_Alt_Entry()
{
	SignX *signX;
	Alt *alt;
	void force_To_Alt_Entry();
	void force_To_SignX_From_Alt_Cont();

	signX = (SignX *) POP;
	alt = (Alt *) signX->x;
	
	PUSH_2(force_To_SignX_From_Alt_Cont, signX);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to evaluate it.
	 */
	if (alt->redirect == NULL)
		PUSH_2(force_To_Alt_Entry, alt);
}

/*
 * At this point we know that the alt has evaluated to a real. We need
 * to consume a sign from this real. 
 */
void
force_To_SignX_From_Alt_Cont()
{
	SignX *signX;
	Alt *alt;

	signX = (SignX *) POP;
	alt = (Alt *) signX->x;
	redirectSignX(signX, alt->redirect);
	PUSH_2(signX->force, signX);
}

void
force_To_SignX_From_Cls_Entry()
{
	SignX *signX;
	Cls *cls;
	void force_To_SignX_From_Cls_Cont();

	signX = (SignX *) POP;
	cls = (Cls *) signX->x;
	
	PUSH_2(force_To_SignX_From_Cls_Cont, signX);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to evaluate it.
	 */
	if (cls->redirect == NULL)
		PUSH_2(cls->force, cls);
}

/*
 * At this point we know that the cls has evaluated to a real. We need
 * to consume a sign from this real. 
 */
void
force_To_SignX_From_Cls_Cont()
{
	SignX *signX;
	Cls *cls;

	signX = (SignX *) POP;
	cls = (Cls *) signX->x;
	redirectSignX(signX, cls->redirect);
	PUSH_2(signX->force, signX);
}

void
redirectSignX(SignX *signX, Real x)
{
	Real r;

	void force_To_SignX_From_SignX_Entry();
	void force_To_SignX_From_Cls_Entry();
	void force_To_SignX_From_Alt_Entry();
	void force_To_SignX_From_DigsX();
	void force_To_SignX_From_Vec();
	void force_To_SignX_From_MatX_Entry();
	void force_To_SignX_From_TenXY_Entry();

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(signX, signX->x);
		newEdgeToOnlyChild(signX, x);
		endGraphUpdate();
	}
	signX->x = x;

	switch (x->gen.tag.type) {
	case SIGNX :
		signX->force = force_To_SignX_From_SignX_Entry;
		break;
	case DIGSX :
		signX->force = force_To_SignX_From_DigsX;
		break;
	case CLOSURE :
		signX->force = force_To_SignX_From_Cls_Entry;
		break;
	case ALT :
		signX->force = force_To_SignX_From_Alt_Entry;
		break;
	case VECTOR :
		/*
		 * First we check that the Vector does not have an equivalent stream.
		 * If it doesn't then the SignX consumer we already have, will
		 * become the root of the new equivalent stream. Note that we
		 * are oblidged to make a copy of the vector since there may be
		 * other consumers, and emitting the sign will change the vector.
		 */
		if (x->vec.strm == NULL) {
			r = vector_Z(x->vec.vec[0], x->vec.vec[1]);
			signX->x = r;
			x->vec.strm = (Real) signX;
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(signX, x);
				newEdgeToOnlyChild(signX, r);
				drawEqEdge(signX, x);
				endGraphUpdate();
			}
			signX->force = force_To_SignX_From_Vec;
		}
		/*
		 * If there already is an equivalent stream, then we arrange to
		 * equate the two streams.
		 */ 
		else {
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(signX, x);
				newEdgeToOnlyChild(signX, x->vec.strm);
				endGraphUpdate();
			}
			signX->x = x->vec.strm;

			/* we already have a stream, so we must equate them as above */
			switch (x->vec.strm->gen.tag.type) {
			case SIGNX :
				signX->force = force_To_SignX_From_SignX_Entry;
				break;
			case DIGSX :
				signX->force = force_To_SignX_From_DigsX;
				break;
			default :
				Error(FATAL, E_INT, "redirectSignX",
						"vector stream is not a stream");
			}
		}
		break;

	case MATX :
		/*
		 * This code is the same as that for the vector case
		 */
		if (x->matX.strm == NULL) {
			r = matrix_Z(x->matX.x,
					x->matX.mat[0][0], x->matX.mat[0][1],
					x->matX.mat[1][0], x->matX.mat[1][1]);
			signX->x = r;
			x->matX.strm = (Real) signX;
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(signX, x);
				newEdgeToOnlyChild(signX, r);
				drawEqEdge(signX, x);
				endGraphUpdate();
			}
			signX->force = force_To_SignX_From_MatX_Entry;
		}

		else {
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(signX, x);
				newEdgeToOnlyChild(signX, x->matX.strm);
				endGraphUpdate();
			}
			signX->x = x->matX.strm;

			/* we already have a stream, so we must equate them as above */
			switch (x->matX.strm->gen.tag.type) {
			case SIGNX :
				signX->force = force_To_SignX_From_SignX_Entry;
				break;
			case DIGSX :
				signX->force = force_To_SignX_From_DigsX;
				break;
			default :
				Error(FATAL, E_INT, "redirectSignX",
						"matrix stream is not a stream");
			}
		}
		break;

	case TENXY :
		/*
		 * This code is the same as that for the vector and matrix cases
		 * except that we don't need to worry about sharing and hence there
		 * is not need to make a copy of the tensor.
		 */
		if (x->tenXY.strm == NULL) {
			x->tenXY.strm = (Real) signX;
			signX->force = force_To_SignX_From_TenXY_Entry;
		}

		else {
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(signX, x);
				newEdgeToOnlyChild(signX, x->tenXY.strm);
				endGraphUpdate();
			}
			signX->x = x->tenXY.strm;

			/* we already have a stream, so we must equate them as above */
			switch (x->tenXY.strm->gen.tag.type) {
			case SIGNX :
				signX->force = force_To_SignX_From_SignX_Entry;
				break;
			case DIGSX :
				signX->force = force_To_SignX_From_DigsX;
				break;
			default :
				Error(FATAL, E_INT, "redirectSignX",
						"tensor stream is not a stream");
			}
		}
		break;
	default :
		Error(FATAL, E_INT, "redirectSignX",
			"value of alternation is not a real");
	}
}
