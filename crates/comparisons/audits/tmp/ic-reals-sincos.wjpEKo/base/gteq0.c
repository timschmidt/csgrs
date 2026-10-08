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
 * This file contains everything to do with the predicate x >= 0.
 */

#define forceGtEqZero_To_PredX_From_DigsX_SPOS_Entry \
			force_To_PredX_From_DigsX_2n_minus_1_True_Entry

void setPredX(PredX *, BoolVal);
void absorbDigsXIntoPredX(PredX *);
void absorbSignXIntoPredX(PredX *);
void force_To_PredX_From_The_Abyss();
static BoolVal gtEq_Vec_0(Vec *);
static BoolVal gtEq_MatX_0(MatX *);

void setGtZeroPredXMethod(PredX *);

/*
 * There are many cases when computing predicates over the reals. First of
 * all, we handle vectors, matrices and streams differently. To understand
 * the vector and matrices, assume a function (sgn n) which yields the
 * sign (-1,0,1) of a large integer. Vectors are simple, For matrices,
 * we need to determine if (info m) is in the interval [0,infty). This can
 * be done by inspecting only the signs of the entries in the matrix. The
 * proof that the following algorithm works is straightforward bearing in
 * mind the definition of (info m). Note that this applies only when
 * the argument of the matrix is unsigned.
 * 
 * mat_gtEq_0 ((a,b),(c,d)) =
 *   if ((sgn a) * (sgn b) >= 0)
 * 	  && ((sgn c) * (sgn d) >= 0) && ((sgn b) * (sgn d) > 0) then
 *     True else
 *     if ((sgn a) * (sgn b) < 0)
 * 	  && ((sgn c) * (sgn d) < 0) && ((sgn b) * (sgn d) > 0) then
 *        False else
 *        Unknown
 * 
 * The conditionals can be sorted to eliminate redundant comparisons to yield
 * the following decision procedure.
 *  
 * mat_gtEq_0 ((a,b),(c,d)) =
 *   if (sgn b) * (sgn d) > 0 then
 *     if (sgn a) * (sgn b) >= 0 then
 *       if (sgn c) * (sgn d) >= 0 then
 *         True
 *       else
 *         Unkown
 *     else
 *       if (sgn c) * (sgn d) < 0 then
 *         False
 *       else
 *         Unknown
 *   else
 *     Unknown
 * 
 * No doubt a similar scheme works for tensors but this has not been coded.
 * Right now we deal with the digit streams emitted from the tensor. The
 * algorithm for handling digit streams is described elsewhere.
 */

Bool
gtEq_R_0(Real x)
{
	PredX *predX;
	PredX *allocPredX();
	void setGtEqZeroPredXMethod(PredX *);

	predX = allocPredX(x);
	setGtEqZeroPredXMethod(predX);
	return (Bool) predX;
}

void
setGtEqZeroPredXMethod(PredX *predX)
{
	void forceGtEqZero_To_PredX_From_SignX_Entry();
	void forceGtEqZero_To_PredX_From_DigsX_SPOS_Entry();
	void forceGtEqZero_To_PredX_From_MatX_Entry();
	void forceGtEqZero_To_PredX_From_MatX_Signed_Entry();
	void forceGtEqZero_To_PredX_From_TenXY();
	void forceGtEqZero_To_PredX_From_Alt_Entry();
	void forceGtEqZero_To_PredX_From_Cls_Entry();

	switch (predX->x->gen.tag.type) {
	case VECTOR :
		setPredX(predX, gtEq_Vec_0((Vec *) predX->x));
		predX->force = force_To_PredX_From_The_Abyss;
		break;
	case MATX :
		if (predX->x->matX.x->gen.tag.isSigned)
			predX->force = forceGtEqZero_To_PredX_From_MatX_Signed_Entry;
		else {
			setPredX(predX, gtEq_MatX_0((MatX *) predX->x));
			predX->force = forceGtEqZero_To_PredX_From_MatX_Entry;
		}
		break;
	case TENXY :
		predX->force = forceGtEqZero_To_PredX_From_TenXY;
		break;
	case SIGNX :
		predX->force = forceGtEqZero_To_PredX_From_SignX_Entry;
		break;
	case DIGSX :
		predX->force = forceGtEqZero_To_PredX_From_DigsX_SPOS_Entry;
		break;
	case CLOSURE :
		predX->force = forceGtEqZero_To_PredX_From_Cls_Entry;
		break;
	case ALT :
		predX->force = forceGtEqZero_To_PredX_From_Alt_Entry;
		break;
	default :
		Error(FATAL, E_INT, "compareGtEqZero", "argument is not a stream");
	}
}

void
forceGtEqZero_To_PredX_From_Alt_Entry()
{
	PredX *predX;
	Alt *alt;
	void force_To_Alt_Entry();
	void forceGtEqZero_To_PredX_From_Alt_Cont();

	predX = (PredX *) POP;
	alt = (Alt *) predX->x;
	
	PUSH_2(forceGtEqZero_To_PredX_From_Alt_Cont, predX);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (alt->redirect == NULL)
		PUSH_2(force_To_Alt_Entry, alt);
}

void
forceGtEqZero_To_PredX_From_Alt_Cont()
{
	PredX *predX;
	Alt *alt;

	predX = (PredX *) POP;
	alt = (Alt *) predX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(predX, alt);
		newEdgeToOnlyChild(predX, alt->redirect);
		endGraphUpdate();
	}
	predX->x = alt->redirect;
	setGtEqZeroPredXMethod(predX);

	PUSH_2(predX->force, predX);
}

void
forceGtEqZero_To_PredX_From_Cls_Entry()
{
	PredX *predX;
	Cls *cls;
	void forceGtEqZero_To_PredX_From_Cls_Cont();

	predX = (PredX *) POP;
	cls = (Cls *) predX->x;
	
	PUSH_2(forceGtEqZero_To_PredX_From_Cls_Cont, predX);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to force it.
	 */
	if (cls->redirect == NULL)
		PUSH_2(cls->force, cls);
}

void
forceGtEqZero_To_PredX_From_Cls_Cont()
{
	PredX *predX;
	Cls *cls;

	predX = (PredX *) POP;
	cls = (Cls *) predX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(predX, cls);
		newEdgeToOnlyChild(predX, cls->redirect);
		endGraphUpdate();
	}
	predX->x = cls->redirect;
	setGtEqZeroPredXMethod(predX);

	PUSH_2(predX->force, predX);
}

/*
 * If we get here then we need to get more information into the matrix.
 */
void
forceGtEqZero_To_PredX_From_MatX_Entry()
{
	PredX *predX;
	MatX *matX;
	void forceGtEqZero_To_PredX_From_MatX_Cont();

	predX = (PredX *) POP;
	matX = (MatX *) predX->x;

	if (matX->tag.type == VECTOR) {
		setPredX(predX, gtEq_Vec_0((Vec *) matX));
		predX->force = force_To_PredX_From_The_Abyss;
		return;
	}

	PUSH_2(forceGtEqZero_To_PredX_From_MatX_Cont, predX);
	PUSH_3(predX->x->matX.force, predX->x, defaultForceCount);
}

void
forceGtEqZero_To_PredX_From_MatX_Cont()
{
	PredX *predX;
	MatX *matX;

	predX = (PredX *) POP;
	matX = (MatX *) predX->x;

	if (matX->tag.type == VECTOR) {
		setPredX(predX, gtEq_Vec_0((Vec *) matX));
		predX->force = force_To_PredX_From_The_Abyss;
		return;
	}
	setPredX(predX, gtEq_MatX_0((MatX *) (predX->x)));
}

/*
 * If we get here then we need to get more information into the matrix.
 */
void
forceGtEqZero_To_PredX_From_MatX_Signed_Entry()
{
	PredX *predX;
	MatX *matX;
	void forceGtEqZero_To_PredX_From_MatX_Signed_Cont();

	predX = (PredX *) POP;
	matX = (MatX *) predX->x;

	if (matX->tag.type == VECTOR) {
		setPredX(predX, gtEq_Vec_0((Vec *) matX));
		predX->force = force_To_PredX_From_The_Abyss;
		return;
	}

	PUSH_2(forceGtEqZero_To_PredX_From_MatX_Signed_Cont, predX);

	if (matX->x->gen.tag.isSigned)
		PUSH_2(matX->force, matX);
}

void
forceGtEqZero_To_PredX_From_MatX_Signed_Cont()
{
	PredX *predX;
	MatX *matX;
	void forceGtEqZero_To_PredX_From_MatX_Entry();

	predX = (PredX *) POP;
	matX = (MatX *) predX->x;

	if (matX->tag.type == VECTOR) {
		setPredX(predX, gtEq_Vec_0((Vec *) matX));
		predX->force = force_To_PredX_From_The_Abyss;
		return;
	}
	predX->force = forceGtEqZero_To_PredX_From_MatX_Entry;
	setPredX(predX, gtEq_MatX_0(matX));
}

void
forceGtEqZero_To_PredX_From_TenXY()
{
	PredX *predX;
	TenXY *tenXY;
	void forceGtEqZero_To_PredX_From_SignX_Entry();
	void forceGtEqZero_To_PredX_From_DigsX_SPOS_Entry();

	predX = (PredX *) POP;
	tenXY = (TenXY *) predX->x;

	/*
	 * The tensor may have reduced to a vector
	 */
	if (tenXY->tag.type == VECTOR) {
		setPredX(predX, gtEq_Vec_0((Vec *) tenXY));
		predX->force = force_To_PredX_From_The_Abyss;
		return;
	}

	/*
	 * The tensor may have reduced to a matrix (signed or otherwise)
	 */
	if (tenXY->tag.type == MATX) {
		if (predX->x->matX.x->gen.tag.isSigned) {
			predX->force = forceGtEqZero_To_PredX_From_MatX_Signed_Entry;
			PUSH_2(predX->force, predX);
		}
		else {
			setPredX(predX, gtEq_MatX_0((MatX *) predX->x));
			predX->force = forceGtEqZero_To_PredX_From_MatX_Entry;
		}
		return;
	}

	if (tenXY->tag.isSigned) {
		createSignedStreamForTenXY(tenXY);
		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(predX, predX->x);
			newEdgeToOnlyChild(predX, tenXY->strm);
			endGraphUpdate();
		}
		predX->x = tenXY->strm;
		predX->force = forceGtEqZero_To_PredX_From_SignX_Entry;
		PUSH_2(predX->force, predX);
	}
	else {
		createUnsignedStreamForTenXY(tenXY);
		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(predX, predX->x);
			newEdgeToOnlyChild(predX, tenXY->strm);
			endGraphUpdate();
		}
		predX->x = tenXY->strm;
		predX->force = forceGtEqZero_To_PredX_From_DigsX_SPOS_Entry;
		PUSH_2(predX->force, predX);
	}
}

void
forceGtEqZero_To_PredX_From_SignX_Entry()
{
	PredX *predX;
	SignX *signX;
	void forceGtEqZero_To_PredX_From_SignX_Cont();

	predX = (PredX *) POP;
	signX = (SignX *) predX->x;

	PUSH_2(forceGtEqZero_To_PredX_From_SignX_Cont, predX);

	if (signX->tag.value == SIGN_UNKN)
		PUSH_2(signX->force, signX);
}

/*
 * At this point we know the sign is valid so we ``absorb'' it. In this
 * case that means that we have to inspect the sign and reset the methods
 * before advancing to the first characteristic pair.
 */
void
forceGtEqZero_To_PredX_From_SignX_Cont()
{
	PredX *predX;
	SignX *signX;
	void forceGtEqZero_To_PredX_From_DigsX_SPOS_Entry();
	void forceGtEqZero_To_PredX_From_DigsX_SNEG_Entry();
	void forceGtEqZero_To_PredX_From_DigsX_SZERO_Entry();
	void forceGtEqZero_To_PredX_From_DigsX_SINF_Entry();

	predX = (PredX *) POP;
	signX = (SignX *) predX->x;

	switch (signX->tag.value) {
	case SPOS :
		predX->force = forceGtEqZero_To_PredX_From_DigsX_SPOS_Entry;
		break;
	case SNEG :
		predX->force = forceGtEqZero_To_PredX_From_DigsX_SNEG_Entry;
		break;
	case SZERO :
		predX->force = forceGtEqZero_To_PredX_From_DigsX_SZERO_Entry;
		break;
	case SINF :
		predX->force = forceGtEqZero_To_PredX_From_DigsX_SINF_Entry;
		break;
	default :
		Error(FATAL, E_INT, "forceGtEqZero_To_PredX_From_SignX_Cont",
				"invalid sign");
	}

	absorbSignXIntoPredX(predX);

	PUSH_2(predX->force, predX);
}

void
forceGtEqZero_To_PredX_From_DigsX_SNEG_Entry()
{
	PredX *predX;
	DigsX *digsX;
	void forceGtEqZero_To_PredX_From_DigsX_SNEG_Cont();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;
	
	PUSH_2(forceGtEqZero_To_PredX_From_DigsX_SNEG_Cont, predX);
	if (digsX->count == 0)
		PUSH_3(digsX->force, digsX, defaultForceCount);
}

void
forceGtEqZero_To_PredX_From_DigsX_SNEG_Cont()
{
	PredX *predX;
	DigsX *digsX;
	int k;
	void force_To_PredX_From_DigsX_2n_minus_1_False_Entry();
	void force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		k = (1 << digsX->count) - 1;
		if (digsX->word.small == k) {
			absorbDigsXIntoPredX(predX);
			predX->force = force_To_PredX_From_DigsX_2n_minus_1_False_Entry;
		}
		else {
 			if (digsX->word.small == -k) {
				absorbDigsXIntoPredX(predX);
				predX->force =
						force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry;
			}
			else
				setPredX(predX, LAZY_FALSE);
		}
	}
	else {
#endif
		/*
		 * This is comparing a big word with +-(2^n - 1). It would be faster
		 * to compare each word with 0xffffffff but this may have to
		 * wait.	####
		 */
		if (mpz_sgn(digsX->word.big) >= 0) {
			if (mpz_popcount(digsX->word.big) == digsX->count) {
				absorbDigsXIntoPredX(predX);
				predX->force = force_To_PredX_From_DigsX_2n_minus_1_False_Entry;
			}
			else
				setPredX(predX, LAZY_FALSE);
		}
		else {
			/*
			 * We negate things here but I don't think it is necessary
			 * since GMP uses sign and magnitude representation 
			 * for big integers. Leave it for now.
			 */
			mpz_neg(digsX->word.big, digsX->word.big);
			if (mpz_popcount(digsX->word.big) == digsX->count) {
				absorbDigsXIntoPredX(predX);
				predX->force =
						force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry;
			}
			else
				setPredX(predX, LAZY_FALSE);
			mpz_neg(digsX->word.big, digsX->word.big);
		}
#ifdef PACK_DIGITS
	}
#endif
}

void
forceGtEqZero_To_PredX_From_DigsX_SZERO_Entry()
{
	PredX *predX;
	DigsX *digsX;
	void forceGtEqZero_To_PredX_From_DigsX_SZERO_Cont();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

	PUSH_2(forceGtEqZero_To_PredX_From_DigsX_SZERO_Cont, predX);
	if (digsX->count == 0)
		PUSH_3(digsX->force, digsX, defaultForceCount);
}

void
forceGtEqZero_To_PredX_From_DigsX_SZERO_Cont()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_PredX_From_DigsX_2n_minus_1_False_Entry();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		if (digsX->word.small > 0)
			setPredX(predX, LAZY_TRUE);
		else {
			if (digsX->word.small == 0)
				absorbDigsXIntoPredX(predX);
			else
				if (digsX->word.small == -1) {
					absorbDigsXIntoPredX(predX);
					predX->force =
							force_To_PredX_From_DigsX_2n_minus_1_False_Entry;
				}
				else
					setPredX(predX, LAZY_FALSE);
		}
	}
	else {
#endif
		switch (mpz_sgn(digsX->word.big)) {
		case -1 :
			if (mpz_cmp_si(digsX->word.big, -1) < 0)
				setPredX(predX, LAZY_FALSE);
			else { /* word == -1 */
				absorbDigsXIntoPredX(predX);
				predX->force =
						force_To_PredX_From_DigsX_2n_minus_1_False_Entry;
			}
			break;
		case 0 :
			absorbDigsXIntoPredX(predX);
			break;
		case 1 :
			setPredX(predX, LAZY_TRUE);
			break;
		default :
			Error(FATAL, E_INT, "forceGtEqZero_To_PredX_From_DigsX_SZERO_Cont",
					"bad value returned from mpz_sgn");
		}
#ifdef PACK_DIGITS
	}
#endif
}

void
forceGtEqZero_To_PredX_From_DigsX_SINF_Entry()
{
	PredX *predX;
	DigsX *digsX;
	void forceGtEqZero_To_PredX_From_DigsX_SINF_Cont();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

	PUSH_2(forceGtEqZero_To_PredX_From_DigsX_SINF_Cont, predX);
	if (digsX->count == 0)
		PUSH_3(digsX->force, digsX, defaultForceCount);
}

void
forceGtEqZero_To_PredX_From_DigsX_SINF_Cont()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_PredX_From_DigsX_2n_minus_1_True_Entry();
	void force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		if (digsX->word.small > 1)
			setPredX(predX, LAZY_FALSE);
		else {
			if (digsX->word.small == 1) {
				absorbDigsXIntoPredX(predX);
				predX->force =
					force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry;
			}
			else {
				if (digsX->word.small == 0)
					absorbDigsXIntoPredX(predX);
				else {
					if (digsX->word.small == -1) {
						absorbDigsXIntoPredX(predX);
						predX->force =
							force_To_PredX_From_DigsX_2n_minus_1_True_Entry;
					}
					else
						setPredX(predX, LAZY_TRUE);
				}
			}
		}
	}
	else {
#endif
		switch (mpz_sgn(digsX->word.big)) {
		case -1 :
			if (mpz_cmp_si(digsX->word.big, -1) < 0)
				setPredX(predX, LAZY_TRUE);
			else { /* word == -1 */
				absorbDigsXIntoPredX(predX);
				predX->force =
						force_To_PredX_From_DigsX_2n_minus_1_True_Entry;
			}
			break;
		case 0 :
			absorbDigsXIntoPredX(predX);
			break;
		case 1 :
			if (mpz_cmp_si(digsX->word.big, 1) > 0)
				setPredX(predX, LAZY_FALSE);
			else { /* word == 1 */
				absorbDigsXIntoPredX(predX);
				predX->force =
						force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry;
			}
			break;
		default :
			Error(FATAL, E_INT, "forceGtEqZero_To_PredX_From_DigsX_SINF_Cont",
					"bad value returned from mpz_sgn");
		}
#ifdef PACK_DIGITS
	}
#endif
}

static BoolVal
gtEq_MatX_0(MatX *matX)
{
	if (mpz_sgn(matX->mat[0][1]) * mpz_sgn(matX->mat[1][1]) > 0) {
		if (mpz_sgn(matX->mat[0][0]) * mpz_sgn(matX->mat[0][1]) >= 0) {
			if (mpz_sgn(matX->mat[1][0]) * mpz_sgn(matX->mat[1][1]) >= 0)
				return LAZY_TRUE;
		}
		else {
			if (mpz_sgn(matX->mat[1][0]) * mpz_sgn(matX->mat[1][1]) < 0)
				return LAZY_FALSE;
		}
	}
	return LAZY_UNKNOWN;
}

static BoolVal
gtEq_Vec_0(Vec *vec)
{
	if (mpz_sgn(vec->vec[1]) != 0) {
		if (mpz_sgn(vec->vec[0]) * mpz_sgn(vec->vec[1]) >= 0)
			return LAZY_TRUE;
		else
			return LAZY_FALSE;
	}
	if (mpz_sgn(vec->vec[0]) == 0)
		Error(FATAL, E_INT, "gtEq_Vec_0", "malformed vector");

	return LAZY_UNKNOWN;
}

