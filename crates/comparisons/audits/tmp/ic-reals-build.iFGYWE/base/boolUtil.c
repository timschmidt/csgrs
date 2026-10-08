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
 * Various utilities used by predicates and boolean operators.
 */
void
force_To_Bool_From_The_Abyss()
{
	Bool b;
	b = (Bool) POP;

	Error(FATAL, E_INT, "force_To_Bool_From_The_Abyss",
		"trying to force a boolean which is already known");
}

void
force_To_PredX_From_The_Abyss()
{
	PredX *predX;
	predX = (PredX *) POP;

	Error(FATAL, E_INT, "force_To_PredX_From_The_Abyss",
		"trying to force a predicate which is already known");
}


void
absorbDigsXIntoPredX(PredX *predX)
{
	DigsX *digsX;

	digsX = (DigsX *) predX->x;
	predX->x = digsX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(predX, digsX);
		newEdgeToOnlyChild(predX, digsX->x);
		endGraphUpdate();
	}
}

void
absorbSignXIntoPredX(PredX *predX)
{
	SignX *signX;

	signX = (SignX *) predX->x;
	predX->x = signX->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(predX, signX);
		newEdgeToOnlyChild(predX, signX->x);
		endGraphUpdate();
	}
}

void
setBoolXY(BoolXY *boolXY, BoolVal v)
{
	boolXY->tag.value = v;

	switch (v) {
	case LAZY_TRUE :
	case LAZY_FALSE :
		if (DAVINCI) {
			beginGraphUpdate();
			deleteEdgeToXChild(boolXY, boolXY->x);
			deleteEdgeToYChild(boolXY, boolXY->y);
			endGraphUpdate();
		}
		boolXY->force = force_To_Bool_From_The_Abyss;
		boolXY->x = NULL;
		boolXY->y = NULL;
		break;
	case LAZY_UNKNOWN :
		break;
	default :
		Error(FATAL, E_INT, "setBoolXY", "bad boolean value");
	}
}

void
setPredX(PredX *predX, BoolVal v)
{
	predX->tag.value = v;

	switch (v) {
	case LAZY_TRUE :
	case LAZY_FALSE :
		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(predX, predX->x);
			endGraphUpdate();
		}
		predX->force = force_To_Bool_From_The_Abyss;
		predX->x = NULL;
		break;
	case LAZY_UNKNOWN :
		break;
	default :
		Error(FATAL, E_INT, "setPredX", "bad boolean value");
	}
}

void
setBoolX(BoolX *boolX, BoolVal v)
{
	boolX->tag.value = v;

	switch (v) {
	case LAZY_TRUE :
	case LAZY_FALSE :
		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(boolX, boolX->x);
			endGraphUpdate();
		}
		boolX->force = force_To_Bool_From_The_Abyss;
		boolX->x = NULL;
		break;
	case LAZY_UNKNOWN :
		break;
	default :
		Error(FATAL, E_INT, "setBoolX", "bad boolean value");
	}
}

PredX *
allocPredX(Real x)
{
	PredX *predX;

	if ((predX = (PredX *) malloc(sizeof(PredX))) == NULL)
		Error(FATAL, E_INT, "allocPredX", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(predX);

	predX->tag.type = PREDX;
	predX->tag.value = LAZY_UNKNOWN;
	predX->tag.dumped = FALSE;
	predX->x = x;

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(predX, PREDX);
		newEdgeToOnlyChild(predX, x);
		endGraphUpdate();
	}

	return predX;
}

/*
 * This consumes a stream of characteristic pairs. If a pair (c,n) has the
 * property that c = 2n-1, then the boolean is LAZY_UNKNOWN, otherwise
 * it gets set to LAZY_TRUE and we advance to the next pair.
 */
void
force_To_PredX_From_DigsX_2n_minus_1_True_Entry()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_PredX_From_DigsX_2n_minus_1_True_Cont();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;
	
	PUSH_2(force_To_PredX_From_DigsX_2n_minus_1_True_Cont, predX);
	if (digsX->count == 0)
		PUSH_3(digsX->force, digsX, defaultForceCount);
}

/*
 * At this point we know we have some digits available, ie a characteristic
 * pair (c,n) with n > 0. The predicate is set LAZY_TRUE if c != 2n-1.
 * Otherwise we just arrange to consume more digits.
 */
void
force_To_PredX_From_DigsX_2n_minus_1_True_Cont()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_Bool_From_The_Abyss();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		if (digsX->word.small == (1 << digsX->count) - 1)
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_TRUE);
	}
	else {
#endif
		/*
		 * This is comparing a big word with +(2^n - 1). It would be faster
		 * to compare each word with 0xffffffff but this may have to
		 * wait.	####
		 */
		if (mpz_sgn(digsX->word.big) > 0 &&
							mpz_popcount(digsX->word.big) == digsX->count)
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_TRUE);
#ifdef PACK_DIGITS
	}
#endif
}

/*
 * This consumes a stream of characteristic pairs. If a pair (c,n) has the
 * property that c = 2n-1, then the boolean is LAZY_UNKNOWN, otherwise
 * it gets set to LAZY_FALSE and we advance to the next pair.
 */
void
force_To_PredX_From_DigsX_2n_minus_1_False_Entry()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_PredX_From_DigsX_2n_minus_1_False_Cont();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;
	
	PUSH_2(force_To_PredX_From_DigsX_2n_minus_1_False_Cont, predX);
	if (digsX->count == 0)
		PUSH_3(digsX->force, digsX, defaultForceCount);
}

/*
 * At this point we know we have some digits available, ie a characteristic
 * pair (c,n) with n > 0. The predicate is set LAZY_FALSE if c != 2n-1.
 * Otherwise we just arrange to consume more digits.
 */
void
force_To_PredX_From_DigsX_2n_minus_1_False_Cont()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_Bool_From_The_Abyss();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		if (digsX->word.small == (1 << digsX->count) - 1)
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_FALSE);
	}
	else {
#endif
		/*
		 * This is comparing a big word with +(2^n - 1). It would be faster
		 * to compare each word with 0xffffffff but this may have to
		 * wait.	####
		 */
		if (mpz_sgn(digsX->word.big) > 0 &&
							mpz_popcount(digsX->word.big) == digsX->count)
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_FALSE);
#ifdef PACK_DIGITS
	}
#endif
}

/*
 * This consumes a stream of characteristic pairs. If a pair (c,n) has the
 * property that -c = 2n-1, then the boolean is LAZY_UNKNOWN, otherwise
 * it gets set to LAZY_FALSE and we advance to the next pair.
 */
void
force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_PredX_From_DigsX_minus_2n_minus_1_False_Cont();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;
	
	PUSH_2(force_To_PredX_From_DigsX_minus_2n_minus_1_False_Cont, predX);
	if (digsX->count == 0)
		PUSH_3(digsX->force, digsX, defaultForceCount);
}

/*
 * At this point we know we have some digits available, ie a characteristic
 * pair (c,n) with n > 0. The predicate is set LAZY_FALSE if -c != 2n-1.
 * Otherwise we just arrange to consume more digits.
 */
void
force_To_PredX_From_DigsX_minus_2n_minus_1_False_Cont()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_Bool_From_The_Abyss();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		if (digsX->word.small == -((1 << digsX->count) - 1))
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_FALSE);
	}
	else {
#endif
		/*
		 * This is comparing a big word with -(2^n - 1).
		 *
		 * THIS RELIES ON GMP USING SIGN/MAGNITUDE REPRESENTATION.
		 */
		if (mpz_sgn(digsX->word.big) < 0 &&
							mpz_popcount(digsX->word.big) == digsX->count)
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_FALSE);
#ifdef PACK_DIGITS
	}
#endif
}

/*
 * This consumes a stream of characteristic pairs. If a pair (c,n) has the
 * property that -c = 2n-1, then the boolean is LAZY_UNKNOWN, otherwise
 * it gets set to LAZY_TRUE and we advance to the next pair.
 */
void
force_To_PredX_From_DigsX_minus_2n_minus_1_True_Entry()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_PredX_From_DigsX_minus_2n_minus_1_True_Cont();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;
	
	PUSH_2(force_To_PredX_From_DigsX_minus_2n_minus_1_True_Cont, predX);
	if (digsX->count == 0)
		PUSH_3(digsX->force, digsX, defaultForceCount);
}

/*
 * At this point we know we have some digits available, ie a characteristic
 * pair (c,n) with n > 0. The predicate is set LAZY_TRUE if -c != 2n-1.
 * Otherwise we just arrange to consume more digits.
 */
void
force_To_PredX_From_DigsX_minus_2n_minus_1_True_Cont()
{
	PredX *predX;
	DigsX *digsX;
	void force_To_Bool_From_The_Abyss();

	predX = (PredX *) POP;
	digsX = (DigsX *) predX->x;

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		if (digsX->word.small == -((1 << digsX->count) - 1))
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_TRUE);
	}
	else {
#endif
		/*
		 * This is comparing a big word with -(2^n - 1).
		 *
		 * THIS RELIES ON GMP USING SIGN/MAGNITUDE REPRESENTATION.
		 */
		if (mpz_sgn(digsX->word.big) < 0 &&
							mpz_popcount(digsX->word.big) == digsX->count)
			absorbDigsXIntoPredX(predX);
		else
			setPredX(predX, LAZY_TRUE);
#ifdef PACK_DIGITS
	}
#endif
}
