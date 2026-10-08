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
 * Here is everything to do with the unary and binary boolean operators.
 */

BoolXY *
allocBoolXY(Bool x, Bool y)
{
	BoolXY *boolXY;

	if ((boolXY = (BoolXY *) malloc(sizeof(BoolXY))) == NULL)
		Error(FATAL, E_INT, "allocBoolXY", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(boolXY);

	boolXY->tag.type = BOOLXY;
	boolXY->tag.value = LAZY_UNKNOWN;
	boolXY->tag.dumped = FALSE;
	boolXY->x = x;
	boolXY->y = y;

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(boolXY, BOOLXY);
		newEdgeToXChild(boolXY, x);
		newEdgeToYChild(boolXY, y);
		endGraphUpdate();
	}

	return boolXY;
}

BoolX *
allocBoolX(Bool x)
{
	BoolX *boolX;

	if ((boolX = (BoolX *) malloc(sizeof(BoolX))) == NULL)
		Error(FATAL, E_INT, "allocBoolX", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(boolX);

	boolX->tag.type = BOOLX;
	boolX->tag.value = LAZY_UNKNOWN;
	boolX->tag.dumped = FALSE;
	boolX->x = x;

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(boolX, BOOLX);
		newEdgeToOnlyChild(boolX, x);
		endGraphUpdate();
	}

	return boolX;
}

Bool
and_B_B(Bool x, Bool y)
{
	BoolXY *boolXY;
	void force_To_And_From_Bool_X_Entry();

	boolXY = allocBoolXY(x, y);
	boolXY->force = force_To_And_From_Bool_X_Entry;
	return (Bool) boolXY;
}

Bool
or_B_B(Bool x, Bool y)
{
	BoolXY *boolXY;
	void force_To_Or_From_Bool_X_Entry();

	boolXY = allocBoolXY(x, y);
	boolXY->force = force_To_Or_From_Bool_X_Entry;
	return (Bool) boolXY;
}

Bool
not_B(Bool x)
{
	BoolX *boolX;
	void force_To_Not_From_Bool_Entry();

	boolX = allocBoolX(x);
	boolX->force = force_To_Not_From_Bool_Entry;
	return (Bool) boolX;
}

void
force_To_Not_From_Bool_Entry()
{
	BoolX *boolX;
	Bool x;
	void force_To_Not_From_Bool_Cont();

	boolX = (BoolX *) POP;
	x = boolX->x;

	PUSH_2(force_To_Not_From_Bool_Cont, boolX);
	if (x->gen.tag.value == LAZY_UNKNOWN)
		PUSH_2(x->gen.force, x);
}

void
force_To_Not_From_Bool_Cont()
{
	BoolX *boolX;
	Bool x;

	boolX = (BoolX *) POP;
	x = boolX->x;

	switch (x->gen.tag.value) {
	case LAZY_TRUE :
		setBoolX(boolX, LAZY_FALSE);
		break;
	case LAZY_FALSE :
		setBoolX(boolX, LAZY_TRUE);
		break;
	case LAZY_UNKNOWN :
		break;
	default :
		Error(FATAL, E_INT, "force_To_Not_From_Bool_Cont", "bad boolean value");
	}
}

void
force_To_And_From_Bool_X_Entry()
{
	BoolXY *boolXY;
	Bool x;
	void force_To_And_From_Bool_X_Cont();

	boolXY = (BoolXY *) POP;
	x = boolXY->x;

	PUSH_2(force_To_And_From_Bool_X_Cont, boolXY);
	if (x->gen.tag.value == LAZY_UNKNOWN)
		PUSH_2(x->gen.force, x);
}

void
force_To_And_From_Bool_X_Cont()
{
	BoolXY *boolXY;
	Bool x;
	void force_To_And_From_Bool_Y_Entry();
	void force_From_Bool_Y_Only_Entry();

	boolXY = (BoolXY *) POP;
	x = boolXY->x;

	switch (x->gen.tag.value) {
	case LAZY_TRUE :
		boolXY->force = force_From_Bool_Y_Only_Entry;
		break;
	case LAZY_FALSE :
		setBoolXY(boolXY, LAZY_FALSE);
		break;
	case LAZY_UNKNOWN :
		boolXY->force = force_To_And_From_Bool_Y_Entry;
		break;
	default :
		Error(FATAL, E_INT, "force_To_And_From_Bool_X_Cont",
				"bad boolean value");
	}
}

void
force_To_And_From_Bool_Y_Entry()
{
	BoolXY *boolXY;
	Bool y;
	void force_To_And_From_Bool_Y_Cont();

	boolXY = (BoolXY *) POP;
	y = boolXY->y;

	PUSH_2(force_To_And_From_Bool_Y_Cont, boolXY);
	if (y->gen.tag.value == LAZY_UNKNOWN)
		PUSH_2(y->gen.force, y);
}

void
force_To_And_From_Bool_Y_Cont()
{
	BoolXY *boolXY;
	Bool y;
	void force_To_And_From_Bool_X_Entry();
	void force_From_Bool_X_Only_Entry();

	boolXY = (BoolXY *) POP;
	y = boolXY->y;

	switch (y->gen.tag.value) {
	case LAZY_TRUE :
		boolXY->force = force_From_Bool_X_Only_Entry;
		break;
	case LAZY_FALSE :
		setBoolXY(boolXY, LAZY_FALSE);
		break;
	case LAZY_UNKNOWN :
		boolXY->force = force_To_And_From_Bool_X_Entry;
		break;
	default :
		Error(FATAL, E_INT, "force_To_And_From_Bool_Y_Cont",
				"bad boolean value");
	}
}

void
force_To_Or_From_Bool_X_Entry()
{
	BoolXY *boolXY;
	Bool x;
	void force_To_Or_From_Bool_X_Cont();

	boolXY = (BoolXY *) POP;
	x = boolXY->x;

	PUSH_2(force_To_Or_From_Bool_X_Cont, boolXY);
	if (x->gen.tag.value == LAZY_UNKNOWN)
		PUSH_2(x->gen.force, x);
}

void
force_To_Or_From_Bool_X_Cont()
{
	BoolXY *boolXY;
	Bool x;
	void force_To_Or_From_Bool_Y_Entry();
	void force_From_Bool_Y_Only_Entry();

	boolXY = (BoolXY *) POP;
	x = boolXY->x;

	switch (x->gen.tag.value) {
	case LAZY_TRUE :
		setBoolXY(boolXY, LAZY_TRUE);
		break;
	case LAZY_FALSE :
		boolXY->force = force_From_Bool_Y_Only_Entry;
		break;
	case LAZY_UNKNOWN :
		boolXY->force = force_To_Or_From_Bool_Y_Entry;
		break;
	default :
		Error(FATAL, E_INT, "force_To_Or_From_Bool_X_Cont",
				"bad boolean value");
	}
}

void
force_To_Or_From_Bool_Y_Entry()
{
	BoolXY *boolXY;
	Bool y;
	void force_To_Or_From_Bool_Y_Cont();

	boolXY = (BoolXY *) POP;
	y = boolXY->y;

	PUSH_2(force_To_Or_From_Bool_Y_Cont, boolXY);
	if (y->gen.tag.value == LAZY_UNKNOWN)
		PUSH_2(y->gen.force, y);
}

void
force_To_Or_From_Bool_Y_Cont()
{
	BoolXY *boolXY;
	Bool y;
	void force_To_Or_From_Bool_X_Entry();
	void force_From_Bool_X_Only_Entry();

	boolXY = (BoolXY *) POP;
	y = boolXY->y;

	switch (y->gen.tag.value) {
	case LAZY_TRUE :
		setBoolXY(boolXY, LAZY_TRUE);
		break;
	case LAZY_FALSE :
		boolXY->force = force_From_Bool_X_Only_Entry;
		break;
	case LAZY_UNKNOWN :
		boolXY->force = force_To_Or_From_Bool_X_Entry;
		break;
	default :
		Error(FATAL, E_INT, "force_To_Or_From_Bool_Y_Cont",
				"bad boolean value");
	}
}

void
force_From_Bool_X_Only_Entry()
{
	BoolXY *boolXY;
	Bool x;
	void force_From_Bool_X_Only_Cont();

	boolXY = (BoolXY *) POP;
	x = boolXY->x;

	PUSH_2(force_From_Bool_X_Only_Cont, boolXY);
	if (x->gen.tag.value == LAZY_UNKNOWN)
		PUSH_2(x->gen.force, x);
}

void
force_From_Bool_X_Only_Cont()
{
	BoolXY *boolXY;
	Bool x;

	boolXY = (BoolXY *) POP;
	x = boolXY->x;

	switch (x->gen.tag.value) {
	case LAZY_TRUE :
		setBoolXY(boolXY, LAZY_TRUE);
		break;
	case LAZY_FALSE :
		setBoolXY(boolXY, LAZY_FALSE);
		break;
	case LAZY_UNKNOWN :
		break;
	default :
		Error(FATAL, E_INT, "force_From_Bool_X_Only_Cont",
				"bad boolean value");
	}
}

void
force_From_Bool_Y_Only_Entry()
{
	BoolXY *boolXY;
	Bool y;
	void force_From_Bool_Y_Only_Cont();

	boolXY = (BoolXY *) POP;
	y = boolXY->y;

	PUSH_2(force_From_Bool_Y_Only_Cont, boolXY);
	if (y->gen.tag.value == LAZY_UNKNOWN)
		PUSH_2(y->gen.force, y);
}

void
force_From_Bool_Y_Only_Cont()
{
	BoolXY *boolXY;
	Bool y;

	boolXY = (BoolXY *) POP;
	y = boolXY->y;

	switch (y->gen.tag.value) {
	case LAZY_TRUE :
		setBoolXY(boolXY, LAZY_TRUE);
		break;
	case LAZY_FALSE :
		setBoolXY(boolXY, LAZY_FALSE);
		break;
	case LAZY_UNKNOWN :
		break;
	default :
		Error(FATAL, E_INT, "force_From_Bool_Y_Only_Cont",
				"bad boolean value");
	}
}
