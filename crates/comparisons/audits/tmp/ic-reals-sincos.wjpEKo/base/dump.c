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
#include <math.h>

/*
 * Debugging routines for displaying heap cells. 
 *
 * This file needs a little cleaning up a bit.
 */

int dumpBase = 10;

static void dumpReal1(Real);
static void clearDumpedFlag(Real);

void
dumpTag(Tag tag)
{
	char *typeToString(ObjType);
	char *boolValToString(ObjType);

	fprintf(stderr, " id=%d", tag.nodeId);
	fprintf(stderr, " %s", typeToString(tag.type));
	fprintf(stderr, "%s", (tag.isSigned ? "(si)" : "----"));

	if (tag.type == PREDX || tag.type == BOOLX || tag.type == BOOLXY)
		fprintf(stderr, "%s", boolValToString(tag.value));

	if (tag.type == SIGNX)
		fprintf(stderr, "%s", signToString(tag.value));
}

void
dumpVector(Vector v)
{
	fprintf(stderr, " a=");
	mpz_out_str(stderr, dumpBase, v[0]);
	fprintf(stderr, "\n b=");
	mpz_out_str(stderr, dumpBase, v[1]);
	fprintf(stderr, "\n");
}

void
dumpMatrix(Matrix m)
{
	fprintf(stderr, " a=");
	mpz_out_str(stderr, dumpBase, m[0][0]);
	fprintf(stderr, "\n b=");
	mpz_out_str(stderr, dumpBase, m[0][1]);
	fprintf(stderr, "\n c=");
	mpz_out_str(stderr, dumpBase, m[1][0]);
	fprintf(stderr, "\n d=");
	mpz_out_str(stderr, dumpBase, m[1][1]);
	fprintf(stderr, "\n");
}

void
dumpTensor(Tensor t)
{
	fprintf(stderr, " a=");
	mpz_out_str(stderr, dumpBase, t[0][0]);
	fprintf(stderr, "\n b=");
	mpz_out_str(stderr, dumpBase, t[0][1]);
	fprintf(stderr, "\n c=");
	mpz_out_str(stderr, dumpBase, t[1][0]);
	fprintf(stderr, "\n d=");
	mpz_out_str(stderr, dumpBase, t[1][1]);
	fprintf(stderr, "\n e=");
	mpz_out_str(stderr, dumpBase, t[2][0]);
	fprintf(stderr, "\n f=");
	mpz_out_str(stderr, dumpBase, t[2][1]);
	fprintf(stderr, "\n g=");
	mpz_out_str(stderr, dumpBase, t[3][0]);
	fprintf(stderr, "\n h=");
	mpz_out_str(stderr, dumpBase, t[3][1]);
	fprintf(stderr, "\n");
}

void
dumpForceFunc(void (*func)())
{
	ForceFuncDesc *p;

	p = getDescForForceFunc(func);
	if (p == NULL)
		fprintf(stderr, " force=%x\n", func);
	else
		fprintf(stderr, " force=%s\n", p->funcName);
}

void
dumpMatX(MatX *matX)
{
	fprintf(stderr, " x=%x\n", (unsigned) matX->x);
	dumpForceFunc(matX->force);
	dumpMatrix(matX->mat);
}

void
dumpTenXY(TenXY *tenXY)
{
	fprintf(stderr, " fair=%d", tenXY->tensorFairness);
	fprintf(stderr, " x=%x y=%x\n", (unsigned) tenXY->x, (unsigned) tenXY->y);
	dumpForceFunc(tenXY->forceX);
	dumpForceFunc(tenXY->forceY);
	dumpTensor(tenXY->ten);
}

void
dumpSignX(SignX *signX)
{
	fprintf(stderr, " x=%x\n", (unsigned) signX->x);
	dumpForceFunc(signX->force);
}

void
dumpDigsX(DigsX *digsX)
{
	fprintf(stderr, " x=%x", (unsigned) digsX->x);
	fprintf(stderr, " count=%d", (int) digsX->count);
	fprintf(stderr, "\n word=");
#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		fprintf(stderr, "%d ", digsX->word.small);
		fprintf(stderr, "(0x%x) ", digsX->word.small);
	}
	else {
#endif
		mpz_out_str(stderr, 10, digsX->word.big);
		fprintf(stderr, " (0x");
		mpz_out_str(stderr, 16, digsX->word.big);
		fprintf(stderr, " )");
#ifdef PACK_DIGITS
	}
#endif
	fprintf(stderr, "\n");
	dumpForceFunc(digsX->force);
}

void
dumpCls(Cls *cls)
{
	fprintf(stderr, " redirect=%x\n", (unsigned) cls->redirect);
	dumpForceFunc(cls->force);
	dumpReal1(cls->redirect);
}

void
dumpAlt(Alt *alt)
{
	fprintf(stderr, " redirect=%x num ge=%d\n",
			(unsigned) alt->redirect, alt->numGE);
}

void
dumpAltChildren(Alt *alt)
{
	int i;

	for (i = 0; i < alt->numGE; i++) {
		if (alt->GE[i].guard != NULL) {
			fprintf(stderr, " (%x,%x)\n", (unsigned) alt->GE[i].guard,
								(unsigned) alt->GE[i].x);
		}
	}

	for (i = 0; i < alt->numGE; i++) {
		if (alt->GE[i].guard != NULL) {
			dumpBool(alt->GE[i].guard);
			dumpReal1(alt->GE[i].x);
		}
	}
}

void
dumpPredX(PredX *predX)
{
	fprintf(stderr, " x=%x\n", (unsigned) predX->x);
	dumpForceFunc(predX->force);
}

void
dumpBoolX(BoolX *boolX)
{
	fprintf(stderr, " x=%x\n", (unsigned) boolX->x);
	dumpForceFunc(boolX->force);
}

void
dumpBoolXY(BoolXY *boolXY)
{
	fprintf(stderr, " x=%x", (unsigned) boolXY->x);
	fprintf(stderr, " y=%x\n", (unsigned) boolXY->y);
	dumpForceFunc(boolXY->force);
}


void
dumpCell(void *p)
{
	Tag tag = *((Tag *) p);

	fprintf(stderr, "%x ", (unsigned) p);

	dumpTag(tag);

	switch (tag.type) {
		case ALT :
			dumpAlt((Alt *) p);
			break;
		case VECTOR :
			fprintf(stderr, "\n");
			dumpVector(((Vec *)p)->vec);
			break;
		case MATX :
			dumpMatX((MatX *) p);
			break;
		case TENXY :
			dumpTenXY((TenXY *) p);
			break;
		case DIGSX :
			dumpDigsX((DigsX *) p);
			break;
		case SIGNX :
			dumpSignX((SignX *) p);
			break;
		case CLOSURE :
			dumpCls((Cls *) p);
			break;
		case PREDX :
			dumpPredX((PredX *) p);
			break;
		case BOOLX :
			dumpBoolX((BoolX *) p);
			break;
		case BOOLXY :
			dumpBoolXY((BoolXY *) p);
			break;
		default :
			break;
	}
}

void
dumpReal(Real r)
{
	dumpReal1(r);
	clearDumpedFlag(r);
}

static void
dumpReal1(Real x)
{
	if (x == NULL)
		return;

	if (x->gen.tag.dumped)
		return;
	x->gen.tag.dumped = TRUE;

	fprintf(stderr, "%x ", (unsigned) x);
	dumpTag(x->gen.tag);

	switch (x->gen.tag.type) {
		case ALT :
			dumpAlt((Alt *) x);
			dumpAltChildren((Alt *) x);
			break;
			
		case VECTOR :
			fprintf(stderr, "\n");
			dumpVector(x->vec.vec);
			break;

		case MATX :
			dumpMatX((MatX *) x);
			dumpReal1((Real) x->matX.x);
			break;

		case TENXY :
			dumpTenXY((TenXY *) x);
			dumpReal1((Real) x->tenXY.x);
			dumpReal1((Real) x->tenXY.y);
			break;

		case SIGNX :
			dumpSignX((SignX *) x);
			dumpReal1(x->signX.x);
			break;

		case DIGSX :
			dumpDigsX((DigsX *) x);
			dumpReal1(x->digsX.x);
			break;

		case CLOSURE :
			fprintf(stderr, "\n");
			break;

		default :
			break;
	}
}

static void
clearDumpedFlag(Real x)
{
	int i;

	if (x == NULL)
		return;

	if (x->gen.tag.dumped == FALSE)
		return;

	x->gen.tag.dumped = FALSE;

	switch (x->gen.tag.type) {
		case ALT :
			for (i = 0; i < x->alt.numGE; i++)
				if (x->alt.GE[i].guard != NULL)
					clearDumpedFlag(x->alt.GE[i].x);
			break;
		case VECTOR :
			break;
		case MATX :
			clearDumpedFlag((Real) x->matX.x);
			break;
		case TENXY :
			clearDumpedFlag((Real) x->tenXY.x);
			clearDumpedFlag((Real) x->tenXY.y);
			break;
		case SIGNX :
			clearDumpedFlag(x->signX.x);
			break;
		case DIGSX :
			clearDumpedFlag(x->digsX.x);
			break;
		case CLOSURE :
			break;
		default :
			break;
	}
}

void
dumpBool(Bool b)
{
	if (b == NULL)
		return;

	fprintf(stderr, "%x ", (unsigned) b);
	dumpTag(b->gen.tag);

	switch (b->gen.tag.type) {
	case PREDX :
		dumpPredX((PredX *) b);
		dumpReal(b->predX.x);
		break;
	case BOOLX :
		dumpBoolX((BoolX *) b);
		dumpBool(b->boolX.x);
		break;
	case BOOLXY :
		dumpBoolXY((BoolXY *) b);
		dumpBool(b->boolXY.x);
		dumpBool(b->boolXY.y);
		break;
	default :
		break;
	}
}

