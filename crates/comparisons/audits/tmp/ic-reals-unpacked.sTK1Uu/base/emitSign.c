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
 * This file deals with emitting signs from LFTs. The algorithm is similar
 * to digit emission. See ``Notes on sign and digit emission'' for details.
 */

/*
 * This version of emitSign favours SZERO over all other signs. This
 * is because most of the analytic functions prefer SZERO.
 */
typedef struct TE {
	Sign sign;
	Comparison compare;
	void (*residual)(Vector); /* the function to compute the residual */
	struct TE *next; /* computes next entry in the sequence */
} TblEntry;

static bool mergeTwoLists(TblEntry **, TblEntry **, Comparison *);
static bool mergeFourLists(TblEntry **, TblEntry **, TblEntry **, TblEntry **);
static TblEntry *setList(Vector);

static mpz_t tmp_z;

/*
 * Functions prefixed with 'r' compute the residual vectors. This is done
 * "in-place". By this we mean that once we have decided that a sign 's'
 * can be emitted from a vector 'v', then we overwrite 'v' with residual
 * after 's' is emitted.
 */

static void rNoOp(Vector);

#define rgg rNoOp
#define regSPOS rNoOp
#define rgeSPOS rNoOp

static void regSNEG(Vector);
static void regSZERO(Vector);
static void relSPOS(Vector);
static void relSNEG(Vector);
static void relSZERO(Vector);
static void rgeSNEG(Vector);
static void rgeSINF(Vector);
static void rleSPOS(Vector);
static void rleSNEG(Vector);
static void rleSINF(Vector);
static void rggeSZERO(Vector);
static void rggeSINF(Vector);
static void rgggSINF(Vector);
static void rgglSZERO(Vector);
static void rll(Vector);
static void rlleSZERO(Vector);
static void rlleSINF(Vector);
static void rllgSZERO(Vector);
static void rlllSINF(Vector);
static void rgl(Vector);
static void rgleSZERO(Vector);
static void rgleSINF(Vector);
static void rglgSZERO(Vector);
static void rgllSINF(Vector);
static void rlg(Vector);
static void rlgeSZERO(Vector);
static void rlgeSINF(Vector);
static void rlggSINF(Vector);
static void rlglSZERO(Vector);

static TblEntry tee4 = {SNEG, EQ, rNoOp, NULL};
static TblEntry tee3 = {SPOS, EQ, rNoOp, &tee4};
static TblEntry tee2 = {SINF, EQ, rNoOp, &tee3};
static TblEntry tee1 = {SZERO, EQ, rNoOp, &tee2};

static TblEntry teg3 = {SNEG, GT, regSNEG, NULL};
static TblEntry teg2 = {SPOS, GT, regSPOS, &teg3};
static TblEntry teg1 = {SZERO, GT, regSZERO, &teg2};

static TblEntry tel3 = {SNEG, LT, relSNEG, NULL};
static TblEntry tel2 = {SPOS, LT, relSPOS, &tel3};
static TblEntry tel1 = {SZERO, LT, relSZERO, &tel2};

static TblEntry tge3 = {SNEG, LT, rgeSNEG, NULL};
static TblEntry tge2 = {SPOS, GT, rgeSPOS, &tge3};
static TblEntry tge1 = {SINF, GT, rgeSINF, &tge2};

static TblEntry tle3 = {SNEG, GT, rleSNEG, NULL};
static TblEntry tle2 = {SPOS, LT, rleSPOS, &tle3};
static TblEntry tle1 = {SINF, LT, rleSINF, &tle2};

static TblEntry tgge3 = {SPOS, GT, rgg, NULL};
static TblEntry tgge2 = {SINF, GT, rggeSINF, &tgge3};
static TblEntry tgge1 = {SZERO, GT, rggeSZERO, &tgge2};

static TblEntry tggg2 = {SPOS, GT, rgg, NULL};
static TblEntry tggg1 = {SINF, GT, rgggSINF, &tggg2};

static TblEntry tggl2 = {SPOS, GT, rgg, NULL};
static TblEntry tggl1 = {SZERO, GT, rgglSZERO, &tggl2};

static TblEntry tlle3 = {SPOS, LT, rll, NULL};
static TblEntry tlle2 = {SINF, LT, rlleSINF, &tlle3};
static TblEntry tlle1 = {SZERO, LT, rlleSZERO, &tlle2};

static TblEntry tllg2 = {SPOS, LT, rll, NULL};
static TblEntry tllg1 = {SZERO, LT, rllgSZERO, &tllg2};

static TblEntry tlll2 = {SPOS, LT, rll, NULL};
static TblEntry tlll1 = {SINF, LT, rlllSINF, &tlll2};

static TblEntry tgle3 = {SNEG, LT, rgl, NULL};
static TblEntry tgle2 = {SINF, GT, rgleSINF, &tgle3};
static TblEntry tgle1 = {SZERO, LT, rgleSZERO, &tgle2};

static TblEntry tglg2 = {SNEG, LT, rgl, NULL};
static TblEntry tglg1 = {SZERO, LT, rglgSZERO, &tglg2};

static TblEntry tgll2 = {SNEG, LT, rgl, NULL};
static TblEntry tgll1 = {SINF, GT, rgllSINF, &tgll2};

static TblEntry tlge3 = {SNEG, GT, rlg, NULL};
static TblEntry tlge2 = {SINF, LT, rlgeSINF, &tlge3};
static TblEntry tlge1 = {SZERO, GT, rlgeSZERO, &tlge2};

static TblEntry tlgg2 = {SNEG, GT, rlg, NULL};
static TblEntry tlgg1 = {SINF, LT, rlggSINF, &tlgg2};

static TblEntry tlgl2 = {SNEG, GT, rlg, NULL};
static TblEntry tlgl1 = {SZERO, GT, rlglSZERO, &tlgl2};

void
initEmitSign(void)
{
	mpz_init(tmp_z);
}

bool
emitSignFromVector(Vector v, Sign *s)
{
	TblEntry *e;

	if ((e = setList(v)) != NULL) {
		(e->residual)(v);
		*s = e->sign;
		if (TRACE && !validVector(v)) {
			printf("emitSignFromVector: bad vector, sign=%d\n", *s);
			dumpVector(v);
			Error(FATAL, E_INT, "emitSignFromVector", "bad vector");
		}
		return TRUE;
	} else
		return FALSE;
}

bool
emitSignFromMatrix(Matrix m, Sign *s)
{
	TblEntry *e0, *e1;
	Comparison comp;	/* not used */

	e0 = setList(m[0]);
	e1 = setList(m[1]);

	if (mergeTwoLists(&e0, &e1, &comp)) {
		(e0->residual)(m[0]);
		(e1->residual)(m[1]);
		*s = e0->sign;
		if (TRACE && !validMatrix(m)) {
			printf("emitSignFromMatrix: bad matrix, sign=%d\n", *s);
			dumpMatrix(m);
			Error(FATAL, E_INT, "emitSignFromMatrix", "bad matrix");
		}
		return TRUE;
	} else
		return FALSE;
}

bool
emitSignFromTensor(Tensor t, Sign *s)
{
	TblEntry *e0, *e1, *e2, *e3;

	e0 = setList(t[0]);
	e1 = setList(t[1]);
	e2 = setList(t[2]);
	e3 = setList(t[3]);

	if (mergeFourLists(&e0, &e1, &e2, &e3)) {
		(e0->residual)(t[0]);
		(e1->residual)(t[1]);
		(e2->residual)(t[2]);
		(e3->residual)(t[3]);
		*s = e0->sign;
		if (TRACE && !validTensor(t)) {
			printf("emitSignFromTensor: bad tensor, sign=%d\n", *s);
			dumpTensor(t);
			Error(FATAL, E_INT, "emitSignFromTensor", "bad tensor");
		}
		return TRUE;
	} else
		return FALSE;
}

static bool
mergeFourLists(TblEntry **e0, TblEntry **e1, TblEntry **e2, TblEntry **e3)
{
	bool flag;
	Comparison compLeft, compRight;

	flag = mergeTwoLists(e0, e1, &compLeft)
				&& mergeTwoLists(e2, e3, &compRight);

	while (flag) {
		if ((*e0)->sign == (*e2)->sign) {
			if (compLeft == compRight || compLeft == EQ || compRight == EQ)
				return TRUE;
			else {
				*e0 = (*e0)->next;
				*e1 = (*e1)->next;
				*e2 = (*e2)->next;
				*e3 = (*e3)->next;
				flag = mergeTwoLists(e0, e1, &compLeft)
						&& mergeTwoLists(e2, e3, &compRight);
			}
		}
		else {
			if ((*e0)->sign < (*e2)->sign) {
				*e0 = (*e0)->next;
				*e1 = (*e1)->next;
				flag = mergeTwoLists(e0, e1, &compLeft);
			}
			else {
				*e2 = (*e2)->next;
				*e3 = (*e3)->next;
				flag = mergeTwoLists(e2, e3, &compRight);
			}
		}
	}
	return FALSE;
}

static bool
mergeTwoLists(TblEntry **e0, TblEntry **e1, Comparison *comp)
{
	while (*e0 != NULL && *e1 != NULL) {
		if ((*e0)->sign == (*e1)->sign) {
			if ((*e0)->compare == (*e1)->compare) {
				*comp = (*e0)->compare;
				return TRUE;
			}
			else {
				if ((*e0)->compare == EQ) {
					*comp = (*e1)->compare;	/* must be GT or LT */
					return TRUE;
				}
				else {
					if ((*e1)->compare == EQ) {
						*comp = (*e0)->compare;  /* must be GT or LT */
						return TRUE;
					}
					else { /* one is LT and the other GT */
						*e0 = (*e0)->next;
						*e1 = (*e1)->next;
					}
				}
			}
		}
		else {
			if ((*e0)->sign < (*e1)->sign)
				*e0 = (*e0)->next;
			else
				*e1 = (*e1)->next;
		}
	}
	return FALSE;
}

/*
 * Given a vector, this function returns a pointer to the table entry (a list)
 * which classifies the vector in terms of what signs can be emitted for
 * that vector.
 */
static TblEntry *
setList(Vector v)
{
	int sign_a, sign_b;
	int tmp;

	sign_a = mpz_sgn(v[0]);
	sign_b = mpz_sgn(v[1]);
	switch (sign_a) {
		case 0 :
			switch (sign_b) {
				case 0 :   /* a == 0, b == 0 */
					return &tee1;
				case 1 :   /* a == 0, b > 0 */
					return &teg1;
				case -1 :   /* a == 0, b < 0 */
					return &tel1;
				default :
					Error(FATAL,E_INT,"signFromVector",
                                           "bad value returned by mpz_sgn");
			}

		case 1 :
			switch (sign_b) {
				case 0 :   /* a > 0, b == 0 */
					return &tge1;

				case 1 :   /* a > 0, b > 0 */
					tmp = mpz_cmp(v[0], v[1]);
					switch (MPZ_SIGN(tmp)) {
						case 0 : /* a > 0, b > 0, a == b */
							return &tgge1;
						case 1 : /* a > b, b > 0, a > b */
							return &tggg1;
						case -1 : /* a > b, b > 0, a < b */
							return &tggl1;
					}

				case -1 :  /* a > 0, b < 0 */
					/* it is faster to negate a twice */
					mpz_neg(v[0], v[0]);
					tmp = mpz_cmp(v[0], v[1]);
					mpz_neg(v[0], v[0]);
					switch (MPZ_SIGN(tmp)) {
						case 0 : /* a > 0, b < 0, -a == b */
							return &tgle1;
						case 1 : /* a > b, b < 0, -a > b */
							return &tglg1;
						case -1 : /* a > b, b < 0, -a < b */
							return &tgll1;
					}

				default :
					Error(FATAL,E_INT,"signFromVector",
                                           "bad value returned by mpz_sgn");
			}

		case -1 :
			switch (sign_b) {
				case 0 :   /* a < 0, b == 0 */
					return &tle1;
				case 1 :   /* a < 0, b > 0 */
					/* it is faster to negate a twice */
					mpz_neg(v[0], v[0]);
					tmp = mpz_cmp(v[0], v[1]);
					mpz_neg(v[0], v[0]);
					switch (MPZ_SIGN(tmp)) {
						case 0 : /* a < 0, b > 0, -a == b */
							return &tlge1;
						case 1 : /* a < b, b > 0, -a > b */
							return &tlgg1;
						case -1 : /* a < b, b > 0, -a < b */
							return &tlgl1;
					}

				case -1 :   /* a < 0, b < 0 */
					tmp = mpz_cmp(v[0], v[1]);
					switch (MPZ_SIGN(tmp)) {
						case 0 : /* a < 0, b < 0, a == b */
							return &tlle1;
						case 1 : /* a < b, b < 0, a > b */
							return &tllg1;
						case -1 : /* a < b, b < 0, a < b */
							return &tlll1;
					}

				default :
					Error(FATAL,E_INT,"signFromVector",
                                           "bad value returned by mpz_sgn");
			}
			break;

		default :
			Error(FATAL,E_INT,"signFromVector","bad value returned by mpz_sgn");
	}
}

static void
rNoOp(Vector v)
{
}

static void
regSNEG(Vector v)
{
	MPZ_SWAP(v[0], v[1]);
}

static void
regSZERO(Vector v)
{
	mpz_set(v[0], v[1]);
}

static void
relSPOS(Vector v)
{
	mpz_neg(v[1], v[1]);
}

static void
relSNEG(Vector v)
{
	mpz_neg(v[1], v[1]);
	MPZ_SWAP(v[0], v[1]);
}

static void
relSZERO(Vector v)
{
	mpz_neg(v[1], v[1]);
	mpz_set(v[0], v[1]);
}

static void
rgeSNEG(Vector v)
{
	MPZ_SWAP(v[0], v[1]);
}

static void
rgeSINF(Vector v)
{
	mpz_set(v[1], v[0]);
}

static void
rleSPOS(Vector v)
{
	mpz_neg(v[0], v[0]);
}

static void
rleSNEG(Vector v)
{
	mpz_neg(v[0], v[0]);
	MPZ_SWAP(v[0], v[1]);
}

static void
rleSINF(Vector v)
{
	mpz_neg(v[0], v[0]);
	mpz_set(v[1], v[0]);
}

static void
rggeSZERO(Vector v)
{
	mpz_mul_2exp(v[0], v[0], (unsigned long) 1);
	mpz_set_ui(v[1], (unsigned long) 0);
}

static void
rggeSINF(Vector v)
{
	mpz_mul_2exp(v[1], v[0], (unsigned long) 1);
	mpz_set_ui(v[0], (unsigned long) 0);
}

static void
rgggSINF(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_sub(v[0], v[0], v[1]);
	mpz_add(v[1], tmp_z, v[1]);
}

static void
rgglSZERO(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_add(v[0], v[0], v[1]);
	mpz_sub(v[1], v[1], tmp_z);
}

static void
rll(Vector v)
{
	mpz_neg(v[0], v[0]);
	mpz_neg(v[1], v[1]);
}

static void
rlleSZERO(Vector v)
{
	mpz_neg(v[0], v[0]);
	mpz_mul_2exp(v[0], v[0], (unsigned long) 1);
	mpz_set_ui(v[1], (unsigned long) 0);
}

static void
rlleSINF(Vector v)
{
	mpz_neg(v[0], v[0]);
	mpz_mul_2exp(v[1], v[0], (unsigned long) 1);
	mpz_set_ui(v[0], (unsigned long) 0);
}

static void
rllgSZERO(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_add(v[0], v[0], v[1]);
	mpz_neg(v[0], v[0]);
	mpz_sub(v[1], tmp_z, v[1]);
}

static void
rlllSINF(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_sub(v[0], v[1], v[0]);
	mpz_add(v[1], tmp_z, v[1]);
	mpz_neg(v[1], v[1]);
}

static void
rgl(Vector v)
{
	mpz_neg(v[1], v[1]);
	MPZ_SWAP(v[0], v[1]);
}

static void
rgleSZERO(Vector v)
{
	mpz_mul_2exp(v[1], v[0], (unsigned long) 1);
	mpz_set_ui(v[0], (unsigned long) 0);
}

static void
rgleSINF(Vector v)
{
	mpz_mul_2exp(v[0], v[0], (unsigned long) 1);
	mpz_set_ui(v[1], (unsigned long) 0);
}

static void
rglgSZERO(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_add(v[0], v[0], v[1]);
	mpz_neg(v[0], v[0]);
	mpz_sub(v[1], tmp_z, v[1]);  /* - (b - a) */
}

static void
rgllSINF(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_sub(v[0], v[0], v[1]);
	mpz_add(v[1], tmp_z, v[1]); 
}

static void
rlg(Vector v)
{
	mpz_neg(v[0], v[0]);
	MPZ_SWAP(v[0], v[1]);
}

static void
rlgeSZERO(Vector v)
{
	mpz_mul_2exp(v[1], v[1], (unsigned long) 1);
	mpz_set_ui(v[0], (unsigned long) 0);
}

static void
rlgeSINF(Vector v)
{
	mpz_mul_2exp(v[0], v[1], (unsigned long) 1);
	mpz_set_ui(v[1], (unsigned long) 0);
}

static void
rlggSINF(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_sub(v[0], v[1], v[0]);
	mpz_add(v[1], tmp_z, v[1]);
	mpz_neg(v[1], v[1]);
}

static void
rlglSZERO(Vector v)
{
	mpz_set(tmp_z, v[0]);
	mpz_add(v[0], v[0], v[1]);
	mpz_sub(v[1], v[1], tmp_z);
}
