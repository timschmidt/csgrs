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
 * This is the algorithm for emitting digits from vectors, matrices and
 * tensors. The naive algorithm is to try each digit in turn. This means
 * given an LFT, L, then for each digit D, we compute inv(D) * L and
 * see if the result is refining (all entries positive). This is hopelessly
 * inefficient. A better way is to assume that L is initially refining
 * and then see for which digits the property is preserved. 
 *
 * The objective with the emission algorithm is to decide which digit(s) if
 * any can be emitted and to compute the residual LFT.
 *
 * The algorithm works with vectors. Given a vector we determine in which
 * digit interval it falls: D_{1} = [0,1], D_0 = [1/3,3] and D_{+1} =
 * [1,infty]. For matrices and tensors, this can be done column by column
 * and then merge sort the list of digits. See my ``Notes on sign and
 * digit emission'' from April 98 for further details.

 * The algorithm makes extensive use of continuations. This way we
 * have postpone the expensive comparisons and residual calculations
 * until needed.

 * Strictly speaking we don't need a functional continuation. There are
 * only a few options so we could do with just an integer to hold the
 * current state. But continuations are nicer.
 */

/*
 * For the purposes of this file, digits are assigned numbers so as to
 * be ordered in such a way that a merge of digit streams prefers
 * Dpos and Dneg over Dzero
 */

typedef enum {_DPOS = 0, _DNEG = 1, _DZERO = 2} _Digit;

#define ConvertDigit(x) (((x)==_DPOS) ? DPOS : (((x)==_DNEG) ? DNEG : DZERO))

typedef struct VS {
	mpz_t *v;              /* the original vector */
	struct TE *e;
	mpz_t three_a;			/* 3 * a */
	mpz_t three_b;			/* 3 * b */
} VecState;

typedef struct TE {
	_Digit digit;
	void (*residual)(struct VS *); /* the function to compute the residual */
	bool (*next)(struct VS *);     /* computes next digit in the sequence */
} TblEntry;

typedef struct {
	VecState *vs0;
	VecState *vs1;
} MatState;

static bool digitFromVector(VecState *); 
static bool digitFromMatrix(MatState *);
static bool digitFromTensor(); 

static VecState vs0, vs1, vs2, vs3;

static MatState ms0 = {&vs0, &vs1};
static MatState ms1 = {&vs2, &vs3};

static mpz_t tmp_z;

static TblEntry tStart = {99, NULL, digitFromVector};

void
initEmitDigit(void)
{
	mpz_init(vs0.three_a);
	mpz_init(vs0.three_b);

	mpz_init(vs1.three_a);
	mpz_init(vs1.three_b);

	mpz_init(vs2.three_a);
	mpz_init(vs2.three_b);

	mpz_init(vs3.three_a);
	mpz_init(vs3.three_b);

	mpz_init(tmp_z);
}

bool
validVector(Vector vec)
{
	int c0 = mpz_sgn(vec[0]);
	int c1 = mpz_sgn(vec[1]);

	if (c0 >= 0 && c1 >= 0)
		return TRUE;
	else
		return FALSE;
}

bool
validMatrix(Matrix mat)
{
	return validVector(mat[0]) && validVector(mat[1]);
}

bool
validTensor(Tensor ten)
{
	return validVector(ten[0]) && validVector(ten[1])
	    && validVector(ten[2]) && validVector(ten[3]);
}

/*
 * This function returns a bool to indicate if a digit has been
 * found. The digit itself is placed in the space indicated by the
 * second argument. This also includes the continuation for computing
 * the remaining digits.
 */
bool
emitDigitFromVector(Vector v, Digit *d)
{
	vs0.v = v;
	if (digitFromVector(&vs0)) {
		(*vs0.e->residual)(&vs0);
		*d = ConvertDigit(vs0.e->digit);
		if (TRACE && !validVector(v)) {
			printf("emitDigitFromVector: bad vector, digit=%d\n", *d);
			dumpVector(v);
			Error(FATAL, E_INT, "emitDigitFromVector", "bad vector");
		}
		return TRUE;
	}
	else
		return FALSE;
}

bool
emitDigitFromMatrix(Matrix m, Digit *d)
{
	if (TRACE && !validMatrix(m)) {
		dumpMatrix(m);
		Error(FATAL, E_INT, "emitDigitFromMatrix", "bad matrix (1)");
	}

	vs0.v = m[0];
	vs0.e = &tStart;
	vs1.v = m[1];
	vs1.e = &tStart;

	if (digitFromMatrix(&ms0)) {
		(*vs0.e->residual)(&vs0);
		(*vs1.e->residual)(&vs1);
		*d = ConvertDigit(vs0.e->digit);
		if (TRACE && !validMatrix(m)) {
			printf("emitDigitFromMatrix: bad matrix, digit=%d\n", *d);
			dumpMatrix(m);
			Error(FATAL, E_INT, "emitDigitFromMatrix", "bad matrix (2)");
		}
		return TRUE;
	} else
		return FALSE;
}

bool
emitDigitFromTensor(Tensor t, Digit *d)
{
	if (TRACE && !validTensor(t)) {
		dumpTensor(t);
		Error(FATAL, E_INT, "emitDigitFromTensor", "bad tensor (1)");
	}

	vs0.v = t[0];
	vs0.e = &tStart;
	vs1.v = t[1];
	vs1.e = &tStart;
	vs2.v = t[2];
	vs2.e = &tStart;
	vs3.v = t[3];
	vs3.e = &tStart;

	if (digitFromTensor(d)) {
		(*vs0.e->residual)(&vs0);
		(*vs1.e->residual)(&vs1);
		(*vs2.e->residual)(&vs2);
		(*vs3.e->residual)(&vs3);
		*d = ConvertDigit(vs0.e->digit);
		if (TRACE && !validTensor(t)) {
			printf("emitDigitFromTensor: bad tensor, digit=%d\n", *d);
			dumpTensor(t);
			Error(FATAL, E_INT, "emitDigitFromTensor", "bad tensor (2)");
		}
		return TRUE;
	} else
		return FALSE;
}

static bool
digitFromTensor()
{
	int flag;
	TblEntry *e1, *e2;

	flag = digitFromMatrix(&ms0) && digitFromMatrix(&ms1);
	while (flag) {
		e1 = ms0.vs0->e;
		e2 = ms1.vs0->e;
		if (e1->digit == e2->digit)
			return TRUE;
		else {
			if (e1->digit < e2->digit)
				flag = digitFromMatrix(&ms0);
			else
				flag = digitFromMatrix(&ms1);
		}
	}
	return FALSE;
}

static bool
digitFromMatrix(MatState *ms)
{
	int flag;
	TblEntry *e1, *e2;

	e1 = ms->vs0->e;
	e2 = ms->vs1->e;
	flag = (*e1->next)(ms->vs0) && (*e2->next)(ms->vs1);
	while (flag) {
		e1 = ms->vs0->e;
		e2 = ms->vs1->e;
		if (e1->digit == e2->digit)
			return TRUE;
		else {
			if (e1->digit < e2->digit)
				flag = (*e1->next)(ms->vs0);
			else
				flag = (*e2->next)(ms->vs1);
		}
	}
	return FALSE;
}

/*
 * Functions prefixed c are comparisons. The suffixes e, g, l refer
 * to equal, greater, or less than in the comparison between the two entries
 * in a vector.
 */
static bool cFalse(VecState *);
static bool cee1(VecState *);
static bool cee2(VecState *);
static bool cgge1(VecState *);
static bool cgge2(VecState *);
static bool cggl(VecState *);
static bool cggg(VecState *);

/*
 * Functions prefixed r compute residuals.
 */
static void rNoOp(VecState *);
static void rgge1(VecState *);
static void rgge2(VecState *);
static void rgge3(VecState *);

static void rggg(VecState *);
static void rggge(VecState *);
static void rgggl(VecState *);

static void rggl(VecState *);
static void rggle(VecState *);
static void rgglg(VecState *);

/*
 * This table drives the search for digits. It corresponds to the
 * table in ``Notes on sign and digit emission''.
 */
static TblEntry tee1 = {_DPOS, rNoOp, cee1};
static TblEntry tee2 = {_DNEG, rNoOp, cee2};
static TblEntry tee3 = {_DZERO, rNoOp, cFalse};

static TblEntry teg1 = {_DNEG, rNoOp, cFalse};
static TblEntry tge1 = {_DPOS, rNoOp, cFalse};

static TblEntry tgge1 = {_DPOS, rgge1, cgge1};
static TblEntry tgge2 = {_DNEG, rgge2, cgge2};
static TblEntry tgge3 = {_DZERO, rgge3, cFalse};

static TblEntry tggg = {_DPOS, rggg, cggg};
static TblEntry tggge = {_DZERO, rggge, cFalse};
static TblEntry tgggl = {_DZERO, rgggl, cFalse};

static TblEntry tggl = {_DNEG, rggl, cggl};
static TblEntry tggle = {_DZERO, rggle, cFalse};
static TblEntry tgglg = {_DZERO, rgglg, cFalse};

static bool
digitFromVector(VecState *vs)
{
	int sign_a, sign_b;
	int tmp;

	sign_a = mpz_sgn(vs->v[0]);
	sign_b = mpz_sgn(vs->v[1]);
	switch (sign_a) {
		case 0 :
			switch (sign_b) {
				case 0 :   /* a == 0, b == 0 */
					return FALSE;
					break;
				case 1 :   /* a == 0, b > 0 */
					vs->e = &teg1;
					return TRUE;
					break;
				case -1 :
					return FALSE;
					break;
				default :
					Error(FATAL,E_INT,"digitFromVector",
                                           "bad value returned by mpz_sgn");
					break;
			}
			break;
		case 1 :
			switch (sign_b) {
				case 0 :   /* a > 0, b == 0 */
					vs->e = &tge1;
					return TRUE;
					break;
				case 1 :
					tmp = mpz_cmp(vs->v[0],vs->v[1]);
					switch (MPZ_SIGN(tmp)) {
						case 0 : /* a > 0, b > 0, a == b */
							vs->e = &tgge1;
							return TRUE;
							break;
						case 1 : /* a > b, b > 0, a > b */
							vs->e = &tggg;
							return TRUE;
							break;
						case -1 : /* a > b, b > 0, a < b */
							vs->e = &tggl;
							return TRUE;
							break;
					}
					break;
				case -1 :
					return FALSE;
					break;
				default :
					Error(FATAL,E_INT,"digitFromVector",
                                           "bad value returned by mpz_sgn");
					break;
			}
			break;
		case -1 :
			return FALSE;
			break;
		default :
			Error(FATAL,E_INT,"digitFromVector","bad value returned by mpz_sgn");
			break;
	}
}

static bool
cFalse(VecState *vs)
{
	return FALSE;
}

static bool
cee1(VecState *vs)
{
	vs->e = &tee2;
	return TRUE;
}

static bool
cee2(VecState *vs)
{
	vs->e = &tee3;
	return TRUE;
}

static bool
cgge1(VecState *vs)
{
	vs->e = &tgge2;
	return TRUE;
}

static bool
cgge2(VecState *vs)
{
	vs->e = &tgge3;
	return TRUE;
}

static void
rNoOp(VecState *vs)
{
}

static void
rgge1(VecState *vs)
{
	mpz_set_ui(vs->v[0], (unsigned long) 0);
	mpz_mul_2exp(vs->v[1], vs->v[1], 1);
}

static void
rgge2(VecState *vs)
{
	mpz_set_ui(vs->v[1], (unsigned long) 0);
	mpz_mul_2exp(vs->v[0], vs->v[0], 1);
}

static void
rgge3(VecState *vs)
{
	mpz_mul_2exp(vs->v[0], vs->v[0], 1);
	mpz_mul_2exp(vs->v[1], vs->v[1], 1);
}

static bool
cggg(VecState *vs)
{
	int tmp;

	mpz_mul_ui(vs->three_b, vs->v[1], (unsigned long) 3);
	tmp = mpz_cmp(vs->v[0], vs->three_b);
	switch (MPZ_SIGN(tmp)) {
		case 0 :  /* a > 0, b > 0, a > b, a = 3b */
			vs->e = &tggge;
			return TRUE;
			break;
		case 1 :
			return FALSE;
			break;
		case -1 :
			vs->e = &tgggl;
			return TRUE;
			break;
	}
}

static void
rggg(VecState *vs)
{
	mpz_set(tmp_z, vs->v[1]);
	mpz_sub(vs->v[0], vs->v[0], vs->v[1]);
	mpz_mul_2exp(vs->v[1], tmp_z, 1);
}

static void
rgggl(VecState *vs)
{
	mpz_set(tmp_z, vs->v[0]);
	mpz_mul_ui(vs->three_a, vs->v[0], (unsigned long) 3);
	mpz_sub(vs->v[0], vs->three_a, vs->v[1]);
	mpz_sub(vs->v[1], vs->three_b, tmp_z);
}

static void
rggge(VecState *vs)
{
	mpz_mul_2exp(vs->v[0], vs->v[1], (unsigned long) 3);
	mpz_set_ui(vs->v[1], (unsigned long) 0);
}

static bool
cggl(VecState *vs)
{
	int tmp;

	mpz_mul_ui(vs->three_a, vs->v[0], (unsigned long) 3);
	tmp = mpz_cmp(vs->three_a, vs->v[1]);
	switch (MPZ_SIGN(tmp)) {
		case 0 :   /* a > 0, b > 0, a < b, 3a = b */
			vs->e = &tggle;
			return TRUE;
			break;
		case 1 :
			vs->e = &tgglg;
			return TRUE;
			break;
		case -1 :
			return FALSE;
			break;
	}
}

static void
rggl(VecState *vs)
{
	mpz_set(tmp_z, vs->v[0]);
	mpz_sub(vs->v[1], vs->v[1], vs->v[0]);
	mpz_mul_2exp(vs->v[0], tmp_z, 1);
}

static void
rggle(VecState *vs)
{
	mpz_mul_2exp(vs->v[1], vs->v[0], (unsigned long) 3);
	mpz_set_ui(vs->v[0], (unsigned long) 0);
}


static void
rgglg(VecState *vs)
{
	mpz_set(tmp_z, vs->v[1]);
	mpz_mul_ui(vs->three_b, vs->v[1], (unsigned long) 3);
	mpz_sub(vs->v[1], vs->three_b, vs->v[0]);
	mpz_sub(vs->v[0], vs->three_a, tmp_z);
}

#ifdef JUNK
(\Comp{a}{0}, \Comp{b}{0}, \Comp{a}{b}, \Comp{3a}{b}, \Comp{a}{3b})
(=,=,\square,\square,\square) & [(\Dpos, (0,0)),
\Dneg, (0,0)),
(\Dzero, (0,0))] \\
(=,>,\square,\square,\square) & [(\Dneg, (0,b))] \\
(>,=,\square,\square,\square) & [(\Dpos, (a,0))] \\
(>,>,=,\square,\square) & [(\Dpos, (0, 2b)), (\Dneg, (2a, 0)), (\Dzero, (2a, 2a)
)] \\
(>,>,>,<,\square) & [(\Dpos, (a-b, 2b)), (\Dzero, (3a-b, 3b-a))] \\
(>,>,>,>,\square) &  [(\Dpos, (a-b, 2b))] \\
(>,>,>,=,\square) & [(\Dpos, (2b,2b)), (\Dzero, (8b,0))]\\
(>,>,<,\square,<) & [(\Dneg, (2a, b-a))] \\
(>,>,<,\square,>) &  [(\Dneg, (2a, b-a)), (\Dzero, (3a-b, 3b-a))] \\
(>,>,<,\square,=) & [(\Dneg, (2a, 2a)), (\Dzero, (0, 8a))]
#endif
