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
 * Utilities used internally by the library
 */
/*
 * This is used only when compiled with -DTRACE=traceOn.
 */
int traceOn = 0;

/*
 * Some temporary big numbers which are shared
 */
mpz_t tmpa_z, tmpb_z, tmpc_z, tmpd_z, tmpe_z, tmpf_z;
mpz_t zero_z;

Matrix bigTmpMat;
Tensor bigTmpTen;

void
debugTrace(int b)
{
	traceOn = b;
}

/*
 * The library shares bignum temporart storage. This needs to change
 * when garbage collection goes back in.
 */
void
initTmp()
{
	mpz_init(tmpa_z);
	mpz_init(tmpb_z);
	mpz_init(tmpc_z);
	mpz_init(tmpd_z);
	mpz_init(tmpe_z);
	mpz_init(tmpf_z);

	mpz_init_set_ui(zero_z, 0);

	mpz_init(bigTmpMat[0][0]);
	mpz_init(bigTmpMat[0][1]);
	mpz_init(bigTmpMat[1][0]);
	mpz_init(bigTmpMat[1][1]);

	mpz_init(bigTmpTen[0][0]);
	mpz_init(bigTmpTen[0][1]);
	mpz_init(bigTmpTen[1][0]);
	mpz_init(bigTmpTen[1][1]);
	mpz_init(bigTmpTen[2][0]);
	mpz_init(bigTmpTen[2][1]);
	mpz_init(bigTmpTen[3][0]);
	mpz_init(bigTmpTen[3][1]);
}

void
multVectorPairTimesVector(Vector vec0, Vector vec1, Vector vec)
{
	/* ae + cf */
	mpz_mul(tmpa_z, vec0[0], vec[0]);
	mpz_mul(tmpe_z, vec1[0], vec[1]);
	mpz_add(tmpa_z, tmpa_z, tmpe_z);

	/* be + df */
	mpz_mul(tmpb_z, vec0[1], vec[0]);
	mpz_mul(tmpe_z, vec1[1], vec[1]);
	mpz_add(tmpb_z, tmpb_z, tmpe_z);

	/*
	 * Computed the product, now replace with the original matrix
	 * with a vector.
	 */
	MPZ_SWAP(tmpa_z, vec0[0]);
	MPZ_SWAP(tmpb_z, vec0[1]);

	mpz_set_ui(vec1[0], 0);
	mpz_set_ui(vec1[1], 0);
}

/*
 * Given vectors (a,b) and (c,d) and the matrix is ((e,f), (g,h)). This
 * computes the product as if the first two vectors are the columns of a
 * matrix. The result is a pair of vectors:
 *
 *    (ae + cf, be + df) and (ag + ch, bg + dh)
 *
 * It is important to note that the result overwrites the original vectors
 * using the SWAP macro.
 */
void
multVectorPairTimesMatrix(Vector vec0, Vector vec1, Matrix mat)
{
	/* ae + cf */
	mpz_mul(tmpa_z, vec0[0], mat[0][0]);
	mpz_mul(tmpe_z, vec1[0], mat[0][1]);
	mpz_add(tmpa_z, tmpa_z, tmpe_z);

	/* be + df */
	mpz_mul(tmpb_z, vec0[1], mat[0][0]);
	mpz_mul(tmpe_z, vec1[1], mat[0][1]);
	mpz_add(tmpb_z, tmpb_z, tmpe_z);

	/* ag + ch */
	mpz_mul(tmpc_z, vec0[0], mat[1][0]);
	mpz_mul(tmpe_z, vec1[0], mat[1][1]);
	mpz_add(tmpc_z, tmpc_z, tmpe_z);

	/* bg + dh */
	mpz_mul(tmpd_z, vec0[1], mat[1][0]);
	mpz_mul(tmpe_z, vec1[1], mat[1][1]);
	mpz_add(tmpd_z, tmpd_z, tmpe_z);

	/*
	 * Computed the product, now replace with the original vectors
	 */
	MPZ_SWAP(tmpa_z, vec0[0]);
	MPZ_SWAP(tmpb_z, vec0[1]);
	MPZ_SWAP(tmpc_z, vec1[0]);
	MPZ_SWAP(tmpd_z, vec1[1]);
}

/*
 * Same as the above except the columns of the matrix are (e,f) and (g,h)
 */
void
multVectorPairTimesMatrix_Z(Vector vec0, Vector vec1,
    mpz_t e, mpz_t f, mpz_t g, mpz_t h)
{
	/* ae + cf */
	mpz_mul(tmpa_z, vec0[0], e);
	mpz_mul(tmpe_z, vec1[0], f);
	mpz_add(tmpa_z, tmpa_z, tmpe_z);

	/* be + df */
	mpz_mul(tmpb_z, vec0[1], e);
	mpz_mul(tmpe_z, vec1[1], f);
	mpz_add(tmpb_z, tmpb_z, tmpe_z);

	/* ag + ch */
	mpz_mul(tmpc_z, vec0[0], g);
	mpz_mul(tmpe_z, vec1[0], h);
	mpz_add(tmpc_z, tmpc_z, tmpe_z);

	/* bg + dh */
	mpz_mul(tmpd_z, vec0[1], g);
	mpz_mul(tmpe_z, vec1[1], h);
	mpz_add(tmpd_z, tmpd_z, tmpe_z);

	/*
	 * Computed the product, now replace with the original vectors
	 */
	MPZ_SWAP(tmpa_z, vec0[0]);
	MPZ_SWAP(tmpb_z, vec0[1]);
	MPZ_SWAP(tmpc_z, vec1[0]);
	MPZ_SWAP(tmpd_z, vec1[1]);
}

/*
 * Not needed for GMP 3.1
inline void
mpz_mul_si(mpz_t x, mpz_t y, int z)
{
	if (z >= 0)
		mpz_mul_ui(x, y, (unsigned) z);
	else {
		mpz_mul_ui(x, y, (unsigned) (-z));
		mpz_neg(x, x);
	}
}
*/

/*
 * This is the same as above except the entries in the matrix all
 * fit in a machine word.
 */
void
multVectorPairTimesSmallMatrix(Vector vec0, Vector vec1, SmallMatrix mat)
{
	/* ae + cf */
	mpz_mul_si(tmpa_z, vec0[0], mat[0][0]);
	mpz_mul_si(tmpe_z, vec1[0], mat[0][1]);
	mpz_add(tmpa_z, tmpa_z, tmpe_z);

	/* be + df */
	mpz_mul_si(tmpb_z, vec0[1], mat[0][0]);
	mpz_mul_si(tmpe_z, vec1[1], mat[0][1]);
	mpz_add(tmpb_z, tmpb_z, tmpe_z);

	/* ag + ch */
	mpz_mul_si(tmpc_z, vec0[0], mat[1][0]);
	mpz_mul_si(tmpe_z, vec1[0], mat[1][1]);
	mpz_add(tmpc_z, tmpc_z, tmpe_z);

	/* bg + dh */
	mpz_mul_si(tmpd_z, vec0[1], mat[1][0]);
	mpz_mul_si(tmpe_z, vec1[1], mat[1][1]);
	mpz_add(tmpd_z, tmpd_z, tmpe_z);

	/*
	 * Computed the product, now replace with the original vectors
	 */
	MPZ_SWAP(tmpa_z, vec0[0]);
	MPZ_SWAP(tmpb_z, vec0[1]);
	MPZ_SWAP(tmpc_z, vec1[0]);
	MPZ_SWAP(tmpd_z, vec1[1]);
}

/*
 * This does the work of taking some number of digits from something bearing
 * information such as an LFT. There are however, possibilities other than
 * LFTs. See realSqrt for another example of the use of this function.
 * The digits are deposited in a digsX structure augmenting any digits
 * already in the structure.
 *
 * The arguments are as follows:
 *
 *   digsX - the place where the digits are to be deposited
 *
 *   emitDigit - this is a pointer to a function which is used to
 *     obtain a single digit from the object and compute the residual of
 *     that object once than digits is emitted.
 *
 *   info - this is the object which holds informations such as a vector,
 *     matrix or tensor and from which we are taking digits. This function 
 *     does not inspect ``info'' directly. It doesn't care what ``info'' is
 *     and only passes it to emitDigit
 *
 *   digitsNeeded - this is what is says - the number of digits we should try
 *     to emit. Often this is chosen to be MAXINT, or in other words take
 *     all the digits possible.
 *
 * The function returns the number of digits it has managed to emit (which may
 * be and is often less than we asked for).
 */
int
emitDigits(DigsX *digsX, edf emitDigit, void *info, int digitsNeeded)
{
	Digit d;
	int word;
	unsigned char count;
	int total;
	bool ok;

	total = 0;
	ok = TRUE;
	while (digitsNeeded > 0 && ok) {
		/*
		 * We now extract digits from info until we have filled
		 * a machine word or until no digit can be extracted.
		 */
		word = 0;
		count = 0;
		while (count < DIGITS_PER_WORD && digitsNeeded > 0 && ok) {
			ok = (*emitDigit)(info, &d);
			if (ok) {
				count++;
				digitsNeeded--;
				word = word * 2 + d;
			}
		}

		/*
		 * If we get here and count > 0, then we need to augment the 
		 * word stored in the DigsX.
		 */
		if (count > 0) {
			total += count;

			/*
			 * First check to see if we are already dealing with a large word.
			 */
#ifdef PACK_DIGITS
			if (digsX->count > DIGITS_PER_WORD) {
#endif
				mpz_mul_2exp(digsX->word.big, digsX->word.big, count);
				if (word >= 0) {	/* there is no mpz_add_si function */
					mpz_add_ui(digsX->word.big, digsX->word.big, word);
				}
				else {
					mpz_sub_ui(digsX->word.big, digsX->word.big, -word);
				}
				digsX->count += count;
#ifdef PACK_DIGITS
			}
			else {
				digsX->count += count;
	
				/*
				 * Now see if we are about to overflow the machine word.
				 */
				if (digsX->count > DIGITS_PER_WORD) {
					mpz_init_set_si(digsX->word.big, digsX->word.small);
					mpz_mul_2exp(digsX->word.big, digsX->word.big, count);
					if (word >= 0) {	/* there is no mpz_add_si function */
						mpz_add_ui(digsX->word.big, digsX->word.big, word);
					}
					else {
						mpz_sub_ui(digsX->word.big, digsX->word.big, -word);
					}
				}
				else
				digsX->word.small = digsX->word.small * (1 << count) + word;
			}
#endif
		}
	}
	return total;
}

/*
 * The assumption here is that the number of digits in digsX <= DIGITS_PER_WORD. */
void
makeSmallMatrixFromDigits(SmallMatrix mat, DigsX *digsX)
{
	int twoPowN;

	if (digsX->count <= 0) {
		mat[0][0] = 1;
		mat[0][1] = 0;
		mat[1][0] = 0;
		mat[1][1] = 1;
	}
	else {
		twoPowN = 1 << digsX->count;
		mat[0][0] = twoPowN + digsX->word.small + 1;
		mat[0][1] = twoPowN - digsX->word.small - 1;
		mat[1][0] = twoPowN + digsX->word.small - 1;
		mat[1][1] = twoPowN - digsX->word.small + 1;
	}
}

/*
 * Same as the function above except for big integers.
 * In this case we assume that the number of digits with which to form
 * the matrix is > DIGITS_PER_WORD so be warned.
 */
void
makeMatrixFromDigits(Matrix mat, DigsX *digsX)
{
	mpz_set_ui(tmpe_z, (unsigned long) 1);
	mpz_mul_2exp(tmpe_z, tmpe_z, (unsigned long) digsX->count);  /* 2^n */
	switch (mpz_sgn(digsX->word.big)) {
		case 0 :   /* not sure I should bother making this a special case */
			mpz_add_ui(mat[0][0], tmpe_z, (unsigned long) 1);
			mpz_sub_ui(mat[0][1], tmpe_z, (unsigned long) 1);
			mpz_sub_ui(mat[1][0], tmpe_z, (unsigned long) 1);
			mpz_add_ui(mat[1][1], tmpe_z, (unsigned long) 1);
			break;
		case 1 :
		case -1 :
			mpz_add(mat[0][0], tmpe_z, digsX->word.big);
			mpz_set(mat[1][0], mat[0][0]);
			mpz_sub(mat[0][1], tmpe_z, digsX->word.big);
			mpz_set(mat[1][1], mat[0][1]);
			mpz_add_ui(mat[0][0], mat[0][0], (unsigned long) 1);
			mpz_sub_ui(mat[0][1], mat[0][1], (unsigned long) 1);
			mpz_sub_ui(mat[1][0], mat[1][0], (unsigned long) 1);
			mpz_add_ui(mat[1][1], mat[1][1], (unsigned long) 1);
			break;
		default :
			Error(FATAL, E_INT, "makeMatrixFromDigits",
						"bad value from mpz_sign");
			break;
	}
}

/*
 * This makes a vector ``canonical'' by removing common factors from the
 * numerator and denominator.
 */
void
canonVector(Vector vec)
{
	mpz_gcd(tmpa_z, vec[0], vec[1]);
	if (mpz_sgn(tmpa_z) != 0) {
		mpz_divexact(vec[0], vec[0], tmpa_z);
		mpz_divexact(vec[1], vec[1], tmpa_z);
	}
}
#define MIN_AB(a,b) ((a)>0 ? ((a)>(b) ? (a) : (b)) : ((b)>0 ? (b) : MAXINT))
#define SIZE_GT_ZERO(size, w)	((size) > 0 ? (w) : 0)

/*
 * A vector, matrix or tensor is normalized when there are no negative
 * entries and there are no common factors. In practice, ensuring that
 * there are no negative entries is dealt with by the functions which 
 * emit signs and digits. Also, rather than looking for gcd's for all the
 * entries, we only look for exponents of 2. See Reinhold Heckman's
 * notes for a justification of this.
 */
static int
normalizeEntries(mpz_ptr *entries, int length)
{
	mp_bitcnt_t count, candidate;
	int i;

	count = (mp_bitcnt_t) -1;
	for (i = 0; i < length; i++) {
		if (mpz_sgn(entries[i]) != 0) {
			candidate = mpz_scan1(entries[i], 0);
			if (candidate < count)
				count = candidate;
		}
	}
	if (count == (mp_bitcnt_t) -1)
		return 0;
	if (count > (mp_bitcnt_t) MAXINT)
		Error(FATAL, E_INT, "normalizeEntries", "power of two too large");
	if (count > 0) {
		for (i = 0; i < length; i++)
			if (mpz_sgn(entries[i]) != 0)
				mpz_tdiv_q_2exp(entries[i], entries[i], count);
	}
	return (int) count;
}

int
normalizeVector(Vector vec)
{
	mpz_ptr entries[2];
	entries[0] = vec[0];
	entries[1] = vec[1];
	return normalizeEntries(entries, 2);
}

int
normalizeMatrix(Matrix mat)
{
	mpz_ptr entries[4];
	entries[0] = mat[0][0];
	entries[1] = mat[0][1];
	entries[2] = mat[1][0];
	entries[3] = mat[1][1];
	return normalizeEntries(entries, 4);
}

int
normalizeTensor(Tensor ten)
{
	mpz_ptr entries[8];
	entries[0] = ten[0][0];
	entries[1] = ten[0][1];
	entries[2] = ten[1][0];
	entries[3] = ten[1][1];
	entries[4] = ten[2][0];
	entries[5] = ten[2][1];
	entries[6] = ten[3][0];
	entries[7] = ten[3][1];
	return normalizeEntries(entries, 8);
}

int
vectorSign(Vector v)
{
	int sum;

	sum = mpz_sgn(v[0]) + mpz_sgn(v[1]);
	if (sum > 0)
		return 1;
	else
		if (sum < 0)
			return -1;
		else
			return 0;
}

/*
 * This function returns 1 if there are no 0 columns and all entries
 * are >= 0, -1 if there are no 0 columns and all entries are <= 0 and
 * 0 otherwise (ie when the signs are mixed or there are 0 columns).
 */
int
matrixSign(Matrix m)
{
	int vecSign;

	if (((vecSign = vectorSign(m[0])) != 0) && (vectorSign(m[1]) == vecSign))
		return vecSign;
	else
		return 0;
}

/*
 * This function returns 1 if there are no 0 columns and all entries
 * are >= 0, -1 if there are no 0 columns and all entries are <= 0 and
 * 0 otherwise (ie when the signs are mixed and/or there are 0 columns).
 */
int
tensorSign(Tensor t)
{
	int vecSign;

	if (((vecSign = vectorSign(t[0])) != 0)
			&& (vectorSign(t[1]) == vecSign)
			&& (vectorSign(t[2]) == vecSign)
			&& (vectorSign(t[3]) == vecSign))
		return vecSign;
	else
		return 0;
}

/*
 * A vector is positive when at least one value is not 0 and both
 * are greater than or equal to zero
 */
bool
vectorIsPositive(Vector v)
{
	return vectorSign(v) == 1;
}

/*
 * A matrix is ``positive'' when it contains no zero columns and
 * all the entries are >= 0
 */
bool
matrixIsPositive(Matrix m)
{
	return matrixSign(m) == 1;
}

/*
 * A matrix is ``positive'' when it contains no zero columns and
 * all the entries are >= 0
 */
bool
tensorIsPositive(Tensor t)
{
	return tensorSign(t) == 1;
}

void
negateMatrix(Matrix m)
{
	mpz_neg(m[0][0], m[0][0]);
	mpz_neg(m[0][1], m[0][1]);
	mpz_neg(m[1][0], m[1][0]);
	mpz_neg(m[1][1], m[1][1]);
}

void
negateTensor(Tensor t)
{
	mpz_neg(t[0][0], t[0][0]);
	mpz_neg(t[0][1], t[0][1]);
	mpz_neg(t[1][0], t[1][0]);
	mpz_neg(t[1][1], t[1][1]);
	mpz_neg(t[2][0], t[2][0]);
	mpz_neg(t[2][1], t[2][1]);
	mpz_neg(t[3][0], t[3][0]);
	mpz_neg(t[3][1], t[3][1]);
}

/*
 * A tensor is refining when it has no zero columns
 * (where both entries 0) and all non-zero entries have the same sign.
 */
bool
tensorIsRefining(Tensor t)
{
	return (tensorSign(t) != 0);
}

void
absorbSignIntoVectorPair(Vector vec0, Vector vec1, Sign sign)
{
	switch (sign) {
	case SPOS :
		break;
	case SNEG :			/* ((c, d), (-a, -b)) */
		mpz_neg(vec0[0], vec0[0]);
		mpz_neg(vec0[1], vec0[1]);
		MPZ_SWAP(vec0[0], vec1[0]);
		MPZ_SWAP(vec0[1], vec1[1]);
		break;
	case SINF :			/* ((a-c,b-d), (a+c,b+d)) */
		mpz_set(tmpa_z, vec0[0]);				/* tmp = a */
		mpz_sub(vec0[0], tmpa_z, vec1[0]);	/* a = tmp - c */
		mpz_add(vec1[0], tmpa_z, vec1[0]);	/* c = tmp + c */
		mpz_set(tmpa_z, vec0[1]);				/* tmp = b */
		mpz_sub(vec0[1], tmpa_z, vec1[1]);	/* b = tmp - d */
		mpz_add(vec1[1], tmpa_z, vec1[1]);	/* d = tmp + d */
		break;
	case SZERO :		/* ((a+c,b+d), (c-a,d-b)) */
		mpz_set(tmpa_z, vec0[0]);				/* tmp = a */
		mpz_add(vec0[0], tmpa_z, vec1[0]);	/* a = tmp + c */
		mpz_sub(vec1[0], vec1[0], tmpa_z);	/* c = c - tmp */
		mpz_set(tmpa_z, vec0[1]);				/* tmp = b */
		mpz_add(vec0[1], tmpa_z, vec1[1]);	/* b = tmp + d */
		mpz_sub(vec1[1], vec1[1], tmpa_z);	/* d = d - tmp */
		break;
	default :
		Error(FATAL, E_INT, "absorbSignIntoVectorPair", "bad sign");
	}
}

Real
derefToStrm(Real x)
{
	if (x != NULL) {
		switch (x->gen.tag.type) {
		case DIGSX :
		case SIGNX :
			break;
		case VECTOR :
			return derefToStrm(x->vec.strm);
		case MATX :
			return derefToStrm(x->matX.strm);
		case TENXY :
			return derefToStrm(x->tenXY.strm);
		case ALT :
			return derefToStrm(x->alt.redirect);
		case CLOSURE :
			return derefToStrm(x->cls.redirect);
		default :
			Error(FATAL, E_INT, "derefToStrm", "invalid real");
		}
	}
	return x;
}

char *
comparisonToString(Comparison d)
{
	switch (d) {
	case LT :
		return "lt";
		break;
	case GT :
		return "gt";
		break;
	case EQ :
		return "eq";
		break;
	default :
		break;
	}
}

char *
signToString(Sign s)
{
	switch (s) {
	case SIGN_UNKN :
		return "unkn";
	case SPOS :
		return "spos";
		break;
	case SNEG :
		return "sneg";
		break;
	case SZERO :
		return "szer";
		break;
	case SINF :
		return "sinf";
		break;
	default :
		break;
	}
}

char *
digitToString(Digit d)
{
	switch (d) {
	case DPOS :
		return "dpos";
		break;
	case DNEG :
		return "dneg";
		break;
	case DZERO :
		return "dzer";
		break;
	default :
		break;
	}
}

char *
typeToString(unsigned type)
{
	switch (type) {
		case ALT :
			return "alt   ";
		case VECTOR :
			return "vector";
		case MATX :
			return "matrix";
		case TENXY :
			return "tensor";
		case SIGNX :
			return "sign  ";
		case DIGSX :
			return "digits";
		case CLOSURE :
			return "closure";
		case BOOLX :
			return "boolx";
		case BOOLXY :
			return "boolxy";
		case PREDX :
			return "predx";
		default :
			Error(FATAL, E_INT, "typeToString", "bad type: %d", type);
			break;
	}
}

char *
boolValToString(unsigned boolVal)
{
	switch (boolVal) {
	case LAZY_TRUE :
		return "true ";
	case LAZY_FALSE :
		return "false";
	case LAZY_UNKNOWN :
		return "unkn ";
	default :
		Error(FATAL, E_INT, "boolValToString", "bad boolean value");
	}
}
