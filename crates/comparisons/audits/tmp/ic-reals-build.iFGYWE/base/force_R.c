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
 * This is the toplevel function called by the user to force information
 * from a real stream.
 */
void
force_R_Digs(Real x, int digitsNeeded)
{
	DigsX *digsX;
	Real r;
	void runStack();
	void force_To_Alt_Entry();

	if (x->gen.tag.type == ALT) {
		if (x->alt.redirect == NULL) {
			PUSH_2(force_To_Alt_Entry, x);
			runStack();
		}
		force_R_Digs(x->alt.redirect, digitsNeeded);
		return;
	}

	if (x->gen.tag.type == CLOSURE) {
		if (x->cls.redirect == NULL) {
			PUSH_2(x->cls.force, x);
			runStack();
		}
		force_R_Digs(x->cls.redirect, digitsNeeded);
		return;
	}

	x = makeStream(x);

	/*
	 * So now we know that x is a real stream. If x is signed and
	 * the sign has not been determined, we force it.
	 */
	if (x->gen.tag.type == SIGNX) {
		if (x->signX.tag.value == SIGN_UNKN) {
			PUSH_2(x->signX.force, x);
			runStack();
		}
		digsX = (DigsX *) x->signX.x;
	}
	else
		digsX = (DigsX *) x;

	if (digsX->tag.type != DIGSX) 
		Error(FATAL, E_INT, "force_R_Digs", "badly formed real");

	/*
	 * Now, if there is not enough information available, we force
	 * the number of digits needed.
	 */
	if (digsX->count < digitsNeeded) {
		PUSH_3(digsX->force, digsX, digitsNeeded - digsX->count);
		runStack();
	}
}

/* force_R_Dec
 * July 2000, Marko Krznaric
 *
 * force_R_Digs(x,digitsNeeded) is a function which forces an
 * emission of digitsNeeded digit matrices from x (of the
 * type Real).
 *
 * On the other hand, force_R_Dec(x,decimalPrecision) guarantees
 * that enough digit matrices, say digitsNeeded, will be emitted
 * from a real number x in order to have required absolute
 * decimal precision, i.e. it guarantees that the result will be
 * accurate within 10^(-decimalPrecision).
 *
 * NOTES:
 * - initGuess = an initial guess (minimum number) of digit
 *   matrices which has to be emitted from a real number x.
 * - digitsNeeded = number of digit matrices required.
 * - e = number of 'bad' digits.
 * - using function retrieveInfo, we can extract the sign (=sign),
 *   the number of digits emitted so far (=count) and the
 *   compressed digits (=digits) for a Real x.
 * - 3.322 = an upper bound for log2(10).
 *
 *
 * PROBLEMS:
 * - should decimalPrecision & digitsNeeded be of type long int?
 *
 */
void
force_R_Dec(Real x, int decimalPrecision)
{
	int initGuess = ceil(3.322 * decimalPrecision) + 2;
	int digitsNeeded;
	int e = 0;

	mpz_t digits;
	Sign sign;
	int count;

	mpz_init(digits);

	digitsNeeded = initGuess;
	force_R_Digs(x, digitsNeeded);
	retrieveInfo(x, &sign, &count, digits);
	
	switch (sign) {
	case SZERO:
		/*
		 * SZERO: every digit matrix will half the interval
		 * (starting with [-1,1]). Easy to determine digitsNeeded.
		 */
		digitsNeeded = initGuess - 1;
		break;

	case SPOS:
	case SNEG:
		/*
		 * SPOS, SNEG: e, the number of 'bad' digits is actually
		 * the number of leading 1s in the binary representation
		 * of digits (the value of compressed digits).
		 */
		while (digitsNeeded < forceDecUpperBound) {
			force_R_Digs(x, digitsNeeded);
			retrieveInfo(x, &sign, &count, digits);
			if (sign == SNEG)
				mpz_neg(digits, digits);
			e = leadingOnes (digits);
			digitsNeeded = 2 * e + 2 + initGuess;
			if (count > e)
				/* not all of the extracted digit matrices are 'bad',
				 * i.e. the interval is bounded. Therefore, we can
				 * leave the loop.
				 */
				break;
		}
		if (digitsNeeded >= forceDecUpperBound)
			Error(FATAL, E_INT, "force_R_Dec",
						"forceDecUpperBound reached (1)");
		break;

	case SINF:
		/* SINF: we are still dealing with the unbounded interval,
		 * i.e. the interval contains infinity, as far as digits
		 * (the value of compressed digits matrices) is either
		 * -1, 0 or 1. As soon as we get something greater (in
		 * absolute value) than 1, the interval doesn't contain
		 * the infinity, and we can calculate digitsNeeded.
		 */
		while (digitsNeeded < forceDecUpperBound) {
			force_R_Digs(x, digitsNeeded);
			retrieveInfo(x, &sign, &count, digits);
			if (mpz_cmpabs_ui(digits, 1) > 0) {
				e = count - mpz_sizeinbase(digits, 2);
				digitsNeeded = 2 * e + 2 + initGuess;
				break;
			}
			else {
				e = count;
				digitsNeeded = 2 * e + 2+ initGuess;
			}
		}
		if (digitsNeeded >= forceDecUpperBound)
			Error(FATAL, E_INT, "force_R_Dec",
						"forceDecUpperBound reached (2)");
		break;

	case SIGN_UNKN:
		Error(FATAL, E_INT, "force_R_Dec", "argument is signed");
		break;
	
	default:
		Error(FATAL, E_INT, "force_R_Dec", "bad sign");
		break;
	}

	force_R_Digs(x, digitsNeeded);
	mpz_clear(digits);
}

/* leadingOnes
 * July 2000, Marko Krznaric
 *
 * For input variable c (of type mpz_t) check how many 1s
 * are leading the binary representation of c.
 *
 * We use this function as an auxiliary function to
 * force_R_Dec. We have to check how many D+ (D-) digit
 * matrices follow S+ (S-). This seems to be the fastest
 * way to do it.
 *
 * If c is zero or negative - result = 0.
 * If there are no zeros at all - result = binary size of c.
 * Otherwise, we scan from left-most digit and count until
 *   we reach 0.
 *
 * NOTES:
 * - The index of the right-most digit is 0, while the index
 *   of the left-most digit is (size-1).
 */ 
int
leadingOnes(mpz_t c)
{
	int size = mpz_sizeinbase(c,2);
	int i;
	int count = 0;

	if (mpz_sgn(c) <= 0)
		return 0;

	if (mpz_scan0(c, 0) == size)
		return size;

	for (i = size - 1; i >= 0; i--) {
		if (mpz_scan1(c, i) == i)
			count++;
		else
			return count;
	}
}
