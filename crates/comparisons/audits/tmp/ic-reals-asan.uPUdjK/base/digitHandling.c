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
 * For a real that has been forced to a stream (by force_R_Digs)
 * this function retrieves the sign and the most significant digit information.
 */
void
retrieveInfo(Real x, Sign *sign, int *count, mpz_t digits)
{
	Real derefToStrm(Real);

	x = derefToStrm(x);

	if (x == NULL)
		Error(FATAL, E_INT, "retrieveInfo", "argument not forced");

	if (x->gen.tag.type == SIGNX) {
		*sign = x->signX.tag.value;
		x = x->signX.x;
	}
	else
		*sign = SPOS;

	if (x->gen.tag.type == DIGSX) {
		*count = x->digsX.count;
#ifdef PACK_DIGITS
		if (*count <= DIGITS_PER_WORD)
        	mpz_set_si(digits, x->digsX.word.small);
    	else
#endif
        	mpz_set(digits, x->digsX.word.big);
	}
	else
		Error(FATAL, E_INT, "retrieveInfo",
				"no information has been forced from the argument");
}

/*
 * Given a characteristic pair (c,n) where n is the number
 * of digits recorded in c, this function returns the most significant
 * digit yielding also (c', n-1)
 */
Digit
takeDigit(int *count, mpz_t digits)
{
	Digit digit;

	if (*count > 0) {
		mpz_tdiv_q_2exp(tmpa_z, digits, *count - 1);
		if (mpz_cmp_ui(tmpa_z, 0) == 0)
			digit = DZERO;
		else
			if (mpz_cmp_ui(tmpa_z, 1) == 0)
				digit = DPOS;
			else
				if (mpz_cmp_si(tmpa_z, -1) == 0)
					digit = DNEG;
				else
					Error(FATAL, E_INT, "takeDigit",
						"characteristic pair is not well formed");
		mpz_mul_2exp(tmpa_z, tmpa_z, *count - 1);
		mpz_sub(digits, digits, tmpa_z);
		*count = *count - 1;
		return digit;
	}
	Error(FATAL, E_INT, "takeDigit", "empty characteristic pair");
	return DZERO;
}
