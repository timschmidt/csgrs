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

void
print_R_Dec(Real x, int precision)
{
	force_R_Dec(x, precision);
	print_R(x);
}

void
print_R_Digs(Real x, int digitsNeeded)
{
	force_R_Digs(x, digitsNeeded);
	print_R(x);
}

/*
 * The following two functions are pretty much the same and the common
 * parts should be reconciled into a single function. The argument must 
 * have been forced so that it dereferences to a stream.
 */
void
print_R(Real x)
{
	SmallMatrix smallAccumMat;
	int tmp;
	mpf_t a, b, c, d, diff, mid;
	int count;
	Sign sign;
	DigsX *digsX;
	Real derefToStrm(Real);

	x = derefToStrm(x);

	if (x == NULL) {
		printf("[-inf,+inf]");
		return;
	}

	if (x->gen.tag.type == SIGNX) {
		sign = x->signX.tag.value;
		digsX = (DigsX *) x->signX.x;
	}
	else {
		sign = SPOS;
		digsX = (DigsX *) x;
	}

	if (digsX->tag.type != DIGSX)
			Error(FATAL, E_INT, "print_R", "real argument not a stream");

	mpf_set_default_prec(digsX->count);
	mpf_init(a);
	mpf_init(b);
	mpf_init(c);
	mpf_init(d);
	mpf_init(diff);
	mpf_init(mid);

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		makeSmallMatrixFromDigits(smallAccumMat, digsX);
		switch (sign) {
		case SIGN_UNKN :
			Error(FATAL, E_INT, "print_R", "sign not valid (1)\n");
		case SPOS :
			break;
		case SNEG :
			tmp = smallAccumMat[0][0];
			smallAccumMat[0][0] = -smallAccumMat[0][1];
			smallAccumMat[0][1] = tmp;
			tmp = smallAccumMat[1][0];
			smallAccumMat[1][0] = -smallAccumMat[1][1];
			smallAccumMat[1][1] = tmp;
			break;
		case SINF :
			tmp = smallAccumMat[0][0];
			smallAccumMat[0][0] = smallAccumMat[0][0] + smallAccumMat[0][1];
			smallAccumMat[0][1] = smallAccumMat[0][1] - tmp;
			tmp = smallAccumMat[1][0];
			smallAccumMat[1][0] = smallAccumMat[1][0] + smallAccumMat[1][1];
			smallAccumMat[1][1] = smallAccumMat[1][1] - tmp;
			break;
		case SZERO :
			tmp = smallAccumMat[0][0];
			smallAccumMat[0][0] = smallAccumMat[0][0] - smallAccumMat[0][1];
			smallAccumMat[0][1] = smallAccumMat[0][1] + tmp;
			tmp = smallAccumMat[1][0];
			smallAccumMat[1][0] = smallAccumMat[1][0] - smallAccumMat[1][1];
			smallAccumMat[1][1] = smallAccumMat[1][1] + tmp;
			break;
		}
/*
		printf("a=%d\n", smallAccumMat[0][0]);
		printf("b=%d\n", smallAccumMat[0][1]);
		printf("c=%d\n", smallAccumMat[1][0]);
		printf("d=%d\n", smallAccumMat[1][1]);
*/
		mpf_set_si(a, smallAccumMat[0][0]);
		mpf_set_si(b, smallAccumMat[0][1]);
		mpf_set_si(c, smallAccumMat[1][0]);
		mpf_set_si(d, smallAccumMat[1][1]);
	}
	else {
#endif PACK_DIGITS
		makeMatrixFromDigits(bigTmpMat, digsX);
		switch (sign) {
		case SIGN_UNKN :
			Error(FATAL, E_INT, "print_R", "sign not valid (2)\n");
		case SPOS :
			break;
		case SNEG :
			MPZ_SWAP(bigTmpMat[0][0], bigTmpMat[0][1]);
			mpz_neg(bigTmpMat[0][0], bigTmpMat[0][0]);
			MPZ_SWAP(bigTmpMat[1][0], bigTmpMat[1][1]);
			mpz_neg(bigTmpMat[1][0], bigTmpMat[1][0]);
			break;
		case SINF :
			mpz_set(tmpa_z, bigTmpMat[0][0]);
			mpz_add(bigTmpMat[0][0], bigTmpMat[0][0], bigTmpMat[0][1]);
			mpz_sub(bigTmpMat[0][1], bigTmpMat[0][1], tmpa_z);
			mpz_set(tmpa_z, bigTmpMat[1][0]);
			mpz_add(bigTmpMat[1][0], bigTmpMat[1][0], bigTmpMat[1][1]);
			mpz_sub(bigTmpMat[1][1], bigTmpMat[1][1], tmpa_z);
			break;
		case SZERO :
			mpz_set(tmpa_z, bigTmpMat[0][0]);
			mpz_sub(bigTmpMat[0][0], bigTmpMat[0][0], bigTmpMat[0][1]);
			mpz_add(bigTmpMat[0][1], bigTmpMat[0][1], tmpa_z);
			mpz_set(tmpa_z, bigTmpMat[1][0]);
			mpz_sub(bigTmpMat[1][0], bigTmpMat[1][0], bigTmpMat[1][1]);
			mpz_add(bigTmpMat[1][1], bigTmpMat[1][1], tmpa_z);
			break;
		}
/*
		dumpMatrix(bigTmpMat);
*/
		mpf_set_z(a, bigTmpMat[0][0]);
		mpf_set_z(b, bigTmpMat[0][1]);
		mpf_set_z(c, bigTmpMat[1][0]);
		mpf_set_z(d, bigTmpMat[1][1]);
#ifdef PACK_DIGITS
	}
#endif PACK_DIGITS

	count = floor(digsX->count * log10(2.0));

	if (mpf_cmp_ui(d,0) != 0 && mpf_cmp_ui(b,0) != 0) {
		mpf_div(c, c, d);
		mpf_div(a, a, b);
		mpf_add(mid, c, a);
		mpf_div_ui(mid, mid, 2);
		mpf_sub(diff, mid, a);
		mpf_abs(diff, diff);
		mpf_out_str(stdout, 10, count + 2, mid);
		printf(" +-");
		mpf_out_str(stdout, 10, 2, diff);
	}
	else {
		printf("[");
		if (mpf_cmp_ui(d,0) == 0)
			printf("NaN,");
		else {
			mpf_div(c, c, d);
			mpf_out_str(stdout, 10, count + 2, c);
			printf(",");
		}
	
		if (mpf_cmp_ui(b,0) == 0)
			printf("NaN]");
		else {
			mpf_div(a, a, b);
			mpf_out_str(stdout, 10, count + 2, a);
		}
	}

	mpf_clear(a);
	mpf_clear(b);
	mpf_clear(c);
	mpf_clear(d);
	mpf_clear(diff);
	mpf_clear(mid);
}

double
realToDouble(Real x)
{
	SmallMatrix smallAccumMat;
	int tmp;
	mpf_t a, b, c, d, diff;
	double f;
	Sign sign;
	DigsX *digsX;
	Real derefToStrm(Real);

	x = derefToStrm(x);

	if (x == NULL)
		Error(FATAL, E_INT, "realToDouble", "real argument not a stream");

	if (x->gen.tag.type == SIGNX) {
		sign = x->signX.tag.value;
		digsX = (DigsX *) x->signX.x;
	}
	else {
		sign = SPOS;
		digsX = (DigsX *) x;
	}

	if (digsX->tag.type != DIGSX)
		Error(FATAL, E_INT, "realToDouble", "real argument not a stream");

	mpf_set_default_prec(digsX->count);
	mpf_init(a);
	mpf_init(b);
	mpf_init(c);
	mpf_init(d);
	mpf_init(diff);

#ifdef PACK_DIGITS
	if (digsX->count <= DIGITS_PER_WORD) {
		makeSmallMatrixFromDigits(smallAccumMat, digsX);
		switch (sign) {
		case SIGN_UNKN :
			Error(FATAL, E_INT, "realToDouble", "sign not valid (1)\n");
		case SPOS :
			break;
		case SNEG :
			tmp = smallAccumMat[0][0];
			smallAccumMat[0][0] = -smallAccumMat[0][1];
			smallAccumMat[0][1] = tmp;
			tmp = smallAccumMat[1][0];
			smallAccumMat[1][0] = -smallAccumMat[1][1];
			smallAccumMat[1][1] = tmp;
			break;
		case SINF :
			tmp = smallAccumMat[0][0];
			smallAccumMat[0][0] = smallAccumMat[0][0] + smallAccumMat[0][1];
			smallAccumMat[0][1] = smallAccumMat[0][1] - tmp;
			tmp = smallAccumMat[1][0];
			smallAccumMat[1][0] = smallAccumMat[1][0] + smallAccumMat[1][1];
			smallAccumMat[1][1] = smallAccumMat[1][1] - tmp;
			break;
		case SZERO :
			tmp = smallAccumMat[0][0];
			smallAccumMat[0][0] = smallAccumMat[0][0] - smallAccumMat[0][1];
			smallAccumMat[0][1] = smallAccumMat[0][1] + tmp;
			tmp = smallAccumMat[1][0];
			smallAccumMat[1][0] = smallAccumMat[1][0] - smallAccumMat[1][1];
			smallAccumMat[1][1] = smallAccumMat[1][1] + tmp;
			break;
		}
		mpf_set_si(a, smallAccumMat[0][0]);
		mpf_set_si(b, smallAccumMat[0][1]);
		mpf_set_si(c, smallAccumMat[1][0]);
		mpf_set_si(d, smallAccumMat[1][1]);
	}
	else {
#endif
		makeMatrixFromDigits(bigTmpMat, digsX);
		switch (sign) {
		case SIGN_UNKN :
			Error(FATAL, E_INT, "realToDouble", "sign not valid (2)\n");
		case SPOS :
			break;
		case SNEG :
			MPZ_SWAP(bigTmpMat[0][0], bigTmpMat[0][1]);
			mpz_neg(bigTmpMat[0][0], bigTmpMat[0][0]);
			MPZ_SWAP(bigTmpMat[1][0], bigTmpMat[1][1]);
			mpz_neg(bigTmpMat[1][0], bigTmpMat[1][0]);
			break;
		case SINF :
			mpz_set(tmpa_z, bigTmpMat[0][0]);
			mpz_add(bigTmpMat[0][0], bigTmpMat[0][0], bigTmpMat[0][1]);
			mpz_sub(bigTmpMat[0][1], bigTmpMat[0][1], tmpa_z);
			mpz_set(tmpa_z, bigTmpMat[1][0]);
			mpz_add(bigTmpMat[1][0], bigTmpMat[1][0], bigTmpMat[1][1]);
			mpz_sub(bigTmpMat[1][1], bigTmpMat[1][1], tmpa_z);
			break;
		case SZERO :
			mpz_set(tmpa_z, bigTmpMat[0][0]);
			mpz_sub(bigTmpMat[0][0], bigTmpMat[0][0], bigTmpMat[0][1]);
			mpz_add(bigTmpMat[0][1], bigTmpMat[0][1], tmpa_z);
			mpz_set(tmpa_z, bigTmpMat[1][0]);
			mpz_sub(bigTmpMat[1][0], bigTmpMat[1][0], bigTmpMat[1][1]);
			mpz_add(bigTmpMat[1][1], bigTmpMat[1][1], tmpa_z);
			break;
		}
		mpf_set_z(a, bigTmpMat[0][0]);
		mpf_set_z(b, bigTmpMat[0][1]);
		mpf_set_z(c, bigTmpMat[1][0]);
		mpf_set_z(d, bigTmpMat[1][1]);
#ifdef PACK_DIGITS
	}
#endif
	if (mpf_cmp_ui(b,0) == 0 || mpf_cmp_ui(d,0) == 0)
		Error(FATAL, E_INT, "realToDouble", "NaN");
	else {
		mpf_div(a, a, b);
		f = mpf_get_d(a);
	}
	mpf_clear(a);
	mpf_clear(b);
	mpf_clear(c);
	mpf_clear(d);
	mpf_clear(diff);
	return f;
}

