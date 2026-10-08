#include <stdio.h>
#include <stdlib.h>
#include "xr.h"

static xr_t logistic(int iterations, int warm_precision)
{
	int i;
	xr_t x = xr_init(671875, 1000000);
	mpz_t scratch;
	mpz_init(scratch);
	for (i = 0; i < iterations; ++i) {
		if (warm_precision >= 0)
			xr_eval(scratch, x, warm_precision);
		x = xr_imul(4, xr_mul(x, xr_isub(1, x)));
	}
	mpz_clear(scratch);
	return x;
}

int main(int argc, char **argv)
{
	int b, iterations, warm_precision, final_precision;
	xr_t cold, warm;
	mpz_t a, z, delta;
	if (argc != 5)
		return 64;
	b = atoi(argv[1]);
	iterations = atoi(argv[2]);
	warm_precision = atoi(argv[3]);
	final_precision = atoi(argv[4]);
	xr_set_b(b);
	cold = logistic(iterations, -1);
	warm = logistic(iterations, warm_precision);
	mpz_init(a);
	mpz_init(z);
	mpz_init(delta);
	xr_eval(a, cold, final_precision);
	xr_eval(z, warm, final_precision);
	mpz_sub(delta, a, z);
	mpz_abs(delta, delta);
	printf("cold="); mpz_out_str(stdout, 10, a);
	printf("\nwarm="); mpz_out_str(stdout, 10, z);
	printf("\ndelta="); mpz_out_str(stdout, 10, delta);
	putchar('\n');
	return 0;
}
