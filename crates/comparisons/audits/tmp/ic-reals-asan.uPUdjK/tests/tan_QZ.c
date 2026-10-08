#include <stdio.h>
#include "real.h"
#include <math.h>

/*
 * Tests the tan_QZ when applied to a rational.
 */
main(int argc, char *argv[])
{
	Real x, y;
	double f;
	mpz_t a, b;

	MyName = argv[0];

	if (argc != 4) {
		fprintf(stderr, "%s <a> <b> <ndigits>\n", MyName);
		exit(1);
	}

	initReals();

	mpz_init_set_str(a, argv[1], 10);
	mpz_init_set_str(b, argv[2], 10);

	x = tan_QZ(a, b);

	print_R_Dec(x, atoi(argv[3]));
	printf("\n");
}
