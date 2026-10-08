#include <stdio.h>
#include "real.h"
#include <math.h>

/*
 * Tests the exp_QInt when applied to an int rational.
 */
main(int argc, char *argv[])
{
	Real x, y;
	int a, b;
	double f;
	Real makeRealSignCNQInt(Sign, char *, int, int, int);

	MyName = argv[0];

	if (argc != 4) {
		fprintf(stderr, "%s <a> <b> <ndigits>\n", MyName);
		exit(1);
	}

	initReals();

	a = atoi(argv[1]);
	b = atoi(argv[2]);	

	x = exp_QInt(a, b);

	print_R_Dec(x, atoi(argv[3]));
	printf("\n");
	printf("exp(x)=%f\n", exp(f));
}
