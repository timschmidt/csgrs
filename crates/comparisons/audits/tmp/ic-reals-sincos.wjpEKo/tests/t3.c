#include <stdio.h>
#include "real.h"
#include <math.h>

main(int argc, char *argv[])
{
	Real x, y, z;
	int a, b;
	double f;

	MyName = argv[0];

	if (argc != 4) {
		fprintf(stderr, "%s <a> <b> <ndigits>\n", MyName);
		exit(1);
	}

	initReals();

	a = atoi(argv[1]);
	b = atoi(argv[2]);
	x = real_QInt(a, b);
	y = mul_R_R(x, x);
	print_R_Dec(y, atoi(argv[3]));
	printf("\n");
}
