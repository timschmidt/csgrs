#include <stdio.h>
#include "real.h"
#include <math.h>

/*
 * Tests the cos_R when applied to a rational a real expressed
 * by a sign, a characteristic pair and a vector.
 */
main(int argc, char *argv[])
{
	Real x, y, z;
	double f;
	Real makeRealSignCNQInt(Sign, char *, int, int, int);

	MyName = argv[0];

	if (argc != 7) {
		fprintf(stderr, "%s <sign> <c> <n> <a> <b> <ndigits>\n", MyName);
		exit(1);
	}

	initReals();

#ifdef NOT_NOW
	y = makeRealSignCNQInt(
				atoi(argv[1]),	/* sign */
				argv[2],		/* c */
				atoi(argv[3]), 	/* n */
				atoi(argv[4]),	/* a */
				atoi(argv[5]));	/* b */
#endif

	y = vector_Int(atoi(argv[4]), atoi(argv[5]));

	print_R_Dec(y, atoi(argv[6]));
	printf("\n");

	f = realToDouble(y);
	printf("x=%f\n",f);

	x = cos_R(y);
	x = mul_R_R(x, x);

	z = sin_R(y);
	z = mul_R_R(z, z);

	x = add_R_R(x, z);
	x = sqrt_R(x);

	print_R_Dec(x, atoi(argv[6]));
	printf("\n");
	printf("t1(x)=%f\n", sqrt(cos(f)*cos(f) + sin(f)*sin(f)));
}
