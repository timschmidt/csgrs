#include <stdio.h>
#include "real.h"
#include <math.h>

/*
 * Tests the tanh_R when applied to a real expressed
 * by a sign, a characteristic pair and a vector.
 */
main(int argc, char *argv[])
{
	Real x, y;
	double f;
	Real makeRealSignCNQInt(Sign, char *, int, int, int);

	MyName = argv[0];

	if (argc != 7) {
		fprintf(stderr, "%s <sign> <c> <n> <a> <b> <ndigits>\n", MyName);
		exit(1);
	}

	initReals();

	y = makeRealSignCNQInt(
				atoi(argv[1]),	/* sign */
				argv[2],		/* c */
				atoi(argv[3]), 	/* n */
				atoi(argv[4]),	/* a */
				atoi(argv[5]));	/* b */

	print_R_Dec(y, atoi(argv[6]));
	printf("\n");
	f = realToDouble(y);
	printf("x=%f\n",f);

	x = tanh_R(y);

	print_R_Dec(x, atoi(argv[6]));
	printf("\n");
	printf("tanh(x)=%f\n", tanh(f));
}
