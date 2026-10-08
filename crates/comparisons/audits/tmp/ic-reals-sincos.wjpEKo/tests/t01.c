#include <stdio.h>
#include "real.h"
#include <math.h>

/*
 * Random test
 */
main(int argc, char *argv[])
{
	Real u, v, w, x, y, z;
	Bool a, b, c, d;
	double f;
	Real makeRealSignCNQInt(Sign, char *, int, int, int);

	MyName = argv[0];

/*
	if (argc != 7) {
		fprintf(stderr, "%s <sign> <c> <n> <a> <b> <ndigits>\n", MyName);
		exit(1);
	}
*/

	if (argc != 4) {
		fprintf(stderr, "%s <a> <b> <ndigits>\n", MyName);
		exit(1);
	}

	initReals();

#ifdef JUNK
	y = makeRealSignCNQInt(
				atoi(argv[1]),	/* sign */
				argv[2],		/* c */
				atoi(argv[3]), 	/* n */
				atoi(argv[4]),	/* a */
				atoi(argv[5]));	/* b */
#endif

	y = makeStream(vector_Int(atoi(argv[1]), atoi(argv[2])));
	print_R_Dec(y, atoi(argv[3]));
	printf("\n");

	y = makeStream(y);

	u = makeStream(vector_Int(-1, 2));
	u = makeStream(mul_R_Int(u, 1));
	v = makeStream(vector_Int(1, 3));
	v = makeStream(mul_R_Int(v, 1));
	w = makeStream(vector_Int(1, 4));
	w = makeStream(mul_R_Int(w, 1));
	x = makeStream(vector_Int(1, 5));

	a = and_B_B(gt_R_QInt(y, 0, 5), lt_R_QInt(y, 2, 5));
	b = and_B_B(gt_R_QInt(y, 1, 5), lt_R_QInt(y, 3, 5));
	c = and_B_B(gt_R_QInt(y, 2, 5), lt_R_QInt(y, 4, 5));
	d = and_B_B(gt_R_QInt(y, 3, 5), lt_R_QInt(y, 5, 5));

	z = realIf(4, a, u, b, v, c, w, d, x);
	print_R_Dec(z, atoi(argv[3]));
	printf("\n");

	z = add_R_R(z, z);

	print_R_Dec(z, atoi(argv[3]));
	printf("\n");
}
