#include <stdio.h>
#include "real.h"
#include "real-impl.h"

main(int argc, char *argv[])
{
	Real x, y;
	double f;
	Real makeRealSignCNQInt(Sign, char *, int, int, int);
	Real iterate(Real, int);
	int niters;

	MyName = argv[0];

	if (argc != 8) {
		fprintf(stderr, "%s <sign> <c> <n> <a> <b> <n-iters> <ndigits>\n",
				MyName);
		exit(1);
	}

	initReals();

	y = makeRealSignCNQInt(
				atoi(argv[1]),  /* sign */
				argv[2],		/* c */
				atoi(argv[3]),  /* n */
				atoi(argv[4]),  /* a */
				atoi(argv[5])); /* b */

	niters = atoi(argv[6]);

	print_R_Dec(y, atoi(argv[7]));
	printf("\n");

	x = iterate(y, niters);

	print_R_Dec(x, atoi(argv[7]));
	printf("\n");
}

/*
 * Iterates the function f(x) = 4x(1-x)
 */
Real
iterate(Real x, int n)
{
	Real p, q, r; 

	if (n > 0) {
		r = iterate(x, n - 1);
		p = mul_R_Int(r, 4);
		q = sub_Int_R(1, r);
		r = mul_R_R(p, q);
		if (n > 15)
			r = makeStream(r);
		return r;
	}
	else
		return x;
}
