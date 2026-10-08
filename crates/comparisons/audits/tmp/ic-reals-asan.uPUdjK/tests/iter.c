#include <stdio.h>
#include "real.h"
#include "real-impl.h"
#include <math.h>

/*
 * Reinhold's iter example from the user manual.
 */
Real eps, eps2;
Real delay(Real (*)(Real), Real);

main(int argc, char *argv[])
{
	Real x, y, z;
	double f;
	Real makeRealSignCNQInt(Sign, char *, int, int, int);
	Real iter(Real);

	MyName = argv[0];

	if (argc != 7) {
		fprintf(stderr, "%s <sign> <c> <n> <a> <b> <ndigits>\n", MyName);
		exit(1);
	}

	initReals();

	debugTrace(1);

	y = makeRealSignCNQInt(
				atoi(argv[1]),	/* sign */
				argv[2],		/* c */
				atoi(argv[3]), 	/* n */
				atoi(argv[4]),	/* a */
				atoi(argv[5]));	/* b */

	print_R_Dec(y, atoi(argv[6]));
	printf("\n");

	eps = vector_Int(1, 100000000);
	eps2 = vector_Int(1, 200000000);

	x = iter(y);
	print_R_Dec(x, atoi(argv[6]));
	printf("\n");
}

Real
iter(Real x)
{
	Real y, d;
	static int doneInit = 0;
	void delayCls();

	y = div_R_Int(x, 2);
	d = abs_R(sub_R_R(x, y));
	return realIf(2,
		lt_R_R(d, eps), y,
		gt_R_R(d, eps2), realDelay((Delay_Fun) iter, (Delay_Arg) y));
}
