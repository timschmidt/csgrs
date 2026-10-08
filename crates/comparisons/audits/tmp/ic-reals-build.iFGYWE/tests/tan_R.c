#include <stdio.h>
#include "real.h"
#include <math.h>

/*
 * Tests the tan_R when applied to a real expressed
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

	y = makeRealSignCNQInt(
				atoi(argv[1]),	/* sign */
				argv[2],		/* c */
				atoi(argv[3]), 	/* n */
				atoi(argv[4]),	/* a */
				atoi(argv[5]));	/* b */

/*
	y = vector_Int(atoi(argv[4]), atoi(argv[5]));
*/
	print_R_Dec(y, atoi(argv[6]));
	printf("\n");
	f = realToDouble(y);
	printf("x=%f\n",f);

	x = tan_R(y);
	printf("tan(x)=");
	print_R_Dec(x, atoi(argv[6]));
	printf("\n");

	z = atan_R(x);
	printf("atan(tan(x))=");
	print_R_Dec(z, atoi(argv[6]));
	printf("\n");

	z = div_R_R(y, z);
	printf("x / atan(tan(x))=");
	print_R_Dec(z, atoi(argv[6]));
	printf("\n");

	z = div_R_R(z, Pi);
	printf("(x / atan(tan(x))) / Pi=");
	print_R_Dec(z, atoi(argv[6]));
	printf("\n");

	printf("tan(x)=%f\n", tan(f));
	printf("atan(tan(x))=%f\nx / atan(tan(x))=%f\n(x / atan(tan(x)))/pi=%f\n",
			atan(tan(f)),
			f / atan(tan(f)), 
			(f / atan(tan(f))) / M_PI);
}
