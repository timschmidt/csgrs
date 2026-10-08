#include <stdio.h>
#include "real.h"
#include <math.h>

/*
 * This is the same as the tan_R test but shows how to retrieve digits
 * from a real incrementally.
 */
main(int argc, char *argv[])
{
	Real x, y;
	double f;
	Real makeRealSignCNQInt(Sign, char *, int, int, int);
	mpz_t digits;
	Sign sign;
	Digit digit;
	int count;

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

	x = tan_R(y);

	print_R_Dec(x, atoi(argv[6]));
	printf("\n");
	printf("tan(x)=%f\n", tan(f));

	mpz_init(digits);
	retrieveInfo(x, &sign, &count, digits);

	printf("%s ", signToString(sign));
	while (count > 0) {
		digit = takeDigit(&count, digits);
		printf("%s ", digitToString(digit));
	}
	printf("\n");
}
