#include <stdio.h>
#include "real.h"

/*
 * Prints pi (in base 10) to the specified number of digits.
 */
main(int argc, char *argv[])
{
	MyName = argv[0];

	if (argc != 2) {
		fprintf(stderr, "%s <n>\n", MyName);
		exit(1);
	}

	initReals();

	print_R_Dec(Pi, atoi(argv[1]));
	printf("\n");
}
