#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "real.h"

static Real apply(const char *op, Real x)
{
	if (strcmp(op, "id") == 0) return x;
	if (strcmp(op, "neg") == 0) return neg_R(x);
	if (strcmp(op, "abs") == 0) return abs_R(x);
	if (strcmp(op, "sqrt") == 0) return sqrt_R(x);
	if (strcmp(op, "exp") == 0) return exp_R(x);
	if (strcmp(op, "log") == 0) return log_R(x);
	if (strcmp(op, "sin") == 0) return sin_R(x);
	if (strcmp(op, "cos") == 0) return cos_R(x);
	if (strcmp(op, "tan") == 0) return tan_R(x);
	if (strcmp(op, "asin") == 0) return asin_R(x);
	if (strcmp(op, "acos") == 0) return acos_R(x);
	if (strcmp(op, "atan") == 0) return atan_R(x);
	if (strcmp(op, "sinh") == 0) return sinh_R(x);
	if (strcmp(op, "cosh") == 0) return cosh_R(x);
	if (strcmp(op, "tanh") == 0) return tanh_R(x);
	if (strcmp(op, "asinh") == 0) return asinh_R(x);
	if (strcmp(op, "acosh") == 0) return acosh_R(x);
	if (strcmp(op, "atanh") == 0) return atanh_R(x);
	if (strcmp(op, "square") == 0) return mul_R_R(x, x);
	if (strcmp(op, "recip") == 0) return div_Int_R(1, x);
	if (strcmp(op, "powself") == 0) return pow_R_R(x, x);
	if (strcmp(op, "pythag") == 0) {
		Real s = sin_R(x);
		Real c = cos_R(x);
		return add_R_R(mul_R_R(s, s), mul_R_R(c, c));
	}
	if (strcmp(op, "logexp") == 0) return log_R(exp_R(x));
	if (strcmp(op, "expsquare") == 0) {
		Real e = exp_R(x);
		return sub_R_R(mul_R_R(e, e), exp_R(mul_R_Int(x, 2)));
	}
	fprintf(stderr, "unknown operation: %s\n", op);
	exit(64);
}

int main(int argc, char **argv)
{
	Real x;
	Real y;
	long numerator;
	long denominator;
	int digits;

	if (argc != 5) {
		fprintf(stderr, "usage: %s OP NUM DEN DIGITS\n", argv[0]);
		return 64;
	}
	numerator = strtol(argv[2], NULL, 10);
	denominator = strtol(argv[3], NULL, 10);
	digits = atoi(argv[4]);
	initReals();
	if (strcmp(argv[1], "pi") == 0)
		y = Pi;
	else if (strcmp(argv[1], "e") == 0)
		y = E;
	else {
		x = vector_Int((int) numerator, (int) denominator);
		y = apply(argv[1], x);
	}
	print_R_Dec(y, digits);
	putchar('\n');
	return 0;
}
