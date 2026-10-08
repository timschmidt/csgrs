#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xr.h"

static long arg(const char *s)
{
	char *end;
	long v = strtol(s, &end, 10);
	if (*s == '\0' || *end != '\0') {
		fprintf(stderr, "bad integer: %s\n", s);
		exit(64);
	}
	return v;
}

int main(int argc, char **argv)
{
	int b, n;
	xr_t x = NULL, y = NULL;
	mpz_t z;

	if (argc < 4) {
		fprintf(stderr, "usage: %s B N OP [ARGS...]\n", argv[0]);
		return 64;
	}
	b = (int) arg(argv[1]);
	n = (int) arg(argv[2]);
	xr_set_b(b);
	if (strcmp(argv[3], "rat") == 0 && argc == 6)
		x = xr_init(arg(argv[4]), arg(argv[5]));
	else if (strcmp(argv[3], "mul") == 0 && argc == 8)
		x = xr_mul(xr_init(arg(argv[4]), arg(argv[5])),
			xr_init(arg(argv[6]), arg(argv[7])));
	else if (strcmp(argv[3], "imul") == 0 && argc == 7)
		x = xr_imul(arg(argv[4]), xr_init(arg(argv[5]), arg(argv[6])));
	else if (strcmp(argv[3], "divi") == 0 && argc == 7)
		x = xr_divi(xr_init(arg(argv[4]), arg(argv[5])), arg(argv[6]));
	else if (strcmp(argv[3], "add") == 0 && argc == 8)
		x = xr_add(xr_init(arg(argv[4]), arg(argv[5])),
			xr_init(arg(argv[6]), arg(argv[7])));
	else if (strcmp(argv[3], "sub") == 0 && argc == 8)
		x = xr_sub(xr_init(arg(argv[4]), arg(argv[5])),
			xr_init(arg(argv[6]), arg(argv[7])));
	else if (strcmp(argv[3], "sqr") == 0 && argc == 6)
		x = xr_sqr(xr_init(arg(argv[4]), arg(argv[5])));
	else if (strcmp(argv[3], "sqrt") == 0 && argc == 6)
		x = xr_sqrt(xr_init(arg(argv[4]), arg(argv[5])));
	else if (strcmp(argv[3], "root") == 0 && argc == 7)
		x = xr_root(xr_init(arg(argv[4]), arg(argv[5])), arg(argv[6]));
	else if (strcmp(argv[3], "recip") == 0 && argc == 6)
		x = xr_recip(xr_init(arg(argv[4]), arg(argv[5])));
	else if (strcmp(argv[3], "powi") == 0 && argc == 7)
		x = xr_powi(xr_init(arg(argv[4]), arg(argv[5])), arg(argv[6]));
	else if (strcmp(argv[3], "exp") == 0 && argc == 6)
		x = xr_exp(xr_init(arg(argv[4]), arg(argv[5])));
	else if (strcmp(argv[3], "exp1") == 0 && argc == 6)
		x = xr_exp1(xr_init(arg(argv[4]), arg(argv[5])));
	else if (strcmp(argv[3], "pi") == 0 && argc == 4)
		x = xr_pi();
	else if (strcmp(argv[3], "near") == 0 && argc == 6)
		x = xr_near_int(xr_init(arg(argv[4]), arg(argv[5])));
	else if (strcmp(argv[3], "cmpmul") == 0 && argc == 10) {
		x = xr_mul(xr_init(arg(argv[4]), arg(argv[5])),
			xr_init(arg(argv[6]), arg(argv[7])));
		y = xr_init(arg(argv[8]), arg(argv[9]));
		printf("%d\n", xr_cmp(x, y));
		return 0;
	}
	else if (strcmp(argv[3], "cmpimul") == 0 && argc == 9) {
		x = xr_imul(arg(argv[4]), xr_init(arg(argv[5]), arg(argv[6])));
		y = xr_init(arg(argv[7]), arg(argv[8]));
		printf("%d\n", xr_cmp(x, y));
		return 0;
	}
	else {
		fprintf(stderr, "bad operation or arity\n");
		return 64;
	}
	mpz_init(z);
	xr_eval(z, x, n);
	mpz_out_str(stdout, 10, z);
	putchar('\n');
	mpz_clear(z);
	return 0;
}
