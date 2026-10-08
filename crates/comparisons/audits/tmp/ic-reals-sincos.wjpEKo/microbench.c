#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "real.h"

static long long elapsed_ns(struct timespec start, struct timespec end)
{
	return (long long) (end.tv_sec - start.tv_sec) * 1000000000LL
		+ (long long) end.tv_nsec - start.tv_nsec;
}

int main(int argc, char **argv)
{
	struct timespec start, end;
	const char *mode;
	int iterations, bits, i;
	Real x, y;

	if (argc != 4) {
		fprintf(stderr, "usage: %s MODE ITERATIONS BITS\n", argv[0]);
		return 64;
	}
	mode = argv[1];
	iterations = atoi(argv[2]);
	bits = atoi(argv[3]);
	initReals();
	clock_gettime(CLOCK_MONOTONIC, &start);
	for (i = 0; i < iterations; i++) {
		if (strcmp(mode, "rational") == 0) {
			x = vector_Int((i & 15) + 1, 37);
		} else if (strcmp(mode, "matrix") == 0) {
			x = matrix_Int(vector_Int((i & 15) + 1, 37), 1, 2, 3, 5);
		} else if (strcmp(mode, "tensor") == 0) {
			x = vector_Int((i & 15) + 1, 37);
			y = vector_Int((i & 7) + 1, 29);
			x = mul_R_R(x, y);
		} else if (strcmp(mode, "exp") == 0) {
			x = exp_R(vector_Int((i & 7) + 1, 37));
		} else {
			fprintf(stderr, "unknown mode: %s\n", mode);
			return 64;
		}
		force_R_Digs(x, bits);
	}
	clock_gettime(CLOCK_MONOTONIC, &end);
	printf("mode=%s iterations=%d bits=%d total_ns=%lld ns_per_op=%.3f\n",
		mode, iterations, bits, elapsed_ns(start, end),
		(double) elapsed_ns(start, end) / iterations);
	return 0;
}
