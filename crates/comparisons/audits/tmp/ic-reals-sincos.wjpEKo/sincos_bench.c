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
	volatile Real sink = NULL;
	int iterations, bits, i;

	if (argc != 4) {
		fprintf(stderr, "usage: %s sin|cos ITERATIONS BITS\n", argv[0]);
		return 64;
	}
	iterations = atoi(argv[2]);
	bits = atoi(argv[3]);
	initReals();
	clock_gettime(CLOCK_MONOTONIC, &start);
	for (i = 0; i < iterations; ++i) {
		Real x = strcmp(argv[1], "sin") == 0
			? sin_QInt((i & 7) + 1, 37)
			: cos_QInt((i & 7) + 1, 37);
		if (bits > 0)
			force_R_Digs(x, bits);
		sink = x;
	}
	clock_gettime(CLOCK_MONOTONIC, &end);
	printf("mode=%s iterations=%d bits=%d total_ns=%lld ns_per_op=%.3f sink=%p\n",
		argv[1], iterations, bits, elapsed_ns(start, end),
		(double) elapsed_ns(start, end) / iterations, (void *) sink);
	return 0;
}
