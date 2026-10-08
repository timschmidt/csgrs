#ifndef EXACTCORE_HYPER_COMPARISON_BENCH_H
#define EXACTCORE_HYPER_COMPARISON_BENCH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ec_benchmark_fixture ec_benchmark_fixture;

/*
 * Construct one of the fixed-input native fixtures named by the Criterion
 * operation ID. All parsing and exactCore input-object construction happens
 * here, outside the measured loop.
 */
ec_benchmark_fixture *ec_benchmark_fixture_new(const char *operation_id);

/* Run the retained native operation `iterations` times. */
int ec_benchmark_fixture_run(ec_benchmark_fixture *fixture,
                             uint64_t iterations);

/* Destroy a retained fixture and all exactCore objects it owns. */
void ec_benchmark_fixture_free(ec_benchmark_fixture *fixture);

#ifdef __cplusplus
}
#endif

#endif
