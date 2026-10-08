#ifndef EXACTCORE_HYPER_COMPARISON_MEMORY_H
#define EXACTCORE_HYPER_COMPARISON_MEMORY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ec_memory_fixture ec_memory_fixture;

typedef struct ec_memory_stats {
  uint64_t allocation_count;
  uint64_t reallocation_count;
  uint64_t deallocation_count;
  uint64_t allocated_bytes;
  uint64_t reallocated_bytes;
  uint64_t deallocated_bytes;
  uint64_t live_bytes;
  uint64_t peak_live_bytes;
} ec_memory_stats;

enum ec_memory_workload {
  EC_MEMORY_LINE_POINT = 0,
  EC_MEMORY_TRIANGLE_POINT = 1,
  EC_MEMORY_TRIANGLE_PAIR = 2
};

/* Install/check the benchmark-only GMP allocator hooks. */
int ec_memory_tracking_init(void);

/* Phase controls. Reset preserves the current live-byte baseline. */
void ec_memory_tracking_enable(int enabled);
void ec_memory_tracking_reset(void);
void ec_memory_tracking_snapshot(ec_memory_stats *output);

/* Release MPFR's per-process cache before the final residual snapshot. */
void ec_memory_release_caches(void);

/* Build native exactCore inputs from `case_count` packed coordinate records. */
ec_memory_fixture *ec_memory_fixture_new(int workload,
                                        const int64_t *coordinates,
                                        size_t case_count);

/* Run every retained case `repetitions` times and return an observed checksum. */
uint64_t ec_memory_fixture_run(ec_memory_fixture *fixture,
                               uint64_t repetitions,
                               int materialize_outputs);

void ec_memory_fixture_free(ec_memory_fixture *fixture);

#ifdef __cplusplus
}
#endif

#endif
