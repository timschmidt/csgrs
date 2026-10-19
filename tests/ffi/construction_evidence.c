#include "csgrs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void) {
  csgrs_triangle_mesh_t *cube = NULL, *shifted = NULL, *empty = NULL, *sphere = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  csgrs_construction_evidence_t evidence;
  memset(&evidence, 0x77, sizeof evidence);
  const csgrs_construction_evidence_t sentinel = evidence;
  assert(csgrs_triangle_mesh_construction_evidence(NULL, &evidence) == CSGRS_STATUS_NULL_POINTER);
  assert(memcmp(&sentinel, &evidence, sizeof evidence) == 0);
  assert(csgrs_triangle_mesh_construction_evidence(cube, NULL) == CSGRS_STATUS_NULL_POINTER);
  assert(csgrs_triangle_mesh_construction_evidence(cube, &evidence) == CSGRS_STATUS_OK);
  assert(!evidence.source_history_missing && !evidence.approximate_512_consumed);
  assert(!evidence.coordinates_projected && !evidence.source_sampled && !evidence.buffers_changed);
  assert(csgrs_triangle_mesh_f64_translate(cube, 3, 5, 7, &shifted) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_difference(shifted, shifted, &empty) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_construction_evidence(empty, &evidence) == CSGRS_STATUS_OK);
  assert(!evidence.source_history_missing && !evidence.approximate_512_consumed);
  assert(csgrs_triangle_mesh_f64_sphere(1, 8, 4, &sphere) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_construction_evidence(sphere, &evidence) == CSGRS_STATUS_OK);
  assert(evidence.source_sampled && !evidence.source_history_missing);
  csgrs_triangle_mesh_free(sphere); csgrs_triangle_mesh_free(empty);
  csgrs_triangle_mesh_free(shifted); csgrs_triangle_mesh_free(cube);
  assert(evidence.source_sampled); /* Caller-owned summary outlives the handle. */
  puts("C construction evidence and typed null failures passed");
}
