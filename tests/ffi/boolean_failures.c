#include "csgrs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef csgrs_status_t (*boolean_fn)(const csgrs_triangle_mesh_t *,
                                   const csgrs_triangle_mesh_t *,
                                   csgrs_triangle_mesh_t **);

int main(void) {
  csgrs_triangle_mesh_t *cube = NULL, *rotated = NULL, *restored = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_rotate(cube, 0, 0, 13, &rotated) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_rotate(rotated, 0, 0, -13, &restored) == CSGRS_STATUS_OK);
  const boolean_fn operations[] = {
      csgrs_triangle_mesh_f64_union, csgrs_triangle_mesh_f64_difference,
      csgrs_triangle_mesh_f64_intersection, csgrs_triangle_mesh_f64_xor};
  size_t structured = 0;
  for (size_t i = 0; i < 4; ++i) {
    csgrs_triangle_mesh_t *result = NULL;
    const csgrs_status_t status = operations[i](cube, restored, &result);
    assert(status != CSGRS_STATUS_OK && result == NULL);
    assert(strstr(csgrs_last_error_message(), "predicate could not be decided") != NULL);
    structured += status == CSGRS_STATUS_PREDICATE_UNDECIDED;
  }
  printf("C undecided Boolean statuses: %zu/4\n", structured);
  fflush(stdout);
  assert(structured == 4);
  csgrs_triangle_mesh_t *empty = NULL, *recovered = NULL;
  assert(csgrs_triangle_mesh_f64_difference(cube, cube, &empty) == CSGRS_STATUS_OK);
  assert(strcmp(csgrs_last_error_message(), "") == 0);
  csgrs_triangle_mesh_buffers_f64_t buffers;
  assert(csgrs_triangle_mesh_f64_vertices_and_indices(empty, &buffers) == CSGRS_STATUS_OK);
  assert(buffers.vertex_len == 0 && buffers.index_len == 0);
  csgrs_triangle_mesh_buffers_f64_free(buffers);
  assert(csgrs_triangle_mesh_f64_union(empty, cube, &recovered) == CSGRS_STATUS_OK);
  csgrs_aabb3_f64_t bounds;
  assert(csgrs_triangle_mesh_f64_bounding_box(recovered, &bounds) == CSGRS_STATUS_OK);
  assert(bounds.mins.x == 0 && bounds.maxs.x == 1);
  csgrs_triangle_mesh_free(recovered);
  csgrs_triangle_mesh_free(empty);
  csgrs_triangle_mesh_free(restored);
  csgrs_triangle_mesh_free(rotated);
  csgrs_triangle_mesh_free(cube);
  puts("C typed Boolean failures, empty result and recovery passed");
}
