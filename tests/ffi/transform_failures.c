#include "csgrs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  csgrs_triangle_mesh_t *cube = NULL, *shifted = NULL, *result = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_scale(cube, 0, 1, 1, &result) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(result == NULL);
  const csgrs_matrix4_f64_t reciprocal_x = {{0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0}};
  assert(csgrs_triangle_mesh_f64_translate(cube, -0.5, 0, 0, &shifted) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_transform(shifted, reciprocal_x, &result) == CSGRS_STATUS_UNBOUNDED);
  assert(result == NULL && strstr(csgrs_last_error_message(), "crosses infinity") != NULL);
  assert(csgrs_triangle_mesh_f64_transform(cube, reciprocal_x, &result) == CSGRS_STATUS_UNBOUNDED);
  assert(result == NULL && strstr(csgrs_last_error_message(), "point is at infinity") != NULL);
  assert(csgrs_triangle_mesh_f64_translate(shifted, 2, 0, 0, &result) == CSGRS_STATUS_OK);
  csgrs_triangle_mesh_free(result);
  csgrs_triangle_mesh_free(shifted);
  csgrs_triangle_mesh_free(cube);
  puts("C typed transform errors and recovery passed");
}
