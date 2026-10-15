#include "csgrs.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  csgrs_triangle_mesh_t *mesh = NULL;
  assert(csgrs_triangle_mesh_f64_cube(-1, &mesh) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(mesh == NULL && strstr(csgrs_last_error_message(), "width") != NULL);
  assert(csgrs_triangle_mesh_f64_cuboid(0, -1, 1, &mesh) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(mesh == NULL);
  assert(csgrs_triangle_mesh_f64_sphere(0, 2, 4, &mesh) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(mesh == NULL);
  assert(csgrs_triangle_mesh_f64_sphere(1, SIZE_MAX, 2, &mesh) == CSGRS_STATUS_RESOURCE_EXHAUSTED);
  assert(mesh == NULL && strstr(csgrs_last_error_message(), "capacity") != NULL);
  assert(csgrs_triangle_mesh_f64_cube(0, &mesh) == CSGRS_STATUS_OK);
  csgrs_triangle_mesh_buffers_f64_t buffers;
  assert(csgrs_triangle_mesh_f64_vertices_and_indices(mesh, &buffers) == CSGRS_STATUS_OK);
  assert(buffers.vertex_len == 0 && buffers.index_len == 0);
  csgrs_triangle_mesh_buffers_f64_free(buffers);
  csgrs_triangle_mesh_t *cube = NULL, *restored = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_union(mesh, cube, &restored) == CSGRS_STATUS_OK);
  csgrs_triangle_mesh_free(restored);
  csgrs_triangle_mesh_free(cube);
  csgrs_triangle_mesh_free(mesh);
  puts("C primitive errors, empty composition, and recovery passed");
}
