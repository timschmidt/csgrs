#include "csgrs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  const csgrs_vec3_f64_t crossed[] = {{0,0,0}, {3,3,0}, {0,2,0}, {2,0,0}};
  const size_t indices[] = {0,1,2,3}, offsets[] = {0,4};
  csgrs_triangle_mesh_t *output = NULL;
  assert(csgrs_triangle_mesh_f64_polyhedron(crossed, 4, indices, 4, offsets, 2, &output) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(output == NULL);
  assert(strstr(csgrs_last_error_message(), "simple ring edges cross") != NULL);

  const csgrs_vec3_f64_t points[] = {{0,0,0}, {1,0,0}, {0,3,4}};
  const size_t face[] = {0,1,2}, bounds[] = {0,3};
  assert(csgrs_triangle_mesh_f64_polyhedron(points, 3, face, 3, bounds, 2, &output) == CSGRS_STATUS_OK);
  for (int attempt = 0; attempt < 2; ++attempt) {
    csgrs_graphics_mesh_f64_t graphics = {NULL, 17, NULL, 19};
    assert(csgrs_triangle_mesh_f64_graphics_mesh(output, &graphics) == CSGRS_STATUS_OK);
    assert(graphics.vertex_len == 3 && graphics.index_len == 3);
    for (size_t i = 0; i < 3; ++i) {
      assert(graphics.vertices[i].normal.x == 0);
      assert(graphics.vertices[i].normal.y == -0.8);
      assert(graphics.vertices[i].normal.z == 0.6);
    }
    csgrs_graphics_mesh_f64_free(graphics);
  }
  csgrs_triangle_mesh_free(output);
  output = NULL;
  assert(csgrs_triangle_mesh_f64_cube(0, &output) == CSGRS_STATUS_OK);
  csgrs_graphics_mesh_f64_t empty = {NULL, 17, NULL, 19};
  assert(csgrs_triangle_mesh_f64_graphics_mesh(output, &empty) == CSGRS_STATUS_OK);
  assert(empty.vertex_len == 0 && empty.index_len == 0 && csgrs_last_error_message()[0] == '\0');
  csgrs_graphics_mesh_f64_free(empty);
  csgrs_triangle_mesh_free(output);
  puts("C polyhedron diagnostics, normal values, repeated export and empty recovery passed");
}
