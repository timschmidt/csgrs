/* Compile and run against the actual csgrs-ffi library and public C header. */
#include "csgrs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef csgrs_status_t (*boolean_fn)(const csgrs_triangle_mesh_t *,
                                   const csgrs_triangle_mesh_t *,
                                   csgrs_triangle_mesh_t **);

static void check_empty(const csgrs_triangle_mesh_t *mesh) {
  csgrs_triangle_mesh_buffers_f64_t buffers;
  assert(csgrs_triangle_mesh_f64_vertices_and_indices(mesh, &buffers) == CSGRS_STATUS_OK);
  assert(buffers.vertex_len == 0 && buffers.index_len == 0);
  csgrs_triangle_mesh_buffers_f64_free(buffers);
}

static void check_unit_box(const csgrs_triangle_mesh_t *mesh) {
  csgrs_triangle_mesh_buffers_f64_t buffers;
  assert(csgrs_triangle_mesh_f64_vertices_and_indices(mesh, &buffers) == CSGRS_STATUS_OK);
  assert(buffers.index_len > 0);
  double six_volume = 0;
  for (size_t i = 0; i < buffers.vertex_len; ++i) {
    const csgrs_vec3_f64_t p = buffers.vertices[i];
    assert(p.x >= 0 && p.x <= 1 && p.y >= 0 && p.y <= 1 && p.z >= 0 && p.z <= 1);
  }
  for (size_t i = 0; i < buffers.index_len; ++i) {
    const csgrs_triangle_u32_t t = buffers.indices[i];
    assert(t.a < buffers.vertex_len && t.b < buffers.vertex_len && t.c < buffers.vertex_len);
    const csgrs_vec3_f64_t a = buffers.vertices[t.a];
    const csgrs_vec3_f64_t b = buffers.vertices[t.b];
    const csgrs_vec3_f64_t c = buffers.vertices[t.c];
    assert((a.x == b.x && b.x == c.x && (a.x == 0 || a.x == 1)) ||
           (a.y == b.y && b.y == c.y && (a.y == 0 || a.y == 1)) ||
           (a.z == b.z && b.z == c.z && (a.z == 0 || a.z == 1)));
    six_volume += a.x * (b.y * c.z - b.z * c.y) +
                  a.y * (b.z * c.x - b.x * c.z) +
                  a.z * (b.x * c.y - b.y * c.x);
  }
  assert(six_volume == 6); /* The coordinates here are exact binary64 integers. */
  csgrs_triangle_mesh_buffers_f64_free(buffers);
}

int main(void) {
  csgrs_triangle_mesh_t *cube = NULL, *empty = NULL, *open = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_difference(cube, cube, &empty) == CSGRS_STATUS_OK);
  check_empty(empty);
  const csgrs_vec3_f64_t points[] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const size_t indices[] = {0, 1, 2}, offsets[] = {0, 3};
  assert(csgrs_triangle_mesh_f64_polyhedron(points, 3, indices, 3, offsets, 2, &open)
         == CSGRS_STATUS_OK);
  const boolean_fn operations[] = {
      csgrs_triangle_mesh_f64_union, csgrs_triangle_mesh_f64_intersection,
      csgrs_triangle_mesh_f64_difference, csgrs_triangle_mesh_f64_xor};
  for (size_t operation = 0; operation < 4; ++operation) {
    for (int empty_on_left = 0; empty_on_left < 2; ++empty_on_left) {
      csgrs_triangle_mesh_t *result = NULL, *next = NULL;
      assert(operations[operation](empty_on_left ? empty : cube,
                                   empty_on_left ? cube : empty, &result) == CSGRS_STATUS_OK);
      if (operation == 1 || (operation == 2 && empty_on_left)) {
        check_empty(result);
        assert(csgrs_triangle_mesh_f64_union(result, cube, &next) == CSGRS_STATUS_OK);
        check_unit_box(next);
      } else {
        check_unit_box(result);
        assert(csgrs_triangle_mesh_f64_difference(result, cube, &next) == CSGRS_STATUS_OK);
        check_empty(next);
      }
      csgrs_triangle_mesh_free(next);
      csgrs_triangle_mesh_free(result);
      result = NULL;
      assert(operations[operation](empty_on_left ? empty : open,
                                   empty_on_left ? open : empty, &result)
             == CSGRS_STATUS_VALIDATION_FAILED);
      assert(result == NULL);
      assert(strcmp(csgrs_last_error_message(), empty_on_left
                    ? "input mesh 1 has 3 boundary edges"
                    : "input mesh 0 has 3 boundary edges") == 0);
    }
  }
  csgrs_triangle_mesh_free(open);
  csgrs_triangle_mesh_free(empty);
  csgrs_triangle_mesh_free(cube);
  puts("C consumer: empty composition, exact box boundaries/volume, and visible failures passed");
  return 0;
}
