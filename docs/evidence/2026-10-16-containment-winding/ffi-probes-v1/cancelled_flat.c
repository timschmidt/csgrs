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
  csgrs_triangle_mesh_t *cube = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  const boolean_fn operations[] = {csgrs_triangle_mesh_f64_union,
    csgrs_triangle_mesh_f64_intersection, csgrs_triangle_mesh_f64_difference,
    csgrs_triangle_mesh_f64_xor};
  const double levels[] = {0.5, 1, 2};
  for (size_t copies=1; copies<=3; ++copies) {
    for (size_t plane=0; plane<3; ++plane) {
      const double z=levels[plane];
      const csgrs_vec3_f64_t points[] = {{-2,-2,z},{4,-2,z},{-2,4,z}};
      size_t indices[18], offsets[7];
      const size_t pair[] = {0,1,2,2,1,0};
      for (size_t i=0; i<6*copies; ++i) indices[i]=pair[i%6];
      for (size_t i=0; i<=2*copies; ++i) offsets[i]=3*i;
      csgrs_triangle_mesh_t *flat=NULL;
      assert(csgrs_triangle_mesh_f64_polyhedron(points,3,indices,6*copies,offsets,2*copies+1,&flat)==CSGRS_STATUS_OK);
      for (size_t operation=0; operation<4; ++operation) {
        for (int reverse=0; reverse<2; ++reverse) {
          csgrs_triangle_mesh_t *output=NULL,*continued=NULL;
          assert(operations[operation](reverse?flat:cube,reverse?cube:flat,&output)==CSGRS_STATUS_OK);
          if (operation==1 || (operation==2 && reverse)) {
            check_empty(output);
            assert(csgrs_triangle_mesh_f64_union(output,cube,&continued)==CSGRS_STATUS_OK);
            check_unit_box(continued);
          } else {
            check_unit_box(output);
            assert(csgrs_triangle_mesh_f64_difference(output,cube,&continued)==CSGRS_STATUS_OK);
            check_empty(continued);
          }
          csgrs_triangle_mesh_free(continued);csgrs_triangle_mesh_free(output);
        }
      }
      csgrs_triangle_mesh_free(flat);
    }
  }
  csgrs_triangle_mesh_free(cube);
  puts("C cancelled sheets: 72 Boolean cases and downstream compositions passed");
}
