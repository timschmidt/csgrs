#include "csgrs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  const csgrs_vec3_f64_t crossed[] = {{0,0,0},{3,3,0},{0,2,0},{2,0,0}};
  const size_t face[] = {0,1,2,3}, offsets[] = {0,4};
  csgrs_triangle_mesh_t *mesh = NULL;
  assert(csgrs_triangle_mesh_f64_polyhedron(crossed,4,face,4,offsets,2,&mesh)
         == CSGRS_STATUS_VALIDATION_FAILED);
  assert(mesh == NULL);
  assert(strstr(csgrs_last_error_message(), "simple ring") != NULL);

  const csgrs_vec3_f64_t points[] = {{3,1,0},{1,1,0},{1,3,0},{0,3,0},{0,0,0},{3,0,0}};
  const size_t concave[] = {0,1,2,3,4,5}, ends[] = {0,6};
  assert(csgrs_triangle_mesh_f64_polyhedron(points,6,concave,6,ends,2,&mesh) == CSGRS_STATUS_OK);
  assert(strcmp(csgrs_last_error_message(), "") == 0);
  csgrs_triangle_mesh_buffers_f64_t buffers;
  assert(csgrs_triangle_mesh_f64_vertices_and_indices(mesh,&buffers) == CSGRS_STATUS_OK);
  double twice_area = 0;
  for (size_t i=0; i<buffers.index_len; ++i) {
    const csgrs_vec3_f64_t a=buffers.vertices[buffers.indices[i].a];
    const csgrs_vec3_f64_t b=buffers.vertices[buffers.indices[i].b];
    const csgrs_vec3_f64_t c=buffers.vertices[buffers.indices[i].c];
    twice_area += (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
  }
  assert(twice_area == 10);
  csgrs_triangle_mesh_buffers_f64_free(buffers);
  csgrs_triangle_mesh_free(mesh);
  puts("C: invalid face stays typed; recovered concave face has signed area 5");
}
