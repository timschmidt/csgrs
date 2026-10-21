/* A C consumer of the packaged csgrs-ffi static library and its shipped
 * header: drill a 12-unit cube with a 4 x 4 bar, then check the result's
 * exact bounds and closed edge pairing from the exported buffers. */
#include "csgrs.h"

#include <stdio.h>

static int fail(const char *step, csgrs_status_t status) {
  const char *message = csgrs_last_error_message();
  fprintf(stderr, "%s failed with status %d: %s\n", step, (int)status,
          message ? message : "(no message)");
  return 1;
}

int main(void) {
  csgrs_triangle_mesh_t *body = NULL, *bar = NULL, *drill = NULL, *part = NULL;
  csgrs_status_t status;
  if ((status = csgrs_triangle_mesh_f64_cube(12.0, &body)) != CSGRS_STATUS_OK)
    return fail("cube", status);
  if ((status = csgrs_triangle_mesh_f64_cuboid(4.0, 4.0, 16.0, &bar)) != CSGRS_STATUS_OK)
    return fail("cuboid", status);
  if ((status = csgrs_triangle_mesh_f64_translate(bar, 4.0, 4.0, -2.0, &drill)) !=
      CSGRS_STATUS_OK)
    return fail("translate", status);
  if ((status = csgrs_triangle_mesh_f64_difference(body, drill, &part)) != CSGRS_STATUS_OK)
    return fail("difference", status);

  csgrs_aabb3_f64_t bounds;
  if ((status = csgrs_triangle_mesh_f64_bounding_box(part, &bounds)) != CSGRS_STATUS_OK)
    return fail("bounding box", status);
  if (bounds.mins.x != 0.0 || bounds.mins.y != 0.0 || bounds.mins.z != 0.0 ||
      bounds.maxs.x != 12.0 || bounds.maxs.y != 12.0 || bounds.maxs.z != 12.0) {
    fprintf(stderr, "unexpected bounds\n");
    return 1;
  }

  csgrs_triangle_mesh_buffers_f64_t buffers;
  if ((status = csgrs_triangle_mesh_f64_vertices_and_indices(part, &buffers)) !=
      CSGRS_STATUS_OK)
    return fail("vertices and indices", status);
  /* A closed, consistently oriented surface uses every directed edge once
   * and its reverse once: count directed edges whose reverse is missing. */
  size_t unpaired = 0;
  for (size_t t = 0; t < buffers.index_len; ++t) {
    uint32_t edge[3][2] = {{buffers.indices[t].a, buffers.indices[t].b},
                           {buffers.indices[t].b, buffers.indices[t].c},
                           {buffers.indices[t].c, buffers.indices[t].a}};
    for (int e = 0; e < 3; ++e) {
      int found = 0;
      for (size_t u = 0; u < buffers.index_len && !found; ++u) {
        uint32_t v[3] = {buffers.indices[u].a, buffers.indices[u].b, buffers.indices[u].c};
        for (int f = 0; f < 3; ++f)
          if (v[f] == edge[e][1] && v[(f + 1) % 3] == edge[e][0]) found = 1;
      }
      unpaired += !found;
    }
  }
  size_t triangles = buffers.index_len;
  csgrs_triangle_mesh_buffers_f64_free(buffers);
  csgrs_triangle_mesh_free(part);
  csgrs_triangle_mesh_free(drill);
  csgrs_triangle_mesh_free(bar);
  csgrs_triangle_mesh_free(body);
  if (triangles == 0 || unpaired != 0) {
    fprintf(stderr, "%zu triangles, %zu unpaired edges\n", triangles, unpaired);
    return 1;
  }
  printf("ffi: %zu triangles, closed, exact bounds\n", triangles);
  return 0;
}
