/*
 * Real C consumer of the csgrs-ffi ABI, used for link-retention size
 * attribution. It builds a drilled body through the f64 family, reads exact
 * bounds and indexed buffers, inspects construction evidence, and frees every
 * handle. Linking this against the static library measures what one ordinary
 * C consumer retains, which is a different quantity from the complete exported
 * surface of the shared library.
 */

#include <stdio.h>
#include <stdlib.h>

#include "csgrs.h"

static void fail(const char *stage, csgrs_status_t status) {
  fprintf(stderr, "%s failed: status=%d message=%s\n", stage, (int)status,
          csgrs_last_error_message());
  exit(1);
}

int main(int argc, char **argv) {
  double width = argc > 1 ? atof(argv[1]) : 12.0;
  long repeats = argc > 2 ? atol(argv[2]) : 1;
  if (repeats < 1) {
    repeats = 1;
  }

  size_t triangles = 0;
  size_t vertices = 0;
  double height = 0.0;
  for (long iteration = 0; iteration < repeats; iteration += 1) {
    csgrs_triangle_mesh_t *body = NULL;
    csgrs_status_t status = csgrs_triangle_mesh_f64_cube(width, &body);
    if (status != CSGRS_STATUS_OK) {
      fail("cube", status);
    }

    csgrs_curve_region_t *profile = NULL;
    status = csgrs_curve_region_f64_square(4.0, &profile);
    if (status != CSGRS_STATUS_OK) {
      fail("square", status);
    }

    csgrs_triangle_mesh_t *tool = NULL;
    status = csgrs_curve_region_f64_extrude(profile, 16.0, &tool);
    if (status != CSGRS_STATUS_OK) {
      fail("extrude", status);
    }

    csgrs_triangle_mesh_t *placed = NULL;
    status = csgrs_triangle_mesh_f64_translate(tool, 4.0, 4.0, -2.0, &placed);
    if (status != CSGRS_STATUS_OK) {
      fail("translate", status);
    }

    csgrs_triangle_mesh_t *part = NULL;
    status = csgrs_triangle_mesh_f64_difference(body, placed, &part);
    if (status != CSGRS_STATUS_OK) {
      fail("difference", status);
    }

    csgrs_aabb3_f64_t bounds;
    status = csgrs_triangle_mesh_f64_bounding_box(part, &bounds);
    if (status != CSGRS_STATUS_OK) {
      fail("bounding_box", status);
    }
    height = bounds.maxs.z - bounds.mins.z;

    csgrs_construction_evidence_t evidence;
    status = csgrs_triangle_mesh_construction_evidence(part, &evidence);
    if (status != CSGRS_STATUS_OK) {
      fail("construction_evidence", status);
    }

    csgrs_triangle_mesh_buffers_f64_t buffers;
    status = csgrs_triangle_mesh_f64_vertices_and_indices(part, &buffers);
    if (status != CSGRS_STATUS_OK) {
      fail("vertices_and_indices", status);
    }
    triangles = buffers.index_len;
    vertices = buffers.vertex_len;
    csgrs_triangle_mesh_buffers_f64_free(buffers);

    csgrs_triangle_mesh_free(part);
    csgrs_triangle_mesh_free(placed);
    csgrs_triangle_mesh_free(tool);
    csgrs_curve_region_free(profile);
    csgrs_triangle_mesh_free(body);
  }

  printf("triangles=%zu vertices=%zu height=%.3f repeats=%ld\n", triangles,
         vertices, height, repeats);
  return 0;
}
