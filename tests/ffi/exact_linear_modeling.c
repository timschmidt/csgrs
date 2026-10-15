#include "csgrs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  const csgrs_i128_t width = {0, UINT64_C(9007199254740993)};
  const csgrs_i128_t zero = {0, 0}, one = {0, 1}, two = {0, 2};
  csgrs_curve_region_t *region = NULL;
  csgrs_triangle_mesh_t *mesh = NULL;
  assert(csgrs_curve_region_i128_rectangle(width, one, &region) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_i128_extrude(region, zero, &mesh) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(mesh == NULL && strstr(csgrs_last_error_message(), "nonzero") != NULL);
  assert(csgrs_curve_region_i128_extrude(region, two, &mesh) == CSGRS_STATUS_OK);
  csgrs_aabb3_i128_t bounds;
  assert(csgrs_triangle_mesh_i128_bounding_box(mesh, &bounds) == CSGRS_STATUS_OK);
  assert(bounds.mins.x.hi == 0 && bounds.mins.x.lo == 0);
  assert(bounds.maxs.x.hi == 0 && bounds.maxs.x.lo == width.lo);
  assert(bounds.maxs.y.lo == 1 && bounds.maxs.z.lo == 2);
  csgrs_triangle_mesh_free(mesh);
  csgrs_curve_region_free(region);
  puts("C exact linear extrusion and failure recovery passed");
}
