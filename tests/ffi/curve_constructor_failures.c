#include "csgrs.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  csgrs_curve_region_t *region = NULL;
  assert(csgrs_curve_region_f64_square(-1, &region) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(region == NULL && strstr(csgrs_last_error_message(), "width") != NULL);
  assert(csgrs_curve_region_f64_rectangle(0, -1, &region) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(region == NULL);
  assert(csgrs_curve_region_f64_circle(0, 2, &region) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(region == NULL);
  assert(csgrs_curve_region_f64_circle(1, SIZE_MAX, &region) == CSGRS_STATUS_RESOURCE_EXHAUSTED);
  assert(region == NULL && strstr(csgrs_last_error_message(), "capacity") != NULL);
  assert(csgrs_curve_region_f64_polygon(NULL, 0, &region) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(region == NULL && strstr(csgrs_last_error_message(), "vertices") != NULL);
  assert(csgrs_curve_region_f64_square(0, &region) == CSGRS_STATUS_OK);
  csgrs_aabb3_f64_t box;
  assert(csgrs_curve_region_f64_bounding_box(region, &box) == CSGRS_STATUS_NO_BOUNDS);
  csgrs_curve_region_t *unit = NULL, *restored = NULL;
  assert(csgrs_curve_region_f64_square(1, &unit) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_union(region, unit, &restored) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_bounding_box(restored, &box) == CSGRS_STATUS_OK);
  assert(box.mins.x == 0 && box.maxs.x == 1 && box.maxs.y == 1);
  csgrs_curve_region_free(restored);
  csgrs_curve_region_free(unit);
  csgrs_curve_region_free(region);
  puts("C planar constructor failures, empty composition, and recovery passed");
}
