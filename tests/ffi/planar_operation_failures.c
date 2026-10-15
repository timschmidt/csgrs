#include "csgrs.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  csgrs_curve_region_t *square = NULL, *result = NULL, *empty = NULL;
  assert(csgrs_curve_region_f64_square(1, &square) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_square(0, &empty) == CSGRS_STATUS_OK);
  for (int i = 0; i < 2; ++i) {
    const csgrs_curve_region_t *input = i ? empty : square;
    assert(csgrs_curve_region_f64_scale(input, 0, 1, 1, &result) == CSGRS_STATUS_VALIDATION_FAILED);
    assert(result == NULL && strlen(csgrs_last_error_message()) > 0);
    assert(csgrs_curve_region_f64_translate(input, 0, 0, 1, &result) == CSGRS_STATUS_UNSUPPORTED);
    assert(result == NULL && strstr(csgrs_last_error_message(), "XY plane") != NULL);
    assert(csgrs_curve_region_f64_rotate(input, 90, 0, 0, &result) == CSGRS_STATUS_UNSUPPORTED);
    assert(result == NULL);
    const csgrs_matrix4_f64_t perspective = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1}};
    assert(csgrs_curve_region_f64_transform(input, perspective, &result) == CSGRS_STATUS_UNSUPPORTED);
    assert(result == NULL);
    const csgrs_matrix4_f64_t infinite = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}};
    assert(csgrs_curve_region_f64_transform(input, infinite, &result) == CSGRS_STATUS_UNBOUNDED);
    assert(result == NULL);
  }
  assert(csgrs_curve_region_f64_translate(square, 0, 0, NAN, &result) == CSGRS_STATUS_CONVERSION_FAILED);
  assert(csgrs_curve_region_f64_scale(square, 1, 1, INFINITY, &result) == CSGRS_STATUS_CONVERSION_FAILED);
  assert(result == NULL);
  const csgrs_matrix4_f64_t affine = {{4, 0, 17, 2, 0, -6, 19, 4, 0, 0, 0, 0, 0, 0, 23, 2}};
  assert(csgrs_curve_region_f64_transform(square, affine, &result) == CSGRS_STATUS_OK);
  csgrs_aabb3_f64_t bounds;
  assert(csgrs_curve_region_f64_bounding_box(result, &bounds) == CSGRS_STATUS_OK);
  assert(bounds.mins.x == 1 && bounds.maxs.x == 3 && bounds.mins.y == -1 && bounds.maxs.y == 2);
  csgrs_curve_region_free(result);
  result = NULL;
  assert(csgrs_curve_region_f64_rotate(square, 180, 0, 0, &result) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_bounding_box(result, &bounds) == CSGRS_STATUS_OK);
  assert(bounds.mins.x == 0 && bounds.maxs.x == 1 && bounds.mins.y == -1 && bounds.maxs.y == 0);
  csgrs_curve_region_free(result);
  csgrs_curve_region_free(square);
  csgrs_curve_region_free(empty);
  puts("C planar matrix meaning, typed errors, empty validation, and recovery passed");
}
