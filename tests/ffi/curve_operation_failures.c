#include "csgrs.h"
#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  csgrs_curve_region_t *square = NULL, *rotated = NULL, *restored = NULL;
  assert(csgrs_curve_region_f64_square(2, &square) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_rotate(square, 0, 0, 13, &rotated) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_rotate(rotated, 0, 0, -13, &restored) == CSGRS_STATUS_OK);
  typedef csgrs_status_t (*operation)(const csgrs_curve_region_t *, const csgrs_curve_region_t *, csgrs_curve_region_t **);
  const operation operations[] = {csgrs_curve_region_f64_union, csgrs_curve_region_f64_difference,
    csgrs_curve_region_f64_intersection, csgrs_curve_region_f64_xor};
  for (size_t i = 0; i < sizeof(operations) / sizeof(operations[0]); ++i) {
    csgrs_curve_region_t *output = NULL;
    assert(operations[i](square, restored, &output) == CSGRS_STATUS_PREDICATE_UNDECIDED);
    assert(output == NULL);
    assert(strstr(csgrs_last_error_message(), "blocked by Ordering") != NULL);
  }
  csgrs_triangle_mesh_t *mesh = NULL;
  assert(csgrs_curve_region_f64_revolve(square, 360, 1, &mesh) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(mesh == NULL && strcmp(csgrs_last_error_message(), "segments must not be less than 2") == 0);

  csgrs_curve_region_t *large = NULL, *scaled = NULL;
  assert(csgrs_curve_region_f64_square(DBL_MAX, &large) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_scale(large, 2, 2, 1, &scaled) == CSGRS_STATUS_OK);
  csgrs_region_profiles_f64_t profiles = {NULL, 17, NULL, 19, NULL, 23};
  assert(csgrs_curve_region_f64_region_profiles(scaled, &profiles) == CSGRS_STATUS_VALIDATION_FAILED);
  assert(profiles.points == NULL && profiles.point_len == 17);
  assert(profiles.ring_offsets == NULL && profiles.ring_offset_len == 19);
  assert(profiles.region_offsets == NULL && profiles.region_offset_len == 23);
  assert(strstr(csgrs_last_error_message(), "finite projection point is not finite") != NULL);

  csgrs_curve_region_t *empty = NULL, *recovered = NULL;
  assert(csgrs_curve_region_f64_difference(square, square, &empty) == CSGRS_STATUS_OK);
  csgrs_aabb3_f64_t bounds;
  assert(csgrs_curve_region_f64_bounding_box(empty, &bounds) == CSGRS_STATUS_NO_BOUNDS);
  assert(csgrs_curve_region_f64_union(empty, square, &recovered) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_bounding_box(recovered, &bounds) == CSGRS_STATUS_OK);
  assert(bounds.mins.x == 0 && bounds.mins.y == 0 && bounds.maxs.x == 2 && bounds.maxs.y == 2);
  assert(csgrs_last_error_message()[0] == '\0');
  csgrs_curve_region_free(recovered);
  csgrs_curve_region_free(empty);
  csgrs_curve_region_free(scaled);
  csgrs_curve_region_free(large);
  csgrs_curve_region_free(restored);
  csgrs_curve_region_free(rotated);
  csgrs_curve_region_free(square);
  puts("C curve failures preserve categories, diagnostics and output ownership; recovery passed");
}
