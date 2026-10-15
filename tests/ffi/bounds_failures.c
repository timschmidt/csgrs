/* Exercise absent bounds through the public ABI, including unchanged outputs. */
#include "csgrs.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
  csgrs_triangle_mesh_t *cube = NULL, *empty = NULL, *placed = NULL, *restored = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_difference(cube, cube, &empty) == CSGRS_STATUS_OK);
  csgrs_aabb3_f64_t box = {{17, 18, 19}, {20, 21, 22}};
  assert(csgrs_triangle_mesh_f64_bounding_box(empty, &box) == CSGRS_STATUS_NO_BOUNDS);
  assert(box.mins.x == 17 && box.mins.y == 18 && box.mins.z == 19);
  assert(box.maxs.x == 20 && box.maxs.y == 21 && box.maxs.z == 22);
  assert(csgrs_triangle_mesh_f64_bounding_box(empty, NULL) == CSGRS_STATUS_NULL_POINTER);
  assert(csgrs_triangle_mesh_f64_center(empty, &placed) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_union(placed, cube, &restored) == CSGRS_STATUS_OK);
  assert(csgrs_triangle_mesh_f64_bounding_box(restored, &box) == CSGRS_STATUS_OK);
  assert(box.mins.x == 0 && box.mins.y == 0 && box.mins.z == 0);
  assert(box.maxs.x == 1 && box.maxs.y == 1 && box.maxs.z == 1);
  csgrs_curve_region_t *square = NULL, *empty_region = NULL;
  assert(csgrs_curve_region_f64_square(1, &square) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_difference(square, square, &empty_region) == CSGRS_STATUS_OK);
  assert(csgrs_curve_region_f64_bounding_box(empty_region, &box) == CSGRS_STATUS_NO_BOUNDS);
  assert(box.mins.x == 0 && box.maxs.x == 1);
  csgrs_curve_region_free(empty_region);
  csgrs_curve_region_free(square);
  csgrs_triangle_mesh_free(restored);
  csgrs_triangle_mesh_free(placed);
  csgrs_triangle_mesh_free(empty);
  csgrs_triangle_mesh_free(cube);
  puts("C consumer: absent bounds, unchanged outputs, and empty positioning passed");
}
