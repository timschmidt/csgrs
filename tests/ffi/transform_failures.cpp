#include "csgrs.hpp"
#include <cassert>
#include <iostream>
#include <string>

int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  const auto cube = Mesh::cube(1);
  try {
    const auto invalid = cube.scale(0, 1, 1);
    (void)invalid;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_VALIDATION_FAILED);
  }
  const auto shifted = cube.translate(-0.5, 0, 0);
  const csgrs_matrix4_f64_t reciprocal_x = {{0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0}};
  try {
    const auto unbounded = shifted.transform(reciprocal_x);
    (void)unbounded;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_UNBOUNDED);
    assert(std::string(error.what()).find("crosses infinity") != std::string::npos);
  }
  const auto restored = shifted.translate(0.5, 0, 0);
  assert(restored.bounding_box().maxs.x == 1);
  std::cout << "C++ typed transform errors and recovery passed\n";
}
