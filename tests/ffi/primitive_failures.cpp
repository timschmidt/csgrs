#include "csgrs.hpp"
#include <cassert>
#include <limits>
#include <string>

int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  try {
    const auto mesh = Mesh::cube(-1);
    (void)mesh;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_VALIDATION_FAILED);
    assert(std::string(error.what()).find("width") != std::string::npos);
  }
  try {
    const auto mesh = Mesh::sphere(1, std::numeric_limits<size_t>::max(), 2);
    (void)mesh;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_RESOURCE_EXHAUSTED);
  }
  const auto restored = Mesh::cube(0).union_with(Mesh::cube(1));
  assert(restored.bounding_box().maxs.x == 1);
}
