#include "csgrs.hpp"
#include <cassert>
#include <iostream>

int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  using Curve = csgrs::CurveRegion<csgrs::F64>;
  const auto cube = Mesh::cube(1);
  const auto empty = cube.difference(cube).center().floating();
  assert(!empty.bounding_box().has_value());
  const auto restored = empty.union_with(cube);
  const auto bounds = restored.bounding_box().value();
  assert(bounds.mins.x == 0 && bounds.maxs.x == 1);
  const auto square = Curve::square(1);
  assert(!square.difference(square).bounding_box().has_value());
  const auto centered = cube.center().floating().bounding_box().value();
  assert(centered.mins.x == -0.5 && centered.maxs.x == 0.5);
  assert(centered.mins.z == 0 && centered.maxs.z == 1);
  std::cout << "C++ consumer: optional bounds and exact positioning passed\n";
}
