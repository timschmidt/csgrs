#include "csgrs.hpp"
#include <cassert>
#include <iostream>
int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  const auto cube = Mesh::cube(1);
  const auto moved = cube.translate(3, 5, 7);
  const auto removed = moved.difference(moved);
  for (const auto *mesh : {&cube, &moved, &removed}) {
    const auto evidence = mesh->construction_evidence();
    assert(!evidence.source_history_missing && !evidence.approximate_512_consumed);
  }
  const auto sphere = Mesh::sphere(1, 8, 4);
  assert(sphere.inverse().construction_evidence().source_sampled);
  std::cout << "C++ native construction evidence passed\n";
}
