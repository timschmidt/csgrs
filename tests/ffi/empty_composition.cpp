#include "csgrs.hpp"
#include <cassert>
#include <iostream>
#include <string>

int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  const auto cube = Mesh::cube(1);
  const auto empty = cube.difference(cube);
  const auto buffers = empty.vertices_and_indices();
  assert(buffers.vertex_len == 0 && buffers.index_len == 0);
  csgrs::F64::mesh_buffers_free(buffers);
  const auto restored = empty.union_with(cube);
  const auto bounds = restored.bounding_box().value();
  assert(bounds.mins.x == 0 && bounds.mins.y == 0 && bounds.mins.z == 0);
  assert(bounds.maxs.x == 1 && bounds.maxs.y == 1 && bounds.maxs.z == 1);
  const auto removed = restored.difference(cube).difference(cube);
  const auto removed_buffers = removed.vertices_and_indices();
  assert(removed_buffers.vertex_len == 0 && removed_buffers.index_len == 0);
  csgrs::F64::mesh_buffers_free(removed_buffers);
  const auto open = Mesh::polyhedron({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {0, 1, 2}, {0, 3});
  try {
    const auto invalid = empty.intersection(open);
    (void)invalid;
    assert(false && "a malformed operand cannot produce successful geometry");
  } catch (const csgrs::error &error) {
    assert(std::string(error.what()) == "input mesh 1 has 3 boundary edges");
  }
  std::cout << "C++ consumer: empty composition and visible failures passed\n";
}
