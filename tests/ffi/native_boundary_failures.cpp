#include "csgrs.hpp"
#include <cassert>
#include <string>

int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  try {
    Mesh::polyhedron({{0,0,0}, {3,3,0}, {0,2,0}, {2,0,0}}, {0,1,2,3}, {0,4});
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_VALIDATION_FAILED);
    assert(std::string(error.what()).find("simple ring edges cross") != std::string::npos);
  }
  const auto mesh = Mesh::polyhedron({{0,0,0}, {1,0,0}, {0,3,4}}, {0,1,2}, {0,3});
  for (int attempt = 0; attempt < 2; ++attempt) {
    auto graphics = mesh.graphics_mesh();
    assert(graphics.vertex_len == 3 && graphics.index_len == 3);
    for (size_t i = 0; i < 3; ++i) {
      assert(graphics.vertices[i].normal.x == 0);
      assert(graphics.vertices[i].normal.y == -0.8);
      assert(graphics.vertices[i].normal.z == 0.6);
    }
    csgrs_graphics_mesh_f64_free(graphics);
  }
  auto empty = Mesh::cube(0).graphics_mesh();
  assert(empty.vertex_len == 0 && empty.index_len == 0);
  csgrs_graphics_mesh_f64_free(empty);
}
