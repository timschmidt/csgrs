#include "csgrs.hpp"
#include <cassert>
#include <iostream>
#include <string>

int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  try {
    const auto mesh = Mesh::polyhedron({{0,0,0},{3,3,0},{0,2,0},{2,0,0}}, {0,1,2,3}, {0,4});
    (void)mesh;
    assert(false && "crossed face returned geometry");
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_VALIDATION_FAILED);
    assert(std::string(error.what()).find("simple ring") != std::string::npos);
  }
  const auto face = Mesh::polyhedron({{3,1,0},{1,1,0},{1,3,0},{0,3,0},{0,0,0},{3,0,0}}, {0,1,2,3,4,5}, {0,6});
  const auto buffers = face.vertices_and_indices();
  double twice_area = 0;
  for (std::size_t i=0; i<buffers.index_len; ++i) {
    const auto a = buffers.vertices[buffers.indices[i].a];
    const auto b = buffers.vertices[buffers.indices[i].b];
    const auto c = buffers.vertices[buffers.indices[i].c];
    twice_area += (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
  }
  assert(twice_area == 10);
  csgrs::F64::mesh_buffers_free(buffers);
  std::cout << "C++: typed face failure and exact concave orientation passed\n";
}
