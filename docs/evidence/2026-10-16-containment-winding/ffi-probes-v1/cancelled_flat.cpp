
#include "csgrs.hpp"
#include <cassert>
#include <iostream>
using Mesh = csgrs::TriangleMesh<csgrs::F64>;
static void check(const Mesh &mesh, bool empty) {
  const auto b=mesh.vertices_and_indices();
  if (empty) { assert(b.vertex_len==0 && b.index_len==0); }
  else {
    double volume=0;
    for (size_t i=0;i<b.index_len;++i) {
      const auto t=b.indices[i]; const auto a=b.vertices[t.a],c=b.vertices[t.b],d=b.vertices[t.c];
      volume+=a.x*(c.y*d.z-c.z*d.y)+a.y*(c.z*d.x-c.x*d.z)+a.z*(c.x*d.y-c.y*d.x);
    }
    assert(volume==6);
  }
  csgrs::F64::mesh_buffers_free(b);
}
int main() {
  const auto cube=Mesh::cube(1);
  const auto flat=Mesh::polyhedron({{-2,-2,0.5},{4,-2,0.5},{-2,4,0.5}},{0,1,2,2,1,0},{0,3,6});
  check(flat.union_with(cube),false);check(cube.difference(flat),false);
  check(flat.intersection(cube),true);check(flat.difference(cube),true);
  check(flat.union_with(cube).difference(cube),true);
  std::cout << "C++ cancelled-sheet composition passed\n";
}
