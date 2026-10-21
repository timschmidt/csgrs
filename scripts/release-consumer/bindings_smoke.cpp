// C++ consumer of the packaged csgrs-ffi library through bindings/cpp:
// drill a 12-unit cube with a 4 x 4 bar and check its exact bounds and
// closed edge pairing.
#include "csgrs.hpp"

#include <cstdio>
#include <set>
#include <utility>

int main() {
  using Mesh = csgrs::TriangleMeshF64;
  const Mesh part = Mesh::cube(12.0).difference(
      Mesh::cuboid(4.0, 4.0, 16.0).translate(4.0, 4.0, -2.0));

  const auto bounds = part.bounding_box();
  if (!bounds || bounds->mins.x != 0.0 || bounds->mins.y != 0.0 || bounds->mins.z != 0.0 ||
      bounds->maxs.x != 12.0 || bounds->maxs.y != 12.0 || bounds->maxs.z != 12.0) {
    std::fprintf(stderr, "unexpected bounds\n");
    return 1;
  }

  auto buffers = part.vertices_and_indices();
  std::set<std::pair<uint32_t, uint32_t>> directed;
  for (size_t t = 0; t < buffers.index_len; ++t) {
    const auto &tri = buffers.indices[t];
    directed.insert({tri.a, tri.b});
    directed.insert({tri.b, tri.c});
    directed.insert({tri.c, tri.a});
  }
  size_t unpaired = 0;
  for (const auto &[a, b] : directed) unpaired += directed.count({b, a}) == 0;
  const size_t triangles = buffers.index_len;
  csgrs::F64::mesh_buffers_free(buffers);
  if (triangles == 0 || unpaired != 0) {
    std::fprintf(stderr, "%zu triangles, %zu unpaired edges\n", triangles, unpaired);
    return 1;
  }
  std::printf("c++: %zu triangles, closed, exact bounds\n", triangles);
  return 0;
}
