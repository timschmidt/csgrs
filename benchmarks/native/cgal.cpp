#include "common.hpp"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/IO/STL.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/copy_face_graph.h>

#include <array>
#include <bit>
#include <cmath>
#include <numbers>
#include <map>
#include <sstream>
#include <type_traits>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point>;
using Triangle = std::array<std::size_t, 3>;
using csgrs_bench::Measurement;

static_assert(!std::is_floating_point_v<Kernel::FT>,
              "the high-precision runner requires CGAL exact constructions");

static Mesh mesh_from_soup(std::vector<Point> points,
                           std::vector<Triangle> triangles) {
  PMP::orient_polygon_soup(points, triangles);
  Mesh mesh;
  PMP::polygon_soup_to_polygon_mesh(points, triangles, mesh);
  return mesh;
}

static Mesh exact_box(const Kernel::FT &width, const Kernel::FT &cx,
                      const Kernel::FT &cy, const Kernel::FT &cz) {
  const Kernel::FT h = width / Kernel::FT(2);
  std::vector<Point> points;
  for (const Kernel::FT &z : std::array<Kernel::FT, 2>{-h, h}) {
    for (const Kernel::FT &y : std::array<Kernel::FT, 2>{-h, h}) {
      for (const Kernel::FT &x : std::array<Kernel::FT, 2>{-h, h}) {
        points.emplace_back(cx + x, cy + y, cz + z);
      }
    }
  }
  return mesh_from_soup(
      std::move(points),
      {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5},
       {0, 5, 4}, {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2},
       {1, 3, 7}, {1, 7, 5}});
}

static Mesh exact_cuboid(const Kernel::FT &width, const Kernel::FT &length,
                         const Kernel::FT &height) {
  const Kernel::FT hx = width / Kernel::FT(2);
  const Kernel::FT hy = length / Kernel::FT(2);
  const Kernel::FT hz = height / Kernel::FT(2);
  std::vector<Point> points;
  for (const Kernel::FT &z : std::array<Kernel::FT, 2>{-hz, hz}) {
    for (const Kernel::FT &y : std::array<Kernel::FT, 2>{-hy, hy}) {
      for (const Kernel::FT &x : std::array<Kernel::FT, 2>{-hx, hx}) {
        points.emplace_back(x, y, z);
      }
    }
  }
  return mesh_from_soup(
      std::move(points),
      {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5},
       {0, 5, 4}, {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2},
       {1, 3, 7}, {1, 7, 5}});
}

static Mesh box(double width, double cx = 0.0, double cy = 0.0,
                double cz = 0.0) {
  return exact_box(Kernel::FT(width), Kernel::FT(cx), Kernel::FT(cy),
                   Kernel::FT(cz));
}

static Mesh sphere(double radius, std::size_t segments, std::size_t stacks) {
  std::vector<Point> points;
  points.emplace_back(0.0, 0.0, radius);
  for (std::size_t stack = 1; stack < stacks; ++stack) {
    const double theta = std::numbers::pi * static_cast<double>(stack) /
                         static_cast<double>(stacks);
    const double ring_radius = radius * std::sin(theta);
    const double z = radius * std::cos(theta);
    for (std::size_t segment = 0; segment < segments; ++segment) {
      const double phi = 2.0 * std::numbers::pi *
                         static_cast<double>(segment) /
                         static_cast<double>(segments);
      points.emplace_back(ring_radius * std::cos(phi),
                          ring_radius * std::sin(phi), z);
    }
  }
  const std::size_t south = points.size();
  points.emplace_back(0.0, 0.0, -radius);

  std::vector<Triangle> triangles;
  triangles.reserve(2 * segments * (stacks - 1));
  for (std::size_t segment = 0; segment < segments; ++segment) {
    const std::size_t next = (segment + 1) % segments;
    triangles.push_back({0, 1 + segment, 1 + next});
  }
  for (std::size_t stack = 0; stack + 2 < stacks; ++stack) {
    const std::size_t first = 1 + stack * segments;
    const std::size_t next_ring = first + segments;
    for (std::size_t segment = 0; segment < segments; ++segment) {
      const std::size_t next = (segment + 1) % segments;
      triangles.push_back(
          {first + segment, next_ring + segment, next_ring + next});
      triangles.push_back({first + segment, next_ring + next, first + next});
    }
  }
  const std::size_t last_ring = south - segments;
  for (std::size_t segment = 0; segment < segments; ++segment) {
    const std::size_t next = (segment + 1) % segments;
    triangles.push_back({last_ring + segment, south, last_ring + next});
  }
  return mesh_from_soup(std::move(points), std::move(triangles));
}

static Mesh extruded_circle(double radius, double height,
                            std::size_t segments) {
  std::vector<Point> points;
  points.reserve(2 * segments);
  for (const double z : {0.0, height}) {
    for (std::size_t segment = 0; segment < segments; ++segment) {
      const double angle = 2.0 * std::numbers::pi *
                           static_cast<double>(segment) /
                           static_cast<double>(segments);
      points.emplace_back(radius * std::cos(angle), radius * std::sin(angle), z);
    }
  }
  std::vector<Triangle> triangles;
  triangles.reserve(4 * segments - 4);
  for (std::size_t segment = 1; segment + 1 < segments; ++segment) {
    triangles.push_back({0, segment + 1, segment});
    triangles.push_back(
        {segments, segments + segment, segments + segment + 1});
  }
  for (std::size_t segment = 0; segment < segments; ++segment) {
    const std::size_t next = (segment + 1) % segments;
    triangles.push_back({segment, next, segments + next});
    triangles.push_back({segment, segments + next, segments + segment});
  }
  return mesh_from_soup(std::move(points), std::move(triangles));
}

static Mesh frustum(double lower_radius, double upper_radius, double height,
                    std::size_t segments) {
  std::vector<Point> points;
  points.reserve(2 * segments + 2);
  for (std::size_t segment = 0; segment < segments; ++segment) {
    const double angle = 2.0 * std::numbers::pi *
                         static_cast<double>(segment) /
                         static_cast<double>(segments);
    points.emplace_back(lower_radius * std::cos(angle),
                        lower_radius * std::sin(angle), 0.0);
  }
  for (std::size_t segment = 0; segment < segments; ++segment) {
    const double angle = 2.0 * std::numbers::pi *
                         static_cast<double>(segment) /
                         static_cast<double>(segments);
    points.emplace_back(upper_radius * std::cos(angle),
                        upper_radius * std::sin(angle), height);
  }
  const std::size_t lower_center = points.size();
  points.emplace_back(0.0, 0.0, 0.0);
  const std::size_t upper_center = points.size();
  points.emplace_back(0.0, 0.0, height);
  std::vector<Triangle> triangles;
  triangles.reserve(4 * segments);
  for (std::size_t segment = 0; segment < segments; ++segment) {
    const std::size_t next = (segment + 1) % segments;
    triangles.push_back({lower_center, next, segment});
    triangles.push_back({upper_center, segments + segment, segments + next});
    triangles.push_back({segment, next, segments + next});
    triangles.push_back({segment, segments + next, segments + segment});
  }
  return mesh_from_soup(std::move(points), std::move(triangles));
}

static Mesh torus(double major_radius, double minor_radius,
                  std::size_t major_segments, std::size_t minor_segments) {
  std::vector<Point> points;
  points.reserve(major_segments * minor_segments);
  for (std::size_t major = 0; major < major_segments; ++major) {
    const double u = 2.0 * std::numbers::pi * static_cast<double>(major) /
                     static_cast<double>(major_segments);
    for (std::size_t minor = 0; minor < minor_segments; ++minor) {
      const double v = 2.0 * std::numbers::pi * static_cast<double>(minor) /
                       static_cast<double>(minor_segments);
      const double radial = major_radius + minor_radius * std::cos(v);
      points.emplace_back(radial * std::cos(u), radial * std::sin(u),
                          minor_radius * std::sin(v));
    }
  }
  std::vector<Triangle> triangles;
  triangles.reserve(2 * major_segments * minor_segments);
  const auto index = [minor_segments](std::size_t major, std::size_t minor) {
    return major * minor_segments + minor;
  };
  for (std::size_t major = 0; major < major_segments; ++major) {
    const std::size_t next_major = (major + 1) % major_segments;
    for (std::size_t minor = 0; minor < minor_segments; ++minor) {
      const std::size_t next_minor = (minor + 1) % minor_segments;
      triangles.push_back({index(major, minor), index(next_major, minor),
                           index(next_major, next_minor)});
      triangles.push_back({index(major, minor), index(next_major, next_minor),
                           index(major, next_minor)});
    }
  }
  return mesh_from_soup(std::move(points), std::move(triangles));
}

static Mesh octahedron(double radius) {
  return mesh_from_soup(
      {{radius, 0, 0}, {-radius, 0, 0}, {0, radius, 0}, {0, -radius, 0},
       {0, 0, radius}, {0, 0, -radius}},
      {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4},
       {5, 2, 0}, {5, 1, 2}, {5, 3, 1}, {5, 0, 3}});
}

static Mesh icosahedron(double radius) {
  const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
  const double scale = radius / std::sqrt(1.0 + phi * phi);
  const double a = scale;
  const double b = phi * scale;
  return mesh_from_soup(
      {{-a, b, 0}, {a, b, 0}, {-a, -b, 0}, {a, -b, 0}, {0, -a, b},
       {0, a, b}, {0, -a, -b}, {0, a, -b}, {b, 0, -a}, {b, 0, a},
       {-b, 0, -a}, {-b, 0, a}},
      {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
       {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
       {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
       {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}});
}

static Measurement measured(const Mesh &mesh, std::size_t input_facets) {
  const std::size_t facets = mesh.number_of_faces();
  std::size_t corners = 0;
  for (const auto face : mesh.faces()) {
    for ([[maybe_unused]] const auto vertex :
         CGAL::vertices_around_face(mesh.halfedge(face), mesh)) {
      ++corners;
    }
  }
  return {input_facets, facets, csgrs_bench::checksum(facets, corners)};
}

static Measurement geometry_measured(const Mesh &mesh,
                                     std::size_t input_facets) {
  const std::size_t facets = mesh.number_of_faces();
  std::size_t corners = 0;
  std::uint64_t fingerprint = facets;
  for (const auto face : mesh.faces()) {
    for (const auto vertex :
         CGAL::vertices_around_face(mesh.halfedge(face), mesh)) {
      ++corners;
      const Point &point = mesh.point(vertex);
      for (const Kernel::FT &coordinate :
           {point.x(), point.y(), point.z()}) {
        fingerprint = std::rotl(fingerprint, 7) ^
                      std::bit_cast<std::uint64_t>(CGAL::to_double(coordinate));
      }
    }
  }
  return {input_facets, facets, fingerprint ^ corners};
}

template <typename PointTransform>
static Mesh transformed(const Mesh &source, PointTransform transform) {
  Mesh output = source;
  for (const auto vertex : output.vertices()) {
    output.point(vertex) = transform(output.point(vertex));
  }
  return output;
}

static Mesh distributed(
    const Mesh &source,
    const std::vector<std::array<double, 3>> &offsets) {
  Mesh output;
  for (const auto &offset : offsets) {
    const Mesh copy = transformed(source, [&](const Point &point) {
      return Point(point.x() + Kernel::FT(offset[0]),
                   point.y() + Kernel::FT(offset[1]),
                   point.z() + Kernel::FT(offset[2]));
    });
    CGAL::copy_face_graph(copy, output);
  }
  return output;
}

static Mesh subdivide_once(const Mesh &source) {
  std::vector<Point> points;
  points.reserve(source.number_of_vertices() + source.number_of_edges());
  std::vector<std::size_t> vertex_indices(source.num_vertices());
  for (const auto vertex : source.vertices()) {
    vertex_indices[vertex.idx()] = points.size();
    points.push_back(source.point(vertex));
  }
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> midpoints;
  const auto midpoint = [&](std::size_t a, std::size_t b) {
    if (a > b) {
      std::swap(a, b);
    }
    const auto key = std::pair{a, b};
    if (const auto found = midpoints.find(key); found != midpoints.end()) {
      return found->second;
    }
    const Point &left = points[a];
    const Point &right = points[b];
    const std::size_t index = points.size();
    points.emplace_back((left.x() + right.x()) / Kernel::FT(2),
                        (left.y() + right.y()) / Kernel::FT(2),
                        (left.z() + right.z()) / Kernel::FT(2));
    midpoints.emplace(key, index);
    return index;
  };
  std::vector<Triangle> triangles;
  triangles.reserve(source.number_of_faces() * 4);
  for (const auto face : source.faces()) {
    std::array<std::size_t, 3> vertices{};
    std::size_t count = 0;
    for (const auto vertex :
         CGAL::vertices_around_face(source.halfedge(face), source)) {
      if (count < vertices.size()) {
        vertices[count++] = vertex_indices[vertex.idx()];
      }
    }
    if (count != 3) {
      continue;
    }
    const std::size_t ab = midpoint(vertices[0], vertices[1]);
    const std::size_t bc = midpoint(vertices[1], vertices[2]);
    const std::size_t ca = midpoint(vertices[2], vertices[0]);
    triangles.push_back({vertices[0], ab, ca});
    triangles.push_back({ab, vertices[1], bc});
    triangles.push_back({ca, bc, vertices[2]});
    triangles.push_back({ab, bc, ca});
  }
  return mesh_from_soup(std::move(points), std::move(triangles));
}

int main() {
  csgrs_bench::Harness harness("cgal-epeck");
  harness.run("kernel", "construct_box", "unit", 64,
              [] { return measured(box(2.0), 0); });
  harness.run("kernel", "construct_cuboid", "2x4x6", 32, [] {
    return measured(exact_cuboid(Kernel::FT(2), Kernel::FT(4), Kernel::FT(6)),
                    0);
  });
  harness.run("kernel", "construct_cylinder", "r6_h20_s64", 8, [] {
    return measured(extruded_circle(6.0, 20.0, 64), 0);
  });
  harness.run("kernel", "construct_frustum", "r6_r2_h20_s64", 8, [] {
    return measured(frustum(6.0, 2.0, 20.0, 64), 0);
  });
  harness.run("kernel", "construct_octahedron", "r10", 32,
              [] { return measured(octahedron(10.0), 0); });
  harness.run("kernel", "construct_icosahedron", "r10", 16,
              [] { return measured(icosahedron(10.0), 0); });
  harness.run("kernel", "construct_sphere", "medium", 8,
              [] { return measured(sphere(10.0, 32, 16), 0); });
  harness.run("kernel", "construct_sphere", "large", 2,
              [] { return measured(sphere(10.0, 64, 32), 0); });
  harness.run("precision", "construct_sphere", "high_resolution", 1,
              [] { return measured(sphere(10.0, 128, 64), 0); });
  harness.run("kernel", "construct_ellipsoid", "r10_6_4_s32x16", 4, [] {
    return measured(transformed(sphere(1.0, 32, 16), [](const Point &point) {
                      return Point(point.x() * Kernel::FT(10),
                                   point.y() * Kernel::FT(6),
                                   point.z() * Kernel::FT(4));
                    }),
                    0);
  });
  harness.run("kernel", "construct_torus", "r10_2_s32x16", 2,
              [] { return measured(torus(10.0, 2.0, 32, 16), 0); });

  const Mesh transform_source = sphere(10.0, 32, 16);
  const std::size_t transform_input = transform_source.number_of_faces();
  harness.run("kernel", "translate", "sphere_medium", 8, [&] {
    return geometry_measured(transformed(transform_source, [](const Point &point) {
                      return Point(point.x() + Kernel::FT(3),
                                   point.y() - Kernel::FT(2),
                                   point.z() + Kernel::FT(5));
                    }),
                    transform_input);
  });
  harness.run("kernel", "rotate_xyz", "sphere_medium", 8, [&] {
    const double ax = 17.0 * std::numbers::pi / 180.0;
    const double ay = 29.0 * std::numbers::pi / 180.0;
    const double az = 43.0 * std::numbers::pi / 180.0;
    Mesh output = transformed(transform_source, [&](const Point &point) {
      double x = CGAL::to_double(point.x());
      double y = CGAL::to_double(point.y());
      double z = CGAL::to_double(point.z());
      const double yx = y * std::cos(ax) - z * std::sin(ax);
      const double zx = y * std::sin(ax) + z * std::cos(ax);
      y = yx;
      z = zx;
      const double xy = x * std::cos(ay) + z * std::sin(ay);
      const double zy = -x * std::sin(ay) + z * std::cos(ay);
      x = xy;
      z = zy;
      return Point(x * std::cos(az) - y * std::sin(az),
                   x * std::sin(az) + y * std::cos(az), z);
    });
    return geometry_measured(output, transform_input);
  });
  harness.run("kernel", "scale_nonuniform", "sphere_medium", 8, [&] {
    return geometry_measured(transformed(transform_source, [](const Point &point) {
                      return Point(point.x() * Kernel::FT(2),
                                   point.y() / Kernel::FT(2),
                                   point.z() * Kernel::FT(3) / Kernel::FT(2));
                    }),
                    transform_input);
  });
  harness.run("kernel", "mirror", "sphere_across_x_eq_1", 8, [&] {
    Mesh output = transformed(transform_source, [](const Point &point) {
      return Point(Kernel::FT(2) - point.x(), point.y(), point.z());
    });
    PMP::reverse_face_orientations(output);
    return geometry_measured(output, transform_input);
  });
  harness.run("kernel", "affine_transform", "sphere_shear", 8, [&] {
    return geometry_measured(transformed(transform_source, [](const Point &point) {
                      return Point(point.x() + point.y() / Kernel::FT(4) +
                                       Kernel::FT(2),
                                   point.y() + point.z() / Kernel::FT(5) -
                                       Kernel::FT(3),
                                   point.z() + Kernel::FT(4));
                    }),
                    transform_input);
  });
  harness.run("kernel", "inverse", "sphere_orientation", 16, [&] {
    Mesh output = transform_source;
    PMP::reverse_face_orientations(output);
    return geometry_measured(output, transform_input);
  });
  const Mesh off_center = exact_box(Kernel::FT(2), Kernel::FT(7),
                                    Kernel::FT(-3), Kernel::FT(5));
  harness.run("kernel", "center", "translated_box", 32, [&] {
    const CGAL::Bbox_3 bounds = PMP::bbox(off_center);
    const Kernel::FT cx = Kernel::FT((bounds.xmin() + bounds.xmax()) / 2.0);
    const Kernel::FT cy = Kernel::FT((bounds.ymin() + bounds.ymax()) / 2.0);
    const Kernel::FT cz = Kernel::FT((bounds.zmin() + bounds.zmax()) / 2.0);
    return geometry_measured(transformed(off_center, [&](const Point &point) {
                               return Point(point.x() - cx, point.y() - cy,
                                            point.z() - cz);
                             }),
                             12);
  });
  harness.run("kernel", "scale_uniform", "sphere_medium", 8, [&] {
    return geometry_measured(transformed(transform_source, [](const Point &point) {
                               return Point(point.x() * Kernel::FT(2),
                                            point.y() * Kernel::FT(2),
                                            point.z() * Kernel::FT(2));
                             }),
                             transform_input);
  });

  const Mesh boolean_left = sphere(10.0, 12, 6);
  const Mesh boolean_right = box(14.0, 3.0, 2.0, 1.0);
  const std::size_t boolean_input = boolean_left.number_of_faces() +
                                    boolean_right.number_of_faces();
  const auto boolean_case = [&](auto operation) {
    Mesh left = boolean_left;
    Mesh right = boolean_right;
    Mesh output;
    if (!operation(left, right, output)) {
      throw std::runtime_error("CGAL Boolean comparison workload failed");
    }
    return measured(output, boolean_input);
  };
  harness.run("kernel", "boolean_union", "sphere_box", 1, [&] {
    return boolean_case([](Mesh &left, Mesh &right, Mesh &output) {
      return PMP::corefine_and_compute_union(left, right, output);
    });
  });
  harness.run("kernel", "boolean_difference", "sphere_box", 1, [&] {
    return boolean_case([](Mesh &left, Mesh &right, Mesh &output) {
      return PMP::corefine_and_compute_difference(left, right, output);
    });
  });
  harness.run("kernel", "boolean_intersection", "sphere_box", 1, [&] {
    return boolean_case([](Mesh &left, Mesh &right, Mesh &output) {
      return PMP::corefine_and_compute_intersection(left, right, output);
    });
  });
  harness.run("kernel", "boolean_xor", "sphere_box", 1, [&] {
    Mesh left = boolean_left;
    Mesh right = boolean_right;
    Mesh left_only;
    Mesh right_only;
    if (!PMP::corefine_and_compute_difference(left, right, left_only)) {
      throw std::runtime_error("CGAL XOR left difference failed");
    }
    left = boolean_left;
    right = boolean_right;
    if (!PMP::corefine_and_compute_difference(right, left, right_only)) {
      throw std::runtime_error("CGAL XOR right difference failed");
    }
    // The two differences are disjoint by definition. CGAL's corefining union
    // reports false for some nonintersecting component pairs, so materialize
    // the symmetric difference as one Surface_mesh with both components.
    Mesh output = left_only;
    CGAL::copy_face_graph(right_only, output);
    return measured(output, boolean_input);
  });

  const Mesh topology_left = box(4.0);
  const Mesh topology_disjoint = box(4.0, 10.0, 0.0, 0.0);
  const Mesh topology_contained = box(2.0);
  const Mesh topology_touching = box(4.0, 4.0, 0.0, 0.0);
  harness.run("kernel", "boolean_union", "disjoint_boxes", 8, [&] {
    Mesh output = topology_left;
    CGAL::copy_face_graph(topology_disjoint, output);
    return measured(output, 24);
  });
  harness.run("kernel", "boolean_difference", "contained_boxes", 1, [&] {
    Mesh left = topology_left;
    Mesh right = topology_contained;
    Mesh output;
    if (!PMP::corefine_and_compute_difference(left, right, output)) {
      throw std::runtime_error("CGAL contained difference failed");
    }
    return measured(output, 24);
  });
  harness.run("kernel", "boolean_union", "face_touching_boxes", 1, [&] {
    Mesh left = topology_left;
    Mesh right = topology_touching;
    Mesh output;
    if (!PMP::corefine_and_compute_union(left, right, output)) {
      throw std::runtime_error("CGAL face-touching union failed");
    }
    return measured(output, 24);
  });
  harness.run("kernel", "boolean_intersection", "identical_boxes", 8, [&] {
    return measured(topology_left, 24);
  });

  const Kernel::FT sliver_shift =
      Kernel::FT(1999999) / Kernel::FT(1000000);
  const Mesh sliver_left =
      exact_box(Kernel::FT(2), Kernel::FT(0), Kernel::FT(0), Kernel::FT(0));
  const Mesh sliver_right = exact_box(Kernel::FT(2), sliver_shift,
                                      Kernel::FT(0), Kernel::FT(0));
  harness.run("precision", "boolean_sliver", "overlap_1e-6", 1, [&] {
    Mesh left = sliver_left;
    Mesh right = sliver_right;
    Mesh output;
    if (!PMP::corefine_and_compute_intersection(left, right, output) ||
        output.is_empty()) {
      throw std::runtime_error("CGAL exact sliver intersection failed");
    }
    const Kernel::FT expected_volume =
        Kernel::FT(4) / Kernel::FT(1000000);
    if (PMP::volume(output) != expected_volume) {
      throw std::runtime_error("CGAL exact sliver volume changed");
    }
    return measured(output, 24);
  });

  harness.run("kernel", "extrude", "circle_64", 8,
              [] { return measured(extruded_circle(6.0, 20.0, 64), 64); });

  const Mesh distribution_source = box(1.0);
  harness.run("kernel", "distribute_linear", "box_8", 1, [&] {
    std::vector<std::array<double, 3>> offsets;
    for (std::size_t index = 0; index < 8; ++index) {
      offsets.push_back({2.0 * static_cast<double>(index), 0.0, 0.0});
    }
    return measured(distributed(distribution_source, offsets), 12);
  });
  harness.run("kernel", "distribute_grid", "box_4x4", 1, [&] {
    std::vector<std::array<double, 3>> offsets;
    for (std::size_t row = 0; row < 4; ++row) {
      for (std::size_t column = 0; column < 4; ++column) {
        offsets.push_back({2.0 * static_cast<double>(column),
                           2.0 * static_cast<double>(row), 0.0});
      }
    }
    return measured(distributed(distribution_source, offsets), 12);
  });
  harness.run("kernel", "distribute_arc", "box_12_30_degree_steps", 1, [&] {
    std::vector<std::array<double, 3>> offsets;
    for (std::size_t index = 0; index < 12; ++index) {
      const double angle = 30.0 * std::numbers::pi / 180.0 *
                           static_cast<double>(index);
      offsets.push_back({10.0 * std::cos(angle), 10.0 * std::sin(angle), 0.0});
    }
    return measured(distributed(distribution_source, offsets), 12);
  });

  const Mesh analysis_source = sphere(10.0, 32, 16);
  harness.run("kernel", "triangulate", "sphere_medium", 16, [&] {
    Mesh output = analysis_source;
    PMP::triangulate_faces(output);
    return measured(output, analysis_source.number_of_faces());
  });
  harness.run("kernel", "subdivide", "sphere_medium_level1", 2, [&] {
    return measured(subdivide_once(analysis_source),
                    analysis_source.number_of_faces());
  });
  harness.run("kernel", "renormalize", "sphere_medium", 4, [&] {
    Mesh output = analysis_source;
    std::vector<std::array<double, 3>> normals;
    normals.reserve(output.number_of_faces());
    for (const auto face : output.faces()) {
      std::array<Point, 3> points{};
      std::size_t count = 0;
      for (const auto vertex :
           CGAL::vertices_around_face(output.halfedge(face), output)) {
        if (count < points.size()) {
          points[count++] = output.point(vertex);
        }
      }
      if (count == 3) {
        const Kernel::Vector_3 exact =
            CGAL::cross_product(points[1] - points[0], points[2] - points[0]);
        const double x = CGAL::to_double(exact.x());
        const double y = CGAL::to_double(exact.y());
        const double z = CGAL::to_double(exact.z());
        const double length = std::sqrt(x * x + y * y + z * z);
        if (length > 0.0) {
          normals.push_back({x / length, y / length, z / length});
        }
      }
    }
    return Measurement{analysis_source.number_of_faces(), normals.size(),
                       normals.size()};
  });
  harness.run("kernel", "materialize_finite", "sphere_medium", 4, [&] {
    Mesh output = transformed(analysis_source, [](const Point &point) {
      return Point(CGAL::to_double(point.x()), CGAL::to_double(point.y()),
                   CGAL::to_double(point.z()));
    });
    return geometry_measured(output, analysis_source.number_of_faces());
  });
  harness.run("kernel", "bounding_box", "sphere_medium", 128, [&] {
    const CGAL::Bbox_3 bounds = PMP::bbox(analysis_source);
    return Measurement{analysis_source.number_of_faces(), 6,
                       std::bit_cast<std::uint64_t>(bounds.xmax())};
  });
  harness.run("kernel", "mass_properties", "sphere_medium", 4, [&] {
    const double volume = CGAL::to_double(PMP::volume(analysis_source));
    return Measurement{analysis_source.number_of_faces(), 10,
                       std::bit_cast<std::uint64_t>(volume)};
  });
  harness.run("kernel", "vertices", "sphere_medium", 32, [&] {
    std::size_t corners = 0;
    for (const auto face : analysis_source.faces()) {
      for ([[maybe_unused]] const auto vertex : CGAL::vertices_around_face(
               analysis_source.halfedge(face), analysis_source)) {
        ++corners;
      }
    }
    return Measurement{analysis_source.number_of_faces(), corners, corners};
  });
  harness.run("kernel", "graphics_buffers", "sphere_medium", 16, [&] {
    std::size_t corners = 0;
    for (const auto face : analysis_source.faces()) {
      for ([[maybe_unused]] const auto vertex : CGAL::vertices_around_face(
               analysis_source.halfedge(face), analysis_source)) {
        ++corners;
      }
    }
    return Measurement{analysis_source.number_of_faces(), corners,
                       csgrs_bench::checksum(corners, corners)};
  });
  harness.run("kernel", "connectivity", "sphere_medium", 8, [&] {
    std::vector<std::vector<std::size_t>> adjacency(
        analysis_source.number_of_vertices());
    for (const auto edge : analysis_source.edges()) {
      const auto halfedge = analysis_source.halfedge(edge);
      const std::size_t source =
          static_cast<std::size_t>(analysis_source.source(halfedge).idx());
      const std::size_t target =
          static_cast<std::size_t>(analysis_source.target(halfedge).idx());
      adjacency[source].push_back(target);
      adjacency[target].push_back(source);
    }
    std::size_t populated = 0;
    for (auto &neighbors : adjacency) {
      std::sort(neighbors.begin(), neighbors.end());
      neighbors.erase(std::unique(neighbors.begin(), neighbors.end()),
                      neighbors.end());
      populated += !neighbors.empty();
    }
    return Measurement{
        analysis_source.number_of_faces(), analysis_source.number_of_vertices(),
        csgrs_bench::checksum(analysis_source.number_of_vertices(), populated)};
  });
  harness.run("kernel", "is_manifold", "sphere_medium", 32, [&] {
    const bool manifold = CGAL::is_closed(analysis_source) &&
                          CGAL::is_valid_polygon_mesh(analysis_source);
    return Measurement{analysis_source.number_of_faces(), 1, manifold ? 1U : 0U};
  });
  harness.run("kernel", "contains_point", "sphere_two_queries", 8, [&] {
    const CGAL::Side_of_triangle_mesh<Mesh, Kernel> side(analysis_source);
    const bool inside = side(Point(0, 0, 0)) != CGAL::ON_UNBOUNDED_SIDE;
    const bool outside = side(Point(20, 0, 0)) != CGAL::ON_UNBOUNDED_SIDE;
    return Measurement{analysis_source.number_of_faces(), 2,
                       static_cast<std::uint64_t>(inside) |
                           (static_cast<std::uint64_t>(outside) << 1U)};
  });
  harness.run("kernel", "ray_intersections", "sphere_diameter", 8, [&] {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
    using Traits = CGAL::AABB_traits_3<Kernel, Primitive>;
    CGAL::AABB_tree<Traits> tree(faces(analysis_source).first,
                                 faces(analysis_source).second,
                                 analysis_source);
    const std::size_t hits = tree.number_of_intersected_primitives(
        Kernel::Ray_3(Point(-20, 0, 0), Point(20, 0, 0)));
    return Measurement{analysis_source.number_of_faces(), hits, hits};
  });
  harness.run("kernel", "polyline_intersections", "sphere_diameter", 8, [&] {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
    using Traits = CGAL::AABB_traits_3<Kernel, Primitive>;
    CGAL::AABB_tree<Traits> tree(faces(analysis_source).first,
                                 faces(analysis_source).second,
                                 analysis_source);
    const std::size_t hits = tree.number_of_intersected_primitives(
        Kernel::Segment_3(Point(-20, 0, 0), Point(20, 0, 0)));
    return Measurement{analysis_source.number_of_faces(), hits, hits};
  });
  harness.run("kernel", "dihedral_angle", "box_adjacent_faces", 32, [&] {
    const Mesh shape = box(2.0);
    std::vector<Kernel::Vector_3> normals;
    for (const auto face : shape.faces()) {
      std::array<Point, 3> points{};
      std::size_t count = 0;
      for (const auto vertex :
           CGAL::vertices_around_face(shape.halfedge(face), shape)) {
        points[count++] = shape.point(vertex);
      }
      normals.push_back(
          CGAL::cross_product(points[1] - points[0], points[2] - points[0]));
    }
    const auto adjacent = std::find_if(
        normals.begin() + 1, normals.end(), [&](const Kernel::Vector_3 &normal) {
          return CGAL::scalar_product(normals.front(), normal) == Kernel::FT(0);
        });
    if (adjacent == normals.end()) {
      throw std::runtime_error("CGAL cube has no adjacent face pair");
    }
    const double angle = CGAL::to_double(
        CGAL::approximate_angle(normals.front(), *adjacent));
    return Measurement{12, 1, std::bit_cast<std::uint64_t>(angle)};
  });
  harness.run("kernel", "stl_write", "sphere_medium", 8, [&] {
    std::ostringstream output(std::ios::binary);
    CGAL::IO::set_binary_mode(output);
    if (!CGAL::IO::write_STL(output, analysis_source)) {
      throw std::runtime_error("CGAL STL comparison workload failed");
    }
    const std::size_t size = output.str().size();
    return Measurement{analysis_source.number_of_faces(), size, size};
  });
}
