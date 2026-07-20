#include "common.hpp"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <StlAPI_Writer.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Lin.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using csgrs_bench::Measurement;

static_assert(std::numeric_limits<Standard_Real>::digits ==
                  std::numeric_limits<double>::digits,
              "OCCT precision policy assumes Standard_Real is double");

struct MeshingPolicy {
  Standard_Real linear_deflection;
  Standard_Real angular_deflection;
};

static constexpr MeshingPolicy kComparisonMesh{0.25, 0.2};
static constexpr MeshingPolicy kLargeMesh{0.125, 0.1};
static constexpr MeshingPolicy kHighResolutionMesh{0.005, 0.07};

static void triangulate(TopoDS_Shape &shape,
                        const MeshingPolicy &policy = kComparisonMesh) {
  BRepTools::Clean(shape);
  BRepMesh_IncrementalMesh mesher(shape, policy.linear_deflection,
                                  Standard_False, policy.angular_deflection,
                                  Standard_True);
  mesher.Perform();
  if (!mesher.IsDone()) {
    throw std::runtime_error("OCCT comparison tessellation failed");
  }
}

static std::size_t triangle_count(const TopoDS_Shape &shape) {
  std::size_t triangles = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    TopLoc_Location location;
    const Handle(Poly_Triangulation) mesh =
        BRep_Tool::Triangulation(TopoDS::Face(explorer.Current()), location);
    if (!mesh.IsNull()) {
      triangles += static_cast<std::size_t>(mesh->NbTriangles());
    }
  }
  return triangles;
}

static Standard_Real max_shape_tolerance(const TopoDS_Shape &shape) {
  Standard_Real tolerance = 0.0;
  for (TopExp_Explorer explorer(shape, TopAbs_VERTEX); explorer.More();
       explorer.Next()) {
    tolerance = std::max(
        tolerance, BRep_Tool::Tolerance(TopoDS::Vertex(explorer.Current())));
  }
  for (TopExp_Explorer explorer(shape, TopAbs_EDGE); explorer.More();
       explorer.Next()) {
    tolerance = std::max(
        tolerance, BRep_Tool::Tolerance(TopoDS::Edge(explorer.Current())));
  }
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    tolerance = std::max(
        tolerance, BRep_Tool::Tolerance(TopoDS::Face(explorer.Current())));
  }
  return tolerance;
}

static Measurement measured(TopoDS_Shape shape, std::size_t input_facets,
                            bool needs_meshing = true,
                            const MeshingPolicy &policy = kComparisonMesh) {
  if (needs_meshing) {
    triangulate(shape, policy);
  }
  const std::size_t facets = triangle_count(shape);
  return {input_facets, facets, csgrs_bench::checksum(facets, facets * 3)};
}

static Measurement geometry_measured(
    TopoDS_Shape shape, std::size_t input_facets,
    const MeshingPolicy &policy = kComparisonMesh) {
  Measurement result = measured(shape, input_facets, true, policy);
  Bnd_Box bounds;
  BRepBndLib::Add(shape, bounds, Standard_True);
  Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
  bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  for (const Standard_Real coordinate : {xmin, ymin, zmin, xmax, ymax, zmax}) {
    result.checksum = std::rotl(result.checksum, 7) ^
                      std::bit_cast<std::uint64_t>(coordinate);
  }
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    result.checksum =
        std::rotl(result.checksum, 3) ^ explorer.Current().Orientation();
  }
  return result;
}

static TopoDS_Shape centered_box(double width, double cx = 0.0, double cy = 0.0,
                                 double cz = 0.0) {
  const double half = width / 2.0;
  return BRepPrimAPI_MakeBox(gp_Pnt(cx - half, cy - half, cz - half), width,
                             width, width)
      .Shape();
}

static TopoDS_Shape centered_cuboid(double width, double length,
                                    double height) {
  return BRepPrimAPI_MakeBox(gp_Pnt(-width / 2.0, -length / 2.0,
                                    -height / 2.0),
                             width, length, height)
      .Shape();
}

static TopoDS_Shape polyhedron_surface(
    const std::vector<gp_Pnt> &points,
    const std::vector<std::array<std::size_t, 3>> &triangles) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const auto &triangle : triangles) {
    BRepBuilderAPI_MakePolygon wire;
    wire.Add(points.at(triangle[0]));
    wire.Add(points.at(triangle[1]));
    wire.Add(points.at(triangle[2]));
    wire.Close();
    builder.Add(compound, BRepBuilderAPI_MakeFace(wire.Wire()).Face());
  }
  return compound;
}

static TopoDS_Shape solid_from_obj_soup(
    const csgrs_bench::ObjTriangleSoup &soup) {
  BRepBuilderAPI_Sewing sewing(Precision::Confusion(), Standard_True,
                               Standard_True, Standard_True, Standard_False);
  for (const auto &triangle : soup.triangles) {
    BRepBuilderAPI_MakePolygon wire;
    for (const std::size_t index : triangle) {
      const auto &point = soup.points.at(index);
      wire.Add(gp_Pnt(point[0], point[1], point[2]));
    }
    wire.Close();
    sewing.Add(BRepBuilderAPI_MakeFace(wire.Wire()).Face());
  }
  sewing.Perform();
  const TopoDS_Shape sewed = sewing.SewedShape();
  BRepBuilderAPI_MakeSolid make_solid;
  std::size_t shell_count{};
  for (TopExp_Explorer explorer(sewed, TopAbs_SHELL); explorer.More();
       explorer.Next()) {
    make_solid.Add(TopoDS::Shell(explorer.Current()));
    ++shell_count;
  }
  if (shell_count == 0 || !make_solid.IsDone()) {
    throw std::runtime_error("OCCT could not sew YeahRight into a solid");
  }
  TopoDS_Solid solid = make_solid.Solid();
  if (!BRepCheck_Analyzer(solid, Standard_True).IsValid()) {
    throw std::runtime_error("OCCT YeahRight solid is invalid after sewing");
  }
  return solid;
}

static TopoDS_Shape yeahright_boolean_operand(const TopoDS_Shape &source) {
  gp_Trsf rotation;
  rotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)),
                       std::numbers::pi / 2.0);
  const TopoDS_Shape rotated =
      BRepBuilderAPI_Transform(source, rotation, Standard_True).Shape();
  gp_Trsf translation;
  translation.SetTranslation(gp_Vec(1, 12, 1));
  return BRepBuilderAPI_Transform(rotated, translation, Standard_True).Shape();
}

static TopoDS_Shape octahedron(double radius) {
  return polyhedron_surface(
      {{radius, 0, 0}, {-radius, 0, 0}, {0, radius, 0}, {0, -radius, 0},
       {0, 0, radius}, {0, 0, -radius}},
      {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4},
       {5, 2, 0}, {5, 1, 2}, {5, 3, 1}, {5, 0, 3}});
}

static TopoDS_Shape icosahedron(double radius) {
  const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
  const double scale = radius / std::sqrt(1.0 + phi * phi);
  const double a = scale;
  const double b = phi * scale;
  return polyhedron_surface(
      {{-a, b, 0}, {a, b, 0}, {-a, -b, 0}, {a, -b, 0}, {0, -a, b},
       {0, a, b}, {0, -a, -b}, {0, a, -b}, {b, 0, -a}, {b, 0, a},
       {-b, 0, -a}, {-b, 0, a}},
      {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
       {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
       {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
       {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}});
}

static TopoDS_Shape distributed(
    const TopoDS_Shape &source,
    const std::vector<std::array<double, 3>> &offsets) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const auto &offset : offsets) {
    gp_Trsf translation;
    translation.SetTranslation(gp_Vec(offset[0], offset[1], offset[2]));
    builder.Add(compound,
                BRepBuilderAPI_Transform(source, translation, Standard_True)
                    .Shape());
  }
  return compound;
}

static std::size_t shape_count(const TopoDS_Shape &shape,
                               TopAbs_ShapeEnum kind) {
  TopTools_IndexedMapOfShape shapes;
  TopExp::MapShapes(shape, kind, shapes);
  return static_cast<std::size_t>(shapes.Extent());
}

static std::size_t face_normal_count(const TopoDS_Shape &shape) {
  std::size_t count = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    BRepAdaptor_Surface surface(face, Standard_True);
    const Standard_Real u =
        (surface.FirstUParameter() + surface.LastUParameter()) / 2.0;
    const Standard_Real v =
        (surface.FirstVParameter() + surface.LastVParameter()) / 2.0;
    gp_Pnt point;
    gp_Vec du;
    gp_Vec dv;
    surface.D1(u, v, point, du, dv);
    if (du.Crossed(dv).SquareMagnitude() > 0.0) {
      ++count;
    }
  }
  return count;
}

struct TriangulatedConnectivity {
  std::size_t vertices = 0;
  std::size_t adjacency_vertices = 0;
};

static TriangulatedConnectivity build_triangulated_connectivity(
    const TopoDS_Shape &shape) {
  std::vector<std::vector<std::size_t>> adjacency;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    TopLoc_Location location;
    const Handle(Poly_Triangulation) mesh =
        BRep_Tool::Triangulation(TopoDS::Face(explorer.Current()), location);
    if (mesh.IsNull()) {
      continue;
    }
    const std::size_t offset = adjacency.size();
    adjacency.resize(offset + static_cast<std::size_t>(mesh->NbNodes()));
    for (Standard_Integer index = 1; index <= mesh->NbTriangles(); ++index) {
      Standard_Integer a;
      Standard_Integer b;
      Standard_Integer c;
      mesh->Triangle(index).Get(a, b, c);
      const std::array<std::size_t, 3> triangle{
          offset + static_cast<std::size_t>(a - 1),
          offset + static_cast<std::size_t>(b - 1),
          offset + static_cast<std::size_t>(c - 1)};
      for (const auto edge :
           {std::array{triangle[0], triangle[1]},
            std::array{triangle[1], triangle[2]},
            std::array{triangle[2], triangle[0]}}) {
        adjacency[edge[0]].push_back(edge[1]);
        adjacency[edge[1]].push_back(edge[0]);
      }
    }
  }
  std::size_t populated = 0;
  for (auto &neighbors : adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()),
                    neighbors.end());
    populated += !neighbors.empty();
  }
  return {adjacency.size(), populated};
}

static std::size_t materialize_triangle_normals(const TopoDS_Shape &shape) {
  std::vector<gp_Vec> normals;
  normals.reserve(triangle_count(shape));
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    TopLoc_Location location;
    const Handle(Poly_Triangulation) mesh =
        BRep_Tool::Triangulation(TopoDS::Face(explorer.Current()), location);
    if (mesh.IsNull()) {
      continue;
    }
    for (Standard_Integer index = 1; index <= mesh->NbTriangles(); ++index) {
      Standard_Integer ia;
      Standard_Integer ib;
      Standard_Integer ic;
      mesh->Triangle(index).Get(ia, ib, ic);
      const gp_Pnt a =
          mesh->Node(ia).Transformed(location.Transformation());
      const gp_Pnt b =
          mesh->Node(ib).Transformed(location.Transformation());
      const gp_Pnt c =
          mesh->Node(ic).Transformed(location.Transformation());
      gp_Vec normal = gp_Vec(a, b).Crossed(gp_Vec(a, c));
      if (normal.SquareMagnitude() > 0.0) {
        normal.Normalize();
        normals.push_back(normal);
      }
    }
  }
  return normals.size();
}

static TopoDS_Shape extruded_circle(double radius, double height,
                                    std::size_t segments) {
  BRepBuilderAPI_MakePolygon polygon;
  for (std::size_t segment = 0; segment < segments; ++segment) {
    const double angle = 2.0 * std::numbers::pi *
                         static_cast<double>(segment) /
                         static_cast<double>(segments);
    polygon.Add(gp_Pnt(radius * std::cos(angle), radius * std::sin(angle), 0));
  }
  polygon.Close();
  const TopoDS_Face face = BRepBuilderAPI_MakeFace(polygon.Wire()).Face();
  return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, height)).Shape();
}

template <typename BooleanOperation>
static TopoDS_Shape boolean_shape(const TopoDS_Shape &left,
                                  const TopoDS_Shape &right) {
  BooleanOperation operation(left, right);
  // OCCT has no arbitrary-precision scalar mode. The tight comparison profile
  // uses the kernel's stored shape tolerances with no additional fuzzy band.
  operation.SetFuzzyValue(0.0);
  operation.SetNonDestructive(Standard_True);
  operation.SetRunParallel(Standard_False);
  operation.Build();
  if (!operation.IsDone()) {
    throw std::runtime_error("OCCT Boolean comparison workload failed");
  }
  return operation.Shape();
}

int main() {
  csgrs_bench::Harness harness("opencascade-double-tight");
  harness.run("kernel", "construct_box", "unit", 64, [] {
    return measured(centered_box(2.0), 0);
  });
  harness.run("kernel", "construct_cuboid", "2x4x6", 32, [] {
    return measured(centered_cuboid(2.0, 4.0, 6.0), 0);
  });
  harness.run("kernel", "construct_cylinder", "r6_h20_s64", 8, [] {
    return measured(BRepPrimAPI_MakeCylinder(6.0, 20.0).Shape(), 0);
  });
  harness.run("kernel", "construct_frustum", "r6_r2_h20_s64", 8, [] {
    return measured(BRepPrimAPI_MakeCone(6.0, 2.0, 20.0).Shape(), 0);
  });
  harness.run("kernel", "construct_octahedron", "r10", 32, [] {
    return measured(octahedron(10.0), 0);
  });
  harness.run("kernel", "construct_icosahedron", "r10", 16, [] {
    return measured(icosahedron(10.0), 0);
  });
  harness.run("kernel", "construct_sphere", "medium", 8, [] {
    return measured(BRepPrimAPI_MakeSphere(10.0).Shape(), 0);
  });
  harness.run("kernel", "construct_sphere", "large", 2, [] {
    return measured(BRepPrimAPI_MakeSphere(10.0).Shape(), 0, true, kLargeMesh);
  });
  harness.run("precision", "construct_sphere", "high_resolution", 1, [] {
    return measured(BRepPrimAPI_MakeSphere(10.0).Shape(), 0, true,
                    kHighResolutionMesh);
  });
  harness.run("kernel", "construct_ellipsoid", "r10_6_4_s32x16", 4, [] {
    gp_GTrsf scale;
    scale.SetVectorialPart(
        gp_Mat(10.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 4.0));
    return measured(BRepBuilderAPI_GTransform(
                        BRepPrimAPI_MakeSphere(1.0).Shape(), scale,
                        Standard_True)
                        .Shape(),
                    0);
  });
  harness.run("kernel", "construct_torus", "r10_2_s32x16", 2, [] {
    return measured(BRepPrimAPI_MakeTorus(10.0, 2.0).Shape(), 0);
  });

  const TopoDS_Shape transform_source = BRepPrimAPI_MakeSphere(10.0).Shape();
  harness.run("kernel", "translate", "sphere_medium", 8, [&] {
    gp_Trsf translation;
    translation.SetTranslation(gp_Vec(3, -2, 5));
    TopoDS_Shape output =
        BRepBuilderAPI_Transform(transform_source, translation, Standard_True)
            .Shape();
    return geometry_measured(output, 960);
  });
  harness.run("kernel", "rotate_xyz", "sphere_medium", 8, [&] {
    TopoDS_Shape output = transform_source;
    for (const auto &[axis, degrees] :
         {std::pair{gp_Dir(1, 0, 0), 17.0},
          std::pair{gp_Dir(0, 1, 0), 29.0},
          std::pair{gp_Dir(0, 0, 1), 43.0}}) {
      gp_Trsf rotation;
      rotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), axis),
                           degrees * std::numbers::pi / 180.0);
      output = BRepBuilderAPI_Transform(output, rotation, Standard_True).Shape();
    }
    return geometry_measured(output, 960);
  });
  harness.run("kernel", "scale_nonuniform", "sphere_medium", 8, [&] {
    gp_GTrsf scale;
    scale.SetVectorialPart(gp_Mat(2.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                   1.5));
    TopoDS_Shape output =
        BRepBuilderAPI_GTransform(transform_source, scale, Standard_True).Shape();
    return geometry_measured(output, 960);
  });
  harness.run("kernel", "mirror", "sphere_across_x_eq_1", 8, [&] {
    gp_Trsf reflection;
    reflection.SetMirror(gp_Ax2(gp_Pnt(1, 0, 0), gp_Dir(1, 0, 0)));
    TopoDS_Shape output =
        BRepBuilderAPI_Transform(transform_source, reflection, Standard_True)
            .Shape();
    return geometry_measured(output, 960);
  });
  harness.run("kernel", "affine_transform", "sphere_shear", 8, [&] {
    gp_GTrsf affine;
    affine.SetVectorialPart(
        gp_Mat(1.0, 0.25, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, 1.0));
    affine.SetTranslationPart(gp_XYZ(2, -3, 4));
    TopoDS_Shape output =
        BRepBuilderAPI_GTransform(transform_source, affine, Standard_True)
            .Shape();
    return geometry_measured(output, 960);
  });
  harness.run("kernel", "inverse", "sphere_orientation", 16, [&] {
    TopoDS_Shape output = transform_source;
    output.Reverse();
    return geometry_measured(output, 960);
  });
  const TopoDS_Shape off_center = centered_box(2.0, 7.0, -3.0, 5.0);
  harness.run("kernel", "center", "translated_box", 32, [&] {
    Bnd_Box bounds;
    BRepBndLib::Add(off_center, bounds, Standard_True);
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    gp_Trsf translation;
    translation.SetTranslation(gp_Vec(-(xmin + xmax) / 2.0,
                                      -(ymin + ymax) / 2.0,
                                      -(zmin + zmax) / 2.0));
    return geometry_measured(
        BRepBuilderAPI_Transform(off_center, translation, Standard_True).Shape(),
        12);
  });
  harness.run("kernel", "scale_uniform", "sphere_medium", 8, [&] {
    gp_Trsf scale;
    scale.SetScale(gp_Pnt(0, 0, 0), 2.0);
    return geometry_measured(
        BRepBuilderAPI_Transform(transform_source, scale, Standard_True).Shape(),
        960);
  });

  const TopoDS_Shape boolean_left = BRepPrimAPI_MakeSphere(10.0).Shape();
  const TopoDS_Shape boolean_right = centered_box(14.0, 3.0, 2.0, 1.0);
  harness.run("kernel", "boolean_union", "sphere_box", 1, [&] {
    return measured(boolean_shape<BRepAlgoAPI_Fuse>(boolean_left, boolean_right),
                    132);
  });
  harness.run("kernel", "boolean_difference", "sphere_box", 1, [&] {
    return measured(boolean_shape<BRepAlgoAPI_Cut>(boolean_left, boolean_right),
                    132);
  });
  harness.run("kernel", "boolean_intersection", "sphere_box", 1, [&] {
    return measured(boolean_shape<BRepAlgoAPI_Common>(boolean_left, boolean_right),
                    132);
  });
  harness.run("kernel", "boolean_xor", "sphere_box", 1, [&] {
    const TopoDS_Shape left_only =
        boolean_shape<BRepAlgoAPI_Cut>(boolean_left, boolean_right);
    const TopoDS_Shape right_only =
        boolean_shape<BRepAlgoAPI_Cut>(boolean_right, boolean_left);
    return measured(boolean_shape<BRepAlgoAPI_Fuse>(left_only, right_only), 132);
  });

  const TopoDS_Shape topology_left = centered_box(4.0);
  const TopoDS_Shape topology_contained = centered_box(2.0);
  const TopoDS_Shape topology_touching = centered_box(4.0, 4.0, 0.0, 0.0);
  harness.run("kernel", "boolean_union", "disjoint_boxes", 8, [&] {
    return measured(distributed(topology_left, {{0, 0, 0}, {10, 0, 0}}), 24);
  });
  harness.run("kernel", "boolean_difference", "contained_boxes", 1, [&] {
    return measured(
        boolean_shape<BRepAlgoAPI_Cut>(topology_left, topology_contained), 24);
  });
  harness.run("kernel", "boolean_union", "face_touching_boxes", 1, [&] {
    return measured(
        boolean_shape<BRepAlgoAPI_Fuse>(topology_left, topology_touching), 24);
  });
  harness.run("kernel", "boolean_intersection", "identical_boxes", 8, [&] {
    return measured(topology_left, 24);
  });

  // Precision::Confusion() is 1e-7 in OCCT. A 1e-6 overlap is deliberately
  // above that modeling threshold and is run without added Boolean fuzz.
  static_assert(1.0e-6 > Precision::Confusion());
  const TopoDS_Shape sliver_left = centered_box(2.0);
  const TopoDS_Shape sliver_right = centered_box(2.0, 1.999999, 0.0, 0.0);
  harness.run("precision", "boolean_sliver", "overlap_1e-6", 1, [&] {
    TopoDS_Shape output =
        boolean_shape<BRepAlgoAPI_Common>(sliver_left, sliver_right);
    if (output.IsNull()) {
      throw std::runtime_error("OCCT tight-tolerance sliver was lost");
    }
    if (max_shape_tolerance(output) >= 1.0e-6) {
      throw std::runtime_error(
          "OCCT result tolerance is not smaller than the sliver");
    }
    GProp_GProps properties;
    BRepGProp::VolumeProperties(output, properties);
    if (properties.Mass() < 3.5e-6 || properties.Mass() > 4.5e-6) {
      throw std::runtime_error("OCCT tight-tolerance sliver volume changed");
    }
    return measured(output, 24, true, kHighResolutionMesh);
  });

  harness.run("kernel", "extrude", "circle_64", 8, [] {
    return measured(extruded_circle(6.0, 20.0, 64), 64);
  });

  const TopoDS_Shape distribution_source = centered_box(1.0);
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

  const TopoDS_Shape analysis_source = BRepPrimAPI_MakeSphere(10.0).Shape();
  TopoDS_Shape analysis_mesh_source = analysis_source;
  triangulate(analysis_mesh_source);
  harness.run("kernel", "triangulate", "sphere_medium", 16, [&] {
    TopoDS_Shape output = analysis_source;
    triangulate(output);
    return measured(output, 960, false);
  });
  harness.run("kernel", "subdivide", "sphere_medium_level1", 2, [&] {
    TopoDS_Shape output = analysis_source;
    triangulate(output, kHighResolutionMesh);
    return measured(output, 960, false);
  });
  harness.run("kernel", "renormalize", "sphere_medium", 4, [&] {
    TopoDS_Shape output = BRepBuilderAPI_Copy(analysis_mesh_source).Shape();
    triangulate(output);
    const std::size_t normals = materialize_triangle_normals(output);
    return Measurement{960, normals, normals};
  });
  harness.run("kernel", "materialize_finite", "sphere_medium", 4, [&] {
    return geometry_measured(BRepBuilderAPI_Copy(analysis_source).Shape(), 960);
  });
  harness.run("kernel", "bounding_box", "sphere_medium", 128, [&] {
    Bnd_Box bounds;
    BRepBndLib::Add(analysis_source, bounds, Standard_True);
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return Measurement{1, 6, std::bit_cast<std::uint64_t>(xmax)};
  });
  harness.run("kernel", "mass_properties", "sphere_medium", 4, [&] {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(analysis_source, properties);
    return Measurement{1, 10,
                       std::bit_cast<std::uint64_t>(properties.Mass())};
  });
  harness.run("kernel", "vertices", "sphere_medium", 32, [&] {
    TopoDS_Shape output = analysis_source;
    triangulate(output);
    const std::size_t corners = triangle_count(output) * 3;
    return Measurement{960, corners, corners};
  });
  harness.run("kernel", "graphics_buffers", "sphere_medium", 16, [&] {
    TopoDS_Shape output = analysis_source;
    triangulate(output);
    const std::size_t corners = triangle_count(output) * 3;
    return Measurement{960, corners, csgrs_bench::checksum(corners, corners)};
  });
  harness.run("kernel", "connectivity", "sphere_medium", 8, [&] {
    const TriangulatedConnectivity topology =
        build_triangulated_connectivity(analysis_mesh_source);
    return Measurement{
        960, topology.vertices,
        csgrs_bench::checksum(topology.vertices, topology.adjacency_vertices)};
  });
  harness.run("kernel", "is_manifold", "sphere_medium", 32, [&] {
    const bool valid = BRepCheck_Analyzer(analysis_source, Standard_True).IsValid();
    return Measurement{960, 1, valid ? 1U : 0U};
  });
  harness.run("kernel", "contains_point", "sphere_two_queries", 8, [&] {
    BRepClass3d_SolidClassifier classifier(analysis_source);
    classifier.Perform(gp_Pnt(0, 0, 0), Precision::Confusion());
    const bool inside = classifier.State() != TopAbs_OUT;
    classifier.Perform(gp_Pnt(20, 0, 0), Precision::Confusion());
    const bool outside = classifier.State() != TopAbs_OUT;
    return Measurement{960, 2, static_cast<std::uint64_t>(inside) |
                                   (static_cast<std::uint64_t>(outside) << 1U)};
  });
  harness.run("kernel", "ray_intersections", "sphere_diameter", 8, [&] {
    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(analysis_source, Precision::Confusion());
    intersector.Perform(gp_Lin(gp_Pnt(-20, 0, 0), gp_Dir(1, 0, 0)), 0.0,
                        40.0);
    const std::size_t hits = static_cast<std::size_t>(intersector.NbPnt());
    return Measurement{960, hits, hits};
  });
  harness.run("kernel", "polyline_intersections", "sphere_diameter", 8, [&] {
    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(analysis_source, Precision::Confusion());
    intersector.Perform(gp_Lin(gp_Pnt(-20, 0, 0), gp_Dir(1, 0, 0)), 0.0,
                        40.0);
    const std::size_t hits = static_cast<std::size_t>(intersector.NbPnt());
    return Measurement{960, hits, hits};
  });
  harness.run("kernel", "dihedral_angle", "box_adjacent_faces", 32, [&] {
    const TopoDS_Shape shape = centered_box(2.0);
    std::vector<gp_Vec> normals;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
         explorer.Next()) {
      BRepAdaptor_Surface surface(TopoDS::Face(explorer.Current()), Standard_True);
      const Standard_Real u =
          (surface.FirstUParameter() + surface.LastUParameter()) / 2.0;
      const Standard_Real v =
          (surface.FirstVParameter() + surface.LastVParameter()) / 2.0;
      gp_Pnt point;
      gp_Vec du;
      gp_Vec dv;
      surface.D1(u, v, point, du, dv);
      gp_Vec normal = du.Crossed(dv);
      if (normal.SquareMagnitude() > 0.0) {
        normal.Normalize();
        normals.push_back(normal);
      }
    }
    const auto adjacent = std::find_if(
        normals.begin() + 1, normals.end(), [&](const gp_Vec &normal) {
          return std::abs(normals.front().Dot(normal)) < 0.5;
        });
    if (adjacent == normals.end()) {
      throw std::runtime_error("OCCT cube has no adjacent face pair");
    }
    const double angle = normals.front().Angle(*adjacent);
    return Measurement{12, 1, std::bit_cast<std::uint64_t>(angle)};
  });

  harness.run("corpus", "obj_import", "yeahright_control_genus131", 1, [] {
    const auto soup =
        csgrs_bench::read_obj_triangle_soup(csgrs_bench::yeahright_control_path());
    return measured(solid_from_obj_soup(soup), soup.source_faces);
  });
  const auto yeahright_soup =
      csgrs_bench::read_obj_triangle_soup(csgrs_bench::yeahright_control_path());
  const TopoDS_Shape yeahright_source = solid_from_obj_soup(yeahright_soup);
  const std::size_t yeahright_input = yeahright_soup.triangles.size();

  harness.run("corpus", "rotate_translate",
              "yeahright_control_rot90_offset", 1, [&] {
                return geometry_measured(
                    yeahright_boolean_operand(yeahright_source),
                    yeahright_input);
              });
  harness.run("corpus", "bounding_box", "yeahright_control_genus131", 1,
              [&] {
                Bnd_Box bounds;
                BRepBndLib::Add(yeahright_source, bounds, Standard_True);
                Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                return Measurement{
                    yeahright_input, 6,
                    std::bit_cast<std::uint64_t>(xmax) ^
                        std::bit_cast<std::uint64_t>(ymax) ^
                        std::bit_cast<std::uint64_t>(zmax)};
              });
  harness.run("corpus", "graphics_buffers", "yeahright_control_genus131", 1,
              [&] {
                TopoDS_Shape output =
                    BRepBuilderAPI_Copy(yeahright_source).Shape();
                triangulate(output);
                const std::size_t corners = triangle_count(output) * 3;
                return Measurement{yeahright_input, corners,
                                   csgrs_bench::checksum(corners, corners)};
              });
  harness.run("corpus", "connectivity", "yeahright_control_genus131", 1,
              [&] {
                TopoDS_Shape output =
                    BRepBuilderAPI_Copy(yeahright_source).Shape();
                triangulate(output);
                const auto topology = build_triangulated_connectivity(output);
                return Measurement{
                    yeahright_input, topology.vertices,
                    csgrs_bench::checksum(topology.vertices,
                                          topology.adjacency_vertices)};
              });
  harness.run("corpus", "is_manifold", "yeahright_control_genus131", 1,
              [&] {
                const bool valid =
                    BRepCheck_Analyzer(yeahright_source, Standard_True).IsValid();
                return Measurement{yeahright_input, 1, valid ? 1U : 0U};
              });
  const auto yeahright_boolean_soup = csgrs_bench::read_obj_triangle_soup(
      csgrs_bench::yeahright_boolean_hull_path());
  const TopoDS_Shape yeahright_boolean_source =
      solid_from_obj_soup(yeahright_boolean_soup);
  gp_Trsf yeahright_box_translation;
  yeahright_box_translation.SetTranslation(gp_Vec(-10, 6, 0));
  const TopoDS_Shape yeahright_box = BRepBuilderAPI_Transform(
                                          centered_cuboid(20.0, 40.0, 40.0),
                                          yeahright_box_translation,
                                          Standard_True)
                                          .Shape();
  const std::size_t yeahright_box_input =
      yeahright_boolean_soup.triangles.size() + 12;
  harness.run("corpus", "boolean_all", "yeahright_hull_box", 1,
              [&] {
                Measurement total;
                total += measured(
                    boolean_shape<BRepAlgoAPI_Fuse>(yeahright_boolean_source,
                                                    yeahright_box),
                    yeahright_box_input);
                total += measured(
                    boolean_shape<BRepAlgoAPI_Cut>(yeahright_boolean_source,
                                                   yeahright_box),
                    yeahright_box_input);
                total += measured(
                    boolean_shape<BRepAlgoAPI_Common>(yeahright_boolean_source,
                                                      yeahright_box),
                    yeahright_box_input);
                const TopoDS_Shape left_only =
                    boolean_shape<BRepAlgoAPI_Cut>(yeahright_boolean_source,
                                                   yeahright_box);
                const TopoDS_Shape right_only =
                    boolean_shape<BRepAlgoAPI_Cut>(yeahright_box,
                                                   yeahright_boolean_source);
                BRep_Builder builder;
                TopoDS_Compound compound;
                builder.MakeCompound(compound);
                builder.Add(compound, left_only);
                builder.Add(compound, right_only);
                total += measured(compound, yeahright_box_input);
                return total;
              });
  const auto yeahright_stress_soup = csgrs_bench::read_obj_triangle_soup(
      csgrs_bench::yeahright_boolean_proxy_path());
  const TopoDS_Shape yeahright_stress_source =
      solid_from_obj_soup(yeahright_stress_soup);
  const TopoDS_Shape yeahright_copy =
      yeahright_boolean_operand(yeahright_stress_source);
  const std::size_t yeahright_boolean_input =
      yeahright_stress_soup.triangles.size() * 2;
  harness.run("stress", "boolean_union",
              "yeahright_genus131_proxy_rot90_offset", 1,
              [&] {
                return measured(
                    boolean_shape<BRepAlgoAPI_Fuse>(yeahright_stress_source,
                                                    yeahright_copy),
                    yeahright_boolean_input);
              });
  harness.run("stress", "boolean_difference",
              "yeahright_genus131_proxy_rot90_offset", 1, [&] {
                return measured(
                    boolean_shape<BRepAlgoAPI_Cut>(yeahright_stress_source,
                                                   yeahright_copy),
                    yeahright_boolean_input);
              });
  harness.run("stress", "boolean_intersection",
              "yeahright_genus131_proxy_rot90_offset", 1, [&] {
                return measured(
                    boolean_shape<BRepAlgoAPI_Common>(yeahright_stress_source,
                                                      yeahright_copy),
                    yeahright_boolean_input);
              });
  harness.run("stress", "boolean_xor",
              "yeahright_genus131_proxy_rot90_offset", 1,
              [&] {
                const TopoDS_Shape left_only =
                    boolean_shape<BRepAlgoAPI_Cut>(yeahright_stress_source,
                                                   yeahright_copy);
                const TopoDS_Shape right_only =
                    boolean_shape<BRepAlgoAPI_Cut>(yeahright_copy,
                                                   yeahright_stress_source);
                BRep_Builder builder;
                TopoDS_Compound compound;
                builder.MakeCompound(compound);
                builder.Add(compound, left_only);
                builder.Add(compound, right_only);
                return measured(compound, yeahright_boolean_input);
              });
  const TopoDS_Shape yeahright_dangerous_copy =
      yeahright_boolean_operand(yeahright_source);
  harness.run("dangerous", "boolean_intersection",
              "yeahright_control_full_rot90_offset_dangerous", 1, [&] {
                return measured(
                    boolean_shape<BRepAlgoAPI_Common>(
                        yeahright_source, yeahright_dangerous_copy),
                    yeahright_input * 2);
              });

  harness.run("kernel", "stl_write", "sphere_medium", 8, [&] {
    TopoDS_Shape output = analysis_mesh_source;
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "csgrs-occt-benchmark.stl";
    StlAPI_Writer writer;
    writer.ASCIIMode() = Standard_False;
    if (!writer.Write(output, path.string().c_str())) {
      throw std::runtime_error("OCCT STL comparison workload failed");
    }
    const std::size_t size = std::filesystem::file_size(path);
    std::filesystem::remove(path);
    return Measurement{960, size, size};
  });
}
