#include "common.hpp"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

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

  const TopoDS_Shape analysis_source = BRepPrimAPI_MakeSphere(10.0).Shape();
  harness.run("kernel", "triangulate", "sphere_medium", 16, [&] {
    TopoDS_Shape output = analysis_source;
    triangulate(output);
    return measured(output, 960, false);
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
}
