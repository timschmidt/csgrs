//! Broad native-carrier coverage for end-to-end optimization anchors.

#[path = "../support/harness.rs"]
mod support;

use std::hint::black_box;
use std::io::Cursor;
use std::num::NonZeroU32;

use csgrs::adapter::{RawReal, ScalarMesh};
use csgrs::parts::{
    BlueprintProjection, CsgPartInterface, PartMetadata, PartSource, blueprint_from_aabb_parts,
};
use csgrs::solid::MetaBall;
use csgrs::{
    AttributedMesh, GeometryContext, Real, TriangleMesh, curve,
    curve::CurveRegionExt,
    solid::{self, SolidExt},
};
use hypercurve::{CurvePolicy, CurveRegion2, Point2};
use hyperlattice::{Matrix4, Point3, Vector3};
use hypersdf::SdfExpr;
use image::{GrayImage, Luma};
use support::{Config, Measurement};

fn mesh_measurement(mesh: &TriangleMesh, work_units: usize) -> Measurement {
    Measurement::new(
        work_units as u64,
        mesh.triangles.len() as u64,
        (mesh.triangles.len() as u64).rotate_left(17) ^ mesh.positions.len() as u64,
    )
}

fn compound_region(
    result: Result<csgrs::GeometryOutcome<CurveRegion2>, csgrs::errors::CurveBooleanError>,
) -> CurveRegion2 {
    result.expect("compound curve Boolean").into_value()
}

fn boolean_results(
    left: &TriangleMesh,
    right: &TriangleMesh,
    count: usize,
) -> Vec<TriangleMesh> {
    let mut results = Vec::with_capacity(count);
    if count >= 1 {
        results.push(left.try_union(right).expect("union"));
    }
    if count >= 2 {
        results.push(left.try_difference(right).expect("difference"));
    }
    if count >= 3 {
        results.push(left.try_intersection(right).expect("intersection"));
    }
    if count >= 4 {
        results.push(left.try_xor(right).expect("xor"));
    }
    results
}

fn boolean_measurement(results: &[TriangleMesh]) -> Measurement {
    let triangles = results.iter().map(|mesh| mesh.triangles.len()).sum::<usize>();
    let positions = results.iter().map(|mesh| mesh.positions.len()).sum::<usize>();
    Measurement::new(
        results.len() as u64,
        triangles as u64,
        ((triangles as u64) << 32) ^ positions as u64,
    )
}

fn curve_measurement(region: &CurveRegion2, work_units: usize) -> Measurement {
    Measurement::new(work_units as u64, region.len() as u64, region.len() as u64)
}

fn main() {
    support::print_header();
    let config = Config::from_env();

    config.run("feature", "exact_scalar", "hyperreal_expression", 64, || {
        let value = (Real::from(7_u8) / Real::from(3_u8))
            .expect("nonzero denominator")
            .sqrt()
            .expect("positive radicand")
            .sin();
        Measurement::new(4, 1, value.to_f64_lossy().unwrap_or_default().to_bits())
    });

    config.run("feature", "mesh_primitives", "catalog", 4, || {
        let points = [
            [Real::zero(), Real::zero(), Real::zero()],
            [Real::from(2), Real::zero(), Real::zero()],
            [Real::zero(), Real::from(2), Real::zero()],
            [Real::zero(), Real::zero(), Real::from(2)],
        ];
        let faces: [&[usize]; 4] = [&[0, 2, 1], &[0, 1, 3], &[0, 3, 2], &[1, 2, 3]];
        let meshes = [
            solid::cuboid(Real::from(2), Real::from(3), Real::from(5)),
            solid::cube(Real::from(4)),
            solid::sphere(Real::from(4), 16, 8),
            solid::cylinder(Real::from(3), Real::from(8), 32),
            solid::frustum(Real::from(3), Real::from(2), Real::from(8), 16),
            solid::polyhedron(&points, &faces).expect("tetrahedron"),
            solid::torus(Real::from(8), Real::from(2), 32, 12),
            solid::ellipsoid(Real::from(3), Real::from(5), Real::from(7), 24, 12),
            solid::arrow(
                Point3::origin(),
                Vector3::from_xyz(Real::from(2), Real::from(3), Real::from(4)),
                12,
                false,
            ),
            solid::octahedron(Real::from(4)),
            solid::icosahedron(Real::from(4)),
            curve::try_extrude(
                &curve::involute_gear(
                    Real::from(2),
                    12,
                    Real::from(20),
                    Real::zero(),
                    Real::zero(),
                    4,
                ),
                Real::from(2),
                &GeometryContext::STRICT,
            )
            .expect("involute extrusion")
            .into_value(),
            curve::try_extrude(
                &curve::cycloidal_gear(Real::from(2), 12, Real::from(2), Real::zero(), 4),
                Real::from(2),
                &GeometryContext::STRICT,
            )
            .expect("cycloidal extrusion")
            .into_value(),
        ];
        let triangles = meshes.iter().map(|mesh| mesh.triangles.len()).sum::<usize>();
        Measurement::new(meshes.len() as u64, triangles as u64, triangles as u64)
    });

    config.run("feature", "profile_primitives", "catalog", 8, || {
        let polygon = [
            [Real::zero(), Real::zero()],
            [Real::from(4), Real::zero()],
            [Real::from(2), Real::from(3)],
        ];
        let regions = [
            curve::rectangle(Real::from(12), Real::from(8)),
            curve::square(Real::from(8)),
            curve::circle(Real::from(4), 24),
            curve::right_triangle(Real::from(6), Real::from(4)),
            curve::polygon(&polygon),
            curve::ellipse(Real::from(8), Real::from(4), 24),
            curve::regular_ngon(7, Real::from(4)),
            curve::arrow(Real::from(6), Real::from(2), Real::from(3), Real::from(4)),
            curve::rounded_rectangle(Real::from(12), Real::from(8), Real::from(2), 8),
            curve::star(12, Real::from(8), Real::from(4)),
            curve::teardrop(Real::from(6), Real::from(10), 24),
            curve::egg(Real::from(6), Real::from(10), 24),
            curve::squircle(Real::from(8), Real::from(6), 24),
            compound_region(curve::keyhole(
                Real::from(4),
                Real::from(2),
                Real::from(6),
                24,
                &GeometryContext::APPROXIMATE_512,
            )),
            compound_region(curve::reuleaux(
                3,
                Real::from(6),
                24,
                &GeometryContext::APPROXIMATE_512,
            )),
            compound_region(curve::ring(
                Real::from(6),
                Real::from(2),
                24,
                &GeometryContext::APPROXIMATE_512,
            )),
            curve::heart(Real::from(8), Real::from(8), 32),
            compound_region(curve::crescent(
                Real::from(6),
                Real::from(4),
                Real::from(3),
                24,
                &GeometryContext::APPROXIMATE_512,
            )),
            curve::involute_gear(
                Real::from(2),
                20,
                Real::from(20),
                Real::zero(),
                Real::zero(),
                4,
            ),
            curve::airfoil_naca4(
                Real::from(2),
                Real::from(4),
                Real::from(12),
                Real::from(20),
                80,
            ),
        ];
        let paths = usize::from(
            curve::bezier_path(
                &[
                    [Real::zero(), Real::zero()],
                    [Real::one(), Real::from(2)],
                    [Real::from(3), Real::zero()],
                ],
                16,
            )
            .is_some(),
        );
        let contours = regions.iter().map(CurveRegion2::len).sum::<usize>();
        Measurement::new(
            (regions.len() + paths) as u64,
            (contours + paths) as u64,
            ((contours as u64) << 32) ^ paths as u64,
        )
    });

    let curve_left = curve::circle(Real::from(10), 64);
    let curve_right = curve::transformed(
        &curve::square(Real::from(12)),
        &Matrix4::affine_translation([Real::from(4), Real::zero(), Real::zero()]),
    );
    config.run("feature", "profile_boolean", "all_operations", 2, || {
        let results = [
            curve_left
                .try_union(&curve_right, &CurvePolicy::STRICT)
                .expect("union")
                .into_value(),
            curve_left
                .try_difference(&curve_right, &CurvePolicy::STRICT)
                .expect("difference")
                .into_value(),
            curve_left
                .try_intersection(&curve_right, &CurvePolicy::STRICT)
                .expect("intersection")
                .into_value(),
            curve_left
                .try_xor(&curve_right, &CurvePolicy::STRICT)
                .expect("xor")
                .into_value(),
        ];
        let contours = results.iter().map(CurveRegion2::len).sum::<usize>();
        Measurement::new(128, contours as u64, contours as u64)
    });
    config.run("feature", "profile_triangulate", "circle_64", 8, || {
        let triangles =
            curve::try_triangulate(black_box(&curve_left), &GeometryContext::STRICT)
                .expect("triangulation")
                .into_value()
                .triangles
                .len();
        Measurement::new(64, triangles as u64, triangles as u64)
    });
    config.run("feature", "profile_offset", "sharp_and_round", 2, || {
        let sharp = curve::offset(
            black_box(&curve_left),
            Real::one(),
            &hypercurve::CurvePolicy::STRICT,
        )
        .expect("offset")
        .into_value();
        let rounded = curve::offset_rounded(
            black_box(&curve_left),
            Real::one(),
            &hypercurve::CurvePolicy::STRICT,
        )
        .expect("rounded offset")
        .into_value();
        Measurement::new(128, (sharp.len() + rounded.len()) as u64, sharp.len() as u64)
    });
    config.run("feature", "profile_transform", "all_csg_helpers", 4, || {
        let source = curve::rectangle(Real::from(8), Real::from(5));
        let outputs = [
            curve::transformed(
                &source,
                &Matrix4::affine_translation([Real::from(3), Real::from(-2), Real::zero()]),
            ),
            source
                .transformed_affine(
                    &Real::zero(),
                    &Real::from(-1),
                    &Real::one(),
                    &Real::zero(),
                    &Real::zero(),
                    &Real::zero(),
                    &CurvePolicy::STRICT,
                )
                .expect("rotation"),
            source
                .transformed_affine(
                    &Real::from(2),
                    &Real::zero(),
                    &Real::zero(),
                    &Real::from(3),
                    &Real::zero(),
                    &Real::zero(),
                    &CurvePolicy::STRICT,
                )
                .expect("scale"),
            curve::transformed(
                &source,
                &Matrix4::affine_translation([Real::from(7), Real::from(-4), Real::zero()]),
            ),
        ];
        let contours = outputs.iter().map(CurveRegion2::len).sum::<usize>();
        Measurement::new(8, contours as u64, contours as u64)
    });
    config.run(
        "feature",
        "profile_distribution",
        "arc_linear_grid",
        1,
        || {
            let source = curve::square(Real::one());
            let mut regions = Vec::new();
            for index in 0..14 {
                regions.push(curve::transformed(
                    &source,
                    &Matrix4::affine_translation([
                        Real::from((index % 4) as u64) * Real::from(3),
                        Real::from((index / 4) as u64) * Real::from(3),
                        Real::zero(),
                    ]),
                ));
            }
            let contours = regions.iter().map(CurveRegion2::len).sum::<usize>();
            Measurement::new(14, contours as u64, contours as u64)
        },
    );
    config.run(
        "feature",
        "profile_to_mesh",
        "extrude_revolve_twist_sweep_loft",
        1,
        || {
            let extrusion =
                curve::try_extrude(&curve_left, Real::from(10), &GeometryContext::STRICT)
                    .expect("extrusion")
                    .into_value();
            let radial = curve::transformed(
                &curve::rectangle(Real::from(3), Real::from(8)),
                &Matrix4::affine_translation([Real::from(5), Real::zero(), Real::zero()]),
            );
            let revolution =
                curve::revolve(&radial, Real::from(360), 32, &GeometryContext::STRICT)
                    .expect("revolution")
                    .into_value();
            let twist = curve::extrude_twisted(
                &curve_right,
                Real::from(12),
                Real::from(90),
                [Real::one(), Real::one()],
                16,
                &GeometryContext::STRICT,
            )
            .expect("twisted extrusion")
            .into_value();
            let sweep = curve::try_sweep(
                &curve::circle(Real::one(), 24),
                &[
                    Point3::origin(),
                    Point3::new(Real::zero(), Real::zero(), Real::from(4)),
                    Point3::new(Real::from(3), Real::zero(), Real::from(8)),
                ],
                &GeometryContext::STRICT,
            )
            .expect("sweep")
            .into_value();
            let loft = solid::loft(&[square_loop(0, 2), square_loop(8, 4)]).expect("loft");
            let triangles = [&extrusion, &revolution, &twist, &sweep, &loft]
                .into_iter()
                .map(|mesh| mesh.triangles.len())
                .sum::<usize>();
            Measurement::new(5, triangles as u64, triangles as u64)
        },
    );

    let mesh = solid::sphere(Real::from(8), 24, 12);
    let positioned = solid::cuboid(Real::from(2), Real::from(3), Real::from(5)).translated(
        Real::from(7),
        Real::from(-4),
        Real::from(-9),
    );
    config.run(
        "feature",
        "mesh_positioning",
        "center_float_vector",
        8,
        || {
            let outputs = [
                solid::center(&positioned),
                solid::float(&positioned),
                positioned.translated(Real::from(3), Real::from(-2), Real::from(5)),
            ];
            let triangles = outputs.iter().map(|mesh| mesh.triangles.len()).sum::<usize>();
            Measurement::new(
                3 * positioned.triangles.len() as u64,
                triangles as u64,
                triangles as u64,
            )
        },
    );
    config.run("feature", "mesh_distribution", "arc_linear_grid", 1, || {
        let source = solid::cube(Real::one());
        let copies = (0..22)
            .map(|index| {
                source.translated(
                    Real::from((index % 6) as u64) * Real::from(3),
                    Real::from((index / 6) as u64) * Real::from(3),
                    Real::zero(),
                )
            })
            .collect::<Vec<_>>();
        mesh_measurement(&solid::merge(&copies), 22)
    });
    config.run(
        "feature",
        "mesh_refinement",
        "subdivide_and_smooth",
        1,
        || {
            let subdivided = solid::subdivide(
                black_box(&mesh),
                NonZeroU32::new(1).expect("one is nonzero"),
            )
            .expect("valid subdivision");
            let first = subdivided
                .laplacian_smooth(&Real::try_from(0.4).unwrap(), 2)
                .expect("valid smoothing");
            let smoothed = first
                .laplacian_smooth(&Real::try_from(-0.41).unwrap(), 2)
                .expect("valid smoothing");
            mesh_measurement(&smoothed, mesh.triangles.len())
        },
    );
    config.run("feature", "mesh_topology", "connectivity_manifold", 4, || {
        let (vertices, _) = black_box(&mesh)
            .connectivity_counts()
            .expect("valid connectivity");
        Measurement::new(
            mesh.triangles.len() as u64,
            vertices as u64,
            vertices as u64 ^ u64::from(mesh.is_closed_manifold()),
        )
    });
    config.run("feature", "mesh_queries", "graphics_buffers", 8, || {
        let graphics = ScalarMesh::<RawReal>::from_native(mesh.clone())
            .graphics_mesh()
            .expect("graphics");
        Measurement::new(
            mesh.triangles.len() as u64,
            graphics.indices.len() as u64,
            ((graphics.vertices.len() as u64) << 32) ^ graphics.indices.len() as u64,
        )
    });
    config.run("feature", "mesh_queries", "ray_mass_graphics", 2, || {
        let hits = solid::ray_intersections(
            &mesh,
            &Point3::new(Real::from(-20), Real::zero(), Real::zero()),
            &Vector3::x(),
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("policy-authorized ray intersections")
        .value;
        let mass = solid::exact_mass_properties(&mesh, Real::one()).expect("mass");
        let graphics = ScalarMesh::<RawReal>::from_native(mesh.clone())
            .graphics_mesh()
            .expect("graphics");
        Measurement::new(
            mesh.triangles.len() as u64,
            graphics.indices.len() as u64,
            hits.len() as u64 ^ mass.mass.to_f64_lossy().unwrap_or_default().to_bits(),
        )
    });
    config.run("feature", "mesh_queries", "ray_intersections", 4, || {
        let hits = solid::ray_intersections(
            &mesh,
            &Point3::new(Real::from(-20), Real::zero(), Real::zero()),
            &Vector3::x(),
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("policy-authorized ray intersections")
        .value;
        Measurement::new(
            mesh.triangles.len() as u64,
            hits.len() as u64,
            hits.len() as u64,
        )
    });
    config.run("feature", "mesh_queries", "mass_properties", 4, || {
        let report = solid::exact_mass_properties(&mesh, Real::one()).expect("mass");
        Measurement::new(
            mesh.triangles.len() as u64,
            10,
            report.mass.to_f64_lossy().unwrap_or_default().to_bits(),
        )
    });
    config.run(
        "feature",
        "mesh_profile_projection",
        "slice_and_flatten",
        1,
        || {
            let (region, strings, paths) = solid::slice_z(&mesh, Real::zero());
            let flattened = solid::flatten(&mesh);
            let output = region.len() + strings.len() + paths.len() + flattened.len();
            Measurement::new(mesh.triangles.len() as u64, output as u64, output as u64)
        },
    );
    config.run("feature", "mesh_profile_projection", "slice", 1, || {
        let (region, strings, paths) = solid::slice_z(&mesh, Real::zero());
        let output = region.len() + strings.len() + paths.len();
        Measurement::new(mesh.triangles.len() as u64, output as u64, output as u64)
    });
    config.run("feature", "mesh_profile_projection", "flatten", 1, || {
        let output = solid::flatten(&mesh).len();
        Measurement::new(mesh.triangles.len() as u64, output as u64, output as u64)
    });
    config.run("feature", "hypermesh", "buffers_hull_minkowski", 1, || {
        let hull = solid::convex_hull(&mesh).expect("hull");
        let sum =
            solid::minkowski_sum(&solid::cube(Real::from(2)), &solid::cube(Real::from(3)))
                .expect("Minkowski");
        let output = mesh.triangles.len() * 3 + hull.triangles.len() + sum.triangles.len();
        Measurement::new(mesh.triangles.len() as u64, output as u64, output as u64)
    });
    config.run("feature", "hypermesh", "adapter_buffers", 1, || {
        Measurement::new(
            mesh.triangles.len() as u64,
            (mesh.triangles.len() * 3) as u64,
            (mesh.positions.len() * 3) as u64,
        )
    });
    config.run("feature", "hypermesh", "convex_hull", 1, || {
        mesh_measurement(
            &solid::convex_hull(&mesh).expect("hull"),
            mesh.triangles.len(),
        )
    });
    config.run("feature", "hypermesh", "cube_minkowski", 1, || {
        let sum =
            solid::minkowski_sum(&solid::cube(Real::from(2)), &solid::cube(Real::from(3)))
                .expect("Minkowski");
        mesh_measurement(&sum, 16)
    });

    let boolean_left = solid::cube(Real::from(4));
    let boolean_right =
        solid::cube(Real::from(4)).translated(Real::one(), Real::one(), Real::one());
    config.run("feature", "mesh_boolean", "immediate_four", 1, || {
        boolean_measurement(&boolean_results(&boolean_left, &boolean_right, 4))
    });
    for count in 1..=4 {
        config.run(
            "feature",
            "mesh_boolean_count",
            &format!("immediate_{count}"),
            1,
            || boolean_measurement(&boolean_results(&boolean_left, &boolean_right, count)),
        );
    }
    let fixtures = [
        (
            "disjoint",
            solid::cube(Real::from(4)),
            solid::cube(Real::from(4)).translated(Real::from(10), Real::zero(), Real::zero()),
        ),
        (
            "identical",
            solid::cube(Real::from(4)),
            solid::cube(Real::from(4)),
        ),
        (
            "contained",
            solid::cube(Real::from(4)),
            solid::cube(Real::from(2)),
        ),
        (
            "face_touching",
            solid::cube(Real::from(4)),
            solid::cube(Real::from(4)).translated(Real::from(4), Real::zero(), Real::zero()),
        ),
    ];
    for (name, left, right) in &fixtures {
        config.run(
            "feature",
            "mesh_boolean_fixtures",
            &format!("{name}_immediate_four"),
            1,
            || boolean_measurement(&boolean_results(left, right, 4)),
        );
    }

    let sdf_min = Point3::new(Real::from(-3), Real::from(-3), Real::from(-3));
    let sdf_max = Point3::new(Real::from(3), Real::from(3), Real::from(3));
    config.run("feature", "implicit", "hypersdf_surface_nets", 1, || {
        let expression = SdfExpr::sphere(
            hyperlimit::Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Real::from(4),
        );
        mesh_measurement(
            &solid::sdf_expr(
                expression,
                (20, 20, 20),
                sdf_min.clone(),
                sdf_max.clone(),
                Real::zero(),
            ),
            20 * 20 * 20,
        )
    });
    config.run("feature", "implicit", "metaballs_3d", 1, || {
        let balls = [
            MetaBall::new(Point3::origin(), Real::from(2)),
            MetaBall::new(
                Point3::new(Real::from(2), Real::zero(), Real::zero()),
                Real::from(2),
            ),
        ];
        mesh_measurement(
            &solid::metaballs(&balls, (20, 20, 20), Real::one(), Real::one()),
            20 * 20 * 20,
        )
    });
    config.run("feature", "implicit", "metaballs_2d", 1, || {
        let balls = [
            (Point2::new(Real::zero(), Real::zero()), Real::from(2)),
            (Point2::new(Real::from(2), Real::zero()), Real::from(2)),
        ];
        curve_measurement(
            &curve::metaballs(&balls, (48, 48), Real::one(), Real::one()),
            48 * 48,
        )
    });
    config.run("feature", "implicit", "tpms_catalog", 1, || {
        let bounds = solid::cube(Real::from(12));
        let gyroid = solid::gyroid(&bounds, 18, Real::from(6), Real::zero());
        let schwarz = solid::schwarz_p(&bounds, 18, Real::from(6), Real::zero());
        let triangles = gyroid.triangles.len() + schwarz.triangles.len();
        Measurement::new(2 * 18 * 18 * 18, triangles as u64, triangles as u64)
    });
    config.run("feature", "raster_vector", "image_contours", 2, || {
        let image = GrayImage::from_fn(64, 64, |x, y| {
            let dx = i64::from(x) - 32;
            let dy = i64::from(y) - 32;
            Luma([u8::from(dx * dx + dy * dy < 24 * 24) * 255])
        });
        curve_measurement(
            &curve::from_image(&image, 128).expect("image contours"),
            64 * 64,
        )
    });
    config.run("feature", "text", "truetype_outline", 2, || {
        curve_measurement(
            &curve::truetype_text(
                "csgrs benchmark",
                include_bytes!("../../asar.ttf"),
                Real::from(24),
            ),
            15,
        )
    });
    config.run("feature", "text", "hershey_strokes", 8, || {
        let strings =
            curve::hershey_strings("CSGRS", &curve::hershey::fonts::FUTURAL, Real::from(2));
        Measurement::new(4, strings.len() as u64, strings.len() as u64)
    });
    config.run("feature", "adapter", "bevy_mesh", 4, || {
        black_box(solid::to_bevy_mesh(black_box(&mesh)));
        Measurement::new(mesh.triangles.len() as u64, 1, 1)
    });

    let parts = (0..32)
        .map(|index| {
            let handle = format!("part-{index}");
            let metadata = PartMetadata::new(
                &handle,
                CsgPartInterface::exact_csg(
                    "benchmark",
                    &handle,
                    PartSource {
                        family: "benchmark".into(),
                        revision: "local".into(),
                    },
                ),
            );
            let geometry = solid::cube(Real::from(2)).translated(
                Real::from(index % 8) * Real::from(3),
                Real::from(index / 8) * Real::from(3),
                Real::from(index % 3),
            );
            AttributedMesh::from_uniform(geometry, metadata)
        })
        .collect::<Vec<_>>();
    config.run("feature", "parts", "blueprint_aabb", 4, || {
        let report = blueprint_from_aabb_parts(&parts, BlueprintProjection::Front, false);
        let edges = report.assembled.edges.len() + report.exploded.edges.len();
        Measurement::new(parts.len() as u64, edges as u64, edges as u64)
    });

    let io_mesh = solid::sphere(Real::from(4), 20, 10);
    config.run("feature", "mesh_io", "all_exporters", 1, || {
        let stl = csgrs::io::stl::to_stl_binary(&io_mesh, "benchmark").expect("STL");
        let dxf = csgrs::io::dxf::to_dxf(&io_mesh).expect("DXF");
        let obj = csgrs::io::obj::to_obj(&io_mesh, "benchmark").expect("OBJ");
        let ply = csgrs::io::ply::to_ply(&io_mesh, "benchmark").expect("PLY");
        let amf = csgrs::io::amf::to_amf(&io_mesh, "benchmark", "millimeter").expect("AMF");
        let gltf = csgrs::io::gltf::to_gltf(&io_mesh, "benchmark").expect("glTF");
        let size = stl.len() + dxf.len() + obj.len() + ply.len() + amf.len() + gltf.len();
        Measurement::new(io_mesh.triangles.len() as u64, size as u64, size as u64)
    });
    config.run("feature", "mesh_io", "public_writer_exporters", 1, || {
        let mut obj = Vec::new();
        csgrs::io::obj::write_obj(&io_mesh, &mut obj, "benchmark").expect("OBJ");
        let mut ply = Vec::new();
        csgrs::io::ply::write_ply(&io_mesh, &mut ply, "benchmark").expect("PLY");
        let mut amf = Vec::new();
        csgrs::io::amf::write_amf(&io_mesh, &mut amf, "benchmark", "millimeter").expect("AMF");
        let mut gltf = Vec::new();
        csgrs::io::gltf::write_gltf(&io_mesh, &mut gltf, "benchmark").expect("glTF");
        let size = obj.len() + ply.len() + amf.len() + gltf.len();
        Measurement::new(io_mesh.triangles.len() as u64, size as u64, size as u64)
    });
    writer_case(&config, "writer_obj", &io_mesh, |output| {
        csgrs::io::obj::write_obj(&io_mesh, output, "benchmark").expect("OBJ")
    });
    writer_case(&config, "writer_ply", &io_mesh, |output| {
        csgrs::io::ply::write_ply(&io_mesh, output, "benchmark").expect("PLY")
    });
    writer_case(&config, "writer_amf", &io_mesh, |output| {
        csgrs::io::amf::write_amf(&io_mesh, output, "benchmark", "millimeter").expect("AMF")
    });
    writer_case(&config, "writer_gltf", &io_mesh, |output| {
        csgrs::io::gltf::write_gltf(&io_mesh, output, "benchmark").expect("glTF")
    });
    let stl = csgrs::io::stl::to_stl_binary(&io_mesh, "benchmark").expect("STL fixture");
    let obj = csgrs::io::obj::to_obj(&io_mesh, "benchmark").expect("OBJ fixture");
    config.run("feature", "mesh_io", "roundtrip_importers", 1, || {
        let from_stl = csgrs::io::stl::from_stl(&stl).expect("STL import");
        let from_obj =
            csgrs::io::obj::from_obj(Cursor::new(obj.as_bytes())).expect("OBJ import");
        let triangles = from_stl.triangles.len() + from_obj.triangles.len();
        Measurement::new(
            (stl.len() + obj.len()) as u64,
            triangles as u64,
            triangles as u64,
        )
    });
    let io_curve = compound_region(curve::ring(
        Real::from(8),
        Real::from(2),
        48,
        &GeometryContext::APPROXIMATE_512,
    ));
    config.run("feature", "profile_io", "svg_gerber_roundtrip", 1, || {
        let svg = csgrs::io::svg::export_svg(&io_curve, &[], &[]).expect("SVG");
        let gerber = csgrs::io::gerber::export_gerber(&io_curve).expect("Gerber");
        let (svg_region, _, _) = csgrs::io::svg::import_svg(&svg).expect("SVG import");
        let (gerber_region, _, _) =
            csgrs::io::gerber::import_gerber(&gerber).expect("Gerber import");
        let output = svg_region.len() + gerber_region.len();
        Measurement::new(
            (svg.len() + gerber.len()) as u64,
            output as u64,
            output as u64,
        )
    });
}

fn square_loop(z: i64, half_width: i64) -> Vec<Point3> {
    [
        (-half_width, -half_width),
        (half_width, -half_width),
        (half_width, half_width),
        (-half_width, half_width),
    ]
    .into_iter()
    .map(|(x, y)| Point3::new(Real::from(x), Real::from(y), Real::from(z)))
    .collect()
}

fn writer_case(
    config: &Config,
    case: &str,
    mesh: &TriangleMesh,
    mut write: impl FnMut(&mut Vec<u8>),
) {
    config.run("feature", "mesh_io", case, 1, || {
        let mut output = Vec::new();
        write(&mut output);
        Measurement::new(
            mesh.triangles.len() as u64,
            output.len() as u64,
            output.len() as u64,
        )
    });
}
