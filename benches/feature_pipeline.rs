//! Broad csgrs/Hyper-stack coverage for end-to-end optimization anchors.

mod support;

use std::hint::black_box;
use std::io::Cursor;
use std::num::NonZeroU32;

use csgrs::io::gerber::{FromGerber, ToGerber};
use csgrs::io::svg::{FromSVG, ToSVG};
use csgrs::mesh::metaballs::MetaBall;
use csgrs::mesh::plane::Plane;
use csgrs::parts::{
    BlueprintProjection, CsgPartInterface, PartMetadata, PartSource, blueprint_from_aabb_parts,
};
use csgrs::{
    Real,
    csg::CSG,
    mesh::Mesh,
    polygon_mesh::{Polygon, PolygonMesh},
    sketch::Profile,
    vertex::Vertex,
};
use hypercurve::Point2;
use hyperlattice::{Matrix4, Point3, Vector3};
use hypersdf::SdfExpr;
use image::{GrayImage, Luma};
use support::{Config, Measurement};

fn mesh_measurement(mesh: &Mesh<()>, work_units: usize) -> Measurement {
    let corners = mesh
        .triangles()
        .iter()
        .map(|polygon| polygon.vertices().len())
        .sum::<usize>();
    Measurement::new(
        work_units as u64,
        mesh.triangles().len() as u64,
        (mesh.triangles().len() as u64).rotate_left(17) ^ corners as u64,
    )
}

fn direct_boolean_results(left: &Mesh<()>, right: &Mesh<()>, count: usize) -> Vec<Mesh<()>> {
    let mut results = Vec::with_capacity(count);
    if count >= 1 {
        results.push(left.try_union(right).unwrap());
    }
    if count >= 2 {
        results.push(left.try_difference(right).unwrap());
    }
    if count >= 3 {
        results.push(left.try_intersection(right).unwrap());
    }
    if count >= 4 {
        results.push(left.try_xor(right).unwrap());
    }
    results
}

fn prepared_boolean_results(
    prepared: &csgrs::mesh::hypermesh::PreparedMeshBoolean<'_, ()>,
    count: usize,
) -> Vec<Mesh<()>> {
    let mut results = Vec::with_capacity(count);
    if count >= 1 {
        results.push(prepared.try_union().unwrap());
    }
    if count >= 2 {
        results.push(prepared.try_difference().unwrap());
    }
    if count >= 3 {
        results.push(prepared.try_intersection().unwrap());
    }
    if count >= 4 {
        results.push(prepared.try_xor().unwrap());
    }
    results
}

fn boolean_measurement(results: &[Mesh<()>]) -> Measurement {
    let polygons = results
        .iter()
        .map(|result| result.triangles().len())
        .sum::<usize>();
    let corners = results
        .iter()
        .flat_map(Mesh::triangles)
        .map(|polygon| polygon.vertices().len())
        .sum::<usize>();
    Measurement::new(
        results.len() as u64,
        polygons as u64,
        ((polygons as u64) << 32) ^ corners as u64,
    )
}

fn profile_measurement(profile: &Profile, work_units: usize) -> Measurement {
    let contours = profile.material_contour_count() + profile.hole_contour_count();
    let wires = profile.wires().len() + profile.curve_paths().len();
    Measurement::new(
        work_units as u64,
        (contours + wires) as u64,
        ((contours as u64) << 32) ^ wires as u64,
    )
}

fn square_section(z: i64, half_width: i64) -> Polygon<()> {
    let vertices = [
        (-half_width, -half_width),
        (half_width, -half_width),
        (half_width, half_width),
        (-half_width, half_width),
    ]
    .into_iter()
    .map(|(x, y)| {
        Vertex::new(
            Point3::new(Real::from(x), Real::from(y), Real::from(z)),
            Vector3::z(),
        )
    })
    .collect();
    Polygon::new(vertices, ())
}

fn main() {
    run();
}

fn run() {
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
        let polyhedron_points = [
            [Real::from(0), Real::from(0), Real::from(0)],
            [Real::from(2), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(2), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(2)],
        ];
        let polyhedron_faces: [&[usize]; 4] = [&[0, 2, 1], &[0, 1, 3], &[0, 3, 2], &[1, 2, 3]];
        let meshes = [
            Mesh::cuboid(Real::from(2_u8), Real::from(3_u8), Real::from(5_u8), ()),
            Mesh::cube(Real::from(4_u8), ()),
            Mesh::sphere(Real::from(4_u8), 16, 8, ()),
            Mesh::cylinder(Real::from(3_u8), Real::from(8_u8), 32, ()),
            Mesh::frustum(Real::from(3_u8), Real::from(2_u8), Real::from(8_u8), 16, ()),
            Mesh::frustum_ptp(
                Point3::origin(),
                Point3::new(Real::from(2), Real::from(3), Real::from(8)),
                Real::from(3_u8),
                Real::from(2_u8),
                16,
                (),
            ),
            Mesh::polyhedron(&polyhedron_points, &polyhedron_faces, ())
                .expect("benchmark tetrahedron remains valid"),
            Mesh::egg(Real::from(4), Real::from(6), 12, 16, ()),
            Mesh::teardrop(Real::from(4), Real::from(6), 12, 16, ()),
            Mesh::teardrop_cylinder(Real::from(4), Real::from(6), Real::from(3), 16, ()),
            Mesh::torus(Real::from(8_u8), Real::from(2_u8), 32, 12, ()),
            Mesh::ellipsoid(
                Real::from(3_u8),
                Real::from(5_u8),
                Real::from(7_u8),
                24,
                12,
                (),
            ),
            Mesh::arrow(
                Point3::origin(),
                Vector3::from_xyz(Real::from(2), Real::from(3), Real::from(4)),
                12,
                false,
                (),
            ),
            Mesh::octahedron(Real::from(4_u8), ()),
            Mesh::icosahedron(Real::from(4_u8), ()),
            Mesh::spur_gear_involute(
                Real::from(2),
                12,
                Real::from(20),
                Real::zero(),
                Real::zero(),
                4,
                Real::from(2),
                (),
            ),
            Mesh::spur_gear_cycloid(
                Real::from(2),
                12,
                Real::from(1),
                Real::zero(),
                4,
                Real::from(2),
                (),
            ),
            Mesh::helical_involute_gear(
                Real::from(2),
                12,
                Real::from(20),
                Real::zero(),
                Real::zero(),
                4,
                Real::from(6),
                Real::from(20),
                4,
                (),
            ),
        ];
        let polygons = meshes
            .iter()
            .map(|mesh| mesh.triangles().len())
            .sum::<usize>();
        Measurement::new(meshes.len() as u64, polygons as u64, polygons as u64)
    });

    config.run("feature", "profile_primitives", "catalog", 8, || {
        let polygon = [
            [Real::from(0), Real::from(0)],
            [Real::from(4), Real::from(0)],
            [Real::from(2), Real::from(3)],
        ];
        let polygon_points = [
            Point2::new(Real::from(0), Real::from(0)),
            Point2::new(Real::from(4), Real::from(0)),
            Point2::new(Real::from(2), Real::from(3)),
        ];
        let bezier_control = [
            [Real::from(0), Real::from(0)],
            [Real::from(1), Real::from(2)],
            [Real::from(2), Real::from(2)],
            [Real::from(3), Real::from(0)],
        ];
        let profiles = [
            Profile::rectangle(Real::from(12), Real::from(8)),
            Profile::square(Real::from(8)),
            Profile::circle(Real::from(4), 24),
            Profile::right_triangle(Real::from(6), Real::from(4)),
            Profile::polygon(&polygon),
            Profile::polygon_points(&polygon_points),
            Profile::ellipse(Real::from(8), Real::from(4), 24),
            Profile::regular_ngon(7, Real::from(4)),
            Profile::arrow(Real::from(6), Real::from(2), Real::from(3), Real::from(4)),
            Profile::trapezoid(Real::from(4), Real::from(8), Real::from(4), Real::from(2)),
            Profile::rounded_rectangle(
                Real::from(12_u8),
                Real::from(8_u8),
                Real::from(2_u8),
                8,
            ),
            Profile::star(12, Real::from(8_u8), Real::from(4_u8)),
            Profile::teardrop(Real::from(6), Real::from(10), 24),
            Profile::egg(Real::from(6), Real::from(10), 24),
            Profile::squircle(Real::from(8), Real::from(6), 24),
            Profile::keyhole(Real::from(4), Real::from(2), Real::from(6), 24),
            Profile::reuleaux(3, Real::from(6), 24),
            Profile::ring(Real::from(6), Real::from(2), 24),
            Profile::pie_slice(Real::from(4), Real::from(10), Real::from(100), 12),
            Profile::supershape(
                Real::from(1),
                Real::from(1),
                Real::from(5),
                Real::from(2),
                Real::from(2),
                Real::from(2),
                32,
            ),
            Profile::circle_with_keyway(Real::from(6), 24, Real::from(2), Real::from(2)),
            Profile::circle_with_flat(Real::from(6), 24, Real::from(2)),
            Profile::circle_with_two_flats(Real::from(6), 24, Real::from(2)),
            Profile::bezier(&bezier_control, 16),
            Profile::bspline(&bezier_control, 3, 8),
            Profile::heart(Real::from(8), Real::from(8), 32),
            Profile::crescent(Real::from(6), Real::from(4), Real::from(3), 24),
            Profile::involute_gear(
                Real::from(2_u8),
                20,
                Real::from(20_u8),
                Real::zero(),
                Real::zero(),
                4,
            ),
            Profile::cycloidal_gear(Real::from(2), 12, Real::from(1), Real::zero(), 4),
            Profile::involute_rack(
                Real::from(2),
                4,
                Real::from(20),
                Real::zero(),
                Real::zero(),
            ),
            Profile::cycloidal_rack(Real::from(2), 4, Real::zero(), 8),
            Profile::airfoil_naca4(
                Real::from(2_u8),
                Real::from(4_u8),
                Real::from(12_u8),
                Real::from(20_u8),
                80,
            ),
            Profile::square(Real::from(8)).hilbert_curve(3, Real::from(1)),
        ];
        let contours = profiles
            .iter()
            .map(|profile| profile.material_contour_count())
            .sum::<usize>();
        let wires = profiles
            .iter()
            .map(|profile| profile.wires().len() + profile.curve_paths().len())
            .sum::<usize>();
        Measurement::new(
            profiles.len() as u64,
            (contours + wires) as u64,
            ((contours as u64) << 32) ^ wires as u64,
        )
    });

    let profile_left = Profile::circle(Real::from(10_u8), 64);
    let profile_right = Profile::square(Real::from(12_u8)).translate(
        Real::from(4_u8),
        Real::zero(),
        Real::zero(),
    );
    config.run("feature", "profile_boolean", "all_operations", 2, || {
        let results = [
            profile_left
                .try_union(&profile_right)
                .expect("profile union remains certified"),
            profile_left
                .try_difference(&profile_right)
                .expect("profile difference remains certified"),
            profile_left
                .try_intersection(&profile_right)
                .expect("profile intersection remains certified"),
            profile_left
                .try_xor(&profile_right)
                .expect("profile xor remains certified"),
        ];
        let contours = results
            .iter()
            .map(|result| result.material_contour_count() + result.hole_contour_count())
            .sum::<usize>();
        Measurement::new(128, contours as u64, contours as u64)
    });
    config.run("feature", "profile_triangulate", "circle_64", 8, || {
        let triangles = black_box(&profile_left).triangulate();
        Measurement::new(64, triangles.len() as u64, triangles.len() as u64)
    });
    config.run("feature", "profile_offset", "sharp_and_round", 2, || {
        let sharp = black_box(&profile_left).offset(Real::one());
        let rounded = black_box(&profile_left).offset_rounded(Real::one());
        let size = sharp.material_contour_count() + rounded.material_contour_count();
        Measurement::new(128, size as u64, size as u64)
    });

    let profile_transform_source = Profile::rectangle(Real::from(8_u8), Real::from(5_u8))
        .translate(Real::from(7_u8), Real::from(-4_i8), Real::zero());
    let profile_affine = Matrix4::from_row_major([
        Real::one(),
        Real::one(),
        Real::zero(),
        Real::from(2_u8),
        Real::zero(),
        Real::one(),
        Real::zero(),
        Real::from(-3_i8),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ]);
    config.run("feature", "profile_transform", "all_csg_helpers", 4, || {
        let outputs = black_box([
            profile_transform_source.translate_vector(Vector3::new([
                Real::from(3_u8),
                Real::from(-2_i8),
                Real::zero(),
            ])),
            profile_transform_source.rotate(Real::zero(), Real::zero(), Real::from(37_u8)),
            profile_transform_source.scale(Real::from(2_u8), Real::from(3_u8), Real::one()),
            profile_transform_source.mirror(Plane::from_normal(Vector3::x(), Real::one())),
            profile_transform_source.center(),
            profile_transform_source.float(),
            profile_transform_source.transform(&profile_affine),
            profile_transform_source.inverse(),
        ]);
        let contours = outputs
            .iter()
            .map(|profile| profile.material_contour_count() + profile.hole_contour_count())
            .sum::<usize>();
        let wires = outputs
            .iter()
            .map(|profile| profile.wires().len() + profile.curve_paths().len())
            .sum::<usize>();
        Measurement::new(
            8,
            (contours + wires) as u64,
            ((contours as u64) << 32) ^ wires as u64,
        )
    });
    let profile_distribution_source = Profile::square(Real::one());
    config.run(
        "feature",
        "profile_distribution",
        "arc_linear_grid",
        1,
        || {
            let outputs = black_box([
                profile_distribution_source.distribute_arc(
                    4,
                    Real::from(5_u8),
                    Real::zero(),
                    Real::from(270_u16),
                ),
                profile_distribution_source.distribute_linear(
                    4,
                    Vector3::x(),
                    Real::from(3_u8),
                ),
                profile_distribution_source.distribute_grid(
                    2,
                    3,
                    Real::from(3_u8),
                    Real::from(3_u8),
                ),
            ]);
            let contours = outputs
                .iter()
                .map(|profile| profile.material_contour_count() + profile.hole_contour_count())
                .sum::<usize>();
            let wires = outputs
                .iter()
                .map(|profile| profile.wires().len() + profile.curve_paths().len())
                .sum::<usize>();
            Measurement::new(
                14,
                (contours + wires) as u64,
                ((contours as u64) << 32) ^ wires as u64,
            )
        },
    );

    config.run(
        "feature",
        "profile_to_mesh",
        "extrude_revolve_twist_sweep_loft",
        1,
        || {
            let extrusion = profile_left.extrude(Real::from(10_u8), ());
            let radial_profile = Profile::rectangle(Real::from(3_u8), Real::from(8_u8))
                .translate(Real::from(5_u8), Real::zero(), Real::zero());
            let revolution = radial_profile
                .revolve(Real::from(360_u16), 32, ())
                .expect("valid full revolution");
            let twist = profile_right
                .extrude_twisted(
                    Real::from(12_u8),
                    Real::from(90_u8),
                    [Real::one(), Real::one()],
                    16,
                    (),
                )
                .expect("valid twisted extrusion");
            let sweep = Profile::circle(Real::one(), 24).sweep(
                &[
                    Point3::origin(),
                    Point3::new(Real::zero(), Real::zero(), Real::from(4_u8)),
                    Point3::new(Real::from(3_u8), Real::zero(), Real::from(8_u8)),
                ],
                (),
            );
            let loft = PolygonMesh::loft(&[square_section(0, 2), square_section(8, 4)])
                .expect("valid corresponding loft sections")
                .triangulate();
            let polygons = extrusion.triangles().len()
                + revolution.triangles().len()
                + twist.triangles().len()
                + sweep.triangles().len()
                + loft.triangles().len();
            Measurement::new(5, polygons as u64, polygons as u64)
        },
    );

    let mesh = Mesh::sphere(Real::from(8_u8), 24, 12, ());
    let positioned_mesh =
        Mesh::cuboid(Real::from(2_u8), Real::from(3_u8), Real::from(5_u8), ()).translate(
            Real::from(7_u8),
            Real::from(-4_i8),
            Real::from(-9_i8),
        );
    config.run(
        "feature",
        "mesh_positioning",
        "center_float_vector",
        8,
        || {
            let outputs = black_box([
                positioned_mesh.center(),
                positioned_mesh.float(),
                positioned_mesh.translate_vector(Vector3::new([
                    Real::from(3_u8),
                    Real::from(-2_i8),
                    Real::from(5_u8),
                ])),
            ]);
            let polygons = outputs
                .iter()
                .map(|output| output.triangles().len())
                .sum::<usize>();
            Measurement::new(
                3 * positioned_mesh.triangles().len() as u64,
                polygons as u64,
                polygons as u64,
            )
        },
    );
    let distribution_source = Mesh::cube(Real::one(), ());
    config.run("feature", "mesh_distribution", "arc_linear_grid", 1, || {
        let arc = distribution_source.distribute_arc(
            12,
            Real::from(10_u8),
            Real::zero(),
            Real::from(330_u16),
        );
        let linear = distribution_source.distribute_linear(4, Vector3::x(), Real::from(3_u8));
        let grid =
            distribution_source.distribute_grid(2, 3, Real::from(3_u8), Real::from(3_u8));
        let polygons =
            arc.triangles().len() + linear.triangles().len() + grid.triangles().len();
        Measurement::new(22, polygons as u64, polygons as u64)
    });
    config.run(
        "feature",
        "mesh_refinement",
        "subdivide_and_smooth",
        1,
        || {
            let subdivided = black_box(&mesh).subdivide_triangles(NonZeroU32::new(1).unwrap());
            let smoothed = subdivided.taubin_smooth(
                Real::try_from(0.4_f64).unwrap(),
                Real::try_from(-0.41_f64).unwrap(),
                2,
                false,
            );
            mesh_measurement(&smoothed, mesh.triangles().len())
        },
    );
    config.run("feature", "mesh_topology", "connectivity_manifold", 4, || {
        let (vertices, adjacency) = black_box(&mesh).build_connectivity();
        let manifold = mesh.is_manifold();
        Measurement::new(
            mesh.triangles().len() as u64,
            adjacency.len() as u64,
            vertices.index_to_position.len() as u64 ^ u64::from(manifold),
        )
    });
    config.run("feature", "mesh_queries", "graphics_buffers", 8, || {
        let graphics = black_box(&mesh).build_graphics_mesh();
        Measurement::new(
            mesh.triangles().len() as u64,
            graphics.indices.len() as u64,
            ((graphics.vertices.len() as u64) << 32) ^ graphics.indices.len() as u64,
        )
    });
    config.run("feature", "mesh_queries", "ray_mass_graphics", 2, || {
        let hits = mesh.ray_intersections(
            &Point3::new(Real::from(-20_i8), Real::zero(), Real::zero()),
            &Vector3::x(),
        );
        let mass = mesh
            .exact_mass_properties(Real::one())
            .expect("closed sphere has mass properties");
        let graphics = mesh.build_graphics_mesh();
        Measurement::new(
            mesh.triangles().len() as u64,
            graphics.indices.len() as u64,
            hits.len() as u64 ^ mass.mass.to_f64_lossy().unwrap_or_default().to_bits(),
        )
    });
    config.run("feature", "mesh_queries", "ray_intersections", 4, || {
        let hits = mesh.ray_intersections(
            &Point3::new(Real::from(-20_i8), Real::zero(), Real::zero()),
            &Vector3::x(),
        );
        Measurement::new(
            mesh.triangles().len() as u64,
            hits.len() as u64,
            hits.iter().fold(0_u64, |checksum, (_, distance)| {
                checksum.rotate_left(7) ^ distance.to_f64_lossy().unwrap_or_default().to_bits()
            }),
        )
    });
    config.run("feature", "mesh_queries", "mass_properties", 4, || {
        let report = mesh
            .exact_mass_properties(Real::one())
            .expect("closed sphere has mass properties");
        Measurement::new(
            mesh.triangles().len() as u64,
            10,
            report.mass.to_f64_lossy().unwrap_or_default().to_bits()
                ^ report.center_of_mass.0[0]
                    .to_f64_lossy()
                    .unwrap_or_default()
                    .to_bits(),
        )
    });
    config.run(
        "feature",
        "mesh_profile_projection",
        "slice_and_flatten",
        1,
        || {
            let slice = mesh.slice(Plane::from_normal(Vector3::z(), Real::zero()));
            let flattened = mesh.flatten();
            let output = slice.wires().len()
                + slice.curve_paths().len()
                + flattened.material_contour_count();
            Measurement::new(mesh.triangles().len() as u64, output as u64, output as u64)
        },
    );
    config.run("feature", "mesh_profile_projection", "slice", 1, || {
        let slice = mesh.slice(Plane::from_normal(Vector3::z(), Real::zero()));
        let output =
            slice.wires().len() + slice.curve_paths().len() + slice.material_contour_count();
        Measurement::new(mesh.triangles().len() as u64, output as u64, output as u64)
    });
    config.run("feature", "mesh_profile_projection", "flatten", 1, || {
        let flattened = mesh.flatten();
        let output = flattened.material_contour_count();
        Measurement::new(mesh.triangles().len() as u64, output as u64, output as u64)
    });
    config.run("feature", "hypermesh", "buffers_hull_minkowski", 1, || {
        let buffers = mesh.to_hypermesh_buffers();
        let hull = mesh.convex_hull(());
        let sum = Mesh::cube(Real::from(2_u8), ())
            .minkowski_sum(&Mesh::cube(Real::from(3_u8), ()), ());
        let output = buffers.indices.len() + hull.triangles().len() + sum.triangles().len();
        Measurement::new(mesh.triangles().len() as u64, output as u64, output as u64)
    });
    config.run("feature", "hypermesh", "adapter_buffers", 1, || {
        let buffers = mesh.to_hypermesh_buffers();
        Measurement::new(
            mesh.triangles().len() as u64,
            buffers.indices.len() as u64,
            buffers.positions.len() as u64,
        )
    });
    config.run("feature", "hypermesh", "convex_hull", 1, || {
        let hull = mesh.convex_hull(());
        mesh_measurement(&hull, mesh.triangles().len())
    });
    config.run("feature", "hypermesh", "cube_minkowski", 1, || {
        let sum = Mesh::cube(Real::from(2_u8), ())
            .minkowski_sum(&Mesh::cube(Real::from(3_u8), ()), ());
        mesh_measurement(&sum, 16)
    });

    let boolean_left = Mesh::cube(Real::from(4_u8), ());
    let boolean_right =
        Mesh::cube(Real::from(4_u8), ()).translate(Real::one(), Real::one(), Real::one());
    config.run("feature", "mesh_boolean", "direct_four", 1, || {
        boolean_measurement(&direct_boolean_results(&boolean_left, &boolean_right, 4))
    });
    config.run(
        "feature",
        "mesh_boolean",
        "prepare_and_extract_four",
        1,
        || {
            let prepared = boolean_left.try_prepare_boolean(&boolean_right).unwrap();
            boolean_measurement(&prepared_boolean_results(&prepared, 4))
        },
    );
    let prepared_boolean = boolean_left.try_prepare_boolean(&boolean_right).unwrap();
    config.run("feature", "mesh_boolean", "extract_four_prebuilt", 1, || {
        boolean_measurement(&prepared_boolean_results(&prepared_boolean, 4))
    });

    for count in 1..=4 {
        let direct_case = format!("direct_{count}");
        config.run("feature", "mesh_boolean_crossover", &direct_case, 1, || {
            boolean_measurement(&direct_boolean_results(&boolean_left, &boolean_right, count))
        });
        let prepare_case = format!("prepare_extract_{count}");
        config.run("feature", "mesh_boolean_crossover", &prepare_case, 1, || {
            let prepared = boolean_left.try_prepare_boolean(&boolean_right).unwrap();
            boolean_measurement(&prepared_boolean_results(&prepared, count))
        });
        let extract_case = format!("prebuilt_extract_{count}");
        config.run("feature", "mesh_boolean_crossover", &extract_case, 1, || {
            boolean_measurement(&prepared_boolean_results(&prepared_boolean, count))
        });
    }

    let boolean_fixtures = [
        (
            "disjoint",
            Mesh::cube(Real::from(4_u8), ()),
            Mesh::cube(Real::from(4_u8), ()).translate(
                Real::from(10_u8),
                Real::zero(),
                Real::zero(),
            ),
        ),
        (
            "identical",
            Mesh::cube(Real::from(4_u8), ()),
            Mesh::cube(Real::from(4_u8), ()),
        ),
        (
            "contained",
            Mesh::cube(Real::from(4_u8), ()),
            Mesh::cube(Real::from(2_u8), ()),
        ),
        (
            "face_touching",
            Mesh::cube(Real::from(4_u8), ()),
            Mesh::cube(Real::from(4_u8), ()).translate(
                Real::from(4_u8),
                Real::zero(),
                Real::zero(),
            ),
        ),
    ];
    for (fixture, left, right) in &boolean_fixtures {
        let prepared = left.try_prepare_boolean(right).unwrap();
        let direct_case = format!("{fixture}_direct_four");
        config.run("feature", "mesh_boolean_fixtures", &direct_case, 1, || {
            boolean_measurement(&direct_boolean_results(left, right, 4))
        });
        let prepare_case = format!("{fixture}_prepare_extract_four");
        config.run("feature", "mesh_boolean_fixtures", &prepare_case, 1, || {
            let prepared = left.try_prepare_boolean(right).unwrap();
            boolean_measurement(&prepared_boolean_results(&prepared, 4))
        });
        let extract_case = format!("{fixture}_prebuilt_extract_four");
        config.run("feature", "mesh_boolean_fixtures", &extract_case, 1, || {
            boolean_measurement(&prepared_boolean_results(&prepared, 4))
        });
    }

    let sdf_min = Point3::new(Real::from(-3_i8), Real::from(-3_i8), Real::from(-3_i8));
    let sdf_max = Point3::new(Real::from(3_u8), Real::from(3_u8), Real::from(3_u8));
    config.run("feature", "implicit", "hypersdf_surface_nets", 1, || {
        let expr = SdfExpr::sphere(
            hyperlimit::Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Real::from(4_u8),
        );
        let output = Mesh::sdf_expr(
            expr,
            (20, 20, 20),
            sdf_min.clone(),
            sdf_max.clone(),
            Real::zero(),
            (),
        );
        mesh_measurement(&output, 20 * 20 * 20)
    });
    config.run("feature", "implicit", "metaballs_3d", 1, || {
        let balls = [
            MetaBall::new(Point3::origin(), Real::from(2_u8)),
            MetaBall::new(
                Point3::new(Real::from(2_u8), Real::zero(), Real::zero()),
                Real::from(2_u8),
            ),
        ];
        let output = Mesh::metaballs(&balls, (20, 20, 20), Real::one(), Real::one(), ());
        mesh_measurement(&output, 20 * 20 * 20)
    });
    config.run("feature", "implicit", "metaballs_2d", 1, || {
        let balls = [
            (Point2::new(Real::zero(), Real::zero()), Real::from(2_u8)),
            (Point2::new(Real::from(2_u8), Real::zero()), Real::from(2_u8)),
        ];
        let output = Profile::metaballs(&balls, (48, 48), Real::one(), Real::one());
        profile_measurement(&output, 48 * 48)
    });
    let tpms_bounds = Mesh::cube(Real::from(12_u8), ());
    config.run("feature", "implicit", "tpms_catalog", 1, || {
        let gyroid = tpms_bounds.gyroid(18, Real::from(6_u8), Real::zero(), ());
        let schwarz = tpms_bounds.schwarz_p(18, Real::from(6_u8), Real::zero(), ());
        let polygons = gyroid.triangles().len() + schwarz.triangles().len();
        Measurement::new(2 * 18 * 18 * 18, polygons as u64, polygons as u64)
    });

    config.run("feature", "raster_vector", "image_contours", 2, || {
        let image = GrayImage::from_fn(64, 64, |x, y| {
            let dx = i64::from(x) - 32;
            let dy = i64::from(y) - 32;
            Luma([u8::from(dx * dx + dy * dy < 24 * 24) * 255])
        });
        let output = Profile::from_image(&image, 128, true);
        profile_measurement(&output, 64 * 64)
    });
    config.run("feature", "text", "truetype_outline", 2, || {
        let output = Profile::text(
            "csgrs benchmark",
            include_bytes!("../asar.ttf"),
            Real::from(24_u8),
        );
        profile_measurement(&output, 15)
    });
    config.run("feature", "text", "hershey_strokes", 8, || {
        static GLYPHS: [&str; 1] = ["MWRMNV RRMVV"];
        let font = hershey::Font::new(&GLYPHS, 'A');
        let output = Profile::from_hershey("AAAA", &font, Real::from(2_u8));
        profile_measurement(&output, 4)
    });

    config.run("feature", "adapter", "bevy_mesh", 4, || {
        let output = black_box(&mesh).to_bevy_mesh();
        black_box(output);
        Measurement::new(mesh.triangles().len() as u64, 1, 1)
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
            Mesh::cube(Real::from(2_u8), metadata).translate(
                Real::from(index % 8) * Real::from(3_u8),
                Real::from(index / 8) * Real::from(3_u8),
                Real::from(index % 3),
            )
        })
        .collect::<Vec<_>>();
    config.run("feature", "parts", "blueprint_aabb", 4, || {
        let report = blueprint_from_aabb_parts(&parts, BlueprintProjection::Front, false);
        let edges = report.assembled.edges.len() + report.exploded.edges.len();
        Measurement::new(parts.len() as u64, edges as u64, edges as u64)
    });

    let io_mesh = Mesh::sphere(Real::from(4_u8), 20, 10, ());
    config.run("feature", "mesh_io", "all_exporters", 1, || {
        let stl = io_mesh.to_stl_binary("benchmark").expect("STL export");
        let dxf = io_mesh.to_dxf().expect("DXF export");
        let obj = io_mesh.to_obj("benchmark").expect("OBJ export");
        let ply = io_mesh.to_ply("benchmark").expect("PLY export");
        let amf = io_mesh.to_amf("benchmark", "millimeter").expect("AMF export");
        let gltf = io_mesh.to_gltf("benchmark").expect("glTF export");
        let size = stl.len() + dxf.len() + obj.len() + ply.len() + amf.len() + gltf.len();
        Measurement::new(io_mesh.triangles().len() as u64, size as u64, size as u64)
    });
    config.run("feature", "mesh_io", "public_writer_exporters", 1, || {
        let mut obj = Vec::new();
        csgrs::io::obj::write_obj(&io_mesh, &mut obj, "benchmark").expect("OBJ writer");
        let mut ply = Vec::new();
        csgrs::io::ply::write_ply(&io_mesh, &mut ply, "benchmark").expect("PLY writer");
        let mut amf = Vec::new();
        csgrs::io::amf::write_amf(&io_mesh, &mut amf, "benchmark", "millimeter")
            .expect("AMF writer");
        let colored_amf = csgrs::io::amf::to_amf_with_color(
            &io_mesh,
            "benchmark",
            "millimeter",
            (Real::one(), Real::zero(), Real::zero()),
        )
        .expect("colored AMF serializer");
        let mut gltf = Vec::new();
        csgrs::io::gltf::write_gltf(&io_mesh, &mut gltf, "benchmark").expect("glTF writer");
        let size = obj.len() + ply.len() + amf.len() + colored_amf.len() + gltf.len();
        Measurement::new(io_mesh.triangles().len() as u64, size as u64, size as u64)
    });
    config.run("feature", "mesh_io", "writer_obj", 1, || {
        let mut output = Vec::new();
        csgrs::io::obj::write_obj(&io_mesh, &mut output, "benchmark").expect("OBJ writer");
        Measurement::new(
            io_mesh.triangles().len() as u64,
            output.len() as u64,
            output.len() as u64,
        )
    });
    config.run("feature", "mesh_io", "writer_ply", 1, || {
        let mut output = Vec::new();
        csgrs::io::ply::write_ply(&io_mesh, &mut output, "benchmark").expect("PLY writer");
        Measurement::new(
            io_mesh.triangles().len() as u64,
            output.len() as u64,
            output.len() as u64,
        )
    });
    config.run("feature", "mesh_io", "writer_amf", 1, || {
        let mut output = Vec::new();
        csgrs::io::amf::write_amf(&io_mesh, &mut output, "benchmark", "millimeter")
            .expect("AMF writer");
        Measurement::new(
            io_mesh.triangles().len() as u64,
            output.len() as u64,
            output.len() as u64,
        )
    });
    config.run("feature", "mesh_io", "writer_gltf", 1, || {
        let mut output = Vec::new();
        csgrs::io::gltf::write_gltf(&io_mesh, &mut output, "benchmark").expect("glTF writer");
        Measurement::new(
            io_mesh.triangles().len() as u64,
            output.len() as u64,
            output.len() as u64,
        )
    });
    let stl = io_mesh.to_stl_binary("benchmark").expect("STL fixture");
    let obj = io_mesh.to_obj("benchmark").expect("OBJ fixture");
    config.run("feature", "mesh_io", "roundtrip_importers", 1, || {
        let from_stl = Mesh::from_stl(&stl, ()).expect("STL import");
        let from_obj = Mesh::from_obj(Cursor::new(obj.as_bytes()), ()).expect("OBJ import");
        let polygons = from_stl.triangles().len() + from_obj.triangles().len();
        Measurement::new(
            (stl.len() + obj.len()) as u64,
            polygons as u64,
            polygons as u64,
        )
    });

    let io_profile = Profile::ring(Real::from(8_u8), Real::from(2_u8), 48);
    config.run("feature", "profile_io", "svg_gerber_roundtrip", 1, || {
        let svg = io_profile.to_svg().expect("SVG export");
        let gerber = io_profile.to_gerber().expect("Gerber export");
        let svg_profile = Profile::from_svg(&svg).expect("SVG import");
        let gerber_profile = Profile::from_gerber(&gerber).expect("Gerber import");
        let output = svg_profile.material_contour_count()
            + gerber_profile.material_contour_count()
            + gerber_profile.hole_contour_count();
        Measurement::new(
            (svg.len() + gerber.len()) as u64,
            output as u64,
            output as u64,
        )
    });
}
