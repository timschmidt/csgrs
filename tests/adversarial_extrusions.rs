//! Adversarial tests for extrusion, revolve, loft, and sweep operations.

use csgrs::csg::CSG;
use csgrs::errors::ValidationError;
use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::Mesh;
use csgrs::polygon::Polygon;
use csgrs::sketch::Sketch;
use csgrs::vertex::Vertex;
use geo::{
    Coord, Geometry, GeometryCollection, Line, LineString, MultiPolygon,
    Polygon as GeoPolygon, Rect, Triangle,
};
use nalgebra::{Point3, Vector3};
use proptest::prelude::*;
use std::panic::{AssertUnwindSafe, catch_unwind};

fn finite_real() -> impl Strategy<Value = Real> {
    #[cfg(feature = "f32")]
    {
        (-250.0f32..250.0f32).prop_filter("finite", |v| v.is_finite())
    }
    #[cfg(feature = "f64")]
    {
        (-250.0f64..250.0f64).prop_filter("finite", |v| v.is_finite())
    }
}

fn positive_real() -> impl Strategy<Value = Real> {
    #[cfg(feature = "f32")]
    {
        (1.0e-4f32..50.0f32).prop_filter("positive finite", |v| v.is_finite() && *v > 0.0)
    }
    #[cfg(feature = "f64")]
    {
        (1.0e-6f64..50.0f64).prop_filter("positive finite", |v| v.is_finite() && *v > 0.0)
    }
}

fn assert_mesh_finite<M: Clone + Send + Sync + std::fmt::Debug>(mesh: &Mesh<M>) {
    for polygon in &mesh.polygons {
        assert!(polygon.vertices.len() >= 3, "degenerate polygon: {polygon:?}");
        let poly_box = polygon.bounding_box();
        assert!(
            poly_box.mins.x.is_finite() && poly_box.maxs.x.is_finite(),
            "{polygon:?}"
        );
        assert!(
            poly_box.mins.y.is_finite() && poly_box.maxs.y.is_finite(),
            "{polygon:?}"
        );
        assert!(
            poly_box.mins.z.is_finite() && poly_box.maxs.z.is_finite(),
            "{polygon:?}"
        );
    }
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite(), "{vertex:?}");
        assert!(vertex.position.y.is_finite(), "{vertex:?}");
        assert!(vertex.position.z.is_finite(), "{vertex:?}");
        assert!(vertex.normal.x.is_finite(), "{vertex:?}");
        assert!(vertex.normal.y.is_finite(), "{vertex:?}");
        assert!(vertex.normal.z.is_finite(), "{vertex:?}");
    }
    if !mesh.polygons.is_empty() {
        let bbox = mesh.bounding_box();
        assert!(bbox.mins.x.is_finite() && bbox.maxs.x.is_finite(), "{bbox:?}");
        assert!(bbox.mins.y.is_finite() && bbox.maxs.y.is_finite(), "{bbox:?}");
        assert!(bbox.mins.z.is_finite() && bbox.maxs.z.is_finite(), "{bbox:?}");
    }
}

fn assert_result_finite(result: Result<Mesh<&'static str>, ValidationError>) {
    if let Ok(mesh) = result {
        assert_mesh_finite(&mesh);
    }
}

fn polygon_at_z(
    points: &[(Real, Real)],
    z: Real,
    metadata: &'static str,
) -> Polygon<&'static str> {
    Polygon::new(
        points
            .iter()
            .map(|&(x, y)| Vertex::new(Point3::new(x, y, z), Vector3::z()))
            .collect(),
        metadata,
    )
}

fn path_catalog() -> Vec<Vec<Point3<Real>>> {
    vec![
        vec![],
        vec![Point3::origin()],
        vec![Point3::origin(), Point3::origin()],
        vec![Point3::origin(), Point3::new(0.0, 0.0, 1.0)],
        vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
        vec![
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        vec![
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ],
        vec![
            Point3::origin(),
            Point3::new(tolerance() * 0.25, 0.0, 0.0),
            Point3::new(0.0, tolerance() * 0.25, 0.0),
            Point3::new(0.0, 0.0, tolerance() * 0.25),
        ],
        vec![Point3::new(Real::NAN, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        vec![
            Point3::new(0.0, Real::INFINITY, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ],
    ]
}

fn sketch_catalog() -> Vec<Sketch<&'static str>> {
    let holed = GeoPolygon::new(
        LineString::from(vec![
            (-2.0, -2.0),
            (2.0, -2.0),
            (2.0, 2.0),
            (-2.0, 2.0),
            (-2.0, -2.0),
        ]),
        vec![LineString::from(vec![
            (-0.5, -0.5),
            (-0.5, 0.5),
            (0.5, 0.5),
            (0.5, -0.5),
            (-0.5, -0.5),
        ])],
    );

    vec![
        Sketch::rectangle(1.0, 0.5, "rect"),
        Sketch::rectangle(1.0, 0.5, "offset_profile").translate(1.5, 0.0, 0.0),
        Sketch::circle(1.0, 16, "circle"),
        Sketch::star(5, 1.0, 0.25, "star"),
        Sketch::ring(2.0, 0.25, 24, "ring"),
        Sketch::from_geo(Geometry::Polygon(holed).into(), "holed"),
        Sketch::from_geo(
            GeometryCollection(vec![
                Geometry::Line(Line::new(Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 0.0 })),
                Geometry::LineString(LineString::from(vec![
                    (0.0, 0.0),
                    (0.0, 1.0),
                    (1.0, 1.0),
                ])),
                Geometry::Rect(Rect::new(
                    Coord { x: -1.0, y: -1.0 },
                    Coord { x: 1.0, y: 1.0 },
                )),
                Geometry::Triangle(Triangle::new(
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 1.0, y: 0.0 },
                    Coord { x: 0.0, y: 1.0 },
                )),
            ]),
            "mixed_geo",
        ),
        Sketch::from_geo(
            Geometry::MultiPolygon(MultiPolygon(vec![
                GeoPolygon::new(
                    LineString::from(vec![
                        (0.0, 0.0),
                        (1.0, 0.0),
                        (1.0, 1.0),
                        (0.0, 1.0),
                        (0.0, 0.0),
                    ]),
                    vec![],
                ),
                GeoPolygon::new(
                    LineString::from(vec![
                        (2.0, 0.0),
                        (3.0, 0.0),
                        (3.0, 1.0),
                        (2.0, 1.0),
                        (2.0, 0.0),
                    ]),
                    vec![],
                ),
            ]))
            .into(),
            "multipolygon",
        ),
    ]
}

#[test]
fn adversarial_extrude_height_and_direction_catalog_is_finite() {
    let heights = [
        -1.0e6,
        -10.0,
        -1.0,
        -tolerance(),
        -tolerance() * 0.25,
        -0.0,
        0.0,
        tolerance() * 0.25,
        tolerance(),
        1.0,
        10.0,
        1.0e6,
        Real::NAN,
        Real::INFINITY,
        Real::NEG_INFINITY,
    ];
    let directions = [
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(tolerance() * 0.25, 0.0, 0.0),
        Vector3::new(Real::NAN, 0.0, 1.0),
        Vector3::new(0.0, Real::INFINITY, 1.0),
    ];

    for sketch in sketch_catalog() {
        for height in heights {
            let mesh = catch_unwind(AssertUnwindSafe(|| sketch.extrude(height)))
                .expect("extrude should not panic");
            assert_mesh_finite(&mesh);
        }
        for direction in directions {
            let mesh = catch_unwind(AssertUnwindSafe(|| sketch.extrude_vector(direction)))
                .expect("extrude_vector should not panic");
            assert_mesh_finite(&mesh);
        }
    }
}

#[test]
fn adversarial_revolve_angle_segment_and_profile_catalog_is_finite() {
    let angles = [
        -1080.0,
        -360.0,
        -180.0,
        -1.0,
        -0.0,
        0.0,
        tolerance(),
        1.0,
        45.0,
        90.0,
        180.0,
        359.999999,
        360.0,
        360.000001,
        720.0,
        1080.0,
        Real::NAN,
        Real::INFINITY,
        Real::NEG_INFINITY,
    ];
    let segments = [0usize, 1, 2, 3, 4, 5, 8, 16, 64];

    for sketch in sketch_catalog() {
        for angle in angles {
            for segment_count in segments {
                let result =
                    catch_unwind(AssertUnwindSafe(|| sketch.revolve(angle, segment_count)))
                        .expect("revolve should not panic");
                assert_result_finite(result);
            }
        }
    }
}

#[test]
fn adversarial_sweep_path_catalog_is_finite() {
    for sketch in sketch_catalog() {
        for path in path_catalog() {
            let mesh = catch_unwind(AssertUnwindSafe(|| sketch.sweep(&path)))
                .expect("sweep should not panic");
            assert_mesh_finite(&mesh);
        }
    }
}

#[test]
fn adversarial_loft_polygon_pair_catalog_is_finite_or_errors() {
    let bottoms = [
        polygon_at_z(&[(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], 0.0, "tri_bottom"),
        polygon_at_z(
            &[(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)],
            0.0,
            "square_bottom",
        ),
        polygon_at_z(
            &[
                (0.0, 0.0),
                (tolerance() * 0.25, 0.0),
                (0.0, tolerance() * 0.25),
            ],
            0.0,
            "tiny_bottom",
        ),
    ];
    let tops = [
        polygon_at_z(&[(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], 1.0, "tri_top"),
        polygon_at_z(
            &[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)],
            2.0,
            "square_top",
        ),
    ];

    for bottom in &bottoms {
        for top in &tops {
            for flip in [false, true] {
                let result =
                    catch_unwind(AssertUnwindSafe(|| Sketch::loft(bottom, top, flip)))
                        .expect("loft should not panic");
                assert_result_finite(result);
            }
        }
    }

    let mut mismatched_top =
        polygon_at_z(&[(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], 1.0, "bad_top");
    mismatched_top.vertices.pop();
    let result = catch_unwind(AssertUnwindSafe(|| {
        Sketch::loft(&bottoms[0], &mismatched_top, true)
    }))
    .expect("loft should report mismatched vertices without panicking");
    assert!(matches!(result, Err(ValidationError::MismatchedVertices)));
}

#[test]
fn adversarial_mesh_shape_helpers_that_delegate_to_extrusion_are_finite() {
    let scalar_cases = [
        -10.0,
        -1.0,
        -tolerance(),
        0.0,
        tolerance() * 0.25,
        tolerance(),
        1.0,
        10.0,
    ];
    let counts = [0usize, 1, 2, 3, 4, 8, 16, 32];

    for &a in &scalar_cases {
        for &b in &scalar_cases {
            for &count in &counts {
                let attempts = [
                    catch_unwind(AssertUnwindSafe(|| {
                        Mesh::<&str>::egg(a, b, count, count, "egg")
                    })),
                    catch_unwind(AssertUnwindSafe(|| {
                        Mesh::<&str>::teardrop(a, b, count, count, "teardrop")
                    })),
                    catch_unwind(AssertUnwindSafe(|| {
                        Mesh::<&str>::teardrop_cylinder(a, b, 1.0, count, "teardrop_cylinder")
                    })),
                    catch_unwind(AssertUnwindSafe(|| {
                        Mesh::<&str>::torus(a, b, count, count, "torus")
                    })),
                ];
                for attempt in attempts {
                    if let Ok(mesh) = attempt {
                        assert_mesh_finite(&mesh);
                    }
                }
            }
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 192,
        max_shrink_iters: 2048,
        .. ProptestConfig::default()
    })]

    #[test]
    fn proptest_extrude_vector_never_emits_nonfinite_vertices(
        width in positive_real(),
        height in positive_real(),
        dx in finite_real(),
        dy in finite_real(),
        dz in finite_real(),
    ) {
        let sketch = Sketch::<&str>::rectangle(width, height, "rect");
        let mesh = sketch.extrude_vector(Vector3::new(dx, dy, dz));
        assert_mesh_finite(&mesh);
    }

    #[test]
    fn proptest_revolve_finite_angles_and_segments_are_contained(
        radius in positive_real(),
        height in positive_real(),
        x_offset in 0.0f64..25.0f64,
        angle in -1080.0f64..1080.0f64,
        segments in 2usize..96,
    ) {
        let sketch = Sketch::<&str>::rectangle(radius, height, "revolve")
            .translate(x_offset as Real, 0.0, 0.0);
        let mesh = sketch.revolve(angle as Real, segments).unwrap();
        assert_mesh_finite(&mesh);
    }

    #[test]
    fn proptest_sweep_duplicate_and_sharp_turn_paths_are_contained(
        profile_radius in 0.001f64..2.0f64,
        p0 in (finite_real(), finite_real(), finite_real()),
        p1 in (finite_real(), finite_real(), finite_real()),
        p2 in (finite_real(), finite_real(), finite_real()),
        duplicate_first in any::<bool>(),
        close_path in any::<bool>(),
    ) {
        let sketch = Sketch::<&str>::circle(profile_radius as Real, 12, "sweep");
        let start = Point3::new(p0.0, p0.1, p0.2);
        let mut path = vec![
            start,
            if duplicate_first { start } else { Point3::new(p1.0, p1.1, p1.2) },
            Point3::new(p2.0, p2.1, p2.2),
        ];
        if close_path {
            path.push(start);
        }
        let mesh = sketch.sweep(&path);
        assert_mesh_finite(&mesh);
    }

    #[test]
    fn proptest_loft_matching_vertex_polygons_are_contained(
        z in -50.0f64..50.0f64,
        scale in 0.001f64..25.0f64,
        skew in -10.0f64..10.0f64,
        flip in any::<bool>(),
    ) {
        let bottom = polygon_at_z(&[(0.0, 0.0), (1.0, 0.0), (0.5, 0.75)], 0.0, "bottom");
        let top = Polygon::new(
            vec![
                Vertex::new(Point3::new(skew as Real, 0.0, z as Real), Vector3::z()),
                Vertex::new(Point3::new(skew as Real + scale as Real, 0.0, z as Real), Vector3::z()),
                Vertex::new(Point3::new(skew as Real + scale as Real * 0.5, scale as Real, z as Real), Vector3::z()),
            ],
            "top",
        );
        let mesh = Sketch::loft(&bottom, &top, flip).unwrap();
        assert_mesh_finite(&mesh);
    }
}
