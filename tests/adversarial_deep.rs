use csgrs::csg::CSG;
use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::Mesh;
use csgrs::mesh::plane::Plane;
use csgrs::polygon::Polygon;
use csgrs::sketch::Sketch;
use csgrs::vertex::{Vertex, VertexCluster};
use geo::{CoordsIter, Geometry, GeometryCollection, LineString, Polygon as GeoPolygon};
use hashbrown::HashMap;
use nalgebra::{Matrix4, Point3, Vector3};
use proptest::prelude::*;
use std::num::NonZeroU32;
use std::panic::{AssertUnwindSafe, catch_unwind};

fn finite_real() -> impl Strategy<Value = Real> {
    #[cfg(feature = "f32")]
    {
        (-1.0e3f32..1.0e3f32).prop_filter("finite", |v| v.is_finite())
    }
    #[cfg(feature = "f64")]
    {
        (-1.0e3f64..1.0e3f64).prop_filter("finite", |v| v.is_finite())
    }
}

fn positive_real() -> impl Strategy<Value = Real> {
    #[cfg(feature = "f32")]
    {
        (1.0e-4f32..50.0f32).prop_filter("finite positive", |v| v.is_finite() && *v > 0.0)
    }
    #[cfg(feature = "f64")]
    {
        (1.0e-6f64..50.0f64).prop_filter("finite positive", |v| v.is_finite() && *v > 0.0)
    }
}

fn finite_point() -> impl Strategy<Value = Point3<Real>> {
    (finite_real(), finite_real(), finite_real()).prop_map(|(x, y, z)| Point3::new(x, y, z))
}

fn assert_vertex_finite(vertex: &Vertex) {
    assert!(vertex.position.x.is_finite(), "non-finite position x: {vertex:?}");
    assert!(vertex.position.y.is_finite(), "non-finite position y: {vertex:?}");
    assert!(vertex.position.z.is_finite(), "non-finite position z: {vertex:?}");
    assert!(vertex.normal.x.is_finite(), "non-finite normal x: {vertex:?}");
    assert!(vertex.normal.y.is_finite(), "non-finite normal y: {vertex:?}");
    assert!(vertex.normal.z.is_finite(), "non-finite normal z: {vertex:?}");
}

fn assert_polygon_sane<S: Clone + Send + Sync>(polygon: &Polygon<S>) {
    assert!(polygon.vertices.len() >= 3);
    for vertex in &polygon.vertices {
        assert_vertex_finite(vertex);
    }
    let bbox = polygon.bounding_box();
    assert!(bbox.mins.x.is_finite() && bbox.maxs.x.is_finite());
    assert!(bbox.mins.y.is_finite() && bbox.maxs.y.is_finite());
    assert!(bbox.mins.z.is_finite() && bbox.maxs.z.is_finite());
}

fn assert_mesh_sane<S: Clone + Send + Sync + std::fmt::Debug>(mesh: &Mesh<S>) {
    for polygon in &mesh.polygons {
        assert_polygon_sane(polygon);
    }
    if !mesh.polygons.is_empty() {
        let bbox = mesh.bounding_box();
        assert!(bbox.mins.x.is_finite() && bbox.maxs.x.is_finite(), "{bbox:?}");
        assert!(bbox.mins.y.is_finite() && bbox.maxs.y.is_finite(), "{bbox:?}");
        assert!(bbox.mins.z.is_finite() && bbox.maxs.z.is_finite(), "{bbox:?}");
        for vertex in mesh.vertices() {
            assert!(vertex.position.x >= bbox.mins.x - tolerance());
            assert!(vertex.position.y >= bbox.mins.y - tolerance());
            assert!(vertex.position.z >= bbox.mins.z - tolerance());
            assert!(vertex.position.x <= bbox.maxs.x + tolerance());
            assert!(vertex.position.y <= bbox.maxs.y + tolerance());
            assert!(vertex.position.z <= bbox.maxs.z + tolerance());
        }
    }
}

fn assert_sketch_sane<S: Clone + Send + Sync + std::fmt::Debug>(sketch: &Sketch<S>) {
    for coord in sketch.geometry.coords_iter() {
        assert!(coord.x.is_finite(), "non-finite sketch x: {coord:?}");
        assert!(coord.y.is_finite(), "non-finite sketch y: {coord:?}");
    }
    if sketch.geometry.coords_iter().next().is_some() {
        let bbox = sketch.bounding_box();
        assert!(bbox.mins.x.is_finite() && bbox.maxs.x.is_finite(), "{bbox:?}");
        assert!(bbox.mins.y.is_finite() && bbox.maxs.y.is_finite(), "{bbox:?}");
    }
}

fn sketch_all_coords_finite<S>(sketch: &Sketch<S>) -> bool {
    sketch
        .geometry
        .coords_iter()
        .all(|coord| coord.x.is_finite() && coord.y.is_finite())
}

fn triangle_at_z(z: Real) -> Polygon<&'static str> {
    Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, z), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, z), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, z), Vector3::z()),
        ],
        Some("loft"),
    )
}

#[test]
fn adversarial_deep_mesh_primitive_catalog_is_contained() {
    let scalars = [
        -1.0,
        -tolerance(),
        0.0,
        tolerance() * 0.5,
        tolerance(),
        1.0,
        10.0,
    ];
    let counts = [0usize, 1, 2, 3, 4, 5, 8, 16];

    for &a in &scalars {
        for &b in &scalars {
            for &segments in &counts {
                let attempts = [
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::cuboid(a, b, 1.0, Some("cuboid")))),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::cylinder(a, b, segments, Some("cylinder")))),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::frustum(a, b, 1.0, segments, Some("frustum")))),
                    catch_unwind(AssertUnwindSafe(|| {
                        Mesh::<&str>::frustum_ptp(
                            Point3::origin(),
                            Point3::new(a, b, 1.0),
                            a,
                            b,
                            segments,
                            Some("frustum_ptp"),
                        )
                    })),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::sphere(a, segments, segments, Some("sphere")))),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::ellipsoid(a, b, 1.0, segments, segments, Some("ellipsoid")))),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::torus(a, b, segments, segments, Some("torus")))),
                    catch_unwind(AssertUnwindSafe(|| {
                        Mesh::<&str>::arrow(Point3::origin(), Vector3::new(a, b, 1.0), segments, false, Some("arrow"))
                    })),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::octahedron(a, Some("octahedron")))),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::icosahedron(a, Some("icosahedron")))),
                    catch_unwind(AssertUnwindSafe(|| Mesh::<&str>::teardrop_cylinder(a, b, 1.0, segments, Some("teardrop_cylinder")))),
                ];

                for attempt in attempts {
                    if let Ok(mesh) = attempt {
                        assert_mesh_sane(&mesh);
                    }
                }
            }
        }
    }
}

#[test]
fn adversarial_deep_sketch_shape_catalog_is_contained() {
    let scalars = [-1.0, 0.0, tolerance() * 0.5, tolerance(), 1.0, 10.0];
    let counts = [0usize, 1, 2, 3, 4, 5, 8, 16];

    for &a in &scalars {
        for &b in &scalars {
            for &segments in &counts {
                let attempts = [
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::rectangle(a, b, Some("rectangle")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::square(a, Some("square")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::circle(a, segments, Some("circle")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::right_triangle(a, b, Some("right_triangle")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::ellipse(a, b, segments, Some("ellipse")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::regular_ngon(segments, a, Some("ngon")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::arrow(a, b, 1.0, 0.5, Some("arrow")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::trapezoid(a, b, 1.0, 0.25, Some("trapezoid")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::star(segments, a, b, Some("star")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::rounded_rectangle(a, b, 0.5, segments, Some("rounded_rectangle")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::squircle(a, b, segments, Some("squircle")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::keyhole(a, b, 1.0, segments, Some("keyhole")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::reuleaux(segments, a, segments, Some("reuleaux")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::ring(a, b, segments, Some("ring")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::pie_slice(a, -90.0, 450.0, segments, Some("pie_slice")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::supershape(a, b, 0.0, 1.0, 1.0, 1.0, segments, Some("supershape")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::heart(a, b, segments, Some("heart")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::crescent(a, b, 0.25, segments, Some("crescent")))),
                    catch_unwind(AssertUnwindSafe(|| Sketch::<&str>::airfoil_naca4(a, b, 12.0, 1.0, segments, Some("airfoil")))),
                ];

                for attempt in attempts {
                    if let Ok(sketch) = attempt {
                        if !sketch_all_coords_finite(&sketch) {
                            continue;
                        }
                        assert_sketch_sane(&sketch);
                        let tris = catch_unwind(AssertUnwindSafe(|| sketch.triangulate()));
                        if let Ok(tris) = tris {
                            for tri in tris {
                                for point in tri {
                                    assert!(point.x.is_finite());
                                    assert!(point.y.is_finite());
                                    assert!(point.z.is_finite());
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn adversarial_deep_invalid_geo_conversion_and_boolean_catalog() {
    let collections = [
        GeometryCollection(vec![]),
        GeometryCollection(vec![Geometry::LineString(LineString::from(vec![
            (0.0, 0.0),
            (1.0, 0.0),
        ]))]),
        GeometryCollection(vec![Geometry::Polygon(GeoPolygon::new(
            LineString::from(vec![(0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0), (0.0, 0.0)]),
            vec![],
        ))]),
        GeometryCollection(vec![Geometry::Polygon(GeoPolygon::new(
            LineString::from(vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]),
            vec![LineString::from(vec![
                (4.0, 2.0),
                (5.0, 2.0),
                (5.0, 3.0),
                (4.0, 3.0),
                (4.0, 2.0),
            ])],
        ))]),
    ];

    let cutter = Sketch::<&str>::rectangle(0.5, 0.5, Some("cutter"));
    for collection in collections {
        let sketch = Sketch::from_geo(collection, Some("geo"));
        assert_sketch_sane(&sketch);
        for result in [
            catch_unwind(AssertUnwindSafe(|| sketch.union(&cutter))),
            catch_unwind(AssertUnwindSafe(|| sketch.difference(&cutter))),
            catch_unwind(AssertUnwindSafe(|| sketch.intersection(&cutter))),
            catch_unwind(AssertUnwindSafe(|| sketch.xor(&cutter))),
            catch_unwind(AssertUnwindSafe(|| sketch.renormalize())),
            catch_unwind(AssertUnwindSafe(|| sketch.offset(0.1))),
            catch_unwind(AssertUnwindSafe(|| sketch.offset_rounded(-0.1))),
            catch_unwind(AssertUnwindSafe(|| sketch.straight_skeleton(true))),
        ] {
            if let Ok(output) = result {
                assert_sketch_sane(&output);
            }
        }
    }
}

#[test]
fn adversarial_deep_extrude_loft_revolve_sweep_catalog() {
    let profile = Sketch::<&str>::rectangle(1.0, 0.5, Some("profile")).translate(0.5, 0.0, 0.0);
    let paths = [
        vec![],
        vec![Point3::origin()],
        vec![Point3::origin(), Point3::origin()],
        vec![Point3::origin(), Point3::new(0.0, 0.0, 1.0)],
        vec![
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ],
    ];

    for height in [-1.0, 0.0, tolerance() * 0.5, 1.0, 10.0] {
        if let Ok(mesh) = catch_unwind(AssertUnwindSafe(|| profile.extrude(height))) {
            assert_mesh_sane(&mesh);
        }
        if let Ok(mesh) = catch_unwind(AssertUnwindSafe(|| profile.extrude_vector(Vector3::new(height, 0.0, 1.0)))) {
            assert_mesh_sane(&mesh);
        }
    }

    for angle in [-360.0, -1.0, 0.0, 90.0, 360.0, 720.0] {
        for segments in [0usize, 1, 2, 3, 8] {
            let result = catch_unwind(AssertUnwindSafe(|| profile.revolve(angle, segments)));
            if let Ok(Ok(mesh)) = result {
                assert_mesh_sane(&mesh);
            }
        }
    }

    for path in paths {
        if let Ok(mesh) = catch_unwind(AssertUnwindSafe(|| profile.sweep(&path))) {
            assert_mesh_sane(&mesh);
        }
    }

    let bottom = triangle_at_z(0.0);
    let top = triangle_at_z(1.0);
    let mut bad_top = top.clone();
    bad_top.vertices.pop();
    if let Ok(Ok(mesh)) = catch_unwind(AssertUnwindSafe(|| Sketch::loft(&bottom, &top, true))) {
        assert_mesh_sane(&mesh);
    }
    assert!(matches!(
        catch_unwind(AssertUnwindSafe(|| Sketch::loft(&bottom, &bad_top, true))),
        Ok(Err(_))
    ));
}

#[test]
fn adversarial_deep_center_float_cache_and_mirror_invariants() {
    let mut mesh = Mesh::<&str>::cube(2.0, Some("cube")).translate(5.0, -7.0, -3.0);
    let before = mesh.bounding_box();
    mesh.invalidate_bounding_box();
    let after = mesh.bounding_box();
    assert_eq!(before.mins, after.mins);
    assert_eq!(before.maxs, after.maxs);

    let centered = mesh.center();
    let centered_box = centered.bounding_box();
    assert!(((centered_box.mins.x + centered_box.maxs.x) * 0.5).abs() <= tolerance() * 16.0);
    assert!(((centered_box.mins.y + centered_box.maxs.y) * 0.5).abs() <= tolerance() * 16.0);
    assert!(((centered_box.mins.z + centered_box.maxs.z) * 0.5).abs() <= tolerance() * 16.0);

    let floated = mesh.float();
    assert!(floated.bounding_box().mins.z.abs() <= tolerance() * 16.0);

    let plane = Plane::from_normal(Vector3::z(), 0.25);
    let mirrored_twice = mesh.mirror(plane.clone()).mirror(plane);
    assert_mesh_sane(&mirrored_twice);

    let identity = mesh.transform(&Matrix4::identity());
    assert_eq!(identity.polygons.len(), mesh.polygons.len());
}

#[test]
fn adversarial_deep_sdf_tpms_image_and_text_catalog() {
    let min = Point3::new(-1.0, -1.0, -1.0);
    let max = Point3::new(1.0, 1.0, 1.0);
    for resolution in [0usize, 1, 2, 3, 8] {
        for iso in [-1.0, 0.0, 0.5, 10.0, Real::NAN, Real::INFINITY] {
            let sdf = catch_unwind(AssertUnwindSafe(|| {
                Mesh::<&str>::sdf(
                    |p| {
                        if p.x > 0.75 {
                            Real::NAN
                        } else {
                            p.coords.norm() - 0.5
                        }
                    },
                    (resolution, resolution, resolution),
                    min,
                    max,
                    iso,
                    Some("sdf"),
                )
            }));
            if let Ok(mesh) = sdf {
                assert_mesh_sane(&mesh);
            }

            let base = Mesh::<&str>::cube(2.0, Some("tpms"));
            for result in [
                catch_unwind(AssertUnwindSafe(|| base.gyroid(resolution, 1.0, iso, Some("gyroid")))),
                catch_unwind(AssertUnwindSafe(|| base.schwarz_p(resolution, 1.0, iso, Some("schwarz_p")))),
                catch_unwind(AssertUnwindSafe(|| base.schwarz_d(resolution, 1.0, iso, Some("schwarz_d")))),
            ] {
                if let Ok(mesh) = result {
                    assert_mesh_sane(&mesh);
                }
            }
        }
    }

    #[cfg(feature = "image-io")]
    {
        use image::{GrayImage, Luma};
        for (width, height) in [(0, 0), (1, 1), (2, 2), (8, 8)] {
            let mut image = GrayImage::new(width, height);
            for y in 0..height {
                for x in 0..width {
                    let value = if (x + y) % 2 == 0 { 255 } else { 0 };
                    image.put_pixel(x, y, Luma([value]));
                }
            }
            if let Ok(sketch) = catch_unwind(AssertUnwindSafe(|| {
                Sketch::<&str>::from_image(&image, 128, true, Some("image"))
            })) {
                assert_sketch_sane(&sketch);
            }
        }
    }

    #[cfg(feature = "truetype-text")]
    {
        for bytes in [b"".as_slice(), b"not a font".as_slice(), &[0, 1, 2, 3, 4, 5, 6, 7]] {
            let sketch = Sketch::<&str>::text("A\t\n\u{202e}", bytes, 1.0, Some("text"));
            assert_sketch_sane(&sketch);
        }
    }
}

#[test]
fn adversarial_deep_vertex_arithmetic_catalog() {
    let values = [-1.0, 0.0, tolerance() * 0.5, 0.5, 1.0, 2.0, Real::NAN, Real::INFINITY];
    let a = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z());
    let b = Vertex::new(Point3::new(1.0, 1.0, 1.0), -Vector3::z());

    for &t in &values {
        let result = catch_unwind(AssertUnwindSafe(|| a.interpolate(&b, t)));
        if t.is_finite() {
            if let Ok(vertex) = result {
                assert_vertex_finite(&vertex);
            }
        }

        let result = catch_unwind(AssertUnwindSafe(|| a.slerp_interpolate(&b, t)));
        if t.is_finite() {
            if let Ok(vertex) = result {
                assert_vertex_finite(&vertex);
            }
        }
    }

    for point in [
        Point3::origin(),
        Point3::new(tolerance() * 0.5, 0.0, 0.0),
        Point3::new(1.0e6, -1.0e6, 1.0),
    ] {
        let vertex = Vertex::new(point, Vector3::new(1.0, 0.0, 0.0));
        assert_vertex_finite(&vertex);
        assert!(a.distance_to(&vertex).is_finite());
        assert!(a.distance_squared_to(&vertex).is_finite());
        assert!(a.normal_angle_to(&vertex).is_finite());
    }
}

#[test]
fn adversarial_deep_vertex_quality_and_cluster_catalog() {
    let vertices = [
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(tolerance() * 0.5, 0.0, 0.0), Vector3::zeros()),
    ];

    assert!(Vertex::weighted_average(&[]).is_none());
    assert!(Vertex::weighted_average(&[(vertices[0], 0.0), (vertices[1], 0.0)]).is_none());
    for weights in [
        vec![(vertices[0], 1.0), (vertices[1], 1.0), (vertices[2], 1.0)],
        vec![(vertices[0], -1.0), (vertices[1], 3.0)],
        vec![(vertices[0], Real::NAN), (vertices[1], 1.0)],
    ] {
        if let Some(vertex) = Vertex::weighted_average(&weights) {
            if weights.iter().all(|(_, weight)| weight.is_finite()) {
                assert_vertex_finite(&vertex);
            }
        }
    }

    for (u, v, w) in [
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (-1.0, 1.0, 1.0),
        (Real::NAN, 0.5, 0.5),
    ] {
        let result = catch_unwind(AssertUnwindSafe(|| {
            Vertex::barycentric_interpolate(&vertices[0], &vertices[1], &vertices[2], u, v, w)
        }));
        if u.is_finite() && v.is_finite() && w.is_finite() {
            if let Ok(vertex) = result {
                assert_vertex_finite(&vertex);
            }
        }
    }

    let refs = [&vertices[0], &vertices[1], &vertices[2]];
    for center in &vertices {
        let weight = Vertex::compute_cotangent_weight(center, &vertices[1], &refs);
        assert!(weight.is_finite());
        let _curvature = center.estimate_mean_curvature(&vertices, &[0.5, 1.0, -1.0]);
    }

    let mut adjacency = HashMap::new();
    adjacency.insert(0usize, vec![1, 2, 3]);
    adjacency.insert(1usize, vec![0]);
    let mut positions = HashMap::new();
    let mut normals = HashMap::new();
    for (idx, vertex) in vertices.iter().enumerate() {
        positions.insert(idx, vertex.position);
        normals.insert(idx, vertex.normal);
    }
    for idx in [0usize, 1, 999] {
        let (_valence, regularity) = Vertex::analyze_connectivity_with_index(idx, &adjacency);
        assert!(regularity.is_finite());
    }
    let (_valence, regularity) =
        vertices[0].analyze_connectivity_by_position(&adjacency, &positions, tolerance() * 16.0);
    assert!(regularity.is_finite());
    let quality = vertices[0].comprehensive_quality_analysis(0, &adjacency, &positions, &normals);
    assert!(quality.0.is_finite());
    assert!(quality.1.is_finite());
    assert!(quality.2.is_finite());
    assert!(quality.3.is_finite());

    assert!(VertexCluster::from_vertices(&[]).is_none());
    let cluster = VertexCluster::from_vertices(&vertices).unwrap();
    assert!(cluster.position.x.is_finite());
    assert!(cluster.normal.norm().is_finite());
    assert_eq!(cluster.count, vertices.len());
    assert!(cluster.radius.is_finite());
    assert_vertex_finite(&cluster.to_vertex());
}

#[test]
fn adversarial_deep_toolpath_all_generators_emit_finite_gcode() {
    use csgrs::toolpath::gcode::Post;
    use csgrs::toolpath::{
        Feeds, FdmLayerCfg, KerfSide, LatheCfg, LeadInOut, MachineKind, Nozzle, PocketCfg, Tool,
        contour_only, cut2d_contours, fdm_layer_from_sketch, lathe_rough_from_profile, pocket2d,
    };

    let feeds = Feeds {
        travel: 3000.0,
        xy: 1200.0,
        plunge: 300.0,
        rpm: Some(12_000.0),
        power: Some(0.75),
        pierce_ms: Some(5),
    };
    let post = Post {
        absolute_e: false,
        units_mm: true,
        z_safe: 5.0,
    };
    let sketches = [
        Sketch::<()>::new(),
        Sketch::rectangle(10.0, 5.0, None),
        Sketch::ring(3.0, 1.0, 16, None),
        Sketch::polygon(&[[0.0, 0.0], [4.0, 0.0], [2.0, 0.01], [0.0, 0.0]], None),
    ];

    for sketch in &sketches {
        let fdm = fdm_layer_from_sketch(
            sketch,
            0.2,
            &FdmLayerCfg {
                nozzle: Nozzle {
                    width: 0.42,
                    layer_height: 0.2,
                    keepout_radii: Some(vec![(0.0, 0.0), (10.0, 1.0)]),
                },
                perimeters: 2,
                hilbert_order: 3,
                infill_density: 0.2,
                e_per_mm: 0.05,
            },
            &feeds,
        );
        let cut = cut2d_contours(
            sketch,
            -1.0,
            0.1,
            KerfSide::Outside,
            Some(LeadInOut {
                length: 0.5,
                radius: 0.0,
            }),
            &feeds,
            MachineKind::Laser,
        );
        let pocket = pocket2d(
            sketch,
            5.0,
            0.0,
            &PocketCfg {
                tool: Tool {
                    diameter: 3.0,
                    corner_radius: 0.0,
                },
                stepover: 0.4,
                stepdown: 1.0,
                depth: 2.0,
                finish_allow: 0.1,
            },
            &feeds,
            true,
        );
        let contour = contour_only(sketch, -0.5, 1000.0, MachineKind::Router);
        let lathe = lathe_rough_from_profile(
            sketch,
            -10.0,
            10.0,
            6.0,
            &LatheCfg {
                doc_radial: 0.5,
                feed: 200.0,
            },
        );
        for toolpath in [&fdm, &cut, &pocket, &contour, &lathe] {
            for mv in &toolpath.moves {
                assert!(mv.position.x.is_finite());
                assert!(mv.position.y.is_finite());
                assert!(mv.position.z.is_finite());
                assert!(mv.feed.map(|f| f.is_finite()).unwrap_or(true));
                assert!(mv.scalar.map(|s| s.is_finite()).unwrap_or(true));
            }
            let gcode = post.write(toolpath);
            assert!(!gcode.contains("NaN"));
            assert!(!gcode.contains("inf"));
        }
    }
}

#[test]
#[cfg(feature = "nurbs")]
fn adversarial_deep_nurbs_catalog_and_booleans_are_contained() {
    use csgrs::nurbs::Nurbs;
    use nalgebra::Point2;

    let point_sets = [
        vec![],
        vec![Point2::origin()],
        vec![Point2::origin(), Point2::new(1.0, 0.0)],
        vec![Point2::origin(), Point2::new(1.0, 0.0), Point2::new(0.0, 1.0)],
        vec![
            Point2::origin(),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 0.0),
        ],
    ];

    for points in point_sets {
        let result =
            catch_unwind(AssertUnwindSafe(|| Nurbs::<&str>::polyline(&points, Some("poly"))));
        if let Ok(Ok(nurbs)) = result {
            let mp = nurbs.to_multipolygon(Some(0.1));
            for polygon in mp.0 {
                for coord in &polygon.exterior().0 {
                    assert!(coord.x.is_finite());
                    assert!(coord.y.is_finite());
                }
            }
            let sketch = nurbs.to_sketch(Some(0.1));
            assert_sketch_sane(&sketch);
            let mesh = nurbs.extrude_vector(Vector3::new(0.0, 0.0, 1.0), Some(0.1));
            assert_mesh_sane(&mesh);
        }
    }

    let empty = Nurbs::<&str>::empty(Some("empty"));
    let rect = Nurbs::rectangle(2.0, 2.0, Some("rect"));
    let circle = Nurbs::circle(1.0, Some("circle"));
    for result in [
        catch_unwind(AssertUnwindSafe(|| empty.try_union(&rect))),
        catch_unwind(AssertUnwindSafe(|| rect.try_difference(&empty))),
        catch_unwind(AssertUnwindSafe(|| rect.try_intersection(&empty))),
    ] {
        if let Ok(Ok(nurbs)) = result {
            let sketch = nurbs.to_sketch(Some(0.1));
            assert_sketch_sane(&sketch);
        }
    }
    if let Ok(circle) = circle {
        for result in [
            catch_unwind(AssertUnwindSafe(|| rect.try_union(&circle))),
            catch_unwind(AssertUnwindSafe(|| rect.try_difference(&circle))),
            catch_unwind(AssertUnwindSafe(|| rect.try_intersection(&circle))),
        ] {
            if let Ok(Ok(nurbs)) = result {
                let sketch = nurbs.to_sketch(Some(0.1));
                assert_sketch_sane(&sketch);
            }
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(96))]

    #[test]
    fn proptest_deep_transform_operation_chains_are_finite(
        size in positive_real(),
        dx in finite_real(),
        dy in finite_real(),
        dz in finite_real(),
        angle in -720.0f64..720.0f64,
        scale in -5.0f64..5.0f64,
    ) {
        let mut mesh = Mesh::<()>::cube(size, None)
            .translate(dx, dy, dz)
            .rotate(angle as Real, (angle * 0.5) as Real, (-angle) as Real)
            .scale(scale as Real, (scale.abs() + tolerance()) as Real, 1.0);
        mesh.renormalize();
        assert_mesh_sane(&mesh);

        let triangulated = mesh.triangulate();
        assert_mesh_sane(&triangulated);
        if let Some(level) = NonZeroU32::new(1) {
            let subdivided = triangulated.subdivide_triangles(level);
            assert_mesh_sane(&subdivided);
        }
    }

    #[test]
    fn proptest_deep_triangle_soup_loft_slice_and_intersections(
        a in finite_point(),
        b in finite_point(),
        c in finite_point(),
        z in -5.0f64..5.0f64,
    ) {
        let normal = (b - a).cross(&(c - a));
        if normal.norm() > tolerance() && normal.norm().is_finite() {
            let poly: Polygon<()> = Polygon::new(
                vec![Vertex::new(a, normal), Vertex::new(b, normal), Vertex::new(c, normal)],
                None,
            );
            let mesh = Mesh::from_polygons(&[poly], None);
            assert_mesh_sane(&mesh);

            let plane = Plane::from_normal(Vector3::z(), z as Real);
            let slice = catch_unwind(AssertUnwindSafe(|| mesh.slice(plane)));
            if let Ok(sketch) = slice {
                assert_sketch_sane(&sketch);
            }

            let ray_hits = mesh.ray_intersections(&Point3::new(0.0, 0.0, -10.0), &Vector3::z());
            for (point, distance) in ray_hits {
                assert!(point.x.is_finite());
                assert!(point.y.is_finite());
                assert!(point.z.is_finite());
                assert!(distance.is_finite());
            }
        }
    }

    #[test]
    fn proptest_deep_structured_geo_holes_triangulate_and_offset(
        width in 0.5f64..50.0f64,
        height in 0.5f64..50.0f64,
        inset in 0.01f64..0.45f64,
        offset in -2.0f64..2.0f64,
    ) {
        let w = width as Real;
        let h = height as Real;
        let margin = (w.min(h) * inset as Real).max(tolerance());
        let outer = LineString::from(vec![(0.0, 0.0), (w, 0.0), (w, h), (0.0, h), (0.0, 0.0)]);
        let hole = LineString::from(vec![
            (margin, margin),
            (w - margin, margin),
            (w - margin, h - margin),
            (margin, h - margin),
            (margin, margin),
        ]);
        let sketch = Sketch::<()>::from_geo(
            GeometryCollection(vec![Geometry::Polygon(GeoPolygon::new(outer, vec![hole]))]),
            None,
        );
        assert_sketch_sane(&sketch);
        for tri in sketch.triangulate() {
            for point in tri {
                prop_assert!(point.x.is_finite());
                prop_assert!(point.y.is_finite());
                prop_assert!(point.z.is_finite());
            }
        }
        let offset_sketch = sketch.offset(offset as Real);
        assert_sketch_sane(&offset_sketch);
    }

    #[test]
    fn proptest_deep_boolean_associativity_clean_cubes(
        size in positive_real(),
        ax in -20.0f64..-5.0f64,
        bx in -2.0f64..2.0f64,
        cx in 5.0f64..20.0f64,
    ) {
        let a = Mesh::<()>::cube(size, None).translate(ax as Real, 0.0, 0.0);
        let b = Mesh::<()>::cube(size, None).translate(bx as Real, 0.0, 0.0);
        let c = Mesh::<()>::cube(size, None).translate(cx as Real, 0.0, 0.0);
        let left = a.union(&b.union(&c));
        let right = a.union(&b).union(&c);
        assert_mesh_sane(&left);
        assert_mesh_sane(&right);
        prop_assert_eq!(left.polygons.len(), right.polygons.len());
    }

    #[test]
    fn proptest_deep_bezier_bspline_and_hilbert_catalog(
        points in prop::collection::vec((finite_real(), finite_real()), 0..10),
        segments in 0usize..32,
        order in 0usize..5,
        padding in -2.0f64..2.0f64,
    ) {
        let control = points.iter().map(|&(x, y)| [x, y]).collect::<Vec<_>>();
        for result in [
            catch_unwind(AssertUnwindSafe(|| Sketch::<()>::bezier(&control, segments, None))),
            catch_unwind(AssertUnwindSafe(|| Sketch::<()>::bspline(&control, 3, segments, None))),
        ] {
            if let Ok(sketch) = result {
                assert_sketch_sane(&sketch);
            }
        }

        let base = Sketch::<()>::rectangle(10.0, 10.0, None);
        let hilbert = base.hilbert_curve(order, padding as Real);
        assert_sketch_sane(&hilbert);
    }
}
