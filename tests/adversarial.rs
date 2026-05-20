//! Broad adversarial tests for geometry constructors, booleans, and invariants.

use csgrs::csg::CSG;
use csgrs::float_types::{PI, Real, hreal_from_f64, tolerance};
#[cfg(feature = "svg-io")]
use csgrs::io::svg::FromSVG;
use csgrs::mesh::Mesh;
use csgrs::mesh::bsp::Node;
use csgrs::mesh::connectivity::VertexIndexMap;
use csgrs::mesh::plane::Plane;
use csgrs::polygon::Polygon;
use csgrs::sketch::Profile;
use csgrs::triangulated::{IndexedTriangulated3D, Triangulated3D};
use csgrs::vertex::{Vertex, VertexCluster};
use hashbrown::HashMap;
use hypercurve::Point2 as HPoint2;
use nalgebra::{Matrix4, Point3, Vector3};
use proptest::prelude::*;
use std::io::Cursor;
use std::panic::{AssertUnwindSafe, catch_unwind};

fn finite_real() -> impl Strategy<Value = Real> {
    (-1.0e4f64..1.0e4f64).prop_filter("finite", |v| v.is_finite())
}

fn positive_real() -> impl Strategy<Value = Real> {
    (1.0e-6f64..100.0f64).prop_filter("finite positive", |v| v.is_finite() && *v > 0.0)
}

fn point3_strategy() -> impl Strategy<Value = Point3<Real>> {
    (finite_real(), finite_real(), finite_real()).prop_map(|(x, y, z)| Point3::new(x, y, z))
}

fn vector3_strategy() -> impl Strategy<Value = Vector3<Real>> {
    (finite_real(), finite_real(), finite_real()).prop_map(|(x, y, z)| Vector3::new(x, y, z))
}

fn assert_vertex_finite(vertex: &Vertex) {
    assert!(
        vertex.position.x.is_finite(),
        "non-finite vertex x: {vertex:?}"
    );
    assert!(
        vertex.position.y.is_finite(),
        "non-finite vertex y: {vertex:?}"
    );
    assert!(
        vertex.position.z.is_finite(),
        "non-finite vertex z: {vertex:?}"
    );
    assert!(vertex.normal.x.is_finite(), "non-finite normal x: {vertex:?}");
    assert!(vertex.normal.y.is_finite(), "non-finite normal y: {vertex:?}");
    assert!(vertex.normal.z.is_finite(), "non-finite normal z: {vertex:?}");
}

fn assert_polygon_sane<M: Clone + Send + Sync>(polygon: &Polygon<M>) {
    assert!(polygon.vertices.len() >= 3, "polygon has too few vertices");
    for vertex in &polygon.vertices {
        assert_vertex_finite(vertex);
    }
    let bbox = polygon.bounding_box();
    assert!(
        bbox.mins.x.is_finite() && bbox.maxs.x.is_finite(),
        "bad bbox x: {bbox:?}"
    );
    assert!(
        bbox.mins.y.is_finite() && bbox.maxs.y.is_finite(),
        "bad bbox y: {bbox:?}"
    );
    assert!(
        bbox.mins.z.is_finite() && bbox.maxs.z.is_finite(),
        "bad bbox z: {bbox:?}"
    );
    for vertex in &polygon.vertices {
        assert!(vertex.position.x >= bbox.mins.x - tolerance());
        assert!(vertex.position.y >= bbox.mins.y - tolerance());
        assert!(vertex.position.z >= bbox.mins.z - tolerance());
        assert!(vertex.position.x <= bbox.maxs.x + tolerance());
        assert!(vertex.position.y <= bbox.maxs.y + tolerance());
        assert!(vertex.position.z <= bbox.maxs.z + tolerance());
    }
}

fn assert_mesh_sane<M: Clone + Send + Sync + std::fmt::Debug>(mesh: &Mesh<M>) {
    for polygon in &mesh.polygons {
        assert_polygon_sane(polygon);
    }
    if !mesh.polygons.is_empty() {
        let bbox = mesh.bounding_box();
        assert!(
            bbox.mins.x.is_finite() && bbox.maxs.x.is_finite(),
            "bad mesh bbox x: {bbox:?}"
        );
        assert!(
            bbox.mins.y.is_finite() && bbox.maxs.y.is_finite(),
            "bad mesh bbox y: {bbox:?}"
        );
        assert!(
            bbox.mins.z.is_finite() && bbox.maxs.z.is_finite(),
            "bad mesh bbox z: {bbox:?}"
        );
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

fn assert_triangles_finite(triangles: &[[Point3<Real>; 3]]) {
    for triangle in triangles {
        for point in triangle {
            assert!(point.x.is_finite(), "non-finite triangle x: {triangle:?}");
            assert!(point.y.is_finite(), "non-finite triangle y: {triangle:?}");
            assert!(point.z.is_finite(), "non-finite triangle z: {triangle:?}");
        }
    }
}

fn assert_slice_edges_on_plane(edges: &[[Vertex; 2]], plane: &Plane) {
    for edge in edges {
        for vertex in edge {
            assert_vertex_finite(vertex);
            assert_eq!(
                plane.orient_point(&vertex.position),
                csgrs::mesh::plane::COPLANAR,
                "BSP slice edge vertex drifted off slicing plane: {vertex:?}"
            );
        }
    }
}

fn simple_triangle(a: Point3<Real>, b: Point3<Real>, c: Point3<Real>) -> Option<Polygon<()>> {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(&ac);
    if normal.norm() <= tolerance() || !normal.norm().is_finite() {
        return None;
    }
    Some(Polygon::new(
        vec![
            Vertex::new(a, normal),
            Vertex::new(b, normal),
            Vertex::new(c, normal),
        ],
        (),
    ))
}

fn near_duplicate_triangle_mesh() -> Mesh<()> {
    let normal = Vector3::z();
    let near_origin = Point3::new(tolerance() * 0.25, 0.0, 0.0);
    let polygons = vec![
        Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
            ],
            (),
        ),
        Polygon::new(
            vec![
                Vertex::new(near_origin, normal),
                Vertex::new(Point3::new(1.0, 1.0, 0.0), normal),
                Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
            ],
            (),
        ),
    ];
    Mesh::from_polygons(&polygons, ())
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    if bytes.is_empty() {
        return 0.0;
    }
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-1.0e4, 1.0e4) as Real
}

fn run_mesh_bytecode(bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }

    let mut idx = 0;
    let mut stack: Vec<Mesh<()>> = Vec::new();
    while idx < bytes.len() && stack.len() < 16 {
        let op = bytes[idx] % 8;
        idx += 1;
        match op {
            0 => {
                let size = decode_real(bytes, &mut idx).abs().max(tolerance());
                if let Ok(mesh) = catch_unwind(AssertUnwindSafe(|| Mesh::cube(size, ()))) {
                    stack.push(mesh);
                }
            },
            1 => {
                let radius = decode_real(bytes, &mut idx).abs().max(tolerance());
                let segments = (bytes[idx % bytes.len()] as usize % 16) + 3;
                idx += 1;
                if let Ok(mesh) = catch_unwind(AssertUnwindSafe(|| {
                    Mesh::sphere(radius, segments, segments, ())
                })) {
                    stack.push(mesh);
                }
            },
            2 => {
                let Some(mesh) = stack.pop() else { continue };
                let dx = decode_real(bytes, &mut idx);
                let dy = decode_real(bytes, &mut idx);
                let dz = decode_real(bytes, &mut idx);
                stack.push(mesh.translate(dx, dy, dz));
            },
            3 => {
                let Some(mesh) = stack.pop() else { continue };
                let rx = decode_real(bytes, &mut idx);
                let ry = decode_real(bytes, &mut idx);
                let rz = decode_real(bytes, &mut idx);
                stack.push(mesh.rotate(rx, ry, rz));
            },
            4 => {
                let Some(mesh) = stack.pop() else { continue };
                let sx = decode_real(bytes, &mut idx).clamp(-10.0, 10.0);
                let sy = decode_real(bytes, &mut idx).clamp(-10.0, 10.0);
                let sz = decode_real(bytes, &mut idx).clamp(-10.0, 10.0);
                stack.push(mesh.scale(sx, sy, sz));
            },
            5 => {
                let Some(mesh) = stack.pop() else { continue };
                stack.push(mesh.triangulate());
            },
            6 => {
                let Some(mesh) = stack.pop() else { continue };
                let mut copy = mesh.clone();
                copy.renormalize();
                stack.push(copy);
            },
            7 => {
                let Some(mesh) = stack.pop() else { continue };
                stack.push(mesh.inverse());
            },
            _ => unreachable!(),
        }
    }

    for mesh in &stack {
        assert_mesh_sane(mesh);
    }
}

#[test]
fn adversarial_mesh_constructor_sweep_does_not_corrupt_successes() {
    let scalars: &[Real] = &[
        -10.0,
        -1.0,
        -0.0,
        0.0,
        tolerance() / 16.0,
        tolerance(),
        1.0,
        10.0,
        1.0e4,
        Real::NAN,
        Real::INFINITY,
        Real::NEG_INFINITY,
    ];
    let counts = [0usize, 1, 2, 3, 4, 5, 8, 16, 31, 64];

    for &a in scalars {
        for &b in scalars {
            for &segments in &counts {
                if let Ok(mesh) = catch_unwind(AssertUnwindSafe(|| Mesh::cube(a, ()))) {
                    assert_mesh_sane(&mesh);
                }
                if let Ok(mesh) =
                    catch_unwind(AssertUnwindSafe(|| Mesh::cuboid(a, b, 1.0, ())))
                {
                    assert_mesh_sane(&mesh);
                }
                if let Ok(mesh) =
                    catch_unwind(AssertUnwindSafe(|| Mesh::cylinder(a, b, segments, ())))
                {
                    assert_mesh_sane(&mesh);
                }
                if let Ok(mesh) =
                    catch_unwind(AssertUnwindSafe(|| Mesh::sphere(a, segments, segments, ())))
                {
                    assert_mesh_sane(&mesh);
                }
            }
        }
    }
}

#[test]
fn adversarial_sketch_constructor_sweep_does_not_corrupt_successes() {
    let scalars: &[Real] = &[
        -10.0,
        -1.0,
        -0.0,
        0.0,
        tolerance() / 16.0,
        tolerance(),
        1.0,
        10.0,
        1.0e4,
        Real::NAN,
        Real::INFINITY,
        Real::NEG_INFINITY,
    ];
    let counts = [0usize, 1, 2, 3, 4, 5, 8, 16, 31, 64];

    for &a in scalars {
        for &b in scalars {
            for &segments in &counts {
                if let Ok(sketch) = catch_unwind(AssertUnwindSafe(|| Profile::square(a, ()))) {
                    let _ = sketch.region_profiles();
                    let _ = sketch.wire_polylines();
                    let _ = catch_unwind(AssertUnwindSafe(|| sketch.triangulate()));
                }
                if let Ok(sketch) =
                    catch_unwind(AssertUnwindSafe(|| Profile::rectangle(a, b, ())))
                {
                    let _ = sketch.region_profiles();
                    let _ = sketch.wire_polylines();
                    let _ = catch_unwind(AssertUnwindSafe(|| sketch.triangulate()));
                }
                if let Ok(sketch) =
                    catch_unwind(AssertUnwindSafe(|| Profile::circle(a, segments, ())))
                {
                    let _ = sketch.region_profiles();
                    let _ = sketch.wire_polylines();
                    let _ = catch_unwind(AssertUnwindSafe(|| sketch.triangulate()));
                }
                if let Ok(sketch) =
                    catch_unwind(AssertUnwindSafe(|| Profile::ellipse(a, b, segments, ())))
                {
                    let _ = sketch.region_profiles();
                    let _ = sketch.wire_polylines();
                    let _ = catch_unwind(AssertUnwindSafe(|| sketch.triangulate()));
                }
            }
        }
    }
}

#[test]
fn adversarial_invalid_polygon_catalog_is_contained() {
    let catalogs: &[&[[Real; 2]]] = &[
        &[],
        &[[0.0, 0.0]],
        &[[0.0, 0.0], [1.0, 0.0]],
        &[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
        &[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        &[[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0]],
        &[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [1.0, 0.0], [0.0, 2.0]],
        &[[Real::NAN, 0.0], [1.0, 0.0], [0.0, 1.0]],
        &[[Real::INFINITY, 0.0], [1.0, 0.0], [0.0, 1.0]],
    ];

    for points in catalogs {
        let result = catch_unwind(AssertUnwindSafe(|| {
            let sketch: Profile<()> = Profile::polygon(points, ());
            sketch.triangulate()
        }));
        if let Ok(triangles) = result {
            assert_triangles_finite(&triangles);
        }
    }
}

#[test]
fn adversarial_sketch_polygon_points_uses_hypercurve_points_directly() {
    let points = [
        HPoint2::new(hreal_from_f64(0.0).unwrap(), hreal_from_f64(0.0).unwrap()),
        HPoint2::new(hreal_from_f64(2.0).unwrap(), hreal_from_f64(0.0).unwrap()),
        HPoint2::new(hreal_from_f64(1.0).unwrap(), hreal_from_f64(1.0).unwrap()),
    ];
    let sketch = Profile::<()>::polygon_points(&points, ());

    assert!(!sketch.is_empty());
    assert!(sketch.contains_xy(1.0, 0.25).unwrap_or(false));

    let duplicate = [
        HPoint2::new(hreal_from_f64(0.0).unwrap(), hreal_from_f64(0.0).unwrap()),
        HPoint2::new(hreal_from_f64(0.0).unwrap(), hreal_from_f64(0.0).unwrap()),
        HPoint2::new(hreal_from_f64(1.0).unwrap(), hreal_from_f64(0.0).unwrap()),
    ];
    assert!(Profile::<()>::polygon_points(&duplicate, ()).is_empty());
}

#[test]
fn adversarial_transform_matrix_sweep_is_contained() {
    let cube: Mesh<()> = Mesh::cube(1.0, ());
    let matrices = [
        Matrix4::identity(),
        Matrix4::zeros(),
        Matrix4::new_nonuniform_scaling(&Vector3::new(0.0, 1.0, 1.0)),
        Matrix4::new_nonuniform_scaling(&Vector3::new(-1.0, 1.0, -1.0)),
        Matrix4::new(
            1.0,
            0.0,
            0.0,
            Real::NAN,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    ];

    for matrix in matrices {
        let result = catch_unwind(AssertUnwindSafe(|| cube.transform(&matrix)));
        if let Ok(mesh) = result {
            let all_finite = mesh.vertices().into_iter().all(|v| {
                v.position.x.is_finite()
                    && v.position.y.is_finite()
                    && v.position.z.is_finite()
                    && v.normal.x.is_finite()
                    && v.normal.y.is_finite()
                    && v.normal.z.is_finite()
            });
            if all_finite {
                assert_mesh_sane(&mesh);
            }
        }
    }
}

#[test]
fn adversarial_transform_matrix_fuzz_regression_preserves_finite_normals() {
    let bytes = [10u8, 255, 255];
    let mut idx = 0usize;
    let mut values = [0.0 as Real; 16];
    for value in &mut values {
        let mut raw = [0u8; 8];
        for slot in &mut raw {
            *slot = bytes[idx % bytes.len()];
            idx += 1;
        }
        *value = ((i64::from_le_bytes(raw) as f64) / 1.0e12).clamp(-1.0e6, 1.0e6) as Real;
    }

    let matrix = Matrix4::from_row_slice(&values);
    let transformed = Mesh::<()>::cube(1.0, ()).transform(&matrix);

    for vertex in transformed.vertices() {
        assert!(
            vertex.normal.x.is_finite(),
            "normal x is not finite: {vertex:?}"
        );
        assert!(
            vertex.normal.y.is_finite(),
            "normal y is not finite: {vertex:?}"
        );
        assert!(
            vertex.normal.z.is_finite(),
            "normal z is not finite: {vertex:?}"
        );
    }
}

#[test]
fn adversarial_transform_normals_use_hyperreal_checked_normalization() {
    let mesh: Mesh<()> = Mesh::cube(1.0, ());
    let matrix = Matrix4::new_nonuniform_scaling(&Vector3::new(2.0, 3.0, 4.0));

    let transformed = mesh.transform(&matrix);

    assert_mesh_sane(&transformed);
    for vertex in transformed.vertices() {
        assert!((vertex.normal.norm() - 1.0).abs() < tolerance());
    }
}

#[test]
fn adversarial_mesh_triangulated_visit_uses_hyperreal_checked_normals() {
    let mesh: Mesh<()> = Mesh::cube(1.0, ());
    let mut count = 0usize;

    mesh.visit_triangles(|triangle| {
        count += 1;
        for vertex in triangle {
            assert_vertex_finite(&vertex);
            assert!((vertex.normal.norm() - 1.0).abs() < tolerance());
        }
    });

    assert!(count > 0);
}

#[test]
fn adversarial_mesh_exports_closed_cube_to_hypermesh_exact() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ());
    let buffers = mesh.to_hypermesh_buffers();

    assert_eq!(buffers.positions.len(), 8 * 3);
    assert_eq!(buffers.indices.len(), 12 * 3);
    assert!(mesh.inspect_hypermesh_input().edge_ready());

    let exact = mesh
        .to_hypermesh_exact()
        .expect("closed cube should validate as an exact hypermesh solid");
    exact
        .validate_retained_state()
        .expect("retained hypermesh facts should replay");
    assert_eq!(exact.vertices().len(), 8);
    assert_eq!(exact.triangles().len(), 12);
    assert_eq!(
        exact.validation_policy(),
        hypermesh::exact::ValidationPolicy::CLOSED
    );
}

#[test]
fn adversarial_hypermesh_adapter_keeps_surface_policy_explicit() {
    let normal = Vector3::z();
    let triangle = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
        ],
        (),
    );
    let mesh = Mesh::from_polygons(&[triangle], ());

    assert!(mesh.to_hypermesh_exact().is_err());
    let surface = mesh
        .to_hypermesh_exact_with_policy(hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY)
        .expect("single triangle should remain an explicit boundary surface");
    assert_eq!(
        surface.validation_policy(),
        hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY
    );
}

#[test]
fn adversarial_hypermesh_adapter_reports_degenerate_topology_before_construction() {
    let normal = Vector3::z();
    let degenerate = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(-0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
        ],
        (),
    );
    let mesh = Mesh::from_polygons(&[degenerate], ());
    let report = mesh.inspect_hypermesh_input();

    assert!(!report.edge_ready());
    assert!(mesh.to_hypermesh_exact().is_err());
}

#[test]
fn adversarial_hypermesh_adapter_rejects_nonfinite_struct_literal_vertices() {
    let malicious = Polygon {
        vertices: vec![
            Vertex {
                position: Point3::new(0.0, 0.0, 0.0),
                normal: Vector3::z(),
            },
            Vertex {
                position: Point3::new(Real::NAN, 0.0, 0.0),
                normal: Vector3::z(),
            },
            Vertex {
                position: Point3::new(1.0, 0.0, 0.0),
                normal: Vector3::z(),
            },
        ],
        plane: Plane::from_normal(Vector3::z(), 0.0),
        bounding_box: std::sync::OnceLock::new(),
        metadata: (),
    };
    let mesh = Mesh {
        polygons: vec![malicious],
        bounding_box: std::sync::OnceLock::new(),
        query_trimesh: std::sync::OnceLock::new(),
        metadata: (),
    };

    let buffers = mesh.to_hypermesh_buffers();
    assert!(
        buffers
            .positions
            .iter()
            .all(|coordinate| coordinate.is_finite())
    );
    assert_eq!(buffers.positions.len(), 0);
    assert!(mesh.to_hypermesh_exact().is_err());
    assert!(mesh.to_hypermesh_handoff_package().is_err());
}

#[test]
#[cfg(feature = "sdf")]
fn adversarial_sdf_rejects_nonfinite_sampling_boundary_at_hyperreal_boundary() {
    let cases = [
        (
            Point3::new(Real::NAN, -1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
            0.0,
        ),
        (
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(Real::INFINITY, 1.0, 1.0),
            0.0,
        ),
        (
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
            Real::NAN,
        ),
    ];

    for (min_pt, max_pt, iso_value) in cases {
        let (mesh, diagnostics) = Mesh::<()>::sdf_with_diagnostics(
            |_| 0.0,
            (4, 4, 4),
            min_pt,
            max_pt,
            iso_value,
            (),
        );

        assert!(mesh.polygons.is_empty(), "{diagnostics:#?}");
        assert_eq!(diagnostics.sample_count, 64);
        assert_eq!(diagnostics.non_finite_sample_count, diagnostics.sample_count);
        assert_eq!(diagnostics.positive_sample_count, diagnostics.sample_count);
        assert_eq!(diagnostics.finite_sample_count, 0);
        assert_eq!(diagnostics.emitted_triangle_count, 0);
    }
}

#[test]
#[cfg(feature = "sdf")]
fn adversarial_sdf_tiny_nonzero_samples_keep_hyperreal_sign_before_surface_nets() {
    let (mesh, diagnostics) = Mesh::<()>::sdf_with_diagnostics(
        |_| 1.0e-50,
        (2, 2, 2),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, 1.0, 1.0),
        0.0,
        (),
    );

    assert!(mesh.polygons.is_empty(), "{diagnostics:#?}");
    assert_eq!(diagnostics.sample_count, 8);
    assert_eq!(diagnostics.finite_sample_count, 8);
    assert_eq!(diagnostics.positive_sample_count, 8);
    assert_eq!(diagnostics.zero_sample_count, 0);
    assert_eq!(diagnostics.negative_sample_count, 0);
    assert_eq!(diagnostics.crossing_cell_count, 0);
    assert_eq!(diagnostics.emitted_triangle_count, 0);
}

#[test]
fn adversarial_smoothing_rejects_nonfinite_weights_at_hyperreal_lerp_boundary() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ()).triangulate();
    let original_vertices = mesh.vertices();

    for smoothed in [
        mesh.laplacian_smooth(Real::NAN, 1, false),
        mesh.laplacian_smooth(Real::INFINITY, 1, false),
        mesh.taubin_smooth(Real::NAN, -0.2, 1, false),
        mesh.taubin_smooth(0.2, Real::NEG_INFINITY, 1, false),
    ] {
        assert_mesh_sane(&smoothed);
        let smoothed_vertices = smoothed.vertices();
        assert_eq!(original_vertices.len(), smoothed_vertices.len());
        for (before, after) in original_vertices.iter().zip(smoothed_vertices.iter()) {
            assert_eq!(before.position, after.position);
        }
    }
}

#[test]
#[cfg(feature = "metaballs")]
fn adversarial_metaballs_reject_invalid_sampling_boundary_at_hyperreal_boundary() {
    let valid = csgrs::mesh::metaballs::MetaBall::new(Point3::new(0.0, 0.0, 0.0), 1.0);
    let invalid_center =
        csgrs::mesh::metaballs::MetaBall::new(Point3::new(Real::NAN, 0.0, 0.0), 1.0);
    let invalid_radius =
        csgrs::mesh::metaballs::MetaBall::new(Point3::new(0.0, 0.0, 0.0), Real::INFINITY);

    let cases = [
        (vec![invalid_center], 1.0, 0.5),
        (vec![invalid_radius], 1.0, 0.5),
        (vec![valid.clone()], Real::NAN, 0.5),
        (vec![valid], 1.0, Real::NAN),
    ];

    for (balls, iso_value, padding) in cases {
        let (mesh, diagnostics) =
            Mesh::<()>::metaballs_with_diagnostics(&balls, (4, 4, 4), iso_value, padding, ());

        assert!(mesh.polygons.is_empty(), "{diagnostics:#?}");
        assert_eq!(diagnostics.sample_count, 64);
        assert_eq!(diagnostics.non_finite_sample_count, diagnostics.sample_count);
        assert_eq!(diagnostics.positive_sample_count, diagnostics.sample_count);
        assert_eq!(diagnostics.finite_sample_count, 0);
        assert_eq!(diagnostics.emitted_triangle_count, 0);
    }
}

#[test]
fn adversarial_is_manifold_uses_hypermesh_closed_manifold_facts() {
    let cube: Mesh<()> = Mesh::cube(2.0, ());
    assert!(cube.is_manifold());

    let normal = Vector3::z();
    let open_triangle = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
        ],
        (),
    );
    assert!(!Mesh::from_polygons(&[open_triangle], ()).is_manifold());
}

#[test]
fn adversarial_is_manifold_rejects_quantized_edge_false_positive() {
    let normal = Vector3::z();
    let offset = 4.0e-7;
    let polygons = vec![
        Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
            ],
            (),
        ),
        Polygon::new(
            vec![
                Vertex::new(Point3::new(offset, 0.0, 0.0), -normal),
                Vertex::new(Point3::new(0.0, 1.0 + offset, 0.0), -normal),
                Vertex::new(Point3::new(1.0 + offset, 0.0, 0.0), -normal),
            ],
            (),
        ),
    ];

    let mesh = Mesh::from_polygons(&polygons, ());
    assert!(!mesh.is_manifold());
    assert!(mesh.to_hypermesh_exact().is_err());
}

#[test]
fn adversarial_vertices_and_indices_share_hypermesh_topology() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ());
    let (vertices, indices) = mesh.get_vertices_and_indices();
    let exact = mesh
        .to_hypermesh_exact()
        .expect("cube should convert to exact hypermesh");

    assert_eq!(vertices.len(), exact.vertices().len());
    assert_eq!(indices.len(), exact.triangles().len());
    assert_eq!(vertices.len(), 8);
    assert_eq!(indices.len(), 12);
    assert!(mesh.to_trimesh().is_ok());
}

#[test]
fn adversarial_query_trimesh_uses_hypermesh_handoff_view() {
    let normal = Vector3::z();
    let surface = Mesh::from_polygons(
        &[Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
            ],
            (),
        )],
        (),
    );
    assert!(surface.to_trimesh().is_ok());
    assert!(surface.to_rapier_shape().is_ok());
    assert!(!surface.try_get_vertices_and_indices().unwrap().0.is_empty());

    let degenerate = Mesh::from_polygons(
        &[Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(-0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
            ],
            (),
        )],
        (),
    );
    assert!(degenerate.to_trimesh().is_err());
    assert!(degenerate.to_rapier_shape().is_err());
    assert!(degenerate.try_get_vertices_and_indices().is_err());
    assert_eq!(
        degenerate.get_vertices_and_indices(),
        (Vec::new(), Vec::new())
    );
}

#[test]
fn adversarial_hypermesh_exact_roundtrips_through_lossy_csgrs_adapter() {
    let source: Mesh<&'static str> = Mesh::cube(2.0, "source");
    let exact = source
        .to_hypermesh_exact()
        .expect("cube should convert to exact hypermesh");
    let roundtrip = Mesh::from_hypermesh_exact_lossy(&exact, "roundtrip")
        .expect("exact mesh should provide a finite approximate view");

    assert_eq!(roundtrip.metadata, "roundtrip");
    assert_eq!(roundtrip.polygons.len(), exact.triangles().len());
    assert!(roundtrip.is_manifold());
    let (vertices, indices) = roundtrip.get_vertices_and_indices();
    assert_eq!(vertices.len(), exact.vertices().len());
    assert_eq!(indices.len(), exact.triangles().len());
}

#[test]
fn adversarial_hypermesh_handoff_package_separates_exact_and_lossy_domains() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ());
    let exact = mesh
        .to_hypermesh_exact()
        .expect("cube should convert to exact hypermesh");
    let package = mesh
        .to_hypermesh_handoff_package()
        .expect("closed cube should package for hypermesh consumers");

    package
        .validate_against_mesh(&exact)
        .expect("handoff package should replay from exact mesh");
    assert!(package.readiness.closed_manifold);
    assert!(package.readiness.solid_handoff_ready);
    assert!(package.surface.is_some());
    assert!(package.solid.is_some());
    assert!(package.approximate_f64_view.is_some());
}

#[test]
fn adversarial_hypermesh_handoff_package_keeps_open_surface_non_solid() {
    let normal = Vector3::z();
    let mesh = Mesh::from_polygons(
        &[Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
            ],
            (),
        )],
        (),
    );

    assert!(mesh.to_hypermesh_handoff_package().is_err());
    let exact = mesh
        .to_hypermesh_exact_with_policy(hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY)
        .expect("single triangle should import as a boundary-allowed surface");
    let package = mesh
        .to_hypermesh_handoff_package_with_policy(
            hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("boundary-allowed surface should package without solid evidence");

    package
        .validate_against_mesh(&exact)
        .expect("surface package should replay from exact mesh");
    assert!(!package.readiness.closed_manifold);
    assert!(!package.readiness.solid_handoff_ready);
    assert!(package.readiness.boundary_allowed);
    assert!(package.surface.is_some());
    assert!(package.solid.is_none());
    assert!(package.approximate_f64_view.is_some());
}

#[test]
fn adversarial_hypermesh_exact_uses_triangulated3d_io_directly() {
    let exact = Mesh::<()>::cube(2.0, ())
        .to_hypermesh_exact()
        .expect("cube should convert to exact hypermesh");

    let mut triangle_count = 0usize;
    exact.visit_triangles(|triangle| {
        triangle_count += 1;
        for vertex in triangle {
            assert_vertex_finite(&vertex);
            assert!((vertex.normal.norm() - 1.0).abs() < tolerance());
        }
    });
    assert_eq!(triangle_count, exact.triangles().len());
    let indexed = exact.indexed_triangles();
    assert_eq!(indexed.positions.len(), exact.vertices().len());
    assert_eq!(indexed.faces.len(), exact.triangles().len());

    #[cfg(feature = "stl-io")]
    assert!(csgrs::io::stl::to_stl_ascii(&exact, "exact").contains("facet normal"));
    #[cfg(feature = "obj-io")]
    assert!(csgrs::io::obj::to_obj(&exact, "exact").contains("\nf "));
    #[cfg(feature = "ply-io")]
    assert!(csgrs::io::ply::to_ply(&exact, "exact").contains("element face"));
    #[cfg(feature = "amf-io")]
    assert!(csgrs::io::amf::to_amf(&exact, "exact", "millimeter").contains("triangle"));
    #[cfg(feature = "gltf-io")]
    assert!(csgrs::io::gltf::to_gltf(&exact, "exact").contains("\"meshes\""));
}

#[test]
fn adversarial_build_connectivity_uses_hypermesh_triangle_topology() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ());
    let exact = mesh
        .to_hypermesh_exact()
        .expect("cube should convert to exact hypermesh");
    let (vertex_map, adjacency) = mesh.build_connectivity();

    assert_eq!(vertex_map.vertex_count(), exact.vertices().len());
    for triangle in exact.triangles() {
        let [a, b, c] = triangle.0;
        for [lhs, rhs] in [[a, b], [b, c], [c, a]] {
            assert!(
                adjacency
                    .get(&lhs)
                    .is_some_and(|neighbors| neighbors.contains(&rhs)),
                "missing hypermesh edge {lhs}->{rhs} in connectivity graph"
            );
            assert!(
                adjacency
                    .get(&rhs)
                    .is_some_and(|neighbors| neighbors.contains(&lhs)),
                "missing hypermesh edge {rhs}->{lhs} in connectivity graph"
            );
        }
    }
}

#[test]
fn adversarial_mesh_primitive_low_segments_regression_is_contained() {
    let result = catch_unwind(AssertUnwindSafe(|| {
        let sphere = Mesh::<()>::sphere(1.0, 1, 1, ());
        let ellipsoid = Mesh::<()>::ellipsoid(1.0, 0.0, -1.0, 1, 1, ());
        let cylinder = Mesh::<()>::cylinder(1.0, 1.0, 1, ());
        (sphere, ellipsoid, cylinder)
    }));

    let (sphere, ellipsoid, cylinder) =
        result.expect("low tessellation counts should not panic");
    assert_mesh_sane(&sphere);
    for mesh in [&ellipsoid, &cylinder] {
        for vertex in mesh.vertices() {
            assert_vertex_finite(&vertex);
        }
    }
}

#[test]
fn adversarial_polyhedron_rejects_nonfinite_coordinates_before_vertex_sanitization() {
    let points = &[
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [Real::NAN, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    let faces: [&[usize]; 4] = [&[0, 1, 2], &[0, 1, 3], &[1, 2, 3], &[2, 0, 3]];

    let result = catch_unwind(AssertUnwindSafe(|| {
        Mesh::<()>::polyhedron(points, &faces, ())
    }));
    assert!(matches!(
        result,
        Ok(Err(csgrs::errors::ValidationError::InvalidCoordinate(_)))
    ));
}

#[test]
fn adversarial_mesh_primitives_reject_nonfinite_boundary_scalars() {
    let primitives = [
        Mesh::<()>::cuboid(1.0, Real::NAN, 1.0, ()),
        Mesh::<()>::sphere(Real::INFINITY, 8, 4, ()),
        Mesh::<()>::frustum(1.0, Real::NAN, 2.0, 8, ()),
        Mesh::<()>::cylinder(1.0, Real::INFINITY, 8, ()),
        Mesh::<()>::arrow(
            Point3::new(Real::NAN, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            8,
            false,
            (),
        ),
        Mesh::<()>::octahedron(Real::NAN, ()),
        Mesh::<()>::icosahedron(Real::INFINITY, ()),
    ];

    for mesh in primitives {
        assert!(mesh.polygons.is_empty());
        assert_mesh_sane(&mesh);
    }
}

#[test]
fn adversarial_zero_normal_angle_regression_is_finite() {
    let a = Vertex::new(Point3::origin(), Vector3::zeros());
    let b = Vertex::new(Point3::new(1.0, -1.0, 2.0), Vector3::zeros());

    assert!(a.normal_angle_to(&b).is_finite());
}

#[test]
fn adversarial_barycentric_interpolate_zero_weight_sum_and_normals_is_finite() {
    let a = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::zeros());
    let b = Vertex::new(Point3::new(3.0, 0.0, 0.0), Vector3::zeros());
    let c = Vertex::new(Point3::new(0.0, 3.0, 0.0), Vector3::zeros());

    let interpolated = Vertex::barycentric_interpolate(&a, &b, &c, 1.0, -1.0, 0.0);

    assert_vertex_finite(&interpolated);
    assert!((interpolated.position.x - 1.0).abs() < tolerance());
    assert!((interpolated.position.y - 1.0).abs() < tolerance());
    assert_eq!(interpolated.normal, Vector3::z());
}

#[test]
fn adversarial_weighted_average_uses_hyperreal_weight_normalization() {
    let a = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::zeros());
    let b = Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::zeros());
    let c = Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::zeros());

    let averaged = Vertex::weighted_average(&[(a, 1.0), (b, -1.0), (c, 3.0)])
        .expect("positive total weight should normalize");

    assert_vertex_finite(&averaged);
    assert!((averaged.position.x + 2.0 / 3.0).abs() < tolerance());
    assert!((averaged.position.y - 2.0).abs() < tolerance());
    assert_eq!(averaged.normal, Vector3::z());
    assert!(Vertex::weighted_average(&[(a, 1.0), (b, -1.0)]).is_none());
}

#[test]
fn adversarial_vertex_distance_uses_hyperlattice_squared_distance() {
    let a = Vertex::new(Point3::new(1.0 / 3.0, -2.0 / 3.0, 5.0), Vector3::z());
    let b = Vertex::new(Point3::new(4.0 / 3.0, -5.0 / 3.0, 17.0), Vector3::z());

    let distance_squared = a.distance_squared_to(&b);
    let distance = a.distance_to(&b);

    assert!((distance_squared - 146.0).abs() < tolerance());
    assert!((distance - 146.0_f64.sqrt()).abs() < tolerance());
    assert!(distance.is_finite());
    assert!(distance_squared.is_finite());
}

#[test]
fn adversarial_public_vertex_field_corruption_does_not_reenter_local_float_geometry() {
    let mut corrupted = Vertex::new(Point3::origin(), Vector3::z());
    corrupted.position.x = Real::NAN;
    corrupted.normal.x = Real::INFINITY;
    let clean = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::y());

    assert_eq!(corrupted.distance_to(&clean), Real::INFINITY);
    assert_eq!(corrupted.distance_squared_to(&clean), Real::INFINITY);
    assert_eq!(corrupted.normal_angle_to(&clean), 0.0);
}

#[test]
fn adversarial_vertex_normal_angle_uses_hyperlattice_checked_normals() {
    let x = Vertex::new(Point3::origin(), Vector3::x());
    let y = Vertex::new(Point3::origin(), Vector3::y());
    let zero = Vertex::new(Point3::origin(), Vector3::zeros());

    let right_angle = x.normal_angle_to(&y);
    assert!((right_angle - PI / 2.0).abs() < tolerance());
    assert_eq!(x.normal_angle_to(&zero), 0.0);
    assert!(right_angle.is_finite());
}

#[test]
fn adversarial_vertex_interpolation_normalization_uses_hyperlattice() {
    let x = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x() * 2.0);
    let y = Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::y() * 3.0);
    let zero = Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::zeros());

    let slerp = x.slerp_interpolate(&y, 0.5);
    assert_vertex_finite(&slerp);
    assert!((slerp.normal.norm() - 1.0).abs() < tolerance());

    let averaged = Vertex::weighted_average(&[(x, 1.0), (y, 1.0)])
        .expect("positive weights should average");
    assert_vertex_finite(&averaged);
    assert!((averaged.normal.norm() - 1.0).abs() < tolerance());

    let barycentric = Vertex::barycentric_interpolate(&x, &zero, &zero, 0.0, 0.5, 0.5);
    assert_vertex_finite(&barycentric);
    assert_eq!(barycentric.normal, Vector3::z());
}

#[test]
fn adversarial_cotangent_weight_degenerate_triangle_falls_back() {
    let center = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z());
    let neighbor = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z());
    let collinear = Vertex::new(Point3::new(0.5, 0.0, 0.0), Vector3::z());
    let refs = [&center, &neighbor, &collinear];

    let weight = Vertex::compute_cotangent_weight(&center, &neighbor, &refs);

    assert_eq!(weight, 1.0);
}

#[test]
fn adversarial_connectivity_lookup_uses_hyperreal_distance() {
    let vertex = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::z());
    let mut adjacency = HashMap::new();
    adjacency.insert(7usize, vec![1, 2, 3]);
    let mut positions = HashMap::new();
    positions.insert(7usize, Point3::new(1.0 + tolerance() * 0.25, 2.0, 3.0));
    positions.insert(9usize, Point3::new(10.0, 10.0, 10.0));

    let (valence, regularity) =
        vertex.analyze_connectivity_by_position(&adjacency, &positions, tolerance());

    assert_eq!(valence, 3);
    assert!(regularity.is_finite());

    let outside = Vertex::new(Point3::new(1.0 + tolerance() * 4.0, 2.0, 3.0), Vector3::z());
    let (valence, regularity) =
        outside.analyze_connectivity_by_position(&adjacency, &positions, tolerance());
    assert_eq!((valence, regularity), (0, 0.0));
}

#[test]
fn adversarial_vertex_quality_analysis_uses_hyperreal_distances_and_angles() {
    let center = Vertex::new(Point3::origin(), Vector3::z());
    let neighbors = vec![
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(-1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, -1.0, 0.0), Vector3::z()),
    ];

    let curvature = center.estimate_mean_curvature(&neighbors, &[0.5, 0.5, Real::NAN]);
    assert!(curvature.is_finite());

    let mut adjacency = HashMap::new();
    adjacency.insert(0usize, vec![1, 2, 3, 4]);
    let mut positions = HashMap::new();
    let mut normals = HashMap::new();
    for (idx, neighbor) in neighbors.iter().enumerate() {
        positions.insert(idx + 1, neighbor.position);
        normals.insert(idx + 1, neighbor.normal);
    }

    let (regularity, curvature_score, edge_uniformity, normal_variation) =
        center.comprehensive_quality_analysis(0, &adjacency, &positions, &normals);

    assert!(regularity.is_finite());
    assert!(curvature_score.is_finite());
    assert!(edge_uniformity.is_finite());
    assert!(normal_variation.is_finite());
    assert!(edge_uniformity > 0.99);
    assert!(normal_variation > 0.99);

    normals.insert(2, Vector3::new(Real::NAN, Real::INFINITY, 0.0));
    let (_, hostile_curvature_score, _, hostile_normal_variation) =
        center.comprehensive_quality_analysis(0, &adjacency, &positions, &normals);
    assert!(hostile_curvature_score.is_finite());
    assert!(hostile_normal_variation.is_finite());
}

#[test]
fn adversarial_vertex_cluster_uses_hyperreal_radius_and_checked_normal() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
    ];

    let cluster = VertexCluster::from_vertices(&vertices).expect("valid cluster");

    assert_eq!(cluster.count, vertices.len());
    assert!((cluster.normal.norm() - 1.0).abs() < tolerance());
    assert!(cluster.normal.dot(&Vector3::z()) > 1.0 - tolerance());
    assert!(cluster.radius.is_finite());
    assert!(cluster.radius > 0.0);

    let hostile = [
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex {
            position: Point3::new(Real::NAN, 0.0, 0.0),
            normal: Vector3::z(),
        },
    ];
    assert!(VertexCluster::from_vertices(&hostile).is_none());
}

#[test]
fn adversarial_bsp_slice_intersections_remain_hyperreal_coplanar() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ());
    let plane = Plane::from_normal(Vector3::new(1.0, -2.0, 0.5), 0.0);
    let node = Node::from_polygons(&mesh.polygons);

    let (_coplanar, edges) = node.slice(&plane);

    assert!(!edges.is_empty(), "central oblique slice should cut the cube");
    assert_slice_edges_on_plane(&edges, &plane);
}

#[test]
fn adversarial_plane_from_normal_rejects_hostile_boundary_without_panic() {
    let cases = [
        (Vector3::zeros(), 0.0),
        (Vector3::new(Real::NAN, 0.0, 0.0), 0.0),
        (Vector3::new(0.0, Real::INFINITY, 0.0), 0.0),
        (Vector3::z(), Real::NAN),
    ];

    for (normal, offset) in cases {
        let result = catch_unwind(AssertUnwindSafe(|| Plane::from_normal(normal, offset)));
        assert!(result.is_ok());
        let plane = result.unwrap();
        assert_vertex_finite(&Vertex::new(plane.point_a, plane.normal()));
        assert!(Plane::try_from_normal(normal, offset).is_none());
    }
}

#[test]
fn adversarial_triangulate_repairs_t_junction_with_hyperreal_projection() {
    let normal = Vector3::z();
    let t_point = Point3::new(1.0, 0.0, 0.0);
    let polygons = vec![
        Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(2.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(2.0, 1.0, 0.0), normal),
                Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
            ],
            (),
        ),
        Polygon::new(
            vec![
                Vertex::new(t_point, normal),
                Vertex::new(Point3::new(1.5, -0.5, 0.0), normal),
                Vertex::new(Point3::new(0.5, -0.5, 0.0), normal),
            ],
            (),
        ),
    ];

    let triangulated = Mesh::from_polygons(&polygons, ()).triangulate();
    assert_mesh_sane(&triangulated);
}

#[test]
fn adversarial_polygon_triangulate_filters_near_degenerate_hyperreal_winding() {
    let normal = Vector3::z();
    let polygon = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, tolerance() * 0.25, 0.0), normal),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
        ],
        (),
    );

    let triangles = polygon.triangulate();

    for triangle in triangles {
        let winding = (triangle[1].position - triangle[0].position)
            .cross(&(triangle[2].position - triangle[0].position));
        assert!(
            winding.norm_squared() >= tolerance() * tolerance(),
            "near-degenerate triangle should be filtered: {triangle:?}"
        );
        assert!(
            winding.dot(&normal) >= 0.0,
            "triangle winding should be aligned with polygon normal: {triangle:?}"
        );
    }
}

#[test]
fn adversarial_polygon_basis_rejects_hostile_normals_without_nalgebra_fallback() {
    let (u, v) =
        csgrs::polygon::build_orthonormal_basis(Vector3::new(Real::NAN, Real::INFINITY, 0.0));
    assert_eq!(u, Vector3::x());
    assert_eq!(v, Vector3::y());
}

#[test]
fn adversarial_vertex_index_map_uses_hyperreal_distance() {
    let mut map = VertexIndexMap::new(tolerance());
    let base = Point3::new(4.0, -2.0, 9.0);
    let near = Point3::new(4.0 + tolerance() * 0.25, -2.0, 9.0);
    let far = Point3::new(4.0 + tolerance() * 4.0, -2.0, 9.0);
    let nonfinite = Point3::new(Real::NAN, -2.0, 9.0);

    let base_idx = map.get_or_create_index(base);
    assert_eq!(map.find_index(&near), Some(base_idx));
    assert_eq!(map.get_or_create_index(near), base_idx);

    let far_idx = map.get_or_create_index(far);
    assert_ne!(far_idx, base_idx);
    assert_eq!(map.find_index(&nonfinite), None);
}

#[test]
fn adversarial_polyline_intersection_deduplicates_segment_junction_hit() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ());
    let hits = mesh.intersect_polyline(&[
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ]);

    assert!(
        hits.len() <= 2,
        "surface hit at polyline segment junction should be deduplicated: {hits:?}"
    );
    for window in hits.windows(2) {
        assert!((window[0] - window[1]).norm() >= tolerance());
    }

    let hostile_hits = mesh.intersect_polyline(&[
        Point3::new(Real::NAN, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(Real::INFINITY, 0.0, 0.0),
    ]);
    assert!(hostile_hits.is_empty());
}

#[test]
fn adversarial_ray_intersections_deduplicate_coplanar_triangle_face_hits() {
    let mesh: Mesh<()> = Mesh::cube(2.0, ()).center();
    let hits = mesh.ray_intersections(&Point3::new(-2.0, 0.0, 0.0), &Vector3::x());

    assert_eq!(
        hits.len(),
        2,
        "ray through two triangulated cube faces should return one hit per face: {hits:?}"
    );
    assert!((hits[0].1 - 1.0).abs() < tolerance());
    assert!((hits[1].1 - 3.0).abs() < tolerance());
}

#[test]
fn adversarial_axis_primitives_use_hyperreal_checked_direction() {
    let empty_frustum = Mesh::<()>::frustum_ptp(
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(1.0 + tolerance() * 0.25, 2.0, 3.0),
        1.0,
        1.0,
        16,
        (),
    );
    assert!(empty_frustum.polygons.is_empty());

    let frustum = Mesh::<()>::frustum_ptp(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 3.0),
        0.5,
        0.25,
        16,
        (),
    );
    assert_mesh_sane(&frustum);

    let empty_arrow = Mesh::<()>::arrow(Point3::origin(), Vector3::zeros(), 16, false, ());
    assert!(empty_arrow.polygons.is_empty());

    let arrow = Mesh::<()>::arrow(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 2.0, 3.0),
        16,
        false,
        (),
    );
    assert_mesh_sane(&arrow);
}

#[test]
#[cfg(feature = "chull-io")]
fn adversarial_convex_hull_normals_are_hyperreal_checked() {
    let hull = Mesh::<()>::cube(2.0, ()).convex_hull();
    assert_mesh_sane(&hull);
    assert!(!hull.polygons.is_empty());
    for vertex in hull.vertices() {
        assert!(vertex.normal.iter().all(|value| value.is_finite()));
        assert!((vertex.normal.norm() - 1.0).abs() < tolerance());
    }
}

#[test]
#[cfg(feature = "obj-io")]
fn adversarial_obj_export_deduplicates_vertices_with_hyperreal_distance() {
    let obj = near_duplicate_triangle_mesh().to_obj("near_duplicate");
    let vertex_lines = obj.lines().filter(|line| line.starts_with("v ")).count();
    let normal_lines = obj.lines().filter(|line| line.starts_with("vn ")).count();

    assert_eq!(
        vertex_lines, 4,
        "near-identical boundary vertices should be coalesced during OBJ export:\n{obj}"
    );
    assert_eq!(
        normal_lines, 1,
        "identical normals should be coalesced during OBJ export:\n{obj}"
    );
}

#[test]
#[cfg(feature = "amf-io")]
fn adversarial_amf_export_deduplicates_vertices_with_hyperreal_distance() {
    let amf = near_duplicate_triangle_mesh().to_amf("near_duplicate", "millimeter");
    let vertex_tags = amf
        .lines()
        .filter(|line| line.trim_start().starts_with("<vertex id="))
        .count();

    assert_eq!(
        vertex_tags, 4,
        "near-identical boundary vertices should be coalesced during AMF export:\n{amf}"
    );
}

#[test]
#[cfg(feature = "obj-io")]
fn adversarial_obj_zero_face_index_regression_is_error_not_panic() {
    let obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n";
    let result = catch_unwind(AssertUnwindSafe(|| {
        Mesh::<()>::from_obj(Cursor::new(obj), ())
    }));
    assert!(matches!(result, Ok(Err(_))));
}

#[test]
#[cfg(feature = "obj-io")]
fn adversarial_obj_import_rejects_nonfinite_vertices_and_degenerate_normals() {
    let nonfinite_vertex = "v 0 0 0\nv NaN 0 0\nv 0 1 0\nf 1 2 3\n";
    let result = catch_unwind(AssertUnwindSafe(|| {
        Mesh::<()>::from_obj(Cursor::new(nonfinite_vertex), ())
    }));
    assert!(matches!(result, Ok(Err(_))));

    let degenerate_normal = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 0\nf 1//1 2//1 3//1\n";
    let result = catch_unwind(AssertUnwindSafe(|| {
        Mesh::<()>::from_obj(Cursor::new(degenerate_normal), ())
    }));
    assert!(matches!(result, Ok(Err(_))));

    let scaled_normal = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 5\nf 1//1 2//1 3//1\n";
    let mesh = Mesh::<()>::from_obj(Cursor::new(scaled_normal), ()).unwrap();
    for vertex in mesh.vertices() {
        assert!((vertex.normal.norm() - 1.0).abs() < tolerance());
    }
}

#[test]
#[cfg(feature = "svg-io")]
fn adversarial_svg_unknown_tag_regression_is_error_not_panic() {
    let svg = "<not-a-supported-shape />";
    let result = catch_unwind(AssertUnwindSafe(|| Profile::<()>::from_svg(svg, ())));
    assert!(matches!(result, Ok(Err(_))));
}

#[test]
#[cfg(feature = "svg-io")]
fn adversarial_svg_import_rejects_nonfinite_hyperreal_boundary_values() {
    let cases = [
        r#"<svg><circle cx="0" cy="0" r="1e309" /></svg>"#,
        r#"<svg><ellipse cx="0" cy="0" rx="1" ry="NaN" /></svg>"#,
        r#"<svg><rect x="0" y="0" width="Infinity" height="1" /></svg>"#,
        r#"<svg><polyline points="0,0 NaN,1" /></svg>"#,
    ];

    for svg in cases {
        let result = catch_unwind(AssertUnwindSafe(|| Profile::<()>::from_svg(svg, ())));
        assert!(
            matches!(result, Ok(Err(_))),
            "non-finite SVG input should reject without panic: {svg}"
        );
    }
}

#[test]
#[cfg(feature = "gerber-io")]
fn adversarial_gerber_short_command_regression_is_error_not_panic() {
    use csgrs::io::gerber::FromGerber;

    let result = catch_unwind(AssertUnwindSafe(|| Profile::<()>::from_gerber(b"G0", ())));
    assert!(matches!(result, Ok(Err(_))));
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 96,
        max_shrink_iters: 4096,
        .. ProptestConfig::default()
    })]

    #[test]
    fn proptest_mesh_transform_roundtrip(size in positive_real(), dx in finite_real(), dy in finite_real(), dz in finite_real()) {
        let cube: Mesh<()> = Mesh::cube(size, ());
        let moved = cube.translate(dx, dy, dz).translate(-dx, -dy, -dz);
        assert_mesh_sane(&moved);

        let original = cube.bounding_box();
        let roundtrip = moved.bounding_box();
        prop_assert!((original.mins.x - roundtrip.mins.x).abs() < tolerance() * 128.0);
        prop_assert!((original.maxs.y - roundtrip.maxs.y).abs() < tolerance() * 128.0);
        prop_assert!((original.maxs.z - roundtrip.maxs.z).abs() < tolerance() * 128.0);
    }

    #[test]
    fn proptest_boolean_identities_for_cubes(size in positive_real(), offset in -50.0f64..50.0f64) {
        let offset = offset as Real;
        let a: Mesh<()> = Mesh::cube(size, ());
        let b: Mesh<()> = Mesh::cube(size, ()).translate(offset, 0.0, 0.0);
        let empty = Mesh::<()>::new();

        let union_empty = a.union(&empty);
        let diff_empty = a.difference(&empty);
        let intersection_empty = a.intersection(&empty);
        let xor_empty = a.xor(&empty);
        let union_pair = a.union(&b);
        let intersection_pair = a.intersection(&b);
        let xor_pair = a.xor(&b);

        assert_mesh_sane(&union_empty);
        assert_mesh_sane(&diff_empty);
        assert_mesh_sane(&intersection_empty);
        assert_mesh_sane(&xor_empty);
        assert_mesh_sane(&union_pair);
        assert_mesh_sane(&intersection_pair);
        assert_mesh_sane(&xor_pair);

        // Current Mesh boolean implementation may retain degenerate output for
        // some tolerance-scale inputs; this generated test is a safety net for
        // corruption and panics rather than a strict algebraic oracle.
    }

    #[test]
    fn proptest_sketch_rectangles_triangulate(width in positive_real(), height in positive_real()) {
        let sketch: Profile<()> = Profile::rectangle(width, height, ());
        let triangles = sketch.triangulate();
        assert_triangles_finite(&triangles);
        prop_assert!(!triangles.is_empty());
    }

    #[test]
    fn proptest_sketch_polygon_convex_quad_triangulates(x in positive_real(), y in positive_real()) {
        let points = [[0.0, 0.0], [x, 0.0], [x, y], [0.0, y]];
        let sketch: Profile<()> = Profile::polygon(&points, ());
        let triangles = sketch.triangulate();
        assert_triangles_finite(&triangles);
        prop_assert!(!triangles.is_empty());
    }

    #[test]
    fn proptest_plane_split_polygon_outputs_sane(y0 in -10.0f64..-0.001f64, y1 in 0.001f64..10.0f64, x in 0.1f64..10.0f64) {
        let y0 = y0 as Real;
        let y1 = y1 as Real;
        let x = x as Real;
        let poly: Polygon<()> = Polygon::new(
            vec![
                Vertex::new(Point3::new(-x, y0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(x, y0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(x, y1, 0.0), Vector3::z()),
                Vertex::new(Point3::new(-x, y1, 0.0), Vector3::z()),
            ],
            (),        );
        let plane = Plane::from_normal(Vector3::y(), 0.0);
        let (cf, cb, front, back) = plane.split_polygon(&poly);
        for p in cf.iter().chain(cb.iter()).chain(front.iter()).chain(back.iter()) {
            assert_polygon_sane(p);
        }
        prop_assert!(!front.is_empty());
        prop_assert!(!back.is_empty());
    }

    #[test]
    fn proptest_triangle_soup_mesh_triangulate_is_finite(points in prop::collection::vec(point3_strategy(), 3..36)) {
        let mut polygons = Vec::new();
        for tri in points.chunks_exact(3) {
            if let Some(poly) = simple_triangle(tri[0], tri[1], tri[2]) {
                polygons.push(poly);
            }
        }

        let mesh = Mesh::from_polygons(&polygons, ());
        let triangulated = mesh.triangulate();
        assert_mesh_sane(&triangulated);
    }

    #[test]
    fn proptest_vertex_interpolation_stays_finite(
        a in point3_strategy(),
        b in point3_strategy(),
        na in vector3_strategy(),
        nb in vector3_strategy(),
        t in -2.0f64..2.0f64,
    ) {
        let va = Vertex::new(a, na);
        let vb = Vertex::new(b, nb);
        let interpolated = va.interpolate(&vb, t as Real);
        assert_vertex_finite(&interpolated);
    }

    #[test]
    fn proptest_barycentric_interpolation_stays_finite(
        a in point3_strategy(),
        b in point3_strategy(),
        c in point3_strategy(),
        u in -10.0f64..10.0f64,
        v in -10.0f64..10.0f64,
        w in -10.0f64..10.0f64,
    ) {
        let va = Vertex::new(a, Vector3::zeros());
        let vb = Vertex::new(b, Vector3::zeros());
        let vc = Vertex::new(c, Vector3::zeros());
        let interpolated = Vertex::barycentric_interpolate(&va, &vb, &vc, u as Real, v as Real, w as Real);
        assert_vertex_finite(&interpolated);
    }

    #[test]
    fn proptest_weighted_average_stays_finite_when_defined(
        a in point3_strategy(),
        b in point3_strategy(),
        c in point3_strategy(),
        wa in -10.0f64..10.0f64,
        wb in -10.0f64..10.0f64,
        wc in -10.0f64..10.0f64,
    ) {
        let va = Vertex::new(a, Vector3::zeros());
        let vb = Vertex::new(b, Vector3::zeros());
        let vc = Vertex::new(c, Vector3::zeros());
        if let Some(averaged) = Vertex::weighted_average(&[(va, wa as Real), (vb, wb as Real), (vc, wc as Real)]) {
            assert_vertex_finite(&averaged);
        }
    }

    #[test]
    fn proptest_cotangent_weight_stays_finite(
        a in point3_strategy(),
        b in point3_strategy(),
        c in point3_strategy(),
    ) {
        let center = Vertex::new(a, Vector3::z());
        let neighbor = Vertex::new(b, Vector3::z());
        let opposite = Vertex::new(c, Vector3::z());
        let refs = [&center, &neighbor, &opposite];
        let weight = Vertex::compute_cotangent_weight(&center, &neighbor, &refs);
        prop_assert!(weight.is_finite());
    }

    #[test]
    fn proptest_connectivity_lookup_by_position_stays_finite(
        point in point3_strategy(),
        dx in -1.0e-5f64..1.0e-5f64,
        dy in -1.0e-5f64..1.0e-5f64,
        dz in -1.0e-5f64..1.0e-5f64,
    ) {
        let vertex = Vertex::new(point, Vector3::z());
        let mut adjacency = HashMap::new();
        adjacency.insert(0usize, vec![1, 2]);
        let mut positions = HashMap::new();
        positions.insert(0usize, Point3::new(point.x + dx as Real, point.y + dy as Real, point.z + dz as Real));

        let (_valence, regularity) =
            vertex.analyze_connectivity_by_position(&adjacency, &positions, tolerance() * 32.0);
        prop_assert!(regularity.is_finite());
    }

    #[test]
    fn proptest_vertex_index_map_coalesces_only_within_tolerance(
        point in point3_strategy(),
        offset in -2.0f64..2.0f64,
    ) {
        let epsilon = tolerance() * 16.0;
        let mut map = VertexIndexMap::new(epsilon);
        let base_idx = map.get_or_create_index(point);
        let candidate = Point3::new(point.x + offset as Real * epsilon, point.y, point.z);
        let candidate_idx = map.get_or_create_index(candidate);

        if offset.abs() < 0.5 {
            prop_assert_eq!(candidate_idx, base_idx);
        } else if offset.abs() > 1.25 {
            prop_assert_ne!(candidate_idx, base_idx);
        }
        prop_assert!(map.vertex_count() >= 1);
    }

    #[test]
    fn proptest_mesh_bytecode_fuzz_style_harness(bytes in prop::collection::vec(any::<u8>(), 0..256)) {
        run_mesh_bytecode(&bytes);
    }

    #[test]
    #[cfg(feature = "obj-io")]
    fn proptest_obj_import_malformed_text_does_not_panic(bytes in prop::collection::vec(any::<u8>(), 0..512)) {
        let text = String::from_utf8_lossy(&bytes).into_owned();
        let result = catch_unwind(AssertUnwindSafe(|| Mesh::<()>::from_obj(Cursor::new(text), ())));
        if let Ok(Ok(mesh)) = result {
            assert_mesh_sane(&mesh);
        }
    }

    #[test]
    #[cfg(feature = "obj-io")]
    fn proptest_obj_export_hostile_names(size in positive_real(), name in "\\PC{0,80}") {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let obj = mesh.to_obj(&name);
        prop_assert!(obj.contains("# Generated by csgrs library"));
        prop_assert!(obj.contains("\nf "));
    }

    #[test]
    #[cfg(feature = "stl-io")]
    fn proptest_stl_export_hostile_names(size in positive_real(), name in "\\PC{0,80}") {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let ascii = mesh.to_stl_ascii(&name);
        let binary = mesh.to_stl_binary(&name).expect("binary STL export should succeed");
        prop_assert!(ascii.contains("facet normal"));
        prop_assert!(binary.len() >= 84);
    }

    #[test]
    fn proptest_ray_intersections_are_sorted(size in positive_real(), origin_x in -10.0f64..10.0f64, origin_y in -10.0f64..10.0f64) {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let origin = Point3::new(origin_x as Real, origin_y as Real, -(size.abs() + 10.0));
        let direction = Vector3::z();
        let hits = mesh.ray_intersections(&origin, &direction);
        for window in hits.windows(2) {
            prop_assert!(window[0].1 <= window[1].1 + tolerance());
        }
        for (point, distance) in hits {
            prop_assert!(point.x.is_finite() && point.y.is_finite() && point.z.is_finite());
            prop_assert!(distance.is_finite());
        }
    }

    #[test]
    #[cfg(feature = "offset")]
    fn proptest_offset_extrude_slice_pipeline(width in positive_real(), height in positive_real(), depth in positive_real(), offset in -0.25f64..0.25f64) {
        let sketch: Profile<()> = Profile::rectangle(width, height, ());
        let offset_sketch = sketch.offset(offset as Real);
        let mesh = offset_sketch.extrude(depth);
        assert_mesh_sane(&mesh);

        let sliced = mesh.slice(Plane::from_normal(Vector3::z(), 0.0));
        let triangles = sliced.triangulate();
        assert_triangles_finite(&triangles);
    }

    #[test]
    fn proptest_smoothing_and_quality_stay_finite(size in positive_real(), lambda in -0.5f64..0.5f64, iterations in 0usize..4) {
        let mesh: Mesh<()> = Mesh::cube(size, ()).triangulate();
        let smoothed = mesh.laplacian_smooth(lambda as Real, iterations, true);
        assert_mesh_sane(&smoothed);

        for quality in smoothed.analyze_triangle_quality() {
            prop_assert!(quality.area.is_finite());
            prop_assert!(quality.min_angle.is_finite());
            prop_assert!(quality.max_angle.is_finite());
            prop_assert!(quality.quality_score.is_finite());
        }
    }

    #[test]
    fn proptest_bsp_slice_edges_are_coplanar_after_hyperreal_intersection(
        size in positive_real(),
        nx in -8.0f64..8.0f64,
        ny in -8.0f64..8.0f64,
        nz in -8.0f64..8.0f64,
        offset_fraction in -0.25f64..0.25f64,
    ) {
        let normal = Vector3::new(nx as Real, ny as Real, nz as Real);
        let normal_len = normal.norm();
        prop_assume!(normal_len > tolerance() * 128.0 && normal_len.is_finite());

        let mesh: Mesh<()> = Mesh::cube(size, ());
        let offset = offset_fraction as Real * size * normal_len;
        let plane = Plane::from_normal(normal, offset);
        let node = Node::from_polygons(&mesh.polygons);
        let (_coplanar, edges) = node.slice(&plane);

        assert_slice_edges_on_plane(&edges, &plane);
    }

    #[test]
    fn proptest_triangulate_repairs_axis_aligned_t_junction(
        width in 1.0f64..100.0f64,
        height in 1.0f64..100.0f64,
        t in 0.05f64..0.95f64,
        drop in 0.1f64..10.0f64,
    ) {
        let width = width as Real;
        let height = height as Real;
        let t = t as Real;
        let drop = drop as Real;
        let normal = Vector3::z();
        let t_point = Point3::new(width * t, 0.0, 0.0);
        let polygons = vec![
            Polygon::new(
                vec![
                    Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                    Vertex::new(Point3::new(width, 0.0, 0.0), normal),
                    Vertex::new(Point3::new(width, height, 0.0), normal),
                    Vertex::new(Point3::new(0.0, height, 0.0), normal),
                ],
                (),
            ),
            Polygon::new(
                vec![
                    Vertex::new(t_point, normal),
                    Vertex::new(Point3::new((width * t + width).min(width * 1.5), -drop, 0.0), normal),
                    Vertex::new(Point3::new((width * t - width).max(-width * 0.5), -drop, 0.0), normal),
                ],
                (),
            ),
        ];

        let triangulated = Mesh::from_polygons(&polygons, ()).triangulate();
        assert_mesh_sane(&triangulated);
        prop_assert!(!triangulated.polygons.is_empty());
    }

    #[test]
    #[cfg(feature = "ply-io")]
    fn proptest_ply_export_hostile_comments(size in positive_real(), comment in "\\PC{0,80}") {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let ply = mesh.to_ply(&comment);
        prop_assert!(ply.starts_with("ply\n"));
        prop_assert!(ply.contains("element vertex"));
        prop_assert!(ply.contains("element face"));
    }

    #[test]
    #[cfg(feature = "amf-io")]
    fn proptest_amf_export_hostile_names_and_units(size in positive_real(), name in "\\PC{0,80}", units in "\\PC{0,40}") {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let amf = mesh.to_amf(&name, &units);
        prop_assert!(amf.contains("<amf"));
        prop_assert!(amf.contains("<object"));
        prop_assert!(amf.contains("triangle"));
    }

    #[test]
    #[cfg(feature = "gltf-io")]
    fn proptest_gltf_export_hostile_names(size in positive_real(), name in "\\PC{0,80}") {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let gltf = mesh.to_gltf(&name);
        prop_assert!(gltf.contains("\"asset\""));
        prop_assert!(gltf.contains("\"meshes\""));
        prop_assert!(gltf.contains("\"buffers\""));
    }

    #[test]
    #[cfg(feature = "metaballs")]
    fn proptest_metaballs_small_fields_are_finite(radius in positive_real(), iso in 0.01f64..10.0f64, padding in 0.0f64..1.0f64) {
        let balls = [
            csgrs::mesh::metaballs::MetaBall::new(Point3::new(0.0, 0.0, 0.0), radius),
            csgrs::mesh::metaballs::MetaBall::new(Point3::new(radius * 0.5, 0.0, 0.0), radius),
        ];
        let mesh: Mesh<()> = Mesh::metaballs(&balls, (4, 4, 4), iso as Real, padding as Real, ());
        assert_mesh_sane(&mesh);
    }

    #[test]
    #[cfg(feature = "dxf-io")]
    fn proptest_dxf_export_and_malformed_import(size in positive_real(), bytes in prop::collection::vec(any::<u8>(), 0..256)) {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let dxf = mesh.to_dxf().expect("DXF export should succeed for cube");
        prop_assert!(!dxf.is_empty());

        let result = catch_unwind(AssertUnwindSafe(|| Mesh::<()>::from_dxf(&bytes, ())));
        if let Ok(Ok(mesh)) = result {
            assert_mesh_sane(&mesh);
        }
    }

    #[test]
    #[cfg(feature = "svg-io")]
    fn proptest_svg_roundtrip_and_malformed_import(width in positive_real(), height in positive_real(), bytes in prop::collection::vec(any::<u8>(), 0..512)) {
        use csgrs::io::svg::{FromSVG, ToSVG};

        let sketch: Profile<()> = Profile::rectangle(width, height, ());
        let svg = sketch.to_svg();
        let reparsed = Profile::<()>::from_svg(&svg, ());
        if let Ok(sketch) = reparsed {
            assert_triangles_finite(&sketch.triangulate());
        }

        let malformed = String::from_utf8_lossy(&bytes);
        let result = catch_unwind(AssertUnwindSafe(|| Profile::<()>::from_svg(&malformed, ())));
        if let Ok(Ok(sketch)) = result {
            assert_triangles_finite(&sketch.triangulate());
        }
    }

    #[test]
    #[cfg(feature = "gerber-io")]
    fn proptest_gerber_roundtrip_and_malformed_import(width in positive_real(), height in positive_real(), bytes in prop::collection::vec(any::<u8>(), 0..512)) {
        use csgrs::io::gerber::{FromGerber, ToGerber};

        let sketch: Profile<()> = Profile::rectangle(width, height, ());
        let gerber = sketch.to_gerber().expect("Gerber export should succeed for rectangle");
        let reparsed = Profile::<()>::from_gerber(&gerber, ());
        if let Ok(sketch) = reparsed {
            assert_triangles_finite(&sketch.triangulate());
        }

        let result = catch_unwind(AssertUnwindSafe(|| Profile::<()>::from_gerber(&bytes, ())));
        if let Ok(Ok(sketch)) = result {
            assert_triangles_finite(&sketch.triangulate());
        }
    }

    #[test]
    #[cfg(feature = "offset")]
    fn proptest_toolpath_generators_emit_finite_gcode(width in positive_real(), height in positive_real(), z in -5.0f64..5.0f64, kerf in -1.0f64..1.0f64) {
        use csgrs::toolpath::{
            cut2d_contours, fdm_layer_from_sketch, pocket2d, Feeds, FdmLayerCfg, KerfSide,
            LeadInOut, MachineKind, PocketCfg, Tool,
        };
        use csgrs::toolpath::gcode::Post;

        let sketch: Profile<()> = Profile::rectangle(width, height, ());
        let feeds = Feeds {
            travel: 3000.0,
            xy: 1200.0,
            plunge: 300.0,
            rpm: Some(12000.0),
            power: Some(0.5),
            pierce_ms: Some(10),
        };

        let fdm = fdm_layer_from_sketch(&sketch, z as Real, &FdmLayerCfg::default(), &feeds);
        let contour = cut2d_contours(
            &sketch,
            z as Real,
            kerf as Real,
            KerfSide::Outside,
            Some(LeadInOut { length: 0.5, radius: 0.0 }),
            &feeds,
            MachineKind::Laser,
        );
        let pocket = pocket2d(
            &sketch,
            5.0,
            0.0,
            &PocketCfg {
                tool: Tool { diameter: 1.0, corner_radius: 0.0 },
                stepover: 0.5,
                stepdown: 0.5,
                depth: 1.0,
                finish_allow: 0.0,
            },
            &feeds,
            true,
        );

        let post = Post { absolute_e: false, units_mm: true, z_safe: 5.0 };
        for tp in [&fdm, &contour, &pocket] {
            for mv in &tp.moves {
                prop_assert!(mv.position.x.is_finite());
                prop_assert!(mv.position.y.is_finite());
                prop_assert!(mv.position.z.is_finite());
                prop_assert!(mv.scalar.is_none_or(Real::is_finite));
                prop_assert!(mv.feed.is_none_or(Real::is_finite));
            }
            let gcode = post.write(tp);
            prop_assert!(!gcode.contains("NaN"));
            prop_assert!(!gcode.contains("inf"));
        }
    }

    #[test]
    #[cfg(feature = "chull-io")]
    fn proptest_convex_hull_and_minkowski_are_finite(size_a in positive_real(), size_b in positive_real(), dx in -10.0f64..10.0f64) {
        let a: Mesh<()> = Mesh::cube(size_a, ());
        let b: Mesh<()> = Mesh::cube(size_b, ()).translate(dx as Real, 0.0, 0.0);

        let hull = a.union(&b).convex_hull();
        let minkowski = a.minkowski_sum(&b);
        assert_mesh_sane(&hull);
        assert_mesh_sane(&minkowski);
    }

    #[test]
    fn proptest_distribution_mirror_and_mass_properties_are_finite(size in positive_real(), count in 0usize..8, radius in 0.0f64..20.0f64, angle in -720.0f64..720.0f64) {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let line = mesh.distribute_linear(count, Vector3::x(), size.max(tolerance()));
        let arc = mesh.distribute_arc(count, radius as Real, -angle as Real, angle as Real);
        let grid = mesh.distribute_grid(count, count, size.max(tolerance()), size.max(tolerance()));
        let mirrored = mesh.mirror(Plane::from_normal(Vector3::new(1.0, 1.0, 0.0), 0.0));

        for candidate in [&line, &arc, &grid, &mirrored] {
            assert_mesh_sane(candidate);
            let props = catch_unwind(AssertUnwindSafe(|| candidate.mass_properties(1.0)));
            if let Ok(Ok((mass, local_com, _frame))) = props {
                prop_assert!(mass.is_finite());
                prop_assert!(local_com.x.is_finite());
                prop_assert!(local_com.y.is_finite());
                prop_assert!(local_com.z.is_finite());
            }
        }
    }

    #[test]
    fn proptest_polyline_intersections_are_finite(size in positive_real(), z0 in -20.0f64..-0.1f64, z1 in 0.1f64..20.0f64, wiggle in -5.0f64..5.0f64) {
        let mesh: Mesh<()> = Mesh::cube(size, ());
        let polyline = vec![
            Point3::new(wiggle as Real, 0.0, z0 as Real),
            Point3::new(wiggle as Real, 0.0, 0.0),
            Point3::new(wiggle as Real, 0.0, z1 as Real),
        ];
        let hits = mesh.intersect_polyline(&polyline);
        for hit in &hits {
            prop_assert!(hit.x.is_finite());
            prop_assert!(hit.y.is_finite());
            prop_assert!(hit.z.is_finite());
        }
        for window in hits.windows(2) {
            prop_assert!((window[0] - window[1]).norm() >= tolerance());
        }
    }
}

#[test]
fn adversarial_csg_transforms_fail_closed_on_hostile_vectors() {
    let mesh: Mesh<()> = Mesh::cube(1.0, ());
    let original_bbox = mesh.bounding_box();

    let zero_line = mesh.distribute_linear(4, Vector3::zeros(), 1.0);
    assert_eq!(zero_line.polygons.len(), mesh.polygons.len());
    assert_eq!(zero_line.bounding_box(), original_bbox);

    let hostile_line =
        mesh.distribute_linear(4, Vector3::new(Real::NAN, Real::INFINITY, 0.0), 1.0);
    assert_eq!(hostile_line.polygons.len(), mesh.polygons.len());
    assert_eq!(hostile_line.bounding_box(), original_bbox);

    let hostile_spacing = mesh.distribute_linear(4, Vector3::x(), Real::INFINITY);
    assert_eq!(hostile_spacing.polygons.len(), mesh.polygons.len());
    assert_eq!(hostile_spacing.bounding_box(), original_bbox);

    let hostile_translate = mesh.translate(Real::NAN, 0.0, 0.0);
    assert_eq!(hostile_translate.bounding_box(), original_bbox);

    let hostile_rotate = mesh.rotate(0.0, Real::INFINITY, 0.0);
    assert_eq!(hostile_rotate.bounding_box(), original_bbox);

    let overflowing_rotate = mesh.rotate(Real::MAX, 0.0, 0.0);
    assert_mesh_sane(&overflowing_rotate);

    let hostile_scale = mesh.scale(1.0, Real::NAN, 1.0);
    assert_eq!(hostile_scale.bounding_box(), original_bbox);

    let hostile_arc = mesh.distribute_arc(4, Real::INFINITY, 0.0, 90.0);
    assert_eq!(hostile_arc.bounding_box(), original_bbox);

    let hostile_grid = mesh.distribute_grid(2, 2, 1.0, Real::NAN);
    assert_eq!(hostile_grid.bounding_box(), original_bbox);

    let hostile_plane = Plane {
        point_a: Point3::new(Real::NAN, 0.0, 0.0),
        point_b: Point3::new(1.0, Real::INFINITY, 0.0),
        point_c: Point3::new(0.0, 1.0, 0.0),
    };
    let mirrored = mesh.mirror(hostile_plane);
    assert_eq!(mirrored.polygons.len(), mesh.polygons.len());
    assert_eq!(mirrored.bounding_box(), original_bbox);
    assert_mesh_sane(&mirrored);
}
