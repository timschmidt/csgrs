use csgrs::{
    TriangleMesh, curve,
    solid::{self, SolidExt},
};
use hyperlattice::{Point3, Real};
use hypermesh::Triangle;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn exact_near_vertices_are_not_tolerance_merged() {
    let mesh = solid::cube(r(1.0));
    let near = p3(1.0e-12, 0.0, 0.0);

    assert!(!solid::contains_point(&mesh, &near).unwrap());
}

#[test]
fn public_mesh_construction_is_the_native_indexed_triangle_carrier() {
    let mesh = TriangleMesh::new(
        vec![p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0)],
        vec![Triangle::new(0, 1, 2)],
    );

    assert_eq!(mesh.triangles.len(), 1);
    assert_eq!(mesh.triangles[0].indices(), [0, 1, 2]);
}

#[test]
fn boolean_pipeline_accepts_hyperreal_transforms() {
    let a = solid::center(&solid::cube(r(2.0)));
    let b = solid::cube(r(1.1)).translated(r(0.4), r(0.2), r(0.1));

    let direct = hypermesh::boolean_native_meshes(
        &[&a, &b],
        hypermesh::BooleanOp::Difference,
        hypermesh::EmberConfig::default(),
    )
    .expect("direct difference");
    assert!(direct.has_unique_nondegenerate_triangles());
    let difference = a.try_difference(&b).expect("difference");
    let intersection = b.try_intersection(&a).expect("intersection");
    let result = difference.try_union(&intersection).expect("union");

    for mesh in [&difference, &intersection, &result] {
        assert!(mesh.has_unique_nondegenerate_triangles());
        assert!(mesh.is_closed_manifold_geometry());
    }
    assert!(!result.triangles.is_empty());
    let bounds = solid::bounding_box(&result);
    assert_eq!(
        hyperlimit::compare_reals(&bounds.maxs.x, &bounds.mins.x).value(),
        Some(std::cmp::Ordering::Greater)
    );
}

#[test]
fn translated_union_does_not_emit_origin_fallback_vertices() {
    let left = solid::cube(r(1.0)).translated(r(10.0), r(0.0), r(0.0));
    let right = solid::cube(r(1.0)).translated(r(13.0), r(0.0), r(0.0));

    let result = left.try_union(&right).expect("union");

    assert!(!result.triangles.is_empty());
    for point in result.positions.iter() {
        let x = point
            .x
            .to_f64_lossy()
            .expect("cube coordinates should export to finite f64");
        assert!(
            x >= 10.0,
            "union emitted an un-translated or origin fallback vertex: x={x}, position={point:?}",
        );
    }
}

#[test]
fn exact_near_plane_translation_is_preserved_by_native_geometry() {
    let shift = r(1.0e-12);
    let mesh = solid::cube(r(2.0)).translated(shift.clone(), r(0.0), r(0.0));
    assert_eq!(solid::bounding_box(&mesh).mins.x, shift);
}

#[test]
fn curve_offset_and_extrude_keep_hyperreal_scalars() {
    let region = curve::square(r(2.0));
    #[cfg(feature = "offset")]
    let region = curve::offset(&region, r(0.125)).expect("offset");
    let mesh = curve::extrude(&region, r(0.75));

    assert!(!mesh.triangles.is_empty());
    assert!(matches!(
        hyperlimit::compare_reals(&solid::bounding_box(&mesh).maxs.z, &r(0.75)).value(),
        Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
    ));
}
