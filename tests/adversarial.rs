//! Fixed native modeling regressions for exact ingress, transforms and downstream consumption.
//! The witnesses use boundary classification, stored indices, mesh predicates and
//! bounds; sampled profiles and discarded local receipts retain their usual API scope.

use csgrs::{
    TriangleMesh, curve,
    solid::{self, SolidExt},
};
use hyperlattice::{Point3, Real};
use hyperlimit::PredicatePolicy;
use hypermesh::{MeshContext, Triangle};

/// Strict policy for the direct arrangement and geometric validation queries.
const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

/// Imports the exact binary64 value; decimal literals are not interpreted as exact decimals.
/// Panics if a fixture supplies NaN or infinity.
fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

/// Constructs a native point by exact finite conversion of each fixture coordinate.
fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
/// Requires strict-interior containment to exclude `(1e-12, 0, 0)` on a cube edge.
/// The point is on the boundary, so this assertion alone does not detect vertex merging.
fn exact_near_vertices_are_not_tolerance_merged() {
    let mesh = solid::cube(r(1.0)).expect("valid primitive fixture");
    let near = p3(1.0e-12, 0.0, 0.0);

    assert!(!solid::contains_point(&mesh, &near).unwrap());
}

#[test]
/// Checks that the public native carrier retains one triangle's authored index triple.
/// This open triangle is not submitted to closed-solid validation.
fn public_mesh_construction_is_the_native_indexed_triangle_carrier() {
    let mesh = TriangleMesh::new(
        vec![p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0)],
        vec![Triangle::new(0, 1, 2)],
    );

    assert_eq!(mesh.triangles.len(), 1);
    assert_eq!(mesh.triangles[0].indices(), [0, 1, 2]);
}

#[test]
/// Exercises direct difference and a wrapper difference/intersection/union chain on shifted boxes.
/// Checks face uniqueness, nondegeneracy and geometric edge pairing for the wrapper results,
/// then nonemptiness and positive X extent; it does not independently prove the final set identity.
fn boolean_pipeline_accepts_hyperreal_transforms() {
    let a = solid::center(&solid::cube(r(2.0)).expect("valid primitive fixture"))
        .expect("valid modeling fixture");
    let b = solid::cube(r(1.1))
        .expect("valid primitive fixture")
        .translated(r(0.4), r(0.2), r(0.1))
        .expect("valid transform fixture");

    let batch = hypermesh::boolean(
        &CONTEXT,
        &[a.as_ref(), b.as_ref()],
        hypermesh::BooleanProgram::Operation(hypermesh::BooleanOp::Difference),
    )
    .expect("direct difference")
    .into_value();
    let direct = batch
        .into_triangle_meshes()
        .expect("bounded direct difference")
        .pop()
        .expect("one direct result");
    assert!(
        direct
            .has_unique_nondegenerate_triangles(&CONTEXT)
            .unwrap()
            .into_value()
    );
    let difference = a.try_difference(&b).expect("difference");
    let intersection = b.try_intersection(&a).expect("intersection");
    let result = difference.try_union(&intersection).expect("union");

    for mesh in [&difference, &intersection, &result] {
        assert!(
            mesh.has_unique_nondegenerate_triangles(&CONTEXT)
                .unwrap()
                .into_value()
        );
        assert!(
            mesh.is_closed_manifold_geometry(&CONTEXT)
                .unwrap()
                .into_value()
        );
    }
    assert!(!result.triangles.is_empty());
    let bounds = solid::bounding_box(&result)
        .expect("decidable fixture bounds")
        .expect("nonempty fixture");
    assert_eq!(
        hyperlimit::compare_reals(&bounds.maxs.x, &bounds.mins.x, PredicatePolicy::STRICT)
            .value(),
        Some(std::cmp::Ordering::Greater)
    );
}

#[test]
/// Unites two separated translated cubes and requires every projected vertex X to be at least ten.
/// The binary64 check targets gross origin-fallback errors, not exact equality of the union set.
fn translated_union_does_not_emit_origin_fallback_vertices() {
    let left = solid::cube(r(1.0))
        .expect("valid primitive fixture")
        .translated(r(10.0), r(0.0), r(0.0))
        .expect("valid transform fixture");
    let right = solid::cube(r(1.0))
        .expect("valid primitive fixture")
        .translated(r(13.0), r(0.0), r(0.0))
        .expect("valid transform fixture");

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
/// Checks the minimum X bound retains the exact imported binary64 `1e-12` translation.
fn exact_near_plane_translation_is_preserved_by_native_geometry() {
    let shift = r(1.0e-12);
    let mesh = solid::cube(r(2.0))
        .expect("valid primitive fixture")
        .translated(shift.clone(), r(0.0), r(0.0))
        .expect("valid transform fixture");
    assert_eq!(
        solid::bounding_box(&mesh)
            .expect("decidable fixture bounds")
            .expect("nonempty fixture")
            .mins
            .x,
        shift
    );
}

#[test]
/// Extrudes a square, optionally after a bounded-miter offset when `offset` is enabled.
/// Requires nonempty triangles and maximum Z at least the authored height; no receipt
/// or full offset/extrusion set equality is asserted here.
fn curve_offset_and_extrude_keep_hyperreal_scalars() {
    let region = curve::square(r(2.0)).expect("valid planar fixture");
    #[cfg(feature = "offset")]
    let region = region
        .offset(
            r(0.125),
            &hypercurve::OffsetCornerStyle2::Miter { limit: r(4.0) },
        )
        .expect("offset");
    let mesh = curve::try_extrude(&region, r(0.75), csgrs::GeometryContext::STRICT)
        .expect("extrude")
        .into_value();

    assert!(!mesh.triangles.is_empty());
    assert!(matches!(
        hyperlimit::compare_reals(
            &solid::bounding_box(&mesh)
                .expect("decidable fixture bounds")
                .expect("nonempty fixture")
                .maxs
                .z,
            &r(0.75),
            PredicatePolicy::STRICT,
        )
        .value(),
        Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
    ));
}
