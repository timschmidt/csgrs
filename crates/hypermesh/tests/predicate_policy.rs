use std::num::NonZeroU32;

use hyperlattice::Matrix4;
use hypermesh::{
    Aabb, Classification, HypermeshError, MeshCertainty, MeshContext, Plane, Point3,
    PredicatePolicy, Real, Triangle, TriangleMesh, classify_point,
};

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
const APPROXIMATE: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn terminal_equality() -> (Real, Real) {
    (Real::pi() + Real::e(), Real::e() + Real::pi())
}

#[test]
fn point_classification_reports_terminal_policy_consumption() {
    let (left, right) = terminal_equality();
    let plane = Plane::axis_aligned(0, left);
    let point = Point3::new(right, Real::zero(), Real::zero());

    assert!(matches!(
        classify_point(&STRICT, &point, &plane),
        Err(HypermeshError::PredicateUndecided { .. })
    ));

    let outcome = classify_point(&APPROXIMATE, &point, &plane).unwrap();
    assert_eq!(outcome.value, Classification::On);
    assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
}

#[test]
fn multi_comparison_geometry_aggregates_terminal_certainty() {
    let (left, right) = terminal_equality();
    let bounds = Aabb::new(
        Point3::new(left.clone(), Real::zero(), Real::zero()),
        Point3::new(&left + &Real::one(), Real::one(), Real::one()),
    );
    let point = Point3::new(right, Real::one(), Real::one());

    assert!(matches!(
        bounds.contains_point(&STRICT, &point),
        Err(HypermeshError::PredicateUndecided { .. })
    ));

    let outcome = bounds.contains_point(&APPROXIMATE, &point).unwrap();
    assert!(outcome.value);
    assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
}

#[test]
fn exact_rational_work_stays_certified_under_both_policies() {
    let plane = Plane::axis_aligned(0, Real::from(2));
    let point = Point3::new(Real::from(2), Real::from(3), Real::from(5));

    for context in [STRICT, APPROXIMATE] {
        let outcome = classify_point(&context, &point, &plane).unwrap();
        assert_eq!(outcome.value, Classification::On);
        assert_eq!(outcome.certainty, MeshCertainty::Certified);
    }
}

#[test]
fn native_bounds_do_not_bypass_the_selected_policy() {
    let (left, right) = terminal_equality();
    let mesh = TriangleMesh::new(
        vec![
            Point3::new(left, Real::zero(), Real::zero()),
            Point3::new(right, Real::one(), Real::one()),
        ],
        Vec::new(),
    );

    assert!(matches!(
        mesh.exact_bounds(&STRICT),
        Err(HypermeshError::PredicateUndecided { .. })
    ));

    let outcome = mesh.exact_bounds(&APPROXIMATE).unwrap();
    assert!(outcome.value.is_some());
    assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
}

#[test]
fn reflection_normal_domain_obeys_the_selected_policy() {
    let (left, right) = terminal_equality();
    let plane = Plane::from_coefficients(left - right, Real::zero(), Real::zero(), Real::zero());

    assert!(matches!(
        plane.reflection_matrix(&STRICT),
        Err(HypermeshError::PredicateUndecided { .. })
    ));
    assert_eq!(
        plane.reflection_matrix(&APPROXIMATE),
        Err(HypermeshError::DegeneratePointSet)
    );
}

#[test]
fn projective_transform_finiteness_obeys_the_selected_policy() {
    let (left, right) = terminal_equality();
    let delta = left - right;
    let zero = Real::zero();
    let matrix = Matrix4::from_row_major([
        Real::one(),
        zero.clone(),
        zero.clone(),
        zero.clone(),
        zero.clone(),
        Real::one(),
        zero.clone(),
        zero.clone(),
        zero.clone(),
        zero.clone(),
        Real::one(),
        zero.clone(),
        zero.clone(),
        zero.clone(),
        zero,
        delta,
    ]);
    let mesh = TriangleMesh::new(vec![Point3::origin()], Vec::new());

    assert!(matches!(
        mesh.try_transformed(&STRICT, &matrix),
        Err(HypermeshError::PredicateUndecided { .. })
    ));
    assert_eq!(
        mesh.try_transformed(&APPROXIMATE, &matrix),
        Err(HypermeshError::PointAtInfinity)
    );
}

#[test]
fn native_edit_operations_reject_invalid_indices() {
    let mesh = TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 0)]);
    let expected = HypermeshError::VertexIndexOutOfBounds {
        index: 1,
        vertex_count: 1,
    };

    assert_eq!(mesh.adjacency().unwrap_err(), expected);
    assert_eq!(mesh.connectivity_counts().unwrap_err(), expected);
    assert_eq!(
        mesh.subdivide_triangles(NonZeroU32::MIN).unwrap_err(),
        expected
    );
    assert_eq!(
        mesh.laplacian_smooth(&Real::one(), 1).unwrap_err(),
        expected
    );
    assert_eq!(
        mesh.taubin_smooth(&Real::one(), &-Real::one(), 1)
            .unwrap_err(),
        expected
    );
    assert_eq!(
        mesh.dihedral_angle(&STRICT, Triangle::new(0, 1, 0), Triangle::new(0, 0, 0))
            .unwrap_err(),
        expected
    );
}

#[test]
fn dihedral_normal_domain_obeys_the_selected_policy() {
    let (left, right) = terminal_equality();
    let mesh = TriangleMesh::new(
        vec![
            Point3::origin(),
            Point3::new(left - right, Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::one(), Real::zero()),
        ],
        vec![Triangle::new(0, 1, 2)],
    );
    let triangle = mesh.triangles[0];

    assert!(matches!(
        mesh.dihedral_angle(&STRICT, triangle, triangle),
        Err(HypermeshError::PredicateUndecided { .. })
    ));
    assert_eq!(
        mesh.dihedral_angle(&APPROXIMATE, triangle, triangle),
        Err(HypermeshError::DegeneratePointSet)
    );
}
