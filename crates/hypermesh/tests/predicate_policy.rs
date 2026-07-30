use hypermesh::{
    Aabb, Classification, HypermeshError, MeshCertainty, MeshContext, Plane, Point3,
    PredicatePolicy, Real, TriangleMesh, classify_point,
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
