use std::cmp::Ordering;

use hyperbrep::{
    Aabb, Curve3, GeometryError, ModelBuilder, Point3, Real, STRICT_PREDICATES, Surface,
    SurfaceBounds,
};
use hyperlimit::compare_reals;

#[test]
fn bounds_retry_after_scalar_evidence_strengthens() {
    // The expressions stay exact. Their positive separation is initially below
    // the strict predicate's refinement budget; explicit later refinement proves it.
    let tiny: Real = "1e-2000".parse().unwrap();
    let first = Real::zero();
    let second = (Real::pi() + &tiny + &tiny).exp().unwrap() - (Real::pi() + tiny).exp().unwrap();
    assert_eq!(
        compare_reals(&first, &second, STRICT_PREDICATES).value(),
        None
    );
    let start = Point3::new(first.clone(), Real::zero(), Real::zero());
    let end = Point3::new(second.clone(), Real::one(), Real::zero());
    let curve =
        Curve3::rational_bezier(vec![start.clone(), end.clone()], vec![Real::one(); 2]).unwrap();
    let surface = Surface::rational_bezier(
        vec![
            vec![start.clone(), end.clone()],
            vec![
                Point3::new(first.clone(), Real::zero(), Real::one()),
                Point3::new(second.clone(), Real::one(), Real::one()),
            ],
        ],
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap();
    let mut builder = ModelBuilder::new();
    builder.vertex(start.clone()).unwrap();
    builder.vertex(end.clone()).unwrap();
    let model = builder.finish().unwrap();
    for error in [
        curve.bounds().unwrap_err(),
        surface.bounds().unwrap_err(),
        model.bounds().unwrap_err(),
    ] {
        assert!(matches!(error, GeometryError::PredicateUnresolved { .. }));
    }
    assert_eq!(
        second.refine_sign_until(-8192),
        Some(hyperreal::RealSign::Positive)
    );
    assert_eq!(
        compare_reals(&first, &second, STRICT_PREDICATES).value(),
        Some(Ordering::Less)
    );
    let retried = [
        curve.bounds().is_ok(),
        surface.bounds().is_ok(),
        model.bounds().is_ok(),
    ];
    assert_eq!(
        retried, [true; 3],
        "temporary errors must not freeze bounds"
    );

    let expected_curve = Aabb::new(start.clone(), end.clone());
    let expected_surface = Aabb::new(start, Point3::new(second, Real::one(), Real::one()));
    assert_eq!(curve.bounds().unwrap(), expected_curve);
    let SurfaceBounds::Bounded(surface_bounds) = surface.bounds().unwrap() else {
        panic!("finite control net must be bounded");
    };
    assert_eq!(*surface_bounds, expected_surface);
    assert_eq!(model.bounds().unwrap(), Some(expected_curve.clone()));

    // Shared successful caches retain the same certified endpoints.
    assert_eq!(curve.clone().bounds().unwrap(), expected_curve);
    let SurfaceBounds::Bounded(cloned_bounds) = surface.clone().bounds().unwrap() else {
        panic!("shared finite control net must be bounded");
    };
    assert_eq!(*cloned_bounds, expected_surface);
    assert_eq!(model.clone().bounds().unwrap(), Some(expected_curve));
}
