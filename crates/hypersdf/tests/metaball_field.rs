#[path = "support/uncertainty.rs"]
mod test_uncertainty;

use hyperlattice::{Point3, Real};
use hyperlimit::PredicatePolicy;
use hypersdf::{Metaball, MetaballField, SdfEvaluationError, SdfMetricStatus};
use std::{cmp::Ordering, error::Error};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    test_uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
fn inverse_square_values_remain_exact_at_near_and_far_points() {
    let ball = Metaball::new(p(0, 0, 0), 1.into());
    assert_eq!(ball.influence(&p(100, 0, 0)).unwrap(), q(1, 10000));
    // Preserve the former tolerance/4 fixture without perturbing its denominator.
    let tiny = Real::try_from(1.0e-9).unwrap() * Real::try_from(0.25).unwrap();
    let value = ball
        .influence(&Point3::new(tiny, 0.into(), 0.into()))
        .unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&value, &Real::from(1_000_000_000), PredicatePolicy::STRICT)
            .value(),
        Some(Ordering::Greater)
    );
    let error = ball.influence(&p(0, 0, 0)).unwrap_err();
    assert_eq!(
        error.source().unwrap().downcast_ref::<hyperreal::Problem>(),
        Some(&hyperreal::Problem::DivideByZero)
    );
}

#[test]
fn normalized_values_match_independent_rationals_and_center_limits() {
    let balls = [Metaball::new(p(0, 0, 0), 2.into())];
    let field = MetaballField::new(&balls, 1.into()).unwrap();
    // Distance squared 9 gives I=4/9 and normalized value 5/13.
    assert_eq!(field.evaluate_point(&p(1, 2, 2)).unwrap(), q(5, 13));
    assert_eq!(field.evaluate_point(&p(2, 0, 0)).unwrap(), Real::zero());
    assert_eq!(field.evaluate_point(&p(0, 0, 0)).unwrap(), -Real::one());
    assert_eq!(field.metric_status(), SdfMetricStatus::SignEquivalent);
    assert_eq!(field.balls(), &balls);
    assert_eq!(field.iso_value(), &Real::one());
}

#[test]
fn iso_scaling_and_coincident_sources_preserve_the_normalized_field() {
    let single = [Metaball::new(p(0, 0, 0), 1.into())];
    let scaled = [Metaball::new(p(0, 0, 0), 3.into())];
    let duplicate = [single[0].clone(), single[0].clone()];
    let a = MetaballField::new(&single, 1.into()).unwrap();
    let b = MetaballField::new(&scaled, 9.into()).unwrap();
    let c = MetaballField::new(&duplicate, 2.into()).unwrap();
    for point in [p(0, 0, 0), p(1, 0, 0), p(2, 0, 0), p(0, 3, 4)] {
        let value = a.evaluate_point(&point).unwrap();
        assert_eq!(b.evaluate_point(&point).unwrap(), value);
        assert_eq!(c.evaluate_point(&point).unwrap(), value);
    }
    assert_eq!(a.evaluate_point(&p(2, 0, 0)).unwrap(), q(3, 5));
    assert_eq!(
        a.sampling_bounds(&0.into()).unwrap(),
        b.sampling_bounds(&0.into()).unwrap()
    );
    assert_eq!(
        a.sampling_bounds(&0.into()).unwrap(),
        c.sampling_bounds(&0.into()).unwrap()
    );
}

#[test]
fn conservative_bounds_have_exact_exterior_samples() {
    let balls = [Metaball::new(p(0, 0, 0), 1.into())];
    let field = MetaballField::new(&balls, 1.into()).unwrap();
    let bounds = field.sampling_bounds(&0.into()).unwrap().unwrap();
    assert_eq!(
        bounds.mins,
        Point3::new(q(-101, 100), q(-101, 100), q(-101, 100))
    );
    assert_eq!(
        bounds.maxs,
        Point3::new(q(101, 100), q(101, 100), q(101, 100))
    );
    let exterior = Point3::new(q(101, 100), 0.into(), 0.into());
    assert_eq!(field.evaluate_point(&exterior).unwrap(), q(201, 20201));
    let balls = [
        Metaball::new(p(-1, 0, 0), 1.into()),
        Metaball::new(p(1, 0, 0), 1.into()),
    ];
    let field = MetaballField::new(&balls, 2.into()).unwrap();
    let bounds = field.sampling_bounds(&0.into()).unwrap().unwrap();
    assert_eq!(bounds.mins.x, q(-201, 100));
    assert_eq!(bounds.maxs.x, q(201, 100));
    let value = field
        .evaluate_point(&Point3::new(bounds.maxs.x, 0.into(), 0.into()))
        .unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&value, &Real::zero(), PredicatePolicy::STRICT).value(),
        Some(Ordering::Greater)
    );
}

#[test]
fn domain_errors_and_uncertainty_are_distinct_before_shortcuts() {
    for radius in [Real::zero(), -Real::one()] {
        assert!(matches!(
            MetaballField::new(&[Metaball::new(p(0, 0, 0), radius)], 1.into()),
            Err(SdfEvaluationError::InvalidDomain { .. })
        ));
    }
    assert!(matches!(
        MetaballField::new(&[Metaball::new(p(0, 0, 0), unknown_zero())], 1.into()),
        Err(SdfEvaluationError::Undecided { .. })
    ));
    assert!(matches!(
        MetaballField::new(&[], 0.into()),
        Err(SdfEvaluationError::InvalidDomain { .. })
    ));
    let invalid = [
        Metaball::new(p(0, 0, 0), 1.into()),
        Metaball::new(p(5, 0, 0), 0.into()),
    ];
    assert!(matches!(
        MetaballField::new(&invalid, 1.into()),
        Err(SdfEvaluationError::InvalidDomain { .. })
    ));
    let empty = MetaballField::new(&[], 1.into()).unwrap();
    assert!(empty.sampling_bounds(&0.into()).unwrap().is_none());
    assert_eq!(empty.evaluate_point(&p(0, 0, 0)).unwrap(), Real::one());
    assert!(matches!(
        empty.sampling_bounds(&(-1).into()),
        Err(SdfEvaluationError::InvalidDomain { .. })
    ));
    assert!(matches!(
        empty.sampling_bounds(&unknown_zero()),
        Err(SdfEvaluationError::Undecided { .. })
    ));
}
