mod common;
use common::*;
use hyperlattice::{Point3, Vector3};
use hyperspatial::*;
use proptest::prelude::*;
use std::{
    collections::BTreeSet,
    sync::{Arc, atomic::Ordering},
};

// An exact zero that no retained scalar relation proves. The scalar layer
// now proves sin(a)^2 + cos(a)^2 = 1, so use the angle-addition identity.
fn terminal_zero() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

#[test]
fn approximate_domain_evidence_is_sticky_through_strict_transformations() {
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let angle = Angle::radians(half_pi + terminal_zero());
    let mut strict = context();
    assert!(matches!(
        Latitude::new(angle.clone(), &mut strict),
        Err(Error::Unknown { .. })
    ));
    let mut approximate = Context::new(PredicatePolicy::APPROXIMATE_512, Limits::default());
    let latitude = Latitude::new(angle, &mut approximate).unwrap();
    assert_eq!(latitude.receipt().certainty(), Certainty::Approximate);
    let ellipsoid = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut strict).unwrap());
    let source = GeodeticPosition::new(
        latitude,
        Longitude::unrolled(Angle::degrees(0.into())),
        Length::metres(0.into()),
        ellipsoid,
        reference(),
        InputProvenance::new("uncertain observation", InputAccuracy::Unspecified),
        &mut strict,
    )
    .unwrap();
    let ecef = source.to_ecef(&mut strict).unwrap();
    assert_eq!(ecef.receipt().certainty(), Certainty::Approximate);
    let frame = LocalFrame::new(&source, LocalAxes::Enu, &mut strict).unwrap();
    let local = frame.to_local_position(&ecef, &mut strict).unwrap();
    let returned = frame.to_ecef_position(&local, &mut strict).unwrap();
    assert_eq!(returned.receipt().certainty(), Certainty::Approximate);
    let mut ancestry = vec![returned.receipt()];
    let mut found = false;
    while let Some(receipt) = ancestry.pop() {
        if let Some(input) = receipt.input() {
            found |= input.source() == "uncertain observation"
                && input.accuracy() == &InputAccuracy::Unspecified;
        }
        ancestry.extend(receipt.parents());
    }
    assert!(found);
}

#[test]
fn cancellation_and_small_budgets_fail_without_poisoning_results() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let source = position(0.into(), 0.into(), 0.into(), ellipsoid, &mut ctx);
    let retained = source.to_ecef(&mut ctx).unwrap();
    let flag = ctx.cancellation_signal();
    flag.store(true, Ordering::Relaxed);
    assert!(matches!(source.to_ecef(&mut ctx), Err(Error::Cancelled)));
    equal(&retained.coordinates().x, &r("6378137"));
    flag.store(false, Ordering::Relaxed);
    assert_eq!(
        source.to_ecef(&mut ctx).unwrap().coordinates(),
        retained.coordinates()
    );
    let mut limited = Context::new(
        PredicatePolicy::STRICT,
        Limits {
            max_steps: 2,
            ..Limits::default()
        },
    );
    assert_eq!(limited.policy(), PredicatePolicy::STRICT);
    assert_eq!(limited.steps_used(), 0);
    assert!(matches!(
        source.to_ecef(&mut limited),
        Err(Error::ResourceExhausted(_))
    ));
    assert_eq!(limited.steps_used(), limited.limits().max_steps);
    assert_eq!(
        source.to_ecef(&mut context()).unwrap().coordinates(),
        retained.coordinates()
    );
}

#[test]
fn public_errors_retain_scalar_causes_and_domain_diagnostics() {
    use std::error::Error as _;
    let scalar = Real::from(-1).sqrt().unwrap_err();
    let wrapped = Error::from(scalar);
    assert!(wrapped.source().is_some());
    assert!(wrapped.to_string().contains("Scalar"));
    let invalid = Latitude::new(Angle::degrees(91.into()), &mut context()).unwrap_err();
    assert!(invalid.source().is_none());
    assert!(invalid.to_string().contains("latitude"));
}

#[test]
fn atomic_batches_cover_empty_length_limit_and_mid_batch_exhaustion() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let source = position(0.into(), 0.into(), 0.into(), ellipsoid, &mut ctx);
    assert!(geodetic_to_ecef_batch(&[], &mut ctx).unwrap().is_empty());
    let sources = [source.clone(), source];
    let output = geodetic_to_ecef_batch(&sources, &mut ctx).unwrap();
    assert_eq!(output.len(), 2);
    let mut length_limited = Context::new(
        PredicatePolicy::STRICT,
        Limits {
            max_batch_len: 1,
            ..Limits::default()
        },
    );
    assert!(matches!(
        geodetic_to_ecef_batch(&sources, &mut length_limited),
        Err(Error::ResourceExhausted("batch length"))
    ));
    let mut work_limited = Context::new(
        PredicatePolicy::STRICT,
        Limits {
            max_steps: 5,
            ..Limits::default()
        },
    );
    assert!(matches!(
        geodetic_to_ecef_batch(&sources, &mut work_limited),
        Err(Error::ResourceExhausted("work checkpoints"))
    ));
    assert_eq!(
        geodetic_to_ecef_batch(&sources, &mut context())
            .unwrap()
            .len(),
        2
    );
}

#[test]
fn repeated_composition_preserves_values_and_bounds_evidence_depth() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::sphere(Length::metres(10.into()), &mut ctx).unwrap());
    let source = position(0.into(), 0.into(), 0.into(), ellipsoid, &mut ctx);
    let frame = LocalFrame::new(&source, LocalAxes::Enu, &mut ctx).unwrap();
    let original = EcefPosition::new(
        Point3::new(11.into(), 2.into(), 3.into()),
        reference(),
        input(),
        &mut ctx,
    )
    .unwrap();
    let mut current = original.clone();
    let mut limited = Context::new(
        PredicatePolicy::STRICT,
        Limits {
            max_receipt_depth: 20,
            ..Limits::default()
        },
    );
    let mut exhausted = false;
    for _ in 0..20 {
        let result = frame
            .to_local_position(&current, &mut limited)
            .and_then(|local| frame.to_ecef_position(&local, &mut limited));
        match result {
            Ok(next) => {
                assert_eq!(next.coordinates(), original.coordinates());
                current = next;
            }
            Err(Error::ResourceExhausted("evidence depth")) => {
                exhausted = true;
                break;
            }
            other => panic!("unexpected composition outcome: {other:?}"),
        }
    }
    assert!(exhausted);
    assert!(frame.to_local_position(&original, &mut context()).is_ok());
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]
    #[test]
    fn generated_ellipsoid_surface_and_basis_invariants(a in 1i32..100, b in 1i32..100, latitude in -3i32..4, longitude in -4i32..5) {
        let mut ctx = context();
        let ellipsoid = Arc::new(Ellipsoid::from_axes(Length::metres(a.into()), Length::metres(b.into()), &mut ctx).unwrap());
        let source = position((latitude * 30).into(), (longitude * 45).into(), 0.into(), ellipsoid, &mut ctx);
        let point = source.to_ecef(&mut ctx).unwrap();
        let p = point.coordinates();
        let surface = ((&p.x * &p.x + &p.y * &p.y) / Real::from(a*a)).unwrap() + ((&p.z * &p.z) / Real::from(b*b)).unwrap();
        equal(&surface, &Real::one());
        let frame = LocalFrame::new(&source, LocalAxes::Enu, &mut ctx).unwrap();
        let basis = frame.rotation();
        for i in 0..3 {
            for j in 0..3 {
                let dot = Vector3::new(basis[i].clone()).dot(&Vector3::new(basis[j].clone()));
                equal(&dot, &Real::from(i32::from(i == j)));
            }
        }
        let local = frame.to_local_position(&point, &mut ctx).unwrap();
        equal(&local.coordinates().x, &Real::zero());
        equal(&local.coordinates().y, &Real::zero());
        equal(&local.coordinates().z, &Real::zero());
        let mut operations = BTreeSet::new();
        let mut receipts = vec![local.receipt()];
        while let Some(receipt) = receipts.pop() { operations.insert(receipt.operation()); receipts.extend(receipt.parents()); }
        prop_assert!(operations.contains("ellipsoid definition"));
        prop_assert!(operations.contains("latitude domain"));
    }
}
