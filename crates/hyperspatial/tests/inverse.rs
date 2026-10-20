mod common;
use common::*;
use hyperlattice::Point3;
use hyperspatial::*;
use std::sync::Arc;

// Certified test enclosure, not the operation's equality/branch policy.
fn enclosed(actual: &Real, expected: &Real) {
    let bound = r("0.0000000000000000000000000000000000000001");
    for (left, right) in [
        (expected - &bound, actual.clone()),
        (actual.clone(), expected + bound),
    ] {
        assert_eq!(
            hyperlimit::compare_reals(&left, &right, PredicatePolicy::STRICT).value(),
            Some(std::cmp::Ordering::Less),
            "value outside independently derived enclosure",
        );
    }
}

fn independent_forward(result: &GeodeticPosition, ctx: &mut Context) -> EcefPosition {
    // Explicit geodetic ingress forces evaluation of the forward equations;
    // it has no retained inverse-source identity to reuse.
    GeodeticPosition::new(
        result.latitude().clone(),
        result.longitude().clone(),
        result.height().clone(),
        result.ellipsoid().clone(),
        result.reference().clone(),
        input(),
        ctx,
    )
    .unwrap()
    .to_ecef(ctx)
    .unwrap()
}

fn surface(a: i32, b: i32, ctx: &mut Context) -> Arc<Ellipsoid> {
    Arc::new(Ellipsoid::from_axes(Length::metres(a.into()), Length::metres(b.into()), ctx).unwrap())
}

fn cartesian(x: Real, y: Real, z: Real, ctx: &mut Context) -> EcefPosition {
    EcefPosition::new(Point3::new(x, y, z), reference(), input(), ctx).unwrap()
}

#[test]
fn closest_inverse_has_exact_axes_and_explicit_pole_meridian() {
    let mut ctx = context();
    for (a, b) in [(5, 3), (3, 5), (4, 4)] {
        let ellipsoid = surface(a, b, &mut ctx);
        let equator = cartesian((a + 2).into(), 0.into(), 0.into(), &mut ctx);
        let result = equator
            .to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx)
            .unwrap();
        equal(result.latitude().angle().as_radians(), &Real::zero());
        equal(result.height().as_metres(), &Real::from(2));
        assert!(result.longitude().is_wrapped());
        assert!(matches!(
            result.source(),
            GeodeticSource::EcefClosest {
                pole_meridian: None,
                ..
            }
        ));
        for sign in [-1, 1] {
            let pole = cartesian(0.into(), 0.into(), (sign * (b + 2)).into(), &mut ctx);
            assert!(matches!(
                pole.to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx),
                Err(Error::AmbiguousInverse(_))
            ));
            let result = pole
                .to_geodetic_closest(
                    ellipsoid.clone(),
                    PoleMeridian::Use(Longitude::unrolled(Angle::degrees(37.into()))),
                    &mut ctx,
                )
                .unwrap();
            equal(
                &result.latitude().angle().as_degrees(),
                &Real::from(sign * 90),
            );
            equal(&result.longitude().angle().as_degrees(), &Real::from(37));
            equal(result.height().as_metres(), &Real::from(2));
            assert!(matches!(
                result.source(),
                GeodeticSource::EcefClosest {
                    pole_meridian: Some(_),
                    ..
                }
            ));
            let local = LocalFrame::new(&result, LocalAxes::Enu, &mut ctx).unwrap();
            assert_eq!(
                local
                    .to_local_position(&pole, &mut ctx)
                    .unwrap()
                    .coordinates(),
                &Point3::origin()
            );
        }
    }
}

#[test]
fn centre_and_medial_axis_ambiguities_are_distinct_from_unique_cusps() {
    let mut ctx = context();
    for (a, b) in [(5, 3), (3, 5), (4, 4)] {
        let ellipsoid = surface(a, b, &mut ctx);
        let centre = cartesian(0.into(), 0.into(), 0.into(), &mut ctx);
        assert!(matches!(
            centre.to_geodetic_closest(
                ellipsoid,
                PoleMeridian::Use(Longitude::unrolled(Angle::degrees(0.into()))),
                &mut ctx
            ),
            Err(Error::AmbiguousInverse(_))
        ));
    }
    for (a, b, interior, cusp, latitude) in [
        (
            5,
            3,
            [r("3"), r("0"), r("0")],
            [r("3.2"), r("0"), r("0")],
            0,
        ),
        (
            3,
            5,
            [r("0"), r("0"), r("3")],
            [r("0"), r("0"), r("3.2")],
            90,
        ),
    ] {
        let ellipsoid = surface(a, b, &mut ctx);
        let p = cartesian(
            interior[0].clone(),
            interior[1].clone(),
            interior[2].clone(),
            &mut ctx,
        );
        let meridian = PoleMeridian::Use(Longitude::unrolled(Angle::degrees(0.into())));
        assert!(matches!(
            p.to_geodetic_closest(ellipsoid.clone(), meridian.clone(), &mut ctx),
            Err(Error::AmbiguousInverse(_))
        ));
        let p = cartesian(cusp[0].clone(), cusp[1].clone(), cusp[2].clone(), &mut ctx);
        let result = p
            .to_geodetic_closest(ellipsoid, meridian, &mut ctx)
            .unwrap();
        equal(
            &result.latitude().angle().as_degrees(),
            &Real::from(latitude),
        );
        equal(result.height().as_metres(), &r("-1.8"));
    }
}

#[test]
fn spherical_inverse_uses_the_radial_normal_at_negative_height() {
    let mut ctx = context();
    let sphere = surface(10, 10, &mut ctx);
    let point = cartesian(2.into(), 3.into(), 6.into(), &mut ctx);
    let result = point
        .to_geodetic_closest(sphere, PoleMeridian::Reject, &mut ctx)
        .unwrap();
    equal(result.height().as_metres(), &Real::from(-3));
    let latitude = Real::from(6)
        .try_atan2(Real::from(13).sqrt().unwrap())
        .unwrap();
    enclosed(result.latitude().angle().as_radians(), &latitude);
    enclosed(
        result.longitude().angle().as_radians(),
        &Real::from(3).try_atan2(2.into()).unwrap(),
    );
    let evaluated = independent_forward(&result, &mut ctx);
    for (actual, expected) in [
        &evaluated.coordinates().x,
        &evaluated.coordinates().y,
        &evaluated.coordinates().z,
    ]
    .into_iter()
    .zip([2, 3, 6])
    {
        enclosed(actual, &Real::from(expected));
    }
}

#[test]
fn analytical_normal_offsets_select_the_same_surface_foot_inside_and_outside() {
    // Ellipse feet (4,9/5) on (a,b)=(5,3), and (12/5,3) on (3,5).
    // Their outward unit normals are (4,5)/sqrt(41) and (20,9)/sqrt(481).
    let mut ctx = context();
    for (a, b, rho, z, normal_rho, normal_z, height) in [
        (
            5,
            3,
            r("4.8"),
            r("2.8"),
            4,
            5,
            (Real::from(41).sqrt().unwrap() / Real::from(5)).unwrap(),
        ),
        (
            5,
            3,
            r("3.6"),
            r("1.3"),
            4,
            5,
            (-Real::from(41).sqrt().unwrap() / Real::from(10)).unwrap(),
        ),
        (
            3,
            5,
            r("4.4"),
            r("3.9"),
            20,
            9,
            (Real::from(481).sqrt().unwrap() / Real::from(10)).unwrap(),
        ),
        (
            3,
            5,
            r("2"),
            r("2.82"),
            20,
            9,
            (-Real::from(481).sqrt().unwrap() / Real::from(50)).unwrap(),
        ),
    ] {
        let ellipsoid = surface(a, b, &mut ctx);
        for sign in [-1, 1] {
            let point = cartesian(0.into(), -&rho, &z * Real::from(sign), &mut ctx);
            let result = point
                .to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx)
                .unwrap();
            let expected_latitude = Real::from(sign * normal_z)
                .try_atan2(normal_rho.into())
                .unwrap();
            enclosed(result.latitude().angle().as_radians(), &expected_latitude);
            equal(&result.longitude().angle().as_degrees(), &Real::from(-90));
            enclosed(result.height().as_metres(), &height);
            let evaluated = independent_forward(&result, &mut ctx);
            for (actual, expected) in [
                &evaluated.coordinates().x,
                &evaluated.coordinates().y,
                &evaluated.coordinates().z,
            ]
            .into_iter()
            .zip([
                &point.coordinates().x,
                &point.coordinates().y,
                &point.coordinates().z,
            ]) {
                enclosed(actual, expected);
            }
        }
    }
}

#[test]
fn deep_unique_interior_uses_the_real_trigonometric_cubic_branch() {
    let mut ctx = context();
    let sqrt10 = Real::from(10).sqrt().unwrap();
    let half = r("0.5");
    let low = (Real::from(5) / &sqrt10).unwrap() - &half;
    let high = (Real::from(9) / &sqrt10).unwrap() - r("2.5");
    let height = (-Real::from(26).sqrt().unwrap() / Real::from(2)).unwrap();
    for (a, b, rho, z, nr, nz) in [
        (5, 3, low.clone(), high.clone(), 1, 5),
        (3, 5, high, low, 5, 1),
    ] {
        let ellipsoid = surface(a, b, &mut ctx);
        let point = cartesian(rho, 0.into(), z, &mut ctx);
        let result = point
            .to_geodetic_closest(ellipsoid, PoleMeridian::Reject, &mut ctx)
            .unwrap();
        enclosed(
            result.latitude().angle().as_radians(),
            &Real::from(nz).try_atan2(nr.into()).unwrap(),
        );
        enclosed(result.height().as_metres(), &height);
        let mut receipts = vec![result.receipt()];
        let mut negative_discriminant = false;
        while let Some(receipt) = receipts.pop() {
            negative_discriminant |= receipt.decisions().iter().any(|decision| {
                decision.claim() == "inverse cubic discriminant"
                    && decision.outcome().value() == Some(std::cmp::Ordering::Less)
            });
            receipts.extend(receipt.parents());
        }
        assert!(negative_discriminant);
    }
}

#[test]
fn tiny_offsets_across_the_medial_boundary_are_never_merged() {
    let mut ctx = context();
    let ellipsoid = surface(5, 3, &mut ctx);
    let offset = r("0.0000000000000000000000000000000000000001");
    let inside = cartesian(r("3.2") - &offset, 0.into(), 0.into(), &mut ctx);
    assert!(matches!(
        inside.to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx),
        Err(Error::AmbiguousInverse(_))
    ));
    let outside = cartesian(r("3.2") + &offset, 0.into(), 0.into(), &mut ctx);
    let result = outside
        .to_geodetic_closest(ellipsoid, PoleMeridian::Reject, &mut ctx)
        .unwrap();
    equal(result.height().as_metres(), &(r("-1.8") + offset));
}

#[test]
fn far_normal_offset_has_no_float_range_or_spherical_shortcut() {
    let mut ctx = context();
    let ellipsoid = surface(5, 3, &mut ctx);
    let height = r(
        "10000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000",
    );
    // Even an exact axis offset beyond binary64's range must retain a, not
    // replace Earth by a point. General far-distance offsets are checked below.
    let point = cartesian(Real::from(5) + &height, 0.into(), 0.into(), &mut ctx);
    let result = point
        .to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx)
        .unwrap();
    equal(result.height().as_metres(), &height);
    let h = r("1000000000000000000000000000000000000000000000000000000000000");
    let normal = Real::from(41).sqrt().unwrap();
    let x = Real::from(4) + (&h * Real::from(4) / &normal).unwrap();
    let z = r("1.8") + (&h * Real::from(5) / normal).unwrap();
    let point = cartesian(x, 0.into(), z, &mut ctx);
    let result = point
        .to_geodetic_closest(ellipsoid, PoleMeridian::Reject, &mut ctx)
        .unwrap();
    enclosed(
        result.latitude().angle().as_radians(),
        &Real::from(5).try_atan2(4.into()).unwrap(),
    );
    enclosed(result.height().as_metres(), &h);
}

#[test]
fn inverse_budget_cancellation_recovery_and_retained_source_compose() {
    use std::sync::atomic::Ordering;
    let mut ctx = context();
    let ellipsoid = surface(5, 3, &mut ctx);
    let point = cartesian(7.into(), 0.into(), 0.into(), &mut ctx);
    for limits in [
        Limits {
            max_steps: 1,
            ..Limits::default()
        },
        Limits {
            max_receipt_depth: 1,
            ..Limits::default()
        },
    ] {
        let mut limited = Context::new(PredicatePolicy::STRICT, limits);
        assert!(matches!(
            point.to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut limited),
            Err(Error::ResourceExhausted(_))
        ));
    }
    let flag = ctx.cancellation_signal();
    flag.store(true, Ordering::Relaxed);
    assert!(matches!(
        point.to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx),
        Err(Error::Cancelled)
    ));
    flag.store(false, Ordering::Relaxed);
    let mut current = point.clone();
    for _ in 0..8 {
        let geodetic = current
            .to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx)
            .unwrap();
        let GeodeticSource::EcefClosest { position, .. } = geodetic.source() else {
            panic!("lost inverse identity")
        };
        assert_eq!(position.coordinates(), current.coordinates());
        current = geodetic.to_ecef(&mut ctx).unwrap();
        assert_eq!(current.coordinates(), point.coordinates());
    }
    assert!(current.receipt().depth() > point.receipt().depth());
}

#[test]
fn approximate_inverse_branch_evidence_survives_latitude_and_forward_reuse() {
    let mut strict = context();
    let a = Real::one() + unresolved_zero();
    let ellipsoid = Arc::new(
        Ellipsoid::from_axes(Length::metres(a), Length::metres(Real::one()), &mut strict).unwrap(),
    );
    let point = cartesian(3.into(), 4.into(), 5.into(), &mut strict);
    assert!(matches!(
        point.to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut strict),
        Err(Error::Unknown { .. })
    ));
    let mut approximate = Context::new(PredicatePolicy::APPROXIMATE_512, Limits::default());
    let result = point
        .to_geodetic_closest(ellipsoid, PoleMeridian::Reject, &mut approximate)
        .unwrap();
    assert_eq!(result.receipt().certainty(), Certainty::Approximate);
    assert_eq!(
        result.latitude().receipt().certainty(),
        Certainty::Approximate
    );
    let forwarded = result.to_ecef(&mut strict).unwrap();
    assert_eq!(forwarded.receipt().certainty(), Certainty::Approximate);
    // Returning via the retained-source identity would have these exact fields.
    // The qualified branch instead evaluates its forward expressions.
    assert_ne!(forwarded.coordinates(), point.coordinates());
}

#[test]
fn inverse_batches_publish_only_complete_results() {
    let mut ctx = context();
    let ellipsoid = surface(5, 3, &mut ctx);
    assert!(
        ecef_to_geodetic_batch(&[], ellipsoid.clone(), PoleMeridian::Reject, &mut ctx)
            .unwrap()
            .is_empty()
    );
    let valid = cartesian(7.into(), 0.into(), 0.into(), &mut ctx);
    let centre = cartesian(0.into(), 0.into(), 0.into(), &mut ctx);
    assert!(matches!(
        ecef_to_geodetic_batch(
            &[valid.clone(), centre],
            ellipsoid.clone(),
            PoleMeridian::Reject,
            &mut ctx
        ),
        Err(Error::AmbiguousInverse(_))
    ));
    let result = ecef_to_geodetic_batch(
        std::slice::from_ref(&valid),
        ellipsoid.clone(),
        PoleMeridian::Reject,
        &mut ctx,
    )
    .unwrap();
    equal(result[0].height().as_metres(), &Real::from(2));
    for limits in [
        Limits {
            max_batch_len: 0,
            ..Limits::default()
        },
        Limits {
            max_steps: 5,
            ..Limits::default()
        },
    ] {
        let mut limited = Context::new(PredicatePolicy::STRICT, limits);
        assert!(matches!(
            ecef_to_geodetic_batch(
                std::slice::from_ref(&valid),
                ellipsoid.clone(),
                PoleMeridian::Reject,
                &mut limited
            ),
            Err(Error::ResourceExhausted(_))
        ));
    }
}
