mod common;
use common::*;
use hyperlattice::Point3;
use hyperspatial::*;
use std::{cmp::Ordering, sync::Arc};

fn close(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(
            &(actual - expected).abs(),
            &r("1e-30"),
            PredicatePolicy::STRICT
        )
        .value(),
        Some(Ordering::Less)
    );
}

fn projection(a: Real, b: Real, policy: LongitudePolicy, ctx: &mut Context) -> EllipsoidalMercator {
    let ellipsoid =
        Arc::new(Ellipsoid::from_axes(Length::metres(a), Length::metres(b), ctx).unwrap());
    EllipsoidalMercator::new(
        ellipsoid,
        reference(),
        MercatorParameters::greenwich(policy),
        ctx,
    )
    .unwrap()
}

#[test]
fn independent_oblate_spherical_and_prolate_inverses_compose_with_ecef_and_local() {
    let mut ctx = context();
    let eccentricity = r("0.8");
    let logarithm = Real::from(2).ln().unwrap();
    let cases = [
        (
            Real::from(5),
            Real::from(3),
            &logarithm - &eccentricity * r("0.48").atanh().unwrap(),
        ),
        (Real::from(5), Real::from(5), logarithm.clone()),
        (
            Real::from(5),
            Real::from(41).sqrt().unwrap(),
            &logarithm + &eccentricity * r("0.48").atan().unwrap(),
        ),
    ];
    for (a, b, psi) in cases {
        let mut parameters = MercatorParameters::greenwich(LongitudePolicy::Unrolled);
        parameters.scale = 2.into();
        parameters.central_meridian = Longitude::unrolled(Angle::degrees(15.into()));
        parameters.false_easting = Length::metres(100.into());
        parameters.false_northing = Length::metres((-50).into());
        let ellipsoid = Arc::new(
            Ellipsoid::from_axes(
                Length::metres(a.clone()),
                Length::metres(b.clone()),
                &mut ctx,
            )
            .unwrap(),
        );
        let projection =
            EllipsoidalMercator::new(ellipsoid, reference(), parameters, &mut ctx).unwrap();
        assert_eq!(projection.parameters().scale, Real::from(2));
        assert_eq!(projection.reference().name(), reference().name());
        for sign in [-1, 1] {
            let northing = Real::from(-50) + Real::from(10 * sign) * &psi;
            let projected = projection
                .position(
                    Point3::new(
                        Real::from(100) + (Real::from(5) * Real::pi() / Real::from(3)).unwrap(),
                        northing.clone(),
                        (-1).into(),
                    ),
                    input(),
                    &mut ctx,
                )
                .unwrap();
            let inverse = projection.unproject(&projected, &mut ctx).unwrap();
            close(
                inverse.latitude().angle().as_radians(),
                &(Real::from(sign) * r("0.75").atan().unwrap()),
            );
            equal(&inverse.longitude().angle().as_degrees(), &Real::from(45));
            equal(inverse.height().as_metres(), &Real::from(-1));
            assert!(matches!(
                inverse.source(),
                GeodeticSource::EllipsoidalMercator(_)
            ));
            let ecef = inverse.to_ecef(&mut ctx).unwrap();
            let divisor = ((&a * &a * r("0.64")) + (&b * &b * r("0.36")))
                .sqrt()
                .unwrap();
            let radius = ((&a * &a / &divisor).unwrap() - Real::one()) * r("0.8");
            close(
                &ecef.coordinates().x,
                &(&radius * Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap(),
            );
            close(&ecef.coordinates().y, &ecef.coordinates().x);
            close(
                &ecef.coordinates().z,
                &(((&b * &b / divisor).unwrap() - Real::one()) * r("0.6") * Real::from(sign)),
            );
            let frame = LocalFrame::new(&inverse, LocalAxes::Enu, &mut ctx).unwrap();
            let local = frame.to_local_position(&ecef, &mut ctx).unwrap();
            for coordinate in [
                &local.coordinates().x,
                &local.coordinates().y,
                &local.coordinates().z,
            ] {
                equal(coordinate, &Real::zero());
            }
            // Rebuild the latitude from its angle to exercise forward evaluation,
            // independently of both the source replay and retained inverse relation.
            let fresh = GeodeticPosition::new(
                Latitude::new(inverse.latitude().angle().clone(), &mut ctx).unwrap(),
                inverse.longitude().clone(),
                inverse.height().clone(),
                inverse.ellipsoid().clone(),
                inverse.reference().clone(),
                input(),
                &mut ctx,
            )
            .unwrap();
            let forward = projection.project(&fresh, &mut ctx).unwrap();
            close(&forward.coordinates().y, &northing);
            let returned = projection.unproject(&forward, &mut ctx).unwrap();
            assert_ne!(returned.receipt().certainty(), Certainty::Approximate);
        }
    }
}

#[test]
fn seam_policies_and_exact_offsets_keep_their_distinct_meanings() {
    let mut ctx = context();
    let tiny = r("0.0000000000000000000000000000000000000001");
    for policy in [
        LongitudePolicy::Unrolled,
        LongitudePolicy::RejectOutsidePrincipal,
        LongitudePolicy::WrapSigned,
    ] {
        let projection = ellipsoid_projection(policy, &mut ctx);
        for longitude in [-540_i32, -180, 0, 180, 540, 730] {
            let source = position(
                0.into(),
                longitude.into(),
                1.into(),
                projection.ellipsoid().clone(),
                &mut ctx,
            );
            let result = projection.project(&source, &mut ctx);
            if policy == LongitudePolicy::RejectOutsidePrincipal
                && !(-180..180).contains(&longitude)
            {
                assert!(matches!(result, Err(Error::InvalidInput(_))));
                continue;
            }
            let point = result.unwrap();
            let expected = if policy == LongitudePolicy::WrapSigned {
                (longitude + 180).rem_euclid(360) - 180
            } else {
                longitude
            };
            equal(
                &point.coordinates().x,
                Angle::degrees(expected.into()).as_radians(),
            );
            equal(
                &point
                    .source_geodetic()
                    .unwrap()
                    .longitude()
                    .angle()
                    .as_degrees(),
                &Real::from(longitude),
            );
            let inverse = projection.unproject(&point, &mut ctx).unwrap();
            equal(
                &inverse.longitude().angle().as_degrees(),
                &Real::from(expected),
            );
        }
        for delta in [Real::pi() - &tiny, Real::pi(), Real::pi() + &tiny] {
            let raw = projection.position(
                Point3::new(delta.clone(), 0.into(), 0.into()),
                input(),
                &mut ctx,
            );
            let outside = hyperlimit::compare_reals(&delta, &Real::pi(), PredicatePolicy::STRICT)
                .value()
                .unwrap()
                != Ordering::Less;
            if outside && policy != LongitudePolicy::Unrolled {
                assert!(matches!(raw, Err(Error::InvalidInput(_))));
            } else {
                equal(&raw.unwrap().coordinates().x, &delta);
            }
        }
    }
    let mut parameters = MercatorParameters::greenwich(LongitudePolicy::WrapSigned);
    parameters.central_meridian = Longitude::unrolled(Angle::degrees(30.into()));
    let surface = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut ctx).unwrap());
    let shifted = EllipsoidalMercator::new(surface, reference(), parameters, &mut ctx).unwrap();
    let source = position(
        0.into(),
        200.into(),
        0.into(),
        shifted.ellipsoid().clone(),
        &mut ctx,
    );
    let point = shifted.project(&source, &mut ctx).unwrap();
    equal(
        &shifted
            .unproject(&point, &mut ctx)
            .unwrap()
            .longitude()
            .angle()
            .as_degrees(),
        &Real::from(200),
    );
}

#[test]
fn poles_scale_and_reference_mismatches_fail_without_fabricating_coordinates() {
    let mut ctx = context();
    let projection = ellipsoid_projection(LongitudePolicy::Unrolled, &mut ctx);
    for latitude in [-90, 90] {
        let pole = position(
            latitude.into(),
            0.into(),
            0.into(),
            projection.ellipsoid().clone(),
            &mut ctx,
        );
        assert!(matches!(
            projection.project(&pole, &mut ctx),
            Err(Error::ProjectionSingularity(_))
        ));
    }
    for scale in [0, -1] {
        let mut parameters = MercatorParameters::greenwich(LongitudePolicy::Unrolled);
        parameters.scale = scale.into();
        assert!(matches!(
            EllipsoidalMercator::new(
                projection.ellipsoid().clone(),
                reference(),
                parameters,
                &mut ctx
            ),
            Err(Error::InvalidInput(_))
        ));
    }
    let source = position(
        30.into(),
        1.into(),
        0.into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    let point = projection.project(&source, &mut ctx).unwrap();
    assert!(projection.same_definition(&projection.clone()));
    let other = ellipsoid_projection(LongitudePolicy::Unrolled, &mut ctx);
    assert!(!projection.same_definition(&other));
    assert!(matches!(
        other.unproject(&point, &mut ctx),
        Err(Error::ProjectionMismatch)
    ));
    for reference in [
        ReferenceFrame::fixed("other").unwrap(),
        ReferenceFrame::at_julian_year(reference().name(), 2000.into()).unwrap(),
    ] {
        let mismatched = GeodeticPosition::new(
            source.latitude().clone(),
            source.longitude().clone(),
            source.height().clone(),
            source.ellipsoid().clone(),
            reference,
            input(),
            &mut ctx,
        )
        .unwrap();
        assert!(matches!(
            projection.project(&mismatched, &mut ctx),
            Err(Error::ReferenceMismatch)
        ));
    }
    for (a, b) in [(2, 1), (1, 2)] {
        let wrong = Arc::new(
            Ellipsoid::from_axes(Length::metres(a.into()), Length::metres(b.into()), &mut ctx)
                .unwrap(),
        );
        let mismatched = position(0.into(), 0.into(), 0.into(), wrong, &mut ctx);
        assert!(matches!(
            projection.project(&mismatched, &mut ctx),
            Err(Error::EllipsoidMismatch)
        ));
    }
}

#[test]
fn inherited_approximate_epoch_and_input_qualification_survive_inverse_and_local_use() {
    let mut strict = context();
    let s = Real::e().sin();
    let c = Real::e().cos();
    let zero = &s * &s + &c * &c - Real::one();
    let surface = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut strict).unwrap());
    let projection = EllipsoidalMercator::new(
        surface.clone(),
        ReferenceFrame::at_julian_year("epoch", 2000.into()).unwrap(),
        MercatorParameters::greenwich(LongitudePolicy::Unrolled),
        &mut strict,
    )
    .unwrap();
    let source = GeodeticPosition::new(
        Latitude::new(Angle::degrees(30.into()), &mut strict).unwrap(),
        Longitude::unrolled(Angle::degrees(45.into())),
        Length::metres(2.into()),
        surface,
        ReferenceFrame::at_julian_year("epoch", Real::from(2000) + zero).unwrap(),
        InputProvenance::new("qualified station", InputAccuracy::Unspecified),
        &mut strict,
    )
    .unwrap();
    assert!(matches!(
        projection.project(&source, &mut strict),
        Err(Error::Unknown { .. })
    ));
    let mut approximate = Context::new(PredicatePolicy::APPROXIMATE_512, Limits::default());
    let point = projection.project(&source, &mut approximate).unwrap();
    assert_eq!(point.receipt().certainty(), Certainty::Approximate);
    let inverse = projection.unproject(&point, &mut strict).unwrap();
    close(
        inverse.latitude().angle().as_radians(),
        source.latitude().angle().as_radians(),
    );
    assert_eq!(
        inverse.latitude().receipt().certainty(),
        Certainty::Approximate
    );
    let ecef = inverse.to_ecef(&mut strict).unwrap();
    let frame = LocalFrame::new(&inverse, LocalAxes::Enu, &mut strict).unwrap();
    let local = frame.to_local_position(&ecef, &mut strict).unwrap();
    let reprojection = projection.project(&inverse, &mut strict).unwrap();
    for receipt in [local.receipt(), reprojection.receipt()] {
        assert_eq!(receipt.certainty(), Certainty::Approximate);
        let mut pending = vec![receipt];
        let mut found = false;
        while let Some(node) = pending.pop() {
            found |= node.input().is_some_and(|p| {
                p.source() == "qualified station" && p.accuracy() == &InputAccuracy::Unspecified
            });
            pending.extend(node.parents());
        }
        assert!(found);
    }
}

#[test]
fn batches_budgets_cancellation_and_repeated_composition_preserve_the_contract() {
    let mut ctx = context();
    let projection = ellipsoid_projection(LongitudePolicy::WrapSigned, &mut ctx);
    let source = position(
        45.into(),
        190.into(),
        (-2).into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    assert!(projection.project_batch(&[], &mut ctx).unwrap().is_empty());
    assert!(
        projection
            .unproject_batch(&[], &mut ctx)
            .unwrap()
            .is_empty()
    );
    let pole = position(
        90.into(),
        0.into(),
        0.into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    assert!(matches!(
        projection.project_batch(&[source.clone(), pole], &mut ctx),
        Err(Error::ProjectionSingularity(_))
    ));
    let projected = projection
        .project_batch(&[source.clone(), source.clone()], &mut ctx)
        .unwrap();
    assert_eq!(
        projection
            .unproject_batch(&projected, &mut ctx)
            .unwrap()
            .len(),
        2
    );
    let other = ellipsoid_projection(LongitudePolicy::WrapSigned, &mut ctx);
    let foreign = other.project(&source, &mut ctx).unwrap();
    assert!(matches!(
        projection.unproject_batch(&[projected[0].clone(), foreign], &mut ctx),
        Err(Error::ProjectionMismatch)
    ));
    for limits in [
        Limits {
            max_steps: 1,
            ..Limits::default()
        },
        Limits {
            max_batch_len: 0,
            ..Limits::default()
        },
        Limits {
            max_receipt_depth: 1,
            ..Limits::default()
        },
    ] {
        let mut limited = Context::new(PredicatePolicy::STRICT, limits);
        assert!(matches!(
            projection.project_batch(std::slice::from_ref(&source), &mut limited),
            Err(Error::ResourceExhausted(_))
        ));
        let mut limited = Context::new(PredicatePolicy::STRICT, limits);
        assert!(matches!(
            projection.unproject_batch(&projected, &mut limited),
            Err(Error::ResourceExhausted(_))
        ));
    }
    let mut measured = context();
    projection.project(&source, &mut measured).unwrap();
    let mut limited = Context::new(
        PredicatePolicy::STRICT,
        Limits {
            max_steps: measured.steps_used() + 2,
            ..Limits::default()
        },
    );
    assert!(matches!(
        projection.project_batch(&[source.clone(), source], &mut limited),
        Err(Error::ResourceExhausted("work checkpoints"))
    ));
    let signal = ctx.cancellation_signal();
    signal.store(true, std::sync::atomic::Ordering::Relaxed);
    assert!(matches!(
        projection.unproject(&projected[0], &mut ctx),
        Err(Error::Cancelled)
    ));
    signal.store(false, std::sync::atomic::Ordering::Relaxed);
    let mut current = projected[0].clone();
    for _ in 0..12 {
        let geographic = projection.unproject(&current, &mut ctx).unwrap();
        current = projection.project(&geographic, &mut ctx).unwrap();
        assert_eq!(current.coordinates(), projected[0].coordinates());
        assert_ne!(current.receipt().certainty(), Certainty::Approximate);
    }
    assert!(current.receipt().depth() > projected[0].receipt().depth());
}

fn ellipsoid_projection(policy: LongitudePolicy, ctx: &mut Context) -> EllipsoidalMercator {
    projection(Real::one(), r("0.6"), policy, ctx)
}

#[test]
fn inverse_open_range_and_defining_relation_survive_huge_northings_and_cloned_latitudes() {
    let mut ctx = context();
    for b in [r("0.6"), Real::one(), r("1.4")] {
        let projection = projection(Real::one(), b.clone(), LongitudePolicy::Unrolled, &mut ctx);
        for psi in [r("1e320"), r("-1e320"), r("1e-80"), r("-1e-80")] {
            let source = projection
                .position(
                    Point3::new(r("0.25"), psi.clone(), (-5).into()),
                    InputProvenance::new(
                        "extreme northing",
                        InputAccuracy::Qualified("mathematical fixture".into()),
                    ),
                    &mut ctx,
                )
                .unwrap();
            let inverse = projection.unproject(&source, &mut ctx).unwrap();
            assert_eq!(
                inverse.latitude().receipt().operation(),
                "inverse isometric latitude: open domain"
            );
            // Evaluate the scalar independently of reprojecting its retained relation.
            assert!(
                inverse
                    .latitude()
                    .angle()
                    .as_radians()
                    .certified_rational_interval(-160)
                    .is_some()
            );
            let replay = projection.project(&inverse, &mut ctx).unwrap();
            equal(&replay.coordinates().y, &psi);
            // A public clone must retain the relation even after rebuilding the
            // geodetic input and using a distinct prepared projection with new offsets.
            let fresh = GeodeticPosition::new(
                inverse.latitude().clone(),
                inverse.longitude().clone(),
                inverse.height().clone(),
                inverse.ellipsoid().clone(),
                inverse.reference().clone(),
                input(),
                &mut ctx,
            )
            .unwrap();
            let mut parameters = MercatorParameters::greenwich(LongitudePolicy::Unrolled);
            parameters.scale = 3.into();
            parameters.false_northing = Length::metres(7.into());
            let other = EllipsoidalMercator::new(
                inverse.ellipsoid().clone(),
                reference(),
                parameters,
                &mut ctx,
            )
            .unwrap();
            let transformed = other.project(&fresh, &mut ctx).unwrap();
            equal(
                &transformed.coordinates().y,
                &(Real::from(7) + Real::from(3) * &psi),
            );
            assert_ne!(transformed.receipt().certainty(), Certainty::Approximate);
            let mut pending = vec![transformed.receipt()];
            let mut found = false;
            while let Some(node) = pending.pop() {
                found |= node
                    .input()
                    .is_some_and(|p| p.source() == "extreme northing");
                pending.extend(node.parents());
            }
            assert!(found);
            if b == Real::one() {
                let spherical = SphericalMercator::new(
                    inverse.ellipsoid().clone(),
                    reference(),
                    MercatorParameters::greenwich(LongitudePolicy::Unrolled),
                    &mut ctx,
                )
                .unwrap();
                equal(
                    &spherical.project(&fresh, &mut ctx).unwrap().coordinates().y,
                    &psi,
                );
            }
        }
    }
}

#[test]
fn retained_relation_is_only_reused_for_the_same_axis_ratio() {
    let mut ctx = context();
    let source_projection = projection(Real::one(), r("0.6"), LongitudePolicy::Unrolled, &mut ctx);
    let psi = Real::from(2).ln().unwrap() - r("0.8") * r("0.48").atanh().unwrap();
    let projected = source_projection
        .position(Point3::new(0.into(), psi, 0.into()), input(), &mut ctx)
        .unwrap();
    let inverse = source_projection.unproject(&projected, &mut ctx).unwrap();
    let spherical = SphericalMercator::new(
        inverse.ellipsoid().clone(),
        reference(),
        MercatorParameters::greenwich(LongitudePolicy::Unrolled),
        &mut ctx,
    )
    .unwrap();
    let spherical_output = spherical.project(&inverse, &mut ctx).unwrap();
    close(
        &spherical_output.coordinates().y,
        &Real::from(2).ln().unwrap(),
    );
    let prolate = projection(
        Real::one(),
        (Real::from(41).sqrt().unwrap() / Real::from(5)).unwrap(),
        LongitudePolicy::Unrolled,
        &mut ctx,
    );
    // Explicit new coordinate ingress chooses a different surface; it retains
    // latitude evidence but must not reuse the old surface's isometric latitude.
    let retargeted = GeodeticPosition::new(
        inverse.latitude().clone(),
        inverse.longitude().clone(),
        inverse.height().clone(),
        prolate.ellipsoid().clone(),
        reference(),
        input(),
        &mut ctx,
    )
    .unwrap();
    close(
        &prolate
            .project(&retargeted, &mut ctx)
            .unwrap()
            .coordinates()
            .y,
        &(Real::from(2).ln().unwrap() + r("0.8") * r("0.48").atan().unwrap()),
    );
}

#[test]
fn unprovable_shape_stays_unknown_and_approximate_preparation_stays_qualified() {
    let mut ctx = context();
    let s = Real::e().sin();
    let c = Real::e().cos();
    let zero = &s * &s + &c * &c - Real::one();
    let ellipsoid = Arc::new(
        Ellipsoid::from_axes(
            Length::metres(Real::one()),
            Length::metres(Real::one() + zero),
            &mut ctx,
        )
        .unwrap(),
    );
    assert!(matches!(
        EllipsoidalMercator::new(
            ellipsoid.clone(),
            reference(),
            MercatorParameters::greenwich(LongitudePolicy::Unrolled),
            &mut ctx
        ),
        Err(Error::Unknown { .. })
    ));
    let mut approximate = Context::new(PredicatePolicy::APPROXIMATE_512, Limits::default());
    let projection = EllipsoidalMercator::new(
        ellipsoid,
        reference(),
        MercatorParameters::greenwich(LongitudePolicy::Unrolled),
        &mut approximate,
    )
    .unwrap();
    assert_eq!(projection.receipt().certainty(), Certainty::Approximate);
    let input = projection
        .position(
            Point3::new(0.into(), r("1e320"), 0.into()),
            input(),
            &mut ctx,
        )
        .unwrap();
    let inverse = projection.unproject(&input, &mut ctx).unwrap();
    let output = projection.project(&inverse, &mut ctx).unwrap();
    assert_eq!(output.receipt().certainty(), Certainty::Approximate);
    equal(&output.coordinates().y, &input.coordinates().y);
}
