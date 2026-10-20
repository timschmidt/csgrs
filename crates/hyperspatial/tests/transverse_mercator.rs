mod common;
use common::*;
use hyperlattice::Point3;
use hyperspatial::*;
use std::cmp::Ordering;

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

#[test]
fn logarithmic_easting_and_quarter_turn_northing_have_independent_spherical_coordinates() {
    let mut ctx = context();
    let projection = SphericalTransverseMercator::new(
        Length::metres(5.into()),
        reference(),
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::RejectOutsideChart),
        &mut ctx,
    )
    .unwrap();
    // eta=ln(2) gives sinh eta=3/4, cosh eta=5/4.
    // xi=pi/4 then gives sin phi=2sqrt(2)/5, cos phi=sqrt(17)/5,
    // and delta=atan(3sqrt(2)/4). ECEF is (2sqrt(2),3,2sqrt(2)).
    let raw = projection
        .position(
            Point3::new(
                Real::from(5) * Real::from(2).ln().unwrap(),
                (Real::from(5) * Real::pi() / Real::from(4)).unwrap(),
                0.into(),
            ),
            input(),
            &mut ctx,
        )
        .unwrap();
    let inverse = projection
        .unproject(&raw, PoleMeridian::Reject, &mut ctx)
        .unwrap();
    let ecef = inverse.to_ecef(&mut ctx).unwrap();
    close(
        &ecef.coordinates().x,
        &(Real::from(2) * Real::from(2).sqrt().unwrap()),
    );
    close(&ecef.coordinates().y, &Real::from(3));
    close(&ecef.coordinates().z, &ecef.coordinates().x);
    let local = LocalFrame::new(&inverse, LocalAxes::Enu, &mut ctx)
        .unwrap()
        .to_local_position(&ecef, &mut ctx)
        .unwrap();
    equal(&local.coordinates().x, &Real::zero());
    equal(&local.coordinates().y, &Real::zero());
    equal(&local.coordinates().z, &Real::zero());
    let fresh = GeodeticPosition::new(
        Latitude::new(inverse.latitude().angle().clone(), &mut ctx).unwrap(),
        Longitude::unrolled(inverse.longitude().angle().clone()),
        inverse.height().clone(),
        inverse.ellipsoid().clone(),
        inverse.reference().clone(),
        input(),
        &mut ctx,
    )
    .unwrap();
    let replay = projection.project(&fresh, &mut ctx).unwrap();
    close(&replay.coordinates().x, &raw.coordinates().x);
    close(&replay.coordinates().y, &raw.coordinates().y);
}

fn chart(ctx: &mut Context) -> SphericalTransverseMercator {
    SphericalTransverseMercator::new(
        Length::metres(1.into()),
        reference(),
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::RejectOutsideChart),
        ctx,
    )
    .unwrap()
}

#[test]
fn poles_have_explicit_meridians_and_equatorial_chart_edges_are_singular() {
    let mut ctx = context();
    let projection = chart(&mut ctx);
    for latitude in [-90, 90] {
        let source = position(
            latitude.into(),
            720.into(),
            (-2).into(),
            projection.ellipsoid().clone(),
            &mut ctx,
        );
        let point = projection.project(&source, &mut ctx).unwrap();
        equal(&point.coordinates().x, &Real::zero());
        equal(
            &point.coordinates().y,
            Angle::degrees(latitude.into()).as_radians(),
        );
        assert!(matches!(
            projection.unproject(&point, PoleMeridian::Reject, &mut ctx),
            Err(Error::AmbiguousInverse(_))
        ));
        for fresh in [false, true] {
            let point = if fresh {
                projection
                    .position(point.coordinates().clone(), input(), &mut ctx)
                    .unwrap()
            } else {
                point.clone()
            };
            let inverse = projection
                .unproject(
                    &point,
                    PoleMeridian::Use(Longitude::unrolled(Angle::degrees(123.into()))),
                    &mut ctx,
                )
                .unwrap();
            equal(&inverse.longitude().angle().as_degrees(), &Real::from(123));
            equal(
                &inverse.latitude().angle().as_degrees(),
                &Real::from(latitude),
            );
            let GeodeticSource::SphericalTransverseMercator { pole_meridian, .. } =
                inverse.source()
            else {
                panic!("lost inverse source")
            };
            assert!(pole_meridian.is_some());
            let ecef = inverse.to_ecef(&mut ctx).unwrap();
            let local = LocalFrame::new(&inverse, LocalAxes::Ned, &mut ctx)
                .unwrap()
                .to_local_position(&ecef, &mut ctx)
                .unwrap();
            equal(&local.coordinates().z, &Real::zero());
            equal(
                &projection
                    .project(&inverse, &mut ctx)
                    .unwrap()
                    .coordinates()
                    .y,
                &point.coordinates().y,
            );
        }
    }
    for longitude in [-90, 90] {
        let source = position(
            0.into(),
            longitude.into(),
            0.into(),
            projection.ellipsoid().clone(),
            &mut ctx,
        );
        assert!(matches!(
            projection.project(&source, &mut ctx),
            Err(Error::ProjectionSingularity(_))
        ));
        let source = position(
            45.into(),
            longitude.into(),
            0.into(),
            projection.ellipsoid().clone(),
            &mut ctx,
        );
        let point = projection.project(&source, &mut ctx).unwrap();
        equal(
            &point.coordinates().y,
            &(Real::pi() / Real::from(2)).unwrap(),
        );
        close(
            &point.coordinates().x,
            &(Real::from(if longitude < 0 { -1 } else { 1 }) * Real::one().asinh().unwrap()),
        );
        let raw = projection
            .position(point.coordinates().clone(), input(), &mut ctx)
            .unwrap();
        let inverse = projection
            .unproject(&raw, PoleMeridian::Reject, &mut ctx)
            .unwrap();
        close(&inverse.latitude().angle().as_degrees(), &Real::from(45));
        close(
            &inverse.longitude().angle().as_degrees(),
            &Real::from(longitude),
        );
    }
}

#[test]
fn false_origins_longitude_policy_and_non_spherical_sources_are_explicit() {
    let mut ctx = context();
    for policy in [
        TransverseLongitudePolicy::RejectOutsideChart,
        TransverseLongitudePolicy::WrapToChart,
    ] {
        let mut parameters = TransverseMercatorParameters::greenwich(policy);
        parameters.central_meridian = Longitude::unrolled(Angle::degrees(15.into()));
        parameters.latitude_of_origin = Angle::degrees((-30).into());
        parameters.false_easting = Length::metres(123.into());
        parameters.false_northing = Length::metres((-45).into());
        parameters.scale = 2.into();
        let projection = SphericalTransverseMercator::new(
            Length::metres(3.into()),
            reference(),
            parameters,
            &mut ctx,
        )
        .unwrap();
        assert!(projection.same_definition(&projection.clone()));
        assert_eq!(projection.parameters().scale, Real::from(2));
        for turns in [-1, 0, 1] {
            let source = position(
                (-30).into(),
                (15 + turns * 360).into(),
                (-2).into(),
                projection.ellipsoid().clone(),
                &mut ctx,
            );
            let output = projection.project(&source, &mut ctx);
            if turns != 0 && policy == TransverseLongitudePolicy::RejectOutsideChart {
                assert!(matches!(output, Err(Error::InvalidInput(_))));
            } else {
                let point = output.unwrap();
                close(&point.coordinates().x, &Real::from(123));
                close(&point.coordinates().y, &Real::from(-45));
                let inverse = projection
                    .unproject(&point, PoleMeridian::Reject, &mut ctx)
                    .unwrap();
                equal(&inverse.longitude().angle().as_degrees(), &Real::from(15));
                equal(inverse.height().as_metres(), &Real::from(-2));
            }
        }
        for longitude in [-180, -90, 106, 180] {
            let source = position(
                30.into(),
                longitude.into(),
                0.into(),
                projection.ellipsoid().clone(),
                &mut ctx,
            );
            assert!(matches!(
                projection.project(&source, &mut ctx),
                Err(Error::InvalidInput(_))
            ));
        }
        let surface = std::sync::Arc::new(
            Ellipsoid::from_axes(Length::metres(3.into()), Length::metres(2.into()), &mut ctx)
                .unwrap(),
        );
        let wrong = position(0.into(), 15.into(), 0.into(), surface, &mut ctx);
        assert!(matches!(
            projection.project(&wrong, &mut ctx),
            Err(Error::EllipsoidMismatch)
        ));
    }
    for scale in [0, -1] {
        let mut parameters =
            TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::WrapToChart);
        parameters.scale = scale.into();
        assert!(matches!(
            SphericalTransverseMercator::new(
                Length::metres(1.into()),
                reference(),
                parameters,
                &mut ctx
            ),
            Err(Error::InvalidInput(_))
        ));
    }
    let mut parameters =
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::WrapToChart);
    parameters.latitude_of_origin = Angle::degrees(91.into());
    assert!(matches!(
        SphericalTransverseMercator::new(
            Length::metres(1.into()),
            reference(),
            parameters,
            &mut ctx
        ),
        Err(Error::InvalidInput(_))
    ));
}

#[test]
fn huge_eastings_keep_the_joint_inverse_relation_across_clones_and_changed_origins() {
    let mut ctx = context();
    let projection = chart(&mut ctx);
    for easting in [
        r("1e320"),
        r("-1e320"),
        r("1e320") + Real::pi(),
        r("-1e320") * Real::e(),
    ] {
        for xi in [
            Real::zero(),
            (Real::pi() / Real::from(4)).unwrap(),
            (Real::pi() / Real::from(2)).unwrap(),
        ] {
            let point = projection
                .position(
                    Point3::new(easting.clone(), xi.clone(), (-2).into()),
                    input(),
                    &mut ctx,
                )
                .unwrap();
            let inverse = projection
                .unproject(&point, PoleMeridian::Reject, &mut ctx)
                .unwrap();
            close(inverse.latitude().angle().as_radians(), &Real::zero());
            let expected_lon = if easting < Real::zero() {
                -(Real::pi() / Real::from(2)).unwrap()
            } else {
                (Real::pi() / Real::from(2)).unwrap()
            };
            close(inverse.longitude().angle().as_radians(), &expected_lon);
            let cloned = GeodeticPosition::new(
                inverse.latitude().clone(),
                inverse.longitude().clone(),
                inverse.height().clone(),
                inverse.ellipsoid().clone(),
                reference(),
                input(),
                &mut ctx,
            )
            .unwrap();
            let mut parameters =
                TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::WrapToChart);
            parameters.scale = 3.into();
            parameters.latitude_of_origin = Angle::degrees(15.into());
            parameters.false_northing = Length::metres(7.into());
            let changed = SphericalTransverseMercator::new(
                Length::metres(1.into()),
                reference(),
                parameters,
                &mut ctx,
            )
            .unwrap();
            let replay = changed.project(&cloned, &mut ctx).unwrap();
            equal(&replay.coordinates().x, &(Real::from(3) * &easting));
            equal(
                &replay.coordinates().y,
                &(Real::from(7) + Real::from(3) * (&xi - Angle::degrees(15.into()).as_radians())),
            );
            let ecef = cloned.to_ecef(&mut ctx).unwrap();
            close(&ecef.coordinates().x, &Real::zero());
            close(&ecef.coordinates().z, &Real::zero());
            close(
                &ecef.coordinates().y,
                &Real::from(if easting < Real::zero() { 1 } else { -1 }),
            );
        }
    }
}

#[test]
fn retained_forward_proof_avoids_reproving_a_northing_arbitrarily_close_to_a_pole() {
    let mut ctx = context();
    let projection = chart(&mut ctx);
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let near = &half_pi - r("1e-150");
    let source = GeodeticPosition::new(
        Latitude::new(Angle::radians(near.clone()), &mut ctx).unwrap(),
        Longitude::unrolled(Angle::radians(near)),
        Length::metres(0.into()),
        projection.ellipsoid().clone(),
        reference(),
        input(),
        &mut ctx,
    )
    .unwrap();
    let point = projection.project(&source, &mut ctx).unwrap();
    let inverse = projection
        .unproject(&point, PoleMeridian::Reject, &mut ctx)
        .unwrap();
    equal(
        inverse.latitude().angle().as_radians(),
        source.latitude().angle().as_radians(),
    );
    equal(
        inverse.longitude().angle().as_radians(),
        source.longitude().angle().as_radians(),
    );
    close(&point.coordinates().y, &half_pi);
    let replay = projection.project(&inverse, &mut ctx).unwrap();
    equal(&replay.coordinates().x, &point.coordinates().x);
    equal(&replay.coordinates().y, &point.coordinates().y);
}

#[test]
fn domain_mismatch_batches_cancellation_and_unknown_boundaries_remain_typed() {
    let mut ctx = context();
    let projection = chart(&mut ctx);
    let source = position(
        30.into(),
        45.into(),
        (-1).into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    let point = projection.project(&source, &mut ctx).unwrap();
    let other = chart(&mut ctx);
    assert!(!projection.same_definition(&other));
    assert!(matches!(
        other.unproject(&point, PoleMeridian::Reject, &mut ctx),
        Err(Error::ProjectionMismatch)
    ));
    for reference in [
        ReferenceFrame::fixed("other").unwrap(),
        ReferenceFrame::at_julian_year(reference().name(), 2000.into()).unwrap(),
    ] {
        let wrong = GeodeticPosition::new(
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
            projection.project(&wrong, &mut ctx),
            Err(Error::ReferenceMismatch)
        ));
    }
    for northing in [Real::pi(), -Real::pi()] {
        assert!(matches!(
            projection.position(Point3::new(0.into(), northing, 0.into()), input(), &mut ctx),
            Err(Error::InvalidInput(_))
        ));
    }
    let s = Real::e().sin();
    let c = Real::e().cos();
    let zero = &s * &s + &c * &c - Real::one();
    assert!(matches!(
        projection.position(
            Point3::new(
                1.into(),
                (Real::pi() / Real::from(2)).unwrap() + zero,
                0.into()
            ),
            input(),
            &mut ctx
        ),
        Err(Error::Unknown { .. })
    ));
    assert!(projection.project_batch(&[], &mut ctx).unwrap().is_empty());
    assert!(
        projection
            .unproject_batch(&[], PoleMeridian::Reject, &mut ctx)
            .unwrap()
            .is_empty()
    );
    let batch = projection
        .project_batch(&[source.clone(), source.clone()], &mut ctx)
        .unwrap();
    assert_eq!(
        projection
            .unproject_batch(&batch, PoleMeridian::Reject, &mut ctx)
            .unwrap()
            .len(),
        2
    );
    let singular = position(
        0.into(),
        90.into(),
        0.into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    assert!(matches!(
        projection.project_batch(&[source.clone(), singular], &mut ctx),
        Err(Error::ProjectionSingularity(_))
    ));
    let foreign = other.project(&source, &mut ctx).unwrap();
    assert!(matches!(
        projection.unproject_batch(&[point.clone(), foreign], PoleMeridian::Reject, &mut ctx),
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
            projection.unproject_batch(&batch, PoleMeridian::Reject, &mut limited),
            Err(Error::ResourceExhausted(_))
        ));
    }
    let signal = ctx.cancellation_signal();
    signal.store(true, std::sync::atomic::Ordering::Relaxed);
    assert!(matches!(
        projection.unproject(&point, PoleMeridian::Reject, &mut ctx),
        Err(Error::Cancelled)
    ));
    signal.store(false, std::sync::atomic::Ordering::Relaxed);
    let mut point = point;
    for _ in 0..12 {
        let inverse = projection
            .unproject(&point, PoleMeridian::Reject, &mut ctx)
            .unwrap();
        point = projection.project(&inverse, &mut ctx).unwrap();
    }
    assert_ne!(point.receipt().certainty(), Certainty::Approximate);
}

#[test]
fn longitude_qualification_survives_new_positions_projection_parameters_and_pole_conventions() {
    let mut strict = context();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    let projection = SphericalTransverseMercator::new(
        Length::metres(1.into()),
        ReferenceFrame::at_julian_year("epoch", 2000.into()).unwrap(),
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::RejectOutsideChart),
        &mut strict,
    )
    .unwrap();
    let source = GeodeticPosition::new(
        Latitude::new(Angle::degrees(30.into()), &mut strict).unwrap(),
        Longitude::unrolled(Angle::degrees(45.into())),
        Length::metres(2.into()),
        projection.ellipsoid().clone(),
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
    let projected = projection.project(&source, &mut approximate).unwrap();
    let inverse = projection
        .unproject(&projected, PoleMeridian::Reject, &mut strict)
        .unwrap();
    let longitude = inverse.longitude().clone();
    let fresh = GeodeticPosition::new(
        Latitude::new(Angle::degrees(0.into()), &mut strict).unwrap(),
        longitude.clone(),
        Length::metres(0.into()),
        projection.ellipsoid().clone(),
        reference(),
        input(),
        &mut strict,
    )
    .unwrap();
    let mut mercator_parameters = MercatorParameters::greenwich(LongitudePolicy::Unrolled);
    mercator_parameters.central_meridian = longitude.clone();
    let mercator = SphericalMercator::new(
        projection.ellipsoid().clone(),
        reference(),
        mercator_parameters.clone(),
        &mut strict,
    )
    .unwrap();
    let ellipsoidal = EllipsoidalMercator::new(
        projection.ellipsoid().clone(),
        reference(),
        mercator_parameters,
        &mut strict,
    )
    .unwrap();
    let mut transverse_parameters =
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::RejectOutsideChart);
    transverse_parameters.central_meridian = longitude.clone();
    let transverse = SphericalTransverseMercator::new(
        Length::metres(1.into()),
        reference(),
        transverse_parameters,
        &mut strict,
    )
    .unwrap();
    let pole = EcefPosition::new(
        Point3::new(0.into(), 0.into(), 2.into()),
        reference(),
        input(),
        &mut strict,
    )
    .unwrap();
    let pole = pole
        .to_geodetic_closest(
            projection.ellipsoid().clone(),
            PoleMeridian::Use(longitude.clone()),
            &mut strict,
        )
        .unwrap();
    let exact_chart = chart(&mut strict);
    let chart_pole = exact_chart
        .position(
            Point3::new(0.into(), (Real::pi() / Real::from(2)).unwrap(), 0.into()),
            input(),
            &mut strict,
        )
        .unwrap();
    assert_ne!(chart_pole.receipt().certainty(), Certainty::Approximate);
    let chart_pole = exact_chart
        .unproject(&chart_pole, PoleMeridian::Use(longitude), &mut strict)
        .unwrap();
    for receipt in [
        inverse.receipt(),
        fresh.receipt(),
        mercator.receipt(),
        ellipsoidal.receipt(),
        transverse.receipt(),
        pole.receipt(),
        chart_pole.receipt(),
    ] {
        assert_eq!(receipt.certainty(), Certainty::Approximate);
        let mut pending = vec![receipt];
        let mut found = false;
        while let Some(node) = pending.pop() {
            found |= node.input().is_some_and(|p| {
                p.source() == "qualified station" && p.accuracy() == &InputAccuracy::Unspecified
            });
            pending.extend(node.parents());
        }
        assert!(found, "discarded longitude input qualification");
    }
}

#[test]
fn changed_latitude_or_central_meridian_cannot_reuse_an_unrelated_inverse_relation() {
    let mut ctx = context();
    let projection = chart(&mut ctx);
    let original = position(
        30.into(),
        40.into(),
        0.into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    let point = projection.project(&original, &mut ctx).unwrap();
    let inverse = projection
        .unproject(&point, PoleMeridian::Reject, &mut ctx)
        .unwrap();
    let changed = GeodeticPosition::new(
        Latitude::new(Angle::degrees(60.into()), &mut ctx).unwrap(),
        inverse.longitude().clone(),
        Length::metres(0.into()),
        projection.ellipsoid().clone(),
        reference(),
        input(),
        &mut ctx,
    )
    .unwrap();
    let independent = position(
        60.into(),
        40.into(),
        0.into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    let from_relation = projection.project(&changed, &mut ctx).unwrap();
    let expected = projection.project(&independent, &mut ctx).unwrap();
    close(&from_relation.coordinates().x, &expected.coordinates().x);
    close(&from_relation.coordinates().y, &expected.coordinates().y);
    let mut parameters =
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::RejectOutsideChart);
    parameters.central_meridian = Longitude::unrolled(Angle::degrees((-100).into()));
    let shifted = SphericalTransverseMercator::new(
        Length::metres(1.into()),
        reference(),
        parameters,
        &mut ctx,
    )
    .unwrap();
    assert!(matches!(
        shifted.project(&inverse, &mut ctx),
        Err(Error::InvalidInput(_))
    ));
}

#[test]
fn inverse_switch_and_tiny_eastings_preserve_fresh_cartesian_identities() {
    let mut ctx = context();
    let projection = chart(&mut ctx);
    let tiny = r("1e-150");
    for eta in [
        Real::from(-1) - &tiny,
        Real::from(-1),
        Real::from(-1) + &tiny,
        -&tiny,
        Real::zero(),
        tiny.clone(),
        Real::one() - &tiny,
        Real::one(),
        Real::one() + &tiny,
    ] {
        let xi = (Real::pi() / Real::from(6)).unwrap();
        let point = projection
            .position(
                Point3::new(eta.clone(), xi.clone(), 0.into()),
                input(),
                &mut ctx,
            )
            .unwrap();
        let inverse = projection
            .unproject(&point, PoleMeridian::Reject, &mut ctx)
            .unwrap();
        let ecef = inverse.to_ecef(&mut ctx).unwrap();
        let inverse_cosh = eta.clone().cosh().unwrap().inverse().unwrap();
        close(&ecef.coordinates().x, &(xi.clone().cos() * &inverse_cosh));
        close(&ecef.coordinates().y, &eta.tanh().unwrap());
        close(&ecef.coordinates().z, &(xi.sin() * inverse_cosh));
    }
    for eta in [tiny.clone(), -tiny] {
        let point = projection
            .position(
                Point3::new(eta.clone(), (Real::pi() / Real::from(2)).unwrap(), 0.into()),
                input(),
                &mut ctx,
            )
            .unwrap();
        let inverse = projection
            .unproject(&point, PoleMeridian::Reject, &mut ctx)
            .unwrap();
        let replay = projection.project(&inverse, &mut ctx).unwrap();
        equal(&replay.coordinates().x, &eta);
        equal(&replay.coordinates().y, &point.coordinates().y);
    }
}

#[test]
fn approximate_strip_equality_cannot_become_a_strict_endpoint_certificate() {
    let mut strict = context();
    let projection = chart(&mut strict);
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    let y = (Real::pi() / Real::from(2)).unwrap() + zero;
    let coordinates = Point3::new(1.into(), y.clone(), 0.into());
    assert!(matches!(
        projection.position(coordinates.clone(), input(), &mut strict),
        Err(Error::Unknown { .. })
    ));
    let mut approximate = Context::new(PredicatePolicy::APPROXIMATE_512, Limits::default());
    let point = projection
        .position(coordinates, input(), &mut approximate)
        .unwrap();
    assert_eq!(point.coordinates().y, y);
    assert_eq!(point.receipt().certainty(), Certainty::Approximate);
    assert!(matches!(
        projection.unproject(&point, PoleMeridian::Reject, &mut strict),
        Err(Error::Unknown { .. })
    ));
    let inverse = projection
        .unproject(&point, PoleMeridian::Reject, &mut approximate)
        .unwrap();
    assert_eq!(inverse.receipt().certainty(), Certainty::Approximate);
    assert_eq!(
        inverse.longitude().receipt().unwrap().certainty(),
        Certainty::Approximate
    );
}

#[test]
fn unresolved_internal_switch_uses_an_overlapping_certified_formula_domain() {
    let mut ctx = context();
    let projection = chart(&mut ctx);
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    for eta in [Real::one() + &zero, -Real::one() - zero] {
        let point = projection
            .position(
                Point3::new(eta.clone(), 0.into(), 0.into()),
                input(),
                &mut ctx,
            )
            .unwrap();
        let inverse = projection
            .unproject(&point, PoleMeridian::Reject, &mut ctx)
            .unwrap();
        equal(inverse.latitude().angle().as_radians(), &Real::zero());
        let ecef = inverse.to_ecef(&mut ctx).unwrap();
        close(
            &ecef.coordinates().x,
            &eta.clone().cosh().unwrap().inverse().unwrap(),
        );
        close(&ecef.coordinates().y, &eta.tanh().unwrap());
        assert_ne!(inverse.receipt().certainty(), Certainty::Approximate);
    }
}
