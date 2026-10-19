mod common;
use common::*;
use hyperlattice::Point3;
use hyperspatial::*;
use std::{cmp::Ordering, sync::Arc};

#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

fn close(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(
            &(actual - expected).abs(),
            &r("0.000000000000000000000000000001"),
            PredicatePolicy::STRICT
        )
        .value(),
        Some(Ordering::Less)
    );
}

fn sphere_projection(policy: LongitudePolicy, ctx: &mut Context) -> SphericalMercator {
    let surface = Arc::new(Ellipsoid::sphere(Length::metres(Real::one()), ctx).unwrap());
    SphericalMercator::new(
        surface,
        reference(),
        MercatorParameters::greenwich(policy),
        ctx,
    )
    .unwrap()
}

fn fresh_projection(
    point: &ProjectedPosition<SphericalMercator>,
    ctx: &mut Context,
) -> ProjectedPosition<SphericalMercator> {
    point
        .projection()
        .position(point.coordinates().clone(), input(), ctx)
        .unwrap()
}

fn fresh_geodetic(point: &GeodeticPosition, ctx: &mut Context) -> GeodeticPosition {
    GeodeticPosition::new(
        point.latitude().clone(),
        point.longitude().clone(),
        point.height().clone(),
        point.ellipsoid().clone(),
        point.reference().clone(),
        input(),
        ctx,
    )
    .unwrap()
}

#[test]
fn web_mercator_equator_preserves_exact_height_and_source() {
    let mut ctx = context();
    let projection =
        SphericalMercator::web_mercator(reference(), LongitudePolicy::Unrolled, &mut ctx).unwrap();
    let origin = position(
        0.into(),
        0.into(),
        (-12).into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    let projected = projection.project(&origin, &mut ctx).unwrap();
    equal(&projected.coordinates().x, &Real::zero());
    equal(&projected.coordinates().y, &Real::zero());
    equal(&projected.coordinates().z, &Real::from(-12));
    assert!(projected.source_geodetic().is_some());
    let inverse = projection.unproject(&projected, &mut ctx).unwrap();
    assert!(matches!(
        inverse.source(),
        GeodeticSource::SphericalMercator(_)
    ));
    equal(inverse.height().as_metres(), &Real::from(-12));
    let ecef = inverse.to_ecef(&mut ctx).unwrap();
    equal(&ecef.coordinates().x, &Real::from(6_378_125));
    let local = LocalFrame::new(&inverse, LocalAxes::Enu, &mut ctx).unwrap();
    equal(
        &local
            .to_local_position(&ecef, &mut ctx)
            .unwrap()
            .coordinates()
            .z,
        &Real::zero(),
    );
}

#[test]
fn independent_logarithmic_inverse_and_false_origin_match_analytic_coordinates() {
    let mut ctx = context();
    let mut parameters = MercatorParameters::greenwich(LongitudePolicy::Unrolled);
    parameters.scale = 2.into();
    parameters.central_meridian = Longitude::unrolled(Angle::degrees(15.into()));
    parameters.false_easting = Length::metres(100.into());
    parameters.false_northing = Length::metres((-50).into());
    let surface = Arc::new(Ellipsoid::sphere(Length::metres(3.into()), &mut ctx).unwrap());
    let projection = SphericalMercator::new(surface, reference(), parameters, &mut ctx).unwrap();
    let x = Real::from(100) + Real::pi();
    let y = Real::from(-50) + Real::from(6) * Real::from(2).ln().unwrap();
    let point = projection
        .position(
            Point3::new(x.clone(), y.clone(), (-1).into()),
            input(),
            &mut ctx,
        )
        .unwrap();
    let inverse = projection.unproject(&point, &mut ctx).unwrap();
    // sinh(ln(2)) = (2 - 1/2)/2 = 3/4; source sphere radius+height is 2.
    close(
        inverse.latitude().angle().as_radians(),
        &r("0.75").atan().unwrap(),
    );
    equal(&inverse.longitude().angle().as_degrees(), &Real::from(45));
    let cartesian = inverse.to_ecef(&mut ctx).unwrap();
    close(
        &cartesian.coordinates().x,
        &(r("0.8") * Real::from(2).sqrt().unwrap()),
    );
    close(
        &cartesian.coordinates().y,
        &(r("0.8") * Real::from(2).sqrt().unwrap()),
    );
    close(&cartesian.coordinates().z, &r("1.2"));
    let replay = projection
        .project(&fresh_geodetic(&inverse, &mut ctx), &mut ctx)
        .unwrap();
    close(&replay.coordinates().x, &x);
    close(&replay.coordinates().y, &y);
    equal(&replay.coordinates().z, &Real::from(-1));
    assert_eq!(projection.parameters().scale, Real::from(2));
    assert_eq!(projection.reference().name(), reference().name());
    assert_ne!(projection.receipt().certainty(), Certainty::Approximate);
}

#[test]
fn proj_published_web_example_and_tile_boundary_are_independent_checks() {
    let mut ctx = context();
    let projection =
        SphericalMercator::web_mercator(reference(), LongitudePolicy::WrapSigned, &mut ctx)
            .unwrap();
    let source = position(
        49.into(),
        2.into(),
        123.into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    let point = projection.project(&source, &mut ctx).unwrap();
    // https://proj.org/en/stable/operations/projections/webmerc.html, centimetre display.
    rounded(&point.coordinates().x, "222638.98", "0.005");
    rounded(&point.coordinates().y, "6274861.39", "0.005");
    let inverse = projection
        .unproject(&fresh_projection(&point, &mut ctx), &mut ctx)
        .unwrap();
    close(
        inverse.latitude().angle().as_radians(),
        source.latitude().angle().as_radians(),
    );
    let high = position(
        89.into(),
        0.into(),
        0.into(),
        projection.ellipsoid().clone(),
        &mut ctx,
    );
    let high = projection.project(&high, &mut ctx).unwrap();
    let half_width = Real::from(6_378_137) * Real::pi();
    assert_eq!(
        hyperlimit::compare_reals(&high.coordinates().y, &half_width, PredicatePolicy::STRICT)
            .value(),
        Some(Ordering::Greater)
    );
    let tile = projection
        .position(
            Point3::new(0.into(), half_width, 0.into()),
            input(),
            &mut ctx,
        )
        .unwrap();
    let tile_latitude = projection.unproject(&tile, &mut ctx).unwrap();
    rounded(
        &tile_latitude.latitude().angle().as_degrees(),
        "85.0511287798066",
        "0.0000000000001",
    );
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
        let projection = sphere_projection(policy, &mut ctx);
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
    let shifted = SphericalMercator::new(surface, reference(), parameters, &mut ctx).unwrap();
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
    let projection = sphere_projection(LongitudePolicy::Unrolled, &mut ctx);
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
            SphericalMercator::new(
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
    let other = sphere_projection(LongitudePolicy::Unrolled, &mut ctx);
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
fn scalar_families_and_large_unrolled_longitude_use_native_values() {
    let mut ctx = context();
    let projection = sphere_projection(LongitudePolicy::Unrolled, &mut ctx);
    for case in scalar_cases::optimized_certificate_representatives() {
        assert_eq!(
            case.value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        let latitude = Latitude::new(
            Angle::radians((&case.value / Real::from(100)).unwrap().atan().unwrap()),
            &mut ctx,
        )
        .unwrap();
        let source = GeodeticPosition::new(
            latitude,
            Longitude::unrolled(Angle::radians(case.value.clone())),
            Length::metres(-&case.value),
            projection.ellipsoid().clone(),
            reference(),
            input(),
            &mut ctx,
        )
        .unwrap();
        let point = projection.project(&source, &mut ctx).unwrap();
        equal(&point.coordinates().x, &case.value);
        let inverse = projection
            .unproject(&fresh_projection(&point, &mut ctx), &mut ctx)
            .unwrap();
        close(
            inverse.latitude().angle().as_radians(),
            source.latitude().angle().as_radians(),
        );
        equal(inverse.height().as_metres(), &(-case.value));
    }
    for exponent in [310, 320] {
        let huge: Real = format!("1{}", "0".repeat(exponent)).parse().unwrap();
        let source = position(
            0.into(),
            huge.clone(),
            huge.clone(),
            projection.ellipsoid().clone(),
            &mut ctx,
        );
        let point = projection.project(&source, &mut ctx).unwrap();
        // 10^310 degrees becomes ~1.745e308 radians, just inside binary64.
        // Preserve that boundary case and also exceed it by ten more decades.
        assert_eq!(
            point.coordinates().x.to_f64_lossy().is_none(),
            exponent == 320
        );
        assert!(point.coordinates().z.to_f64_lossy().is_none());
        let inverse = projection
            .unproject(&fresh_projection(&point, &mut ctx), &mut ctx)
            .unwrap();
        equal(&inverse.longitude().angle().as_degrees(), &huge);
        equal(inverse.height().as_metres(), &huge);
    }
}

#[test]
fn inherited_approximate_epoch_and_input_qualification_survive_inverse_and_local_use() {
    let mut strict = context();
    let s = Real::e().sin();
    let c = Real::e().cos();
    let zero = &s * &s + &c * &c - Real::one();
    let surface = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut strict).unwrap());
    let projection = SphericalMercator::new(
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
    let projection = sphere_projection(LongitudePolicy::WrapSigned, &mut ctx);
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
    let other = sphere_projection(LongitudePolicy::WrapSigned, &mut ctx);
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
