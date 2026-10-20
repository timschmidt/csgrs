mod common;
use common::*;
use hyperspatial::*;
use std::sync::Arc;

#[test]
fn every_ordinary_zone_has_exact_parameters_and_retained_source() {
    let mut ctx = context();
    let sphere = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut ctx).unwrap());
    for number in 1..=60 {
        let longitude = Real::from(6 * i32::from(number) - 183);
        for (latitude, hemisphere, northing) in [
            (0, Hemisphere::North, 0),
            (-1, Hemisphere::South, 10_000_000),
        ] {
            let source = position(
                latitude.into(),
                longitude.clone(),
                (-12).into(),
                sphere.clone(),
                &mut ctx,
            );
            let selected = UtmSelection::select(&source, &mut ctx).unwrap();
            assert_eq!(selected.zone(), UtmZone::new(number).unwrap());
            assert_eq!(selected.hemisphere(), hemisphere);
            equal(selected.source().height().as_metres(), &Real::from(-12));
            let parameters = selected.parameters().unwrap();
            equal(
                &parameters.central_meridian.angle().as_degrees(),
                &longitude,
            );
            equal(&parameters.scale, &r("0.9996"));
            equal(parameters.false_easting.as_metres(), &Real::from(500_000));
            equal(parameters.false_northing.as_metres(), &Real::from(northing));
            assert!(parameters.receipt().is_some());
        }
    }
}

#[test]
fn exact_zone_edges_and_special_areas_obey_half_open_intervals() {
    let mut ctx = context();
    let sphere = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut ctx).unwrap());
    let tiny = r("1e-60");
    for number in 1..=60 {
        let boundary = Real::from(6 * number - 186);
        for (longitude, expected) in [
            (boundary.clone(), number),
            (&boundary + &tiny, number),
            (&boundary - &tiny, number - 1),
        ] {
            let source = position(0.into(), longitude, 0.into(), sphere.clone(), &mut ctx);
            let selection = UtmSelection::select(&source, &mut ctx);
            if expected == 0 {
                assert!(matches!(selection, Err(Error::InvalidInput(_))));
            } else {
                assert_eq!(i32::from(selection.unwrap().zone().number()), expected);
            }
        }
    }
    for (latitude, longitude, expected) in [
        ("56", "3", 32),
        ("56", "2.999999", 31),
        ("56", "11.999999", 32),
        ("56", "12", 33),
        ("55.999999", "3", 31),
        ("63.999999", "3", 32),
        ("64", "3", 31),
        ("72", "-0.000001", 30),
        ("72", "0", 31),
        ("72", "8.999999", 31),
        ("72", "9", 33),
        ("72", "20.999999", 33),
        ("72", "21", 35),
        ("72", "32.999999", 35),
        ("72", "33", 37),
        ("72", "41.999999", 37),
        ("72", "42", 38),
        ("71.999999", "9", 32),
        ("83.999999", "9", 33),
        ("-80", "179.999999", 60),
    ] {
        let source = position(
            r(latitude),
            r(longitude),
            0.into(),
            sphere.clone(),
            &mut ctx,
        );
        assert_eq!(
            UtmSelection::select(&source, &mut ctx)
                .unwrap()
                .zone()
                .number(),
            expected
        );
    }
    for (latitude, longitude) in [(-81, 0), (84, 0), (90, 0), (0, 180), (0, -181), (0, 540)] {
        let source = position(
            latitude.into(),
            longitude.into(),
            0.into(),
            sphere.clone(),
            &mut ctx,
        );
        assert!(matches!(
            UtmSelection::select(&source, &mut ctx),
            Err(Error::InvalidInput(_))
        ));
    }
    let sub_float = r("1e-500");
    for (latitude, hemisphere) in [
        (-&sub_float, Hemisphere::South),
        (Real::zero(), Hemisphere::North),
        (sub_float.clone(), Hemisphere::North),
    ] {
        let source = position(latitude, 0.into(), 0.into(), sphere.clone(), &mut ctx);
        assert_eq!(
            UtmSelection::select(&source, &mut ctx)
                .unwrap()
                .hemisphere(),
            hemisphere
        );
    }
    for (latitude, valid) in [
        (Real::from(-80) - &sub_float, false),
        (Real::from(-80) + &sub_float, true),
        (Real::from(84) - &sub_float, true),
        (Real::from(84) + &sub_float, false),
    ] {
        let source = position(latitude, 0.into(), 0.into(), sphere.clone(), &mut ctx);
        let selected = UtmSelection::select(&source, &mut ctx);
        if valid {
            assert!(selected.is_ok());
        } else {
            assert!(matches!(selected, Err(Error::InvalidInput(_))));
        }
    }
    for invalid in [0, 61, 255] {
        assert!(matches!(UtmZone::new(invalid), Err(Error::InvalidInput(_))));
    }
}

#[test]
fn unresolved_selection_stays_unknown_and_approximate_parameter_evidence_is_sticky() {
    let mut strict = context();
    let sphere = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut strict).unwrap());
    let zero = unresolved_zero();
    let source = position(
        56.into(),
        Real::from(3) + zero,
        0.into(),
        sphere.clone(),
        &mut strict,
    );
    assert!(matches!(
        UtmSelection::select(&source, &mut strict),
        Err(Error::Unknown { .. })
    ));
    let mut approximate = Context::new(PredicatePolicy::APPROXIMATE_512, Limits::default());
    let selection = UtmSelection::select(&source, &mut approximate).unwrap();
    assert_eq!(selection.zone().number(), 32);
    assert_eq!(selection.receipt().certainty(), Certainty::Approximate);
    let parameters = selection.parameters().unwrap();
    assert_eq!(
        parameters.receipt().unwrap().certainty(),
        Certainty::Approximate
    );
    let projection = SphericalTransverseMercator::new(
        Length::metres(1.into()),
        reference(),
        parameters,
        &mut strict,
    )
    .unwrap();
    assert_eq!(projection.receipt().certainty(), Certainty::Approximate);
    let independent = position(20.into(), 10.into(), 0.into(), sphere, &mut strict);
    let projected = projection.project(&independent, &mut strict).unwrap();
    assert_eq!(projected.receipt().certainty(), Certainty::Approximate);
    let inverse = projection
        .unproject(&projected, PoleMeridian::Reject, &mut strict)
        .unwrap();
    assert_eq!(inverse.receipt().certainty(), Certainty::Approximate);
}

#[test]
fn batches_cancellation_and_resource_limits_publish_no_partial_selection() {
    let mut ctx = context();
    let sphere = Arc::new(Ellipsoid::sphere(Length::metres(1.into()), &mut ctx).unwrap());
    let source = position(0.into(), 0.into(), 0.into(), sphere.clone(), &mut ctx);
    let invalid = position(84.into(), 0.into(), 0.into(), sphere, &mut ctx);
    assert!(
        UtmSelection::select_batch(&[], &mut ctx)
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        UtmSelection::select_batch(&[source.clone(), source.clone()], &mut ctx)
            .unwrap()
            .len(),
        2
    );
    assert!(matches!(
        UtmSelection::select_batch(&[source.clone(), invalid], &mut ctx),
        Err(Error::InvalidInput(_))
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
            UtmSelection::select_batch(std::slice::from_ref(&source), &mut limited),
            Err(Error::ResourceExhausted(_))
        ));
    }
    let signal = ctx.cancellation_signal();
    signal.store(true, std::sync::atomic::Ordering::Relaxed);
    assert!(matches!(
        UtmSelection::select(&source, &mut ctx),
        Err(Error::Cancelled)
    ));
    signal.store(false, std::sync::atomic::Ordering::Relaxed);
    assert_eq!(
        UtmSelection::select(&source, &mut ctx)
            .unwrap()
            .zone()
            .number(),
        31
    );
}
