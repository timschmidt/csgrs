mod common;
use common::*;
use hyperlattice::{Point3, Vector3};
use hyperspatial::*;
use std::sync::Arc;

#[test]
fn ellipsoid_validation_and_defining_constants_are_exact() {
    let mut ctx = context();
    for (a, b) in [(0, 1), (1, 0), (-1, 2), (2, -1)] {
        assert!(matches!(
            Ellipsoid::from_axes(Length::metres(a.into()), Length::metres(b.into()), &mut ctx),
            Err(Error::InvalidInput(_))
        ));
    }
    let earth = Ellipsoid::wgs84(&mut ctx).unwrap();
    equal(earth.equatorial_radius().as_metres(), &r("6378137"));
    let f = ((earth.equatorial_radius().as_metres() - earth.polar_radius().as_metres())
        / earth.equatorial_radius().as_metres())
    .unwrap();
    equal(&(Real::one() / f).unwrap(), &r("298.257223563"));
    assert!(
        earth
            .polar_radius()
            .as_metres()
            .exact_rational_ref()
            .is_some()
    );
}

#[test]
fn equator_poles_and_negative_heights_have_independent_exact_answers() {
    let mut ctx = context();
    for (a, b) in [(5, 3), (3, 5), (4, 4)] {
        let ellipsoid = Arc::new(
            Ellipsoid::from_axes(Length::metres(a.into()), Length::metres(b.into()), &mut ctx)
                .unwrap(),
        );
        for (lat, lon, h, expected) in [
            (0, 0, 0, [a, 0, 0]),
            (0, 90, 0, [0, a, 0]),
            (0, 180, 0, [-a, 0, 0]),
            (0, -90, 2, [0, -a - 2, 0]),
            (90, 37, 0, [0, 0, b]),
            (-90, -121, -1, [0, 0, -b + 1]),
            (0, 0, -1, [a - 1, 0, 0]),
        ] {
            let source = position(
                lat.into(),
                lon.into(),
                h.into(),
                ellipsoid.clone(),
                &mut ctx,
            );
            let point = source.to_ecef(&mut ctx).unwrap();
            for (actual, expected) in [
                &point.coordinates().x,
                &point.coordinates().y,
                &point.coordinates().z,
            ]
            .into_iter()
            .zip(expected)
            {
                equal(actual, &Real::from(expected));
            }
            assert!(matches!(point.source(), EcefSource::Geodetic(_)));
        }
    }
}

#[test]
fn ellipsoid_normal_differs_from_radial_up() {
    let mut ctx = context();
    let ellipsoid = Arc::new(
        Ellipsoid::from_axes(Length::metres(2.into()), Length::metres(1.into()), &mut ctx).unwrap(),
    );
    let origin = position(45.into(), 0.into(), 0.into(), ellipsoid, &mut ctx);
    let ecef = origin.to_ecef(&mut ctx).unwrap();
    let sqrt5 = Real::from(5).sqrt().unwrap();
    equal(&ecef.coordinates().x, &(Real::from(4) / &sqrt5).unwrap());
    equal(&ecef.coordinates().z, &(Real::one() / sqrt5).unwrap());
    let local = LocalFrame::new(&origin, LocalAxes::Enu, &mut ctx).unwrap();
    let up = local
        .vector(
            Vector3::new([0.into(), 0.into(), 1.into()]),
            input(),
            &mut ctx,
        )
        .unwrap();
    let up_ecef = local.to_ecef_vector(&up, &mut ctx).unwrap();
    equal(&up_ecef.coordinates()[0], &up_ecef.coordinates()[2]);
    let cross = &ecef.coordinates().x * &up_ecef.coordinates()[2]
        - &ecef.coordinates().z * &up_ecef.coordinates()[0];
    assert_eq!(
        hyperlimit::classify_real_sign(&cross, PredicatePolicy::STRICT).value(),
        Some(hyperlimit::Sign::Positive)
    );
}

#[test]
fn enu_and_ned_have_expected_axes_and_point_vector_semantics() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::sphere(Length::metres(10.into()), &mut ctx).unwrap());
    let origin = position(0.into(), 0.into(), 0.into(), ellipsoid, &mut ctx);
    let absolute = EcefPosition::new(
        Point3::new(11.into(), 2.into(), 3.into()),
        reference(),
        input(),
        &mut ctx,
    )
    .unwrap();
    let vector = EcefVector::new(
        Vector3::new([1.into(), 2.into(), 3.into()]),
        reference(),
        input(),
        &mut ctx,
    )
    .unwrap();
    for (axes, expected) in [(LocalAxes::Enu, [2, 3, 1]), (LocalAxes::Ned, [3, 2, -1])] {
        let frame = LocalFrame::new(&origin, axes, &mut ctx).unwrap();
        assert_eq!(frame.axes(), axes);
        let point = frame.to_local_position(&absolute, &mut ctx).unwrap();
        let displacement = frame.to_local_vector(&vector, &mut ctx).unwrap();
        assert_eq!(
            point.source_ecef().unwrap().coordinates(),
            absolute.coordinates()
        );
        assert_eq!(
            displacement.source_ecef().unwrap().coordinates(),
            vector.coordinates()
        );
        assert_eq!(
            point.coordinates(),
            &Point3::new(expected[0].into(), expected[1].into(), expected[2].into())
        );
        assert_eq!(
            displacement.coordinates(),
            &Vector3::new(expected.map(Real::from))
        );
        assert_eq!(
            frame
                .to_ecef_position(&point, &mut ctx)
                .unwrap()
                .coordinates(),
            absolute.coordinates()
        );
        let restored = frame.to_ecef_vector(&displacement, &mut ctx).unwrap();
        assert_eq!(restored.coordinates(), vector.coordinates());
        assert!(restored.source_local().unwrap().frame().same_anchor(&frame));
        assert_eq!(
            frame
                .to_local_position(frame.origin(), &mut ctx)
                .unwrap()
                .coordinates(),
            &Point3::origin()
        );
    }
}

#[test]
fn pole_meridian_is_retained_and_controls_the_basis() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::sphere(Length::metres(10.into()), &mut ctx).unwrap());
    let north = position(90.into(), 0.into(), 0.into(), ellipsoid.clone(), &mut ctx);
    let east = position(90.into(), 90.into(), 0.into(), ellipsoid, &mut ctx);
    let north_frame = LocalFrame::new(&north, LocalAxes::Enu, &mut ctx).unwrap();
    let east_frame = LocalFrame::new(&east, LocalAxes::Enu, &mut ctx).unwrap();
    assert_eq!(
        north_frame.origin().coordinates(),
        east_frame.origin().coordinates()
    );
    let x = EcefVector::new(
        Vector3::new([1.into(), 0.into(), 0.into()]),
        reference(),
        input(),
        &mut ctx,
    )
    .unwrap();
    assert_eq!(
        north_frame
            .to_local_vector(&x, &mut ctx)
            .unwrap()
            .coordinates(),
        &Vector3::new([0.into(), (-1).into(), 0.into()])
    );
    assert_eq!(
        east_frame
            .to_local_vector(&x, &mut ctx)
            .unwrap()
            .coordinates(),
        &Vector3::new([(-1).into(), 0.into(), 0.into()])
    );
}

#[test]
fn distinct_anchors_frames_and_epochs_are_rejected() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let origin = position(0.into(), 0.into(), 0.into(), ellipsoid, &mut ctx);
    let frame = LocalFrame::new(&origin, LocalAxes::Enu, &mut ctx).unwrap();
    let other_anchor = LocalFrame::new(&origin, LocalAxes::Enu, &mut ctx).unwrap();
    let local = frame.position(Point3::origin(), input(), &mut ctx).unwrap();
    assert!(local.source_ecef().is_none());
    let vector = frame.vector(Vector3::zero(), input(), &mut ctx).unwrap();
    assert!(vector.source_ecef().is_none());
    assert!(matches!(
        other_anchor.to_ecef_vector(&vector, &mut ctx),
        Err(Error::LocalFrameMismatch)
    ));
    assert!(frame.same_anchor(&frame.clone()));
    assert!(!frame.same_anchor(&other_anchor));
    assert!(matches!(
        other_anchor.to_ecef_position(&local, &mut ctx),
        Err(Error::LocalFrameMismatch)
    ));
    for reference in [
        ReferenceFrame::fixed("different realization").unwrap(),
        ReferenceFrame::at_julian_year(reference().name(), 2025.into()).unwrap(),
    ] {
        let point = EcefPosition::new(Point3::origin(), reference, input(), &mut ctx).unwrap();
        assert!(matches!(
            frame.to_local_position(&point, &mut ctx),
            Err(Error::ReferenceMismatch)
        ));
    }
    assert!(matches!(
        ReferenceFrame::fixed(" "),
        Err(Error::InvalidInput(_))
    ));
}

#[test]
fn coordinate_epochs_compare_as_values_and_the_decision_is_retained() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let origin = GeodeticPosition::new(
        Latitude::new(Angle::degrees(0.into()), &mut ctx).unwrap(),
        Longitude::unrolled(Angle::degrees(0.into())),
        Length::metres(0.into()),
        ellipsoid,
        ReferenceFrame::at_julian_year("epoch fixture", r("2025.0")).unwrap(),
        input(),
        &mut ctx,
    )
    .unwrap();
    assert_eq!(origin.reference().epoch_julian_year(), Some(&r("2025")));
    equal(
        origin.ellipsoid().equatorial_radius().as_metres(),
        &r("6378137"),
    );
    let frame = LocalFrame::new(&origin, LocalAxes::Enu, &mut ctx).unwrap();
    for (epoch, succeeds) in [
        (Real::from(2024) + Real::one(), true),
        (r("2025.00000000000000000000001"), false),
    ] {
        let point = EcefPosition::new(
            Point3::origin(),
            ReferenceFrame::at_julian_year("epoch fixture", epoch).unwrap(),
            input(),
            &mut ctx,
        )
        .unwrap();
        let result = frame.to_local_position(&point, &mut ctx);
        if succeeds {
            let result = result.unwrap();
            assert_eq!(result.receipt().decisions().len(), 1);
            assert_eq!(
                result.receipt().decisions()[0].claim(),
                "coordinate epochs agree"
            );
            let decision = &result.receipt().decisions()[0];
            equal(decision.left(), &r("2025"));
            equal(decision.right(), &r("2025"));
            assert_eq!(decision.outcome().value(), Some(std::cmp::Ordering::Equal));
        } else {
            assert!(matches!(result, Err(Error::ReferenceMismatch)));
        }
    }
}

#[test]
fn published_proj_grs80_cartesian_and_topocentric_fixtures() {
    // Primary fixtures and their printed precision:
    // https://proj.org/en/stable/operations/conversions/cart.html
    // https://proj.org/en/stable/operations/conversions/topocentric.html
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::grs80(&mut ctx).unwrap());
    let point = position(
        r("45.3935192042"),
        r("17.7562015132"),
        r("133.12"),
        ellipsoid.clone(),
        &mut ctx,
    )
    .to_ecef(&mut ctx)
    .unwrap();
    for (actual, expected) in [
        &point.coordinates().x,
        &point.coordinates().y,
        &point.coordinates().z,
    ]
    .into_iter()
    .zip(["4272922.1553", "1368283.0597", "4518261.3501"])
    {
        rounded(actual, expected, "0.00005");
    }
    let origin = position(55.into(), 5.into(), 200.into(), ellipsoid.clone(), &mut ctx);
    let frame = LocalFrame::new(&origin, LocalAxes::Enu, &mut ctx).unwrap();
    let point = position(
        r("53.80939444"),
        r("2.12955"),
        73.into(),
        ellipsoid,
        &mut ctx,
    )
    .to_ecef(&mut ctx)
    .unwrap();
    let local = frame.to_local_position(&point, &mut ctx).unwrap();
    for (actual, expected) in [
        &local.coordinates().x,
        &local.coordinates().y,
        &local.coordinates().z,
    ]
    .into_iter()
    .zip(["-189013.869", "-128642.040", "-4220.171"])
    {
        rounded(actual, expected, "0.0005");
    }
}
