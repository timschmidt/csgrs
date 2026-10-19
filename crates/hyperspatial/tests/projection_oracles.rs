mod common;
use common::*;
use hyperlattice::Point3;
use hyperspatial::*;
use proj4rs::{proj::Proj, transform::transform};

#[test]
fn pinned_proj4rs_spherical_mercator_agrees_on_a_qualified_grid() {
    let from = Proj::from_proj_string("+proj=longlat +a=6378137 +b=6378137").unwrap();
    let to = Proj::from_proj_string("+proj=merc +a=6378137 +b=6378137 +units=m").unwrap();
    let mut ctx = context();
    let projection = SphericalMercator::web_mercator(
        reference(),
        LongitudePolicy::RejectOutsidePrincipal,
        &mut ctx,
    )
    .unwrap();
    let qualified = InputProvenance::new(
        "proj4rs 0.2.1 comparison",
        InputAccuracy::Qualified("binary64 fixture, spherical equations".into()),
    );
    for latitude in [-89.0_f64, -80.0, -45.0, -12.5, 0.0, 12.5, 45.0, 80.0, 89.0] {
        for longitude in [-179.0_f64, -120.0, -60.0, 0.0, 60.0, 120.0, 179.0] {
            let input = (longitude.to_radians(), latitude.to_radians(), -123.0);
            let mut oracle = input;
            transform(&from, &to, &mut oracle).unwrap();
            let source = GeodeticPosition::new(
                Latitude::new(Angle::radians(Real::try_from(input.1).unwrap()), &mut ctx).unwrap(),
                Longitude::unrolled(Angle::radians(Real::try_from(input.0).unwrap())),
                Length::metres(Real::try_from(input.2).unwrap()),
                projection.ellipsoid().clone(),
                reference(),
                qualified.clone(),
                &mut ctx,
            )
            .unwrap();
            let native = projection.project(&source, &mut ctx).unwrap();
            for (actual, expected) in [
                (&native.coordinates().x, oracle.0),
                (&native.coordinates().y, oracle.1),
            ] {
                assert!(
                    (actual.to_f64_lossy().unwrap() - expected).abs() < 3e-7,
                    "{latitude} {longitude}"
                );
            }
            equal(&native.coordinates().z, &Real::try_from(oracle.2).unwrap());
            let raw = projection
                .position(
                    Point3::new(
                        Real::try_from(oracle.0).unwrap(),
                        Real::try_from(oracle.1).unwrap(),
                        Real::try_from(oracle.2).unwrap(),
                    ),
                    qualified.clone(),
                    &mut ctx,
                )
                .unwrap();
            let native_inverse = projection.unproject(&raw, &mut ctx).unwrap();
            transform(&to, &from, &mut oracle).unwrap();
            for (actual, expected) in [
                (native_inverse.longitude().angle().as_radians(), input.0),
                (native_inverse.latitude().angle().as_radians(), input.1),
            ] {
                let difference = (actual - Real::try_from(expected).unwrap()).abs();
                assert_eq!(
                    hyperlimit::compare_reals(
                        &difference,
                        &r("0.00000000000003"),
                        PredicatePolicy::STRICT
                    )
                    .value(),
                    Some(std::cmp::Ordering::Less)
                );
            }
            assert!((oracle.0 - input.0).abs() < 3e-14);
            assert!((oracle.1 - input.1).abs() < 3e-14);
            assert_ne!(native_inverse.receipt().certainty(), Certainty::Approximate);
            let GeodeticSource::SphericalMercator(retained) = native_inverse.source() else {
                panic!("lost projected source")
            };
            assert_eq!(retained.receipt().input().unwrap(), &qualified);
        }
    }
}

#[test]
fn pinned_proj4rs_ellipsoidal_mercator_agrees_on_a_qualified_grid() {
    let from = Proj::from_proj_string("+proj=longlat +ellps=WGS84").unwrap();
    let to = Proj::from_proj_string("+proj=merc +ellps=WGS84 +units=m").unwrap();
    let mut ctx = context();
    let projection = EllipsoidalMercator::new(
        std::sync::Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap()),
        reference(),
        MercatorParameters::greenwich(LongitudePolicy::RejectOutsidePrincipal),
        &mut ctx,
    )
    .unwrap();
    let qualified = InputProvenance::new(
        "proj4rs 0.2.1 comparison",
        InputAccuracy::Qualified("binary64 fixture, WGS84 ellipsoidal equations".into()),
    );
    for latitude in [-89.0_f64, -80.0, -45.0, -12.5, 0.0, 12.5, 45.0, 80.0, 89.0] {
        for longitude in [-179.0_f64, -120.0, -60.0, 0.0, 60.0, 120.0, 179.0] {
            let input = (longitude.to_radians(), latitude.to_radians(), -123.0);
            let mut oracle = input;
            transform(&from, &to, &mut oracle).unwrap();
            let source = GeodeticPosition::new(
                Latitude::new(Angle::radians(Real::try_from(input.1).unwrap()), &mut ctx).unwrap(),
                Longitude::unrolled(Angle::radians(Real::try_from(input.0).unwrap())),
                Length::metres(Real::try_from(input.2).unwrap()),
                projection.ellipsoid().clone(),
                reference(),
                qualified.clone(),
                &mut ctx,
            )
            .unwrap();
            let native = projection.project(&source, &mut ctx).unwrap();
            for (actual, expected) in [
                (&native.coordinates().x, oracle.0),
                (&native.coordinates().y, oracle.1),
            ] {
                assert!(
                    (actual.to_f64_lossy().unwrap() - expected).abs() < 3e-7,
                    "{latitude} {longitude}"
                );
            }
            equal(&native.coordinates().z, &Real::try_from(oracle.2).unwrap());
            let raw = projection
                .position(
                    Point3::new(
                        Real::try_from(oracle.0).unwrap(),
                        Real::try_from(oracle.1).unwrap(),
                        Real::try_from(oracle.2).unwrap(),
                    ),
                    qualified.clone(),
                    &mut ctx,
                )
                .unwrap();
            let native_inverse = projection.unproject(&raw, &mut ctx).unwrap();
            transform(&to, &from, &mut oracle).unwrap();
            for (actual, expected) in [
                (native_inverse.longitude().angle().as_radians(), input.0),
                (native_inverse.latitude().angle().as_radians(), input.1),
            ] {
                let difference = (actual - Real::try_from(expected).unwrap()).abs();
                assert_eq!(
                    hyperlimit::compare_reals(
                        &difference,
                        &r("0.00000000000003"),
                        PredicatePolicy::STRICT
                    )
                    .value(),
                    Some(std::cmp::Ordering::Less)
                );
            }
            assert!((oracle.0 - input.0).abs() < 3e-14);
            assert!((oracle.1 - input.1).abs() < 3e-14);
            assert_ne!(native_inverse.receipt().certainty(), Certainty::Approximate);
            let GeodeticSource::EllipsoidalMercator(retained) = native_inverse.source() else {
                panic!("lost projected source")
            };
            assert_eq!(retained.receipt().input().unwrap(), &qualified);
        }
    }
}
