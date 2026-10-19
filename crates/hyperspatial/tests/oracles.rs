//! Pinned binary64 oracle comparisons at explicit development-only boundaries.
mod common;
use common::*;
use hyperlattice::Point3;
use hyperspatial::*;
use std::sync::Arc;

fn qualification() -> InputProvenance {
    InputProvenance::new(
        "map_3d 0.1.7 represented binary64 fixture",
        InputAccuracy::Qualified(Arc::from(
            "rounded parameters/coordinates; no physical accuracy claim",
        )),
    )
}

fn imported(x: f64, y: f64, z: f64, ctx: &mut Context) -> Result<EcefPosition> {
    EcefPosition::new(
        Point3::new(Real::try_from(x)?, Real::try_from(y)?, Real::try_from(z)?),
        reference(),
        qualification(),
        ctx,
    )
}

fn geodetic(
    lat: f64,
    lon: f64,
    h: f64,
    ellipsoid: Arc<Ellipsoid>,
    ctx: &mut Context,
) -> GeodeticPosition {
    GeodeticPosition::new(
        Latitude::new(Angle::radians(Real::try_from(lat).unwrap()), ctx).unwrap(),
        Longitude::unrolled(Angle::radians(Real::try_from(lon).unwrap())),
        Length::metres(Real::try_from(h).unwrap()),
        ellipsoid,
        reference(),
        qualification(),
        ctx,
    )
    .unwrap()
}

fn agrees(actual: f64, expected: f64, bound: f64) {
    assert!(actual.is_finite() && expected.is_finite());
    assert!(
        (actual - expected).abs() <= bound,
        "{actual} differs from {expected} by {}, bound {bound}",
        (actual - expected).abs()
    );
}

#[test]
fn map3d_175_case_corpus_validates_native_workflows_and_records_oracle_limits() {
    // Primary API and algorithm reference:
    // https://docs.rs/map_3d/0.1.7/map_3d/
    // Match the represented axes, rather than pretending a rounded inverse
    // flattening is the native exact WGS84 definition. Inputs are dyadic here.
    let mut ctx = context();
    let oracle = map_3d::Ellipsoid::WGS84;
    let (a, b, _, _) = oracle.parameters();
    let ellipsoid = Arc::new(
        Ellipsoid::from_axes(
            Length::metres(Real::try_from(a).unwrap()),
            Length::metres(Real::try_from(b).unwrap()),
            &mut ctx,
        )
        .unwrap(),
    );
    let origin = geodetic(0.3, 0.7, 100.0, ellipsoid.clone(), &mut ctx);
    let frame = LocalFrame::new(&origin, LocalAxes::Enu, &mut ctx).unwrap();
    for lat in [-89.0_f64, -55.0, -12.5, 0.0, 23.4, 55.0, 89.0] {
        for lon in [-179.0_f64, -73.0, 0.0, 123.0, 179.0] {
            for h in [-1000.0, -10.0, 0.0, 100.0, 1_000_000.0] {
                let (lat, lon) = (lat.to_radians(), lon.to_radians());
                let native = geodetic(lat, lon, h, ellipsoid.clone(), &mut ctx)
                    .to_ecef(&mut ctx)
                    .unwrap();
                let (x, y, z) = map_3d::geodetic2ecef(lat, lon, h, oracle);
                for (actual, expected) in native
                    .coordinates()
                    .to_f64_array_lossy()
                    .unwrap()
                    .into_iter()
                    .zip([x, y, z])
                {
                    agrees(actual, expected, 3e-8);
                }
                let point = imported(x, y, z, &mut ctx).unwrap();
                let result = point
                    .to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx)
                    .unwrap();
                let (latitude, longitude, height) = map_3d::ecef2geodetic(x, y, z, oracle);
                let native_latitude = result
                    .latitude()
                    .angle()
                    .as_radians()
                    .to_f64_lossy()
                    .unwrap();
                agrees(native_latitude, lat, 5e-13);
                if h == 1_000_000.0 && lat != 0.0 {
                    // Retain the known finite-correction oracle discrepancy.
                    // Native acceptance remains at the original strict bound.
                    assert!((latitude - native_latitude).abs() > 5e-13);
                } else {
                    agrees(native_latitude, latitude, 5e-13);
                }
                agrees(
                    result
                        .longitude()
                        .angle()
                        .as_radians()
                        .to_f64_lossy()
                        .unwrap(),
                    longitude,
                    5e-13,
                );
                agrees(
                    result.height().as_metres().to_f64_lossy().unwrap(),
                    height,
                    5e-7,
                );
                agrees(result.height().as_metres().to_f64_lossy().unwrap(), h, 5e-7);
                // Independently evaluate the native forward equations; the
                // explicit input below cannot reuse an inverse-source identity.
                let evaluated = GeodeticPosition::new(
                    result.latitude().clone(),
                    result.longitude().clone(),
                    result.height().clone(),
                    ellipsoid.clone(),
                    reference(),
                    qualification(),
                    &mut ctx,
                )
                .unwrap()
                .to_ecef(&mut ctx)
                .unwrap();
                let bound = r("0.0000000000000000000000001");
                for (actual, expected) in [
                    &evaluated.coordinates().x,
                    &evaluated.coordinates().y,
                    &evaluated.coordinates().z,
                ]
                .into_iter()
                .zip([x, y, z])
                {
                    let residual = (actual - Real::try_from(expected).unwrap()).abs();
                    assert_eq!(
                        hyperlimit::compare_reals(&residual, &bound, PredicatePolicy::STRICT)
                            .value(),
                        Some(std::cmp::Ordering::Less)
                    );
                }
                let local = frame.to_local_position(&point, &mut ctx).unwrap();
                let (e, n, u) = map_3d::ecef2enu(x, y, z, 0.3, 0.7, 100.0, oracle);
                for (actual, expected) in local
                    .coordinates()
                    .to_f64_array_lossy()
                    .unwrap()
                    .into_iter()
                    .zip([e, n, u])
                {
                    agrees(actual, expected, 3e-8);
                }
                let GeodeticSource::EcefClosest { position, .. } = result.source() else {
                    panic!("lost oracle ingress")
                };
                assert_eq!(position.receipt().input().unwrap(), &qualification());
            }
        }
    }
}

#[test]
fn map3d_high_orbit_latitude_discrepancy_is_retained_as_an_oracle_regression() {
    let lat = 55.0_f64.to_radians();
    let (x, y, z) = map_3d::geodetic2ecef(lat, 0.0, 1_000_000.0, map_3d::Ellipsoid::WGS84);
    let (actual, _, _) = map_3d::ecef2geodetic(x, y, z, map_3d::Ellipsoid::WGS84);
    assert!((actual - lat).abs() > 1e-9);
    // The full matrix above retains this same case with native 5e-13 radian
    // and certified 1e-25 metre forward-residual acceptance.
}

#[test]
fn nonfinite_oracle_values_cannot_enter_native_coordinates() {
    let mut ctx = context();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            imported(value, 0.0, 0.0, &mut ctx),
            Err(Error::Scalar(_))
        ));
        assert!(matches!(
            imported(0.0, value, 0.0, &mut ctx),
            Err(Error::Scalar(_))
        ));
        assert!(matches!(
            imported(0.0, 0.0, value, &mut ctx),
            Err(Error::Scalar(_))
        ));
    }
    let centre = map_3d::ecef2geodetic(0.0, 0.0, 0.0, map_3d::Ellipsoid::WGS84);
    assert!(Real::try_from(centre.0).is_err());
    let surface = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let native = imported(0.0, 0.0, 0.0, &mut ctx).unwrap();
    assert!(matches!(
        native.to_geodetic_closest(surface, PoleMeridian::Reject, &mut ctx),
        Err(Error::AmbiguousInverse(_))
    ));
}
