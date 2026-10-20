//! Development-only spherical chart and angular UTM workflow, shared by gates.
use crate::scalar_cases;
use hyperlattice::Point3;
use hyperspatial::*;
use std::cmp::Ordering;

fn close(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(
            &(actual - expected).abs(),
            &"1e-30".parse().unwrap(),
            PredicatePolicy::STRICT
        )
        .value(),
        Some(Ordering::Less)
    );
}

/// Six bytes select scalar families, height, signs/origin, chart regime and warming.
pub fn exercise(data: [u8; 6]) -> Result<(u64, usize, &'static str, &'static str)> {
    let cases = scalar_cases::optimized_certificate_representatives();
    let first = &cases[usize::from(data[0]) % cases.len()];
    let second = &cases[usize::from(data[1]) % cases.len()];
    for case in [first, second] {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        if data[5] & 4 != 0 {
            let _ = case.value.to_f64_lossy();
        }
    }
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let mut parameters =
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::WrapToChart);
    parameters.scale = second.value.clone();
    parameters.false_easting = Length::metres(first.value.clone());
    parameters.false_northing = Length::metres(-&second.value);
    parameters.latitude_of_origin = Angle::degrees(Real::from(i32::from(data[3] % 31) - 15));
    let scale_radius = Real::from(5) * &parameters.scale;
    let projection = SphericalTransverseMercator::new(
        Length::metres(5.into()),
        ReferenceFrame::fixed("fuzz:transverse")?,
        parameters,
        &mut context,
    )?;
    let sign = Real::from(if data[3] & 32 == 0 { 1 } else { -1 });
    let height = Real::from(i32::from(data[2]) - 128);
    let input = InputProvenance::new(
        "transverse representation corpus",
        InputAccuracy::ExactModel,
    );
    let half_pi = (Real::pi() / Real::from(2))?;
    let mode = data[4] % 4;
    let (eta, xi, expected_phi, expected_delta) = if mode == 0 || mode == 3 {
        let tangent = (&first.value / Real::from(100))?;
        let phi = tangent.clone().atan()?;
        let delta = if mode == 3 {
            &sign * &half_pi
        } else {
            &sign * (&second.value / Real::from(100))?.atan()?
        };
        let cosine = delta.clone().cos();
        let eta = (delta.clone().sin() / Real::hypot2(&tangent, &cosine)?)?.asinh()?;
        let xi = if mode == 3 {
            half_pi.clone()
        } else {
            (tangent / cosine)?.atan()?
        };
        let source = GeodeticPosition::new(
            Latitude::new(Angle::radians(phi.clone()), &mut context)?,
            Longitude::unrolled(Angle::radians(delta.clone())),
            Length::metres(height.clone()),
            projection.ellipsoid().clone(),
            projection.reference().clone(),
            input.clone(),
            &mut context,
        )?;
        let point = projection.project(&source, &mut context)?;
        close(
            &point.coordinates().x,
            &(projection.parameters().false_easting.as_metres() + &scale_radius * &eta),
        );
        close(
            &point.coordinates().y,
            &(projection.parameters().false_northing.as_metres()
                + &scale_radius * (&xi - projection.parameters().latitude_of_origin.as_radians())),
        );
        (eta, xi, Some(phi), Some(delta))
    } else {
        let eta = if mode == 2 {
            &sign * "1e320".parse::<Real>().unwrap() * &first.value
        } else {
            &sign * (&first.value / Real::from(10))?
        };
        let xi = (&second.value / Real::from(100))?;
        (eta, xi, None, None)
    };
    let coordinates = Point3::new(
        projection.parameters().false_easting.as_metres() + &scale_radius * &eta,
        projection.parameters().false_northing.as_metres()
            + &scale_radius * (&xi - projection.parameters().latitude_of_origin.as_radians()),
        height.clone(),
    );
    let raw = projection.position(coordinates.clone(), input.clone(), &mut context)?;
    let inverse = projection.unproject(&raw, PoleMeridian::Reject, &mut context)?;
    if let (Some(phi), Some(delta)) = (expected_phi, expected_delta) {
        close(inverse.latitude().angle().as_radians(), &phi);
        close(inverse.longitude().angle().as_radians(), &delta);
    }
    // Fresh ingress preserves the joint inverse relation across public clones.
    let cloned = GeodeticPosition::new(
        inverse.latitude().clone(),
        inverse.longitude().clone(),
        inverse.height().clone(),
        inverse.ellipsoid().clone(),
        inverse.reference().clone(),
        input.clone(),
        &mut context,
    )?;
    let returned = projection.project(&cloned, &mut context)?;
    close(&returned.coordinates().x, &coordinates.x);
    close(&returned.coordinates().y, &coordinates.y);
    close(&returned.coordinates().z, &height);
    let ecef = cloned.to_ecef(&mut context)?;
    let radius = Real::from(5) + &height;
    if mode == 2 {
        close(inverse.latitude().angle().as_radians(), &Real::zero());
        close(inverse.longitude().angle().as_radians(), &(&sign * half_pi));
        close(&ecef.coordinates().x, &Real::zero());
        close(&ecef.coordinates().y, &(radius * sign));
        close(&ecef.coordinates().z, &Real::zero());
    } else {
        // Independent Cartesian chart identity, not the inverse's tan/atan path.
        let inverse_cosh = eta.clone().cosh()?.inverse()?;
        close(
            &ecef.coordinates().x,
            &(&radius * xi.clone().cos() * &inverse_cosh),
        );
        close(&ecef.coordinates().y, &(&radius * eta.tanh()?));
        close(&ecef.coordinates().z, &(radius * xi.sin() * inverse_cosh));
    }
    let frame = LocalFrame::new(&inverse, LocalAxes::Enu, &mut context)?;
    let local = frame.to_local_position(&ecef, &mut context)?;
    for value in [
        &local.coordinates().x,
        &local.coordinates().y,
        &local.coordinates().z,
    ] {
        close(value, &Real::zero());
    }
    assert_ne!(local.receipt().certainty(), Certainty::Approximate);
    // Exercise exact selection for the same scalar families, away from chart
    // limits where finite predicate precision may intentionally return Unknown.
    let angular = GeodeticPosition::new(
        Latitude::new(
            Angle::degrees((&first.value / Real::from(100))?),
            &mut context,
        )?,
        Longitude::unrolled(Angle::degrees(
            Real::from(3) + (&second.value / Real::from(100))?,
        )),
        Length::metres(height),
        projection.ellipsoid().clone(),
        projection.reference().clone(),
        input,
        &mut context,
    )?;
    let selected = UtmSelection::select(&angular, &mut context)?;
    assert_eq!(selected.zone().number(), 31);
    assert_eq!(selected.hemisphere(), Hemisphere::North);
    assert_ne!(
        selected.parameters()?.receipt().unwrap().certainty(),
        Certainty::Approximate
    );
    Ok((
        context.steps_used(),
        local.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
