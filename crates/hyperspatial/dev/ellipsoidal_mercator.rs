//! Development-only ellipsoidal projection with independent tangent-parameter fixtures.

use hyperspatial::*;
use std::{cmp::Ordering, sync::Arc};

use crate::scalar_cases;

fn close(actual: &Real, expected: &Real) {
    let bound: Real = "0.000000000000000000000000000001".parse().unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&(actual - expected).abs(), &bound, PredicatePolicy::STRICT)
            .value(),
        Some(Ordering::Less)
    );
}

/// Six bytes select latitude/longitude recipes, scale, turns, policy and shape.
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
    let (a, b) = [(5, 3), (3, 5), (5, 5)][usize::from(data[5]) % 3];
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(a.into()),
        Length::metres(b.into()),
        &mut context,
    )?);
    let policy = [
        LongitudePolicy::Unrolled,
        LongitudePolicy::RejectOutsidePrincipal,
        LongitudePolicy::WrapSigned,
    ][usize::from(data[4]) % 3];
    let mut parameters = MercatorParameters::greenwich(policy);
    parameters.scale = &second.value * Real::from(1 + i32::from(data[2] % 4));
    parameters.false_easting = Length::metres(first.value.clone());
    parameters.false_northing = Length::metres(-&second.value);
    parameters.central_meridian = Longitude::unrolled(Angle::degrees(Real::from(
        (i32::from(data[3] % 17) - 8) * 45,
    )));
    let scale_radius = Real::from(a) * &parameters.scale;
    let projection = EllipsoidalMercator::new(
        ellipsoid,
        ReferenceFrame::fixed("fuzz:projection")?,
        parameters,
        &mut context,
    )?;
    let sign = Real::from(if data[2] & 8 == 0 { 1 } else { -1 });
    let tangent = (sign * &first.value / Real::from(100))?;
    let latitude = tangent.clone().atan()?;
    let base_longitude = (&second.value / Real::from(100))?;
    let turns = if policy == LongitudePolicy::RejectOutsidePrincipal {
        0
    } else {
        i32::from(data[3] % 5) - 2
    };
    let delta = &base_longitude + Real::from(2 * turns) * Real::pi();
    let longitude = projection
        .parameters()
        .central_meridian
        .angle()
        .as_radians()
        + &delta;
    let height = Real::from(i32::from(data[2]) - 128);
    let input = InputProvenance::new(
        "projection representation corpus",
        InputAccuracy::ExactModel,
    );
    let source = GeodeticPosition::new(
        Latitude::new(Angle::radians(latitude.clone()), &mut context)?,
        Longitude::unrolled(Angle::radians(longitude)),
        Length::metres(height.clone()),
        projection.ellipsoid().clone(),
        projection.reference().clone(),
        input.clone(),
        &mut context,
    )?;
    let projected = projection.project(&source, &mut context)?;
    let canonical_delta = if policy == LongitudePolicy::Unrolled {
        delta
    } else {
        base_longitude
    };
    close(
        &projected.coordinates().x,
        &(projection.parameters().false_easting.as_metres() + &scale_radius * &canonical_delta),
    );
    let sine = (&tangent / (Real::one() + &tangent * &tangent).sqrt()?)?;
    let spherical = tangent.asinh()?;
    let psi = match a.cmp(&b) {
        Ordering::Greater => {
            let e: Real = "0.8".parse().unwrap();
            spherical - &e * (&e * &sine).atanh()?
        }
        Ordering::Less => {
            let c = (Real::from(4) / Real::from(3))?;
            spherical + &c * (&c * &sine).atan()?
        }
        Ordering::Equal => spherical,
    };
    close(
        &projected.coordinates().y,
        &(projection.parameters().false_northing.as_metres() + &scale_radius * psi),
    );
    // Fresh ingress requires evaluation of the inverse, not source replay.
    let fresh =
        projection.position(projected.coordinates().clone(), input.clone(), &mut context)?;
    let inverse = projection.unproject(&fresh, &mut context)?;
    close(inverse.latitude().angle().as_radians(), &latitude);
    close(
        inverse.longitude().angle().as_radians(),
        &(projection
            .parameters()
            .central_meridian
            .angle()
            .as_radians()
            + canonical_delta),
    );
    close(inverse.height().as_metres(), &height);
    let source = GeodeticPosition::new(
        inverse.latitude().clone(),
        inverse.longitude().clone(),
        inverse.height().clone(),
        inverse.ellipsoid().clone(),
        inverse.reference().clone(),
        input,
        &mut context,
    )?;
    let returned = projection.project(&source, &mut context)?;
    close(&returned.coordinates().x, &projected.coordinates().x);
    close(&returned.coordinates().y, &projected.coordinates().y);
    let ecef = inverse.to_ecef(&mut context)?;
    let frame = LocalFrame::new(&inverse, LocalAxes::Enu, &mut context)?;
    let local = frame.to_local_position(&ecef, &mut context)?;
    close(&local.coordinates().x, &Real::zero());
    close(&local.coordinates().y, &Real::zero());
    close(&local.coordinates().z, &Real::zero());
    assert_ne!(local.receipt().certainty(), Certainty::Approximate);
    Ok((
        context.steps_used(),
        local.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
