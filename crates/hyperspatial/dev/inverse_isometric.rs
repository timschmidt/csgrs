//! Development-only exact inverse → geodetic → ECEF → local-frame workflow.

use crate::scalar_cases;
use hyperspatial::*;
use std::{cmp::Ordering, sync::Arc};

fn close(actual: &Real, expected: &Real) {
    let bound: Real = "1e-30".parse().unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&(actual - expected).abs(), &bound, PredicatePolicy::STRICT)
            .value(),
        Some(Ordering::Less)
    );
}

/// Bytes select tangent/axis families, height, meridian, axis scaling and warming.
pub fn exercise(data: [u8; 6]) -> Result<(u64, usize, &'static str, &'static str)> {
    let cases = scalar_cases::optimized_certificate_representatives();
    let first = &cases[usize::from(data[0]) % cases.len()];
    let second = &cases[usize::from(data[1]) % cases.len()];
    for case in [first, second] {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        if data[5] & 2 != 0 {
            let _ = case.value.to_f64_lossy();
        }
    }
    let sign = if data[2] & 1 == 0 { 1 } else { -1 };
    let tangent = (Real::from(sign) * &first.value / Real::from(100))?;
    let ratio = (&second.value / Real::from(1 + i32::from(data[4] % 4)))?;
    let square = &ratio * &ratio;
    let sine = (&tangent / (Real::one() + &tangent * &tangent).sqrt()?)?;
    let cosine = (Real::one() + &tangent * &tangent).sqrt()?.inverse()?;
    let eccentricity_squared = Real::one() - &square;
    let shape = hyperlimit::compare_reals(
        &eccentricity_squared,
        &Real::zero(),
        PredicatePolicy::STRICT,
    )
    .value()
    .expect("fixture shape is decidable");
    let spherical = tangent.clone().asinh()?;
    let psi = match shape {
        Ordering::Equal => spherical,
        Ordering::Greater => {
            let eccentricity = eccentricity_squared.sqrt()?;
            spherical - &eccentricity * (&eccentricity * &sine).atanh()?
        }
        Ordering::Less => {
            let eccentricity = (-eccentricity_squared).sqrt()?;
            spherical + &eccentricity * (&eccentricity * &sine).atan()?
        }
    };
    let latitude = psi.inverse_isometric_latitude(&ratio)?;
    close(&latitude, &tangent.atan()?);
    close(&latitude.clone().sin(), &sine);
    close(&latitude.clone().cos(), &cosine);
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(Real::one()),
        Length::metres(ratio),
        &mut context,
    )?);
    let longitude = (Real::from(i32::from(data[3] % 9) - 4) * Real::pi() / Real::from(4))?;
    let height = Real::from(i32::from(data[2]) - 128);
    let source = GeodeticPosition::new(
        Latitude::new(Angle::radians(latitude), &mut context)?,
        Longitude::unrolled(Angle::radians(longitude.clone())),
        Length::metres(height.clone()),
        ellipsoid,
        ReferenceFrame::fixed("fixture:inverse-isometric")?,
        InputProvenance::new("inverse equation fixture", InputAccuracy::ExactModel),
        &mut context,
    )?;
    let ecef = source.to_ecef(&mut context)?;
    let divisor = (&cosine * &cosine + &square * &sine * &sine).sqrt()?;
    let radius = (divisor.clone().inverse()? + &height) * &cosine;
    close(&ecef.coordinates().x, &(&radius * longitude.clone().cos()));
    close(&ecef.coordinates().y, &(radius * longitude.sin()));
    close(
        &ecef.coordinates().z,
        &(((square / divisor)? + height) * &sine),
    );
    let axes = if data[5] & 1 == 0 {
        LocalAxes::Enu
    } else {
        LocalAxes::Ned
    };
    let local =
        LocalFrame::new(&source, axes, &mut context)?.to_local_position(&ecef, &mut context)?;
    for coordinate in [
        &local.coordinates().x,
        &local.coordinates().y,
        &local.coordinates().z,
    ] {
        close(coordinate, &Real::zero());
    }
    assert_ne!(local.receipt().certainty(), Certainty::Approximate);
    Ok((
        context.steps_used(),
        local.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
