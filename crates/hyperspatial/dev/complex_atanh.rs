//! Native principal atanh components consumed by geodetic and anchored local operations.
use crate::scalar_cases;
use hyperlattice::Complex;
use hyperspatial::*;
use std::sync::Arc;

fn has_input(receipt: &Receipt, expected: &InputProvenance) -> bool {
    receipt.input() == Some(expected) || receipt.parents().iter().any(|p| has_input(p, expected))
}

/// Two scalar families, near-cut scale/bank, argument regime and display warming.
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
    let small_x = (&first.value / (Real::from(8) * (&first.value + Real::one())))?;
    let small_y = (&second.value / (Real::from(8) * (&second.value + Real::one())))?;
    let mut argument = match data[4] % 4 {
        0 => Complex::new(small_x, small_y),
        1 => Complex::new(small_x, small_y).reciprocal()?,
        2 if data[2] & 1 == 0 => Complex::from(small_x),
        2 => Complex::new(Real::zero(), small_y),
        _ => {
            let scale = Real::from(10).powi((-(i32::from(data[2]) + 1)).into())?;
            let side = if data[3] & 1 == 0 {
                Real::one()
            } else {
                -Real::one()
            };
            Complex::new(Real::one() + side * small_x * &scale, small_y * scale)
        }
    };
    if data[5] & 1 != 0 {
        argument = -argument;
    }
    let atanh = argument.atanh()?;
    for value in [&atanh.re, &atanh.im] {
        let [lo, hi] = value
            .certified_rational_interval(-128)
            .expect("finite atanh component");
        assert!(lo <= hi);
    }
    let axis = &atanh.re * &atanh.re + Real::from(2);
    let expected_x = &axis + &atanh.im;
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(axis),
        Length::metres(Real::one()),
        &mut context,
    )?);
    let provenance = InputProvenance::new(
        "atanh geometry",
        InputAccuracy::Qualified("external observation remains qualified".into()),
    );
    let position = GeodeticPosition::new(
        Latitude::new(Angle::radians(Real::zero()), &mut context)?,
        Longitude::unrolled(Angle::radians(Real::zero())),
        Length::metres(atanh.im),
        ellipsoid,
        ReferenceFrame::fixed("atanh consumer")?,
        provenance.clone(),
        &mut context,
    )?;
    let ecef = position.to_ecef(&mut context)?;
    assert_eq!(
        (&ecef.coordinates().x - &expected_x).exact_rational_normal_form(),
        Some(hyperreal::Rational::zero()),
        "ECEF atanh composition, {data:?}"
    );
    let axes = if data[5] & 2 == 0 {
        LocalAxes::Enu
    } else {
        LocalAxes::Ned
    };
    let frame = LocalFrame::new(&position, axes, &mut context)?;
    let local = frame.to_local_position(&ecef, &mut context)?;
    for value in [
        &local.coordinates().x,
        &local.coordinates().y,
        &local.coordinates().z,
    ] {
        assert_eq!(
            value.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero()),
            "local origin, {data:?}"
        );
    }
    let restored = frame.to_ecef_position(&local, &mut context)?;
    assert_eq!(
        (&restored.coordinates().x - &expected_x).exact_rational_normal_form(),
        Some(hyperreal::Rational::zero()),
        "restored ECEF, {data:?}"
    );
    for receipt in [ecef.receipt(), local.receipt(), restored.receipt()] {
        assert_ne!(receipt.certainty(), Certainty::Approximate);
        assert!(has_input(receipt, &provenance));
    }
    Ok((
        context.steps_used(),
        restored.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
