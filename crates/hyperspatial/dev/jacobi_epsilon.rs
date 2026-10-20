//! Native epsilon components consumed by geodetic and anchored local operations.
use crate::scalar_cases;
use hyperlattice::Complex;
use hyperspatial::*;
use std::sync::Arc;

fn has_input(receipt: &Receipt, expected: &InputProvenance) -> bool {
    receipt.input() == Some(expected) || receipt.parents().iter().any(|p| has_input(p, expected))
}

/// Two scalar families, period count/parity, parameter regime and display warming.
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
    let small = (&first.value / (Real::from(4) * (&first.value + &second.value)))?;
    let m = match data[4] % 4 {
        0 => small.clone(),
        1 => -small.clone(),
        2 => Real::zero(),
        _ => "0.5".parse().unwrap(),
    };
    let mut x = first.value.clone();
    let mut y = (&second.value / (Real::from(8) * (&second.value + Real::one())))?;
    if data[4] % 4 == 3 {
        let k = Real::elliptic_k(&m)?;
        x = &small + Real::from(2 * u16::from(data[2]) + u16::from(data[3] & 1)) * &k;
        y = k + y;
    }
    if data[5] & 1 != 0 {
        x = -x;
        y = -y;
    }
    let real_epsilon = x.clone().jacobi_epsilon(&m)?;
    let recovered = real_epsilon.inverse_elliptic_e(&m)?.elliptic_f(&m)?;
    assert_eq!(
        (recovered - &x).exact_rational_normal_form(),
        Some(hyperreal::Rational::zero()),
        "real epsilon composition, {data:?}"
    );
    let epsilon = Complex::new(x, y).jacobi_epsilon(&m)?;
    for value in [&epsilon.re, &epsilon.im] {
        let [lo, hi] = value
            .certified_rational_interval(-128)
            .expect("finite epsilon component");
        assert!(lo <= hi);
    }
    let axis = &epsilon.re * &epsilon.re + Real::from(2);
    let expected_x = &axis + &epsilon.im;
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(axis),
        Length::metres(Real::one()),
        &mut context,
    )?);
    let provenance = InputProvenance::new(
        "epsilon geometry",
        InputAccuracy::Qualified("external observation remains qualified".into()),
    );
    let position = GeodeticPosition::new(
        Latitude::new(Angle::radians(Real::zero()), &mut context)?,
        Longitude::unrolled(Angle::radians(Real::zero())),
        Length::metres(epsilon.im),
        ellipsoid,
        ReferenceFrame::fixed("epsilon consumer")?,
        provenance.clone(),
        &mut context,
    )?;
    let ecef = position.to_ecef(&mut context)?;
    assert_eq!(
        (&ecef.coordinates().x - &expected_x).exact_rational_normal_form(),
        Some(hyperreal::Rational::zero()),
        "ECEF epsilon composition, {data:?}"
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
