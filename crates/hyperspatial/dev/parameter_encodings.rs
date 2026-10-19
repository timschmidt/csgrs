//! Exact bounded-parameter replay consumed by geodetic and anchored local operations.
use crate::scalar_cases;
use hyperspatial::*;
use std::sync::Arc;

fn has_input(receipt: &Receipt, expected: &InputProvenance) -> bool {
    receipt.input() == Some(expected) || receipt.parents().iter().any(|p| has_input(p, expected))
}

/// Parameter/meridian scalar families, near-boundary scale, time endpoint,
/// chart regime and sign/local-axes/display flags. All inputs remain exact.
pub fn exercise(data: [u8; 6]) -> Result<(u64, usize, &'static str, &'static str)> {
    let cases = scalar_cases::optimized_certificate_representatives();
    let first = &cases[usize::from(data[0]) % cases.len()];
    let second = &cases[usize::from(data[1]) % cases.len()];
    for case in [first, second] {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
    }
    let x = (&first.value / (&first.value + Real::one()))?;
    let y = (&second.value / (&second.value + Real::one()))?;
    let delta = Real::from(10).powi((-(i32::from(data[2]) + 1)).into())?;
    let (m, mut v) = match data[4] % 4 {
        0 => ((&x / Real::from(2))?, y),
        1 => (Real::zero(), y),
        2 => (delta.clone(), Real::zero()),
        _ => (Real::one() - &delta, Real::one() - &delta),
    };
    if data[5] & 1 != 0 {
        v = -v;
    }
    let u = if data[4] % 4 == 2 {
        Real::from(i32::from(data[3] % 3) - 1)
    } else {
        v.clone()
    };
    let r = (&m / (Real::one() - &m))?.sqrt()?;
    let a = (&v / (Real::one() - &v * &v).sqrt()?)?;
    let tau = u.clone().asin()?;
    let payload = [v, m, u, r, a, tau];
    let wire = serde_json::to_string(&payload).expect("finite Real payload");
    let [v, m, u, r, a, tau]: [Real; 6] = serde_json::from_str(&wire).expect("own payload replay");
    if data[5] & 4 != 0 {
        for value in [&v, &m, &u, &r, &a, &tau] {
            let _ = value.to_f64_lossy();
        }
    }
    let decoded_h = (Real::one() / (Real::one() + &r * &r))?;
    let decoded_v = (&a / (Real::one() + &a * &a).sqrt()?)?;
    let decoded_u = tau.sin();
    for (label, residual) in [
        ("parameter", &decoded_h - (Real::one() - &m)),
        ("meridian", &decoded_v - &v),
        ("time", decoded_u - &u),
    ] {
        assert_eq!(
            residual.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero()),
            "{label}: {data:?}"
        );
        assert_eq!(
            residual.sin().exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
    }
    let axis = decoded_h + Real::from(2);
    let expected_x = &axis + &decoded_v;
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(axis),
        Length::metres(Real::one()),
        &mut context,
    )?);
    let provenance = InputProvenance::new(
        "parameter encoding geometry",
        InputAccuracy::Qualified("external observation remains qualified".into()),
    );
    let position = GeodeticPosition::new(
        Latitude::new(Angle::radians(Real::zero()), &mut context)?,
        Longitude::unrolled(Angle::radians(Real::zero())),
        Length::metres(decoded_v),
        ellipsoid,
        ReferenceFrame::fixed("parameter encoding consumer")?,
        provenance.clone(),
        &mut context,
    )?;
    let ecef = position.to_ecef(&mut context)?;
    assert_eq!(
        (&ecef.coordinates().x - &expected_x).exact_rational_normal_form(),
        Some(hyperreal::Rational::zero()),
        "ECEF parameter encoding composition, {data:?}"
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
