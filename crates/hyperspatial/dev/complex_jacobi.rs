//! Complex Jacobi relations consumed by strict geodetic construction.
use crate::scalar_cases;
use hyperlattice::Complex;
use hyperspatial::*;
use std::sync::Arc;

fn exact(value: &Complex, real: i32, data: [u8; 6], relation: &str) {
    assert_eq!(
        value.re.exact_rational_normal_form(),
        Some(hyperreal::Rational::from(real)),
        "case {data:?}, {relation}, real"
    );
    assert_eq!(
        value.im.exact_rational_normal_form(),
        Some(hyperreal::Rational::zero()),
        "case {data:?}, {relation}, imaginary"
    );
}

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
    let (sn, cn, dn) = Complex::new(x, y).jacobi_sncndn(&m)?;
    for component in [&sn.re, &sn.im, &cn.re, &cn.im, &dn.re, &dn.im] {
        let [lo, hi] = component
            .certified_rational_interval(-128)
            .expect("finite Jacobi component");
        assert!(lo <= hi);
    }
    let norm = &sn * &sn + &cn * &cn;
    exact(&norm, 1, data, "unit norm");
    exact(
        &(&dn * &dn + Complex::from(m.clone()) * (&sn * &sn)),
        1,
        data,
        "elliptic norm",
    );
    exact(
        &(&dn * &dn - Complex::from(m.clone()) * (&cn * &cn) - Complex::from(Real::one() - &m)),
        0,
        data,
        "complementary norm",
    );
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(&norm.re + Real::from(2)),
        Length::metres(Real::from(2)),
        &mut context,
    )?);
    let provenance = InputProvenance::new(
        "complex Jacobi geometry",
        InputAccuracy::Qualified("external observation remains qualified".into()),
    );
    let position = GeodeticPosition::new(
        Latitude::new(Angle::radians(norm.im), &mut context)?,
        Longitude::unrolled(Angle::radians(Real::zero())),
        Length::metres(norm.re),
        ellipsoid,
        ReferenceFrame::fixed("complex Jacobi consumer")?,
        provenance.clone(),
        &mut context,
    )?;
    let point = position.to_ecef(&mut context)?;
    assert_eq!(
        (point.coordinates().x.clone() - Real::from(4)).exact_rational_normal_form(),
        Some(hyperreal::Rational::zero()),
        "ECEF composition, case {data:?}"
    );
    assert_ne!(point.receipt().certainty(), Certainty::Approximate);
    assert!(has_input(point.receipt(), &provenance));
    Ok((
        context.steps_used(),
        point.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
