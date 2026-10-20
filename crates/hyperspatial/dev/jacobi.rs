//! Retained Jacobi norms and positive axes consumed by strict spatial operations.
use crate::scalar_cases;
use hyperspatial::*;
use std::{cmp::Ordering, sync::Arc};

fn close(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(
            &(actual - expected).abs(),
            &"1e-30".parse().unwrap(),
            PredicatePolicy::STRICT,
        )
        .value(),
        Some(Ordering::Less)
    );
}

fn has_input(receipt: &Receipt, expected: &InputProvenance) -> bool {
    receipt.input() == Some(expected) || receipt.parents().iter().any(|p| has_input(p, expected))
}

/// Two scalar families, period count/parity, regime, sign and display warming.
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
    let small = (&first.value / (Real::from(2) * (&first.value + &second.value)))?;
    let c = match data[4] % 4 {
        1 => Real::one(),
        3 => &first.value * "1e-20".parse::<Real>().unwrap(),
        _ => (&first.value / &second.value)?,
    };
    let m = Real::one() - &c;
    let periods = if data[4] % 4 == 2 {
        Real::from(2 * u16::from(data[2]) + u16::from(data[3] & 1))
    } else {
        Real::zero()
    };
    let input = &small + periods * Real::elliptic_k(&m)?;
    let input = if data[5] & 1 == 0 { input } else { -input };
    let (sn, cn, dn) = input.clone().jacobi_sncndn(&m)?;
    for value in [&sn, &cn, &dn] {
        let [lo, hi] = value
            .certified_rational_interval(-128)
            .expect("Jacobi enclosure");
        assert!(lo <= hi);
        if lo == hi {
            assert_eq!(value.exact_rational_normal_form(), Some(lo));
        }
    }
    let norm = &sn * &sn + &cn * &cn;
    assert_eq!(
        norm.exact_rational_normal_form(),
        Some(hyperreal::Rational::one())
    );
    // Independent evaluation routes exercise the amplitude/trig relation;
    // the integration tests separately compare fresh MPFR constructions.
    let amplitude = input.jacobi_amplitude(&m)?;
    close(&sn, &amplitude.clone().sin());
    close(&cn, &amplitude.cos());
    close(&(&dn * &dn), &(&cn * &cn + &c * (&sn * &sn)));
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let a = Real::one() + &dn;
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(a.clone()),
        Length::metres(dn),
        &mut context,
    )?);
    let input = InputProvenance::new(
        "Jacobi geometry",
        InputAccuracy::Qualified("finite external observations remain qualified".into()),
    );
    let position = GeodeticPosition::new(
        Latitude::new(Angle::radians(Real::zero()), &mut context)?,
        Longitude::unrolled(Angle::radians(Real::zero())),
        Length::metres(norm),
        ellipsoid,
        ReferenceFrame::fixed("Jacobi consumer")?,
        input.clone(),
        &mut context,
    )?;
    let point = position.to_ecef(&mut context)?;
    close(&point.coordinates().x, &(a + Real::one()));
    assert_ne!(point.receipt().certainty(), Certainty::Approximate);
    assert!(has_input(point.receipt(), &input));
    Ok((
        context.steps_used(),
        point.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
