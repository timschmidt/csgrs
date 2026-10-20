//! Native inverse-integral outputs consumed by strict spatial operations.
use crate::scalar_cases;
use hyperspatial::*;
use std::{cmp::Ordering, sync::Arc};

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

fn has_input(receipt: &Receipt, expected: &InputProvenance) -> bool {
    receipt.input() == Some(expected)
        || receipt
            .parents()
            .iter()
            .any(|parent| has_input(parent, expected))
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
    let m = Real::one() - c;
    let mut outputs = Vec::new();
    for second_kind in [false, true] {
        let complete = if second_kind {
            Real::elliptic_e_complete(&m)?
        } else {
            Real::elliptic_k(&m)?
        };
        let periods = if data[4] % 4 == 2 {
            Real::from(2 * u16::from(data[2]) + u16::from(data[3] & 1))
        } else {
            Real::zero()
        };
        let input = &small + &periods * &complete;
        let input = if data[5] & 1 == 0 { input } else { -input };
        let output = if second_kind {
            input.clone().inverse_elliptic_e(&m)?
        } else {
            input.clone().jacobi_amplitude(&m)?
        };
        // Force a new numerical inverse, not just an inverse-relation shortcut.
        let [lo, hi] = output
            .certified_rational_interval(-128)
            .expect("inverse enclosure");
        assert!(hi >= lo);
        if hi == lo {
            assert_eq!(output.exact_rational(), Some(lo));
        }
        let recovered = if second_kind {
            output.clone().elliptic_e(&m)?
        } else {
            output.clone().elliptic_f(&m)?
        };
        close(&recovered, &input);
        outputs.push(output.abs());
    }
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(outputs[0].clone()),
        Length::metres(outputs[1].clone()),
        &mut context,
    )?);
    let input = InputProvenance::new(
        "inverse elliptic geometry",
        InputAccuracy::Qualified("finite external observations remain qualified".into()),
    );
    let position = GeodeticPosition::new(
        Latitude::new(Angle::radians(Real::zero()), &mut context)?,
        Longitude::unrolled(Angle::radians(Real::zero())),
        Length::metres(small.clone()),
        ellipsoid,
        ReferenceFrame::fixed("inverse Legendre consumer")?,
        input.clone(),
        &mut context,
    )?;
    let point = position.to_ecef(&mut context)?;
    close(&point.coordinates().x, &(&outputs[0] + small));
    assert_ne!(point.receipt().certainty(), Certainty::Approximate);
    assert!(has_input(point.receipt(), &input));
    Ok((
        context.steps_used(),
        point.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
