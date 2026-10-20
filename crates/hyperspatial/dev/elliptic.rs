//! Native elliptic prerequisites, shared by representation and adversarial gates.
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

/// Families, scale sign, height, endpoint regime, and optional display warming.
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
    let (x, y, z) = match data[4] % 4 {
        1 => (Real::zero(), second.value.clone(), second.value.clone()),
        3 => (
            Real::zero(),
            &first.value * "1e-40".parse::<Real>().unwrap(),
            second.value.clone(),
        ),
        _ => (
            first.value.clone(),
            second.value.clone(),
            Real::from(1 + u16::from(data[3])),
        ),
    };
    let rf = Real::elliptic_rf(&x, &y, &z)?;
    let rd = Real::elliptic_rd(&x, &y, &z)?;
    let scale: Real = if data[4] % 4 == 2 {
        if data[2] & 1 == 0 { "1e-40" } else { "1e40" }
    } else {
        "4"
    }
    .parse()
    .unwrap();
    let root = scale.clone().sqrt()?;
    close(
        &(Real::elliptic_rf(&(&x * &scale), &(&y * &scale), &(&z * &scale))? * &root),
        &rf,
    );
    close(
        &(Real::elliptic_rd(&(&x * &scale), &(&y * &scale), &(&z * &scale))? * &root * &scale),
        &rd,
    );
    close(&Real::elliptic_rf(&z, &x, &y)?, &rf);
    close(&Real::elliptic_rd(&y, &x, &z)?, &rd);
    if x.definitely_zero() {
        if data[4] % 4 == 1 {
            close(&rf, &(Real::pi() / (Real::from(2) * y.clone().sqrt()?))?);
            close(
                &rd,
                &(Real::from(3) * Real::pi() / (Real::from(4) * &y * y.clone().sqrt()?))?,
            );
        }
    } else {
        let sum = &rd + Real::elliptic_rd(&y, &z, &x)? + Real::elliptic_rd(&z, &x, &y)?;
        close(&sum, &(Real::from(3) / (&x * &y * &z).sqrt()?)?);
    }
    let small = (&first.value / (Real::from(2) * (&first.value + &second.value)))?;
    let complement = match data[4] % 4 {
        1 => Real::one(),
        3 => &first.value * "1e-20".parse::<Real>().unwrap(),
        _ => (&first.value / &second.value)?,
    };
    let parameter = Real::one() - complement;
    let amplitude = match data[4] % 4 {
        1 => (Real::pi() / Real::from(2))?,
        2 => {
            let quarter = if data[3] & 1 == 0 {
                Real::zero()
            } else {
                (Real::pi() / Real::from(2))?
            };
            Real::pi() * Real::from(data[2]) + quarter + small
        }
        _ => small,
    };
    let amplitude = if data[5] & 1 == 0 {
        amplitude
    } else {
        -amplitude
    };
    let f = amplitude.clone().elliptic_f(&parameter)?;
    let e = amplitude.clone().elliptic_e(&parameter)?;
    close(
        &(&amplitude + Real::pi()).elliptic_f(&parameter)?,
        &(&f + Real::from(2) * Real::elliptic_k(&parameter)?),
    );
    close(
        &(&amplitude + Real::pi()).elliptic_e(&parameter)?,
        &(&e + Real::from(2) * Real::elliptic_e_complete(&parameter)?),
    );
    close(&(-&amplitude).elliptic_f(&parameter)?, &(-&f));
    close(&(-amplitude).elliptic_e(&parameter)?, &(-&e));
    // Both Carlson and Legendre outputs must remain usable by a native spatial
    // construction, strict positivity proof, and geodetic forward operation.
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let mut depth = 0;
    for (a, b) in [(rf, rd), (f.abs(), e.abs())] {
        let ellipsoid = Arc::new(Ellipsoid::from_axes(
            Length::metres(a.clone()),
            Length::metres(b),
            &mut context,
        )?);
        let height = Real::from(data[2]);
        let position = GeodeticPosition::new(
            Latitude::new(Angle::radians(Real::zero()), &mut context)?,
            Longitude::unrolled(Angle::radians(Real::zero())),
            Length::metres(height.clone()),
            ellipsoid,
            ReferenceFrame::fixed("elliptic prerequisite composition")?,
            InputProvenance::new("native elliptic model", InputAccuracy::ExactModel),
            &mut context,
        )?;
        let point = position.to_ecef(&mut context)?;
        close(&point.coordinates().x, &(a + height));
        assert_ne!(point.receipt().certainty(), Certainty::Approximate);
        depth = depth.max(point.receipt().depth());
    }
    Ok((
        context.steps_used(),
        depth,
        first.certificate,
        second.certificate,
    ))
}
