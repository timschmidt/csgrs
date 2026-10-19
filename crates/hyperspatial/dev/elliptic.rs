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
    // The new scalar output must remain usable by an existing native spatial
    // construction, strict positivity proof, and geodetic forward operation.
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(rf.clone()),
        Length::metres(rd),
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
    close(&point.coordinates().x, &(rf + height));
    assert_ne!(point.receipt().certainty(), Certainty::Approximate);
    Ok((
        context.steps_used(),
        point.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
