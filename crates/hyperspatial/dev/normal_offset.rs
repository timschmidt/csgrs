//! Development-only, independently constructed ellipsoid-normal workflow.

use std::{cmp::Ordering, sync::Arc};

use hyperlattice::{Point3, Vector3};
use hyperspatial::*;

use crate::scalar_cases;

fn close(actual: &Real, expected: &Real) {
    let bound: Real = "0.000000000000000000000000000001".parse().unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&(actual - expected).abs(), &bound, PredicatePolicy::STRICT)
            .value(),
        Some(Ordering::Less)
    );
}

/// Six bytes select two scalar families, two axis shifts, height and axes/cache.
/// Returns work checkpoints, evidence depth and the two input certificate names.
pub fn exercise(data: [u8; 6]) -> Result<(u64, usize, &'static str, &'static str)> {
    let cases = scalar_cases::optimized_certificate_representatives();
    assert_eq!(cases.len(), 22);
    let first = &cases[usize::from(data[0]) % cases.len()];
    let second = &cases[usize::from(data[1]) % cases.len()];
    for case in [first, second] {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        if data[5] & 2 != 0 {
            let _ = case.value.to_f32_lossy();
            let _ = case.value.to_f64_lossy();
        }
    }
    let a = Real::from(100 + i32::from(data[2])) + &first.value;
    let b = Real::from(100 + i32::from(data[3])) + &second.value;
    let h = (&first.value + &second.value) * Real::from(1 + i32::from(data[4] % 8));
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let ellipsoid = Arc::new(Ellipsoid::from_axes(
        Length::metres(a.clone()),
        Length::metres(b.clone()),
        &mut context,
    )?);
    let reference = ReferenceFrame::fixed("test:normal-offset")?;
    let input = InputProvenance::new("independent normal construction", InputAccuracy::ExactModel);
    // At normal latitude 45 degrees the foot is (a²,b²)/hypot(a,b).
    // Strictly positive h selects its unique closest exterior normal.
    let d = Real::hypot2(&a, &b)?;
    let delta = (&h / Real::from(2).sqrt()?)?;
    let point = EcefPosition::new(
        Point3::new(
            ((&a * &a) / &d)? + &delta,
            Real::zero(),
            ((&b * &b) / d)? + delta,
        ),
        reference.clone(),
        input.clone(),
        &mut context,
    )?;
    let inverse = point.to_geodetic_closest(ellipsoid, PoleMeridian::Reject, &mut context)?;
    close(
        inverse.latitude().angle().as_radians(),
        Angle::degrees(45.into()).as_radians(),
    );
    close(inverse.longitude().angle().as_radians(), &Real::zero());
    close(inverse.height().as_metres(), &h);
    // Construct a new geodetic value so retained source identity cannot hide a
    // wrong inverse behind an identity shortcut on conversion back to ECEF.
    let evaluated = GeodeticPosition::new(
        inverse.latitude().clone(),
        inverse.longitude().clone(),
        inverse.height().clone(),
        inverse.ellipsoid().clone(),
        reference.clone(),
        input.clone(),
        &mut context,
    )?
    .to_ecef(&mut context)?;
    close(&evaluated.coordinates().x, &point.coordinates().x);
    close(&evaluated.coordinates().y, &point.coordinates().y);
    close(&evaluated.coordinates().z, &point.coordinates().z);
    let axes = if data[5] & 1 == 0 {
        LocalAxes::Enu
    } else {
        LocalAxes::Ned
    };
    let frame = LocalFrame::new(&inverse, axes, &mut context)?;
    let local = frame.to_local_position(&point, &mut context)?;
    close(&local.coordinates().x, &Real::zero());
    close(&local.coordinates().y, &Real::zero());
    close(&local.coordinates().z, &Real::zero());
    let vector = EcefVector::new(
        Vector3::new([first.value.clone(), second.value.clone(), h]),
        reference,
        input,
        &mut context,
    )?;
    let rotated = frame.to_local_vector(&vector, &mut context)?;
    let returned = frame.to_ecef_vector(&rotated, &mut context)?;
    close(&returned.coordinates()[0], &vector.coordinates()[0]);
    close(&returned.coordinates()[1], &vector.coordinates()[1]);
    close(&returned.coordinates()[2], &vector.coordinates()[2]);
    assert_ne!(returned.receipt().certainty(), Certainty::Approximate);
    assert_ne!(evaluated.receipt().certainty(), Certainty::Approximate);
    Ok((
        context.steps_used(),
        returned.receipt().depth(),
        first.certificate,
        second.certificate,
    ))
}
