use super::*;
use crate::{Point2, RationalQuadraticBezier2};
use std::sync::Arc;

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn half_sqrt_two() -> Real {
    (Real::one() / Real::from(2)).unwrap().sqrt().unwrap()
}

/// The quarter of the unit circle from (1, 0) to (0, 1).
fn quarter() -> RationalQuadraticBezier2 {
    RationalQuadraticBezier2::try_new(
        point(1, 0),
        point(1, 1),
        point(0, 1),
        Real::one(),
        half_sqrt_two(),
        Real::one(),
    )
    .unwrap()
}

/// Attaches a certificate without checking it, as a live construction would.
fn with_circle(conic: RationalQuadraticBezier2, center: Point2, radius_squared: Real) -> Curve2 {
    let circle = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
        center,
        radius_squared,
        tangent_contacts: None,
    });
    Curve2::from(
        conic.with_retained_conic_provenance(Some(circle.implicit_coefficients()), Some(circle)),
    )
}

fn replay(curve: &Curve2) -> SvgResult<Curve2> {
    let mut writer = GraphWriter::new(1_048_576, 256);
    let reference = writer.curve(curve)?;
    let mut input = writer.count.to_be_bytes().to_vec();
    input.extend_from_slice(&writer.writer.bytes);
    let mut reader = ExactReader::new(&input, 1_048_576);
    let definitions = read_definitions(&mut reader, 256)?;
    assert!(reader.is_empty());
    curve_reference(&definitions, reference as usize).cloned()
}

fn retained_circle(curve: &Curve2) -> Option<(Point2, Real)> {
    match curve.geometry() {
        Some(CurveGeometry2::RationalQuadraticBezier(conic)) => conic
            .retained_circular_conic()
            .map(|circle| (circle.center.clone(), circle.radius_squared.clone())),
        _ => None,
    }
}

#[test]
fn replay_rechecks_and_keeps_a_retained_circle() {
    let curve = with_circle(quarter(), point(0, 0), Real::one());
    let restored = replay(&curve).unwrap();
    let (center, radius_squared) = retained_circle(&restored).unwrap();
    assert_eq!(center, point(0, 0));
    assert_eq!(radius_squared, Real::one());

    // The pole-free major arc about (-1, 1) has a negative interior weight.
    let major = RationalQuadraticBezier2::try_new(
        point(-1, 0),
        point(0, 0),
        point(0, 1),
        Real::one(),
        -half_sqrt_two(),
        Real::one(),
    )
    .unwrap();
    let restored = replay(&with_circle(major, point(-1, 1), Real::one())).unwrap();
    assert_eq!(retained_circle(&restored).unwrap().0, point(-1, 1));

    // Without a certificate the native record is unchanged and none appears.
    let restored = replay(&Curve2::from(quarter())).unwrap();
    assert!(retained_circle(&restored).is_none());
}

#[test]
fn replay_rejects_a_certificate_the_curve_does_not_satisfy() {
    // Both endpoints lie on the circle of radius 1 about (1, 1), but the
    // weighted interior control does not.
    for (center, radius_squared) in [
        (point(1, 1), Real::one()),
        (point(0, 0), Real::from(2)),
        (point(0, 0), Real::zero()),
        (point(0, 0), -Real::one()),
    ] {
        let curve = with_circle(quarter(), center, radius_squared);
        assert!(matches!(replay(&curve), Err(SvgError::MalformedInput(_))));
    }
}
