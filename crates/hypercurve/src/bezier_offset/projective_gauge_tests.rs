//! A homogeneous sign change preserves the affine point and its predicates.
use super::*;

fn gauged_point(x_offset: i8, y: i8, gauge: i8) -> CurvePoint2 {
    let half = (Real::one() / Real::from(2)).unwrap();
    let parameter = BezierAlgebraicParameter2::try_isolate(
        crate::BezierParameterPolynomial::try_new_power_basis(vec![
            -&half,
            Real::zero(),
            Real::one(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(half, Real::one()).unwrap(),
    )
    .unwrap();
    let g = Real::from(gauge);
    // t > 0, t^2=1/2. Coordinates are (t+x_offset,y), represented
    // with weight g*t so the sign is selected-field data, not a flag.
    let image = RationalBezierAlgebraicPointImage2::from_retained_expression(
        parameter.clone(),
        parameter_representation(&parameter, &CurveContext::STRICT),
        vec![Real::zero(), &g * Real::from(x_offset), g.clone()],
        vec![Real::zero(), &g * Real::from(y)],
        vec![Real::zero(), g],
        "projective gauge predicate regression",
    );
    CurvePoint2(CurvePointData2::Algebraic(image))
}

#[test]
fn signed_algebraic_gauges_preserve_orientation_order_and_equality() {
    use crate::classify::LineSide;
    use std::cmp::Ordering;
    for policy in [
        CurveContext::STRICT,
        CurveContext::APPROXIMATE_512,
        CurveContext::STRICT,
    ] {
        for a in [-2, 1] {
            let start = gauged_point(0, 0, a);
            assert_eq!(
                recursive_projective_evidence_denominator_sign(&start, &policy).unwrap(),
                Classification::Decided(if a < 0 {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                })
            );
            for b in [-1, 2] {
                let end = gauged_point(1, 0, b);
                assert_eq!(
                    recursive_projective_point_evidence_axis_order(&start, &end, Axis2::X, &policy)
                        .unwrap(),
                    Classification::Decided(Some(Ordering::Less))
                );
                assert_eq!(
                    recursive_projective_point_evidence_axis_order(&end, &start, Axis2::X, &policy)
                        .unwrap(),
                    Classification::Decided(Some(Ordering::Greater))
                );
                for g in [-2, 1] {
                    let above = gauged_point(0, 1, g);
                    let same = gauged_point(0, 0, g);
                    for certified_nonzero in [false, true] {
                        assert_eq!(
                            recursive_projective_point_evidence_oriented_side(
                                &start,
                                &end,
                                &above,
                                certified_nonzero,
                                &policy
                            )
                            .unwrap(),
                            Classification::Decided(Some(LineSide::Left))
                        );
                        assert_eq!(
                            recursive_projective_point_evidence_oriented_side(
                                &end,
                                &start,
                                &above,
                                certified_nonzero,
                                &policy
                            )
                            .unwrap(),
                            Classification::Decided(Some(LineSide::Right))
                        );
                    }
                    assert_eq!(
                        recursive_projective_point_evidence_oriented_side(
                            &start, &end, &same, false, &policy
                        )
                        .unwrap(),
                        Classification::Decided(Some(LineSide::On))
                    );
                    assert_eq!(
                        recursive_projective_point_evidence_axis_order(
                            &start,
                            &same,
                            Axis2::X,
                            &policy
                        )
                        .unwrap(),
                        Classification::Decided(Some(Ordering::Equal))
                    );
                    assert_eq!(
                        recursive_projective_point_evidence_equality(&start, &same, &policy)
                            .unwrap(),
                        Classification::Decided(Some(true))
                    );
                    assert_eq!(
                        recursive_projective_point_evidence_equality(&start, &above, &policy)
                            .unwrap(),
                        Classification::Decided(Some(false))
                    );
                }
            }
        }
    }
}

fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("exact gauge fixture: {reason:?}"),
    }
}

#[test]
fn reversed_support_intersections_keep_positive_recursive_weights() {
    use crate::classify::LineSide::{Left, Right};
    let flip = |side| match side {
        Left => Right,
        Right => Left,
        other => other,
    };
    for policy in [
        CurveContext::STRICT,
        CurveContext::APPROXIMATE_512,
        CurveContext::STRICT,
    ] {
        for gauges in [
            [1, 1, 1, 1],
            [-1, 1, 1, 1],
            [1, -1, 1, -1],
            [-1, -1, -1, -1],
        ] {
            let horizontal = decided(
                BezierAlgebraicChord2::try_new(
                    gauged_point(0, 0, gauges[0]),
                    gauged_point(2, 0, gauges[1]),
                    &policy,
                )
                .unwrap(),
            );
            let vertical = decided(
                BezierAlgebraicChord2::try_new(
                    gauged_point(1, -1, gauges[2]),
                    gauged_point(1, 1, gauges[3]),
                    &policy,
                )
                .unwrap(),
            );
            for reverse_first in [false, true] {
                for reverse_second in [false, true] {
                    let first = if reverse_first {
                        horizontal.reversed()
                    } else {
                        horizontal.clone()
                    };
                    let second = if reverse_second {
                        vertical.reversed()
                    } else {
                        vertical.clone()
                    };
                    let mut first_sides = [Left, Right];
                    let mut second_sides = [Right, Left];
                    if reverse_first {
                        first_sides.reverse();
                        second_sides = second_sides.map(flip);
                    }
                    if reverse_second {
                        second_sides.reverse();
                        first_sides = first_sides.map(flip);
                    }
                    let cross = if reverse_first ^ reverse_second {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    };
                    let intersection = BezierAlgebraicChordPairPoint2::new(
                        first,
                        second,
                        first_sides,
                        second_sides,
                        cross,
                        &policy,
                    );
                    for _ in 0..2 {
                        let point =
                            decided(intersection.recursive_projective_point(&policy).unwrap())
                                .unwrap();
                        assert_eq!(
                            point.denominator.sign_with_nonzero_certificate().unwrap(),
                            Classification::Decided(RealSign::Positive),
                            "gauges={gauges:?}, reversed={reverse_first}/{reverse_second}"
                        );
                    }
                    let intersection = CurvePoint2::from(intersection);
                    assert_eq!(
                        recursive_projective_point_evidence_equality(
                            &intersection,
                            &gauged_point(1, 0, -1),
                            &policy,
                        )
                        .unwrap(),
                        Classification::Decided(Some(true))
                    );
                    assert_eq!(
                        recursive_projective_point_evidence_oriented_side(
                            &gauged_point(0, -1, 1),
                            &gauged_point(2, -1, 1),
                            &intersection,
                            true,
                            &policy,
                        )
                        .unwrap(),
                        Classification::Decided(Some(Left))
                    );
                }
            }
        }
    }
}

#[test]
fn cardinal_displacements_retain_the_imported_weight_sign() {
    use crate::classify::LineSide::Left;
    for policy in [
        CurveContext::STRICT,
        CurveContext::APPROXIMATE_512,
        CurveContext::STRICT,
    ] {
        for first_gauge in [-1, 1] {
            for second_gauge in [-1, 1] {
                let source = decided(
                    BezierAlgebraicChord2::try_new(
                        gauged_point(0, 0, first_gauge),
                        gauged_point(1, 0, second_gauge),
                        &policy,
                    )
                    .unwrap(),
                );
                assert!(source.certified_axis_direction().is_some());
                let (start, end) = BezierAlgebraicChordParallelPoint2::new_pair(
                    source,
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                    &policy,
                );
                let points = [CurvePoint2::from(start), CurvePoint2::from(end)];
                let raw = decided(
                    recursive_projective_evidence_points(&[&points[0], &points[1]], &policy)
                        .unwrap(),
                )
                .unwrap();
                for (evidence, raw) in points.iter().zip(raw) {
                    assert_eq!(
                        recursive_projective_evidence_denominator_sign(evidence, &policy).unwrap(),
                        raw.denominator.sign_with_nonzero_certificate().unwrap()
                    );
                }
                assert_eq!(
                    recursive_projective_point_evidence_equality(
                        &points[0],
                        &gauged_point(0, 1, 1),
                        &policy,
                    )
                    .unwrap(),
                    Classification::Decided(Some(true))
                );
                assert_eq!(
                    recursive_projective_point_evidence_oriented_side(
                        &points[0],
                        &points[1],
                        &gauged_point(0, 2, 1),
                        true,
                        &policy,
                    )
                    .unwrap(),
                    Classification::Decided(Some(Left))
                );
            }
        }
    }
}
