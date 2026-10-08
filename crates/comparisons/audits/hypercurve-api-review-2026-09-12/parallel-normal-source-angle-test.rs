
#[cfg(test)]
mod parallel_normal_source_angle_tests {
    use super::*;
    use std::cmp::Ordering;

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("exact fixture: {reason:?}"),
        }
    }

    #[test]
    fn selected_chord_parallel_normal_angles_use_source_direction() {
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        // P(t)=(t,t²) has source tangent (1,0) at t=0. The distance-one
        // parallel has the opposite tangent there: Q'(0)=(-1,0). Both
        // normal frames still use the source's upward unit normal.
        let source = BezierParallelSource2::Quadratic(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            Point2::from_values(1, 1),
        ));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for distance in [0, 1] {
                let parallel = BezierParallel2::from_source(source.clone(), Real::from(distance));
                let selected = BezierParameter2::Exact(Real::zero());
                assert_eq!(
                    decided(parallel.parallel_derivative_scale_sign(&selected, &policy).unwrap()),
                    if distance == 0 { RealSign::Positive } else { RealSign::Negative }
                );
                for clockwise in [false, true] {
                    let tangent = (Real::from(4), Real::from(if clockwise { -3 } else { 3 }));
                    for unit_evidence in [false, true] {
                        let chord = if unit_evidence {
                            let unit = decided(crate::direction::UnitDirection2::from_direction(&tangent).unwrap());
                            decided(BezierAlgebraicChord2::from_unit_direction(&unit, &policy).unwrap())
                        } else {
                            decided(BezierAlgebraicChord2::try_new(
                                Point2::from_values(0, 0).into(),
                                Point2::new(tangent.0.clone(), tangent.1.clone()).into(),
                                &policy,
                            ).unwrap())
                        };
                        for radius in [-1, 1] {
                            let radius = Real::from(radius);
                            let circle = decided(BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                                parallel.clone(), selected.clone(), radius.clone(), clockwise, &policy,
                            ).unwrap()).unwrap();
                            let center = CurvePoint2::from(BezierAnalyticParallelPoint2::new(
                                parallel.clone(), selected.clone(), &policy,
                            ));
                            let contact = chord.normal_displaced_point_evidence(center, radius.clone(), &policy).unwrap();
                            let parameter = decided(circle.certified_selected_chord_parallel_normal_contact_parameter(
                                chord.clone(), contact.clone(), radius.clone(),
                                if clockwise { RealSign::Negative } else { RealSign::Positive }, &policy,
                            ).unwrap());
                            // At u=1/4 the half-circle chart has cos=4/5,
                            // sin=3/5. This independent 3-4-5 construction
                            // fixes the contact point and its exact angular order.
                            let expected = CurvePoint2::from(Point2::new(
                                -tangent.1.clone() * q(1, 5) * &radius,
                                Real::from(distance) + q(4, 5) * &radius,
                            ));
                            assert_eq!(contact.same_point(&expected, &policy), Classification::Decided(true));
                            let evaluated = decided(circle.point_evidence_at(&q(1, 4), &policy).unwrap());
                            assert_eq!(evaluated.same_point(&expected, &policy), Classification::Decided(true));
                            for (cut, expected_order) in [(q(1, 8), Ordering::Greater), (q(1, 4), Ordering::Equal), (q(3, 8), Ordering::Less)] {
                                assert_eq!(
                                    parameter.order_to_real(&cut, &policy).unwrap(),
                                    Classification::Decided(expected_order),
                                    "distance={distance} clockwise={clockwise} unit_evidence={unit_evidence} radius={radius:?} cut={cut:?} policy={policy:?}"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}
