
#[cfg(test)]
mod empty_incident_component_tests {
    use super::*;

    fn decided<T>(value: CurveResult<Classification<T>>) -> T {
        match value {
            Ok(Classification::Decided(value)) => value,
            Ok(Classification::Uncertain(reason)) => panic!("empty ray query uncertain: {reason:?}"),
            Err(error) => panic!("empty ray query failed: {error}"),
        }
    }

    #[test]
    fn empty_incident_charts_preserve_finite_components_and_constraints() {
        use CurveParameterComponentSelection2::{NeedsConstraint, Selected};
        let diagonal = BivariatePolynomial::new(vec![
            vec![Real::zero(), -Real::one()], vec![Real::one()],
        ]);
        let positive = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let selector = ParameterComponentSelector2::Positive(&positive, None);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for shift in [-2_i32, 0, 2] {
                let lower = Real::from(shift);
                let upper = Real::from(shift + 1);
                let finite = CurveParameterRange2::new_validated(
                    lower.clone().into(), upper.clone().into(),
                );
                for increasing in [false, true] {
                    let anchor = if increasing { &upper } else { &lower };
                    let direction = if increasing {
                        BezierParameterRayDirection2::Increasing
                    } else {
                        BezierParameterRayDirection2::Decreasing
                    };
                    for blocked_before_anchor in [false, true] {
                        let barrier = BezierParameter2::Exact(if blocked_before_anchor {
                            anchor + Real::from(if increasing { -1_i32 } else { 1 })
                        } else { anchor.clone() });
                        let ray = BezierParameterRay2 { anchor, direction, barrier: Some(&barrier) };
                        for axes in 0..4 {
                            let domains = [0, 1].map(|axis| CurveParameterDomain2::new(
                                &finite, (axes & (1 << axis) != 0).then_some(ray),
                            ));
                            let result = decided(select_parameter_component_in_domain(
                                &diagonal, &selector, domains,
                                ParameterComponentQuery2::AllComponents(None), &policy, config,
                            ));
                            assert_eq!(result.components.len(), 1,
                                "shift {shift}, increasing {increasing}, before {blocked_before_anchor}, axes {axes}");
                            assert!(result.selected_pairs.is_empty());
                            let component = &result.components[0];
                            assert!(matches!(decided(component.constrain([None, None], &policy)), NeedsConstraint));
                            // Either exact contact fixes the retained diagonal family,
                            // including finite endpoints that an empty ray must not revoke.
                            for value in [lower.clone(), ((&lower + &upper) / Real::from(2_i8)).unwrap(), upper.clone()] {
                                let parameter = CurveParameter2::from(value);
                                for constrained_axis in 0..2 {
                                    let constraints = [0, 1].map(|axis| (axis == constrained_axis).then_some(&parameter));
                                    let Selected(pair) = decided(component.constrain(constraints, &policy)) else {
                                        panic!("one exact contact must select the finite family");
                                    };
                                    for actual in pair {
                                        assert!(decided(actual.same_value(&parameter, &policy)));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
