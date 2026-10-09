//! Optional scalar views of selected roots, retaining the original isolator.

use super::*;

/// Candidate construction shared with non-rational low-degree isolation.
/// Every caller must certify polynomial replay and its own root domain.
pub(super) fn exact_low_degree_root_candidates(
    coefficients: &[Real],
) -> CurveResult<Option<Vec<Real>>> {
    const EXACT_REFINEMENT_PRECISION: i32 = -512;

    let candidates = match coefficients {
        [_constant] => Vec::new(),
        [constant, linear] => {
            if !matches!(
                linear.certified_sign_until(EXACT_REFINEMENT_PRECISION),
                CertifiedRealSign::Known {
                    sign: RealSign::Positive | RealSign::Negative,
                    ..
                }
            ) {
                return Ok(None);
            }
            vec![((-constant.clone()) / linear)?]
        }
        [constant, linear, quadratic] => {
            match quadratic.certified_sign_until(EXACT_REFINEMENT_PRECISION) {
                CertifiedRealSign::Known {
                    sign: RealSign::Positive | RealSign::Negative,
                    ..
                } => {}
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                }
                | CertifiedRealSign::Unknown { .. } => return Ok(None),
            };
            let discriminant = linear * linear - Real::from(4_i8) * quadratic * constant;
            match discriminant.certified_sign_until(EXACT_REFINEMENT_PRECISION) {
                CertifiedRealSign::Known {
                    sign: RealSign::Negative,
                    ..
                } => Vec::new(),
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => vec![((-linear.clone()) / (Real::from(2_i8) * quadratic))?],
                CertifiedRealSign::Known {
                    sign: RealSign::Positive,
                    ..
                } => {
                    let denominator = Real::from(2_i8) * quadratic;
                    let vertex = ((-linear.clone()) / &denominator)?;
                    let delta = (discriminant / (&denominator * &denominator))?.sqrt()?;
                    vec![&vertex - &delta, vertex + delta]
                }
                CertifiedRealSign::Unknown { .. } => return Ok(None),
            }
        }
        _ => return Ok(None),
    };

    Ok(Some(candidates))
}

impl BezierAlgebraicParameter2 {
    /// Explicit optional projection for a native line/arc endpoint.
    /// General field kernels keep their existing compact projection behavior.
    pub(crate) fn represented_quadratic_point_with_policy(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        if let Some(root) = self.scalar() {
            return Ok(Classification::Decided(Some(root.clone())));
        }
        if self.polynomial().degree() != 2 {
            return Ok(Classification::Decided(None));
        }
        // Rational-search exhaustion does not disprove a quadratic witness.
        // Do not publish this view into the general scalar-witness cache:
        // doing so can change which selected-field representation a later
        // chord/offset query receives. The owning native contour caches its
        // completed projection without changing the root's field authority.
        self.represented_quadratic_root(&policy.strict_counterpart())
    }

    fn represented_quadratic_root(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        selected_quadratic_scalar(
            self.polynomial().coefficients(),
            self.interval().start(),
            self.interval().end(),
            policy,
        )
    }
}

/// Optional exact scalar view of a validated singleton root, without changing
/// its owning field or caches. The caller retains the source/selection proof.
pub(crate) fn selected_quadratic_scalar(
    coefficients: &[Real],
    lower: &Real,
    upper: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    if coefficients.len() != 3 {
        return Ok(Classification::Decided(None));
    }
    let Some(candidates) = exact_low_degree_root_candidates(coefficients)? else {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    };
    let mut unresolved = None;
    for root in candidates {
        match (
            compare_reals(lower, &root, policy),
            compare_reals(&root, upper, policy),
        ) {
            (Some(Ordering::Equal | Ordering::Greater), _)
            | (_, Some(Ordering::Equal | Ordering::Greater)) => continue,
            (Some(Ordering::Less), Some(Ordering::Less)) => {}
            _ => {
                unresolved = Some(UncertaintyReason::Ordering);
                continue;
            }
        }
        if real_sign(&Real::eval_poly(coefficients, &root), policy) == Some(RealSign::Zero) {
            // The retained interval owns exactly one root. No decision
            // about the other quadratic branch is needed after this proof.
            return Ok(Classification::Decided(Some(root)));
        }
        unresolved = Some(UncertaintyReason::RealSign);
    }
    Ok(match unresolved {
        Some(reason) => Classification::Uncertain(reason),
        None => Classification::Decided(None),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rational(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn selected(coefficients: Vec<Real>, start: Real, end: Real) -> BezierAlgebraicParameter2 {
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(start, end).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn quadratic_scalar_view_selects_both_exterior_branches_without_mutating_field_authority() {
        let root = Real::from(2).sqrt().unwrap();
        for scale in [Real::one(), rational(-7, 97)] {
            for (lower, upper, expected) in [(-2, -1, -&root), (1, 2, root.clone())] {
                for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                    let selected = selected(
                        [-2, 0, 1]
                            .into_iter()
                            .map(|c| Real::from(c) * &scale)
                            .collect(),
                        Real::from(lower),
                        Real::from(upper),
                    );
                    let clone = selected.clone();
                    assert!(clone.scalar().is_none());
                    let before = selected.interval().clone();
                    let outcome = crate::evaluate_with_budget(
                        crate::PredicatePolicy::STRICT,
                        crate::WorkBudget::ZERO,
                        || selected.represented_quadratic_point_with_policy(&policy),
                    );
                    assert_eq!(outcome.usage, crate::WorkUsage::default());
                    let value = outcome.result.unwrap().certified().unwrap().unwrap();
                    let Classification::Decided(Some(actual)) = value else {
                        panic!("quadratic root must have an exact scalar view");
                    };
                    assert_eq!(
                        compare_reals(&actual, &expected, &CurveContext::STRICT),
                        Some(Ordering::Equal)
                    );
                    assert_eq!(selected.interval(), &before);
                    assert!(clone.scalar().is_none());
                    assert!(selected.scalar().is_none());
                    for policy in [CurveContext::APPROXIMATE_512, CurveContext::STRICT] {
                        assert_eq!(
                            clone
                                .represented_quadratic_point_with_policy(&policy)
                                .unwrap(),
                            Classification::Decided(Some(actual.clone()))
                        );
                    }
                    let parameter = BezierParameter2::Algebraic(selected.clone());
                    let mut refinement = BezierParameterRefinement2::new(&parameter, &policy);
                    let BezierParameter2::Algebraic(refined) = refinement.refine_to(8) else {
                        panic!("the root authority must survive its optional scalar view");
                    };
                    assert!(Arc::ptr_eq(&refined.data.shared, &selected.data.shared));
                    assert_eq!(refined.polynomial(), selected.polynomial());
                }
            }
        }
    }

    #[test]
    fn failed_rational_search_during_isolation_does_not_poison_quadratic_projection() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for scale in [Real::one(), rational(-7, 97)] {
                let polynomial = BezierParameterPolynomial::try_new_power_basis(
                    [1, -8, 8]
                        .into_iter()
                        .map(|c| Real::from(c) * &scale)
                        .collect(),
                )
                .unwrap();
                let Classification::Decided(roots) = polynomial
                    .isolate_unit_interval_roots_with_policy(&policy)
                    .unwrap()
                else {
                    panic!("two strictly isolated quadratic roots");
                };
                assert_eq!(roots.len(), 2);
                let delta = rational(1, 8).sqrt().unwrap();
                for (root, expected) in roots
                    .iter()
                    .zip([rational(1, 2) - &delta, rational(1, 2) + &delta])
                {
                    let BezierParameter2::Algebraic(selected) = root else {
                        panic!("isolation must retain the quadratic root carrier");
                    };
                    assert!(root.scalar().is_none());
                    assert!(
                        selected
                            .data
                            .shared
                            .point_reconstruction_exhausted
                            .load(AtomicOrdering::Relaxed)
                    );
                    let Classification::Decided(Some(actual)) = selected
                        .represented_quadratic_point_with_policy(&policy)
                        .unwrap()
                    else {
                        panic!("rational-search exhaustion must allow quadratic reconstruction");
                    };
                    assert_eq!(
                        compare_reals(&actual, &expected, &CurveContext::STRICT),
                        Some(Ordering::Equal)
                    );
                    assert_eq!(
                        real_sign(&polynomial.evaluate(&actual), &CurveContext::STRICT),
                        Some(RealSign::Zero)
                    );
                }
            }
        }
    }

    #[test]
    fn native_projection_does_not_change_independent_chord_field_queries() {
        use crate::{
            BezierAlgebraicChord2, BezierSubcurve2, CurvePoint2, LineSeg2, Point2,
            QuadraticBezier2, RationalBezier2,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parameters = [2, 3].map(|denominator| {
                selected(
                    vec![-rational(1, denominator), Real::zero(), Real::one()],
                    Real::zero(),
                    Real::one(),
                )
            });
            for parameter in &parameters {
                for _ in 0..2 {
                    assert!(matches!(
                        parameter
                            .represented_quadratic_point_with_policy(&policy)
                            .unwrap(),
                        Classification::Decided(Some(_))
                    ));
                }
                assert!(parameter.scalar().is_none());
            }
            let points = [Point2::from_values(1, 0), Point2::from_values(0, 1)]
                .into_iter()
                .zip(&parameters)
                .map(|(end, parameter)| {
                    let source = RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                        QuadraticBezier2::from_line_segment(
                            LineSeg2::try_new(Point2::from_values(0, 0), end).unwrap(),
                        ),
                    ))
                    .unwrap();
                    let Classification::Decided(point) = source
                        .point_at_algebraic_parameter(parameter, &policy)
                        .unwrap()
                    else {
                        panic!("selected source point");
                    };
                    assert!(matches!(
                        point.exact_native_point(&policy).unwrap(),
                        Classification::Decided(Some(_))
                    ));
                    assert!(matches!(
                        point.exact_native_point(&policy).unwrap(),
                        Classification::Decided(Some(_))
                    ));
                    assert!(parameter.scalar().is_none());
                    CurvePoint2::from(point)
                })
                .collect::<Vec<_>>();
            let Classification::Decided(chord) =
                BezierAlgebraicChord2::try_new(points[0].clone(), points[1].clone(), &policy)
                    .unwrap()
            else {
                panic!("independent selected chord");
            };
            assert!(matches!(
                chord.algebraic_ray_evaluator(&policy).unwrap(),
                Classification::Decided(_)
            ));
        }
    }

    #[test]
    fn repeated_and_nested_quadratic_roots_replay_exactly() {
        let alpha = Real::from(2).sqrt().unwrap();
        for (coefficients, lower, upper) in [
            (
                vec![Real::from(2), Real::from(-2) * alpha, Real::one()],
                1,
                2,
            ),
            (
                vec![-rational(1, 2).sqrt().unwrap(), Real::one(), Real::one()],
                0,
                1,
            ),
        ] {
            let selected = selected(coefficients, Real::from(lower), Real::from(upper));
            let Classification::Decided(Some(actual)) = selected
                .represented_quadratic_point_with_policy(&CurveContext::STRICT)
                .unwrap()
            else {
                panic!("explicit native endpoint projection");
            };
            assert!(selected.scalar().is_none());
            assert_eq!(
                real_sign(
                    &selected.polynomial().evaluate(&actual),
                    &CurveContext::STRICT
                ),
                Some(RealSign::Zero)
            );
            assert_eq!(
                compare_reals(&Real::from(lower), &actual, &CurveContext::STRICT),
                Some(Ordering::Less)
            );
            assert_eq!(
                compare_reals(&actual, &Real::from(upper), &CurveContext::STRICT),
                Some(Ordering::Less)
            );
        }
    }
}
