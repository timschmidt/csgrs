//! Point evaluation in a curve's retained parameter chart.

use super::*;
use crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2;
use crate::{BezierParallelSource2, BezierSplitFragment2, CurveParameterRange2, UncertaintyReason};
use std::cmp::Ordering;

fn decided<T>(value: Classification<T>, family: CurveFamily2) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            family,
            reason,
        )),
    }
}

fn evaluation_error(family: CurveFamily2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Evaluation, family, cause)
}

impl Curve2 {
    pub(super) fn point_at_parameter_with_policy(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        if self.source_range().is_some() {
            return self
                .source_range_point_at(parameter, side, policy)
                .map_err(|error| error.with_operation(CurveOperation2::Evaluation));
        }
        if self.geometry().is_some() {
            if let Some(BezierParameter2::Exact(parameter)) = parameter.as_bezier_parameter() {
                return self
                    .point_at_side_raw(parameter, side, policy)
                    .map(CurvePoint2::from);
            }
            return self.point_at_selected_native_parameter(parameter, side, policy);
        }

        let family = self.family();
        let fragment = self.retained_fragment().expect("retained curve");
        let scalar_circle_parameter;
        let parameter = if matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
            && let Some(BezierParameter2::Exact(value)) = parameter.as_bezier_parameter()
        {
            scalar_circle_parameter = CurveParameter2::from_algebraic_cusp(
                BezierAlgebraicCuspSemicircleParameter2::Exact(value.clone()),
            );
            &scalar_circle_parameter
        } else {
            parameter
        };
        let range = self.parameter_domain();
        let compare = |boundary| {
            decided(
                parameter
                    .cmp_by_refinement(boundary, policy)
                    .map_err(|cause| evaluation_error(family, cause))?,
                family,
            )
        };
        let start_order = compare(range.start())?;
        let end_order = compare(range.end())?;
        if start_order == Ordering::Less || end_order == Ordering::Greater {
            return Err(evaluation_error(family, CurveError::InvalidCurveParameter));
        }

        let reversed = match fragment {
            BezierSplitFragment2::RetainedBezier { reversed, .. } => *reversed,
            BezierSplitFragment2::AnalyticParallel(fragment) => fragment.is_reversed(),
            BezierSplitFragment2::SelectedFiber(fragment) => fragment.is_reversed(),
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                fragment
                    .validate_policy(policy)
                    .map_err(|cause| evaluation_error(family, cause))?;
                fragment.is_reversed()
            }
            BezierSplitFragment2::AlgebraicChord(chord) => {
                chord
                    .validate_policy(policy)
                    .map_err(|cause| evaluation_error(family, cause))?;
                let local = parameter
                    .as_algebraic_chord()
                    .ok_or_else(|| evaluation_error(family, CurveError::InvalidCurveParameter))?;
                // A parameter certified on this identical finite chord
                // already owns incidence and domain membership. Parameters
                // transported from another chord still need the predicate.
                if local.is_endpoint_of(chord, true)
                    || local.is_endpoint_of(chord, false)
                    || local.is_certified_strict_interior_of(chord)
                {
                    return Ok(local.point().clone());
                }
                if !decided(
                    chord
                        .contains_point(local.point(), policy)
                        .map_err(|cause| evaluation_error(family, cause))?,
                    family,
                )? {
                    return Err(evaluation_error(family, CurveError::InvalidCurveParameter));
                }
                return Ok(local.point().clone());
            }
            BezierSplitFragment2::Materialized { .. } => unreachable!("native curve"),
        };

        // Endpoints retain the exact join witnesses accepted at construction.
        // The range is a source chart: reversing traversal does not negate or
        // rebuild a selected scalar to make it look like a new unit parameter.
        if start_order == Ordering::Equal || end_order == Ordering::Equal {
            return Ok(self.endpoint((start_order == Ordering::Equal) != reversed));
        }
        let point = match fragment {
            BezierSplitFragment2::RetainedBezier { source_curve, .. } => {
                let source = RationalBezier2::try_from_subcurve(source_curve)
                    .map_err(|cause| evaluation_error(family, cause))?;
                return rational_point(&source, parameter, family, policy);
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => {
                parallel_point(fragment.parallel(), parameter, range, family, policy)?
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                if let Some(source) = fragment.rational_curve() {
                    return rational_point(source, parameter, family, policy);
                }
                parallel_point(
                    fragment.analytic_parallel().expect("analytic source"),
                    parameter,
                    range,
                    family,
                    policy,
                )?
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                let local = parameter
                    .as_algebraic_cusp()
                    .filter(|_| !parameter.is_algebraic_cusp_complement())
                    .ok_or_else(|| evaluation_error(family, CurveError::InvalidCurveParameter))?;
                let evidence = if let Some(source) = local
                    .mapped_semicircle_carrier()
                    .filter(|source| *source != fragment.semicircle())
                {
                    local.concentric_offset_point_evidence(source, fragment.semicircle(), policy)
                } else {
                    local.coincident_point_evidence(fragment.semicircle(), policy)
                }
                .map_err(|cause| evaluation_error(family, cause))?;
                decided(evidence, family)?.ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Evaluation,
                        family,
                        UncertaintyReason::Unsupported,
                    )
                })?
            }
            BezierSplitFragment2::Materialized { .. } | BezierSplitFragment2::AlgebraicChord(_) => {
                unreachable!()
            }
        };
        Ok(point)
    }

    /// Derivatives of a generated carrier in its retained source chart.
    ///
    /// The public parameter of a generated carrier is its source chart, so no
    /// chain-rule factor applies. Rational sources evaluate every order at
    /// represented and selected parameters; analytic parallels give first
    /// derivatives at represented and selected source parameters, and exact
    /// chords their constant first derivative. Other carriers and orders
    /// report the unsupported capability rather than a guess.
    pub(super) fn retained_derivatives_at(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveVector2>> {
        let family = self.family();
        let unsupported = || {
            ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                family,
                UncertaintyReason::Unsupported,
            )
        };
        let fragment = self.retained_fragment().expect("retained curve");
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            chord
                .validate_policy(policy)
                .map_err(|cause| evaluation_error(family, cause))?;
            let line = chord.exact_line().ok_or_else(unsupported)?;
            // A chord is affine in its parameter: constant first derivative.
            let first = CurveDerivative2::new(
                line.end().x() - line.start().x(),
                line.end().y() - line.start().y(),
            );
            return Ok((1..=max_order)
                .map(|order| {
                    CurveVector2::represented(if order == 1 {
                        first.clone()
                    } else {
                        CurveDerivative2::new(Real::zero(), Real::zero())
                    })
                })
                .collect());
        }
        let range = self.parameter_domain();
        let compare = |boundary| {
            decided(
                parameter
                    .cmp_by_refinement(boundary, policy)
                    .map_err(|cause| evaluation_error(family, cause))?,
                family,
            )
        };
        if compare(range.start())? == Ordering::Less || compare(range.end())? == Ordering::Greater {
            return Err(evaluation_error(family, CurveError::InvalidCurveParameter));
        }
        let parallel = match fragment {
            BezierSplitFragment2::RetainedBezier { source_curve, .. } => {
                let source = RationalBezier2::try_from_subcurve(source_curve)
                    .map_err(|cause| evaluation_error(family, cause))?;
                return rational_derivatives(&source, parameter, max_order, family, policy);
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                if let Some(source) = fragment.rational_curve() {
                    return rational_derivatives(source, parameter, max_order, family, policy);
                }
                fragment.analytic_parallel().ok_or_else(unsupported)?
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => fragment.parallel(),
            BezierSplitFragment2::AlgebraicCuspSemicircle(_) => return Err(unsupported()),
            BezierSplitFragment2::Materialized { .. } | BezierSplitFragment2::AlgebraicChord(_) => {
                unreachable!("native curves and chords are handled above")
            }
        };
        if max_order != 1 {
            return Err(unsupported());
        }
        let Some(scalar) = parameter.scalar() else {
            return selected_parallel_derivative(parallel, parameter, family, policy);
        };
        let derivative = decided(
            parallel
                .derivative_at_with_policy(scalar, policy)
                .map_err(|cause| evaluation_error(family, cause))?,
            family,
        )?;
        Ok(vec![CurveVector2::represented(derivative)])
    }

    fn point_at_selected_native_parameter(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        let family = self.family();
        if parameter.as_bezier_parameter().is_none() && !parameter.is_retained_scalar() {
            return Err(evaluation_error(family, CurveError::InvalidCurveParameter));
        }
        let fragments =
            self.native_bezier_fragments_for_operation(policy, CurveOperation2::Evaluation)?;
        for (fragment_index, fragment) in fragments.iter().enumerate() {
            let (start, end) = fragment.parameter_range();
            let compare = |boundary: &Real| {
                decided(
                    parameter
                        .cmp_by_refinement(&CurveParameter2::from(boundary.clone()), policy)
                        .map_err(|cause| evaluation_error(family, cause))?,
                    family,
                )
            };
            let start_order = compare(start)?;
            let end_order = compare(end)?;
            // At knots the authored evaluator owns multiplicity and side rules.
            if start_order == Ordering::Equal || end_order == Ordering::Equal {
                let knot = if start_order == Ordering::Equal {
                    start
                } else {
                    end
                };
                return self
                    .point_at_side_raw(knot, side, policy)
                    .map(CurvePoint2::from);
            }
            if start_order != Ordering::Greater || end_order != Ordering::Less {
                continue;
            }
            let scale = (Real::one() / (end - start))
                .map_err(|cause| evaluation_error(family, cause.into()))?;
            let local = decided(
                parameter
                    .affine_image_unbounded(&scale, &(-start * &scale), policy)
                    .map_err(|cause| evaluation_error(family, cause))?,
                family,
            )?;
            let source = &self
                .rational_evaluators_for_operation(policy, CurveOperation2::Evaluation)?
                [fragment_index];
            return rational_point(source, &local, family, policy);
        }
        Err(evaluation_error(family, CurveError::InvalidCurveParameter))
    }
}

fn validate_rational_point(
    source: &RationalBezier2,
    parameter: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    // Common-sign Bernstein weights already prove a nonzero denominator on
    // the closed unit span. Replaying its power polynomial in an unrelated
    // selected field can multiply refinement and coefficient costs even for
    // a quadratic NURBS span. The certificate says nothing about exterior
    // parameters, which must continue through the original polynomial proof.
    let strict = policy.strict_counterpart();
    if matches!(
        source.control_weight_sign(),
        Classification::Decided(RealSign::Positive | RealSign::Negative)
    ) && matches!(
        CurveParameterDomain2::new(&CurveParameterRange2::unit(), None)
            .contains_finite_parameter(parameter, &strict)
            .map_err(|cause| evaluation_error(family, cause))?,
        Classification::Decided(true)
    ) {
        return Ok(());
    }
    let basis = source
        .homogeneous_power_basis()
        .map_err(|cause| evaluation_error(family, cause))?;
    match decided(
        parameter
            .polynomial_sign(&basis.weight, policy)
            .map_err(|cause| evaluation_error(family, cause))?,
        family,
    )? {
        RealSign::Positive | RealSign::Negative => Ok(()),
        RealSign::Zero => Err(evaluation_error(family, CurveError::InvalidCurveParameter)),
    }
}

pub(super) fn rational_point(
    source: &RationalBezier2,
    parameter: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    validate_rational_point(source, parameter, family, policy)?;
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return match parameter {
            BezierParameter2::Exact(parameter) => {
                // The caller checked the retained range, which can extend
                // beyond the source's original unit chart. The denominator
                // proof above is the remaining affine evaluation condition.
                decided(source.point_at_affine_classified(parameter, policy), family)
                    .map(CurvePoint2::from)
            }
            BezierParameter2::Algebraic(parameter) => Ok(CurvePoint2::from(
                crate::RationalBezierAlgebraicPointImage2::from_parametric_source(
                    source.clone(),
                    parameter.clone(),
                    policy,
                ),
            )),
        };
    }
    let parallel = source
        .parallel_left(Real::zero())
        .map_err(|cause| evaluation_error(family, cause))?;
    crate::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
        parallel,
        parameter,
        Real::zero(),
        policy,
    )
    .map(CurvePoint2::from)
    .ok_or_else(|| {
        ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            family,
            UncertaintyReason::Unsupported,
        )
    })
}

/// First derivative of an analytic parallel at a selected source parameter.
///
/// The parallel shares its source's chart, and its derivative is the source
/// velocity scaled by the exact speed ratio whose sign the selected field
/// certifies. Other parameter families remain unsupported.
fn selected_parallel_derivative(
    parallel: &crate::BezierParallel2,
    parameter: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<CurveVector2>> {
    let unsupported = || {
        ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            family,
            UncertaintyReason::Unsupported,
        )
    };
    let Some(BezierParameter2::Algebraic(local)) = parameter.as_bezier_parameter() else {
        return Err(unsupported());
    };
    let source = match parallel.source() {
        crate::BezierParallelSource2::Quadratic(source) => RationalBezier2::try_new(
            source.control_points().map(Clone::clone).to_vec(),
            vec![Real::one(); 3],
        ),
        crate::BezierParallelSource2::Cubic(source) => RationalBezier2::try_new(
            source.control_points().map(Clone::clone).to_vec(),
            vec![Real::one(); 4],
        ),
        crate::BezierParallelSource2::Rational(source) => Ok(source.clone()),
    }
    .map_err(|cause| evaluation_error(family, cause))?;
    validate_rational_point(&source, parameter, family, policy)?;
    let images = decided(
        source
            .derivatives_at_algebraic_parameter(local, 2, policy)
            .map_err(|cause| evaluation_error(family, cause))?,
        family,
    )?;
    let velocity = crate::BezierAlgebraicTangentVector2::from_image(&images[0]);
    let acceleration = crate::BezierAlgebraicTangentVector2::from_image(&images[1]);
    let ratio_sign = decided(
        crate::bezier_tangent_order::parallel_speed_ratio_sign(
            &velocity,
            &acceleration,
            parallel.distance(),
        ),
        family,
    )?;
    Ok(vec![CurveVector2::selected_parallel(
        velocity,
        acceleration,
        parallel.distance().clone(),
        ratio_sign,
    )])
}

fn rational_derivatives(
    source: &RationalBezier2,
    parameter: &CurveParameter2,
    max_order: usize,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<CurveVector2>> {
    validate_rational_point(source, parameter, family, policy)?;
    if let Some(scalar) = parameter.scalar() {
        return Ok(decided(
            source.derivatives_at_classified(scalar, max_order, policy),
            family,
        )?
        .into_iter()
        .map(CurveVector2::represented)
        .collect());
    }
    let Some(BezierParameter2::Algebraic(local)) = parameter.as_bezier_parameter() else {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            family,
            UncertaintyReason::Unsupported,
        ));
    };
    let images = decided(
        source
            .derivatives_at_algebraic_parameter(local, max_order, policy)
            .map_err(|cause| evaluation_error(family, cause))?,
        family,
    )?;
    Ok(images
        .iter()
        .map(|image| {
            CurveVector2::selected(crate::BezierAlgebraicTangentVector2::from_image(image))
        })
        .collect())
}

fn parallel_point(
    parallel: &BezierParallel2,
    parameter: &CurveParameter2,
    range: &CurveParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    if let BezierParallelSource2::Rational(source) = parallel.source() {
        validate_rational_point(source, parameter, family, policy)?;
    }
    decided(
        parallel
            .point_evidence_on_regular_range(parameter, range, policy)
            .map_err(|cause| evaluation_error(family, cause))?,
        family,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BezierAlgebraicChord2, BezierAlgebraicParameter2, BezierParallelFragment2,
        BezierParameterInterval, BezierParameterPolynomial, CurveCertainty,
    };

    fn selected_half_root_two() -> CurveParameter2 {
        let policy = CurveContext::STRICT;
        let family = CurveFamily2::RationalBezier;
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis_with_policy(
                vec![-Real::one(), Real::zero(), Real::from(2)],
                &policy,
            )
            .unwrap(),
            family,
        )
        .unwrap();
        let interval = decided(
            BezierParameterInterval::try_new_with_policy(
                (Real::one() / Real::from(2)).unwrap(),
                Real::one(),
                &policy,
            )
            .unwrap(),
            family,
        )
        .unwrap();
        CurveParameter2::from(BezierParameter2::Algebraic(
            decided(
                BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, &policy)
                    .unwrap(),
                family,
            )
            .unwrap(),
        ))
    }

    fn assert_point(point: &CurvePoint2, expected: Point2, policy: &CurveContext) {
        let equality = point.coincides_with(&CurvePoint2::from(expected), policy);
        assert_eq!(equality.certainty, CurveCertainty::Certified);
        assert_eq!(equality.value, Classification::Decided(true));
    }

    #[test]
    fn positive_weights_do_not_authorize_an_exterior_rational_pole() {
        let source = RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 1)],
            vec![Real::one(), Real::from(2)],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // W(t)=1+t is strictly positive on [0,1], with a pole at -1.
            assert!(matches!(
                rational_point(
                    &source,
                    &Real::from(-1).into(),
                    CurveFamily2::RationalBezier,
                    &policy
                ),
                Err(ExactCurveError::Invalid {
                    cause: CurveError::InvalidCurveParameter,
                    ..
                })
            ));
            let regular = rational_point(
                &source,
                &Real::from(-2).into(),
                CurveFamily2::RationalBezier,
                &policy,
            )
            .unwrap();
            assert_point(&regular, Point2::from_values(4, 4), &policy);

            // A retained chart can be regular entirely outside [0,1]. Public
            // evaluation must use that finite range, including after reversal.
            let end = decided(
                selected_half_root_two()
                    .affine_image_unbounded(
                        &(Real::one() / Real::from(2)).unwrap(),
                        &Real::from(-2),
                        &policy,
                    )
                    .unwrap(),
                CurveFamily2::RationalBezier,
            )
            .unwrap();
            let end_point =
                rational_point(&source, &end, CurveFamily2::RationalBezier, &policy).unwrap();
            let curve = Curve2::from_retained_fragment(BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    crate::bezier_split::BezierSelectedFiberSource2::Rational(source.clone()),
                    CurveParameterRange2::new_validated(Real::from(-2).into(), end),
                    regular,
                    end_point,
                ),
            ));
            for curve in [
                curve.clone(),
                curve.reversed_with_policy(&policy).unwrap().value,
            ] {
                let point = curve
                    .point_at_with_policy(
                        &(Real::from(-7) / Real::from(4)).unwrap().into(),
                        &policy,
                    )
                    .unwrap();
                let coordinate = (Real::from(14) / Real::from(3)).unwrap();
                assert_point(
                    &point.value,
                    Point2::new(coordinate.clone(), coordinate),
                    &policy,
                );
                assert!(matches!(
                    curve.point_at_with_policy(&Real::from(-3).into(), &policy),
                    Err(ExactCurveError::Invalid {
                        cause: CurveError::InvalidCurveParameter,
                        ..
                    })
                ));
            }
        }
    }

    #[test]
    fn selected_native_evaluation_preserves_exact_curve_images_and_rejects_poles() {
        let parameter = selected_half_root_two();
        let source = Curve2::from(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new((Real::one() / Real::from(2)).unwrap(), Real::zero()),
            Point2::from_values(1, 1),
        ));
        let pole = Curve2::from(
            RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, 0),
                    Point2::from_values(1, 2),
                    Point2::from_values(2, 3),
                ],
                vec![Real::one(), Real::one(), -Real::one()],
            )
            .unwrap(),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let point = source.point_at_with_policy(&parameter, &policy).unwrap();
            assert_eq!(point.certainty, CurveCertainty::Certified);
            assert_point(
                &point.value,
                Point2::new(
                    (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap(),
                    (Real::one() / Real::from(2)).unwrap(),
                ),
                &policy,
            );
            assert!(matches!(
                pole.point_at_with_policy(&parameter, &policy),
                Err(ExactCurveError::Invalid {
                    cause: CurveError::InvalidCurveParameter,
                    ..
                })
            ));
        }
    }

    #[test]
    fn selected_major_arc_evaluation_keeps_the_full_authored_chart() {
        let parameter = selected_half_root_two();
        let represented = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
        let arc = Curve2::from(
            CircularArc2::try_from_center(
                Point2::from_values(1, 0),
                Point2::new(
                    (Real::from(3) / Real::from(5)).unwrap(),
                    (Real::from(4) / Real::from(5)).unwrap(),
                ),
                Point2::from_values(0, 0),
                true,
            )
            .unwrap(),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let point = arc.point_at_with_policy(&parameter, &policy).unwrap();
            assert_eq!(point.certainty, CurveCertainty::Certified);
            let spans = arc
                .native_bezier_fragments_for_operation(&policy, CurveOperation2::Evaluation)
                .unwrap();
            let (index, span) = spans
                .iter()
                .enumerate()
                .find(|(_, span)| {
                    let (start, end) = span.parameter_range();
                    crate::classify::compare_reals(start, &represented, &policy)
                        == Some(Ordering::Less)
                        && crate::classify::compare_reals(&represented, end, &policy)
                            == Some(Ordering::Less)
                })
                .unwrap();
            assert!(
                index > 0,
                "the query must cross an internal arc chart boundary"
            );
            let (start, end) = span.parameter_range();
            let local = ((&represented - start) / (end - start)).unwrap();
            let image = point.value.as_algebraic().unwrap();
            assert_eq!(
                BezierParameter2::Algebraic(image.retained_parameter().unwrap().clone())
                    .same_value(&BezierParameter2::Exact(local), &policy)
                    .unwrap(),
                Classification::Decided(true)
            );
            let source = RationalBezier2::try_from_subcurve(span.native_curve()).unwrap();
            let basis = source.homogeneous_power_basis().unwrap();
            assert_eq!(
                image.retained_coordinate_polynomials(),
                Some((
                    basis.x_numerator.as_slice(),
                    basis.y_numerator.as_slice(),
                    basis.weight.as_slice()
                ))
            );
            // Rational chart partitioning also permits exact replay against
            // independently expanded scalar coordinates. The previous
            // square-root chart partition returned Unsupported here.
            let expanded = arc
                .point_at_with_policy(&represented.clone().into(), &policy)
                .unwrap();
            assert_eq!(expanded.certainty, CurveCertainty::Certified);
            let equality = point.value.coincides_with(&expanded.value, &policy);
            assert_eq!(equality.certainty, CurveCertainty::Certified);
            assert_eq!(equality.value, Classification::Decided(true));
        }
    }

    #[test]
    fn rational_cusp_endpoint_evaluation_retains_its_regular_branch_frame() {
        let third = (Real::one() / Real::from(3)).unwrap();
        let two_thirds = &third * Real::from(2);
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let source = RationalBezier2::try_new(
            vec![
                Point2::from_values(1, -1),
                Point2::new(-third.clone(), Real::one()),
                Point2::new(-third, -Real::one()),
                Point2::from_values(1, 1),
            ],
            vec![Real::from(8), Real::from(4), Real::from(2), Real::one()],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parallel = BezierParallel2::from_source(
                BezierParallelSource2::Rational(source.clone()),
                quarter.clone(),
            );
            let fragment = BezierParallelFragment2::from_certified_range(
                parallel,
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(two_thirds.clone()),
                    BezierParameter2::Exact(Real::one()),
                ),
                false,
            );
            let curve =
                Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(fragment));
            let expected = Point2::new(Real::zero(), quarter.clone());
            let point = curve
                .point_at_with_policy(curve.parameter_domain().start(), &policy)
                .unwrap();
            assert_eq!(point.certainty, CurveCertainty::Certified);
            assert_point(&point.value, expected.clone(), &policy);
            let reversed = curve.reversed_with_policy(&policy).unwrap().value;
            let point = reversed
                .point_at_with_policy(curve.parameter_domain().start(), &policy)
                .unwrap();
            assert_point(&point.value, expected, &policy);
            assert!(point.value.shares_storage(&reversed.end()));
        }
    }

    #[test]
    fn point_order_from_a_parallel_chord_is_not_an_incident_parameter() {
        let policy = CurveContext::STRICT;
        let chord = |y| {
            decided(
                BezierAlgebraicChord2::try_new(
                    Point2::from_values(0, y).into(),
                    Point2::from_values(2, y).into(),
                    &policy,
                )
                .unwrap(),
                CurveFamily2::Line,
            )
            .unwrap()
        };
        let foreign = chord(1);
        let curve = Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(chord(0)));
        let parameter = CurveParameter2::from_algebraic_chord(foreign.start_parameter());
        assert!(curve.point_at_with_policy(&parameter, &policy).is_err());
    }

    #[test]
    fn repeated_unit_chart_evaluation_reuses_selected_root_images() {
        let parameter = selected_half_root_two();
        let BezierParameter2::Algebraic(root) = parameter.as_bezier_parameter().unwrap() else {
            unreachable!();
        };
        let source = Curve2::from(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(2, 2),
        ));
        let policy = CurveContext::STRICT;
        let rational = &source
            .rational_evaluators_for_operation(&policy, CurveOperation2::Evaluation)
            .unwrap()[0];
        let cached = crate::tests::decided(
            rational
                .point_at_algebraic_parameter(root, &policy)
                .unwrap(),
        );
        for _ in 0..16 {
            let point = source
                .point_at_with_policy(&parameter, &policy)
                .unwrap()
                .value;
            let image = point.as_algebraic().unwrap().resolved(&policy).unwrap();
            assert!(image.shares_storage(&cached));
        }
    }

    #[test]
    fn selected_fiber_evaluation_keeps_a_local_field_beyond_global_projection_budget() {
        let policy = CurveContext::STRICT;
        let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            (Real::one() / Real::from(2)).unwrap(),
            2,
            &policy,
        );
        let parameter = CurveParameter2::from_selected_fiber(selected);
        let curve = Curve2::from(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let point = curve.point_at_with_policy(&parameter, &policy).unwrap();
            assert_eq!(point.certainty, CurveCertainty::Certified);
            for (bound, expected) in [(0, Ordering::Greater), (1, Ordering::Less)] {
                let order = point
                    .value
                    .compare_coordinate(
                        &Point2::from_values(bound, 0).into(),
                        crate::Axis2::X,
                        &policy,
                    )
                    .unwrap();
                assert_eq!(order.certainty, CurveCertainty::Certified);
                assert_eq!(order.value, Classification::Decided(expected));
            }
        }
    }

    #[test]
    fn selected_knot_evaluation_preserves_discontinuous_spline_side_rules() {
        let policy = CurveContext::STRICT;
        let BezierParameter2::Algebraic(root) = selected_half_root_two()
            .as_bezier_parameter()
            .unwrap()
            .clone()
        else {
            unreachable!();
        };
        let parameter = CurveParameter2::from_selected_fiber(
            crate::bezier_offset::exact_selected_fiber_parameter_for_test(
                root,
                Real::one(),
                &policy,
            ),
        );
        let curve = Curve2::from(
            crate::PolynomialSplineCurve2::try_new_with_policy(
                2,
                vec![
                    Point2::from_values(0, 0),
                    Point2::from_values(1, 1),
                    Point2::from_values(2, 0),
                    Point2::from_values(10, 0),
                    Point2::from_values(11, 1),
                    Point2::from_values(12, 0),
                ],
                vec![0, 0, 0, 1, 1, 1, 2, 2, 2]
                    .into_iter()
                    .map(Real::from)
                    .collect(),
                &policy,
            )
            .unwrap()
            .value,
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(matches!(curve.point_at_with_policy(&parameter, &policy),
                Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::Boundary));
            for (side, x) in [
                (CurveParameterSide2::Left, 2),
                (CurveParameterSide2::Right, 10),
            ] {
                let point = curve
                    .point_at_side_with_policy(&parameter, side, &policy)
                    .unwrap();
                assert_eq!(point.certainty, CurveCertainty::Certified);
                assert_point(&point.value, Point2::from_values(x, 0), &policy);
            }
        }
    }
}
