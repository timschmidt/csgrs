//! Exact geometric supports shared by range restriction and arrangements.
//!
//! A support owns curve equations and their retained coefficient evidence.
//! Selected circles also carry chart and endpoint-tangency evidence. Active
//! ranges, traversal and region fill semantics belong to the caller; restricting
//! a support reuses its coefficient field and surviving endpoint evidence.

use crate::bezier_split::{
    BezierSelectedFiberFragment2, BezierSelectedFiberSource2, CurveParameterDomain2,
};
use crate::{
    Aabb2, BezierParallel2, BezierParameterRange2, BezierSplitFragment2, BezierSubcurve2,
    Classification, CurveContext, CurveDerivative2, CurveError, CurveFamily2, CurveParameterRange2,
    CurvePoint2, CurveResult, RationalBezier2, UncertaintyReason,
};
use std::sync::Arc;

#[derive(Clone, Debug)]
pub(crate) enum CurveSupport2 {
    Bezier(BezierSubcurve2),
    Parallel(BezierParallel2),
    Line(crate::BezierAlgebraicChord2),
    Circle(crate::BezierAlgebraicCuspSemicircleFragment2),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
        BezierParameterPolynomial, Curve2, CurveCertainty, Point2, QuadraticBezier2, Real,
    };

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("expected exact evidence: {reason:?}"),
        }
    }

    #[test]
    fn injectivity_certificates_own_their_domain_and_circle_chart() {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let cubic = CurveSupport2::Bezier(BezierSubcurve2::Cubic(crate::CubicBezier2::new(
                Point2::from_values(-1, 0),
                Point2::new((-1).into(), q(-1, 3)),
                Point2::new(q(-2, 3), q(-2, 3)),
                Point2::from_values(0, 0),
            )));
            assert!(cubic.has_certified_injective_image(&CurveParameterRange2::unit(), &policy));
            for t in [-2, -1, 0, 1, 2] {
                let derivative = decided(cubic.derivative_at(&Real::from(t), &policy).unwrap());
                assert_eq!(derivative.dx(), &Real::from(2 * t));
                assert_eq!(derivative.dy(), &Real::from(3 * t * t - 1));
            }

            for (start, end) in [(-2, 2), (2, -2)] {
                let range = CurveParameterRange2::new_validated(
                    Real::from(start).into(),
                    Real::from(end).into(),
                );
                assert!(!cubic.has_certified_injective_image(&range, &policy));
            }
            let arc = crate::CircularArc2::try_from_center(
                Point2::from_values(1, 0),
                Point2::from_values(0, 1),
                Point2::from_values(0, 0),
                false,
            )
            .unwrap();
            let (implicit, circle) = crate::arc_bezier::circular_conic_provenance(&arc);
            let quadratic = RationalBezier2::try_new(
                vec![
                    Point2::from_values(1, 0),
                    Point2::from_values(1, 1),
                    Point2::from_values(0, 1),
                ],
                vec![Real::one(), q(1, 2).sqrt().unwrap(), Real::one()],
            )
            .unwrap()
            .with_implicit_quadratic_conic(implicit, Some(circle));
            let elevated = BezierSubcurve2::Rational(quadratic.elevated_to_degree(5).unwrap());
            assert!(elevated.has_certified_injective_image(&policy));
            let collapsed = decided(
                quadratic
                    .subcurve_between_exact(&q(1, 2), &q(1, 2), &policy)
                    .unwrap(),
            );
            assert!(collapsed.retained_circular_conic().is_some());
            assert!(!BezierSubcurve2::Rational(collapsed).has_certified_injective_image(&policy));
            // s=4t(1-t), C(s)=((1-s²)/(1+s²),2s/(1+s²)) retraces
            // a circular arc. Its genuine circle equation is not injectivity.
            let retraced = RationalBezier2::try_new(
                vec![
                    Point2::from_values(1, 0),
                    Point2::from_values(1, 2),
                    Point2::new(q(-5, 11), q(8, 11)),
                    Point2::from_values(1, 2),
                    Point2::from_values(1, 0),
                ],
                vec![Real::one(), Real::one(), q(11, 3), Real::one(), Real::one()],
            )
            .unwrap()
            .with_implicit_quadratic_conic(
                quadratic
                    .retained_implicit_quadratic_conic()
                    .unwrap()
                    .clone(),
                quadratic.retained_circular_conic().cloned(),
            );
            let retraced = BezierSubcurve2::Rational(retraced);
            assert!(!retraced.has_certified_injective_image(&policy));
            assert_eq!(
                retraced.point_at(&q(1, 4), &policy),
                retraced.point_at(&q(3, 4), &policy)
            );
        }
    }

    #[test]
    fn exterior_bounds_include_interior_extrema_and_parallel_displacement() {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        // P(t) = (t, (t-2)^2) has its minimum outside the native unit chart.
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 4),
            Point2::new(q(1, 2), 2.into()),
            Point2::from_values(1, 1),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (start, end) in [(1, 3), (3, 1)] {
                let range = CurveParameterRange2::new_validated(
                    Real::from(start).into(),
                    Real::from(end).into(),
                );
                for distance in [-1, 0, 1] {
                    let support = if distance == 0 {
                        CurveSupport2::Bezier(BezierSubcurve2::Quadratic(source.clone()))
                    } else {
                        CurveSupport2::Parallel(source.parallel_left(q(distance, 4)).unwrap())
                    };
                    let bounds = decided(support.certified_outer_bounds(&range, 0, &policy));
                    for n in 4..=12 {
                        let point = decided(support.point_at(&q(n, 4), &policy).unwrap());
                        assert_eq!(
                            bounds.contains_point(&point, &policy),
                            Classification::Decided(true)
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn finite_bounds_exclude_poles_elsewhere_in_the_native_chart() {
        let source = RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 1)],
            vec![Real::one(), -Real::one()],
        )
        .unwrap();
        assert!(matches!(
            source.certified_bounds_classified(),
            Classification::Uncertain(_)
        ));
        let supports = [
            CurveSupport2::Bezier(BezierSubcurve2::Rational(source.clone())),
            CurveSupport2::Parallel(source.parallel_left(Real::one()).unwrap()),
        ];
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        for support in supports {
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                for (start, end) in [(q(0, 1), q(1, 4)), (q(3, 4), q(1, 1))] {
                    let range = CurveParameterRange2::new_validated(
                        start.clone().into(),
                        end.clone().into(),
                    );
                    let bounds = decided(support.certified_outer_bounds(&range, 0, &policy));
                    for parameter in [&start, &end] {
                        let point = decided(support.point_at(parameter, &policy).unwrap());
                        assert_eq!(
                            bounds.contains_point(&point, &policy),
                            Classification::Decided(true)
                        );
                    }
                }
                let crossing_pole =
                    CurveParameterRange2::new_validated(q(1, 4).into(), q(3, 4).into());
                assert!(matches!(
                    support.certified_outer_bounds(&crossing_pole, 0, &policy),
                    Classification::Uncertain(_)
                ));
            }
        }
    }

    #[test]
    fn certified_bezier_ranges_preserve_source_chart_and_traversal() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = QuadraticBezier2::new(
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(2, 2),
            );
            let support = CurveSupport2::Bezier(BezierSubcurve2::Quadratic(source.clone()));
            let source = Curve2::from(source);
            let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
            let polynomial = decided(
                BezierParameterPolynomial::try_new_power_basis(
                    vec![(-1).into(), 0.into(), 2.into()],
                    &policy,
                )
                .unwrap(),
            );
            let interval =
                decided(BezierParameterInterval::try_new(q(1, 2), Real::one(), &policy).unwrap());
            let root = decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
            );
            for range in [
                CurveParameterRange2::unit(),
                CurveParameterRange2::new_validated(q(1, 4).into(), q(3, 4).into()),
                CurveParameterRange2::new_validated(
                    BezierParameter2::Algebraic(root.clone()).into(),
                    Real::one().into(),
                ),
            ] {
                for reversed in [false, true] {
                    let fragment = support
                        .restrict_certified(range.clone(), None, reversed, &policy)
                        .unwrap();
                    if let BezierSplitFragment2::RetainedBezier {
                        start_image,
                        end_image,
                        ..
                    } = &fragment
                    {
                        assert!(
                            start_image
                                .iter()
                                .chain(end_image)
                                .all(|image| image.is_lazy_first_order())
                        );
                    }
                    let curve = Curve2::from_retained_fragment(fragment);
                    assert_eq!(curve.parameter_domain(), &range);
                    let midpoint = if range
                        .start()
                        .as_bezier_parameter()
                        .unwrap()
                        .scalar()
                        .is_some()
                    {
                        q(1, 2)
                    } else {
                        q(7, 8)
                    };
                    for parameter in [range.start().clone(), midpoint.into(), range.end().clone()] {
                        let actual = curve.point_at(&parameter, &policy).unwrap();
                        let expected = source.point_at(&parameter, &policy).unwrap();
                        assert_eq!(actual.certainty, CurveCertainty::Certified);
                        assert_eq!(expected.certainty, CurveCertainty::Certified);
                        assert_eq!(
                            actual.value.same_point(&expected.value, &policy),
                            Classification::Decided(true)
                        );
                    }
                    let (start, end) = if reversed {
                        (range.end(), range.start())
                    } else {
                        (range.start(), range.end())
                    };
                    for (point, parameter) in [(curve.start(), start), (curve.end(), end)] {
                        assert_eq!(
                            point.same_point(
                                &source.point_at(parameter, &policy).unwrap().value,
                                &policy
                            ),
                            Classification::Decided(true)
                        );
                    }
                }
            }
        }
    }
}

fn subcurve_certified_outer_bounds(
    curve: &BezierSubcurve2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let unit = CurveParameterRange2::unit();
    if matches!(
        CurveParameterDomain2::new(&unit, None).contains_finite_range(range, policy),
        Ok(Classification::Decided(true))
    ) {
        let bounds = native_subcurve_outer_bounds(curve);
        if matches!(bounds, Classification::Decided(_)) || range == &unit {
            return bounds;
        }
        // The full source may have a pole outside the active subrange.
        // A missing whole-unit box must not reject a finite restriction.
    }
    let discover = || -> CurveResult<Classification<Aabb2>> {
        let rational = RationalBezier2::try_from_subcurve(curve)?;
        let envelope = match rational.finite_discovery_envelope(range, policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (start, end) = envelope
            .scalar_endpoints()
            .expect("a finite discovery envelope has represented bounds");
        Ok(
            match curve.subcurve_between_affine_exact(start, end, policy)? {
                Classification::Decided(curve) => native_subcurve_outer_bounds(&curve),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    };
    discover().unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported))
}

fn native_subcurve_outer_bounds(curve: &BezierSubcurve2) -> Classification<Aabb2> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(),
    };
    if matches!(bounds, Classification::Decided(_)) {
        return bounds;
    }
    let Some((_, circle)) = retained_circular_support(curve) else {
        return bounds;
    };
    // Mixed-weight major-circle charts are finite even though their rational
    // control hull is not convex. Retained circular provenance certifies the
    // complete image lies in this exact full-circle envelope, which is a
    // conservative broad-phase fallback when quotient-extremum isolation did
    // not produce a tighter box.
    let radius = match circle.radius_squared.clone().sqrt() {
        Ok(radius) => radius,
        Err(_) => return bounds,
    };
    Classification::Decided(Aabb2::new_unchecked(
        crate::Point2::new(circle.center.x() - &radius, circle.center.y() - &radius),
        crate::Point2::new(circle.center.x() + &radius, circle.center.y() + &radius),
    ))
}

pub(crate) fn retained_circular_support(
    curve: &BezierSubcurve2,
) -> Option<(
    &Arc<[crate::Real; 6]>,
    &Arc<crate::rational_bezier::RationalQuadraticCircle2>,
)> {
    let (implicit, circular) = match curve {
        BezierSubcurve2::RationalQuadratic(curve) => (
            curve.retained_implicit_quadratic_conic(),
            curve.retained_circular_conic(),
        ),
        BezierSubcurve2::Rational(curve) => (
            curve.retained_implicit_quadratic_conic(),
            curve.retained_circular_conic(),
        ),
        BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => return None,
    };
    Some((implicit?, circular?))
}

const fn subcurve_family(curve: &BezierSubcurve2) -> CurveFamily2 {
    match curve {
        BezierSubcurve2::Quadratic(_) => CurveFamily2::QuadraticBezier,
        BezierSubcurve2::Cubic(_) => CurveFamily2::CubicBezier,
        BezierSubcurve2::RationalQuadratic(_) => CurveFamily2::RationalQuadraticBezier,
        BezierSubcurve2::Rational(_) => CurveFamily2::RationalBezier,
    }
}

impl CurveSupport2 {
    /// Borrows no region bookkeeping and retains the original support field.
    pub(crate) fn from_fragment(fragment: &BezierSplitFragment2) -> Self {
        match fragment {
            BezierSplitFragment2::Materialized { curve, .. }
            | BezierSplitFragment2::RetainedBezier {
                source_curve: curve,
                ..
            } => Self::Bezier(curve.clone()),
            BezierSplitFragment2::AnalyticParallel(fragment) => {
                Self::Parallel(fragment.parallel().clone())
            }
            BezierSplitFragment2::AlgebraicChord(chord) => Self::Line(chord.clone()),
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                Self::Circle(fragment.clone())
            }
            BezierSplitFragment2::SelectedFiber(fragment) => match fragment.source() {
                BezierSelectedFiberSource2::Rational(curve) => {
                    Self::Bezier(BezierSubcurve2::Rational(curve.clone()))
                }
                BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                    Self::Parallel(parallel.clone())
                }
            },
        }
    }

    /// Publishes a range whose strict order and membership in this support
    /// have already been certified by the calling operation.
    ///
    /// Endpoint images, when supplied, correspond to ascending source order
    /// and preserve the selected parameter-to-point relation. Ordinary Bezier,
    /// analytic, line and circle ranges retain lazy endpoint evaluators;
    /// selected-fiber ranges require their certified point images. Traversal
    /// is applied after restriction without changing the source chart.
    pub(crate) fn restrict_certified(
        &self,
        range: CurveParameterRange2,
        endpoint_images: Option<[CurvePoint2; 2]>,
        reversed: bool,
        policy: &CurveContext,
    ) -> CurveResult<BezierSplitFragment2> {
        let selected_source = match self {
            Self::Bezier(curve) if endpoint_images.is_some() => Some(
                BezierSelectedFiberSource2::Rational(RationalBezier2::try_from_subcurve(curve)?),
            ),
            Self::Parallel(parallel) if endpoint_images.is_some() => Some(
                BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
            ),
            _ => None,
        };
        let fragment = if let Some(source) = selected_source {
            let [start_point, end_point] = endpoint_images.ok_or_else(|| {
                CurveError::Topology(
                    "a selected rational restriction requires its certified endpoint images".into(),
                )
            })?;
            BezierSplitFragment2::SelectedFiber(BezierSelectedFiberFragment2::new(
                source,
                range,
                start_point,
                end_point,
            ))
        } else {
            match self {
                Self::Bezier(curve) => {
                    let (Some(start), Some(end)) = (
                        range.start().as_bezier_parameter(),
                        range.end().as_bezier_parameter(),
                    ) else {
                        return Err(CurveError::Topology(
                            "a selected rational restriction requires its certified endpoint images".into(),
                        ));
                    };
                    if !reversed && range == CurveParameterRange2::unit() {
                        return Ok(BezierSplitFragment2::Materialized {
                            start: start.clone(),
                            end: end.clone(),
                            curve: curve.clone(),
                        });
                    }
                    let endpoint = |parameter: &crate::BezierParameter2| match parameter {
                        crate::BezierParameter2::Exact(_) => Ok(None),
                        crate::BezierParameter2::Algebraic(parameter) => {
                            crate::BezierAlgebraicEndpointImage2::from_source_curve_first_order(
                                curve, parameter, policy,
                            )
                            .map(Some)
                        }
                    };
                    BezierSplitFragment2::RetainedBezier {
                        reversed: false,
                        start: start.clone(),
                        end: end.clone(),
                        source_curve: curve.clone(),
                        start_image: endpoint(start)?,
                        end_image: endpoint(end)?,
                    }
                }
                Self::Parallel(parallel) => {
                    let (Some(start), Some(end)) = (
                        range.start().as_bezier_parameter(),
                        range.end().as_bezier_parameter(),
                    ) else {
                        return Err(CurveError::Topology(
                            "a selected analytic restriction requires its certified endpoint images".into(),
                        ));
                    };
                    BezierSplitFragment2::AnalyticParallel(
                        crate::BezierParallelFragment2::from_certified_range(
                            parallel.clone(),
                            BezierParameterRange2::new_validated(start.clone(), end.clone()),
                            false,
                        ),
                    )
                }
                Self::Line(chord) => {
                    let (Some(start), Some(end)) = (
                        range.start().as_algebraic_chord(),
                        range.end().as_algebraic_chord(),
                    ) else {
                        return Err(CurveError::InvalidCurveParameter);
                    };
                    if start.is_endpoint_of(chord, true) && end.is_endpoint_of(chord, false) {
                        chord.validate_policy(policy)?;
                        let fragment = BezierSplitFragment2::AlgebraicChord(chord.clone());
                        return if reversed {
                            fragment.reversed()
                        } else {
                            Ok(fragment)
                        };
                    }
                    BezierSplitFragment2::AlgebraicChord(
                        crate::BezierAlgebraicChord2::from_certified_ordered_parameter_range(
                            chord, start, end, policy,
                        )?,
                    )
                }
                Self::Circle(source) => {
                    let (Some(start), Some(end)) = (
                        range.start().as_algebraic_cusp(),
                        range.end().as_algebraic_cusp(),
                    ) else {
                        return Err(CurveError::InvalidCurveParameter);
                    };
                    if start.shares_exact_evidence(source.start_parameter())
                        && end.shares_exact_evidence(source.end_parameter())
                    {
                        let fragment =
                            BezierSplitFragment2::AlgebraicCuspSemicircle(source.clone());
                        return if reversed == source.is_reversed() {
                            Ok(fragment)
                        } else {
                            fragment.reversed()
                        };
                    }
                    BezierSplitFragment2::AlgebraicCuspSemicircle(
                        crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                            source.semicircle().clone(),
                            start.clone(),
                            end.clone(),
                            false,
                            policy,
                        )
                        .inherit_certified_tangent_endpoints(source),
                    )
                }
            }
        };
        if reversed {
            fragment.reversed()
        } else {
            Ok(fragment)
        }
    }

    pub(crate) const fn family(&self) -> CurveFamily2 {
        match self {
            Self::Bezier(curve) => subcurve_family(curve),
            Self::Line(_) => CurveFamily2::Line,
            Self::Parallel(_) | Self::Circle(_) => CurveFamily2::RationalBezier,
        }
    }

    pub(crate) fn bezier(&self) -> &BezierSubcurve2 {
        match self {
            Self::Bezier(curve) => curve,
            Self::Parallel(_) => {
                unreachable!("parallel/rational dispatch requires a Bezier carrier")
            }
            Self::Line(_) | Self::Circle(_) => {
                unreachable!("cusp/rational dispatch requires a Bezier carrier")
            }
        }
    }

    pub(crate) fn parallel(&self) -> &BezierParallel2 {
        match self {
            Self::Parallel(parallel) => parallel,
            Self::Bezier(_) => {
                unreachable!("analytic pair dispatch requires a parallel carrier")
            }
            Self::Line(_) | Self::Circle(_) => {
                unreachable!("cusp/parallel dispatch requires a parallel carrier")
            }
        }
    }

    pub(crate) fn circle(&self) -> &crate::BezierAlgebraicCuspSemicircleFragment2 {
        match self {
            Self::Circle(fragment) => fragment,
            Self::Bezier(_) | Self::Parallel(_) | Self::Line(_) => {
                unreachable!("algebraic-cusp dispatch requires a cusp carrier")
            }
        }
    }

    pub(crate) fn point_at(
        &self,
        parameter: &crate::Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::Point2>> {
        match self {
            Self::Bezier(BezierSubcurve2::Rational(curve)) => {
                Ok(curve.point_at_affine_classified(parameter, policy))
            }
            Self::Bezier(curve) => Ok(curve.point_at(parameter, policy)),
            Self::Parallel(parallel) => parallel.point_at_affine(parameter, policy),
            Self::Line(chord) => match chord.exact_line() {
                Some(line) => Ok(Classification::Decided(line.point_at(parameter.clone()))),
                None => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            },
            Self::Circle(fragment) => {
                Ok(match fragment.semicircle().point_at(parameter, policy)? {
                    Classification::Decided(point) => point.exact_point(policy).map_or(
                        Classification::Uncertain(UncertaintyReason::Unsupported),
                        Classification::Decided,
                    ),
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                })
            }
        }
    }

    pub(crate) fn derivative_at(
        &self,
        parameter: &crate::Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveDerivative2>> {
        match self {
            Self::Bezier(curve) => RationalBezier2::try_from_subcurve(curve)
                .map(|curve| curve.derivative_at_affine_classified(parameter, policy)),
            Self::Parallel(parallel) => parallel.derivative_at(parameter, policy),
            Self::Line(chord) => match chord.exact_line() {
                Some(line) => Ok(Classification::Decided(CurveDerivative2::new(
                    line.end().x() - line.start().x(),
                    line.end().y() - line.start().y(),
                ))),
                None => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            },
            Self::Circle(_) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        }
    }

    /// Bounds the active finite image. Native hulls only certify their unit
    /// chart; exterior ranges use a pole-free represented enclosure.
    pub(crate) fn certified_outer_bounds(
        &self,
        range: &CurveParameterRange2,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        match self {
            Self::Bezier(curve) => subcurve_certified_outer_bounds(curve, range, policy),
            Self::Parallel(parallel) => {
                let unit = CurveParameterRange2::unit();
                if matches!(
                    CurveParameterDomain2::new(&unit, None).contains_finite_range(range, policy),
                    Ok(Classification::Decided(true))
                ) {
                    let bounds = parallel
                        .conservative_bounds()
                        .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported));
                    if matches!(bounds, Classification::Decided(_)) || range == &unit {
                        return bounds;
                    }
                }
                let source = match parallel.source() {
                    crate::BezierParallelSource2::Quadratic(curve) => {
                        BezierSubcurve2::Quadratic(curve.clone())
                    }
                    crate::BezierParallelSource2::Cubic(curve) => {
                        BezierSubcurve2::Cubic(curve.clone())
                    }
                    crate::BezierParallelSource2::Rational(curve) => {
                        BezierSubcurve2::Rational(curve.clone())
                    }
                };
                subcurve_certified_outer_bounds(&source, range, policy).map(|bounds| {
                    let radius = parallel.distance().abs();
                    Aabb2::new_unchecked(
                        crate::Point2::new(bounds.min_x() - &radius, bounds.min_y() - &radius),
                        crate::Point2::new(bounds.max_x() + &radius, bounds.max_y() + &radius),
                    )
                })
            }
            Self::Line(chord) => chord
                .conservative_bounds_refined(refinement_steps, policy)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            Self::Circle(fragment) => fragment
                .semicircle()
                .conservative_bounds_refined(refinement_steps, policy)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
        }
    }

    /// These native injectivity certificates own the unit chart. A missing
    /// certificate on the active range must leave self-incidence discovery live.
    pub(crate) fn has_certified_injective_image(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> bool {
        if matches!(self, Self::Line(_) | Self::Circle(_)) {
            return true;
        }
        let unit = CurveParameterRange2::unit();
        if !matches!(
            CurveParameterDomain2::new(&unit, None)
                .contains_finite_range(range, &policy.strict_counterpart()),
            Ok(Classification::Decided(true))
        ) {
            return false;
        }
        match self {
            Self::Bezier(curve) => curve.has_certified_injective_image(policy),
            Self::Parallel(parallel) => {
                parallel.regular_fragment_has_certified_injective_axis(range, policy)
                    || matches!(
                        parallel.exact_rational_parallel_component(policy),
                        Ok(Classification::Decided(Some(curve)))
                            if curve.has_certified_injective_axis(policy)
                    )
            }
            Self::Line(_) | Self::Circle(_) => unreachable!("intrinsically injective support"),
        }
    }

    pub(crate) fn exact_rational_component(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezier2>>> {
        match self {
            Self::Bezier(curve) => RationalBezier2::try_from_subcurve(curve)
                .map(Some)
                .map(Classification::Decided),
            Self::Parallel(parallel) => parallel.exact_rational_parallel_component(policy),
            Self::Line(_) => Ok(Classification::Decided(None)),
            Self::Circle(_) => Ok(Classification::Decided(None)),
        }
    }
}
