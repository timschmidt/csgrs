//! Exact geometric supports shared by range restriction and arrangements.
//!
//! A support owns curve equations and their retained coefficient evidence.
//! Selected circles also carry chart and endpoint-tangency evidence. Active
//! ranges, traversal and region fill semantics belong to the caller; restricting
//! a support reuses its coefficient field and surviving endpoint evidence.

use crate::bezier_split::{BezierSelectedFiberFragment2, BezierSelectedFiberSource2};
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

fn subcurve_certified_outer_bounds(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
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
            | BezierSplitFragment2::AlgebraicEndpointImages {
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
    /// and preserve the selected parameter-to-point relation. Rational spans
    /// require those images; ordinary analytic, line and circle ranges already
    /// retain their endpoint evaluators. Traversal is applied after restriction.
    pub(crate) fn restrict_certified(
        &self,
        range: CurveParameterRange2,
        endpoint_images: Option<[CurvePoint2; 2]>,
        reversed: bool,
        policy: &CurveContext,
    ) -> CurveResult<BezierSplitFragment2> {
        let selected_source = match self {
            Self::Bezier(curve) => Some(BezierSelectedFiberSource2::Rational(
                RationalBezier2::try_from_subcurve(curve)?,
            )),
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
                Self::Bezier(_) => unreachable!("rational range handled above"),
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
            Self::Bezier(curve) => Ok(curve.point_at(parameter, policy)),
            Self::Parallel(parallel) => parallel.point_at(parameter, policy),
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
                .map(|curve| curve.derivative_at_classified(parameter, policy)),
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

    pub(crate) fn certified_outer_bounds(&self, policy: &CurveContext) -> Classification<Aabb2> {
        match self {
            Self::Bezier(curve) => subcurve_certified_outer_bounds(curve, policy),
            Self::Parallel(parallel) => match parallel.conservative_bounds(policy) {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
            Self::Line(chord) => match chord.conservative_bounds(policy) {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
            Self::Circle(fragment) => match fragment.conservative_bounds() {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        }
    }

    pub(crate) fn certified_outer_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        match self {
            Self::Bezier(curve) => subcurve_certified_outer_bounds(curve, policy),
            Self::Parallel(parallel) => match parallel.conservative_bounds(policy) {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
            Self::Line(chord) => {
                match chord.conservative_bounds_refined(refinement_steps, policy) {
                    Ok(bounds) => bounds,
                    Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
                }
            }
            Self::Circle(fragment) => match fragment
                .semicircle()
                .conservative_bounds_refined(refinement_steps, policy)
            {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        }
    }

    pub(crate) fn has_certified_injective_axis(&self, policy: &CurveContext) -> bool {
        match self {
            Self::Bezier(curve) => curve.has_certified_injective_axis(policy),
            Self::Parallel(parallel) => {
                parallel.regular_fragment_has_certified_injective_axis(policy)
                    || matches!(
                        parallel.exact_rational_parallel_component(policy),
                        Ok(Classification::Decided(Some(curve)))
                            if curve.has_certified_injective_axis(policy)
                    )
            }
            Self::Line(_) => false,
            Self::Circle(_) => false,
        }
    }

    pub(crate) fn has_certified_injective_image(&self, policy: &CurveContext) -> bool {
        match self {
            Self::Bezier(curve) => curve.has_certified_injective_image(policy),
            Self::Parallel(_) => self.has_certified_injective_axis(policy),
            Self::Line(_) => true,
            Self::Circle(_) => true,
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
