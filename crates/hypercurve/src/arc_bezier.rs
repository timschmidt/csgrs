//! Exact circular-arc decomposition into rational quadratic Bezier spans.

use std::cmp::Ordering;
use std::sync::Arc;

use hyperreal::RealSign;

use crate::policy::{resolve_cached_evaluation, resolve_certified_operation};
use crate::rational_bezier::RationalQuadraticCircle2;
use crate::{
    CircularArc2, Classification, CurveContext, CurveError, CurveFamily2, CurveOperation2,
    CurveOutcome, CurveResult, ExactCurveError, ExactCurveResult, Point2, RationalBezier2,
    RationalQuadraticBezier2, Real, UncertaintyReason,
};

/// Exact rational quadratic span from one circular-arc decomposition.
#[derive(Clone, Debug, PartialEq)]
pub struct CircularArcBezierSpan2 {
    curve: RationalQuadraticBezier2,
    parameter_start: Real,
    parameter_end: Real,
}

/// Exact piecewise-rational representation of one circular arc.
#[derive(Clone, Debug, PartialEq)]
pub struct CircularArcBezierDecomposition2 {
    spans: Vec<CircularArcBezierSpan2>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ArcSweepKind {
    Minor,
    Semicircle,
    Major,
    FullCircle,
}

impl CircularArc2 {
    /// Decomposes this arc into exact rational quadratic Bezier spans.
    ///
    /// Minor sweeps use one span, semicircles and major sweeps use two, and a
    /// full circle uses four quarter-circle spans. The returned parameter
    /// intervals partition `[0, 1]`; each interval uses the native rational
    /// Bezier parameter locally. The returned [`CurveOutcome`] records whether
    /// classifying the exact sweep consumed the `APPROXIMATE_512` terminal.
    #[inline(always)]
    pub fn rational_bezier_decomposition(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&CircularArcBezierDecomposition2>> {
        resolve_certified_operation(policy, |attempt| {
            match self.rational_bezier_decomposition_with_policy(attempt)? {
                Classification::Decided(decomposition) => Ok(decomposition),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::BezierDecomposition,
                    CurveFamily2::CircularArc,
                    reason,
                )),
            }
        })
    }

    #[inline]
    pub(crate) fn rational_bezier_decomposition_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&CircularArcBezierDecomposition2>> {
        resolve_cached_evaluation(
            &self.retained_facts.bezier_decomposition,
            policy,
            |attempt| compute_circular_arc_decomposition(self, attempt),
        )
    }
}

impl CircularArcBezierDecomposition2 {
    /// Returns exact rational quadratic spans in traversal order.
    pub fn spans(&self) -> &[CircularArcBezierSpan2] {
        &self.spans
    }

    /// Evaluates the piecewise-rational arc parameterization on `[0, 1]`.
    ///
    /// The returned [`CurveOutcome`] records whether selecting or evaluating
    /// the exact span consumed the `APPROXIMATE_512` terminal.
    #[inline(always)]
    pub fn point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        resolve_certified_operation(policy, |attempt| {
            evaluate_decomposition(self, parameter, attempt)
        })
    }

    pub(crate) fn point_at_with_policy(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        evaluate_decomposition(self, parameter, policy)
    }
}

impl CircularArcBezierSpan2 {
    /// Returns the exact rational quadratic circular-arc span.
    pub const fn curve(&self) -> &RationalQuadraticBezier2 {
        &self.curve
    }

    /// Returns this span's exact global arc parameter interval.
    pub fn parameter_range(&self) -> (&Real, &Real) {
        (&self.parameter_start, &self.parameter_end)
    }
}

pub(crate) fn decompose_circular_arc(
    arc: &CircularArc2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<CircularArcBezierDecomposition2>> {
    arc.rational_bezier_decomposition_with_policy(policy)
        .map(|classification| classification.map(Clone::clone))
        .map_err(contextualize_arc_error)
}

fn compute_circular_arc_decomposition(
    arc: &CircularArc2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<CircularArcBezierDecomposition2>> {
    if let Classification::Uncertain(reason) = validate_radius(arc, policy)? {
        return Ok(Classification::Uncertain(reason));
    }
    let kind = match classify_sweep_with_policy(arc, policy)? {
        Classification::Decided(kind) => kind,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let points = match kind {
        ArcSweepKind::Minor => vec![arc.start().clone(), arc.end().clone()],
        ArcSweepKind::Semicircle => vec![
            arc.start().clone(),
            perpendicular_midpoint(arc),
            arc.end().clone(),
        ],
        ArcSweepKind::Major => vec![arc.start().clone(), major_midpoint(arc)?, arc.end().clone()],
        ArcSweepKind::FullCircle => full_circle_quarter_points(arc),
    };
    let span_count = points.len() - 1;
    let denominator = Real::from(span_count as u8);
    let (implicit_quadratic_conic, circular_conic) = circular_conic_provenance(arc);
    let mut spans = Vec::with_capacity(span_count);
    for (span_index, endpoints) in points.windows(2).enumerate() {
        let parameter_start = (Real::from(span_index as u8) / &denominator)
            .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, cause.into()))?;
        let parameter_end = (Real::from((span_index + 1) as u8) / &denominator)
            .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, cause.into()))?;
        spans.push(CircularArcBezierSpan2 {
            curve: rational_minor_arc_span(&implicit_quadratic_conic, &circular_conic, endpoints)
                .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, cause))?,
            parameter_start,
            parameter_end,
        });
    }
    Ok(Classification::Decided(CircularArcBezierDecomposition2 {
        spans,
    }))
}

pub(crate) fn evaluate_decomposition(
    decomposition: &CircularArcBezierDecomposition2,
    parameter: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<Point2> {
    for span in &decomposition.spans {
        let lower = crate::classify::compare_reals(&span.parameter_start, parameter, policy);
        let upper = crate::classify::compare_reals(parameter, &span.parameter_end, policy);
        match (lower, upper) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less | Ordering::Equal)) => {
                if lower == Some(Ordering::Equal) {
                    return Ok(span.curve.start().clone());
                }
                if upper == Some(Ordering::Equal) {
                    return Ok(span.curve.end().clone());
                }
                let width = &span.parameter_end - &span.parameter_start;
                let local = ((parameter - &span.parameter_start) / width)
                    .map_err(|cause| arc_error(CurveOperation2::Evaluation, cause.into()))?;
                return match span.curve.point_at(local, policy) {
                    Classification::Decided(point) => Ok(point),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        reason,
                    )),
                };
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::CircularArc,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    Err(arc_error(
        CurveOperation2::Evaluation,
        CurveError::InvalidCurveParameter,
    ))
}

fn validate_radius(
    arc: &CircularArc2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<()>> {
    match crate::classify::is_zero(arc.radius_squared_ref(), policy) {
        Some(false) => {}
        Some(true) => {
            return Err(arc_error(
                CurveOperation2::BezierDecomposition,
                CurveError::ZeroRadiusArc,
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    if arc.endpoints_on_stored_circle_are_certified() {
        return Ok(Classification::Decided(()));
    }
    let mismatch =
        arc.start().distance_squared(arc.center()) - arc.end().distance_squared(arc.center());
    match crate::classify::is_zero(&mismatch, policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => Err(arc_error(
            CurveOperation2::BezierDecomposition,
            CurveError::RadiusMismatch,
        )),
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

pub(crate) fn classify_sweep(arc: &CircularArc2) -> ExactCurveResult<ArcSweepKind> {
    match classify_sweep_with_policy(arc, &CurveContext::STRICT)? {
        Classification::Decided(kind) => Ok(kind),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::BezierDecomposition,
            CurveFamily2::CircularArc,
            reason,
        )),
    }
}

pub(crate) fn classify_sweep_with_policy(
    arc: &CircularArc2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<ArcSweepKind>> {
    resolve_cached_evaluation(&arc.retained_facts.sweep_kind, policy, |attempt| {
        classify_sweep_uncached(arc, attempt)
    })
    .map(|classification| classification.map(|kind| *kind))
    .map_err(contextualize_arc_error)
}

fn classify_sweep_uncached(
    arc: &CircularArc2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<ArcSweepKind>> {
    let endpoint_distance = arc.start().distance_squared(arc.end());
    match crate::classify::is_zero(&endpoint_distance, policy) {
        Some(true) => return Ok(Classification::Decided(ArcSweepKind::FullCircle)),
        Some(false) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }

    let start = arc.start().delta_from(arc.center());
    let end = arc.end().delta_from(arc.center());
    let cross = (&start.0 * &end.1) - (&start.1 * &end.0);
    let Some(sign) = crate::classify::real_sign(&cross, policy) else {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    };
    match sign {
        RealSign::Positive => Ok(Classification::Decided(if arc.is_clockwise() {
            ArcSweepKind::Major
        } else {
            ArcSweepKind::Minor
        })),
        RealSign::Negative => Ok(Classification::Decided(if arc.is_clockwise() {
            ArcSweepKind::Minor
        } else {
            ArcSweepKind::Major
        })),
        RealSign::Zero => {
            let dot = (&start.0 * &end.0) + (&start.1 * &end.1);
            match crate::classify::real_sign(&dot, policy) {
                Some(RealSign::Negative) => Ok(Classification::Decided(ArcSweepKind::Semicircle)),
                Some(_) => Err(arc_error(
                    CurveOperation2::BezierDecomposition,
                    CurveError::InvalidArcSweep,
                )),
                None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
    }
}

fn perpendicular_midpoint(arc: &CircularArc2) -> Point2 {
    let radius = arc.start().delta_from(arc.center());
    let (x, y) = if arc.is_clockwise() {
        (radius.1, -radius.0)
    } else {
        (-radius.1, radius.0)
    };
    Point2::new(arc.center().x() + x, arc.center().y() + y)
}

fn major_midpoint(arc: &CircularArc2) -> ExactCurveResult<Point2> {
    let start = arc.start().delta_from(arc.center());
    let end = arc.end().delta_from(arc.center());
    let sum_x = &start.0 + &end.0;
    let sum_y = &start.1 + &end.1;
    let sum_length_squared = (&sum_x * &sum_x) + (&sum_y * &sum_y);
    let scale = (arc.radius_squared() / sum_length_squared)
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, cause.into()))?
        .sqrt()
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, cause.into()))?;
    Ok(Point2::new(
        arc.center().x() - (&sum_x * &scale),
        arc.center().y() - (&sum_y * &scale),
    ))
}

fn full_circle_quarter_points(arc: &CircularArc2) -> Vec<Point2> {
    let radius = arc.start().delta_from(arc.center());
    let first_quarter = if arc.is_clockwise() {
        (radius.1.clone(), -radius.0.clone())
    } else {
        (-radius.1.clone(), radius.0.clone())
    };
    let opposite = (-radius.0.clone(), -radius.1.clone());
    let third_quarter = (-first_quarter.0.clone(), -first_quarter.1.clone());
    let point = |vector: (Real, Real)| {
        Point2::new(arc.center().x() + vector.0, arc.center().y() + vector.1)
    };
    vec![
        arc.start().clone(),
        point(first_quarter),
        point(opposite),
        point(third_quarter),
        arc.end().clone(),
    ]
}

/// Builds the normalized implicit and metric certificates for one exact circle.
///
/// Recognition, decomposition, and retained-curve admission all use this one
/// representation so a circle does not acquire operation-specific provenance.
pub(crate) fn circular_conic_provenance(
    arc: &CircularArc2,
) -> (Arc<[Real; 6]>, Arc<RationalQuadraticCircle2>) {
    let two = Real::from(2_i8);
    (
        Arc::new([
            Real::one(),
            Real::zero(),
            Real::one(),
            -(&two * arc.center().x()),
            -(&two * arc.center().y()),
            arc.center().x() * arc.center().x() + arc.center().y() * arc.center().y()
                - arc.radius_squared_ref(),
        ]),
        Arc::new(RationalQuadraticCircle2 {
            center: arc.center().clone(),
            radius_squared: arc.radius_squared_ref().clone(),
            tangent_contacts: None,
        }),
    )
}

pub(crate) fn rational_bezier_circular_arc(
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<CircularArc2>>> {
    // Degree elevation and exact subdivision retain the authoritative circle
    // even when reconstructing a quadratic representative would do needless
    // projective work. Common-sign weights make the first control edge a
    // positive multiple of the endpoint tangent, which is sufficient to
    // recover traversal orientation without changing parameterization.
    if let Some(circle) = curve.retained_circular_conic() {
        if let Classification::Uncertain(reason) = curve.common_weight_sign(policy) {
            return Ok(Classification::Uncertain(reason));
        }
        let Some(control) = curve.control_points().get(1) else {
            return Ok(Classification::Decided(None));
        };
        let (radial_x, radial_y) = curve.start().delta_from(&circle.center);
        let (tangent_x, tangent_y) = control.delta_from(curve.start());
        let tangent_cross = &radial_x * tangent_y - &radial_y * tangent_x;
        let clockwise = match crate::classify::real_sign(&tangent_cross, policy) {
            Some(RealSign::Positive) => false,
            Some(RealSign::Negative) => true,
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            None => {
                // Exact round joins retain the adjacent directed line and
                // contact point.  That endpoint tangent is an authoritative
                // traversal certificate even when expanding the selected
                // control point into a radial cross product exceeds the
                // predicate budget.
                let retained_cross = circle.tangent_contacts.as_deref().and_then(|contacts| {
                    contacts.iter().find_map(|contact| {
                        let crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line {
                            line,
                            point,
                        } = contact
                        else {
                            return None;
                        };
                        // The retained line is directed with the authored
                        // boundary traversal.  Radial cross tangent therefore
                        // fixes the circle orientation at any certified
                        // contact, including contacts outside this particular
                        // decomposed span.
                        let (radial_x, radial_y) = point.delta_from(&circle.center);
                        let (tangent_x, tangent_y) = line.delta();
                        crate::classify::real_sign(
                            &(&radial_x * tangent_y - &radial_y * tangent_x),
                            policy,
                        )
                    })
                });
                match retained_cross {
                    Some(RealSign::Positive) => false,
                    Some(RealSign::Negative) => true,
                    Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
            }
        };
        return Ok(Classification::Decided(Some(
            CircularArc2::new_with_certified_radius(
                curve.start().clone(),
                curve.end().clone(),
                circle.center.clone(),
                circle.radius_squared.clone(),
                clockwise,
                None,
            ),
        )));
    }
    let conic = match curve.retained_quadratic_representative(policy)? {
        Classification::Decided(Some(conic)) => conic,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    rational_quadratic_circular_arc(&conic, policy)
}

pub(crate) fn rational_quadratic_circular_arc(
    curve: &RationalQuadraticBezier2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<CircularArc2>>> {
    let (start_weight_sign, control_weight_sign) =
        match crate::rational_bezier::pole_free_quadratic_weight_signs(
            [
                curve.start_weight(),
                curve.control_weight(),
                curve.end_weight(),
            ],
            policy,
        ) {
            Classification::Decided(Some(signs)) => signs,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if let Some(circle) = curve.retained_circular_conic() {
        let (radial_x, radial_y) = curve.start().delta_from(&circle.center);
        let (tangent_x, tangent_y) = curve.control().delta_from(curve.start());
        let tangent_cross = &radial_x * tangent_y - &radial_y * tangent_x;
        let tangent_reversed = control_weight_sign != start_weight_sign;
        let clockwise = match crate::classify::real_sign(&tangent_cross, policy) {
            Some(RealSign::Positive) => tangent_reversed,
            Some(RealSign::Negative) => !tangent_reversed,
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        return Ok(Classification::Decided(Some(
            CircularArc2::new_with_certified_radius(
                curve.start().clone(),
                curve.end().clone(),
                circle.center.clone(),
                circle.radius_squared.clone(),
                clockwise,
                None,
            ),
        )));
    }

    let homogeneous =
        |point: &Point2, weight: &Real| [weight * point.x(), weight * point.y(), weight.clone()];
    let first = homogeneous(curve.start(), curve.start_weight());
    let control = homogeneous(curve.control(), curve.control_weight());
    let last = homogeneous(curve.end(), curve.end_weight());
    let cross = |left: &[Real; 3], right: &[Real; 3]| {
        [
            &left[1] * &right[2] - &left[2] * &right[1],
            &left[2] * &right[0] - &left[0] * &right[2],
            &left[0] * &right[1] - &left[1] * &right[0],
        ]
    };
    let lambda0 = cross(&control, &last);
    let lambda1 = cross(&last, &first);
    let lambda2 = cross(&first, &control);
    let four = Real::from(4_i8);
    let two = Real::from(2_i8);
    let xx = &lambda1[0] * &lambda1[0] - &four * &lambda0[0] * &lambda2[0];
    let xy = &two * &lambda1[0] * &lambda1[1]
        - &four * (&lambda0[0] * &lambda2[1] + &lambda0[1] * &lambda2[0]);
    let yy = &lambda1[1] * &lambda1[1] - &four * &lambda0[1] * &lambda2[1];
    let x = &two * &lambda1[0] * &lambda1[2]
        - &four * (&lambda0[0] * &lambda2[2] + &lambda0[2] * &lambda2[0]);
    let y = &two * &lambda1[1] * &lambda1[2]
        - &four * (&lambda0[1] * &lambda2[2] + &lambda0[2] * &lambda2[1]);
    let constant = &lambda1[2] * &lambda1[2] - &four * &lambda0[2] * &lambda2[2];
    match (
        crate::classify::real_sign(&xx, policy),
        crate::classify::real_sign(&(&xx - &yy), policy),
        crate::classify::real_sign(&xy, policy),
    ) {
        (
            Some(RealSign::Positive | RealSign::Negative),
            Some(RealSign::Zero),
            Some(RealSign::Zero),
        ) => {}
        (Some(_), Some(_), Some(_)) => return Ok(Classification::Decided(None)),
        _ => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let start_tangent = curve.control().delta_from(curve.start());
    let end_tangent = curve.end().delta_from(curve.control());
    let start_normal = (-start_tangent.1, start_tangent.0);
    let end_normal = (-end_tangent.1, end_tangent.0);
    let normal_cross = &start_normal.0 * &end_normal.1 - &start_normal.1 * &end_normal.0;
    let center = if matches!(
        crate::classify::real_sign(&normal_cross, policy),
        Some(RealSign::Positive | RealSign::Negative)
    ) {
        let chord = curve.end().delta_from(curve.start());
        let scale = ((&chord.0 * &end_normal.1 - &chord.1 * &end_normal.0) / normal_cross)?;
        Point2::new(
            curve.start().x() + &scale * &start_normal.0,
            curve.start().y() + scale * &start_normal.1,
        )
    } else {
        let denominator = &two * &xx;
        Point2::new(((-x) / &denominator)?, ((-y) / denominator)?)
    };
    let radius_squared = curve.start().distance_squared(&center);
    let implicit_radius_squared =
        center.x() * center.x() + center.y() * center.y() - ((constant / &xx)?);
    match crate::classify::real_sign(&(radius_squared - implicit_radius_squared), policy) {
        Some(RealSign::Zero) => {}
        Some(RealSign::Positive | RealSign::Negative) => {
            return Ok(Classification::Decided(None));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let (radial_x, radial_y) = curve.start().delta_from(&center);
    let (tangent_x, tangent_y) = curve.control().delta_from(curve.start());
    // The endpoint derivative is `2*w1/w0*(P1-P0)`. Common-sign weights made
    // the old control-edge shortcut sufficient; a regular major arc reverses
    // that edge because its middle weight has the opposite sign.
    let tangent_cross = &radial_x * tangent_y - &radial_y * tangent_x;
    let tangent_reversed = control_weight_sign != start_weight_sign;
    let clockwise = match crate::classify::real_sign(&tangent_cross, policy) {
        Some(RealSign::Positive) => tangent_reversed,
        Some(RealSign::Negative) => !tangent_reversed,
        Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    Ok(Classification::Decided(Some(
        CircularArc2::try_from_center(
            curve.start().clone(),
            curve.end().clone(),
            center,
            clockwise,
        )?,
    )))
}

pub(crate) fn rational_minor_arc_span(
    implicit_quadratic_conic: &Arc<[Real; 6]>,
    circular_conic: &Arc<RationalQuadraticCircle2>,
    endpoints: &[Point2],
) -> CurveResult<RationalQuadraticBezier2> {
    let center = &circular_conic.center;
    let radius_squared = &circular_conic.radius_squared;
    let start = endpoints[0].delta_from(center);
    let end = endpoints[1].delta_from(center);
    let dot = (&start.0 * &end.0) + (&start.1 * &end.1);
    let cross = (&start.0 * &end.1) - (&start.1 * &end.0);
    let tangent_half = (cross / (radius_squared + dot))?;
    let control = Point2::new(
        center.x() + &start.0 - &tangent_half * &start.1,
        center.y() + &start.1 + &tangent_half * &start.0,
    );
    let end_weight = Real::one() + &tangent_half * &tangent_half;
    RationalQuadraticBezier2::try_new_with_common_weight_sign_and_implicit_conic(
        endpoints[0].clone(),
        control,
        endpoints[1].clone(),
        Real::one(),
        Real::one(),
        end_weight,
        None,
        Some(Arc::clone(implicit_quadratic_conic)),
        Some(Arc::clone(circular_conic)),
    )
}

fn arc_error(operation: CurveOperation2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(operation, CurveFamily2::CircularArc, cause)
}

fn contextualize_arc_error(error: ExactCurveError) -> ExactCurveError {
    error
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::{CircularArc2, rational_quadratic_circular_arc};
    use crate::{Classification, CurveContext, Point2, RationalQuadraticBezier2, Real};

    fn point(x: i8, y: i8) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn arc_clones_share_sweep_and_decomposition_caches() {
        let arc =
            CircularArc2::try_from_center(point(5, 0), point(0, 5), point(0, 0), false).unwrap();
        let clone = arc.clone();

        assert!(arc.retained_facts.sweep_kind.is_empty());
        assert!(arc.retained_facts.bezier_decomposition.is_empty());
        arc.rational_bezier_decomposition(&CurveContext::STRICT)
            .unwrap();

        assert!(Arc::ptr_eq(&arc.retained_facts, &clone.retained_facts));
        assert!(!clone.retained_facts.sweep_kind.is_empty());
        assert!(!clone.retained_facts.bezier_decomposition.is_empty());
    }

    #[test]
    fn one_arc_decomposition_shares_exact_conic_provenance() {
        let arc =
            CircularArc2::try_from_center(point(2, 0), point(2, 0), point(0, 0), false).unwrap();
        let decomposition = arc
            .rational_bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value();
        let [first, second, third, fourth] = decomposition.spans() else {
            panic!("a full circle must retain four spans")
        };
        for span in [second, third, fourth] {
            assert!(Arc::ptr_eq(
                first.curve().retained_circular_conic().unwrap(),
                span.curve().retained_circular_conic().unwrap(),
            ));
            assert!(Arc::ptr_eq(
                first.curve().retained_implicit_quadratic_conic().unwrap(),
                span.curve().retained_implicit_quadratic_conic().unwrap(),
            ));
        }
    }

    #[test]
    fn independent_algebraic_quarter_circle_recovers_minimal_center() {
        let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
        let curve = RationalQuadraticBezier2::try_unit_end_weights(
            point(1, 0),
            point(1, 1),
            point(0, 1),
            half_sqrt_two,
        )
        .unwrap();

        let Classification::Decided(Some(arc)) =
            rational_quadratic_circular_arc(&curve, &CurveContext::STRICT).unwrap()
        else {
            panic!("the canonical algebraic quarter circle must be recognized strictly")
        };
        assert_eq!(arc.center(), &point(0, 0));
        assert_eq!(arc.radius_squared_ref(), &Real::one());
        assert!(!arc.is_clockwise());
    }

    #[test]
    fn mixed_weight_major_circle_is_recognized_without_a_projective_pole() {
        let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
        let curve = RationalQuadraticBezier2::try_unit_end_weights(
            point(1, 0),
            point(1, 1),
            point(0, 1),
            -half_sqrt_two,
        )
        .unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(arc)) =
                rational_quadratic_circular_arc(&curve, &policy).unwrap()
            else {
                panic!("the pole-free mixed-weight major circle must be recognized")
            };
            assert_eq!(arc.center(), &point(0, 0));
            assert_eq!(arc.radius_squared_ref(), &Real::one());
            assert!(arc.is_clockwise());
            assert_eq!(
                arc.rational_bezier_decomposition(&policy)
                    .unwrap()
                    .into_value()
                    .spans()
                    .len(),
                2,
            );
        }
    }

    #[test]
    fn mixed_weight_conic_with_a_projective_pole_is_not_a_regular_arc() {
        for middle_weight in [-Real::one(), -Real::from(2_i8)] {
            let curve = RationalQuadraticBezier2::try_unit_end_weights(
                point(1, 0),
                point(1, 1),
                point(0, 1),
                middle_weight,
            )
            .unwrap();
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                assert!(matches!(
                    rational_quadratic_circular_arc(&curve, &policy).unwrap(),
                    Classification::Decided(None)
                ));
            }
        }
    }
}
