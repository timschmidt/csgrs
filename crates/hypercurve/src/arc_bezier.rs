//! Exact circular-arc decomposition into rational quadratic Bezier spans.

use std::cmp::Ordering;

use hyperreal::RealSign;

use crate::{
    CircularArc2, Classification, CurveError, CurveFamily2, CurveOperation2, CurvePolicy,
    CurveSource2, ExactCurveError, ExactCurveResult, Point2, RationalQuadraticBezier2, Real,
    UncertaintyReason,
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
    /// Bezier parameter locally.
    pub fn rational_bezier_decomposition(
        &self,
    ) -> ExactCurveResult<&CircularArcBezierDecomposition2> {
        match self
            .retained_facts
            .bezier_decomposition
            .get_or_init(|| compute_circular_arc_decomposition(self))
        {
            Ok(decomposition) => Ok(decomposition),
            Err(error) => Err(error.clone()),
        }
    }
}

impl CircularArcBezierDecomposition2 {
    /// Returns exact rational quadratic spans in traversal order.
    pub fn spans(&self) -> &[CircularArcBezierSpan2] {
        &self.spans
    }

    /// Evaluates the piecewise-rational arc parameterization on `[0, 1]`.
    pub fn point_at(&self, parameter: &Real) -> ExactCurveResult<Point2> {
        evaluate_decomposition(self, parameter, None)
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<CircularArcBezierDecomposition2> {
    arc.rational_bezier_decomposition()
        .cloned()
        .map_err(|error| contextualize_arc_error(error, source))
}

fn compute_circular_arc_decomposition(
    arc: &CircularArc2,
) -> ExactCurveResult<CircularArcBezierDecomposition2> {
    validate_radius(arc, None)?;
    let kind = classify_sweep(arc, None)?;
    let points = match kind {
        ArcSweepKind::Minor => vec![arc.start().clone(), arc.end().clone()],
        ArcSweepKind::Semicircle => vec![
            arc.start().clone(),
            perpendicular_midpoint(arc),
            arc.end().clone(),
        ],
        ArcSweepKind::Major => vec![
            arc.start().clone(),
            major_midpoint(arc, None)?,
            arc.end().clone(),
        ],
        ArcSweepKind::FullCircle => full_circle_quarter_points(arc),
    };
    let span_count = points.len() - 1;
    let denominator = Real::from(span_count as u8);
    let mut spans = Vec::with_capacity(span_count);
    for (span_index, endpoints) in points.windows(2).enumerate() {
        let parameter_start = (Real::from(span_index as u8) / &denominator)
            .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, None, cause.into()))?;
        let parameter_end = (Real::from((span_index + 1) as u8) / &denominator)
            .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, None, cause.into()))?;
        spans.push(CircularArcBezierSpan2 {
            curve: rational_minor_arc_span(
                arc.center(),
                arc.radius_squared_ref(),
                endpoints,
                None,
            )?,
            parameter_start,
            parameter_end,
        });
    }
    Ok(CircularArcBezierDecomposition2 { spans })
}

pub(crate) fn evaluate_decomposition(
    decomposition: &CircularArcBezierDecomposition2,
    parameter: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Point2> {
    let policy = CurvePolicy::certified();
    for span in &decomposition.spans {
        let lower = crate::classify::compare_reals(&span.parameter_start, parameter, &policy);
        let upper = crate::classify::compare_reals(parameter, &span.parameter_end, &policy);
        match (lower, upper) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less | Ordering::Equal)) => {
                let width = &span.parameter_end - &span.parameter_start;
                let local = ((parameter - &span.parameter_start) / width).map_err(|cause| {
                    arc_error(CurveOperation2::Evaluation, source, cause.into())
                })?;
                return match span.curve.point_at(local, &policy) {
                    Classification::Decided(point) => Ok(point),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        source,
                        reason,
                    )),
                };
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::CircularArc,
                    source,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    Err(arc_error(
        CurveOperation2::Evaluation,
        source,
        CurveError::InvalidCurveParameter,
    ))
}

fn validate_radius(arc: &CircularArc2, source: Option<CurveSource2>) -> ExactCurveResult<()> {
    match crate::classify::is_zero(arc.radius_squared_ref(), &CurvePolicy::certified()) {
        Some(false) => {}
        Some(true) => {
            return Err(arc_error(
                CurveOperation2::BezierDecomposition,
                source,
                CurveError::ZeroRadiusArc,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::BezierDecomposition,
                CurveFamily2::CircularArc,
                source,
                UncertaintyReason::RealSign,
            ));
        }
    }
    if arc.endpoints_on_stored_circle_are_certified() {
        return Ok(());
    }
    let mismatch =
        arc.start().distance_squared(arc.center()) - arc.end().distance_squared(arc.center());
    match crate::classify::is_zero(&mismatch, &CurvePolicy::certified()) {
        Some(true) => Ok(()),
        Some(false) => Err(arc_error(
            CurveOperation2::BezierDecomposition,
            source,
            CurveError::RadiusMismatch,
        )),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::BezierDecomposition,
            CurveFamily2::CircularArc,
            source,
            UncertaintyReason::RealSign,
        )),
    }
}

pub(crate) fn classify_sweep(
    arc: &CircularArc2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<ArcSweepKind> {
    arc.retained_facts
        .sweep_kind
        .get_or_init(|| classify_sweep_uncached(arc))
        .clone()
        .map_err(|error| contextualize_arc_error(error, source))
}

fn classify_sweep_uncached(arc: &CircularArc2) -> ExactCurveResult<ArcSweepKind> {
    let policy = CurvePolicy::certified();
    let endpoint_distance = arc.start().distance_squared(arc.end());
    match crate::classify::is_zero(&endpoint_distance, &policy) {
        Some(true) => return Ok(ArcSweepKind::FullCircle),
        Some(false) => {}
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::BezierDecomposition,
                CurveFamily2::CircularArc,
                None,
                UncertaintyReason::RealSign,
            ));
        }
    }

    let start = arc.start().delta_from(arc.center());
    let end = arc.end().delta_from(arc.center());
    let cross = (&start.0 * &end.1) - (&start.1 * &end.0);
    let sign = crate::classify::real_sign(&cross, &policy).ok_or_else(|| {
        ExactCurveError::blocked(
            CurveOperation2::BezierDecomposition,
            CurveFamily2::CircularArc,
            None,
            UncertaintyReason::RealSign,
        )
    })?;
    match sign {
        RealSign::Positive => Ok(if arc.is_clockwise() {
            ArcSweepKind::Major
        } else {
            ArcSweepKind::Minor
        }),
        RealSign::Negative => Ok(if arc.is_clockwise() {
            ArcSweepKind::Minor
        } else {
            ArcSweepKind::Major
        }),
        RealSign::Zero => {
            let dot = (&start.0 * &end.0) + (&start.1 * &end.1);
            match crate::classify::real_sign(&dot, &policy) {
                Some(RealSign::Negative) => Ok(ArcSweepKind::Semicircle),
                Some(_) => Err(arc_error(
                    CurveOperation2::BezierDecomposition,
                    None,
                    CurveError::InvalidArcSweep,
                )),
                None => Err(ExactCurveError::blocked(
                    CurveOperation2::BezierDecomposition,
                    CurveFamily2::CircularArc,
                    None,
                    UncertaintyReason::RealSign,
                )),
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

fn major_midpoint(arc: &CircularArc2, source: Option<CurveSource2>) -> ExactCurveResult<Point2> {
    let start = arc.start().delta_from(arc.center());
    let end = arc.end().delta_from(arc.center());
    let sum_x = &start.0 + &end.0;
    let sum_y = &start.1 + &end.1;
    let sum_length_squared = (&sum_x * &sum_x) + (&sum_y * &sum_y);
    let scale = (arc.radius_squared() / sum_length_squared)
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, source, cause.into()))?
        .sqrt()
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, source, cause.into()))?;
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

fn rational_minor_arc_span(
    center: &Point2,
    radius_squared: &Real,
    endpoints: &[Point2],
    source: Option<CurveSource2>,
) -> ExactCurveResult<RationalQuadraticBezier2> {
    let start = endpoints[0].delta_from(center);
    let end = endpoints[1].delta_from(center);
    let dot = (&start.0 * &end.0) + (&start.1 * &end.1);
    let two_radius_squared = Real::from(2_i8) * radius_squared;
    let weight_squared = ((radius_squared + dot) / two_radius_squared)
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, source, cause.into()))?;
    match crate::classify::compare_reals(&weight_squared, &Real::zero(), &CurvePolicy::certified())
    {
        Some(Ordering::Greater) => {}
        Some(_) => {
            return Err(arc_error(
                CurveOperation2::BezierDecomposition,
                source,
                CurveError::InvalidArcSweep,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::BezierDecomposition,
                CurveFamily2::CircularArc,
                source,
                UncertaintyReason::RealSign,
            ));
        }
    }
    let weight = weight_squared
        .clone()
        .sqrt()
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, source, cause.into()))?;
    let control_denominator = Real::from(2_i8) * weight_squared;
    let control_x = ((&start.0 + &end.0) / &control_denominator)
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, source, cause.into()))?;
    let control_y = ((&start.1 + &end.1) / control_denominator)
        .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, source, cause.into()))?;
    RationalQuadraticBezier2::try_unit_end_weights(
        endpoints[0].clone(),
        Point2::new(center.x() + control_x, center.y() + control_y),
        endpoints[1].clone(),
        weight,
    )
    .map_err(|cause| arc_error(CurveOperation2::BezierDecomposition, source, cause))
}

fn arc_error(
    operation: CurveOperation2,
    source: Option<CurveSource2>,
    cause: CurveError,
) -> ExactCurveError {
    ExactCurveError::invalid(operation, CurveFamily2::CircularArc, source, cause)
}

fn contextualize_arc_error(
    error: ExactCurveError,
    source: Option<CurveSource2>,
) -> ExactCurveError {
    let Some(source) = source else {
        return error;
    };
    match error {
        ExactCurveError::Invalid {
            operation,
            family,
            cause,
            ..
        } => ExactCurveError::invalid(operation, family, Some(source), cause),
        ExactCurveError::Blocked(blocker) => ExactCurveError::blocked(
            blocker.operation(),
            blocker.family(),
            Some(source),
            blocker.reason(),
        ),
    }
}

#[cfg(test)]
mod tests {
    use std::rc::Rc;

    use super::{CircularArc2, decompose_circular_arc};
    use crate::{CurveSource2, Point2, Real};

    fn point(x: i8, y: i8) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn arc_clones_share_sweep_and_decomposition_caches() {
        let arc =
            CircularArc2::try_from_center(point(5, 0), point(0, 5), point(0, 0), false).unwrap();
        let clone = arc.clone();

        assert!(arc.retained_facts.sweep_kind.get().is_none());
        assert!(arc.retained_facts.bezier_decomposition.get().is_none());
        arc.rational_bezier_decomposition().unwrap();

        assert!(Rc::ptr_eq(&arc.retained_facts, &clone.retained_facts));
        assert!(clone.retained_facts.sweep_kind.get().is_some());
        assert!(clone.retained_facts.bezier_decomposition.get().is_some());
    }

    #[test]
    fn cached_arc_failure_receives_each_top_level_source_context() {
        let arc = CircularArc2::new_unchecked_with_radius(
            point(0, 0),
            point(1, 0),
            point(0, 0),
            Real::zero(),
            false,
            None,
        );

        let first = decompose_circular_arc(&arc, Some(CurveSource2::new(17))).unwrap_err();
        let second = decompose_circular_arc(&arc, Some(CurveSource2::new(29))).unwrap_err();

        assert_eq!(first.source(), Some(CurveSource2::new(17)));
        assert_eq!(second.source(), Some(CurveSource2::new(29)));
    }
}
