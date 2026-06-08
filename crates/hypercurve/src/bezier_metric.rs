//! Certified metric adapters for polynomial Bezier segments.
//!
//! Arc length is not evaluated by sampling here. A polynomial Bezier segment's
//! length is bounded below by its endpoint chord and above by its control
//! polygon length; exact de Casteljau subdivision tightens that interval by
//! summing the same enclosure over subcurves. This is the classical
//! variation-diminishing/control-polygon bound for Bezier curves described by
//! Farin, *Curves and Surfaces for Computer-Aided Geometric Design* (5th ed.,
//! 2002). Following Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997), this module returns explicit
//! certified intervals instead of converting them into floating approximations.

use hyperreal::Real;
use std::cmp::Ordering;

use crate::classify::{compare_reals, in_closed_unit_interval, is_zero};
use crate::{
    BezierMonotoneSpan, Classification, CubicBezier2, CurveError, CurvePolicy, CurveResult, Point2,
    QuadraticBezier2, UncertaintyReason,
};

/// Exact lower and upper bounds for a Bezier segment's arc length.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierLengthBounds2 {
    lower: Real,
    upper: Real,
}

/// Certified parameter region for an inverse arc-length query.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierArcLengthParameterRegion2 {
    target_length: Real,
    parameter_span: BezierMonotoneSpan,
    prefix_bounds_at_span_end: BezierLengthBounds2,
}

impl BezierArcLengthParameterRegion2 {
    /// Constructs an inverse-length region from exact metric witnesses.
    const fn new(
        target_length: Real,
        parameter_span: BezierMonotoneSpan,
        prefix_bounds_at_span_end: BezierLengthBounds2,
    ) -> Self {
        Self {
            target_length,
            parameter_span,
            prefix_bounds_at_span_end,
        }
    }

    /// Returns the requested arc length.
    pub const fn target_length(&self) -> &Real {
        &self.target_length
    }

    /// Returns the certified parameter span that contains the inverse query.
    pub const fn parameter_span(&self) -> &BezierMonotoneSpan {
        &self.parameter_span
    }

    /// Returns prefix length bounds at the span end parameter.
    ///
    /// This witness proves the target has not been pruned from the returned
    /// parameter span by the monotone prefix-length enclosure.
    pub const fn prefix_bounds_at_span_end(&self) -> &BezierLengthBounds2 {
        &self.prefix_bounds_at_span_end
    }
}

impl BezierLengthBounds2 {
    /// Constructs length bounds after preserving `lower <= upper` responsibility
    /// at the caller's certified geometric construction site.
    const fn new(lower: Real, upper: Real) -> Self {
        Self { lower, upper }
    }

    /// Returns the exact chord-length lower bound.
    pub const fn lower(&self) -> &Real {
        &self.lower
    }

    /// Returns the exact control-polygon-length upper bound.
    pub const fn upper(&self) -> &Real {
        &self.upper
    }

    /// Returns whether the metric interval has zero width as a structural fact.
    ///
    /// This is useful for certified line-image cases: when all control points
    /// are ordered on the endpoint segment, the Bezier arc length is exactly
    /// both the endpoint chord and the control-polygon length.
    pub fn is_exact(&self) -> bool {
        self.lower == self.upper
    }

    /// Returns the exact interval width `upper - lower`.
    pub fn width(&self) -> Real {
        &self.upper - &self.lower
    }
}

impl QuadraticBezier2 {
    /// Returns certified exact lower/upper bounds for this quadratic's length.
    ///
    /// The lower bound is the endpoint chord. The upper bound is the length of
    /// the two-edge control polygon. Those bounds are structural Bezier facts,
    /// not sampled estimates; see Farin (2002). Yap's EGC model is respected by
    /// returning exact `Real` endpoints for the interval and by surfacing any
    /// square-root construction failure through [`CurveResult`].
    pub fn length_bounds(&self) -> CurveResult<BezierLengthBounds2> {
        length_bounds_for_controls(&self.control_points())
    }

    /// Returns subdivision-refined certified length bounds.
    ///
    /// `max_depth = 0` is identical to [`QuadraticBezier2::length_bounds`].
    /// Larger depths split by exact de Casteljau bisection and add the
    /// chord/control-polygon intervals over each leaf. Farin's Bezier
    /// subdivision length enclosure is used only as a metric certificate; per
    /// Yap (1997), callers must not turn these intervals into topology events
    /// without a separate certified predicate.
    pub fn refined_length_bounds(&self, max_depth: usize) -> CurveResult<BezierLengthBounds2> {
        refined_length_bounds_for_controls(
            self.control_points().into_iter().cloned().collect(),
            max_depth,
        )
    }

    /// Returns certified length bounds for the prefix interval `[0, t]`.
    ///
    /// The parameter is first certified against `[0, 1]` through the active
    /// policy. The prefix curve is then built by exact de Casteljau subdivision
    /// at `t`, and its chord/control-polygon length interval is returned. This
    /// is the exact-object prerequisite for inverse arc-length queries; as Yap
    /// (1997) requires, ambiguous parameter ordering is reported as
    /// uncertainty instead of becoming an approximate branch.
    pub fn prefix_length_bounds(
        &self,
        t: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLengthBounds2>> {
        prefix_length_bounds_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            0,
            policy,
        )
    }

    /// Returns subdivision-refined certified length bounds for `[0, t]`.
    ///
    /// This combines exact de Casteljau prefix splitting with the same
    /// subdivision-refined interval used by [`QuadraticBezier2::refined_length_bounds`].
    pub fn refined_prefix_length_bounds(
        &self,
        t: Real,
        max_depth: usize,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLengthBounds2>> {
        prefix_length_bounds_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            max_depth,
            policy,
        )
    }

    /// Returns a certified parameter region for an inverse arc-length query.
    ///
    /// This is not a floating inverse-length solve. Exact degree-elevated line
    /// parameterizations first return a zero-width region from the linear
    /// length law. Other curves repeatedly bisect the parameter interval and
    /// compare `target_length` with certified prefix length intervals. If exact
    /// signs prove the target is before or after the midpoint, the bracket is
    /// reduced; if the prefix interval straddles the target, the remaining
    /// parameter span is returned. This follows Yap's exact-computation
    /// boundary: metric approximants may guide refinement, but only certified
    /// comparisons decide branches.
    pub fn inverse_length_parameter_region(
        &self,
        target_length: Real,
        search_depth: usize,
        metric_depth: usize,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierArcLengthParameterRegion2>> {
        inverse_length_parameter_region_for_controls(
            self.control_points().into_iter().cloned().collect(),
            target_length,
            search_depth,
            metric_depth,
            policy,
        )
    }
}

impl CubicBezier2 {
    /// Returns certified exact lower/upper bounds for this cubic's length.
    ///
    /// The lower bound is the endpoint chord. The upper bound is the length of
    /// the three-edge control polygon. This is the certified interval form of
    /// the standard Bezier control-polygon length enclosure; see Farin (2002)
    /// and Yap (1997).
    pub fn length_bounds(&self) -> CurveResult<BezierLengthBounds2> {
        length_bounds_for_controls(&self.control_points())
    }

    /// Returns subdivision-refined certified length bounds.
    ///
    /// `max_depth = 0` is identical to [`CubicBezier2::length_bounds`].
    /// Larger depths split by exact de Casteljau bisection and add the
    /// chord/control-polygon intervals over each leaf, preserving a certified
    /// interval rather than a sampled estimate.
    pub fn refined_length_bounds(&self, max_depth: usize) -> CurveResult<BezierLengthBounds2> {
        refined_length_bounds_for_controls(
            self.control_points().into_iter().cloned().collect(),
            max_depth,
        )
    }

    /// Returns certified length bounds for the prefix interval `[0, t]`.
    ///
    /// The prefix is constructed by exact de Casteljau subdivision at `t`;
    /// parameter validation is performed through the active exact predicate
    /// policy before any metric interval is emitted.
    pub fn prefix_length_bounds(
        &self,
        t: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLengthBounds2>> {
        prefix_length_bounds_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            0,
            policy,
        )
    }

    /// Returns subdivision-refined certified length bounds for `[0, t]`.
    pub fn refined_prefix_length_bounds(
        &self,
        t: Real,
        max_depth: usize,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLengthBounds2>> {
        prefix_length_bounds_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            max_depth,
            policy,
        )
    }

    /// Returns a certified parameter region for an inverse arc-length query.
    ///
    /// Exact degree-elevated line parameterizations are solved directly before
    /// interval bisection; general collinear-but-nonlinear line images are not
    /// collapsed to that fast path because their arc-length parameter is not
    /// the affine Bezier parameter.
    pub fn inverse_length_parameter_region(
        &self,
        target_length: Real,
        search_depth: usize,
        metric_depth: usize,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierArcLengthParameterRegion2>> {
        inverse_length_parameter_region_for_controls(
            self.control_points().into_iter().cloned().collect(),
            target_length,
            search_depth,
            metric_depth,
            policy,
        )
    }
}

fn length_bounds_for_controls(controls: &[&Point2]) -> CurveResult<BezierLengthBounds2> {
    let lower = distance(controls[0], controls[controls.len() - 1])?;
    let mut upper = Real::zero();
    for edge in controls.windows(2) {
        upper = &upper + distance(edge[0], edge[1])?;
    }
    Ok(BezierLengthBounds2::new(lower, upper))
}

fn distance(first: &Point2, second: &Point2) -> CurveResult<Real> {
    Ok(first.distance_squared(second).sqrt()?)
}

fn refined_length_bounds_for_controls(
    controls: Vec<Point2>,
    max_depth: usize,
) -> CurveResult<BezierLengthBounds2> {
    let mut lower = Real::zero();
    let mut upper = Real::zero();
    accumulate_refined_length_bounds(controls, max_depth, &mut lower, &mut upper)?;
    Ok(BezierLengthBounds2::new(lower, upper))
}

fn prefix_length_bounds_for_controls(
    controls: Vec<Point2>,
    t: Real,
    max_depth: usize,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BezierLengthBounds2>> {
    match in_closed_unit_interval(&t, policy) {
        Some(true) => {}
        Some(false) => return Err(CurveError::InvalidBezierParameter),
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
    let (prefix, _) = subdivide_controls_at(&controls, t)?;
    refined_length_bounds_for_controls(prefix, max_depth).map(Classification::Decided)
}

fn inverse_length_parameter_region_for_controls(
    controls: Vec<Point2>,
    target_length: Real,
    search_depth: usize,
    metric_depth: usize,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BezierArcLengthParameterRegion2>> {
    match compare_reals(&target_length, &Real::zero(), policy) {
        Some(Ordering::Less) => return Err(CurveError::InvalidBezierArcLengthTarget),
        Some(Ordering::Equal) => {
            let zero_bounds = BezierLengthBounds2::new(Real::zero(), Real::zero());
            let span = match arc_length_span(Real::zero(), Real::zero()) {
                Ok(span) => span,
                Err(reason) => return Ok(Classification::Uncertain(reason)),
            };
            return Ok(Classification::Decided(
                BezierArcLengthParameterRegion2::new(target_length, span, zero_bounds),
            ));
        }
        Some(Ordering::Greater) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }

    let total_bounds = refined_length_bounds_for_controls(controls.clone(), metric_depth)?;
    match compare_reals(&target_length, total_bounds.upper(), policy) {
        Some(Ordering::Greater) => return Err(CurveError::InvalidBezierArcLengthTarget),
        Some(Ordering::Less | Ordering::Equal) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }

    match exact_linear_parameter_inverse_length_region(&controls, target_length.clone(), policy)? {
        Classification::Decided(Some(region)) => return Ok(Classification::Decided(region)),
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let mut low = Real::zero();
    let mut high = Real::one();
    let two = Real::from(2_i8);
    for _ in 0..search_depth {
        let mid = ((&low + &high) / &two)?;
        let mid_bounds = match prefix_length_bounds_for_controls(
            controls.clone(),
            mid.clone(),
            metric_depth,
            policy,
        )? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let lower_cmp = compare_reals(&target_length, mid_bounds.lower(), policy);
        let upper_cmp = compare_reals(&target_length, mid_bounds.upper(), policy);
        match (lower_cmp, upper_cmp) {
            (Some(Ordering::Equal), Some(Ordering::Equal)) => {
                let span = match arc_length_span(mid.clone(), mid) {
                    Ok(span) => span,
                    Err(reason) => return Ok(Classification::Uncertain(reason)),
                };
                return Ok(Classification::Decided(
                    BezierArcLengthParameterRegion2::new(target_length, span, mid_bounds),
                ));
            }
            (Some(Ordering::Less), _) => high = mid,
            (_, Some(Ordering::Greater)) => low = mid,
            (Some(_), Some(_)) => break,
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }

    let high_bounds =
        match prefix_length_bounds_for_controls(controls, high.clone(), metric_depth, policy)? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let span = match arc_length_span(low, high) {
        Ok(span) => span,
        Err(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(
        BezierArcLengthParameterRegion2::new(target_length, span, high_bounds),
    ))
}

fn exact_linear_parameter_inverse_length_region(
    controls: &[Point2],
    target_length: Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<BezierArcLengthParameterRegion2>>> {
    match controls_are_degree_elevated_linear_parameterization(controls, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let (Some(start), Some(end)) = (controls.first(), controls.last()) else {
        return Ok(Classification::Decided(None));
    };
    let total = distance(start, end)?;
    match compare_reals(&target_length, &total, policy) {
        Some(Ordering::Greater) => return Err(CurveError::InvalidBezierArcLengthTarget),
        Some(Ordering::Less | Ordering::Equal) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
    match compare_reals(&total, &Real::zero(), policy) {
        Some(Ordering::Equal) => return Err(CurveError::InvalidBezierArcLengthTarget),
        Some(Ordering::Greater) => {}
        Some(Ordering::Less) => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }

    let parameter = (target_length.clone() / total)?;
    let bounds = BezierLengthBounds2::new(target_length.clone(), target_length.clone());
    let span = match arc_length_span(parameter.clone(), parameter) {
        Ok(span) => span,
        Err(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(Some(
        BezierArcLengthParameterRegion2::new(target_length, span, bounds),
    )))
}

fn arc_length_span(start: Real, end: Real) -> Result<BezierMonotoneSpan, UncertaintyReason> {
    BezierMonotoneSpan::new(start, end).map_err(|_| UncertaintyReason::Ordering)
}

fn controls_are_degree_elevated_linear_parameterization(
    controls: &[Point2],
    policy: &CurvePolicy,
) -> Classification<bool> {
    let Some(start) = controls.first() else {
        return Classification::Decided(false);
    };
    let Some(end) = controls.last() else {
        return Classification::Decided(false);
    };
    if controls.len() <= 1 {
        return Classification::Decided(false);
    }

    // A collinear control polygon is not enough: `P0, P1, P2 = 0, 1, 4`
    // traces a line image with nonlinear speed. The exact inverse-length
    // shortcut is valid only for the degree-elevated linear Bezier controls
    // `P_i = lerp(P0, P_n, i/n)`. This is the standard Bernstein
    // degree-elevation identity in Farin (2002), used as a certified metric
    // fact under Yap's exact-computation boundary (1997).
    let degree = controls.len() - 1;
    let denominator = Real::from(degree as i32);
    for (index, control) in controls
        .iter()
        .enumerate()
        .skip(1)
        .take(controls.len().saturating_sub(2))
    {
        let parameter = (Real::from(index as i32) / &denominator)
            .expect("division by positive degree is defined");
        let expected = start.lerp(end, parameter);
        match is_zero(&expected.distance_squared(control), policy) {
            Some(true) => {}
            Some(false) => return Classification::Decided(false),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(true)
}

fn accumulate_refined_length_bounds(
    controls: Vec<Point2>,
    remaining_depth: usize,
    lower: &mut Real,
    upper: &mut Real,
) -> CurveResult<()> {
    if remaining_depth == 0 {
        let refs = controls.iter().collect::<Vec<_>>();
        let bounds = length_bounds_for_controls(&refs)?;
        *lower = &*lower + bounds.lower;
        *upper = &*upper + bounds.upper;
        return Ok(());
    }

    let (left, right) = subdivide_controls_half(&controls)?;
    accumulate_refined_length_bounds(left, remaining_depth - 1, lower, upper)?;
    accumulate_refined_length_bounds(right, remaining_depth - 1, lower, upper)
}

fn subdivide_controls_half(controls: &[Point2]) -> CurveResult<(Vec<Point2>, Vec<Point2>)> {
    if controls.is_empty() {
        return Err(CurveError::InvalidBezierRange);
    }

    let mut levels = vec![controls.to_vec()];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let Some(previous) = levels.last() else {
            return Err(CurveError::InvalidBezierRange);
        };
        let next = previous
            .windows(2)
            .map(|pair| midpoint_point(&pair[0], &pair[1]))
            .collect::<Vec<_>>();
        levels.push(next);
    }

    let left = levels
        .iter()
        .map(|level| level[0].clone())
        .collect::<Vec<_>>();
    let right = levels
        .iter()
        .rev()
        .map(|level| level[level.len() - 1].clone())
        .collect::<Vec<_>>();
    Ok((left, right))
}

fn subdivide_controls_at(controls: &[Point2], t: Real) -> CurveResult<(Vec<Point2>, Vec<Point2>)> {
    if controls.is_empty() {
        return Err(CurveError::InvalidBezierRange);
    }

    let mut levels = vec![controls.to_vec()];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let Some(previous) = levels.last() else {
            return Err(CurveError::InvalidBezierRange);
        };
        let next = previous
            .windows(2)
            .map(|pair| pair[0].lerp(&pair[1], t.clone()))
            .collect::<Vec<_>>();
        levels.push(next);
    }

    let left = levels
        .iter()
        .map(|level| level[0].clone())
        .collect::<Vec<_>>();
    let right = levels
        .iter()
        .rev()
        .map(|level| level[level.len() - 1].clone())
        .collect::<Vec<_>>();
    Ok((left, right))
}

fn midpoint_point(first: &Point2, second: &Point2) -> Point2 {
    let two = Real::from(2_i8);
    Point2::new(
        ((first.x() + second.x()) / &two).expect("division by two is valid"),
        ((first.y() + second.y()) / two).expect("division by two is valid"),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn linear_quadratic() -> QuadraticBezier2 {
        QuadraticBezier2::new(point(0, 0), point(1, 0), point(2, 0))
    }

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
        }
    }

    #[test]
    fn inverse_length_zero_target_returns_zero_parameter_certificate() {
        let curve = linear_quadratic();
        let region = decided(
            curve
                .inverse_length_parameter_region(Real::zero(), 4, 2, &CurvePolicy::certified())
                .unwrap(),
        );

        assert_eq!(region.parameter_span().start(), &Real::zero());
        assert_eq!(region.parameter_span().end(), &Real::zero());
        assert!(region.prefix_bounds_at_span_end().is_exact());
    }

    #[test]
    fn inverse_length_linear_image_keeps_exact_parameter_certificate() {
        let curve = linear_quadratic();
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let region = decided(
            curve
                .inverse_length_parameter_region(Real::one(), 4, 2, &CurvePolicy::certified())
                .unwrap(),
        );

        assert_eq!(region.parameter_span().start(), &half);
        assert_eq!(region.parameter_span().end(), &half);
        assert_eq!(region.prefix_bounds_at_span_end().lower(), &Real::one());
        assert_eq!(region.prefix_bounds_at_span_end().upper(), &Real::one());
    }

    #[test]
    fn metric_subdivision_rejects_empty_controls() {
        assert_eq!(
            subdivide_controls_half(&[]),
            Err(CurveError::InvalidBezierRange)
        );
        assert_eq!(
            subdivide_controls_at(&[], Real::zero()),
            Err(CurveError::InvalidBezierRange)
        );
    }
}
