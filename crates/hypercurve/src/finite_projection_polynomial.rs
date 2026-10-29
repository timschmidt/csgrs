//! Polynomial display projection uses certified whole-arc boxes.

use super::*;
use crate::{Aabb2, PolynomialCurveBranch2, UncertaintyReason};
use std::cmp::Ordering;

fn uncertain(reason: UncertaintyReason) -> CurveError {
    CurveError::UncertainFiniteCurveProjection { reason }
}

pub(super) fn exact_error(error: crate::ExactCurveError) -> CurveError {
    match error {
        crate::ExactCurveError::Blocked(blocker) => uncertain(blocker.reason()),
        crate::ExactCurveError::Invalid { cause, .. } => cause,
    }
}

fn real(value: f64) -> CurveResult<Real> {
    Real::try_from(value).map_err(|_| CurveError::NonFiniteProjectionPoint)
}

fn leq(first: &Real, second: &Real, policy: &CurveContext) -> bool {
    matches!(
        crate::classify::compare_reals(first, second, policy),
        Some(Ordering::Less | Ordering::Equal)
    )
}

fn all_corners(bounds: &Aabb2, mut predicate: impl FnMut(&Real, &Real) -> bool) -> bool {
    for x in [bounds.min().x(), bounds.max().x()] {
        for y in [bounds.min().y(), bounds.max().y()] {
            if !predicate(x, y) {
                return false;
            }
        }
    }
    true
}

fn project_point(point: &CurvePoint2, policy: &CurveContext) -> CurveResult<[f64; 2]> {
    if let Some(point) = point.coordinates() {
        return finite_point(point);
    }
    for steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let bounds = match algebraic_chord_endpoint_bounds_refined(point, steps, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                return Err(uncertain(UncertaintyReason::BudgetExhausted));
            }
            Classification::Uncertain(_) => continue,
        };
        let lower = finite_point(bounds.min())?;
        let upper = finite_point(bounds.max())?;
        if lower == upper {
            return Ok(lower);
        }
    }
    Err(uncertain(UncertaintyReason::BudgetExhausted))
}

fn certify_endpoint(
    point: &CurvePoint2,
    projected: [f64; 2],
    squared_error: &Real,
    policy: &CurveContext,
) -> CurveResult<()> {
    let [px, py] = [real(projected[0])?, real(projected[1])?];
    for steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let bounds = match algebraic_chord_endpoint_bounds_refined(point, steps, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                return Err(uncertain(UncertaintyReason::BudgetExhausted));
            }
            Classification::Uncertain(_) => continue,
        };
        if all_corners(&bounds, |x, y| {
            let dx = x - &px;
            let dy = y - &py;
            leq(&(&dx * &dx + &dy * &dy), squared_error, policy)
        }) {
            return Ok(());
        }
    }
    // A finite coordinate can still be too coarsely quantized for this error.
    // Never silently enlarge the caller's tolerance to its binary64 spacing.
    Err(uncertain(UncertaintyReason::Predicate))
}

fn inside_capsule(
    bounds: &Aabb2,
    start: [f64; 2],
    end: [f64; 2],
    squared_error: &Real,
    policy: &CurveContext,
) -> CurveResult<bool> {
    let [ax, ay] = [real(start[0])?, real(start[1])?];
    let [bx, by] = [real(end[0])?, real(end[1])?];
    let dx = &bx - &ax;
    let dy = &by - &ay;
    let length_squared = &dx * &dx + &dy * &dy;
    // A segment's closed error capsule is convex. Containing every rectangle
    // corner therefore contains the entire certified curve enclosure.
    Ok(all_corners(bounds, |x, y| {
        let vx = x - &ax;
        let vy = y - &ay;
        let dot = &vx * &dx + &vy * &dy;
        if length_squared.definitely_zero() || leq(&dot, &Real::zero(), policy) {
            return leq(&(&vx * &vx + &vy * &vy), squared_error, policy);
        }
        if leq(&length_squared, &dot, policy) {
            let ex = x - &bx;
            let ey = y - &by;
            return leq(&(&ex * &ex + &ey * &ey), squared_error, policy);
        }
        // Both comparisons must be definite before choosing the strip case.
        if crate::classify::compare_reals(&dot, &Real::zero(), policy) != Some(Ordering::Greater)
            || crate::classify::compare_reals(&dot, &length_squared, policy) != Some(Ordering::Less)
        {
            return false;
        }
        let cross = &vx * &dy - &vy * &dx;
        leq(
            &(&cross * &cross),
            &(squared_error * &length_squared),
            policy,
        )
    }))
}

pub(super) fn append(
    points: &mut Vec<[f64; 2]>,
    curve: &Curve2,
    branch: &PolynomialCurveBranch2,
    options: &FiniteProjectionOptions,
    policy: &CurveContext,
    boxes: &mut usize,
) -> CurveResult<()> {
    policy.strict_predicate_pass(|| {
        let error = real(options.chord_error)?;
        let error_squared = &error * &error;
        let endpoint_error_squared = (&error_squared / Real::from(16))?;
        let start = points
            .last()
            .copied()
            .map_or_else(|| project_point(&curve.start(), policy), Ok)?;
        let end = project_point(&curve.end(), policy)?;
        certify_endpoint(&curve.start(), start, &endpoint_error_squared, policy)?;
        certify_endpoint(&curve.end(), end, &endpoint_error_squared, policy)?;
        if points.is_empty() {
            points.push(start);
        }
        let margin = (&error / Real::from(4))?;
        let mut sampler = Sampler {
            branch,
            options,
            policy,
            boxes,
            error_squared,
            endpoint_error_squared,
        };
        sampler.interval(
            points,
            branch.parameter_range().start(),
            branch.parameter_range().end(),
            start,
            end,
            &margin,
            0,
        )
    })
}

struct Sampler<'a> {
    branch: &'a PolynomialCurveBranch2,
    options: &'a FiniteProjectionOptions,
    policy: &'a CurveContext,
    boxes: &'a mut usize,
    error_squared: Real,
    endpoint_error_squared: Real,
}

impl Sampler<'_> {
    #[allow(clippy::too_many_arguments)]
    fn interval(
        &mut self,
        points: &mut Vec<[f64; 2]>,
        lower_parameter: &CurveParameter2,
        upper_parameter: &CurveParameter2,
        start: [f64; 2],
        end: [f64; 2],
        margin: &Real,
        depth: usize,
    ) -> CurveResult<()> {
        if *self.boxes >= self.options.polynomial_max_boxes {
            return Err(uncertain(UncertaintyReason::BudgetExhausted));
        }
        *self.boxes += 1;
        let mut separated = None;
        for steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let lower = finite_region_parameter_projection(lower_parameter, steps, self.policy)?;
            let upper = finite_region_parameter_projection(upper_parameter, steps, self.policy)?;
            let gap = &upper.lower - &lower.upper;
            let uncertainty = (&lower.upper - &lower.lower) + (&upper.upper - &upper.lower);
            // Separation alone permits midpoints to converge to a stale lower
            // isolator instead of the selected endpoint. Bound uncertainty by
            // the gap so each child makes progress in the actual domain.
            if crate::classify::compare_reals(&gap, &Real::zero(), self.policy)
                == Some(Ordering::Greater)
                && leq(&(uncertainty * Real::from(2)), &gap, self.policy)
            {
                separated = Some((lower, upper));
                break;
            }
        }
        let (lower, upper) =
            separated.ok_or_else(|| uncertain(UncertaintyReason::BudgetExhausted))?;
        let midpoint = ((&lower.upper + &upper.lower) / Real::from(2))?;
        let (bounds, anchor) = self
            .branch
            .projection_enclosure([&lower.lower, &upper.upper], &midpoint, margin, self.policy)
            .map_err(exact_error)?;
        if let Some(bounds) = &bounds
            && inside_capsule(bounds, start, end, &self.error_squared, self.policy)?
        {
            push_if_new(points, end);
            return Ok(());
        }
        if depth >= self.options.polynomial_max_depth {
            return Err(uncertain(UncertaintyReason::BudgetExhausted));
        }
        let middle = project_point(&anchor, self.policy)?;
        certify_endpoint(&anchor, middle, &self.endpoint_error_squared, self.policy)?;
        let next_margin = if bounds.is_some() {
            (margin / Real::from(2))?
        } else {
            margin.clone()
        };
        let parameter = midpoint.into();
        if self.branch.is_reversed() {
            self.interval(
                points,
                &parameter,
                upper_parameter,
                start,
                middle,
                &next_margin,
                depth + 1,
            )?;
            self.interval(
                points,
                lower_parameter,
                &parameter,
                middle,
                end,
                &next_margin,
                depth + 1,
            )
        } else {
            self.interval(
                points,
                lower_parameter,
                &parameter,
                start,
                middle,
                &next_margin,
                depth + 1,
            )?;
            self.interval(
                points,
                &parameter,
                upper_parameter,
                middle,
                end,
                &next_margin,
                depth + 1,
            )
        }
    }
}
