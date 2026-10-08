//! Exact minimum distances on closed native segments.
//!
//! A minimum on two compact smooth spans is either an intersection, has an
//! endpoint, or is stationary in both interiors. For a line and circle the
//! interior connector is normal to the line; for distinct circle centers it
//! lies on their center line. Concentric arcs with overlapping angular ranges
//! have an endpoint direction in that overlap, so endpoint/radial candidates
//! also cover that case. All candidate admission and ordering stays exact.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::classify::compare_reals;
use crate::{
    ArcArcIntersection, CircularArc2, Classification, CurveContext, CurveError, CurveOperation2,
    ExactCurveError, ExactCurveResult, LineArcIntersection, LineLineIntersection, LineSeg2, Point2,
    Segment2, SegmentIntersection, UncertaintyReason,
};

impl Segment2 {
    /// Minimum squared Euclidean distance to another closed native segment.
    ///
    /// Measures the exact line or circular arc, including endpoints, tangencies,
    /// overlaps, and concentric arcs. An unchecked zero-length line represents
    /// its single endpoint. No finite projection participates in this query.
    /// Undecided intersections, sweep membership, or ordering return a typed
    /// blocker; an unavailable candidate is never discarded from the minimum.
    ///
    /// Uses the active curve policy, including [`crate::evaluate_under`] and
    /// [`crate::evaluate_with_budget`]. The candidate count is constant; this
    /// does not bound scalar precision, expression growth, or elapsed time.
    pub fn distance_squared(&self, other: &Self) -> ExactCurveResult<Real> {
        let context = MetricContext(crate::policy::principal_context());
        context.check_budget()?;
        let result = context.segment_distance(self, other)?;
        context.check_budget()?;
        Ok(result)
    }

    /// Minimum squared Euclidean distance from an exact point to this segment.
    ///
    /// Uses exact endpoint, perpendicular-foot, or radial candidates with the
    /// same policy and failure contract as [`Self::distance_squared`].
    pub fn point_distance_squared(&self, point: &Point2) -> ExactCurveResult<Real> {
        let context = MetricContext(crate::policy::principal_context());
        context.check_budget()?;
        let result = context.point_distance(point, self)?;
        context.check_budget()?;
        Ok(result)
    }
}

struct MetricContext(CurveContext);

impl MetricContext {
    fn blocker(&self, reason: UncertaintyReason) -> ExactCurveError {
        ExactCurveError::blocked_unattributed(
            CurveOperation2::Evaluation,
            if hypersolve::work_budget::exhaustion().is_some() {
                UncertaintyReason::BudgetExhausted
            } else {
                reason
            },
        )
    }

    fn check_budget(&self) -> ExactCurveResult<()> {
        if hypersolve::work_budget::exhaustion().is_some() {
            Err(self.blocker(UncertaintyReason::BudgetExhausted))
        } else {
            Ok(())
        }
    }

    fn invalid(cause: impl Into<CurveError>) -> ExactCurveError {
        ExactCurveError::invalid_unattributed(CurveOperation2::Evaluation, cause.into())
    }

    fn compare(&self, left: &Real, right: &Real) -> ExactCurveResult<Ordering> {
        self.check_budget()?;
        compare_reals(left, right, &self.0).ok_or_else(|| self.blocker(UncertaintyReason::Ordering))
    }

    fn decided<T>(&self, value: Classification<T>) -> ExactCurveResult<T> {
        self.check_budget()?;
        match value {
            Classification::Decided(value) => Ok(value),
            Classification::Uncertain(reason) => Err(self.blocker(reason)),
        }
    }

    fn retain_minimum(&self, minimum: &mut Real, candidate: Real) -> ExactCurveResult<()> {
        if self.compare(&candidate, minimum)? == Ordering::Less {
            *minimum = candidate;
        }
        Ok(())
    }

    fn point_distance(&self, point: &Point2, segment: &Segment2) -> ExactCurveResult<Real> {
        match segment {
            Segment2::Line(line) => self.point_line_distance(point, line),
            Segment2::Arc(arc) => self.point_arc_distance(point, arc),
        }
    }

    fn point_line_distance(&self, point: &Point2, line: &LineSeg2) -> ExactCurveResult<Real> {
        let (dx, dy) = line.delta();
        let length_squared = line.length_squared();
        if self.compare(&length_squared, &Real::zero())? == Ordering::Equal {
            return Ok(point.distance_squared(line.start()));
        }
        let (px, py) = point.delta_from(line.start());
        let numerator = Real::signed_product_sum([true; 2], [[&px, &dx], [&py, &dy]]);
        if self.compare(&numerator, &Real::zero())? != Ordering::Greater {
            return Ok(point.distance_squared(line.start()));
        }
        if self.compare(&numerator, &length_squared)? != Ordering::Less {
            return Ok(point.distance_squared(line.end()));
        }
        // Squared perpendicular distance avoids subtracting two nearly equal
        // squared lengths and does not construct a new projected point.
        let cross = Real::diff_of_products(&dx, &py, &dy, &px);
        (&cross * &cross / length_squared).map_err(Self::invalid)
    }

    fn point_arc_distance(&self, point: &Point2, arc: &CircularArc2) -> ExactCurveResult<Real> {
        let center_distance_squared = point.distance_squared(arc.center());
        if self.compare(&center_distance_squared, &Real::zero())? == Ordering::Equal {
            return Ok(arc.radius_squared());
        }
        if self.decided(arc.contains_sweep_point_with_policy(point, &self.0))? {
            let delta = center_distance_squared.sqrt().map_err(Self::invalid)?
                - arc.radius_squared().sqrt().map_err(Self::invalid)?;
            return Ok(&delta * &delta);
        }
        let mut minimum = point.distance_squared(arc.start());
        self.retain_minimum(&mut minimum, point.distance_squared(arc.end()))?;
        Ok(minimum)
    }

    fn segment_distance(&self, left: &Segment2, right: &Segment2) -> ExactCurveResult<Real> {
        for (candidate, other) in [(left, right), (right, left)] {
            if let Segment2::Line(line) = candidate
                && self.compare(&line.length_squared(), &Real::zero())? == Ordering::Equal
            {
                return self.point_distance(line.start(), other);
            }
        }
        let intersection = left
            .intersect_segment_with_policy(right, &self.0)
            .map_err(Self::invalid)?;
        match &intersection {
            SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason })
            | SegmentIntersection::LineArc {
                result: LineArcIntersection::Uncertain { reason },
                ..
            }
            | SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
                return Err(self.blocker(*reason));
            }
            _ if !intersection.is_none() => return Ok(Real::zero()),
            _ => {}
        }
        let mut minimum = self.point_distance(left.start(), right)?;
        for (point, segment) in [
            (left.end(), right),
            (right.start(), left),
            (right.end(), left),
        ] {
            self.retain_minimum(&mut minimum, self.point_distance(point, segment)?)?;
        }
        match (left, right) {
            (Segment2::Line(_), Segment2::Line(_)) => {}
            (Segment2::Line(line), Segment2::Arc(arc))
            | (Segment2::Arc(arc), Segment2::Line(line)) => {
                self.line_arc_stationary(line, arc, &mut minimum)?;
            }
            (Segment2::Arc(left), Segment2::Arc(right)) => {
                self.arc_arc_stationary(left, right, &mut minimum)?;
            }
        }
        Ok(minimum)
    }

    fn line_arc_stationary(
        &self,
        line: &LineSeg2,
        arc: &CircularArc2,
        minimum: &mut Real,
    ) -> ExactCurveResult<()> {
        let (dx, dy) = line.delta();
        let length_squared = line.length_squared();
        let (cx, cy) = arc.center().delta_from(line.start());
        let numerator = Real::signed_product_sum([true; 2], [[&cx, &dx], [&cy, &dy]]);
        if self.compare(&numerator, &Real::zero())? == Ordering::Less
            || self.compare(&numerator, &length_squared)? == Ordering::Greater
        {
            return Ok(());
        }
        let signed_height = (Real::diff_of_products(&dx, &cy, &dy, &cx)
            / length_squared.sqrt().map_err(Self::invalid)?)
        .map_err(Self::invalid)?;
        let radius = arc.radius_squared().sqrt().map_err(Self::invalid)?;
        for sign in [-Real::one(), Real::one()] {
            // Sweep membership only needs a direction from the center, so the
            // normal need not be normalized or rounded to a circle point.
            let direction = Point2::new(
                arc.center().x() - &sign * &dy,
                arc.center().y() + &sign * &dx,
            );
            if self.decided(arc.contains_sweep_point_with_policy(&direction, &self.0))? {
                let delta = &signed_height + sign * &radius;
                self.retain_minimum(minimum, &delta * &delta)?;
            }
        }
        Ok(())
    }

    fn arc_arc_stationary(
        &self,
        left: &CircularArc2,
        right: &CircularArc2,
        minimum: &mut Real,
    ) -> ExactCurveResult<()> {
        let center_distance_squared = left.center().distance_squared(right.center());
        if self.compare(&center_distance_squared, &Real::zero())? == Ordering::Equal {
            return Ok(());
        }
        let (dx, dy) = right.center().delta_from(left.center());
        let distance = center_distance_squared.sqrt().map_err(Self::invalid)?;
        let left_radius = left.radius_squared().sqrt().map_err(Self::invalid)?;
        let right_radius = right.radius_squared().sqrt().map_err(Self::invalid)?;
        for left_sign in [-Real::one(), Real::one()] {
            let left_direction = Point2::new(
                left.center().x() + &left_sign * &dx,
                left.center().y() + &left_sign * &dy,
            );
            if !self.decided(left.contains_sweep_point_with_policy(&left_direction, &self.0))? {
                continue;
            }
            for right_sign in [-Real::one(), Real::one()] {
                let right_direction = Point2::new(
                    right.center().x() + &right_sign * &dx,
                    right.center().y() + &right_sign * &dy,
                );
                if self
                    .decided(right.contains_sweep_point_with_policy(&right_direction, &self.0))?
                {
                    let delta = &distance + right_sign * &right_radius - &left_sign * &left_radius;
                    self.retain_minimum(minimum, &delta * &delta)?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
