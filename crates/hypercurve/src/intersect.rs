//! Intersection result types and early segment topology.
//!
//! The primitive line-line, line-circle, and circle-circle formulas are the
//! standard parametric constructions. This module
//! keeps their algebraic branch points explicit so sign/order uncertainty can
//! propagate instead of being hidden behind a global epsilon.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::classify::{
    at_unit_interval_endpoint, compare_reals, in_closed_unit_interval, is_zero, max_real, min_real,
    sort_pair,
};
use crate::segment::RetainedLineRelation2;
use crate::{
    CircularArc2, Classification, CurveError, CurvePolicy, CurveResult, LineSeg2, NumericMode,
    Point2, Segment2, UncertaintyReason,
};

/// Parameter range on a segment.
#[derive(Clone, Debug, PartialEq)]
pub struct ParamRange {
    start: Real,
    end: Real,
}

impl ParamRange {
    /// Constructs a parameter range.
    pub const fn new(start: Real, end: Real) -> Self {
        Self { start, end }
    }

    /// Range start.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Range end.
    pub const fn end(&self) -> &Real {
        &self.end
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct OrientedParamRangeOverlap {
    pub(crate) first: ParamRange,
    pub(crate) second: ParamRange,
    pub(crate) same_orientation: bool,
}

pub(crate) fn oriented_param_range_overlap(
    first: &ParamRange,
    second: &ParamRange,
    policy: &CurvePolicy,
) -> Classification<Option<OrientedParamRangeOverlap>> {
    let first_direction = match compare_reals(first.start(), first.end(), policy) {
        Some(Ordering::Equal) => return Classification::Decided(None),
        Some(direction) => direction,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let second_direction = match compare_reals(second.start(), second.end(), policy) {
        Some(Ordering::Equal) => return Classification::Decided(None),
        Some(direction) => direction,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let (first_low, first_high) = if first_direction == Ordering::Less {
        (first.start(), first.end())
    } else {
        (first.end(), first.start())
    };
    let (second_low, second_high) = if second_direction == Ordering::Less {
        (second.start(), second.end())
    } else {
        (second.end(), second.start())
    };
    let overlap_low = match compare_reals(first_low, second_low, policy) {
        Some(Ordering::Less) => second_low,
        Some(_) => first_low,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let overlap_high = match compare_reals(first_high, second_high, policy) {
        Some(Ordering::Greater) => second_high,
        Some(_) => first_high,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    match compare_reals(overlap_low, overlap_high, policy) {
        Some(Ordering::Less) => {}
        Some(Ordering::Equal | Ordering::Greater) => return Classification::Decided(None),
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    }

    let (source_start, source_end) = if first_direction == Ordering::Less {
        (overlap_low, overlap_high)
    } else {
        (overlap_high, overlap_low)
    };
    let Some(first_start) = local_parameter_in_range(first, source_start) else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    let Some(first_end) = local_parameter_in_range(first, source_end) else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    let Some(second_start) = local_parameter_in_range(second, source_start) else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    let Some(second_end) = local_parameter_in_range(second, source_end) else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    Classification::Decided(Some(OrientedParamRangeOverlap {
        first: ParamRange::new(first_start, first_end),
        second: ParamRange::new(second_start, second_end),
        same_orientation: first_direction == second_direction,
    }))
}

fn local_parameter_in_range(range: &ParamRange, source_parameter: &Real) -> Option<Real> {
    ((source_parameter - range.start()) / (range.end() - range.start())).ok()
}

/// Local intersection kind.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IntersectionKind {
    /// Proper crossing at both segment interiors.
    Crossing,
    /// Contact at one or more segment endpoints.
    Endpoint,
    /// Tangential contact away from segment endpoints.
    Tangent,
    /// Collinear overlap.
    Overlap,
}

/// Intersection between two line segments.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum LineLineIntersection {
    /// No intersection.
    None,
    /// A single intersection point.
    Point {
        /// Intersection point.
        point: Point2,
        /// Parameter on the first segment.
        a_param: Real,
        /// Parameter on the second segment.
        b_param: Real,
        /// Local kind of point contact.
        kind: IntersectionKind,
    },
    /// A collinear overlapping interval.
    Overlap {
        /// Overlapping segment geometry.
        segment: LineSeg2,
        /// Parameter range on the first segment.
        a_range: ParamRange,
        /// Parameter range on the second segment.
        b_range: ParamRange,
    },
    /// The active policy could not classify this relation.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl LineLineIntersection {
    /// Returns true when this result has no intersection.
    pub const fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }
}

/// Relation between an oriented supporting line and a full circle.
///
/// The line is represented by a finite [`LineSeg2`], but this relation is
/// intentionally about the infinite supporting line. The returned parameters
/// are affine coordinates on that segment's support, so downstream segment and
/// arc filters can reuse the same roots without recomputing the quadratic. This
/// is the line-circle primitive described in standard geometric constructions, exposed as a separate
/// exact predicate boundary in the exactness model's sense: derive the algebraic relation once,
/// then let higher-level objects decide which roots lie on their finite
/// topology.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum LineCircleRelation {
    /// The supporting line does not meet the circle.
    Disjoint,
    /// The supporting line touches the circle at one point.
    Tangent {
        /// Tangency point.
        point: Point2,
        /// Affine parameter on the line support.
        line_param: Real,
    },
    /// The supporting line crosses the circle at two points.
    Secant {
        /// First point in increasing line-parameter order.
        first_point: Point2,
        /// First affine parameter on the line support.
        first_param: Real,
        /// Second point in increasing line-parameter order.
        second_point: Point2,
        /// Second affine parameter on the line support.
        second_param: Real,
    },
    /// The active policy could not decide the relation.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl LineCircleRelation {
    /// Returns true when the supporting line is disjoint from the circle.
    pub const fn is_disjoint(&self) -> bool {
        matches!(self, Self::Disjoint)
    }

    /// Returns true when the supporting line has exactly one tangent contact.
    pub const fn is_tangent(&self) -> bool {
        matches!(self, Self::Tangent { .. })
    }

    /// Returns true when the supporting line crosses the circle at two points.
    pub const fn is_secant(&self) -> bool {
        matches!(self, Self::Secant { .. })
    }
}

/// One point in a line-arc intersection result.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArcIntersectionPoint {
    /// Intersection point.
    pub point: Point2,
    /// Parameter on the line segment.
    pub line_param: Real,
    /// Exact directed-angular sweep fraction on the arc segment.
    ///
    /// This is retained event-ordering evidence in `[0, 1]`. It supports minor,
    /// semicircular, major, and full-circle traversal and is not a
    /// primitive-float approximation.
    pub arc_param: Real,
    /// Local kind of point contact.
    pub kind: IntersectionKind,
}

/// Intersection between a line segment and a circular arc.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum LineArcIntersection {
    /// No intersection.
    None,
    /// A single intersection point.
    Point(LineArcIntersectionPoint),
    /// Two intersection points, ordered by line parameter.
    TwoPoints {
        /// First hit along the line.
        first: LineArcIntersectionPoint,
        /// Second hit along the line.
        second: LineArcIntersectionPoint,
    },
    /// The active policy could not classify this relation.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl LineArcIntersection {
    /// Returns true when this result has no intersection.
    pub const fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }
}

/// Relation between two full circles supporting circular arcs.
///
/// This is deliberately a circle predicate, not an arc predicate: coincident,
/// disjoint, tangent, and secant outcomes are classified before either arc's
/// angular sweep is considered. Arc-arc intersection then filters the returned
/// point witnesses through finite sweep predicates. The radical-axis
/// construction is the standard circle-circle primitive in standard geometric constructions;
/// exposing it separately follows the exactness model's recommendation to keep exact algebraic
/// predicates at explicit object boundaries.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum CircleCircleRelation {
    /// The two full circles are the same circle.
    Coincident,
    /// The two full circles do not meet.
    Disjoint,
    /// The two full circles touch at one point.
    Tangent {
        /// Tangency point.
        point: Point2,
    },
    /// The two full circles cross at two points.
    Secant {
        /// First point in deterministic radical-axis construction order.
        first_point: Point2,
        /// Second point in deterministic radical-axis construction order.
        second_point: Point2,
    },
    /// The active policy could not decide the relation.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl CircleCircleRelation {
    /// Returns true when the circles are coincident.
    pub const fn is_coincident(&self) -> bool {
        matches!(self, Self::Coincident)
    }

    /// Returns true when the circles are disjoint.
    pub const fn is_disjoint(&self) -> bool {
        matches!(self, Self::Disjoint)
    }

    /// Returns true when the circles have exactly one tangent contact.
    pub const fn is_tangent(&self) -> bool {
        matches!(self, Self::Tangent { .. })
    }

    /// Returns true when the circles have two crossings.
    pub const fn is_secant(&self) -> bool {
        matches!(self, Self::Secant { .. })
    }
}

/// One point in an arc-arc intersection result.
#[derive(Clone, Debug, PartialEq)]
pub struct ArcArcIntersectionPoint {
    /// Intersection point.
    pub point: Point2,
    /// Exact directed-angular sweep fraction on the first arc segment.
    pub a_param: Real,
    /// Exact directed-angular sweep fraction on the second arc segment.
    pub b_param: Real,
    /// Local kind of point contact.
    pub kind: IntersectionKind,
}

/// Intersection between two circular arcs.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum ArcArcIntersection {
    /// No intersection.
    None,
    /// A single intersection point.
    Point(ArcArcIntersectionPoint),
    /// Two intersection points.
    TwoPoints {
        /// First hit in deterministic construction order.
        first: ArcArcIntersectionPoint,
        /// Second hit in deterministic construction order.
        second: ArcArcIntersectionPoint,
    },
    /// A same-circle overlapping arc interval.
    Overlap {
        /// Overlapping arc geometry, oriented in the first arc's direction.
        segment: CircularArc2,
        /// Parameter range on the first arc segment.
        a_range: ParamRange,
        /// Parameter range on the second arc segment.
        b_range: ParamRange,
    },
    /// The active policy could not classify this relation, or the relation is
    /// outside this slice.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl ArcArcIntersection {
    /// Returns true when this result has no intersection.
    pub const fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }
}

/// Operand order for a line-arc segment intersection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineArcOrder {
    /// The first operand is the line and the second operand is the arc.
    LineThenArc,
    /// The first operand is the arc and the second operand is the line.
    ArcThenLine,
}

/// Intersection between two native segments.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum SegmentIntersection {
    /// Line-line relation.
    LineLine(LineLineIntersection),
    /// Line-arc relation, with explicit operand order.
    LineArc {
        /// Whether the original operands were line-then-arc or arc-then-line.
        order: LineArcOrder,
        /// The line-arc intersection result. Point parameters are on the line.
        result: LineArcIntersection,
    },
    /// Arc-arc relation.
    ArcArc(ArcArcIntersection),
}

impl SegmentIntersection {
    /// Returns true when this result has no intersection.
    pub const fn is_none(&self) -> bool {
        match self {
            Self::LineLine(result) => result.is_none(),
            Self::LineArc { result, .. } => result.is_none(),
            Self::ArcArc(result) => result.is_none(),
        }
    }
}

impl Segment2 {
    /// Intersects this segment with another native segment.
    pub fn intersect_segment(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<SegmentIntersection> {
        match (self, other) {
            (Self::Line(a), Self::Line(b)) => a
                .intersect_line(b, policy)
                .map(SegmentIntersection::LineLine),
            (Self::Line(line), Self::Arc(arc)) => Ok(SegmentIntersection::LineArc {
                order: LineArcOrder::LineThenArc,
                result: line.intersect_arc(arc, policy)?,
            }),
            (Self::Arc(arc), Self::Line(line)) => Ok(SegmentIntersection::LineArc {
                order: LineArcOrder::ArcThenLine,
                result: line.intersect_arc(arc, policy)?,
            }),
            (Self::Arc(a), Self::Arc(b)) => {
                a.intersect_arc(b, policy).map(SegmentIntersection::ArcArc)
            }
        }
    }
}

impl LineSeg2 {
    /// Intersects this line segment with another line segment.
    ///
    /// Uses the standard parametric cross-product relation
    /// `p + t r = q + u s`. Parallel, collinear, point, and overlap cases stay
    /// separate because polygon clipping degeneracies need those distinctions
    /// later in the boolean pipeline.
    pub fn intersect_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<LineLineIntersection> {
        if line_segments_decided_axis_separated(self, other, policy) {
            return Ok(LineLineIntersection::None);
        }
        if self.retained_support_ranges_decided_disjoint(other, policy) == Some(true) {
            return Ok(LineLineIntersection::None);
        }
        if let Some(relation) = self.retained_offset_relation(other, policy) {
            return match relation {
                RetainedLineRelation2::Coincident => intersect_collinear(self, other, policy),
                RetainedLineRelation2::ParallelDistinct => Ok(LineLineIntersection::None),
                RetainedLineRelation2::Uncertain => Ok(LineLineIntersection::Uncertain {
                    reason: UncertaintyReason::RealSign,
                }),
            };
        }

        let (rx, ry) = self.delta();
        let (sx, sy) = other.delta();
        let qmp = other.start().delta_from(self.start());
        let (support_rx, support_ry) = if self.has_retained_support() {
            self.support_delta()
        } else {
            (rx.clone(), ry.clone())
        };
        let (support_sx, support_sy) = if other.has_retained_support() {
            other.support_delta()
        } else {
            (sx.clone(), sy.clone())
        };
        let support_qmp = other.support_start().delta_from(self.support_start());

        let denominator = cross(&support_rx, &support_ry, &support_sx, &support_sy);
        match is_zero(&denominator, policy) {
            Some(false) => intersect_non_parallel(self, other, policy, &rx, &ry, &sx, &sy, qmp),
            Some(true) => {
                intersect_parallel(self, other, policy, &support_rx, &support_ry, support_qmp)
            }
            None if line_segments_decided_axis_separated(self, other, policy) => {
                Ok(LineLineIntersection::None)
            }
            None => Ok(LineLineIntersection::Uncertain {
                reason: UncertaintyReason::RealSign,
            }),
        }
    }

    /// Intersects this line segment with a circular arc.
    ///
    /// Substitutes the segment's affine parameter into the circle equation and
    /// classifies the resulting quadratic roots once at the supporting-line
    /// layer, then filters those roots by the finite line interval and by the
    /// arc sweep. Keeping the line-circle relation explicit gives future
    /// line/arc batches a reusable exact predicate surface without moving arc
    /// topology into the algebraic primitive.
    pub fn intersect_arc(
        &self,
        arc: &CircularArc2,
        policy: &CurvePolicy,
    ) -> CurveResult<LineArcIntersection> {
        match self.supporting_line_circle_relation(arc, policy)? {
            LineCircleRelation::Disjoint => Ok(LineArcIntersection::None),
            LineCircleRelation::Tangent { line_param, .. } => {
                match line_arc_hit_candidate(
                    self,
                    arc,
                    line_param,
                    IntersectionKind::Tangent,
                    policy,
                )? {
                    LineArcCandidate::Hit(hit) => Ok(LineArcIntersection::Point(hit)),
                    LineArcCandidate::Miss => Ok(LineArcIntersection::None),
                    LineArcCandidate::Uncertain(reason) => {
                        Ok(LineArcIntersection::Uncertain { reason })
                    }
                }
            }
            LineCircleRelation::Secant {
                first_param,
                second_param,
                ..
            } => line_arc_two_candidates(self, arc, first_param, second_param, policy),
            LineCircleRelation::Uncertain { reason } => {
                Ok(LineArcIntersection::Uncertain { reason })
            }
        }
    }

    /// Classifies the relation between this segment's supporting line and an
    /// arc's full circle.
    ///
    /// This method ignores the finite line interval and the finite arc sweep.
    /// It returns affine parameters on the line support so callers can perform
    /// those object-level filters without recomputing the line-circle
    /// quadratic. Structural-dispatch note: exact-rational line and circle
    /// facts carried by prepared curve objects can later select specialized
    /// discriminant reducers here while preserving this public relation shape.
    pub fn supporting_line_circle_relation(
        &self,
        arc: &CircularArc2,
        policy: &CurvePolicy,
    ) -> CurveResult<LineCircleRelation> {
        let (dx, dy) = self.delta();
        let start_from_center = self.start().delta_from(arc.center());
        let a = dot(&dx, &dy, &dx, &dy);
        let half_b = dot(&start_from_center.0, &start_from_center.1, &dx, &dy);
        let c = dot(
            &start_from_center.0,
            &start_from_center.1,
            &start_from_center.0,
            &start_from_center.1,
        ) - arc.radius_squared();
        let discriminant = (&half_b * &half_b) - (&a * &c);

        match crate::classify::real_sign(&discriminant, policy) {
            Some(RealSign::Negative) => Ok(LineCircleRelation::Disjoint),
            Some(RealSign::Zero) => {
                let t = ((-half_b) / &a)?;
                let point = line_point_at_for_policy(self, &t, policy)?;
                Ok(LineCircleRelation::Tangent {
                    point,
                    line_param: t,
                })
            }
            Some(RealSign::Positive) => {
                let sqrt_discriminant = discriminant.clone().sqrt()?;
                let negative_half_b = -half_b;
                let t0 = ((&negative_half_b - &sqrt_discriminant) / &a)?;
                let t1 = ((negative_half_b.clone() + sqrt_discriminant) / &a)?;
                let (first_param, second_param) = match compare_reals(&t0, &t1, policy) {
                    Some(Ordering::Greater) => (t1, t0),
                    Some(Ordering::Less | Ordering::Equal) => (t0, t1),
                    None => {
                        return Ok(LineCircleRelation::Uncertain {
                            reason: UncertaintyReason::Ordering,
                        });
                    }
                };
                let first_point = line_point_at_for_policy(self, &first_param, policy)?;
                let second_point = line_point_at_for_policy(self, &second_param, policy)?;
                Ok(LineCircleRelation::Secant {
                    first_point,
                    first_param,
                    second_point,
                    second_param,
                })
            }
            None => Ok(LineCircleRelation::Uncertain {
                reason: UncertaintyReason::RealSign,
            }),
        }
    }
}

fn line_segments_decided_axis_separated(
    first: &LineSeg2,
    second: &LineSeg2,
    policy: &CurvePolicy,
) -> bool {
    fn endpoints_strictly_before(
        first: [&Real; 2],
        second: [&Real; 2],
        policy: &CurvePolicy,
    ) -> bool {
        first.into_iter().all(|left| {
            second
                .iter()
                .all(|right| compare_reals(left, right, policy) == Some(Ordering::Less))
        })
    }

    let first_x = [first.start().x(), first.end().x()];
    let second_x = [second.start().x(), second.end().x()];
    if endpoints_strictly_before(first_x, second_x, policy)
        || endpoints_strictly_before(second_x, first_x, policy)
    {
        return true;
    }

    let first_y = [first.start().y(), first.end().y()];
    let second_y = [second.start().y(), second.end().y()];
    endpoints_strictly_before(first_y, second_y, policy)
        || endpoints_strictly_before(second_y, first_y, policy)
}

impl CircularArc2 {
    /// Classifies the relation between this arc's full circle and another.
    ///
    /// This ignores both finite arc sweeps. The returned point witnesses are
    /// therefore suitable for exact arc-arc filtering, containment probes, and
    /// future batch dispatch without recomputing the radical-axis algebra.
    /// Structural-dispatch note: cached exact-rational radius and center facts
    /// can later select specialized circle-circle reducers here, while this
    /// API remains the semantic boundary seen by curve topology.
    pub fn circle_relation(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<CircleCircleRelation> {
        let center_delta = other.center().delta_from(self.center());
        let center_distance_squared = dot(
            &center_delta.0,
            &center_delta.1,
            &center_delta.0,
            &center_delta.1,
        );

        match is_zero(&center_distance_squared, policy) {
            Some(true) => {
                let radius_delta = self.radius_squared() - other.radius_squared();
                match is_zero(&radius_delta, policy) {
                    Some(true) => Ok(CircleCircleRelation::Coincident),
                    Some(false) => Ok(CircleCircleRelation::Disjoint),
                    None => Ok(CircleCircleRelation::Uncertain {
                        reason: UncertaintyReason::RealSign,
                    }),
                }
            }
            Some(false) => circle_relation_for_distinct_centers(
                self,
                other,
                center_delta,
                center_distance_squared,
                policy,
            ),
            None => Ok(CircleCircleRelation::Uncertain {
                reason: UncertaintyReason::RealSign,
            }),
        }
    }

    /// Intersects this circular arc with another circular arc.
    ///
    /// Distinct centers use the usual radical-axis construction; coincident
    /// centers split into same-radius overlap handling and disjoint concentric
    /// circles. Keeping same-circle overlaps out of the ordinary point path is
    /// essential for the degenerate-boundary cases discussed by the degenerate-intersection clipping model.
    pub fn intersect_arc(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ArcArcIntersection> {
        let center_delta = other.center().delta_from(self.center());
        let center_distance_squared = dot(
            &center_delta.0,
            &center_delta.1,
            &center_delta.0,
            &center_delta.1,
        );

        match is_zero(&center_distance_squared, policy) {
            Some(true) => intersect_concentric_arcs(self, other, policy),
            Some(false) => intersect_distinct_circle_arcs(
                self,
                other,
                center_delta,
                center_distance_squared,
                policy,
            ),
            None => Ok(ArcArcIntersection::Uncertain {
                reason: UncertaintyReason::RealSign,
            }),
        }
    }
}

fn intersect_non_parallel(
    a: &LineSeg2,
    b: &LineSeg2,
    policy: &CurvePolicy,
    rx: &Real,
    ry: &Real,
    sx: &Real,
    sy: &Real,
    qmp: (Real, Real),
) -> CurveResult<LineLineIntersection> {
    if let Some(intersection) = non_parallel_endpoint_intersection(a, b, policy)? {
        return Ok(intersection);
    }

    let denominator = cross(rx, ry, sx, sy);
    if is_zero(&denominator, policy) != Some(false) {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::RealSign,
        });
    }
    let t_numerator = cross(&qmp.0, &qmp.1, sx, sy);
    let u_numerator = cross(&qmp.0, &qmp.1, rx, ry);
    let t = (t_numerator / &denominator)?;
    let u = (u_numerator / &denominator)?;

    let Some(t_in_range) = in_closed_unit_interval(&t, policy) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };
    let Some(u_in_range) = in_closed_unit_interval(&u, policy) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    if !(t_in_range && u_in_range) {
        return Ok(LineLineIntersection::None);
    }

    let t_endpoint = at_unit_interval_endpoint(&t, policy).ok_or_else(|| {
        crate::CurveError::Real("could not classify first line parameter endpoint".into())
    })?;
    let u_endpoint = at_unit_interval_endpoint(&u, policy).ok_or_else(|| {
        crate::CurveError::Real("could not classify second line parameter endpoint".into())
    })?;
    let kind = if t_endpoint || u_endpoint {
        IntersectionKind::Endpoint
    } else {
        IntersectionKind::Crossing
    };

    Ok(LineLineIntersection::Point {
        point: a.point_at(t.clone()),
        a_param: t,
        b_param: u,
        kind,
    })
}

fn non_parallel_endpoint_intersection(
    a: &LineSeg2,
    b: &LineSeg2,
    policy: &CurvePolicy,
) -> CurveResult<Option<LineLineIntersection>> {
    for (a_point, a_param) in [(a.start(), Real::zero()), (a.end(), Real::one())] {
        for (b_point, b_param) in [(b.start(), Real::zero()), (b.end(), Real::one())] {
            if a_point == b_point {
                return Ok(Some(LineLineIntersection::Point {
                    point: a_point.clone(),
                    a_param,
                    b_param,
                    kind: IntersectionKind::Endpoint,
                }));
            }
        }
    }
    for (point, a_param) in [(a.start(), Real::zero()), (a.end(), Real::one())] {
        if b.contains_point(point, policy) == Classification::Decided(true) {
            return Ok(Some(LineLineIntersection::Point {
                point: point.clone(),
                a_param,
                b_param: parameter_on_line(b, point, policy)?,
                kind: IntersectionKind::Endpoint,
            }));
        }
    }
    for (point, b_param) in [(b.start(), Real::zero()), (b.end(), Real::one())] {
        if a.contains_point(point, policy) == Classification::Decided(true) {
            return Ok(Some(LineLineIntersection::Point {
                point: point.clone(),
                a_param: parameter_on_line(a, point, policy)?,
                b_param,
                kind: IntersectionKind::Endpoint,
            }));
        }
    }
    Ok(None)
}

fn intersect_parallel(
    a: &LineSeg2,
    b: &LineSeg2,
    policy: &CurvePolicy,
    rx: &Real,
    ry: &Real,
    qmp: (Real, Real),
) -> CurveResult<LineLineIntersection> {
    let collinear_test = cross(&qmp.0, &qmp.1, rx, ry);
    match is_zero(&collinear_test, policy) {
        Some(false) => Ok(LineLineIntersection::None),
        Some(true) => intersect_collinear(a, b, policy),
        None => Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::RealSign,
        }),
    }
}

fn intersect_collinear(
    a: &LineSeg2,
    b: &LineSeg2,
    policy: &CurvePolicy,
) -> CurveResult<LineLineIntersection> {
    let t0 = parameter_on_line(a, b.start(), policy)?;
    let t1 = parameter_on_line(a, b.end(), policy)?;
    let Some((t_min, t_max)) = sort_pair(t0, t1, policy) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    let overlap_start = max_real(t_min, Real::zero(), policy);
    let overlap_end = min_real(t_max, Real::one(), policy);
    let (Some(overlap_start), Some(overlap_end)) = (overlap_start, overlap_end) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    match compare_reals(&overlap_start, &overlap_end, policy) {
        Some(Ordering::Greater) => Ok(LineLineIntersection::None),
        Some(Ordering::Equal) => {
            let point = a.point_at(overlap_start.clone());
            let b_param = parameter_on_line(b, &point, policy)?;
            Ok(LineLineIntersection::Point {
                point,
                a_param: overlap_start,
                b_param,
                kind: IntersectionKind::Endpoint,
            })
        }
        Some(Ordering::Less) => {
            let start_point = a.point_at(overlap_start.clone());
            let end_point = a.point_at(overlap_end.clone());
            let b_start = parameter_on_line(b, &start_point, policy)?;
            let b_end = parameter_on_line(b, &end_point, policy)?;
            let segment = LineSeg2::try_new(start_point, end_point)?;
            Ok(LineLineIntersection::Overlap {
                segment,
                a_range: ParamRange::new(overlap_start, overlap_end),
                b_range: ParamRange::new(b_start, b_end),
            })
        }
        None => Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        }),
    }
}

fn parameter_on_line(line: &LineSeg2, point: &Point2, policy: &CurvePolicy) -> CurveResult<Real> {
    let (dx, dy) = line.delta();
    let delta = point.delta_from(line.start());

    match is_zero(&dx, policy) {
        Some(false) => (delta.0 / dx).map_err(Into::into),
        Some(true) => (delta.1 / dy).map_err(Into::into),
        None => match is_zero(&dy, policy) {
            Some(false) => (delta.1 / dy).map_err(Into::into),
            _ => Err(crate::CurveError::Real(
                "could not choose nonzero line component".into(),
            )),
        },
    }
}

fn cross(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    (ax * by) - (ay * bx)
}

fn dot(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    (ax * bx) + (ay * by)
}

fn line_arc_two_candidates(
    line: &LineSeg2,
    arc: &CircularArc2,
    t0: Real,
    t1: Real,
    policy: &CurvePolicy,
) -> CurveResult<LineArcIntersection> {
    let ordered = match compare_reals(&t0, &t1, policy) {
        Some(Ordering::Greater) => (t1, t0),
        Some(Ordering::Less | Ordering::Equal) => (t0, t1),
        None => {
            return Ok(LineArcIntersection::Uncertain {
                reason: UncertaintyReason::Ordering,
            });
        }
    };

    let first = line_arc_hit_candidate(line, arc, ordered.0, IntersectionKind::Crossing, policy)?;
    let second = line_arc_hit_candidate(line, arc, ordered.1, IntersectionKind::Crossing, policy)?;

    match (first, second) {
        (LineArcCandidate::Hit(first), LineArcCandidate::Hit(second)) => {
            Ok(LineArcIntersection::TwoPoints { first, second })
        }
        (LineArcCandidate::Hit(hit), LineArcCandidate::Miss)
        | (LineArcCandidate::Miss, LineArcCandidate::Hit(hit)) => {
            Ok(LineArcIntersection::Point(hit))
        }
        (LineArcCandidate::Miss, LineArcCandidate::Miss) => Ok(LineArcIntersection::None),
        (LineArcCandidate::Uncertain(reason), _) | (_, LineArcCandidate::Uncertain(reason)) => {
            Ok(LineArcIntersection::Uncertain { reason })
        }
    }
}

fn intersect_concentric_arcs(
    a: &CircularArc2,
    b: &CircularArc2,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection> {
    let radius_delta = a.radius_squared() - b.radius_squared();
    match is_zero(&radius_delta, policy) {
        Some(false) => Ok(ArcArcIntersection::None),
        Some(true) => intersect_same_circle_arcs(a, b, policy),
        None => Ok(ArcArcIntersection::Uncertain {
            reason: UncertaintyReason::RealSign,
        }),
    }
}

#[derive(Clone, Debug)]
struct SameCircleArcCandidate {
    point: Point2,
    a_param: Real,
    b_param: Real,
}

fn intersect_same_circle_arcs(
    a: &CircularArc2,
    b: &CircularArc2,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection> {
    // Same-circle arc overlaps are degenerate intersections, not ordinary
    // circle-circle points. For the minor/semicircle arc model, the common
    // sweep is bounded by source arc
    // endpoints, so endpoint candidates plus an interior sweep test certify
    // whether the common set is empty, point-only, or one finite arc interval.
    let mut candidates = Vec::new();
    for point in [a.start(), a.end(), b.start(), b.end()] {
        if let Some(reason) = insert_same_circle_candidate(&mut candidates, a, b, point, policy)? {
            return Ok(ArcArcIntersection::Uncertain { reason });
        }
    }

    if candidates.is_empty() {
        return Ok(ArcArcIntersection::None);
    }

    if let Some(reason) = sort_same_circle_candidates(&mut candidates, policy) {
        return Ok(ArcArcIntersection::Uncertain { reason });
    }

    if let Some(overlap) = same_circle_overlap_interval(a, b, &candidates, policy)? {
        return Ok(overlap);
    }

    match candidates.len() {
        1 => Ok(ArcArcIntersection::Point(ArcArcIntersectionPoint {
            point: candidates[0].point.clone(),
            a_param: candidates[0].a_param.clone(),
            b_param: candidates[0].b_param.clone(),
            kind: IntersectionKind::Endpoint,
        })),
        2 => Ok(ArcArcIntersection::TwoPoints {
            first: ArcArcIntersectionPoint {
                point: candidates[0].point.clone(),
                a_param: candidates[0].a_param.clone(),
                b_param: candidates[0].b_param.clone(),
                kind: IntersectionKind::Endpoint,
            },
            second: ArcArcIntersectionPoint {
                point: candidates[1].point.clone(),
                a_param: candidates[1].a_param.clone(),
                b_param: candidates[1].b_param.clone(),
                kind: IntersectionKind::Endpoint,
            },
        }),
        _ => Ok(ArcArcIntersection::Uncertain {
            reason: UncertaintyReason::Unsupported,
        }),
    }
}

fn insert_same_circle_candidate(
    candidates: &mut Vec<SameCircleArcCandidate>,
    a: &CircularArc2,
    b: &CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Option<UncertaintyReason>> {
    match a.contains_sweep_point(point, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(None),
        Classification::Uncertain(reason) => return Ok(Some(reason)),
    }
    match b.contains_sweep_point(point, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(None),
        Classification::Uncertain(reason) => return Ok(Some(reason)),
    }

    for existing in candidates.iter() {
        match is_zero(&existing.point.distance_squared(point), policy) {
            Some(true) => return Ok(None),
            Some(false) => {}
            None => return Ok(Some(UncertaintyReason::RealSign)),
        }
    }

    let a_param = match a.sweep_fraction_for_incident_point(point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => return Ok(Some(reason)),
    };
    let b_param = match b.sweep_fraction_for_incident_point(point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => return Ok(Some(reason)),
    };
    candidates.push(SameCircleArcCandidate {
        point: point.clone(),
        a_param,
        b_param,
    });
    Ok(None)
}

fn sort_same_circle_candidates(
    candidates: &mut [SameCircleArcCandidate],
    policy: &CurvePolicy,
) -> Option<UncertaintyReason> {
    for index in 1..candidates.len() {
        let mut cursor = index;
        while cursor > 0 {
            match compare_reals(
                &candidates[cursor - 1].a_param,
                &candidates[cursor].a_param,
                policy,
            ) {
                Some(Ordering::Greater) => candidates.swap(cursor - 1, cursor),
                Some(Ordering::Less | Ordering::Equal) => break,
                None => return Some(UncertaintyReason::Ordering),
            }
            cursor -= 1;
        }
    }

    None
}

fn same_circle_overlap_interval(
    a: &CircularArc2,
    b: &CircularArc2,
    candidates: &[SameCircleArcCandidate],
    policy: &CurvePolicy,
) -> CurveResult<Option<ArcArcIntersection>> {
    let mut overlap = None;

    for adjacent in candidates.windows(2) {
        let start = &adjacent[0];
        let end = &adjacent[1];
        match compare_reals(&start.a_param, &end.a_param, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal) => continue,
            Some(Ordering::Greater) | None => {
                return Ok(Some(ArcArcIntersection::Uncertain {
                    reason: UncertaintyReason::Ordering,
                }));
            }
        }

        let segment = CircularArc2::try_from_center(
            start.point.clone(),
            end.point.clone(),
            a.center().clone(),
            a.is_clockwise(),
        )?;
        let representative = match segment.representative_point(policy)? {
            Classification::Decided(representative) => representative,
            Classification::Uncertain(reason) => {
                return Ok(Some(ArcArcIntersection::Uncertain { reason }));
            }
        };
        match b.contains_sweep_point(&representative, policy) {
            Classification::Decided(true) => {
                if overlap.is_some() {
                    return Ok(Some(ArcArcIntersection::Uncertain {
                        reason: UncertaintyReason::Unsupported,
                    }));
                }
                overlap = Some(ArcArcIntersection::Overlap {
                    segment,
                    a_range: ParamRange::new(start.a_param.clone(), end.a_param.clone()),
                    b_range: ParamRange::new(start.b_param.clone(), end.b_param.clone()),
                });
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                return Ok(Some(ArcArcIntersection::Uncertain { reason }));
            }
        }
    }

    Ok(overlap)
}

fn intersect_distinct_circle_arcs(
    a: &CircularArc2,
    b: &CircularArc2,
    center_delta: (Real, Real),
    center_distance_squared: Real,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection> {
    if matches!(policy.numeric_mode, NumericMode::EdgePreview)
        && let Some(result) = intersect_distinct_circle_arcs_edge_preview(a, b, policy)?
    {
        return Ok(result);
    }

    match circle_relation_for_distinct_centers(a, b, center_delta, center_distance_squared, policy)?
    {
        CircleCircleRelation::Disjoint => Ok(ArcArcIntersection::None),
        CircleCircleRelation::Tangent { point } => {
            match arc_arc_hit_candidate(a, b, point, IntersectionKind::Tangent, policy)? {
                ArcArcCandidate::Hit(hit) => Ok(ArcArcIntersection::Point(hit)),
                ArcArcCandidate::Miss => Ok(ArcArcIntersection::None),
                ArcArcCandidate::Uncertain(reason) => Ok(ArcArcIntersection::Uncertain { reason }),
            }
        }
        CircleCircleRelation::Secant {
            first_point,
            second_point,
        } => arc_arc_two_candidates(a, b, first_point, second_point, policy),
        CircleCircleRelation::Uncertain { reason } => Ok(ArcArcIntersection::Uncertain { reason }),
        CircleCircleRelation::Coincident => Ok(ArcArcIntersection::Uncertain {
            reason: UncertaintyReason::Unsupported,
        }),
    }
}

fn circle_relation_for_distinct_centers(
    a: &CircularArc2,
    b: &CircularArc2,
    center_delta: (Real, Real),
    center_distance_squared: Real,
    policy: &CurvePolicy,
) -> CurveResult<CircleCircleRelation> {
    let radius_a_squared = a.radius_squared();
    let radius_b_squared = b.radius_squared();
    // Radical-axis circle intersection: project from `a.center` toward
    // `b.center`, then step perpendicular by the solved height. This is the
    // standard closed-form circle-circle construction
    // primitive-intersection catalogue; exact sign classification of
    // `height_squared` decides disjoint/tangent/two-point topology.
    let along_numerator = (&radius_a_squared - &radius_b_squared) + &center_distance_squared;
    let along_denominator = Real::from(2_i8) * &center_distance_squared;
    let along = (along_numerator / &along_denominator)?;
    let base = Point2::new(
        a.center().x() + (&center_delta.0 * &along),
        a.center().y() + (&center_delta.1 * &along),
    );
    let height_squared = radius_a_squared - ((&along * &along) * &center_distance_squared);

    match crate::classify::real_sign(&height_squared, policy) {
        Some(RealSign::Negative) => Ok(CircleCircleRelation::Disjoint),
        Some(RealSign::Zero) => Ok(CircleCircleRelation::Tangent { point: base }),
        Some(RealSign::Positive) => {
            let offset_scale = (height_squared / &center_distance_squared)?.sqrt()?;
            let offset_x = &center_delta.1 * &offset_scale;
            let offset_y = &center_delta.0 * &offset_scale;
            let first = Point2::new(base.x() - &offset_x, base.y() + &offset_y);
            let second = Point2::new(base.x() + offset_x, base.y() - offset_y);
            Ok(CircleCircleRelation::Secant {
                first_point: first,
                second_point: second,
            })
        }
        None => Ok(CircleCircleRelation::Uncertain {
            reason: UncertaintyReason::RealSign,
        }),
    }
}

fn intersect_distinct_circle_arcs_edge_preview(
    a: &CircularArc2,
    b: &CircularArc2,
    policy: &CurvePolicy,
) -> CurveResult<Option<ArcArcIntersection>> {
    let Some([ax, ay, bx, by, radius_a_squared, radius_b_squared]) = preview_circle_data(a, b)
    else {
        return Ok(None);
    };

    if radius_a_squared < 0.0 || radius_b_squared < 0.0 {
        return Ok(None);
    }

    let radius_a = radius_a_squared.sqrt();
    let radius_b = radius_b_squared.sqrt();
    let dx = bx - ax;
    let dy = by - ay;
    let center_distance_squared = dx.mul_add(dx, dy * dy);
    let tolerance = preview_length_tolerance(policy, [ax, ay, bx, by, radius_a, radius_b]);
    if center_distance_squared <= tolerance * tolerance {
        return Ok(None);
    }

    let along = (radius_a_squared - radius_b_squared + center_distance_squared)
        / (2.0 * center_distance_squared);
    let base_x = ax + dx * along;
    let base_y = ay + dy * along;
    let height_squared = radius_a_squared - along * along * center_distance_squared;
    let squared_tolerance = tolerance * tolerance;

    if height_squared < -squared_tolerance {
        return Ok(Some(ArcArcIntersection::None));
    }

    if height_squared.abs() <= squared_tolerance {
        let base = Point2::new(Real::try_from(base_x)?, Real::try_from(base_y)?);
        return Ok(Some(
            match arc_arc_hit_candidate(a, b, base, IntersectionKind::Tangent, policy)? {
                ArcArcCandidate::Hit(hit) => ArcArcIntersection::Point(hit),
                ArcArcCandidate::Miss => ArcArcIntersection::None,
                ArcArcCandidate::Uncertain(reason) => ArcArcIntersection::Uncertain { reason },
            },
        ));
    }

    let offset_scale = (height_squared / center_distance_squared).sqrt();
    let offset_x = dy * offset_scale;
    let offset_y = dx * offset_scale;
    let first = Point2::new(
        Real::try_from(base_x - offset_x)?,
        Real::try_from(base_y + offset_y)?,
    );
    let second = Point2::new(
        Real::try_from(base_x + offset_x)?,
        Real::try_from(base_y - offset_y)?,
    );

    arc_arc_two_candidates(a, b, first, second, policy).map(Some)
}

fn preview_circle_data(a: &CircularArc2, b: &CircularArc2) -> Option<[f64; 6]> {
    let data = [
        a.center().x().to_f64_lossy()?,
        a.center().y().to_f64_lossy()?,
        b.center().x().to_f64_lossy()?,
        b.center().y().to_f64_lossy()?,
        a.radius_squared().to_f64_lossy()?,
        b.radius_squared().to_f64_lossy()?,
    ];

    data.iter().all(|value| value.is_finite()).then_some(data)
}

fn preview_length_tolerance(policy: &CurvePolicy, values: [f64; 6]) -> f64 {
    let scale = values.into_iter().map(f64::abs).fold(1.0_f64, f64::max);
    policy
        .tolerance
        .map(|tolerance| tolerance.absolute.max(tolerance.relative) * scale)
        .unwrap_or(1e-12 * scale)
}

fn arc_arc_two_candidates(
    a: &CircularArc2,
    b: &CircularArc2,
    first: Point2,
    second: Point2,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection> {
    let first = arc_arc_hit_candidate(a, b, first, IntersectionKind::Crossing, policy)?;
    let second = arc_arc_hit_candidate(a, b, second, IntersectionKind::Crossing, policy)?;

    match (first, second) {
        (ArcArcCandidate::Hit(first), ArcArcCandidate::Hit(second)) => {
            Ok(ArcArcIntersection::TwoPoints { first, second })
        }
        (ArcArcCandidate::Hit(hit), ArcArcCandidate::Miss)
        | (ArcArcCandidate::Miss, ArcArcCandidate::Hit(hit)) => Ok(ArcArcIntersection::Point(hit)),
        (ArcArcCandidate::Miss, ArcArcCandidate::Miss) => Ok(ArcArcIntersection::None),
        (ArcArcCandidate::Uncertain(reason), _) | (_, ArcArcCandidate::Uncertain(reason)) => {
            Ok(ArcArcIntersection::Uncertain { reason })
        }
    }
}

#[allow(clippy::large_enum_variant)]
enum ArcArcCandidate {
    Hit(ArcArcIntersectionPoint),
    Miss,
    Uncertain(UncertaintyReason),
}

fn arc_arc_hit_candidate(
    a: &CircularArc2,
    b: &CircularArc2,
    point: Point2,
    base_kind: IntersectionKind,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcCandidate> {
    match a.contains_sweep_point(&point, policy) {
        Classification::Decided(false) => return Ok(ArcArcCandidate::Miss),
        Classification::Decided(true) => {}
        Classification::Uncertain(reason) => return Ok(ArcArcCandidate::Uncertain(reason)),
    }
    match b.contains_sweep_point(&point, policy) {
        Classification::Decided(false) => return Ok(ArcArcCandidate::Miss),
        Classification::Decided(true) => {}
        Classification::Uncertain(reason) => return Ok(ArcArcCandidate::Uncertain(reason)),
    }

    let Some(a_endpoint) = point_on_arc_endpoint(a, &point, policy) else {
        return Ok(ArcArcCandidate::Uncertain(UncertaintyReason::RealSign));
    };
    let Some(b_endpoint) = point_on_arc_endpoint(b, &point, policy) else {
        return Ok(ArcArcCandidate::Uncertain(UncertaintyReason::RealSign));
    };
    let kind = if a_endpoint || b_endpoint {
        IntersectionKind::Endpoint
    } else {
        base_kind
    };

    let a_param = match a.sweep_fraction_for_incident_point(&point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => return Ok(ArcArcCandidate::Uncertain(reason)),
    };
    let b_param = match b.sweep_fraction_for_incident_point(&point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => return Ok(ArcArcCandidate::Uncertain(reason)),
    };
    Ok(ArcArcCandidate::Hit(ArcArcIntersectionPoint {
        a_param,
        b_param,
        point,
        kind,
    }))
}

#[allow(clippy::large_enum_variant)]
enum LineArcCandidate {
    Hit(LineArcIntersectionPoint),
    Miss,
    Uncertain(UncertaintyReason),
}

fn line_arc_hit_candidate(
    line: &LineSeg2,
    arc: &CircularArc2,
    line_param: Real,
    base_kind: IntersectionKind,
    policy: &CurvePolicy,
) -> CurveResult<LineArcCandidate> {
    let in_line_range = in_closed_unit_interval(&line_param, policy);
    if in_line_range == Some(false) {
        return Ok(LineArcCandidate::Miss);
    }

    let point = line_point_at_for_policy(line, &line_param, policy)?;
    match arc.contains_sweep_point(&point, policy) {
        Classification::Decided(false) => return Ok(LineArcCandidate::Miss),
        Classification::Decided(true) => {}
        Classification::Uncertain(reason) => {
            return Ok(LineArcCandidate::Uncertain(reason));
        }
    }
    if in_line_range.is_none() {
        return Ok(LineArcCandidate::Uncertain(UncertaintyReason::Ordering));
    }

    let Some(line_endpoint) = at_unit_interval_endpoint(&line_param, policy) else {
        return Ok(LineArcCandidate::Uncertain(UncertaintyReason::Ordering));
    };
    let Some(arc_endpoint) = point_on_arc_endpoint(arc, &point, policy) else {
        return Ok(LineArcCandidate::Uncertain(UncertaintyReason::RealSign));
    };
    let kind = if line_endpoint || arc_endpoint {
        IntersectionKind::Endpoint
    } else {
        base_kind
    };

    let arc_param = match arc.sweep_fraction_for_incident_point(&point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => return Ok(LineArcCandidate::Uncertain(reason)),
    };
    Ok(LineArcCandidate::Hit(LineArcIntersectionPoint {
        arc_param,
        point,
        line_param,
        kind,
    }))
}

fn line_point_at_for_policy(
    line: &LineSeg2,
    line_param: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Point2> {
    if matches!(policy.numeric_mode, NumericMode::EdgePreview)
        && let (Some(t), Some(start_x), Some(start_y), Some(end_x), Some(end_y)) = (
            line_param.to_f64_lossy(),
            line.start().x().to_f64_lossy(),
            line.start().y().to_f64_lossy(),
            line.end().x().to_f64_lossy(),
            line.end().y().to_f64_lossy(),
        )
    {
        // Edge-preview mode is intentionally approximate. Re-lifting the
        // interpolated point keeps line/arc sweep tests from depending on
        // unsimplified radical expressions in the preview expression. This is
        // a display-side finite-output choice, the category finite-output segment intersection isolates from
        // exact segment-intersection predicates in "Practical Segment
        // Intersection with Finite Precision Output".
        let x = start_x.mul_add(1.0 - t, end_x * t);
        let y = start_y.mul_add(1.0 - t, end_y * t);
        if x.is_finite() && y.is_finite() {
            return Ok(Point2::new(
                Real::try_from(x)
                    .map_err(|_| CurveError::Real("could not lift preview x".into()))?,
                Real::try_from(y)
                    .map_err(|_| CurveError::Real("could not lift preview y".into()))?,
            ));
        }
    }

    Ok(line.point_at(line_param.clone()))
}

fn point_on_arc_endpoint(arc: &CircularArc2, point: &Point2, policy: &CurvePolicy) -> Option<bool> {
    let start = point.distance_squared(arc.start());
    if is_zero(&start, policy)? {
        return Some(true);
    }
    let end = point.distance_squared(arc.end());
    is_zero(&end, policy)
}
