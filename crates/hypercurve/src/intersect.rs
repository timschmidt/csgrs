//! Intersection result types and early line-line topology.

use std::cmp::Ordering;

use hyperlattice::{Backend, DefaultBackend, Scalar, ScalarSign};

use crate::classify::{
    at_unit_interval_endpoint, compare_scalars, in_closed_unit_interval, is_zero, max_scalar,
    min_scalar, sort_pair,
};
use crate::{
    CircularArc2, Classification, CurvePolicy, CurveResult, LineSeg2, Point2, Segment2,
    UncertaintyReason,
};

/// Parameter range on a segment.
#[derive(Clone, Debug, PartialEq)]
pub struct ParamRange<B: Backend = DefaultBackend> {
    start: Scalar<B>,
    end: Scalar<B>,
}

impl<B: Backend> ParamRange<B> {
    /// Constructs a parameter range.
    pub const fn new(start: Scalar<B>, end: Scalar<B>) -> Self {
        Self { start, end }
    }

    /// Range start.
    pub const fn start(&self) -> &Scalar<B> {
        &self.start
    }

    /// Range end.
    pub const fn end(&self) -> &Scalar<B> {
        &self.end
    }
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
pub enum LineLineIntersection<B: Backend = DefaultBackend> {
    /// No intersection.
    None,
    /// A single intersection point.
    Point {
        /// Intersection point.
        point: Point2<B>,
        /// Parameter on the first segment.
        a_param: Scalar<B>,
        /// Parameter on the second segment.
        b_param: Scalar<B>,
        /// Local kind of point contact.
        kind: IntersectionKind,
    },
    /// A collinear overlapping interval.
    Overlap {
        /// Overlapping segment geometry.
        segment: LineSeg2<B>,
        /// Parameter range on the first segment.
        a_range: ParamRange<B>,
        /// Parameter range on the second segment.
        b_range: ParamRange<B>,
    },
    /// The active policy could not classify this relation.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl<B: Backend> LineLineIntersection<B> {
    /// Returns true when this result has no intersection.
    pub const fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }
}

/// One point in a line-arc intersection result.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArcIntersectionPoint<B: Backend = DefaultBackend> {
    /// Intersection point.
    pub point: Point2<B>,
    /// Parameter on the line segment.
    pub line_param: Scalar<B>,
    /// Local kind of point contact.
    pub kind: IntersectionKind,
}

/// Intersection between a line segment and a circular arc.
#[derive(Clone, Debug, PartialEq)]
pub enum LineArcIntersection<B: Backend = DefaultBackend> {
    /// No intersection.
    None,
    /// A single intersection point.
    Point(LineArcIntersectionPoint<B>),
    /// Two intersection points, ordered by line parameter.
    TwoPoints {
        /// First hit along the line.
        first: LineArcIntersectionPoint<B>,
        /// Second hit along the line.
        second: LineArcIntersectionPoint<B>,
    },
    /// The active policy could not classify this relation.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl<B: Backend> LineArcIntersection<B> {
    /// Returns true when this result has no intersection.
    pub const fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }
}

/// One point in an arc-arc intersection result.
#[derive(Clone, Debug, PartialEq)]
pub struct ArcArcIntersectionPoint<B: Backend = DefaultBackend> {
    /// Intersection point.
    pub point: Point2<B>,
    /// Local kind of point contact.
    pub kind: IntersectionKind,
}

/// Intersection between two circular arcs.
#[derive(Clone, Debug, PartialEq)]
pub enum ArcArcIntersection<B: Backend = DefaultBackend> {
    /// No intersection.
    None,
    /// A single intersection point.
    Point(ArcArcIntersectionPoint<B>),
    /// Two intersection points.
    TwoPoints {
        /// First hit in deterministic construction order.
        first: ArcArcIntersectionPoint<B>,
        /// Second hit in deterministic construction order.
        second: ArcArcIntersectionPoint<B>,
    },
    /// A same-circle overlapping arc interval.
    Overlap {
        /// Overlapping arc geometry, oriented in the first arc's direction.
        segment: CircularArc2<B>,
        /// Parameter range on the first arc segment.
        a_range: ParamRange<B>,
        /// Parameter range on the second arc segment.
        b_range: ParamRange<B>,
    },
    /// The active policy could not classify this relation, or the relation is
    /// outside this slice.
    Uncertain {
        /// Why classification stopped.
        reason: UncertaintyReason,
    },
}

impl<B: Backend> ArcArcIntersection<B> {
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
pub enum SegmentIntersection<B: Backend = DefaultBackend> {
    /// Line-line relation.
    LineLine(LineLineIntersection<B>),
    /// Line-arc relation, with explicit operand order.
    LineArc {
        /// Whether the original operands were line-then-arc or arc-then-line.
        order: LineArcOrder,
        /// The line-arc intersection result. Point parameters are on the line.
        result: LineArcIntersection<B>,
    },
    /// Arc-arc relation.
    ArcArc(ArcArcIntersection<B>),
}

impl<B: Backend> SegmentIntersection<B> {
    /// Returns true when this result has no intersection.
    pub const fn is_none(&self) -> bool {
        match self {
            Self::LineLine(result) => result.is_none(),
            Self::LineArc { result, .. } => result.is_none(),
            Self::ArcArc(result) => result.is_none(),
        }
    }
}

impl<B: Backend> Segment2<B> {
    /// Intersects this segment with another native segment.
    pub fn intersect_segment(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<SegmentIntersection<B>> {
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

impl<B: Backend> LineSeg2<B> {
    /// Intersects this line segment with another line segment.
    pub fn intersect_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<LineLineIntersection<B>> {
        let (rx, ry) = self.delta();
        let (sx, sy) = other.delta();
        let qmp = other.start().delta_from(self.start());

        let denominator = cross(&rx, &ry, &sx, &sy);
        match is_zero(&denominator, policy) {
            Some(false) => intersect_non_parallel(self, policy, &rx, &ry, &sx, &sy, qmp),
            Some(true) => intersect_parallel(self, other, policy, &rx, &ry, qmp),
            None => Ok(LineLineIntersection::Uncertain {
                reason: UncertaintyReason::ScalarSign,
            }),
        }
    }

    /// Intersects this line segment with a circular arc.
    pub fn intersect_arc(
        &self,
        arc: &CircularArc2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<LineArcIntersection<B>> {
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

        match crate::classify::scalar_sign(&discriminant, policy) {
            Some(hyperlattice::ScalarSign::Negative) => Ok(LineArcIntersection::None),
            Some(hyperlattice::ScalarSign::Zero) => {
                let t = ((-half_b) / &a)?;
                match line_arc_hit_candidate(self, arc, t, IntersectionKind::Tangent, policy)? {
                    LineArcCandidate::Hit(hit) => Ok(LineArcIntersection::Point(hit)),
                    LineArcCandidate::Miss => Ok(LineArcIntersection::None),
                    LineArcCandidate::Uncertain(reason) => {
                        Ok(LineArcIntersection::Uncertain { reason })
                    }
                }
            }
            Some(hyperlattice::ScalarSign::Positive) => {
                let sqrt_discriminant = discriminant.clone().sqrt()?;
                let negative_half_b = -half_b;
                let t0 = ((&negative_half_b - &sqrt_discriminant) / &a)?;
                let t1 = ((negative_half_b.clone() + sqrt_discriminant) / &a)?;
                line_arc_two_candidates(
                    self,
                    arc,
                    t0,
                    t1,
                    QuadraticRootContext {
                        numerator: &negative_half_b,
                        discriminant: &discriminant,
                        denominator: &a,
                    },
                    policy,
                )
            }
            None => Ok(LineArcIntersection::Uncertain {
                reason: UncertaintyReason::ScalarSign,
            }),
        }
    }
}

impl<B: Backend> CircularArc2<B> {
    /// Intersects this circular arc with another circular arc.
    pub fn intersect_arc(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ArcArcIntersection<B>> {
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
                reason: UncertaintyReason::ScalarSign,
            }),
        }
    }
}

fn intersect_non_parallel<B: Backend>(
    a: &LineSeg2<B>,
    policy: &CurvePolicy,
    rx: &Scalar<B>,
    ry: &Scalar<B>,
    sx: &Scalar<B>,
    sy: &Scalar<B>,
    qmp: (Scalar<B>, Scalar<B>),
) -> CurveResult<LineLineIntersection<B>> {
    let denominator = cross(rx, ry, sx, sy);
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
        crate::CurveError::Scalar("could not classify first line parameter endpoint".into())
    })?;
    let u_endpoint = at_unit_interval_endpoint(&u, policy).ok_or_else(|| {
        crate::CurveError::Scalar("could not classify second line parameter endpoint".into())
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

fn intersect_parallel<B: Backend>(
    a: &LineSeg2<B>,
    b: &LineSeg2<B>,
    policy: &CurvePolicy,
    rx: &Scalar<B>,
    ry: &Scalar<B>,
    qmp: (Scalar<B>, Scalar<B>),
) -> CurveResult<LineLineIntersection<B>> {
    let collinear_test = cross(&qmp.0, &qmp.1, rx, ry);
    match is_zero(&collinear_test, policy) {
        Some(false) => Ok(LineLineIntersection::None),
        Some(true) => intersect_collinear(a, b, policy),
        None => Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::ScalarSign,
        }),
    }
}

fn intersect_collinear<B: Backend>(
    a: &LineSeg2<B>,
    b: &LineSeg2<B>,
    policy: &CurvePolicy,
) -> CurveResult<LineLineIntersection<B>> {
    let t0 = parameter_on_line(a, b.start(), policy)?;
    let t1 = parameter_on_line(a, b.end(), policy)?;
    let Some((t_min, t_max)) = sort_pair(t0, t1, policy) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    let overlap_start = max_scalar(t_min, Scalar::<B>::zero(), policy);
    let overlap_end = min_scalar(t_max, Scalar::<B>::one(), policy);
    let (Some(overlap_start), Some(overlap_end)) = (overlap_start, overlap_end) else {
        return Ok(LineLineIntersection::Uncertain {
            reason: UncertaintyReason::Ordering,
        });
    };

    match compare_scalars(&overlap_start, &overlap_end, policy) {
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

fn parameter_on_line<B: Backend>(
    line: &LineSeg2<B>,
    point: &Point2<B>,
    policy: &CurvePolicy,
) -> CurveResult<Scalar<B>> {
    let (dx, dy) = line.delta();
    let delta = point.delta_from(line.start());

    match is_zero(&dx, policy) {
        Some(false) => (delta.0 / dx).map_err(Into::into),
        Some(true) => (delta.1 / dy).map_err(Into::into),
        None => match is_zero(&dy, policy) {
            Some(false) => (delta.1 / dy).map_err(Into::into),
            _ => Err(crate::CurveError::Scalar(
                "could not choose nonzero line component".into(),
            )),
        },
    }
}

fn arc_chord_param<B: Backend>(arc: &CircularArc2<B>, point: &Point2<B>) -> CurveResult<Scalar<B>> {
    let (dx, dy) = arc.end().delta_from(arc.start());
    let (px, py) = point.delta_from(arc.start());
    let numerator = (&px * &dx) + (&py * &dy);
    let denominator = (&dx * &dx) + (&dy * &dy);
    (numerator / denominator).map_err(Into::into)
}

fn cross<B: Backend>(ax: &Scalar<B>, ay: &Scalar<B>, bx: &Scalar<B>, by: &Scalar<B>) -> Scalar<B> {
    (ax * by) - (ay * bx)
}

fn dot<B: Backend>(ax: &Scalar<B>, ay: &Scalar<B>, bx: &Scalar<B>, by: &Scalar<B>) -> Scalar<B> {
    (ax * bx) + (ay * by)
}

fn line_arc_two_candidates<B: Backend>(
    line: &LineSeg2<B>,
    arc: &CircularArc2<B>,
    t0: Scalar<B>,
    t1: Scalar<B>,
    root_context: QuadraticRootContext<'_, B>,
    policy: &CurvePolicy,
) -> CurveResult<LineArcIntersection<B>> {
    let ordered = match compare_scalars(&t0, &t1, policy) {
        Some(Ordering::Greater) => (t1, t0),
        Some(Ordering::Less | Ordering::Equal) => (t0, t1),
        None => {
            return Ok(LineArcIntersection::Uncertain {
                reason: UncertaintyReason::Ordering,
            });
        }
    };

    let first =
        if quadratic_root_in_closed_unit_interval(&root_context, QuadraticRoot::Lower, policy)
            == Some(false)
        {
            LineArcCandidate::Miss
        } else {
            line_arc_hit_candidate(line, arc, ordered.0, IntersectionKind::Crossing, policy)?
        };
    let second =
        if quadratic_root_in_closed_unit_interval(&root_context, QuadraticRoot::Upper, policy)
            == Some(false)
        {
            LineArcCandidate::Miss
        } else {
            line_arc_hit_candidate(line, arc, ordered.1, IntersectionKind::Crossing, policy)?
        };

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

struct QuadraticRootContext<'a, B: Backend> {
    numerator: &'a Scalar<B>,
    discriminant: &'a Scalar<B>,
    denominator: &'a Scalar<B>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum QuadraticRoot {
    Lower,
    Upper,
}

fn quadratic_root_in_closed_unit_interval<B: Backend>(
    context: &QuadraticRootContext<'_, B>,
    root: QuadraticRoot,
    policy: &CurvePolicy,
) -> Option<bool> {
    // Structural-dispatch note: line/arc intersections already know the root
    // shape `(n +/- sqrt(D)) / a`. Carrying that shape lets us compare the root
    // against interval endpoints by squaring exact terms, avoiding a lossy
    // `f64` parameter rejection and avoiding a harder sign query on an
    // expression containing `sqrt(D)`.
    let zero = Scalar::<B>::zero();
    let lower = quadratic_root_numerator_sign(
        context.numerator,
        context.discriminant,
        &zero,
        root,
        policy,
    )?;
    if lower == ScalarSign::Negative {
        return Some(false);
    }

    let upper = quadratic_root_numerator_sign(
        context.numerator,
        context.discriminant,
        context.denominator,
        root,
        policy,
    )?;
    Some(upper != ScalarSign::Positive)
}

fn quadratic_root_numerator_sign<B: Backend>(
    numerator: &Scalar<B>,
    discriminant: &Scalar<B>,
    shift: &Scalar<B>,
    root: QuadraticRoot,
    policy: &CurvePolicy,
) -> Option<ScalarSign> {
    let offset = numerator - shift;
    let offset_sign = crate::classify::scalar_sign(&offset, policy)?;
    match root {
        QuadraticRoot::Lower => match offset_sign {
            ScalarSign::Negative => Some(ScalarSign::Negative),
            ScalarSign::Zero => match crate::classify::scalar_sign(discriminant, policy)? {
                ScalarSign::Zero => Some(ScalarSign::Zero),
                ScalarSign::Positive => Some(ScalarSign::Negative),
                ScalarSign::Negative => None,
            },
            ScalarSign::Positive => {
                let squared_gap = (&offset * &offset) - discriminant;
                crate::classify::scalar_sign(&squared_gap, policy)
            }
        },
        QuadraticRoot::Upper => match offset_sign {
            ScalarSign::Positive => Some(ScalarSign::Positive),
            ScalarSign::Zero => match crate::classify::scalar_sign(discriminant, policy)? {
                ScalarSign::Zero => Some(ScalarSign::Zero),
                ScalarSign::Positive => Some(ScalarSign::Positive),
                ScalarSign::Negative => None,
            },
            ScalarSign::Negative => {
                let squared_gap = discriminant - (&offset * &offset);
                crate::classify::scalar_sign(&squared_gap, policy)
            }
        },
    }
}

fn intersect_concentric_arcs<B: Backend>(
    a: &CircularArc2<B>,
    b: &CircularArc2<B>,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection<B>> {
    let radius_delta = a.radius_squared() - b.radius_squared();
    match is_zero(&radius_delta, policy) {
        Some(false) => Ok(ArcArcIntersection::None),
        Some(true) => intersect_same_circle_arcs(a, b, policy),
        None => Ok(ArcArcIntersection::Uncertain {
            reason: UncertaintyReason::ScalarSign,
        }),
    }
}

#[derive(Clone, Debug)]
struct SameCircleArcCandidate<B: Backend> {
    point: Point2<B>,
    a_param: Scalar<B>,
    b_param: Scalar<B>,
}

fn intersect_same_circle_arcs<B: Backend>(
    a: &CircularArc2<B>,
    b: &CircularArc2<B>,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection<B>> {
    // Same-circle arc overlaps are degenerate intersections, not ordinary
    // circle-circle points. Foster, Hormann, and Popa separate coincident
    // boundary handling from entry/exit traversal (E. L. Foster, K. Hormann,
    // and R. T. Popa, "Clipping simple polygons with degenerate
    // intersections," Computers & Graphics: X 2, 100007, 2019). For the MVP
    // minor/semicircle arc model, the common sweep is bounded by source arc
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
            kind: IntersectionKind::Endpoint,
        })),
        2 => Ok(ArcArcIntersection::TwoPoints {
            first: ArcArcIntersectionPoint {
                point: candidates[0].point.clone(),
                kind: IntersectionKind::Endpoint,
            },
            second: ArcArcIntersectionPoint {
                point: candidates[1].point.clone(),
                kind: IntersectionKind::Endpoint,
            },
        }),
        _ => Ok(ArcArcIntersection::Uncertain {
            reason: UncertaintyReason::Unsupported,
        }),
    }
}

fn insert_same_circle_candidate<B: Backend>(
    candidates: &mut Vec<SameCircleArcCandidate<B>>,
    a: &CircularArc2<B>,
    b: &CircularArc2<B>,
    point: &Point2<B>,
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
            None => return Ok(Some(UncertaintyReason::ScalarSign)),
        }
    }

    candidates.push(SameCircleArcCandidate {
        point: point.clone(),
        a_param: arc_chord_param(a, point)?,
        b_param: arc_chord_param(b, point)?,
    });
    Ok(None)
}

fn sort_same_circle_candidates<B: Backend>(
    candidates: &mut [SameCircleArcCandidate<B>],
    policy: &CurvePolicy,
) -> Option<UncertaintyReason> {
    candidates.sort_by(|left, right| {
        compare_scalars(&left.a_param, &right.a_param, policy).unwrap_or(Ordering::Equal)
    });

    for adjacent in candidates.windows(2) {
        if compare_scalars(&adjacent[0].a_param, &adjacent[1].a_param, policy).is_none() {
            return Some(UncertaintyReason::Ordering);
        }
    }

    None
}

fn same_circle_overlap_interval<B: Backend>(
    a: &CircularArc2<B>,
    b: &CircularArc2<B>,
    candidates: &[SameCircleArcCandidate<B>],
    policy: &CurvePolicy,
) -> CurveResult<Option<ArcArcIntersection<B>>> {
    let mut overlap = None;

    for adjacent in candidates.windows(2) {
        let start = &adjacent[0];
        let end = &adjacent[1];
        match compare_scalars(&start.a_param, &end.a_param, policy) {
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

fn intersect_distinct_circle_arcs<B: Backend>(
    a: &CircularArc2<B>,
    b: &CircularArc2<B>,
    center_delta: (Scalar<B>, Scalar<B>),
    center_distance_squared: Scalar<B>,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection<B>> {
    let radius_a_squared = a.radius_squared();
    let radius_b_squared = b.radius_squared();
    let along_numerator = (&radius_a_squared - &radius_b_squared) + &center_distance_squared;
    let along_denominator = Scalar::<B>::from(2_i8) * &center_distance_squared;
    let along = (along_numerator / &along_denominator)?;
    let base = Point2::new(
        a.center().x() + (&center_delta.0 * &along),
        a.center().y() + (&center_delta.1 * &along),
    );
    let height_squared = radius_a_squared - ((&along * &along) * &center_distance_squared);

    match crate::classify::scalar_sign(&height_squared, policy) {
        Some(hyperlattice::ScalarSign::Negative) => Ok(ArcArcIntersection::None),
        Some(hyperlattice::ScalarSign::Zero) => {
            match arc_arc_hit_candidate(a, b, base, IntersectionKind::Tangent, policy)? {
                ArcArcCandidate::Hit(hit) => Ok(ArcArcIntersection::Point(hit)),
                ArcArcCandidate::Miss => Ok(ArcArcIntersection::None),
                ArcArcCandidate::Uncertain(reason) => Ok(ArcArcIntersection::Uncertain { reason }),
            }
        }
        Some(hyperlattice::ScalarSign::Positive) => {
            let offset_scale = (height_squared / &center_distance_squared)?.sqrt()?;
            let offset_x = &center_delta.1 * &offset_scale;
            let offset_y = &center_delta.0 * &offset_scale;
            let first = Point2::new(base.x() - &offset_x, base.y() + &offset_y);
            let second = Point2::new(base.x() + offset_x, base.y() - offset_y);
            arc_arc_two_candidates(a, b, first, second, policy)
        }
        None => Ok(ArcArcIntersection::Uncertain {
            reason: UncertaintyReason::ScalarSign,
        }),
    }
}

fn arc_arc_two_candidates<B: Backend>(
    a: &CircularArc2<B>,
    b: &CircularArc2<B>,
    first: Point2<B>,
    second: Point2<B>,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcIntersection<B>> {
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

enum ArcArcCandidate<B: Backend> {
    Hit(ArcArcIntersectionPoint<B>),
    Miss,
    Uncertain(UncertaintyReason),
}

fn arc_arc_hit_candidate<B: Backend>(
    a: &CircularArc2<B>,
    b: &CircularArc2<B>,
    point: Point2<B>,
    base_kind: IntersectionKind,
    policy: &CurvePolicy,
) -> CurveResult<ArcArcCandidate<B>> {
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
        return Ok(ArcArcCandidate::Uncertain(UncertaintyReason::ScalarSign));
    };
    let Some(b_endpoint) = point_on_arc_endpoint(b, &point, policy) else {
        return Ok(ArcArcCandidate::Uncertain(UncertaintyReason::ScalarSign));
    };
    let kind = if a_endpoint || b_endpoint {
        IntersectionKind::Endpoint
    } else {
        base_kind
    };

    Ok(ArcArcCandidate::Hit(ArcArcIntersectionPoint {
        point,
        kind,
    }))
}

enum LineArcCandidate<B: Backend> {
    Hit(LineArcIntersectionPoint<B>),
    Miss,
    Uncertain(UncertaintyReason),
}

fn line_arc_hit_candidate<B: Backend>(
    line: &LineSeg2<B>,
    arc: &CircularArc2<B>,
    line_param: Scalar<B>,
    base_kind: IntersectionKind,
    policy: &CurvePolicy,
) -> CurveResult<LineArcCandidate<B>> {
    let Some(in_line_range) = in_closed_unit_interval(&line_param, policy) else {
        return Ok(LineArcCandidate::Uncertain(UncertaintyReason::Ordering));
    };
    if !in_line_range {
        return Ok(LineArcCandidate::Miss);
    }

    let point = line.point_at(line_param.clone());
    match arc.contains_sweep_point(&point, policy) {
        Classification::Decided(false) => return Ok(LineArcCandidate::Miss),
        Classification::Decided(true) => {}
        Classification::Uncertain(reason) => {
            return Ok(LineArcCandidate::Uncertain(reason));
        }
    }

    let Some(line_endpoint) = at_unit_interval_endpoint(&line_param, policy) else {
        return Ok(LineArcCandidate::Uncertain(UncertaintyReason::Ordering));
    };
    let Some(arc_endpoint) = point_on_arc_endpoint(arc, &point, policy) else {
        return Ok(LineArcCandidate::Uncertain(UncertaintyReason::ScalarSign));
    };
    let kind = if line_endpoint || arc_endpoint {
        IntersectionKind::Endpoint
    } else {
        base_kind
    };

    Ok(LineArcCandidate::Hit(LineArcIntersectionPoint {
        point,
        line_param,
        kind,
    }))
}

fn point_on_arc_endpoint<B: Backend>(
    arc: &CircularArc2<B>,
    point: &Point2<B>,
    policy: &CurvePolicy,
) -> Option<bool> {
    let start = point.distance_squared(arc.start());
    if is_zero(&start, policy)? {
        return Some(true);
    }
    let end = point.distance_squared(arc.end());
    is_zero(&end, policy)
}
