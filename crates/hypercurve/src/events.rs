//! Normalized topology events for contour-level algorithms.
//!
//! Event collection is deliberately a candidate-filtered pair scan today, not
//! a sweep-line implementation. Bounding boxes only remove pairs whose
//! disjointness is decided; every remaining candidate goes through the exact
//! segment kernels. Bentley and Ottmann, "Algorithms for Reporting and Counting
//! Geometric Intersections" (*IEEE Transactions on Computers* C-28(9),
//! 643-647, 1979), is the reference point for replacing this flat scan with an
//! output-sensitive sweep once the crate needs larger arrangements.

use hyperreal::Real;

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_contour_aabb, decided_segment_aabb};
use crate::classify::{
    compare_reals, compare_reals_for_split_ordering, in_closed_unit_interval, is_zero, min_real,
};
use crate::{
    ArcArcIntersection, Classification, Contour2, CurveError, CurvePolicy, CurveResult,
    IntersectionKind, LineArcIntersection, LineArcOrder, LineLineIntersection, ParamRange, Point2,
    Segment2, SegmentIntersection, UncertaintyReason,
};

/// Which side of a contour-pair event to inspect.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourOperand {
    /// First contour passed to the intersection query.
    First,
    /// Second contour passed to the intersection query.
    Second,
}

/// A normalized set of contour-pair topology events.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContourIntersectionSet {
    events: Vec<ContourIntersection>,
}

impl ContourIntersectionSet {
    /// Constructs an event set from already-normalized events.
    pub fn new(events: Vec<ContourIntersection>) -> CurveResult<Self> {
        Self::new_with_policy(events, &CurvePolicy::certified())
    }

    fn new_with_policy(
        events: Vec<ContourIntersection>,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        validate_contour_intersection_events(&events, policy)?;
        Ok(Self { events })
    }

    /// Returns all events in segment-pair scan order.
    pub fn events(&self) -> &[ContourIntersection] {
        &self.events
    }

    /// Consumes the set and returns its events.
    pub fn into_events(self) -> Vec<ContourIntersection> {
        self.events
    }

    /// Returns true when no events were collected.
    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Returns the number of collected events.
    pub fn len(&self) -> usize {
        self.events.len()
    }

    /// Returns events for one segment sorted by that segment's local parameter.
    pub fn sorted_events_for_segment<'a>(
        &'a self,
        operand: ContourOperand,
        segment_index: usize,
        policy: &CurvePolicy,
    ) -> Classification<Vec<&'a ContourIntersection>> {
        let mut sorted: Vec<(&ContourIntersection, Real)> = Vec::new();

        for event in self.events.iter() {
            if event.segment_index(operand) != Some(segment_index) {
                continue;
            }

            let order_param = match event.order_param(operand, policy) {
                Ok(order_param) => order_param,
                Err(reason) => return Classification::Uncertain(reason),
            };

            let Some(insert_at) = insertion_index(&sorted, &order_param, policy) else {
                return Classification::Uncertain(UncertaintyReason::Ordering);
            };
            // Sorted local events are the contour-level analogue of the event
            // ordering used by sweep-line clipping algorithms. We keep the
            // order proof explicit because a wrong tie-breaker here can create
            // branch vertices in the downstream boundary graph.
            sorted.insert(insert_at, (event, order_param));
        }

        Classification::Decided(sorted.into_iter().map(|(event, _)| event).collect())
    }
}

fn validate_contour_intersection_events(
    events: &[ContourIntersection],
    policy: &CurvePolicy,
) -> CurveResult<()> {
    for (left_index, left) in events.iter().enumerate() {
        validate_contour_intersection_event(left, policy)?;
        if events[left_index + 1..].iter().any(|right| right == left) {
            return Err(CurveError::Topology(
                "contour intersection set must not contain duplicate events".into(),
            ));
        }
    }
    Ok(())
}

fn validate_contour_intersection_event(
    event: &ContourIntersection,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match event {
        ContourIntersection::Point(point) => {
            validate_event_unit_parameter(&point.a_param, policy, "first point")?;
            validate_event_unit_parameter(&point.b_param, policy, "second point")
        }
        ContourIntersection::Overlap(overlap) => {
            validate_event_unit_range(&overlap.a_range, policy, "first overlap")?;
            validate_event_unit_range(&overlap.b_range, policy, "second overlap")
        }
        ContourIntersection::Uncertain(_) => Ok(()),
    }
}

fn validate_event_unit_parameter(
    parameter: &Real,
    policy: &CurvePolicy,
    name: &str,
) -> CurveResult<()> {
    if in_closed_unit_interval(parameter, policy) != Some(true) {
        return Err(CurveError::Topology(format!(
            "contour intersection {name} parameter must be certified inside the unit interval"
        )));
    }
    Ok(())
}

fn validate_event_unit_range(
    range: &ParamRange,
    policy: &CurvePolicy,
    name: &str,
) -> CurveResult<()> {
    validate_event_unit_parameter(range.start(), policy, name)?;
    validate_event_unit_parameter(range.end(), policy, name)?;
    match compare_reals_for_split_ordering(range.start(), range.end(), policy) {
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => Ok(()),
        Some(std::cmp::Ordering::Equal) => Err(CurveError::Topology(format!(
            "contour intersection {name} range must be positive-dimensional"
        ))),
        None => Err(CurveError::Topology(format!(
            "contour intersection {name} range ordering must be certified"
        ))),
    }
}

/// One normalized contour-pair topology event.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum ContourIntersection {
    /// A single point event.
    Point(ContourPointIntersection),
    /// A finite overlap event.
    Overlap(ContourOverlapIntersection),
    /// Segment-pair classification could not be completed.
    Uncertain(ContourUncertainIntersection),
}

impl ContourIntersection {
    /// Returns the segment index on one side of the event.
    pub const fn segment_index(&self, operand: ContourOperand) -> Option<usize> {
        match self {
            Self::Point(event) => Some(match operand {
                ContourOperand::First => event.a_segment_index,
                ContourOperand::Second => event.b_segment_index,
            }),
            Self::Overlap(event) => Some(match operand {
                ContourOperand::First => event.a_segment_index,
                ContourOperand::Second => event.b_segment_index,
            }),
            Self::Uncertain(event) => Some(match operand {
                ContourOperand::First => event.a_segment_index,
                ContourOperand::Second => event.b_segment_index,
            }),
        }
    }

    fn order_param(
        &self,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Result<Real, UncertaintyReason> {
        match self {
            Self::Point(event) => Ok(match operand {
                ContourOperand::First => event.a_param.clone(),
                ContourOperand::Second => event.b_param.clone(),
            }),
            Self::Overlap(event) => {
                let range = match operand {
                    ContourOperand::First => &event.a_range,
                    ContourOperand::Second => &event.b_range,
                };
                min_real(range.start().clone(), range.end().clone(), policy)
                    .ok_or(UncertaintyReason::Ordering)
            }
            Self::Uncertain(event) => Err(event.reason),
        }
    }
}

/// A point event between two contours.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourPointIntersection {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Intersection point.
    pub point: Point2,
    /// Local parameter on the first contour segment.
    pub a_param: Real,
    /// Local parameter on the second contour segment.
    pub b_param: Real,
    /// Local contact kind.
    pub kind: IntersectionKind,
}

/// A finite overlap event between two contours.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourOverlapIntersection {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Overlap geometry.
    pub segment: Segment2,
    /// Parameter range on the first contour segment.
    pub a_range: ParamRange,
    /// Parameter range on the second contour segment.
    pub b_range: ParamRange,
}

/// An uncertain segment-pair relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContourUncertainIntersection {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Why classification stopped.
    pub reason: UncertaintyReason,
}

pub(crate) fn intersect_contours(
    a: &Contour2,
    b: &Contour2,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    // The bounding-box broad phase is only a candidate filter. Bentley and
    // Ottmann's sweep-line algorithm uses ordering structures to reduce
    // candidate pairs asymptotically; this crate currently keeps the simpler
    // pair scan but skips pairs whose boxes are decidably disjoint.
    let a_box = decided_contour_aabb(a, policy);
    let b_box = decided_contour_aabb(b, policy);
    let a_boxes: Vec<_> = a
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();
    let b_boxes: Vec<_> = b
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();

    intersect_contours_with_cached_aabbs(
        a,
        b,
        a_box.as_ref(),
        b_box.as_ref(),
        &a_boxes,
        &b_boxes,
        policy,
    )
}

pub(crate) fn intersect_contour_self(
    contour: &Contour2,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    let segment_boxes: Vec<_> = contour
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();

    intersect_contour_self_with_cached_aabbs(contour, &segment_boxes, policy)
}

pub(crate) fn intersect_contours_with_cached_aabbs(
    a: &Contour2,
    b: &Contour2,
    a_box: Option<&Aabb2>,
    b_box: Option<&Aabb2>,
    a_segment_boxes: &[Option<Aabb2>],
    b_segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    if let (Some(a_box), Some(b_box)) = (a_box, b_box)
        && aabbs_decided_disjoint(a_box, b_box, policy)
    {
        return ContourIntersectionSet::new_with_policy(Vec::new(), policy);
    }

    let mut events = Vec::new();

    for (a_segment_index, a_segment) in a.segments().iter().enumerate() {
        for (b_segment_index, b_segment) in b.segments().iter().enumerate() {
            if let (Some(Some(a_box)), Some(Some(b_box))) = (
                a_segment_boxes.get(a_segment_index),
                b_segment_boxes.get(b_segment_index),
            ) && aabbs_decided_disjoint(a_box, b_box, policy)
            {
                continue;
            }

            let relation = a_segment.intersect_segment(b_segment, policy)?;
            append_segment_relation_events(
                &mut events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                relation,
                policy,
            )?;
        }
    }

    ContourIntersectionSet::new_with_policy(events, policy)
}

pub(crate) fn intersect_contour_self_with_cached_aabbs(
    contour: &Contour2,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    let segments = contour.segments();
    let mut events = Vec::new();

    for first_index in 0..segments.len() {
        for second_index in (first_index + 1)..segments.len() {
            if let (Some(Some(first_box)), Some(Some(second_box))) = (
                segment_boxes.get(first_index),
                segment_boxes.get(second_index),
            ) && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                continue;
            }

            let relation =
                segments[first_index].intersect_segment(&segments[second_index], policy)?;
            let mut pair_events = Vec::new();
            append_segment_relation_events(
                &mut pair_events,
                first_index,
                second_index,
                &segments[first_index],
                &segments[second_index],
                relation,
                policy,
            )?;

            for event in pair_events {
                if is_contour_connectivity_event(
                    &event,
                    segments,
                    first_index,
                    second_index,
                    policy,
                ) {
                    continue;
                }
                events.push(event);
            }
        }
    }

    ContourIntersectionSet::new_with_policy(events, policy)
}

fn append_segment_relation_events(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    relation: SegmentIntersection,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match relation {
        SegmentIntersection::LineLine(LineLineIntersection::None) => {}
        SegmentIntersection::LineLine(LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            kind,
        }) => events.push(ContourIntersection::Point(ContourPointIntersection {
            a_segment_index,
            b_segment_index,
            point,
            a_param,
            b_param,
            kind,
        })),
        SegmentIntersection::LineLine(LineLineIntersection::Overlap {
            segment,
            a_range,
            b_range,
        }) => events.push(ContourIntersection::Overlap(ContourOverlapIntersection {
            a_segment_index,
            b_segment_index,
            segment: Segment2::Line(segment),
            a_range,
            b_range,
        })),
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason }) => {
            append_uncertain(events, a_segment_index, b_segment_index, reason);
        }
        SegmentIntersection::LineArc { order, result } => {
            append_line_arc_events(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                result,
                policy,
            )?;
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::None) => {}
        SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => {
            append_point_from_segments(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                hit.point,
                hit.kind,
                policy,
            )?;
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
            append_point_from_segments(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                first.point,
                first.kind,
                policy,
            )?;
            append_point_from_segments(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                second.point,
                second.kind,
                policy,
            )?;
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::Overlap {
            segment,
            a_range,
            b_range,
        }) => events.push(ContourIntersection::Overlap(ContourOverlapIntersection {
            a_segment_index,
            b_segment_index,
            segment: Segment2::Arc(segment),
            a_range,
            b_range,
        })),
        SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
            append_uncertain(events, a_segment_index, b_segment_index, reason);
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn append_line_arc_events(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    order: LineArcOrder,
    result: LineArcIntersection,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match result {
        LineArcIntersection::None => {}
        LineArcIntersection::Point(hit) => {
            append_line_arc_hit(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                hit,
                policy,
            )?;
        }
        LineArcIntersection::TwoPoints { first, second } => {
            append_line_arc_hit(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                first,
                policy,
            )?;
            append_line_arc_hit(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                second,
                policy,
            )?;
        }
        LineArcIntersection::Uncertain { reason } => {
            append_uncertain(events, a_segment_index, b_segment_index, reason);
        }
    }

    Ok(())
}

fn append_line_arc_hit(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    order: LineArcOrder,
    hit: crate::LineArcIntersectionPoint,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let point = hit.point;
    let (a_param, b_param) = match order {
        LineArcOrder::LineThenArc => {
            let arc_param = segment_chord_param(b_segment, &point)?;
            (hit.line_param, arc_param)
        }
        LineArcOrder::ArcThenLine => {
            let arc_param = segment_chord_param(a_segment, &point)?;
            (arc_param, hit.line_param)
        }
    };

    append_certified_point_event(
        events,
        a_segment_index,
        b_segment_index,
        point,
        a_param,
        b_param,
        hit.kind,
        policy,
    );

    Ok(())
}

fn append_point_from_segments(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    point: Point2,
    kind: IntersectionKind,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let a_param = segment_chord_param(a_segment, &point)?;
    let b_param = segment_chord_param(b_segment, &point)?;
    append_certified_point_event(
        events,
        a_segment_index,
        b_segment_index,
        point,
        a_param,
        b_param,
        kind,
        policy,
    );
    Ok(())
}

fn append_certified_point_event(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    point: Point2,
    a_param: Real,
    b_param: Real,
    kind: IntersectionKind,
    policy: &CurvePolicy,
) {
    if in_closed_unit_interval(&a_param, policy) == Some(true)
        && in_closed_unit_interval(&b_param, policy) == Some(true)
    {
        events.push(ContourIntersection::Point(ContourPointIntersection {
            a_segment_index,
            b_segment_index,
            point,
            a_param,
            b_param,
            kind,
        }));
    } else {
        append_uncertain(
            events,
            a_segment_index,
            b_segment_index,
            UncertaintyReason::Ordering,
        );
    }
}

fn append_uncertain(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    reason: UncertaintyReason,
) {
    events.push(ContourIntersection::Uncertain(
        ContourUncertainIntersection {
            a_segment_index,
            b_segment_index,
            reason,
        },
    ));
}

fn is_contour_connectivity_event(
    event: &ContourIntersection,
    segments: &[Segment2],
    first_index: usize,
    second_index: usize,
    policy: &CurvePolicy,
) -> bool {
    let Some(shared_point) = connected_contour_vertex(segments, first_index, second_index) else {
        return false;
    };

    match event {
        ContourIntersection::Point(point) => {
            points_match_for_connectivity(&point.point, shared_point, policy)
        }
        ContourIntersection::Overlap(_) | ContourIntersection::Uncertain(_) => false,
    }
}

fn connected_contour_vertex(
    segments: &[Segment2],
    first_index: usize,
    second_index: usize,
) -> Option<&Point2> {
    if first_index + 1 == second_index {
        return Some(segments[first_index].end());
    }

    if first_index == 0 && second_index + 1 == segments.len() {
        return Some(segments[first_index].start());
    }

    None
}

fn points_match_for_connectivity(point: &Point2, expected: &Point2, policy: &CurvePolicy) -> bool {
    let distance = point.distance_squared(expected);
    if is_zero(&distance, policy) == Some(true) {
        return true;
    }

    if matches!(policy.numeric_mode, crate::NumericMode::EdgePreview)
        && let (Some(distance), Some(tolerance)) = (distance.to_f64_lossy(), policy.tolerance)
    {
        let tolerance = tolerance.absolute.max(tolerance.relative);
        return distance.is_finite() && distance <= tolerance * tolerance;
    }

    false
}

fn segment_chord_param(segment: &Segment2, point: &Point2) -> CurveResult<Real> {
    let (dx, dy) = segment.end().delta_from(segment.start());
    let (px, py) = point.delta_from(segment.start());
    let numerator = (&px * &dx) + (&py * &dy);
    let denominator = (&dx * &dx) + (&dy * &dy);
    (numerator / denominator).map_err(Into::into)
}

fn insertion_index(
    sorted: &[(&ContourIntersection, Real)],
    order_param: &Real,
    policy: &CurvePolicy,
) -> Option<usize> {
    for (index, (_, existing_param)) in sorted.iter().enumerate() {
        match compare_reals(order_param, existing_param, policy)? {
            std::cmp::Ordering::Less => return Some(index),
            std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => {}
        }
    }
    Some(sorted.len())
}
