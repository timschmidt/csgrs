//! Normalized topology events for contour-level algorithms.

use hyperlattice::{Backend, DefaultBackend, Scalar};

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_contour_aabb, decided_segment_aabb};
use crate::classify::{compare_scalars, min_scalar};
use crate::{
    ArcArcIntersection, Classification, Contour2, CurvePolicy, CurveResult, IntersectionKind,
    LineArcIntersection, LineArcOrder, LineLineIntersection, ParamRange, Point2, Segment2,
    SegmentIntersection, UncertaintyReason,
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
pub struct ContourIntersectionSet<B: Backend = DefaultBackend> {
    events: Vec<ContourIntersection<B>>,
}

impl<B: Backend> ContourIntersectionSet<B> {
    /// Constructs an event set from already-normalized events.
    pub const fn new(events: Vec<ContourIntersection<B>>) -> Self {
        Self { events }
    }

    /// Returns all events in segment-pair scan order.
    pub fn events(&self) -> &[ContourIntersection<B>] {
        &self.events
    }

    /// Consumes the set and returns its events.
    pub fn into_events(self) -> Vec<ContourIntersection<B>> {
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
    ) -> Classification<Vec<&'a ContourIntersection<B>>> {
        let mut sorted: Vec<(&ContourIntersection<B>, Scalar<B>)> = Vec::new();

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
            sorted.insert(insert_at, (event, order_param));
        }

        Classification::Decided(sorted.into_iter().map(|(event, _)| event).collect())
    }
}

/// One normalized contour-pair topology event.
#[derive(Clone, Debug, PartialEq)]
pub enum ContourIntersection<B: Backend = DefaultBackend> {
    /// A single point event.
    Point(ContourPointIntersection<B>),
    /// A finite overlap event.
    Overlap(ContourOverlapIntersection<B>),
    /// Segment-pair classification could not be completed.
    Uncertain(ContourUncertainIntersection),
}

impl<B: Backend> ContourIntersection<B> {
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
    ) -> Result<Scalar<B>, UncertaintyReason> {
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
                min_scalar(range.start().clone(), range.end().clone(), policy)
                    .ok_or(UncertaintyReason::Ordering)
            }
            Self::Uncertain(event) => Err(event.reason),
        }
    }
}

/// A point event between two contours.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourPointIntersection<B: Backend = DefaultBackend> {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Intersection point.
    pub point: Point2<B>,
    /// Local parameter on the first contour segment.
    pub a_param: Scalar<B>,
    /// Local parameter on the second contour segment.
    pub b_param: Scalar<B>,
    /// Local contact kind.
    pub kind: IntersectionKind,
}

/// A finite overlap event between two contours.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourOverlapIntersection<B: Backend = DefaultBackend> {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Overlap geometry.
    pub segment: Segment2<B>,
    /// Parameter range on the first contour segment.
    pub a_range: ParamRange<B>,
    /// Parameter range on the second contour segment.
    pub b_range: ParamRange<B>,
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

pub(crate) fn intersect_contours<B: Backend>(
    a: &Contour2<B>,
    b: &Contour2<B>,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet<B>> {
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

pub(crate) fn intersect_contours_with_cached_aabbs<B: Backend>(
    a: &Contour2<B>,
    b: &Contour2<B>,
    a_box: Option<&Aabb2<B>>,
    b_box: Option<&Aabb2<B>>,
    a_segment_boxes: &[Option<Aabb2<B>>],
    b_segment_boxes: &[Option<Aabb2<B>>],
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet<B>> {
    if let (Some(a_box), Some(b_box)) = (a_box, b_box) {
        if aabbs_decided_disjoint(a_box, b_box, policy) {
            return Ok(ContourIntersectionSet::new(Vec::new()));
        }
    }

    let mut events = Vec::new();

    for (a_segment_index, a_segment) in a.segments().iter().enumerate() {
        for (b_segment_index, b_segment) in b.segments().iter().enumerate() {
            if let (Some(Some(a_box)), Some(Some(b_box))) = (
                a_segment_boxes.get(a_segment_index),
                b_segment_boxes.get(b_segment_index),
            ) {
                if aabbs_decided_disjoint(a_box, b_box, policy) {
                    continue;
                }
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

    Ok(ContourIntersectionSet::new(events))
}

fn append_segment_relation_events<B: Backend>(
    events: &mut Vec<ContourIntersection<B>>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2<B>,
    b_segment: &Segment2<B>,
    relation: SegmentIntersection<B>,
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
            )?;
            append_point_from_segments(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                second.point,
                second.kind,
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

fn append_line_arc_events<B: Backend>(
    events: &mut Vec<ContourIntersection<B>>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2<B>,
    b_segment: &Segment2<B>,
    order: LineArcOrder,
    result: LineArcIntersection<B>,
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
            )?;
            append_line_arc_hit(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                second,
            )?;
        }
        LineArcIntersection::Uncertain { reason } => {
            append_uncertain(events, a_segment_index, b_segment_index, reason);
        }
    }

    let _ = policy;
    Ok(())
}

fn append_line_arc_hit<B: Backend>(
    events: &mut Vec<ContourIntersection<B>>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2<B>,
    b_segment: &Segment2<B>,
    order: LineArcOrder,
    hit: crate::LineArcIntersectionPoint<B>,
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

    events.push(ContourIntersection::Point(ContourPointIntersection {
        a_segment_index,
        b_segment_index,
        point,
        a_param,
        b_param,
        kind: hit.kind,
    }));

    Ok(())
}

fn append_point_from_segments<B: Backend>(
    events: &mut Vec<ContourIntersection<B>>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2<B>,
    b_segment: &Segment2<B>,
    point: Point2<B>,
    kind: IntersectionKind,
) -> CurveResult<()> {
    let a_param = segment_chord_param(a_segment, &point)?;
    let b_param = segment_chord_param(b_segment, &point)?;
    events.push(ContourIntersection::Point(ContourPointIntersection {
        a_segment_index,
        b_segment_index,
        point,
        a_param,
        b_param,
        kind,
    }));
    Ok(())
}

fn append_uncertain<B: Backend>(
    events: &mut Vec<ContourIntersection<B>>,
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

fn segment_chord_param<B: Backend>(
    segment: &Segment2<B>,
    point: &Point2<B>,
) -> CurveResult<Scalar<B>> {
    let (dx, dy) = segment.end().delta_from(segment.start());
    let (px, py) = point.delta_from(segment.start());
    let numerator = (&px * &dx) + (&py * &dy);
    let denominator = (&dx * &dx) + (&dy * &dy);
    (numerator / denominator).map_err(Into::into)
}

fn insertion_index<B: Backend>(
    sorted: &[(&ContourIntersection<B>, Scalar<B>)],
    order_param: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<usize> {
    for (index, (_, existing_param)) in sorted.iter().enumerate() {
        match compare_scalars(order_param, existing_param, policy)? {
            std::cmp::Ordering::Less => return Some(index),
            std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => {}
        }
    }
    Some(sorted.len())
}
