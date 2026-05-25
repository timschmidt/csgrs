//! Retained exact arrangement cleanup for line path sets.
//!
//! This module does not build a planar subdivision or perform a boolean
//! operation. It records the exact event schedule that later CAM/EDA cleanup
//! stages can consume: proper crossings, endpoint touches, positive-length
//! collinear overlaps, and the exact split fragments induced on every retained
//! input segment.

use std::cmp::Ordering;

use hyperlimit::{
    Point2, PointSegmentLocation, PredicatePolicy, SegmentIntersection,
    classify_point_segment_with_policy, classify_segment_intersection_with_policy_and_facts,
    compare_reals_with_policy, point2_equal_with_policy,
    proper_segment_intersection_point_with_policy,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::arc::{
    ExplicitArcArrangementClass, ExplicitArcArrangementReport, ExplicitArcIntersectionClass,
    ExplicitArcPointClassification, ExplicitCircularArc, LineExplicitArcIntersectionClass,
};
use crate::provenance::PathProvenance;
use crate::segment::LinePathSegment;

/// Topological event class for a pair of retained line segments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineArrangementEventClass {
    /// The segments are certified disjoint.
    Disjoint,
    /// The segments cross at one point interior to both closed segments.
    ProperCrossing,
    /// The common set is a single endpoint or one endpoint on the other segment.
    EndpointTouch,
    /// The common set is a positive-length collinear interval.
    CollinearOverlap,
    /// The retained closed segments have the same endpoint set.
    Identical,
    /// The predicate policy could not certify the relation.
    Unknown,
}

/// Errors that prevent line arrangement cleanup from producing trusted splits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineArrangementError {
    /// A retained line segment is degenerate and cannot induce a 1D parameter order.
    DegenerateSegment { segment: usize },
    /// A point used as a split witness did not lie on the referenced segment.
    SplitPointOffSegment { segment: usize },
    /// Exact parameter construction required a division that the scalar layer rejected.
    ParameterDivision,
    /// Exact comparison of retained split parameters was undecidable.
    UndecidableParameterOrder { segment: usize },
    /// The same geometric point could not be de-duplicated exactly.
    UndecidablePointEquality,
}

/// Errors that prevent explicit-arc arrangement cleanup from producing trusted splits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitArcArrangementError {
    /// Full-circle arcs need an explicit angular branch cut before split ordering.
    FullCircleArcUnsupported { arc: usize },
    /// A split witness was not certified on the referenced arc.
    SplitPointOffArc { arc: usize },
    /// Exact ordering along an arc sweep was undecidable.
    UndecidableArcOrder { arc: usize },
    /// The same geometric point could not be de-duplicated exactly.
    UndecidablePointEquality,
    /// A retained explicit sub-arc could not be reconstructed from certified endpoints.
    FragmentConstruction,
}

/// Exact event class for one retained line segment against one explicit arc.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineArcArrangementEventClass {
    /// The retained segment and arc are certified disjoint.
    Disjoint,
    /// The line segment touches the arc at one certified point.
    Tangent,
    /// The line segment crosses the arc at two certified points.
    Secant,
    /// The current exact predicates cannot decide the relation.
    Unknown,
}

/// Exact facts cached for one arranged line path set.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArrangementFacts {
    /// Exact-set facts across all input endpoint coordinates.
    pub endpoint_exact: RealExactSetFacts,
    /// Exact-set facts across all emitted fragment endpoint coordinates.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the arrangement schedule.
    pub provenance: PathProvenance,
}

/// Exact parameter and point witness on one input segment.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArrangementBreakpoint {
    /// Input segment index.
    pub segment: usize,
    /// Exact point on the segment.
    pub point: Point2,
    /// Numerator of the retained parameter `dot(point-start, end-start) / |end-start|^2`.
    pub parameter_numerator: Real,
    /// Positive denominator of the retained parameter.
    pub parameter_denominator: Real,
}

/// Exact fragment produced by splitting one retained segment at all events.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArrangementFragment {
    /// Input segment index.
    pub source_segment: usize,
    /// Fragment start witness on the source segment.
    pub start: LineArrangementBreakpoint,
    /// Fragment end witness on the source segment.
    pub end: LineArrangementBreakpoint,
    /// Retained exact line fragment.
    pub segment: LinePathSegment,
}

/// Pairwise arrangement event between two retained line segments.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArrangementEvent {
    /// First input segment index.
    pub first: usize,
    /// Second input segment index.
    pub second: usize,
    /// Exact topological class.
    pub class: LineArrangementEventClass,
    /// Raw segment classifier value when available.
    pub segment_intersection: Option<SegmentIntersection>,
    /// Single-point witness for proper crossings and endpoint touches.
    pub point: Option<Point2>,
    /// Positive-length overlap witness for collinear overlaps and identical segments.
    pub overlap: Option<LinePathSegment>,
}

/// Pairwise arrangement event between one line segment and one explicit arc.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArcArrangementEvent {
    /// Line segment index.
    pub line: usize,
    /// Explicit arc index.
    pub arc: usize,
    /// Certified line/arc event class.
    pub class: LineArcArrangementEventClass,
    /// Raw axis-aligned line/arc classifier value when available.
    pub line_arc_intersection: Option<LineExplicitArcIntersectionClass>,
    /// Certified intersection points in line construction order.
    pub points: Vec<Point2>,
}

/// Exact breakpoint witness on one explicit arc.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcArrangementBreakpoint {
    /// Explicit arc index.
    pub arc: usize,
    /// Exact point on the arc sweep.
    pub point: Point2,
}

/// Exact explicit-arc fragment induced by retained arc/arc events.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcArrangementFragment {
    /// Input explicit arc index.
    pub source_arc: usize,
    /// Fragment start witness on the source arc.
    pub start: ExplicitArcArrangementBreakpoint,
    /// Fragment end witness on the source arc.
    pub end: ExplicitArcArrangementBreakpoint,
    /// Retained exact explicit sub-arc.
    pub arc: ExplicitCircularArc,
}

/// Pairwise retained event between two explicit circular arcs.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcSetArrangementEvent {
    /// First explicit arc index.
    pub first: usize,
    /// Second explicit arc index.
    pub second: usize,
    /// Certified arrangement class from the retained arc predicate.
    pub class: ExplicitArcArrangementClass,
    /// Raw retained pair report.
    pub report: ExplicitArcArrangementReport,
    /// Certified point witnesses that should split one or both arcs.
    pub points: Vec<Point2>,
}

/// Retained line arrangement schedule and split fragments.
///
/// The report is a Yap-style exact object package: input geometry is preserved,
/// pairwise events are certified by exact predicates, and split fragments are
/// emitted only after their exact segment parameters are ordered. The pairwise
/// classifier is the standard segment-intersection test from de Berg, Cheong,
/// van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed. (2008), as exposed by `hyperlimit`. The split ordering
/// uses exact rational parameter comparison, following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): topology is
/// accepted only after exact predicate replay, not after sampled tolerance
/// cleanup.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArrangementReport {
    /// Retained input segments.
    pub segments: Vec<LinePathSegment>,
    /// Certified or unknown pairwise events.
    pub events: Vec<LineArrangementEvent>,
    /// Sorted breakpoints for every source segment.
    pub breakpoints: Vec<Vec<LineArrangementBreakpoint>>,
    /// Positive-length split fragments. Point fragments are intentionally omitted.
    pub fragments: Vec<LineArrangementFragment>,
    /// Cached exact facts for the retained arrangement schedule.
    pub facts: LineArrangementFacts,
}

/// Retained mixed line/arc arrangement schedule for the axis-aligned line subset.
///
/// This report is intentionally narrower than a full circular-arc arrangement
/// graph. It schedules exact line/arc events and line split fragments so later
/// CAM/EDA stages can consume certified witnesses without flattening the arc or
/// constructing planar cells in `hyperpath`. The exact line/circle solve is the
/// retained axis-aligned branch described by CGAL-style circular-arc
/// arrangements, while acceptance follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): every event point is
/// replayed against exact segment bounds and exact arc-sweep predicates before
/// it can split a source line.
#[derive(Clone, Debug, PartialEq)]
pub struct LineArcArrangementReport {
    /// Retained input line segments.
    pub lines: Vec<LinePathSegment>,
    /// Retained input explicit arcs.
    pub arcs: Vec<ExplicitCircularArc>,
    /// Certified or unknown line/arc pair events.
    pub events: Vec<LineArcArrangementEvent>,
    /// Sorted line breakpoints induced by line endpoints and line/arc events.
    pub line_breakpoints: Vec<Vec<LineArrangementBreakpoint>>,
    /// Positive-length line fragments induced by exact line/arc split points.
    pub line_fragments: Vec<LineArrangementFragment>,
    /// Cached exact facts for retained line endpoints and emitted line fragments.
    pub facts: LineArrangementFacts,
}

/// Retained explicit-arc arrangement schedule and split fragments.
///
/// This is the arc-only companion to [`LineArrangementReport`]. It promotes
/// retained arc/arc predicate reports into exact breakpoints and sub-arcs
/// without constructing planar cells. Different-circle tangent/secant events
/// contribute exact point witnesses from the retained radical-axis/tangent
/// construction. Same-circle positive overlaps contribute the endpoints that
/// delimit the shared sweep. Breakpoint ordering is accepted only for non-full
/// explicit arcs and is certified by exact sub-arc containment predicates,
/// following Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997), and the CGAL circular-arc arrangement split between
/// exact curve objects and exact topology predicates.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcSetArrangementReport {
    /// Retained input explicit arcs.
    pub arcs: Vec<ExplicitCircularArc>,
    /// Certified or unknown pairwise arc events.
    pub events: Vec<ExplicitArcSetArrangementEvent>,
    /// Sorted breakpoints for every source arc.
    pub breakpoints: Vec<Vec<ExplicitArcArrangementBreakpoint>>,
    /// Positive-length explicit sub-arcs. Point fragments are intentionally omitted.
    pub fragments: Vec<ExplicitArcArrangementFragment>,
    /// Exact-set facts across all emitted fragment endpoints.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the arrangement schedule.
    pub provenance: PathProvenance,
}

/// Arrange a retained set of line segments into exact pair events and fragments.
///
/// Degenerate input segments are rejected before pair classification because a
/// zero-length carrier has no strict one-dimensional order for split fragments.
/// Proper crossings use the exact construction from `hyperlimit`; endpoint and
/// overlap events reuse retained endpoint witnesses, so this layer does not
/// invent new topology or perform region materialization.
pub fn arrange_line_segments(
    segments: &[LinePathSegment],
    policy: PredicatePolicy,
) -> Result<LineArrangementReport, LineArrangementError> {
    arrange_line_segments_with_provenance(segments, policy, PathProvenance::native())
}

/// Arrange a retained set of line segments with explicit source provenance.
pub fn arrange_line_segments_with_provenance(
    segments: &[LinePathSegment],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineArrangementReport, LineArrangementError> {
    for (index, segment) in segments.iter().enumerate() {
        if segment.facts().known_degenerate == Some(true) {
            return Err(LineArrangementError::DegenerateSegment { segment: index });
        }
        if matches!(
            compare_reals_with_policy(&segment.length_squared(), &Real::zero(), policy).value(),
            Some(Ordering::Equal)
        ) {
            return Err(LineArrangementError::DegenerateSegment { segment: index });
        }
    }

    let mut breakpoints = seed_endpoint_breakpoints(segments, policy)?;
    let mut events = Vec::new();

    for first in 0..segments.len() {
        for second in (first + 1)..segments.len() {
            let event =
                classify_line_arrangement_event(first, second, segments, &mut breakpoints, policy)?;
            events.push(event);
        }
    }

    sort_and_dedup_breakpoints(&mut breakpoints, policy)?;
    let fragments = build_fragments(&breakpoints, policy)?;
    let endpoint_refs = segments
        .iter()
        .flat_map(|segment| {
            [
                &segment.start().x,
                &segment.start().y,
                &segment.end().x,
                &segment.end().y,
            ]
        })
        .collect::<Vec<_>>();
    let fragment_refs = fragments
        .iter()
        .flat_map(|fragment| {
            [
                &fragment.segment.start().x,
                &fragment.segment.start().y,
                &fragment.segment.end().x,
                &fragment.segment.end().y,
            ]
        })
        .collect::<Vec<_>>();
    let facts = LineArrangementFacts {
        endpoint_exact: Real::exact_set_facts(endpoint_refs),
        fragment_exact: Real::exact_set_facts(fragment_refs),
        provenance,
    };
    Ok(LineArrangementReport {
        segments: segments.to_vec(),
        events,
        breakpoints,
        fragments,
        facts,
    })
}

/// Arrange retained line segments against retained explicit circular arcs.
///
/// Only line fragments are emitted because ordering points along arbitrary arc
/// sweeps is a later mixed-curve arrangement problem. The event list still
/// records exact arc witnesses, so a downstream curve-aware scheduler can
/// promote them without recomputing the line/circle predicates.
pub fn arrange_line_segments_with_explicit_arcs(
    lines: &[LinePathSegment],
    arcs: &[ExplicitCircularArc],
    policy: PredicatePolicy,
) -> Result<LineArcArrangementReport, LineArrangementError> {
    arrange_line_segments_with_explicit_arcs_and_provenance(
        lines,
        arcs,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange retained line segments against explicit arcs with source provenance.
pub fn arrange_line_segments_with_explicit_arcs_and_provenance(
    lines: &[LinePathSegment],
    arcs: &[ExplicitCircularArc],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineArcArrangementReport, LineArrangementError> {
    for (index, line) in lines.iter().enumerate() {
        if line.facts().known_degenerate == Some(true) {
            return Err(LineArrangementError::DegenerateSegment { segment: index });
        }
        if matches!(
            compare_reals_with_policy(&line.length_squared(), &Real::zero(), policy).value(),
            Some(Ordering::Equal)
        ) {
            return Err(LineArrangementError::DegenerateSegment { segment: index });
        }
    }

    let mut line_breakpoints = seed_endpoint_breakpoints(lines, policy)?;
    let mut events = Vec::new();
    for (line_index, line) in lines.iter().enumerate() {
        for (arc_index, arc) in arcs.iter().enumerate() {
            let event = classify_line_arc_arrangement_event(
                line_index,
                line,
                arc_index,
                arc,
                &mut line_breakpoints,
                policy,
            )?;
            events.push(event);
        }
    }

    sort_and_dedup_breakpoints(&mut line_breakpoints, policy)?;
    let line_fragments = build_fragments(&line_breakpoints, policy)?;
    let endpoint_refs = lines
        .iter()
        .flat_map(|segment| {
            [
                &segment.start().x,
                &segment.start().y,
                &segment.end().x,
                &segment.end().y,
            ]
        })
        .collect::<Vec<_>>();
    let fragment_refs = line_fragments
        .iter()
        .flat_map(|fragment| {
            [
                &fragment.segment.start().x,
                &fragment.segment.start().y,
                &fragment.segment.end().x,
                &fragment.segment.end().y,
            ]
        })
        .collect::<Vec<_>>();
    let facts = LineArrangementFacts {
        endpoint_exact: Real::exact_set_facts(endpoint_refs),
        fragment_exact: Real::exact_set_facts(fragment_refs),
        provenance,
    };
    Ok(LineArcArrangementReport {
        lines: lines.to_vec(),
        arcs: arcs.to_vec(),
        events,
        line_breakpoints,
        line_fragments,
        facts,
    })
}

/// Arrange retained explicit circular arcs into pair events and sub-arc fragments.
///
/// Full-circle arcs are rejected for now because a full circle has no retained
/// start/end order unless the caller supplies a branch cut. Non-full arcs keep
/// their authored direction and are split only at exact witnesses certified by
/// existing arc/arc predicates.
pub fn arrange_explicit_arcs(
    arcs: &[ExplicitCircularArc],
    policy: PredicatePolicy,
) -> Result<ExplicitArcSetArrangementReport, ExplicitArcArrangementError> {
    arrange_explicit_arcs_with_provenance(arcs, policy, PathProvenance::native())
}

/// Arrange retained explicit arcs with source provenance.
pub fn arrange_explicit_arcs_with_provenance(
    arcs: &[ExplicitCircularArc],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<ExplicitArcSetArrangementReport, ExplicitArcArrangementError> {
    for (index, arc) in arcs.iter().enumerate() {
        if arc.facts().known_full_circle {
            return Err(ExplicitArcArrangementError::FullCircleArcUnsupported { arc: index });
        }
    }

    let mut breakpoints = seed_arc_endpoint_breakpoints(arcs, policy)?;
    let mut events = Vec::new();
    for first in 0..arcs.len() {
        for second in (first + 1)..arcs.len() {
            let event = classify_explicit_arc_arrangement_event(
                first,
                second,
                arcs,
                &mut breakpoints,
                policy,
            )?;
            events.push(event);
        }
    }

    sort_and_dedup_arc_breakpoints(&mut breakpoints, arcs, policy)?;
    let fragments = build_arc_fragments(arcs, &breakpoints, policy)?;
    let fragment_refs = fragments
        .iter()
        .flat_map(|fragment| {
            [
                &fragment.arc.start().x,
                &fragment.arc.start().y,
                &fragment.arc.end().x,
                &fragment.arc.end().y,
            ]
        })
        .collect::<Vec<_>>();
    let fragment_exact = Real::exact_set_facts(fragment_refs);
    Ok(ExplicitArcSetArrangementReport {
        arcs: arcs.to_vec(),
        events,
        breakpoints,
        fragments,
        fragment_exact,
        provenance,
    })
}

fn classify_explicit_arc_arrangement_event(
    first: usize,
    second: usize,
    arcs: &[ExplicitCircularArc],
    breakpoints: &mut [Vec<ExplicitArcArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<ExplicitArcSetArrangementEvent, ExplicitArcArrangementError> {
    let report = arcs[first].arrange_with(&arcs[second], policy);
    let mut points = Vec::new();
    match report.class {
        ExplicitArcArrangementClass::DifferentCircleOnePoint
        | ExplicitArcArrangementClass::DifferentCircleTwoPoints => {
            if let Some(intersection) = &report.intersection {
                if matches!(
                    intersection.class,
                    ExplicitArcIntersectionClass::OnePoint
                        | ExplicitArcIntersectionClass::TwoPoints
                ) {
                    for point in &intersection.points {
                        push_unique_arc_point(&mut points, point.clone(), policy)?;
                        add_arc_breakpoint(
                            breakpoints,
                            first,
                            &arcs[first],
                            point.clone(),
                            policy,
                        )?;
                        add_arc_breakpoint(
                            breakpoints,
                            second,
                            &arcs[second],
                            point.clone(),
                            policy,
                        )?;
                    }
                }
            }
        }
        ExplicitArcArrangementClass::SameCircleEndpointTouch
        | ExplicitArcArrangementClass::SameCircleOverlap
        | ExplicitArcArrangementClass::SameCircleFirstCoversSecond
        | ExplicitArcArrangementClass::SameCircleSecondCoversFirst
        | ExplicitArcArrangementClass::SameCircleEqual => {
            let overlap_points =
                same_circle_overlap_breakpoints(&arcs[first], &arcs[second], policy)?;
            for point in overlap_points {
                push_unique_arc_point(&mut points, point.clone(), policy)?;
                if point_on_arc_bool_for_arrangement(&arcs[first], &point, policy)? {
                    add_arc_breakpoint(breakpoints, first, &arcs[first], point.clone(), policy)?;
                }
                if point_on_arc_bool_for_arrangement(&arcs[second], &point, policy)? {
                    add_arc_breakpoint(breakpoints, second, &arcs[second], point, policy)?;
                }
            }
        }
        ExplicitArcArrangementClass::SameCircleDisjoint
        | ExplicitArcArrangementClass::DifferentCircleDisjoint
        | ExplicitArcArrangementClass::DifferentCircleOutsideArcSweeps
        | ExplicitArcArrangementClass::Unknown => {}
    }
    Ok(ExplicitArcSetArrangementEvent {
        first,
        second,
        class: report.class,
        report,
        points,
    })
}

fn classify_line_arc_arrangement_event(
    line_index: usize,
    line: &LinePathSegment,
    arc_index: usize,
    arc: &ExplicitCircularArc,
    line_breakpoints: &mut [Vec<LineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<LineArcArrangementEvent, LineArrangementError> {
    let report = arc.intersect_axis_aligned_segment(line, policy);
    match report.class {
        LineExplicitArcIntersectionClass::Disjoint => Ok(LineArcArrangementEvent {
            line: line_index,
            arc: arc_index,
            class: LineArcArrangementEventClass::Disjoint,
            line_arc_intersection: Some(report.class),
            points: Vec::new(),
        }),
        LineExplicitArcIntersectionClass::Tangent | LineExplicitArcIntersectionClass::Secant => {
            for point in &report.points {
                add_breakpoint(line_breakpoints, line_index, line, point.clone(), policy)?;
            }
            Ok(LineArcArrangementEvent {
                line: line_index,
                arc: arc_index,
                class: match report.class {
                    LineExplicitArcIntersectionClass::Tangent => {
                        LineArcArrangementEventClass::Tangent
                    }
                    LineExplicitArcIntersectionClass::Secant => {
                        LineArcArrangementEventClass::Secant
                    }
                    LineExplicitArcIntersectionClass::Disjoint
                    | LineExplicitArcIntersectionClass::Unknown => unreachable!("matched above"),
                },
                line_arc_intersection: Some(report.class),
                points: report.points,
            })
        }
        LineExplicitArcIntersectionClass::Unknown => Ok(LineArcArrangementEvent {
            line: line_index,
            arc: arc_index,
            class: LineArcArrangementEventClass::Unknown,
            line_arc_intersection: Some(report.class),
            points: Vec::new(),
        }),
    }
}

fn classify_line_arrangement_event(
    first: usize,
    second: usize,
    segments: &[LinePathSegment],
    breakpoints: &mut [Vec<LineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<LineArrangementEvent, LineArrangementError> {
    let a = &segments[first];
    let b = &segments[second];
    let Some(intersection) = classify_segment_intersection_with_policy_and_facts(
        a.start(),
        a.end(),
        b.start(),
        b.end(),
        policy,
        a.facts().segment,
        b.facts().segment,
    )
    .value() else {
        return Ok(LineArrangementEvent {
            first,
            second,
            class: LineArrangementEventClass::Unknown,
            segment_intersection: None,
            point: None,
            overlap: None,
        });
    };

    match intersection {
        SegmentIntersection::Disjoint => Ok(LineArrangementEvent {
            first,
            second,
            class: LineArrangementEventClass::Disjoint,
            segment_intersection: Some(intersection),
            point: None,
            overlap: None,
        }),
        SegmentIntersection::Proper => {
            let Some(point) = proper_segment_intersection_point_with_policy(
                a.start(),
                a.end(),
                b.start(),
                b.end(),
                policy,
            )
            .value()
            .flatten() else {
                return Ok(LineArrangementEvent {
                    first,
                    second,
                    class: LineArrangementEventClass::Unknown,
                    segment_intersection: Some(intersection),
                    point: None,
                    overlap: None,
                });
            };
            add_breakpoint(breakpoints, first, a, point.clone(), policy)?;
            add_breakpoint(breakpoints, second, b, point.clone(), policy)?;
            Ok(LineArrangementEvent {
                first,
                second,
                class: LineArrangementEventClass::ProperCrossing,
                segment_intersection: Some(intersection),
                point: Some(point),
                overlap: None,
            })
        }
        SegmentIntersection::EndpointTouch => {
            let Some(point) = collect_shared_points(a, b, policy)?.into_iter().next() else {
                return Ok(LineArrangementEvent {
                    first,
                    second,
                    class: LineArrangementEventClass::Unknown,
                    segment_intersection: Some(intersection),
                    point: None,
                    overlap: None,
                });
            };
            add_breakpoint(breakpoints, first, a, point.clone(), policy)?;
            add_breakpoint(breakpoints, second, b, point.clone(), policy)?;
            Ok(LineArrangementEvent {
                first,
                second,
                class: LineArrangementEventClass::EndpointTouch,
                segment_intersection: Some(intersection),
                point: Some(point),
                overlap: None,
            })
        }
        SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
            let shared_points = collect_shared_points(a, b, policy)?;
            let Some((start, end)) = overlap_endpoints(shared_points, a, policy)? else {
                return Ok(LineArrangementEvent {
                    first,
                    second,
                    class: LineArrangementEventClass::Unknown,
                    segment_intersection: Some(intersection),
                    point: None,
                    overlap: None,
                });
            };
            add_breakpoint(breakpoints, first, a, start.clone(), policy)?;
            add_breakpoint(breakpoints, first, a, end.clone(), policy)?;
            add_breakpoint(breakpoints, second, b, start.clone(), policy)?;
            add_breakpoint(breakpoints, second, b, end.clone(), policy)?;
            Ok(LineArrangementEvent {
                first,
                second,
                class: if intersection == SegmentIntersection::Identical {
                    LineArrangementEventClass::Identical
                } else {
                    LineArrangementEventClass::CollinearOverlap
                },
                segment_intersection: Some(intersection),
                point: None,
                overlap: Some(LinePathSegment::new(start, end)),
            })
        }
    }
}

fn seed_arc_endpoint_breakpoints(
    arcs: &[ExplicitCircularArc],
    policy: PredicatePolicy,
) -> Result<Vec<Vec<ExplicitArcArrangementBreakpoint>>, ExplicitArcArrangementError> {
    arcs.iter()
        .enumerate()
        .map(|(index, arc)| {
            Ok(vec![
                make_arc_breakpoint(index, arc, arc.start().clone(), policy)?,
                make_arc_breakpoint(index, arc, arc.end().clone(), policy)?,
            ])
        })
        .collect()
}

fn add_arc_breakpoint(
    breakpoints: &mut [Vec<ExplicitArcArrangementBreakpoint>],
    arc_index: usize,
    arc: &ExplicitCircularArc,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), ExplicitArcArrangementError> {
    breakpoints[arc_index].push(make_arc_breakpoint(arc_index, arc, point, policy)?);
    Ok(())
}

fn make_arc_breakpoint(
    arc_index: usize,
    arc: &ExplicitCircularArc,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<ExplicitArcArrangementBreakpoint, ExplicitArcArrangementError> {
    if !point_on_arc_bool_for_arrangement(arc, &point, policy)? {
        return Err(ExplicitArcArrangementError::SplitPointOffArc { arc: arc_index });
    }
    Ok(ExplicitArcArrangementBreakpoint {
        arc: arc_index,
        point,
    })
}

fn same_circle_overlap_breakpoints(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    policy: PredicatePolicy,
) -> Result<Vec<Point2>, ExplicitArcArrangementError> {
    let mut points = Vec::new();
    for point in [first.start(), first.end()] {
        if point_on_arc_bool_for_arrangement(second, point, policy)? {
            push_unique_arc_point(&mut points, point.clone(), policy)?;
        }
    }
    for point in [second.start(), second.end()] {
        if point_on_arc_bool_for_arrangement(first, point, policy)? {
            push_unique_arc_point(&mut points, point.clone(), policy)?;
        }
    }
    Ok(points)
}

fn sort_and_dedup_arc_breakpoints(
    breakpoints: &mut [Vec<ExplicitArcArrangementBreakpoint>],
    arcs: &[ExplicitCircularArc],
    policy: PredicatePolicy,
) -> Result<(), ExplicitArcArrangementError> {
    for (arc_index, points) in breakpoints.iter_mut().enumerate() {
        let mut sorted = Vec::new();
        for point in std::mem::take(points) {
            insert_sorted_arc_breakpoint(&mut sorted, point, &arcs[arc_index], policy)?;
        }
        *points = sorted;
    }
    Ok(())
}

fn insert_sorted_arc_breakpoint(
    sorted: &mut Vec<ExplicitArcArrangementBreakpoint>,
    point: ExplicitArcArrangementBreakpoint,
    arc: &ExplicitCircularArc,
    policy: PredicatePolicy,
) -> Result<(), ExplicitArcArrangementError> {
    for index in 0..sorted.len() {
        match compare_arc_breakpoints(&point, &sorted[index], arc, policy)? {
            Ordering::Less => {
                sorted.insert(index, point);
                return Ok(());
            }
            Ordering::Equal => return Ok(()),
            Ordering::Greater => {}
        }
    }
    sorted.push(point);
    Ok(())
}

fn compare_arc_breakpoints(
    left: &ExplicitArcArrangementBreakpoint,
    right: &ExplicitArcArrangementBreakpoint,
    arc: &ExplicitCircularArc,
    policy: PredicatePolicy,
) -> Result<Ordering, ExplicitArcArrangementError> {
    if point2_equal_with_policy(&left.point, &right.point, policy).value() == Some(true) {
        return Ok(Ordering::Equal);
    }
    if point2_equal_with_policy(&left.point, arc.start(), policy).value() == Some(true)
        || point2_equal_with_policy(&right.point, arc.end(), policy).value() == Some(true)
    {
        return Ok(Ordering::Less);
    }
    if point2_equal_with_policy(&right.point, arc.start(), policy).value() == Some(true)
        || point2_equal_with_policy(&left.point, arc.end(), policy).value() == Some(true)
    {
        return Ok(Ordering::Greater);
    }
    let prefix = ExplicitCircularArc::with_provenance(
        arc.center().clone(),
        arc.radius().clone(),
        arc.start().clone(),
        right.point.clone(),
        arc.direction(),
        arc.provenance(),
    )
    .map_err(|_| ExplicitArcArrangementError::FragmentConstruction)?;
    match prefix.classify_point(&left.point, policy) {
        ExplicitArcPointClassification::OnArc => Ok(Ordering::Less),
        ExplicitArcPointClassification::OnCircleOutsideSweep => Ok(Ordering::Greater),
        ExplicitArcPointClassification::OffCircle => {
            Err(ExplicitArcArrangementError::SplitPointOffArc { arc: left.arc })
        }
        ExplicitArcPointClassification::Unknown => {
            Err(ExplicitArcArrangementError::UndecidableArcOrder { arc: left.arc })
        }
    }
}

fn build_arc_fragments(
    arcs: &[ExplicitCircularArc],
    breakpoints: &[Vec<ExplicitArcArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<ExplicitArcArrangementFragment>, ExplicitArcArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if point2_equal_with_policy(&window[0].point, &window[1].point, policy).value()
                == Some(true)
            {
                continue;
            }
            let source = &arcs[window[0].arc];
            let fragment = ExplicitCircularArc::with_provenance(
                source.center().clone(),
                source.radius().clone(),
                window[0].point.clone(),
                window[1].point.clone(),
                source.direction(),
                source.provenance(),
            )
            .map_err(|_| ExplicitArcArrangementError::FragmentConstruction)?;
            fragments.push(ExplicitArcArrangementFragment {
                source_arc: window[0].arc,
                start: window[0].clone(),
                end: window[1].clone(),
                arc: fragment,
            });
        }
    }
    Ok(fragments)
}

fn point_on_arc_bool_for_arrangement(
    arc: &ExplicitCircularArc,
    point: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, ExplicitArcArrangementError> {
    match arc.classify_point(point, policy) {
        ExplicitArcPointClassification::OnArc => Ok(true),
        ExplicitArcPointClassification::OnCircleOutsideSweep
        | ExplicitArcPointClassification::OffCircle => Ok(false),
        ExplicitArcPointClassification::Unknown => {
            Err(ExplicitArcArrangementError::UndecidableArcOrder { arc: 0 })
        }
    }
}

fn push_unique_arc_point(
    points: &mut Vec<Point2>,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), ExplicitArcArrangementError> {
    for existing in points.iter() {
        match point2_equal_with_policy(existing, &point, policy).value() {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => return Err(ExplicitArcArrangementError::UndecidablePointEquality),
        }
    }
    points.push(point);
    Ok(())
}

fn seed_endpoint_breakpoints(
    segments: &[LinePathSegment],
    policy: PredicatePolicy,
) -> Result<Vec<Vec<LineArrangementBreakpoint>>, LineArrangementError> {
    segments
        .iter()
        .enumerate()
        .map(|(index, segment)| {
            Ok(vec![
                make_breakpoint(index, segment, segment.start().clone(), policy)?,
                make_breakpoint(index, segment, segment.end().clone(), policy)?,
            ])
        })
        .collect()
}

fn add_breakpoint(
    breakpoints: &mut [Vec<LineArrangementBreakpoint>],
    segment_index: usize,
    segment: &LinePathSegment,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), LineArrangementError> {
    breakpoints[segment_index].push(make_breakpoint(segment_index, segment, point, policy)?);
    Ok(())
}

fn make_breakpoint(
    segment_index: usize,
    segment: &LinePathSegment,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<LineArrangementBreakpoint, LineArrangementError> {
    match classify_point_segment_with_policy(segment.start(), segment.end(), &point, policy).value()
    {
        Some(location) if location.is_on_segment() => {}
        Some(_) => {
            return Err(LineArrangementError::SplitPointOffSegment {
                segment: segment_index,
            });
        }
        None => {
            return Err(LineArrangementError::SplitPointOffSegment {
                segment: segment_index,
            });
        }
    }
    let direction = Point2::new(
        segment.end().x.clone() - segment.start().x.clone(),
        segment.end().y.clone() - segment.start().y.clone(),
    );
    let offset = Point2::new(
        point.x.clone() - segment.start().x.clone(),
        point.y.clone() - segment.start().y.clone(),
    );
    let denominator = squared_norm(&direction);
    if !matches!(
        compare_reals_with_policy(&denominator, &Real::zero(), policy).value(),
        Some(Ordering::Greater)
    ) {
        return Err(LineArrangementError::DegenerateSegment {
            segment: segment_index,
        });
    }
    Ok(LineArrangementBreakpoint {
        segment: segment_index,
        point,
        parameter_numerator: dot(&offset, &direction),
        parameter_denominator: denominator,
    })
}

fn sort_and_dedup_breakpoints(
    breakpoints: &mut [Vec<LineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineArrangementError> {
    for (segment_index, points) in breakpoints.iter_mut().enumerate() {
        let mut sorted = Vec::new();
        for point in std::mem::take(points) {
            insert_sorted_breakpoint(&mut sorted, point, segment_index, policy)?;
        }
        *points = sorted;
    }
    Ok(())
}

fn insert_sorted_breakpoint(
    sorted: &mut Vec<LineArrangementBreakpoint>,
    point: LineArrangementBreakpoint,
    segment_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineArrangementError> {
    for index in 0..sorted.len() {
        match compare_breakpoints(&point, &sorted[index], policy)? {
            Ordering::Less => {
                sorted.insert(index, point);
                return Ok(());
            }
            Ordering::Equal => {
                if point2_equal_with_policy(&point.point, &sorted[index].point, policy).value()
                    != Some(true)
                {
                    return Err(LineArrangementError::UndecidablePointEquality);
                }
                return Ok(());
            }
            Ordering::Greater => {}
        }
    }
    if sorted
        .last()
        .and_then(|last| point2_equal_with_policy(&point.point, &last.point, policy).value())
        == Some(true)
    {
        return Ok(());
    }
    if sorted.last().is_some()
        && compare_breakpoints(sorted.last().expect("checked"), &point, policy).is_err()
    {
        return Err(LineArrangementError::UndecidableParameterOrder {
            segment: segment_index,
        });
    }
    sorted.push(point);
    Ok(())
}

fn compare_breakpoints(
    left: &LineArrangementBreakpoint,
    right: &LineArrangementBreakpoint,
    policy: PredicatePolicy,
) -> Result<Ordering, LineArrangementError> {
    let left_scaled = left.parameter_numerator.clone() * right.parameter_denominator.clone();
    let right_scaled = right.parameter_numerator.clone() * left.parameter_denominator.clone();
    compare_reals_with_policy(&left_scaled, &right_scaled, policy)
        .value()
        .ok_or(LineArrangementError::UndecidableParameterOrder {
            segment: left.segment,
        })
}

fn build_fragments(
    breakpoints: &[Vec<LineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<LineArrangementFragment>, LineArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_breakpoints(&window[0], &window[1], policy)? == Ordering::Equal {
                continue;
            }
            fragments.push(LineArrangementFragment {
                source_segment: window[0].segment,
                start: window[0].clone(),
                end: window[1].clone(),
                segment: LinePathSegment::new(window[0].point.clone(), window[1].point.clone()),
            });
        }
    }
    Ok(fragments)
}

fn collect_shared_points(
    first: &LinePathSegment,
    second: &LinePathSegment,
    policy: PredicatePolicy,
) -> Result<Vec<Point2>, LineArrangementError> {
    let mut shared = Vec::new();
    for point in [first.start(), first.end()] {
        if classify_point_segment_with_policy(second.start(), second.end(), point, policy)
            .value()
            .is_some_and(PointSegmentLocation::is_on_segment)
        {
            push_unique_point(&mut shared, point.clone(), policy)?;
        }
    }
    for point in [second.start(), second.end()] {
        if classify_point_segment_with_policy(first.start(), first.end(), point, policy)
            .value()
            .is_some_and(PointSegmentLocation::is_on_segment)
        {
            push_unique_point(&mut shared, point.clone(), policy)?;
        }
    }
    Ok(shared)
}

fn push_unique_point(
    points: &mut Vec<Point2>,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), LineArrangementError> {
    for existing in points.iter() {
        match point2_equal_with_policy(existing, &point, policy).value() {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => return Err(LineArrangementError::UndecidablePointEquality),
        }
    }
    points.push(point);
    Ok(())
}

fn overlap_endpoints(
    points: Vec<Point2>,
    reference: &LinePathSegment,
    policy: PredicatePolicy,
) -> Result<Option<(Point2, Point2)>, LineArrangementError> {
    if points.len() < 2 {
        return Ok(None);
    }
    let mut breakpoints = points
        .into_iter()
        .map(|point| make_breakpoint(0, reference, point, policy))
        .collect::<Result<Vec<_>, _>>()?;
    sort_and_dedup_breakpoints(std::slice::from_mut(&mut breakpoints), policy)?;
    if breakpoints.len() < 2 {
        return Ok(None);
    }
    Ok(Some((
        breakpoints.first().expect("len checked").point.clone(),
        breakpoints.last().expect("len checked").point.clone(),
    )))
}

fn squared_norm(vector: &Point2) -> Real {
    Real::signed_product_sum(
        [true, true],
        [[&vector.x, &vector.x], [&vector.y, &vector.y]],
    )
}

fn dot(first: &Point2, second: &Point2) -> Real {
    Real::signed_product_sum([true, true], [[&first.x, &second.x], [&first.y, &second.y]])
}
