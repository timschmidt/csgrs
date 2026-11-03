//! Exact convex polygon PCB pad predicates.
//!
//! Convex polygon pads cover chamfered, diamond, trapezoid, and other
//! non-rectangular footprint copper without making `hyperpath` a copper boolean
//! engine. The retained pad is a simple strictly convex polygon; clearance
//! predicates replay exact segment/edge and squared-distance tests.

use std::cmp::Ordering;

use hyperlimit::{
    Point2, PredicatePolicy, SegmentIntersection, classify_segment_intersection_with_facts,
    compare_reals,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::pcb::{
    BoardContourError, BoardContourOrientation, ClearanceStatus, PadBoardClearanceReport,
    PcbBoardOutline, PcbTrace, TraceClearanceReport,
};
use crate::pcb_shared::{
    classify_margins, classify_swept_distance, edge_cross, polygon_edges,
    polygon_signed_area_twice, segment_segment_distance_squared, unknown_pad_board_report,
    unknown_trace_report, update_minimum_distance, vertex_extrema,
};
use crate::segment::LinePathSegment;

/// Cached facts for an exact convex polygon PCB pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbConvexPadFacts {
    /// Exact-set facts across all retained vertex coordinates.
    pub exact: RealExactSetFacts,
    /// Certified winding orientation.
    pub orientation: BoardContourOrientation,
}

/// Exact strictly convex polygonal PCB pad.
///
/// This retained footprint carrier represents polygon copper directly as
/// authored vertices, not as a triangulated mesh or post-union shape. The
/// constructor accepts only strictly convex polygons with certified winding,
/// and the trace predicate certifies overlap or spacing by exact
/// segment/segment predicates plus squared distances. Candidate routes and
/// footprint imports become trusted only after
/// exact object validation and exact predicate replay. The point-in-polygon
/// half-plane test is the convex specialization of the crossing/winding tests
/// surveyed by Haines, "Point in Polygon Strategies," *Graphics Gems IV*
/// (1994), avoiding a tolerance ray cast for this pad family.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbConvexPad {
    net: crate::pcb::NetId,
    layer: crate::pcb::TraceLayer,
    vertices: Vec<Point2>,
    facts: PcbConvexPadFacts,
}

impl PcbConvexPad {
    /// Construct a convex polygon pad under an explicit predicate policy.
    pub fn new(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        vertices: Vec<Point2>,
        policy: PredicatePolicy,
    ) -> Result<Self, BoardContourError> {
        if vertices.len() < 3 {
            return Err(BoardContourError::TooFewVertices);
        }
        let signed_area_twice = polygon_signed_area_twice(&vertices);
        let orientation = match compare_reals(&signed_area_twice, &Real::zero(), policy).value() {
            Some(Ordering::Greater) => BoardContourOrientation::CounterClockwise,
            Some(Ordering::Less) => BoardContourOrientation::Clockwise,
            Some(Ordering::Equal) => return Err(BoardContourError::DegenerateArea),
            None => return Err(BoardContourError::UnknownOrientation),
        };
        validate_strict_convexity(&vertices, orientation, policy)?;
        let refs = vertices
            .iter()
            .flat_map(|point| [&point.x, &point.y])
            .collect::<Vec<_>>();
        let facts = PcbConvexPadFacts {
            exact: Real::exact_set_facts(refs),
            orientation,
        };
        Ok(Self {
            net,
            layer,
            vertices,
            facts,
        })
    }

    /// Return pad net.
    pub const fn net(&self) -> crate::pcb::NetId {
        self.net
    }

    /// Return pad layer.
    pub const fn layer(&self) -> crate::pcb::TraceLayer {
        self.layer
    }

    /// Return retained vertices in winding order.
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }

    /// Return certified winding orientation.
    pub const fn orientation(&self) -> BoardContourOrientation {
        self.facts.orientation
    }

    /// Return cached exact facts.
    pub const fn facts(&self) -> &PcbConvexPadFacts {
        &self.facts
    }
}

/// Check same-layer different-net clearance between a trace and a convex pad.
///
/// Endpoint containment or centerline/edge intersection certifies contact.
/// Otherwise the minimum exact squared distance between the trace segment and
/// every retained pad edge is compared against the swept trace width plus the
/// requested clearance. No pad tessellation, copper union, or floating
/// tolerance participates in this decision.
pub fn check_trace_convex_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbConvexPad,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if trace.layer() != pad.layer() || trace.net() == pad.net() {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let Some(distance_squared) =
        segment_convex_polygon_distance_squared(trace.swept().centerline(), pad, policy)
    else {
        return unknown_trace_report();
    };
    classify_swept_distance(
        distance_squared,
        trace.swept().width(),
        required_clearance,
        policy,
    )
}

/// Check exact clearance from a convex pad to an axis-aligned rectangular board.
///
/// The rectangular board-envelope rule uses the exact extrema of the retained
/// polygon vertices. This is a manufacturing clearance predicate, not a
/// clipping or boolean operation.
pub fn check_convex_pad_board_clearance(
    pad: &PcbConvexPad,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    let Some((min_x, max_x, min_y, max_y)) = vertex_extrema(pad.vertices(), policy) else {
        return unknown_pad_board_report();
    };
    let margins = [
        min_x - board.min().x.clone(),
        board.max().x.clone() - max_x,
        min_y - board.min().y.clone(),
        board.max().y.clone() - max_y,
    ];
    match classify_margins(&margins, required_clearance, policy) {
        Some((status, copper_gap)) => PadBoardClearanceReport {
            status,
            copper_gap: Some(copper_gap),
        },
        None => unknown_pad_board_report(),
    }
}

fn segment_convex_polygon_distance_squared(
    segment: &LinePathSegment,
    pad: &PcbConvexPad,
    policy: PredicatePolicy,
) -> Option<Real> {
    if point_inside_convex_polygon(segment.start(), pad.vertices(), pad.orientation(), policy)?
        || point_inside_convex_polygon(segment.end(), pad.vertices(), pad.orientation(), policy)?
    {
        return Some(Real::zero());
    }
    let edges = polygon_edges(pad.vertices(), policy)?;
    let mut minimum = None;
    for edge in &edges {
        let intersection = classify_segment_intersection_with_facts(
            segment.start(),
            segment.end(),
            edge.start(),
            edge.end(),
            segment.facts().segment,
            edge.facts().segment,
            policy,
        )
        .value()?;
        if !matches!(intersection, SegmentIntersection::Disjoint) {
            return Some(Real::zero());
        }
        update_minimum_distance(
            &mut minimum,
            segment_segment_distance_squared(segment, edge, policy)?,
            policy,
        )?;
    }
    minimum
}

fn point_inside_convex_polygon(
    point: &Point2,
    vertices: &[Point2],
    orientation: BoardContourOrientation,
    policy: PredicatePolicy,
) -> Option<bool> {
    for index in 0..vertices.len() {
        let edge_start = &vertices[index];
        let edge_end = &vertices[(index + 1) % vertices.len()];
        let side = oriented_edge_side(edge_start, edge_end, point, orientation);
        if compare_reals(&side, &Real::zero(), policy).value()? == Ordering::Less {
            return Some(false);
        }
    }
    Some(true)
}

fn validate_strict_convexity(
    vertices: &[Point2],
    orientation: BoardContourOrientation,
    policy: PredicatePolicy,
) -> Result<(), BoardContourError> {
    for index in 0..vertices.len() {
        let previous = &vertices[index];
        let current = &vertices[(index + 1) % vertices.len()];
        let next = &vertices[(index + 2) % vertices.len()];
        let cross_value = edge_cross(previous, current, next);
        let expected = match orientation {
            BoardContourOrientation::CounterClockwise => Ordering::Greater,
            BoardContourOrientation::Clockwise => Ordering::Less,
        };
        match compare_reals(&cross_value, &Real::zero(), policy).value() {
            Some(Ordering::Equal) => return Err(BoardContourError::CollinearEdge),
            Some(ordering) if ordering == expected => {}
            Some(_) => return Err(BoardContourError::NonConvex),
            None => return Err(BoardContourError::UnknownOrientation),
        }
    }
    Ok(())
}

fn oriented_edge_side(
    edge_start: &Point2,
    edge_end: &Point2,
    point: &Point2,
    orientation: BoardContourOrientation,
) -> Real {
    let cross_value = edge_cross(edge_start, edge_end, point);
    match orientation {
        BoardContourOrientation::CounterClockwise => cross_value,
        BoardContourOrientation::Clockwise => -cross_value,
    }
}
