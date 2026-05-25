//! Exact convex polygon PCB pad predicates.
//!
//! Convex polygon pads cover chamfered, diamond, trapezoid, and other
//! non-rectangular footprint copper without making `hyperpath` a copper boolean
//! engine. The retained pad is a simple strictly convex polygon; clearance
//! predicates replay exact segment/edge and squared-distance tests.

use std::cmp::Ordering;

use hyperlimit::{
    Point2, PredicatePolicy, SegmentIntersection, classify_segment_intersection_with_facts,
    compare_reals_with_policy,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::pcb::{
    BoardContourError, BoardContourOrientation, ClearanceStatus, PadBoardClearanceReport,
    PcbBoardOutline, PcbTrace, TraceClearanceReport,
};
use crate::provenance::PathProvenance;
use crate::segment::LinePathSegment;

/// Cached facts for an exact convex polygon PCB pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbConvexPadFacts {
    /// Exact-set facts across all retained vertex coordinates.
    pub exact: RealExactSetFacts,
    /// Certified winding orientation.
    pub orientation: BoardContourOrientation,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact strictly convex polygonal PCB pad.
///
/// This retained footprint carrier represents polygon copper directly as
/// authored vertices, not as a triangulated mesh or post-union shape. The
/// constructor accepts only strictly convex polygons with certified winding,
/// and the trace predicate certifies overlap or spacing by exact
/// segment/segment predicates plus squared distances. This follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): candidate routes and footprint imports become trusted only after
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
    /// Construct a convex polygon pad with native provenance.
    pub fn new(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        vertices: Vec<Point2>,
    ) -> Result<Self, BoardContourError> {
        Self::with_provenance(net, layer, vertices, PathProvenance::native())
    }

    /// Construct a convex polygon pad with source provenance.
    pub fn with_provenance(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        vertices: Vec<Point2>,
        provenance: PathProvenance,
    ) -> Result<Self, BoardContourError> {
        if vertices.len() < 3 {
            return Err(BoardContourError::TooFewVertices);
        }
        let signed_area_twice = polygon_signed_area_twice(&vertices);
        let orientation = match compare_reals_with_policy(
            &signed_area_twice,
            &Real::zero(),
            PredicatePolicy::default(),
        )
        .value()
        {
            Some(Ordering::Greater) => BoardContourOrientation::CounterClockwise,
            Some(Ordering::Less) => BoardContourOrientation::Clockwise,
            Some(Ordering::Equal) => return Err(BoardContourError::DegenerateArea),
            None => return Err(BoardContourError::UnknownOrientation),
        };
        validate_strict_convexity(&vertices, orientation)?;
        let refs = vertices
            .iter()
            .flat_map(|point| [&point.x, &point.y])
            .collect::<Vec<_>>();
        let facts = PcbConvexPadFacts {
            exact: Real::exact_set_facts(refs),
            orientation,
            provenance,
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

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
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

fn classify_swept_distance(
    distance_squared: Real,
    trace_width: &Real,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit_squared = trace_width.clone() * trace_width.clone();
    let clearance_limit = trace_width.clone() + required_clearance.clone() * Real::from(2);
    let clearance_limit_squared = clearance_limit.clone() * clearance_limit;
    let status =
        match compare_reals_with_policy(&four_distance_squared, &overlap_limit_squared, policy)
            .value()
        {
            Some(Ordering::Less | Ordering::Equal) => ClearanceStatus::NoShortViolation,
            Some(Ordering::Greater) => {
                match compare_reals_with_policy(
                    &four_distance_squared,
                    &clearance_limit_squared,
                    policy,
                )
                .value()
                {
                    Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
                    Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
                    None => ClearanceStatus::Unknown,
                }
            }
            None => ClearanceStatus::Unknown,
        };
    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
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
    let edges = polygon_edges(pad.vertices());
    let mut minimum = None;
    for edge in &edges {
        let intersection = classify_segment_intersection_with_facts(
            segment.start(),
            segment.end(),
            edge.start(),
            edge.end(),
            segment.facts().segment,
            edge.facts().segment,
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

fn segment_segment_distance_squared(
    first: &LinePathSegment,
    second: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    let mut minimum = None;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(first.start(), second, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(first.end(), second, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(second.start(), first, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(second.end(), first, policy)?,
        policy,
    )?;
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
        if compare_reals_with_policy(&side, &Real::zero(), policy).value()? == Ordering::Less {
            return Some(false);
        }
    }
    Some(true)
}

fn point_segment_distance_squared(
    point: &Point2,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    let ab = Point2::new(
        segment.end().x.clone() - segment.start().x.clone(),
        segment.end().y.clone() - segment.start().y.clone(),
    );
    let ap = Point2::new(
        point.x.clone() - segment.start().x.clone(),
        point.y.clone() - segment.start().y.clone(),
    );
    let bp = Point2::new(
        point.x.clone() - segment.end().x.clone(),
        point.y.clone() - segment.end().y.clone(),
    );
    let length_squared = squared_norm(&ab);
    match compare_reals_with_policy(&length_squared, &Real::zero(), policy).value()? {
        Ordering::Equal => return Some(squared_norm(&ap)),
        Ordering::Less => return None,
        Ordering::Greater => {}
    }
    let projection = dot(&ap, &ab);
    if !matches!(
        compare_reals_with_policy(&projection, &Real::zero(), policy).value()?,
        Ordering::Greater
    ) {
        return Some(squared_norm(&ap));
    }
    if !matches!(
        compare_reals_with_policy(&projection, &length_squared, policy).value()?,
        Ordering::Less
    ) {
        return Some(squared_norm(&bp));
    }
    let cross_value = cross(&ap, &ab);
    let cross_squared = cross_value.clone() * cross_value;
    (cross_squared / length_squared).ok()
}

fn validate_strict_convexity(
    vertices: &[Point2],
    orientation: BoardContourOrientation,
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
        match compare_reals_with_policy(&cross_value, &Real::zero(), PredicatePolicy::default())
            .value()
        {
            Some(Ordering::Equal) => return Err(BoardContourError::CollinearEdge),
            Some(ordering) if ordering == expected => {}
            Some(_) => return Err(BoardContourError::NonConvex),
            None => return Err(BoardContourError::UnknownOrientation),
        }
    }
    Ok(())
}

fn polygon_signed_area_twice(vertices: &[Point2]) -> Real {
    let mut area = Real::zero();
    for index in 0..vertices.len() {
        let current = &vertices[index];
        let next = &vertices[(index + 1) % vertices.len()];
        area = area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone();
    }
    area
}

fn polygon_edges(vertices: &[Point2]) -> Vec<LinePathSegment> {
    (0..vertices.len())
        .map(|index| {
            LinePathSegment::new(
                vertices[index].clone(),
                vertices[(index + 1) % vertices.len()].clone(),
            )
        })
        .collect()
}

fn vertex_extrema(
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Option<(Real, Real, Real, Real)> {
    let mut min_x = vertices[0].x.clone();
    let mut max_x = vertices[0].x.clone();
    let mut min_y = vertices[0].y.clone();
    let mut max_y = vertices[0].y.clone();
    for vertex in vertices.iter().skip(1) {
        if compare_reals_with_policy(&vertex.x, &min_x, policy).value()? == Ordering::Less {
            min_x = vertex.x.clone();
        }
        if compare_reals_with_policy(&vertex.x, &max_x, policy).value()? == Ordering::Greater {
            max_x = vertex.x.clone();
        }
        if compare_reals_with_policy(&vertex.y, &min_y, policy).value()? == Ordering::Less {
            min_y = vertex.y.clone();
        }
        if compare_reals_with_policy(&vertex.y, &max_y, policy).value()? == Ordering::Greater {
            max_y = vertex.y.clone();
        }
    }
    Some((min_x, max_x, min_y, max_y))
}

fn classify_margins(
    margins: &[Real; 4],
    required: &Real,
    policy: PredicatePolicy,
) -> Option<(ClearanceStatus, Real)> {
    let mut minimum_margin = margins[0].clone();
    for margin in margins {
        if matches!(
            compare_reals_with_policy(margin, &minimum_margin, policy).value(),
            Some(Ordering::Less)
        ) {
            minimum_margin = margin.clone();
        }
        match compare_reals_with_policy(margin, required, policy).value()? {
            Ordering::Less => return Some((ClearanceStatus::ClearanceViolation, margin.clone())),
            Ordering::Equal | Ordering::Greater => {}
        }
    }
    Some((ClearanceStatus::CertifiedClear, minimum_margin))
}

fn update_minimum_distance(
    minimum: &mut Option<Real>,
    candidate: Real,
    policy: PredicatePolicy,
) -> Option<()> {
    let replace = match minimum.as_ref() {
        Some(current) => {
            compare_reals_with_policy(&candidate, current, policy).value()? == Ordering::Less
        }
        None => true,
    };
    if replace {
        *minimum = Some(candidate);
    }
    Some(())
}

fn unknown_trace_report() -> TraceClearanceReport {
    TraceClearanceReport {
        status: ClearanceStatus::Unknown,
        centerline_intersection: None,
        axis_gap: None,
    }
}

fn unknown_pad_board_report() -> PadBoardClearanceReport {
    PadBoardClearanceReport {
        status: ClearanceStatus::Unknown,
        copper_gap: None,
    }
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

fn edge_cross(a: &Point2, b: &Point2, c: &Point2) -> Real {
    let ab_x = b.x.clone() - a.x.clone();
    let ab_y = b.y.clone() - a.y.clone();
    let ac_x = c.x.clone() - a.x.clone();
    let ac_y = c.y.clone() - a.y.clone();
    ab_x * ac_y - ab_y * ac_x
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

fn cross(first: &Point2, second: &Point2) -> Real {
    first.x.clone() * second.y.clone() - first.y.clone() * second.x.clone()
}
