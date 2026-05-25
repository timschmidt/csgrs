//! Exact orthogonal polygon PCB pad predicates.
//!
//! Non-convex orthogonal pads cover castellated/custom SMD copper, thermal
//! islands, and footprint artwork that cannot be represented by the convex pad
//! carrier. This module keeps those pads as retained path-domain polygon
//! records: it validates a simple rectilinear boundary, then certifies trace
//! and board clearance by exact edge predicates. It deliberately does not
//! triangulate, union copper, clip to boards, or materialize meshes. That is
//! Yap's object/predicate split from "Towards Exact Geometric Computation"
//! (*Computational Geometry* 7.1-2, 1997): imported geometry remains a precise
//! object until exact predicates decide whether a candidate route is valid. The
//! containment test is an exact rectilinear specialization of Shimrat's
//! crossing test and Haines' "Point in Polygon Strategies" (*Graphics Gems IV*,
//! 1994), avoiding tolerance rays or sampled fill rules.

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

/// Cached facts for an exact orthogonal polygon PCB pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbOrthogonalPadFacts {
    /// Exact-set facts across all retained vertex coordinates.
    pub exact: RealExactSetFacts,
    /// Certified winding orientation.
    pub orientation: BoardContourOrientation,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact simple orthogonal PCB pad.
///
/// The pad may be non-convex, but every edge must be horizontal or vertical and
/// the boundary must be simple. Keeping this footprint as an ordered polygon
/// lets route predicates reason about notches and tabs while avoiding copper
/// boolean execution in `hyperpath`.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbOrthogonalPad {
    net: crate::pcb::NetId,
    layer: crate::pcb::TraceLayer,
    vertices: Vec<Point2>,
    facts: PcbOrthogonalPadFacts,
}

impl PcbOrthogonalPad {
    /// Construct an orthogonal polygon pad with native provenance.
    pub fn new(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        vertices: Vec<Point2>,
    ) -> Result<Self, BoardContourError> {
        Self::with_provenance(net, layer, vertices, PathProvenance::native())
    }

    /// Construct an orthogonal polygon pad with source provenance.
    pub fn with_provenance(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        vertices: Vec<Point2>,
        provenance: PathProvenance,
    ) -> Result<Self, BoardContourError> {
        if vertices.len() < 4 {
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
        validate_orthogonal_edges(&vertices)?;
        validate_simple_polygon(&vertices)?;
        let refs = vertices
            .iter()
            .flat_map(|point| [&point.x, &point.y])
            .collect::<Vec<_>>();
        let facts = PcbOrthogonalPadFacts {
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
    pub const fn facts(&self) -> &PcbOrthogonalPadFacts {
        &self.facts
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }
}

/// Check same-layer different-net clearance between a trace and an orthogonal pad.
///
/// Endpoint containment and edge intersections certify contact exactly.
/// Otherwise the minimum squared distance between the trace centerline and
/// every retained pad edge is compared against the swept trace width plus the
/// requested clearance.
pub fn check_trace_orthogonal_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbOrthogonalPad,
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
        segment_orthogonal_polygon_distance_squared(trace.swept().centerline(), pad, policy)
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

/// Check exact clearance from an orthogonal pad to a rectangular board.
///
/// For an axis-aligned board envelope, pad-to-edge clearance is decided by the
/// exact extrema of the retained polygon vertices. This reports manufacturing
/// fit only; it does not clip the pad to the board or execute a copper boolean.
pub fn check_orthogonal_pad_board_clearance(
    pad: &PcbOrthogonalPad,
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

fn segment_orthogonal_polygon_distance_squared(
    segment: &LinePathSegment,
    pad: &PcbOrthogonalPad,
    policy: PredicatePolicy,
) -> Option<Real> {
    if point_inside_orthogonal_polygon(segment.start(), pad.vertices(), policy)?
        || point_inside_orthogonal_polygon(segment.end(), pad.vertices(), policy)?
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

fn point_inside_orthogonal_polygon(
    point: &Point2,
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Option<bool> {
    let mut crossings = 0usize;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        if point_on_closed_segment(point, start, end, policy)? {
            return Some(true);
        }
        if compare_reals_with_policy(&start.x, &end.x, policy).value()? != Ordering::Equal {
            continue;
        }
        let (min_y, max_y) = ordered_pair(&start.y, &end.y, policy)?;
        let above_low = compare_reals_with_policy(&min_y, &point.y, policy).value()?;
        let below_high = compare_reals_with_policy(&point.y, &max_y, policy).value()?;
        if !matches!(above_low, Ordering::Less | Ordering::Equal)
            || !matches!(below_high, Ordering::Less)
        {
            continue;
        }
        if compare_reals_with_policy(&point.x, &start.x, policy).value()? == Ordering::Less {
            crossings += 1;
        }
    }
    Some(crossings % 2 == 1)
}

fn point_on_closed_segment(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> Option<bool> {
    let cross_value = edge_cross(start, end, point);
    if compare_reals_with_policy(&cross_value, &Real::zero(), policy).value()? != Ordering::Equal {
        return Some(false);
    }
    let (min_x, max_x) = ordered_pair(&start.x, &end.x, policy)?;
    let (min_y, max_y) = ordered_pair(&start.y, &end.y, policy)?;
    Some(
        matches!(
            compare_reals_with_policy(&min_x, &point.x, policy).value()?,
            Ordering::Less | Ordering::Equal
        ) && matches!(
            compare_reals_with_policy(&point.x, &max_x, policy).value()?,
            Ordering::Less | Ordering::Equal
        ) && matches!(
            compare_reals_with_policy(&min_y, &point.y, policy).value()?,
            Ordering::Less | Ordering::Equal
        ) && matches!(
            compare_reals_with_policy(&point.y, &max_y, policy).value()?,
            Ordering::Less | Ordering::Equal
        ),
    )
}

fn validate_orthogonal_edges(vertices: &[Point2]) -> Result<(), BoardContourError> {
    for edge in polygon_edges(vertices) {
        match edge.facts().axis_aligned {
            Some(_) if edge.facts().known_degenerate == Some(false) => {}
            Some(_) => return Err(BoardContourError::CollinearEdge),
            None => return Err(BoardContourError::NonOrthogonal),
        }
    }
    Ok(())
}

fn validate_simple_polygon(vertices: &[Point2]) -> Result<(), BoardContourError> {
    let edges = polygon_edges(vertices);
    for first_index in 0..edges.len() {
        for second_index in (first_index + 1)..edges.len() {
            if adjacent_edges(first_index, second_index, edges.len()) {
                continue;
            }
            let first = &edges[first_index];
            let second = &edges[second_index];
            let relation = classify_segment_intersection_with_facts(
                first.start(),
                first.end(),
                second.start(),
                second.end(),
                first.facts().segment,
                second.facts().segment,
            )
            .value()
            .ok_or(BoardContourError::UnknownOrientation)?;
            if !matches!(relation, SegmentIntersection::Disjoint) {
                return Err(BoardContourError::SelfIntersecting);
            }
        }
    }
    Ok(())
}

fn adjacent_edges(first: usize, second: usize, len: usize) -> bool {
    first + 1 == second || (first == 0 && second + 1 == len)
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

fn ordered_pair(first: &Real, second: &Real, policy: PredicatePolicy) -> Option<(Real, Real)> {
    match compare_reals_with_policy(first, second, policy).value()? {
        Ordering::Less | Ordering::Equal => Some((first.clone(), second.clone())),
        Ordering::Greater => Some((second.clone(), first.clone())),
    }
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
