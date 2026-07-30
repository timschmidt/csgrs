//! Pairwise convex polygon intersection primitives.

use hyperlattice::{
    HomogeneousPoint3, Point3, intersect_homogeneous_line_plane, intersect_two_planes,
};

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, cross_arrays, dot_point, sub_points};
use crate::polygon::ConvexPolygon;
use crate::predicate::{
    classify_point_decision, classify_projective_point_decision, classify_real,
};
use crate::segment_trace::certified_leaf_test_points;

/// Intersection segment between two polygons.
#[derive(Clone, Debug, PartialEq)]
pub struct IntersectionSegment {
    /// First segment endpoint.
    pub v0: Point3,
    /// Second segment endpoint.
    pub v1: Point3,
    /// Supporting plane of the other polygon.
    pub split_plane: Plane,
    /// Local index of the other polygon.
    pub other_polygon_idx: usize,
}

/// Coplanar overlap information.
#[derive(Clone, Debug, PartialEq)]
pub struct OverlapInfo {
    /// Local index of the other polygon.
    pub other_polygon_idx: usize,
    /// Edge planes of the other polygon.
    pub other_edges: Vec<Plane>,
    /// Supporting plane of the other polygon.
    pub other_support: Plane,
}

/// Type of pairwise polygon intersection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PairwiseIntersectionType {
    /// No intersection.
    None,
    /// Single point intersection.
    Point,
    /// Non-degenerate segment intersection.
    Segment,
    /// Coplanar non-empty overlap.
    Overlap,
}

/// Result of intersecting two convex polygons.
#[derive(Clone, Debug, PartialEq)]
pub struct PairwiseIntersection {
    /// Intersection kind.
    pub kind: PairwiseIntersectionType,
    /// Segment payload when `kind == Segment`.
    pub segment: Option<IntersectionSegment>,
    /// Overlap payload when `kind == Overlap`.
    pub overlap: Option<OverlapInfo>,
}

impl PairwiseIntersection {
    /// Creates a no-intersection result.
    pub const fn none() -> Self {
        Self {
            kind: PairwiseIntersectionType::None,
            segment: None,
            overlap: None,
        }
    }

    /// Creates a point-intersection result.
    pub const fn point() -> Self {
        Self {
            kind: PairwiseIntersectionType::Point,
            segment: None,
            overlap: None,
        }
    }
}

/// Computes the pairwise intersection between two convex polygons.
pub fn intersect_polygons(
    context: &MeshContext,
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
    other_polygon_idx: usize,
) -> HypermeshResult<MeshOutcome<PairwiseIntersection>> {
    let decisions = DecisionContext::new(context);
    let polygon_vertices = polygon.vertices_decision(&decisions)?;
    let other_vertices = other.vertices_decision(&decisions)?;
    let intersection = intersect_polygons_with_vertices(
        &decisions,
        polygon,
        &polygon_vertices,
        other,
        &other_vertices,
        other_polygon_idx,
    )?;
    Ok(decisions.finish(intersection))
}

/// Computes a pairwise intersection from affine vertices already materialized
/// for both polygons. Subdivision compares each polygon with many candidates,
/// so retaining these exact points at that boundary avoids repeatedly solving
/// the same adjacent plane triples.
pub(crate) fn intersect_polygons_with_vertices(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    if polygon.vertex_count() == 0 || other.vertex_count() == 0 {
        return Ok(PairwiseIntersection::none());
    }

    let supports_parallel = supports_are_parallel(decisions, &polygon.support, &other.support)?;
    if supports_parallel {
        crate::trace_dispatch!("intersect-polygons", "parallel-supports");
        let other_vertex = other_vertices
            .first()
            .ok_or(HypermeshError::UnknownClassification)?;
        return if classify_point_decision(decisions, other_vertex, &polygon.support)?
            == Classification::On
        {
            intersect_coplanar(
                decisions,
                polygon,
                polygon_vertices,
                other,
                other_vertices,
                other_polygon_idx,
            )
        } else {
            Ok(PairwiseIntersection::none())
        };
    }

    let mut points = Vec::new();
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-forward");
    collect_edge_plane_crossings(decisions, polygon, polygon_vertices, other, &mut points)?;
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-reverse");
    collect_edge_plane_crossings(decisions, other, other_vertices, polygon, &mut points)?;
    dedup_points(&mut points);

    match points.len() {
        0 => Ok(PairwiseIntersection::none()),
        1 => Ok(PairwiseIntersection::point()),
        _ => Ok(PairwiseIntersection {
            kind: PairwiseIntersectionType::Segment,
            segment: Some(IntersectionSegment {
                v0: points[0].clone(),
                v1: points[1].clone(),
                split_plane: other.support.clone(),
                other_polygon_idx,
            }),
            overlap: None,
        }),
    }
}

fn intersect_coplanar(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    if polygons_share_area(decisions, polygon, polygon_vertices, other, other_vertices)? {
        Ok(PairwiseIntersection {
            kind: PairwiseIntersectionType::Overlap,
            segment: None,
            overlap: Some(OverlapInfo {
                other_polygon_idx,
                other_edges: other.edges.as_ref().clone(),
                other_support: other.support.clone(),
            }),
        })
    } else {
        Ok(PairwiseIntersection::none())
    }
}

fn polygons_share_area(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for (candidate, container) in [(polygon, other), (other, polygon)] {
        match polygon_has_certified_interior_witness_in_other(decisions, candidate, container) {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => {
                crate::trace_dispatch!("coplanar-overlap", "witness-unknown");
                saw_unknown = true;
            }
            Err(error) => return Err(error),
        }
    }

    for point in polygon_vertices {
        match affine_point_strictly_in_polygon(decisions, point, other) {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => {
                crate::trace_dispatch!("coplanar-overlap", "left-vertex-unknown");
                saw_unknown = true;
            }
            Err(error) => return Err(error),
        }
    }
    for point in other_vertices {
        match affine_point_strictly_in_polygon(decisions, point, polygon) {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => {
                crate::trace_dispatch!("coplanar-overlap", "right-vertex-unknown");
                saw_unknown = true;
            }
            Err(error) => return Err(error),
        }
    }

    for edge in segment_edges(polygon_vertices) {
        for other_edge in segment_edges(other_vertices) {
            match segments_properly_cross(
                decisions,
                edge.0,
                edge.1,
                other_edge.0,
                other_edge.1,
                &polygon.support,
            ) {
                Ok(true) => return Ok(true),
                Ok(false) => {}
                Err(HypermeshError::PredicateUndecided { .. }) => {
                    crate::trace_dispatch!("coplanar-overlap", "edge-crossing-unknown");
                    saw_unknown = true;
                }
                Err(error) => return Err(error),
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "coplanar polygon positive-area overlap",
        })
    } else {
        Ok(false)
    }
}

fn polygon_has_certified_interior_witness_in_other(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for point in certified_leaf_test_points(decisions, &polygon.support, &polygon.edges)? {
        match other.contains_point_decision(decisions, &point) {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "coplanar polygon interior witness",
        })
    } else {
        Ok(false)
    }
}

fn collect_edge_plane_crossings(
    decisions: &DecisionContext,
    edge_polygon: &ConvexPolygon,
    vertices: &[Point3],
    plane_polygon: &ConvexPolygon,
    points: &mut Vec<Point3>,
) -> HypermeshResult<()> {
    if let [v0, v1, v2] = vertices {
        let c0 = classify_point_decision(decisions, v0, &plane_polygon.support)?;
        let c1 = classify_point_decision(decisions, v1, &plane_polygon.support)?;
        let c2 = classify_point_decision(decisions, v2, &plane_polygon.support)?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            0,
            v0,
            v1,
            c0,
            c1,
            plane_polygon,
            points,
        )?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            1,
            v1,
            v2,
            c1,
            c2,
            plane_polygon,
            points,
        )?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            2,
            v2,
            v0,
            c2,
            c0,
            plane_polygon,
            points,
        )?;
        return Ok(());
    }

    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        let start_class = classify_point_decision(decisions, start, &plane_polygon.support)?;
        let end_class = classify_point_decision(decisions, end, &plane_polygon.support)?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            index,
            start,
            end,
            start_class,
            end_class,
            plane_polygon,
            points,
        )?;
    }
    Ok(())
}

#[inline]
fn collect_edge_plane_crossing(
    decisions: &DecisionContext,
    edge_polygon: &ConvexPolygon,
    edge_index: usize,
    start: &Point3,
    end: &Point3,
    start_class: Classification,
    end_class: Classification,
    plane_polygon: &ConvexPolygon,
    points: &mut Vec<Point3>,
) -> HypermeshResult<()> {
    let candidate = match (start_class, end_class) {
        (Classification::On, _) => {
            affine_point_in_polygon_on_support(decisions, start, plane_polygon)?
                .then(|| start.clone())
        }
        (_, Classification::On) => {
            affine_point_in_polygon_on_support(decisions, end, plane_polygon)?.then(|| end.clone())
        }
        (Classification::Negative, Classification::Positive)
        | (Classification::Positive, Classification::Negative) => {
            let point = intersect_segment_plane(start, end, &plane_polygon.support)?;
            let contained =
                match affine_point_in_polygon_on_support(decisions, &point, plane_polygon) {
                    Ok(contained) => contained,
                    Err(HypermeshError::PredicateUndecided { .. }) => {
                        match projective_edge_plane_intersection_in_polygon(
                            decisions,
                            edge_polygon,
                            edge_index,
                            plane_polygon,
                        ) {
                            Ok(contained) => contained,
                            Err(HypermeshError::PredicateUndecided { .. }) => {
                                segment_plane_intersection_in_polygon(
                                    decisions,
                                    start,
                                    end,
                                    start_class,
                                    plane_polygon,
                                )?
                            }
                            Err(error) => return Err(error),
                        }
                    }
                    Err(error) => return Err(error),
                };
            contained.then_some(point)
        }
        _ => None,
    };

    if let Some(point) = candidate {
        points.push(point);
    }
    Ok(())
}

fn projective_edge_plane_intersection_in_polygon(
    decisions: &DecisionContext,
    edge_polygon: &ConvexPolygon,
    edge_index: usize,
    plane_polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let edge_plane = edge_polygon
        .edges
        .get(edge_index)
        .ok_or(HypermeshError::UnknownClassification)?;
    let line = intersect_two_planes(&edge_polygon.support, edge_plane);
    let point = intersect_homogeneous_line_plane(&line, &plane_polygon.support);
    let mut saw_unknown = false;
    for edge in plane_polygon.edges.iter() {
        match classify_projective_point_decision(decisions, &point, edge) {
            Ok(Classification::Positive) => return Ok(false),
            Ok(Classification::Negative | Classification::On) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => {
                if homogeneous_point_certifiably_nonzero(decisions, &point)
                    && crate::predicate::classify_real(
                        decisions,
                        &four_plane_determinant(
                            &edge_polygon.support,
                            edge_plane,
                            &plane_polygon.support,
                            edge,
                        ),
                    ) == Ok(Classification::On)
                {
                    continue;
                }
                saw_unknown = true;
            }
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "projective edge/polygon containment",
        })
    } else {
        Ok(true)
    }
}

fn homogeneous_point_certifiably_nonzero(
    decisions: &DecisionContext,
    point: &HomogeneousPoint3,
) -> bool {
    [&point.x, &point.y, &point.z, &point.w]
        .into_iter()
        .any(|coordinate| {
            matches!(
                crate::predicate::classify_real(decisions, coordinate),
                Ok(Classification::Negative | Classification::Positive)
            )
        })
}

pub(crate) fn four_plane_determinant(
    a: &Plane,
    b: &Plane,
    c: &Plane,
    d: &Plane,
) -> hyperlattice::Real {
    const PERMUTATIONS: [[usize; 4]; 24] = [
        [0, 1, 2, 3],
        [0, 1, 3, 2],
        [0, 2, 1, 3],
        [0, 2, 3, 1],
        [0, 3, 1, 2],
        [0, 3, 2, 1],
        [1, 0, 2, 3],
        [1, 0, 3, 2],
        [1, 2, 0, 3],
        [1, 2, 3, 0],
        [1, 3, 0, 2],
        [1, 3, 2, 0],
        [2, 0, 1, 3],
        [2, 0, 3, 1],
        [2, 1, 0, 3],
        [2, 1, 3, 0],
        [2, 3, 0, 1],
        [2, 3, 1, 0],
        [3, 0, 1, 2],
        [3, 0, 2, 1],
        [3, 1, 0, 2],
        [3, 1, 2, 0],
        [3, 2, 0, 1],
        [3, 2, 1, 0],
    ];
    const POSITIVE: [bool; 24] = [
        true, false, false, true, true, false, false, true, true, false, false, true, true, false,
        false, true, true, false, false, true, true, false, false, true,
    ];
    let rows = [
        [&a.normal.x, &a.normal.y, &a.normal.z, &a.offset],
        [&b.normal.x, &b.normal.y, &b.normal.z, &b.offset],
        [&c.normal.x, &c.normal.y, &c.normal.z, &c.offset],
        [&d.normal.x, &d.normal.y, &d.normal.z, &d.offset],
    ];
    let terms: [[&hyperlattice::Real; 4]; 24] =
        std::array::from_fn(|term| std::array::from_fn(|row| rows[row][PERMUTATIONS[term][row]]));
    hyperlattice::Real::signed_product_sum(POSITIVE, terms)
}

/// Certifies containment of a proper segment/plane intersection without first
/// expanding the affine intersection point.
///
/// For support values `a`, `b` at the segment endpoints and edge-plane values
/// `q0`, `q1`, the edge value at the intersection is
/// `(a*q1 - b*q0) / (a - b)`. The endpoints are known to be on opposite sides,
/// so the denominator sign is already certified by `start_class`. Keeping this
/// predicate as a two-term determinant preserves cancellations that can become
/// opaque after all three affine coordinates are materialized.
fn segment_plane_intersection_in_polygon(
    decisions: &DecisionContext,
    start: &Point3,
    end: &Point3,
    start_class: Classification,
    polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    debug_assert!(matches!(
        start_class,
        Classification::Negative | Classification::Positive
    ));

    let start_support = polygon.support.expression_at_point(start);
    let end_support = polygon.support.expression_at_point(end);
    let denominator_is_positive = start_class == Classification::Positive;
    let mut saw_unknown = false;

    for edge in polygon.edges.iter() {
        let start_edge = edge.expression_at_point(start);
        let end_edge = edge.expression_at_point(end);
        let numerator = hyperlattice::Real::signed_product_sum(
            [true, false],
            [[&start_support, &end_edge], [&end_support, &start_edge]],
        );
        let candidate_class = match classify_real(decisions, &numerator) {
            Ok(classification) if denominator_is_positive => classification,
            Ok(Classification::Negative) => Classification::Positive,
            Ok(Classification::Positive) => Classification::Negative,
            Ok(Classification::On) => Classification::On,
            Err(HypermeshError::PredicateUndecided { .. }) => {
                saw_unknown = true;
                continue;
            }
            Err(error) => return Err(error),
        };
        if candidate_class == Classification::Positive {
            return Ok(false);
        }
    }

    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "segment-plane intersection containment",
        })
    } else {
        Ok(true)
    }
}

fn intersect_segment_plane(start: &Point3, end: &Point3, plane: &Plane) -> HypermeshResult<Point3> {
    let start_value = plane.expression_at_point(start);
    let end_value = plane.expression_at_point(end);
    let denom = &start_value - &end_value;
    let t = (start_value / denom).map_err(|_| HypermeshError::UnknownClassification)?;

    Ok(Point3::new(
        &start.x + &(t.clone() * (&end.x - &start.x)),
        &start.y + &(t.clone() * (&end.y - &start.y)),
        &start.z + &(t * (&end.z - &start.z)),
    ))
}

fn affine_point_in_polygon_on_support(
    decisions: &DecisionContext,
    point: &Point3,
    polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    if polygon.has_retained_vertex(point) {
        return Ok(true);
    }
    let mut saw_unknown = false;
    for edge in polygon.edges.iter() {
        match classify_point_decision(decisions, point, edge) {
            Ok(Classification::Positive) => return Ok(false),
            Ok(Classification::Negative | Classification::On) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "affine point/polygon containment",
        })
    } else {
        Ok(true)
    }
}

fn affine_point_strictly_in_polygon(
    decisions: &DecisionContext,
    point: &Point3,
    polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    if polygon.has_retained_vertex(point) {
        return Ok(false);
    }
    if classify_point_decision(decisions, point, &polygon.support)? != Classification::On {
        return Ok(false);
    }
    let mut saw_unknown = false;
    for edge in polygon.edges.iter() {
        match classify_point_decision(decisions, point, edge) {
            Ok(Classification::Negative) => {}
            Ok(Classification::On | Classification::Positive) => return Ok(false),
            Err(HypermeshError::PredicateUndecided { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "strict affine point/polygon containment",
        })
    } else {
        Ok(true)
    }
}

fn segment_edges(vertices: &[Point3]) -> impl Iterator<Item = (&Point3, &Point3)> {
    vertices
        .iter()
        .zip(vertices.iter().cycle().skip(1))
        .take(vertices.len())
}

fn segments_properly_cross(
    decisions: &DecisionContext,
    a0: &Point3,
    a1: &Point3,
    b0: &Point3,
    b1: &Point3,
    support: &Plane,
) -> HypermeshResult<bool> {
    let a_line = segment_split_plane(a0, a1, support);
    let b_line = segment_split_plane(b0, b1, support);

    let b0_side = classify_point_decision(decisions, b0, &a_line)?;
    let b1_side = classify_point_decision(decisions, b1, &a_line)?;
    let a0_side = classify_point_decision(decisions, a0, &b_line)?;
    let a1_side = classify_point_decision(decisions, a1, &b_line)?;

    Ok(((b0_side.is_negative() && b1_side.is_positive())
        || (b0_side.is_positive() && b1_side.is_negative()))
        && ((a0_side.is_negative() && a1_side.is_positive())
            || (a0_side.is_positive() && a1_side.is_negative())))
}

fn segment_split_plane(a: &Point3, b: &Point3, support: &Plane) -> Plane {
    let edge = sub_points(b, a);
    let support_normal = [
        support.normal.x.clone(),
        support.normal.y.clone(),
        support.normal.z.clone(),
    ];
    let normal = cross_arrays(&edge, &support_normal);
    let offset = -dot_point(&normal, a);
    Plane::new(normal, offset)
}

fn supports_are_parallel(
    decisions: &DecisionContext,
    left: &Plane,
    right: &Plane,
) -> HypermeshResult<bool> {
    let cross = Point3::new(
        hyperlattice::Real::signed_product_sum(
            [true, false],
            [
                [&left.normal.y, &right.normal.z],
                [&left.normal.z, &right.normal.y],
            ],
        ),
        hyperlattice::Real::signed_product_sum(
            [true, false],
            [
                [&left.normal.z, &right.normal.x],
                [&left.normal.x, &right.normal.z],
            ],
        ),
        hyperlattice::Real::signed_product_sum(
            [true, false],
            [
                [&left.normal.x, &right.normal.y],
                [&left.normal.y, &right.normal.x],
            ],
        ),
    );
    let mut saw_unknown = false;
    for component in [&cross.x, &cross.y, &cross.z] {
        match classify_real(decisions, component) {
            Ok(Classification::On) => {}
            Ok(Classification::Negative | Classification::Positive) => return Ok(false),
            Err(HypermeshError::PredicateUndecided { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "polygon support-plane parallelism",
        })
    } else {
        Ok(true)
    }
}

fn dedup_points(points: &mut Vec<Point3>) {
    let mut unique = Vec::with_capacity(points.len());
    for point in points.drain(..) {
        if !unique.iter().any(|existing| existing == &point) {
            unique.push(point);
        }
    }
    *points = unique;
}
