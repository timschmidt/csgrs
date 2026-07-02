//! Pairwise convex polygon intersection primitives.

use hyperlattice::Point3;

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, classify_point, classify_real};
use crate::polygon::ConvexPolygon;

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
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    if polygon.vertex_count() == 0 || other.vertex_count() == 0 {
        return Ok(PairwiseIntersection::none());
    }

    let other_vertex = other.vertex_point(0)?;
    if classify_point(&other_vertex, &polygon.support)? == Classification::On
        && supports_are_parallel(&polygon.support, &other.support)?
    {
        return intersect_coplanar(polygon, other, other_polygon_idx);
    }

    if supports_are_parallel(&polygon.support, &other.support)? {
        return Ok(PairwiseIntersection::none());
    }

    let mut points = Vec::new();
    collect_edge_plane_crossings(polygon, other, &mut points)?;
    collect_edge_plane_crossings(other, polygon, &mut points)?;
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
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    if polygons_share_area(polygon, other)? {
        Ok(PairwiseIntersection {
            kind: PairwiseIntersectionType::Overlap,
            segment: None,
            overlap: Some(OverlapInfo {
                other_polygon_idx,
                other_edges: other.edges.clone(),
                other_support: other.support.clone(),
            }),
        })
    } else {
        Ok(PairwiseIntersection::none())
    }
}

fn polygons_share_area(polygon: &ConvexPolygon, other: &ConvexPolygon) -> HypermeshResult<bool> {
    for point in polygon.vertices()? {
        if affine_point_in_polygon(&point, other)? {
            return Ok(true);
        }
    }
    for point in other.vertices()? {
        if affine_point_in_polygon(&point, polygon)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn collect_edge_plane_crossings(
    edge_polygon: &ConvexPolygon,
    plane_polygon: &ConvexPolygon,
    points: &mut Vec<Point3>,
) -> HypermeshResult<()> {
    let vertices = edge_polygon.vertices()?;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        let start_class = classify_point(start, &plane_polygon.support)?;
        let end_class = classify_point(end, &plane_polygon.support)?;

        let candidate = match (start_class, end_class) {
            (Classification::On, _) => Some(start.clone()),
            (_, Classification::On) => Some(end.clone()),
            (Classification::Negative, Classification::Positive)
            | (Classification::Positive, Classification::Negative) => {
                Some(intersect_segment_plane(start, end, &plane_polygon.support)?)
            }
            _ => None,
        };

        if let Some(point) = candidate
            && affine_point_in_polygon(&point, edge_polygon)?
            && affine_point_in_polygon(&point, plane_polygon)?
        {
            points.push(point);
        }
    }
    Ok(())
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

fn affine_point_in_polygon(point: &Point3, polygon: &ConvexPolygon) -> HypermeshResult<bool> {
    if classify_point(point, &polygon.support)? != Classification::On {
        return Ok(false);
    }
    for edge in &polygon.edges {
        if classify_point(point, edge)?.is_positive() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn supports_are_parallel(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    let cross = Point3::new(
        (&left.normal.y * &right.normal.z) - (&left.normal.z * &right.normal.y),
        (&left.normal.z * &right.normal.x) - (&left.normal.x * &right.normal.z),
        (&left.normal.x * &right.normal.y) - (&left.normal.y * &right.normal.x),
    );
    Ok(classify_real(&cross.x)? == Classification::On
        && classify_real(&cross.y)? == Classification::On
        && classify_real(&cross.z)? == Classification::On)
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
