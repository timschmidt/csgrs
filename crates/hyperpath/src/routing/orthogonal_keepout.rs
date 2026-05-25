//! Exact predicates for retained orthogonal route keepouts.
//!
//! Orthogonal polygon keepouts let router imports retain notched no-route
//! regions without turning `hyperpath` into a planar boolean engine. The
//! predicates here are the same object/predicate boundary advocated by Yap,
//! "Towards Exact Geometric Computation" (*Computational Geometry* 7.1-2,
//! 1997): source geometry is kept as exact vertices, and route candidates are
//! rejected only by exact decisions. Boundary containment uses Shimrat's
//! crossing rule as surveyed by Haines, "Point in Polygon Strategies"
//! (*Graphics Gems IV*, 1994), specialized to rectilinear loops.

use std::cmp::Ordering;

use hyperlimit::{
    Point2, PredicatePolicy, SegmentIntersection, classify_segment_intersection_with_facts,
    compare_reals_with_policy,
};
use hyperreal::Real;

use crate::routing::MeanderError;
use crate::segment::LinePathSegment;

pub(super) fn validate_orthogonal_keepout_vertices(
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Result<(), MeanderError> {
    if vertices.len() < 4 {
        return Err(MeanderError::InvalidObstaclePolygon);
    }
    match compare_reals_with_policy(&signed_area_twice(vertices), &Real::zero(), policy).value() {
        Some(Ordering::Less | Ordering::Greater) => {}
        Some(Ordering::Equal) => return Err(MeanderError::InvalidObstaclePolygon),
        None => return Err(MeanderError::ObstacleDecisionUnknown),
    }
    for edge in polygon_edges(vertices) {
        if edge.facts().axis_aligned.is_none() || edge.facts().known_degenerate != Some(false) {
            return Err(MeanderError::InvalidObstaclePolygon);
        }
    }
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
            .ok_or(MeanderError::ObstacleDecisionUnknown)?;
            if !matches!(relation, SegmentIntersection::Disjoint) {
                return Err(MeanderError::InvalidObstaclePolygon);
            }
        }
    }
    Ok(())
}

pub(super) fn segment_intersects_orthogonal_keepout(
    segment: &LinePathSegment,
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    if point_inside_orthogonal_polygon(segment.start(), vertices, policy)?
        || point_inside_orthogonal_polygon(segment.end(), vertices, policy)?
    {
        return Ok(true);
    }
    for edge in polygon_edges(vertices) {
        let relation = classify_segment_intersection_with_facts(
            segment.start(),
            segment.end(),
            edge.start(),
            edge.end(),
            segment.facts().segment,
            edge.facts().segment,
        )
        .value()
        .ok_or(MeanderError::ObstacleDecisionUnknown)?;
        if !matches!(relation, SegmentIntersection::Disjoint) {
            return Ok(true);
        }
    }
    Ok(false)
}

fn point_inside_orthogonal_polygon(
    point: &Point2,
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    let mut crossings = 0usize;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        if point_on_closed_segment(point, start, end, policy)? {
            return Ok(true);
        }
        if compare_reals_with_policy(&start.x, &end.x, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?
            != Ordering::Equal
        {
            continue;
        }
        let (min_y, max_y) = ordered_pair(&start.y, &end.y, policy)?;
        let above_low = compare_reals_with_policy(&min_y, &point.y, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?;
        let below_high = compare_reals_with_policy(&point.y, &max_y, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?;
        if !matches!(above_low, Ordering::Less | Ordering::Equal)
            || !matches!(below_high, Ordering::Less)
        {
            continue;
        }
        if compare_reals_with_policy(&point.x, &start.x, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?
            == Ordering::Less
        {
            crossings += 1;
        }
    }
    Ok(crossings % 2 == 1)
}

fn point_on_closed_segment(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    let cross_value = edge_cross(start, end, point);
    if compare_reals_with_policy(&cross_value, &Real::zero(), policy)
        .value()
        .ok_or(MeanderError::ObstacleDecisionUnknown)?
        != Ordering::Equal
    {
        return Ok(false);
    }
    let (min_x, max_x) = ordered_pair(&start.x, &end.x, policy)?;
    let (min_y, max_y) = ordered_pair(&start.y, &end.y, policy)?;
    Ok(matches!(
        compare_reals_with_policy(&min_x, &point.x, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?,
        Ordering::Less | Ordering::Equal
    ) && matches!(
        compare_reals_with_policy(&point.x, &max_x, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?,
        Ordering::Less | Ordering::Equal
    ) && matches!(
        compare_reals_with_policy(&min_y, &point.y, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?,
        Ordering::Less | Ordering::Equal
    ) && matches!(
        compare_reals_with_policy(&point.y, &max_y, policy)
            .value()
            .ok_or(MeanderError::ObstacleDecisionUnknown)?,
        Ordering::Less | Ordering::Equal
    ))
}

fn ordered_pair(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Result<(Real, Real), MeanderError> {
    match compare_reals_with_policy(first, second, policy)
        .value()
        .ok_or(MeanderError::ObstacleDecisionUnknown)?
    {
        Ordering::Less | Ordering::Equal => Ok((first.clone(), second.clone())),
        Ordering::Greater => Ok((second.clone(), first.clone())),
    }
}

fn signed_area_twice(vertices: &[Point2]) -> Real {
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

fn adjacent_edges(first: usize, second: usize, len: usize) -> bool {
    first + 1 == second || (first == 0 && second + 1 == len)
}

fn edge_cross(a: &Point2, b: &Point2, c: &Point2) -> Real {
    let ab_x = b.x.clone() - a.x.clone();
    let ab_y = b.y.clone() - a.y.clone();
    let ac_x = c.x.clone() - a.x.clone();
    let ac_y = c.y.clone() - a.y.clone();
    ab_x * ac_y - ab_y * ac_x
}
