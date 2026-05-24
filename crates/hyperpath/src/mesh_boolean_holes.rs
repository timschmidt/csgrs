//! Shared strict-hole validation for orthogonal mesh-boolean sources.
//!
//! Holed PCB pours and CAM pockets with islands both need the same planar
//! certificate before any 3D boolean can be trusted: each inner loop must be
//! strictly inside the outer orthogonal loop, and inner loops must be pairwise
//! disjoint. This module keeps that predicate layer separate from domain
//! semantics. The point-in-loop test follows Shimrat, "Algorithm 112: Position
//! of Point Relative to Polygon," *Communications of the ACM* 5.8 (1962), as
//! corrected and surveyed by Haines, "Point in Polygon Strategies," *Graphics
//! Gems IV* (1994); all comparisons are exact `hyperlimit` predicate reports
//! in the object/predicate style advocated by Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997).

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::mesh_boolean::PathMeshBooleanError;

/// Validate strict containment and disjointness for orthogonal hole loops.
pub(crate) fn validate_strict_orthogonal_holes(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    if holes.is_empty() {
        return Err(PathMeshBooleanError::EmptyPolygonHoles);
    }
    validate_holes_strictly_inside_outer(outer, holes, policy)?;
    validate_holes_pairwise_disjoint(holes, policy)
}

fn validate_holes_strictly_inside_outer(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    for hole in holes {
        for point in hole {
            if classify_point_in_orthogonal_loop(point, outer, policy)?
                != OrthogonalLoopPointLocation::Inside
            {
                return Err(PathMeshBooleanError::PolygonHoleOutsideOuter);
            }
        }
        if loops_have_edge_intersection(outer, hole, policy)? {
            return Err(PathMeshBooleanError::PolygonHoleOutsideOuter);
        }
    }
    Ok(())
}

fn validate_holes_pairwise_disjoint(
    holes: &[Vec<Point2>],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    for left in 0..holes.len() {
        for right in left + 1..holes.len() {
            if loops_have_edge_intersection(&holes[left], &holes[right], policy)? {
                return Err(PathMeshBooleanError::PolygonHoleOverlap);
            }
            if holes[left].iter().any(|point| {
                classify_point_in_orthogonal_loop(point, &holes[right], policy)
                    == Ok(OrthogonalLoopPointLocation::Inside)
            }) || holes[right].iter().any(|point| {
                classify_point_in_orthogonal_loop(point, &holes[left], policy)
                    == Ok(OrthogonalLoopPointLocation::Inside)
            }) {
                return Err(PathMeshBooleanError::PolygonHoleOverlap);
            }
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OrthogonalLoopPointLocation {
    Inside,
    Boundary,
    Outside,
}

fn classify_point_in_orthogonal_loop(
    point: &Point2,
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Result<OrthogonalLoopPointLocation, PathMeshBooleanError> {
    let mut inside = false;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        if point_on_axis_aligned_segment(point, start, end, policy)? {
            return Ok(OrthogonalLoopPointLocation::Boundary);
        }
        if compare_reals_with_policy(&start.x, &end.x, policy).value() != Some(Ordering::Equal) {
            continue;
        }
        let y_min = real_min(&start.y, &end.y, policy)?;
        let y_max = real_max(&start.y, &end.y, policy)?;
        let crosses_lower =
            compare_reals_with_policy(&point.y, y_min, policy).value() != Some(Ordering::Less);
        let crosses_upper =
            compare_reals_with_policy(&point.y, y_max, policy).value() == Some(Ordering::Less);
        let right_of_point = compare_reals_with_policy(&start.x, &point.x, policy).value()
            == Some(Ordering::Greater);
        if crosses_lower && crosses_upper && right_of_point {
            inside = !inside;
        }
    }
    Ok(if inside {
        OrthogonalLoopPointLocation::Inside
    } else {
        OrthogonalLoopPointLocation::Outside
    })
}

fn loops_have_edge_intersection(
    left: &[Point2],
    right: &[Point2],
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    for left_index in 0..left.len() {
        let left_next = (left_index + 1) % left.len();
        for right_index in 0..right.len() {
            let right_next = (right_index + 1) % right.len();
            if orthogonal_segments_intersect(
                &left[left_index],
                &left[left_next],
                &right[right_index],
                &right[right_next],
                policy,
            )? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn point_on_axis_aligned_segment(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let same_x = compare_reals_with_policy(&start.x, &end.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    let same_y = compare_reals_with_policy(&start.y, &end.y, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    if same_x {
        let point_same_x = compare_reals_with_policy(&point.x, &start.x, policy)
            .value()
            .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
            == Ordering::Equal;
        return Ok(point_same_x && interval_contains_point(&start.y, &end.y, &point.y, policy)?);
    }
    if same_y {
        let point_same_y = compare_reals_with_policy(&point.y, &start.y, policy)
            .value()
            .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
            == Ordering::Equal;
        return Ok(point_same_y && interval_contains_point(&start.x, &end.x, &point.x, policy)?);
    }
    Ok(false)
}

fn orthogonal_segments_intersect(
    a0: &Point2,
    a1: &Point2,
    b0: &Point2,
    b1: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let a_vertical = compare_reals_with_policy(&a0.x, &a1.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    let b_vertical = compare_reals_with_policy(&b0.x, &b1.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    match (a_vertical, b_vertical) {
        (true, true) => Ok(real_equal(&a0.x, &b0.x, policy)?
            && ranges_overlap(&a0.y, &a1.y, &b0.y, &b1.y, policy)?),
        (false, false) => Ok(real_equal(&a0.y, &b0.y, policy)?
            && ranges_overlap(&a0.x, &a1.x, &b0.x, &b1.x, policy)?),
        (true, false) => Ok(real_between_closed(&a0.x, &b0.x, &b1.x, policy)?
            && real_between_closed(&b0.y, &a0.y, &a1.y, policy)?),
        (false, true) => Ok(real_between_closed(&b0.x, &a0.x, &a1.x, policy)?
            && real_between_closed(&a0.y, &b0.y, &b1.y, policy)?),
    }
}

fn interval_contains_point(
    a0: &Real,
    a1: &Real,
    point: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let min = real_min(a0, a1, policy)?;
    let max = real_max(a0, a1, policy)?;
    Ok(
        compare_reals_with_policy(point, min, policy).value() != Some(Ordering::Less)
            && compare_reals_with_policy(point, max, policy).value() != Some(Ordering::Greater),
    )
}

fn ranges_overlap(
    a0: &Real,
    a1: &Real,
    b0: &Real,
    b1: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let a_min = real_min(a0, a1, policy)?;
    let a_max = real_max(a0, a1, policy)?;
    let b_min = real_min(b0, b1, policy)?;
    let b_max = real_max(b0, b1, policy)?;
    Ok(
        compare_reals_with_policy(a_min, b_max, policy).value() != Some(Ordering::Greater)
            && compare_reals_with_policy(b_min, a_max, policy).value() != Some(Ordering::Greater),
    )
}

fn real_between_closed(
    value: &Real,
    a: &Real,
    b: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let min = real_min(a, b, policy)?;
    let max = real_max(a, b, policy)?;
    Ok(
        compare_reals_with_policy(min, value, policy).value() != Some(Ordering::Greater)
            && compare_reals_with_policy(value, max, policy).value() != Some(Ordering::Greater),
    )
}

fn real_min<'a>(
    a: &'a Real,
    b: &'a Real,
    policy: PredicatePolicy,
) -> Result<&'a Real, PathMeshBooleanError> {
    match compare_reals_with_policy(a, b, policy).value() {
        Some(Ordering::Less | Ordering::Equal) => Ok(a),
        Some(Ordering::Greater) => Ok(b),
        None => Err(PathMeshBooleanError::UnknownPolygonOrientation),
    }
}

fn real_max<'a>(
    a: &'a Real,
    b: &'a Real,
    policy: PredicatePolicy,
) -> Result<&'a Real, PathMeshBooleanError> {
    match compare_reals_with_policy(a, b, policy).value() {
        Some(Ordering::Greater | Ordering::Equal) => Ok(a),
        Some(Ordering::Less) => Ok(b),
        None => Err(PathMeshBooleanError::UnknownPolygonOrientation),
    }
}

fn real_equal(a: &Real, b: &Real, policy: PredicatePolicy) -> Result<bool, PathMeshBooleanError> {
    compare_reals_with_policy(a, b, policy)
        .value()
        .map(|ordering| ordering == Ordering::Equal)
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)
}
