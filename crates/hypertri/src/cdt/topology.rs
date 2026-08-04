//! Orientation-only point-set triangulation for constrained topology.

use super::{
    LOCATED_CAVITY_THRESHOLD, locate_triangle, make_oriented, spatial_point_cmp,
    triangle_if_not_degenerate, triangle_neighbors,
};
use crate::error::{Error, Result};
use crate::kernel::ExactKernel;
use crate::predicates;
use crate::types::{Point2, Sign, Triangle, TriangleLocation};

pub(super) fn triangulate_point_set(
    kernel: &ExactKernel,
    points: &[Point2],
) -> Result<Vec<Triangle>> {
    match points.len() {
        0..=2 => return Ok(Vec::new()),
        3 => {
            return triangle_if_not_degenerate(kernel, points, [0, 1, 2])?
                .map(|triangle| vec![triangle])
                .ok_or(Error::InvalidInput {
                    reason: "point set is collinear",
                });
        }
        _ => {}
    }

    let order = lexicographic_point_order(kernel, points)?;
    let mut hull = convex_hull_from_order(kernel, points, &order)?;
    if hull.len() < 3 {
        return Err(Error::InvalidInput {
            reason: "point set is collinear",
        });
    }

    let mut triangles = Vec::with_capacity(points.len().saturating_mul(2));
    for index in 1..hull.len() - 1 {
        triangles.push(make_oriented(
            kernel,
            points,
            [hull[0], hull[index], hull[index + 1]],
        )?);
    }

    hull.sort_unstable();
    for point in 0..points.len() {
        if hull.binary_search(&point).is_err() {
            insert_point(kernel, points, &mut triangles, point)?;
        }
    }

    if !crate::cdt_validate::triangulates_convex_hull(kernel, points, &triangles)? {
        return Err(Error::InvalidInput {
            reason: "topological point insertion did not cover the convex hull",
        });
    }
    Ok(triangles)
}

fn lexicographic_point_order(kernel: &ExactKernel, points: &[Point2]) -> Result<Vec<usize>> {
    let mut order = (0..points.len()).collect::<Vec<_>>();
    let mut merged = order.clone();
    let mut width = 1_usize;
    while width < order.len() {
        let mut start = 0;
        while start < order.len() {
            let middle = start.saturating_add(width).min(order.len());
            let end = middle.saturating_add(width).min(order.len());
            let (mut left, mut right) = (start, middle);
            for output in &mut merged[start..end] {
                let take_left = right == end
                    || (left < middle
                        && spatial_point_cmp(kernel, points, order[left], order[right], false)?
                            != std::cmp::Ordering::Greater);
                if take_left {
                    *output = order[left];
                    left += 1;
                } else {
                    *output = order[right];
                    right += 1;
                }
            }
            start = end;
        }
        order.copy_from_slice(&merged);
        width = width.saturating_mul(2);
    }
    Ok(order)
}

fn convex_hull_from_order(
    kernel: &ExactKernel,
    points: &[Point2],
    order: &[usize],
) -> Result<Vec<usize>> {
    fn append(
        kernel: &ExactKernel,
        points: &[Point2],
        half: &mut Vec<usize>,
        point: usize,
    ) -> Result<()> {
        while half.len() >= 2 {
            let end = half.len();
            if predicates::orient2(
                kernel,
                &points[half[end - 2]],
                &points[half[end - 1]],
                &points[point],
            )? == Sign::Positive
            {
                break;
            }
            half.pop();
        }
        half.push(point);
        Ok(())
    }

    let mut lower = Vec::with_capacity(order.len());
    for &point in order {
        append(kernel, points, &mut lower, point)?;
    }
    let mut upper = Vec::with_capacity(order.len());
    for &point in order.iter().rev() {
        append(kernel, points, &mut upper, point)?;
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    Ok(lower)
}

fn insert_point(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    point: usize,
) -> Result<()> {
    let mut located = None;
    if triangles.len() >= LOCATED_CAVITY_THRESHOLD {
        let neighbors = triangle_neighbors(triangles)?;
        if let Some(triangle) = locate_triangle(
            kernel,
            points,
            triangles,
            &neighbors,
            point,
            triangles.len().saturating_sub(1),
        )? {
            let location = kernel.classify_point_triangle(
                &points[triangles[triangle][0]],
                &points[triangles[triangle][1]],
                &points[triangles[triangle][2]],
                &points[point],
            )?;
            if location != TriangleLocation::Outside {
                located = Some((triangle, location));
            }
        }
    }
    if located.is_none() {
        for (triangle_index, triangle) in triangles.iter().copied().enumerate() {
            let location = kernel.classify_point_triangle(
                &points[triangle[0]],
                &points[triangle[1]],
                &points[triangle[2]],
                &points[point],
            )?;
            if !matches!(
                location,
                TriangleLocation::Outside | TriangleLocation::Degenerate
            ) {
                located = Some((triangle_index, location));
                break;
            }
        }
    }

    let Some((triangle_index, location)) = located else {
        return Err(Error::InvalidInput {
            reason: "point lies outside its convex-hull triangulation",
        });
    };
    match location {
        TriangleLocation::Inside => {
            let [a, b, c] = triangles[triangle_index];
            triangles[triangle_index] = make_oriented(kernel, points, [a, b, point])?;
            triangles.push(make_oriented(kernel, points, [b, c, point])?);
            triangles.push(make_oriented(kernel, points, [c, a, point])?);
            Ok(())
        }
        TriangleLocation::OnEdge => split_edge(kernel, points, triangles, triangle_index, point),
        TriangleLocation::OnVertex => Err(Error::InvalidInput {
            reason: "unique point coincides with a triangulation vertex",
        }),
        TriangleLocation::Degenerate => Err(Error::InvalidInput {
            reason: "topological point insertion reached a degenerate triangle",
        }),
        TriangleLocation::Outside => Err(Error::InvalidInput {
            reason: "point lies outside its convex-hull triangulation",
        }),
    }
}

fn split_edge(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    triangle_index: usize,
    point: usize,
) -> Result<()> {
    let triangle = triangles[triangle_index];
    let mut edge = None;
    for candidate in [
        (triangle[0], triangle[1]),
        (triangle[1], triangle[2]),
        (triangle[2], triangle[0]),
    ] {
        if predicates::point_on_segment(
            kernel,
            &points[candidate.0],
            &points[candidate.1],
            &points[point],
        )? {
            edge = Some(candidate);
            break;
        }
    }
    let edge = edge.ok_or(Error::InvalidInput {
        reason: "edge point was not on a triangulation edge",
    })?;

    let mut incident = [usize::MAX; 2];
    let mut incident_len = 0;
    for (index, triangle) in triangles.iter().enumerate() {
        if triangle.contains(&edge.0) && triangle.contains(&edge.1) {
            if incident_len == incident.len() {
                return Err(Error::InvalidInput {
                    reason: "triangulation edge has invalid incidence",
                });
            }
            incident[incident_len] = index;
            incident_len += 1;
        }
    }
    if incident_len == 0 || !incident[..incident_len].contains(&triangle_index) {
        return Err(Error::InvalidInput {
            reason: "triangulation edge has invalid incidence",
        });
    }

    for &index in &incident[..incident_len] {
        let source = triangles[index];
        let opposite = source
            .into_iter()
            .find(|vertex| *vertex != edge.0 && *vertex != edge.1)
            .ok_or(Error::InvalidInput {
                reason: "triangulation edge has no opposite vertex",
            })?;
        let first = make_oriented(kernel, points, [edge.0, point, opposite])?;
        let second = make_oriented(kernel, points, [point, edge.1, opposite])?;
        triangles[index] = first;
        triangles.push(second);
    }
    Ok(())
}
