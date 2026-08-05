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

    if let Some(triangles) = triangulate_from_enclosing_prefix_triangle(kernel, points)? {
        return Ok(triangles);
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

/// Consume an already useful point order without making it part of the
/// topology contract. Some producers naturally retain the three vertices of a
/// convex source triangle before the points constructed inside or on it. An
/// isolated STRICT three-halfspace check proves that schedule; an undecided
/// proof or any negative side simply declines to the complete hull discovery
/// above without changing the operation's aggregate certainty.
fn triangulate_from_enclosing_prefix_triangle(
    kernel: &ExactKernel,
    points: &[Point2],
) -> Result<Option<Vec<Triangle>>> {
    let proof_context =
        crate::context::TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
    let proof_kernel = ExactKernel::new(&proof_context);
    let (seed, first_edge) = match prove_enclosing_prefix_triangle(&proof_kernel, points) {
        Ok(Some(proof)) => proof,
        Ok(None) | Err(Error::PredicateUndecided { .. }) => return Ok(None),
        Err(error) => return Err(error),
    };

    // A planar triangulation of n unique points has at most 2n - 5
    // triangles. Reserve that upper bound once instead of growing the
    // one-triangle seed.
    let mut triangles = Vec::with_capacity(points.len().saturating_mul(2).saturating_sub(5));
    if let Some([from, to, opposite]) = first_edge {
        triangles.extend([[from, 3, opposite], [3, to, opposite]]);
    } else {
        triangles.extend([
            [seed[0], seed[1], 3],
            [seed[1], seed[2], 3],
            [seed[2], seed[0], 3],
        ]);
    }
    for point in 4..points.len() {
        insert_point(kernel, points, &mut triangles, point)?;
    }
    Ok(Some(triangles))
}

fn prove_enclosing_prefix_triangle(
    kernel: &ExactKernel,
    points: &[Point2],
) -> Result<Option<(Triangle, Option<[usize; 3]>)>> {
    let Some(seed) = triangle_if_not_degenerate(kernel, points, [0, 1, 2])? else {
        return Ok(None);
    };
    let mut first_edge = None;
    for (index, point) in points[3..].iter().enumerate() {
        let mut on_edge = None;
        for [from, to, opposite] in [
            [seed[0], seed[1], seed[2]],
            [seed[1], seed[2], seed[0]],
            [seed[2], seed[0], seed[1]],
        ] {
            match predicates::orient2(kernel, &points[from], &points[to], point)? {
                Sign::Negative => return Ok(None),
                Sign::Zero if on_edge.is_some() => return Ok(None),
                Sign::Zero => on_edge = Some((from, to, opposite)),
                Sign::Positive => {}
            }
        }
        if index == 0
            && let Some((from, to, opposite)) = on_edge
        {
            first_edge = Some([from, to, opposite]);
        }
    }

    Ok(Some((seed, first_edge)))
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

/// Insert one indexed point into an existing exact planar triangulation.
pub(crate) fn insert_point(
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::TriangulationContext;
    use crate::types::Real;

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn enclosing_prefix_triangle_is_a_complete_exact_schedule() {
        for points in [
            [p(0, 0), p(6, 0), p(0, 6), p(1, 1), p(3, 0), p(0, 3)],
            [p(0, 0), p(0, 6), p(6, 0), p(3, 0), p(1, 1), p(0, 3)],
        ] {
            for policy in [
                hyperlimit::PredicatePolicy::STRICT,
                hyperlimit::PredicatePolicy::APPROXIMATE_512,
            ] {
                let context = TriangulationContext::new(policy);
                let kernel = ExactKernel::new(&context);
                let triangles = triangulate_from_enclosing_prefix_triangle(&kernel, &points)
                    .unwrap()
                    .expect("the prefix triangle exactly encloses every other point");

                assert!(
                    crate::cdt_validate::triangulates_convex_hull(&kernel, &points, &triangles)
                        .unwrap()
                );
                assert_eq!(
                    kernel.finish(()).certainty,
                    crate::TriangulationCertainty::Certified
                );
            }
        }
    }

    #[test]
    fn nonenclosing_prefix_declines_to_general_hull_discovery() {
        let points = [p(0, 0), p(1, 0), p(0, 1), p(2, 2)];
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let kernel = ExactKernel::new(&context);

        assert_eq!(
            triangulate_from_enclosing_prefix_triangle(&kernel, &points).unwrap(),
            None
        );
        assert_eq!(triangulate_point_set(&kernel, &points).unwrap().len(), 2);
    }

    #[test]
    fn degenerate_prefix_declines_to_general_hull_discovery() {
        let points = [p(0, 0), p(1, 0), p(2, 0), p(0, 1)];
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let kernel = ExactKernel::new(&context);

        assert_eq!(
            triangulate_from_enclosing_prefix_triangle(&kernel, &points).unwrap(),
            None
        );
        assert_eq!(triangulate_point_set(&kernel, &points).unwrap().len(), 2);
    }
}
