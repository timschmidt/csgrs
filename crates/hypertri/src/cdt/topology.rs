//! Orientation-only point-set triangulation for constrained topology.

use super::{
    LOCATED_CAVITY_THRESHOLD, RETAINED_ADJACENCY_THRESHOLD, locate_triangle, make_oriented,
    spatial_point_cmp, triangle_if_not_degenerate, triangle_neighbors,
};
use crate::cdt_insert::TriangleTopology;
use crate::error::{Error, Result};
use crate::kernel::ExactKernel;
use crate::predicates;
use crate::types::{Point2, Sign, Triangle, TriangleLocation};

pub(super) struct PointTriangulation {
    pub(super) triangles: Vec<Triangle>,
    pub(super) topology: Option<Box<TriangleTopology>>,
}

impl PointTriangulation {
    fn without_topology(triangles: Vec<Triangle>) -> Self {
        Self {
            triangles,
            topology: None,
        }
    }
}

pub(super) fn triangulate_point_set(
    kernel: &ExactKernel,
    points: &[Point2],
) -> Result<PointTriangulation> {
    match points.len() {
        0..=2 => return Ok(PointTriangulation::without_topology(Vec::new())),
        3 => {
            return triangle_if_not_degenerate(kernel, points, [0, 1, 2])?
                .map(|triangle| vec![triangle])
                .map(PointTriangulation::without_topology)
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
    let mut topology = None;
    for point in 0..points.len() {
        if hull.binary_search(&point).is_err() {
            insert_point(kernel, points, &mut triangles, point, &mut topology)?;
        }
    }

    if !crate::cdt_validate::triangulates_convex_hull(kernel, points, &triangles)? {
        return Err(Error::InvalidInput {
            reason: "topological point insertion did not cover the convex hull",
        });
    }
    Ok(PointTriangulation {
        triangles,
        topology,
    })
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
) -> Result<Option<PointTriangulation>> {
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
    let mut topology = None;
    for point in 4..points.len() {
        insert_point(kernel, points, &mut triangles, point, &mut topology)?;
    }
    Ok(Some(PointTriangulation {
        triangles,
        topology,
    }))
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

/// Insert one indexed point while retaining exact triangle topology once the
/// located schedule has paid to construct it.
pub(crate) fn insert_point(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    point: usize,
    topology: &mut Option<Box<TriangleTopology>>,
) -> Result<()> {
    let mut located = None;
    if triangles.len() >= LOCATED_CAVITY_THRESHOLD {
        let mut temporary_neighbors = None;
        if topology.is_none() && triangles.len() >= RETAINED_ADJACENCY_THRESHOLD {
            *topology = Some(Box::new(TriangleTopology::new(triangles, points.len())?));
        } else if topology.is_none() {
            temporary_neighbors = Some(triangle_neighbors(triangles)?);
        }
        let retained = topology
            .as_ref()
            .map(|topology| topology.neighbors())
            .or(temporary_neighbors.as_deref())
            .ok_or(Error::InvalidInput {
                reason: "located point insertion did not construct adjacency",
            })?;
        if let Some(triangle) = locate_triangle(
            kernel,
            points,
            triangles,
            retained,
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
            let replacement = [
                make_oriented(kernel, points, [a, b, point])?,
                make_oriented(kernel, points, [b, c, point])?,
                make_oriented(kernel, points, [c, a, point])?,
            ];
            if let Some(topology) = topology {
                topology.replace_point_region(triangles, &[triangle_index], &replacement, None)?;
            } else {
                triangles[triangle_index] = replacement[0];
                triangles.extend_from_slice(&replacement[1..]);
            }
            Ok(())
        }
        TriangleLocation::OnEdge => {
            split_edge(kernel, points, triangles, triangle_index, point, topology)
        }
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
    topology: &mut Option<Box<TriangleTopology>>,
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

    let mut incident = [triangle_index, usize::MAX];
    let mut incident_len = 1;
    if let Some(retained) = topology.as_ref() {
        if let Some(neighbor) =
            retained.neighbor_across_vertices(triangles, triangle_index, edge.0, edge.1)?
        {
            incident[1] = neighbor;
            incident_len = 2;
            incident[..incident_len].sort_unstable();
        }
    } else {
        incident_len = 0;
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
    }
    if incident_len == 0 || !incident[..incident_len].contains(&triangle_index) {
        return Err(Error::InvalidInput {
            reason: "triangulation edge has invalid incidence",
        });
    }

    let mut first = [[0; 3]; 2];
    let mut second = [[0; 3]; 2];
    for (position, &index) in incident[..incident_len].iter().enumerate() {
        let source = triangles[index];
        let opposite = source
            .into_iter()
            .find(|vertex| *vertex != edge.0 && *vertex != edge.1)
            .ok_or(Error::InvalidInput {
                reason: "triangulation edge has no opposite vertex",
            })?;
        first[position] = make_oriented(kernel, points, [edge.0, point, opposite])?;
        second[position] = make_oriented(kernel, points, [point, edge.1, opposite])?;
    }

    if let Some(retained) = topology {
        let mut replacement = [[0; 3]; 4];
        replacement[..incident_len].copy_from_slice(&first[..incident_len]);
        replacement[incident_len..incident_len * 2].copy_from_slice(&second[..incident_len]);
        retained.replace_point_region(
            triangles,
            &incident[..incident_len],
            &replacement[..incident_len * 2],
            Some((edge.0, edge.1, point)),
        )?;
    } else {
        for (position, &index) in incident[..incident_len].iter().enumerate() {
            triangles[index] = first[position];
        }
        triangles.extend_from_slice(&second[..incident_len]);
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

    fn insert_with_retained_adjacency(
        kernel: &ExactKernel,
        points: &[Point2],
        mut triangles: Vec<Triangle>,
        point: usize,
    ) -> Vec<Triangle> {
        let mut topology = Some(Box::new(
            TriangleTopology::new(&triangles, points.len()).unwrap(),
        ));
        insert_point(kernel, points, &mut triangles, point, &mut topology).unwrap();
        assert_eq!(
            topology
                .expect("retained topology remains initialized")
                .neighbors(),
            triangle_neighbors(&triangles).unwrap(),
        );
        triangles
    }

    #[test]
    fn retained_adjacency_matches_complete_rebuild_for_every_point_split() {
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let kernel = ExactKernel::new(&context);

            let interior = [p(0, 0), p(8, 0), p(0, 8), p(1, 1)];
            let interior_triangles =
                insert_with_retained_adjacency(&kernel, &interior, vec![[0, 1, 2]], 3);
            assert_eq!(interior_triangles.len(), 3);

            let boundary = [p(0, 0), p(8, 0), p(0, 8), p(4, 0)];
            let boundary_triangles =
                insert_with_retained_adjacency(&kernel, &boundary, vec![[0, 1, 2]], 3);
            assert_eq!(boundary_triangles.len(), 2);

            let shared_edge = [p(0, 0), p(8, 0), p(8, 8), p(0, 8), p(4, 4)];
            let shared_triangles = insert_with_retained_adjacency(
                &kernel,
                &shared_edge,
                vec![[0, 1, 2], [0, 2, 3]],
                4,
            );
            assert_eq!(shared_triangles.len(), 4);
            assert!(
                crate::cdt_validate::triangulates_convex_hull(
                    &kernel,
                    &shared_edge,
                    &shared_triangles,
                )
                .unwrap()
            );
            assert_eq!(
                kernel.finish(()).certainty,
                crate::TriangulationCertainty::Certified,
            );
        }
    }

    #[test]
    fn retained_point_set_topology_matches_complete_adjacency() {
        let points = vec![
            p(0, 0),
            p(64, 0),
            p(0, 64),
            p(1, 1),
            p(8, 0),
            p(0, 8),
            p(8, 8),
            p(16, 4),
            p(4, 16),
            p(16, 16),
            p(24, 8),
            p(8, 24),
            p(24, 24),
            p(32, 4),
            p(4, 32),
            p(32, 16),
            p(16, 32),
            p(32, 24),
            p(24, 32),
            p(40, 8),
            p(8, 40),
            p(40, 16),
            p(16, 40),
            p(40, 20),
            p(20, 40),
        ];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let kernel = ExactKernel::new(&context);
            let triangulation = triangulate_point_set(&kernel, &points).unwrap();
            let topology = triangulation
                .topology
                .expect("the nontrivial point set retains its checked topology");

            assert_eq!(
                topology.neighbors(),
                triangle_neighbors(&triangulation.triangles).unwrap(),
            );
            assert!(
                crate::cdt_validate::triangulates_convex_hull(
                    &kernel,
                    &points,
                    &triangulation.triangles,
                )
                .unwrap(),
            );
            assert_eq!(
                kernel.finish(()).certainty,
                crate::TriangulationCertainty::Certified,
            );
        }
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
                let triangulation = triangulate_from_enclosing_prefix_triangle(&kernel, &points)
                    .unwrap()
                    .expect("the prefix triangle exactly encloses every other point");

                assert!(
                    crate::cdt_validate::triangulates_convex_hull(
                        &kernel,
                        &points,
                        &triangulation.triangles,
                    )
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
            triangulate_from_enclosing_prefix_triangle(&kernel, &points)
                .unwrap()
                .map(|triangulation| triangulation.triangles),
            None,
        );
        assert_eq!(
            triangulate_point_set(&kernel, &points)
                .unwrap()
                .triangles
                .len(),
            2,
        );
    }

    #[test]
    fn degenerate_prefix_declines_to_general_hull_discovery() {
        let points = [p(0, 0), p(1, 0), p(2, 0), p(0, 1)];
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let kernel = ExactKernel::new(&context);

        assert_eq!(
            triangulate_from_enclosing_prefix_triangle(&kernel, &points)
                .unwrap()
                .map(|triangulation| triangulation.triangles),
            None,
        );
        assert_eq!(
            triangulate_point_set(&kernel, &points)
                .unwrap()
                .triangles
                .len(),
            2,
        );
    }
}
