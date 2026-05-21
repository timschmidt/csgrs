//! Constraint recovery and local legalization for the CDT port.
//!
//! The public [`crate::cdt`] module owns API shape and result records; this
//! module keeps the incremental segment-insertion machinery local. The
//! implementation starts from the exact Delaunay triangulation, planarizes
//! crossing constraints into exact Steiner vertices, recovers each protected
//! subsegment by flipping crossed unconstrained edges, then re-legalizes only
//! unconstrained edges. This follows the incremental CDT construction family
//! described by Shewchuk and Brown, where segment insertion deletes or mutates
//! crossed structure, and the Constrained Delaunay Lemma of Lee and Lin reduces
//! correctness to local Delaunay checks on non-segment edges. Exact predicate
//! ownership stays in the kernel/predicate layer in the sense of Yap's
//! exact-geometric-computation architecture.

use crate::error::{Error, Result};
use crate::kernel::Kernel;
use crate::predicates::{self, SegmentIntersection};
use crate::types::Sign;
use crate::types::{Constraint, Point2, Real, Triangle};
use std::cmp::Ordering;

/// Exact planar straight-line graph produced from caller constraints.
pub(crate) struct PlanarConstraints {
    /// Original points plus exact intersection vertices inserted by planarizing
    /// crossing constraints.
    pub(crate) points: Vec<Point2>,
    /// Constraint subsegments over `points`.
    pub(crate) constraints: Vec<Constraint>,
}

/// Insert all constraints into an existing triangulation.
///
/// `triangles` must triangulate the convex hull of `points`. The function keeps
/// every requested segment as a triangulation edge and restores local Delaunay
/// legality for unconstrained interior edges where exact predicates can decide
/// the required flips.
pub(crate) fn insert_constraints<K>(
    points: &[Point2],
    mut triangles: Vec<Triangle>,
    constraints: &[Constraint],
) -> Result<Vec<Triangle>>
where
    K: Kernel,
{
    // Structural-dispatch note: constraint recovery processes the planarized
    // subsegments in caller-derived order. The retained PSLG facts already
    // keep intersection vertices and split subsegments explicit; richer
    // prepared objects can add axis-alignment, endpoint order, and
    // exact-rational denominator classes without changing the topology
    // contract. This is Yap's "beyond BigNumber" object layer: preserve useful
    // structure beside exact arithmetic rather than leaking scalar internals.
    let mut constrained_edges = Vec::new();
    for &constraint in constraints {
        let edge = EdgeKey::new(constraint.from, constraint.to);
        if triangulation_has_edge(&triangles, edge) {
            push_unique_edge(&mut constrained_edges, edge);
            continue;
        }

        recover_constraint::<K>(points, &mut triangles, constraint, &constrained_edges)?;
        push_unique_edge(&mut constrained_edges, edge);
        legalize_unconstrained_edges::<K>(points, &mut triangles, &constrained_edges)?;
    }

    legalize_unconstrained_edges::<K>(points, &mut triangles, &constrained_edges)?;
    Ok(triangles)
}

/// Planarize caller constraints into exact PSLG subsegments.
///
/// Segment insertion algorithms normally work on a planar straight-line graph.
/// When constraints properly cross or pass through existing vertices, the
/// intersection becomes a graph vertex and each original segment is normalized
/// into subsegments. This is the PSLG view used by Lee and Lin's generalized
/// Delaunay triangulation and by Shewchuk and Brown's incremental
/// segment-insertion formulation.
pub(crate) fn planarize_constraints<K>(
    points: &[Point2],
    constraints: &[Constraint],
) -> Result<PlanarConstraints>
where
    K: Kernel,
{
    let mut planar_points = points.to_vec();

    // Structural-dispatch note: this exact O(m^2) pair scan is the conservative
    // baseline. If constraints carry exact bounding boxes plus grid/dyadic
    // scale facts, a sweep or spatial index can reject most pairs while still
    // routing every surviving intersection test through exact predicates.
    for first in 0..constraints.len() {
        for second in first + 1..constraints.len() {
            let a = constraints[first];
            let b = constraints[second];
            if constraints_share_endpoint(a, b) {
                continue;
            }

            if predicates::segment_intersection::<K>(
                &planar_points[a.from],
                &planar_points[a.to],
                &planar_points[b.from],
                &planar_points[b.to],
            )? == SegmentIntersection::Proper
            {
                let point = segment_intersection_point::<K>(&planar_points, a, b)?;
                push_unique_point(&mut planar_points, point);
            }
        }
    }

    let mut split = Vec::new();
    for constraint in constraints {
        let mut on_segment = Vec::new();
        for point_index in 0..planar_points.len() {
            if predicates::point_on_segment::<K>(
                &planar_points[constraint.from],
                &planar_points[constraint.to],
                &planar_points[point_index],
            )? {
                on_segment.push(point_index);
            }
        }

        sort_indices_on_segment::<K>(&planar_points, constraint, &mut on_segment)?;
        for pair in on_segment.windows(2) {
            push_unique_constraint(&mut split, Constraint::new(pair[0], pair[1]));
        }
    }

    Ok(PlanarConstraints {
        points: planar_points,
        constraints: split,
    })
}

fn segment_intersection_point<K>(
    points: &[Point2],
    first: Constraint,
    second: Constraint,
) -> Result<Point2>
where
    K: Kernel,
{
    let a = &points[first.from];
    let b = &points[first.to];
    let c = &points[second.from];
    let d = &points[second.to];

    let ab_x = K::sub(&b.x, &a.x);
    let ab_y = K::sub(&b.y, &a.y);
    let cd_x = K::sub(&d.x, &c.x);
    let cd_y = K::sub(&d.y, &c.y);
    let ca_x = K::sub(&c.x, &a.x);
    let ca_y = K::sub(&c.y, &a.y);

    let denominator = cross::<K>(&ab_x, &ab_y, &cd_x, &cd_y);
    let numerator = cross::<K>(&ca_x, &ca_y, &cd_x, &cd_y);
    let t = K::div(&numerator, &denominator)?;

    Ok(Point2::new(
        K::add(&a.x, &K::mul(&t, &ab_x)),
        K::add(&a.y, &K::mul(&t, &ab_y)),
    ))
}

fn cross<K>(left_x: &Real, left_y: &Real, right_x: &Real, right_y: &Real) -> Real
where
    K: Kernel,
{
    K::sub(&K::mul(left_x, right_y), &K::mul(left_y, right_x))
}

fn push_unique_point(points: &mut Vec<Point2>, point: Point2) -> usize {
    if let Some(index) = points.iter().position(|candidate| candidate == &point) {
        index
    } else {
        let index = points.len();
        points.push(point);
        index
    }
}

fn constraints_share_endpoint(first: Constraint, second: Constraint) -> bool {
    first.from == second.from
        || first.from == second.to
        || first.to == second.from
        || first.to == second.to
}

fn sort_indices_on_segment<K>(
    points: &[Point2],
    constraint: &Constraint,
    indices: &mut [usize],
) -> Result<()>
where
    K: Kernel,
{
    let use_x = K::cmp(&points[constraint.from].x, &points[constraint.to].x)? != Ordering::Equal;

    for index in 1..indices.len() {
        let mut cursor = index;
        while cursor > 0
            && compare_segment_indices::<K>(points, indices[cursor], indices[cursor - 1], use_x)?
                == Ordering::Less
        {
            indices.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }

    Ok(())
}

fn compare_segment_indices<K>(
    points: &[Point2],
    left: usize,
    right: usize,
    use_x: bool,
) -> Result<Ordering>
where
    K: Kernel,
{
    if use_x {
        K::cmp(&points[left].x, &points[right].x)
    } else {
        K::cmp(&points[left].y, &points[right].y)
    }
}

fn push_unique_constraint(constraints: &mut Vec<Constraint>, constraint: Constraint) {
    let edge = EdgeKey::new(constraint.from, constraint.to);
    if !constraints
        .iter()
        .any(|existing| EdgeKey::new(existing.from, existing.to) == edge)
    {
        constraints.push(constraint);
    }
}

fn recover_constraint<K>(
    points: &[Point2],
    triangles: &mut [Triangle],
    constraint: Constraint,
    constrained_edges: &[EdgeKey],
) -> Result<()>
where
    K: Kernel,
{
    let target = EdgeKey::new(constraint.from, constraint.to);
    let max_flips = flip_budget(points.len(), triangles.len());

    for _ in 0..max_flips {
        if triangulation_has_edge(triangles, target) {
            return Ok(());
        }

        let Some(crossing_edge) =
            first_edge_crossing_constraint::<K>(points, triangles, constraint, constrained_edges)?
        else {
            return Err(Error::UnsupportedFeature {
                feature: "constraint recovery without a flippable crossing edge",
            });
        };

        let Some([first, second]) = two_adjacent_triangles(triangles, crossing_edge)? else {
            return Err(Error::UnsupportedFeature {
                feature: "constraint recovery reached a boundary edge",
            });
        };
        let new_edge = EdgeKey::new(first.opposite, second.opposite);
        if !flip_preserves_constraints::<K>(points, new_edge, constrained_edges)? {
            return Err(Error::UnsupportedFeature {
                feature: "constraint recovery would cross a previous constraint",
            });
        }

        if !flip_edge::<K>(points, triangles, crossing_edge)? {
            return Err(Error::UnsupportedFeature {
                feature: "constraint recovery across a non-convex edge cavity",
            });
        }
    }

    Err(Error::UnsupportedFeature {
        feature: "constraint edge recovery did not converge",
    })
}

fn first_edge_crossing_constraint<K>(
    points: &[Point2],
    triangles: &[Triangle],
    constraint: Constraint,
    constrained_edges: &[EdgeKey],
) -> Result<Option<EdgeKey>>
where
    K: Kernel,
{
    for edge in unique_edges(triangles) {
        if edge.contains(constraint.from) || edge.contains(constraint.to) {
            continue;
        }
        let intersection = predicates::segment_intersection::<K>(
            &points[constraint.from],
            &points[constraint.to],
            &points[edge.from],
            &points[edge.to],
        )?;
        if intersection == SegmentIntersection::Proper {
            if constrained_edges.contains(&edge) {
                return Err(Error::InvalidInput {
                    reason: "constraint crosses an existing constrained edge",
                });
            }
            return Ok(Some(edge));
        }
    }

    Ok(None)
}

fn legalize_unconstrained_edges<K>(
    points: &[Point2],
    triangles: &mut [Triangle],
    constrained_edges: &[EdgeKey],
) -> Result<()>
where
    K: Kernel,
{
    let max_flips = flip_budget(points.len(), triangles.len()) * 4;

    for _ in 0..max_flips {
        let mut flipped = false;
        for edge in unique_edges(triangles) {
            if constrained_edges.contains(&edge) {
                continue;
            }

            let Some([first, second]) = two_adjacent_triangles(triangles, edge)? else {
                continue;
            };

            let new_edge = EdgeKey::new(first.opposite, second.opposite);
            if !edge_is_illegal::<K>(points, edge, first.opposite, second.opposite)? {
                continue;
            }
            if !flip_preserves_constraints::<K>(points, new_edge, constrained_edges)? {
                continue;
            }
            if flip_edge::<K>(points, triangles, edge)? {
                flipped = true;
                break;
            }
        }

        if !flipped {
            return Ok(());
        }
    }

    Err(Error::UnsupportedFeature {
        feature: "constrained Delaunay legalization did not converge",
    })
}

fn edge_is_illegal<K>(
    points: &[Point2],
    edge: EdgeKey,
    first_opposite: usize,
    second_opposite: usize,
) -> Result<bool>
where
    K: Kernel,
{
    if !edge_is_flippable::<K>(points, edge, first_opposite, second_opposite)? {
        return Ok(false);
    }

    let orientation = predicates::orient2d::<K>(
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    if orientation == Sign::Zero {
        return Ok(false);
    }

    let incircle = K::incircle2d(
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
        &points[second_opposite],
    )?;

    Ok(matches!(
        (orientation, incircle),
        (Sign::Positive, Sign::Positive) | (Sign::Negative, Sign::Negative)
    ))
}

fn flip_edge<K>(points: &[Point2], triangles: &mut [Triangle], edge: EdgeKey) -> Result<bool>
where
    K: Kernel,
{
    let Some([first, second]) = two_adjacent_triangles(triangles, edge)? else {
        return Ok(false);
    };

    if !edge_is_flippable::<K>(points, edge, first.opposite, second.opposite)? {
        return Ok(false);
    }

    let first_new = make_oriented::<K>(points, [first.opposite, second.opposite, edge.from])?;
    let second_new = make_oriented::<K>(points, [second.opposite, first.opposite, edge.to])?;
    triangles[first.triangle] = first_new;
    triangles[second.triangle] = second_new;
    Ok(true)
}

fn edge_is_flippable<K>(
    points: &[Point2],
    edge: EdgeKey,
    first_opposite: usize,
    second_opposite: usize,
) -> Result<bool>
where
    K: Kernel,
{
    if first_opposite == second_opposite
        || edge.contains(first_opposite)
        || edge.contains(second_opposite)
    {
        return Ok(false);
    }

    let first_side = predicates::orient2d::<K>(
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    let second_side = predicates::orient2d::<K>(
        &points[edge.from],
        &points[edge.to],
        &points[second_opposite],
    )?;
    let opposite_edge_side = predicates::orient2d::<K>(
        &points[first_opposite],
        &points[second_opposite],
        &points[edge.from],
    )?;
    let opposite_other_side = predicates::orient2d::<K>(
        &points[first_opposite],
        &points[second_opposite],
        &points[edge.to],
    )?;

    Ok(signs_strictly_differ(first_side, second_side)
        && signs_strictly_differ(opposite_edge_side, opposite_other_side))
}

fn flip_preserves_constraints<K>(
    points: &[Point2],
    new_edge: EdgeKey,
    constrained_edges: &[EdgeKey],
) -> Result<bool>
where
    K: Kernel,
{
    for point_index in 0..points.len() {
        if new_edge.contains(point_index) {
            continue;
        }
        if predicates::point_on_segment::<K>(
            &points[new_edge.from],
            &points[new_edge.to],
            &points[point_index],
        )? {
            return Ok(false);
        }
    }

    for &constraint in constrained_edges {
        if new_edge == constraint {
            continue;
        }

        let intersection = predicates::segment_intersection::<K>(
            &points[new_edge.from],
            &points[new_edge.to],
            &points[constraint.from],
            &points[constraint.to],
        )?;
        match intersection {
            SegmentIntersection::Disjoint => {}
            SegmentIntersection::EndpointTouch if new_edge.shares_endpoint(constraint) => {}
            SegmentIntersection::EndpointTouch
            | SegmentIntersection::Proper
            | SegmentIntersection::CollinearOverlap => return Ok(false),
        }
    }

    Ok(true)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AdjacentTriangle {
    triangle: usize,
    opposite: usize,
}

fn two_adjacent_triangles(
    triangles: &[Triangle],
    edge: EdgeKey,
) -> Result<Option<[AdjacentTriangle; 2]>> {
    let mut adjacent = Vec::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        if triangle_contains_edge(*triangle, edge) {
            let Some(opposite) = triangle
                .iter()
                .copied()
                .find(|&index| !edge.contains(index))
            else {
                return Err(Error::InvalidInput {
                    reason: "triangle edge has no opposite vertex",
                });
            };
            adjacent.push(AdjacentTriangle {
                triangle: triangle_index,
                opposite,
            });
        }
    }

    match adjacent.len() {
        0 | 1 => Ok(None),
        2 => Ok(Some([adjacent[0], adjacent[1]])),
        _ => Err(Error::InvalidInput {
            reason: "triangulation edge has more than two adjacent triangles",
        }),
    }
}

fn make_oriented<K>(points: &[Point2], mut triangle: Triangle) -> Result<Triangle>
where
    K: Kernel,
{
    let sign = predicates::orient2d::<K>(
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
    )?;
    match sign {
        Sign::Positive => Ok(triangle),
        Sign::Negative => {
            triangle.swap(1, 2);
            Ok(triangle)
        }
        Sign::Zero => Err(Error::InvalidInput {
            reason: "degenerate triangle",
        }),
    }
}

fn triangulation_has_edge(triangles: &[Triangle], edge: EdgeKey) -> bool {
    triangles
        .iter()
        .any(|triangle| triangle_contains_edge(*triangle, edge))
}

fn triangle_contains_edge(triangle: Triangle, edge: EdgeKey) -> bool {
    triangle.contains(&edge.from) && triangle.contains(&edge.to)
}

fn unique_edges(triangles: &[Triangle]) -> Vec<EdgeKey> {
    let mut edges = Vec::new();
    for triangle in triangles {
        push_unique_edge(&mut edges, EdgeKey::new(triangle[0], triangle[1]));
        push_unique_edge(&mut edges, EdgeKey::new(triangle[1], triangle[2]));
        push_unique_edge(&mut edges, EdgeKey::new(triangle[2], triangle[0]));
    }
    edges
}

fn push_unique_edge(edges: &mut Vec<EdgeKey>, edge: EdgeKey) {
    if !edges.contains(&edge) {
        edges.push(edge);
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct EdgeKey {
    from: usize,
    to: usize,
}

impl EdgeKey {
    fn new(first: usize, second: usize) -> Self {
        if first <= second {
            Self {
                from: first,
                to: second,
            }
        } else {
            Self {
                from: second,
                to: first,
            }
        }
    }

    fn contains(self, index: usize) -> bool {
        self.from == index || self.to == index
    }

    fn shares_endpoint(self, other: Self) -> bool {
        self.contains(other.from) || self.contains(other.to)
    }
}

fn signs_strictly_differ(first: Sign, second: Sign) -> bool {
    matches!(
        (first, second),
        (Sign::Negative, Sign::Positive) | (Sign::Positive, Sign::Negative)
    )
}

fn flip_budget(point_count: usize, triangle_count: usize) -> usize {
    point_count
        .saturating_add(triangle_count)
        .saturating_add(1)
        .pow(2)
}
