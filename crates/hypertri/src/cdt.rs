//! Constrained Delaunay triangulation API foundation.
//!
//! The implementation handles exact incremental Delaunay point sets, planarizes
//! crossing constraints with exact Steiner points, recovers protected edges,
//! and re-legalizes unprotected edges. The production path owns its topology
//! implementation and has no external triangulation dependency.

use crate::cdt_constraints;
use crate::error::{Error, Result};
use crate::kernel::{ExactKernel, Kernel};
use crate::predicates;
use crate::types::{Constraint, ExactPoint, Point2, Real, Triangle};
use crate::types::{Sign, TriangleLocation};

// Sorting a handful of integer edge handles costs more than the exhaustive
// exact test, so retain the simple path until a nontrivial mesh exists.
const LOCATED_CAVITY_THRESHOLD: usize = 4;

/// Triangulation result for an unconstrained 2D point set.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DelaunayTriangulation {
    points: Vec<ExactPoint>,
    triangles: Vec<Triangle>,
}

impl DelaunayTriangulation {
    /// Construct a triangulation record from points and triangle indices.
    pub fn from_parts(points: Vec<ExactPoint>, triangles: Vec<Triangle>) -> Self {
        Self { points, triangles }
    }

    /// Input points stored by the triangulation.
    pub fn points(&self) -> &[ExactPoint] {
        &self.points
    }

    /// Output triangles as input point indices.
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Consume the triangulation into its point and triangle buffers.
    pub fn into_parts(self) -> (Vec<ExactPoint>, Vec<Triangle>) {
        (self.points, self.triangles)
    }

    /// Validate exact Delaunay topology and local empty-circle legality.
    ///
    /// This is intentionally `O(n^2)` over the produced topology. It is meant
    /// for tests, debug assertions, and downstream callers that want to audit a
    /// triangulation boundary before consuming it.
    pub fn validate(&self) -> Result<()> {
        crate::cdt_validate::validate_delaunay(&self.points, &self.triangles)
    }
}

/// Triangulation result for a 2D point set with constrained edges.
///
/// Constraint insertion can append exact Steiner vertices where caller
/// constraints properly intersect. [`Self::constraints`] returns the original
/// caller constraints, while [`Self::triangles`] indexes into [`Self::points`],
/// which may be longer than the caller's input point buffer.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct ConstrainedDelaunayTriangulation {
    points: Vec<ExactPoint>,
    constraints: Vec<Constraint>,
    constraint_edges: Vec<Constraint>,
    triangles: Vec<Triangle>,
}

impl ConstrainedDelaunayTriangulation {
    /// Construct a constrained triangulation record from raw parts.
    pub fn from_parts(
        points: Vec<ExactPoint>,
        constraints: Vec<Constraint>,
        triangles: Vec<Triangle>,
    ) -> Self {
        Self::from_parts_with_constraint_edges(points, constraints.clone(), constraints, triangles)
    }

    /// Construct a constrained triangulation record with explicit protected
    /// subsegments.
    ///
    /// `constraints` are the caller-visible constraints. `constraint_edges` are
    /// the planarized PSLG edges actually present in the triangulation and may
    /// reference Steiner vertices appended to [`Self::points`].
    pub fn from_parts_with_constraint_edges(
        points: Vec<ExactPoint>,
        constraints: Vec<Constraint>,
        constraint_edges: Vec<Constraint>,
        triangles: Vec<Triangle>,
    ) -> Self {
        Self {
            points,
            constraints,
            constraint_edges,
            triangles,
        }
    }

    /// Points stored by the triangulation.
    ///
    /// For intersecting constraints this can include exact Steiner vertices
    /// appended during PSLG planarization.
    pub fn points(&self) -> &[ExactPoint] {
        &self.points
    }

    /// Caller constraints stored by the triangulation.
    pub fn constraints(&self) -> &[Constraint] {
        &self.constraints
    }

    /// Protected constraint subsegments present as triangulation edges.
    ///
    /// These edges index into [`Self::points`]. They are equal to
    /// [`Self::constraints`] for simple non-intersecting inputs, but differ when
    /// constraints are split at existing vertices or exact intersection
    /// vertices.
    pub fn constraint_edges(&self) -> &[Constraint] {
        &self.constraint_edges
    }

    /// Output triangles as input point indices.
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Consume the triangulation into its buffers.
    pub fn into_parts(self) -> (Vec<ExactPoint>, Vec<Constraint>, Vec<Triangle>) {
        (self.points, self.constraints, self.triangles)
    }

    /// Consume the triangulation into all raw buffers, including protected
    /// planarized constraint edges.
    pub fn into_parts_with_constraint_edges(
        self,
    ) -> (
        Vec<ExactPoint>,
        Vec<Constraint>,
        Vec<Constraint>,
        Vec<Triangle>,
    ) {
        (
            self.points,
            self.constraints,
            self.constraint_edges,
            self.triangles,
        )
    }

    /// Validate exact constrained triangulation topology.
    ///
    /// The check verifies triangle index/orientation validity and that every
    /// planarized [`Self::constraint_edges`] entry is present as a triangulation
    /// edge. It does not require Delaunay legality, so it also applies to the
    /// closed-ring earcut fallback.
    pub fn validate(&self) -> Result<()> {
        crate::cdt_validate::validate_constrained_topology(
            &self.points,
            &self.constraint_edges,
            &self.triangles,
        )
    }

    /// Validate the exact local Delaunay condition on unconstrained interior
    /// edges.
    ///
    /// This check implements the Constrained Delaunay Lemma criterion: after
    /// protected PSLG edges are excluded, each interior edge must satisfy the
    /// empty-circle legality test against the two adjacent triangles.
    pub fn validate_unconstrained_edges_are_delaunay(&self) -> Result<()> {
        crate::cdt_validate::validate_constrained_delaunay(
            &self.points,
            &self.constraint_edges,
            &self.triangles,
        )
    }
}

/// Triangulate an exact point set with Delaunay edge selection.
///
/// The local edge choices use the empty-circumcircle rule from Delaunay
/// triangulation; the in-circle predicate is evaluated exactly through the
/// crate-local kernel.
pub fn delaunay(points: &[ExactPoint]) -> Result<DelaunayTriangulation> {
    validate_unique_points(points)?;
    let triangles = delaunay_triangles::<ExactKernel>(points)?;
    // Every construction branch admits only nondegenerate, positively
    // oriented triangles and makes each Delaunay choice with the exact
    // in-circle predicate. Re-running the public validator here would repeat
    // those predicates over the completed mesh; callers can still invoke
    // `validate` explicitly on records assembled or deserialized elsewhere.
    Ok(DelaunayTriangulation::from_parts(
        points.to_vec(),
        triangles,
    ))
}

/// Triangulate an exact point set using a deterministic BRIO-style batch order.
///
/// The input point buffer and every triangle index still refer to the caller's
/// original order. Only the internal Bowyer--Watson insertion schedule changes:
/// a deterministic randomized hierarchy supplies successively larger rounds,
/// and a median spatial traversal improves locality within each round. This is
/// useful for large batches whose caller order is unrelated to geometry.
///
/// For cocircular point sets, Delaunay triangulations are not unique, so this
/// function can choose different valid diagonals than [`delaunay`]. Callers
/// that require the historical insertion-order tie topology should use
/// [`delaunay`].
pub fn delaunay_spatial(points: &[ExactPoint]) -> Result<DelaunayTriangulation> {
    validate_unique_points(points)?;
    let triangles = delaunay_triangles_spatial::<ExactKernel>(points)?;
    Ok(DelaunayTriangulation::from_parts(
        points.to_vec(),
        triangles,
    ))
}

/// Triangulate exact points with constraints.
///
/// The constrained path appends exact intersection vertices for proper
/// constraint crossings, normalizes constraints into internal PSLG
/// subsegments, and recovers those subsegments with exact edge flips.
///
/// Closed polygon-with-hole inputs are routed through the boundary-preserving
/// polygon path when `earcut` is enabled. Other planar straight-line graphs are
/// triangulated over their convex hull, with protected subsegments excluded
/// from local Delaunay flips. Local legality requires every unprotected
/// interior edge to satisfy the exact empty-circle test.
pub fn constrained_delaunay(
    points: &[ExactPoint],
    constraints: &[Constraint],
) -> Result<ConstrainedDelaunayTriangulation> {
    validate_constraints(points.len(), constraints)?;
    validate_unique_points(points)?;

    if constraints.is_empty() {
        let triangulation = delaunay(points)?;
        let constrained = ConstrainedDelaunayTriangulation::from_parts_with_constraint_edges(
            triangulation.points,
            Vec::new(),
            Vec::new(),
            triangulation.triangles,
        );
        constrained.validate()?;
        constrained.validate_unconstrained_edges_are_delaunay()?;
        return Ok(constrained);
    }

    let planar = crate::cdt_insert::planarize_constraints(points, constraints)?;
    let points = planar.points;
    let internal_constraints = planar.constraints;
    validate_constraint_geometry(&points, &internal_constraints)?;

    if let Some(polygon) =
        cdt_constraints::polygon_from_closed_constraints(&points, &internal_constraints)?
    {
        // Structural-dispatch note: closed-constraint recognition has already
        // proved ring topology. Preserve facts such as convexity, hole count,
        // monotone chains, and lattice/grid provenance on the polygon record so
        // this branch can select a fan, monotone triangulator, earcut, or full
        // CDT recovery path without rediscovering those properties from exact
        // coordinates.
        #[cfg(feature = "earcut")]
        {
            let (flat_points, hole_indices, source_indices) = polygon.to_flat_polygon(&points);
            let flat = crate::earcut::triangulate(&flat_points, &hole_indices)?;
            let triangles = flat
                .chunks_exact(3)
                .map(|tri| {
                    [
                        source_indices[tri[0]],
                        source_indices[tri[1]],
                        source_indices[tri[2]],
                    ]
                })
                .collect::<Vec<_>>();

            let triangulation = ConstrainedDelaunayTriangulation::from_parts_with_constraint_edges(
                points,
                constraints.to_vec(),
                internal_constraints,
                triangles,
            );
            triangulation.validate()?;
            return Ok(triangulation);
        }

        #[cfg(not(feature = "earcut"))]
        {
            // Without the boundary-preserving polygon module, a closed ring is
            // still a valid PSLG. Fall through to exact edge recovery over the
            // convex hull instead of rejecting the input. This keeps the
            // feature matrix faithful to the exact-computation contract: a
            // missing fast object-level algorithm must not force an approximate
            // or absent topology decision when the exact predicate path can
            // still decide it.
            let _ = polygon;
        }
    }

    if let Some(triangulation) =
        constrained_edges_already_delaunay(&points, &internal_constraints, constraints)?
    {
        return Ok(triangulation);
    }

    let base = delaunay(&points)?;
    let triangles = crate::cdt_insert::insert_constraints::<ExactKernel>(
        &points,
        base.triangles().to_vec(),
        &internal_constraints,
    )?;
    let triangulation = ConstrainedDelaunayTriangulation::from_parts_with_constraint_edges(
        points,
        constraints.to_vec(),
        internal_constraints,
        triangles,
    );
    triangulation.validate()?;
    triangulation.validate_unconstrained_edges_are_delaunay()?;
    Ok(triangulation)
}

fn delaunay_triangles<K>(points: &[Point2]) -> Result<Vec<Triangle>>
where
    K: Kernel,
{
    match points.len() {
        0..=2 => Ok(Vec::new()),
        3 => triangle_if_not_degenerate::<K>(points, [0, 1, 2])
            .map(|triangle| triangle.into_iter().collect()),
        4 => delaunay_quad::<K>(points),
        _ => incremental_delaunay::<K>(points),
    }
}

fn delaunay_triangles_spatial<K>(points: &[Point2]) -> Result<Vec<Triangle>>
where
    K: Kernel,
{
    match points.len() {
        0..=4 => delaunay_triangles::<K>(points),
        _ => {
            let order = brio_insertion_order::<K>(points)?;
            incremental_delaunay_in_order::<K>(points, &order)
        }
    }
}

fn triangle_if_not_degenerate<K>(points: &[Point2], triangle: Triangle) -> Result<Option<Triangle>>
where
    K: Kernel,
{
    let sign = predicates::orient2d::<K>(
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
    )?;
    Ok((sign != Sign::Zero).then_some(oriented_triangle(sign, triangle)))
}

fn delaunay_quad<K>(points: &[Point2]) -> Result<Vec<Triangle>>
where
    K: Kernel,
{
    if let Some(triangles) = triangulate_interior_point::<K>(points)? {
        return Ok(triangles);
    }

    for diagonal in [(0, 1, 2, 3), (0, 2, 1, 3), (0, 3, 1, 2)] {
        let (a, b, c, d) = diagonal;
        let ac = predicates::orient2d::<K>(&points[a], &points[b], &points[c])?;
        let ad = predicates::orient2d::<K>(&points[a], &points[b], &points[d])?;
        if !opposite_sides(ac, ad) {
            continue;
        }

        if diagonal_is_delaunay::<K>(points, a, b, c, d)? {
            return Ok(vec![
                oriented_triangle(ac, [a, b, c]),
                oriented_triangle(ad.reversed(), [a, d, b]),
            ]);
        }
    }

    Err(Error::InvalidInput {
        reason: "four-point set is degenerate",
    })
}

fn triangulate_interior_point<K>(points: &[Point2]) -> Result<Option<Vec<Triangle>>>
where
    K: Kernel,
{
    for interior in 0..4 {
        let outer = (0..4)
            .filter(|&index| index != interior)
            .collect::<Vec<_>>();
        let location = K::classify_point_triangle(
            &points[outer[0]],
            &points[outer[1]],
            &points[outer[2]],
            &points[interior],
        )?;
        match location {
            TriangleLocation::Inside => {
                return Ok(Some(vec![
                    make_oriented::<K>(points, [interior, outer[0], outer[1]])?,
                    make_oriented::<K>(points, [interior, outer[1], outer[2]])?,
                    make_oriented::<K>(points, [interior, outer[2], outer[0]])?,
                ]));
            }
            TriangleLocation::OnEdge => {
                return triangulate_point_on_triangle_edge::<K>(
                    points,
                    interior,
                    [outer[0], outer[1], outer[2]],
                )
                .map(Some);
            }
            TriangleLocation::Degenerate
            | TriangleLocation::Outside
            | TriangleLocation::OnVertex => {}
        }
    }
    Ok(None)
}

fn triangulate_point_on_triangle_edge<K>(
    points: &[Point2],
    point: usize,
    triangle: Triangle,
) -> Result<Vec<Triangle>>
where
    K: Kernel,
{
    for edge_index in 0..3 {
        let a = triangle[edge_index];
        let b = triangle[(edge_index + 1) % 3];
        let c = triangle[(edge_index + 2) % 3];
        if predicates::point_on_segment(&points[a], &points[b], &points[point])? {
            return Ok(vec![
                make_oriented::<K>(points, [a, point, c])?,
                make_oriented::<K>(points, [point, b, c])?,
            ]);
        }
    }

    Err(Error::InvalidInput {
        reason: "edge point was not on a triangle edge",
    })
}

fn diagonal_is_delaunay<K>(
    points: &[Point2],
    a: usize,
    b: usize,
    c: usize,
    d: usize,
) -> Result<bool>
where
    K: Kernel,
{
    let abc = predicates::orient2d::<K>(&points[a], &points[b], &points[c])?;
    let abd = predicates::orient2d::<K>(&points[a], &points[b], &points[d])?;
    let d_in_abc = incircle_inside::<K>(points, [a, b, c], d, abc)?;
    let c_in_abd = incircle_inside::<K>(points, [a, d, b], c, abd.reversed())?;
    Ok(!d_in_abc && !c_in_abd)
}

fn incircle_inside<K>(
    points: &[Point2],
    triangle: Triangle,
    point: usize,
    orientation: Sign,
) -> Result<bool>
where
    K: Kernel,
{
    let sign = K::incircle2d(
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
        &points[point],
    )?;
    Ok(matches!(
        (orientation, sign),
        (Sign::Positive, Sign::Positive) | (Sign::Negative, Sign::Negative)
    ))
}

fn incircle_inside_or_on_positive<K>(
    points: &[Point2],
    triangle: Triangle,
    point: usize,
) -> Result<bool>
where
    K: Kernel,
{
    // Incremental Bowyer-Watson insertion creates the seed and every cavity
    // replacement through `make_oriented`, so active triangles carry a
    // certified positive-orientation invariant. Reuse that object fact rather
    // than evaluating the orientation determinant again for every candidate
    // point/circumcircle pair.
    let sign = K::incircle2d(
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
        &points[point],
    )?;
    Ok(matches!(sign, Sign::Positive | Sign::Zero))
}

fn incremental_delaunay<K>(points: &[Point2]) -> Result<Vec<Triangle>>
where
    K: Kernel,
{
    incremental_delaunay_in_order::<K>(points, &(0..points.len()).collect::<Vec<_>>())
}

fn incremental_delaunay_in_order<K>(
    points: &[Point2],
    insertion_order: &[usize],
) -> Result<Vec<Triangle>>
where
    K: Kernel,
{
    let mut work_points = points.to_vec();
    let first_super = work_points.len();
    work_points.extend(super_triangle::<K>(points)?);

    // This is the Bowyer-Watson empty-circumcircle update: remove every
    // triangle whose circumcircle contains the inserted point, then stitch the
    // boundary cavity back to the point. The empty-circle test is evaluated by
    // the exact predicate kernel.
    let mut triangles = vec![make_oriented::<K>(
        &work_points,
        [first_super, first_super + 1, first_super + 2],
    )?];

    for &point in insertion_order {
        let mut bad = vec![false; triangles.len()];
        if triangles.len() >= LOCATED_CAVITY_THRESHOLD {
            let neighbors = triangle_neighbors(&triangles);
            if let Some(seed) = locate_triangle::<K>(
                &work_points,
                &triangles,
                &neighbors,
                point,
                triangles.len().saturating_sub(1),
            )? {
                let mut visited = vec![false; triangles.len()];
                let mut pending = vec![seed];
                while let Some(triangle_index) = pending.pop() {
                    if visited[triangle_index] {
                        continue;
                    }
                    visited[triangle_index] = true;
                    if incircle_inside_or_on_positive::<K>(
                        &work_points,
                        triangles[triangle_index],
                        point,
                    )? {
                        bad[triangle_index] = true;
                        pending.extend(neighbors[triangle_index].iter().flatten().copied());
                    }
                }
                if !bad.iter().any(|is_bad| *is_bad) {
                    mark_bad_triangles_exhaustive::<K>(&work_points, &triangles, point, &mut bad)?;
                }
            } else {
                mark_bad_triangles_exhaustive::<K>(&work_points, &triangles, point, &mut bad)?;
            }
        } else {
            mark_bad_triangles_exhaustive::<K>(&work_points, &triangles, point, &mut bad)?;
        }

        let mut cavity = Vec::new();
        for (&triangle, is_bad) in triangles.iter().zip(&bad) {
            if *is_bad {
                add_cavity_edge(&mut cavity, Edge::new(triangle[0], triangle[1]));
                add_cavity_edge(&mut cavity, Edge::new(triangle[1], triangle[2]));
                add_cavity_edge(&mut cavity, Edge::new(triangle[2], triangle[0]));
            }
        }

        triangles = triangles
            .into_iter()
            .zip(bad)
            .filter_map(|(triangle, is_bad)| (!is_bad).then_some(triangle))
            .collect();

        for edge in cavity {
            match make_oriented::<K>(&work_points, [edge.from, edge.to, point]) {
                Ok(triangle) => triangles.push(triangle),
                Err(Error::InvalidInput {
                    reason: "degenerate triangle",
                }) => {}
                Err(error) => return Err(error),
            }
        }
    }

    triangles.retain(|triangle| !triangle.iter().any(|&index| index >= first_super));
    Ok(triangles)
}

fn brio_insertion_order<K>(points: &[Point2]) -> Result<Vec<usize>>
where
    K: Kernel,
{
    // A minimum first-round population avoids spending exact spatial-sort
    // comparisons on many tiny buckets. Above that floor, splitmix-assigned
    // levels produce the geometrically growing rounds of a biased randomized
    // insertion order without adding an RNG or making results process-seeded.
    const MIN_FIRST_ROUND: usize = 16;
    let mut highest_level = 0_usize;
    while (points.len() >> (highest_level + 1)) >= MIN_FIRST_ROUND {
        highest_level += 1;
    }

    let mut rounds = vec![Vec::new(); highest_level + 1];
    for point in 0..points.len() {
        let level = (splitmix64(point as u64).trailing_zeros() as usize).min(highest_level);
        rounds[level].push(point);
    }

    let mut order = Vec::with_capacity(points.len());
    for round in rounds.iter_mut().rev() {
        spatial_median_order::<K>(points, round, false)?;
        order.append(round);
    }
    Ok(order)
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn spatial_median_order<K>(points: &[Point2], indices: &mut [usize], split_y: bool) -> Result<()>
where
    K: Kernel,
{
    if indices.len() <= 1 {
        return Ok(());
    }

    let midpoint = indices.len() / 2;
    select_spatial_nth::<K>(points, indices, midpoint, split_y)?;

    let (lower, upper) = indices.split_at_mut(midpoint);
    spatial_median_order::<K>(points, lower, !split_y)?;
    spatial_median_order::<K>(points, &mut upper[1..], !split_y)
}

fn select_spatial_nth<K>(
    points: &[Point2],
    indices: &mut [usize],
    nth: usize,
    compare_y_first: bool,
) -> Result<()>
where
    K: Kernel,
{
    // `slice::select_nth_unstable_by` cannot propagate a fallible exact
    // comparison without making its comparator stateful and potentially
    // inconsistent after an error. This compact quickselect keeps comparison
    // failure explicit and leaves topology untouched if scheduling cannot be
    // decided.
    let mut lower = 0;
    let mut upper = indices.len();
    while upper - lower > 1 {
        let pivot_position = lower + (upper - lower) / 2;
        indices.swap(pivot_position, upper - 1);
        let pivot = indices[upper - 1];
        let mut split = lower;
        for candidate in lower..upper - 1 {
            if spatial_point_cmp::<K>(points, indices[candidate], pivot, compare_y_first)?
                == std::cmp::Ordering::Less
            {
                indices.swap(candidate, split);
                split += 1;
            }
        }
        indices.swap(split, upper - 1);
        match split.cmp(&nth) {
            std::cmp::Ordering::Less => lower = split + 1,
            std::cmp::Ordering::Greater => upper = split,
            std::cmp::Ordering::Equal => return Ok(()),
        }
    }
    Ok(())
}

fn spatial_point_cmp<K>(
    points: &[Point2],
    left: usize,
    right: usize,
    compare_y_first: bool,
) -> Result<std::cmp::Ordering>
where
    K: Kernel,
{
    let (left_primary, left_secondary) = if compare_y_first {
        (&points[left].y, &points[left].x)
    } else {
        (&points[left].x, &points[left].y)
    };
    let (right_primary, right_secondary) = if compare_y_first {
        (&points[right].y, &points[right].x)
    } else {
        (&points[right].x, &points[right].y)
    };
    let primary = K::cmp(left_primary, right_primary)?;
    if primary != std::cmp::Ordering::Equal {
        return Ok(primary);
    }
    Ok(K::cmp(left_secondary, right_secondary)?.then(left.cmp(&right)))
}

fn mark_bad_triangles_exhaustive<K>(
    points: &[Point2],
    triangles: &[Triangle],
    point: usize,
    bad: &mut [bool],
) -> Result<()>
where
    K: Kernel,
{
    for (triangle_index, &triangle) in triangles.iter().enumerate() {
        if incircle_inside_or_on_positive::<K>(points, triangle, point)? {
            bad[triangle_index] = true;
        }
    }
    Ok(())
}

fn triangle_neighbors(triangles: &[Triangle]) -> Vec<[Option<usize>; 3]> {
    let mut edges = Vec::with_capacity(triangles.len() * 3);
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        for (edge_index, (from, to)) in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ]
        .into_iter()
        .enumerate()
        {
            edges.push((from.min(to), from.max(to), triangle_index, edge_index));
        }
    }
    edges.sort_unstable();

    let mut neighbors = vec![[None; 3]; triangles.len()];
    let mut start = 0;
    while start < edges.len() {
        let mut end = start + 1;
        while end < edges.len() && edges[end].0 == edges[start].0 && edges[end].1 == edges[start].1
        {
            end += 1;
        }
        if end - start == 2 {
            let (_, _, first_triangle, first_edge) = edges[start];
            let (_, _, second_triangle, second_edge) = edges[start + 1];
            neighbors[first_triangle][first_edge] = Some(second_triangle);
            neighbors[second_triangle][second_edge] = Some(first_triangle);
        }
        start = end;
    }
    neighbors
}

fn locate_triangle<K>(
    points: &[Point2],
    triangles: &[Triangle],
    neighbors: &[[Option<usize>; 3]],
    point: usize,
    seed: usize,
) -> Result<Option<usize>>
where
    K: Kernel,
{
    if triangles.is_empty() {
        return Ok(None);
    }
    let mut current = seed.min(triangles.len() - 1);
    let mut visited = vec![false; triangles.len()];
    while !visited[current] {
        visited[current] = true;
        let triangle = triangles[current];
        let mut crossed_edge = false;
        for (edge_index, (from, to)) in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ]
        .into_iter()
        .enumerate()
        {
            if predicates::orient2d::<K>(&points[from], &points[to], &points[point])?
                == Sign::Negative
            {
                crossed_edge = true;
                let Some(neighbor) = neighbors[current][edge_index] else {
                    return Ok(None);
                };
                current = neighbor;
                break;
            }
        }
        if !crossed_edge {
            return Ok(Some(current));
        }
    }
    Ok(None)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Edge {
    from: usize,
    to: usize,
}

impl Edge {
    const fn new(from: usize, to: usize) -> Self {
        Self { from, to }
    }

    const fn same_undirected(self, other: Self) -> bool {
        (self.from == other.from && self.to == other.to)
            || (self.from == other.to && self.to == other.from)
    }
}

fn add_cavity_edge(edges: &mut Vec<Edge>, edge: Edge) {
    if let Some(position) = edges
        .iter()
        .position(|candidate| candidate.same_undirected(edge))
    {
        edges.remove(position);
    } else {
        edges.push(edge);
    }
}

fn super_triangle<K>(points: &[Point2]) -> Result<[Point2; 3]>
where
    K: Kernel,
{
    let bounds = Bounds::from_points::<K>(points)?;
    let dx = K::sub(&bounds.max_x, &bounds.min_x);
    let dy = K::sub(&bounds.max_y, &bounds.min_y);
    let span = if K::cmp(&dx, &dy)? == std::cmp::Ordering::Less {
        dy
    } else {
        dx
    };
    let one = K::from_i64(1);
    let two = K::from_i64(2);
    let radius = K::add(&K::mul(&span, &K::from_i64(64)), &one);
    let double_radius = K::mul(&radius, &two);
    let mid_x = K::div(&K::add(&bounds.min_x, &bounds.max_x), &two)?;
    let mid_y = K::div(&K::add(&bounds.min_y, &bounds.max_y), &two)?;

    Ok([
        Point2::new(K::sub(&mid_x, &double_radius), K::sub(&mid_y, &radius)),
        Point2::new(mid_x.clone(), K::add(&mid_y, &double_radius)),
        Point2::new(K::add(&mid_x, &double_radius), K::sub(&mid_y, &radius)),
    ])
}

#[derive(Clone, Debug)]
struct Bounds {
    min_x: Real,
    max_x: Real,
    min_y: Real,
    max_y: Real,
}

impl Bounds {
    fn from_points<K>(points: &[Point2]) -> Result<Self>
    where
        K: Kernel,
    {
        let Some(first) = points.first() else {
            return Err(Error::InvalidInput {
                reason: "Delaunay point set must be non-empty",
            });
        };

        let mut bounds = Bounds {
            min_x: first.x.clone(),
            max_x: first.x.clone(),
            min_y: first.y.clone(),
            max_y: first.y.clone(),
        };

        for point in &points[1..] {
            if K::cmp(&point.x, &bounds.min_x)? == std::cmp::Ordering::Less {
                bounds.min_x = point.x.clone();
            }
            if K::cmp(&point.x, &bounds.max_x)? == std::cmp::Ordering::Greater {
                bounds.max_x = point.x.clone();
            }
            if K::cmp(&point.y, &bounds.min_y)? == std::cmp::Ordering::Less {
                bounds.min_y = point.y.clone();
            }
            if K::cmp(&point.y, &bounds.max_y)? == std::cmp::Ordering::Greater {
                bounds.max_y = point.y.clone();
            }
        }

        Ok(bounds)
    }
}

fn make_oriented<K>(points: &[Point2], triangle: Triangle) -> Result<Triangle>
where
    K: Kernel,
{
    let sign = predicates::orient2d::<K>(
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
    )?;
    if sign == Sign::Zero {
        return Err(Error::InvalidInput {
            reason: "degenerate triangle",
        });
    }
    Ok(oriented_triangle(sign, triangle))
}

fn oriented_triangle(sign: Sign, mut triangle: Triangle) -> Triangle {
    if sign == Sign::Negative {
        triangle.swap(1, 2);
    }
    triangle
}

fn opposite_sides(first: Sign, second: Sign) -> bool {
    matches!(
        (first, second),
        (Sign::Negative, Sign::Positive) | (Sign::Positive, Sign::Negative)
    )
}

fn validate_constraints(point_count: usize, constraints: &[Constraint]) -> Result<()> {
    for constraint in constraints {
        if constraint.from >= point_count || constraint.to >= point_count {
            return Err(Error::InvalidInput {
                reason: "constraint index out of bounds",
            });
        }
        if constraint.from == constraint.to {
            return Err(Error::InvalidInput {
                reason: "constraint endpoints must differ",
            });
        }
    }

    Ok(())
}

fn validate_constraint_geometry(points: &[Point2], constraints: &[Constraint]) -> Result<()> {
    for first in 0..constraints.len() {
        for second in first + 1..constraints.len() {
            let a = constraints[first];
            let b = constraints[second];
            if constraints_share_endpoint(a, b) {
                continue;
            }

            // Public constraints are planarized before this check, so any
            // proper crossing or overlap here indicates a remaining PSLG
            // normalization bug. The classification remains exact and
            // predicate-backed.
            let intersection = predicates::segment_intersection(
                &points[a.from],
                &points[a.to],
                &points[b.from],
                &points[b.to],
            )?;
            if intersection.is_proper_crossing() {
                return Err(Error::InvalidInput {
                    reason: "properly crossing constraints are not supported",
                });
            }
            if intersection.has_positive_length_overlap() {
                return Err(Error::InvalidInput {
                    reason: "overlapping constraints are not supported",
                });
            }
        }
    }

    Ok(())
}

fn constraints_share_endpoint(first: Constraint, second: Constraint) -> bool {
    first.from == second.from
        || first.from == second.to
        || first.to == second.from
        || first.to == second.to
}

fn constrained_edges_already_delaunay(
    points: &[ExactPoint],
    constraints: &[Constraint],
    public_constraints: &[Constraint],
) -> Result<Option<ConstrainedDelaunayTriangulation>> {
    let triangulation = delaunay(points)?;

    // This accepts a narrow, exact subset of CDT inputs before full edge
    // insertion is ported: if every requested constrained segment already
    // exists as an edge in the unconstrained Delaunay triangulation, no DCEL
    // mutation or legalization is required. The empty-circumcircle property is
    // inherited from the Delaunay triangulation itself.
    if constraints
        .iter()
        .all(|constraint| triangulation_has_edge(triangulation.triangles(), *constraint))
    {
        let (points, triangles) = triangulation.into_parts();
        let constrained = ConstrainedDelaunayTriangulation::from_parts_with_constraint_edges(
            points,
            public_constraints.to_vec(),
            constraints.to_vec(),
            triangles,
        );
        constrained.validate()?;
        constrained.validate_unconstrained_edges_are_delaunay()?;
        return Ok(Some(constrained));
    }

    Ok(None)
}

fn triangulation_has_edge(triangles: &[Triangle], constraint: Constraint) -> bool {
    triangles
        .iter()
        .any(|triangle| triangle_contains_edge(*triangle, constraint.from, constraint.to))
}

fn triangle_contains_edge(triangle: Triangle, first: usize, second: usize) -> bool {
    triangle.contains(&first) && triangle.contains(&second)
}

fn validate_unique_points(points: &[Point2]) -> Result<()> {
    for i in 0..points.len() {
        for j in i + 1..points.len() {
            if points[i] == points[j] {
                return Err(Error::InvalidInput {
                    reason: "duplicate points are not supported",
                });
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i32, y: i32) -> ExactPoint {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn delaunay_returns_single_exact_triangle() {
        let points = vec![p(0, 0), p(2, 0), p(0, 2)];

        let triangulation = delaunay(&points).unwrap();

        assert_eq!(triangulation.triangles(), &[[0, 1, 2]]);
    }

    #[test]
    fn delaunay_rejects_collinear_triangle() {
        let points = vec![p(0, 0), p(1, 1), p(2, 2)];

        let triangulation = delaunay(&points).unwrap();

        assert!(triangulation.triangles().is_empty());
    }

    #[test]
    fn delaunay_rejects_duplicate_points() {
        let points = vec![p(0, 0), p(1, 0), p(1, 0), p(0, 1)];

        let error = delaunay(&points).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "duplicate points are not supported"
            }
        );
    }

    #[test]
    fn spatial_delaunay_preserves_indices_and_unique_topology() {
        let points = vec![
            p(0, 0),
            p(7, 1),
            p(2, 6),
            p(9, 8),
            p(4, 3),
            p(1, 9),
            p(8, 4),
            p(5, 10),
            p(11, 2),
        ];
        let ordinary = delaunay(&points).unwrap();
        let spatial = delaunay_spatial(&points).unwrap();

        ordinary.validate().unwrap();
        spatial.validate().unwrap();
        assert_eq!(spatial.points(), points.as_slice());

        let canonical = |triangulation: &DelaunayTriangulation| {
            let mut triangles = triangulation.triangles().to_vec();
            for triangle in &mut triangles {
                triangle.sort_unstable();
            }
            triangles.sort_unstable();
            triangles
        };
        assert_eq!(canonical(&ordinary), canonical(&spatial));
    }

    #[test]
    fn brio_order_is_deterministic_permutation() {
        let points = (0..80)
            .map(|index| p((index * 17) % 83, (index * 29) % 89))
            .collect::<Vec<_>>();
        let first = brio_insertion_order::<ExactKernel>(&points).unwrap();
        let second = brio_insertion_order::<ExactKernel>(&points).unwrap();
        let mut sorted = first.clone();
        sorted.sort_unstable();

        assert_eq!(first, second);
        assert_eq!(sorted, (0..points.len()).collect::<Vec<_>>());
    }

    #[test]
    fn delaunay_triangulates_convex_quad_with_one_diagonal() {
        let points = vec![p(0, 0), p(1, 0), p(1, 1), p(0, 1)];

        let triangulation = delaunay(&points).unwrap();

        assert_eq!(triangulation.triangles().len(), 2);
        assert!(
            triangulation
                .triangles()
                .iter()
                .all(|triangle| triangle[0] != triangle[1]
                    && triangle[1] != triangle[2]
                    && triangle[0] != triangle[2])
        );
    }

    #[test]
    fn delaunay_triangulates_square_with_center_point() {
        let points = vec![p(0, 0), p(4, 0), p(4, 4), p(0, 4), p(2, 2)];

        let triangulation = delaunay(&points).unwrap();

        assert_eq!(triangulation.triangles().len(), 4);
        assert!(
            triangulation
                .triangles()
                .iter()
                .all(|triangle| triangle.contains(&4))
        );
    }

    #[test]
    fn delaunay_triangulates_larger_non_cocircular_set() {
        let points = vec![p(0, 0), p(3, 0), p(5, 2), p(4, 5), p(1, 4), p(2, 2)];

        let triangulation = delaunay(&points).unwrap();

        assert_eq!(triangulation.triangles().len(), 5);
        assert!(
            triangulation
                .triangles()
                .iter()
                .flatten()
                .all(|&index| index < points.len())
        );
    }

    #[test]
    fn located_cavity_triangulates_and_validates_large_exact_set() {
        let points = (0..400_i32)
            .map(|index| {
                p(
                    (index % 20) * 100 + (index * 17) % 31,
                    (index / 20) * 100 + (index * 29) % 37,
                )
            })
            .collect::<Vec<_>>();

        let triangulation = delaunay(&points).unwrap();

        triangulation.validate().unwrap();
        for index in 0..points.len() {
            assert!(
                triangulation
                    .triangles()
                    .iter()
                    .any(|triangle| triangle.contains(&index))
            );
        }
    }

    #[cfg(feature = "earcut")]
    #[test]
    fn constrained_closed_ring_uses_exact_polygon_path() {
        let points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
        let constraints = vec![
            Constraint::new(0, 1),
            Constraint::new(1, 2),
            Constraint::new(2, 3),
            Constraint::new(3, 0),
        ];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert_eq!(triangulation.triangles().len(), 2);
    }

    #[cfg(not(feature = "earcut"))]
    #[test]
    fn constrained_closed_ring_falls_back_to_general_cdt_without_earcut() {
        let points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
        let constraints = vec![
            Constraint::new(0, 1),
            Constraint::new(1, 2),
            Constraint::new(2, 3),
            Constraint::new(3, 0),
        ];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate().unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay()
            .unwrap();
        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert!(
            constraints
                .iter()
                .all(|constraint| triangulation_has_edge(triangulation.triangles(), *constraint))
        );
    }

    #[cfg(feature = "earcut")]
    #[test]
    fn constrained_closed_rings_with_hole_use_exact_polygon_path() {
        let points = vec![
            p(0, 0),
            p(6, 0),
            p(6, 6),
            p(0, 6),
            p(2, 2),
            p(4, 2),
            p(4, 4),
            p(2, 4),
        ];
        let constraints = vec![
            Constraint::new(0, 1),
            Constraint::new(6, 5),
            Constraint::new(2, 3),
            Constraint::new(7, 4),
            Constraint::new(1, 2),
            Constraint::new(5, 4),
            Constraint::new(3, 0),
            Constraint::new(7, 6),
        ];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert_eq!(triangulation.triangles().len(), 8);
        assert!(
            triangulation
                .triangles()
                .iter()
                .flatten()
                .all(|&index| index < points.len())
        );
    }

    #[test]
    fn separated_closed_rings_use_general_cdt_recovery() {
        let points = vec![
            p(0, 0),
            p(1, 0),
            p(1, 1),
            p(0, 1),
            p(3, 0),
            p(4, 0),
            p(4, 1),
            p(3, 1),
        ];
        let constraints = vec![
            Constraint::new(0, 1),
            Constraint::new(1, 2),
            Constraint::new(2, 3),
            Constraint::new(3, 0),
            Constraint::new(4, 5),
            Constraint::new(5, 6),
            Constraint::new(6, 7),
            Constraint::new(7, 4),
        ];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate().unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay()
            .unwrap();
        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert!(
            constraints
                .iter()
                .all(|constraint| triangulation_has_edge(triangulation.triangles(), *constraint))
        );
    }

    #[test]
    fn constrained_edges_already_present_in_delaunay_are_accepted() {
        let points = vec![p(0, 0), p(3, 0), p(0, 2), p(1, 1)];
        let constraints = vec![Constraint::new(0, 1), Constraint::new(0, 3)];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert!(
            constraints
                .iter()
                .all(|constraint| triangulation_has_edge(triangulation.triangles(), *constraint))
        );
    }

    #[test]
    fn constrained_non_delaunay_diagonal_is_recovered_by_edge_flips() {
        let points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
        let constraints = vec![Constraint::new(1, 3)];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert_eq!(triangulation.triangles().len(), 2);
        assert!(triangulation_has_edge(
            triangulation.triangles(),
            constraints[0]
        ));
        assert!(!triangulation_has_edge(
            triangulation.triangles(),
            Constraint::new(0, 2)
        ));
    }

    #[test]
    fn constrained_segment_through_existing_vertex_is_split_and_recovered() {
        let points = vec![p(0, 0), p(2, 0), p(1, 0), p(0, 2)];
        let constraints = vec![Constraint::new(0, 1)];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert_eq!(
            triangulation.constraint_edges(),
            &[Constraint::new(0, 2), Constraint::new(2, 1)]
        );
        assert_eq!(triangulation.triangles().len(), 2);
        assert!(triangulation_has_edge(
            triangulation.triangles(),
            Constraint::new(0, 2)
        ));
        assert!(triangulation_has_edge(
            triangulation.triangles(),
            Constraint::new(2, 1)
        ));
    }

    #[test]
    fn overlapping_collinear_constraints_are_split_to_shared_subsegments() {
        let points = vec![p(0, 0), p(1, 0), p(2, 0), p(3, 0), p(0, 2)];
        let constraints = vec![Constraint::new(0, 3), Constraint::new(1, 2)];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert_eq!(
            triangulation.constraint_edges(),
            &[
                Constraint::new(0, 1),
                Constraint::new(1, 2),
                Constraint::new(2, 3)
            ]
        );
        for edge in [
            Constraint::new(0, 1),
            Constraint::new(1, 2),
            Constraint::new(2, 3),
        ] {
            assert!(triangulation_has_edge(triangulation.triangles(), edge));
        }
    }

    #[test]
    fn properly_crossing_constraints_insert_exact_intersection_vertex() {
        let points = vec![p(0, 0), p(2, 2), p(0, 2), p(2, 0)];
        let constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        assert_eq!(triangulation.constraints(), constraints.as_slice());
        assert_eq!(triangulation.points().len(), 5);
        assert_eq!(triangulation.points()[4], p(1, 1));
        assert_eq!(
            triangulation.constraint_edges(),
            &[
                Constraint::new(0, 4),
                Constraint::new(4, 1),
                Constraint::new(2, 4),
                Constraint::new(4, 3)
            ]
        );
        for edge in [
            Constraint::new(0, 4),
            Constraint::new(4, 1),
            Constraint::new(2, 4),
            Constraint::new(4, 3),
        ] {
            assert!(triangulation_has_edge(triangulation.triangles(), edge));
        }
    }

    #[test]
    fn validation_rejects_missing_constraint_edge() {
        let points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
        let triangulation = ConstrainedDelaunayTriangulation::from_parts_with_constraint_edges(
            points,
            vec![Constraint::new(0, 2)],
            vec![Constraint::new(0, 2)],
            vec![[0, 1, 3], [1, 2, 3]],
        );

        let error = triangulation.validate().unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "constraint edge missing from triangulation"
            }
        );
    }

    #[test]
    fn validation_rejects_illegal_unconstrained_interior_edge() {
        let points = vec![p(0, 0), p(4, 0), p(3, 2), p(0, 3)];
        let triangulation = DelaunayTriangulation::from_parts(points, vec![[0, 1, 3], [1, 2, 3]]);

        let error = triangulation.validate().unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "unconstrained interior edge violates Delaunay legality"
            }
        );
    }

    #[test]
    fn validation_allows_constrained_interior_edge_to_be_non_delaunay() {
        let points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
        let constraints = vec![Constraint::new(1, 3)];

        let triangulation = constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate().unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay()
            .unwrap();
    }

    #[test]
    fn rejects_all_collinear_overlapping_constraints_without_area() {
        let points = vec![p(0, 0), p(3, 0), p(1, 0), p(4, 0)];
        let constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

        let error = constrained_delaunay(&points, &constraints).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "four-point set is degenerate"
            }
        );
    }

    #[test]
    fn rejects_constraint_index_out_of_bounds() {
        let points = vec![p(0, 0), p(1, 0), p(0, 1)];

        let error = constrained_delaunay(&points, &[Constraint::new(0, 3)]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "constraint index out of bounds"
            }
        );
    }
}
