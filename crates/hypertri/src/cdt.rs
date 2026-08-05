//! Constrained topology and Delaunay triangulation.
//!
//! The implementation handles exact incremental Delaunay point sets, planarizes
//! crossing constraints with exact Steiner points, recovers protected edges by
//! flipping or exact cavity retriangulation, and re-legalizes unprotected
//! edges. Topology-only consumers can omit empty-circle quality work and use a
//! deterministic lexicographic triangulation. The production paths own their
//! topology implementation and have no external triangulation dependency.

use crate::cdt_constraints;
use crate::context::{TriangulationContext, TriangulationOutcome};
use crate::error::{Error, Result};
use crate::kernel::ExactKernel;
use crate::predicates;
use crate::types::{Constraint, ExactPoint, Point2, Real, Triangle};
use crate::types::{Sign, TriangleLocation};

mod topology;

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
    pub fn validate(&self, context: &TriangulationContext) -> Result<TriangulationOutcome<()>> {
        let kernel = ExactKernel::new(context);
        crate::cdt_validate::validate_delaunay(&kernel, &self.points, &self.triangles)?;
        Ok(kernel.finish(()))
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
pub struct ConstrainedTriangulation {
    points: Vec<ExactPoint>,
    constraints: Vec<Constraint>,
    constraint_edges: Vec<Constraint>,
    triangles: Vec<Triangle>,
}

impl ConstrainedTriangulation {
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
    pub fn validate(&self, context: &TriangulationContext) -> Result<TriangulationOutcome<()>> {
        let kernel = ExactKernel::new(context);
        crate::cdt_validate::validate_constrained_topology(
            &kernel,
            &self.points,
            &self.constraint_edges,
            &self.triangles,
        )?;
        Ok(kernel.finish(()))
    }

    /// Validate the exact local Delaunay condition on unconstrained interior
    /// edges.
    ///
    /// This check implements the Constrained Delaunay Lemma criterion: after
    /// protected PSLG edges are excluded, each interior edge must satisfy the
    /// empty-circle legality test against the two adjacent triangles.
    pub fn validate_unconstrained_edges_are_delaunay(
        &self,
        context: &TriangulationContext,
    ) -> Result<TriangulationOutcome<()>> {
        let kernel = ExactKernel::new(context);
        crate::cdt_validate::validate_constrained_delaunay(
            &kernel,
            &self.points,
            &self.constraint_edges,
            &self.triangles,
        )?;
        Ok(kernel.finish(()))
    }
}

/// Triangulate an exact point set with Delaunay edge selection.
///
/// The local edge choices use the empty-circumcircle rule from Delaunay
/// triangulation; the in-circle predicate is evaluated exactly through the
/// crate-local kernel.
pub fn delaunay(
    context: &TriangulationContext,
    points: &[ExactPoint],
) -> Result<TriangulationOutcome<DelaunayTriangulation>> {
    let kernel = ExactKernel::new(context);
    let triangulation = delaunay_inner(&kernel, points)?;
    Ok(kernel.finish(triangulation))
}

fn delaunay_inner(kernel: &ExactKernel, points: &[ExactPoint]) -> Result<DelaunayTriangulation> {
    validate_unique_points(kernel, points)?;
    delaunay_from_validated(kernel, points)
}

fn delaunay_from_validated(
    kernel: &ExactKernel,
    points: &[ExactPoint],
) -> Result<DelaunayTriangulation> {
    delaunay_from_validated_owned(kernel, points.to_vec())
}

fn delaunay_from_validated_owned(
    kernel: &ExactKernel,
    points: Vec<ExactPoint>,
) -> Result<DelaunayTriangulation> {
    let triangles = delaunay_triangles(kernel, &points)?;
    // Every construction branch admits only nondegenerate, positively
    // oriented triangles and makes each Delaunay choice with the exact
    // in-circle predicate. Re-running the public validator here would repeat
    // those predicates over the completed mesh; callers can still invoke
    // `validate` explicitly on records assembled or deserialized elsewhere.
    Ok(DelaunayTriangulation::from_parts(points, triangles))
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
pub fn delaunay_spatial(
    context: &TriangulationContext,
    points: &[ExactPoint],
) -> Result<TriangulationOutcome<DelaunayTriangulation>> {
    let kernel = ExactKernel::new(context);
    let triangulation = delaunay_spatial_inner(&kernel, points)?;
    Ok(kernel.finish(triangulation))
}

fn delaunay_spatial_inner(
    kernel: &ExactKernel,
    points: &[ExactPoint],
) -> Result<DelaunayTriangulation> {
    validate_unique_points(kernel, points)?;
    delaunay_spatial_from_validated(kernel, points)
}

fn delaunay_spatial_from_validated(
    kernel: &ExactKernel,
    points: &[ExactPoint],
) -> Result<DelaunayTriangulation> {
    delaunay_spatial_from_validated_owned(kernel, points.to_vec())
}

fn delaunay_spatial_from_validated_owned(
    kernel: &ExactKernel,
    points: Vec<ExactPoint>,
) -> Result<DelaunayTriangulation> {
    let triangles = delaunay_triangles_spatial(kernel, &points)?;
    Ok(DelaunayTriangulation::from_parts(points, triangles))
}

/// Triangulate exact points with constraints.
///
/// The constrained path appends exact intersection vertices for proper
/// constraint crossings, normalizes constraints into internal PSLG
/// subsegments, and recovers those subsegments with exact edge flips or exact
/// cavity retriangulation.
///
/// Closed polygon-with-hole inputs are routed through the boundary-preserving
/// polygon path when `earcut` is enabled. Other planar straight-line graphs are
/// triangulated over their convex hull, with protected subsegments excluded
/// from local Delaunay flips. Local legality requires every unprotected
/// interior edge to satisfy the exact empty-circle test.
pub fn constrained_delaunay(
    context: &TriangulationContext,
    points: &[ExactPoint],
    constraints: &[Constraint],
) -> Result<TriangulationOutcome<ConstrainedTriangulation>> {
    let kernel = ExactKernel::new(context);
    let triangulation = constrained_delaunay_inner(&kernel, points, constraints)?;
    Ok(kernel.finish(triangulation))
}

/// Triangulate a planar straight-line graph over its complete convex hull.
///
/// Unlike [`constrained_delaunay`], this entry point does not interpret closed
/// constraint cycles as a polygon boundary with holes. Every bounded region on
/// either side of every protected cycle remains triangulated. This is the
/// appropriate domain for surface corefinement and other planar-complex work
/// where an interior ring separates cells instead of removing material.
/// Constraints must already form a PSLG: they may not cross, overlap, or contain
/// an input point that is not one of their endpoints. Invalid structure is
/// rejected rather than silently planarized; use [`constrained_delaunay`] when
/// authored constraints still require exact intersection insertion.
pub fn constrained_delaunay_convex_hull(
    context: &TriangulationContext,
    points: &[ExactPoint],
    constraints: &[Constraint],
) -> Result<TriangulationOutcome<ConstrainedTriangulation>> {
    let kernel = ExactKernel::new(context);
    validate_constraints(points.len(), constraints)?;
    validate_unique_points(&kernel, points)?;
    validate_constraint_geometry(&kernel, points, constraints, true)?;
    let triangulation = constrained_delaunay_planar_inner(
        &kernel,
        points.to_vec(),
        constraints,
        constraints.to_vec(),
        false,
    )?;
    Ok(kernel.finish(triangulation))
}

/// Triangulate a planar straight-line graph over its complete convex hull
/// without imposing a Delaunay quality policy.
///
/// Every input point is inserted with exact orientation predicates, then every
/// protected PSLG edge is recovered through the same exact flip/cavity path as
/// [`constrained_delaunay_convex_hull`]. The result covers both sides of closed
/// interior cycles. Unlike the Delaunay entry point, unprotected diagonals are
/// not legalized with empty-circle predicates. This is the appropriate path
/// for topology-only consumers such as surface arrangements.
///
/// Constraints must already be planarized: they may not cross, overlap, or
/// contain an input point other than an endpoint. Every returned triangle is
/// strictly positively oriented in the caller's coordinate axes, and every
/// input constraint is present as a triangulation edge. Because this checked
/// entry point never inserts Steiner points, the returned triangles index the
/// caller's point slice directly. The active policy and aggregate certainty
/// govern every predicate used to establish those postconditions.
pub fn constrained_triangulation_convex_hull(
    context: &TriangulationContext,
    points: &[ExactPoint],
    constraints: &[Constraint],
) -> Result<TriangulationOutcome<Vec<Triangle>>> {
    let kernel = ExactKernel::new(context);
    validate_constraints(points.len(), constraints)?;
    validate_unique_points(&kernel, points)?;
    validate_constraint_geometry(&kernel, points, constraints, true)?;

    let triangles = topology::triangulate_point_set(&kernel, points)?;
    let triangles =
        crate::cdt_insert::insert_constraints_topology(&kernel, points, triangles, constraints)?;
    crate::cdt_validate::validate_constrained_convex_hull_topology(
        &kernel,
        points,
        constraints,
        &triangles,
    )?;
    Ok(kernel.finish(triangles))
}

pub(crate) fn constrained_delaunay_inner(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    constraints: &[Constraint],
) -> Result<ConstrainedTriangulation> {
    constrained_delaunay_inner_with_polygon_dispatch(kernel, points, constraints, true)
}

fn constrained_delaunay_inner_with_polygon_dispatch(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    constraints: &[Constraint],
    dispatch_closed_polygon: bool,
) -> Result<ConstrainedTriangulation> {
    validate_constraints(points.len(), constraints)?;
    validate_unique_points(kernel, points)?;

    if constraints.is_empty() {
        return constrained_delaunay_planar_inner(
            kernel,
            points.to_vec(),
            constraints,
            Vec::new(),
            dispatch_closed_polygon,
        );
    }

    let planar = crate::cdt_insert::planarize_constraints(kernel, points, constraints)?;
    let points = planar.points;
    let internal_constraints = planar.constraints;
    validate_constraint_geometry(kernel, &points, &internal_constraints, false)?;

    constrained_delaunay_planar_inner(
        kernel,
        points,
        constraints,
        internal_constraints,
        dispatch_closed_polygon,
    )
}

fn constrained_delaunay_planar_inner(
    kernel: &ExactKernel,
    points: Vec<ExactPoint>,
    public_constraints: &[Constraint],
    internal_constraints: Vec<Constraint>,
    dispatch_closed_polygon: bool,
) -> Result<ConstrainedTriangulation> {
    if internal_constraints.is_empty() {
        let triangulation = delaunay_from_validated_owned(kernel, points)?;
        return Ok(ConstrainedTriangulation::from_parts_with_constraint_edges(
            triangulation.points,
            Vec::new(),
            Vec::new(),
            triangulation.triangles,
        ));
    }

    if dispatch_closed_polygon
        && let Some(polygon) = cdt_constraints::polygon_from_closed_constraints(
            kernel,
            &points,
            &internal_constraints,
        )?
    {
        // Closed-constraint recognition has already proved ring topology.
        // Policy-independent structural facts can guide later scheduling;
        // winding, convexity, and other policy-derived decisions remain local
        // to this operation and use this same kernel.
        #[cfg(feature = "earcut")]
        {
            let (flat_points, hole_indices, source_indices) = polygon.to_flat_polygon(&points);
            let flat = crate::earcut::triangulate_inner(kernel, &flat_points, &hole_indices)?;
            if !flat.len().is_multiple_of(3) {
                return Err(Error::InvalidInput {
                    reason: "polygon triangulation index count is not a multiple of three",
                });
            }
            if flat.iter().any(|&index| index >= source_indices.len()) {
                return Err(Error::InvalidInput {
                    reason: "polygon triangulation index is out of bounds",
                });
            }
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

            let triangulation = ConstrainedTriangulation::from_parts_with_constraint_edges(
                points,
                public_constraints.to_vec(),
                internal_constraints,
                triangles,
            );
            crate::cdt_validate::validate_constrained_topology(
                kernel,
                &triangulation.points,
                &triangulation.constraint_edges,
                &triangulation.triangles,
            )?;
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

    let base = match constrained_edges_already_delaunay(
        kernel,
        points,
        &internal_constraints,
        public_constraints,
    )? {
        Ok(triangulation) => return Ok(triangulation),
        Err(base) => base,
    };

    let (points, base_triangles) = base.into_parts();
    let triangles = crate::cdt_insert::insert_constraints(
        kernel,
        &points,
        base_triangles,
        &internal_constraints,
    )?;
    let triangulation = ConstrainedTriangulation::from_parts_with_constraint_edges(
        points,
        public_constraints.to_vec(),
        internal_constraints,
        triangles,
    );
    crate::cdt_validate::validate_constrained_convex_hull_delaunay(
        kernel,
        &triangulation.points,
        &triangulation.constraint_edges,
        &triangulation.triangles,
    )?;
    Ok(triangulation)
}

fn delaunay_triangles(kernel: &ExactKernel, points: &[Point2]) -> Result<Vec<Triangle>> {
    match points.len() {
        0..=2 => Ok(Vec::new()),
        3 => triangle_if_not_degenerate(kernel, points, [0, 1, 2])
            .map(|triangle| triangle.into_iter().collect()),
        4 => delaunay_quad(kernel, points),
        _ => incremental_delaunay(kernel, points),
    }
}

fn delaunay_triangles_spatial(kernel: &ExactKernel, points: &[Point2]) -> Result<Vec<Triangle>> {
    match points.len() {
        0..=4 => delaunay_triangles(kernel, points),
        _ => {
            let order = brio_insertion_order(kernel, points)?;
            incremental_delaunay_in_order(kernel, points, &order)
        }
    }
}

fn triangle_if_not_degenerate(
    kernel: &ExactKernel,
    points: &[Point2],
    triangle: Triangle,
) -> Result<Option<Triangle>> {
    let sign = predicates::orient2(
        kernel,
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
    )?;
    Ok((sign != Sign::Zero).then_some(oriented_triangle(sign, triangle)))
}

fn delaunay_quad(kernel: &ExactKernel, points: &[Point2]) -> Result<Vec<Triangle>> {
    if let Some(triangles) = triangulate_interior_point(kernel, points)? {
        return Ok(triangles);
    }

    for diagonal in [(0, 1, 2, 3), (0, 2, 1, 3), (0, 3, 1, 2)] {
        let (a, b, c, d) = diagonal;
        let ac = predicates::orient2(kernel, &points[a], &points[b], &points[c])?;
        let ad = predicates::orient2(kernel, &points[a], &points[b], &points[d])?;
        if !opposite_sides(ac, ad) {
            continue;
        }

        if diagonal_is_delaunay(kernel, points, a, b, c, d)? {
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

fn triangulate_interior_point(
    kernel: &ExactKernel,
    points: &[Point2],
) -> Result<Option<Vec<Triangle>>> {
    for interior in 0..4 {
        let outer = (0..4)
            .filter(|&index| index != interior)
            .collect::<Vec<_>>();
        let location = kernel.classify_point_triangle(
            &points[outer[0]],
            &points[outer[1]],
            &points[outer[2]],
            &points[interior],
        )?;
        match location {
            TriangleLocation::Inside => {
                return Ok(Some(vec![
                    make_oriented(kernel, points, [interior, outer[0], outer[1]])?,
                    make_oriented(kernel, points, [interior, outer[1], outer[2]])?,
                    make_oriented(kernel, points, [interior, outer[2], outer[0]])?,
                ]));
            }
            TriangleLocation::OnEdge => {
                return triangulate_point_on_triangle_edge(
                    kernel,
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

fn triangulate_point_on_triangle_edge(
    kernel: &ExactKernel,
    points: &[Point2],
    point: usize,
    triangle: Triangle,
) -> Result<Vec<Triangle>> {
    for edge_index in 0..3 {
        let a = triangle[edge_index];
        let b = triangle[(edge_index + 1) % 3];
        let c = triangle[(edge_index + 2) % 3];
        if predicates::point_on_segment(kernel, &points[a], &points[b], &points[point])? {
            return Ok(vec![
                make_oriented(kernel, points, [a, point, c])?,
                make_oriented(kernel, points, [point, b, c])?,
            ]);
        }
    }

    Err(Error::InvalidInput {
        reason: "edge point was not on a triangle edge",
    })
}

fn diagonal_is_delaunay(
    kernel: &ExactKernel,
    points: &[Point2],
    a: usize,
    b: usize,
    c: usize,
    d: usize,
) -> Result<bool> {
    let abc = predicates::orient2(kernel, &points[a], &points[b], &points[c])?;
    let abd = predicates::orient2(kernel, &points[a], &points[b], &points[d])?;
    let d_in_abc = incircle_inside(kernel, points, [a, b, c], d, abc)?;
    let c_in_abd = incircle_inside(kernel, points, [a, d, b], c, abd.reversed())?;
    Ok(!d_in_abc && !c_in_abd)
}

fn incircle_inside(
    kernel: &ExactKernel,
    points: &[Point2],
    triangle: Triangle,
    point: usize,
    orientation: Sign,
) -> Result<bool> {
    let sign = kernel.incircle2(
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

fn incircle_inside_or_on_positive(
    kernel: &ExactKernel,
    points: &[Point2],
    triangle: Triangle,
    point: usize,
) -> Result<bool> {
    // Incremental Bowyer-Watson insertion creates the seed and every cavity
    // replacement through `make_oriented`, so active triangles carry a
    // certified positive-orientation invariant. Reuse that object fact rather
    // than evaluating the orientation determinant again for every candidate
    // point/circumcircle pair.
    let sign = kernel.incircle2(
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
        &points[point],
    )?;
    Ok(matches!(sign, Sign::Positive | Sign::Zero))
}

fn incremental_delaunay(kernel: &ExactKernel, points: &[Point2]) -> Result<Vec<Triangle>> {
    incremental_delaunay_in_order(kernel, points, &(0..points.len()).collect::<Vec<_>>())
}

fn incremental_delaunay_in_order(
    kernel: &ExactKernel,
    points: &[Point2],
    insertion_order: &[usize],
) -> Result<Vec<Triangle>> {
    let two = ExactKernel::from_i64(2);
    let mut super_scale = ExactKernel::from_i64(64);
    loop {
        let triangles =
            incremental_delaunay_attempt(kernel, points, insertion_order, &super_scale)?;
        if crate::cdt_validate::triangulates_convex_hull(kernel, points, &triangles)? {
            return Ok(triangles);
        }
        super_scale = ExactKernel::mul(&super_scale, &two);
    }
}

fn incremental_delaunay_attempt(
    kernel: &ExactKernel,
    points: &[Point2],
    insertion_order: &[usize],
    super_scale: &Real,
) -> Result<Vec<Triangle>> {
    let mut work_points = points.to_vec();
    let first_super = work_points.len();
    work_points.extend(super_triangle(kernel, points, super_scale)?);

    // This is the Bowyer-Watson empty-circumcircle update: remove every
    // triangle whose circumcircle contains the inserted point, then stitch the
    // boundary cavity back to the point. The empty-circle test is evaluated by
    // the exact predicate kernel.
    let mut triangles = vec![make_oriented(
        kernel,
        &work_points,
        [first_super, first_super + 1, first_super + 2],
    )?];

    for &point in insertion_order {
        let mut bad = vec![false; triangles.len()];
        if triangles.len() >= LOCATED_CAVITY_THRESHOLD {
            let neighbors = triangle_neighbors(&triangles)?;
            if let Some(seed) = locate_triangle(
                kernel,
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
                    if incircle_inside_or_on_positive(
                        kernel,
                        &work_points,
                        triangles[triangle_index],
                        point,
                    )? {
                        bad[triangle_index] = true;
                        pending.extend(neighbors[triangle_index].iter().flatten().copied());
                    }
                }
                if !bad.iter().any(|is_bad| *is_bad) {
                    mark_bad_triangles_exhaustive(
                        kernel,
                        &work_points,
                        &triangles,
                        point,
                        &mut bad,
                    )?;
                }
            } else {
                mark_bad_triangles_exhaustive(kernel, &work_points, &triangles, point, &mut bad)?;
            }
        } else {
            mark_bad_triangles_exhaustive(kernel, &work_points, &triangles, point, &mut bad)?;
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
            match make_oriented(kernel, &work_points, [edge.from, edge.to, point]) {
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

fn brio_insertion_order(kernel: &ExactKernel, points: &[Point2]) -> Result<Vec<usize>> {
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
        spatial_median_order(kernel, points, round, false)?;
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

fn spatial_median_order(
    kernel: &ExactKernel,
    points: &[Point2],
    indices: &mut [usize],
    split_y: bool,
) -> Result<()> {
    if indices.len() <= 1 {
        return Ok(());
    }

    let midpoint = indices.len() / 2;
    select_spatial_nth(kernel, points, indices, midpoint, split_y)?;

    let (lower, upper) = indices.split_at_mut(midpoint);
    spatial_median_order(kernel, points, lower, !split_y)?;
    spatial_median_order(kernel, points, &mut upper[1..], !split_y)
}

fn select_spatial_nth(
    kernel: &ExactKernel,
    points: &[Point2],
    indices: &mut [usize],
    nth: usize,
    compare_y_first: bool,
) -> Result<()> {
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
            if spatial_point_cmp(kernel, points, indices[candidate], pivot, compare_y_first)?
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

fn spatial_point_cmp(
    kernel: &ExactKernel,
    points: &[Point2],
    left: usize,
    right: usize,
    compare_y_first: bool,
) -> Result<std::cmp::Ordering> {
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
    let primary = kernel.cmp(left_primary, right_primary)?;
    if primary != std::cmp::Ordering::Equal {
        return Ok(primary);
    }
    Ok(kernel
        .cmp(left_secondary, right_secondary)?
        .then(left.cmp(&right)))
}

fn mark_bad_triangles_exhaustive(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &[Triangle],
    point: usize,
    bad: &mut [bool],
) -> Result<()> {
    for (triangle_index, &triangle) in triangles.iter().enumerate() {
        if incircle_inside_or_on_positive(kernel, points, triangle, point)? {
            bad[triangle_index] = true;
        }
    }
    Ok(())
}

fn triangle_neighbors(triangles: &[Triangle]) -> Result<Vec<[Option<usize>; 3]>> {
    let mut neighbors = vec![[None; 3]; triangles.len()];
    if triangles.is_empty() {
        return Ok(neighbors);
    }
    let table_capacity = triangles
        .len()
        .checked_mul(2)
        .and_then(|edges| edges.checked_add(1))
        .and_then(usize::checked_next_power_of_two)
        .ok_or(Error::InvalidInput {
            reason: "triangle adjacency capacity overflow",
        })?
        .max(8);
    // A connected triangulated disk has at most `2T + 1` unique edges: the
    // upper bound is reached when every finite vertex lies on its boundary.
    // Keep the table sparse without paying to sort all three edge occurrences
    // after every insertion. The bounded probe below turns any violated
    // occupancy invariant into a typed error rather than an unbounded search.
    let mut table = vec![NeighborEdgeSlot::EMPTY; table_capacity];
    let mask = table_capacity - 1;

    for (triangle_index, triangle) in triangles.iter().enumerate() {
        let owner_base = triangle_index
            .checked_mul(3)
            .filter(|owner| *owner <= NeighborEdgeSlot::PAIRED - 3)
            .ok_or(Error::InvalidInput {
                reason: "triangle adjacency owner overflow",
            })?;
        for (edge_index, (from, to)) in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ]
        .into_iter()
        .enumerate()
        {
            let (from, to) = (from.min(to), from.max(to));
            let mut position = neighbor_edge_hash(from, to) & mask;
            let owner = owner_base + edge_index;
            let mut probes = 0;
            loop {
                let slot = &mut table[position];
                if slot.owner == usize::MAX {
                    *slot = NeighborEdgeSlot { from, to, owner };
                    break;
                }
                if slot.from == from && slot.to == to {
                    if slot.owner == NeighborEdgeSlot::PAIRED {
                        return Err(Error::InvalidInput {
                            reason: "Delaunay edge has more than two incident faces",
                        });
                    }
                    let first_triangle = slot.owner / 3;
                    let first_edge = slot.owner % 3;
                    neighbors[first_triangle][first_edge] = Some(triangle_index);
                    neighbors[triangle_index][edge_index] = Some(first_triangle);
                    slot.owner = NeighborEdgeSlot::PAIRED;
                    break;
                }
                probes += 1;
                if probes == table_capacity {
                    return Err(Error::InvalidInput {
                        reason: "triangle adjacency table occupancy invariant failed",
                    });
                }
                position = (position + 1) & mask;
            }
        }
    }
    Ok(neighbors)
}

#[derive(Clone, Copy)]
struct NeighborEdgeSlot {
    from: usize,
    to: usize,
    owner: usize,
}

impl NeighborEdgeSlot {
    const PAIRED: usize = usize::MAX - 1;
    const EMPTY: Self = Self {
        from: 0,
        to: 0,
        owner: usize::MAX,
    };
}

fn neighbor_edge_hash(from: usize, to: usize) -> usize {
    splitmix64((from as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ (to as u64).rotate_left(32))
        as usize
}

fn locate_triangle(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &[Triangle],
    neighbors: &[[Option<usize>; 3]],
    point: usize,
    seed: usize,
) -> Result<Option<usize>> {
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
            if predicates::orient2(kernel, &points[from], &points[to], &points[point])?
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

fn super_triangle(kernel: &ExactKernel, points: &[Point2], scale: &Real) -> Result<[Point2; 3]> {
    let bounds = Bounds::from_points(kernel, points)?;
    let dx = ExactKernel::sub(&bounds.max_x, &bounds.min_x);
    let dy = ExactKernel::sub(&bounds.max_y, &bounds.min_y);
    let span = if kernel.cmp(&dx, &dy)? == std::cmp::Ordering::Less {
        dy
    } else {
        dx
    };
    let one = ExactKernel::from_i64(1);
    let two = ExactKernel::from_i64(2);
    let radius = ExactKernel::add(&ExactKernel::mul(&span, scale), &one);
    let double_radius = ExactKernel::mul(&radius, &two);
    let mid_x = ExactKernel::div(&ExactKernel::add(&bounds.min_x, &bounds.max_x), &two)?;
    let mid_y = ExactKernel::div(&ExactKernel::add(&bounds.min_y, &bounds.max_y), &two)?;

    Ok([
        Point2::new(
            ExactKernel::sub(&mid_x, &double_radius),
            ExactKernel::sub(&mid_y, &radius),
        ),
        Point2::new(mid_x.clone(), ExactKernel::add(&mid_y, &double_radius)),
        Point2::new(
            ExactKernel::add(&mid_x, &double_radius),
            ExactKernel::sub(&mid_y, &radius),
        ),
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
    fn from_points(kernel: &ExactKernel, points: &[Point2]) -> Result<Self> {
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
            if kernel.cmp(&point.x, &bounds.min_x)? == std::cmp::Ordering::Less {
                bounds.min_x = point.x.clone();
            }
            if kernel.cmp(&point.x, &bounds.max_x)? == std::cmp::Ordering::Greater {
                bounds.max_x = point.x.clone();
            }
            if kernel.cmp(&point.y, &bounds.min_y)? == std::cmp::Ordering::Less {
                bounds.min_y = point.y.clone();
            }
            if kernel.cmp(&point.y, &bounds.max_y)? == std::cmp::Ordering::Greater {
                bounds.max_y = point.y.clone();
            }
        }

        Ok(bounds)
    }
}

fn make_oriented(kernel: &ExactKernel, points: &[Point2], triangle: Triangle) -> Result<Triangle> {
    let sign = predicates::orient2(
        kernel,
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

fn validate_constraint_geometry(
    kernel: &ExactKernel,
    points: &[Point2],
    constraints: &[Constraint],
    reject_unsplit_points: bool,
) -> Result<()> {
    let approximate_points = exact_points_f64(points);
    for first in 0..constraints.len() {
        for second in first + 1..constraints.len() {
            let a = constraints[first];
            let b = constraints[second];
            if (a.from == b.from && a.to == b.to) || (a.from == b.to && a.to == b.from) {
                return Err(Error::InvalidInput {
                    reason: "overlapping constraints are not supported",
                });
            }
            if constraints_share_endpoint(a, b) {
                continue;
            }
            if approximate_points
                .as_ref()
                .is_some_and(|points| !approximate_constraint_bounds_overlap(points, a, b))
            {
                continue;
            }

            // Public constraints are planarized before this check, so any
            // proper crossing or overlap here indicates a remaining PSLG
            // normalization bug. The classification remains exact and
            // predicate-backed.
            let intersection = predicates::segment_intersection(
                kernel,
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

    if !reject_unsplit_points {
        return Ok(());
    }
    for &constraint in constraints {
        for point in 0..points.len() {
            if point == constraint.from || point == constraint.to {
                continue;
            }
            if approximate_points.as_ref().is_some_and(|points| {
                !approximate_point_within_constraint_bounds(points, constraint, point)
            }) {
                continue;
            }
            if predicates::point_on_segment(
                kernel,
                &points[constraint.from],
                &points[constraint.to],
                &points[point],
            )? {
                return Err(Error::InvalidInput {
                    reason: "constraint contains an unsplit point",
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
    kernel: &ExactKernel,
    points: Vec<ExactPoint>,
    constraints: &[Constraint],
    public_constraints: &[Constraint],
) -> Result<std::result::Result<ConstrainedTriangulation, DelaunayTriangulation>> {
    let triangulation = if points.len() >= 32 {
        delaunay_spatial_from_validated_owned(kernel, points)?
    } else {
        delaunay_from_validated_owned(kernel, points)?
    };

    // Exact fast path: if every requested constrained segment already exists
    // in the unconstrained Delaunay triangulation, no topology mutation or
    // legalization is required. The empty-circumcircle property is inherited
    // from the Delaunay triangulation itself.
    if constraints
        .iter()
        .all(|constraint| triangulation_has_edge(triangulation.triangles(), *constraint))
    {
        let (points, triangles) = triangulation.into_parts();
        let constrained = ConstrainedTriangulation::from_parts_with_constraint_edges(
            points,
            public_constraints.to_vec(),
            constraints.to_vec(),
            triangles,
        );
        return Ok(Ok(constrained));
    }

    Ok(Err(triangulation))
}

fn triangulation_has_edge(triangles: &[Triangle], constraint: Constraint) -> bool {
    triangles
        .iter()
        .any(|triangle| triangle_contains_edge(*triangle, constraint.from, constraint.to))
}

fn triangle_contains_edge(triangle: Triangle, first: usize, second: usize) -> bool {
    triangle.contains(&first) && triangle.contains(&second)
}

fn validate_unique_points(kernel: &ExactKernel, points: &[Point2]) -> Result<()> {
    let approximate_points = exact_points_f64(points);
    for i in 0..points.len() {
        for j in i + 1..points.len() {
            if approximate_points
                .as_ref()
                .is_some_and(|points| points[i] != points[j])
            {
                continue;
            }
            if predicates::points_equal(kernel, &points[i], &points[j])? {
                return Err(Error::InvalidInput {
                    reason: "duplicate points are not supported",
                });
            }
        }
    }

    Ok(())
}

pub(crate) fn exact_points_f64(points: &[Point2]) -> Option<Vec<[f64; 2]>> {
    points
        .iter()
        .map(|point| {
            if point.x.exact_rational_ref().is_none() || point.y.exact_rational_ref().is_none() {
                return None;
            }
            let [Some(x), Some(y)] = [point.x.to_f64_lossy(), point.y.to_f64_lossy()] else {
                return None;
            };
            (x.is_finite() && y.is_finite()).then_some([x, y])
        })
        .collect()
}

pub(crate) fn approximate_constraint_bounds_overlap(
    points: &[[f64; 2]],
    first: Constraint,
    second: Constraint,
) -> bool {
    (0..2).all(|axis| {
        points[first.from][axis].max(points[first.to][axis])
            >= points[second.from][axis].min(points[second.to][axis])
            && points[second.from][axis].max(points[second.to][axis])
                >= points[first.from][axis].min(points[first.to][axis])
    })
}

pub(crate) fn approximate_point_within_constraint_bounds(
    points: &[[f64; 2]],
    constraint: Constraint,
    point: usize,
) -> bool {
    (0..2).all(|axis| {
        points[point][axis] >= points[constraint.from][axis].min(points[constraint.to][axis])
            && points[point][axis] <= points[constraint.from][axis].max(points[constraint.to][axis])
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const APPROX: TriangulationContext =
        TriangulationContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);

    fn kernel() -> ExactKernel {
        ExactKernel::new(&APPROX)
    }

    fn approx_delaunay(points: &[ExactPoint]) -> Result<DelaunayTriangulation> {
        delaunay(&APPROX, points).map(TriangulationOutcome::into_value)
    }

    fn approx_delaunay_spatial(points: &[ExactPoint]) -> Result<DelaunayTriangulation> {
        delaunay_spatial(&APPROX, points).map(TriangulationOutcome::into_value)
    }

    fn approx_constrained_delaunay(
        points: &[ExactPoint],
        constraints: &[Constraint],
    ) -> Result<ConstrainedTriangulation> {
        constrained_delaunay(&APPROX, points, constraints).map(TriangulationOutcome::into_value)
    }

    fn p(x: i32, y: i32) -> ExactPoint {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn sparse_triangle_adjacency_matches_shared_half_edges() {
        assert_eq!(
            triangle_neighbors(&[[0, 1, 2], [1, 0, 3]]).unwrap(),
            vec![[Some(1), None, None], [Some(0), None, None]]
        );
    }

    #[test]
    fn sparse_triangle_adjacency_rejects_non_manifold_edges() {
        assert_eq!(
            triangle_neighbors(&[[0, 1, 2], [1, 0, 3], [0, 1, 4]]),
            Err(Error::InvalidInput {
                reason: "Delaunay edge has more than two incident faces",
            })
        );
    }

    #[test]
    fn delaunay_returns_single_exact_triangle() {
        let points = vec![p(0, 0), p(2, 0), p(0, 2)];

        let triangulation = approx_delaunay(&points).unwrap();

        assert_eq!(triangulation.triangles(), &[[0, 1, 2]]);
    }

    #[test]
    fn delaunay_rejects_numeric_duplicates_with_distinct_representations() {
        let left = Real::pi() + Real::e();
        let right = Real::e() + Real::pi();
        assert_ne!(left, right);

        let points = vec![
            Point2::new(left, Real::zero()),
            Point2::new(right, Real::zero()),
            p(0, 1),
        ];
        assert!(matches!(
            approx_delaunay(&points),
            Err(Error::InvalidInput {
                reason: "duplicate points are not supported"
            })
        ));
    }

    #[test]
    fn delaunay_rejects_collinear_triangle() {
        let points = vec![p(0, 0), p(1, 1), p(2, 2)];

        let triangulation = approx_delaunay(&points).unwrap();

        assert!(triangulation.triangles().is_empty());
    }

    #[test]
    fn empty_constraint_set_accepts_collinear_points() {
        let points = vec![p(0, 0), p(1, 1), p(2, 2), p(3, 3), p(4, 4)];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let outcome = constrained_delaunay(&context, &points, &[]).unwrap();

            assert_eq!(outcome.certainty, crate::TriangulationCertainty::Certified);
            assert!(outcome.value.triangles().is_empty());
            outcome.value.validate(&context).unwrap();
        }
    }

    #[test]
    fn delaunay_rejects_duplicate_points() {
        let points = vec![p(0, 0), p(1, 0), p(1, 0), p(0, 1)];

        let error = approx_delaunay(&points).unwrap_err();

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
        let ordinary = approx_delaunay(&points).unwrap();
        let spatial = approx_delaunay_spatial(&points).unwrap();

        ordinary.validate(&APPROX).unwrap();
        spatial.validate(&APPROX).unwrap();
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
        let first = brio_insertion_order(&kernel(), &points).unwrap();
        let second = brio_insertion_order(&kernel(), &points).unwrap();
        let mut sorted = first.clone();
        sorted.sort_unstable();

        assert_eq!(first, second);
        assert_eq!(sorted, (0..points.len()).collect::<Vec<_>>());
    }

    #[test]
    fn delaunay_triangulates_convex_quad_with_one_diagonal() {
        let points = vec![p(0, 0), p(1, 0), p(1, 1), p(0, 1)];

        let triangulation = approx_delaunay(&points).unwrap();

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

        let triangulation = approx_delaunay(&points).unwrap();

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

        let triangulation = approx_delaunay(&points).unwrap();

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

        let triangulation = approx_delaunay(&points).unwrap();

        triangulation.validate(&APPROX).unwrap();
        for index in 0..points.len() {
            assert!(
                triangulation
                    .triangles()
                    .iter()
                    .any(|triangle| triangle.contains(&index))
            );
        }
    }

    #[test]
    fn nearly_collinear_hull_expands_super_triangle_until_complete() {
        let points = vec![
            p(-905, 756),
            p(-1490, 702),
            p(-1611, 691),
            p(-2273, -576),
            p(-385, -1400),
        ];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let outcome = delaunay(&context, &points).unwrap();

            assert_eq!(outcome.certainty, crate::TriangulationCertainty::Certified);
            assert_eq!(outcome.value.triangles().len(), 4);
            assert!(triangulation_has_edge(
                outcome.value.triangles(),
                Constraint::new(0, 2)
            ));
            outcome.value.validate(&context).unwrap();

            let constrained =
                constrained_delaunay(&context, &points, &[Constraint::new(0, 2)]).unwrap();
            assert_eq!(
                constrained.certainty,
                crate::TriangulationCertainty::Certified
            );
            constrained.value.validate(&context).unwrap();
        }
    }

    #[test]
    fn general_constraint_recovers_across_nonconvex_flip_strip() {
        let points = vec![
            p(-2087, -1476),
            p(-2676, -124),
            p(-1936, -2394),
            p(-2561, -766),
            p(-1509, -832),
            p(-2582, -618),
        ];
        let constraint = Constraint::new(1, 2);

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let outcome = constrained_delaunay(&context, &points, &[constraint]).unwrap();

            assert_eq!(outcome.certainty, crate::TriangulationCertainty::Certified);
            assert!(triangulation_has_edge(
                outcome.value.triangles(),
                constraint
            ));
            outcome.value.validate(&context).unwrap();
            outcome
                .value
                .validate_unconstrained_edges_are_delaunay(&context)
                .unwrap();
        }
    }

    #[test]
    fn validation_rejects_locally_legal_incomplete_convex_hull() {
        let points = vec![
            p(-905, 756),
            p(-1490, 702),
            p(-1611, 691),
            p(-2273, -576),
            p(-385, -1400),
        ];
        let incomplete =
            DelaunayTriangulation::from_parts(points, vec![[0, 1, 3], [1, 2, 3], [0, 3, 4]]);

        assert_eq!(
            incomplete.validate(&APPROX).unwrap_err(),
            Error::InvalidInput {
                reason: "triangulation does not cover the convex hull"
            }
        );
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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

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
    fn convex_hull_pslg_keeps_both_sides_of_an_interior_ring() {
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
            Constraint::new(1, 2),
            Constraint::new(2, 3),
            Constraint::new(3, 0),
            Constraint::new(4, 5),
            Constraint::new(5, 6),
            Constraint::new(6, 7),
            Constraint::new(7, 4),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let outcome = constrained_delaunay_convex_hull(&context, &points, &constraints)
                .expect("the bounded PSLG is exactly triangulable");

            assert_eq!(outcome.certainty, crate::TriangulationCertainty::Certified);
            assert_eq!(outcome.value.points(), points);
            assert_eq!(outcome.value.triangles().len(), 10);
            assert!(
                constraints.iter().all(|constraint| triangulation_has_edge(
                    outcome.value.triangles(),
                    *constraint
                ))
            );
            outcome.value.validate(&context).unwrap();
            outcome
                .value
                .validate_unconstrained_edges_are_delaunay(&context)
                .unwrap();
        }
    }

    #[test]
    fn topology_only_convex_hull_pslg_preserves_every_point_and_constraint() {
        let points = vec![
            p(0, 0),
            p(3, 0),
            p(6, 0),
            p(6, 6),
            p(0, 6),
            p(2, 2),
            p(4, 2),
            p(4, 4),
            p(2, 4),
            p(3, 3),
        ];
        let constraints = vec![
            Constraint::new(0, 1),
            Constraint::new(1, 2),
            Constraint::new(2, 3),
            Constraint::new(3, 4),
            Constraint::new(4, 0),
            Constraint::new(5, 6),
            Constraint::new(6, 7),
            Constraint::new(7, 8),
            Constraint::new(8, 5),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let outcome = constrained_triangulation_convex_hull(&context, &points, &constraints)
                .expect("the topology-only PSLG must be exactly triangulable");

            assert_eq!(outcome.certainty, crate::TriangulationCertainty::Certified);
            assert_eq!(outcome.value.len(), 13);
            assert!(points.iter().enumerate().all(|(point, _)| {
                outcome
                    .value
                    .iter()
                    .any(|triangle| triangle.contains(&point))
            }));
            assert!(
                constraints
                    .iter()
                    .all(|constraint| triangulation_has_edge(&outcome.value, *constraint))
            );
        }
    }

    #[test]
    fn convex_hull_pslg_rejects_constraints_that_still_require_planarization() {
        let crossing_points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
        let crossing = [Constraint::new(0, 2), Constraint::new(1, 3)];
        assert_eq!(
            constrained_delaunay_convex_hull(&APPROX, &crossing_points, &crossing).unwrap_err(),
            Error::InvalidInput {
                reason: "properly crossing constraints are not supported"
            }
        );

        let unsplit_points = vec![p(0, 0), p(2, 0), p(1, 0), p(0, 2)];
        assert_eq!(
            constrained_delaunay_convex_hull(&APPROX, &unsplit_points, &[Constraint::new(0, 1)],)
                .unwrap_err(),
            Error::InvalidInput {
                reason: "constraint contains an unsplit point"
            }
        );

        let duplicate = [Constraint::new(0, 1), Constraint::new(1, 0)];
        assert_eq!(
            constrained_delaunay_convex_hull(&APPROX, &crossing_points, &duplicate).unwrap_err(),
            Error::InvalidInput {
                reason: "overlapping constraints are not supported"
            }
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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

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
        let triangulation = ConstrainedTriangulation::from_parts_with_constraint_edges(
            points,
            vec![Constraint::new(0, 2)],
            vec![Constraint::new(0, 2)],
            vec![[0, 1, 3], [1, 2, 3]],
        );

        let error = triangulation.validate(&APPROX).unwrap_err();

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

        let error = triangulation.validate(&APPROX).unwrap_err();

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

        let triangulation = approx_constrained_delaunay(&points, &constraints).unwrap();

        triangulation.validate(&APPROX).unwrap();
        triangulation
            .validate_unconstrained_edges_are_delaunay(&APPROX)
            .unwrap();
    }

    #[test]
    fn rejects_all_collinear_overlapping_constraints_without_area() {
        let points = vec![p(0, 0), p(3, 0), p(1, 0), p(4, 0)];
        let constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

        let error = approx_constrained_delaunay(&points, &constraints).unwrap_err();

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

        let error = approx_constrained_delaunay(&points, &[Constraint::new(0, 3)]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "constraint index out of bounds"
            }
        );
    }
}
