//! Coordinate projections, triangle degeneracy, and coplanar contact reports.
//!
//! A nonzero coordinate-plane orientation witnesses 3D noncollinearity. The
//! degeneracy predicate preserves its decision receipt and searches projections
//! by arithmetic stage. Coplanar contact classification instead assumes the
//! triangles already lie in one plane and selects a projection from the left
//! triangle before running 2D edge and vertex predicates.
//!
//! The coplanar report and several `Option`-returning construction helpers
//! discard certainty, escalation, and unresolved reasons. Their labels can
//! therefore reflect policy-authorized approximation. Local validation and
//! source replay do not restore those receipts or prove 3D coplanarity.
//! Exact coordinate expressions alone do not certify the topology selected
//! using them; consumers must retain the necessary source and policy evidence.

use crate::classify::{SegmentIntersection, TriangleLocation};
use crate::geometry::{Point2, Point3};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, PredicatePolicy, Sign};
use crate::predicates::orient::orient2d_with_policy;
use crate::predicates::ring::ring_area_sign_with_policy;
use crate::predicates::segment::classify_segment_intersection_with_policy;
use crate::predicates::segment_plane::segment_parameter_from_axis_with_policy;
use crate::predicates::triangle::classify_point_triangle_with_policy;
use hyperreal::Real;

/// Coordinate plane selected by dropping one coordinate without rounding.
///
/// The enum alone does not establish that projection preserves the source plane's
/// incidence or that a projected triangle is nondegenerate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarProjection {
    /// Drop z and project to `(x, y)`.
    Xy,
    /// Drop y and project to `(x, z)`.
    Xz,
    /// Drop x and project to `(y, z)`.
    Yz,
}

/// Coarse contact label derived from projected edge and vertex decisions.
///
/// Qualification belongs to the originating predicates; this enum carries none.
/// For nondegenerate coplanar triangles, overlap includes shared edge intervals
/// as well as area. With unchecked degenerate inputs or arbitrary supplied facts,
/// the label is not independently a dimension or positive-area certificate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarTriangleRelation {
    /// The projected closed triangles are disjoint.
    Disjoint,
    /// Boundary contact was found without a higher-priority overlap witness.
    Touching,
    /// A proper crossing, overlap edge, or inside vertex was reported.
    /// For nondegenerate coplanar triangles this includes shared edge intervals.
    Overlapping,
    /// Projection selection or a required predicate was unresolved, or indices
    /// were invalid. The report does not distinguish these reasons.
    Unknown,
}

impl CoplanarTriangleRelation {
    /// Retain every label except `Disjoint` for subsequent graph handling.
    ///
    /// `Unknown` also returns true; this neither resolves it nor constructs a graph.
    pub const fn needs_graph_construction(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Failure of local report-shape checks or field-for-field source replay.
///
/// These errors do not cover every invalid geometric claim: validation does not
/// prove coplanarity, source incidence, or the discarded predicate receipts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarTriangleValidationError {
    /// A decided relation was stored without a projection tag.
    DecidedRelationWithoutProjection,
    /// An unknown relation with no projection retained downstream predicate
    /// facts.
    ProjectionlessUnknownHasFacts,
    /// A decided relation stored an edge vector whose length was not nine.
    MissingEdgeIntersections,
    /// A decided relation did not retain all six vertex/triangle locations.
    MissingVertexLocations,
    /// Retained edge and vertex facts derive a different relation.
    RelationMismatch,
    /// Recomputing the classifier from supplied source triangles did not
    /// reproduce this retained report.
    SourceReplayMismatch,
}

/// Projected triangle contact facts with no retained certainty or source identity.
///
/// Fields are mutable and contain labels rather than proof objects. A classifier
/// may return an incomplete `Unknown` report; even decided reports can contain
/// policy-authorized approximate decisions. Validation checks the documented
/// shape/replay conditions, not an independent geometric certificate.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleClassification {
    /// Coordinate-plane tag, or `None` after invalid indices or unsuccessful
    /// projection selection. The nonzero orientation receipt is not stored.
    pub projection: Option<CoplanarProjection>,
    /// Coarse contact label, without its certainty or unresolved reason.
    pub relation: CoplanarTriangleRelation,
    /// Edge-pair labels in left-edge-major order (01, 12, 20 on each side).
    /// The classifier retains a prefix after an unresolved edge, otherwise nine.
    pub edge_intersections: Vec<SegmentIntersection>,
    /// Right vertices relative to the left triangle, in source order.
    /// The classifier clears all vertex labels if any vertex query is unresolved.
    pub right_vertices_in_left: [Option<TriangleLocation>; 3],
    /// Left vertices relative to the right triangle, in source order.
    /// Classifier-produced Unknown reports leave this array entirely `None`.
    pub left_vertices_in_right: [Option<TriangleLocation>; 3],
}

impl CoplanarTriangleClassification {
    /// Check field presence and agreement with the coarse relation reducer.
    ///
    /// Without a projection, only `Unknown` with no edge or vertex facts is accepted.
    /// With a projection, **every `Unknown` report is accepted without further checks**.
    /// Decided reports require exactly nine edge facts, six present vertex locations,
    /// and agreement with [`derive_coplanar_triangle_relation`]. `Degenerate` locations
    /// are not rejected. Projection validity, fact order, incidence, coplanarity,
    /// and predicate certainty are not checked.
    pub fn validate(&self) -> Result<(), CoplanarTriangleValidationError> {
        if self.projection.is_none() {
            return if self.relation == CoplanarTriangleRelation::Unknown
                && self.edge_intersections.is_empty()
                && self.right_vertices_in_left == [None, None, None]
                && self.left_vertices_in_right == [None, None, None]
            {
                Ok(())
            } else if self.relation == CoplanarTriangleRelation::Unknown {
                Err(CoplanarTriangleValidationError::ProjectionlessUnknownHasFacts)
            } else {
                Err(CoplanarTriangleValidationError::DecidedRelationWithoutProjection)
            };
        }

        if self.relation == CoplanarTriangleRelation::Unknown {
            return Ok(());
        }
        if self.edge_intersections.len() != 9 {
            return Err(CoplanarTriangleValidationError::MissingEdgeIntersections);
        }
        if self.right_vertices_in_left.iter().any(Option::is_none)
            || self.left_vertices_in_right.iter().any(Option::is_none)
        {
            return Err(CoplanarTriangleValidationError::MissingVertexLocations);
        }
        if derive_coplanar_triangle_relation(
            &self.edge_intersections,
            self.right_vertices_in_left,
            self.left_vertices_in_right,
        ) == self.relation
        {
            Ok(())
        } else {
            Err(CoplanarTriangleValidationError::RelationMismatch)
        }
    }

    /// Run local checks, bounds-check both index triples, and compare a fresh report.
    ///
    /// Replay uses the supplied policy and requires derived field equality, including
    /// projection, fact order, and labels. It does not check 3D coplanarity or bind a
    /// unique source identity. Matching `Unknown` reports can pass. Equality cannot
    /// restore discarded certainty or certify earlier approximate decisions; different
    /// representations or vertex orderings need not yield an identical report.
    pub fn validate_against_sources(
        &self,
        points: &[Point3],
        left: [usize; 3],
        right: [usize; 3],
        policy: PredicatePolicy,
    ) -> Result<(), CoplanarTriangleValidationError> {
        self.validate()?;
        if !indices_in_range(points, left) || !indices_in_range(points, right) {
            return Err(CoplanarTriangleValidationError::SourceReplayMismatch);
        }
        let replay = classify_coplanar_triangles_with_policy(points, left, right, policy);
        if self == &replay {
            Ok(())
        } else {
            Err(CoplanarTriangleValidationError::SourceReplayMismatch)
        }
    }
}

/// Collinearity label whose qualification is carried by its predicate outcome.
///
/// The label alone contains no certainty; inspect the enclosing receipt before
/// interpreting a policy-decided zero as exact collinearity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleDegeneracy {
    /// At least one coordinate orientation was decided nonzero; inspect its receipt.
    NonDegenerate,
    /// All three coordinate orientations were decided zero; terminal zeros can
    /// give this label with approximate certainty.
    Degenerate,
}

/// Classify 3D collinearity using XY, XZ, and YZ orientation witnesses.
///
/// All three zero determinants characterize collinearity; any nonzero one
/// suffices for nondegeneracy. Search is batched across projections by stage:
/// dyadic filter, checked-word kernel, rational filter, full rational kernel,
/// then policy-driven Real orientation on unresolved projections. Exact zeros
/// are retained so later stages skip those projections.
///
/// A nonzero witness returns its own receipt, even after an earlier unresolved
/// projection. All-zero completion joins the weakest zero certainty and latest
/// stage, starting at `Exact`; terminal approximate zeros remain qualified.
/// If no nonzero witness exists and some projections remain unresolved, return
/// the first unresolved outcome. This search order differs from the per-plane
/// search in [`choose_coplanar_projection_with_policy`].
#[inline]
pub fn classify_triangle3_degeneracy_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleDegeneracy> {
    let coordinates = [[&a.x, &a.y, &a.z], [&b.x, &b.y, &b.z], [&c.x, &c.y, &c.z]];
    let projections = [[0, 1], [0, 2], [1, 2]].map(|[u, v]| {
        (
            [coordinates[0][u], coordinates[0][v]],
            [coordinates[1][u], coordinates[1][v]],
            [coordinates[2][u], coordinates[2][v]],
        )
    });
    let mut signs = [None; 3];
    let mut zero_certainty = Certainty::Exact;
    let mut zero_stage = Escalation::Exact;
    let mut unknown = None;

    for (a, b, c) in projections.iter().copied() {
        if let Some(sign) = super::orient::orient2d_certified_real_filter(a, b, c) {
            debug_assert_ne!(
                sign,
                Sign::Zero,
                "the floating filter declines exact boundaries"
            );
            return PredicateOutcome::decided(
                TriangleDegeneracy::NonDegenerate,
                Certainty::Exact,
                Escalation::Exact,
            );
        }
    }
    for (index, (a, b, c)) in projections.iter().copied().enumerate() {
        if signs[index].is_none()
            && let Some(sign) = super::orient::orient2d_exact_word_filter(a, b, c)
            && let Some(value) = record_exact_projection_sign(sign, &mut signs[index])
        {
            return PredicateOutcome::decided(value, Certainty::Exact, Escalation::Exact);
        }
    }
    for (index, (a, b, c)) in projections.iter().copied().enumerate() {
        if signs[index].is_none()
            && let Some(sign) = super::orient::orient2d_certified_rational_filter(a, b, c)
        {
            debug_assert_ne!(
                sign,
                Sign::Zero,
                "the certified rational filter declines exact boundaries"
            );
            return PredicateOutcome::decided(
                TriangleDegeneracy::NonDegenerate,
                Certainty::Exact,
                Escalation::Exact,
            );
        }
    }
    for (index, (a, b, c)) in projections.iter().copied().enumerate() {
        if signs[index].is_none()
            && let Some(sign) = super::exact::orient2d_coordinates(a, b, c)
            && let Some(value) = record_exact_projection_sign(sign, &mut signs[index])
        {
            return PredicateOutcome::decided(value, Certainty::Exact, Escalation::Exact);
        }
    }
    for (index, (a, b, c)) in projections.iter().copied().enumerate() {
        if signs[index].is_none() {
            let outcome = super::orient::orient2d_real_coordinates(a, b, c, policy);
            match outcome {
                PredicateOutcome::Decided {
                    value: Sign::Positive | Sign::Negative,
                    certainty,
                    stage,
                } => {
                    return PredicateOutcome::decided(
                        TriangleDegeneracy::NonDegenerate,
                        certainty,
                        stage,
                    );
                }
                PredicateOutcome::Decided {
                    value: Sign::Zero,
                    certainty,
                    stage,
                } => {
                    signs[index] = Some(Sign::Zero);
                    zero_certainty = weaker_certainty(zero_certainty, certainty);
                    zero_stage = later_stage(zero_stage, stage);
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    unknown.get_or_insert((needed, stage));
                }
            }
        }
    }

    if signs.into_iter().all(|sign| sign == Some(Sign::Zero)) {
        PredicateOutcome::decided(TriangleDegeneracy::Degenerate, zero_certainty, zero_stage)
    } else {
        let (needed, stage) = unknown.expect(
            "every projection without a decided zero or nonzero sign records its uncertainty",
        );
        PredicateOutcome::unknown(needed, stage)
    }
}

/// Remember an exact zero, or return a sufficient nondegeneracy witness.
///
/// A nonzero sign leaves the slot untouched because its caller returns at once.
fn record_exact_projection_sign(sign: Sign, slot: &mut Option<Sign>) -> Option<TriangleDegeneracy> {
    if sign == Sign::Zero {
        *slot = Some(Sign::Zero);
        None
    } else {
        Some(TriangleDegeneracy::NonDegenerate)
    }
}

/// Join zero-witness qualification, preferring Approximate, then Filtered.
const fn weaker_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        (Certainty::Exact, Certainty::Exact) => Certainty::Exact,
    }
}

/// Retain the later zero-witness stage according to the local escalation rank.
const fn later_stage(left: Escalation, right: Escalation) -> Escalation {
    if escalation_rank(left) >= escalation_rank(right) {
        left
    } else {
        right
    }
}

/// Order Structural, Filter, Exact, Refined, and Undecided for receipt joins.
const fn escalation_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

/// Classify indexed triangles under the caller's existing coplanarity assumption.
///
/// Out-of-range indices return a projectionless, fact-free `Unknown`, without a
/// separate invalid-input reason. Repeated indices are allowed. Valid indices
/// forward to [`classify_coplanar_triangle_points_with_policy`], including its
/// projection, degeneracy, and receipt limitations.
pub fn classify_coplanar_triangles_with_policy(
    points: &[Point3],
    left: [usize; 3],
    right: [usize; 3],
    policy: PredicatePolicy,
) -> CoplanarTriangleClassification {
    if !indices_in_range(points, left) || !indices_in_range(points, right) {
        return CoplanarTriangleClassification {
            projection: None,
            relation: CoplanarTriangleRelation::Unknown,
            edge_intersections: Vec::new(),
            right_vertices_in_left: [None, None, None],
            left_vertices_in_right: [None, None, None],
        };
    }
    let left_points = [&points[left[0]], &points[left[1]], &points[left[2]]];
    let right_points = [&points[right[0]], &points[right[1]], &points[right[2]]];
    classify_coplanar_triangle_points_with_policy(left_points, right_points, policy)
}

/// Reduce projected edge and vertex contacts for already-coplanar triangles.
///
/// Coplanarity is assumed, not checked. Choose a projection using only the left
/// triangle; the right triangle's nondegeneracy is not independently validated.
/// Evaluate all nine edge pairs in left-edge-major order, each triangle using
/// edges 0→1, 1→2, 2→0. An unresolved edge returns `Unknown` with the preceding
/// edge facts. A contact witness does not short-circuit the remaining edges.
///
/// Next classify all right vertices against the left triangle, then all left
/// vertices against the right. If any location is unresolved, retain the edges
/// but discard all six vertex locations. Otherwise proper crossings, overlap
/// edges, or inside vertices take priority over boundary touches. Degenerate
/// vertex locations are ignored; a degenerate right triangle can consequently
/// receive `Overlapping` without an area intersection. All child receipts and
/// unresolved reasons are discarded from this report.
pub fn classify_coplanar_triangle_points_with_policy(
    left: [&Point3; 3],
    right: [&Point3; 3],
    policy: PredicatePolicy,
) -> CoplanarTriangleClassification {
    let Some(projection) = choose_coplanar_projection_with_policy(left, policy) else {
        return CoplanarTriangleClassification {
            projection: None,
            relation: CoplanarTriangleRelation::Unknown,
            edge_intersections: Vec::new(),
            right_vertices_in_left: [None, None, None],
            left_vertices_in_right: [None, None, None],
        };
    };

    let left2 = project_triangle3(left, projection);
    let right2 = project_triangle3(right, projection);
    let mut edge_intersections = Vec::with_capacity(9);
    let mut saw_touch = false;
    let mut saw_overlap = false;

    for left_edge in triangle_edges2(&left2) {
        for right_edge in triangle_edges2(&right2) {
            match classify_segment_intersection_with_policy(
                left_edge[0],
                left_edge[1],
                right_edge[0],
                right_edge[1],
                policy,
            ) {
                PredicateOutcome::Decided { value, .. } => {
                    if value.is_proper_crossing() || value.has_positive_length_overlap() {
                        saw_overlap = true;
                    } else if value.is_endpoint_touch() {
                        saw_touch = true;
                    }
                    edge_intersections.push(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    return unknown_with_projection(projection, edge_intersections);
                }
            }
        }
    }

    let right_vertices_in_left = classify_vertices_in_triangle(&left2, &right2, policy);
    let left_vertices_in_right = classify_vertices_in_triangle(&right2, &left2, policy);

    finish_coplanar_triangle_classification(
        projection,
        edge_intersections,
        right_vertices_in_left,
        left_vertices_in_right,
        saw_touch,
        saw_overlap,
    )
}

/// Combine complete vertex labels with the caller's accumulated edge flags.
///
/// Any missing vertex label returns `Unknown` and drops all vertex labels.
/// Otherwise inside beats boundary contact, while outside/degenerate add nothing.
/// The edge flags are trusted; this does not rescan or validate the edge vector.
fn finish_coplanar_triangle_classification(
    projection: CoplanarProjection,
    edge_intersections: Vec<SegmentIntersection>,
    right_vertices_in_left: [Option<TriangleLocation>; 3],
    left_vertices_in_right: [Option<TriangleLocation>; 3],
    mut saw_touch: bool,
    mut saw_overlap: bool,
) -> CoplanarTriangleClassification {
    if right_vertices_in_left.iter().any(Option::is_none)
        || left_vertices_in_right.iter().any(Option::is_none)
    {
        return unknown_with_projection(projection, edge_intersections);
    }

    for location in right_vertices_in_left
        .iter()
        .chain(left_vertices_in_right.iter())
        .flatten()
    {
        match location {
            TriangleLocation::Inside => saw_overlap = true,
            TriangleLocation::OnEdge | TriangleLocation::OnVertex => saw_touch = true,
            TriangleLocation::Degenerate | TriangleLocation::Outside => {}
        }
    }

    let relation = if saw_overlap {
        CoplanarTriangleRelation::Overlapping
    } else if saw_touch {
        CoplanarTriangleRelation::Touching
    } else {
        CoplanarTriangleRelation::Disjoint
    };

    CoplanarTriangleClassification {
        projection: Some(projection),
        relation,
        edge_intersections,
        right_vertices_in_left,
        left_vertices_in_right,
    }
}

/// Choose the first XY, XZ, or YZ orientation decided nonzero under `policy`.
///
/// Zero and unresolved outcomes both advance to the next plane. `None` conflates
/// collinearity with unresolved projections; `Some` discards the witness's
/// certainty and stage, so it need not denote an exact nondegeneracy proof.
pub fn choose_coplanar_projection_with_policy(
    triangle: [&Point3; 3],
    policy: PredicatePolicy,
) -> Option<CoplanarProjection> {
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let projected = project_triangle3(triangle, projection);
        let outcome = orient2d_with_policy(&projected[0], &projected[1], &projected[2], policy);
        if matches!(outcome.value(), Some(Sign::Positive | Sign::Negative)) {
            return Some(projection);
        }
    }
    None
}

/// Clone two exact coordinates in the selected plane, dropping the third.
///
/// This supplies no proof that projection preserves the source's incidence.
pub fn project_point3(point: &Point3, projection: CoplanarProjection) -> Point2 {
    match projection {
        CoplanarProjection::Xy => Point2::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => Point2::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => Point2::new(point.y.clone(), point.z.clone()),
    }
}

/// Project three points in their supplied order, without validating a triangle.
pub fn project_triangle3(points: [&Point3; 3], projection: CoplanarProjection) -> [Point2; 3] {
    [
        project_point3(points[0], projection),
        project_point3(points[1], projection),
        project_point3(points[2], projection),
    ]
}

/// Return the sign of signed doubled projected area, preserving its receipt.
///
/// Allocate the projected ring and delegate to the ring-area predicate. The
/// shoelace quantity is algebraic signed area for a self-intersecting ring,
/// not generally filled-set area or Euclidean area in the original 3D plane.
/// No validity or nondegeneracy check is made on the chosen projection.
pub fn projected_polygon_area2_sign_with_policy(
    points: &[Point3],
    projection: CoplanarProjection,
    policy: PredicatePolicy,
) -> crate::PredicateOutcome<Sign> {
    let ring = points
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();
    ring_area_sign_with_policy(&ring, policy)
}

/// Construct the exact signed doubled shoelace area in the selected plane.
///
/// The cyclic sum is zero for fewer than three points. It is neither an absolute
/// area nor a 3D Euclidean area; self-intersections contribute algebraically.
/// This expression carries no sign-decision receipt.
pub fn projected_polygon_area2_value(points: &[Point3], projection: CoplanarProjection) -> Real {
    if points.len() < 3 {
        return Real::from(0);
    }
    let mut sum = Real::from(0);
    for index in 0..points.len() {
        let current = project_point3(&points[index], projection);
        let next = project_point3(&points[(index + 1) % points.len()], projection);
        sum += current.x * next.y.clone() - current.y * next.x;
    }
    sum
}

/// Negate signed projected area only when comparison with zero decides `Less`.
///
/// `Equal` and `Greater` retain the original expression; unresolved comparison
/// returns `None`. The comparison receipt is discarded. In particular, an
/// approximate `Equal` can retain a negative exact expression, so this helper
/// alone does not certify a nonnegative mathematical absolute value.
pub fn projected_polygon_area2_abs_value_with_policy(
    points: &[Point3],
    projection: CoplanarProjection,
    policy: PredicatePolicy,
) -> Option<Real> {
    let signed = projected_polygon_area2_value(points, projection);
    match crate::predicates::order::compare_reals_with_policy(&signed, &Real::from(0), policy)
        .value()?
    {
        core::cmp::Ordering::Less => Some(Real::from(0) - &signed),
        core::cmp::Ordering::Equal | core::cmp::Ordering::Greater => Some(signed),
    }
}

/// Construct `(a + b) / 2` coordinatewise with an exact rational half.
///
/// No incidence or topology receipt is attached to the resulting point.
pub fn midpoint3(a: &Point3, b: &Point3) -> Point3 {
    let half = (Real::from(1) / &Real::from(2)).expect("2 is nonzero");
    Point3::new(
        (a.x.clone() + &b.x) * &half,
        (a.y.clone() + &b.y) * &half,
        (a.z.clone() + &b.z) * &half,
    )
}

/// Subtract projected coordinates to represent `to - from` as a `Point2`.
///
/// The vector is not normalized; a collapsed projection gives the zero vector.
pub fn projected_vector3(from: &Point3, to: &Point3, projection: CoplanarProjection) -> Point2 {
    let from = project_point3(from, projection);
    let to = project_point3(to, projection);
    Point2::new(to.x - &from.x, to.y - &from.y)
}

/// Compare counterclockwise turns from `base` using cross/dot sign decisions.
///
/// For nonzero vectors and exact signs, half-plane buckets order turns from zero
/// to a full revolution, with cross-product order inside a bucket. Equal turns
/// return false. Nonzero inputs are required for the angular interpretation but
/// are not checked. Evaluate the left bucket before the right; any unresolved
/// comparison returns `None`. Successful comparison receipts are discarded,
/// so a policy-authorized terminal result is not an unqualified exact order.
pub fn ccw_projected_turn_less_with_policy(
    base: &Point2,
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> Option<bool> {
    let left_bucket = ccw_turn_bucket(base, left, policy)?;
    let right_bucket = ccw_turn_bucket(base, right, policy)?;
    if left_bucket != right_bucket {
        return Some(left_bucket < right_bucket);
    }
    match crate::predicates::order::compare_reals_with_policy(
        &cross2(left, right),
        &Real::from(0),
        policy,
    )
    .value()?
    {
        core::cmp::Ordering::Greater => Some(true),
        core::cmp::Ordering::Less | core::cmp::Ordering::Equal => Some(false),
    }
}

/// Project the query and triangle, then preserve the 2D location outcome.
///
/// The chosen projection's validity and 3D coplanarity are not checked. The
/// returned receipt qualifies only the delegated projected classification.
pub fn classify_point_projected_triangle3_with_policy(
    point: &Point3,
    triangle: [&Point3; 3],
    projection: CoplanarProjection,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleLocation> {
    let query = project_point3(point, projection);
    let a = project_point3(triangle[0], projection);
    let b = project_point3(triangle[1], projection);
    let c = project_point3(triangle[2], projection);
    classify_point_triangle_with_policy(&a, &b, &c, &query, policy)
}

/// Interpolate the original 3D segment at its projected line-intersection ratio.
///
/// No unit-interval bound or incidence with the original 3D line is checked:
/// projected lines may intersect even when their 3D supports are skew. Callers
/// must establish the intended crossing and projection preconditions separately.
/// `None` inherits the parameter helper's merged error/unknown outcomes, and
/// successful division certainty is not retained.
pub fn intersect_segment_with_projected_line3_with_policy(
    segment_start: &Point3,
    segment_end: &Point3,
    line_start: &Point3,
    line_end: &Point3,
    projection: CoplanarProjection,
    policy: PredicatePolicy,
) -> Option<Point3> {
    let parameter = projected_line_parameter3_with_policy(
        segment_start,
        segment_end,
        line_start,
        line_end,
        projection,
        policy,
    )?;
    Some(interpolate_projected_point3(
        segment_start,
        segment_end,
        &parameter,
    ))
}

/// Return the exact signed 2D orientation determinant.
///
/// This is the raw determinant value behind the orientation predicate. Callers
/// must still use [`crate::orient2`] or another certified sign classifier for
/// topology decisions; the value helper exists for exact construction
/// parameters that are consumed only after predicates have selected the
/// combinatorial case, preserving the predicate/construction boundary.
pub fn orient2d_value(a: &Point2, b: &Point2, c: &Point2) -> Real {
    (b.x.clone() - &a.x) * (c.y.clone() - &a.y) - (b.y.clone() - &a.y) * (c.x.clone() - &a.x)
}

/// Recover an affine parameter from the first usable projected coordinate.
///
/// Try projected x, then projected y on any `None` (including an unresolved
/// comparison or division error). Successful receipts and failure reasons are
/// discarded. The helper checks neither agreement between coordinates nor
/// incidence, a unit-interval bound, or membership in the original 3D segment.
/// Those conditions require separate evidence before consuming the parameter.
pub fn projected_segment_parameter3_with_policy(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    projection: CoplanarProjection,
    policy: PredicatePolicy,
) -> Option<Real> {
    let point = project_point3(point, projection);
    let start = project_point3(start, projection);
    let end = project_point3(end, projection);
    segment_parameter_from_axis_with_policy(&point.x, &start.x, &end.x, policy)
        .or_else(|| segment_parameter_from_axis_with_policy(&point.y, &start.y, &end.y, policy))
}

/// Construct `d0 / (d0 - d1)` from projected line-orientation determinants.
///
/// `d0` and `d1` evaluate the projected segment endpoints against the projected
/// supporting line. Division errors and unresolved outcomes collapse to `None`;
/// a successful receipt is discarded. No unit-interval bound, nondegenerate
/// line, or 3D crossing is independently validated. Callers must establish the
/// intended topology before consuming this construction ratio.
pub fn projected_line_parameter3_with_policy(
    segment_start: &Point3,
    segment_end: &Point3,
    line_start: &Point3,
    line_end: &Point3,
    projection: CoplanarProjection,
    policy: PredicatePolicy,
) -> Option<Real> {
    let a = project_point3(line_start, projection);
    let b = project_point3(line_end, projection);
    let p0 = project_point3(segment_start, projection);
    let p1 = project_point3(segment_end, projection);
    let d0 = orient2d_value(&a, &b, &p0);
    let d1 = orient2d_value(&a, &b, &p1);
    let denominator = d0.clone() - &d1;
    crate::predicates::order::divide_real_with_policy(&d0, &denominator, policy)
        .ok()?
        .value()
}

/// Evaluate `(1 - t) * start + t * end` in all original 3D coordinates.
///
/// Despite the name, this performs no projection and allows extrapolation.
fn interpolate_projected_point3(start: &Point3, end: &Point3, t: &Real) -> Point3 {
    let one_minus_t = Real::from(1) - t;
    Point3::new(
        start.x.clone() * &one_minus_t + end.x.clone() * t,
        start.y.clone() * &one_minus_t + end.y.clone() * t,
        start.z.clone() * &one_minus_t + end.z.clone() * t,
    )
}

/// Select the first or second half-turn from cross sign, then collinear dot sign.
///
/// For nonzero vectors and exact decisions, bucket 0 covers angles from zero
/// up to but excluding pi; bucket 1 covers pi up to but excluding a full turn.
/// Zero vectors are not rejected. Unknown returns `None`; receipts are dropped.
fn ccw_turn_bucket(base: &Point2, candidate: &Point2, policy: PredicatePolicy) -> Option<u8> {
    match crate::predicates::order::compare_reals_with_policy(
        &cross2(base, candidate),
        &Real::from(0),
        policy,
    )
    .value()?
    {
        core::cmp::Ordering::Greater => Some(0),
        core::cmp::Ordering::Less => Some(1),
        core::cmp::Ordering::Equal => {
            match crate::predicates::order::compare_reals_with_policy(
                &dot2(base, candidate),
                &Real::from(0),
                policy,
            )
            .value()?
            {
                core::cmp::Ordering::Greater | core::cmp::Ordering::Equal => Some(0),
                core::cmp::Ordering::Less => Some(1),
            }
        }
    }
}

/// Construct the exact planar determinant `left.x * right.y - left.y * right.x`.
fn cross2(left: &Point2, right: &Point2) -> Real {
    left.x.clone() * &right.y - left.y.clone() * &right.x
}

/// Construct the exact Euclidean dot product of the two coordinate vectors.
fn dot2(left: &Point2, right: &Point2) -> Real {
    left.x.clone() * &right.x + left.y.clone() * &right.y
}

/// Reduce supplied labels with overlap taking priority over touch and disjoint.
///
/// Proper crossings, overlap edges, and inside vertices select overlap; boundary
/// contacts select touch. Missing, outside, and degenerate vertex labels add
/// nothing. This reducer never returns `Unknown`: even empty edges and absent
/// vertices reduce to `Disjoint`. It validates neither completeness nor sources.
/// The segment overlap helper also accepts `Identical` for zero-length segments;
/// therefore arbitrary fact sets do not establish positive area or length.
pub fn derive_coplanar_triangle_relation(
    edge_intersections: &[SegmentIntersection],
    right_vertices_in_left: [Option<TriangleLocation>; 3],
    left_vertices_in_right: [Option<TriangleLocation>; 3],
) -> CoplanarTriangleRelation {
    let mut saw_touch = false;
    let mut saw_overlap = false;
    for relation in edge_intersections {
        if relation.is_proper_crossing() || relation.has_positive_length_overlap() {
            saw_overlap = true;
        } else if relation.is_endpoint_touch() {
            saw_touch = true;
        }
    }
    for location in right_vertices_in_left
        .iter()
        .chain(left_vertices_in_right.iter())
        .flatten()
    {
        match location {
            TriangleLocation::Inside => saw_overlap = true,
            TriangleLocation::OnEdge | TriangleLocation::OnVertex => saw_touch = true,
            TriangleLocation::Degenerate | TriangleLocation::Outside => {}
        }
    }
    if saw_overlap {
        CoplanarTriangleRelation::Overlapping
    } else if saw_touch {
        CoplanarTriangleRelation::Touching
    } else {
        CoplanarTriangleRelation::Disjoint
    }
}

/// Check all three indices against the point count, allowing repeated indices.
fn indices_in_range(points: &[Point3], indices: [usize; 3]) -> bool {
    indices.iter().all(|&index| index < points.len())
}

/// Borrow directed edges 0→1, 1→2, and 2→0 in report traversal order.
fn triangle_edges2(tri: &[Point2; 3]) -> [[&Point2; 2]; 3] {
    [[&tri[0], &tri[1]], [&tri[1], &tri[2]], [&tri[2], &tri[0]]]
}

/// Classify all three queries in order, retaining only their optional labels.
///
/// An unresolved query does not prevent later queries. Certainty, escalation,
/// and unresolved reasons are discarded individually.
fn classify_vertices_in_triangle(
    triangle: &[Point2; 3],
    query: &[Point2; 3],
    policy: PredicatePolicy,
) -> [Option<TriangleLocation>; 3] {
    [
        classify_point_triangle_with_policy(
            &triangle[0],
            &triangle[1],
            &triangle[2],
            &query[0],
            policy,
        )
        .value(),
        classify_point_triangle_with_policy(
            &triangle[0],
            &triangle[1],
            &triangle[2],
            &query[1],
            policy,
        )
        .value(),
        classify_point_triangle_with_policy(
            &triangle[0],
            &triangle[1],
            &triangle[2],
            &query[2],
            policy,
        )
        .value(),
    ]
}

/// Package a projection and supplied edge facts as `Unknown` with no vertex facts.
///
/// This constructor neither checks the edge-prefix shape nor adds qualification.
fn unknown_with_projection(
    projection: CoplanarProjection,
    edge_intersections: Vec<SegmentIntersection>,
) -> CoplanarTriangleClassification {
    CoplanarTriangleClassification {
        projection: Some(projection),
        relation: CoplanarTriangleRelation::Unknown,
        edge_intersections,
        right_vertices_in_left: [None, None, None],
        left_vertices_in_right: [None, None, None],
    }
}

/// Finite controls for projections, degeneracy receipts, contact reports, and ratios.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::Real;
    use crate::test_support::exact_normal_positive;
    use hyperreal::Rational;

    /// Terminal approximation policy used alongside strict controls in this module.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Keep the normalized trigonometric identity control and return a harder zero.
    ///
    /// The returned angle-addition expression supplies unresolved/terminal fixtures;
    /// the original squared-sine/cosine identity is separately asserted normalized.
    fn terminally_unresolved_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let original = &sine * &sine + &cosine * &cosine - Real::one();
        // Preserve the original identity as a newly certified proof case.
        assert_eq!(
            original.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        // A separate angle-addition identity retains the terminal-policy control.
        crate::test_support::terminally_unresolved_zero()
    }

    /// Construct an exact integer-coordinate 3D point for the fixtures.
    fn p3(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    /// Check nondegeneracy in each coordinate plane and a collinear integer triple.
    #[test]
    fn triangle3_degeneracy_uses_projected_orientations() {
        let xy =
            crate::classify_triangle3_degeneracy(&p3(0, 0, 0), &p3(1, 0, 0), &p3(0, 1, 0), APPROX);
        assert_eq!(xy.value(), Some(TriangleDegeneracy::NonDegenerate));

        let xz =
            crate::classify_triangle3_degeneracy(&p3(0, 0, 0), &p3(1, 0, 0), &p3(0, 0, 1), APPROX);
        assert_eq!(xz.value(), Some(TriangleDegeneracy::NonDegenerate));

        let yz =
            crate::classify_triangle3_degeneracy(&p3(0, 0, 0), &p3(0, 1, 0), &p3(0, 0, 1), APPROX);
        assert_eq!(yz.value(), Some(TriangleDegeneracy::NonDegenerate));

        let degenerate =
            crate::classify_triangle3_degeneracy(&p3(0, 0, 0), &p3(1, 1, 1), &p3(2, 2, 2), APPROX);
        assert_eq!(degenerate.value(), Some(TriangleDegeneracy::Degenerate));
    }

    /// Require Exact/Exact nondegeneracy for wide integer projections under both policies.
    #[test]
    fn triangle3_degeneracy_filters_rationals_beyond_word_kernel_exactly() {
        let word = Rational::new(i64::MAX);
        let square = &word * &word;
        let wide = Real::from(&square * &Rational::new(8));
        let zero = Real::zero();
        let a = Point3::new(zero.clone(), zero.clone(), zero.clone());
        let b = Point3::new(wide.clone(), zero.clone(), zero.clone());
        let c = Point3::new(zero.clone(), wide, zero);

        for policy in [PredicatePolicy::STRICT, APPROX] {
            assert_eq!(
                crate::classify_triangle3_degeneracy(&a, &b, &c, policy),
                PredicateOutcome::decided(
                    TriangleDegeneracy::NonDegenerate,
                    Certainty::Exact,
                    Escalation::Exact
                )
            );
        }
    }

    /// Check tiny dyadic filter declines and exact nondegeneracy under both policies.
    #[test]
    fn triangle3_degeneracy_replays_nonzero_underflowed_dyadic_projection() {
        let denominator = Rational::new(2)
            .powi(2048.into())
            .expect("the test power fits the exact eager budget");
        let tiny = Real::new(Rational::one() / denominator);
        let zero = Real::zero();
        let one = Real::one();
        let a = Point3::new(zero.clone(), zero.clone(), zero.clone());
        let b = Point3::new(one, zero.clone(), zero.clone());
        let c = Point3::new(zero.clone(), tiny, zero);
        let xy = ([&a.x, &a.y], [&b.x, &b.y], [&c.x, &c.y]);

        assert_eq!(
            super::super::orient::orient2d_certified_real_filter(xy.0, xy.1, xy.2),
            None
        );
        assert_eq!(
            super::super::orient::orient2d_exact_word_filter(xy.0, xy.1, xy.2),
            None
        );
        assert_eq!(
            super::super::orient::orient2d_certified_rational_filter(xy.0, xy.1, xy.2),
            None
        );
        for policy in [PredicatePolicy::STRICT, APPROX] {
            assert_eq!(
                classify_triangle3_degeneracy_with_policy(&a, &b, &c, policy),
                PredicateOutcome::decided(
                    TriangleDegeneracy::NonDegenerate,
                    Certainty::Exact,
                    Escalation::Exact,
                )
            );
        }
    }

    /// Check wide collinearity and the exact-zero versus nonzero slot bookkeeping.
    #[test]
    fn triangle3_degeneracy_covers_exact_boundary_and_projection_bookkeeping() {
        let word = Rational::new(i64::MAX);
        let square = &word * &word;
        let wide = &square * &Rational::new(8);
        let two_wide = &wide * &Rational::new(2);
        let four_wide = &wide * &Rational::new(4);
        let zero = Real::zero();
        let a = Point3::new(zero.clone(), zero.clone(), zero.clone());
        let b = Point3::new(
            Real::from(wide.clone()),
            Real::from(two_wide.clone()),
            zero.clone(),
        );
        let c = Point3::new(Real::from(two_wide), Real::from(four_wide), zero);

        assert_eq!(
            classify_triangle3_degeneracy_with_policy(&a, &b, &c, PredicatePolicy::STRICT).value(),
            Some(TriangleDegeneracy::Degenerate)
        );

        let mut slot = None;
        assert_eq!(record_exact_projection_sign(Sign::Zero, &mut slot), None);
        assert_eq!(slot, Some(Sign::Zero));
        assert_eq!(
            record_exact_projection_sign(Sign::Positive, &mut slot),
            Some(TriangleDegeneracy::NonDegenerate)
        );
    }

    /// Check missing vertex labels produce Unknown and exercise receipt-join ordering.
    ///
    /// These assertions check the relation, not retention of known vertex labels.
    #[test]
    fn coplanar_completion_preserves_unknown_vertex_classifications_and_all_stage_ranks() {
        let outside = Some(TriangleLocation::Outside);
        let unknown_right = finish_coplanar_triangle_classification(
            CoplanarProjection::Xy,
            Vec::new(),
            [None, outside, outside],
            [outside; 3],
            false,
            false,
        );
        assert_eq!(unknown_right.relation, CoplanarTriangleRelation::Unknown);

        let unknown_left = finish_coplanar_triangle_classification(
            CoplanarProjection::Xy,
            Vec::new(),
            [outside; 3],
            [outside, None, outside],
            false,
            false,
        );
        assert_eq!(unknown_left.relation, CoplanarTriangleRelation::Unknown);

        assert_eq!(escalation_rank(Escalation::Structural), 0);
        assert_eq!(escalation_rank(Escalation::Filter), 1);
        assert_eq!(escalation_rank(Escalation::Exact), 2);
        assert_eq!(escalation_rank(Escalation::Refined), 3);
        assert_eq!(escalation_rank(Escalation::Undecided), 4);
        assert_eq!(
            weaker_certainty(Certainty::Exact, Certainty::Filtered),
            Certainty::Filtered
        );
    }

    /// Require strict uncertainty and an approximate degeneracy receipt for symbolic zero.
    #[test]
    fn triangle3_degeneracy_preserves_terminal_approximation_evidence() {
        let terminal_zero = terminally_unresolved_zero();
        let a = p3(0, 0, 0);
        let b = p3(1, 1, 0);
        let c = Point3::new(Real::from(2), Real::from(2) + terminal_zero, Real::zero());

        assert!(matches!(
            crate::classify_triangle3_degeneracy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::classify_triangle3_degeneracy(&a, &b, &c, APPROX),
            PredicateOutcome::Decided {
                value: TriangleDegeneracy::Degenerate,
                certainty: Certainty::Approximate,
                ..
            }
        ));
    }

    /// Check three planar contact labels, their replay, and rejection of changed sources.
    #[test]
    fn coplanar_triangle_classifier_distinguishes_disjoint_touching_and_overlap() {
        let disjoint_points = [
            p3(0, 0, 0),
            p3(4, 0, 0),
            p3(0, 4, 0),
            p3(5, 5, 0),
            p3(7, 5, 0),
            p3(5, 7, 0),
        ];
        let touching_points = [
            p3(0, 0, 0),
            p3(4, 0, 0),
            p3(0, 4, 0),
            p3(4, 0, 0),
            p3(6, 0, 0),
            p3(4, 2, 0),
        ];
        let overlapping_points = [
            p3(0, 0, 0),
            p3(4, 0, 0),
            p3(0, 4, 0),
            p3(1, 1, 0),
            p3(5, 1, 0),
            p3(1, 5, 0),
        ];

        let disjoint =
            crate::classify_coplanar_triangles(&disjoint_points, [0, 1, 2], [3, 4, 5], APPROX);
        let touching =
            crate::classify_coplanar_triangles(&touching_points, [0, 1, 2], [3, 4, 5], APPROX);
        let overlapping =
            crate::classify_coplanar_triangles(&overlapping_points, [0, 1, 2], [3, 4, 5], APPROX);

        assert_eq!(disjoint.relation, CoplanarTriangleRelation::Disjoint);
        assert_eq!(touching.relation, CoplanarTriangleRelation::Touching);
        assert_eq!(overlapping.relation, CoplanarTriangleRelation::Overlapping);
        disjoint
            .validate_against_sources(&disjoint_points, [0, 1, 2], [3, 4, 5], APPROX)
            .unwrap();
        touching
            .validate_against_sources(&touching_points, [0, 1, 2], [3, 4, 5], APPROX)
            .unwrap();
        overlapping
            .validate_against_sources(&overlapping_points, [0, 1, 2], [3, 4, 5], APPROX)
            .unwrap();
        assert_eq!(
            disjoint.validate_against_sources(&overlapping_points, [0, 1, 2], [3, 4, 5], APPROX,),
            Err(CoplanarTriangleValidationError::SourceReplayMismatch)
        );
    }

    /// Check exact half parameters from a projected point and projected line crossing.
    #[test]
    fn projected_parameters_preserve_exact_affine_ratios() {
        let start = p3(0, 0, 0);
        let end = p3(4, 0, 0);
        let midpoint = p3(2, 0, 0);
        let half = (Real::from(1) / &Real::from(2)).unwrap();

        assert_eq!(
            crate::projected_segment_parameter3(
                &midpoint,
                &start,
                &end,
                CoplanarProjection::Xy,
                APPROX
            ),
            Some(half.clone())
        );

        let crossing = crate::projected_line_parameter3(
            &p3(0, -2, 0),
            &p3(0, 2, 0),
            &p3(-1, 0, 0),
            &p3(1, 0, 0),
            CoplanarProjection::Xy,
            APPROX,
        );
        assert_eq!(crossing, Some(half));
    }

    /// Check plane choices, coordinate order, and valid Unknown report shapes.
    #[test]
    fn projection_helpers_cover_all_coordinate_planes_and_degenerate_cases() {
        let xy = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
        let xz = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 0, 1)];
        let yz = [p3(0, 0, 0), p3(0, 1, 0), p3(0, 0, 1)];
        let line = [p3(0, 0, 0), p3(1, 1, 1), p3(2, 2, 2)];

        assert_eq!(
            choose_coplanar_projection_with_policy([&xy[0], &xy[1], &xy[2]], APPROX),
            Some(CoplanarProjection::Xy)
        );
        assert_eq!(
            choose_coplanar_projection_with_policy([&xz[0], &xz[1], &xz[2]], APPROX),
            Some(CoplanarProjection::Xz)
        );
        assert_eq!(
            choose_coplanar_projection_with_policy([&yz[0], &yz[1], &yz[2]], APPROX),
            Some(CoplanarProjection::Yz)
        );
        assert_eq!(
            choose_coplanar_projection_with_policy([&line[0], &line[1], &line[2]], APPROX),
            None
        );

        let point = p3(2, 3, 5);
        assert_eq!(
            project_point3(&point, CoplanarProjection::Xy),
            Point2::new(Real::from(2), Real::from(3))
        );
        assert_eq!(
            project_point3(&point, CoplanarProjection::Xz),
            Point2::new(Real::from(2), Real::from(5))
        );
        assert_eq!(
            project_point3(&point, CoplanarProjection::Yz),
            Point2::new(Real::from(3), Real::from(5))
        );
        assert_eq!(
            project_triangle3([&yz[0], &yz[1], &yz[2]], CoplanarProjection::Yz),
            [
                Point2::new(Real::from(0), Real::from(0)),
                Point2::new(Real::from(1), Real::from(0)),
                Point2::new(Real::from(0), Real::from(1)),
            ]
        );

        let classification = classify_coplanar_triangle_points_with_policy(
            [&line[0], &line[1], &line[2]],
            [&xy[0], &xy[1], &xy[2]],
            APPROX,
        );
        assert_eq!(classification.relation, CoplanarTriangleRelation::Unknown);
        assert_eq!(classification.validate(), Ok(()));

        let indexed = classify_coplanar_triangles_with_policy(&xy, [0, 1, 2], [0, 1, 3], APPROX);
        assert_eq!(indexed.relation, CoplanarTriangleRelation::Unknown);
        assert_eq!(indexed.validate(), Ok(()));
    }

    /// Exercise signed area, turn ties, axis fallback, parallel lines, and symbolic ratios.
    #[test]
    fn projected_area_turn_and_parameter_helpers_cover_boundary_branches() {
        let clockwise = [p3(0, 0, 0), p3(0, 3, 0), p3(4, 3, 0), p3(4, 0, 0)];
        assert_eq!(
            projected_polygon_area2_sign_with_policy(&clockwise, CoplanarProjection::Xy, APPROX,)
                .value(),
            Some(Sign::Negative)
        );
        assert_eq!(
            projected_polygon_area2_value(&clockwise, CoplanarProjection::Xy),
            Real::from(-24)
        );
        assert_eq!(
            projected_polygon_area2_abs_value_with_policy(
                &clockwise,
                CoplanarProjection::Xy,
                APPROX,
            ),
            Some(Real::from(24))
        );
        assert_eq!(
            projected_polygon_area2_value(&[], CoplanarProjection::Xy),
            Real::from(0)
        );

        let base = Point2::new(Real::from(1), Real::from(0));
        let up = Point2::new(Real::from(0), Real::from(1));
        let down = Point2::new(Real::from(0), Real::from(-1));
        let northeast = Point2::new(Real::from(1), Real::from(1));
        assert_eq!(
            ccw_projected_turn_less_with_policy(&base, &up, &down, APPROX),
            Some(true)
        );
        assert_eq!(
            ccw_projected_turn_less_with_policy(&base, &down, &up, APPROX),
            Some(false)
        );
        assert_eq!(
            ccw_projected_turn_less_with_policy(&base, &northeast, &up, APPROX),
            Some(true)
        );
        assert_eq!(
            ccw_projected_turn_less_with_policy(&base, &up, &northeast, APPROX),
            Some(false)
        );
        assert_eq!(
            ccw_projected_turn_less_with_policy(&base, &up, &up, APPROX),
            Some(false)
        );
        assert_eq!(
            ccw_projected_turn_less_with_policy(
                &base,
                &Point2::new(Real::from(2), Real::from(0)),
                &Point2::new(Real::from(-1), Real::from(0)),
                APPROX,
            ),
            Some(true)
        );

        let start = p3(0, 0, 0);
        let end = p3(0, 4, 0);
        assert_eq!(
            projected_segment_parameter3_with_policy(
                &p3(0, 2, 0),
                &start,
                &end,
                CoplanarProjection::Xy,
                APPROX,
            ),
            Some((Real::from(1) / Real::from(2)).unwrap())
        );
        assert_eq!(
            projected_segment_parameter3_with_policy(
                &start,
                &start,
                &start,
                CoplanarProjection::Xy,
                APPROX,
            ),
            None
        );
        assert_eq!(
            projected_line_parameter3_with_policy(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(0, 1, 0),
                &p3(4, 1, 0),
                CoplanarProjection::Xy,
                APPROX,
            ),
            None
        );
        assert_eq!(
            intersect_segment_with_projected_line3_with_policy(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(0, 1, 0),
                &p3(4, 1, 0),
                CoplanarProjection::Xy,
                APPROX,
            ),
            None
        );

        let half = (Real::from(1) / Real::from(2)).unwrap();
        let deep_positive = exact_normal_positive();
        let segment_start = Point3::new(Real::zero(), deep_positive.clone() * &half, Real::zero());
        let segment_end = Point3::new(Real::zero(), -(deep_positive.clone() * &half), Real::zero());
        let parameter = projected_line_parameter3_with_policy(
            &segment_start,
            &segment_end,
            &p3(0, 0, 0),
            &p3(1, 0, 0),
            CoplanarProjection::Xy,
            PredicatePolicy::STRICT,
        )
        .expect("the exact-normal projected denominator should construct a parameter");
        assert_eq!(
            parameter.exact_rational_normal_form(),
            half.exact_rational()
        );
        let crossing = intersect_segment_with_projected_line3_with_policy(
            &segment_start,
            &segment_end,
            &p3(0, 0, 0),
            &p3(1, 0, 0),
            CoplanarProjection::Xy,
            PredicatePolicy::STRICT,
        )
        .expect("the certified parameter should construct the projected crossing");
        assert_eq!(
            crate::compare_reals(&crossing.y, &Real::zero(), PredicatePolicy::STRICT).value(),
            Some(core::cmp::Ordering::Equal)
        );

        let unresolved = terminally_unresolved_zero();
        assert_eq!(
            projected_line_parameter3_with_policy(
                &Point3::new(Real::zero(), unresolved.clone(), Real::zero()),
                &Point3::new(Real::zero(), -unresolved, Real::zero()),
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                CoplanarProjection::Xy,
                PredicatePolicy::STRICT,
            ),
            None
        );
    }

    /// Require Unknown with an XY projection for unresolved strict contact.
    ///
    /// The assertions validate the report shape, not the retained edge-prefix content.
    #[test]
    fn strict_coplanar_composition_keeps_unresolved_edge_evidence() {
        let left = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let right = [
            Point3::new(Real::from(1), terminally_unresolved_zero(), Real::from(0)),
            p3(5, 1, 0),
            p3(1, 5, 0),
        ];
        let report = classify_coplanar_triangle_points_with_policy(
            [&left[0], &left[1], &left[2]],
            [&right[0], &right[1], &right[2]],
            PredicatePolicy::STRICT,
        );
        assert_eq!(report.relation, CoplanarTriangleRelation::Unknown);
        assert_eq!(report.projection, Some(CoplanarProjection::Xy));
        assert_eq!(report.validate(), Ok(()));
    }

    /// Check rational and translated symbolic nondegeneracy controls.
    ///
    /// These value assertions do not count dispatches or prove a particular stage ran.
    #[test]
    fn triangle3_degeneracy_exercises_each_exact_representation_stage() {
        let third = Real::new(Rational::fraction(1, 3).unwrap());
        let fifth = Real::new(Rational::fraction(1, 5).unwrap());
        let word_a = Point3::new(Real::zero(), Real::zero(), Real::zero());
        let word_b = Point3::new(third.clone(), Real::zero(), Real::zero());
        let word_c = Point3::new(Real::zero(), fifth, Real::zero());
        assert_eq!(
            classify_triangle3_degeneracy_with_policy(
                &word_a,
                &word_b,
                &word_c,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(TriangleDegeneracy::NonDegenerate)
        );

        let word_late_b = Point3::new(third.clone(), Real::zero(), third.clone());
        let word_late_c = Point3::new(&third + &third, Real::zero(), Real::one());
        assert_eq!(
            classify_triangle3_degeneracy_with_policy(
                &word_a,
                &word_late_b,
                &word_late_c,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(TriangleDegeneracy::NonDegenerate)
        );

        let limb = Rational::new(i64::MAX);
        let limb = Real::from(&limb * &limb * Rational::new(8));
        let wide_b = Point3::new(limb.clone(), Real::zero(), Real::zero());
        let wide_c = Point3::new(Real::zero(), limb, Real::zero());
        assert_eq!(
            classify_triangle3_degeneracy_with_policy(
                &word_a,
                &wide_b,
                &wide_c,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(TriangleDegeneracy::NonDegenerate)
        );

        let pi = Real::pi();
        let symbolic_a = Point3::new(pi.clone(), Real::zero(), Real::zero());
        let symbolic_b = Point3::new(&pi + Real::one(), Real::zero(), Real::zero());
        let symbolic_c = Point3::new(pi, Real::one(), Real::zero());
        assert!(matches!(
            classify_triangle3_degeneracy_with_policy(
                &symbolic_a,
                &symbolic_b,
                &symbolic_c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Decided {
                value: TriangleDegeneracy::NonDegenerate,
                ..
            }
        ));
    }
}
