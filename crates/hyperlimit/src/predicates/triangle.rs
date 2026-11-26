//! Triangle containment, segment/ray contact reports, and tetrahedron classification.
//!
//! Orientation and scalar sign decisions follow an explicit policy. Retained
//! normal/fact records require matching sources; local report validation checks
//! stored consistency, while source replay recomputes. Coplanar contact labels
//! leave finite planar overlap to another operation. Report values alone do not
//! carry the producing outcome's certainty receipt.

use crate::predicate::PredicatePolicy;
use crate::predicates::decision::{certainty_rank, max_certainty, max_stage, stage_rank};
use core::cmp::Ordering;

use crate::classify::{
    PlaneSide, RayTriangleIntersection, SegmentTriangleIntersection, TetrahedronLocation,
    Triangle3Location, TriangleLocation,
};
use crate::geometry::{HomogeneousLine3, Plane3, Point2, Point3, Triangle2Facts};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::{compare_reals_with_policy, divide_real_with_policy};
use crate::predicates::orient::{orient2d_with_policy, orient3d_with_policy};
use crate::predicates::segment_plane::{
    SegmentPlaneIntersection, SegmentPlaneRelation, SegmentPlaneValidationError,
    intersect_segment_with_plane_values_with_policy, point_plane_value,
};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{resolve_composite_policy, resolve_real_sign_direct};
use hyperreal::Real;

/// Retained winding normal and component-sign outcome for one ordered spatial triangle.
///
/// Owns exact scalar expressions, but no vertices or checked source identity. Signs
/// may be Unknown or policy-qualified. Reuse requires the same current ordered
/// vertices; changes require rebuilding this record. It carries no point-query
/// containment result.
#[derive(Clone, Debug)]
pub struct Triangle3Orientation {
    /// Unnormalized winding-normal expressions for the original ordered triangle.
    normal: Triangle3Normal,
    /// Stored component-sign outcome, including possible uncertainty or approximation.
    normal_signs: PredicateOutcome<[Sign; 3]>,
}

impl Triangle3Orientation {
    /// Return the stored normal-sign outcome, including its receipt or unresolved reason.
    ///
    /// Does not refine, revalidate source association, or apply a new policy.
    pub const fn normal_signs(&self) -> PredicateOutcome<[Sign; 3]> {
        self.normal_signs
    }
}

/// Failure of local segment-report consistency checking or source replay.
///
/// Local validation checks the plane event and the combination of stored labels;
/// only source replay recomputes triangle containment. Neither validator returns
/// the certainty receipts of its checks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentTriangleValidationError {
    /// The retained segment/plane event is internally inconsistent.
    PlaneEventInvalid(
        /// Underlying local segment/plane validation failure.
        SegmentPlaneValidationError,
    ),
    /// A retained point/triangle location was missing when the plane event
    /// constructed a candidate point.
    MissingTriangleLocation,
    /// A retained point/triangle location was present for a relation that
    /// should not construct a single candidate point.
    UnexpectedTriangleLocation,
    /// Retained plane-event and point/triangle facts derive a different coarse
    /// relation.
    RelationMismatch,
    /// Recomputing from source geometry did not reproduce this report.
    SourceReplayMismatch,
}

/// Retained segment/support-plane event and its triangle-location claim.
///
/// Proper plane crossings retain a determinant-ratio segment parameter through
/// [`SegmentPlaneIntersection`], even if the candidate lies outside the triangle.
/// Coplanar means that the support-plane test is coplanar, not that the finite
/// segment intersects the triangle. Public fields are mutable claims; the record
/// itself contains no aggregate certainty receipt. Keep the producing
/// [`PredicateOutcome`] when consuming policy-dependent results.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentTriangleIntersectionReport {
    /// Stored coarse relation, checked against the other labels by local validation.
    pub relation: SegmentTriangleIntersection,
    /// Retained segment/support-plane event; public contents require validation.
    pub plane_event: SegmentPlaneIntersection,
    /// Stored candidate location; None for disjoint-plane and coplanar events.
    pub triangle_location: Option<Triangle3Location>,
}

impl SegmentTriangleIntersectionReport {
    /// Check the plane event locally, then compare the relation derived from stored labels.
    ///
    /// Trusts `triangle_location`: no triangle is supplied and containment is not
    /// recomputed. Propagates plane-validation errors but discards successful check
    /// receipts. Success establishes local consistency, not source fidelity.
    pub fn validate(&self, policy: PredicatePolicy) -> Result<(), SegmentTriangleValidationError> {
        self.plane_event
            .validate(policy)
            .map_err(SegmentTriangleValidationError::PlaneEventInvalid)?;
        let expected =
            relation_from_segment_plane_event(&self.plane_event, self.triangle_location)?;
        if expected == self.relation {
            Ok(())
        } else {
            Err(SegmentTriangleValidationError::RelationMismatch)
        }
    }

    /// Validate locally, then recompute with these sources and compare the report value.
    ///
    /// Unknown replay or unequal reports become SourceReplayMismatch. The fresh
    /// outcome's certainty/stage are discarded; success does not return a receipt or
    /// establish equivalence between arbitrary alternative representations.
    pub fn validate_against_sources(
        &self,
        p: &Point3,
        q: &Point3,
        a: &Point3,
        b: &Point3,
        c: &Point3,
        policy: PredicatePolicy,
    ) -> Result<(), SegmentTriangleValidationError> {
        self.validate(policy)?;
        match classify_segment_triangle3_intersection_report_with_policy(p, q, a, b, c, policy) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(SegmentTriangleValidationError::SourceReplayMismatch),
        }
    }

    /// Test whether the plane event stores a point, without validating it or proving a hit.
    pub fn has_candidate_point(&self) -> bool {
        self.plane_event.point.is_some()
    }
}

/// Failure of ray-report field consistency, parameter checks, or source replay.
///
/// Unresolved scalar checks are mapped to parameter errors. This vocabulary does
/// not distinguish an invalid scalar relation from inability to decide it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RayTriangleValidationError {
    /// A relation that should not construct a candidate retained one.
    UnexpectedCandidate,
    /// An intersecting relation did not retain the constructed candidate point.
    MissingCandidate,
    /// A relation that should not retain a triangle location retained one.
    UnexpectedTriangleLocation,
    /// A constructed candidate did not retain its point/triangle location.
    MissingTriangleLocation,
    /// Ratio shape, division, or equality failed or was undecided under policy.
    InvalidParameterRatio,
    /// A parameter was missing, negative, inconsistent with an origin event, or undecidable.
    InvalidParameter,
    /// Retained construction facts derive a different coarse relation.
    RelationMismatch,
    /// Recomputing from source geometry did not reproduce this report.
    SourceReplayMismatch,
}

/// Retained scalar expressions for a transverse ray/support-plane parameter.
///
/// For `r(t) = origin + t * direction` and `E(x) = normal . x + offset`, the
/// construction uses `t = -E(origin) / (normal . direction)`. Public fields do
/// not enforce this source association or a nonzero denominator. Local report
/// validation compares the quotient with the retained parameter, not its derivation.
#[derive(Clone, Debug, PartialEq)]
pub struct RayTriangleParameterRatio {
    /// Claimed numerator `-E(origin)`; source derivation is not enforced by this field.
    pub numerator: Real,
    /// Claimed denominator `normal . direction`; nonzeroness is not enforced by this field.
    pub denominator: Real,
}

/// Retained ray/support-plane candidate and triangle-location claim.
///
/// A transverse plane crossing can retain a point outside the finite triangle.
/// Origin-on-plane events with nonzero normal-direction dot product retain t = 0;
/// coplanar events retain no single candidate and do not test planar overlap.
/// Fields contain claims and exact expressions, without an aggregate certainty
/// receipt. Keep the producing [`PredicateOutcome`]; local validation alone does
/// not establish that the candidate belongs to the supplied ray or triangle.
#[derive(Clone, Debug, PartialEq)]
pub struct RayTriangleIntersectionReport {
    /// Stored coarse relation, which may describe a candidate outside the triangle.
    pub relation: RayTriangleIntersection,
    /// Stored origin-side claim; no per-field certainty receipt is retained.
    pub origin_side: Option<PlaneSide>,
    /// Stored sign claim for `normal . direction`, without its original receipt.
    pub direction_sign: Option<Sign>,
    /// Candidate ray parameter; may use the source-vertex projection formula.
    pub parameter: Option<Real>,
    /// Original support-plane ratio for transverse events, including vertex contacts.
    pub parameter_ratio: Option<RayTriangleParameterRatio>,
    /// Stored candidate, not necessarily a triangle hit; mutable without source revalidation.
    pub point: Option<Point3>,
    /// Stored location of the candidate relative to the closed triangle.
    pub triangle_location: Option<Triangle3Location>,
}

impl RayTriangleIntersectionReport {
    /// Check optional-field consistency, nonnegative parameter, ratio, and derived relation.
    ///
    /// With no ratio, a retained parameter must be zero and origin_side must be On.
    /// With a ratio, checks its quotient and rejects an explicitly On origin or Zero
    /// direction sign; missing side/sign labels are not rejected by that branch.
    /// A Disjoint report without a candidate does not have its side/sign claims checked.
    ///
    /// Does not check the point against a ray equation, supporting plane, or triangle,
    /// nor derive the ratio from sources. Unknown scalar checks become parameter errors
    /// and successful check receipts are discarded. Use source replay for recomputation.
    pub fn validate(&self, policy: PredicatePolicy) -> Result<(), RayTriangleValidationError> {
        match (self.point.is_some(), self.parameter.is_some()) {
            (true, false) => return Err(RayTriangleValidationError::InvalidParameter),
            (false, true) => return Err(RayTriangleValidationError::UnexpectedCandidate),
            _ => {}
        }

        if let Some(parameter) = self.parameter.as_ref() {
            assert_ray_parameter_nonnegative(parameter, policy)?;
        }

        match (self.parameter_ratio.as_ref(), self.parameter.as_ref()) {
            (Some(ratio), Some(parameter)) => {
                validate_ray_parameter_ratio(ratio, parameter, policy)?;
                if self.origin_side == Some(PlaneSide::On)
                    || self.direction_sign == Some(Sign::Zero)
                {
                    return Err(RayTriangleValidationError::InvalidParameterRatio);
                }
            }
            (Some(_), None) => return Err(RayTriangleValidationError::InvalidParameterRatio),
            (None, Some(parameter)) => {
                validate_ray_origin_parameter(self.origin_side, parameter, policy)?;
            }
            (None, None) => {}
        }

        let expected = relation_from_ray_report_facts(self)?;
        if expected == self.relation {
            Ok(())
        } else {
            Err(RayTriangleValidationError::RelationMismatch)
        }
    }

    /// Validate locally, then recompute with these sources and compare the report value.
    ///
    /// Unknown replay or unequal values become SourceReplayMismatch. Comparison does
    /// not retain the new certainty/stage or prove equality of alternative constructions.
    pub fn validate_against_sources(
        &self,
        origin: &Point3,
        direction: &Point3,
        a: &Point3,
        b: &Point3,
        c: &Point3,
        policy: PredicatePolicy,
    ) -> Result<(), RayTriangleValidationError> {
        self.validate(policy)?;
        match classify_ray_triangle3_intersection_report_with_policy(
            origin, direction, a, b, c, policy,
        ) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(RayTriangleValidationError::SourceReplayMismatch),
        }
    }

    /// Test whether a candidate point is stored, without checking validity or triangle membership.
    pub fn has_candidate_point(&self) -> bool {
        self.point.is_some()
    }
}

/// Classify a planar point relative to a closed triangle of either winding.
///
/// Zero triangle orientation yields Degenerate before any query-edge test. For a
/// nondegenerate triangle, resolves AB, BC, CA orientations in that order, propagates
/// the first Unknown, then distinguishes outside, interior, edge, and vertex.
/// Combines consumed receipts; approximate policy does not guarantee exact decisions.
pub fn classify_point_triangle_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_impl(a, b, c, point, policy, None, None)
}

/// Construct `(b-a) x (c-a)` and retain its component-sign outcome under policy.
///
/// Does not reject a zero normal or Unknown signs. The returned record is associated
/// with the ordered source vertices only by caller discipline.
pub fn triangle3_orientation(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> Triangle3Orientation {
    let normal = triangle3_normal(a, b, c);
    let normal_signs = triangle3_normal_signs_outcome(&normal, policy);
    Triangle3Orientation {
        normal,
        normal_signs,
    }
}

/// Classify a spatial point using orientation retained for the same ordered vertices.
///
/// The source association is required but unchecked. Accepted decided signs and
/// Unknown outcomes are reused; a cached Unknown therefore propagates without a
/// fresh sign attempt. Disallowed certainty triggers new signs of the stored normal,
/// not reconstruction from the current vertices. Remaining plane/edge checks follow
/// the supplied policy and accumulate consumed receipts.
pub fn classify_point_triangle3_with_orientation(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
    orientation: &Triangle3Orientation,
    policy: PredicatePolicy,
) -> PredicateOutcome<Triangle3Location> {
    let normal_signs = reusable_outcome(orientation.normal_signs, policy)
        .unwrap_or_else(|| triangle3_normal_signs_outcome(&orientation.normal, policy));
    classify_point_triangle3_impl(a, b, c, point, policy, &orientation.normal, normal_signs)
}

/// Classify a spatial point by normal nondegeneracy, coplanarity, and edge halfspaces.
///
/// A zero winding normal yields Degenerate; a nonzero plane orientation yields
/// OffPlane. Otherwise tests `normal . ((edge_end-edge_start) x (point-edge_start))`
/// for AB, BC, CA. All three edge calls run before their outcomes are reduced, so
/// an unresolved edge can prevent an Outside answer even after another is negative.
/// Decisions retain consumed certainty/stage; encountered Unknown remains explicit.
pub fn classify_point_triangle3_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Triangle3Location> {
    let normal = triangle3_normal(a, b, c);
    let normal_signs = triangle3_normal_signs_outcome(&normal, policy);
    classify_point_triangle3_impl(a, b, c, point, policy, &normal, normal_signs)
}

/// Decide the sign of `((b-a) x (c-a)) . reference_normal` under policy.
///
/// Positive/negative distinguish the two dot-product hemispheres, not parallelism.
/// Zero includes a degenerate triangle, zero reference, or orthogonal normals.
/// Neither vector is normalized; the scalar resolver's receipt is returned.
pub fn triangle3_winding_normal_sign_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    reference_normal: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    crate::trace_dispatch!("hyperlimit", "triangle3_winding_normal_sign", "normal-dot");
    let normal = triangle3_normal(a, b, c);
    let dot = Real::signed_product_sum(
        [true; 3],
        [
            [&normal.x, &reference_normal.x],
            [&normal.y, &reference_normal.y],
            [&normal.z, &reference_normal.z],
        ],
    );
    resolve_real_sign_direct(&dot, policy, RefinementNeed::RealRefinement)
}

/// Classify a segment and retain its support-plane construction and triangle location.
///
/// Resolves both endpoint plane orientations, constructs the segment/plane event,
/// then classifies a single candidate against the triangle. Coplanar is a support
/// relation without a finite-overlap test; triangle degeneracy is not rejected
/// up front. A candidate outside the triangle is retained in a Disjoint report.
///
/// APPROXIMATE_512 first tries STRICT; only an unresolved attempt is retried with
/// approximation, with successful replay marked Approximate/Refined. The internal
/// attempt drops child receipts and labels success Exact/Exact; the wrapper does
/// not preserve the individual decision stages. Retain the outer outcome.
///
/// # Panics
///
/// The report finisher expects a point in endpoint/proper plane events; violating
/// that internal event invariant panics.
pub fn classify_segment_triangle3_intersection_report_with_policy(
    p: &Point3,
    q: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentTriangleIntersectionReport> {
    resolve_composite_policy(policy, |policy| {
        classify_segment_triangle3_intersection_report_impl(p, q, a, b, c, policy)
    })
}

/// Build one segment report attempt under the supplied policy.
///
/// Evaluates both endpoint orientations before selecting the first Unknown. Their
/// receipts are discarded when converting to side labels; the finisher also labels
/// success Exact/Exact. Must remain behind the composite-policy wrapper when
/// approximation is allowed.
fn classify_segment_triangle3_intersection_report_impl(
    p: &Point3,
    q: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentTriangleIntersectionReport> {
    crate::trace_dispatch!(
        "hyperlimit",
        "segment_triangle3_report",
        "plane-event-replay"
    );
    let plane = triangle_support_plane(a, b, c);
    let outcomes = [
        orient3d_with_policy(a, b, c, p, policy),
        orient3d_with_policy(a, b, c, q, policy),
    ];
    for outcome in outcomes {
        if let PredicateOutcome::Unknown { needed, stage } = outcome {
            return PredicateOutcome::unknown(needed, stage);
        }
    }
    let sides = [
        outcomes[0].value().map(PlaneSide::from),
        outcomes[1].value().map(PlaneSide::from),
    ];
    let d0 = point_plane_value(&plane, p);
    let d1 = point_plane_value(&plane, q);
    let plane_event =
        intersect_segment_with_plane_values_with_policy(&d0, &d1, p, q, sides, policy);

    finish_segment_triangle3_report(plane_event, a, b, c, policy)
}

/// Combine a plane event with candidate containment, returning a bare report in an outcome.
///
/// Unknown events become Unsupported/Undecided, construction failure becomes
/// Unsupported/Exact, and every success is labelled Exact/Exact without child
/// receipt aggregation. Disjoint/coplanar events skip containment. Endpoint/proper
/// events classify their point and propagate Unknown.
///
/// # Panics
///
/// Panics if an endpoint/proper event has no point; callers must supply a valid event.
fn finish_segment_triangle3_report(
    plane_event: SegmentPlaneIntersection,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentTriangleIntersectionReport> {
    match plane_event.relation {
        SegmentPlaneRelation::Unknown => {
            PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
        }
        SegmentPlaneRelation::ConstructionFailed => {
            PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Exact)
        }
        SegmentPlaneRelation::Disjoint | SegmentPlaneRelation::Coplanar => {
            let relation = if plane_event.relation == SegmentPlaneRelation::Disjoint {
                SegmentTriangleIntersection::Disjoint
            } else {
                SegmentTriangleIntersection::Coplanar
            };
            PredicateOutcome::decided(
                SegmentTriangleIntersectionReport {
                    relation,
                    plane_event,
                    triangle_location: None,
                },
                Certainty::Exact,
                Escalation::Exact,
            )
        }
        SegmentPlaneRelation::EndpointOnPlane | SegmentPlaneRelation::ProperCrossing => {
            let point = plane_event
                .point
                .as_ref()
                .expect("a certified segment/plane intersection event retains its point");
            let location = match classify_point_triangle3_with_policy(a, b, c, point, policy) {
                PredicateOutcome::Decided { value, .. } => value,
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            };
            let relation =
                relation_from_constructed_segment_triangle_point(plane_event.relation, location);
            PredicateOutcome::decided(
                SegmentTriangleIntersectionReport {
                    relation,
                    plane_event,
                    triangle_location: Some(location),
                },
                Certainty::Exact,
                Escalation::Exact,
            )
        }
    }
}

/// Classify a closed spatial segment against a triangle without retaining a report.
///
/// Resolves endpoint orientations in order, merging their receipts. Both zero yields
/// Coplanar without testing finite overlap or rejecting a degenerate triangle.
/// Equal nonzero sides yield Disjoint; an on-plane endpoint uses point containment.
/// A strict crossing first requires a homogeneous line/plane point to convert to
/// affine form, then discards that point and tests input line/edge determinants.
/// Affine conversion failure becomes Unsupported/Undecided. This path differs from
/// the report builder, which classifies its constructed candidate.
pub fn classify_segment_triangle3_intersection_with_policy(
    p: &Point3,
    q: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentTriangleIntersection> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let p_side = match segment_triangle_sign(
        orient3d_with_policy(a, b, c, p, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown,
    };
    let q_side = match segment_triangle_sign(
        orient3d_with_policy(a, b, c, q, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown,
    };

    classify_segment_triangle3_intersection_from_sides(
        p, q, a, b, c, p_side, q_side, None, policy, certainty, stage,
    )
}

/// Classify an edge using trusted endpoint sides and a matching triangle support plane.
///
/// The caller must establish the side labels and source association. They are not
/// revalidated and carry no receipt input: accumulation starts Exact/Exact. Any
/// inherited qualification must be transported by the caller. Only zero/equal/opposite
/// side relationships are used, so simultaneous reversal of the side convention
/// does not change the branch.
pub(crate) fn classify_segment_triangle3_intersection_with_preclassified_sides(
    edge: [&Point3; 2],
    triangle: [&Point3; 3],
    endpoint_sides: [PlaneSide; 2],
    plane: &Plane3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentTriangleIntersection> {
    let sign = |side| match side {
        PlaneSide::Below => Sign::Negative,
        PlaneSide::On => Sign::Zero,
        PlaneSide::Above => Sign::Positive,
    };
    classify_segment_triangle3_intersection_from_sides(
        edge[0],
        edge[1],
        triangle[0],
        triangle[1],
        triangle[2],
        sign(endpoint_sides[0]),
        sign(endpoint_sides[1]),
        Some(plane),
        policy,
        Certainty::Exact,
        Escalation::Exact,
    )
}

/// Reduce trusted endpoint signs, then handle endpoint or transverse containment.
///
/// Both zero gives Coplanar, equal nonzero gives Disjoint, and a single zero delegates
/// to endpoint containment. A crossing uses the retained plane if present, otherwise
/// constructs one; affine conversion is only a success gate before line/edge tests.
/// Combines those tests with the incoming receipt. Sources, signs, and any retained
/// plane must correspond; this helper does not check that association.
#[allow(clippy::too_many_arguments)]
fn classify_segment_triangle3_intersection_from_sides(
    p: &Point3,
    q: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    p_side: Sign,
    q_side: Sign,
    retained_plane: Option<&Plane3>,
    policy: PredicatePolicy,
    certainty: Certainty,
    stage: Escalation,
) -> PredicateOutcome<SegmentTriangleIntersection> {
    if p_side == Sign::Zero && q_side == Sign::Zero {
        return PredicateOutcome::decided(SegmentTriangleIntersection::Coplanar, certainty, stage);
    }
    if p_side != Sign::Zero && p_side == q_side {
        return PredicateOutcome::decided(SegmentTriangleIntersection::Disjoint, certainty, stage);
    }

    if p_side == Sign::Zero {
        return segment_endpoint_triangle_relation(p, a, b, c, policy, certainty, stage);
    }
    if q_side == Sign::Zero {
        return segment_endpoint_triangle_relation(q, a, b, c, policy, certainty, stage);
    }

    let owned_plane;
    let plane = if let Some(plane) = retained_plane {
        plane
    } else {
        owned_plane = triangle_support_plane(a, b, c);
        &owned_plane
    };
    let line = line_from_points(p, q);
    let point = line.intersect_plane(plane);
    match point.to_affine_point() {
        Ok(_) => {
            let direction = Point3::new(
                sub_ref(&q.x, &p.x),
                sub_ref(&q.y, &p.y),
                sub_ref(&q.z, &p.z),
            );
            match line_triangle3_crossing_location(p, &direction, a, b, c, policy) {
                PredicateOutcome::Decided {
                    value: (value, _),
                    certainty: point_certainty,
                    stage: point_stage,
                } => {
                    let relation = match value {
                        Triangle3Location::Inside => SegmentTriangleIntersection::Proper,
                        Triangle3Location::OnEdge | Triangle3Location::OnVertex => {
                            SegmentTriangleIntersection::BoundaryTouch
                        }
                        Triangle3Location::Outside
                        | Triangle3Location::OffPlane
                        | Triangle3Location::Degenerate => SegmentTriangleIntersection::Disjoint,
                    };
                    PredicateOutcome::decided(
                        relation,
                        max_certainty(certainty, point_certainty),
                        max_stage(stage, point_stage),
                    )
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            }
        }
        Err(_) => PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided),
    }
}

/// Classify `origin + t*direction`, t >= 0, retaining a support-plane candidate when unique.
///
/// Zero normal-direction dot product gives Coplanar for an on-plane origin and
/// Disjoint otherwise, including zero directions. Coplanar does not test finite
/// triangle overlap; triangle degeneracy is not rejected up front. A ray pointing
/// away retains no candidate. An on-plane transverse origin retains t = 0; other
/// forward crossings use line/edge determinants and retain even outside candidates.
/// Vertex contacts reuse the source vertex and its projection parameter while
/// retaining the original support-plane ratio.
///
/// APPROXIMATE_512 tries STRICT first, then retries Unknown with approximation;
/// successful replay is Approximate/Refined. Internal attempts discard child
/// receipts and label success Exact/Exact. Keep the outer outcome with the report.
///
/// # Panics
///
/// Panics if reciprocal construction fails after a policy-decided nonzero plane
/// denominator, or for the squared direction length used at a vertex contact.
pub fn classify_ray_triangle3_intersection_report_with_policy(
    origin: &Point3,
    direction: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<RayTriangleIntersectionReport> {
    resolve_composite_policy(policy, |policy| {
        classify_ray_triangle3_intersection_report_impl(origin, direction, a, b, c, policy)
    })
}

/// Build one ray report attempt by origin/direction signs and triangle containment.
///
/// Drops child receipts and labels successful branches Exact/Exact; approximate
/// use requires the public composite wrapper. Transverse vertex hits reuse the
/// source vertex and dot-product parameter but keep the original plane ratio.
///
/// # Panics
///
/// Expects reciprocal construction to succeed for the decided nonzero plane
/// denominator and, at a vertex, the direction's squared length.
fn classify_ray_triangle3_intersection_report_impl(
    origin: &Point3,
    direction: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<RayTriangleIntersectionReport> {
    crate::trace_dispatch!("hyperlimit", "ray_triangle3_report", "ray-plane-replay");
    let plane = triangle_support_plane(a, b, c);
    let origin_expression = plane_expression_at(&plane, origin);
    let origin_sign = match sign_for_ray_triangle(&origin_expression, policy) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let direction_expression = dot_point3(&plane.normal, direction);
    let direction_sign = match sign_for_ray_triangle(&direction_expression, policy) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let origin_side = Some(PlaneSide::from(origin_sign));
    if direction_sign == Sign::Zero {
        let relation = if origin_sign == Sign::Zero {
            RayTriangleIntersection::Coplanar
        } else {
            RayTriangleIntersection::Disjoint
        };
        return PredicateOutcome::decided(
            RayTriangleIntersectionReport {
                relation,
                origin_side,
                direction_sign: Some(direction_sign),
                parameter: None,
                parameter_ratio: None,
                point: None,
                triangle_location: None,
            },
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if origin_sign != Sign::Zero && origin_sign == direction_sign {
        return PredicateOutcome::decided(
            RayTriangleIntersectionReport {
                relation: RayTriangleIntersection::Disjoint,
                origin_side,
                direction_sign: Some(direction_sign),
                parameter: None,
                parameter_ratio: None,
                point: None,
                triangle_location: None,
            },
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if origin_sign == Sign::Zero {
        return match classify_point_triangle3_with_policy(a, b, c, origin, policy) {
            PredicateOutcome::Decided { value, .. } => {
                let relation = relation_from_ray_origin_triangle_point(value);
                PredicateOutcome::decided(
                    RayTriangleIntersectionReport {
                        relation,
                        origin_side,
                        direction_sign: Some(direction_sign),
                        parameter: Some(Real::from(0)),
                        parameter_ratio: None,
                        point: Some(origin.clone()),
                        triangle_location: Some(value),
                    },
                    Certainty::Exact,
                    Escalation::Exact,
                )
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        };
    }

    let numerator = neg_real(&origin_expression);
    let reciprocal = direction_expression
        .inverse_ref_assuming_nonzero()
        .expect("a certified nonzero ray/plane denominator is a valid divisor");
    let parameter = &numerator * reciprocal;
    match line_triangle3_crossing_location(origin, direction, a, b, c, policy) {
        PredicateOutcome::Decided {
            value: (value, vertex),
            ..
        } => {
            // A vertex contact is reported as the vertex itself, with its
            // parameter along the ray, so every triangle sharing the vertex
            // reports identical exact values; reconstructing the hit from
            // the plane would give each triangle a different expression whose
            // equality cannot be certified.
            let (parameter, intersection) = match vertex.map(|index| [a, b, c][index]) {
                Some(vertex) => {
                    let offset = Point3::new(
                        sub_ref(&vertex.x, &origin.x),
                        sub_ref(&vertex.y, &origin.y),
                        sub_ref(&vertex.z, &origin.z),
                    );
                    let length_squared = dot_point3(direction, direction);
                    let reciprocal = length_squared
                        .inverse_ref_assuming_nonzero()
                        .expect("a ray crossing a plane has a nonzero direction");
                    (&dot_point3(&offset, direction) * reciprocal, vertex.clone())
                }
                None => {
                    let intersection = ray_point_at(origin, direction, &parameter);
                    (parameter, intersection)
                }
            };
            let relation = relation_from_constructed_ray_triangle_point(value);
            PredicateOutcome::decided(
                RayTriangleIntersectionReport {
                    relation,
                    origin_side,
                    direction_sign: Some(direction_sign),
                    parameter: Some(parameter),
                    parameter_ratio: Some(RayTriangleParameterRatio {
                        numerator,
                        denominator: direction_expression,
                    }),
                    point: Some(intersection),
                    triangle_location: Some(value),
                },
                Certainty::Exact,
                Escalation::Exact,
            )
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Return the ray report's coarse relation, preserving the outer outcome receipt.
///
/// Uses [`classify_ray_triangle3_intersection_report_with_policy`], including its
/// strict-first retry, candidate construction, coplanar convention, and panic paths.
/// This is not a construction-free classifier.
pub fn classify_ray_triangle3_intersection_with_policy(
    origin: &Point3,
    direction: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<RayTriangleIntersection> {
    match classify_ray_triangle3_intersection_report_with_policy(origin, direction, a, b, c, policy)
    {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.relation, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Consume a matching winding normal, reject degeneracy/off-plane queries, then test edges.
///
/// Trusts the normal/source association and supplied sign outcome. Evaluates all
/// three edge calls before reducing AB, BC, CA outcomes; a later Unknown prevents
/// an earlier negative edge from becoming Outside. Combines consumed receipts,
/// then uses zero counts to distinguish interior, edge, and vertex.
fn classify_point_triangle3_impl(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
    normal: &Triangle3Normal,
    normal_signs_outcome: PredicateOutcome<[Sign; 3]>,
) -> PredicateOutcome<Triangle3Location> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    let normal_signs = match normal_signs_outcome {
        PredicateOutcome::Decided {
            value,
            certainty: normal_certainty,
            stage: normal_stage,
        } => {
            certainty = max_certainty(certainty, normal_certainty);
            stage = max_stage(stage, normal_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    if normal_signs == [Sign::Zero, Sign::Zero, Sign::Zero] {
        return PredicateOutcome::decided(Triangle3Location::Degenerate, certainty, stage);
    }

    let plane_sign = match triangle3_sign(
        orient3d_with_policy(a, b, c, point, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown.into_outcome(),
    };
    if plane_sign != Sign::Zero {
        return PredicateOutcome::decided(Triangle3Location::OffPlane, certainty, stage);
    }

    let edge_ab = edge_halfspace3_sign(normal, a, b, point, policy, &mut certainty, &mut stage);
    let edge_bc = edge_halfspace3_sign(normal, b, c, point, policy, &mut certainty, &mut stage);
    let edge_ca = edge_halfspace3_sign(normal, c, a, point, policy, &mut certainty, &mut stage);
    let edge_signs = match (edge_ab, edge_bc, edge_ca) {
        (Ok(ab), Ok(bc), Ok(ca)) => [ab, bc, ca],
        (Err(unknown), _, _) | (_, Err(unknown), _) | (_, _, Err(unknown)) => {
            return unknown.into_outcome();
        }
    };

    if edge_signs.contains(&Sign::Negative) {
        return PredicateOutcome::decided(Triangle3Location::Outside, certainty, stage);
    }

    let zero_count = edge_signs
        .iter()
        .filter(|&&sign| sign == Sign::Zero)
        .count();
    let location = match zero_count {
        0 => Triangle3Location::Inside,
        1 => Triangle3Location::OnEdge,
        _ => Triangle3Location::OnVertex,
    };
    PredicateOutcome::decided(location, certainty, stage)
}

/// Classify a point in a closed tetrahedron using oriented-volume signs.
///
/// Zero base orientation yields Degenerate. Otherwise evaluates all four vertex
/// replacement determinants before reducing them in order. A sign opposite the
/// base means Outside; with no opposite sign, zero counts 0, 1, 2, and >= 3 mean
/// Inside, OnFace, OnEdge, and OnVertex. An encountered Unknown propagates and
/// successful decisions combine the consumed receipts, for either winding.
pub fn classify_point_tetrahedron_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<TetrahedronLocation> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let tetra_sign = match tetrahedron_sign(
        orient3d_with_policy(a, b, c, d, policy),
        &mut certainty,
        &mut stage,
    ) {
        Ok(sign) => sign,
        Err(unknown) => return unknown,
    };
    if tetra_sign == Sign::Zero {
        return PredicateOutcome::decided(TetrahedronLocation::Degenerate, certainty, stage);
    }

    let signs = [
        tetrahedron_sign(
            orient3d_with_policy(a, b, c, point, policy),
            &mut certainty,
            &mut stage,
        ),
        tetrahedron_sign(
            orient3d_with_policy(a, b, point, d, policy),
            &mut certainty,
            &mut stage,
        ),
        tetrahedron_sign(
            orient3d_with_policy(a, point, c, d, policy),
            &mut certainty,
            &mut stage,
        ),
        tetrahedron_sign(
            orient3d_with_policy(point, b, c, d, policy),
            &mut certainty,
            &mut stage,
        ),
    ];
    let face_signs = match signs {
        [Ok(s0), Ok(s1), Ok(s2), Ok(s3)] => [s0, s1, s2, s3],
        [Err(unknown), _, _, _]
        | [_, Err(unknown), _, _]
        | [_, _, Err(unknown), _]
        | [_, _, _, Err(unknown)] => return unknown,
    };

    let opposite = tetra_sign.reversed();
    if face_signs.contains(&opposite) {
        return PredicateOutcome::decided(TetrahedronLocation::Outside, certainty, stage);
    }

    let zero_count = face_signs
        .iter()
        .filter(|&&sign| sign == Sign::Zero)
        .count();
    let location = match zero_count {
        0 => TetrahedronLocation::Inside,
        1 => TetrahedronLocation::OnFace,
        2 => TetrahedronLocation::OnEdge,
        _ => TetrahedronLocation::OnVertex,
    };
    PredicateOutcome::decided(location, certainty, stage)
}

/// Classify a planar point using an orientation outcome for the same ordered triangle.
///
/// Source identity and the claimed sign/stage are trusted. Decided certainty must
/// be accepted by policy or orientation is recomputed. Unknown is reused unchanged,
/// so it prevents classification without a fresh orientation attempt. Subsequent
/// edge checks combine their receipts with the retained or recomputed orientation.
pub fn classify_point_triangle_with_orientation(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    orientation: PredicateOutcome<Sign>,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleLocation> {
    let orientation = reusable_outcome(orientation, policy);
    classify_point_triangle_impl(a, b, c, point, policy, None, orientation)
}

/// Classify a planar point using facts associated with the current triangle.
///
/// Known-degenerate facts supply a Zero/Exact/Structural orientation and return
/// Degenerate without checking vertices.
/// All other facts fall through to direct orientation; containment still resolves
/// edge signs under policy. The source association is an unchecked precondition,
/// and the fact record supplies no inherited certainty receipt.
pub fn classify_point_triangle_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    facts: Triangle2Facts,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_impl(a, b, c, point, policy, Some(facts), None)
}

/// Select cached, fact-assisted, or direct orientation, then resolve three edge signs.
///
/// Cached orientation takes precedence over facts and is trusted as already policy
/// screened. Zero returns Degenerate; otherwise AB, BC, CA run sequentially until
/// Unknown, without early Outside short-circuiting. Four successful receipts are
/// combined before opposite-sign and zero-count classification.
fn classify_point_triangle_impl(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    facts: Option<Triangle2Facts>,
    cached_orientation: Option<PredicateOutcome<Sign>>,
) -> PredicateOutcome<TriangleLocation> {
    let triangle_outcome = cached_orientation
        .unwrap_or_else(|| triangle_orientation_with_optional_facts(a, b, c, policy, facts));

    let triangle = match triangle_outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };

    if triangle.sign == Sign::Zero {
        return PredicateOutcome::decided(
            TriangleLocation::Degenerate,
            triangle.certainty,
            triangle.stage,
        );
    }

    let ab = match orient2d_with_policy(a, b, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };
    let bc = match orient2d_with_policy(b, c, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };
    let ca = match orient2d_with_policy(c, a, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };

    let certainty =
        combine_certainties([triangle.certainty, ab.certainty, bc.certainty, ca.certainty]);
    let stage = combine_stages([triangle.stage, ab.stage, bc.stage, ca.stage]);
    let edge_signs = [ab.sign, bc.sign, ca.sign];

    let opposite = if triangle.sign == Sign::Positive {
        Sign::Negative
    } else {
        Sign::Positive
    };

    if edge_signs.contains(&opposite) {
        return PredicateOutcome::decided(TriangleLocation::Outside, certainty, stage);
    }

    let zero_count = edge_signs
        .iter()
        .filter(|&&sign| sign == Sign::Zero)
        .count();
    let location = match zero_count {
        0 => TriangleLocation::Inside,
        1 => TriangleLocation::OnEdge,
        _ => TriangleLocation::OnVertex,
    };

    PredicateOutcome::decided(location, certainty, stage)
}

/// Reject only decided outcomes whose certainty the policy disallows.
///
/// Returns Some for accepted decisions and for Unknown, so cached uncertainty is
/// not retried. Does not validate the value, stage, source identity, or proof scope.
#[inline]
fn reusable_outcome<T: Copy>(
    outcome: PredicateOutcome<T>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<T>> {
    match outcome {
        PredicateOutcome::Decided { certainty, .. } if !policy.accepts(certainty) => None,
        outcome => Some(outcome),
    }
}

/// Select fact-assisted orientation when facts exist, otherwise direct planar orientation.
fn triangle_orientation_with_optional_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
    facts: Option<Triangle2Facts>,
) -> PredicateOutcome<Sign> {
    if let Some(facts) = facts {
        triangle_orientation_with_policy_and_facts(a, b, c, policy, facts)
    } else {
        orient2d_with_policy(a, b, c, policy)
    }
}

/// Trust known degeneracy as Zero/Exact/Structural; otherwise compute planar orientation.
///
/// Facts must correspond to the supplied vertices. Known nondegeneracy alone does
/// not determine winding and does not bypass the determinant.
fn triangle_orientation_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
    facts: Triangle2Facts,
) -> PredicateOutcome<Sign> {
    if facts.known_degenerate() == Some(true) {
        // Same-axis and duplicate-vertex degeneracies can be certified from
        // exact zero/nonzero structure before constructing the orientation
        // determinant; it is still an exact predicate result.
        PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Structural)
    } else {
        orient2d_with_policy(a, b, c, policy)
    }
}

/// Unnormalized exact component expressions of a spatial triangle's winding normal.
///
/// May be zero; construction alone does not decide degeneracy or retain vertex identity.
#[derive(Clone, Debug)]
struct Triangle3Normal {
    /// X component of the unnormalized winding normal.
    x: Real,
    /// Y component of the unnormalized winding normal.
    y: Real,
    /// Z component of the unnormalized winding normal.
    z: Real,
}

/// Construct `(b-a) x (c-a)` with difference-of-products kernels preserving cancellations.
fn triangle3_normal(a: &Point3, b: &Point3, c: &Point3) -> Triangle3Normal {
    let abx = sub_ref(&b.x, &a.x);
    let aby = sub_ref(&b.y, &a.y);
    let abz = sub_ref(&b.z, &a.z);
    let acx = sub_ref(&c.x, &a.x);
    let acy = sub_ref(&c.y, &a.y);
    let acz = sub_ref(&c.z, &a.z);

    // Vertical/extruded faces can have equal projected edges. The scalar
    // primitive proves their cancelling products before expression expansion.
    Triangle3Normal {
        x: Real::diff_of_products(&aby, &acz, &abz, &acy),
        y: Real::diff_of_products(&abz, &acx, &abx, &acz),
        z: Real::diff_of_products(&abx, &acy, &aby, &acx),
    }
}

/// Construct the oriented support equation `normal . x - normal . a = 0`.
///
/// Does not reject a zero normal: degenerate triangles produce a zero equation.
fn triangle_support_plane(a: &Point3, b: &Point3, c: &Point3) -> Plane3 {
    let normal = triangle3_normal(a, b, c);
    let normal_point = Point3::new(normal.x, normal.y, normal.z);
    let offset = neg_real(&dot_point3(&normal_point, a));
    Plane3::new(normal_point, offset)
}

/// Construct a homogeneous line with direction `end-start` and moment `start x direction`.
///
/// Does not reject coincident endpoints; callers must establish a usable line.
fn line_from_points(start: &Point3, end: &Point3) -> HomogeneousLine3 {
    let direction = Point3::new(
        sub_ref(&end.x, &start.x),
        sub_ref(&end.y, &start.y),
        sub_ref(&end.z, &start.z),
    );
    let moment = Point3::new(
        sub_ref(
            &mul_ref(&start.y, &direction.z),
            &mul_ref(&start.z, &direction.y),
        ),
        sub_ref(
            &mul_ref(&start.z, &direction.x),
            &mul_ref(&start.x, &direction.z),
        ),
        sub_ref(
            &mul_ref(&start.x, &direction.y),
            &mul_ref(&start.y, &direction.x),
        ),
    );
    HomogeneousLine3::new(direction, moment)
}

/// Form `plane.normal . point + plane.offset` as a four-term signed product sum.
fn plane_expression_at(plane: &Plane3, point: &Point3) -> Real {
    let one = Real::one();
    Real::signed_product_sum(
        [true; 4],
        [
            [&plane.normal.x, &point.x],
            [&plane.normal.y, &point.y],
            [&plane.normal.z, &point.z],
            [&plane.offset, &one],
        ],
    )
}

/// Form a three-component dot product with the shared signed-product-sum kernel.
fn dot_point3(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

/// Represent exact negation as zero minus the supplied scalar expression.
fn neg_real(value: &Real) -> Real {
    sub_ref(&Real::from(0), value)
}

/// Classify an on-plane endpoint and map closed-triangle membership to BoundaryTouch.
///
/// Outside, OffPlane, and Degenerate map to Disjoint. Preserves Unknown and merges
/// the point-classifier receipt with the incoming segment-side receipt.
fn segment_endpoint_triangle_relation(
    endpoint: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
    certainty: Certainty,
    stage: Escalation,
) -> PredicateOutcome<SegmentTriangleIntersection> {
    match classify_point_triangle3_with_policy(a, b, c, endpoint, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: endpoint_certainty,
            stage: endpoint_stage,
        } => {
            let relation = match value {
                Triangle3Location::Inside
                | Triangle3Location::OnEdge
                | Triangle3Location::OnVertex => SegmentTriangleIntersection::BoundaryTouch,
                Triangle3Location::Outside
                | Triangle3Location::OffPlane
                | Triangle3Location::Degenerate => SegmentTriangleIntersection::Disjoint,
            };
            PredicateOutcome::decided(
                relation,
                max_certainty(certainty, endpoint_certainty),
                max_stage(stage, endpoint_stage),
            )
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Derive a relation from plane-event kind and an optional, trusted triangle-location label.
///
/// Disjoint/coplanar require no location; endpoint/proper require one. Unknown or
/// failed construction is a RelationMismatch. Does not validate the event itself
/// or recompute containment; the outer validator checks the plane event first.
fn relation_from_segment_plane_event(
    plane_event: &SegmentPlaneIntersection,
    triangle_location: Option<Triangle3Location>,
) -> Result<SegmentTriangleIntersection, SegmentTriangleValidationError> {
    match plane_event.relation {
        SegmentPlaneRelation::Disjoint => {
            if triangle_location.is_some() {
                Err(SegmentTriangleValidationError::UnexpectedTriangleLocation)
            } else {
                Ok(SegmentTriangleIntersection::Disjoint)
            }
        }
        SegmentPlaneRelation::Coplanar => {
            if triangle_location.is_some() {
                Err(SegmentTriangleValidationError::UnexpectedTriangleLocation)
            } else {
                Ok(SegmentTriangleIntersection::Coplanar)
            }
        }
        SegmentPlaneRelation::EndpointOnPlane | SegmentPlaneRelation::ProperCrossing => {
            let Some(location) = triangle_location else {
                return Err(SegmentTriangleValidationError::MissingTriangleLocation);
            };
            Ok(relation_from_constructed_segment_triangle_point(
                plane_event.relation,
                location,
            ))
        }
        SegmentPlaneRelation::Unknown | SegmentPlaneRelation::ConstructionFailed => {
            Err(SegmentTriangleValidationError::RelationMismatch)
        }
    }
}

/// Map a single plane candidate's location to segment contact, trusting the inputs.
///
/// Proper+Inside gives Proper; included endpoint locations and proper edge/vertex
/// locations give BoundaryTouch. All other combinations give Disjoint.
fn relation_from_constructed_segment_triangle_point(
    plane_relation: SegmentPlaneRelation,
    location: Triangle3Location,
) -> SegmentTriangleIntersection {
    match (plane_relation, location) {
        (SegmentPlaneRelation::ProperCrossing, Triangle3Location::Inside) => {
            SegmentTriangleIntersection::Proper
        }
        (
            SegmentPlaneRelation::EndpointOnPlane,
            Triangle3Location::Inside | Triangle3Location::OnEdge | Triangle3Location::OnVertex,
        )
        | (
            SegmentPlaneRelation::ProperCrossing,
            Triangle3Location::OnEdge | Triangle3Location::OnVertex,
        ) => SegmentTriangleIntersection::BoundaryTouch,
        _ => SegmentTriangleIntersection::Disjoint,
    }
}

/// Reduce the report's optional-field shape and stored location to a ray relation.
///
/// Coplanar requires no candidate data and On/Zero side/direction. Other candidate
/// branches require point, parameter, and location, then choose origin or transverse
/// mapping by ratio presence. Disjoint without a candidate checks absent data but
/// does not verify side/direction claims. Does not validate parameter values or
/// point geometry, and may derive a relation different from the stored one.
fn relation_from_ray_report_facts(
    report: &RayTriangleIntersectionReport,
) -> Result<RayTriangleIntersection, RayTriangleValidationError> {
    match report.relation {
        RayTriangleIntersection::Coplanar => {
            if report.point.is_some()
                || report.parameter.is_some()
                || report.parameter_ratio.is_some()
            {
                return Err(RayTriangleValidationError::UnexpectedCandidate);
            }
            if report.triangle_location.is_some() {
                return Err(RayTriangleValidationError::UnexpectedTriangleLocation);
            }
            if report.origin_side != Some(PlaneSide::On)
                || report.direction_sign != Some(Sign::Zero)
            {
                return Err(RayTriangleValidationError::RelationMismatch);
            }
            Ok(RayTriangleIntersection::Coplanar)
        }
        RayTriangleIntersection::Proper | RayTriangleIntersection::BoundaryTouch => {
            if report.point.is_none() || report.parameter.is_none() {
                return Err(RayTriangleValidationError::MissingCandidate);
            }
            let Some(location) = report.triangle_location else {
                return Err(RayTriangleValidationError::MissingTriangleLocation);
            };
            Ok(if report.parameter_ratio.is_some() {
                relation_from_constructed_ray_triangle_point(location)
            } else {
                relation_from_ray_origin_triangle_point(location)
            })
        }
        RayTriangleIntersection::Disjoint => {
            if report.point.is_none() {
                if report.parameter.is_some() || report.parameter_ratio.is_some() {
                    return Err(RayTriangleValidationError::UnexpectedCandidate);
                }
                if report.triangle_location.is_some() {
                    return Err(RayTriangleValidationError::UnexpectedTriangleLocation);
                }
                return Ok(RayTriangleIntersection::Disjoint);
            }
            if report.parameter.is_none() {
                return Err(RayTriangleValidationError::MissingCandidate);
            }
            let Some(location) = report.triangle_location else {
                return Err(RayTriangleValidationError::MissingTriangleLocation);
            };
            Ok(if report.parameter_ratio.is_some() {
                relation_from_constructed_ray_triangle_point(location)
            } else {
                relation_from_ray_origin_triangle_point(location)
            })
        }
    }
}

/// Map an included origin-on-plane location to BoundaryTouch and every other label to Disjoint.
fn relation_from_ray_origin_triangle_point(location: Triangle3Location) -> RayTriangleIntersection {
    match location {
        Triangle3Location::Inside | Triangle3Location::OnEdge | Triangle3Location::OnVertex => {
            RayTriangleIntersection::BoundaryTouch
        }
        Triangle3Location::Outside
        | Triangle3Location::OffPlane
        | Triangle3Location::Degenerate => RayTriangleIntersection::Disjoint,
    }
}

/// Locate a transverse line/triangle crossing by scalar triple products on source coordinates.
///
/// Requires a nondegenerate triangle and a line transverse to its support plane.
/// Computes `det(edge_start-origin, edge_end-origin, direction)` for AB, BC, CA
/// before reducing outcomes in that order. Mixed strict signs give Outside; zeros
/// select an edge or vertex (indices 0=A, 1=B, 2=C), and all zero gives Degenerate.
/// Avoiding reconstructed coordinates can preserve useful source cancellations,
/// but unresolved signs still return Unknown. Successful receipts are combined.
fn line_triangle3_crossing_location(
    origin: &Point3,
    direction: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<(Triangle3Location, Option<usize>)> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let mut side = |start: &Point3, end: &Point3| {
        let (sx, sy, sz) = (
            sub_ref(&start.x, &origin.x),
            sub_ref(&start.y, &origin.y),
            sub_ref(&start.z, &origin.z),
        );
        let (ex, ey, ez) = (
            sub_ref(&end.x, &origin.x),
            sub_ref(&end.y, &origin.y),
            sub_ref(&end.z, &origin.z),
        );
        let cross_x = sub_ref(&mul_ref(&ey, &direction.z), &mul_ref(&ez, &direction.y));
        let cross_y = sub_ref(&mul_ref(&ez, &direction.x), &mul_ref(&ex, &direction.z));
        let cross_z = sub_ref(&mul_ref(&ex, &direction.y), &mul_ref(&ey, &direction.x));
        let product = add_ref(
            &add_ref(&mul_ref(&sx, &cross_x), &mul_ref(&sy, &cross_y)),
            &mul_ref(&sz, &cross_z),
        );
        triangle3_sign(
            resolve_real_sign_direct(&product, policy, RefinementNeed::RealRefinement),
            &mut certainty,
            &mut stage,
        )
    };
    let signs = match (side(a, b), side(b, c), side(c, a)) {
        (Ok(ab), Ok(bc), Ok(ca)) => [ab, bc, ca],
        (Err(unknown), _, _) | (_, Err(unknown), _) | (_, _, Err(unknown)) => {
            return unknown.into_outcome();
        }
    };
    let crossing = if signs.contains(&Sign::Positive) && signs.contains(&Sign::Negative) {
        (Triangle3Location::Outside, None)
    } else {
        // Edges are ab, bc, ca: the vertex between two zero edges is the
        // contact.
        match signs.map(|sign| sign == Sign::Zero) {
            [false, false, false] => (Triangle3Location::Inside, None),
            [true, true, false] => (Triangle3Location::OnVertex, Some(1)),
            [false, true, true] => (Triangle3Location::OnVertex, Some(2)),
            [true, false, true] => (Triangle3Location::OnVertex, Some(0)),
            [true, true, true] => (Triangle3Location::Degenerate, None),
            _ => (Triangle3Location::OnEdge, None),
        }
    };
    PredicateOutcome::decided(crossing, certainty, stage)
}

/// Map transverse candidate Inside to Proper, edge/vertex to BoundaryTouch, otherwise Disjoint.
fn relation_from_constructed_ray_triangle_point(
    location: Triangle3Location,
) -> RayTriangleIntersection {
    match location {
        Triangle3Location::Inside => RayTriangleIntersection::Proper,
        Triangle3Location::OnEdge | Triangle3Location::OnVertex => {
            RayTriangleIntersection::BoundaryTouch
        }
        Triangle3Location::Outside
        | Triangle3Location::OffPlane
        | Triangle3Location::Degenerate => RayTriangleIntersection::Disjoint,
    }
}

/// Require parameter >= 0 under policy, mapping negative or Unknown to InvalidParameter.
///
/// Discards the successful ordering receipt.
fn assert_ray_parameter_nonnegative(
    parameter: &Real,
    policy: PredicatePolicy,
) -> Result<(), RayTriangleValidationError> {
    match compare_reals_with_policy(parameter, &Real::from(0), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Err(RayTriangleValidationError::InvalidParameter),
        PredicateOutcome::Decided { .. } => Ok(()),
        PredicateOutcome::Unknown { .. } => Err(RayTriangleValidationError::InvalidParameter),
    }
}

/// Divide the retained ratio under policy and require equality with the parameter.
///
/// Division failure, Unknown, or unequal values become InvalidParameterRatio.
/// Discards successful receipts and does not verify the ratio's source derivation.
fn validate_ray_parameter_ratio(
    ratio: &RayTriangleParameterRatio,
    parameter: &Real,
    policy: PredicatePolicy,
) -> Result<(), RayTriangleValidationError> {
    let quotient = divide_real_with_policy(&ratio.numerator, &ratio.denominator, policy)
        .map_err(|_| RayTriangleValidationError::InvalidParameterRatio)?
        .value()
        .ok_or(RayTriangleValidationError::InvalidParameterRatio)?;
    match compare_reals_with_policy(&quotient, parameter, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(RayTriangleValidationError::InvalidParameterRatio),
        PredicateOutcome::Unknown { .. } => Err(RayTriangleValidationError::InvalidParameterRatio),
    }
}

/// Require an On origin label and a policy-decided zero parameter.
///
/// Other labels, nonzero values, and Unknown become InvalidParameter. Successful
/// comparison receipts are discarded; the source origin is not supplied.
fn validate_ray_origin_parameter(
    origin_side: Option<PlaneSide>,
    parameter: &Real,
    policy: PredicatePolicy,
) -> Result<(), RayTriangleValidationError> {
    if origin_side != Some(PlaneSide::On) {
        return Err(RayTriangleValidationError::InvalidParameter);
    }
    match compare_reals_with_policy(parameter, &Real::from(0), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(RayTriangleValidationError::InvalidParameter),
        PredicateOutcome::Unknown { .. } => Err(RayTriangleValidationError::InvalidParameter),
    }
}

/// Construct `origin + parameter*direction` without checking parameter domain or incidence.
fn ray_point_at(origin: &Point3, direction: &Point3, parameter: &Real) -> Point3 {
    Point3::new(
        add_ref(&origin.x, &mul_ref(&direction.x, parameter)),
        add_ref(&origin.y, &mul_ref(&direction.y, parameter)),
        add_ref(&origin.z, &mul_ref(&direction.z, parameter)),
    )
}

/// Merge a decided sign's receipt into segment accumulators or return a typed Unknown outcome.
fn segment_triangle_sign(
    outcome: PredicateOutcome<Sign>,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, PredicateOutcome<SegmentTriangleIntersection>> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            *certainty = max_certainty(*certainty, value_certainty);
            *stage = max_stage(*stage, value_stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

/// Resolve a ray-construction scalar sign directly under policy with RealRefinement fallback.
fn sign_for_ray_triangle(value: &Real, policy: PredicatePolicy) -> PredicateOutcome<Sign> {
    resolve_real_sign_direct(value, policy, RefinementNeed::RealRefinement)
}

/// Resolve normal components in x/y/z order, returning their combined receipt or first Unknown.
fn triangle3_normal_signs_outcome(
    normal: &Triangle3Normal,
    policy: PredicatePolicy,
) -> PredicateOutcome<[Sign; 3]> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    match real_signs3(
        [&normal.x, &normal.y, &normal.z],
        policy,
        &mut certainty,
        &mut stage,
    ) {
        Ok(signs) => PredicateOutcome::decided(signs, certainty, stage),
        Err(unknown) => unknown.into_outcome(),
    }
}

/// Resolve `normal . ((edge_end-edge_start) x (point-edge_start))` and merge its receipt.
///
/// The normal must correspond to the containing triangle; this helper checks no
/// source association, coplanarity, or normal nondegeneracy.
fn edge_halfspace3_sign(
    normal: &Triangle3Normal,
    start: &Point3,
    end: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, Triangle3Unknown> {
    let ex = sub_ref(&end.x, &start.x);
    let ey = sub_ref(&end.y, &start.y);
    let ez = sub_ref(&end.z, &start.z);
    let px = sub_ref(&point.x, &start.x);
    let py = sub_ref(&point.y, &start.y);
    let pz = sub_ref(&point.z, &start.z);

    let cross_x = sub_ref(&mul_ref(&ey, &pz), &mul_ref(&ez, &py));
    let cross_y = sub_ref(&mul_ref(&ez, &px), &mul_ref(&ex, &pz));
    let cross_z = sub_ref(&mul_ref(&ex, &py), &mul_ref(&ey, &px));

    let nx = mul_ref(&normal.x, &cross_x);
    let ny = mul_ref(&normal.y, &cross_y);
    let nz = mul_ref(&normal.z, &cross_z);
    let nxy = add_ref(&nx, &ny);
    let dot = add_ref(&nxy, &nz);

    triangle3_sign(
        resolve_real_sign_direct(&dot, policy, RefinementNeed::RealRefinement),
        certainty,
        stage,
    )
}

/// Resolve three scalar signs sequentially, accumulating receipts until the first Unknown.
fn real_signs3(
    values: [&Real; 3],
    policy: PredicatePolicy,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<[Sign; 3], Triangle3Unknown> {
    Ok([
        triangle3_sign(
            resolve_real_sign_direct(values[0], policy, RefinementNeed::RealRefinement),
            certainty,
            stage,
        )?,
        triangle3_sign(
            resolve_real_sign_direct(values[1], policy, RefinementNeed::RealRefinement),
            certainty,
            stage,
        )?,
        triangle3_sign(
            resolve_real_sign_direct(values[2], policy, RefinementNeed::RealRefinement),
            certainty,
            stage,
        )?,
    ])
}

/// Extract a sign while merging its receipt, or preserve uncertainty in Triangle3Unknown.
fn triangle3_sign(
    outcome: PredicateOutcome<Sign>,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, Triangle3Unknown> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            *certainty = max_certainty(*certainty, value_certainty);
            *stage = max_stage(*stage, value_stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => Err(Triangle3Unknown { needed, stage }),
    }
}

/// Extract a sign while merging its receipt, or return uncertainty typed for tetrahedron location.
fn tetrahedron_sign(
    outcome: PredicateOutcome<Sign>,
    certainty: &mut Certainty,
    stage: &mut Escalation,
) -> Result<Sign, PredicateOutcome<TetrahedronLocation>> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            *certainty = max_certainty(*certainty, value_certainty);
            *stage = max_stage(*stage, value_stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

/// One decided orientation sign and its receipt, retained until four-way planar aggregation.
#[derive(Clone, Copy)]
struct DecidedSign {
    /// Decided orientation sign.
    sign: Sign,
    /// Qualification of this sign decision.
    certainty: Certainty,
    /// Escalation reached by this sign decision.
    stage: Escalation,
}

/// Unresolved reason/stage transported through spatial triangle sign helpers.
#[derive(Clone, Copy, Debug)]
struct Triangle3Unknown {
    /// Reason the sign query remains unresolved.
    needed: RefinementNeed,
    /// Escalation stage reported by that unresolved query.
    stage: Escalation,
}

impl Triangle3Unknown {
    /// Rebuild an Unknown outcome of the caller's result type without changing reason or stage.
    fn into_outcome<T>(self) -> PredicateOutcome<T> {
        PredicateOutcome::unknown(self.needed, self.stage)
    }
}

/// Select the weakest of four certainties by the shared rank ordering.
///
/// The Exact fallback is unreachable for this fixed nonempty array.
fn combine_certainties(values: [Certainty; 4]) -> Certainty {
    values
        .into_iter()
        .max_by_key(|certainty| certainty_rank(*certainty))
        .unwrap_or(Certainty::Exact)
}

/// Select the furthest of four escalation stages by the shared rank ordering.
///
/// The Undecided fallback is unreachable for this fixed nonempty array.
fn combine_stages(values: [Escalation; 4]) -> Escalation {
    values
        .into_iter()
        .max_by_key(|stage| stage_rank(*stage))
        .unwrap_or(Escalation::Undecided)
}

/// Finite triangle, tetrahedron, retained-report, and unresolved-sign regression controls.
///
/// Most geometry checks compare values only; helper controls separately assert
/// selected receipt ranks. These fixtures do not prove universal exact closure.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::exact_normal_positive;

    /// Policy permitting the terminal approximation in finite geometry controls.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Import a finite binary64 fixture exactly; panic on an invalid scalar literal.
    fn real(value: f64) -> hyperreal::Real {
        hyperreal::Real::try_from(value).expect("finite test Real")
    }

    /// Build a planar fixture from the exact binary values of two finite literals.
    fn p2(x: f64, y: f64) -> Point2 {
        Point2::new(real(x), real(y))
    }

    /// Build a spatial fixture from the exact binary values of three finite literals.
    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    /// Assert normalization of the original unit-circle identity, then return the unresolved control.
    ///
    /// Uses the shared angle-addition zero to keep uncertainty tests distinct from the
    /// now-certified `sin(e)^2 + cos(e)^2 - 1` identity.
    fn terminal_zero() -> Real {
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

    /// Check interior classification for both planar windings under APPROX, comparing values.
    #[test]
    fn classifies_point_inside_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(0.0, 2.0);
        let point = p2(0.5, 0.5);

        assert_eq!(
            crate::classify_point_triangle(&a, &b, &c, &point, APPROX).value(),
            Some(TriangleLocation::Inside)
        );
        assert_eq!(
            crate::classify_point_triangle(&a, &c, &b, &point, APPROX).value(),
            Some(TriangleLocation::Inside)
        );
    }

    /// Check one planar edge-interior query returns OnEdge under APPROX.
    #[test]
    fn classifies_point_on_triangle_edge() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(0.0, 2.0);
        let point = p2(1.0, 0.0);

        assert_eq!(
            crate::classify_point_triangle(&a, &b, &c, &point, APPROX).value(),
            Some(TriangleLocation::OnEdge)
        );
    }

    /// Check five spatial point locations: inside, edge, vertex, outside, and off plane.
    #[test]
    fn classifies_point_inside_3d_triangle() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(2.0, 0.0, 0.0);
        let c = p3(0.0, 2.0, 0.0);

        assert_eq!(
            crate::classify_point_triangle3(&a, &b, &c, &p3(0.5, 0.5, 0.0), APPROX).value(),
            Some(Triangle3Location::Inside)
        );
        assert_eq!(
            crate::classify_point_triangle3(&a, &b, &c, &p3(1.0, 0.0, 0.0), APPROX).value(),
            Some(Triangle3Location::OnEdge)
        );
        assert_eq!(
            crate::classify_point_triangle3(&a, &b, &c, &p3(0.0, 0.0, 0.0), APPROX).value(),
            Some(Triangle3Location::OnVertex)
        );
        assert_eq!(
            crate::classify_point_triangle3(&a, &b, &c, &p3(2.0, 2.0, 0.0), APPROX).value(),
            Some(Triangle3Location::Outside)
        );
        assert_eq!(
            crate::classify_point_triangle3(&a, &b, &c, &p3(0.5, 0.5, 1.0), APPROX).value(),
            Some(Triangle3Location::OffPlane)
        );
    }

    /// Check winding-normal dot signs for upward and downward reference normals.
    #[test]
    fn triangle_winding_normal_sign_classifies_reference_direction() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let up = p3(0.0, 0.0, 1.0);
        let down = p3(0.0, 0.0, -1.0);

        assert_eq!(
            crate::triangle3_winding_normal_sign(&a, &b, &c, &up, APPROX).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            crate::triangle3_winding_normal_sign(&a, &b, &c, &down, APPROX).value(),
            Some(Sign::Negative)
        );
    }

    /// Check retained normal signs are decided and an immediate spatial query is Inside.
    ///
    /// Does not assert a particular receipt or dispatch count.
    #[test]
    fn immediate_triangle3_classifier_reuses_orientation_evidence() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(2.0, 0.0, 0.0);
        let c = p3(0.0, 2.0, 0.0);
        let orientation = crate::triangle3_orientation(&a, &b, &c, APPROX);

        assert!(matches!(
            orientation.normal_signs(),
            PredicateOutcome::Decided { .. }
        ));
        assert_eq!(
            crate::classify_point_triangle3_with_orientation(
                &a,
                &b,
                &c,
                &p3(0.25, 0.25, 0.0),
                &orientation,
                APPROX
            )
            .value(),
            Some(Triangle3Location::Inside)
        );
    }

    /// Inject disallowed approximate normal-sign evidence and check STRICT still returns Inside.
    ///
    /// Checks the resulting value, not the recomputed receipt or dispatch count.
    #[test]
    fn strict_triangle3_classifier_recomputes_rejected_cached_certainty() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(2.0, 0.0, 0.0);
        let c = p3(0.0, 2.0, 0.0);
        let point = p3(0.25, 0.25, 0.0);
        let mut orientation = crate::triangle3_orientation(&a, &b, &c, APPROX);
        let signs = orientation
            .normal_signs()
            .value()
            .expect("exact triangle normal should decide");
        orientation.normal_signs =
            PredicateOutcome::decided(signs, Certainty::Approximate, Escalation::Refined);

        assert_eq!(
            crate::classify_point_triangle3_with_orientation(
                &a,
                &b,
                &c,
                &point,
                &orientation,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Triangle3Location::Inside)
        );
    }

    /// Check coarse segment relations for crossing, boundary, outside, and coplanar controls.
    #[test]
    fn segment_triangle3_intersection_distinguishes_crossing_boundary_and_coplanar() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);

        assert_eq!(
            crate::classify_segment_triangle3_intersection(
                &p3(1.0, 1.0, -1.0),
                &p3(1.0, 1.0, 1.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(SegmentTriangleIntersection::Proper)
        );
        assert_eq!(
            crate::classify_segment_triangle3_intersection(
                &p3(4.0, 0.0, -1.0),
                &p3(4.0, 0.0, 1.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(SegmentTriangleIntersection::BoundaryTouch)
        );
        assert_eq!(
            crate::classify_segment_triangle3_intersection(
                &p3(5.0, 5.0, -1.0),
                &p3(5.0, 5.0, 1.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(SegmentTriangleIntersection::Disjoint)
        );
        assert_eq!(
            crate::classify_segment_triangle3_intersection(
                &p3(1.0, 1.0, 0.0),
                &p3(2.0, 1.0, 0.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(SegmentTriangleIntersection::Coplanar)
        );
    }

    /// Check a crossing report retains event, ratio, point presence, and Inside location.
    ///
    /// Runs local/source validation but does not assert the candidate coordinates.
    #[test]
    fn segment_triangle3_report_retains_crossing_construction() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let p = p3(1.0, 1.0, -1.0);
        let q = p3(1.0, 1.0, 1.0);
        let report =
            crate::classify_segment_triangle3_intersection_report(&p, &q, &a, &b, &c, APPROX)
                .value()
                .expect("exact crossing should decide");

        assert_eq!(report.relation, SegmentTriangleIntersection::Proper);
        assert_eq!(
            report.plane_event.relation,
            SegmentPlaneRelation::ProperCrossing
        );
        assert!(report.has_candidate_point());
        assert_eq!(report.triangle_location, Some(Triangle3Location::Inside));
        assert!(report.plane_event.parameter_ratio.is_some());
        assert_eq!(report.validate(APPROX), Ok(()));
        assert_eq!(
            report.validate_against_sources(&p, &q, &a, &b, &c, APPROX,),
            Ok(())
        );
    }

    /// Distinguish endpoint-vertex and coplanar reports, including location presence and local validation.
    #[test]
    fn segment_triangle3_report_keeps_endpoint_and_coplanar_cases_distinct() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let endpoint = crate::classify_segment_triangle3_intersection_report(
            &p3(4.0, 0.0, 0.0),
            &p3(4.0, 0.0, 3.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("endpoint touch should decide");
        let coplanar = crate::classify_segment_triangle3_intersection_report(
            &p3(1.0, 1.0, 0.0),
            &p3(2.0, 1.0, 0.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("coplanar segment should decide");

        assert_eq!(
            endpoint.relation,
            SegmentTriangleIntersection::BoundaryTouch
        );
        assert_eq!(
            endpoint.plane_event.relation,
            SegmentPlaneRelation::EndpointOnPlane
        );
        assert_eq!(
            endpoint.triangle_location,
            Some(Triangle3Location::OnVertex)
        );
        assert_eq!(endpoint.validate(APPROX), Ok(()));
        assert_eq!(coplanar.relation, SegmentTriangleIntersection::Coplanar);
        assert_eq!(
            coplanar.plane_event.relation,
            SegmentPlaneRelation::Coplanar
        );
        assert_eq!(coplanar.triangle_location, None);
        assert_eq!(coplanar.validate(APPROX), Ok(()));
    }

    /// Check ray relations for forward crossing, away direction, vertex contact, and coplanarity.
    #[test]
    fn ray_triangle3_intersection_distinguishes_direction_and_origin_cases() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);

        assert_eq!(
            crate::classify_ray_triangle3_intersection(
                &p3(1.0, 1.0, -2.0),
                &p3(0.0, 0.0, 1.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(RayTriangleIntersection::Proper)
        );
        assert_eq!(
            crate::classify_ray_triangle3_intersection(
                &p3(1.0, 1.0, -2.0),
                &p3(0.0, 0.0, -1.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(RayTriangleIntersection::Disjoint)
        );
        assert_eq!(
            crate::classify_ray_triangle3_intersection(
                &p3(4.0, 0.0, -2.0),
                &p3(0.0, 0.0, 1.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(RayTriangleIntersection::BoundaryTouch)
        );
        assert_eq!(
            crate::classify_ray_triangle3_intersection(
                &p3(1.0, 1.0, 0.0),
                &p3(1.0, 0.0, 0.0),
                &a,
                &b,
                &c,
                APPROX
            )
            .value(),
            Some(RayTriangleIntersection::Coplanar)
        );
    }

    /// Check a forward report's t = 2, point (1,1,0), side/sign, ratio, location, and replay.
    #[test]
    fn ray_triangle3_report_retains_crossing_construction() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let origin = p3(1.0, 1.0, -2.0);
        let direction = p3(0.0, 0.0, 1.0);
        let report = crate::classify_ray_triangle3_intersection_report(
            &origin, &direction, &a, &b, &c, APPROX,
        )
        .value()
        .expect("exact ray crossing should decide");

        assert_eq!(report.relation, RayTriangleIntersection::Proper);
        assert_eq!(report.origin_side, Some(PlaneSide::Below));
        assert_eq!(report.direction_sign, Some(Sign::Positive));
        assert_eq!(report.parameter, Some(Real::from(2)));
        assert!(report.parameter_ratio.is_some());
        assert_eq!(report.point, Some(p3(1.0, 1.0, 0.0)));
        assert_eq!(report.triangle_location, Some(Triangle3Location::Inside));
        assert!(report.has_candidate_point());
        assert_eq!(report.validate(APPROX), Ok(()));
        assert_eq!(
            report.validate_against_sources(&origin, &direction, &a, &b, &c, APPROX,),
            Ok(())
        );
    }

    /// Check STRICT crossing with a positive exact-normal-form direction retains t = 1/2.
    ///
    /// Checks the candidate's zero z coordinate and local/source validation; no dispatch
    /// count assertion establishes which successful proof path ran.
    #[test]
    fn ray_triangle3_report_reuses_policy_certified_nonzero_direction() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(2.0, 0.0, 0.0);
        let c = p3(0.0, 2.0, 0.0);
        let half = (Real::from(1) / Real::from(2)).unwrap();
        let deep_positive = exact_normal_positive();
        let origin = Point3::new(half.clone(), half.clone(), -(deep_positive.clone() * &half));
        let direction = Point3::new(Real::zero(), Real::zero(), deep_positive);

        let report = crate::classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &a,
            &b,
            &c,
            PredicatePolicy::STRICT,
        )
        .value()
        .expect("the exact-normal direction should construct a ray crossing");

        assert_eq!(report.relation, RayTriangleIntersection::Proper);
        assert_eq!(
            report
                .parameter
                .as_ref()
                .and_then(Real::exact_rational_normal_form),
            half.exact_rational()
        );
        assert_eq!(
            crate::compare_reals(
                &report.point.as_ref().expect("crossing point").z,
                &Real::zero(),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(report.validate(PredicatePolicy::STRICT), Ok(()));
        assert_eq!(
            report.validate_against_sources(
                &origin,
                &direction,
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            Ok(())
        );
    }

    /// Distinguish t = 0 origin contact from coplanarity and reject a forged nonzero origin parameter.
    #[test]
    fn ray_triangle3_report_keeps_origin_touch_and_coplanar_cases_distinct() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let origin_touch = crate::classify_ray_triangle3_intersection_report(
            &p3(1.0, 1.0, 0.0),
            &p3(0.0, 0.0, 1.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("origin touch should decide");
        let coplanar = crate::classify_ray_triangle3_intersection_report(
            &p3(1.0, 1.0, 0.0),
            &p3(1.0, 0.0, 0.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("coplanar ray should decide");

        assert_eq!(
            origin_touch.relation,
            RayTriangleIntersection::BoundaryTouch
        );
        assert_eq!(origin_touch.parameter, Some(Real::from(0)));
        assert_eq!(origin_touch.parameter_ratio, None);
        assert_eq!(
            origin_touch.triangle_location,
            Some(Triangle3Location::Inside)
        );
        assert_eq!(origin_touch.validate(APPROX), Ok(()));
        assert_eq!(coplanar.relation, RayTriangleIntersection::Coplanar);
        assert!(!coplanar.has_candidate_point());
        assert_eq!(coplanar.triangle_location, None);
        assert_eq!(coplanar.validate(APPROX), Ok(()));

        let mut forged_origin_touch = origin_touch.clone();
        forged_origin_touch.parameter = Some(Real::from(1));
        assert_eq!(
            forged_origin_touch.validate(APPROX),
            Err(RayTriangleValidationError::InvalidParameter)
        );
    }

    /// Check parallel/away reports omit candidates while an outside forward crossing retains one.
    #[test]
    fn ray_triangle3_report_validates_parallel_away_and_outside_candidates() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let wrong_direction = crate::classify_ray_triangle3_intersection_report(
            &p3(1.0, 1.0, 1.0),
            &p3(0.0, 0.0, 1.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("wrong-direction ray should decide");
        let parallel_disjoint = crate::classify_ray_triangle3_intersection_report(
            &p3(1.0, 1.0, 1.0),
            &p3(1.0, 0.0, 0.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("parallel disjoint ray should decide");
        let outside_candidate = crate::classify_ray_triangle3_intersection_report(
            &p3(5.0, 5.0, -1.0),
            &p3(0.0, 0.0, 1.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("outside crossing candidate should decide");

        assert_eq!(wrong_direction.relation, RayTriangleIntersection::Disjoint);
        assert!(!wrong_direction.has_candidate_point());
        assert_eq!(wrong_direction.validate(APPROX), Ok(()));
        assert_eq!(
            parallel_disjoint.relation,
            RayTriangleIntersection::Disjoint
        );
        assert!(!parallel_disjoint.has_candidate_point());
        assert_eq!(parallel_disjoint.validate(APPROX), Ok(()));
        assert_eq!(
            outside_candidate.relation,
            RayTriangleIntersection::Disjoint
        );
        assert!(outside_candidate.has_candidate_point());
        assert_eq!(
            outside_candidate.triangle_location,
            Some(Triangle3Location::Outside)
        );
        assert_eq!(outside_candidate.validate(APPROX), Ok(()));
    }

    /// Check a collinear spatial triangle is Degenerate under APPROX.
    #[test]
    fn classifies_degenerate_3d_triangle() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 1.0, 1.0);
        let c = p3(2.0, 2.0, 2.0);

        assert_eq!(
            crate::classify_point_triangle3(&a, &b, &c, &p3(1.0, 1.0, 1.0), APPROX).value(),
            Some(Triangle3Location::Degenerate)
        );
    }

    /// Check tetrahedron interior, face, edge, vertex, and outside values for finite fixtures.
    #[test]
    fn classifies_point_relative_to_tetrahedron() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let d = p3(0.0, 0.0, 1.0);

        assert_eq!(
            crate::classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.1, 0.1, 0.1), APPROX).value(),
            Some(TetrahedronLocation::Inside)
        );
        assert_eq!(
            crate::classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.2, 0.2, 0.0), APPROX).value(),
            Some(TetrahedronLocation::OnFace)
        );
        assert_eq!(
            crate::classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.5, 0.0, 0.0), APPROX).value(),
            Some(TetrahedronLocation::OnEdge)
        );
        assert_eq!(
            crate::classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.0, 0.0, 0.0), APPROX).value(),
            Some(TetrahedronLocation::OnVertex)
        );
        assert_eq!(
            crate::classify_point_tetrahedron(&a, &b, &c, &d, &p3(1.0, 1.0, 1.0), APPROX).value(),
            Some(TetrahedronLocation::Outside)
        );
    }

    /// Check a coplanar four-vertex tetrahedron is Degenerate under APPROX.
    #[test]
    fn classifies_degenerate_tetrahedron() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let d = p3(1.0, 1.0, 0.0);

        assert_eq!(
            crate::classify_point_tetrahedron(&a, &b, &c, &d, &p3(0.25, 0.25, 0.0), APPROX).value(),
            Some(TetrahedronLocation::Degenerate)
        );
    }

    /// Check a collinear planar triangle is Degenerate under APPROX.
    #[test]
    fn classifies_degenerate_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(1.0, 1.0);
        let c = p2(2.0, 2.0);
        let point = p2(1.0, 1.0);

        assert_eq!(
            crate::classify_point_triangle(&a, &b, &c, &point, APPROX).value(),
            Some(TriangleLocation::Degenerate)
        );
    }

    /// Check axis-aligned triangle facts identify degeneracy and the STRICT classifier consumes it.
    #[test]
    fn fact_aware_classifier_uses_structural_triangle_degeneracy() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(5.0, 0.0);
        let point = p2(1.0, 0.0);
        let facts = crate::geometry::triangle2_facts(&a, &b, &c);
        let policy = PredicatePolicy::STRICT;

        assert_eq!(facts.known_degenerate(), Some(true));
        assert_eq!(
            classify_point_triangle_with_policy_and_facts(&a, &b, &c, &point, facts, policy)
                .value(),
            Some(TriangleLocation::Degenerate)
        );
    }

    /// Check retained planar orientation/facts and immediate Inside/Outside classifications.
    #[test]
    fn immediate_triangle_classifier_accepts_cached_orientation() {
        let a = p2(0.0, 0.0);
        let b = p2(3.0, 0.0);
        let c = p2(0.0, 3.0);
        let inside = p2(1.0, 1.0);
        let outside = p2(3.0, 3.0);

        let orientation = crate::orient2(&a, &b, &c, APPROX);
        assert_eq!(orientation.value(), Some(Sign::Positive));
        assert_eq!(
            crate::geometry::triangle2_facts(&a, &b, &c).known_non_degenerate(),
            Some(true)
        );
        assert_eq!(
            crate::classify_point_triangle_with_orientation(
                &a,
                &b,
                &c,
                &inside,
                orientation,
                APPROX
            )
            .value(),
            Some(TriangleLocation::Inside)
        );
        assert_eq!(
            crate::classify_point_triangle_with_orientation(
                &a,
                &b,
                &c,
                &outside,
                orientation,
                APPROX
            )
            .value(),
            Some(TriangleLocation::Outside)
        );
    }

    /// Exercise selected segment-report corruptions, replay mismatch, and unresolved construction branches.
    ///
    /// Includes missing/unexpected locations, failed/unknown plane events, and terminal
    /// zero candidates. These finite controls do not enumerate every malformed report.
    #[test]
    fn retained_segment_triangle_report_rejects_relation_location_and_replay_corruption() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let p = p3(1.0, 1.0, -1.0);
        let q = p3(1.0, 1.0, 1.0);
        let crossing =
            classify_segment_triangle3_intersection_report_with_policy(&p, &q, &a, &b, &c, APPROX)
                .value()
                .expect("rational crossing should decide");

        let mut forged = crossing.clone();
        forged.relation = SegmentTriangleIntersection::Disjoint;
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentTriangleValidationError::RelationMismatch)
        );
        assert_eq!(
            crossing.validate_against_sources(&p, &p3(1.0, 1.0, 2.0), &a, &b, &c, APPROX),
            Err(SegmentTriangleValidationError::SourceReplayMismatch)
        );

        let disjoint = classify_segment_triangle3_intersection_report_with_policy(
            &p3(0.0, 0.0, 2.0),
            &p3(1.0, 0.0, 2.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("same-side segment should decide");
        let mut forged = disjoint;
        forged.triangle_location = Some(Triangle3Location::Outside);
        assert_eq!(
            relation_from_segment_plane_event(&forged.plane_event, forged.triangle_location),
            Err(SegmentTriangleValidationError::UnexpectedTriangleLocation)
        );

        let coplanar = classify_segment_triangle3_intersection_report_with_policy(
            &p3(1.0, 1.0, 0.0),
            &p3(2.0, 1.0, 0.0),
            &a,
            &b,
            &c,
            APPROX,
        )
        .value()
        .expect("coplanar segment should decide");
        assert_eq!(
            relation_from_segment_plane_event(
                &coplanar.plane_event,
                Some(Triangle3Location::Inside),
            ),
            Err(SegmentTriangleValidationError::UnexpectedTriangleLocation)
        );
        assert_eq!(
            relation_from_segment_plane_event(&crossing.plane_event, None),
            Err(SegmentTriangleValidationError::MissingTriangleLocation)
        );

        let mut unsupported = crossing.plane_event.clone();
        unsupported.relation = SegmentPlaneRelation::Unknown;
        assert_eq!(
            relation_from_segment_plane_event(&unsupported, None),
            Err(SegmentTriangleValidationError::RelationMismatch)
        );
        unsupported.relation = SegmentPlaneRelation::ConstructionFailed;
        assert_eq!(
            relation_from_segment_plane_event(&unsupported, None),
            Err(SegmentTriangleValidationError::RelationMismatch)
        );

        for relation in [
            SegmentPlaneRelation::Unknown,
            SegmentPlaneRelation::ConstructionFailed,
        ] {
            let mut event = crossing.plane_event.clone();
            event.relation = relation;
            assert!(matches!(
                finish_segment_triangle3_report(event, &a, &b, &c, PredicatePolicy::STRICT),
                PredicateOutcome::Unknown { .. }
            ));
        }

        let terminal_point = Point3::new(terminal_zero(), Real::from(1), Real::from(0));
        let mut terminal_event = crossing.plane_event.clone();
        terminal_event.relation = SegmentPlaneRelation::EndpointOnPlane;
        terminal_event.point = Some(terminal_point.clone());
        assert!(matches!(
            finish_segment_triangle3_report(terminal_event, &a, &b, &c, PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));

        let retained_plane = triangle_support_plane(&a, &b, &c);
        assert!(matches!(
            classify_segment_triangle3_intersection_from_sides(
                &Point3::new(terminal_zero(), Real::from(1), Real::from(-1)),
                &Point3::new(terminal_zero(), Real::from(1), Real::from(1)),
                &a,
                &b,
                &c,
                Sign::Negative,
                Sign::Positive,
                Some(&retained_plane),
                PredicatePolicy::STRICT,
                Certainty::Exact,
                Escalation::Exact,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Exercise selected ray field-shape, replay, ratio, negative, and unresolved-parameter failures.
    ///
    /// Despite the test name, the finite mutations are not exhaustive over all reports.
    #[test]
    fn retained_ray_report_rejects_every_shape_and_parameter_corruption() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let origin = p3(1.0, 1.0, -2.0);
        let direction = p3(0.0, 0.0, 1.0);
        let crossing = classify_ray_triangle3_intersection_report_with_policy(
            &origin, &direction, &a, &b, &c, APPROX,
        )
        .value()
        .expect("rational ray crossing should decide");

        let mut forged = crossing.clone();
        forged.parameter = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(RayTriangleValidationError::InvalidParameter)
        );

        let mut forged = crossing.clone();
        forged.point = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(RayTriangleValidationError::UnexpectedCandidate)
        );

        let mut forged = crossing.clone();
        forged.origin_side = Some(PlaneSide::On);
        assert_eq!(
            forged.validate(APPROX),
            Err(RayTriangleValidationError::InvalidParameterRatio)
        );

        let mut forged = crossing.clone();
        forged.point = None;
        forged.parameter = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(RayTriangleValidationError::InvalidParameterRatio)
        );

        let mut forged = crossing.clone();
        forged.relation = RayTriangleIntersection::BoundaryTouch;
        assert_eq!(
            forged.validate(APPROX),
            Err(RayTriangleValidationError::RelationMismatch)
        );
        assert_eq!(
            crossing.validate_against_sources(&p3(1.0, 1.0, -3.0), &direction, &a, &b, &c, APPROX,),
            Err(RayTriangleValidationError::SourceReplayMismatch)
        );

        assert_eq!(
            assert_ray_parameter_nonnegative(&Real::from(-1), APPROX),
            Err(RayTriangleValidationError::InvalidParameter)
        );
        assert_eq!(
            assert_ray_parameter_nonnegative(&terminal_zero(), PredicatePolicy::STRICT),
            Err(RayTriangleValidationError::InvalidParameter)
        );
        assert_eq!(
            validate_ray_parameter_ratio(
                &RayTriangleParameterRatio {
                    numerator: Real::from(1),
                    denominator: Real::from(0),
                },
                &Real::from(1),
                APPROX,
            ),
            Err(RayTriangleValidationError::InvalidParameterRatio)
        );
        assert_eq!(
            validate_ray_parameter_ratio(
                &RayTriangleParameterRatio {
                    numerator: Real::from(2),
                    denominator: Real::from(1),
                },
                &Real::from(1),
                APPROX,
            ),
            Err(RayTriangleValidationError::InvalidParameterRatio)
        );
        assert_eq!(
            validate_ray_parameter_ratio(
                &RayTriangleParameterRatio {
                    numerator: terminal_zero(),
                    denominator: Real::from(1),
                },
                &Real::from(0),
                PredicatePolicy::STRICT,
            ),
            Err(RayTriangleValidationError::InvalidParameterRatio)
        );
        assert_eq!(
            validate_ray_origin_parameter(Some(PlaneSide::Above), &Real::from(0), APPROX),
            Err(RayTriangleValidationError::InvalidParameter)
        );
        assert_eq!(
            validate_ray_origin_parameter(Some(PlaneSide::On), &Real::from(1), APPROX),
            Err(RayTriangleValidationError::InvalidParameter)
        );
        assert_eq!(
            validate_ray_origin_parameter(
                Some(PlaneSide::On),
                &terminal_zero(),
                PredicatePolicy::STRICT,
            ),
            Err(RayTriangleValidationError::InvalidParameter)
        );
    }

    /// Visit all six location labels and assert mapper results belong to permitted relation sets.
    ///
    /// Does not assert the expected mapping separately for each individual label.
    #[test]
    fn relation_helpers_cover_all_segment_and_ray_location_collapses() {
        for location in [
            Triangle3Location::Inside,
            Triangle3Location::OnEdge,
            Triangle3Location::OnVertex,
            Triangle3Location::Outside,
            Triangle3Location::OffPlane,
            Triangle3Location::Degenerate,
        ] {
            let crossing = relation_from_constructed_segment_triangle_point(
                SegmentPlaneRelation::ProperCrossing,
                location,
            );
            let endpoint = relation_from_constructed_segment_triangle_point(
                SegmentPlaneRelation::EndpointOnPlane,
                location,
            );
            let ray_origin = relation_from_ray_origin_triangle_point(location);
            let ray_crossing = relation_from_constructed_ray_triangle_point(location);
            assert!(matches!(
                crossing,
                SegmentTriangleIntersection::Proper
                    | SegmentTriangleIntersection::BoundaryTouch
                    | SegmentTriangleIntersection::Disjoint
            ));
            assert!(matches!(
                endpoint,
                SegmentTriangleIntersection::BoundaryTouch | SegmentTriangleIntersection::Disjoint
            ));
            assert!(matches!(
                ray_origin,
                RayTriangleIntersection::BoundaryTouch | RayTriangleIntersection::Disjoint
            ));
            assert!(matches!(
                ray_crossing,
                RayTriangleIntersection::Proper
                    | RayTriangleIntersection::BoundaryTouch
                    | RayTriangleIntersection::Disjoint
            ));
        }
    }

    /// Exercise selected optional-field shapes in the relation reducers directly.
    ///
    /// Bypasses outer scalar validation and includes derived relations differing from
    /// stored labels; does not enumerate all field combinations or prove source geometry.
    #[test]
    fn retained_report_fact_reduction_rejects_every_optional_field_shape() {
        let candidate = p3(0.0, 0.0, 0.0);
        let blank = RayTriangleIntersectionReport {
            relation: RayTriangleIntersection::Coplanar,
            origin_side: Some(PlaneSide::On),
            direction_sign: Some(Sign::Zero),
            parameter: None,
            parameter_ratio: None,
            point: None,
            triangle_location: None,
        };
        assert_eq!(
            relation_from_ray_report_facts(&blank),
            Ok(RayTriangleIntersection::Coplanar)
        );

        let mut report = blank.clone();
        report.point = Some(candidate.clone());
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::UnexpectedCandidate)
        );
        let mut report = blank.clone();
        report.triangle_location = Some(Triangle3Location::Inside);
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::UnexpectedTriangleLocation)
        );
        let mut report = blank.clone();
        report.origin_side = Some(PlaneSide::Above);
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::RelationMismatch)
        );

        let crossing_ratio = RayTriangleParameterRatio {
            numerator: Real::from(1),
            denominator: Real::from(1),
        };
        let crossing = RayTriangleIntersectionReport {
            relation: RayTriangleIntersection::Proper,
            origin_side: Some(PlaneSide::Below),
            direction_sign: Some(Sign::Positive),
            parameter: Some(Real::from(1)),
            parameter_ratio: Some(crossing_ratio.clone()),
            point: Some(candidate.clone()),
            triangle_location: Some(Triangle3Location::Inside),
        };
        assert_eq!(
            relation_from_ray_report_facts(&crossing),
            Ok(RayTriangleIntersection::Proper)
        );
        let mut report = crossing.clone();
        report.point = None;
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::MissingCandidate)
        );
        let mut report = crossing.clone();
        report.triangle_location = None;
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::MissingTriangleLocation)
        );
        let mut report = crossing.clone();
        report.parameter_ratio = None;
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Ok(RayTriangleIntersection::BoundaryTouch)
        );

        let disjoint = RayTriangleIntersectionReport {
            relation: RayTriangleIntersection::Disjoint,
            origin_side: Some(PlaneSide::Above),
            direction_sign: Some(Sign::Positive),
            parameter: None,
            parameter_ratio: None,
            point: None,
            triangle_location: None,
        };
        assert_eq!(
            relation_from_ray_report_facts(&disjoint),
            Ok(RayTriangleIntersection::Disjoint)
        );
        let mut report = disjoint.clone();
        report.parameter = Some(Real::from(1));
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::UnexpectedCandidate)
        );
        let mut report = disjoint.clone();
        report.triangle_location = Some(Triangle3Location::Outside);
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::UnexpectedTriangleLocation)
        );
        let mut report = disjoint.clone();
        report.point = Some(candidate.clone());
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::MissingCandidate)
        );
        let mut report = disjoint;
        report.point = Some(candidate);
        report.parameter = Some(Real::from(1));
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Err(RayTriangleValidationError::MissingTriangleLocation)
        );
        report.triangle_location = Some(Triangle3Location::Inside);
        assert_eq!(
            relation_from_ray_report_facts(&report),
            Ok(RayTriangleIntersection::BoundaryTouch)
        );

        let disjoint_plane_event = SegmentPlaneIntersection {
            relation: SegmentPlaneRelation::Disjoint,
            point: None,
            parameter: None,
            parameter_ratio: None,
            endpoint_on_plane: None,
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
            construction_failure: None,
        };
        assert_eq!(
            relation_from_segment_plane_event(&disjoint_plane_event, None),
            Ok(SegmentTriangleIntersection::Disjoint)
        );
    }

    /// Check STRICT propagates retained, triangle, and each planar edge uncertainty; check fact-assisted success.
    #[test]
    fn strict_planar_triangle_classifier_propagates_triangle_and_each_edge_uncertainty() {
        let a = p2(0.0, 0.0);
        let b = p2(4.0, 0.0);
        let c = p2(0.0, 4.0);
        let unknown =
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided);
        assert!(matches!(
            classify_point_triangle_with_orientation(
                &a,
                &b,
                &c,
                &p2(1.0, 1.0),
                unknown,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let unresolved_y = Point2::new(real(1.0), terminal_zero());
        assert!(matches!(
            classify_point_triangle_with_policy(&a, &b, &c, &unresolved_y, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        let unresolved_bc = Point2::new(terminal_zero(), real(4.0));
        assert!(matches!(
            classify_point_triangle_with_policy(
                &a,
                &b,
                &c,
                &unresolved_bc,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        let unresolved_ca = Point2::new(terminal_zero(), real(1.0));
        assert!(matches!(
            classify_point_triangle_with_policy(
                &a,
                &b,
                &c,
                &unresolved_ca,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let uncertain_triangle = Point2::new(real(0.0), terminal_zero());
        assert!(matches!(
            classify_point_triangle_with_policy(
                &a,
                &b,
                &uncertain_triangle,
                &p2(1.0, 1.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let facts = crate::geometry::triangle2_facts(&a, &b, &c);
        assert_eq!(
            classify_point_triangle_with_policy_and_facts(
                &a,
                &b,
                &c,
                &p2(1.0, 1.0),
                facts,
                APPROX,
            )
            .value(),
            Some(TriangleLocation::Inside)
        );
    }

    /// Check STRICT spatial normal/plane/edge and tetrahedron determinant uncertainties remain Unknown.
    #[test]
    fn strict_spatial_triangle_and_tetrahedron_classifiers_propagate_uncertainty() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);

        let uncertain_normal = Point3::new(real(0.0), terminal_zero(), real(0.0));
        assert!(matches!(
            classify_point_triangle3_with_policy(
                &a,
                &b,
                &uncertain_normal,
                &a,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        let off_plane_unknown = Point3::new(real(0.25), real(0.25), terminal_zero());
        assert!(matches!(
            classify_point_triangle3_with_policy(
                &a,
                &b,
                &c,
                &off_plane_unknown,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        let edge_unknown = Point3::new(terminal_zero(), real(0.25), real(0.0));
        assert!(matches!(
            classify_point_triangle3_with_policy(
                &a,
                &b,
                &c,
                &edge_unknown,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let d = p3(0.0, 0.0, 1.0);
        let uncertain_d = Point3::new(real(0.0), real(0.0), terminal_zero());
        assert!(matches!(
            classify_point_tetrahedron_with_policy(
                &a,
                &b,
                &c,
                &uncertain_d,
                &a,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let queries = [
            Point3::new(real(0.0), real(0.0), terminal_zero()),
            Point3::new(real(0.0), terminal_zero(), real(0.0)),
            Point3::new(terminal_zero(), real(0.0), real(0.0)),
            Point3::new(&Real::from(1) + &terminal_zero(), real(0.0), real(0.0)),
        ];
        for query in &queries {
            assert!(matches!(
                classify_point_tetrahedron_with_policy(
                    &a,
                    &b,
                    &c,
                    &d,
                    query,
                    PredicatePolicy::STRICT,
                ),
                PredicateOutcome::Unknown { .. }
            ));
        }
    }

    /// Check unresolved segment/ray sources propagate Unknown and an inconsistent retained plane is Unsupported.
    #[test]
    fn strict_segment_and_ray_triangle_apis_propagate_unresolved_constructions() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(4.0, 0.0, 0.0);
        let c = p3(0.0, 4.0, 0.0);
        let unresolved = Point3::new(real(1.0), real(1.0), terminal_zero());

        assert!(matches!(
            classify_segment_triangle3_intersection_report_with_policy(
                &unresolved,
                &p3(1.0, 1.0, 1.0),
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_segment_triangle3_intersection_with_policy(
                &unresolved,
                &p3(1.0, 1.0, 1.0),
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_segment_triangle3_intersection_with_policy(
                &p3(1.0, 1.0, 1.0),
                &unresolved,
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let plane = triangle_support_plane(&a, &b, &c);
        assert!(matches!(
            classify_segment_triangle3_intersection_with_preclassified_sides(
                [&p3(0.0, 0.0, 1.0), &p3(1.0, 0.0, 1.0)],
                [&a, &b, &c],
                [PlaneSide::Below, PlaneSide::Above],
                &plane,
                APPROX,
            ),
            PredicateOutcome::Unknown {
                needed: RefinementNeed::Unsupported,
                ..
            }
        ));

        let unresolved_direction = Point3::new(real(0.0), real(0.0), terminal_zero());
        assert!(matches!(
            classify_ray_triangle3_intersection_report_with_policy(
                &p3(1.0, 1.0, -1.0),
                &unresolved_direction,
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_ray_triangle3_intersection_with_policy(
                &p3(1.0, 1.0, -1.0),
                &unresolved_direction,
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let origin_unknown = Point3::new(terminal_zero(), real(1.0), real(0.0));
        assert!(matches!(
            classify_ray_triangle3_intersection_report_with_policy(
                &origin_unknown,
                &p3(0.0, 0.0, 1.0),
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        let crossing_unknown = Point3::new(terminal_zero(), real(1.0), real(-1.0));
        assert!(matches!(
            classify_ray_triangle3_intersection_report_with_policy(
                &crossing_unknown,
                &p3(0.0, 0.0, 1.0),
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            segment_endpoint_triangle_relation(
                &origin_unknown,
                &a,
                &b,
                &c,
                PredicatePolicy::STRICT,
                Certainty::Exact,
                Escalation::Exact,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Check uncertainty transport at each three-sign position and explicit certainty/stage rank controls.
    #[test]
    fn triangle_sign_and_trace_helpers_cover_unknown_positions_and_all_ranks() {
        for values in [
            [terminal_zero(), Real::from(0), Real::from(0)],
            [Real::from(1), terminal_zero(), Real::from(0)],
            [Real::from(1), Real::from(1), terminal_zero()],
        ] {
            let mut certainty = Certainty::Exact;
            let mut stage = Escalation::Structural;
            assert!(matches!(
                real_signs3(
                    [&values[0], &values[1], &values[2]],
                    PredicatePolicy::STRICT,
                    &mut certainty,
                    &mut stage,
                ),
                Err(Triangle3Unknown { .. })
            ));
        }

        let mut certainty = Certainty::Exact;
        let mut stage = Escalation::Structural;
        let unknown =
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided);
        assert!(matches!(
            segment_triangle_sign(unknown, &mut certainty, &mut stage),
            Err(PredicateOutcome::Unknown { .. })
        ));
        assert!(matches!(
            triangle3_sign(unknown, &mut certainty, &mut stage),
            Err(Triangle3Unknown { .. })
        ));
        assert!(matches!(
            tetrahedron_sign(unknown, &mut certainty, &mut stage),
            Err(PredicateOutcome::Unknown { .. })
        ));

        assert_eq!(combine_certainties([Certainty::Exact; 4]), Certainty::Exact);
        assert_eq!(
            max_certainty(Certainty::Exact, Certainty::Filtered),
            Certainty::Filtered
        );
        assert_eq!(certainty_rank(Certainty::Exact), 0);
        assert_eq!(certainty_rank(Certainty::Filtered), 1);
        assert_eq!(certainty_rank(Certainty::Approximate), 2);
        assert_eq!(combine_stages([Escalation::Exact; 4]), Escalation::Exact);
        assert_eq!(
            max_stage(Escalation::Filter, Escalation::Refined),
            Escalation::Refined
        );
        assert_eq!(stage_rank(Escalation::Structural), 0);
        assert_eq!(stage_rank(Escalation::Filter), 1);
        assert_eq!(stage_rank(Escalation::Exact), 2);
        assert_eq!(stage_rank(Escalation::Refined), 3);
        assert_eq!(stage_rank(Escalation::Undecided), 4);
    }
}
