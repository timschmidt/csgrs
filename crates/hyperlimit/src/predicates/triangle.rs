//! Triangle classification predicates.

use crate::predicate::PredicatePolicy;
use core::cmp::Ordering;

use crate::classify::{
    PlaneSide, RayTriangleIntersection, SegmentTriangleIntersection, TetrahedronLocation,
    Triangle3Location, TriangleLocation,
};
use crate::geometry::{HomogeneousLine3, Plane3, Point2, Point3, Triangle2Facts};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::compare_reals_with_policy;
use crate::predicates::orient::{orient2d_with_policy, orient3d_with_policy};
use crate::predicates::segment_plane::{
    SegmentPlaneIntersection, SegmentPlaneRelation, SegmentPlaneValidationError,
    intersect_segment_with_plane_values_with_policy, point_plane_value,
};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{resolve_composite_policy, resolve_real_sign_direct};
use hyperreal::Real;

/// Derived orientation evidence for one ordered 3D triangle.
///
/// This value owns the exact winding normal and its certified component signs,
/// but not the triangle vertices. Retain it when classifying many points
/// against the same triangle.
#[derive(Clone, Debug)]
pub struct Triangle3Orientation {
    normal: Triangle3Normal,
    normal_signs: PredicateOutcome<[Sign; 3]>,
}

impl Triangle3Orientation {
    /// Return the certified signs of the winding-normal components.
    pub const fn normal_signs(&self) -> PredicateOutcome<[Sign; 3]> {
        self.normal_signs
    }
}

/// Structural inconsistency in a retained segment/triangle report.
///
/// The report validates the composition of a segment/plane construction event
/// with a point/triangle classifier. The exact predicate layer owns replayable evidence,
/// while mesh, voxel, and boolean crates own any topology mutation derived from
/// that evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentTriangleValidationError {
    /// The retained segment/plane event is internally inconsistent.
    PlaneEventInvalid(SegmentPlaneValidationError),
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

/// Report-bearing segment/triangle intersection classification.
///
/// The coarse [`SegmentTriangleIntersection`] relation is kept for cheap
/// callers, while this report retains the exact segment/plane construction and
/// the point/triangle location that justified it. Proper crossings keep the
/// determinant-ratio segment parameter through [`SegmentPlaneIntersection`].
/// This is the construction-preserving counterpart to the
/// standard triangle-intersection decomposition and retains evidence before
/// topology is changed.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentTriangleIntersectionReport {
    /// Coarse segment/triangle relation.
    pub relation: SegmentTriangleIntersection,
    /// Exact segment against triangle-supporting-plane event.
    pub plane_event: SegmentPlaneIntersection,
    /// Location of the constructed endpoint or proper-crossing point relative
    /// to the closed triangle.
    pub triangle_location: Option<Triangle3Location>,
}

impl SegmentTriangleIntersectionReport {
    /// Validate retained construction and classification facts.
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

    /// Replay this report against source segment and triangle geometry.
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

    /// Return whether this report retained a constructed candidate point.
    pub fn has_candidate_point(&self) -> bool {
        self.plane_event.point.is_some()
    }
}

/// Structural inconsistency in a retained ray/triangle report.
///
/// The ray report validates the exact ray/support-plane construction before
/// trusting the coarse triangle relation. Topology-facing callers receive a
/// replayable certificate-shaped object instead of an untestable floating
/// intersection.
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
    /// The ray parameter ratio was missing or present for the wrong event.
    InvalidParameterRatio,
    /// The retained parameter was negative or could not be ordered.
    InvalidParameter,
    /// Retained construction facts derive a different coarse relation.
    RelationMismatch,
    /// Recomputing from source geometry did not reproduce this report.
    SourceReplayMismatch,
}

/// Exact numerator and denominator for a ray/support-plane crossing.
///
/// For ray `r(t) = origin + t * direction` and supporting plane expression
/// `E(x) = normal . x + offset`, the strict crossing parameter is
/// `t = -E(origin) / (normal . direction)`. Retaining the ratio keeps the
/// division auditable and separates certified predicates from constructed geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct RayTriangleParameterRatio {
    /// Numerator `-E(origin)`.
    pub numerator: Real,
    /// Denominator `normal . direction`.
    pub denominator: Real,
}

/// Report-bearing ray/triangle intersection classification.
///
/// The coarse [`RayTriangleIntersection`] relation remains available for cheap
/// callers. This report retains the exact ray/plane parameter, constructed
/// candidate point, and point/triangle replay location when a single candidate
/// exists. The decomposition first certifies the
/// supporting-plane event, then replay containment with exact predicates.
#[derive(Clone, Debug, PartialEq)]
pub struct RayTriangleIntersectionReport {
    /// Coarse ray/triangle relation.
    pub relation: RayTriangleIntersection,
    /// Certified side of the ray origin relative to the triangle's supporting
    /// plane.
    pub origin_side: Option<PlaneSide>,
    /// Certified sign of `normal . direction`.
    pub direction_sign: Option<Sign>,
    /// Exact ray parameter for retained candidate-point events.
    pub parameter: Option<Real>,
    /// Exact numerator/denominator for strict support-plane crossings.
    pub parameter_ratio: Option<RayTriangleParameterRatio>,
    /// Exact candidate point for origin-on-plane or strict crossing events.
    pub point: Option<Point3>,
    /// Location of [`Self::point`] relative to the closed triangle.
    pub triangle_location: Option<Triangle3Location>,
}

impl RayTriangleIntersectionReport {
    /// Validate retained construction and classification facts.
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

    /// Replay this report against source ray and triangle geometry.
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

    /// Return whether this report retained a constructed candidate point.
    pub fn has_candidate_point(&self) -> bool {
        self.point.is_some()
    }
}

/// Classify `point` relative to triangle `abc` with an explicit escalation
/// policy.
pub fn classify_point_triangle_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_impl(a, b, c, point, policy, None, None)
}

/// Derive reusable orientation evidence for ordered triangle `abc`.
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

/// Classify `point` relative to triangle `abc` using retained orientation.
///
/// `orientation` must have been derived from the same ordered vertices with
/// [`triangle3_orientation`]. The derived normal and its signs are reused while
/// the point query remains immediate.
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

/// Classify `point` relative to the 3D triangle `abc` with an explicit
/// predicate escalation policy.
///
/// The classifier first certifies that `abc` has a nonzero normal, then
/// certifies that `point` is on the supporting plane. Containment is decided by
/// exact signs of `normal . ((edge_end - edge_start) x (point - edge_start))`
/// for each oriented edge.
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

/// Decide the policy-controlled sign of a triangle winding normal dotted with a
/// reference normal.
///
/// The triangle normal is `(b - a) x (c - a)`. The returned sign is positive
/// when that winding agrees with `reference_normal`, negative when it is
/// reversed, and zero when the dot product is exactly zero.
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

/// Policy-controlled report-bearing variant of
/// [`classify_segment_triangle3_intersection_with_policy`].
///
/// Endpoint signs are first certified against the triangle's supporting plane.
/// A single candidate point is retained only for endpoint-on-plane and proper
/// crossing events, using an exact segment/plane determinant ratio. The candidate is then replayed
/// through the exact 3D point/triangle classifier before the coarse relation is
/// accepted.
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

/// Classify the intersection of a closed 3D segment `pq` with triangle `abc`
/// using an explicit predicate policy.
///
/// The classifier first uses exact orientation signs to locate the segment
/// endpoints relative to the triangle's supporting plane. A strict crossing is
/// lowered through a homogeneous line-plane construction and only then through
/// the existing exact point/triangle classifier. Coplanar cases are reported as
/// a first-class exact relation instead of being projected with a primitive
/// tolerance, keeping planar arrangement ownership in higher crates.
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

/// Classify an edge against a triangle when a caller has already certified
/// both endpoint sides against the supplied supporting plane.
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
        Ok(intersection) => {
            match classify_point_triangle3_with_policy(a, b, c, &intersection, policy) {
                PredicateOutcome::Decided {
                    value,
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

/// Policy-controlled report-bearing variant of
/// [`classify_ray_triangle3_intersection_with_policy`].
///
/// The ray first certifies the origin side and the sign of
/// `normal . direction` against the triangle's supporting plane. A candidate is
/// retained only when the origin lies on the plane or the ray parameter
/// `-E(origin) / (normal . direction)` is certified by sign logic to be
/// nonnegative. The retained ratio and point/triangle replay preserve evidence
/// before topology changes, while keeping the classic
/// ray-plane-then-triangle-containment decomposition explicit.
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
    let parameter = (&numerator / &direction_expression)
        .expect("a certified nonzero ray/plane denominator is a valid divisor");
    let intersection = ray_point_at(origin, direction, &parameter);
    match classify_point_triangle3_with_policy(a, b, c, &intersection, policy) {
        PredicateOutcome::Decided { value, .. } => {
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

/// Classify the intersection of a 3D ray with triangle `abc` using an explicit
/// predicate policy.
///
/// The ray parameter is tested without division by comparing the signs of
/// `-(plane(origin))` and `normal.direction`. The actual candidate point is
/// constructed only after the parameter is certified nonnegative. The final triangle
/// containment reuses the existing exact edge-halfspace classifier.
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

/// Classify `point` relative to tetrahedron `abcd` with an explicit predicate
/// escalation policy.
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

/// Classify `point` relative to triangle `abc` using a retained orientation.
///
/// `orientation` must be the exact [`crate::orient2`] outcome for the same
/// ordered vertices `a`, `b`, and `c`. Retaining that compact outcome avoids
/// recomputing the triangle's fixed orientation across repeated point queries
/// while keeping the query itself immediate.
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

/// Classify `point` relative to triangle `abc` with both an explicit policy and
/// cached structural facts.
///
/// Cached facts can certify structurally degenerate triangles without building
/// the orientation determinant. Non-degenerate containment still uses exact
/// orientation signs for the three triangle edges.
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

#[derive(Clone, Debug)]
struct Triangle3Normal {
    x: Real,
    y: Real,
    z: Real,
}

fn triangle3_normal(a: &Point3, b: &Point3, c: &Point3) -> Triangle3Normal {
    let abx = sub_ref(&b.x, &a.x);
    let aby = sub_ref(&b.y, &a.y);
    let abz = sub_ref(&b.z, &a.z);
    let acx = sub_ref(&c.x, &a.x);
    let acy = sub_ref(&c.y, &a.y);
    let acz = sub_ref(&c.z, &a.z);

    Triangle3Normal {
        x: sub_ref(&mul_ref(&aby, &acz), &mul_ref(&abz, &acy)),
        y: sub_ref(&mul_ref(&abz, &acx), &mul_ref(&abx, &acz)),
        z: sub_ref(&mul_ref(&abx, &acy), &mul_ref(&aby, &acx)),
    }
}

fn triangle_support_plane(a: &Point3, b: &Point3, c: &Point3) -> Plane3 {
    let normal = triangle3_normal(a, b, c);
    let normal_point = Point3::new(normal.x, normal.y, normal.z);
    let offset = neg_real(&dot_point3(&normal_point, a));
    Plane3::new(normal_point, offset)
}

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

fn neg_real(value: &Real) -> Real {
    sub_ref(&Real::from(0), value)
}

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

fn validate_ray_parameter_ratio(
    ratio: &RayTriangleParameterRatio,
    parameter: &Real,
    policy: PredicatePolicy,
) -> Result<(), RayTriangleValidationError> {
    let quotient = (&ratio.numerator / &ratio.denominator)
        .map_err(|_| RayTriangleValidationError::InvalidParameterRatio)?;
    match compare_reals_with_policy(&quotient, parameter, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(RayTriangleValidationError::InvalidParameterRatio),
        PredicateOutcome::Unknown { .. } => Err(RayTriangleValidationError::InvalidParameterRatio),
    }
}

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

fn ray_point_at(origin: &Point3, direction: &Point3, parameter: &Real) -> Point3 {
    Point3::new(
        add_ref(&origin.x, &mul_ref(&direction.x, parameter)),
        add_ref(&origin.y, &mul_ref(&direction.y, parameter)),
        add_ref(&origin.z, &mul_ref(&direction.z, parameter)),
    )
}

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

fn sign_for_ray_triangle(value: &Real, policy: PredicatePolicy) -> PredicateOutcome<Sign> {
    resolve_real_sign_direct(value, policy, RefinementNeed::RealRefinement)
}

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

#[derive(Clone, Copy)]
struct DecidedSign {
    sign: Sign,
    certainty: Certainty,
    stage: Escalation,
}

#[derive(Clone, Copy, Debug)]
struct Triangle3Unknown {
    needed: RefinementNeed,
    stage: Escalation,
}

impl Triangle3Unknown {
    fn into_outcome<T>(self) -> PredicateOutcome<T> {
        PredicateOutcome::unknown(self.needed, self.stage)
    }
}

fn combine_certainties(values: [Certainty; 4]) -> Certainty {
    values
        .into_iter()
        .max_by_key(|certainty| certainty_rank(*certainty))
        .unwrap_or(Certainty::Exact)
}

fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
        Certainty::Approximate => 2,
    }
}

fn combine_stages(values: [Escalation; 4]) -> Escalation {
    values
        .into_iter()
        .max_by_key(|stage| stage_rank(*stage))
        .unwrap_or(Escalation::Undecided)
}

fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    fn real(value: f64) -> hyperreal::Real {
        hyperreal::Real::try_from(value).expect("finite test Real")
    }

    fn p2(x: f64, y: f64) -> Point2 {
        Point2::new(real(x), real(y))
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    fn terminal_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        &sine * &sine + &cosine * &cosine - Real::one()
    }

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
