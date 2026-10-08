//! Exact segment/plane construction helpers.
//!
//! A segment/plane crossing is the construction counterpart to plane-side
//! predicates. Endpoint sides decide the topology, and only a certified proper
//! crossing constructs `p0 + t (p1 - p0)` with `t = d0 / (d0 - d1)`, where
//! `d0` and `d1` are exact oriented plane evaluations. Predicates decide the
//! combinatorics, and constructions preserve the arithmetic structure needed by
//! later predicates.

use crate::classify::PlaneSide;
use crate::geometry::{Plane3, Point3};
use crate::oriented_plane3_evidence;
use crate::predicate::PredicatePolicy;
use crate::predicates::order::{compare_reals_with_policy, divide_real_with_policy};
use crate::predicates::orient::orient3d_with_policy;
use hyperreal::{Problem, Real};

/// Exact segment relation to an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentPlaneRelation {
    /// Both endpoints are on the same strict side of the plane.
    Disjoint,
    /// Both endpoints lie on the plane.
    Coplanar,
    /// Exactly one endpoint lies on the plane.
    EndpointOnPlane,
    /// The endpoints are on opposite strict sides and an exact point was built.
    ProperCrossing,
    /// At least one endpoint predicate was undecided.
    Unknown,
    /// The side predicates certified a crossing, but exact construction failed.
    ConstructionFailed,
}

/// Exact construction failure for a certified segment/plane crossing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentPlaneConstructionFailure {
    /// The determinant denominator `d0 - d1` was certified as zero.
    ZeroDenominator,
    /// The exact scalar backend could not form `d0 / (d0 - d1)`.
    ParameterDivisionFailed,
}

/// Structural inconsistency in a retained segment/plane construction event.
///
/// This validates the event record produced by the construction layer rather
/// than recomputing the geometry. A segment/plane event whose relation,
/// endpoint-side facts, exact point, and parameter disagree is not a safe
/// construction artifact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentPlaneValidationError {
    /// An unknown event retained a decided endpoint side for both endpoints.
    UnknownHasDecidedSides,
    /// A disjoint event was not certified by two endpoints on the same strict
    /// side of the plane.
    DisjointSideFactsMismatch,
    /// A coplanar event was not certified by both endpoints on the plane.
    CoplanarSideFactsMismatch,
    /// An endpoint event was missing an endpoint index or used an invalid one.
    InvalidEndpointIndex,
    /// An endpoint event did not retain the exact endpoint point and parameter.
    MissingEndpointConstruction,
    /// An endpoint event's side facts do not put the chosen endpoint on the
    /// plane and the other endpoint off or on the plane.
    EndpointSideFactsMismatch,
    /// A proper crossing event was not certified by opposite strict endpoint
    /// sides.
    ProperCrossingSideFactsMismatch,
    /// A proper crossing event did not retain its exact point and parameter.
    MissingProperCrossingConstruction,
    /// A proper crossing retained a segment parameter outside the open unit
    /// interval.
    ProperCrossingParameterOutOfRange,
    /// A proper crossing did not retain the determinant numerator and
    /// denominator that produced its segment parameter.
    MissingProperCrossingRatio,
    /// A retained determinant ratio has a zero denominator or does not equal
    /// the retained segment parameter.
    ProperCrossingRatioMismatch,
    /// A construction-failed event was not certified by opposite strict
    /// endpoint sides.
    ConstructionFailedSideFactsMismatch,
    /// A construction-failed event did not retain a structured failure reason.
    MissingConstructionFailureReason,
    /// A relation that should not carry constructed geometry retained one.
    UnexpectedConstruction,
    /// A relation that did not fail retained a construction-failure reason.
    UnexpectedConstructionFailureReason,
    /// Recomputing the event from the supplied segment and plane did not
    /// reproduce this retained construction record.
    SourceReplayMismatch,
}

/// Certified segment/plane event with retained construction data.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentPlaneIntersection {
    /// Coarse relation between the closed segment and oriented plane.
    pub relation: SegmentPlaneRelation,
    /// Exact intersection point for endpoint and proper-crossing events.
    pub point: Option<Point3>,
    /// Exact segment parameter `t` where `p(t) = p0 + t * (p1 - p0)`.
    pub parameter: Option<Real>,
    /// Determinant ratio that produced [`Self::parameter`] for proper
    /// crossings.
    pub parameter_ratio: Option<SegmentPlaneParameterRatio>,
    /// Endpoint index, `0` or `1`, when [`SegmentPlaneRelation::EndpointOnPlane`].
    pub endpoint_on_plane: Option<usize>,
    /// Certified side for each segment endpoint, or `None` when undecided.
    pub endpoint_sides: [Option<PlaneSide>; 2],
    /// Structured construction failure retained when a certified crossing
    /// could not produce a split point.
    pub construction_failure: Option<SegmentPlaneConstructionFailure>,
}

/// Determinant numerator and denominator for a segment/plane crossing.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentPlaneParameterRatio {
    /// Oriented plane value at the first segment endpoint.
    pub numerator: Real,
    /// Difference between first and second endpoint plane values.
    pub denominator: Real,
}

impl SegmentPlaneIntersection {
    /// Validate relation, endpoint-side, and construction-field consistency.
    pub fn validate(&self, policy: PredicatePolicy) -> Result<(), SegmentPlaneValidationError> {
        match self.relation {
            SegmentPlaneRelation::Unknown => {
                if self.endpoint_sides.iter().all(Option::is_some) {
                    return Err(SegmentPlaneValidationError::UnknownHasDecidedSides);
                }
                self.expect_no_construction()
            }
            SegmentPlaneRelation::Disjoint => {
                match self.endpoint_sides {
                    [Some(PlaneSide::Above), Some(PlaneSide::Above)]
                    | [Some(PlaneSide::Below), Some(PlaneSide::Below)] => {}
                    _ => return Err(SegmentPlaneValidationError::DisjointSideFactsMismatch),
                }
                self.expect_no_construction()
            }
            SegmentPlaneRelation::Coplanar => {
                if self.endpoint_sides != [Some(PlaneSide::On), Some(PlaneSide::On)] {
                    return Err(SegmentPlaneValidationError::CoplanarSideFactsMismatch);
                }
                self.expect_no_construction()
            }
            SegmentPlaneRelation::EndpointOnPlane => {
                let Some(endpoint) = self.endpoint_on_plane else {
                    return Err(SegmentPlaneValidationError::InvalidEndpointIndex);
                };
                if endpoint > 1 {
                    return Err(SegmentPlaneValidationError::InvalidEndpointIndex);
                }
                if self.point.is_none() || self.parameter.is_none() {
                    return Err(SegmentPlaneValidationError::MissingEndpointConstruction);
                }
                if self.parameter_ratio.is_some() {
                    return Err(SegmentPlaneValidationError::UnexpectedConstruction);
                }
                if self.construction_failure.is_some() {
                    return Err(SegmentPlaneValidationError::UnexpectedConstructionFailureReason);
                }
                if self.endpoint_sides[endpoint] != Some(PlaneSide::On)
                    || self.endpoint_sides[1 - endpoint].is_none()
                    || self.endpoint_sides[1 - endpoint] == Some(PlaneSide::On)
                {
                    return Err(SegmentPlaneValidationError::EndpointSideFactsMismatch);
                }
                let expected = Real::from(endpoint as i64);
                if !self
                    .parameter
                    .as_ref()
                    .is_some_and(|parameter| real_eq(parameter, &expected, policy))
                {
                    return Err(SegmentPlaneValidationError::MissingEndpointConstruction);
                }
                Ok(())
            }
            SegmentPlaneRelation::ProperCrossing => {
                if !opposite_strict_sides(self.endpoint_sides) {
                    return Err(SegmentPlaneValidationError::ProperCrossingSideFactsMismatch);
                }
                if self.endpoint_on_plane.is_some()
                    || self.point.is_none()
                    || self.parameter.is_none()
                {
                    return Err(SegmentPlaneValidationError::MissingProperCrossingConstruction);
                }
                if self.construction_failure.is_some() {
                    return Err(SegmentPlaneValidationError::UnexpectedConstructionFailureReason);
                }
                let parameter = self.parameter.as_ref().expect("checked above");
                if !real_between_open_unit(parameter, policy) {
                    return Err(SegmentPlaneValidationError::ProperCrossingParameterOutOfRange);
                }
                let Some(ratio) = self.parameter_ratio.as_ref() else {
                    return Err(SegmentPlaneValidationError::MissingProperCrossingRatio);
                };
                let Some(ratio_parameter) =
                    divide_real_with_policy(&ratio.numerator, &ratio.denominator, policy)
                        .ok()
                        .and_then(crate::PredicateOutcome::value)
                else {
                    return Err(SegmentPlaneValidationError::ProperCrossingRatioMismatch);
                };
                if !real_eq(&ratio_parameter, parameter, policy) {
                    return Err(SegmentPlaneValidationError::ProperCrossingRatioMismatch);
                }
                Ok(())
            }
            SegmentPlaneRelation::ConstructionFailed => {
                if !opposite_strict_sides(self.endpoint_sides) {
                    return Err(SegmentPlaneValidationError::ConstructionFailedSideFactsMismatch);
                }
                if self.construction_failure.is_none() {
                    return Err(SegmentPlaneValidationError::MissingConstructionFailureReason);
                }
                self.expect_no_success_construction()
            }
        }
    }

    /// Validate this event against the segment and oriented point-defined
    /// plane that produced it.
    pub fn validate_against_sources(
        &self,
        a: &Point3,
        b: &Point3,
        c: &Point3,
        p0: &Point3,
        p1: &Point3,
        policy: PredicatePolicy,
    ) -> Result<(), SegmentPlaneValidationError> {
        self.validate(policy)?;
        let replay = intersect_segment_with_oriented_plane_with_policy(a, b, c, p0, p1, policy);
        if self == &replay {
            Ok(())
        } else {
            Err(SegmentPlaneValidationError::SourceReplayMismatch)
        }
    }

    fn expect_no_construction(&self) -> Result<(), SegmentPlaneValidationError> {
        if self.point.is_none()
            && self.parameter.is_none()
            && self.parameter_ratio.is_none()
            && self.endpoint_on_plane.is_none()
        {
            if self.construction_failure.is_some() {
                Err(SegmentPlaneValidationError::UnexpectedConstructionFailureReason)
            } else {
                Ok(())
            }
        } else {
            Err(SegmentPlaneValidationError::UnexpectedConstruction)
        }
    }

    fn expect_no_success_construction(&self) -> Result<(), SegmentPlaneValidationError> {
        if self.point.is_none()
            && self.parameter.is_none()
            && self.parameter_ratio.is_none()
            && self.endpoint_on_plane.is_none()
        {
            Ok(())
        } else {
            Err(SegmentPlaneValidationError::UnexpectedConstruction)
        }
    }
}

/// Intersect a closed segment with an oriented point-defined plane.
pub fn intersect_segment_with_oriented_plane_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    p0: &Point3,
    p1: &Point3,
    policy: PredicatePolicy,
) -> SegmentPlaneIntersection {
    let outcomes = [
        orient3d_with_policy(a, b, c, p0, policy),
        orient3d_with_policy(a, b, c, p1, policy),
    ];
    let sides = [
        outcomes[0].value().map(PlaneSide::from),
        outcomes[1].value().map(PlaneSide::from),
    ];

    let evidence = oriented_plane3_evidence(a, b, c);
    let d0 = point_plane_value(evidence.plane(), p0);
    let d1 = point_plane_value(evidence.plane(), p1);
    intersect_segment_with_plane_values_with_policy(&d0, &d1, p0, p1, sides, policy)
}

/// Intersect a closed segment with an explicit oriented plane.
pub fn intersect_segment_with_plane_with_policy(
    plane: &Plane3,
    p0: &Point3,
    p1: &Point3,
    policy: PredicatePolicy,
) -> SegmentPlaneIntersection {
    let d0 = point_plane_value(plane, p0);
    let d1 = point_plane_value(plane, p1);
    let sides = [
        plane_side_from_value(&d0, policy),
        plane_side_from_value(&d1, policy),
    ];
    intersect_segment_with_plane_values_with_policy(&d0, &d1, p0, p1, sides, policy)
}

/// Build a segment/plane event from already-computed exact endpoint plane
/// values and side facts.
pub fn intersect_segment_with_plane_values_with_policy(
    d0: &Real,
    d1: &Real,
    p0: &Point3,
    p1: &Point3,
    sides: [Option<PlaneSide>; 2],
    policy: PredicatePolicy,
) -> SegmentPlaneIntersection {
    let Some([side0, side1]) = transpose_sides(sides) else {
        return event(
            SegmentPlaneRelation::Unknown,
            sides,
            SegmentPlaneEventConstruction::none(),
        );
    };

    match (side0, side1) {
        (PlaneSide::On, PlaneSide::On) => event(
            SegmentPlaneRelation::Coplanar,
            sides,
            SegmentPlaneEventConstruction::none(),
        ),
        (PlaneSide::On, _) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            sides,
            SegmentPlaneEventConstruction::endpoint(0, p0.clone(), Real::from(0)),
        ),
        (_, PlaneSide::On) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            sides,
            SegmentPlaneEventConstruction::endpoint(1, p1.clone(), Real::from(1)),
        ),
        (PlaneSide::Above, PlaneSide::Above) | (PlaneSide::Below, PlaneSide::Below) => event(
            SegmentPlaneRelation::Disjoint,
            sides,
            SegmentPlaneEventConstruction::none(),
        ),
        (PlaneSide::Above, PlaneSide::Below) | (PlaneSide::Below, PlaneSide::Above) => {
            match construct_segment_plane_crossing_from_values_with_policy(d0, d1, p0, p1, policy) {
                Ok((parameter, ratio, point)) => event(
                    SegmentPlaneRelation::ProperCrossing,
                    sides,
                    SegmentPlaneEventConstruction::proper_crossing(point, parameter, ratio),
                ),
                Err(failure) => event(
                    SegmentPlaneRelation::ConstructionFailed,
                    sides,
                    SegmentPlaneEventConstruction::failed(failure),
                ),
            }
        }
    }
}

/// Construct the exact proper crossing point from endpoint plane values.
pub fn construct_segment_plane_crossing_from_values_with_policy(
    d0: &Real,
    d1: &Real,
    p0: &Point3,
    p1: &Point3,
    policy: PredicatePolicy,
) -> Result<(Real, SegmentPlaneParameterRatio, Point3), SegmentPlaneConstructionFailure> {
    let denominator = d0.clone() - d1;
    let t = match divide_real_with_policy(d0, &denominator, policy) {
        Ok(outcome) => outcome
            .value()
            .ok_or(SegmentPlaneConstructionFailure::ParameterDivisionFailed)?,
        Err(Problem::DivideByZero) => {
            return Err(SegmentPlaneConstructionFailure::ZeroDenominator);
        }
        Err(_) => return Err(SegmentPlaneConstructionFailure::ParameterDivisionFailed),
    };
    let point = interpolate_point3(p0, p1, &t);
    let ratio = SegmentPlaneParameterRatio {
        numerator: d0.clone(),
        denominator,
    };
    Ok((t, ratio, point))
}

/// Return the exact affine point `start + t * (end - start)`.
pub fn interpolate_point3(start: &Point3, end: &Point3, t: &Real) -> Point3 {
    Point3::new(
        start.x.clone() + t.clone() * (end.x.clone() - &start.x),
        start.y.clone() + t.clone() * (end.y.clone() - &start.y),
        start.z.clone() + t.clone() * (end.z.clone() - &start.z),
    )
}

/// Evaluate an explicit plane expression at a point.
pub fn point_plane_value(plane: &Plane3, point: &Point3) -> Real {
    plane.normal.x.clone() * point.x.clone()
        + plane.normal.y.clone() * point.y.clone()
        + plane.normal.z.clone() * point.z.clone()
        + &plane.offset
}

/// Return a segment parameter from one nonconstant coordinate axis.
pub fn segment_parameter_from_axis_with_policy(
    point: &Real,
    start: &Real,
    end: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    let denominator = end.clone() - start;
    divide_real_with_policy(&(point.clone() - start), &denominator, policy)
        .ok()?
        .value()
}

fn plane_side_from_value(value: &Real, policy: PredicatePolicy) -> Option<PlaneSide> {
    match compare_reals_with_policy(value, &Real::from(0), policy).value()? {
        core::cmp::Ordering::Less => Some(PlaneSide::Below),
        core::cmp::Ordering::Equal => Some(PlaneSide::On),
        core::cmp::Ordering::Greater => Some(PlaneSide::Above),
    }
}

struct SegmentPlaneEventConstruction {
    point: Option<Point3>,
    parameter: Option<Real>,
    parameter_ratio: Option<SegmentPlaneParameterRatio>,
    endpoint_on_plane: Option<usize>,
    construction_failure: Option<SegmentPlaneConstructionFailure>,
}

impl SegmentPlaneEventConstruction {
    fn none() -> Self {
        Self {
            point: None,
            parameter: None,
            parameter_ratio: None,
            endpoint_on_plane: None,
            construction_failure: None,
        }
    }

    fn endpoint(endpoint: usize, point: Point3, parameter: Real) -> Self {
        Self {
            point: Some(point),
            parameter: Some(parameter),
            parameter_ratio: None,
            endpoint_on_plane: Some(endpoint),
            construction_failure: None,
        }
    }

    fn proper_crossing(
        point: Point3,
        parameter: Real,
        parameter_ratio: SegmentPlaneParameterRatio,
    ) -> Self {
        Self {
            point: Some(point),
            parameter: Some(parameter),
            parameter_ratio: Some(parameter_ratio),
            endpoint_on_plane: None,
            construction_failure: None,
        }
    }

    fn failed(failure: SegmentPlaneConstructionFailure) -> Self {
        Self {
            point: None,
            parameter: None,
            parameter_ratio: None,
            endpoint_on_plane: None,
            construction_failure: Some(failure),
        }
    }
}

fn event(
    relation: SegmentPlaneRelation,
    endpoint_sides: [Option<PlaneSide>; 2],
    construction: SegmentPlaneEventConstruction,
) -> SegmentPlaneIntersection {
    SegmentPlaneIntersection {
        relation,
        point: construction.point,
        parameter: construction.parameter,
        parameter_ratio: construction.parameter_ratio,
        endpoint_on_plane: construction.endpoint_on_plane,
        endpoint_sides,
        construction_failure: construction.construction_failure,
    }
}

fn transpose_sides(sides: [Option<PlaneSide>; 2]) -> Option<[PlaneSide; 2]> {
    Some([sides[0]?, sides[1]?])
}

fn opposite_strict_sides(sides: [Option<PlaneSide>; 2]) -> bool {
    matches!(
        sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
            | [Some(PlaneSide::Below), Some(PlaneSide::Above)]
    )
}

fn real_eq(left: &Real, right: &Real, policy: PredicatePolicy) -> bool {
    matches!(
        compare_reals_with_policy(left, right, policy).value(),
        Some(core::cmp::Ordering::Equal)
    )
}

fn real_between_open_unit(value: &Real, policy: PredicatePolicy) -> bool {
    let zero = Real::from(0);
    let one = Real::from(1);
    matches!(
        compare_reals_with_policy(value, &zero, policy).value(),
        Some(core::cmp::Ordering::Greater)
    ) && matches!(
        compare_reals_with_policy(value, &one, policy).value(),
        Some(core::cmp::Ordering::Less)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Real;

    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    fn p3(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn segment_plane_constructs_proper_crossing_as_ratio() {
        let event = crate::intersect_segment_with_oriented_plane(
            &p3(0, 0, 0),
            &p3(1, 0, 0),
            &p3(0, 1, 0),
            &p3(0, 0, -1),
            &p3(0, 0, 1),
            APPROX,
        );
        assert_eq!(event.relation, SegmentPlaneRelation::ProperCrossing);
        let half = (Real::from(1) / &Real::from(2)).unwrap();
        assert_eq!(event.parameter, Some(half));
        assert!(event.parameter_ratio.is_some());
        event
            .validate_against_sources(
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                &p3(0, 1, 0),
                &p3(0, 0, -1),
                &p3(0, 0, 1),
                APPROX,
            )
            .unwrap();
    }

    #[test]
    fn segment_plane_classifies_endpoint_coplanar_and_disjoint_cases() {
        let a = p3(0, 0, 0);
        let b = p3(1, 0, 0);
        let c = p3(0, 1, 0);
        assert_eq!(
            crate::intersect_segment_with_oriented_plane(
                &a,
                &b,
                &c,
                &p3(0, 0, 0),
                &p3(0, 0, 2),
                APPROX
            )
            .relation,
            SegmentPlaneRelation::EndpointOnPlane
        );
        assert_eq!(
            crate::intersect_segment_with_oriented_plane(
                &a,
                &b,
                &c,
                &p3(0, 0, 1),
                &p3(1, 0, 1),
                APPROX
            )
            .relation,
            SegmentPlaneRelation::Disjoint
        );
        assert_eq!(
            crate::intersect_segment_with_oriented_plane(
                &a,
                &b,
                &c,
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                APPROX
            )
            .relation,
            SegmentPlaneRelation::Coplanar
        );
    }

    #[test]
    fn explicit_plane_intersection_covers_every_decided_relation() {
        let plane = Plane3::new(p3(0, 0, 1), Real::from(0));
        let cases = [
            (
                p3(0, 0, -1),
                p3(0, 0, 1),
                SegmentPlaneRelation::ProperCrossing,
                [Some(PlaneSide::Below), Some(PlaneSide::Above)],
            ),
            (
                p3(0, 0, 0),
                p3(0, 0, 1),
                SegmentPlaneRelation::EndpointOnPlane,
                [Some(PlaneSide::On), Some(PlaneSide::Above)],
            ),
            (
                p3(0, 0, -1),
                p3(0, 0, 0),
                SegmentPlaneRelation::EndpointOnPlane,
                [Some(PlaneSide::Below), Some(PlaneSide::On)],
            ),
            (
                p3(0, 0, 0),
                p3(1, 1, 0),
                SegmentPlaneRelation::Coplanar,
                [Some(PlaneSide::On), Some(PlaneSide::On)],
            ),
            (
                p3(0, 0, 1),
                p3(1, 1, 2),
                SegmentPlaneRelation::Disjoint,
                [Some(PlaneSide::Above), Some(PlaneSide::Above)],
            ),
            (
                p3(0, 0, -1),
                p3(1, 1, -2),
                SegmentPlaneRelation::Disjoint,
                [Some(PlaneSide::Below), Some(PlaneSide::Below)],
            ),
        ];

        for (p0, p1, relation, sides) in cases {
            let intersection = crate::intersect_segment_with_plane(&plane, &p0, &p1, APPROX);
            assert_eq!(intersection.relation, relation);
            assert_eq!(intersection.endpoint_sides, sides);
            intersection.validate(APPROX).unwrap();
        }
    }

    #[test]
    fn retained_segment_plane_validation_rejects_every_inconsistent_field_family() {
        let p0 = p3(0, 0, -1);
        let p1 = p3(0, 0, 1);
        let plane_a = p3(0, 0, 0);
        let plane_b = p3(1, 0, 0);
        let plane_c = p3(0, 1, 0);

        let unknown = event(
            SegmentPlaneRelation::Unknown,
            [None, Some(PlaneSide::Above)],
            SegmentPlaneEventConstruction::none(),
        );
        unknown.validate(APPROX).unwrap();

        let mut forged = unknown.clone();
        forged.endpoint_sides = [Some(PlaneSide::Below), Some(PlaneSide::Above)];
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::UnknownHasDecidedSides)
        );

        let mut forged = unknown.clone();
        forged.point = Some(p0.clone());
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::UnexpectedConstruction)
        );

        let mut forged = unknown.clone();
        forged.construction_failure = Some(SegmentPlaneConstructionFailure::ZeroDenominator);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::UnexpectedConstructionFailureReason)
        );

        let disjoint = event(
            SegmentPlaneRelation::Disjoint,
            [Some(PlaneSide::Above), Some(PlaneSide::Above)],
            SegmentPlaneEventConstruction::none(),
        );
        disjoint.validate(APPROX).unwrap();
        let mut forged = disjoint;
        forged.endpoint_sides[1] = Some(PlaneSide::Below);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::DisjointSideFactsMismatch)
        );

        let coplanar = event(
            SegmentPlaneRelation::Coplanar,
            [Some(PlaneSide::On), Some(PlaneSide::On)],
            SegmentPlaneEventConstruction::none(),
        );
        coplanar.validate(APPROX).unwrap();
        let mut forged = coplanar;
        forged.endpoint_sides[1] = Some(PlaneSide::Above);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::CoplanarSideFactsMismatch)
        );

        let endpoint = event(
            SegmentPlaneRelation::EndpointOnPlane,
            [Some(PlaneSide::On), Some(PlaneSide::Above)],
            SegmentPlaneEventConstruction::endpoint(0, p3(0, 0, 0), Real::from(0)),
        );
        endpoint.validate(APPROX).unwrap();

        let mut forged = endpoint.clone();
        forged.endpoint_on_plane = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::InvalidEndpointIndex)
        );
        let mut forged = endpoint.clone();
        forged.endpoint_on_plane = Some(2);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::InvalidEndpointIndex)
        );
        let mut forged = endpoint.clone();
        forged.point = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::MissingEndpointConstruction)
        );
        let mut forged = endpoint.clone();
        forged.parameter_ratio = Some(SegmentPlaneParameterRatio {
            numerator: Real::from(0),
            denominator: Real::from(1),
        });
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::UnexpectedConstruction)
        );
        let mut forged = endpoint.clone();
        forged.construction_failure = Some(SegmentPlaneConstructionFailure::ZeroDenominator);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::UnexpectedConstructionFailureReason)
        );
        let mut forged = endpoint.clone();
        forged.endpoint_sides = [Some(PlaneSide::Above), Some(PlaneSide::On)];
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::EndpointSideFactsMismatch)
        );
        let mut forged = endpoint;
        forged.parameter = Some(Real::from(1));
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::MissingEndpointConstruction)
        );

        let crossing = crate::intersect_segment_with_oriented_plane(
            &plane_a, &plane_b, &plane_c, &p0, &p1, APPROX,
        );
        crossing.validate(APPROX).unwrap();

        let mut forged = crossing.clone();
        forged.endpoint_sides = [Some(PlaneSide::Above), Some(PlaneSide::Above)];
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::ProperCrossingSideFactsMismatch)
        );
        let mut forged = crossing.clone();
        forged.point = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::MissingProperCrossingConstruction)
        );
        let mut forged = crossing.clone();
        forged.construction_failure = Some(SegmentPlaneConstructionFailure::ZeroDenominator);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::UnexpectedConstructionFailureReason)
        );
        let mut forged = crossing.clone();
        forged.parameter = Some(Real::from(0));
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::ProperCrossingParameterOutOfRange)
        );
        let mut forged = crossing.clone();
        forged.parameter = Some(Real::from(2));
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::ProperCrossingParameterOutOfRange)
        );
        let mut forged = crossing.clone();
        forged.parameter_ratio = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::MissingProperCrossingRatio)
        );
        let mut forged = crossing.clone();
        forged.parameter_ratio.as_mut().unwrap().denominator = Real::from(0);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::ProperCrossingRatioMismatch)
        );
        let root_two = Real::from(2).sqrt().unwrap();
        let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
        let half = (Real::from(1) / Real::from(2)).unwrap();
        let shared_offset = root_two.clone() * Real::from(3) + half;
        let contact = (((root_two.clone() * Real::from(4) - shared_offset.clone()) * Real::pi())
            * root_two_over_pi.clone()
            / Real::from(4))
        .unwrap();
        let domain = (((root_two * Real::from(2) - shared_offset) * Real::pi()) * root_two_over_pi
            / Real::from(4))
        .unwrap()
            + Real::from(1);
        let exact_normal_form_positive = contact - domain + Real::from(2).powi_i64(-5000).unwrap();
        assert!(matches!(
            exact_normal_form_positive.certified_sign_until(-4096),
            hyperreal::CertifiedRealSign::Unknown { .. }
        ));
        assert_eq!(
            compare_reals_with_policy(
                &exact_normal_form_positive,
                &Real::from(0),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(core::cmp::Ordering::Greater)
        );
        let mut forged = crossing.clone();
        forged.parameter_ratio.as_mut().unwrap().denominator = exact_normal_form_positive;
        assert_eq!(
            forged.validate(PredicatePolicy::STRICT),
            Err(SegmentPlaneValidationError::ProperCrossingRatioMismatch)
        );
        let mut forged = crossing.clone();
        forged.parameter_ratio.as_mut().unwrap().numerator = Real::from(3);
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::ProperCrossingRatioMismatch)
        );

        let mut forged = crossing;
        forged.point = Some(p3(9, 9, 0));
        forged.validate(APPROX).unwrap();
        assert_eq!(
            forged.validate_against_sources(&plane_a, &plane_b, &plane_c, &p0, &p1, APPROX,),
            Err(SegmentPlaneValidationError::SourceReplayMismatch)
        );
    }

    #[test]
    fn construction_failure_records_are_validated_and_preserved() {
        let p0 = p3(0, 0, -1);
        let p1 = p3(0, 0, 1);
        let unknown = intersect_segment_with_plane_values_with_policy(
            &Real::from(-1),
            &Real::from(1),
            &p0,
            &p1,
            [None, Some(PlaneSide::Above)],
            APPROX,
        );
        assert_eq!(unknown.relation, SegmentPlaneRelation::Unknown);
        unknown.validate(APPROX).unwrap();

        let failed = intersect_segment_with_plane_values_with_policy(
            &Real::from(0),
            &Real::from(0),
            &p0,
            &p1,
            [Some(PlaneSide::Below), Some(PlaneSide::Above)],
            PredicatePolicy::STRICT,
        );
        assert_eq!(failed.relation, SegmentPlaneRelation::ConstructionFailed);
        assert_eq!(
            failed.construction_failure,
            Some(SegmentPlaneConstructionFailure::ZeroDenominator)
        );
        failed.validate(APPROX).unwrap();

        let mut forged = failed.clone();
        forged.endpoint_sides = [Some(PlaneSide::Above), Some(PlaneSide::Above)];
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::ConstructionFailedSideFactsMismatch)
        );
        let mut forged = failed.clone();
        forged.construction_failure = None;
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::MissingConstructionFailureReason)
        );
        let mut forged = failed;
        forged.point = Some(p3(0, 0, 0));
        assert_eq!(
            forged.validate(APPROX),
            Err(SegmentPlaneValidationError::UnexpectedConstruction)
        );

        let tiny = "1e-2000".parse::<Real>().unwrap();
        let left = (Real::pi() + tiny.clone() + tiny.clone()).exp().unwrap();
        let right = (Real::pi() + tiny).exp().unwrap();
        let opaque_positive = left - right;
        assert_eq!(
            opaque_positive.zero_status(),
            hyperreal::ZeroKnowledge::Unknown
        );
        assert!(matches!(
            opaque_positive.certified_sign_until(-4096),
            hyperreal::CertifiedRealSign::Unknown { .. }
        ));
        let failed = intersect_segment_with_plane_values_with_policy(
            &opaque_positive,
            &Real::from(0),
            &p0,
            &p1,
            [Some(PlaneSide::Below), Some(PlaneSide::Above)],
            PredicatePolicy::STRICT,
        );
        assert_eq!(failed.relation, SegmentPlaneRelation::ConstructionFailed);
        assert_eq!(
            failed.construction_failure,
            Some(SegmentPlaneConstructionFailure::ParameterDivisionFailed)
        );
    }

    #[test]
    fn segment_parameter_rejects_constant_axes_and_solves_nonconstant_axes() {
        assert_eq!(
            crate::segment_parameter_from_axis(
                &Real::from(2),
                &Real::from(1),
                &Real::from(1),
                APPROX,
            ),
            None
        );
        assert_eq!(
            crate::segment_parameter_from_axis(
                &Real::from(2),
                &Real::from(1),
                &Real::from(3),
                APPROX,
            ),
            Some((Real::from(1) / &Real::from(2)).unwrap())
        );

        let half = (Real::from(1) / Real::from(2)).unwrap();
        let denominator = crate::test_support::exact_normal_positive();
        assert_eq!(denominator.zero_status(), hyperreal::ZeroKnowledge::Unknown);
        assert_eq!(
            denominator.inverse_ref(),
            Err(hyperreal::Problem::UnknownZero)
        );
        let midpoint = &denominator * &half;
        let parameter = crate::segment_parameter_from_axis(
            &midpoint,
            &Real::zero(),
            &denominator,
            PredicatePolicy::STRICT,
        )
        .expect("policy-certified denominator should construct a parameter");
        assert_eq!(
            parameter.exact_rational_normal_form(),
            half.exact_rational()
        );

        let tiny = "1e-2000".parse::<Real>().unwrap();
        let left = (Real::pi() + tiny.clone() + tiny.clone()).exp().unwrap();
        let right = (Real::pi() + tiny).exp().unwrap();
        let unsupported = left - right;
        assert_eq!(
            crate::segment_parameter_from_axis(
                &(&unsupported * &half),
                &Real::zero(),
                &unsupported,
                PredicatePolicy::STRICT,
            ),
            None
        );
    }
}
