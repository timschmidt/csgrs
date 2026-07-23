//! Exact segment/plane construction helpers.
//!
//! A segment/plane crossing is the construction counterpart to plane-side
//! predicates. Endpoint sides decide the topology, and only a certified proper
//! crossing constructs `p0 + t (p1 - p0)` with `t = d0 / (d0 - d1)`, where
//! `d0` and `d1` are exact oriented plane evaluations. Predicates decide the
//! combinatorics, and constructions preserve the arithmetic structure needed by
//! later predicates.

use crate::PreparedOrientedPlane3;
use crate::classify::PlaneSide;
use crate::geometry::{Plane3, Point3};
use crate::predicates::order::compare_reals;
use crate::predicates::orient::orient3d;
use hyperreal::Real;

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
    pub fn validate(&self) -> Result<(), SegmentPlaneValidationError> {
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
                    .is_some_and(|parameter| real_eq(parameter, &expected))
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
                if !real_between_open_unit(parameter) {
                    return Err(SegmentPlaneValidationError::ProperCrossingParameterOutOfRange);
                }
                let Some(ratio) = self.parameter_ratio.as_ref() else {
                    return Err(SegmentPlaneValidationError::MissingProperCrossingRatio);
                };
                if matches!(
                    compare_reals(&ratio.denominator, &Real::from(0)).value(),
                    Some(core::cmp::Ordering::Equal) | None
                ) {
                    return Err(SegmentPlaneValidationError::ProperCrossingRatioMismatch);
                }
                let Some(ratio_parameter) = (&ratio.numerator / &ratio.denominator).ok() else {
                    return Err(SegmentPlaneValidationError::ProperCrossingRatioMismatch);
                };
                if !real_eq(&ratio_parameter, parameter) {
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
    ) -> Result<(), SegmentPlaneValidationError> {
        self.validate()?;
        let replay = intersect_segment_with_oriented_plane(a, b, c, p0, p1);
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
pub fn intersect_segment_with_oriented_plane(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    p0: &Point3,
    p1: &Point3,
) -> SegmentPlaneIntersection {
    let outcomes = [orient3d(a, b, c, p0), orient3d(a, b, c, p1)];
    let sides = [
        outcomes[0].value().map(PlaneSide::from),
        outcomes[1].value().map(PlaneSide::from),
    ];

    let prepared = PreparedOrientedPlane3::new(a, b, c);
    let d0 = point_plane_value(prepared.plane(), p0);
    let d1 = point_plane_value(prepared.plane(), p1);
    intersect_segment_with_plane_values(&d0, &d1, p0, p1, sides)
}

/// Intersect a closed segment with an explicit oriented plane.
pub fn intersect_segment_with_plane(
    plane: &Plane3,
    p0: &Point3,
    p1: &Point3,
) -> SegmentPlaneIntersection {
    let d0 = point_plane_value(plane, p0);
    let d1 = point_plane_value(plane, p1);
    let sides = [plane_side_from_value(&d0), plane_side_from_value(&d1)];
    intersect_segment_with_plane_values(&d0, &d1, p0, p1, sides)
}

/// Build a segment/plane event from already-computed exact endpoint plane
/// values and side facts.
pub fn intersect_segment_with_plane_values(
    d0: &Real,
    d1: &Real,
    p0: &Point3,
    p1: &Point3,
    sides: [Option<PlaneSide>; 2],
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
            match construct_segment_plane_crossing_from_values(d0, d1, p0, p1) {
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
pub fn construct_segment_plane_crossing_from_values(
    d0: &Real,
    d1: &Real,
    p0: &Point3,
    p1: &Point3,
) -> Result<(Real, SegmentPlaneParameterRatio, Point3), SegmentPlaneConstructionFailure> {
    let denominator = d0.clone() - d1;
    if matches!(
        compare_reals(&denominator, &Real::from(0)).value(),
        Some(core::cmp::Ordering::Equal)
    ) {
        return Err(SegmentPlaneConstructionFailure::ZeroDenominator);
    }
    let t = (d0 / &denominator)
        .map_err(|_| SegmentPlaneConstructionFailure::ParameterDivisionFailed)?;
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
pub fn segment_parameter_from_axis(point: &Real, start: &Real, end: &Real) -> Option<Real> {
    let denominator = end.clone() - start;
    if matches!(
        compare_reals(&denominator, &Real::from(0)).value(),
        Some(core::cmp::Ordering::Equal) | None
    ) {
        return None;
    }
    ((point.clone() - start) / &denominator).ok()
}

fn plane_side_from_value(value: &Real) -> Option<PlaneSide> {
    match compare_reals(value, &Real::from(0)).value()? {
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

fn real_eq(left: &Real, right: &Real) -> bool {
    matches!(
        compare_reals(left, right).value(),
        Some(core::cmp::Ordering::Equal)
    )
}

fn real_between_open_unit(value: &Real) -> bool {
    let zero = Real::from(0);
    let one = Real::from(1);
    matches!(
        compare_reals(value, &zero).value(),
        Some(core::cmp::Ordering::Greater)
    ) && matches!(
        compare_reals(value, &one).value(),
        Some(core::cmp::Ordering::Less)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Real;

    fn p3(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn segment_plane_constructs_proper_crossing_as_ratio() {
        let event = intersect_segment_with_oriented_plane(
            &p3(0, 0, 0),
            &p3(1, 0, 0),
            &p3(0, 1, 0),
            &p3(0, 0, -1),
            &p3(0, 0, 1),
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
            )
            .unwrap();
    }

    #[test]
    fn segment_plane_classifies_endpoint_coplanar_and_disjoint_cases() {
        let a = p3(0, 0, 0);
        let b = p3(1, 0, 0);
        let c = p3(0, 1, 0);
        assert_eq!(
            intersect_segment_with_oriented_plane(&a, &b, &c, &p3(0, 0, 0), &p3(0, 0, 2)).relation,
            SegmentPlaneRelation::EndpointOnPlane
        );
        assert_eq!(
            intersect_segment_with_oriented_plane(&a, &b, &c, &p3(0, 0, 1), &p3(1, 0, 1)).relation,
            SegmentPlaneRelation::Disjoint
        );
        assert_eq!(
            intersect_segment_with_oriented_plane(&a, &b, &c, &p3(0, 0, 0), &p3(1, 0, 0)).relation,
            SegmentPlaneRelation::Coplanar
        );
    }
}
