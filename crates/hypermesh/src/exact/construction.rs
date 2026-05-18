//! Exact construction helpers for mesh intersection events.
//!
//! This module starts the boolmesh-to-hypermesh construction port at the event
//! that every triangle splitter needs: a segment crossing an oriented face
//! plane.  The classification is still a predicate question, so endpoint sides
//! are certified with `hyperlimit::orient3d_report`.  When a proper crossing is
//! proven, the constructed point is a retained exact `Real` determinant ratio,
//! not an interpolated primitive float.  This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): predicates
//! decide combinatorics, and constructions retain the arithmetic structure
//! needed by later predicates.
//!
//! Segment/plane crossings are the construction counterpart to the
//! triangle/triangle tests of Moller, "A Fast Triangle-Triangle Intersection
//! Test," *Journal of Graphics Tools* 2.2 (1997), and Guigue and Devillers,
//! "Fast and Robust Triangle-Triangle Overlap Test Using Orientation
//! Predicates," *Journal of Graphics Tools* 8.1 (2003).  Hypermesh keeps the
//! event exact so those algorithms can be ported without epsilon tests.

use hyperlimit::{
    Plane3, PlaneSide, Point3, PreparedOrientedPlane3, compare_reals, orient3d_report,
};

use super::facts::FacePlaneFacts;
use super::provenance::PredicateUse;
use super::scalar::ExactReal;

/// Exact segment relation to an oriented face plane.
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
///
/// A failure reason is retained only after endpoint predicates have certified
/// opposite strict sides. Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), treats this as an explicit
/// refinement/policy artifact: topology code may reject, retry at higher
/// precision, or route to a richer construction kernel, but it must not infer
/// a split point from missing coordinates.
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
/// than recomputing the geometry. Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), separates certified predicate facts
/// from later topology mutation; a segment/plane event whose relation,
/// endpoint-side facts, exact point, and parameter disagree is not a safe
/// construction artifact for the boolean graph.
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
}

/// Certified segment/plane event with retained construction data.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentPlaneIntersection {
    /// Coarse relation between the closed segment and oriented plane.
    pub relation: SegmentPlaneRelation,
    /// Exact intersection point for endpoint and proper-crossing events.
    pub point: Option<Point3>,
    /// Exact segment parameter `t` where `p(t) = p0 + t * (p1 - p0)`.
    pub parameter: Option<ExactReal>,
    /// Determinant ratio that produced [`Self::parameter`] for proper
    /// crossings.
    pub parameter_ratio: Option<SegmentPlaneParameterRatio>,
    /// Endpoint index, `0` or `1`, when [`SegmentPlaneRelation::EndpointOnPlane`].
    pub endpoint_on_plane: Option<usize>,
    /// Certified side for each segment endpoint, or `None` when undecided.
    pub endpoint_sides: [Option<PlaneSide>; 2],
    /// Predicate certificates used to classify the two endpoints.
    pub predicates: Vec<PredicateUse>,
    /// Structured construction failure retained when a certified crossing
    /// could not produce a split point.
    pub construction_failure: Option<SegmentPlaneConstructionFailure>,
}

/// Determinant numerator and denominator for a segment/plane crossing.
///
/// For a proper crossing, `parameter = numerator / denominator`, where
/// `numerator` is the oriented plane evaluation at the first endpoint and
/// `denominator = d0 - d1`. Retaining this ratio preserves the construction
/// shape described by Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): downstream edge ordering can audit
/// the determinant form instead of receiving only an expanded scalar.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentPlaneParameterRatio {
    /// Oriented plane value at the first segment endpoint.
    pub numerator: ExactReal,
    /// Difference between first and second endpoint plane values.
    pub denominator: ExactReal,
}

impl SegmentPlaneIntersection {
    /// Return whether every predicate route produced an exact-preserving proof.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Validate relation, endpoint-side, and construction-field consistency.
    ///
    /// Proper crossings and endpoint events must retain the exact construction
    /// data that downstream split ordering consumes. Disjoint, coplanar,
    /// unknown, and construction-failed states must keep unsupported
    /// construction data absent so callers cannot accidentally treat them as
    /// split points.
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
                let expected = ExactReal::from(endpoint as i64);
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
                    compare_reals(&ratio.denominator, &ExactReal::from(0)).value(),
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

/// Intersect a mesh segment with the oriented plane of one triangular face.
///
/// The face orientation is the vertex order in `face`, matching
/// `hyperlimit::orient3d_report(a, b, c, point)`.  A proper crossing constructs
/// `t = d0 / (d0 - d1)`, where `d0` and `d1` are exact evaluations of the same
/// oriented plane at the segment endpoints.  This determinant-ratio form keeps
/// the construction exact and auditable for later edge ordering.
pub fn intersect_segment_with_face_plane(
    points: &[Point3],
    face: [usize; 3],
    segment: [usize; 2],
) -> SegmentPlaneIntersection {
    intersect_segment_with_oriented_plane(
        &points[face[0]],
        &points[face[1]],
        &points[face[2]],
        &points[segment[0]],
        &points[segment[1]],
    )
}

/// Intersect a closed segment with a retained exact face plane.
///
/// This is the cached construction path for validated mesh faces. It consumes
/// the determinant-form coefficients retained in [`FacePlaneFacts`] and builds
/// the same segment event as [`intersect_segment_with_oriented_plane`] without
/// reconstructing a point-defined plane. Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), treats this retained
/// numerical structure as part of the object model: exact constructions should
/// reuse certified object facts rather than reintroducing representative
/// primitive normals.
pub fn intersect_segment_with_retained_face_plane(
    plane: &FacePlaneFacts,
    p0: &Point3,
    p1: &Point3,
) -> SegmentPlaneIntersection {
    let d0 = retained_point_plane_value(plane, p0);
    let d1 = retained_point_plane_value(plane, p1);
    let sides = [retained_plane_side(&d0), retained_plane_side(&d1)];
    let predicates = Vec::new();

    let Some([side0, side1]) = transpose_sides(sides) else {
        return event(
            SegmentPlaneRelation::Unknown,
            None,
            None,
            None,
            None,
            sides,
            predicates,
            None,
        );
    };

    match (side0, side1) {
        (PlaneSide::On, PlaneSide::On) => event(
            SegmentPlaneRelation::Coplanar,
            None,
            None,
            None,
            None,
            sides,
            predicates,
            None,
        ),
        (PlaneSide::On, _) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p0.clone()),
            Some(ExactReal::from(0)),
            None,
            Some(0),
            sides,
            predicates,
            None,
        ),
        (_, PlaneSide::On) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p1.clone()),
            Some(ExactReal::from(1)),
            None,
            Some(1),
            sides,
            predicates,
            None,
        ),
        (PlaneSide::Above, PlaneSide::Above) | (PlaneSide::Below, PlaneSide::Below) => event(
            SegmentPlaneRelation::Disjoint,
            None,
            None,
            None,
            None,
            sides,
            predicates,
            None,
        ),
        (PlaneSide::Above, PlaneSide::Below) | (PlaneSide::Below, PlaneSide::Above) => {
            match construct_crossing_from_values(&d0, &d1, p0, p1) {
                Ok((parameter, ratio, point)) => event(
                    SegmentPlaneRelation::ProperCrossing,
                    Some(point),
                    Some(parameter),
                    Some(ratio),
                    None,
                    sides,
                    predicates,
                    None,
                ),
                Err(failure) => event(
                    SegmentPlaneRelation::ConstructionFailed,
                    None,
                    None,
                    None,
                    None,
                    sides,
                    predicates,
                    Some(failure),
                ),
            }
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
    let reports = [orient3d_report(a, b, c, p0), orient3d_report(a, b, c, p1)];
    let predicates = reports
        .iter()
        .map(|report| PredicateUse::from_certificate(report.certificate))
        .collect::<Vec<_>>();
    let sides = [
        reports[0].value().map(PlaneSide::from),
        reports[1].value().map(PlaneSide::from),
    ];

    let Some([side0, side1]) = transpose_sides(sides) else {
        return event(
            SegmentPlaneRelation::Unknown,
            None,
            None,
            None,
            None,
            sides,
            predicates,
            None,
        );
    };

    match (side0, side1) {
        (PlaneSide::On, PlaneSide::On) => event(
            SegmentPlaneRelation::Coplanar,
            None,
            None,
            None,
            None,
            sides,
            predicates,
            None,
        ),
        (PlaneSide::On, _) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p0.clone()),
            Some(ExactReal::from(0)),
            None,
            Some(0),
            sides,
            predicates,
            None,
        ),
        (_, PlaneSide::On) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p1.clone()),
            Some(ExactReal::from(1)),
            None,
            Some(1),
            sides,
            predicates,
            None,
        ),
        (PlaneSide::Above, PlaneSide::Above) | (PlaneSide::Below, PlaneSide::Below) => event(
            SegmentPlaneRelation::Disjoint,
            None,
            None,
            None,
            None,
            sides,
            predicates,
            None,
        ),
        (PlaneSide::Above, PlaneSide::Below) | (PlaneSide::Below, PlaneSide::Above) => {
            let prepared = PreparedOrientedPlane3::new(a, b, c);
            match construct_crossing(prepared.plane(), p0, p1) {
                Ok((parameter, ratio, point)) => event(
                    SegmentPlaneRelation::ProperCrossing,
                    Some(point),
                    Some(parameter),
                    Some(ratio),
                    None,
                    sides,
                    predicates,
                    None,
                ),
                Err(failure) => event(
                    SegmentPlaneRelation::ConstructionFailed,
                    None,
                    None,
                    None,
                    None,
                    sides,
                    predicates,
                    Some(failure),
                ),
            }
        }
    }
}

fn construct_crossing(
    plane: &Plane3,
    p0: &Point3,
    p1: &Point3,
) -> Result<(ExactReal, SegmentPlaneParameterRatio, Point3), SegmentPlaneConstructionFailure> {
    let d0 = point_plane_value(plane, p0);
    let d1 = point_plane_value(plane, p1);
    construct_crossing_from_values(&d0, &d1, p0, p1)
}

fn construct_crossing_from_values(
    d0: &ExactReal,
    d1: &ExactReal,
    p0: &Point3,
    p1: &Point3,
) -> Result<(ExactReal, SegmentPlaneParameterRatio, Point3), SegmentPlaneConstructionFailure> {
    let denominator = sub(&d0, &d1);
    if matches!(
        compare_reals(&denominator, &ExactReal::from(0)).value(),
        Some(core::cmp::Ordering::Equal)
    ) {
        return Err(SegmentPlaneConstructionFailure::ZeroDenominator);
    }
    let t = (d0 / &denominator)
        .map_err(|_| SegmentPlaneConstructionFailure::ParameterDivisionFailed)?;
    let point = interpolate(p0, p1, &t);
    let ratio = SegmentPlaneParameterRatio {
        numerator: d0.clone(),
        denominator,
    };
    Ok((t, ratio, point))
}

fn interpolate(p0: &Point3, p1: &Point3, t: &ExactReal) -> Point3 {
    Point3::new(
        add(&p0.x, &mul(t, &sub(&p1.x, &p0.x))),
        add(&p0.y, &mul(t, &sub(&p1.y, &p0.y))),
        add(&p0.z, &mul(t, &sub(&p1.z, &p0.z))),
    )
}

fn point_plane_value(plane: &Plane3, point: &Point3) -> ExactReal {
    let x = mul(&plane.normal.x, &point.x);
    let y = mul(&plane.normal.y, &point.y);
    let z = mul(&plane.normal.z, &point.z);
    add(&add(&add(&x, &y), &z), &plane.offset)
}

fn retained_point_plane_value(plane: &FacePlaneFacts, point: &Point3) -> ExactReal {
    let x = mul(&plane.normal[0], &point.x);
    let y = mul(&plane.normal[1], &point.y);
    let z = mul(&plane.normal[2], &point.z);
    add(&add(&add(&x, &y), &z), &plane.offset)
}

fn retained_plane_side(value: &ExactReal) -> Option<PlaneSide> {
    // `hyperlimit::orient3d_report(a, b, c, p)` uses the opposite sign
    // convention from the stored `(b - a) x (c - a)` dot-product form.
    match compare_reals(value, &ExactReal::from(0)).value()? {
        core::cmp::Ordering::Less => Some(PlaneSide::Above),
        core::cmp::Ordering::Equal => Some(PlaneSide::On),
        core::cmp::Ordering::Greater => Some(PlaneSide::Below),
    }
}

fn event(
    relation: SegmentPlaneRelation,
    point: Option<Point3>,
    parameter: Option<ExactReal>,
    parameter_ratio: Option<SegmentPlaneParameterRatio>,
    endpoint_on_plane: Option<usize>,
    endpoint_sides: [Option<PlaneSide>; 2],
    predicates: Vec<PredicateUse>,
    construction_failure: Option<SegmentPlaneConstructionFailure>,
) -> SegmentPlaneIntersection {
    SegmentPlaneIntersection {
        relation,
        point,
        parameter,
        parameter_ratio,
        endpoint_on_plane,
        endpoint_sides,
        predicates,
        construction_failure,
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

fn real_eq(left: &ExactReal, right: &ExactReal) -> bool {
    matches!(
        compare_reals(left, right).value(),
        Some(core::cmp::Ordering::Equal)
    )
}

fn real_between_open_unit(value: &ExactReal) -> bool {
    let zero = ExactReal::from(0);
    let one = ExactReal::from(1);
    matches!(
        compare_reals(value, &zero).value(),
        Some(core::cmp::Ordering::Greater)
    ) && matches!(
        compare_reals(value, &one).value(),
        Some(core::cmp::Ordering::Less)
    )
}

fn add(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}
