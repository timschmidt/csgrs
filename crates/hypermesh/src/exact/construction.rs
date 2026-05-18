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

/// Certified segment/plane event with retained construction data.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentPlaneIntersection {
    /// Coarse relation between the closed segment and oriented plane.
    pub relation: SegmentPlaneRelation,
    /// Exact intersection point for endpoint and proper-crossing events.
    pub point: Option<Point3>,
    /// Exact segment parameter `t` where `p(t) = p0 + t * (p1 - p0)`.
    pub parameter: Option<ExactReal>,
    /// Endpoint index, `0` or `1`, when [`SegmentPlaneRelation::EndpointOnPlane`].
    pub endpoint_on_plane: Option<usize>,
    /// Certified side for each segment endpoint, or `None` when undecided.
    pub endpoint_sides: [Option<PlaneSide>; 2],
    /// Predicate certificates used to classify the two endpoints.
    pub predicates: Vec<PredicateUse>,
}

impl SegmentPlaneIntersection {
    /// Return whether every predicate route produced an exact-preserving proof.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
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
            sides,
            predicates,
        );
    };

    match (side0, side1) {
        (PlaneSide::On, PlaneSide::On) => event(
            SegmentPlaneRelation::Coplanar,
            None,
            None,
            None,
            sides,
            predicates,
        ),
        (PlaneSide::On, _) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p0.clone()),
            Some(ExactReal::from(0)),
            Some(0),
            sides,
            predicates,
        ),
        (_, PlaneSide::On) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p1.clone()),
            Some(ExactReal::from(1)),
            Some(1),
            sides,
            predicates,
        ),
        (PlaneSide::Above, PlaneSide::Above) | (PlaneSide::Below, PlaneSide::Below) => event(
            SegmentPlaneRelation::Disjoint,
            None,
            None,
            None,
            sides,
            predicates,
        ),
        (PlaneSide::Above, PlaneSide::Below) | (PlaneSide::Below, PlaneSide::Above) => {
            match construct_crossing_from_values(&d0, &d1, p0, p1) {
                Some((parameter, point)) => event(
                    SegmentPlaneRelation::ProperCrossing,
                    Some(point),
                    Some(parameter),
                    None,
                    sides,
                    predicates,
                ),
                None => event(
                    SegmentPlaneRelation::ConstructionFailed,
                    None,
                    None,
                    None,
                    sides,
                    predicates,
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
            sides,
            predicates,
        );
    };

    match (side0, side1) {
        (PlaneSide::On, PlaneSide::On) => event(
            SegmentPlaneRelation::Coplanar,
            None,
            None,
            None,
            sides,
            predicates,
        ),
        (PlaneSide::On, _) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p0.clone()),
            Some(ExactReal::from(0)),
            Some(0),
            sides,
            predicates,
        ),
        (_, PlaneSide::On) => event(
            SegmentPlaneRelation::EndpointOnPlane,
            Some(p1.clone()),
            Some(ExactReal::from(1)),
            Some(1),
            sides,
            predicates,
        ),
        (PlaneSide::Above, PlaneSide::Above) | (PlaneSide::Below, PlaneSide::Below) => event(
            SegmentPlaneRelation::Disjoint,
            None,
            None,
            None,
            sides,
            predicates,
        ),
        (PlaneSide::Above, PlaneSide::Below) | (PlaneSide::Below, PlaneSide::Above) => {
            let prepared = PreparedOrientedPlane3::new(a, b, c);
            match construct_crossing(prepared.plane(), p0, p1) {
                Some((parameter, point)) => event(
                    SegmentPlaneRelation::ProperCrossing,
                    Some(point),
                    Some(parameter),
                    None,
                    sides,
                    predicates,
                ),
                None => event(
                    SegmentPlaneRelation::ConstructionFailed,
                    None,
                    None,
                    None,
                    sides,
                    predicates,
                ),
            }
        }
    }
}

fn construct_crossing(plane: &Plane3, p0: &Point3, p1: &Point3) -> Option<(ExactReal, Point3)> {
    let d0 = point_plane_value(plane, p0);
    let d1 = point_plane_value(plane, p1);
    construct_crossing_from_values(&d0, &d1, p0, p1)
}

fn construct_crossing_from_values(
    d0: &ExactReal,
    d1: &ExactReal,
    p0: &Point3,
    p1: &Point3,
) -> Option<(ExactReal, Point3)> {
    let denominator = sub(&d0, &d1);
    if matches!(
        compare_reals(&denominator, &ExactReal::from(0)).value(),
        Some(core::cmp::Ordering::Equal)
    ) {
        return None;
    }
    let t = (d0 / &denominator).ok()?;
    let point = interpolate(p0, p1, &t);
    Some((t, point))
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
    endpoint_on_plane: Option<usize>,
    endpoint_sides: [Option<PlaneSide>; 2],
    predicates: Vec<PredicateUse>,
) -> SegmentPlaneIntersection {
    SegmentPlaneIntersection {
        relation,
        point,
        parameter,
        endpoint_on_plane,
        endpoint_sides,
        predicates,
    }
}

fn transpose_sides(sides: [Option<PlaneSide>; 2]) -> Option<[PlaneSide; 2]> {
    Some([sides[0]?, sides[1]?])
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
