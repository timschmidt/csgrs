//! Staged exact intersection and stationary-distance reports for retained surfaces.
//!
//! The currently supported analytic family is planar. Finite model-space lines
//! receive exact curve/plane classification, intersection parameters, and
//! closest witnesses. Plane pairs receive exact intersection lines or parallel
//! classification plus stationary separation evidence.

use hyperlimit::{
    HomogeneousLine3, Plane3, Point2, Point3, SegmentPlaneIntersection, SegmentPlaneRelation,
    intersect_segment_with_plane, intersect_two_planes, point_plane_value,
};
use hyperreal::{Real, RealSign};

use crate::{
    BrepCurve3, BrepCurveFamily3, BrepCurveGeometry3, BrepCurveSource3, BrepSurface, BrepSurfaceId,
    BrepSurfaceKind, BrepSurfaceSource,
};

/// Furthest analytic intersection stage reached.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepSurfaceIntersectionStage {
    /// Retained source/family evidence was checked.
    InputValidation,
    /// Exact analytic equations were classified.
    AnalyticClassification,
    /// Intersection or stationary-distance constructions were materialized.
    Complete,
}

/// Exact relation between a retained spatial curve and surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepCurveSurfaceIntersectionRelation {
    /// Curve and surface are disjoint over the retained parameter domain.
    Disjoint,
    /// One exact point intersection was constructed.
    Point,
    /// The supported curve lies in the surface.
    Coincident,
    /// The relation was not decided.
    Unknown,
}

/// Explicit curve/surface report blocker.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepCurveSurfaceBlocker {
    /// Surface provenance is lossy or unknown.
    NonExactSurfaceSource,
    /// Surface family is unsupported.
    UnsupportedSurface,
    /// Curve family has no current exact analytic intersection solver.
    UnsupportedCurveFamily,
    /// A required predicate remained undecidable.
    UnknownPredicate,
    /// A certified crossing could not materialize exact geometry.
    IntersectionConstructionFailed,
    /// Exact closest-point or distance construction failed.
    DistanceConstructionFailed,
}

/// Exact finite-curve/surface intersection and stationary-distance report.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepCurveSurfaceIntersectionReport {
    /// Stable curve source/version when retained.
    pub curve_source: Option<BrepCurveSource3>,
    /// Retained curve family.
    pub curve_family: BrepCurveFamily3,
    /// Retained surface id.
    pub surface: BrepSurfaceId,
    /// Furthest stage reached.
    pub stage: BrepSurfaceIntersectionStage,
    /// Exact classified relation.
    pub relation: BrepCurveSurfaceIntersectionRelation,
    /// Exact curve parameter for a unique point intersection.
    pub parameter: Option<Real>,
    /// Exact point intersection.
    pub point: Option<Point3>,
    /// Exact closest curve witness when supported.
    pub curve_witness: Option<Point3>,
    /// Exact closest surface witness when supported.
    pub surface_witness: Option<Point3>,
    /// Exact squared stationary/minimum distance for the supported finite line.
    pub squared_distance: Option<Real>,
    /// Exact/algebraic stationary/minimum distance.
    pub distance: Option<Real>,
    /// Underlying exact segment/plane event.
    pub segment_plane_event: Option<SegmentPlaneIntersection>,
    /// Explicit blockers.
    pub blockers: Vec<BrepCurveSurfaceBlocker>,
    /// Whether the relation was exactly classified.
    pub exact_classification_ready: bool,
    /// Whether stationary/minimum distance evidence is exact-ready.
    pub exact_distance_ready: bool,
}

/// Exact relation between two retained analytic surfaces.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepSurfaceIntersectionRelation {
    /// Surfaces intersect in one analytic curve.
    Curve,
    /// Surfaces are the same unbounded analytic set.
    Coincident,
    /// Surfaces are disjoint.
    Disjoint,
    /// The relation was not decided.
    Unknown,
}

/// Explicit surface/surface report blocker.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSurfaceIntersectionBlocker {
    /// At least one surface source is lossy or unknown.
    NonExactSource,
    /// At least one surface family is unsupported.
    UnsupportedSurface,
    /// Parallelism or coincidence could not be certified.
    UnknownClassification,
    /// Exact stationary-distance construction failed.
    DistanceConstructionFailed,
}

/// Exact analytic surface/surface intersection report.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceIntersectionReport {
    /// First retained surface id.
    pub first: BrepSurfaceId,
    /// Second retained surface id.
    pub second: BrepSurfaceId,
    /// Furthest stage reached.
    pub stage: BrepSurfaceIntersectionStage,
    /// Exact relation.
    pub relation: BrepSurfaceIntersectionRelation,
    /// Homogeneous Pluecker line for a plane/plane curve intersection.
    pub curve: Option<HomogeneousLine3>,
    /// Explicit blockers.
    pub blockers: Vec<BrepSurfaceIntersectionBlocker>,
    /// Whether analytic classification is exact-ready.
    pub exact_classification_ready: bool,
}

/// Exact stationary-distance report for two retained analytic surfaces.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceStationaryDistanceReport {
    /// Intersection classification reused by this report.
    pub intersection: BrepSurfaceIntersectionReport,
    /// Exact witness on the first surface when a finite representative was built.
    pub first_witness: Option<Point3>,
    /// Exact witness on the second surface when a finite representative was built.
    pub second_witness: Option<Point3>,
    /// Exact squared minimum distance.
    pub squared_distance: Option<Real>,
    /// Exact/algebraic minimum distance.
    pub distance: Option<Real>,
    /// Additional construction blockers.
    pub blockers: Vec<BrepSurfaceIntersectionBlocker>,
    /// Whether exact stationary-distance evidence is available.
    pub exact_distance_ready: bool,
}

impl BrepCurve3 {
    /// Intersect this retained model-space curve with an analytic surface.
    pub fn intersect_surface(&self, surface: &BrepSurface) -> BrepCurveSurfaceIntersectionReport {
        let mut blockers = Vec::new();
        if !exact_source(surface.source) {
            blockers.push(BrepCurveSurfaceBlocker::NonExactSurfaceSource);
        }
        let plane = match &surface.kind {
            BrepSurfaceKind::Plane(plane) if surface.is_supported_exact_plane() => {
                Some(plane.as_ref())
            }
            BrepSurfaceKind::Plane(_) => {
                blockers.push(BrepCurveSurfaceBlocker::UnsupportedSurface);
                None
            }
            BrepSurfaceKind::Unsupported { .. } => {
                blockers.push(BrepCurveSurfaceBlocker::UnsupportedSurface);
                None
            }
        };
        let line = match self.geometry() {
            BrepCurveGeometry3::Line(line) => Some(line.as_ref()),
            BrepCurveGeometry3::RationalBezier(_) | BrepCurveGeometry3::Nurbs(_) => {
                blockers.push(BrepCurveSurfaceBlocker::UnsupportedCurveFamily);
                None
            }
        };
        let (Some(plane), Some(line)) = (plane, line) else {
            return blocked_curve_report(self, surface, blockers);
        };
        if !blockers.is_empty() {
            return blocked_curve_report(self, surface, blockers);
        }

        let event = intersect_segment_with_plane(plane, line.start(), line.end());
        curve_plane_report(self, surface, plane, line.start(), line.end(), event)
    }
}

impl BrepSurface {
    /// Classify and construct the analytic intersection with another surface.
    pub fn intersect_surface(&self, other: &Self) -> BrepSurfaceIntersectionReport {
        let mut blockers = Vec::new();
        if !exact_source(self.source) || !exact_source(other.source) {
            blockers.push(BrepSurfaceIntersectionBlocker::NonExactSource);
        }
        let (first_plane, second_plane) = match (&self.kind, &other.kind) {
            (BrepSurfaceKind::Plane(first), BrepSurfaceKind::Plane(second))
                if self.is_supported_exact_plane() && other.is_supported_exact_plane() =>
            {
                (Some(first.as_ref()), Some(second.as_ref()))
            }
            _ => {
                blockers.push(BrepSurfaceIntersectionBlocker::UnsupportedSurface);
                (None, None)
            }
        };
        let (Some(first_plane), Some(second_plane)) = (first_plane, second_plane) else {
            return blocked_surface_report(self.id, other.id, blockers);
        };
        if !blockers.is_empty() {
            return blocked_surface_report(self.id, other.id, blockers);
        }

        match plane_pair_relation(first_plane, second_plane) {
            Some(BrepSurfaceIntersectionRelation::Curve) => BrepSurfaceIntersectionReport {
                first: self.id,
                second: other.id,
                stage: BrepSurfaceIntersectionStage::Complete,
                relation: BrepSurfaceIntersectionRelation::Curve,
                curve: Some(intersect_two_planes(first_plane, second_plane)),
                blockers,
                exact_classification_ready: true,
            },
            Some(relation) => BrepSurfaceIntersectionReport {
                first: self.id,
                second: other.id,
                stage: BrepSurfaceIntersectionStage::Complete,
                relation,
                curve: None,
                blockers,
                exact_classification_ready: true,
            },
            None => blocked_surface_report(
                self.id,
                other.id,
                vec![BrepSurfaceIntersectionBlocker::UnknownClassification],
            ),
        }
    }

    /// Compute exact stationary/minimum distance evidence to another surface.
    pub fn stationary_distance_to_surface(
        &self,
        other: &Self,
    ) -> BrepSurfaceStationaryDistanceReport {
        let intersection = self.intersect_surface(other);
        let mut blockers = intersection.blockers.clone();
        if !intersection.exact_classification_ready {
            return BrepSurfaceStationaryDistanceReport {
                intersection,
                first_witness: None,
                second_witness: None,
                squared_distance: None,
                distance: None,
                blockers,
                exact_distance_ready: false,
            };
        }
        if matches!(
            intersection.relation,
            BrepSurfaceIntersectionRelation::Curve | BrepSurfaceIntersectionRelation::Coincident
        ) {
            return BrepSurfaceStationaryDistanceReport {
                intersection,
                first_witness: None,
                second_witness: None,
                squared_distance: Some(Real::zero()),
                distance: Some(Real::zero()),
                blockers,
                exact_distance_ready: true,
            };
        }

        let (BrepSurfaceKind::Plane(_), BrepSurfaceKind::Plane(second)) = (&self.kind, &other.kind)
        else {
            unreachable!("ready disjoint classification currently implies two planes")
        };
        let first_evaluation = self.evaluate_frame_uv(Point2::new(Real::zero(), Real::zero()));
        let construction = first_evaluation
            .point
            .and_then(|point| project_point_to_plane(point, second));
        let Some((first_witness, second_witness, squared_distance, distance)) = construction else {
            blockers.push(BrepSurfaceIntersectionBlocker::DistanceConstructionFailed);
            return BrepSurfaceStationaryDistanceReport {
                intersection,
                first_witness: None,
                second_witness: None,
                squared_distance: None,
                distance: None,
                blockers,
                exact_distance_ready: false,
            };
        };
        BrepSurfaceStationaryDistanceReport {
            intersection,
            first_witness: Some(first_witness),
            second_witness: Some(second_witness),
            squared_distance: Some(squared_distance),
            distance: Some(distance),
            blockers,
            exact_distance_ready: true,
        }
    }
}

fn curve_plane_report(
    curve: &BrepCurve3,
    surface: &BrepSurface,
    plane: &Plane3,
    start: &Point3,
    end: &Point3,
    event: SegmentPlaneIntersection,
) -> BrepCurveSurfaceIntersectionReport {
    let mut blockers = Vec::new();
    let (relation, parameter, point) = match event.relation {
        SegmentPlaneRelation::ProperCrossing | SegmentPlaneRelation::EndpointOnPlane => (
            BrepCurveSurfaceIntersectionRelation::Point,
            event.parameter.clone(),
            event.point.clone(),
        ),
        SegmentPlaneRelation::Coplanar => {
            (BrepCurveSurfaceIntersectionRelation::Coincident, None, None)
        }
        SegmentPlaneRelation::Disjoint => {
            (BrepCurveSurfaceIntersectionRelation::Disjoint, None, None)
        }
        SegmentPlaneRelation::Unknown => {
            blockers.push(BrepCurveSurfaceBlocker::UnknownPredicate);
            (BrepCurveSurfaceIntersectionRelation::Unknown, None, None)
        }
        SegmentPlaneRelation::ConstructionFailed => {
            blockers.push(BrepCurveSurfaceBlocker::IntersectionConstructionFailed);
            (BrepCurveSurfaceIntersectionRelation::Unknown, None, None)
        }
    };

    let stationary = match relation {
        BrepCurveSurfaceIntersectionRelation::Point => point
            .clone()
            .map(|point| (point.clone(), point, Real::zero(), Real::zero())),
        BrepCurveSurfaceIntersectionRelation::Coincident => {
            Some((start.clone(), start.clone(), Real::zero(), Real::zero()))
        }
        BrepCurveSurfaceIntersectionRelation::Disjoint => {
            closest_segment_endpoint_to_plane(start, end, plane)
        }
        BrepCurveSurfaceIntersectionRelation::Unknown => None,
    };
    if relation != BrepCurveSurfaceIntersectionRelation::Unknown && stationary.is_none() {
        blockers.push(BrepCurveSurfaceBlocker::DistanceConstructionFailed);
    }
    let (curve_witness, surface_witness, squared_distance, distance) = stationary
        .map(
            |(curve_witness, surface_witness, squared_distance, distance)| {
                (
                    Some(curve_witness),
                    Some(surface_witness),
                    Some(squared_distance),
                    Some(distance),
                )
            },
        )
        .unwrap_or((None, None, None, None));
    let exact_classification_ready = blockers.iter().all(|blocker| {
        !matches!(
            blocker,
            BrepCurveSurfaceBlocker::UnknownPredicate
                | BrepCurveSurfaceBlocker::IntersectionConstructionFailed
        )
    });
    BrepCurveSurfaceIntersectionReport {
        curve_source: curve.source(),
        curve_family: curve.family(),
        surface: surface.id,
        stage: if exact_classification_ready {
            BrepSurfaceIntersectionStage::Complete
        } else {
            BrepSurfaceIntersectionStage::AnalyticClassification
        },
        relation,
        parameter,
        point,
        curve_witness,
        surface_witness,
        squared_distance,
        distance,
        segment_plane_event: Some(event),
        exact_classification_ready,
        exact_distance_ready: exact_classification_ready
            && !blockers.contains(&BrepCurveSurfaceBlocker::DistanceConstructionFailed),
        blockers,
    }
}

fn closest_segment_endpoint_to_plane(
    start: &Point3,
    end: &Point3,
    plane: &Plane3,
) -> Option<(Point3, Point3, Real, Real)> {
    let start_value = point_plane_value(plane, start);
    let end_value = point_plane_value(plane, end);
    let start_squared = start_value.clone() * start_value.clone();
    let end_squared = end_value.clone() * end_value.clone();
    let choice = (start_squared - end_squared).refine_sign_until(-64)?;
    let witness = match choice {
        RealSign::Negative | RealSign::Zero => start.clone(),
        RealSign::Positive => end.clone(),
    };
    project_point_to_plane(witness, plane)
}

fn project_point_to_plane(point: Point3, plane: &Plane3) -> Option<(Point3, Point3, Real, Real)> {
    let value = point_plane_value(plane, &point);
    let normal_squared = plane.normal.x.clone() * plane.normal.x.clone()
        + plane.normal.y.clone() * plane.normal.y.clone()
        + plane.normal.z.clone() * plane.normal.z.clone();
    let scale = (value.clone() / &normal_squared).ok()?;
    let projected = Point3::new(
        point.x.clone() - scale.clone() * plane.normal.x.clone(),
        point.y.clone() - scale.clone() * plane.normal.y.clone(),
        point.z.clone() - scale * plane.normal.z.clone(),
    );
    let squared_distance = (value.clone() * value / normal_squared).ok()?;
    let distance = squared_distance.clone().sqrt().ok()?;
    Some((point, projected, squared_distance, distance))
}

fn plane_pair_relation(first: &Plane3, second: &Plane3) -> Option<BrepSurfaceIntersectionRelation> {
    let cross = Point3::new(
        first.normal.y.clone() * second.normal.z.clone()
            - first.normal.z.clone() * second.normal.y.clone(),
        first.normal.z.clone() * second.normal.x.clone()
            - first.normal.x.clone() * second.normal.z.clone(),
        first.normal.x.clone() * second.normal.y.clone()
            - first.normal.y.clone() * second.normal.x.clone(),
    );
    match point_zero_status(&cross)? {
        false => Some(BrepSurfaceIntersectionRelation::Curve),
        true => {
            let pivot = [&first.normal.x, &first.normal.y, &first.normal.z]
                .into_iter()
                .position(|coordinate| {
                    matches!(
                        coordinate.refine_sign_until(-64),
                        Some(RealSign::Negative | RealSign::Positive)
                    )
                })?;
            let first_component = [&first.normal.x, &first.normal.y, &first.normal.z][pivot];
            let second_component = [&second.normal.x, &second.normal.y, &second.normal.z][pivot];
            let residual = second_component.clone() * first.offset.clone()
                - first_component.clone() * second.offset.clone();
            match residual.refine_sign_until(-64)? {
                RealSign::Zero => Some(BrepSurfaceIntersectionRelation::Coincident),
                RealSign::Negative | RealSign::Positive => {
                    Some(BrepSurfaceIntersectionRelation::Disjoint)
                }
            }
        }
    }
}

fn point_zero_status(point: &Point3) -> Option<bool> {
    let mut unknown = false;
    for coordinate in [&point.x, &point.y, &point.z] {
        match coordinate.refine_sign_until(-64) {
            Some(RealSign::Negative | RealSign::Positive) => return Some(false),
            Some(RealSign::Zero) => {}
            None => unknown = true,
        }
    }
    (!unknown).then_some(true)
}

fn exact_source(source: BrepSurfaceSource) -> bool {
    matches!(
        source,
        BrepSurfaceSource::ExactConstruction | BrepSurfaceSource::ExactImport
    )
}

fn blocked_curve_report(
    curve: &BrepCurve3,
    surface: &BrepSurface,
    blockers: Vec<BrepCurveSurfaceBlocker>,
) -> BrepCurveSurfaceIntersectionReport {
    BrepCurveSurfaceIntersectionReport {
        curve_source: curve.source(),
        curve_family: curve.family(),
        surface: surface.id,
        stage: BrepSurfaceIntersectionStage::InputValidation,
        relation: BrepCurveSurfaceIntersectionRelation::Unknown,
        parameter: None,
        point: None,
        curve_witness: None,
        surface_witness: None,
        squared_distance: None,
        distance: None,
        segment_plane_event: None,
        blockers,
        exact_classification_ready: false,
        exact_distance_ready: false,
    }
}

fn blocked_surface_report(
    first: BrepSurfaceId,
    second: BrepSurfaceId,
    blockers: Vec<BrepSurfaceIntersectionBlocker>,
) -> BrepSurfaceIntersectionReport {
    BrepSurfaceIntersectionReport {
        first,
        second,
        stage: BrepSurfaceIntersectionStage::InputValidation,
        relation: BrepSurfaceIntersectionRelation::Unknown,
        curve: None,
        blockers,
        exact_classification_ready: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BrepCurveGeometry3, BrepLineSegment3};

    fn r(value: i64) -> Real {
        Real::from(value)
    }

    fn q(numerator: i64, denominator: i64) -> Real {
        (r(numerator) / r(denominator)).unwrap()
    }

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn plane(id: u64, normal: Point3, offset: i64) -> BrepSurface {
        BrepSurface::plane(
            BrepSurfaceId::new(id),
            Plane3::new(normal, r(offset)),
            BrepSurfaceSource::ExactConstruction,
        )
    }

    fn line(start: Point3, end: Point3) -> BrepCurve3 {
        BrepCurve3::new(
            BrepCurveGeometry3::Line(Box::new(BrepLineSegment3::new(start, end))),
            Some(BrepCurveSource3::new(7)),
        )
    }

    #[test]
    fn line_plane_report_constructs_crossing_and_separated_distance() {
        let surface = plane(1, p(0, 0, 1), 0);
        let crossing = line(p(0, 0, -1), p(0, 0, 1)).intersect_surface(&surface);
        assert_eq!(
            crossing.relation,
            BrepCurveSurfaceIntersectionRelation::Point
        );
        assert_eq!(crossing.parameter, Some(q(1, 2)));
        assert_eq!(crossing.point, Some(p(0, 0, 0)));
        assert_eq!(crossing.squared_distance, Some(Real::zero()));
        assert!(crossing.exact_distance_ready);

        let separated = line(p(0, 0, 3), p(2, 0, 5)).intersect_surface(&surface);
        assert_eq!(
            separated.relation,
            BrepCurveSurfaceIntersectionRelation::Disjoint
        );
        assert_eq!(separated.curve_witness, Some(p(0, 0, 3)));
        assert_eq!(separated.surface_witness, Some(p(0, 0, 0)));
        assert_eq!(separated.squared_distance, Some(r(9)));
        assert_eq!(separated.distance, Some(r(3)));
    }

    #[test]
    fn plane_pair_reports_curve_coincidence_and_parallel_distance() {
        let x_zero = plane(2, p(1, 0, 0), 0);
        let y_zero = plane(3, p(0, 1, 0), 0);
        let crossing = x_zero.intersect_surface(&y_zero);
        assert_eq!(crossing.relation, BrepSurfaceIntersectionRelation::Curve);
        assert_eq!(crossing.curve.as_ref().unwrap().direction, p(0, 0, 1));

        let same = plane(4, p(2, 0, 0), 0);
        assert_eq!(
            x_zero.intersect_surface(&same).relation,
            BrepSurfaceIntersectionRelation::Coincident
        );

        let x_three = plane(5, p(1, 0, 0), -3);
        let distance = x_zero.stationary_distance_to_surface(&x_three);
        assert_eq!(
            distance.intersection.relation,
            BrepSurfaceIntersectionRelation::Disjoint
        );
        assert_eq!(distance.squared_distance, Some(r(9)));
        assert_eq!(distance.distance, Some(r(3)));
        assert_eq!(distance.first_witness.as_ref().unwrap().x, r(0));
        assert_eq!(distance.second_witness.as_ref().unwrap().x, r(3));
        assert!(distance.exact_distance_ready);
    }
}
