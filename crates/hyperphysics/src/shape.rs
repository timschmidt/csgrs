//! Exact physics shape carriers and first query reports.
//!
//! This module keeps shape classification at the object layer before any
//! runtime collision engine is chosen. Exact boxes and support maps are carried
//! as Hyper-native facts, while mesh closure/manifold proof remains delegated to
//! future exact mesh reports. This follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): geometric decisions should
//! use certified object facts and exact predicates instead of hidden
//! primitive-float tolerances.
//!
//! The support-map surface is the first dependency-removal hook for collision
//! algorithms such as GJK. See Gilbert, Johnson, and Keerthi, "A fast procedure
//! for computing the distance between complex objects in three-dimensional
//! space," *IEEE Journal of Robotics and Automation* 4(2), 1988. The code here
//! only reports exact support points for boxes; broader collision algorithms
//! must replay their decisions through certified Hyper reports before replacing
//! Parry/Rapier query paths.
//!
//! Plane/ray/segment classification uses exact oriented signed distances, the
//! primitive required by many collision pipelines before interval or continuous
//! contact certificates are available.
//!
//! Point/triangle classification uses exact same-side edge tests against the
//! triangle normal. This is the orientation-test form common in computational
//! geometry texts, and it keeps the predicate replayable without projecting to
//! primitive floats.

use hyperlattice::{Point3, Vector3};
use hyperlimit::{
    Aabb3Intersection, Aabb3PointLocation, PlaneSegmentRelation, PlaneSide, PredicateOutcome,
    Triangle3Location,
};
use hyperreal::{Real, RealSign};

use crate::{PhysicsError, PhysicsResult};

/// Triangle with exact 3D vertices.
#[derive(Clone, Debug, PartialEq)]
pub struct Triangle3 {
    vertices: [Vector3; 3],
}

/// Closed oriented triangle mesh used for exact uniform-density reports.
#[derive(Clone, Debug, PartialEq)]
pub struct ClosedTriangleMesh3 {
    triangles: Vec<Triangle3>,
}

/// Exact axis-aligned box by inclusive min/max corners.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisAlignedBox3 {
    /// Minimum corner.
    pub min: Vector3,
    /// Maximum corner.
    pub max: Vector3,
}

/// Exact ray with origin and direction.
#[derive(Clone, Debug, PartialEq)]
pub struct Ray3 {
    /// Ray origin.
    pub origin: Vector3,
    /// Ray direction.
    pub direction: Vector3,
}

/// Exact segment with start/end points.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment3 {
    /// Start point.
    pub start: Vector3,
    /// End point.
    pub end: Vector3,
}

/// Exact plane represented by one point and a nonzero normal supplied by caller.
#[derive(Clone, Debug, PartialEq)]
pub struct Plane3 {
    /// A point on the plane.
    pub point: Vector3,
    /// Plane normal.
    pub normal: Vector3,
}

/// Shape variants understood by the first exact physics boundary.
#[derive(Clone, Debug, PartialEq)]
pub enum PhysicsShape3 {
    /// Closed oriented triangle surface.
    ClosedTriangleMesh(ClosedTriangleMesh3),
    /// Exact axis-aligned box.
    AxisAlignedBox(Box<AxisAlignedBox3>),
}

/// Point relation to a box.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoxPointClassification {
    /// Point is strictly inside.
    Inside,
    /// Point lies on at least one boundary plane and outside none.
    Boundary,
    /// Point is outside.
    Outside,
    /// Classification could not be certified.
    Unknown,
}

/// Shape classification facts.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ShapeClassificationReport3 {
    /// Human-readable shape family.
    pub family: &'static str,
    /// Whether the shape is convex.
    pub convex: bool,
    /// Whether the shape is axis aligned.
    pub axis_aligned: bool,
    /// Whether support mapping is exact and implemented.
    pub exact_support_map: bool,
}

/// Exact support-map report for convex shapes.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportMapReport3 {
    /// Direction queried.
    pub direction: Vector3,
    /// Exact support point.
    pub support_point: Vector3,
    /// Whether the direction was the zero vector.
    pub zero_direction: bool,
}

/// Point relation to an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlanePointClassification {
    /// Point is on the positive-normal side.
    Positive,
    /// Point is on the negative-normal side.
    Negative,
    /// Point lies on the plane.
    OnPlane,
}

/// Ray relation to a plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RayPlaneClassification {
    /// Ray origin lies on the plane.
    OriginOnPlane,
    /// Ray intersects the plane at non-negative ray parameter.
    ForwardIntersection,
    /// Plane intersection lies behind the ray origin.
    BehindOrigin,
    /// Ray is parallel and not on the plane.
    Parallel,
}

/// Segment relation to a plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentPlaneClassification {
    /// Both segment endpoints lie on the plane.
    Coplanar,
    /// Start point lies on the plane.
    StartOnPlane,
    /// End point lies on the plane.
    EndOnPlane,
    /// Segment endpoints are on opposite sides.
    Crosses,
    /// Both endpoints are positive.
    PositiveSide,
    /// Both endpoints are negative.
    NegativeSide,
}

/// Point relation to a triangle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TrianglePointClassification {
    /// Point is strictly inside the triangle.
    Inside,
    /// Point lies on a triangle edge or vertex.
    Boundary,
    /// Point is coplanar but outside the triangle.
    Outside,
    /// Point is not on the triangle plane.
    OffPlane,
    /// Triangle is degenerate and no orientation predicate is available.
    DegenerateTriangle,
}

/// Exact point/plane query report.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanePointReport3 {
    /// Exact signed distance numerator `(point - plane.point) dot normal`.
    pub signed_distance: Real,
    /// Certified classification.
    pub classification: PlanePointClassification,
}

/// Exact ray/plane query report.
#[derive(Clone, Debug, PartialEq)]
pub struct RayPlaneReport3 {
    /// Exact ray origin signed distance.
    pub origin_signed_distance: Real,
    /// Exact direction denominator `direction dot normal`.
    pub direction_dot_normal: Real,
    /// Exact ray parameter when the ray is not parallel.
    pub parameter: Option<Real>,
    /// Certified classification.
    pub classification: RayPlaneClassification,
}

/// Exact segment/plane query report.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentPlaneReport3 {
    /// Exact start signed distance.
    pub start_signed_distance: Real,
    /// Exact end signed distance.
    pub end_signed_distance: Real,
    /// Certified classification.
    pub classification: SegmentPlaneClassification,
}

/// Exact point/triangle query report.
#[derive(Clone, Debug, PartialEq)]
pub struct TrianglePointReport3 {
    /// Exact plane signed-distance numerator.
    pub plane_signed_distance: Real,
    /// Exact edge-orientation signs in triangle edge order.
    pub edge_signs: Option<[RealSign; 3]>,
    /// Certified classification.
    pub classification: TrianglePointClassification,
}

impl Triangle3 {
    /// Creates a triangle from three exact vertices.
    pub const fn new(vertices: [Vector3; 3]) -> Self {
        Self { vertices }
    }

    /// Returns vertices in `[a, b, c]` order.
    pub const fn vertices(&self) -> &[Vector3; 3] {
        &self.vertices
    }

    /// Classifies a point against this triangle using exact orientation tests.
    pub fn classify_point(&self, point: &Vector3) -> PhysicsResult<TrianglePointReport3> {
        let [a, b, c] = self.vertices();
        let ab = b - a;
        let ac = c - a;
        let normal = ab.cross(&ac);
        let plane_signed_distance = (point - a).dot(&normal);

        let location = decide(hyperlimit::classify_point_triangle3(
            &point3_from_vector(a),
            &point3_from_vector(b),
            &point3_from_vector(c),
            &point3_from_vector(point),
        ))?;
        let classification = match location {
            Triangle3Location::Degenerate => TrianglePointClassification::DegenerateTriangle,
            Triangle3Location::OffPlane => TrianglePointClassification::OffPlane,
            Triangle3Location::Outside => TrianglePointClassification::Outside,
            Triangle3Location::Inside => TrianglePointClassification::Inside,
            Triangle3Location::OnEdge | Triangle3Location::OnVertex => {
                TrianglePointClassification::Boundary
            }
        };
        let edge_signs = match classification {
            TrianglePointClassification::Inside
            | TrianglePointClassification::Boundary
            | TrianglePointClassification::Outside => Some([
                sign(&(b - a).cross(&(point - a)).dot(&normal))?,
                sign(&(c - b).cross(&(point - b)).dot(&normal))?,
                sign(&(a - c).cross(&(point - c)).dot(&normal))?,
            ]),
            TrianglePointClassification::OffPlane
            | TrianglePointClassification::DegenerateTriangle => None,
        };
        Ok(TrianglePointReport3 {
            plane_signed_distance,
            edge_signs,
            classification,
        })
    }
}

impl ClosedTriangleMesh3 {
    /// Creates a closed triangle mesh carrier.
    ///
    /// The constructor only checks that some facets exist. Closure and
    /// orientation are semantic obligations of the caller or upstream geometry
    /// crate until hypermesh owns exact manifold validation.
    pub fn new(triangles: Vec<Triangle3>) -> PhysicsResult<Self> {
        if triangles.is_empty() {
            return Err(PhysicsError::EmptyTriangleMesh);
        }
        Ok(Self { triangles })
    }

    /// Returns the mesh triangles.
    pub fn triangles(&self) -> &[Triangle3] {
        &self.triangles
    }

    /// Returns the number of triangle facets.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Lowers this physics mesh carrier into hypermesh exact topology storage.
    ///
    /// Physics keeps material and mass-property interpretation. Mesh
    /// validation and topology facts are delegated to hypermesh at this
    /// boundary.
    pub fn to_hypermesh_exact(&self) -> hypermesh::HypermeshResult<hypermesh::InputMesh> {
        let mut positions = Vec::with_capacity(self.triangles.len() * 3);
        let mut triangles = Vec::with_capacity(self.triangles.len());
        for triangle in &self.triangles {
            let base = positions.len();
            for vertex in triangle.vertices() {
                positions.push(Point3::new(
                    vertex[0].clone(),
                    vertex[1].clone(),
                    vertex[2].clone(),
                ));
            }
            triangles.push(hypermesh::Triangle::new(base, base + 1, base + 2));
        }
        let mesh = hypermesh::InputMesh::new(positions, triangles);
        hypermesh::prepare_input(&[mesh.as_ref()])?;
        Ok(mesh)
    }
}

impl AxisAlignedBox3 {
    /// Creates an exact axis-aligned box after certifying `min <= max` on every axis.
    pub fn new(min: Vector3, max: Vector3) -> PhysicsResult<Self> {
        for axis in 0..3 {
            if !leq(&min[axis], &max[axis])? {
                return Err(PhysicsError::InvalidAxisAlignedBox);
            }
        }
        Ok(Self { min, max })
    }

    /// Classifies a point against the inclusive box.
    pub fn classify_point(&self, point: &Vector3) -> PhysicsResult<BoxPointClassification> {
        match decide(hyperlimit::classify_point_aabb3(
            &point3_from_vector(&self.min),
            &point3_from_vector(&self.max),
            &point3_from_vector(point),
        ))? {
            Aabb3PointLocation::Outside => Ok(BoxPointClassification::Outside),
            Aabb3PointLocation::Boundary => Ok(BoxPointClassification::Boundary),
            Aabb3PointLocation::Inside => Ok(BoxPointClassification::Inside),
        }
    }

    /// Returns true when two inclusive AABBs are certified disjoint.
    pub fn certified_disjoint(&self, other: &Self) -> PhysicsResult<bool> {
        Ok(matches!(
            decide(hyperlimit::classify_aabb3_intersection(
                &point3_from_vector(&self.min),
                &point3_from_vector(&self.max),
                &point3_from_vector(&other.min),
                &point3_from_vector(&other.max),
            ))?,
            Aabb3Intersection::Disjoint
        ))
    }

    /// Returns an exact support point in the requested direction.
    ///
    /// For a zero direction, this returns the minimum corner and marks
    /// `zero_direction` so GJK-style callers cannot mistake the point for a
    /// unique support witness.
    pub fn support_map(&self, direction: Vector3) -> PhysicsResult<SupportMapReport3> {
        let mut zero_direction = true;
        let support_point = Vector3::new([
            choose_support_axis(
                &self.min[0],
                &self.max[0],
                &direction[0],
                &mut zero_direction,
            )?,
            choose_support_axis(
                &self.min[1],
                &self.max[1],
                &direction[1],
                &mut zero_direction,
            )?,
            choose_support_axis(
                &self.min[2],
                &self.max[2],
                &direction[2],
                &mut zero_direction,
            )?,
        ]);
        Ok(SupportMapReport3 {
            direction,
            support_point,
            zero_direction,
        })
    }
}

impl Ray3 {
    /// Creates an exact ray carrier.
    pub const fn new(origin: Vector3, direction: Vector3) -> Self {
        Self { origin, direction }
    }
}

impl Segment3 {
    /// Creates an exact segment carrier.
    pub const fn new(start: Vector3, end: Vector3) -> Self {
        Self { start, end }
    }
}

impl Plane3 {
    /// Creates an exact plane carrier.
    pub const fn new(point: Vector3, normal: Vector3) -> Self {
        Self { point, normal }
    }

    /// Computes exact signed point distance numerator and classification.
    pub fn classify_point(&self, point: &Vector3) -> PhysicsResult<PlanePointReport3> {
        let signed_distance = (point - &self.point).dot(&self.normal);
        let predicate_plane = self.predicate_plane();
        let classification = match decide(hyperlimit::classify_point_plane(
            &point3_from_vector(point),
            &predicate_plane,
        ))? {
            PlaneSide::Above => PlanePointClassification::Positive,
            PlaneSide::Below => PlanePointClassification::Negative,
            PlaneSide::On => PlanePointClassification::OnPlane,
        };
        Ok(PlanePointReport3 {
            signed_distance,
            classification,
        })
    }

    /// Classifies an exact ray against this plane.
    pub fn classify_ray(&self, ray: &Ray3) -> PhysicsResult<RayPlaneReport3> {
        let origin_signed_distance = (&ray.origin - &self.point).dot(&self.normal);
        let direction_dot_normal = ray.direction.dot(&self.normal);
        let origin_sign = sign(&origin_signed_distance)?;
        let direction_sign = sign(&direction_dot_normal)?;
        let (parameter, classification) = match (origin_sign, direction_sign) {
            (RealSign::Zero, _) => (Some(Real::zero()), RayPlaneClassification::OriginOnPlane),
            (_, RealSign::Zero) => (None, RayPlaneClassification::Parallel),
            _ => {
                let parameter = (-origin_signed_distance.clone() / direction_dot_normal.clone())
                    .map_err(|_| PhysicsError::UnknownShapeQuery)?;
                let parameter_sign = sign(&parameter)?;
                let classification = match parameter_sign {
                    RealSign::Negative => RayPlaneClassification::BehindOrigin,
                    RealSign::Zero | RealSign::Positive => {
                        RayPlaneClassification::ForwardIntersection
                    }
                };
                (Some(parameter), classification)
            }
        };
        Ok(RayPlaneReport3 {
            origin_signed_distance,
            direction_dot_normal,
            parameter,
            classification,
        })
    }

    /// Classifies an exact segment against this plane.
    pub fn classify_segment(&self, segment: &Segment3) -> PhysicsResult<SegmentPlaneReport3> {
        let start_signed_distance = (&segment.start - &self.point).dot(&self.normal);
        let end_signed_distance = (&segment.end - &self.point).dot(&self.normal);
        let predicate_plane = self.predicate_plane();
        let classification = match decide(hyperlimit::classify_plane_segment(
            &predicate_plane,
            &point3_from_vector(&segment.start),
            &point3_from_vector(&segment.end),
        ))? {
            PlaneSegmentRelation::Coplanar => SegmentPlaneClassification::Coplanar,
            PlaneSegmentRelation::Crossing => SegmentPlaneClassification::Crosses,
            PlaneSegmentRelation::Above => SegmentPlaneClassification::PositiveSide,
            PlaneSegmentRelation::Below => SegmentPlaneClassification::NegativeSide,
            PlaneSegmentRelation::EndpointTouch => {
                let start_sign = sign(&start_signed_distance)?;
                if start_sign == RealSign::Zero {
                    SegmentPlaneClassification::StartOnPlane
                } else {
                    SegmentPlaneClassification::EndOnPlane
                }
            }
        };
        Ok(SegmentPlaneReport3 {
            start_signed_distance,
            end_signed_distance,
            classification,
        })
    }

    fn predicate_plane(&self) -> hyperlimit::Plane3 {
        hyperlimit::Plane3::new(
            point3_from_vector(&self.normal),
            -self.point.dot(&self.normal),
        )
    }
}

impl PhysicsShape3 {
    /// Wraps a closed triangle mesh shape.
    pub const fn closed_triangle_mesh(mesh: ClosedTriangleMesh3) -> Self {
        Self::ClosedTriangleMesh(mesh)
    }

    /// Wraps an exact axis-aligned box.
    pub fn axis_aligned_box(aabb: AxisAlignedBox3) -> Self {
        Self::AxisAlignedBox(Box::new(aabb))
    }

    /// Returns cheap retained shape facts for dispatch.
    pub fn classification_report(&self) -> ShapeClassificationReport3 {
        match self {
            Self::ClosedTriangleMesh(_) => ShapeClassificationReport3 {
                family: "closed-triangle-mesh",
                convex: false,
                axis_aligned: false,
                exact_support_map: false,
            },
            Self::AxisAlignedBox(_) => ShapeClassificationReport3 {
                family: "axis-aligned-box",
                convex: true,
                axis_aligned: true,
                exact_support_map: true,
            },
        }
    }
}

fn choose_support_axis(
    min: &Real,
    max: &Real,
    direction: &Real,
    zero_direction: &mut bool,
) -> PhysicsResult<Real> {
    match sign(direction)? {
        RealSign::Positive => {
            *zero_direction = false;
            Ok(max.clone())
        }
        RealSign::Negative => {
            *zero_direction = false;
            Ok(min.clone())
        }
        RealSign::Zero => Ok(min.clone()),
    }
}

fn sign(value: &Real) -> PhysicsResult<RealSign> {
    match hyperlimit::compare_reals(value, &Real::zero()).value() {
        Some(core::cmp::Ordering::Less) => Ok(RealSign::Negative),
        Some(core::cmp::Ordering::Equal) => Ok(RealSign::Zero),
        Some(core::cmp::Ordering::Greater) => Ok(RealSign::Positive),
        None => Err(PhysicsError::UnknownShapeQuery),
    }
}

fn leq(left: &Real, right: &Real) -> PhysicsResult<bool> {
    Ok(!matches!(
        sign(&(left.clone() - right.clone()))?,
        RealSign::Positive
    ))
}

fn decide<T>(outcome: PredicateOutcome<T>) -> PhysicsResult<T> {
    outcome.value().ok_or(PhysicsError::UnknownShapeQuery)
}

fn point3_from_vector(vector: &Vector3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(vector[0].clone(), vector[1].clone(), vector[2].clone())
}
