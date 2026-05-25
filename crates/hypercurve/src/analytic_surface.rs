//! Retained analytic surface frames and exact point replay reports.
//!
//! BREP faces on analytic supports need more than a surface identifier: the
//! parameter frame, seam convention, and bounded domain are part of the
//! evidence that decides whether a query is on the face support before trim
//! topology is considered. This module starts that layer with exact retained
//! cylindrical frames. The construction/predicate split follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7(1-2), 3-23
//! (1997), while the parametric cylinder convention follows Piegl and Tiller,
//! *The NURBS Book* (2nd ed., 1997), where analytic surfaces carry explicit
//! domains and periodic seams.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::{Classification, CurveError, CurvePolicy, CurveResult, UncertaintyReason};

/// Exact 3D point used by retained analytic surface frames.
#[derive(Clone, Debug, PartialEq)]
pub struct Point3 {
    x: Real,
    y: Real,
    z: Real,
}

/// Opaque identity of a retained analytic support surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RetainedAnalyticSurfaceIdentity3 {
    source_index: u64,
}

/// Signed coordinate axis used by exact axis-aligned analytic frames.
///
/// This is intentionally a frame certificate, not a general floating vector.
/// Axis-aligned retained frames are enough to begin exact cylinder support
/// replay without introducing normalized approximate directions. General
/// workplane and non-axis frames should enter through exact transform evidence
/// rather than silently storing primitive floats.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SignedAxis3 {
    /// Positive X axis.
    PosX,
    /// Negative X axis.
    NegX,
    /// Positive Y axis.
    PosY,
    /// Negative Y axis.
    NegY,
    /// Positive Z axis.
    PosZ,
    /// Negative Z axis.
    NegZ,
}

/// Unsigned coordinate axis in 3D.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Axis3 {
    /// X axis.
    X,
    /// Y axis.
    Y,
    /// Z axis.
    Z,
}

/// Optional exact interval for a cylinder's axial parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedCylinderAxialDomain3 {
    min: Option<Real>,
    max: Option<Real>,
}

/// Retained axis-aligned cylindrical analytic surface frame.
///
/// The frame stores an origin on the cylinder axis, a signed axis direction,
/// an exact squared radius, a signed seam ray perpendicular to the axis, and
/// an optional bounded axial domain. Point replay checks the polynomial
/// cylinder equation in the two radial coordinates and only then reports seam
/// and axial-domain evidence. This is the exact-support counterpart to later
/// trim-boundary tests.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedCylinderFrame3 {
    surface: RetainedAnalyticSurfaceIdentity3,
    origin: Point3,
    axis: SignedAxis3,
    seam: SignedAxis3,
    radius_squared: Real,
    axial_domain: RetainedCylinderAxialDomain3,
}

/// Exact seam status for a retained analytic support query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedAnalyticSeamRelation3 {
    /// The support is periodic, but the query is not on the selected seam ray.
    NotOnSeam,
    /// The query is on the selected periodic seam ray.
    OnSeam,
}

/// Exact pole status for a retained analytic support query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedAnalyticPoleRelation3 {
    /// Cylinders have no pole singularity under this retained frame model.
    NotApplicable,
}

/// Point replay relation against a retained cylinder support.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedCylinderPointRelation3 {
    /// The point satisfies the cylinder equation and lies in the axial domain.
    OnSurface,
    /// The point satisfies the cylinder equation, lies in the axial domain, and
    /// also lies on the selected periodic seam ray.
    OnSeam,
    /// The squared radial distance does not match the retained squared radius.
    OutsideRadius,
    /// The radial equation matches, but the axial coordinate is outside the
    /// retained bounded domain.
    OutsideAxialDomain,
}

/// Evidence report for exact point replay against a retained cylinder.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedCylinderPointReport3 {
    relation: RetainedCylinderPointRelation3,
    surface: RetainedAnalyticSurfaceIdentity3,
    seam: RetainedAnalyticSeamRelation3,
    pole: RetainedAnalyticPoleRelation3,
    axial_coordinate: Real,
    radial_squared: Real,
    radius_squared: Real,
    axial_domain_boundary: bool,
}

impl Point3 {
    /// Constructs an exact 3D point.
    pub const fn new(x: Real, y: Real, z: Real) -> Self {
        Self { x, y, z }
    }

    /// Returns the x coordinate.
    pub const fn x(&self) -> &Real {
        &self.x
    }

    /// Returns the y coordinate.
    pub const fn y(&self) -> &Real {
        &self.y
    }

    /// Returns the z coordinate.
    pub const fn z(&self) -> &Real {
        &self.z
    }
}

impl RetainedAnalyticSurfaceIdentity3 {
    /// Constructs an opaque retained analytic surface identity.
    pub const fn new(source_index: u64) -> Self {
        Self { source_index }
    }

    /// Returns the opaque source index for this analytic support surface.
    pub const fn source_index(self) -> u64 {
        self.source_index
    }
}

impl SignedAxis3 {
    /// Returns the unsigned coordinate axis.
    pub const fn axis(self) -> Axis3 {
        match self {
            Self::PosX | Self::NegX => Axis3::X,
            Self::PosY | Self::NegY => Axis3::Y,
            Self::PosZ | Self::NegZ => Axis3::Z,
        }
    }

    /// Returns whether this signed axis points in the negative direction.
    pub const fn is_negative(self) -> bool {
        matches!(self, Self::NegX | Self::NegY | Self::NegZ)
    }
}

impl RetainedCylinderAxialDomain3 {
    /// Constructs an unbounded axial domain.
    pub const fn unbounded() -> Self {
        Self {
            min: None,
            max: None,
        }
    }

    /// Constructs a bounded axial domain after exact interval validation.
    pub fn bounded(min: Real, max: Real, policy: &CurvePolicy) -> CurveResult<Self> {
        match crate::classify::compare_reals(&min, &max, policy) {
            Some(Ordering::Greater) | None => Err(CurveError::InvalidAnalyticSurfaceFrame),
            Some(Ordering::Less | Ordering::Equal) => Ok(Self {
                min: Some(min),
                max: Some(max),
            }),
        }
    }

    /// Returns the lower axial bound, if any.
    pub const fn min(&self) -> Option<&Real> {
        self.min.as_ref()
    }

    /// Returns the upper axial bound, if any.
    pub const fn max(&self) -> Option<&Real> {
        self.max.as_ref()
    }
}

impl RetainedCylinderFrame3 {
    /// Constructs a retained exact cylinder frame.
    ///
    /// The seam ray must be perpendicular to the cylinder axis and
    /// `radius_squared` must be certified positive. The radius is stored
    /// squared so point replay stays polynomial:
    /// `(p_u-o_u)^2 + (p_v-o_v)^2 - r^2 == 0`. This avoids square roots and
    /// angle sampling at the topology boundary, following Yap's exact
    /// predicate discipline.
    pub fn try_new(
        surface: RetainedAnalyticSurfaceIdentity3,
        origin: Point3,
        axis: SignedAxis3,
        seam: SignedAxis3,
        radius_squared: Real,
        axial_domain: RetainedCylinderAxialDomain3,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        if axis.axis() == seam.axis() {
            return Err(CurveError::InvalidAnalyticSurfaceFrame);
        }
        if !matches!(
            crate::classify::real_sign(&radius_squared, policy),
            Some(RealSign::Positive)
        ) {
            return Err(CurveError::InvalidAnalyticSurfaceFrame);
        }

        Ok(Self {
            surface,
            origin,
            axis,
            seam,
            radius_squared,
            axial_domain,
        })
    }

    /// Returns the retained analytic support identity.
    pub const fn surface(&self) -> RetainedAnalyticSurfaceIdentity3 {
        self.surface
    }

    /// Returns the retained origin on the cylinder axis.
    pub const fn origin(&self) -> &Point3 {
        &self.origin
    }

    /// Returns the signed cylinder axis.
    pub const fn axis(&self) -> SignedAxis3 {
        self.axis
    }

    /// Returns the signed seam ray.
    pub const fn seam(&self) -> SignedAxis3 {
        self.seam
    }

    /// Returns the exact squared radius.
    pub const fn radius_squared(&self) -> &Real {
        &self.radius_squared
    }

    /// Returns the retained axial domain.
    pub const fn axial_domain(&self) -> &RetainedCylinderAxialDomain3 {
        &self.axial_domain
    }

    /// Replays an exact point-on-cylinder support query.
    ///
    /// The report separates radial equation failure, axial-domain failure,
    /// seam membership, and pole status. Cylinders have no pole, so the pole
    /// field is always [`RetainedAnalyticPoleRelation3::NotApplicable`]; it is
    /// still part of the report to keep the analytic-surface vocabulary stable
    /// for cones and spheres.
    pub fn classify_point(
        &self,
        point: &Point3,
        policy: &CurvePolicy,
    ) -> Classification<RetainedCylinderPointReport3> {
        let axial_coordinate = signed_delta_component(point, &self.origin, self.axis);
        let radial_squared = self.radial_squared(point);
        let radius_delta = &radial_squared - &self.radius_squared;
        match crate::classify::is_zero(&radius_delta, policy) {
            Some(false) => {
                return Classification::Decided(self.point_report(
                    RetainedCylinderPointRelation3::OutsideRadius,
                    RetainedAnalyticSeamRelation3::NotOnSeam,
                    axial_coordinate,
                    radial_squared,
                    false,
                ));
            }
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
            Some(true) => {}
        }

        let axial_boundary =
            match axial_domain_location(&axial_coordinate, &self.axial_domain, policy) {
                Classification::Decided(AxialDomainLocation::Inside) => false,
                Classification::Decided(AxialDomainLocation::Boundary) => true,
                Classification::Decided(AxialDomainLocation::Outside) => {
                    return Classification::Decided(self.point_report(
                        RetainedCylinderPointRelation3::OutsideAxialDomain,
                        RetainedAnalyticSeamRelation3::NotOnSeam,
                        axial_coordinate,
                        radial_squared,
                        false,
                    ));
                }
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };

        let seam = match self.classify_seam(point, policy) {
            Classification::Decided(seam) => seam,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let relation = if seam == RetainedAnalyticSeamRelation3::OnSeam {
            RetainedCylinderPointRelation3::OnSeam
        } else {
            RetainedCylinderPointRelation3::OnSurface
        };

        Classification::Decided(self.point_report(
            relation,
            seam,
            axial_coordinate,
            radial_squared,
            axial_boundary,
        ))
    }

    fn radial_squared(&self, point: &Point3) -> Real {
        let [first, second] = radial_axes(self.axis.axis());
        let first_delta = delta_component(point, &self.origin, first);
        let second_delta = delta_component(point, &self.origin, second);
        &(&first_delta * &first_delta) + &(&second_delta * &second_delta)
    }

    fn classify_seam(
        &self,
        point: &Point3,
        policy: &CurvePolicy,
    ) -> Classification<RetainedAnalyticSeamRelation3> {
        let seam_component = signed_delta_component(point, &self.origin, self.seam);
        let lateral_component = delta_component(point, &self.origin, other_radial_axis(self));
        match crate::classify::is_zero(&lateral_component, policy) {
            Some(false) => Classification::Decided(RetainedAnalyticSeamRelation3::NotOnSeam),
            None => Classification::Uncertain(UncertaintyReason::RealSign),
            Some(true) => match crate::classify::real_sign(&seam_component, policy) {
                Some(RealSign::Positive) => {
                    Classification::Decided(RetainedAnalyticSeamRelation3::OnSeam)
                }
                Some(RealSign::Negative | RealSign::Zero) => {
                    Classification::Decided(RetainedAnalyticSeamRelation3::NotOnSeam)
                }
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            },
        }
    }

    fn point_report(
        &self,
        relation: RetainedCylinderPointRelation3,
        seam: RetainedAnalyticSeamRelation3,
        axial_coordinate: Real,
        radial_squared: Real,
        axial_domain_boundary: bool,
    ) -> RetainedCylinderPointReport3 {
        RetainedCylinderPointReport3 {
            relation,
            surface: self.surface,
            seam,
            pole: RetainedAnalyticPoleRelation3::NotApplicable,
            axial_coordinate,
            radial_squared,
            radius_squared: self.radius_squared.clone(),
            axial_domain_boundary,
        }
    }
}

impl RetainedCylinderPointReport3 {
    /// Returns the point-support relation.
    pub const fn relation(&self) -> RetainedCylinderPointRelation3 {
        self.relation
    }

    /// Returns the retained analytic support surface.
    pub const fn surface(&self) -> RetainedAnalyticSurfaceIdentity3 {
        self.surface
    }

    /// Returns the seam evidence.
    pub const fn seam(&self) -> RetainedAnalyticSeamRelation3 {
        self.seam
    }

    /// Returns the pole evidence.
    pub const fn pole(&self) -> RetainedAnalyticPoleRelation3 {
        self.pole
    }

    /// Returns the exact signed axial coordinate.
    pub const fn axial_coordinate(&self) -> &Real {
        &self.axial_coordinate
    }

    /// Returns the exact squared radial distance from the cylinder axis.
    pub const fn radial_squared(&self) -> &Real {
        &self.radial_squared
    }

    /// Returns the retained exact squared radius.
    pub const fn radius_squared(&self) -> &Real {
        &self.radius_squared
    }

    /// Returns whether the axial coordinate lies on a retained domain bound.
    pub const fn axial_domain_boundary(&self) -> bool {
        self.axial_domain_boundary
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AxialDomainLocation {
    Inside,
    Boundary,
    Outside,
}

fn axial_domain_location(
    axial_coordinate: &Real,
    domain: &RetainedCylinderAxialDomain3,
    policy: &CurvePolicy,
) -> Classification<AxialDomainLocation> {
    let mut boundary = false;
    if let Some(min) = domain.min() {
        match crate::classify::compare_reals(axial_coordinate, min, policy) {
            Some(Ordering::Less) => return Classification::Decided(AxialDomainLocation::Outside),
            Some(Ordering::Equal) => boundary = true,
            Some(Ordering::Greater) => {}
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    if let Some(max) = domain.max() {
        match crate::classify::compare_reals(axial_coordinate, max, policy) {
            Some(Ordering::Greater) => {
                return Classification::Decided(AxialDomainLocation::Outside);
            }
            Some(Ordering::Equal) => boundary = true,
            Some(Ordering::Less) => {}
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(if boundary {
        AxialDomainLocation::Boundary
    } else {
        AxialDomainLocation::Inside
    })
}

fn delta_component(point: &Point3, origin: &Point3, axis: Axis3) -> Real {
    match axis {
        Axis3::X => point.x() - origin.x(),
        Axis3::Y => point.y() - origin.y(),
        Axis3::Z => point.z() - origin.z(),
    }
}

fn signed_delta_component(point: &Point3, origin: &Point3, axis: SignedAxis3) -> Real {
    let delta = delta_component(point, origin, axis.axis());
    if axis.is_negative() { -delta } else { delta }
}

fn radial_axes(axis: Axis3) -> [Axis3; 2] {
    match axis {
        Axis3::X => [Axis3::Y, Axis3::Z],
        Axis3::Y => [Axis3::X, Axis3::Z],
        Axis3::Z => [Axis3::X, Axis3::Y],
    }
}

fn other_radial_axis(frame: &RetainedCylinderFrame3) -> Axis3 {
    radial_axes(frame.axis.axis())
        .into_iter()
        .find(|axis| *axis != frame.seam.axis())
        .expect("validated cylinder seam must be perpendicular to axis")
}
