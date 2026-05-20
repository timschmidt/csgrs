//! Analytic SDF primitives with retained exact geometry.
//!
//! The primitive carriers keep their authored structure instead of lowering
//! immediately into a generic scalar expression. This follows Yap's EGC
//! guidance and lets `hypersdf` replay topology decisions through `hyperlimit`
//! predicates such as point-plane, point-sphere, and AABB/sphere classifiers.

use hyperlimit::{
    Aabb3PointLocation, Escalation, Plane3, PlaneSide, Point3, PredicateOutcome, RefinementNeed,
    SpherePointLocation, classify_point_aabb3, classify_point_plane, classify_point_sphere3,
    compare_reals,
};
use hyperreal::Real;
use std::cmp::Ordering;

use crate::expr::SdfCoordinate;
use crate::status::{SdfMetricStatus, SdfPointLocation};

/// Exact-friendly primitive node.
#[derive(Clone, Debug, PartialEq)]
pub enum SdfPrimitive {
    /// Oriented half-space whose inside is the plane's `Below` side.
    Plane { plane: Plane3 },
    /// Sphere or ball represented by center and squared radius.
    ///
    /// The squared radius keeps classification square-root-free, mirroring the
    /// exact squared-distance route used by `hyperlimit`.
    Sphere {
        center: Point3,
        radius_squared: Real,
    },
    /// Closed axis-aligned box.
    Aabb { min: Point3, max: Point3 },
    /// Finite axis-aligned cylinder.
    ///
    /// The retained sign-equivalent field is
    /// `max(radial_squared - radius_squared, abs(axis_delta) - half_height)`.
    /// This preserves the authored cylinder object and keeps classification
    /// square-root-free, following Yap's exact-geometric-computation principle
    /// of answering topology predicates over retained geometric packages.
    Cylinder {
        /// Cylinder axis.
        axis: SdfCoordinate,
        /// Center of the cylinder frame.
        center: Point3,
        /// Squared cylinder radius.
        radius_squared: Real,
        /// Half height along `axis`.
        half_height: Real,
    },
    /// Axis-aligned capsule or segment tube.
    ///
    /// The retained field is squared distance to the axis segment minus the
    /// squared radius. Like the sphere and cylinder routes, this avoids square
    /// roots in topology predicates and keeps the authored segment-tube object
    /// visible before scalar expansion, following Yap (1997).
    Capsule {
        /// Capsule segment axis.
        axis: SdfCoordinate,
        /// Center of the capsule segment.
        center: Point3,
        /// Squared capsule radius.
        radius_squared: Real,
        /// Half length of the central segment.
        half_length: Real,
    },
    /// Axis-aligned torus represented by its polynomial implicit equation.
    ///
    /// For radial squared distance `rho2`, axial delta `z`, major squared
    /// radius `R2`, and minor squared radius `r2`, the retained field is
    /// `(rho2 + z*z + R2 - r2)^2 - 4*R2*rho2`. This is the standard implicit
    /// torus equation in square-root-free form, kept as an exact object package
    /// before scalar expansion in the sense of Yap (1997).
    Torus {
        /// Torus symmetry axis.
        axis: SdfCoordinate,
        /// Center of the torus frame.
        center: Point3,
        /// Squared major radius. Must be strictly positive.
        major_radius_squared: Real,
        /// Squared minor radius. Must be nonnegative.
        minor_radius_squared: Real,
    },
    /// Finite slab around a plane, represented as `abs(plane(point)) - half_width`.
    ///
    /// This is a sign-equivalent implicit field for the closed region between
    /// two parallel planes. The half-width is retained and validated exactly,
    /// keeping the slab as a geometric object package rather than immediately
    /// lowering it into arithmetic nodes; see Yap, "Towards Exact Geometric
    /// Computation" (1997).
    Slab { plane: Plane3, half_width: Real },
}

impl SdfPrimitive {
    /// Construct an oriented plane/half-space primitive.
    pub const fn plane(plane: Plane3) -> Self {
        Self::Plane { plane }
    }

    /// Construct a sphere primitive from center and squared radius.
    pub const fn sphere(center: Point3, radius_squared: Real) -> Self {
        Self::Sphere {
            center,
            radius_squared,
        }
    }

    /// Construct a closed axis-aligned box primitive.
    pub const fn aabb(min: Point3, max: Point3) -> Self {
        Self::Aabb { min, max }
    }

    /// Construct a finite axis-aligned cylinder.
    pub const fn cylinder(
        axis: SdfCoordinate,
        center: Point3,
        radius_squared: Real,
        half_height: Real,
    ) -> Self {
        Self::Cylinder {
            axis,
            center,
            radius_squared,
            half_height,
        }
    }

    /// Construct an axis-aligned capsule or segment tube.
    pub const fn capsule(
        axis: SdfCoordinate,
        center: Point3,
        radius_squared: Real,
        half_length: Real,
    ) -> Self {
        Self::Capsule {
            axis,
            center,
            radius_squared,
            half_length,
        }
    }

    /// Construct an axis-aligned torus from squared radii.
    pub const fn torus(
        axis: SdfCoordinate,
        center: Point3,
        major_radius_squared: Real,
        minor_radius_squared: Real,
    ) -> Self {
        Self::Torus {
            axis,
            center,
            major_radius_squared,
            minor_radius_squared,
        }
    }

    /// Construct a finite slab around an oriented plane.
    pub const fn slab(plane: Plane3, half_width: Real) -> Self {
        Self::Slab { plane, half_width }
    }

    /// Return the primitive's metric claim.
    pub const fn metric_status(&self) -> SdfMetricStatus {
        match self {
            Self::Plane { .. } => SdfMetricStatus::SignEquivalent,
            Self::Sphere { .. } => SdfMetricStatus::SignEquivalent,
            Self::Aabb { .. } => SdfMetricStatus::SignEquivalent,
            Self::Cylinder { .. } => SdfMetricStatus::SignEquivalent,
            Self::Capsule { .. } => SdfMetricStatus::SignEquivalent,
            Self::Torus { .. } => SdfMetricStatus::SignEquivalent,
            Self::Slab { .. } => SdfMetricStatus::SignEquivalent,
        }
    }

    /// Compute a retained scalar or sign-equivalent value when cheap.
    ///
    /// Sphere values are squared-distance-minus-radius-squared, so they are
    /// sign-equivalent rather than metric distances. This deliberately avoids
    /// square-root construction in topology paths.
    pub fn scalar_value(&self, point: &Point3) -> Option<Real> {
        match self {
            Self::Plane { plane } => Some(plane_value(plane, point)),
            Self::Sphere {
                center,
                radius_squared,
            } => {
                if !radius_squared_domain(radius_squared)
                    .value()
                    .unwrap_or(false)
                {
                    return None;
                }
                Some(&squared_distance3(center, point) - radius_squared)
            }
            Self::Aabb { min, max } => aabb_value(min, max, point),
            Self::Cylinder {
                axis,
                center,
                radius_squared,
                half_height,
            } => cylinder_value(*axis, center, radius_squared, half_height, point),
            Self::Capsule {
                axis,
                center,
                radius_squared,
                half_length,
            } => capsule_value(*axis, center, radius_squared, half_length, point),
            Self::Torus {
                axis,
                center,
                major_radius_squared,
                minor_radius_squared,
            } => torus_value(
                *axis,
                center,
                major_radius_squared,
                minor_radius_squared,
                point,
            ),
            Self::Slab { plane, half_width } => slab_value(plane, half_width, point),
        }
    }

    /// Classify a point using Hyperlimit predicates.
    pub fn classify_point(&self, point: &Point3) -> hyperlimit::PredicateOutcome<SdfPointLocation> {
        match self {
            Self::Plane { plane } => map_plane_point(classify_point_plane(point, plane)),
            Self::Sphere {
                center,
                radius_squared,
            } => match radius_squared_domain(radius_squared) {
                PredicateOutcome::Decided { value: true, .. } => {
                    map_sphere_point(classify_point_sphere3(center, radius_squared, point))
                }
                PredicateOutcome::Decided { value: false, .. } => {
                    PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            },
            Self::Aabb { min, max } => map_aabb_point(classify_point_aabb3(min, max, point)),
            Self::Cylinder { .. } => classify_scalar_primitive_value(self.scalar_value(point)),
            Self::Capsule { .. } => classify_scalar_primitive_value(self.scalar_value(point)),
            Self::Torus { .. } => classify_scalar_primitive_value(self.scalar_value(point)),
            Self::Slab { .. } => classify_scalar_primitive_value(self.scalar_value(point)),
        }
    }
}

/// Certify whether a squared radius is nonnegative.
pub(crate) fn radius_squared_domain(radius_squared: &Real) -> PredicateOutcome<bool> {
    nonnegative_domain(radius_squared)
}

/// Certify whether a slab half-width is nonnegative.
pub(crate) fn half_width_domain(half_width: &Real) -> PredicateOutcome<bool> {
    nonnegative_domain(half_width)
}

/// Certify whether both cylinder radial and axial domain parameters are valid.
pub(crate) fn cylinder_domain(radius_squared: &Real, half_height: &Real) -> PredicateOutcome<bool> {
    match (
        radius_squared_domain(radius_squared),
        half_width_domain(half_height),
    ) {
        (
            PredicateOutcome::Decided {
                value: radius_ok,
                certainty,
                stage,
            },
            PredicateOutcome::Decided {
                value: height_ok, ..
            },
        ) => PredicateOutcome::decided(radius_ok && height_ok, certainty, stage),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

/// Certify whether both capsule radius and half-length parameters are valid.
pub(crate) fn capsule_domain(radius_squared: &Real, half_length: &Real) -> PredicateOutcome<bool> {
    cylinder_domain(radius_squared, half_length)
}

/// Certify whether torus squared radii are in-domain.
pub(crate) fn torus_domain(
    major_radius_squared: &Real,
    minor_radius_squared: &Real,
) -> PredicateOutcome<bool> {
    match (
        positive_domain(major_radius_squared),
        radius_squared_domain(minor_radius_squared),
    ) {
        (
            PredicateOutcome::Decided {
                value: major_ok,
                certainty,
                stage,
            },
            PredicateOutcome::Decided {
                value: minor_ok, ..
            },
        ) => PredicateOutcome::decided(major_ok && minor_ok, certainty, stage),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn nonnegative_domain(value: &Real) -> PredicateOutcome<bool> {
    match compare_reals(value, &Real::zero()) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value != Ordering::Less, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn positive_domain(value: &Real) -> PredicateOutcome<bool> {
    match compare_reals(value, &Real::zero()) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value == Ordering::Greater, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn classify_scalar_primitive_value(value: Option<Real>) -> PredicateOutcome<SdfPointLocation> {
    let Some(value) = value else {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    };
    match compare_reals(&value, &Real::zero()) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                Ordering::Less => SdfPointLocation::Inside,
                Ordering::Equal => SdfPointLocation::Boundary,
                Ordering::Greater => SdfPointLocation::Outside,
            };
            PredicateOutcome::decided(location, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn map_plane_point(
    outcome: hyperlimit::PredicateOutcome<PlaneSide>,
) -> hyperlimit::PredicateOutcome<SdfPointLocation> {
    match outcome {
        hyperlimit::PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                PlaneSide::Below => SdfPointLocation::Inside,
                PlaneSide::On => SdfPointLocation::Boundary,
                PlaneSide::Above => SdfPointLocation::Outside,
            };
            hyperlimit::PredicateOutcome::decided(location, certainty, stage)
        }
        hyperlimit::PredicateOutcome::Unknown { needed, stage } => {
            hyperlimit::PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn map_sphere_point(
    outcome: hyperlimit::PredicateOutcome<SpherePointLocation>,
) -> hyperlimit::PredicateOutcome<SdfPointLocation> {
    match outcome {
        hyperlimit::PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                SpherePointLocation::Inside => SdfPointLocation::Inside,
                SpherePointLocation::On => SdfPointLocation::Boundary,
                SpherePointLocation::Outside => SdfPointLocation::Outside,
            };
            hyperlimit::PredicateOutcome::decided(location, certainty, stage)
        }
        hyperlimit::PredicateOutcome::Unknown { needed, stage } => {
            hyperlimit::PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn map_aabb_point(
    outcome: hyperlimit::PredicateOutcome<Aabb3PointLocation>,
) -> hyperlimit::PredicateOutcome<SdfPointLocation> {
    match outcome {
        hyperlimit::PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                Aabb3PointLocation::Inside => SdfPointLocation::Inside,
                Aabb3PointLocation::Boundary => SdfPointLocation::Boundary,
                Aabb3PointLocation::Outside => SdfPointLocation::Outside,
            };
            hyperlimit::PredicateOutcome::decided(location, certainty, stage)
        }
        hyperlimit::PredicateOutcome::Unknown { needed, stage } => {
            hyperlimit::PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn plane_value(plane: &Plane3, point: &Point3) -> Real {
    let nx = &plane.normal.x * &point.x;
    let ny = &plane.normal.y * &point.y;
    let nz = &plane.normal.z * &point.z;
    &(&(&nx + &ny) + &nz) + &plane.offset
}

fn aabb_value(min: &Point3, max: &Point3, point: &Point3) -> Option<Real> {
    let candidates = [
        &min.x - &point.x,
        &point.x - &max.x,
        &min.y - &point.y,
        &point.y - &max.y,
        &min.z - &point.z,
        &point.z - &max.z,
    ];
    let mut value = candidates[0].clone();
    for candidate in candidates.iter().skip(1) {
        match compare_reals(&value, candidate) {
            PredicateOutcome::Decided {
                value: Ordering::Less,
                ..
            } => value = candidate.clone(),
            PredicateOutcome::Decided { .. } => {}
            PredicateOutcome::Unknown { .. } => return None,
        }
    }
    Some(value)
}

fn slab_value(plane: &Plane3, half_width: &Real, point: &Point3) -> Option<Real> {
    if !half_width_domain(half_width).value().unwrap_or(false) {
        return None;
    }
    let value = plane_value(plane, point);
    match compare_reals(&value, &Real::zero()) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Some(-&value - half_width),
        PredicateOutcome::Decided { .. } => Some(&value - half_width),
        PredicateOutcome::Unknown { .. } => None,
    }
}

fn cylinder_value(
    axis: SdfCoordinate,
    center: &Point3,
    radius_squared: &Real,
    half_height: &Real,
    point: &Point3,
) -> Option<Real> {
    if !cylinder_domain(radius_squared, half_height)
        .value()
        .unwrap_or(false)
    {
        return None;
    }
    let radial = &radial_squared(axis, center, point) - radius_squared;
    let axis_delta = axis_value(axis, point) - axis_value(axis, center);
    let axial = match compare_reals(&axis_delta, &Real::zero()) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => -&axis_delta - half_height,
        PredicateOutcome::Decided { .. } => &axis_delta - half_height,
        PredicateOutcome::Unknown { .. } => return None,
    };
    match compare_reals(&radial, &axial) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Some(axial),
        PredicateOutcome::Decided { .. } => Some(radial),
        PredicateOutcome::Unknown { .. } => None,
    }
}

fn capsule_value(
    axis: SdfCoordinate,
    center: &Point3,
    radius_squared: &Real,
    half_length: &Real,
    point: &Point3,
) -> Option<Real> {
    if !capsule_domain(radius_squared, half_length)
        .value()
        .unwrap_or(false)
    {
        return None;
    }
    let radial = radial_squared(axis, center, point);
    let axis_delta = axis_value(axis, point) - axis_value(axis, center);
    let zero = Real::zero();
    let outside_axis = match compare_reals(&axis_delta, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => {
            let positive_delta = -&axis_delta;
            match compare_reals(&positive_delta, half_length) {
                PredicateOutcome::Decided {
                    value: Ordering::Greater,
                    ..
                } => &positive_delta - half_length,
                PredicateOutcome::Decided { .. } => zero,
                PredicateOutcome::Unknown { .. } => return None,
            }
        }
        PredicateOutcome::Decided { .. } => match compare_reals(&axis_delta, half_length) {
            PredicateOutcome::Decided {
                value: Ordering::Greater,
                ..
            } => &axis_delta - half_length,
            PredicateOutcome::Decided { .. } => zero,
            PredicateOutcome::Unknown { .. } => return None,
        },
        PredicateOutcome::Unknown { .. } => return None,
    };
    Some(&(&radial + &(&outside_axis * &outside_axis)) - radius_squared)
}

fn torus_value(
    axis: SdfCoordinate,
    center: &Point3,
    major_radius_squared: &Real,
    minor_radius_squared: &Real,
    point: &Point3,
) -> Option<Real> {
    if !torus_domain(major_radius_squared, minor_radius_squared)
        .value()
        .unwrap_or(false)
    {
        return None;
    }
    let radial = radial_squared(axis, center, point);
    let axis_delta = axis_value(axis, point) - axis_value(axis, center);
    let axial_squared = &axis_delta * &axis_delta;
    let q = &(&(&radial + &axial_squared) + major_radius_squared) - minor_radius_squared;
    let q_squared = &q * &q;
    let four = Real::from(4_i32);
    let radial_term = &(&four * major_radius_squared) * &radial;
    Some(&q_squared - &radial_term)
}

pub(crate) fn radial_squared(axis: SdfCoordinate, center: &Point3, point: &Point3) -> Real {
    let (a0, a1, b0, b1) = match axis {
        SdfCoordinate::X => (&center.y, &center.z, &point.y, &point.z),
        SdfCoordinate::Y => (&center.x, &center.z, &point.x, &point.z),
        SdfCoordinate::Z => (&center.x, &center.y, &point.x, &point.y),
    };
    let d0 = a0 - b0;
    let d1 = a1 - b1;
    &(&d0 * &d0) + &(&d1 * &d1)
}

fn axis_value(axis: SdfCoordinate, point: &Point3) -> &Real {
    match axis {
        SdfCoordinate::X => &point.x,
        SdfCoordinate::Y => &point.y,
        SdfCoordinate::Z => &point.z,
    }
}

pub(crate) fn squared_distance3(a: &Point3, b: &Point3) -> Real {
    let dx = &a.x - &b.x;
    let dy = &a.y - &b.y;
    let dz = &a.z - &b.z;
    let dx2 = &dx * &dx;
    let dy2 = &dy * &dy;
    let dz2 = &dz * &dz;
    &(&dx2 + &dy2) + &dz2
}
