//! Exact optical setup and report carriers.
//!
//! Optical simulation often crosses between exact geometry, empirical material
//! data, and lossy ray/field solvers. This module keeps the first boundary
//! exact: media, rays, planar interfaces, Beer-Lambert optical depth, and
//! normal-incidence Snell/Fresnel reports are expressed with [`Real`] values
//! and explicit status. Approximate optics adapters may propose samples, but
//! boundary classification and closed-form reports retain exact provenance or
//! return explicit uncertainty.
//!
//! Beer-Lambert attenuation uses optical depth, normal-incidence refraction
//! records Snell's law (`n_1 sin(theta_1) = n_2 sin(theta_2)`), and normal
//! reflectance uses Fresnel amplitude coefficients.

use hyperlattice::Vector3;
use hyperreal::{Real, RealSign};

use crate::{PhysicsError, PhysicsResult};

/// Status of an optical report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OpticalReportStatus {
    /// Report was computed exactly from exact inputs.
    Exact,
    /// A bounded solver certified the report.
    Certified,
    /// Classification or scalar sign was not decided.
    BoundedUnknown,
    /// Report came from a lossy adapter.
    Lossy,
}

/// Optical medium with exact scalar properties.
#[derive(Clone, Debug, PartialEq)]
pub struct OpticalMedium {
    /// Medium/source label.
    pub source: String,
    /// Refractive index.
    pub refractive_index: Real,
    /// Absorption coefficient used by Beer-Lambert slab reports.
    pub absorption_coefficient: Real,
}

/// Exact ray carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct OpticalRay3 {
    /// Ray origin.
    pub origin: Vector3,
    /// Ray direction. The direction need not be normalized for classification.
    pub direction: Vector3,
}

/// Planar optical interface carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct OpticalInterface3 {
    /// A point on the plane.
    pub point: Vector3,
    /// Plane normal. The normal need not be unit length.
    pub normal: Vector3,
    /// Medium on the normal-positive side.
    pub positive_medium: OpticalMedium,
    /// Medium on the normal-negative side.
    pub negative_medium: OpticalMedium,
}

/// Exact ray/interface relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RayInterfaceClassification {
    /// Ray starts on the interface.
    OnInterface,
    /// Ray crosses toward the interface plane.
    Incident,
    /// Ray points away from the interface plane.
    Receding,
    /// Ray direction is parallel to the interface plane.
    Parallel,
}

/// Normal-incidence Snell report.
#[derive(Clone, Debug, PartialEq)]
pub struct SnellNormalReport {
    /// Incident medium.
    pub incident: OpticalMedium,
    /// Transmitted medium.
    pub transmitted: OpticalMedium,
    /// Exact refractive ratio `n1 / n2`.
    pub refractive_ratio: Real,
    /// Exact sine invariant, zero for normal incidence.
    pub sine_invariant: Real,
    /// Report status.
    pub status: OpticalReportStatus,
}

/// Normal-incidence Fresnel reflectance report.
#[derive(Clone, Debug, PartialEq)]
pub struct FresnelNormalReport {
    /// Incident medium.
    pub incident: OpticalMedium,
    /// Transmitted medium.
    pub transmitted: OpticalMedium,
    /// Exact power reflectance `((n1 - n2) / (n1 + n2))^2`.
    pub reflectance: Real,
    /// Exact transmittance complement `1 - reflectance`.
    pub transmittance: Real,
    /// Report status.
    pub status: OpticalReportStatus,
}

/// Beer-Lambert slab attenuation report.
#[derive(Clone, Debug, PartialEq)]
pub struct BeerLambertSlabReport {
    /// Medium used by the slab.
    pub medium: OpticalMedium,
    /// Path length through the slab.
    pub thickness: Real,
    /// Exact optical depth `alpha * thickness`.
    pub optical_depth: Real,
    /// Symbolic expression label for transmittance `exp(-optical_depth)`.
    pub transmittance_expression: String,
    /// Report status.
    pub status: OpticalReportStatus,
}

impl BeerLambertSlabReport {
    /// Evaluates the exact-real Beer-Lambert transmittance `exp(-optical_depth)`.
    ///
    /// Optical depth remains the compact retained fact; the transcendental
    /// expression is built only for callers that need transmitted intensity.
    pub fn transmittance(&self) -> PhysicsResult<Real> {
        (-self.optical_depth.clone())
            .exp()
            .map_err(|_| PhysicsError::UnknownOpticalClassification)
    }
}

impl OpticalMedium {
    /// Creates an optical medium after validating positive `n` and non-negative absorption.
    pub fn new(
        source: impl Into<String>,
        refractive_index: Real,
        absorption_coefficient: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&refractive_index, PhysicsError::NonPositiveRefractiveIndex)?;
        require_nonnegative(
            &absorption_coefficient,
            PhysicsError::NegativeAbsorptionCoefficient,
        )?;
        Ok(Self {
            source: source.into(),
            refractive_index,
            absorption_coefficient,
        })
    }
}

impl OpticalInterface3 {
    /// Classifies a ray against this exact plane interface.
    ///
    /// The signs of `(origin - point) dot normal` and `direction dot normal`
    /// are exact sign queries. No primitive-float epsilon is used for topology.
    pub fn classify_ray(&self, ray: &OpticalRay3) -> PhysicsResult<RayInterfaceClassification> {
        let offset = &ray.origin - &self.point;
        let signed_distance = offset.dot(&self.normal);
        let direction_dot = ray.direction.dot(&self.normal);
        let distance_sign = signed_distance
            .refine_sign_until(-64)
            .ok_or(PhysicsError::UnknownOpticalClassification)?;
        let direction_sign = direction_dot
            .refine_sign_until(-64)
            .ok_or(PhysicsError::UnknownOpticalClassification)?;
        match (distance_sign, direction_sign) {
            (RealSign::Zero, _) => Ok(RayInterfaceClassification::OnInterface),
            (_, RealSign::Zero) => Ok(RayInterfaceClassification::Parallel),
            (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
                Ok(RayInterfaceClassification::Incident)
            }
            (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
                Ok(RayInterfaceClassification::Receding)
            }
        }
    }
}

impl SnellNormalReport {
    /// Reports normal-incidence Snell data for two known media.
    pub fn new(incident: OpticalMedium, transmitted: OpticalMedium) -> PhysicsResult<Self> {
        let refractive_ratio = div_real(&incident.refractive_index, &transmitted.refractive_index)?;
        Ok(Self {
            incident,
            transmitted,
            refractive_ratio,
            sine_invariant: Real::zero(),
            status: OpticalReportStatus::Exact,
        })
    }
}

impl FresnelNormalReport {
    /// Computes exact normal-incidence Fresnel reflectance.
    pub fn new(incident: OpticalMedium, transmitted: OpticalMedium) -> PhysicsResult<Self> {
        let numerator = incident.refractive_index.clone() - transmitted.refractive_index.clone();
        let denominator = incident.refractive_index.clone() + transmitted.refractive_index.clone();
        require_positive(&denominator, PhysicsError::NonPositiveRefractiveIndex)?;
        let amplitude = div_real(&numerator, &denominator)?;
        let reflectance = &amplitude * &amplitude;
        let transmittance = Real::one() - reflectance.clone();
        Ok(Self {
            incident,
            transmitted,
            reflectance,
            transmittance,
            status: OpticalReportStatus::Exact,
        })
    }
}

impl BeerLambertSlabReport {
    /// Computes exact Beer-Lambert optical depth through a slab.
    pub fn through_slab(medium: OpticalMedium, thickness: Real) -> PhysicsResult<Self> {
        require_positive(&thickness, PhysicsError::NonPositiveThickness)?;
        let optical_depth = &medium.absorption_coefficient * &thickness;
        Ok(Self {
            medium,
            thickness,
            transmittance_expression: "exp(-optical_depth)".into(),
            optical_depth,
            status: OpticalReportStatus::Exact,
        })
    }
}

fn require_positive(value: &Real, error: PhysicsError) -> PhysicsResult<()> {
    match value.refine_sign_until(-64) {
        Some(RealSign::Positive) => Ok(()),
        Some(RealSign::Negative | RealSign::Zero) | None => Err(error),
    }
}

fn require_nonnegative(value: &Real, error: PhysicsError) -> PhysicsResult<()> {
    match value.refine_sign_until(-64) {
        Some(RealSign::Positive | RealSign::Zero) => Ok(()),
        Some(RealSign::Negative) | None => Err(error),
    }
}

fn div_real(lhs: &Real, rhs: &Real) -> PhysicsResult<Real> {
    (lhs / rhs).map_err(|_| PhysicsError::NonPositiveRefractiveIndex)
}
