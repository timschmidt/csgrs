//! Photochemical and fabrication process report surfaces.
//!
//! The first fabrication model is the vat-photopolymer working curve used in
//! stereolithography process planning. It keeps exposure, critical exposure,
//! penetration depth, layer thickness, and chemistry provenance as exact
//! [`Real`] values. Decisions are certified from exact signs; no primitive
//! float threshold is used to decide whether a layer clears. This follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>): process adapters may
//! be approximate, but acceptance decisions should replay through exact data or
//! return explicit uncertainty.
//!
//! The working-curve formula `C_d = D_p ln(E / E_c)` is the standard SLA cure
//! depth model described by Jacobs, "Rapid Prototyping & Manufacturing:
//! Fundamentals of Stereolithography," 1992. This module computes that
//! expression symbolically through `hyperreal::Real::ln` and records the
//! exposure ratio and cure-depth expression in the report.
//!
//! Reaction-diffusion carriers use exact concentration, conversion, and
//! transport coefficients while leaving full state evolution to a certified
//! solver surface. Diffusive Courant diagnostics use the Fickian form
//! `D dt / h^2`; see Fick, "Ueber Diffusion," *Annalen der Physik* 94, 1855.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::{PhysicsError, PhysicsResult};

/// Exposure policy for a photochemical process.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExposureMode {
    /// One-photon exposure.
    OnePhoton,
    /// Two-photon exposure.
    TwoPhoton,
    /// Multiphoton exposure.
    Multiphoton,
    /// CLIP/dead-zone exposure policy.
    ClipDeadZone,
    /// Computed axial or volumetric lithography policy.
    ComputedAxialVolumetric,
}

/// Photochemical solver/proposal policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PhotochemicalPolicy {
    /// Closed-form working-curve replay.
    WorkingCurveReplay,
    /// Reaction-diffusion adapter.
    ReactionDiffusionAdapter,
    /// Tomographic optimization adapter.
    TomographicOptimizationAdapter,
    /// Resin-flow/thermal-stress/post-cure coupled adapter.
    CoupledPostProcessAdapter,
}

/// Exact concentration inputs for a photopolymer system.
#[derive(Clone, Debug, PartialEq)]
pub struct PhotochemicalConcentrations {
    /// Absorber concentration.
    pub absorber: Real,
    /// Initiator concentration.
    pub initiator: Real,
    /// Oxygen concentration.
    pub oxygen: Real,
    /// Inhibitor or quencher concentration.
    pub inhibitor: Real,
}

/// Exact reaction-diffusion material/process state.
#[derive(Clone, Debug, PartialEq)]
pub struct ReactionDiffusionState {
    /// Source or process-state label.
    pub source: String,
    /// Exact concentration metadata.
    pub concentrations: PhotochemicalConcentrations,
    /// Monomer conversion fraction in `[0, 1]`.
    pub monomer_conversion: Real,
    /// Radical concentration.
    pub radical_concentration: Real,
    /// Local heat generation or integrated heat-source strength.
    pub heat_generation: Real,
    /// Shrinkage fraction in `[0, 1]`.
    pub shrinkage: Real,
    /// Gel threshold fraction in `[0, 1]`.
    pub gel_threshold: Real,
}

/// Exact Fickian transport coefficients for photochemical species.
#[derive(Clone, Debug, PartialEq)]
pub struct ReactionDiffusionTransport {
    /// Absorber diffusion coefficient.
    pub absorber_diffusion: Real,
    /// Initiator diffusion coefficient.
    pub initiator_diffusion: Real,
    /// Oxygen diffusion coefficient.
    pub oxygen_diffusion: Real,
    /// Inhibitor/quencher diffusion coefficient.
    pub inhibitor_diffusion: Real,
}

/// Exact explicit-grid diffusion diagnostic.
#[derive(Clone, Debug, PartialEq)]
pub struct DiffusiveCourantReport {
    /// Transport coefficients used by the diagnostic.
    pub transport: ReactionDiffusionTransport,
    /// Time step.
    pub time_step: Real,
    /// Grid spacing.
    pub grid_spacing: Real,
    /// Exact absorber `D dt / h^2`.
    pub absorber: Real,
    /// Exact initiator `D dt / h^2`.
    pub initiator: Real,
    /// Exact oxygen `D dt / h^2`.
    pub oxygen: Real,
    /// Exact inhibitor/quencher `D dt / h^2`.
    pub inhibitor: Real,
    /// Human-readable diagnostic provenance.
    pub expression: String,
    /// Policy associated with this diagnostic.
    pub policy: PhotochemicalPolicy,
}

/// Vat-photopolymer working-curve setup.
#[derive(Clone, Debug, PartialEq)]
pub struct VatPhotopolymerWorkingCurve {
    /// Source or resin calibration label.
    pub source: String,
    /// Exposure `E`.
    pub exposure: Real,
    /// Critical exposure `E_c`.
    pub critical_exposure: Real,
    /// Penetration depth `D_p`.
    pub penetration_depth: Real,
    /// Target layer thickness.
    pub layer_thickness: Real,
    /// Exact concentration metadata.
    pub concentrations: PhotochemicalConcentrations,
    /// Exposure mode.
    pub mode: ExposureMode,
}

/// Cure status for a layer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CureStatus {
    /// Cure depth is certified to meet or exceed layer thickness.
    ClearsLayer,
    /// Cure depth is certified below layer thickness.
    UnderCured,
    /// Cure depth or comparison was not certified.
    Unknown,
}

/// Decision payload for cure depth.
#[derive(Clone, Debug, PartialEq)]
pub struct CureDecision {
    /// Certified status.
    pub status: CureStatus,
    /// Exact margin `cure_depth - layer_thickness` when available.
    pub margin: Option<Real>,
}

/// Working-curve report.
#[derive(Clone, Debug, PartialEq)]
pub struct WorkingCurveReport {
    /// Setup used by the report.
    pub setup: VatPhotopolymerWorkingCurve,
    /// Exact exposure ratio `E / E_c`.
    pub exposure_ratio: Real,
    /// Exact cure depth `D_p ln(E / E_c)`.
    pub cure_depth: Real,
    /// Decision against target layer thickness.
    pub decision: CureDecision,
    /// Human-readable expression provenance.
    pub expression: String,
    /// Policy used to produce the report.
    pub policy: PhotochemicalPolicy,
}

impl PhotochemicalConcentrations {
    /// Creates exact concentration metadata after rejecting negative values.
    pub fn new(
        absorber: Real,
        initiator: Real,
        oxygen: Real,
        inhibitor: Real,
    ) -> PhysicsResult<Self> {
        for value in [&absorber, &initiator, &oxygen, &inhibitor] {
            require_nonnegative(value, PhysicsError::NegativeConcentration)?;
        }
        Ok(Self {
            absorber,
            initiator,
            oxygen,
            inhibitor,
        })
    }
}

impl ReactionDiffusionState {
    /// Creates exact reaction-diffusion state metadata.
    pub fn new(
        source: impl Into<String>,
        concentrations: PhotochemicalConcentrations,
        monomer_conversion: Real,
        radical_concentration: Real,
        heat_generation: Real,
        shrinkage: Real,
        gel_threshold: Real,
    ) -> PhysicsResult<Self> {
        require_fraction(&monomer_conversion)?;
        require_nonnegative(&radical_concentration, PhysicsError::NegativeConcentration)?;
        require_nonnegative(&heat_generation, PhysicsError::NegativeConcentration)?;
        require_fraction(&shrinkage)?;
        require_fraction(&gel_threshold)?;
        Ok(Self {
            source: source.into(),
            concentrations,
            monomer_conversion,
            radical_concentration,
            heat_generation,
            shrinkage,
            gel_threshold,
        })
    }
}

impl ReactionDiffusionTransport {
    /// Creates exact non-negative diffusion coefficients for process species.
    pub fn new(
        absorber_diffusion: Real,
        initiator_diffusion: Real,
        oxygen_diffusion: Real,
        inhibitor_diffusion: Real,
    ) -> PhysicsResult<Self> {
        for value in [
            &absorber_diffusion,
            &initiator_diffusion,
            &oxygen_diffusion,
            &inhibitor_diffusion,
        ] {
            require_nonnegative(value, PhysicsError::NegativeDiffusionCoefficient)?;
        }
        Ok(Self {
            absorber_diffusion,
            initiator_diffusion,
            oxygen_diffusion,
            inhibitor_diffusion,
        })
    }

    /// Reports exact Fickian `D dt / h^2` diffusion numbers without advancing state.
    pub fn diffusive_courant_report(
        &self,
        time_step: Real,
        grid_spacing: Real,
    ) -> PhysicsResult<DiffusiveCourantReport> {
        require_positive(&time_step, PhysicsError::NonPositiveTimeStep)?;
        require_positive(&grid_spacing, PhysicsError::NonPositiveGridSpacing)?;
        let h_squared = &grid_spacing * &grid_spacing;
        let scale = div_real(&time_step, &h_squared)?;
        Ok(DiffusiveCourantReport {
            transport: self.clone(),
            time_step,
            grid_spacing,
            absorber: &self.absorber_diffusion * &scale,
            initiator: &self.initiator_diffusion * &scale,
            oxygen: &self.oxygen_diffusion * &scale,
            inhibitor: &self.inhibitor_diffusion * &scale,
            expression: "D dt / h^2".into(),
            policy: PhotochemicalPolicy::ReactionDiffusionAdapter,
        })
    }
}

impl VatPhotopolymerWorkingCurve {
    /// Creates a working-curve setup after validating positive scalar inputs.
    pub fn new(
        source: impl Into<String>,
        exposure: Real,
        critical_exposure: Real,
        penetration_depth: Real,
        layer_thickness: Real,
        concentrations: PhotochemicalConcentrations,
        mode: ExposureMode,
    ) -> PhysicsResult<Self> {
        require_positive(&exposure, PhysicsError::NonPositiveExposure)?;
        require_positive(&critical_exposure, PhysicsError::NonPositiveExposure)?;
        require_positive(
            &penetration_depth,
            PhysicsError::NonPositivePenetrationDepth,
        )?;
        require_positive(&layer_thickness, PhysicsError::NonPositiveLayerThickness)?;
        Ok(Self {
            source: source.into(),
            exposure,
            critical_exposure,
            penetration_depth,
            layer_thickness,
            concentrations,
            mode,
        })
    }

    /// Replays the exact Jacobs working-curve formula.
    pub fn replay(&self) -> PhysicsResult<WorkingCurveReport> {
        let exposure_ratio = div_real(&self.exposure, &self.critical_exposure)?;
        let cure_depth = &self.penetration_depth
            * &exposure_ratio
                .clone()
                .ln()
                .map_err(|_| PhysicsError::NonPositiveExposure)?;
        let margin = cure_depth.clone() - self.layer_thickness.clone();
        let status = match margin.partial_cmp(&Real::zero()) {
            Some(Ordering::Less) => CureStatus::UnderCured,
            Some(Ordering::Equal | Ordering::Greater) => CureStatus::ClearsLayer,
            None => CureStatus::Unknown,
        };
        let margin = if status == CureStatus::Unknown {
            None
        } else {
            Some(margin)
        };
        Ok(WorkingCurveReport {
            setup: self.clone(),
            exposure_ratio,
            cure_depth,
            decision: CureDecision { status, margin },
            expression: "C_d = D_p ln(E / E_c)".into(),
            policy: PhotochemicalPolicy::WorkingCurveReplay,
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

fn require_fraction(value: &Real) -> PhysicsResult<()> {
    match (
        value.partial_cmp(&Real::zero()),
        value.partial_cmp(&Real::one()),
    ) {
        (Some(Ordering::Greater | Ordering::Equal), Some(Ordering::Less | Ordering::Equal)) => {
            Ok(())
        }
        _ => Err(PhysicsError::InvalidFraction),
    }
}

fn div_real(lhs: &Real, rhs: &Real) -> PhysicsResult<Real> {
    (lhs / rhs).map_err(|_| PhysicsError::NonPositiveExposure)
}
