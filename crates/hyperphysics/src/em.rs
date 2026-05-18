//! Exact electromagnetic setup and constitutive report carriers.
//!
//! Full-wave electromagnetic solvers routinely combine exact CAD geometry,
//! stackup/material tables, meshing heuristics, and lossy floating-point field
//! updates. This module keeps the Hyper boundary exact: materials, field
//! regions, boundary conditions, and simple linear isotropic constitutive
//! reports use [`Real`] values and explicit status before any FDTD/FEM/BEM/MoM
//! adapter is trusted. That follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): topological and material
//! facts should be retained exactly, while approximate solver outputs remain
//! proposals until certified or replayed.
//!
//! The implemented fixture is the local linear isotropic constitutive boundary:
//! `D = epsilon E` and `J = sigma E`. These are the standard macroscopic
//! Maxwell-material relation and Ohmic conduction relation; see Maxwell,
//! "A Dynamical Theory of the Electromagnetic Field" (1865), and Stratton,
//! *Electromagnetic Theory* (1941). The report records the material and field
//! assumptions instead of folding them into an opaque mesh-solver input.

use hyperlattice::Vector3;
use hyperreal::{Real, RealSign};

use crate::{AxisAlignedBox3, PhysicsError, PhysicsResult};

/// Status of an electromagnetic setup or report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ElectromagneticReportStatus {
    /// Computed exactly from exact inputs.
    Exact,
    /// A bounded solver certified the value.
    Certified,
    /// Solver or comparison did not decide.
    BoundedUnknown,
    /// Value came from a lossy adapter.
    Lossy,
}

/// Electromagnetic solver or model regime.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ElectromagneticRegime {
    /// Electrostatic field setup.
    Electrostatic,
    /// Magnetostatic field setup.
    Magnetostatic,
    /// Quasi-static coupled electric/magnetic setup.
    QuasiStatic,
    /// Frequency-domain wave setup.
    FrequencyDomain,
    /// Finite-difference time-domain adapter boundary.
    Fdtd,
    /// Finite-element adapter boundary.
    Fem,
    /// Boundary-element adapter boundary.
    Bem,
    /// Method-of-moments adapter boundary.
    Mom,
    /// External black-box adapter boundary.
    External(String),
}

/// Region role for EM field setup.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FieldRegionKind {
    /// Dielectric region.
    Dielectric,
    /// Conductive region.
    Conductor,
    /// Magnetic material region.
    Magnetic,
    /// Source region.
    Source,
    /// Boundary-only region.
    Boundary,
    /// Source-specific region class.
    Custom(String),
}

/// Boundary condition role for EM adapters.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BoundaryConditionKind {
    /// Perfect electric conductor boundary.
    PerfectElectricConductor,
    /// Perfect magnetic conductor boundary.
    PerfectMagneticConductor,
    /// Dirichlet electric-potential or field value.
    Dirichlet,
    /// Neumann flux/normal derivative value.
    Neumann,
    /// Impedance boundary condition.
    Impedance,
    /// Radiation/open boundary condition.
    Radiation,
    /// Periodic boundary condition.
    Periodic,
    /// Source-specific boundary class.
    Custom(String),
}

/// Exact scalar electromagnetic material data.
#[derive(Clone, Debug, PartialEq)]
pub struct ElectromagneticMaterial {
    /// Provenance label or material handle.
    pub source: String,
    /// Permittivity used by `D = epsilon E`.
    pub permittivity: Real,
    /// Permeability used by magnetic-field adapters.
    pub permeability: Real,
    /// Electrical conductivity used by `J = sigma E`.
    pub conductivity: Real,
    /// Status of this material data.
    pub status: ElectromagneticReportStatus,
}

/// Exact EM boundary-condition carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct ElectromagneticBoundaryCondition3 {
    /// Boundary handle.
    pub handle: String,
    /// Boundary role.
    pub kind: BoundaryConditionKind,
    /// Optional scalar value, such as potential, flux, or impedance.
    pub value: Option<Real>,
    /// Boundary normal when known.
    pub normal: Option<Vector3>,
    /// Status of the boundary value.
    pub status: ElectromagneticReportStatus,
}

/// Exact EM field-region setup over retained bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct ElectromagneticFieldRegion3 {
    /// Region handle.
    pub handle: String,
    /// Region role.
    pub kind: FieldRegionKind,
    /// Region material.
    pub material: ElectromagneticMaterial,
    /// Exact bounding box used for broad setup, meshing, or adapter export.
    pub bounds: AxisAlignedBox3,
    /// Declared model/solver regime.
    pub regime: ElectromagneticRegime,
    /// Boundary conditions attached to this region.
    pub boundary_conditions: Vec<ElectromagneticBoundaryCondition3>,
}

/// Linear isotropic electric constitutive report.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearIsotropicElectricReport3 {
    /// Material used by the report.
    pub material: ElectromagneticMaterial,
    /// Exact electric field input.
    pub electric_field: Vector3,
    /// Exact displacement field `D = epsilon E`.
    pub displacement_field: Vector3,
    /// Exact conduction-current density `J = sigma E`.
    pub conduction_current_density: Vector3,
    /// Assumption text.
    pub assumption: String,
    /// Report status.
    pub status: ElectromagneticReportStatus,
}

impl ElectromagneticMaterial {
    /// Creates exact isotropic EM material data after validating physical domains.
    pub fn new(
        source: impl Into<String>,
        permittivity: Real,
        permeability: Real,
        conductivity: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&permittivity, PhysicsError::NonPositivePermittivity)?;
        require_positive(&permeability, PhysicsError::NonPositivePermeability)?;
        require_nonnegative(&conductivity, PhysicsError::NegativeElectricalConductivity)?;
        Ok(Self {
            source: source.into(),
            permittivity,
            permeability,
            conductivity,
            status: ElectromagneticReportStatus::Exact,
        })
    }

    /// Reports exact linear isotropic `D = epsilon E` and `J = sigma E`.
    pub fn linear_isotropic_electric_response(
        &self,
        electric_field: Vector3,
    ) -> LinearIsotropicElectricReport3 {
        let displacement_field = electric_field.clone() * &self.permittivity;
        let conduction_current_density = electric_field.clone() * &self.conductivity;
        LinearIsotropicElectricReport3 {
            material: self.clone(),
            electric_field,
            displacement_field,
            conduction_current_density,
            assumption: "linear isotropic macroscopic EM material: D = epsilon E, J = sigma E"
                .into(),
            status: ElectromagneticReportStatus::Exact,
        }
    }
}

impl ElectromagneticFieldRegion3 {
    /// Creates an EM field region with exact bounds and explicit solver regime.
    pub fn new(
        handle: impl Into<String>,
        kind: FieldRegionKind,
        material: ElectromagneticMaterial,
        bounds: AxisAlignedBox3,
        regime: ElectromagneticRegime,
    ) -> Self {
        Self {
            handle: handle.into(),
            kind,
            material,
            bounds,
            regime,
            boundary_conditions: Vec::new(),
        }
    }

    /// Adds a boundary condition to the region.
    pub fn with_boundary_condition(
        mut self,
        boundary_condition: ElectromagneticBoundaryCondition3,
    ) -> Self {
        self.boundary_conditions.push(boundary_condition);
        self
    }
}

impl ElectromagneticBoundaryCondition3 {
    /// Creates an exact or proposal EM boundary condition.
    pub fn new(
        handle: impl Into<String>,
        kind: BoundaryConditionKind,
        value: Option<Real>,
        normal: Option<Vector3>,
        status: ElectromagneticReportStatus,
    ) -> Self {
        Self {
            handle: handle.into(),
            kind,
            value,
            normal,
            status,
        }
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
