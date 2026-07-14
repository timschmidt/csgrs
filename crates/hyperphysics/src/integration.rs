//! Exact/replayable integration and co-simulation reports.
//!
//! This module does not implement a production dynamics solver. It defines the
//! report surface needed before runtime adapters are trusted: exact force
//! accumulation, explicit step proposals, and energy/momentum diagnostics. An
//! external or lossy integrator can propose a state, but exact quantities must
//! replay through exact data or remain explicitly unknown.
//!
//! Symplectic, variational, and complementarity policies are represented by
//! name before their corresponding exact kernels and contact solvers exist.

use hyperlattice::Vector3;
use hyperreal::{Real, RealSign};

use crate::{PhysicsError, PhysicsResult};

/// Solver or co-simulation policy used to generate a step proposal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum IntegrationPolicy {
    /// Exact force accumulation plus explicit Euler proposal.
    ExplicitEulerReplay,
    /// Symplectic or variational integrator proposal.
    SymplecticVariational,
    /// Implicit Euler proposal.
    ImplicitEuler,
    /// BDF-family proposal.
    BackwardDifferentiation,
    /// Complementarity/contact impulse proposal.
    Complementarity,
    /// IDA/SUNDIALS-style DAE adapter proposal.
    IdaDaeAdapter,
}

/// Cross-domain coupling policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CouplingPolicy {
    /// Thermal field coupling.
    Heat,
    /// Reaction-diffusion coupling.
    ReactionDiffusion,
    /// Photon transport coupling.
    PhotonTransport,
    /// Full-wave EM coupling.
    FullWaveEm,
    /// Modal or eigenvalue coupling.
    ModalEigen,
    /// Fluid-structure interaction coupling.
    FluidStructure,
    /// Field/circuit coupling with `hypercircuit`.
    FieldCircuit,
}

/// Status of a diagnostic quantity in a step report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DiagnosticStatus {
    /// Quantity was computed exactly from exact state.
    Exact,
    /// Quantity was certified by a bounded replay.
    Certified,
    /// Replay did not decide.
    BoundedUnknown,
    /// Quantity came from a lossy adapter.
    Lossy,
}

/// One exact force contribution.
#[derive(Clone, Debug, PartialEq)]
pub struct ForceContribution3 {
    /// Provenance label for this force.
    pub source: String,
    /// Exact force vector.
    pub force: Vector3,
}

/// Exact force accumulator.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ForceAccumulator3 {
    contributions: Vec<ForceContribution3>,
}

/// Energy and momentum diagnostics for a step.
#[derive(Clone, Debug, PartialEq)]
pub struct SystemDiagnostics3 {
    /// Total linear momentum.
    pub momentum: Vector3,
    /// Translational kinetic energy.
    pub kinetic_energy: Real,
    /// Diagnostic status.
    pub status: DiagnosticStatus,
}

/// Replay report for one proposed integration step.
#[derive(Clone, Debug, PartialEq)]
pub struct StepReplayReport3 {
    /// Policy that generated or names the step.
    pub policy: IntegrationPolicy,
    /// Coupling policies active during the step.
    pub couplings: Vec<CouplingPolicy>,
    /// Time step.
    pub dt: Real,
    /// Initial position.
    pub initial_position: Vector3,
    /// Initial velocity.
    pub initial_velocity: Vector3,
    /// Accumulated exact force.
    pub accumulated_force: Vector3,
    /// Proposed position.
    pub proposed_position: Vector3,
    /// Proposed velocity.
    pub proposed_velocity: Vector3,
    /// Diagnostics at the proposed state.
    pub diagnostics: SystemDiagnostics3,
    /// True when the proposal was produced by exact replay rather than a lossy adapter.
    pub exact_replay: bool,
}

impl ForceAccumulator3 {
    /// Adds an exact force contribution.
    pub fn push(&mut self, contribution: ForceContribution3) {
        self.contributions.push(contribution);
    }

    /// Returns force contributions.
    pub fn contributions(&self) -> &[ForceContribution3] {
        &self.contributions
    }

    /// Returns the exact summed force.
    pub fn total_force(&self) -> Vector3 {
        self.contributions
            .iter()
            .fold(Vector3::zero(), |total, contribution| {
                total + &contribution.force
            })
    }
}

impl StepReplayReport3 {
    /// Builds an exact explicit-Euler replay report for one body point mass.
    ///
    /// This is intentionally a small certified kernel: `v_{n+1} = v_n +
    /// (F/m)dt`, `x_{n+1} = x_n + v_{n+1}dt`, and diagnostics are evaluated
    /// exactly from the proposed state. It provides the report shape that
    /// richer symplectic, implicit, complementarity, heat, EM, and field/circuit
    /// adapters must match before their proposals are accepted.
    pub fn explicit_euler_replay(
        mass: Real,
        dt: Real,
        initial_position: Vector3,
        initial_velocity: Vector3,
        forces: &ForceAccumulator3,
    ) -> PhysicsResult<Self> {
        require_positive(&mass, PhysicsError::NonPositiveMass)?;
        require_positive(&dt, PhysicsError::NonPositiveTimeStep)?;
        let accumulated_force = forces.total_force();
        let acceleration = div_vector_by_real(accumulated_force.clone(), &mass)?;
        let proposed_velocity = initial_velocity.clone() + (acceleration * &dt);
        let proposed_position = initial_position.clone() + (proposed_velocity.clone() * &dt);
        let diagnostics = SystemDiagnostics3::from_mass_velocity(&mass, &proposed_velocity)?;
        Ok(Self {
            policy: IntegrationPolicy::ExplicitEulerReplay,
            couplings: Vec::new(),
            dt,
            initial_position,
            initial_velocity,
            accumulated_force,
            proposed_position,
            proposed_velocity,
            diagnostics,
            exact_replay: true,
        })
    }
}

impl SystemDiagnostics3 {
    /// Computes exact translational momentum and kinetic energy.
    pub fn from_mass_velocity(mass: &Real, velocity: &Vector3) -> PhysicsResult<Self> {
        require_positive(mass, PhysicsError::NonPositiveMass)?;
        let momentum = velocity.clone() * mass;
        let speed_squared = velocity.dot(velocity);
        let kinetic_energy = (mass * &speed_squared) / Real::from(2);
        Ok(Self {
            momentum,
            kinetic_energy: kinetic_energy.map_err(|_| PhysicsError::UnknownDiagnostic)?,
            status: DiagnosticStatus::Exact,
        })
    }
}

fn require_positive(value: &Real, error: PhysicsError) -> PhysicsResult<()> {
    match value.refine_sign_until(-64) {
        Some(RealSign::Positive) => Ok(()),
        Some(RealSign::Negative | RealSign::Zero) | None => Err(error),
    }
}

fn div_vector_by_real(vector: Vector3, rhs: &Real) -> PhysicsResult<Vector3> {
    Ok(Vector3::new([
        div_real(&vector[0], rhs)?,
        div_real(&vector[1], rhs)?,
        div_real(&vector[2], rhs)?,
    ]))
}

fn div_real(lhs: &Real, rhs: &Real) -> PhysicsResult<Real> {
    (lhs / rhs).map_err(|_| PhysicsError::NonPositiveMass)
}
