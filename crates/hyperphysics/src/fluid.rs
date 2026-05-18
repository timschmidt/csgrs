//! Exact fluid/SPH setup and coupling diagnostics.
//!
//! Fluid solvers are usually approximate particle or grid engines, but their
//! material data, boundary handoff, particle masses, and conservation checks
//! can still be represented exactly. This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): approximate SPH/DFSPH/
//! IISPH solvers may propose states, while exact setup and replay diagnostics
//! remain explicit Hyper-owned facts.
//!
//! The policy names are the standard SPH family boundary: Monaghan's SPH
//! formulation ("Smoothed particle hydrodynamics," *Annual Review of Astronomy
//! and Astrophysics*, 1992), IISPH (Ihmsen et al., 2014), and DFSPH (Bender and
//! Koschier, 2015). This module does not advance those solvers; it records the
//! exact material/boundary state and mass/momentum diagnostics they must
//! preserve or report as approximate.

use hyperlattice::Vector3;
use hyperreal::{Real, RealSign};

use crate::{AxisAlignedBox3, PhysicsError, PhysicsResult};

/// Fluid report or diagnostic status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FluidReportStatus {
    /// Computed exactly from exact setup facts.
    Exact,
    /// Bounded solver certified the value.
    Certified,
    /// Solver or comparison did not decide.
    BoundedUnknown,
    /// Value came from a lossy solver proposal.
    Lossy,
}

/// Fluid solver or adapter policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FluidPolicy {
    /// Classic SPH adapter boundary.
    Sph,
    /// Divergence-free SPH adapter boundary.
    Dfsph,
    /// Implicit incompressible SPH adapter boundary.
    Iisph,
    /// Finite-volume/finite-element continuum adapter.
    ContinuumAdapter,
    /// Immersed-boundary adapter.
    ImmersedBoundaryAdapter,
    /// External black-box adapter.
    External(String),
}

/// Boundary classification for fluid/solid handoff.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FluidBoundaryKind {
    /// No-slip rigid boundary.
    NoSlipWall,
    /// Free-slip rigid boundary.
    FreeSlipWall,
    /// Inlet boundary.
    Inlet,
    /// Outlet boundary.
    Outlet,
    /// Free surface marker.
    FreeSurface,
    /// Source-specific boundary class.
    Custom(String),
}

/// Exact fluid material data.
#[derive(Clone, Debug, PartialEq)]
pub struct FluidMaterial {
    /// Provenance label or material handle.
    pub source: String,
    /// Rest density.
    pub rest_density: Real,
    /// Dynamic viscosity.
    pub viscosity: Real,
    /// Status of this material data.
    pub status: FluidReportStatus,
}

/// Exact particle setup for SPH-style adapters.
#[derive(Clone, Debug, PartialEq)]
pub struct FluidParticle3 {
    /// Particle handle.
    pub handle: String,
    /// Particle position.
    pub position: Vector3,
    /// Particle velocity.
    pub velocity: Vector3,
    /// Particle mass.
    pub mass: Real,
    /// Particle smoothing length.
    pub smoothing_length: Real,
}

/// Exact fluid boundary handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct FluidBoundary3 {
    /// Boundary handle.
    pub handle: String,
    /// Boundary kind.
    pub kind: FluidBoundaryKind,
    /// Exact broad bounds for the boundary region.
    pub bounds: AxisAlignedBox3,
    /// Boundary normal when known.
    pub normal: Option<Vector3>,
    /// Boundary velocity when known.
    pub velocity: Option<Vector3>,
    /// Boundary status.
    pub status: FluidReportStatus,
}

/// Exact fluid setup for a particle/boundary fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct FluidFixture3 {
    /// Fixture handle.
    pub handle: String,
    /// Material used by this fixture.
    pub material: FluidMaterial,
    /// Solver/adapter policy.
    pub policy: FluidPolicy,
    /// Particles exported to or replayed from an adapter.
    pub particles: Vec<FluidParticle3>,
    /// Exact boundaries exported to or replayed from an adapter.
    pub boundaries: Vec<FluidBoundary3>,
}

/// Exact mass/momentum conservation diagnostic for particles.
#[derive(Clone, Debug, PartialEq)]
pub struct FluidConservationReport3 {
    /// Total particle mass.
    pub total_mass: Real,
    /// Total linear momentum.
    pub total_momentum: Vector3,
    /// Number of particles included.
    pub particle_count: usize,
    /// Policy associated with the setup.
    pub policy: FluidPolicy,
    /// Diagnostic status.
    pub status: FluidReportStatus,
}

impl FluidMaterial {
    /// Creates exact fluid material data after validating density and viscosity.
    pub fn new(
        source: impl Into<String>,
        rest_density: Real,
        viscosity: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&rest_density, PhysicsError::NonPositiveDensity)?;
        require_nonnegative(&viscosity, PhysicsError::NegativeViscosity)?;
        Ok(Self {
            source: source.into(),
            rest_density,
            viscosity,
            status: FluidReportStatus::Exact,
        })
    }
}

impl FluidParticle3 {
    /// Creates an exact SPH-style particle setup.
    pub fn new(
        handle: impl Into<String>,
        position: Vector3,
        velocity: Vector3,
        mass: Real,
        smoothing_length: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&mass, PhysicsError::NonPositiveParticleMass)?;
        require_positive(&smoothing_length, PhysicsError::NonPositiveSmoothingLength)?;
        Ok(Self {
            handle: handle.into(),
            position,
            velocity,
            mass,
            smoothing_length,
        })
    }
}

impl FluidBoundary3 {
    /// Creates an exact fluid boundary handoff record.
    pub fn new(
        handle: impl Into<String>,
        kind: FluidBoundaryKind,
        bounds: AxisAlignedBox3,
        normal: Option<Vector3>,
        velocity: Option<Vector3>,
    ) -> Self {
        Self {
            handle: handle.into(),
            kind,
            bounds,
            normal,
            velocity,
            status: FluidReportStatus::Exact,
        }
    }
}

impl FluidFixture3 {
    /// Creates an exact fluid fixture for a named solver/adapter policy.
    pub fn new(handle: impl Into<String>, material: FluidMaterial, policy: FluidPolicy) -> Self {
        Self {
            handle: handle.into(),
            material,
            policy,
            particles: Vec::new(),
            boundaries: Vec::new(),
        }
    }

    /// Adds one exact particle setup record.
    pub fn with_particle(mut self, particle: FluidParticle3) -> Self {
        self.particles.push(particle);
        self
    }

    /// Adds one exact boundary setup record.
    pub fn with_boundary(mut self, boundary: FluidBoundary3) -> Self {
        self.boundaries.push(boundary);
        self
    }

    /// Computes exact total mass and linear momentum for the fixture particles.
    pub fn conservation_report(&self) -> FluidConservationReport3 {
        let mut total_mass = Real::zero();
        let mut total_momentum = Vector3::zero();
        for particle in &self.particles {
            total_mass = total_mass + particle.mass.clone();
            total_momentum = total_momentum + particle.velocity.clone() * &particle.mass;
        }
        FluidConservationReport3 {
            total_mass,
            total_momentum,
            particle_count: self.particles.len(),
            policy: self.policy.clone(),
            status: FluidReportStatus::Exact,
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
