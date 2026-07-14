//! Exact circuit/physics coupling carriers.
//!
//! Coupled field/circuit problems are naturally residual systems: circuit
//! unknowns, thermal states, mechanical states, and field states are tied
//! together by equations whose numerical solution may be approximate. This
//! module keeps the coupling boundary exact and report-bearing before any DAE
//! or field solver adapter is selected, so proposed coupled states require
//! exact residual replay or explicit uncertainty.
//!
//! The first exact fixture is electrothermal RC coupling. It records the
//! Joule-heating relation `P = I^2 R(T)` and the linear temperature coefficient
//! `R(T) = R0 * (1 + alpha * (T - T_ref))`. It supplies circuit-owned ports and
//! residual payloads, not a full field solver. The README lists the underlying
//! circuit, exact-computation, and field-coupling references.

use hyperreal::{Real, RealSign};
use hypersolve::{Constraint, Expr, Problem};

use crate::{CircuitError, CircuitResult, ComponentId, NetId};

/// Electrical port that binds circuit topology to a physical handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicalElectricalPort {
    /// Circuit port handle.
    pub handle: String,
    /// Circuit net attached to the port.
    pub net: NetId,
    /// Physical-domain handle owned by another Hyper crate.
    pub physical_handle: String,
}

/// Thermal port reference consumed from `hyperphysics`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ThermalPort {
    /// Circuit-side handle.
    pub handle: String,
    /// `hyperphysics` thermal port handle.
    pub physics_handle: String,
}

/// Electromechanical port reference consumed from `hyperphysics`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ElectromechanicalPort {
    /// Circuit-side handle.
    pub handle: String,
    /// `hyperphysics` body, fixture, or actuator handle.
    pub physics_handle: String,
}

/// Coupled residual block metadata before solver lowering.
#[derive(Clone, Debug, PartialEq)]
pub struct CoupledResidualBlock {
    /// Block handle.
    pub handle: String,
    /// Circuit components participating in the block.
    pub components: Vec<ComponentId>,
    /// Physical handles participating in the block.
    pub physical_handles: Vec<String>,
    /// Exact residual values when replayed.
    pub residuals: Vec<Real>,
    /// Human-readable provenance/equation labels.
    pub evidence: Vec<String>,
}

impl CoupledResidualBlock {
    /// Lowers replayed residual values into a `hypersolve` problem fragment.
    ///
    /// Circuit and physics crates keep domain ownership of ports and evidence;
    /// the generic residual rows are handed to `hypersolve` so candidate
    /// acceptance semantics stay centralized.
    pub fn to_hypersolve_problem(&self) -> Problem {
        let mut problem = Problem::default();
        for (index, residual) in self.residuals.iter().enumerate() {
            let name = self
                .evidence
                .get(index)
                .cloned()
                .unwrap_or_else(|| format!("{} residual {index}", self.handle));
            problem.add_constraint(Constraint::equality(name, Expr::real(residual.clone())));
        }
        problem
    }
}

/// Exact electrothermal RC coupling report.
#[derive(Clone, Debug, PartialEq)]
pub struct ElectrothermalRcReport {
    /// Component being coupled.
    pub component: ComponentId,
    /// Base/reference resistance `R0`.
    pub reference_resistance: Real,
    /// Temperature coefficient `alpha`.
    pub thermal_coefficient: Real,
    /// Current circuit temperature.
    pub temperature: Real,
    /// Reference temperature.
    pub reference_temperature: Real,
    /// Circuit current.
    pub current: Real,
    /// Exact temperature-adjusted resistance.
    pub adjusted_resistance: Real,
    /// Exact Joule heating `I^2 R(T)`.
    pub joule_heating: Real,
    /// Coupled residual block for replay.
    pub residual_block: CoupledResidualBlock,
}

impl ElectrothermalRcReport {
    /// Computes exact `R(T)` and `P = I^2 R(T)` for an electrothermal RC fixture.
    pub fn replay(
        component: ComponentId,
        reference_resistance: Real,
        thermal_coefficient: Real,
        temperature: Real,
        reference_temperature: Real,
        current: Real,
        physics_handle: impl Into<String>,
    ) -> CircuitResult<Self> {
        require_nonnegative(&reference_resistance, CircuitError::NegativeResistance)?;
        let temperature_delta = temperature.clone() - reference_temperature.clone();
        let multiplier = Real::one() + (&thermal_coefficient * &temperature_delta);
        let adjusted_resistance = &reference_resistance * &multiplier;
        require_nonnegative(&adjusted_resistance, CircuitError::NegativeResistance)?;
        let joule_heating = (&current * &current) * adjusted_resistance.clone();
        let residuals = vec![Real::zero()];
        Ok(Self {
            component: component.clone(),
            reference_resistance,
            thermal_coefficient,
            temperature,
            reference_temperature,
            current,
            adjusted_resistance,
            joule_heating,
            residual_block: CoupledResidualBlock {
                handle: "electrothermal-rc".into(),
                components: vec![component],
                physical_handles: vec![physics_handle.into()],
                residuals,
                evidence: vec![
                    "R(T) = R0 * (1 + alpha * (T - T_ref))".into(),
                    "P = I^2 R(T)".into(),
                ],
            },
        })
    }
}

fn require_nonnegative(value: &Real, error: CircuitError) -> CircuitResult<()> {
    match value.refine_sign_until(-64) {
        Some(RealSign::Positive | RealSign::Zero) => Ok(()),
        Some(RealSign::Negative) | None => Err(error),
    }
}
