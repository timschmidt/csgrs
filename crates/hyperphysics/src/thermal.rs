//! Exact thermal setup and certification surfaces.
//!
//! Thermal solvers routinely mix finite-difference/FEM approximations, contact
//! models, and empirical material tables. This module keeps the setup boundary
//! exact and report-bearing: material/source provenance is retained, simple
//! closed-form slab/contact/Joule reports are replayed with [`Real`], and
//! richer FEM/FVM/transient adapters must report lossy or bounded status.
//! Approximate field solvers may propose values, but accepted boundary
//! decisions and scalar balances must be certified or explicitly unknown.
//!
//! The steady slab relation is Fourier conduction,
//! `q = k A (T_hot - T_cold) / L`. Contact resistance is represented as a
//! series thermal-resistance term.
//!
//! The transient lumped report uses an explicit energy balance,
//! `C dT/dt = P - (T - T_ambient) / R`, which is the standard first-order RC
//! thermal network analogue of electrical RC circuits. The report exposes the
//! residual terms instead of hiding them inside a time-stepper.

use hyperlattice::Vector3;
use hyperreal::{Real, RealSign};

use crate::{PhysicsError, PhysicsResult};

/// Status of a thermal report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ThermalReportStatus {
    /// Computed exactly from exact inputs.
    Exact,
    /// A bounded solver certified the value.
    Certified,
    /// Solver or comparison did not decide.
    BoundedUnknown,
    /// Value came from a lossy adapter.
    Lossy,
}

/// Thermal solver or adapter policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ThermalPolicy {
    /// Closed-form steady-state linear conduction.
    SteadyStateLinearConduction,
    /// Exact replay of one transient heat-equation or energy-balance step.
    TransientHeatEquationStep,
    /// Lumped RC thermal network replay.
    LumpedRcNetwork,
    /// Finite-element adapter boundary.
    FemAdapter,
    /// Finite-volume adapter boundary.
    FvmAdapter,
    /// External black-box adapter boundary.
    External(String),
}

/// Thermal material data.
#[derive(Clone, Debug, PartialEq)]
pub struct ThermalMaterial {
    /// Provenance label or material handle.
    pub source: String,
    /// Thermal conductivity.
    pub conductivity: Real,
}

/// Temperature field boundary value or field sample.
#[derive(Clone, Debug, PartialEq)]
pub struct TemperatureField3 {
    /// Source or boundary-condition label.
    pub source: String,
    /// Temperature value.
    pub temperature: Real,
    /// Status of the value.
    pub status: ThermalReportStatus,
}

/// Heat-flux boundary condition.
#[derive(Clone, Debug, PartialEq)]
pub struct HeatFluxBoundary3 {
    /// Boundary label.
    pub source: String,
    /// Normal heat flux.
    pub flux: Real,
    /// Boundary normal when known.
    pub normal: Option<Vector3>,
    /// Status of the value.
    pub status: ThermalReportStatus,
}

/// Volumetric or surface heat source.
#[derive(Clone, Debug, PartialEq)]
pub struct HeatSource3 {
    /// Source label.
    pub source: String,
    /// Power or integrated source strength.
    pub power: Real,
    /// Status of the value.
    pub status: ThermalReportStatus,
}

/// Thermal contact pair with optional contact resistance.
#[derive(Clone, Debug, PartialEq)]
pub struct ThermalContactPair3 {
    /// First body/surface handle.
    pub left: String,
    /// Second body/surface handle.
    pub right: String,
    /// Contact area.
    pub area: Real,
    /// Area-normalized or lumped contact resistance used by the caller.
    pub resistance: Real,
}

/// Coupling port for thermal/circuit/optics/mechanics handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct ThermalPort3 {
    /// Port handle.
    pub handle: String,
    /// Coupled domain, such as `hypercircuit` or `hyperpath`.
    pub domain: String,
    /// Temperature field at the port.
    pub temperature: TemperatureField3,
    /// Optional heat source attached to the port.
    pub heat_source: Option<HeatSource3>,
}

/// Lumped thermal node used by RC and transient balance reports.
#[derive(Clone, Debug, PartialEq)]
pub struct LumpedThermalNode {
    /// Node handle.
    pub handle: String,
    /// Current node temperature.
    pub temperature: Real,
    /// Lumped heat capacity/thermal capacitance.
    pub heat_capacity: Real,
}

/// Exact steady-state conduction report for a slab or box layer.
#[derive(Clone, Debug, PartialEq)]
pub struct SteadySlabConductionReport {
    /// Material used by the report.
    pub material: ThermalMaterial,
    /// Slab thickness.
    pub thickness: Real,
    /// Cross-sectional area.
    pub area: Real,
    /// Hot-side temperature.
    pub hot_temperature: Real,
    /// Cold-side temperature.
    pub cold_temperature: Real,
    /// Optional contact resistance in series with the slab resistance.
    pub contact_resistance: Real,
    /// Exact heat rate from hot to cold.
    pub heat_rate: Real,
    /// Equivalent thermal resistance.
    pub equivalent_resistance: Real,
    /// Report status.
    pub status: ThermalReportStatus,
}

/// Exact transient energy-balance step report.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientThermalStepReport {
    /// Node before the step.
    pub node: LumpedThermalNode,
    /// Time step.
    pub time_step: Real,
    /// Net heat rate into the node.
    pub net_heat_rate: Real,
    /// Exact temperature increment `dt * q / C`.
    pub temperature_delta: Real,
    /// Exact next temperature.
    pub next_temperature: Real,
    /// Residual expression.
    pub expression: String,
    /// Policy used by this report.
    pub policy: ThermalPolicy,
    /// Report status.
    pub status: ThermalReportStatus,
}

/// Exact one-node lumped RC thermal-network step report.
#[derive(Clone, Debug, PartialEq)]
pub struct LumpedRcThermalStepReport {
    /// Node before the step.
    pub node: LumpedThermalNode,
    /// Ambient/reference temperature.
    pub ambient_temperature: Real,
    /// Thermal resistance to ambient.
    pub thermal_resistance: Real,
    /// Applied heat source into the node.
    pub heat_source: Real,
    /// Time step.
    pub time_step: Real,
    /// Exact conductive heat rate `-(T - T_ambient) / R`.
    pub conductive_heat_rate: Real,
    /// Exact net heat rate into the node.
    pub net_heat_rate: Real,
    /// Exact temperature increment.
    pub temperature_delta: Real,
    /// Exact next temperature.
    pub next_temperature: Real,
    /// Residual expression.
    pub expression: String,
    /// Policy used by this report.
    pub policy: ThermalPolicy,
    /// Report status.
    pub status: ThermalReportStatus,
}

impl ThermalMaterial {
    /// Creates a thermal material after certifying positive conductivity.
    pub fn new(source: impl Into<String>, conductivity: Real) -> PhysicsResult<Self> {
        require_positive(&conductivity, PhysicsError::NonPositiveThermalConductivity)?;
        Ok(Self {
            source: source.into(),
            conductivity,
        })
    }
}

impl LumpedThermalNode {
    /// Creates a lumped thermal node after certifying positive heat capacity.
    pub fn new(
        handle: impl Into<String>,
        temperature: Real,
        heat_capacity: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&heat_capacity, PhysicsError::NonPositiveThermalCapacitance)?;
        Ok(Self {
            handle: handle.into(),
            temperature,
            heat_capacity,
        })
    }
}

impl ThermalContactPair3 {
    /// Creates a thermal contact pair with positive area and non-negative resistance.
    pub fn new(
        left: impl Into<String>,
        right: impl Into<String>,
        area: Real,
        resistance: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&area, PhysicsError::NonPositiveArea)?;
        require_nonnegative(&resistance, PhysicsError::NegativeResistance)?;
        Ok(Self {
            left: left.into(),
            right: right.into(),
            area,
            resistance,
        })
    }
}

impl SteadySlabConductionReport {
    /// Computes exact steady conduction through one slab with optional contact resistance.
    pub fn through_slab(
        material: ThermalMaterial,
        thickness: Real,
        area: Real,
        hot_temperature: Real,
        cold_temperature: Real,
        contact_resistance: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&thickness, PhysicsError::NonPositiveThickness)?;
        require_positive(&area, PhysicsError::NonPositiveArea)?;
        require_nonnegative(&contact_resistance, PhysicsError::NegativeResistance)?;

        let conductance_area = &material.conductivity * &area;
        let slab_resistance = div_real(&thickness, &conductance_area)?;
        let equivalent_resistance = slab_resistance + contact_resistance.clone();
        require_positive(
            &equivalent_resistance,
            PhysicsError::InvalidThermalDenominator,
        )?;
        let delta_t = hot_temperature.clone() - cold_temperature.clone();
        let heat_rate = div_real(&delta_t, &equivalent_resistance)?;

        Ok(Self {
            material,
            thickness,
            area,
            hot_temperature,
            cold_temperature,
            contact_resistance,
            heat_rate,
            equivalent_resistance,
            status: ThermalReportStatus::Exact,
        })
    }

    /// Computes exact Joule heating `P = I^2 R` for a PCB trace or lumped conductor.
    pub fn joule_heating(
        source: impl Into<String>,
        current: Real,
        resistance: Real,
    ) -> PhysicsResult<HeatSource3> {
        require_nonnegative(&resistance, PhysicsError::NegativeResistance)?;
        let power = (&current * &current) * resistance;
        Ok(HeatSource3 {
            source: source.into(),
            power,
            status: ThermalReportStatus::Exact,
        })
    }
}

impl TransientThermalStepReport {
    /// Replays an exact explicit transient energy-balance step.
    pub fn energy_balance_step(
        node: LumpedThermalNode,
        time_step: Real,
        net_heat_rate: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&time_step, PhysicsError::NonPositiveTimeStep)?;
        let temperature_delta = div_real(&(&time_step * &net_heat_rate), &node.heat_capacity)?;
        let next_temperature = node.temperature.clone() + temperature_delta.clone();
        Ok(Self {
            node,
            time_step,
            net_heat_rate,
            temperature_delta,
            next_temperature,
            expression: "T_next = T + dt * q / C".into(),
            policy: ThermalPolicy::TransientHeatEquationStep,
            status: ThermalReportStatus::Exact,
        })
    }
}

impl LumpedRcThermalStepReport {
    /// Replays one exact lumped RC step `C dT/dt = P - (T - T_ambient) / R`.
    pub fn explicit_euler_step(
        node: LumpedThermalNode,
        ambient_temperature: Real,
        thermal_resistance: Real,
        heat_source: Real,
        time_step: Real,
    ) -> PhysicsResult<Self> {
        require_positive(&thermal_resistance, PhysicsError::InvalidThermalDenominator)?;
        require_positive(&time_step, PhysicsError::NonPositiveTimeStep)?;
        let temperature_difference = node.temperature.clone() - ambient_temperature.clone();
        let conductive_heat_rate = -div_real(&temperature_difference, &thermal_resistance)?;
        let net_heat_rate = heat_source.clone() + conductive_heat_rate.clone();
        let balance = TransientThermalStepReport::energy_balance_step(
            node.clone(),
            time_step.clone(),
            net_heat_rate.clone(),
        )?;
        Ok(Self {
            node,
            ambient_temperature,
            thermal_resistance,
            heat_source,
            time_step,
            conductive_heat_rate,
            net_heat_rate,
            temperature_delta: balance.temperature_delta,
            next_temperature: balance.next_temperature,
            expression: "C dT/dt = P - (T - T_ambient) / R".into(),
            policy: ThermalPolicy::LumpedRcNetwork,
            status: ThermalReportStatus::Exact,
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
    (lhs / rhs).map_err(|_| PhysicsError::InvalidThermalDenominator)
}
