//! Electronics and EDA metadata carriers.
//!
//! `hyperparts` keeps electronic package, pinout, power-domain, and rating
//! facts source-attributed so EDA metadata can be queried without becoming a
//! circuit simulator. Circuit residuals, stamps, and behavioral models remain
//! owned by `hypercircuit`; this module only records catalog/interface facts and
//! reports the evidence and unknowns used to construct electrical handoffs.
//!
//! This mirrors Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>) at the EDA boundary:
//! exact voltage/current envelopes are carried when sources provide them, and
//! missing internal die nets or pin semantics remain explicit unknowns. Circuit
//! model interpretation is deliberately delegated, following the separation of
//! topology/model data from numerical solution in Nagel and Pederson, "SPICE
//! (Simulation Program with Integrated Circuit Emphasis)," ERL-M382, 1973.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::{
    AssertionCondition, AssertionValue, ConnectionDecision, PartId, PartQueryEvidence, PartsError,
    PartsResult, TerminalId,
};

/// Review status for an electronic fact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ElectricalFactStatus {
    /// Exact source fact or exact downstream handle.
    Exact,
    /// Reviewed/certified fact.
    Certified,
    /// Fact applies only under stated conditions.
    Conditional,
    /// Source was inspected and did not provide the fact.
    Unknown,
    /// Evidence conflicts across sources.
    Conflicting,
}

/// Broad electronic part family.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ElectronicPart {
    /// Passive resistor.
    Resistor,
    /// Passive capacitor.
    Capacitor,
    /// Passive inductor.
    Inductor,
    /// Diode or LED.
    Diode,
    /// Transistor or FET.
    Transistor,
    /// Integrated circuit.
    IntegratedCircuit,
    /// Connector.
    Connector,
    /// Electromechanical or source-specific class.
    Custom(String),
}

/// Electronic package metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ElectronicPackage {
    /// Package name, such as SOIC-8 or QFN-32.
    pub name: String,
    /// Package aspect/footprint handle.
    pub handle: String,
    /// Terminal count when known.
    pub terminal_count: Option<usize>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Exact voltage range carried with ratings or supply rails.
#[derive(Clone, Debug, PartialEq)]
pub struct VoltageRange {
    /// Minimum voltage.
    pub min: Real,
    /// Maximum voltage.
    pub max: Real,
}

/// Current limit/range carried with ratings or pins.
#[derive(Clone, Debug, PartialEq)]
pub struct CurrentLimit {
    /// Current value or exact interval.
    pub value: AssertionValue,
    /// Unit label.
    pub units: Option<String>,
    /// Conditions under which the limit applies.
    pub conditions: Vec<AssertionCondition>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Semantic function assigned to a pin or pad.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PinFunction {
    /// Positive or negative supply input/output.
    Power,
    /// Ground/common return.
    Ground,
    /// Digital signal.
    Digital,
    /// Analog signal.
    Analog,
    /// Reset pin.
    Reset,
    /// Test/debug/programming pin.
    Test,
    /// No-connect pin.
    NoConnect,
    /// Exposed pad, shield, or chassis connection.
    Shield,
    /// Differential pair member with pair label.
    Differential(String),
    /// Thermal pad/sense terminal.
    Thermal,
    /// Source-specific function.
    Custom(String),
    /// Source did not specify the function.
    Unknown,
}

/// Pinout entry for an external package terminal.
#[derive(Clone, Debug, PartialEq)]
pub struct Pinout {
    /// Package terminal id.
    pub terminal: TerminalId,
    /// Package pin/pad name.
    pub name: String,
    /// Semantic function.
    pub function: PinFunction,
    /// Voltage range when known.
    pub voltage: Option<VoltageRange>,
    /// Current limit when known.
    pub current: Option<CurrentLimit>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Mapping from a package terminal to a die port or circuit-model node.
#[derive(Clone, Debug, PartialEq)]
pub struct PinMap {
    /// Package terminal id.
    pub package_terminal: TerminalId,
    /// Die port, model node, or explicit unknown text.
    pub internal: AssertionValue,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Internal die port when source layout/netlist data exists.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DiePort {
    /// Stable internal port handle.
    pub handle: String,
    /// Semantic function.
    pub function: PinFunction,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Internal die net when source layout/netlist data exists.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DieNet {
    /// Stable internal net handle.
    pub handle: String,
    /// Ports connected by the net.
    pub ports: Vec<String>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Named supply or logic power domain.
#[derive(Clone, Debug, PartialEq)]
pub struct PowerDomain {
    /// Domain name.
    pub name: String,
    /// Rails that supply the domain.
    pub rails: Vec<SupplyRail>,
    /// Ground or return references.
    pub grounds: Vec<GroundReference>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Supply rail metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct SupplyRail {
    /// Rail name.
    pub name: String,
    /// Voltage range.
    pub voltage: VoltageRange,
    /// Terminals tied to the rail.
    pub terminals: Vec<TerminalId>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Ground, return, chassis, or shield reference.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GroundReference {
    /// Reference name.
    pub name: String,
    /// Terminals tied to the reference.
    pub terminals: Vec<TerminalId>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Absolute maximum rating.
#[derive(Clone, Debug, PartialEq)]
pub struct AbsoluteMaximumRating {
    /// Rating key.
    pub key: String,
    /// Rating value/range.
    pub value: AssertionValue,
    /// Unit label.
    pub units: Option<String>,
    /// Applicability conditions.
    pub conditions: Vec<AssertionCondition>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Recommended operating condition.
#[derive(Clone, Debug, PartialEq)]
pub struct RecommendedOperatingCondition {
    /// Condition key.
    pub key: String,
    /// Condition value/range.
    pub value: AssertionValue,
    /// Unit label.
    pub units: Option<String>,
    /// Applicability conditions.
    pub conditions: Vec<AssertionCondition>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Power-intent summary for a part or IC.
#[derive(Clone, Debug, PartialEq)]
pub struct PowerIntent {
    /// Power domains described by the source.
    pub domains: Vec<PowerDomain>,
    /// Required passive or sequencing notes retained as source text.
    pub requirements: Vec<String>,
    /// Review/exactness status.
    pub status: ElectricalFactStatus,
}

/// Source-attributed electrical compatibility result.
#[derive(Clone, Debug, PartialEq)]
pub struct ElectricalCompatibilityReport {
    /// Part associated with the report.
    pub part: PartId,
    /// Package metadata.
    pub package: Option<ElectronicPackage>,
    /// External pinout entries.
    pub pinout: Vec<Pinout>,
    /// Package-to-internal mappings.
    pub pin_maps: Vec<PinMap>,
    /// Internal ports available from source data.
    pub die_ports: Vec<DiePort>,
    /// Internal nets available from source data.
    pub die_nets: Vec<DieNet>,
    /// Power intent.
    pub power_intent: Option<PowerIntent>,
    /// Absolute maximum ratings.
    pub absolute_maximum_ratings: Vec<AbsoluteMaximumRating>,
    /// Recommended operating conditions.
    pub recommended_operating_conditions: Vec<RecommendedOperatingCondition>,
    /// Overall status.
    pub status: ElectricalFactStatus,
    /// Evidence used to construct the report.
    pub evidence: PartQueryEvidence,
    /// Explicit unknowns, such as unavailable internal IC routing.
    pub unknowns: Vec<String>,
}

/// Safe-connection report with the same explicit evidence style as graph
/// queries, but suitable for EDA/import adapters to preserve richer context.
#[derive(Clone, Debug, PartialEq)]
pub struct SafeConnectionReport {
    /// Left terminal handle.
    pub left: TerminalId,
    /// Right terminal handle.
    pub right: TerminalId,
    /// Conservative decision.
    pub decision: ConnectionDecision,
    /// Evidence used to make the decision.
    pub evidence: PartQueryEvidence,
    /// Explicit unknowns preventing a certified-safe decision.
    pub unknowns: Vec<String>,
}

impl VoltageRange {
    /// Creates an exact voltage range after validating `min <= max`.
    pub fn new(min: Real, max: Real) -> PartsResult<Self> {
        match min.partial_cmp(&max) {
            Some(Ordering::Less | Ordering::Equal) => Ok(Self { min, max }),
            Some(Ordering::Greater) | None => Err(PartsError::InvalidVoltageEnvelope),
        }
    }

    /// Returns true when two exact ranges overlap.
    pub fn overlaps(&self, other: &Self) -> Option<bool> {
        let left = self.min.partial_cmp(&other.max)?;
        let right = other.min.partial_cmp(&self.max)?;
        Some(!matches!(left, Ordering::Greater) && !matches!(right, Ordering::Greater))
    }
}

impl ElectricalCompatibilityReport {
    /// Returns true when the report has no unknown internal electrical facts.
    pub fn has_internal_detail(&self) -> bool {
        self.unknowns.is_empty() && !self.die_ports.is_empty() && !self.die_nets.is_empty()
    }
}
