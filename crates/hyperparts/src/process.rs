//! Tool and process capability carriers.
//!
//! Manufacturing metadata is source-attributed planning data, not a CAM kernel.
//! `hyperparts` records which tool/process/fixture/material combinations are
//! claimed to work and leaves geometric toolpath validation to downstream
//! crates such as `hyperpath` and `hyperdrc`.
//!
//! Process claims expose source conditions and uncertainty rather than treating
//! a catalog note as a certified fabrication recipe.

use crate::{AssertionCondition, AssertionValue, PartId, PartQueryEvidence};

/// Manufacturing or inspection process family.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ProcessKind {
    /// Additive manufacturing process.
    Additive,
    /// Subtractive manufacturing process.
    Subtractive,
    /// PCB fabrication or assembly process.
    Pcb,
    /// Inspection or test process.
    Inspection,
    /// Custom source-specific process.
    Custom(String),
}

/// Review status for tool/process facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CapabilityStatus {
    /// Exact/reviewed capability.
    Certified,
    /// Imported but needs review.
    NeedsReview,
    /// Capability is valid only under stated envelope/fixture conditions.
    Conditional,
    /// Evidence conflicts across sources.
    Conflicting,
    /// Source did not provide enough information.
    Unknown,
}

/// A tool represented as a part or external toolchain handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ToolPart {
    /// Tool handle or source id.
    pub handle: String,
    /// Optional part id when the tool itself is modeled in the part graph.
    pub part: Option<PartId>,
    /// Human-readable tool name.
    pub name: String,
}

/// Source-attributed process definition.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Process {
    /// Process handle.
    pub handle: String,
    /// Process kind.
    pub kind: ProcessKind,
    /// Human-readable process name.
    pub name: String,
}

/// Operation within a manufacturing route.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Operation {
    /// Operation handle.
    pub handle: String,
    /// Process handle.
    pub process: String,
    /// Target part, aspect, feature, or material handle.
    pub target: String,
}

/// Capability input requirement.
#[derive(Clone, Debug, PartialEq)]
pub struct CapabilityInput {
    /// Input role, such as material, stock, feature, or fixture.
    pub role: String,
    /// Required input handle.
    pub handle: String,
    /// Quantity or exact unknown.
    pub quantity: AssertionValue,
}

/// Capability output description.
#[derive(Clone, Debug, PartialEq)]
pub struct CapabilityOutput {
    /// Output role, such as part, feature, inspection, or artifact.
    pub role: String,
    /// Produced output handle.
    pub handle: String,
    /// Quantity or exact unknown.
    pub quantity: AssertionValue,
}

/// Operating envelope for a process capability.
#[derive(Clone, Debug, PartialEq)]
pub struct CapabilityEnvelope {
    /// Materials supported by this capability.
    pub materials: Vec<String>,
    /// Exact scalar/range conditions such as speed, temperature, power, or load.
    pub conditions: Vec<AssertionCondition>,
    /// Safety limits that must be retained with the claim.
    pub safety_limits: Vec<AssertionCondition>,
    /// Review status for the envelope.
    pub status: CapabilityStatus,
}

/// Tolerance envelope retained with a capability or route.
#[derive(Clone, Debug, PartialEq)]
pub struct ToleranceEnvelope {
    /// Dimension or feature being constrained.
    pub target: String,
    /// Tolerance value/range.
    pub tolerance: AssertionValue,
    /// Unit label.
    pub units: Option<String>,
    /// Conditions under which the tolerance applies.
    pub conditions: Vec<AssertionCondition>,
    /// Review status.
    pub status: CapabilityStatus,
}

/// Fixture requirement for an operation or route.
#[derive(Clone, Debug, PartialEq)]
pub struct FixtureRequirement {
    /// Fixture handle.
    pub fixture: String,
    /// Required setup condition or unknown.
    pub setup: AssertionValue,
    /// Review status.
    pub status: CapabilityStatus,
}

/// Consumable requirement for an operation or route.
#[derive(Clone, Debug, PartialEq)]
pub struct ConsumableRequirement {
    /// Consumable handle.
    pub consumable: String,
    /// Quantity or exact unknown.
    pub quantity: AssertionValue,
    /// Unit label.
    pub units: Option<String>,
    /// Review status.
    pub status: CapabilityStatus,
}

/// Calibration state retained with a tool capability.
#[derive(Clone, Debug, PartialEq)]
pub struct CalibrationState {
    /// Calibration label or certificate id.
    pub label: String,
    /// Calibration value/date/status from the source.
    pub value: AssertionValue,
    /// Review status.
    pub status: CapabilityStatus,
}

/// Full capability claim for a tool/process pair.
#[derive(Clone, Debug, PartialEq)]
pub struct Capability {
    /// Tool that performs the capability.
    pub tool: ToolPart,
    /// Process performed by the tool.
    pub process: Process,
    /// Inputs consumed or required.
    pub inputs: Vec<CapabilityInput>,
    /// Outputs produced.
    pub outputs: Vec<CapabilityOutput>,
    /// Operating envelope.
    pub envelope: CapabilityEnvelope,
    /// Tolerance envelope.
    pub tolerances: Vec<ToleranceEnvelope>,
    /// Fixture requirements.
    pub fixtures: Vec<FixtureRequirement>,
    /// Consumable requirements.
    pub consumables: Vec<ConsumableRequirement>,
    /// Calibration state.
    pub calibration: Option<CalibrationState>,
    /// Evidence used to construct the capability.
    pub evidence: PartQueryEvidence,
    /// Review status.
    pub status: CapabilityStatus,
}

/// Ordered manufacturing route.
#[derive(Clone, Debug, PartialEq)]
pub struct ManufacturingRoute {
    /// Route handle.
    pub handle: String,
    /// Target part, feature, or artifact.
    pub target: String,
    /// Ordered operations.
    pub operations: Vec<Operation>,
    /// Capabilities used by the route.
    pub capabilities: Vec<Capability>,
    /// Explicit unknowns that prevent the route from being certified.
    pub unknowns: Vec<String>,
    /// Evidence used to construct the route.
    pub evidence: PartQueryEvidence,
    /// Review status.
    pub status: CapabilityStatus,
}

/// A process capability attached to a part variant or tool.
#[derive(Clone, Debug, PartialEq)]
pub struct ProcessCapability {
    /// Process kind.
    pub kind: ProcessKind,
    /// Human-readable capability statement.
    pub statement: String,
}

/// Tool/process capability record.
#[derive(Clone, Debug, PartialEq)]
pub struct ToolCapability {
    /// Tool name or id.
    pub tool: String,
    /// Supported process.
    pub capability: ProcessCapability,
}
