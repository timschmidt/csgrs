//! Reports for external numeric circuit adapters.

use hyperreal::Real;

/// External adapter family.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AdapterKind {
    /// DC operating-point proposal.
    Dc,
    /// AC/small-signal proposal.
    Ac,
    /// DAE/transient proposal.
    TransientDae,
    /// Coupled electrothermal proposal.
    Electrothermal,
}

/// Report for a lossy or numeric circuit adapter.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitAdapterReport {
    /// Adapter kind.
    pub kind: AdapterKind,
    /// Solver or external engine name.
    pub solver: String,
    /// Tolerance policy reported by the adapter.
    pub tolerance_policy: String,
    /// Whether exact residual replay accepted the proposal.
    pub exact_replay_accepted: Option<bool>,
    /// Human-readable notes.
    pub notes: Vec<String>,
}

/// First electrothermal trace fixture payload.
#[derive(Clone, Debug, PartialEq)]
pub struct ElectrothermalTraceFixture {
    /// Trace/path handle owned by `hyperpath` or `hyperdrc`.
    pub trace_handle: String,
    /// Material handle owned by `hyperphysics`.
    pub material_handle: String,
    /// Exact resistance used by the circuit model.
    pub resistance: Real,
    /// Exact thermal coefficient or explicit model scalar.
    pub thermal_coefficient: Real,
}
