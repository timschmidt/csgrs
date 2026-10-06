//! Query carriers for source-attributed part discovery.
//!
//! Query results rank candidates with explicit evidence, match status, and
//! unknowns. A query either returns source-backed facts or reports the missing
//! or conflicting evidence that prevents certification.

use crate::{PartId, PartQueryEvidence, TerminalId};

/// Query match/failure status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueryMatchStatus {
    /// Exact match to the requested constraint.
    ExactMatch,
    /// Certified compatible but not identical.
    CertifiedCompatible,
    /// Source evidence conflicts.
    SourceConflicting,
    /// Human review is required.
    NeedsReview,
    /// Required geometry/model is missing.
    MissingModel,
    /// Safe only in a narrower envelope.
    NarrowerEnvelope,
    /// Source says the item is unavailable or obsolete.
    Unavailable,
    /// Query could not be certified.
    Unknown,
}

/// Unknown field surfaced by a query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QueryUnknown {
    /// Field or constraint that could not be resolved.
    pub field: String,
    /// Reason the field remains unknown.
    pub reason: String,
}

/// Evidence bundle attached to a candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct QueryEvidence {
    /// Source/fact evidence from graph data.
    pub evidence: PartQueryEvidence,
    /// Match/failure status.
    pub status: QueryMatchStatus,
    /// Human-readable notes.
    pub notes: Vec<String>,
}

/// Ranked candidate returned by a query.
#[derive(Clone, Debug, PartialEq)]
pub struct QueryCandidate<T> {
    /// Candidate payload.
    pub value: T,
    /// Higher rank sorts earlier.
    pub rank: i32,
    /// Evidence for the candidate.
    pub evidence: QueryEvidence,
}

/// Query result with candidates and explicit unresolved fields.
#[derive(Clone, Debug, PartialEq)]
pub struct QueryResult<T> {
    /// Ranked candidates.
    pub candidates: Vec<QueryCandidate<T>>,
    /// Unknowns preventing a complete result.
    pub unknowns: Vec<QueryUnknown>,
    /// Source conflicts encountered during evaluation.
    pub conflicts: Vec<String>,
}

/// General part constraint.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PartConstraint {
    /// Family display name contains this text.
    FamilyNameContains(String),
    /// Part id contains this text.
    PartIdContains(String),
    /// Candidate must expose at least one interface.
    HasInterface,
    /// Candidate must have a geometry assertion or handoff.
    HasGeometry,
    /// Candidate must have a capability touching this target handle.
    HasCapabilityTarget(String),
    /// Source-specific constraint retained for adapters.
    Custom(String),
}

/// General part query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PartQuery {
    /// Constraints to apply.
    pub constraints: Vec<PartConstraint>,
}

/// Interface query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InterfaceQuery {
    /// Optional part id.
    pub part: Option<PartId>,
    /// Required interface name or kind label.
    pub interface: Option<String>,
}

/// Terminal query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TerminalQuery {
    /// Optional part id.
    pub part: Option<PartId>,
    /// Optional terminal id.
    pub terminal: Option<TerminalId>,
    /// Optional function/polarity label.
    pub function: Option<String>,
}

/// Compatibility query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompatibilityQuery {
    /// Left part or aspect handle.
    pub left: String,
    /// Right part or aspect handle.
    pub right: String,
}

/// Capability query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityQuery {
    /// Optional tool handle.
    pub tool: Option<String>,
    /// Optional target handle.
    pub target: Option<String>,
}

/// Geometry handoff query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GeometryQuery {
    /// Part or aspect handle.
    pub target: String,
    /// Required owning geometry namespace, when any.
    pub owner: Option<String>,
}

/// Electrical metadata query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ElectricalQuery {
    /// Part id.
    pub part: PartId,
    /// Required rail, pin, function, or model label.
    pub key: Option<String>,
}

/// Physics handoff query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicsQuery {
    /// Part id.
    pub part: PartId,
    /// Required material/property key.
    pub key: Option<String>,
}

/// Procurement query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProcurementQuery {
    /// Part id or manufacturer/supplier handle.
    pub target: String,
    /// Required currency, region, supplier, or lifecycle label.
    pub key: Option<String>,
}
