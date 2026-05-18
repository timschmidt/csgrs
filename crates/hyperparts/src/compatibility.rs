//! Compatibility and connection decisions.

use crate::{PartId, PartQueryEvidence};

/// Relationship kind between part records.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CompatibilityKind {
    /// Symbol maps to a footprint.
    SymbolFootprint,
    /// Footprint maps to a package.
    FootprintPackage,
    /// Package maps to a concrete device or catalog part.
    PackageDevice,
    /// Mechanical mating relation.
    MechanicalMate,
    /// Tool/process can fabricate or inspect the target.
    ProcessSupport,
}

/// Source-attributed compatibility edge.
#[derive(Clone, Debug, PartialEq)]
pub struct CompatibilityRelation {
    /// Left part family id.
    pub left: PartId,
    /// Right part family id.
    pub right: PartId,
    /// Compatibility kind.
    pub kind: CompatibilityKind,
    /// Evidence for this relation.
    pub evidence: PartQueryEvidence,
}

/// General relationship kind between parts or aspects.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RelationshipKind {
    /// Contains relation.
    Contains,
    /// Variant-of relation.
    IsVariantOf,
    /// Family-instantiation relation.
    InstantiatesFamily,
    /// Substitution relation.
    SubstitutesFor,
    /// Mating relation.
    MatesWith,
    /// Fastening relation.
    Fastens,
    /// Conductive relation.
    Conducts,
    /// Insulating relation.
    Insulates,
    /// Routing relation.
    Routes,
    /// Heating relation.
    Heats,
    /// Cooling relation.
    Cools,
    /// Support relation.
    Supports,
    /// Drive relation.
    Drives,
    /// Sensing relation.
    Senses,
    /// Shielding relation.
    Shields,
    /// Sealing relation.
    Seals,
    /// Lubrication relation.
    Lubricates,
    /// Tool requirement relation.
    RequiresTool,
    /// Fabrication capability relation.
    CanBeMadeBy,
    /// Service capability relation.
    CanBeServicedBy,
    /// Fixture requirement relation.
    RequiresFixture,
    /// Compatibility relation.
    CompatibleWith,
    /// Incompatibility relation.
    IncompatibleWith,
    /// Source-specific relationship.
    Custom(String),
}

/// Compatibility class carried by a relationship.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CompatibilityClass {
    /// Exact/certified compatibility.
    Certified,
    /// Compatibility requires human review.
    NeedsReview,
    /// Evidence conflicts.
    Conflicting,
    /// Explicitly unknown.
    Unknown,
}

/// Interaction family for aspect-level relationships.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum InteractionKind {
    /// Mechanical interaction.
    Mechanical,
    /// Electrical interaction.
    Electrical,
    /// Thermal interaction.
    Thermal,
    /// Optical interaction.
    Optical,
    /// Fluid interaction.
    Fluid,
    /// Process/tool interaction.
    Process,
    /// Source-specific interaction.
    Custom(String),
}

/// General source-attributed relationship.
#[derive(Clone, Debug, PartialEq)]
pub struct Relationship {
    /// Left part or aspect handle.
    pub left: String,
    /// Right part or aspect handle.
    pub right: String,
    /// Relationship kind.
    pub kind: RelationshipKind,
    /// Compatibility class.
    pub compatibility: CompatibilityClass,
    /// Interaction kind.
    pub interaction: InteractionKind,
    /// Evidence for this relation.
    pub evidence: PartQueryEvidence,
}

/// Decision returned by safe-connection queries.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConnectionDecision {
    /// Available evidence says the connection is safe.
    Safe,
    /// Available evidence says the connection is unsafe.
    Unsafe,
    /// More evidence is required.
    Unknown,
}
