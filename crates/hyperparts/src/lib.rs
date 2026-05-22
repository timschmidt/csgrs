//! Source-attributed part knowledge graph carriers.
//!
//! `hyperparts` owns cross-domain part knowledge: families, variants,
//! revisions, subparts, terminals, interfaces, compatibility relations,
//! tool/process capabilities, import reports, and query evidence. It does not
//! own downstream geometry, physics, circuit solving, routing, or fabrication
//! kernels; it indexes and hands off to those crates.
//!
//! The crate follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>) at the system boundary:
//! source data is retained as attributed facts, exact values, or explicit
//! unknowns. Missing catalog, EDA, or manufacturing fields are not guessed into
//! primitive floats or silent defaults.

pub mod assertion;
pub mod compatibility;
pub mod eda_intake;
pub mod electronics;
pub mod error;
pub mod geometry;
pub mod graph;
pub mod identity;
pub mod interface;
pub mod physics;
pub mod process;
pub mod query;

pub use assertion::{
    Assertion, AssertionCondition, AssertionConfidence, AssertionStatus, AssertionValue,
    ComplianceClaim, GeneralPartAssertion, PartAssertion, PartKnowledgeReport, PartLifecycle,
    ProcurementOffer, SourceRef, SourceRevision,
};
pub use compatibility::{
    CompatibilityClass, CompatibilityKind, CompatibilityRelation, ConnectionDecision,
    InteractionKind, Relationship, RelationshipKind,
};
pub use eda_intake::{
    AutorouterOutputRecord, CircuitJsonSourceRecord, CircuitResidualFactHandoff,
    CircuitResidualParameter, DrcFabricationHandoff, EdaAuthoringBundle, EdaAuthoringImportResult,
    EdaExactField, EdaFabricationReadiness, EdaFootprintString, EdaHandoffStatus, EdaIntakeStatus,
    EdaModelStatus, EdaPackageMetadata, EdaPackagePin, EdaRouteStatus, FabricationOutputRecord,
    GeneratedModelReference, RouteGeometryHandoff, import_eda_authoring_bundle,
};
pub use electronics::{
    AbsoluteMaximumRating, CurrentLimit, DieNet, DiePort, ElectricalCompatibilityReport,
    ElectricalFactStatus, ElectronicPackage, ElectronicPart, GroundReference, PinFunction, PinMap,
    Pinout, PowerDomain, PowerIntent, RecommendedOperatingCondition, SafeConnectionReport,
    SupplyRail, VoltageRange,
};
pub use error::{PartsError, PartsResult};
pub use geometry::{
    CurveProfileHandle, FootprintHandle, GeometryHandle, GeometryHandoffReport, GeometryStatus,
    GridFeature, GridSystem, ModelArtifact, MountingPattern, PackageBodyHandle, ShapeSource,
};
pub use graph::{
    ImportIssue, ImportIssueKind, ImportReport, ImportTargetKind, PartFamily, PartGraph,
    PartQueryEvidence, PartQueryResult, PartVariant,
};
pub use hyperreal::Real;
pub use identity::{
    InternalPartNumber, ManufacturerPartNumber, PartId, RevisionId, SupplierSku, TerminalId,
    VariantId,
};
pub use interface::{
    AspectKind, ElectricalPolarity, Hole, Interface, InterfaceKind, Lead, MaterialRegion,
    MountingFeature, Pad, PartAspect, Pin, Port, ReferenceDesignatorClass, Terminal, TerminalRole,
    VoltageEnvelope,
};
pub use physics::{
    EnvironmentalEnvelope, MassPropertyNeed, MaterialRequirement, MechanicalLoadPath,
    PhysicalFactStatus, PhysicalPropertyHandle, PhysicsHandoffReport, ThermalPath,
};
pub use process::{
    CalibrationState, Capability, CapabilityEnvelope, CapabilityInput, CapabilityOutput,
    CapabilityStatus, ConsumableRequirement, FixtureRequirement, ManufacturingRoute, Operation,
    Process, ProcessCapability, ProcessKind, ToleranceEnvelope, ToolCapability, ToolPart,
};
pub use query::{
    CapabilityQuery, CompatibilityQuery, ElectricalQuery, GeometryQuery, InterfaceQuery,
    PartConstraint, PartQuery, PhysicsQuery, ProcurementQuery, QueryCandidate, QueryEvidence,
    QueryMatchStatus, QueryResult, QueryUnknown, TerminalQuery,
};
