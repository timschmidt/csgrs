//! Hyper-native exact boundary representation.
//!
//! HyperBREP owns immutable retained 3D geometry and typed-arena topology.
//! [`hyperreal::Real`] is the only authoritative scalar. Mathematical
//! decisions are delegated to certified Hyperlimit predicates; an unresolved
//! predicate is an error, never an implicit tolerance decision.
//!
//! Construction is a validation boundary: [`ModelBuilder`] rejects invalid
//! local relationships as they are staged, and [`ModelBuilder::finish`]
//! publishes an immutable [`Model`] only after global ownership checks pass.

// Retained spatial points add another geometry layer to rustdoc auto-trait traversal.
#![recursion_limit = "256"]
#![warn(missing_docs)]

mod cache;
mod context;
mod error;
mod geometry;
mod parameter;
mod persistence;
mod topology;

mod boolean_kernel;
mod construction;
#[cfg(feature = "tessellation")]
mod meshing;

/// Predicate policy used by HyperBREP's strictly certified source-topology
/// operations.
///
/// HyperBREP does not reinterpret an unresolved predicate as equality. Derived
/// approximate products such as chordal tessellations remain outside the
/// authoritative BREP topology.
pub const STRICT_PREDICATES: hyperlimit::PredicatePolicy = hyperlimit::PredicatePolicy::STRICT;

/// Explicit finite-refinement policy for test oracles that compare independently
/// constructed representations of the same mathematical value.
#[cfg(test)]
pub(crate) const TEST_ORACLE_PREDICATES: hyperlimit::PredicatePolicy =
    hyperlimit::PredicatePolicy::APPROXIMATE_512;

pub(crate) use boolean_kernel::{
    ArenaBooleanResult, ArenaClassifiedFace, ArenaFacePairIntersection, ArenaFaceSelection,
    ArenaSolidIntersectionGraph,
};
pub use construction::{ConstructionError, LoftSection, RationalBezierSweepFrame, TensorPatch};
pub use context::OperationContext;
pub use error::{GeometryError, GeometryResult, NonConicalGeneratorReason};
pub use geometry::{
    Axis, CircularArcGeometry, ConeBranch, ConeGeometry, ConicalRevolution, ConicalRevolutionChart,
    Curve3, Curve3Kind, CurveDerivative3, CurveParameterLocation, CurveSurfaceComponents,
    CurveSurfaceContactKind, CurveSurfaceIntersection, CurveSurfacePoint, CylinderGeometry,
    GeometricPoint3, MaterializedSurfacePcurve, NurbsCurveGeometry, ParameterDomain, Pcurve,
    PlanarCurveGeometry, RationalBezierCurveGeometry, Surface, SurfaceBounds, SurfaceDomain,
    SurfaceIntersectionComponents, SurfaceIntersectionCurve, SurfaceIntersectionLine,
    SurfaceIntersectionOperand, SurfaceIntersectionParameterRay, SurfaceIntersectionPcurve,
    SurfaceIntersectionRay, SurfaceIsoAxis, SurfaceKind, SurfaceParameterDomain, SurfacePartials,
    SurfacePcurveCorrespondence, SurfaceSurfaceIntersection, TorusGeometry,
    TranslationCurveGeometry,
};
pub use hyperlattice::{Aabb, Matrix4, Point2, Point3, Real, Vector2, Vector3};
pub use hypersolve::work_budget::{BudgetExceeded, BudgetOutcome, WorkBudget, WorkKind, WorkUsage};
pub use parameter::{Parameter, ParameterRange};
pub(crate) use persistence::ImportedModel;
pub use persistence::{ArchiveLimitExceeded, ArchiveLimits, ArchiveResource, PersistenceError};
pub use topology::{
    BuildError, Direction, EditError, Endpoint, EntityKind, GeometryReferenceError, ModelCounts,
    Orientation, ParameterCorrespondence, QueryError, SolidPointLocation, TopologyEditError,
    ValidationReport,
};
// Arena positions are diagnostic labels only; public operations take bound references.
pub(crate) use topology::{
    ArenaBoundaryBridgeCurveSplit, ArenaBoundaryBridgeFaceSplit, ArenaClosedSurfaceCurveFaceSplit,
    ArenaCurveFaceSplit, ArenaEdgeSplit, ArenaEdit, ArenaFacePartition, ArenaFaceSplit,
    ArenaFaceTracePartition, ArenaModel, ArenaModelBuilder, ArenaSurfaceCurveFaceSplit, Edge,
    EdgeUse, Face, Solid,
};
pub use topology::{
    Curve3Id, EdgeId, EdgeUseId, FaceId, PcurveId, ShellId, SolidId, SurfaceId, VertexId, WireId,
};

mod ownership;
pub use ownership::boolean::{
    ClassifiedFaceRef, FacePairIntersection, FacePairIntersectionRef, FaceSelection,
    PartitionedModel, SolidIntersectionGraph, partition_contained_face_by_plane_region,
};
#[cfg(feature = "tessellation")]
pub use ownership::tessellation;
pub use ownership::*;
pub use ownership::{boolean, builder};

pub use boolean_kernel::{
    BooleanError, BooleanOperation, FacePairRelation, FacePairTrim, FaceSelectionAction,
};
