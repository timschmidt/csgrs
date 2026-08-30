//! Infinite-precision triangulation algorithms built on hyperreal.
//!
//! This crate owns earcut-style polygon triangulation and incremental
//! Delaunay/CDT topology. Algorithm modules are individually
//! feature-gated (`earcut`, `cdt`, and `nd`) so applications only compile the
//! triangulators they use. The exact API uses [`Real`] directly, while the
//! optional `f64` module exposes boundary entry points that lift finite `f64`
//! coordinates into exact hyperreal-backed values before topology is decided.

#[cfg(feature = "cdt")]
pub mod cdt;
#[cfg(feature = "cdt")]
mod cdt_constraints;
#[cfg(feature = "cdt")]
mod cdt_insert;
#[cfg(feature = "cdt")]
mod cdt_validate;
mod context;
#[cfg(feature = "earcut")]
mod earcut;
pub mod error;
#[cfg(feature = "f64-interop")]
pub mod f64;
#[cfg(feature = "nd")]
pub mod nd;
pub mod polygon;
#[cfg(any(
    feature = "earcut",
    feature = "cdt",
    feature = "nd",
    feature = "runtime-select"
))]
mod predicate_evaluator;
#[cfg(any(feature = "earcut", feature = "cdt"))]
mod predicates;
#[cfg(feature = "runtime-select")]
pub mod runtime;
pub mod types;

pub use context::{TriangulationCertainty, TriangulationContext, TriangulationOutcome};
#[cfg(feature = "earcut")]
pub use earcut::{
    EarcutDiagnostics, EarcutReport, triangulate as earcut, triangulate_report as earcut_report,
};
pub use error::{Error, Result};
pub use hyperlimit::PredicatePolicy;
#[cfg(feature = "nd")]
pub use nd::{
    BistellarFlipApplyReportD, BistellarFlipD, BistellarFlipReportD, Cell, CellHandle,
    DelaunayComplex, DelaunayInsertionReportD, DelaunayTriangulationD, Face, Facet, FacetKey,
    PointD, Simplex, TdsBoundaryPolicyD, TdsCombinatorialValidationReportD,
    TdsCombinatorialViolationD, TdsGeometricValidationReportD, TdsGeometricViolationD,
    TdsManifoldValidationReportD, TdsManifoldViolationD, TriangulationD,
    TriangulationDataStructureD, VertexD, VertexHandle,
};
#[cfg(feature = "runtime-select")]
pub use runtime::{
    PolygonTriangulationAlgorithm, PolygonTriangulationReport, QualityPolicy, TriangulationOptions,
    triangulate_polygon, triangulate_polygon_with_report,
};
pub use types::{
    Constraint, ExactPoint, Point2, PolygonInput, PolygonInputFacts, Rational, Real,
    RingInputFacts, Sign, Triangle, TriangleIndices, TriangleLocation,
};
