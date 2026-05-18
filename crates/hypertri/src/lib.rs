//! Infinite-precision triangulation algorithms built on hyperreal.
//!
//! This crate owns source ports of earcut-style polygon triangulation and
//! spade-style Delaunay/CDT topology. Algorithm modules are individually
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
#[cfg(feature = "earcut")]
pub mod earcut;
pub mod error;
#[cfg(feature = "f64-interop")]
pub mod f64;
pub mod kernel;
#[cfg(feature = "nd")]
pub mod nd;
pub mod polygon;
pub mod predicates;
#[cfg(feature = "runtime-select")]
pub mod runtime;
pub mod types;

pub use error::{Error, Result};
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
    PolygonTriangulationAlgorithm, PolygonTriangulationPlan, QualityPolicy, TriangulationOptions,
    plan_polygon_triangulation, triangulate_polygon,
};
pub use types::{
    Constraint, ExactPoint, Point2, PolygonInput, PolygonInputFacts, Rational, Real, RingConvexity,
    RingInputFacts, Sign, Triangle, TriangleIndices, TriangleLocation,
};

/// Triangulate an exact polygon with the earcut-style algorithm.
#[cfg(feature = "earcut")]
pub fn earcut(vertices: &[ExactPoint], hole_indices: &[usize]) -> Result<TriangleIndices> {
    earcut::triangulate(vertices, hole_indices)
}

/// Triangulate an exact polygon with earcut-style diagnostics.
#[cfg(feature = "earcut")]
pub fn earcut_report(
    vertices: &[ExactPoint],
    hole_indices: &[usize],
) -> Result<earcut::EarcutReport> {
    earcut::triangulate_report(vertices, hole_indices)
}
