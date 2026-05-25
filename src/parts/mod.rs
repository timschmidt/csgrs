//! Part-interface metadata and assembly documentation carriers.
//!
//! This module is the `csgrs` side of the Hyper part boundary.  Geometry stays
//! in CSG/Profile/Mesh carriers, while source-attributed part facts are attached
//! as ordinary metadata that can later be indexed by `hyperparts`.
//!
//! The trust boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): an exploded diagram,
//! blueprint, or part handoff is useful only when it states which geometry facts
//! were exact, which were preview-only, and which could not be decided.

mod blueprint;
mod metadata;

pub use blueprint::{
    BlueprintEdge, BlueprintEdgeStyle, BlueprintOcclusionStatus, BlueprintProjection,
    BlueprintReport, BlueprintView, OcclusionEvidence, ProjectedPoint2, ProjectedRect,
    blueprint_from_aabb_parts,
};
pub use metadata::{
    AnchorFrame, AssemblyDocumentation, AssemblyFlag, CsgPartInterface, ExactVector3,
    GeometryCertainty, InstallationVector, InterfaceAspect, InterfaceKind, MaterialRegion,
    PartMetadata, PartSource, PartTerminal, PortFrame, SourceCertainty,
};
