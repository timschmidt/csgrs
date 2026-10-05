//! Assembly documents, bills of materials, and orthographic hidden-line drawings.
//!
//! Geometry stays in native triangle meshes. Frames, placements, explode offsets,
//! and measured lengths are `hyperlattice` values of `hyperreal::Real`. Prices
//! and procurement stay in `hyperparts`. Circuit and PCB ownership stays outside
//! this module; see `PCB_MIGRATION.md`.
//!
//! The trust boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): an exploded diagram,
//! blueprint, or part handoff states which geometry facts were exact, which were
//! preview-only, and which could not be decided.

mod assembly;
mod attachment;
mod blueprint;
mod bom;
mod dimension;
mod manual;
mod metadata;
mod occlude;
mod scalar;

pub use assembly::{
    AssemblyDocument, AssemblyError, AssemblyNode, AssemblyView, BomPolicy, DocumentationPose,
    ExplodeSpec, GuideSegment, NodeKind, PosedAssembly, PosedPart, instance_count,
    posed_parts,
};
pub use attachment::{AttachmentFrame, AttachmentRole, mate};
#[cfg(feature = "attributed")]
pub use blueprint::blueprint_from_aabb_parts;
pub use blueprint::{
    BlueprintEdge, BlueprintEdgeStyle, BlueprintOcclusionStatus, BlueprintProjection,
    BlueprintReport, BlueprintView, BoundsPart, OcclusionEvidence, ProjectedPoint2,
    ProjectedRect, blueprint_from_bounds, export_blueprint_svg,
};
pub use bom::{
    BomAssembly, BomCategory, BomLine, BomReport, BomSubassembly, bill_of_materials,
};
pub use dimension::{Dimension, annotate, dimension, dimension_x, dimension_y, dimension_z};
pub use manual::{Manual, assembly_manual};
pub use metadata::{
    AnchorFrame, AssemblyDocumentation, AssemblyFlag, CsgPartInterface, GeometryCertainty,
    InstallationVector, InterfaceAspect, InterfaceKind, MaterialRegion, NodeId, PartMetadata,
    PartSource, PartTerminal, PortFrame, SourceCertainty,
};
pub use occlude::blueprint_from_posed_mesh;
