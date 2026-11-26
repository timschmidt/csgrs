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
//! Authored metadata labels are separate from kernel predicate receipts and do
//! not establish those facts on their own. Drawing/pose blockers and current
//! validation/reporting limits must be interpreted through each API's contract.

/// Document-local hierarchy, placement traversal and exploded-view records.
mod assembly;
/// Checked attachment bases and rigid mating transforms.
mod attachment;
/// Bounds projections, edge/status records and finite SVG rendering.
mod blueprint;
/// Hierarchical, flat and merged-column bills of materials.
mod bom;
/// Native dimension measurements and projected annotations.
mod dimension;
/// Markdown assembly instructions derived from document steps and BOM rows.
mod manual;
/// Authored part/source/frame declarations and document-local identity labels.
mod metadata;
/// Posed-triangle visibility classification for orthographic drawings.
mod occlude;
/// Shared strict scalar comparisons and native placement arithmetic.
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
