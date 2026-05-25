//! Exact-aware voxel grid frames and conservative sparse-grid facts.
//!
//! `hypervoxel` is the Hyper-owned semantic layer for voxel grids. The
//! harvested `voxelis` code in this repository remains useful for SVO-DAG
//! storage, interning, batching, and mesh fixtures, but this crate owns the
//! exact model: grid frames are expressed with [`hyperreal::Real`], voxel
//! addresses are integer grid addresses, LOD cells carry conservative
//! aggregate facts, and lossy voxelizers or renderers must report their adapter
//! status.
//!
//! The guiding design follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997, pp. 3-23. Yap argues that exact
//! geometric systems need more than a scalar BigNumber layer: expressions and
//! geometric objects must preserve structure so combinatorial decisions are not
//! inferred from fixed-precision approximations. `hypervoxel` applies that
//! rule to grids: a voxel parent is not an averaged material value; it is a
//! conservative object-level fact packet over its children.

mod aabb;
mod adapter;
mod address;
mod affine;
mod aggregate;
mod artifact;
mod audit;
mod batch;
mod candidate;
mod cell;
mod chunk;
mod compression;
mod continuous;
mod coupling;
mod differential;
mod distance;
mod error;
mod export_manifest;
mod field;
mod frame;
mod halfspace;
mod handoff;
#[cfg(feature = "hypermesh-adapter")]
mod hypermesh_adapter;
mod io_manifest;
mod legacy;
mod lod;
mod material;
mod mesh;
mod path;
mod process;
mod query;
mod report;
mod serialize;
mod side_table;
mod solid;
mod spatial;
mod storage;
mod support;
mod svo;
mod trace;
mod transform;
mod triangle_mesh;
mod triangle_prepared;
#[cfg(feature = "legacy-voxelis")]
mod voxelis_adapter;
mod voxelize;

pub use aabb::{ExactAabb3, GridAabbHandoff, LatticeAabbHandoff};
pub use adapter::{
    AdapterNumericContract, AdapterNumericReport, AdapterScalarPrecision, AdapterToleranceStatus,
};
pub use address::{CellBounds, VoxelAddress};
pub use affine::ExactAffineTransform;
pub use aggregate::{AggregateCertainty, VoxelAggregateFacts, VoxelOccupancyInterval};
pub use artifact::{
    VoxelArtifactId, VoxelArtifactManifest, VoxelArtifactReport, VoxelArtifactRole,
};
pub use audit::VoxelizationAudit;
pub use batch::{VoxelEdit, VoxelEditBatch, VoxelEditBatchReport};
pub use candidate::{VoxelCandidateKind, VoxelCandidateManifest, VoxelCandidateReport};
pub use cell::{
    FieldSampleId, MaterialRegionId, OccupancyState, ProcessStateId, VoxelCell, VoxelCellReport,
    VoxelPayload,
};
pub use chunk::{ChunkAddress, ChunkLocalAddress, ChunkPageSummary, ChunkShape};
pub use compression::{
    CompressedStorageKind, CompressedStorageManifest, CompressedStorageReport, StorageReplayStatus,
    VoxelMemoryBudgetManifest, VoxelMemoryBudgetReport,
};
pub use continuous::{
    ContinuousFieldMaterializationBlocker, ContinuousFieldVoxelCell,
    ContinuousFieldVoxelInterchangeManifest, ContinuousFieldVoxelInterchangeReport,
    ContinuousFieldVoxelManifest, ContinuousFieldVoxelReport, ContinuousFieldVoxelRowOrder,
    continuous_field_address,
};
pub use coupling::{VoxelFieldCouplingKind, VoxelFieldCouplingManifest, VoxelFieldCouplingReport};
pub use differential::{SparseGridDiffReport, diff_sparse_grids};
pub use distance::{
    DistanceFieldPreview, DistanceSample, SignedDistanceFieldPreview, SignedDistanceSample,
    sample_manhattan_distance_field, sample_signed_manhattan_distance_field,
};
pub use error::{HypervoxelError, HypervoxelResult};
pub use export_manifest::{
    PreviewExportFormat, PreviewExportManifest, PreviewExportReport, PreviewScalarPolicy,
};
pub use field::{
    CertifiedFieldBall, CertifiedFieldInterval, CertifiedTensorInterval, CertifiedVectorInterval,
    FieldAggregateFacts, FieldEnvelopeFacts, FieldSampleQuery, query_field_samples,
};
pub use frame::{
    GridAxis, GridBasis, GridCoordinateSystem, GridFrame, GridFrameBuilder, GridFrameFacts,
    GridFrameManifest, GridFrameManifestReport, GridHandedness, GridSource, LengthUnit,
};
pub use halfspace::{
    ExactHalfSpace, ExactHalfSpaceReport, VoxelHalfSpaceClassifier,
    classify_cell_against_halfspace, voxelize_exact_halfspace,
};
pub use handoff::{
    SideTableLinkStatus, VoxelHandoffDomain, VoxelHandoffManifest, VoxelHandoffReport,
};
#[cfg(feature = "hypermesh-adapter")]
pub use hypermesh_adapter::{
    HypermeshTriangleSolidAdapter, HypermeshTriangleSolidAdapterBlocker,
    HypermeshTriangleSolidAdapterReport, adapt_hypermesh_exact_solid,
};
pub use io_manifest::{
    ImageStackContainer, ImageStackManifest, VoxelChannelMapping, VoxelIndexConvention,
    VoxelInterchangeFormat, VoxelInterchangeManifest, VoxelIoCompression, VoxelIoMetadata,
    VoxelIoMetadataStatus, VoxelIoPaletteStatus, VoxelIoPayloadStatus, VoxelIoReport,
    VoxelSliceNaming, VoxelSliceOrdering,
};
pub use legacy::{LegacyAdapterKind, LegacyAdapterStatus};
pub use lod::{LodCellSelection, LodSelectionReport, select_lod_cells};
pub use material::{
    MaterialColorLookupReport, MaterialDisplayColor, MaterialDisplayPalette,
    MaterialRegionMetadataReport, MaterialRegionQuery, lookup_material_display_colors,
    query_material_regions, report_material_region_metadata,
};
pub use mesh::{
    ExactFaceExtractionReport, ExactVoxelFace, GreedyFacePatch, GreedyFacePatchPlan,
    LossyMeshExportReport, LossyObjExport, LossyQuadMesh, VoxelFaceSide, extract_exposed_faces,
    extract_exposed_faces_with_report, greedy_face_patch_plan, lossy_obj_from_quad_mesh,
    lossy_quad_mesh_from_faces,
};
pub use path::{
    AddressRay, AddressRayTrace, AddressSegmentTrace, SegmentSweepQuery, sweep_address_segment,
    trace_address_ray, trace_address_segment,
};
pub use process::{ProcessGridArtifact, ProcessGridRole, SweptVolumeProvenance, SweptVolumeReport};
pub use query::{
    AabbBroadPhaseCandidate, AabbBroadPhaseQuery, ConnectedComponentQuery, ManhattanDistanceBand,
    NeighborQuery, OccupancyQuery, PreparedQueryReport, PreparedSparseVoxelGridExt, QueryRegion,
    voxel_neighbors6,
};
pub use report::{
    BoundaryPolicy, FreshnessStatus, PreparedVoxelGrid, QuantizationPolicy,
    VoxelPredicateCertificateReport, VoxelizationPolicy, VoxelizationReport,
};
pub use serialize::{DeterministicSnapshot, DeterministicSnapshotReport, SnapshotFormat};
pub use side_table::{
    FieldSampleRecord, MaterialRegionRecord, ProcessStateRecord, VoxelSideTables,
};
pub use solid::{
    ExactConvexHalfSpaceSet, ExactConvexHalfSpaceSetReport, VoxelConvexClassifier,
    classify_cell_against_convex_halfspace_set, voxelize_exact_convex_halfspace_set,
};
pub use spatial::VoxelSpatialAggregateFacts;
pub use storage::{SparseVoxelGrid, VoxelEditReport};
pub use support::{
    SupportCellReport, SupportCellStatus, SupportDirection, SupportMaskReport,
    classify_support_mask,
};
pub use svo::{SvoDagStats, SvoEditReport, SvoNodeId, SvoStorageReport, SvoVoxelGrid};
pub use trace::{VoxelTraceDimension, VoxelTraceManifest, VoxelTraceReport};
pub use transform::{AxisPermutationTransform, SignedAxis};
pub use triangle_mesh::{
    ExactTriangle3, ExactTriangle3Report, ExactTriangleSolidMesh, ExactTriangleSolidMeshReport,
    ExactTriangleSurfaceMesh, ExactTriangleSurfaceMeshReport, VoxelTriangleMeshClassifier,
    VoxelTriangleSolidClassifier, classify_cell_against_triangle_solid_mesh,
    classify_cell_against_triangle_surface_mesh, voxelize_exact_triangle_solid_mesh,
    voxelize_exact_triangle_surface_mesh,
};
pub use triangle_prepared::{
    PreparedExactTriangle, PreparedExactTriangleSolidMesh, PreparedExactTriangleSolidMeshReport,
    PreparedRayParityAttemptReport, PreparedTriangleSolidCellReport,
    PreparedTriangleSolidVoxelizationReport, classify_cell_against_prepared_triangle_solid_mesh,
    voxelize_prepared_exact_triangle_solid_mesh,
};
#[cfg(feature = "legacy-voxelis")]
pub use voxelis_adapter::{LegacyVoxelisStorageDiffReport, compare_legacy_voxelis_u8_samples};
pub use voxelize::{ExactBox, ExactBoxReport, VoxelBoxClassifier, voxelize_exact_box};
