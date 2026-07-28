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
//! Exact geometric systems need more than a large scalar type: expressions and
//! objects must preserve enough structure that combinatorial decisions are not
//! inferred from fixed-precision approximations. `hypervoxel` applies that
//! principle to grids: a voxel parent is not an averaged material value but a
//! conservative fact packet over its children. See the repository README for
//! the design references.

mod aabb;
mod address;
mod affine;
mod aggregate;
mod batch;
mod cell;
mod chunk;
mod chunk_diff;
mod chunk_faces;
mod chunk_storage;
mod chunk_support;
mod chunk_surface_mesh;
mod component_row_plan;
mod continuous;
mod differential;
mod distance;
mod error;
mod field;
mod frame;
mod halfspace;
#[cfg(feature = "hypermesh-adapter")]
mod hypermesh_adapter;
mod legacy;
mod lod;
mod material;
mod mesh;
mod path;
mod query;
mod ray_schedule;
mod report;
mod serialize;
mod side_table;
mod solid;
mod sparse_surface_mesh;
mod spatial;
mod storage;
mod support;
mod surface_mesh;
mod surface_topology;
mod svo;
mod svo_surface;
mod transform;
mod triangle_mesh;
mod triangle_row_cache;
mod triangle_solid;
#[cfg(feature = "legacy-voxelis")]
mod voxelis_adapter;
mod voxelize;

pub use aabb::{ExactAabb3, GridAabbHandoff, LatticeAabbHandoff};
pub use address::{CellBounds, VoxelAddress};
pub use affine::ExactAffineTransform;
pub use aggregate::{AggregateCertainty, VoxelAggregateFacts, VoxelOccupancyInterval};
pub use batch::{VoxelEdit, VoxelEditBatch};
pub use cell::{
    FieldSampleId, MaterialRegionId, OccupancyState, ProcessStateId, VoxelCell, VoxelCellReport,
    VoxelPayload,
};
pub use chunk::{ChunkAddress, ChunkLocalAddress, ChunkPageSummary, ChunkShape};
pub use chunk_diff::{ChunkPagedSparseGridDiffReport, diff_chunk_paged_sparse_grids};
pub use chunk_faces::{chunk_paged_greedy_face_patch_plan, extract_chunk_paged_exposed_faces};
pub use chunk_storage::{
    ChunkPagedAabbBroadPhaseReport, ChunkPagedConnectedComponentReport,
    ChunkPagedManhattanBandReport, ChunkPagedRegionAggregateReport, ChunkPagedSparseGrid,
    ChunkPagedSparsePage, ChunkPagedSparsePageReport, ChunkPagedSparseStorageReport,
};
pub use chunk_support::{ChunkPagedSupportMaskReport, classify_chunk_paged_support_mask};
pub use chunk_surface_mesh::chunk_paged_exact_surface_triangle_mesh;
pub use continuous::{
    ContinuousFieldVoxelBatch, ContinuousFieldVoxelCell, continuous_field_address,
};
pub use differential::{SparseGridDiffReport, diff_sparse_grids};
pub use distance::{
    DistanceFieldPreview, DistanceSample, SignedDistanceFieldPreview, SignedDistanceSample,
    sample_manhattan_distance_field, sample_signed_manhattan_distance_field,
};
pub use error::{HypervoxelError, HypervoxelResult};
pub use field::{
    CertifiedFieldBall, CertifiedFieldInterval, CertifiedTensorInterval, CertifiedVectorInterval,
    FieldAggregateFacts, FieldEnvelopeFacts, FieldSampleQuery, query_field_samples,
};
pub use frame::{GridFrame, GridFrameFacts, LengthUnit};
pub use halfspace::{
    ExactHalfSpace, ExactHalfSpaceReport, VoxelHalfSpaceClassifier,
    classify_cell_against_halfspace, voxelize_exact_halfspace,
};
#[cfg(feature = "hypermesh-adapter")]
pub use hypermesh_adapter::adapt_hypermesh_exact_solid;
pub use legacy::{LegacyAdapterKind, LegacyAdapterStatus};
pub use lod::{LodCellSelection, LodSelectionReport, select_lod_cells};
pub use material::{
    MaterialColorLookupReport, MaterialDisplayColor, MaterialDisplayPalette, MaterialRegionQuery,
    lookup_material_display_colors, query_material_regions,
};
pub use mesh::{
    ExactVoxelFace, GreedyFacePatch, GreedyFacePatchPlan, LossyObjExport, LossyQuadMesh,
    VoxelFaceSide, extract_exposed_faces, greedy_face_patch_plan, lossy_obj_from_quad_mesh,
    lossy_quad_mesh_from_faces,
};
pub use path::{
    AddressRay, AddressRayTrace, AddressSegmentTrace, SegmentSweepQuery, sweep_address_segment,
    trace_address_ray, trace_address_segment,
};
pub use query::{
    AabbBroadPhaseCandidate, AabbBroadPhaseQuery, ConnectedComponentQuery, ManhattanDistanceBand,
    NeighborQuery, OccupancyQuery, QueryRegion, voxel_neighbors6,
};
pub use report::{
    BoundaryPolicy, QuantizationPolicy, VoxelPredicateCertificateReport, VoxelizationPolicy,
    VoxelizationReport,
};
pub use serialize::{DeterministicSnapshot, SnapshotFormat};
pub use side_table::{
    FieldSampleRecord, MaterialRegionRecord, ProcessStateRecord, VoxelSideTables,
};
pub use solid::{
    ExactConvexHalfSpaceSet, ExactConvexHalfSpaceSetReport, VoxelConvexClassifier,
    classify_cell_against_convex_halfspace_set, voxelize_exact_convex_halfspace_set,
};
pub use sparse_surface_mesh::sparse_exact_surface_triangle_mesh;
pub use spatial::VoxelSpatialAggregateFacts;
pub use storage::SparseVoxelGrid;
pub use support::{
    SupportCellReport, SupportCellStatus, SupportDirection, SupportMaskReport,
    classify_support_mask,
};
pub use surface_mesh::{
    ExactSurfaceTriangle, ExactVoxelSurfaceTriangleMesh,
    exact_voxel_surface_triangle_mesh_from_faces,
};
pub use surface_topology::{ExactSurfaceEdge, ExactSurfaceFaceKey, ExactSurfaceVertex};
pub use svo::{SvoDagStats, SvoNodeId, SvoVoxelGrid};
pub use svo_surface::{extract_svo_exposed_faces, svo_exact_surface_triangle_mesh};
pub use transform::{AxisPermutationTransform, SignedAxis};
pub use triangle_mesh::{
    ExactTriangle3, ExactTriangle3Report, ExactTriangleSolidMesh, ExactTriangleSolidMeshReport,
    ExactTriangleSurfaceMesh, ExactTriangleSurfaceMeshReport, VoxelTriangleMeshClassifier,
    VoxelTriangleSolidClassifier, classify_cell_against_triangle_solid_mesh,
    classify_cell_against_triangle_surface_mesh, voxelize_exact_triangle_solid_mesh,
    voxelize_exact_triangle_surface_mesh,
};
pub use triangle_solid::{
    ExactTriangleSolid, RayParityAttemptReport, TriangleSolidAdaptiveAxisSweepVoxelizationReport,
    TriangleSolidAxisSweepVoxelizationReport, TriangleSolidCellReport,
    TriangleSolidComponentConsensusVoxelizationReport, TriangleSolidComponentVoxelizationReport,
    TriangleSolidConsensusAxisSweepVoxelizationReport, TriangleSolidVoxelizationReport,
    classify_cell_against_exact_triangle_solid, voxelize_exact_triangle_solid,
    voxelize_exact_triangle_solid_by_adaptive_axis_sweeps,
    voxelize_exact_triangle_solid_by_adaptive_local_component_consensus,
    voxelize_exact_triangle_solid_by_axis_sweeps,
    voxelize_exact_triangle_solid_by_component_consensus,
    voxelize_exact_triangle_solid_by_components,
    voxelize_exact_triangle_solid_by_consensus_axis_sweeps,
    voxelize_exact_triangle_solid_by_local_component_consensus,
};
#[cfg(feature = "legacy-voxelis")]
pub use voxelis_adapter::{
    materialize_legacy_voxelis_u8_chunk_paged_storage,
    materialize_legacy_voxelis_u8_exact_surface_triangle_mesh,
};
pub use voxelize::{ExactBox, ExactBoxReport, VoxelBoxClassifier, voxelize_exact_box};
