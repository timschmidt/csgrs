//! Feature-gated semantic comparison against harvested `voxelis` storage.
//!
//! The compatibility feature is deliberately narrow: it samples a legacy
//! `voxelis::VoxTree<u8>` and compares the resulting default/nonzero cell
//! semantics against a Hyper [`SparseVoxelGrid`]. It does not make `voxelis`
//! the owner of Hyper's grid frame, source geometry, voxelization predicates, or
//! material laws.
//!
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997, pp. 3-23: an approximate or legacy implementation
//! can be useful test evidence only when the compared object-level facts are
//! named explicitly. Storage agreement is therefore reported separately from
//! source-geometry or voxelization truth.

use glam::IVec3;
use voxelis::{
    Lod, VoxInterner,
    spatial::{VoxOpsConfig, VoxOpsRead, VoxTree},
};

use crate::{
    ChunkPagedExactSurfaceTriangleMeshReport, ChunkPagedSparseGrid, ChunkPagedSparseStorageReport,
    ChunkShape, GridFrame, HypervoxelError, HypervoxelResult, LegacyAdapterKind,
    LegacyAdapterStatus, MaterialRegionId, SparseVoxelGrid, VoxelAddress, VoxelCell,
    chunk_paged_exact_surface_triangle_mesh_with_report,
};

/// Sampled semantic comparison between a legacy `voxelis` tree and a Hyper grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyVoxelisStorageDiffReport {
    /// Hyper frame depth used for the comparison.
    pub frame_depth: u8,
    /// Legacy tree depth reported through `voxelis`' own configuration API.
    pub legacy_depth: u8,
    /// Whether the legacy tree and Hyper grid describe the same leaf depth.
    pub legacy_depth_matches_frame: bool,
    /// Number of sample addresses supplied by the caller.
    pub sampled_addresses: usize,
    /// Number of sample addresses actually compared.
    pub compared_addresses: usize,
    /// Whether at least one sample was compared.
    ///
    /// Two empty sample sets are not evidence that a legacy backend matches the
    /// Hyper model. Yap's EGC contract requires positive object evidence before
    /// a representation can be admitted as preserving exact facts.
    pub has_compared_addresses: bool,
    /// Addresses skipped because they were not full-resolution frame leaves.
    pub skipped_non_leaf_addresses: Vec<VoxelAddress>,
    /// Addresses whose mapped legacy cell differs from the Hyper grid cell.
    pub differing_cells: Vec<VoxelAddress>,
    /// Total number of depth, sample, or cell mismatches.
    pub mismatch_count: usize,
    /// Explicit adapter status for the harvested legacy storage comparison.
    pub adapter: LegacyAdapterStatus,
    /// Whether sampled storage semantics matched non-vacuously.
    ///
    /// This is a storage-port readiness bit only. It does not claim exact
    /// source-geometry replay or exact voxelization, because `voxelis`' public
    /// model uses primitive world sizes and rendering/game LOD semantics.
    pub sampled_storage_equivalence_ready: bool,
    /// Whether the sampled legacy comparison can stand in for exact voxelization.
    ///
    /// This remains false by construction. Legacy storage can provide
    /// differential evidence for a port, but exact voxelization belongs to the
    /// Hyper predicate/report path.
    pub exact_voxelization_ready: bool,
}

/// Exhaustive Hyper chunk-paged materialization report for a legacy `voxelis` tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyVoxelisChunkPagedMaterializationReport {
    /// Hyper frame depth requested by the caller.
    pub frame_depth: u8,
    /// Legacy tree depth reported through `voxelis`' own configuration API.
    pub legacy_depth: u8,
    /// Whether the legacy tree and Hyper frame describe the same leaf depth.
    pub legacy_depth_matches_frame: bool,
    /// Number of frame leaf cells enumerated from the legacy tree.
    pub scanned_cells: usize,
    /// Number of legacy zero/default cells interpreted as implicit empty.
    pub empty_cells: usize,
    /// Number of nonzero legacy cells materialized into Hyper sparse storage.
    pub materialized_cells: usize,
    /// Number of nonzero cells whose `u8` value was promoted to a Hyper
    /// material-region id.
    pub material_region_cells: usize,
    /// Number of frame cells replayed through the produced chunk-paged backend.
    pub replayed_cells: usize,
    /// Number of replayed cells whose paged Hyper payload differed from the
    /// legacy value at the same integer address.
    pub paging_mismatch_cells: usize,
    /// Exact chunk-paged storage evidence for the produced Hyper backend.
    pub storage: ChunkPagedSparseStorageReport,
    /// Explicit adapter status for the harvested legacy materialization.
    pub adapter: LegacyAdapterStatus,
    /// Whether the legacy storage was exhaustively ported into Hyper
    /// chunk-paged storage without changing address or payload facts.
    ///
    /// This is a storage-port readiness bit only. It says the `voxelis` SVO-DAG
    /// values have been replayed into Hyper's exact integer page model. It does
    /// not promote the legacy source to exact geometry or exact voxelization.
    pub exhaustive_chunk_port_ready: bool,
    /// Whether the ported legacy storage can stand in for exact voxelization.
    ///
    /// This remains false by construction. Yap's EGC model requires the
    /// source predicates and construction history to be replayed explicitly;
    /// harvested storage values alone are not geometric truth.
    pub exact_voxelization_ready: bool,
}

/// Exact page-backed surface mesh report for materialized legacy `voxelis` storage.
#[derive(Clone, Debug, PartialEq)]
pub struct LegacyVoxelisExactSurfaceTriangleMeshReport {
    /// Exhaustive legacy-to-Hyper chunk-paged storage materialization report.
    pub materialization: LegacyVoxelisChunkPagedMaterializationReport,
    /// Exact page-backed shell, triangle mesh, and vocabulary report over the
    /// materialized Hyper storage.
    pub surface: ChunkPagedExactSurfaceTriangleMeshReport,
    /// Explicit adapter status for the harvested legacy surface handoff.
    pub adapter: LegacyAdapterStatus,
    /// Whether legacy storage was exhaustively ported and the resulting Hyper
    /// pages replayed as exact shared surface-triangle vocabulary.
    ///
    /// This is exact evidence about the materialized integer storage surface,
    /// not about the original geometry that may have produced the legacy tree.
    pub exact_legacy_storage_surface_ready: bool,
    /// Whether this legacy surface handoff can stand in for exact voxelization.
    ///
    /// This remains false by construction. Yap's exact-geometric-computation
    /// contract requires source-geometry predicates and construction replay;
    /// a legacy voxel tree plus a valid storage surface is still not source
    /// voxelization proof.
    pub exact_voxelization_ready: bool,
}

/// Compares sampled `voxelis` `u8` tree values against a Hyper sparse grid.
///
/// `voxelis` numeric voxels use `Default::default()` as empty storage. This
/// adapter maps `0` to [`VoxelCell::empty`] and nonzero values to material-region
/// cells with the same numeric id. That is the narrow semantic overlap useful
/// for storage differentials; richer material meaning stays in Hyper side
/// tables.
pub fn compare_legacy_voxelis_u8_samples<I>(
    tree: &VoxTree<u8>,
    interner: &VoxInterner<u8>,
    expected: &SparseVoxelGrid,
    samples: I,
) -> HypervoxelResult<LegacyVoxelisStorageDiffReport>
where
    I: IntoIterator<Item = VoxelAddress>,
{
    let frame_depth = expected.frame().depth();
    let legacy_depth = tree.max_depth(Lod::new(0)).max();
    let legacy_depth_matches_frame = legacy_depth == frame_depth;
    let mut sampled_addresses = 0_usize;
    let mut compared_addresses = 0_usize;
    let mut skipped_non_leaf_addresses = Vec::new();
    let mut differing_cells = Vec::new();

    for address in samples {
        sampled_addresses += 1;
        if address.depth != frame_depth {
            skipped_non_leaf_addresses.push(address);
            continue;
        }

        let expected_cell = expected.get(address)?;
        let legacy_cell = if legacy_depth_matches_frame {
            let position = IVec3::new(
                address.xyz[0] as i32,
                address.xyz[1] as i32,
                address.xyz[2] as i32,
            );
            legacy_u8_cell(tree.get(interner, position).unwrap_or_default())
        } else {
            VoxelCell::unknown()
        };
        compared_addresses += 1;
        if legacy_cell != expected_cell {
            differing_cells.push(address);
        }
    }

    let mismatch_count = usize::from(!legacy_depth_matches_frame)
        + skipped_non_leaf_addresses.len()
        + differing_cells.len();
    let sampled_storage_equivalence_ready =
        compared_addresses > 0 && legacy_depth_matches_frame && mismatch_count == 0;

    Ok(LegacyVoxelisStorageDiffReport {
        frame_depth,
        legacy_depth,
        legacy_depth_matches_frame,
        sampled_addresses,
        compared_addresses,
        has_compared_addresses: compared_addresses > 0,
        skipped_non_leaf_addresses,
        differing_cells,
        mismatch_count,
        adapter: LegacyAdapterStatus::lossy(
            LegacyAdapterKind::VoxelisStorage,
            "sampled legacy voxelis u8 storage comparison",
        ),
        sampled_storage_equivalence_ready,
        exact_voxelization_ready: false,
    })
}

/// Exhaustively materializes a legacy `voxelis::VoxTree<u8>` into Hyper pages.
///
/// This is the storage-port counterpart to [`compare_legacy_voxelis_u8_samples`].
/// The legacy tree is scanned over every finest-depth frame address, nonzero
/// `u8` values are promoted to [`MaterialRegionId`] payloads, and the resulting
/// [`SparseVoxelGrid`] is immediately lowered into [`ChunkPagedSparseGrid`].
/// The function then replays every frame cell through the paged backend and
/// compares it to the original legacy lookup.
///
/// The design follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997: the performance-oriented `voxelis`
/// SVO-DAG may propose a storage representation, but the Hyper object facts
/// are accepted only after exact integer address replay through the target
/// model. It also mirrors the spatial-subdivision discipline described by
/// Samet, *The Design and Analysis of Spatial Data Structures*,
/// Addison-Wesley, 1990, while keeping page coordinates as integer evidence
/// rather than metric approximations.
pub fn materialize_legacy_voxelis_u8_chunk_paged_storage(
    tree: &VoxTree<u8>,
    interner: &VoxInterner<u8>,
    frame: GridFrame,
    shape: ChunkShape,
) -> HypervoxelResult<(
    ChunkPagedSparseGrid,
    LegacyVoxelisChunkPagedMaterializationReport,
)> {
    let frame_depth = frame.depth();
    let legacy_depth = tree.max_depth(Lod::new(0)).max();
    let legacy_depth_matches_frame = legacy_depth == frame_depth;
    let cells_per_axis = checked_cells_per_axis(frame_depth)?;
    let mut sparse = SparseVoxelGrid::new(frame);
    let mut scanned_cells = 0_usize;
    let mut empty_cells = 0_usize;
    let mut materialized_cells = 0_usize;
    let mut material_region_cells = 0_usize;

    if legacy_depth_matches_frame {
        for z in 0..cells_per_axis {
            for y in 0..cells_per_axis {
                for x in 0..cells_per_axis {
                    scanned_cells += 1;
                    let address = VoxelAddress::new(frame_depth, [x, y, z])?;
                    let legacy = tree
                        .get(interner, ivec3_from_xyz([x, y, z])?)
                        .unwrap_or_default();
                    if legacy == 0 {
                        empty_cells += 1;
                        continue;
                    }
                    sparse.set(address, legacy_u8_cell(legacy))?;
                    materialized_cells += 1;
                    material_region_cells += 1;
                }
            }
        }
    }

    let paged = ChunkPagedSparseGrid::from_sparse_grid(&sparse, shape)?;
    let mut replayed_cells = 0_usize;
    let mut paging_mismatch_cells = 0_usize;
    if legacy_depth_matches_frame {
        for z in 0..cells_per_axis {
            for y in 0..cells_per_axis {
                for x in 0..cells_per_axis {
                    replayed_cells += 1;
                    let address = VoxelAddress::new(frame_depth, [x, y, z])?;
                    let legacy = legacy_u8_cell(
                        tree.get(interner, ivec3_from_xyz([x, y, z])?)
                            .unwrap_or_default(),
                    );
                    if paged.get(address)? != legacy {
                        paging_mismatch_cells += 1;
                    }
                }
            }
        }
    }

    let storage = paged.report().clone();
    let exhaustive_chunk_port_ready = legacy_depth_matches_frame
        && scanned_cells > 0
        && scanned_cells == replayed_cells
        && scanned_cells == logical_frame_cells(frame_depth)?
        && empty_cells + materialized_cells == scanned_cells
        && materialized_cells == storage.summary.stored_cells
        && paging_mismatch_cells == 0
        && storage.exact_chunk_storage_ready;
    let report = LegacyVoxelisChunkPagedMaterializationReport {
        frame_depth,
        legacy_depth,
        legacy_depth_matches_frame,
        scanned_cells,
        empty_cells,
        materialized_cells,
        material_region_cells,
        replayed_cells,
        paging_mismatch_cells,
        storage,
        adapter: LegacyAdapterStatus::lossy(
            LegacyAdapterKind::VoxelisStorage,
            "exhaustive legacy voxelis u8 chunk-paged materialization",
        ),
        exhaustive_chunk_port_ready,
        exact_voxelization_ready: false,
    };
    Ok((paged, report))
}

/// Materializes legacy `voxelis` storage and audits its exact page-backed surface mesh.
///
/// This composes [`materialize_legacy_voxelis_u8_chunk_paged_storage`] with
/// [`crate::chunk_paged_exact_surface_triangle_mesh_with_report`]. The legacy
/// tree is still treated as a lossy adapter boundary, but once its leaf values
/// have been exhaustively replayed into Hyper chunk pages, the exposed surface
/// of that materialized storage can be audited using the same exact shell,
/// triangle, and shared mesh-vocabulary reports as native Hyper pages.
///
/// The acceptance split is deliberate and follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7(1-2), 1997: exact
/// claims must name the object whose facts replay. Here the replayed object is
/// the materialized integer voxel storage surface, not the source geometry
/// that may have produced the legacy `VoxTree`. The indexed surface vocabulary
/// follows Botsch et al., *Polygon Mesh Processing*, AK Peters, 2010, while
/// retaining exact grid-lattice vertices and source voxel-face identities.
pub fn materialize_legacy_voxelis_u8_exact_surface_triangle_mesh(
    tree: &VoxTree<u8>,
    interner: &VoxInterner<u8>,
    frame: GridFrame,
    shape: ChunkShape,
) -> HypervoxelResult<(
    ChunkPagedSparseGrid,
    LegacyVoxelisExactSurfaceTriangleMeshReport,
)> {
    let (paged, materialization) =
        materialize_legacy_voxelis_u8_chunk_paged_storage(tree, interner, frame, shape)?;
    let surface = chunk_paged_exact_surface_triangle_mesh_with_report(&paged)?;
    let exact_legacy_storage_surface_ready = materialization.exhaustive_chunk_port_ready
        && surface.exact_paged_triangle_mesh_ready
        && surface.mesh.report.exact_triangle_surface_mesh_ready
        && surface.vocabulary.exact_shared_mesh_vocabulary_ready;
    let report = LegacyVoxelisExactSurfaceTriangleMeshReport {
        materialization,
        surface,
        adapter: LegacyAdapterStatus::lossy(
            LegacyAdapterKind::VoxelisStorage,
            "legacy voxelis u8 materialized exact surface triangle mesh",
        ),
        exact_legacy_storage_surface_ready,
        exact_voxelization_ready: false,
    };
    Ok((paged, report))
}

fn legacy_u8_cell(value: u8) -> VoxelCell {
    if value == 0 {
        VoxelCell::empty()
    } else {
        VoxelCell::material(MaterialRegionId(u32::from(value)))
    }
}

fn checked_cells_per_axis(depth: u8) -> HypervoxelResult<u64> {
    1_u64
        .checked_shl(u32::from(depth))
        .ok_or(HypervoxelError::AddressOverflow)
}

fn logical_frame_cells(depth: u8) -> HypervoxelResult<usize> {
    usize::try_from(
        checked_cells_per_axis(depth)?
            .checked_pow(3)
            .ok_or(HypervoxelError::AddressOverflow)?,
    )
    .map_err(|_| HypervoxelError::AddressOverflow)
}

fn ivec3_from_xyz(xyz: [u64; 3]) -> HypervoxelResult<IVec3> {
    Ok(IVec3::new(
        i32::try_from(xyz[0]).map_err(|_| HypervoxelError::AddressOverflow)?,
        i32::try_from(xyz[1]).map_err(|_| HypervoxelError::AddressOverflow)?,
        i32::try_from(xyz[2]).map_err(|_| HypervoxelError::AddressOverflow)?,
    ))
}
