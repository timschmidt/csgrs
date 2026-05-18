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
    HypervoxelResult, LegacyAdapterKind, LegacyAdapterStatus, MaterialRegionId, SparseVoxelGrid,
    VoxelAddress, VoxelCell,
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

fn legacy_u8_cell(value: u8) -> VoxelCell {
    if value == 0 {
        VoxelCell::empty()
    } else {
        VoxelCell::material(MaterialRegionId(u32::from(value)))
    }
}
