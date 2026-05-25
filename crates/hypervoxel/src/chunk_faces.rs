//! Exact exposed-face extraction over chunk-paged sparse storage.
//!
//! The ordinary sparse-grid shell extractor proves exposed voxel faces by
//! checking exact integer neighbors. This module ports that same object-level
//! proof to [`crate::ChunkPagedSparseGrid`], where occupied pages can accelerate
//! absence checks but cannot decide shell topology by themselves.

use crate::{
    ChunkAddress, ChunkPagedSparseGrid, ExactVoxelFace, GreedyFacePatchPlan, HypervoxelError,
    HypervoxelResult, OccupancyState, VoxelAddress, VoxelFaceSide, greedy_face_patch_plan,
};
use std::collections::BTreeSet;

/// Report from exact exposed-face extraction over chunk-paged storage.
///
/// The extracted faces are the same combinatorial voxel-boundary facts as
/// [`crate::ExactFaceExtractionReport`], with additional page-evidence counters
/// that keep storage acceleration visible. This follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7(1-2), 1997: exact object
/// facts are not inferred from a compact representation, and undecided cells
/// remain explicit blockers. The grid-face boundary model is the
/// combinatorial-cell view used by Mäntylä, *An Introduction to Solid Modeling*,
/// Computer Science Press, 1988, where boundary incidence is discrete topology
/// rather than a floating display mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedExactFaceExtractionReport {
    /// Extracted exact exposed faces.
    pub faces: Vec<ExactVoxelFace>,
    /// Number of extracted faces.
    pub exact_faces: usize,
    /// Whether at least one exact face was extracted.
    pub has_exact_faces: bool,
    /// Number of occupied pages visited in deterministic chunk order.
    pub tested_pages: usize,
    /// Number of explicit non-empty cells visited.
    pub tested_cells: usize,
    /// Number of cell sides tested on exact-ready source cells.
    pub tested_sides: usize,
    /// Neighbor-side checks whose target page existed.
    pub page_hits: usize,
    /// Neighbor-side checks whose target page was absent.
    pub page_misses: usize,
    /// Neighbor-side checks that crossed an integer page boundary.
    pub cross_page_sides: usize,
    /// Sides exposed because the neighbor lies outside the finite frame.
    pub frame_boundary_sides: usize,
    /// Stored source cells skipped because their occupancy was unknown.
    pub skipped_unknown_cells: usize,
    /// Stored source cells skipped because their occupancy came from a lossy adapter.
    pub skipped_lossy_cells: usize,
    /// Neighbor sides whose exposure could not be certified because the neighbor was unknown.
    pub unknown_neighbor_sides: usize,
    /// Neighbor sides whose exposure could not be certified because the neighbor was lossy.
    pub lossy_neighbor_sides: usize,
    /// Whether non-empty extracted shell facts can be consumed as exact
    /// page-backed grid-boundary evidence.
    pub exact_paged_shell_ready: bool,
}

/// Page-backed greedy face-patch plan with exact cover replay.
///
/// Greedy rectangle merging is a display/storage compression strategy, not a
/// new boundary predicate. This report therefore keeps both the exact paged
/// shell and the greedy plan, then expands every emitted patch back into exact
/// face keys and compares that set with the extracted shell. The cover audit
/// follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 1997, by validating the compressed representation against
/// retained object facts. The rectangle-generation heuristic is the voxel
/// meshing idea described by Mikola Lysenko, "Meshing in a Minecraft Game,"
/// 2012, but used here only after exact face extraction.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedGreedyFacePatchReport {
    /// Exact page-backed shell consumed by the patch planner.
    pub shell: ChunkPagedExactFaceExtractionReport,
    /// Deterministic greedy patch plan.
    pub plan: GreedyFacePatchPlan,
    /// Number of face slots covered by expanded greedy patches.
    pub patch_area_faces: usize,
    /// Number of expanded patch face slots that repeated a previous patch slot.
    pub duplicate_patch_faces: usize,
    /// Exact shell faces not covered by any expanded patch.
    pub missing_shell_faces: usize,
    /// Expanded patch face slots that were not present in the exact shell.
    pub extra_patch_faces: usize,
    /// Whether the greedy patch plan is a lossless exact cover of the
    /// page-backed shell and the shell itself is exact-ready.
    pub exact_patch_cover_ready: bool,
}

/// Extracts exact exposed faces from a chunk-paged sparse grid.
///
/// This is a page-accelerated shell query, not a greedy meshing pass. Present
/// pages are only candidate containers; every neighbor relation is resolved by
/// exact [`VoxelAddress`] lookup. Missing pages certify sparse absence for that
/// page because [`ChunkPagedSparseGrid`] is built by replaying all explicit
/// source cells into exact integer chunk pages.
pub fn extract_chunk_paged_exposed_faces_with_report(
    grid: &ChunkPagedSparseGrid,
) -> HypervoxelResult<ChunkPagedExactFaceExtractionReport> {
    let mut faces = Vec::new();
    let mut tested_pages = 0_usize;
    let mut tested_cells = 0_usize;
    let mut tested_sides = 0_usize;
    let mut page_hits = 0_usize;
    let mut page_misses = 0_usize;
    let mut cross_page_sides = 0_usize;
    let mut frame_boundary_sides = 0_usize;
    let mut skipped_unknown_cells = 0_usize;
    let mut skipped_lossy_cells = 0_usize;
    let mut unknown_neighbor_sides = 0_usize;
    let mut lossy_neighbor_sides = 0_usize;

    for (_, page) in grid.pages() {
        tested_pages += 1;
        for (address, cell) in page.iter() {
            if cell.occupancy == OccupancyState::Empty {
                continue;
            }
            tested_cells += 1;
            if cell.occupancy == OccupancyState::Unknown {
                skipped_unknown_cells += 1;
                continue;
            }
            if cell.occupancy == OccupancyState::LossyAdapterValue {
                skipped_lossy_cells += 1;
                continue;
            }

            let source_page = ChunkAddress::containing(*address, grid.shape());
            let cell_bounds = address.bounds(grid.frame())?;
            for side in FACE_SIDES {
                tested_sides += 1;
                let Some(neighbor) = neighbor_address(*address, side) else {
                    frame_boundary_sides += 1;
                    faces.push(ExactVoxelFace {
                        address: *address,
                        side,
                        cell_bounds: cell_bounds.clone(),
                    });
                    continue;
                };

                let neighbor_page = ChunkAddress::containing(neighbor, grid.shape());
                if neighbor_page != source_page {
                    cross_page_sides += 1;
                }
                if grid.page(neighbor_page).is_some() {
                    page_hits += 1;
                } else {
                    page_misses += 1;
                }

                match grid.get(neighbor)?.occupancy {
                    OccupancyState::Empty => {
                        faces.push(ExactVoxelFace {
                            address: *address,
                            side,
                            cell_bounds: cell_bounds.clone(),
                        });
                    }
                    OccupancyState::Unknown => unknown_neighbor_sides += 1,
                    OccupancyState::LossyAdapterValue => lossy_neighbor_sides += 1,
                    _ => {}
                }
            }
        }
    }

    let exact_faces = faces.len();
    let has_exact_faces = exact_faces > 0;
    let exact_paged_shell_ready = has_exact_faces
        && grid.report().exact_chunk_storage_ready
        && skipped_unknown_cells == 0
        && skipped_lossy_cells == 0
        && unknown_neighbor_sides == 0
        && lossy_neighbor_sides == 0;
    Ok(ChunkPagedExactFaceExtractionReport {
        faces,
        exact_faces,
        has_exact_faces,
        tested_pages,
        tested_cells,
        tested_sides,
        page_hits,
        page_misses,
        cross_page_sides,
        frame_boundary_sides,
        skipped_unknown_cells,
        skipped_lossy_cells,
        unknown_neighbor_sides,
        lossy_neighbor_sides,
        exact_paged_shell_ready,
    })
}

/// Builds a page-backed greedy face-patch plan and audits its exact cover.
///
/// The returned patches are still exact integer rectangles over voxel face
/// keys. They may be lowered to a lossy display mesh later, but this function
/// does not treat renderer vertices as geometry evidence. Instead it verifies
/// that patch compression covers exactly the same combinatorial faces returned
/// by [`extract_chunk_paged_exposed_faces_with_report`].
pub fn chunk_paged_greedy_face_patch_plan_with_report(
    grid: &ChunkPagedSparseGrid,
    policy: impl Into<String>,
) -> HypervoxelResult<ChunkPagedGreedyFacePatchReport> {
    let shell = extract_chunk_paged_exposed_faces_with_report(grid)?;
    let plan = greedy_face_patch_plan(&shell.faces, policy);
    let shell_keys = shell.faces.iter().map(face_key).collect::<BTreeSet<_>>();
    let mut patch_keys = BTreeSet::new();
    let mut patch_area_faces = 0_usize;
    let mut duplicate_patch_faces = 0_usize;

    for patch in &plan.patches {
        for u in patch.u_min..patch.u_max {
            for v in patch.v_min..patch.v_max {
                patch_area_faces = patch_area_faces
                    .checked_add(1)
                    .ok_or(HypervoxelError::AddressOverflow)?;
                let key = FaceKey {
                    side: patch.side,
                    depth: patch.depth,
                    plane: patch.plane,
                    u,
                    v,
                };
                if !patch_keys.insert(key) {
                    duplicate_patch_faces += 1;
                }
            }
        }
    }

    let missing_shell_faces = shell_keys.difference(&patch_keys).count();
    let extra_patch_faces = patch_keys.difference(&shell_keys).count();
    let exact_patch_cover_ready = shell.exact_paged_shell_ready
        && plan.exact_faces == shell.exact_faces
        && patch_area_faces == shell.exact_faces
        && duplicate_patch_faces == 0
        && missing_shell_faces == 0
        && extra_patch_faces == 0;

    Ok(ChunkPagedGreedyFacePatchReport {
        shell,
        plan,
        patch_area_faces,
        duplicate_patch_faces,
        missing_shell_faces,
        extra_patch_faces,
        exact_patch_cover_ready,
    })
}

const FACE_SIDES: [VoxelFaceSide; 6] = [
    VoxelFaceSide::XNeg,
    VoxelFaceSide::XPos,
    VoxelFaceSide::YNeg,
    VoxelFaceSide::YPos,
    VoxelFaceSide::ZNeg,
    VoxelFaceSide::ZPos,
];

fn neighbor_address(address: VoxelAddress, side: VoxelFaceSide) -> Option<VoxelAddress> {
    let cells = 1_u64 << address.depth;
    let offset = side.integer_normal();
    let mut xyz = address.xyz;
    for axis in 0..3 {
        match offset[axis] {
            -1 if xyz[axis] == 0 => return None,
            -1 => xyz[axis] -= 1,
            1 if xyz[axis] + 1 >= cells => return None,
            1 => xyz[axis] += 1,
            _ => {}
        }
    }
    VoxelAddress::new(address.depth, xyz).ok()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct FaceKey {
    side: VoxelFaceSide,
    depth: u8,
    plane: u64,
    u: u64,
    v: u64,
}

fn face_key(face: &ExactVoxelFace) -> FaceKey {
    let [x, y, z] = face.address.xyz;
    let (plane, u, v) = match face.side {
        VoxelFaceSide::XNeg => (x, y, z),
        VoxelFaceSide::XPos => (x + 1, y, z),
        VoxelFaceSide::YNeg => (y, x, z),
        VoxelFaceSide::YPos => (y + 1, x, z),
        VoxelFaceSide::ZNeg => (z, x, y),
        VoxelFaceSide::ZPos => (z + 1, x, y),
    };
    FaceKey {
        side: face.side,
        depth: face.address.depth,
        plane,
        u,
        v,
    }
}
