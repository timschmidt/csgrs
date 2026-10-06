//! Exact support-mask classification over chunk-paged sparse storage.
//!
//! Support masks are process-planning evidence, not contact mechanics. This
//! module ports the sparse-grid support classifier to [`crate::ChunkPagedSparseGrid`]
//! so support lookups can be page-probed while the exact object facts remain
//! the target and support cell addresses.

use crate::{
    ChunkAddress, ChunkPagedSparseGrid, HypervoxelError, HypervoxelResult, OccupancyState,
    SupportCellReport, SupportCellStatus, SupportDirection, SupportMaskReport, VoxelAddress,
};

/// Page-backed support-mask classification report.
///
/// [`SupportMaskReport`] contains the semantic per-cell support decisions. The
/// remaining fields expose the chunk-page schedule used to make those exact
/// address lookups. Page layout is acceleration evidence only; exact support
/// status comes from retained cells or named absence, not floating
/// overhang/contact heuristics.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedSupportMaskReport {
    /// Semantic support report in deterministic target-address order.
    pub support: SupportMaskReport,
    /// Number of occupied target pages visited.
    pub target_pages: usize,
    /// Explicit target cells inspected, including unknown/lossy blockers.
    pub target_cells: usize,
    /// Target cells whose support point lies on the finite frame boundary.
    pub support_plane_probes: usize,
    /// Support-neighbor probes whose target page existed.
    pub support_page_hits: usize,
    /// Support-neighbor probes whose target page was absent.
    pub support_page_misses: usize,
    /// Support-neighbor probes crossing an integer page boundary.
    pub cross_page_support_probes: usize,
    /// Whether both page backends and the semantic support mask are exact-ready.
    pub exact_paged_support_ready: bool,
}

/// Classifies target cells against a page-backed support mask.
///
/// The target and support grids must share the same finite address depth. Each
/// support lookup is still resolved with [`ChunkPagedSparseGrid::get`], so a
/// present page never certifies support by itself and a missing page only
/// certifies sparse absence for that exact support address.
pub fn classify_chunk_paged_support_mask(
    target: &ChunkPagedSparseGrid,
    support: &ChunkPagedSparseGrid,
    direction: SupportDirection,
) -> HypervoxelResult<ChunkPagedSupportMaskReport> {
    if target.frame().depth() != support.frame().depth() {
        return Err(HypervoxelError::MismatchedAddressDepth {
            left: target.frame().depth(),
            right: support.frame().depth(),
        });
    }

    let mut report = SupportMaskReport {
        direction,
        checked_cells: 0,
        has_checked_cells: false,
        supported_cells: 0,
        unsupported_cells: 0,
        support_plane_cells: 0,
        unknown_cells: 0,
        lossy_cells: 0,
        exact_support_mask_ready: false,
        cells: Vec::new(),
    };
    let mut target_pages = 0_usize;
    let mut target_cells = 0_usize;
    let mut counters = PagedSupportCounters::default();

    for (_, page) in target.pages() {
        target_pages += 1;
        for (address, cell) in page.iter() {
            if cell.occupancy == OccupancyState::Empty {
                continue;
            }
            target_cells += 1;
            report.checked_cells += 1;
            let status = match cell.occupancy {
                OccupancyState::Unknown => SupportCellStatus::Unknown,
                OccupancyState::LossyAdapterValue => SupportCellStatus::Lossy,
                _ => {
                    classify_one_paged(*address, target.shape(), support, direction, &mut counters)?
                }
            };
            match status {
                SupportCellStatus::Supported => report.supported_cells += 1,
                SupportCellStatus::Unsupported => report.unsupported_cells += 1,
                SupportCellStatus::OnSupportPlane => report.support_plane_cells += 1,
                SupportCellStatus::Unknown => report.unknown_cells += 1,
                SupportCellStatus::Lossy => report.lossy_cells += 1,
            }
            report.cells.push(SupportCellReport {
                address: *address,
                status,
            });
        }
    }

    report.has_checked_cells = report.checked_cells > 0;
    report.exact_support_mask_ready = report.has_checked_cells
        && report.unsupported_cells == 0
        && report.unknown_cells == 0
        && report.lossy_cells == 0;
    let exact_paged_support_ready = report.exact_support_mask_ready
        && target.report().exact_chunk_storage_ready
        && support.report().exact_chunk_storage_ready;

    Ok(ChunkPagedSupportMaskReport {
        support: report,
        target_pages,
        target_cells,
        support_plane_probes: counters.support_plane_probes,
        support_page_hits: counters.support_page_hits,
        support_page_misses: counters.support_page_misses,
        cross_page_support_probes: counters.cross_page_support_probes,
        exact_paged_support_ready,
    })
}

#[derive(Default)]
struct PagedSupportCounters {
    support_plane_probes: usize,
    support_page_hits: usize,
    support_page_misses: usize,
    cross_page_support_probes: usize,
}

fn classify_one_paged(
    address: VoxelAddress,
    target_shape: crate::ChunkShape,
    support: &ChunkPagedSparseGrid,
    direction: SupportDirection,
    counters: &mut PagedSupportCounters,
) -> HypervoxelResult<SupportCellStatus> {
    let mut below = address.xyz;
    if direction.sign < 0 {
        if below[direction.axis] == 0 {
            counters.support_plane_probes += 1;
            return Ok(SupportCellStatus::OnSupportPlane);
        }
        below[direction.axis] -= 1;
    } else {
        let cells = 1_u64 << address.depth;
        if below[direction.axis] + 1 >= cells {
            counters.support_plane_probes += 1;
            return Ok(SupportCellStatus::OnSupportPlane);
        }
        below[direction.axis] += 1;
    }

    let support_address = VoxelAddress::new(address.depth, below)?;
    let source_page = ChunkAddress::containing(address, target_shape);
    let support_page = ChunkAddress::containing(support_address, support.shape());
    if support_page != source_page {
        counters.cross_page_support_probes += 1;
    }
    if support.page(support_page).is_some() {
        counters.support_page_hits += 1;
    } else {
        counters.support_page_misses += 1;
    }

    Ok(match support.get(support_address)?.occupancy {
        OccupancyState::Empty => SupportCellStatus::Unsupported,
        OccupancyState::Unknown => SupportCellStatus::Unknown,
        OccupancyState::LossyAdapterValue => SupportCellStatus::Lossy,
        _ => SupportCellStatus::Supported,
    })
}
