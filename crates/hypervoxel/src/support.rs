//! Conservative support-mask reports.
//!
//! Packing, additive manufacturing, and process planning often need support
//! masks, but a voxel support mask is not a replacement for exact contact,
//! load, or stability predicates. This module reports support evidence over
//! integer grid addresses while preserving unknown and lossy cells explicitly.
//! Combinatorial decisions trace to exact object facts or remain reported as
//! uncertainty rather than being inferred from approximate samples.

use crate::{HypervoxelResult, OccupancyState, SparseVoxelGrid, VoxelAddress};

/// Axis-aligned support direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SupportDirection {
    /// Axis index in `0..3`.
    pub axis: usize,
    /// Direction of gravity/load along the axis, either `-1` or `1`.
    pub sign: i8,
}

impl SupportDirection {
    /// Creates a support direction after validating the axis and sign.
    pub fn new(axis: usize, sign: i8) -> HypervoxelResult<Self> {
        if axis >= 3 || !matches!(sign, -1 | 1) {
            return Err(crate::HypervoxelError::InvalidSupportDirection);
        }
        Ok(Self { axis, sign })
    }
}

/// Conservative support status for a target cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SupportCellStatus {
    /// A support cell was explicitly present opposite the load direction.
    Supported,
    /// No support cell was present and the target is not on the support plane.
    Unsupported,
    /// The target is on the domain boundary/support plane.
    OnSupportPlane,
    /// Target or support evidence was unknown.
    Unknown,
    /// Target or support evidence came from a lossy adapter.
    Lossy,
}

/// Per-cell support classification.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupportCellReport {
    /// Target address.
    pub address: VoxelAddress,
    /// Conservative support status.
    pub status: SupportCellStatus,
}

/// Aggregate support-mask report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupportMaskReport {
    /// Support/load direction.
    pub direction: SupportDirection,
    /// Number of checked non-empty target cells.
    pub checked_cells: usize,
    /// Whether at least one non-empty target cell was checked.
    ///
    /// An empty target mask is a precise absence report, but it is not exact
    /// support evidence. Exact decisions need object-level evidence rather
    /// than vacuous count checks.
    pub has_checked_cells: bool,
    /// Number of cells with explicit support.
    pub supported_cells: usize,
    /// Number of cells without support evidence.
    pub unsupported_cells: usize,
    /// Number of cells on the support plane.
    pub support_plane_cells: usize,
    /// Number of cells with explicit unknown evidence.
    pub unknown_cells: usize,
    /// Number of cells with lossy evidence.
    pub lossy_cells: usize,
    /// Whether this mask can be consumed as exact support evidence.
    pub exact_support_mask_ready: bool,
    /// Per-cell reports in deterministic address order.
    pub cells: Vec<SupportCellReport>,
}

impl SupportMaskReport {
    /// Returns whether all checked cells are supported or on the support plane.
    pub fn is_conservatively_supported(&self) -> bool {
        self.exact_support_mask_ready
    }
}

/// Classifies target cells against a support mask on the same integer grid.
pub fn classify_support_mask(
    target: &SparseVoxelGrid,
    support: &SparseVoxelGrid,
    direction: SupportDirection,
) -> HypervoxelResult<SupportMaskReport> {
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

    for (address, cell) in target.iter() {
        if cell.occupancy == OccupancyState::Empty {
            continue;
        }
        report.checked_cells += 1;
        let status = match cell.occupancy {
            OccupancyState::Unknown => SupportCellStatus::Unknown,
            OccupancyState::LossyAdapterValue => SupportCellStatus::Lossy,
            _ => classify_one(*address, support, direction)?,
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

    // This boolean is the support-mask equivalent of the other Hyper report
    // readiness flags: downstream process planners should not infer exact
    // usability from counts by convention. Unsupported, unknown, and lossy
    // cells are all non-ready evidence.
    report.has_checked_cells = report.checked_cells > 0;
    report.exact_support_mask_ready = report.has_checked_cells
        && report.unsupported_cells == 0
        && report.unknown_cells == 0
        && report.lossy_cells == 0;

    Ok(report)
}

fn classify_one(
    address: VoxelAddress,
    support: &SparseVoxelGrid,
    direction: SupportDirection,
) -> HypervoxelResult<SupportCellStatus> {
    let mut below = address.xyz;
    if direction.sign < 0 {
        if below[direction.axis] == 0 {
            return Ok(SupportCellStatus::OnSupportPlane);
        }
        below[direction.axis] -= 1;
    } else {
        let cells = 1_u64 << address.depth;
        if below[direction.axis] + 1 >= cells {
            return Ok(SupportCellStatus::OnSupportPlane);
        }
        below[direction.axis] += 1;
    }

    let support_cell = support.get(VoxelAddress::new(address.depth, below)?)?;
    Ok(match support_cell.occupancy {
        OccupancyState::Empty => SupportCellStatus::Unsupported,
        OccupancyState::Unknown => SupportCellStatus::Unknown,
        OccupancyState::LossyAdapterValue => SupportCellStatus::Lossy,
        _ => SupportCellStatus::Supported,
    })
}
