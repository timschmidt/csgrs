//! Integer-grid segment and sweep queries.
//!
//! The first path API is deliberately address-space based. It traces an exact
//! integer segment through voxel addresses and reports the cells encountered,
//! keeping the combinatorial path separate from any later metric or controller
//! interpolation. The incremental stepping is the voxel analogue of
//! Bresenham's line rasterization ("Algorithm for computer control of a
//! digital plotter," IBM Systems Journal, 1965), used here under Yap's exact
//! geometric computation rule that grid decisions remain integer/exact until a
//! lossy adapter is explicitly selected.

use crate::{
    HypervoxelError, HypervoxelResult, PreparedVoxelGrid, SparseVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell,
};

/// Deterministic address-space segment trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AddressSegmentTrace {
    /// Segment start address.
    pub start: VoxelAddress,
    /// Segment end address.
    pub end: VoxelAddress,
    /// Addresses visited by the integer-grid segment, including both endpoints.
    pub addresses: Vec<VoxelAddress>,
    /// Whether the segment was traced entirely in exact integer address space.
    pub exact_address_trace_ready: bool,
    /// Whether the trace includes the end address.
    pub reached_end: bool,
}

/// Sparse-grid sweep result along an address-space segment.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SegmentSweepQuery {
    /// Segment trace.
    pub trace: AddressSegmentTrace,
    /// Cells sampled along the trace in the same order as `trace.addresses`.
    pub cells: Vec<VoxelCell>,
    /// Conservative aggregate over sampled cells.
    pub aggregate: VoxelAggregateFacts,
    /// Whether every address-space sample was resolved to a cell value.
    pub exact_sweep_samples_ready: bool,
}

/// Axis-aligned address-space ray.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AddressRay {
    /// Ray start address.
    pub start: VoxelAddress,
    /// Axis index in `0..3`.
    pub axis: usize,
    /// Direction, either `1` or `-1`.
    pub direction: i8,
    /// Maximum number of cells to visit.
    pub max_steps: u64,
}

/// Deterministic address-space ray trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AddressRayTrace {
    /// Ray definition.
    pub ray: AddressRay,
    /// Addresses visited, including the start address.
    pub addresses: Vec<VoxelAddress>,
    /// Whether the ray stopped at the grid boundary.
    pub stopped_at_boundary: bool,
    /// Whether the ray stopped because `max_steps` was reached first.
    pub stopped_at_step_limit: bool,
    /// Whether the trace is exact address-space evidence.
    ///
    /// This is a grid-combinatorial fact, not continuous ray casting. It keeps
    /// the Bresenham-style integer traversal of Bresenham, "Algorithm for
    /// computer control of a digital plotter," *IBM Systems Journal*, 1965,
    /// inside Yap's exact-object boundary for voxel addresses.
    pub exact_address_trace_ready: bool,
}

/// Traces a deterministic integer-grid segment between two addresses.
///
/// This is a conservative path fixture, not a replacement for future exact
/// ray/solid intersection kernels. It is useful for tool-access, process-grid,
/// and regression tests that already live in voxel address space.
pub fn trace_address_segment(
    start: VoxelAddress,
    end: VoxelAddress,
) -> HypervoxelResult<AddressSegmentTrace> {
    if start.depth != end.depth {
        return Err(HypervoxelError::MismatchedAddressDepth {
            left: start.depth,
            right: end.depth,
        });
    }

    let delta = [
        end.xyz[0] as i128 - start.xyz[0] as i128,
        end.xyz[1] as i128 - start.xyz[1] as i128,
        end.xyz[2] as i128 - start.xyz[2] as i128,
    ];
    let steps = delta.iter().map(|v| v.unsigned_abs()).max().unwrap_or(0) as u64;
    let mut addresses = Vec::with_capacity(steps as usize + 1);
    if steps == 0 {
        addresses.push(start);
    } else {
        for step in 0..=steps {
            let xyz = [
                rounded_step(start.xyz[0], delta[0], step, steps)?,
                rounded_step(start.xyz[1], delta[1], step, steps)?,
                rounded_step(start.xyz[2], delta[2], step, steps)?,
            ];
            let address = VoxelAddress::new(start.depth, xyz)?;
            if addresses.last().copied() != Some(address) {
                addresses.push(address);
            }
        }
    }

    Ok(AddressSegmentTrace {
        start,
        end,
        addresses,
        exact_address_trace_ready: true,
        reached_end: true,
    })
}

/// Sweeps through explicitly stored sparse cells along an address-space segment.
pub fn sweep_address_segment(
    prepared: &PreparedVoxelGrid<SparseVoxelGrid>,
    start: VoxelAddress,
    end: VoxelAddress,
) -> HypervoxelResult<SegmentSweepQuery> {
    let trace = trace_address_segment(start, end)?;
    let cells = trace
        .addresses
        .iter()
        .map(|address| prepared.storage.get(*address))
        .collect::<HypervoxelResult<Vec<_>>>()?;
    let aggregate = VoxelAggregateFacts::from_cells(cells.iter());
    Ok(SegmentSweepQuery {
        trace,
        cells,
        aggregate,
        exact_sweep_samples_ready: true,
    })
}

/// Traces an axis-aligned address-space ray until boundary or step limit.
///
/// This is an integer-grid query helper, not continuous ray casting. Exact
/// shape/ray predicates belong in the geometry crates; this function provides
/// deterministic voxel-address traces for process fixtures and broad-phase
/// queries while preserving Yap's separation between combinatorial grid facts
/// and numerical geometry.
pub fn trace_address_ray(ray: AddressRay) -> HypervoxelResult<AddressRayTrace> {
    if ray.axis >= 3 || !matches!(ray.direction, -1 | 1) {
        return Err(HypervoxelError::AddressOverflow);
    }
    let cells = 1_u64 << ray.start.depth;
    let mut addresses = Vec::new();
    let mut current = ray.start;
    let mut stopped_at_boundary = false;
    let mut stopped_at_step_limit = false;
    for step in 0..=ray.max_steps {
        addresses.push(current);
        let mut xyz = current.xyz;
        match ray.direction {
            -1 if xyz[ray.axis] == 0 => {
                stopped_at_boundary = true;
                break;
            }
            -1 => xyz[ray.axis] -= 1,
            1 if xyz[ray.axis] + 1 >= cells => {
                stopped_at_boundary = true;
                break;
            }
            1 => xyz[ray.axis] += 1,
            _ => unreachable!("direction validated above"),
        }
        if step == ray.max_steps {
            stopped_at_step_limit = true;
            break;
        }
        current = VoxelAddress::new(current.depth, xyz)?;
    }
    Ok(AddressRayTrace {
        ray,
        addresses,
        stopped_at_boundary,
        stopped_at_step_limit,
        exact_address_trace_ready: true,
    })
}

fn rounded_step(start: u64, delta: i128, step: u64, steps: u64) -> HypervoxelResult<u64> {
    let numerator = delta
        .checked_mul(step as i128)
        .ok_or(HypervoxelError::AddressOverflow)?;
    let half = (steps / 2) as i128;
    let offset = if numerator >= 0 {
        (numerator + half) / steps as i128
    } else {
        (numerator - half) / steps as i128
    };
    let value = start as i128 + offset;
    u64::try_from(value).map_err(|_| HypervoxelError::AddressOverflow)
}
