//! Deterministic snapshots for semantic grid artifacts.
//!
//! This is not the final compressed storage format. It is a stable diagnostic
//! and fixture format for exact frame, cell, and aggregate data. A serialized
//! artifact states which object facts it preserves instead of relying on a byte
//! prefix.

use std::fmt::Write;

use crate::{
    FieldSampleId, MaterialRegionId, OccupancyState, ProcessStateId, SparseVoxelGrid, VoxelPayload,
    VoxelSideTables,
};

/// Snapshot format identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapshotFormat {
    /// Human-readable deterministic text.
    TextV1,
    /// Deterministic little-endian binary fixture format.
    BinaryV1,
    /// Deterministic run-length encoded binary fixture format.
    RunLengthBinaryV1,
}

/// Deterministic snapshot output.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeterministicSnapshot {
    /// Format used by `bytes`.
    pub format: SnapshotFormat,
    /// Snapshot bytes.
    pub bytes: Vec<u8>,
}

impl DeterministicSnapshot {
    /// Creates a deterministic text snapshot from a sparse grid and side tables.
    pub fn text_v1(grid: &SparseVoxelGrid, side_tables: &VoxelSideTables) -> Self {
        let mut out = String::new();
        let frame = grid.frame();
        writeln!(&mut out, "hypervoxel-text-v1").unwrap();
        writeln!(&mut out, "depth={}", frame.depth()).unwrap();
        writeln!(&mut out, "units={:?}", frame.units()).unwrap();
        for axis in 0..3 {
            writeln!(&mut out, "origin[{axis}]={}", frame.origin()[axis]).unwrap();
            writeln!(&mut out, "pitch[{axis}]={}", frame.pitch(axis)).unwrap();
        }
        for (address, cell) in grid.iter() {
            writeln!(
                &mut out,
                "cell d={} xyz={},{},{} occ={:?} payload={:?}",
                address.depth,
                address.xyz[0],
                address.xyz[1],
                address.xyz[2],
                cell.occupancy,
                cell.payload
            )
            .unwrap();
        }
        for (id, material) in side_tables.materials() {
            writeln!(
                &mut out,
                "material {} label={:?} density={:?}",
                id.0, material.label, material.density
            )
            .unwrap();
        }
        for (id, sample) in side_tables.field_samples() {
            writeln!(
                &mut out,
                "field_sample {} label={:?} lower={:?} upper={:?}",
                id.0, sample.label, sample.lower, sample.upper
            )
            .unwrap();
        }
        for (id, state) in side_tables.process_states() {
            writeln!(&mut out, "process_state {} label={:?}", id.0, state.label).unwrap();
        }

        Self {
            format: SnapshotFormat::TextV1,
            bytes: out.into_bytes(),
        }
    }

    /// Creates a deterministic binary snapshot from a sparse grid and side tables.
    ///
    /// The format stores exact scalars as canonical display strings and integer
    /// grid addresses as little-endian integers. It is a fixture/interchange
    /// format rather than a compressed store; the important semantic rule is
    /// that no exact scalar is lowered to a primitive float at this boundary.
    pub fn binary_v1(grid: &SparseVoxelGrid, side_tables: &VoxelSideTables) -> Self {
        let mut out = Vec::new();
        out.extend_from_slice(b"HYPERVOXEL-BIN-V1\0");

        let frame = grid.frame();
        write_u8(&mut out, frame.depth());
        write_u8(&mut out, length_unit_tag(frame.units()));
        write_u64(&mut out, frame.cells_per_axis());
        for axis in 0..3 {
            write_string(&mut out, &frame.origin()[axis].to_string());
            write_string(&mut out, &frame.pitch(axis).to_string());
        }

        write_u64(&mut out, grid.len() as u64);
        for (address, cell) in grid.iter() {
            write_u8(&mut out, address.depth);
            for axis in 0..3 {
                write_u64(&mut out, address.xyz[axis]);
            }
            write_u8(&mut out, occupancy_tag(cell.occupancy));
            write_payload(&mut out, cell.payload);
        }

        let materials = side_tables.materials().collect::<Vec<_>>();
        write_u64(&mut out, materials.len() as u64);
        for (id, material) in materials {
            write_u32(&mut out, id.0);
            write_string(&mut out, &material.label);
            match &material.density {
                Some(density) => {
                    write_u8(&mut out, 1);
                    write_string(&mut out, &density.to_string());
                }
                None => write_u8(&mut out, 0),
            }
        }
        let samples = side_tables.field_samples().collect::<Vec<_>>();
        write_u64(&mut out, samples.len() as u64);
        for (id, sample) in samples {
            write_u32(&mut out, id.0);
            write_string(&mut out, &sample.label);
            write_optional_real_string(&mut out, sample.lower.as_ref());
            write_optional_real_string(&mut out, sample.upper.as_ref());
        }
        let states = side_tables.process_states().collect::<Vec<_>>();
        write_u64(&mut out, states.len() as u64);
        for (id, state) in states {
            write_u32(&mut out, id.0);
            write_string(&mut out, &state.label);
        }

        Self {
            format: SnapshotFormat::BinaryV1,
            bytes: out,
        }
    }

    /// Creates a deterministic run-length encoded binary snapshot.
    ///
    /// The compression is intentionally simple and semantic: explicit cells are
    /// sorted by exact `(depth, Morton code)` identity, then adjacent cells with
    /// identical payloads are emitted as runs. This is not the final SVO-DAG
    /// storage format, but it gives fixtures a compact deterministic form while
    /// keeping exact addresses recoverable.
    pub fn run_length_binary_v1(grid: &SparseVoxelGrid) -> Self {
        let mut out = Vec::new();
        out.extend_from_slice(b"HYPERVOXEL-RLE-V1\0");
        write_u8(&mut out, grid.frame().depth());

        let mut cells = grid.iter().collect::<Vec<_>>();
        cells.sort_by_cached_key(|(address, cell)| (address.depth, address.morton_code(), **cell));

        let mut runs = Vec::<Run<'_>>::new();
        for (address, cell) in cells {
            let key = (address.depth, cell);
            match runs.last_mut() {
                Some(run)
                    if run.depth == key.0
                        && run.cell == key.1
                        && run.start_morton + run.len == address.morton_code() =>
                {
                    run.len += 1;
                }
                _ => runs.push(Run {
                    depth: address.depth,
                    start_morton: address.morton_code(),
                    len: 1,
                    cell,
                }),
            }
        }

        write_u64(&mut out, runs.len() as u64);
        for run in runs {
            write_u8(&mut out, run.depth);
            write_u64(&mut out, run.start_morton);
            write_u64(&mut out, run.len);
            write_u8(&mut out, occupancy_tag(run.cell.occupancy));
            write_payload(&mut out, run.cell.payload);
        }

        Self {
            format: SnapshotFormat::RunLengthBinaryV1,
            bytes: out,
        }
    }
}

struct Run<'a> {
    depth: u8,
    start_morton: u64,
    len: u64,
    cell: &'a crate::VoxelCell,
}

fn write_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn write_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn write_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn write_string(out: &mut Vec<u8>, value: &str) {
    write_u64(out, value.len() as u64);
    out.extend_from_slice(value.as_bytes());
}

fn write_optional_real_string(out: &mut Vec<u8>, value: Option<&hyperreal::Real>) {
    match value {
        Some(value) => {
            write_u8(out, 1);
            write_string(out, &value.to_string());
        }
        None => write_u8(out, 0),
    }
}

fn length_unit_tag(units: crate::LengthUnit) -> u8 {
    match units {
        crate::LengthUnit::Unitless => 0,
        crate::LengthUnit::Meter => 1,
        crate::LengthUnit::Millimeter => 2,
        crate::LengthUnit::Micrometer => 3,
        crate::LengthUnit::Nanometer => 4,
    }
}

fn occupancy_tag(occupancy: OccupancyState) -> u8 {
    match occupancy {
        OccupancyState::Empty => 0,
        OccupancyState::Filled => 1,
        OccupancyState::Boundary => 2,
        OccupancyState::Mixed => 3,
        OccupancyState::Unknown => 4,
        OccupancyState::LossyAdapterValue => 5,
    }
}

fn write_payload(out: &mut Vec<u8>, payload: VoxelPayload) {
    match payload {
        VoxelPayload::Occupancy(occupancy) => {
            write_u8(out, 0);
            write_u32(out, u32::from(occupancy_tag(occupancy)));
        }
        VoxelPayload::MaterialRegion(MaterialRegionId(id)) => {
            write_u8(out, 1);
            write_u32(out, id);
        }
        VoxelPayload::FieldSample(FieldSampleId(id)) => {
            write_u8(out, 2);
            write_u32(out, id);
        }
        VoxelPayload::ProcessState(ProcessStateId(id)) => {
            write_u8(out, 3);
            write_u32(out, id);
        }
        VoxelPayload::LossyAdapterValue(id) => {
            write_u8(out, 4);
            write_u32(out, id);
        }
    }
}
