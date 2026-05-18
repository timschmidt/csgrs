//! Voxel cell payloads.
//!
//! Payloads are compact IDs by design. The physical meaning of a material,
//! process state, or field sample remains in `hyperphysics`, `hyperparts`, or a
//! process crate; the grid stores exact occupancy and references.

/// Compact material-region handle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MaterialRegionId(pub u32);

/// Compact field-sample handle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FieldSampleId(pub u32);

/// Compact process-state handle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ProcessStateId(pub u32);

/// Conservative occupancy state for a cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum OccupancyState {
    /// The cell is certified empty.
    Empty,
    /// The cell is certified filled.
    Filled,
    /// The cell intersects a source boundary.
    Boundary,
    /// The cell contains multiple child states or material regions.
    Mixed,
    /// The cell could not be classified under the active policy.
    Unknown,
    /// The cell came from a lossy adapter value.
    LossyAdapterValue,
}

/// Compact value stored in a voxel cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum VoxelPayload {
    /// Pure occupancy state.
    Occupancy(OccupancyState),
    /// Material-region reference.
    MaterialRegion(MaterialRegionId),
    /// Field-sample reference.
    FieldSample(FieldSampleId),
    /// Process-state reference.
    ProcessState(ProcessStateId),
    /// Lossy adapter value carried for preview or compatibility only.
    LossyAdapterValue(u32),
}

/// Cell payload plus occupancy classification.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VoxelCell {
    /// Conservative occupancy classification.
    pub occupancy: OccupancyState,
    /// Compact payload.
    pub payload: VoxelPayload,
}

impl VoxelCell {
    /// Creates an empty cell.
    pub fn empty() -> Self {
        Self {
            occupancy: OccupancyState::Empty,
            payload: VoxelPayload::Occupancy(OccupancyState::Empty),
        }
    }

    /// Creates a filled material-region cell.
    pub fn material(region: MaterialRegionId) -> Self {
        Self {
            occupancy: OccupancyState::Filled,
            payload: VoxelPayload::MaterialRegion(region),
        }
    }

    /// Creates a filled field-sample cell.
    pub fn field_sample(sample: FieldSampleId) -> Self {
        Self {
            occupancy: OccupancyState::Filled,
            payload: VoxelPayload::FieldSample(sample),
        }
    }

    /// Creates a filled process-state cell.
    pub fn process_state(state: ProcessStateId) -> Self {
        Self {
            occupancy: OccupancyState::Filled,
            payload: VoxelPayload::ProcessState(state),
        }
    }

    /// Creates a boundary cell.
    pub fn boundary(payload: VoxelPayload) -> Self {
        Self {
            occupancy: OccupancyState::Boundary,
            payload,
        }
    }

    /// Creates an unknown cell.
    pub fn unknown() -> Self {
        Self {
            occupancy: OccupancyState::Unknown,
            payload: VoxelPayload::Occupancy(OccupancyState::Unknown),
        }
    }
}

impl Default for VoxelCell {
    fn default() -> Self {
        Self::empty()
    }
}
