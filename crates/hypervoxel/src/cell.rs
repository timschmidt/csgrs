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

/// Semantic report for a [`VoxelCell`].
///
/// `VoxelCell` is intentionally a compact public value, so callers may build
/// cells directly when importing side-table IDs or legacy fixtures. This report
/// makes the exactness boundary explicit: a cell whose occupancy and payload do
/// not agree is not exact voxel evidence, and unknown/lossy cells stay visible
/// instead of being repaired by convention. That follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7(1-2), 1997, where object
/// structure and uncertainty must remain part of the geometric state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxelCellReport {
    /// Occupancy state carried by the cell.
    pub occupancy: OccupancyState,
    /// Whether the payload is semantically compatible with the occupancy.
    pub payload_matches_occupancy: bool,
    /// Whether the cell explicitly carries unknown evidence.
    pub has_unknown: bool,
    /// Whether the cell explicitly carries lossy adapter evidence.
    pub has_lossy: bool,
    /// Whether this cell can be consumed as exact cell evidence.
    pub exact_cell_evidence_ready: bool,
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

    /// Creates a lossy adapter cell for preview or compatibility evidence.
    pub fn lossy_adapter_value(id: u32) -> Self {
        Self {
            occupancy: OccupancyState::LossyAdapterValue,
            payload: VoxelPayload::LossyAdapterValue(id),
        }
    }

    /// Reports whether this cell's payload and occupancy are exact-ready.
    pub fn report(&self) -> VoxelCellReport {
        let payload_matches_occupancy = match (self.occupancy, self.payload) {
            (OccupancyState::Empty, VoxelPayload::Occupancy(OccupancyState::Empty)) => true,
            (OccupancyState::Filled, VoxelPayload::Occupancy(OccupancyState::Filled))
            | (OccupancyState::Filled, VoxelPayload::MaterialRegion(_))
            | (OccupancyState::Filled, VoxelPayload::FieldSample(_))
            | (OccupancyState::Filled, VoxelPayload::ProcessState(_)) => true,
            (OccupancyState::Boundary, VoxelPayload::Occupancy(OccupancyState::Boundary))
            | (OccupancyState::Boundary, VoxelPayload::MaterialRegion(_))
            | (OccupancyState::Boundary, VoxelPayload::FieldSample(_))
            | (OccupancyState::Boundary, VoxelPayload::ProcessState(_)) => true,
            (OccupancyState::Mixed, VoxelPayload::Occupancy(OccupancyState::Mixed)) => true,
            (OccupancyState::Unknown, VoxelPayload::Occupancy(OccupancyState::Unknown)) => true,
            (OccupancyState::LossyAdapterValue, VoxelPayload::LossyAdapterValue(_)) => true,
            _ => false,
        };
        let has_unknown = self.occupancy == OccupancyState::Unknown
            || matches!(
                self.payload,
                VoxelPayload::Occupancy(OccupancyState::Unknown)
            );
        let has_lossy = self.occupancy == OccupancyState::LossyAdapterValue
            || matches!(self.payload, VoxelPayload::LossyAdapterValue(_));
        VoxelCellReport {
            occupancy: self.occupancy,
            payload_matches_occupancy,
            has_unknown,
            has_lossy,
            exact_cell_evidence_ready: payload_matches_occupancy && !has_unknown && !has_lossy,
        }
    }
}

impl Default for VoxelCell {
    fn default() -> Self {
        Self::empty()
    }
}
