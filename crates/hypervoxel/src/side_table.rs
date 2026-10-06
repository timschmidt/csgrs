//! Side tables for compact voxel payload IDs.
//!
//! Voxel cells intentionally store small IDs rather than embedding material
//! laws or field data in every octree node. The grid stores combinatorial and
//! aggregate facts while the owning domain crate keeps the richer model.

use std::collections::BTreeMap;

use hyperreal::Real;

use crate::{FieldSampleId, MaterialRegionId, ProcessStateId};

/// Material-region metadata referenced by [`MaterialRegionId`].
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialRegionRecord {
    /// Human-readable label.
    pub label: String,
    /// Optional exact density or domain-owned scalar property.
    pub density: Option<Real>,
}

/// Field-sample metadata referenced by [`FieldSampleId`].
#[derive(Clone, Debug, PartialEq)]
pub struct FieldSampleRecord {
    /// Human-readable label.
    pub label: String,
    /// Exact lower bound for scalar samples, when known.
    pub lower: Option<Real>,
    /// Exact upper bound for scalar samples, when known.
    pub upper: Option<Real>,
}

/// Process-state metadata referenced by [`ProcessStateId`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProcessStateRecord {
    /// Human-readable label.
    pub label: String,
}

/// Side tables for compact voxel payload references.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct VoxelSideTables {
    materials: BTreeMap<MaterialRegionId, MaterialRegionRecord>,
    field_samples: BTreeMap<FieldSampleId, FieldSampleRecord>,
    process_states: BTreeMap<ProcessStateId, ProcessStateRecord>,
}

impl VoxelSideTables {
    /// Inserts a material-region record.
    pub fn insert_material(
        &mut self,
        id: MaterialRegionId,
        record: MaterialRegionRecord,
    ) -> Option<MaterialRegionRecord> {
        self.materials.insert(id, record)
    }

    /// Returns a material-region record.
    pub fn material(&self, id: MaterialRegionId) -> Option<&MaterialRegionRecord> {
        self.materials.get(&id)
    }

    /// Inserts a field-sample record.
    pub fn insert_field_sample(
        &mut self,
        id: FieldSampleId,
        record: FieldSampleRecord,
    ) -> Option<FieldSampleRecord> {
        self.field_samples.insert(id, record)
    }

    /// Returns a field-sample record.
    pub fn field_sample(&self, id: FieldSampleId) -> Option<&FieldSampleRecord> {
        self.field_samples.get(&id)
    }

    /// Inserts a process-state record.
    pub fn insert_process_state(
        &mut self,
        id: ProcessStateId,
        record: ProcessStateRecord,
    ) -> Option<ProcessStateRecord> {
        self.process_states.insert(id, record)
    }

    /// Returns a process-state record.
    pub fn process_state(&self, id: ProcessStateId) -> Option<&ProcessStateRecord> {
        self.process_states.get(&id)
    }

    /// Deterministically iterates material records.
    pub fn materials(&self) -> impl Iterator<Item = (&MaterialRegionId, &MaterialRegionRecord)> {
        self.materials.iter()
    }

    /// Deterministically iterates field-sample records.
    pub fn field_samples(&self) -> impl Iterator<Item = (&FieldSampleId, &FieldSampleRecord)> {
        self.field_samples.iter()
    }

    /// Deterministically iterates process-state records.
    pub fn process_states(&self) -> impl Iterator<Item = (&ProcessStateId, &ProcessStateRecord)> {
        self.process_states.iter()
    }
}
