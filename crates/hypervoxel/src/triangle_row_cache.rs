//! Retained row certificates for component-local triangle-solid schedules.
//!
//! The cache in this module is deliberately an exact replay cache, not a
//! geometric oracle. A row key is just the integer arrangement row
//! `(axis, row_a, row_b)` in the current grid frame, and the cached value is
//! the certified or ambiguous row result produced by exact predicates in
//! [`crate::triangle_prepared`]. This follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997: acceleration may reuse
//! retained object facts, but it must not introduce approximate decisions or
//! unreported topology state.

use std::collections::BTreeMap;

use crate::HypervoxelResult;
use crate::triangle_prepared::AxisRowParity;

/// Integer identity of an axis-parallel row in a voxel frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct ComponentAxisRowKey {
    /// Sweep axis, where `0`, `1`, and `2` are `+X`, `+Y`, and `+Z`.
    pub axis: usize,
    /// First perpendicular grid-row coordinate.
    pub row_a: u64,
    /// Second perpendicular grid-row coordinate.
    pub row_b: u64,
}

impl ComponentAxisRowKey {
    /// Build a component row key from a sweep axis and perpendicular row pair.
    pub(crate) fn new(axis: usize, row: [u64; 2]) -> Self {
        Self {
            axis,
            row_a: row[0],
            row_b: row[1],
        }
    }
}

/// In-memory exact row-certificate cache for one voxelization pass.
#[derive(Clone, Debug, Default)]
pub(crate) struct ComponentAxisRowCache {
    rows: BTreeMap<ComponentAxisRowKey, AxisRowParity>,
}

impl ComponentAxisRowCache {
    /// Return a retained row certificate or compute, retain, and return it.
    ///
    /// The boolean return value is `true` only when the row came from retained
    /// cache evidence. A miss runs the caller-provided exact scheduler and
    /// stores its result for later components that reference the same row.
    pub(crate) fn get_or_insert_with<F>(
        &mut self,
        key: ComponentAxisRowKey,
        compute: F,
    ) -> HypervoxelResult<(AxisRowParity, bool)>
    where
        F: FnOnce() -> HypervoxelResult<AxisRowParity>,
    {
        if let Some(row) = self.rows.get(&key) {
            return Ok((row.clone(), true));
        }

        let row = compute()?;
        self.rows.insert(key, row.clone());
        Ok((row, false))
    }
}
