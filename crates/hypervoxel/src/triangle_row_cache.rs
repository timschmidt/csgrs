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

#[derive(Clone, Debug, PartialEq)]
struct RetainedAxisRowParity {
    min_axis_coord: u64,
    row: AxisRowParity,
}

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
    rows: BTreeMap<ComponentAxisRowKey, RetainedAxisRowParity>,
}

impl ComponentAxisRowCache {
    /// Return a retained row certificate or compute, retain, and return it for
    /// a component row window.
    ///
    /// The cached row may have been scheduled with an exact lower-bound window:
    /// only row intersections at or after `min_axis_coord` were retained. Such
    /// a certificate is replay-valid for a later request only when the retained
    /// minimum coordinate is less than or equal to the requested minimum. If a
    /// later component needs an earlier row segment, the scheduler recomputes
    /// and replaces the retained row with the broader certificate.
    ///
    /// The boolean return value is `true` only when the row came from retained
    /// cache evidence. A miss runs the caller-provided exact scheduler and
    /// stores its result for later components that reference a compatible row
    /// window.
    pub(crate) fn get_or_insert_window_with<F>(
        &mut self,
        key: ComponentAxisRowKey,
        min_axis_coord: u64,
        compute: F,
    ) -> HypervoxelResult<(AxisRowParity, bool, bool)>
    where
        F: FnOnce() -> HypervoxelResult<AxisRowParity>,
    {
        if let Some(row) = self.rows.get(&key)
            && row.min_axis_coord <= min_axis_coord
        {
            return Ok((row.row.clone(), true, false));
        }

        let broadened = self.rows.contains_key(&key);
        let row = compute()?;
        self.rows.insert(
            key,
            RetainedAxisRowParity {
                min_axis_coord,
                row: row.clone(),
            },
        );
        Ok((row, false, broadened))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn certified() -> AxisRowParity {
        AxisRowParity::Certified {
            parameters: Vec::new(),
        }
    }

    #[test]
    fn window_cache_reuses_only_when_retained_lower_bound_is_broad_enough() {
        let mut cache = ComponentAxisRowCache::default();
        let key = ComponentAxisRowKey::new(0, [2, 3]);

        let (_, hit, broadened) = cache
            .get_or_insert_window_with(key, 5, || Ok(certified()))
            .unwrap();
        assert!(!hit);
        assert!(!broadened);

        let (_, hit, broadened) = cache
            .get_or_insert_window_with(key, 7, || Ok(AxisRowParity::Ambiguous))
            .unwrap();
        assert!(hit);
        assert!(!broadened);

        let (row, hit, broadened) = cache
            .get_or_insert_window_with(key, 4, || Ok(AxisRowParity::Ambiguous))
            .unwrap();
        assert!(!hit);
        assert!(broadened);
        assert_eq!(row, AxisRowParity::Ambiguous);
    }
}
