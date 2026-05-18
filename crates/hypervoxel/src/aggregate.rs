//! Conservative multi-resolution aggregate facts.
//!
//! This module is where `hypervoxel` deliberately diverges from
//! rendering-oriented voxel averaging. Yap's exact geometric computation
//! paradigm treats combinatorial facts as first-class results of exact
//! predicates, not values inferred from nearby floating-point samples. A parent
//! cell therefore reports what is proved by its children: all-filled,
//! all-empty, mixed, boundary, unknown, or lossy.

use std::collections::BTreeSet;

use crate::{MaterialRegionId, OccupancyState, VoxelCell, VoxelPayload};

/// Certainty attached to an aggregate fact packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AggregateCertainty {
    /// Facts are exact consequences of child cells or exact predicates.
    Exact,
    /// Facts are certified by an interval/ball enclosure.
    Certified,
    /// Facts include explicit unresolved states.
    Unknown,
    /// Facts include values from a lossy adapter.
    Lossy,
}

/// Conservative facts for a parent or prepared grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelAggregateFacts {
    /// Number of children represented by this aggregate.
    pub child_count: usize,
    /// Whether every child is empty.
    pub all_empty: bool,
    /// Whether every child is filled with no boundary/unknown/lossy state.
    pub all_filled: bool,
    /// Whether any child touches a boundary.
    pub has_boundary: bool,
    /// Whether any child is mixed.
    pub has_mixed: bool,
    /// Whether any child is unknown.
    pub has_unknown: bool,
    /// Whether any child came from a lossy adapter.
    pub has_lossy: bool,
    /// Material regions observed in the aggregate.
    pub material_regions: BTreeSet<MaterialRegionId>,
    /// Aggregate certainty.
    pub certainty: AggregateCertainty,
}

impl VoxelAggregateFacts {
    /// Builds aggregate facts from child cells.
    pub fn from_cells<'a>(cells: impl IntoIterator<Item = &'a VoxelCell>) -> Self {
        let mut child_count = 0;
        let mut all_empty = true;
        let mut all_filled = true;
        let mut has_boundary = false;
        let mut has_mixed = false;
        let mut has_unknown = false;
        let mut has_lossy = false;
        let mut material_regions = BTreeSet::new();

        for cell in cells {
            child_count += 1;
            all_empty &= cell.occupancy == OccupancyState::Empty;
            all_filled &= cell.occupancy == OccupancyState::Filled;
            has_boundary |= cell.occupancy == OccupancyState::Boundary;
            has_mixed |= cell.occupancy == OccupancyState::Mixed;
            has_unknown |= cell.occupancy == OccupancyState::Unknown;
            has_lossy |= cell.occupancy == OccupancyState::LossyAdapterValue;

            if let VoxelPayload::MaterialRegion(region) = cell.payload {
                material_regions.insert(region);
            }
            if matches!(cell.payload, VoxelPayload::LossyAdapterValue(_)) {
                has_lossy = true;
            }
        }

        let certainty = if has_lossy {
            AggregateCertainty::Lossy
        } else if has_unknown {
            AggregateCertainty::Unknown
        } else {
            AggregateCertainty::Exact
        };

        Self {
            child_count,
            all_empty,
            all_filled: child_count > 0 && all_filled,
            has_boundary,
            has_mixed,
            has_unknown,
            has_lossy,
            material_regions,
            certainty,
        }
    }

    /// Builds parent facts from child aggregate packets.
    pub fn from_aggregates<'a>(facts: impl IntoIterator<Item = &'a VoxelAggregateFacts>) -> Self {
        let mut child_count = 0;
        let mut all_empty = true;
        let mut all_filled = true;
        let mut has_boundary = false;
        let mut has_mixed = false;
        let mut has_unknown = false;
        let mut has_lossy = false;
        let mut material_regions = BTreeSet::new();

        for fact in facts {
            child_count += fact.child_count.max(1);
            all_empty &= fact.all_empty;
            all_filled &= fact.all_filled;
            has_boundary |= fact.has_boundary;
            has_mixed |= fact.has_mixed || !(fact.all_empty || fact.all_filled);
            has_unknown |= fact.has_unknown;
            has_lossy |= fact.has_lossy;
            material_regions.extend(fact.material_regions.iter().copied());
        }

        let certainty = if has_lossy {
            AggregateCertainty::Lossy
        } else if has_unknown {
            AggregateCertainty::Unknown
        } else {
            AggregateCertainty::Exact
        };

        Self {
            child_count,
            all_empty,
            all_filled: child_count > 0 && all_filled,
            has_boundary,
            has_mixed,
            has_unknown,
            has_lossy,
            material_regions,
            certainty,
        }
    }

    /// Returns the conservative occupancy state implied by this aggregate.
    pub fn conservative_occupancy(&self) -> OccupancyState {
        if self.has_lossy {
            OccupancyState::LossyAdapterValue
        } else if self.has_unknown || self.child_count == 0 {
            OccupancyState::Unknown
        } else if self.all_empty {
            OccupancyState::Empty
        } else if self.all_filled && self.material_regions.len() <= 1 {
            OccupancyState::Filled
        } else if self.has_boundary {
            OccupancyState::Boundary
        } else {
            OccupancyState::Mixed
        }
    }
}
