//! Conservative multi-resolution aggregate facts.
//!
//! This module deliberately diverges from rendering-oriented voxel averaging.
//! Combinatorial facts are first-class results of exact predicates, not values
//! inferred from nearby floating-point samples. A parent cell therefore
//! reports what is proved by its children: all-filled,
//! all-empty, mixed, boundary, unknown, or lossy.

use std::collections::BTreeSet;

use hyperreal::{Rational, Real};

use crate::{
    HypervoxelError, HypervoxelResult, MaterialRegionId, OccupancyState, VoxelCell, VoxelPayload,
};

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

/// Conservative facts for a parent or voxel grid.
#[derive(Clone, Debug, PartialEq)]
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
    /// Certified occupancy interval over represented children.
    pub occupancy_interval: VoxelOccupancyInterval,
    /// Aggregate certainty.
    pub certainty: AggregateCertainty,
}

impl Eq for VoxelAggregateFacts {}

/// Certified occupancy fraction bounds for an aggregate.
///
/// The lower bound counts cells definitely filled. The upper bound counts cells
/// that may be occupied because they are filled, boundary, mixed, unknown, or
/// lossy. This is interval evidence, not an averaged LOD material: an unknown
/// value is enclosed rather than guessed, and combinatorial facts remain
/// explicit.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelOccupancyInterval {
    /// Number of children examined.
    pub total_cells: usize,
    /// Number of cells definitely filled.
    pub definite_filled_cells: usize,
    /// Number of cells that may be occupied.
    pub possible_occupied_cells: usize,
    /// Lower exact occupancy fraction.
    pub lower: Real,
    /// Upper exact occupancy fraction.
    pub upper: Real,
    /// Certainty of the interval evidence.
    pub certainty: AggregateCertainty,
}

impl Eq for VoxelOccupancyInterval {}

impl VoxelOccupancyInterval {
    /// Builds exact rational occupancy bounds from explicit counts.
    pub fn from_counts(
        total_cells: usize,
        definite_filled_cells: usize,
        possible_occupied_cells: usize,
        certainty: AggregateCertainty,
    ) -> Self {
        let (lower, upper, certainty) = if total_cells == 0 {
            // An empty child set proves no occupancy ratio. Keep the full unit
            // interval as explicit unknown evidence rather than certifying a
            // vacuous average as exact. Unproved geometric facts stay outside
            // the exact layer.
            (Real::from(0), Real::from(1), AggregateCertainty::Unknown)
        } else {
            (
                Rational::fraction(definite_filled_cells as i64, total_cells as u64)
                    .expect("positive aggregate denominator")
                    .into(),
                Rational::fraction(possible_occupied_cells as i64, total_cells as u64)
                    .expect("positive aggregate denominator")
                    .into(),
                certainty,
            )
        };
        Self {
            total_cells,
            definite_filled_cells,
            possible_occupied_cells,
            lower,
            upper,
            certainty,
        }
    }

    /// Returns whether the interval collapsed to one exact value.
    pub fn is_point_interval(&self) -> bool {
        self.lower == self.upper
    }
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
        let mut definite_filled_cells = 0_usize;
        let mut possible_occupied_cells = 0_usize;

        for cell in cells {
            child_count += 1;
            all_empty &= cell.occupancy == OccupancyState::Empty;
            all_filled &= cell.occupancy == OccupancyState::Filled;
            has_boundary |= cell.occupancy == OccupancyState::Boundary;
            has_mixed |= cell.occupancy == OccupancyState::Mixed;
            has_unknown |= cell.occupancy == OccupancyState::Unknown;
            has_lossy |= cell.occupancy == OccupancyState::LossyAdapterValue;
            if cell.occupancy == OccupancyState::Filled {
                definite_filled_cells += 1;
            }
            if matches!(
                cell.occupancy,
                OccupancyState::Filled
                    | OccupancyState::Boundary
                    | OccupancyState::Mixed
                    | OccupancyState::Unknown
                    | OccupancyState::LossyAdapterValue
            ) {
                possible_occupied_cells += 1;
            }

            if let VoxelPayload::MaterialRegion(region) = cell.payload {
                material_regions.insert(region);
            }
            if matches!(cell.payload, VoxelPayload::LossyAdapterValue(_)) {
                has_lossy = true;
            }
        }

        let certainty = if child_count == 0 {
            AggregateCertainty::Unknown
        } else if has_lossy {
            AggregateCertainty::Lossy
        } else if has_unknown {
            AggregateCertainty::Unknown
        } else if has_boundary || has_mixed {
            AggregateCertainty::Certified
        } else {
            AggregateCertainty::Exact
        };
        let occupancy_interval = VoxelOccupancyInterval::from_counts(
            child_count,
            definite_filled_cells,
            possible_occupied_cells,
            certainty,
        );

        Self {
            child_count,
            all_empty,
            all_filled: child_count > 0 && all_filled,
            has_boundary,
            has_mixed,
            has_unknown,
            has_lossy,
            material_regions,
            occupancy_interval,
            certainty,
        }
    }

    /// Builds whole-frame aggregate facts from explicitly stored non-empty cells.
    ///
    /// Sparse voxel stores normally omit empty cells. For a finite voxelization
    /// report, those omitted cells are still proved exact-empty when the
    /// classifier visited the whole frame. This constructor records that
    /// evidence without expanding every empty cell, preserving the distinction
    /// between proved combinatorial facts and storage layout.
    pub fn from_explicit_cells_in_frame<'a>(
        total_cells: usize,
        cells: impl IntoIterator<Item = &'a VoxelCell>,
    ) -> HypervoxelResult<Self> {
        let mut explicit_cells = 0_usize;
        let mut has_boundary = false;
        let mut has_mixed = false;
        let mut has_unknown = false;
        let mut has_lossy = false;
        let mut material_regions = BTreeSet::new();
        let mut definite_filled_cells = 0_usize;
        let mut possible_occupied_cells = 0_usize;

        for cell in cells {
            explicit_cells += 1;
            has_boundary |= cell.occupancy == OccupancyState::Boundary;
            has_mixed |= cell.occupancy == OccupancyState::Mixed;
            has_unknown |= cell.occupancy == OccupancyState::Unknown;
            has_lossy |= cell.occupancy == OccupancyState::LossyAdapterValue;
            if cell.occupancy == OccupancyState::Filled {
                definite_filled_cells += 1;
            }
            if matches!(
                cell.occupancy,
                OccupancyState::Filled
                    | OccupancyState::Boundary
                    | OccupancyState::Mixed
                    | OccupancyState::Unknown
                    | OccupancyState::LossyAdapterValue
            ) {
                possible_occupied_cells += 1;
            }

            if let VoxelPayload::MaterialRegion(region) = cell.payload {
                material_regions.insert(region);
            }
            if matches!(cell.payload, VoxelPayload::LossyAdapterValue(_)) {
                has_lossy = true;
            }
        }

        if explicit_cells > total_cells {
            return Err(HypervoxelError::InvalidAggregateSummary {
                total_cells,
                explicit_cells,
            });
        }

        let certainty = if total_cells == 0 {
            AggregateCertainty::Unknown
        } else if has_lossy {
            AggregateCertainty::Lossy
        } else if has_unknown {
            AggregateCertainty::Unknown
        } else if has_boundary || has_mixed {
            AggregateCertainty::Certified
        } else {
            AggregateCertainty::Exact
        };
        let occupancy_interval = VoxelOccupancyInterval::from_counts(
            total_cells,
            definite_filled_cells,
            possible_occupied_cells,
            certainty,
        );

        Ok(Self {
            child_count: total_cells,
            all_empty: total_cells > 0 && possible_occupied_cells == 0,
            all_filled: total_cells > 0 && definite_filled_cells == total_cells,
            has_boundary,
            has_mixed,
            has_unknown,
            has_lossy,
            material_regions,
            occupancy_interval,
            certainty,
        })
    }

    /// Builds aggregate facts for a uniform compressed subtree.
    ///
    /// SVO-DAG storage may collapse millions of equal descendant cells into one
    /// interned leaf. The aggregate still describes the logical subtree, not
    /// the physical node count. Compression may change representation but not
    /// the exact combinatorial facts consumed downstream.
    pub fn from_uniform_cell(total_cells: usize, cell: &VoxelCell) -> Self {
        let has_boundary = cell.occupancy == OccupancyState::Boundary;
        let has_mixed = cell.occupancy == OccupancyState::Mixed;
        let has_unknown = cell.occupancy == OccupancyState::Unknown;
        let has_lossy = cell.occupancy == OccupancyState::LossyAdapterValue
            || matches!(cell.payload, VoxelPayload::LossyAdapterValue(_));
        let mut material_regions = BTreeSet::new();
        if let VoxelPayload::MaterialRegion(region) = cell.payload {
            material_regions.insert(region);
        }
        let definite_filled_cells = if cell.occupancy == OccupancyState::Filled {
            total_cells
        } else {
            0
        };
        let possible_occupied_cells = if matches!(
            cell.occupancy,
            OccupancyState::Filled
                | OccupancyState::Boundary
                | OccupancyState::Mixed
                | OccupancyState::Unknown
                | OccupancyState::LossyAdapterValue
        ) {
            total_cells
        } else {
            0
        };
        let certainty = if total_cells == 0 {
            AggregateCertainty::Unknown
        } else if has_lossy {
            AggregateCertainty::Lossy
        } else if has_unknown {
            AggregateCertainty::Unknown
        } else if has_boundary || has_mixed {
            AggregateCertainty::Certified
        } else {
            AggregateCertainty::Exact
        };
        let occupancy_interval = VoxelOccupancyInterval::from_counts(
            total_cells,
            definite_filled_cells,
            possible_occupied_cells,
            certainty,
        );

        Self {
            child_count: total_cells,
            all_empty: total_cells > 0 && possible_occupied_cells == 0,
            all_filled: total_cells > 0 && definite_filled_cells == total_cells,
            has_boundary,
            has_mixed,
            has_unknown,
            has_lossy,
            material_regions,
            occupancy_interval,
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
        let mut definite_filled_cells = 0_usize;
        let mut possible_occupied_cells = 0_usize;
        let mut interval_certainty = AggregateCertainty::Exact;

        for fact in facts {
            child_count += fact.child_count.max(1);
            all_empty &= fact.all_empty;
            all_filled &= fact.all_filled;
            has_boundary |= fact.has_boundary;
            has_mixed |= fact.has_mixed || !(fact.all_empty || fact.all_filled);
            has_unknown |= fact.has_unknown;
            has_lossy |= fact.has_lossy;
            material_regions.extend(fact.material_regions.iter().copied());
            definite_filled_cells += fact.occupancy_interval.definite_filled_cells;
            possible_occupied_cells += fact.occupancy_interval.possible_occupied_cells;
            interval_certainty =
                max_certainty(interval_certainty, fact.occupancy_interval.certainty);
        }

        let certainty = if child_count == 0 {
            AggregateCertainty::Unknown
        } else if has_lossy {
            AggregateCertainty::Lossy
        } else if has_unknown {
            AggregateCertainty::Unknown
        } else if has_boundary || has_mixed {
            AggregateCertainty::Certified
        } else {
            AggregateCertainty::Exact
        };
        let occupancy_interval = VoxelOccupancyInterval::from_counts(
            child_count,
            definite_filled_cells,
            possible_occupied_cells,
            max_certainty(certainty, interval_certainty),
        );

        Self {
            child_count,
            all_empty,
            all_filled: child_count > 0 && all_filled,
            has_boundary,
            has_mixed,
            has_unknown,
            has_lossy,
            material_regions,
            occupancy_interval,
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

fn max_certainty(left: AggregateCertainty, right: AggregateCertainty) -> AggregateCertainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

fn certainty_rank(certainty: AggregateCertainty) -> u8 {
    match certainty {
        AggregateCertainty::Exact => 0,
        AggregateCertainty::Certified => 1,
        AggregateCertainty::Unknown => 2,
        AggregateCertainty::Lossy => 3,
    }
}
