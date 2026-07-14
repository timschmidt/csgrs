//! Exact multi-bin packing replay and cost aggregation.
//!
//! Multi-bin packing separates assignment policy from geometric validity. This
//! module groups proposed placements by bin, delegates each used bin to exact
//! one-bin replay, and aggregates exact objective evidence. Bin assignment and
//! heuristic routing are proposal decisions, while containment, no-overlap, and
//! objective arithmetic stay exact.

use std::collections::{BTreeMap, BTreeSet};

use hyperreal::{Real, RealSign};

use crate::{
    Bin3, FeasibilityStatus, Item3, ItemId, PackError, PackResult, PackingVerification3,
    Placement3, verify_packing_3d,
};

/// Stable bin id for a multi-bin packing proposal.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BinId(String);

/// Exact bin instance with optional cost.
#[derive(Clone, Debug, PartialEq)]
pub struct BinInstance3 {
    /// Stable bin id.
    pub id: BinId,
    /// Exact bin geometry.
    pub bin: Bin3,
    /// Exact nonnegative cost charged when this bin is used.
    pub cost: Real,
}

/// Placement assigned to a named bin.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiBinPlacement3 {
    /// Bin receiving the placement.
    pub bin: BinId,
    /// Placed item id.
    pub item: ItemId,
    /// Exact x origin inside the bin.
    pub x: Real,
    /// Exact y origin inside the bin.
    pub y: Real,
    /// Exact z origin inside the bin.
    pub z: Real,
}

/// Exact aggregate objective for a multi-bin replay.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiBinObjective3 {
    /// Number of bins with at least one placement.
    pub used_bins: usize,
    /// Exact total cost of used bins.
    pub total_cost: Real,
    /// Exact total capacity of used bins.
    pub total_bin_volume: Real,
    /// Exact volume of placed item assignments.
    pub used_volume: Real,
    /// Exact `total_bin_volume - used_volume`.
    pub waste_volume: Real,
    /// Number of item ids placed at least once.
    pub placed_items: usize,
    /// Number of item ids never assigned to any bin.
    pub unplaced_items: usize,
    /// Number of duplicate item assignments beyond the first.
    pub duplicate_assignments: usize,
}

/// Per-bin exact replay payload.
#[derive(Clone, Debug, PartialEq)]
pub struct BinReplay3 {
    /// Bin id.
    pub bin: BinId,
    /// Exact one-bin replay for placements assigned to this bin.
    pub replay: PackingVerification3,
}

/// Full exact multi-bin replay report.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiBinVerification3 {
    /// Overall feasibility status across bin geometry and assignment accounting.
    pub status: FeasibilityStatus,
    /// Per-used-bin replay reports.
    pub bins: Vec<BinReplay3>,
    /// Exact aggregate objective.
    pub objective: MultiBinObjective3,
    /// Item ids never assigned.
    pub unplaced: Vec<ItemId>,
    /// Item ids assigned to more than one bin or placement.
    pub duplicates: Vec<ItemId>,
    /// Human-readable facts.
    pub facts: Vec<String>,
}

impl BinId {
    /// Creates a non-empty bin id.
    pub fn new(value: impl Into<String>) -> PackResult<Self> {
        let value = value.into();
        if value.is_empty() {
            return Err(PackError::EmptyIdentifier);
        }
        Ok(Self(value))
    }

    /// Returns the id text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl BinInstance3 {
    /// Creates a named bin with exact nonnegative cost.
    pub fn new(id: BinId, bin: Bin3, cost: Real) -> PackResult<Self> {
        if negative(&cost).unwrap_or(true) {
            return Err(PackError::NegativeLoadValue);
        }
        Ok(Self { id, bin, cost })
    }
}

/// Replays a fixed multi-bin assignment with exact per-bin geometry.
///
/// This function assumes quantity-one item semantics across the whole
/// assignment: each declared item should appear in exactly one placement in one
/// bin. Per-bin geometry is checked by [`verify_packing_3d`] with only the
/// items assigned to that bin, so an item assigned to two bins is still
/// reported as a duplicate at the aggregate layer.
pub fn verify_multi_bin_packing_3d(
    bins: &[BinInstance3],
    items: &[Item3],
    placements: &[MultiBinPlacement3],
) -> PackResult<MultiBinVerification3> {
    let bin_map = bins
        .iter()
        .map(|bin| (bin.id.clone(), bin))
        .collect::<BTreeMap<BinId, &BinInstance3>>();
    let item_map = items
        .iter()
        .map(|item| (item.id.clone(), item))
        .collect::<BTreeMap<ItemId, &Item3>>();
    let mut by_bin = BTreeMap::<BinId, Vec<Placement3>>::new();
    let mut counts = BTreeMap::<ItemId, usize>::new();
    let mut facts = Vec::new();

    for placement in placements {
        if !bin_map.contains_key(&placement.bin) {
            return Err(PackError::MissingBin);
        }
        if !item_map.contains_key(&placement.item) {
            return Err(PackError::MissingItem);
        }
        *counts.entry(placement.item.clone()).or_default() += 1;
        by_bin
            .entry(placement.bin.clone())
            .or_default()
            .push(Placement3 {
                item: placement.item.clone(),
                x: placement.x.clone(),
                y: placement.y.clone(),
                z: placement.z.clone(),
            });
    }

    let mut unplaced = Vec::new();
    let mut duplicates = Vec::new();
    let mut placed_items = 0_usize;
    let mut used_volume = Real::zero();
    for item in items {
        match counts.get(&item.id).copied().unwrap_or(0) {
            0 => unplaced.push(item.id.clone()),
            1 => {
                placed_items += 1;
                used_volume += item.size.volume();
            }
            count => {
                duplicates.push(item.id.clone());
                placed_items += 1;
                used_volume += item.size.volume() * Real::from(count as i64);
            }
        }
    }

    let mut status = FeasibilityStatus::Feasible;
    let mut bin_reports = Vec::new();
    let mut used_bin_ids = BTreeSet::<BinId>::new();
    let mut total_cost = Real::zero();
    let mut total_bin_volume = Real::zero();
    for (bin_id, bin_placements) in &by_bin {
        let bin = bin_map.get(bin_id).ok_or(PackError::MissingBin)?;
        let assigned_items = bin_placements
            .iter()
            .map(|placement| {
                item_map
                    .get(&placement.item)
                    .copied()
                    .cloned()
                    .ok_or(PackError::MissingItem)
            })
            .collect::<PackResult<Vec<_>>>()?;
        let replay = verify_packing_3d(&bin.bin, &assigned_items, bin_placements)?;
        match replay.feasibility.status {
            FeasibilityStatus::Feasible => {}
            FeasibilityStatus::Infeasible => status = FeasibilityStatus::Infeasible,
            FeasibilityStatus::Unknown if status != FeasibilityStatus::Infeasible => {
                status = FeasibilityStatus::Unknown;
            }
            FeasibilityStatus::Unknown => {}
        }
        for fact in &replay.feasibility.facts {
            facts.push(format!("{}: {fact}", bin_id.as_str()));
        }
        used_bin_ids.insert(bin_id.clone());
        total_cost += bin.cost.clone();
        total_bin_volume += bin.bin.size.volume();
        bin_reports.push(BinReplay3 {
            bin: bin_id.clone(),
            replay,
        });
    }

    if !duplicates.is_empty() {
        status = FeasibilityStatus::Infeasible;
        for duplicate in &duplicates {
            facts.push(format!(
                "{} assigned more than once across bins",
                duplicate.as_str()
            ));
        }
    }

    let objective = MultiBinObjective3 {
        used_bins: used_bin_ids.len(),
        total_cost,
        waste_volume: total_bin_volume.clone() - used_volume.clone(),
        total_bin_volume,
        used_volume,
        placed_items,
        unplaced_items: unplaced.len(),
        duplicate_assignments: counts.values().map(|count| count.saturating_sub(1)).sum(),
    };

    Ok(MultiBinVerification3 {
        status,
        bins: bin_reports,
        objective,
        unplaced,
        duplicates,
        facts,
    })
}

fn negative(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Negative => Some(true),
        RealSign::Zero | RealSign::Positive => Some(false),
    }
}
