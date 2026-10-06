//! Exact 1D stock-packing carriers and replay.
//!
//! One-dimensional stock cutting is the smallest useful packing model: items
//! occupy exact intervals on an exact stock length. Containment, no-overlap,
//! and item accounting use exact sign queries, with uncertified comparisons
//! represented as unknown instead of rounded decisions.

use std::collections::BTreeMap;

use hyperreal::{Real, RealSign};

use crate::{FeasibilityStatus, ItemId, PackError, PackResult, model::unique_item_map};

/// Exact one-dimensional stock/bin length.
#[derive(Clone, Debug, PartialEq)]
pub struct StockBin1 {
    /// Exact stock length.
    pub length: Real,
}

/// Exact one-dimensional item length.
#[derive(Clone, Debug, PartialEq)]
pub struct StockItem1 {
    /// Item id.
    pub id: ItemId,
    /// Exact item length.
    pub length: Real,
}

/// Placement of a 1D item by its lower/left coordinate.
#[derive(Clone, Debug, PartialEq)]
pub struct StockPlacement1 {
    /// Placed item id.
    pub item: ItemId,
    /// Exact start coordinate.
    pub start: Real,
}

/// Exact objective summary for one-stock 1D packing replay.
#[derive(Clone, Debug, PartialEq)]
pub struct StockObjective1 {
    /// Exact stock length.
    pub bin_length: Real,
    /// Sum of placed item lengths.
    pub used_length: Real,
    /// Exact `bin_length - used_length`.
    pub waste_length: Real,
    /// Number of item ids placed at least once.
    pub placed_items: usize,
    /// Number of item ids not placed.
    pub unplaced_items: usize,
    /// Number of duplicate placement records beyond the first placement.
    pub duplicate_placements: usize,
}

/// Full one-stock verification report.
#[derive(Clone, Debug, PartialEq)]
pub struct StockVerification1 {
    /// Overall feasibility status.
    pub status: FeasibilityStatus,
    /// Number of containment checks.
    pub containment_checks: usize,
    /// Number of pairwise no-overlap checks.
    pub no_overlap_checks: usize,
    /// Exact objective/accounting replay.
    pub objective: StockObjective1,
    /// Item ids that were not placed.
    pub unplaced: Vec<ItemId>,
    /// Item ids that appeared in more than one placement.
    pub duplicates: Vec<ItemId>,
    /// Human-readable exact facts.
    pub facts: Vec<String>,
}

impl StockBin1 {
    /// Creates a positive exact stock length.
    pub fn new(length: Real) -> PackResult<Self> {
        require_positive(&length)?;
        Ok(Self { length })
    }
}

impl StockItem1 {
    /// Creates a positive exact item length.
    pub fn new(id: ItemId, length: Real) -> PackResult<Self> {
        require_positive(&length)?;
        Ok(Self { id, length })
    }
}

/// Verifies one-stock 1D packing geometry and item accounting.
///
/// The current model uses quantity-one semantics: every declared item should
/// appear exactly once. Duplicate placements are infeasible even if the
/// intervals are disjoint because they claim the same source item twice.
pub fn verify_packing_1d(
    bin: &StockBin1,
    items: &[StockItem1],
    placements: &[StockPlacement1],
) -> PackResult<StockVerification1> {
    let item_map = unique_item_map(items, |item| item.id.clone())?;
    let mut facts = Vec::new();
    let mut containment_checks = 0_usize;
    let mut no_overlap_checks = 0_usize;
    let mut status = FeasibilityStatus::Feasible;
    let mut placement_items = Vec::with_capacity(placements.len());

    for placement in placements {
        let item = item_map
            .get(&placement.item)
            .copied()
            .ok_or(PackError::MissingItem)?;
        placement_items.push(item);
        containment_checks += 1;
        match contains(bin, item, placement) {
            Some(true) => {}
            Some(false) => {
                facts.push(format!("{} outside stock", placement.item.as_str()));
                status = FeasibilityStatus::Infeasible;
                break;
            }
            None => {
                status = FeasibilityStatus::Unknown;
                break;
            }
        }
    }

    if status == FeasibilityStatus::Feasible {
        for left_index in 0..placements.len() {
            for right_index in (left_index + 1)..placements.len() {
                let left = &placements[left_index];
                let right = &placements[right_index];
                no_overlap_checks += 1;
                match disjoint(
                    placement_items[left_index],
                    left,
                    placement_items[right_index],
                    right,
                ) {
                    Some(true) => {}
                    Some(false) => {
                        facts.push(format!(
                            "{} overlaps {}",
                            left.item.as_str(),
                            right.item.as_str()
                        ));
                        status = FeasibilityStatus::Infeasible;
                        break;
                    }
                    None => {
                        status = FeasibilityStatus::Unknown;
                        break;
                    }
                }
            }
            if status != FeasibilityStatus::Feasible {
                break;
            }
        }
    }

    let mut placement_counts = BTreeMap::<ItemId, usize>::new();
    for placement in placements {
        *placement_counts.entry(placement.item.clone()).or_default() += 1;
    }
    let mut unplaced = Vec::new();
    let mut duplicates = Vec::new();
    let mut used_length = Real::zero();
    let mut placed_items = 0_usize;
    for item in items {
        match placement_counts.get(&item.id).copied().unwrap_or(0) {
            0 => unplaced.push(item.id.clone()),
            1 => {
                placed_items += 1;
                used_length += item.length.clone();
            }
            count => {
                duplicates.push(item.id.clone());
                placed_items += 1;
                used_length += item.length.clone() * Real::from(count as i64);
            }
        }
    }
    if !duplicates.is_empty() {
        status = FeasibilityStatus::Infeasible;
        for duplicate in &duplicates {
            facts.push(format!("{} placed more than once", duplicate.as_str()));
        }
    }

    let objective = StockObjective1 {
        waste_length: bin.length.clone() - used_length.clone(),
        bin_length: bin.length.clone(),
        used_length,
        placed_items,
        unplaced_items: unplaced.len(),
        duplicate_placements: placement_counts
            .values()
            .map(|count| count.saturating_sub(1))
            .sum(),
    };

    Ok(StockVerification1 {
        status,
        containment_checks,
        no_overlap_checks,
        objective,
        unplaced,
        duplicates,
        facts,
    })
}

fn contains(bin: &StockBin1, item: &StockItem1, placement: &StockPlacement1) -> Option<bool> {
    crate::predicate::decide_all!(
        nonnegative(&placement.start),
        leq(
            &(placement.start.clone() + item.length.clone()),
            &bin.length,
        ),
    )
}

fn disjoint(
    left_item: &StockItem1,
    left: &StockPlacement1,
    right_item: &StockItem1,
    right: &StockPlacement1,
) -> Option<bool> {
    crate::predicate::decide_any!(
        leq(
            &(left.start.clone() + left_item.length.clone()),
            &right.start,
        ),
        leq(
            &(right.start.clone() + right_item.length.clone()),
            &left.start,
        ),
    )
}

fn require_positive(value: &Real) -> PackResult<()> {
    match crate::predicate::sign(value) {
        Some(RealSign::Positive) => Ok(()),
        Some(RealSign::Negative | RealSign::Zero) | None => Err(PackError::NonPositiveDimension),
    }
}

fn nonnegative(value: &Real) -> Option<bool> {
    match crate::predicate::sign(value)? {
        RealSign::Negative => Some(false),
        RealSign::Zero | RealSign::Positive => Some(true),
    }
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    Some(!crate::predicate::compare(left, right)?.is_gt())
}
