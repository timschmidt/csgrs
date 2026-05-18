//! Exact containment and no-overlap feasibility replay.
//!
//! Packing heuristics are proposal engines. This module turns proposed
//! placements into exact reports: containment, no-overlap, one-placement-per-
//! item accounting, and exact objective summaries. That follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997: a
//! heuristic layout is accepted only after exact/certified replay, otherwise
//! the result remains infeasible or explicitly unknown. The volume accounting
//! is also the first simple lower-bound/objective surface common in
//! Martello/Vigo-style rectangular packing work: total occupied volume and bin
//! waste are evidence, not a proof of global optimality.

use std::collections::BTreeMap;

use hyperreal::{Real, RealSign};

use crate::{Bin3, Item3, ItemId, PackError, PackResult, Placement3};

/// Replay status for a proposed packing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeasibilityStatus {
    /// All exact checks passed.
    Feasible,
    /// An exact containment or overlap check failed.
    Infeasible,
    /// A comparison could not be certified.
    Unknown,
}

/// Report from exact feasibility replay.
#[derive(Clone, Debug, PartialEq)]
pub struct FeasibilityReplay3 {
    /// Overall status.
    pub status: FeasibilityStatus,
    /// Number of containment checks.
    pub containment_checks: usize,
    /// Number of pairwise no-overlap checks.
    pub no_overlap_checks: usize,
    /// Human-readable facts.
    pub facts: Vec<String>,
}

/// Exact objective summary for one-bin 3D packing replay.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjectiveReport3 {
    /// Exact bin volume.
    pub bin_volume: Real,
    /// Sum of placed item volumes.
    pub used_volume: Real,
    /// Exact `bin_volume - used_volume`.
    pub waste_volume: Real,
    /// Number of item ids placed at least once.
    pub placed_items: usize,
    /// Number of item ids not placed.
    pub unplaced_items: usize,
    /// Number of duplicate placement records beyond the first placement.
    pub duplicate_placements: usize,
}

/// Full one-bin packing verification report.
#[derive(Clone, Debug, PartialEq)]
pub struct PackingVerification3 {
    /// Exact geometric feasibility replay.
    pub feasibility: FeasibilityReplay3,
    /// Exact objective/accounting replay.
    pub objective: ObjectiveReport3,
    /// Item ids that were not placed.
    pub unplaced: Vec<ItemId>,
    /// Item ids that appeared in more than one placement.
    pub duplicates: Vec<ItemId>,
}

impl FeasibilityReplay3 {
    /// Replays placements against exact bin containment and pairwise no-overlap.
    pub fn replay(bin: &Bin3, items: &[Item3], placements: &[Placement3]) -> PackResult<Self> {
        let item_map = items
            .iter()
            .map(|item| (item.id.clone(), item))
            .collect::<BTreeMap<ItemId, &Item3>>();
        let mut facts = Vec::new();
        let mut containment_checks = 0;
        let mut no_overlap_checks = 0;

        for placement in placements {
            let item = item_map
                .get(&placement.item)
                .ok_or(PackError::MissingItem)?;
            containment_checks += 1;
            match contains(bin, item, placement) {
                Some(true) => {}
                Some(false) => {
                    facts.push(format!("{} outside bin", placement.item.as_str()));
                    return Ok(Self {
                        status: FeasibilityStatus::Infeasible,
                        containment_checks,
                        no_overlap_checks,
                        facts,
                    });
                }
                None => {
                    return Ok(Self {
                        status: FeasibilityStatus::Unknown,
                        containment_checks,
                        no_overlap_checks,
                        facts,
                    });
                }
            }
        }

        for left_index in 0..placements.len() {
            for right_index in (left_index + 1)..placements.len() {
                let left = &placements[left_index];
                let right = &placements[right_index];
                let left_item = item_map.get(&left.item).ok_or(PackError::MissingItem)?;
                let right_item = item_map.get(&right.item).ok_or(PackError::MissingItem)?;
                no_overlap_checks += 1;
                match disjoint(left_item, left, right_item, right) {
                    Some(true) => {}
                    Some(false) => {
                        facts.push(format!(
                            "{} overlaps {}",
                            left.item.as_str(),
                            right.item.as_str()
                        ));
                        return Ok(Self {
                            status: FeasibilityStatus::Infeasible,
                            containment_checks,
                            no_overlap_checks,
                            facts,
                        });
                    }
                    None => {
                        return Ok(Self {
                            status: FeasibilityStatus::Unknown,
                            containment_checks,
                            no_overlap_checks,
                            facts,
                        });
                    }
                }
            }
        }

        Ok(Self {
            status: FeasibilityStatus::Feasible,
            containment_checks,
            no_overlap_checks,
            facts,
        })
    }
}

/// Verifies one-bin 3D packing geometry and item accounting.
///
/// Existing [`FeasibilityReplay3::replay`] checks geometry. This wrapper also
/// enforces the current core model's implicit quantity-one item semantics:
/// every declared item should appear exactly once. Duplicate placements are
/// infeasible even if their boxes do not overlap, because they claim the same
/// source item twice.
pub fn verify_packing_3d(
    bin: &Bin3,
    items: &[Item3],
    placements: &[Placement3],
) -> PackResult<PackingVerification3> {
    let mut placement_counts = BTreeMap::<ItemId, usize>::new();
    for placement in placements {
        *placement_counts.entry(placement.item.clone()).or_default() += 1;
    }

    let mut unplaced = Vec::new();
    let mut duplicates = Vec::new();
    let mut used_volume = Real::zero();
    let mut placed_items = 0_usize;
    for item in items {
        match placement_counts.get(&item.id).copied().unwrap_or(0) {
            0 => unplaced.push(item.id.clone()),
            1 => {
                placed_items += 1;
                used_volume = used_volume + item.size.volume();
            }
            count => {
                duplicates.push(item.id.clone());
                placed_items += 1;
                used_volume = used_volume + item.size.volume() * Real::from(count as i64);
            }
        }
    }

    let mut feasibility = FeasibilityReplay3::replay(bin, items, placements)?;
    if !duplicates.is_empty() {
        feasibility.status = FeasibilityStatus::Infeasible;
        for duplicate in &duplicates {
            feasibility
                .facts
                .push(format!("{} placed more than once", duplicate.as_str()));
        }
    }
    let bin_volume = bin.size.volume();
    let objective = ObjectiveReport3 {
        waste_volume: bin_volume.clone() - used_volume.clone(),
        bin_volume,
        used_volume,
        placed_items,
        unplaced_items: unplaced.len(),
        duplicate_placements: placement_counts
            .values()
            .map(|count| count.saturating_sub(1))
            .sum(),
    };
    Ok(PackingVerification3 {
        feasibility,
        objective,
        unplaced,
        duplicates,
    })
}

fn contains(bin: &Bin3, item: &Item3, placement: &Placement3) -> Option<bool> {
    Some(
        nonnegative(&placement.x)?
            && nonnegative(&placement.y)?
            && nonnegative(&placement.z)?
            && leq(&(placement.x.clone() + item.size.x.clone()), &bin.size.x)?
            && leq(&(placement.y.clone() + item.size.y.clone()), &bin.size.y)?
            && leq(&(placement.z.clone() + item.size.z.clone()), &bin.size.z)?,
    )
}

fn disjoint(
    left_item: &Item3,
    left: &Placement3,
    right_item: &Item3,
    right: &Placement3,
) -> Option<bool> {
    Some(
        leq(&(left.x.clone() + left_item.size.x.clone()), &right.x)?
            || leq(&(right.x.clone() + right_item.size.x.clone()), &left.x)?
            || leq(&(left.y.clone() + left_item.size.y.clone()), &right.y)?
            || leq(&(right.y.clone() + right_item.size.y.clone()), &left.y)?
            || leq(&(left.z.clone() + left_item.size.z.clone()), &right.z)?
            || leq(&(right.z.clone() + right_item.size.z.clone()), &left.z)?,
    )
}

fn nonnegative(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Negative => Some(false),
        RealSign::Zero | RealSign::Positive => Some(true),
    }
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative | RealSign::Zero => Some(true),
        RealSign::Positive => Some(false),
    }
}
