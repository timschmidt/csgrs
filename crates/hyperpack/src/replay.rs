//! Exact containment and no-overlap feasibility replay.

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
