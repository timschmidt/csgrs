//! Exact objective reports beyond feasibility replay.
//!
//! Objective calculations are separate from geometric acceptance. A heuristic
//! may optimize height, cost, waste, or balance, but the resulting layout still
//! needs exact containment/no-overlap replay. Combinatorial decisions and scalar
//! comparisons expose certified evidence or explicit uncertainty instead of
//! relying on primitive-float tolerances.

use hyperreal::{Real, RealSign};
use std::cmp::Ordering;

use crate::{
    Bin3, Item3, PackError, PackResult, PackingVerification3, Placement3, model::unique_item_map,
};

/// Exact used-height objective report for a 3D packing proposal.
#[derive(Clone, Debug, PartialEq)]
pub struct HeightObjective3 {
    /// Number of placement records evaluated.
    pub checked_placements: usize,
    /// Exact bin height.
    pub bin_height: Real,
    /// Exact maximum `placement.z + item.height` if all max comparisons certified.
    pub used_height: Option<Real>,
    /// Exact `bin_height - used_height` when used height is certified.
    pub remaining_height: Option<Real>,
    /// Number of exact max-height comparisons attempted.
    pub exact_comparisons: usize,
    /// Number of max-height comparisons that could not be certified.
    pub unknown_comparisons: usize,
    /// Human-readable objective facts.
    pub facts: Vec<String>,
}

/// One term in an exact lexicographic 3D objective policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObjectiveTerm3 {
    /// Prefer fewer unplaced item ids.
    MinimizeUnplacedItems,
    /// Prefer fewer duplicate placement records.
    MinimizeDuplicatePlacements,
    /// Prefer larger exact used volume.
    MaximizeUsedVolume,
    /// Prefer smaller exact waste volume.
    MinimizeWasteVolume,
    /// Prefer smaller exact used height from [`HeightObjective3`].
    MinimizeUsedHeight,
    /// Prefer larger exact remaining height from [`HeightObjective3`].
    MaximizeRemainingHeight,
}

/// Exact lexicographic comparison report for two 3D packing candidates.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjectiveComparison3 {
    /// Certified ordering of the left candidate relative to the right candidate.
    pub ordering: Option<Ordering>,
    /// First term that decided the ordering, if any.
    pub decisive_term: Option<ObjectiveTerm3>,
    /// Number of policy terms inspected.
    pub compared_terms: usize,
    /// Number of policy terms that could not be certified.
    pub unknown_terms: usize,
    /// Human-readable comparison facts.
    pub facts: Vec<String>,
}

/// Computes exact used height for a proposed 3D layout.
///
/// The report scans placement upper `z` coordinates, `placement.z + item.z`,
/// and records the certified maximum. Duplicate placements are intentionally
/// included because this is an objective scan over placement records, not a
/// quantity validator. Callers that need feasibility should pair this with
/// [`crate::verify_packing_3d`].
pub fn height_objective_3d(
    bin: &Bin3,
    items: &[Item3],
    placements: &[Placement3],
) -> PackResult<HeightObjective3> {
    let item_map = unique_item_map(items, |item| item.id.clone())?;
    let mut used_height = None::<Real>;
    let mut exact_comparisons = 0_usize;
    let mut unknown_comparisons = 0_usize;
    let mut facts = Vec::new();

    for placement in placements {
        let item = item_map
            .get(&placement.item)
            .copied()
            .ok_or(PackError::MissingItem)?;
        let top = placement.z.clone() + item.size.z.clone();
        update_max(
            &mut used_height,
            &top,
            &mut exact_comparisons,
            &mut unknown_comparisons,
        );
    }

    if unknown_comparisons > 0 {
        facts.push(format!(
            "{unknown_comparisons} used-height comparisons were unknown"
        ));
    }
    let remaining_height = used_height
        .as_ref()
        .map(|height| bin.size.z.clone() - height.clone());

    Ok(HeightObjective3 {
        checked_placements: placements.len(),
        bin_height: bin.size.z.clone(),
        used_height,
        remaining_height,
        exact_comparisons,
        unknown_comparisons,
        facts,
    })
}

/// Compares two exact replayed 3D packing objectives lexicographically.
///
/// The comparison policy is explicit because bin count, unplaced demand,
/// utilization, height, load, and cost can each be primary for different
/// problem variants. Scalar comparisons return certified order/equality or
/// explicit unknowns.
pub fn compare_objectives_3d(
    left: &PackingVerification3,
    left_height: Option<&HeightObjective3>,
    right: &PackingVerification3,
    right_height: Option<&HeightObjective3>,
    policy: &[ObjectiveTerm3],
) -> ObjectiveComparison3 {
    let mut facts = Vec::new();
    let mut unknown_terms = 0_usize;

    for (index, term) in policy.iter().copied().enumerate() {
        let ordering = match term {
            ObjectiveTerm3::MinimizeUnplacedItems => Some(
                left.objective
                    .unplaced_items
                    .cmp(&right.objective.unplaced_items),
            ),
            ObjectiveTerm3::MinimizeDuplicatePlacements => Some(
                left.objective
                    .duplicate_placements
                    .cmp(&right.objective.duplicate_placements),
            ),
            ObjectiveTerm3::MaximizeUsedVolume => {
                compare_real_descending(&left.objective.used_volume, &right.objective.used_volume)
            }
            ObjectiveTerm3::MinimizeWasteVolume => {
                compare_real_ascending(&left.objective.waste_volume, &right.objective.waste_volume)
            }
            ObjectiveTerm3::MinimizeUsedHeight => compare_optional_real_ascending(
                left_height.and_then(|height| height.used_height.as_ref()),
                right_height.and_then(|height| height.used_height.as_ref()),
            ),
            ObjectiveTerm3::MaximizeRemainingHeight => compare_optional_real_descending(
                left_height.and_then(|height| height.remaining_height.as_ref()),
                right_height.and_then(|height| height.remaining_height.as_ref()),
            ),
        };

        match ordering {
            Some(Ordering::Equal) => {}
            Some(ordering) => {
                return ObjectiveComparison3 {
                    ordering: Some(ordering),
                    decisive_term: Some(term),
                    compared_terms: index + 1,
                    unknown_terms,
                    facts,
                };
            }
            None => {
                unknown_terms += 1;
                facts.push(format!("{term:?} comparison was unknown"));
            }
        }
    }

    ObjectiveComparison3 {
        ordering: if unknown_terms == 0 {
            Some(Ordering::Equal)
        } else {
            None
        },
        decisive_term: None,
        compared_terms: policy.len(),
        unknown_terms,
        facts,
    }
}

fn update_max(
    current: &mut Option<Real>,
    candidate: &Real,
    exact_comparisons: &mut usize,
    unknown_comparisons: &mut usize,
) {
    let Some(existing) = current.as_ref() else {
        *current = Some(candidate.clone());
        return;
    };
    *exact_comparisons += 1;
    match (candidate - existing).refine_sign_until(-64) {
        Some(RealSign::Positive) => *current = Some(candidate.clone()),
        Some(RealSign::Zero | RealSign::Negative) => {}
        None => {
            *unknown_comparisons += 1;
            *current = None;
        }
    }
}

fn compare_optional_real_ascending(left: Option<&Real>, right: Option<&Real>) -> Option<Ordering> {
    compare_real_ascending(left?, right?)
}

fn compare_optional_real_descending(left: Option<&Real>, right: Option<&Real>) -> Option<Ordering> {
    compare_real_descending(left?, right?)
}

fn compare_real_ascending(left: &Real, right: &Real) -> Option<Ordering> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative => Some(Ordering::Less),
        RealSign::Zero => Some(Ordering::Equal),
        RealSign::Positive => Some(Ordering::Greater),
    }
}

fn compare_real_descending(left: &Real, right: &Real) -> Option<Ordering> {
    compare_real_ascending(left, right).map(Ordering::reverse)
}
