//! Exact one-bin capacity lower bounds.
//!
//! These checks are intentionally necessary, not sufficient: they reject
//! impossible instances before search, but they do not prove that the remaining
//! instances admit a placement. Decisions that affect the combinatorial search
//! state use exact signs; uncertain signs stay explicit rather than being
//! rounded into a Boolean.

use hyperreal::{Real, RealSign};

use crate::{Bin3, Item3, ItemId, SheetBin2, SheetItem2};

/// Certification state for necessary capacity lower bounds.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CapacityBoundStatus {
    /// All currently implemented necessary capacity bounds were certified.
    Satisfied,
    /// At least one necessary capacity bound proves one-bin infeasibility.
    Violated,
    /// At least one exact comparison could not be certified.
    Unknown,
}

/// Necessary one-bin capacity lower-bound checks.
#[derive(Clone, Debug, PartialEq)]
pub struct CapacityBoundReport3 {
    /// Overall status derived from the certified bound checks.
    pub status: CapacityBoundStatus,
    /// Number of item records included in the bound.
    pub checked_items: usize,
    /// Sum of all item volumes.
    pub total_item_volume: Real,
    /// Bin volume.
    pub bin_volume: Real,
    /// Exact excess `total_item_volume - bin_volume` when volume capacity fails.
    pub volume_excess: Option<Real>,
    /// Whether total item volume is certified `<= bin_volume`.
    pub volume_capacity_ok: Option<bool>,
    /// Whether every item dimension is certified `<=` corresponding bin dimension.
    pub max_dimension_ok: Option<bool>,
    /// Human-readable exact bound facts.
    pub facts: Vec<String>,
}

/// Exact pair lower-bound evidence for two items that cannot share one bin.
#[derive(Clone, Debug, PartialEq)]
pub struct PairIncompatibility3 {
    /// First item id.
    pub left: ItemId,
    /// Second item id.
    pub right: ItemId,
}

/// Necessary pair-incompatibility lower-bound report.
#[derive(Clone, Debug, PartialEq)]
pub struct PairIncompatibilityReport3 {
    /// Overall status derived from certified pair checks.
    pub status: CapacityBoundStatus,
    /// Number of unordered item pairs checked.
    pub checked_pairs: usize,
    /// Certified pairs that cannot both fit in this bin.
    pub incompatible_pairs: Vec<PairIncompatibility3>,
    /// Number of pairs with at least one uncertified exact comparison and no
    /// certified separating axis.
    pub unknown_pairs: usize,
    /// Human-readable exact bound facts.
    pub facts: Vec<String>,
}

/// Necessary one-sheet 2D capacity lower-bound checks.
#[derive(Clone, Debug, PartialEq)]
pub struct CapacityBoundReport2 {
    /// Overall status derived from the certified bound checks.
    pub status: CapacityBoundStatus,
    /// Number of item records included in the bound.
    pub checked_items: usize,
    /// Sum of all item areas.
    pub total_item_area: Real,
    /// Sheet area.
    pub bin_area: Real,
    /// Exact excess `total_item_area - bin_area` when area capacity fails.
    pub area_excess: Option<Real>,
    /// Whether total item area is certified `<= bin_area`.
    pub area_capacity_ok: Option<bool>,
    /// Whether every item dimension is certified `<=` corresponding sheet dimension.
    pub max_dimension_ok: Option<bool>,
    /// Human-readable exact bound facts.
    pub facts: Vec<String>,
}

/// Exact 2D pair lower-bound evidence for two rectangles that cannot share one sheet.
#[derive(Clone, Debug, PartialEq)]
pub struct PairIncompatibility2 {
    /// First item id.
    pub left: ItemId,
    /// Second item id.
    pub right: ItemId,
}

/// Necessary 2D pair-incompatibility lower-bound report.
#[derive(Clone, Debug, PartialEq)]
pub struct PairIncompatibilityReport2 {
    /// Overall status derived from certified pair checks.
    pub status: CapacityBoundStatus,
    /// Number of unordered item pairs checked.
    pub checked_pairs: usize,
    /// Certified pairs that cannot both fit in this sheet.
    pub incompatible_pairs: Vec<PairIncompatibility2>,
    /// Number of pairs with at least one uncertified exact comparison and no
    /// certified separating axis.
    pub unknown_pairs: usize,
    /// Human-readable exact bound facts.
    pub facts: Vec<String>,
}

impl CapacityBoundReport3 {
    /// Returns whether a necessary capacity bound proves one-bin infeasibility.
    pub fn proves_infeasible(&self) -> bool {
        self.status == CapacityBoundStatus::Violated
    }
}

impl CapacityBoundReport2 {
    /// Returns whether a necessary capacity bound proves one-sheet infeasibility.
    pub fn proves_infeasible(&self) -> bool {
        self.status == CapacityBoundStatus::Violated
    }
}

impl PairIncompatibilityReport2 {
    /// Returns whether at least one pair proves one-sheet infeasibility for a
    /// quantity-one instance containing both items.
    pub fn proves_infeasible(&self) -> bool {
        self.status == CapacityBoundStatus::Violated
    }
}

impl PairIncompatibilityReport3 {
    /// Returns whether at least one pair proves one-bin infeasibility for a
    /// quantity-one instance containing both items.
    pub fn proves_infeasible(&self) -> bool {
        self.status == CapacityBoundStatus::Violated
    }
}

/// Computes exact necessary one-bin capacity bounds before placement search.
///
/// Passing these checks does not prove that a layout exists. Failing either
/// total-volume capacity or per-axis maximum item dimensions proves that no
/// axis-aligned one-bin packing can satisfy the current model.
pub fn capacity_bounds_3d(bin: &Bin3, items: &[Item3]) -> CapacityBoundReport3 {
    let mut total_item_volume = Real::zero();
    let mut facts = Vec::new();
    let mut max_dimension_ok = Some(true);

    for item in items {
        total_item_volume += item.size.volume();
        for (axis, item_extent, bin_extent) in [
            ("x", &item.size.x, &bin.size.x),
            ("y", &item.size.y, &bin.size.y),
            ("z", &item.size.z, &bin.size.z),
        ] {
            match leq(item_extent, bin_extent) {
                Some(true) => {}
                Some(false) => {
                    facts.push(format!("{} exceeds bin {axis} extent", item.id.as_str()));
                    max_dimension_ok = Some(false);
                }
                None if max_dimension_ok != Some(false) => max_dimension_ok = None,
                None => {}
            }
        }
    }

    let bin_volume = bin.size.volume();
    let volume_capacity_ok = leq(&total_item_volume, &bin_volume);
    let volume_excess = if volume_capacity_ok == Some(false) {
        facts.push("total item volume exceeds bin volume".into());
        Some(total_item_volume.clone() - bin_volume.clone())
    } else {
        None
    };
    let status = match (volume_capacity_ok, max_dimension_ok) {
        (Some(false), _) | (_, Some(false)) => CapacityBoundStatus::Violated,
        (Some(true), Some(true)) => CapacityBoundStatus::Satisfied,
        _ => CapacityBoundStatus::Unknown,
    };

    CapacityBoundReport3 {
        status,
        checked_items: items.len(),
        total_item_volume,
        bin_volume,
        volume_excess,
        volume_capacity_ok,
        max_dimension_ok,
        facts,
    }
}

/// Computes exact necessary one-sheet capacity bounds before placement search.
///
/// Passing these checks does not prove that a layout exists. Failing either
/// total-area capacity or per-axis maximum item dimensions proves that no
/// fixed-orientation one-sheet rectangular packing can satisfy the current
/// model.
pub fn capacity_bounds_2d(bin: &SheetBin2, items: &[SheetItem2]) -> CapacityBoundReport2 {
    let mut total_item_area = Real::zero();
    let mut facts = Vec::new();
    let mut max_dimension_ok = Some(true);

    for item in items {
        total_item_area += item.size.area();
        for (axis, item_extent, bin_extent) in [
            ("x", &item.size.x, &bin.size.x),
            ("y", &item.size.y, &bin.size.y),
        ] {
            match leq(item_extent, bin_extent) {
                Some(true) => {}
                Some(false) => {
                    facts.push(format!("{} exceeds sheet {axis} extent", item.id.as_str()));
                    max_dimension_ok = Some(false);
                }
                None if max_dimension_ok != Some(false) => max_dimension_ok = None,
                None => {}
            }
        }
    }

    let bin_area = bin.size.area();
    let area_capacity_ok = leq(&total_item_area, &bin_area);
    let area_excess = if area_capacity_ok == Some(false) {
        facts.push("total item area exceeds sheet area".into());
        Some(total_item_area.clone() - bin_area.clone())
    } else {
        None
    };
    let status = match (area_capacity_ok, max_dimension_ok) {
        (Some(false), _) | (_, Some(false)) => CapacityBoundStatus::Violated,
        (Some(true), Some(true)) => CapacityBoundStatus::Satisfied,
        _ => CapacityBoundStatus::Unknown,
    };

    CapacityBoundReport2 {
        status,
        checked_items: items.len(),
        total_item_area,
        bin_area,
        area_excess,
        area_capacity_ok,
        max_dimension_ok,
        facts,
    }
}

/// Computes exact pair-incompatibility lower bounds before placement search.
///
/// Two axis-aligned cuboids can avoid overlap only if at least one axis can
/// separate them. Therefore, if `left.axis + right.axis > bin.axis` is
/// certified on every axis, the pair cannot coexist in this one bin. This is a
/// necessary pruning certificate, not a complete multi-item feasibility proof.
pub fn pair_incompatibilities_3d(bin: &Bin3, items: &[Item3]) -> PairIncompatibilityReport3 {
    let mut checked_pairs = 0_usize;
    let mut unknown_pairs = 0_usize;
    let mut incompatible_pairs = Vec::new();
    let mut facts = Vec::new();

    for left_index in 0..items.len() {
        for right_index in (left_index + 1)..items.len() {
            checked_pairs += 1;
            let left = &items[left_index];
            let right = &items[right_index];
            let mut has_separating_axis = false;
            let mut has_unknown_axis = false;

            for (left_extent, right_extent, bin_extent) in [
                (&left.size.x, &right.size.x, &bin.size.x),
                (&left.size.y, &right.size.y, &bin.size.y),
                (&left.size.z, &right.size.z, &bin.size.z),
            ] {
                match leq(&(left_extent.clone() + right_extent.clone()), bin_extent) {
                    Some(true) => {
                        has_separating_axis = true;
                        break;
                    }
                    Some(false) => {}
                    None => has_unknown_axis = true,
                }
            }

            if has_separating_axis {
                continue;
            }
            if has_unknown_axis {
                unknown_pairs += 1;
                continue;
            }

            facts.push(format!(
                "{} cannot share one bin with {}",
                left.id.as_str(),
                right.id.as_str()
            ));
            incompatible_pairs.push(PairIncompatibility3 {
                left: left.id.clone(),
                right: right.id.clone(),
            });
        }
    }

    let status = if !incompatible_pairs.is_empty() {
        CapacityBoundStatus::Violated
    } else if unknown_pairs > 0 {
        CapacityBoundStatus::Unknown
    } else {
        CapacityBoundStatus::Satisfied
    };

    PairIncompatibilityReport3 {
        status,
        checked_pairs,
        incompatible_pairs,
        unknown_pairs,
        facts,
    }
}

/// Computes exact 2D pair-incompatibility lower bounds before placement search.
///
/// Two fixed-orientation rectangles can avoid overlap only if at least one axis
/// can separate them. If `left.x + right.x > sheet.x` and
/// `left.y + right.y > sheet.y` are both certified, the pair cannot coexist in
/// this one sheet. The check is necessary evidence only; exact replay still
/// owns acceptance.
pub fn pair_incompatibilities_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PairIncompatibilityReport2 {
    let mut checked_pairs = 0_usize;
    let mut unknown_pairs = 0_usize;
    let mut incompatible_pairs = Vec::new();
    let mut facts = Vec::new();

    for left_index in 0..items.len() {
        for right_index in (left_index + 1)..items.len() {
            checked_pairs += 1;
            let left = &items[left_index];
            let right = &items[right_index];
            let mut has_separating_axis = false;
            let mut has_unknown_axis = false;

            for (left_extent, right_extent, bin_extent) in [
                (&left.size.x, &right.size.x, &bin.size.x),
                (&left.size.y, &right.size.y, &bin.size.y),
            ] {
                match leq(&(left_extent.clone() + right_extent.clone()), bin_extent) {
                    Some(true) => {
                        has_separating_axis = true;
                        break;
                    }
                    Some(false) => {}
                    None => has_unknown_axis = true,
                }
            }

            if has_separating_axis {
                continue;
            }
            if has_unknown_axis {
                unknown_pairs += 1;
                continue;
            }

            facts.push(format!(
                "{} cannot share one sheet with {}",
                left.id.as_str(),
                right.id.as_str()
            ));
            incompatible_pairs.push(PairIncompatibility2 {
                left: left.id.clone(),
                right: right.id.clone(),
            });
        }
    }

    let status = if !incompatible_pairs.is_empty() {
        CapacityBoundStatus::Violated
    } else if unknown_pairs > 0 {
        CapacityBoundStatus::Unknown
    } else {
        CapacityBoundStatus::Satisfied
    };

    PairIncompatibilityReport2 {
        status,
        checked_pairs,
        incompatible_pairs,
        unknown_pairs,
        facts,
    }
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative | RealSign::Zero => Some(true),
        RealSign::Positive => Some(false),
    }
}
