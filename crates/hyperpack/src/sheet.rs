//! Exact 2D sheet-packing carriers and replay.
//!
//! Sheet packing lifts the same proposal/replay discipline to exact rectangles:
//! a heuristic may propose coordinates, but containment, no-overlap, and item
//! accounting are replayed with exact sign predicates. Topology-changing
//! decisions use certified signs or remain explicitly unknown.

use std::collections::BTreeMap;

use hyperreal::{Real, RealSign};

use crate::{FeasibilityStatus, ItemId, PackError, PackResult, model::unique_item_map};

/// Exact two-dimensional rectangle size.
#[derive(Clone, Debug, PartialEq)]
pub struct Rect2 {
    /// Width along x.
    pub x: Real,
    /// Height along y.
    pub y: Real,
}

/// Exact two-dimensional sheet/bin.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetBin2 {
    /// Sheet size.
    pub size: Rect2,
}

/// Exact two-dimensional rectangular item.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetItem2 {
    /// Item id.
    pub id: ItemId,
    /// Item size.
    pub size: Rect2,
}

/// Placement of a 2D item by its lower-left coordinate.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetPlacement2 {
    /// Placed item id.
    pub item: ItemId,
    /// Exact x coordinate.
    pub x: Real,
    /// Exact y coordinate.
    pub y: Real,
}

/// Allowed fixed-orientation choices for rectangular 2D packing.
///
/// Cardinal 90-degree rotations preserve exact rectangle dimensions by
/// permutation only; no trigonometric approximation is introduced.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Orientation2 {
    /// Use the source width/height as stored.
    Deg0,
    /// Swap width and height.
    Deg90,
}

/// Exact 2D item with an explicit orientation policy.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientedSheetItem2 {
    /// Item id.
    pub id: ItemId,
    /// Source item size before orientation is applied.
    pub size: Rect2,
    /// Orientations this item may legally use.
    pub allowed_orientations: Vec<Orientation2>,
    /// Source unit/provenance label for validation reports.
    pub source_unit: String,
}

/// Placement of an oriented 2D item.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientedSheetPlacement2 {
    /// Placed item id.
    pub item: ItemId,
    /// Exact x coordinate.
    pub x: Real,
    /// Exact y coordinate.
    pub y: Real,
    /// Orientation used by this placement.
    pub orientation: Orientation2,
}

/// Exact objective summary for one-sheet 2D packing replay.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetObjective2 {
    /// Exact sheet area.
    pub bin_area: Real,
    /// Sum of placed item areas.
    pub used_area: Real,
    /// Exact `bin_area - used_area`.
    pub waste_area: Real,
    /// Number of item ids placed at least once.
    pub placed_items: usize,
    /// Number of item ids not placed.
    pub unplaced_items: usize,
    /// Number of duplicate placement records beyond the first placement.
    pub duplicate_placements: usize,
}

/// Full one-sheet verification report.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetVerification2 {
    /// Overall feasibility status.
    pub status: FeasibilityStatus,
    /// Number of containment checks.
    pub containment_checks: usize,
    /// Number of pairwise no-overlap checks.
    pub no_overlap_checks: usize,
    /// Exact objective/accounting replay.
    pub objective: SheetObjective2,
    /// Item ids that were not placed.
    pub unplaced: Vec<ItemId>,
    /// Item ids that appeared in more than one placement.
    pub duplicates: Vec<ItemId>,
    /// Human-readable exact facts.
    pub facts: Vec<String>,
}

/// Validation report for oriented 2D sheet inputs and placements.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientationValidationReport2 {
    /// Number of oriented placements checked.
    pub checked_placements: usize,
    /// Number of oriented item policies checked.
    pub checked_items: usize,
    /// Item ids with empty orientation policies.
    pub empty_orientation_items: Vec<ItemId>,
    /// Placement item ids using an orientation not allowed by the item policy.
    pub illegal_orientation_items: Vec<ItemId>,
    /// Human-readable exact validation facts.
    pub facts: Vec<String>,
}

/// Full oriented one-sheet verification report.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientedSheetVerification2 {
    /// Orientation-policy validation facts.
    pub orientation: OrientationValidationReport2,
    /// Exact fixed-orientation replay after legal cardinal orientation.
    pub sheet: SheetVerification2,
}

impl Rect2 {
    /// Creates positive exact rectangle dimensions.
    pub fn new(x: Real, y: Real) -> PackResult<Self> {
        require_positive(&x)?;
        require_positive(&y)?;
        Ok(Self { x, y })
    }

    /// Exact area.
    pub fn area(&self) -> Real {
        &self.x * &self.y
    }
}

impl SheetBin2 {
    /// Creates a sheet with positive exact dimensions.
    pub fn new(size: Rect2) -> Self {
        Self { size }
    }
}

impl SheetItem2 {
    /// Creates a rectangular item with positive exact dimensions.
    pub fn new(id: ItemId, size: Rect2) -> Self {
        Self { id, size }
    }
}

impl OrientedSheetItem2 {
    /// Creates an oriented rectangular item policy.
    pub fn new(
        id: ItemId,
        size: Rect2,
        allowed_orientations: Vec<Orientation2>,
        source_unit: impl Into<String>,
    ) -> Self {
        Self {
            id,
            size,
            allowed_orientations,
            source_unit: source_unit.into(),
        }
    }
}

impl Orientation2 {
    /// Applies this cardinal orientation exactly by permuting dimensions.
    pub fn apply(self, size: &Rect2) -> Rect2 {
        match self {
            Self::Deg0 => size.clone(),
            Self::Deg90 => Rect2 {
                x: size.y.clone(),
                y: size.x.clone(),
            },
        }
    }
}

/// Verifies one-sheet 2D packing geometry and item accounting.
///
/// This entry point uses fixed-orientation, quantity-one rectangles. Use
/// [`verify_oriented_packing_2d`] for cardinal rotations and
/// [`crate::verify_clearance_2d`] for kerf or trim clearance.
pub fn verify_packing_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
    placements: &[SheetPlacement2],
) -> PackResult<SheetVerification2> {
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
                facts.push(format!("{} outside sheet", placement.item.as_str()));
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
        'pairs: for left_index in 0..placements.len() {
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
                        break 'pairs;
                    }
                    None => {
                        status = FeasibilityStatus::Unknown;
                        break 'pairs;
                    }
                }
            }
        }
    }

    let mut placement_counts = BTreeMap::<ItemId, usize>::new();
    for placement in placements {
        *placement_counts.entry(placement.item.clone()).or_default() += 1;
    }
    let mut unplaced = Vec::new();
    let mut duplicates = Vec::new();
    let mut used_area = Real::zero();
    let mut placed_items = 0_usize;
    for item in items {
        match placement_counts.get(&item.id).copied().unwrap_or(0) {
            0 => unplaced.push(item.id.clone()),
            1 => {
                placed_items += 1;
                used_area += item.size.area();
            }
            count => {
                duplicates.push(item.id.clone());
                placed_items += 1;
                used_area += item.size.area() * Real::from(count as i64);
            }
        }
    }
    if !duplicates.is_empty() {
        status = FeasibilityStatus::Infeasible;
        for duplicate in &duplicates {
            facts.push(format!("{} placed more than once", duplicate.as_str()));
        }
    }

    let bin_area = bin.size.area();
    let objective = SheetObjective2 {
        waste_area: bin_area.clone() - used_area.clone(),
        bin_area,
        used_area,
        placed_items,
        unplaced_items: unplaced.len(),
        duplicate_placements: placement_counts
            .values()
            .map(|count| count.saturating_sub(1))
            .sum(),
    };

    Ok(SheetVerification2 {
        status,
        containment_checks,
        no_overlap_checks,
        objective,
        unplaced,
        duplicates,
        facts,
    })
}

/// Verifies a 2D packing with explicit cardinal orientation policies.
///
/// Orientation legality is checked before fixed-rectangle replay. Illegal
/// placements are reported as infeasible instead of being silently normalized
/// or lowered through approximate rotation math.
pub fn verify_oriented_packing_2d(
    bin: &SheetBin2,
    items: &[OrientedSheetItem2],
    placements: &[OrientedSheetPlacement2],
) -> PackResult<OrientedSheetVerification2> {
    let item_map = unique_item_map(items, |item| item.id.clone())?;
    let mut orientation = OrientationValidationReport2 {
        checked_placements: 0,
        checked_items: items.len(),
        empty_orientation_items: Vec::new(),
        illegal_orientation_items: Vec::new(),
        facts: Vec::new(),
    };

    for item in items {
        if item.allowed_orientations.is_empty() {
            orientation.empty_orientation_items.push(item.id.clone());
            orientation
                .facts
                .push(format!("{} has no allowed orientations", item.id.as_str()));
        }
    }

    let mut replay_items = BTreeMap::<ItemId, SheetItem2>::new();
    let mut replay_placements = Vec::with_capacity(placements.len());
    for placement in placements {
        orientation.checked_placements += 1;
        let item = item_map
            .get(&placement.item)
            .ok_or(PackError::MissingItem)?;
        if !item.allowed_orientations.contains(&placement.orientation) {
            orientation
                .illegal_orientation_items
                .push(placement.item.clone());
            orientation.facts.push(format!(
                "{} uses disallowed orientation {:?}",
                placement.item.as_str(),
                placement.orientation
            ));
        }
        replay_items.entry(item.id.clone()).or_insert_with(|| {
            SheetItem2::new(item.id.clone(), placement.orientation.apply(&item.size))
        });
        replay_placements.push(SheetPlacement2 {
            item: placement.item.clone(),
            x: placement.x.clone(),
            y: placement.y.clone(),
        });
    }
    for item in items {
        replay_items
            .entry(item.id.clone())
            .or_insert_with(|| SheetItem2::new(item.id.clone(), item.size.clone()));
    }

    let mut sheet = verify_packing_2d(
        bin,
        &replay_items.into_values().collect::<Vec<_>>(),
        &replay_placements,
    )?;
    if !orientation.empty_orientation_items.is_empty()
        || !orientation.illegal_orientation_items.is_empty()
    {
        sheet.status = FeasibilityStatus::Infeasible;
        sheet.facts.extend(orientation.facts.iter().cloned());
    }

    Ok(OrientedSheetVerification2 { orientation, sheet })
}

fn contains(bin: &SheetBin2, item: &SheetItem2, placement: &SheetPlacement2) -> Option<bool> {
    crate::predicate::decide_all!(
        nonnegative(&placement.x),
        nonnegative(&placement.y),
        leq(&(placement.x.clone() + item.size.x.clone()), &bin.size.x),
        leq(&(placement.y.clone() + item.size.y.clone()), &bin.size.y),
    )
}

fn disjoint(
    left_item: &SheetItem2,
    left: &SheetPlacement2,
    right_item: &SheetItem2,
    right: &SheetPlacement2,
) -> Option<bool> {
    crate::predicate::decide_any!(
        leq(&(left.x.clone() + left_item.size.x.clone()), &right.x),
        leq(&(right.x.clone() + right_item.size.x.clone()), &left.x),
        leq(&(left.y.clone() + left_item.size.y.clone()), &right.y),
        leq(&(right.y.clone() + right_item.size.y.clone()), &left.y),
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
