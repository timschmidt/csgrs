//! Exact clearance replay for 2D and 3D packing proposals.
//!
//! Base feasibility replay allows exact edge/face/corner contact because
//! non-overlap and clearance are different policies. This module adds replay
//! layers for callers that require a positive gap, kerf allowance, or access
//! margin. Separation uses exact interval comparisons, and uncertified
//! comparisons become explicit unknowns rather than tolerance decisions.

use hyperreal::{Real, RealSign};

use crate::{
    Item3, ItemId, PackError, PackResult, Placement3, SheetItem2, SheetPlacement2,
    model::unique_item_map,
};

/// Overall status for clearance replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClearanceStatus3 {
    /// Every checked pair has at least the requested exact clearance.
    Satisfied,
    /// At least one pair is exactly closer than the requested clearance.
    Violated,
    /// At least one required comparison could not be certified.
    Unknown,
}

/// Overall status for 2D clearance replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClearanceStatus2 {
    /// Every checked pair has at least the requested exact clearance.
    Satisfied,
    /// At least one pair is exactly closer than the requested clearance.
    Violated,
    /// At least one required comparison could not be certified.
    Unknown,
}

/// Exact pairwise clearance evidence for one placement pair.
#[derive(Clone, Debug, PartialEq)]
pub struct ClearancePairEvidence3 {
    /// First item id in placement order.
    pub left: ItemId,
    /// Second item id in placement order.
    pub right: ItemId,
    /// Certified separating-axis gap used for the clearance decision, if any.
    pub separating_gap: Option<Real>,
    /// Whether this pair satisfies the requested clearance.
    pub satisfied: Option<bool>,
}

/// Exact pairwise clearance evidence for one 2D placement pair.
#[derive(Clone, Debug, PartialEq)]
pub struct ClearancePairEvidence2 {
    /// First item id in placement order.
    pub left: ItemId,
    /// Second item id in placement order.
    pub right: ItemId,
    /// Certified separating-axis gap used for the clearance decision, if any.
    pub separating_gap: Option<Real>,
    /// Whether this pair satisfies the requested clearance.
    pub satisfied: Option<bool>,
}

/// Exact clearance replay report.
#[derive(Clone, Debug, PartialEq)]
pub struct ClearanceReport3 {
    /// Requested exact clearance.
    pub required_clearance: Real,
    /// Overall status.
    pub status: ClearanceStatus3,
    /// Per-pair evidence.
    pub pairs: Vec<ClearancePairEvidence3>,
    /// Exact comparisons performed by clearance replay.
    pub exact_comparisons: usize,
    /// Human-readable facts.
    pub facts: Vec<String>,
}

/// Exact 2D clearance replay report.
#[derive(Clone, Debug, PartialEq)]
pub struct ClearanceReport2 {
    /// Requested exact clearance or kerf.
    pub required_clearance: Real,
    /// Overall status.
    pub status: ClearanceStatus2,
    /// Per-pair evidence.
    pub pairs: Vec<ClearancePairEvidence2>,
    /// Exact comparisons performed by clearance replay.
    pub exact_comparisons: usize,
    /// Human-readable facts.
    pub facts: Vec<String>,
}

/// Replays exact pairwise clearance for already proposed 2D placements.
///
/// A pair satisfies clearance when at least one axis has exact gap greater than
/// or equal to `required_clearance`. A zero clearance matches ordinary
/// rectangle contact semantics, while positive clearance rejects exact edge
/// contact and can model a required saw/tool kerf between rectangles. This
/// function does not replace [`crate::verify_packing_2d`]; callers should run
/// both when they need containment, no-overlap, and gap policy evidence.
pub fn verify_clearance_2d(
    items: &[SheetItem2],
    placements: &[SheetPlacement2],
    required_clearance: Real,
) -> PackResult<ClearanceReport2> {
    if negative(&required_clearance).unwrap_or(true) {
        return Err(PackError::NegativeClearance);
    }

    let item_map = unique_item_map(items, |item| item.id.clone())?;
    let placement_items = placements
        .iter()
        .map(|placement| {
            item_map
                .get(&placement.item)
                .copied()
                .ok_or(PackError::MissingItem)
        })
        .collect::<PackResult<Vec<_>>>()?;
    let mut pairs = Vec::new();
    let mut facts = Vec::new();
    let mut status = ClearanceStatus2::Satisfied;
    let mut exact_comparisons = 0_usize;

    for left_index in 0..placements.len() {
        for right_index in (left_index + 1)..placements.len() {
            let left = &placements[left_index];
            let right = &placements[right_index];
            let gap = separating_gap_2d(
                placement_items[left_index],
                left,
                placement_items[right_index],
                right,
                &mut exact_comparisons,
            );
            let satisfied = gap.as_ref().and_then(|gap| {
                exact_comparisons += 1;
                leq(&required_clearance, gap)
            });
            match satisfied {
                Some(true) => {}
                Some(false) => {
                    status = ClearanceStatus2::Violated;
                    facts.push(format!(
                        "{} and {} are closer than required clearance",
                        left.item.as_str(),
                        right.item.as_str()
                    ));
                }
                None if status != ClearanceStatus2::Violated => {
                    status = ClearanceStatus2::Unknown;
                    facts.push(format!(
                        "{} and {} clearance could not be certified",
                        left.item.as_str(),
                        right.item.as_str()
                    ));
                }
                None => {}
            }
            pairs.push(ClearancePairEvidence2 {
                left: left.item.clone(),
                right: right.item.clone(),
                separating_gap: gap,
                satisfied,
            });
        }
    }

    Ok(ClearanceReport2 {
        required_clearance,
        status,
        pairs,
        exact_comparisons,
        facts,
    })
}

/// Replays exact pairwise clearance for already proposed 3D placements.
///
/// A pair satisfies clearance when at least one axis has exact gap greater than
/// or equal to `required_clearance`. A zero clearance therefore matches ordinary
/// no-overlap/contact semantics, while positive clearance rejects exact face
/// contact. This function does not replace [`crate::verify_packing_3d`];
/// callers should run both when they need containment, no-overlap, and gap
/// policy evidence.
pub fn verify_clearance_3d(
    items: &[Item3],
    placements: &[Placement3],
    required_clearance: Real,
) -> PackResult<ClearanceReport3> {
    if negative(&required_clearance).unwrap_or(true) {
        return Err(PackError::NegativeClearance);
    }

    let item_map = unique_item_map(items, |item| item.id.clone())?;
    let placement_items = placements
        .iter()
        .map(|placement| {
            item_map
                .get(&placement.item)
                .copied()
                .ok_or(PackError::MissingItem)
        })
        .collect::<PackResult<Vec<_>>>()?;
    let mut pairs = Vec::new();
    let mut facts = Vec::new();
    let mut status = ClearanceStatus3::Satisfied;
    let mut exact_comparisons = 0_usize;

    for left_index in 0..placements.len() {
        for right_index in (left_index + 1)..placements.len() {
            let left = &placements[left_index];
            let right = &placements[right_index];
            let gap = separating_gap(
                placement_items[left_index],
                left,
                placement_items[right_index],
                right,
                &mut exact_comparisons,
            );
            let satisfied = gap.as_ref().and_then(|gap| {
                exact_comparisons += 1;
                leq(&required_clearance, gap)
            });
            match satisfied {
                Some(true) => {}
                Some(false) => {
                    status = ClearanceStatus3::Violated;
                    facts.push(format!(
                        "{} and {} are closer than required clearance",
                        left.item.as_str(),
                        right.item.as_str()
                    ));
                }
                None if status != ClearanceStatus3::Violated => {
                    status = ClearanceStatus3::Unknown;
                    facts.push(format!(
                        "{} and {} clearance could not be certified",
                        left.item.as_str(),
                        right.item.as_str()
                    ));
                }
                None => {}
            }
            pairs.push(ClearancePairEvidence3 {
                left: left.item.clone(),
                right: right.item.clone(),
                separating_gap: gap,
                satisfied,
            });
        }
    }

    Ok(ClearanceReport3 {
        required_clearance,
        status,
        pairs,
        exact_comparisons,
        facts,
    })
}

fn separating_gap_2d(
    left_item: &SheetItem2,
    left: &SheetPlacement2,
    right_item: &SheetItem2,
    right: &SheetPlacement2,
    exact_comparisons: &mut usize,
) -> Option<Real> {
    let mut best = None::<Real>;
    for gap in [
        right.x.clone() - (left.x.clone() + left_item.size.x.clone()),
        left.x.clone() - (right.x.clone() + right_item.size.x.clone()),
        right.y.clone() - (left.y.clone() + left_item.size.y.clone()),
        left.y.clone() - (right.y.clone() + right_item.size.y.clone()),
    ] {
        *exact_comparisons += 1;
        if nonnegative(&gap).unwrap_or(false)
            && best.as_ref().is_none_or(|current| {
                *exact_comparisons += 1;
                gt(&gap, current).unwrap_or(false)
            })
        {
            best = Some(gap);
        }
    }
    best
}

fn separating_gap(
    left_item: &Item3,
    left: &Placement3,
    right_item: &Item3,
    right: &Placement3,
    exact_comparisons: &mut usize,
) -> Option<Real> {
    let mut best = None::<Real>;
    for gap in [
        right.x.clone() - (left.x.clone() + left_item.size.x.clone()),
        left.x.clone() - (right.x.clone() + right_item.size.x.clone()),
        right.y.clone() - (left.y.clone() + left_item.size.y.clone()),
        left.y.clone() - (right.y.clone() + right_item.size.y.clone()),
        right.z.clone() - (left.z.clone() + left_item.size.z.clone()),
        left.z.clone() - (right.z.clone() + right_item.size.z.clone()),
    ] {
        *exact_comparisons += 1;
        if nonnegative(&gap).unwrap_or(false)
            && best.as_ref().is_none_or(|current| {
                *exact_comparisons += 1;
                gt(&gap, current).unwrap_or(false)
            })
        {
            best = Some(gap);
        }
    }
    best
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative | RealSign::Zero => Some(true),
        RealSign::Positive => Some(false),
    }
}

fn gt(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Positive => Some(true),
        RealSign::Zero | RealSign::Negative => Some(false),
    }
}

fn nonnegative(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Negative => Some(false),
        RealSign::Zero | RealSign::Positive => Some(true),
    }
}

fn negative(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Negative => Some(true),
        RealSign::Zero | RealSign::Positive => Some(false),
    }
}
