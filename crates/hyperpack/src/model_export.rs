//! Exact CP/MIP-style model export reports for packing adapters.
//!
//! This module does not call an external optimizer. It lowers one-sheet
//! rectangle and one-bin cuboid instances into exact coordinate domains and
//! pairwise no-overlap disjunctions that a CP-SAT, MIP, or future `hypersolve`
//! adapter can consume. Following
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>), this export
//! preserves exact bounds and explicit infeasibility evidence instead of
//! normalizing the model to primitive floats. The disjunctive rectangular
//! packing form is the standard finite-bin/sheet model discussed by Martello,
//! Vigo, and Iori et al. for 2D cutting/packing and by Martello,
//! Pisinger, and Vigo, "The Three-Dimensional Bin Packing Problem,"
//! *Operations Research* 48(2), 2000: every item pair must be separated along
//! at least one axis.

use hyperreal::{Real, RealSign};

use crate::{
    Bin3, CapacityBoundStatus, Item3, ItemId, SheetBin2, SheetItem2, capacity_bounds_2d,
    capacity_bounds_3d,
};

/// Export status for a one-bin exact no-overlap model.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModelExportStatus3 {
    /// All domains and pair disjunctions were constructed.
    Ready,
    /// Exact necessary bounds already prove the one-bin model impossible.
    Infeasible,
    /// At least one exact comparison could not be certified.
    Unknown,
}

/// Export status for a one-sheet exact no-overlap model.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModelExportStatus2 {
    /// All domains and pair disjunctions were constructed.
    Ready,
    /// Exact necessary bounds already prove the one-sheet model impossible.
    Infeasible,
    /// At least one exact comparison could not be certified.
    Unknown,
}

/// Exact coordinate domain for one rectangle origin variable pair.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementDomain2 {
    /// Item id.
    pub item: ItemId,
    /// Exact minimum x origin.
    pub x_min: Real,
    /// Exact maximum x origin.
    pub x_max: Real,
    /// Exact minimum y origin.
    pub y_min: Real,
    /// Exact maximum y origin.
    pub y_max: Real,
}

/// Axis-side disjunction available to separate a pair of rectangles.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NoOverlapDisjunct2 {
    /// `left.x + left.width <= right.x`.
    LeftBeforeRightX,
    /// `right.x + right.width <= left.x`.
    RightBeforeLeftX,
    /// `left.y + left.height <= right.y`.
    LeftBeforeRightY,
    /// `right.y + right.height <= left.y`.
    RightBeforeLeftY,
}

/// Exact pairwise disjunction payload for a rectangle pair.
#[derive(Clone, Debug, PartialEq)]
pub struct PairNoOverlapDisjunction2 {
    /// Left item id in source order.
    pub left: ItemId,
    /// Right item id in source order.
    pub right: ItemId,
    /// Disjuncts whose axis has enough exact sheet extent to separate this pair.
    pub disjuncts: Vec<NoOverlapDisjunct2>,
}

/// Exact no-overlap model export report for one 2D sheet.
#[derive(Clone, Debug, PartialEq)]
pub struct NoOverlapModelReport2 {
    /// Export status.
    pub status: ModelExportStatus2,
    /// Exact origin domains.
    pub domains: Vec<PlacementDomain2>,
    /// Exact pairwise separation disjunctions.
    pub disjunctions: Vec<PairNoOverlapDisjunction2>,
    /// Capacity/lower-bound status checked before export.
    pub lower_bound_status: CapacityBoundStatus,
    /// Exact comparisons performed while constructing the model.
    pub exact_comparisons: usize,
    /// Human-readable model facts.
    pub facts: Vec<String>,
}

/// Exact coordinate domain for one cuboid origin variable triple.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementDomain3 {
    /// Item id.
    pub item: ItemId,
    /// Exact minimum x origin.
    pub x_min: Real,
    /// Exact maximum x origin.
    pub x_max: Real,
    /// Exact minimum y origin.
    pub y_min: Real,
    /// Exact maximum y origin.
    pub y_max: Real,
    /// Exact minimum z origin.
    pub z_min: Real,
    /// Exact maximum z origin.
    pub z_max: Real,
}

/// Axis-side disjunction available to separate a pair of cuboids.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NoOverlapDisjunct3 {
    /// `left.x + left.width <= right.x`.
    LeftBeforeRightX,
    /// `right.x + right.width <= left.x`.
    RightBeforeLeftX,
    /// `left.y + left.depth <= right.y`.
    LeftBeforeRightY,
    /// `right.y + right.depth <= left.y`.
    RightBeforeLeftY,
    /// `left.z + left.height <= right.z`.
    LeftBeforeRightZ,
    /// `right.z + right.height <= left.z`.
    RightBeforeLeftZ,
}

/// Exact pairwise disjunction payload for a cuboid pair.
#[derive(Clone, Debug, PartialEq)]
pub struct PairNoOverlapDisjunction3 {
    /// Left item id in source order.
    pub left: ItemId,
    /// Right item id in source order.
    pub right: ItemId,
    /// Disjuncts whose axis has enough exact bin extent to separate this pair.
    pub disjuncts: Vec<NoOverlapDisjunct3>,
}

/// Exact no-overlap model export report for one 3D bin.
#[derive(Clone, Debug, PartialEq)]
pub struct NoOverlapModelReport3 {
    /// Export status.
    pub status: ModelExportStatus3,
    /// Exact origin domains.
    pub domains: Vec<PlacementDomain3>,
    /// Exact pairwise separation disjunctions.
    pub disjunctions: Vec<PairNoOverlapDisjunction3>,
    /// Capacity/lower-bound status checked before export.
    pub lower_bound_status: CapacityBoundStatus,
    /// Exact comparisons performed while constructing the model.
    pub exact_comparisons: usize,
    /// Human-readable model facts.
    pub facts: Vec<String>,
}

/// Exports an exact one-sheet 2D no-overlap model skeleton.
///
/// Each item receives exact origin domains `[0, sheet_axis - item_axis]`. Each
/// item pair receives the subset of the four standard separation disjuncts
/// whose axis has enough exact extent to place the pair without overlap. A pair
/// with no feasible separating axis makes the report infeasible. The returned
/// model is only a solver-adapter input; any solver placement must still be
/// checked by [`crate::verify_packing_2d`].
pub fn export_no_overlap_model_2d(bin: &SheetBin2, items: &[SheetItem2]) -> NoOverlapModelReport2 {
    let lower_bound = capacity_bounds_2d(bin, items);
    let mut exact_comparisons = 0_usize;
    let mut facts = Vec::new();
    let mut domains = Vec::new();
    let mut disjunctions = Vec::new();
    let mut status = if lower_bound.status == CapacityBoundStatus::Violated {
        facts.push("capacity lower bound proves one-sheet model infeasible".into());
        ModelExportStatus2::Infeasible
    } else {
        ModelExportStatus2::Ready
    };

    for item in items {
        let x_max = bin.size.x.clone() - item.size.x.clone();
        let y_max = bin.size.y.clone() - item.size.y.clone();
        exact_comparisons += 2;
        for (axis, value) in [("x", &x_max), ("y", &y_max)] {
            match nonnegative(value) {
                Some(true) => {}
                Some(false) => {
                    status = ModelExportStatus2::Infeasible;
                    facts.push(format!(
                        "{} has negative {axis} origin domain upper bound",
                        item.id.as_str()
                    ));
                }
                None if status != ModelExportStatus2::Infeasible => {
                    status = ModelExportStatus2::Unknown;
                    facts.push(format!(
                        "{} {axis} origin domain could not be certified",
                        item.id.as_str()
                    ));
                }
                None => {}
            }
        }
        domains.push(PlacementDomain2 {
            item: item.id.clone(),
            x_min: Real::zero(),
            x_max,
            y_min: Real::zero(),
            y_max,
        });
    }

    for left_index in 0..items.len() {
        for right_index in (left_index + 1)..items.len() {
            let left = &items[left_index];
            let right = &items[right_index];
            let mut disjuncts = Vec::new();
            push_axis_disjuncts_2d(
                &mut disjuncts,
                left.size.x.clone() + right.size.x.clone(),
                &bin.size.x,
                NoOverlapDisjunct2::LeftBeforeRightX,
                NoOverlapDisjunct2::RightBeforeLeftX,
                &mut exact_comparisons,
            );
            push_axis_disjuncts_2d(
                &mut disjuncts,
                left.size.y.clone() + right.size.y.clone(),
                &bin.size.y,
                NoOverlapDisjunct2::LeftBeforeRightY,
                NoOverlapDisjunct2::RightBeforeLeftY,
                &mut exact_comparisons,
            );
            if disjuncts.is_empty() {
                status = ModelExportStatus2::Infeasible;
                facts.push(format!(
                    "{} and {} have no feasible separating-axis disjunction",
                    left.id.as_str(),
                    right.id.as_str()
                ));
            }
            disjunctions.push(PairNoOverlapDisjunction2 {
                left: left.id.clone(),
                right: right.id.clone(),
                disjuncts,
            });
        }
    }

    NoOverlapModelReport2 {
        status,
        domains,
        disjunctions,
        lower_bound_status: lower_bound.status,
        exact_comparisons,
        facts,
    }
}

/// Exports an exact one-bin 3D no-overlap model skeleton.
///
/// Each item receives exact origin domains `[0, bin_axis - item_axis]`. Each
/// item pair receives the subset of the six standard separation disjuncts whose
/// axis has enough exact extent to place the pair without overlap. A pair with
/// no feasible separating axis makes the report infeasible. The returned model
/// is only a solver-adapter input; any solver placement must still be checked by
/// [`crate::verify_packing_3d`].
pub fn export_no_overlap_model_3d(bin: &Bin3, items: &[Item3]) -> NoOverlapModelReport3 {
    let lower_bound = capacity_bounds_3d(bin, items);
    let mut exact_comparisons = 0_usize;
    let mut facts = Vec::new();
    let mut domains = Vec::new();
    let mut disjunctions = Vec::new();
    let mut status = if lower_bound.status == CapacityBoundStatus::Violated {
        facts.push("capacity lower bound proves one-bin model infeasible".into());
        ModelExportStatus3::Infeasible
    } else {
        ModelExportStatus3::Ready
    };

    for item in items {
        let x_max = bin.size.x.clone() - item.size.x.clone();
        let y_max = bin.size.y.clone() - item.size.y.clone();
        let z_max = bin.size.z.clone() - item.size.z.clone();
        exact_comparisons += 3;
        for (axis, value) in [("x", &x_max), ("y", &y_max), ("z", &z_max)] {
            match nonnegative(value) {
                Some(true) => {}
                Some(false) => {
                    status = ModelExportStatus3::Infeasible;
                    facts.push(format!(
                        "{} has negative {axis} origin domain upper bound",
                        item.id.as_str()
                    ));
                }
                None if status != ModelExportStatus3::Infeasible => {
                    status = ModelExportStatus3::Unknown;
                    facts.push(format!(
                        "{} {axis} origin domain could not be certified",
                        item.id.as_str()
                    ));
                }
                None => {}
            }
        }
        domains.push(PlacementDomain3 {
            item: item.id.clone(),
            x_min: Real::zero(),
            x_max,
            y_min: Real::zero(),
            y_max,
            z_min: Real::zero(),
            z_max,
        });
    }

    for left_index in 0..items.len() {
        for right_index in (left_index + 1)..items.len() {
            let left = &items[left_index];
            let right = &items[right_index];
            let mut disjuncts = Vec::new();
            push_axis_disjuncts(
                &mut disjuncts,
                left.size.x.clone() + right.size.x.clone(),
                &bin.size.x,
                NoOverlapDisjunct3::LeftBeforeRightX,
                NoOverlapDisjunct3::RightBeforeLeftX,
                &mut exact_comparisons,
            );
            push_axis_disjuncts(
                &mut disjuncts,
                left.size.y.clone() + right.size.y.clone(),
                &bin.size.y,
                NoOverlapDisjunct3::LeftBeforeRightY,
                NoOverlapDisjunct3::RightBeforeLeftY,
                &mut exact_comparisons,
            );
            push_axis_disjuncts(
                &mut disjuncts,
                left.size.z.clone() + right.size.z.clone(),
                &bin.size.z,
                NoOverlapDisjunct3::LeftBeforeRightZ,
                NoOverlapDisjunct3::RightBeforeLeftZ,
                &mut exact_comparisons,
            );
            if disjuncts.is_empty() {
                status = ModelExportStatus3::Infeasible;
                facts.push(format!(
                    "{} and {} have no feasible separating-axis disjunction",
                    left.id.as_str(),
                    right.id.as_str()
                ));
            }
            disjunctions.push(PairNoOverlapDisjunction3 {
                left: left.id.clone(),
                right: right.id.clone(),
                disjuncts,
            });
        }
    }

    NoOverlapModelReport3 {
        status,
        domains,
        disjunctions,
        lower_bound_status: lower_bound.status,
        exact_comparisons,
        facts,
    }
}

fn push_axis_disjuncts(
    disjuncts: &mut Vec<NoOverlapDisjunct3>,
    combined_extent: Real,
    bin_extent: &Real,
    forward: NoOverlapDisjunct3,
    reverse: NoOverlapDisjunct3,
    exact_comparisons: &mut usize,
) {
    *exact_comparisons += 1;
    if leq(&combined_extent, bin_extent).unwrap_or(false) {
        disjuncts.push(forward);
        disjuncts.push(reverse);
    }
}

fn push_axis_disjuncts_2d(
    disjuncts: &mut Vec<NoOverlapDisjunct2>,
    combined_extent: Real,
    bin_extent: &Real,
    forward: NoOverlapDisjunct2,
    reverse: NoOverlapDisjunct2,
    exact_comparisons: &mut usize,
) {
    *exact_comparisons += 1;
    if leq(&combined_extent, bin_extent).unwrap_or(false) {
        disjuncts.push(forward);
        disjuncts.push(reverse);
    }
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative | RealSign::Zero => Some(true),
        RealSign::Positive => Some(false),
    }
}

fn nonnegative(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Negative => Some(false),
        RealSign::Zero | RealSign::Positive => Some(true),
    }
}
