//! Replay-gated heuristic portfolio schedulers.
//!
//! A portfolio runner is a scheduler, not a new geometric predicate. It tries
//! several proposal engines under an explicit deterministic budget and then
//! ranks their exact replay reports. Heuristic search can propose combinatorial
//! states, but acceptance and comparison use exact reports. No single packing
//! heuristic is treated as universally best.

use hyperreal::{Real, RealSign};

use crate::{
    Bin3, CuboidHeuristic3, CuboidHeuristicReport3, FeasibilityStatus, Item3, PackResult,
    SheetBin2, SheetHeuristic2, SheetHeuristicReport2, SheetItem2,
    cuboid_best_fit_decreasing_footprint_area_3d, cuboid_best_fit_decreasing_max_side_3d,
    cuboid_best_fit_decreasing_volume_3d, cuboid_extreme_point_decreasing_volume_3d,
    cuboid_first_fit_decreasing_footprint_area_3d, cuboid_first_fit_decreasing_max_side_3d,
    cuboid_first_fit_decreasing_volume_3d, cuboid_guillotine_best_volume_fit_3d,
    cuboid_laff_largest_area_fit_first_3d, cuboid_maximal_space_decreasing_volume_3d,
    guillotine_best_area_fit_2d, guillotine_best_long_side_fit_2d,
    guillotine_best_short_side_fit_2d, maxrects_best_area_fit_2d, maxrects_best_long_side_fit_2d,
    maxrects_best_short_side_fit_2d, maxrects_bottom_left_2d, maxrects_contact_point_2d,
    shelf_best_fit_decreasing_height_2d, shelf_first_fit_decreasing_height_2d,
    shelf_next_fit_decreasing_height_2d, skyline_bottom_left_2d, skyline_minimum_waste_2d,
};

/// Deterministic budget for a fixed 2D sheet heuristic portfolio.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SheetPortfolioBudget2 {
    /// Maximum number of candidate algorithms to run in portfolio order.
    pub max_algorithms: usize,
}

/// Completion status for a 2D sheet heuristic portfolio.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SheetPortfolioStatus2 {
    /// At least one algorithm was evaluated and a best report was selected.
    Complete,
    /// The budget allowed no algorithm evaluations.
    BudgetExhausted,
}

/// Deterministic report for a 2D sheet heuristic portfolio run.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetPortfolioReport2 {
    /// Completion status.
    pub status: SheetPortfolioStatus2,
    /// Algorithms evaluated in deterministic order.
    pub evaluated: Vec<SheetHeuristic2>,
    /// Reports emitted by evaluated algorithms.
    pub reports: Vec<SheetHeuristicReport2>,
    /// Best exact replay-ranked report, if at least one algorithm ran.
    pub best: Option<SheetHeuristicReport2>,
    /// Human-readable scheduler facts.
    pub facts: Vec<String>,
}

/// Deterministic budget for a fixed 3D cuboid heuristic portfolio.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CuboidPortfolioBudget3 {
    /// Maximum number of candidate algorithms to run in portfolio order.
    pub max_algorithms: usize,
}

/// Completion status for a 3D cuboid heuristic portfolio.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CuboidPortfolioStatus3 {
    /// At least one algorithm was evaluated and a best report was selected.
    Complete,
    /// The budget allowed no algorithm evaluations.
    BudgetExhausted,
}

/// Deterministic report for a 3D cuboid heuristic portfolio run.
#[derive(Clone, Debug, PartialEq)]
pub struct CuboidPortfolioReport3 {
    /// Completion status.
    pub status: CuboidPortfolioStatus3,
    /// Algorithms evaluated in deterministic order.
    pub evaluated: Vec<CuboidHeuristic3>,
    /// Reports emitted by evaluated algorithms.
    pub reports: Vec<CuboidHeuristicReport3>,
    /// Best exact replay-ranked report, if at least one algorithm ran.
    pub best: Option<CuboidHeuristicReport3>,
    /// Human-readable scheduler facts.
    pub facts: Vec<String>,
}

/// Runs a deterministic exact-replay-ranked 2D sheet heuristic portfolio.
///
/// Algorithms are evaluated in a fixed broad-to-dense order: shelf baselines,
/// skyline variants, MaxRects variants, then guillotine. Ranking first prefers
/// exact feasible replay over infeasible/unknown replay, then fewer unplaced
/// items, larger exact used area, fewer duplicate placements, and finally the
/// first evaluated algorithm for deterministic ties.
pub fn auto_sheet_portfolio_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
    budget: SheetPortfolioBudget2,
) -> PackResult<SheetPortfolioReport2> {
    if budget.max_algorithms == 0 {
        return Ok(SheetPortfolioReport2 {
            status: SheetPortfolioStatus2::BudgetExhausted,
            evaluated: Vec::new(),
            reports: Vec::new(),
            best: None,
            facts: vec!["portfolio budget allowed no algorithms".into()],
        });
    }

    let algorithms = sheet_portfolio_algorithms();
    let mut evaluated = Vec::new();
    let mut reports = Vec::new();
    let mut best = None::<SheetHeuristicReport2>;
    for (heuristic, algorithm) in algorithms.into_iter().take(budget.max_algorithms) {
        let report = algorithm(bin, items)?;
        if best
            .as_ref()
            .is_none_or(|current| sheet_report_better(&report, current))
        {
            best = Some(report.clone());
        }
        evaluated.push(heuristic);
        reports.push(report);
    }

    Ok(SheetPortfolioReport2 {
        status: SheetPortfolioStatus2::Complete,
        evaluated,
        reports,
        best,
        facts: Vec::new(),
    })
}

/// Runs a deterministic exact-replay-ranked 3D cuboid heuristic portfolio.
///
/// The portfolio evaluates fixed-orientation 3D proposal engines in a stable
/// order: corner first/best-fit baselines, DBLF/extreme-point, conservative
/// maximal-space/free-box, and guillotine free-box splitting. Ranking mirrors
/// [`auto_sheet_portfolio_2d`]: exact feasible replay first, then fewer
/// unplaced items, larger exact used volume, fewer duplicate placements, and
/// deterministic evaluation order for ties.
pub fn auto_cuboid_portfolio_3d(
    bin: &Bin3,
    items: &[Item3],
    budget: CuboidPortfolioBudget3,
) -> PackResult<CuboidPortfolioReport3> {
    if budget.max_algorithms == 0 {
        return Ok(CuboidPortfolioReport3 {
            status: CuboidPortfolioStatus3::BudgetExhausted,
            evaluated: Vec::new(),
            reports: Vec::new(),
            best: None,
            facts: vec!["portfolio budget allowed no algorithms".into()],
        });
    }

    let algorithms = cuboid_portfolio_algorithms();
    let mut evaluated = Vec::new();
    let mut reports = Vec::new();
    let mut best = None::<CuboidHeuristicReport3>;
    for (heuristic, algorithm) in algorithms.into_iter().take(budget.max_algorithms) {
        let report = algorithm(bin, items)?;
        if best
            .as_ref()
            .is_none_or(|current| cuboid_report_better(&report, current))
        {
            best = Some(report.clone());
        }
        evaluated.push(heuristic);
        reports.push(report);
    }

    Ok(CuboidPortfolioReport3 {
        status: CuboidPortfolioStatus3::Complete,
        evaluated,
        reports,
        best,
        facts: Vec::new(),
    })
}

type SheetAlgorithm2 = fn(&SheetBin2, &[SheetItem2]) -> PackResult<SheetHeuristicReport2>;
type CuboidAlgorithm3 = fn(&Bin3, &[Item3]) -> PackResult<CuboidHeuristicReport3>;

fn sheet_portfolio_algorithms() -> Vec<(SheetHeuristic2, SheetAlgorithm2)> {
    vec![
        (
            SheetHeuristic2::NextFitDecreasingHeight,
            shelf_next_fit_decreasing_height_2d,
        ),
        (
            SheetHeuristic2::FirstFitDecreasingHeight,
            shelf_first_fit_decreasing_height_2d,
        ),
        (
            SheetHeuristic2::BestFitDecreasingHeight,
            shelf_best_fit_decreasing_height_2d,
        ),
        (SheetHeuristic2::SkylineBottomLeft, skyline_bottom_left_2d),
        (
            SheetHeuristic2::SkylineMinimumWaste,
            skyline_minimum_waste_2d,
        ),
        (
            SheetHeuristic2::MaxRectsBestShortSideFit,
            maxrects_best_short_side_fit_2d,
        ),
        (
            SheetHeuristic2::MaxRectsBestLongSideFit,
            maxrects_best_long_side_fit_2d,
        ),
        (
            SheetHeuristic2::MaxRectsBestAreaFit,
            maxrects_best_area_fit_2d,
        ),
        (SheetHeuristic2::MaxRectsBottomLeft, maxrects_bottom_left_2d),
        (
            SheetHeuristic2::MaxRectsContactPoint,
            maxrects_contact_point_2d,
        ),
        (
            SheetHeuristic2::GuillotineBestAreaFit,
            guillotine_best_area_fit_2d,
        ),
        (
            SheetHeuristic2::GuillotineBestShortSideFit,
            guillotine_best_short_side_fit_2d,
        ),
        (
            SheetHeuristic2::GuillotineBestLongSideFit,
            guillotine_best_long_side_fit_2d,
        ),
    ]
}

fn cuboid_portfolio_algorithms() -> Vec<(CuboidHeuristic3, CuboidAlgorithm3)> {
    vec![
        (
            CuboidHeuristic3::FirstFitDecreasingVolume,
            cuboid_first_fit_decreasing_volume_3d,
        ),
        (
            CuboidHeuristic3::BestFitDecreasingVolume,
            cuboid_best_fit_decreasing_volume_3d,
        ),
        (
            CuboidHeuristic3::FirstFitDecreasingMaxSide,
            cuboid_first_fit_decreasing_max_side_3d,
        ),
        (
            CuboidHeuristic3::BestFitDecreasingMaxSide,
            cuboid_best_fit_decreasing_max_side_3d,
        ),
        (
            CuboidHeuristic3::FirstFitDecreasingFootprintArea,
            cuboid_first_fit_decreasing_footprint_area_3d,
        ),
        (
            CuboidHeuristic3::BestFitDecreasingFootprintArea,
            cuboid_best_fit_decreasing_footprint_area_3d,
        ),
        (
            CuboidHeuristic3::ExtremePointDecreasingVolume,
            cuboid_extreme_point_decreasing_volume_3d,
        ),
        (
            CuboidHeuristic3::MaximalSpaceDecreasingVolume,
            cuboid_maximal_space_decreasing_volume_3d,
        ),
        (
            CuboidHeuristic3::GuillotineBestVolumeFit,
            cuboid_guillotine_best_volume_fit_3d,
        ),
        (
            CuboidHeuristic3::LaffLargestAreaFitFirst,
            cuboid_laff_largest_area_fit_first_3d,
        ),
    ]
}

fn sheet_report_better(candidate: &SheetHeuristicReport2, current: &SheetHeuristicReport2) -> bool {
    let candidate_rank = feasibility_rank(candidate.replay.status);
    let current_rank = feasibility_rank(current.replay.status);
    if candidate_rank != current_rank {
        return candidate_rank < current_rank;
    }
    if candidate.replay.objective.unplaced_items != current.replay.objective.unplaced_items {
        return candidate.replay.objective.unplaced_items < current.replay.objective.unplaced_items;
    }
    if gt(
        &candidate.replay.objective.used_area,
        &current.replay.objective.used_area,
    )
    .unwrap_or(false)
    {
        return true;
    }
    exact_eq(
        &candidate.replay.objective.used_area,
        &current.replay.objective.used_area,
    ) && candidate.replay.objective.duplicate_placements
        < current.replay.objective.duplicate_placements
}

fn cuboid_report_better(
    candidate: &CuboidHeuristicReport3,
    current: &CuboidHeuristicReport3,
) -> bool {
    let candidate_rank = feasibility_rank(candidate.replay.feasibility.status);
    let current_rank = feasibility_rank(current.replay.feasibility.status);
    if candidate_rank != current_rank {
        return candidate_rank < current_rank;
    }
    if candidate.replay.objective.unplaced_items != current.replay.objective.unplaced_items {
        return candidate.replay.objective.unplaced_items < current.replay.objective.unplaced_items;
    }
    if gt(
        &candidate.replay.objective.used_volume,
        &current.replay.objective.used_volume,
    )
    .unwrap_or(false)
    {
        return true;
    }
    exact_eq(
        &candidate.replay.objective.used_volume,
        &current.replay.objective.used_volume,
    ) && candidate.replay.objective.duplicate_placements
        < current.replay.objective.duplicate_placements
}

fn feasibility_rank(status: FeasibilityStatus) -> u8 {
    match status {
        FeasibilityStatus::Feasible => 0,
        FeasibilityStatus::Unknown => 1,
        FeasibilityStatus::Infeasible => 2,
    }
}

fn gt(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Positive => Some(true),
        RealSign::Zero | RealSign::Negative => Some(false),
    }
}

fn exact_eq(left: &Real, right: &Real) -> bool {
    matches!((left - right).refine_sign_until(-64), Some(RealSign::Zero))
}
