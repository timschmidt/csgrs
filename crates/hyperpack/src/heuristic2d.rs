//! Exact-aware 2D heuristic proposal reports.
//!
//! Shelf algorithms such as NFDH/FFDH/BFDH are classical fast rectangular
//! packing heuristics. `hyperpack` treats them as proposal engines: they may
//! produce coordinates cheaply, but the result is accepted only after exact
//! replay. Heuristic combinatorics remain separate from exact predicates.

use hyperreal::{Real, RealSign};

use crate::{
    FeasibilityStatus, ItemId, PackResult, Rect2, SheetBin2, SheetItem2, SheetPlacement2,
    SheetVerification2, verify_packing_2d,
};

/// 2D heuristic family implemented by this module.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SheetHeuristic2 {
    /// Next-fit decreasing-height shelf proposal.
    NextFitDecreasingHeight,
    /// First-fit decreasing-height shelf proposal.
    FirstFitDecreasingHeight,
    /// Best-fit decreasing-height shelf proposal.
    BestFitDecreasingHeight,
    /// Bottom-left skyline-style proposal.
    SkylineBottomLeft,
    /// Minimum-waste skyline-style proposal.
    SkylineMinimumWaste,
    /// MaxRects best-short-side-fit proposal.
    MaxRectsBestShortSideFit,
    /// MaxRects best-long-side-fit proposal.
    MaxRectsBestLongSideFit,
    /// MaxRects best-area-fit proposal.
    MaxRectsBestAreaFit,
    /// MaxRects bottom-left proposal.
    MaxRectsBottomLeft,
    /// MaxRects contact-point proposal.
    MaxRectsContactPoint,
    /// Guillotine best-area-fit proposal.
    GuillotineBestAreaFit,
    /// Guillotine best-short-side-fit proposal.
    GuillotineBestShortSideFit,
    /// Guillotine best-long-side-fit proposal.
    GuillotineBestLongSideFit,
}

/// Exact free rectangle emitted by a 2D heuristic trace.
#[derive(Clone, Debug, PartialEq)]
pub struct FreeRect2 {
    /// Exact x origin.
    pub x: Real,
    /// Exact y origin.
    pub y: Real,
    /// Exact width.
    pub width: Real,
    /// Exact height.
    pub height: Real,
}

/// Candidate placement emitted before exact replay acceptance.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementCandidate2 {
    /// Proposed placement.
    pub placement: SheetPlacement2,
    /// Exact item size used by this fixed-orientation candidate.
    pub size: Rect2,
    /// Shelf/skyline index that generated the candidate.
    pub shelf_index: usize,
}

/// Trace counters for a 2D heuristic proposal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SheetHeuristicTrace2 {
    /// Items considered in deterministic heuristic order.
    pub considered_items: usize,
    /// Candidate placements emitted.
    pub emitted_candidates: usize,
    /// Shelves opened by the proposal.
    pub opened_shelves: usize,
    /// Items rejected before replay because no shelf placement fit.
    pub rejected_items: usize,
    /// Exact comparisons performed by the proposal stage.
    pub exact_comparisons: usize,
    /// Candidate positions inspected by the proposal stage.
    pub candidate_positions: usize,
}

/// Full proposal plus exact replay report for a 2D heuristic.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetHeuristicReport2 {
    /// Heuristic family.
    pub heuristic: SheetHeuristic2,
    /// Candidate placements proposed before replay.
    pub candidates: Vec<PlacementCandidate2>,
    /// Free rectangles left by shelf rows and unstarted sheet space.
    pub free_rects: Vec<FreeRect2>,
    /// Trace counters.
    pub trace: SheetHeuristicTrace2,
    /// Exact replay of the emitted placements.
    pub replay: SheetVerification2,
    /// Item ids rejected by the proposal stage.
    pub rejected: Vec<ItemId>,
}

/// Proposes a fixed-orientation 2D layout with next-fit decreasing-height shelves.
///
/// Items are sorted by non-increasing exact height when the comparison can be
/// certified; uncertified ties retain source order. The heuristic only emits a
/// proposal. Consumers should inspect [`SheetHeuristicReport2::replay`] before
/// treating placements as feasible.
pub fn shelf_next_fit_decreasing_height_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    shelf_decreasing_height_2d(bin, items, SheetHeuristic2::NextFitDecreasingHeight)
}

/// Proposes a fixed-orientation 2D layout with first-fit decreasing-height shelves.
///
/// Each item is placed into the first existing shelf with enough certified
/// remaining width, opening a new shelf only when no previous shelf can accept
/// it. The resulting proposal is still replayed exactly before use.
pub fn shelf_first_fit_decreasing_height_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    shelf_decreasing_height_2d(bin, items, SheetHeuristic2::FirstFitDecreasingHeight)
}

/// Proposes a fixed-orientation 2D layout with best-fit decreasing-height shelves.
///
/// Among shelves that can accept an item, this chooses the shelf with least
/// certified remaining width after placement. Uncertified best-fit comparisons
/// keep deterministic first-seen order.
pub fn shelf_best_fit_decreasing_height_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    shelf_decreasing_height_2d(bin, items, SheetHeuristic2::BestFitDecreasingHeight)
}

/// Proposes a fixed-orientation 2D layout with bottom-left skyline candidates.
///
/// The candidate set is generated from exact edge events of already placed
/// rectangles. Among certified feasible candidates, the proposal chooses the
/// lowest `y` and then lowest `x`, a standard bottom-left heuristic rule. The
/// returned layout remains a proposal and is accepted only through exact replay.
pub fn skyline_bottom_left_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    let mut trace = SheetHeuristicTrace2 {
        considered_items: items.len(),
        emitted_candidates: 0,
        opened_shelves: 0,
        rejected_items: 0,
        exact_comparisons: 0,
        candidate_positions: 0,
    };
    let mut candidates = Vec::<PlacementCandidate2>::new();
    let mut rejected = Vec::new();
    let mut free_rects = Vec::new();

    for item in items {
        trace.exact_comparisons += 2;
        if !leq(&item.size.x, &bin.size.x).unwrap_or(false)
            || !leq(&item.size.y, &bin.size.y).unwrap_or(false)
        {
            rejected.push(item.id.clone());
            trace.rejected_items += 1;
            continue;
        }

        let mut best = None::<(Real, Real, usize)>;
        for (point_index, (x, y)) in skyline_candidate_points(&candidates)
            .into_iter()
            .enumerate()
        {
            trace.candidate_positions += 1;
            if !candidate_fits(bin, item, &candidates, &x, &y, &mut trace) {
                continue;
            }
            match &best {
                None => best = Some((x, y, point_index)),
                Some((best_x, best_y, _)) => {
                    trace.exact_comparisons += 1;
                    let lower_y = lt(&y, best_y).unwrap_or(false);
                    trace.exact_comparisons += 1;
                    let same_y_lower_x = exact_eq(&y, best_y) && lt(&x, best_x).unwrap_or(false);
                    if lower_y || same_y_lower_x {
                        best = Some((x, y, point_index));
                    }
                }
            }
        }

        match best {
            Some((x, y, point_index)) => {
                candidates.push(PlacementCandidate2 {
                    placement: SheetPlacement2 {
                        item: item.id.clone(),
                        x: x.clone(),
                        y: y.clone(),
                    },
                    size: item.size.clone(),
                    shelf_index: point_index,
                });
                trace.emitted_candidates += 1;
                trace.opened_shelves = trace.opened_shelves.max(point_index + 1);
                push_skyline_residuals(&mut free_rects, bin, item, &x, &y);
            }
            None => {
                rejected.push(item.id.clone());
                trace.rejected_items += 1;
            }
        }
    }

    let placements = candidates
        .iter()
        .map(|candidate| candidate.placement.clone())
        .collect::<Vec<_>>();
    let mut replay = verify_packing_2d(bin, items, &placements)?;
    if !rejected.is_empty() && replay.status == FeasibilityStatus::Feasible {
        replay
            .facts
            .push("proposal rejected at least one item before replay".into());
    }

    Ok(SheetHeuristicReport2 {
        heuristic: SheetHeuristic2::SkylineBottomLeft,
        candidates,
        free_rects,
        trace,
        replay,
        rejected,
    })
}

/// Proposes a fixed-orientation 2D layout with minimum-waste skyline candidates.
///
/// The candidate set is the same exact edge-event set used by
/// [`skyline_bottom_left_2d`]. Among certified feasible candidates, the
/// heuristic minimizes exact enclosing-rectangle residual area,
/// `(x + width) * (y + height) - item_area`, then uses bottom-left
/// tie-breaking. As with all heuristic paths here, the result is accepted only
/// through exact replay.
pub fn skyline_minimum_waste_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    let mut trace = SheetHeuristicTrace2 {
        considered_items: items.len(),
        emitted_candidates: 0,
        opened_shelves: 0,
        rejected_items: 0,
        exact_comparisons: 0,
        candidate_positions: 0,
    };
    let mut candidates = Vec::<PlacementCandidate2>::new();
    let mut rejected = Vec::new();
    let mut free_rects = Vec::new();

    for item in items {
        trace.exact_comparisons += 2;
        if !leq(&item.size.x, &bin.size.x).unwrap_or(false)
            || !leq(&item.size.y, &bin.size.y).unwrap_or(false)
        {
            rejected.push(item.id.clone());
            trace.rejected_items += 1;
            continue;
        }

        let mut best = None::<SkylineWasteChoice2>;
        for (point_index, (x, y)) in skyline_candidate_points(&candidates)
            .into_iter()
            .enumerate()
        {
            trace.candidate_positions += 1;
            if !candidate_fits(bin, item, &candidates, &x, &y, &mut trace) {
                continue;
            }
            let waste = skyline_candidate_waste(item, &x, &y);
            let choice = SkylineWasteChoice2 {
                x,
                y,
                point_index,
                waste,
            };
            match &best {
                None => best = Some(choice),
                Some(current) => {
                    trace.exact_comparisons += 1;
                    let lower_waste = lt(&choice.waste, &current.waste).unwrap_or(false);
                    trace.exact_comparisons += 1;
                    let same_waste = exact_eq(&choice.waste, &current.waste);
                    if lower_waste
                        || (same_waste && skyline_bottom_left_better(&choice, current, &mut trace))
                    {
                        best = Some(choice);
                    }
                }
            }
        }

        match best {
            Some(choice) => {
                candidates.push(PlacementCandidate2 {
                    placement: SheetPlacement2 {
                        item: item.id.clone(),
                        x: choice.x.clone(),
                        y: choice.y.clone(),
                    },
                    size: item.size.clone(),
                    shelf_index: choice.point_index,
                });
                trace.emitted_candidates += 1;
                trace.opened_shelves = trace.opened_shelves.max(choice.point_index + 1);
                push_skyline_residuals(&mut free_rects, bin, item, &choice.x, &choice.y);
            }
            None => {
                rejected.push(item.id.clone());
                trace.rejected_items += 1;
            }
        }
    }

    let placements = candidates
        .iter()
        .map(|candidate| candidate.placement.clone())
        .collect::<Vec<_>>();
    let mut replay = verify_packing_2d(bin, items, &placements)?;
    if !rejected.is_empty() && replay.status == FeasibilityStatus::Feasible {
        replay
            .facts
            .push("proposal rejected at least one item before replay".into());
    }

    Ok(SheetHeuristicReport2 {
        heuristic: SheetHeuristic2::SkylineMinimumWaste,
        candidates,
        free_rects,
        trace,
        replay,
        rejected,
    })
}

/// Proposes a fixed-orientation 2D layout with a MaxRects best-short-side-fit scan.
///
/// Chooses a free rectangle minimizing the shorter leftover side, then the
/// longer leftover side. The free-rectangle split is intentionally conservative
/// and report-bearing; exact replay remains the acceptance gate.
pub fn maxrects_best_short_side_fit_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    maxrects_2d(bin, items, SheetHeuristic2::MaxRectsBestShortSideFit)
}

/// Proposes a fixed-orientation 2D layout with MaxRects best-long-side-fit scoring.
///
/// This uses the same exact free-rectangle scan as
/// [`maxrects_best_short_side_fit_2d`], but prioritizes the longer leftover
/// side before the shorter leftover side. The layout is still only accepted
/// after exact replay.
pub fn maxrects_best_long_side_fit_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    maxrects_2d(bin, items, SheetHeuristic2::MaxRectsBestLongSideFit)
}

/// Proposes a fixed-orientation 2D layout with MaxRects best-area-fit scoring.
///
/// This follows the common MaxRects BAF rule: minimize exact free-area waste
/// first, then exact short-side residual as a deterministic tie-breaker. Exact
/// replay remains the acceptance gate.
pub fn maxrects_best_area_fit_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    maxrects_2d(bin, items, SheetHeuristic2::MaxRectsBestAreaFit)
}

/// Proposes a fixed-orientation 2D layout with MaxRects bottom-left scoring.
///
/// This scans the same exact free-rectangle set as the other MaxRects variants
/// and chooses the feasible free rectangle with lowest exact `y`, then lowest
/// exact `x`. Exact replay remains the acceptance gate.
pub fn maxrects_bottom_left_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    maxrects_2d(bin, items, SheetHeuristic2::MaxRectsBottomLeft)
}

/// Proposes a fixed-orientation 2D layout with MaxRects contact-point scoring.
///
/// Maximizes exact edge contact against the bin boundary and already placed
/// rectangles, then uses exact short/long-side residuals as deterministic
/// tie-breakers. The contact score is proposal evidence; exact replay decides
/// whether the emitted placement set is accepted.
pub fn maxrects_contact_point_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    maxrects_2d(bin, items, SheetHeuristic2::MaxRectsContactPoint)
}

/// Proposes a fixed-orientation 2D layout with guillotine best-area-fit splits.
///
/// Guillotine cutting recursively splits the remaining sheet with full-width or
/// full-height cuts, a classical restriction in cutting and packing. This
/// proposal chooses the feasible free rectangle with least exact area waste,
/// then splits the used rectangle into exact right and top residual rectangles.
/// Guillotine structure is a combinatorial restriction; feasibility is still
/// accepted only after exact replay.
pub fn guillotine_best_area_fit_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    guillotine_2d(bin, items, SheetHeuristic2::GuillotineBestAreaFit)
}

/// Proposes a fixed-orientation 2D layout with guillotine best-short-side-fit splits.
///
/// This uses the same exact full-cut residual state as
/// [`guillotine_best_area_fit_2d`], but selects the feasible free rectangle
/// minimizing the shorter exact leftover side before area waste. The score only
/// chooses a proposal; exact replay remains authoritative.
pub fn guillotine_best_short_side_fit_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    guillotine_2d(bin, items, SheetHeuristic2::GuillotineBestShortSideFit)
}

/// Proposes a fixed-orientation 2D layout with guillotine best-long-side-fit splits.
///
/// This selects the feasible free rectangle minimizing the longer exact
/// leftover side before area waste. The retained free rectangles are
/// guillotine scheduling evidence, not proof; acceptance still comes from
/// [`verify_packing_2d`].
pub fn guillotine_best_long_side_fit_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
) -> PackResult<SheetHeuristicReport2> {
    guillotine_2d(bin, items, SheetHeuristic2::GuillotineBestLongSideFit)
}

fn maxrects_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
    heuristic: SheetHeuristic2,
) -> PackResult<SheetHeuristicReport2> {
    let mut trace = SheetHeuristicTrace2 {
        considered_items: items.len(),
        emitted_candidates: 0,
        opened_shelves: 0,
        rejected_items: 0,
        exact_comparisons: 0,
        candidate_positions: 0,
    };
    let mut free_rects = vec![FreeRect2 {
        x: Real::zero(),
        y: Real::zero(),
        width: bin.size.x.clone(),
        height: bin.size.y.clone(),
    }];
    let mut candidates = Vec::new();
    let mut rejected = Vec::new();

    for item in items {
        trace.exact_comparisons += 2;
        if !leq(&item.size.x, &bin.size.x).unwrap_or(false)
            || !leq(&item.size.y, &bin.size.y).unwrap_or(false)
        {
            rejected.push(item.id.clone());
            trace.rejected_items += 1;
            continue;
        }

        let mut best = None::<MaxRectsChoice2>;
        for (free_index, free) in free_rects.iter().enumerate() {
            trace.candidate_positions += 1;
            trace.exact_comparisons += 2;
            if !leq(&item.size.x, &free.width).unwrap_or(false)
                || !leq(&item.size.y, &free.height).unwrap_or(false)
            {
                continue;
            }
            let width_left = free.width.clone() - item.size.x.clone();
            let height_left = free.height.clone() - item.size.y.clone();
            let short_side = min_exact(&width_left, &height_left, &mut trace);
            let long_side = max_exact(&width_left, &height_left, &mut trace);
            let area_fit = free.width.clone() * free.height.clone() - item.size.area();
            let contact_score = maxrects_contact_score(bin, item, free, &candidates, &mut trace);
            let choice = MaxRectsChoice2 {
                free_index,
                x: free.x.clone(),
                y: free.y.clone(),
                width_left,
                height_left,
                short_side,
                long_side,
                area_fit,
                contact_score,
            };
            match &best {
                None => best = Some(choice),
                Some(current) => {
                    if maxrects_choice_better(&choice, current, heuristic, &mut trace) {
                        best = Some(choice);
                    }
                }
            }
        }

        match best {
            Some(choice) => {
                let used = free_rects.remove(choice.free_index);
                candidates.push(PlacementCandidate2 {
                    placement: SheetPlacement2 {
                        item: item.id.clone(),
                        x: choice.x.clone(),
                        y: choice.y.clone(),
                    },
                    size: item.size.clone(),
                    shelf_index: choice.free_index,
                });
                trace.emitted_candidates += 1;
                split_maxrects_free_rect(&mut free_rects, &used, item, &choice);
                trace.opened_shelves = free_rects.len();
            }
            None => {
                rejected.push(item.id.clone());
                trace.rejected_items += 1;
            }
        }
    }

    let placements = candidates
        .iter()
        .map(|candidate| candidate.placement.clone())
        .collect::<Vec<_>>();
    let mut replay = verify_packing_2d(bin, items, &placements)?;
    if !rejected.is_empty() && replay.status == FeasibilityStatus::Feasible {
        replay
            .facts
            .push("proposal rejected at least one item before replay".into());
    }

    Ok(SheetHeuristicReport2 {
        heuristic,
        candidates,
        free_rects,
        trace,
        replay,
        rejected,
    })
}

fn guillotine_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
    heuristic: SheetHeuristic2,
) -> PackResult<SheetHeuristicReport2> {
    let mut trace = SheetHeuristicTrace2 {
        considered_items: items.len(),
        emitted_candidates: 0,
        opened_shelves: 0,
        rejected_items: 0,
        exact_comparisons: 0,
        candidate_positions: 0,
    };
    let mut free_rects = vec![FreeRect2 {
        x: Real::zero(),
        y: Real::zero(),
        width: bin.size.x.clone(),
        height: bin.size.y.clone(),
    }];
    let mut candidates = Vec::new();
    let mut rejected = Vec::new();

    for item in items {
        let mut best = None::<GuillotineChoice2>;
        for (free_index, free) in free_rects.iter().enumerate() {
            trace.candidate_positions += 1;
            trace.exact_comparisons += 2;
            if !leq(&item.size.x, &free.width).unwrap_or(false)
                || !leq(&item.size.y, &free.height).unwrap_or(false)
            {
                continue;
            }
            let area_waste = free.width.clone() * free.height.clone() - item.size.area();
            let short_side = min_exact(
                &(free.width.clone() - item.size.x.clone()),
                &(free.height.clone() - item.size.y.clone()),
                &mut trace,
            );
            let long_side = max_exact(
                &(free.width.clone() - item.size.x.clone()),
                &(free.height.clone() - item.size.y.clone()),
                &mut trace,
            );
            let choice = GuillotineChoice2 {
                free_index,
                x: free.x.clone(),
                y: free.y.clone(),
                area_waste,
                short_side,
                long_side,
            };
            match &best {
                None => best = Some(choice),
                Some(current) => {
                    if guillotine_choice_better(&choice, current, heuristic, &mut trace) {
                        best = Some(choice);
                    }
                }
            }
        }

        match best {
            Some(choice) => {
                let used = free_rects.remove(choice.free_index);
                candidates.push(PlacementCandidate2 {
                    placement: SheetPlacement2 {
                        item: item.id.clone(),
                        x: choice.x.clone(),
                        y: choice.y.clone(),
                    },
                    size: item.size.clone(),
                    shelf_index: choice.free_index,
                });
                trace.emitted_candidates += 1;
                split_guillotine_free_rect(&mut free_rects, &used, item, &choice);
                trace.opened_shelves = free_rects.len();
            }
            None => {
                rejected.push(item.id.clone());
                trace.rejected_items += 1;
            }
        }
    }

    let placements = candidates
        .iter()
        .map(|candidate| candidate.placement.clone())
        .collect::<Vec<_>>();
    let mut replay = verify_packing_2d(bin, items, &placements)?;
    if !rejected.is_empty() && replay.status == FeasibilityStatus::Feasible {
        replay
            .facts
            .push("proposal rejected at least one item before replay".into());
    }

    Ok(SheetHeuristicReport2 {
        heuristic,
        candidates,
        free_rects,
        trace,
        replay,
        rejected,
    })
}

fn shelf_decreasing_height_2d(
    bin: &SheetBin2,
    items: &[SheetItem2],
    heuristic: SheetHeuristic2,
) -> PackResult<SheetHeuristicReport2> {
    let mut trace = SheetHeuristicTrace2 {
        considered_items: items.len(),
        emitted_candidates: 0,
        opened_shelves: 0,
        rejected_items: 0,
        exact_comparisons: 0,
        candidate_positions: 0,
    };
    let mut ordered = items.iter().enumerate().collect::<Vec<_>>();
    ordered.sort_by(|(left_index, left), (right_index, right)| {
        trace.exact_comparisons += 1;
        match compare_desc(&left.size.y, &right.size.y) {
            Some(ordering) => ordering.then_with(|| left_index.cmp(right_index)),
            None => left_index.cmp(right_index),
        }
    });

    let mut shelves = Vec::<ShelfState2>::new();
    let mut candidates = Vec::new();
    let mut rejected = Vec::new();
    let mut next_shelf_y = Real::zero();

    for (_, item) in ordered {
        trace.exact_comparisons += 2;
        if !leq(&item.size.x, &bin.size.x).unwrap_or(false)
            || !leq(&item.size.y, &bin.size.y).unwrap_or(false)
        {
            rejected.push(item.id.clone());
            trace.rejected_items += 1;
            continue;
        }

        let selected = select_shelf(bin, &shelves, item, heuristic, &mut trace);
        let selected = match selected {
            Some(index) => index,
            None => {
                trace.exact_comparisons += 1;
                if !leq(&(next_shelf_y.clone() + item.size.y.clone()), &bin.size.y).unwrap_or(false)
                {
                    rejected.push(item.id.clone());
                    trace.rejected_items += 1;
                    continue;
                }
                let index = shelves.len();
                shelves.push(ShelfState2 {
                    cursor_x: Real::zero(),
                    y: next_shelf_y.clone(),
                    height: item.size.y.clone(),
                    index,
                });
                next_shelf_y += item.size.y.clone();
                trace.opened_shelves += 1;
                index
            }
        };

        trace.exact_comparisons += 1;
        if !leq(
            &(shelves[selected].cursor_x.clone() + item.size.x.clone()),
            &bin.size.x,
        )
        .unwrap_or(false)
        {
            rejected.push(item.id.clone());
            trace.rejected_items += 1;
            continue;
        }

        candidates.push(PlacementCandidate2 {
            placement: SheetPlacement2 {
                item: item.id.clone(),
                x: shelves[selected].cursor_x.clone(),
                y: shelves[selected].y.clone(),
            },
            size: item.size.clone(),
            shelf_index: shelves[selected].index,
        });
        trace.emitted_candidates += 1;
        shelves[selected].cursor_x = shelves[selected].cursor_x.clone() + item.size.x.clone();
    }

    let mut free_rects = Vec::new();
    for shelf in &shelves {
        push_shelf_remainder(
            &mut free_rects,
            &shelf.cursor_x,
            &shelf.y,
            &bin.size.x,
            &shelf.height,
        );
    }
    trace.exact_comparisons += 1;
    if leq(&next_shelf_y, &bin.size.y).unwrap_or(false) {
        let remaining_height = bin.size.y.clone() - next_shelf_y.clone();
        trace.exact_comparisons += 1;
        if positive(&remaining_height).unwrap_or(false) {
            free_rects.push(FreeRect2 {
                x: Real::zero(),
                y: next_shelf_y,
                width: bin.size.x.clone(),
                height: remaining_height,
            });
        }
    }

    let placements = candidates
        .iter()
        .map(|candidate| candidate.placement.clone())
        .collect::<Vec<_>>();
    let replay = verify_packing_2d(bin, items, &placements)?;
    let mut replay = replay;
    if !rejected.is_empty() && replay.status == FeasibilityStatus::Feasible {
        replay
            .facts
            .push("proposal rejected at least one item before replay".into());
    }

    Ok(SheetHeuristicReport2 {
        heuristic,
        candidates,
        free_rects,
        trace,
        replay,
        rejected,
    })
}

fn skyline_candidate_points(candidates: &[PlacementCandidate2]) -> Vec<(Real, Real)> {
    let mut points = vec![(Real::zero(), Real::zero())];
    for candidate in candidates {
        points.push((
            candidate.placement.x.clone() + candidate.size.x.clone(),
            candidate.placement.y.clone(),
        ));
        points.push((
            candidate.placement.x.clone(),
            candidate.placement.y.clone() + candidate.size.y.clone(),
        ));
    }
    points
}

fn candidate_fits(
    bin: &SheetBin2,
    item: &SheetItem2,
    candidates: &[PlacementCandidate2],
    x: &Real,
    y: &Real,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    trace.exact_comparisons += 4;
    if !nonnegative(x).unwrap_or(false)
        || !nonnegative(y).unwrap_or(false)
        || !leq(&(x.clone() + item.size.x.clone()), &bin.size.x).unwrap_or(false)
        || !leq(&(y.clone() + item.size.y.clone()), &bin.size.y).unwrap_or(false)
    {
        return false;
    }
    for candidate in candidates {
        trace.exact_comparisons += 4;
        if !rects_disjoint(
            x,
            y,
            &item.size,
            &candidate.placement.x,
            &candidate.placement.y,
            &candidate.size,
        )
        .unwrap_or(false)
        {
            return false;
        }
    }
    true
}

fn push_skyline_residuals(
    free_rects: &mut Vec<FreeRect2>,
    bin: &SheetBin2,
    item: &SheetItem2,
    x: &Real,
    y: &Real,
) {
    let right_width = bin.size.x.clone() - (x.clone() + item.size.x.clone());
    if positive(&right_width).unwrap_or(false) && positive(&item.size.y).unwrap_or(false) {
        free_rects.push(FreeRect2 {
            x: x.clone() + item.size.x.clone(),
            y: y.clone(),
            width: right_width,
            height: item.size.y.clone(),
        });
    }
    let top_height = bin.size.y.clone() - (y.clone() + item.size.y.clone());
    if positive(&top_height).unwrap_or(false) && positive(&item.size.x).unwrap_or(false) {
        free_rects.push(FreeRect2 {
            x: x.clone(),
            y: y.clone() + item.size.y.clone(),
            width: item.size.x.clone(),
            height: top_height,
        });
    }
}

#[derive(Clone, Debug, PartialEq)]
struct SkylineWasteChoice2 {
    x: Real,
    y: Real,
    point_index: usize,
    waste: Real,
}

#[derive(Clone, Debug, PartialEq)]
struct MaxRectsChoice2 {
    free_index: usize,
    x: Real,
    y: Real,
    width_left: Real,
    height_left: Real,
    short_side: Real,
    long_side: Real,
    area_fit: Real,
    contact_score: Real,
}

#[derive(Clone, Debug, PartialEq)]
struct GuillotineChoice2 {
    free_index: usize,
    x: Real,
    y: Real,
    area_waste: Real,
    short_side: Real,
    long_side: Real,
}

fn guillotine_choice_better(
    choice: &GuillotineChoice2,
    current: &GuillotineChoice2,
    heuristic: SheetHeuristic2,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    match heuristic {
        SheetHeuristic2::GuillotineBestAreaFit => score_pair_better(
            &choice.area_waste,
            &current.area_waste,
            &choice.short_side,
            &current.short_side,
            trace,
        ),
        SheetHeuristic2::GuillotineBestShortSideFit => score_pair_better(
            &choice.short_side,
            &current.short_side,
            &choice.area_waste,
            &current.area_waste,
            trace,
        ),
        SheetHeuristic2::GuillotineBestLongSideFit => score_pair_better(
            &choice.long_side,
            &current.long_side,
            &choice.area_waste,
            &current.area_waste,
            trace,
        ),
        _ => false,
    }
}

fn maxrects_choice_better(
    choice: &MaxRectsChoice2,
    current: &MaxRectsChoice2,
    heuristic: SheetHeuristic2,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    match heuristic {
        SheetHeuristic2::MaxRectsBestShortSideFit => score_pair_better(
            &choice.short_side,
            &current.short_side,
            &choice.long_side,
            &current.long_side,
            trace,
        ),
        SheetHeuristic2::MaxRectsBestLongSideFit => score_pair_better(
            &choice.long_side,
            &current.long_side,
            &choice.short_side,
            &current.short_side,
            trace,
        ),
        SheetHeuristic2::MaxRectsBestAreaFit => score_pair_better(
            &choice.area_fit,
            &current.area_fit,
            &choice.short_side,
            &current.short_side,
            trace,
        ),
        SheetHeuristic2::MaxRectsBottomLeft => {
            score_pair_better(&choice.y, &current.y, &choice.x, &current.x, trace)
        }
        SheetHeuristic2::MaxRectsContactPoint => {
            score_pair_greater_then_less(
                &choice.contact_score,
                &current.contact_score,
                &choice.short_side,
                &current.short_side,
                trace,
            ) || score_triple_contact_tie_better(choice, current, trace)
        }
        _ => false,
    }
}

fn score_pair_better(
    left_primary: &Real,
    right_primary: &Real,
    left_secondary: &Real,
    right_secondary: &Real,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    trace.exact_comparisons += 1;
    let better_primary = lt(left_primary, right_primary).unwrap_or(false);
    trace.exact_comparisons += 1;
    let same_primary = exact_eq(left_primary, right_primary);
    trace.exact_comparisons += 1;
    let better_secondary = lt(left_secondary, right_secondary).unwrap_or(false);
    better_primary || (same_primary && better_secondary)
}

fn score_pair_greater_then_less(
    left_primary: &Real,
    right_primary: &Real,
    left_secondary: &Real,
    right_secondary: &Real,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    trace.exact_comparisons += 1;
    let better_primary = gt(left_primary, right_primary).unwrap_or(false);
    trace.exact_comparisons += 1;
    let same_primary = exact_eq(left_primary, right_primary);
    trace.exact_comparisons += 1;
    let better_secondary = lt(left_secondary, right_secondary).unwrap_or(false);
    better_primary || (same_primary && better_secondary)
}

fn score_triple_contact_tie_better(
    choice: &MaxRectsChoice2,
    current: &MaxRectsChoice2,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    trace.exact_comparisons += 2;
    exact_eq(&choice.contact_score, &current.contact_score)
        && exact_eq(&choice.short_side, &current.short_side)
        && score_pair_better(
            &choice.long_side,
            &current.long_side,
            &choice.y,
            &current.y,
            trace,
        )
}

fn maxrects_contact_score(
    bin: &SheetBin2,
    item: &SheetItem2,
    free: &FreeRect2,
    candidates: &[PlacementCandidate2],
    trace: &mut SheetHeuristicTrace2,
) -> Real {
    let x0 = free.x.clone();
    let y0 = free.y.clone();
    let x1 = x0.clone() + item.size.x.clone();
    let y1 = y0.clone() + item.size.y.clone();
    let mut score = Real::zero();

    trace.exact_comparisons += 4;
    if exact_eq_zero(&x0) {
        score += item.size.y.clone();
    }
    if exact_eq_zero(&y0) {
        score += item.size.x.clone();
    }
    if exact_eq(&x1, &bin.size.x) {
        score += item.size.y.clone();
    }
    if exact_eq(&y1, &bin.size.y) {
        score += item.size.x.clone();
    }

    for candidate in candidates {
        let placed_x0 = candidate.placement.x.clone();
        let placed_y0 = candidate.placement.y.clone();
        let placed_x1 = placed_x0.clone() + candidate.size.x.clone();
        let placed_y1 = placed_y0.clone() + candidate.size.y.clone();

        trace.exact_comparisons += 2;
        if exact_eq(&x1, &placed_x0) || exact_eq(&placed_x1, &x0) {
            score += exact_interval_overlap_length(&y0, &y1, &placed_y0, &placed_y1, trace);
        }
        trace.exact_comparisons += 2;
        if exact_eq(&y1, &placed_y0) || exact_eq(&placed_y1, &y0) {
            score += exact_interval_overlap_length(&x0, &x1, &placed_x0, &placed_x1, trace);
        }
    }

    score
}

fn exact_interval_overlap_length(
    left_start: &Real,
    left_end: &Real,
    right_start: &Real,
    right_end: &Real,
    trace: &mut SheetHeuristicTrace2,
) -> Real {
    let start = max_exact(left_start, right_start, trace);
    let end = min_exact(left_end, right_end, trace);
    let length = end - start;
    trace.exact_comparisons += 1;
    if positive(&length).unwrap_or(false) {
        length
    } else {
        Real::zero()
    }
}

fn split_maxrects_free_rect(
    free_rects: &mut Vec<FreeRect2>,
    used: &FreeRect2,
    item: &SheetItem2,
    choice: &MaxRectsChoice2,
) {
    if positive(&choice.width_left).unwrap_or(false) && positive(&used.height).unwrap_or(false) {
        free_rects.push(FreeRect2 {
            x: choice.x.clone() + item.size.x.clone(),
            y: choice.y.clone(),
            width: choice.width_left.clone(),
            height: used.height.clone(),
        });
    }
    if positive(&choice.height_left).unwrap_or(false) && positive(&item.size.x).unwrap_or(false) {
        free_rects.push(FreeRect2 {
            x: choice.x.clone(),
            y: choice.y.clone() + item.size.y.clone(),
            width: item.size.x.clone(),
            height: choice.height_left.clone(),
        });
    }
}

fn split_guillotine_free_rect(
    free_rects: &mut Vec<FreeRect2>,
    used: &FreeRect2,
    item: &SheetItem2,
    choice: &GuillotineChoice2,
) {
    let width_left = used.width.clone() - item.size.x.clone();
    let height_left = used.height.clone() - item.size.y.clone();
    if positive(&width_left).unwrap_or(false) {
        free_rects.push(FreeRect2 {
            x: choice.x.clone() + item.size.x.clone(),
            y: choice.y.clone(),
            width: width_left,
            height: item.size.y.clone(),
        });
    }
    if positive(&height_left).unwrap_or(false) {
        free_rects.push(FreeRect2 {
            x: choice.x.clone(),
            y: choice.y.clone() + item.size.y.clone(),
            width: used.width.clone(),
            height: height_left,
        });
    }
}

fn skyline_candidate_waste(item: &SheetItem2, x: &Real, y: &Real) -> Real {
    (x.clone() + item.size.x.clone()) * (y.clone() + item.size.y.clone()) - item.size.area()
}

fn skyline_bottom_left_better(
    candidate: &SkylineWasteChoice2,
    current: &SkylineWasteChoice2,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    trace.exact_comparisons += 1;
    let lower_y = lt(&candidate.y, &current.y).unwrap_or(false);
    trace.exact_comparisons += 1;
    let same_y_lower_x =
        exact_eq(&candidate.y, &current.y) && lt(&candidate.x, &current.x).unwrap_or(false);
    lower_y || same_y_lower_x
}

#[derive(Clone, Debug, PartialEq)]
struct ShelfState2 {
    cursor_x: Real,
    y: Real,
    height: Real,
    index: usize,
}

fn select_shelf(
    bin: &SheetBin2,
    shelves: &[ShelfState2],
    item: &SheetItem2,
    heuristic: SheetHeuristic2,
    trace: &mut SheetHeuristicTrace2,
) -> Option<usize> {
    match heuristic {
        SheetHeuristic2::NextFitDecreasingHeight => {
            let last = shelves.len().checked_sub(1)?;
            if shelf_accepts(bin, &shelves[last], item, trace) {
                Some(last)
            } else {
                None
            }
        }
        SheetHeuristic2::FirstFitDecreasingHeight => shelves
            .iter()
            .enumerate()
            .find_map(|(index, shelf)| shelf_accepts(bin, shelf, item, trace).then_some(index)),
        SheetHeuristic2::BestFitDecreasingHeight => {
            let mut best = None::<(usize, Real)>;
            for (index, shelf) in shelves.iter().enumerate() {
                if !shelf_accepts(bin, shelf, item, trace) {
                    continue;
                }
                let remainder = bin.size.x.clone() - (shelf.cursor_x.clone() + item.size.x.clone());
                match &best {
                    None => best = Some((index, remainder)),
                    Some((_, best_remainder)) => {
                        trace.exact_comparisons += 1;
                        if lt(&remainder, best_remainder).unwrap_or(false) {
                            best = Some((index, remainder));
                        }
                    }
                }
            }
            best.map(|(index, _)| index)
        }
        SheetHeuristic2::SkylineBottomLeft => None,
        SheetHeuristic2::SkylineMinimumWaste => None,
        SheetHeuristic2::MaxRectsBestShortSideFit => None,
        SheetHeuristic2::MaxRectsBestLongSideFit => None,
        SheetHeuristic2::MaxRectsBestAreaFit => None,
        SheetHeuristic2::MaxRectsBottomLeft => None,
        SheetHeuristic2::MaxRectsContactPoint => None,
        SheetHeuristic2::GuillotineBestAreaFit => None,
        SheetHeuristic2::GuillotineBestShortSideFit => None,
        SheetHeuristic2::GuillotineBestLongSideFit => None,
    }
}

fn shelf_accepts(
    bin: &SheetBin2,
    shelf: &ShelfState2,
    item: &SheetItem2,
    trace: &mut SheetHeuristicTrace2,
) -> bool {
    trace.exact_comparisons += 2;
    leq(&(shelf.cursor_x.clone() + item.size.x.clone()), &bin.size.x).unwrap_or(false)
        && leq(&item.size.y, &shelf.height).unwrap_or(false)
}

fn push_shelf_remainder(
    free_rects: &mut Vec<FreeRect2>,
    cursor_x: &Real,
    cursor_y: &Real,
    bin_width: &Real,
    shelf_height: &Real,
) {
    let width = bin_width.clone() - cursor_x.clone();
    if positive(&width).unwrap_or(false) && positive(shelf_height).unwrap_or(false) {
        free_rects.push(FreeRect2 {
            x: cursor_x.clone(),
            y: cursor_y.clone(),
            width,
            height: shelf_height.clone(),
        });
    }
}

fn compare_desc(left: &Real, right: &Real) -> Option<std::cmp::Ordering> {
    crate::predicate::compare(left, right).map(std::cmp::Ordering::reverse)
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    Some(!crate::predicate::compare(left, right)?.is_gt())
}

fn lt(left: &Real, right: &Real) -> Option<bool> {
    Some(crate::predicate::compare(left, right)?.is_lt())
}

fn gt(left: &Real, right: &Real) -> Option<bool> {
    Some(crate::predicate::compare(left, right)?.is_gt())
}

fn exact_eq(left: &Real, right: &Real) -> bool {
    crate::predicate::equal(left, right)
}

fn exact_eq_zero(value: &Real) -> bool {
    matches!(crate::predicate::sign(value), Some(RealSign::Zero))
}

fn min_exact(left: &Real, right: &Real, trace: &mut SheetHeuristicTrace2) -> Real {
    trace.exact_comparisons += 1;
    if leq(left, right).unwrap_or(false) {
        left.clone()
    } else {
        right.clone()
    }
}

fn max_exact(left: &Real, right: &Real, trace: &mut SheetHeuristicTrace2) -> Real {
    trace.exact_comparisons += 1;
    if leq(left, right).unwrap_or(false) {
        right.clone()
    } else {
        left.clone()
    }
}

fn nonnegative(value: &Real) -> Option<bool> {
    match crate::predicate::sign(value)? {
        RealSign::Negative => Some(false),
        RealSign::Zero | RealSign::Positive => Some(true),
    }
}

fn rects_disjoint(
    left_x: &Real,
    left_y: &Real,
    left_size: &Rect2,
    right_x: &Real,
    right_y: &Real,
    right_size: &Rect2,
) -> Option<bool> {
    crate::predicate::decide_any!(
        leq(&(left_x.clone() + left_size.x.clone()), right_x),
        leq(&(right_x.clone() + right_size.x.clone()), left_x),
        leq(&(left_y.clone() + left_size.y.clone()), right_y),
        leq(&(right_y.clone() + right_size.y.clone()), left_y),
    )
}

fn positive(value: &Real) -> Option<bool> {
    match crate::predicate::sign(value)? {
        RealSign::Positive => Some(true),
        RealSign::Negative | RealSign::Zero => Some(false),
    }
}
