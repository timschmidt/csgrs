//! Replay-gated local search schedulers for packing proposals.
//!
//! Local search changes combinatorial decisions such as item order; it does not
//! certify geometry by itself. Following Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>), every candidate order here
//! is converted to exact placements and then replayed before objective
//! comparison. The move neighborhood follows standard local-search operations
//! used in bin-packing metaheuristics such as tabu/guided local search; see
//! Glover, "Tabu Search - Part I," *ORSA Journal on Computing* 1(3), 1989 for
//! the search-neighborhood framing. Seeded multistart uses deterministic
//! randomized order proposals in the spirit of multi-start metaheuristics; the
//! seed affects only proposal order, never feasibility certification.

use hyperreal::{Real, RealSign};

use crate::{
    Bin3, BinId, BinInstance3, Item3, ItemId, MultiBinPlacement3, MultiBinVerification3,
    PackResult, PackingVerification3, Placement3, verify_multi_bin_packing_3d, verify_packing_3d,
};

/// Local-search move over an item-order permutation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum OrderMove3 {
    /// Swap two order positions.
    Swap { left: usize, right: usize },
    /// Remove `from` and insert before `to` after removal.
    Insert { from: usize, to: usize },
    /// Reverse an inclusive order range.
    Reverse { start: usize, end: usize },
}

/// Limits for deterministic local search over 3D item order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LocalSearchConfig3 {
    /// Maximum accepted-improvement passes.
    pub max_steps: usize,
    /// Maximum neighbor moves inspected per pass.
    pub max_neighbors_per_step: usize,
}

/// Local-search completion status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LocalSearchStatus3 {
    /// No improving neighbor was found before a limit interrupted the search.
    LocalOptimum,
    /// Search stopped after reaching `max_steps`.
    StepLimit,
    /// Search stopped after reaching `max_neighbors_per_step` in a pass.
    NeighborLimit,
}

/// Exact replay result for one item-order candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct OrderEvaluation3 {
    /// Item ids in evaluated order.
    pub order: Vec<ItemId>,
    /// Placements proposed by deterministic corner first-fit in that order.
    pub placements: Vec<Placement3>,
    /// Exact replay of the proposal.
    pub replay: PackingVerification3,
}

/// Report from deterministic order local search.
#[derive(Clone, Debug, PartialEq)]
pub struct LocalSearchReport3 {
    /// Initial order evaluation.
    pub initial: OrderEvaluation3,
    /// Best certified evaluation found.
    pub best: OrderEvaluation3,
    /// Accepted improving moves.
    pub accepted_moves: Vec<OrderMove3>,
    /// Number of search passes performed.
    pub steps: usize,
    /// Candidate neighbor moves evaluated.
    pub evaluated_moves: usize,
    /// Completion status.
    pub status: LocalSearchStatus3,
}

/// Limits for deterministic tabu search over 3D item order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TabuSearchConfig3 {
    /// Maximum accepted-neighbor steps.
    pub max_steps: usize,
    /// Maximum neighbor moves inspected per step.
    pub max_neighbors_per_step: usize,
    /// Number of accepted moves retained in tabu memory.
    pub tabu_tenure: usize,
}

/// Tabu-search completion status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TabuSearchStatus3 {
    /// No admissible neighbor was available before another limit interrupted search.
    LocalOptimum,
    /// Search stopped after reaching `max_steps`.
    StepLimit,
    /// Search stopped after reaching `max_neighbors_per_step` in a step.
    NeighborLimit,
}

/// Report from deterministic replay-gated tabu search.
#[derive(Clone, Debug, PartialEq)]
pub struct TabuSearchReport3 {
    /// Initial order evaluation.
    pub initial: OrderEvaluation3,
    /// Best certified evaluation found.
    pub best: OrderEvaluation3,
    /// Accepted moves, including non-improving admissible moves.
    pub accepted_moves: Vec<OrderMove3>,
    /// Final tabu memory after the search stops.
    pub tabu_memory: Vec<OrderMove3>,
    /// Candidate neighbor moves evaluated.
    pub evaluated_moves: usize,
    /// Evaluated tabu candidates rejected because aspiration did not apply.
    pub tabu_rejections: usize,
    /// Accepted-neighbor steps performed.
    pub steps: usize,
    /// Completion status.
    pub status: TabuSearchStatus3,
}

/// Limits for deterministic seeded multistart over 3D item order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MultistartConfig3 {
    /// First deterministic seed used to generate candidate orders.
    pub seed: u64,
    /// Number of seeded starts to evaluate.
    pub starts: usize,
}

/// Seeded multistart completion status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MultistartStatus3 {
    /// All requested starts were evaluated.
    Complete,
    /// No starts were requested, so no candidate order was evaluated.
    BudgetExhausted,
}

/// One seeded order evaluation.
#[derive(Clone, Debug, PartialEq)]
pub struct SeededOrderEvaluation3 {
    /// Seed used to produce this item order.
    pub seed: u64,
    /// Exact replayed evaluation for the seed order.
    pub evaluation: OrderEvaluation3,
}

/// Report from deterministic seeded multistart order search.
#[derive(Clone, Debug, PartialEq)]
pub struct MultistartReport3 {
    /// Completion status.
    pub status: MultistartStatus3,
    /// Evaluated seed-order candidates.
    pub evaluations: Vec<SeededOrderEvaluation3>,
    /// Best certified evaluation found, if at least one start was evaluated.
    pub best: Option<SeededOrderEvaluation3>,
}

/// One deterministic reinsert-unplaced repair move.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReinsertMove3 {
    /// Item id selected from the current exact replay's unplaced set.
    pub item: ItemId,
    /// Candidate insertion index in the current item order after removing the item.
    pub insert_at: usize,
}

/// Limits for deterministic reinsert-unplaced repair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReinsertUnplacedConfig3 {
    /// Maximum accepted-improvement passes.
    pub max_passes: usize,
    /// Maximum candidate reinsertions inspected per pass.
    pub max_trials_per_pass: usize,
}

/// Reinsert-unplaced completion status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReinsertUnplacedStatus3 {
    /// The current replay places every item, so no repair remains.
    Complete,
    /// No improving reinsertion was found before another limit interrupted the pass.
    LocalOptimum,
    /// Repair stopped after reaching `max_passes`.
    PassLimit,
    /// Repair stopped after reaching `max_trials_per_pass` in a pass.
    TrialLimit,
}

/// Report from deterministic reinsert-unplaced repair.
#[derive(Clone, Debug, PartialEq)]
pub struct ReinsertUnplacedReport3 {
    /// Initial order evaluation.
    pub initial: OrderEvaluation3,
    /// Best exact-replayed evaluation found.
    pub best: OrderEvaluation3,
    /// Accepted improving reinsertion moves.
    pub accepted_moves: Vec<ReinsertMove3>,
    /// Candidate reinsertions evaluated.
    pub evaluated_reinsertions: usize,
    /// Improvement passes performed.
    pub passes: usize,
    /// Completion status.
    pub status: ReinsertUnplacedStatus3,
}

/// One accepted deterministic bin-emptying move.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BinEmptyingMove3 {
    /// Bin whose assignments were removed from the candidate layout.
    pub emptied_bin: BinId,
    /// Item ids moved out of `emptied_bin`.
    pub moved_items: Vec<ItemId>,
}

/// Limits for deterministic multi-bin bin-emptying repair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BinEmptyingConfig3 {
    /// Maximum accepted-improvement passes.
    pub max_passes: usize,
    /// Maximum source bins inspected per pass.
    pub max_bins_per_pass: usize,
}

/// Bin-emptying completion status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BinEmptyingStatus3 {
    /// The current assignment already uses zero or one bin.
    Complete,
    /// No improving bin-emptying move was found before another limit fired.
    LocalOptimum,
    /// Search stopped after reaching `max_passes`.
    PassLimit,
    /// Search stopped after reaching `max_bins_per_pass` in a pass.
    BinLimit,
}

/// Exact replay result for one multi-bin assignment candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiBinEvaluation3 {
    /// Candidate multi-bin placements.
    pub placements: Vec<MultiBinPlacement3>,
    /// Exact replay of the candidate assignment.
    pub replay: MultiBinVerification3,
}

/// Report from deterministic multi-bin bin-emptying repair.
#[derive(Clone, Debug, PartialEq)]
pub struct BinEmptyingReport3 {
    /// Initial exact multi-bin replay.
    pub initial: MultiBinEvaluation3,
    /// Best exact-replayed assignment found.
    pub best: MultiBinEvaluation3,
    /// Accepted bin-emptying moves.
    pub accepted_moves: Vec<BinEmptyingMove3>,
    /// Source-bin emptying candidates evaluated.
    pub evaluated_bins: usize,
    /// Improvement passes performed.
    pub passes: usize,
    /// Completion status.
    pub status: BinEmptyingStatus3,
}

/// Runs deterministic local search over item order for fixed-orientation cuboids.
///
/// Each order is packed by a simple exact-candidate corner first-fit proposal.
/// Objective comparison first minimizes unplaced item count, then maximizes
/// exact used volume, then prefers fewer duplicate placements. Moves are
/// generated in swap, insert, and reverse order so the search is deterministic.
pub fn local_search_order_3d(
    bin: &Bin3,
    items: &[Item3],
    config: LocalSearchConfig3,
) -> PackResult<LocalSearchReport3> {
    let initial = evaluate_order(bin, items, items)?;
    let mut best = initial.clone();
    let mut current_items = items.to_vec();
    let mut accepted_moves = Vec::new();
    let mut evaluated_moves = 0_usize;
    let mut status = LocalSearchStatus3::LocalOptimum;

    for step in 0..config.max_steps {
        let mut accepted = None::<(OrderMove3, Vec<Item3>, OrderEvaluation3)>;
        for order_move in order_moves(current_items.len()) {
            if evaluated_moves >= config.max_neighbors_per_step.saturating_mul(step + 1) {
                status = LocalSearchStatus3::NeighborLimit;
                break;
            }
            let neighbor_items = apply_order_move(&current_items, &order_move);
            let evaluation = evaluate_order(bin, items, &neighbor_items)?;
            evaluated_moves += 1;
            if evaluation_better(&evaluation, &best) {
                accepted = Some((order_move, neighbor_items, evaluation));
                break;
            }
        }
        match accepted {
            Some((order_move, neighbor_items, evaluation)) => {
                accepted_moves.push(order_move);
                current_items = neighbor_items;
                best = evaluation;
            }
            None => {
                return Ok(LocalSearchReport3 {
                    initial,
                    best,
                    accepted_moves,
                    steps: step,
                    evaluated_moves,
                    status,
                });
            }
        }
    }
    if config.max_steps > 0 {
        status = LocalSearchStatus3::StepLimit;
    }
    Ok(LocalSearchReport3 {
        initial,
        best,
        accepted_moves,
        steps: config.max_steps,
        evaluated_moves,
        status,
    })
}

/// Runs deterministic tabu search over fixed-orientation cuboid order.
///
/// Each neighbor order is converted into exact corner first-fit placements and
/// replayed before ranking. The tabu memory stores accepted order moves for a
/// bounded tenure. A tabu move is admissible only by aspiration: its exact
/// replayed objective must improve the best certified candidate. This follows
/// Yap's requirement that approximate or heuristic stages produce candidates
/// rather than truth, and Glover, "Tabu Search - Part I," *ORSA Journal on
/// Computing* 1(3), 1989, for the short-term-memory search mechanism.
pub fn tabu_search_order_3d(
    bin: &Bin3,
    items: &[Item3],
    config: TabuSearchConfig3,
) -> PackResult<TabuSearchReport3> {
    let initial = evaluate_order(bin, items, items)?;
    let mut best = initial.clone();
    let mut current_items = items.to_vec();
    let mut accepted_moves = Vec::new();
    let mut tabu_memory = Vec::<OrderMove3>::new();
    let mut evaluated_moves = 0_usize;
    let mut tabu_rejections = 0_usize;

    for step in 0..config.max_steps {
        let mut selected = None::<(OrderMove3, Vec<Item3>, OrderEvaluation3)>;
        for (inspected, order_move) in order_moves(current_items.len()).into_iter().enumerate() {
            if inspected >= config.max_neighbors_per_step {
                return Ok(TabuSearchReport3 {
                    initial,
                    best,
                    accepted_moves,
                    tabu_memory,
                    evaluated_moves,
                    tabu_rejections,
                    steps: step,
                    status: TabuSearchStatus3::NeighborLimit,
                });
            }
            let neighbor_items = apply_order_move(&current_items, &order_move);
            let evaluation = evaluate_order(bin, items, &neighbor_items)?;
            evaluated_moves += 1;
            let improves_best = evaluation_better(&evaluation, &best);
            if is_tabu(&tabu_memory, &order_move) && !improves_best {
                tabu_rejections += 1;
                continue;
            }
            if selected.as_ref().is_none_or(|(_, _, selected_evaluation)| {
                evaluation_better(&evaluation, selected_evaluation)
            }) {
                selected = Some((order_move, neighbor_items, evaluation));
            }
        }

        let Some((order_move, neighbor_items, evaluation)) = selected else {
            return Ok(TabuSearchReport3 {
                initial,
                best,
                accepted_moves,
                tabu_memory,
                evaluated_moves,
                tabu_rejections,
                steps: step,
                status: TabuSearchStatus3::LocalOptimum,
            });
        };

        remember_tabu(&mut tabu_memory, order_move.clone(), config.tabu_tenure);
        accepted_moves.push(order_move);
        current_items = neighbor_items;
        if evaluation_better(&evaluation, &best) {
            best = evaluation;
        }
    }

    Ok(TabuSearchReport3 {
        initial,
        best,
        accepted_moves,
        tabu_memory,
        evaluated_moves,
        tabu_rejections,
        steps: config.max_steps,
        status: TabuSearchStatus3::StepLimit,
    })
}

/// Runs deterministic seeded multistart over fixed-orientation cuboid order.
///
/// Each start shuffles the input item order with a deterministic xorshift
/// generator derived from `config.seed + start_index`, proposes placements with
/// the same exact corner first-fit evaluator as [`local_search_order_3d`], and
/// ranks candidates by exact replayed objective values. This follows Yap's
/// exact-geometric-computation boundary: randomized or seeded proposal
/// generation is allowed, but accepted results remain replay reports rather
/// than unchecked coordinates.
pub fn multistart_order_3d(
    bin: &Bin3,
    items: &[Item3],
    config: MultistartConfig3,
) -> PackResult<MultistartReport3> {
    if config.starts == 0 {
        return Ok(MultistartReport3 {
            status: MultistartStatus3::BudgetExhausted,
            evaluations: Vec::new(),
            best: None,
        });
    }

    let mut evaluations = Vec::with_capacity(config.starts);
    let mut best = None::<SeededOrderEvaluation3>;
    for start in 0..config.starts {
        let seed = config.seed.wrapping_add(start as u64);
        let ordered_items = shuffled_order(items, seed);
        let evaluation = evaluate_order(bin, items, &ordered_items)?;
        let seeded = SeededOrderEvaluation3 { seed, evaluation };
        if best
            .as_ref()
            .is_none_or(|current| evaluation_better(&seeded.evaluation, &current.evaluation))
        {
            best = Some(seeded.clone());
        }
        evaluations.push(seeded);
    }

    Ok(MultistartReport3 {
        status: MultistartStatus3::Complete,
        evaluations,
        best,
    })
}

/// Tries to repair a one-bin 3D order by reinserting currently unplaced items.
///
/// This is a deterministic proposal stage for the common packing repair move
/// "take an unplaced item and try it earlier/later in the construction order."
/// Like the other search helpers, it follows Yap's exact-geometric-computation
/// boundary: a reinsertion is accepted only after rebuilding placements and
/// comparing exact replay reports. The move type is one of the repair
/// neighborhoods discussed for metaheuristic bin packing; see Glover,
/// "Tabu Search - Part I," *ORSA Journal on Computing* 1(3), 1989, for the
/// neighborhood-search framing.
pub fn reinsert_unplaced_order_3d(
    bin: &Bin3,
    items: &[Item3],
    config: ReinsertUnplacedConfig3,
) -> PackResult<ReinsertUnplacedReport3> {
    let initial = evaluate_order(bin, items, items)?;
    let mut best = initial.clone();
    let mut current = initial.clone();
    let mut current_items = items.to_vec();
    let mut accepted_moves = Vec::new();
    let mut evaluated_reinsertions = 0_usize;

    if current.replay.unplaced.is_empty() {
        return Ok(ReinsertUnplacedReport3 {
            initial,
            best,
            accepted_moves,
            evaluated_reinsertions,
            passes: 0,
            status: ReinsertUnplacedStatus3::Complete,
        });
    }

    for pass in 0..config.max_passes {
        let mut accepted = None::<(ReinsertMove3, Vec<Item3>, OrderEvaluation3)>;
        let mut pass_trials = 0_usize;
        let unplaced = current.replay.unplaced.clone();

        'moves: for item_id in unplaced {
            let Some(item) = items.iter().find(|item| item.id == item_id).cloned() else {
                continue;
            };
            let without_item = current_items
                .iter()
                .filter(|candidate| candidate.id != item_id)
                .cloned()
                .collect::<Vec<_>>();
            for insert_at in 0..=without_item.len() {
                if pass_trials >= config.max_trials_per_pass {
                    return Ok(ReinsertUnplacedReport3 {
                        initial,
                        best,
                        accepted_moves,
                        evaluated_reinsertions,
                        passes: pass,
                        status: ReinsertUnplacedStatus3::TrialLimit,
                    });
                }
                let mut repaired = without_item.clone();
                repaired.insert(insert_at, item.clone());
                let evaluation = evaluate_order(bin, items, &repaired)?;
                pass_trials += 1;
                evaluated_reinsertions += 1;
                if evaluation_better(&evaluation, &best) {
                    accepted = Some((
                        ReinsertMove3 {
                            item: item_id.clone(),
                            insert_at,
                        },
                        repaired,
                        evaluation,
                    ));
                    break 'moves;
                }
            }
        }

        match accepted {
            Some((repair_move, repaired, evaluation)) => {
                accepted_moves.push(repair_move);
                current_items = repaired;
                current = evaluation.clone();
                best = evaluation;
                if best.replay.unplaced.is_empty() {
                    return Ok(ReinsertUnplacedReport3 {
                        initial,
                        best,
                        accepted_moves,
                        evaluated_reinsertions,
                        passes: pass + 1,
                        status: ReinsertUnplacedStatus3::Complete,
                    });
                }
            }
            None => {
                return Ok(ReinsertUnplacedReport3 {
                    initial,
                    best,
                    accepted_moves,
                    evaluated_reinsertions,
                    passes: pass,
                    status: ReinsertUnplacedStatus3::LocalOptimum,
                });
            }
        }
    }

    Ok(ReinsertUnplacedReport3 {
        initial,
        best,
        accepted_moves,
        evaluated_reinsertions,
        passes: config.max_passes,
        status: ReinsertUnplacedStatus3::PassLimit,
    })
}

/// Tries to reduce a multi-bin assignment by emptying used bins.
///
/// A bin-emptying move removes every placement assigned to one source bin and
/// tries to reinsert those items into the remaining bins using the same exact
/// face-induced corner candidates as the one-bin order search. This is a repair
/// proposal, not a proof. Following Yap's exact-geometric-computation boundary,
/// and the bin-reduction neighborhoods used in bin-packing local search (see
/// Glover, "Tabu Search - Part I," *ORSA Journal on Computing* 1(3), 1989),
/// a candidate is accepted only after exact [`verify_multi_bin_packing_3d`]
/// replay improves bin count, cost, assignment accounting, and used volume.
pub fn empty_bins_3d(
    bins: &[BinInstance3],
    items: &[Item3],
    placements: &[MultiBinPlacement3],
    config: BinEmptyingConfig3,
) -> PackResult<BinEmptyingReport3> {
    let initial = evaluate_multi_bin(bins, items, placements)?;
    let mut best = initial.clone();
    let mut current = initial.clone();
    let mut accepted_moves = Vec::new();
    let mut evaluated_bins = 0_usize;

    if current.replay.objective.used_bins <= 1 {
        return Ok(BinEmptyingReport3 {
            initial,
            best,
            accepted_moves,
            evaluated_bins,
            passes: 0,
            status: BinEmptyingStatus3::Complete,
        });
    }

    for pass in 0..config.max_passes {
        let mut accepted = None::<(BinEmptyingMove3, MultiBinEvaluation3)>;
        for (inspected_this_pass, source_bin) in
            used_bin_order(&current.placements).into_iter().enumerate()
        {
            if inspected_this_pass >= config.max_bins_per_pass {
                return Ok(BinEmptyingReport3 {
                    initial,
                    best,
                    accepted_moves,
                    evaluated_bins,
                    passes: pass,
                    status: BinEmptyingStatus3::BinLimit,
                });
            }
            evaluated_bins += 1;
            let Some((candidate, moved_items)) =
                try_empty_bin(bins, items, &current.placements, &source_bin)
            else {
                continue;
            };
            let evaluation = evaluate_multi_bin(bins, items, &candidate)?;
            if multi_evaluation_better(&evaluation, &best) {
                accepted = Some((
                    BinEmptyingMove3 {
                        emptied_bin: source_bin,
                        moved_items,
                    },
                    evaluation,
                ));
                break;
            }
        }

        match accepted {
            Some((repair_move, evaluation)) => {
                accepted_moves.push(repair_move);
                current = evaluation.clone();
                best = evaluation;
                if best.replay.objective.used_bins <= 1 {
                    return Ok(BinEmptyingReport3 {
                        initial,
                        best,
                        accepted_moves,
                        evaluated_bins,
                        passes: pass + 1,
                        status: BinEmptyingStatus3::Complete,
                    });
                }
            }
            None => {
                return Ok(BinEmptyingReport3 {
                    initial,
                    best,
                    accepted_moves,
                    evaluated_bins,
                    passes: pass,
                    status: BinEmptyingStatus3::LocalOptimum,
                });
            }
        }
    }

    Ok(BinEmptyingReport3 {
        initial,
        best,
        accepted_moves,
        evaluated_bins,
        passes: config.max_passes,
        status: BinEmptyingStatus3::PassLimit,
    })
}

fn evaluate_order(
    bin: &Bin3,
    all_items: &[Item3],
    ordered_items: &[Item3],
) -> PackResult<OrderEvaluation3> {
    let mut placements = Vec::<Placement3>::new();
    for item in ordered_items {
        let Some(point) = candidate_points(&placements, all_items)
            .into_iter()
            .find(|point| candidate_fits(bin, item, &placements, all_items, point))
        else {
            continue;
        };
        placements.push(Placement3 {
            item: item.id.clone(),
            x: point.x,
            y: point.y,
            z: point.z,
        });
    }
    let replay = verify_packing_3d(bin, all_items, &placements)?;
    Ok(OrderEvaluation3 {
        order: ordered_items.iter().map(|item| item.id.clone()).collect(),
        placements,
        replay,
    })
}

fn evaluate_multi_bin(
    bins: &[BinInstance3],
    items: &[Item3],
    placements: &[MultiBinPlacement3],
) -> PackResult<MultiBinEvaluation3> {
    let replay = verify_multi_bin_packing_3d(bins, items, placements)?;
    Ok(MultiBinEvaluation3 {
        placements: placements.to_vec(),
        replay,
    })
}

fn used_bin_order(placements: &[MultiBinPlacement3]) -> Vec<BinId> {
    let mut bins = Vec::<BinId>::new();
    for placement in placements {
        if !bins.iter().any(|bin| bin == &placement.bin) {
            bins.push(placement.bin.clone());
        }
    }
    bins
}

fn try_empty_bin(
    bins: &[BinInstance3],
    items: &[Item3],
    placements: &[MultiBinPlacement3],
    source_bin: &BinId,
) -> Option<(Vec<MultiBinPlacement3>, Vec<ItemId>)> {
    let mut candidate = placements
        .iter()
        .filter(|placement| &placement.bin != source_bin)
        .cloned()
        .collect::<Vec<_>>();
    let moved = placements
        .iter()
        .filter(|placement| &placement.bin == source_bin)
        .cloned()
        .collect::<Vec<_>>();
    let moved_items = moved
        .iter()
        .map(|placement| placement.item.clone())
        .collect::<Vec<_>>();

    for placement in moved {
        let item = items.iter().find(|item| item.id == placement.item)?;
        let mut placed = false;
        for bin in bins.iter().filter(|bin| &bin.id != source_bin) {
            let target_placements = candidate
                .iter()
                .filter(|candidate| candidate.bin == bin.id)
                .map(|candidate| Placement3 {
                    item: candidate.item.clone(),
                    x: candidate.x.clone(),
                    y: candidate.y.clone(),
                    z: candidate.z.clone(),
                })
                .collect::<Vec<_>>();
            if let Some(point) = candidate_points(&target_placements, items)
                .into_iter()
                .find(|point| candidate_fits(&bin.bin, item, &target_placements, items, point))
            {
                candidate.push(MultiBinPlacement3 {
                    bin: bin.id.clone(),
                    item: item.id.clone(),
                    x: point.x,
                    y: point.y,
                    z: point.z,
                });
                placed = true;
                break;
            }
        }
        if !placed {
            return None;
        }
    }

    Some((candidate, moved_items))
}

fn order_moves(len: usize) -> Vec<OrderMove3> {
    let mut moves = Vec::new();
    for left in 0..len {
        for right in (left + 1)..len {
            moves.push(OrderMove3::Swap { left, right });
        }
    }
    for from in 0..len {
        for to in 0..len {
            if from != to {
                moves.push(OrderMove3::Insert { from, to });
            }
        }
    }
    for start in 0..len {
        for end in (start + 1)..len {
            moves.push(OrderMove3::Reverse { start, end });
        }
    }
    moves
}

fn apply_order_move(items: &[Item3], order_move: &OrderMove3) -> Vec<Item3> {
    let mut moved = items.to_vec();
    match *order_move {
        OrderMove3::Swap { left, right } => moved.swap(left, right),
        OrderMove3::Insert { from, to } => {
            let item = moved.remove(from);
            let adjusted = if from < to { to.saturating_sub(1) } else { to };
            moved.insert(adjusted, item);
        }
        OrderMove3::Reverse { start, end } => moved[start..=end].reverse(),
    }
    moved
}

fn is_tabu(tabu_memory: &[OrderMove3], order_move: &OrderMove3) -> bool {
    tabu_memory.iter().any(|tabu| tabu == order_move)
}

fn remember_tabu(tabu_memory: &mut Vec<OrderMove3>, order_move: OrderMove3, tabu_tenure: usize) {
    if tabu_tenure == 0 {
        tabu_memory.clear();
        return;
    }
    tabu_memory.push(order_move);
    if tabu_memory.len() > tabu_tenure {
        tabu_memory.remove(0);
    }
}

fn shuffled_order(items: &[Item3], seed: u64) -> Vec<Item3> {
    let mut ordered = items.to_vec();
    let mut state = if seed == 0 {
        0x9e37_79b9_7f4a_7c15
    } else {
        seed
    };
    for index in (1..ordered.len()).rev() {
        let next = next_seed(&mut state);
        let swap_with = (next as usize) % (index + 1);
        ordered.swap(index, swap_with);
    }
    ordered
}

fn next_seed(state: &mut u64) -> u64 {
    let mut value = *state;
    value ^= value << 13;
    value ^= value >> 7;
    value ^= value << 17;
    *state = value;
    value
}

fn evaluation_better(candidate: &OrderEvaluation3, current: &OrderEvaluation3) -> bool {
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

fn multi_evaluation_better(candidate: &MultiBinEvaluation3, current: &MultiBinEvaluation3) -> bool {
    if candidate.replay.objective.used_bins != current.replay.objective.used_bins {
        return candidate.replay.objective.used_bins < current.replay.objective.used_bins;
    }
    if lt(
        &candidate.replay.objective.total_cost,
        &current.replay.objective.total_cost,
    )
    .unwrap_or(false)
    {
        return true;
    }
    if !exact_eq(
        &candidate.replay.objective.total_cost,
        &current.replay.objective.total_cost,
    ) {
        return false;
    }
    if candidate.replay.objective.unplaced_items != current.replay.objective.unplaced_items {
        return candidate.replay.objective.unplaced_items < current.replay.objective.unplaced_items;
    }
    if candidate.replay.objective.duplicate_assignments
        != current.replay.objective.duplicate_assignments
    {
        return candidate.replay.objective.duplicate_assignments
            < current.replay.objective.duplicate_assignments;
    }
    gt(
        &candidate.replay.objective.used_volume,
        &current.replay.objective.used_volume,
    )
    .unwrap_or(false)
}

#[derive(Clone, Debug, PartialEq)]
struct Point3 {
    x: Real,
    y: Real,
    z: Real,
}

fn candidate_points(placements: &[Placement3], items: &[Item3]) -> Vec<Point3> {
    let mut points = vec![Point3 {
        x: Real::zero(),
        y: Real::zero(),
        z: Real::zero(),
    }];
    for placement in placements {
        let Some(item) = items.iter().find(|item| item.id == placement.item) else {
            continue;
        };
        push_unique(
            &mut points,
            Point3 {
                x: placement.x.clone() + item.size.x.clone(),
                y: placement.y.clone(),
                z: placement.z.clone(),
            },
        );
        push_unique(
            &mut points,
            Point3 {
                x: placement.x.clone(),
                y: placement.y.clone() + item.size.y.clone(),
                z: placement.z.clone(),
            },
        );
        push_unique(
            &mut points,
            Point3 {
                x: placement.x.clone(),
                y: placement.y.clone(),
                z: placement.z.clone() + item.size.z.clone(),
            },
        );
    }
    points
}

fn push_unique(points: &mut Vec<Point3>, point: Point3) {
    if !points
        .iter()
        .any(|candidate| points_equal(candidate, &point))
    {
        points.push(point);
    }
}

fn candidate_fits(
    bin: &Bin3,
    item: &Item3,
    placements: &[Placement3],
    items: &[Item3],
    point: &Point3,
) -> bool {
    if !nonnegative(&point.x).unwrap_or(false)
        || !nonnegative(&point.y).unwrap_or(false)
        || !nonnegative(&point.z).unwrap_or(false)
        || !leq(&(point.x.clone() + item.size.x.clone()), &bin.size.x).unwrap_or(false)
        || !leq(&(point.y.clone() + item.size.y.clone()), &bin.size.y).unwrap_or(false)
        || !leq(&(point.z.clone() + item.size.z.clone()), &bin.size.z).unwrap_or(false)
    {
        return false;
    }
    placements.iter().all(|placement| {
        let Some(placed_item) = items.iter().find(|placed| placed.id == placement.item) else {
            return false;
        };
        boxes_disjoint(item, point, placed_item, placement).unwrap_or(false)
    })
}

fn boxes_disjoint(
    item: &Item3,
    point: &Point3,
    placed_item: &Item3,
    placement: &Placement3,
) -> Option<bool> {
    Some(
        leq(&(point.x.clone() + item.size.x.clone()), &placement.x)?
            || leq(
                &(placement.x.clone() + placed_item.size.x.clone()),
                &point.x,
            )?
            || leq(&(point.y.clone() + item.size.y.clone()), &placement.y)?
            || leq(
                &(placement.y.clone() + placed_item.size.y.clone()),
                &point.y,
            )?
            || leq(&(point.z.clone() + item.size.z.clone()), &placement.z)?
            || leq(
                &(placement.z.clone() + placed_item.size.z.clone()),
                &point.z,
            )?,
    )
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    exact_eq(&left.x, &right.x) && exact_eq(&left.y, &right.y) && exact_eq(&left.z, &right.z)
}

fn gt(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Positive => Some(true),
        RealSign::Negative | RealSign::Zero => Some(false),
    }
}

fn lt(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative => Some(true),
        RealSign::Positive | RealSign::Zero => Some(false),
    }
}

fn exact_eq(left: &Real, right: &Real) -> bool {
    matches!((left - right).refine_sign_until(-64), Some(RealSign::Zero))
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
