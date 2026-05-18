//! Prepared placement batches for deterministic exact replay.
//!
//! Prepared layouts are a normalization layer, not a new feasibility checker.
//! They canonicalize placement record order and preserve evidence about any
//! uncertified ordering comparisons. Acceptance still comes from exact replay.
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>):
//! preprocessing may organize combinatorial data, but geometric decisions stay
//! certified or explicitly reported as unknown.

use std::cmp::Ordering;

use hyperreal::{Real, RealExactSetFacts, RealSign};

use crate::{
    AxisBox3, Bin3, CapacityBoundReport3, FreeBox3, Item3, ItemId, PackResult,
    PackingVerification3, PairIncompatibilityReport3, Placement3, capacity_bounds_3d,
    pair_incompatibilities_3d, verify_packing_3d,
};

/// Exact demand class for items with the same certified dimensions.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedDemandClass3 {
    /// Representative exact dimensions for this class.
    pub size: AxisBox3,
    /// Item ids assigned to this class in deterministic order.
    pub item_ids: Vec<ItemId>,
    /// Number of items in the class.
    pub count: usize,
    /// Exact total class volume.
    pub total_volume: Real,
}

/// Exact dimensional summary retained by a prepared packing problem.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedDimensionFacts3 {
    /// Number of item records summarized.
    pub item_count: usize,
    /// Exact total volume of all items.
    pub total_item_volume: Real,
    /// Largest item x extent when all comparisons certified.
    pub max_item_x: Option<Real>,
    /// Largest item y extent when all comparisons certified.
    pub max_item_y: Option<Real>,
    /// Largest item z extent when all comparisons certified.
    pub max_item_z: Option<Real>,
    /// Exact comparisons used while computing maxima.
    pub exact_comparisons: usize,
    /// Max-dimension comparisons that could not be certified.
    pub unknown_max_comparisons: usize,
}

/// Common-scale and grid facts for a prepared packing problem.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedGridSummary3 {
    /// Coarse exact-rational facts for bin and item dimensions.
    pub scalar_facts: RealExactSetFacts,
    /// Whether all dimension scalars are exact rational integers.
    pub integer_grid: bool,
    /// Whether all dimension scalars can use a dyadic exact schedule.
    pub dyadic_schedule: bool,
    /// Whether all dimension scalars share one exact rational denominator.
    pub shared_denominator_schedule: bool,
    /// Human-readable grid facts.
    pub facts: Vec<String>,
}

/// Cache-payoff metadata for prepared problem reuse.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedCacheMetadata3 {
    /// Number of scalar dimension values scanned for grid facts.
    pub scalar_values: usize,
    /// Number of item records avoided by demand-class collapsing.
    pub demand_class_reduction: usize,
    /// Number of initial free-space records retained.
    pub initial_free_boxes: usize,
    /// Pair checks a raw replay would perform for one placement per item.
    pub expected_replay_pair_checks: usize,
    /// Whether total/max-dimension lower bounds were cached.
    pub capacity_bound_cached: bool,
    /// Whether pair-incompatibility lower bounds were cached.
    pub pair_bound_cached: bool,
}

/// Prepared 3D packing problem.
///
/// This is the problem-side counterpart to [`PreparedPlacements3`]. It stores
/// exact demand classes, common-scale facts, lower-bound reports, and initial
/// free-space cache state so proposal engines do not repeatedly rediscover the
/// same structure. The cached data remains advisory: as in Yap's exact
/// geometric computation model, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>), every accepted layout must
/// still pass exact replay instead of trusting preprocessing.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedPacking3 {
    /// Exact bin dimensions.
    pub bin: Bin3,
    /// Demand classes sorted by their first item id.
    pub demand_classes: Vec<PreparedDemandClass3>,
    /// Exact dimensional facts.
    pub dimensions: PreparedDimensionFacts3,
    /// Common-scale/grid facts over all bin and item dimensions.
    pub grid: PreparedGridSummary3,
    /// Initial exact free-space cache. This starts with the whole bin.
    pub initial_free_boxes: Vec<FreeBox3>,
    /// Cached necessary total-volume and max-dimension lower bounds.
    pub capacity_bound: CapacityBoundReport3,
    /// Cached necessary pair-incompatibility lower bounds.
    pub pair_bound: PairIncompatibilityReport3,
    /// Cache payoff and replay-cost metadata.
    pub cache: PreparedCacheMetadata3,
    /// Human-readable preparation facts.
    pub facts: Vec<String>,
}

/// Prepared 3D placement batch.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedPlacements3 {
    /// Placements sorted into deterministic replay order when comparisons certify.
    pub placements: Vec<Placement3>,
    /// Number of input placement records.
    pub input_placements: usize,
    /// Exact coordinate comparisons performed while preparing.
    pub exact_comparisons: usize,
    /// Number of ordering comparisons that could not be certified.
    pub unknown_orderings: usize,
    /// Human-readable preparation facts.
    pub facts: Vec<String>,
}

/// Prepares a 3D packing problem for exact-aware proposal engines.
///
/// The preparation pass collapses equal-dimension demand classes, scans
/// dimension scalars through [`Real::exact_set_facts`] for common-scale
/// scheduling evidence, caches necessary lower bounds, and records the initial
/// full-bin free-space frontier. It deliberately does not certify feasibility;
/// proposed placements still return through [`verify_packing_3d`].
pub fn prepare_packing_3d(bin: &Bin3, items: &[Item3]) -> PreparedPacking3 {
    let mut demand_classes = demand_classes_3d(items);
    demand_classes.sort_by(|left, right| left.item_ids[0].cmp(&right.item_ids[0]));

    let mut scalars = vec![&bin.size.x, &bin.size.y, &bin.size.z];
    for item in items {
        scalars.extend([&item.size.x, &item.size.y, &item.size.z]);
    }
    let scalar_facts = Real::exact_set_facts(scalars.iter().copied());
    let grid = grid_summary_3d(scalar_facts);
    let dimensions = dimension_facts_3d(items);
    let capacity_bound = capacity_bounds_3d(bin, items);
    let pair_bound = pair_incompatibilities_3d(bin, items);
    let initial_free_boxes = vec![FreeBox3 {
        x: Real::zero(),
        y: Real::zero(),
        z: Real::zero(),
        size: bin.size.clone(),
    }];
    let expected_replay_pair_checks = items.len().saturating_mul(items.len().saturating_sub(1)) / 2;
    let mut facts = Vec::new();
    if !items.is_empty() && demand_classes.len() < items.len() {
        facts.push(format!(
            "{} item records collapsed into {} demand classes",
            items.len(),
            demand_classes.len()
        ));
    }
    if dimensions.unknown_max_comparisons > 0 {
        facts.push(format!(
            "{} max-dimension comparisons were unknown",
            dimensions.unknown_max_comparisons
        ));
    }
    if !scalar_facts.all_exact_rational {
        facts.push("at least one dimension scalar is not an exact rational".into());
    }

    PreparedPacking3 {
        bin: bin.clone(),
        cache: PreparedCacheMetadata3 {
            scalar_values: scalar_facts.len,
            demand_class_reduction: items.len().saturating_sub(demand_classes.len()),
            initial_free_boxes: initial_free_boxes.len(),
            expected_replay_pair_checks,
            capacity_bound_cached: true,
            pair_bound_cached: true,
        },
        demand_classes,
        dimensions,
        grid,
        initial_free_boxes,
        capacity_bound,
        pair_bound,
        facts,
    }
}

/// Prepares placement records for deterministic replay.
///
/// Sorting is by item id, then exact `z`, `y`, and `x` coordinates. If a
/// coordinate ordering cannot be certified, the original relative order is
/// retained for that comparison and the report records an unknown ordering.
pub fn prepare_placements_3d(placements: &[Placement3]) -> PreparedPlacements3 {
    let mut indexed = placements
        .iter()
        .cloned()
        .enumerate()
        .collect::<Vec<(usize, Placement3)>>();
    let mut exact_comparisons = 0_usize;
    let mut unknown_orderings = 0_usize;
    indexed.sort_by(|(left_index, left), (right_index, right)| {
        left.item
            .cmp(&right.item)
            .then_with(|| {
                compare_real(
                    &left.z,
                    &right.z,
                    &mut exact_comparisons,
                    &mut unknown_orderings,
                )
                .unwrap_or_else(|| left_index.cmp(right_index))
            })
            .then_with(|| {
                compare_real(
                    &left.y,
                    &right.y,
                    &mut exact_comparisons,
                    &mut unknown_orderings,
                )
                .unwrap_or_else(|| left_index.cmp(right_index))
            })
            .then_with(|| {
                compare_real(
                    &left.x,
                    &right.x,
                    &mut exact_comparisons,
                    &mut unknown_orderings,
                )
                .unwrap_or_else(|| left_index.cmp(right_index))
            })
            .then_with(|| left_index.cmp(right_index))
    });
    let mut facts = Vec::new();
    if unknown_orderings > 0 {
        facts.push(format!(
            "{unknown_orderings} placement order comparisons were unknown"
        ));
    }
    PreparedPlacements3 {
        placements: indexed
            .into_iter()
            .map(|(_, placement)| placement)
            .collect(),
        input_placements: placements.len(),
        exact_comparisons,
        unknown_orderings,
        facts,
    }
}

/// Replays a prepared 3D placement batch with the exact verifier.
pub fn replay_prepared_packing_3d(
    bin: &Bin3,
    items: &[crate::Item3],
    prepared: &PreparedPlacements3,
) -> PackResult<PackingVerification3> {
    verify_packing_3d(bin, items, &prepared.placements)
}

fn compare_real(
    left: &Real,
    right: &Real,
    exact_comparisons: &mut usize,
    unknown_orderings: &mut usize,
) -> Option<Ordering> {
    *exact_comparisons += 1;
    match (left - right).refine_sign_until(-64) {
        Some(RealSign::Negative) => Some(Ordering::Less),
        Some(RealSign::Zero) => Some(Ordering::Equal),
        Some(RealSign::Positive) => Some(Ordering::Greater),
        None => {
            *unknown_orderings += 1;
            None
        }
    }
}

fn demand_classes_3d(items: &[Item3]) -> Vec<PreparedDemandClass3> {
    let mut classes = Vec::<PreparedDemandClass3>::new();
    for item in items {
        if let Some(class) = classes.iter_mut().find(|class| class.size == item.size) {
            class.item_ids.push(item.id.clone());
            class.item_ids.sort();
            class.count += 1;
            class.total_volume = class.total_volume.clone() + item.size.volume();
            continue;
        }
        classes.push(PreparedDemandClass3 {
            size: item.size.clone(),
            item_ids: vec![item.id.clone()],
            count: 1,
            total_volume: item.size.volume(),
        });
    }
    classes
}

fn dimension_facts_3d(items: &[Item3]) -> PreparedDimensionFacts3 {
    let mut total_item_volume = Real::zero();
    let mut max_item_x = None::<Real>;
    let mut max_item_y = None::<Real>;
    let mut max_item_z = None::<Real>;
    let mut exact_comparisons = 0_usize;
    let mut unknown_max_comparisons = 0_usize;

    for item in items {
        total_item_volume = total_item_volume + item.size.volume();
        update_max(
            &mut max_item_x,
            &item.size.x,
            &mut exact_comparisons,
            &mut unknown_max_comparisons,
        );
        update_max(
            &mut max_item_y,
            &item.size.y,
            &mut exact_comparisons,
            &mut unknown_max_comparisons,
        );
        update_max(
            &mut max_item_z,
            &item.size.z,
            &mut exact_comparisons,
            &mut unknown_max_comparisons,
        );
    }

    PreparedDimensionFacts3 {
        item_count: items.len(),
        total_item_volume,
        max_item_x,
        max_item_y,
        max_item_z,
        exact_comparisons,
        unknown_max_comparisons,
    }
}

fn update_max(
    current: &mut Option<Real>,
    candidate: &Real,
    exact_comparisons: &mut usize,
    unknown_max_comparisons: &mut usize,
) {
    let Some(existing) = current.as_ref() else {
        *current = Some(candidate.clone());
        return;
    };
    match compare_real(
        candidate,
        existing,
        exact_comparisons,
        unknown_max_comparisons,
    ) {
        Some(Ordering::Greater) => *current = Some(candidate.clone()),
        Some(Ordering::Less | Ordering::Equal) => {}
        None => *current = None,
    }
}

fn grid_summary_3d(scalar_facts: RealExactSetFacts) -> PreparedGridSummary3 {
    let integer_grid = scalar_facts.has_integer_grid_schedule();
    let dyadic_schedule = scalar_facts.has_dyadic_schedule();
    let shared_denominator_schedule = scalar_facts.has_shared_denominator_schedule();
    let mut facts = Vec::new();
    if integer_grid {
        facts.push("all dimension scalars are exact rational integers".into());
    } else if dyadic_schedule {
        facts.push("all dimension scalars support a dyadic exact schedule".into());
    } else if shared_denominator_schedule {
        facts.push("all dimension scalars share one exact rational denominator".into());
    } else if scalar_facts.all_exact_rational {
        facts.push("dimension scalars are exact rationals with mixed denominators".into());
    } else {
        facts.push("dimension scalar set includes non-rational exact values".into());
    }

    PreparedGridSummary3 {
        scalar_facts,
        integer_grid,
        dyadic_schedule,
        shared_denominator_schedule,
        facts,
    }
}
