use std::hint::black_box;
use std::time::Instant;

use hyperpack::{
    AxisBox3, Bin3, BinEmptyingConfig3, BinId, BinInstance3, CuboidPortfolioBudget3,
    DomainBoxFact3, DomainCrate, DomainFactStatus, ExactSearchLimit3, FeasibilityReplay3, Item3,
    ItemId, ItemWeight3, LoadLimit3, LocalSearchConfig3, MultiBinPlacement3, MultistartConfig3,
    ObjectiveTerm3, Orientation2, Orientation3, OrientedItem3, OrientedPlacement3,
    OrientedSheetItem2, OrientedSheetPlacement2, Placement3, Real, Rect2, ReinsertUnplacedConfig3,
    SheetBin2, SheetItem2, SheetPlacement2, SheetPortfolioBudget2, StockBin1, StockItem1,
    StockPlacement1, SupportPolicy3, TabuSearchConfig3, analyze_packing_3d,
    auto_cuboid_portfolio_3d, auto_sheet_portfolio_2d, branch_and_bound_one_bin_3d,
    capacity_bounds_2d, capacity_bounds_3d, compare_objectives_3d,
    cuboid_best_fit_decreasing_footprint_area_3d, cuboid_best_fit_decreasing_max_side_3d,
    cuboid_best_fit_decreasing_volume_3d, cuboid_extreme_point_decreasing_volume_3d,
    cuboid_first_fit_decreasing_footprint_area_3d, cuboid_first_fit_decreasing_max_side_3d,
    cuboid_first_fit_decreasing_volume_3d, cuboid_guillotine_best_volume_fit_3d,
    cuboid_laff_largest_area_fit_first_3d, cuboid_maximal_space_decreasing_volume_3d,
    empty_bins_3d, export_no_overlap_model_2d, export_no_overlap_model_3d,
    guillotine_best_area_fit_2d, guillotine_best_long_side_fit_2d,
    guillotine_best_short_side_fit_2d, height_objective_3d, import_domain_items_3d,
    local_search_order_3d, maxrects_best_area_fit_2d, maxrects_best_long_side_fit_2d,
    maxrects_best_short_side_fit_2d, maxrects_bottom_left_2d, maxrects_contact_point_2d,
    multistart_order_3d, order_placements_3d, pair_incompatibilities_2d, pair_incompatibilities_3d,
    reinsert_unplaced_order_3d, shelf_best_fit_decreasing_height_2d,
    shelf_first_fit_decreasing_height_2d, shelf_next_fit_decreasing_height_2d,
    skyline_bottom_left_2d, skyline_minimum_waste_2d, snapshot_packing_3d_binary,
    snapshot_packing_3d_text, snapshot_sheet_2d_binary, snapshot_sheet_2d_text,
    snapshot_stock_1d_binary, snapshot_stock_1d_text, tabu_search_order_3d, verify_clearance_2d,
    verify_clearance_3d, verify_direct_stack_load_3d, verify_multi_bin_packing_3d,
    verify_oriented_packing_2d, verify_oriented_packing_3d, verify_packing_1d, verify_packing_2d,
    verify_packing_3d, verify_support_3d,
};

fn r(value: i32) -> Real {
    value.into()
}

fn main() {
    let bin = Bin3 {
        size: AxisBox3::new(r(100), r(100), r(10)).unwrap(),
    };
    let mut items = Vec::new();
    let mut placements = Vec::new();
    for index in 0..50 {
        let id = ItemId::new(format!("item-{index}")).unwrap();
        items.push(Item3 {
            id: id.clone(),
            size: AxisBox3::new(r(2), r(2), r(1)).unwrap(),
        });
        placements.push(Placement3 {
            item: id,
            x: r((index % 25) * 2),
            y: r((index / 25) * 2),
            z: r(0),
        });
    }
    let oriented_items = (0..50)
        .map(|index| {
            OrientedItem3::new(
                ItemId::new(format!("oriented-3d-{index}")).unwrap(),
                AxisBox3::new(r(2), r(2), r(1)).unwrap(),
                vec![Orientation3::Xyz, Orientation3::Xzy],
                "bench-unit",
            )
        })
        .collect::<Vec<_>>();
    let oriented_placements = oriented_items
        .iter()
        .enumerate()
        .map(|(index, item)| OrientedPlacement3 {
            item: item.id.clone(),
            x: r(((index as i32) % 25) * 2),
            y: r(((index as i32) / 25) * 2),
            z: r(0),
            orientation: Orientation3::Xyz,
        })
        .collect::<Vec<_>>();
    let multi_bins = vec![
        BinInstance3::new(BinId::new("bench-a").unwrap(), bin.clone(), r(10)).unwrap(),
        BinInstance3::new(BinId::new("bench-b").unwrap(), bin.clone(), r(12)).unwrap(),
    ];
    let multi_placements = placements
        .iter()
        .map(|placement| MultiBinPlacement3 {
            bin: BinId::new("bench-a").unwrap(),
            item: placement.item.clone(),
            x: placement.x.clone(),
            y: placement.y.clone(),
            z: placement.z.clone(),
        })
        .collect::<Vec<_>>();
    let load_weights = items
        .iter()
        .map(|item| ItemWeight3::new(item.id.clone(), r(1)).unwrap())
        .collect::<Vec<_>>();
    let load_limits = items
        .iter()
        .map(|item| LoadLimit3::new(item.id.clone(), r(50)).unwrap())
        .collect::<Vec<_>>();
    let stock_bin = StockBin1::new(r(100)).unwrap();
    let stock_items = (0..50)
        .map(|index| StockItem1::new(ItemId::new(format!("stock-{index}")).unwrap(), r(2)).unwrap())
        .collect::<Vec<_>>();
    let stock_placements = stock_items
        .iter()
        .enumerate()
        .map(|(index, item)| StockPlacement1 {
            item: item.id.clone(),
            start: r((index as i32) * 2),
        })
        .collect::<Vec<_>>();
    let sheet_bin = SheetBin2::new(Rect2::new(r(100), r(10)).unwrap());
    let sheet_items = (0..50)
        .map(|index| {
            SheetItem2::new(
                ItemId::new(format!("sheet-{index}")).unwrap(),
                Rect2::new(r(2), r(2)).unwrap(),
            )
        })
        .collect::<Vec<_>>();
    let sheet_placements = sheet_items
        .iter()
        .enumerate()
        .map(|(index, item)| SheetPlacement2 {
            item: item.id.clone(),
            x: r((index as i32) * 2),
            y: r(0),
        })
        .collect::<Vec<_>>();
    let oriented_sheet_items = (0..50)
        .map(|index| {
            OrientedSheetItem2::new(
                ItemId::new(format!("oriented-{index}")).unwrap(),
                Rect2::new(r(2), r(2)).unwrap(),
                vec![Orientation2::Deg0, Orientation2::Deg90],
                "bench-unit",
            )
        })
        .collect::<Vec<_>>();
    let oriented_sheet_placements = oriented_sheet_items
        .iter()
        .enumerate()
        .map(|(index, item)| OrientedSheetPlacement2 {
            item: item.id.clone(),
            x: r((index as i32) * 2),
            y: r(0),
            orientation: Orientation2::Deg0,
        })
        .collect::<Vec<_>>();

    // Custom benchmark binaries also run under `cargo test --all-targets`.
    // One debug pass is enough for that smoke check; release bench runs retain
    // enough work for aggregate timing.
    let iterations = if cfg!(debug_assertions) { 1 } else { 10 };
    let started = Instant::now();
    let mut checks = 0_usize;
    let mut waste_seen = 0_usize;
    for _ in 0..iterations {
        let replay = FeasibilityReplay3::replay(black_box(&bin), &items, &placements).unwrap();
        checks ^= replay.containment_checks;
        checks ^= replay.no_overlap_checks;
        let order = order_placements_3d(&placements);
        let ordered_replay = verify_packing_3d(black_box(&bin), &items, &order.placements).unwrap();
        checks ^= ordered_replay.feasibility.no_overlap_checks;
        let analysis = analyze_packing_3d(black_box(&bin), &items);
        checks ^= analysis.demand_classes.len();
        checks ^= analysis.scalar_value_count();
        let verification = verify_packing_3d(black_box(&bin), &items, &placements).unwrap();
        if verification.objective.unplaced_items == 0 {
            waste_seen ^= verification.objective.placed_items;
        }
        let height = height_objective_3d(black_box(&bin), &items, &placements).unwrap();
        checks ^= height.checked_placements;
        let objective_compare = compare_objectives_3d(
            &verification,
            Some(&height),
            &verification,
            Some(&height),
            &[
                ObjectiveTerm3::MinimizeUnplacedItems,
                ObjectiveTerm3::MaximizeUsedVolume,
                ObjectiveTerm3::MinimizeUsedHeight,
            ],
        );
        checks ^= objective_compare.compared_terms;
        let clearance = verify_clearance_3d(black_box(&items), &placements, r(0)).unwrap();
        checks ^= clearance.pairs.len();
        let multi =
            verify_multi_bin_packing_3d(black_box(&multi_bins), &items, &multi_placements).unwrap();
        checks ^= multi.objective.used_bins;
        let bin_emptying = empty_bins_3d(
            black_box(&multi_bins),
            &items,
            &multi_placements,
            BinEmptyingConfig3 {
                max_passes: 1,
                max_bins_per_pass: 4,
            },
        )
        .unwrap();
        checks ^= bin_emptying.evaluated_bins;
        let support =
            verify_support_3d(black_box(&items), &placements, SupportPolicy3::FullBase).unwrap();
        checks ^= support.evidence.len();
        let support_center = verify_support_3d(
            black_box(&items),
            &placements,
            SupportPolicy3::CenterOfMassProjection,
        )
        .unwrap();
        checks ^= support_center.evidence.len();
        let load = verify_direct_stack_load_3d(
            black_box(&items),
            &placements,
            &load_weights,
            &load_limits,
        )
        .unwrap();
        checks ^= load.evidence.len();
        let exact_search = branch_and_bound_one_bin_3d(
            black_box(&bin),
            &items[..3],
            ExactSearchLimit3 {
                max_items: 3,
                max_nodes: 128,
            },
        )
        .unwrap();
        checks ^= exact_search.nodes;
        let model = export_no_overlap_model_3d(black_box(&bin), &items[..6]);
        checks ^= model.domains.len();
        checks ^= model.disjunctions.len();
        let local_search = local_search_order_3d(
            black_box(&bin),
            &items[..5],
            LocalSearchConfig3 {
                max_steps: 2,
                max_neighbors_per_step: 32,
            },
        )
        .unwrap();
        checks ^= local_search.evaluated_moves;
        let tabu = tabu_search_order_3d(
            black_box(&bin),
            &items[..5],
            TabuSearchConfig3 {
                max_steps: 2,
                max_neighbors_per_step: 32,
                tabu_tenure: 2,
            },
        )
        .unwrap();
        checks ^= tabu.evaluated_moves;
        let multistart = multistart_order_3d(
            black_box(&bin),
            &items[..5],
            MultistartConfig3 {
                seed: 19,
                starts: 4,
            },
        )
        .unwrap();
        checks ^= multistart.evaluations.len();
        let repair = reinsert_unplaced_order_3d(
            black_box(&bin),
            &items[..5],
            ReinsertUnplacedConfig3 {
                max_passes: 2,
                max_trials_per_pass: 32,
            },
        )
        .unwrap();
        checks ^= repair.evaluated_reinsertions;
        let domain_fact = DomainBoxFact3::new(
            DomainCrate::Hyperparts,
            "bench-domain-item",
            AxisBox3::new(r(2), r(2), r(1)).unwrap(),
            "bench",
            DomainFactStatus::Exact,
        )
        .unwrap();
        let domain = import_domain_items_3d(DomainCrate::Hyperparts, &[domain_fact]).unwrap();
        checks ^= domain.items.len();
        let cuboid_first = cuboid_first_fit_decreasing_volume_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_first.trace.emitted_candidates;
        let cuboid_best = cuboid_best_fit_decreasing_volume_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_best.trace.emitted_candidates;
        let cuboid_first_side =
            cuboid_first_fit_decreasing_max_side_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_first_side.trace.emitted_candidates;
        let cuboid_best_side =
            cuboid_best_fit_decreasing_max_side_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_best_side.trace.emitted_candidates;
        let cuboid_first_area =
            cuboid_first_fit_decreasing_footprint_area_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_first_area.trace.emitted_candidates;
        let cuboid_best_area =
            cuboid_best_fit_decreasing_footprint_area_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_best_area.trace.emitted_candidates;
        let cuboid_extreme =
            cuboid_extreme_point_decreasing_volume_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_extreme.trace.emitted_candidates;
        let cuboid_spaces =
            cuboid_maximal_space_decreasing_volume_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_spaces.trace.emitted_candidates;
        checks ^= cuboid_spaces.free_boxes.len();
        let cuboid_guillotine =
            cuboid_guillotine_best_volume_fit_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_guillotine.trace.emitted_candidates;
        checks ^= cuboid_guillotine.free_boxes.len();
        let cuboid_laff = cuboid_laff_largest_area_fit_first_3d(black_box(&bin), &items).unwrap();
        checks ^= cuboid_laff.trace.emitted_candidates;
        let cuboid_portfolio = auto_cuboid_portfolio_3d(
            black_box(&bin),
            &items,
            CuboidPortfolioBudget3 { max_algorithms: 10 },
        )
        .unwrap();
        checks ^= cuboid_portfolio.evaluated.len();
        checks ^= cuboid_portfolio
            .best
            .as_ref()
            .map(|best| best.trace.emitted_candidates)
            .unwrap_or(0);
        let oriented_3d =
            verify_oriented_packing_3d(black_box(&bin), &oriented_items, &oriented_placements)
                .unwrap();
        checks ^= oriented_3d.packing.feasibility.no_overlap_checks;
        let bounds = capacity_bounds_3d(black_box(&bin), &items);
        if !bounds.proves_infeasible() {
            waste_seen ^= bounds.checked_items;
        }
        let pairs = pair_incompatibilities_3d(black_box(&bin), &items);
        if !pairs.proves_infeasible() {
            checks ^= pairs.checked_pairs;
        }
        let stock =
            verify_packing_1d(black_box(&stock_bin), &stock_items, &stock_placements).unwrap();
        checks ^= stock.no_overlap_checks;
        checks ^=
            snapshot_stock_1d_text(black_box(&stock_bin), &stock_items, &stock_placements).len();
        checks ^=
            snapshot_stock_1d_binary(black_box(&stock_bin), &stock_items, &stock_placements).len();
        let sheet =
            verify_packing_2d(black_box(&sheet_bin), &sheet_items, &sheet_placements).unwrap();
        checks ^= sheet.no_overlap_checks;
        let sheet_bounds = capacity_bounds_2d(black_box(&sheet_bin), &sheet_items);
        if !sheet_bounds.proves_infeasible() {
            checks ^= sheet_bounds.checked_items;
        }
        let sheet_pairs = pair_incompatibilities_2d(black_box(&sheet_bin), &sheet_items);
        if !sheet_pairs.proves_infeasible() {
            checks ^= sheet_pairs.checked_pairs;
        }
        let sheet_model = export_no_overlap_model_2d(black_box(&sheet_bin), &sheet_items[..6]);
        checks ^= sheet_model.domains.len();
        checks ^= sheet_model.disjunctions.len();
        let sheet_clearance =
            verify_clearance_2d(black_box(&sheet_items), &sheet_placements, r(0)).unwrap();
        checks ^= sheet_clearance.pairs.len();
        let shelf =
            shelf_next_fit_decreasing_height_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= shelf.trace.emitted_candidates;
        let first_fit =
            shelf_first_fit_decreasing_height_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= first_fit.trace.emitted_candidates;
        let best_fit =
            shelf_best_fit_decreasing_height_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= best_fit.trace.emitted_candidates;
        let skyline = skyline_bottom_left_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= skyline.trace.emitted_candidates;
        let skyline_waste = skyline_minimum_waste_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= skyline_waste.trace.emitted_candidates;
        let maxrects =
            maxrects_best_short_side_fit_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= maxrects.trace.emitted_candidates;
        let maxrects_long =
            maxrects_best_long_side_fit_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= maxrects_long.trace.emitted_candidates;
        let maxrects_area = maxrects_best_area_fit_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= maxrects_area.trace.emitted_candidates;
        let maxrects_bl = maxrects_bottom_left_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= maxrects_bl.trace.emitted_candidates;
        let maxrects_contact =
            maxrects_contact_point_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= maxrects_contact.trace.emitted_candidates;
        let guillotine = guillotine_best_area_fit_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= guillotine.trace.emitted_candidates;
        let guillotine_short =
            guillotine_best_short_side_fit_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= guillotine_short.trace.emitted_candidates;
        let guillotine_long =
            guillotine_best_long_side_fit_2d(black_box(&sheet_bin), &sheet_items).unwrap();
        checks ^= guillotine_long.trace.emitted_candidates;
        let sheet_portfolio = auto_sheet_portfolio_2d(
            black_box(&sheet_bin),
            &sheet_items,
            SheetPortfolioBudget2 { max_algorithms: 13 },
        )
        .unwrap();
        checks ^= sheet_portfolio.evaluated.len();
        checks ^= sheet_portfolio
            .best
            .as_ref()
            .map(|best| best.trace.emitted_candidates)
            .unwrap_or(0);
        checks ^=
            snapshot_sheet_2d_text(black_box(&sheet_bin), &sheet_items, &sheet_placements).len();
        checks ^=
            snapshot_sheet_2d_binary(black_box(&sheet_bin), &sheet_items, &sheet_placements).len();
        let oriented = verify_oriented_packing_2d(
            black_box(&sheet_bin),
            &oriented_sheet_items,
            &oriented_sheet_placements,
        )
        .unwrap();
        checks ^= oriented.sheet.no_overlap_checks;
        checks ^= snapshot_packing_3d_text(black_box(&bin), &items, &placements).len();
        checks ^= snapshot_packing_3d_binary(black_box(&bin), &items, &placements).len();
    }
    let elapsed = started.elapsed();
    println!(
        "packing_feasibility_and_objective_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checks={checks}, waste_seen={waste_seen}",
        elapsed / iterations
    );
}
