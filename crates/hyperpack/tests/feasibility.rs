use hyperpack::{
    AxisBox3, Bin3, BinId, BinInstance3, CapacityBoundStatus, ClearanceStatus2, ClearanceStatus3,
    CuboidHeuristic3, CuboidPortfolioBudget3, CuboidPortfolioStatus3, DomainBoxFact3,
    DomainConstraintHandoff, DomainCrate, DomainFactStatus, DomainHandoffStatus,
    FeasibilityReplay3, FeasibilityStatus, FreeSpaceReport3, HeuristicFamily, Item3, ItemId,
    LowerBoundReport, ModelExportStatus2, ModelExportStatus3, MultiBinPlacement3,
    NoOverlapDisjunct2, NoOverlapDisjunct3, ObjectiveTerm3, PackError, PackingReport3, Placement3,
    Real, SheetHeuristic2, SheetPortfolioBudget2, SheetPortfolioStatus2, analyze_packing_3d,
    auto_cuboid_portfolio_3d, auto_sheet_portfolio_2d, branch_and_bound_one_bin_3d,
    capacity_bounds_2d, capacity_bounds_3d, compare_objectives_3d,
    cuboid_best_fit_decreasing_footprint_area_3d, cuboid_best_fit_decreasing_max_side_3d,
    cuboid_best_fit_decreasing_volume_3d, cuboid_extreme_point_decreasing_volume_3d,
    cuboid_first_fit_decreasing_footprint_area_3d, cuboid_first_fit_decreasing_max_side_3d,
    cuboid_first_fit_decreasing_volume_3d, cuboid_guillotine_best_volume_fit_3d,
    cuboid_laff_largest_area_fit_first_3d, cuboid_maximal_space_decreasing_volume_3d,
    empty_bins_3d, export_no_overlap_model_2d, export_no_overlap_model_3d,
    guillotine_best_area_fit_2d, guillotine_best_long_side_fit_2d,
    guillotine_best_short_side_fit_2d, height_objective_3d, import_domain_bin_3d,
    import_domain_items_3d, local_search_order_3d, maxrects_best_area_fit_2d,
    maxrects_best_long_side_fit_2d, maxrects_best_short_side_fit_2d, maxrects_bottom_left_2d,
    maxrects_contact_point_2d, order_placements_3d, pair_incompatibilities_2d,
    pair_incompatibilities_3d, shelf_best_fit_decreasing_height_2d,
    shelf_first_fit_decreasing_height_2d, shelf_next_fit_decreasing_height_2d,
    skyline_bottom_left_2d, skyline_minimum_waste_2d, summarize_domain_handoffs,
    verify_clearance_2d, verify_clearance_3d, verify_multi_bin_packing_3d, verify_packing_3d,
    verify_support_3d,
};
use hyperpack::{BinEmptyingConfig3, BinEmptyingStatus3};
use hyperpack::{ExactSearchLimit3, ExactSearchStatus3};
use hyperpack::{
    ItemWeight3, LoadLimit3, SupportPolicy3, SupportStatus3, verify_direct_stack_load_3d,
};
use hyperpack::{LocalSearchConfig3, LocalSearchStatus3, MultistartConfig3, MultistartStatus3};
use hyperpack::{
    Orientation2, OrientedSheetItem2, OrientedSheetPlacement2, Rect2, SheetBin2, SheetItem2,
    SheetPlacement2, verify_oriented_packing_2d, verify_packing_2d,
};
use hyperpack::{Orientation3, OrientedItem3, OrientedPlacement3, verify_oriented_packing_3d};
use hyperpack::{ReinsertUnplacedConfig3, ReinsertUnplacedStatus3};
use hyperpack::{StockBin1, StockItem1, StockPlacement1, verify_packing_1d};
use hyperpack::{TabuSearchConfig3, TabuSearchStatus3};
use hyperpack::{
    snapshot_packing_3d_binary, snapshot_packing_3d_text, snapshot_sheet_2d_binary,
    snapshot_sheet_2d_text, snapshot_stock_1d_binary, snapshot_stock_1d_text,
};
use hyperreal::Rational;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn item(id: &str, x: i32, y: i32, z: i32) -> Item3 {
    Item3 {
        id: ItemId::new(id).unwrap(),
        size: AxisBox3::new(r(x), r(y), r(z)).unwrap(),
    }
}

fn placement(id: &str, x: i32, y: i32, z: i32) -> Placement3 {
    Placement3 {
        item: ItemId::new(id).unwrap(),
        x: r(x),
        y: r(y),
        z: r(z),
    }
}

fn bin_instance(id: &str, x: i32, y: i32, z: i32, cost: i32) -> BinInstance3 {
    BinInstance3::new(
        BinId::new(id).unwrap(),
        Bin3 {
            size: AxisBox3::new(r(x), r(y), r(z)).unwrap(),
        },
        r(cost),
    )
    .unwrap()
}

fn multi_placement(bin: &str, item: &str, x: i32, y: i32, z: i32) -> MultiBinPlacement3 {
    MultiBinPlacement3 {
        bin: BinId::new(bin).unwrap(),
        item: ItemId::new(item).unwrap(),
        x: r(x),
        y: r(y),
        z: r(z),
    }
}

fn stock_item(id: &str, length: i32) -> StockItem1 {
    StockItem1::new(ItemId::new(id).unwrap(), r(length)).unwrap()
}

fn stock_placement(id: &str, start: i32) -> StockPlacement1 {
    StockPlacement1 {
        item: ItemId::new(id).unwrap(),
        start: r(start),
    }
}

fn sheet_item(id: &str, x: i32, y: i32) -> SheetItem2 {
    SheetItem2::new(ItemId::new(id).unwrap(), Rect2::new(r(x), r(y)).unwrap())
}

fn sheet_placement(id: &str, x: i32, y: i32) -> SheetPlacement2 {
    SheetPlacement2 {
        item: ItemId::new(id).unwrap(),
        x: r(x),
        y: r(y),
    }
}

fn oriented_sheet_item(id: &str, x: i32, y: i32, allowed: Vec<Orientation2>) -> OrientedSheetItem2 {
    OrientedSheetItem2::new(
        ItemId::new(id).unwrap(),
        Rect2::new(r(x), r(y)).unwrap(),
        allowed,
        "test-unit",
    )
}

fn oriented_sheet_placement(
    id: &str,
    x: i32,
    y: i32,
    orientation: Orientation2,
) -> OrientedSheetPlacement2 {
    OrientedSheetPlacement2 {
        item: ItemId::new(id).unwrap(),
        x: r(x),
        y: r(y),
        orientation,
    }
}

fn oriented_item3(id: &str, x: i32, y: i32, z: i32, allowed: Vec<Orientation3>) -> OrientedItem3 {
    OrientedItem3::new(
        ItemId::new(id).unwrap(),
        AxisBox3::new(r(x), r(y), r(z)).unwrap(),
        allowed,
        "test-unit",
    )
}

fn oriented_placement3(
    id: &str,
    x: i32,
    y: i32,
    z: i32,
    orientation: Orientation3,
) -> OrientedPlacement3 {
    OrientedPlacement3 {
        item: ItemId::new(id).unwrap(),
        x: r(x),
        y: r(y),
        z: r(z),
        orientation,
    }
}

#[test]
fn declared_item_ids_must_be_unique_in_every_dimension() {
    let id = ItemId::new("duplicate").unwrap();

    let stock_bin = StockBin1::new(r(10)).unwrap();
    let stock_items = [stock_item("duplicate", 2), stock_item("duplicate", 3)];
    assert_eq!(
        verify_packing_1d(&stock_bin, &stock_items, &[]).unwrap_err(),
        PackError::DuplicateItem
    );

    let sheet_bin = SheetBin2::new(Rect2::new(r(10), r(10)).unwrap());
    let sheet_items = [sheet_item("duplicate", 2, 2), sheet_item("duplicate", 3, 3)];
    assert_eq!(
        verify_packing_2d(&sheet_bin, &sheet_items, &[]).unwrap_err(),
        PackError::DuplicateItem
    );

    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [
        Item3 {
            id: id.clone(),
            size: AxisBox3::new(r(2), r(2), r(2)).unwrap(),
        },
        Item3 {
            id,
            size: AxisBox3::new(r(3), r(3), r(3)).unwrap(),
        },
    ];
    assert_eq!(
        verify_packing_3d(&bin, &items, &[]).unwrap_err(),
        PackError::DuplicateItem
    );
}

#[test]
fn exact_1d_verification_accepts_contained_non_overlapping_intervals() {
    let bin = StockBin1::new(r(10)).unwrap();
    let items = [stock_item("a", 4), stock_item("b", 6)];
    let placements = [stock_placement("a", 0), stock_placement("b", 4)];

    let report = verify_packing_1d(&bin, &items, &placements).unwrap();

    assert_eq!(report.status, FeasibilityStatus::Feasible);
    assert_eq!(report.containment_checks, 2);
    assert_eq!(report.no_overlap_checks, 1);
    assert_eq!(report.objective.used_length, r(10));
    assert_eq!(report.objective.waste_length, r(0));
}

#[test]
fn exact_1d_verification_rejects_overlap_outside_and_duplicates() {
    let bin = StockBin1::new(r(10)).unwrap();
    let items = [stock_item("a", 6), stock_item("b", 6), stock_item("c", 1)];

    let overlap = verify_packing_1d(
        &bin,
        &items,
        &[stock_placement("a", 0), stock_placement("b", 4)],
    )
    .unwrap();
    assert_eq!(overlap.status, FeasibilityStatus::Infeasible);
    assert!(overlap.facts[0].contains("overlaps"));

    let outside = verify_packing_1d(&bin, &items, &[stock_placement("a", 5)]).unwrap();
    assert_eq!(outside.status, FeasibilityStatus::Infeasible);
    assert!(outside.facts[0].contains("outside"));

    let duplicate = verify_packing_1d(
        &bin,
        &items,
        &[stock_placement("c", 0), stock_placement("c", 1)],
    )
    .unwrap();
    assert_eq!(duplicate.status, FeasibilityStatus::Infeasible);
    assert_eq!(duplicate.objective.duplicate_placements, 1);
    assert_eq!(duplicate.unplaced.len(), 2);
    assert_eq!(duplicate.duplicates[0].as_str(), "c");
}

#[test]
fn deterministic_stock_snapshot_serializes_exact_scalars_as_text() {
    let bin = StockBin1::new(r(10)).unwrap();
    let items = [stock_item("a\nb", 4)];
    let placements = [stock_placement("a\nb", 0)];

    let snapshot = snapshot_stock_1d_text(&bin, &items, &placements);

    assert_eq!(
        snapshot,
        "hyperpack-snapshot-v1\nkind\tstock-1d\nbin\t10\nitem\ta\\nb\t4\nplacement\ta\\nb\t0"
    );
    assert!(!snapshot.contains("10.0"));
}

#[test]
fn deterministic_stock_binary_snapshot_uses_length_prefixed_exact_text_fields() {
    let bin = StockBin1::new(r(10)).unwrap();
    let items = [stock_item("a\nb", 4)];
    let placements = [stock_placement("a\nb", 0)];

    let snapshot = snapshot_stock_1d_binary(&bin, &items, &placements);

    assert_eq!(&snapshot[..4], b"HPB1");
    assert!(snapshot.windows(2).any(|window| window == b"10"));
    assert!(!snapshot.windows(4).any(|window| window == b"10.0"));
    assert!(snapshot.windows(3).any(|window| window == b"a\nb"));
}

#[test]
fn exact_2d_verification_accepts_contained_non_overlapping_rectangles() {
    let bin = SheetBin2::new(Rect2::new(r(10), r(5)).unwrap());
    let items = [sheet_item("a", 4, 5), sheet_item("b", 6, 5)];
    let placements = [sheet_placement("a", 0, 0), sheet_placement("b", 4, 0)];

    let report = verify_packing_2d(&bin, &items, &placements).unwrap();

    assert_eq!(report.status, FeasibilityStatus::Feasible);
    assert_eq!(report.containment_checks, 2);
    assert_eq!(report.no_overlap_checks, 1);
    assert_eq!(report.objective.used_area, r(50));
    assert_eq!(report.objective.waste_area, r(0));
}

#[test]
fn exact_2d_verification_rejects_overlap_outside_and_duplicates() {
    let bin = SheetBin2::new(Rect2::new(r(10), r(10)).unwrap());
    let items = [
        sheet_item("a", 6, 6),
        sheet_item("b", 6, 6),
        sheet_item("c", 1, 1),
    ];

    let overlap = verify_packing_2d(
        &bin,
        &items,
        &[sheet_placement("a", 0, 0), sheet_placement("b", 4, 0)],
    )
    .unwrap();
    assert_eq!(overlap.status, FeasibilityStatus::Infeasible);
    assert!(overlap.facts[0].contains("overlaps"));

    let outside = verify_packing_2d(&bin, &items, &[sheet_placement("a", 5, 5)]).unwrap();
    assert_eq!(outside.status, FeasibilityStatus::Infeasible);
    assert!(outside.facts[0].contains("outside"));

    let duplicate = verify_packing_2d(
        &bin,
        &items,
        &[sheet_placement("c", 0, 0), sheet_placement("c", 1, 0)],
    )
    .unwrap();
    assert_eq!(duplicate.status, FeasibilityStatus::Infeasible);
    assert_eq!(duplicate.objective.duplicate_placements, 1);
    assert_eq!(duplicate.unplaced.len(), 2);
    assert_eq!(duplicate.duplicates[0].as_str(), "c");
}

#[test]
fn exact_2d_replay_accepts_edge_contacts_gaps_and_prime_denominators() {
    let touching_bin = SheetBin2::new(Rect2::new(r(6), r(3)).unwrap());
    let touching_items = [sheet_item("left", 3, 3), sheet_item("right", 3, 3)];
    let touching_placements = [
        sheet_placement("left", 0, 0),
        sheet_placement("right", 3, 0),
    ];
    let touching = verify_packing_2d(&touching_bin, &touching_items, &touching_placements).unwrap();
    assert_eq!(touching.status, FeasibilityStatus::Feasible);
    assert_eq!(touching.no_overlap_checks, 1);

    let gap_bin = SheetBin2::new(Rect2::new(r(7), r(3)).unwrap());
    let gap = verify_packing_2d(
        &gap_bin,
        &touching_items,
        &[
            sheet_placement("left", 0, 0),
            sheet_placement("right", 4, 0),
        ],
    )
    .unwrap();
    assert_eq!(gap.status, FeasibilityStatus::Feasible);
    assert_eq!(gap.objective.waste_area, r(3));

    let rational_bin = SheetBin2::new(Rect2::new(r(1), r(1)).unwrap());
    let rational_items = [
        SheetItem2::new(
            ItemId::new("third").unwrap(),
            Rect2::new(q(1, 3), r(1)).unwrap(),
        ),
        SheetItem2::new(
            ItemId::new("two-thirds").unwrap(),
            Rect2::new(q(2, 3), r(1)).unwrap(),
        ),
    ];
    let rational_placements = [
        SheetPlacement2 {
            item: ItemId::new("third").unwrap(),
            x: r(0),
            y: r(0),
        },
        SheetPlacement2 {
            item: ItemId::new("two-thirds").unwrap(),
            x: q(1, 3),
            y: r(0),
        },
    ];
    let rational = verify_packing_2d(&rational_bin, &rational_items, &rational_placements).unwrap();
    assert_eq!(rational.status, FeasibilityStatus::Feasible);
    assert_eq!(rational.objective.used_area, r(1));
    assert_eq!(rational.objective.waste_area, r(0));
}

#[test]
fn exact_2d_clearance_distinguishes_edge_contact_from_kerf_gap() {
    let items = [sheet_item("left", 2, 2), sheet_item("right", 2, 2)];
    let touching = [
        sheet_placement("left", 0, 0),
        sheet_placement("right", 2, 0),
    ];
    let gap = [
        sheet_placement("left", 0, 0),
        sheet_placement("right", 3, 0),
    ];

    let zero = verify_clearance_2d(&items, &touching, r(0)).unwrap();
    let kerf = verify_clearance_2d(&items, &touching, r(1)).unwrap();
    let satisfied = verify_clearance_2d(&items, &gap, r(1)).unwrap();

    assert_eq!(zero.status, ClearanceStatus2::Satisfied);
    assert_eq!(zero.pairs[0].separating_gap, Some(r(0)));
    assert_eq!(kerf.status, ClearanceStatus2::Violated);
    assert_eq!(kerf.pairs[0].satisfied, Some(false));
    assert_eq!(satisfied.status, ClearanceStatus2::Satisfied);
    assert_eq!(satisfied.pairs[0].separating_gap, Some(r(1)));
}

#[test]
fn exact_2d_clearance_rejects_negative_kerf() {
    let items = [sheet_item("only", 1, 1)];
    let placements = [sheet_placement("only", 0, 0)];

    assert_eq!(
        verify_clearance_2d(&items, &placements, r(-1)).unwrap_err(),
        PackError::NegativeClearance
    );
}

#[test]
fn exact_2d_rejects_zero_and_negative_dimensions() {
    assert_eq!(
        Rect2::new(r(0), r(1)).unwrap_err(),
        PackError::NonPositiveDimension
    );
    assert_eq!(
        Rect2::new(r(1), r(-1)).unwrap_err(),
        PackError::NonPositiveDimension
    );
}

#[test]
fn deterministic_sheet_snapshot_serializes_exact_scalars_as_text() {
    let bin = SheetBin2::new(Rect2::new(r(10), r(5)).unwrap());
    let items = [sheet_item("panel\tA", 4, 5)];
    let placements = [sheet_placement("panel\tA", 0, 0)];

    let snapshot = snapshot_sheet_2d_text(&bin, &items, &placements);

    assert_eq!(
        snapshot,
        "hyperpack-snapshot-v1\nkind\tsheet-2d\nbin\t10\t5\nitem\tpanel\\tA\t4\t5\nplacement\tpanel\\tA\t0\t0"
    );
    assert!(!snapshot.contains(".0"));
}

#[test]
fn deterministic_sheet_binary_snapshot_preserves_raw_ids_and_exact_scalars() {
    let bin = SheetBin2::new(Rect2::new(r(10), r(5)).unwrap());
    let items = [sheet_item("panel\tA", 4, 5)];
    let placements = [sheet_placement("panel\tA", 0, 0)];

    let snapshot = snapshot_sheet_2d_binary(&bin, &items, &placements);

    assert_eq!(&snapshot[..4], b"HPB1");
    assert!(snapshot.windows(7).any(|window| window == b"panel\tA"));
    assert!(snapshot.windows(2).any(|window| window == b"10"));
    assert!(!snapshot.windows(4).any(|window| window == b"10.0"));
}

#[test]
fn shelf_nfdh_proposes_rows_and_replays_exactly() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("short", 3, 1),
        sheet_item("tall", 3, 2),
        sheet_item("wide", 3, 2),
    ];

    let report = shelf_next_fit_decreasing_height_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::NextFitDecreasingHeight);
    assert_eq!(report.trace.considered_items, 3);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert_eq!(report.trace.rejected_items, 0);
    assert_eq!(report.trace.opened_shelves, 2);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.candidates[0].placement.item.as_str(), "tall");
    assert_eq!(report.candidates[1].placement.item.as_str(), "wide");
    assert_eq!(report.candidates[2].placement.item.as_str(), "short");
    assert!(!report.free_rects.is_empty());
}

#[test]
fn shelf_nfdh_reports_rejected_impossible_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(5), r(5)).unwrap());
    let items = [sheet_item("fits", 5, 5), sheet_item("too-wide", 6, 1)];

    let report = shelf_next_fit_decreasing_height_2d(&bin, &items).unwrap();

    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "too-wide");
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(
        report
            .replay
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn shelf_ffdh_uses_first_existing_shelf_that_fits() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(6)).unwrap());
    let items = [
        sheet_item("a", 4, 3),
        sheet_item("b", 4, 2),
        sheet_item("c", 2, 2),
    ];

    let report = shelf_first_fit_decreasing_height_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::FirstFitDecreasingHeight);
    assert_eq!(report.trace.opened_shelves, 2);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.candidates[2].placement.item.as_str(), "c");
    assert_eq!(report.candidates[2].shelf_index, 0);
    assert_eq!(report.candidates[2].placement.x, r(4));
    assert_eq!(report.candidates[2].placement.y, r(0));
}

#[test]
fn shelf_bfdh_uses_tightest_certified_shelf_that_fits() {
    let bin = SheetBin2::new(Rect2::new(r(10), r(8)).unwrap());
    let items = [
        sheet_item("a", 7, 4),
        sheet_item("b", 4, 3),
        sheet_item("c", 2, 3),
    ];

    let report = shelf_best_fit_decreasing_height_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::BestFitDecreasingHeight);
    assert_eq!(report.trace.opened_shelves, 2);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.candidates[2].placement.item.as_str(), "c");
    assert_eq!(report.candidates[2].shelf_index, 0);
    assert_eq!(report.candidates[2].placement.x, r(7));
    assert_eq!(report.candidates[2].placement.y, r(0));
}

#[test]
fn skyline_bottom_left_fills_lowest_leftmost_exact_candidate() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("left", 3, 2),
        sheet_item("right", 3, 2),
        sheet_item("top-left", 2, 2),
    ];

    let report = skyline_bottom_left_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::SkylineBottomLeft);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert!(report.trace.candidate_positions >= 5);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.candidates[0].placement.x, r(0));
    assert_eq!(report.candidates[0].placement.y, r(0));
    assert_eq!(report.candidates[1].placement.x, r(3));
    assert_eq!(report.candidates[1].placement.y, r(0));
    assert_eq!(report.candidates[2].placement.x, r(0));
    assert_eq!(report.candidates[2].placement.y, r(2));
}

#[test]
fn skyline_bottom_left_reports_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fits", 3, 3), sheet_item("blocked", 1, 1)];

    let report = skyline_bottom_left_2d(&bin, &items).unwrap();

    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(
        report
            .replay
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn skyline_minimum_waste_can_choose_higher_tighter_candidate() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(5)).unwrap());
    let items = [sheet_item("floor", 5, 1), sheet_item("upright", 1, 4)];

    let bottom_left = skyline_bottom_left_2d(&bin, &items).unwrap();
    let minimum_waste = skyline_minimum_waste_2d(&bin, &items).unwrap();

    assert_eq!(
        minimum_waste.heuristic,
        SheetHeuristic2::SkylineMinimumWaste
    );
    assert_eq!(bottom_left.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(minimum_waste.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(bottom_left.candidates[1].placement.x, r(5));
    assert_eq!(bottom_left.candidates[1].placement.y, r(0));
    assert_eq!(minimum_waste.candidates[1].placement.x, r(0));
    assert_eq!(minimum_waste.candidates[1].placement.y, r(1));
}

#[test]
fn skyline_minimum_waste_reports_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fits", 3, 3), sheet_item("blocked", 1, 1)];

    let report = skyline_minimum_waste_2d(&bin, &items).unwrap();

    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(
        report
            .replay
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn maxrects_bssf_chooses_tight_short_side_free_rectangle() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("wide", 4, 2),
        sheet_item("leftover-top", 2, 2),
        sheet_item("tight", 2, 2),
    ];

    let report = maxrects_best_short_side_fit_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::MaxRectsBestShortSideFit);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.candidates[2].placement.x, r(4));
    assert_eq!(report.candidates[2].placement.y, r(2));
    assert!(!report.free_rects.is_empty());
}

#[test]
fn maxrects_bssf_reports_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fits", 3, 3), sheet_item("blocked", 1, 1)];

    let report = maxrects_best_short_side_fit_2d(&bin, &items).unwrap();

    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(
        report
            .replay
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn maxrects_blsf_and_baf_report_distinct_heuristics_with_exact_replay() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("wide", 4, 2),
        sheet_item("leftover-top", 2, 2),
        sheet_item("tight", 2, 2),
    ];

    let long_side = maxrects_best_long_side_fit_2d(&bin, &items).unwrap();
    let area_fit = maxrects_best_area_fit_2d(&bin, &items).unwrap();

    assert_eq!(
        long_side.heuristic,
        SheetHeuristic2::MaxRectsBestLongSideFit
    );
    assert_eq!(area_fit.heuristic, SheetHeuristic2::MaxRectsBestAreaFit);
    assert_eq!(long_side.trace.emitted_candidates, 3);
    assert_eq!(area_fit.trace.emitted_candidates, 3);
    assert_eq!(long_side.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(area_fit.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(long_side.candidates[2].placement.x, r(4));
    assert_eq!(area_fit.candidates[2].placement.y, r(2));
}

#[test]
fn maxrects_blsf_and_baf_report_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fits", 3, 3), sheet_item("blocked", 1, 1)];

    let long_side = maxrects_best_long_side_fit_2d(&bin, &items).unwrap();
    let area_fit = maxrects_best_area_fit_2d(&bin, &items).unwrap();

    assert_eq!(long_side.rejected[0].as_str(), "blocked");
    assert_eq!(area_fit.rejected[0].as_str(), "blocked");
    assert_eq!(long_side.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(area_fit.replay.status, FeasibilityStatus::Feasible);
}

#[test]
fn maxrects_bottom_left_reports_heuristic_and_exact_replay() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("wide", 4, 2),
        sheet_item("leftover-top", 2, 2),
        sheet_item("tight", 2, 2),
    ];

    let report = maxrects_bottom_left_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::MaxRectsBottomLeft);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.candidates[2].placement.x, r(0));
    assert_eq!(report.candidates[2].placement.y, r(2));
}

#[test]
fn maxrects_bottom_left_reports_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fits", 3, 3), sheet_item("blocked", 1, 1)];

    let report = maxrects_bottom_left_2d(&bin, &items).unwrap();

    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
}

#[test]
fn maxrects_contact_point_prefers_edge_and_neighbor_contact() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("wide", 4, 2),
        sheet_item("leftover-top", 2, 2),
        sheet_item("tight", 2, 2),
    ];

    let report = maxrects_contact_point_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::MaxRectsContactPoint);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.candidates[2].placement.x, r(4));
    assert_eq!(report.candidates[2].placement.y, r(2));
}

#[test]
fn maxrects_contact_point_reports_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fits", 3, 3), sheet_item("blocked", 1, 1)];

    let report = maxrects_contact_point_2d(&bin, &items).unwrap();

    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(
        report
            .replay
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn guillotine_best_area_fit_splits_and_replays_exactly() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("wide", 4, 2),
        sheet_item("right", 2, 2),
        sheet_item("top", 6, 2),
    ];

    let report = guillotine_best_area_fit_2d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, SheetHeuristic2::GuillotineBestAreaFit);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(report.replay.objective.used_area, r(24));
    assert!(report.free_rects.is_empty());
}

#[test]
fn guillotine_best_area_fit_reports_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fills", 3, 3), sheet_item("blocked", 1, 1)];

    let report = guillotine_best_area_fit_2d(&bin, &items).unwrap();

    assert_eq!(report.trace.emitted_candidates, 1);
    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(
        report
            .replay
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn guillotine_short_and_long_side_variants_report_distinct_heuristics() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(4)).unwrap());
    let items = [
        sheet_item("wide", 4, 2),
        sheet_item("right", 2, 2),
        sheet_item("top", 6, 2),
    ];

    let short = guillotine_best_short_side_fit_2d(&bin, &items).unwrap();
    let long = guillotine_best_long_side_fit_2d(&bin, &items).unwrap();

    assert_eq!(short.heuristic, SheetHeuristic2::GuillotineBestShortSideFit);
    assert_eq!(long.heuristic, SheetHeuristic2::GuillotineBestLongSideFit);
    assert_eq!(short.trace.emitted_candidates, 3);
    assert_eq!(long.trace.emitted_candidates, 3);
    assert_eq!(short.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(long.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(short.replay.objective.used_area, r(24));
    assert_eq!(long.replay.objective.used_area, r(24));
}

#[test]
fn guillotine_short_and_long_side_variants_report_unplaceable_items_before_replay() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(3)).unwrap());
    let items = [sheet_item("fills", 3, 3), sheet_item("blocked", 1, 1)];

    let short = guillotine_best_short_side_fit_2d(&bin, &items).unwrap();
    let long = guillotine_best_long_side_fit_2d(&bin, &items).unwrap();

    assert_eq!(short.trace.emitted_candidates, 1);
    assert_eq!(long.trace.emitted_candidates, 1);
    assert_eq!(short.trace.rejected_items, 1);
    assert_eq!(long.trace.rejected_items, 1);
    assert_eq!(short.rejected[0].as_str(), "blocked");
    assert_eq!(long.rejected[0].as_str(), "blocked");
    assert_eq!(short.replay.status, FeasibilityStatus::Feasible);
    assert_eq!(long.replay.status, FeasibilityStatus::Feasible);
}

#[test]
fn auto_sheet_portfolio_ranks_exact_replay_objectives() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(5)).unwrap());
    let items = [sheet_item("floor", 5, 1), sheet_item("upright", 1, 4)];

    let report =
        auto_sheet_portfolio_2d(&bin, &items, SheetPortfolioBudget2 { max_algorithms: 5 }).unwrap();

    assert_eq!(report.status, SheetPortfolioStatus2::Complete);
    assert_eq!(report.evaluated.len(), 5);
    assert_eq!(
        report.best.as_ref().unwrap().replay.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(
        report
            .best
            .as_ref()
            .unwrap()
            .replay
            .objective
            .unplaced_items,
        0
    );
    assert_eq!(
        report.best.as_ref().unwrap().replay.objective.used_area,
        r(9)
    );
}

#[test]
fn auto_sheet_portfolio_reports_zero_budget_explicitly() {
    let bin = SheetBin2::new(Rect2::new(r(6), r(5)).unwrap());
    let items = [sheet_item("only", 1, 1)];

    let report =
        auto_sheet_portfolio_2d(&bin, &items, SheetPortfolioBudget2 { max_algorithms: 0 }).unwrap();

    assert_eq!(report.status, SheetPortfolioStatus2::BudgetExhausted);
    assert!(report.best.is_none());
    assert!(report.evaluated.is_empty());
    assert!(report.facts[0].contains("budget"));
}

#[test]
fn oriented_2d_verification_applies_legal_cardinal_rotation_exactly() {
    let bin = SheetBin2::new(Rect2::new(r(3), r(4)).unwrap());
    let items = [oriented_sheet_item(
        "rotated",
        4,
        3,
        vec![Orientation2::Deg90],
    )];
    let placements = [oriented_sheet_placement(
        "rotated",
        0,
        0,
        Orientation2::Deg90,
    )];

    let report = verify_oriented_packing_2d(&bin, &items, &placements).unwrap();

    assert_eq!(report.orientation.checked_items, 1);
    assert_eq!(report.orientation.checked_placements, 1);
    assert!(report.orientation.facts.is_empty());
    assert_eq!(report.sheet.status, FeasibilityStatus::Feasible);
    assert_eq!(report.sheet.objective.used_area, r(12));
    assert_eq!(report.sheet.objective.waste_area, r(0));
}

#[test]
fn oriented_2d_verification_rejects_empty_and_illegal_orientation_policies() {
    let bin = SheetBin2::new(Rect2::new(r(5), r(5)).unwrap());
    let items = [
        oriented_sheet_item("empty", 1, 1, vec![]),
        oriented_sheet_item("fixed", 2, 1, vec![Orientation2::Deg0]),
    ];
    let placements = [
        oriented_sheet_placement("empty", 0, 0, Orientation2::Deg0),
        oriented_sheet_placement("fixed", 1, 0, Orientation2::Deg90),
    ];

    let report = verify_oriented_packing_2d(&bin, &items, &placements).unwrap();

    assert_eq!(report.sheet.status, FeasibilityStatus::Infeasible);
    assert_eq!(
        report.orientation.empty_orientation_items[0].as_str(),
        "empty"
    );
    assert_eq!(
        report.orientation.illegal_orientation_items[0].as_str(),
        "empty"
    );
    assert_eq!(
        report.orientation.illegal_orientation_items[1].as_str(),
        "fixed"
    );
    assert!(
        report
            .sheet
            .facts
            .iter()
            .any(|fact| fact.contains("disallowed orientation"))
    );
}

#[test]
fn exact_replay_accepts_contained_non_overlapping_boxes() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [item("a", 5, 5, 5), item("b", 5, 5, 5)];
    let placements = [placement("a", 0, 0, 0), placement("b", 5, 0, 0)];

    let replay = FeasibilityReplay3::replay(&bin, &items, &placements).unwrap();

    assert_eq!(replay.status, FeasibilityStatus::Feasible);
    assert_eq!(replay.containment_checks, 2);
    assert_eq!(replay.no_overlap_checks, 1);
}

#[test]
fn exact_replay_rejects_overlap_and_outside_bin() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [item("a", 6, 6, 6), item("b", 6, 6, 6)];

    let overlap = FeasibilityReplay3::replay(
        &bin,
        &items,
        &[placement("a", 0, 0, 0), placement("b", 4, 0, 0)],
    )
    .unwrap();
    assert_eq!(overlap.status, FeasibilityStatus::Infeasible);
    assert!(overlap.facts[0].contains("overlaps"));

    let outside = FeasibilityReplay3::replay(&bin, &items, &[placement("a", 5, 5, 5)]).unwrap();
    assert_eq!(outside.status, FeasibilityStatus::Infeasible);
    assert!(outside.facts[0].contains("outside"));
}

#[test]
fn exact_3d_replay_accepts_face_edge_corner_contacts_and_gaps() {
    let bin = Bin3 {
        size: AxisBox3::new(r(2), r(2), r(2)).unwrap(),
    };
    let items = [
        item("origin", 1, 1, 1),
        item("face", 1, 1, 1),
        item("edge", 1, 1, 1),
        item("corner", 1, 1, 1),
    ];
    let placements = [
        placement("origin", 0, 0, 0),
        placement("face", 1, 0, 0),
        placement("edge", 1, 1, 0),
        placement("corner", 1, 1, 1),
    ];

    let report = verify_packing_3d(&bin, &items, &placements).unwrap();

    assert_eq!(report.feasibility.status, FeasibilityStatus::Feasible);
    assert_eq!(report.feasibility.no_overlap_checks, 6);
    assert_eq!(report.objective.used_volume, r(4));

    let gap_bin = Bin3 {
        size: AxisBox3::new(r(3), r(1), r(1)).unwrap(),
    };
    let gap_items = [item("left", 1, 1, 1), item("right", 1, 1, 1)];
    let gap = verify_packing_3d(
        &gap_bin,
        &gap_items,
        &[placement("left", 0, 0, 0), placement("right", 2, 0, 0)],
    )
    .unwrap();
    assert_eq!(gap.feasibility.status, FeasibilityStatus::Feasible);
    assert_eq!(gap.objective.waste_volume, r(1));
}

#[test]
fn clearance_replay_distinguishes_contact_from_positive_gap() {
    let items = [item("left", 1, 1, 1), item("right", 1, 1, 1)];
    let touching = [placement("left", 0, 0, 0), placement("right", 1, 0, 0)];
    let gap = [placement("left", 0, 0, 0), placement("right", 2, 0, 0)];

    let zero = verify_clearance_3d(&items, &touching, r(0)).unwrap();
    let positive = verify_clearance_3d(&items, &touching, r(1)).unwrap();
    let satisfied = verify_clearance_3d(&items, &gap, r(1)).unwrap();

    assert_eq!(zero.status, ClearanceStatus3::Satisfied);
    assert_eq!(zero.pairs[0].separating_gap, Some(r(0)));
    assert_eq!(positive.status, ClearanceStatus3::Violated);
    assert_eq!(positive.pairs[0].satisfied, Some(false));
    assert_eq!(satisfied.status, ClearanceStatus3::Satisfied);
    assert_eq!(satisfied.pairs[0].separating_gap, Some(r(1)));
}

#[test]
fn clearance_replay_rejects_negative_clearance() {
    let items = [item("only", 1, 1, 1)];
    let placements = [placement("only", 0, 0, 0)];

    assert_eq!(
        verify_clearance_3d(&items, &placements, r(-1)).unwrap_err(),
        PackError::NegativeClearance
    );
}

#[test]
fn ordered_3d_replay_agrees_with_input_order_for_permuted_layouts() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(2), r(1)).unwrap(),
    };
    let items = [item("b", 2, 2, 1), item("a", 2, 2, 1)];
    let placements = [placement("b", 2, 0, 0), placement("a", 0, 0, 0)];

    let raw = verify_packing_3d(&bin, &items, &placements).unwrap();
    let order = order_placements_3d(&placements);
    let replay = verify_packing_3d(&bin, &items, &order.placements).unwrap();

    assert_eq!(order.input_placements, 2);
    assert_eq!(order.placements[0].item.as_str(), "a");
    assert_eq!(order.unknown_orderings, 0);
    assert_eq!(replay.feasibility.status, raw.feasibility.status);
    assert_eq!(replay.objective.used_volume, raw.objective.used_volume);
    assert_eq!(replay.objective.waste_volume, raw.objective.waste_volume);
}

#[test]
fn packing_analysis_3d_collapses_demand_and_caches_exact_problem_facts() {
    let bin = Bin3 {
        size: AxisBox3::new(q(30, 3), q(15, 3), q(6, 3)).unwrap(),
    };
    let items = [
        Item3 {
            id: ItemId::new("b").unwrap(),
            size: AxisBox3::new(q(6, 3), q(3, 3), q(3, 3)).unwrap(),
        },
        Item3 {
            id: ItemId::new("a").unwrap(),
            size: AxisBox3::new(q(6, 3), q(3, 3), q(3, 3)).unwrap(),
        },
        Item3 {
            id: ItemId::new("c").unwrap(),
            size: AxisBox3::new(q(3, 3), q(3, 3), q(3, 3)).unwrap(),
        },
    ];

    let analysis = analyze_packing_3d(&bin, &items);

    assert_eq!(analysis.demand_classes.len(), 2);
    assert_eq!(analysis.demand_classes[0].item_ids[0].as_str(), "a");
    assert_eq!(analysis.demand_classes[0].count, 2);
    assert_eq!(analysis.demand_classes[0].total_volume, r(4));
    assert_eq!(analysis.dimensions.item_count, 3);
    assert_eq!(analysis.dimensions.total_item_volume, r(5));
    assert_eq!(analysis.dimensions.max_item_x, Some(r(2)));
    assert!(analysis.grid.scalar_facts.all_exact_rational);
    assert!(analysis.grid.shared_denominator_schedule);
    assert_eq!(analysis.metadata.scalar_values, 12);
    assert_eq!(analysis.metadata.demand_class_reduction, 1);
    assert_eq!(analysis.metadata.initial_free_boxes, 1);
    assert_eq!(analysis.metadata.expected_replay_pair_checks, 3);
    assert_eq!(
        analysis.capacity_bound.status,
        CapacityBoundStatus::Satisfied
    );
    assert!(analysis.pair_bound.incompatible_pairs.is_empty());
    assert_eq!(
        analysis.initial_free_boxes[0].size.volume(),
        bin.size.volume()
    );
    assert!(analysis.facts.iter().any(|fact| fact.contains("collapsed")));
}

#[test]
fn packing_analysis_3d_preserves_lower_bound_violations() {
    let bin = Bin3 {
        size: AxisBox3::new(r(2), r(2), r(2)).unwrap(),
    };
    let items = [item("wide", 3, 1, 1), item("tall", 1, 1, 3)];

    let analysis = analyze_packing_3d(&bin, &items);

    assert_eq!(
        analysis.capacity_bound.status,
        CapacityBoundStatus::Violated
    );
    assert!(analysis.capacity_bound.proves_infeasible());
    assert!(
        analysis
            .capacity_bound
            .facts
            .iter()
            .any(|fact| fact.contains("exceeds bin"))
    );
    assert!(analysis.metadata.capacity_bound_cached);
    assert!(analysis.metadata.pair_bound_cached);
}

#[test]
fn deterministic_3d_snapshot_serializes_exact_scalars_as_text() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(1)).unwrap(),
    };
    let items = [item("box\\A", 2, 2, 1)];
    let placements = [placement("box\\A", 0, 0, 0)];

    let snapshot = snapshot_packing_3d_text(&bin, &items, &placements);

    assert_eq!(
        snapshot,
        "hyperpack-snapshot-v1\nkind\tpacking-3d\nbin\t10\t10\t1\nitem\tbox\\\\A\t2\t2\t1\nplacement\tbox\\\\A\t0\t0\t0"
    );
    assert!(!snapshot.contains(".0"));
}

#[test]
fn deterministic_3d_binary_snapshot_preserves_raw_ids_and_exact_scalars() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(1)).unwrap(),
    };
    let items = [item("box\\A", 2, 2, 1)];
    let placements = [placement("box\\A", 0, 0, 0)];

    let snapshot = snapshot_packing_3d_binary(&bin, &items, &placements);

    assert_eq!(&snapshot[..4], b"HPB1");
    assert!(snapshot.windows(5).any(|window| window == b"box\\A"));
    assert!(snapshot.windows(2).any(|window| window == b"10"));
    assert!(!snapshot.windows(4).any(|window| window == b"10.0"));
}

#[test]
fn oriented_3d_verification_applies_legal_axis_permutation_exactly() {
    let bin = Bin3 {
        size: AxisBox3::new(r(2), r(3), r(4)).unwrap(),
    };
    let items = [oriented_item3("rotated", 4, 2, 3, vec![Orientation3::Yzx])];
    let placements = [oriented_placement3("rotated", 0, 0, 0, Orientation3::Yzx)];

    let report = verify_oriented_packing_3d(&bin, &items, &placements).unwrap();

    assert_eq!(report.orientation.checked_items, 1);
    assert_eq!(report.orientation.checked_placements, 1);
    assert!(report.orientation.facts.is_empty());
    assert_eq!(
        report.packing.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(report.packing.objective.used_volume, r(24));
    assert_eq!(report.packing.objective.waste_volume, r(0));
}

#[test]
fn oriented_3d_verification_rejects_empty_and_illegal_orientation_policies() {
    let bin = Bin3 {
        size: AxisBox3::new(r(5), r(5), r(5)).unwrap(),
    };
    let items = [
        oriented_item3("empty", 1, 1, 1, vec![]),
        oriented_item3("fixed", 2, 1, 1, vec![Orientation3::Xyz]),
    ];
    let placements = [
        oriented_placement3("empty", 0, 0, 0, Orientation3::Xyz),
        oriented_placement3("fixed", 1, 0, 0, Orientation3::Yxz),
    ];

    let report = verify_oriented_packing_3d(&bin, &items, &placements).unwrap();

    assert_eq!(
        report.packing.feasibility.status,
        FeasibilityStatus::Infeasible
    );
    assert_eq!(
        report.orientation.empty_orientation_items[0].as_str(),
        "empty"
    );
    assert_eq!(
        report.orientation.illegal_orientation_items[0].as_str(),
        "empty"
    );
    assert_eq!(
        report.orientation.illegal_orientation_items[1].as_str(),
        "fixed"
    );
    assert!(
        report
            .packing
            .feasibility
            .facts
            .iter()
            .any(|fact| fact.contains("disallowed orientation"))
    );
}

#[test]
fn support_replay_accepts_full_base_stack_and_threshold_support() {
    let items = [item("base", 4, 4, 1), item("top", 2, 2, 1)];
    let placements = [placement("base", 0, 0, 0), placement("top", 1, 1, 1)];

    let full = verify_support_3d(&items, &placements, SupportPolicy3::FullBase).unwrap();
    let half = verify_support_3d(
        &items,
        &placements,
        SupportPolicy3::AreaRatio {
            numerator: 1,
            denominator: 2,
        },
    )
    .unwrap();

    assert_eq!(full.status, SupportStatus3::Satisfied);
    assert_eq!(half.status, SupportStatus3::Satisfied);
    assert_eq!(full.evidence[1].supported_area, r(4));
    assert_eq!(full.evidence[1].footprint_area, r(4));
    assert_eq!(full.evidence[1].supporters[0].as_str(), "base");
}

#[test]
fn support_replay_rejects_overhang_and_invalid_ratio() {
    let items = [item("base", 2, 2, 1), item("top", 4, 2, 1)];
    let placements = [placement("base", 0, 0, 0), placement("top", 0, 0, 1)];

    let full = verify_support_3d(&items, &placements, SupportPolicy3::FullBase).unwrap();
    let half = verify_support_3d(
        &items,
        &placements,
        SupportPolicy3::AreaRatio {
            numerator: 1,
            denominator: 2,
        },
    )
    .unwrap();

    assert_eq!(full.status, SupportStatus3::Violated);
    assert!(full.proves_unsupported());
    assert_eq!(full.evidence[1].supported_area, r(4));
    assert_eq!(full.evidence[1].footprint_area, r(8));
    assert_eq!(half.status, SupportStatus3::Satisfied);
    assert_eq!(
        verify_support_3d(
            &items,
            &placements,
            SupportPolicy3::AreaRatio {
                numerator: 1,
                denominator: 0,
            },
        )
        .unwrap_err(),
        PackError::InvalidSupportRatio
    );
}

#[test]
fn support_replay_checks_center_of_mass_projection_patch() {
    let items = [item("base", 2, 2, 1), item("top", 5, 2, 1)];
    let centered = [placement("base", 1, 0, 0), placement("top", 0, 0, 1)];
    let overhung = [placement("base", 0, 0, 0), placement("top", 0, 0, 1)];

    let centered_report =
        verify_support_3d(&items, &centered, SupportPolicy3::CenterOfMassProjection).unwrap();
    let overhung_report =
        verify_support_3d(&items, &overhung, SupportPolicy3::CenterOfMassProjection).unwrap();

    assert_eq!(centered_report.status, SupportStatus3::Satisfied);
    assert_eq!(centered_report.evidence[1].center_projected, Some(true));
    assert_eq!(overhung_report.status, SupportStatus3::Violated);
    assert_eq!(overhung_report.evidence[1].center_projected, Some(false));
}

#[test]
fn direct_stack_load_accepts_and_rejects_exact_limits() {
    let items = [item("base", 4, 4, 1), item("top", 2, 2, 1)];
    let placements = [placement("base", 0, 0, 0), placement("top", 1, 1, 1)];
    let weights = [
        ItemWeight3::new(ItemId::new("base").unwrap(), r(10)).unwrap(),
        ItemWeight3::new(ItemId::new("top").unwrap(), r(3)).unwrap(),
    ];
    let accepting_limits = [LoadLimit3::new(ItemId::new("base").unwrap(), r(3)).unwrap()];
    let rejecting_limits = [LoadLimit3::new(ItemId::new("base").unwrap(), r(2)).unwrap()];

    let accepted =
        verify_direct_stack_load_3d(&items, &placements, &weights, &accepting_limits).unwrap();
    let rejected =
        verify_direct_stack_load_3d(&items, &placements, &weights, &rejecting_limits).unwrap();

    assert_eq!(accepted.status, SupportStatus3::Unknown);
    assert_eq!(accepted.evidence[0].direct_supported_weight, Some(r(3)));
    assert_eq!(accepted.evidence[0].within_limit, Some(true));
    assert_eq!(rejected.status, SupportStatus3::Violated);
    assert_eq!(rejected.evidence[0].within_limit, Some(false));
    assert!(
        rejected
            .facts
            .iter()
            .any(|fact| fact.contains("stack-load limit"))
    );
}

#[test]
fn direct_stack_load_reports_missing_weight_and_negative_input() {
    let items = [item("base", 4, 4, 1), item("top", 2, 2, 1)];
    let placements = [placement("base", 0, 0, 0), placement("top", 1, 1, 1)];
    let weights = [ItemWeight3::new(ItemId::new("base").unwrap(), r(10)).unwrap()];
    let limits = [LoadLimit3::new(ItemId::new("base").unwrap(), r(3)).unwrap()];

    let report = verify_direct_stack_load_3d(&items, &placements, &weights, &limits).unwrap();

    assert_eq!(report.status, SupportStatus3::Unknown);
    assert!(report.facts.iter().any(|fact| fact.contains("no weight")));
    assert_eq!(
        ItemWeight3::new(ItemId::new("bad").unwrap(), r(-1)).unwrap_err(),
        PackError::NegativeLoadValue
    );
    assert_eq!(
        LoadLimit3::new(ItemId::new("bad").unwrap(), r(-1)).unwrap_err(),
        PackError::NegativeLoadValue
    );
}

#[test]
fn direct_stack_load_rejects_conflicting_keyed_evidence() {
    let items = [item("base", 4, 4, 1)];
    let placements = [placement("base", 0, 0, 0)];
    let duplicate_weights = [
        ItemWeight3::new(ItemId::new("base").unwrap(), r(1)).unwrap(),
        ItemWeight3::new(ItemId::new("base").unwrap(), r(2)).unwrap(),
    ];
    let duplicate_limits = [
        LoadLimit3::new(ItemId::new("base").unwrap(), r(1)).unwrap(),
        LoadLimit3::new(ItemId::new("base").unwrap(), r(2)).unwrap(),
    ];

    assert_eq!(
        verify_direct_stack_load_3d(&items, &placements, &duplicate_weights, &[]).unwrap_err(),
        PackError::DuplicateItemWeight
    );
    assert_eq!(
        verify_direct_stack_load_3d(&items, &placements, &[], &duplicate_limits).unwrap_err(),
        PackError::DuplicateLoadLimit
    );
}

#[test]
fn cuboid_first_fit_decreasing_volume_places_at_first_exact_corner() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(2)).unwrap(),
    };
    let items = [item("large", 3, 4, 2), item("side", 3, 2, 2)];

    let report = cuboid_first_fit_decreasing_volume_3d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, CuboidHeuristic3::FirstFitDecreasingVolume);
    assert_eq!(report.trace.emitted_candidates, 2);
    assert_eq!(report.trace.rejected_items, 0);
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(report.candidates[1].placement.x, r(3));
    assert_eq!(report.candidates[1].placement.y, r(0));
    assert_eq!(report.candidates[1].placement.z, r(0));
}

#[test]
fn cuboid_best_fit_decreasing_volume_reports_rejection_before_replay() {
    let bin = Bin3 {
        size: AxisBox3::new(r(3), r(3), r(3)).unwrap(),
    };
    let items = [item("fills", 3, 3, 3), item("blocked", 1, 1, 1)];

    let report = cuboid_best_fit_decreasing_volume_3d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, CuboidHeuristic3::BestFitDecreasingVolume);
    assert_eq!(report.trace.emitted_candidates, 1);
    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert!(
        report
            .replay
            .feasibility
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn cuboid_max_side_and_footprint_orders_report_distinct_heuristics() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(3)).unwrap(),
    };
    let items = [
        item("rod", 6, 1, 1),
        item("slab", 3, 4, 1),
        item("cube", 2, 2, 2),
    ];

    let first_side = cuboid_first_fit_decreasing_max_side_3d(&bin, &items).unwrap();
    let best_side = cuboid_best_fit_decreasing_max_side_3d(&bin, &items).unwrap();
    let first_area = cuboid_first_fit_decreasing_footprint_area_3d(&bin, &items).unwrap();
    let best_area = cuboid_best_fit_decreasing_footprint_area_3d(&bin, &items).unwrap();

    assert_eq!(
        first_side.heuristic,
        CuboidHeuristic3::FirstFitDecreasingMaxSide
    );
    assert_eq!(
        best_side.heuristic,
        CuboidHeuristic3::BestFitDecreasingMaxSide
    );
    assert_eq!(
        first_area.heuristic,
        CuboidHeuristic3::FirstFitDecreasingFootprintArea
    );
    assert_eq!(
        best_area.heuristic,
        CuboidHeuristic3::BestFitDecreasingFootprintArea
    );
    assert_eq!(
        first_side.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(
        best_side.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(
        first_area.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(
        best_area.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
}

#[test]
fn cuboid_extreme_point_prefers_deep_bottom_left_exact_corner() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(4)).unwrap(),
    };
    let items = [item("base", 4, 2, 2), item("cube", 2, 2, 2)];

    let report = cuboid_extreme_point_decreasing_volume_3d(&bin, &items).unwrap();

    assert_eq!(
        report.heuristic,
        CuboidHeuristic3::ExtremePointDecreasingVolume
    );
    assert_eq!(report.trace.emitted_candidates, 2);
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(report.candidates[1].placement.x, r(4));
    assert_eq!(report.candidates[1].placement.y, r(0));
    assert_eq!(report.candidates[1].placement.z, r(0));
}

#[test]
fn cuboid_maximal_space_splits_free_boxes_and_replays_exactly() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(2)).unwrap(),
    };
    let items = [
        item("left", 4, 2, 2),
        item("right", 2, 2, 2),
        item("top", 6, 2, 2),
    ];

    let report = cuboid_maximal_space_decreasing_volume_3d(&bin, &items).unwrap();

    assert_eq!(
        report.heuristic,
        CuboidHeuristic3::MaximalSpaceDecreasingVolume
    );
    assert_eq!(report.trace.emitted_candidates, 3);
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(report.replay.objective.used_volume, r(48));
    assert!(report.free_boxes.is_empty());
}

#[test]
fn cuboid_maximal_space_reports_unplaceable_items_before_replay() {
    let bin = Bin3 {
        size: AxisBox3::new(r(3), r(3), r(3)).unwrap(),
    };
    let items = [item("fills", 3, 3, 3), item("blocked", 1, 1, 1)];

    let report = cuboid_maximal_space_decreasing_volume_3d(&bin, &items).unwrap();

    assert_eq!(report.trace.emitted_candidates, 1);
    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert!(
        report
            .replay
            .feasibility
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn cuboid_guillotine_best_volume_fit_reports_cut_state_and_replay() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(2)).unwrap(),
    };
    let items = [
        item("top", 6, 2, 2),
        item("left", 4, 2, 2),
        item("right", 2, 2, 2),
    ];

    let report = cuboid_guillotine_best_volume_fit_3d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, CuboidHeuristic3::GuillotineBestVolumeFit);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert!(report.trace.candidate_points >= 3);
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(report.replay.objective.waste_volume, r(0));
}

#[test]
fn cuboid_guillotine_best_volume_fit_reports_unplaceable_items_before_replay() {
    let bin = Bin3 {
        size: AxisBox3::new(r(3), r(3), r(3)).unwrap(),
    };
    let items = [item("fills", 3, 3, 3), item("blocked", 1, 1, 1)];

    let report = cuboid_guillotine_best_volume_fit_3d(&bin, &items).unwrap();

    assert_eq!(report.trace.emitted_candidates, 1);
    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert!(
        report
            .replay
            .feasibility
            .facts
            .iter()
            .any(|fact| fact.contains("proposal rejected"))
    );
}

#[test]
fn cuboid_laff_largest_area_fit_first_reports_low_layer_replay() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(2)).unwrap(),
    };
    let items = [
        item("base", 4, 4, 1),
        item("side", 2, 2, 2),
        item("top", 4, 4, 1),
    ];

    let report = cuboid_laff_largest_area_fit_first_3d(&bin, &items).unwrap();

    assert_eq!(report.heuristic, CuboidHeuristic3::LaffLargestAreaFitFirst);
    assert_eq!(report.trace.emitted_candidates, 3);
    assert!(report.trace.candidate_points >= 3);
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(report.replay.objective.used_volume, r(40));
}

#[test]
fn cuboid_laff_reports_unplaceable_items_before_replay() {
    let bin = Bin3 {
        size: AxisBox3::new(r(3), r(3), r(3)).unwrap(),
    };
    let items = [item("fills", 3, 3, 3), item("blocked", 1, 1, 1)];

    let report = cuboid_laff_largest_area_fit_first_3d(&bin, &items).unwrap();

    assert_eq!(report.trace.emitted_candidates, 1);
    assert_eq!(report.trace.rejected_items, 1);
    assert_eq!(report.rejected[0].as_str(), "blocked");
    assert_eq!(
        report.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
}

#[test]
fn auto_cuboid_portfolio_ranks_exact_replay_objectives() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(2)).unwrap(),
    };
    let items = [item("left", 4, 2, 2), item("right", 2, 2, 2)];

    let report =
        auto_cuboid_portfolio_3d(&bin, &items, CuboidPortfolioBudget3 { max_algorithms: 10 })
            .unwrap();

    assert_eq!(report.status, CuboidPortfolioStatus3::Complete);
    assert_eq!(report.evaluated.len(), 10);
    assert_eq!(
        report.best.as_ref().unwrap().replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(
        report
            .best
            .as_ref()
            .unwrap()
            .replay
            .objective
            .unplaced_items,
        0
    );
    assert_eq!(
        report.best.as_ref().unwrap().replay.objective.used_volume,
        r(24)
    );
}

#[test]
fn auto_cuboid_portfolio_reports_zero_budget_explicitly() {
    let bin = Bin3 {
        size: AxisBox3::new(r(6), r(4), r(2)).unwrap(),
    };
    let items = [item("only", 1, 1, 1)];

    let report =
        auto_cuboid_portfolio_3d(&bin, &items, CuboidPortfolioBudget3 { max_algorithms: 0 })
            .unwrap();

    assert_eq!(report.status, CuboidPortfolioStatus3::BudgetExhausted);
    assert!(report.best.is_none());
    assert!(report.evaluated.is_empty());
    assert!(report.facts[0].contains("budget"));
}

#[test]
fn proposal_reports_keep_heuristic_lower_bound_and_free_space_evidence() {
    let report = PackingReport3 {
        heuristic: HeuristicFamily::MaxRects2,
        seed: 42,
        placements: vec![placement("a", 0, 0, 0)],
        free_space: FreeSpaceReport3 {
            boxes: vec![AxisBox3::new(r(5), r(10), r(1)).unwrap()],
            exact: true,
        },
        lower_bound: Some(LowerBoundReport {
            lower_bound: r(1),
            incumbent: Some(r(1)),
            method: "area".into(),
        }),
    };
    assert_eq!(report.seed, 42);
    assert_eq!(report.free_space.boxes.len(), 1);
    assert_eq!(report.lower_bound.unwrap().method, "area");
}

#[test]
fn verification_reports_unplaced_duplicates_and_exact_waste() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(1)).unwrap(),
    };
    let items = [item("a", 2, 2, 1), item("b", 3, 2, 1), item("c", 1, 1, 1)];
    let placements = [
        placement("a", 0, 0, 0),
        placement("a", 2, 0, 0),
        placement("b", 5, 0, 0),
    ];

    let report = verify_packing_3d(&bin, &items, &placements).unwrap();

    assert_eq!(report.feasibility.status, FeasibilityStatus::Infeasible);
    assert_eq!(report.objective.bin_volume, r(100));
    assert_eq!(report.objective.used_volume, r(14));
    assert_eq!(report.objective.waste_volume, r(86));
    assert_eq!(report.objective.placed_items, 2);
    assert_eq!(report.objective.unplaced_items, 1);
    assert_eq!(report.objective.duplicate_placements, 1);
    assert_eq!(report.unplaced[0].as_str(), "c");
    assert_eq!(report.duplicates[0].as_str(), "a");
    assert!(
        report
            .feasibility
            .facts
            .iter()
            .any(|fact| fact.contains("placed more than once"))
    );
}

#[test]
fn height_objective_reports_exact_used_and_remaining_height() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [item("base", 2, 2, 3), item("top", 2, 2, 2)];
    let placements = [placement("base", 0, 0, 0), placement("top", 0, 0, 3)];

    let report = height_objective_3d(&bin, &items, &placements).unwrap();

    assert_eq!(report.checked_placements, 2);
    assert_eq!(report.bin_height, r(10));
    assert_eq!(report.used_height, Some(r(5)));
    assert_eq!(report.remaining_height, Some(r(5)));
    assert_eq!(report.unknown_comparisons, 0);
}

#[test]
fn height_objective_counts_duplicate_records_and_rejects_missing_items() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [item("dup", 2, 2, 1)];
    let placements = [placement("dup", 0, 0, 0), placement("dup", 0, 0, 4)];

    let report = height_objective_3d(&bin, &items, &placements).unwrap();

    assert_eq!(report.checked_placements, 2);
    assert_eq!(report.used_height, Some(r(5)));
    assert_eq!(
        height_objective_3d(&bin, &items, &[placement("missing", 0, 0, 0)]).unwrap_err(),
        PackError::MissingItem
    );
}

#[test]
fn objective_comparison_uses_explicit_lexicographic_policy() {
    let bin = Bin3 {
        size: AxisBox3::new(r(5), r(4), r(1)).unwrap(),
    };
    let items = [
        item("strip-a", 2, 1, 1),
        item("strip-b", 2, 1, 1),
        item("strip-c", 2, 1, 1),
        item("strip-d", 2, 1, 1),
        item("block", 3, 3, 1),
    ];
    let fragmented = [
        placement("strip-a", 0, 0, 0),
        placement("strip-b", 2, 0, 0),
        placement("strip-c", 0, 1, 0),
        placement("strip-d", 2, 1, 0),
    ];
    let repaired = [
        placement("block", 0, 0, 0),
        placement("strip-a", 3, 0, 0),
        placement("strip-b", 3, 1, 0),
        placement("strip-c", 3, 2, 0),
        placement("strip-d", 0, 3, 0),
    ];

    let fragmented_replay = verify_packing_3d(&bin, &items, &fragmented).unwrap();
    let repaired_replay = verify_packing_3d(&bin, &items, &repaired).unwrap();
    let comparison = compare_objectives_3d(
        &repaired_replay,
        None,
        &fragmented_replay,
        None,
        &[ObjectiveTerm3::MinimizeUnplacedItems],
    );

    assert_eq!(comparison.ordering, Some(std::cmp::Ordering::Less));
    assert_eq!(
        comparison.decisive_term,
        Some(ObjectiveTerm3::MinimizeUnplacedItems)
    );
    assert_eq!(comparison.compared_terms, 1);
    assert_eq!(comparison.unknown_terms, 0);
}

#[test]
fn objective_comparison_can_use_height_after_volume_tie() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(4), r(4)).unwrap(),
    };
    let items = [item("a", 2, 2, 2), item("b", 2, 2, 2)];
    let low = [placement("a", 0, 0, 0), placement("b", 2, 0, 0)];
    let high = [placement("a", 0, 0, 0), placement("b", 0, 0, 2)];

    let low_replay = verify_packing_3d(&bin, &items, &low).unwrap();
    let high_replay = verify_packing_3d(&bin, &items, &high).unwrap();
    let low_height = height_objective_3d(&bin, &items, &low).unwrap();
    let high_height = height_objective_3d(&bin, &items, &high).unwrap();
    let comparison = compare_objectives_3d(
        &low_replay,
        Some(&low_height),
        &high_replay,
        Some(&high_height),
        &[
            ObjectiveTerm3::MaximizeUsedVolume,
            ObjectiveTerm3::MinimizeUsedHeight,
        ],
    );

    assert_eq!(comparison.ordering, Some(std::cmp::Ordering::Less));
    assert_eq!(
        comparison.decisive_term,
        Some(ObjectiveTerm3::MinimizeUsedHeight)
    );
    assert_eq!(comparison.compared_terms, 2);
}

#[test]
fn objective_comparison_reports_missing_height_as_unknown() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(4), r(4)).unwrap(),
    };
    let items = [item("a", 1, 1, 1)];
    let placements = [placement("a", 0, 0, 0)];
    let replay = verify_packing_3d(&bin, &items, &placements).unwrap();

    let comparison = compare_objectives_3d(
        &replay,
        None,
        &replay,
        None,
        &[ObjectiveTerm3::MinimizeUsedHeight],
    );

    assert_eq!(comparison.ordering, None);
    assert_eq!(comparison.decisive_term, None);
    assert_eq!(comparison.unknown_terms, 1);
    assert!(comparison.facts[0].contains("MinimizeUsedHeight"));
}

#[test]
fn multi_bin_replay_aggregates_exact_cost_and_waste() {
    let bins = [
        bin_instance("cheap", 4, 2, 1, 5),
        bin_instance("expensive", 2, 2, 1, 7),
    ];
    let items = [item("left", 2, 2, 1), item("right", 2, 2, 1)];
    let placements = [
        multi_placement("cheap", "left", 0, 0, 0),
        multi_placement("cheap", "right", 2, 0, 0),
    ];

    let report = verify_multi_bin_packing_3d(&bins, &items, &placements).unwrap();

    assert_eq!(report.status, FeasibilityStatus::Feasible);
    assert_eq!(report.objective.used_bins, 1);
    assert_eq!(report.objective.total_cost, r(5));
    assert_eq!(report.objective.total_bin_volume, r(8));
    assert_eq!(report.objective.used_volume, r(8));
    assert_eq!(report.objective.waste_volume, r(0));
    assert_eq!(report.bins[0].bin.as_str(), "cheap");
}

#[test]
fn multi_bin_replay_reports_duplicates_unplaced_and_missing_bins() {
    let bins = [bin_instance("a", 2, 2, 1, 3), bin_instance("b", 2, 2, 1, 4)];
    let items = [item("part", 2, 2, 1), item("missing", 1, 1, 1)];
    let placements = [
        multi_placement("a", "part", 0, 0, 0),
        multi_placement("b", "part", 0, 0, 0),
    ];

    let report = verify_multi_bin_packing_3d(&bins, &items, &placements).unwrap();

    assert_eq!(report.status, FeasibilityStatus::Infeasible);
    assert_eq!(report.objective.used_bins, 2);
    assert_eq!(report.objective.total_cost, r(7));
    assert_eq!(report.objective.duplicate_assignments, 1);
    assert_eq!(report.duplicates[0].as_str(), "part");
    assert_eq!(report.unplaced[0].as_str(), "missing");
    assert!(
        report
            .facts
            .iter()
            .any(|fact| fact.contains("assigned more than once"))
    );
    assert_eq!(
        verify_multi_bin_packing_3d(
            &bins,
            &items,
            &[multi_placement("unknown", "part", 0, 0, 0)]
        )
        .unwrap_err(),
        PackError::MissingBin
    );
    assert_eq!(
        verify_multi_bin_packing_3d(
            &[bin_instance("a", 2, 2, 1, 3), bin_instance("a", 3, 3, 1, 4),],
            &items,
            &[]
        )
        .unwrap_err(),
        PackError::DuplicateBin
    );
}

#[test]
fn bin_emptying_repair_moves_assignments_into_existing_bin_with_exact_replay() {
    let bins = [
        bin_instance("roomy", 4, 2, 1, 5),
        bin_instance("spare", 2, 2, 1, 7),
    ];
    let items = [item("left", 2, 2, 1), item("right", 2, 2, 1)];
    let placements = [
        multi_placement("roomy", "left", 0, 0, 0),
        multi_placement("spare", "right", 0, 0, 0),
    ];

    let report = empty_bins_3d(
        &bins,
        &items,
        &placements,
        BinEmptyingConfig3 {
            max_passes: 2,
            max_bins_per_pass: 4,
        },
    )
    .unwrap();

    assert_eq!(report.status, BinEmptyingStatus3::Complete);
    assert_eq!(report.initial.replay.objective.used_bins, 2);
    assert_eq!(report.best.replay.objective.used_bins, 1);
    assert_eq!(report.best.replay.objective.total_cost, r(5));
    assert_eq!(report.accepted_moves.len(), 1);
    assert_eq!(report.accepted_moves[0].emptied_bin.as_str(), "spare");
    assert_eq!(report.accepted_moves[0].moved_items[0].as_str(), "right");
    assert_eq!(report.best.replay.status, FeasibilityStatus::Feasible);
}

#[test]
fn bin_emptying_repair_reports_bin_limit_explicitly() {
    let bins = [bin_instance("a", 2, 2, 1, 3), bin_instance("b", 2, 2, 1, 4)];
    let items = [item("left", 2, 2, 1), item("right", 2, 2, 1)];
    let placements = [
        multi_placement("a", "left", 0, 0, 0),
        multi_placement("b", "right", 0, 0, 0),
    ];

    let report = empty_bins_3d(
        &bins,
        &items,
        &placements,
        BinEmptyingConfig3 {
            max_passes: 2,
            max_bins_per_pass: 0,
        },
    )
    .unwrap();

    assert_eq!(report.status, BinEmptyingStatus3::BinLimit);
    assert_eq!(report.evaluated_bins, 0);
    assert!(report.accepted_moves.is_empty());
}

#[test]
fn capacity_bounds_reject_volume_and_dimension_impossibilities() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(4), r(1)).unwrap(),
    };
    let volume_items = [item("a", 4, 4, 1), item("b", 1, 1, 1)];
    let volume = capacity_bounds_3d(&bin, &volume_items);
    assert_eq!(volume.status, CapacityBoundStatus::Violated);
    assert_eq!(volume.checked_items, 2);
    assert_eq!(volume.total_item_volume, r(17));
    assert_eq!(volume.bin_volume, r(16));
    assert_eq!(volume.volume_excess, Some(r(1)));
    assert_eq!(volume.volume_capacity_ok, Some(false));
    assert!(volume.proves_infeasible());

    let dimension_items = [item("too-wide", 5, 1, 1)];
    let dimension = capacity_bounds_3d(&bin, &dimension_items);
    assert_eq!(dimension.status, CapacityBoundStatus::Violated);
    assert_eq!(dimension.max_dimension_ok, Some(false));
    assert!(dimension.facts[0].contains("exceeds bin x"));
    assert!(dimension.proves_infeasible());
}

#[test]
fn capacity_bounds_2d_reject_area_and_dimension_impossibilities() {
    let bin = SheetBin2::new(Rect2::new(r(4), r(4)).unwrap());
    let area_items = [sheet_item("a", 4, 4), sheet_item("b", 1, 1)];
    let area = capacity_bounds_2d(&bin, &area_items);
    assert_eq!(area.status, CapacityBoundStatus::Violated);
    assert_eq!(area.checked_items, 2);
    assert_eq!(area.total_item_area, r(17));
    assert_eq!(area.bin_area, r(16));
    assert_eq!(area.area_excess, Some(r(1)));
    assert_eq!(area.area_capacity_ok, Some(false));
    assert!(area.proves_infeasible());

    let dimension_items = [sheet_item("too-tall", 1, 5)];
    let dimension = capacity_bounds_2d(&bin, &dimension_items);
    assert_eq!(dimension.status, CapacityBoundStatus::Violated);
    assert_eq!(dimension.max_dimension_ok, Some(false));
    assert!(dimension.facts[0].contains("exceeds sheet y"));
    assert!(dimension.proves_infeasible());
}

#[test]
fn pair_incompatibilities_reject_pairs_without_any_separating_axis() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [
        item("left", 6, 6, 6),
        item("right", 5, 5, 5),
        item("skinny", 4, 1, 1),
    ];

    let report = pair_incompatibilities_3d(&bin, &items);

    assert_eq!(report.status, CapacityBoundStatus::Violated);
    assert_eq!(report.checked_pairs, 3);
    assert_eq!(report.unknown_pairs, 0);
    assert_eq!(report.incompatible_pairs.len(), 1);
    assert_eq!(report.incompatible_pairs[0].left.as_str(), "left");
    assert_eq!(report.incompatible_pairs[0].right.as_str(), "right");
    assert!(report.facts[0].contains("cannot share one bin"));
    assert!(report.proves_infeasible());
}

#[test]
fn pair_incompatibilities_2d_reject_pairs_without_any_separating_axis() {
    let bin = SheetBin2::new(Rect2::new(r(10), r(10)).unwrap());
    let items = [
        sheet_item("left", 6, 6),
        sheet_item("right", 5, 5),
        sheet_item("skinny", 4, 1),
    ];

    let report = pair_incompatibilities_2d(&bin, &items);

    assert_eq!(report.status, CapacityBoundStatus::Violated);
    assert_eq!(report.checked_pairs, 3);
    assert_eq!(report.unknown_pairs, 0);
    assert_eq!(report.incompatible_pairs.len(), 1);
    assert_eq!(report.incompatible_pairs[0].left.as_str(), "left");
    assert_eq!(report.incompatible_pairs[0].right.as_str(), "right");
    assert!(report.facts[0].contains("cannot share one sheet"));
    assert!(report.proves_infeasible());
}

#[test]
fn no_overlap_model_export_2d_preserves_exact_domains_and_disjunctions() {
    let bin = SheetBin2::new(Rect2::new(r(4), r(2)).unwrap());
    let items = [sheet_item("left", 2, 2), sheet_item("right", 2, 2)];

    let report = export_no_overlap_model_2d(&bin, &items);

    assert_eq!(report.status, ModelExportStatus2::Ready);
    assert_eq!(report.domains.len(), 2);
    assert_eq!(report.domains[0].x_min, r(0));
    assert_eq!(report.domains[0].x_max, r(2));
    assert_eq!(report.domains[0].y_max, r(0));
    assert_eq!(report.disjunctions.len(), 1);
    assert_eq!(
        report.disjunctions[0].disjuncts,
        vec![
            NoOverlapDisjunct2::LeftBeforeRightX,
            NoOverlapDisjunct2::RightBeforeLeftX
        ]
    );
}

#[test]
fn no_overlap_model_export_2d_reports_impossible_domains_and_pairs() {
    let bin = SheetBin2::new(Rect2::new(r(4), r(4)).unwrap());
    let too_wide = [sheet_item("wide", 5, 1)];
    let domain = export_no_overlap_model_2d(&bin, &too_wide);
    assert_eq!(domain.status, ModelExportStatus2::Infeasible);
    assert_eq!(domain.lower_bound_status, CapacityBoundStatus::Violated);
    assert!(domain.facts.iter().any(|fact| fact.contains("capacity")));
    assert!(
        domain
            .facts
            .iter()
            .any(|fact| fact.contains("negative x origin"))
    );

    let pair_items = [sheet_item("left", 3, 3), sheet_item("right", 3, 3)];
    let pair = export_no_overlap_model_2d(&bin, &pair_items);
    assert_eq!(pair.status, ModelExportStatus2::Infeasible);
    assert!(pair.disjunctions[0].disjuncts.is_empty());
    assert!(pair.facts.iter().any(|fact| fact.contains("no feasible")));
}

#[test]
fn no_overlap_model_export_preserves_exact_domains_and_disjunctions() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(2), r(1)).unwrap(),
    };
    let items = [item("left", 2, 2, 1), item("right", 2, 2, 1)];

    let report = export_no_overlap_model_3d(&bin, &items);

    assert_eq!(report.status, ModelExportStatus3::Ready);
    assert_eq!(report.domains.len(), 2);
    assert_eq!(report.domains[0].x_min, r(0));
    assert_eq!(report.domains[0].x_max, r(2));
    assert_eq!(report.domains[0].y_max, r(0));
    assert_eq!(report.domains[0].z_max, r(0));
    assert_eq!(report.disjunctions.len(), 1);
    assert_eq!(
        report.disjunctions[0].disjuncts,
        vec![
            NoOverlapDisjunct3::LeftBeforeRightX,
            NoOverlapDisjunct3::RightBeforeLeftX
        ]
    );
}

#[test]
fn no_overlap_model_export_reports_impossible_domains_and_pairs() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(4), r(1)).unwrap(),
    };
    let too_large = [item("too-wide", 5, 1, 1)];
    let pair = [item("a", 3, 3, 1), item("b", 3, 3, 1)];

    let domain_report = export_no_overlap_model_3d(&bin, &too_large);
    let pair_report = export_no_overlap_model_3d(&bin, &pair);

    assert_eq!(domain_report.status, ModelExportStatus3::Infeasible);
    assert!(domain_report.domains[0].x_max == r(-1));
    assert!(
        domain_report
            .facts
            .iter()
            .any(|fact| fact.contains("negative x origin"))
    );
    assert_eq!(pair_report.status, ModelExportStatus3::Infeasible);
    assert!(pair_report.disjunctions[0].disjuncts.is_empty());
    assert!(
        pair_report
            .facts
            .iter()
            .any(|fact| fact.contains("no feasible separating-axis"))
    );
}

#[test]
fn domain_imports_exact_hyperparts_items_and_bin_with_provenance() {
    let item_fact = DomainBoxFact3::new(
        DomainCrate::Hyperparts,
        "part-a",
        AxisBox3::new(r(2), r(3), r(4)).unwrap(),
        "bom:line-1",
        DomainFactStatus::Exact,
    )
    .unwrap();
    let bin_fact = DomainBoxFact3::new(
        DomainCrate::Hyperparts,
        "crate",
        AxisBox3::new(r(10), r(10), r(5)).unwrap(),
        "container:crate",
        DomainFactStatus::Exact,
    )
    .unwrap();

    let items = import_domain_items_3d(DomainCrate::Hyperparts, &[item_fact]).unwrap();
    let bin = import_domain_bin_3d(&bin_fact).unwrap();

    assert_eq!(items.exact_facts, 1);
    assert_eq!(items.items[0].id.as_str(), "part-a");
    assert_eq!(items.items[0].size.volume(), r(24));
    assert_eq!(bin.exact_facts, 1);
    assert_eq!(bin.bin.unwrap().size.volume(), r(500));
}

#[test]
fn domain_imports_preserve_lossy_and_unknown_facts_as_evidence() {
    let lossy = DomainBoxFact3::new(
        DomainCrate::Hypervoxel,
        "occupancy-aabb",
        AxisBox3::new(r(2), r(2), r(2)).unwrap(),
        "voxel:aabb",
        DomainFactStatus::Lossy,
    )
    .unwrap();
    let unknown = DomainBoxFact3::new(
        DomainCrate::Hypercurve,
        "nesting-shape",
        AxisBox3::new(r(1), r(1), r(1)).unwrap(),
        "curve:pending",
        DomainFactStatus::Unknown,
    )
    .unwrap();

    let report = import_domain_items_3d(DomainCrate::Hypervoxel, &[lossy, unknown]).unwrap();

    assert!(report.items.is_empty());
    assert_eq!(report.lossy_facts, 1);
    assert_eq!(report.unknown_facts, 1);
    assert!(report.facts.iter().any(|fact| fact.contains("Lossy")));
    assert!(report.facts.iter().any(|fact| fact.contains("Unknown")));
}

#[test]
fn domain_handoff_summary_keeps_violations_and_lossy_status_visible() {
    let report = summarize_domain_handoffs(vec![
        DomainConstraintHandoff {
            source: DomainCrate::Hyperphysics,
            constraint: "center-of-mass".into(),
            status: DomainHandoffStatus::Satisfied,
            provenance: "physics:com-cert".into(),
        },
        DomainConstraintHandoff {
            source: DomainCrate::Hyperpath,
            constraint: "tool-clearance".into(),
            status: DomainHandoffStatus::Lossy,
            provenance: "path:coarse-grid".into(),
        },
        DomainConstraintHandoff {
            source: DomainCrate::Hyperdrc,
            constraint: "no-go-region".into(),
            status: DomainHandoffStatus::Violated,
            provenance: "drc:rule-7".into(),
        },
    ]);

    assert_eq!(report.status, DomainHandoffStatus::Violated);
    assert_eq!(report.handoffs.len(), 3);
    assert!(report.facts[2].contains("Violated"));
}

#[test]
fn bounded_exact_search_finds_small_feasible_packing() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(2), r(1)).unwrap(),
    };
    let items = [item("left", 2, 2, 1), item("right", 2, 2, 1)];

    let report = branch_and_bound_one_bin_3d(
        &bin,
        &items,
        ExactSearchLimit3 {
            max_items: 4,
            max_nodes: 32,
        },
    )
    .unwrap();

    assert_eq!(report.status, ExactSearchStatus3::Feasible);
    assert!(report.nodes > 0);
    assert!(report.candidate_points >= 2);
    let incumbent = report.incumbent.unwrap();
    assert_eq!(incumbent.feasibility.status, FeasibilityStatus::Feasible);
    assert_eq!(incumbent.objective.unplaced_items, 0);
}

#[test]
fn bounded_exact_search_reports_bound_violation_and_limits() {
    let bin = Bin3 {
        size: AxisBox3::new(r(2), r(2), r(1)).unwrap(),
    };
    let impossible = [item("a", 2, 2, 1), item("b", 1, 1, 1)];
    let too_many = [item("only", 1, 1, 1)];

    let infeasible = branch_and_bound_one_bin_3d(
        &bin,
        &impossible,
        ExactSearchLimit3 {
            max_items: 4,
            max_nodes: 32,
        },
    )
    .unwrap();
    let unknown = branch_and_bound_one_bin_3d(
        &bin,
        &too_many,
        ExactSearchLimit3 {
            max_items: 0,
            max_nodes: 32,
        },
    )
    .unwrap();

    assert_eq!(infeasible.status, ExactSearchStatus3::Infeasible);
    assert_eq!(infeasible.nodes, 0);
    assert_eq!(unknown.status, ExactSearchStatus3::Unknown);
    assert!(unknown.incumbent.is_none());
}

#[test]
fn local_search_order_reports_replay_gated_non_worsening_result() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(3), r(1)).unwrap(),
    };
    let items = [
        item("wide", 4, 1, 1),
        item("block", 2, 2, 1),
        item("tall", 1, 3, 1),
    ];

    let report = local_search_order_3d(
        &bin,
        &items,
        LocalSearchConfig3 {
            max_steps: 4,
            max_neighbors_per_step: 64,
        },
    )
    .unwrap();

    assert!(matches!(
        report.status,
        LocalSearchStatus3::LocalOptimum | LocalSearchStatus3::StepLimit
    ));
    assert!(report.evaluated_moves > 0);
    assert!(
        report.best.replay.objective.unplaced_items
            <= report.initial.replay.objective.unplaced_items
    );
    assert_eq!(
        report.best.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
}

#[test]
fn local_search_order_reports_neighbor_limit_explicitly() {
    let bin = Bin3 {
        size: AxisBox3::new(r(3), r(3), r(1)).unwrap(),
    };
    let items = [item("a", 2, 2, 1), item("b", 1, 1, 1), item("c", 1, 1, 1)];

    let report = local_search_order_3d(
        &bin,
        &items,
        LocalSearchConfig3 {
            max_steps: 2,
            max_neighbors_per_step: 0,
        },
    )
    .unwrap();

    assert_eq!(report.status, LocalSearchStatus3::NeighborLimit);
    assert_eq!(report.evaluated_moves, 0);
    assert!(report.accepted_moves.is_empty());
}

#[test]
fn tabu_search_order_reports_memory_and_replay_gated_best() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(3), r(1)).unwrap(),
    };
    let items = [
        item("wide", 4, 1, 1),
        item("block", 2, 2, 1),
        item("tall", 1, 3, 1),
    ];

    let report = hyperpack::tabu_search_order_3d(
        &bin,
        &items,
        TabuSearchConfig3 {
            max_steps: 3,
            max_neighbors_per_step: 16,
            tabu_tenure: 2,
        },
    )
    .unwrap();

    assert_eq!(report.status, TabuSearchStatus3::StepLimit);
    assert_eq!(report.accepted_moves.len(), 3);
    assert!(report.tabu_memory.len() <= 2);
    assert!(report.evaluated_moves > 0);
    assert!(
        report.best.replay.objective.unplaced_items
            <= report.initial.replay.objective.unplaced_items
    );
    assert_eq!(
        report.best.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
}

#[test]
fn tabu_search_order_reports_neighbor_limit_explicitly() {
    let bin = Bin3 {
        size: AxisBox3::new(r(3), r(3), r(1)).unwrap(),
    };
    let items = [item("a", 2, 2, 1), item("b", 1, 1, 1), item("c", 1, 1, 1)];

    let report = hyperpack::tabu_search_order_3d(
        &bin,
        &items,
        TabuSearchConfig3 {
            max_steps: 2,
            max_neighbors_per_step: 0,
            tabu_tenure: 1,
        },
    )
    .unwrap();

    assert_eq!(report.status, TabuSearchStatus3::NeighborLimit);
    assert_eq!(report.evaluated_moves, 0);
    assert!(report.accepted_moves.is_empty());
    assert!(report.tabu_memory.is_empty());
}

#[test]
fn multistart_order_reports_seeded_replay_ranked_candidates() {
    let bin = Bin3 {
        size: AxisBox3::new(r(4), r(3), r(1)).unwrap(),
    };
    let items = [
        item("wide", 4, 1, 1),
        item("block", 2, 2, 1),
        item("tall", 1, 3, 1),
    ];

    let report =
        hyperpack::multistart_order_3d(&bin, &items, MultistartConfig3 { seed: 7, starts: 5 })
            .unwrap();

    assert_eq!(report.status, MultistartStatus3::Complete);
    assert_eq!(report.evaluations.len(), 5);
    assert_eq!(report.evaluations[0].seed, 7);
    assert_eq!(report.evaluations[4].seed, 11);
    assert_eq!(
        report
            .best
            .as_ref()
            .unwrap()
            .evaluation
            .replay
            .feasibility
            .status,
        FeasibilityStatus::Feasible
    );
    assert!(report.evaluations.iter().any(|evaluation| {
        evaluation.evaluation.order != report.evaluations[0].evaluation.order
    }));
}

#[test]
fn multistart_order_reports_zero_budget_explicitly() {
    let bin = Bin3 {
        size: AxisBox3::new(r(2), r(2), r(1)).unwrap(),
    };
    let items = [item("only", 1, 1, 1)];

    let report = hyperpack::multistart_order_3d(
        &bin,
        &items,
        MultistartConfig3 {
            seed: 123,
            starts: 0,
        },
    )
    .unwrap();

    assert_eq!(report.status, MultistartStatus3::BudgetExhausted);
    assert!(report.evaluations.is_empty());
    assert!(report.best.is_none());
}

#[test]
fn reinsert_unplaced_repairs_fragmenting_order_with_exact_replay() {
    let bin = Bin3 {
        size: AxisBox3::new(r(5), r(4), r(1)).unwrap(),
    };
    let items = [
        item("strip-a", 2, 1, 1),
        item("strip-b", 2, 1, 1),
        item("strip-c", 2, 1, 1),
        item("strip-d", 2, 1, 1),
        item("block", 3, 3, 1),
    ];

    let report = hyperpack::reinsert_unplaced_order_3d(
        &bin,
        &items,
        ReinsertUnplacedConfig3 {
            max_passes: 4,
            max_trials_per_pass: 16,
        },
    )
    .unwrap();

    assert_eq!(report.status, ReinsertUnplacedStatus3::Complete);
    assert_eq!(report.initial.replay.objective.unplaced_items, 1);
    assert_eq!(report.best.replay.objective.unplaced_items, 0);
    assert_eq!(
        report.best.replay.feasibility.status,
        FeasibilityStatus::Feasible
    );
    assert_eq!(report.accepted_moves.len(), 1);
    assert_eq!(report.accepted_moves[0].item.as_str(), "block");
    assert!(report.evaluated_reinsertions > 0);
}

#[test]
fn reinsert_unplaced_reports_trial_limit_explicitly() {
    let bin = Bin3 {
        size: AxisBox3::new(r(2), r(2), r(1)).unwrap(),
    };
    let items = [item("a", 2, 2, 1), item("b", 2, 2, 1)];

    let report = hyperpack::reinsert_unplaced_order_3d(
        &bin,
        &items,
        ReinsertUnplacedConfig3 {
            max_passes: 2,
            max_trials_per_pass: 0,
        },
    )
    .unwrap();

    assert_eq!(report.status, ReinsertUnplacedStatus3::TrialLimit);
    assert_eq!(report.evaluated_reinsertions, 0);
    assert!(report.accepted_moves.is_empty());
}

#[test]
fn invalid_dimensions_and_missing_items_are_rejected() {
    assert_eq!(
        AxisBox3::new(r(0), r(1), r(1)).unwrap_err(),
        PackError::NonPositiveDimension
    );
    let bin = Bin3 {
        size: AxisBox3::new(r(1), r(1), r(1)).unwrap(),
    };
    assert_eq!(
        FeasibilityReplay3::replay(&bin, &[], &[placement("missing", 0, 0, 0)]).unwrap_err(),
        PackError::MissingItem
    );
}

proptest! {
    #[test]
    fn empty_item_ids_are_rejected(id in "\\PC*") {
        if id.is_empty() {
            prop_assert!(ItemId::new(id).is_err());
        } else {
            prop_assert!(ItemId::new(id).is_ok());
        }
    }

    #[test]
    fn generated_single_item_verification_volume_is_exact(
        bin_x in 1_i32..=40,
        bin_y in 1_i32..=40,
        bin_z in 1_i32..=40,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];
        let placements = [placement("only", 0, 0, 0)];

        let report = verify_packing_3d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.feasibility.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.objective.used_volume, r(item_x * item_y * item_z));
        prop_assert_eq!(
            report.objective.waste_volume,
            r(bin_x * bin_y * bin_z - item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_height_objective_accepts_single_contained_item(
        bin_x in 1_i32..=40,
        bin_y in 1_i32..=40,
        bin_z in 1_i32..=40,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
        z in 0_i32..=20,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(z + item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];
        let placements = [placement("only", 0, 0, z)];

        let report = height_objective_3d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.used_height, Some(r(z + item_z)));
        prop_assert_eq!(report.remaining_height, Some(r(bin_z - z - item_z)));
        prop_assert_eq!(report.unknown_comparisons, 0);
    }

    #[test]
    fn generated_objective_comparison_equal_for_same_single_item(
        bin_x in 1_i32..=40,
        bin_y in 1_i32..=40,
        bin_z in 1_i32..=40,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];
        let placements = [placement("only", 0, 0, 0)];
        let replay = verify_packing_3d(&bin, &items, &placements).unwrap();
        let height = height_objective_3d(&bin, &items, &placements).unwrap();

        let comparison = compare_objectives_3d(
            &replay,
            Some(&height),
            &replay,
            Some(&height),
            &[
                ObjectiveTerm3::MinimizeUnplacedItems,
                ObjectiveTerm3::MaximizeUsedVolume,
                ObjectiveTerm3::MinimizeUsedHeight,
            ],
        );

        prop_assert_eq!(comparison.ordering, Some(std::cmp::Ordering::Equal));
        prop_assert_eq!(comparison.decisive_term, None);
        prop_assert_eq!(comparison.unknown_terms, 0);
    }

    #[test]
    fn generated_two_cuboids_touching_on_grid_remain_feasible(
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
        z in 1_i32..=20,
        extra_gap in 0_i32..=5,
    ) {
        let bin = Bin3 {
            size: AxisBox3::new(r(left_x + right_x + extra_gap), r(y), r(z)).unwrap(),
        };
        let items = [item("left", left_x, y, z), item("right", right_x, y, z)];
        let placements = [
            placement("left", 0, 0, 0),
            placement("right", left_x + extra_gap, 0, 0),
        ];

        let report = verify_packing_3d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.feasibility.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.feasibility.no_overlap_checks, 1);
        prop_assert_eq!(report.objective.used_volume, r((left_x + right_x) * y * z));
        prop_assert_eq!(report.objective.waste_volume, r(extra_gap * y * z));
    }

    #[test]
    fn generated_two_cuboids_exact_gap_satisfies_matching_clearance(
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
        z in 1_i32..=20,
        gap in 0_i32..=5,
    ) {
        let items = [item("left", left_x, y, z), item("right", right_x, y, z)];
        let placements = [
            placement("left", 0, 0, 0),
            placement("right", left_x + gap, 0, 0),
        ];

        let report = verify_clearance_3d(&items, &placements, r(gap)).unwrap();

        prop_assert_eq!(report.status, ClearanceStatus3::Satisfied);
        prop_assert_eq!(report.pairs[0].separating_gap.clone(), Some(r(gap)));
        prop_assert_eq!(report.pairs[0].satisfied, Some(true));
    }

    #[test]
    fn generated_multi_bin_single_used_bin_cost_is_exact(
        bin_x in 1_i32..=30,
        bin_y in 1_i32..=30,
        bin_z in 1_i32..=30,
        item_x in 1_i32..=20,
        item_y in 1_i32..=20,
        item_z in 1_i32..=20,
        cost in 0_i32..=100,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bins = [bin_instance("bin", bin_x, bin_y, bin_z, cost)];
        let items = [item("only", item_x, item_y, item_z)];
        let placements = [multi_placement("bin", "only", 0, 0, 0)];

        let report = verify_multi_bin_packing_3d(&bins, &items, &placements).unwrap();

        prop_assert_eq!(report.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.objective.used_bins, 1);
        prop_assert_eq!(report.objective.total_cost, r(cost));
        prop_assert_eq!(
            report.objective.used_volume,
            r(item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_bin_emptying_single_bin_is_complete(
        bin_x in 1_i32..=30,
        bin_y in 1_i32..=30,
        bin_z in 1_i32..=30,
        item_x in 1_i32..=20,
        item_y in 1_i32..=20,
        item_z in 1_i32..=20,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bins = [bin_instance("bin", bin_x, bin_y, bin_z, 1)];
        let items = [item("only", item_x, item_y, item_z)];
        let placements = [multi_placement("bin", "only", 0, 0, 0)];

        let report = empty_bins_3d(
            &bins,
            &items,
            &placements,
            BinEmptyingConfig3 {
                max_passes: 2,
                max_bins_per_pass: 4,
            },
        )
        .unwrap();

        prop_assert_eq!(report.status, BinEmptyingStatus3::Complete);
        prop_assert_eq!(report.best.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.best.replay.objective.used_bins, 1);
        prop_assert_eq!(report.evaluated_bins, 0);
    }

    #[test]
    fn generated_ordered_replay_agrees_with_input_order_for_two_cuboids(
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
        z in 1_i32..=20,
        extra_gap in 0_i32..=5,
    ) {
        let bin = Bin3 {
            size: AxisBox3::new(r(left_x + right_x + extra_gap), r(y), r(z)).unwrap(),
        };
        let items = [item("right", right_x, y, z), item("left", left_x, y, z)];
        let placements = [
            placement("right", left_x + extra_gap, 0, 0),
            placement("left", 0, 0, 0),
        ];

        let raw = verify_packing_3d(&bin, &items, &placements).unwrap();
        let order = order_placements_3d(&placements);
        let replay = verify_packing_3d(&bin, &items, &order.placements).unwrap();

        prop_assert_eq!(replay.feasibility.status, raw.feasibility.status);
        prop_assert_eq!(replay.objective.used_volume, raw.objective.used_volume);
        prop_assert_eq!(replay.objective.waste_volume, raw.objective.waste_volume);
        prop_assert_eq!(order.input_placements, placements.len());
    }

    #[test]
    fn generated_packing_analysis_summarizes_single_exact_item(
        item_x in 1_i32..=20,
        item_y in 1_i32..=20,
        item_z in 1_i32..=20,
        extra_x in 0_i32..=20,
        extra_y in 0_i32..=20,
        extra_z in 0_i32..=20,
    ) {
        let bin = Bin3 {
            size: AxisBox3::new(r(item_x + extra_x), r(item_y + extra_y), r(item_z + extra_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let analysis = analyze_packing_3d(&bin, &items);

        prop_assert_eq!(analysis.demand_classes.len(), 1);
        prop_assert_eq!(analysis.demand_classes[0].count, 1);
        prop_assert_eq!(analysis.dimensions.total_item_volume, r(item_x * item_y * item_z));
        prop_assert!(analysis.grid.integer_grid);
        prop_assert_eq!(analysis.capacity_bound.status, CapacityBoundStatus::Satisfied);
        prop_assert_eq!(analysis.metadata.expected_replay_pair_checks, 0);
        prop_assert_eq!(&analysis.initial_free_boxes[0].size, &bin.size);
    }

    #[test]
    fn generated_two_cuboids_with_one_unit_overlap_are_rejected(
        left_x in 2_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
        z in 1_i32..=20,
    ) {
        let bin = Bin3 {
            size: AxisBox3::new(r(left_x + right_x), r(y), r(z)).unwrap(),
        };
        let items = [item("left", left_x, y, z), item("right", right_x, y, z)];
        let placements = [
            placement("left", 0, 0, 0),
            placement("right", left_x - 1, 0, 0),
        ];

        let report = verify_packing_3d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.feasibility.status, FeasibilityStatus::Infeasible);
        prop_assert!(report.feasibility.facts.iter().any(|fact| fact.contains("overlaps")));
    }

    #[test]
    fn generated_floor_item_support_is_exactly_satisfied(
        item_x in 1_i32..=20,
        item_y in 1_i32..=20,
        item_z in 1_i32..=20,
    ) {
        let items = [item("floor", item_x, item_y, item_z)];
        let placements = [placement("floor", 0, 0, 0)];

        let report = verify_support_3d(&items, &placements, SupportPolicy3::FullBase).unwrap();
        let center =
            verify_support_3d(&items, &placements, SupportPolicy3::CenterOfMassProjection)
                .unwrap();

        prop_assert_eq!(report.status, SupportStatus3::Satisfied);
        prop_assert_eq!(center.status, SupportStatus3::Satisfied);
        prop_assert_eq!(report.evidence[0].on_floor, true);
        prop_assert_eq!(report.evidence[0].footprint_area.clone(), r(item_x * item_y));
        prop_assert_eq!(report.evidence[0].supported_area.clone(), r(item_x * item_y));
        prop_assert_eq!(report.evidence[0].supported, Some(true));
    }

    #[test]
    fn generated_direct_stack_load_accepts_exact_limit(
        base_x in 1_i32..=20,
        base_y in 1_i32..=20,
        top_x in 1_i32..=20,
        top_y in 1_i32..=20,
        top_weight in 0_i32..=50,
        spare in 0_i32..=50,
    ) {
        prop_assume!(top_x <= base_x);
        prop_assume!(top_y <= base_y);
        let items = [item("base", base_x, base_y, 1), item("top", top_x, top_y, 1)];
        let placements = [placement("base", 0, 0, 0), placement("top", 0, 0, 1)];
        let weights = [
            ItemWeight3::new(ItemId::new("base").unwrap(), r(1)).unwrap(),
            ItemWeight3::new(ItemId::new("top").unwrap(), r(top_weight)).unwrap(),
        ];
        let limits = [
            LoadLimit3::new(ItemId::new("base").unwrap(), r(top_weight + spare)).unwrap(),
            LoadLimit3::new(ItemId::new("top").unwrap(), r(0)).unwrap(),
        ];

        let report = verify_direct_stack_load_3d(&items, &placements, &weights, &limits).unwrap();

        prop_assert_eq!(report.status, SupportStatus3::Satisfied);
        prop_assert_eq!(report.evidence[0].direct_supported_weight.clone(), Some(r(top_weight)));
        prop_assert_eq!(report.evidence[0].within_limit, Some(true));
    }

    #[test]
    fn generated_bounded_exact_search_accepts_single_contained_item(
        bin_x in 1_i32..=20,
        bin_y in 1_i32..=20,
        bin_z in 1_i32..=20,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let report = branch_and_bound_one_bin_3d(
            &bin,
            &items,
            ExactSearchLimit3 {
                max_items: 1,
                max_nodes: 8,
            },
        )
        .unwrap();

        prop_assert_eq!(report.status, ExactSearchStatus3::Feasible);
        prop_assert_eq!(
            report.incumbent.unwrap().objective.used_volume,
            r(item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_local_search_order_accepts_single_contained_item(
        bin_x in 1_i32..=20,
        bin_y in 1_i32..=20,
        bin_z in 1_i32..=20,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let report = local_search_order_3d(
            &bin,
            &items,
            LocalSearchConfig3 {
                max_steps: 2,
                max_neighbors_per_step: 8,
            },
        )
        .unwrap();

        prop_assert_eq!(report.best.replay.feasibility.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.best.replay.objective.unplaced_items, 0);
        prop_assert_eq!(
            report.best.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_tabu_search_order_accepts_single_contained_item(
        bin_x in 1_i32..=20,
        bin_y in 1_i32..=20,
        bin_z in 1_i32..=20,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let report = hyperpack::tabu_search_order_3d(
            &bin,
            &items,
            TabuSearchConfig3 {
                max_steps: 2,
                max_neighbors_per_step: 8,
                tabu_tenure: 1,
            },
        )
        .unwrap();

        prop_assert_eq!(report.best.replay.feasibility.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.best.replay.objective.unplaced_items, 0);
        prop_assert_eq!(
            report.best.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_multistart_order_accepts_single_contained_item(
        bin_x in 1_i32..=20,
        bin_y in 1_i32..=20,
        bin_z in 1_i32..=20,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
        seed in any::<u64>(),
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let report = hyperpack::multistart_order_3d(
            &bin,
            &items,
            MultistartConfig3 { seed, starts: 3 },
        )
        .unwrap();

        prop_assert_eq!(report.status, MultistartStatus3::Complete);
        prop_assert_eq!(report.evaluations.len(), 3);
        prop_assert_eq!(
            report.best.as_ref().unwrap().evaluation.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            &report.best.as_ref().unwrap().evaluation.replay.objective.used_volume,
            &r(item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_reinsert_unplaced_accepts_single_contained_item(
        bin_x in 1_i32..=20,
        bin_y in 1_i32..=20,
        bin_z in 1_i32..=20,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let report = hyperpack::reinsert_unplaced_order_3d(
            &bin,
            &items,
            ReinsertUnplacedConfig3 {
                max_passes: 2,
                max_trials_per_pass: 8,
            },
        )
        .unwrap();

        prop_assert_eq!(report.status, ReinsertUnplacedStatus3::Complete);
        prop_assert_eq!(report.best.replay.feasibility.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.best.replay.objective.unplaced_items, 0);
    }

    #[test]
    fn generated_domain_exact_item_import_preserves_volume(
        item_x in 1_i32..=20,
        item_y in 1_i32..=20,
        item_z in 1_i32..=20,
    ) {
        let fact = DomainBoxFact3::new(
            DomainCrate::Hyperparts,
            "generated",
            AxisBox3::new(r(item_x), r(item_y), r(item_z)).unwrap(),
            "proptest",
            DomainFactStatus::Exact,
        )
        .unwrap();

        let report = import_domain_items_3d(DomainCrate::Hyperparts, &[fact]).unwrap();

        prop_assert_eq!(report.items.len(), 1);
        prop_assert_eq!(report.items[0].size.volume(), r(item_x * item_y * item_z));
        prop_assert_eq!(report.exact_facts, 1);
    }

    #[test]
    fn generated_cuboid_decreasing_volume_accepts_single_contained_item(
        bin_x in 1_i32..=40,
        bin_y in 1_i32..=40,
        bin_z in 1_i32..=40,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let first_fit = cuboid_first_fit_decreasing_volume_3d(&bin, &items).unwrap();
        let best_fit = cuboid_best_fit_decreasing_volume_3d(&bin, &items).unwrap();
        let first_side = cuboid_first_fit_decreasing_max_side_3d(&bin, &items).unwrap();
        let best_side = cuboid_best_fit_decreasing_max_side_3d(&bin, &items).unwrap();
        let first_area = cuboid_first_fit_decreasing_footprint_area_3d(&bin, &items).unwrap();
        let best_area = cuboid_best_fit_decreasing_footprint_area_3d(&bin, &items).unwrap();
        let extreme = cuboid_extreme_point_decreasing_volume_3d(&bin, &items).unwrap();
        let maximal = cuboid_maximal_space_decreasing_volume_3d(&bin, &items).unwrap();
        let guillotine = cuboid_guillotine_best_volume_fit_3d(&bin, &items).unwrap();
        let laff = cuboid_laff_largest_area_fit_first_3d(&bin, &items).unwrap();

        prop_assert_eq!(
            first_fit.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            best_fit.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            first_side.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            best_side.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            first_area.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            best_area.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            extreme.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            maximal.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            guillotine.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            laff.replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(first_fit.trace.emitted_candidates, 1);
        prop_assert_eq!(best_fit.trace.emitted_candidates, 1);
        prop_assert_eq!(first_side.trace.emitted_candidates, 1);
        prop_assert_eq!(best_side.trace.emitted_candidates, 1);
        prop_assert_eq!(first_area.trace.emitted_candidates, 1);
        prop_assert_eq!(best_area.trace.emitted_candidates, 1);
        prop_assert_eq!(extreme.trace.emitted_candidates, 1);
        prop_assert_eq!(maximal.trace.emitted_candidates, 1);
        prop_assert_eq!(guillotine.trace.emitted_candidates, 1);
        prop_assert_eq!(laff.trace.emitted_candidates, 1);
        prop_assert_eq!(
            first_fit.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            best_fit.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            first_side.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            best_side.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            first_area.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            best_area.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            extreme.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            maximal.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            guillotine.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            laff.replay.objective.used_volume,
            r(item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_auto_cuboid_portfolio_accepts_single_contained_item(
        bin_x in 1_i32..=40,
        bin_y in 1_i32..=40,
        bin_z in 1_i32..=40,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let report = auto_cuboid_portfolio_3d(
            &bin,
            &items,
            CuboidPortfolioBudget3 { max_algorithms: 10 },
        )
        .unwrap();

        prop_assert_eq!(report.status, CuboidPortfolioStatus3::Complete);
        prop_assert_eq!(
            report.best.as_ref().unwrap().replay.feasibility.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            report
                .best
                .as_ref()
                .unwrap()
                .replay
                .objective
                .used_volume
                .clone(),
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            report.best.as_ref().unwrap().replay.objective.unplaced_items,
            0
        );
    }

    #[test]
    fn generated_single_stock_item_verification_length_is_exact(
        bin_length in 1_i32..=100,
        item_length in 1_i32..=100,
    ) {
        prop_assume!(item_length <= bin_length);
        let bin = StockBin1::new(r(bin_length)).unwrap();
        let items = [stock_item("only", item_length)];
        let placements = [stock_placement("only", 0)];

        let report = verify_packing_1d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.objective.used_length, r(item_length));
        prop_assert_eq!(report.objective.waste_length, r(bin_length - item_length));
    }

    #[test]
    fn generated_single_sheet_item_verification_area_is_exact(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];
        let placements = [sheet_placement("only", 0, 0)];

        let report = verify_packing_2d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.objective.used_area, r(item_x * item_y));
        prop_assert_eq!(
            report.objective.waste_area,
            r(bin_x * bin_y - item_x * item_y)
        );
    }

    #[test]
    fn generated_two_sheet_items_touching_on_grid_remain_feasible(
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        height in 1_i32..=20,
        extra_gap in 0_i32..=5,
    ) {
        let bin = SheetBin2::new(Rect2::new(r(left_x + right_x + extra_gap), r(height)).unwrap());
        let items = [sheet_item("left", left_x, height), sheet_item("right", right_x, height)];
        let placements = [
            sheet_placement("left", 0, 0),
            sheet_placement("right", left_x + extra_gap, 0),
        ];

        let report = verify_packing_2d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.no_overlap_checks, 1);
        prop_assert_eq!(report.objective.used_area, r((left_x + right_x) * height));
        prop_assert_eq!(report.objective.waste_area, r(extra_gap * height));
    }

    #[test]
    fn generated_two_sheet_items_exact_gap_satisfies_matching_clearance(
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        height in 1_i32..=20,
        gap in 0_i32..=5,
    ) {
        let items = [sheet_item("left", left_x, height), sheet_item("right", right_x, height)];
        let placements = [
            sheet_placement("left", 0, 0),
            sheet_placement("right", left_x + gap, 0),
        ];

        let report = verify_clearance_2d(&items, &placements, r(gap)).unwrap();

        prop_assert_eq!(report.status, ClearanceStatus2::Satisfied);
        prop_assert_eq!(report.pairs[0].separating_gap.clone(), Some(r(gap)));
        prop_assert_eq!(report.pairs[0].satisfied, Some(true));
    }

    #[test]
    fn generated_two_sheet_items_with_one_unit_overlap_are_rejected(
        left_x in 2_i32..=20,
        right_x in 1_i32..=20,
        height in 1_i32..=20,
    ) {
        let bin = SheetBin2::new(Rect2::new(r(left_x + right_x), r(height)).unwrap());
        let items = [sheet_item("left", left_x, height), sheet_item("right", right_x, height)];
        let placements = [
            sheet_placement("left", 0, 0),
            sheet_placement("right", left_x - 1, 0),
        ];

        let report = verify_packing_2d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.status, FeasibilityStatus::Infeasible);
        prop_assert!(report.facts.iter().any(|fact| fact.contains("overlaps")));
    }

    #[test]
    fn generated_shelf_nfdh_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = shelf_next_fit_decreasing_height_2d(&bin, &items).unwrap();

        prop_assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.trace.emitted_candidates, 1);
        prop_assert_eq!(report.trace.rejected_items, 0);
        prop_assert_eq!(report.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_shelf_ffdh_and_bfdh_accept_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let first_fit = shelf_first_fit_decreasing_height_2d(&bin, &items).unwrap();
        let best_fit = shelf_best_fit_decreasing_height_2d(&bin, &items).unwrap();

        prop_assert_eq!(first_fit.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(best_fit.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(first_fit.trace.emitted_candidates, 1);
        prop_assert_eq!(best_fit.trace.emitted_candidates, 1);
        prop_assert_eq!(first_fit.replay.objective.used_area, r(item_x * item_y));
        prop_assert_eq!(best_fit.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_skyline_bottom_left_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = skyline_bottom_left_2d(&bin, &items).unwrap();

        prop_assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.trace.emitted_candidates, 1);
        prop_assert_eq!(report.trace.rejected_items, 0);
        prop_assert_eq!(report.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(report.candidates[0].placement.y.clone(), r(0));
        prop_assert_eq!(report.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_skyline_minimum_waste_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = skyline_minimum_waste_2d(&bin, &items).unwrap();

        prop_assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.trace.emitted_candidates, 1);
        prop_assert_eq!(report.trace.rejected_items, 0);
        prop_assert_eq!(report.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(report.candidates[0].placement.y.clone(), r(0));
        prop_assert_eq!(report.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_guillotine_best_area_fit_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = guillotine_best_area_fit_2d(&bin, &items).unwrap();

        prop_assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.trace.emitted_candidates, 1);
        prop_assert_eq!(report.trace.rejected_items, 0);
        prop_assert_eq!(report.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(report.candidates[0].placement.y.clone(), r(0));
        prop_assert_eq!(report.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_guillotine_short_and_long_side_accept_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let short = guillotine_best_short_side_fit_2d(&bin, &items).unwrap();
        let long = guillotine_best_long_side_fit_2d(&bin, &items).unwrap();

        prop_assert_eq!(short.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(long.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(short.trace.emitted_candidates, 1);
        prop_assert_eq!(long.trace.emitted_candidates, 1);
        prop_assert_eq!(short.trace.rejected_items, 0);
        prop_assert_eq!(long.trace.rejected_items, 0);
        prop_assert_eq!(short.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(long.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(short.replay.objective.used_area, r(item_x * item_y));
        prop_assert_eq!(long.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_maxrects_bssf_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = maxrects_best_short_side_fit_2d(&bin, &items).unwrap();

        prop_assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.trace.emitted_candidates, 1);
        prop_assert_eq!(report.trace.rejected_items, 0);
        prop_assert_eq!(report.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(report.candidates[0].placement.y.clone(), r(0));
        prop_assert_eq!(report.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_auto_sheet_portfolio_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = auto_sheet_portfolio_2d(
            &bin,
            &items,
            SheetPortfolioBudget2 { max_algorithms: 11 },
        )
        .unwrap();

        prop_assert_eq!(report.status, SheetPortfolioStatus2::Complete);
        prop_assert_eq!(
            report.best.as_ref().unwrap().replay.status,
            FeasibilityStatus::Feasible
        );
        prop_assert_eq!(
            report
                .best
                .as_ref()
                .unwrap()
                .replay
                .objective
                .used_area
                .clone(),
            r(item_x * item_y)
        );
        prop_assert_eq!(
            report.best.as_ref().unwrap().replay.objective.unplaced_items,
            0
        );
    }

    #[test]
    fn generated_maxrects_blsf_and_baf_accept_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let long_side = maxrects_best_long_side_fit_2d(&bin, &items).unwrap();
        let area_fit = maxrects_best_area_fit_2d(&bin, &items).unwrap();

        prop_assert_eq!(long_side.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(area_fit.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(long_side.trace.emitted_candidates, 1);
        prop_assert_eq!(area_fit.trace.emitted_candidates, 1);
        prop_assert_eq!(long_side.replay.objective.used_area, r(item_x * item_y));
        prop_assert_eq!(area_fit.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_maxrects_bottom_left_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = maxrects_bottom_left_2d(&bin, &items).unwrap();

        prop_assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.trace.emitted_candidates, 1);
        prop_assert_eq!(report.trace.rejected_items, 0);
        prop_assert_eq!(report.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(report.candidates[0].placement.y.clone(), r(0));
        prop_assert_eq!(report.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_maxrects_contact_point_accepts_single_contained_item(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = maxrects_contact_point_2d(&bin, &items).unwrap();

        prop_assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.trace.emitted_candidates, 1);
        prop_assert_eq!(report.trace.rejected_items, 0);
        prop_assert_eq!(report.candidates[0].placement.x.clone(), r(0));
        prop_assert_eq!(report.candidates[0].placement.y.clone(), r(0));
        prop_assert_eq!(report.replay.objective.used_area, r(item_x * item_y));
    }

    #[test]
    fn generated_oriented_sheet_rotation_preserves_exact_area(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        item_x in 1_i32..=30,
        item_y in 1_i32..=30,
        rotate in any::<bool>(),
    ) {
        let (required_x, required_y, orientation) = if rotate {
            (item_y, item_x, Orientation2::Deg90)
        } else {
            (item_x, item_y, Orientation2::Deg0)
        };
        prop_assume!(required_x <= bin_x);
        prop_assume!(required_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [oriented_sheet_item(
            "only",
            item_x,
            item_y,
            vec![Orientation2::Deg0, Orientation2::Deg90],
        )];
        let placements = [oriented_sheet_placement("only", 0, 0, orientation)];

        let report = verify_oriented_packing_2d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.sheet.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(report.sheet.objective.used_area, r(item_x * item_y));
        prop_assert_eq!(
            report.sheet.objective.waste_area,
            r(bin_x * bin_y - item_x * item_y)
        );
    }

    #[test]
    fn generated_oriented_3d_axis_permutation_preserves_exact_volume(
        bin_x in 1_i32..=60,
        bin_y in 1_i32..=60,
        bin_z in 1_i32..=60,
        item_x in 1_i32..=20,
        item_y in 1_i32..=20,
        item_z in 1_i32..=20,
        orientation_index in 0_usize..6,
    ) {
        let orientation = [
            Orientation3::Xyz,
            Orientation3::Xzy,
            Orientation3::Yxz,
            Orientation3::Yzx,
            Orientation3::Zxy,
            Orientation3::Zyx,
        ][orientation_index];
        let (oriented_x, oriented_y, oriented_z) = match orientation {
            Orientation3::Xyz => (item_x, item_y, item_z),
            Orientation3::Xzy => (item_x, item_z, item_y),
            Orientation3::Yxz => (item_y, item_x, item_z),
            Orientation3::Yzx => (item_y, item_z, item_x),
            Orientation3::Zxy => (item_z, item_x, item_y),
            Orientation3::Zyx => (item_z, item_y, item_x),
        };
        prop_assume!(oriented_x <= bin_x);
        prop_assume!(oriented_y <= bin_y);
        prop_assume!(oriented_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [oriented_item3(
            "only",
            item_x,
            item_y,
            item_z,
            vec![
                Orientation3::Xyz,
                Orientation3::Xzy,
                Orientation3::Yxz,
                Orientation3::Yzx,
                Orientation3::Zxy,
                Orientation3::Zyx,
            ],
        )];
        let placements = [oriented_placement3("only", 0, 0, 0, orientation)];

        let report = verify_oriented_packing_3d(&bin, &items, &placements).unwrap();

        prop_assert_eq!(report.packing.feasibility.status, FeasibilityStatus::Feasible);
        prop_assert_eq!(
            report.packing.objective.used_volume,
            r(item_x * item_y * item_z)
        );
        prop_assert_eq!(
            report.packing.objective.waste_volume,
            r(bin_x * bin_y * bin_z - item_x * item_y * item_z)
        );
    }

    #[test]
    fn generated_capacity_bounds_accept_single_contained_item(
        bin_x in 1_i32..=40,
        bin_y in 1_i32..=40,
        bin_z in 1_i32..=40,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
        item_z in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        prop_assume!(item_z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("only", item_x, item_y, item_z)];

        let report = capacity_bounds_3d(&bin, &items);

        prop_assert_eq!(report.status, CapacityBoundStatus::Satisfied);
        prop_assert_eq!(report.checked_items, 1);
        prop_assert_eq!(report.volume_capacity_ok, Some(true));
        prop_assert_eq!(report.max_dimension_ok, Some(true));
        prop_assert!(report.volume_excess.is_none());
        prop_assert!(!report.proves_infeasible());
    }

    #[test]
    fn generated_capacity_bounds_2d_accept_single_contained_item(
        bin_x in 1_i32..=40,
        bin_y in 1_i32..=40,
        item_x in 1_i32..=10,
        item_y in 1_i32..=10,
    ) {
        prop_assume!(item_x <= bin_x);
        prop_assume!(item_y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("only", item_x, item_y)];

        let report = capacity_bounds_2d(&bin, &items);

        prop_assert_eq!(report.status, CapacityBoundStatus::Satisfied);
        prop_assert_eq!(report.checked_items, 1);
        prop_assert_eq!(report.area_capacity_ok, Some(true));
        prop_assert_eq!(report.max_dimension_ok, Some(true));
        prop_assert!(report.area_excess.is_none());
        prop_assert!(!report.proves_infeasible());
    }

    #[test]
    fn generated_pair_bound_accepts_two_items_with_x_separation(
        bin_x in 2_i32..=80,
        bin_y in 1_i32..=40,
        bin_z in 1_i32..=40,
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
        z in 1_i32..=20,
    ) {
        prop_assume!(left_x + right_x <= bin_x);
        prop_assume!(y <= bin_y);
        prop_assume!(z <= bin_z);
        let bin = Bin3 {
            size: AxisBox3::new(r(bin_x), r(bin_y), r(bin_z)).unwrap(),
        };
        let items = [item("left", left_x, y, z), item("right", right_x, y, z)];

        let report = pair_incompatibilities_3d(&bin, &items);

        prop_assert_eq!(report.status, CapacityBoundStatus::Satisfied);
        prop_assert_eq!(report.checked_pairs, 1);
        prop_assert!(report.incompatible_pairs.is_empty());
        prop_assert!(!report.proves_infeasible());
    }

    #[test]
    fn generated_pair_bound_2d_accepts_two_items_with_x_separation(
        bin_x in 2_i32..=80,
        bin_y in 1_i32..=40,
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
    ) {
        prop_assume!(left_x + right_x <= bin_x);
        prop_assume!(y <= bin_y);
        let bin = SheetBin2::new(Rect2::new(r(bin_x), r(bin_y)).unwrap());
        let items = [sheet_item("left", left_x, y), sheet_item("right", right_x, y)];

        let report = pair_incompatibilities_2d(&bin, &items);

        prop_assert_eq!(report.status, CapacityBoundStatus::Satisfied);
        prop_assert_eq!(report.checked_pairs, 1);
        prop_assert!(report.incompatible_pairs.is_empty());
        prop_assert!(!report.proves_infeasible());
    }

    #[test]
    fn generated_no_overlap_model_2d_exports_x_disjunction_for_two_items(
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
    ) {
        let bin = SheetBin2::new(Rect2::new(r(left_x + right_x), r(y)).unwrap());
        let items = [sheet_item("left", left_x, y), sheet_item("right", right_x, y)];

        let report = export_no_overlap_model_2d(&bin, &items);

        prop_assert_eq!(report.status, ModelExportStatus2::Ready);
        prop_assert_eq!(report.domains.len(), 2);
        prop_assert!(report.disjunctions[0]
            .disjuncts
            .contains(&NoOverlapDisjunct2::LeftBeforeRightX));
        prop_assert!(report.disjunctions[0]
            .disjuncts
            .contains(&NoOverlapDisjunct2::RightBeforeLeftX));
    }

    #[test]
    fn generated_no_overlap_model_exports_x_disjunction_for_two_items(
        left_x in 1_i32..=20,
        right_x in 1_i32..=20,
        y in 1_i32..=20,
        z in 1_i32..=20,
    ) {
        let bin = Bin3 {
            size: AxisBox3::new(r(left_x + right_x), r(y), r(z)).unwrap(),
        };
        let items = [item("left", left_x, y, z), item("right", right_x, y, z)];

        let report = export_no_overlap_model_3d(&bin, &items);

        prop_assert_eq!(report.status, ModelExportStatus3::Ready);
        prop_assert_eq!(report.domains.len(), 2);
        prop_assert!(report.disjunctions[0]
            .disjuncts
            .contains(&NoOverlapDisjunct3::LeftBeforeRightX));
        prop_assert!(report.disjunctions[0]
            .disjuncts
            .contains(&NoOverlapDisjunct3::RightBeforeLeftX));
    }
}
