//! Exact-aware packing carriers and feasibility replay.
//!
//! `hyperpack` owns item/bin/sheet/container models, placements, free-space
//! reports, heuristic proposal metadata, exact lower-bound evidence, and
//! feasibility replay. Heuristics such as shelf, skyline, MaxRects, guillotine,
//! extreme-point, DBLF, layer, and LAFF are proposal surfaces until their output
//! is replayed exactly.
//!
//! Combinatorial acceptance is based on exact containment, no-overlap, and
//! support checks or explicit unknowns.

pub mod analysis;
pub mod bounds;
pub mod clearance;
pub mod domain;
pub mod error;
pub mod heuristic2d;
pub mod heuristic3d;
pub mod irregular2d;
pub mod model;
pub mod model_export;
pub mod multibin;
pub mod objective;
pub mod orientation;
pub mod portfolio;
pub mod replay;
pub mod search;
pub mod sheet;
pub mod snapshot;
pub mod solver;
pub mod stock;
pub mod support;

pub use analysis::{
    DemandClass3, PackingAnalysis3, PackingAnalysisMetadata3, PackingDimensionFacts3,
    PackingGridFacts3, PlacementOrder3, analyze_packing_3d, order_placements_3d,
};
pub use bounds::{
    CapacityBoundReport2, CapacityBoundReport3, CapacityBoundStatus, PairIncompatibility2,
    PairIncompatibility3, PairIncompatibilityReport2, PairIncompatibilityReport3,
    capacity_bounds_2d, capacity_bounds_3d, pair_incompatibilities_2d, pair_incompatibilities_3d,
};
pub use clearance::{
    ClearancePairEvidence2, ClearancePairEvidence3, ClearanceReport2, ClearanceReport3,
    ClearanceStatus2, ClearanceStatus3, verify_clearance_2d, verify_clearance_3d,
};
pub use domain::{
    DomainBoxFact3, DomainConstraintHandoff, DomainCrate, DomainFactStatus, DomainHandoffReport,
    DomainHandoffStatus, DomainImportReport3, import_domain_bin_3d, import_domain_items_3d,
    summarize_domain_handoffs,
};
pub use error::{PackError, PackResult};
pub use heuristic2d::{
    FreeRect2, PlacementCandidate2, SheetHeuristic2, SheetHeuristicReport2, SheetHeuristicTrace2,
    guillotine_best_area_fit_2d, guillotine_best_long_side_fit_2d,
    guillotine_best_short_side_fit_2d, maxrects_best_area_fit_2d, maxrects_best_long_side_fit_2d,
    maxrects_best_short_side_fit_2d, maxrects_bottom_left_2d, maxrects_contact_point_2d,
    shelf_best_fit_decreasing_height_2d, shelf_first_fit_decreasing_height_2d,
    shelf_next_fit_decreasing_height_2d, skyline_bottom_left_2d, skyline_minimum_waste_2d,
};
pub use heuristic3d::{
    CandidatePoint3, CuboidHeuristic3, CuboidHeuristicReport3, CuboidHeuristicTrace3, FreeBox3,
    PlacementCandidate3, cuboid_best_fit_decreasing_footprint_area_3d,
    cuboid_best_fit_decreasing_max_side_3d, cuboid_best_fit_decreasing_volume_3d,
    cuboid_extreme_point_decreasing_volume_3d, cuboid_first_fit_decreasing_footprint_area_3d,
    cuboid_first_fit_decreasing_max_side_3d, cuboid_first_fit_decreasing_volume_3d,
    cuboid_guillotine_best_volume_fit_3d, cuboid_laff_largest_area_fit_first_3d,
    cuboid_maximal_space_decreasing_volume_3d,
};
pub use hyperreal::Real;
pub use irregular2d::{
    IrregularBottomLeftReport2, IrregularPackError2, IrregularPackResult2, IrregularSheetItem2,
    IrregularSheetObjective2, IrregularSheetPlacement2, IrregularSheetVerification2,
    PreparedIrregularPacking2, PreparedNoFitPair2, bottom_left_irregular_2d,
    prepare_irregular_packing_2d, verify_irregular_packing_2d,
};
pub use model::{
    AxisBox3, Bin3, ContainerFrame3, FreeSpaceReport3, HeuristicFamily, Item3, ItemId,
    LowerBoundReport, PackingReport3, Placement3,
};
pub use model_export::{
    ModelExportStatus2, ModelExportStatus3, NoOverlapDisjunct2, NoOverlapDisjunct3,
    NoOverlapModelReport2, NoOverlapModelReport3, PairNoOverlapDisjunction2,
    PairNoOverlapDisjunction3, PlacementDomain2, PlacementDomain3, export_no_overlap_model_2d,
    export_no_overlap_model_3d,
};
pub use multibin::{
    BinId, BinInstance3, BinReplay3, MultiBinObjective3, MultiBinPlacement3, MultiBinVerification3,
    verify_multi_bin_packing_3d,
};
pub use objective::{
    HeightObjective3, ObjectiveComparison3, ObjectiveTerm3, compare_objectives_3d,
    height_objective_3d,
};
pub use orientation::{
    Orientation3, OrientationValidationReport3, OrientedItem3, OrientedPackingVerification3,
    OrientedPlacement3, verify_oriented_packing_3d,
};
pub use portfolio::{
    CuboidPortfolioBudget3, CuboidPortfolioReport3, CuboidPortfolioStatus3, SheetPortfolioBudget2,
    SheetPortfolioReport2, SheetPortfolioStatus2, auto_cuboid_portfolio_3d,
    auto_sheet_portfolio_2d,
};
pub use replay::{
    FeasibilityReplay3, FeasibilityStatus, ObjectiveReport3, PackingVerification3,
    verify_packing_3d,
};
pub use search::{
    BinEmptyingConfig3, BinEmptyingMove3, BinEmptyingReport3, BinEmptyingStatus3,
    LocalSearchConfig3, LocalSearchReport3, LocalSearchStatus3, MultiBinEvaluation3,
    MultistartConfig3, MultistartReport3, MultistartStatus3, OrderEvaluation3, OrderMove3,
    ReinsertMove3, ReinsertUnplacedConfig3, ReinsertUnplacedReport3, ReinsertUnplacedStatus3,
    SeededOrderEvaluation3, TabuSearchConfig3, TabuSearchReport3, TabuSearchStatus3, empty_bins_3d,
    local_search_order_3d, multistart_order_3d, reinsert_unplaced_order_3d, tabu_search_order_3d,
};
pub use sheet::{
    Orientation2, OrientationValidationReport2, OrientedSheetItem2, OrientedSheetPlacement2,
    OrientedSheetVerification2, Rect2, SheetBin2, SheetItem2, SheetObjective2, SheetPlacement2,
    SheetVerification2, verify_oriented_packing_2d, verify_packing_2d,
};
pub use snapshot::{
    snapshot_packing_3d_binary, snapshot_packing_3d_text, snapshot_sheet_2d_binary,
    snapshot_sheet_2d_text, snapshot_stock_1d_binary, snapshot_stock_1d_text,
};
pub use solver::{
    ExactSearchLimit3, ExactSearchReport3, ExactSearchStatus3, branch_and_bound_one_bin_3d,
};
pub use stock::{
    StockBin1, StockItem1, StockObjective1, StockPlacement1, StockVerification1, verify_packing_1d,
};
pub use support::{
    ItemWeight3, LoadEvidence3, LoadLimit3, LoadReport3, SupportEvidence3, SupportPolicy3,
    SupportReport3, SupportStatus3, verify_direct_stack_load_3d, verify_support_3d,
};
