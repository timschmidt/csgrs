# Changelog

## [0.3.0] - Unreleased

### Breaking

- `FeasibilityReplay3::replay` returns `PackError::DuplicateItem` when two
  declared items share an id. It previously kept one of the conflicting
  definitions.
- `PackError` gains `InvalidSupportRatio`, `NegativeLoadValue`,
  `NegativeClearance`, `DuplicateItem`, `DuplicateBin`, `DuplicateItemWeight`,
  `DuplicateLoadLimit`, and `MissingBin`. The enum is exhaustive, so `match`
  expressions over it must handle the new variants.

### Added

- Exact verification beyond single-bin 3D replay: `verify_packing_1d` for
  stock cutting, `verify_packing_2d` and `verify_oriented_packing_2d` for
  rectangular sheets, `verify_packing_3d` and `verify_oriented_packing_3d` for
  cuboids, and `verify_multi_bin_packing_3d` for named bins with capacity and
  cost. Each reports feasible, infeasible, or unknown, plus unplaced and
  duplicate items.
- Heuristic proposals that attach exact replay: shelf, skyline, MaxRects, and
  guillotine for sheets; first/best fit, extreme-point, maximal-space,
  guillotine, and LAFF for cuboids; and the budgeted portfolios
  `auto_sheet_portfolio_2d` and `auto_cuboid_portfolio_3d`.
- Bounded search: `local_search_order_3d`, `tabu_search_order_3d`,
  `multistart_order_3d`, `reinsert_unplaced_order_3d`, `empty_bins_3d`, and the
  limit-bearing `branch_and_bound_one_bin_3d` for small one-bin instances.
- `IrregularPacking2` for convex line-contour sheet items, with cached
  Hypercurve no-fit regions, exact verification, and a bottom-left proposal.
  Concave input reports that decomposition is required.
- Necessary infeasibility and analysis evidence (`capacity_bounds_2d`,
  `capacity_bounds_3d`, `pair_incompatibilities_2d`,
  `pair_incompatibilities_3d`, `analyze_packing_3d`, `order_placements_3d`),
  exact height objectives, clearance checks (`verify_clearance_2d`,
  `verify_clearance_3d`), and support and stack-load checks
  (`verify_support_3d`, `verify_direct_stack_load_3d`).
- Domain handoffs (`import_domain_items_3d`, `import_domain_bin_3d`), solver
  model export (`export_no_overlap_model_2d`, `export_no_overlap_model_3d`),
  and exact text and binary snapshots for 1D, 2D, and 3D packings.
- `dispatch-trace` feature, which forwards the exact-dispatch instrumentation
  of `hyperreal` and `hyperlimit`.

### Changed

- Dimension, containment, and no-overlap checks use `hyperlimit`'s strict
  predicate policy (`PredicatePolicy::STRICT`) instead of
  `Real::refine_sign_until(-64)`. A certified violation or separation now
  decides a placement even when another of its comparisons is undecided;
  previously the first undecided comparison made the result unknown.
- Feasibility replay resolves each placement's item once instead of twice per
  placement pair. `PERFORMANCE.md` records local medians of 1.220 ms before
  and 0.855 ms after this change for a 100-placement 3D replay.
- New dependencies on `hypercurve` and `hyperlimit`; `hyperreal` is used with
  its `serde` feature.
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.2.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names (the previous release named none).

For 0.2.0 and earlier, see [crates.io](https://crates.io/crates/hyperpack/versions).
