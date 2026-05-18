# hyperpack

`hyperpack` owns exact-aware packing models and feasibility replay for the Hyper
ecosystem. It records items, bins, placements, free-space summaries, lower-bound
evidence, deterministic seeds, and heuristic proposal reports over `hyperreal::Real`
dimensions.

The crate does not try to be a full optimizer yet. It gives heuristic and future exact
packing algorithms a shared evidence boundary: proposals are separate from the exact
checks that decide whether a placement is usable.

## Hyper Ecosystem

`hyperpack` is a proposal-plus-replay domain crate.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact dimensions, positions,
  volumes, and lower-bound values.
- [hyperparts](https://github.com/timschmidt/hyperparts): part, package, and process
  facts that can generate packable items.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): mass, material, support,
  and center-of-mass constraints.
- [hyperpath](https://github.com/timschmidt/hyperpath) and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): routing or manufacturing checks
  that can become domain handoff constraints.
- [hypersolve](https://github.com/timschmidt/hypersolve): future exact/solver backend
  for small optimality and feasibility fixtures.

## Typical Packing Problems

Packing software often conflates a heuristic placement with proof of feasibility. Shelf,
skyline, MaxRects, guillotine, extreme-point, DBLF, layer, and LAFF variants can be fast,
but they commonly use rounded dimensions, order-dependent shortcuts, and partial
collision checks. A result that looks good can still violate containment, overlap,
support, weight, routing, or process rules.

`hyperpack` keeps those concerns separate. Heuristics propose placements; exact replay
checks the constraints it knows; unsupported constraints remain unknown until the crate
has a certified handoff for them.

## Main Types

- `ItemId`, `Item3`, `Bin3`, `AxisBox3`, `ContainerFrame3`, and `Placement3` describe
  axis-aligned packing inputs and placements.
- `StockBin1`, `StockItem1`, `StockPlacement1`, and `verify_packing_1d` describe and
  verify exact 1D stock-packing intervals.
- `Rect2`, `SheetBin2`, `SheetItem2`, `SheetPlacement2`, and `verify_packing_2d`
  describe and verify exact fixed-orientation 2D sheet-packing rectangles.
- `Orientation2`, `OrientedSheetItem2`, `OrientedSheetPlacement2`, and
  `verify_oriented_packing_2d` add exact cardinal orientation-policy replay for 2D
  rectangles.
- `HeuristicFamily` records which proposal family produced or motivated a report.
- `PackingReport3`, `FreeSpaceReport3`, and `LowerBoundReport` preserve proposal and
  bound evidence.
- `FeasibilityReplay3`, `PackingVerification3`, `ObjectiveReport3`, and
  `FeasibilityStatus` distinguish feasible, infeasible, unknown, duplicate, unplaced,
  and exact waste/accounting outcomes.
- `BinId`, `BinInstance3`, `MultiBinPlacement3`, and
  `verify_multi_bin_packing_3d` replay named-bin assignments and aggregate exact
  bin-count, cost, used-volume, and waste evidence.
- `capacity_bounds_3d`, `CapacityBoundReport3`, and `CapacityBoundStatus` expose exact
  necessary one-bin lower bounds for total volume and maximum item dimensions.
- `capacity_bounds_2d` and `CapacityBoundReport2` expose matching exact one-sheet
  lower bounds for total area and maximum rectangle dimensions.
- `pair_incompatibilities_3d` reports item pairs that cannot be separated along any
  axis of the current bin.
- `pair_incompatibilities_2d` reports fixed-orientation rectangle pairs that cannot
  be separated along either sheet axis.
- `Orientation3`, `OrientedItem3`, `OrientedPlacement3`, and
  `verify_oriented_packing_3d` add exact six-permutation orientation-policy replay for
  3D cuboids.
- `snapshot_stock_1d_text`, `snapshot_sheet_2d_text`, `snapshot_packing_3d_text`,
  and their binary counterparts emit deterministic fixtures with exact scalar text
  fields rather than primitive floats.
- `shelf_next_fit_decreasing_height_2d`, `shelf_first_fit_decreasing_height_2d`, and
  `shelf_best_fit_decreasing_height_2d` emit classical shelf proposals with
  `PlacementCandidate2`, `FreeRect2`, trace counters, rejected ids, and exact replay.
- `skyline_bottom_left_2d` and `skyline_minimum_waste_2d` emit skyline-style proposals
  by scanning exact candidate edge points, again with exact replay before acceptance.
- `maxrects_best_short_side_fit_2d`, `maxrects_best_long_side_fit_2d`,
  `maxrects_best_area_fit_2d`, `maxrects_bottom_left_2d`, and
  `maxrects_contact_point_2d` emit MaxRects-style free-rectangle proposals using
  exact scoring and exact replay before acceptance.
- `guillotine_best_area_fit_2d`, `guillotine_best_short_side_fit_2d`, and
  `guillotine_best_long_side_fit_2d` emit guillotine-split free-rectangle proposals
  with exact scoring and exact replay before acceptance.
- `auto_sheet_portfolio_2d`, `SheetPortfolioBudget2`, and
  `SheetPortfolioReport2` run a deterministic 2D heuristic portfolio and rank
  candidate reports by exact replay status and exact objective values.
- `cuboid_first_fit_decreasing_volume_3d` and
  `cuboid_best_fit_decreasing_volume_3d`, plus the matching max-side and
  footprint-area order variants, emit exact-aware 3D corner-point proposals with
  replay-backed acceptance.
- `cuboid_extreme_point_decreasing_volume_3d` emits an exact
  deepest-bottom-left extreme-point proposal with replay-backed acceptance.
- `cuboid_maximal_space_decreasing_volume_3d` emits a conservative exact
  origin-bearing free-box proposal with replay-backed acceptance.
- `cuboid_guillotine_best_volume_fit_3d` emits a conservative exact 3D
  guillotine/free-box proposal with replay-backed acceptance.
- `cuboid_laff_largest_area_fit_first_3d` emits a LAFF-style broad-footprint,
  low-layer proposal with replay-backed acceptance.
- `auto_cuboid_portfolio_3d`, `CuboidPortfolioBudget3`, and
  `CuboidPortfolioReport3` run a deterministic 3D cuboid portfolio and rank
  candidate reports by exact replay status and exact objective values.
- `verify_support_3d`, `SupportPolicy3`, and `SupportReport3` replay exact
  support evidence for no-support, full-base, center-projection, and
  support-area-ratio policies.
- `verify_direct_stack_load_3d`, `ItemWeight3`, and `LoadLimit3` replay exact
  direct top-face stack-load evidence while leaving richer physical load
  transfer to `hyperphysics`.
- `verify_clearance_3d`, `ClearanceReport3`, and `ClearanceStatus3` replay
  exact pairwise gap policies separately from ordinary contact-allowing
  no-overlap.
- `verify_clearance_2d`, `ClearanceReport2`, and `ClearanceStatus2` replay
  exact sheet kerf/clearance policies separately from ordinary rectangle
  contact.
- `branch_and_bound_one_bin_3d`, `ExactSearchLimit3`, and `ExactSearchReport3`
  provide a bounded exact backend for small fixed-orientation 3D instances.
- `export_no_overlap_model_3d`, `PlacementDomain3`, and
  `PairNoOverlapDisjunction3` expose exact CP/MIP-style origin domains and
  no-overlap disjunctions for solver adapters.
- `export_no_overlap_model_2d`, `PlacementDomain2`, and
  `PairNoOverlapDisjunction2` expose the same exact solver-adapter boundary for
  one-sheet rectangular no-overlap models.
- `local_search_order_3d`, `LocalSearchConfig3`, and `LocalSearchReport3`
  schedule deterministic swap/insert/reverse item-order moves with exact replay
  after every proposal.
- `multistart_order_3d`, `MultistartConfig3`, and `MultistartReport3` schedule
  deterministic seeded item-order starts and rank them by exact replayed objective
  data.
- `reinsert_unplaced_order_3d`, `ReinsertUnplacedConfig3`, and
  `ReinsertUnplacedReport3` try deterministic unplaced-item repair moves with
  exact replay after every candidate reinsertion.
- `empty_bins_3d`, `BinEmptyingConfig3`, and `BinEmptyingReport3` try
  deterministic multi-bin bin-emptying repair moves with exact replay after
  every candidate reassignment.
- `height_objective_3d` and `HeightObjective3` report exact used/remaining height
  for proposed 3D placements without replacing feasibility replay.
- `compare_objectives_3d`, `ObjectiveTerm3`, and `ObjectiveComparison3` compare
  replay and height reports through explicit exact lexicographic policies.
- `prepare_placements_3d` and `replay_prepared_packing_3d` canonicalize
  placement batches for deterministic replay without creating a separate
  feasibility path.
- `prepare_packing_3d`, `PreparedPacking3`, and related prepared reports cache
  exact demand classes, scalar grid facts, lower bounds, and initial free-space
  state for proposal engines.
- `DomainBoxFact3`, `DomainConstraintHandoff`, `import_domain_items_3d`, and
  `summarize_domain_handoffs` preserve Hyper ecosystem provenance, lossy status,
  and explicit unknowns at domain boundaries.
- `PackError` and `PackResult` keep construction and replay failures typed.

## Precision Model

Dimensions, positions, lengths, areas, volumes, and bounds are `Real` values. 1D interval
containment/no-overlap, 2D rectangle containment/no-overlap, cardinal 2D orientation by
dimension permutation, 3D cuboid containment/no-overlap, six-permutation 3D orientation,
one-placement-per-item accounting, waste/used replay, total-volume capacity,
maximum-dimension capacity, and pair-incompatibility bounds are exact for the models
currently implemented. The crate does not silently round dimensions to primitive floats
to make a heuristic fit.
The 2D lower-bound surface mirrors the 3D one: area overflow, over-wide/over-tall
rectangles, and pairs with no certified separating sheet axis are explicit necessary
certificates, not hidden heuristic conclusions.

Support replay uses exact footprint/contact areas. Full-base support compares exact
supported area against exact footprint area; center projection checks the cuboid
footprint center against exact contact patches; ratio support cross-multiplies exact
areas against integer policy bounds to avoid division or primitive floats.
Direct stack-load replay sums supplied exact weights for items resting on each exact
top-face contact patch and compares them with supplied exact load limits. Missing
weights or limits are reported as unknown rather than inferred.
Clearance replay checks exact separating-axis gaps. Ordinary no-overlap may allow
contact; a positive 2D kerf or 3D clearance policy rejects exact contact without
introducing an epsilon tolerance.
Multi-bin replay groups placements by exact named bin, reuses one-bin replay for
geometry, and aggregates exact bin cost and waste. Duplicate item assignments across
bins are infeasible even when each individual bin is geometrically valid.

Future support, load, route, center-of-mass, and process checks should keep the same
pattern: exact inputs where possible, explicit adapter reports where not, and unknown
when the constraint has not been certified.

Fixture snapshots use line-oriented text with escaped ids or binary length-prefixed
UTF-8 fields with raw ids. Rational `Real` values are emitted as rational text;
non-rational structural values stay inside escaped `hyperreal` JSON strings rather
than being rounded to `f64`.

## Performance Model

`hyperpack` keeps the current replay surface intentionally small. Fast heuristic
generation can remain domain-specific, while replay uses simple axis-aligned interval
checks and grouped evidence records. Lower-bound and free-space reports are separated
from placement certification so future algorithms can compare proposal quality without
rerunning every exact check.

Performance should improve by narrowing candidate pairs, retaining free-space
structure, and dispatching specialized exact checks before invoking generic solver or
search machinery.
The 3D maximal-space path keeps free boxes as exact scheduling evidence and partitions
only the selected box, so it avoids scalar tolerance decisions and numerical explosion
from speculative global collision state. Missing a useful free box is a heuristic
quality issue; accepting an invalid placement is prevented by exact replay.
The 3D guillotine path exposes the same exact split state as a cut-oriented proposal
family, making manufacturing-style constraints visible without treating the cut tree as
proof of feasibility.

The bounded exact backend is intentionally limited by item count and node count. It
prefilters with exact capacity bounds, branches over exact candidate points, and returns
`Unknown` rather than pretending an interrupted search proves infeasibility.
The no-overlap model export is likewise an adapter boundary: it preserves exact origin
domains and exact pairwise separation disjunctions, but any external solver assignment
must come back through exact replay before it becomes a trusted packing.
The 2D export follows the same adapter boundary for sheets, preserving exact
rectangle-origin domains and x/y separation disjunctions without lowering to `f64`.
The local-search scheduler is likewise limit-bearing: it treats order moves as proposal
generation, compares only exact replayed objectives, and reports step or neighbor
limits explicitly.
Seeded multistart uses deterministic pseudo-random order proposals only to vary search
routes; every seed returns an exact replay report and the best seed is selected by exact
objective comparison.
Reinsert-unplaced repair follows the same boundary for repair moves: it changes item
order only as a proposal, then promotes the move only when exact replay improves the
objective report.
Bin-emptying repair applies that boundary to multi-bin assignments: it tries to
relocate all items from one used bin into the remaining bins and accepts the candidate
only when exact multi-bin replay improves bin count, cost, and assignment evidence.
Height objectives scan exact placement upper `z` coordinates and return unknown
comparison evidence instead of introducing a display tolerance or `f64` height key.
Lexicographic objective comparison keeps policy visible: exact integer counts compare
directly, exact `Real` volumes/heights compare by sign certification, and missing or
uncertified height evidence is reported as unknown.
Domain handoffs follow the same rule: exact facts may become exact carriers, while
conservative, lossy, and unknown facts remain report evidence and never certify
geometric feasibility by implication.
Prepared placement batches are normalization evidence only: certified coordinate
ordering is recorded, unknown ordering remains explicit, and exact replay still owns
all acceptance decisions.
Prepared packing problems apply the same rule to inputs: demand classes, exact
rational grid summaries, lower-bound reports, and initial free boxes are cached so
heuristics can avoid repeatedly expanding the same scalar structure. Those caches are
performance evidence, not feasibility certificates; layouts still replay through exact
containment and no-overlap reports.
The 2D portfolio runner keeps the same boundary: it can spend a budget across shelf,
skyline, MaxRects, and guillotine proposal engines, but it compares only replayed
reports. Exact feasible layouts beat unknown/infeasible layouts, and exact used area
and unplaced counts decide quality before deterministic algorithm order breaks ties.
The 3D portfolio runner applies the same replay-ranked policy to corner-point,
extreme-point, maximal-space, and guillotine cuboid proposals, avoiding a hidden
"default best" heuristic while keeping every candidate auditable.

## Current Status

Implemented today:

- exact 1D stock item/bin/placement carriers and quantity-one verification reports;
- exact 2D sheet item/bin/placement carriers and quantity-one verification reports;
- exact 2D cardinal orientation-policy validation and replay;
- exact axis-aligned 3D item, bin, container, and placement carriers;
- exact 3D six-permutation orientation-policy validation and replay;
- deterministic exact text and binary snapshots for 1D, 2D, and 3D fixtures;
- exact-aware 2D NFDH, FFDH, and BFDH shelf heuristic proposal paths with
  replay-backed reporting;
- exact-aware 2D bottom-left and minimum-waste skyline proposal paths with
  replay-backed reporting;
- exact-aware 2D MaxRects BSSF, BLSF, BAF, bottom-left, and contact-point
  proposal paths with replay-backed reporting;
- exact-aware 2D guillotine best-area, best-short-side, and best-long-side proposal
  paths with replay-backed reporting;
- deterministic 2D sheet heuristic portfolio scheduling across shelf, skyline,
  MaxRects, and guillotine proposals with exact replay-ranked selection;
- exact-aware 3D first-fit and best-fit decreasing-volume, decreasing-max-side,
  and decreasing-footprint-area corner-point proposal paths with replay-backed
  reporting;
- exact-aware 3D decreasing-volume extreme-point/DBLF proposal path with
  replay-backed reporting;
- exact-aware 3D decreasing-volume maximal-space/free-box proposal path with
  replay-backed reporting;
- exact-aware 3D guillotine best-volume-fit proposal path with replay-backed
  reporting;
- exact-aware 3D LAFF-style largest-area-fit-first proposal path with
  replay-backed reporting;
- deterministic 3D cuboid heuristic portfolio scheduling across corner-point,
  extreme-point, maximal-space, guillotine, and LAFF proposals with exact
  replay-ranked selection;
- exact support replay reports for no-support, full-base support,
  center-projection, and exact support-area-ratio policies;
- exact direct stack-load reports with supplied item weights, supplied max-load
  limits, and explicit missing-evidence states;
- exact 3D clearance replay with contact-vs-positive-gap distinctions and
  negative-clearance rejection;
- exact 2D sheet kerf/clearance replay with contact-vs-positive-gap
  distinctions and negative-clearance rejection;
- exact multi-bin 3D assignment replay with named bins, exact bin costs,
  aggregate used/waste volume, duplicate assignment detection, and missing-bin
  rejection;
- bounded exact small-instance 3D branch-and-bound with exact capacity prefilters,
  node/item limits, and exact replayed incumbents;
- exact CP/MIP-style one-bin 3D no-overlap model export with origin domains,
  pairwise disjunctions, and explicit infeasible/unknown evidence;
- exact CP/MIP-style one-sheet 2D no-overlap model export with origin domains,
  pairwise disjunctions, and explicit infeasible/unknown evidence;
- deterministic 3D item-order local search with swap, insert, and reverse moves,
  exact replayed objectives, and explicit step/neighbor-limit statuses;
- deterministic seeded 3D item-order multistart with exact replayed objective ranking;
- deterministic 3D unplaced-item reinsertion repair with exact replayed objective
  ranking and explicit pass/trial-limit statuses;
- deterministic multi-bin bin-emptying repair with exact replayed cost/bin-count
  ranking and explicit pass/bin-limit statuses;
- exact 3D used-height objective reports with missing-item validation;
- exact lexicographic 3D objective comparison over replay and height evidence;
- prepared 3D problem summaries and placement batches with exact demand classes,
  grid/cache metadata, deterministic canonical order, and exact replay agreement
  checks;
- report-bearing Hyper ecosystem domain handoffs with exact item/bin imports,
  lossy/conservative/unknown fact accounting, and non-owned constraint summaries;
- heuristic-family, packing, free-space, and lower-bound reports;
- exact containment and pairwise no-overlap replay;
- exact one-bin verification reports with placed/unplaced/duplicate item accounting and
  exact used/waste volume;
- exact necessary one-bin capacity bound reports for volume overflow and item dimensions;
- exact necessary one-sheet capacity bound reports for area overflow and item dimensions;
- exact pair-incompatibility bound reports for cuboid pairs with no separating axis;
- exact pair-incompatibility bound reports for rectangle pairs with no separating axis;
- feasible, infeasible, and unknown replay states.

Known limits: center-of-mass checks, load propagation, routing clearance, process
policy, and optimality certificates are still future work.

## Installation

```toml
[dependencies]
hyperpack = "0.2.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperpack = { path = "../hyperpack" }
```

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench feasibility
```
