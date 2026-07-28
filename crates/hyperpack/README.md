<h1>
  Hyperpack
  <img src="./doc/hyperpack.png" alt="Hyperpack logo" width="144" align="right">
</h1>

Exact-aware packing models, heuristic proposals, bounded search, and
authoritative feasibility replay for the Hyper ecosystem.

Hyperpack covers one-dimensional stock cutting, rectangular and convex
irregular sheet packing, cuboid packing, cardinal orientations, multi-bin
assignment, clearances, support/load policies, local search, and bounded exact
search. The governing rule is simple: an algorithm proposes coordinates;
replay decides whether those coordinates satisfy the modeled constraints.

This README describes crate version `0.3.0`.

## Primary types

| Type | Role |
| --- | --- |
| `StockBin1`, `StockItem1`, `StockPlacement1` | Exact one-dimensional stock model |
| `SheetBin2`, `SheetItem2`, `SheetPlacement2`, `Rect2` | Fixed-orientation sheet model |
| `OrientedSheetItem2`, `OrientedSheetPlacement2` | Cardinally oriented sheet items |
| `Bin3`, `Item3`, `Placement3`, `AxisBox3` | Exact cuboid model |
| `OrientedItem3`, `OrientedPlacement3`, `Orientation3` | Six cardinal cuboid permutations |
| `BinInstance3`, `MultiBinPlacement3` | Named-bin assignment and cost |
| `IrregularPacking2`, `IrregularSheetItem2` | Convex line-contour sheet packing |
| `FeasibilityStatus`, verification/report types | Feasible, infeasible, or unknown replay evidence |
| `PackingAnalysis3`, `PlacementOrder3` | Reusable demand, grid, lower-bound, and ordering facts |

## Install

```toml
[dependencies]
hyperpack = "0.3.0"
```

There are no default features. `dispatch-trace` forwards Hyperreal’s
exact-dispatch instrumentation.

## Quick start

Every heuristic report contains exact replay. This checked example packs two
fixed-orientation rectangles with MaxRects and verifies the result.

<!-- quickstart:start -->
```rust
use hyperpack::{
    FeasibilityStatus, ItemId, Real, Rect2, SheetBin2, SheetItem2, maxrects_best_short_side_fit_2d,
};

fn main() -> hyperpack::PackResult<()> {
    let bin = SheetBin2::new(Rect2::new(Real::from(10), Real::from(10))?);
    let items = vec![
        SheetItem2::new(ItemId::new("a")?, Rect2::new(Real::from(4), Real::from(4))?),
        SheetItem2::new(ItemId::new("b")?, Rect2::new(Real::from(6), Real::from(3))?),
    ];

    let report = maxrects_best_short_side_fit_2d(&bin, &items)?;
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(report.rejected.is_empty());
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

For placements supplied by another optimizer, construct `StockPlacement1`,
`SheetPlacement2`, or `Placement3` values and call the corresponding
`verify_packing_*` function directly.

## Proposal and replay model

```text
bin + items + policy
         │
 analysis / heuristic / local or bounded search
         │
    placement proposal
         │ exact replay
 containment + overlap + accounting + policy
         │
 feasible / infeasible / unknown
```

`FeasibilityStatus::Unknown` preserves an uncertified comparison, unsupported
constraint, or exhausted proof budget. Objective scores and lower bounds never
turn an unverified proposal into a feasible packing.

## API guide

### Models and authoritative replay

- `ItemId::new`, `Rect2::new`, and `AxisBox3::new` validate identifiers and
  positive exact dimensions.
- `StockBin1::new`, `StockItem1::new`, and `verify_packing_1d` cover stock
  intervals.
- `SheetBin2::new`, `SheetItem2::new`, and `verify_packing_2d` cover exact
  rectangles.
- `OrientedSheetItem2::new`, `Orientation2::apply`, and
  `verify_oriented_packing_2d` cover cardinal sheet rotations.
- `Bin3`, `Item3`, `Placement3`, and `verify_packing_3d` cover fixed-orientation
  cuboids; `PackingVerification3::replay` is the associated constructor.
- `OrientedItem3::new`, `Orientation3::apply`, and
  `verify_oriented_packing_3d` cover six dimension permutations.
- `BinId::new`, `BinInstance3::new`, and
  `verify_multi_bin_packing_3d` check named-bin assignment, uniqueness, capacity,
  and exact cost.

All replay paths reject duplicate item or bin identifiers rather than choosing
one conflicting definition.

### Bounds, objectives, clearance, support, and handoffs

- `capacity_bounds_2d`, `capacity_bounds_3d`,
  `pair_incompatibilities_2d`, and `pair_incompatibilities_3d` provide exact
  necessary infeasibility evidence.
- `analyze_packing_3d` retains demand classes, dimension/grid facts, initial
  free space, and lower bounds; `order_placements_3d` records deterministic
  order evidence.
- `height_objective_3d` and `compare_objectives_3d` provide exact objective
  values and lexicographic comparisons.
- `verify_clearance_2d` and `verify_clearance_3d` enforce positive exact
  kerf/clearance separation.
- `verify_support_3d` supports full-base, area-ratio, and footprint-center
  policies. `verify_direct_stack_load_3d` uses caller-supplied `ItemWeight3`
  and `LoadLimit3` evidence.
- `import_domain_items_3d`, `import_domain_bin_3d`, and
  `summarize_domain_handoffs` keep geometry/manufacturing/physics constraints
  explicit.
- `export_no_overlap_model_2d` and `export_no_overlap_model_3d` retain exact
  placement domains and pairwise axis-separation disjunctions for solver
  adapters.

### Two-dimensional proposals

- Shelf proposals:
  `shelf_next_fit_decreasing_height_2d`,
  `shelf_first_fit_decreasing_height_2d`, and
  `shelf_best_fit_decreasing_height_2d`.
- Skyline proposals: `skyline_bottom_left_2d` and
  `skyline_minimum_waste_2d`.
- MaxRects proposals: best-short-side, best-long-side, best-area,
  bottom-left, and contact-point functions.
- Guillotine proposals: best-area, best-short-side, and best-long-side
  functions.
- `auto_sheet_portfolio_2d` runs a deterministic, budgeted portfolio and
  retains each evaluation and authoritative replay.

Each `SheetHeuristicReport2` retains placed/rejected items, free rectangles,
trace counts, objective, and replay.

### Three-dimensional proposals and search

- Cuboid heuristics include first/best fit by volume, maximum side, or footprint
  area; extreme-point; maximal-space; guillotine best-volume; and LAFF.
- `auto_cuboid_portfolio_3d` evaluates a deterministic budgeted portfolio.
- `local_search_order_3d`, `tabu_search_order_3d`,
  `multistart_order_3d`, `reinsert_unplaced_order_3d`, and `empty_bins_3d`
  expose bounded neighborhood search and reproducible termination reports.
- `branch_and_bound_one_bin_3d` is a limit-bearing fixed-orientation solver for
  small one-bin instances. `ExactSearchStatus3::Unknown` means an item or node
  limit prevented exhaustive proof.

### Irregular convex sheets

- `IrregularPacking2::new` owns a stable inventory and caches one exact
  Hypercurve translation obstacle (no-fit region) per unordered item pair.
- `IrregularPacking2::verify` replays sheet containment, pair contact/overlap,
  and accounting.
- `IrregularPacking2::bottom_left` proposes translations from the sheet corner
  and cached no-fit boundaries, then attaches authoritative replay.

Boundary contact is feasible. The current cache accepts simple convex
line-contour items; concave input reports that decomposition is required, and
native curved contours are unsupported on this surface.

### Snapshots

- `snapshot_stock_1d_text`, `snapshot_sheet_2d_text`, and
  `snapshot_packing_3d_text` preserve exact rational/Hyperreal structure.
- The corresponding `*_binary` functions use length-prefixed UTF-8 fields,
  not primitive-float coordinate encodings.

## Guarantees and boundaries

- Dimensions, positions, lengths, areas, volumes, costs, weights, and bounds
  use `hyperreal::Real`.
- Containment, non-overlap, support, load, and objective decisions use
  certified signs or return `Unknown`; no epsilon is introduced.
- A heuristic replay can prove feasibility, but a good objective does not prove
  global optimality.
- Orientations are cardinal permutations, not arbitrary rotations.
- Footprint-center support is geometric and does not infer a physical mass
  distribution.
- Support/load reports omit friction, deformation, dynamics, and transitive
  load propagation.
- Routing, manufacturing-process, and richer physical rules require explicit
  domain handoffs.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Hyperreal exact-dispatch instrumentation |

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --all-features --locked
cargo clippy --all-targets --all-features --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features --locked
cargo check --benches --all-features
```

Benchmark definitions, baselines, retained optimizations, and rejected
experiments are in [PERFORMANCE.md](PERFORMANCE.md). Fuzz ownership and replay
instructions live in [fuzz/README.md](fuzz/README.md).

## References

These sources describe the cutting/packing taxonomy, proposal algorithms,
bounded search, and exact-replay boundary relevant to Hyperpack:

- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).
- Dyckhoff, H. “A Typology of Cutting and Packing Problems.”
  *European Journal of Operational Research* 44(2), 1990.
  [DOI: 10.1016/0377-2217(90)90350-K](https://doi.org/10.1016/0377-2217(90)90350-K).
- Martello, S., Pisinger, D., and Vigo, D. “The Three-Dimensional Bin Packing
  Problem.” *Operations Research* 48(2), 2000.
  [DOI: 10.1287/opre.48.2.256.12386](https://doi.org/10.1287/opre.48.2.256.12386).
- Lodi, A., Martello, S., and Vigo, D. “Heuristic Algorithms for the
  Three-Dimensional Bin Packing Problem.” *European Journal of Operational
  Research* 141(2), 2002.
  [DOI: 10.1016/S0377-2217(02)00134-0](https://doi.org/10.1016/S0377-2217(02)00134-0).
- Crainic, T. G., Perboli, G., and Tadei, R. “Extreme Point-Based Heuristics
  for Three-Dimensional Bin Packing.” *INFORMS Journal on Computing* 20(3),
  2008. [DOI: 10.1287/ijoc.1070.0250](https://doi.org/10.1287/ijoc.1070.0250).
- Jylänki, J. “A Thousand Ways to Pack the Bin—A Practical Approach to
  Two-Dimensional Rectangle Bin Packing.” 2010.
  [Report](https://trszdev.github.io/maxrects-bssf-global-demo/RectangleBinPack.pdf).
- Iori, M., de Lima, V. L., Martello, S., Miyazawa, F. K., and Monaci, M.
  “Exact Solution Techniques for Two-Dimensional Cutting and Packing.”
  *European Journal of Operational Research* 289(2), 2021.
  [DOI: 10.1016/j.ejor.2020.06.050](https://doi.org/10.1016/j.ejor.2020.06.050).
- Bortfeldt, A., and Wäscher, G. “Constraints in Container Loading—A
  State-of-the-Art Review.” *European Journal of Operational Research* 229(1),
  2013. [DOI: 10.1016/j.ejor.2012.12.006](https://doi.org/10.1016/j.ejor.2012.12.006).
- Glover, F. “Tabu Search—Part I.” *ORSA Journal on Computing* 1(3), 1989.
  [DOI: 10.1287/ijoc.1.3.190](https://doi.org/10.1287/ijoc.1.3.190).
- Wolpert, D. H., and Macready, W. G. “No Free Lunch Theorems for
  Optimization.” *IEEE Transactions on Evolutionary Computation* 1(1), 1997.
  [DOI: 10.1109/4235.585893](https://doi.org/10.1109/4235.585893).
- Hoos, H. H., and Stützle, T. *Stochastic Local Search: Foundations and
  Applications*. Morgan Kaufmann, 2004.
  [Companion site](https://www.cs.ubc.ca/~hoos/SLS-Book/).

## Acknowledgements

Hyperpack builds on
[Hyperreal](https://github.com/timschmidt/hyperreal) and
[Hypercurve](https://github.com/timschmidt/hypercurve). Related constraint,
physical, manufacturing, and search semantics remain in Hypersolve,
Hyperphysics, Hyperpath, Hyperdrc, and Hyperevolution. The research cited above
informs algorithms and report boundaries without implying source-code
derivation.

## License and contributing

Licensed under Apache-2.0 as declared in [Cargo.toml](Cargo.toml).

Bug reports should include exact bins/items/placements, policy inputs, enabled
features, and full replay. Before proposing a change, run formatting, the
focused regression, all-feature tests, and strict Clippy.
