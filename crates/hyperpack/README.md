<h1>
  hyperpack
  <img src="./doc/hyperpack.png" alt="hyperpack logo" width="144" align="right">
</h1>

`hyperpack` provides exact-aware packing models, proposal algorithms, and
feasibility replay over [`hyperreal::Real`](https://github.com/timschmidt/hyperreal).
It covers stock cutting, rectangular and convex-irregular sheet packing, cuboid
packing, cardinal orientations, multi-bin assignment, constraints, local
search, and bounded exact search.

The central rule is simple: a heuristic proposes coordinates; replay decides
whether those coordinates satisfy the modeled constraints. Unsupported or
uncertified constraints remain explicit instead of being rounded into success.

## Installation

```toml
[dependencies]
hyperpack = "0.3.0"
```

For a sibling checkout:

```toml
[dependencies]
hyperpack = { path = "../hyperpack" }
```

## Quick Start

Every proposal report includes its exact replay. This example packs two
fixed-orientation rectangles with MaxRects and checks the result:

```rust
use hyperpack::{
    FeasibilityStatus, ItemId, Real, Rect2, SheetBin2, SheetItem2,
    maxrects_best_short_side_fit_2d,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bin = SheetBin2::new(Rect2::new(Real::from(10), Real::from(10))?);
    let items = vec![
        SheetItem2::new(
            ItemId::new("a")?,
            Rect2::new(Real::from(4), Real::from(4))?,
        ),
        SheetItem2::new(
            ItemId::new("b")?,
            Rect2::new(Real::from(6), Real::from(3))?,
        ),
    ];

    let report = maxrects_best_short_side_fit_2d(&bin, &items)?;
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(report.rejected.is_empty());
    Ok(())
}
```

For coordinates supplied by another algorithm, construct `StockPlacement1`,
`SheetPlacement2`, or `Placement3` values and call `verify_packing_1d`,
`verify_packing_2d`, or `verify_packing_3d` directly.

## Model and Reports

The principal input types are:

- `StockBin1` and `StockItem1` for exact intervals;
- `SheetBin2`, `SheetItem2`, and `Rect2` for exact rectangles;
- `Bin3`, `Item3`, and `AxisBox3` for exact axis-aligned cuboids;
- `OrientedSheetItem2` and `OrientedItem3` for allowed cardinal dimension
  permutations; and
- `BinInstance3` and `MultiBinPlacement3` for named-bin assignment and cost.

`FeasibilityStatus` distinguishes `Feasible`, `Infeasible`, and `Unknown`.
Verification reports retain exact objective values, item accounting, check
counts, and human-readable evidence. Orientation, clearance, support, load,
multi-bin, and domain handoff checks have separate reports so a geometric pass
cannot accidentally imply that an unmodeled policy also passed.
Declared item identifiers must be unique; replay returns
`PackError::DuplicateItem` instead of silently choosing one of two conflicting
item definitions. Multi-bin and direct-load replay apply the same rule to bin
IDs, weight evidence, and load limits.

## Proposal and Search Algorithms

The proposal surface includes:

- 2D shelf (NFDH, FFDH, BFDH), skyline, MaxRects, and guillotine variants;
- 3D corner-point, extreme-point, maximal-space, guillotine, and LAFF variants;
- deterministic portfolios, order local search, tabu search, seeded multistart,
  reinsertion repair, and bin-emptying repair; and
- `branch_and_bound_one_bin_3d`, a limit-bearing fixed-orientation solver for
  small one-bin instances.

Proposal reports include trace counters, rejected items, retained free-space
state, and exact replay. `PackingAnalysis3` retains exact demand classes, grid
facts, lower bounds, and initial free space for repeated search; cheap work
counts are derived directly from those facts. `PlacementOrder3` separately
reports deterministic ordering evidence. Both are advisory;
`verify_packing_3d` remains the immediate authoritative query.

`IrregularPacking2::new` owns a stable item inventory and caches one exact
Hypercurve no-fit region per unordered convex-item pair. Cache blockers remain
data, while `.verify` immediately replays sheet containment, pair
contact/overlap, and accounting without introducing an epsilon. Boundary
contact is feasible; an unavailable no-fit region produces `Unknown`.
`.bottom_left` deterministically proposes translations from the sheet corner
and cached no-fit boundary vertices, then attaches authoritative replay to the
proposal report.

The bounded solver returns `Unknown` when its item or node limit prevents an
exhaustive result. A feasible replay proves feasibility, while objective values,
lower bounds, and heuristic rankings do not by themselves prove global
optimality.

## Exactness Boundary

Dimensions, positions, lengths, areas, volumes, costs, weights, and bounds use
`Real`. The implemented interval, rectangle, and cuboid containment and
no-overlap tests use certified sign queries. Uncertified comparisons propagate
as `Unknown`; the crate does not introduce an epsilon or silently lower a
decision to `f64`.

Exact replay currently covers:

- 1D, fixed-orientation 2D, cardinally oriented 2D, fixed-orientation 3D, and
  six-permutation oriented 3D geometry;
- translation-only convex line-contour sheet geometry through exact no-fit
  regions;
- one-placement-per-item accounting and multi-bin assignment;
- exact used space, waste, height, cost, and lexicographic objective comparison;
- capacity and pair-incompatibility necessary bounds;
- positive kerf/clearance separation;
- full-base, area-ratio, and footprint-center support policies; and
- direct top-face load limits with caller-supplied exact weights.

Snapshot helpers preserve rational text or full `hyperreal` structural JSON.
The binary format uses length-prefixed UTF-8 fields rather than primitive-float
encodings. No-overlap model exports preserve exact coordinate domains and
pairwise axis-separation disjunctions for solver adapters.

## Limitations

- Orientations are cardinal dimension permutations, not arbitrary rotations.
- Irregular no-fit caching currently accepts simple convex line contours;
  concave shapes explicitly report that convex decomposition is required, and
  native curves remain unsupported by this cache surface.
- `CenterOfMassProjection` checks the geometric footprint center, not a mass
  distribution supplied by a physics model.
- Support and direct-load reports do not model friction, deformation, dynamics,
  or transitive load propagation.
- Routing, manufacturing-process, and richer physical constraints require
  certified domain handoffs.
- The bounded 3D branch-and-bound backend is intentionally for small,
  fixed-orientation, one-bin instances; the crate is not a general
  optimality-proving optimizer.

## Development

```sh
cargo fmt --all -- --check
cargo test --locked
cargo check --benches --locked
cargo clippy --all-targets --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
cargo test --all-features --test dispatch_trace
cargo bench --bench feasibility
cargo bench --bench replay_micro
```

The optional `dispatch-trace` feature forwards `hyperreal`'s exact-dispatch
instrumentation. The targeted integration test verifies that representative
rational replay and pair-bound workloads request neither approximation nor an
unknown-fact fallback. Benchmark baselines, retained changes, and rejected
experiments are recorded in [PERFORMANCE.md](PERFORMANCE.md).

## Reference-Guided Design

The references are used as design constraints rather than as a claim that every
algorithm in each source is implemented:

- Yap motivates the proposal/replay boundary, certified sign decisions, and
  explicit `Unknown` outcomes.
- Dyckhoff's typology motivates separate stock, sheet, cuboid, and multi-bin
  carriers instead of one ambiguous packing interface.
- Martello and Toth, and Martello–Pisinger–Vigo, motivate exact capacity and
  incompatibility bounds plus limit-bearing branch-and-bound for small 3D
  instances.
- Lodi–Martello–Vigo motivates keeping constructive 3D heuristics subordinate
  to exact replay and combining them with bounded tabu improvement.
- Crainic–Perboli–Tadei motivates the extreme-point proposal surface and its
  retained candidate-point trace.
- Jylänki motivates the explicit shelf, skyline, MaxRects, and guillotine 2D
  families and their alternative scoring rules.
- Iori and coauthors motivate keeping 2D problem structure, no-overlap model
  exports, necessary bounds, and proof status explicit for future exact solver
  adapters.
- Bortfeldt and Wäscher motivate separate orientation, clearance, support,
  load, assignment, cost, and domain-handoff reports for practical container
  constraints.
- Glover motivates explicit tenure and evaluation budgets in tabu search.
- Wolpert and Macready motivate problem-specific deterministic portfolios,
  with no heuristic presented as universally best.
- Hoos and Stützle motivate seeded multistart, bounded neighborhoods, explicit
  termination status, and reproducible local-search traces.

## References

- Chee K. Yap, [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721(95)00040-2),
  *Computational Geometry* 7(1–2), 1997.
- Harald Dyckhoff, [“A Typology of Cutting and Packing Problems”](https://doi.org/10.1016/0377-2217(90)90350-K),
  *European Journal of Operational Research* 44(2), 1990.
- Silvano Martello and Paolo Toth, [*Knapsack Problems: Algorithms and Computer
  Implementations*](https://books.google.com/books?id=0dhQAAAAMAAJ), Wiley, 1990.
- Silvano Martello, David Pisinger, and Daniele Vigo, [“The Three-Dimensional Bin
  Packing Problem”](https://doi.org/10.1287/opre.48.2.256.12386), *Operations
  Research* 48(2), 2000.
- Andrea Lodi, Silvano Martello, and Daniele Vigo, [“Heuristic Algorithms for the
  Three-Dimensional Bin Packing Problem”](https://doi.org/10.1016/S0377-2217(02)00134-0),
  *European Journal of Operational Research* 141(2), 2002.
- Teodor Gabriel Crainic, Guido Perboli, and Roberto Tadei, [“Extreme Point-Based
  Heuristics for Three-Dimensional Bin Packing”](https://doi.org/10.1287/ijoc.1070.0250),
  *INFORMS Journal on Computing* 20(3), 2008.
- Jukka Jylänki, [“A Thousand Ways to Pack the Bin—A Practical Approach to
  Two-Dimensional Rectangle Bin Packing”](https://trszdev.github.io/maxrects-bssf-global-demo/RectangleBinPack.pdf),
  2010.
- Manuel Iori, Vinícius L. de Lima, Silvano Martello, Flávio K. Miyazawa, and
  Michele Monaci, [“Exact Solution Techniques for Two-Dimensional Cutting and
  Packing”](https://doi.org/10.1016/j.ejor.2020.06.050), *European Journal of
  Operational Research* 289(2), 2021.
- Andreas Bortfeldt and Gerhard Wäscher, [“Constraints in Container Loading—A
  State-of-the-Art Review”](https://doi.org/10.1016/j.ejor.2012.12.006),
  *European Journal of Operational Research* 229(1), 2013.
- Fred Glover, [“Tabu Search—Part I”](https://doi.org/10.1287/ijoc.1.3.190),
  *ORSA Journal on Computing* 1(3), 1989.
- David H. Wolpert and William G. Macready, [“No Free Lunch Theorems for
  Optimization”](https://doi.org/10.1109/4235.585893), *IEEE Transactions on
  Evolutionary Computation* 1(1), 1997.
- Holger H. Hoos and Thomas Stützle, [*Stochastic Local Search: Foundations and
  Applications*](https://www.cs.ubc.ca/~hoos/SLS-Book/), Morgan Kaufmann, 2004.

## Hyper Ecosystem

`hyperpack` uses [hyperreal](https://github.com/timschmidt/hyperreal) for exact
scalars. Related geometry, manufacturing, search, and integration crates include
[hyperparts](https://github.com/timschmidt/hyperparts),
[hyperphysics](https://github.com/timschmidt/hyperphysics),
[hyperpath](https://github.com/timschmidt/hyperpath),
[hyperdrc](https://github.com/timschmidt/hyperdrc),
[hypersolve](https://github.com/timschmidt/hypersolve), and
[hyperevolution](https://github.com/timschmidt/hyperevolution).
