<h1>
  hyperpack
  <img src="./doc/hyperpack.png" alt="hyperpack logo" width="144" align="right">
</h1>

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
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector and transform
  carriers used by geometry and physics consumers.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy for
  geometry-heavy sibling crates.
- [hypertri](https://github.com/timschmidt/hypertri), [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypermesh](https://github.com/timschmidt/hypermesh), and
  [hypervoxel](https://github.com/timschmidt/hypervoxel): geometric and sampled
  evidence that can become packable envelopes, keepouts, or process fixtures.
- [hyperparts](https://github.com/timschmidt/hyperparts): part, package, and process
  facts that can generate packable items.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): mass, material, support,
  and center-of-mass constraints.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): electrical constraints and
  electromechanical package context.
- [hyperpath](https://github.com/timschmidt/hyperpath) and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): routing or manufacturing checks
  that can become domain handoff constraints.
- [hypersolve](https://github.com/timschmidt/hypersolve): future exact/solver backend
  for small optimality and feasibility fixtures.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): proposal-search layer
  for packing order, mutation, and portfolio exploration.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): exact boundary-representation
  evidence for future enclosure and fixture envelopes.
- [hypersdf](https://github.com/timschmidt/hypersdf): signed-distance and implicit-field
  evidence for future clearance and fit previews.

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

- `StockBin1`, `SheetBin2`, `Bin3`, `Item3`, `Rect2`, `AxisBox3`, `Placement3`, and
  their oriented variants model exact 1D, 2D, and 3D packing inputs.
- `verify_packing_1d`, `verify_packing_2d`, `verify_packing_3d`,
  `verify_oriented_packing_*`, and `verify_multi_bin_packing_3d` replay exact
  containment, no-overlap, orientation, assignment, cost, and waste evidence.
- `CapacityBoundReport*`, pair-incompatibility reports, support reports, load reports,
  clearance reports, and height/objective reports make necessary bounds and side
  constraints auditable.
- Shelf, skyline, MaxRects, guillotine, corner-point, extreme-point, maximal-space,
  LAFF, local-search, tabu, multistart, reinsertion, and bin-emptying helpers are
  proposal engines; their reports are useful only after exact replay.
- `PreparedPacking3`, snapshot helpers, no-overlap model exports, exact search reports,
  and domain handoff reports preserve performance caches, fixtures, solver-adapter
  boundaries, and ecosystem provenance without becoming proof by themselves.
- `FeasibilityStatus`, `SheetVerification2`, `PackingVerification3`,
  `ExactSearchReport3`, portfolio reports, and local/tabu/multistart reports keep the
  difference between proposal, replay, infeasibility, and unknown explicit.

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

Numerical explosion is controlled by using specialized interval, rectangle, cuboid,
orientation, support, and clearance replays before reaching for general solvers. Lower
bounds, prepared demand classes, candidate points, free-space summaries, snapshots, and
model-export reports preserve reusable structure without claiming proof.

Fixture snapshots use line-oriented text with escaped ids or binary length-prefixed
UTF-8 fields with raw ids. Rational `Real` values are emitted as rational text;
non-rational structural values stay inside escaped `hyperreal` JSON strings rather
than being rounded to `f64`.

## Performance Model

`hyperpack` keeps exact replay simple and pushes speed into proposal generation.
Specialized interval, rectangle, and cuboid checks avoid general solvers for common
cases. Prepared packing records cache demand classes, scalar grid facts, lower bounds,
initial free-space state, and deterministic placement order so heuristics do not have
to rediscover the same structure.

The bounded exact search backend is intentionally limit-bearing: it prefilters with
capacity bounds, branches over exact candidate points, and returns `Unknown` when an
item or node limit prevents a certificate. Portfolio and local-search helpers spend
their budgets on diverse proposals, but ranking is based on replay status and exact
objective evidence rather than hidden floating-point scores.
Tabu memory is reported explicitly, and tabu moves are admissible only when exact
replay proves an aspiration improvement over the current best report.

## Current Status

Implemented today:

- exact 1D stock, 2D sheet, oriented 2D, 3D cuboid, oriented 3D, and multi-bin replay;
- capacity, pair-incompatibility, clearance, support, direct-load, height, objective,
  snapshot, model-export, prepared-problem, and domain-handoff reports;
- 2D shelf, skyline, MaxRects, guillotine, and portfolio proposal engines;
- 3D corner-point, extreme-point, maximal-space, guillotine, LAFF, portfolio,
  local-search, tabu, multistart, repair, and bin-emptying proposal engines;
- bounded small-instance exact 3D branch-and-bound and explicit feasible, infeasible,
  and unknown replay states.

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

## Usage

Replay first, then trust the layout:

```rust,ignore
use hyperpack::{
    FeasibilityStatus, ItemId, Rect2, SheetBin2, SheetItem2, SheetPlacement2,
    verify_packing_2d,
    maxrects_best_short_side_fit_2d,
};
use hyperreal::Real;

let bin = SheetBin2::new(Rect2::new(Real::from(10), Real::from(10))?);
let items = vec![
    SheetItem2::new(ItemId::new("a")?, Rect2::new(Real::from(4), Real::from(4))?),
    SheetItem2::new(ItemId::new("b")?, Rect2::new(Real::from(6), Real::from(3))?),
];

let proposal = maxrects_best_short_side_fit_2d(&bin, &items);
let replay = verify_packing_2d(&bin, &items, &proposal.placements)?;
assert_eq!(replay.status, FeasibilityStatus::Feasible);
```

Use the same shape for 3D: run a corner-point, maximal-space, guillotine, LAFF, exact
search, local-search, or portfolio proposal; replay it with `verify_packing_3d`; then
layer in orientation, support, load, clearance, multi-bin, objective, and domain-handoff
reports as needed.

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry* 7.1-2
  (1997): 3-23.
- Jylanki, Jukka. "A Thousand Ways to Pack the Bin: A Practical Approach to
  Two-Dimensional Rectangle Bin Packing." 2010.
- Martello, Silvano, and Paolo Toth. *Knapsack Problems: Algorithms and Computer
  Implementations*. Wiley, 1990.
- Martello, Silvano, David Pisinger, and Daniele Vigo. "The Three-Dimensional Bin
  Packing Problem." *Operations Research* 48.2 (2000): 256-267.
- Lodi, Andrea, Silvano Martello, and Daniele Vigo. "Heuristic Algorithms for the
  Three-Dimensional Bin Packing Problem." *European Journal of Operational Research*
  141.2 (2002): 410-420.
- Iori, Manuel, Silvano Martello, and Michele Monaci. "Exact Solution Techniques for
  Two-dimensional Cutting and Packing." *European Journal of Operational Research*
  179.3 (2007): 910-931.
- Wolpert, David H., and William G. Macready. "No Free Lunch Theorems for Optimization."
  *IEEE Transactions on Evolutionary Computation* 1.1 (1997): 67-82.
- Hoos, Holger H., and Thomas Stutzle. *Stochastic Local Search: Foundations and
  Applications*. Morgan Kaufmann, 2004.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench feasibility
```
