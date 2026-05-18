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
- `HeuristicFamily` records which proposal family produced or motivated a report.
- `PackingReport3`, `FreeSpaceReport3`, and `LowerBoundReport` preserve proposal and
  bound evidence.
- `FeasibilityReplay3` and `FeasibilityStatus` distinguish feasible, infeasible, and
  unknown replay outcomes.
- `PackError` and `PackResult` keep construction and replay failures typed.

## Precision Model

Dimensions, positions, volumes, and bounds are `Real` values. Containment and pairwise
no-overlap replay are exact for the axis-aligned 3D model currently implemented. The
crate does not silently round dimensions to primitive floats to make a heuristic fit.

Future support, load, route, center-of-mass, and process checks should keep the same
pattern: exact inputs where possible, explicit adapter reports where not, and unknown
when the constraint has not been certified.

## Performance Model

`hyperpack` keeps the current replay surface intentionally small. Fast heuristic
generation can remain domain-specific, while replay uses simple axis-aligned interval
checks and grouped evidence records. Lower-bound and free-space reports are separated
from placement certification so future algorithms can compare proposal quality without
rerunning every exact check.

Performance should improve by narrowing candidate pairs, retaining free-space
structure, and dispatching specialized exact checks before invoking generic solver or
search machinery.

## Current Status

Implemented today:

- exact axis-aligned 3D item, bin, container, and placement carriers;
- heuristic-family, packing, free-space, and lower-bound reports;
- exact containment and pairwise no-overlap replay;
- feasible, infeasible, and unknown replay states.

Known limits: rotated packing, rich support/contact constraints, center-of-mass checks,
routing clearance, process policy, and optimality certificates are still future work.

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
