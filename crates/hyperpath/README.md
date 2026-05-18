<h1>
  hyperpath
</h1>

`hyperpath` owns exact-aware path planning and routing carriers for the Hyper ecosystem.
It records line, arc, Bezier, offset, tangent, CAM, PCB, Specctra, swept-volume, and
provenance facts while delegating scalar arithmetic to `hyperreal`, exact predicates to
`hyperlimit`, and constraint certification to `hypersolve`.

The crate is not a full autorouter or CAM kernel yet. It is the path-domain layer where
candidates, source-grid provenance, clearance reports, tangent facts, and certification
evidence remain explicit.

## Hyper Ecosystem

`hyperpath` connects exact geometry decisions to routing and manufacturing workflows.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact path coordinates,
  distances, widths, offsets, and timing values.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate decisions for
  clearance, sidedness, and tangency.
- [hypersolve](https://github.com/timschmidt/hypersolve): length, skew, feed-time, and
  future constrained path certification.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB readiness checks that consume
  routing and board evidence.
- [hypercircuit](https://github.com/timschmidt/hypercircuit) and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): electrical and physical
  context for coupled routing, heating, support, and process checks.

## Typical Path Problems

Routing and toolpath software often mixes candidate generation, clearance checks,
offsetting, smoothing, source-grid import, and manufacturing policy in one algorithm.
That makes failures hard to audit: a bad route can come from a lossy import, rounded
clearance test, tangent discontinuity, slot/offset approximation, or solver-side
constraint miss.

`hyperpath` keeps those responsibilities visible. It records provenance and source-grid
units, separates path candidates from certification reports, and exposes exact-aware
checks for clearances, tangency, length matching, offsets, and CAM rectangular plans
before downstream crates accept the path as ready.

## Main Types

- `LinePathSegment`, `CircularArc`, `ExplicitCircularArc`, Bezier types, and swept-line
  carriers describe path primitives and retained facts.
- `PathProvenance`, `SourceGrid`, `ConstructionStamp`, and source-format/unit enums
  preserve import and construction evidence.
- Offset candidate types cover axis-aligned segments, cardinal arcs, explicit arcs, and
  sampled Bezier offsets.
- CAM types describe rectangular pocket, bead, infill, support, and rectangular-region
  boolean plans.
- PCB types describe traces, pads, vias, board outlines, clearance reports, annular
  ring checks, via drill policy, and layer-span reports.
- Routing helpers describe length matching, meanders, obstacle-aware detours,
  differential-pair skew, and constant feed-time certification.
- Specctra import/export records provide a text/grid route handoff surface.

## Precision Model

Path coordinates, widths, distances, offsets, and timing values use `Real`. Source-grid
records preserve units and import scale so downstream crates can distinguish exact input
from converted or adapter-generated geometry. Clearance, tangency, length, and skew
reports should return certified status or explicit failure/unknown rather than
manufacturing a primitive-float decision.

## Performance Model

The crate focuses on small exact-aware carriers and specialized checks rather than a
single global path search. Axis-aligned, cardinal, rectangular, and grid-route helpers
give common CAD/CAM/PCB cases cheap exact paths. Provenance records and facts are
retained so repeated checks can avoid reinterpreting source units, tangent orientation,
and simple geometry classes.

Future performance work should add prepared path objects, spatial indexes, and batch
certification without changing the exact replay boundary.

## Current Status

Implemented today:

- line, arc, explicit-arc, quadratic/cubic/higher-order Bezier, rational-conic,
  swept-segment, and tangent-chain carriers;
- source-grid, construction-stamp, source-format, and provenance records;
- line, arc, Bezier, and cardinal offset candidate APIs;
- CAM rectangular pocket, bead, infill, support, and region-boolean helpers;
- PCB trace, pad, via, board-outline, clearance, via-policy, annular-ring, and
  layer-span reports;
- length-match, meander, obstacle-aware detour, differential-pair skew, constant
  feed-time, and Specctra route helpers.

Known limits: general path search, full curved offset trimming, freeform CAM pockets,
and autorouting are not complete.

## Installation

```toml
[dependencies]
hyperpath = "0.2.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperpath = { path = "../hyperpath" }
```

## Usage

Keep imported geometry, candidate construction, and certification reports separate:

```rust,ignore
use hyperpath::{
    LinePathSegment, NetId, OffsetSide, PcbTrace, SourceGrid, SourceLengthUnit,
    SweptLineSegment, TraceLayer, offset_axis_aligned_segment,
};
use hyperreal::Real;

let grid = SourceGrid::new(SourceLengthUnit::Millimeter, Real::from(1_000));
let centerline = LinePathSegment::new([Real::from(0), Real::from(0)].into(),
                                      [Real::from(10), Real::from(0)].into());
let offset = offset_axis_aligned_segment(&centerline, Real::from(2), OffsetSide::Left)?;

let swept = SweptLineSegment::new(centerline, Real::from(1))?;
let trace = PcbTrace::new(NetId::new("D+")?, TraceLayer::Top, swept);
```

For CAM use `RectangularPocket`, bead/infill/support planners, and rectangular-region
boolean reports. For PCB use trace, pad, via, board-outline, clearance, annular-ring,
layer-span, and Specctra route records. For smooth paths use tangent and G1 chain
certification helpers.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench path_predicates
```
