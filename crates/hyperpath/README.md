<h1>
  hyperpath
  <img src="./doc/hyperpath.png" alt="hyperpath logo" width="144" align="right">
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
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector, point, and
  homogeneous carriers used by sibling geometry crates.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate decisions for
  clearance, sidedness, and tangency.
- [hypertri](https://github.com/timschmidt/hypertri): triangulation and planar
  subdivision support for future pocket and offset arrangements.
- [hypercurve](https://github.com/timschmidt/hypercurve): exact curve and Bezier
  primitives that supply curved path and offset inputs.
- [hypermesh](https://github.com/timschmidt/hypermesh): exact mesh and surface evidence
  for obstacle, fixture, and swept-volume consumers.
- [hypervoxel](https://github.com/timschmidt/hypervoxel): sparse-grid process and swept
  evidence for sampled manufacturing handoffs.
- [hypersolve](https://github.com/timschmidt/hypersolve): length, skew, feed-time, and
  future constrained path certification.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB readiness checks that consume
  routing and board evidence.
- [hypercircuit](https://github.com/timschmidt/hypercircuit) and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): electrical and physical
  context for coupled routing, heating, support, and process checks.
- [hyperpack](https://github.com/timschmidt/hyperpack): package and panel metadata for
  board-level manufacturing context.
- [hyperparts](https://github.com/timschmidt/hyperparts): part and footprint records that
  anchor routing constraints.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): optimization and design
  exploration layer for route and process candidates.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): exact boundary-representation
  surfaces for future toolpath and fixture geometry.
- [hypersdf](https://github.com/timschmidt/hypersdf): signed-distance evidence for future
  clearance previews and implicit obstacles.

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
- Mesh-boolean handoff types extrude retained rectangular path/CAM footprints,
  certified axis-aligned swept segments, and layer-aware PCB trace/rectangular
  pad, strictly convex polygonal copper, and simple orthogonal copper sources
  into exact solids, replay holed orthogonal copper as outer-minus-void
  difference programs, fold solid and retained-holed PCB copper through
  composite same-net/layer union programs, clip copper to retained convex or
  orthogonal board outlines, fold multi-source boolean chains, mixed-operation
  boolean programs, same-net/layer PCB copper union programs, and CAM
  stock-minus-cutter plus orthogonal island-pocket rest-material programs,
  and clip retained additive support footprints and infill graphs to convex or
  orthogonal boundaries through `hypermesh`, and replay exact evidence before
  exposing accepted output topology.
- PCB types describe traces, pads, vias, board outlines, clearance reports, annular
  ring checks, via drill policy, and layer-span reports.
- Routing helpers describe length matching, meanders, obstacle-aware detours,
  differential-pair skew, and constant feed-time certification.
- `PcbConstraintSet`, `ToolpathConstraintSet`, `RectangularRegion`, and residual builders
  lower PCB/CAM semantics into `hypersolve` without moving domain ownership out of
  `hyperpath`.
- Specctra import/export records provide a text/grid route handoff surface.

## Precision Model

Path coordinates, widths, distances, offsets, and timing values use `Real`. `SourceGrid`
converts fixed-grid source tokens directly to rationals, so KiCad, Gerber, Excellon,
Specctra, and G-code imports do not pass through primitive floats just to become exact
again. Clearance, tangency, length, skew, area, and containment reports return certified
status or explicit failure/unknown rather than manufacturing a tolerance decision.

Numerical explosion is controlled by keeping source grids, construction stamps, tangent
facts, simple axis/cardinal cases, and solver residuals as structured objects. The crate
does not eagerly expand every path into sampled polylines or global arrangements.

## Performance Model

The crate focuses on small exact-aware carriers and specialized checks rather than a
single global path search. Axis-aligned, cardinal, rectangular, and grid-route helpers
give common CAD/CAM/PCB cases cheap exact paths. Provenance records, cached facts, and
domain residual builders let repeated checks avoid reinterpreting source units, tangent
orientation, simple geometry classes, and low-degree constraint structure.

Future performance work should add prepared path objects, spatial indexes, and batch
certification without changing the exact replay boundary.

## Current Status

Implemented today:

- line, arc, explicit-arc, quadratic/cubic/higher-order Bezier, rational-conic,
  swept-segment, and tangent-chain carriers;
- source-grid, construction-stamp, source-format, and provenance records;
- line, arc, Bezier, and cardinal offset candidate APIs;
- CAM rectangular pocket, bead, infill, support, and region-boolean helpers;
- retained CAM stock/cutter rest-material mesh-boolean programs, retained
  additive support/infill clipping programs, and strictly convex/simple
  orthogonal polygon prism sources plus holed orthogonal difference programs
  for PCB copper booleans;
- PCB trace, pad, via, board-outline, clearance, via-policy, annular-ring, and
  layer-span reports;
- length-match, meander, obstacle-aware detour, differential-pair skew, constant
  feed-time, and Specctra route helpers;
- PCB and toolpath residual builders for center clearance, differential-pair skew,
  length matching, feed/time, Bezier offset samples, rectangular containment, and
  rectangular area replay.

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
use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    LinePathSegment, NetId, OffsetSide, PcbTrace, SourceGrid, SourceLengthUnit,
    SweptLineSegment, TraceLayer, offset_axis_aligned_segment,
};
use hyperreal::Real;

let grid = SourceGrid::with_unit(1_000_000, SourceLengthUnit::Millimeter).unwrap();
let centerline = LinePathSegment::new(
    Point2::new(grid.real_from_units(0).unwrap(), grid.real_from_units(0).unwrap()),
    Point2::new(grid.real_from_units(10_000_000).unwrap(), grid.real_from_units(0).unwrap()),
);
let offset = offset_axis_aligned_segment(
    &centerline,
    Real::from(2),
    OffsetSide::Left,
    PredicatePolicy::STRICT,
)?;

let swept = SweptLineSegment::new(centerline.clone(), Real::from(1))?;
let trace = PcbTrace::new(NetId(1), TraceLayer(0), swept);
```

Solver-facing helpers keep PCB and CAM semantics in `hyperpath` while producing
`hypersolve` residuals:

```rust,ignore
use hyperlimit::Point2;
use hyperpath::{
    PcbConstraintSet, RectangularRegion, ToolpathConstraintSet,
    center_clearance_squared_constraint, rectangular_region_area_equation,
};
use hyperreal::Real;
use hypersolve::{SolverPoint2, VariableId};

let first = SolverPoint2::new(VariableId(0), VariableId(1));
let second = SolverPoint2::new(VariableId(2), VariableId(3));

let mut pcb = PcbConstraintSet::default();
pcb.push(center_clearance_squared_constraint(
    "diff-pair center clearance",
    first,
    second,
    Real::from(6),
));

let pocket = RectangularRegion::new(
    Point2::new(Real::from(0), Real::from(0)),
    Point2::new(Real::from(10), Real::from(5)),
);
let mut toolpath = ToolpathConstraintSet::default();
toolpath.push(rectangular_region_area_equation("pocket area", pocket, Real::from(50)));
```

For CAM use `RectangularPocket`, bead/infill/support planners, and rectangular-region
boolean reports. For PCB use trace, pad, via, board-outline, clearance, annular-ring,
layer-span, and Specctra route records. For smooth paths use tangent and G1 chain
certification helpers.

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry* 7.1-2
  (1997): 3-23.
- Lee, C. Y. "An Algorithm for Path Connections and Its Applications." *IRE
  Transactions on Electronic Computers* EC-10.3 (1961): 346-365.
- Hightower, David W. "A Solution to Line-Routing Problems on the Continuous Plane."
  *Proceedings of the 6th Design Automation Workshop* (1969): 1-24.
- Farouki, Rida T., and Takis Sakkalis. "Pythagorean Hodographs." *IBM Journal of
  Research and Development* 34.5 (1990): 736-752.
- Ucamco. *Gerber Layer Format Specification*.
- KiCad Project. *KiCad PCB File Format / S-expression Board Format*.
- Cadence Design Systems. *SPECCTRA Design Language Reference*.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench path_predicates
```
