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
  set-algebra planning records.
- PCB types describe traces, pads, vias, board outlines, clearance reports, annular
  ring checks, via drill policy, and layer-span reports.
- Routing helpers describe length matching, meanders, obstacle-aware detours,
  differential-pair skew, and constant feed-time certification.
- `PcbConstraintSet`, `ToolpathConstraintSet`, `RectangularRegion`, and residual builders
  lower PCB/CAM semantics into `hypersolve` without moving domain ownership out of
  `hyperpath`.
- Specctra import/export records provide a text/grid route handoff surface,
  including retained route envelopes, quoted aliases, vias, and multi-segment
  path wires.

## Precision Model

Path coordinates, widths, distances, offsets, and timing values use `Real`. `SourceGrid`
converts fixed-grid source tokens directly to rationals, so KiCad, Gerber, Excellon,
Specctra, and G-code imports do not pass through primitive floats just to become exact
again. Explicit decimal tokens can be lifted with
`PathProvenance::real_from_decimal_token` under the same rule. Clearance, tangency,
length, skew, area, and containment reports return certified
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
- CAM rectangular pocket, bead, infill, support, and rectangular region
  set-algebra helpers;
- PCB trace, pad, via, board-outline, clearance, via-policy, annular-ring, and
  layer-span reports;
- length-match, meander, obstacle-aware detour, differential-pair skew, constant
  feed-time, and Specctra route helpers;
- PCB and toolpath residual builders for center clearance, differential-pair skew,
  length matching, feed/time, Bezier offset samples, rectangular containment, and
  rectangular area replay.

Known limits: general path search, full curved offset trimming, freeform CAM pockets,
and autorouting are not complete.

See [the reference and performance audit](PERFORMANCE.md) for the complete
source mapping, retained benchmark result, rejected trials, and validation evidence.

## Installation

```toml
[dependencies]
hyperpath = "0.3.0"
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
    PredicatePolicy::default(),
)?;

let swept = SweptLineSegment::new(centerline.clone(), Real::from(1))?;
let trace = PcbTrace::new(NetId(1), TraceLayer(0), swept);
```

See [`examples/basic.rs`](examples/basic.rs) for a compiling version of this
workflow.

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
set-algebra reports. For PCB use trace, pad, via, board-outline, clearance,
annular-ring, layer-span, and Specctra route records. For smooth paths use tangent and
G1 chain certification helpers.

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry* 7.1-2
  (1997): 3-23. https://doi.org/10.1016/0925-7721(95)00040-2.
- Lee, C. Y. "An Algorithm for Path Connections and Its Applications." *IRE
  Transactions on Electronic Computers* EC-10.3 (1961): 346-365.
  https://doi.org/10.1109/TEC.1961.5219222.
- Hightower, David W. "A Solution to Line-Routing Problems on the Continuous Plane."
  *Proceedings of the 6th Design Automation Workshop* (1969): 1-24.
  https://doi.org/10.1145/800260.809014.
- Farouki, Rida T., and Takis Sakkalis. "Pythagorean Hodographs." *IBM Journal of
  Research and Development* 34.5 (1990): 736-752.
  https://doi.org/10.1147/rd.345.0736.
- Erkorkmaz, Kaan, and Yusuf Altintas. "High Speed CNC System Design. Part I:
  Jerk Limited Trajectory Generation and Quintic Spline Interpolation."
  *International Journal of Machine Tools and Manufacture* 41.9 (2001):
  1323-1345. https://doi.org/10.1016/S0890-6955(01)00002-5.
- Ucamco. [*Gerber Layer Format Specification*](https://www.ucamco.com/en/gerber/downloads).
- KiCad Project. [*KiCad PCB File Format / S-expression Board
  Format*](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/index.html).
- Cadence Design Systems. [*SPECCTRA Design Language
  Reference*](https://cdn.hackaday.io/files/1666717130852064/specctra.pdf).

## Development

Useful local checks:

```sh
cargo test
cargo test --all-features --all-targets
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
cargo check --examples --benches
cargo run --example basic
cargo bench --bench path_predicates
```

## Hyper Ecosystem

`hyperpath` builds on [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlimit](https://github.com/timschmidt/hyperlimit), and
[hypersolve](https://github.com/timschmidt/hypersolve). It connects exact curve,
PCB, CAM, and routing evidence to the other [Hyper geometry and engineering
crates](https://github.com/timschmidt?tab=repositories&q=hyper&type=source).
