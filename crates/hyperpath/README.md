<h1>
  Hyperpath
  <img src="./doc/hyperpath.png" alt="Hyperpath logo" width="144" align="right">
</h1>

Exact-aware path, routing, PCB, and CAM carriers for the Hyper ecosystem.

Hyperpath records line, arc, Bézier, offset, tangent, swept-volume, toolpath,
board-routing, source-grid, and solver-handoff evidence. It owns path-domain
semantics while delegating scalars to Hyperreal, exact predicates to
Hyperlimit, and generic equation certification to Hypersolve.

It is not a complete autorouter or freeform CAM kernel. Candidate generation,
imported coordinates, clearance checks, manufacturing policy, and
certification remain separate and inspectable.

This README describes crate version `0.3.0`.

## Primary types

| Type | Role |
| --- | --- |
| `LinePathSegment`, `CircularArc`, `ExplicitCircularArc` | Exact line and circular-arc paths |
| `QuadraticBezier`, `CubicBezier`, `HigherOrderBezier`, `RationalQuadraticBezier` | Exact-aware smooth path carriers |
| `SweptLineSegment`, `PcbTrace` | Width-bearing path and PCB trace |
| `PcbBoardOutline`, pad and via types | Board geometry and fabrication intent |
| `RectangularPocket`, `RectangularRegion` | Supported exact CAM planning domains |
| `TangentSpan`, `G1ChainCertificationReport` | Tangency and continuity evidence |
| `SpecctraRoute`, `SpecctraGridRouteRecords` | In-memory and fixed-grid route exchange |
| `PcbConstraintSet`, `ToolpathConstraintSet` | Domain-owned Hypersolve residual collections |

## Install

```toml
[dependencies]
hyperpath = "0.3.0"
```

There are no default features. `dispatch-trace` enables exact-predicate
instrumentation in Hyperlimit and Hyperreal.

## Quick start

This checked example offsets an axis-aligned centerline, creates its swept
width, and assigns it to a PCB net and layer.

<!-- quickstart:start -->
```rust
use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    LinePathSegment, NetId, OffsetSide, PcbTrace, SweptLineSegment, TraceLayer,
    offset_axis_aligned_segment,
};
use hyperreal::Real;

fn main() -> Result<(), String> {
    let centerline = LinePathSegment::new(
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(10), Real::from(0)),
    );
    let offset = offset_axis_aligned_segment(
        &centerline,
        Real::from(2),
        OffsetSide::Left,
        PredicatePolicy,
    )
    .map_err(|error| format!("offset failed: {error:?}"))?;
    assert_eq!(offset.segment.start(), &Point2::new(0.into(), 2.into()));

    let swept = SweptLineSegment::new(centerline, Real::from(1))?;
    let _trace = PcbTrace::new(NetId(1), TraceLayer(0), swept);
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

## Ownership and workflow

```text
source grid / exact points
            │
       path primitives
            │
 candidate arrangement / offset / route / toolpath
            │
 domain clearance + tangency + manufacturing reports
            │
 certified handoff or explicit blocker
```

Construction functions return completed domain values or reports immediately;
there is no public prepared-object lifecycle. Provenance and source-grid units
remain attached so an import is not accidentally treated as authored exact
geometry.

## API guide

### Path primitives and evaluation

- `LinePathSegment::{new, start, end, facts, bounds_min, bounds_max,
  length_squared, axis_length, start_tangent, end_tangent,
  compare_points_along}` is the basic exact segment carrier.
- `CircularArc` covers cardinal-center arcs; `ExplicitCircularArc` covers
  exact endpoints, center, direction, sweep, classification, intersections,
  overlap, and tangent reports.
- `BezierParameter::new` creates an exact rational parameter.
  `QuadraticBezier`, `CubicBezier`, `HigherOrderBezier`, and
  `RationalQuadraticBezier` expose construction, facts, evaluation,
  derivative, and squared speed.
- `SweptLineSegment::{new, centerline, width, facts, axis_centerline_length}`
  adds exact width to a line centerline.

### Arrangement, intersection, offset, and continuity

- `arrange_line_segments`, `arrange_explicit_arcs`, and
  `arrange_line_segments_with_explicit_arcs` produce retained event,
  breakpoint, fragment, half-edge, and cell-graph reports.
- Line/Bézier intersection functions cover quadratic, cubic, and rational
  quadratic carriers, including axis-aligned specialized routes and algebraic
  breakpoint evidence.
- Mixed arrangement entry points combine lines with quadratic, rational
  quadratic, cubic, or all supported Bézier families.
- `offset_axis_aligned_segment`, `offset_cardinal_arc`,
  `offset_explicit_arc`, and the `offset_*_bezier_sample` functions return
  named exact or sampled candidates; sampled offsets are not promoted to
  trimmed topology.
- `classify_tangent_alignment`, `classify_tangent_join`,
  `classify_tangent_chain`, and `certify_g1_chain` report direction and G1
  evidence.
- Pythagorean-hodograph and quintic PH smoothing APIs retain exact hodograph,
  feed, and endpoint-tangent evidence where supported.

### PCB geometry and rules

- `PcbTrace::new`, trace facts, `NetId`, and `TraceLayer` model routed copper.
- Board outlines include rectangular, orthogonal, convex, obround, and circular
  carriers. Pad types include rectangular, cardinal rectangular, oriented
  rectangular, rounded rectangular, orthogonal, convex, obround, and circular
  geometry.
- `check_trace_clearance`, trace-to-pad, trace-to-board, pad-to-board, and
  drill-to-board functions return `ClearanceStatus` plus exact evidence.
- `PcbViaStack` retains drill intent and layer span.
  `certify_via_fabrication_policy` and the via annular-ring, aspect-ratio,
  drill-policy, layer-span, transition, and acceptance reports keep fabrication
  checks distinct.
- Same-net identity does not silently waive geometric or fabrication checks;
  callers select the policy represented by the requested report.

### CAM and additive planning

- `RectangularPocket::new` validates an exact axis-aligned pocket.
- `rectangular_pocket_rings` and `rectangular_pocket_link_graph` build offset
  rings and links with explicit stop/error status.
- `rectangular_beads` and `rectangular_serpentine_infill_graph` build supported
  additive bead and infill plans.
- `rectangular_support_footprint` reports support coverage.
- `intersect_rectangular_regions` and `subtract_rectangular_region` provide
  exact set-algebra reports; `rectangular_rest_material_graph` retains staged
  rest-material evidence.

Freeform curved pocket trimming is not inferred from these rectangular APIs.

### Routing and timing

- `length_match_problem` and `certify_length_extension` define and replay a
  length extension.
- `single_detour_meander`, `multi_detour_meander`,
  `alternating_detour_meander`, `nonuniform_detour_meander`,
  `obstacle_aware_detour_meander`, and `keepout_aware_detour_meander` build
  exact structured candidates.
- Placement classifiers report which slots or keepouts accept a candidate.
- `certify_differential_pair_skew`, `certify_constant_feed_time`, and
  `certify_acceleration_limited_feed_time` cover common route/toolpath checks.
- Path-wide feed APIs certify constant, acceleration-limited, symmetric
  jerk-limited, and corner-lookahead schedules with per-join evidence.

These are structured proposal and certification helpers, not a global route
search.

### Specctra and solver handoffs

- Fixed-grid constructors, serializers, parsers, and import functions cover
  Specctra traces, vias, arc wires, keepouts, and route rules.
- `SpecctraRoute::{new, with_vias, with_vias_and_arcs, with_curves,
  from_records, from_trace_and_via_records,
  from_trace_via_and_arc_records}` retains traces, exact arcs, and cubic
  Béziers.
- `import_specctra_text_route` parses the supported route surface. Quoted
  aliases and source-grid coordinates remain explicit.
- `PcbConstraintSet::push` and `ToolpathConstraintSet::push` retain
  domain-owned constraints.
- Solver helpers cover center clearance, differential-pair skew, length
  matching, constant and jerk-limited feed time, sampled Bézier offset,
  rectangular containment, area, and difference-area equations.

## Guarantees and boundaries

- Coordinates, widths, distances, offsets, lengths, areas, and timing values
  use `hyperreal::Real`.
- Fixed-grid or decimal import should lift source tokens directly to rationals;
  no primitive-float round trip is required.
- Clearance, tangency, containment, length, skew, and manufacturing reports
  return certified status, failure, or explicit uncertainty.
- Source-grid facts, construction stamps, tangent records, and low-degree
  residuals stay structured to avoid unnecessary symbolic expansion.
- Sampled Bézier offsets, incomplete arrangements, and imported routes remain
  candidates until their required domain reports pass.

General path search, complete curved-offset trimming, freeform CAM pockets, and
full autorouting remain outside the current API.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Hyperlimit/Hyperreal exact-predicate instrumentation |

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --all-features --all-targets
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --examples --benches --all-features
```

The reference-to-implementation audit, benchmark definitions, measured
results, and rejected optimization trials are in
[PERFORMANCE.md](PERFORMANCE.md). Fuzz ownership is documented in
[fuzz/README.md](fuzz/README.md).

## References

These sources describe the exact-computation, routing, path-smoothing, and
exchange standards relevant to Hyperpath:

- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).
- Lee, C. Y. “An Algorithm for Path Connections and Its Applications.”
  *IRE Transactions on Electronic Computers* EC-10(3), 1961.
  [DOI: 10.1109/TEC.1961.5219222](https://doi.org/10.1109/TEC.1961.5219222).
- Hightower, D. W. “A Solution to Line-Routing Problems on the Continuous
  Plane.” *Proceedings of the 6th Design Automation Workshop*, 1969.
  [DOI: 10.1145/800260.809014](https://doi.org/10.1145/800260.809014).
- Farouki, R. T., and Sakkalis, T. “Pythagorean Hodographs.”
  *IBM Journal of Research and Development* 34(5), 1990.
  [DOI: 10.1147/rd.345.0736](https://doi.org/10.1147/rd.345.0736).
- Erkorkmaz, K., and Altintas, Y. “High Speed CNC System Design. Part I:
  Jerk Limited Trajectory Generation and Quintic Spline Interpolation.”
  *International Journal of Machine Tools and Manufacture* 41(9), 2001.
  [DOI: 10.1016/S0890-6955(01)00002-5](https://doi.org/10.1016/S0890-6955(01)00002-5).
- Ucamco. *The Gerber Layer Format Specification*.
  [Official downloads](https://www.ucamco.com/en/gerber/downloads).
- KiCad Project. *S-expression PCB File Format*.
  [Official developer documentation](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/index.html).
- Cadence Design Systems. *SPECCTRA Design Language Reference*.
  [Reference manual](https://cdn.hackaday.io/files/1666717130852064/specctra.pdf).

## Acknowledgements

Hyperpath builds on
[Hyperreal](https://github.com/timschmidt/hyperreal),
[Hyperlimit](https://github.com/timschmidt/hyperlimit), and
[Hypersolve](https://github.com/timschmidt/hypersolve). The cited research and
format owners define algorithms and exchange semantics; they do not imply
source-code derivation or vendor endorsement.

## License and contributing

Licensed under Apache-2.0 as declared in [Cargo.toml](Cargo.toml).

Bug reports should include exact path inputs, source-grid/provenance records,
the requested check, enabled features, and the complete result. Before
proposing a change, run formatting, focused tests, all targets/features, and
strict Clippy.
