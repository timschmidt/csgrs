# Changelog

## [0.3.0] - Unreleased

### Breaking

- Constructors and importers take an explicit `hyperlimit::PredicatePolicy`:
  `LinePathSegment::new`, `SweptLineSegment::new`, `PcbTrace::new`,
  `CircularArc::cardinal`, `ExplicitCircularArc::new`,
  `RationalQuadraticBezier::new`, `RectangularPocket::new`, the board-outline
  and pad constructors, `PcbCardinalRectPad::effective_rect`,
  `PcbViaStack::{new, with_drill, with_drill_intent}`,
  `SpecctraRoute::{from_records, from_trace_and_via_records}`, and
  `import_specctra_{text_route, trace_record, via_record}`.
- `LinePathSegment::new` returns
  `Result<LinePathSegment, LinePathSegmentError>`, and
  `LinePathSegment::prepared_bounds` is replaced by `bounds_min` and
  `bounds_max`.
- The `build_` prefix is dropped from the routing and tangent constructors.
  For example, `build_length_match_problem` is now `length_match_problem` and
  `build_g1_join_problem` is now `g1_join_problem`.
- CAM planners return immediate reports: `build_rectangular_pocket_plan` is
  now `rectangular_pocket_rings`, `build_rectangular_bead_plan` is now
  `rectangular_beads`, `build_rectangular_support_plan` is now
  `rectangular_support_footprint`, and
  `build_rectangular_serpentine_infill_graph` is now
  `rectangular_serpentine_infill_graph`. The `*Plan` result and error
  types become `PocketRingReport`, `RectangularBeadReport`,
  `RectangularSupportReport`, `PocketRingError`, `RectangularBeadError`, and
  `SupportFootprintError`. `PocketPlanStopReason` becomes
  `RectangularScheduleStop`, whose `LimitReached` replaces `MaxRingsReached`.
- `RectangularInfillGraph` exposes its region, axis, bead width, spacing, beads,
  and stop reason directly instead of a `plan` field, and
  `InfillGraphError::EmptyBeadPlan` is now `EmptyBeads`.
- Error enums gain variants, including `PredicateUnresolved` on the offset,
  arc, Bézier, meander, route-certification, infill, and Specctra errors.
  `SpecctraGridRouteRecords` gains `arcs`, `keepouts`, and `rules` fields.

### Added

- Exact arrangements with event, breakpoint, fragment, half-edge, and cell
  reports: `arrange_line_segments`, `arrange_explicit_arcs`,
  `arrange_line_segments_with_explicit_arcs`, quadratic, cubic, and rational
  quadratic Bézier arrangements, and mixed line/curve arrangements
  (`arrange_line_segments_with_mixed_curves` and related functions).
- Line/Bézier intersection for quadratic, cubic, and rational quadratic
  curves, and curve arrangement cell graphs with loop-role classification.
- PCB rounded, oriented, orthogonal, convex, and obround pads, circular and
  obround board outlines, their clearance checks, and via fabrication policy
  certification (`certify_via_fabrication_policy`).
- Pythagorean-hodograph cubic and quintic curves with exact inverse-length
  certification, and quintic PH G1 smoothing.
- Feed and motion certification for acceleration- and jerk-limited feed time
  and corner lookahead limits. The planners `plan_lookahead_feed_schedule`,
  `plan_monotonic_jerk_transition`, `plan_jerk_feasible_lookahead_schedule`,
  and `plan_axis_projected_motion_limits` replay their proposals before
  returning them.
- Keepout-aware meanders, rectangular pocket link graphs, and staged
  rectangular rest-material graphs.
- Specctra arc wires, keepouts, and route rules, with route-rule width and
  trace-clearance audits.
- `PcbConstraintSet`, `ToolpathConstraintSet`, and residual-equation helpers
  for Hypersolve handoff.
- `dispatch-trace` feature, which forwards the exact-dispatch instrumentation
  of `hyperreal` and `hyperlimit`.

### Changed

- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.2.1 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.

### Removed

- The `provenance` module (`PathProvenance`, `SourceGrid`,
  `ConstructionStamp`, `PathSourceFormat`, `SourceLengthUnit`), the
  `with_provenance` and `provenance` methods, and the `provenance` fields of
  the facts and Specctra record types.
- The empty placeholder `hyperpath` binary target.

For 0.2.1 and earlier, see [crates.io](https://crates.io/crates/hyperpath/versions).
