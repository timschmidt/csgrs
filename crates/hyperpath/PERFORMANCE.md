# Reference and performance audit

This audit maps every academic paper and interchange specification in the
README reference section to HyperPath's exact path, routing, motion, and import
boundaries. Code changes were retained only when their behavior was covered and
their target benchmark improved statistically.

## Retained result

The 2026-07-28 immediate CAM result migration removed the public
`RectangularPocketPlan`, `RectangularBeadPlan`, and `RectangularSupportPlan`
staging types. Pocket rings and additive beads are now returned as concise
reports over borrowed source regions, while pocket and infill graphs accept
source geometry and process parameters directly. Graph results expose their
rings, beads, stop reason, and source facts without embedding a mutable plan.
The shared stop enum now uses the accurate `LimitReached` variant instead of
reporting `MaxRingsReached` for additive beads.

The affected paths were measured serially with 100 Criterion samples. The two
graph sentinels include both generation and graph construction on both sides:

| Benchmark | Before | After | Midpoint change |
| --- | ---: | ---: | ---: |
| `rectangular_pocket_offset_ring_schedule` | `[5.2305 us, 5.2417 us, 5.2549 us]` | `[5.1080 us, 5.1290 us, 5.1514 us]` | -2.15% |
| `rectangular_pocket_link_graph_immediate` | `[67.540 us, 67.853 us, 68.237 us]` | `[66.222 us, 66.448 us, 66.738 us]` | -2.07% |
| `rectangular_additive_bead_schedule` | `[15.906 us, 15.936 us, 15.969 us]` | `[15.684 us, 15.718 us, 15.759 us]` | -1.37% |
| `rectangular_serpentine_infill_graph_immediate` | `[32.817 us, 32.930 us, 33.076 us]` | `[32.863 us, 33.047 us, 33.292 us]` | +0.36% |
| `rectangular_support_footprint_plan` | `[1.1720 us, 1.1775 us, 1.1842 us]` | `[1.1767 us, 1.1805 us, 1.1848 us]` | +0.25% |

The infill before/after confidence intervals overlap and its final confirmation
remained within Criterion's noise threshold; support also showed no significant
change (`p = 0.11`). The other three paths improved. Historical `schedule` and
`plan` benchmark identifiers remain stable for comparison continuity and are
not public API names.

The 2026-07-28 immediate-construction API migration removed the redundant
`build_` prefix from 16 public CAM, routing, and tangent functions. These
functions already returned completed values, so the implementation stayed
unchanged. The later CAM migration above finished the lifecycle cleanup by
removing the plan carriers that this naming pass deliberately had not changed.
The affected paths were measured serially with 100 Criterion samples before
and after the rename:

| Benchmark | Before | After | Midpoint change |
| --- | ---: | ---: | ---: |
| `tangent_alignment_problem_construction` | `[278.35 ns, 279.55 ns, 280.90 ns]` | `[280.57 ns, 281.09 ns, 281.64 ns]` | +0.55% |
| `length_match_problem_construction` | `[154.37 ns, 155.35 ns, 156.46 ns]` | `[154.82 ns, 155.28 ns, 155.81 ns]` | -0.05% |
| `multi_detour_meander_exact_build` | `[12.895 us, 12.937 us, 12.985 us]` | `[12.925 us, 12.968 us, 13.014 us]` | +0.24% |
| `rectangular_pocket_offset_ring_schedule` | `[5.3898 us, 5.4204 us, 5.4551 us]` | `[5.3142 us, 5.3377 us, 5.3627 us]` | -1.53% |

No measured midpoint regressed by 2%. The historical
`multi_detour_meander_exact_build` benchmark identifier remains unchanged so
the saved comparison series stays continuous; it is not a public API.

The 2026-07-27 prepared-bounds API removal was gated on the segment carrier
with saved Criterion baselines. `line_segment_exact_tangent` measured
`[53.466 ns, 53.725 ns, 54.025 ns]` before and
`[53.229 ns, 53.312 ns, 53.399 ns]` after. Criterion found no change
(`p = 0.37`). `LinePathSegment` continues to retain exact bounds and
`Aabb2Facts`; callers now pass its immediate `bounds_min()`/`bounds_max()`
views directly to Hyperlimit predicates instead of constructing a public
prepared wrapper.

HyperSolve's problem-analysis API migration was also gated end to end here.
`g1_chain_hypersolve_certification` measured
`[5.0214 us, 5.0639 us, 5.1151 us]` against archived pre-change HyperPath and
HyperSolve sources, then `[5.0183 us, 5.0323 us, 5.0475 us]` after migration.
Criterion found no performance change (+0.23%, `p = 0.36`, 95% interval
-0.27% to +0.73%). HyperPath now calls the immediate `problem.analyze()`
surface; the inline accessor preserves the former direct construction cost.

The main retained optimization reuses a certified positive axis gap for
parallel axis-aligned PCB traces. Previously `check_trace_clearance` first ran
the full exact segment-intersection classifier, then independently computed the
axis gap. A positive exact gap already proves that the centerlines are
`Disjoint`, so the optimized path returns that same witness and proceeds
directly to the width/clearance comparison. Zero, unknown, crossing, and
unsupported cases still use the original intersection path.

On the same release Criterion profile, 30 samples of
`axis_aligned_trace_clearance_exact` changed from
`[881.69 ns, 889.34 ns, 900.77 ns]` to
`[379.76 ns, 381.79 ns, 384.68 ns]`. Criterion measured a 57.2% improvement
with `p < 0.05`. This path is also consumed by retained SPECCTRA route-rule
clearance audits.

The generated parallel-trace test now checks that every positive integer gap
retains both the exact gap and `SegmentIntersection::Disjoint`. The opt-in
`dispatch-trace` test proves that the optimized exact-rational fixture records
predicate/sign work with no approximation and no unknown fact.

## Source-by-source audit

### Yap, *Towards Exact Geometric Computation*

Yap's object/predicate separation is pervasive: source grids, construction
stamps, line/arc/Bezier/PH objects, candidate routes, and solver residuals are
retained separately from decisions. Arrangement and clearance APIs return
certified classes or explicit unknown results. The clearance optimization is
an application of the same discipline: reuse a stronger certified fact rather
than recompute a more general predicate. Dispatch tracing makes the no-silent-
approximation claim executable for the retained fast path.

### Lee, *An Algorithm for Path Connections and Its Applications*

Lee's grid-wave path connection is represented as candidate-search precedent,
not as an unclaimed full autorouter. HyperPath supplies exact equal-window and
caller-supplied meander candidates, obstacle/keepout side classification, and
length certification that a maze or graph router can consume. It deliberately
does not claim globally shortest or complete grid search.

### Hightower, *A Solution to Line-Routing Problems on the Continuous Plane*

Hightower motivates continuous rectilinear candidate channels rather than only
fixed-cell routes. Axis-aligned segments, exact detour amplitudes, arbitrary
placement windows, rectangular/circular/orthogonal keepouts, and exact offset
candidates cover that handoff. Final copper and board-edge predicates remain
separate from candidate placement, so an inexpensive route proposal cannot
silently become accepted geometry.

### Farouki and Sakkalis, *Pythagorean Hodographs*

Cubic and quintic PH carriers retain the polynomial hodograph, integrated
endpoint, exact polynomial arc length, partial length, endpoint derivatives,
and denominator-free inverse-length replay. The representative cached cubic
length benchmark is already about 15.1 ns; replacing the retained fact with
fresh polynomial evaluation would contradict the paper-derived advantage and
was not attempted as an optimization.

### Erkorkmaz and Altintas, *High Speed CNC System Design. Part I*

The motion layer exposes constant-jerk span identities, endpoint and midpoint
feed constraints, acceleration and jerk bounds, multi-phase length sums, and
feed/acceleration continuity. It treats controller schedules as proposals for
exact replay rather than claiming to implement the paper's full interpolation,
Newton parameter update, resampling, or servo loop. The representative jerk
schedule baseline is about 30.5 microseconds.

### Ucamco, *Gerber Layer Format Specification*

Import adapters should preserve Gerber's integer coordinate/format semantics as exact
rationals before geometry construction. HyperPath does not yet claim a complete Gerber aperture,
region, attribute, or image parser; those belong in an import adapter that
emits these exact path carriers.

### KiCad Project, *KiCad PCB File Format*

KiCad's board format uses S-expressions and textual coordinates for tracks,
vias, arcs, layers, nets, and stack-up data. Import adapters should convert decimal
coordinate text directly through HyperReal's exact parser, avoiding an `f64` boundary. A complete
`.kicad_pcb` parser is still outside this crate's current claim.

### Cadence, *SPECCTRA Design Language Reference*

The crate implements a deliberately typed DSN/SES-style subset: borrowed
S-expression tokens, quoted aliases and comments, fixed-grid wires, multi-point
paths, vias, arcs, keepouts, rules, route envelopes, canonical serialization,
and exact lifting into routing carriers. The 256-wire parse baseline is roughly
67--69 microseconds. Unsupported language groups remain explicit rather than
being mistaken for a complete Cadence implementation.

## Rejected trials

- Converting rectangular obstacles directly through a specialized meander
  predicate path measured 25.36 microseconds versus a 25.21 microsecond
  baseline. Criterion found no change (`p = 0.54`), so the generalized keepout
  path was restored.
- Reserving the SPECCTRA lexer's wide `Cow<str>` token vector from input length
  regressed one-wire parsing by 3.2% and 256-wire parsing by 13.4%. The original
  incremental vector growth was restored.
- Sharing one precomputed maximum-acceleration square between the two jerk-span
  endpoint constraints regressed schedule replay by 6.5% (32.4 versus 30.5
  microseconds). The original small exact multiplications were restored.

## Representative baselines

Before changes, targeted release measurements were 269.5 ns for fixed-grid
lifting, 889 ns for axis clearance, 25.2 microseconds for meander placement,
15.1 ns for cached cubic-PH length, 30.5 microseconds for jerk replay, and about
67 microseconds for the 256-record SPECCTRA parser. These fixtures span the
eight references without making every optimization trial run the entire large
benchmark suite.

## Validation

The retained changes are checked by the full 422-test default suite, the
all-feature dispatch test, all benchmark smoke targets, generated property
tests, formatting, Clippy with warnings denied, all-target checking, rustdoc
with warnings denied, examples, doc tests, and focused Criterion measurements.
