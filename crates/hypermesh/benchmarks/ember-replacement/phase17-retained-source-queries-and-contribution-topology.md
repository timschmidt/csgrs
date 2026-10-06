# Phase 17/18 checkpoint: retained source queries and contribution topology

Date: 2026-08-05

Implementation parent: Hypermesh `d17f1ec2`

Implementation commits: Hypermesh `11b61be8` and `1b253caf`

Companion data: `phase17-retained-source-queries-and-contribution-topology.toml`

Status: correctness and performance checkpoint; Phases 17 and 18 remain open

## Outcome

The one production Boolean engine now retains Hyperreal's compact certified
rational point-query facts at the source-position owner and preserves every
authored contribution through coincident surface-cell assembly.

The retained query schedule is representation-driven. It builds one 32-byte
`RationalLinearForm4Query` for each referenced exact-rational source point,
uses a dense row when every position is referenced, and otherwise stores a
compact `u32` indirection. If any referenced point cannot construct the
certified query, the whole schedule declines atomically and all predicates use
the unchanged exact cascade. A filter miss also reaches that same cascade.
Neither policy semantics nor accepted topology depend on the filter result.

Coincident geometric facets still produce one output facet, but no longer
collapse their authored sheets before radial topology. Each contribution owns
front/back side nodes and its own checked winding transition. Equal rays form
ordered zero-angle stacks; the order is derived from retained source-component,
source-face, sheet, and angular crossing facts. The geometric outer sides are
recovered from directed sheet balance and checked against the aggregate
transition. Net-zero bundles may form internal sheet cycles, while malformed
branching or inconsistent transitions fail with typed arrangement errors.

Retained layer continuity joins an intermediate coincident stratum only when:

1. the source faces on both authored components prove manifold continuation;
2. the geometric ray groups are adjacent; and
3. the cyclic route has zero signed crossings for every retained authored
   source component.

The third invariant was found by the sanitizer corpus. Without it, a third
surface could cross between two locally coincident sheets and an optional
compression union could assign two operand windings to one cell. The fix is a
general allocation-free radial balance test. It contains no coordinate,
fixture, triangle-count, operation, result, policy, or competitor branch.

Structural construction identity also avoids recomputing radial equality,
angular-half, and comparison predicates when two uses retain the same opposite
arrangement point. A one-entry comparison cache was measured, returned only a
0.06--0.16% instruction gain, and was removed rather than obscuring the clean
radial algorithm.

There is no compatibility shim, dual Boolean engine, repair retry, hidden pass
limit, or benchmark-shaped shortcut.

## Exactness and policy

- `hyperreal::Real` remains the sole accepted construction scalar.
- `STRICT` never consumes an approximate terminal.
- `APPROXIMATE_512` can terminate only through Hyperlimit's 512-bit terminal.
- One operation-local `DecisionContext` aggregates terminal consumption into
  `MeshCertainty`.
- Retained rational queries are certified filters only. Inconclusive or
  unavailable facts decline to the same exact-rational/`Real` path.
- Every rational regression and large row in this checkpoint is `Certified`
  under both policies; therefore no approximate terminal was consumed.

## Permanent path-completeness corpus

The monotonic fixture registry now contains 59 cases. Two new deterministic
records cover contribution-level coincidence:

- `corner_coincident_same_operand_box_tetra` combines overlapping same-operand
  box/tetra shells with a cutter. Three source orderings, both operand orders,
  all five Boolean expressions, and both policies produce closed certified
  boundaries
  with exact signed-six-volumes `542/3`, `50/3`, `110/3`, `382/3`, and `164`.
- `subdivided_face_coincident_shell_stack` is the readable reduction of the
  newly exposed fuzz path. A subdivided wide box and an adjoining box occupy
  one operand while a second operand is a coincident subset. Two source
  orderings, both operand orders, all five Boolean expressions, and both
  policies produce closed certified results with signed-six-volumes `114`, `6`,
  `108`, `0`, and `108`.

Unit coverage separately proves directed sheet-balance recovery, canceling
cycles, malformed branching, inconsistent transitions, same-operand
opposite-side cancellation, identical coincident shells, and the retained
component-balance rule.

## Validation and sanitizer

The final matrix passes:

- `cargo test --all-features`: 214 executed tests, 7 documented manual/opt-in
  ignores, no failures;
- the public dense crossing arrangement under both policies;
- `cargo check --no-default-features`;
- `cargo clippy --all-targets --all-features -- -D warnings`;
- `cargo doc --all-features --no-deps`;
- `cargo fmt --all -- --check` and `git diff --check`;
- AddressSanitizer/libFuzzer `boolean_pipeline`: all 1,973 input corpus files
  replayed and 4,113 executions completed in 31 seconds, with 493 MiB maximum
  sanitizer RSS and no crash.

LeakSanitizer alone remains disabled with `ASAN_OPTIONS=detect_leaks=0` because
the managed environment prevents its final ptrace scan. AddressSanitizer and
libFuzzer remain enabled. Generated artifact/corpus directories are not
committed; the failure is represented by the readable regression and manifest
record above.

## Deterministic retained-work controls

The table reports retired work for 1,000 strict arrangements that materialize
union, intersection, difference, and reverse difference together. Parent and
current use the same Rust 1.97 toolchain and CPU 11. Counters vary below 0.03%
within a row; wall clocks are frequency-sensitive and are not used to infer
algorithmic movement.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 2,925,008,334 | 2,877,407,910 | -1.627% | 496,036,275 | 487,434,463 | -1.734% |
| overlapping boxes | 3,811,847,729 | 3,649,566,101 | -4.257% | 648,564,823 | 620,146,509 | -4.382% |
| affine boxes | 6,897,183,448 | 6,983,652,745 | +1.254% | 1,180,053,625 | 1,189,397,519 | +0.792% |
| identical boxes | 3,908,061,655 | 3,765,590,748 | -3.646% | 665,157,793 | 639,603,228 | -3.842% |

The retained source query wins every non-affine control above. Complete sheet
topology activates heavily on identical geometry, where structural point-ID
reuse more than repays its added bookkeeping. The affine row exposes the
remaining 1.254% general constant-factor cost; it remains open rather than
being hidden behind a special path.

The 10,000-iteration overlapping-box control reports 357,246 ns per strict
arrangement and 361,488 ns per approximate-512 arrangement, both certified and
with identical 28-vertex, 48/24/40/32-triangle outputs.

## Current CGAL boundary

CGAL 6.0.3 EPECK was rerun from freshly exported exact-rational OFF inputs.
Every output is valid, closed, structurally valid, and matches the exact
triangle/volume oracle. A 1,000-iteration retained-input sample reports:

| Fixture | CGAL EPECK median | Adjacent Hypermesh strict sample | Ratio |
| --- | ---: | ---: | ---: |
| crossing octahedra | 117,702 ns | 302,099 ns | 2.567x |
| affine boxes | 360,265 ns | 720,297 ns | 1.999x |

These clock samples are frequency-sensitive; the deterministic counters above
are the regression authority. The established same-day 63-process/paired
confidence rows remain in `phase17-current-cgal-small-controls.{md,toml}`.
Every current small competitive runtime gap remains open.

The established exact overlapping-box CGAL copy-outside median is 130,086.5
ns, making the 10,000-iteration Hypermesh strict sample 2.746x slower. This is
a substantial improvement over the earlier 7.94x checkpoint, but not parity.

## Full-resolution historical hard case

The checksum-pinned 11,894-by-11,894-triangle rotated YeahRight intersection
returns the exact empty result with `Certified` certainty under both policies:

| Policy | Timed Boolean | Maximum process RSS | Kernel incremental heap |
| --- | ---: | ---: | ---: |
| `STRICT` | 964.855 ms | 71,440 KiB | 54,471,340 B |
| `APPROXIMATE_512` | 950.168 ms | 71,424 KiB | 54,471,340 B |

The strict three-run counter mean is 10,355,874,218 instructions,
1,764,194,965 branches, and 23,599,126 branch misses. Against the exact parent,
instructions/branches fall 3.661%/3.465%. Both policies allocate the same
10,804,022 blocks and retain the same empty output.

Historical EMBER required 3,312.66 seconds and 329,352 KiB. The current
approximate-512 row is 3,486x faster and lowers maximum RSS 78.31%. Historical
CGAL EPECK required 0.09 seconds and 15,516 KiB, so Hypermesh remains 10.56x
slower and 4.60x larger on that boundary. Those deficits remain open.

## Large-fixture heap

These are global-allocator requested-payload measurements. The incremental
column subtracts retained decoded input from the exact Boolean peak. Both
policies produce byte-identical rows for the box, dense-coplanar, and full
YeahRight fixtures.

| Fixture | Input triangles | Parent incremental peak | Current incremental peak | Change | Current allocations | Current added bytes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| general subdivided boxes | 6,144 | 12,033,530 B | 12,132,042 B | +0.819% | 35,732 | 18,182,830 B |
| dense coplanar boxes 32 | 24,576 | 55,365,778 B | 55,759,202 B | +0.711% | 926,777 | 221,799,367 B |
| transverse self-PWN clusters 512 | 4,100 | 11,320,356 B | 11,451,636 B | +1.160% | 197,043 | 31,255,206 B |
| full rotated YeahRight | 23,788 | n/a | 54,471,340 B | n/a | 10,804,022 | 611,301,578 B |

The 32-byte retained point query explains most peak movement on source-heavy
rational fixtures. It is operation-local and buys the deterministic work
reductions above. Dense-coplanar allocation calls rise 4.66%; reducing that
traffic without losing the query wins is an open target.

## Source and linked size

Production `src` grows from 19,314 to 20,300 Tokei code lines (+986, +5.11%).
The linked change is much smaller because generic checks and tests do not all
survive release linking:

| Consumer/profile | Parent native text | Current native text | Change | Parent optimized WASM | Current optimized WASM | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| default release/general | 2,008,250 | 2,025,170 | +0.843% | 1,424,419 | 1,442,248 | +1.252% |
| all-feature release/general | 2,144,019 | 2,160,379 | +0.763% | 1,499,118 | 1,517,400 | +1.220% |
| default size/general | 1,099,983 | 1,116,183 | +1.473% | 687,434 | 700,085 | +1.840% |
| all-feature size/general | 1,102,319 | 1,118,247 | +1.445% | 687,245 | 700,018 | +1.859% |

Immediate-consumer rows move by comparable amounts. This is a bounded but
material completeness cost; source and linked recovery remain Phase 17 work.

## Call graph and removal audit

The workspace call-graph utility was run over exactly Hyperreal,
Hyperlattice, Hyperlimit, Hypertri, and Hypermesh, excluding concurrently owned
Hypercurve/HyperSolve:

- production: 15,453 function nodes / 25,830 edges;
- tests, examples, benches, and fuzz included: 21,856 nodes / 35,342 edges;
- 49 direct static Hypermesh/Hypertri-to-Hyperlimit predicate boundaries;
- one `build_surface_arrangement -> assemble_surface_cells` production route;
- direct edges from radial assembly to the retained component-balance rule;
- direct source-query classification edges to the unchanged predicate cascade;
- zero exact EMBER, `segment_trace`, or `local_bsp` nodes.

A source search over production, manifest, and README also finds none of the
removed engine identifiers. Static import resolution is approximate, so the
graph is navigation/runtime evidence rather than a substitute for the passing
dispatch, integration, and sanitizer gates.

## Open work

This checkpoint does not declare Phase 17 or Phase 18 complete. The following
remain explicit:

- every measured CGAL runtime gap and the full-case RSS gap;
- the small affine instruction regression;
- dense source-query allocation traffic and the 0.7--1.2% large-heap growth;
- source/native/WASM recovery after the correctness structure stabilizes;
- broader real-world/generated pathology admission and fuzz-source audit;
- full current CGAL confidence/RSS matrices and final requirement-by-requirement
  release audit.

Any subsequent optimization must preserve the contribution-level topology,
retained component-balance invariant, one exact decline path, both policy
semantics, and the prohibition on fixture- or competitor-aware dispatch.
