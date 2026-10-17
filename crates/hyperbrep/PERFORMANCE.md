# HyperBREP Performance Contract

## 2026-10-16: retain positive builder admission proofs

Kept after the [source-bound qualification](docs/evidence/2026-10-16-builder-certificates/README.md).
The builder reuses successful strict admission proofs, retries missing families,
and preserves complete validated-transfer semantics. No validation, output
materialization, policy, representation or artifact migration is changed.

All 167 original cases across 17 corpora preserve their exact receipts in three
alternating CPU-0 pairs (102 processes). Four fresh/reused circular-import cases
have candidate/baseline time ratios 0.721784–0.778750 and fewer allocator
requests. This local result is not a whole-kernel or controlled-host speedup.
The sole row above the 10% review marker is the unchanged arc executable's
structural replay, 5.85585 to 6.48705 ns (1.107790). Both executable hashes are
identical; retain the observation without attributing it to this BREP change
or claiming an all-row timing pass. Other slower rows remain in the complete
report, including analytic export ratios up to 1.051813.

Eleven rows request eight additional bytes per operation (sixteen for the two
two-body cases), with no additional allocation calls. Across all corpora,
median process peak-RSS deltas range from -480 to +372 KiB. Native harness size
deltas range from -424 to +17,920 bytes; the 83-group WASM probe grows 12,198
bytes. These costs are accepted for the measured reduction in repeated proof
work. Allocator requests and process RSS do not establish retained-heap bounds;
harness sizes are not distribution sizes. Earlier campaigns and unresolved
regressions remain historical evidence, not waived or arithmetically combined
with these ratios. All 28 work-package gates remain open or in progress.

## 2026-10-15: direct circular STEP composition and scalar certificates

The [source-bound evidence](docs/evidence/2026-10-15-step-circles/README.md)
qualifies exact direct circle/cylinder import through native splitting, archive
replay, reflection, disconnected Boolean composition and empty continuation.
Hyperreal owns the added quarter-turn/reconstructed-angle proof; topology uses
ordinary native validation. This is a completeness extension, not an optimization.

The initial three alternating CPU-0 pairs preserve all receipts but exceed the
10% gate in 53 of 64 native rows, with median/max ratios 1.256921/1.340681.
Raw processes show a common timing shift in both binaries. One predefined
complete replication has median/max 0.992343/1.036984; pooling all six pairs
has 0.997557/1.053811. No observation is dropped and no cause or significance is
inferred. The default-feature native corpus omits the same two optional
tessellation rows on both sides.

The complete 12-row default arc corpus retains 256-arc inputs and unchanged
iteration counts. Initial/repeat/pooled median ratios are
0.997553/1.011208/0.998373. The pooled cached 256-arc query costs 39.2 to
43.25 ns (**1.103316**), exceeding the default budget. The repeat's cold row
also exceeds the budget (1.104945); its pooled ratio is 0.991284. Keep these
failures explicit. Retain the new exact domain as a feature baseline under the
contract's semantic exception; controlled-host and all-cohort timing admission
remain open, including the older archive and closed-edge observations.

All five closed-edge rows retain their exact measures, source/topology facts,
complete JSON bytes and allocator requests across six pairs. Their pooled
median/max ratios are 0.984569/1.046618. Native/arc/closed-edge executables grow
5,424/18,016/5,408 bytes. Process peak RSS spans 21,680–22,168 / 21,524–21,968 KiB
for native baseline/candidate, 29,256–29,976 / 29,448–29,796 for arc, and
9,592–9,912 / 9,516–9,844 for closed edges. RSS is separate from Rust allocation
requests and is not an internal retention bound.

The broader scalar fallback is rejected with its source and raw evidence:
frustum replay cost 1.384127 times baseline and retained arc-witness replay
1.653519. The narrower shared recognizer preserves all 64/12/5 native receipts,
all 6/7 existing advanced/faceted receipts, and the focused 134/124 scalar probes.
New circular import costs and full materialization boundaries are documented in
[HyperBRep IO](../hyperbrep-io/PERFORMANCE.md#direct-analytic-circles-and-cylinders--2026-10-15).
All work-package, complete resource, release, formal and coverage gates remain open.

## 2026-10-15: native closed circles and ellipses

Kept for exact closed-edge and full-circle contour completeness. The
[evidence](docs/evidence/2026-10-15-closed-edges/README.md) starts from
`d60bd1cf4`, after the upstream acceptance integration and BodySet migration.
All 64 existing default-feature BREP receipts match in three alternating CPU-0
pairs. The median of case median time ratios is **1.020399**, with range
**0.993175–1.041137**. The largest row is linear tensor/plane iso-curve
intersection, 10.633 to 11.070 microseconds. These are observed costs, not a
speedup claim. Both binaries omit the same two optional tessellation rows.

The complete 12-row default arc harness also retains its existing observations,
including 256-arc inputs and ten large-case iterations. Its median ratio is
1.003615; the largest is **1.118080** for structural-fact replay, 6.462 to
7.225 nanoseconds. This exceeds the default 10% cohort budget and is retained
explicitly, not discarded as a timer outlier. The semantic retention reason is
native full-circle composition through the existing exact region carrier;
this is not an optimization or an all-cohort timing-gate pass. The final
candidate is the new feature baseline; its predecessor and all samples remain
available. Controlled-host performance qualification remains open, including
the earlier independent archive-admission issue. No statistical significance
or cause for the subnanosecond replay difference is inferred.

The new `closed_edges` benchmark measures 100 completed operations per row in
each of three fresh processes. Input/expected-result preparation is outside
timing; exact measures, classification, output counts, JSON materialization
and complete archive equality are inside. Split operations reuse their input
models. These are absolute costs because the old closed-edge constructor failed.

| Completed operation | Median | Rust allocation calls | Requested bytes | Archive bytes |
| --- | ---: | ---: | ---: | ---: |
| Construct | 287.738 µs | 757.01 | 133,250.76 | 12,287 |
| Replay | 452.817 µs | 1,064 | 166,429 | 12,287 |
| Split circular edge | 175.748 µs | 397 | 106,426 | 14,328 |
| Split seam edge | 174.660 µs | 401 | 108,074 | 14,211 |
| Replay reversed/shifted chart | 503.273 µs | 1,718 | 218,925 | 12,453 |

Calls and bytes are per-completion averages of allocation/reallocation requests,
not retained heap; fractional values include one-time work within the loop.
Archive sizes, fingerprints and allocation totals match all three processes.
The BREP executable grows 1,136 bytes to 24,890,392; the arc executable shrinks
128 bytes to 16,714,312. The new benchmark executable is 22,534,448 bytes.
BREP process peak RSS ranges from 21,756–21,980 KiB before and 21,672–22,040
after; the new benchmark uses 9,564–9,920 KiB. These do not establish memory
bounds or significant savings. Existing cost rows do not isolate the added
finite-cylinder face bound. Cold/warm correctness is tested, but no separate
cold allocation/timing study or new general resource contract is claimed.

A provisional whole-model bound fallback passed the same receipts and focused
tests. It was replaced by the more local finite-cylinder face-bound proof;
its source, controls and all measurements are preserved separately and excluded
from final-source statistics. New source geometry, policies, harness resolutions
and iteration counts were not weakened to obtain completion.

## 2026-10-15: shared archive structure admission

Kept as partial WP13 resource-contract work. The
[qualification](docs/evidence/2026-10-15-archive-structure/README.md) compares
against `155f15e4f` in three alternating process pairs pinned to CPU 0.
All 66 native receipts match. The median of case median timing ratios is
0.995768; the largest is 1.073196, within the default 10% budget. Three further
pairs preserve eight spline replay receipts with ratios 0.999766–1.014925.
This comparison does not close the prior archive-input-admission timing issue.

The allocation-instrumented cuboid decode/encode case changes from 379.930 to
380.386 microseconds (1.001200 ratio). Its 695 allocation calls and 216,464
requested allocator bytes per iteration are unchanged. The three existing
syntax rejections remain allocation-free and within the timing budget.
Four new structural rejections, each measured for 10,000 iterations per process,
have these medians and Rust allocator requests per iteration:

| Rejected allowance | Median | Allocation calls | Requested bytes |
| --- | ---: | ---: | ---: |
| Records (100 observed, 99 allowed) | 180.694 µs | 7 | 214 |
| Degree (300 observed, 256 allowed) | 1.082 µs | 4 | 126 |
| Geometry elements (128 observed, 64 allowed) | 9.104 µs | 5 | 166 |
| Topology references (128 observed, 64 allowed) | 8.556 µs | 4 | 126 |

Fixtures differ; the rows are not a relative cost comparison of the resources.
Error construction and metadata parsing can allocate. Counts include Rust
allocation/reallocation requests, not foreign allocations, retained heap or
native stack. The kernel executable grows 224,768 bytes to 24,997,880;
median native process peak RSS is 21,796 versus 22,008 KiB. No significance
or universal resource bound is inferred.

An initial implementation retained separate version and metadata passes.
Its four established allocation-instrumented cohorts exceeded the 10% gate;
cuboid decode/encode had ratio 1.529619. That prototype was superseded by shared
version/metadata admission on the successful path. Every initial observation
and the source difference remain in the evidence bundle; none contributes to
the final-source cost statistics. Numeric reconstruction and iterative WASM-safe
decoding remain open. This is a resource-contract extension, not a speedup claim.

## 2026-10-15: explicit archive input admission

Kept as a WP13 resource-contract extension with an architectural cost
exception. **Pooled native timing qualification remains unresolved.** The
[qualification and decision](docs/evidence/2026-10-15-archive-admission/README.md)
retain every sample and explain why an iterative pre-deserialization guard is
required. No optimization or full regression-budget pass is claimed.

Against `11078836b`, six alternating pinned pairs preserve all 66 native
receipts. First-three-pair median/largest pooled case ratios are
1.001281/1.265185; repeat-three-pair ratios are 1.003303/1.127975. Combining all
six yields 1.243534/1.340618, exceeding the default 10% gate. A diagnostic median
of each case's six within-pair ratios yields 1.003536/1.050614; it does not
replace the failed pooled gate. Pairs 3 and 5 show roughly 26% harness-wide
median differences, whose cause has not been established. Controlled-host
requalification and investigation of stable failures remain required before
release performance admission.

Three additional paired runs preserve eight existing spline replay receipts.
The seven replay/encode cohorts grow 2.04–4.01%; the short prepared injectivity
query has equal 86 ns medians. The kernel executable grows 1,872 bytes to
24,773,112. Median process peak RSS across all six native trials is 22,046 KiB
for baseline and 21,748 KiB for candidate. No significance or general memory
bound is inferred.

The new `archive_limits` bench records 256 native cuboid decode/encode
iterations and 100,000 iterations per rejection cohort in each of three
processes. Native decode/encode has median 470.637 µs, 695 Rust allocator calls
and 216,464 requested bytes per iteration. Byte, depth and token rejection
medians are 25, 117 and 213 ns; all have zero Rust allocation calls and requested
bytes. Instrumentation counts Rust allocation/reallocation requests, not foreign
allocations or retained/peak storage. Sources are prepared outside timing.
Input syntax limits do not bound scalar reconstruction or geometric replay.
The new guarded executable is retained as the next feature baseline while the
frozen unguarded baseline and failed pooled timing evidence remain available.

## 2026-10-15: trimmed spline replay and complete-image injectivity

Kept for exact native/selected spline trim replay. The
[qualification](docs/evidence/2026-10-15-spline-replay/README.md) compares against
`7091576d8` in three alternating process pairs pinned to CPU 0. All 66 native
receipts retain case order, iterations, units and checksums. The median of
per-case median timing ratios is 0.989699; the largest is 1.073174 for
`analytic_kernel/homothetic_loft_build_measure_classify`
(1,116,308 to 1,197,993 ns), within the 10% cohort budget. Executable size
increases 10,704 bytes to 24,771,240. Median process peak RSS is 21,764 KiB for
the baseline and 21,712 KiB for the candidate. These descriptive samples do
not establish a speedup or significant memory difference.

The `spline_replay_cost` harness measures 100 iterations in each of three
processes. Authoring and initial root isolation are outside timing. Each replay
parses JSON, reconstructs proofs, checks topology/selected-coordinate counts,
and encodes byte-identical JSON. Selected cases contain one native cut and two
algebraic cuts.

| Cohort | Median per iteration | Archive bytes |
| --- | ---: | ---: |
| Full rational Bezier sheet | 379.494 µs | 9,206 |
| Native rational Bezier trim | 884.277 µs | 10,892 |
| Selected rational Bezier trims | 1.160422 ms | 14,141 |
| Selected rational quadratic trims | 1.133407 ms | 14,175 |
| Selected single-span NURBS trims | 1.426204 ms | 15,428 |
| Selected multi-span NURBS trims | 1.616475 ms | 17,047 |
| Eight multi-span replay/encode cycles | 12.824790 ms | 17,047 input |
| Prepared two-span injectivity query | 72 ns | — |

Each selected archive retains one root and one pcurve source; rational Bezier
variants have three chart images and NURBS variants two. Eight-cycle timing is
a fixed composition case, not an asymptotic bound. The prepared query is a
short sample with existing span caches warm; it is not a cold/warm comparison.
No per-query allocation count, statistical significance, external-kernel
comparison or general growth bound is claimed.

## 2026-10-15: independent selected solid proofs

Kept for exact selected-solid replay and composition. The
[qualification](docs/evidence/2026-10-15-selected-solids/README.md) compares
against `340d51bce` in three alternating process pairs pinned to CPU 0.
All 66 native receipts preserve case order, iteration count, unit and checksum.
The median of per-case median timing ratios is 1.011113; the largest is 1.093023
for `rational_bilinear_tensor_plane_complete_containment` (903 to 987 ns).
Executable size increases 29,640 bytes to 24,760,536. Median process peak RSS
is 21,668 KiB for the baseline and 21,424 KiB for the candidate. These descriptive
samples establish neither a speedup nor a significant memory difference.

`selected_solids` measures independent JSON parsing/proof reconstruction,
exact volume and two point classifications. Sources and selected roots are
constructed outside timing. Three processes run 100 iterations per cohort:

| Cohort | Median per iteration |
| --- | ---: |
| Native cuboid replay and queries | 868.034 µs |
| Cuboid with one selected edge split | 1.091859 ms |
| Cuboid with all twelve edges split | 2.705168 ms |
| Cylinder with all four axial edges split | 2.127920 ms |
| All-edge cuboid reload, split, reload and queries | 6.093168 ms |
| Eight complete all-edge cuboid reload/encode/query cycles | 24.682930 ms |

The native, one-edge, all-edge and cylinder archives are respectively 33,834,
36,397, 47,138 and 46,999 bytes. Each selected archive retains one root and one
chart image; pcurve source counts are 2, 12 and 8. Repeated replay preserves
archive bytes and topology counts. These fixed composition cases are not
allocation counts or bounds on arbitrary operation chains.

The receipt checker rejected an initial runner mistake that selected the
integration-test executable sharing the benchmark's name. The corrected runner
selects Cargo artifacts by target kind. Initial feature metadata also counted
all parameter records under a `roots` label; the final harness reports roots
and chart images separately and its three processes were rerun. Both setup
records are retained in the qualification bundle. No invalid process contributes
to the reported native comparison or final feature medians.

## 2026-10-15: selected archive replay

[Evidence and commands](docs/evidence/2026-10-15-selected-archive/README.md)
cover format 9 graph persistence and independent sheet validation. Three
alternating pinned process pairs preserve all 66 native benchmark receipts
against `3426a3a43`. The median of case median timing ratios is **1.00163**;
the largest sampled ratio is **1.03629**. The benchmark executable grows
**205,040 bytes**; median process peak RSS changes from **21,636 to 22,020 KiB**.
This is a correctness/completeness extension, not a claimed optimization.
Format 9 intentionally changes archive bytes; its own round trips are checked
for byte stability. Existing geometric benchmark outputs retain their checksums.

The new `selected_archive` benchmark constructs sources outside timing and
checks exact area, selected-coordinate retention and topology counts during
replay. Medians across three processes, each with 100 iterations:

| Operation | Median per iteration | Archive bytes |
| --- | ---: | ---: |
| Native sheet encode | 27.472 µs | 8,163 |
| Native sheet parse, validate and area | 109.994 µs | 8,163 |
| Selected sheet encode | 44.462 µs | 9,681 |
| Selected sheet parse, validate and area | 218.026 µs | 9,681 |
| Selected sheet plus 16 cuts encode | 133.211 µs | 26,923 |
| Selected sheet plus 16 cuts replay and area | 587.420 µs | 26,923 |
| Selected replay, another split and area | 258.493 µs | 9,681 input |

The 5-edge and 21-edge archives both retain one selected root and one authored
pcurve source. This fixed composition fixture checks sharing; it is not an
asymptotic time/memory bound. RSS is not an allocation count. Selected solid
revalidation and bounded hostile-input decoding remain open.


Correctness is exact. Performance determines which certified route reaches the
answer, not what answer is accepted.

## Architecture

- Typed integer IDs provide O(1) arena lookup.
- `Model` retains vertex/edge, edge/use, use/wire, wire/face, face/shell, and
  shell/solid ownership indexes.
- Geometry is immutable and clone-shared through `Arc`.
- Expensive immutable geometry facts use thread-safe once caches.
- Ordinary model queries do not rebuild identifier maps.
- Broad phases may eliminate work only with conservative certified bounds.
- Numerical solvers may propose candidates only; exact replay decides topology.

## Required benchmark cohorts

Every established operation will be measured for:

- small integers and rationals;
- large rationals;
- dyadic values lifted from binary interchange;
- algebraic values;
- lazy/computable values;
- close but unequal values;
- exact degeneracies;
- mixed scalar representations;
- cold and warm caches;
- increasing topology and candidate-pair counts.

Operations include geometry construction/evaluation, model commit, adjacency,
classification, intersections, editing, Booleans, area, volume, and derived
tessellation as those capabilities land.

## Instrumentation

Benchmarks should retain counts for:

- broad-phase candidates and certified rejections;
- exact predicate calls and unresolved outcomes;
- refinement and solver replay;
- cache hits and misses;
- allocations;
- expression-size or exact-arithmetic growth where observable without changing
  semantics.

The default regression budget is 10% for an established cohort. A deliberate
regression requires a recorded semantic or architectural reason and a new
baseline.

## Selected topology — 2026-10-15

Kept for exact selected vertices, trims, shared-edge subdivision and pcurve
composition. The [qualification](docs/evidence/2026-10-15-selected-topology/README.md)
records ten public regressions, four rejected mutants and all native/WASM-build
checks. The first native-mapping prototype exceeded the default budget in
three editing cohorts and was replaced with direct stored-scalar formulas.
Selected charts continue to use Hypercurve's existing algebraic authority.

All 66 existing receipts agree across three alternating process pairs. The
median of case median ratios is **1.011410**, with a maximum of **1.058066**;
all cohorts stay within 10%. Binary size grows **67,040 bytes**; median process
peak RSS is **21,472 → 21,808 KiB**. The
[comparison](docs/evidence/2026-10-15-selected-topology/comparison.json) and
[rejected prototype](docs/evidence/2026-10-15-selected-topology/prototype/comparison.json)
retain every sample. No allocation-count or statistical significance is claimed.

The new four-row topology harness measures native/selected splits, sixteen
composed splits and reflection/inverse with exact checks. Prepared roots and
source construction are outside timing. Median costs are 148.213,
161.454, 1931.441 and 337.269 µs respectively; these are
feature costs, not speedups. Selected archive graphs, complete work bounds,
large-chain growth and external comparisons remain open.

## Retained spatial points — 2026-10-15

Kept for exact evaluation and composition of selected line, Bézier and NURBS
images. A point shares immutable source geometry and one selected parameter.
XY/XZ carrier and image preparation are independently lazy; identity proofs
avoid projection work. Affine composition transforms the native controls and
keeps the selector, without adding point-wrapper depth. The tests cover source
lifetime, concurrent readers and retry after stronger scalar evidence. This
is a capability extension, not a claimed optimization.

The [qualification record](docs/evidence/2026-10-15-retained-spatial-points/manifest.json)
binds sources, commands, four rejected mutants, executables and raw logs.
The kernel harness has a direct public-API migration at six call sites;
[the audit](docs/evidence/2026-10-15-retained-spatial-points/migration-audit.json)
verifies unchanged fixtures, iterations and assertions. It is not byte-identical
source. All **66 receipts** agree across three alternating process pairs.
The median of case median ratios is **1.00352**; the largest is **1.07127** for
`boolean_kernel/disjoint_sphere_union_remap_validate`, within the 10% cohort
budget. Binary size increases by **25,232 bytes**. Whole-process median peak
RSS is **21,468 → 21,736 KiB**. These descriptive samples establish neither
allocation counts nor statistical significance. All samples and rows remain in
[comparison.json](docs/evidence/2026-10-15-retained-spatial-points/comparison.json).

The new `retained_points` harness runs 1,000 exact-assertion iterations per row.
Three-run median costs are:

| Scenario | Median per iteration |
| --- | ---: |
| Authored scalar line evaluation and stored-coordinate check | 735 ns |
| Fresh selected line carrier/image and six coordinate order checks | 76.804 µs |
| Shared point identity | 17 ns |
| Independent weighted selected images, warm coincidence | 989 ns |
| Fresh NURBS image on warm carrier, authored knot chart and origin check | 75.174 µs |
| Warm bounds plus six enclosure checks | 115.144 µs |
| Affine transform/inverse pair plus exact coincidence | 30.750 µs |

Root isolation is outside these timings; even the cold-carrier row shares a
prepared selector. These are feature costs, not baseline speedups. The four
retained-parameter and four migrated work-budget receipts also agree in three
runs each. See
[feature-cost.json](docs/evidence/2026-10-15-retained-spatial-points/feature-cost.json).

## Retained inverse parameters — 2026-10-15

Kept for exact inverse-query completeness. `Parameter` retains admitted roots
without requiring a stored scalar; NURBS chart transport reuses Hypercurve's
existing implementation. Represented Bézier queries keep the previous 3D
evaluation check. A second projection is built only when a selected candidate
needs a nonconstant third-coordinate proof, and is shared across that query's
candidates. This is a capability extension, not a claimed optimization.

The [qualification record](docs/evidence/2026-10-15-retained-inverse-parameters/manifest.json)
binds sources, commands, three rejected mutants, executable hashes and logs.
All 66 unchanged kernel receipts agree across three alternating process pairs.
The median of case median ratios is **0.97391**; the largest is **1.05369** for
`spline_kernel/nurbs_planar_face_build_exact_area`, within the 10% cohort budget.
Binary size increases by **18,096 bytes**. Whole-process median peak RSS is
**21,712 → 21,784 KiB**. These are descriptive samples, without statistical or
allocation-count claims. Every row and sample remains in
[comparison.json](docs/evidence/2026-10-15-retained-inverse-parameters/comparison.json).

The new `retained_parameters` release harness includes exact assertions and
runs 250 iterations per row. Three-run median costs are **44.280 µs** for a
quintic root query, **126.885 µs** for a weighted spatial quintic,
**139.514 µs** for rejection of a planar-only candidate, and **137.711 µs** for
two authored NURBS spans. These capabilities were unavailable in the baseline;
the measurements are feature costs. The migrated work-budget harness preserves
its four receipts as well. See
[feature-cost.json](docs/evidence/2026-10-15-retained-inverse-parameters/feature-cost.json).

## Initial kernel baseline

`benches/kernel.rs` measures the first stable final-API cohorts: exact cuboid,
native-cylinder, boundaryless-face sphere, truncated-cone, periodic-patch
torus, polygonal-revolution, affine linear-sweep, fixed-frame rational
Bézier curved-sweep, and homothetic-loft
construction; retained-certificate
solid-volume evaluation; and exact point classification.
Every cohort includes semantic assertions so a faster but incorrect path
cannot become the baseline.
The editing cohorts separately expose canonical edge attachment, repeated
whole-model revalidation, deterministic multi-trace partitioning, and retained
intersection-graph partition dispatch.

The 2026-07-29 optimized baseline on the development workstation is:

The tensor-plane cohort is deliberately rebased from `4.455 us` to
`5.203 us`: it now constructs and retains exact pcurves on both surface
operands rather than returning only a spatial curve. That semantic expansion
is the recorded reason for exceeding the ordinary 10% repeat budget.

Tensor patch validation now derives and compares exact homogeneous iso-curves
rather than recognizing only stored boundary rows and columns. That stronger
certificate deliberately rebases rational Bézier patch construction from
`66.082 us` to `74.365 us` and NURBS patch construction from `286.598 us` to
`323.509 us`.

- cuboid build + exact volume + classification: `248.478 us/iteration`;
- cylinder build + exact volume + classification: `341.097 us/iteration`;
- sphere build + exact volume + classification: `3.610 us/iteration`;
- cone-frustum build + exact volume + classification: `4.153020 ms/iteration`;
- torus build + exact volume + classification: `1.208950 ms/iteration`;
- revolved polygon build + exact volume + classification:
  `889.698 us/iteration`;
- native two-arc circular-profile revolution with eight periodic faces,
  exact curved face carriers, Pappus volume, and radial/profile
  classification: `711.462 us/iteration`;
- affine linear sweep build + exact volume + classification:
  `307.438 us/iteration`;
- fixed-frame nonuniform-weight rational Bézier curved sweep with four native
  tensor translation sides, Bernstein affine-progress certification, exact
  volume, and exact section classification: `306.620 us/iteration`;
- explicit moving-frame polynomial taper with native tensor sides, exact
  Bernstein area-law integration, volume, and section classification:
  `387.317 us/iteration`;
- homothetic loft build + exact volume + classification:
  `343.839 us/iteration`;
- convex corresponding non-homothetic loft with four native bilinear sides,
  exact quadratic section-area volume, and interpolated-section
  classification: `230.517 us/iteration`;
- three-section C⁰ loft with one homothetic span, one convex bilinear span,
  topology-derived layer recertification, exact piecewise volume, and seam
  classification: `485.146 us/iteration`;
- cuboid edge split + full revalidation: `214.314 us/iteration`;
- cuboid face split + full revalidation: `203.518 us/iteration`;
- cuboid curve-driven face split, including two exact boundary-edge splits and
  three full revalidations: `708.639 us/iteration`;
- cuboid two-trace face partition, including four exact boundary-edge splits,
  two face splits, and their current full revalidations:
  `1.596921 ms/iteration`;
- cuboid two-diagonal arrangement, including one exact shared-chord split,
  three face splits, and their current full revalidations:
  `947.797 us/iteration`;
- unit prism cap replacement by an affine-image rational Bézier tensor,
  full shell recertification, and exact volume: `242.584 us/iteration`;
- rational Bézier patch build + validation: `74.365 us/iteration`;
- NURBS patch build + validation: `323.509 us/iteration`;
- native-domain NURBS extrusion-patch build, complete profile-image
  validation, and certified monotone-planar exact face area:
  `176.412 us/iteration`;
- paired complete rational Bézier and native-domain NURBS affine-image area
  queries under separable positive projective weights:
  `18.645 us/paired iteration`;
- two-face rational Bézier patch shell construction with projective boundary
  matching, identity stitching, and full validation: `76.707 us/iteration`;
- explicit tensor-face chordal derivation with four exact samples per boundary
  use, three shared-midpoint refinement levels, 896 output triangles, and
  exact source-surface evaluation at every retained vertex:
  `1.361081 ms/iteration`;
- linear tensor patch / parallel-plane exact native iso-curve and two-pcurve
  intersection: `4.820 us/iteration`;
- curved u-linear translation tensor / oblique-plane exact native
  non-isoparametric curve, rational graph pcurve materialization, and midpoint
  evaluation: `11.368 us/iteration`;
- non-separable positive-weight rational bilinear tensor / oblique-plane exact
  rational-quartic spatial section, rational-quadratic graph construction, and
  midpoint evaluation: `32.671 us/iteration`;
- equivalent degree-one NURBS tensor over authored `[2, 5] × [-3, 7]` active
  knot domains, including exact native-coordinate pcurve remapping and
  midpoint evaluation: `34.393 us/iteration`;
- non-separable positive-weight rational bilinear tensor / plane intersection
  with one retained denominator pole, two bounded rational-quartic branches,
  and spatial plus pcurve midpoint evaluation on both components:
  `69.205 us/iteration`;
- non-separable positive-weight planar rational bilinear tensor / containing
  plane exact complete-surface relation: `395 ns/iteration`;
- non-separable positive-weight planar rational bilinear face / partially
  overlapping bounded plane face exact `CurveRegion2` trim:
  `54.358 us/iteration`, including the exact contained-face coverage
  difference proof;
- complete non-isoparametric rational tensor section, graph-control
  recertification, curved-loop validation, identity-shared face split, and full
  model revalidation: `180.175 us/iteration`;
- complete single-span non-isoparametric NURBS tensor section with a native
  `[2,5]` parameter domain, rational graph recertification, identity-shared
  face split, and full model revalidation: `407.129 us/iteration` for the
  u-linear layout and `404.990 us/iteration` for the v-linear layout;
- complete two-span non-isoparametric NURBS tensor section with exact
  per-span graph elevation, native-domain NURBS pcurve assembly, knot/control
  recertification, identity-shared split, and full revalidation:
  `500.329 us/iteration`;
- two represented partial NURBS graph fragments, exact cross-span carrier
  merging, deterministic all-fragment descendant partitioning, two
  identity-shared same-boundary splits, exact Bezier-loop orientation,
  partial-profile recertification, and full model revalidation:
  `3.641689 ms/iteration`;
- two transverse retained tensor iso-curves, certified pcurve intersection,
  exact crossing atomization, one identity-shared crossing vertex, four
  descendant faces, and full model revalidation: `1.322209 ms/iteration`;
- tensor face intersection, exact two-pcurve clipping, canonical boundary
  attachment, identity-shared iso-curve split, and full revalidation:
  `340.603 us/iteration`;
- affine rational-tensor auxiliary-plane section, authoritative inverse
  pcurve, two-boundary endpoint attachment, identity-shared face partition,
  and full model revalidation: `380.718 us/iteration`;
- constant-weight affine-tensor parameter-plane certification, exact curved
  rational inverse pcurve, identity-shared face partition, and full model
  revalidation: `553.064 us/iteration`;
- rank-one projective affine-bilinear certification, two exact Möbius inverse
  coordinates, homogeneous Bernstein denominator product, identity-shared
  face partition, and full model revalidation: `1.556065 ms/iteration`;
- degree-one projective affine NURBS certification over authored
  `[2, 5] × [-3, 7]` active knot domains, native-coordinate Möbius inverse,
  identity-shared face partition, and full model revalidation:
  `2.076926 ms/iteration`;
- circular rational-span normalization, one closed NURBS inverse pcurve,
  identity-shared inner/outer topology, and full model revalidation:
  `1.082481 ms/iteration`;
- mixed-degree rational Bézier/NURBS span normalization, homogeneous degree
  elevation, one closed NURBS inverse pcurve, identity-shared inner/outer
  topology, and full model revalidation: `1.548178 ms/iteration`;
- two disjoint affine-tensor inverse-pcurve fragments crossing one face hole,
  four exact boundary attachments, paired outer/inner bridge authorship,
  identity-shared descendants, exact volume, and full revalidation:
  `3.992495 ms/iteration`;
- three affine-tensor inverse-pcurve fragments threading two face holes, six
  exact boundary attachments, generalized bridge-cycle authorship, explicit
  wire-arena compaction, exact volume, and full revalidation:
  `8.555741 ms/iteration`;
- disjoint cuboid 6×6 face intersection graph with 36 certified broad-phase
  rejections: `11.423 us/iteration`;
- overlapping cuboid intersection graph with exact transverse trim clipping
  and coincident-plane support evidence: `319.782 us/iteration`;
- retained overlapping-cuboid graph to deterministic transverse and
  coplanar-support face partitions: `6.389954 ms/iteration`;
- retained graph partition plus exact planar interior-witness classification
  and intersection selection: `6.638052 ms/iteration`;
- retained graph partition, selection, identity transfer, union shell
  stitching, validation, and exact volume: `15.505052 ms/iteration`;
- skew-cuboid transverse arrangement, selection, convex-shell stitching,
  validation, and exact volume: `25.483189 ms/iteration`;
- skew-cuboid difference through general concave planar-shell pair
  certification, validation, and exact volume: `31.308049 ms/iteration`;
- contained skew-cuboid difference through general planar void assignment,
  strict shell nesting, validation, and exact volume: `3.141293 ms/iteration`;
- sphere/planar-face graph with exact rational-conic region clipping:
  `3.091935 ms/iteration`;
- disjoint sphere union + arena remap + validation: `3.569 us/iteration`;
- partial sphere intersection + periodic-cap build + exact volume:
  `152.334 us/iteration`;
- oriented coaxial cylinder interior cut + two-solid rebuild + exact volume:
  `1.537104 ms/iteration`;
- coincident cone-frustum interior cut + two-solid rebuild + exact volume:
  `2.274817 ms/iteration`.
- axial plane/cone ray clipping, longitudinal half-frustum standard
  intersection, exact volume, compact JSON encoding, untrusted parse, and full
  revalidation: `460.130112 ms/iteration`. The result JSON is required to stay
  below 100 KB so planar arrangement-expression history cannot silently
  re-enter mixed-shell pcurves.
- coaxial revolution contained cut + inward-shell rebuild + exact volume:
  `2.415905 ms/iteration`.

The cuboid, cylinder, and sphere figures are wall-clock means over 1,000
semantic-checked iterations. The larger torus, cone-frustum, editing, spline,
and Boolean cohorts use the iteration counts printed by the benchmark.

Run the core cohorts with:

```sh
cargo bench --bench kernel
```

Run the derived-output cohort as well with:

```sh
cargo bench --all-features --bench kernel
```

The deleted report-oriented benchmark measured the displaced API and is not a
compatibility target.

## Native analytic similarities (2026-10-16)

The [source-bound admission record](docs/evidence/2026-10-16-analytic-similarity/README.md) retains the exact metric/chart,
profile-reflection, archive and torus-measurement changes. No surrounding
implementation was changed. All 123 prior case receipts and fixture facts agree
across 60 alternating processes; seven previous STEP files are byte-identical.
The largest case-median candidate/baseline timing ratio is 1.063686;
3 rows have changed allocator observations. Per-case costs,
all unfavorable samples, allocation requests, requested bytes, executable sizes
and process resources remain in the manifest. These are descriptive shared-host
observations, not a statistical performance gate or a general speedup.

Nine separately named new workloads include exact checks and complete archive
materialization/revalidation. Three processes measure rational, reflected and
radical cylinder replay, primitive/clipped/void/profile matrices, repeated
selected subdivision/inverse cycles, periodic near-offsets and rejection/recovery.
A previously failing scale operation is not their performance baseline. Integer
matrix receipts count completed workflows; each operation also asserts exact
formulas, images, provenance or deterministic archive contracts as appropriate.
No total scalar-work, heap, expression-growth or cancellation bound is established.
The source-path WASM consumer and optional OCCT reader do not close release gates.

## Seam-aligned analytic cuts (2026-10-16)

The [source-bound record](docs/evidence/2026-10-16-seam-cuts/README.md) qualifies the original full-size seam cuts
and their replay dependencies. The complete final 66-process campaign preserves
131 of 132 prior receipts. One radical-cylinder archive has an explicit exact
expression migration: all 32 changed scalar values prove equal, every non-scalar
field agrees, and both full archives validate. No other receipt is waived.
The largest final case-median timing ratio is 1.150828; 1 rows
exceed 10%, and 5 rows change allocator observations. All raw
timing, allocation, requested-byte, executable-size and process-resource records
remain. These are shared-host observations, not statistical performance admission.

The first campaign remains intact, including its failed byte comparison and
five spline timing ratios of 1.135–1.241. The optional boundary proof now checks
its native line representation before reconstructing nonlinear pcurves; complete
correctness and all corpora were repeated. Six new workloads have separate
absolute costs over three processes, including exact checks, complete archive
materialization and validation. Previously failing cuts are not a speedup
baseline. The original 70 MB swapped cone archive failure and the 46,077-byte
completed result both remain; the 16 MiB default admission cap is unchanged.
The remaining native NURBS Mobius inverse pcurve partition row has ratio
1.150828; its cause is not isolated. Retain the exact extension as the next
feature baseline for the recorded semantic reason, with the all-cohort timing
gate open. No total-work, retained-heap, expression-growth or cancellation bound
is claimed.
All broader work-package, typed-frontier and delivery gates remain open.

## Repeated curved composition (2026-10-16)

The [source-bound record](docs/evidence/2026-10-16-curved-coplanar/README.md) qualifies native recuts, canonical-empty
recovery and complementary reconstruction on three original full-size torus/cone
half cuts, both applicable orders and six exact similarities. All 138 preserved
cases run in twelve complete corpora, three alternating CPU-0 pairs each, without
receipt waivers. The largest case-median ratio is 1.117570; 3 rows
exceed 10%, and 10 rows change allocator observations. Every sample,
allocator request, process resource and executable size is retained. These are
shared-host observations, not statistical performance admission.

Eleven new workloads have separate absolute costs from three processes. Exact
checks and complete archive materialization/revalidation remain inside timing;
each case has one untimed warmup. The matrix hashes all complete outputs and
continues through split/reflection/replay. Existing failures cannot serve as a
speedup baseline. The old NURBS Mobius inverse, archive and cached-arc timing
exceptions remain open, alongside any current over-budget rows. Retain this
increment for its exact completion/correctness extension with all costs visible.
The first complete campaign remains, including six over-budget rows. The native
line-side exclusion reduces the largest focused native ratio from 1.295617 to
1.119734; four focused spline rows remain over budget. The final full campaign
above follows complete requalification and retains every cohort. No total-work,
retained-heap, cancellation or controlled-host claim is made.
The five remaining quarter/band diagnostics and all broader work-package,
resource/provenance and delivery gates remain open.

## Analytic sector and band continuation (2026-10-16)

The [source-bound record](docs/evidence/2026-10-16-sector-cuts/README.md) retains the complete preserved corpus, new successful workflows and affected scalar costs.

The complete 78-process campaign compares all 149 preserved cases across thirteen corpora, with no receipt waiver. The largest case-median candidate/baseline ratio is 1.3891891098983489; 5 rows exceed 10%, and 51 rows change allocator observations. Every timing sample, allocator request, process resource and executable size is retained. Seven new successful workflows have separate absolute costs across three processes, including full materialization, exact checks, replay and the thirty-configuration matrix. The affected inverse-trigonometric corpus has 33 cases and 12 alternating baseline/candidate processes with raw Criterion samples. Scalar costs retain their original construction/approximation boundaries; they are not topology receipts. No failed operation is used as a speedup baseline. Shared-host observations do not close controlled-host or statistical performance admission. Prior spline, archive and cached-arc timing exceptions remain open.

No public compatibility shim or archive schema change is added. A private clipping carrier is replaced by the existing materialized correspondence. Iso coefficients and area expressions consume existing exact scalar proofs; one necessary scalar certificate preserves acos(±1/2) as exact multiples of pi. The complete old benchmark receipts and all 22 STEP artifacts remain unchanged. Rejected split-parameter, reflection and boundary-contact compaction trials are removed and their failures retained.

These observations qualify their named contracts only. All 28 work-package gates remain open; prior timing exceptions and the explicit dependency/release limitations in the manifest remain. Previously failed geometry is not a speedup baseline.


The two largest timing ratios are `holed_affine_tensor_paired_bridge_partition`
(1.389189) and `multi_hole_affine_tensor_bridge_cycle_partition` (1.312397).
Complete wire validation is retained because the original crossing-hole control
was incorrectly accepted. These observations do not isolate its contribution to
either slowdown. Future work should reuse native validation evidence while
preserving the complete original fixtures and rejection controls; reinstating
an unjustified simplicity flag would invalidate the repair. The other rows above
10% are `arc_structural_facts_replay` (1.127722),
`arc_large_cached_native_promotion_256_arcs` (1.137500), and `step/units_2000`
(1.135400). Scalar case medians remain within 10% in this shared-host campaign;
that observation is not a controlled-host performance gate.

## Periodic conic continuation (2026-10-16)

The [source-bound record](docs/evidence/2026-10-16-oblique-cuts/README.md) retains complete before/after outputs and the original failed fixtures.

The full 84-process campaign compares 156 preserved cases in fourteen corpora; every receipt agrees. The largest case-median candidate/baseline ratio is 1.248962; 3 rows exceed 10%, and 37 rows change allocator observations. Every adverse row, raw sample, allocation request, process resource and executable size is retained. Three newly successful torus continuation workloads have separate absolute costs across three processes, with exact checks and full output materialization inside timing. No failed operation serves as a speedup baseline. Shared-host measurements do not close controlled-host/statistical gates or earlier timing exceptions.

Rows above 10% in this campaign:

- `coplanar/coplanar/circular_boundary_provenance`: 1.248962.
- `pcurves/pcurves/partial_fresh`: 1.112107.
- `pcurves/pcurves/partial_reused`: 1.121601.

Retain this increment for its exact inverse-query repair and completed Boolean continuation. It makes no aggregate speedup claim. Earlier archive/spline/cached-arc observations remain visible; a favorable later sample does not discharge their gates. No operation-wide resource bound is established.

The new periodic lifting helper repeats certified membership/end-point comparisons. Reusing those comparisons is the next BREP performance experiment. Its contribution to the partial-pcurve allocation/latency changes has not yet been isolated; preserve the full admitted corpus and exact receipts when evaluating it.

## Upstream native integration (2026-10-16)

The [integration evidence](docs/evidence/2026-10-16-upstream-native-integration/README.md) compares the same completed BREP contracts before and after the dependency merge.

The full 90-process campaign preserves all receipts across 159 cases and fifteen corpora. Maximum case-median candidate/baseline latency is 1.148357; 3 rows exceed 10%, and 25 rows change allocation observations. All raw samples, allocation requests, process resources, executable sizes and adverse rows are retained. These shared-host observations do not establish a statistical speedup, total-resource bound or closure of prior performance gates.

Rows above 10% in this campaign:

- `faceted/step/units_2000`: 1.147205.
- `kernel/boolean_kernel/sphere_planar_conic_trim_graph`: 1.148357.
- `oblique/oblique/millimetre_reflection`: 1.102298.

Prior pcurve, provenance, archive, spline and cached-arc observations remain visible. Any conic comparison-reuse experiment needs its own source-bound qualification and complete preserved receipts; it introduces no performance claim by being planned.

## Per-call conic comparison reuse (2026-10-16)

The [comparison record](docs/evidence/2026-10-16-conic-comparison-reuse/README.md) isolates one BREP helper against the qualified upstream integration.

Retained after the complete preserved correctness and cost campaign. The helper reuses strict decided orderings for the same immutable value/domain and changes no accepted geometry, endpoint identity or deterministic output contract. Both partial pcurve cases remove 656 allocation calls and 45,904 requested bytes per operation; their observed full-campaign time ratios are 0.971130 and 0.977347. All 159 cases preserve their receipts and no case-median time ratio exceeds 1.035024. The two similarity matrices with increased allocation observations remain visible; each increase is less than 0.075% of its corresponding baseline. Native harness growth of up to 1,208 bytes and WASM growth of 275 bytes are retained as adverse size observations. These shared-host samples justify this limited local reuse; they do not establish a general speedup, universal allocation reduction or closure of earlier performance, resource, correctness or release gates.

The full 90-process campaign preserves all receipts across 159 cases and fifteen corpora. Maximum case-median candidate/baseline latency is 1.035024; 0 rows exceed 10%, and 33 rows change allocation observations, including 2 with increased calls or requested bytes. All raw samples, allocation requests, process resources, executable sizes and adverse rows are retained. The six-case focused probe is preserved separately. Shared-host observations do not establish a controlled-host speedup, total-resource bound or closure of prior cost gates. Native harness size changes range from 0 to 1208 bytes; the WASM probe grows 275 bytes. These are harness sizes, not shipped-package measurements.

Full-campaign pcurve observations per iteration; requested allocation bytes are not a retained-heap bound:

| Case | Median time ratio | Calls before → after | Requested bytes before → after |
| --- | ---: | ---: | ---: |
| `pcurves/full_fresh` | 0.981064 | 3,542.0 → 3,354.0 | 444,493.0 → 430,933.0 |
| `pcurves/full_reused` | 0.982889 | 3,485.0 → 3,297.0 | 323,089.0 → 309,529.0 |
| `pcurves/partial_fresh` | 0.971130 | 11,634.0 → 10,978.0 | 1,080,321.0 → 1,034,417.0 |
| `pcurves/partial_reused` | 0.977347 | 11,577.0 → 10,921.0 | 958,917.0 → 913,013.0 |
| `pcurves/edit_replay_composition` | 0.991581 | 14,611.0 → 14,423.0 | 1,964,675.0 → 1,951,115.0 |
| `pcurves/reject_nonmaster_height` | 0.948991 | 742.0 → 660.0 | 171,900.0 → 165,948.0 |

No case has a median time more than 10% above baseline in this campaign. Prior cost gates remain open.

Observed allocation increases are retained separately from the pcurve reductions:

| Case | Calls before → after | Requested bytes before → after |
| --- | ---: | ---: |
| `analytic_similarity/similarity/primitive_matrix` | 2,647,253.0 → 2,648,868.0 | 209,903,801.0 → 210,003,137.0 |
| `analytic_similarity/similarity/clipped_void_matrix` | 7,356,861.0 → 7,362,304.0 | 519,115,139.0 → 519,496,547.0 |

These are process observations, not a universal allocation reduction claim.

The original focused probe and all unfavorable rows remain in the evidence. No prior archive, spline, cached-arc, pcurve or provenance cost gate is closed by this experiment.


## Exact prism continuation (2026-10-16)

The [source-bound record](docs/evidence/2026-10-16-prism-continuation/README.md) retains original failures, the full preserved corpus and reviewed representation migrations.

The complete 96-process campaign measures 159 preserved cases in fifteen corpora and eight new completed workflows. 8 old case receipts and 1 fixture-fact records have individually reviewed migrations. Maximum preserved-case median latency ratio is 1.713776; 19 rows exceed 10%. 68 rows change allocation observations, including 60 increases in calls or requested bytes. All raw samples, adverse rows, output receipts, process resources and executable sizes remain. Native harness size changes range from 4336 to 67920 bytes; the WASM harness grows 492304 bytes and includes twelve new test groups. Shared-host observations do not establish a general speedup, controlled-host performance, isolated scalar primitive costs, shipped binary size or closure of earlier cost gates.

Retain the exact capability and correctness extension under the governing ladder: the original rational/radical prism cuts, bore continuation, two-semicircle annulus orientation, split/archive replay and supported STEP compositions now complete their independent exact assertions, while every preceding supported workload remains. This is not retained as an optimization. The 19 preserved-case median ratios above 1.10 (maximum 1.713776), 60 allocation increases and harness growth are adverse outcomes and remain open cost work. Eight old case receipts and one fixture-fact record change intentionally; four complete old/new archives prove equality of all 115 changed scalar values and identity of all non-scalar fields. Seven prism artifacts have separate documented carrier/chart migrations. Neither migration substitutes for semantic tests. Retain the bounded extension with these disclosed costs; merge upstream next, then investigate repeated BREP certificate construction against this complete feature baseline without weakening validation or moving unrelated work out of its owning thread. No broader work-package, release, resource-bound or pre-push gate is closed.

Every observed preserved-case median ratio above 10%:

- `analytic_export/analytic_export/partial_imported`: 1.179108.
- `analytic_export/analytic_export/edit_roundtrip`: 1.179471.
- `analytic_similarity/similarity/radical_replay`: 1.124558.
- `arc/arc_retained_inverse_witness_replay`: 1.262752.
- `circles/circles/closed_fresh_document`: 1.287108.
- `circles/circles/closed_reused_document`: 1.286667.
- `circles/circles/partial_fresh_document`: 1.696032.
- `circles/circles/partial_reused_document`: 1.713776.
- `circles/circles/edits_replay_composition`: 1.315466.
- `kernel/analytic_kernel/cylinder_build_measure_classify`: 1.169058.
- `kernel/analytic_kernel/homothetic_loft_build_measure_classify`: 1.170378.
- `kernel/spline_kernel/nurbs_path_extrusion_build_exact_volume`: 1.200637.
- `kernel/boolean_kernel/oriented_cylinder_interval_cut_build_measure`: 1.113730.
- `pcurves/pcurves/full_fresh`: 1.202172.
- `pcurves/pcurves/full_reused`: 1.210364.
- `pcurves/pcurves/partial_fresh`: 1.389983.
- `pcurves/pcurves/partial_reused`: 1.389129.
- `pcurves/pcurves/edit_replay_composition`: 1.230131.
- `pcurves/pcurves/reject_nonmaster_height`: 1.197891.

New successful workflows have separate absolute costs, with exact checks and full archive/STEP materialization inside timing. A formerly failed operation is not a speedup baseline.

| New workflow | Median milliseconds per iteration |
| --- | ---: |
| `prism_exchange/prism_exchange/half_cylinder` | 15.502098 |
| `prism_exchange/prism_exchange/annular_prism` | 29.438153 |
| `prism_exchange/prism_exchange/preserved_planar` | 84.642063 |
| `prism_exchange/prism_exchange/covering_circular` | 134.602519 |
| `prisms/prisms/same_height` | 129.102996 |
| `prisms/prisms/covering_cylinders` | 4878.928089 |
| `prisms/prisms/covering_bores` | 596.449667 |
| `prisms/prisms/annular_cut` | 39.546634 |

Requested allocation bytes and process RSS do not bound retained heap or total work. Earlier archive, spline, cached-arc, pcurve and provenance timing gates remain open. All broader work-package and delivery gates remain open.

## Exact ring-torus exchange (2026-10-17)

The [source-bound admission record](docs/evidence/2026-10-16-torus-exchange/README.md) retains the original exchange/replay failures, shared scalar proof, native contact/image facts, rejected alternatives and explicit capability migrations.

The complete 105-process campaign compares 167 preserved cases in seventeen corpora and records three new completed torus workflows separately. 162 old exact receipts are unchanged; five archive receipts have explicitly reviewed exact representation migrations. Their complete artifact comparison is linked in the admission record. The original no-migration gate remains failed. All 54 separately retained STEP/archive artifacts agree byte-for-byte. The maximum preserved-case median latency ratio is 1.807459; 15 rows exceed 10%. 65 rows change allocation observations, including 35 increases in calls or requested bytes. All raw samples, unfavorable results, allocation requests, process resources and executable sizes remain in the evidence. These shared-host observations do not establish a general speedup or close earlier cost gates.

Retain the selected exact feature and composition extension under the governing ladder. Ring-torus STEP workflows now compose through native cuts, units, transforms, periodic seams, subdivision and archive replay using shared scalar and native proofs. Prior certified behavior and independent geometric assertions remain; five canonical-expression receipts change only under the complete explicit migration proof. This is not an optimization. The main campaign retains 15 median latency ratios above 1.10 (maximum 1.807459), 65 changed allocation rows including 35 increases, native harness growth of 6880-65176 bytes, and WASM growth of 318775 bytes with seven added groups. The dependency supplement retains five ratios above 1.10 (maximum 1.452677), including four inverse-angle scalar cases and one 9-11 ns predicate case. All adverse observations remain; shared-host samples do not establish a general speedup or controlled-host performance admission. Profile repeated angular certificates against this complete feature baseline next, retaining the same strict success, replay, output and resource-observation contracts.

Every preserved-workflow median ratio above 10%:

| Corpus / case | Candidate / baseline |
| --- | ---: |
| `analytic_export/analytic_export/closed_imported` | 1.149632 |
| `analytic_export/analytic_export/partial_imported` | 1.334782 |
| `analytic_export/analytic_export/edit_roundtrip` | 1.251074 |
| `arc/arc_retained_inverse_witness_replay` | 1.807459 |
| `circles/circles/closed_fresh_document` | 1.393839 |
| `circles/circles/closed_reused_document` | 1.460995 |
| `circles/circles/partial_fresh_document` | 1.627308 |
| `circles/circles/partial_reused_document` | 1.621428 |
| `circles/circles/edits_replay_composition` | 1.194098 |
| `pcurves/pcurves/full_fresh` | 1.283222 |
| `pcurves/pcurves/full_reused` | 1.291432 |
| `pcurves/pcurves/partial_fresh` | 1.456298 |
| `pcurves/pcurves/partial_reused` | 1.466652 |
| `pcurves/pcurves/edit_replay_composition` | 1.161300 |
| `pcurves/pcurves/reject_nonmaster_height` | 1.275334 |

New workflows include exact assertions and full output materialization inside timing. Formerly unsupported operations are not speedup baselines.

| New workflow | Median milliseconds per iteration |
| --- | ---: |
| `torus_exchange/whole_frames_units` | 216.689574 |
| `torus_exchange/halves_reconstruction` | 1780.628593 |
| `torus_exchange/rational_band` | 899.330116 |

Native harness size changes range from 6,880 to 65,176 bytes. The WASM harness changes by 318,775 bytes and now includes 90 probe groups; this is not a matched package-size comparison. Cumulative allocator requests and process RSS do not bound retained heap or total work. Four native subdivision/reload cycles grow the archive from 91,506 to 142,937 bytes and serialized Computable occurrences from 780 to 2,089. Their maximum serialized depth rises from 16 to 18. These five states establish no arbitrary-depth or cancellation guarantee.

The separate dependency microcost supplement records 175 cases in 12 processes, with 5 median ratios above 1.10 and maximum 1.452677. Its scalar baseline was reconstructed after the blocker was discovered; it was not frozen prospectively. Criterion alone supplies no geometric completion or allocation receipt. The four complete dependency library suites preserve all 3,514 named outcomes (3,508 passed, six ignored); the separately reproduced inherited zero-factor debug failures remain.

The [follow-up experiment](docs/evidence/2026-10-16-torus-exchange/future-angle-costs.md) targets repeated proof work while preserving every successful workflow. No individual helper's contribution has yet been isolated. Prior archive, spline, cached-arc, pcurve, provenance and prism cost observations remain open. All broader work-package, resource and delivery gates remain open.


## Exact rotated reconstruction (2026-10-17)

The [source-bound record](docs/evidence/2026-10-17-curved-continuation/README.md)
qualifies two formerly exhausted full-size torus reconstructions and their
repeated native composition. This is an exact-completion extension, not an
optimization. Existing successful proof paths remain first; later exact
rounding and function-congruence attempts may add work.

The complete 108-process campaign preserves all 170 existing receipts across eighteen corpora; two new completed reconstruction workflows have separate absolute costs. 8 existing timing rows exceed 1.10, with maximum ratio 1.261011; 30 rows increase allocation calls or requested bytes. The dependency supplement retains all 196 scalar/predicate/rounding cases across eighteen processes, with 1 ratio above 1.10 and maximum 1.108295.

All main timing ratios above 1.10:

| Case | Candidate/baseline median |
| --- | ---: |
| `circles/circles/closed_fresh_document` | 1.219381 |
| `circles/circles/closed_reused_document` | 1.261011 |
| `circles/circles/partial_fresh_document` | 1.248502 |
| `circles/circles/partial_reused_document` | 1.249149 |
| `circles/circles/edits_replay_composition` | 1.167773 |
| `kernel/spline_kernel/rational_bilinear_tensor_plane_complete_containment` | 1.149944 |
| `kernel/boolean_kernel/sphere_planar_conic_trim_graph` | 1.115818 |
| `oblique/oblique/millimetre_reflection` | 1.102625 |

New complete-workflow absolute medians, including cold replay, exact measures,
classification, topology, two further cycles and full archive receipts:

| Case | Seconds | Allocation calls | Requested bytes |
| --- | ---: | ---: | ---: |
| `sectors/rotated_torus_reconstruction_false` | 13.705385 | 97,057,293 | 7,332,676,263 |
| `sectors/rotated_torus_reconstruction_true` | 14.514327 | 103,895,846 | 7,763,426,189 |

All dependency microcost ratios above 1.10:

| Case | Candidate/baseline median |
| --- | ---: |
| `scalar_micro/symbolic_reductions/div_pi_square_e` | 1.108295 |

All raw samples and adverse allocation, RSS and size observations remain in
the record. All prospective frozen baselines have matching library features
and profiles. Sector whole-process resources/size include two additional
workflows, and WASM adds four groups. These are harness observations, not
package sizes, controlled-host performance or total-resource bounds. Earlier
cost gates remain open; later favorable rows do not discharge them.

Follow the [proof-cost investigation](docs/evidence/2026-10-17-curved-continuation/future-proof-costs.md).
Measure repeated exact proof attempts against this complete feature baseline
before changing scheduling or caching. A new proof attempt's contribution to
any observed slowdown has not been isolated. Preserve every original input,
successful receipt, invalid-input control and explicit unresolved outcome.
