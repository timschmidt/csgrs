# HyperDRC reference and performance audit

This audit maps every README reference to a current implementation boundary and
records only measured performance work. HyperDRC remains a conservative release
reviewer: a spatial index may propose candidates, but exact or explicitly lossy
narrow-phase geometry still decides findings.

## Exact measurement cache repair, 2026-10-08

Polygon area and bounds now cache completed STRICT measurements only. A retained
source-construction failure or exhausted operation cannot publish a measurement,
including a warm cached value. Unavailable native area is no longer replaced by
the area of report chords. Five focused regressions cover invalid placeholders,
warm and cold budget denial, retry, concurrent preview/budget scopes, and an
unsupported degree-five rational cap. The complete release library suite passed
1,119 tests; strict all-target Clippy and rustdoc also passed. The
[validation manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-geometry/measurement-manifest.json)
retains commands, failures, source hashes, and the separate Gerber lint cleanup.

Five alternating baseline/candidate process pairs measured the same three valid
fixtures, with 1,000 iterations per row, release compilation, and CPU 0 affinity.
All task builds and tests had exited. Report row counts, serialized byte lengths,
checksums, and bounds match. Median time per observation, in microseconds:

| Fixture | Mode | Baseline | Guarded cache |
| --- | --- | ---: | ---: |
| Rectangle | Warm | 0.090 | 0.081 |
| Rectangle | Construct and measure | 29.690 | 30.410 |
| Native circle | Warm | 0.282 | 0.279 |
| Native circle | Construct and measure | 1,060.552 | 1,048.535 |
| 64 rectangles | Warm | 4.228 | 3.538 |
| 64 rectangles | Construct and measure | 2,110.515 | 2,103.470 |

The rectangle construction median increased 2.42%; the other construction
medians changed by less than 1.14%. Warm timings are small and variable, so this
probe supports no general speedup claim. Whole-process peak RSS was
9,756–9,844 KiB for the baseline and 9,672–9,996 KiB for the candidate. Linked probe
sizes were 18,658,032 and 18,650,496 bytes. These are process and executable
measurements, not allocation counts or a library-size comparison. The
[raw runs and replay inputs](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-geometry/measurement-probe/manifest.json)
retain all observations and limitations.

This repair covers the measurement boundary. Readiness consumers that discard
`None`, component/projection caches, native source geometry, and conservative
spatial rejection still require integration work.

## Native circle and source-bound repair, 2026-10-08

`circle_polygon` now constructs native arcs while retaining every requested
report sample. KiCad via copper retains the exact decimal center and diameter
on each copper layer, and profile bounds come from authoritative regions. The
configured `cargo test --locked -p hyperdrc --release` command passed 1,126 library
tests, two README integration tests, and two doctests. Strict all-target Clippy,
rustdoc, and formatting pass. The
[source and validation record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-geometry/circle-manifest.json)
retains the original failing source-via case and the unchanged million-sample
fixture alongside the completed checks.

Five sequential processes, pinned to CPU 0 after builds/tests ended, measured
100 iterations per case. Median microseconds per observation:

| Report samples | Native area/bounds on an existing circle | Construct report ring and measure |
| ---: | ---: | ---: |
| 16 | 32.862 | 163.279 |
| 48 | 23.082 | 148.839 |
| 128 | 22.130 | 157.832 |
| 1,000,000 | 22.048 | 14,034.286 |

These queries call the native region directly under STRICT, rather than the
Polygon measurement cache measured above. The source-via load/classification
case took a median 188.375 microseconds, including file reading and exact
membership checks on both copper layers. Report coordinate counts, payload sizes
and bit-pattern checksums are stable across circle trials; the million-sample
case retains all 1,000,001 closed coordinates. Whole-process peak RSS was
24,212–24,384 KiB, and the linked probe was 19,035,736 bytes. No allocation count or
library-size delta was measured.

This is capability cost evidence: the former chord constructor represented a
different exact region. Small-case and warm timings varied substantially, so no
general speedup is claimed. The
[raw probe and replay inputs](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-geometry/circle-cost/manifest.json)
retain every trial. Native clearance consumers, projection/component failures,
conservative spatial rejection, and readiness uncertainty propagation remain
open; the source-circle repair does not certify those paths.

## Native boundary metric adoption, 2026-10-08

All 52 report-metric call sites and the prepared net-clearance cache now query
regularized native boundaries, including holes. Required metric failures and
comparisons remain uncertainty through findings and readiness receipts. Six
retained pre-fix failures cover circle/chord disagreement, a hole, an internal
union seam, an unsupported rational boundary, and an exhausted warm query.
The release gate passed 1,138 library tests, two README integration tests and
two doctests; strict all-feature/all-target Clippy, rustdoc and formatting pass.
The [manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-distance/manifest.json)
retains the source hashes, failures, commands, and scope limitations.

Five fresh processes pinned to CPU 0 measured the unchanged 2,000-square,
64-probe field after all builds/tests ended. Median filled-region construction
and boundary preparation took 628.555 ms; 64 probe constructions and queries took
22.352 ms, within the existing two-second gate. Every trial produced 64 exact
unit distances, 257 serialized bytes and the same SHA-256. Whole-process peak RSS
was 65,088–65,608 KiB. The test executable was 51,646,568 bytes.

One full fixture smoke run, also pinned to CPU 0, passed in 65.446 seconds with
1,483,976 KiB peak RSS; its executable was 42,051,792 bytes. The
[raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-distance/cost/manifest.json)
includes every trial and both executable hashes. These are candidate capability
costs: corrected native answers can differ from report-chord answers. No matched
speedup, allocation count or library-size delta is claimed. Original rectangle
construction is outside the field preparation timer but inside process RSS.

Finite spatial rejection elsewhere, component/projection failure handling,
other source shapes, and non-distance readiness measurements remain open. The
finite reference distance helpers are retained only in tests.

## Fallible native components and report projection, 2026-10-08

`PcbRegionExt::try_to_multipolygon` now returns typed failures. Native component
and report caches retain successful results only, check source and shared-budget
failures even when warm, and execute under STRICT. Generated report polygons
retain each native component's identity and holes; authored profiles retain
their duplicate decomposition and requested sample counts. Required callers
propagate projection failures into uncertainty findings and readiness receipts.
Migrated candidate indices retain unknown native bounds. Required net width now
queries native bounds even without parser-attached bounds; unavailable width no
longer becomes zero.

Five original failing regressions and the consumer width failure are retained in
the [evidence manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-projection/manifest.json).
The final configured DRC release command passed 1,154 library tests, two README
integration tests and two doctests, including sixteen new regressions. Strict
all-feature/all-target Clippy, rustdoc and formatting pass. Consumer validation
passed nine tests selected by `hyperdrc` and all five curved-outline tests, with
strict all-feature/all-target Clippy. Source hashes remained fixed through each
validation chain.

All six Easyduino fast-DRC snapshots were audited before updating their expected
findings and shared check implementation version. Every one of the 190 changed
finding IDs flips the clearance threshold decision between native boundaries and
the retained report-chord metric. Exhaustive native segment comparisons covered
202 matching feature pairs with zero decision mismatches. This cross-check shares
Hypercurve certificates with production; it is not an independent-library proof.
The [snapshot audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-projection/snapshot-audit/summary.json)
retains exact values, source feature identities, old/new hashes and replay code.
The fast gate's existing zone omission and all native fixtures are unchanged.

Five sequential fresh processes pinned to CPU 0 measured six projection rows
after all builds and tests exited. Median query times, in microseconds:

| Fixture | PCB cache state | Time |
| --- | --- | ---: |
| Authored circle, 17 samples | Warm | 0.061 |
| Authored circle, 1,000,000 samples | Warm | 1,097.348 |
| Native circle | Fresh component/report caches | 892.325 |
| Native circle | Warm | 0.095 |
| 64 native components | Fresh component/report caches | 51.784 |
| 64 native components | Warm | 2.670 |

All 1,000,001 closed coordinates in the million-sample case and all 64 components
are retained. First/last report fingerprints and native areas match across
trials; six identity rows serialize to 768 bytes with a stable SHA-256. Input
construction is outside query timers and inside process RSS. Fresh PCB caches
clone a native region whose internal kernel evidence can already be warm. Peak
RSS was 54,992–55,336 KiB; the linked probe was 22,798,256 bytes.

The unchanged full fixture smoke gate passed in 60.113 seconds against its
120-second limit, with 1,466,320 KiB peak RSS and a 42,234,920-byte executable.
The [raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-projection/cost/manifest.json)
retains every trial, source hash and artifact identity. These are candidate
capability costs, with no matched speedup, allocation count or library-size
delta claim. Sub-microsecond warm timings have limited precision.

The remaining audit includes other finite spatial and shape heuristics,
area/containment/source-value consumers, legacy optional dimensions, and native
Boolean/offset/translation boundaries. General curve aggregation still has typed
blockers. This repair does not establish complete DRC certification or a total
time, memory or precision bound.

## Guarded native operations and measurements, 2026-10-08

PCB Boolean operations, offsets, point classification and bounds now retain
source failures and known shared-budget exhaustion under STRICT. Classification
distinguishes boundary from uncertainty, while empty geometry has no bounds.
Offset bounds come from the resulting native set; collapsed or removed
components cannot leave inferred input bounds. Circle callers use typed native
construction, and migrated envelope-dimension consumers retain uncertainty
through findings and readiness receipts. The former panicking circle adapter,
origin-box fallback and ambiguous membership alias are removed.

The [evidence manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-operations/manifest.json)
retains nine failing pre-fix regressions. Sixteen final regressions include exact
Boolean sets, eroded components, boundary/empty distinctions, unknown queries
inside preview, concurrent preview/denied scopes, and readiness propagation.
The configured release command passed 1,170 library tests, two README integration
tests and two doctests. Strict all-feature/all-target Clippy, rustdoc and
formatting pass, with source hashes fixed through the complete validation chain.

Consumer validation passed nine tests selected by `hyperdrc` and all five
curved-outline tests, with strict all-feature/all-target Clippy. All six
Easyduino fast-DRC snapshots preserve their finding rows and counts; only the
shared implementation version changes from 3 to 4. The
[version audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-operations/snapshot-versions.json)
records the old/new hashes and exact field-only comparison.

Five fresh processes pinned to CPU 0 measured the following median query costs,
in microseconds, after every build and test exited:

| Operation | Time |
| --- | ---: |
| Rectangle union and result bounds | 105.725 |
| Rectangle intersection and result bounds | 100.514 |
| Rectangle difference and result bounds | 101.231 |
| Rectangle XOR and result bounds | 116.166 |
| Circle offset and result bounds | 87.099 |
| Drill circle construction and result bounds | 111.236 |
| Circle point classification | 31.465 |
| Circle bounds | 7.717 |

Initial fixtures are constructed outside query timers. Each region row times
the operation and checked result bounds with warm inputs and new PCB output
wrappers; internal kernel evidence may already be warm. First/last native areas
and report-coordinate fingerprints are checked outside timers. Eight witness
rows retain the same 721 serialized bytes and SHA-256 in every trial. Peak
process RSS was 10,336–10,608 KiB; the linked probe was 23,386,088 bytes.

The full, unchanged two-board and Gerber fixture gate passed in 60.283 seconds
against its 120-second limit, with 1,483,620 KiB peak RSS and a 42,319,320-byte
executable. The [raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-operations/cost/manifest.json)
retains all trials, source hashes, output witnesses and artifact identities.
These are candidate capability costs; no matched speedup, allocation count or
library-size delta is claimed.

Raw native access through `Deref`, early empty-region shortcuts, aggregation,
remaining optional area/length measurements, scalar threshold decisions and
finite source adapters still require audit. Envelope dimensions and endpoint
spans remain readiness proxies, not proofs of physical trace width or routed
length. The shared exhaustion guard does not establish total runtime, memory or
precision bounds.

## Checked native handoff and aggregation, 2026-10-08

Implicit `PcbRegion` native access is removed. Checked area, emptiness and named
native handoffs retain source and shared-budget failures under STRICT. Explicit
replacement clears old facts and caches while preserving metadata and existing
clones. Singleton aggregation shares the native carrier; mixed supported curve
families use balanced native union while line/arc batches retain their bulk
path. Net-length measurement failures produce uncertainty and prevent partial
net or pair totals from appearing complete.

The [evidence manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-handoff/manifest.json)
retains seven original failures and the already-passing exhaustion control.
Seventeen final regressions cover handoff/replacement, exact quadratic unions,
report-coordinate independence, preview/concurrent budget scopes, retry and
partial totals. The configured release gate passed 1,187 library tests, two
README integration tests and three doctests, including the compile-fail native
access check. Strict all-feature/all-target Clippy, rustdoc and formatting pass.
DRC sources stayed fixed through validation and cost measurements.

Consumer validation passed nine tests selected by `hyperdrc` and five
curved-outline tests, with strict all-feature/all-target Clippy. One test needed
migration to checked emptiness; its initial compiler error and final replay are
retained. The six Easyduino snapshots preserve all finding rows and counts;
only implementation-version fields change from 4 to 5. The
[version audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-handoff/snapshot-versions.json)
records the exact field-only comparison and both hashes.

Five sequential fresh processes pinned to CPU 0 measured median query costs
in microseconds after all builds/tests exited:

| Operation | Time |
| --- | ---: |
| Rectangle union and result bounds | 102.378 |
| Rectangle intersection and result bounds | 97.131 |
| Rectangle difference and result bounds | 98.866 |
| Rectangle XOR and result bounds | 113.261 |
| Circle offset and result bounds | 83.388 |
| Drill circle construction and result bounds | 107.477 |
| Circle point classification | 30.834 |
| Circle bounds | 7.273 |
| Single quadratic aggregation and result bounds | 0.456 |
| Mixed quadratic/rectangle aggregation and result bounds | 28.204 |
| Native replacement and result bounds | 2.168 |
| Rectangle area | 2.377 |
| Rectangle emptiness | 0.018 |
| Clone and checked native handoff | 0.108 |

Region rows include checked result bounds with fixed warm inputs; native kernel
evidence may already be warm. Construction of initial fixtures and first/last
exact-area/report-fingerprint checks are outside timers. The handoff row includes
cloning the PCB wrapper. Fourteen witness rows retain 1,260 bytes and the same
SHA-256 in all trials; the previous eight operation witness rows are unchanged.
Whole-process peak RSS was 11,632–12,156 KiB, and the linked probe was
23,767,144 bytes. No matched speedup, allocation count or library-size delta is
claimed.

The unchanged full two-board and Gerber fixture gate passed in 59.603 seconds
against its 120-second limit, with 1,343,616 KiB peak RSS. The
[raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-handoff/cost/manifest.json)
retains all trials, source hashes, witnesses and artifact identities.

At this checkpoint, required Polygon-area consumers and report filtering still
discarded unavailable integrals through `Option`; the following repair addresses
those paths. Remaining scalar thresholds, source geometry and finite adapters
still require repair. Endpoint spans remain readiness proxies, and shared
exhaustion guards do not establish total runtime or memory bounds.

## Required area and regularized coverage, 2026-10-08

`Polygon::try_area` and `multipolygon_to_shapes_scalar` now return typed failures
for unavailable integrals, unknown comparisons and exhausted scopes. Report
projection also rejects non-finite output and cannot publish a partial shape
list. Successful cached area remains native; finite report-chord area stays an
output value. Required callers propagate these failures into uncertainty
findings and readiness receipts, retaining the selected layer names.

Whole-layer totals and profile collection areas measure native filled sets.
Paste/mask aperture ratios union full qualifying apertures, including their
existing overhang contribution. Windowpane and tombstone coverage union clipped
intersections. Windowpane counts use full aperture components before clipping,
so duplicates and copper holes cannot create extra openings. KiCad retains
failed pad geometry for diagnostics and excludes certified empty pads.

The [evidence manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-area/manifest.json)
retains nine failures on the prior production revision, one passing source-error
control, and four further duplicate-coverage failures from the recorded staged
worktree. Both baseline source reconstructions match every retained hash.
Twenty-three area regressions and two new parser regressions pass; the existing
incomplete-pad fixture is preserved and now requires its uncertainty finding.
The final configured release gate passes 1,212 library tests, two README tests
and three doctests, with strict all-feature/all-target Clippy, rustdoc and
workspace formatting. Production sources stayed fixed throughout that chain.

Consumer validation passes nine tests selected by `hyperdrc` and five
curved-outline tests, plus strict all-feature/all-target Clippy. All six
Easyduino snapshots preserve finding rows and counts; their implementation
version fields alone change from 5 to 6, as recorded in the
[snapshot audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-area/snapshot-versions.json).
This is the targeted consumer suite, not the entire HyperCircuit suite.

Five sequential fresh processes pinned to CPU 0 measured these median costs
in microseconds after builds and tests exited:

| Query | Time |
| --- | ---: |
| Warm Polygon circle area, 17 report samples | 0.038 |
| Warm Polygon circle area, 1,000,000 report samples | 0.037 |
| Construct circle and measure area, 17 samples | 96.776 |
| Construct circle and measure area, 128 samples | 99.382 |
| Native area gate and report output, 17 samples | 1.049 |
| Native area gate and report output, 128 samples | 1.309 |
| Filled area of overlapping/duplicated profiles | 1.311 |
| Duplicate paste union coverage check | 8.804 |
| Duplicate mask union coverage check | 8.808 |
| Duplicate aperture windowpane check | 11.256 |

Warm rows query existing polygons; construction and first/last exact-area and
report-fingerprint checks are outside their timers. Construction rows include
new report samples, first area evaluation and drop; internal native evidence
may already be warm. Report rows include finite output allocation. Coverage
rows time whole checks on fixed inputs and verify expected finding counts.
All 24 witness rows retain identical 2,330-byte output in all five trials; the
previous 14 operation witnesses are unchanged. Whole-process peak RSS was
58,648–58,900 KiB, including the million-sample fixture. The linked probe was
23,994,272 bytes. These are capability costs; no matched speedup, allocation
count or library-size delta is claimed. The
[raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-area/cost/manifest.json)
retains commands, source hashes, artifacts and all trials.

The unchanged full two-board and Gerber fixture workload passed in 64.778
seconds against its 120-second gate, with 1,388,444 KiB peak RSS. CPArti's
layer-sanity count increases from four to five; the other eight logged check
counts are unchanged. The [full-board audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-area/fixture-audit/manifest.json)
certifies that parsed In1.Cu and B.Cu both have native area 10,000 and an empty
native symmetric difference. Their authored-area sums differ, so the old
area-equality gate missed the duplicate. The full board is preserved. This
replays the old numeric gate algebraically and validates the parsed native sets;
it does not certify every upstream parser or execute the entire old check.

Required bounds/centers/dimensions, other scalar thresholds, report-chord width
shortcuts, the bounding-box density proxy and remaining source adapters are
still open. Unsupported integrals remain typed failures; this change does not
add missing formulas. Shared guards do not establish total resource bounds.

## Required bounds and native candidate selection, 2026-10-08

`Polygon::try_bounds` now distinguishes a certified empty set from a failed
measurement. Required aperture, stencil, text-height, skinny/sliver,
duplicate-layer and density preparation paths retain bounds failures as
uncertainty; tombstone pad-center construction is checked. Minimum envelope
dimensions require a certified comparison. Warm and empty caches still respect
source failures, STRICT policy and shared-budget exhaustion. Text-height report
centers are formed in native arithmetic before finite projection, so a finite
center cannot overflow through an intermediate float sum. Optional display
markers remain separate from the existence of proved findings.

The layer-polygon grid covers every cell between certified native envelope
endpoints. Unknown, unrepresentable or oversized envelopes use fallback
candidates, with a 256-cell cap on each indexed/query box. The endpoint-to-cell
map is monotone; closed overlapping enclosures therefore share a cell despite
rounding or integer-key saturation. Positive radius expansion rounds outward.
Duplicate-island checks use this grid followed by native area/intersection
predicates, removing the quantized report-area signature that missed identical
circles with different sample counts. Native envelope filters reject pairs only
on certified separating axes.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-bounds/manifest.json)
retains fourteen reproduced production failures, three already-passing controls,
and twenty-four final regressions. All 1,241 configured tests pass: 1,236 library,
two README and three doctests. Strict all-feature/all-target Clippy, rustdoc,
formatting and the fixture build pass with Rust sources frozen. Fourteen selected
HyperCircuit tests also pass, including all six Easyduino snapshots. Those
snapshots change only implementation version 6 to 7, preserving finding rows,
counts and evidence IDs.

Five sequential release processes pinned to CPU 0 measured the following median
costs in microseconds after all builds/tests exited:

| Query | Time |
| --- | ---: |
| Warm Polygon circle bounds, 17 report samples | 0.093 |
| Warm Polygon circle bounds, 1,000,000 report samples | 0.095 |
| Construct circle and measure bounds, 17 samples | 105.548 |
| Construct circle and measure bounds, 128 samples | 106.840 |
| Warm certified-empty Polygon bounds | 0.026 |
| Unknown native Polygon bounds | 3.280 |
| Duplicate circles with 17/128 report samples, complete check | 17.139 |
| 1,003 sparse rectangles with one triple duplicate, complete check | 961.948 |

Warm query fixtures, exact-area checks and report fingerprints are outside the
query timers. Construction rows include new circles, report samples, first
bounds queries and drop. Complete-check rows include index construction and
narrow predicates; first/last geometry and expected finding checks are outside
their timers. Internal native evidence may already be warm. All 32 witness rows
retain identical 3,212-byte output across all five trials, and the preceding
24 rows are unchanged. Peak process RSS was 58,804–59,284 KiB, including the
million-sample fixture; the linked probe was 24,097,696 bytes. These are capability
costs, without a matched speedup, allocation count or library-size claim. The
[cost manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-bounds/cost/manifest.json)
retains all trials and source/artifact hashes.

The unchanged full two-board and Gerber fixture passed in 65.072 seconds against
its 120-second gate, with 1,339,172 KiB peak RSS. All nine logged check counts are
unchanged. The [rendered-report audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-bounds/cost/fixture-report-audit.json)
preserves every finding's geometry, messages and coordinates after explicit
normalization: temporary paths and IDs are excluded, finding rows are sorted,
and four CPArti duplicate markers are reordered. Their exact coordinates are
unchanged; native source traversal replaces the old report-signature order.
Raw ordered output differs. This does not prove original report ordering,
path-independent IDs or identity of unrendered serialized fields.

Stencil point-index projections and point-distance thresholds, other spatial
indices, report-chord width shortcuts, the density interpolation proxy and
remaining source adapters are still open. Shared guards do not establish total
resource bounds. Full-fixture report ordering needs a separate audit. The
complete properties.md scope remains active; this is a bounded repair.

## Native stencil point and radius decisions, 2026-10-08

Stencil pad pairing now indexes certified enclosures of native centers and the
search radius. The grid is shared with polygon candidates in
`checks/spatial/envelope.rs`; unavailable projections retain candidates. The
required predicate compares squared native distance and radius under STRICT,
with a separate negative-radius decision. It cannot skip an unavailable square
root or turn an unresolved order into a definite pair.

Via/drill matching checks its radius and distance decisions before selecting a
drill or copper fallback. Irrelevant drills are skipped before numerical work;
a known first match does not inspect later entries. An unresolved earlier match
remains uncertainty. Via report anchors use checked finite projection, removing
the projection panic at that boundary.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-stencil-points/manifest.json)
retains six reproduced production failures, including two panics, and five
already-passing controls. Sixteen final regressions cover threshold boundaries,
unknown orders, huge/tiny/radical coordinates, candidate retention, budget
denial/retry and a blocking readiness receipt. The configured release gate passes
1,252 library tests, two README tests and three doctests. Strict all-target and
all-feature Clippy, rustdoc, formatting and the fixture build pass with Rust
sources frozen. Fourteen selected consumer tests pass; all six Easyduino
snapshots preserve finding rows, counts and evidence IDs while implementation
version fields advance from 7 to 8.

Five sequential release processes pinned to CPU 0 after builds/tests exited
measured these complete-check medians, in microseconds:

| Query | Time |
| --- | ---: |
| Tombstone pair exactly on the radius boundary | 75.186 |
| Radius just below the pair distance | 6.724 |
| Qualifying pair with a radius beyond finite projection | 71.560 |
| Unresolved pair radius | 24.778 |
| 1,002 sparse pads with one qualifying imbalance | 2,329.067 |
| Via exposure with a matched plated drill | 1,088.357 |
| Unresolved drill match | 12.716 |
| Unrepresentable via report marker | 580.271 |

Timers include index construction, native predicates, coverage geometry and
report allocation on fixed inputs. First/last native areas, report geometry
and serialized-finding fingerprints are checked outside timers. Internal native
evidence may be warm. All 40 witness rows retain identical 4,024-byte output
across five trials, including the previous 32 unchanged witnesses. Peak process
RSS was 58,744–59,228 KiB, including the million-sample report fixture; the linked
probe was 24,260,128 bytes. These are capability costs, without a matched speedup,
allocation count or library-size claim. The [raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-stencil-points/cost/manifest.json)
retains commands, source/artifact hashes and all trials.

The unchanged full two-board and Gerber fixture passed in 68.097 seconds against
its 120-second gate, with 1,334,628 KiB peak RSS. All nine logged check counts are
unchanged. The [report audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-stencil-points/cost/fixture-report-audit.json)
found a duplicate-layer row with reversed layer order and different optional
markers. The [full-board witness](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-stencil-points/fixture-order-audit/manifest.json)
reproduces both variants by reversing the In1.Cu/B.Cu inputs. The relevant source
files are unchanged: layer collection iterates a `HashMap`, and duplicate-layer
markers come from the left source's first authored profiles. Both native layers
have area 10,000 and empty symmetric difference; the finding polygon and message
are identical in both orders. Every other rendered finding matches after
explicit temporary-path/ID and row-order normalization. Original report order,
optional-marker identity and path-independent IDs remain unproved.

Other point/copper/drill indices, affine operation guards, remaining scalar
thresholds, source adapters, report-chord widths and density interpolation remain
open. Shared guards do not establish total resource bounds. All 39 properties,
all 14 targets and crate-specific future work remain in scope.

## Native affine operation guards, 2026-10-08

Polygon affine transforms now use the shared STRICT and work-budget guard.
The native helper checks retained source failure before transforming geometry,
including identity and empty fast paths. The public wrapper retains a failed
construction on its returned polygon; leaving an exhausted scope cannot turn
that polygon into valid geometry. Retrying from the original source in a fresh
budget remains possible. Report rings are transformed separately.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-affine/manifest.json)
retains four baseline failures: two public transforms escaped an exhausted
budget, the private helper accepted a failed source, and an enclosing preview
changed an unresolved native transform diagnosis into an invalid-transform
diagnosis. The last case demonstrates policy-dependent failure reporting, not
a successful wrong geometry result. Public source-error retention and native
circle report-sample independence already passed as controls.

Twelve final regressions cover warm caches, thread scopes, holes and boundary
membership, a complete degree-five rational carrier, and blocking readiness
receipts. A fresh zero budget admits an exact 30-degree circle rotation with
zero instrumented work usage. The configured release gate passes 1,264 library
tests, two README tests and three doctests. Formatting, strict all-feature and
all-target Clippy, strict rustdoc and the fixture build pass. Fourteen selected
consumer tests pass; all six Easyduino snapshots change only implementation
version fields from 8 to 9. Rust sources remain fixed through validation and cost
measurement.

Five sequential release processes pinned to CPU 0 measured these public
transform medians, in microseconds:

| Query | Time |
| --- | ---: |
| Rectangle identity | 0.273 |
| Rectangle quarter turn and translation | 8.464 |
| Native circle, 30 degrees, 17 report samples | 50.763 |
| Native circle, 30 degrees, 128 report samples | 52.281 |
| Quadratic cap quarter turn and translation | 5.089 |
| Empty native region | 1.954 |
| Retained source failure | 1.665 |
| Explicitly exhausted scope, including its setup | 12.422 |

Input construction and first/last native area, bounds and report fingerprints
are outside timers. Report allocation is timed; native evidence may be warm.
Bounds are checked with STRICT numeric comparisons because scalar structural
equality does not prove numeric inequality. The [rejected probe draft](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-affine/draft/cost-structural-equality/manifest.json)
records that corrected harness mistake; it is excluded from final trials.

All 48 witness rows preserve identical 4,788-byte output across the five trials,
including the previous 40 unchanged rows. Peak process RSS was 58,736–59,404 KiB,
including the existing million-sample report fixture. The linked probe was
24,309,000 bytes. These are capability costs, without a matched speedup,
allocation count or library-size comparison. The [raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-affine/cost/manifest.json)
retains commands, source/artifact hashes and every trial.

The unchanged full two-board and Gerber fixture passed in 64.971 seconds against
its 120-second gate, with 1,390,436 KiB peak RSS. All nine logged check counts and
every rendered finding match after explicit temporary-path/ID and finding-row
normalization. The [report audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-affine/cost/fixture-report-audit.json)
records row-order differences and no marker changes. The [known-variant audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-affine/cost/known-order-audit.json)
also checks the duplicate-layer row against the previously reproduced full-board
variants, retaining its geometry, message and header counts. It reuses that
native witness; original report order and path-independent IDs remain unproved.

Legacy point/copper/drill indices, remaining scalar thresholds, finite source
adapters, native pad/track/slot/custom geometry, report-chord widths and density
interpolation remain open. Shared guards reject known exhaustion without
establishing total resource bounds. All 39 properties, all 14 targets and
crate-specific future work remain in scope.

## Native drill candidates and correspondence, 2026-10-08

Drill spacing now indexes certified outward enclosures of native circular
proxies. Unknown, unrepresentable and oversized bounds remain candidates with
their original source indices. Cross-source table and IPC diameter checks use
native point enclosures and checked center/diameter comparisons. Hardware and
castellation spacing use required signed native gaps; castellation selection
queries the authoritative outline boundary. All five former drill-index
consumers use the shared STRICT and work-budget guard, with typed failures
carried into readiness receipts.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-drills/manifest.json)
retains ten baseline failures and two passing controls. Failures include finite
projection panics, an omitted huge drill, shifted IPC source identity, skipped
unresolved matches and definite findings from unresolved orders. Twenty-one
final regressions cover these cases, signed spacing, invalid radii, warm/cold
budget denial and retry, and native circle report-sample independence. Hardware
diameter-selection failures also reach the distribution check; its remaining
hull, spread and report path is still open.

The release gate passes 1,285 library tests, two README tests and three doctests.
Formatting, strict all-feature/all-target Clippy, strict rustdoc and the fixture
build pass. Fourteen selected HyperCircuit tests and strict Clippy pass. The six
Easyduino snapshots change only their implementation-version fields from 9 to
10. Source hashes remain fixed through validation and cost measurement.

Five sequential fresh release processes pinned to CPU 0 measured these complete
check medians, in microseconds:

| Check/input | Time |
| --- | ---: |
| Drill spacing, exact boundary | 2.543 |
| Drill spacing, huge diameter | 5.451 |
| Drill spacing, unprojectable marker | 2.149 |
| Drill spacing, unresolved threshold | 13.209 |
| Drill spacing, sparse field of 2,002 drills | 1,997.527 |
| Drill table, preserved IPC source index | 2.326 |
| Drill table, unresolved diameter | 9.793 |
| IPC drill diameter | 1.748 |
| Mounting-hole spacing | 4.766 |
| Castellation spacing, rectangular outline | 6.771 |
| Castellation spacing, circle with 17 report samples | 21.394 |
| Castellation spacing, circle with 128 report samples | 21.957 |

Timers include index construction, predicates and report allocation. Constant
input construction and first/last full finding fingerprints are outside timers;
native geometry evidence may be warm. All 60 witness rows match across trials,
including the previous 48 unchanged rows. Output is 6,019 bytes with SHA-256
`6585bd062e2fd2273da3364f4c92527b9141589af0b0b6df47e10928fdf537d2`.
Whole-process RSS was 58,856–59,508 KiB, including the existing million-sample
fixture; the linked probe was 24,548,144 bytes. The
[raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-drills/cost/manifest.json)
retains every trial. These are capability costs, without a matched speedup,
allocation count or library-size comparison.

The unchanged full two-board/Gerber fixture passed in 65.027 seconds against its
120-second gate, with 1,338,864 KiB peak RSS. All nine logged check counts and
every rendered finding match after temporary-path/ID and finding-row
normalization. The [report audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-drills/cost/fixture-report-audit.json)
records row-order differences and no marker changes. The
[known-variant audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-native-drills/cost/known-order-audit.json)
reuses the prior full-board duplicate-layer witness; original report order and
path-independent IDs remain unproved.

Legacy point/copper indices, other drill decisions, native pad/track/slot/custom
source geometry, report-chord widths and density interpolation remain open.
Drill inputs retain their documented circular proxies. Shared guards reject
known exhaustion without establishing total resource bounds. All 39 properties,
all 14 targets and crate-specific future work remain in scope.

## Atomic native IPC annotation and coverage, 2026-10-08

IPC net annotation now returns a typed result and stages borrowed net/diameter
updates until all required native comparisons and the shared STRICT/budget
guard succeed. Failure leaves board metadata unchanged. Existing values and
the first matching record keep priority; a copied diameter must be certified
nonnegative. The CLI propagates annotation failure before checks and reporting.
Native coverage accepts any certified matching center, retains unresolved
candidates when no match is proved, and checks required report projections.
These operations and the unchanged IPC diameter check share `checks/board/ipc`;
their existing public paths remain available through re-exports.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-ipc-native/manifest.json)
retains twelve baseline failures and two controls. Twenty-three final
regressions cover projection panics, partial metadata updates, invalid copied
diameters, unresolved matching, exhausted scopes and retry, first-record
priority, irrelevant missing information, enclosing provisional policy,
blocking coverage receipts, and typed CLI failure without a report summary.
The configured release command passes 1,308 library tests, two README tests and
three doctests. Formatting, strict all-feature/all-target Clippy, strict rustdoc
and fixture compilation pass. Fourteen selected HyperCircuit tests and strict
Clippy pass. All six snapshots change only implementation version 10 to 11.
Sources remain fixed through validation and cost measurement.

Five sequential fresh release processes pinned to CPU 0 measured these complete
operation medians, in microseconds:

| Operation/input | Time |
| --- | ---: |
| Annotation, one copper feature and drill | 1.306 |
| Annotation, huge native tolerance | 1.022 |
| Annotation, unprojectable native centers | 0.953 |
| Annotation, rollback on a later unresolved match | 14.051 |
| Annotation, invalid copied diameter | 1.220 |
| Annotation, 2,002 copper features and 2,002 drills | 978.671 |
| Coverage, exact boundary and nearby distinct point | 1.671 |
| Coverage, unresolved match | 12.473 |
| Coverage, unprojectable native centers | 0.276 |
| Coverage, certified match after an unresolved candidate | 11.854 |
| Coverage, 2,002 copper features | 313.747 |

Annotation timers include a fresh board clone with shared immutable native
geometry. All timers include index construction, required predicates and output
allocation. Input construction and first/last full metadata/error or finding
fingerprints are outside timers; native evidence may be warm. All 71 witness
rows match across trials, including the previous 60 unchanged rows. Output is
7,153 bytes with SHA-256
`206693d04e0527aee3c748ece3c60eaf02fc91967132a355afadf95874d76816`.
Whole-process RSS was 58,992–59,760 KiB, including the retained million-sample
fixture; the linked probe was 24,605,840 bytes. The
[raw cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-ipc-native/cost/manifest.json)
retains every observation. These are capability costs without a matched speedup,
allocation count or library-size comparison.

The unchanged full two-board/Gerber fixture passed in 63.000 seconds against its
120-second gate, with 1,336,444 KiB peak RSS. All nine logged check counts match.
The [report audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-ipc-native/cost/fixture-report-audit.json)
records row-order differences and a changed duplicate-layer finding. The
[known-variant audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-ipc-native/cost/known-order-audit.json)
checks its two layer/marker variants against the previously reproduced full-board
witness. Its remaining header counts, geometry and message, and every other
finding match after path/ID and finding-row normalization. That native witness
was reused, not rerun; original report order and path-independent IDs remain
unproved.

A [syntactic inventory](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-ipc-native/remaining-index-sites.json)
still finds eight legacy point-index and 54 copper-index constructor sites in
check implementations. Their required predicates, remaining drill decisions,
source geometry, width/density proxies and total resource bounds remain open.
The inventory is not a behavioral audit. All 39 properties, all 14 targets and
crate-specific future work remain in scope.

## Deterministic layer and duplicate-finding order, 2026-10-08

KiCad copper layers now use lexical layer-name order. Duplicate-layer checks
sort source names before preparation, so distinct names define failure choice,
pair order and the authored source of optional markers. Authored geometry order
within a layer is retained; equal names keep caller order. Native predicates,
report geometry and marker formulas are unchanged.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-layer-order/manifest.json) retains five baseline failures,
two controls and a fresh full-board reproduction: the two CPArti layers have
native area 10,000 and empty XOR, but reversed input previously changed IDs and
markers. Seven final regressions pass. The configured release command passes
1,315 library tests, two README tests and three doctests; strict all-feature,
all-target Clippy, rustdoc, formatting and fixture compilation pass. Fourteen
selected HyperCircuit tests and strict Clippy pass. All six snapshots change
only implementation version 11 to 12. The first full run's two test failures
and their corrections remain recorded; the source audit was not weakened.

Two fresh processes serialize the complete public Report at identical source
paths for both full KiCad boards and the existing Gerber edge-cuts smoke subset.
Every byte matches, including IDs, coverage, diagnostics, findings and geometry.
No field or row is normalized. Serialized payloads are retained in the
[replay record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-layer-order/report-replay/manifest.json):

| Report | Bytes | SHA-256 |
| --- | ---: | --- |
| cparti-fpga | 439,932 | `26e8162fe449e8b8eac0ab54661404feadb53fabc3aa725443262a2c473eeaa0` |
| hvp109a | 98,882 | `ba9f8d4c363c84b38eb9eb40f429d3f5e2cc12cd384621e2f153d01ad94a5e7e` |
| cparti-gerber | 725 | `c1b6942b5310c39631e8b81331a96cdfa6c01018cdf4519e5926c07328e0c719` |

The repaired full-board duplicate finding equals the baseline's B.Cu-first
variant in both input orders. Against the previous fixture text, every other
finding matches after explicit path/ID/row normalization; the duplicate's
remaining header, geometry and message match as well. These comparisons cover
the retained fixtures and selected checks. Source paths still affect IDs,
and universal serialized determinism remains open.

Five fresh release processes pinned to CPU 0, after builds and tests ended,
produced these complete-operation medians in microseconds:

| Operation/input | Time |
| --- | ---: |
| board_copper_layers_32x32 | 47,084.764 |
| duplicate_layers_forward | 13.937 |
| duplicate_layers_reversed | 13.506 |
| duplicate_layers_source_failure | 0.764 |

The layer projection retains all 1,024 authored features across 32 layers.
Input construction and first/last native/report fingerprints are outside timers;
native evidence may be warm. All 75 witness rows match, including all 71 previous
rows and iteration counts. The payload is 7,565 bytes with SHA-256
`c365e3681ee6c65ab62cd959e3916e5343566bc0cab09a68cf99864a32bd51f6`.
Whole-process peak RSS is 58,940–59,392 KiB, including the prior million-sample
fixture; the linked probe is 24,690,440 bytes. The unchanged two-board/Gerber
fixture passes in 66.278 seconds against its 120-second gate, with
1,387,592 KiB peak RSS and unchanged counts for all nine logged checks.
The [cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-08-hyperdrc-layer-order/cost/manifest.json) retains every trial.
These are capability costs without a matched speedup, allocation count or
library-size comparison. Other geometry consumers, complete report identity,
resource bounds and the full properties/future-work scope remain open.

## Native point consumers and fixed-decimal diagnostics, 2026-10-09

All eight former `PointSpatialIndex` constructor sites across seven readiness
checks now use native center enclosures and checked radius predicates: edge and
chassis stitching, return paths, power-via arrays, differential-via proximity
and return paths, and mouse-bite rows. Native centers remain borrowed until a
finding requires finite report markers. Unknown candidates cannot become
certified absence; any certified matching alternative can prove existence.
Power arrays require distinct native centers, including across copper layers.
Mouse-bite diameter and pitch decisions retain uncertainty, while optional
nearest-bad-pair ordering cannot erase an already certified violation. All seven
checks run under the shared STRICT and work-budget guard, including empty paths.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-point-consumers/manifest.json)
retains 27 baseline failures and four controls. The failures include projection
panics, lost sub-ULP separations, negative radii accepted after squaring,
unresolved thresholds becoming absence, a dropped diameter, and irrelevant
neighbor ordering. Forty-seven final regressions pass. An expanded native
mouse-bite witness exposed a separate diagnostic stall: alternate decimal
formatting searched for a nonzero scientific leading bit in unresolved zero.
A timed baseline probe reaches formatting and exits at its ten-second timeout;
the same native distance renders as `0.000000` with `Real::fixed_decimal()`.
The new borrowed view reuses the existing absolute-precision formatter and
never feeds a predicate. Mouse-bite diagnostic numbers now use six fixed
fractional places. General alternate/scientific formatting remains open.

The complete configured DRC command passes 1,362 library tests, two README tests
and three doctests. Hyperreal's all-feature tests and doctests pass, including
five new format regressions and one new doctest. Its log records 969 successful
test executions, including two child-process precision witnesses. Strict
all-feature/all-target Clippy and rustdoc pass for both crates. Fourteen selected
HyperCircuit tests and strict Clippy pass. All six Easyduino snapshots change
only implementation version 12 to 13. Rust sources remain frozen through the
validation, cost and replay jobs. The first Clippy import error, interrupted
expanded run, missing API-inventory classification and benchmark-only integer
conversion error remain recorded with their corrections.

Two fresh processes run all seven migrated checks on both complete KiCad
fixtures at fixed source paths. Every byte of each public Report matches,
including IDs, coverage, diagnostics, messages, markers and geometry. Against
the verified reused baseline library, only seven literal implementation-version
fields change per board; every other byte matches. The
[complete reports](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-point-consumers/report-replay/manifest.json)
are retained without normalization:

| Report | Findings | Bytes | SHA-256 |
| --- | ---: | ---: | --- |
| cparti-fpga | 62 | 42,864 | `ca2516dfdd2966a4796088190186a9ecfb3dd200f78655fa30a5334268acc923` |
| hvp109a | 8 | 5,667 | `2db9febb3512767d72cc8e4ba5488af927f670d8571badb0df9bbdabf13cda0a` |

Five sequential release processes pinned to CPU 0, after all builds and tests
exited, measured these median complete-check costs in microseconds:

| Check | Huge native centers | Unknown radius | 2,002-record sparse fixture |
| --- | ---: | ---: | ---: |
| Edge stitching | 947.884 | 982.806 | 1,676.423 |
| Chassis stitching | 0.409 | 14.054 | 398.284 |
| Return path | 0.492 | 14.444 | 573.106 |
| Power via arrays | 0.981 | 14.225 | 2,125.607 |
| Differential via proximity | 0.878 | 15.491 | 1,529.392 |
| Differential via return | 1.032 | 15.358 | 512.989 |
| Mouse-bite pitch | 0.801 | 14.582 | 2,246.576 |

The native mouse-bite optional-order finding costs
62.025 microseconds; coincident power-via copies cost
2.436 microseconds. The three fixed-decimal rows cost
0.295, 0.720 and
0.601 microseconds for unresolved zero distance, pi and tiny pi.
Board/expression construction and first/last output fingerprints are outside
timers; queries include small fixed rule-argument construction and native
evidence may be warm. All 110 witness rows match across trials, retaining every
one of the previous 75 rows and iteration counts. The payload is
11,031 bytes with SHA-256
`87f07643824c7cc270b0b1d4f47f63cd5f126c86fc1c159b35619408f5e345b6`.
Whole-process peak RSS is 59,172–59,672 KiB, including the prior million-sample
fixture; the probe is 24,894,960 bytes. These are candidate capability costs,
without a matched speedup, allocation count or library-size comparison. The
[cost manifest](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-point-consumers/cost/manifest.json)
retains every trial, binary hash and input definition.

The unchanged two-board/Gerber smoke fixture passes in 64.782 seconds against
its 120-second gate, with 1,387,736 KiB peak RSS. All nine logged check counts
and complete rendered reports remain unchanged after the explicitly retained
path/ID normalization; no row or marker sorting is needed for this comparison.
The [constructor inventory](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-point-consumers/constructor-inventory.json)
finds zero legacy point-index sites and 54 remaining copper-index sites. This is
a syntactic inventory, not a complete audit of all geometric consumers. Source
adapters, width/density proxies, other drill/distribution logic, universal report
identity, total resource bounds and all remaining properties/future work stay
open. No full pre-push workflow matrix or push was performed.

## Native authored-center consumers, 2026-10-09

Six readiness checks now use native authored centers through the shared point
enclosure grid: connector return paths, decoupling proximity, RF via fences,
plating intent, mounting-hole grounding and mounting-hole plating intent.
Layer/net/kind metadata is checked before required center matching. Native
squared-radius predicates decide the closed boundary; unknown candidates stay
uncertain unless another candidate proves a match. Required markers use checked
finite projection. All six checks retain STRICT and shared-budget failures,
including empty and warm paths. Per-query native radii reuse the same enclosure
index for expanded NPTH and mounting-hole searches.

Mounting-size eligibility checks both diameter bounds without requiring an order
between the clearance hint and one. A certified too-small bound proves
ineligibility even if another bound is unresolved. An explicit ground/chassis
net proves plated mounting intent without optional geometry. The shared checked
selector also propagates uncertainty to mounting-hole copper keepout and edge
clearance; their other geometry/projection paths remain open. Copper carriers
are independent of authored centers, except the connector check's required
native edge selection. Required connector, decoupling and mounting diagnostic
numbers use the explicit six-place fixed-decimal view.

The [checkpoint record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-authored-centers/manifest.json)
retains 28 baseline failures and five controls, reconstructible from 122 Rust
source hashes. Failures include coordinate/radius projection panics, unresolved
radius and diameter decisions becoming ordinary findings or ineligibility,
invalid NPTH diameter handling and required marker failures. All 49 final
regressions pass, including enclosing preview, exhausted warm/empty scopes,
retry, readiness receipts, metadata filters, sub-ULP separations, variable query
radii, unused shape failures and unresolved-zero diagnostics. The full DRC
command passes 1,411 library tests, two README tests and three doctests. Strict
all-feature/all-target Clippy, strict rustdoc and workspace formatting pass.
Fourteen selected HyperCircuit tests and strict Clippy pass. All six Easyduino
snapshots change only implementation version 13 to 14. The README tests pass
again after the prose update; Rust sources remain frozen through validation,
cost measurement and report replay.

Two fresh processes run all eight affected checks on both complete KiCad boards
at fixed input paths. Their complete public Reports match byte for byte. The
initial version-only baseline gate correctly rejected six CPArti connector
message changes: `0.000000e0` becomes `0.000000` through explicit fixed-decimal
presentation. The [exact delta audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-authored-centers/report-replay/baseline-delta-audit.json)
permits those six specific messages and eight version fields per board.
Every other byte, including finding IDs, order, coverage, markers and geometry,
is preserved. The original failed gate, raw trial manifest and complete
compressed reports remain retained. Candidate reports:

| Report | Findings | Bytes | SHA-256 |
| --- | ---: | ---: | --- |
| cparti-fpga | 946 | 398,899 | `d8517f89ec4aafc797e54c9a10a692ffd5f45ced25a37a872cc34ad1fab49219` |
| hvp109a | 153 | 67,267 | `db67df939dd8fd320f587469ac65f7f30bd9664f2bfac232412b67cd01256c94` |

Five sequential fresh release processes pinned to CPU 0, after builds/tests
exited, measured these median complete-check costs in microseconds:

| Check | Huge native centers | Unknown radius | 2,002-record sparse fixture |
| --- | ---: | ---: | ---: |
| Connector return | 9.488 | 26.817 | 777.537 |
| Decoupling proximity | 0.485 | 14.882 | 526.975 |
| RF via fence | 0.467 | 14.551 | 464.835 |
| Plating intent | 0.338 | 14.915 | 317.058 |
| Mounting-hole grounding | 0.540 | 21.671 | 373.425 |
| Mounting-hole plating intent | 0.584 | 22.087 | 360.146 |

All 139 output witnesses match across trials, preserving every previous 110
witness and iteration count. The payload is 14,080 bytes, SHA-256
`185d3c0c25222fba0a47d155bf3dc437160f5b42c340c167e6cc045122a262cc`.
Additional rows cover huge radii, closed NPTH matching, explicit plated ground
intent, unordered mounting thresholds, uncertain connector edge selection and
a negative radius. Board/expression construction and first/last fingerprints
are outside timers; query argument construction is included and native evidence
may be warm. Whole-process peak RSS is 59,412–59,624 KiB, including the retained
million-sample geometry. The linked probe is 25,035,192 bytes. These are candidate
capability costs, with no matched speedup, allocation count or library-size
comparison. The [cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-authored-centers/cost/manifest.json)
retains raw trials, source and binary hashes.

The unchanged full two-board/Gerber smoke fixture passes in 65.943 seconds
against its 120-second gate, with 1,387,960 KiB peak RSS. Its nine ordered check
counts and complete rendered reports are unchanged after the explicitly
recorded temporary-path/ID normalization; no finding or marker sorting is
needed. The [constructor inventory](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-authored-centers/constructor-inventory.json)
records 54 to 48 legacy copper-index sites and 14 to 19 native point-index sites;
a shared bonding constructor serves two public checks. This is a syntactic
inventory. Derived cluster centers, remaining copper-shape indices and required
predicates/projections, other drill/distribution logic, source adapters,
width/density proxies, general alternate/scientific formatting, universal report
identity and total resource bounds remain open. All 39 properties, 14 targets
and crate future work remain in scope. No push or full pre-push matrix was run.

## Native dense-cluster consumers, 2026-10-09

All four dense-pad readiness checks now qualify clusters with native pitch and
native averaged authored centers. Local registration uses checked native
fiducial dimensions and counts; escape review accepts certified nearby vias.
Via/pad and mask-web spacing use native copper-boundary metrics and a shared
shape-enclosure index independent of authored centers. Missing optional
enclosures retain candidates. Required predicates, source failures and report
markers retain typed uncertainty under the shared STRICT/budget guard, including
empty and warm paths.

A certified qualifying pitch or clearance proves existence even when another
candidate or optional minimum order is unavailable. The diagnostic says
“certified pitch/clearance” unless every comparison needed to claim the minimum
was certified. Two certified fiducials prove sufficient registration despite
unknown alternatives; a reported count below two must be exact. Fiducial
eligibility uses six native inequalities, avoiding unnecessary width/height
ordering and division. A certified exclusion can settle eligibility after an
unknown dimension comparison. Report numbers use the explicit six-place
fixed-decimal view and never decide geometry.

The [checkpoint record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-dense-clusters/manifest.json)
retains 19 failures and two passing controls reproduced at `e0e2a984d`, with all
126 baseline source hashes reconstructed. The final 38 regressions cover large
centers, required markers, unresolved pitch/radius/dimensions, qualified
witnesses, retained source failures, closed shape contacts below a float ULP,
empty shapes, layer eligibility, negative radii, budget exhaustion/retry and
blocking readiness receipts. Together with the 13 existing dense-pad tests,
they pass 51 focused tests. The full gate passes 1,454 tests: 1,449 library tests,
two README tests and three doctests. Strict all-feature/all-target Clippy,
strict rustdoc and the fixture build pass. HyperCircuit passes strict Clippy and
14 selected tests. All six Easyduino snapshots differ only in their 160 shared
check-version fields, from 14 to 15. Rust sources remain fixed from final
validation through cost and complete-report replay.

The expanded suite initially exposed a finite-source rectangle limitation:
a width of 16,384 at x=1e20 rounds its corners together before native admission,
retaining an insufficient-vertices construction failure. The original staged
source and failed test remain in the
[fixture correction record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-dense-clusters/expanded-validation/fixture-correction.json).
The grid test now constructs two unit-wide rectangles directly from native
vertices at 1e20 and 1e20+1, preserving their exact closed contact. Cost probes
also retain the original finite construction failure. This checkpoint repairs
the consumers and grid; finite rectangle source arithmetic remains open.
A standalone cost-driver compile failure from calling a private method is
retained separately; the corrected probe uses public checked bounds.

Complete public reports for all four checks on the full CPArti and HVP109A
boards are byte-identical across two fresh processes at fixed input paths.
The initial version-only gate failed on presentation changes. The
[exact delta audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-dense-clusters/report-replay/baseline-delta-audit.json)
permits exactly four version fields per board plus twelve CPArti and three
HVP109A messages changing zero from `0.000000e0` to `0.000000`.
Every other report byte, including findings, IDs, geometry, markers, receipts
and ordering, is preserved. This covers these checks and these full fixtures;
it does not establish universal or path-independent report determinism.

Five sequential fresh release processes pinned to CPU 0 produce 166 identical
witness rows. All previous 139 rows and iteration counts are unchanged; the 27
new rows cover the dense-cluster consumers. The identity artifact is 16,868
bytes with SHA-256
`a8670bc33482de589c9824ff08d8d2d479f32f7e4d9b9aa749794998bae753a4`.
Medians below are microseconds per query:

| Check | Huge centers | Huge pitch threshold | Unknown pitch | 2,002-record sparse fixture |
| --- | ---: | ---: | ---: | ---: |
| Local fiducials | 32.773 | 23.109 | 147.743 | 1,290.248 |
| Escape vias | 31.007 | 20.871 | 146.535 | 1,289.610 |
| Via/pad spacing | 40.432 | 30.698 | 146.029 | 2,607.109 |
| Mask web | 43.609 | 31.676 | 147.843 | 3,248.109 |

The sparse cases retain 2,000 pads plus two fiducials, vias or ignored artwork
records. Separate huge search-radius rows exercise only the three rules that
use that argument; the mask-web rule does not. Additional medians:

| Query | µs/query |
| --- | ---: |
| Fiducials, huge search radius | 11.646 |
| Escape vias, huge search radius | 9.514 |
| Via/pad spacing, huge search radius | 18.726 |
| Fiducial dimension below 0.5 by 1e-50 | 14.650 |
| Fiducials, unknown radius | 32.547 |
| Two certified fiducials after unknown candidate | 25.949 |
| Dense pitch with optional minimum order unknown | 244.949 |
| Via/pad qualified clearance witness | 506.826 |
| Mask-web qualified clearance witness | 32.417 |
| Via/pad required shape failure | 20.283 |
| Mask-web required shape failure | 24.406 |

Board/expression construction and first/last fingerprints are outside timers;
queries include small fixed rule-argument construction and native evidence may
be warm. Process peak RSS ranges from 59,404 to 59,720 KiB and includes
the prior million-sample geometry. These are capability costs without a matched
speedup, allocation count or library-size comparison. The
[cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-dense-clusters/cost/manifest.json)
retains raw trials and source/binary hashes.

The unchanged full two-board/Gerber smoke fixture passes in 67.049 seconds
against its 120-second gate, with 1,388,100 KiB peak RSS. Its nine ordered check
counts and rendered reports remain unchanged after the recorded temporary-path
and ID normalization; no finding or marker sorting is needed. The
[constructor inventory](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-dense-clusters/constructor-inventory.json)
records legacy copper-index sites falling from 48 to 43, native point sites
rising from 19 to 23, and two new native shape-index sites. The shared native
pitch constructor serves all four dense checks. This is a syntactic inventory,
not complete consumer coverage. Thermal copper-area readiness still matches
authored power-feature/zone centers through a legacy path. IPC side parity and
accessibility, remaining copper-shape queries, source constructors, required
projections, width/density proxies, report identity and total resource bounds
remain open. All 39 properties, 14 targets and crate future work remain in scope.
No push or full repository pre-push matrix was run.

## Native fixture-access consumers, 2026-10-09

Thermal copper-area readiness and all three IPC fixture checks now preserve
native evidence and typed uncertainty. Thermal matching uses authored power
feature and zone centers after layer/net eligibility; it remains a placement
proxy, not a heat-spreading area measurement. Net-name coverage projects only
the first three required markers for a missing critical net. Covered nets and
unused markers need no projection.

Accessibility uses native diameter, side-parity, outline and spacing predicates.
Effective probe size is represented by alternative radii, avoiding an unrelated
maximum-order certificate. A side mismatch requires both certified opposite-side
presence and certified requested-side absence. A certified requested-side match
or certified opposite-side absence settles the non-mismatch case despite unknown
alternatives. Shared native circle envelopes preserve original source indices,
including missing diameters, and retain candidates when optional bounds fail;
the existing drill index delegates to the same implementation. Spacing findings
follow authored point-pair order. Native hypot is required only for their gap
diagnostic, after the strict radius predicate proves the finding.

Copper keepout uses native shape enclosures independent of authored feature
markers. Same-net and certified empty geometry do not require unused probe
predicates. Required source, classification, distance and marker failures retain
uncertainty instead of silently omitting candidates or emitting an asserted hit.
All four checks enforce shared STRICT/budget guards, including empty and warm
paths. Numeric displays use six-place fixed decimals and do not decide geometry.

The [checkpoint record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-fixture-access/manifest.json)
retains 20 failures and four passing controls at `3b3fd8c34`, with 129 baseline
source hashes reconstructed. The final 48 regressions and 39 existing IPC,
thermal and drill tests pass. The full release DRC gate passes 1,502 tests:
1,497 library tests, two README tests and three doctests. Strict
all-feature/all-target Clippy, strict rustdoc and the fixture build pass.
HyperCircuit passes strict Clippy and 14 selected tests. All six Easyduino
snapshots change only 160 shared check-version fields each, from 15 to 16.
The 134 DRC Rust sources remain frozen through final validation, costs and
complete-report replay. Initial compiler warnings and a subsequent Clippy
`collapsible_if` failure are retained with their corrections.

Complete public reports for these four checks on the full CPArti and HVP109A
boards are byte-identical across two fresh processes at fixed input paths.
Both runs use the retained synthetic seven-record IPC sidecar, not an original
board-exported IPC file. The initial version-only comparison failed on numeric
display changes. The
[exact delta audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-fixture-access/report-replay/baseline-delta-audit.json)
permits four version fields per board plus 2,412 CPArti and 426 HVP109A messages.
Each board has 23 accessibility display changes, including ten gaps independently
checked from the sidecar; the remaining changes render the unchanged thermal
radius as `1.200000` instead of `1 1/5`. Rational and symbolic gap displays become
decimals, such as `0.078062` instead of `1 x Irrational`. Every other report byte,
including finding counts, IDs, geometry, markers, receipts and ordering, is
preserved. This establishes neither universal nor path-independent determinism.

Five sequential fresh release processes pinned to CPU 0 produce 193 identical
witness rows. All previous 166 rows and iteration counts are unchanged; the
27 new rows exercise these consumers. The identity artifact is 19,723 bytes with
SHA-256 `54adc61223ce7a47678f140d1fc2ed5d638ce31f64878ca9099936c3f814f7a8`.
Selected medians are microseconds per query; all rows remain in the
[cost record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-fixture-access/cost/manifest.json).

| Query | µs/query |
| --- | ---: |
| Thermal, huge centers | 0.406 |
| Thermal, unknown radius | 14.415 |
| Thermal, certified alternative after unknown | 12.400 |
| Thermal, 2,002 sparse features | 427.112 |
| Coverage, covered huge marker | 0.185 |
| Coverage, required unavailable marker | 0.836 |
| Coverage, 2,000 features plus two IPC records | 203.301 |
| Accessibility, unknown side match | 22.492 |
| Accessibility, huge diameters | 25.183 |
| Accessibility, spacing below a float ULP | 7.968 |
| Accessibility, 2,002 sparse IPC points | 4,970.609 |
| Side parity, 2,001 features plus one IPC record | 571.668 |
| Copper keepout, huge radius | 27.064 |
| Copper keepout, unknown clearance | 53.961 |
| Copper keepout, unneeded maximum ordering | 6.233 |
| Copper keepout, required failed source shape | 1.755 |
| Copper keepout, 2,001 features plus one IPC record | 1,126.243 |

Input construction and first/last fingerprints are outside timers; queries
include small fixed rule-argument construction and native evidence may be warm.
Process peak RSS ranges from 62,684 to 62,912 KiB and includes the previous
million-sample geometry. These are capability costs without a matched speedup,
allocation count or library-size comparison. The unchanged full two-board/Gerber
smoke fixture passes in 70.038 seconds against its 120-second gate, with
1,387,708 KiB peak RSS. Its nine ordered check counts and rendered reports remain
unchanged after the recorded temporary-path and ID normalization; finding and
marker sorting are unnecessary.

The [constructor inventory](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-fixture-access/constructor-inventory.json)
records legacy copper-index sites falling from 43 to 40, native point sites
rising from 23 to 25, native shape sites from two to three and two native circle
sites. The inventory filter now includes production `testpoint.rs` and excludes
regression subdirectories; it remains a syntactic inventory, not full behavior
coverage. The finite rectangle source failure at x=1e20 is still retained,
including the actual failed shape in a new copper probe. Its repair, remaining
shape consumers and source adapters, width/density proxies, general formatting,
report identity and total resource bounds remain open. All 39 properties,
14 targets and crate future work remain in scope. No push or complete repository
pre-push workflow matrix was run.

## Native rectangle construction, 2026-10-09

`rect_polygon` now imports finite center, size and degree inputs into the native
scalar domain before halving dimensions, rotating or translating. Report corners
retain their historical finite calculation, but cannot decide the native shape.
A one-unit rectangle at x=1e20, a minimum-subnormal dimension and corners beyond
binary64's range remain valid native geometry even when the report view collapses
or is unavailable. Signed dimensions denote the same regularized rectangle;
an authored zero dimension is certified empty. Construction, empty paths and warm
measurements enforce the shared STRICT and budget contract.

The original hard case remains at x=1e20 with width and height 16384. Its native
area is exactly 268435456, and adjacent imported centers share an exact closed
edge. No coordinate or dimension was reduced to make the case pass. KiCad now
keeps a missing or unavailable primitive size as a retained source failure.
A finite zero must be backed by an exact source-zero certificate: a nonzero
`1e-400` dimension cannot silently disappear after float underflow. Admitted
custom primitives do not require an unused size.

The [checkpoint record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-native-rectangles/manifest.json)
retains nine failing tests and three passing controls against `5e94512ea`, with
135 baseline source hashes reconstructed. Twenty rectangle and six KiCad
regressions were added; the final focused gate passes 20 rectangle and 42 KiCad
tests. The full release DRC gate passes 1,528 tests: 1,523 library tests, two
README tests and three doctests. Strict all-feature/all-target Clippy, strict
rustdoc, formatting and fixture build pass. HyperCircuit passes strict Clippy
and 14 selected tests. Each of the six Easyduino snapshots changes only its
160 check-version fields from 16 to 17. All 137 DRC Rust sources remain frozen
through final validation, costs and report replay.

Retained intermediate failures include a test closure compile error, two
existing KiCad tests exposing the missing-size/zero-size distinction, and two
doctest linkage failures caused by overlapping Cargo feature builds overwriting
the un-hashed `libcsgrs.rlib`. The final validation phases run sequentially.
After an earlier 1,527-test pass, a public probe found the `1e-400` size omission;
the final parser fix, added regression and successful public replay are retained.
These records distinguish the final tested sources from earlier green runs.

Five sequential release processes pinned to CPU 0 produce 205 identical witness
rows. All 193 prior workload inputs and iteration counts remain; 188 rows are
unchanged. The same original large rectangle now certifies, removing three
construction-uncertainty findings and changing two diagnostics from `certified`
to `nearest`. The [row audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-native-rectangles/cost/row-delta-audit.json)
replays all five cases against both verified libraries and checks their complete
serialized findings against the measured witnesses. Twelve new rows measure
complete rectangle construction or warm measurements, with exact independent
area/bounds assertions and source-bit witnesses. The identity artifact contains
21,727 bytes, SHA-256
`83b3ee792a0f82553d2cac4da93e0c5d1988fe282bc23b17c04a40bedba7dff7`.

| Query | Median µs/query |
| --- | ---: |
| Construct unit rectangle | 27.597 |
| Construct original rectangle at x=1e20 | 44.234 |
| Construct one-unit rectangle at x=1e20 | 37.185 |
| Construct minimum-subnormal dimension | 38.686 |
| Construct corners beyond binary64 | 54.685 |
| Construct 30-degree rectangle | 132.649 |
| Construct zero-dimension rectangle | 0.097 |
| Warm original rectangle bounds | 0.079 |
| Warm original rectangle area | 0.025 |

First/last assertions are outside the timers; construction rows include the
native constructor itself. Process peak RSS ranges from 60,576 to 60,772 KiB
and includes earlier million-sample geometry. These are capability costs,
without a matched speedup, allocation-count or library-size claim. The unchanged
two-board/Gerber fixture passes in 68.130 seconds against its 120-second gate,
with 1,387,232 KiB peak RSS. Its nine ordered check counts and complete rendered
findings remain unchanged after the recorded temporary-path/ID normalization.

Complete reports for twelve selected smoke, dense, thermal and IPC checks repeat
byte-for-byte across two fresh processes on both full boards at fixed paths.
The same synthetic seven-record IPC sidecar is retained. CPArti keeps 2,610
findings and changes only twelve version fields. HVP109A keeps 658 findings;
its initial version-only comparison fails on three findings. The
[exact report audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-native-rectangles/report-replay/baseline-delta-audit.json)
accepts twelve version fields, two via-overlap report polygons with four area
fields and six coordinate components changed, and one selected mask pair with
four marker components and both IDs changed. Direct native replay reproduces
both old and new overlap polygons. Exhaustive native segment distances show
that the former mask pair now has a gap of exactly 3/2^56 mm; the newly selected
pair still touches exactly. Independent rational reconstruction verifies the
three affected rectangles. All diagnostics, counts and every other complete
report byte remain unchanged. No geometric tolerance or broad normalization
accepts these changes.

This repairs the finite rectangle API boundary. Other finite primitives,
including rounded/chamfered rectangles, trapezoids, lines and sampled strokes,
remain open, as do parser token-to-float paths, other primitive size arithmetic
and authored-center rounding. Forty legacy copper-index constructor sites,
remaining shape/drill/distribution consumers, scalar thresholds, report-chord
widths and density proxies still require audit. General scalar trigonometric
identities, total operation resource bounds and universal/path-independent
report identity are not established. All 39 properties, 14 targets and crate
future work remain in scope. No push or complete pre-push workflow matrix was run.

## Native pad primitive construction, 2026-10-09

Rounded and chamfered rectangles and trapezoids now import finite source
parameters before half-size arithmetic, clamping, corner construction, rotation
and translation. Rounded corners remain native circular arcs independently of
report sample count. Native clamping preserves limiting capsules, circles,
chamfer diamonds and subnormal dimensions. Invalid inputs and excessive
trapezoid deltas retain failures; valid zero dimensions denote empty geometry.
Construction and warm queries retain STRICT and shared-budget guards.

Trapezoids use half the opposing-edge delta and the corrected lower-edge signs,
following the [primary KiCad source references](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-native-pad-primitives/reference-sources.json).
The finite API imports binary64 values exactly; it does not imitate KiCad's
integer-unit truncation. Report drawings stay separate from native geometry,
including when finite corners collapse or cannot be represented. The original
x=1e20, size 16384 case remains unchanged for all three shape families.

The [checkpoint record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-native-pad-primitives/manifest.json)
retains 16 failing baseline tests and four passing controls against `7f922aca6`,
with all 138 baseline source hashes reconstructed. Thirty-three new regressions
cover 28 primitive cases, three KiCad handoffs and two overlap-bound cases.
The final release gate passes 1,561 tests: 1,556 library, two README and three
doctests. Strict all-feature/all-target Clippy, strict rustdoc, formatting and
fixture build pass. HyperCircuit passes strict Clippy and 14 selected tests.
Each of six Easyduino snapshots changes only 160 version fields from 17 to 18.
All 141 DRC Rust sources remain fixed through the final validation and replay.

The initial 1,559-test green suite missed a completeness regression on the full
CPArti board. The J1 A5/A6 rounded-pad intersection retains exact circular
algebraic fragments whose area integral is unavailable. Duplicate detection now
uses the intersection of native bounding boxes as an optional area upper bound:
if it cannot contain enough area to meet either existing duplicate tolerance,
the pair is certified non-duplicate. Unavailable bounds or comparisons retain
the required intersection path. Making this bound mandatory initially failed an
existing optional-marker regression (1,555 passed, one failed); that run and its
exact source bytes are preserved. The corrected focused gate includes all
24 bounds regressions.

The [original-pair replay](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-native-pad-primitives/area-bound/manifest.json)
links both verified libraries against the same full board. Its native geometry
and unavailable integral remain unchanged while the duplicate decision becomes
certified. An independent rational reconstruction of the input boxes, together
with the strict lower bound pi > 3 for rounded area, proves the cap is below the
existing required overlap. General retained circular-fragment integration is
still open; the check no longer requires it for this decidable pair.

Five fresh sequential release processes pinned to CPU 0 produce 231 identical
correctness rows. All 205 previous inputs, names, iteration counts and complete
witnesses remain unchanged. Twenty new construction rows and six warm queries
carry independent exact area/bounds assertions and input/report fingerprints.
The identity artifact contains 26,801 bytes, SHA-256
`5fa8936b39564a0ac9158917935e4d43a8e36a5c1359cd1ff36474fc5ac27c4b`.

| Query | Median µs/query |
| --- | ---: |
| Rounded construction, four samples/corner | 145.768 |
| Rounded construction, 128 samples/corner | 151.462 |
| Rounded construction, original x=1e20 scale | 183.610 |
| Rounded construction, subnormal dimensions | 209.544 |
| Rounded construction, unavailable finite corners | 1069.293 |
| Chamfer construction, selected corners | 109.699 |
| Chamfer construction, original x=1e20 scale | 181.363 |
| Trapezoid construction, half-delta convention | 60.809 |
| Trapezoid construction, limiting triangle | 53.455 |
| Trapezoid construction, rejected excessive delta | 0.573 |
| Rounded warm area, four samples/corner | 0.029 |
| Rounded warm bounds, four samples/corner | 0.083 |

First/last assertions are outside timers; construction rows include native
construction. Process peak RSS spans 60,620–61,108 KiB and includes
earlier million-sample geometry. These are capability costs without a matched
speedup, allocation-count or library-size claim. The unchanged full two-board
and Gerber fixture passes in 62.834 seconds against its 120-second gate,
with 917,200 KiB peak RSS. Its nine ordered check counts and rendered findings
remain identical after the recorded temporary-path/ID normalization.

Complete reports for twelve selected checks repeat byte-for-byte across two
fresh processes on both full boards at the existing fixed paths and with the
same synthetic seven-record IPC sidecar. CPArti retains 2,610 findings and
HVP109A retains 658. Each report changes only twelve version fields from 17 to
18: every finding, marker, coordinate, ID and other serialized byte remains
unchanged. The rejected initial reports and their failed comparison remain in
`pre-area-fix`; they are not accepted as final evidence. This does not establish
universal or path-independent report identity.

Parser ratio products, malformed optional tokens, combined chamfer/rounding
intent, finite placement, circle/oval size arithmetic, line normalization and
sampled arc/Bezier strokes remain open. Forty legacy copper-index sites,
remaining shape/drill/distribution consumers, scalar thresholds, report-chord
widths and density proxies still require audit. General scalar identities,
retained circular integration and total resource/report-allocation bounds are
not established. All 39 properties, 14 targets and crate future work remain in
scope. No push or complete repository pre-push workflow matrix was run.

## Retained optimization

The regular-grid broad phases in `checks::spatial` previously stored exact cell
keys in `BTreeMap`s. Bentley establishes the value of multidimensional search
structures, while Teschner et al. show that a regular spatial grid can be
compressed efficiently with hashing. HyperDRC already had the conservative grid,
cell-size inflation, and exact narrow phase; the retained change substitutes
`HashMap` only for exact bucket-key lookup.

Candidate report order does not depend on hash iteration. Queries probe cells in
explicit coordinate order, bucket members retain insertion order, and every
returned candidate vector is sorted and deduplicated before use. A regression test
rebuilds randomized hash tables and checks the same sorted IDs. Rust's map also
resolves hash collisions by key equality, so unrelated cells cannot alias at the
API boundary.

The isolated release benchmark constructs 10,003 drills, including a populated
nearby cluster, and runs `drill_spacing` 200 times. Five sequential runs were used
for each median on 2026-07-15:

| Storage | Runs (ms) | Median | Change |
| --- | --- | ---: | ---: |
| Ordered grid buckets | 1615.162, 1618.816, 1605.697, 1593.236, 1605.558 | 1605.697 ms | baseline |
| Hashed exact grid buckets | 1268.479, 1261.417, 1260.223, 1257.716, 1272.318 | 1261.417 ms | 21.44% faster |

Reproduce with `cargo bench --bench spatial_index_audit`. The benchmark is kept
separate from `parser_geometry_smoke` so parser, CSG, and report costs cannot hide
the bucket-storage effect. During validation the broad target exposed uncertified
mask offset-ring difference and intersection cases. Those helpers now retain the
expanded cover or subject island conservatively instead of aborting; that safety
hardening is not counted as a speedup.

## Reference-by-reference disposition

### Algorithms, geometry, data, and reporting

| Reference | Current target and disposition |
| --- | --- |
| Andrew, 2D convex hull | Directly implemented as the exact monotone-chain reduction in `checks::spread`; sorting and orientation predicates retain exact `Scalar` coordinates. |
| Apache Arrow columnar format | `arrow_report` writes the shared report model; it is an output adapter and does not justify duplicating analysis or geometry. |
| Apache Parquet | `parquet_report` serializes the Arrow-shaped report; compression/encoding belongs to the library and was not reimplemented. |
| Bentley, multidimensional binary search trees | The applicable associative-search idea is retained as private spatial broad phases. A k-d tree was not added alongside the existing conservative variable-radius grid because it would duplicate indexing without removing the exact narrow phase. |
| Ericson, *Real-Time Collision Detection* | AABB rejection, uniform-grid candidates, and exact narrow-phase replay are retained throughout `checks::spatial`, `distance`, and geometry checks. |
| Farin, CAGD | Bezier and arc construction is isolated in geometry/KiCad adapters; exact curve ownership is delegated to [Hypercurve](https://github.com/timschmidt/hypercurve). Flattened CAM checks do not reconstruct curves from polygons. |
| GitHub workflow commands | `github_annotations` emits CI annotations from the already-built report; no analysis pass is repeated per sink. |
| Hinnant, date algorithms | Date parsing and freshness evidence live in `date`/artifact checks. Date arithmetic is not a geometry hot path. |
| KiCad S-expression format | `sexp` and `kicad` retain parsed board intent and exact decimal scalars. Parser throughput is measured, but replacing the format model with an ad-hoc scanner was rejected as an evidence loss. |
| Lee and Preparata, computational geometry survey | Supports the separation of hull, distance, intersection, and spatial-search primitives. Those remain private helpers behind PCB-specific findings. |
| Lin and Canny, incremental distance | Temporal coherence is not present in a static release-package run. Cached simplex state would need a stable object lifecycle that the current immutable report pipeline does not expose, so it is deferred. |
| OASIS SARIF 2.1.0 | `sarif` maps stable findings into the standard. It consumes the shared report and does not rerun checks. |
| Parnas, module decomposition | Parser, policy, check, geometry, and report ownership are kept in separate modules; the spatial implementation remains private so public behavior is PCB-domain behavior. |
| Teschner et al., spatial hashing | Newly researched and directly tested. Exact tuple-key hashing for the existing regular grid is retained by the 21.44% result above; custom modulo tables and collision chains were unnecessary because `HashMap` already preserves key identity. |
| Toussaint, rotating calipers | Directly implemented after Andrew hull reduction in `checks::spread`, replacing all-pairs point-set diameter work while retaining exact distances. |
| Yap, exact geometric computation | Exact source decimals, `Scalar` decisions, certified comparisons, and named float projections define the proof boundary. The retained hash change only changes candidate storage. |

### PCB, assembly, electrical, and manufacturing research

| Reference | Current target and disposition |
| --- | --- |
| Areny et al., solder-paste transfer | Paste area/coverage and stencil readiness retain configurable evidence. Reflow physics is a process input, not inferred from flattened geometry. |
| Becerra et al., press-fit process | Press-fit keepout and package-handoff checks retain the relevant geometry and process declarations; force/compliance simulation is outside this reviewer. |
| Bhargava et al., buck-converter EMI layout | Switch-node, inductor, return-path, and power-converter geometry checks implement review prompts. Electromagnetic field solving is deferred to `hyperphysics` or external tools. |
| Black, electromigration | High-current width/neck and current-policy checks expose geometry risk. Lifetime prediction needs material, temperature, and current-density evidence not present in Gerber alone. |
| Chen and Lee, trapped no-clean flux | Paste/via exposure and production-artifact checks retain review evidence; chemical drying is not claimed from planar polygons. |
| Chesser and Porley, mixed-signal PCB layout | Mixed-signal partition, sensitive-net spacing, guard, and return-path checks are implemented as conservative layout prompts. |
| Cohn, shielded strip transmission line | The current impedance screen does not model shielded-strip geometry. Adding the equation without complete stackup/shield evidence would create false authority, so it is deferred. |
| Eurocircuits, tombstoning | Neighbor-pad and paste-imbalance checks directly cover asymmetric deposits while retaining configurable thresholds. |
| FixturFab, design for test | Testpoint diameter, spacing, accessibility, edge clearance, and IPC-D-356 coverage checks are implemented. Fixture mechanics remain external evidence. |
| Hammerstad and Jensen, microstrip | The quasi-static single-ended outer-microstrip estimate is directly implemented in `checks::impedance` with exact inputs and an explicit first-pass limitation. |
| Harter et al., stencil area shape/ratio | Stencil area-ratio, aspect-ratio, minimum aperture, and windowpane checks retain the applicable planar metrics. |
| Hollstein et al., QFN thermal analysis | Thermal-pad via, via distribution, copper-area, paste-windowpane, and mechanical keepout checks expose layout inputs; coupled simulation is not approximated. |
| Kirschning and Jansen, coupled microstrip | Frequency-dependent coupled-line equations require differential geometry, frequency, loss, and stackup detail beyond the current single-ended screen. Deferred rather than exposed as a universal rule. |
| Oezkoek et al., ENEPIG/ENEP bonding | Surface-finish compatibility and handoff evidence are checked; bond-process qualification remains a manufacturer decision. |
| Paterson and Tinker, type size | Silkscreen text-height readiness uses disconnected-island bounds only as a documented readability proxy because flattened Gerber has lost font/glyph semantics. |
| Chin and Ramakrishna, BGA escape traces | Dense-pad escape, via proximity, pad/via spacing, mask-web, and local-fiducial checks are implemented. Solder-joint simulation is deferred. |
| Jonnalagadda, via-in-pad fatigue | Via-in-pad exposure/plating-intent checks identify the configuration; fatigue life requires mechanical/process evidence and is not synthesized. |
| Lee et al., pulse-reverse copper plating | Copper density/balance and fabrication handoff checks expose plating-uniformity inputs. Process waveform and material-property simulation are external. |
| ST AN576, PCB layout and ESD | ESD protection distance, edge/keepout, and return-path checks implement conservative placement prompts. Surge waveform simulation is not inferred. |
| Sun et al., pattern-plating uniformity | Local copper-density windows and whole-layer balance retain the applicable geometric proxies. Multiphysics plating simulation is deferred. |
| Tang et al., fine-line wet etching | Minimum width, neck, acid-trap, and local-density checks expose etch-risk geometry with explicit fabricator thresholds. |
| Wilcoxon et al., QFN thermal-pad voiding | Thermal-pad paste windowpane, via exposure, and coverage checks retain the applicable stencil/layout evidence; void fraction is not predicted. |
| Wheeler, centered stripline | The zero-thickness centered single-ended stripline estimate is directly implemented and named as a first-pass readiness model. |
| Wong et al., small antennas | Antenna copper keepout, RF launch, and via-fence checks provide geometry review. Radiation efficiency requires a field solver. |
| Xu and Wang, guard trace ring | Sensitive-net guard/return proximity and mixed-signal checks implement placement evidence without claiming crosstalk magnitude. |

### Standards and interchange specifications

| Reference | Current target and disposition |
| --- | --- |
| IPC-2221B | Governs configurable clearance, stackup, conductor, and general board-design review; values stay policy inputs rather than hidden universal constants. |
| IPC-2152 | Current-carrying readiness uses declared current/width/stackup evidence. Thermal-current solving is deliberately not reconstructed from copper alone. |
| IPC-D-356B | `ipc356` parses electrical-test points, nets, diameters, access, and diagnostics; cross-source coverage and drill checks consume the retained report. |
| IPC-NC-349 | Excellon/router command and routed-slot readiness preserve parsing diagnostics and unsupported-command evidence. |
| IPC-7351B | Land-pattern, dense-pad, courtyard/component, paste, and assembly policies use configurable thresholds; package-specific qualification remains external. |
| IPC-2581C | Review/export companions retain manufacturing-description evidence. Full authoritative IPC-2581 import remains a documented boundary. |
| IEC 60352-5 | Press-in connection declarations and keepouts are reviewed; mechanical insertion qualification is deferred. |
| IEC 61000-4-5 | Surge protection placement/keepout evidence is reviewed; immunity testing is not replaced by geometry. |
| IEEE 828-2012 | Provenance, baselines, waivers, manifests, and generated-output freshness retain configuration/release evidence. |
| IPC-9797 | Automotive/high-reliability press-fit readiness consumes explicit profile and keepout policy; it does not certify the process. |
| IPC J-STD-001H | Soldering, polarity, paste, coating, and assembly-handoff checks expose missing evidence without asserting workmanship acceptance. |
| IPC-9252B | Bare-board electrical-test readiness is represented through IPC-D-356 coverage, net continuity, and package evidence. |
| IPC-7530 | Reflow-profile readiness remains artifact/process evidence. HyperDRC does not optimize or simulate an oven profile. |
| IPC-4552B | ENIG selection and compatibility are retained in surface-finish/handoff checks. Plating qualification is external. |
| IPC-6012D | Rigid-board capability and package-readiness thresholds remain declared fabricator policy, not baked-in geometry truth. |
| IPC-4556 | ENEPIG selection/bonding compatibility is reviewed from handoff evidence; deposition quality is not inferred. |
| Ucamco Gerber 2024.05 | Gerber/X2 metadata, polarity, units, roles, and parser diagnostics are retained; lossy polygon conversion stays explicit. |
| IPC-4553A | Immersion-silver selection and handoff compatibility are reviewed; process performance remains external. |
| IPC-7525B | Stencil area/width/aspect, paste coverage, windowpane, and tombstone heuristics are implemented with configurable policy. |

## Considered but not retained

- A second k-d tree alongside the regular grid would duplicate construction for
  checks whose radii and feature spans vary, while all candidates would still
  need exact replay. The measured hashed grid is simpler and faster on the
  representative sparse workload.
- Custom integer hash functions and fixed modulo tables from graphics-oriented
  spatial hashing were not retained. Exact tuple keys in the standard map avoid
  exposing table collisions and already deliver the measured gain.
- Incremental closest-feature caching from Lin-Canny was not introduced because
  independent static checks do not provide coherent successive configurations or
  stable simplex ownership.
- Frequency-dependent coupled-line, field, thermal, plating, fatigue, reflow,
  and electromagnetic models remain explicit delegated boundaries. Adding a
  partial formula without the referenced paper's required physical inputs would
  change a review prompt into an unsupported certification claim.

## Validation protocol

The retained path is covered by all spatial-index unit tests, all drill-spacing
tests, a hash-randomization ordering regression, the isolated release benchmark,
formatting, Clippy with warnings denied, all targets/features, and rustdoc.
`cargo test --all-targets --all-features` passed 1,082 library tests, both board
fixture benchmarks, the full debug parser/geometry smoke target, and the new
spatial benchmark. `cargo check --all-targets --all-features`, Clippy, generated
rustdoc, and `git diff --check` also passed. One executable Gerber-metadata
doctest could not be linked on this host: GNU `ld` repeatedly terminated with
`SIGBUS` while swap was saturated, including when run alone and with an alternate
linker request. Its parser assertions are independently covered by the passing
unit suite; this environmental linker failure is not counted as a passing test.

## Original rounded-pad overlap integral (2026-10-09)

The shared Hypercurve [admission record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hypercurve-retained-circular-area/manifest.json) now evaluates the exact
area of the full-board CPArti A5/A6 intersection (copper indices 1723/1724) and
exposes its native contour view. The independent rounded-pad area, overlap-box
cap and certified non-duplicate result remain unchanged. The optional duplicate
cap and unavailable-bound fallback remain in place.

Validation passes 1561 DRC tests and 14 selected consumers,
with strict lints/docs and all six version-only snapshot updates. Five trials
preserve all 231 previous cost rows. The unchanged fixture passes in
59.753837 seconds. Two fresh processes repeat each complete
full-board report exactly; only twelve version fields per board change from 18
to 19 against the prior record. Broader parser/native-source conversion, remaining
readiness consumers and general integration/resource closure remain open.

## Exact KiCad pad sources (2026-10-09)

The [pad-source record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/manifest.json)
retains exact decimal pad centers, dimensions, ratios and placement arithmetic,
including sizes outside binary64 range. It corrects pad rotation, mixed rounded
and chamfered corners, and the `F&B.Cu` outer-layer alias. Required copper/drill
markers propagate typed failure; net-class windows use native comparisons and
class aggregation delays marker projection until a finding needs it.

All eight scoped validation gates pass: 1,597 DRC tests, 14 selected consumer
tests, formatting, strict Clippy and docs, and the fixture benchmark build.
There are 36 new regressions. All six consumer snapshots change only their 160
implementation-version fields from 19 to 20. Prior core/demo source hashes are
unchanged; their historical tests were not rerun for this change.

Five fresh processes preserve all 231 prior cost rows: 26,801 bytes, SHA-256
`5fa8936b39564a0ac9158917935e4d43a8e36a5c1359cd1ff36474fc5ac27c4b`.
The unchanged full fixture completes in 56.205894 seconds against its 120-second
gate, with peak process RSS of 889,512 KiB. A separate loader probe runs 19
identical inputs, 50 iterations each, in five fresh processes per library;
18 correctness witnesses change and the ordinary control stays unchanged.
These are capability costs, with no matched speedup, allocation-count or
library-size improvement claim.

An independent rational/quadratic-surd source oracle matches positions, areas
and envelopes of all 2,837 native pad components on the complete CPArti and
HVP109A boards, plus placement of 17 custom components. A separate oracle
matches all 1,932 source drills and the complete clearance-pair sets at 0.25 mm
(zero CPArti pairs and 192 HVP109A pairs). This supports removal of ten old
drill-spacing findings. The existing circular proxy for slot drills remains.

Two fresh processes emit identical complete reports: 2,605 CPArti findings and
654 HVP109A findings. Every changed finding and metadata field is retained in
the [report audit](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/report-replay/finding-audit.json),
with [scoped review](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/report-replay/review.json).
All prior geometry-uncertainty findings remain identical. The record preserves
the failing baseline, eager-marker regression, version-only snapshot failures,
and the complete-board alias failure discovered after the earlier suite passed.
It also records the oracle's corrected handling of string-only net names;
production net parsing did not change.

Custom local primitives, other finite source adapters, 40 legacy copper-index
sites, remaining readiness heuristics and total resource bounds still require
work. These source formulas preserve authored values without imitating KiCad's
integer-unit rounding. The oracle does not establish whole-application KiCad
equivalence or certify every readiness rule. All properties and future-work
requirements remain in scope; the repository-wide pre-push matrix is still
required before any push.
