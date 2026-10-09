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
[validation manifest](../../docs/evidence/2026-10-08-hyperdrc-native-geometry/measurement-manifest.json)
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
[raw runs and replay inputs](../../docs/evidence/2026-10-08-hyperdrc-native-geometry/measurement-probe/manifest.json)
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
[source and validation record](../../docs/evidence/2026-10-08-hyperdrc-native-geometry/circle-manifest.json)
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
[raw probe and replay inputs](../../docs/evidence/2026-10-08-hyperdrc-native-geometry/circle-cost/manifest.json)
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
The [manifest](../../docs/evidence/2026-10-08-hyperdrc-native-distance/manifest.json)
retains the source hashes, failures, commands, and scope limitations.

Five fresh processes pinned to CPU 0 measured the unchanged 2,000-square,
64-probe field after all builds/tests ended. Median filled-region construction
and boundary preparation took 628.555 ms; 64 probe constructions and queries took
22.352 ms, within the existing two-second gate. Every trial produced 64 exact
unit distances, 257 serialized bytes and the same SHA-256. Whole-process peak RSS
was 65,088–65,608 KiB. The test executable was 51,646,568 bytes.

One full fixture smoke run, also pinned to CPU 0, passed in 65.446 seconds with
1,483,976 KiB peak RSS; its executable was 42,051,792 bytes. The
[raw cost record](../../docs/evidence/2026-10-08-hyperdrc-native-distance/cost/manifest.json)
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
the [evidence manifest](../../docs/evidence/2026-10-08-hyperdrc-projection/manifest.json).
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
The [snapshot audit](../../docs/evidence/2026-10-08-hyperdrc-projection/snapshot-audit/summary.json)
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
The [raw cost record](../../docs/evidence/2026-10-08-hyperdrc-projection/cost/manifest.json)
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

The [evidence manifest](../../docs/evidence/2026-10-08-hyperdrc-operations/manifest.json)
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
[version audit](../../docs/evidence/2026-10-08-hyperdrc-operations/snapshot-versions.json)
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
executable. The [raw cost record](../../docs/evidence/2026-10-08-hyperdrc-operations/cost/manifest.json)
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

The [evidence manifest](../../docs/evidence/2026-10-08-hyperdrc-handoff/manifest.json)
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
[version audit](../../docs/evidence/2026-10-08-hyperdrc-handoff/snapshot-versions.json)
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
[raw cost record](../../docs/evidence/2026-10-08-hyperdrc-handoff/cost/manifest.json)
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

The [evidence manifest](../../docs/evidence/2026-10-08-hyperdrc-area/manifest.json)
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
[snapshot audit](../../docs/evidence/2026-10-08-hyperdrc-area/snapshot-versions.json).
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
[raw cost record](../../docs/evidence/2026-10-08-hyperdrc-area/cost/manifest.json)
retains commands, source hashes, artifacts and all trials.

The unchanged full two-board and Gerber fixture workload passed in 64.778
seconds against its 120-second gate, with 1,388,444 KiB peak RSS. CPArti's
layer-sanity count increases from four to five; the other eight logged check
counts are unchanged. The [full-board audit](../../docs/evidence/2026-10-08-hyperdrc-area/fixture-audit/manifest.json)
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

The [evidence record](../../docs/evidence/2026-10-08-hyperdrc-bounds/manifest.json)
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
[cost manifest](../../docs/evidence/2026-10-08-hyperdrc-bounds/cost/manifest.json)
retains all trials and source/artifact hashes.

The unchanged full two-board and Gerber fixture passed in 65.072 seconds against
its 120-second gate, with 1,339,172 KiB peak RSS. All nine logged check counts are
unchanged. The [rendered-report audit](../../docs/evidence/2026-10-08-hyperdrc-bounds/cost/fixture-report-audit.json)
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

The [evidence record](../../docs/evidence/2026-10-08-hyperdrc-stencil-points/manifest.json)
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
allocation count or library-size claim. The [raw cost record](../../docs/evidence/2026-10-08-hyperdrc-stencil-points/cost/manifest.json)
retains commands, source/artifact hashes and all trials.

The unchanged full two-board and Gerber fixture passed in 68.097 seconds against
its 120-second gate, with 1,334,628 KiB peak RSS. All nine logged check counts are
unchanged. The [report audit](../../docs/evidence/2026-10-08-hyperdrc-stencil-points/cost/fixture-report-audit.json)
found a duplicate-layer row with reversed layer order and different optional
markers. The [full-board witness](../../docs/evidence/2026-10-08-hyperdrc-stencil-points/fixture-order-audit/manifest.json)
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

The [evidence record](../../docs/evidence/2026-10-08-hyperdrc-affine/manifest.json)
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
equality does not prove numeric inequality. The [rejected probe draft](../../docs/evidence/2026-10-08-hyperdrc-affine/draft/cost-structural-equality/manifest.json)
records that corrected harness mistake; it is excluded from final trials.

All 48 witness rows preserve identical 4,788-byte output across the five trials,
including the previous 40 unchanged rows. Peak process RSS was 58,736–59,404 KiB,
including the existing million-sample report fixture. The linked probe was
24,309,000 bytes. These are capability costs, without a matched speedup,
allocation count or library-size comparison. The [raw cost record](../../docs/evidence/2026-10-08-hyperdrc-affine/cost/manifest.json)
retains commands, source/artifact hashes and every trial.

The unchanged full two-board and Gerber fixture passed in 64.971 seconds against
its 120-second gate, with 1,390,436 KiB peak RSS. All nine logged check counts and
every rendered finding match after explicit temporary-path/ID and finding-row
normalization. The [report audit](../../docs/evidence/2026-10-08-hyperdrc-affine/cost/fixture-report-audit.json)
records row-order differences and no marker changes. The [known-variant audit](../../docs/evidence/2026-10-08-hyperdrc-affine/cost/known-order-audit.json)
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

The [evidence record](../../docs/evidence/2026-10-08-hyperdrc-native-drills/manifest.json)
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
[raw cost record](../../docs/evidence/2026-10-08-hyperdrc-native-drills/cost/manifest.json)
retains every trial. These are capability costs, without a matched speedup,
allocation count or library-size comparison.

The unchanged full two-board/Gerber fixture passed in 65.027 seconds against its
120-second gate, with 1,338,864 KiB peak RSS. All nine logged check counts and
every rendered finding match after temporary-path/ID and finding-row
normalization. The [report audit](../../docs/evidence/2026-10-08-hyperdrc-native-drills/cost/fixture-report-audit.json)
records row-order differences and no marker changes. The
[known-variant audit](../../docs/evidence/2026-10-08-hyperdrc-native-drills/cost/known-order-audit.json)
reuses the prior full-board duplicate-layer witness; original report order and
path-independent IDs remain unproved.

Legacy point/copper indices, other drill decisions, native pad/track/slot/custom
source geometry, report-chord widths and density interpolation remain open.
Drill inputs retain their documented circular proxies. Shared guards reject
known exhaustion without establishing total resource bounds. All 39 properties,
all 14 targets and crate-specific future work remain in scope.

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
