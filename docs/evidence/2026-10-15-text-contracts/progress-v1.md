# Typed text construction and native outline checkpoint

This checkpoint extends the work at `72cd216cc` under
[the recorded contract](contract-v1.md). It does not complete `properties.md`
or admit the 35 separately pending native files.

Hypercurve now owns complete, fallible Hershey decoding and stroke construction.
CSGRS propagates those errors through Cartesian text and atomic dimension
annotation. Missing or malformed glyphs, invalid/undecided scale and failed
strokes cannot become successful empty or partial text. Valid blank results
still compose. Error causes and operation-local certainty remain available.

TrueType construction now uses native lines and quadratic/cubic Bézier curves,
exact requested `Real` scale and the exact point unit `127/360` millimeters.
This explicitly replaces eight-step polygons and binary64 scale rounding.
The README documents the result-type, unit and representation migration,
parser f32 boundary, simple layout domain, curved extrusion sampling and STL
quantization. Class-based GPOS pairs now respect glyph coverage. The optional
unused `ttf-parser-utils` dependency is removed without updating other locked
versions. The TrueType PNG intentionally migrates; the Hershey PNG is unchanged.

Independent semantic tests establish square area 10000, quadratic area 10000/3,
and cubic callback area 24000, with interior/boundary/exterior witnesses. They
also cover exact translation, empty union/self subtraction, line-profile mesh
volume and topology, distinct scales 1 and 1+2^-100, pi and 2^±1100, malformed
and blank inputs, GPOS layout, Cartesian Hershey coordinates and atomic label
failure. Both original full TrueType workloads survive construction, transform,
composition, extrusion, topology validation and STL round trips under both
policies. Reimport compares exact represented f32 vertex sets and validates
mesh topology; it does not certify the original curved font surface.

The final production/test/asset sources pass all 16 commands in
[qualification-v3.json](qualification-v3.json), with raw logs and guarded hashes
in `qualification-v3/`. Results are:

| Check | Passed | Existing ignored |
| --- | ---: | ---: |
| Root all-feature library and integration tests | 377 | 1 |
| Root curve-only library and integration tests | 260 | 0 |
| TrueType-only public text tests | 5 | 0 |
| Hershey-only public text tests | 2 | 0 |
| Focused native Hershey tests | 6 | 0 |
| Root all-feature doctests | 1 | 0 |
| Full native library, preserved from qualification-v1 | 1760 | 1 |

These rows overlap and must not be added as unique coverage. Every `crates/`
source hash matches the full native-library run. Final checks also pass root
all-feature/all-target Clippy, native Hershey Clippy, no-default root Clippy,
formatting, warning-free root/native documentation, benchmark/example builds,
the actual text renderer and actual feature-benchmark text cases. The ignored
full README render comparison and long selected-fiber benchmark remain ignored;
text-specific rendering is executed and its two output images are checked
against the assets. No CI configuration changes or push are included.

[The summary](qualification-summary-v1.json) retains every failed probe, draft
build, qualification and measurement phase. [Diagnostics](diagnostic-findings-v1.md)
explain the corrected synthetic layout witness, const lint and measurement
serializer assumption. No policy, original font, full-size workload, resolution
or validation was weakened. Historical "components" counts in old probe output
mean boundary loops, including holes; final code and records use the proper term.

The [fixed cost contract](cost-contract-v1.md) is completed by `resources-v2`:
48 uninstrumented timing rows, 16 separate allocation rows and four additional
positive/negative Hershey preservation checks. All constructions complete with
local certified receipts. Serialization is outside construction timing; an
untimed replay supplies deterministic exact artifacts after allocation and
post-drop retention snapshots. `serialization-v1` checks the corrected native
endpoint serializer against both revisions before the complete matrix.

Each timing value below is the median of three fresh-process measurements in
nanoseconds. Warm processes perform one identical untimed construction. Orders
alternate within pairs. TrueType changes its output contract, so its ratios
cannot establish performance improvement.

| Fixture | Cache | Baseline ns | Candidate ns | Candidate/baseline |
| --- | --- | ---: | ---: | ---: |
| readme-truetype | cold | 28953054 | 10699876 | 0.3696 |
| readme-truetype | warm | 20582679 | 7444020 | 0.3617 |
| bench-truetype | cold | 123009284 | 46841548 | 0.3808 |
| bench-truetype | warm | 91118691 | 38340600 | 0.4208 |
| readme-hershey | cold | 294400 | 277101 | 0.9412 |
| readme-hershey | warm | 92304 | 102813 | 1.1139 |
| bench-hershey | cold | 356756 | 333377 | 0.9345 |
| bench-hershey | warm | 137481 | 140850 | 1.0245 |

The earlier failed `resources-v1` matrix remains intact: 32 TrueType attempts
completed construction but failed untimed serialization; their costs are not
credited. Its 32 successful Hershey rows (24 timing, eight allocation) remain
in the summary and raw logs. Their candidate/baseline timing ratios were
1.0112/0.9796 for README cold/warm and 0.9499/0.9812 for benchmark cold/warm.
The complete rerun was required by a serializer defect, not selected by cost.
No optimization claim is made from either small, noisy trial set.

Requested allocation totals (allocations plus reallocations), incremental peak
live requested bytes and post-drop retained deltas are below. Each cell is
baseline → candidate bytes, one instrumented observation per case/temperature.
These are allocator-requested bytes, not total heap or process memory.

| Fixture | Cache | Requested total | Incremental peak | Post-drop retained |
| --- | --- | ---: | ---: | ---: |
| readme-truetype | cold | 33176057 → 7832102 | 14917792 → 3200638 | 4912 → 130544 |
| readme-truetype | warm | 33172361 → 7871542 | 14911848 → 3171502 | 0 → 68376 |
| bench-truetype | cold | 138574217 → 36560145 | 62242509 → 12846860 | 7688 → 458040 |
| bench-truetype | warm | 138568673 → 36593657 | 62234893 → 12908772 | 896 → 110920 |
| readme-hershey | cold | 79248 → 77904 | 41816 → 41912 | 10264 → 10264 |
| readme-hershey | warm | 69544 → 68200 | 32816 → 32912 | 2280 → 2280 |
| bench-hershey | cold | 134872 → 133000 | 64856 → 65048 | 13240 → 13240 |
| bench-hershey | warm | 123080 → 121208 | 53896 → 54088 | 1760 → 1760 |

Hershey preserves the four original coordinate artifacts exactly. Its requested
allocation totals fall slightly, but each candidate sample adds one allocation
and 96/192 incremental peak bytes for README/benchmark. TrueType retention
increases as shown; no bounded-retention claim is made. The root rlib grows from
9,910,970 to 10,042,160 bytes and Hypercurve from 49,018,684 to 49,028,054 bytes.
The timing driver grows from 36,293,096 to 36,342,344 bytes; the allocator driver
from 36,296,976 to 36,341,616. Source deltas, compiler, hardware, dependency hashes,
commands, timing boundaries, policies and all raw samples are retained in the
phase protocol, `cost-host-v2.json` and the summary. Unfavorable results are kept.

Owned text buffers use checked capacities/reservations, but internal total-work,
heap-byte and cancellation bounds remain open. Validation uses external
1800-second/8-GiB process containment; measurement children also have 120-second
walls. Parser f32 rounding, full font-file validation, CFF2 blank certification,
shaping, end-to-end provenance, complete external package/release validation,
full CI, formal proof/fuzz/coverage and other properties obligations remain open.
Text construction is not exposed by current C/WASM bindings; no new binding
surface is claimed. The tests validate local workspace consumers, not a clean
published-package installation.

The closure checks every inherited artifact expectation, all new phase logs and
builds, immutable overlay lowers, source guards and the 35 pending native-file
hashes. The inherited 27,093 missing artifact expectations remain explicit;
there is no recovery claim. Only `current_work` in the protected properties
ledger may change. The full-plan flag remains false.
