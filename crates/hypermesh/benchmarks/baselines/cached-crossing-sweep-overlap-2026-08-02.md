# Cached crossing-sweep overlap — 2026-08-02

This is Phase 7 checkpoint 34 of the workspace Hypermesh path-completeness
plan. The retained Hypermesh implementation is
`29e316d4cca18c8c636a5e4300355caa1d75f2ee`, based on checkpoint 33 evidence
revision `cb46ef5b704d4b50f185232b3916361570451bc0`. Hyperreal,
Hyperlattice, Hyperlimit, and Hypertri remain at
`3d50951775764f6ca50f5805b149c54cc423432c`,
`d11ca2f0e825d8e26048cfda5d1101df21dcfef0`,
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`, and
`c47601266e0b9b17d0c5a0764fa22b18168ada73`.

## Outcome

The certified-enclosure cached crossing sweep no longer tests overlap on its
sweep axis twice. Its complete 48-byte edge-bound entry is stored in traversal
order: the selected sweep interval first, followed by the two projection-axis
intervals. Sorting and the existing break establish overlap on the first
interval. The hot nested scan therefore reads fixed offsets and checks only the
two intervals that can still prove separation.

On the canonical 852-triangle generated union, policy-paired direct A/B reduces
task clock 1.253%, cycles 1.179%, instructions 1.404%, and branches 0.603%.
The 4,524-triangle retained arrangement improves 0.569% / 0.568% / 0.149% /
0.396%. The below-threshold dense-box control is clock/cycle neutral,
instructions move +0.034%, and branches fall 0.024%. The sampled crossing owner
falls from 4.42% to 4.21% self. All large-fixture allocation counts and peaks
are unchanged. The canonical linked-size changes are at most 288 native text
bytes and 75 optimized WASM bytes; the equal-layout repeated executable
shrinks 92 text bytes and 88 file bytes.

## Exactness proof and complete paths

`exact_output_vertex_enclosures` admits the approximate sweep only when every
exact-rational coordinate has finite outward-rounded binary64 bounds. For each
cached pair reached with `right_index > left_index`:

1. sorting by the selected finite enclosure minimum proves
   `right_min >= left_min`;
2. failure of the unchanged break proves `right_min <= left_max`; and
3. valid interval construction proves `right_max >= right_min`.

The sweep-axis intervals therefore overlap. Endpoint equality remains overlap.
The cached scan still tests both directions of interval overlap on each of the
other two axes. A rejection is possible only when disjoint outward enclosures
prove exact separation. Every survivor still executes the unchanged exact
edge-bound comparison, projected orientation, 3-D coplanarity, intersection
construction, interning, repair, and closure paths.

All path families remain explicit:

- every one of the three adaptive sweep axes is tested by the focused
  regression, including its corresponding two projection axes;
- cached scans at or above 256 edges use the reordered complete side vector;
- smaller scans and cache-allocation failure retain the complete direct
  enclosure path;
- inputs without certified enclosures retain the symbolic/exact bounds sweep;
- exact separation hidden by overlapping binary64 bounds still reaches the
  exact predicate;
- equality at enclosure endpoints, shared endpoints, collapsed exact events,
  independent batches, T-junctions, coplanar/projected crossings, and
  more-than-historical-pass-limit event sets retain their tests and paths; and
- no candidate limit, pass limit, epsilon, index narrowing, public API,
  dependency, compatibility shim, allocation, or retained byte was added.

`STRICT` still forbids approximate decisions. `APPROXIMATE_512` can consume
approximation only in Hyperlimit's terminal 512-bit equality/sign
interpretation. The reordered broad phase does not inspect policy, set
certainty, or certify a predicate. Every measured output under both policies
is `Certified` and policy outputs remain identical.

## Serialized CPU A/B

Parent and candidate executables use equal-layout five-crate source trees,
identical `-C target-cpu=native -C codegen-units=1` flags, CPU 9, one fixture
construction per process, complete immediate unions, and two measurements per
revision in bracketed order. Negative values are improvements.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 852 / `STRICT` | 1,001 | -1.448% | -1.429% | -1.406% | -0.604% | -0.403% | -1.938% |
| Generated 852 / `APPROXIMATE_512` | 1,001 | -1.058% | -0.928% | -1.403% | -0.601% | +0.120% | -1.172% |
| Retained 4,524 / `STRICT` | 201 | -1.338% | -1.239% | -0.147% | -0.393% | -0.461% | -1.653% |
| Retained 4,524 / `APPROXIMATE_512` | 201 | +0.200% | +0.104% | -0.152% | -0.399% | -0.582% | -1.644% |
| Dense boxes 6,144 / `STRICT` | 10,001 | -0.675% | -0.613% | +0.037% | -0.021% | -0.304% | -1.852% |
| Dense boxes 6,144 / `APPROXIMATE_512` | 10,001 | +0.520% | +0.269% | +0.031% | -0.028% | +0.870% | -1.294% |

Policy-paired means are:

- generated: -1.253% task, -1.179% cycles, -1.404% instructions, and
  -0.603% branches;
- retained: -0.569% task, -0.568% cycles, -0.149% instructions, and
  -0.396% branches; and
- dense boxes: -0.077% task, -0.172% cycles, +0.034% instructions, and
  -0.024% branches.

The opposite clock movements between box policies expose layout/thermal noise;
that fixture never enters the cached tier and its deterministic work is nearly
flat. A supplemental 13,452-triangle generated bracket reduces instructions
0.897% and branches 0.392% under both policies. Its clocks varied by several
seconds while the work counts remained fixed, so they are deliberately not
used as a retention gate.

## Dispatch and profile evidence

The generated dispatch trace exactly reproduces checkpoint 33:

| Event | Count |
| --- | ---: |
| Dispatch | 97,347 |
| Predicate | 676 |
| Linear algebra | 1,411 |
| Cache / filter hits / filter misses | 6,345 / 6,107 / 216 |
| Active cycles proposed / certified | 45 / 45 |
| Rational temporaries | 12,794 |
| Unknown / fallback-or-abort | 0 / 0 |

The final frame-pointer profile covers 501 strict generated unions on CPU 9,
13,172 samples, zero lost samples, and approximately 13.172 billion cycle
events. `split_edge_crossing_events` is 4.21% self, down from checkpoint 33's
4.42%. Current leading owners are word GCD (4.18%), crossing splitting (4.21%),
the certified rational line filter (3.92%), allocator internals (3.40%), fixed
512-bit GCD (2.86%), mixed-width GCD (2.29%), and lossy rational export
(2.16%). Sampling percentages are directional; serialized counters are the
retention gate.

## Large-fixture heap

Heaptrack covers fixture construction plus one production, no-hook immediate
union under both policies.

| Fixture | Input triangles | Allocations | Reconstructed temporary | Heaptrack peak | Strict / approximate RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,743 | 10,358 | 7.50 MiB | 18.58 / 18.48 MiB |
| Retained arrangement | 4,524 | 453,990 | 28,734 | 11.67 MiB | 21.99 / 21.73 MiB |
| Dense boxes | 6,144 | 27,189 | 62 | 2.34 MiB | 11.30 / 11.24 MiB |

Counts and rounded peaks exactly reproduce checkpoint 33 under both policies.
Direct byte-level Massif A/B uses equal-length, equal-layout parent/candidate
executables:

| Fixture | Parent maximum | Candidate maximum | Parent useful | Candidate useful |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 8,245,672 B | 8,245,672 B | 7,420,566 B | 7,420,566 B |
| Retained arrangement | 12,700,760 B | 12,700,760 B | 11,591,447 B | 11,591,447 B |
| Dense boxes | 2,597,144 B | 2,597,144 B | 2,262,518 B | 2,262,518 B |

A standalone production retained Massif run was 1,320 bytes above checkpoint
33 while generated and boxes were byte-identical. The equal-layout retained
parent and candidate then matched exactly, proving that the small standalone
shift is process/environment sampling rather than an implementation
allocation. The cache entry remains exactly 48 bytes and its capacity and
lifetime are unchanged.

## Linked code and call graph

| Consumer | Checkpoint 33 | Checkpoint 34 | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,204 B | 4,065,468 B | +264 B |
| General release native aggregate | 4,305,535 B | 4,305,527 B | -8 B |
| Immediate release native text | 4,098,852 B | 4,099,116 B | +264 B |
| Immediate release native aggregate | 4,342,399 B | 4,342,391 B | -8 B |
| General release WASM `wasm-opt -Oz` | 2,740,709 B | 2,740,784 B | +75 B |
| Immediate release WASM `wasm-opt -Oz` | 2,755,752 B | 2,755,827 B | +75 B |
| General size native text | 1,871,042 B | 1,871,322 B | +280 B |
| General size native aggregate | 2,114,116 B | 2,114,124 B | +8 B |
| Immediate size native text | 1,883,510 B | 1,883,798 B | +288 B |
| Immediate size native aggregate | 2,126,408 B | 2,126,408 B | 0 B |
| General size WASM `wasm-opt -Oz` | 1,165,389 B | 1,165,394 B | +5 B |
| Immediate size WASM `wasm-opt -Oz` | 1,175,768 B | 1,175,773 B | +5 B |
| Equal-layout repeated probe text | 4,636,065 B | 4,635,973 B | -92 B |
| Equal-layout repeated probe aggregate | 4,887,164 B | 4,887,136 B | -28 B |
| Equal-layout repeated probe file | 5,586,352 B | 5,586,264 B | -88 B |

The implementation is 52 insertions and 14 deletions including focused tests.
It adds no public type, dependency, carrier storage, allocation, policy path,
or compatibility layer.

The Hypermesh-only graph is 8,071 nodes / 19,871 edges. The complete five-crate
graph is 19,747 / 39,505 with JSON SHA-256
`519faa2e0bf1d03f4bea75531c56a2608c211543f9a8d3a4178b50421eb9f88c`.
Relative to checkpoint 33, both graphs move +3 nodes / +7 edges for the private
ordered-bounds construction, overlap helper, closure, and focused test calls.
No equality, terminal-policy, allocation-fallback, topology, or public-call
edge was added. The Hypermesh-only JSON SHA-256 is
`eaad3f50eb2ce76783fe1c6660b6270f89f923c3d5a99e53756a93a00ee779a4`.

## Competitive and historical controls

Fresh CPU-9 Criterion centers cover all three competitive operations:

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 8.5042 ms | 995.54 us | 682.47 us | 8.54x / 12.46x slower |
| Projective / intersection | 5.9026 ms | 749.17 us | 669.78 us | 7.88x / 8.81x slower |
| Projective / difference | 4.2704 ms | 772.89 us | 679.86 us | 5.53x / 6.28x slower |
| Dense boxes / union | 761.97 us | 7.5605 ms | 4.9556 ms | 9.92x / 6.50x faster |
| Dense boxes / intersection | 552.51 us | 3.9526 ms | 3.4721 ms | 7.15x / 6.28x faster |
| Dense boxes / difference | 705.53 us | 6.1517 ms | 4.0537 ms | 8.72x / 5.75x faster |

The projective union row contained two severe high outliers and its Hypermesh
interval widened to 6.3646–9.9927 ms. Boolmesh simultaneously regressed and
Manifold improved relative to their preceding sessions. Same-session ratios
are therefore reported for competitive orientation, while serialized hardware
counters remain the candidate gate. Competitors remain throughput comparators,
not exactness or policy oracles.

The final direct retained centers are 34.914 ms under `STRICT`, 35.657 ms under
`APPROXIMATE_512`, and 35.285 ms policy-paired. Against the directional
historical 944.8 ms row, this is 96.265% lower or 26.776x faster.

## Rejected implementations

- Keeping natural `[x, y, z]` cache order and passing runtime-selected axes to
  a two-axis helper improved the generated row but increased retained
  instructions about 0.16%. It was fully removed.
- Reusing the general natural-axis bounds constructor before permuting the
  entry reduced retained instructions another 0.07%, but a direct equal-name
  A/B against the retained constructor increased retained task clock 0.678%
  and cycles 0.610%. Performance has priority, so it was fully removed.
- Iterator, explicit-match, and precomputed-axis cached forms had already been
  rejected at the earlier direct-sweep checkpoint for 0.18–0.36% retained
  instruction growth. The retained cache-native ordering avoids their dynamic
  indexing.

No diagnostic counter, rejected representation, temporary repetition hook, or
measurement-only source remains in a production tree.

## Validation

The retained implementation passes:

- Hypermesh default/no-default/all-feature library suites with
  1,063 / 1,063 / 1,064 tests and every default integration suite;
- all 1,063 default library tests under AddressSanitizer with leak detection
  disabled;
- warning-denied Clippy and rustdoc under all and minimal features, formatting,
  every fuzz-bin check, and all-feature benchmark compilation;
- release every-operation exactness and policy identity, polygon/immediate API
  agreement, 3,360/13,440-triangle stress, and full 11,894-triangle input
  validation;
- generated dispatch trace, canonical and supplemental CPU rows, twelve
  Heaptrack/analysis passes, nine Massif runs, native/WASM size consumers,
  competitive and historical controls, frame-pointer profile, and both call
  graphs; and
- exact hidden-separation, collapsed-event, symbolic/no-enclosure,
  allocation/direct fallback, cached threshold, all-axis, event batching,
  T-junction, closure, and policy regressions.

The first opt-in release-test attempt used an isolated target directory without
the already validated external fixture and failed before test execution when
the sandbox blocked a download. Reusing the canonical cached fixture made the
unchanged test pass. This was an environment-path failure, not a test failure.

The approximately 56-minute rotated 11,894-by-11,894 Boolean was not rerun.
The change removes only a mathematically implied enclosure comparison and does
not alter output topology or any exact/fallback path; the full input validator,
13,440-triangle output stress, exact hidden-separation tests, identical dispatch
sequence, and sanitizer suite exercise the affected schedule.

Hyperlimit's pre-existing untracked `hyperlimit` executable is untouched.
