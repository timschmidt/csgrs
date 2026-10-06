# Unified certified point query carrier — 2026-08-02

This is Phase 7 checkpoint 35 of the workspace Hypermesh path-completeness
plan. The retained implementations are Hyperreal
`fde2f8d3ad12e314a706ae39ef3ba1805cdaea84` and Hypermesh
`301ca262a7d5122f975722d40e3ce6b3a1703b54`. The direct source parents are
Hyperreal `3d50951775764f6ca50f5805b149c54cc423432c` and Hypermesh checkpoint 34
implementation `29e316d4cca18c8c636a5e4300355caa1d75f2ee`; checkpoint 34's evidence
revision is `abfb48d657256b903c98b2090cdd9dc8f925f6dc`. Hyperlattice, Hyperlimit,
and Hypertri remain at `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`,
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`, and
`c47601266e0b9b17d0c5a0764fa22b18168ada73`.

## Outcome

Hypermesh's retained exact-output vertex data is now one 48-byte carrier
instead of two different 48-byte views of the same coordinates. Hyperreal's
`RationalPoint3Query` stores each coordinate as a verified finite midpoint and
absolute radius. It can therefore supply both conservative outward bounds and
the midpoint/error form consumed by certified projected-line filters.

The exact-output pass constructs this carrier once per vertex. T-junction and
crossing broad phases borrow its bounds; crossing filters borrow the same
queries. The previous hot loop built two queries for every left edge and two
more for every surviving pair. The candidate builds none there. It adds no
allocation, cache, dependency, retained byte, public type, or compatibility
layer.

Across both policies, the generated 852-triangle control drops 0.755%
instructions and 1.048% branches; the retained 4,524-triangle arrangement
drops 0.786% and 1.215%. The below-threshold 6,144-triangle box control drops
0.094% and 0.485%. Reverse confirmation pairs reduce clock and cycles on the
two initially noisy rows while reproducing the deterministic counters. The
frame-pointer profile moves crossing self-time from 4.21% to 3.76% and the
certified line-filter owner from 3.92% to 3.43%.

Useful heap is equal on generated and box fixtures and 120 bytes lower on the
retained fixture. Its total retained Massif peak moves +16 bytes solely in
allocator metadata. Speed-profile native text shrinks 760–792 bytes and
optimized WASM shrinks 609–615 bytes. Size-profile text/WASM grows at most 606
bytes, while native aggregate sections are unchanged or +8 bytes. Performance
has priority, and the deterministic hot-path win is substantially larger than
that sub-kilobyte size-profile tradeoff.

## Exactness proof and complete paths

For each exact rational coordinate, `Rational::to_f64_enclosure` first supplies
finite outward binary64 bounds. The carrier constructor rejects reversed or
non-finite bounds, a non-finite span or midpoint, a midpoint outside the source
interval, a non-finite outward-rounded radius, non-finite reconstructed bounds,
or any reconstructed bound that fails to enclose the supplied interval.
Degenerate intervals retain radius zero. Subnormal-width and signed-zero cases
are covered explicitly.

Every admitted carrier therefore satisfies, for each coordinate,

`reconstructed_lower <= exact_value <= reconstructed_upper`.

The midpoint/radius conversion can widen an input enclosure. That is safe in
all consumers:

- broad-phase interval rejection occurs only when the wider intervals prove
  exact separation, so widening can retain work but cannot discard a true
  candidate;
- approximate edge ordering is used only when disjoint certified intervals
  prove the exact order;
- the approximate projection normal only selects which complete exact
  projection is attempted first and never proves or rejects a crossing;
- `RationalLine2Filter` propagates every retained coordinate radius through
  its certified error bound and returns `None` whenever the sign is uncertain;
  and
- every uncertain or surviving crossing still executes the unchanged exact
  rational orientation, 3-D coplanarity, construction, interning, repair, and
  closure paths.

Construction failure remains complete. If any exact output coordinate cannot
form the carrier, `exact_output_vertex_enclosures` returns `None` and the whole
operation uses the existing symbolic/exact bounds search. The allocation-fail
direct crossing path is unchanged. Inputs without exact-rational enclosures,
hidden exact separation inside overlapping binary64 bounds, endpoint equality,
shared endpoints, collapsed events, independent batches, every sweep axis,
T-junctions, and event sets beyond the historical pass limit retain their
existing regressions and exact fallbacks.

The representation is policy-independent certified filtering, not approximate
equality. `STRICT` never accepts an approximate decision. `APPROXIMATE_512`
may differ only when Hyperlimit owns the terminal 512-bit equality/sign
interpretation. Both policies produced identical `Certified` topology on every
measured fixture.

## Serialized CPU A/B

Parent and candidate probes use equal flags (`-C target-cpu=native -C
codegen-units=1`), CPU 9, one fixture construction per process, complete
immediate unions, and bracket order parent/candidate/candidate/parent. Negative
values are improvements.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 852 / `STRICT` | 1,001 | +0.543% | +0.473% | -0.754% | -1.047% | -0.904% | +0.653% |
| Generated 852 / `APPROXIMATE_512` | 1,001 | -1.060% | -0.928% | -0.755% | -1.048% | -1.087% | -0.662% |
| Retained 4,524 / `STRICT` | 201 | -0.059% | -0.113% | -0.786% | -1.215% | -0.776% | -0.867% |
| Retained 4,524 / `APPROXIMATE_512` | 201 | +2.537% | +2.189% | -0.786% | -1.214% | -0.553% | +1.137% |
| Dense boxes 6,144 / `STRICT` | 10,001 | +0.276% | +0.358% | -0.091% | -0.482% | +1.604% | +2.899% |
| Dense boxes 6,144 / `APPROXIMATE_512` | 10,001 | -1.342% | -1.305% | -0.096% | -0.488% | -1.410% | -0.764% |

Policy-paired deterministic means are -0.755% instructions / -1.048%
branches on generated, -0.786% / -1.215% on retained, and -0.094% / -0.485%
on boxes. Policy-paired task/cycle means are -0.259% / -0.228% generated and
-0.533% / -0.474% boxes.

One retained approximate candidate measurement took 7.249 seconds while its
instruction and branch counts were unchanged to measurement noise. That makes
the retained bracket clock mean unusable. A reverse candidate/parent
confirmation reduced retained approximate task clock 1.146% and cycles 1.073%
while again reducing instructions 0.789% and branches 1.219%. A reverse
generated strict confirmation reduced clock 1.093%, cycles 0.794%, instructions
0.755%, and branches 1.047%. The stable hardware work counts and profile, not a
selected clock sample, are the retention gate.

## Dispatch and profile evidence

The generated all-family dispatch trace exactly reproduces checkpoint 34:

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
12,962 samples, zero lost samples, and approximately 12.962 billion cycle
events. Leading self owners are word GCD 4.00%, crossing splitting 3.76%,
`_int_malloc` 3.65%, the certified rational line filter 3.43%, fixed 512-bit
GCD 3.02%, lossy rational export 2.31%, and mixed-width GCD 1.87%. Sampling is
directional; serialized counters remain the performance gate.

## Large-fixture heap

Production, no-hook Heaptrack recordings include fixture construction and one
complete immediate union under both policies.

| Fixture | Input triangles | Allocations | Reconstructed temporary | Heaptrack peak | Strict / approximate RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,743 | 10,358 | 7.50 MiB | 18.35 / 18.36 MiB |
| Retained arrangement | 4,524 | 453,990 | 28,734 | 11.67 MiB | 21.51 / 21.70 MiB |
| Dense boxes | 6,144 | 27,189 | 62 | 2.34 MiB | 11.52 / 11.42 MiB |

Counts and rounded peaks reproduce checkpoint 34. Direct byte-level Massif
A/B uses the equal-flag repeated parent and candidate probes:

| Fixture | Parent total | Candidate total | Parent useful | Candidate useful |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 8,245,672 B | 8,245,672 B | 7,420,566 B | 7,420,566 B |
| Retained arrangement | 12,698,560 B | 12,698,576 B | 11,589,647 B | 11,589,527 B |
| Dense boxes | 2,597,160 B | 2,597,160 B | 2,262,518 B | 2,262,518 B |

The retained carrier is still exactly 48 bytes. The 16-byte retained total
movement is allocator metadata; useful heap falls 120 bytes. No capacity,
lifetime, allocation count, or stored byte was added.

## Linked code and call graph

| Consumer | Checkpoint 34 | Checkpoint 35 | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,468 B | 4,064,708 B | -760 B |
| General release native aggregate | 4,305,527 B | 4,305,535 B | +8 B |
| Immediate release native text | 4,099,116 B | 4,098,324 B | -792 B |
| Immediate release native aggregate | 4,342,391 B | 4,338,303 B | -4,088 B |
| General release WASM `wasm-opt -Oz` | 2,740,784 B | 2,740,169 B | -615 B |
| Immediate release WASM `wasm-opt -Oz` | 2,755,827 B | 2,755,218 B | -609 B |
| General size native text | 1,871,322 B | 1,871,754 B | +432 B |
| General size native aggregate | 2,114,124 B | 2,114,124 B | unchanged |
| Immediate size native text | 1,883,798 B | 1,884,222 B | +424 B |
| Immediate size native aggregate | 2,126,408 B | 2,126,416 B | +8 B |
| General size WASM `wasm-opt -Oz` | 1,165,394 B | 1,166,000 B | +606 B |
| Immediate size WASM `wasm-opt -Oz` | 1,175,773 B | 1,176,378 B | +605 B |
| Equal-flag repeated probe text | 4,635,973 B | 4,634,709 B | -1,264 B |
| Equal-flag repeated probe aggregate | 4,887,136 B | 4,887,168 B | +32 B |
| Equal-flag repeated probe file | 5,586,264 B | 5,585,416 B | -848 B |

Hyperreal changes 64 insertions / 18 deletions and Hypermesh 65 / 46,
including focused tests. The sole cross-crate API addition is one doc-hidden
bound accessor on the existing query type.

The Hypermesh-only graph is 8,051 nodes / 19,877 edges, moving -20 / +6. Its
JSON SHA-256 is
`efbebd2b6288e1bedefadc6082cac08d94d526383e28849e5d4c9908b570f617`.
The five-crate graph is 19,765 / 39,528, moving +18 / +23, with SHA-256
`1d2df68a290da9ef3ba611de02840c4692130fb0195486c5452a1e556d14b33e`.
The additional Hyperreal constructor checks and focused tests explain the
five-crate growth; removing duplicate Hypermesh construction expressions
shrinks its node count. No policy terminal, equality, allocation-fallback, or
topology edge changed.

## Competitive and historical controls

Fresh CPU-9 Criterion centers cover all three competitive operations:

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 6.2185 ms | 762.01 us | 677.51 us | 8.16x / 9.18x slower |
| Projective / intersection | 4.5295 ms | 748.12 us | 671.39 us | 6.05x / 6.75x slower |
| Projective / difference | 4.1144 ms | 762.14 us | 682.05 us | 5.40x / 6.03x slower |
| Dense boxes / union | 751.00 us | 7.0070 ms | 5.0355 ms | 9.33x / 6.71x faster |
| Dense boxes / intersection | 537.97 us | 3.9369 ms | 4.1187 ms | 7.32x / 7.66x faster |
| Dense boxes / difference | 683.29 us | 6.6236 ms | 4.7097 ms | 9.69x / 6.89x faster |

Competitors remain throughput comparators, not exactness or policy oracles.
The same-session results retain Hypermesh's exactness-cost disadvantage on the
projective workload and strong advantage on dense boxes.

The direct retained strict bracket averages 34.446 ms per union. Against the
directional historical 944.8 ms row, that is 96.354% lower or 27.428x faster.
The retained 11.67 MiB peak, 453,990 allocations, and 21.51 MiB strict RSS are
82.77%, 90.96%, and 73.93% below the historical 67.74 MiB, 5,020,891, and
82.5 MiB. Fixture and implementation evolution make these historical values a
trend rather than a direct A/B.

## Rejected implementations

- Storing lower bound plus width made bound reconstruction branchier and
  regressed the dense-box control. It was fully removed.
- A per-output-vertex query cache had previously reduced retained instructions
  but added allocation, heap, text, and a 6.67% projective Criterion regression.
  The retained implementation instead reuses the existing 48-byte storage.
- Retaining both the enclosure array and the query would have made borrowing
  trivial but doubled the hot per-vertex storage to 96 bytes. It was not
  implemented.

No diagnostic counter, rejected representation, temporary repetition hook,
or measurement-only source remains in a production tree.

## Validation

The retained implementation passes:

- Hyperreal default/minimal/all-feature library suites with 562 / 562 / 639
  tests, the full default integration/doctest suite, warning-denied Clippy and
  rustdoc, formatting, and a 20,000-case randomized exact-sign oracle;
- Hypermesh default/minimal/all-feature library suites with 1,063 / 1,063 /
  1,064 tests and every default integration suite;
- all 1,063 Hypermesh default library tests under AddressSanitizer with leak
  detection disabled;
- warning-denied Hypermesh Clippy and rustdoc under all and minimal features,
  formatting, every fuzz-bin check, and all-feature benchmark compilation;
- release every-operation exactness and strict/approximate policy identity,
  polygon/immediate API agreement, 3,360/13,440-triangle stress, and full
  11,894-triangle input validation;
- dispatch, serialized CPU, profile, six production Heaptrack analyses, six
  direct Massif runs, native/WASM size, both call graphs, competitive, and
  historical controls; and
- invalid/reversed/non-finite/extreme, least-subnormal, signed-zero,
  hidden-separation, collapsed-event, symbolic/no-enclosure, allocation/direct
  fallback, cached-threshold, all-axis, batching, T-junction, closure, and
  policy regressions.

The approximately 56-minute rotated 11,894-by-11,894 Boolean was not rerun.
The change replaces equal-sized certified storage and removes reconstruction;
it does not alter exact predicates or topology. The full input validator,
13,440-triangle output stress, exact fallback regressions, unchanged dispatch
trace, randomized oracle, and sanitizer suite cover the affected paths.

Hyperlimit's pre-existing untracked `hyperlimit` executable is untouched.
