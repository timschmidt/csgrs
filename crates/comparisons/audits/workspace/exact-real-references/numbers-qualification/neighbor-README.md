# numbers follow-through: inverse-series domain checks

Scalar repair qualified on 2026-09-06, including terminal Memcheck with zero
errors; outstanding Hypercurve status is tracked in the root progress ledger.
The numbers target remains OPEN for
the independently frozen ln_1p failures. This is not ecosystem completion.

## Defect and retained design

`asin` and `atanh` used a cheap, potentially inexact magnitude estimate to
schedule a tiny-argument Taylor series. A chain of additions can outgrow that
estimate. The public input `sqrt(5)/64 + 740*sqrt(7)/2048` is strictly between
zero and one (about 0.9909229225916945), but both functions violated their
32-bit error contract. At 746 terms, debug asin overflowed a coefficient at
160 bits; release returned an inaccurate finite value.

The repair treats planning as scheduling, not proof. Constructors stay lazy.
The generic series kernels validate the operand approximation they already
need, before evaluating any series term. If its integer sample is `a` at
precision `s`, then `|x - a*2^s| <= 2^s`. For `k = -s-3`, the check
`bits(|a|) <= k` proves `|a| <= 2^k-1`, hence `|x| <= 1/8`, including sample
error. This is an integer bit-length check with no new allocation or second
operand request. Outside that range, asin uses its existing stable half-angle
transform and atanh its existing logarithmic quotient. These fallbacks do not
re-enter the same prescaled node, so the guard cannot cause a reduction loop.

For coarse requests `p >= 1`, zero is valid for asin on its whole domain:
`|asin(x)| <= pi/2 < 2`. Atanh is unbounded at its domain endpoints, so its
coarse zero shortcut first uses an exact cached magnitude or the bounded
eight-bit probe `|approx(-8)| <= 31`, again proving `|x| <= 1/8`. Unknown or
inexact facts cannot bypass that proof. Failed guards use the general formula.
Abort checks precede work and fallback; aborted results are not published.

The approximation boundary also repairs old serialized prescaled nodes whose
public construction selected the wrong schedule. No serialization format,
public API, node variant, object field, or dependency was added. This is not a
promise to validate all arbitrary malformed serialized expression variants.

## Rejected candidates

- Constructor-only exact-magnitude dispatch plus kernel guards: numerically
  repaired the corpus, but lost two existing tiny-nonrational dispatch tests
  and was approximately four times slower on one pilot small-sum control.
  Preserved as `neighbor-strict-*` source/binary/log evidence, not retained.
- Lazy scheduling with a separate six-bit probe: restored dispatch shape, but
  its coarse uncertainty needlessly fell back for a tested small-sum control.
  Preserved as `neighbor-coarse-guard-*` / `neighbor-controls-scheduled`.
- Separate eight-bit probe: accepted that small control, but still duplicated
  input approximation and allocation. Preserved as `neighbor-certified-*`.
- The sample-reusing kernel guard is the selected version, frozen under
  `.audit-numbers.rjcbha/neighbor-sample-*.rs` and `neighbor-controls-sample`.

These pilot comparisons guided scope; only the final paired measurements below
are used for reported performance conclusions. No donor numerical code copied.

## Correctness evidence

- Original frozen `neighbor_probe.rs` unchanged, SHA-256
  `ff5ed272647aa1485f900dc0edc46e6ba21abe5cab1a2a08cb944d200ae28d80`.
  4096-bit outward MPFR square-root enclosures, exact rational combination,
  directed asin/atanh, and explicit disjoint-interval failure classification.
- Same 80 public requests per build: two operations, ten chain lengths,
  8/32/80/160 bits. Before: 42 pass per build; debug 37 numeric failures and
  one overflow exception; release 38 numeric failures. Final: **80/80 pass in
  both debug and release**, with no exception or cap.
- Four initial regressions failed before and passed after. Public-dispatch
  assertions were subsequently relaxed to allow the correctly checked lazy
  schedule; numerical assertions remain. The frozen public corpus separately
  establishes the old numerical failures.
- Final tests add 46,050 directed-MPFR checks over every legal eight-bit cache
  rounding on a 2,047-point grid, both functions, and five request precisions
  including cold coarse output, refinement, and cache reuse. Another 624
  checks exercise every legal rounding immediately around the reused working
  sample's series boundary at four output precisions, both signs/functions.
- Also covers positive/negative long chains, direct legacy prescaled nodes,
  serde roundtrips, and abort/cache recovery. Rational kernels are unchanged.
- Final full tests: **735 debug passes**, **841 release/all-feature passes**,
  including 19/24 doctests. All-feature library Clippy with `-D warnings` and
  repository format check pass. Current-source downstream --lib tests:
  Hyperlattice 19, Hyperlimit 242, Hypertri 3, Hypersolve 432 (696 total).
- A longer Hypercurve run began against the separate-probe candidate, before
  the sample-reuse revision. It must not be credited as final-source coverage.
  Its terminal result and the final-source rerun belong in the root ledger.
- `neighbor-memcheck.log` checks the final debug boundary/abort tests with
  Valgrind's error exit code enabled: three tests pass in 191.83 seconds,
  terminal exit zero, zero errors. Leak checking is disabled; remaining shared
  cache allocations are not a leak-freedom result.

## Performance, memory, and size

`neighbor_controls.rs` uses preconstructed inputs and times clone, public
function construction, and approximation. Each process handles 256 fresh
inputs at alternating 32/128 bits; every output is independently enclosed by
4096-bit outward MPFR outside timing. No timings for numerically wrong outputs
are reported as useful work.

270 CPU6-pinned fresh-process observations: 252 paired controls (two functions,
seven families, nine alternating rounds) and 18 repaired-case measurements.
Exclude round zero: 240 observations, eight pairs per family. The analyzer
hashes sources/binaries, validates every record and checksum, and reports
20,000-resample paired-bootstrap intervals in `neighbor-bench-analysis.json`.

All 14 control families have exactly unchanged allocation counts and requested
bytes. CPU ratios are broadly near parity, with wide intervals under concurrent
system work. No general speedup is claimed; even the apparently faster exact
rational atanh control does not execute the changed generic kernel. Timings
include allocator counters; requested bytes are cumulative, not peak residency.

For 256 repaired inputs, median CPU time is about 250.37 ms asin / 167.53 ms
atanh; allocation requests total 427,684,256 / 293,956,968 bytes respectively.
There is no meaningful speed ratio against the numerically incorrect baseline.

Harness binary: file 2,216,160 -> 2,217,096 bytes (+936); text +816, data +48,
bss -880, loaded sum -16 bytes. This is effectively unchanged total loaded
size, not a claim that every Hyper consumer binary shrinks.

Incremental production diff over the retained atan checkpoint: +46/-3 lines;
204 new test lines. Display and atan repairs remain preserved. High-precision
coefficient/counter limits in other series are not established by this bounded
test grid and remain a separate completeness audit concern.

## Reproduction

Build the qualification Cargo manifest offline with target directory
`.audit-targets/ireal-derivative-18555`; binaries are copied, never overwritten,
into `.audit-numbers.rjcbha`. `run-neighbor-repair.mjs debug|release|paired|repaired`
uses frozen final binaries and records every process, cap, error, and hash.
Subprocess execution requires explicit sandbox approval on this host.
`analyze-neighbor-bench.mjs` checks the immutable timing corpus.
`analyze-neighbor.mjs` checks the completed scalar qualification checkpoint.
Earlier donor and atan validators deliberately retain their historical scope.
