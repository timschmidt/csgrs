# Constructible field and contract qualification — 2026-09-06

This continues the four-file/490-line source audit at donor commit
`46d760cbd2d21f955ec96c8fe2c13fdf3b2dd9d0`. Original donor and dependency bytes
remain unchanged. No production Hyper change was made in this slice. Full
reference-queue completion is not claimed.

## Independent identity corpus

`generate-field-corpus.mjs` generated 1,324 cases; `field-corpus.tsv` SHA256
`9ff02bde757b516d384b3460c20ee3fd2daab2b9d6582e9bf88705c04c45ebff`.
The generator refuses overwrites. Expected identities and signs use integer
algebra, not donor output or floating-point comparisons:

- 1,168 quadratic cases: radicands 2,3,5,7; denominator 1,2; both coefficients
  -3 through 3. Independently expand the square, select the principal root by
  the exact rational norm, compare signs, and rationalize nonzero inverses.
- 140 nested cases: depths 1–5, 28 per depth. Start r=2, repeatedly replace r
  by 2+sqrt(r). In every case 1<sqrt(r)<2 proves the sign of a±sqrt(r), a=-3..3.
  Expanded squares and conjugate-denominator inverses exercise different
  expressions, not merely x*x versus x.square().
- 16 independent-radical cases: all sign choices for pairs (2,3),(2,5),(3,7),
  (5,7), with the square expanded to a+b±2sqrt(ab).

Prefix grammar and TSV schema are in the generator and probes. Both probes
parse the same bytes through public APIs. Native exact Ord gives 1,324/1,324
PASS in O0 and O2, exit0, no 60s cap. Hyper debug/release give 1,310 PASS,
14 UNKNOWN, zero WRONG at -2048, exit3. All unknowns are depth-five inverse
identities. At -4096 release proves 24/28 depth-five cases; the four remaining
unknowns have |a|=3. At -16384 both debug and release prove all 28/28, exit0.
Thus this corpus establishes a bounded proof-cost difference, not an incorrect
answer or an identity missing from Hyper's tested exact capabilities.

The per-row native correctness logs contain process CPU picoseconds; Hyper
correctness logs contain wall nanoseconds. They are NOT paired benchmarks.
`analyze-field.mjs` checks source hashes, case identity/uniqueness, expected
answers, unknown locations, native contracts, and benchmark observations.

## Operation contracts and examples

Corrected ContractProbe passes 126/126 in both O0/O2: 85 dyadic powers,
17 negative-base integer powers, 9 zero/domain cases, 14 unsupported unary
functions, and unsupported pi. Both optimization levels also pass the golden
ratio/Fibonacci identities, the documented four-step AGM floor, the exact
17th-root-of-unity identity, and the documented close radical sum's broad
finite-range check. The latter alone is not an accuracy proof; see the separate
directed-MPFR floating qualification.

First-run logs are preserved honestly: the original golden test accidentally
defaulted to floating point; two exception expectations used DivideByZero
instead of GHC's RatioZeroDenominator. Those were harness errors, not donor
arithmetic failures. The corrected source explicitly uses Construct and the
actual typed rational-zero-denominator exception. The initial binary remains
`contracts-before-O2`; the corrected binaries have distinct names.

Build recipe is the direct GHC9.6.7 recipe in boundary-README.md, with each
Probe.hs passed explicitly, -O0/-O2, -fforce-recomp, and distinct output names.
FieldBench imports the frozen FieldProbe Main module and uses -main-is
FieldBench. No claim of compatibility with the archived Cabal upper bounds.
Hyper binaries use the existing pinned numbers-qualification Cargo harness,
offline, in debug/release. The benchmark has no counting allocator.

## Paired benchmark

`FieldBench.hs` and `hyper_field_bench.rs` read fresh group-file bytes and build
fresh exact objects every repetition. No previous comparison or approximation
is reused. Timed scope includes cached-file ingestion, parsing, construction,
and exact comparison; it is not an isolated arithmetic-kernel microbenchmark.
Both report process CPU and monotonic wall nanoseconds. Every result is checked.
Hyper's comparison floor is -16384, so incomplete answers cannot count as fast
successes. GHC O2 runs with +RTS -s; both run under GNU time for whole-process
maximum RSS. Native allocation bytes are GHC heap allocation, not live memory
and not directly comparable with Rust allocator traffic.

Run `partition-field-corpus.mjs` then `run-field-bench.mjs RUN-NAME` once per
fresh output name. The successful run is `field-bench-approved`: 126 terminal
observations, seven families, one discarded warm-up plus eight measured pairs,
alternating executable order on CPU6. 1,103,040 verified comparisons, including
warm-ups; no timeouts, wrong answers, or unresolved results. Initial sandbox
Node spawnSync reported EPERM despite a child's exit0: that sample was rejected,
its logs retained, and the separately approved run used entirely new files.

Median paired CPU ratio, Hyper/native, with deterministic 10,000-resample
percentile bootstrap intervals for eight pairs:

| Family | Ratio | 95% interval |
| --- | ---: | ---: |
| quadratic | 0.2242 | 0.1889–0.2369 |
| tower-1 | 0.2136 | 0.2077–0.2356 |
| tower-2 | 0.6600 | 0.6366–0.7131 |
| tower-3 | 0.8111 | 0.7523–0.8465 |
| tower-4 | 1.2069 | 1.1703–1.3024 |
| tower-5 | 2.0862 | 1.9739–2.2216 |
| independent | 0.3637 | 0.3514–0.3765 |

Wall ratios closely track CPU; all raw output and exact results are retained
in field-bench-analysis.json. These are workload/language/runtime comparisons,
not a before/after Hyper patch or proof of universal speed. Native median
whole-process maximum RSS spans 10,088–11,840 KiB; Hyper 3,496–3,688 KiB.
GHC allocation spans 678,160,552–1,368,145,320 bytes per observation, overwhelmingly
short-lived; maximum RSS must not be confused with that cumulative quantity.

Frozen benchmark file sizes: native 3,378,632 bytes, Hyper 1,814,128 bytes.
GNU size text/data/bss: native 2,698,900/150,864/20,408; Hyper
1,284,137/218,704/1,048. These include different runtimes and link configurations,
so they are descriptive standalone-binary observations, not library-size or
patch-size savings.

## Transfer review

The donor recursively tests whether a square root already belongs to a
quadratic tower before adjoining it, and signs opposing terms a+b*sqrt(r)
through a²-b²r. This requires exact base-field equality and irreducible positive
extensions; its rational base is narrower than general computable Real.

Hyperreal already has rational-radicand QuadraticSurd normal forms and nested
root reduction. Its bounded algebraic-separation certificate handles the
deeper corpus, but depth-five inverse bounds require more refinement. This is
a measured candidate for a cold proof-cost improvement, not justification for
putting an eager recursive field into every scalar or increasing object size.

Current Hypercurve already owns Arc-shared recursive quadratic towers,
positive-sheet generator reuse, field joining, and recursive squared-magnitude
sign decisions. Its selected algebraic bases are not the donor's Q base:
blindly importing donor square-membership/zero assumptions would be unsound.
Hypersolve represents selected-field coefficients as reduced rational functions
modulo retained-root evidence, with exact denominator checks and explicit
undecided states. DenseTensorPolynomial preserves selected axes and correlations;
it is not a duplicate arbitrary constructive-real backend.

This review read current Hypercurve bezier_offset.rs ranges 6370–6520,
56080–56640, 56700–56920, 57142–57450, 57645–57772; these are partial-file
inspection ranges, NOT full-file credit. Snapshot SHA256
f37d3795ad1bead2de3e4bf7c20fb3d6fee60526c961c3d7978c1768b406af9f,
HEAD149bea52c9c16797001fee2a95cf1dc739fe82e5 with user-owned edits. Hypersolve
algebraic_fiber.rs 1–150 and 3060–3260 SHA256
2ec1ccb96feeb2dff2f62753964ac666f840b6ca24a3b75ed8113dcff0b0c966;
tensor_resultant.rs 1–220 SHA256
f5f499af1162a9f7b66b1532b21a453273aa705cab0073fabca1f17f8b1a97df.
These files were inspected, not changed or newly regression-qualified.

Decision so far: preserve existing scalar repairs, keep the corpus and
reproducible measurements, do not transplant a duplicate eager backend. The
API and floating qualification are complete for the declared corpora (see
api-README.md and float-README.md). The target's cold scalar proof-cost transfer
follow-up is OPEN; the broad goal is ACTIVE.

## Frozen binary provenance

All paths below are under workspace `.audit-constructible-build.l4UDoe/`.

| Binary | SHA256 |
| --- | --- |
| field-before-O0 | ec78322af632cc62d3777e69854054bb9cec06c001120172e377f8dc3f97ee6b |
| field-before-O2 | 34011e2683b44f5c72ed5a1098c2d799a8be85222548bfda9fd7f952e2ca49a9 |
| hyper-field-before-debug | 0c0d40f7f526f12bc08aa7cee026af88b5b6aa3fccf6e075f4a1ec6bcc48b085 |
| hyper-field-before-release | b3b67bb5bd94a75533ca4f8a9376f6cefbf1f66179fa3a373d03d28e4387b777 |
| field-bench-O2 | dec74b91ccfd8725302f57367ea52414f20550c8a860c16b0cac746ca1eabc20 |
| hyper-field-bench-release | 25ed91a0df3f22d1f4b363b72381d4774f7c8c331c3aa3b0acc0e4d695fbb00c |
| contracts-before-O2 (initial harness) | db9bbc330ee9f2b86b57ebd0c356700254b0a70ef3987488b8692de7f60971a7 |
| contracts-corrected-O0 | 899b3e93b22de6c31bacd339516292213586b263c05cf88532d8f70f5f9f0bb7 |
| contracts-corrected-O2 | cda46196dbbaf725fbb2c07ee6d712cb643af17a6af1cea593e35b5366099f51 |
