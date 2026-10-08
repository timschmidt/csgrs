# Plume functional/logistic qualification and paired benchmark

This checkpoint is progress, not closure of every report container or the wider
reference inventory. No Hyper production change is selected. Hyperreal is
bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c. Original174 source-container hashes
are unchanged. No original source, archived SPARC executable or Maple worksheet
was executed/modified: only copied Haskell sources are built with modern GHC.

## Build isolation

Scratch .audit-plume-late.4Yth3g/overlay/Tests.hs exposes the hidden test
functions. The only changes are its module export list and removal of one final
blank line, recorded exactly in late-compatibility.patch. Other modules import
from the prior main compatibility copy .audit-plume.SZHOs8/compat, using only
the already-recorded Utils/Alex compatibility changes. No numerical recurrence
is changed. Prior KernelProbe/InstrumentedProbe sources and benches stay frozen.

GHC9.6.7, Node22.22.2, Rug1.30/MPFR4.2.2, Linux x86_64. From workspace root:

```sh
env CCACHE_DISABLE=1 TMPDIR=/home/tim/Documents/GitHub/workspace/.audit-plume-late.4Yth3g /tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc --make -O2 -rtsopts -i/home/tim/Documents/GitHub/workspace/.audit-plume-late.4Yth3g/overlay -i/home/tim/Documents/GitHub/workspace/.audit-plume.SZHOs8/compat exact-real-references/plume-qualification/LateProbe.hs -outputdir .audit-plume-late.4Yth3g/build -o .audit-plume-late.4Yth3g/late-probe
```

Repeat with -O0, debug-build and late-probe-debug. Build logs name the imported
paths. Probe emits an exact midpoint/radius from a finite signed prefix; input
representations include greedy/delayed signed digits and native decimal imports.
Stream-only wrappers rebase decimal mantissas using the native binary shift,
not by replacing the input with an approximation. Iteration0 checks this boundary.

## Numerical and resource checks

```sh
node exact-real-references/plume-qualification/run-late.mjs logistic
node exact-real-references/plume-qualification/run-late.mjs logistic --debug
node exact-real-references/plume-qualification/run-late.mjs functional
node exact-real-references/plume-qualification/run-late.mjs functional --debug
node exact-real-references/plume-qualification/run-late.mjs long
node exact-real-references/plume-qualification/run-late.mjs first-digit
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo build --offline --release --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin late_oracle
.audit-targets/ireal-derivative-18555/release/late_oracle logistic exact-real-references/plume-qualification/late-logistic-results.tsv
.audit-targets/ireal-derivative-18555/release/late_oracle logistic exact-real-references/plume-qualification/late-long-results.tsv
.audit-targets/ireal-derivative-18555/release/late_oracle functional exact-real-references/plume-qualification/late-functional-results.tsv
.audit-targets/ireal-derivative-18555/release/late_oracle selfcheck
```

Both debug-result TSV files are also separately qualified. Initial sandbox
child-process EPERM records are preserved separately; approved reruns have none.
All requests have256MB GHC heap caps and explicit per-process timeouts. Numeric
tests were allowed to overlap; their timeouts are observations under those run
conditions, not a performance comparison or proof of nontermination.

- Short grid:392 requests/build over7 variants,7 values,2 representations and
  iterations0..3, at12 fractional bits. Each build returns350 prefixes;42 hit
  750ms caps. Returned TSVs are byte-identical. Independent qualification finds
  28 wrong dyadic-stream results and no other returned-prefix failures.
- Long:42 requests over the7 variants, two decimal inputs and10/40/60
  iterations at32 bits.34 finite prefixes, two wrong dyadic-stream results;
  eight2s caps in dyadic variants at40/60 iterations. All signed/cross paths
  return correctly qualified prefixes at every requested long iteration count.
- Functionals: quadratic minimum, quadratic maximum, square integral and
  narrow positive-domain reciprocal maximum at0/4/6/8 bits. O2 returns12/16
  and O0 returns10/16 before3s caps; all returned values pass, and shared
  outputs are identical. Numeric max reference for0.23+1.1*x-x*x is213/400.
  The reciprocal example reproduces1857/128 +/-1/256 at8 bits, matching the
  worksheet enclosure with a different mantissa/exponent representation.
- First-digit:8/8 observations for two representations of1/2 and four
  iteration counts. The greedy dyadic stream has a2047-bit denominator
  exponent at iteration10, while delayed digits return0. It does not reproduce
  the report's2558 exponent; the active dyadic-stream recurrence is numerically
  wrong, so its big digit is not valid logistic benchmark evidence.

The logistic oracle uses a dependency-safe interval extension of4*x*(1-x),
performing exact GMP rational endpoint arithmetic and rounding outward to4096
MPFR bits after every step. Clipping to[0,1] uses the known invariant. It does
not incorrectly treat this nonmonotone function as endpoint-monotone.91 checks
against an independently expanded exact rational recurrence through iteration12
pass. No binary64 approximation is trusted for correctness.

Hyper controls use public Computable constructors and four precision/history
requests per case.40 input/iteration cases give160 passing checks per release/
debug build, including zero, fixed points and60 iterations. A preliminary
eager-rational-growth concern does not occur in this workload: multiplying by4
creates an Offset node, keeping the later expression demand-driven. No policy
change is justified by that disproved concern. Bounded single runs at10/20/60
iterations pass; their /usr/bin/time output is diagnostic, not benchmarking.

```sh
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo run --offline --release --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin late_oracle hyper-grid
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo run --offline --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin late_oracle hyper-grid
valgrind --tool=memcheck --error-exitcode=77 --leak-check=no .audit-targets/ireal-derivative-18555/release/late_oracle hyper-grid
```

Memcheck has zero memory errors. No leak or full-library qualification claim.

## Paired benchmark: qualified signed vs dyadic-float recurrences

```sh
env CCACHE_DISABLE=1 TMPDIR=/home/tim/Documents/GitHub/workspace/.audit-plume-late.4Yth3g /tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc --make -O2 -rtsopts -i/home/tim/Documents/GitHub/workspace/.audit-plume-late.4Yth3g/overlay -i/home/tim/Documents/GitHub/workspace/.audit-plume.SZHOs8/compat exact-real-references/plume-qualification/LateBenchmark.hs -outputdir .audit-plume-late.4Yth3g/bench-build -o .audit-plume-late.4Yth3g/late-benchmark
node exact-real-references/plume-qualification/run-late-bench.mjs
node exact-real-references/plume-qualification/analyze-late.mjs
```

108 fresh processes, pinned toCPU6, two inputs (0.1,0.5467),4/8/10 iterations,
32-bit fractional output demand, nine alternating-order rounds. Discard12
round0 warmups:96 observations/eight per variant/input/iteration group. Every
output passes an exact Rational oracle outside the timed and allocation region.
No child permission or process-status failures are mixed into the benchmark.

The timer includes native decimal input evaluation, recurrence construction/
evaluation and forcing output digits. It excludes process startup, exact prefix
decoding/checking and the explicit final GC. Automatic GC during evaluation is
included. Cumulative managed allocation is measured across a final collection;
it is not retained heap or RSS. This is not a Hyper/cross-language/binary-size
comparison. GHC kernel/input costs and representation-specific exponents remain
part of the measured public workload.

Median dyadic/signed CPU ratios are15.56/16.19 at4 iterations,25.83/26.89
at8, and147.27/124.42 at10, for0.1/0.5467 respectively. Allocation ratios
are17.04/18.48,39.33/44.50 and90.65/103.30. Signed10-iteration median CPU
is about1.02/1.06ms; dyadic is149.62/132.25ms. The lower signed demands are
submillisecond and variable. Full min/max, eight paired ratios and a descriptive
seeded10000-resample paired-median bootstrap are retained in late-summary.json;
these finite-host intervals are not universal performance guarantees.

Benchmark source and its frozen copy SHA-256:
d3ca7d8aa46ba789f1e42a89c584fe5aa0244d67f377034c071abf5feb58240d.
Benchmark binary:
140522bd3521d354dd55494bf5b8949a28b6483399c5b012d2215937a1ebe271.
LateProbe source:
b02ff829139382457ad20cba4906b03f43291090e9cd1017ba1353bd12c8676c.
Final oracle source:
e37beb789ee61d654f5d8f4b221afd1f5425a26ffcb8c82160c33719a7d9a10a.

The validator checks these hashes, every original inventory hash, the exact
Tests bridge, all process/numeric counts, shared O0/O2 outputs, Hyper/oracle/
Memcheck evidence and108 benchmark rows. Its initial check correctly detected
the unrecorded final-blank-line removal; that error log is kept, and the exact
nonsemantic difference is now recorded rather than broadly ignoring whitespace.
