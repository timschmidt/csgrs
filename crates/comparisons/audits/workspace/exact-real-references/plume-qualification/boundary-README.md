# Plume boundaries and Hyper fixed-decimal productivity

This is a qualified improvement checkpoint, not whole-Plume/ecosystem closure.
All174 donor source-container hashes are unchanged. Hyperreal baseline is
bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c. The only production change in this
slice is src/computable/format.rs: two executable lines replace one, plus four
comment lines and37 regression-test lines. No donor implementation was copied.

## Donor controls

BoundaryProbe.hs imports the previously qualified main compatibility tree
.audit-plume.SZHOs8/compat. No new donor bridge or numerical edit. Independent
Haskell Data.Ratio Integer arithmetic constructs inputs/checks finite prefixes.
Separate Node BigInt arithmetic rechecks every decimal row and finite process
result. GHC9.6.7, Node22.22.2, Linux x86_64. Build from workspace root:

```sh
env CCACHE_DISABLE=1 TMPDIR=/home/tim/Documents/GitHub/workspace/.audit-plume-boundary.XNOLY9 /tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc --make -O2 -rtsopts -i/home/tim/Documents/GitHub/workspace/.audit-plume.SZHOs8/compat exact-real-references/plume-qualification/BoundaryProbe.hs -outputdir .audit-plume-boundary.XNOLY9/build -o .audit-plume-boundary.XNOLY9/boundary-probe
```

Repeat with-O0, debug-build and boundary-probe-debug. Run each mode both with
and without --debug:

```sh
node exact-real-references/plume-qualification/run-boundary.mjs grid
node exact-real-references/plume-qualification/run-boundary.mjs format
node exact-real-references/plume-qualification/run-boundary.mjs printed
node exact-real-references/plume-qualification/run-boundary.mjs productive
node exact-real-references/plume-qualification/run-boundary.mjs literal
```

Each child has a256MB GHC heap cap; productive/literal cases have750ms caps.
Initial sandbox EPERM records are separate; approved reruns contain none.
Qualification processes could overlap, so caps are observations, not timing
benchmarks or stand-alone proofs of nontermination.

- Grid:5128 checks/build,17 failures, zero exceptions, O0/O2 byte-identical.
  All signed clipped helpers, valid prefix identities and bounded signed
  normalizers pass. All32 dyadic normalization values qualify, but17/32
  violate their requested normalization cap. Value preservation and bounded
  demand are deliberately separate requirements.
- Printed formulas:51 exact clipped-subtraction cases reject32 printed cases;
  34 residual cases reject32 printed cases. Source implementations pass the
  independent finite-prefix grid. The report's dyadic-division closure claim
  is disproved by exact1/(3/4)=4/3.
- Decimal output:2080 cases/build,128 failures beyond10^-places, zero
  exceptions, all outputs O0/O2-identical.65 mantissas on the1/32 grid,
  greedy/delayed representations, exponents-4/0/1/4, places0/1/3/6. Every row
  independently rechecked with BigInt. Failures occur for positive exponents
  and positive requested places, including printing1 for represented2.
  sbfDec' ignores fcarry from the fractional converter, so endpoint/carry
  representations need not produce even the promised approximation. This
  is not merely a choice of halfway rounding convention.
- Productivity:31 requests/build,21 finite,10 caps. Finite outputs qualify.
  Signed bounded normalizers work on zero. Dyadic caps>0 fail to return;
  unbounded signed normalization also does. The no-negative-exponent wrapper
  passes an already-negative exponent as a negative recursion cap and fails
  to return for the tested zero inputs. Multiplication returns a zero prefix
  at exponent sums498/499 but hits the cap at500/501; nonzero controls return
  at all four sums. The source's unconditional switch to normalization is
  therefore not a safe scheduling policy for all defined real inputs.
- Direct literal conversion:26 requests/build,11 finite,15 exceptions, no
  caps. Empty input and lone signs become zero; trailing decimal points fail;
  unsupported exponent notation, whitespace and invalid digits fail. These
  are direct helper observations, not a change to the calculator lexer grammar.

## Retained Hyper change

The corresponding Hyper controls uncovered a separate issue: public
sqrt(2)*sqrt(2)-2 approximates successfully at finite precision, but fixed
decimal formatting searches for a leading nonzero bit. The existing
iter_msd() fallback can refine toward roughly Precision::MIN/3 even when the
caller asks for zero decimal places. Two10s trace runs stop at formatting the
zero cancellation (exit124), after all2080 ordinary format controls passed;
the first untraced30s observation produced no completion record. Trace logs
are boundary-hyper-trace-{initial,detail}.log. No timeout is called a speedup.

Display now calls iter_msd_stop(-enough_bits(0, requested_places)), using scale0
when no leading bit is established. Existing guarded approximation, sign and
decimal digit conversion remain unchanged. A value unresolved at this floor
is already far below the decimal output unit. It does not require an exact
zero/equality decision. Scientific notation is intentionally unchanged: its
significant-digit/exponent contract differs from fixed decimal places.

Two unit regressions check cancellation and positive/negative tiny radicals,
including exact cache-demand bounds, sign flags and warmed histories. Public
external controls pass2188 decimal/history and300 shifted-zero/nonzero
precision/history checks in release/debug. They allow public simplification;
these are not opaque/raw-node tests. An initial audit compile error used the
private shift_left method; the final harness uses public multiplication by
exact powers of two. The error log is retained separately.

```sh
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo build --offline --release --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin boundary_hyper_controls
.audit-targets/ireal-derivative-18555/release/boundary_hyper_controls
```

Repeat without --release for the debug build/run. Memcheck passes with zero
errors (--error-exitcode=77 --leak-check=no); no leak qualification claim.
Full Hyperreal default-debug tests pass723 including19 doctests; release
all-features passes828 including24 doctests. cargo fmt --all -- --check and
all-feature library Clippy with-D warnings pass. Downstream library tests pass
Hyperlattice19, Hyperlimit242, Hypertri3, Hypersolve432. Hypercurve's full892
library-test run subsequently completed:891 passed,1 ignored,0 failures in
578.20s (session31037 exit0). Total scoped downstream library passes:1587;
the ignored case is not a pass. This is not a downstream integration-test claim.
No push or commit has been performed.

## Paired before/after benchmark

Frozen numerical benchmark source boundary_format_bench.rs:
bd163ce02fa4cdf1f0be9b18837046cf8bc3cc9f0038a5315bba1519f4605520.
Before binary .audit-plume-boundary.XNOLY9/format-before:
c290fb7ba7de8ca71a8dd9c56e6e510f3fe5ea3c3d4ac172dc913c9066a6eddd.
After binary .audit-plume-boundary.XNOLY9/format-after:
ec8a4de42104346ae5518409369dfff749c16334891e871b7830515147844f33.
The before binary was compiled against clean bd92d87 before the production
edit; both use the same manifest, source, target and release settings.

```sh
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo build --offline --release --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin boundary_format_bench
node exact-real-references/plume-qualification/run-boundary-bench.mjs
node exact-real-references/plume-qualification/analyze-boundary.mjs
```

72 fresh processes pinned toCPU6, nine alternating-order rounds, four families,
512 independently constructed values each. Round0 is discarded:64 retained
observations, eight per family/variant. All decimal results are independently
GMP-qualified outside timing, using rational or squared radical enclosures.
Checksums match before/after. Construction, process startup, oracle work and
final drops are excluded. CPU and wall time cover formatting/output allocation;
Rust allocator counts are cumulative requests, not RSS or retained heap.

| Family | Before/after median CPU | Paired-median bootstrap95% | Allocated bytes before/after |
| --- | ---: | --- | --- |
| Rational |0.9991x|0.9511--1.0125|364079 /364079|
| Tiny rational |1.0005x|0.9902--1.0034|74973843 /74973843|
| Tiny radical |140.9181x|139.5118--141.8401|75703168 /213504|
| Ordinary radical |1.0232x|0.9937--1.0497|659072 /659072|

Tiny-radical median CPU is46.465ms before,0.330ms after (whole512-value batch),
with99.718% fewer requested allocation bytes. This is a scoped workload gain,
not a general library speedup. Ordinary samples are near/submillisecond and
variable; the results do not justify claiming an ordinary-value speedup.
Full extrema, paired ratios and seeded10000-resample bootstrap intervals are
preserved. An initial analyzer compared0/-0 with Object.is; it now compares
integer exponents as BigInt. The initial error is retained. An initial LCG
low-bit resampling draft produced degenerate intervals for eight observations;
the final analyzer uses high bits and has a nondegenerate-bootstrap control.
Previous late-analysis already used high-bit resampling and is unaffected.

These audit executables differ by+64 file bytes; size reports+16 text,-16 bss,
unchanged data/aggregate loaded size. This is not a universal application-size
claim. The correctness/productivity benefit warrants the tiny production edit.
No new dependencies, ABI or public API surface were added to Hyperreal.
