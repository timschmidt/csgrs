# Ruffini matrix, transform and integer checkpoint — partial, 2026-09-06

The [original donor pin](https://github.com/jonas-lj/Ruffini/tree/82d552fee22d92e493936183fab8672517694e56)
remains unchanged. Explicit full-read credit is now 148/245 tracked files and
8,122/26,234 physical lines: all common, integers, reals, parser, permutations
and finite-fields files, plus selected metadata and a constructive-real demo.
The 97 remaining files are still UNREAD/in scope. File notes and the two root
TSVs distinguish actual reading from compilation and hash verification.

This slice adds no production change. It qualifies matrix/transform/integer
questions from the source review and checks current Hyper behavior. Parser,
permutation and extension-field source hypotheses are not executed findings yet.
The earlier scalar/shared checkpoint in README.md remains historical evidence;
this document supplements it without relabeling its original coverage.

## Native provenance

`run-matrix.mjs` compiles unchanged integer main files except IntegerPolynomial,
whose polynomial dependency has not yet been compiled in this qualification,
plus unchanged TestIntegers and the new RuffiniMatrix harness. It uses the
already pinned common/reals classes and dependency jars from README.md.
Output classes live in `.audit-ruffini-build.LmZgYM/matrix`; previous `/classes`
and `/tests` artifacts are preserved. The native build manifest fingerprints
every new source and class, and the aggregate also verifies prior dependencies
and common/reals sources/classes.

javac21.0.11 compiles with `--release 16` and UTF-8; OpenJDK25.0.4 runs the code.
This is direct compilation, not qualification of the original Maven lifecycle
or Java16 runtime. Separate JIT and `-Xint` processes run with assertions, 512MiB
heap, 1MiB stack and common-pool parallelism1. All eight processes exit0 before
their 120-second caps. Exit0 means the harness completed, **not all probes pass**:
each probe emits PASS, WRONG or EXCEPTION and is classified by the analyzer.

An initial harness-only overload ambiguity in Integers.divide(MIN_VALUE,3) was
fixed using explicitly boxed Integer arguments. Original diagnostics remain
in `matrix-build-initial-failure.log`; donor sources were not edited.

## Native results

Both modes produce identical complete arithmetic, boundary and transform logs.
There are 2,477 supplemental probes per mode, plus the two original integer
JUnit tests, which both pass in each mode. Independent BigInteger triple-loop
products/cofactor determinants and exact modular arithmetic decide results;
no approximate float is used as an exact integer oracle.

| Family | Probes per mode | Result |
| --- | ---: | --- |
| Ordinary rectangular product | 1,296 | All pass |
| Strassen product | 264 | 186 pass; 78 null-padding exceptions |
| Gram product A*Aᵀ | 128 | All pass |
| Nonempty determinant | 120 | All pass |
| Empty lazy determinant | 1 | Returns0 instead of1 |
| Matrix equality | 32 | 8 square pass; 24 rectangular exceptions |
| Column collapse | 32 | 8 square pass; 12 wide wrong; 12 tall exceptions |
| One-variable congruence | 444 | 36 zero RHS pass; 408 nonzero RHS wrong |
| Modular square-root of a square | 75 | 58 pass; 17 unsupported p=17 exceptions |
| Other boundary checks | 16 | 4 controls pass; 9 wrong; 3 exceptions |
| Exact DFT and inverse roundtrip | 69 | All pass |

The 1,296 products use dimensions1,2,3,4,5,8, three deterministic signed input
seeds, and both concrete/lazy matrices. The Strassen grid uses sizes
1,2,3,4,5,6,7,8,9,12,16, cutoffs1,2,4,8, three seeds and both representations.
Exactly the tested non-power-of-two cases with n>cutoff and cutoff>1 fail:
padding inserts null, but the ordinary multiplication base case is not null-safe.
Default cutoff1 passes the tested sizes. This is not a claim about all sizes.

Other boundary results:

- A 1×1 complex identity inverse throws IllegalArgumentException because the
  inversion check uses object equality; the real identity control passes.
- A built sparse matrix changes after modifying its builder. The separate
  public mutable-copy independence control passes; do not conflate the paths.
- Predicate vector equality accepts [1] equal to [1,2]; reversing the operands
  throws instead of returning false.
- Complex projection of i onto1 gives -i. Tall3×2 QR throws an array-index error.
- List ksubsets argument reversal fails; the array overload control passes.
- Width/populator multidimensional construction reports dimension3 for3×2;
  the explicit-shape construction control passes.
- Limb addition drops a final carry; limb multiply returns null; int MIN_VALUE
  norm is negative; an int division intermediate overflows.

Congruence tests exhaust p=1..m-1 and RHS=0..m-1 for m=5,7,11,17. Square-root
tests square every representative for prime moduli3,5,7,11,13,17,19; the p=17
failure is an explicit unsupported branch, not a wrong computed root. DFT
tests use all positive divisors of p-1 for p=7,13,17,31, independently find
primitive generators and compare against an exact quadratic modular sum before
checking the inverse. Invalid roots, malformed sizes and precision overflow
are not covered by those valid-domain transform results.

Log SHA256 values, identical across modes:

- Arithmetic: `24b9738cbf9798f72625a9e6919b8745720603eb6c82d519979fad0ecd09212d`.
- Boundaries: `eceb08ecc0b311013845a11407cdcbcdb7be87683fc81bc9a82bdbe687a37bda`.
- Transforms: `1867c450392c78567dbec8d4029c62b992ebb4684c1b7a7d0b51a6f6218ee63f`.

## Current Hyper controls

The isolated publish=false `matrix-hyper` harness depends on current Hyperreal,
Hyperlattice, Hyperlimit and Hypersolve. A 201-source/manifest-file hash snapshot
is checked before and after both builds/runs. No production crate is edited.
Offline Cargo uses the existing private target directory and workspace TMPDIR.
An initial harness assumption that PredicatePolicy implements Default was
corrected to the required explicit STRICT policy; its diagnostic is retained
in `hyper-matrix-debug-build-failure-1788736376139.log`. It is not a Hyper failure.

Debug and release each pass 420 grouped checks:

- 72 determinants for sizes0..5 against an independent bounded integer cofactor
  oracle, including the empty determinant1.
- 96 strictly diagonally dominant dense Bareiss solves, sizes1..8, against a
  known exact integer solution and exact RHS.
- Dimension mismatch and singular-system domain checks.
- 240 Matrix3/Matrix4 product, determinant and transpose-involution checks
  across40 seeds. Each product check compares every entry exactly.
- Five identity-inverse/complex reciprocal/norm controls.
- Four STRICT solves with nonzero pivots2^-17,2^-64,2^-128,2^-512.
- Rational zero-denominator rejection.

There are252 printed PASS rows because matrix rows group three checks and some
boundary assertions do not print individual rows. The terminal summary is420
in both modes. The aggregate validates both counts and exact output identity.
These are not a claim of generic complex-matrix or QR functionality in Hyper:
the checked public APIs are fixed-size exact matrices and general square
fraction-free solves. No full downstream suite was rerun in this no-change slice.

Source SHA256: `760f13e078c4cdeaf06d7002e0bf1b2c79abc32e4642d1fc191785354b18c03f`.
Output SHA256: `5c5a483cae419f795cec39766d419bb866b615a901955c3964eeab28d91a4a89`.
Debug binary: `16c2f4effc7f9181dedec4095dcb52e5c432a57b2f4372c02763d12497f72be4`.
Release binary: `d2c7eab9c68ec72704071825cacbd6ea39f75df4bbc1e06b74e3dbba4a96cc4c`.

## Donor-only benchmark design

MatrixBench.java and its first six process logs are retained as preliminary
evidence. That run completed216 observations,162 measured batches and170,136
exactly checked measured products. However, calibration preceded JIT warmup;
some later batches became too short for the approximately10ms process CPU
timer. Its bootstrap output must not be treated as precise timing evidence.

MatrixBenchStable.java is a separately versioned follow-up; it does not replace
the preliminary sources, class, logs or analysis. It first spends at least0.8s
process CPU warming each algorithm, calibrates a fixed batch to at least1s CPU,
discards three additional rotated warmup rounds, then measures nine rounds.
Every measured batch must be at least0.5s CPU or the entire process fails
qualification. A failed or incomplete stable run is never silently overwritten.

Each of six JVMs compares ordinary multiplication, Strassen cutoff1, and Strassen
cutoff8 at size4,8,16 and coefficient parameter32,256bits. Inputs and independent
triple-loop exact BigInteger expected outputs are prepared outside the timer.
Every resulting matrix is fully compared to the oracle inside the timed batch,
and a volatile sink consumes an entry hash. The coefficient parameter shifts
small signed integers and adds small offsets; it is not a claim that every
entry has precisely that bit length. Padded cases that failed correctness are
excluded from timing, not counted as fast results.

All JVM threads are pinned to CPU6, with common-pool parallelism1, -ea,
-Xms256m, -Xmx512m and1MiB stack. Algorithm position rotates each round. CPU and
wall durations are retained. Ratios are candidate/ordinary-product time per
verified result, paired by round; intervals are20,000 fixed-seed bootstrap
resamples of nine paired ratios. These are descriptive within-process
intervals, not independent-process reproducibility or universal crossover
claims. The host is shared, without an exclusive CPU reservation; no other
heavy process was started by this audit during measurement. Unrelated user
work must not be stopped or assumed absent.

The stable run completed all six processes successfully:216 observations,
162 measured batches and **2,435,328 exactly checked measured products**.
The shortest measured batch used1.04s CPU, exceeding the0.5s rejection floor;
no short samples or process failures were discarded. The analyzer checks source,
class, oracle and output hashes, fixed batch counts, warmup/calibration duration,
complete rotated rounds and all measured duration floors.

| Size | Coefficient parameter | Cutoff1 CPU ratio [95% descriptive interval] | Cutoff8 CPU ratio [interval] |
| ---: | ---: | ---: | ---: |
| 4 | 32 | 6.62 [6.46,6.67] | 0.977 [0.972,0.989] |
| 4 | 256 | 5.83 [5.72,5.87] | 1.010 [0.991,1.019] |
| 8 | 32 | 13.08 [12.92,13.17] | 1.006 [1.000,1.022] |
| 8 | 256 | 10.74 [10.59,10.84] | 1.000 [0.991,1.000] |
| 16 | 32 | 21.64 [21.24,22.62] | 2.46 [2.41,2.50] |
| 16 | 256 | 15.89 [15.89,16.00] | 2.12 [2.12,2.16] |

Wall ratios agree on the large regressions: cutoff1 medians5.82–21.84,
cutoff8 at size16 medians2.46 and2.13. For sizes4 and8, cutoff8 takes the same
ordinary multiplication base route, so its roughly1× results are controls,
not an extracted algorithmic speedup. In particular the small4/32 interval
excluding1 does **not** justify claiming a2.3% transferable improvement: these
are different wrapper call sites within one JVM and descriptive paired rounds,
not independent process replication or a change to Hyper's algorithm.

Full unrounded timings, ratios and intervals are in
`matrix-bench-stable-analysis.json`; raw logs and provenance are retained next
to it. The measured recursive Strassen paths are slower on every tested family.
Combined with the padding failures and existing Hyper architecture, this is
enough to reject transferring this implementation for these workloads. It is
not evidence that all Strassen implementations lose at all dimensions or
precisions, nor a cross-language Hyper/Ruffini speed comparison. Memory, RSS,
binary size and code size were not benchmarked because no candidate production
change survived the higher-priority correctness/performance screening.

## Retention and remaining work

Hyper already has exact fixed-size matrix kernels, shared scalar graphs,
fraction-free dense solves and certified/unknown pivot semantics. This donor's
cofactor expansion, null padding, builder aliasing and object-equality inversion
do not improve those contracts. No new production change is retained. The
possible borrowed-cache-rescale optimization from the earlier scalar review
remains unimplemented/unqualified; this matrix benchmark does not test it.

Remaining source includes polynomials, elliptic curves, class groups, demos and
one root file. Full Ruffini and ecosystem scope remain OPEN. The earlier
Hypercurve concurrency caveat in README.md remains: old downstream results
belong only to their original snapshot, not later user edits. Frozen prior
artifacts are not adjusted to accept changed source. No agents, commits,
pushes, donor edits or deletions are involved.

Final checkpoint validation passes through
`node exact-real-references/ruffini-qualification/verify-checkpoint.mjs`.
It now aggregates scalar/shared and matrix controls, both benchmark artifact
sets, dependencies/classes and the current 81/201-file Hyper snapshots. Current
Hyperreal HEAD remains bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c; all earlier
retained repair fingerprints match, and git diff --check passes. Hyperlattice,
Hyperlimit and Hypersolve are clean. All owned build/test/benchmark processes
are terminal; all six stable-benchmark stderr files are empty. Full scope is
ACTIVE/OPEN, not complete or blocked.
