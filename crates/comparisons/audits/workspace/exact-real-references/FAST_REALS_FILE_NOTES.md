# haskell-fast-reals per-file audit

Snapshot: `8fa09b2457d7099c75b7db62895b5c8f3391d9e3`, 2017-04-19.
All 23 tracked text files / 2,954 physical lines have been read. The remaining
tracked path is an ELF benchmark artifact, identified without execution.
`FAST_REALS_FILE_INVENTORY.tsv` and `FAST_REALS_READ_COVERAGE.tsv` reconcile
every path, full read range, byte length and SHA-256. A truncated lecture-note
read was repaired by explicitly rereading lines 570–860 before crediting it.

Source and targeted Hyper transfer comparison are complete. This does NOT mean
the original full package has passed a native build: its declared GHC 7.8-era
base/integer-gmp dependencies cannot use the installed GHC 9.6.7. Unchanged
backend-independent modules were compiled and tested directly. Stock-MPFR
policy reconstructions are explicitly separate from the unavailable historical
GHC/C-- ABI. No donor implementation or new production change is retained.

| File | Comparison disposition |
| --- | --- |
| `.gitignore` | Local output exclusion; no numerical idea. |
| `.project` | Eclipse/Haskell IDE project description; no scalar transfer. |
| `LICENSE` | BSD-2-Clause text read. The historical MPFR dependency has its own LGPL-3.0 text; neither implementation was copied into Hyper. |
| `README.md` | Explicit GHC 7.8.4/custom MPFR requirement and warning about newer compiler internals. Historical speed figures need new accuracy-matched measurements. |
| `Setup.hs` | Standard Cabal entry point, no scalar algorithm. |
| `haskell-fast-reals.cabal` | Exposed completion, interval, MPFR and Lipschitz modules; base <4.8; HTF tests and two benchmarks. Pure Dyadic/HMPFR alternatives are not exposed here. Listed README.txt is absent. |
| `notes/Searchable.hs` | Finite nonempty selection and searchable-space monad; empty input is partial. Decidable predicates on searchable objects must not become a promise of decidable equality for arbitrary real procedures. Old Monad examples omit modern superclasses. |
| `notes/monads.lhs` | Abort, nondeterministic-choice and state-monad tutorial. List append inside the choice accumulation has avoidable growth. No new certified scalar method. |
| `notes/notes.tex` | Domain/realisability, compact/overt/searchable spaces and semidecision semantics; monad appendix duplicates the tutorial. Some formulas are explicitly unfinished and external bibliography files are absent. Useful distinction between mathematical partiality and total machine APIs; Hyper already exposes bounded unknown outcomes. |
| `notes/test_lim.hs` | Stale imports/API, Babylonian bracketing, atan/Machin and factorial-series examples. Not a working implementation of the commented-out public limit constructor; guard/tail ideas already represented in Hyper. |
| `src/Data/Approximate/ApproximateField.hs` | Directed approximate-field contract, magnitude/precision metadata and midpoint. The default appMul2 multiplies by the exponent integer instead of its power of two; 10/10 independent default-method checks fail. Active backends override it. The generic midpoint reserves one significand bit, which is not a general proof of the comment's exact midpoint claim for widely separated exponents. |
| `src/Data/Approximate/Floating/Dyadic.hs` | Integer mantissa/exponent with exact operations followed by normalization; finite reciprocal/division round via floor/upward increment. Negative normalization rounds inward, confirmed independently in addition, subtraction, multiplication and scaling. Even 128-bit interval division composition can exclude the answer. abs(-infinity) remains negative infinity. Rational conversion is undefined. Reject normalization transfer. |
| `src/Data/Approximate/Floating/HMPFR.hs` | Alternate unexposed backend; rational conversion and abs incomplete, unordered predicate always false, exponent negated and traced. No transfer of these metadata/NaN contracts. |
| `src/Data/Approximate/Floating/MPFR.hs` | Fixed 64-bit downward appFromInteger becomes a frozen point through Interval/Reals, losing large integer exactness. appAbs is undefined; Interval's own abs bypasses it but rounds its upper candidate downward. appGetExp negates MPFR's exponent, violating the declared field bound; unordered is always false. Binary shift shares mantissa through the historical binding. Hyper already has exact integer leaves, checked dyadic offsets and valid magnitude facts. |
| `src/Data/Approximate/Interval.hs` | Generalized/Kaucher two-endpoint arithmetic and partial separation order. Proper small integer +,-,* enclosures pass independent controls; dyadic inverse-then-multiply exposes normalization defects. abs always chooses lower=0, so nonzero point inputs do not converge, and finite-precision negative endpoint negation can make the upper bound inward. Width uses the backend exponent convention, so its meaning is inconsistent across backends; zero's maxBound is not alone a demonstrated MPFR-width bug. Center/radius storage is a comment-level suggestion, not an implemented improvement over Hyper's one-integer approximation cache. |
| `src/Data/Reals/Lipschitz.hs` | Experimental Float-valued Arrow metadata. first/second omit the untouched identity coordinate's unit bound; both constant-function controls report 0 despite an output distance of 1. These are not sound norm certificates. Hyper derivative/predicate decisions require exact evidence, not unchecked sensitivity hints. |
| `src/Data/Reals/Reals.hs` | RealNum is a precision-indexed list of intervals; integer points are frozen, rational endpoints staged. Eq/Ord explicitly partial, signum raises, general Cauchy limit and target-accuracy APIs are commented out. Compact forall uses interval certificates and midpoint counterexamples but appends children to a list queue. Specialized Hyper Bernstein/root/predicate layers already separate certified results from depth/refinement limits; the donor is not a drop-in general function-space API. |
| `src/Data/Reals/Space.hs` | Sierpinski lower/upper Boolean approximations, stagewise logical operations, linear precision force loop and potentially diverging Show. Eight positive/negative finite-semidecision controls pass. Hyper already distinguishes exact certificates and bounded nondecisions; no unbounded Boolean forcing transfer. |
| `src/Data/Reals/Staged.hs` | Reader-function and memoized-list completions. List limit builds from stages [1..], but approximate indexes from zero: p requests p+1, and an identity lift requests p+2 from its source. Each query walks the retained list, and high requests allocate its whole spine. Forty-eight pinned process-CPU/live-storage measurements reject this as a Hyper cache replacement. |
| `testsuite/benchmarks/bench-float.hs` | Fixed-precision rounded arithmetic and historical timing comments. Repeated constant closures/WHNF and discarded results can primarily measure sharing or incomplete evaluation; not achieved-accuracy exact-real comparison. |
| `testsuite/benchmarks/bench-reals.hs` | Shared Rump-polynomial computations, stage 5 and WHNF Interval demand. Endpoint evaluation and achieved accuracy are not independently established. Reject quoted speed ratios as transfer evidence. |
| `testsuite/benchmarks/timings-mpfr` | 12,328-byte ELF, SHA-256 a216b23573dc8b9200f5018164f9ea8291d72bc5572b6751a9556e5edd647851. Identified, not executed; adjacent C source was read instead. |
| `testsuite/benchmarks/timings-mpfr.c` | Ordinary in-place MPFR loops with fixed precision and time-based iteration growth. This excludes Haskell allocation and has no independent exact-real error target. LGPL notice is separate from the main package license. No historical binary or speed claim adopted. |
| `testsuite/tests/HTFTestReals.hs` | Only two broad arithmetic/compact-quantifier tests, no independent large-integer, directed-negative-rounding, absolute-value refinement or cache-history oracle. Full original HTF execution remains unqualified on this host. |

## Independent numerical evidence

Probe directory: `/tmp/fast-reals-audit.bwgyTf`. GHC 9.6.7 compiles the original
ApproximateField, Dyadic, Interval, Staged, Space and Lipschitz modules unchanged.

- `PureProbe.hs`, `/tmp/fast-reals-pure-probe.log`: 348,720 exact-Rational
  scalar directional checks. Addition/subtraction each fail 26,826/86,490;
  multiplication fails 25,596/86,490; scaling fails 855/4,650. Direct division
  passes 83,700 and reciprocal passes 900. These are directional-contract
  failures, not claims that every operation or precision is bad.
- Proper small-endpoint interval addition/subtraction/multiplication each pass
  2,025 checks. Division fails 228/900 at 128 bits because its multiplication
  stage inherits the signed normalization defect. Forty-five interval-abs
  enclosure checks pass on exactly negatable dyadic endpoints, but 32/36
  point-refinement checks fail: abs([-4,-4]) remains [0,4] at every tested stage.
  The abs(-infinity), default scaling and stage-index checks fail as above.
- `mpfr-policy-probe.c`, `/tmp/fast-reals-mpfr-policy.log`: stock MPFR
  reconstruction of the exact calls made by the source policy. 36/90 frozen
  integer points miss their exact input, while all 90 underlying directed
  MPFR conversion checks pass. For example 2^64+1 becomes 2^64, not an exact
  point. The interval abs policy loses the upper endpoint in 580/1,542 checks;
  at 2-bit significand precision abs(-5) is bounded above by 4. The declared
  exponent inequality fails 256/257 powers-of-two checks. This is NOT a native
  test of the historical GHC binding, and its output is not labeled as such.
- `SemanticProbe.hs`, `/tmp/fast-reals-semantic-probe.log`: both lifted
  Lipschitz bounds fail; eight finite semidecision-force controls pass.
- Baseline Hyperreal 21e76ea, frozen client `hyper-fast-reals-controls`:
  810 large-integer approximation/history, 2,150 exact Real::abs, 57,680 binary
  scaling/history and 19,065 rational arithmetic/history checks all pass.
  Exact Rational oracles are separate from the Computable kernels being tested.
  Source: `/tmp/haskell-creal-audit.rafb8h/hyper-cache/src/bin/fast_reals.rs`;
  output: `/tmp/fast-reals-hyper-controls.log`. No production test or code was
  added merely to duplicate these already-covered contracts.

## Cache measurements and transfer disposition

`StageBench.hs`, `run-stage-bench.sh`, `/tmp/fast-reals-stage-bench.tsv`:
48 observations, four samples per function/list and six precision points,
alternating order, CPU 6, process CPU time, results forced/checksummed,
full-laziness/CSE disabled. These isolate completion lookup mechanics, not
arbitrary-accuracy numerical throughput. Query allocation accounting is flushed
after the timed loop; setup and live storage are reported separately.

| Requested stage | Function median ns/query | List median ns/query | List retained live bytes |
| --- | ---: | ---: | ---: |
| 64 | 14.58 | 77.76 | 7,696 |
| 256 | 14.70 | 279.07 | 18,480 |
| 1,024 | 14.35 | 1,297.44 | 61,488 |
| 4,096 | 14.36 | 4,979.85 | 233,520 |
| 16,384 | 14.69 | 22,859.33 | 921,648 |
| 65,536 | 14.65 | 103,877.55 | 3,674,160 |

Function live storage stays about 3.9 KB. The list is retaining its index spine
even though only the final neighboring stage results are demanded. Warm query
cost and memory growth materially reject this policy for Hyper, whose existing
single finest cache can round to coarser requests without traversing every
intermediate precision. No candidate survived that would warrant a production
patch, binary-size comparison or repeat full cross-crate gate.

Other ideas are already represented (dyadic shifts, compact approximations,
typed partial decisions) or belong to a future certified function-space layer
(general compact search). The unimplemented center/radius comment does not
justify replacing Hyper's already compact integer/error contract. No worthwhile
new exactness, completeness, performance, memory, binary-size or code-size
change was selected from this target.

## Required historical backend slice

`comius/haskell-mpfr` is pinned at
`b5d91ca5d3ff3f0455860dcd3a38ebeee599a4e9` (2017-05-21).
It has 503 tracked paths, including 478 under the vendored MPFR tree. Plain
MPFR is outside the user's main exact-real inventory. This is a deliberately
scoped dependency review, NOT an assertion that the whole vendored tree or all
25 nonvendor files were read. `FAST_REALS_BACKEND_READ_SLICE.tsv` records the
19 files / 3,039 lines actually read: manifest/build/license/readme, cpphs,
the low-level types and operations, generated-operation headers, C-- wrappers,
derived constants and three allocation/custom-interface files.

The read slice explains the obsolete integer-GMP constructors, GHC primitive
FFI, sign/precision packing, shared byte-array mantissas, exponent-only binary
shifts and custom MPFR allocation setup. The build combines archive objects and
depends on old Cabal/GHC interfaces. Static concerns include incomplete special
value handling, unchecked primitive exponent arithmetic, mutable aliasing in
some nominally pure wrappers and inconsistent multi-result signatures. These
were not executed as unsafe ABI probes and are not needed to demonstrate the
higher-level scalar contract failures. Hyper's existing Arc ownership, exact
integers, checked scale paths and short synchronized cache publication do not
benefit from importing this runtime coupling.

`/tmp/fast-reals-native-build.log` records the actual build-plan failure on
base 4.18.3.0 / integer-gmp 1.1 versus the declared base <4.8 /
integer-gmp <0.6. No broad constraint relaxation or rewritten backend is
misrepresented as a native pass. All original donor worktrees remain pristine.
