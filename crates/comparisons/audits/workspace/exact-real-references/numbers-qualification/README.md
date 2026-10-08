# numbers / ERA numerical qualification checkpoint

Later checkpoint: Hyper atan repair is now RETAINED with full correctness,
downstream and measured tradeoff evidence in atan-README.md. This file preserves
the initial donor/baseline qualification. Its original open-atan wording below
is historical. The numbers target stays OPEN for adjacent inverse-function
repairs; no whole-target or ecosystem completion is claimed.

Subsequent checkpoint: the asin/atanh sample-domain repair is retained with
735/841 full test passes,160 repaired public requests,46674 MPFR boundary
checks/build and measured allocation/CPU evidence in neighbor-README.md.
The subsequent log-README.md checkpoint retains the ln_1p range repair after
741/848 full tests,1593downstream library passes and directed-oracle/benchmark
qualification. The newly confirmed192000bit asin/asinh coefficient defect
remains OPEN in series-limit-README.md; the older text below is historical.

Source reading is complete (19 files, 1,786 lines), but this target and the broad
audit remain OPEN. No new Hyper production edit was made at this checkpoint.

## Builds and controls

GHC 9.6.7:
`/tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc`.
Untouched ten-module build fails because Fixed's generalized derived Real and
RealFrac instances omit Epsilon constraints. `native-build.log` records it.
The separately generated compatibility copy only adds StandaloneDeriving and
the two required instance contexts; all method bodies are byte-identical.
Its SHA-256 is `40c5cccb110ef179fd3bc74bbb4173cbeec493cf0ecfca8376b2f095b69a4cd3`.
The independent CReal module/export-only visibility bridge preserves CRLF and
every numerical byte; SHA `3a0931a743fed1eb442d810317c12b113a389c1beacc576b7e0c125be20d2699`.
`inventory.mjs --bridge` regenerates and reverse-checks both. Original sources
are never edited. Scratch: `.audit-numbers.rjcbha` in the workspace root.

The initial compiled builds hit the sandbox's ccache write restriction. Final
builds explicitly ran with approval and CCACHE_DISABLE=1; O0, O2 and the upstream
test executable all exited 0. Initial failure logs are retained. The upstream
two-property suite passes 10,000 generated tests per property with fixed seed
271828 and existing local QuickCheck/test-framework dependencies. This is a
compatibility build, not an untouched modern-GHC native pass.

Run `run.mjs basic|elementary|fixed O0|O2` with subprocess permission. Every
request is isolated with 256 MiB GHC heap cap; elementary/fixed requests have
1.5-second wall caps. All 296 requests per family/build finish. Status-zero is
not a numerical pass. The original public CReal type is also exercised and 20
public/bridge formatter comparisons agree.

Per O0/O2 (all output bytes and numerical verdicts agree):

- 117,304 ordinary exact Rational/Integer approximation checks pass: guarded
  arithmetic, pointwise min/max, division away from zero, abs, and square-root
  enclosures. This finite grid is not a universal correctness proof.
- 21/64 public sign/equality/order checks fail for tiny nonzero inputs; 56/129
  properFraction integer components violate truncation toward zero.
- 297 polynomial derivative controls pass. Five atan derivative controls fail,
  plus one exact nonzero derivative falsely pruned through CReal's approximate
  Eq. Sixteen of 17 scaled-zero BigFloat equality controls fail.
- Symbolic substitution after x/x returns 1 at x=0, demonstrating erased domain;
  nonsingleton Interval self-equality and self-inequality both return False.
  These observations are not counted among the 117,837 basic assertions.
- 296 elementary outputs checked by 4096-bit outward MPFR: 12 failures, all in
  the negative atan t=-5 unreduced-series region. This is a term-budget error,
  not a pi-sign branch error. Other tested outputs pass, not all possible inputs.
- 296 continued-fraction outputs: 63 exceed the source functions' stated
  absolute epsilon accuracy. This includes wrong acos quadrant and accumulated
  argument-reduction/relative-stopping errors; not just final decimal rounding.

The oracle requires input rationals to be exactly represented at MPFR precision,
uses directed rounding and rejects only disjoint result/error intervals. All
43 exact oracle identities pass. Current Hyper passes 1,184 matching elementary/
history checks and 219 tiny-sign/truncation/history checks in debug and release.
Release Memcheck reports 0 errors; leak checking was disabled, so no leak-freedom
claim. Build and command logs are retained here.

## New Hyper finding: OPEN

Current Hyperreal commit `bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c`, with the already
qualified Plume Display edit still uncommitted. No other Hyperreal change.
`src/computable/node/roots_inverse_hyperbolic.rs` SHA-256:
`902de4335b14ab19bc9d176f2a90de203f6482913a366a15b0929c2285b8d64d`.

Cold `Computable::rational(100).sin().multiply(4).atan().approx(-32)` reaches
construction but fails a five-second cap. The negative argument has no retained
sign when atan_reduced runs. It samples approx(-4), but tests only `rough <= 8`
before choosing PrescaledAtan. A large negative argument therefore enters the
small-magnitude series; its squared ratio exceeds 1 and terms grow. A warm input
cache proves the sign first and avoids that path.

Frozen `.audit-numbers.rjcbha/numbers-oracle-before` SHA-256:
`a7f436d72b107a7ade1f4adab337af1baf14c3974485377eab682ec01ed69d74`.
`run-hyper-baseline.mjs` records 18 independent combinations of sin(100),
sin(-100), sin(4), scales 1/4/64, and cold/warm input. Four cold large-negative
requests hit two-second caps; all 14 others pass directed MPFR, including each
warm partner. Caps are not timing benchmarks or a measured speedup.

Next: preserve baseline binaries and design a signed range-dispatch regression
that fails promptly before evaluating a divergent kernel; implement the narrow
repair, prove its reduction/termination boundary, run full correctness and
downstream gates, and measure ordinary-path CPU/allocations/size before retention.
Also review the magnitude-only structural fast path's sign precondition. The
reference's CF/Euler series are not selected as replacement Hyper kernels.
No performance experiment has yet been claimed for numbers.

## Revalidation

`node exact-real-references/numbers-qualification/analyze.mjs` verifies pinned
source/bridge inputs, complete run counts, O0/O2 consistency, numerical summaries,
oracle identities, Hyper controls, frozen baseline identity and cap classification.
It does not prove that source was read or that arbitrary real equality terminates.
