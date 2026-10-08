# numbers / Data.Number.CReal audit

Status: all 19 pinned repository files independently read; numerical qualification
and Hyper transfer comparison are complete for the pinned scalar files. Signed/exact-domain atan dispatch repair,
asin/atanh sample-domain repair, checked ln_1p, and the high-precision asin/asinh
coefficient-width repair are retained after their recorded gates; no additional
Numbers implementation change is pending.

Current numerical checkpoint: 117304 ordinary approximation controls pass per
O0/O2, while decision/truncation/derivative/zero-canonicalization controls confirm
99 basic failures. MPFR confirms12/296 CReal elementary failures and63/296
FixedFunctions epsilon failures per build. The independent Hyper atan comparison
now retains a qualified range-dispatch repair, including estimated-magnitude and
signless-large-certificate cases. See numbers-qualification/atan-README.md for
728/833 full-gate passes,6456 public controls/build,1589 downstream passes and
explicit performance costs (not a speedup claim). The subsequent asin/atanh
kernel-domain repair now passes all160 frozen failure requests,46674 directed
MPFR boundary checks/build,735/841 full tests,696 current downstream library
tests and Memcheck (zero errors). Its270 benchmark observations show unchanged
control allocations and broadly near-parity CPU; no universal speedup claim.
See numbers-qualification/neighbor-README.md. The subsequent ln_1p repair now
passes154original log-boundary requests plus160current inverse cross-checks,
4186newMPFRchecks/build,741/848 full tests and1593downstream library tests
(Hypercurve897plus1ignored), with zeroMemcheckerrors. Moderate asinh/acosh
controls improve1.39x/1.18x with fewer allocations; ordinary controls are near
parity. See numbers-qualification/log-README.md for snapshots and limitations.
The previously confirmed 192000-bit asin(1/16)/asinh(1/16) i32 coefficient
overflow is repaired in all four rational/generic recurrences. Independent
184000/192000/256000/524288-bit probes, directed-MPFR checks, full tests,
allocation benchmarks, and Memcheck are recorded in
numbers-qualification/coefficient-README.md; no further coefficient repair is pending.

## Provenance and scope

- Actual implementation: https://github.com/jwiegley/numbers at
  `0ab9e067e10a2e097239f73bdcbbffeae09790b0` (2023-08-18).
- The supplied https://github.com/haskellcats/haskell-numbers is a one-file,
  54-line numeric-type catalogue, independently read at
  `366f5c8f4fa02243241060469b04ed4119f2f154`. It is not the implementation.
- https://hackage.haskell.org/package/numbers identifies the actual repository.
  HEAD retains version 3000.2.0.2 but differs from its release tag
  `3f3a03ed9e8a11465f380b893a317388e11462e5`: Abstract, symbolic sum collection,
  Fixed's epsilon-method export and project files changed. Do not conflate HEAD
  with the published archive. The tag-to-HEAD scalar diff was read separately.
- CReal explicitly identifies itself as David Lester's ERA v1.0 (2000–2001).
  This is a surviving Manchester ERA lineage lead, not proof of identity with
  the historical competition artifact.
- No gitlinks, symlinks or AGENTS.md in these pinned source inventories.
  Full ranges/hashes are in NUMBERS_READ_COVERAGE.tsv. Truncated output was
  repeated before read credit. No original source modified.
- LICENSE/cabal say BSD3, but FixedFunctions retains a GPL copyright header
  from Numeric Quest. No donor code will be copied; do not assume a resolved
  license grant for that file.

## File-by-file findings (static until executable qualification)

| File | Findings and transfer questions |
| --- | --- |
| CReal.hs | Precision-indexed Integer closures, guarded arithmetic, magnitude-dependent multiplication, pointwise min/max, no explicit best-approximation cache. Eq/Ord/signum instead use fixed 40-digit demand (137 bits), contrary to the comparison documentation. properFraction uses a rounded approximation, not certified truncation. atan t=-5 falls through to the unreduced series; its term-budget failure is confirmed, not a sign-branch error. The range comparison produced the retained Hyper atan dispatcher fix. Finite-decimal display corroborates the already retained Plume fix. Partial Real/RealFloat methods and machine-Int precision arithmetic are explicit limitations. |
| Dif.hs | Lazy derivative tower and constant-zero pruning; atan derivative has p*p-1 instead of p*p+1. A base with approximate Eq can falsely prune nonzero constants. Existing ireal/Few Digits finite-jet comparisons are the Hyper baseline. |
| Fixed.hs | Rational bounded-denominator approximation at fixed epsilon, not a refinable real object. Rank-2 dynamic precision is type-scoped. Invalid epsilon can prevent termination; RealFloat is intentionally partial. |
| FixedFunctions.hs | Generalized rational continued fractions, Taylor-to-CF and argument reductions. Adjacent-convergent relative stopping is not a generic absolute enclosure proof. acos for negative interior inputs has the wrong quadrant. Magnitude/range-reduction error budgets need independent qualification. Repeated indexed list traversal and rational growth are performance concerns, not measured conclusions. |
| BigFloat.hs | Fixed rational mantissa plus Integer decimal exponent; normalization repeatedly scales. Derived structural Eq assumes canonical values, but scaleFloat zero bypasses canonicalization. Elementary tolerance scales with input magnitude, not output sensitivity. |
| Interval.hs | Generic endpoint arithmetic without outward floating rounding. Bool equality/inequality mean certain singleton equality/disjointness, not complementary value comparisons; Ord is partial. No automatic transfer into Hyper certificates. |
| Symbolic.hs | Debug expression tree, structural equality, constant folding and quadratic pair collection. x/x and nested division erase zero-domain restrictions; power reassociation ignores real branch conditions. No sound relation-discovery improvement selected. |
| Natural.hs | Lazy unary naturals including infinity; arithmetic strictness and productive prefixes are deliberate. Not a compact scalar carrier or replacement for bounded certificate counts. |
| Abstract.hs | Deliberate one-element numerical abstraction; no represented exact value. |
| Vectorspace.hs | Minimal scalar/vector functional dependency; no implementation or performance mechanism. |
| Test/Data/Number/BigFloat.hs; TestSuite.hs | Only two random Double conversion/ordering properties. No CReal approximation, decision, derivative, domain or symbolic tests. |
| .gitignore; .travis.yml; default.nix | Old Nix/GHC build matrix and pinned package environment, no numerical proof/benchmark. |
| LICENSE; README.md; Setup.hs; numbers.cabal | Package metadata, license discrepancy above, base-only library and optional test dependencies. |

## Qualification plan

Build untouched modules first. A separately hashed, module-name/export-only
CReal bridge may expose the internal approximation function for exact integer
error checks; it must preserve every numerical byte. Test public behavior too.
Use exact Rational/Integer or outward MPFR oracles, optimization-level repeats,
bounded isolated requests for partial operations, and verified controls before
benchmarking. Performance experiments and all Hyper comparisons remain open.
