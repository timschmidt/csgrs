# Constructive-real-to-Hyper line audit

This is the persistent workspace-root ledger for the active audit of
`ExactCalculator`, `crcalc-boehm`, and `crcalc`. A file is marked **complete**
only after every line in the stated range has been read and its transferable
ideas have been checked against the current Hyper scalar and performance
architecture.

## Objective and retention order

Audit the three source trees file by file and line by line; inspect `hyperreal`,
`hyperlattice`, `hyperlimit`, `hypertri`, `hypercurve`, `hypersolve`, and other
Hyper crates as needed; and retain only rigorously tested changes that improve,
in order: exactness, completeness, performance, memory use, binary size, or
source size. Exact behavior and existing public contracts are mandatory gates
for every lower-priority optimization.

## Safety and baseline

- Audit started 2026-09-03 in `/home/tim/Documents/GitHub/workspace`.
- `ExactCalculator`, `crcalc-boehm`, `crcalc`, and `hypersolve` began clean.
- `hyperreal`, `hyperlattice`, `hyperlimit`, `hypertri`, and `hypercurve` began
  with pre-existing uncommitted changes. Those changes are user-owned. Audit
  work must be isolated by file/range and must not erase or rewrite them.
- The user reported that `hypercurve` is under concurrent editing and need not
  gate this audit. Its architecture was inspected, but its long aggregate test
  run was stopped on request and its concurrent failures are not attributed to
  these scalar changes.
- No candidate is retained from inspection alone. Each candidate needs a
  semantic comparison, focused regression coverage, and proportionate
  performance/memory/size measurements.

## Progress summary

| Area | Status | Audited physical lines | Notes |
| --- | --- | ---: | --- |
| Hyper scalar/performance architecture map | complete | — | Scalar representation, predicate ownership, downstream policy, certification, cache, and benchmark paths mapped through the relevant crates. |
| `crcalc-boehm` implementation/docs/tests | complete | 5,491 | Every tracked source, test, build, documentation, launcher, and license line audited. |
| `crcalc` implementation/docs/tests | complete | 3,724 | Every tracked source, test, build, metadata, documentation, and license line audited. |
| `ExactCalculator` exact-number implementation | complete | 1,849 | `BoundedRational` and `UnifiedReal` read in full. |
| `ExactCalculator` evaluator/expression/UI support | complete | 11,582 | All 21 Java files complete; non-code resources were audited separately below. |
| `ExactCalculator` complete tracked tree | complete | 23,827 | All 194 tracked files audited; 157 XML files parsed, 83 locale sets checked, and 5 PNG assets validated. |
| Candidate experiments | complete | — | Three focused changes retained; every other transfer candidate was rejected or found already subsumed. |
| Final cross-crate validation | complete | — | Authoritative Hyperreal, Hyperlattice, Hyperlimit, Hypertri, and Hypersolve gates pass; Hypercurve explicitly excluded as concurrently edited. |

## Architecture findings

- The dependency direction is `hyperreal` -> `hyperlattice` -> `hyperlimit`,
  with `hypertri`, `hypersolve`, and `hypercurve` consuming those certified
  scalar and limit layers. Exact branch ownership belongs in `hyperlimit` and
  its consumers, not in heuristic scalar comparisons.
- `Rational` uses shared immutable storage, lazy reduction, compact retained
  facts, and specialized word/dyadic paths. `Real` preserves a rational scale
  and symbolic class before falling back to a shared lazy `Computable` graph.
- `Computable` publishes monotone precision caches without publishing aborted
  work. Hyper already has bounded binary64 filters, algebraic zero separation,
  exact rational-angle tables, constructor folding, and operation-specific
  approximation nodes. Candidate transfers must therefore beat an already
  substantially more complete architecture than either Java implementation.
- The governing dispatch order is exact rational structure, symbolic facts,
  smaller exact rewrites, shared computable construction, and only then
  requested-precision approximation. This is also the retention order for
  proposed changes.
- `hyperlattice` retains exact homogeneous/affine carriers and object facts,
  but uses those facts for scheduling rather than as substitutes for scalar
  proof. Delaying division keeps projective operations in cheaper exact rings
  and makes unknown denominators explicit.
- `hyperlimit` owns the common predicate cascade: exact rational/common-scale
  paths, structural and interval filters, certified `Real` refinement, then an
  explicit `Unknown` or policy-authorized terminal result. Its public outcome
  and certainty types prevent a bounded approximation from masquerading as an
  exact decision.
- `hypertri` funnels irreversible orientation, incidence, comparison, and
  in-circle choices through one operation-local Hyperlimit evaluator. It
  propagates `Unknown` as an error under strict policy and records any terminal
  approximation in the operation result.
- `hypersolve` separates numerical proposal generation from exact replay. It
  already supplies symbolic derivatives, exact polynomial packages,
  Sturm/Bernstein root isolation, exact algebraic-root carriers, and interval
  and Krawczyk certificates. Unsupported identities and exhausted predicates
  remain explicit rather than falling through to floats.
- `hypercurve` consumes those layers through `CurveContext`, policy-aware
  caches, and decided/uncertain result types. Lossy tolerances exist only in a
  scoped preview adapter; native curve topology remains exact and carries
  blocker/completeness evidence. This is why heuristic scalar selection or
  integer promises would weaken, rather than extend, the stack.

## File-by-file ledger

### `crcalc-boehm`

- Audited snapshot: standalone 18-file historical tree (not a Git worktree).
- Complete (entire 5,491-line tree): `CR.java` (1,336), `UnaryCRFunction.java` (657),
  `StringFloatRep.java` (73), `AbortedError.java` (50),
  `PrecisionOverflowError.java` (52), `TestCR.java` (145), `CRCalc.java`
  (1,933), `rpn_calc.java` (264), `Makefile` (61), `instrs.html` (305), and
  `Constructive Real Calculator and Library Implementation Notes.html` (112).
- Complete: `CRCalc.html`, all four `RunCRCalc*.html` launchers,
  `impl.html`, and `COPYRIGHT.txt`.
- The old/new Java revisions were compared with full whitespace-insensitive
  diffs so that deleted as well as added behavior was covered.

### `crcalc`

- Audited snapshot: Git commit `4f398d699c9d498deb45520ea7ebb020627bc640`
  (17 tracked files).
- Complete (entire 3,724-line tree): `CR.java` (1,652), `UnaryCRFunction.java` (667),
  `StringFloatRep.java` (73), `CRTest.java` (163), `ConversionTest.java` (85),
  and `SlowCRTest.java` (248).
- Complete: `impl.html`, `Android.bp`, `build.gradle`, test build/manifest/readme,
  ownership/metadata markers, and the byte-identical `COPYRIGHT.txt`/`LICENSE` pair.

### `ExactCalculator`

- Audited snapshot: Git commit `7265aa3a922a7e7c0389dae4b26cca562aa777a6`
  (194 tracked files).
- Complete: all 21 Java files (11,582 lines), including `BoundedRational.java`
  (564), `UnifiedReal.java` (1,285), `CalculatorExpr.java` (1,118),
  `Evaluator.java` (1,963), and the complete evaluation/display/database/UI
  support layer. Both overview documents are complete (322 lines).
- Per-file Java coverage: `AlertDialogFragment.java` (134),
  `AlignedTextView.java` (76), `BoundedRational.java` (564),
  `Calculator.java` (1,538), `CalculatorDisplay.java` (202),
  `CalculatorExpr.java` (1,118), `CalculatorFormula.java` (392),
  `CalculatorPadViewPager.java` (252), `CalculatorResult.java` (1,180),
  `CalculatorScrollView.java` (83), `DragController.java` (482),
  `DragLayout.java` (357), `Evaluator.java` (1,963),
  `ExpressionDB.java` (619), `HistoryAdapter.java` (221),
  `HistoryFragment.java` (243), `HistoryItem.java` (66),
  `KeyMaps.java` (683), `Licenses.java` (30), `StringUtils.java` (94), and
  `UnifiedReal.java` (1,285).
- Complete: Android build metadata, all base and configuration-specific
  layouts/styles/resources, notices, license asset, and `.gitignore`.
- All 157 resource XML files are well formed. Each of 83 locale files has
  exactly the same 67 translated keys in the same order, without duplicates,
  empty values, unexpected child markup, or an invalid visible decimal
  separator. The five launcher PNGs decode successfully at the expected
  48/72/96/144/192-pixel density sequence.

## Candidate transfer ledger

| Candidate | Priority | Source | Hyper target | Status | Evidence / disposition |
| --- | --- | --- | --- | --- | --- |
| Balanced range products and pre-cancelled factorial ratios | performance, memory | `UnifiedReal.genFactorial` | `Real::factorial_biguint`, gamma/beta exact paths | retained | A 512-factor-leaf product tree with word-sized leaf batching and a direct `n <= 20` path improves both small and large gamma. Half-integer gamma, recurrence inverses, and integer beta now cancel factorial ratios before allocation. Sequential-reference regression coverage spans thresholds, signs, and asymmetric beta inputs. |
| Exact logarithm/root round trips | exactness, completeness, performance | `UnifiedReal` named-log and half-log identities | `Real::ln`, `Real::exp` | retained | `ln(n^k sqrt(n))`, rational scales on retained `Pow2`/`Pow10`, and `exp(q ln n)` now preserve exact algebraic structure. This closes the only material gap in ExactCalculator's named-log handling while extending it to every supported rational exponent. |
| Irrational-boundary integer extraction | completeness | `UnifiedReal.toStringTruncated` | `Real` floor/truncation/integer predicates | rejected as a rounding change | `floor_certified` must prove the side of the rational boundary before returning, and `ceil_certified` rechecks that same cached difference. Irrationality proves disequality, not ordering, so it cannot make the bounded integer operation complete without changing its explicit refinement policy. The useful equality-only subset was retained separately. |
| Certified irrational-vs-rational inequality | completeness | `UnifiedReal.definitelyIrrational` / exact comparison | `Real::certified_eq_until` and public facts | retained | `Real::definitely_irrational` conservatively recognizes provable classes and lets certified equality reject an exact rational without approximation. It intentionally excludes unresolved products such as `pi*e`, log products, mixed pi/e/sqrt products, and opaque computables. A 4,096-bit rational approximation of pi is separated at refinement budget zero in both operand orders. |
| Capped integer refinement (`assumeInt`) | performance | `CR.assumeInt`, calculator fact propagation | `Real` exact/fact layer | rejected | Hyper retains exact rationals and explicit symbolic facts rather than accepting a caller promise. A false promise in Java changes refinement behavior; importing that contract would be unsound at Hyperlimit's topology boundary, and no repeated over-refinement workload survived the existing fact/cache paths. |
| Continuous conditional/select node | completeness | `CR.select` | `Computable`/public `Real` | rejected | The operation is constructive only when its branches agree at an undecidable zero selector. Hyper already represents continuous `abs` safely and centralizes order-dependent `min`/`max`/`clamp` choices in Hyperlimit. No consumer demand justified a new graph variant, serde surface, and binary cost. |
| Certified monotone inverse/derivative combinators | completeness | `UnaryCRFunction` | `hyperreal` vs `hypersolve` ownership | rejected | Hyperreal already has specialized inverse kernels, while Hypersolve owns symbolic differentiation, exact root isolation, interval/Krawczyk uniqueness proofs, and explicit undecided reports. A closure-like generic scalar graph would duplicate that proof boundary and complicate serialization without adding a demonstrated capability. |
| Previous-precision seeding and bucketed refinement | performance | `CR.sqrt`, `slow_CR` | approximation caches | rejected as subsumed | Hyper's shared monotone `Computable` cache already reuses any finer published result; operation-specific kernels also retain their own safe schedules. Porting Java's object-local seeding would duplicate state and did not expose a missed hot path. |
| Exact `pi/12` trig table | exactness/completeness | `UnifiedReal` | `Real` rational-angle tables | rejected as subsumed | Despite its name, ExactCalculator's table resolves only the same denominators 2, 3, 4, and 6 already covered by Hyper. Hyper additionally retains canonical `SinPi`/`TanPi` certificates for non-tabulated rational turns. |
| Named logarithm rewrites | exactness/completeness | `UnifiedReal` | `Real` log classes | partially retained | Hyper already recognizes powers of 2, 3, 5, 6, 7, and 10 and has broader rational-factor log algebra. The missing square-root/rational-exponent connections were retained in the exact logarithm/root change above. |
| Gauss-Legendre pi and calculator display rounding | performance / presentation | `CR.PI`, `Evaluator`/display layer | `Computable::pi`, formatting | rejected | Hyper's Chudnovsky-style pi kernel and precision cache are already the stronger scalar implementation. Calculator digit-window, truncation, and `9`-to-`0` display policies are UI semantics, not transferable exact-number behavior. |

## Validation and benchmark log

| Date | Command / measurement | Result | Interpretation |
| --- | --- | --- | --- |
| 2026-09-03 | Initial repository-state inventory | complete | Dirty Hyper trees recorded before edits. |
| 2026-09-03 | `javac -Xlint:all` over all `crcalc` core Java sources | pass | 21 expected `serialVersionUID` warnings only. |
| 2026-09-03 | `javac -Xlint:all` over `crcalc-boehm` core plus `TestCR`; run `TestCR` | pass | 19 expected `serialVersionUID` warnings; historical standalone test exited 0. |
| 2026-09-03 | JUnit `CRTest ConversionTest SlowCRTest` with system JUnit/Hamcrest | pass | 5 test methods passed; slow suite completed in 0.684 s. Initial run without Hamcrest exposed and corrected a local harness classpath omission, not a source failure. |
| 2026-09-03 | `cargo test --all-features` (`hyperreal`, dirty-tree baseline) | pass | 690 library tests, all integration suites, and 24 doctests passed with no failures. |
| 2026-09-03 | Focused irrational-class and 4,096-bit near-pi equality tests | pass | Provable classes certify irrationality; deliberately unresolved/opaque classes do not; rational-vs-pi equality is rejected structurally at budget zero. |
| 2026-09-03 | Sequential-reference gamma/beta product corpus | pass | Exact agreement across product thresholds through 10,000!, all supported signed half-integers from -199/2 through 201/2, and 36 asymmetric integer-beta pairs. |
| 2026-09-03 | Criterion A/B, `real_normal_scientific_substrate/gamma_integer` (`gamma(8)`) | retained 103.32 ns vs baseline 454.28 ns | 77.6% lower median/point estimate on the existing common-case benchmark. |
| 2026-09-03 | Standalone release medians, 31 samples (large exact forms) | retained | `gamma(10001)`: 0.51 ms vs 4.91 ms; `gamma(20001/2)`: 0.65-0.71 ms vs 48.5-52.2 ms; `gamma(-20001/2)`: 0.84-0.91 ms vs 281-292 ms; `beta(5000,7000)`: 0.56-0.61 ms vs 11.4-11.9 ms; half-integer `ln_beta`: 55.8-58.4 ms vs 124.9-129.5 ms. |
| 2026-09-03 | Valgrind Massif A/B, `gamma(-20001/2)`, stacks included | retained 144,248 B vs baseline 338,744 B peak | 57.4% lower total peak; useful heap fell from 296,412 B to 114,356 B. |
| 2026-09-03 | Optimized `hyperreal` CLI size A/B (`--release --features simple`) | retained 2,291,888 B vs baseline 2,291,184 B | 704-byte / 0.031% increase accepted for the exact large-integer speed and memory gains. |
| 2026-09-03 | Focused exact log/root round-trip regression | pass | Pure and rationally scaled square roots, signed half-log exponentials, and base-2/base-10 rational powers retain exact identities. |
| 2026-09-03 | Standalone release medians, 31 samples (log/root A/B) | retained | `ln(sqrt(2))`: 224 ns vs 1,040 ns; `ln(sqrt(8))`: 315 ns vs 5,195 ns; `ln(2^(1/3))`: 306-318 ns vs 956 ns. `exp(ln(2)/2)` is about 149 ns vs 134 ns, an accepted small cost because it now returns the exact structural square root rather than only a generic computable. |
| 2026-09-03 | Final `cargo test --all-features` (`hyperreal`) | pass | 694 library tests, every integration suite, and 24 doctests pass. The GMP public-API inventory and exhaustive representation/serde matrices include the new certificate query. |
| 2026-09-03 | `cargo test --no-default-features --lib` (`hyperreal`) | pass | 614 library tests pass without optional features. |
| 2026-09-03 | `cargo test --all-features` in `hyperlattice`, `hyperlimit`, `hypertri`, and `hypersolve` | pass | Every authoritative downstream scalar, predicate, topology, algebraic-root, and certification suite passes against the modified Hyperreal. |
| 2026-09-03 | `cargo test --all-features` (`hypercurve`) | excluded by user | The concurrently edited long suite was stopped on request after broad partial coverage; failures in pre-existing dirty Hypercurve files are not an audit gate. |
| 2026-09-03 | `cargo fmt --all -- --check`; `git diff --check`; Clippy all targets/features with warnings denied; rustdoc all features with warnings denied | pass | Formatting, whitespace, lints, and public documentation are clean. |
| 2026-09-03 | Final standalone release confirmation, 31 samples | pass | `gamma(10001)` 0.541 ms; `gamma(20001/2)` 0.683 ms; `gamma(-20001/2)` 0.902 ms; `beta(5000,7000)` 0.588 ms; half-integer `ln_beta` 61.2 ms. Log/root medians remain 235 ns, 328 ns, 159 ns, and 336 ns for the four measured cases. |
| 2026-09-03 | Valgrind Memcheck, release `gamma(-20001/2)` | pass | Zero definite, indirect, or possible leaks and zero memory errors; 752 process/static-cache bytes remain reachable at exit. |
| 2026-09-03 | Final optimized CLI size (`--release --features simple`) | accepted: 2,295,360 B | The combined product/log candidate is 4,176 B (0.182%) above the recorded pre-product/log snapshot; the exactness and measured time/memory gains outrank this small binary increase. No retained method appears among the 40 largest symbols in the bloat report. |
| 2026-09-03 | Post-validation source/invariant review; repeat `cargo fmt --all -- --check` and `git diff --check` | pass | Retained class certificates, radical normalization, signed half-integer bounds, product-tree endpoints, and exact log/root identities were rechecked directly; generated benchmark documentation remains absent from the patch. |
