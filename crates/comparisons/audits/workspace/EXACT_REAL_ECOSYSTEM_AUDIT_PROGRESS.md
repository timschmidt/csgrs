# Exact-real ecosystem to Hyper audit

This is the persistent workspace-root ledger for the active audit of the
public exact-real implementations and adjacent exact-number systems listed in
the 2026-09-03 audit request. A source target is marked **complete** only after
every relevant tracked file and every line in the recorded snapshot has been
read, each plausible idea has been compared with the current Hyper scalar and
performance architecture, and every experiment has a recorded disposition.

## Objective and retention order

Inspect `hyperreal`, `hyperlattice`, `hyperlimit`, `hypertri`, `hypercurve`,
`hypersolve`, and related crates as needed. Audit the supplied public
references file by file and line by line, looking for changes that improve, in
strict priority order: exactness, completeness, performance, memory use,
binary size, then source size. Retain a change only after focused correctness
tests and proportionate benchmark, allocation, and size evidence show that it
is worthwhile.

## Scope conventions

- Broad multi-purpose repositories such as FLINT, CGAL, Sage, Z3, Spire,
  Tcllib, and TypeTopology are audited line by line over their exact-real,
  algebraic-real, lazy-number, or directly supporting implementation slice;
  unrelated subsystems are inventoried but are not falsely counted as scalar
  audit coverage.
- Generated files, vendored third-party dependencies, opaque binaries, and
  image assets are identified and validated where useful, but are not counted
  as source lines read.
- Project sites, primary papers, and documentation linked by the request are
  reviewed for algorithms and semantics; source claims are checked against the
  pinned repository snapshot whenever source is available.
- Previously completed work remains in `CONSTRUCTIVE_REAL_AUDIT_PROGRESS.md`,
  `EXACTCORELIB_AUDIT_PROGRESS.md`, and related root ledgers. Hyper, exactCore,
  and the Boehm `CR`/`UnifiedReal` family are architecture baselines here, not
  duplicate source-audit targets.

## Safety and reproducibility

- Audit started 2026-09-03 in `/home/tim/Documents/GitHub/workspace`.
- `hyperreal`, `hyperlattice`, `hyperlimit`, and `hypertri` began clean.
- `hypercurve` began 16 commits ahead of `origin/main` with pre-existing edits
  to `src/bezier_offset.rs` and `src/bezier_region.rs`; those files and commits
  are user-owned and must not be overwritten or attributed to this audit.
  Additional concurrent edits later appeared in `src/bezier_arrangement.rs`,
  `src/bezier_parameter.rs`, and `src/curve_region_boolean.rs`, and the branch
  advanced to 19 commits ahead; they are likewise excluded.
- `hypersolve` began 10 commits ahead of `origin/main` and otherwise clean.
- At the Ariadne symbolic checkpoint, `hypercurve` is 19 commits ahead with
  only `src/bezier_offset.rs`, `src/bezier_parameter.rs`, and
  `src/curve_region_boolean.rs` modified; all remain concurrent user work.
  Hyperreal commit `bbd2e6f73d605bbe818453f2e7577e3d3496750d`
  contains the sole symbolic-slice audit change: denominator-before-identity
  regression coverage. Its worktree is clean at this checkpoint.
- At the Ariadne hybrid checkpoint, concurrent work has advanced `hyperreal`
  to `6ba6520c1b95245decabe8e6003833a57267d65c` (three commits ahead) with
  uncommitted `src/rational/arithmetic/ops.rs` and
  `src/rational/arithmetic/tests.rs`; `hyperlimit` is two commits ahead,
  `hypercurve` is 24 commits ahead and clean, and the other named trees retain
  their recorded clean/ahead states. The hybrid audit changes none of them.
- Reference repositories are pinned by commit below before coverage is
  credited. No source reference is modified.

## Hyper architecture map

| Area | Status | Evidence / notes |
| --- | --- | --- |
| Repository state and dependency manifests | complete | Six named repositories inventoried; path/version and feature relationships recorded. |
| `hyperreal` scalar representation and refinement | in progress | `Computable`/`Node`, compact atomic facts, lazy synchronized single-best dyadic cache, approximation dispatch, long-chain evaluators, bounded structural-normal-form memoization, magnitude-sensitive multiplication, rational/dyadic specialization, and integer-decision APIs inspected through the XRC comparison. A guarded total floor-or-ceiling choice was retained after full validation. |
| `hyperlattice` exact carrier and scheduling layer | in progress | Fixed-arity `Real` kernels, exact-rational/dyadic aggregate facts, zero-lane pruning, shared-denominator product sums, retained self-dots, and prepared matrix dispatch inspected during the Ariadne geometry/profiling comparison. |
| `hyperlimit` predicate/certification layer | in progress | `PredicateOutcome`, exact/refined/unknown staging, and the absence of a replayable semidecision object were inspected for the iRRAM/iRRAM+ comparisons. |
| `hypertri` irreversible topology decisions | in progress | Predicate ownership, exact AABB rejection, BRIO rounds, exact alternating-axis spatial medians, and the documented cheap-exact-grid-key architecture trigger inspected during the Ariadne geometry comparison. |
| `hypersolve` proposal, proof, and algebraic-root layer | in progress | Exact affine rank/DOF and Bareiss multi-RHS evidence, policy-certified reciprocal fallback, affine/quadratic Krawczyk reports, rational/integer Bernstein sign counts and recursive subdivision inspected. The independent AERN2 comparison found and fixed a dimensionally incorrect contraction proof. Joint inverse reuse and common-positive-scale integer Bernstein subdivision are separately retained after controlled qualification; public exact coefficient magnitudes and unknown outcomes are preserved. |
| `hypercurve` consumer architecture | in progress | Targeted Bezier parameter/sign-variation and rational Bezier subdivision paths were read for AERN2: exact endpoint guards, Sturm fallback and linear-live-storage de Casteljau. The ireal finite-jet comparison qualified exact high-order quotient coefficients and endpoint factorials in retained d978852. This is not whole-crate coverage. Concurrent user edits are excluded from audit patches and validation attribution. |
| Baseline correctness/performance/memory/size measurements | pending | |

## Source audit queue

### Tier 1: closest implementation and performance comparators

| Target | Snapshot | Coverage | Status | Candidate summary |
| --- | --- | ---: | --- | --- |
| RealLib | `b36b18ecd3d712311a12f384210735dfa319f86c` | 54 text files / 12,911 physical lines; bundled manual 1,311 extracted lines; companion paper 788 extracted lines; 2 binary artifacts inventoried | source complete; transfer experiments pending | Consumption-counted shared-node caches and difficult-child-first scheduling merit focused Hyper experiments. Approximation-level program slices are substantially covered by Hyper's specialized/fused kernels. Unsound legacy SIMD rounding, global precision state, and custom FFT arithmetic are rejected. |
| iRRAM | `a4d2409b9591227f1bbba93557989d0a2ba29d26` | 120 tracked files / 29,269 physical lines; all scalar, limit, container, test, example, build, and documentation sources read; 35-page PostScript paper converted to 1,852 text lines and read | source and transfer comparison complete | Whole-program reiteration and execution-order choice replay are poorer fits than Hyper's reusable immutable DAG and explicit certificates. Local retry, Lipschitz repair, and provisional algebraic metadata are already subsumed by Hyper's per-node refinement/exact facts or do not preserve its stronger model. Only profile-gated temporary pooling and transcendental algorithm comparisons remain cross-target experiments. |
| iRRAM-bigsteps | `0c6efb521f9529c270cb3aa34741694713b562bf` | 42 tracked text files / 9,730 physical lines; all solver variants, support/visualization code, and ODE fixtures read | source and transfer comparison complete | Certified best-partial Taylor selection is unnecessary under Hyper's fixed integer error budgets and monotone reduced-argument series. Lazy memoized coefficient convolutions and sparse support-index iteration are useful for a future exact IVP layer, but provide no scalar change today; compare sparse support traversal with `hypersolve` before finally closing that narrower implementation pattern. |
| iRRAM+ | `13b7229af93e2a528153d85f439b4158378f6b14` | 41 tracked text files / 3,085 physical lines; 6-page PDF / 273 extracted lines; ELF artifact inventoried | source and transfer comparison complete | A continuous conditional and dimension-certified discontinuous linear algebra are the substantive ideas. Hyper lacks the replayable semidecision contract required for the former, while its explicit rank/certificate reports already realize the latter more safely at current solver boundaries. Analytic continuation and several matrix routines are incorrect or incomplete, so no code is transferred. |
| iRRAMx | `1d33af09e005fdff8655e4b29fed3918f54f8d21` | 68 non-documentation text artifacts / 10,357 physical lines (including the 2,576-line generated Doxygen configuration); 340 generated documentation assets / 1,729,903 bytes inventoried, with 96 human-facing pages and all 22 stale source snapshots extracted and read | source and transfer comparison complete | Operational discovery of a modulus by interval perturbation and compact-domain coverage is the only new general contract, but Hyper has no generic function-space consumer and this snapshot's multidimensional subdivision is wrong. Complex-root component replay is weaker than `hypersolve`'s persistent polynomial/isolator/fiber identities. Strassen, approximate compact membership, random/Wiener sampling, and the defective matrix/polynomial implementations are rejected. |
| IC-Reals | release 6.3 archive, SHA-256 `9f8ebab0b67a7c64126e85f3dc35b3bf0889432086c6fa6021b894faec67a2c1` | all 133 files / 20,913 physical lines read; separately published 16-page manual and 1-page licence read | source and transfer comparison complete | Destructive linear-fractional digit streaming, persistent shared prefixes, sensitivity-directed demand, producer/consumer fusion, and packed signed digits were evaluated. Hyper's immutable batched-dyadic model already subsumes the useful effects more safely; controlled packing tests produced no high-precision gain, and extensive correctness probes reject the implementation as a donor. |
| XRC | release 1.2 archive, SHA-256 `c4b6d83264324a7714630f5628a4c73cb580c1f49ba7a9397896acc7f8e11af3` | all 29 files / 4,498 physical non-PDF lines read; three-page PDF manual read | source and transfer comparison complete | Its intended total floor-or-ceiling choice survives as Hyper's guarded `near_integer`; cold `sqrt(2)` benchmarks are about 5.3--5.5x faster than directional certified floor and exact integer-boundary identities need no equality proof. One-finest caching, magnitude-sensitive multiplication, integer nodes, and exp scaling are already subsumed. The reference violates its approximation contract broadly, leaks every graph, overflows recursive evaluation, and is not a code donor. |
| Spigot | `4ef5af3408a890ce440833201c35cf511cdaa52d` (2026-08-17; published as `spigot-20260818.4ef5af3.tar.gz`) | all 77 tracked text files / 33,912 physical lines read; public landing page and all 768 rendered manual lines read | source, adversarial validation, and scalar transfer comparison complete | Its certified removable-hole treatment produced retained local analytic-continuation nodes for Hyper's `sinc`, `sinc_pi`, and `cosc`: an opaque mathematical zero now succeeds and formats immediately instead of returning `UnknownZero`, with 2,430 MPFR comparisons, full-suite/downstream gates, about 32--96x cold construction improvement over the former failed path, unchanged node layouts, and a 3,768-byte stripped-example cost. Sensitivity scheduling is already subsumed; restart clones, unbounded retained input, the internal bigint, and reference defects are rejected. Correctly rounded text and its broader special-function algorithms remain explicit cross-reference candidates rather than uncritically copied code. |
| Ariadne | `a86839f37e7e7ccb7077fd6fddb9cc09f418a29b` (2025-07-03) | all 809 regular parent-tracked files / 183,414 physical lines read; four parent gitlinks inventoried; official project, release, installation, tutorial, and publication pages reviewed | source and transfer comparison complete | The scalar/function/symbolic/solver/geometry/dynamics/hybrid architecture, Python exposure, tutorials/examples, dormant experiments, documentation, and project support were audited end to end. Useful high-level patterns—typed information paradigms, immutable DAGs, explicit uncertainty, numerical-proposal/certificate separation, sparse expansions, normalized domains, certificate-only scheduling, and independently rigorous algorithm portfolios—are already subsumed or target validated function/dynamics layers Hyper does not have. Focused probes instead expose wrong values, erased domains, invalid reads/writes, dangling captures/references, recursive/nonterminating paths, ignored terms, broken progress, silent corruption, ownership faults, stale APIs, and an upstream MP Taylor-compose segfault; documentation and CI do not repair those contracts. Only two test-only Hyper assertions guarding denominator-before-identity ordering are retained. |
| Boost.Real 2018 | `5532037f5e4e5cad5d7e940a49191bb44099a7e5` (2019-04-16; `v1.0.0-beta` is its immediate parent) | all 292 regular parent files / 45,762 physical lines read; one gitlink inventoried | closed | Every parent file and generated-document byte is read. The untouched suite needs a modern-Catch compatibility define, then passes 19/19, but independent oracles expose non-enclosing addition/subtraction, broken terminal and batched refinement, negative-zero ordering, copy/assignment/precision defects, ODR and header-self-containment failures, crashes from valid public construction/assignment paths, and deterministic iterator/expression leaks. Public documentation confirms rather than resolves the raw-tree, redundant-recomputation, and base-conversion costs. No change retained. |
| Boost.Real 2021 | `4db9b8f87b2b71748054190d085c449e866a529b` (2021-08-23) | all 101 regular parent files / 49,841 physical lines read; two gitlinks inventoried | source, adversarial validation, and transfer comparison complete | The shared-pointer rewrite frees complete acyclic DAGs, but its mutable child iterators make answers history-dependent and racy. Independent exact oracles expose broken bigint arithmetic, excluded true values, reversed transcendental intervals, sign loss, crashes, UB, and nontermination; its benchmark claims are invalid. Hyper already has the useful DAG/cache/tree-specialization ideas in synchronized, history-independent form. No change retained. |
| flatsurf exact-real | `fc4104076cb76aa6efe11df6ae15b208b18d12f5` (2025-12-17; 92 commits after 4.0.1) | all 178 regular parent files / 41,794 physical lines read; two symlinks and six pinned top-level gitlinks inventoried | source, validation, and transfer comparison complete | The untouched 12-program suite and its 182,883 assertions pass normally and under Memcheck, but independent numerical checks expose contract defects. Corrected sparse/cold benchmarks reject dense zero-term growth and do not justify unsynchronized caches or global interning. The power comparison found and fixed Hyper's own negative-base logarithmic fallback; 96 MPFR checks, full gates, paired timing, allocations, and binary-size review support retaining commit `8a74b42`. No reference code was copied. |

### Tier 2: Rust and Haskell representation experiments

| Target | Snapshot | Coverage | Status | Candidate summary |
| --- | --- | ---: | --- | --- |
| realistic | `d7d292aff5a1fa0add9161cc8bd1e88f90905fee` (2026-08-13; 0.8.2) | all 16 files / 5,794 physical lines read | source, numerical validation, and transfer comparison complete | 133 debug/release tests, 18 doctests, and the normal Memcheck suite pass. Independent arithmetic/MPFR checks validate low-level kernels but expose higher-level canonicalization, sign, cancellation, parsing, and rounding defects. Hyper already addresses those architecture patterns. A measured low-precision square-root lead yields the only retained transfer: demand-sized guarded integer seeds in commit `44ce87e`, with about 21--61% improvement at 16/64/96 bits, fewer allocations, a documented 140-bit timing tradeoff, and 96-byte stripped-driver growth. |
| computable-real | official 0.3.0 crate; VCS `2c962a8ffcebf91fd0e4e4b76eb9f0c948123941` | all 8 files / 1,572 physical lines read | source, numerical validation, and transfer comparison complete | Official archive SHA-256 `d15f7d44e4a3b8bbaf4222ccc45a3196a9dec539d2a0d10eb14e31635fd50d9c`; debug/release unit and 31 doctests pass, but independent checks expose magnitude-sensitive sqrt and comparison failures. Deep clones and local caches lose to Hyper's shared DAG. A safe integer-only sqrt crossover experiment is retained in `93c44ab` after full gates, paired timings, allocations, and size review. |
| reals | official 0.4.0 crate; VCS `57107c9161fd0a5fa235b2055f42da3f0c364810` | all 9 files / 2,979 physical lines read | source, numerical validation, and transfer comparison complete | Official archive SHA-256 `5931e5a33599abd0514ffc6e7bb902d161963bb1a1670b1c03df89fe0f4d9775`; debug/release unit and 25 doctests pass with pinned companion. Independent checks expose parsing, powers, sign-reversing comparisons, duplicated factors, and lost scales; 51 Hyper controls pass. No additional symbolic change survives comparison. |
| computable | official 0.1.0 crate; VCS `8441d7b2032481091399e77ddba74b1b5680263d` | all 59 files / 12,421 physical lines read | source, numerical validation, and transfer comparison complete | Archive SHA-256 `e3134cfa5d49bf2fb4292fb9711cedc1eb0571f4ce790a4e840eea8f3f847912`; untouched debug/release/doctest gates pass, but independent exact and MPFR checks confirm unsound normalization, wrong roots, pi/sin enclosure failures, and history-sensitive incomplete refinement. All 459 Hyper controls pass. Paired scalar timings and allocations reject per-node OS-thread scheduling and dual-BigInt dyadic intervals. No Hyper change retained. |
| exact-real-rs | `4f1f99149da7aee269eb1f26375ba4ee7ba123e4` | all 6 tracked files / 133 physical lines read | complete; empty scaffold, no transfer | This is an empty scaffold, not an implemented exact-real system: src/lib.rs has only documentation and a no_std attribute. Default tests report zero tests and no-default-features build passes. The official docs.rs 0.0.0 published source has the same three lines. No algorithm or meaningful scalar benchmark exists to transfer. |
| boehm_reals | `7787d8811ec7f64c599d3d9176fd83b3c2e1dabf` | all 35 tracked files / 5,772 physical lines read | source and numerical validation complete; cold-storage cross-reference experiment pending | Debug/release suites pass, the stale fuzz target does not compile, and independent arithmetic/float checks expose import/export/formatting contract defects. The comparison produced Hyper correctness fix `3f57867`, fully qualified and retained. Reused-input timing favors Hyper; fresh-input timing/memory exposes an inline-storage versus shared-canonical-storage tradeoff that remains explicitly pending, not silently rejected. |
| AERN2 / ERC implementation slice | `d1ac3664bfb5c7f70fcf68f7fb412d288def65cb` | 407 text files / 45,409 physical lines read, including all handwritten source; two bundled PDFs / 692 extracted lines and ERC paper / 2,977 extracted lines read; all 66 other single-page PDFs visually inspected; all 1,483 parent-tracked paths inventoried | handwritten source and targeted transfer comparison complete; artifact limits recorded | All numerical code, tests, benchmark/visualization generators, metadata and prose read. Generated/vendored artifacts are explicitly not counted as source reads: 214 format checks pass, all 21,800 serialized polynomials parse, 652 matrix/fnreps report rows match logs, and five network traces reconcile. Stored fixtures repeat inputs and historical reports mix working/achieved accuracies. Active tests, independent rational/MPFR checks and scalar/linear/integer-polynomial/DCT kernel benchmarks are recorded; untouched ERC/linear/net/univariate builds fail and external compatibility slices are qualified only within their stated scope. The factorization comparison's independent public oracle found and qualified Hyper's dimensionless Krawczyk contraction correction, retained in Hypersolve commit `1273a7a` (771-test debug/release gates, paired speed/allocation/size improvements). Joint exact inverse elimination is also retained in Hypersolve `a20be23`, with 773-test debug/release gates, 196 independent matrix identities, 18 paired public workloads, lower allocation demand and smaller linked drivers. Shared-scale Bernstein subdivision is retained in Hypersolve `4dfbb39`: 208 identical exact reports, independent coefficient/partition oracles, 778-test debug/release gates, 17 pinned A/B workloads and Memcheck. Deeper workloads improve about 2--12.5x with up to 91% lower cumulative allocation demand; small-case costs and 12.5KB driver growth are documented. No generic function-space or DCT layer is justified. |
| AERN | `3a45d80cfa2197ddb4935b96509348331c8b763d` (2015-12-02) | All 281 text files / 58,894 physical lines read; all 288 tracked paths inventoried, including two resolved symlinks and five visually inspected binary assets; ICMS2014 paper, 420 extracted lines and all five figures read | source, native qualification and targeted transfer comparison complete; documentation limits recorded | Retained Hyperreal exp accuracy fix da66ba3, expm1 fix d219a35, finite binary64 enclosure correction/bound-based scaling 2808bc8, and Hypersolve monomial-preserving quadratic extraction bb85085 after rigorous qualification. Native order/generator/scheduling, mixed arithmetic, both MPFR backends, Double elementary, polynomial, public IVP, DCT, temporary root/function-space and hybrid-catalog probes are recorded. DCT transfer rejected by exact-oracle/three-run timing and allocation comparison. Generalized function enclosures, shrink-wrapping and hybrid simulation need a different consumer; replacing Hyper's exact scalars would weaken their contract. No extra production changes retained. |
| CDAR and `mBound` branch | master `1919f75c2cacd9f994ad50e925d8932d2639b690` (2023-07-19); mBound `132e0da252cb723dc1247f419919bdbfd4072e9e` (2026-08-11) | all master27 text files /4,025 lines and mBound24 /3,717 read; identical ELF identified without execution; both ODS artifacts'981 populated cells/formulas and thumbnails inspected | source, native qualification and targeted transfer comparison complete; paper retrieval limit recorded | Both unchanged90-test suites pass, but independent Rational/MPFR probes expose division, rounding, polynomial, interval-transcendental, alternative-wrapper and conversion defects. Mantissa capping passes13,965 boundary cases and39,690 arithmetic cases;72 CPU6 ABBA storage samples show a large excess-exact-work squaring benefit but material small-case overhead. Hyper already budgets intermediate precision and retains one finest approximation;5,244 matching MPFR checks and18 conversion checks pass. No new production change retained. |
| ireal | `a7c83281362c5dc857e97d98b424ec314a216b56` (2025-03-12) | all38 text files /3,769 physical lines read; all9 PDF pages visually reviewed; inventory/ranges/SHA-256 reconcile | source, numerical qualification and targeted transfer comparison complete | Unchanged46-property suite passes4,303 cases. Independent277,500 point-arithmetic checks pass, but mixed-sign interval multiplication/division and interval sqrt have certified enclosure failures. Retained Hypercurve high-derivative completeness fix d978852 and Hyperreal scale-aware additive scheduling21e76ea follow independent exact/MPFR controls, full proportional gates and paired timing/allocation/size qualification. Raw-depth scheduling was rejected after a4.5x scaled regression; signed demand removes it while improving the512-term shared-prefix case about99x. Full costs and measurement limits are recorded below. |
| Haskell `exact-real` / `Data.CReal` | `7bb2abaed01ac874d914c99919e81cfd00dd7d6d` (2026-06-11) | all36 text files /2,466 physical lines read; inventory/ranges/hashes reconcile | source, numerical qualification and targeted transfer comparison complete | Unchanged499-test suite and29 doctests pass.337456 exact arithmetic and15379 MPFR controls pass, but retained signum/atan2 histories, false convergence and128 display-contract cases fail independently. Eager Hyper cache seeding is rejected after432 MPFR controls,900 exact endpoint checks,480 paired CPU samples and120 allocation controls: warm query gains do not justify substantial unqueried construction costs. Baseline2345 decimal,120 MPFR quadrant/history and20 sign controls pass. No new production change retained. |
| haskell-fast-reals | `8fa09b2457d7099c75b7db62895b5c8f3391d9e3` (2017-04-19) | all 23 text files / 2,954 physical lines read; one ELF identified without execution; hashes/ranges reconcile | source and targeted transfer comparison complete; historical full native ABI unqualified | Unchanged generic modules compile on GHC 9.6.7. Independent directed/interval checks confirm negative-rounding and nonconvergent abs defects; stock-MPFR policy reconstructions expose frozen 64-bit integer points and inward abs upper bounds. Forty-eight precision-cache observations reject retained-list lookup/storage for Hyper; 79,705 baseline Hyper controls pass. Required custom MPFR dependency has an explicitly scoped 19-file read slice, not whole-vendor coverage. No new production change retained. |
| HERA | official HERA-0.2 archive, SHA-256 `b4dcd62c876c5cb4cc5bb42465dfcc7dfe71695934b9887f9e7acbf29015eae9` (2008-09 source dates) | all 14 source-package files / 2,849 physical lines read; hashes/ranges reconcile; RZ paper and both slide decks / 1,601 extracted lines read | source, scoped numerical qualification and targeted transfer comparison complete; native-build limits recorded | Original build fails on obsolete MPFR tags; explicit current-host compatibility build preserves numerical formulas. Independent probes expose ball-mul/div/exp/log, constants, frozen imports, limRec and finite-series failures, while 684,162 ordinary dyadic-kernel controls pass. 108 cache and 48 compact-radius CPU/allocation observations reject last-stage eviction and show why variable-radius compression is not a Hyper scalar-cache improvement. Both cache variants pass 156 independent radical-sum/history controls, and unchanged Hyper passes 227 matching controls. No production change retained. |
| Few Digits | official 0.5.0 archive SHA-256 `78ac940fdc86ce546bddfbb2ff45bd14d34c812e3ee1b0079f42aad394a837cf` (2006-07 source dates) | all 11 files / 1,037 physical lines read; all 21 supporting-paper pages visually read; hashes/ranges reconcile | source, scoped numerical qualification and targeted transfer comparison complete; native-build limits recorded | Compatibility-only build passes 655,415 ordinary exact controls and 216 interior MPFR checks but exposes wrong cosine dispatch, result-changing sqrt bounds, zero/endpoint nontermination and polynomial derivative exceptions. 180 donor plus 180 Hyper sum observations and 48 counter-disabled repeats reject balanced LCM for Hyper. 54 independently qualified compression observations favor existing dyadic approximation architecture. 920 Hyper scalar/history and 486 scoped Hypersolve derivative controls pass. No new production change retained. |
| Escardo signed-binary exact reals | 971ff20d35a4af4be595364c704ad118c4bf3c37 (2025-11-14) | all 5 files / 1,995 physical lines read; full interval-object and functional-integration papers read | source, native qualification and targeted transfer complete | Native build passes but independent Rational/MPFR grids expose selected multiplication, min/max, root-domain, fixed55-digit imports and machine-Int overflow failures. All498 BBP decimal digits pass. 405 qualified donor timing/allocation rows establish lookahead and zero-prefix tradeoffs. The continuous-absolute-value comparison produced Hyperreal bd92d87: evaluate unresolved sqrt(Square(x)) with one child approximation. 480 final A/B rows show2.68--3.70x targeted speedup and49--78% lower cumulative allocation; debug/release826-test gates,967 independent controls and693 scoped downstream tests pass. No new node, field or wire format; broader searchable-function closures are not selected. |
| Plume | original archives preserved in martinescardo.github.io at2f2aa72457d14f2363a741af1ebb52932a48d6ec |all174 artifacts inventoried;170 text files/24049 physical lines independently read; both4871-line report texts read, primary133 visual pages and11 selected alternate pages read;147 generated HTML section links indexed | source, scoped numerical qualification and targeted transfer comparison complete; generated-container/native-build limits recorded | Main/instrumented/boundary grids expose arithmetic, normalization, decimal carry, demand-tag and functional-contract defects. Qualified paired benchmarks reject lower lookahead as a general CPU/allocation improvement. The fixed-decimal normalization comparison produced the sole retained Plume Hyperreal edit: bounded Display search for cancellation/tiny values.723 default-debug and828 release-all-feature tests,2488 independent controls perbuild and1587 scoped downstream library passes qualify it;72 paired process rows give140.918x tiny-radical median CPU improvement and99.718% fewer requested bytes, ordinary-value intervals include parity. Audit loaded size unchanged. No general function-space API or unqualified donor code transferred; all details/limitations in PLUME_FILE_NOTES.md. |
| `numbers` / `Data.Number.CReal` | jwiegley/numbers 0ab9e067e10a2e097239f73bdcbbffeae09790b0 | all19 tracked files/1786lines independently read; supplied haskellcats catalogue54lines separately read | source read complete; scalar coefficient repair qualified and retained; broader target scope tracked separately | Retained atan, asin/atanh-domain, ln_1p-domain and inverse-series coefficient-width repairs have frozen per-slice evidence. The last repair removes debug overflow and release non-enclosure at192000bits:16public high-precision requests,746/853full tests,696scoped downstream passes and zero Memcheck errors.198benchmark observations/41508MPFR checks quantify mixed timing and lower high-precision allocation demand; no blanket speedup. Two historical Hypercurve failures reproduce before/after on the same frozen user snapshot. See numbers-qualification/coefficient-README.md and the detailed checkpoint below; these historical gates do not qualify later user edits. |
| haskell-constructible | 46d760cbd2d21f955ec96c8fe2c13fdf3b2dd9d0 (0.1.2;2021-11-09) | all4 tracked regular files/490physical lines independently read; selected dependency slice5files/466lines | source, scoped native and transfer qualification complete | Direct GHC9.6.7 O0/O2 builds bypass obsolete Cabal bounds; independent checks confirm negative irrational properFraction and extreme-scale floating-conversion defects. Retained algebraic-integer quotient separation resolves1324/1324public field cases at-512 versus28baseline UNKNOWN, with unchanged representation/API/caps.750/857full tests and frozen downstream gates pass.162paired timing observations/4508640comparisons plus54allocation observations qualify lower deep proof cost and memory; shallow timing intervals include parity. See constructible-qualification/fractional/README.md for exact proof, final results and user-snapshot limits. |
| haskell-reals-comparison |7c53d4e4259f633606b18d655cdbbf813866ab81 (2022-07-15)|14 text/code files/1534 lines read; all567 files inventoried; 525 logs and28 chart assets reconciled|source/report provenance audit complete; no transfer|All source and report files read. Every log maps one-to-one to an all.js row; RSS matches, 385 timings were normalized, and all CDAR logs omit achieved accuracy. Static chart assets contain no scalar machinery. See HASKELL_COMPARISON_FILE_NOTES.md.|

### Tier 3: other-language implementations

| Target | Snapshot | Coverage | Status | Candidate summary |
| --- | --- | ---: | --- | --- |
| Ruffini (whole repository) | 82d552fee22d92e493936183fab8672517694e56 | 244 source/data files read, 26,222 lines; 1 generated SVG reviewed separately; all 245 files/26,234 physical lines accounted for |READ COVERAGE COMPLETE; scoped cache transfer qualified and retained; other native claims bounded|Overflow-safe borrowed coarsening preserves the gap-one path and rounding.756/863retained Hyperreal tests/static gates pass; downstream202/348/7/796pass, current Hypercurve910pass/1ignored.2828public timing/312allocation observations qualify large-gap gains with disclosed1–3%small rational-case costs. Earlier polynomial/sqrt pilots not retained. See cache-public-pilot/retained-checkpoint.json.|
| Common Lisp computable-reals |607a5d5b95387c06f92a661aa5562a7be0f5cad8 (2021-03-05)|8 files/877 lines read|source and transfer review complete|See COMPUTABLE_REALS_FILE_NOTES.md.|
| Creal |0520c6351fdc431db3c0941b5cf111211826f88a|13 files/1850 lines read|source and transfer review complete|Mutable base-4 caches, coarse branch heuristics and uncertified pole/domain behavior are weaker than Hyper; see CREAL_FILE_NOTES.md.|
| Spire `Real` and algebraic-real slice |0fe5a6a9714181a20fc9cef4c8b2af088ff2b4c9|5 files/2959 lines (Real, Algebraic, IsReal, and two ScalaCheck suites) read and inventoried|source/transfer review complete; no transfer|Signed-binary precision-indexed Real, exact algebraic AST with Li–Yap/BFMSS separation bounds, atomic root refinement, and replay conversion audited. Scala tests could not run because sbt is unavailable. Fixed-precision inexact comparison and unbounded recursion are unsafe; no worthwhile Hyper change. See SPIRE_REAL_FILE_NOTES.md and SPIRE_REAL_FILE_INVENTORY.tsv.|
| CRCalc.js |ea5d92921599f3666c6afbf86235864e66469da9|63 files/19537 lines inventoried; scalar and web TypeScript sources read|source/transfer review complete; no transfer|AOSP-style CR cache, bounded-rational/UnifiedReal symbolic side channel, precision guards and demand-driven worker scheduling audited. Equality may diverge by design; worker maps are unbounded; UnifiedReal.asin has an always-true OR guard. Checked-in distribution test passes; native npm rebuild lacks terser. See CRCALC_JS_FILE_NOTES.md.|
| Numbas constructive-real extension |b840893fe89308ee2a3e5167a901295dc5c2851b|2 files/2253 lines (README and embedded CReal/JME JavaScript) read and inventoried|source/transfer review complete; no transfer|Embedded AOSP/Boehm BigInt CReal with precision cache, slow-worker batching, postfix serialization, timeout and AGM pi state reuse audited. Node core probe verifies 20-digit sqrt2/pi/e; browser host unavailable. Malformed max/min display strings, coarse equality and unresolved select state found; no worthwhile Hyper change. See NUMBAS_CONSTRUCTIVE_REAL_FILE_NOTES.md and NUMBAS_CONSTRUCTIVE_REAL_FILE_INVENTORY.tsv.|
| Tcllib `math::exact` |6093f8d6246572ae461bf344333483ca329413bc|6 exact-real artifacts/7595 lines inventoried and read (implementation, tests, source/manual plus generated Markdown/nroff/HTML)|source/transfer review complete; no transfer|TclOO Mobius/tensor lazy computable reals, intrusive refcounts with deferred deletion, memoized digit workers, and explicit tolerance sign APIs audited. Tcl test suite: 198 passed, 0 skipped, 0 failed; documented reference-counted runtime probe passed. Domain failures may loop/overflow and ownership is manual; no worthwhile Hyper change. See TCLLIB_MATH_EXACT_FILE_NOTES.md and TCLLIB_MATH_EXACT_FILE_INVENTORY.tsv.|
| ExactReals.jl |19d32155e69ad988cd68282d4e88b859552033e3|11 files/382 lines read|source and transfer review complete|Zero multiplication nonterminates in independent bounded probe; mutable one-cache Cauchy model adds no safe transfer. See EXACTREALS_JL_FILE_NOTES.md.|
| DedekindCutArithmetic.jl |dc7a51b784a6d8f04be22fcca862656432a3d87e|55 files/2651 total lines; source/docs/tests read plus complete 55-file inventory|source/transfer review complete; no transfer|Dyadic/Kaucher intervals, mutable cached Dedekind cuts, composite refinement and derivative-assisted quantifiers audited. Division-by-zero diagnostic has an undefined variable; exponent-zero round can shift negatively; recursive comparisons/quantifiers have no unknown/termination contract. Julia tests blocked by missing dependencies and full registry quota. See DEDEKINDCUT_ARITHMETIC_FILE_NOTES.md.|
| Edalat prototype |474b133ad9cd52d94000efcfcb6febb3722edf2f|3 files/2095 lines (README, interval-domain core, tests) read and inventoried|source/transfer review complete; no transfer|Python domain-theory prototype audited. Checked-in source has raw 0xB2 UTF-8 SyntaxError; in-memory normalized run passes 32/32 tests. Float-limited rationals, weak chain validation, inconsistent bottom semantics, broad exception-to-bottom handling, and rough measure pushforwards make it unsuitable for Hyper. See EDALAT_FILE_NOTES.md and EDALAT_FILE_INVENTORY.tsv.|

### Tier 4: formal and research implementations

| Target | Snapshot | Coverage | Status | Candidate summary |
| --- | --- | ---: | --- | --- |
| CoRN exact-real/analysis slice | `ada7c0b497ff15dd67cf7932c6f20e143a2aee2f` | 375 files / 189,019 lines read; 375 source files / 189,930 lines inventoried | optimized and selected fast arithmetic slices complete; all 375 inventoried CoRN files read | Read the optimized files plus COrdAbs, Bernstein, CAbGroups, CAbMonoids, CFields, CGroups, CMonoids, CPoly_ApZero, CPoly_Degree, CPoly_Newton, CPoly_NthCoeff, CRing_Homomorphisms, CSemiGroups, CSetoidFun, CSetoidInc, CSetoids, Cauchy_COF, OperationClasses, RSetoid, broken/CCayleyHamilton, broken/CPoly_Lagrange, broken/CompletePointFree, broken/DivDiff_RepeatedIntegral, broken/IntegrationExamples, broken/NewAbstractIntegration, broken/SimpsonIntegration, broken/abstract_gsum, broken/algebra/bigopsClass, broken/diff, broken/lagrange, broken/matrixClass, complex/CComplex, complex/Complex_Exponential, coq_reals/Rreals, coq_reals/Rreals_iso, examples/Calculemus2011, examples/Circle, examples/IntegrationExamples, examples/LMCS2011, examples/Picard, examples/PlotExamples, examples/RealFast, examples/RealFaster, fta/CC_Props, fta/CPoly_Contin1, fta/CPoly_Rev, fta/CPoly_Shift, fta/FTA, fta/FTAreg, fta/KneserLemma, fta/MainLemma, fta/KeyLemma, ftc/COrdLemmas, ftc/Derivative, ftc/DerivativeOps, ftc/CalculusTheorems, ftc/Composition, ftc/Continuity, ftc/Differentiability, ftc/FTC, ftc/FunctSequence, ftc/FunctSums, ftc/IntervalFunct, ftc/MoreFunSeries, ftc/MoreFunctions, ftc/MoreIntegrals, ftc/MoreIntervals, ftc/NthDerivative, ftc/PartFunEquality, ftc/PartInterval, ftc/Partitions, ftc/Taylor, ftc/Rolle, ftc/StrongIVT, ftc/TaylorLemma, ftc/WeakIVT, ftc/WeakIVTQ, liouville/CPoly_Euclid, liouville/CRingClass, liouville/Liouville, liouville/QX_ZX, liouville/QX_extract_roots, liouville/QX_root_loc, liouville/Q_can, liouville/RX_deg, liouville/RX_div, liouville/RingClass, liouville/Zlcm, liouville/nat_Q_lists, ftc/RefLemma, ftc/RefSeparated, ftc/RefSeparating, ftc/RefSepRef, CRexp, CRroot, contraction/fixed-point, UniformContinuity, OddPolyRootIR, IntegrationRules, LinfDistMonad, MetricMorphisms, Qsec, CMTcast, IrrCrit, OpenUnit, Cesaro, QnonNeg, StepFunctionMonad, Qmetric, IVT, AbsCC, CReals1, NNUpperR, COrdCauchy, TrigMon, CMetricSpaces, FunctSeries, ArTanH, Hausdorff, iso_CReals, CRings, Bridges_iso, Max_AbsIR, CMTReals, ConstructiveUniformCont, NRootCC, ConstructiveCauchyIntegral, trigonometric/logarithmic kernels, series, representation bridges, group operations, CRFieldOps, Bridges_LUB, Cauchy_IR, metric Classification, BoundedFunction, IntegrableFunction, CRpartialorder, UCFnMonoid, Qpossemigroup, QSpossemigroup, CRreal, and Raster line by line. TrigMon’s exact sign/derivative proofs and cosine-away-from-zero bounds were compared against Hyperreal trig kernels; no worthwhile change selected. See exact-real-references/CORN_FILE_NOTES.md and CORN_FILE_INVENTORY.tsv. |
| Rocq exact-real-arithmetic | d6bc1fee859c5b773dc46ef8efd4d43ba29697c5 | 35 tracked files / 5,471 physical lines read | source complete; native Coq validation unavailable | Unsafe proof-oriented Reelc Z->Z representation audited; Axiomes.v proves False from an inconsistent msd axiom and Inverse.v ends Abort. Magnitude-aware index planning is already subsumed; no Hyper change. See exact-real-references/ROCQ_EXACT_REAL_ARITHMETIC_FILE_NOTES.md. |
| coq-aern | pending | 0 | pending | |
| Marshall | completed | 37 text/source files fully read (plus 18 generated figures classified) | no native toolchain; tests not runnable | No worthwhile Hyper change; see exact-real-references/MARSHALL_FILE_NOTES.md |
| RZ | d92cacaf78fb50d61fc6712c74b8fdaf5d2c6d28 | 283 tracked files / 148,131 lines inventoried; all `src` compiler/printer files, examples/tests, and all 55 unique code-like private artifacts read in bounded non-truncated chunks | executable source audit complete; non-code private papers/slides/LaTeX/binaries and duplicate historical copies classified rather than treated as scalar source; executable RZ corpus has no scalar runtime/cache mechanism | See exact-real-references/RZ_FILE_NOTES.md and RZ_FILE_INVENTORY.tsv; findings include proof-preserving obligation hoisting, dual-context thinning, and non-transferable compiler bugs; no Hyper change selected. |
| Ternary Boehm reals artifact | pending | 0 | pending | |
| Clerical paper and Coq formalization | pending | 0 | pending | |
| TypeTopology exact-real/searchable slice | pending | 0 | pending | |
| Bishop | pending | 0 | pending | |
| Lean ComputableReal | pending | 0 | pending | |
| Acorn | pending | 0 | pending | |
| ROSCoq constructive-real boundary | pending | 0 | pending | |

### Tier 5: exact algebraic and symbolic-real systems

| Target | Snapshot | Coverage | Status | Candidate summary |
| --- | --- | ---: | --- | --- |
| FLINT Calcium | pending | 0 | pending | |
| LEDA `real` | pending | 0 | pending | |
| CGAL lazy/exact number types and CORE interface | pending | 0 | pending | |
| e-antic | pending | 0 | pending | |
| Sage `AA` / `QQbar` | pending | 0 | pending | |
| SRI LibPoly algebraic numbers | pending | 0 | pending | |
| Z3 algebraic/real-closed-field slice | pending | 0 | pending | |
| AlgebraicNumbers.jl | pending | 0 | pending | |
| Nemo/Oscar exact-field layer | pending | 0 | pending | |
| Rust `algebraics`, `puremp`, and `neco-algnum` | pending | 0 | pending | |

### Historical sources and surveys

| Target | Evidence | Status | Notes |
| --- | --- | --- | --- |
| Manchester ERA | linked paper/site | pending | Public source availability must be rechecked. |
| ExactLib | linked report | pending | Public source availability must be rechecked. |
| XR | Briggs software page | pending | Surviving source availability must be rechecked. |
| Competition and survey references | linked primary/secondary sources | pending | Used to cross-check implementation inventory and missed algorithm families. |

## Candidate transfer ledger

| Candidate | Priority | Source | Hyper target | Status | Evidence / disposition |
| --- | --- | --- | --- | --- | --- |
| Make deeply nested evaluation stack-safe for every reachable node family | completeness | RealLib `RealObject.cpp`, paper section 2.4; IC-Reals continuation stack | `hyperreal::computable` dispatcher | partial qualification; generic families pending | Hyper’s existing iterative evaluator passes four focused 5,000-node debug tests: deep Add approximation, exact Multiply-by-one, irrational Multiply-by-one, and structural-bound traversal. Generic nested elementary/inverse/root families are not covered by these tests, so no implementation change or full closure is claimed. |
| Evaluate the more difficult/deeper operand first | performance / memory | RealLib `RealObject.cpp`; paper section 3; ireal `FoldB.hs`/`Zeta.hs` | binary `hyperreal::computable` kernels | scale-aware Add ordering retained21e76ea; other kernels remain pending | Shared-prefix comparisons show about99x improvement with a signed saturated demand hint incorporating Add guards and Offset shifts. Raw depth was rejected after a4.5x counter-regression. The hint changes ordering only, never precision or numerical facts. Nonlinear difficult-child ordering and peak-live-memory policies are not closed by this limited transfer. |
| Release cached approximations after their last consumer in an evaluation pass | memory | RealLib `RealObject::{GetEstimate,ClearEstimate}`; paper section 3 | `Node::ApproximationCache` | rejected after measurement | Hyper caches one finest synchronized value per immutable shared node and frees it with `Node`; there is no evaluation-pass consumer accounting. Existing cache microbenchmarks measured ~19.1–19.7 ns hits versus ~28.4–30.5 ns cold queries for rational/pi cases (about 1.5x benefit). The release-profile counting-allocator across 20 recipes × 4 lifecycles showed retained bytes only for process/shared constants or live roots (0–888 B per recipe) and zero retained bytes for most dropped graphs. Clearing on inferred “last consumer” would be unsafe for later refinement/concurrent/shared queries and offers no demonstrated memory win. |
| Encapsulate loops/aggregates as approximation-level programs rather than operation DAGs | performance / memory / code size | RealLib `RealEncapsulation.h` and manual section 4 | Hyper fused kernels and downstream scalar-query APIs | provisionally subsumed | Hyper already exposes specialized sum/product/linear-combination kernels and downstream query objects. Revisit only if a concrete reference workload still builds an avoidable operation-per-iteration DAG. |
| Add a machine-interval first pass before arbitrary precision | performance | RealLib `MachineEstimate*` | Hyper bounded/f64 filters | subsumed | Hyper already uses cheap exact/primitive/`f64` filters before dyadic refinement. RealLib's packed SSE2 version changes global rounding mode, violates aliasing assumptions, and contains suspect/incorrect edge handling, so it is not a safe implementation donor. |
| Add a fixed double-double tier | performance | RealLib manual section 6 and paper section 6.3 | Hyper approximation dispatch | pending only if profiles justify it | Hyper already has exact integer/rational and bounded-binary64 shortcuts. A new tier adds code/binary size and must beat the current direct jump to `BigInt` on a demonstrated precision band. |
| Replace exact dyadic bounds with midpoint plus compact floating error | performance / memory | RealLib `ErrorEstimate`, `RealEstimate` | Hyper approximation representation | rejected | It would weaken Hyper's straightforward integer enclosure invariant and relies on fragile legacy exponent arithmetic; lower priority gains cannot justify exactness risk. |
| Reuse RealLib's custom floating-FFT multiplication | performance | RealLib `convolution.cpp`, `LongFloat.cpp` | Hyper big-number backend | rejected | The code assumes empirically safe floating rounding and has unchecked sizing/overflow paths. Hyper's exact integer backend and exact NTT work are stronger. |
| Port RealLib transcendental kernels directly | performance | RealLib `kernels.cpp` / `RealFuncs.cpp` | Hyper elementary functions | pending algorithm-by-algorithm comparison | Relevant ideas are Newton precision staging, exp strength reduction/repeated squaring, base-3 sine reduction/triple-angle reconstruction, and Borwein reciprocal-pi. Keep only a rigorously enclosed algorithm that wins representative precision bands. |
| Retry only a precision-sensitive subcomputation | performance / completeness | iRRAM `limit_templates.h`, `limits.cc`, paper sections 7 and 10 | `hyperlimit` and `hyperlattice` refinement schedulers | subsumed | Hyper's immutable nodes request and cache each child at independently chosen precision; quotient and sign kernels already tighten just the sensitive operand/query. No outer whole-program work is discarded, so importing an exception/restart layer would add state and code without recovering work. |
| Repair propagated enclosure error from a certified Lipschitz bound | exactness / performance | iRRAM `limit_lip`, `lipschitz`, paper sections 10.2 and 11 | limit/function adapters in `hyperlimit` and scalar kernels | rejected for the scalar core | The technique repairs dependency/error accumulation in iRRAM's approximate-value execution. Hyper reevaluates an immutable exact expression directly at requested dyadic precision and does not carry an input ball through arbitrary user code. An unchecked Lipschitz promise would weaken exactness; a checked specialized kernel can already choose its own child precision. |
| Replay partial or multivalued branch choices across refinement | completeness | iRRAM typed caches, `LAZY_BOOLEAN`, `limit_mv`; paper sections 3, 9, 10.3 | `hyperlimit`, `hypertri`, `hypersolve` decision/certificate layers | subsumed by stronger identities | `hyperlimit::PredicateOutcome`/certainties, `hypertri::PredicateEvaluator`, and `hypersolve` residual/certificate replay attach decisions to explicit values and proofs. iRRAM's execution-order cache is fragile under control-flow changes and would be a semantic regression. |
| Pool arbitrary-precision temporaries per thread | performance / memory | iRRAM MPFR/GMP wrappers; IC-Reals emission scratch values | `hyperreal` big-integer hot paths | profile-gated | Reuse can reduce allocator traffic, but iRRAM retains as many as 1,000 high-precision objects of each type per thread. IC goes further in the unsafe direction with process-global GMP scratch registers, making otherwise independent evaluation non-reentrant and non-thread-safe. Any Hyper experiment needs allocator counts, peak/steady RSS, and thread-exit behavior; prefer scoped scratch storage. |
| Use grouped Taylor terms plus aggressive argument reduction for elementary functions | performance | iRRAM `exp_log.cc`, `sin_cos.cc`; history and paper | `hyperreal` transcendental kernels | pending algorithm comparison | Four-term grouping, repeated squaring, divide-by-three/triple-angle reconstruction, and a low/high-precision log split are concrete ideas. Hyper's existing certified kernels may already use stronger binary splitting or rectangular splitting. |
| Carry algebraic degree/height separation metadata with generated exact values | exactness / completeness | iRRAM `examples/algebraic-BFMS.cc` | symbolic facts in `hyperreal` / algebraic roots in `hypersolve` | subsumed | Hyper already derives on-demand algebraic-norm separation bounds during computable sign refinement, with explicit node/generator/degree budgets, and `hypersolve` isolates algebraic roots. iRRAM's eager provisional integers can overflow and are strictly weaker. |
| Keep the certified best partial Taylor/Picard iterate | exactness / performance | iRRAM-bigsteps `FUNCTIONAL_taylor_sum`, `SOLVER_PICARD` | `hyperreal` series kernels | rejected as unnecessary | Bigsteps minimizes propagated approximation error plus analytic tail because iRRAM arithmetic error may eventually dominate. Hyper's reduced-argument series use fixed-integer arithmetic with an up-front cumulative-rounding budget and stop while terms decrease monotonically; retaining prior sums would clone large integers without tightening the proven result. |
| Memoize only demanded power-series coefficients and sparse convolution supports | performance / memory | iRRAM-bigsteps IVP solvers and flow iterators | future IVP support; `hypersolve` sparse polynomial traversal | pending narrow comparison | The recursive `a[component][power][coefficient]` validity table avoids eager coefficient generation, and each monomial stores only nonzero variable indices. This is compelling for exact polynomial ODEs, but the current Hyper scalar has no IVP object. Check whether `hypersolve` already has equivalent sparse support metadata before retaining any local change. |
| Merge branches with a continuous conditional while a semidecision is unresolved | completeness | iRRAM+ `conditional.h`; IC-Reals `Alt`/lazy booleans | possible future `hyperlimit` selection combinator | rejected without a matching semidecision contract | The enclosure hull of both branches is sound only when the condition is a replayable/refinable semidecision and unresolved inputs force the branch values to converge. `PredicateOutcome::Unknown` is an explicit terminal query result, not such an object. IC's overlapping guarded alternatives provide no stronger portable proof contract and destructively redirect the chosen graph. Capturing arbitrary predicates inside `Real` nodes would enlarge the scalar representation and complicate serialization, synchronization, and cache identity. Reconsider only for a concrete higher-level continuous operation carrying the convergence proof. |
| Require a certified output rank/dimension for otherwise discontinuous subspace operations | completeness / exactness | iRRAM+ Grassmann paper and `GRASSMANN/main.cc` | `hypersolve` exact-rank reports and future Grassmann/projective APIs | subsumed at current boundaries | The mathematical contract is valuable: pivot selection and subspace meet/join become computable when the result dimension is supplied/proved. Hyper already reports exact coefficient and augmented rank, DOF, inconsistency, unsupported rows, and undecided minors rather than silently choosing a pivot. There is no current general Grassmann API to extend; preserve the contract when one is added. |
| Add a generic bound-described analytic/power-series scalar | completeness | iRRAM+ `ANALYTIC` and `POWERSERIES` | `hyperreal` computable nodes | rejected from this implementation | The truncation count does not account for the evaluation radius ratio, evaluation is routed through process-global static pointers, and continuation computes powers with `j^(j-1)` where the Taylor formula needs `j!`. A cubic probe returned `0.119791666667` after continuation instead of the direct exact value `0.125`. |
| Generate exact random reals as lazily extended random-bit streams | completeness | iRRAM+ `RANDOM` | no current Hyper scalar consumer | rejected as out of scope | Lazy prefix extension is coherent, but this implementation relies on iRRAM's execution-order replay cache and probabilistic termination of exact comparisons. Hyper currently exposes deterministic exact numbers, not a probability/random-real layer; adding one would be a separate API and reproducibility design project. |
| Integrate compact-domain functions using a supplied modulus of continuity | completeness | iRRAM+ `HAAR` | possible future exact integration API | rejected as a present transfer | A modulus is the right constructive contract, but the implementation performs exhaustive uniform Riemann sums, depends on an unpublished iRRAM branch, and has exponential dimension/precision cost. Hyper has no generic function/integration object to receive it. |
| Discover a local/global modulus by validated interval evaluation and compact-domain coverage | completeness | iRRAMx generated historical `euclidean.hpp`, `Path`, and `Surface` snapshots | possible future `hyperlimit` function-space or integration API | rejected as a present transfer | Inflating an input enclosure until interval evaluation attains the requested output width is a useful way to turn an executable continuous function into local modulus evidence; recursive finite coverage can globalize it on a compact domain. Hyper presently has no generic enclosed-function object, so there is nowhere to state or verify this contract. The archived implementation also writes only one coordinate in `IR_origin`, computes positive subdivision signs as `3, 7, ...` on axes above zero, relies on execution-order replay, and has exponential grids. It cannot be copied safely. |
| Preserve the identity and ordering of multivalued polynomial roots across refinement | completeness / exactness | iRRAMx `polynomialroot`, component tree, and `limit_mv_` | `hypersolve` algebraic-root and selected-fiber representations | subsumed by stronger certified identities | iRRAMx matches newly refined complex root centers to cached prior rational centers. `hypersolve` instead retains the exact defining polynomial, a validated unit isolator, its ordinal in the Sturm report, exact candidate replay, and local-field selected-fiber counts. That identity is proof based and independent of evaluation order. Importing nearest-center replay would weaken exactness. |
| Isolate all complex polynomial roots with Graeffe/Pellet components and Newton cluster acceleration | completeness / performance | iRRAMx complex polynomial subsystem | no current real-scalar or complex-root target | rejected as a present transfer | The high-level algorithm is relevant to a future certified complex algebraic package, but Hyper's present algebraic closure is explicitly real. This implementation has an uninitialized/incorrect coefficient-error scan, uninitialized default degree, quadratic-time evaluation, incomplete component initialization/merging, malformed Newton boundary expressions, empty-vector dereferences, and a reproduced constant-polynomial crash. It is neither a safe donor nor a justified addition to the current scalar architecture. |
| Use Strassen multiplication for exact-real matrices | performance / code size / memory | iRRAMx linear algebra | `hypersolve` matrix kernels | rejected | The implementation copies and pads dense matrices recursively, increases peak storage and code, has no crossover measurements, and cannot be called through its public header because declaration and definition have different signatures. Hyper's important solver matrices also exploit exact-rational, fraction-free, and sparse structure that this generic dense algorithm discards. |
| Stream balanced signed digits through persistent shared chunks and pack 29 digits per machine word | performance / memory | IC-Reals `DigsX`, digit handling, and forcing runtime | `hyperreal::computable` approximation cache | rejected | IC's consumers can advance independently through a shared immutable prefix, but Hyper's single finest cached dyadic `BigInt` already shares all coarser information while producing a requested precision in one batch. Packed and unpacked IC builds were byte-identical, yet at 20,000 decimal digits differed by at most 0.01 s and roughly noise-level RSS; packing improved artificial cold 20–64-bit object microbenchmarks by only 1.3–6.8%, while adding 2,367 bytes to text/data/BSS. A second scalar representation would increase code and conversion cost without an end-to-end win. |
| Destructively reduce homographic/bihomographic transforms as input digits arrive | performance / memory | IC-Reals `MatX`, `TenXY`, sign/digit emitters | `hyperreal::computable` DAG and arithmetic kernels | rejected | In-place residual updates can keep a streaming evaluator compact, but destroy replayable expression state, frustrate concurrent consumers, and require bespoke scheduling for every transform. Hyper's immutable `Arc<Node>` graph, exact rational leaves, structural rewrites, and cached batched approximations preserve sharing and thread safety. IC's implementation also leaks the graph and reproduced pointer corruption, so it is not a safe donor. |
| Choose the next operand and requested precision from a local epsilon-delta sensitivity estimate | performance / memory | IC-Reals `epsDel.c`, matrix/tensor strategies | binary `hyperreal::computable` kernels | subsumed for the demonstrated scalar paths | The useful principle is already present in stronger form for Hyper multiplication: it evaluates the unknown-magnitude operand first, measures the actual most-significant bit, then requests only the precision needed from the other operand. Addition has symmetric sensitivity, while division and elementary kernels already stage denominator/range information before expensive refinement. IC's fallback overlap heuristic adds mutable stream state without demonstrated benefit. |
| Fuse a digit producer directly into its consuming transform | performance / memory / code size | IC-Reals `log_R.c` and `pi.c` | constants and fused/specialized Hyper kernels | subsumed | IC avoids allocating some intermediate matrix/closure nodes for its logarithm and pi streams. Hyper already evaluates shared constants and common compositions through direct certified kernels, fused nodes, exact rational shortcuts, and structural normalization rather than materializing per-digit producers. No remaining intermediate allocation was identified on the compared paths. |
| Normalize transform coefficients only by a common power of two | performance | IC-Reals vector/matrix/tensor normalization | exact rational and fixed-point dyadic boundaries | subsumed | This cheaply preserves a streaming LFT without full gcd reduction. Hyper's hot approximation values are already dyadic integers with implicit powers of two, so no gcd exists to remove there; exact `Rational` boundaries remain canonical, and specialized dyadic scaling avoids general normalization. Importing noncanonical rational state would weaken equality/hash invariants for no measured gain. |
| Offer a total floor-or-ceiling integer choice | completeness / performance | XRC `xr_near_int` contract | `Computable::near_integer`, `Real::near_integer` | retained | Two guard bits suffice: an approximation within one unit of `4x`, rounded after division by four, lies within `3/4` of `x` and is therefore floor or ceiling; at an exact integer it must be that integer. Hyper's implementation also uses exact rational floor directly. It handles an unrecognized `sin(sqrt(2))^2 + cos(sqrt(2))^2 = 1` boundary where directional floor exhausts. Two 100-sample Criterion runs put cold `sqrt(2)` near-choice at 723--775 ns versus 3.98--4.11 us for certified floor (about 5.3--5.5x faster); rational choice remains about 78 ns. XRC's own loop is not copied: it loses negative signs and hangs on exact integers. |
| Cache only the finest scaled-integer approximation and derive coarser requests by shifting | performance / memory | XRC `_eval` high-water cache | `hyperreal::computable::ApproximationCache` | subsumed by a safer implementation | Hyper already keeps exactly one finest result per immutable node, derives coarser values by signed rounded scaling, publishes only finer results under a synchronized lazy cell, and frees it with the `Arc<Node>`. XRC's cache made a five-step shared logistic DAG about 40x faster on its first 1,000-bit evaluation and reduced repeated requests to single-digit nanoseconds, but its unsynchronized global-granularity cache leaks permanently and lets a broken multiplication scheduler make results depend on query history. |
| Cache mutable precision iterators inside shared expression nodes | exactness / performance / memory | Boost.Real 2021 `real_data`, `const_precision_iterator`, and `real_operation` | `hyperreal::computable::ApproximationCache` | rejected; safer effect already subsumed | A warmed Boost.Real alias changes six of seven replayed interval levels, setting a precision cap through one copy changes the other, and ThreadSanitizer reports 51 races during two-thread refinement. Its apparently faster replay (median 355 us versus 1,469 us for a fresh four-level refinement) is therefore state consumption, not a referentially transparent cache hit. Hyper stores a synchronized single finest value per immutable node, derives coarser answers without mutating expression semantics, and passes concurrent-clone, cache-warming, and adversarial precision-order tests; matching Criterion cache hits take roughly 9--68 ns. |
| Rewrite a shared binary expression DAG by pointer-identity distribution and collection | performance / memory / code size | Boost.Real 2021 `real.hpp` optimization traversal | Hyper constructors, bounded structural analysis, and fused nodes | subsumed in stronger bounded forms; reference code rejected | Sharing immutable operands is valuable, but Boost.Real interleaves a large recursive rewrite engine with arithmetic overloads, recognizes only pointer identities, aliases mutable evaluation state, and leaves public conversion/overload paths ill-formed. A stripped minimal arithmetic program carries about 200 KiB of text/data/BSS and preprocesses to 98,689 lines. Hyper already uses immutable `Arc<Node>` sharing, cheap identities, bounded normal forms, exact rational/dyadic folds, squares, and fused linear combinations without importing this API or code-size burden. |
| Replace the established bigint/rational backend with arbitrary-radix vector limbs | performance / binary size / code size | Boost.Real 2021 `exact_number`, `integer_number`, and `real_rational` | Hyper `Rational` and approximation integers | rejected on exactness first | Small exhaustive and 512-bit `cpp_int` oracles find thousands of both-negative subtraction, comparison, remainder, and negative-zero failures; valid parsing executes out-of-bounds reads, Knuth division reads past a vector, and negative rationals lose their sign in mixed operations. Multiplication alone happened to match the sampled oracle, which is insufficient to justify replacing Hyper's canonical exact backend. |
| Bound undecidable comparisons with mutable per-value or process-global precision caps | completeness / exactness | Boost.Real 2021 `const_precision_iterator::global_maximum_precision` and comparison overloads | explicit bounded Hyper decision APIs | rejected | A cap is a useful caller policy only when stated at the query boundary. Boost.Real stores it in aliased mutable values, defaults to ten radix limbs, and turns overlapping equal finite decimals into exceptions; concurrent or unrelated users can affect one another. Hyper exposes explicit `*_until`/unknown outcomes while keeping scalar identity and caches independent of a process-global decision budget. |
| Allocate multiplication precision from operand magnitudes and specialize integer operands | performance / memory | XRC `_mul`, `_imul`, `_iadd`, `_isub`, `_divi` | Hyper arithmetic nodes and kernels | subsumed | Hyper's multiplication already consumes exact planning facts when available, otherwise evaluates the unknown-size operand first and uses its actual most-significant bit to request only the needed precision from the other operand. Exact rational folds, `Int`/`One`, binary `Offset`, rational scales, squares, and fused linear forms cover XRC's integer-node motivation more broadly. XRC's `mpz_popcount-1` masquerading as bit length and unsigned handling of negative integer scales are correctness regressions, not optimizations to port. |
| Make approximation granularity a mutable process-global parameter | performance / code size | XRC `_b`, manual | no Hyper target | rejected | The setting changes the meaning of every cached integer and is explicitly unsafe to change while values live; it is non-reentrant and adds precision quantization. Worse, several formulas are not invariant in `b`: the BBP pi digit count underproduces for `b>4`, while default-`b` multiplication is broken. Hyper's per-request bit precision and immutable cache contract are both simpler and exact. |
| Generate pi through floating BBP hexadecimal digit extraction | performance / code size | XRC `pi_c_kmb.c`, `_pih` | Hyper shared pi kernel | rejected on exactness | The helper uses machine floating-point modular sums without a rigorous rounding enclosure and computes `4*n/b` hexadecimal digits where the requested scale requires roughly `b*n/4`; sampled `b=5` and `b=8` cases return zero at higher precisions. Hyper's certified Chudnovsky kernel and shared constant cache are stronger. |
| Continue removable small-angle quotients through a certified local series without deciding equality | completeness / performance | Spigot `HoleFiller`, `PowPrefixWrapper`, and removable `sinc` construction | `Real::{sinc,sinc_pi,cosc}` and private computable kernels | retained | Hyper now admits private `SincSmall`/`CoscSmall` nodes only after `approx(-2)` certifies `|x| <= 3/4`; alternating Taylor tails and integral-form derivative bounds allocate a strict one-ulp error budget without testing whether `x=0`. The opaque identity `sin(e)^2+cos(e)^2-1` formerly returned `UnknownZero`; all three helpers now evaluate and format their limits. A 135-input, nine-precision MPFR oracle made 2,430 comparisons through -1,024 bits, serialization restores positive structural facts, abort signals remain attached, and known symbolic/rational paths are unchanged. Cold construction is about 6.0 us versus 199--588 us for the former failed paths (about 32--96x), shared construction about 0.12--0.14 us versus 3.07--3.13 us after failed refinement, while approximation plus `f64` export is about 12.4--12.6 us. |
| Race only the finitely many digit/exponent hypotheses compatible with an enclosure and preserve open/closed endpoint semantics | exactness / completeness | Spigot `Formatter`, `Rounder`, and exact ten-mode output layer | possible explicit certified formatting API | deferred for cross-reference, not applied to `Display` | The architecture is sounder than choosing a digit from one approximation, and 100,000 sampled base-format conversions plus 1,204,080 finite IEEE-lattice comparisons passed. However, exact-boundary formatting is necessarily partial: ordinary output of an opaque exact-one `sinc` did not emit in 20 s, while tentative output returned immediately. The binding's toward-zero integer conversion is wrong for negative magnitudes below one, and directed IEEE overflow ignores rounding direction. Hyper's current terminating `Display` has a different approximate-output contract; any exact formatter should be an explicit fallible/partial API and will be compared with later systems before design. |
| Refine the more sensitive side of a bilinear transform first | performance / memory | Spigot `Core2` tensor sensitivity and input-choice logic | Hyper binary/fused approximation kernels | subsumed | Hyper multiplication already evaluates the operand whose magnitude is needed, then allocates asymmetric child precision from observed or exact structural magnitude; division stages denominator bounds and fused linear/product forms avoid many binary intermediates. Spigot's mutable consumable streams cannot share or replay a computed prefix like Hyper's immutable node cache, and no uncovered scalar workload justified another scheduler. |
| Use nondirectional nested brackets when either neighboring discrete answer is valid | completeness / performance | Spigot `BracketingGenerator`, nonblocking sign, and tentative continued-fraction output | `near_integer`, dyadic intervals, and future streaming output | subsumed at current scalar boundaries | Hyper's retained `near_integer` already chooses floor or ceiling from one guarded approximation, and certified dyadic intervals expose both endpoints without resolving equality. Continued-fraction retractions and tentative digit markers require a streaming/revision protocol that no current Hyper API promises, so adding their state and code has no present consumer. |
| Invert a monotone exact function by certified range bracketing, derivative maxima, and branch racing | completeness | Spigot `MonotoneHelper`, `MonotoneInverter`, and `NewtonInverter` | inverse elementary kernels and possible future enclosed-function API | rejected as a generic present transfer | Spigot's trisection and derivative-bound Newton steps are useful contracts for a higher-order validated function object. Hyper currently has dedicated inverse-trig, inverse-hyperbolic, normal-quantile, and root kernels with their own range reduction and one-ulp proofs, but no serializable generic function/enclosure type on which callers can prove monotonicity or a Lipschitz maximum. Adding an unchecked callback contract would weaken exactness. |
| Add certified AGM, general hypergeometric, Lambert W, zeta, Bessel, and unrestricted gamma families | completeness | Spigot special-function sources and prefix wrappers | future Hyper scientific-function surface | deferred for algorithm-by-algorithm cross-reference | The general hypergeometric tail bound, AGM prefix repair, Stirling-residual restart, and branch-aware Lambert inversion are substantial completeness ideas; Hyper presently covers only selected gamma forms and lacks most of this surface. An independent 100-decimal mpmath sweep passed 28 of 30 sampled cases, while exact-boundary `W(-1/e)` and `zeta(-2)` failed to finish within 30 s (`W_{-1}(-1/e)` did return exact `-1` in 10.84 s). These algorithms merit comparison with AERN/Arb-adjacent and symbolic systems before accepting the considerable API, proof, and binary-size cost; the Spigot code itself is not copied. |
| Clone a value by restarting a consumable generator from its original source | performance / memory | Spigot `Source`/`Core`/`Generator` ownership model | Hyper `Arc<Node>` DAG | rejected | Every Spigot clone restarts evaluation rather than sharing a finest prefix. Hyper clones are cheap shared graph handles with synchronized monotone caches, support concurrent refinements, and release the graph when the last owner drops. Restart semantics would regress repeated-query time and duplicate retained state. |
| Retain an entire buffered input source so cloned iterators can restart at old positions | memory | Spigot 16-KiB buffered input and iterator cloning | parsing/input boundary | rejected | The parser keeps the backing source indefinitely and therefore cannot bound memory for long-lived or unbounded streams. Hyper parses owned finite text into canonical rationals/expressions and has no replaying stream consumer; importing this lifetime pattern would only add retention. |
| Replace the established bigint backend with Spigot's compact internal sign/magnitude integer | performance / binary size | Spigot optional internal bigint | Hyper exact integer backend | rejected on exactness, performance, and size | The internal backend is about 2.7x slower on upstream CTest, grows the stripped executable from 896,800 to 1,232,456 bytes, triggers signed-negation UB on `INT_MIN`, and shifts a negative value past its magnitude to zero instead of floor `-1`. After excluding that known case, a 12,000-case Boost `cpp_int` oracle still passed 178,984 checks through 4,096 bits, but the demonstrated defects and disabled Karatsuba make it strictly inferior. |
| Stratify exact values into two-sided, lower-only, upper-only, and non-rate approximation types | exactness / completeness | Ariadne `Real`, `LowerReal`, `UpperReal`, `NaiveReal` | possible `hyperlimit`/future enclosed-function API | deferred without a consumer | Monotonicity-aware return types are a sound constructive API pattern when every constructor preserves its proof. Ariadne's declarations are mostly unimplemented and its public positivity casts are unchecked; Hyper already preserves undecided outcomes explicitly. Adding four scalar families now would expand API and binary surface without closing a demonstrated user case. |
| Lower an immutable formula DAG to a reusable CSE instruction tape | performance / memory | Ariadne `Formula` and `Procedure` | `hyperreal::Computable` evaluator | rejected as a generic scalar transfer | Ariadne only deduplicates pointer-identical nodes, keeps every full-precision temporary alive, does not deduplicate constants, cannot instantiate its vector evaluator for precision-carrying approximation types, and has uninitialized metadata paths. Hyper's shared `Arc<Node>` caches already eliminate repeated work at shared nodes while permitting per-child precision, and hot aggregates use fused kernels. No superior workload or measurement justifies a second representation. |
| Normalize domains to a unit box and refine Taylor ranges by joint linear/quadratic terms | performance / completeness | Ariadne `ScaledFunctionPatch` and `TaylorModel::range` | future validated multivariate function layer | rejected as a present scalar transfer | Conditioning and completing squares can tighten bounded polynomial images, but Hyper has no generic validated-function object and its exact polynomial consumers already own domain-specific certificates. Ariadne's sweepers provably erase terms without charging error, derivative erases model uncertainty, singleton scaling divides by zero, and the shipped MP composition test crashes; the algorithm needs an independent proof/substrate before any later use. |
| Preserve partial-operation domains before applying symbolic identities | exactness | Ariadne `simplify(x/x)` and `simplify(exp(log(x)))` counterexamples | `hyperreal::Real` division/domain regression suite | retained in `bbd2e6f` as test-only defense | Ariadne rewrites `x/x` to one even at zero and `exp(log(x))` to `x` even for negative inputs. Hyper already validates a denominator before zero/identity folding and validates logarithm sign before symbolic collapse. New tests pin `0/0 -> DivideByZero` and unresolved opaque `x/x -> UnknownZero`; they change no production code, allocation, binary, or hot path. |
| Order or deduplicate symbolic exact expressions by running exact comparison | completeness / performance | Ariadne expression ordering/CSE | no Hyper target | rejected | Ariadne's ordering can call an unbounded exact-real decision merely to order constants, while recursive `std::set` collection adds potentially quadratic traversal and recognizes neither semantic nor bounded structural equivalence reliably. Hyper deliberately uses pointer identity and bounded structural normal forms without requiring arbitrary exact equality, preserving termination boundaries. |
| Separate numerical candidate generation from exact or interval certification | exactness / completeness | Ariadne nonlinear, simplex, interval-Newton, and Krawczyk solvers | `hypersolve` proposal reports and candidate/interval certification | subsumed by a stronger explicit boundary | The architectural principle is correct, but Ariadne sometimes returns the exploratory result after computing a validator, invokes empty proposal helpers, or labels failed proof as no solution. Hypersolve already marks lossy adapters as non-certificates, exact-replays every active row, exposes certified violation/unknown/domain failure, and keeps Krawczyk/alpha evidence separate. Focused unit and generated-property checks pass, so no duplicate API or code is added. |
| Refine approximate roots with interval Newton or a midpoint-preconditioned Krawczyk operator | completeness | Ariadne `IntervalNewtonSolver`, `KrawczykSolver`, and `FactoredKrawczykSolver` | `hypersolve::interval` | API principle subsumed; Hyper proof correction found later | Hypersolve exposes separate affine and quadratic reports, but the later AERN2 factorization experiment's independent oracle found an erroneous extra radius factor in the multivariate contraction bound. The dimensionless correction and exact two-root regression are retained as recorded below; the earlier existence of the API and passing tests did not establish this formula's soundness. Ariadne's defective Taylor substrate and generic recursion are not transferred. |
| Keep self-map displacement and dimensionless derivative contraction separate | exactness / completeness | AERN2 linear-factorization comparison and independent Hyper public-certificate oracle | Hypersolve multivariate quadratic Krawczyk proof | retained | Removes a second radius factor that falsely certified two roots as unique; 80 scaled cases and coupled unequal-radius exact oracles pass, as do 771-test debug/release gates. Paired public-proof timing improves about 6--16%, allocation demand falls, and the pinned stripped driver shrinks 1,624 bytes. No reference code is copied. |
| Reuse one exact elimination across inverse columns | performance / memory / binary size | AERN2 linear-factorization comparison | Hypersolve multivariate Krawczyk inverse | retained in `a20be23` | Augmented [A\|I] Gauss-Jordan preserves the existing certified pivot/fallback rules and reduces arithmetic operation count from O(n^4) to O(n^3). 196 exact matrix identities, algebraic/partial-pivot tests, 773-test debug/release gates and 18 paired public workloads qualify it. Larger proofs improve about 2--3x; allocation-byte demand falls up to about 58%, and three linked drivers shrink 8.7--12.1KB. Small approximately 1% timing tradeoffs are documented, not hidden. |
| Preserve one common positive scale during rational Bernstein subdivision | performance / memory | AERN2 RootsIntVector common-scale controls, independently rederived | Hypersolve Bernstein subdivision | retained in `4dfbb39` | Linear-storage integer adjacent sums and per-level shifts preserve all signs and exact midpoint zeros; public coefficient reports keep actual magnitudes and nonrational/unknown paths stay explicit. Consume parent controls and delay first quadratic conversion. Independent exact oracles, 778-test debug/release gates, 17 paired public workloads, Memcheck and peak-heap checks qualify roughly 2--12.5x deeper-workload gains and up to 91% lower cumulative allocation demand. Small timing/allocation regressions and 12.5KB linked-driver growth are recorded, not hidden. |
| Adapt an ODE step by bounding a flow box and retrying smaller dyadic steps | completeness / performance | Ariadne bounders and Taylor/Picard/series integrators | possible future validated-dynamics crate | rejected as a present transfer | Hyper has no generic ODE/function-enclosure consumer. The audited implementation reverses initial widening, can repeat the same half step indefinitely, uses stale ranges, and depends on unsound Taylor errors; its approximate RK4 weights and instance-state handling are also wrong. A future dynamics layer would need an independent proof and progress contract, not this code. |
| Bound uncertain-input flow error by analytic `dexp`/`psi` remainder functions | exactness / completeness | Ariadne inclusion integrator | possible future validated-dynamics crate | rejected | The formulas are meaningful only with removable limits, certified-positive denominators, owned function/input lifetimes, and a sound Taylor substrate. Here zero returns NaN or an unbounded interval, positivity is unchecked, factory products retain dangling references, and the piecewise route omits first-half error. It cannot improve a current Hyper scalar path. |
| Encode a dyadic paving as a root extent plus cyclic-axis path bits and recombine equal children | performance / memory | Ariadne `GridCell`, `BinaryTreeNode`, and `GridTreePaving` | `hypertri` spatial scheduling | rejected as an implementation transfer | The representation can make exact cell ancestry and Boolean set operations cheap, but Ariadne maps lattice indices and grid spacing through `double`, repeats the first-level coordinate bits when projecting deeper cells, and spends 24 bytes plus a separate allocation per tree node. Its collapse routine leaves dangling children and double-deletes them, while subtree cloning changes the represented set. Hypertri already keeps topology decisions exact, uses exact AABBs and opt-in BRIO median ordering, and explicitly requires a demonstrably cheap exact dyadic key before adopting grid hashing. A pointer paving would add an unrelated ownership-heavy subsystem without a current consumer. |
| Split whichever of parameter range or output cell is wider, while using cheap enclosures only to schedule exact certification | performance / completeness | Ariadne subdivision/affine/constraint pavers | `hypertri` and `hypersolve` work scheduling | principle subsumed; code rejected | Scheduling by uncertainty is sound only when every rejection remains certified and every split has a progress bound. Hypertri already uses structural facts solely as certificates/schedulers and routes irreversible predicates through Hyperlimit; Hypersolve separates proposals from exact replay. Ariadne's subdivision can recurse forever on insensitive or zero-width domains, one alternate paver blocks on stdin, another selects a constraint-space root for a state-space paving and falls through after recursive splitting, and several paths reinterpret exact boxes as mutable upper boxes. No new exactness or measured hot-path advantage remains to justify another scheduler. |
| Represent one-dimensional set unions as sorted, coalesced exact intervals with linear two-pointer operations | performance / memory | Ariadne `UnionOfIntervals` | possible `hyperlimit` interval-set utility | rejected | The standard representation is compact, but no current Hyper public algorithm needs a general interval-union object. More importantly, Ariadne's intersection advances the wrong iterator: a direct probe reports `intersects=true` while returning an empty intersection for `[2,3]` against `[0,1] U [2,4]`; measuring an empty union dereferences element zero and segfaults. The converting constructors also skip normalization. Importing this surface would add code without a demonstrated consumer or trustworthy implementation. |
| Compute both directed dot bounds under one hardware rounding mode by accumulating the negated lower sum | performance | Ariadne `profiling/profile_arithmetic.cpp` | Hyperlattice dot kernels / Hyperreal approximation | rejected as inapplicable | The algebraic sign trick can halve ambient rounding-mode changes for finite IEEE interval arithmetic, but Hyper's public dots first preserve exact rationals or immutable symbolic forms and its approximation contract is a one-unit exact-integer enclosure, not a pair of mutable binary64 endpoints. Ariadne's unregistered benchmark mutates process-global floating state, assumes compiler observance and finite normal intermediates, has no NaN/overflow/subnormal contract, and no longer compiles against its own tree. Adding an approximate vector tier would weaken exactness and enlarge code without improving an existing hot path. |
| Encode one-sided, validated, approximate, and positive arithmetic as a compile-time return-type matrix | exactness / completeness | Ariadne `experimental/prototypes/simple_numeric` and `clang_friend_mixin` | Hyperlimit result/certificate types | rejected as an implementation transfer | Keeping proof strength in result types is already represented at Hyper's decision boundary by explicit certified/unknown outcomes, while scalar values retain one exact semantic type. Ariadne's sketches contain contradictory overload return types, unchecked positivity, duplicate class definitions, syntax errors, and do not build. A large cross-product of scalar wrapper types would increase API, monomorphization, and binary size without a current consumer. |
| Exchange exact reals across runtimes through an effort-indexed centre/radius ABI | completeness | Ariadne experimental Kirk adapter | possible future foreign exact-real interface | deferred; implementation rejected | A narrow approximation callback ABI could eventually interoperate with other exact-real engines, but the adapter's effort and absolute-accuracy routes assert, deletion is intentionally unresolved, radius conversion narrows to binary64 and asserts outside normal exponents, and ownership/reference semantics are ambiguous. Hyper has no requested foreign-runtime consumer, so this incomplete bridge does not justify a new unsafe ABI. |
| Check every newly produced certificate contribution before terminating, and require monotone nonnegative progress | exactness / completeness | Ariadne reachability analyser, vector/inclusion evolvers, and PDE steppers | Hyperlimit/Hypersolve certificate and refinement loops | principle already subsumed; reference code rejected | Ariadne checks the accumulated reach set before each expansion but can return `true` when the final expansion empties the frontier without checking its newly added reach cells; two restriction calls also target `initial_cells` instead of the evolved set. Its finite-time partition uses nearest-integer rounding: a direct exact probe gives one lock step and a certified `-2/5` remainder for time `3/5`, while several evolvers neither cap the last step nor reject zero progress. Hyperlimit and Hypersolve already preserve unknown, bounded progress, and final exact replay rather than treating loop termination as proof, so no duplicate API or production code is warranted. |
| Select among several independently sound enclosure algorithms by approximate output volume and exponential backoff | performance | Ariadne differential-inclusion evolver | possible future validated-dynamics layer | deferred without a consumer; implementation rejected | Choosing the tightest validated result with a cheap approximate volume is a sound portfolio pattern because a rigorous reach result is retained even when a non-rigorous evolver wins. Hyper has no general differential-inclusion consumer, and its scalar kernels are fixed proof-producing algorithms rather than interchangeable enclosures. The audited scheduler also eventually shifts `1u` by an unbounded delay count, so its backoff state is not safe to transplant. |
| Turn validated uniform remainder into explicit independent `[-1,1]` parameters, then discard low-sensitivity directions | completeness / memory | Ariadne `Enclosure::uniform_error_recondition` and Kuhn reconditioning | possible future validated multivariate-function/dynamics layer | deferred; no present Hyper target | Reifying remainder as explicit noise symbols can preserve correlations through later evolution, and sensitivity-based parameter retention can bound model growth. Current Hyper values are scalar exact DAGs, while Hyperlattice retains exact coordinate relationships through fused forms; neither owns a validated Taylor-model state requiring this transformation. Ariadne's implementation additionally type-puns an error coefficient, loses variable-kind metadata during split/reconditioning, underflows when configured with zero blocks, and depends on the already-unsound Taylor error layer. |
| Key labelled transforms by their declared target and fold predicates over every component | exactness / completeness | Ariadne hybrid automata, spaces, and pavings | Hyperlattice/Hypertri labelled consumer boundaries | principle already subsumed; no code transfer | A transform whose outputs have semantic labels must canonicalize by the target label, and a result over a disjoint union must inspect every component. Ariadne instead fills reset outputs in caller-list order (a direct reordered reset maps expected `[22,11]` to `[11,22]`), defines asymmetric hybrid-space equality, returns after the first nonempty location in `inside`, and implements `separated` with an inverted existence condition that reports an overlap as separated and dereferences `end()` for an absent location. Hyper's fixed-arity kernels and downstream exact predicates already bind coordinates structurally and certify all relevant lanes; introducing a hybrid labelled-container layer would add API and binary surface without a scalar consumer. |
| Classify event crossings and carry parameter-dependent time functions through a validated hybrid evolution step | completeness | Ariadne `GeneralHybridEvolver` | possible future validated hybrid-dynamics layer | deferred without a consumer; implementation rejected | Transverse, convex/concave, grazing, urgent, permissive, and impact cases are usefully distinguished, and representing both evolution and finishing time as validated functions can avoid a blunt final-step overshoot. Hyper has no hybrid-dynamics object. This implementation still accepts nonpositive configuration, can form negative evolution times for already-past points, mutates a const event set, uses unchecked composition, drops unsupported cases into coarse sampling, and has no universal positive-progress guard; the approximate simulator demonstrably steps from 0.8 to 1.2 for a requested horizon of 1.0. The semantic taxonomy is recorded, but no current Hyper change or benchmarkable hot path results. |
| Present a runtime generic facade over monomorphic foreign-language bindings | code size / completeness | Ariadne Python `template_` registry and `Type[...]` constructors | possible future Hyper language binding | deferred without a consumer; implementation rejected | Mapping runtime type keys to already-bound concrete classes can make a Python API look generic without adding another scalar implementation. Hyper currently exposes a Rust API and has no requested Python/FFI surface, so there is no consumer or binary-size baseline. Ariadne's helper also captures every constructor callable by reference after its stack lifetime ends, duplicates large overload matrices, and its boundary layer silently reverses operations, violates stable hashing, and permits invalid indices to segfault. A future binding should generate owned callables and exhaustive boundary tests independently rather than transplant this machinery. |

| Normalize a negative base to its positive magnitude before the lazy logarithmic integer-power fallback | exactness / completeness | flatsurf product/exponent audit compared with Hyper's own power dispatch | `hyperreal::Real::exp_ln_powi` | retained: `8a74b42b9059c1284aa7ca18adf6cfd92e29a66b` | The eager rational budget can reject a moderate exponent even when the value's magnitude remains close to one. Hyper's fallback then took `ln(negative)` and panicked. Negating the folded base before the logarithm, then restoring odd/even parity, fixes the domain while preserving the existing lazy representation. Ninety-six independent MPFR checks over three public APIs and 128/256-bit approximation fail before the edit and pass afterward; the full all-feature/all-target test gate, Clippy, fuzz-target check, 24 doctests, and 12 selected Hyperlattice consumer tests pass. Forty-one paired ABBA batches show no sustained positive-path slowdown; positive allocation counts are unchanged and the stripped driver grows by 224 bytes. |

| Demand-sized guarded integer square-root seeds | performance / memory | realistic's smaller square-root seed exposed excess fixed-width work in Hyper | `computable::approximation::sqrt` | retained: `44ce87ed2feb6fbfcd493d649e1648fb667da39e` | Up to 96 requested significant bits, request a four-guard-bit integer radicand, take its integer root, and round once. The error is below 5/8 output ulp; larger seeds retain existing behavior. 334 directed-MPFR boundary checks and all core/downstream gates pass. Final paired median ratios are 0.386/0.787/0.701 at 16/64/96 bits, with fewer allocations and +96 stripped-driver bytes. The 140-bit control is about 2.4% slower; 97/128/1024-bit controls are effectively unchanged. The large small-request gains and memory reduction justify this explicitly recorded tradeoff. |

## Validation and benchmark log

| Date | Command / measurement | Result | Interpretation |
| --- | --- | --- | --- |
| 2026-09-03 | Initial repository-state inventory | complete | Dirty and ahead-of-origin Hyper trees recorded before any audit edit. |
| 2026-09-03 | RealLib `CCACHE_DISABLE=1 make Real.a` | pass | Current GCC builds the archived core after bypassing the environment's unwritable ccache. |
| 2026-09-03 | RealLib manual programs and all non-`complex` examples | build pass | The aggregate Makefile has a parallel dependency race; the old `complex.cpp` is ambiguous with modern `std::complex`. These are reference-project/toolchain issues, not Hyper failures. |
| 2026-09-03 | `empty`, `fragments`, `exp`, `expimpr`, and `fibonacci` smoke runs | pass | Outputs agree with bundled examples, including 120/300-digit identities. |
| 2026-09-03 | iRRAM `CCACHE_DISABLE=1 ./configure`; `CCACHE_DISABLE=1 make -j4`; `CCACHE_DISABLE=1 make check -j4` | configure/build pass; 7/7 tests pass | Modern compiler warnings identify a possibly uninitialized formatter result and unconditional recursion in `RATIONAL::operator=(int)`. The small suite does not cover interval, matrix, sparse-matrix, complex, or many edge paths. |
| 2026-09-03 | Focused iRRAM defect harness, modes 1-8; selected runs under timeout, GDB, and Valgrind | eight independent failures reproduced | Integer `RATIONAL` assignment recurses; interval hull/intersection are wrong; complex `asin(1/2)` crashes in multivalue cache replay; `acoth(2)` and division by `[0,1]` do not terminate; rectangular matrix scaling corrupts memory; `sparse_ones(2,3)` reads uninitialized data and mismatches `new[]`/`delete`; sparse subtraction returns `+3` for `0-[3]`. These validate that the passing upstream suite is not evidence for unsafe code transfer. |
| 2026-09-03 | iRRAM-bigsteps `ivp.cc` compile/link against audited iRRAM | pass with two compatibility macro aliases | The two public default snapshots are API-skewed (`iRRAM_REITERATE` and `actual_stack()` are absent from iRRAM master); no source reference was modified. SDL 1 development metadata is unavailable, so the two optional visualizers were source-audited but not built. |
| 2026-09-03 | iRRAM-bigsteps constant-flow and nonlinear Riccati smoke runs | pass with decimal-valued controls; Riccati completed 21 certified big steps | Exercises coefficient generation, sparse flow traversal, analytic step bounds, Taylor termination, Lipschitz recentering, and non-autonomous coefficient shifting. A focused constant-flow run also confirms that the `step_control_alg=1` path can choose an unclamped radius and fail to terminate when the evaluation point reaches/outgrows the certified disk. |
| 2026-09-03 | iRRAM+ component compilation against audited iRRAM | `ANALYTIC`/`POWERSERIES` syntax pass; `RANDOM` builds after forced `<cassert>` include; GAUSSELIM, HAAR, and tracked GRASSMANN source fail | Failures are attributable to stale/private APIs and source defects: obsolete FASTMATRIX fields/macros and undeclared identifiers, an unpublished `gen-modulus`/lambda-limit API, and an untracked `complement.h`. Optional components therefore cannot support performance claims. |
| 2026-09-03 | iRRAM+ `RANDOM/examples/plot.cc` with `src_official` | pass | Exercises lazy bit-prefix generation for real and complex samples; observed distributions are qualitatively consistent, but this is not statistical certification and the model has no current Hyper consumer. |
| 2026-09-03 | Focused iRRAM+ cubic analytic-continuation probe | fail: direct `x^3` at `1/2` = `0.125`; continuation via center `1/4` = `0.119791666667` | Independently reproduces the factorial-loop defect; the missing declared iRRAM integer-power definition had to be supplied only in the probe. |
| 2026-09-03 | Bundled iRRAM+ `GRASSMANN/a.out` through the ELF loader and audited iRRAM shared-library path | runs and prints two planes plus an intersection | The non-executable tracked binary has a stale hard-coded RUNPATH and does not establish reproducibility from tracked source, which is missing a header. It was treated only as an opaque historical artifact. |
| 2026-09-03 | iRRAM commit `6545f78` (the dependency pinned by iRRAMx) `autoreconf`, out-of-tree configure/build/install; then iRRAMx `make -j4` | pass with `CCACHE_DISABLE=1` | Establishes that the archived extension can be compiled against its declared iRRAM revision. Compiler diagnostics independently flag possibly uninitialized determinant pivot coordinates and polynomial error state. |
| 2026-09-03 | iRRAMx bundled test compilation and smoke runs | main, Grassmann, plot, both polynomial tests, and random tests pass; compact and linear tests do not compile | `test_compact` relies on an obsolete implicit `INTEGER` conversion. `test_linear` calls an obsolete two-argument eigen API and expects a different return type. The passing tests exercise only happy paths and do not cover the reproduced edge failures. |
| 2026-09-03 | Link a client against every object in the iRRAMx static archive | fail | Random-matrix and linear modules export duplicate `inner` and `transpose` symbols. The public `strassen(REALMATRIX, REALMATRIX)` declaration also differs from the defined const-reference overload and produces an undefined reference for ordinary callers. The archive is not a coherent linkable library. |
| 2026-09-03 | Focused iRRAMx matrix/polynomial/palette probe, including Valgrind | five defects reproduced; simple/repeated quadratic roots pass | Matrix exponent zero returns the input rather than identity; `Enorm` accesses row 2 of a 2x2 matrix; roots of the nonzero constant polynomial segfault; the palette destructor produces six mismatched `new[]`/`delete` reports. Root isolation returns `-1,+1` for `x^2-1` and two copies of `1` for `(x-1)^2`, confirming the crash is an uncovered boundary rather than total nonfunctionality. |
| 2026-09-03 | IC-Reals 6.3 pristine `make`, followed by isolated compatibility port under GCC 15.3.1 and GMP 6.3.0 | pristine build fails; compatibility port and test executables build | Beyond the environment's read-only ccache, the 2000-era source collides with C23 `bool`, includes removed private GMP headers/fields, truncates pointers into `unsigned`, defines globals in a public header, and contains several compile/link defects. The `/tmp` port changed only portability and independently obvious control-flow defects; the pinned source and Hyper repositories were untouched. |
| 2026-09-03 | IC-Reals patched-source GCC `-Wall -Wextra -Wpedantic -fsyntax-only` and original-source Cppcheck exhaustive C89 scan | 834 compiler warnings plus one dead-file error; numerous static findings confirmed | The compiler tally includes 159 implicit declarations, 46 pointer-to-integer conversions, 19 missing returns, and six format defects. Cppcheck independently found the always-true `sin_QInt`/`cos_QInt` overflow test, uninitialized test data, and missing returns; its `Alt` and `emitSign` uninitialized reports were manually shown to be false positives. |
| 2026-09-03 | IC-Reals independent 190-case, 40-decimal oracle grid | 159 pass; 11 understated printed radii; 13 crashes; six timeouts; one fatal error | Negative `asin`, `acos`, and `asinh` crash; several endpoints hang; `pow(2,2)` reaches a fatal Boolean-state error; and negative `log(exp(x))` cases crash. Decimal `+-` radii are rounded inward, so even correct-looking values do not always enclose the exact result. |
| 2026-09-03 | IC-Reals deterministic 816-case sweep at 20, 80, and 200 decimal digits | 732 pass; 72 crashes; 12 radius failures | Every sampled negative `asin`/`acos`/`asinh` case crashed. Eleven enclosures understated radius by at most 5%; `-16/-6` at 200 digits understated it by 5.02%. This is independent numerical evidence, not reliance on the print-and-inspect upstream tests. |
| 2026-09-03 | IC-Reals Clang ASan/UBSan probes | pointer corruption and null dereference reproduced | Negative inverse functions feed malformed/misaligned `DigsX *` values into `TenXY.c`; negative `log(exp(x))` reaches a null `MatX` dereference. Signed-negative left shifts were also caught and repaired only in the disposable compatibility tree so later measurements could proceed. |
| 2026-09-03 | IC-Reals explicit domain/boundary probes | severe failures reproduced | `sqrt(0)`, `sqrt(-1)`, and `sqrt(-4)` return the same shrinking near-zero stream; `log(0)` and `pow(0,0)` overflow the continuation stack; inverse-hyperbolic endpoints fail or hang; reciprocal zero fails only after forcing; and a positive rational written with two negative components is mishandled. Hyper's corresponding domain, sign, and endpoint tests pass. |
| 2026-09-03 | IC-Reals 1,000- and 10,000-decimal stress probes | representative 1,000-digit paths pass; most nominal 10,000-digit paths hit a built-in fatal limit | Pi, square root, exponential, sine, logarithm, and a trigonometric identity agree at 1,000 digits. The default `FORCE_DEC_UPPER_BOUND=10000` rejects many requests at its advertised boundary before completing, demonstrating a fixed implementation ceiling rather than an exact-real semantic limit. |
| 2026-09-03 | IC-Reals Valgrind `id(1/3)` at 20 digits | 234 blocks / 169,672 bytes remain at exit from 248 allocations | Only 14 frees occur; Valgrind reports definitely, indirectly, possibly lost, and still-reachable storage. The Makefile omits the unfinished collector, so destructive graph reduction does not translate into bounded memory. |
| 2026-09-03 | IC-Reals packed versus unpacked 20,000-decimal runs and nine-pair cold microbenchmarks | no high-precision win; 1.3–6.8% artificial cold-path gain; binary grows 2,367 bytes | Pi, sqrt, exp, sin, and log outputs are byte-identical. High-precision wall time and RSS are effectively unchanged. Packing helps tiny rational/matrix/tensor construction most, but Hyper already batches a whole requested fixed-point value, so the representation and added code were rejected. |
| 2026-09-03 | IC-Reals corrected `sin_QInt`/`cos_QInt` overflow predicate, 11 alternating cold-construction pairs | byte-identical output; median construction improves about 1.2% for sine and 3.1% for cosine | Unsigned conversion makes the original disjunction true for every 32-bit denominator, so it always constructs GMP temporaries. The intended integer route saves little and disappears beneath forced evaluation; this reference-local defect offers no Hyper transfer. |
| 2026-09-03 | Hyperreal `cargo test --all-targets --all-features` after IC comparison | pass | All 694 unit tests plus integration tests, examples, and bench binaries in test mode pass, including negative-domain, inverse-function, huge-argument, exact-identity, and thousand-digit coverage relevant to IC's failures. Generated benchmark-report changes were precisely reverted; `hyperreal` is clean. |
| 2026-09-03 | Hyperreal focused Criterion cold transcendental baseline | pass | At the selected precisions: exp p128 3.82–3.90 us, sqrt p128 631–641 ns, sin p96 1.565–1.580 us, asin p96 6.39–6.48 us, acos p96 5.80–5.87 us, atan p96 2.00–2.03 us, asinh p128 6.59–6.73 us, and atanh p128 102–103 ns. These are retained baselines; IC supplied no safe algorithm that justified a competing patch. |
| 2026-09-03 | XRC 1.2 GCC 15.3.1/GMP 6.3.0 build, upstream executable build, and smoke runs | core and principal tests build/run; aggregate Makefile incomplete | Pi, Graham, exp, harmonic, both logistic variants, Mueller, reciprocal, and miscellaneous programs run to completion. `poly_test.c` violates the Makefile's own C90 mode with `//` comments, and the implicit `example` link omits XRC/GMP. The tests print values but contain no numerical assertions. |
| 2026-09-03 | XRC C17 warning build and exhaustive C89 Cppcheck pass | numerous definite findings | Compiler/static diagnostics confirm the no-op/null-dereferencing `xr_free`, missing `xr_near_int` return, invalid format strings, unchecked allocation paths, leaking `realloc` assignments, dead conditions, and nonportable pointer formatting. The experimental `tryfast1.c` reaches into private GMP limbs and has additional ownership and sizing defects. |
| 2026-09-03 | XRC deterministic exact-rational invariant sweep over 9,108 cases | 850 strict approximation-contract failures; no process failures | Across `b={1,2,3,4,5,8}` and six precisions, add, subtract, multiply, integer multiply/divide, reciprocal, and integer powers fail. There are 337 exact one-unit misses despite the documented strict `<1` bound; the worst sampled error is 137,438,953,472 ulps because default-`b` multiplication returns zero for a power-of-two operand. Replacing only the erroneous population count with true bit length still leaves 700 failures. |
| 2026-09-03 | XRC exp/exp1/pi oracle sweep over 592 cases | exp and exp1 sampled cases pass; pi fails 9/64 | Every pi failure occurs at `b=5` or `b=8`; the digit-count conversion eventually returns zero, with worst sampled scaled error about `1.4e482`. Sampled `b<=4` pi requests through 5,000 precision units pass, but the floating BBP path has no certified rounding proof. |
| 2026-09-03 | XRC exact logistic cache-history oracle over 315 cases | 63 failures, all at default `b=1` | Starting at exact `43/64`, a coarse pre-query changes later 100-bit answers and both cold/warm results violate the exact rational oracle from iteration six onward. A disposable one-line true-bit-length correction eliminates all 315 sampled failures, isolating the history dependence to the multiplication scheduler defect rather than validating the remaining arithmetic. |
| 2026-09-03 | XRC domain, comparison, and multivalued-rounding probes | severe semantic failures reproduced | Default-`b` comparison claims `1*1 < 1/2` and `2*2 < 3`; `near_int(-7/3)` returns `+2` and hangs on zero/exact nonzero integers; a negative integer divisor aborts; reciprocal zero hangs; degree-zero and even negative roots trap in GMP. |
| 2026-09-03 | XRC Clang ASan/UBSan and Valgrind probes | undefined behavior/traps and leaks confirmed | UBSan catches `xr_free(NULL)` dereferencing null; ASan records GMP `SIGFPE` paths for zero-degree and even negative roots. A three-node multiply leaves 248 bytes in 12 blocks after `xr_free`; printing leaves 310 bytes in 13 blocks, including an extra 46-byte string leak. |
| 2026-09-03 | XRC recursive depth and cache/no-cache probes | stack overflow and exponential replay measured | Under an 8 MiB stack, 70,000 nested unary nodes evaluate but 80,000 crash; the 70,000-node run peaks near 16.6 MiB RSS. For a shared logistic DAG at `b=2`, caching cuts the five-iteration first evaluation from about 1.35 ms to 33.8 us and repeat hits to about 5.6 ns. With caching disabled, cost grows roughly sevenfold per iteration, reaches about 2.5 s at iteration nine, and times out at iteration ten; Hyper already retains the useful high-water cache without XRC's leaks/global state. |
| 2026-09-03 | Hyper `near_integer` 8,224-rational choices, irrational/huge/cache-history cases, and exact trig boundary | pass | Both `Real` and `Computable` always return the expected floor-or-ceiling pair over the rational grid; pi, negative pi, sqrt, exp, a 4,096-bit offset, warmed cache, and the exact unrecognized trigonometric integer boundary pass. |
| 2026-09-03 | Hyper focused Criterion `near_integer` comparison, two 100-sample runs | retained | Exact rational choice is about 78 ns, matching certified rational floor. Cold `sqrt(2)` choice measures 723--775 ns versus 3.98--4.11 us for directional certified floor, a stable roughly 5.3--5.5x advantage. |
| 2026-09-03 | Stripped release `readme_quickstart` at pre-transfer `HEAD` versus retained change | effectively size-neutral | Separate clean-target builds measure 1,379,064 versus 1,379,128 bytes on disk and 1,376,792 versus 1,376,804 loaded bytes: +64 file bytes and +12 loaded bytes (under 0.005%). No node/data representation changed, and the full layout test remains green. |
| 2026-09-03 | Hyper `cargo test --all-targets --all-features`, Clippy `-D warnings`, rustdoc tests, docs, fmt, and diff checks after XRC transfer | pass | All 694 library tests, integration/property/oracle/README/API-accounting tests, benchmark binaries in test mode, examples, 24 rustdoc tests, documentation, lint, and formatting pass. The public API guard explicitly classifies multivalued near-integer choice as having no like-for-like GMP/MPFR operation. |
| 2026-09-03 | Downstream `cargo test --all-targets --all-features` after XRC transfer | pass | `hyperlattice`, `hyperlimit`, `hypertri`, and `hypersolve` all pass with their complete feature sets and benchmark smoke targets. Test-generated `hypersolve` timing/trace rewrites were restored exactly; all four downstream worktrees are clean. `hypercurve` was deliberately excluded because concurrent user edits are present. |
| 2026-09-03 | Spigot pinned Release builds and upstream CTest under GMP 6.3.0 and its internal bigint | pass | Commit `4ef5af3408a890ce440833201c35cf511cdaa52d` configures and builds in both bigint modes. The single upstream `test.sh` test passes in 22.16 s with GMP and 59.27 s with the internal bigint. Halibut is unavailable, so generated local HTML/man documentation was not rebuilt; checked-in manual sources and the 768-line published HTML remain in the audit scope. |
| 2026-09-03 | Spigot modern warning build, Cppcheck, and PTY/Valgrind input probe | one definite uninitialized-descriptor defect reproduced | `FdReaderRaw`'s constructor parameter shadows its member and never initializes the stored `fd`; compiler/static review identifies the path, and Valgrind reports an uninitialized byte passed to `ioctl(fd)` during terminal setup. This is reference-local and rules out blind source reuse. |
| 2026-09-03 | Spigot internal-bigint sanitizers, independent Boost `cpp_int` oracle, size, and CTest timing | rejected backend | UBSan catches negation overflow for `INT_MIN`; a negative right shift beyond the magnitude returns zero rather than floor `-1`. Excluding that known semantic defect, 12,000 randomized cases still pass 178,984 arithmetic checks at 32--4,096 bits under ASan/UBSan. It is about 2.7x slower than GMP in CTest and enlarges the stripped executable by 335,656 bytes (896,800 to 1,232,456). |
| 2026-09-03 | Spigot Python 3.14 binding build and pytest | binding builds; 23 upstream tests pass with 18 resource warnings | The test helper accidentally generates both binary operands from `x`, so the advertised randomized binary coverage is correlated. A corrected disposable copy passes all 19 scalar/binding checks in 0.301 s; this validates sampled bindings, not the missed edge semantics below. |
| 2026-09-03 | Spigot exact-rational base-format/integer oracle, 10,000 cases x ten modes | 200,000 checks; 12 toward-zero integer failures | All 100,000 base conversions and every non-`RZ` integer conversion pass. For negative `|x|<1`, `to_int(RZ)` returns `-1` instead of zero (for example `-1/10`), because approximate sign classification selects floor; `fracpart_rz` and `fmod` inherit the defect. |
| 2026-09-03 | Spigot exact finite IEEE-lattice oracle over half, single, and double formats | 120,408 cases / 1,204,080 mode comparisons pass; directed overflow defect remains | Every sampled finite-lattice conversion, including representable boundaries and random codes, matches the independent exact oracle in all ten modes. Overflow always returns infinity: positive round-down/toward-zero and negative round-up/toward-zero should instead return the maximum finite value. |
| 2026-09-03 | Spigot 100-decimal mpmath special-function oracle and Valgrind gamma probe | 28/30 sampled cases pass; two exact-boundary timeouts | Gamma/lgamma, both Lambert-W branches, zeta, AGM, hypergeometric, and Bessel samples agree to roughly requested precision. `W(-1/e)` and `zeta(-2)` do not finish within 30 s; `W_{-1}(-1/e)` returns exact `-1` in 10.841 s. A representative `gamma(10/3)` run is Valgrind-clean. |
| 2026-09-03 | Spigot opaque removable-hole formatting probe | ordinary fixed output stalls; tentative output succeeds | `sinc(sin(e)^2+cos(e)^2-1)` has exact value one but ordinary ten-digit formatting emits nothing within 20 s because it cannot commit a boundary digit. `--tentative-test=30` returns `!1 (10^-30)` immediately at roughly 5,820 KiB RSS. The hole filler computes enclosing values, but exact digit commitment remains a separate partial operation. |
| 2026-09-03 | Hyper certified small-angle MPFR oracle, serialization/layout/abort tests, and decimal-output regression | pass | Seven boundary/zero inputs plus 128 deterministic random rationals, nine precisions from `p=4` through `p=-1024`, and both new kernels produce 2,430 results within one scaled-integer ulp of 256-guard-bit MPFR. The 60-node JSON/CBOR inventory round-trips, restored nodes retain nonzero structural facts, caller abort signals survive, pre-aborted behavior is unchanged, 40-byte `Approximation` and 56-byte `Node` layouts remain fixed, and the opaque limits format as `1.000000000000`, `1.000000000000`, and `0.500000000000`. |
| 2026-09-03 | Hyper pre-transfer/current release A/B harness and two 100-sample Criterion runs | retained performance improvement | For 1,000 cold constructions, old/new `sinc` is 573--579/5.997--6.035 us, `sinc_pi` 580--588/8.140--8.207 us, and `cosc` 198.7--200.5/6.145--6.322 us; old calls return `UnknownZero`, new calls return values. A warmed shared `sinc` falls from 3.07--3.13 us after failed refinement to 0.119--0.136 us, and new construction plus `f64` export is 12.41--12.59 us. Permanent Criterion rows measure 3.586 us, 5.783 us, and 3.740 us respectively; the second run's 1--2.3% shifts are treated as noise because no successful old counterpart exists. |
| 2026-09-03 | Stripped all-feature `readme_quickstart`, exact pre-Spigot snapshot versus final retained change | modest size cost; no scalar-layout growth | Identical release configurations measure 1,441,928 versus 1,445,696 bytes on disk (+3,768). ELF sections change from text/data/BSS 1,218,549/219,448/4,512 to 1,222,301/219,472/4,840 (+4,104 loaded bytes). The improvement closes a real completeness hole and is roughly 32--96x faster on that path, so the sub-4-KiB file cost is worthwhile. |
| 2026-09-03 | Hyper all-feature/all-target tests, strict Clippy, doctests, docs, fmt, and final focused rerun after Spigot transfer | pass | All 695 library tests, integration/property/oracle/README/API-accounting tests, benchmark binaries in test mode, examples, 24 doctests, documentation, lint, and formatting pass. The final added decimal assertions pass in a focused rerun. Test-generated report noise was restored rather than retained. |
| 2026-09-03 | Downstream all-feature/all-target tests after Spigot transfer | pass after expected fixture update | `hyperlattice`, `hyperlimit`, `hypertri`, and `hypersolve` pass their full unit/integration/benchmark-smoke/example matrices. Hyperlimit's deliberate private-node drift detector was extended from 58 to 60 nodes and strict Clippy passes. Generated Hypersolve timing/trace changes were restored; Hypercurve remains excluded because four concurrent user-edited files are present. |
| 2026-09-04 | Ariadne pinned GCC 15.3/GMP/MPFR Release build and labelled numeric/utility/algebra CTest runs | 14/14 numeric, 5/5 utility, and 8/8 algebra tests pass | Establishes a reproducible upstream smoke baseline. Disabled checkers, uncalled concept/factorization bodies, and the independent adversarial failures below show that the labels do not cover the advertised contracts. |
| 2026-09-04 | Ariadne focused numeric/algebra probes under Release, MPFR oracles, timeout, GDB, and Valgrind | numerous exactness, semantic, and memory defects reproduced | Highlights include unsound DP `cos`/`tan` enclosures over 200,000 inputs, capped/truncating exact dyadics, integer/rational/decimal boundary failures, 18 invalid `MultiIndex` reads, out-of-bounds vector/symmetric-matrix/differential writes, wrong first/second derivatives, a lost sparse Jacobian, and incomplete/incorrect solver routines. Detailed operands and dispositions are retained in the Ariadne record. |
| 2026-09-04 | Ariadne build of all nine registered function targets and `ctest -L function --output-on-failure` | 8/9 pass; `test_taylor_model` segfaults | The MP test first produces a malformed refinement polynomial, then crashes in `MultiIndex::operator=` through Horner compose. This is an upstream test failure in the untouched pinned mirror, not a speculative probe. |
| 2026-09-04 | Ariadne focused function Release/Valgrind/GDB/compile probes | fourteen observed failure modes confirmed; two suspicions retracted | Reproduced omitted affine intercept, wrong affine-model product, uninitialized scalar-to-vector procedure metadata, an unavailable precision-carrying vector evaluator, degree-zero polynomial overwrite, Chebyshev scalar-add no-op, truncated join, alias-mutating function copy, scaled-patch subtraction-as-addition, mismatched-size comparison invalid reads/crash, patch-plus-constant recursive stack overflow, two sweepers losing validated error, and derivative error erasure. Public negative Taylor power and sampled Chebyshev multiplication work, so those two source suspicions were explicitly withdrawn. |
| 2026-09-04 | Ariadne symbolic targets and `ctest --test-dir /tmp/ariadne-audit-build2 -L symbolic --output-on-failure` | 2/2 pass in 0.02 s | Both registered suites explicitly accept `simplify(x/x) -> 1`, omit the zero-domain case, and do not instantiate the stale predicate/conditional/iteration paths or mismatched vector shapes. |
| 2026-09-04 | Ariadne symbolic Release, debug/Valgrind, Clang ASan, and compile probes | domain erasure, memory faults, lifetime bugs, wrong values, and unusable APIs reproduced | `x/x` at zero and `exp(log(x))` at -1 lose their domains; mismatched vector expressions segfault; three-argument templates read uninitialized storage; returned iteration sequences use stack-after-scope and compute the wrong recurrence; public predicate, conditional, and hybrid-valuation paths fail compilation; swapped comparison `opposite` aborts through `bad_variant_access`. |
| 2026-09-04 | Hyperreal domain-ordering regressions and complete all-feature gate | pass; retained in `bbd2e6f` | The new denominator-before-identity test and all five opaque-value tests pass; `cargo test --all-targets --all-features` passes all 696 library tests plus every integration, example, and benchmark-smoke target; strict all-target Clippy, 24 doctests, documentation, formatting, and diff checks pass. Test-generated trace noise was removed exactly. The retained patch is assertions only, so it cannot alter runtime, allocation, release-binary, or hot-path measurements. |
| 2026-09-04 | Ariadne build of all six registered solver targets and `ctest --test-dir /tmp/ariadne-audit-build2 -L solvers --output-on-failure` | 6/6 pass in 2.05 s | Establishes a build/smoke baseline only. RK4 and simplex are not exercised, affine integration is disabled as incorrect, most nonlinear optimizers are unreachable after an unconditional return, and warning-only catches allow incomplete integrators to pass. |
| 2026-09-04 | Ariadne focused bounder, RK4, nonlinear, inclusion, and timeout probes | six independent contract failures reproduced | Widening factors one/two produce `[-2,2]`/`[-1,1]`; RK4 gives about 2.917 instead of 2.708333 for one unit step of `x'=x`; a later half-step object reuses an earlier quarter step; zero-step evolution times out; epsilon 0.1 does not admit 1.05 into `[0,1]`; a trivial singular equality throws; `dexp(0)` is NaN and both `psi` limits are unbounded; the deprecated constraint initializer is an observable no-op. |
| 2026-09-04 | Ariadne simplex exact oracle plus Hypersolve proposal-boundary unit/property tests | 1,520/1,520 LP classifications agree; both Hyper checks pass | The sampled one-row/two-variable unit-box oracle found no simplex false positive, false negative, indeterminate, or exception, so a suspected wrong-answer claim was withdrawn. Hypersolve's lossy-adapter unit test and generated active-row property test confirm that proposal output remains explicit uncertainty until exact replay. No production change survived, so there is no new hot path, allocation, binary, or benchmark delta to measure. |
| 2026-09-04 | Ariadne build of all nine registered geometry targets and `ctest --test-dir /tmp/ariadne-audit-build2 -L geometry --output-on-failure -j2` | 9/9 pass in 1.35 s | This establishes only the untouched Release smoke baseline. The suite omits destructive tree collapse, self-assignment, polymorphic subtree cloning, empty interval-union measure, contradictory union intersection, and projection beyond one coordinate level; focused probes below reproduce each relevant defect despite the green label. |
| 2026-09-04 | Ariadne focused geometry Release and Valgrind probes | four semantic failures and one memory-safety failure reproduced | `BinaryTreeNode::set` leaves both freed child pointers non-null and ordinary destruction exits 139; Valgrind reports invalid reads and double frees. Self-assignment silently turns a split tree into a disabled leaf. Cloning an empty half-cell yields a full enabled root (`source_size=0`, `clone_size=1`). `[2,3]` versus `[0,1] U [2,4]` reports overlap but returns `[]`, and empty-union measure exits 139. Projecting 2-D path `0110` onto coordinate one returns `11` rather than the required level-wise `10`. Layout probing measures 24-byte individually allocated tree nodes, 80-byte cells, and a 128-byte paving handle. |
| 2026-09-04 | Ariadne standalone profiling/prototype configure, syntax, and build checks | concept-only archive builds; arithmetic/Taylor profiles and three executable prototypes fail | The independent prototype CMake tree configures only with missing-project/minimum-version warnings. `clang_friend_mixin` fails on a duplicate `Float`, duplicate constructors, and invalid inherited calls; `simple_numeric` fails on missing traits/syntax and contradictory return-only overloads; `double_dispatching` has a broken relative include. The Taylor profile immediately fails on `using StringType;` and many stale API assumptions, while the arithmetic profile cannot name even its fallback `Int` against current headers. These files can supply design hypotheses but no reproducible performance claims. |
| 2026-09-04 | Ariadne build of all three registered I/O targets and exact-label `ctest --test-dir /tmp/ariadne-audit-build2 -L '^io$' --output-on-failure -j2` | 3/3 pass in 3.56 s | The first exploratory `-L io` invocation also matched the `function` label and reproduced the already-recorded Taylor-model crash; the anchored run is the authoritative I/O result. The tests exercise drawing smoke paths but not value copying, empty boundaries, later shapes after a zero-dimensional object, or 3-D loop bounds. |
| 2026-09-04 | Ariadne `Figure` copy probe in Release and under Valgrind | normal run aborts with double free; Valgrind exits 99 with invalid reads and duplicate deletes | `Figure` owns a raw `Data*`, defines a destructor, and accepts an implicit shallow copy. The two destructors free the same 496-byte object and nested allocation. This confirms the I/O ownership layer cannot supply a memory-saving value pattern. |
| 2026-09-04 | Ariadne build of all twelve registered dynamics targets and exact-label `ctest --test-dir /tmp/ariadne-audit-build2 -L '^dynamics$' --output-on-failure -j2` | 12/12 pass in 10.46 s wall / 20.57 s process | This is an untouched Release smoke baseline across enclosures, maps, vector fields, inclusions, finite/infinite reachability, safety, and PDEs. Several suites execute no success-path assertions, and the focused probes below contradict important advertised contracts. |
| 2026-09-04 | Ariadne exact time-partition probe | `time=3/5`, lock `1` yields one whole step and remainder bounds around `-0.4`; exact comparison says negative | `compute_time_steps` rounds instead of flooring. The finite-time reachability routines can therefore perform more than the requested number of full lock steps and pass a negative remainder onward, violating the time-horizon partition before any enclosure quality is considered. |
| 2026-09-04 | Ariadne focused 2-D PDE tensor-slot probe and Valgrind classification | expected time-zero Gaussian boundary `0.36787944117144228`, observed `-0.020420122421933418`; Valgrind reports zero heap errors | The last iteration writes index `Ntime` into a tensor whose last extent is `Ntime`. Unchecked row-major flattening aliases that index to the next spatial cell rather than leaving the allocation, silently overwriting its time-zero initial condition; this explains why both Release and Valgrind can look clean. |
| 2026-09-04 | Ariadne full default Release build, explicit `tests-cpp` target, and exact-label `ctest --test-dir /tmp/ariadne-audit-build2 -L '^hybrid$' --output-on-failure -j2` | complete build; 8/8 hybrid tests pass in 1.97 s wall / 2.66 s process | The first anchored CTest correctly reported all eight executables missing because the default build excludes them; after building the explicit aggregate target, the authoritative run is green. The tests are mostly smoke/plot checks and omit reordered resets, proper restrictions, symmetric space equality, absent paving locations, and all-location folds. |
| 2026-09-04 | Ariadne focused hybrid Release semantic probe | six independently observable contract defects reproduced | Restricting `(a|one,b|two)` to `{a}` throws an inverted-subset assertion; a one-location space equals a two-location space while the reverse comparison is false; a reordered reset expected to bind `[x',y']=[22,11]` evaluates as `[11,22]`; a mixed acyclic/cyclic auxiliary system throws a generic internal assertion rather than `AlgebraicLoopError`; an overlapping box reports `separated=true`; and a two-location paving reports `inside=true` after inspecting only its first, contained location while the second lies outside. |
| 2026-09-04 | Ariadne absent-location hybrid paving probe under Release and Valgrind | native exit 139; Valgrind finds three uninitialized/invalid-control errors ending in a null jump | `HybridGridTreePaving::separated` uses `iter != end || iter->second...`: an existing location is unconditionally “separated,” while an absent but valid location dereferences `end()`. This is a direct memory-safety failure in an untouched pinned build despite the green hybrid label. |
| 2026-09-04 | Ariadne fixed-step hybrid simulator horizon probe | four points at times 0, 0.4, 0.8, and `1.2000000000000002` for requested final time 1 | The approximate simulator checks the horizon only before each full RK4 step and never clamps the last step. The result confirms the source-level overshoot and provides no scheduling implementation suitable for Hyper. |
| 2026-09-04 | Ariadne exact-label `ctest --test-dir /tmp/ariadne-audit-build2 -L '^python$' --output-on-failure -j2` | 8/8 Python tests pass in 1.48 s wall / 2.88 s process | The suite mostly checks that types import and construct; `test_import.py` is only `assert True`, and it has no adversarial value, hash, lifetime, operator, or index assertions. |
| 2026-09-04 | Ariadne `/tmp/ariadne-python-binding-probe.py` semantic and invalid-index modes | four semantic contracts fail; invalid negative index exits 139 | `RoundedFloatDP` returns 1 for `max(1,2)` and 2 for `min(1,2)`; differential-vector `+`, `-`, and reverse `-` all return `[11,12]` for `[10,10]` and `[1,2]`; value-equal decimals/events produce two distinct pointer-address hashes; and `RationalVector([1,2])[-3]` segfaults after the valid `[-1]` access returns 2. Ten thousand generic constructor calls happen to survive despite the statically proven dangling callable capture. |
| 2026-09-04 | Ariadne explicit `tutorials` aggregate build and all seven tutorial/demonstration executables | build passes; 7/7 processes exit 0, hybrid tutorial in 24.35 s | Confirms the shipped C++ tutorial surface compiles and nominally runs against the untouched Release library. These are logger/plot demonstrations with no independent expected-value assertions, so they do not override the focused failures already recorded. |
| 2026-09-04 | Ariadne explicit `examples` aggregate build and all 22 public example executables under a 15-second per-process cap | build passes; 10 processes exit 0 and 12 active long-running reachability/noisy workloads time out; no crash, assertion, or other non-timeout failure | The bounded sweep covers continuous, discrete, noisy differential-inclusion, hybrid, and PDE examples without pretending a smoke timeout is a correctness failure. The programs are demonstrations/benchmark drivers rather than independent value or enclosure oracles, and add no scalar donor beyond already-audited core paths. |
| 2026-09-04 | Ariadne disabled experimental feature targets enabled only in `/tmp` and built individually | `control_validator`, `dynamic_game`, and `riccati` all fail to compile | The first two have extensive stale API/source errors in addition to line-level semantic defects; Riccati immediately includes the removed `numeric/float-user.hpp`. These dormant sketches provide no executable correctness or performance evidence. The pinned mirror and Hyper trees remain untouched. |
| 2026-09-04 | Ariadne experimental application target build and bounded executable sweep | 19/20 registered targets build; `laser` fails against removed APIs. Unregistered `SUTR21` builds, `power_converters` fails against removed APIs, and `QUAD20` lacks its included header. Of 20 buildable programs, 3 exit zero, 16 remain active to their individual 15-second caps, and `vanderpol-noisy` deterministically aborts with an uncaught `FlowTimeStepException`. | The broad application sweep supplies workload coverage rather than an independent exact-real oracle. `CVDP23` and `SPRE22` are two of the clean exits only because they return before computation; ARCH “verified” fields derive from approximate widths/areas. The abort reproduces alone, after 4.5 seconds, because the Taylor-Picard step exceeds its configured maximum error. No application or benchmark implementation justifies a Hyper transfer. |
| 2026-09-04 | Ariadne complete documentation build after the 49-file line audit | target exits zero and generates 1,787 HTML plus 1,024 LaTeX files; Doxygen 1.14.0 emits 602 warnings and two missing-snippet errors | The errors name absent `tutorials/symbolic_usage.cpp` and `tutorials/numeric_usage.cpp`; other diagnostics include obsolete configuration tags, invalid `a4wide`, a stale layout entry, unresolved references, and malformed section nesting. A successful generator exit is therefore only an artifact-creation smoke result, not validation of the documented numerical contracts. |
| 2026-09-04 | Ariadne support/metadata validation and exact manifest reconciliation | 809/809 regular parent files audited (183,414 lines); 4/4 parent gitlinks inventoried; `bash -n ariadne_filter` passes | Exact `git ls-files -s` reconciliation corrected the preceding checkpoint by +1 file/+391 lines. The 22 support files add 2,358 lines but no numerical implementation. Existing configure/build/test/doc runs exercise the relevant CMake; optional `actionlint` and Debian changelog validators are unavailable. The pinned mirror remains clean, and the current v2.5.3 project/release/installation/tutorial/publication pages add no transferable scalar mechanism. |
| 2026-09-04 | Boost.Real 2018 untouched and compatibility Release builds | untouched configure passes but vendored Catch fails on runtime `MINSIGSTKSZ`; with `CATCH_CONFIG_NO_POSIX_SIGNALS`, 19/19 targets build and 19/19 CTests pass in 0.02 s | The compatibility define disables only Catch's POSIX signal handler. It establishes nominal modern-compiler coverage without modifying the pinned reference, but the suite lacks independent oracles and omits the public failures reproduced below. |
| 2026-09-04 | Boost.Real 2018 header self-containment and two-translation-unit linkage probes | 5/11 direct includes fail; valid two-TU program fails to link with many duplicate definitions | Include cycles/missing standard declarations break `boundary`, `boundary_helper`, `interval`, `real_algorithm`, and `real_helpers` as standalone headers. Non-inline definitions in the advertised header-only surface violate the ODR across ordinary translation units. |
| 2026-09-04 | Boost.Real 2018 independent finite-decimal oracles plus Clang ASan/UBSan and Valgrind public-API probes | 32/174,243 terminal results exclude the exact value; 160/871,215 prefix intervals exclude it; crashes, wrong values, and deterministic leaks reproduced | Addition/subtraction mishandle zero versus negative fractions; batched and terminal-nine refinement fail; negative zero orders incorrectly; assignment can segfault or compute the wrong self-compound result; empty/default states crash. One hundred operation iterators leak 52,800 bytes/1,000 blocks, and 100 tree assignments leak 22,128 bytes/410 blocks. No production Hyper change is justified. |
| 2026-09-04 | flatsurf exact-real minimal FLINT 3.0.1 build and untouched C++ suite through checksum-verified Pixi 0.42.1 | build passes; 12/12 upstream test executables pass | Establishes the pinned release-mode baseline (`-DNDEBUG -O2 -g3`) in the declared `libexactreal-flint-30` environment. The same-implementation approximation checks and missing malformed-input/concurrency cases do not cover the independent failures below. |
| 2026-09-04 | flatsurf complete normal C++ suite under system Valgrind 3.27.1 | 12/12 executables pass; 182,883 assertions; zero reported memory errors or leaks | ELF test binaries were run directly with the pinned library search path. This correctly instruments the tests themselves rather than only a libtool wrapper. Logs are `/tmp/flatsurf-memcheck-*.log`. |
| 2026-09-04 | flatsurf corrected sparse/cached/cold benchmark driver, seven repetitions per case, CPU 6 | rank-64 sparse-one square costs about 2.414 ms versus 400 ns after prior simplification; approximation costs 11.076 us versus 185 ns | The source-independent driver has result barriers and separates construction/refinement from cache hits. Its exact-one shape assertions show ranks 1/4/16/32/64 growing to 1/10/136/528/2080 on squaring, with coefficient storage alone reaching 66,560 bytes. Hashes stay stable and copied elements retain their original module through simplification. The combined shape sweep peaks at 2,116,641 live heap bytes plus 550,759 allocator overhead bytes under Massif. These are within-reference representation comparisons, not claimed Hyper speedups. |
| 2026-09-04 | Hyper signed lazy-power MPFR regression and full validation | 96/96 approximation checks pass after a one-operation production fix; before the fix the regression panics in `logarithms.rs` | Numerators +/-65,535 and +/-65,537 over 65,536, exponents +/-4,000 and +/-4,001, and `powi`, `powi_i64`, and `pow(Real)` all use the rejected-eager-power fallback and now agree with 512-bit MPFR to one output ulp at 128/256 bits. `cargo test --all-targets --all-features` passes including 701 library tests and every integration/example/benchmark smoke target; strict all-feature/all-target Clippy, fuzz compilation, 24 doctests, and Hyperlattice complex/matrix API tests also pass. Generated report-only noise was removed. |
| 2026-09-04 | flatsurf exact-real aggregate-header, exactness, malformed-input, crash, and system-Valgrind probes | five wrong/unsupported exact results, one malformed value, one zero-division abort, one aggregate-header compile failure, and 15 Valgrind memory errors reproduced | Rational Arb `!=` returns the equality answer for both equal and unequal operands; `floor(-1/2)` returns 0; release promotion changes one to zero; an integer module generated by -1 rejects its valid one; a rank-one element accepts zero coefficients; division by exact zero reaches an infinite/NaN floor and aborts because an `optional<bool>{false}` is tested for engagement; `gen(rank)` corrupts the heap, with invalid GMP reads/writes rooted at `module.cc:190`; and `exact-real.hpp` includes the removed `number_field_ideal.hpp`. These are defects and tri-state/API lessons, not transferable implementation wins. |

## iRRAM source-audit record

- Repository: `https://github.com/norbert-mueller/iRRAM`, commit
  `a4d2409b9591227f1bbba93557989d0a2ba29d26` (master, 2015-05-01).
- All 120 tracked files and 29,269 physical lines were inspected. This includes
  every implementation/header, all 40 examples, all seven tests, build and
  release metadata, the generated `aclocal.m4`, and both bundled documents.
  `doc/irram.ps` is a 35-page generated PostScript paper; it was converted via
  PostScript/PDF text extraction and all 1,852 extracted lines were read.
- Architecture: ordinary imperative C++ is rerun at increasingly fine global
  precision when an enclosure cannot decide a control-flow operation. Typed
  per-thread caches replay discrete inputs, outputs, conversions, and
  multivalued decisions by execution order. A limit can intercept reiteration
  locally; Lipschitz limits discard input approximation error while evaluating
  at the center, then restore a certified image-radius bound. Scalars begin as
  directed-rounding binary64 intervals and promote to MPFR values carrying a
  compact absolute-error summary. MPFR/GMP objects are pooled per thread.
- Correctness caveat: the snapshot's passing tests cover only a narrow subset.
  Source review found, among other issues, unconditional recursion in integer
  assignment to `RATIONAL`; wrong parity handling for negative rational powers;
  `INT_MIN` negation paths; unchecked precision/cache limits; uninitialized
  formatting state; incorrect `acoth`; multiple incorrect complex inverse
  functions; missed zero endpoints in interval division; invalid tangent hulls
  over poles; wrong interval hull/intersection operands; transposed dense-matrix
  loops for non-square matrices; mismatched array deletion; and severe sparse
  matrix construction, subtraction, and error-scan defects. None is a safe code
  donor; only independently rederived algorithms/contracts remain candidates.

## iRRAM-bigsteps completion record

- Repository: `https://github.com/fbrausse/iRRAM-bigsteps`, commit
  `0c6efb521f9529c270cb3aa34741694713b562bf` (`functional-irram`,
  2017-11-28).
- Every tracked artifact is textual and was read: 42 files and 9,730 physical
  lines. Coverage includes the three historical/current IVP solver variants,
  Lipschitz adapter, polynomial and Picard machinery, ring-buffer and SDL/Cairo
  visualizers, Makefiles, and every coefficient/start/description fixture.
- Architecture: polynomial ODEs are represented as sparse monomial streams.
  Power-series coefficients and powers are generated either eagerly by degree
  or recursively on demand into a three-dimensional memo table. Certified
  analytic radius/maximum bounds choose a large step; a Taylor evaluator tracks
  the sum's arithmetic error and a geometric Cauchy tail, retaining the partial
  sum with the smallest combined enclosure. Successive steps are recentered at
  exact dyadic times, and a Lipschitz wrapper prevents prior input uncertainty
  from being expanded syntactically through the next coefficient graph.
- The default snapshots require compatibility aliases to build together, but
  the solver compiles and its constant and nonlinear paths run. Source and a
  focused run expose an unsafe alternate step controller: `R_simple` is not
  clamped by the requested time radius, so a constant flow drives the evaluator
  to or beyond its certified convergence disk and can loop indefinitely. The
  unused multivariate-polynomial `operator*=` also multiplies sequentially by
  every term rather than distributing over a sum. Neither implementation is a
  code donor.

## iRRAM+ completion record

- Repository: `https://github.com/realcomputation/irramplus`, commit
  `13b7229af93e2a528153d85f439b4158378f6b14` (master, 2019-11-18).
- All 43 tracked artifacts were inspected. Every line of all 41 textual files
  (3,085 physical lines) was read across `ANALYTIC`, `GAUSSELIM`, `GRASSMANN`,
  `HAAR`, `RANDOM`, the continuous conditional, and the repository metadata.
  `GRASSMANN/GRASSMANN.pdf` is a six-page paper, SHA-256
  `a0db3566ca085be0221ae63864e38588940a012cd94edd4f08f4392ea18d821d`;
  all 273 extracted lines were read. `GRASSMANN/a.out` is a 64-bit x86 ELF,
  SHA-256
  `a208dfea4fce73dad4b3ea6e1a6f7ef818ea1c6170cd4a0cd558218c73344360`;
  its headers, linkage, strings, and runtime behavior were inspected without
  treating it as source coverage.
- The strongest semantic contribution is contractual. A conditional can make
  progress before its boolean resolves when both result branches converge, and
  rank-sensitive Grassmann operations become computable when their output
  dimension is supplied as discrete evidence. Hyper's current predicate values
  do not retain a replayable semidecision, so importing the first abstraction
  would be unsound. Hyper's exact affine-rank report already embodies the second
  pattern with stronger explicit certified/inconsistent/unsupported/undecided
  outcomes; there is no present Grassmann scalar API requiring a new change.
- The repository is an exploratory collection rather than a coherent build.
  The analytic classes use process-global pointers for limit callbacks and an
  insufficient truncation rule; analytic continuation uses `j^(j-1)` instead
  of `j!`, reproduced by the cubic probe. Gaussian/QR code contains stale APIs,
  swapped/invalid matrix bounds, an out-of-bounds Hessenberg step, undeclared
  temporaries, and unfinished pivot logic. HAAR requires an unmerged iRRAM
  branch; GRASSMANN source omits a tracked header; RANDOM omits `<cassert>` and
  relies on execution-order replay. No implementation is a safe code donor and
  no Hyper change survived this target.

## iRRAMx completion record

- Repository: `https://github.com/realcomputation/iRRAMx`, commit
  `1d33af09e005fdff8655e4b29fed3918f54f8d21` (master, 2021-10-29).
- All 68 tracked non-documentation artifacts were inspected, totaling 10,357
  physical lines. Every substantive line in the headers, implementations,
  tests, Makefile, README, and repository metadata was read. The 2,576-line
  `Doxyfile` is the stock Doxygen 1.8.20 template; every active setting was
  reviewed while its generated explanatory boilerplate was classified rather
  than treated as algorithm source. The 340 generated documentation assets
  comprise 211 HTML, 97 JavaScript, 26 PNG, three CSS, and three SVG files,
  totaling 1,729,903 bytes; their aggregate manifest SHA-256 is
  `e0a41b7841b92ecdfe197dc3d5fb9c2ebb59cba13029fd17eef45e89f1006778`.
- The generated pages are not merely duplicates of the checked-in headers.
  All 96 human-facing pages were text-extracted and read (6,440 lines, 16,043
  words; SHA-256
  `4c5243d74e3e55844b0315dd991fd8eaaac62de2e7e842d883c8d1d81aa99c02`),
  and all 22 historical source pages were separately extracted and read
  (4,783 lines; SHA-256
  `8dafa19c3cc0a6eb657c900b9864ef77c8cb8709e659f7ee9d8459d133f6b520`).
  The stale snapshot contains an older compact/path/surface design that
  searches for local moduli by evaluating functions over inflated input
  enclosures, then recursively covers a cube to obtain a global modulus.
- The current compact layer is only a finite-precision raster/membership
  prototype. It has array-deletion errors, unchecked coordinates and empty
  inputs, reference-capturing lifetime hazards, a broken black-on-white union,
  and a homotopy norm that takes the `N`th root instead of the square root.
  The archived compact snapshot has incorrect origin initialization and
  multidimensional subdivision signs, so its interesting modulus contract is
  not a code donor.
- The linear layer contains an out-of-bounds `Enorm`, off-by-one matrix power,
  mismatched `strassen` declaration/definition, potentially uninitialized
  determinant pivots, incomplete eigen machinery, nonstandard variable-length
  arrays, and duplicate public symbols when linked with the random layer. Its
  bundled legacy Gaussian-elimination subtree is byte-for-byte identical to
  the already rejected iRRAM+ version.
- The complex polynomial layer combines Graeffe/Pellet component isolation,
  Newton cluster acceleration, and choice replay to keep root order stable.
  Hyper's certified real-root identity is stronger, and there is no current
  complex-root API. The implementation's error scan repeatedly examines only
  coefficient zero and can return an uninitialized value, evaluation repeats
  powers quadratically, component merging is incomplete, several Newton
  formulas are malformed, empty result wrappers dereference element zero, and
  roots of a nonzero constant polynomial segfault. No implementation change or
  benchmark candidate survived this target.

## RealLib completion record

- Repository: `https://github.com/blambov/RealLib`, commit
  `b36b18ecd3d712311a12f384210735dfa319f86c` (master, 2015-04-05).
- Every tracked textual artifact was read: 54 files and 12,911 physical lines
  across the scalar implementation, approximation backends, kernels, examples,
  manual sources, build files, license, and legacy Visual C++ metadata.
- `manual.pdf` was identified, hashed
  (`2fd742228b72e524d3929946759330c5c3fb741b78be7097b9be64c831cb72c8`),
  extracted, and all 1,311 extracted lines were read. `Visual C++/real.suo` is a
  13,312-byte OLE compound binary rather than source; it was identified and
  hashed (`7c7c4badac0902327005c4b62fab031f1a1467725e1ab4282c108c3a28ce8db8`).
- The linked 17-page paper, *RealLib: An Efficient Implementation of Exact
  Real Arithmetic* (DOI `10.1017/S0960129506005822`), was independently
  obtained and all 788 extracted lines were read.
- Architecture: an intrusive-reference-counted expression DAG is evaluated
  bottom-up at one global precision. Only shared nodes cache approximations;
  per-pass reference counts release them after their final consumer. Large
  user computations can instead be compiled as templates over a fast machine
  interval and an arbitrary-precision midpoint/error type, preserving source
  control flow and locality without allocating a DAG node per operation.
- Correctness caveat: this snapshot is an idea source, not a trustworthy donor.
  The audit found wrong `delete`/`delete[]` pairs, self-assignment hazards,
  unchecked size/exponent arithmetic, global non-thread-safe state, an obvious
  `Real / double` implementation typo, integer edge-case undefined behavior,
  suspect SSE2 division/square-root paths, and external-libm assumptions the
  manual itself says prevent full accuracy verification.

## IC-Reals completion record

- Source: Imperial College Exact Real Arithmetic Library release 6.3 from
  `https://www.doc.ic.ac.uk/exact-computation/`. The release archive SHA-256 is
  `9f8ebab0b67a7c64126e85f3dc35b3bf0889432086c6fa6021b894faec67a2c1`.
  All 133 archive files and all 20,913 physical lines were inspected, including
  every top-level/header file, every base-runtime and math-library file, all 32
  tests, both TeX documents, build metadata, and every PostScript/EPS/XBM/XPM
  asset. The aggregate SHA-256 of the per-file content-hash manifest is
  `c054dedefb8dfb6f949112b8844aaeb07c74f3aaf280b7da60ee7ac8d6bcd1fe`.
- The separately published 16-page manual and one-page licence were downloaded,
  text-extracted, and read in full. Their SHA-256 values are respectively
  `cd9c5477e2311313a5930a9060583c81aa9783470e46ad215159060455b7195c`
  and `bd3df7a8c02bc893125c9de94405adddee76ba4d88f503ad40390b777dd3fab8`.
- Architecture: a real is a rational vector, unary linear-fractional transform,
  binary bilinear-fractional transform, sign/digit stream, closure, or guarded
  alternative over a one-point compactification. Forcing consumes balanced
  signed digits into mutable transform coefficients until a sign or output
  digit can be emitted. Epsilon-delta estimates select which child needs more
  information, while persistent digit chunks let consumers sit at different
  prefix positions. An explicit continuation stack trampolines evaluation;
  selected logarithm and pi producers feed transforms directly. Up to 29
  signed digits can be packed into one machine integer before GMP storage.
- The pristine source is not viable on a current 64-bit toolchain. It stores
  pointers and function pointers in 32-bit `unsigned` stack slots, depends on
  removed private GMP headers and representation fields, mutates GMP signs
  directly, conflicts with C23 `bool`, and defines `Pi`/`E` in a public header.
  An isolated `/tmp` compatibility port using `uintptr_t` and public GMP APIs
  built under GCC 15.3.1/GMP 6.3.0 and supported tests, sanitizers, Valgrind,
  and benchmarks. No reference or Hyper source was modified by that port.
- Manual review found additional definite defects: assignment in place of a
  type comparison, missing braces and switch breaks, a missing tensor-strategy
  equality return, uninitialized signedness tags, a misspelled Boolean macro,
  undefined negative left shifts, format/type errors, and missing non-void
  returns. The integer sine/cosine overflow disjunction is tautologically true
  because `0xC0000000` is unsigned. The Makefile omits the unfinished collector
  and several source files; the active evaluator intentionally never reclaims
  its expression graph and uses process-global GMP scratch objects.
- Independent oracle and sanitizer probes expose failures more important than
  portability: negative `asin`, `acos`, and `asinh` corrupt transform pointers;
  negative `log(exp(x))` dereferences a null matrix; several exact endpoints
  hang or overflow the continuation stack; rational square root returns the
  same near-zero stream for zero and negative radicands; `acos` lacks quadrant
  correction; denominator signs are not canonicalized; and printed decimal
  radii round inward. The 32 upstream tests contain no assertions and include
  their own uninitialized-input errors, so they do not certify these paths.
- The strongest representation experiment did not transfer. Packed digits and
  unpacked digits produced byte-identical results. At 20,000 decimals, five
  representative computations had essentially identical time and RSS; packing
  saved only 1.3--6.8% in artificial cold low-precision construction tests and
  enlarged text/data/BSS by 2,367 bytes. The corrected integer sine/cosine path
  saved only 1.2--3.1% during construction and vanished under evaluation cost.
  Hyper already shares one finest batched dyadic approximation, independently
  schedules magnitude-sensitive inputs, uses direct certified elementary
  kernels and exact rational shortcuts, and remains immutable and thread-safe.
  No IC-Reals-inspired implementation change survived exactness and benchmark
  review.

## XRC completion record

- Source: XRC release 1.2 from `https://keithbriggs.info/xrc.html`, whose
  project page was last modified 2024-01-21. The archived tarball SHA-256 is
  `c4b6d83264324a7714630f5628a4c73cb580c1f49ba7a9397896acc7f8e11af3`.
  All 29 files were inventoried and inspected. Every line of all 28 non-PDF
  files was read (4,498 physical lines), including 967 lines of scalar source,
  headers/wrappers, the BBP helper, tests/examples, Makefile, manual sources,
  the 480-line generated PostScript manual, and the 1,090-line generated graph
  asset. The aggregate SHA-256 of the per-file content-hash manifest is
  `3d1a463a94cd39ce06159e5c11b7b37750ed3b9e75baf9637d253c5c9e002edd`.
  The three-page PDF manual (SHA-256
  `7c4be175c186f1075b8d8038156a1c529da8bd579ac486bf977a35235e005e40`)
  was separately text-extracted and all 156 extracted lines were read.
- Architecture: each exact real is a heap-allocated recursive expression node
  with an operation tag, two untyped operands, and one GMP integer cache. A
  global granularity `b` defines the scale `2^(b*n)`; each node retains only its
  largest requested `n` and right-shifts that integer for coarser requests.
  Multiplication probes operand magnitudes to allocate asymmetric precision;
  integer add/subtract/multiply/divide nodes avoid generic binary graphs. Pi is
  streamed through floating BBP hexadecimal digits, exp uses Taylor after
  binary range reduction and repeated squaring, comparison semidecides strict
  order, and `near_int` is intended to make a total multivalued choice between
  floor and ceiling.
- The implementation does not satisfy its own strict scaled-error invariant.
  Its `_log2(mpz_t)` returns population count minus one rather than bit length,
  so powers of two appear to have logarithm zero and default-granularity
  multiplication can return exact zero. A 9,108-case rational oracle found 850
  violations across arithmetic operations; a disposable true-bit-length fix
  still left 700. A separate 315-case exact logistic test found 63 cache-history
  failures at `b=1`; the one-line fix removed those sampled history failures but
  does not repair reciprocal, signed scaling/division, addition, subtraction,
  power, or rounding errors. Pi's `4*n/b` hexadecimal-digit count has the scale
  conversion backwards and failed every sufficiently fine sampled `b=5`/`b=8`
  case, often by returning zero.
- Edge behavior is correspondingly unsafe. `near_int` drops a negative sign via
  `mpz_get_ui` and does not terminate at exact integers; comparison can reverse
  obvious inequalities; reciprocal zero hangs; root degree zero and even roots
  of negatives trap in GMP. `xr_free` is a no-op for every live object and
  dereferences null for `NULL`; rational construction and printing add further
  leaks. The C++ wrapper shallow-copies those unmanaged nodes. Recursive
  evaluation overflows an 8 MiB stack between 70,000 and 80,000 unary nodes.
  The mutable global granularity, global dump state, unchecked allocations,
  invalid format strings, and private-GMP `tryfast1.c` experiment further rule
  out direct code transfer.
- The useful caching/magnitude/specialization ideas are already present more
  safely in Hyper's immutable synchronized DAG, exact planning facts, rational
  folds, integer/one/dyadic nodes, squares, and fused linear forms. XRC caching
  was measurably essential on its shared logistic graph, but Hyper has the same
  benefit without permanent leaks or mutable scale semantics. Floating BBP,
  configurable granularity, recursive evaluation, and the experimental limb
  fusion were rejected.
- One semantic contract was retained without XRC code: `Computable::near_integer`
  and `Real::near_integer` return either floor or ceiling using two guard bits,
  with an exact-rational shortcut at the `Real` layer. The proof leaves a strict
  `3/4`-unit distance from the source value, so an exact integer boundary needs
  no equality decision. The implementation passed rational-grid, irrational,
  huge-magnitude, cache-history, exact-identity, full-suite, lint, and docs
  checks. Two Criterion runs show a cold irrational choice roughly 5.3--5.5
  times faster than certified directional floor, while the exact-rational path
  remains cost-neutral. This is the only XRC-inspired Hyper change retained.

## Spigot completion record

- Source: `https://git.tartarus.org/simon/spigot.git`, commit
  `4ef5af3408a890ce440833201c35cf511cdaa52d` (2026-08-17), corresponding to
  the published `spigot-20260818.4ef5af3.tar.gz` snapshot. All 77 tracked text
  files and all 33,912 physical lines were read, including every C++ source and
  header, both bigint backends, Python bindings, parser/formatter/calculator,
  tests, build metadata, generated parser, manual source, and historical notes.
  The aggregate SHA-256 of the per-file content-hash manifest is
  `10466e9002e3e98cba84ebeea2d297bbefe9ca71d9a490cc9a7cd80be88dd3eb`.
  The public landing page and all 768 lines of the separately rendered manual
  were also read.
- Architecture: values are consumable exact streams. A `Source` feeds a mutable
  `Core`, which transforms exact integer intervals through linear-fractional
  matrices; `Core2` generalizes this to a bilinear tensor and chooses which
  operand to refine from sensitivity. A `Generator` emits validated intervals,
  while `BracketingGenerator` deliberately widens a dyadic bracket to avoid
  committing to an exact boundary and `StaticGenerator` exposes forced and
  nonblocking sign/approximation queries. Values own their generators through
  unique pointers, and cloning reconstructs evaluation from the beginning
  rather than sharing an evaluated prefix.
- The higher layers contain several distinct constructive techniques.
  `MonotoneHelper` repairs endpoint images from monotonicity and slope/Lipschitz
  information; `MonotoneInverter` trisects while racing sign alternatives, and
  `NewtonInverter` uses a certified derivative maximum. `HoleFiller` encloses a
  removable singularity over a shrinking input region until the special point
  is excluded; AGM and power prefix wrappers use the same idea around unstable
  prefixes. Gamma restarts a Stirling residual when its ratio worsens, and the
  general hypergeometric evaluator proves a tail from a limiting term ratio.
  The formatter races at most two compatible exponent/digit hypotheses with
  open/closed endpoint rules across ten rounding modes, while continued
  fractions can emit tentative terms and retract them.
- The implementation is useful as an algorithmic reference but not a safe code
  donor. The terminal reader uses an uninitialized file descriptor; the
  optional bigint invokes undefined signed negation for `INT_MIN`, mishandles
  sufficiently large negative right shifts, is slower, and produces a larger
  binary than GMP. The Python randomized helper accidentally uses the first
  operand twice. Toward-zero integer conversion floors negative magnitudes
  below one, directed IEEE overflow always chooses infinity, and several exact
  special-function/output boundaries do not terminate. Buffered iterator
  clones retain the full backing input, ordinary clones restart expensive
  computations, and the single connected-interval model cannot preserve
  disconnected alternatives.
- One idea was rederived and retained in Hyper. `SincSmall` and `CoscSmall`
  evaluate the analytic continuations of `sin(x)/x` and
  `(1-cos(x))/x^2` after a coarse dyadic enclosure certifies the local domain,
  so they never need to decide whether an opaque argument is exactly zero.
  `sinc_pi` reuses the same kernel after multiplying by shared pi. Integral-form
  derivative bounds, decreasing alternating tails, fixed-point guard budgets,
  MPFR oracles, serialization/sign restoration, abort behavior, ordinary
  formatting, full downstream suites, Criterion, and ELF size measurements all
  passed. Existing rational and symbolic quotient reductions remain unchanged.
- Sensitivity-directed arithmetic, nondirectional integer/bracket selection,
  cached replay, and specialized inverse functions are already present more
  safely in Hyper's immutable shared DAG and explicit certification layers.
  Restarting clones, stream retention, generic deferred domain errors, and the
  internal bigint were rejected. Exact correctly rounded text remains a
  deliberately separate fallible/partial API candidate, and Spigot's broader
  special-function algorithms are deferred for comparison with later exact-real
  and symbolic systems rather than expanding Hyper from a single reference.

## Ariadne source-audit record (in progress)

- Source: `https://github.com/ariadne-cps/ariadne`, commit
  `a86839f37e7e7ccb7077fd6fddb9cc09f418a29b` (2025-07-03, version 2.5.3).
  The parent contains 813 tracked files. Its pinned `pybind11`,
  `betterthreads`, `conclog`, and `helper` submodules were initialized so the
  build could be checked, but vendored submodule bodies are not credited as
  parent-source coverage.
- The review began with the complete `source/numeric` and `tests/numeric` trees
  (107 files / 26,191 physical lines), their foundations, algebra, and utility
  dependencies, the validated function interfaces/models and solver machinery
  that can alter scalar evaluation or certification, relevant documentation,
  and rigorous-numerics tutorials. To satisfy the repository-wide file/line
  standard rather than infer irrelevance from directory names, every parent
  `source`/`tests` subsystem was closed in turn; the complete parent
  `source`, `tests`, and `python` text trees—including geometry, I/O,
  dynamics/PDE/reachability, hybrid, bindings, binding tests, Python examples,
  and Python tutorials—are now closed, while root tutorials/examples and
  remaining project assets remain in progress.
- Fully read so far: top-level `README.md` and `CMakeLists.txt`; all five files
  under `source/foundations`, the orphaned `tests/foundations/CMakeLists.txt`
  and `test_logical.cpp`; and numeric `CMakeLists.txt`, `accuracy.hpp`,
  `approximate_real.hpp`, `arithmetic.hpp`, `bits.hpp`, `builtin.hpp`,
  `casts.hpp`, `extended.hpp`, `field.hpp`, `lower_real.hpp`, `naive_real.hpp`,
  `positive.hpp`, `gmp.hpp`, `int.hpp`, `int.cpp`, `integer.hpp`,
  `integer.cpp`, `rational.hpp`, `rational.cpp`, `dyadic.hpp`, `dyadic.cpp`,
  `decimal.hpp`, `decimal.cpp`, `real_interface.hpp`,
  `real.hpp`, `real.cpp`, `reals.hpp`, `sequence.hpp`, `sign.hpp`,
  `twoexp.hpp`, `upper_real.hpp`, `validated_real.hpp`, `numeric.hpp`,
  `module.hpp`, and `tests/numeric/test_integer.cpp`, `test_rational.cpp`,
  `test_dyadic.cpp`, and `test_real.cpp`. Partial or
  truncated terminal output is never counted; uncertain files are reread in
  numbered bounded ranges before being credited here.
- Additional numeric files now fully read: `archetypes.hpp`, `concepts.hpp`,
  `concepts.cpp`, `float.decl.hpp`, `float_traits.hpp`, `rounding.hpp`,
  `float_operations.hpp`, `float-raw.hpp`, `flt64.hpp`, `float64-crtp.hpp`,
  `double.hpp`, `double.cpp`, `floatdp.hpp`, `floatdp.cpp`, `floatmp.hpp`,
  `floatmp.cpp`, all three `float_error` files, `float_factory.hpp`, both
  `float_literals` files, all three `float_approximation` files, all three
  `float_lower_bound` files, all three `float_upper_bound` files, all four
  `float_bounds` files, and all four `float_ball` files.
- The remaining numeric implementation is now fully read: `mpfr_array.hpp`,
  `mpfr_array.cpp`, `rounded_float.hpp`, `floats.hpp`, `complex.hpp`,
  `complex.cpp`, `number.decl.hpp`, `number_interface.hpp`, `number.cpp`,
  `lower_number.hpp`, `upper_number.hpp`, `number.hpp`, `number_wrapper.hpp`,
  `operators.hpp`, `operators.cpp`, and `operators.tpl.hpp`. This closes all
  89 files directly under `source/numeric`.
- The complete `tests/numeric` tree is also now read: in addition to the four
  tests listed above, this includes its `CMakeLists.txt`, `check_numeric.hpp`,
  disabled `check_numeric.cpp`, `test_floats.hpp`, and every float, bounds,
  ball, complex, rounding, rounded-float, approximation, directed-bound, and
  generic-number test. This closes the requested numeric implementation/test
  slice at 107 files / 26,191 physical lines; execution evidence is recorded
  separately below and the wider dependency/document scope remains open.
- The complete direct utility layer is now read as well: all 36 files / 4,584
  physical lines under `source/utility` and all six files / 440 lines under
  `tests/utility`. Together with the numeric slice this is 149 files / 31,215
  lines of closed implementation/test coverage. The five registered utility
  executables built and `ctest -L utility --output-on-failure` passed 5/5 in
  0.07 seconds. Building those tiny tests nevertheless pulled the entire
  `ariadne-kernel` dependency closure (I/O, numeric, algebra, functions,
  solvers, geometry, and symbolic code) and repeatedly emitted hidden-virtual
  warnings from the function-model hierarchy; that is concrete negative
  build-time/code-coupling evidence rather than a scalar donor.
- The complete direct algebra layer is now read: all 52 files / 16,320
  physical lines under `source/algebra` and all 11 files / 2,332 lines under
  `tests/algebra`. Together with the already closed numeric, utility, and
  directly relevant foundations slices, this brings credited Ariadne coverage
  to 219 files / 51,261 lines. All eight registered algebra executables built
  against the pinned Release mirror, and `ctest -L algebra
  --output-on-failure` passed 8/8 in 0.04 seconds. The result is only a smoke
  baseline: the excluded concept checker, uncalled test bodies, and
  adversarial failures below materially contradict broad correctness.
- The complete direct function layer is now read line by line: all 61 files /
  19,045 physical lines under `source/function` and all 12 files / 3,781 lines
  under `tests/function`, or 73 files / 22,826 lines. Source coverage includes
  every affine and affine-model file; calculus/domain/constraint/projection/
  scaling support; formula, procedure, polynomial, and Chebyshev
  implementations; every erased function interface, mixin, wrapper, model,
  patch, and traits file; measurable/multifunction and user-function layers;
  and the complete Taylor series, Taylor model, Taylor function, scaled patch,
  and Taylor multifunction stack. Test coverage includes both checker files,
  its CMake manifest, and all nine registered `test_*` translation units. This
  closes the fifth credited slice and brings Ariadne coverage to 292 files /
  74,087 lines; documentation and other directly supporting source remain in
  progress.
- The complete direct symbolic layer is now read line by line: 19 files / 5,162
  physical lines under `source/symbolic` and three files / 679 lines under
  `tests/symbolic`, or 22 files / 5,841 lines. The manifest is
  `CMakeLists.txt`, `assignment.hpp`, `constant.hpp`, `expression.cpp`,
  `expression.decl.hpp`, `expression.hpp`, `expression.tpl.hpp`,
  `expression_set.cpp`, `expression_set.hpp`, `function_expression.hpp`,
  `identifier.hpp`, `operations.hpp`, `predicate.hpp`, `space.cpp`,
  `space.hpp`, `templates.hpp`, `templates.tpl.hpp`, `valuation.hpp`, and
  `variable.hpp`, plus the test `CMakeLists.txt`, `test_expression.cpp`, and
  `test_expression_set.cpp`. This closes the sixth credited slice and brings
  Ariadne coverage to 314 files / 79,928 lines. Both registered symbolic test
  executables built and the labelled suite passed 2/2 in 0.02 seconds.
- The complete direct solver layer is now read line by line: all 21 files /
  10,474 physical lines under `source/solvers` and all seven files / 1,317
  lines under `tests/solvers`, or 28 files / 11,791 lines. The source manifest
  is `CMakeLists.txt`, both bounder files, `configuration_interface.hpp`, both
  constraint-solver files, both inclusion-integrator files, all three ordinary
  integrator files, both integrator-interface files, both linear-programming
  files, both nonlinear-programming files, both Runge--Kutta files,
  `simplex_algorithm.cpp`, and both general-solver files. The test manifest is
  its `CMakeLists.txt` plus `test_bounder.cpp`, `test_constraint_solver.cpp`,
  `test_integrator.cpp`, `test_linear_programming.cpp`,
  `test_nonlinear_programming.cpp`, and `test_solvers.cpp`. This closes the
  seventh credited slice and brings Ariadne coverage to 342 files / 91,719
  lines. All six registered targets built and `ctest -L solvers
  --output-on-failure` passed 6/6 in 2.05 seconds.
- A disposable full repository mirror configured successfully in Release mode
  with GCC 15.3, GMP, MPFR, packaged submodules, and `CCACHE_DISABLE=1`.
  `ariadne-core` and all 14 registered numeric executables built. A labelled
  `ctest -L numeric --output-on-failure` run completed 14/14 with exit status
  zero in 0.14 seconds, including integer, dyadic, rational, real, generic
  number, DP/MP float, directed bounds, interval, ball, rounded-mode, and
  complex tests. The clean result is recorded as upstream smoke evidence, not
  as validation of the omitted contracts itemized below. Configuration writes
  a generated header into the source tree, which is why all build and test work
  is being performed in the mirror.
- Initial scalar architecture: an immutable shared type-erased `RealInterface`
  graph computes directed dyadic or floating enclosures by effort. Every
  expression eagerly stores a double-precision enclosure, uses it through
  effort 52, then recursively evaluates MPFR bounds at approximately 16 bits
  per effort unit. `RealBase::compute(Accuracy)` increases effort until the
  radius meets the requested dyadic error; no reusable high-precision per-node
  cache has yet appeared.
- The type system separately exposes effective two-sided `Real`, monotone
  `LowerReal` and `UpperReal`, non-rate `NaiveReal`, finite validated bounds,
  approximation-only values, and positive refinements. Unsupported
  non-monotone operations are deleted from one-sided types, while sign-safe
  multiplication, reciprocal, elementary functions, and increasing/decreasing
  limits preserve the strongest available result type. This is a substantive
  completeness/API candidate, not yet a justified Hyper change.
- Exact comparison is expressed through lazy Sierpinski/Kleenean predicates;
  threshold-overlap permits a terminating nondeterministic greater-than choice,
  and `when` combines two upper-semidecidable branches by taking an enclosure
  hull until one alternative is refuted. The existing nearby-integer routine is
  conceptually covered by Hyper's retained `near_integer`. A suspicious
  `same(Real, Real)` implementation cross-compares lower to upper endpoints.
  A separately compiled probe confirmed `same(pi,pi)`, `same(copy_of_pi,pi)`,
  and `same(sqrt(2),sqrt(2))` all return false, while exact singleton integers
  pass accidentally. The upstream equality assertion is commented out and no
  active test covers this contract, so this is a confirmed reference defect.
- The foundations logical test directory is never added by the repository's
  top-level test CMake file, so `test_logical` has no generated build target.
  Compiling it manually against `libariadne-core` succeeds and all active
  checks pass, but its paradigm concept method is itself never called. The
  advertised one-sided and `NaiveReal` APIs are also only partly implemented:
  exported symbols cover basic lower/upper computation and a subset of
  arithmetic, while most declared monotone functions, limits, and all
  `NaiveReal` methods have no implementation. These gaps materially qualify
  the apparent API-completeness advantage.
- Positivity wrappers and `cast_positive` are unchecked public construction
  mechanisms, so the stronger return types encode an assumption rather than a
  runtime certificate. The distinction is still useful as a static internal
  architecture pattern, but cannot be copied into Hyper as a sound public
  proof boundary. The generic signed-power helper also negates a signed `Int`
  before conversion, which is undefined for the minimum integer and will be
  tested where instantiated.
- The exact integer substrate fails multiple adversarial Release checks despite
  its upstream suite passing: `Integer(Int32(-3))` becomes
  `18446744073709551613`, `INT64_MIN` becomes `-4294967296`, and `UINT64_MAX`
  becomes `8589934591`; `log2floor(Natural(0))` returns `0` despite documenting
  `-1`; invalid integer strings silently become zero; and separately selected
  floor quotient and ceiling remainder operations do not reconstruct signed
  dividends. The tests stop below the broken 64-bit ranges and omit these
  contracts.
- Rational adversarial checks found further independent defects. Halving
  `[2,4]` produces the reversed interval `[2:1]`; even powers of `[-2,-1]` and
  `[-1,2]` produce `[4:1]` and `[1:4]`; NaN is simultaneously less than and
  greater than zero because `INCOMPARABLE=-128` is fed into ordinary ordering;
  and formatting a 601-digit rational overflows a fixed 512-byte stack buffer
  and exits with signal 11. The full upstream rational test still passes
  because it contains no rational-bounds coverage, no NaN ordering assertions,
  and no large-output case. Signed rational powers also repeat the minimum-`Int`
  negation overflow. None of this code is a viable donor for Hyper's exact
  scalar substrate.
- `Dyadic` is an especially strong negative architectural result. Each finite
  value is a GMP `mpf_t` initialized at the hard-coded maximum precision of
  65,535 bits: an allocator probe measured approximately 8.22 KiB of heap
  reservation per scalar (82,240,032 bytes for 10,000 values), while an exact
  70,001-bit integer was silently truncated. A raw-GMP constructor mistakenly
  executes `mpf_set(_mpf,_mpf)` and turns seven into zero; the move constructor
  reinitializes its source at GMP's 64-bit default, so assigning a later
  1,000-bit value to that moved-from object silently truncates it; the named
  interval `neg([1,3])` returns the reversed `[-1,-3]`; and scientific output
  loops forever on zero. Infinity and NaN support mutates private GMP
  representation fields, interval/error constructors do not validate their
  invariants, and the documented exact-literal guard merely accepts a rounded
  `double` (the long-double literal also narrows to `double`). The complete
  upstream dyadic test exits successfully but covers none of these cases.
  Hyper's arbitrary-size integer dyadics, compact allocation behavior, checked
  interval construction, and terminating formatting must therefore be
  preserved; adopting this substrate would regress exactness first and memory
  and performance second.
- The decimal layer is not a sound input or comparison donor either. Its
  hand-written relational operators reverse both `<=` and `>` (`1<=2` and
  `2>1` are false, while `<` and `>=` happen to work). Empty, sign-only, and
  point-only strings all silently parse as zero; `Decimal(+infinity)` loops
  forever in decimal normalization; and `Decimal(NaN)` produced
  `-4.294967296` in the Release probe. Floating literals narrow `long double`
  to `double` and retain only a nine-significant-figure candidate, bounds and
  positive wrappers again lack a sound public invariant boundary, and ordinary
  constructors and multiplication retain redundant powers of ten. Addition
  and comparison cross-multiply by both full decimal denominators instead of
  aligning to the larger scale, inflating otherwise small exact operations.
  The passing upstream rational test contains several normal decimal examples
  but no malformed/nonfinite input or complete ordering-law coverage.
- Ariadne's machine enclosure tier is empirically unsound. A deterministic
  differential harness compared each directed elementary kernel against
  correctly rounded 53-bit MPFR for 200,000 inputs. `cos` violated its lower
  enclosure 46,981 times and its upper enclosure 47,066 times; `tan` failed
  34,295 and 34,168 times respectively. Representative errors were many
  thousands of ulps, not boundary-only discrepancies. The other sampled
  kernels enclosed their references over the tested domains, but `log(0)`
  returns positive rather than negative infinity, and `sin`/`cos` throw from
  internal series preconditions for inputs such as `1e20` and `1e100` after
  range reduction narrows an enormous quotient to a machine integer. These
  hand-written kernels feed `FloatDPBounds` and the eagerly stored enclosure
  on every real expression, so their speed cannot compensate for the lost
  exactness.
- Exact-to-machine conversion has separate completeness failures. Constructing
  a downward `FloatDP` bound for the finite exact value `2^2000` did not finish
  in two seconds: the conversion starts at `+inf` and repeatedly subtracts the
  smallest normal double, which remains `+inf`. Its advertised one-ulp `next`
  step maps zero to `0x1p-1022`, skipping all subnormals. A `Float32`
  conversion also changes the ambient processor rounding mode without
  restoring it, while all explicit double operations temporarily mutate that
  ambient state rather than carrying rounding as local data.
- The MPFR-backed tier delegates arithmetic soundly to MPFR, but its boundaries
  reintroduce the dyadic cap and unsafe formatting. Converting the exact
  70,001-bit `2^70000+1` at 70,002-bit MPFR precision to `Rational` loses the
  low bit through the fixed-precision `Dyadic` intermediary. `cmp(NaN,0)`
  reports `EQUAL` even though boolean equality is false. Literal output uses a
  fixed 1,024-byte buffer and estimates decimal magnitude through `double`;
  both `2^10000` and `2^-10000` produced exactly 1,023 characters, with the
  latter ending in zeros before its first significant digit. Thus values can
  compute beyond 65,535 bits inside MPFR but cannot safely cross the public
  exact-value or text boundaries.
- The validated float wrappers contain direct direction and operation errors.
  Re-precisioning an `UpperBound` from a raw float or `Bounds` rounds
  *downward*; one positive-lower-bound division overload multiplies instead;
  another positive-upper-bound division rounds downward; unsigned powers are
  repeatedly narrowed to signed `Int`; and `PositiveBounds::min` ignores its
  first upper endpoint. `Bounds::narrow` adds to its upper endpoint instead of
  subtracting, while both `widen` and `narrow` use the smallest normal
  single-precision value rather than one ulp. Tangent bounds do not handle an
  interval crossing a pole and can create a reversed finite interval. Bounds
  output can concatenate two 1,023-byte strings into a 1,024-byte stack buffer,
  and its parser rejects its own brace-form output while MPFR input ignores the
  requested directed rounding.
- The centre-radius `Ball<F,FE>` design is a useful conceptual memory pattern:
  it permits a high-precision centre with a cheaper error type and explicitly
  charges re-centering error when precision changes. This implementation is
  not reusable. Raw construction accepts negative radii; reciprocal assumes a
  positive interval; absolute value returns a negative ball unchanged when it
  is wholly below zero; `round` passes rounded endpoints as centre and radius;
  several public `widen`, `narrow`, and `models` methods reference nonexistent
  members; and the input parser compares a non-terminated two-byte buffer with
  the UTF-8 plus/minus token using the assertion condition backwards. Output
  repeats the fixed-buffer overflow risks. A compact centre-radius cache will
  only be considered for Hyper if its invariant and end-to-end memory/performance
  benefit can be proved independently.
- Ariadne's experimental contiguous MPFR-array allocator suggests a legitimate
  locality idea—one allocation for same-precision headers and limbs—but its
  implementation is unusable. Initialization computes each element's limb
  address and then passes the MPFR-header address instead, overlapping object
  metadata and mantissas; changing precision within the same limb bucket does
  not update MPFR precision metadata; allocation arithmetic and failures are
  unchecked; and empty arrays are dereferenced. No production source calls the
  API. Hyper has no comparable bulk homogeneous-MPFR workload, so even the
  repaired concept is deferred unless a later profile identifies one.
- The generic `Number<P>` layer is a code-size, allocation, and soundness
  counterexample rather than a donor. It type-erases concrete values behind a
  very broad virtual interface, instantiates multi-axis dispatcher matrices,
  uses multiple-inheritance wrappers, and heap-allocates a fresh wrapper for
  each operation. Unsupported combinations throw after runtime dispatch, while
  transcendental fallbacks for rational/dyadic bounds call `std::abort()`.
  A Release probe confirmed that `sqrt(ValidatedNumber([1,4]))` terminates with
  signal 6. `cast_exact` merely reuses a `ValidatedUpperNumber` or
  `ValidatedLowerNumber` handle without checking singleton equality: a
  `FloatDPUpperBound(5/2)` consequently reports itself as an `ExactNumber`, but
  retrieving it through the resulting exact-number API throws a paradigm error
  and terminated the uncaught probe. Hyper should retain explicit fallible
  conversion and exhaustive scalar variants rather than adopt this matrix.
- Operator metadata and calculus helpers are independently incomplete. The
  central kind switch throws for declared `FMA`, `NUL`, `HLF`, `ASIN`, `ACOS`,
  `XOR`, and `SGN` codes (among others), and names `FMA`/`NUL` as `UNKNOWN`;
  `Abs::kind()` itself says binary. The arcsine derivative is coded as
  `1/(1-sqrt(x))`, arccosine repeats that formula without a minus sign, and the
  absolute-value derivative ignores its direction argument and returns
  `abs(x)`. These helpers cannot support a correct extraction.
- Complex-number support has similarly untested semantic holes: the polar
  constructor exponentiates its radius, the exact-real `atan2` path throws on
  the negative real axis, public positivity is unchecked, several mixed return
  declarations disagree with the expressions they construct, and declared
  `pow`, `same`, and argument helpers have no definitions. The active complex
  suite comments out constructors and the `Real`/approximation instantiations,
  never exercises the polar-radius constructor or negative real axis, and
  silently ignores all exceptions in two trigonometric blocks.
- Test structure explains why these failures coexist with passing smoke tests.
  Almost every numeric test defines a broad compile-time concept check but does
  not call it; `test_number` explicitly prints `INCOMPLETE`, skips exact-number
  retrieval and generic operation coverage, and never invokes its transcendental
  operation body. The 589-line type-matrix checker is excluded from CMake and
  its `main` only prints `SKIPPED`. Ball validation is declared but never run,
  aliasing tests are defined but omitted, interval input accepts warned
  mis-rounding, and the float suites use a few hand-picked identities rather
  than randomized MPFR-oracle checks. This reinforces the differential and
  adversarial harness strategy already used for Hyper.
- Utility ownership and dispatch reinforce that negative result. `Array` and
  `UniformArray` overwrite their live allocation on move assignment, leaking
  every prior element and buffer; neither manual uninitialized-construction
  path unwinds a partially constructed prefix on exception. `SharedArray`
  leaks its separately allocated reference counter at final destruction and
  uses a non-atomic hand-written count. Other clone/copy-on-write pointers are
  likewise non-atomic and null/move-fragile. The tests exercise ordinary
  conversion and printing, but not ownership, exception, move, or concurrency
  contracts. Hyper's standard RAII containers and `Arc` graph ownership are
  both smaller in bespoke code and safer, so none of these mechanisms transfer.
- The remaining utility abstractions add cost or defects rather than useful
  scalar machinery. `LRUCache` increments every stored age on every hit or
  insertion, making nominally cached access linear; path comparison copies
  both stored words and is not a total order across differing heights;
  `BinaryWord::erase_prefix` is quadratic; the header-global randomizer gives
  each translation unit an independently initialized `rand()` wrapper; and
  `Attribute::Generator::operator=` downcasts a newly created base
  `Attribute<V>` object to arbitrary derived `T`, which is undefined behavior.
  `Exception` does not derive from `std::exception`, `BadCast` discards its
  message, stopwatch seconds can lose sub-unit precision after an earlier
  duration cast, and stream operators are injected into `namespace std`.
  The small-code `CodedVariant` avoids payload storage but manually expands
  switch cases only through 16 and aborts on an invalid code. Hyper's typed
  enums/variants, direct exhaustive dispatch, and existing cache structures
  remain preferable for exactness, performance, memory, binary size, and code
  size.
- The algebra containers are further negative memory-safety evidence. The live
  `MultiIndex` allocates exactly one 16-bit slot per variable but its degree
  rollover still reads/writes the removed cached-degree slot at index `n`;
  Valgrind reports 18 invalid reads from this single context while the upstream
  `test_multi_index` continues to pass. Unequal-length `Vector` equality walks
  the left length with no size check and a focused two-versus-one probe reads
  beyond the right allocation. `SymmetricMatrix::resize(3)` allocates
  `3*(3-1)/2` rather than `3*(3+1)/2`; assigning element `(2,2)` is an invalid
  write 16 bytes beyond its allocation. Vector/covector transposes, symmetric
  storage views, multi-index views, and the specialized differential-vector
  facade also use base-to-derived or unrelated-type `reinterpret_cast` /
  `static_cast` references rather than constructing the claimed objects.
- Dense, sparse, and graded Taylor representations contain appealing generic
  ideas—choose sparsity by term count, keep index and coefficient arrays
  contiguous, and compose by truncated Horner recurrences—but not a reusable
  implementation. The live multi-index fault poisons every enumeration;
  dense degree reduction iterates through the source degree while writing the
  smaller result; one scaled accumulation uses the maximum rather than minimum
  buffer length; univariate differentiation allocates degree zero for every
  input degree above zero and then writes later coefficients out of bounds;
  and sparse `Differential::check()` dereferences its past-the-end successor.
  Focused Valgrind probes reproduced the latter two faults, with the invariant
  checker aborting on an otherwise valid two-term variable. `Series` power is
  singular at a zero centre, `FiniteSeries` discards rather than stores its
  generated coefficient, sparse/dense evaluators reject constants, and
  multivariate/univariate composition temporarily mutate `const` inputs with
  neither synchronization nor exception-safe restoration.
- The derivative formulas are not trustworthy enough to inform `hypercurve`
  or `hypersolve`. For first differentials, quotient differentiation adds the
  denominator-gradient term instead of subtracting it: the probe
  `(value,derivative)=(2,3)/(4,5)` returned derivative `1.375` instead of
  `0.125`. Squaring a second differential with value 2, derivative 3, and
  half-Hessian 4 returned half-Hessian 17 instead of 25, and the general
  second-order formulas omit or double several chain-rule factors. Sparse
  affine construction writes exponent coordinate `i` rather than `j`; a
  requested Jacobian `[[1,2],[3,4]]` became `[[1,0],[0,3]]`. The active
  differential executable hides this particularly well: the scalar test
  object is commented out in `main`, while its Hessian assertion explicitly
  expects twice the mathematical mixed derivative.
- Linear algebra supplies no sound solver transfer. Ordinary multiplication is
  an eager row-major `i,j,k` triple loop with no blocking or fused scalar
  operation. LU pivot selection later compares the original matrix rather than
  the evolving upper factor, `dd_solve` dispatches to the Gauss--Seidel
  routine, the PLU solve refers to nonexistent fields and has no return,
  orthogonal decomposition computes and then disables its pivot, and
  Gram--Schmidt prints unconditionally while mishandling rectangular shapes.
  Several paths construct unavailable matrix forms, normalize zero rows, or
  alter global rounding without RAII. The test file defines factorization
  checks but `TestMatrix::test()` never calls them; only projection and basic
  arithmetic run. Formal-series implicit solving uses an explicit inverse and
  repeatedly recomputes all lower-degree work, so it is also weaker than
  `hypersolve`'s fraction-free exact decisions and explicit certificates.
- Extraction decision for this closed algebra slice: reject all direct code
  transfer. Packed symmetric storage, structure-of-arrays sparse expansions,
  and sparse/dense Taylor selection are standard techniques, but Ariadne's
  realizations lose exactness or memory safety and there is no identified Hyper
  workload for which reimplementing them would beat the existing typed scalar
  DAG, exact polynomial kernels, and certified solver boundaries. No Hyper
  source change was made from this slice, so there is no speculative patch to
  retain or benchmark.
- All nine registered function executables built in the pinned Release mirror,
  but `ctest -L function --output-on-failure` passed only 8/9. The first
  multiple-precision run of `test_taylor_model` loses monomial indices while
  printing its refinement result and then segfaults during composition. A GDB
  run places the fault in `MultiIndex::operator=`, called by
  `horner_evaluate<FloatMP, TaylorModel<ValidatedTag,FloatMP>>`, then
  `TaylorModel<ValidatedTag,FloatMP>::_compose`. The same executable warns that
  singleton-domain unscaling is not constant, reports `0+/-inf*0` as NaN, and
  labels predicate support incomplete before the crash. Compilation also warns
  that the concrete vector scaled-patch class hides virtual `_unchecked_compose`
  overloads from its interface.
- Function-test structure again explains the discrepancy. `check_function` is
  commented out of CMake. `test_function::test_concept`, the Chebyshev concept
  body, the multifunction Taylor concept, and the Taylor-function/model concept
  bodies are never called. Scalar Taylor-function gradient checks are wholly
  commented; compose is empty; conversion returns unconditionally before most
  of its body; and that executable prints `INCOMPLETE`. Taylor-model rescale and
  restrict tests are empty, powers cover only nonnegative exponents, and the
  known singleton-unscale failure is downgraded to a warning. No active test
  exercises aliasing after vector mutation, malformed scalar/vector joins,
  validated-coefficient sweeping, or precision-carrying procedure output.
- Formula nodes form an immutable heap DAG, but there is no interning or
  constructor folding. The advertised cached evaluator allocates a new map for
  each recursive child call instead of threading the current cache, so only
  shared output roots—not shared internal subexpressions—benefit. Its generic
  fallback silently returns input coordinate zero, coordinate evaluation does
  not validate dimension, and `same(EffectiveNumber)` cross-compares opposite
  interval endpoints. These choices are weaker than Hyper's shared `Arc<Node>`
  graph, constructor-time exact rewrites, per-node monotone approximation cache,
  and bounded structural normal forms.
- Procedure lowering is the one plausible performance pattern: immutable DAGs
  become a linear instruction tape and pointer-identical subexpressions are
  emitted once. In this implementation constants are not deduplicated, there is
  no liveness/register reuse, and one full-precision temporary remains live for
  every instruction. Structural duplicates that do not share a pointer are not
  recognized. Hyper already obtains pointer-sharing CSE through node caches and
  uses specialized fused nodes/kernels for hot aggregates, while retaining
  demand-selected child precisions; a generic eager tape would add storage and
  code without a measured scalar win.
- Procedure completeness and safety also rule out source reuse. The public
  vector evaluator constructs `Vector<X>(result_size)` with no precision, so a
  minimal call with `FloatDPApproximation` fails template instantiation because
  that element is not default-constructible. Constructing a vector procedure
  from a scalar procedure leaves `_argument_size` uninitialized: the Release
  probe printed zero instead of one and Valgrind exited 97 on uninitialized
  use. Size-only procedures can own no instruction and evaluate `back()`;
  validation is otherwise assertion-only. Duplicate result indices happened to
  survive destructive moves for the sampled DP approximation and are therefore
  not claimed as a reproduced failure. Reverse interval propagation is
  mathematically incomplete for square, powers, and periodic functions, and
  the `Leq` contractor restricts its first operand twice. `Max`, `Min`, and
  `Abs` gradients assert in debug and silently retain zero work in Release.
- Focused Release probes establish independent basic-function failures. An
  `Affine` with intercept 5, gradient 3, and input 2 evaluates to 6 rather than
  11 because evaluation omits the intercept. Multiplying approximate affine
  models `(2,3)` and `(4,5)` returns centre 6 and gradient 17 instead of 8 and
  22; the validated product's remainder expression also omits parentheses and
  can underbound. Multivariate partial evaluation of the constant polynomial 3
  prints a plausible result but writes `cpowers[1]` past a degree-zero
  allocation, producing Valgrind exit 97. Adding 3 to the coordinate
  Chebyshev polynomial is a Release no-op and evaluates at 2 as 2 instead of 5.
  A suspected unsigned-index fault in Chebyshev multiplication was explicitly
  retracted after `(T0+T1)^2` produced the correct
  `1.5*T0+2*T1+0.5*T2` under Valgrind.
- The erased `Function` hierarchy combines a large virtual `_call` matrix,
  multiple virtual inheritance, RTTI decomposition/rebuilding, and explicit
  DP/MP approximation, bounds, differential, Taylor-model, formula, and algebra
  instantiations for every concrete wrapper. It therefore has a substantial
  build and binary-size surface despite unsupported paths. Handles may be null,
  dynamic casts are commonly unchecked, and many dimension contracts disappear
  with assertions. Range/gradient/Jacobian helpers reinterpret vectors of one
  interval representation as another. `VectorOfScalarFunction::operator[]`
  base-to-derived casts an actual `ScalarFunction` to a different concrete
  class. Release derivatives of `Abs`, `Max`, and `Min` return an arbitrary
  argument derivative after disabled assertions.
- Two erased-function bugs were reproduced end to end. Joining a scalar
  constant to a two-component identity reports only two results rather than
  three because the scalar/vector branch sizes and loops by the scalar result.
  Copying a two-component effective identity and replacing one element changes
  the original as well: both evaluate to `[3,3]` instead of leaving the original
  `[2,3]`, because mutation uses `dynamic_cast` plus `const_cast` on shared
  supposedly immutable storage. This also precludes thread-safe aliases.
  At `function.cpp:1114-1116`, patch-plus-constant dispatch extracts a patch but
  recursively invokes the same erased operands; the probe segfaulted in under
  0.1 seconds, and GDB showed alternating `AlgebraOperations::apply` and
  `coded_visit` frames through stack exhaustion.
- Function-model and patch layers multiply rather than repair those issues.
  Scalar copy is deep while vector copy is shallow, yet vector arithmetic
  mutates components, so ordinary unary/binary operations may mutate aliases.
  Empty-vector paths index component zero, vector arithmetic omits size checks,
  factories return scalar handles from vector declarations or hard-code
  `ValidatedTag` for approximate models, and broad univariate/vector derivative
  and factory routes are unimplemented. A `UniquePointer` constructor clones
  rather than moves. Scaled patch normalization onto a unit box is a standard,
  useful conditioning technique, but the implementation divides by zero on
  singleton radii, accepts incompatible domains in Release, exposes a dangling
  reference from an element proxy's by-value domain, and relies on representation
  reinterpretation. Its vector `operator-=` actually adds; subtracting an
  identity from itself evaluated to `[1]` at `x=0.5` instead of zero.
- Vector scaled-patch `same` does not compare result sizes. Comparing a
  two-result joined identity with a one-result identity segfaulted natively;
  Valgrind exited 97 after multiple invalid reads beyond the smaller model
  allocation and eventually attempted an allocation of a fishy negative size.
  Other dead or latent paths call nonexistent helpers, omit required template
  arguments, or return types inconsistent with their declarations. The
  measurable/multifunction layer is experimental rather than a donor: stored
  callables lack a usable constructor, casts are assertion-guarded only, a
  fixed `2^-4` accuracy is embedded in one model, a regular-set path is disabled
  by `false and`, a non-inline header definition risks ODR violations, and one
  polymorphic set-function interface lacks a virtual destructor.
- Taylor models contain the most serious exactness failures. Both threshold
  sweeper overloads for bounded and upper-interval coefficients write
  `te + mag(coefficient)` without assignment. Direct probes swept the validated
  term `0.5*x` at threshold 1 to zero terms while leaving the enclosure error
  exactly zero; the mathematically required error is at least 0.5. Public
  differentiation calls `clobber()` first: differentiating a unit-error model
  returned a zero polynomial with zero error, although a uniform function-value
  error gives no finite derivative enclosure in general. Sweepers also mutate
  the process rounding mode instead of restoring the caller's mode, and an
  ostensibly approximate model advertises `ValidatedTag` and validated function
  aliases in its public traits.
- Additional Taylor defects include missing returns from `ZeroError` compound
  operations; a hard-coded 64-bit MP unknown-error default; assertion-only
  nonnegative error and compatibility invariants; a latent `_sma` call with
  missing arguments; an approximate `range()` that asserts and then returns
  only the initial error interval in Release; invalid empty-index handling in
  variable discard; singleton unscale mapping to zero; unchecked partial
  evaluation indices; incompatible Jacobian matrix types for bounded
  coefficients; and a Taylor-series compose that attempts to mutate a value
  returned from a `const` operand. Analytic composition hard-codes DP bounds
  even for MP and assumes, without a supplied proof, a monotone truncation
  derivative bound. A source-level negative-power overload that looked wrong is
  not on the public dispatch path: a direct public `pow(model,-1)` probe at two
  returned the certified singleton 0.5, so that candidate is explicitly
  retracted.
- Taylor quadratic-plus-linear range refinement and sparse Horner composition
  are mathematically relevant to bounded multivariate function models, but not
  to Hyper's scalar representation. The former needs a proof and sound
  coefficient/error substrate; the latter already appears in Hyper's exact
  polynomial consumers where applicable. Header-anonymous duplicated model
  utilities, duplicated Taylor kernels, global rounding-state changes, and the
  full DP/MP/coefficient virtual-instantiation matrix work against both binary
  and source size.
- Extraction decision for the closed function slice: retain no Hyper change.
  Formula sharing, tape lowering, sparse polynomial storage, normalized domains,
  and local quadratic range refinement were compared with Hyper's current
  scalar DAG, node-local caches, fused aggregate kernels, exact polynomial
  layers, and explicit certified solver boundaries. They are either already
  subsumed, belong to a different validated-function abstraction, or cannot be
  justified atop this implementation's lost enclosures and memory faults. No
  benchmark can redeem a lower-priority performance idea that first regresses
  exactness, and no speculative source patch was made.
- The symbolic layer's attractive surface is a typed, immutable expression DAG
  over named variables, valuations, predicates, vector expressions, and finite
  set constraints. It can lower function expressions to the formula/procedure
  machinery already assessed above. Hyper already has the materially useful
  scalar pieces: shared immutable nodes, exact rational/symbolic facts, bounded
  structural comparison, node-local caches, and explicit proof/certificate
  boundaries. Ariadne adds no hash-consing or construction-time canonicalizer;
  its CSE walks recursive sets and maps, vector-output lowering creates a fresh
  cache per component, and one `Space` route disables caching entirely.
- Its simplifier is unsound for partial operations. It reduces `x/x` to one
  without proving `x != 0`; at `x=0` the original interval is unbounded while
  the result is the singleton one. It also reduces `exp(log(x))` to `x`; at
  `x=-1` the original raises a domain error while the result evaluates to -1.
  The same source rule reduces `sqr(sqrt(x))` without a nonnegative premise.
  The upstream test explicitly blesses the first textual identity but never
  evaluates it on the removed domain. In contrast, Hyper's division checks a
  denominator's exact/opaque zero status before numerator or same-basis
  identities, and logarithm checks sign before any symbolic collapse.
- Public predicate support is both mathematically and mechanically broken.
  Calling `opposite(x<y,y<x)` aborts with `std::bad_variant_access` because the
  real-comparison branch then extracts a Boolean binary-node variant. Even with
  that cast repaired, its table treats swapped strict or non-strict comparisons
  as complements despite equality, and misses the actual `<`/`>=` and
  `>`/`<=` complements. A standalone include/usage probe for `predicate.hpp`
  fails compilation through unknown validation types, incomplete evaluation,
  and obsolete interval endpoint methods; several CNF/disjunctive declarations
  lack definitions, tautology state is uninitialized, and its polymorphic base
  lacks a virtual destructor.
- Vector and component shape invariants are not enforced. Combining expression
  vectors of lengths two and one reports the maximum length, then simplifying
  component one reads past the shorter allocation and segfaults; Valgrind
  records an invalid read immediately beyond its 16-byte block. The
  component-constancy helper ignores its component index, so constant component
  one of `[x,1]` is falsely reported variable in `x`.
  `Expression<RealVector>::set` is declared but has no definition, component
  node counting omits nested vector components, and two vector operator
  families are redundantly defined.
- Template utilities contain independent lifetime and initialization faults.
  Both three-argument symbolic/temporary constructors omit `_arg2`; a Release
  probe returned 5 rather than 7 and a debug Valgrind run reported 18
  uninitialized-value errors. `Iterate` returns a sequence whose lambda captures
  a local start value and `this` by reference; it returned 4 instead of 13, and
  Clang ASan with stack-use-after-return detection pinpointed a
  stack-use-after-scope at `templates.hpp:173`. It also recomputes the recurrence
  from the start for every term. The advertised conditional does not compile
  because it references `_atr` rather than `_atru`.
- Symbolic names, categories, and valuations have smaller correctness holes.
  The stream formatter swaps `PRIMED` and `DOTTED`; `Constant<String>::name()`
  downcasts a sibling-derived string object to `Identifier`, which is undefined
  behavior; valuation equality throws rather than returning false when key sets
  differ; space/array lengths are unchecked; and a two-map `HybridValuation`
  constructor cannot compile because it passes an identifier-keyed map to a
  variable-keyed continuous valuation. A nearby `algebraic_sort` implementation
  fails to reset its `found` flag each pass, allowing some residual cycles to
  escape Release detection after an initial leaf.
- Header/API hygiene further argues against transfer. `function_expression.hpp`
  defines non-template free functions without `inline`, risking multiple
  definitions. Taylor-model projection composition maps indices in the wrong
  direction and can address outside the new dimension. Duplicate set bounds
  are rejected only by a debug assertion after a map has already discarded
  them, and a nominally `const` labelled-set insertion mutates its object.
  Ordering symbolic constants may invoke unbounded exact-real comparison merely
  to populate an ordered container, which is incompatible with Hyper's bounded
  structural-equality and explicit undecidability policy.
- Extraction decision for the closed symbolic slice: copy no Ariadne code and
  do not add a second expression representation. Retain only narrow regression
  assertions proving Hyper continues to check a denominator before identity
  rewrites: `0/0` remains `DivideByZero`, and unresolved opaque `x/x` remains
  `UnknownZero`. This is a test-only exactness defense with no production,
  memory, binary, or hot-path cost. The complete all-feature/all-target test,
  lint, doctest, documentation, formatting, and diff gates pass.
- The solver tests are a narrow smoke surface despite their green label. The
  Runge--Kutta class is not mentioned by `test_integrator`; simplex is not
  instantiated by `test_linear_programming`, whose unconstrained feasibility
  and optimization calls are commented out; `test_nonlinear_programming`
  returns immediately after the first optimizer, leaving the other three
  optimizer blocks unreachable; and the affine integrator test is commented
  out with an explicit warning that it is incorrect. Several tests accept
  warning-only failure paths, and both integrator and linear-programming tests
  print `INCOMPLETE`. Consequently the 6/6 CTest result is retained only as an
  upstream build/smoke baseline.
- The approximate Runge--Kutta implementation is demonstrably wrong and
  process-history dependent. Its RK4 sum is `k1+2*k3+2*k4+k2`, rather than
  `k1+2*k2+2*k3+k4`; for `x'=x`, `x(0)=1`, and `h=1`, a direct public probe
  returns about 2.917 rather than the classical RK4 value 2.708333. `evolve`
  stores the member step size in a function-static local, so a later 0.5-step
  instance exactly repeats the earlier 0.25-step instance's time grid and
  values. A separate zero-step process enters its evolution loop and times out
  after two seconds because neither time nor state can advance. The loop also
  takes a full fixed step past a nonmultiple terminal time. No approximate ODE
  code is a candidate for Hyper.
- Euler flow bounding reverses its initial-widening branch. `_formula` widens
  only when `INITIAL_BOX_WIDENING == 1`, so its named no-widening value doubles
  the radius while its named factor two performs no widening. A disposable
  access probe on the zero vector field and `D=[-1,1]` returns `[-2,2]` for
  factor one and `[-1,1]` for factor two. The source also shadows its configured
  minimum step with an unused local constant, divides by a possibly zero
  Lipschitz estimate, and contains retry loops without a universal progress
  floor. These defects disqualify the bounder as a validated-function donor.
- Constraint and nonlinear optimization code does not maintain its advertised
  proposal/certificate boundary. `ConstraintSolver::feasible` calls two public
  deprecated `NonlinearInteriorPointOptimiser` helpers whose inline bodies are
  empty; a direct `compute_tz` call leaves point, violation, and slack exactly
  at `[0.25]`, 7, and `[3,4]`. The subsequent fixed loop therefore reduces to
  the original midpoint check or uncertainty, not the documented interior-
  point search. Separately, `cast_exact_widen` constructs a widened constraint
  box and returns the original: `almost_feasible_point` rejects 1.05 in `[0,1]`
  even with epsilon 0.1. `contains_feasible_point` attempts a singular inverse
  once outside its handler; the identically-zero equality over `[0,1]` throws
  `SingularMatrixException` instead of returning a feasibility result. Other
  public paths are explicit no-ops or `NOT_IMPLEMENTED`, ignore multiplier
  arguments, divide by zero-sized constraint counts, return singleton
  approximations as purported optimum boxes, or validate feasibility without
  validating optimality.
- The simplex implementation contains dead/broken consistency checks and an
  uncomfortable handoff: `hotstarted_feasible` computes a rigorous
  `verify_feasibility` result, warns on disagreement, then deliberately returns
  the earlier pivot-loop result under `//FIXME: Return correct value`. This was
  not promoted to a confirmed wrong-answer claim. For the deliberately bad
  basis of `x0+x1=1/2`, `0<=x<=1`, the verifier correctly returns
  `indeterminate`, and the hot-start loop moves to the actually feasible result.
  A separate exact oracle over all 1,520 one-row/two-variable integer systems
  with coefficients -4 through 4, right sides -9 through 9, and unit-box bounds
  found zero false positives, zero false negatives, zero indeterminate results,
  and zero exceptions. The concern remains a design rejection, not evidence of
  a sampled misclassification.
- Interval Newton and Krawczyk proposal/filter structure is the solver slice's
  sound high-level idea, but it is already stronger and more explicit in
  `hypersolve`. Ariadne's generic solver can recursively bisect without a depth
  limit when configured with a nonpositive maximum error, retries a known
  singular midpoint inverse merely to print a warning, marks component
  refinement flags permanently even after replacing the whole candidate, and
  reports iteration exhaustion as `NoSolutionException` although it failed to
  prove existence. Its continuation entry point is unimplemented. More
  importantly, these routines consume the function/Taylor enclosure layer
  already shown to drop error and compute wrong derivatives, so their local
  certificate algebra cannot repair the upstream proof substrate.
- The ODE/inclusion machinery adds no transferable scalar kernel. One exact-
  step retry repeatedly assigns `hred=hlf(h)` instead of halving `hred`, a
  graded-order test is tautologically biased toward spatial order, and a Taylor
  bounder checks a stale range while changing step size. The inclusion error
  helpers evaluate removable limits naively: direct zero probes produce NaN for
  `dexp(0)` and `[-inf,inf]` for both `psi0(0)` and `psi1(0)`. Denominators are
  converted with unchecked positivity casts; the integrator and its error
  processor retain references to factory arguments and can outlive them;
  equality ignores the function, inputs, and underlying integrator; zero-input
  substitution indexes `w[0]`; and the piecewise route charges approximation
  error only to its second half. Hyper has no generic validated ODE object to
  receive this large, unsafe surface.
- Extraction decision for the closed solver slice: retain no code or test
  change. Ariadne's useful principle—let approximate iteration propose and an
  independent interval/exact check decide—is already an explicit public
  invariant in `hypersolve`: proposal precision is reported, every active row
  is exact-replayed as certified/violated/unknown/domain-failed, Krawczyk/alpha
  reports are separate certificates, and iteration counts are bounded. The
  focused lossy-adapter unit test and generated active-row property test both
  pass. With no new exactness or completeness gain, benchmarking a speculative
  duplicate solver layer would not justify its memory, binary, and code cost.
- The complete direct geometry layer is now read line by line: 47 files under
  `source/geometry` and ten files under `tests/geometry`, or 57 files / 21,791
  physical lines. The source manifest comprises its `CMakeLists.txt`; all
  affine-set, binary-tree, box, curve, function-set, generic-geometry, grid,
  grid-cell, grid-paving, interval, list-set, measurable-set, paver and paving-
  interface, point, polyhedron, polytope, set-interface/wrapper, interval-union,
  and zonotope files. Test coverage comprises its `CMakeLists.txt` and all nine
  registered translation units: affine sets, binary tree, box, constrained
  image set, grid paving, interval, measurable set, paving, and point/curve.
  This closes the eighth credited slice and brings Ariadne coverage to 399
  files / 113,510 lines.
- All nine geometry targets built in the untouched pinned Release mirror and
  the labelled CTest suite passed 9/9 in 1.35 seconds. This is weak smoke
  evidence rather than a broad correctness result: the tests hard-code many
  serialized paving shapes but do not exercise destructive tree collapse,
  self-assignment, polymorphic subtree cloning, empty interval unions, a
  two-list intersection that must advance past an early disjoint interval, or
  coordinate projection beyond its first refinement level.
- The interval layer is not an enclosure donor. A direct probe shows its
  purported `equal` relation is asymmetric (`equal([0,1],[-1,1])` is false but
  the reverse is true); widening `[1,1]`, `[-1,-1]`, or `[0,0]` moves only the
  upper endpoint, so the original lower endpoint is not interior. The
  implementation computes the missing downward endpoint and then discards it,
  uses the smallest normal rather than adjacent representable values, mutates
  process-global rounding/formatting state, ignores parsed openness, and names
  an MP interval `FloatDPInterval`. The stock interval test passes without
  covering these contracts.
- Pointer ownership makes the binary paving actively unsafe. After `split()`,
  `BinaryTreeNode::set(true)` deletes both children without nulling them: the
  node still reports non-leaf, ordinary destruction exits with signal 11, and
  Valgrind reports invalid reads and duplicate deletes of both allocations.
  `node=node` deletes the source children before copying them and silently
  changes the split tree into a disabled leaf. Empty-path prepend underflows an
  unsigned index, restoration trusts malformed parallel arrays, const accessors
  expose mutable descendants, and the representation pays a separate heap
  allocation for each 24-byte node.
- The polymorphic paving copy boundary is independently wrong. A focused probe
  constructs an empty left half-cell and invokes `GridTreeSubpaving::clone`;
  the source has size zero, but the clone has size one and denotes the entire
  enabled extent-zero cell. The implementation passes a non-null node pointer
  as the two-argument `GridTreePaving(Grid, Bool)` flag instead of copying the
  subtree, and also discards the source root extent and path. Measured native
  layouts are 24 bytes for `BinaryTreeNode`, 80 for `GridCell`, and 128 for
  `GridTreePaving`, before per-node allocator overhead.
- `UnionOfIntervals` confirms the value of normalized disjoint runs but not the
  implementation. For `[2,3]` against `[0,1] U [2,4]`, its Boolean predicate
  correctly reports an intersection while its materializing intersection
  returns `[]`: the latter advances the later list rather than the earlier
  disjoint interval. `measure(empty)` dereferences element zero and exits with
  signal 11, and converting constructors bypass sorting/coalescing. Hyper has
  no current general interval-set consumer, so repairing and importing this API
  would add unsupported surface rather than improve an existing path.
- The grid's advertised dyadic architecture is approximate at its foundation:
  `DyadicType` is an alias for `double`, arbitrary lattice indices pass through
  binary64 and lose integers beyond `2^53`, spacing/origin values are not
  checked finite and positive, and space boxes are built by approximate
  arithmetic followed by an `exact` cast. Zero-dimensional words take modulo
  zero; empty neighbor paths underflow; enclosing-cell search can enlarge an
  exactly equal cell and can loop on invalid/unbounded input; and logarithmic
  subdivision can under-refine. These properties violate Hyper's requirement
  that a scheduling key may reject topology work only when the rejection is
  exact or independently certified.
- Multi-level coordinate projection is observably wrong. A 2-D cell word
  `0110` records `(x0,y0,x1,y1)`; projecting coordinate one must yield `10`,
  but the Release probe yields `11` because every level rereads index one.
  Product-word construction likewise iterates only the first cell's depth,
  while product pavings do not require equal leaf depths. Outer skew product
  mutates its `const` input through `const_cast`, can underflow its fineness,
  and repeatedly reconstructs boxes while traversing pointer nodes.
- Affine and constrained-image geometry supplies a sensible high-level
  decomposition—cheap range/affine rejection followed by LP or interval
  certification—but not a reusable implementation. Real-ball domain error uses
  the lower endpoint twice; non-unit affine domains silently leave zero models;
  several constructors index element zero of an empty function vector; one
  robust recursion immediately calls the non-robust routine; and boundary
  drawing deliberately perturbs an approximate simplex problem. Bounded-set
  intersections substitute constraint codomains for parameter domains,
  reduction mutates an exact box through `reinterpret_cast`, domain
  restriction is a no-op for functions, and overlap exhaustion discards its
  accumulated indeterminate result.
- The pavers lack universal progress and resource bounds. Pure subdivision can
  bisect a zero-width or insensitive parameter forever; the affine variant can
  materialize up to `2^16` domains; a feasibility failure blocks on
  `std::cin`; signed `1<<e` scaling overflows; and the optimal constraint paver
  chooses its root cell from constraint-space output rather than the mapped
  state, then falls through after recursively splitting a domain. These paths
  depend on the already-unsound function/Taylor and interval layers and cannot
  strengthen Hypertri or Hypersolve.
- Remaining primitives reinforce the rejection. Curves dereference an empty
  point map, boxes and polytopes index dimension/vertex zero when empty, and
  several declared box operations do not match their definitions. Polyhedron
  construction compares matrix columns with the constraint-vector length,
  intersection copies the first matrix into both halves, and its parser does
  not advance the lookahead character. Zonotope membership and pairwise
  separation omit stored error, splitting admits that it is not a guaranteed
  over-approximation, generator-reduction matrices leave entries uninitialized,
  and nominal orthogonal over-approximation is a stub returning its input.
- Extraction decision for the closed geometry slice: retain no Hyper code or
  test change. Hypertri already has exact triangle AABB rejection, exact
  predicate ownership through Hyperlimit, deterministic BRIO rounds with exact
  alternating-axis median order, and an explicit architecture trigger requiring
  a cheap exact dyadic/grid key. Ariadne's cyclic cell path is therefore either
  subsumed as scheduling or weaker because it uses binary64 and unsafe owning
  pointers. No production experiment survived the exactness/completeness gate,
  so there is no honest runtime, allocation, binary-size, or code-size delta to
  benchmark; the focused upstream probes and full labelled smoke suite are the
  relevant disposition evidence.
- The two profiling files, all thirteen isolated prototype files, the complete
  Kirk adapter/demo/metadata set, the experimental exact-real calculus sketch,
  their enclosing manifests/readme, and the seven root source/test harness files
  are now read line by line: 34 files / 3,464 physical lines. This ninth credited
  slice brings Ariadne coverage to 433 files / 116,974 lines. The arithmetic
  profile compares repeated hardware-rounding switches, a one-mode negated-lower
  dot, and heuristic midpoint/radius error sums; the Taylor profile compares
  copying, sweeping, scaling, addition, multiplication, and elementary models.
  Neither profile is registered, and both are stale against this snapshot.
- The one-mode dot identity is valid only under a rigorously honored upward IEEE
  environment and finite intermediates: the upper lane accumulates `x*y`, while
  the negated lower lane accumulates `(-x)*y`. It has no matching Hyper path.
  Hyperlattice delegates exact-rational lanes to shared-denominator/dyadic
  reducers and preserves other lanes as immutable exact-real forms; Hyperreal's
  approximation kernels return exact scaled integers with a stated one-unit
  error, not ambient-rounding interval arrays. Ariadne's code restores a global
  mode manually, uses `volatile` as a compiler barrier, omits exceptional-value
  proofs, and its aggregate `eps` formulas do not cover overflow or underflow.
  An approximate dot tier would therefore lose the first-priority exactness
  property to chase a performance result the reference can no longer reproduce.
- The prototype type lattices clarify Ariadne's intended distinction between
  exact, upper/lower, validated, approximate, and positive results, but are not
  implementation donors. The isolated CMake tree configures with missing-root
  warnings; only the concept archive builds. The friend-mixin prototype has a
  duplicate `Float` definition and invalid inherited calls, the numeric table
  has syntax errors and functions distinguished only by return type, and the
  double-dispatch target cannot locate its hard-coded relative include. The
  Kirk bridge is likewise unfinished: two accuracy methods assert, deletion is
  unresolved, a radius narrows through binary64 normal-exponent assertions, and
  ownership is ambiguous. Hyperlimit's explicit certificate/unknown values
  preserve the useful proof boundary without multiplying scalar wrapper types.
  No Hyper change survives this auxiliary slice.
- All 22 `source/io` and four `tests/io` files are now read line by line: 26
  files / 4,497 physical lines. This tenth credited slice brings Ariadne
  coverage to 459 files / 121,471 lines. The layer makes its semantic boundary
  explicit by converting exact/validated geometry to `double` or approximate
  coordinates before Cairo/Gnuplot rendering. That is appropriate for
  non-certifying visualization, but it cannot improve Hyper's scalar,
  certificate, or lattice kernels and must never be mistaken for a proof path.
- The registered I/O targets build and the exactly anchored `io` label passes
  3/3 in 3.56 seconds. An unanchored `-L io` also matches `function` and therefore
  reruns the already-known Taylor-model crash; it is not counted as an I/O
  failure. Source review finds uncovered boundary defects: empty polygon paths
  index element zero, a zero-dimensional shape stops painting all later shapes,
  the Gnuplot 3-D fill loop admits index `dimension`, and its command is assembled
  through a shell from a filename. These are output-layer concerns, not scalar
  transfer candidates.
- `Figure` and `LabelledFigure` use owning raw `Data*` fields with destructors but
  no copy control. A focused Release copy aborts on double free; Valgrind traces
  invalid reads and duplicate deletion to the same 496-byte `Figure::Data` and
  nested allocation. Together with redundant Gnuplot buffer growth and several
  unimplemented 3-D paths, this rejects the I/O layer as a memory/performance
  donor. No Hyper edit or synthetic benchmark is justified by this slice.
- All 33 `source/dynamics` and 13 `tests/dynamics` files are now read line by
  line: 46 files / 9,981 physical lines. This eleventh credited slice brings
  Ariadne coverage to 505 files / 131,452 lines. It includes both experimental
  wave solvers, the rigorous first-order PDE path, enclosure representation and
  reconditioning, vector-field/map/inclusion systems and evolvers, simulators,
  orbits and flow tubes, labelled storage, finite/infinite reachability, safety
  certification, every test translation unit, and both CMake manifests.
- All twelve registered dynamics targets build against the untouched pinned
  Release mirror, and the exactly anchored `dynamics` label passes 12/12 in
  10.46 seconds wall time. Coverage is weak: the inclusion-evolver suite has no
  correctness assertion, the wave and first-order PDE tests assert only inside
  failure-triggered branches, and the vector-evolver test accidentally checks
  its earlier flow tube again where it claims to check a trajectory. Green CTest
  is therefore retained only as build/smoke evidence.
- The wave solvers are neither exact nor internally coherent. Their header
  `linspace` routines use `L/n` but force the final sample to `L`; stale `.cpp`
  copies use `n-1` yet are uninstantiated and contain further invalid code. Both
  header solvers iterate through `n == Ntime-1` and store `n+1` into a tensor
  whose last extent is only `Ntime`. Because Tensor flattens unchecked indices,
  this does not necessarily cross the heap boundary: a focused 3x3 2-D probe
  silently replaces the next spatial cell's Gaussian time-zero value
  `0.36787944117144228` with `-0.020420122421933418`, while Valgrind reports zero
  errors. The production-size test also loops its time coordinate using the
  second spatial extent rather than the time extent.
- The advertised rigorous first-order PDE implementation stores `Ds`, `Ts`, and
  `f` by reference, allowing temporaries to dangle; computes a matrix supremum
  with `column_size` for both dimensions; derives grid sizes through `double`,
  `log2`, and `std::pow`; and binds the source function `f` but never uses it in
  the update. Its shipped test deliberately supplies a zero source, so that
  omitted equation term is invisible. These paths cannot supply scalar or
  certificate logic to Hyper.
- The enclosure layer has a valuable conceptual separation between an exact
  parameter box, validated state/time/dwell maps, constraints, variable kinds,
  and optional auxiliary mappings. It also makes accumulated uniform model
  error explicit as independent normalized parameters and trims directions by
  sensitivity. There is no current Hyper validated-function or dynamics object
  on which to apply either pattern, and this implementation loses variable-kind
  metadata on splitting/reconditioning, loses metadata and auxiliary maps in
  products, can underflow its block count, continues after rejecting an
  incompatible dynamic model, and type-puns coefficient/error storage. The
  idea is recorded for a future layer, not imported into exact scalar code.
- The differential-inclusion evolver runs multiple approximation algorithms,
  scores their validated output boxes by approximate volume, and always keeps a
  rigorous reach enclosure even if a non-rigorous evolver wins. This is the one
  substantive performance architecture in the slice. Its exponentially delayed
  retry state eventually performs an out-of-range `1u << delay`, and Hyper has
  no portfolio of interchangeable differential-inclusion enclosures, so it is
  deferred rather than converted into unused code or binary surface.
- Progress and horizon contracts are not reliable. Vector-field and inclusion
  evolvers can overshoot the requested end time, do not guard a zero returned
  step, and can recondition indefinitely at one time. The simulator likewise
  accepts nonpositive steps, mutates configuration during calls, uses heuristic
  widening and approximate RK4, and overshoots its endpoint. The flow-tube
  template names nonexistent `phi` rather than its `_phi` member and remains
  uninstantiated. These reinforce Hyperlimit's existing bounded-refinement and
  explicit-unknown discipline rather than offering an implementation donor.
- The reachability analyser partitions a requested duration with `round` rather
  than `floor`. A direct exact probe of time `3/5` and lock interval `1` returns
  one full step and a remainder whose double-precision enclosure is around
  `-0.4`; exact comparison proves it negative. More seriously,
  `verify_safety` tests the old accumulated reach before expansion, then returns
  `true` without testing the final newly added reach whenever that expansion
  empties the frontier. Its transient and recurrent branches also restrict
  `initial_cells` where the evolved/reached storage was intended. Loop
  termination is therefore not a safety proof; Hypersolve's final exact replay
  and Hyperlimit's non-success unknown outcomes already enforce the stronger
  general rule.
- Storage and metadata handling add further negative evidence. Constructing a
  `LabelledStorage` from a populated `GridTreePaving` delegates with only the
  paving's grid and erases its cells; merges validate state names but not the
  auxiliary mapping; enclosure splitting and products drop semantic metadata;
  and parallel drawing shares one canvas pointer across submitted work. None is
  on a Hyper scalar hot path, and no production edit, allocation optimization,
  binary-size trade, or benchmark candidate survived the exactness and
  completeness gates for this slice.
- All 36 `source/hybrid` and nine `tests/hybrid` files are now read line by
  line: 45 files / 13,969 physical lines. This twelfth credited slice brings
  Ariadne coverage to 550 files / 145,421 lines and closes every tracked file
  under the parent repository's `source` and `tests` trees. It covers
  discrete locations/events, monolithic and composite automata, labelled
  spaces/sets/grids/pavings/storage, enclosures and orbits, approximate
  simulation, validated evolution and event-crossing classification,
  reachability configuration, graphics, every hybrid test translation unit,
  and the CMake manifest.
- The complete untouched Release build succeeds. Because the default target
  deliberately omits C++ test executables, an initial exactly anchored hybrid
  CTest correctly yielded eight “Not Run” results. Building `tests-cpp` then
  running `ctest -L '^hybrid$'` is the authoritative result: 8/8 pass in 1.97
  seconds wall / 2.66 seconds process. The tests emphasize construction,
  plotting, and nominal evolution; many cardinality expectations are warnings,
  the simulator test has no numerical assertion, and none exercises the
  adversarial contracts below.
- Basic labelled-space operations are already incorrect. The public
  `restrict(location, variables)` assertion tests whether every location key
  is in the requested subset, the reverse of its documented operation, so a
  direct restriction from `(a|one,b|two)` to `{a}` throws. Monolithic
  hybrid-space equality checks only the left operand's locations: the focused
  probe prints `small_eq_large=true large_eq_small=false`. The
  initializer-list scaling constructor also leaves its default scaling without
  an explicit value, and composite spaces retain raw pointers to the automaton
  that created them.
- Automaton construction does not consistently preserve semantic labels. The
  atomic reset builder writes expressions by input-list position rather than
  the primed target variable's canonical state index; evaluating reordered
  assignments on `[x,y]=[1,2]` returns `[11,22]` where the declared
  `[x',y']` values are `[22,11]`. `DiscreteTransition` leaves its event
  kind uninitialized, one constructor discards its discrete-variable argument,
  empty composite mode lookup succeeds vacuously, composite cache merging
  silently keeps duplicate/missing updates, and flattening is unimplemented.
  The topological auxiliary sorter detects a pure cycle, but after resolving
  one independent equation it reaches a generic internal size assertion rather
  than its public `AlgebraicLoopError`; the earlier suspicion that Release
  silently accepted it is withdrawn after direct execution.
- Hybrid set templates contain numerous latent compile or API failures:
  nondeducible `is_empty`, references to nonexistent `_esets`, mutation
  through a const list-set accessor, pair/object confusion in bounding boxes,
  dereference-before-location-check in intersection, unchecked missing
  locations in `inside`, an explicitly unimplemented Euclidean adapter, and
  reinterpret-cast conversion of bounding boxes. One orbit specialization owns
  a raw allocated vector without a destructor and indexes element zero when
  constructed empty; storage proxies can retain a reference to a temporary
  location.
- The hybrid paving predicates are decisively broken. `separated` uses
  `iter != end || iter->second.separated(...)`; an overlapping box therefore
  reports certified true whenever its location is present. For an absent valid
  location it dereferences `end()`: the native probe exits 139 and Valgrind
  reports conditional control on uninitialized values, an invalid call, and a
  null jump. `inside` returns after the first nonempty location; a paving with
  a contained first component and disjoint second component reports true.
  `restrict` iterates only right-hand locations and retains unmatched
  left-hand components, while nominally const access can insert a location.
- Hybrid enclosure metadata is incomplete rather than a transferable
  certificate architecture. Event arguments are ignored by several
  guard/invariant/constraint methods, `set_time_function` is explicitly
  absent, multiple declared constructors and flow/guard overloads have no
  definition, drawing ignores the requested location set, and list bounding
  boxes dereference an empty list and assume one shared space. Graphics adds
  raw nonowning shape pointers and shares a canvas across worker threads, so
  neither layer improves Hyper's ownership or memory model.
- The validated evolver usefully distinguishes transverse, convex, concave,
  grazing, urgent, permissive, impact, upper, and lower cases, and carries
  parameter-dependent evolution/finishing functions. However, it const-casts
  and mutates an event set, relies heavily on unchecked composition, can form
  negative step times for already-past parameters, accepts nonpositive accuracy
  and step limits, handles degenerate blocking crossings through a fixed
  two-sample upper approximation, and has no universal positive-progress
  certificate. Several enum cases are never emitted or handled consistently,
  and crossing-time fallback behavior depends on broad exception catches. This
  is useful taxonomy for a future validated hybrid layer, not scalar code.
- The approximate simulator contains an ODR-visible non-inline header function,
  dereferences the first cell when discretization produces an empty grid,
  constructs but discards the widened box for partly zero-width input, chooses
  the first enabled map-ordered event, and does not cap its final integration
  step. A direct constant-velocity run with step 0.4 and horizon 1 returns a
  last point at `1.2000000000000002`. Reachability configuration also allows
  null grid/domain pointers and retains the already-proven round-versus-floor
  time-partition defect.
- Extraction decision for the closed hybrid slice: retain no production or
  test change in Hyper. Canonical target-key binding, exhaustive component
  folds, symmetric equality, owned views, and positive bounded progress are
  already invariants of the relevant Hyper/Hyperlattice/Hyperlimit paths or
  have no current hybrid-dynamics consumer. The adversarial results strengthen
  the audit method but leave no worthwhile hot path, allocation, binary-size,
  or code-size experiment to benchmark.
- All 55 physical text files under `python` are now read line by line: 12,421
  physical lines across the build manifests, 25 binding sources/headers, eight
  tests, five examples, thirteen tutorials, and the GDB pretty-printer. The
  separately tracked `python/pybind11` entry is a pinned gitlink; its vendored
  body is not parent-source coverage. This thirteenth credited slice brings
  Ariadne coverage to 605 files / 157,842 lines.
- The exactly anchored Python label passes 8/8 in 1.48 seconds wall / 2.88
  seconds process, but the suite predominantly checks type registration and
  construction. Its import test merely asserts true; it never verifies
  rounded extrema, mixed vector subtraction, hash invariants, invalid indices,
  iterator owners, or the generic constructor registry. The examples and
  tutorials add demonstrations rather than independent value or enclosure
  oracles.
- The binding layer reverses several operations. `RoundedFloatDP` binds `max`
  to the minimum implementation and `min` to maximum; the focused probe prints
  `rounded_max 1` and `rounded_min 2`. Mixed differential-vector subtraction
  and reverse subtraction are both registered as addition: with values
  `[10,10]` and offsets `[1,2]`, all of plus, minus, and reverse minus produce
  `[11,12]`. The generic inplace helpers are registered on the module rather
  than their class, and `define_inplace_algebra` registers its subtraction
  body under a second `__iadd__` name.
- Python container boundaries are memory-unsafe. The negative-index helper
  subtracts an unchecked magnitude from an unsigned length; `[-1]` works, but
  `RationalVector([1,2])[-3]` exits 139. Matrix indices are unsigned and cannot
  implement Python negative indexing, while matrix getters inherit the core
  unchecked row access. Several `make_iterator` registrations omit an owner
  keep-alive policy. The dormant generic slice helper additionally calls a
  nonexistent three-argument `pyindex` overload, so instantiating it would fail
  compilation rather than enforce range semantics.
- Hashing violates Python's value-object contract. Decimal, string-variable,
  discrete-event, and discrete-location hashes feed temporary `c_str()`
  addresses to `std::hash<const char*>` instead of hashing their content. Eight
  value-equal decimals and events reproducibly produce two distinct hashes;
  repeated calls can even change the hash of one object as temporary placement
  changes. Such objects are used as mapping keys throughout the exposed hybrid
  API.
- The runtime template facade stores concrete classes in an instantiation map,
  but every `def_new` registers a lambda capturing its forwarding-reference
  parameter by reference. The callable dies when `def_new` returns, leaving
  all generic `Float[...]`, `Bounds[...]`, `Function[...]`, and Taylor-model
  constructors with a dangling reference. A 10,000-iteration stateless-lambda
  stress happens to pass in this Release build; that observation does not make
  the lifetime legal and means the defect is optimization/layout dependent.
  Other boundary errors include three optimizer methods bound to different
  feasibility routines, an automaton method named `"__str__ "` with a trailing
  space, wrapper overrides that discard required arguments, and an example
  that aliases and clears the same automaton object twice.
- Extraction decision for the closed Python slice: retain no Hyper change.
  Hyper currently has no Python/FFI package, and its Rust scalar, lattice, and
  certificate APIs do not share these runtime hash/index/ownership paths. A
  runtime generic facade remains a possible future binding convenience, but it
  must own registered callables and treat operator, hashing, indexing, and
  iterator lifetime as exactness boundaries. Ariadne supplies no scalar
  algorithm, performance result, allocation reduction, or binary/code-size
  evidence that survives comparison, so no synthetic production experiment is
  justified.
- All 15 files in the root `tutorials` tree are now read line by line: 1,247
  physical lines covering five demonstrations, the rigorous-numerics and
  hybrid-evolution tutorials, their standalone find/build recipes, and both
  readmes. This fourteenth credited slice brings Ariadne coverage to 620 files
  / 159,089 lines. It also folds the previously read but uncredited four-file
  rigorous-numerics directory into the explicit repository count.
- The explicit `tutorials` aggregate builds successfully. All seven resulting
  executables exit zero; the numeric, algebra, function, solver, geometry, and
  rigorous-numerics programs complete immediately, while the hybrid tutorial
  completes in 24.35 seconds. These programs print or plot nominal results but
  assert no independent enclosure, derivative, solver, or horizon oracle. They
  repeat the already-audited APIs, including the Taylor and hybrid paths whose
  adversarial failures are recorded above, and yield no new transfer candidate
  or honest Hyper benchmark.
- All 45 files in the root `examples` tree are now read line by line: 2,592
  physical lines across continuous attractors, ten uncertain-input systems and
  their shared quality/time harness, the Henon map, three standalone hybrid
  systems, two composed water-tank systems, two PDE demonstrations, and every
  enclosing CMake manifest. This fifteenth credited slice brings Ariadne
  coverage to 665 files / 161,681 lines.
- The examples preserve exact decimal/rational model constants while keeping
  tolerances, step limits, plotting, and the noisy-suite volume-derived score
  explicitly approximate. That is an appropriate proof-versus-scheduling
  boundary, but it is already enforced more narrowly in Hyperlimit and
  Hypersolve. The noisy harness compares five independently validated input
  approximation families by enclosure volume and time; this repeats the
  already-recorded portfolio idea and has no current scalar or fixed-arity
  lattice consumer. The Fourier Dirichlet and finite-difference acoustic
  programs likewise reuse the Taylor/PDE implementations whose uncertainty
  loss and tensor-slot overwrite were established by focused probes.
- The explicit `examples` aggregate builds successfully against the untouched
  Release tree (with the existing overloaded-virtual warnings). A 15-second
  cap was applied separately to every one of the 22 public example binaries:
  ten exit zero (`henon_map`, `bouncingball`, both PDEs, `lorenz`, `vanderpol`,
  `rectifier`, both water-tank programs, and `pi-controller`); twelve
  computation-heavy reachability/noisy programs remain active until timeout.
  No process crashes, asserts, or returns another failure status. Since these
  programs log/plot rather than assert independent certified answers, the
  result is a broad nominal smoke check, not evidence overriding the recorded
  exactness defects.
- Extraction decision for the closed examples slice: retain no new Hyper
  change. Exact model/approximate scheduler separation and rigorous-result
  portfolios are already subsumed, while copying an application-specific ODE,
  hybrid, plotting, or PDE benchmark would enlarge code without measuring a
  present Hyper path. There is consequently no honest production performance,
  memory, binary-size, or code-size experiment to retain.
- The remaining experimental inventory was reconciled against the earlier
  34-file auxiliary credit: exactly 72 files / 4,756 lines were outstanding.
  All seven control-validator, dynamic-game, and Riccati feature files are now
  read line by line (877 lines), bringing Ariadne coverage to 672 files /
  162,558 lines and leaving 65 experimental application files / 3,879 lines.
- These disabled features are unfinished sketches rather than latent solver
  implementations. The control validator has functions with missing returns,
  writes indexed results into an empty list, and obtains both operands of its
  vector addition from the first argument. The dynamic-game probability sums
  repeatedly use element one instead of the loop index, initialize a nominally
  uniform distribution without element zero, and omit returns from prefix
  increment. The Riccati target only evaluates the residual at the zero matrix;
  it contains no algebraic Riccati solver. None strengthens Hypersolve's exact
  replay or Hyperlimit's certificate boundary.
- The disabled `features` directory was enabled only in the disposable
  `/tmp/ariadne-audit-src2` copy and configured successfully. Each of its three
  non-Kirk targets then fails independently: control-validator and dynamic-game
  encounter extensive renamed/missing types, inaccessible construction, and
  invalid output operations; Riccati stops at the removed
  `numeric/float-user.hpp`. No reference or Hyper source was modified, and no
  runnable benchmark claim can be made from these targets.
- All 65 remaining experimental application files were read line by line
  (3,879 lines): 16 ARCH benchmark files / 1,290 lines, 24 continuous and
  noisy-system files, and 25 hybrid laser, power-converter, and water-tank
  files, including every manifest. This closes the entire 97-file / 6,338-line
  experimental tree and brings credited Ariadne coverage to 737 files /
  166,437 physical lines.
- The application layer contains model fixtures and drivers, not a new scalar
  or enclosure implementation. Its duplicated noisy utility header is
  byte-identical to the already-read public-example copy (matching SHA-256
  `3bcd428b08154e1389926d0ac1ae86f6d706269f20fb462aa3db32b7ac91098a`).
  The ARCH harness judges “verified” using approximate `get_d()` width/area
  thresholds rather than an independent certificate; `CVDP23` and `SPRE22`
  write an empty result row and return before any benchmark computation.
  `QUAD20`, `SUTR21`, and `power_converters` are not registered by their
  manifests, and `QUAD20.cpp` includes an absent `QUAD20.hpp`.
- All twenty registered application targets were requested against the
  disposable Release tree. Nineteen build; `laser` has extensive drift across
  removed automaton, location, verbosity, `Real(double)`, initial-set, and
  hybrid-time APIs. The two registered water-tank targets were also rebuilt
  explicitly. Disposable-only CMake entries establish that unregistered
  `SUTR21` still builds, whereas `power_converters` fails across urgent-event,
  integrator, evolver, analyser, and configuration APIs. The pinned mirror and
  every Hyper repository remain untouched.
- The nineteen registered buildable programs plus `SUTR21` were each run with
  an independent 15-second cap. `CVDP23`, `SPRE22`, and
  `harmonic-oscillator` exit zero; sixteen computation-heavy programs remain
  active until timeout. `vanderpol-noisy` instead aborts reproducibly after
  throwing an uncaught `FlowTimeStepException`: its Taylor-Picard step reports
  error about `0.00107845` against a `0.001` maximum. This is a stale/fragile
  benchmark outcome, not evidence for weakening a proof boundary or adopting
  its scheduler.
- Extraction decision for the closed experimental application slice: retain
  no Hyper change. Exact model constants, approximate workload ranking, and
  independently certified portfolios were already considered in the core and
  public-example slices; these drivers add neither a stronger algorithm nor a
  trustworthy measured scalar hot path. Importing any would increase code and
  binary surface without an exactness, completeness, performance, or memory
  benefit.
- All 49 tracked documentation files were read line by line (14,228 physical
  lines), including the 2,540-line generated Doxygen configuration, layout and
  macro definitions, all mathematical/API/tutorial pages, the FIG and EPS
  sources, and visual inspection of both JPEG exports. This brings credited
  Ariadne coverage to 787 files / 181,056 physical lines. Exact manifest
  reconciliation corrected the preceding running tally by one file and 391
  lines; this was an accounting error, not newly omitted source.
- The documentation usefully states the intended separation among exact,
  effective, validated, approximate, lower, upper, and naive information; the
  monotonicity/convergence obligations on validated computation; explicit
  partial domains; proposal-then-certificate solving; normalized Taylor
  domains; and conservative handling of undecidable event ordering. Each
  applicable scalar/solver principle is already present more narrowly in
  Hyperreal, Hyperlimit, or Hypersolve, while the remaining ideas require a
  validated multivariate-function or hybrid-dynamics consumer Hyper does not
  have.
- The prose is not an independent correctness oracle. `real_numbers.dox`
  gives the fast-Cauchy modulus with `max` where the other pages correctly use
  `min`, writes an increasing inequality for a decreasing upper-real
  sequence, and the functional-analysis page repeats the same operand on both
  sides of a monotonicity condition. The pi tutorial labels an exact
  `two**120` construction wrong, several APIs are stale, and `macros.sty`
  maps `\\B` to naturals. The built source already contradicts more important
  advertised contracts through the focused failures recorded above.
- The `doc` target completes and produces 1,787 HTML and 1,024 LaTeX files,
  but Doxygen reports 602 warnings and two missing-snippet errors while still
  exiting zero. Its generated artifacts therefore confirm parsability and
  inventory coverage only. No exactness, completeness, performance, memory,
  binary-size, or code-size improvement survives comparison, so this slice
  retains no Hyper change and needs no artificial benchmark.
- All remaining 22 regular parent-tracked support files were read line by line
  (2,358 physical lines): CI workflows, coverage/configuration modules,
  packaging metadata, licence/history, filters, ignore/module manifests, and
  the submodule dispatcher. This closes all 809 regular parent files / 183,414
  physical lines. They contain provenance, build, and test plumbing rather
  than another exact-real implementation; the CI invokes already-audited
  suites and cannot supersede the adversarial failures above.
- `bash -n ariadne_filter` passes. The complete configure/build/test/doc matrix
  already validates the relevant CMake paths; optional `actionlint` and Debian
  changelog validators are not installed. The pinned Ariadne worktree remains
  clean.
- The four parent gitlinks are inventoried separately: pybind11
  (`4dc4aca...`), betterthreads (`0e403ed...`), conclog (`2027bf...`), and
  helper (`234dd...`). Betterthreads repeats conclog/helper as nested pins;
  these third-party dependency bodies are not parent Ariadne source and do not
  alter the exact-real transfer decision.
- The official project, release, installation, tutorial, and publication pages
  were reviewed. They agree with the pinned v2.5.3 release and describe the
  same validated/hybrid machinery already covered by source and documentation;
  no additional scalar algorithm or trustworthy benchmark appears there.
- Extraction decision for the closed Ariadne reference: retain only the two
  previously committed Hyper regression assertions for denominator-before-
  identity domain ordering. Add no production code; no further candidate
  survives the requested exactness, completeness, performance, memory,
  binary-size, then code-size ordering.
- Next live cursor: Boost.Real 2018.

## Boost.Real 2018 source-audit record (closed)

- Repository: `https://github.com/BoostGSoC18/Real`, commit
  `5532037f5e4e5cad5d7e940a49191bb44099a7e5` (master, 2019-04-16). The
  `v1.0.0-beta` tag is the immediate pre-merge parent (`586b08c`); the two
  later commits contain typo/grammar corrections. The pinned clone is clean.
- Parent inventory: 293 tracked entries comprising 292 regular files and the
  `cmake-modules` gitlink at `fcfc0494...`. The regular files total 45,762
  `wc -l` lines: 11 implementation headers, 21 test files, one vendored Catch2
  header, project/build metadata, Doxygen configuration, an SVG logo, and 250
  generated HTML assets including 51 PNGs. Generated and vendored files remain
  in the explicit parent-file audit count; the dependency gitlink is recorded
  separately.
- Initial architecture surface: separate `real_explicit` and
  `real_algorithm` digit generators feed decimal boundary/interval refinement,
  while `real` builds arithmetic/comparison expressions over them. The audit
  will establish the actual convergence, carry/sign, domain, termination,
  ownership, and precision-limit contracts before treating any design comment
  or generated documentation as evidence.
- The README and all 11 implementation headers were read line by line (12 files
  / 2,780 physical lines). Explicit finite decimals and function-pointer digit
  streams produce nested decimal-prefix intervals; arithmetic advances every
  leaf by one digit per root step and recomputes ordinary endpoint arithmetic.
  There is no sensitivity scheduling, shared subexpression representation,
  memoized result, division, or elementary-function layer.
- The representation admits invalid public states. Default `real`,
  `real_explicit`, and `real_algorithm` objects leave discriminants, callbacks,
  or digit storage unusable; initializer-list constructors accept empty lists
  and digits outside 0--9 without normalization. String parsing merely
  subtracts `'0'` from arbitrary characters inside a non-throwing expression,
  and signed textual zero preserves a negative sign. Direct single-header
  inclusion also appears cyclic and will be compile-probed.
- Precision/refinement contracts are internally inconsistent. Explicit
  batched refinement repeats digit `_n` instead of consuming `_n+i` unless it
  happens to jump to full precision. Algorithmic `set_maximum_precision`
  writes an instance member that `max_precision()` ignores; zero global
  precision underflows the `cend` jump. Top-level comparison skips the initial
  interval, advances beyond the nominal count, and uses the larger rather than
  the tighter of two instance caps. Operation `cend()` recursively uses leaf
  ends and ignores the operation's instance cap.
- Ownership and code-size architecture is strictly weaker than Hyper's shared
  immutable DAG. Every arithmetic construction recursively deep-copies both
  operands; compound assignment overwrites existing raw child pointers without
  freeing them; assignment neither clears old children nor copies the
  algorithmic value/precision state; and operation iterators allocate a
  parallel recursive iterator tree with no destructor. Header-defined free
  functions, stream operators, utilities, and the global irrational constant
  are not `inline`, creating cross-translation-unit ODR failures.
- Candidate disposition at the source checkpoint: decimal-prefix caching and
  endpoint propagation are already subsumed more compactly and with a stronger
  error contract by Hyper's batched dyadic approximations and shared node
  caches. No implementation change is justified before build and adversarial
  validation.
- Root build metadata and all 21 test files were read line by line, adding 26
  files / 8,718 lines and bringing credited coverage to 38 files / 11,498
  lines. The CMake project glob-builds 19 Catch executables, searches for an
  unused Boost.Test dependency, offers no install/export target, and makes its
  Debug configuration depend on the separate `cmake-modules` gitlink. CI was
  pinned to Trusty, GCC 7, and Boost 1.67, with Linux explicitly allowed to
  fail. The tests contain many repeated hand-written endpoint tables but use
  no independent arithmetic oracle, normally refine only four digits, and omit
  malformed/default inputs, multi-step refinement, assignment/self-aliasing,
  copied precision, cap enforcement, iterator destruction, multi-TU linkage,
  and direct-header inclusion. They even require `precision_exception` when
  comparing equal, finite explicit values whose textual length exceeds the
  global search cap, thereby encoding avoidable incompleteness.
- An untouched GCC 15.3 Release configure succeeds, but compilation stops in
  the vendored 2018 Catch header because current glibc's `MINSIGSTKSZ` is not a
  constant expression. Defining Catch's documented
  `CATCH_CONFIG_NO_POSIX_SIGNALS` compatibility switch changes only the test
  runner; all 19 executables then build and 19/19 CTest cases pass. This green
  smoke result does not exercise the omitted public contracts.
- Direct-header syntax probes pass for only six of the eleven implementation
  headers. `boundary.hpp`, `boundary_helper.hpp`, `interval.hpp`,
  `real_algorithm.hpp`, and `real_helpers.hpp` fail in isolation through an
  include cycle and missing declarations/includes. Linking two otherwise valid
  translation units that include `real.hpp` fails with multiple definitions of
  boundary/vector helpers, stream operators, and other non-inline header
  functions. The advertised header-only library therefore is neither
  self-contained nor ODR-safe.
- Focused public-API probes confirm the source defects. A two-step explicit
  refinement of `1.23456` returns `[1.22,1.23]`, excluding the exact value.
  Terminal refinement of `0.9` remains `[0.9,1]`, so even `0.9 == 0.9` reaches
  `precision_exception`. Textual `-0` compares less than `0` and unequal to it.
  Malformed strings (`1a2`, `1e2`, `--1`, and a leading space) and out-of-range
  initializer digits are accepted; sign-only/dot-only strings leak
  `std::out_of_range`; empty/default explicit values crash under Clang
  ASan/UBSan. An algorithmic value assigned into an explicit one segfaults in
  `real_algorithm::operator[]`; copy and assignment reset an instance cap of 2
  to 10; setting a direct algorithm's cap to 2 still yields cap/end length 10;
  comparing equal algorithms capped at 1 and 4 calls each callback five times;
  and `x += x` maps an expression equal to 3 to 8 rather than 6.
- The independent finite-decimal terminal oracle enumerates every ordered pair
  from -1.20 through 1.20 at hundredth resolution for addition, subtraction,
  and multiplication: 174,243 enclosure/exactness checks. Addition and
  subtraction each return 16 falsely exact intervals that exclude the true
  result (32 total), all involving canonical zero and negative hundredths;
  multiplication has no exclusion in this domain. The cause is magnitude
  ordering by exponent before recognizing zero: for example `-0.08 + 0`
  becomes exact `0.92`. Separately, 5,728 finite operations fail to collapse
  (1,912 additions, 1,912 subtractions, and 1,904 products), chiefly exposing
  the terminal-nine carry path. A second oracle checks five successive prefix
  intervals for all the same operations—871,215 checks—and finds the same 32
  bad cases at every step: 160 non-enclosures split 80/80/0 by operation and
  32 at each refinement depth.
- Valgrind makes the ownership cost deterministic. Constructing 100 operation
  iterators leaves 52,800 bytes in 1,000 blocks (49,600 bytes directly lost in
  200 blocks and 3,200 indirectly in 800); assigning one expression tree over
  another 100 times leaves 22,128 bytes in 410 blocks (21,296 directly lost in
  202 and 832 indirectly in 208). These are ordinary public paths, not merely
  shutdown globals. Deep copy, one-digit-at-a-time lockstep evaluation, raw
  ownership, and duplicated generated operators offer no performance, memory,
  binary-size, or code-size mechanism worth transferring to Hyper.
- Extraction decision: retain no Boost.Real 2018 implementation change. Its
  useful high-level notion—nested, demand-driven certified intervals—is
  already Hyper's stronger invariant, while every distinctive realization is
  less exact, less complete, or materially more expensive.
- `doc/Doxyfile` (2,427 lines), the vendored Catch 2.3.0 amalgamation (13,358
  lines), and the 1,720-line Inkscape SVG were read completely. The Doxygen
  configuration merely generates HTML/tree/include graphs from the already
  inventoried inputs. Catch contributes generic test/report/CLI machinery; its
  adaptive `BENCHMARK` macro is unused by this project, and its POSIX signal
  stack definition is the exact modern-glibc build failure already isolated.
  Neither file contains a scalar algorithm or transferable benchmark method.
- All 51 tracked PNGs were dimension-inventoried and visually inspected in two
  contact sheets. The 24 unique class/containment diagrams, two equal logos,
  and 25 Doxygen UI sprites account for every image. The diagrams only confirm
  the boundary-to-interval-to-iterator containment and exception inheritance
  visible in source; there is no hidden numerical design.
- Fifteen generated overview/index pages and all eleven generated source pages
  were then read in full. The latter required explicit character-range reads
  because Doxygen packs headers into single physical lines as long as 96,064
  characters. They reproduce the audited headers exactly. The narrative says
  a radix higher than decimal digits could improve time and memory, but Hyper's
  dyadic/big-integer approximation architecture already realizes the stronger
  form of that idea.
- All 13 member-list pages, all 13 detailed class/struct pages, their 13 data
  files, all 47 graph maps/hashes, all 71 search shards, and every remaining
  Doxygen navigation/search/style/runtime asset were read byte-completely.
  The detail pages expose documentation contradictions rather than hidden
  machinery: defaults are described as valid zero intervals despite empty
  backing digits, refinement is described as strictly nested despite the
  terminal-nine failure, per-instance precision is described despite the
  ignored/static implementation, and arbitrary functors/lambdas are promised
  despite storing a raw function pointer. The final 67-line, 146,331-byte
  `jquery.js` is stock jQuery 1.7.1, jQuery UI widgets, hashchange/scroll and
  PowerTip UI code with no numerical content.
- Manifest reconciliation is exact: 293 parent entries comprise 292 regular
  files totaling 45,762 `wc -l` lines plus the `cmake-modules` gitlink at
  `fcfc0494c45fc24fae39996db658b9bdeeaa4fd8`; every regular parent file and
  every byte of its long generated lines was read. The dependency gitlink is
  recorded but, consistently with the audit boundary used for other
  references, is not misattributed as Boost.Real source. The pinned worktree
  remains clean.
- The current public repository page matches the pinned tree and still labels
  the August 2018 beta as its sole release. Its prose repeats the shrinking-
  interval, accepted-functor, valid-string, and precision claims contradicted
  by the source and probes. The related 2019 project report independently
  reports the same catastrophic raw-pointer/deep-copy memory behavior, then
  describes the shared-pointer/variant rewrite, division, cached iterator-state
  proposal, tree rewrites, base-conversion slowdown, broken irrationals, and
  Google Benchmark work that belong to the separate 2021 repository audit.
- Final Boost.Real 2018 result: closed at 292/292 regular parent files and
  45,762/45,762 physical lines. Retain no code, benchmark, or API change. The
  later shared-DAG and incremental-state ideas are already fundamental to
  Hyper; their concrete successor implementation will be evaluated rather
  than inferred here.
- Next live cursor: Boost.Real 2021.

## Boost.Real 2021 source-audit record (closed)

- Repository: `https://github.com/BoostGSoC21/Real`, commit
  `4db9b8f87b2b71748054190d085c449e866a529b` (master, 2021-08-23). The
  fresh pinned clone is clean.
- Parent inventory: 103 tracked entries comprising 101 regular files and two
  gitlinks: Google Benchmark at `090faecb...` and `cmake-modules` at
  `fcfc0494...`. The regular parent files total 49,841 `wc -l` lines: 19
  implementation headers, 31 tests, 35 comparative-benchmark artifacts, five
  local benchmark files plus their dependency gitlink, Catch 2, build/project
  metadata, Doxygen configuration, and two logo assets. Dependency bodies are
  recorded separately and are not counted as parent Boost.Real source.
- This audit starts from zero despite the 2018 ancestry. It will verify the
  actual enclosure/domain/termination contracts before considering the
  advertised `shared_ptr`/`variant` graph, arbitrary-base `exact_number`,
  rational and division paths, elementary functions, algebraic tree rewrite,
  incremental iteration, or benchmark results transferable.
- Every regular parent file has now been read completely: root/project
  metadata and README (seven files, 679 lines), all 19 implementation headers
  (9,682 lines), all 31 tests (11,282 lines), all 35 comparative-benchmark
  artifacts (6,652 lines), all five local benchmark files (271 lines), all
  three documentation/assets files (4,200 lines), and the vendored Catch 2.9.2
  amalgamation (17,075 lines). That closes the source cursor at 101/101 files
  and 49,841/49,841 physical lines. The two dependency gitlinks remain pinned
  inventory entries rather than parent-source line counts.
- The implementation replaces 2018's raw expression trees with a
  `shared_ptr`/`variant` DAG and caches a precision iterator per node. Copies
  consequently alias mutable refinement history, however, making value copies
  state-coupled and concurrent refinement unsafe; construction also eagerly
  computes an initial interval. Operand refinement remains lockstep rather
  than sensitivity-driven.
- Its custom vector-limb `exact_number` duplicates bigint addition,
  Karatsuba, division, parsing, base conversion, rounding, and formatting.
  Source-level edge defects include unsafe empty/default values and malformed
  exponents, negative-integral `floor` moving one unit too low, and costly
  repeated decimal conversion. `integer_number` additionally has an incorrect
  both-negative subtraction branch, can create negative zero, and returns the
  divisor for some negative exact-multiple remainders.
- Exactness-critical interval defects are visible before probing: a
  multiplication branch assigns a lower bound through an `up_to` temporary;
  even integer powers test an empty remainder even though zero is represented
  by one zero limb; trigonometric extrema are inferred from endpoint/midpoint
  signs and crude width thresholds; and elementary series terminate on a next
  term without a certified tail. Several error-directed `up_to` return values
  are ignored, mutable global caches are precision/direction dependent, and
  `asin(-1)` explicitly returns positive rather than negative pi/2.
- Negative `real_rational` values lose their stored sign when converted into
  mixed operations and comparisons. Other dormant public paths appear
  ill-formed on instantiation, while header definitions include mutable or
  non-inline globals with likely multi-translation-unit ODR failures. Parser,
  self-containment, instantiation, arithmetic-oracle, aliasing, sanitizer, and
  ODR probes remain scheduled after the shipped tests have been read.
- The complete 31-file test suite heavily repeats four-step checks that each
  interval is ordered and no wider than its predecessor, without checking that
  an independent true value is enclosed. Long arithmetic and comparison
  matrices therefore validate internal monotonicity rather than truth. The
  rational test even leaves both-negative subtraction, mixed-sign
  multiplication, and rational/integer division sections empty. There is no
  zero-denominator coverage; powers omit negative bases and zero edges; math
  compares via the same comparator against loose decimal brackets, including a
  very broad `sqrt(4)` lower bound, while the negative `asin` endpoint is
  omitted and an `acos` identity is disabled. One multiplication math section
  assigns new operands but accidentally reuses an old result. No independent
  oracle, signed-rational, negative-zero, alias/history, or uneven-cap test
  closes the source-level risks above.
- An untouched modern build first fails because vendored Catch uses
  `MINSIGSTKSZ` in a constant expression. With only
  `CATCH_CONFIG_NO_POSIX_SIGNALS` and `CCACHE_DISABLE=1`, all 28 executables
  build and CTest reports 28/28 passing in 15.13 seconds (math 9.67 seconds,
  equality 4.79, ordering roughly 4.3 each). This is compatibility smoke
  evidence, not evidence for the missing enclosure contracts.
- The local Google Benchmark drivers time parsing and eager initial evaluation
  while labelling it construction, then label an unused `cend()` or discarded
  comparison as evaluation without optimizer barriers. Equal 100-digit
  comparisons can exceed the global ten-iteration cap. The broader comparison
  harness is not cross-library evidence: random decimal strings can have
  leading/all zero divisors, arrays allocated with `new[]` are released with
  scalar `delete`, and results are discarded. RealLib and iRRAM benchmarks
  construct lazy nodes without forcing values; iRRAM additionally forks a new
  process per iteration while excluding child CPU time. Its result JSON is
  empty. GMP, MPFR, NTL, and mpdecimal use unrelated fixed precisions (including
  an incorrect three-bits-per-decimal MPFR conversion), ranges and operands
  differ, mpdecimal filtered runs can observe an uninitialized global context,
  and the NTL result set is internally inconsistent. All recorded outputs use
  one repetition with CPU scaling enabled; Real old/new results are mixed and
  some key measurements have one iteration. They support no performance claim.
- Catch's own optional benchmark subsystem does have optimizer barriers,
  clock-resolution/cost calibration, warmup, repeated samples, bootstrap
  confidence intervals, and outlier reporting, but Boost.Real does not use it.
  The remaining Catch code is generic assertion, generator, CLI, signal,
  reporter, and formatting infrastructure. Apart from the modern-glibc
  `MINSIGSTKSZ` compatibility break already reproduced, it contributes no
  scalar or evaluation algorithm worth extracting.
- Header/API validation rejects the advertised header-only surface. Directly
  including each of the 19 public headers succeeds for only ten; the other nine
  depend on undeclared exceptions, algorithms, namespaces, or the completed
  `real` template. A normal three-translation-unit program fails to link on
  duplicate `KARATSUBA_BASE_CASE_THRESHOLD`, `append_digits<int,int>`, and
  Champernowne digit-function definitions. Public `to_explicit()` and
  `to_operation()` fail when instantiated, and `integer_number /
  integer_number` is ambiguous because two overloads differ only by return
  type.
- Parser and lifetime sanitizers expose unconditional undefined behavior rather
  than malformed-input trivia. Clang ASan/UBSan reports a global-buffer read
  before `main` while constructing the inline rational constants; the same
  empty decimal normalization is reached by valid strings. Unsanitized builds
  accept `""`, signs, a decimal point, `e`, and incomplete signed exponents as
  zero or one. A default `real`, and a string constructed with an unrecognized
  representation tag, both dereference a null representation and segfault.
  `exact_number("12.30e2").as_string()` also emits an unrelated 28-digit
  integer. Valgrind independently finds the parser read and a one-past-vector
  read in Knuth/single-digit division.
- Exact-integer differential testing is decisive. Exhausting both operands from
  -50 through 50 finds 2,450 wrong subtractions out of 10,201, 207 wrong
  Euclidean remainders out of 5,050, and 1,225 negative-zero division
  representations; addition and multiplication happen to pass that small
  range. With a fixed seed and 500 signed cases through 512 bits, comparison and
  limb-length defects produce 135 wrong additions, 192 wrong subtractions, 84
  wrong remainders, and 121 negative-zero quotients. Multiplication and quotient
  magnitude pass those sampled values, but the combined 411 wrong values make
  the custom numeric base unusable.
- Exact rational arithmetic in isolation passes 200,475 small signed checks,
  locating a separate conversion defect: mixed rational/real operations exclude
  the exact value in 434 of 720 tested intervals and seven of 15 comparisons are
  wrong because a negative rational's sign is discarded. Rational division by
  zero constructs an invalid empty-denominator value; using that value in real
  division segfaults, while the explicit-number zero path does throw.
- The integer-real enclosure oracle checks 8,937 intervals from 250 fixed-seed
  and boundary operand pairs through 256 bits. GCC excludes the true quotient
  in 48 cases and expands 44 supposedly nested intervals. The same source under
  Clang `-O2` throws during construction for all 243 nonzero divisions because
  the exponent guard evaluates `abs(INT_MIN)`; Clang `-O0` instead reaches a
  `string_view` assertion. This is compiler-dependent undefined behavior, not a
  portable rounding policy.
- A second exact oracle covers 90 finite decimals, five base arithmetic forms,
  eight refinement levels, and alias/distinct controls. Across 5,136 intervals
  it finds 56 true-value exclusions and 43 nonnested steps: ten in leaf values,
  five additions, seven subtractions, four divisions, and six each in alias,
  copied, and independently reconstructed self-add/self-multiply controls.
  Multiplication of distinct general pairs happened to enclose the sampled
  truth. Matching alias and distinct counts prove those particular exclusions
  originate in decimal conversion/enclosure rather than operand sharing.
- MPFR at 2,048 directed bits exposes the elementary-function contracts that
  the upstream self-comparison tests miss. Sampled `exp` and `log` intervals
  enclose their references, but `sqrt` excludes two of 36 checks. `sin`, `cos`,
  and `tan` respectively exclude 11/30, 6/18, and 13/30 checks and often reverse
  their bounds. `asin(-1)` excludes all six levels by returning positive pi/2;
  nontrivial `asin`, `acos`, and every sampled nonzero `atan` can exceed a
  12-second bound, with the unbounded run reaching more than 21,800 recursive
  `tan_inverse` frames before stack failure. Integer powers exclude 12 of 126
  checks, reverse 12 intervals, and reject every sampled nonpositive base to
  exponent zero; even powers of `-2.1` are wrong. The source's
  `exact_number("-2").floor()` returns -3. Independently isolated domain guards
  do correctly reject negative square roots, nonpositive logarithms,
  out-of-range inverse trig, and `atan2(0,0)`.
- Copying a value shares its mutable child refinement state. After warming one
  copy, a fresh iterator from its alias disagrees with a cold equivalent at six
  of seven nominal precision levels; a maximum-precision update through one
  alias is immediately visible through the other. A two-thread shared-DAG
  harness under Clang ThreadSanitizer completes with 51 reported races in
  iterator precision, interval limbs, and `exact_number` assignment. This
  invalidates both value semantics and the principal performance proposal.
- The 2021 ownership rewrite does at least close the 2018 acyclic-tree leak. A
  bounded Valgrind run reports zero live bytes and equal 366,207
  allocations/frees, but only after constructing and refining three small
  expressions; it allocates 5,944,770 bytes and reports 18 invalid-read events
  from five contexts. The original 300-expression run remained CPU-bound beyond
  a minute, itself useful evidence of the evaluator's overhead.
- The floating user-defined literal is not exact: routing a long double through
  `std::to_string` turns `0.123456789012345678_r` into exactly `0.123457`, while
  the string literal retains the supplied digits. In a corrected 21-sample
  microbenchmark, constructing two decimals plus a four-operation tree and
  observing its eager initial interval costs a 343-us median; fresh refinement
  through level four costs 1,469 us. Stateful replay appears to cost 355 us,
  but produces the history-dependent trace above. These figures are diagnostic
  only, not a cross-library ranking.
- Size measurements agree with the source review: a stripped `-Os`, section-GC
  program performing one multiplication, addition, and refinement is 208,208
  bytes versus 10,888 bytes for an empty C++ program, with 200,493 versus 850
  bytes of text and 3,664 versus eight bytes of BSS. Its preprocessed source is
  98,689 lines / 2,685,148 bytes. The optional Catch benchmark machinery has
  sound barriers, calibration, repeated samples, confidence intervals, and
  outlier reporting, but none of the project benchmarks uses it correctly.
- Direct comparison with current Hyper closes extraction. Hyper's expression is
  immutable behind `Arc<Node>`; lazily allocated synchronized storage publishes
  only the finest completed scaled integer, derives coarser queries by shifting,
  never lets a coarser concurrent result evict it, and frees the cell with the
  node. Existing focused tests for concurrent clone refinement, cache-warming
  invariance, adversarial out-of-order precision, and scalar/node layout all
  pass. The matching six-case, 100-sample Criterion cache benchmark reports
  roughly 9.10--68.4 ns per hit with calibration and optimizer barriers. Exact
  rationals, bounded structural rewrites, specialized nodes, fused linear forms,
  independent MPFR oracles, explicit bounded decisions, and downstream
  certificate layers already cover every valid architectural motivation here
  without mutable replay.
- Final Boost.Real 2021 result: closed at 101/101 regular parent files and
  49,841/49,841 physical lines. The two dependency gitlinks remain correctly
  inventoried, the pinned reference worktree remains clean, and no production,
  test, benchmark, memory, binary-size, or code-size change is retained.
- Next live cursor: inventory and pin flatsurf `exact-real`, then begin its
  parent source pass from file one.

## flatsurf exact-real source-audit record (complete)

- Repository: `https://github.com/flatsurf/exact-real`, commit
  `fc4104076cb76aa6efe11df6ae15b208b18d12f5` (master, 2025-12-17),
  described as `4.0.1-92-gfc41040`. Release 4.0.1 is
  `0dec0ecf94462dd32470817c2f1a0898fd59864b` (2025-03-26). The fresh
  parent and recursively initialized dependency worktrees are clean.
- Parent inventory: 186 tracked entries: 178 regular files, two `bootstrap`
  symlinks to the root script, and six gitlinks. The regular parent files total
  41,794 `wc -l` lines / 1,847,785 bytes. Category totals are: 17 root files /
  21,882 lines (including the 19,845-line `pixi.lock`), eight GitHub files / 237
  lines, 33 documentation/news files / 3,462 lines, 32 public library-header
  files / 4,395 lines, six benchmark files / 527 lines, 20 implementation files
  / 4,580 lines, 28 C++ test files / 2,728 lines, seven other library build/m4
  files / 1,463 lines, 25 Python/binding/test files / 2,479 lines, and two tool
  files / 41 lines. Both symlink payloads and all six gitlink identities are
  separate inventory entries rather than falsely counted regular-file lines.
- The pinned top-level gitlinks are `spimpl` at `ba1b834c...`, `gmpxxll` at
  `121c0448...`, `hash-combine` at `76f757e2...`, `unique-factory` at
  `c30fa92d...`, Catch2 2.13.9 at `62fd6605...`, and cereal 1.3.2 at
  `ebef1e92...`. Recursive Catch dependencies inside `gmpxxll` and
  `unique-factory` are also checked out at their recorded commits. Vendor test
  frameworks remain dependency inventories; directly consumed support APIs
  will be read where the parent architecture relies on them.
- This is an explicitly generated exact-real system rather than a full
  computable elementary-function tower. Initial extraction questions are
  therefore exactness-first: generator/seed identity, module and number-field
  canonicalization, coefficient normalization, certified Arb/Arf comparison
  and rounding, equality/hash consistency, product-module behavior, and
  conversion/serialization boundaries. Performance candidates include unique
  factories, expression-template rounding, sparse generator coefficients, and
  specialized rational/algebraic fast paths, but none is accepted before the
  complete source and independent validation passes.
- Root progress is 16/17 regular files and 2,037/21,882 lines. All hand-written
  public/build/release metadata is read; only the generated 19,845-line
  `pixi.lock` remains in this category. The project deliberately builds with
  hidden visibility and a Linux version script after historical cross-header
  collisions, and CI exercises debug/release, ASan, UBSan, Valgrind, FLINT
  2.9/3.x, and multiple Sage releases.
- The public `libexactreal/exact-real` surface is complete at 32/32 files and
  4,395/4,395 lines. `RealNumber` is a shared polymorphic generator with
  certified `arb` refinement, optional rational recognition, and canonical
  product/quotient hooks; `Module<Ring>` supplies shared generator bases;
  `Element<Ring>` stores a dense coefficient vector that promotes operands into a
  common module and forms product bases for multiplication. Number fields are
  exact through e-antic but compositum/coercion support is intentionally
  narrow. Arb comparisons return explicit three-valued `optional<bool>` at the
  enclosure layer, whereas exact element ordering repeatedly refines.
- Serialization explicitly preserves shared `RealNumber` and module identity
  through cereal pointer IDs and reconstructs modules through the unique
  factory. This is relevant to canonical DAG persistence, but remains only a
  candidate until factory and load-path invariants are traced and attacked.
- The 15-file Boost.YAP opt-in layer is rejected provisionally as a Hyper
  transfer: it recursively evaluates the right side into a temporary before
  mutating the left, does no reordering or primitive-specialized dispatch,
  exposes template machinery in every consumer, and its own public comments
  say it is not recommended and considered for removal. Its only general idea,
  requiring explicit precision and rounding at evaluation, already exists in
  Hyper's explicit approximation contracts without this compile-time surface.
- `exact-real/exact-real.hpp` includes an untracked
  `number_field_ideal.hpp`; the advertised aggregate header therefore appears
  stale. This is queued for an isolated compile check once the dependency/build
  surface has been established.
- All 20 parent implementation files / 4,580 lines are read. Primitive and
  product `RealNumber` objects cache requested Arf points in two optionals plus
  a mutable hash map, clearing that map wholesale at 128 entries. None of these
  caches is synchronized. Default seed allocation is a non-atomic global
  increment, while generated bits come from finite-period `rand48`; the system
  consequently treats reproducible pseudorandom indeterminates as
  algebraically independent/transcendental even though that is not literally
  a mathematical property of the bit streams.
- Product generators are flattened into sorted `(generator-id, exponent)`
  vectors and interned, giving pointer equality to canonical commutative
  monomials. Transient two-factor keys avoid allocating normalized vectors on
  cache hits, and the last 1,024 products/modules are deliberately retained.
  The approximation path nevertheless expands every exponent as repeated
  multiplication, uses a heuristic floating-point guard-bit estimate, and
  stores degrees/exponents in overflow-prone signed integers. Hyper already has
  immutable structural nodes and bounded exact normal forms; exponentiation by
  squaring and checked exponent domains must be compared against its existing
  integer-power implementation before this becomes a candidate.
- `Element` stores dense coefficient vectors. Multiplication forms and interns
  every Cartesian-product generator even when either coefficient is zero, then
  retains zero coefficients and the enlarged basis until callers explicitly
  invoke `simplify()`. The tests intentionally expect the module rank of a
  numerically unchanged rational-one coordinate to grow under multiplication.
  This is a negative memory/performance result, not a transfer candidate.
- Source inspection found exactness/completeness defects queued for independent
  reproduction: `Arb::operator!=(mpq_class)` executes and returns equality;
  `RationalField::floor()` uses GMP truncation for negative fractions;
  `Element::floordiv(0)` mishandles exact zero; `Module::gen()` can index out of
  bounds; `Module::submodule()` ignores coefficient-ring compatibility;
  promotion can silently drop a nonzero generator when the requested target is
  not a supermodule; multiplication by rational generators other than one is
  unimplemented; and several precision-doubling loops can overflow.
- All 28 parent C++ test files / 2,728 lines are read. They use a 4,096-bit
  approximation from the same implementation as the oracle for lower-precision
  point checks, cover only small product degrees, do not exercise concurrency,
  and omit the rational-Arb inequality path. The floor/ceil test repeats
  `x == 0` in its `else if`, making the intended nonintegral assertion
  unreachable. Serialization checks reconstructed equality but not preserved
  alias identity.
- Directly consumed dependency code is also traced: the 184-line
  `unique-factory` implementation, 82-line hash combiner, 149-line long-long
  GMP adapter, and 455-line SPIMPL header. The factory's mutex does not cover
  deleter-driven cache erasure; its creator-exception path unlocks before
  erasing with a live iterator; and allocation/hash/keep-alive exceptions can
  strand its manually managed lock. This makes the otherwise appealing
  transient-key interning design unsafe under concurrent destruction.
- All six benchmark files / 527 lines are read. The random-Arf benchmark asks
  one object for the same precision repeatedly, so after its first iteration it
  measures cached-value copying rather than bit generation. The Arb create/copy
  cases do not protect their results from optimization and feed 65,536-bit
  inputs into fixed 64-bit operations. YAP is compared both with naive
  arithmetic and a hand-fused `arb_addmul`, but without isolating expression
  construction or code-size costs. Element setup is outside timing, yet every
  timed repetition can hit the retained canonical-module factory; degrees stop
  at seven, global pseudo-random state is not reset, and no sparse, zero-heavy,
  cold-cache, or high-rank case is measured. These results cannot justify a
  Hyper transfer without replacement benchmarks.
- All 33 documentation/news files / 3,462 lines are read, including every line
  of the generated Doxygen configuration. The manuals openly describe the
  generators as pseudo-random but then call the represented values "random
  transcendental"; this does not supply the independence proof on which exact
  coefficient comparison relies. Python documentation warns that the low-level
  cppyy interface is likely to segfault. Release notes say
  `NumberFieldIdeal`, pi, e, and Liouville-number APIs were removed because
  implementations were absent, confirming that the still-installed aggregate
  header's `number_field_ideal.hpp` include is stale. Doxygen extracts all
  public headers recursively with warnings neither comprehensive nor fatal;
  despite a comment that YAP cannot be parsed, the exclusion list is empty.
- Parent source progress is now 135/178 regular files and 17,729/41,794
  physical lines. Finished categories are root metadata except `pixi.lock`, all
  documentation, all public headers, all benchmarks, all implementation files,
  and all C++ tests.
- All eight GitHub/CI files / 237 lines and all seven remaining library
  build/m4 files / 1,463 lines are read. CI spans Linux and macOS, FLINT 2.9 and
  3.x, C++ address/undefined sanitizers, memcheck, and multiple Sage releases,
  but it has no Windows job and no active thread-race job. `configure.ac`
  explicitly disables Helgrind and DRD by default despite the mutable global
  seed and unsynchronized per-object caches. Benchmarks are required unless
  manually disabled. The coverage upload executes a downloaded shell script;
  that is a supply-chain concern, not a numerical donor. The long C++ standard
  and Valgrind m4 files are vendored Autoconf macros and contain no scalar
  algorithm.
- All 25 Python/binding/test files / 2,479 lines are read. The wrapper correctly
  turns enclosure-level unknown relations into a `PrecisionError`, but its
  higher Sage layer admits that finite-period pseudo-random streams are not
  transcendental and nevertheless registers their polynomial structure as an
  integral domain with guaranteed terminating total comparison. That promise
  depends on the very algebraic independence it disclaims. `simplify()` mutates
  a Sage element in place and inversion calls it on the operand; this needs a
  hash/alias probe. Separately created rational generators are deliberately
  noncomposable (`rational(2) + rational(3)` throws) instead of normalizing to a
  coefficient times the unit. The packaging version is still 4.0.0 while both
  Autoconf projects report 4.0.1. Python tests remain small and omit negative
  rational floor, rational-Arb inequality, alias-preserving serialization,
  hash-after-mutation, malformed indices, zero division, and concurrency.
- Both tool files / 41 lines are read. The ASV adapter merely publishes the
  already-audited C++ benchmark executable through `cppasv`; it does not add a
  cold-cache setup, independent oracle, or optimizer barrier. The sole
  Valgrind suppression concerns the platform `memmove` interceptor.
- The generated `pixi.lock` was consumed in full at 19,845/19,845 physical
  lines and audited structurally rather than treated as source code. YAML
  duplicate-key checking and schema/reference validation found 13 environments,
  890 unique package records (888 Conda and two PyPI), and 6,490 environment
  references: every reference resolves, every record is used exactly once by
  identity, every Conda URL is HTTPS with valid SHA-256 and MD5 syntax, and the
  sole VCS dependency is pinned to a 40-hex commit. The only unhashed record is
  that pinned `cppasv` Git source; the other PyPI record is the local editable
  4.0.0 wrapper. The lock's size comes chiefly from three-platform Sage 10.0--
  10.5 solutions; it contains no runtime scalar technique.
- The flatsurf parent pass is complete at 178/178 regular files and
  41,794/41,794 physical lines, plus both symlinks and all six top-level
  gitlinks inventoried. No reference source was modified.
- The checksum-verified official Pixi 0.42.1 binary created the declared
  `libexactreal-flint-30` environment with FLINT 3.0.1, e-antic 2.0.0, Boost
  1.85, and GMP 6.3. The untouched release-mode library and all 12 upstream
  C++ test executables build and pass. This is a smoke baseline rather than an
  oracle: the build defines `NDEBUG`, and important public invariants are
  guarded only by a project assertion macro that therefore disappears.
- Independent release probes confirm the source findings. Rational Arb
  inequality returns `true` for `1/2 != 1/2` and `false` for `1 != 2`;
  rational floor returns zero for `-1/2`; promoting the element one into an
  invalid empty target silently returns zero; and the integer module generated
  by -1 throws rather than producing its valid one. The advertised aggregate
  header cannot compile because it includes removed
  `number_field_ideal.hpp`.
- A rank-one element can be constructed with a zero-length coefficient vector
  in release mode and reports itself as zero while retaining rank one. Calling
  `gen(rank)` corrupts the heap; system Valgrind reports ten invalid GMP
  read/write contexts directly rooted at `Module<RationalField>::gen()` at
  `module.cc:190`, plus five leak contexts (including FLINT cache storage).
  Exact-zero floor division does not merely refine forever:
  the code treats `optional<bool>{false}` as truthy, divides by zero, and
  aborts while converting an infinite/NaN bound to an integer. The latter is a
  strong API warning against boolean-context conversion of tri-state results.
- All 12 ordinary upstream C++ test programs also pass under Valgrind 3.27.1,
  with 182,883 assertions and no reported errors or leaks. The Python wrapper
  was read fully but its Sage runtime suite has not been run; its hash method
  delegates to the C++ value hash, and a direct copy/hash/simplification check
  preserves both hash and value at ranks 1, 4, 16, 32, and 64. The suspected
  hash-corruption consequence is therefore not established. Concurrency and
  extreme-precision concerns remain source findings, not new runtime claims.
- A further benchmark defect is now explicit: `elements()` prepends rational
  one to `gens`, then `monomial()` starts at index zero. The nominal first
  variable is therefore the constant one; all one-variable cases are rational,
  and the last generated random variable is never used. This invalidates the
  intended variable-count comparisons independently of cache effects.
- A separate numerical benchmark driver uses known exact-one inputs, correct
  generator indexing, observable results, prior warmup, and seven measured
  repetitions pinned to CPU 6. Squaring rank-64 one creates 2,080 coefficient
  slots, taking a median 2.414 ms; simplifying the input first gives 400 ns.
  Requesting its 128-bit ball costs 11.076 us versus 185 ns after simplification.
  The unsimplified coefficient array alone occupies 66,560 bytes. Massif's
  combined 1/4/16/32/64 shape sweep peaks at 2,116,641 live heap bytes plus
  550,759 bytes of allocator overhead. The result reinforces Hyper's existing
  zero-lane pruning; it does not motivate a dense module representation.
- The published project and documentation landing pages at
  `https://github.com/flatsurf/exact-real` and
  `https://flatsurf.github.io/exact-real/` were checked against the pinned
  source. They advertise the same limited random-transcendental model and
  Pixi/Autotools workflow, without an additional scalar algorithm.
- Comparing the product/exponent architecture with current Hyper found an
  actual Hyper exactness bug: after the 65,536-bit eager rational output budget
  is exceeded, `exp_ln_powi` used the negative folded value as a logarithm
  argument. The retained candidate inserts magnitude normalization before the
  logarithm and keeps the original parity restoration. Its independent MPFR
  regression is red before the change and green afterward, and the complete
  Hyper gate passes. This change is independently derived from the arithmetic
  contract; no GPL reference code is copied.
- The power fix is retained in Hyperreal commit
  `8a74b42b9059c1284aa7ca18adf6cfd92e29a66b`; no other repository was changed.
  A 41-batch CPU-6-pinned ABBA recheck with 10,000 iterations per process gives
  fixed/baseline median ratios 0.999871 for positive-base exponent -4001
  (bootstrap 95% interval 0.995213--1.003298) and 0.992886 for exponent 4001
  (0.989492--0.995338). Scaling affects absolute timings, so the conclusion is
  absence of a sustained slowdown, not a claimed performance improvement.
  Corrected negative cases evaluate in roughly 4 us in initial samples; the
  broken baseline has no meaningful evaluation timing. Both positive versions
  perform exactly 2,621 allocations in the 100-evaluation Memcheck workload;
  all three positive/negative runs report zero memory errors and zero lost
  bytes. The stripped identical-driver executable grows from 1,425,280 to
  1,425,504 bytes (+224), with no scalar layout change. Sources and raw evidence
  are `/tmp/hyperreal-power-audit.rs`,
  `/tmp/hyperreal-power-bench-results.json`,
  `/tmp/hyperreal-power-recheck-results.json`, and
  `/tmp/hyperreal-power-memory-*.log`.
- The separate flatsurf 1,024-bit cold-refinement recheck remains noisy:
  median 12.554 us, coefficient of variation 30.4%, eleven repetitions. Its
  cached copy is about 20.6 ns, but no precise cold/cached speed ratio is
  claimed. Raw benchmark sources and results are
  `/tmp/flatsurf-performance.cc`, `/tmp/flatsurf-performance.json`, and
  `/tmp/flatsurf-cold1024-recheck.json`.
- Final transfer disposition: Hyper already uses exponentiation by squaring,
  one-finest synchronized dyadic caches, immutable structural nodes, bounded
  exact normal forms, and zero-lane pruning. Canonical product/module interning
  supplies no measured incremental win sufficient to justify a new retained
  global factory, especially with the reference's independence and lifetime
  assumptions. Dense coefficient/module growth is explicitly rejected by the
  controlled exact-one measurements. Tri-state results and domain guards are
  already explicit in Hyper. The only retained change is the independently
  derived negative-power correction and its regression. All plausible scalar
  transfers now have dispositions. ASan, UBSan, concurrent execution, and the
  large Sage runtime suite are not claimed as executed gates.

## realistic source-audit record (complete)

- Repository: `https://github.com/tialaramex/realistic`, pinned commit
  `d7d292aff5a1fa0add9161cc8bd1e88f90905fee`, 2026-08-13, version 0.8.2.
  Inventory: 16 regular tracked files / 5,794 physical lines, no gitlinks or
  repository-specific agent instructions. Reference source remains untouched.
- Fully read `.gitignore` (1), `Cargo.toml` (10), `Cargo.lock` (89),
  `README.md` (118), `src/lib.rs` (14), `src/main.rs` (66), and
  `src/problem.rs` (50): 7 files / 348 lines. Apache-2.0, edition 2024,
  `num` 0.4.3; the pinned lock uses `num-bigint` 0.4.6. The advertised layers
  map Boehm's BoundedRational/UnifiedReal/CR to Rational/Real/Computable.
  The README disclaims performance measurements and promises ordinary
  arithmetic errors as `Problem`, plus closest-binary floating conversion.
  The CLI retains all named answers and does not install an abort signal.
- The remaining nine files are now read in full: `src/computable.rs` (784),
  `src/computable/approximation.rs` (399), `src/computable/format.rs` (563),
  `src/rational.rs` (955), `src/rational/convert.rs` (249), `src/real.rs`
  (1,290), `src/real/convert.rs` (443), `src/real/test.rs` (277), and
  `src/simple.rs` (486). Parent coverage is 16/16 files, 5,794/5,794 lines.
  The public GitHub landing page was checked against the same README.
- `Computable` owns a boxed recursive approximation enum and a `RefCell`
  one-best cache. Deriving `Clone` recursively duplicates both the expression
  tree and every cached bigint. `Real` always carries that payload, even for
  rationals; formatting clones/folds the complete value each time. Magnitude-
  sensitive multiplication, unary squaring, Newton square roots, prescaled
  Taylor kernels, and binary exponentiation are already represented in Hyper,
  which additionally has shared immutable nodes, synchronized lazy caches,
  compact rational values, and bounded/iterative structural paths.
- Cancellation is not an error result: series loops break with partial sums,
  some kernels return the integer one, and `approx_signal()` publishes these
  unfinished results into the ordinary cache. Real's abort documentation does
  warn that aborted calculations can be incorrect, but clearing the signal
  cannot repair an already populated same-precision cache. Hyper's existing
  cache-publication guard and `aborted_approximation_is_not_shared_through_cache`
  regression already cover this distinction.
- Numerical source concerns, not yet all runtime claims: `sign()` gives up at
  about 2,000 bits and returns `NoSign`, while arbitrary powers interpret that
  as mathematical zero. `Class::is_non_zero()` unconditionally returns true,
  including the unknown `Irrational` catch-all, so `definitely_not_equal` can
  make an invalid assertion. The negative-pi sine reduction loses the input
  sign. The generic negative-base logarithmic-power fallback has the same
  magnitude omission just fixed in Hyper, while its unknown-sign negative-
  exponent fallback negates a power instead of taking its reciprocal.
- Rational construction canonicalizes but parsing and `MulAssign` do not;
  `is_integer()` assumes the denominator is already one. Proper-fraction
  truncation retains a nonzero sign on a zero numerator. Parsing accepts a
  zero denominator and counts underscore characters in the decimal scale.
  `powi(0, negative)` returns zero; its resource bound counts exponent bits
  rather than estimated output size. Square extraction tests a limited factor
  set and an approximate integer-root proposal, and its public residual
  guarantee is stronger than that algorithm establishes.
- Floating import decodes IEEE fields exactly, but export uses an approximation
  and shifts away a low bit without a tie/sticky-bit decision. Its roundtrip
  tests do not establish the README's closest-binary claim for values between
  floats. Decimal formatting similarly rounds a finite approximation, not a
  certified rounding interval. The parser accepts trailing input, cannot parse
  its advertised `log10` spelling (operator scanning accepts only letters),
  and does not accept underscores in symbols despite its own comment.
- Untouched validation: 133/133 tests pass in both debug and release;
  all 18 doctests pass. No shipped benchmark or CI configuration exists in
  this snapshot. The ordinary release suite also passes Memcheck with zero
  reported errors or definitely/indirectly lost bytes; 48 possibly lost bytes
  are rooted in the Rust test harness, and 2,320 bytes remain reachable. Log:
  `/tmp/realistic-normal-memcheck.log`.
- Independent driver `/tmp/realistic-audit/numerical.rs` passes 49,792 exact
  rational approximation checks (ratio, negate, square, addition,
  multiplication, inverse) and 2,349 directed 1,024-bit MPFR checks (exp, sin,
  cos, sqrt, ln, pi), including cold-to-fine and cached-coarse requests. The
  dependency versions match realistic's lock, including num-bigint 0.4.6,
  num-integer 0.1.46, and num-iter 0.1.45.
- The same finite numerical driver confirms ten wrong negative-angle sine
  answers among 27 exact pi/6 cases, both signed-zero truncation defects,
  three missed parsed integers, noncanonical multiply-assignment, accepted
  zero^-1, wrong decimal underscore scaling (`0.1_2` becomes `3/250`), and
  `sqrt(pi)-sqrt(pi)` being declared definitely unequal to zero. All five
  cancellation/resume cases retain unfinished cached values after the flag
  is cleared. Three of six quarter-ulp tests fail for each of f32 and f64;
  Hyper passes all twelve and all 27 sine checks. Both parser discrepancies
  are confirmed. No third-party crash or memory-corruption reproducer was
  added.
- Hyper's existing rational-oracle (5 tests), representation-state (4 tests),
  abort-cache (1), rational-pi sine (2), and unknown-sign reciprocal-power
  (1) gates pass unchanged. Source comparison confirms canonicalizing
  multiply-assignment, sign/unknown certificates, domain guards, and abort-
  guarded cache publication already address the donor's weaknesses.
- Sixteen bounded benchmark cases use optimizer barriers, warmup, CPU 6,
  eleven ABBA batches, and bootstrap paired-ratio intervals. Typical medians:
  a warmed 33-term clone costs 28.528 us in realistic versus 3.89 ns in Hyper;
  a cached 128-bit result costs 36.7 versus 27.3 ns; cold 1,024-bit cosine
  costs 39.292 versus 20.008 us; and repeated 32-digit formatting of
  sqrt(2)+sqrt(3) costs 14.035 versus 9.570 us. Frequency scaling affects
  absolute values, so paired comparisons are primary. Scalar layouts are
  Real/Rational/Computable = 192/56/64 bytes versus Hyper's 48/8/16 bytes.
  Sources and raw samples: `/tmp/realistic-audit/performance.rs`,
  `/tmp/realistic-audit/measure.py`, `/tmp/realistic-performance-results.json`.
- Memcheck on the bounded 33-term clone workload (100 warmup + 100 timed
  clones plus setup) reports 154,232 allocations / 9,999,561 allocated bytes
  in realistic versus 3,286 / 137,573 in Hyper. Corresponding fresh 64-bit
  square-root workloads report 4,821 / 157,613 versus 6,222 / 255,289. All
  four runs report zero errors and no lost bytes. This includes setup and
  process overhead, not per-operation allocation counts.
- One performance lead remains live: realistic's cold 64-bit sqrt is about
  600 ns versus Hyper's 875 ns in the initial paired suite. Hyper's generic
  integer-root seed always uses roughly 300 operand bits. A local Hyper
  experiment sizes that seed to the request, using four guard bits and final
  rounding; from |A-t^2| <= 1 and integer-root truncation, its error is below
  5/8 output ulp. The first 21-batch ABBA run improves 16/32/64-bit requests
  by about 61/52/21 percent but has a small 140-bit slowdown, so the current
  candidate restricts the new seed to at most 96 requested significant bits,
  retaining the existing Newton threshold and large seed otherwise. The
  unmodified benchmark binary is `/tmp/realistic-hyper-sqrt-baseline`; first
  samples are `/tmp/realistic-sqrt-candidate-results.json`. The candidate is
  NOT yet retained or committed.
- Next live cursor: finish the candidate's directed-MPFR boundary regression,
  run paired timing again with the 96-bit cutoff, check allocations/size and
  full Hyper/downstream gates, then retain or revert it and close realistic.
  Current Hyper edits are only `src/computable/approximation/exp_sqrt.rs` and
  `src/computable/node/tests.rs`; reference source remains untouched.
- Candidate gate update: its 334 directed-2,048-bit MPFR cases pass, covering
  2/3/5/17 scaled from 2^-600 to 2^600, request/seed/Newton boundaries, and
  exact squares plus/minus 2^-256. All 702 all-feature library tests, every
  integration/example/all-target benchmark smoke case, Clippy with warnings
  denied, fuzz-target checks, 24 doctests, 12 Hyperlattice tests, 140
  Hypersolve root-family/residual tests, and six Hyperlimit tests pass. The
  smoke run has generated report-only changes to `benchmarks.md` and
  `dispatch_trace.md`; these must be removed before any commit.
- The 96-bit-cutoff candidate reduces the 200-evaluation square-root
  workload from 6,222 to 3,222 allocations at 16 bits and to 5,022 at
  64 bits; no errors or lost bytes are reported by Memcheck. Its stripped
  combined benchmark binary is 1,567,760 bytes versus 1,567,664 (+96), with
  unchanged scalar layouts. The first cutoff timing pass still reports a
  narrow 140-bit regression despite unchanged high-precision arithmetic;
  a 41-batch follow-up is running after the full build/test workload finishes.
  `/tmp/realistic-sqrt-unrestricted-results.json` and
  `/tmp/realistic-sqrt-cutoff96-first-results.json` preserve earlier passes;
  `/tmp/realistic-sqrt-candidate-results.json` is the live follow-up output.
- Final disposition: retain the demand-sized seed in Hyperreal commit
  `44ce87ed2feb6fbfcd493d649e1648fb667da39e`, with 13 production lines and
  66 regression-test lines added. The 41-batch post-gate run gives candidate/
  baseline median ratios and bootstrap 95% intervals:
  16 bits 0.386460 [0.385709, 0.388920];
  64 bits 0.787106 [0.784345, 0.789575];
  96 bits 0.701362 [0.696917, 0.705926];
  97 bits 0.997438 [0.993122, 0.999286];
  128 bits 0.998204 [0.991337, 1.003825];
  140 bits 1.024059 [1.008818, 1.041454];
  256 bits 1.011839 [0.999237, 1.021221];
  1,024 bits 0.994658 [0.991355, 1.002454].
  The isolated 140-bit slowdown is an explicit performance tradeoff, not
  concealed as noise or a claim of universal speedup. The much larger gains
  and allocation reductions for common small requests justify retention.
  Generated benchmark/trace report changes were reversed using `apply_patch`;
  the committed worktree is clean, and no other Hyper crate was modified.
- All other plausible transfers are closed: deep cloning, unsynchronized
  caches, eager unbounded rational products, fixed-depth best-effort decisions,
  and approximate text/float rounding are not improvements. Hyper's existing
  certificate-aware layered representation, synchronized caches, canonical
  rationals, magnitude-sensitive kernels, symbolic special cases, and binary
  exponentiation already subsume the useful source patterns. No reference
  implementation code was copied.
- Next source cursor: `computable-real` 0.3.0 and its companion `reals` 0.4.0,
  whose primary rustdoc landing pages have been read. Their crates.io metadata
  endpoint currently returns HTTP 403 and deeper docs.rs source pages are
  unavailable through the browsing cache; official release archives or public
  repository metadata are the next retrieval alternatives. This is not a
  blocker for the broader inventory, and no source-read coverage is credited
  for either package yet. Retrieval subsequently succeeded; see the next entry.

### computable-real 0.3.0 / reals 0.4.0 published-archive audit

- Official static.crates.io archives are pinned by the SHA-256 values in the
  inventory table. Their VCS metadata records paths `computable` and `reals`,
  respectively, but neither manifest provides a repository URL. No README,
  license file, independent test directory, benchmark, or CI configuration is
  shipped; MIT is declared in both manifests. Archive membership, not the
  docs.rs generated listing, defines coverage. The VCS JSON files have a final
  non-newline-terminated line, counted as a physical line.
- computable-real: every line of all eight files is read: `.cargo_vcs_info.json`
  (6), `Cargo.toml` (30), `Cargo.toml.orig` (9), `src/lib.rs` (66),
  `src/approx.rs` (76), `src/prim.rs` (293), `src/traits.rs` (281), and
  `src/real.rs` (811), totaling 1,572 physical lines.
- reals: seven complete files read so far: `.cargo_vcs_info.json` (6),
  `Cargo.toml` (33), `Cargo.toml.orig` (10), `src/lib.rs` (65),
  `src/comparator.rs` (97), `src/property.rs` (313), and `src/traits.rs` (293),
  totaling 817 physical lines. Remaining: `src/util.rs` (467) and
  `src/real.rs` (1,695); no unread lines are credited.
- Ordinary offline tests pass for both untouched archives: computable-real has
  one unit test and 31 doctests; reals has one unit test and 25 doctests, using
  a Cargo command-line path patch to the pinned companion. Release tests,
  independent numerical checks, and bounded performance/memory comparisons
  remain pending. Generated Cargo.lock/target artifacts are not release source.
- Low-level source findings awaiting numerical discrimination: every cache miss
  clones the whole boxed parent expression, including existing cached big
  integers, before primitive dispatch. `Approx` exposes value/precision fields
  and arithmetic that does not itself preserve its documented error contract;
  callers manually budget guards. Integer square-root seeds halve the operand
  magnitude before sizing a floating seed, apparently mis-scaling large values.
  Negative outer-range asin recursively invokes itself unchanged (source-only;
  no crash reproducer). Tolerance comparison directly orders approximations
  without a separating error margin; bounded comparison may confuse unknown
  with equal. Decimal parsing removes multiple decimal points, and nonfinite
  binary64 imports lack domain rejection. Hyper already has shared DAGs,
  certificate-aware comparisons, checked domains, and certified conversions.
- The companion comparator's equal-factor monotonic branch uses the sign of
  the computable factor instead of the rational multiplier. Negative multiples
  of positive monotonic functions therefore appear to be ordered backwards;
  numerical checks are pending. Symbolic tags carry useful monotonicity and
  magnitude information, but public property arguments are only partially
  validated and `Irrational` is universally considered nonzero. The full Real
  construction paths must be read before deciding whether this can misclassify
  a normally constructed value.
- Source coverage update: all 467 lines of reals `src/util.rs` and all 1,695
  lines of `src/real.rs` are now read, closing the nine-file / 2,979-line
  published archive. Utility findings: decimal parsing adds the positive
  fraction to a negative whole part; the private rational-power helper swaps
  even and odd cases; common-power handling of reciprocal rationals loses
  the exponent sign; exact roots use a heuristic computable candidate followed
  by an exact power check. The latter check certifies success, but failure of
  an approximation search cannot certify irrationality, as `Real::pow` assumes.
- Real construction findings: the generic product both folds rational factors
  into its computable operands and retains their product as the outer factor;
  sqrt of a scaled tagged exponential assigns the unscaled exponential tag.
  Inverse immediately replaces a newly derived Exp tag with generic Irrational.
  Negative twelfths use a signed remainder and lose their fast symbolic route.
  Same-kind cases in `definitely_independent` are unreachable after its initial
  equal-kind return; its Exp/Ln independence comment does not supply a valid
  Lindemann-Weierstrass proof. No unproved theorem is transferred to Hyper.
  Numerical confirmation and comparison against Hyper remain in progress.
- Independent finite driver `/tmp/published-reals-audit/numerical.rs` now
  completes: 28,128 exact-rational approximation checks and 1,494 directed
  1,024-bit MPFR checks pass across arithmetic, exp/sin/cos/sqrt/ln/asin/atan/pi
  within the bounded domain. The separately varied square-root magnitude
  family fails 32 of 56 checks (e.g. sqrt(65536) approximates as 128 at unit
  precision instead of 256). Tolerance comparison gives a strict order for
  equal rational expressions in 34 of 900 checks. Negative outer-range asin
  remains a static finding only; the driver does not exercise its recursion.
- Higher-level finite checks confirm: two of four signed-decimal parses are
  wrong; multiple decimal points are accepted by the low-level parser;
  24 of 30 exp(integer*ln(integer)) identities use the wrong rational power;
  all four negative-factor monotonic comparisons are reversed (all four
  positive controls pass); all three scaled generic products apply factors
  twice; and all three scaled-exponential sqrt/log identities lose the scale.
  Hyper passes the 30 power, eight comparison, four parse, three product,
  three scaled-exponential, and three scaled-pi controls. An initial harness
  error compared Hyper's mixed-number display strings to num's improper-
  fraction strings; replacing that with exact rational equality eliminates
  those three false alarms. Hyper's `into_computable` extracts only the basis,
  unlike `fold`, so its scaled-pi square-root path is also confirmed sound.
- Both untouched release unit suites pass. Normal Memcheck on each reports
  one 48-byte possibly-lost allocation in Rust's test harness, no definite or
  indirect loss, and no invalid-access error; the default error-exitcode is
  consequently 99, not a clean zero-error run. The full independent numerical
  executable finishes under Memcheck with zero errors and no lost bytes
  (12,320 bytes remain reachable in library/global caches). Logs:
  `/tmp/computable-real-normal-memcheck.log`,
  `/tmp/reals-normal-memcheck.log`, and
  `/tmp/published-reals-numerical-memcheck.log`.
- Sixteen bounded paired benchmark cases use the same num-bigint 0.4.8,
  num-integer 0.1.47, and num-iter 0.1.46 for both libraries, CPU 6, optimizer
  barriers, warmup, eleven ABBA batches, and 5,000 bootstrap resamples.
  Representative medians: cloning a warmed 33-term chain is 13.731 us versus
  Hyper 3.47 ns; cached reads are 18.22 versus 19.70 ns; cold 1,024-bit cosine
  is 37.547 versus 15.922 us. Cloning sqrt(2)+sqrt(3) at the high-level Real
  costs 275.63 versus 12.27 ns. The repeated clone-and-refine workload takes
  657.053 us versus 24.79 ns at 33 terms: this explicitly measures repeated
  refinements of clones, where Hyper shares the first warmup's finer cache
  but the donor recomputes it for every independent clone. It is not a
  cold-kernel comparison. Layout Real/Rational/Computable is 184/64/48 bytes
  versus Hyper's 48/8/16. Raw paired samples:
  `/tmp/published-reals-performance-results.json`.
- A bounded performance lead merits one experiment before final disposition:
  cold sqrt(2) at 64 bits is 478.94 ns versus Hyper's 658.24 ns, although
  Hyper is faster at 256 and 1,024 bits. The donor uses a smaller floating
  seed plus Newton, but its floating conversion fails the magnitude checks.
  No floating code is copied. A one-line Hyper experiment lowers the existing
  integer-seed/Newton crossover from 140 to 56 significant bits; this may
  keep the guarded integer seed within a native-width integer root and then
  use the already-proved Newton path. The 334-case directed-MPFR seed gate
  passes. Baseline executable: `/tmp/published-reals-hyper-sqrt-baseline`.
  Eleven-batch timings are running in `/tmp/published-reals-sqrt56-results.json`.
  This candidate is NOT retained; the only Hyper edit is that threshold.
- Candidate update: the 56-bit threshold has a clear ~29% slowdown at the
  adjacent 57-bit request and is rejected. A second threshold at 59 bits
  leaves 56/57/59-bit controls effectively unchanged and improves 60/64-bit
  paired medians by ~25/24%; 128/256-bit controls also improve in the first
  eleven-batch run, and larger controls show no reliable regression.
  Samples: `/tmp/published-reals-sqrt56-results.json` and
  `/tmp/published-reals-sqrt59-results.json`. Absolute times vary strongly
  across batches, so paired ratios, not ratios of independent medians, are
  primary. The first hypothesis of a native u128 root was inaccurate:
  num-bigint 0.4.8's inspected sqrt implementation has a u64 fast path and
  floating estimates corrected by exact integer Newton iterations otherwise.
  The useful boundary here concerns seed/root and division limb widths.
- At 64 bits the 59-bit candidate reduces the 200-evaluation workload from
  3,818 allocations / 173,647 bytes to 2,618 / 132,033, with zero Memcheck
  errors or lost bytes. Its combined driver's ELF text is 16 bytes smaller,
  data unchanged; bss padding increases 16 bytes, not a semantic memory change.
  Existing donor benchmark memory counts: the 200-clone/33-term workload
  allocates 131,889 times / 11,081,399 bytes versus Hyper 1,222 / 120,051;
  the donor sqrt workload allocates 3,617 times / 319,147 bytes. All four
  workload logs are clean. Counts include warmup, setup, and process overhead.
- The live candidate is now threshold 59, with redundant <=96 seed dispatch
  removed and the correctness regression expanded from 334 to 678 directed-
  MPFR cases, including odd magnitude exponents and all 56--64-bit crossover
  neighbors. Full Hyper/all-feature/all-target and downstream tests, Clippy,
  doctests, and fuzz-target checks are running. A final 41-batch timing pass
  is configured but will run only after these competing workloads finish.
  The candidate remains uncommitted and unretained.

### Next-source inventory checkpoint

- `computable` 0.1.0 has 59 regular archive members and 12,421 physical lines,
  with no symlinks. Its manifest identifies https://github.com/evgunter/computable.
  Read in full: `AGENTS.md` (13), `STYLE.md` (57), `Cargo.toml` (124),
  `Cargo.toml.orig` (77), and `.cargo_vcs_info.json` (6). No algorithm lines
  are credited yet. Next source cursor is root documentation and lockfile,
  followed by binary/interval/node/refinement/operation modules and benches.
- The next two Rust repositories are already cloned and pinned as recorded
  in the inventory. They remain unread; retrieval does not imply coverage.
- computable additional full-file reads: README (112), DESIGN_NOTES (91),
  TODOS (68), src/README (58), EXPERIMENT_RESULTS (455),
  docs/pub-sub-refinement-experiment (180), Cargo.lock (710), clippy.toml
  (43), LICENSE (21), .gitignore (5), six .claude command files (one line
  each), src/lib.rs (90), src/error.rs (102), src/concurrency.rs (23),
  src/node.rs (240), and src/computable.rs (374). Total: 26 files / 2,855
  physical lines. The command templates are audited data, not invoked tasks.
- Initial architecture: dynamic shared nodes, exact dyadic lower/width bounds,
  explicit monotone-state refiners, snapshot caching, and external per-node
  threaded scheduling. Prior experimental performance numbers in the docs
  are not independently reproduced yet. Their key lesson is that untargeted
  parallel refinement can overshoot precision badly; demand budgets and early
  stopping matter more than maximizing worker activity. Hyper already pulls
  child precision from certified kernels. A Relaxed cancellation flag is not
  automatically a correctness defect: the historical document's proposed
  Acquire/Release change needs a real publication dependency before transfer.
- TypedBaseNode commits new state only after validating nested bounds, a
  useful existing invariant in Hyper's cache publication. Node caches can
  remain stale after child refinement (documented; still valid enclosures),
  and refinement activation precedes fallible graph construction without a
  cleanup guard (source-only observation; no hang reproducer). Public bounds
  and closure contracts establish inclusion/convergence by caller promise,
  not a machine-checked proof. Next cursor: `src/ordered_pair.rs`, binary
  modules, and refinement/operation machinery. No computable tests had run
  at this initial checkpoint; subsequent coverage and tests follow below.
- Additional complete computable reads: ordered_pair (141), binary.rs (373),
  binary/{binary_impl (411), display (146), error (79), shift (90), reciprocal
  (252), ubinary (272), uxbinary (322), xbinary (335)}, sane (314), test_utils
  (134), ops.rs (26), ops/{base (35), arithmetic (178), inv (420), pow (311),
  nth_root (560)}, refinement (977), binary_utils.rs (12), and
  binary_utils/power (229). Coverage is now 47 files / 8,472 lines. Next:
  bisection, pi, sin, examples/analysis, and all eight benchmark files.
- Untouched computable debug validation: `cargo test --locked --lib --quiet`
  passes all 204 tests. Offline initially lacked anstyle 1.0.13; the approved
  network retry fetched dependencies and preserved Cargo.lock. Release,
  doctest, independent numerical validation, and benchmarks remain pending.
- Static computable findings awaiting finite numerical validation: root
  bisection captures the midpoint of an input enclosure and then treats it
  as the exact target despite later input refinement; normalized prefix
  recovery may lose the normalized lower endpoint's exponent. Reciprocal's
  64-fractional-bit seed can become zero for a large positive input, which
  is then a Newton fixed point. A cached coarse expression whose children
  have independently become exact may skip every zero-width leaf and fail
  a bounded refinement request despite exact available inputs. These are
  numerical correctness/completeness checks, not crash or hang probes.
- Additional architecture comparison: arbitrary-size dyadic exponents avoid
  fixed exponent overflow but ordinary comparisons can eagerly shift very
  large mantissas. Lower/width bounds lose finite one-sided information at
  an infinite lower endpoint. One OS thread per refiner, per-round channels,
  and width-only demand budgeting bring overhead and can over-refine low-
  sensitivity branches. Hyper already has certified demand-driven kernels,
  transactional publication, and shared finest caches. The graph-construction
  cleanup observation above is only a future-maintenance concern: the
  current Result-returning graph builder always returns Ok, so it does not
  establish a present failure path.

### Published Rust pair: final Hyper crossover qualification

- The 59-bit square-root candidate passes all 702 library tests and every
  all-target/all-feature integration, example, and benchmark-smoke target;
  strict all-target/all-feature Clippy, formatting, fuzz-bin compilation,
  19 default and 24 all-feature doctests also pass. Selected consumers pass:
  140 Hypersolve, six Hyperlimit, and twelve Hyperlattice tests. The expanded
  678-case directed-MPFR test and all 51 external Hyper numerical controls
  pass. The generated benchmark/dispatch report changes are removed exactly.
- Final 41 paired ABBA batches, CPU 6, warmup/adaptive 30-ms batches and
  5,000 bootstrap samples ran after all audit builds/tests ended. Candidate
  ratios at 16/32/56/57/59/60/64/96/128/140/256/1024 bits are respectively
  0.9850 / 1.0166 / 1.0118 / 1.0090 / 1.0191 / 0.7485 / 0.7546 / 0.9985 /
  0.9703 / 0.9797 / 0.9406 / 0.9765. The 60/64-bit 95% paired intervals
  are [0.7425,0.7536] / [0.7492,0.7591]. There are small measured slowdowns
  at 32 bits [1.0056,1.0296] and 59 bits [1.0031,1.0642]; these controls
  use the same mathematical seed algorithm, but instruction layout/dispatch
  and system variability still have a measurable cost. Do not call them
  regression-free. Raw data: `/tmp/published-reals-sqrt59-final-results.json`.
- The ~25% local speedup, 31% fewer allocations and 24% fewer allocated bytes
  in the bounded 64-bit workload, and improvements at 128--1024 bits justify
  the small control tradeoffs. Final stripped combined driver is 1,674,376
  bytes versus baseline 1,674,392 (-16); scalar layouts are unchanged. The
  patch adds six net source/test lines across two files. No donor code is
  copied, and no representation, domain, or precision contract is relaxed.
- Retained in Hyperreal commit `93c44ab19b966b4e92c1644ad47bc368391a7d57`.
  This closes the computable-real/reals source, validation, benchmark, and
  transfer comparison. Their symbolic/fact claims, floating sqrt seed, deep
  cloning, unsynchronized cache, and parser/comparison designs are rejected;
  useful certified precision budgeting and shared symbolic factors are
  already present in Hyper. No other crate changes or pushes were made.

### Computable: complete source checkpoint

- Final complete reads: binary_utils/bisection (873), ops/pi (490), ops/sin
  (1,392), examples/analysis (524), and benches/{asymmetric_convergence (114),
  common (73), complex (77), integer_roots (67), inv (57), pi (156), sin (65),
  summation (61)}. All 59 archive files / 12,421 physical lines are now read.
- Untouched release tests pass 203/203 (one debug-only case is absent), and
  all ten doctests pass. No reference source edits. Independent validation
  and benchmark/transfer disposition remain unfinished.
- Normalization's claimed enlargement is not guaranteed merely by choosing
  a prefix width several times the input width: an interval can straddle
  any fixed dyadic cell boundary. Public normalize_bounds also cannot
  represent a zero-crossing interval in its one-cell prefix form. The
  zero-crossing-specific wrapper uses proper outward endpoint rounding,
  but the same-sign path still uses the unproved cell selection.
- Pi's raw Machin series uses directed term reciprocals plus an explicit
  truncation bound; its public node subsequently applies that normalization.
  Sine tracks pi uncertainty through range reduction and evaluates both
  extrema when necessary, but some branch tests permit inputs just outside
  a claimed monotonic interval. Its high-precision tests compare width or
  ~52 bits against f64, not a full-precision independent enclosure oracle.
  The example's `contains_zero` checks only the lower sign. Performance
  benchmarks construct fresh balanced graphs and use output barriers, but
  their accuracy assertions cannot establish arbitrary-precision exactness.
- Independent driver `/tmp/interval-computable-audit/numerical.rs` confirms
  failures in 1,752/7,776 same-sign normalizations, 480/480 zero-crossing
  direct normalizations, and 294/1,056 high-precision wrapper normalizations.
  Exact dyadic add/multiply/negate/cube all pass 196 cases each. Nine of 288
  constant roots exclude the exact mathematical root: e.g. sqrt(17/16)
  converges to an interval ending at 1/2 because prefix recovery loses the
  normalized lower endpoint's exponent. The valid refining representation
  [4,4+2^-n] also produces a wrong square root after its initial midpoint
  is frozen as a target; this is separate from constant-root normalization.
- Raw Machin pi passes all six 2,048-bit directed-MPFR checks; the normalized
  pi node fails at 64 and 128 requested bits. sqrt(pi) fails two of four,
  sine on the quarter-unit grid fails 22/147, and the bounded magnitude sine
  family fails 2/12. Corresponding Hyper checks all pass: 288 roots, six pi,
  four sqrt(pi), 147 grid sine, twelve magnitude sine, and two reciprocal
  query-history controls (459 total).
- Refined reciprocal hypothesis: thirty direct 128-bit requests pass. The
  implementation chooses max(64, requested budget), not always 64. However,
  after an 8-bit query seeds reciprocals of 2^80 / 2^100 with lower zero,
  both later 128-bit requests exhaust four bounded steps; zero is a fixed
  point and the stored upper bound never improves. Hyper's same histories
  produce exact requested integers. A separately refined exact leaf also
  leaves its cached parent stale; all-zero widths skip every refiner and a
  four-step request reports exhaustion despite the available exact answer.
- All-target compile check passes. Normal release Memcheck runs 203 tests,
  with 202 passing and the hard 20-ms parallelism timing assertion failing
  under instrumentation (23.1 ms); no invalid access or definite/indirect
  leak is reported. It reports a 48-byte possibly-lost Rust thread record.
  The independent finite driver also completes, with no invalid access or
  definite/indirect loss, but 48 bytes of Rust thread bookkeeping and 304
  bytes of glibc thread-local storage are possibly lost (exit 99, not a clean
  zero-error run). Logs: `/tmp/computable-normal-memcheck.log` and
  `/tmp/computable-independent-memcheck.log`.
- Nine bounded benchmark workloads use CPU 6, the same num-bigint 0.4.8,
  optimizer barriers, warmup, adaptive batches, eleven paired ABBA rounds,
  and 5,000 bootstrap resamples. Hyper returns center +/- one at one finer
  precision, giving the same 2^-p output-width bound; no looser tolerance is
  substituted. Shared clone is effectively tied (~4 ns); cached 33-term
  root-chain bounds take 123 versus 84 ns. Fresh sqrt(2) takes about
  366/836/1,245 us versus Hyper 0.620/1.095/0.966 us at 16/64/128 bits.
  Fresh inverse(3) and 16-bit sqrt(2)+pi also favor Hyper by hundreds of
  times. These are end-to-end single-core scalar operations, including donor
  worker creation/context switches. Absolute times vary sharply with system
  scheduling; do not interpret the anomalous 64-bit inverse row as kernel
  complexity scaling. Shared pi caches also mean the mixed case is not a
  cold-pi-kernel comparison. Samples: `/tmp/interval-computable-performance-results.json`.
- The 200-evaluation 64-bit sqrt workload allocates 45,010 times / 4,980,820
  bytes versus Hyper 2,818 / 135,228, including setup/warmup. Donor Memcheck
  has the same 48-byte possibly-lost thread record; Hyper has zero errors or
  lost bytes. Handle sizes are 8 vs 16 bytes, but donor Binary / Bounds are
  64 / 120 bytes versus Hyper's 40-byte (BigInt, precision) cache payload.
  The smaller handle does not justify the greater heap and execution costs.
- Final transfer disposition: demand budgeting, shared caches, transactional
  state publication, directed truncation, and error-bound bookkeeping are
  already subsumed by Hyper. The independently unsound prefix normalization,
  frozen-midpoint root, and history-sensitive scheduler/seed are rejected.
  Linear bisection plus per-node OS workers is vastly slower on the measured
  scalar workloads. No production/test patch, binary growth, or code growth
  is retained from computable; Hyper's worktree remains clean.

### Exact-real-rs / Boehm prototype checkpoint

- exact-real-rs complete files: tests workflow (79), .gitignore (3), manifest
  (21), MIT license (21), README (6), src/lib.rs (3): 133 physical lines.
  There are no real representations or arithmetic functions. Untouched
  `cargo test` runs zero unit/doctests and passes; no-default-features build
  passes. Offline lacked ibig, so the approved network retry fetched it.
- boehm_reals full reads so far: manifest, README, ROADMAP, SECURITY, LICENSE,
  .gitignore, src/lib, evaluation/{mod,constants,errors,creals/mod},
  creals/{cr,string_float_rep}. Partial: bounded_rational/br.rs lines 1--1000.
  The CR layer has only a trait and cache scaffold, with no arithmetic
  implementors; the cache/error helpers are not reexported from the private
  cr module. Its written approximation inequality omits the precision scale
  on the error bound. The exact-rational layer remains the implemented target.
  No boehm_reals validation or benchmark has run yet.
- The exact-real-rs published 0.0.0 source at
  https://docs.rs/exact-real/latest/src/exact_real/lib.rs.html was also read
  completely and confirms the same three-line empty library. Publication
  cross-check complete; inventory classification corrected, no transfer.
- boehm_reals remaining reads completed: all 2,027 lines of br.rs, all 1,127
  of add.rs, the 352-line arithmetic macros, bounded_rational/mod (10),
  property_test (67), fuzz target (40), fuzz manifest/ignore/corpus, the full
  689-line lockfile, Dependabot, and all eleven workflow files. All 35 tracked
  files / 5,772 physical lines are now read. Templates/CI commands were
  inspected as data; no GitHub workflows or external issue writes were run.
- Untouched debug gate passes 242 library tests, five property tests, and
  three doctests; one constants import example is intentionally ignored.
  Missing locked dependencies required an approved network retry. Release
  tests and the untouched fuzz target compile check are running.
- Source comparison: despite README's promised None-and-CR fallback, the
  actual rational APIs always return exact rationals (or a zero-domain error),
  with advisory too_big and random 1/16 reduction. No CR arithmetic or
  implemented concrete CR source exists. Helpers cache all fields under one
  mutex, already subsumed by Hyper, but are inaccessible through public
  exports. Repeated scalar/ref forwarding clones and denominator products
  add costs; Hyper already specializes small rationals, equal denominators,
  canonical zero, and deterministic delayed canonicalization.
- Finite numerical concerns to verify: f64 import's inclusive i64::MAX-as-f64
  check incorrectly accepts 2^63 and saturates to 2^63-1 (an existing test
  explicitly expects the wrong value). Float export masks off a rounded
  mantissa carry without incrementing the exponent; its integer fast path
  may also differ from the documented ties-away policy. StringFloatRep
  validates digit characters but not sign range, empty fields, or exponent
  representability; sign zero is ignored during display. No crash or
  resource-exhaustion reproducer is added for these static parser concerns.
- Untouched release gate also passes 242 unit, five property, and three
  doctests (one ignored). The normal 242-test Memcheck run passes assertions,
  reports no invalid access or definite/indirect leak, and has the standard
  48-byte possibly-lost Rust harness thread record (exit 99).
  `/tmp/boehm-reals-normal-memcheck.log`. The untouched fuzz target fails to
  compile: its old Option-based operands no longer match the arithmetic APIs,
  and Add is not imported. No source repair or fuzzing was performed.
- Independent `/tmp/boehm-reals-audit/numerical.rs`: add/sub/mul/cmp/hash each
  pass 2,080 exact-rational checks, divide passes 2,048, decimal truncation
  passes 6,240, and 9,994 deterministic random finite-float imports and
  round trips pass. Boundary checks confirm the 2^63 import error, ten of
  fourteen rounding-carry failures, all six integer ties-away contract
  failures, and three safe formatting/sign-validation failures. This does
  not endorse its incomplete constructive-real layer.
- A new Hyper finding interrupts transfer disposition: its borrowed rational
  f64 path calls BigUint::to_f64, which may return Some(infinity), then accepts
  finite_numerator/infinity as zero. Thus exact values near the smallest
  normal/subnormal boundary can export as zero. The stable owned conversion
  and certified scalar arithmetic are separate paths. A minimal candidate
  rejects non-finite converted denominators, routing them through the existing
  magnitude-aware computable fallback. It is NOT retained yet. Baseline
  executable: `/tmp/hyper-float-denominator-baseline`; new directed 4,096-bit
  MPFR regression covers subnormal values, the rounding carry to normal,
  odd huge denominators, signs, and cache repeatability. Qualification is
  in progress; no new benchmark disposition or commit is claimed.
- Qualification correction: the first candidate passed exact subnormals but
  the new nearest-rounding assertion failed by one ULP near MIN_POSITIVE.
  That is allowed by the existing explicitly lossy single-approximation path,
  not a second production defect. The regression now distinguishes exact
  representable dyadics (bit-exact) from other values (one-ULP comparison),
  verifies repeatability and signs, and covers normal-valued fractions whose
  individual denominators overflow f64 too. All 36 signed cases pass. Shared
  exact-view facts are also checked through the certified dyadic export: a
  sibling's retained fact must not make a fresh Real certify a false zero.
  The independent Boehm driver now measures Hyper's owned nearest conversion
  separately from its borrowed one-ULP conversion. Full gates are running.
- New guard gates pass: 621 default debug library tests; all-feature
  all-target debug suite (703 library tests plus integrations, examples,
  and complete benchmark smoke); 703 release library tests; 19 default and
  24 all-feature doctests; all-feature all-target Clippy with -D warnings;
  untouched Hyper fuzz-target check. Hyperlimit passes 242 library and
  46 selected representation/DOP/degeneracy/predicate tests; Hyperlattice
  passes 19 library and two representation tests. Generated benchmark and
  dispatch-report noise was reversed via apply_patch, not retained.
- Independent Boehm/Hyper numerical-driver Memcheck completes with zero
  errors or lost bytes (544 runtime bytes reachable). All 32 Hyper boundary
  controls pass. The correction's 200 cold subnormal exports plus 100 warmup
  use 1,817 allocations / 174,527 bytes versus 17 / 4,109 in the wrong-zero
  baseline: six additional allocations per corrected cold export. Neither
  executable reports a Memcheck error or leak. Stripped driver size is
  unchanged at 1,375,192 bytes. No scalar or cache layout changes.
- Initial 31-round timing is inconclusive for the subnormal row because one
  noisy calibration chose a tiny baseline batch. It is preserved as
  `/tmp/boehm-reals-guard-bench.json`; a fresh run uses five calibration
  batches, a median, a 20,000-iteration floor, and longer target duration.
  The initial ordinary one-third path costs about 1 ns more; no performance
  win is claimed for this correctness change. Retention still awaits the
  final timing disposition.

### AERN2 source audit opened

- Pinned parent snapshot d1ac3664bfb5c7f70fcf68f7fb412d288def65cb. No
  AGENTS.md or SKILL.md is present. MIME inventory totals 1,483 tracked
  paths, including 15 symlinks, 20 gzip artifacts, 68 PDFs, 40 PNGs,
  73 SVGs, generated JavaScript, and numerous benchmark logs. These are not
  falsely counted as read implementation lines.
- Complete first reads: root README (50), stack.yaml (92), .gitignore (13),
  CI workflow (76), docs/install.md (21), package.yaml for mp (67), real
  (63), affarith (56), and ERC (46), and ERC README (7): ten files / 491
  physical lines. The checked-in configuration builds mp, affarith, and real
  only; net/ERC/function packages are disabled. Its resolver is lts-22.44;
  the comment says GHC 9.6.5 but the resolved compiler is actually 9.6.7.
  explicit CDAR-mBound, collect-errors, and mixed-types-num dependencies
  are part of the representation/proof boundary. No build started yet.
- Complete next reads: mp README (128), real README (345), Real/Type (189),
  Real/CKleenean (199), MP/Float/Type (106), MP/Precision (228), MP/Accuracy
  (282), and Real/Comparisons (345). Total now 18 files / 2,313 physical
  lines. CReal is a lazy list of error-tagged balls at a fixed Fibonacci-like
  precision schedule, capped around five million bits. Accuracy queries
  search forward; ordinary Eq/Ord use one default-precision enclosure and
  raise an exception if undecided. CKleenean exposes finite-stage unknowns,
  merged continuous branches, and diagonal countable choice. These are
  semantics/API ideas to compare, not yet approved transfers.
- Static concerns for ordinary numerical validation: all four mixed
  Precision/Integer-or-Int leq instances call themselves; countable choice
  assumes infinite inner lists while ordinary precision-generated CReals
  have finite lists; unchecked approximation extensions and clearing potential
  errors deliberately weaken the representation contract. No dynamic claim
  is made yet. Only Stack is on PATH; no GHC/Cabal executable was found.

### Finite-denominator guard retained

- The final five-calibration, 31-ABBA-round CPU-6 run with 5,000 bootstrap
  resamples is `/tmp/boehm-reals-guard-bench-final.json`. Candidate/baseline
  ratios: third 1.0150 [1.0095, 1.0268]; word dyadic 0.9928 [0.9822, 1.0043];
  wide normal dyadic 0.9968 [0.9884, 1.0167]; wide fraction 1.0026
  [0.9902, 1.0113]; below-floor underflow 0.9935 [0.9858, 1.0104].
  The roughly 1.5% ordinary-third cost is explicitly accepted for correctness.
- Previously wrong-zero paths now cost about 0.812 us versus 0.143 us
  (boundary dyadic), and 0.931 us versus 0.136 us (tiny odd-denominator
  fraction). These are costs of obtaining the value, not performance gains.
  Their ratio CIs are [5.5367, 5.7176] and [6.7400, 7.0269]. Other user
  tasks were active on the host; paired ordering and affinity reduce but
  do not eliminate system variation. No cross-case scaling claim is made.
- Retention is justified first by preserving finite values and certified
  exact-dyadic exports. Production changes one expression and adds three
  explanatory comment lines; 75 regression lines cover 36 signed MPFR
  cases, cold/repeated views, and sibling-retained exact facts. All recorded
  correctness gates pass; no binary growth or layout change. No donor code
  copied. Retained commit `3f5786794e0ad127c264e44e9c9451ab58b077a7`;
  Hyperreal is clean. No push is requested or run.
- Boehm comparison's eight reused-input cases pass exact rational setup
  assertions and complete 15 paired ABBA rounds each; results are
  `/tmp/boehm-reals-rational-bench-final.json`. Hyper is faster on every
  such case, but the 15-ns large addition is a retained-result cache hit,
  not a cold arithmetic-kernel measurement. The two libraries also use
  different bigint versions (donor 0.5.1, Hyper 0.4.8), so these remain
  end-to-end comparisons, not isolated high-level algorithm attribution.
- Seven fresh-input constructor-plus-owned-operation cases are separately
  measured in `/tmp/boehm-reals-cold-bench-final.json`. The donor wins six:
  Hyper/donor ratios are about 1.50 add-small, 1.26 mul-small, 1.84
  compare-small, 1.55 same-denominator add, 1.79 cross-cancel multiply,
  and 3.26 compare-large. Hyper wins large addition at 0.702. Unlike the
  borrowed workloads, these include new independent input storage and its
  canonicalization on every iteration. No cached result is mislabeled cold.
  Both bigint versions have the same inline one-limb storage. Their local
  BigDigits files have identical SHA-256 b70899171a1850714ede4a1ea2cb387bd9245ab2d8dfa8ae47e533a6cf95acc1;
  a recursive source diff finds differences only in RNG integration and
  lib.rs, not arithmetic kernels. The donor's rational object directly owns
  two bigints and avoids constructor canonical gcd, while Hyper shares
  canonical exact-rational storage and proof/cache facts behind an Arc.
  This identifies a cold-construction tradeoff, not grounds to discard the
  retained representation or adopt randomized reduction. Compact cold
  rational construction remains a cross-reference candidate; no bigint
  migration is suggested by this evidence.
- The reused 2,049-bit clone workload (1,000 iterations + 100 warmup) uses
  1,276 allocations / 321,424 bytes in the donor versus 176 / 31,024 in
  Hyper, including identical setup and independent checking. That is exactly
  one 264-byte extra allocation per donor clone. Both Memcheck logs have
  zero errors or lost bytes. Cold-input memory qualification follows.

### AERN2 core representation checkpoint

- Additional complete files: MP/Ball/Type (280), MP/ErrorBound (231),
  Real/Limit (62), MP/Ball/Limit (155), Real/Elementary (227),
  MP/Ball/Elementary (165), Real/Field (234), Real/FieldTH (205).
  Coverage is now 26 files / 3,872 physical lines, not the full repository.
- Ball stores a CDAR-backed center and a separate 53-bit upward-rounded
  radius; precision reduction explicitly adds center-rounding error.
  Elementary functions lift directed endpoints or center/Lipschitz bounds.
  Limit widens a selected term by the caller-promised 2^-p error; the ball
  functional version backs off demanded accuracy by about 5% until valid.
  These contracts are being compared to Hyper's prescribed integer budgets;
  no semantic or performance transfer is accepted yet.
- Further static numerical concerns: ErrorBound/Integer division is a direct
  self-call; getAccuracy uses floor(log2(radius)) rather than a conservative
  ceiling in the radius >= 1 case and can overstate bits; the MPBall lossy
  approximation's isAccurate flag tests accuracy < request. Unsupported
  inverse trig and hyperbolic functions are explicit errors. Ordinary
  executable validation is pending; no crash-oriented tests are introduced.
- Stack first rejected a not-yet-created root directly under /tmp, so an
  owned mktemp directory `/tmp/aern2-stack.ryVCFK` was created. The subsequent
  build failed sandbox DNS and was retried through approval to download
  the pinned GHC/dependencies. Reference source remains untouched.
- Cold small-add constructor/owned-operation memory run (1,000 iterations
  plus 100 warmup) confirms a real tradeoff: 46 allocations / 4,975 bytes
  for the donor versus 3,346 / 348,175 for Hyper, including identical setup
  and validation; both have zero Memcheck errors or lost bytes. Hyper pays
  three extra allocations per fresh pair-and-result, while repeated shared
  values and cloned large values favor its current architecture. The compact
  cold-rational representation candidate must therefore be isolated and
  evaluated across downstream workloads before its final disposition. No
  speculative storage change or public bigint-type migration is retained.
- AERN2 completed support reads: mp/LICENSE (12), real/LICENSE (12),
  ERC/LICENSE (27), affarith.cabal (85), real/changelog (60), mp/changelog
  (59), and MP/Float/Arithmetic (104), MP/Ball/Field (316). Total now 34
  files / 4,547 physical lines. The underlying elementary kernels are CDAR
  calls, not MPFR despite stale comments; ring propagation includes center
  rounding error, radius products, and sign-aware endpoint multiplication.
  Quotient bounds use a downward-rounded |center|-radius separation in the
  denominator; domain handling must still be checked through CN wrappers.
- Complete MP/Enclosure (334) and MP/Float/Auxi (36): 36 files / 4,917
  physical lines read. The generic API distinguishes hull/union, intersection,
  ball radius expansion, and monotone endpoint propagation. CN containment
  consults optional values but does not itself reject attached errors, so
  numerical validation must inspect both enclosures and error status.
  The approved build has downloaded the Hackage index and is populating
  its temporary cache; no AERN2 test pass is claimed yet.
- Source-attribution correction: inspection of Hyper's actual bigint 0.4.8
  confirms that it already has exactly the same inline digit storage as
  donor 0.5.1. The cold-allocation delta is the rational ownership layer,
  not a missing bigint optimization. Earlier backend speculation is removed.
- Additional completed files: MP/Ball/Comparisons (387), MP/Float/Operators
  (106), MP/Ball/Conversions (164), MP/Float/Conversions (190), Kleenean
  (117), and all MP/Dyadic (630). Coverage now 42 files / 6,511 physical
  lines. Exact dyadic ring operations repeatedly double MP precision until
  the error is zero; rational-to-dyadic conversion verifies the power-of-two
  denominator first. Directed endpoint wrappers preserve explicit rounding.
  Ball union requires overlap and attaches a certain error otherwise, so
  unresolved-branch/coarse-prefix behavior needs dedicated validation.
- Complete Real/Tests (238), MP/Ball/Tests (121), MP/Float/Tests (522),
  and all six mp/real test entrypoints (82 total). Coverage now 51 files /
  7,474 physical lines. CReal accuracy properties explicitly pass when an
  error is returned and otherwise compare the package's own getAccuracy,
  rather than independent radius bounds. MPFloat tests are predominantly
  self-consistency/directed-identity checks, with several accuracy assertions
  disabled (including a CDAR sine TODO); non-finite outputs bypass many
  comparisons. Independent rational and MPFR containment checks are needed
  even if the untouched suite passes.
- The bigint version cross-check is complete: all shared arithmetic source
  files are identical; the entire lib.rs diff is RNG API/documentation plus
  an html-root attribute. No arithmetic-backend-version difference explains
  the measured cold rational storage/canonicalization tradeoff.
- Additional complete wrapper/support reads: MP (40), MP/Ball (68),
  MP/Float (60), Limit (27), Norm (83), Normalize (21), Select (79),
  Real (86), Complex (101), and Continuity/Principles (73). Total 61 files /
  8,112 physical lines. Generic select is a left-first scan at each
  approximation; CReal adds precision-stage interleaving. Complex is a pair
  of CReals, not the single sequence suggested by its header. Continuity
  instrumentation observes demanded sequence indices through unsafePerformIO
  and TVars; this is an operational observation, not a certified modulus.
- The untouched GHC 9.6.7 Stack build completed successfully. Bundled tests
  pass: mp 304, real 30, affarith 124 examples. A seeded rerun also passes;
  a repeat is capturing both stdout and stderr for durable evidence. No
  independent oracle pass or donor-to-Hyper performance gain is implied.
- Completed all ten remaining mp source files: WithCurrentPrec (51), its
  Type (104), Comparisons (199), Elementary (119), Field (209), Limit (64),
  PreludeInstances (99), Ball/PreludeOps (79), Float/PreludeNum (72), and
  Utils/Bench (33). Completed all 12 affarith source/test files (1,254 lines):
  Affine (58), Type (202), Conversions (58), Order (75), Field (101), Ring
  (283), Exp (81), Sqrt (109), SinCos (147), Tests (122), AffineSpec (17),
  Spec (1). Total 83 files / 10,395 physical lines. All active mp/real/affarith
  src/test files are now read, not the parent repository's dormant layers.
- Affine forms retain a bounded map of correlated error coefficients and
  collapse the smallest terms into an upward-rounded residual. The source
  explicitly warns that correctness assumes injective machine-integer hashes
  as error IDs. This identity policy is unsuitable for certified Hyper
  arithmetic. Unary Exp/Sqrt/SinCos paths also do not call the normalizer,
  so maxTerms is not enforced there. Precision wrappers mostly reify a type
  parameter and forward operations; their boilerplate offers no immediate
  runtime-kernel gain over Hyper's integer demand budgets.
- Seeded upstream log `/tmp/aern2-upstream-tests.log` records 458 examples /
  zero failures. Independent driver `/tmp/aern2-audit/Numerical.hs` passes:
  180 rational imports; 6,480 each add/sub/mul; 5,760 division; 108 precision
  reductions; 80 rational-sqrt enclosures; 1,296 CReal rational expressions
  with error status checked; 6,480 each directed-float add/sub/mul and 5,760
  directed division. Oracles use Prelude Rational center +/- radius, not
  donor containment/accuracy predicates. Source and package dependencies stay
  untouched; compile required approval for the system ccache location.
- Ordinary metadata diagnostics confirm radius 3/4 reports bits 1 and
  radius 3/2 reports bits 0: these are rough Accuracy estimates, not certified
  radius <= 2^-bits bounds. getApproximate returns False for an exact ball
  and True for a coarse ball at bits 20; its API is explicitly unsafe/display
  oriented. A nonzero partial branch using pi-pi+10^-30 reports a certain
  union error at bits 5/20 but resolves to 1 at bits 100. The continuous abs
  counterpart succeeds throughout. These observations constrain possible
  transfers; they are not mislabeled failures of a promised strict-accuracy
  or total/discontinuous-branch contract. Log:
  `/tmp/aern2-independent-numerical.log`. MPFR and memory checks follow.
- All 15 remaining tracked files under active packages read (2,265 lines):
  mp/bench/old BallOps (66), BallOpsSpace (75), Values (51), benchOp (95),
  cabal (71); real/examples ClosestPairDist (172), Introduction (314);
  real/attic CIDR (272), ClosestPairA (108), DemoPar (62), EffortConcept1
  (200), Sort (56), bench/BenchMain (270), bench/BenchTasks/Fourier (389),
  Logistic (64). Total 98 files / 12,660 physical lines. Old benchmark code
  refers to removed MPFR/QA-era interfaces and is not part of current Stack
  tests; some parallel forcing instances inspect only accuracy rather than
  the whole result. No old printed timing is accepted as current evidence.
- The attic records useful distinctions between strict and guide accuracy,
  whole-expression retries versus per-node caches, and user-supplied
  Lipschitz bounds on limit functions. The current Introduction calls bits
  requests "guaranteed", in tension with the core Accuracy module's rough
  definition and the measured radius metadata; independent benchmarks will
  explicitly check strict radii. ClosestPairDist's fixed soft partition
  tolerance requires a separation proof before treating its cross-partition
  nearest-pair shortcut as exact; no geometry transfer is accepted from it.
- Directed 2,048-bit MPFR oracle `/tmp/aern2-audit/oracle.rs` verifies all
  420 exported elementary ball enclosures (sqrt/exp/log/sin/cos at five
  precisions). Monotone functions use directed rational input endpoints;
  trig uses a 1-Lipschitz allowance for input rounding. Zero failures or
  indeterminate overlaps. Logs: `/tmp/aern2-elementary-oracle.log` and
  `/tmp/aern2-elementary-enclosures.tsv`.
- Independent affine driver `/tmp/aern2-audit/Affine.hs` passes 22,500 point
  checks each for correlated add/sub/mul/div, normalization, and precision
  lowering, plus 180 exact shared-subtraction cases. The 125 substitutions
  include a shared variable assigned consistently across operands; no hash
  collisions are forced or sought. A separate 25-point grid validates 3,825
  sqrt/exp/sin/cos enclosures against MPFR, with zero failures/indeterminates.
  Logs: `/tmp/aern2-affine-{numerical,elementary-oracle}.log`.
- Term-cap diagnostics confirm all four affine unary functions tested turn
  a maxTerms=2/two-term input into three terms. Numerical enclosure succeeds
  in these cases, but the cap is not a hard memory guarantee. Hyper already
  cancels identical opaque atoms in a bounded exact affine normal form
  (`Real::exact_rational_normal_form`); approximate affine maps would be a
  distinct, separately proved filter, not a replacement for that exact tier.
- Both independent drivers pass ordinary Memcheck with zero errors and no
  lost bytes. Scalar driver: 152,881 native allocations / 5,848,959 bytes,
  4,195,840 reachable. Affine driver: 1,345 / 4,275,713, 4,195,872 reachable.
  These are native allocator observations, not Haskell managed-heap object
  counts. Logs: `/tmp/aern2-{independent,affine}-memcheck.log`.
- Comparable scalar performance drivers validate all 582 input/operation/
  precision combinations in each library against MPFR. AERN2 uses two
  explicit Accuracy guard bits and verifies radius <= 2^-requested_bits;
  Hyper uses its computable integer approximation contract. Both pass all
  582. Cold cases rebuild from 97 cyclic rational inputs; warm cases reuse
  one expression. Haskell results are fully forced, not merely tagged or
  accuracy-inspected. Fifteen paired ABBA rounds, CPU 6, calibrated ~50-ms
  batches and 5,000 bootstrap samples are running; no timing claim yet.
- Scalar benchmark complete: `/tmp/aern2-hyper-scalar-bench.json` records
  all 12 cases. Hyper/AERN2 median paired cold ratios at 53 bits are 0.03434
  sqrt, 0.02465 exp, 0.01342 sin; at 200 bits 0.02595, 0.02857, 0.01352.
  Their bootstrap intervals are all wholly below 1. Reused-expression
  accuracy-query ratios are 0.00218/0.00216/0.00235 at 53 bits and
  0.00154/0.00150/0.00157 at 200. Warm comparisons include AERN2's repeated
  Accuracy search/measurement and Hyper's cached integer export; neither is
  described as fresh arithmetic. These are end-to-end scalar/API comparisons,
  not language-independent kernel speedups or downstream geometry results.
  CPU affinity/pairing mitigate but do not eliminate other workspace activity.
- Stripped complete scalar+validation executables: AERN2 11,374,392 bytes,
  Hyper 1,638,600 bytes. Haskell includes its runtime, while the Rust test
  binary also contains the rational export helper; no portable library-only
  binary-size claim follows. Exp/200, 2,000 measured + 100 warmup iterations:
  AERN2 RTS reports 1,500,093,312 managed heap bytes cold versus 102,141,744
  warm, max residency 51,616 versus 44,328 bytes, 6 MiB runtime memory.
  These managed-heap figures must not be equated to native malloc counts.
- Complete remaining ERC reads (11 files / 1,178 lines): .gitignore (3),
  Setup (2), Monad (74), Logic (67), Real (131), Integer (56), Statements
  (53), Variables (23), Pair (26), Array (219), Examples (524). Total 109
  files / 13,838 physical lines. ERC is a global-precision ST transformer
  with a validity bit and whole-program retries. Arithmetic/statement
  wrappers propagate invalidity; multivalued choose scans evaluated options.
  Parallel expression branches form a hull, distinct from current CReal's
  overlap-requiring union. Side-effectful branch/limit actions need scoped
  state rules from the semantics before any transfer to Hyper is justified.
- Static ERC concerns retained for qualification: inner limit compares an
  achieved accuracy to its negative exponent parameter rather than its
  positive magnitude; Array.checkA executes an error action to obtain its
  dummy before checking validity; getters can inspect a dummy index before
  the guard; several partial real APIs remain runtime errors. No crash-focused
  reproducer is introduced. Root package normally disables ERC; an external
  `/tmp/aern2-audit/erc-stack.yaml` enables just mp/ERC without editing the
  donor's tracked source or build configuration. Build qualification started
  after the scalar benchmarks completed.
- Hyper Exp/200 native-memory controls (2,000 measured + 100 warmup) pass
  Memcheck with zero errors/lost bytes: cold 159,620 allocations / 7,396,200
  bytes, warm 2,192 / 74,640; 544 reachable in each. These reinforce cache
  reuse benefits but are not directly comparable object counts to GHC RTS
  managed-heap accounting. Logs: `/tmp/hyper-aern-exp-{cold,warm}-memcheck.log`.
- ERC build hit sandbox DNS fetching STMonadTrans 0.4.8 and was retried
  through approval. No donor source changes or Hyper changes were made in
  this AERN2 checkpoint; both tracked worktrees remain clean.
- Untouched ERC build fails with five missing-lift import errors in
  ERC/Monad under the pinned current dependency snapshot. A compile-only
  overlay revealed two analogous missing zipWithM_ import errors in Array.
  `/tmp/aern2-erc-untouched-build.log` preserves the original failure.
  A separate overlay under `/tmp/aern2-audit/erc-overlay` adds only
  `Control.Monad.Trans.Class (lift)` and `Control.Monad (zipWithM_)`, verified
  by whole-file diffs; no numerical or state-control code is changed.
  The overlay builds and permits ordinary numerical qualification, which
  must not be described as an untouched upstream build/test pass.
- ERC import-only compatibility driver `/tmp/aern2-audit/ERCCheck.hs` passes
  28 linear and 28 logarithmic nearby-integer cases, 18 Heron square-root
  enclosures, six trisection square roots, 12 exact-rational Muller recurrence
  controls, and two nonsingular determinant controls. Thirty Heron/exp
  enclosures additionally pass directed MPFR with no indeterminate overlaps.
  This tests normal positive-domain/stateful calculations, not the static
  dummy-array failure path. Logs: `/tmp/aern2-erc-numerical.log` and
  `/tmp/aern2-erc-elementary-oracle.log`.
- ERC import-only compatibility driver also passes Memcheck: zero errors,
  zero lost bytes, 857 native allocations / 4,269,815 bytes, 4,195,808
  reachable in its runtime. Log: `/tmp/aern2-erc-memcheck.log`.
- Current AERN2 transfer disposition: no production change accepted.
  Replacing demand-sized/cached Hyper scalars with the measured lazy ball
  sequence is unsupported by timing, allocation, and footprint results.
  Hash-only affine identity and rough accuracy metadata are rejected as
  certified evidence. A separately certified bounded affine-error filter,
  pure retry scopes for higher-order limits, and Lipschitz-carrying function
  approximations remain cross-references for the unread net/function/formal
  layers, not silently dismissed or claimed implemented. ERC's nearby integer
  idea is already covered by Hyper's retained XRC-derived bounded query.
- Next source continuation: AERN2 net/linear/function packages and supporting
  reference documents; the ERC v10 paper is downloaded but not read beyond
  its abstract. The broader inventory and the cold-rational ownership
  experiment remain open. This is a checkpoint, not full-audit completion.
- Linear source/metadata continuation: completely read README (13), cabal
  (98), changelog (3), package.yaml (79), Vector (148), Matrix (337),
  AdaptedLinear (232 newline-count lines; final unterminated line read),
  TestApprox (53), matrix-benchmark (169), runBench.sh (115): ten files /
  1,247 lines. Historical result logs/generated JavaScript remain uncredited.
  Runtime dimension checks wrap boxed fixed-size vectors; LU uses no pivoting.
  The named inftyNorm computes max absolute entry, not induced row-sum norm.
  Memoized Laplace expansion uses signed column-subset keys and zero pruning.
  The benchmarks force a norm/accuracy but lack an independent full-result
  oracle and reuse old logs; those logs are not current qualification.
- Linear mulViaFP implements a directed midpoint/radius matrix product;
  solveBViaFP uses approximate inverse/solution proposals followed by residual
  inclusion iteration (15 attempts, epsilon inflation). Targeted primary-paper
  comparison read Rump 2010 printed pp. 43--45 and 57--59, including (9.14),
  Theorems 10.6/10.8 and Algorithm 10.7. Full paper is NOT read; some other
  extracted pages have broken font encoding and are not credited. Local
  `/tmp/aern2-rump2010.{pdf,txt}`; primary URL:
  https://www.tuhh.de/ti3/rump/intlab/ActaNumerica2010.pdf . The theorem requires
  strict interior inclusion to prove nonsingularity; the donor uses ordinary
  contains, a static proof gap not justified by passing regular cases.
  Matrix multiplication's published performance rationale relies on BLAS;
  the donor itself uses generic boxed-vector loops. Untouched linear build
  started via external config, then approved dependency downloads after DNS
  restriction; output `/tmp/aern2-linear-untouched-build.log`.
- Six QA files fully read, 887 lines: Protocol (277), Cached (23),
  CachedUnsafe (97), Parallel (227), Cached/Arrow (121), Cached/NetState (142).
  Cumulative source coverage 125 files / 15,972 lines. QA separates query,
  answer, cache and execution strategy; asynchronous requests return promises.
  Parallel execution atomically coalesces dominated active queries with STM
  and forks per fresh query, but no exception/cancellation cleanup is present.
  Do not introduce thread/race failure reproducers. Pure unsafe caching updates
  from an earlier cache snapshot, while stateful caches use existential ID
  maps and unchecked internal casts; those mechanisms are not transfer donors.
  Net logging appends singleton lists repeatedly and retains full query/answer
  strings. The useful semantic idea is query dominance and protocol-specific
  answer reduction; concrete accuracy/cache instances remain to be read.
- Additional 31 net files / 4,772 lines fully read, bringing coverage to
  156 files / 20,744 lines: AccuracySG (139), Sequence/Type (203), package.yaml
  (84), QA/NetLog (120), Utils/Arrows (58), Sequence/Helpers (245), Ring (289),
  Field (197), Elementary (307), Branching (161), Comparison (360), PreludeOps
  (95), Real/Type (132), Limit (156), Real/Arithmetic (306), aern2-real.cabal-
  (190), changelog (14), WithGlobalParam/Type (200), Helpers (85), Branching
  (73), Field (106), Elementary (158), Ring (203), Comparison (261),
  WithGlobalParam wrapper (40), MPBallWithGlobalPrec (73), Sequence wrapper
  (67), Real wrapper/examples (174), Real/Tests (255), test/RealSpec (20),
  test/Spec (1). All net src/test files now read; old/bench/tools/artifacts
  remain outside that claim.
- Net's single intersected ball cache pairs achieved accuracy with a
  componentwise strict/guide query. Default guide is strict+20; output may be
  reduced to the requested guide. Binary operations estimate sensitivity
  from norms, then increase both child requests one bit per unsuccessful
  iteration. Separate WithGlobalParam objects forward one immutable precision
  through the whole computation and reuse answers for dominated requests.
  Hyper already has magnitude-sensitive demand, output scaling and a single
  best synchronized cache; a 20-bit guide is not itself evidence of a better
  schedule. Net Eq/Ord use a fixed default query and raise on undecided cases;
  staged comparisons preserve partiality. Mixed Double arithmetic explicitly
  lowers to Double. Most inverse/hyperbolic functions remain unimplemented.
  Limit wrappers add explicit convergence radius; the Lipschitz version repairs
  a center evaluation using a bound over the full input ball, with direct
  interval evaluation as fallback. This is the iRRAM pattern, not a new scalar
  primitive. Net tests often accept a missing result as a vacuous accuracy pass.
- Untouched linear build fails at exactly two removed tuple-conversion
  instances. A source-compatible external overlay replaces both tuples by
  CentreRadius and imports that constructor. The published 0.2.9.0 conversion
  module was read in full separately to verify old tuple semantics:
  https://hackage.haskell.org/package/aern2-mp-0.2.9.0/src/src/AERN2/MP/Ball/Conversions.hs .
  Thus `(0.9,1.1)` really meant center/radius (about [-0.2,2]), NOT the paper's
  endpoint interval [0.9,1.1]. The overlay preserves this behavior; it is not
  an algorithm repair and is not described as an untouched upstream pass.
- `/tmp/aern2-audit/LinearCheck.hs` passes 60 cases each of exact Rational
  memoized Laplace, standard/adapted LU determinants, standard/adapted LU
  solves. At each precision 10/24/53/100 it passes 48 directed matrix products,
  96 ball product matrix checks and 864 wide-ball substituted-point checks.
  The nine substitutions include repeats; all actual rational points are
  independently checked, not claimed exhaustive interval validation. All 180
  regular diagonally dominant interval solves pass with no error status.
  Exact oracles are Prelude Rational products and independent Leibniz sums.
  Numerical log `/tmp/aern2-linear-numerical.log`; Memcheck log
  `/tmp/aern2-linear-memcheck.log`: zero errors/lost bytes, 344,367 native
  allocations / 7,517,293 bytes, 4,195,808 reachable runtime bytes.
- Hyper cross-reference: hyperlattice core 8894--8968 and 9261--9395 already
  uses division-free fixed 3x3/4x4 cofactors, reusable six-minor factors and
  delayed exact signed product sums. Generic subset-map allocation is not a
  clear improvement at these sizes. Hypersolve interval 624--810,
  915--1178 and 1847--1940 already certifies affine roots through exact
  pivot-checked solves and quadratic boxes through separate strict contraction
  checks. Correction to this early assessment: the later independent public
  oracle found the multivariate contraction formula dimensionally wrong;
  existence of a separate check did not establish its soundness. See the
  dimensionless-contraction correction checkpoint below. Approximate
  preconditioning for larger systems remains a distinct
  candidate; repeated exact per-column elimination in invert_exact_matrix
  is a potential factorization-reuse experiment, not yet changed or dismissed.
- Linear product benchmark uses fully forced entries of 13 rotating input
  matrix pairs, input construction excluded, ordinary versus mulViaFP kernels
  in the same executable. Exact Rational checks cover every benchmark pair.
  Fifteen paired ABBA rounds with CPU 6 affinity and bootstrap intervals are
  running at dimensions 2/4/8/16 and precisions 53/200. No Hyper patch accepted.
- Another 13 net files / 1,303 lines completely read: old/Effort (158),
  old/Tolerant (123), bench/benchOp (126), simpleOp (76), cdar-simpleOp (74),
  runBench.sh (173), tools/aern2-generate-netlog (37), LICENSE (12),
  visualise-netlog/Main.elm (438), QANetLog.elm (46), elm-package.json (21),
  index.html (18), .gitignore (1). Cumulative 169 files / 22,047 lines.
  Old Effort is a nested string-map precision budget experiment with dummy
  cache and stale QA APIs. Tolerant keeps approximate judgments explicitly
  distinct from certain booleans but ignores the requested tolerance when
  measuring the actual difference; not an exact decision substitute.
  Bench scripts are inherited historic scalar recipes with reused logs,
  reported accuracy rather than independent answer assertions, and rounded
  zero durations replaced by 0.01. Elm displays dependency edges and a
  step-through event history; pending queries are keyed only by provider,
  losing concurrent request multiplicity in visualization. Display/logging
  carries no numerical certificate. Generated JS/charts and historical
  log/data copies are inventoried but not credited as hand-written source.
- The preliminary linear benchmark is superseded: tiny batches at large
  dimensions used differing prefixes of 13 cyclic input pairs. Final runs
  now use complete 13-input cycles for both methods (also 13 warmups), so
  every batch has the same operand mixture. The independent oracle validates
  all 208 method/input/precision/dimension matrix results used in the benchmark.
  This correction is required before interpreting the timing differences.
- Fully read all eight tracked aern2-fun files / 1,044 lines: README (5),
  LICENSE (12), package.yaml (65), Interval (260), PQueue (63),
  RealFun/Operations (204), SineCosine (331), Tests (104). Cumulative AERN2
  coverage 177 files / 23,091 lines. This package supplies function/domain
  interfaces rather than a complete concrete function representation.
  sampledRange is explicitly a heuristic, not a range enclosure. Generic
  sine/cosine composition reduces by pi/2, chooses degree from a Lagrange
  remainder, reuses parity-specific powers, propagates power accuracy demands
  backward, and restores input uncertainty through the global Lipschitz bound.
  Its maps of factorials/powers are function-polynomial schedules, not a
  demonstrated improvement to Hyper's scalar reduced-argument recurrence.
  Concrete polynomial/ball-function instances and their tests remain unread,
  so composition/domain conclusions are provisional. No new Hyper code.
- Final authoritative linear timing run: `/tmp/aern2-linear-bench-final.json`
  and `.log`, 21 paired ABBA rounds, 200 ms target batches, CPU 6 affinity,
  identical whole 13-input cycles for both methods, 5,000 paired bootstrap
  samples. Earlier 15-round/short-batch results are superseded. Median
  mulViaFP/ordinary ratios (95% intervals), dimensions 2/4/8/16:
  at p=53: 1.1612 [1.1550,1.1682], 0.9252 [0.9140,0.9297],
  0.8096 [0.8022,0.8166], 0.8073 [0.8009,0.8123];
  at p=200: 1.1950 [1.1833,1.2048], 1.0163 [1.0064,1.0288],
  0.9329 [0.9256,0.9443], 0.9230 [0.9167,0.9347]. All 208 benchmark
  method/input/precision/dimension results contain the independent exact
  Rational products. This compares donor kernels, not donor versus Hyper.
- Radius quality on every benchmark entry: median via/ordinary ratios at
  dimensions 2/4/8/16 are 1.2055/1.2124/1.2140/1.2143 (p=53) and
  1.3288/1.4211/1.5524/1.6162 (p=200). Worst ratios across these sets are
  1.5574 and 4.8721 respectively; no ordinary zero-radius entry became
  nonzero. Thus a faster fixed-precision product is not equal-cost accuracy.
  RTS p=200,n=16,count=26 (plus 13 warmups and input construction): ordinary
  1,386,878,480 managed allocated bytes / 1,272,712 maximum resident bytes;
  via 1,145,575,520 / 1,364,272. Logs `/tmp/aern2-linear-memory-{plain,via}.log`.
  Same executable contains both paths, so no incremental binary-size claim.
  No Hyper port justified yet; larger-system certification/factorization reuse
  remains separately pending.
- All 15 tracked aern2-mfun files / 2,114 physical newline lines fully read:
  README (5), package.yaml (73), Linear/Matrix/Type (132), Matrix/Inverse
  (125), Linear/Vector/Type (175), Util/Util (13), AD/Differential (10),
  AD/GenericOperations (220), AD/MPBallOperations (223), AD/Type (46),
  BoxFun/Box (149), Optimisation (319), TestFunctions (424), Type (170),
  bench/OptimisationBenchmark (30). Several final lines lack a newline;
  complete final lines were read, counts consistently use wc -l. Cumulative
  AERN2 192 files / 25,205 lines. No mfun tests are enabled by its manifest.
- Mfun second directional jets propagate order 0/1/2 and share triangular
  Hessian evaluations. Min/max intentionally drop derivatives; general power
  drops second to first order (explicit FIXME), while abs order two is absent.
  Integer-power differentiation uses negative intermediate powers for n=0/1,
  potentially adding avoidable domain uncertainty. No approximation is a proof
  of differentiability at abs/min/max singularities. Hessian/gradient range
  refinement uses derivative bounds and center-plus-radius enclosures.
- Mfun inverse is a separate implementation from the qualified linear package.
  It proposes a rounded Gauss inverse, checks ||I-YA||<1, then intersects a
  Neumann-style initial bound with Y+(I-YA)X. Static problems to qualify:
  Vector.inftyNorm omits absolute values; Matrix/Vector accuracy takes the
  maximum entry accuracy, not a whole-object minimum; pivot search rereads the
  current winner rather than the candidate row; elimination on the inverse
  uses coefficients from the inverse itself rather than the left matrix.
  The residual contraction check can reject bad proposals, but cannot justify
  an incorrectly bounded starting enclosure. Numerical checks pending.
- Box optimization uses lower-bound priority search, sampled center upper
  cutoffs, derivative-weighted splitting, Lipschitz range intersections,
  interval Newton and recursive boundary restrictions. Its local-minimum
  search discards monotone boxes (global boundaries are handled separately).
  SearchBox Eq always False and list-appending traces are not transfer patterns;
  local-search empty-queue fallback returns a previous range, not an explicit
  no-stationary-point result. The benchmark only prints chosen examples and
  does not independently verify minima. No broad benchmark launched before
  qualifying the basic numerical certificates.
- Untouched net build stops during planning: missing README warning and
  obsolete executable dependency `cdar` absent from the active build plan.
  Log `/tmp/aern2-net-untouched-build.log`. This is not a passing package build;
  a separately scoped QA-protocol runtime check remains pending.
- Untouched fun/mfun libraries build with the active mp/real packages and
  psqueues-0.2.8.1; log `/tmp/aern2-mfun-untouched-build.log`. No upstream
  source overlay was needed. Independent `/tmp/aern2-audit/MfunCheck.hs`
  validates jets against exact truncated bivariate Rational convolution and
  coefficient-by-coefficient division. At each p=24/53/100: 100 ring/division
  jets, 300 regular nonzero-base integer-power jets and 48 elementary
  special-value jets all pass. Rosenbrock value/gradient/Hessian passes 24/25
  at each precision. Powers 0/1/2 at zero fail to supply a valid full jet,
  while powers 3/4/5 do; the square's internal zero-to-zero power introduces
  an avoidable error into an otherwise smooth polynomial derivative.
  Logs `/tmp/aern2-mfun-{numerical,ad-numerical}.log`. These are qualification
  counts, not an assertion that the whole package passes.
- Mfun inverse qualification uses 24 signed nonsingular diagonal and nine
  mildly coupled exact Rational 2x2 matrices. At EACH p=24/53/100, 13/33
  yield error-free enclosures of the independent exact inverse, 18/33 return
  an error-bearing matrix under Just, and two exhaust a one-second per-case
  qualification budget. A preceding unbounded whole-driver run was interrupted;
  a 45-second whole-driver run also failed to finish. Timeouts are unresolved,
  not false certificates or performance measurements. No error-free incorrect
  enclosure observed. The negative-vector norm fails its exact max-absolute
  oracle; a vector with one exact and one radius-1/4 entry reports Exact.
  Do not transplant these inverse/norm/accuracy implementations.
- Net QA infrastructure compiles directly from unchanged source independently
  of obsolete net numeric layers. `/tmp/aern2-audit/NetCheck.hs` provides a
  dyadic approximation protocol for exact 22/7, explicitly rescales cached
  answers and checks each answer against an independent Rational floor bound.
  Ten queries at scales 0--200 pass for each of eight paths: pure cached,
  state cached/uncached sequential, state batch, STM serial/parallel sequential,
  STM parallel batch, STM logged sequential (80 answers). Cached state and
  logged STM sequential each show five dominated-query hits; uncached shows
  zero; each log has one creation/ten queries/ten answers. This does NOT
  qualify the old Sequence/Real arithmetic. Log `/tmp/aern2-net-qa-numerical.log`.
  Newly generated untracked net cabal manifest removed after identifying it
  as this audit's Hpack output; original tracked files remain unchanged.
- Six more univariate files / 1,353 lines fully read: UnaryBallFun wrapper
  (26), Type (196), Evaluation (321), Integration (134), UnaryBallDFun (244),
  UnaryModFun (432). Cumulative 198 files / 26,558 lines. Ball evaluators
  represent convergence on open balls; modulus functions instead combine
  dyadic point evaluation with a local monotone continuity modulus and invert
  that modulus by exponential bracketing plus binary search. This is a useful
  semantic distinction, not evidence that either implementation is production
  ready. Their APIs still reference the old net CauchyReal/AccuracySG layers.
- Univariate range search uses upper-bound priority subdivision with retained
  lower bounds; interval integration shares absolute error budgets by giving
  each half one extra bit. Derivative evaluators can tighten via monotonicity
  or center-plus-derivative enclosures. Static caveats for later qualification:
  ball-function domain check repeats the left endpoint in its certain-error
  guard; binary lifts ignore the second domain; fixed-ball constants claim
  Exact at the function wrapper level; scalar-minus-derivative-function leaves
  derivative signs unchanged. The derivative-based midpoint remainder appears
  to be half the generic derivative-range bound and needs an independent
  smooth-function integral check before any reuse. The reciprocal-modulus
  heuristic uses an enclosure norm rather than a proved uniform separation
  from zero. These are static findings, not passing runtime assertions.
- Net QA and mfun AD sequential Memcheck runs finish with zero errors and
  zero definitely/indirectly/possibly lost bytes. Net: 119 native allocations /
  4,690,930 bytes; mfun AD: 15,514 / 4,851,488. Both retain 4,555,036 runtime
  bytes. Logs `/tmp/aern2-net-qa-memcheck.log` and
  `/tmp/aern2-mfun-ad-memcheck.log`. Memory cleanliness does not repair the
  numerical qualification failures above; inverse timeouts were not included
  in the mfun AD memory run.
- Univariate cabal (236) and LICENSE (12) fully read. Its 0.1.0.0 manifest
  requires mp/real/fun 0.1.*, incompatible with the repository's active 0.2.*
  packages, plus an unsupplied memoize dependency. Untouched build planning
  confirms these conflicts in `/tmp/aern2-univariate-untouched-build.log`.
  No blanket --allow-newer or algorithm repair applied to claim a build pass.
- Poly/Basics (325), Power/Type (176), Power/Eval (48) and Power/SizeReduction
  (22) completely read. Total now 204 files / 27,377 lines. Sparse coefficient
  maps use unbounded Integer degrees; direct evaluation nevertheless visits
  every degree down to zero (dense Horner with map lookups). Multiplication
  repeatedly shifts/maps/merges whole operand maps. Coefficient accuracy takes
  a minimum correctly, unlike mfun; polynomial uniform error is sum of radii
  only under the explicitly stated [-1,1] domain. Truncation bounds discarded
  terms by sum |c_k| max(|l|,|r|)^k and adds the bound to the minimum-key entry;
  it relies on preserving a constant term. Derivatives can remove that term.
  No sparse-map machinery shown worthwhile against Hyper's current kernels.
- Historical compatibility evidence: separately read the complete published
  aern2-mp-0.1.4 Ball/Type and Ball/Conversions modules:
  https://hackage.haskell.org/package/aern2-mp-0.1.4/src/src/AERN2/MP/Ball/Type.hs
  and https://hackage.haskell.org/package/aern2-mp-0.1.4/src/src/AERN2/MP/Ball/Conversions.hs .
  These confirm ball_error is the radius, endpoints are center +/- radius,
  tuples are center/radius, and ball norm uses an absolute upper endpoint.
  They are supplemental version checks, not extra parent-tree line credits.
  Thus Power/Eval's Lipschitz error multiplied by 0.5 is not explained by a
  historical full-width representation. Isolated expression qualification is
  prepared separately from the unbuildable complete package.
- Isolated expression results, `/tmp/aern2-audit/UnivariateRemainderCheck.hs`
  and `/tmp/aern2-univariate-remainder-formulas.log`: Power.evalLip's formula
  applied to the identity polynomial with exact Lipschitz bound 1 on [-1,1]
  returns [-1/2,1/2], excluding the exact endpoint values. The derivative
  midpoint formula with the smooth function sqrt(x^2+(1/16)^2), center value
  1/16 and valid derivative range [-1,1], returns [-3/8,5/8] over [-1,1].
  Its true integral is at least integral |x| = 1, so the asserted upper bound
  is too small. These are independently justified mathematical counterexamples
  to the extracted arithmetic expressions, run using current MPBall with the
  verified same radius convention; NOT claims to have executed the complete
  historical univariate package. Neither formula is a valid transfer donor.
- Checkpoint: no new Hyper production patch accepted in the AERN2 work so far;
  Hyperreal and reference tracked worktrees clean. No benchmarks or checks left
  running from this checkpoint. Next continue the remaining univariate
  polynomial/root-isolation/range modules, then fnreps and supporting artifacts.
  Preserve pending larger-system factorization/preconditioning experiments and
  the prior cold-rational ownership candidate; they have not been silently
  rejected. Overall reference inventory remains open, not complete.

### AERN2 integer polynomial continuation

- Previous turn classified as progress: additional line coverage, completed
  numerical/memory qualification and authoritative balanced matrix benchmarks.
  Rechecked source HEAD d1ac3664bfb5c7f70fcf68f7fb412d288def65cb; AERN2 and
  Hyperreal tracked worktrees clean at continuation start. No subagents used.
- Seven further files / 1,396 lines fully read: Power/Roots (237), RootsInt
  wrapper (16), RootsIntMap (251), RootsIntVector (403), SignedSubresultant
  wrapper (6), SignedSubresultantMap (242), SignedSubresultantVector (241).
  Coverage now 211 files / 28,773 lines. Commented-out former algorithms were
  also read; exported wrappers select the vector variants. Despite its name,
  SignedSubresultantVector still uses polynomial/maps throughout and is almost
  identical to the Map module (different constant-polynomial guard).
- Integer Bernstein arithmetic clears binomial denominators into one scale,
  then subdivides with integer affine weights and compensating powers of the
  endpoint difference. Map variant stores the full triangular tableau keyed by
  coordinate pairs; vector variant still retains every triangle row in a map.
  Both therefore retain quadratic intermediate storage. Sign variation treats
  the shared error as a coefficient radius scaled by the common multiplier.
  Initial conversion extrapolates twice from [-1,1] (or [-1,2] when l=1), so
  its stated positive-scale guarantee and nonzero-error handling need checks
  when the requested interval lies beyond that base interval.
- Integer findRoots explicitly promises only an outer cover of roots in an
  OPEN interval, not a unique root in every returned box. Acceptance predicates
  are checked before sign exclusion; midpoint roots are emitted before the
  recursive left cover despite the documented ordered-list claim. Once one
  variation is established, the algorithm switches to endpoint-sign bisection
  without reusing coefficients. It does not separately guard a zero endpoint.
  Numerical completeness/order qualification is pending. Ball-coefficient
  Roots uses uncertain signs and may return a coarse box or raise; it is not
  equivalent to the exact Integer variant.
- Signed subresultants use exact-division assumptions without remainder checks,
  keep extended P/U/V maps, and have a defective-degree branch whose descending
  intended zero-fill range is written as an ascending list. We will compare
  square-free/gcd-free outputs up to a nonzero scalar with independent rational
  polynomial division; no presumed Bezout normalization is asserted without
  checking its convention. No malformed-storage or memory-failure probes.
- Hyper cross-reference: hypercurve/bezier_parameter 3400--3660 and 4256--4284,
  rational_bezier 1788--1858. Hyper already uses exact Bernstein sign variation,
  explicitly requires nonzero endpoints in its square-free fast path, and
  retains Sturm fallback on undecided/zero-midpoint cases. Scalar de Casteljau
  subdivision uses one mutable work vector plus two outputs (linear live
  storage), better than either donor tableau. Shared integer scaling remains
  a distinct performance idea, not yet measured against Hyper.
- Direct compilation of the untouched root dependency slice confirms removed
  generic error/type APIs in Poly/Basics, separately from the earlier package
  version-plan failure. Log `/tmp/aern2-root-untouched-compile.log`. An external
  `/tmp/aern2-audit/root-overlay` qualifies ONLY exact Integer polynomial kernels:
  minimal Map-based Poly/PowPoly compatibility preserves donor arithmetic and
  Horner recurrence; root/subresultant copies translate obsolete unchecked
  operator spellings and omit obsolete error wrappers/duplicate division API.
  It is NOT an untouched full-package build or generic interval qualification.
- Read Power/Maximum (332), MaximumInt (328), MaximumIntAlt (301) completely,
  adding 961 lines: cumulative 214 files / 29,734 lines. Every source file
  inside univariate Poly/Power/ is now read. Maxima use derivative-root covers
  plus endpoint candidates and upper-bound priority queues. Truncated derivative
  ladders increase degree when sign evidence is insufficient, otherwise bisect
  a proved single critical interval. The Alt version carries a global candidate
  lower bound and clamps future evaluations to known range bounds. All variants
  may stop on lack of accuracy improvement, not just the caller's target;
  integer variants can give up on unresolved signs, and exact zero-sign searches
  still need explicit progress/endpoint handling. They depend on already-unsound
  interval evaluation formulas, so no whole-optimizer validity assumed.
- Exact Integer compatibility qualification `/tmp/aern2-audit/RootCheck.hs`,
  log `/tmp/aern2-root-numerical.log`: each map/vector variant passes 924 initial
  Bernstein conversions, 924 exact sign-variation checks, and 1,848 subdivision
  and extrapolation checks against a closed-form independent Rational oracle.
  Degrees 0--10, twelve coefficient seeds, seven intervals including ones
  outside [-1,1]. Only 804/924 multipliers are positive, contradicting that
  advertised intermediate invariant; zero-error variation remains invariant
  under global negative scaling. With radius 1/2 and included constant shifts,
  each variant passes 2,767/2,772 claimed certain-variation checks. Five fail;
  the error-sign logic must not be transplanted without positive-scale proof.
- Integer root cover qualification: five interior-root fixtures (including
  repeated roots) contain every expected root, remain inside the requested
  domain and satisfy width <=1/64; a midpoint-first fixture is not ordered.
  Of six endpoint-root fixtures, three lose an interior root in both findRoots
  and findRootsWithEvaluation. All outputs are otherwise in-domain and narrow;
  narrowness alone is therefore an inadequate completeness oracle. A polynomial
  with no real roots correctly returns an empty cover. This uses exact products
  of known rational linear factors, not the donor's root count as its oracle.
  Hyper's guarded nonzero-endpoint fast path already avoids this donor branch
  assumption. No Hyper patch accepted from these findings.
- Root compatibility-slice Memcheck completes with zero errors and zero
  definitely/indirectly/possibly lost bytes; 19,203 native allocations /
  4,445,297 bytes, 4,195,808 runtime bytes remain reachable. Log
  `/tmp/aern2-root-memcheck.log`. The numerical failures above remain failures.
- Independent Rational long-division/monic-gcd oracle qualifies signed
  subresultant outputs up to a nonzero scalar: fourteen regular fixtures each
  pass gcd, gcd-free division and separable-part comparisons in both Map and
  Vector modules. A fifteenth, ordinary high-degree-gap fixture reaches the
  defective-degree branch and terminates with division by zero in Vector;
  equivalent Map body not rerun on that known failure. Logs
  `/tmp/aern2-subresultant-vector-numerical.log` and
  `/tmp/aern2-subresultant-map-numerical.log`; driver
  `/tmp/aern2-audit/SubresultantCheck.hs`. No security/malformed-input probes.
  Separate fourteen-fixture Vector Memcheck passes with zero errors/lost bytes,
  93 native allocations / 4,263,617 bytes and 4,195,808 runtime reachable bytes
  (`/tmp/aern2-subresultant-regular-memcheck.log`). This does NOT qualify the
  defective-degree branch. No subresultant implementation transfer accepted.
- Finished `/tmp/aern2-audit/RootPerformance.hs` and `root-bench.py` exact
  integer-kernel comparison: degrees 8/16/32/64, thirteen independent operands
  rotated through an IORef, inputs forced before timing and every output
  coefficient/scale forced per call. Closed-form Rational oracle passes 26
  method/input cases per degree, each covering initial conversion and both
  half-subdivision outputs (104 cases / 312 coefficient vectors). CPU 6,
  21 paired ABBA rounds, five-call calibration, complete thirteen-input cycles,
  thirteen warmups, approximately 200 ms for the slower kernel per sample;
  5,000-bootstrap intervals on the median paired Vector/Map ratio. Results in
  `/tmp/aern2-root-bench.json` (ratios are NOT ratios of independent medians):

  | Degree | Initial conversion Vector/Map (95% CI) | Half split Vector/Map (95% CI) |
  | --- | --- | --- |
  | 8 | 0.5836 [0.5773, 0.5950] | 0.4289 [0.4265, 0.4352] |
  | 16 | 0.4373 [0.4311, 0.4425] | 0.2957 [0.2889, 0.3042] |
  | 32 | 0.3390 [0.3280, 0.3458] | 0.2466 [0.2403, 0.2544] |
  | 64 | 0.2466 [0.2409, 0.2477] | 0.1739 [0.1728, 0.1761] |

  This demonstrates indexed storage versus per-coefficient Map overhead in
  donor kernels, NOT an end-to-end root solver or Hyper speedup. Both donor
  variants still retain a quadratic tableau; Hyper scalar subdivision already
  uses linear storage. Shared integer scaling remains a separate open candidate.
- Eighteen further files / 2,619 lines fully read: Poly wrapper (37), Power
  wrapper (38), Cheb wrapper (102); Cheb/Type (390), Eval (215), Ring (188),
  DCT (355), Derivative (140), Integration (90), Field (216), ShiftScale (140),
  Maximum (75), MaximumInt (152), MaximumMP (122), MaxNaive (51), MinMax (176),
  Elementary (48), Analytic (84). Coverage 232 files / 32,353 lines.
- Chebyshev polynomials use a mapped domain, sparse coefficient map, accuracy
  guide and lazy cached min/max. Normalization collects coefficient radii into
  a constant uniform error, sound on [-1,1]; accuracy/radius operations assume
  this invariant. Raw constructors can bypass normalization, but no malformed
  serialization probes are made. Size reduction uses a uniform remainder.
  Clenshaw evaluation retains a list of recurrence levels rather than two
  accumulators; Cheb's Lipschitz evaluator uses the full input radius, unlike
  the already-failed Power formula. Exact derivative is a unit-coordinate
  derivative; ordinary derivative applies the physical-domain scale. Domain
  checks and derivative-of-uniform-error assumptions require caller discipline.
- Ring multiplication uses the exact two-term Chebyshev product identity;
  the DCT route is selected for exact large operands. The DCT grid is a strict
  next power of two and precision includes a linear grid-size allowance.
  Recursive DCT-I/III uses repeated list indexing, so its operation-count
  complexity alone is not a runtime complexity proof. DCT quotient construction
  offers a useful proposal/certificate split: bound the residual p-r*q and
  input errors, then divide by a strictly positive lower bound for |q|.
  Existing range/error-contract failures prevent treating the complete donor
  quotient as qualified. No interpolation transform benchmark yet credited.
- Static exactness finding: ShiftScale negation swaps cached min/max, but
  multiplication/division by a negative scalar scales each cached bound without
  swapping it. Maximum's public full-domain operations return those caches
  directly. This breaks the cache invariant even with otherwise valid inputs;
  complete obsolete-package execution has NOT been claimed. Cheb MinMax also
  relies on ordered root covers (already disproved) and appears to mix unit
  root coordinates with physical-domain endpoints. MaxNaive hardcodes +/-1
  endpoint evaluations despite accepting subinterval bounds. Domain-dependent
  derivative/analytic error scalings remain explicit qualification candidates,
  not unverified assertions of observed runtime failures.
- Read the remaining Cheb/MaximumPrime (411), Cheb/Tests (341), and
  Poly/Conversion (90) in full: cumulative 235 files / 33,195 lines; every
  Cheb source file now read. MaximumPrime repeats derivative-ladder queues,
  integer root variation and critical bisection. It can carry coefficients
  from a previous ladder degree while advancing the function index, and a
  child left-sign lookup uses the original global left endpoint. No proof
  justifies those invariant changes. Its point interval may stop without the
  requested accuracy. Tests mostly generate centered arithmetic expressions
  and compare pointwise consistency; this does not independently establish
  enclosure, root completeness or negative-scalar cache correctness.
- Cheb-to-power conversion builds powers of the 2x2 Chebyshev recurrence
  matrix and combines coefficient pairs in a binary tree. This is an algebraic
  divide-and-conquer idea, but current polynomial multiplication is map-based
  and naive; no asymptotic speedup is assumed. The MPBall variant centers
  coefficients, converts exactly, then restores their summed radii as one
  uniform error, valid on the specified unit domain. Full numeric qualification
  of this transform and the DCT remains pending.
- Serial RTS memory measurements for the root coefficient benchmark, degree
  64, 260 calls plus thirteen warmups and shared setup: initial Map/Vector
  allocate 2,101,834,768 / 1,167,274,696 bytes, maximum sampled residency
  3,259,944 / 410,440 bytes; split Map/Vector allocate 802,960,504 /
  405,861,280 bytes, residency 410,320 / 410,392 bytes. Logs
  `/tmp/aern2-root-{initial,split}-{map,vector}-rts.log`. Whole-process numbers
  include constructing/forcing BOTH input representations, so split residency
  is dominated by common setup and cannot establish incremental kernel memory.
  No binary-size claim: both implementations are in one qualification binary.
- DCT transform compatibility slice `/tmp/aern2-audit/dct-overlay/DonorDCT.hs`
  retains all six reference/recursive transform bodies from lines 182--355,
  changing only obsolete `/!` spelling to current raw `/`; it compiles with
  active mp/real. It excludes the unbuildable ChPoly lifting, normalization,
  range caches and precision scheduler. Driver `DCTCheck.hs` passes 126 exact
  polynomial roundtrips and 126 product coefficient-vector checks against the
  independent Rational Chebyshev product identity: precisions 24/53/100,
  grids 2/4/8/16/32/64, seven coefficient seeds. Full input/output lists forced.
  Log `/tmp/aern2-dct-numerical.log`.
- Independent 2,048-bit directed MPFR cosine-matrix oracle then qualifies
  5,730 individual transform outputs: DCT-I, DCT-III and simplified DCT-III,
  both direct and recursive algorithms, grids 2/4/8/16/32, five seeds, three
  precisions. Zero failures or indeterminate overlaps; every donor interval
  contains the entire independent enclosure. Oracle uses exact cosine
  symmetries before summing, exact rational coefficients, directed pi and
  cosine rounding, plus the global cosine Lipschitz bound for argument error.
  It does not reuse the donor's transform or trigonometry. Files
  `/tmp/aern2-audit/dct-oracle.rs`, `/tmp/aern2-dct-transform-values.txt`,
  `/tmp/aern2-dct-oracle.log`. This is positive kernel evidence, not validation
  of the surrounding univariate function package.
- Five files / 945 lines fully read: Poly/Ball (341), Analytic wrapper (34),
  Analytic/Type (76), Analytic/Field (24), AnalyticMV/Type (470). Total
  240 files / 34,140 lines. PolyBall separates center-function arithmetic from
  a uniform radius; multiplication has the standard three-term error bound
  and sine/cosine preserve outer radius by their global 1-Lipschitz property.
  Scalar Analytic carries geometric coefficient decay, a clamped unit-domain
  wrapper, Horner partial sums and an explicit tail modulus. The old realLim
  uses removed net request APIs. MV series memoize coefficients and derivative
  recurrences and include a polynomial ODE prototype, but coefficient-growth
  metadata is not closed under its multiplication or partial substitution:
  multiplying two unary series with all coefficients 1 and A=k=1 yields
  coefficient n+1 while retaining A=k=1; substituting x=1/2 into the valid
  bivariate all-one series doubles remaining coefficients but retains A=1.
  These are algebraic invariant counterexamples, not a compiled ODE verdict.
  No verified ODE machinery is inferred from the prototype's examples.
- Eight Frac files / 534 lines fully read: wrapper (15), Type (139), Ring
  (90), Field (84), Eval (61), Maximum (87), Conversion (39), Integration (19).
  Total 248 files / 34,674 lines. Fractions pair Chebyshev numerator/denominator
  with a reciprocal-denominator bound. Its Lipschitz identity is useful when
  that bound is valid and positive. Scalar division changes the denominator
  without updating the bound, and may make it negative; pointwise min/max
  assumes positive denominator products. Division of two fractions rejects
  separated negative denominators instead of sign-normalizing them. The
  radius-update implementation sets numerator radius to qmax*(new-old), so
  even identity update removes its old uncertainty. Range search supplies
  p and q where evalDf expects their derivatives and mixes unit-domain search
  coordinates with evaluation of the original physical-domain fraction.
  These are static contracts to reject, not claimed full-package executions.
- Eight PPoly files / 1,949 lines completely read: wrapper (50), Type (376),
  Eval (86), Integration (44), MinMax (108), Division (393), Maximum (510),
  Tests (382). Total 256 files / 36,623 lines. Piece partitions use global
  unit coordinates with a separate physical domain; ordered partition merge
  is linear and independent of polynomial coefficients. Most arithmetic does
  not check matching physical domains, although pointwise max does. Piece
  evaluation scans all pieces, hulls intersecting results, and does not clip
  the input ball to each piece. evalLDf maps to unit coordinates twice for
  piece selection; evalDI applies physical derivative scaling but then uses
  unit input radius. Subdomain/global-coordinate invariants need proof before
  reuse. Integral decomposes into centered primitive differences plus radius
  times segment width and applies the physical-domain Jacobian.
- PPoly reciprocal constructs a piecewise linear proposal, certifies it by
  residual/minimum, then performs centered Newton refinement with squared
  error propagation. This is a sound high-level pattern when the lower bound
  and initial tube are valid. However, the final uncertain-input correction
  simply adds radius(f) to inverse(centre f), omitting reciprocal sensitivity;
  checked division wraps the unchecked path with an explicit TODO for zero.
  Negative denominators are explicitly unsupported. Pointwise max depends on
  already-failed root ordering/completeness and chooses a branch from uncertain
  midpoint values. The default optimized maximum delegates each segment to
  Cheb; alternate global queues have further segment reindexing assumptions.
  Bundled tests center generated inputs and disable direct evaluation, range,
  size-reduction and trigonometric test groups. No complete PPoly numeric pass
  or Hyper production transfer is claimed.
- Twelve Local files / 833 lines fully read: wrapper (18), Basics (29), Poly
  (88), DPoly (80), Frac (79), PPoly (37), Ring (60), Field (33), MinMax (9),
  SineCosine (25), Integration (37), Maximum (338). Total 268 files / 37,456
  lines; every univariate src file now read. Local values take an interval and
  accuracy and return a local approximation. Generic arithmetic passes the
  same requested accuracy to operands without a compositional modulus; Exact
  is claimed at wrapper level even for fixed approximations. Maxima combine
  priority subdivision with rebuilding local polynomial certificates at higher
  accuracy; derivative sign and enclosure both come from those certificates.
  Polynomial donors automatically invoke the already-limited separable-part
  kernel, while DPoly's optional local Lipschitz evaluator is disabled.
  Integration chooses a uniform partition from requested bit count and shares
  an error budget. Both partition constructors center rounded endpoints with
  no endpoint-coverage correction; no complete local-function convergence or
  optimality proof follows from these scheduling heuristics.
- Six more univariate support files / 393 lines fully read: bench operation
  driver (180), runBench.sh (83), bench/.gitignore (2), data serializer (107),
  test/AERN2/PolySpec (20), test/Spec (1). Total 274 files / 37,849 lines.
  The runner supplies a degree where the executable expects a serialized
  filename. Per-result timing forces reported accuracy, not necessarily all
  coefficients or cached extrema; the summary also includes between-result
  bookkeeping. Inputs are centered before timing and the generator makes
  denominators positive. Therefore these historical benchmarks do not exercise
  uncertain-input propagation and are not authoritative full-result kernel
  timings. Only Cheb's test spec is wired in; the PPoly spec is not selected.
  Nineteen compressed serialized sample files remain unread, explicitly not
  counted as audited merely because their generators have been inspected.
- `/tmp/aern2-audit/FunctionContractFormulas.hs`, log
  `/tmp/aern2-function-contract-formulas.log`, independently executes the
  extracted Rational error/cache expressions, NOT the complete obsolete
  packages. Negative scaling sends cached (min,max)=(0,4) to (0,-8), although
  the correct pair is (-8,0). Frac scalar division's stale reciprocal bound
  gives [-1/2,1/2] for 2*x on x in [-1/2,1/2], excluding +/-1; identity radius
  update erases a numerator tube +/-1/8. PPoly's doubly scaled derivative
  gives [7/4,9/4] for identity on [3/2,5/2] within physical domain [0,4]. Its
  final reciprocal input correction gives [15/8,17/8] for 1/(1/2 +/-1/8),
  excluding both exact reciprocal endpoints 8/5 and 8/3, even granting an
  exact centered inverse. PowS product and substitution metadata failures are
  also checked algebraically. These reject the arithmetic transfer formulas
  without falsely claiming a full-package runtime result.
- Ten fnreps files / 1,568 lines completely read: README.md (245), README.rst
  (150), cabal (124), LICENSE (30); main/Demo (64), fnreps-ops (489),
  fnreps-bounds (349), waac-benchmarks (107), range-bench (5),
  rootIsolation-bench (5). Total 284 files / 39,417 lines. All fnreps main code
  read. Manifest repeats the obsolete 0.1 dependency requirements; two tiny
  dormant benchmark entrypoints refer to absent predecessor namespaces.
  READMEs describe historical MPFR/radius and hardware configurations, NOT
  this audit's active CDAR backend or host. The Frac documentation requires
  denominator >=1, stronger than mere positivity: its public scalar-division
  operation can itself violate that invariant. A generic benchmark expression
  does not establish equal accuracy: global polynomial/rational paths often
  ignore the requested output accuracy, local paths interpret guide updates
  differently, and expression-specific inflation ranges up to 10*ac+10.
  WAAC compares 70-bit global tasks to 53-bit local tasks. Printing a result
  and its self-reported accuracy is not an independent enclosure check.
- fnreps-bounds models floating arithmetic with per-operation relative-error
  balls and additive underflow allowance, splitting approximation/model error
  from rounding error. That is a useful application pattern, not a new scalar
  backend; it inherits unqualified polynomial-range contracts. No historical
  chart timing or plotted enclosure has been credited as reproduced evidence.
- DCT numerical compatibility slice also completes all 252 roundtrip/product
  vector cases under serial Memcheck: zero errors and zero definitely,
  indirectly or possibly lost bytes; 1,012,650 native allocations / 31,364,447
  bytes and 4,195,840 Haskell-runtime bytes reachable at exit. Log
  `/tmp/aern2-dct-memcheck.log`. Its instrumented run was kept separate from
  kernel timing. No allocator-probe or malformed-input testing was needed.
- Three fnreps support files / 466 lines fully read: benchresults/README-charts
  (22), runBench.sh (424), plots/makect2.sh (20). Total 287 files / 39,883 lines.
  Runner reuses logs by default without pinning the binary/commit, overwrites
  timed-out logs with a short marker, then can reuse that marker as an empty
  CSV row. It changes printed 0.00 CPU times to 0.01 and embeds current import
  time rather than original execution time. Several difficult cases are
  disabled or change requested precision to evade known nonconvergence.
  Plot generator's nested sine+cos formula uses cos(10*x), while current
  fnreps-ops uses cos(3*pi*x). Historical plots and timings need per-artifact
  provenance; these are not trustworthy regression baselines without it.
- Read all twelve fun-plot tracked files / 1,577 lines: cabal (117), LICENSE
  (12), server PlotService wrapper (26), API (225), App (227), client Api.elm
  (201), DInterval.elm (125), Main.elm (380), GenerateElm.hs (17), Makefile
  (11), elm-package.json (24), example/Main.hs (212). Total 299 files /
  41,460 lines. The numerical serialization reduces dyadic numerators to
  machine Int, then the client compares/displays via Float; it is not an
  arbitrary-precision exact scalar protocol. Server sampling checks endpoint
  differences and segment width, not an interior interpolation-error bound;
  the drawn parallelograms therefore are not certified continuous-curve
  enclosures. UI coalesces resize work and renders retained samples, offering
  no new scalar algorithm. The checked-in generated client lacks its referenced
  data-type definitions and targets Elm 0.17; server imports retired MPFR APIs.
  No server was launched, no network/malformed-input probes were performed,
  and no production readiness is inferred from the visualization example.
- Additional Hyper cross-reference read: hypersolve/root_isolation 2550--2735,
  2980--3195 and 3470--3535; certification 1--125 and 313--378. Hypersolve
  already has a fraction-free BigInt Bernstein sign-count path with explicit
  positive denominator clearing and a degree-80 regression beyond u64 binomial
  range. Thus integer scaling for sign counts is already subsumed; the open
  candidate is specifically retaining a shared scale through recursive
  subdivision, not adding a duplicate sign-count algorithm. Current generic
  subdivision has linear live storage and guards endpoint roots before
  classifying one variation as isolating. Candidate certification already
  separates numerical proposals, exact residual replay, whole-ball signs,
  bounded unknown and domain failures. DCT/Chebyshev transforms would add a
  new representation layer, not replace an existing scalar hot path directly.
- All 42 fnreps/benchresults/sine+cos logs / 1,139 lines completely read:
  integrate ball at 10/11/12, dball 10/15/20/25, fun 06/08/09, lpoly
  10/30/50/70/90 and poly 30/50/70/90/110; max ball 10/15/20/25,
  dball 10/30/50/70/90, fun 10/15/20, lpoly 10/30/50/70/90 and poly
  30/50/70/90/110. Every log has 27 lines except the five integrate-poly
  logs with 28. Total 341 files / 42,599 lines. All report exit zero, but
  requested and achieved precision differ materially: global integration at
  requests 30/50/70/90/110 reports 28/32/53/73/93 bits, versus local
  integration at 10/30/50/70/90 reporting 14/36/57/74/96. Printed radii
  are only strict power-of-two upper bounds, not the actual stored radii.
  These historic process timings are not matched-accuracy Hyper comparisons.
- Completed DCT-I timings: same working precision, grids n=8/16/32 (n+1
  outputs), 13 independently seeded exact-dyadic input vectors rotated via
  IORef, all inputs pre-forced and all outputs NFData-forced, 13 warmups,
  NOINLINE kernel, five calibration runs and 21 paired ABBA rounds pinned to
  CPU 6. Median paired fast/reference ratios and 5,000-resample bootstrap
  95% intervals are: p53 n8 0.34206 [0.33979,0.34373], n16 0.13400
  [0.13316,0.13452], n32 0.062268 [0.061802,0.062976]; p100 n8 0.34080
  [0.33581,0.34231], n16 0.13221 [0.13172,0.13336], n32 0.062242
  [0.061712,0.063013]. All six benchmark sets pass their 26 exact
  double-transform identity checks. Raw paired samples and validation are
  `/tmp/aern2-dct-bench.json`; drivers `DCTPerformance.hs` and `dct-bench.py`
  are under `/tmp/aern2-audit`. No compilation or memory instrumentation was
  run concurrently with these timings.
- Fast DCT intervals are not a strict width improvement: per-coefficient
  maximum fast/reference radius ratios range 1.05--1.86, while median ratios
  range 0.0524--0.5626. Minimum reported output accuracy improves by 1--3
  bits in these sets. This is same-working-precision kernel evidence, not a
  matched-output-accuracy end-to-end result. The direct comparator repeatedly
  computes its cosine matrix; no precomputed-matrix competitor was timed.
  Both kernels passed the independent MPFR tests already recorded above.
- Serial whole-process GHC RTS measurements at p100/n32, 26 measured calls
  plus 13 warmups and common input setup: direct allocates 10,676,166,016
  managed bytes, maximum residency 103,704 bytes; fast allocates 708,656,944
  bytes, maximum residency 167,056 bytes. Logs `/tmp/aern2-dct-{ref,fast}-rts.log`.
  Fast has much lower allocation volume but somewhat higher sampled live
  memory; native allocations are not the same metric. Both kernels remain
  in one executable, so no incremental binary-size claim is possible. No
  Hyper patch is accepted: there is no existing DCT scalar hot path to replace.
- Nine additional files / 876 lines completely read: bench-chart/ChartTweaks
  (130), Main (533), cabal (32); ireal-benchOp/LICENSE (12), cabal (26),
  src/Main (124); docs/portal appls (3), bench (3), intro (13). Total
  350 files / 43,475 lines. Chart generation clips all accuracies above 100
  bits to 100, collapses identical points into sets, joins them in ascending
  order and does not carry repeated-sample uncertainty. Axis rounding is
  presentation-only. The ireal adapter uses unseeded random sampling and
  unsafePerformIO; it prints interval-derived accuracies, not independent
  values, and pins obsolete aern2-mp 0.1. Its approximation request is a
  working-precision request, not an enforced final accuracy. No new reliable
  benchmark or scalar mechanism follows from this adapter or portal prose.
- bench-chart/LICENCE (30 lines) completely read: BSD3 notice retained.
  Total 351 non-PDF text files / 43,505 physical lines. Two bundled PDFs
  completely text-read: cca2016/representations-feasibly-approximable (3 pages,
  137 extracted lines), talk-cca-2016 (39 pages, 555 extracted lines).
  All substantive embedded representation, Newton, root-count and benchmark
  diagrams on slides 9--17 and 25--36 were also rendered and visually read.
  These diagram readings do not credit their separately tracked SVG/PDF
  sources or copies as fully read. The presentation motivates piecewise
  polynomial uniform approximations for closure under integration/maxima,
  distinguishes representation complexity from evaluation complexity, and
  describes avoiding separable-part coefficient blowup in range search.
  None supplies a missing scalar algorithm; the already-audited implementation
  still must establish every domain, partition, derivative and remainder
  invariant. Historical plotted curves do not constitute independent proofs.
- Slide 17's stated Newton error squaring is valid for the residual
  r=1-f*q, not for unscaled absolute inverse error: e'=f*e^2. Exact control
  f=4, q=3/10 has e=1/20 and e'=1/100, exceeding e^2=1/400. An absolute
  error below 1/2 alone is not a contraction certificate for unrestricted
  f>=1. The donor code's residual/minimum checks, where valid, are stronger
  than this slide shorthand. No direct transfer of the shorthand bound.

### ERC paper semantics and specification audit

- Fully read every line of the 55-page paper (including references and
  Appendix A): /tmp/aern2-erc-paper.txt, 2,977 extracted lines. Version 10
  of https://arxiv.org/abs/1608.05787v10, revised 2024-06-21; published in
  LMCS 20(2), 2024, DOI 10.46298/LMCS-20(2:17)2024. PDF SHA-256
  b93fdee5a9772ecc9855273c28296ab9f1c8e126535fca5004a2fcb12d02a179.
  Critical displayed formulas on pages 26, 32 and 40 were also rendered and
  inspected to exclude text-extraction errors.
- Useful contracts: distinguish an unobservably unresolved Kleenean from
  invalid/undefined computation; finite overlapping choice must allow
  progress without exact equality; all nondeterministic paths must terminate
  and meet the same error bound for a real limit to denote one exact value.
  State assignment fixes a chosen value; independently reevaluating the same
  multivalued expression need not preserve that choice. This matters for
  algebraic rewrites and replay. The decidable base specification language
  intentionally restricts integer multiplication/coercions and does not
  automatically cover richer transcendental extensions or all program
  termination properties. These are design/proof contracts, not new measured
  scalar speedups or a ready-made verification engine.
- Printed recipes need corrections. All following checks are independent
  finite-set/Rational models of paper formulas, NOT a build of a formal ERC
  interpreter, NOT new failures of the prior import-only Haskell slice, and
  NOT a disproof of the intended corrected abstract constructions:
  /tmp/aern2-audit/PaperContractFormulas.py, output
  /tmp/aern2-paper-contract-formulas.log.
- Example 2.1(7) tests each precision independently, but the stated delta_K
  names must remain at +1 or -1 once decided. Valid delta_R names of 0 and
  1/2 can produce digits [0,1,0,1,1,...]. At n=1 use integer approximants
  (-1,2); at n=2 use (1,1); at n>=3 use exact (0,2^(n-1)); all satisfy the
  stated unit-error contract. Latching the first certified sign repairs this
  shape. No equality decision is needed or inferred.
- Example 2.2(6) infers that both branches are defined from the definedness
  of Kond's result. Kond(true,{0},{bottom})={0} directly disproves that
  intermediate inference. Its described evaluator first requesting both
  branch approximations can consequently wait on an irrelevant partial
  branch. A correct implementation needs lazy selection/fair interleaving
  with the condition; this does not invalidate computability of a suitably
  implemented continuous conditional. The Haskell ERC helper already takes
  only the selected branch when its current condition is CertainTrue/False.
- Algorithm 1 initializes y=1, z=x/y under merely x>=0. At x=4 and p=-4
  the only available choose index is 0, so it returns 1 with promised error
  1/16 although sqrt(4)=2. The claimed initial bracket is reversed. The
  Haskell example differs: it initializes y=x and wraps the positive ranges;
  its earlier positive-domain test pass does not validate this printed
  algorithm. A separate range/zero-domain contract is required.
- Lemma 6.4's existential graph-composition formula loses demonic
  definedness. With first result set {0,1}, and second-stage images
  0->{bottom}, 1->{1}, the semantic composition is {bottom,1}, whose
  Definition 6.3 graph is empty, but the printed formula admits result 1.
  This is expressible using inverse(2^choose(true,true)-1). Exhaustive
  enumeration of all 343 compositions over two ordinary values plus bottom
  finds 24 mismatches; adding universal definedness of every intermediate
  choice yields zero mismatches. Therefore the printed formula must not be
  used unchanged to generate total-correctness certificates.
- The printed cont predicate on page 32 is vacuous for delta>b-a because
  its antecedent includes x+delta<=b with x>=a. This observation concerns the
  written formula; it does not refute trisection for an actually continuous,
  unique-root input. Section 7's claimed variant decrement L=2^(p-2) is
  independently too large: p=0, [a,b]=[0,5/8], f(x)=x-1/4 permits continuing
  and either inner branch, but width drops only 5/24 < 1/4. The smaller
  dyadic bound 2^(p-3) is valid under the stated continuation inequality.
  Appendix A.1 also writes the original state sigma in C's guard where the
  evolving state delta is needed; a terminating two-state loop gives an
  empty printed exit set after two unfoldings instead of {0}.
- Hyper comparison reread: hyperlimit/predicate 145--240; Hyperreal
  computable/node/approximation_queries 40--137 and
  real/arithmetic/representation 296--355; Hypersolve certification 175--250.
  PredicateOutcome keeps Decided and Unknown distinct, certified signs are
  retained facts, and near_integer already provides the total overlapping
  rounding operation without digit-by-digit search. Candidate replay retains
  domain failures and all_satisfied requires proof for every nonempty active
  row set. These subsume the applicable contracts; no generic nondeterministic
  program interpreter or continuous-function layer is added. Paper-derived
  evaluator/proof recipes remain rejected until corrected and qualified.
- Checkpoint: AERN2, Hyperreal and Hypersolve tracked worktrees remain clean;
  no new Hyper production change accepted in this continuation. Completed
  timing and memory runs have exited. Numerous historical/generated artifacts
  still need coverage/reconciliation, so AERN2 and the overall audit remain
  explicitly open.

### AERN2 artifact reconciliation and matrix-report provenance

- Additional complete reads: 12 fnreps/plots instruction files / 132 lines
  (bumpy.ct2, bumpy2.ct2, fracSin.ct2, fracSinSC.ct2,
  function.ct2.template, runge.ct2, rungeSC.ct2, sine+cos, sine+cos.ct2,
  sine+cospi.ct2, sinesine+cos.ct2, sinesine.ct2); linear/bench/results.html
  (234), linear/bench/all.js (315 newline-count lines, including its final
  non-newline closing row), and net/tools/visualise-netlog/nodepos-fft-n2.js
  (14) plus nodepos-fft-n4.js (36). Also read all 23 fnreps logs without an
  exit status, 26 lines total: 22 one-line timeout markers and one four-line
  debug-only log, bumpy2/run-integrate-bumpy2-lppoly-25.log. The latter is
  NOT empty and contains no result; all four lines report NoInformation and
  Exact for numer/denom/inv/frac. None is a completed benchmark.
- Cumulative human-read text coverage is 390 files / 44,262 physical
  newline-count lines. Exact manifest reconciliation before all.js and the
  23 incomplete logs gave 366 files / 43,921 lines: all 310 files classified
  source-or-metadata, ten .ct2 files, the two handwritten HTML files, the
  two nodepos JS files, and the 42 previously read sine+cos logs. Counting
  inventory reads or parser passes as human reads is explicitly avoided.
- /tmp/aern2-audit/ArtifactInventory.mjs produced the per-path Git blob,
  SHA-256, byte/line counts, types, log metadata and validation results in
  /tmp/aern2-artifact-inventory.json. All 1,483 tracked paths reconciled:
  310 source-or-metadata, 918 logs, 68 PDFs, 73 SVGs, 40 PNGs, four regular
  HTML, 16 CSV, one tgz, ten ct2, 19 gz, nine regular JS, 15 symlinks.
  All symlinks resolve to their expected local common visualization files.
  All 214 format checks pass: nine JS syntax parses, four HTML inline-JS
  parses, 73 non-network XML parses, 68 PDF inventories, 40 PNG decodes,
  20 gzip CRC/decompressions. Scripts were parsed, not executed. These are
  readability/integrity checks, NOT numerical qualification or source-read
  coverage. No user or donor production file was changed by this script.
- Log inventory: fnreps 350 exit-zero / 23 no-exit-status; linear 315
  exit-zero; net 230 exit-zero. Parsed net self-reported accuracies do not
  fall below the requested precision, but this is no independent oracle.
  Unread generated log bodies remain uncredited as line-by-line reads.
- Plot instructions sample only 501 points over [-1,1] into a 600x300
  raster with oversampling eight; these are pictures, not certified ranges.
  The nested sine+cos instruction uses cos(10*x), unlike cos(3*pi*x) in
  the current benchmark driver, confirming the earlier provenance mismatch.
  nodepos files provide ten/32 graph-layout coordinates only.
- The historical matrix report uses M[i,j]=1/(i*n+j+1), a nonsymmetric
  Cauchy matrix. Its dimension chart selects the FIRST record for each
  increasing size that is MPFloat or achieves >30 bits. Thus compared
  methods need not use equal working precision or equal achieved accuracy;
  changing data order can change the selected points. MPFloat rows have
  sentinel accuracy -10000000 yet are always included. The n=80 accuracy
  chart also excludes rows at <=30 bits. Plotly's unversioned latest CDN
  and MathJax 2.7.7 were identified, not loaded/executed.
- all.js contains 126 product and 189 solve records. Product uses seven
  sizes (10,20,30,50,80,130,210), six working precisions (200 through 2100),
  and three methods; solve adds 3400/5500/8900 bits. All recorded dates
  span seven seconds despite much longer runs: these are import times,
  not execution timestamps. Failed/low-accuracy ball solves and all float
  rows use the same negative sentinel. At n=80 the dimension chart selects
  MPBall p=2100 / 374 achieved bits versus MPBallViaFP p=800 / 124 bits.
  These are not accuracy-matched timings. Zero CPU times are floored to
  0.01 separately for user/system by the already-read import script. The
  report's GHC 9.0.2 / Ubuntu 20.04 / i7-4710MQ / 16GB setup is historical,
  distinct from this audit's compatibility toolchain and current host.
  Large historical memory differences remain motivation, not a new measured
  Hyper benefit. No production transfer is justified by these plots.

### AERN2 historical data, fixture and visual closure checkpoint

- All 16 CSV files completely read, 1,116 lines: six fnreps
  results-ppoly-test-{chp,chpbnds,pb}-{cdar,mpfr}.csv files (five each),
  results.csv (338), results-test.csv (450), net/bench/{fieldops,logistic,ops}
  (31/21/71), net/bench/ireal/{fieldops,logistic,ops} (31/21/71), and
  net/bench/native/{fieldops,logistic} (31/21). Also completely read
  fnreps/benchresults/fracSinSC/run-integrate-fracSinSC-lppoly-20.log (31).
  Cumulative human-read parent text coverage: 407 files / 45,409 lines.
  Rereads of Utils/Bench, the serializer and benchmark driver do not inflate
  this count. Generated artifact parser reads do not inflate it either.
- /tmp/aern2-audit/HistoricalReportCheck.mjs and its output
  /tmp/aern2-historical-report-check.json reconcile all 337 results.csv
  rows and all 315 matrix all.js rows against their actual stored logs:
  accuracy, user/system time with the documented zero-time floor, and RSS
  agree, with no missing log or mismatch. This checks faithful transcription,
  NOT correctness of the numerical result. 44/337 fnreps rows and 59/449
  older results-test rows report fewer achieved bits than their parameter.
  For example fracSinSC integrate lppoly parameter 20 achieves only five
  bits and explicitly records that result; the CSV has not concealed it.
- The six 2019 ppoly comparison CSVs each contain four single runs at the
  same achieved accuracies (9/14 bits for max, 10/16 for integration).
  Across the stored cases MPFR is faster for max while CDAR is faster for
  integration. No repeated uncertainty estimates or current matched host
  exist; this is evidence against a blanket backend conclusion, not a reason
  to replace Hyper's integer/dyadic backend. The 2017 net/native/ireal tables
  mix independent historical dates and CPU/system costs and cannot stand in
  for this audit's controlled rotating-input measurements.
- Both generated Criterion reports were inventoried and their complete
  embedded JSON data parsed without running vendor JavaScript. All 24 range
  and 27 rootIsolation names/means match the overview data; all raw timing
  records have finite nonnegative time and positive integral iteration counts.
  The range rootMultiSetSize/12 row has only nine samples, R^2=0.8271,
  mean 0.1123s and 95% CI [0.09815,0.13348], with severe outlier variance.
  Root isolation oneNRoot2OneRoots/10 has R^2=0.9895, also marked severe.
  The report's explanatory prose (range.html 1645--1708) was read. Generated
  HTML/vendor scripts and every raw sample number are not credited as
  fully human-read source; this is data consistency/statistical review only.
- All five generated network trace payloads parse, their node sources and
  client/provider references resolve in event order, and query/answer counts
  balance per endpoint pair with no outstanding requests. n2 serial/parallel
  each have ten nodes and 16 queries; n4 serial/parallel/MP-parallel each
  have 32 nodes and 56 queries. Serial logs report ten/32 cache-empty events,
  versus 12/47 in parallel and 41 in MP-parallel. Repeated concurrent misses
  motivate coalescing only if measured in a real consumer; these small stored
  traces are neither a benchmark nor evidence for changing Hyper's cache.
  Cache-description wording differs by backend; a previous exploratory
  search for 'used cached' was corrected to retain the full descriptions,
  not misreported as zero cache use. Compiled Elm main.js is generated from
  the already-audited visualization source, not another scalar algorithm.
- /tmp/aern2-audit/SerializedSampleCheck.mjs, output
  /tmp/aern2-serialized-sample-check.json, parses all 19 compressed fixture
  files as data with no execution: 21,800 polynomials / 1,661,698 exact
  dyadic coefficients, domain [0,1], correct ordered sparse indices and
  filename degree, no unequal interval endpoints. There are 102 omitted
  zero coefficients; an initial dense-index check was corrected because
  sparse zero omission is legitimate, not a donor failure. No build of
  the obsolete deserializer or independent arithmetic qualification of the
  whole univariate package is inferred from this parser.
- These fixtures have far fewer distinct inputs than their names suggest:
  the nine 100-pair files at degrees 100/150/200/250/300/350/400/450/500
  have 57/66/68/68/71/74/74/74/74 distinct pairs. The ten 1,000-pair files
  at degrees 10 through 100 have 10/10/12/15/12/39/58/92/139/196 distinct
  pairs. Utils/Bench.hs reuses mkQCGen(148548830) in every ten-item batch,
  varying only QuickCheck size. Thus repeatability is intentional but not
  independent input diversity. The audit's separate DCT/polynomial timing
  workloads use explicitly rotating inputs, not these repetitions. Do not
  interpret 1,000 fixture entries as 1,000 independently sampled workloads.
- Rendered and visually inspected ALL 66 single-page PDFs, separate from
  the two previously read bundled documents: 42 benchmark charts, 14
  presentation diagrams, ten sampled function plots. Evidence is
  /tmp/aern2-chart-review.6lytAG/sheet-{0,1,2,3,4,5}.png, generated by
  /tmp/aern2-audit/render-charts.sh. The plots agree qualitatively with the
  stored data: logarithmic time/RSS axes, per-operation representation
  tradeoffs, derivative-assisted range search, and local versus global
  approximation shapes. They provide no new certified numerical algorithm.
  Paper charts omit axis titles for embedding and squeeze the 20/24-bit
  ticks together; their connected curves do not imply confidence bands.
- The hat-reprs.tgz archive contains only eight existing diagram assets:
  DFun/Fun/PPoly PDF+SVG copies are byte-identical to tracked counterparts;
  the Poly PDF/SVG are older drawings, not hidden implementation source.
  The differing archived PDF was separately rendered and inspected at
  /tmp/aern2-chart-review.6lytAG/archive-hat-Poly.png. It shows the same
  global approximation-envelope concept; no separate transfer follows.
- Parent manifest closure: all handwritten implementation/support text is
  read. Remaining 1,076 parent paths are explicitly classified, not silently
  claimed as source-line coverage: 852 uncredited historical log bodies,
  two generated Criterion HTML, six generated JS (five trace payloads and
  compiled Elm), 19 compressed fixtures, one diagram archive, 73 SVGs,
  40 PNGs, 68 PDFs and 15 symlinks. PDFs are covered as described above;
  other assets have integrity/data checks, not invented human byte reads.
- Source review can now move to the two already identified Hyper candidates:
  reuse exact matrix factorization across inverse columns and retain a shared
  positive Bernstein scale across recursive subdivisions. Both remain open
  pending controlled correctness/time/memory/size comparison. No AERN2
  production patch is accepted and the overall multi-reference goal remains
  active. All processes started in this checkpoint have exited. AERN2 and
  Hypersolve remain clean; a new concurrent user edit appeared in Hyperreal
  src/real/arithmetic/tests.rs and was neither changed nor attributed here.

### AERN2 transfer follow-up: dimensionless Krawczyk contraction correction

- The factorization-reuse investigation first compared Hypersolve's current
  inverse-column Gauss-Jordan loop with a jointly augmented identity matrix
  and its public multi-RHS Bareiss solver. Current Hypersolve baseline is
  2bca88cc43487239269ad8bc40746bb118e4b880. Reviewed interval.rs proof/report,
  radius, gradient/variation, matrix solve/inverse and tests; bareiss.rs
  single/multi-RHS elimination and certificate construction; corresponding
  smoke/property tests and the certification benchmark. This is targeted
  Hyper architecture review, not new donor source coverage.
- A necessary independent public-certificate oracle exposed a correctness
  defect BEFORE any inverse implementation was changed. For f(x)=x^2+x/4,
  candidate zero and radius 1/4, the original implementation reports
  CertifiedUniqueRoot on [-1/4,1/4], although zero and -1/4 are distinct
  exact roots in that closed box. Both roots and box membership were checked
  with exact BigRational arithmetic. The original reported image radius is
  1/4 and its purported contraction bound is 1/2; the derivative operator's
  actual infinity norm is two. This is a numerical proof defect in Hyper,
  not a reference-library defect and not a performance-only opportunity.
- Cause: quadratic_derivative_variation already contains the box radii.
  contraction_row_bound multiplied its operator entries by the radii AGAIN
  and compared the dimensional image displacement with dimensionless one.
  For g(x)=x-C*f(x), C=J(x0)^-1, the correct ordinary infinity-norm sufficient
  bound is sum_j sum_k |C_ik|*V_kj. The patch computes this as
  sum_k |C_ik|*sum_j V_kj, removes the second radius multiplication and its
  radius-map/variable parameters, and documents the report field's meaning.
  The exact self-map image check and strict contraction threshold remain.
  No floating approximation, new dependency, layout or public signature is
  introduced. A weighted anisotropic norm is NOT implemented or claimed.
- The primary mathematical cross-check was Rump's author-hosted
  https://www.tuhh.de/ti3/paper/rump/Ru95b.pdf, section 3: self-map inclusion
  and derivative-operator contraction have distinct roles. Only the relevant
  section was read, not credited as a complete new paper audit.
- Added three permanent unit tests: the two-root counterexample; 80 exact
  scaled scalar cases (eight signed a values, centers zero/-a, and five
  radii including zero, threshold and image-outside cases); and a coupled
  two-variable system with unequal radii. The coupled oracle independently
  computes |C|*V row sums 49/250 and 19/125 and image radii 83/25000 and
  227/50000. Large valid boxes previously rejected also gain completeness:
  a=16,r=4 has corrected norm 1/2 rather than erroneous bound two.
- Regression discipline: the first partial-name command combined with
  --exact selected ZERO tests and is not qualification. Retried with the
  complete interval::tests::multivariate_krawczyk_rejects_two_roots_in_small_box
  name: one test ran and failed on the original CertifiedUniqueRoot status.
  It passes after the correction. All three final focused tests pass.
- The external harness /tmp/hyper-inverse-audit.nizGgs/src/main.rs checks
  117 actual public certificates (13 rotating seeds over dimensions
  1/2/4/8/12 dense, 4/8 Cauchy, and 8 diagonal/permutation matrices).
  Every successful status, Newton step, image radius and corrected norm is
  checked against exact BigRational identities, and each reference inverse
  satisfies A*C=I. Public output is
  /tmp/hypersolve-krawczyk-public-oracle.log. This is not merely a copied
  private-helper result comparison. Memcheck public-check 8 dense passes
  with zero errors and no definite/indirect/possible losses; 66,792 bytes
  in 558 blocks remain reachable. Full process heap usage is 501,661
  allocations / 29,035,624 bytes, including setup and oracle work, not a
  per-certificate performance claim.
- Final current-tree all-feature debug AND release suites each pass
  771 tests (414 library, 9/2/4 auxiliary, 131 property, 211 smoke; zero
  doctests). cargo clippy --offline --all-targets --all-features -- -D warnings,
  cargo fmt --check and git diff --check pass. Evidence logs are
  /tmp/hypersolve-krawczyk-final-{debug,release}.log and
  /tmp/hypersolve-krawczyk-clippy.log. Earlier default/debug and all-feature
  release runs passed 769/770 tests before the third regression was added;
  those earlier counts are superseded, not silently combined.
- The existing multivariate Krawczyk benchmark used 16 rows for two
  variables and immediately returned ShapeMismatch, never timing inversion
  or proof. Replaced that one benchmark with square dimensions 1/2/4/8,
  exact root all-ones, radius 1/16, and a successful-certificate assertion
  outside the timing loop. Other users of the nonsquare fixture are intact.
  The benchmark qualification and controlled correction A/B measurements
  are in progress; inverse-factorization production code is UNCHANGED.
- Concurrent user edits in Hyperreal arithmetic/tests.rs and
  arithmetic/linear_algebra.rs are preserved and not attributed here.
  For fair performance/size comparison, isolated clean local clones pin
  Hyperreal 3f5786794e0ad127c264e44e9c9451ab58b077a7, Hyperlattice
  d783af0b5ba52cacfeac62b1b25e1f9e36a8e7b0, Hyperlimit
  7bf9261fea34bb2ddc96ca612c40babf4244f71f and the Hypersolve baseline above.
  Both isolated binaries use identical external driver source and lockfile,
  differing only in the production contraction helper/call correction.
  Those clones exclude uncommitted user changes; current-tree gates above
  separately exercise the actual live dependencies.
- Controlled A/B is complete: /tmp/hyper-krawczyk-ab.WlCBFy/measure.mjs
  uses 21 alternating ABBA/BAAB process blocks per workload, CPU 6 affinity,
  13 rotating exact input systems, untimed construction/oracle/warmup,
  calibration to 70ms (whole 13-input cycles, cap 13,000 calls), and
  measured public proof construction plus result destruction. Box radii
  are 2^-512 so every Cauchy case also admits a strict proof. The old
  result fields are checked against their legacy formula, NOT represented
  as correct derivative bounds. Both methods' exact root/image checks pass.
  Rust 1.97.0/LLVM 22.1.6, release debug=1, Ryzen 7 5800X3D. An initial
  sandbox child-process EPERM was not treated as benchmark evidence; the
  approved rerun outside the sandbox completed all 504 timing records.
- Median fixed/baseline paired ratios with 5,000 bootstrap resamples and
  95% intervals (seed 314159265): dense n=1 .94369 [.93244,.95423],
  n=2 .84136 [.83397,.84413], n=4 .85853 [.85011,.86037], n=8 .93736
  [.93375,.94202], n=12 .91916 [.91488,.92664]; Cauchy n=8 .94038
  [.93851,.94298]. Separate baseline/fixed time medians in microseconds
  are 3.291/3.191, 13.799/11.434, 97.817/83.714, 1456.521/1366.459,
  13704.119/12586.238, and 652.826/613.255 respectively. Ratios are
  medians of paired block ratios, not quotients of these separate medians.
  Raw evidence /tmp/hypersolve-krawczyk-ab.jsonl; summarizer
  /tmp/hyper-krawczyk-ab.WlCBFy/summarize.mjs.
- For 13 complete proof calls excluding setup, baseline -> fixed allocation
  counts/requested bytes are: n=1 dense 280/20,124 -> 272/19,340;
  n=2 1,653/188,860 -> 1,461/154,428; n=4 10,589/1,353,324 ->
  9,681/1,150,572; n=8 191,203/14,455,996 -> 180,959/13,067,140;
  n=12 2,571,368/105,541,044 -> 2,217,418/95,117,588; n=8 Cauchy
  58,042/7,222,132 -> 55,832/6,899,420. These count allocations/reallocations
  and cumulative requested bytes, NOT peak live storage or RSS.
- Identical-driver stripped sizes: 2,523,632 -> 2,522,008 bytes (-1,624).
  size text: 2,289,371 -> 2,287,799; data: 230,240 -> 230,184.
  Full binaries baseline/fixed SHA-256 are
  8c97d5315912c3dbaab74d5f40513586ba834d87693ea7c3b027ce082683029e and
  cb305c9b8be605ca0077a61de9dfa8d8019ee0177b3ad549b87bf4b5c233d04d;
  shared driver source SHA-256
  5013facc8d638c197986c05a57f598a227a325abe93f9c316d4fdc240529bfc6;
  lockfile 14c84d429373b24666c35cf23aee37968e2499c6e75578d8bf6fb6d33fe354ba.
  This is one linked consumer, not a universal binary-size promise.
- The repaired actual Criterion cases all pass their untimed certificate
  assertions. Thirty-sample estimates (95% intervals), dimensions 1/2/4/8:
  1.7073us [1.6955,1.7197], 4.8796us [4.8691,4.8915], 40.089us
  [39.978,40.235], 350.97us [348.63,353.61]. These use live dependencies
  and radius 1/16; they are useful future successful-proof baselines, NOT
  directly comparable with the separate pinned 2^-512 A/B inputs. Command
  uses HYPERSOLVE_SKIP_BENCHMARK_REPORTS=1 so generated reports are untouched.
  Evidence /tmp/hypersolve-krawczyk-criterion.log.
- Decision: retain the production proof correction, three regression tests,
  and truthful square-system benchmark. No copied donor code, new public
  API or inverse-kernel change. The correctness gain is primary; measured
  speed, allocation and size gains also support retention. The broad audit
  remains active and the inverse/shared-Bernstein-scale candidates remain open.
- Retained as Hypersolve commit 1273a7a (Fix dimensionless multivariate
  Krawczyk contraction proof). The two-file worktree is clean afterward;
  no push was made. Concurrent Hyperreal edits remain untouched.

### AERN2 inverse reuse prototype checkpoint (not a production change)

- /tmp/hyper-inverse-audit.nizGgs/src/main.rs compares the original
  per-column inverse, one augmented-matrix elimination with a dummy zero
  RHS, and public multi-RHS Bareiss with its certificate payloads. All
  196 nonsingular exact matrices (four families, seven dimensions including
  empty, seven seeds) pass independent two-sided BigRational inverse
  identities and entry equality across all three methods. Singular controls
  preserve the old/joint error pivot. These rational checks do not yet
  establish identical partial-pivot behavior for arbitrary exact reals.
- Kernel timings use 13 rotating matrices, 21 alternating symmetric
  six-leg old/joint/Bareiss blocks, CPU 6, input clone and result destruction
  included, construction excluded. Joint/old medians and 95% bootstrap CIs:
  n=1 dense 1.12481 [1.11778,1.13364]; n=2 .76353 [.75362,.76678];
  n=4 .50425 [.50288,.50614]; n=8 .36549 [.36216,.36638];
  n=12 .26226 [.26201,.26493]; Cauchy n=4 .51770 [.51603,.52015],
  n=8 .26763 [.26676,.27046]; n=8 diagonal .26060 [.25894,.26238],
  permutation .25268 [.25183,.25579]. Evidence
  /tmp/hyper-inverse-bench-{n}-{kind}.csv and the harness summarize.mjs.
- Public multi-RHS Bareiss/old ratios in the same order are 1.79261,
  1.26537, 1.03698, .60023, .48689, 1.25684, .75388, 1.47235,
  1.96208. Its determinant/Cramer/residual evidence has a real cost and
  its unsupported/unknown handling differs; a blanket API substitution
  loses to the simpler joint kernel on every measured workload and is
  not retained. Bareiss may still have independently justified uses.
- For 13 kernel calls, old -> joint allocations/requested bytes, dense
  n=1: 144/6,448 -> 105/8,008; n=2: 534/56,040 -> 360/45,728;
  n=4: 4,504/587,392 -> 2,239/312,112; n=8: 141,772/9,753,712 ->
  60,998/3,391,280; n=12: 1,466,458/72,560,168 -> 348,962/16,966,952.
  /tmp/hyper-inverse-allocations.jsonl also retains Bareiss counts. Again
  these are cumulative allocation requests, not peak memory.
- Joint elimination is promising from n=2, but the dummy RHS adds useless
  work and n=1 regresses about 12.5% while allocating more bytes. Next
  experiment should eliminate that overhead and qualify pivot/fallback
  semantics plus actual public proof performance before any production
  inverse change is accepted. No generic approximate factorization or
  weaker pivot evidence is authorized by these measurements.

### Joint inverse elimination qualification in progress

- A second external prototype, /tmp/hyper-inverse-joint.qM4hlJ/src/main.rs,
  removes the dummy RHS operations while keeping the same pivot search,
  exact division and policy-certified reciprocal fallback. All 196 exact
  matrices again pass two-sided BigRational identities and singular controls.
  The initial n=1 rotating-input kernel medians are 344.94ns old,
  382.23ns dummy-RHS, 296.27ns no-dummy (not a public-path claim).
  Reserving exactly the augmented row width also avoids Vec's small default
  growth slack: final n=1 allocation demand is 79 calls/5,512 requested bytes
  versus the old 144/6,448 for 13 inverse constructions.
- The candidate in Hypersolve interval.rs augments A with I and reduces it
  once, then returns the right half. A single const-generic Gauss-Jordan
  helper shares the original elimination between scalar-RHS affine solves
  and augmented inverses. Const selection removes dummy RHS work rather
  than adding dynamic branches. Pivot order, certified nonzero checks,
  exact normalization/fallback and error pivot indices are preserved.
  Arithmetic complexity falls from repeated O(n^4) elimination to O(n^3).
  This is reuse of Hyper's own exact kernel, not copied AERN2 LU code.
- Two further permanent tests pass: 196 matrices check A*C=C*A=I with an
  independent BigRational product, plus nonzero scalar solutions against
  preselected rational roots. Families are dense diagonally dominant,
  Cauchy, diagonal and cyclic permutation; dimensions 0/1/2/3/4/6/8,
  seven seeds each. The other test covers coupled sqrt(2) pivots,
  a row swap with an opaque but policy-certified 2^-3000 pivot requiring
  the nonzero reciprocal fallback, exact singular matrices, and unresolved
  pivots at columns zero/one under STRICT and APPROXIMATE_512 policies.
- Current-tree all-feature debug/release suites each pass 773 tests
  (416 library plus the existing 9/2/4/131/211 suites); Clippy all-targets,
  all-features -D warnings and diff whitespace checks pass. Logs
  /tmp/hypersolve-joint-{debug,release,clippy}.log. No unrelated tree was
  edited. The inverse candidate is NOT yet committed or declared retained.
- Paired public quadratic/affine proof timing and allocation measurements
  are running from pinned clones, excluding concurrent Hyperreal edits.
  Quadratic baseline is the corrected-contraction binary from the previous
  checkpoint, so this comparison isolates inverse reuse from the correctness
  fix. Affine baseline/new drivers have identical source and nonzero known
  root/step oracles. The clone interval.rs is byte-identical to the candidate.
  Initial small-case timing has visible host variability; report paired
  bootstrap uncertainty and do not infer an n=1 public speedup prematurely.
  Evidence paths /tmp/hypersolve-joint-ab.jsonl and
  /tmp/hyper-inverse-joint.qM4hlJ/{measure,summarize}.mjs. A syntax typo in
  the new orchestration script was corrected before any successful sample;
  its failed invocation is not measurement evidence.
- Linked size qualification is complete: quadratic baseline/new stripped
  2,522,008/2,512,704 bytes (-9,304); affine 2,394,248/2,385,576 (-8,672).
  Quadratic text 2,287,799/2,278,815; affine text 2,165,027/2,156,499.
  These are scoped linked drivers, not a universal binary-size assertion.
- While timing runs, rechecked Hypersolve root_isolation.rs 1530--1710,
  2550--2740 and 2984--3195 for the next pending Bernstein experiment.
  Subdivision starts from public exact Real coefficients and uses only their
  signs/endpoint zeros internally; it does not expose intermediate control
  magnitudes in terminal reports. The integer sign-count path currently
  scales different controls by DIFFERENT positive falling factorials; its
  values cannot be fed directly to de Casteljau as if they had one common
  scale. A future shared-scale path must construct or recover a genuinely
  common positive factor and preserve endpoint/depth/unknown behavior.
  No Bernstein code was changed and no new donor source coverage is claimed.

### Joint inverse reuse: controlled qualification results

- The first full joint-elimination trial completed 1,176 timing records and
  28 allocation records. It showed strong larger quadratic gains but a
  4.55% n=1 affine slowdown (95% interval 3.71--5.32%). Symbol inspection
  established that the old scalar solve remained a separate 0x1593-byte
  function, whereas the refactor was inlined into the affine public caller,
  growing that caller from 0x18cb to 0x2df6 bytes. The first variant's raw
  output /tmp/hypersolve-joint-ab.jsonl is retained, not silently discarded.
  Early quadratic measurements also had conspicuous host variability; their
  separate medians must not be divided to invent a paired ratio.
- Added #[inline(never)] to the shared large elimination kernel. A separate
  six-workload, 21-block pilot removed the n=1 affine regression: ratio
  .98133 [.95830,.99088], while quadratic n=1/4/8 ratios are .95569,
  .75673 and .47524. The pilot n=4 affine ratio remained 1.02475
  [1.01516,1.02816]; no claim of universal no-regression is made. Evidence
  /tmp/hypersolve-joint-noinline-ab.jsonl. No arithmetic or pivot policy was
  changed by this compiler-layout qualification.
- Final non-inlined run is complete: 18 workloads, 21 alternating
  ABBA/BAAB blocks each, 1,512 timing records and 36 allocation records.
  Same pinned dependencies, rustc 1.97.0, CPU 6, 13 rotating inputs,
  untimed construction/oracle/warmup, calibrated 70ms batches capped at
  13,000 calls in whole 13-input cycles. Public proof construction and
  result destruction are timed. No other audit compile/test process ran
  during this final paired run. Bootstrap uses 5,000 paired resamples,
  seed 314159265; intervals below are 95%. Input setup is excluded, but
  every timed method's public report has passed its exact field oracle.
  /tmp/hypersolve-joint-final-ab.jsonl and
  /tmp/hypersolve-joint-final-summary.jsonl retain the complete result.
- Tiny-box quadratic systems use radius 2^-512 and center zero. Final
  joint/baseline paired ratios (smaller is faster):

  | Matrix | Ratio | 95% interval | Baseline / joint median us |
  | --- | ---: | --- | ---: |
  | dense 1 | .96588 | [.95566,.97775] | 1.669 / 1.597 |
  | dense 2 | .95130 | [.94967,.95511] | 11.206 / 10.739 |
  | dense 4 | .75914 | [.75628,.76187] | 84.584 / 63.645 |
  | dense 8 | .46993 | [.46668,.47090] | 1365.143 / 638.315 |
  | dense 12 | .38980 | [.38731,.39501] | 12562.651 / 4930.211 |
  | Cauchy 8 | .34229 | [.34063,.34384] | 606.018 / 207.053 |
  | diagonal 8 | .52993 | [.52601,.53067] | 55.084 / 29.177 |
  | permutation 8 | .51981 | [.51723,.52315] | 57.660 / 29.910 |

- The separate ordinary-radius fixture rotates 13 diagonally dominant
  systems with exact root all-ones, radius 1/16, and Jacobian dI+11^T,
  d=2n+seed. Its independent closed-form absolute inverse row sum is
  (d+2n-2)/(d(d+n)); every image/norm/step field is checked against that
  exact rational identity. This avoids relying exclusively on tiny boxes.
  The seed-zero case matches the new permanent Criterion fixture.

  | Dimension | Ratio | 95% interval | Baseline / joint median us |
  | --- | ---: | --- | ---: |
  | 1 | 1.01110 | [1.00526,1.02403] | 1.623 / 1.646 |
  | 2 | .90575 | [.90425,.92351] | 6.621 / 6.029 |
  | 4 | .70229 | [.69793,.70749] | 41.721 / 29.233 |
  | 8 | .43344 | [.42744,.43988] | 347.737 / 150.943 |

- Affine controls have nonzero known roots/steps, a separate source-identical
  driver, and check every returned root and step exactly before timing.
  Their allocation counts and cumulative requested bytes are IDENTICAL
  before/after. Final ratios retain the small tradeoffs explicitly:

  | Matrix | Ratio | 95% interval | Baseline / joint median us |
  | --- | ---: | --- | ---: |
  | dense 1 | .98236 | [.96977,.98918] | .568 / .558 |
  | dense 2 | .97374 | [.96649,.98298] | 2.048 / 1.994 |
  | dense 4 | 1.01103 | [1.00668,1.01872] | 12.231 / 12.396 |
  | dense 8 | 1.00412 | [1.00143,1.00581] | 165.898 / 166.651 |
  | dense 12 | .99873 | [.99403,1.00122] | 950.017 / 948.876 |
  | Cauchy 8 | 1.01003 | [1.00756,1.01998] | 84.311 / 85.136 |

- Allocation evidence, each for 13 complete proof calls excluding setup:

  | Quadratic matrix | Allocations old -> new | Requested bytes old -> new |
  | --- | ---: | ---: |
  | tiny dense 1 | 272 -> 207 | 19,340 -> 18,404 |
  | tiny dense 2 | 1,461 -> 1,293 | 154,428 -> 146,532 |
  | tiny dense 4 | 9,681 -> 7,582 | 1,150,572 -> 902,076 |
  | tiny dense 8 | 180,959 -> 101,144 | 13,067,140 -> 6,872,588 |
  | tiny dense 12 | 2,217,418 -> 1,102,436 | 95,117,588 -> 39,957,828 |
  | tiny Cauchy 8 | 55,832 -> 22,815 | 6,899,420 -> 2,956,540 |
  | tiny diagonal/permutation 8, each | 2,665 -> 871 | 859,196 -> 397,436 |
  | ordinary 1 | 262 -> 197 | 18,148 -> 17,212 |
  | ordinary 2 | 700 -> 544 | 76,564 -> 69,700 |
  | ordinary 4 | 3,546 -> 2,518 | 490,388 -> 369,764 |
  | ordinary 8 | 20,274 -> 10,869 | 2,942,316 -> 1,612,260 |

  These are cumulative allocator/reallocator requests, not peak live memory
  or RSS. Affine unchanged counts/bytes for n=1/2/4/8/12 dense and n=8
  Cauchy are 106/12,012; 232/29,188; 1,202/159,124; 18,169/1,279,716;
  134,619/6,535,132; 7,293/910,716 respectively.
- Final linked sizes, original -> non-inlined joint: tiny quadratic
  2,522,008 -> 2,512,352 stripped bytes (-9,656); affine
  2,394,248 -> 2,385,536 (-8,712); ordinary quadratic
  2,328,536 -> 2,316,456 (-12,080). Tiny quadratic text falls
  2,287,799 -> 2,278,415; affine 2,165,027 -> 2,156,419.
  No dependency or public-layout change. The earlier unannotated candidate's
  linked sizes are retained above as pilot measurements, not final sizes.
- Reproducibility: final tiny-quadratic and affine binaries are
  /tmp/hyper-inverse-joint.qM4hlJ/{quadratic,affine}-noinline, SHA-256
  c6a9c8440af5f87b04e1db4f94998630d1c41c4e4d0c28396cc0ed7f05447ef0 and
  0f114f96c882a53f5d172109d398b223098b0fa4fb403e4941fb617352d77090.
  Ordinary baseline/new hashes are
  9dc042b79baaf8b01fcfe362a044936e2553750b4750e6b5e93d47f11011733e and
  f808e4f78d99a26c65b7f0cc1707e7f52b22c009f72fd2ced2b8bbe78419fade.
  Drivers src/{affine,standard}.rs and Cargo.lock reside in the same folder;
  standard source hash 057ba10357c2847821fd45845b60b39db4bce223e47bd65edb38c40cc113cdd0.
  Tiny quadratic reuses the unchanged prior main.rs/lockfile. Affine's
  original contraction implementation is not linked into its proof path;
  the ordinary/tiny quadratic baselines both include the correctness fix.
- Final non-inlined current-tree debug/release all-feature suites each pass
  773 tests, Clippy all-targets/all-features -D warnings, formatting and
  diff checks pass. Logs /tmp/hypersolve-joint-final-{debug,release,clippy}.log.
  Final benchmark smoke, documentation and Memcheck runs are finishing.
  No production Bernstein change has been made.
- Final gates finished: all four permanent Criterion cases assert successful
  proof and complete 30 samples. Current-tree medians/95% intervals for
  n=1/2/4/8 are 1.6645us [1.6423,1.6878], 4.2687us [4.2385,4.3155],
  28.817us [28.735,28.898], and 155.25us [154.29,156.06]. The prior
  unannotated live-tree run recorded 1.7887/4.6591/30.319/169.15us.
  These unpaired live-tree figures and Criterion's automatic historical
  deltas are NOT substitutes for the pinned paired retention comparison:
  concurrent Hyperreal source and host state can differ across runs.
  Documentation builds with RUSTDOCFLAGS=-D warnings. Final Memcheck
  public-check 8 dense passes all thirteen public certificates and reports
  zero errors, zero definitely/indirectly/possibly lost bytes; 65,320 bytes
  in 549 blocks remain reachable. Total process allocations (including
  setup/oracles) are 421,732 / 22,690,192 bytes. Logs
  /tmp/hypersolve-joint-final-{criterion,doc,memcheck}.log.
- Decision: retain the non-inlined shared elimination, exact-width reserve,
  and two permanent test functions. Public quadratic proof gains are about
  2--3x on the larger measured cases, with up to about 58% lower cumulative
  allocation-byte demand and smaller linked drivers. The ordinary-radius
  n=1 and several affine controls are about 1% slower in the final run;
  that small measured tradeoff is accepted and explicitly preserved above.
  No blanket all-workload speedup is claimed. New source is 21 net production
  lines plus 172 regression-test lines; no duplicated elimination body,
  unsafe code, public interface or dependency was introduced.
- Retained as Hypersolve a20be2326e585f8224dfc5bfd7054c9e2970825f,
  after correctness commit 1273a7a32cd00412f080e85f2e2a1a9a7c16dc8f.
  Hypersolve is clean and no push was made. At the final state check, the
  concurrent Hyperreal changes had been committed by the user as
  c2d147c4613540fe5c2a43dfdaa5c0d010455bba (Retain polynomial root certificates
  in the authoritative scalar evaluator); Hyperreal is now clean too.
  That two-file change is not this audit's work. Earlier test gates exercised
  the live tree; pinned performance intentionally retains the older clean
  dependency snapshot. Debug/release tests are being rechecked against the
  newly clean state. The full reference audit
  remains ACTIVE. Next bounded transfer experiment is common-positive-scale
  Bernstein subdivision; the later AERN/CDAR/ireal and other queued source
  audits are still pending, not claimed complete by this checkpoint.
- Post-commit verification is complete against clean Hyperreal c2d147c:
  all 773 tests pass again in BOTH debug and release with all features.
  Cargo needed no recompilation (0.05s/0.06s preparation), confirming that
  the final pre-commit test artifacts already included this dependency
  source state. Logs /tmp/hypersolve-joint-post-commit-{debug,release}.log.
  All audit-started processes are now finished; both Hyperreal and
  Hypersolve worktrees are clean. Overall multi-reference goal remains active.

### Shared-scale Bernstein transfer: current-state review

- Previous goal turn is classified as progress: two qualified Hypersolve
  commits and complete corresponding test/timing/allocation/size evidence.
  Current authoritative starting state is clean Hypersolve a20be2326e585f8224dfc5bfd7054c9e2970825f
  and clean Hyperreal c2d147c4613540fe5c2a43dfdaa5c0d010455bba. AERN2 remains
  clean at d1ac3664bfb5c7f70fcf68f7fb412d288def65cb. No subagents used.
- Rechecked the reference inventory and applicable Hyperreal agent rules
  (pre-push gates; no push planned). Reread all 403 lines of the donor
  RootsIntVector file, including disabled former implementations; this is
  reread coverage, not additional donor lines. Its common-factor integer
  subdivision motivates the experiment, but its stored triangular Map,
  extrapolation sign issue and previously confirmed endpoint/root failures
  are not transplanted.
- Current Hyper review includes root_isolation.rs 1--120, 325--465,
  1280--1710, 2550--2740, 2984--3195 and endpoint/property tests
  4550--4705; certification benchmark 7650--7698; Hyperreal
  aggregate_products.rs 1980--2125; Hyperlimit predicate 310--405 and
  predicates/order 1--210. Public BernsteinRootCountReport exposes actual
  coefficient magnitudes and must stay unchanged. Internal subdivision
  reports expose interval/endpoint/variation evidence, not those magnitudes.
- primitive_bigint_ratio is explicitly a common POSITIVE projective scale:
  it clears positive denominators then removes positive integer content.
  Its factor relative to the original rationals need not itself be an integer
  (e.g. [2,4] -> [1,2]); only the output coefficients must be integers.
  By contrast the existing count-only fraction-free kernel scales controls
  by different falling factorials, so those outputs cannot feed de Casteljau.
- Candidate proof: for degree n, retain integer controls c=S*b with S>0.
  At midpoint level l, unhalved adjacent sums are 2^l times the ordinary
  tableau entries; shift each emitted boundary control by n-l bits. Every
  child then has the same positive factor 2^n*S. Sign variation and exact
  endpoint/midpoint zero witnesses are preserved without rational reduction
  at every addition. Use linear live storage, and avoid conversion on nodes
  that do not actually subdivide. Nonrational controls must retain the old
  Real/policy path and all unknown/depth/endpoint behavior.
- External experiment directory is /tmp/hyper-bernstein-scale.xDkXjE.
  No production Bernstein change is made or accepted at this review point.
- Isolated clones pin Hyperreal c2d147c, Hyperlattice d783af0,
  Hyperlimit 7bf9261 and Hypersolve a20be23. The first external Cargo build
  selected newer cached packages (including num-bigint 0.4.8 rather than
  workspace 0.4.6). Before any qualification/timing, seeded the driver lock
  from the actual Hypersolve lock and regenerated the small package graph;
  every shared package version now matches, including num-integer 0.1.46,
  num-rational 0.4.2 and serde_json 1.0.149. Lock SHA-256
  d8e06bcd7ad4934f49e0c916bca8afa04b8e012c39f8bef2916f4bddd1166c13.
  No old/new dependency mismatch is included in the claimed measurements.
- driver/src/main.rs qualifies 768 control vectors and 3,072 chained
  subdivisions. Degrees 0--12 plus 16/32/64, eight seeds, six input
  families: integer, rational, dyadic, wide, sparse and alternating signs.
  Independent closed-form binomial sums check EVERY left/right coefficient,
  the common positive factor and exact midpoint identity at each of four
  chained splits. Empty input and zero controls are included. Both the old
  Real recurrence and new integer recurrence pass. Evidence
  /tmp/hyper-bernstein-kernel-check.log. The oracle is not the new recurrence
  compared with a renamed copy of itself.
- Two initial 21-block rotating-input kernel measurements separate prepared
  integer work from first conversion cost: degree-2 rational controls,
  one split, median Real/warm/cold 543.26/220.86/528.61ns; degree 4
  1981.78/419.26/981.62ns. These are pilot kernel medians, not final
  public-path performance estimates. Degree-2 thirteen-call allocation
  demand changes 279/20,472 bytes -> 333/8,680 bytes including conversion:
  fewer bytes does not mean fewer allocation calls. Public timing is the
  retention gate. Raw /tmp/bernstein-kernel-{2,4}-rational-1.csv.
- The isolated candidate uses private BernsteinControls Real/Integer
  variants, converts only upon actual subdivision, and retains integer
  controls down the recursive branch. Public coefficient reports and
  terminal record shapes are unchanged. Its in-place adjacent sums use
  disjoint mutable slices and one work vector; no new unsafe production
  code or donor tableau/uncertainty machinery. Workspace Hypersolve remains
  completely unchanged at this stage.
- driver/src/public.rs supplies complete known root sets with multiplicity,
  not just sampled function values. It verifies interval partition coverage,
  root inclusion, endpoint uniqueness, exact roots, Empty/Isolating evidence,
  variation upper bounds and parity, and exact depth-limit widths. Sixteen
  configurations with 13 seeds each cover spread/clustered/repeated/endpoint
  roots, positive no-root polynomials, outside roots, and zero depth budget.
  All 208 original reports pass this independent oracle. All 208 candidate
  reports pass too and their canonical exact JSON is BYTE-IDENTICAL to the
  original, including terminal order and reason strings. Logs
  /tmp/bernstein-{baseline,candidate}-{degree}-{kind}-{depth}.jsonl.
  The clone's existing 31 root-isolation tests pass; log
  /tmp/bernstein-candidate-root-tests.log. New permanent and nonrational/
  unknown-path tests are still required before a production transfer.
- First complete public A/B run: 16 workloads, 21 alternating ABBA/BAAB
  process blocks, CPU 6, 13 rotating inputs, exact report oracle/setup and
  warmup outside timing, target 70ms whole-13-call batches capped at 13,000.
  1,344 timing records and 32 allocation records. Five thousand paired
  bootstrap resamples (seed 314159265). Raw /tmp/bernstein-public-ab.jsonl;
  scripts /tmp/hyper-bernstein-scale.xDkXjE/{measure,summarize}.mjs.
  Degree 4/8/16 spread ratios are .49451/.24895/.10239; degree 2/4/8
  cluster .35083/.19010/.08597; repeated 2/4 .38520/.24929; endpoint
  4/8 .54622/.25579. All corresponding 95% intervals lie below one.
  Degree-16 spread bytes for 13 calls fall 19,737,440 -> 1,886,488;
  degree-8 cluster 10,868,192 -> 1,148,920. These are cumulative requested
  bytes, not peak live storage. Detailed paired output is retained.
- Costs are not hidden: no-subdivision outside degree 1 and endpoint
  degree 2 regress about 3.8/4.0% with IDENTICAL allocation counts/bytes.
  A positive degree-2 polynomial finishes after one split and regresses
  7.43% [7.04,8.54%], allocations 476 -> 760 and bytes 51,616 -> 51,984
  for thirteen calls. Degree-2 spread is about even (.99220), depth-zero
  degree-4 cluster is statistically unresolved (1.00351 [.99616,1.01274]).
  The initial candidate adds 13,296 stripped-driver bytes (2,187,152 ->
  2,200,448). Performance/memory gains may justify size growth, but this
  is not yet a retained patch or a blanket improvement claim.
- Initial baseline/candidate binary SHA-256:
  3c22b9821f350dfd063ba85c490aa1cce4ef476a69fd517d5d39c636ce566013 /
  217e3483a4879acd812d46d20dbffb58b1ae64bb9d0d6a3d3bf4edf251eb0586.
  Public driver source hash
  5d85fc10ac71f7b658fd9c1118bdc378d13aac16a439721b756071dc1fa98b72.
- The next isolated trial delays conversion for a quadratic's FIRST split;
  if a child really needs another split it converts there and continues in
  integers. This targets the measured one-split conversion cost while
  retaining deeper-case gains. It is building now. No workspace Bernstein
  source has been edited, and the broad reference goal remains active.
- Delayed-conversion trial completes with all 208 public exact reports still
  byte-identical. The five small/deep-quadratic workloads were measured with
  the same 21 paired process blocks. Candidate/baseline ratios: outside
  degree 1 .99252 [.97918,.99979]; spread degree 2 1.01923 [1.00079,1.03162];
  cluster degree 2 .36965 [.36657,.37604]; endpoint degree 2 1.01306
  [1.00674,1.02279]; positive degree 2 1.02298 [1.01187,1.03252]. The
  positive quadratic now has exactly baseline allocation calls and bytes;
  spread uses 479/41,712 vs 405/43,824, cluster 4,451/316,576 vs
  6,657/620,312 (all thirteen calls). Raw /tmp/bernstein-delayed-ab.jsonl.
  No-work timings vary with code generation even though those paths never
  convert; do not attribute these changes to integer arithmetic.
- Additional isolated ownership trial: consume the controls during splitting
  and reuse the owned integer vector as the work row. This removes a full
  BigInt-vector clone and drops parent controls before recursive child work.
  Common-scale proof and output representation remain unchanged. This is
  being measured before selecting the final transfer; workspace source is
  still untouched.
- Ownership crossover measurements (same five cases and 21 paired blocks):
  consuming controls with immediate conversion gives spread-2 .94274,
  cluster-2 .31574, but positive-2 still regresses to 1.06145. Consuming
  controls WITH delayed quadratic conversion gives spread-2 .99076
  [.98564,.99857], cluster-2 .32805 [.32465,.33155], positive-2 .99887
  [.98798,1.00473], outside-1 1.00126 [.99053,1.02802], endpoint-2
  1.01790 [1.01271,1.03384]. The latter is selected for broad qualification:
  preserve the cheap one-split quadratic path, release parent storage before
  recursion, and consume the integer work row. Raw
  /tmp/bernstein-{owned,owned-delayed}-ab.jsonl. The original/delayed/owned
  alternatives remain only isolated artifacts, not extra production paths.
- Added five permanent tests to the isolated chosen source; all 36 root
  isolation tests pass (3.58s). New tests include 72 rational/integer control
  vectors and 216 chained splits against independent binomial-sum formulas,
  degrees 0--8 and 16/32/64, wide signed integers, sparse/zero/alternating
  data, positive common-factor checks and exact midpoint identity. A second
  independent public oracle checks 45 complete partitions from known root
  multisets, including multiplicity, variation/parity, endpoint uniqueness,
  exact depth-limit width and coverage. Additional tests preserve actual
  public coefficient magnitudes, pi-valued nonrational controls and interval
  endpoints, and undecided strict comparisons. Log
  /tmp/bernstein-owned-delayed-root-tests.log. Full gates/benchmarks pending.

### Shared-scale Bernstein: selected transfer qualification

- Selected owned-control / delayed-quadratic source reproduces all 208
  external baseline reports byte-for-byte. Copied only its reviewed
  root_isolation.rs changes and new benchmark coverage into a clean live
  Hypersolve worktree. Its source SHA-256 is
  25ef14f3f50bdbf713dd5c353a53da75f11aba90da5601d5574f4806c5476cbf,
  matching the isolated source exactly. Final executable SHA-256
  31fcdd9775d77e389eef47d047f8bc91eed11387ff94083cb51b4795e20afc62.
  Dependencies and original baseline remain pinned as above.
- Completed final 16-workload run: 1,344 timings plus 32 allocation records,
  21 alternating ABBA/BAAB process blocks per workload, CPU 6, thirteen
  rotating input expressions, exact outcome oracle/setup outside timing,
  5,000 paired-block bootstrap resamples. No audit compilation/tests ran
  concurrently with these timing blocks. Raw /tmp/bernstein-final-ab.jsonl;
  complete numerical summary /tmp/bernstein-final-summary.jsonl.
  Ratios below are candidate/baseline; lower is better. Bytes are cumulative
  allocator requests, not peak storage.

| Degree / roots / depth | Paired time ratio [95% CI] | Allocations, old -> new (13 calls) | Requested bytes, old -> new (13 calls) |
| --- | --- | --- | --- |
| 1 / outside / 8 | 0.98599 [0.97305, 0.99598] | 149 -> 149 | 20,208 -> 20,208 |
| 2 / spread / 8 | 1.01845 [1.01395, 1.02542] | 405 -> 460 | 43,824 -> 41,120 |
| 4 / spread / 8 | 0.48260 [0.47403, 0.48994] | 3,986 -> 1,701 | 322,408 -> 115,064 |
| 8 / spread / 16 | 0.23434 [0.23214, 0.23452] | 27,248 -> 5,465 | 2,092,304 -> 372,520 |
| 16 / spread / 16 | 0.09992 [0.09892, 0.10024] | 383,911 -> 29,724 | 19,737,440 -> 1,701,984 |
| 2 / cluster / 32 | 0.32784 [0.32504, 0.33056] | 6,657 -> 3,515 | 620,312 -> 288,496 |
| 4 / cluster / 32 | 0.16601 [0.16494, 0.16754] | 21,100 -> 5,589 | 1,710,616 -> 420,784 |
| 8 / cluster / 32 | 0.07990 [0.07934, 0.08042] | 261,623 -> 18,717 | 10,868,192 -> 984,112 |
| 2 / repeated / 16 | 0.35347 [0.35120, 0.35478] | 5,299 -> 2,969 | 528,032 -> 269,624 |
| 4 / repeated / 16 | 0.22354 [0.22280, 0.22427] | 16,041 -> 4,396 | 1,335,000 -> 345,832 |
| 2 / endpoint / 8 | 1.05042 [1.02810, 1.06055] | 195 -> 195 | 26,208 -> 26,208 |
| 4 / endpoint / 8 | 0.53858 [0.53751, 0.54490] | 2,785 -> 1,248 | 253,704 -> 101,776 |
| 8 / endpoint / 16 | 0.24405 [0.24268, 0.24509] | 26,127 -> 4,883 | 2,029,600 -> 359,136 |
| 2 / positive / 24 | 1.01451 [1.00690, 1.01900] | 476 -> 476 | 51,616 -> 51,616 |
| 4 / positive / 24 | 0.87042 [0.86207, 0.88709] | 2,086 -> 1,896 | 169,304 -> 128,400 |
| 4 / cluster / 0 | 1.00299 [0.98297, 1.00594] | 814 -> 814 | 79,840 -> 79,840 |

- These results supersede pilot figures, including the small-case costs:
  spread-2 +1.84%, endpoint-only-2 +5.04%, positive-2 +1.45%; depth-zero
  remains unresolved. No claim of uniform speedup is made. Substantive
  subdivision workloads improve about 2--12.5x, including depth-budget
  reports for repeated roots. Ownership and common-scale arithmetic lower
  allocation demand on those workloads; small spread-2 uses more allocation
  calls despite fewer bytes.
- Stripped external public driver grows 12,536 bytes
  (2,187,152 -> 2,199,688, about 0.57%). ELF text/data/BSS are
  1,957,291/225,840/1,640 -> 1,969,715/225,928/5,488 bytes. This is a
  measured linked-driver size cost, not a smaller-binary claim.
- Separate whole-process Massif runs for thirteen degree-16 spread inputs
  (including fixture setup, independent oracle and thirteen measured API
  calls) show peak live requested heap 1,455,325 -> 1,384,866 bytes;
  allocator overhead 228,387 -> 219,078 bytes. These modest whole-process
  peak reductions must not be confused with the 91% reduction in cumulative
  API allocation demand. Profiles /tmp/bernstein-massif-{baseline,final}16.out.
- All-feature live Hypersolve debug and release tests pass: 421 unit + 9
  exactcore_root_families + 2 readme + 4 real_representations + 131
  residual_props + 211 smoke tests = 778 in each build, plus zero doc tests. Logs
  /tmp/hypersolve-bernstein-{debug,release}.log. Clippy all-target/all-feature
  -D warnings also passes. Documentation, benchmark execution and remaining
  memory checks are being completed before the retention commit.
- All 16 new Criterion fixtures execute with their independent setup outcome
  assertions, 40 samples, 0.2s warmup and 0.5s measurement on CPU 6. The
  existing sixteen-row (x-1)^2 fixture also completes with exactly the root
  1 in each report. Its comparison against an OLD cached Criterion baseline
  reports about +8%; this is being investigated with a fresh source-pinned
  batch A/B instead of treating cached history as a controlled comparison.
  Logs /tmp/hypersolve-bernstein-criterion{,-existing}.log. The benchmark
  runner regenerated benchmarks.md and also pulled in four stale prior-run
  Krawczyk rows; restored only that generated side effect with apply_patch.
  The root audit ledger and raw paired measurements remain authoritative.
- The two large public Memcheck runs have zero errors and zero definite,
  indirect or possible leaks; both exact report files remain byte-identical
  to baseline. All 36 root-isolation unit tests also pass under Memcheck
  with zero definite/indirect leaks and zero access errors. A separately
  displayed 48-byte possible leak originates in the Rust test runner's
  std::thread::Thread / std::sync::mpmc::context TLS initialization, even for
  a single coefficient-report test, not the new integer subdivision path.
  Logs /tmp/bernstein-memcheck-{spread16,cluster8,unit,runtime-context}.log.
- Concurrent user work advances live Hypersolve to 0b00cb20083f7d4525c401e236328e03e13efcdb,
  an unrelated curve_resultant test change; it is preserved and not credited
  to this audit. The fresh batch baseline clone initially inherited this
  new tip, detected BEFORE any batch timing, and is explicitly re-pinned to
  a20be23 to match the isolated selected candidate. Earlier sixteen-workload
  binaries remain immutable and correctly matched to a20be23.
- Fresh source-pinned original-fixture A/B completes: the exact sixteen-row
  public reports match byte-for-byte, and 21 CPU-6 ABBA/BAAB blocks give
  candidate/baseline 1.00342 [0.99526,1.00709], medians 53.84/53.98us.
  The cached Criterion +8% is NOT reproduced by this controlled experiment;
  it is not evidence of an 8% source-change regression. There is still an
  allocation cost: thirteen sixteen-row calls use 4,957 -> 8,224 allocation
  calls and 727,536 -> 738,744 requested bytes (+1.54%). Record this short
  dyadic-root workload cost alongside the large-workload wins, not just time.
  Raw /tmp/bernstein-batch-ab.jsonl. Driver driver/src/batch.rs mirrors the
  existing fixture exactly, checks partition/endpoint evidence before timing,
  and uses the same pinned lock hash. Binary SHA-256 baseline/final:
  3183e0b2600aa11fe95e037a1a1a26b23a00bb6ad2bac2ef6c799f4af192cbad /
  e1ad46125f4bd47b5127358889bfcf56a69185256b588a9a991acaf11200eca3.
- Retention decision: the proof-preserving private control representation is
  worthwhile for multi-split/higher-degree subdivision (about 2--12.5x and
  up to 91% fewer requested bytes), despite documented 1--5% small-case time
  costs, the short dyadic batch's allocation cost and 12.5KB driver growth.
  It does not broaden exact coefficient admission, guess an unknown sign,
  change public coefficient magnitudes, adopt donor defects, or introduce
  unsafe production code. The 83-net-line production change is accompanied
  by 339 test lines and 135 benchmark lines; no dependency is added.
- fmt --check, all-target/all-feature Clippy -D warnings, documentation with
  warnings denied, both 778-test build configurations, all 17 permanent
  subdivision benchmark fixtures and memory checks are complete. No other
  production file was edited. A scoped commit and final current-state
  verification follow; the overall reference audit is still active.
- Retained commit: Hypersolve
  4dfbb39948a2f6e41438e6430e451fc8e97fb755
  (Keep rational Bernstein subdivision in common-scale integer controls).
  Only src/root_isolation.rs and benches/certification.rs are committed;
  the worktree is clean. No push was performed.
- Final dependency reconciliation: live Hyperreal is now clean at concurrent
  user commit 90797d766d8a92523996c98728329dbb423f1051, retaining successful
  exact normal-form sign proofs in the native fact cache. Read its complete
  five-file diff and preserved it without attribution to the audit. Live
  Hypersolve also preserves user commit 0b00cb2 below the audit commit.
  Both post-commit 778-test configurations pass and Cargo reports no
  recompilation (0.06s each), confirming that the already-qualified live
  artifacts include this current source state. Logs
  /tmp/hypersolve-bernstein-post-commit-{debug,release}.log. Controlled A/B
  timing/size remains explicitly pinned to Hyperreal c2d147c on BOTH sides,
  not a mixed-dependency comparison.
- AERN2 handwritten-source and targeted-transfer comparison checkpoint is
  now closed with the stated generated-artifact limitations, three retained
  Hypersolve ideas, and all rejected/deferred ideas still recorded. No new
  donor source lines are credited by this transfer follow-up. The broad
  inventory audit is NOT complete and the goal remains ACTIVE; this turn
  is progress, not blocked. Next source cursor: pin and inventory original
  AERN (michalkonecny/aern), then audit its files/lines; CDAR and mBound
  follow in the existing queue. Earlier cross-reference candidates remain
  pending where explicitly marked, including RealLib cache/scheduling and
  Boehm prototype cold-storage comparisons.
- All audit-started compilers, tests, benchmarks and memory-check processes
  have exited at this checkpoint. No subagents were used.

### Original AERN source audit opened

- Previous goal turn is progress: retained Hypersolve 4dfbb39 with exact
  report equivalence, independent oracles, complete gates and controlled
  timing/allocation/size evidence. Current Hypersolve and Hyperreal worktrees
  are clean. Broad scope remains unchanged; no subagents used.
- Cloned official https://github.com/michalkonecny/aern without modification
  at 3a45d80cfa2197ddb4935b96509348331c8b763d (2015-12-02). All 288 tracked
  paths are inventoried with mode, byte count, physical line count and
  SHA-256 in exact-real-references/AERN_FILE_INVENTORY.tsv. There are 281
  regular UTF-8 text files / 58,894 lines, two symlinks, five binary assets,
  no gitlinks, and 2,190,342 total tracked bytes including symlink targets.
  Inventory is not source-read credit. The scripts/metadata distinction and
  generated-artifact exclusions follow the root ledger's existing convention.
- Read all seven root text files: README.md (3), LICENSE (27), .hgtags (2),
  install-all.sh (14), install-nogtk-nompfr.sh (9), install-nogtk.sh (9),
  install-nompfr.sh (11): 75 lines. The two no-GTK scripts are identical and
  BOTH install aern-mpfr, despite the no-MPFR one's name. Installation uses
  force-reinstalls into the caller's Cabal environment; these scripts are
  being audited, not executed against the user's environment.
- Reviewed the official project and installation pages. They describe
  Kaucher arithmetic, polynomial function enclosures and separate effort
  dimensions. The published logistic-map example reportedly spent over two
  of three hours on unsuccessful lower-precision retries; this is a claim
  to verify against code and controlled workloads, not an accepted benchmark.
  Installation targets GHC 7.6.3 and says the older MPFR binding needs a
  modified GHC. The linked ICMS2014 paper fetch timed out; it is not read yet.
  Sources: https://michalkonecny.github.io/aern/_site/index.html and
  https://michalkonecny.github.io/aern/_site/pages/installation-from-source.html.
- GHC/Cabal are not on PATH, as in the earlier AERN2 audit. Reuse of that
  isolated Stack toolchain is being checked; no build/test success is claimed
  and this does not block continued source review.
- Read all thirteen package Cabal manifests (1,213 lines): aern-order (103),
  aern-real (79), aern-interval (71), aern-double (137), aern-mpfr (109),
  aern-mpfr-rounded (92), aern-fnreps (49), aern-realfn (52), aern-poly (90),
  aern-realfn-plot-gtk (49), aern-poly-plot-gtk (131), aern-ivp (138),
  aern-temp (113). The order layer includes test/benchmark dependencies in
  the library, real defines rounded arithmetic classes, interval supplies
  in/out Kaucher operations, and concrete endpoints live in Double or either
  of two alternative MPFR packages exporting the same module names. Test
  programs are executables, not Cabal test suites. A successful stack test
  alone would not demonstrate those tests ran. Later function, polynomial,
  plotting and IVP packages have progressively broader dependencies.
- Read aern-order/src/Numeric/AERN/Basics/{Effort (225), SizeLimits (70),
  Consistency (104), Exception (107), PartialOrdering (385), Bench (120),
  Arbitrary (144)}.hs; NumericOrder/PartialComparison.hs (280),
  RefinementOrder/PartialComparison.hs (259); Misc/{List (116), Bool (19),
  Maybe (58), QuickCheck (100), Debug (49)}.hs; and all six Basics/Laws
  modules: Operation (70), OperationRelation (111), PartialRelation (46),
  Relation (38), RoundedOperation (180), Utilities (50). Together with
  root metadata and manifests this is 40 fully read files / 3,819 lines
  (recomputed from the per-file inventory, correcting a 54-line arithmetic
  overcount in the initial intermediate note).
- Architecture/validation findings: numeric and refinement order are
  separate, known incomparability is distinct from unknown, and intrinsic
  size/precision limits are separate from operation effort. Tuple effort
  sequences hold exhausted coordinates, not a Cartesian fairness schedule.
  Int1ToN names constrain random generation, NOT runtime increment bounds.
  Several partial laws accept unknown premises/results; this is weak
  soundness testing, not evidence of completeness or achieved accuracy.
  Numeric comparison's reflexivity property is defined but not registered
  in its test list. Criterion uses deep result forcing, but its geometric
  input-size and effort sweeps need controlled fixed-workload comparisons
  before any precision/performance conclusion. Potential partial-info and
  generator information losses remain hypotheses pending direct probes.
- Reused isolated Stack 3.7.1 / GHC 9.6.7 with lts-22.44 under
  /tmp/aern2-stack.ryVCFK; trial clone and stack.yaml are in
  /tmp/aern-audit.zp9Mzz. Untouched aern-order:lib build fails at twelve
  failable-list do patterns in the numeric/refinement Arbitrary modules
  because current QuickCheck Gen has no MonadFail instance. Dependencies
  downloaded with explicit network approval; no global force-install
  scripts executed. Log /tmp/aern-order-untouched-build.log. The build
  process has exited; this is a compatibility failure, not a numerical test.
- Independent /tmp/aern-audit.zp9Mzz/ExceptionProbe.hs compiles the unchanged
  donor Exception module and confirms direct scalar exceptions are caught,
  while an exception inside a lazy list escapes when subsequently forced:
  the wrapper returns Right and raisesAERNException reports False. Its
  evaluate is WHNF-only. Treat this as a boundary limitation, not as a
  blanket claim that the helper promises deep capture. Compilation needed
  explicit escalation for the configured ccache path; all objects/binary
  are in the owned /tmp trial directory. No Hyper code changed this slice.
- Completed every aern-order file: remaining NumericOrder/{Arbitrary (331),
  Arbitrary/Linear (419), RoundedLattice (250), RefinementRoundedLattice
  (317), Extrema (33), Operators (19)}.hs; RefinementOrder/{Arbitrary (352),
  RoundedLattice (341), RoundedBasis (174), IntervalLike (89), Extrema (40),
  Operators (22)}.hs; NumericOrder.hs (33), RefinementOrder.hs (34),
  Basics/ShowInternals.hs (26), LICENCE (30), Setup.lhs (3), and
  tools/BenchCsvToGnuplot.hs (207). Package total 39 files / 5,354 lines;
  repository cumulative 140 files / 45,670 lines. Machine-checkable completed
  ranges and content hashes are in exact-real-references/AERN_READ_COVERAGE.tsv.
  The dormant CSV plotter is not executed and contributes no scalar transfer.
- Independent OrderProbe.hs exercises unchanged reference source: all 729
  six-field ternary knowledge records, of which 223 are consistent. There
  are NO dropped possible relations, but 40 records retain alternatives
  already ruled out (eg GEQ=true, GT=false still yields EQ or GT). This is
  conservative information loss, not a false proof. Full-order roundtrips,
  all 29 triples witnessed by subsets of a three-element set, all 16
  transitivity-consequence pairs, all 355 four-element preorders (independent
  reflexivity/transitivity matrix oracle), and 192 forward/reverse constrained
  cases pass. Partial-info conjunction discards False AND Unknown although
  the same package's Misc.Maybe helper correctly retains False.
  Driver /tmp/aern-audit.zp9Mzz/OrderProbe.hs; log /tmp/aern-order-oracle.log.
  Exception probe output is now also in /tmp/aern-exception-probe.log.
- Further source findings being qualified: AreaLinear merges lower bounds
  with min and upper bounds with max, while its strict flags drive endpoint
  successor/predecessor choices in the opposite direction. Whole-area
  forbidden values are not filtered from its special-value sampling arm.
  Ordered-tuple generators assume 3*n random draws yield n distinct values
  and silently truncate otherwise. The refinement-rounded numeric-lattice
  test named distributive actually invokes roundedModular twice, so it
  cannot establish full distributivity. None of these are adopted by Hyper.
- Hyper comparison: re-read hyperlimit/src/predicate.rs completely and
  predicates/order.rs lines 1--240. It already has explicit Unknown,
  certainty/stage provenance, strong Boolean decisive-operand handling and
  rational/certified-Real ordering paths. No six-field partial-order object
  is justified for its scalar total-order domain from this evidence alone.
  Existing Boolean-combinator regression is being rerun. This is targeted
  transfer comparison, not new donor read credit.
- For further generator validation ONLY, twelve failable list-bind patterns
  in the two already-read Arbitrary modules are made irrefutable in the
  /tmp trial clone. This is modern-GHC source compatibility on valid tuple
  outputs, not an algorithmic repair or production transfer; malformed-tuple
  failure forcing can differ. The pinned reference remains unmodified.
  Adapted build log /tmp/aern-order-compat-build.log; the complete 35-module
  package now builds successfully. The targeted Hyperlimit Boolean test also
  passes (1 test, 241 filtered), /tmp/aern-transfer-hyperlimit-booleans.log.
- GeneratorProbe.hs confirms, over 4,096 fixed seeds, 1,028 forbidden-zero
  samples from an area forbidding its sole special value zero, and 129
  truncated two-element LT tuples from the feasible value set {0,1}.
  Exact endpoint chooser probes also confirm lower/upper bound merging
  widens rather than intersects the constraints, and strict/inclusive
  integer endpoints are reversed. Well-shaped generated tuples in this
  probe satisfy LT. Log /tmp/aern-generator-probe.log. These are donor
  generator defects, not Hyper defects or arithmetic accuracy measurements.
- Read aern-real/concepts/BinaryOpsSimplified.hs (226), src/Numeric/AERN/
  Misc/IntegerArithmetic.hs (62), and RealArithmetic/{ExactOps (115),
  Measures (293), Auxiliary (46), Bench (91), Laws (523)}.hs; both aggregate
  NumericOrderRounding.hs (99) / RefinementOrderRounding.hs (139); their
  Operators (27/30), Conversion (114/130) and Elementary (201/308) modules;
  NumericOrderRounding/ElementaryFromFieldOps/Sqrt.hs (432, including the
  entire commented mutable implementation), and RefinementOrderRounding/
  ElementaryFromFieldOps/{Exponentiation (208), SineCosine (427)}.hs.
  Cumulative coverage is 76 files / 10,010 lines, with unique ranges/hashes
  in AERN_READ_COVERAGE.tsv. Nine aern-real files and the other package
  implementations remain unread. Source cursor is paused here for the
  exactness transfer investigation below, not marked complete.
- The aern-real build requires one modern-Prelude compatibility import,
  hiding Applicative (<*>) in its arithmetic-operator re-export. With that
  isolated adjustment it builds all 25 modules; no donor numerical code is
  altered. Logs /tmp/aern-real-{build,compat-build}.log. All reference
  worktrees remain pristine; trial changes are restricted to three files.
- Measures separates imprecision from effort, but ScheduleProbe.hs confirms
  the simple iterator skips its initial effort and tries nothing for unit
  effort, even when the initial result already meets the goal. The second
  iterator evaluates its initial point, but can accept worse imprecision
  because it selects among successors without comparing to the incumbent.
  Its limit counts selected stages, not all candidate evaluations. Exact
  singleton imprecision probe log /tmp/aern-schedule-probe.log. Do not
  transplant this scheduler as a guaranteed-progress or strict-budget one.
- Numerical ideas in this slice: generic reciprocal-sqrt Newton with a
  conservative fast-region threshold and division fallback; Horner Taylor
  exp on an explicitly bounded argument plus power reconstruction; trig
  range reduction and endpoint monotonicity with [-1,1] fallback; coefficient
  and result types can differ. Hyper already uses demand-sized integer
  sqrt seeds/Newton and exponential Taylor with ln(2) reduction. Re-read
  hyperreal's exp_sqrt.rs lines 1--190, exp_trig.rs 1--200 and the relevant
  dispatch, bounds and approximation-query contracts for comparison.
- Not-yet-qualified donor concerns: sqrt positivity relies on conversion
  to Double (potential tiny-positive admission loss); trig cosine endpoint
  direction helpers appear reversed, but earlier range branches may make
  them unreachable for ordinary interval instances. Do NOT report a
  demonstrated trig enclosure failure without a reachable counterexample.
  Taylor coefficient integer products and exp scale abs(Int) also merit
  boundary tests. The elementary sine/cosine property skeleton is entirely
  commented out. Laws checks only the first five scheduled outcomes and
  accepts unresolved comparisons/domain exceptions; no MPFR accuracy
  validation of donor kernels is claimed from a successful build.

### AERN-triggered Hyper exponential exactness investigation (in progress)

- Directed-MPFR public-API sweep on clean Hyperreal
  90797d766d8a92523996c98728329dbb423f1051 finds 630 one-unit accuracy
  violations in 4,068 fresh exp approximations (rational inputs in [-8,8],
  denominators 2/3/7/16, p=4,1,0,-1,-8,-32,-64,-128,-256). Example:
  exp(3/2).approx(1) returns zero, 2.240845 target units from the true value.
  Driver /tmp/aern-hyper-exp.PdU63J/src/main.rs, baseline log
  /tmp/aern-hyper-exp-probe.log. MPFR uses 2,048-bit outward-rounded input
  conversion and exponential enclosures, not comparison with nearest-only
  binary64 or another Hyper expression. Baseline run exited 1 as expected.
- Root cause: the constructor labels planning MSD <= 2 as small enough
  for a Taylor kernel requiring roughly |x| <= 1/2. This admits values near
  eight, while the kernel returns zero for p>=1 and budgets child error as
  if the reduced-domain derivative were small. Moreover bounded ln(2)
  correction can defer an unreduced operand to the same kernel. This is an
  exactness issue, prioritized ahead of continuing donor source reads.
- Isolated candidate clone /tmp/aern-hyper-exp.PdU63J/hyperreal-candidate
  tightens the constructor bypass to an EXACT MSD <= -2, not a planning
  estimate, and verifies the small-domain bound inside the kernel. Unreduced
  operands use binary argument scaling and certified square reconstruction.
  The original 4,068-case sweep now has ZERO violations; log
  /tmp/aern-hyper-exp-candidate-probe.log. This is an initial candidate,
  NOT a retained change: raw/deferred-node tests, larger/composite inputs,
  cache order, complete regression gates, controlled benchmarks, allocation
  and size checks remain required. Live Hyperreal source is unchanged.
- The previous goal checkpoint is progress, not blocked. Additional donor
  reads: aern-real NumericOrderRounding/SpecialConst.hs (47) and
  RefinementOrderRounding/SpecialConst.hs (53), both interface/default-effort
  wrappers. Coverage is now 78 unique files / 10,110 lines. The per-file
  coverage manifest has been checked for duplicate paths (none). Seven
  aern-real files remain unread; the scalar audit is still paused for this
  higher-priority correctness issue.
- Candidate tests now independently cover rational inputs, raw PrescaledExp
  nodes, both cold and coarse-to-fine/fine-to-coarse cache orders, shared
  pi/sqrt2 and inexact-bound composites, and inputs through +/-10,000.
  The large-input oracle uses 16,384 bits so its enclosure remains narrow
  at ABSOLUTE 256-bit accuracy after e^10000. The first draft mistakenly
  parsed Hyper's mixed-fraction Display string as a GMP rational; corrected
  the test to import signed numerator and denominator separately before
  accepting results. Both final directed tests pass, and applying these
  tests alone to the unchanged baseline fails at exp(4/3), p=1 as expected.
  Logs /tmp/aern-hyper-exp-{directed-tests,composite-tests,baseline-regression}.log.
- Initial eager-reduction candidate passes 788 all-feature tests/examples
  in each of debug/release, 24 doctests, all-target/all-feature Clippy with
  warnings denied, fmt --check and diff --check. Memcheck on a fresh e^10000
  approximation reports zero errors and zero definite/indirect/possible
  leaks; remaining 4,160 bytes are reachable shared caches. Logs
  /tmp/aern-hyper-exp-{all-debug,all-release,doctests,clippy,memcheck}.log.
- Controlled benchmark harness bench.rs and paired-bench.mjs live under
  /tmp/aern-hyper-exp.PdU63J. Both source baselines are pinned to 90797d7;
  both arithmetic dependency graphs explicitly use native-lock num-bigint
  0.4.6. Standalone benchmark lock files have the identical SHA-256
  168ac13b15fbc5a2c28f6fc9f322595a7b250e2c090e8c59f6daa44c9cbc4178.
  The earlier public-API probe used 0.4.8; the native-lock tests above
  independently reproduce the bug and qualify the fix with 0.4.6.
- Initial candidate timing: thirteen workloads, fifteen CPU-6 ABBA/BAAB
  blocks each, approximately 50ms per arm (cached runs capped at two million
  calls), fixed common-e/ln2 cache warmup. Raw
  /tmp/aern-hyper-exp-ab.jsonl; bootstrap summaries
  /tmp/aern-hyper-exp-ab-summary.jsonl. Small/half arguments cost about
  +3--4%; several non-small full approximations are faster (7/2 ~0.784x,
  15/2 ~0.709x time), but constructor-only 7/2 costs ~23.1x. The coarse
  repair costs ~20x relative to the INVALID old zero result, which must not
  be presented as an equal-accuracy comparison. Allocation demand grows on
  affected full approximations (eg 7/2: 66->93 calls, 2,800->3,360 bytes).
  Benchmark executable grows 1,040 bytes; text+data grows 984 bytes while
  reported bss shrinks 984. Preserve these tradeoffs, not just selected wins.
- The eager constructor cost motivates a second isolated variant,
  /tmp/aern-hyper-exp.PdU63J/hyperreal-demand. It keeps the old cheap
  expression-construction choice but explicitly treats it as scheduling,
  never a range proof. The kernel checks the actual rough enclosure and
  requests factored ln(2) reduction only on demand; if the bounded correction
  fails, binary scaling/square reconstruction remains the certified fallback.
  All nineteen exp-related release tests including the new directed suites
  pass. Its same-lock thirteen-workload A/B has started, log
  /tmp/aern-hyper-exp-demand-ab.jsonl. This is not yet a retained change;
  live Hyperreal remains clean at 90797d7 and neither variant is committed.
- Selected third variant: /tmp/aern-hyper-exp.PdU63J/hyperreal-demand-cached
  keeps demand-time reduction and uses retained EXACT cached MSD <= -2 to
  bypass the numeric guard. Inexact planning magnitudes do not qualify.
  This restores the small-input allocation count without expression walks
  or eager approximation. The first eager variant and second uncached-guard
  variant are rejected in favor of this one; their evidence is retained.
- Selected variant passes 790 all-feature tests/examples in EACH of debug
  and release, 24 doctests, all-target/all-feature Clippy -D warnings,
  fmt --check, diff --check, and the e^10000 Memcheck driver (zero errors,
  zero definite/indirect/possible leaks; 4,160 bytes reachable shared caches).
  Logs /tmp/aern-hyper-exp-final-{all-debug,all-release,doctests,clippy,memcheck}.log.
  Four permanent tests cover directed MPFR, raw/deferred nodes, cache order,
  composite inputs, cancellation without cache poisoning, and rejection of
  inexact small-magnitude planning facts. Three permanent microbenchmarks
  cover deferred construction, correct coarse exp(3/2), and exp(-10000).
  Their Criterion run succeeds; /tmp/aern-hyper-exp-final-criterion.log.
- Selected same-lock paired results: /tmp/aern-hyper-exp-demand-cached-ab.jsonl
  and /tmp/aern-hyper-exp-demand-cached-ab-summary.jsonl. Fifteen CPU-6
  ABBA/BAAB blocks per workload, paired median candidate/baseline ratios,
  10,000-resample bootstrap 95% intervals: small 1/4 0.9915 [0.9879,0.9987];
  half 1.0249 [1.0222,1.0540]; 7/5 1.0225 [1.0036,1.0623];
  7/2 0.8576 [0.8068,0.8731]; 15/2 0.7728 [0.7690,0.7764];
  -7/2 0.8277 [0.8166,0.8482]; 2 0.9736 [0.9285,0.9951];
  128 1.0203 [0.9858,1.0400]; 257 1.0142 [1.0038,1.0285];
  -32 1.0159 [1.0004,1.0275]; cached 1.0023 [0.9807,1.0128];
  constructor 0.9972 [0.9612,1.0123]. Host drift makes these paired ratios,
  not ratios of separately pooled absolute medians, the comparison metric.
  Do not overstate the tiny small-input differences as a substantive win.
- Directed MPFR independently confirms all ten full-approximation benchmark
  inputs at p=-128 meet the one-unit contract on BOTH baseline and final
  code, with identical observed errors. Thus those timing comparisons are
  equal-accuracy. The coarse case is different: exp(3/2), p=1 changes from
  INVALID error 2.24084454 to valid error 0.24084454 target units; its 25.42x
  time ratio is the cost of repairing an invalid shortcut, not an accuracy-
  equivalent regression. Logs /tmp/aern-hyper-exp-bench-oracle-{baseline,final}.log.
- Allocation traffic (not peak RAM): small 1/4 unchanged 37 calls/1,488 B;
  half 41/1,672 -> 44/1,696; 7/5 51/2,112 -> 98/3,464;
  7/2 66/2,800 -> 96/3,512; 15/2 84/3,704 -> 105/4,016;
  -7/2 66/2,792 -> 100/3,472. Integer 2/128 unchanged; 257/-32 each
  add one 8-byte allocation. Same-harness executable +616 bytes (1,531,552
  -> 1,532,168); text +524 B, data unchanged. Bss layout shrinks 528 B,
  not claimed as an intentional memory-use optimization. No new dependency
  or unsafe production code. Retention rationale is correctness first,
  with preserved cheap construction/cached queries and explicit tradeoffs.
- Ported ONLY the selected four source/test/bench files to live Hyperreal,
  preserving concurrent base commit 90797d7 and existing benchmarks.md.
  The trial's generated three-row benchmarks.md is NOT copied over the
  live measurement history. Live debug/release tests also pass 790 each;
  downstream release library tests pass: Hyperlattice 19, Hyperlimit 242,
  Hypertri 3. Live doctests 24 and fmt --check pass. Logs use the
  /tmp/aern-hyper-exp-live-* and /tmp/aern-hyper-exp-hyper*.log prefixes.
  Hypersolve/Hypercurve downstream and live Clippy checks are running;
  no commit or push yet. Donor coverage remains 78 files / 10,110 lines.
- Final source review also flags Expm1's unconditional p>=1 zero return
  before its range guard as a separate exactness candidate. This is NOT
  yet an independently qualified finding or part of the retained exp fix;
  check through the public API before resuming donor reads.
- Selected live exp also passes the original standalone 4,068-case public
  sweep with ZERO violations, /tmp/aern-hyper-exp-live-probe.log. Live
  all-target/all-feature Clippy -D warnings passes. Hypersolve release
  all-feature tests/examples pass 783 tests at its current source; its
  worktree is clean. Hypercurve downstream checks are advancing through
  library/integration suites with no failure so far; not yet terminal.
- Expm1 candidate independently CONFIRMED: 342 one-unit violations in the
  4,068-case outward-MPFR sweep on the selected exp-fixed live code. Example
  expm1(3/2).approx(1) returns zero, error 1.74084454 units. Log
  /tmp/aern-hyper-expm1-live-probe.log (expected exit 1). This is specifically
  its early coarse-zero branch, not a remaining exp failure. Isolated
  follow-up /tmp/aern-hyper-exp.PdU63J/hyperreal-expm1 moves the shortcut
  below the existing argument guard, preserving the fine-precision path.
  Direct MPFR expm1 regression covers cold and both cache orders, rationals,
  range boundaries, +/-10,000 and +/-2^-512. Qualification in progress;
  this follow-up has NOT been applied live. Original AERN cursor unchanged.
- RETAINED exp fix as Hyperreal da66ba3, "Certify deferred exponential
  kernel arguments before approximation" (four intended files only).
  Hypercurve downstream is now terminal: 44 suites, 1,702 passed, 4 explicitly
  ignored, no failures, against Hypercurve 8fa284ed76ee6c703474b65aacbeae74a5e4d896.
  All five named downstream crates pass. Live source/test/bench SHA-256s
  match the selected qualified trial byte-for-byte. No push performed.
  Expm1 follow-up remains isolated; same-lock CPU-pinned paired comparisons
  are being built against the exp-fixed baseline, not the inaccurate old exp.
- First isolated Expm1 correction passes the independent 4,068-case sweep
  with zero violations and its native-lock directed MPFR regression. Logs
  /tmp/aern-hyper-expm1-{candidate-probe,directed-test}.log. Twelve-workload,
  fifteen-block CPU-6 ABBA/BAAB comparison completed; raw/summary files
  /tmp/aern-hyper-expm1-ab{,-summary}.jsonl. Most fine/cached/construction
  cases remain near baseline, allocation traffic unchanged for all fine
  cases; e^10000-1 fine query median ratio 1.0395 [1.0231,1.0882]. Coarse
  positive repair costs 32.32x relative to invalid zero; valid coarse-small
  costs 1.844x, valid coarse -32 costs 28.41x. The latter is avoidable and
  motivates a second variant, not blind retention of a large regression.
  Initial benchmark executable 1,533,160 vs 1,533,384 baseline (-224 B),
  text -168 B; bss layout +160 B. No allocation/size win is claimed yet.
- Second Expm1 candidate uses the EXISTING immediate_sign root-only exact
  certificate to preserve zero for coarse nonpositive operands, and uses
  the already-required rough upper bound without imposing a lower bound:
  x < 9/16 implies -1 < expm1(x) < 1. Unknown signs still fall through to
  numerical validation. This avoids full exp evaluation for large negative
  inputs without adding a graph walk or treating unknown as false. Original
  variant benchmark binary remains in expm1-bench-candidate; new binary is
  expm1-bench-sign. Matching lock hash for the initial two benchmark graphs:
  9e648ff5d9031d99356c2d3201f1795c26c01da7d0a09c36049e55666e836417.
  Second-variant full debug/release, Clippy and benchmark build are running.
- While the Expm1 benchmark runs, resumed bounded donor reading: aern-real
  LICENCE (30), Setup.hs (2), NumericOrderRounding/FieldOps.hs (734), all
  lines including comments/disabled properties. Coverage is 81 unique files
  / 10,876 lines. FieldOps keeps per-operation effort distinct from ring/
  field effort projections; directed integer powers switch rounding on
  negative odd powers and conservatively combine alternatives when sign
  is unknown. This agrees with Hyper's proof-first sign dispatch; no new
  production transfer selected. Nonnegative exponent preconditions are
  documented rather than encoded in Int; Hyper's bounded/u32 paths already
  make that boundary more explicit. Rounded law comparisons are enclosure
  compatibility checks, not exact identity/completeness or independent
  accuracy oracles. Next file MixedFieldOps.hs has only lines 1--150 read,
  so it is NOT yet credited as a complete file.
- NumericOrderRounding/MixedFieldOps.hs now fully read (510). Coverage
  82 files / 11,386 lines. Conversion-based mixed arithmetic preserves
  outward conversion bounds and combines both endpoints when multiplier
  sign is unknown. Two source-level concerns require independent probes:
  mixed multiply/divide default helpers project addDefaultEffort rather
  than their own operation's effort; mixedDivUp's result combines with
  maxDnEff rather than maxUpEff. The latter need not fail for types whose
  min/max select an exactly representable operand, so do not claim an
  observed scalar enclosure failure without an applicable instance/probe.
- Expm1 second variant passes 792 all-feature tests/examples in EACH of
  debug/release, 24 doctests, all-target/all-feature Clippy -D warnings,
  fmt --check and diff --check; 4,068 public directed-MPFR cases have zero
  violations. Added second permanent test explicitly checks unknown input
  signs are numerically validated and retained negative signs do not force
  evaluation. Logs /tmp/aern-hyper-expm1-final-{all-debug,all-release,doctests,
  clippy,probe}.log. Three permanent Criterion cases run successfully;
  /tmp/aern-hyper-expm1-final-criterion.log. Memcheck of the benchmark's
  e^10000-1 path reports zero errors/leaks and 4,280 bytes still reachable.
- Selected Expm1 same-lock A/B: /tmp/aern-hyper-expm1-sign-ab{,-summary}.jsonl,
  twelve workloads / fifteen CPU-6 paired blocks each. Candidate/baseline
  paired ratios with bootstrap 95% intervals: tiny 1.0004 [0.9863,1.0813];
  small 1.0169 [0.9474,1.0411]; half 0.9831 [0.9472,1.0074];
  7/2 1.0206 [0.9819,1.0744]; -7/2 1.0100 [0.9958,1.0377];
  10000 1.0265 [1.0134,1.0658]; -10000 0.9863 [0.9562,1.0131];
  cached 0.9928 [0.9765,1.0126]; constructor 0.9716 [0.9490,1.0493];
  coarse-small 1.7811 [1.5018,1.9919]; coarse-negative 0.9745 [0.9470,1.0897].
  All fine-precision and coarse-negative allocation traffic is unchanged.
  Coarse-small adds four calls/80 bytes (5/464 -> 9/544), about 74 ns by
  pooled medians. The large-input 2.65% time cost is retained as a measured
  tradeoff, not hidden as noise. Executable 1,533,608 vs 1,533,384 (+224 B),
  text +348 B, data unchanged, bss layout -352 B.
- Direct 16,384-bit MPFR Expm1 oracle confirms all nine already-valid
  benchmark answers meet the contract on BOTH variants, with identical
  errors. Only the coarse positive repair differs: 1.74084454 invalid
  target units -> 0.25915546 valid; its 31.32x timing ratio is NOT an
  equal-accuracy comparison. Logs /tmp/aern-hyper-expm1-bench-oracle-
  {baseline,sign}.log. All three benchmark locks match the hash above.
- Ported the selected Expm1 kernel/test/bench changes (three files only)
  to live Hyperreal over da66ba3. No generated benchmarks.md overwrite.
  Live debug/release/Clippy and all five downstream crates are running.
  No Expm1 commit or push yet; production change uses only existing sign
  proof API, no new dependency or unsafe code.
- Finished aern-real RefinementOrderRounding/FieldOps.hs (727), its hs-boot
  dependency-cycle interface/commentary (78), and MixedFieldOps.hs (422).
  ALL 28 aern-real tracked files / 6,153 lines now read. Overall original
  AERN coverage 85 files / 12,613 lines. The in/out mixed helpers correctly
  select multiply/divide efforts (unlike their numeric-order counterparts).
  Refinement-monotonicity laws and weaker consistency-dependent power/
  distributivity relations account for interval dependency; omitted a-a=0
  and a/a=1 interval identities are not themselves arithmetic defects.
  No new transfer chosen from these interface/default-wrapper files.
- MixedOpsProbe.hs uses an independent rational specimen with exact division
  and valid but coarser directed lattice rounding to test the actual donor
  generic mixed-division helper. Separate tagged default efforts distinguish
  addition (13), multiplication (29), division (47). Build is in progress;
  no claim yet of an observed MPFR/Double backend failure. Pinned donor
  remains untouched; all compatibility changes/probes remain under /tmp.
- RETAINED Expm1 correction as Hyperreal
  d219a350462357af07d035fd316cba3b1cc339c9, "Require a proven range for
  coarse expm1 approximations". Live 792-test debug/release, 24-doctest,
  all-target/all-feature Clippy, fmt and diff gates pass. Downstream rerun
  passes Hyperlattice 19, Hyperlimit 242, Hypertri 3, Hypersolve 783, and
  Hypercurve 1,702 (44 suites; 4 explicitly ignored). Hyperreal is clean.
  Concurrent user edits appeared in Hypercurve src/bezier_offset.rs during
  downstream qualification and are untouched/not attributed to this audit.
  No push. Final source is the qualified isolated candidate, excluding only
  its generated benchmarks.md replacement.
- MixedOpsProbe.hs confirms the actual donor numeric mixed defaults return
  addition effort (13,13) rather than multiplication/division (29,47);
  the refinement counterparts return (29,47). It also reproduces the generic
  upper-bound defect: exact rational division 1/2 followed by a legitimate
  coarser downward maximum returns 0 as an alleged upper bound. This is a
  counterexample to the generic directed contract, NOT an observed AERN
  MPFR/Double instance failure. Logs /tmp/aern-mixed-ops-{build-final,probe}.log.
  Hypersolve algebraic_binary.rs interval_mul/div and exact endpoint sorting
  were re-read: they retain exact Real endpoints and return unknown when
  ordering is unresolved, not a rounded maximum in the wrong direction.
  Existing downstream interval-division regressions pass; no transfer needed.
- aern-interval reads now include LICENCE (30), Setup.hs (2), all six
  Basics/Interval facade and implementation files (27/72/216/134/315/447),
  RealArithmetic/Interval.hs (147), Effort.hs (205), Floating.hs (246), and
  UpDnConversion.hs (159). Twelve new complete files / 2,000 lines;
  cumulative 97 files / 14,613 lines. Strict endpoint fields only force WHNF;
  explicit NFData instances deeply force both endpoints. Size-limit changes
  use opposite endpoint rounding for outer/inner bounds. Anticonsistent
  intervals are retained as refinement-order objects, not confused with
  ordinary empty sets. Numeric comparison refuses non-consistent inputs;
  refinement comparison remains a contravariant/covariant endpoint order.
- Scalar conveniences intentionally remain partial: Eq/Ord throw when
  comparison is unresolved; signum/pi and many Floating methods are missing
  or explicit errors. IntervalApprox ordering compares outer intervals,
  unlike equality which compares both inner/outer. Generator concerns:
  some relation requests deliberately generate undecidable cases; missing
  Just Inconsistent case and unsupported relation generators can throw;
  endpoint list pairing is partial. These are not borrowed as safe APIs.
- Pinned interval package initially fails because Effort.hs references a
  class omitted from its explicit import list. Adding only that import in
  /tmp reveals a modern-Prelude (<*>) ambiguity in Floating.hs. The second
  isolated adjustment hides Prelude's Applicative operator; no numerical
  implementation is altered. Logs /tmp/aern-interval-{untouched,import,
  compat}-build.log. The complete package build is still being checked.
- IntervalApprox inclusion's false branch uses outer1-not-contained-in-
  outer2, which does not generally refute containment of the represented
  interval. Independent IntervalOrderProbe.hs is prepared: exact endpoint
  refinement-order checks plus exhaustive small inner/outer pairs, including
  loose outer [0,10], inner [1,2] versus exact [0,5]. Not yet accepted as a
  reproduced finding until the actual donor helper runs successfully.
- Interval package now builds all 18 modules after ONLY the two recorded
  import compatibility fixes. IntervalOrderProbe runs the actual untouched
  order/inclusion code: 625 exact endpoint refinement comparisons pass, but
  1,006 of 4,900 inner/outer approximation pairs yield unsound inclusion
  certificates (false refutations). Each contradiction has a concrete
  admissible pair of represented intervals, so this is not inferred merely
  from an unknown comparator. The loose [0,10]/[1,2] versus exact [0,5]
  example is reproduced. Logs /tmp/aern-interval-order-{build,probe}.log.
- Finished all remaining interval source: .gitignore (1), Conversion (152),
  Measures (94), ExactOps (51), SpecialConst (47), FieldOps (524),
  ElementaryFromBasis (71), ElementaryFromFieldOps (252), MixedFieldOps
  (882). ALL 22 aern-interval files / 4,145 physical lines now read;
  cumulative original AERN 106 files / 16,687 lines. This is source read
  completion for that package, NOT closure of its numerical validation.
- Interval ideas: sign/consistency-sensitive Kaucher multiplication with
  conservative alternatives on missing facts; thin-endpoint elementary
  evaluation; distinct inner/outer conversions/constants; mixed scalar
  operations reuse native endpoint arithmetic. Hyper's exact endpoints and
  certificate/unknown boundaries already cover the immediate scalar needs.
  Inward sin/cos, IntervalApprox inward exp/sqrt, and multiple Floating
  methods remain explicit errors. Anti-consistency is not a substitute for
  Hyper's domain-error handling. Distance aggregation uses default rather
  than caller-selected addition effort, a scheduling limitation to retain
  in the comparison, not yet a numerical counterexample.
- CORRECTION: the initial FieldOps all-unknown multiplication suspicion was
  a source-reading error: the third upper candidate is l1*l2, not a repeated
  l1*r2. All four corners are present. The actual donor function agrees with
  the independent four-corner oracle on all 225 fully known consistent
  interval pairs and has zero inward/outward enclosure violations among
  57,600 cases covering all 256 masks of known/unknown sign and consistency
  facts. The initial probe's final assertion incorrectly expected the alleged
  defect (1,2); the actual result for [-2,-1]^2 is the correct (1,4). Correcting
  that assertion does not change the donor or the independent oracle.
  No multiplication defect, anticonsistent-table validation, or MPFR-backed
  kernel validation is claimed from this consistent-interval probe.
- Corrected IntervalProductProbe rebuilt and exits 0; log
  /tmp/aern-interval-product-probe.log records all 57,600 cases passing.
- Finished all 22 aern-double tracked text files / 2,388 lines (manifest
  already read); coverage increases by 21 files / 2,251 lines to 127 files /
  18,938 lines. Three API facades are type synonyms over Interval Double,
  not separately enforced scalar/set types. Constant pi/e bounds are explicit
  binary endpoints. Mixed arithmetic reuses the generic conversion helpers;
  exact Double min/max masks the generic wrong-direction maximum issue.
- Double arithmetic uses unsafePerformIO to switch hardware rounding upward
  without restoring it; downward operations negate upward calculations. The
  Minimal demo itself records standalone rounding trouble. This optimization
  is not a safe transfer without evaluation-order and execution-context proofs.
  Conversion steps decodeFloat mantissas; subnormal/overflow and one-sided Int
  bounds require adversarial validation. PositiveDI maps endpoint signs without
  re-sorting. Bisection uses 0.5*(left+right), may overflow or fail to progress,
  and does not validate an explicit split point. Demo integration lacks a
  progress guard; demo zero search has one but claims roots from interval image
  containment alone (not an existence certificate). No new Hyper change selected.
- Isolated Double build pins ieee-utils-tempfix 0.4.0.1, Cabal metadata SHA
  435f35ed975e6562a9cd825008ffcdf56bddf105b87755aae4e297f0c4bddd57.
  Numerical validation is in progress; no successful backend build or oracle
  qualification is implied yet. Pinned AERN remains untouched.
- aern-double builds all 12 modules after adding Prelude hiding ((<*>))
  ONLY to its three isolated API facades. Existing missing-method warning
  rrEffortSelfMixedField remains; this is buildability, not warning-clean or
  numerical qualification. Log /tmp/aern-double-compat-build.log.
- Actual donor DoubleConversionProbe checks 2,178 exact-rational cases in
  each fresh process: 598 invalid directed enclosures in nearest mode and
  474 in upward mode, zero exceptions. Subnormal mantissa nudges can round
  back to the same endpoint, and overflowing rational conversion can return
  two identical infinities for a finite input. Logs
  /tmp/aern-double-conversion-{nearest,up}.log. Out-of-range Int wraparound
  looks surprising but all six sampled results are valid DIRECTIONAL bounds;
  these APIs do not promise exact casts. Do not label those six as unsound.
  Actual bisect confirms overflow for [MAX,MAX] and no progress for [0,minsub].
- Hyper re-read: finite f64 import decomposes bits exactly; checked integer
  exports reject out-of-range; enclosure filters use next_up/next_down and
  reject unsupported subnormal ratios. However, the dyadic enclosure branch
  can produce infinity next to f64::MAX despite the public finite-bound
  contract. Actual client reproduces both signs of MAX-1 and MAX+1 returning
  Some with an infinite endpoint; these are extended-real enclosures but NOT
  finite binary64 enclosures. Exact MAX returns correct finite singleton.
  Evidence /tmp/aern-hyper-finite-baseline.log; no live fix yet. Candidate
  work will preserve finite enclosures below MAX and reject values above it.
- DoubleFieldProbe runs actual donor add/multiply/divide against exact
  rational operations on 52 finite binary inputs. In EACH initial rounding
  mode (nearest and upward), 5,408 addition, 5,408 multiplication and 5,304
  division directed checks pass with zero invalid bounds. Every operation
  starts after an explicit mode reset. This qualifies these cases under the
  current GHC build, NOT arbitrary optimizer/concurrency behavior. Logs
  /tmp/aern-double-field-{nearest,up}.log; elementary functions remain pending.
- Read eight aern-mpfr-rounded files fully: LICENCE 30, Setup 3, MPFR facade
  80, Basics 190, Utilities 37, ExactOps 42, Elementary 47, FieldOps 138.
  Coverage now 135 / 19,505. Existential MPFR precision is checked dynamically
  before unsafeCoerce aligns precision types; withPrecRoundDown coerces only
  rounding-mode phantom types. Arithmetic uses explicit type-level upward
  rounding and sign duality, unlike Double's hardware-mode mutation. Basic
  literals deliberately throw; Elementary's downward exp/sqrt nevertheless
  contain bare MPFR literal 1, an apparent reachable partiality to reproduce.
  Mixed precision comparisons also throw; no adoption of those partial APIs.
- Hyper finite-bound candidate uses the existing normalized top-64-bit
  round-to-odd head to compare a value rounded to MAX with exact MAX. Since
  MAX's normalized head ends in eleven zero bits, any discarded positive tail
  changes its sticky bit and is distinguished from equality. The rounded-MAX
  precondition fixes the exact exponent at 1023. Below-boundary values retain
  [MAX.next_down(),MAX], sign-reflected for negatives; above-boundary values
  return None. No dependency, allocation, or unsafe production code added.
- Initial inline candidate passes 126 GMP rational boundary cases (seven
  denominator shifts through 4,096 bits, both signs, exact/quarter/half/full
  ULP offsets and tiny tails), 793 tests/examples each debug/release, 24 docs,
  all-feature/all-target Clippy and fmt/diff checks. Same permanent regression
  fails on d219a35 test-only baseline. Initial compilation drafts had a Rust
  shift-type annotation and a benchmark splice error; both fixed before tests
  or measurements. Evidence /tmp/aern-hyper-finite-candidate-*.log and
  /tmp/aern-hyper-finite-baseline-focused.log.
- Inline candidate NOT retained: 14-workload, 15-block CPU6 ABBA/BAAB benchmark
  finds a 22.81% wide non-dyadic slowdown [21.57%,24.93%], despite unchanged
  numerical path. Other small inexact paths +2.12%/+3.84%; allocations unchanged.
  /tmp/aern-hyper-finite-ab{,-summary}.jsonl; common lock SHA
  e0e0e133e43a2c5762987f7bb6f1b3ea402219503dd61ca2e1458a32d5bfbc5b.
  Inline source delta preserved /tmp/aern-hyper-finite-inline.patch; original
  benchmark binary in bench-candidate is not overwritten. Testing a cold,
  out-of-line boundary helper in isolated hyperreal-candidate next. No live
  finite-bound edit or commit yet; Hyperreal and concurrent Hypercurve are clean.
- Eight more aern-mpfr-rounded files read: NumericOrder 199, Conversion 154,
  Measures 46, ShowInternals 37, SpecialConst 34, MixedFieldOps 316,
  Interval/MPFR 103, tests/Main 94. Cumulative 143 / 20,488. Size changes and
  MPFR-to-Double conversion are explicitly unimplemented. Integer-to-MPFR
  downward conversion negates Int before promoting, so minBound needs a probe.
  Sample values, area generators, default bisection and downward e also use
  bare MPFR literals despite the throwing Num/Fractional constructors. Mixed
  primitive operations currently convert rather than use MPFR's native mixed
  entry points. Tests still have interval exp/sqrt registrations commented out.
  These are source findings, not yet actual rounded-backend executions.
- Cold out-of-line MAX helper does not remove the observed wide-rational
  slowdown: paired median 1.2248 [1.2165,1.2583], no allocation change. Profiling
  locates the extra cycles in unchanged compare_shifted_biguints, called by
  exact MSD classification; the function has the same 2,049-byte size and hot
  instruction sequence but different placement. Treat binary-layout sensitivity
  as a hypothesis, not a proven numerical-path regression cause. Evidence
  /tmp/aern-hyper-finite-{baseline,outlined}.perf, outlined source delta
  /tmp/aern-hyper-finite-outlined.patch, and outlined ABBA/BAAB logs. No live port.
- New bounded-scale candidate avoids that unnecessary exact MSD comparison:
  numerator_bits-denominator_bits selects an exact power-of-two scale, while
  the existing leading-word numerator/denominator intervals provide containment.
  Final next_down/next_up and finite checks remain. Scale range [-1022,1023]
  also admits some formerly unsupported near-boundary non-dyadic values without
  accepting infinities. Added 1,408 independent GMP signed-rational checks over
  widths 8/54/129/4096 and exponents -1074 through 1024; focused enclosure tests
  pass. Full qualification and the 14-workload paired benchmark are pending.
- Finished rounded-MPFR API facade (621) and all five demos: Minimal 108,
  LogisticMap 101, IntermediateValue 157, SpringMass 213, SpringMassV 727.
  ALL 23 aern-mpfr-rounded files / 3,569 lines now read; original AERN total
  149 / 22,415. This closes source reading, NOT backend validation.
  Facade setPrecOut converts BOTH endpoints upward via withPrec: decreasing
  precision can move the lower endpoint inward (actual reproduction pending).
  IntermediateValue uses alternate trisection points but its interior/thin-point
  guard is commented out. SpringMassV retains initial-condition dependence as
  polynomial variables, bounds discarded terms, and tests derivative signs for
  corner evaluation; these ideas are already represented by Hyperlimit/Hypersolve
  certificates and exact polynomial ranges. Its Picard stop check examines only
  y, not the full (y,y') vector, and iteration exhaustion returns without a final
  inclusion certificate. Do not copy it as a validated IVP solver. These older
  demos also contain stale APIs and unimplemented precision/literal paths.
- SELECTED bounded-scale variant: 14 workloads, 15 CPU6 ABBA/BAAB blocks,
  10,000 paired bootstrap medians; /tmp/aern-hyper-finite-bounded-ab{,-summary}.jsonl.
  Candidate/baseline ratios: exact word 1.0016 [0.9955,1.0034], exact wide
  0.9419 [0.9389,0.9738], exact subnormal 0.9902 [0.9725,1.0030], inexact word
  0.9962 [0.9918,1.0017], inexact wide 1.0075 [1.0025,1.0111] (about 0.15 ns),
  non-dyadic 0.7429 [0.7332,0.7466], wide non-dyadic 0.4328 [0.4286,0.4379],
  overflow rejection 0.3645 [0.3614,0.3691], unsupported subnormal rejection
  0.6954 [0.6935,0.6993], exact MAX 0.9101 [0.9065,0.9239]. Four repaired
  near-MAX cases cost roughly 0.45-1.25 ns more; their old infinite endpoints
  violate the finite contract, so those are not equal-contract regressions.
  Every allocation count/requested-byte count is unchanged: exact subnormal
  1 allocation/8 bytes, all other measured exports zero.
- Selected standalone benchmark executable 1,005,096 bytes vs 1,004,664
  baseline (+432); text +428 bytes, data unchanged, BSS +3,680 from layout.
  No object layout or production dependency change. Same benchmark lock SHA
  e0e0e133e43a2c5762987f7bb6f1b3ea402219503dd61ca2e1458a32d5bfbc5b, bigint0.4.6.
  Separate GMP benchmark-point oracle confirms identical bound bits/results
  on ALL ten previously valid measured cases; all four previously invalid
  near-MAX cases now satisfy the finite contract. Logs
  /tmp/aern-hyper-finite-bench-oracle-{baseline,bounded}.log.
- Selected isolated source passes 794 tests/examples each debug/release,
  24 docs, all-feature/all-target Clippy -D warnings, fmt/diff; three permanent
  Criterion cases execute successfully. Memcheck near-MAX deep-tail export:
  zero errors, zero definite/indirect/possible leaks, 552 bytes reachable in
  runtime caches. Logs /tmp/aern-hyper-finite-bounded-{debug,release,clippy,docs,
  criterion,memcheck}.log. Ported ONLY the three qualified source/test/bench
  files live over d219a35; no generated benchmark document overwrite. Live
  and all five downstream checks are running; no finite-bound commit/push yet.
- Live finite-bound source passes 794 tests/examples each debug/release,
  24 docs, all-feature/all-target Clippy -D warnings, fmt/diff checks. Live
  downstream all-feature tests pass: Hyperlattice 199, Hyperlimit 361,
  Hypertri 187, Hypersolve 784. Hypercurve's expanded 895-unit suite is still
  running expensive exact geometric cases with continued progress and no
  observed failures; no completion count or finite-bound commit claimed yet.
  Logs /tmp/aern-hyper-finite-live-{debug,release,clippy,docs}.log and
  /tmp/aern-hyper-finite-hyper{lattice,limit,tri,solve,curve}.log.
- Rounded-MPFR builds all 14 modules using rounded 1.1.1, hgmp 0.1.2.1 and
  long-double 0.1.1.1 with pinned Cabal metadata in the isolated Stack YAML.
  Only its API facade needs Prelude hiding ((<*>)); no numerical code changed.
  Missing-method warnings remain. This is a GHC 9.6.7 compatibility build,
  NOT a reproduction of the 2015 dependency environment. The separately
  downloaded rounded 0.1 Cabal/README files reveal later compiler bounds
  and a copyright through 2018; do not call that a period dependency either.
- Actual RoundedMPFRProbe reproduces 3,864 invalid setPrecOut enclosures
  among 7,695 exactly representable singleton/precision pairs. Both endpoints
  round upward; e.g. -255/2 at precision 2 becomes the singleton -96.
  Downward Int minBound converts to +2^63 (one invalid result / 14 directed
  checks). expDn(1), sqrtDn(2), eDn and default bisection throw due to the
  deliberately unimplemented bare MPFR literal constructors; MPFR-to-Double
  throws its missing-method error. Explicit bisection and upward counterparts
  execute; mixed-precision comparison explicitly throws as its API specifies.
  Basic add/multiply/divide pass all 10,080 exact-rational directed checks at
  precisions 2/24/53/100. Log /tmp/aern-rounded-mpfr-probe.log; source/probe
  build logs /tmp/aern-mpfr-rounded-{modern,compat}-build.log and
  /tmp/aern-rounded-mpfr-probe-build.log. Donor findings are not Hyper defects.
- Read 17 additional hmpfr-backend files / 1,428 lines (licence, setup,
  gitignore, scalar facade, all scalar modules, interval utilities and tests).
  Cumulative AERN 166 / 23,843; this package's public interval facade and five
  demos remain unread. Unlike rounded-MPFR, hmpfr uses direct directed MPFR
  exp/sqrt, supports mixed precisions via maximum operand precision, implements
  directed size changes and native Int/Double mixed arithmetic. Its integer
  import avoids bounded negation. MPFR-to-Double reconstructs a scaled binary
  significand and uses suspicious MAX constants decoded from zero; subnormal
  double-rounding and overflow need actual probes. Mixed Double downward add
  uses detectNaNUp, an inconsistent extended-endpoint fallback to investigate.
  No Hyper transfer selected. hmpfr 0.4.5 builds; AERN needs the same isolated
  modern Prelude import adjustment before backend probes can run.
- Finished the hmpfr public interval facade (591) and all five demos (1,311),
  another six files / 1,902 lines. ALL 24 aern-mpfr files / 3,439 lines read;
  cumulative AERN 172 / 25,745, every coverage range and SHA reverified.
  The public facade uses correct directed changeSizeLimitsOut and direct
  M.toDouble in doubleBounds, unlike the generic conversion implementation.
  Demos repeat the previously recorded trisection, Picard and polynomial
  dependency/range ideas; SpringMassV's inclusion stop still examines only y,
  retains whole iteration histories for reporting, and hardcodes three partial
  derivatives. No full IVP certificate or new transferable algorithm claimed.
- hmpfr 0.4.5 plus all 14 AERN backend modules build after facade-only
  Prelude hiding ((<*>)); warning rrEffortSelfMixedField remains. Actual
  HMPFRProbe runs 1,400 directed binary64 exports in each fresh hardware
  rounding mode: generic conversion has 38 invalid bounds in nearest and
  112 in upward, while facade doubleBounds (direct MPFR export) has ZERO.
  Source dyadics are checked exactly before use; subnormal reconstruction
  loses direction. Both modes pass all 7,695 precision-reduction enclosures
  and the signed Int minimum conversion. Downward exp/sqrt/e execute, unlike
  rounded-MPFR. Opposed-infinity mixed add maps BOTH directions to +infinity:
  an inconsistent NaN-widening fallback, not a finite-input real counterexample.
  Logs /tmp/aern-hmpfr-{compat-build,probe-build,probe-nearest,probe-up}.log.
  Initial probe compile lacked the rounding helper in the new Stack plan;
  registering the already-compatible Double package resolves it, without
  modifying numerical code. Pinned original AERN stays clean.
- Hyper comparison: signed primitive Rational construction uses unsigned_abs
  before magnitude promotion, avoiding bounded negation; exact rational
  endpoints are not reduced by an upward-only precision operation. No fix
  is warranted for those donor patterns. The selected finite-export regression
  remains the sole uncommitted Hyper candidate from this backend slice.
- Hypercurve unit gate completed: 889 passed, six ignored, zero failed
  (895 total entries), 749.81 seconds. Integration suites continue; the earlier
  commentary shorthand about 895 passing units should be read with this exact
  pass/ignore distinction. This is not yet the full downstream completion.
- Actual Double elementary probe uses independent 512-bit directed MPFR
  bounds on exact binary inputs, resets hardware rounding before EVERY donor
  evaluation and distinguishes proven violations, oracle ambiguity, exceptions
  and one-second timeouts. Each fresh initial mode (nearest/upward) passes
  50 exp, 33 sqrt, 64 sin and 64 cos singleton enclosures, with no ambiguity,
  exception or timeout. Inputs span subnormals, binade boundaries, large
  arguments, and binary approximations near trigonometric quadrant boundaries.
  This is targeted default-effort qualification, not all effort/domain cases.
  Logs /tmp/aern-double-elementary-{build,nearest,up}.log.
- Finished all 17 aern-realfn text files / 3,428 lines (52-line manifest
  already counted); adds 16 files / 3,376 lines. AERN total 188 / 29,121.
  All 14 modules build without package-specific source edits, but with many
  pre-existing partial-pattern/unused/missing-method warnings; log
  /tmp/aern-realfn-initial-build.log. This closes source reading only.
- Function-space ideas: separate derivatives of represented functions from
  derivatives of enclosing boundary functions; only exact approximants make
  these interchangeable. Hyper's exact polynomial/symbolic derivative paths
  preserve that distinction. Domain sampling is fairly interleaved and used
  only to obtain negative comparison information; positive whole-domain claims
  require a range proof. Box bisection permits overlap and does not guarantee
  strict progress. Interval-of-functions wrappers assume common domains/limits;
  IntervalApprox projection explicitly duplicates outer as inner despite the
  known degree-zero problem, and several inner/aggregate methods remain TODO.
  Generic law tests accept unknown comparisons and sample only one point,
  so their pass result is weaker than independent certificate validation.
- Bernstein basis construction uses rounded factorial/product coefficients;
  Chebyshev uses the standard three-term recurrence. Tau reciprocal constructs
  a range-normalized Chebyshev polynomial and an explicit residual error bound.
  Its author's derivation was read at
  https://michalkonecny.github.io/aern/_site/posts/division.html (2014-04-27),
  including the coefficient recurrence and residual identity. Constant-range
  c=b and coefficient overflow are not handled explicitly in the generic
  source. Bernstein min/max uses convex hinge approximants and iterative
  downward-offset adjustment without a progress cap. Rounded coefficient
  endpoint choices and uncertain branch comparisons require concrete polynomial
  backend probes before either algorithm is considered safe for transfer.
  Hyper currently has exact scalar min/max/reciprocal and certified polynomial
  ranges; no unqualified generic function-space layer is adopted.
- Function generator caveats: single-function arbitrary selection ignores
  its area/range argument, NC pair generation forces contact but not necessarily
  genuine crossing, and power-term selection uses take(n-1), omitting an extra
  variable. Domain tuple generators and sampling have empty-domain/thin-domain
  edge cases to test. These are source findings pending actual concrete
  polynomial probes, not alleged new Hyper defects. Next source package:
  aern-poly; do not count its source as read from its manifest alone.
- Opened aern-poly: read LICENCE (30), public scalar facade IntPoly.hs (203),
  and Config.hs (541) completely; manifest (90) was already read. Adds three
  files / 774 lines, AERN total 191 / 29,895. Configuration stores ordered
  variable names, left-shifted domains and origins separately, with degree,
  term-count, coefficient-limit and algorithm-effort controls. Config combining
  keeps the first domain and merges limits only; callers must establish domain
  compatibility. Increment variants/sequence leave coefficient limits unchanged;
  IntPolyEffort.effortIncrementSequence calls effortIncrementVariants rather than
  the sequence operation. These are scheduling/contract findings awaiting
  concrete probes. Isolated aern-poly build stops at removed IntMap.fold in
  IntPoly/IntPoly.hs:114; no poly source compatibility edit made yet. Next read
  is that complete 457-line representation file, then inspect fold semantics
  before a narrowly scoped compatibility change. Log /tmp/aern-poly-initial-build.log.
- RETAINED Hyperreal 2808bc886db969d487263ea073eb1cf9df562eac,
  "Preserve finite rational enclosures and avoid exact binade comparisons".
  Only three qualified source/test/benchmark files changed (153 additions,
  five deletions); worktree clean after commit, no push. The finite-bound
  contract correction and 26%/57% typical/wide non-dyadic export improvements
  justify the measured small code-size and boundary-path costs above; the
  two slower experimental variants remain rejected.
- ALL five live downstream all-feature test gates completed successfully:
  Hyperlattice 199, Hyperlimit 361, Hypertri 187, Hypersolve 784, Hypercurve
  1,731 passed / nine ignored across 44 suites, zero failures. Hypercurve's
  total is 889 unit passes / six ignored plus 842 integration passes / three
  ignored; its tested HEAD is 9e2796cbf36dde2f56087a717b34d31edbf80f51.
  Hypersolve was tested at 1841884f175254460cf15facd83f8c4947315ff5;
  concurrent user work advanced it to 8bd17304631bdea0f9a68bde58e98cfac1d8d961.
  The gate is NOT claimed to qualify that later unrelated revision. Live
  Hyperreal itself passed 794 tests/examples in EACH debug/release mode,
  24 doctests, Clippy -D warnings and format/diff checks, as recorded above.
- Read all 457 lines of aern-poly IntPoly/IntPoly.hs; AERN total now
  192 files / 30,352 physical lines. The representation is a recursively
  variable-ordered IntMap of coefficients, requires a constant term at every
  variable level, and normalizes degree-zero branches. Degree collection is
  ascending; coefficient traversal's degree list is reversed as documented.
  The local legality check does not enforce the mandatory constant-term
  invariant. Common variable order/domain assumptions require caller review.
- Actual PolyStructureProbe confirms termsMapConstCoeff (+2) changes x+1 to
  3*x+3, rather than x+3: line 139 maps ALL coefficients instead of applying
  the existing constant-only map helper. This is a donor structural defect;
  reachability through public numerical operations and Hyper analogues remain
  to inspect. Log /tmp/aern-poly-structure-probe.log; build log
  /tmp/aern-poly-structure-build.log. The isolated source's ONLY new change
  is IntMap.fold -> IntMap.foldr at line 114, verified equivalent against
  https://hackage.haskell.org/package/containers-0.5.5.1/docs/Data-IntMap.html#v:fold.
  The pinned reference remains unmodified; no arithmetic algorithm patched.
  Next read: New.hs, Reduction.hs and arithmetic modules, before concrete
  function-space enclosure probes. Full polynomial-package build remains open.
- Finished ALL 26 aern-poly text files / 8,597 physical lines, including
  demos, tests, comments and inactive implementations. Adds 21 files / 7,276
  lines; cumulative AERN 213 / 37,628. One output truncation in Evaluation.hs
  was resolved by rereading lines 481-720 completely before crediting the file.
  Pinned reference remains clean. All 21 polynomial modules build with warnings
  in the isolated modern environment: three deprecated IntMap fold names map
  to their documented right-fold equivalents; New, Reduction, Evaluation and
  Integration additionally need UndecidableInstances. These are compatibility
  adjustments only, not arithmetic repairs. Logs /tmp/aern-poly-compat-build{,2,3,4}.log.
- Polynomial ideas compared: sparse recursive convolution followed by degree
  and term-count reduction; discarded high-degree terms flow into lower-degree
  coefficients with enclosing domain-power factors, rather than being dropped.
  Horner evaluation, outward derivative sign tests, endpoint specialization,
  bounded subdivision and counterexample sampling separate proof from heuristics.
  These broad ideas already exist in Hyper's exact/certified polynomial paths;
  no new algorithm is transferred solely from the source-level similarity.
  Composition reuses evaluator operations; its placeholder width is zero,
  disabling adaptive splitting. Constant divisors bypass Tau reciprocal, so the
  generic c=b concern does NOT automatically apply to public constant division.
- Source-level caveats awaiting concrete probes: projection uses fromAscList
  on descending keys [1,0]; domain adjustment changes stored origins without
  translating coefficients; differentiation may remove required degree-zero
  branches, while mixed addition's missing-constant fallback keeps an extra
  variable level. Term-count reduction retains all ties at its threshold and
  indexes negatively for maxsize=0, the default for zero variables. Empty
  composition takes a head of an empty substitution box. Polynomial-interval
  evaluation constructs an entirely uninitialized effort record, and its merge
  is a left/right splice. Multiple inward/elementary operations remain absent.
  Up/down wrappers generally select endpoints from outward arithmetic; constant
  min/max and pi/e wrappers instead return full coefficient enclosures and need
  contract-specific checking. No unsupported donor numerical bug claim yet.
- The confirmed constant-only mapper defect has no callers outside its own
  helper family in the pinned repository; ordinary mixed addition has its own
  implementation. It is publicly reexported, but must NOT be blamed for all
  constant additions. Public-API boundary and independent enclosure probes are
  next, followed by remaining IVP/function-representation/GUI source packages.
- Actual PolyBoundaryProbe (public APIs, actual donor library) results:
  projection lookup/traversal and evaluation at 2/3/4 on [2,4] are CORRECT
  in current containers, despite violating fromAscList's documented ordering
  precondition. No observed projection numerical defect is claimed. Adjusting
  the same projection's domain to [3,4] changes its value at 3 to 2, confirming
  origin rebasing without coefficient translation; this is not a restriction
  preserving the represented function. Caller semantics remain to inspect,
  especially the unread IVP SplitNearEvents caller at line 364.
- The derivative of x^2 has no constant branch and evaluates correctly to 2
  at x=1; subsequently adding integer 2 throws an illegal polynomial-shape
  exception (an extra variable node in the synthesized constant branch).
  A six-term equal-width polynomial remains six terms after maxsize=2 reduction.
  Zero-variable default maxsize=0 makes addition of two constant-one
  polynomials throw a negative-index exception. These are actual public-path
  donor completeness/resource-limit failures, not inferred hypothetical cases.
  Log /tmp/aern-poly-boundary-probe.log. Probe import drafts were corrected
  to use public reexports and the explicit Basis.Double orphan-instance import;
  donor numerical source was unchanged.
- Independent PolyOracleProbe evaluates actual donor outputs against exact
  Rational functions at nine dyadic points in each of [-1,1], [0,1], [2,4].
  EACH initial hardware rounding mode (nearest/upward, reset per case) passes
  all 309 operation cases / 2,781 point-enclosure checks: projections,
  addition/subtraction/multiplication, powers reduced across degree limits
  0/1/2/5 and term limits 1/2/6, positive Tau reciprocals at degrees 2/3/5/8,
  and shifted pointwise min/max at four Bernstein degrees. Zero violations,
  exceptions or timeouts. This is sampled consistent-input qualification, NOT
  a proof of all domains, inner operations, effort choices or termination.
  Logs /tmp/aern-poly-oracle-{build,nearest,up}.log. In particular the earlier
  min/max rounding suspicions are NOT confirmed by these experiments.
- Hyper comparison reread Hypersolve's exact coefficient/interval evaluation
  and quadratic accumulators. Dense coefficient derivative/evaluation paths
  do not require AERN's recursive constant-branch invariant. However, both
  quadratic accumulators collapse ALL higher-degree coefficients into one Real,
  allowing cancellation between distinct degrees or monomials. Public
  QuadraticResidual::from_expr and UnivariateQuadraticResidual::from_expr
  therefore require an independent reproduction and possible exactness repair.
  ProblemAnalysis already gates these calls on structural degree=2, so do NOT
  infer an end-to-end false solver certificate from the direct-API suspicion.
  No live Hypersolve changes made; clean HEAD d897ff0a5d416daa16c83f1bfe32334077fd07de,
  polynomial.rs last changed at 67f9ab4b16dd5d3b3d570d2c8f131ee9051e82b8.
  Standalone reproduction is building in /tmp/aern-hyper-quadratic.m4tTz4,
  log /tmp/aern-hyper-quadratic-baseline.log; this candidate takes priority
  over the next unread AERN IVP source while its exactness impact is resolved.
- CONFIRMED direct Hypersolve API exactness defect: actual standalone client
  exits 0 after reproducing all three mismatches. At x=2,y=3, x^2*(x-x^2)
  evaluates to -8 but BOTH extractors return a zero polynomial;
  x^2*(x-y) evaluates to -4 but the multivariate extractor returns zero;
  x^2+x^2*(x-y) evaluates to zero but its extracted quadratic evaluates to 4.
  The same client asserts that ProblemAnalysis rejects all three forms via
  its existing structural-degree guard. No false end-to-end solver certificate
  is claimed. Standalone dependency resolution used num-bigint 0.4.8; the
  reproduction is exact integer arithmetic, not a timing comparison.
  Next action: an isolated repair and regression/oracle/performance comparison
  for these public extraction boundaries. Preserve valid same-monomial
  cancellation where feasible; do not indiscriminately expand high-degree
  expressions or silently narrow accepted forms without measuring/documenting
  the completeness and resource tradeoff. No live Hypersolve edit or commit.
- Previous goal turn classified as PROGRESS: retained Hyperreal change,
  completed polynomial source reading/probes, and reproduced the new direct-API
  defect. Current live Hypersolve revalidated clean at d897ff0a5d416daa16c83f1bfe32334077fd07de.
  Isolated baseline/candidate clones in /tmp/aern-quadratic-fix.fsxgfP pin that
  revision plus Hyperreal 2808bc8, Hyperlattice d783af0b5ba52cacfeac62b1b25e1f9e36a8e7b0,
  Hyperlimit 0f1a32d54a1c063d134b0476082e49168516102a. Both native lockfiles SHA
  a3bd1655b3b08ac34afd084b1c74392c107a06a243672180f2d3a986ff442e43, bigint 0.4.6.
- Initial quadratic candidate preserves exact higher-degree coefficients by
  degree/monomial, merges ONLY identical monomials, and propagates higher tails
  through multiplication/scaling. It bounds cancellation expansion at degree
  64 and 256 nonzero higher-degree monomials; exhaustion returns None, never
  an invented zero. Ordinary quadratic term count is not capped. Multivariate
  leaf validation preserves unbound-symbol rejection even when coefficients
  cancel or get multiplied by zero. These limits and the tradeoff for larger
  genuine cancellations are explicitly documented rather than silently hidden.
- Candidate's six focused tests pass, including 2,048 generated univariate and
  2,048 multivariate expressions checked against an independent BigRational
  coefficient oracle with fixed-axis exponent arrays. Regressions cover the
  three confirmed false extractions, genuine cubic/quartic cancellation across
  multiply/divide/zero scaling, degree-64 acceptance/degree-128 exhaustion,
  many-variable term exhaustion, unbound symbols and zero-power/division domain
  obligations. Initial test draft referenced a nonexistent conversion helper;
  corrected to construct the oracle rational directly from signed numerator
  and denominator. Logs /tmp/aern-quadratic-candidate-{check,focused}.log.
  Full qualification and paired 18-workload performance screening are running;
  no live quadratic fix selected or retained yet.
- First map-tail candidate passes all 794 tests/examples in EACH debug/release
  run (10 suites); baseline with test-only regression fails as expected.
  Screening 18 workloads in 15 CPU6 ABBA/BAAB blocks reports dense multivariate
  extraction about 5-7% slower, while normal row analysis improves about 2%,
  zero-term elimination about 54%, and ordinary univariate work is neutral or
  faster. This screening overlapped early test compilation, so it is NOT the
  final retention benchmark. Two genuine-cancellation paths cost 6-8% more;
  correct recognition of scaled cancellation newly succeeds where baseline
  returned None. All 14 previously valid workload results remain byte-identical.
  Logs /tmp/aern-quadratic-map-screen-{ab,summary}.jsonl; original source delta
  /tmp/aern-quadratic-map-candidate.patch, immutable binary bench-map-candidate.
- The map-tail driver adds 46,208 file bytes / 33,632 text bytes. Testing a
  sorted-vector univariate tail to remove the second BTreeMap instantiation
  and reduce tiny-tail allocation size. Also replacing unnecessary complete
  Real structural-fact queries with zero_status's exact answer first, retaining
  structural_facts fallback on Unknown; no exact-zero evidence is discarded.
  Hyperreal source confirms full structural facts also compute magnitude/MSD,
  which coefficient zero filtering does not need. No live port yet.
- Final vector-tail candidate passes 795 tests/examples in EACH debug/release
  all-feature run (10 suites), plus warning-denying all-feature/all-target Clippy,
  formatting/diff checks and the doc gate (0 doctests). Seven focused tests now
  cover 22 named/opaque scalar families as well as rational coefficients.
  Another 32 reproducible seeds (1..32) each pass 2 x 2,048 independent-rational
  cases: 131,072 additional generated expressions. Logs
  /tmp/aern-quadratic-vector-{debug,release,seeded,clippy-final,doc}.log.
- Post-compilation timing: 18 workloads, 15 CPU6 paired ABBA/BAAB blocks,
  30ms arms, deterministic 10,000-resample paired bootstrap. Candidate/baseline
  median ratios [95% CI]: uni_square .947 [.931,.960], uni_factored .925
  [.920,.936], uni_wide .948 [.939,.959], uni_pi .950 [.906,1.021],
  uni_cancellation .983 [.979,.999], multi_square .771 [.768,.778],
  multi_cross .761 [.749,.766], dense8 .863 [.855,.872], dense32 .863
  [.857,.870], multi_zero_terms .399 [.395,.413], multi_pi .764 [.749,.785],
  multi_cancellation .901 [.880,.928], analyze_uni16 .902 [.884,.906],
  analyze_multi16 .819 [.807,.830]. The 14 previously valid driver outputs
  match byte-for-byte; a separate complete-Debug comparison of BOTH 16-row
  ProblemAnalysis values, not just summary facts, also matches byte-for-byte.
  Logs /tmp/aern-quadratic-vector-{ab,summary}.jsonl and
  /tmp/aern-quadratic-analysis-{baseline,candidate}.log.
- Changed-contract timings are not like-for-like speedups: the three repaired
  invalid extractions now return None at ratios .864/.722/.735; the newly valid
  scaled-cancellation case now returns Some at 1.029 [1.021,1.037]. Ordinary
  quadratic and analysis allocation counts/bytes are unchanged. Zero-term work
  drops 26 to 12 allocations (5,832 to 5,704 requested bytes), pi multivariate
  32 to 25 (5,128 to 4,624). Genuine univariate cancellation now requires two
  allocations/448B, scaled cancellation four/896B; multivariate cancellation
  adds one allocation/808B. Tail identity has a measured, explicit memory cost.
- Common standalone driver file grows 23,224B (6,572,904 to 6,596,128), text
  +15,700B, data +176B, BSS +512B. This includes the driver/unwind/runtime and
  is not an isolated library-size claim. No public scalar/object layout change.
  Vector tails reduce the initial map candidate's binary and allocation costs.
  Exactness repair and normal-path gains justify this bounded size tradeoff.
- Memcheck: all seven focused tests pass; zero definite/indirect leaks or memory
  access errors, with 48B possibly lost in the test process and 39,680B reachable.
  Separate direct dense32 driver has zero definite/indirect/possible loss and
  5,248B/100 blocks reachable, IDENTICAL to baseline. An intentionally all-kind
  leak-error run exits 99 on BOTH binaries for the same ten reachable contexts
  (global LazyLock scalar/cache/runtime allocations), not new lost memory.
  Logs /tmp/aern-quadratic-{vector-memcheck,vector-driver-memcheck,
  baseline-driver-memcheck}.log; final direct gate excludes reachable blocks.
- Permanent Criterion module adds all 18 workloads with checks outside timing;
  its --test smoke passes. Initial pi benchmark assertion incorrectly required
  PartialEq to recognize equality between expanded and unexpanded computable
  expressions; replaced by exact expected coefficient assertions, not a numeric
  tolerance. This was a benchmark assertion limitation, not a new scalar defect.
  Smoke log /tmp/aern-quadratic-vector-bench-smoke-final.log.
- Carefully ported ONLY polynomial.rs, tests/quadratic_extraction.rs and the
  certification benchmark module/registration to clean live Hypersolve d897ff0;
  both existing-file baseline hashes and native lockfile matched before port.
  Final polynomial SHA 263840d2556d62c5b95a5f954bc9720bbf3c8798b640e474f5157b0a99688035;
  test SHA 9cfc62a0efe58861170ca42bbd4769b5bfcd21f4c9ded602efcfff72065fb9e6.
  All four live files match qualified isolated files byte-for-byte. Live debug,
  release and Hypercurve downstream gates are running. Not committed yet.
  Hypercurve pin remains 9e2796cbf36dde2f56087a717b34d31edbf80f51; dependency
  Hyperreal/Hyperlattice/Hyperlimit pins remain exactly those above.
- Resumed AERN source reading: complete IVP Specification/{Hybrid,ODE},
  Solver/Bisection and Solver/Events/{Locate,SplitNearEvents}, 5 files / 2,252
  lines. Cumulative 218 text files / 39,880 lines. Complete ranges and pinned
  hashes added to AERN_READ_COVERAGE.tsv; no broader IVP completion claim.
- IVP architecture: uncertain-time/value parameterization, outward/inner endpoint
  evaluation, reusable split-once results, direct-vs-split precision heuristics,
  explicit Maybe failure and domain-exit flags. First-event search retains a
  certain/maybe/poor-enclosure distinction, merges possible event sets, and
  skips later search after a certain first event. These ideas reinforce Hyper's
  existing separation of candidate scheduling from exact certification; no new
  production transfer selected solely from similarity.
- SplitNearEvents's actual adjustDomain caller uses minOut(oldTimeDomain, cut),
  and bisectionInfoTrimAt removes segments entirely after the cut. With thin,
  ordered endpoints this only reduces the upper endpoint of a retained segment;
  its coordinate origin stays fixed. The earlier arbitrary-left-endpoint-change
  defect therefore does NOT establish this caller is numerically wrong. Actual
  right-trim probes are building to check the boundary, plus a positive-step
  event-localization probe near adjacent binary64 endpoints (midpoint progress).
- Other source caveats awaiting concrete validation: SplitNearEvents accepts but
  ignores its max event segment size and leaves the poor-enclosure predicate
  as False/TODO. Its catMaybes aggregation can mix missing mode results with
  successful ones; inspect EventTree/Picard semantics before claiming unsoundness.
  Bisection's reusable right result can predate invariant narrowing; this is
  conservative if it enclosed the wider initial state, not inherently unsound.
  Empty/mismatched component vectors remain caller assumptions in foldl1/zipWith.
- Isolated IVP compilation needs one more modern-containers compatibility edit:
  Specification/Hybrid Map.fold -> Map.foldr, preserving the legacy right fold.
  Pristine pinned donor unchanged. Initial compile hit compiler-cache sandbox
  writes, then rerun with approved escalation; no arithmetic changes introduced.
- Read complete IVP Picard/UncertainValue (584), Events/EventTree (716) and
  ShrinkWrap (411): 3 more files / 1,711 lines; cumulative 221 / 41,591.
  Picard inclusion is certified before iterating/stopping by improvement;
  shrink wrapping rescales domains through composition and derivative-based
  widening, with a bounded 100-trial search and full-width enclosure evaluation.
  No donor algorithm transferred into Hyper without independent payoff evidence.
- Actual IVPBoundaryProbe: all six right cuts at 2/2.5/3/3.25/3.5/4 of a
  two-segment projection pass endpoint/midpoint value checks; no-events/maybe/
  certain/poor-enclosure controls return their expected distinct statuses.
  With EXACTLY constructed binary64 endpoints 1 and 1+2^-52, actual midpoint
  enclosure is [1,1+2^-52] and its chosen lower endpoint is unchanged. Step
  limit 2^-52 returns a maybe-event normally; positive step 2^-60 exhausts the
  256MiB heap instead of returning uncertainty (caught heap-overflow exception).
  Log /tmp/aern-ivp-boundary-probe-exact.log is authoritative. Earlier probe
  drafts used floating (**) to build powers of two after the hardware rounding
  mode changed; those were NOT reliable adjacent-endpoint controls. Corrected
  to encodeFloat dyadics. Intermediate assertion failures are probe-construction
  issues, not additional donor interval-arithmetic bugs.
- Hyper comparison: ordered_field_roots.rs uses exact Real midpoint arithmetic,
  explicit max_subdivision_depth/refinement_steps and recorded exhaustion;
  algebraic_fiber likewise retains exact dyadic midpoint values. The donor's
  fixed-binary64 stagnation pattern is not itself a new defect in those paths.
- Actual IVPEventProbe (public donor APIs): EventNextMaybe with an
  EventInconsistent child returns Nothing, just as EventGivenUp does, while a
  fixed-point child returns the expected retained states. The inconsistent
  child is defined as an impossible branch, so conflating it with unknown loses
  otherwise valid no-event results (completeness). EventTree's ordinary mode
  aggregation uses sequence, correctly propagating missing/unknown results;
  it does NOT have SplitNearEvents's catMaybes behavior.
- Actual public Picard/bisection probe: zero ODE x'=0 with initial parameter
  x=u*(1-u), u in [0,1], and invariant x>=1/8. The sample-only range helper
  yields [0,0] despite value 1/4 at u=1/2. Unwrapped unrestricted solving gives
  Just [0,1/4]; unwrapped invariant-pruned solving returns Nothing; wrapped
  invariant-pruned solving gives Just [0,1/4]. This confirms a completeness
  loss from using endpoint samples for domain pruning, not merely for precision
  heuristics. No false final enclosure/certificate is claimed: the public
  result is failure, and the wrapped output is still a conservative enclosure.
  Log /tmp/aern-ivp-event-probe.log; source /tmp/aern-audit.zp9Mzz/IVPEventProbe.hs.
  Initial probe needed an explicit Double type to resolve associated effort
  types; donor Picard/EventTree/ShrinkWrap compiled without numerical edits.
- Live Hypersolve debug/release gates both pass 795 tests/examples; live
  warning-denying Clippy and fmt/diff checks pass. Hypercurve downstream unit
  stage passes 889 tests, 6 ignored in 645.45s; integration stages still running.
  DURING that run concurrent user changes appeared in Hypersolve algebraic APIs
  and several benchmarks, including certification.rs; Hypercurve advanced to
  93ec28cc6b93df4829ce0a3f991176ed7abb3874. These changes are preserved and not
  attributed to this audit. Gates qualify their compiled starting snapshots,
  not the later edits. Quadratic source/test/new-module hashes remain exact
  candidate matches. Keep the three benchmark-registration lines separate from
  the user's certification.rs changes when staging; index was empty on check.
- Read complete Events/Bisection (144) and Events/PWL (279); cumulative AERN
  223 text files / 42,014 lines. All ten IVP solver/specification source files
  have now been read, but IVP examples, GUI, demos, assets and other remaining
  packages still prevent package/audit completion. PWL is explicitly unfinished:
  encloseOneEvent throws "not fully implemented", function-union returns only
  its first operand, and its ODE record omits fields required by later solvers.
  These are source-visible incompleteness, not a newly qualified implementation.
- CONFIRMED public SplitNearEvents loss of a feasible mode: IVPHybridProbe uses
  two event-free modes with x(0)=1 and unrestricted invariants. Constant mode
  has x'=0; exponential mode has x'=100*x, an everywhere-defined solution.
  With degree4/32-term polynomials and the selected Picard/step effort, each
  solver returns Just {constant:1} for constant alone and Nothing for exponential
  alone. Ordinary EventTree with BOTH modes returns Nothing, correctly retaining
  the unresolved obligation. SplitNearEvents with BOTH returns Just {constant:1},
  omitting the feasible exponential mode. This confirms the catMaybes path can
  publish an incomplete final-state enclosure after a sibling computation fails.
  Source /tmp/aern-audit.zp9Mzz/IVPHybridProbe.hs; actual log
  /tmp/aern-ivp-hybrid-probe.log. SplitNearEvents compiled without arithmetic
  edits; the initial probe needed a concrete HybridIVP P signature.
- Hyper boundary comparison checked CandidateCertificationReport construction
  and all_satisfied, candidate-domain reports, sketch certification and
  PredicateOutcome boolean composition. These retain failed/unknown rows and
  require all active rows certified, while logical short-circuiting preserves
  the appropriate unknown outcome. None uses the donor's missing-mode filter.
  This is a scoped comparison, not a claim that every Hyper aggregation site
  has been audited in this step. No extra Hyper change retained from IVP probes.
- Read eight more IVP files / 964 lines: .gitignore, LICENCE, PicardView.glade,
  Zeno-paper reproduction instructions, demoPicardView, Plot/PicardView and its
  Layout/State modules. Cumulative 231 / 42,978. These files are source/metadata
  coverage, not additional numerical qualification. PicardView explicitly says
  stalled and retains old IVP/EvalOps interfaces; its Glade loader is commented
  out, most handlers are inactive, and State treats an initial-value function
  builder as a value vector. The historical demo likewise uses superseded module
  and polynomial-config names. No new Hyper scalar/performance idea extracted.
  The plot reproduction document's old timings are NOT this audit's benchmarks.
- Read complete Plot/UsingFnView, 909 lines: cumulative 232 / 43,887. Rendering
  aggregates tiny segments into outward range boxes and carries labels/active
  components separately. Several plot-only empty-list/shape assumptions and
  duplicated configuration branches remain; an earlier unconditional guarded
  addPlotVar equation shadows the later scan-variable-aware equation. These are
  source-level UI caveats, not claims about certified numerical output. No new
  scalar/performance transfer; next unread IVP targets are the two large example
  modules, four current CLI demos and saved numerical/bitmap assets.
- CURRENT TURN CHECKPOINT: PROGRESS, not complete or blocked. Quadratic repair
  remains uncommitted in live Hypersolve; source/test/new benchmark module hashes
  still match the isolated qualified candidate. User's ongoing algebraic API,
  benchmark and fuzz edits are preserved. A registration-only index patch is
  prepared at /tmp/aern-quadratic-certification-index.patch and passes
  git apply --cached --check against d897ff0; it has NOT been staged. It contains
  only the two module-declaration lines and one Criterion registration line,
  excluding the user's certification.rs changes. Recheck index/HEAD before use.
- One ongoing tool session to RESUME, not restart: 36975, live Hypercurve
  cargo test --offline --all-features --tests --examples, output redirected to
  /tmp/aern-quadratic-hypercurve-downstream.log. Current completed stages:
  21 suites / 1,359 passed / 6 ignored / zero failures; further region integration
  work is running. Long unit stage 645.45s passed; long fillet integration cases
  subsequently passed too. Read-only host process inspection confirmed active
  CPU work, not a deadlock. Gate started at Hypercurve 9e2796c; current user HEAD
  is 93ec28c. All other tool sessions in this turn are closed.
- Next: finish this same downstream session, record final counts and decide
  final retention/staging of ONLY the quadratic repair. Do not include or undo
  concurrent user changes. Continue the file-by-file AERN queue afterward;
  do not repeat credited files or confuse source coverage with numerical proof.
- Previous turn classified PROGRESS. Resumed session 36975 and confirmed it is
  still live, without restarting. Current Hypersolve HEAD remains d897ff0,
  index empty, audit-owned polynomial/test/module hashes unchanged; concurrent
  algebraic API and shared-benchmark edits remain user-owned and preserved.
- Read complete IVP Examples/ODE/Simple.hs (782); cumulative 233 / 44,669.
  Catalog has exponential decay, exact/uncertain spring systems, damped/cubic
  systems with sensitivity equations, falling-body drag, Lorenz/Roessler and
  Van der Pol examples. Some maybeExactValuesAtTEnd entries are actually
  binary64 exp/sin/cos values converted into the coefficient type. They must
  not serve as independent exact-real test oracles; current audit probes do
  not rely on them. No new scalar architecture transferred from example setup.
  Downstream now has 22 completed suites / 1,459 passed / 6 ignored; path-stroke
  integration stage is ongoing, with zero reported failures.
- COMPLETED downstream gate: session 36975 exited 0. Hypercurve all-feature
  tests/examples finished 46 suites / 1,731 passed / 9 ignored / zero failed;
  log /tmp/aern-quadratic-hypercurve-downstream.log. This qualifies the compiled
  starting snapshot 9e2796cbf36dde2f56087a717b34d31edbf80f51 with the recorded
  dependency pins and quadratic candidate, not subsequent concurrent user work.
  All prior tool sessions are now closed; do not resume or restart 36975.
- RETAINED Hypersolve bb85085438e398b92ef56b0855332fc686403562, parent
  d897ff0a5d416daa16c83f1bfe32334077fd07de: Preserve monomial identity during
  quadratic extraction. Exactly four audit-owned files committed: polynomial.rs,
  seven-test quadratic_extraction.rs, eighteen-case benchmark support module,
  and ONLY three registration lines in certification.rs. Staged diff inspected;
  user's shared-benchmark API edits and all algebraic/fuzz edits remain untouched
  and uncommitted. No push. Source/test/module hashes match the isolated final
  candidate; earlier 795-test debug/release, seeded oracle, Clippy, memory,
  paired timing/allocation/size and now downstream evidence support retention.
  Production source delta +107 lines. Standalone driver text +15,700 bytes
  (file +23,224); exactness/completeness and measured common-path improvements
  outweigh this documented size cost. This is not a whole-library size claim.
- Read the remaining five aern-fnreps files / 436 lines; cumulative AERN
  238 text files / 45,105 lines. All six package files / 485 lines now read.
  The Chebyshev prototype has partial Eq/Ord/Num operations and undefined Main;
  its DCT multiplication is a distinct transform idea, not a production-ready
  implementation. Direct multiplication uses the exact Chebyshev product
  identity. DCT uses a strictly larger power-of-two grid than degree sum (so
  removing that extra padding requires endpoint/Nyquist normalization review),
  logs all intermediate arrays, keeps the entire numerical coefficient tail,
  uses repeated list indexing, and assumes nonempty/nonnegative sparse keys.
  SDCT recurrence explicitly records trial-and-error index adjustments.
  No numerical failure or asymptotic speed claim inferred solely from this read;
  native donor probes and scoped Hyper comparison are next. The cited BT97 paper
  has not yet been opened in this step; source comments are not paper evidence.
- Native DCT qualification completed: DCTProbe.hs uses an independent Rational
  oracle (Chebyshev three-term recurrence -> ordinary monomial convolution ->
  triangular Chebyshev conversion), not the donor's direct-product formula.
  All 69 degree0..32 cases pass coefficient containment for both donor methods;
  six DCT-I reference/recursive overlap controls (N=2..64) pass. Extra benchmark
  operands at degrees4/16/32/64/128 also pass this oracle. These are enclosure
  checks, not a general proof of the SDCT recurrence. Original trace-enabled
  binary/logs: /tmp/aern-audit.zp9Mzz/dct-probe,
  /tmp/aern-dct-oracle.log, /tmp/aern-dct-trace.log (293,796 diagnostic bytes).
  Compatibility only: old DoubleBasis import and old mixed operators/piOut
  mapped to the pinned current DI outward operations by AuditDCTCompat.hs;
  integer-logarithms supplies the same intLog2 module. Pristine donor untouched.
  Actual empty-polynomial control: direct returns empty map; DCT throws maximum
  of empty list. Empty SDCT exhausts a 128MiB heap (caught). Preconditions and
  incomplete interfaces exclude these from any claim of a ready scalar donor.
- DCT transfer screen: three independent CPU6-pinned Criterion runs, 0.3s target
  per case, 1,000 resamples, full NF forcing; audit-only diagnostic trace removed
  via AuditDCTTrace.hs, with arithmetic/representation unchanged. Logs/CSV
  /tmp/aern-dct-bench-{1,2,3}.{log,csv}; DCTBench.hs and separate build directory
  retained under /tmp/aern-audit.zp9Mzz. DCT/direct mean ratios by degree:
  4:108.5..117.6x; 16:26.2..27.8x; 32:7.9..13.2x (third direct sample noisy);
  64:5.05..5.17x; 128:2.336..2.352x. This is a donor screening comparison,
  not a Hyper benchmark or universal asymptotic claim. Allocation regressions
  in runs2/3 also disfavor DCT: direct/quiet-DCT about 126,488/8,327,162 bytes
  at degree4, 1,550,744/22,470,000 at16, 6,157,320/42,690,000 at32,
  24,320,000/84,810,000 at64, 96,610,000/172,200,000 at128 per operation.
  The direct dyadic results have zero width; DCT max width grows from 1.06e-5
  at4 to 1.746 at128 and stores 2d-padding coefficients (513 vs257 at128).
  No transfer retained. Hyper's tensor convolution keeps exact Real coefficients,
  explicit shape/overflow checks, sparse nonzero traversal and square symmetry.
  Its bigint NTT/CRT API is an existing explicit candidate, NOT the default
  backend dispatch. The donor's fixed-interval DCT is not an exact substitute.
- Read seven remaining aern-temp files / 783 lines and both IVP saved .fkt
  plots / 256 lines; cumulative intermediate 247 / 46,144. Temporary function
  abstraction is unfinished (polynomial eval undefined, builder handles only
  literals/projections, several class methods missing). The .fkt files are
  decimal plotting fixtures, not exact arithmetic or independent certificates.
- Actual FnRepProbe against the rounded-backed public zeroUsingTrisection:
  linear f(x)=x-1/2 on [0,1], requested20bits returns anti-interval [1,0], width
  -1, and explicitly does NOT contain1/2. The code takes endpoint intersection
  instead of hull and accepts its negative width before evaluating the function.
  Only the module/export name was changed for isolated linkage; no arithmetic
  modification. Log /tmp/aern-fnrep-probe.log. Its reversed quarter-point order
  is an additional source caveat, not a separately validated repair.
- Actual P01Probe compiles the hmpfr iRRAM-style and SimpleExactReal variants;
  only module/export names and two obsolete mixed-operator spellings changed.
  Exact Rational endpoint-power checks over72 precision/radicand/degree triples
  pass all adaptive Newton results; fixed22-step helper fails13 rough-seed cases.
  Example: 16th root100 with seed101 returns [22.89,24.42], excluding the root.
  These rough seeds bypass the normal preparatory bisection; do NOT attribute
  those13 failures to realRootFinder's seeded path. All16 ordinary end-to-end
  rational examples pass for BOTH wrappers. Logs /tmp/aern-p01-oracle.log.
  Boundary log /tmp/aern-p01-boundary.log: rational benchmark zero-numerator
  shortcut returns0 even with denominator0 (outside valid benchmark domain);
  unresolved thick [1,4] bisection exhausts128MiB while merely raising stored
  precision; SimpleExactReal zero and negative-odd roots hit1.5s timeout.
  The exception catcher renders these as EXCEPTION <<timeout>>. Hyper's
  direct bounded-degree root kernel instead encloses integer powers from the
  child approximation error contract, and handles near-zero requests without
  waiting on equality; no corresponding fixed-iteration transfer selected.
- Read all13 remaining aern-realfn-plot-gtk files /3,630 lines; cumulative
  260 /49,774. Source-complete GUI comparison: transactional data/meta/state
  snapshots, update flags, explicit draw effort, sample-based plot meshes and
  separate inner/outer styling are visualization facilities, not new scalar
  certification. Metadata equality omits group names; the watch path can pass
  undefined function data for metadata/default-point-only updates; zoom accepts
  nonpositive values; empty/gapped domains and failed numeric conversions have
  partial matches. These are source caveats, not GUI-runtime reproductions.
  Sampled outlines must not be promoted to whole-domain enclosure proofs.
  No GUI was opened and no extra Hyper change retained.
- Visually inspected all five AERN binary assets: architecture overview JPEG,
  shrink-wrap PNG, and the three single-page Cairo min/max comparison PDFs.
  PDF metadata and1100px renders retained at /tmp/aern-assets.UK1CnR. Diagrams
  illustrate previously read architecture and approximation offsets; neither
  diagrams nor saved decimal plots are numerical certification evidence.
  Both tracked symlinks resolve to the now-read master FnView.glade. No binary
  bytes are counted as source lines. All tool sessions in this checkpoint are
  closed (including benchmark2341); no outstanding build or test to resume.
- Read all16 remaining aern-poly-plot-gtk files /2,987 lines; cumulative AERN
  276 text files /52,761 physical lines. Both plotting packages are now source
  complete. Demos cover inner/outer inclusion, polynomial integration, logistic
  iteration, reciprocal, min/max, abs, erf and sine/cosine composition. Explicit
  parameterization of uncertainty preserves correlation, but Hyper's immutable
  exact expressions already retain symbol identity; replacing them with rounded
  fixed-degree polynomials would weaken its scalar contract. Several tutorials
  retain superseded operator names and inaccurate labels; visual screenshots
  and commented historical failing seeds are not fresh numerical validation.
  plotPicard's classical path is explicitly nonrigorous, and its piecewise
  iteration returns its final approximation without an inclusion gate. Its
  separate stepwise validated path does check Picard inclusion before endpoint
  propagation. No new Hyper production change selected from these demos.
- FINAL TURN CHECKPOINT: PROGRESS, not complete or blocked. Quadratic fix
  bb85085438e398b92ef56b0855332fc686403562 is retained and fully qualified as
  recorded above. During the audit the user's independent algebraic API work
  was committed as 0d3a33f68c3e2479e88e2aa149bbec23508d4254 on top; current
  Hypersolve worktree/index are clean and all three audit-owned file hashes
  remain unchanged. Do not attribute validation of bb85085's pinned snapshot
  to this subsequent commit. Reference donor is pristine. All processes closed.
  Next AERN source queue is EXACTLY five files /6,133 lines:
  IVP Examples/Hybrid/Simple.hs (3,801), demos/simple-events-locate-polyint.hs
  (614), simple-events.hs (676), simple-ode-polyint.hs (521), simple-ode.hs
  (521). None has been credited or read in this turn. Continue them, finish
  AERN's remaining transfer dispositions, then advance the broader inventory;
  do not treat this near-complete source count as whole-audit completion.
- Read all five final AERN files /6,133 lines: complete Hybrid/Simple catalog
  (3,801), both event CLIs (614/676), and both ODE CLIs (521 each). All281 text
  files /58,894 physical lines are now read; five binary assets and two symlinks
  remain accounted for separately. Full-source coverage is not, by itself,
  disposition of every transfer experiment or completion of the ecosystem audit.
- Hybrid catalog: energy/sum/relative-velocity auxiliary variables and repeated
  invariant intersections preserve useful model correlations. Several reset
  energies deliberately use conservative widening to aid event-tree convergence;
  that is a model/enclosure tradeoff, not evidence of an exact scalar identity.
  Hyper already preserves exact expression identity and proof-gates narrowing.
  No generic scalar replacement selected. The circle model's positive square
  root reconstruction relies on trajectory/quadrant assumptions: no reachable
  counterexample has been established, so do not claim one. Commented obsolete
  models and binary64 transcendental fixtures are not independent exact oracles.
- Event CLI enclosure checks collapse mode/value associations into a mode set
  and a union box, so they do not establish mode-specific containment. All four
  CLIs' zipWith-based vector checks omit a length check. These are source-level
  checker limitations, separate from the prior executed solver probes. PWL
  remains explicitly unsupported; simple-events-locate-polyint is marked
  disabled and uses superseded interfaces. No GUI/CLI execution is claimed.
- ODE CSV timing divides picosecond CPU time by10^9 (milliseconds), although
  the header labels microseconds; it times diagnostic printing and uses one
  sample per parameter. Therefore these CSV timings are not accepted as rigorous
  performance evidence. Undefined substSplitSizeLimit is passed only into an
  ignored argument, not falsely treated as a demonstrated crash. The explicit
  end-time CLI clears its optional exact fixture; legacy CSV mode does not.
  Its side-printing shrink-wrap demo is not a whole-domain containment proof.
  No new production change retained from these final example/demo files.
- Full coverage reconciliation passes:281 unique paths,58,894 physical lines,
  no unread text paths, every range and SHA-256 agrees with the pinned donor
  and inventory. Donor and Hypersolve trees remain clean; Hypersolve HEAD is
  unchanged at0d3a33f. The audit has not modified the user's subsequent work.
- Native HybridExamplesProbe compiles the actual catalog after ONLY hiding
  modern Prelude's conflicting Applicative operator in the isolated copy.
  All24 catalog mode-key/arity/initial-invariant checks pass. Both two-bead
  collision representations pass135 exact-Rational cases each, checking reset
  velocities/energies and containment after invariant narrowing (270 total).
  These are model-boundary checks, not entire-trajectory or reachability proofs.
  Actual donor getHybridStateUnion plus the CLI checker formula accepts swapped
  mode values although per-mode containment fails; its zipWith formula accepts
  a missing output vector. These two checker witnesses do not execute the GTK
  CLIs. Source /tmp/aern-audit.zp9Mzz/HybridExamplesProbe.hs, log
  /tmp/aern-hybrid-examples-probe.log; all checks finished, no new Hyper patch.
- Closed the previously unread ICMS2014 documentation gap: downloaded the
  institutional accepted manuscript of Function Interval Arithmetic (eight
  paper pages plus repository cover), SHA-256
  bf65a944c41ace399b9de8783e152cc1368178bfe6f2a149e3fd14482f5cbd01.
  Read all420 extracted lines and visually inspected all five figures.
  Files /tmp/aern-audit.zp9Mzz/function-interval-arithmetic.{pdf,txt}; source:
  https://hh.diva-portal.org/smash/get/diva2%3A767024/FULLTEXT01.pdf.
  Outer/inner function approximations, directed intervals, effort dimensions,
  polynomial degree reduction and min/max case merging match the audited
  implementation architecture. These target function-space enclosures, not a
  superior exact scalar representation. The paper's generic property-testing
  assurance is not a substitute for the concrete counterexamples above.
- BT97 is a transitive citation in the DCT prototype, not an unread tracked
  AERN file. Its publisher abstract and Rostock preprint entry were checked,
  but publisher full text returned403 and the preprint entry supplies no file.
  No full-paper reading, proof verification, or universal transform-performance
  claim is made. The independently derived oracle and measured donor rejection
  do not rely on adopting that paper's algorithm or accepting its claims.
  https://www.sciencedirect.com/science/article/pii/0024379595006966
  https://www.math.uni-rostock.de/math/pub/preprints/preprints1993_1994.html
- AERN TARGET CLOSURE: source/native qualification/targeted transfer comparison
  complete, with the stated old-toolchain, GUI, unsupported-operation and
  transitive-paper limits. Four Hyper exactness/completeness changes retained,
  each with its own full gates and controlled cost evidence recorded above.
  Generic effort retries, rounded polynomial function spaces, DCT, fixed-count
  Newton and hybrid/GUI runtime patterns yield no further worthwhile present
  scalar change. This does NOT close pending RealLib/cold-storage cross-target
  candidates or the ecosystem audit. Next target: CDAR and its mBound branch.
  All AERN probe/build/download sessions are closed; pristine donor unchanged.

### CDAR and bounded-mantissa branch audit opened

- Pinned official master1919f75c2cacd9f994ad50e925d8932d2639b690 and the
  requested mBound branch132e0da252cb723dc1247f419919bdbfd4072e9e. All paths,
  byte/physical-line counts and SHA-256 hashes are inventoried separately in
  CDAR_FILE_INVENTORY.tsv and CDAR_MBOUND_FILE_INVENTORY.tsv. Master has27
  text files /4,025 lines and one3,604,391-byte executable; mBound has24 text
  files /3,717 lines, the identical executable and two ODS spreadsheets.
  No binary execution or spreadsheet reading has been performed. Both primary
  repository/readme pages were reviewed; their performance claims remain claims.
- Read master README, both manifests, all six library modules and Test.hs in
  full:10 files /2,749 lines. Also read mBound README, manifest and Test.hs
  (three files /507 lines). Separate READ_COVERAGE.tsv files credit only these
  completed files, not merely inventoried or grep-inspected files.
- CR uses ZipList of centered dyadic approximations and a shared80,120,180,...
  resource ladder. Operations zip equal stages; require scans for adequate
  absolute accuracy. Error mantissas are bounded to roughly10bits, while the
  main mantissa can grow with absolute magnitude; mBound advertises an added
  precision-size cap. The latter implementation is still unread. Pairwise
  consistency/convergence are constructor obligations, not checked internally.
  caseCR explicitly permits different branch values across approximation stages;
  this is not a single-valued exact-real contract unless the caller ensures it.
- Main source concerns to qualify: divAInteger omits the compensating exponent
  shift; floor/ceiling ignore positive exponents; poly uses coefficient centers
  but omits coefficient uncertainty and assumes error exponents align;
  reciprocal-square-root endpoint offsets point inward; direct Taylor helpers
  discard tails despite documentation describing an error allowance. Several
  exported alternative transcendental/CR wrappers have inconsistent precision
  signs or reconstruction arguments. These are not yet numerical conclusions.
  Extended's fmap-based negate/abs do not change infinity constructors; mixed
  opposite-infinity addition is explicitly left-biased by design.
- Isolated shared clones and Stack project /tmp/cdar-audit.HQMgJs use the
  existing /tmp/aern2-stack.ryVCFK GHC9.6.7/lts22.44 toolchain. Both UNCHANGED
  libraries and test executables compile; no compatibility source edits needed.
  Stack preserves master's manually edited cabal file and warns about its
  nonexistent README.md extra-source entry. Initial missing-dependency network
  denial was retried with approval; final build log /tmp/cdar-native-build.log.
  BOTH native suites pass all90 tests with QuickCheck replay424242, default100
  samples/property: /tmp/cdar-{master,mbound}-native-test.log. Test wall times
  are not benchmarks. Tests omit exact interval addition/multiplication oracles,
  allow broad/Bottom enclosure results, and the acos endpoint test calls asin.
  Independent boundary probes are now building; no Hyper edit made.
- Actual BoundaryProbe runs against BOTH untouched installed libraries.
  Independent Rational endpoint oracles pass32,400 addition and32,400 product
  cases each,168 error-normalization and672 precision-limiting containment
  cases each. The shared divAInteger helper violates2,562/3,591 positive-divisor
  cases. Master floorA and ceilingA each violate104/189 cases (positive binary
  exponents ignored); these APIs do not exist in mBound and are not claimed
  tested there. Initial mBound probe compile included those absent APIs, then
  CPP excluded only those checks; no donor source was changed.
- Master sqrtRecA40 on[1,9] returns[684/2048,2046/2048], excluding both1/3
  and1; mBound's corresponding endpoint offsets correctly contain both controls.
  Both return a finite range about[0,0.71] for[0,2], excluding reciprocal-square
  roots1 and2 of valid positive points in that input. Exact reciprocal sqrt4
  passes both. This is an actual public helper result, not a claim that every
  enclosing sqrtA caller fails. Both Extended variants leave negate(PosInf)
  and abs(NegInf) unchanged, matching the source concern.
- Both poly helpers collapse uncertain constant[0,2] at exact0 to exact1.
  More importantly, the PUBLIC CR polynomial [1/3] at0 and [0,1/3] at3 each
  return an incorrect zero-error dyadic at requested20/100/500bits. Thus these
  failures are reachable from valid converging exact-real inputs, not only
  hand-built nonconverging interval streams. require accepts the first result
  at every demand because polynomial evaluation erased coefficient uncertainty.
  The exact-coefficient x+x^2 on[1,2] control encloses its[2,6] endpoint image;
  no universal polynomial failure claim is made.
  Source /tmp/cdar-audit.HQMgJs/BoundaryProbe.hs; authoritative logs
  /tmp/cdar-{master,mbound}-boundary-probe.log. All probes completed normally;
  no unsupported donor code was transplanted or Hyper production code changed.
- TURN CHECKPOINT: PROGRESS. AERN source/targeted transfer audit is closed with
  its explicit qualifications; CDAR remains in progress. Master read coverage
  is10/27 text files (2,749/4,025 lines), mBound3/24 (507/3,717 lines).
  Next read mBound Approx.hs and support modules completely, then remaining
  master/branch examples and benchmarks, inspect two ODS artifacts and identify
  the shared executable without running it. Follow with independent elementary/
  convergence checks and controlled storage/scheduling/mantissa comparisons;
  do not treat the passing native suite or its wall time as benchmark evidence.
  Blanck2006 centered-representation paper is linked but not yet read.
  Both donor and trial-clone tracked trees are clean; all build/test/probe
  sessions are closed. Concurrent USER edits have now appeared in Hypersolve
  algebraic_fiber.rs, algebraic_rational_image.rs and root_isolation.rs. These
  are untouched and excluded from audit validation/retention attribution.
- Read mBound's entire Approx.hs (1,882 lines), all five support modules and
  both donors' remaining metadata/license files. Coverage now master13/27
  text files /2,800 lines and mBound12/24 /2,839 lines. These are complete
  file reads, including identical support modules, not diff-only credits.
- mBound rounds discarded mantissa bits into its error term and propagates
  the larger operand budget. This is a candidate representation technique,
  not yet a selected Hyper change. Small-budget cap containment and actual
  storage/throughput need independent measurement. Raw constructors and some
  helpers can bypass cap enforcement; no public-CR cap failure is yet claimed.
- mBound also repairs the positive reciprocal-square-root endpoint offsets
  and adds a next-term allowance to taylorA. Consequently branch speed/size
  comparisons cannot be attributed solely to mantissa bounding. Its nonexact
  exponential and logarithm paths, alternative CR wrappers and nonfinite
  binary64 conversion still need targeted numerical qualification.
- Important precondition nuance: mBound explicitly documents poly as requiring
  exact coefficients. The raw uncertain-constant helper probe violates that
  restriction in this branch. The public CR polynomial wrapper imposes no such
  restriction and passes approximated coefficients to it; the independently
  reproduced polynomial [1/3] 0 failure therefore remains a valid exact-real
  API counterexample. Do not conflate these two claims.
- Completed every remaining example, benchmark and .lll program in BOTH trees.
  Source coverage is now master27/27 /4,025 physical lines, mBound24/24
  /3,717 physical lines. No unread tracked text paths remain. Obsolete BR
  imports/constructor arities occur in retained legacy examples/benchmarks;
  those files are not claimed to compile against the current library.
- Logistic specialization retains dependency by evaluating the quadratic's
  vertex/endpoints, useful as a comparison with Hyper's existing polynomial
  bound machinery. Its shifts assume suitable exponents; orbit1 also uses
  +4*x^2 instead of the documented logistic negative term. The Lll interpreter
  exposes limit/selection assumptions to callers, uses partial parser/stack
  handling, and leaves Entc/Lvc as no-ops. These are example source findings,
  not new failures of the core CR arithmetic.
- Binary-splitting examples include commented-out tail corrections and a
  parallel example that returns four uncombined blocks. The four-way benchmark
  silently drops a remainder when n is not divisible by four (its fixed320000
  fixture is divisible). Existing Criterion suites include shared lazy input
  streams, constant pi retrieval, weak-head-only results and duplicate labels.
  They do not by themselves measure fresh expression evaluation or correctness.
- Inspected both ODS artifacts by reading every populated cell and formula:
  branch comparison693 cells (99rows), optimization comparison288 (96rows),
  metadata dates2019-01-11/2021-05-08. Also viewed both saved thumbnails.
  Extraction/parser retained at /tmp/cdar-artifacts.HgwKA0. Formula columns
  simply flag2x/10x mean ratios; no sample distributions, correctness checks,
  pinned compiler/commit or allocation evidence are stored. Even unchanged
  Double controls differ severalfold across branch columns. Historical results
  are not accepted as fresh controlled benchmarks or mantissa-only effects.
- Shared3,604,391-byte BinarySplit artifact identified without execution:
  x86-64 GNU/Linux ELF, dynamically linked, debug information retained,
  build-id0deae94fd7c000f226fdf6810eba29be6f4ac75a. readelf recognizes its
  GHC link-info section (and warns it is not a conventional ELF note).
  Binary metadata/thumbnail inspection is not source-line credit.
- Author's centered-interval PDF URL fails browser fetch and timed out after
  an approved30-second curl retry; institutional record also failed browser
  fetch. The2006 paper remains unread, not silently replaced by a related
  2008 domain-representation paper. Further primary-source retrieval is pending.
- Independent native ElementaryProbe now runs unchanged donor libraries with
  an MPFR C oracle (512bits, escalating to4096 if ambiguous), exact Rational
  endpoints, exact dyadic inputs, and directed rounding. All1,421 queries per
  donor finished within the overall180-second bound. Individual queries have
  a0.3-second deadline and512MiB heap cap; timeouts and resource exceptions are
  reported separately from numerical exclusions, not claimed nontermination.
  Final logs /tmp/cdar-{master,mbound}-elementary-probe.log; source and oracle
  /tmp/cdar-audit.HQMgJs/{ElementaryProbe.hs,elementary-oracle.c}. Initial
  exception handling caught the timeout token as an exception; the handler
  was corrected and both complete runs repeated. No donor numerical edits.
- Master:1,270PASS,69 numerical exclusions,51timeouts,31exceptions. mBound:
  1,310PASS,37exclusions,2Bottom,50timeouts,22exceptions. These totals combine
  interval helpers, point-valued CR and alternative wrappers; they are not a
  general failure rate. Five samples per finite input interval give witnessed
  exclusions but do not prove whole-interval validity when all samples pass.
  Shared pi wrapper thunks can retain the first exception across demands, so
  its three failing requests are not three independent constructions.
- mBound cap oracle:13,965 signed/boundary/magnitude/budget cases PASS both
  enclosure containment and |m|<=2^mb; each of boundErrorTerm, boundErrorTermMB
  and two limitSize demands passes13,965 containment checks. For budgets
  2/3/5/8/10, each budget passes3,969 additions and3,969 multiplications against
  exact Rational endpoint extrema. Thus the cap mechanism remains a useful
  candidate even though unrelated transcendental paths contain defects.
- Both logA helpers exclude values at14/30 positive interval inputs per
  tested32/100bit setting. In particular log[1,3] returns[197,1223]/1024,
  excluding log(1)=0: the radius estimate uses e/m rather than a whole-interval
  derivative bound. Master expA excludes exp(endpoint) for zero-centered
  [-1/8,1/8] and[-1,1]; mBound passes those controls. Master positive sqrtRecA
  fails14/30 intervals per precision, mBound passes all30. Both sin/cos helper
  sample grids pass; mBound sqrt[0,1/4] returns Bottom, not a false enclosure.
  The earlier possible mBound straddling-zero exponential failure was NOT
  reproduced on this grid and remains an unconfirmed source concern.
- Ordinary CR exp/log/sin/cos tested point grids pass at20/100/300bits. Master
  sqrt(fromRational0) times out and atan(fromRational0) throws a partial-match
  exception; branch equivalents pass. Both asin/acos at0 throw, and endpoint
  +/-1 queries time out. Alternative atanCR excludes6/7 exact input fixtures;
  piCRMachin built on it excludes pi at all three demands. Default pi passes.
  Alternative piMachinCR/piBorweinCR fail resource gates; master sqrtCR times
  out on all four probes whereas branch sqrtCR passes. These are distinguished
  from the default Floating implementations, not generalized to every path.
- Both binary64 import functions in mBound turn +/-Infinity and NaN into
  finite dyadic Approx values, confirming the mantissa-versus-exponent typo.
  Master returns Bottom for them. Finite MAX controls pass representational
  checks in both. fromDouble is an intentionally uncertain, fixed interval
  import; it should not be confused with fromDoubleAsExactValue's exact import.
- Oracle-gated StorageBench completed72 sequential CPU6 ABBA process samples:
  three workloads x raw/explicit-limit policies x threeABBA blocks. SameGHC
  -O2 toolchain, variable runtime inputs and fully forced results, >=0.2CPU sec
  per sample; three exact Rational reference fixtures/width gates per process.
  Both donor libraries remain unchanged. Logs /tmp/cdar-storage-bench.jsonl,
  source /tmp/cdar-audit.HQMgJs/{StorageBench.hs,run-storage-bench.mjs}. Node's
  sandbox subprocess denial was retried with approval, not bypassed.
- Raw12-fold squaring: master midpoint131,073bits versus mBound128; medians
  155.6us versus3.13us, allocated52,921 versus10,826bytes/evaluation. This is
  avoidance of excess exact intermediate work, NOT equal-accuracy evidence of
  a new transcendental algorithm. Explicit precision limiting brings both to
  roughly3.5–6.3us; substantial temporal variation prevents a precise winner
  claim for that control. Raw256-product chains are40.5us versus58.5us;
  explicit-limited chains63.4us versus59.9us (12.1% fewer allocated bytes).
  Raw/limited1024-add chains are18.5%/19.6% slower in mBound with9.7%/11.1%
  more allocation and identical exact output shape. Small-case cap overhead
  is therefore material; no blanket scalar representation transplant selected.
- Storage driver text segments are2,711,530/master and2,740,074/mBound bytes;
  this includes Haskell/runtime/linkage and unrelated branch changes, not a
  projected Hyper binary-size delta. RTS max_live_bytes is GC-sampled and
  misses transient peak integer storage; do not treat it as a peak-RSS result.
- Re-read current Hyper's unit-error approximation contract, single-value cache,
  ln/ln1p guard-precision/range reductions, exact root/endpoint handling and
  IEEE-754 conversion. It already avoids unbounded exact intermediate growth
  through child precision demands, without carrying a per-node arbitrary error
  radius or an infinite approximation list. Its ln kernel requires |x|<1/2
  after certified construction reduction and budgets input error explicitly;
  it does not use CDAR's wide-interval e/m shortcut. Independent matching Hyper
  point/endpoint/conversion probes are building against clean2808bc8; no new
  Hyper production changes have been made or credited in this CDAR pass.
- Institutional record was retrieved successfully via curl and explicitly says
  full text is not available there; publisher and primary repository searches
  found metadata/abstract only. The2006 paper remains an explicit retrieval
  limitation, not a source-coverage gap inside the now-complete donor trees.
- Matching Hyper oracle completed at clean2808bc886db969d487263ea073eb1cf9df562eac:
  5,244 directed-MPFR unit-error checks PASS over exp/log/sin/cos/sqrt/atan/
  asin/acos/pi, including tiny values, endpoint neighbors, large dyadics and
  high-then-coarse cache demands;18 finite/nonfinite Rational/Real conversion
  checks PASS. Log /tmp/cdar-hyper-elementary-oracle.log; complete standalone
  client /tmp/cdar-audit.HQMgJs/hyper-oracle. Initial client compile needed
  an explicit integer shift type; no Hyper source change was needed. These are
  targeted controls, not a new full-suite or downstream qualification claim.
- Extended BOTH exact Rational polynomial probes with8,640 integer-coefficient
  quadratic/interval cases, including vertex extrema and positive/negative
  input exponents. Each donor violates1,800 cases even under mBound's exact-
  coefficient precondition. Example p(x)=-2-2x-2x^2 on[-10,-6] returns[-152,-76],
  excluding p(-10)=-182. The helper drops the derivative-radius product's
  exponent and assumes it matches the value exponent. This is separate from
  the earlier uncertain-coefficient/public-CR defect. Full BoundaryProbe logs
  were regenerated successfully; all previous controls/counterexamples remain.
- CDAR TARGET CLOSURE: complete tracked-source read, actual native suites,
  independent exact/MPFR qualification, artifact review and targeted transfer
  comparison are recorded. Centered dyadics and bounded mantissas corroborate
  Hyper's one-integer, requested-accuracy kernels; replacing them with a variable
  error radius/budget adds metadata without an established consumer benefit.
  Infinite ZipList stages retain prior approximations and force intermediate
  resource demands; Hyper's immutable DAG plus finest-value downsampling already
  supplies reusable refinement without that storage model. No scheduling/cache
  replacement selected. Generic coefficient-center/derivative evaluation is
  weaker than Hyper's retained exact coefficients and scale-aware polynomial
  enclosures; the defective donor helper is not transferred. Parallel factorial
  examples and stale benchmark interfaces add no measured present scalar gain.
- The centered-interval2006 paper remains unavailable through the author URL,
  institutional record and publisher routes inspected; no full-paper/proof
  validation is claimed. Obsolete example interfaces are source-reviewed only;
  no complete legacy-example build or opaque ELF execution is claimed. This
  qualified target closure does not mean every donor API is correct, and does
  NOT close the wider ecosystem, RealLib scheduling or cold-storage candidates.
- TURN CHECKPOINT: PROGRESS. Both CDAR coverage inventories/ranges/SHA-256
  checks reconcile with zero unread text files. Both donor trees and Hyperreal
  remain pristine; no new Hyper patch retained. During this work the user's
  three Hypersolve edits were independently committed; that tree is now clean
  atd9726e1265f3ab9a55121f612e2a35003d410089. This audit neither modified nor
  validated that new commit. All probe/build/benchmark/download sessions are
  closed. Next target is ireal: pin source, inventory paths and begin complete
  source reading, followed by the remaining Haskell targets/comparison harness.
  The overall goal remains ACTIVE, neither complete nor blocked.

### ireal audit opened

- Previous goal turn classified PROGRESS: CDAR complete source/artifact read,
  independent native/oracle evidence and72 controlled storage benchmark samples
  changed the target disposition; no new Hyper patch survived comparison.
- Official ireal repository cloned and pinned at
  a7c83281362c5dc857e97d98b424ec314a216b56 (2025-03-12), version0.2.3.
  IREAL_FILE_INVENTORY.tsv records every tracked path, byte/physical-line count
  and SHA-256:38 text files /3,769 lines plus a158,981-byte9-page PDF. No
  submodules, symlinks or repository-specific agent instructions are present.
  IREAL_READ_COVERAGE.tsv currently credits only README, manifest and changelog
  (90lines), all read in full. The primary GitHub landing page was reviewed.
- Manifest advertises pure-Haskell exact reals/intervals, partial Eq/Ord and
  improved performance; changelog describes convolution-based higher-order AD
  and type-level fixed-decimal enclosures. These remain claims awaiting source
  and numerical qualification. Source/applications/PDF have not yet been read.
- Hyperreal and Hypersolve worktrees were rechecked clean before starting this
  target. Current work is reference inspection only; no production code edited.
- Continued read coverage: all12 library modules, native tests, license/setup,
  the977-line paper source (including text after end-document), FFT/rounded FFT
  and both Clenshaw examples are now fully read and credited:23/38 text files,
  2,674/3,769 physical lines. PDF rendering is complete; visual review pending.
- Unchanged library and standalone native tests compile under GHC9.6.7 in
  /tmp/ireal-audit.Sc4TzO; native tests are running. No compatibility patches
  were required. Build log: /tmp/ireal-native-build.log.
- Source architecture: two integer endpoints per requested binary precision,
  finest-result MVar memoization, domain-dependent guard bits and power-series
  reductions; fixed decimal precision deliberately freezes an enclosure.
  Balanced addition and binomial-convolution higher derivatives are transfer
  candidates, not accepted changes. Boundary nontermination, strict-versus-
  closed endpoint contracts, cache-sensitive Cauchy tests and ordered folds
  need independent probes before assigning defects or transfer value.
- A subsequent worktree check found concurrent user edits in Hyperreal's
  src/real/arithmetic/{linear_algebra.rs,tests.rs}. They are preserved untouched;
  no qualification of those edits is implied by earlier clean-commit results.
- All remaining15 application files are now read completely; coverage is38/38
  text files and3,769/3,769 lines. All9 rendered PDF pages were visually read,
  including both historical timing tables and all mathematical notation. The
  file hashes and complete ranges reconcile against the pinned pristine tree.
- Unchanged native tests PASS:46 properties /4,303 QuickCheck cases (43 random
  properties at100 cases and3 constants). This suite is not an independent
  enclosure oracle. /tmp/ireal-native-tests.log contains the complete output.
- Exact Rational probe:277,500 point add/mul/div checks and144 sum checks PASS;
  7,800 interval construction/abs/power/add/mul/div checks have244 exclusions.
  IntegerInterval multiplication's zero-crossing-left/negative-right branch
  uses l1*u2 as its upper product instead of l1*l2. For example[-3,1]*[-5,-1]
  can return[-5,3], omitting15; division inherits the multiplication defect.
  Closed endpoints occur in4,450 interval checks, separately tracked from
  actual exclusions rather than conflated with numerical failures.
- Independent binomial convolution:3,600 exact-integer list pairs PASS;
  polynomial derivatives:2,499 exact-integer checks PASS. The generic foldb'
  reverses associative noncommutative inputs ("ab" becomes"ba"); foldb keeps
  their order. Addition of finite derivative lists creates an infinite zero
  tail. force30 does not evaluate a deliberately bottom-valued endpoint.
  A deliberately inconsistent approximation fails raw isCauchy but passes its
  memoized version, demonstrating the native test's cache-order blind spot.
  signum(abs0), signum(sq0), and signum(0+-0) each enclose1, not0: the strict
  enclosure assumption used by sign tests is not preserved by those producers.
  Complete probe source/log: /tmp/ireal-audit.Sc4TzO/ArithmeticProbe.hs and
  /tmp/ireal-arithmetic-probe.log. Probe import correction was client-only.
- Directed MPFR oracle(512..4096bits):4,812 point/interval/constant requests,
  including512-bit demands and warm-coarse reuse, yield4,745 PASS,17 certified
  FAIL,34 timeout and16 exceptions; no unresolved oracle result. All exclusions
  are interval sqrt, which floors the upper root. sqrt([1/2,3/2]) at2bits can
  return[1/2,1], excluding sqrt(3/2). Fifty incomplete requests are sqrt0 and
  asin/acos endpoints; the timeout/heap-overflow split is resource-dependent
  and cached cancellations can affect later requests. Boundary partiality is
  acknowledged by the paper; no claim of50 independent numerical errors.
  /tmp/ireal-elementary-probe.log records every non-pass. Exact dyadic inputs
  and rational output endpoints avoid binary64 reference-rounding ambiguity.
- Source-review application limits: IntegralsRounded's success branch literally
  returns1; quadrature's n0 weights/points disagree; inverse DCT divides by0 at
  the documented length2 case. Newton/allZeros lack an explicit unresolved
  result for all non-simple/nonterminating cases. ODE uses fixed/ad-hoc precision
  and imports an optional process-spawning plotting hack; that GUI is not run.
  MoreDigits explicitly disclaims rigorous truncation proofs for some answers.
  Independent executable application controls are running, not yet credited.
- Hyper comparison: existing Real sum APIs already use order-preserving binary
  carry at256 proven terms, with cheaper homogeneous rational/symbolic prefixes.
  Hypercurve's derivative recurrences already avoid exponential symbolic AD,
  but source inspection found machine-word factorial/binomial limits on otherwise
  exact high derivatives. This is a completeness candidate, not a proved Hyper
  regression or accepted patch yet. Concurrent user edits exist in Hypercurve
  bezier_algebraic_image.rs, bezier_moment.rs and rational_bezier_general.rs;
  those files remain untouched by this audit pending overlap inspection.
- Per-file dispositions for all38 text files and the PDF are now recorded in
  exact-real-references/IREAL_FILE_NOTES.md. Numerical qualification is separate
  from source-read completion; ireal transfer follow-up remains in progress.
- Application probe completed against unchanged examples:66 containment checks
  PASS, one FAIL (n0 quadrature of x gives2 instead of0),124 exceptions, no
  timeouts. The124 exceptions are inverse-FFT component checks: modern GHC's
  Complex division calls decodeFloat, deliberately undefined in IReal. This
  is a native compatibility/API limit, not124 independent FFT formula errors.
  Both Double inverse-DCT length2 controls return infinities; the rounded
  constant2 integral on[-1,1] returns1 rather than4. Polynomial integration,
  small exact linear systems and affine Newton roots pass the bounded controls.
  /tmp/ireal-application-probe.log and ApplicationProbe.hs preserve evidence.
- Arithmetic logs were rebuilt/rerun with exact input intervals in each failure:
  132 multiplication plus112 inherited division exclusions. All earlier counts
  and standalone API counterexamples reproduce. Two probe type/import fixes
  affected only audit clients, not donor or Hyper sources.
- Summation experiment:72 CPU6-pinned pilot samples over16/128/512 independent
  or cumulative-prefix sine terms, followed by24 fixed-input control samples
  at512 terms and128 bits. Each mode has4 counterordered control samples,
  >=0.2s process CPU time each; both endpoint integers and a width2 check are
  forced. A runtime IORef read supplies the fixed seed so the whole expression
  cannot float outside the timed loop. All allocations include fresh expression
  construction; this is not a warm-cache query benchmark. Pilot input sequences
  varied with iteration counts, so use fixed-input controls for precise ratios.
- Independent directed-MPFR finite-sum oracle passes all18 mode/shape/size
  controls. It bounds rational-input conversion error using sine's global
  Lipschitz constant, rounds each weighted term and sum outward at1024 bits,
  and checks donor128-bit rational endpoints. Shared-prefix sums are reduced
  independently to sum((n-k+1)*sin(q_k)); agreement is not a donor identity test.
  /tmp/ireal-sum-oracle.log; SumOracle.hs and sum-oracle.c contain the oracle.
- Fixed-input medians for512 independent terms: sum16.137ms /50,027,188B
  allocated; bsum7.529ms /19,456,676B; isum6.322ms /18,705,886B. For cumulative
  prefixes: sum17.807ms /50,946,212B; bsum3,093.667ms /7,795,469,952B;
  isum2,731.410ms /7,653,441,880B. Thus balancing/flat-demand evaluation improves
  this independent workload but is174x/153x slower on this dependent workload,
  with153x/150x allocation demand. This is a donor scheduling result, not a
  measured Hyper regression or license to transplant a generic sum node.
  GC-sampled maximum live heap also grows substantially for dependent bsum;
  it is not peak RSS. /tmp/ireal-sum-control.tsv is the24-sample evidence;
  /tmp/ireal-sum-bench.tsv is the72-sample pilot. SumBenchPilot.hs preserves the
  original varying-input driver, while SumBench.hs is the fixed-input control.
- Hyper public derivative oracle CONFIRMS a completeness gap: the valid
  rational line x(t)=3t/(2+t), y(t)=2x(t), at t=0,1/2,1, produces exact matching
  derivatives through62 but returns Unsupported at requested orders63,64,68,80.
  The45-case grid has33 complete results matching the independent factorial
  closed form and12 incomplete results. The failure occurs while constructing
  machine-word binomials even for zero denominator derivatives above degree1.
  /tmp/ireal-hyper-derivative-baseline.log; standalone public client is
  /tmp/ireal-audit.Sc4TzO/hyper-derivative. No Hyper patch has been applied yet.
- The concurrent user changes were independently committed during this pass.
  Current clean Hyperreal HEAD is5de9a5f6950be06726c2f9d5714222bfd007b7eb;
  Hypercurve is18555bb12fb671fd34ee992ad691fe0dc6559528; Hypersolve remains
  d9726e1265f3ab9a55121f612e2a35003d410089. Rebuilding the derivative client on
  these clean trees is up-to-date; the public baseline reproduces. This is a
  targeted baseline, not full validation or authorship credit for those commits.
  Current rational_bezier_general.rs SHA-256 is
  7eff254897dd6c7cede0fd5bfd74c397380704ba66c7287b515a58c8999841c8.
- TURN CHECKPOINT: PROGRESS. Complete ireal source/artifact coverage, independent
  interval counterexamples,96 summation timing samples and a newly reproduced
  Hyper completeness gap materially advance the audit. All started sessions
  are closed. Donor and production worktrees are clean; no new production patch
  retained in this pass. Next: fix/qualify the order63 derivative blocker using
  retained exact coefficients and denominator degree, test dense higher-degree
  denominators and endpoint helpers, benchmark common low orders and memory,
  and run proportional Hypercurve/downstream gates before retaining any change.
  Then compare the shared-prefix scheduling workload against current Hyper and
  finish ireal's transfer disposition before continuing the remaining references.
  The overall audit goal remains ACTIVE, neither complete nor blocked.

### ireal-inspired high-derivative completeness follow-up

- Previous turn classified PROGRESS: full ireal source read, independent
  counterexamples and controlled benchmarks determined concrete transfer work.
  Hypercurve, Hyperreal and Hypersolve trees rechecked clean before this pass.
- Added three public regression tests before changing production logic: the
  degree-one rational line through128 derivatives, a dense degree68 denominator
  through80 derivatives, and exact polynomial tails/domain/size guards. All
  three fail on the unchanged implementation with Unsupported, reproducing the
  machine-binomial limit rather than relying only on the standalone client.
- The dense case is x(t)=1/(1+t)^68, built from positive Bernstein weights2^i
  and reciprocal affine controls. Its nonzero high denominator derivatives
  exercise actual large coefficients; skipping zero tails alone cannot pass it.
  Expected derivatives use an independent rising-factorial closed form.
- Added a private endpoint-helper regression for zero polynomial tails through
 128 and a pi*t^24 monomial through80, including24! beyond u64. This separately
  probes the endpoint factorial limit and retains exact symbolic coefficients.
  Production code is still unchanged while baseline builds/tests run.
- Baseline endpoint regression also FAILS as expected: even an affine
  polynomial's zero tail through128 returns None because factorial21 overflows.
  The complete public baseline tests use the repository lockfile(num-bigint0.4.6);
  the standalone comparison client resolves0.4.8, held fixed between A/B builds.
- Large baseline builds hit /tmp's user quota, including on escalated retry;
  mount inspection confirms tmpfs usrquota rather than an inode shortage. After
  both interrupted build handles closed, only this audit's generated target
  directory was moved (not deleted) to
  /home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555.
  Sources, logs and clients remain in /tmp/ireal-audit.Sc4TzO. The private
  baseline regression now compiles/runs; optimized baseline build continues.
- Candidate now passes all four public regressions: exact line derivatives
  through128, dense degree68 denominator through80, common transcendental pi
  weight cancellation, and size/domain/pole guards. Both private endpoint tests
  pass, including24!, high zero tails and existing cubic endpoint/Horner equality.
  Denominator iteration is bounded by the last structurally nonzero coefficient;
  genuinely large binomials reuse the existing exact fallback. Endpoint factorials
  use exact Real integers, with fallible output allocation and no tail growth.
- First256 counterordered CPU6 A/B samples cover32 public workloads: four curve
  families, orders1/3/8/32, cold and warm basis state, four samples per variant,
  each>=0.2s. At order32, line evaluation is about27–28% faster and dense8 about
 14–15% faster. Common low orders mostly vary within a few percent; line order1
  is2.8–4.2% slower in this run. Do not hide these small-case costs or treat every
  small measured difference as significant. /tmp/ireal-derivative-bench.tsv.
- A separate extracted-helper control found the naive exact endpoint-factorial
  loop costs roughly20–27% at order3 despite improving order20. An explicit
  factorial prefix [1,1,2,6] was therefore tested independently:48 three-way
  samples restore near-baseline order3 cost while retaining the higher-order
  benefit. That prefix is now in the candidate, with exact cubic coefficient
  tests for requested orders0..8. The initial32 endpoint samples and subsequent
 48 samples are saved as /tmp/ireal-endpoint-bench.tsv and
  /tmp/ireal-endpoint-fast-bench.tsv; these are microkernel, not whole-curve timings.
- Initial Massif line/order32 control shows cumulative allocation/free traffic
 564,912->364,632B and heap peaks98,370->97,850B for warmup plus one request.
  Initial linked driver text grows1,352B before the endpoint prefix; final size
  and memory qualification must use the final candidate. Release rational-Bezier
  tests and all-target/all-feature Clippy are now running. No commit/retention
  decision yet. Untracked benchmarks/checkpoints/2026-09-05-native-polynomial-
  kernel.json appeared independently and is preserved as user work.
- Qualification update: all37 filtered release rational-Bezier unit tests pass;
  all four public regression tests pass again with top-level Curve2 view
  forwarding of128 derivatives included. All-target/all-feature strict Clippy,
  cargo fmt check and diff whitespace check pass on the final factorial-prefix
  implementation. The full all-feature release gate has finished compiling and
  is executing897 unit tests plus integration/doctest binaries; it is not yet
  qualified as passed. /tmp/ireal-hypercurve-release-all.log, session50336.
- Final256-sample public A/B run corroborates the substantial order32 line
  benefit (23–27%) and dense8 benefit (13–17%). Some small controls instead slow
  down: dense8/order1 by6–8%, dense32/order32 by2–4%. Several observations show
  temporal outliers while the broad gate compiles on separate cores; focused
  counterordered controls remain necessary before treating small differences
  as stable. /tmp/ireal-derivative-bench-final.tsv retains all observations.
- Final16 Memcheck controls all exit0 with no errors and no definitely,
  indirectly or possibly lost allocations. Whole-process warmup+one-request
  allocation counts/bytes: line32warm2912/263177 ->2077/176336;
  cubic32warm24145/1014498 ->23405/937537;
  dense8/32warm3928/360147 ->3424/307730;
  dense32/32warm17023/1272764 ->17023/1272763. Order3cold allocation counts
  are unchanged in all four families. The one-byte path-length difference is
  not an algorithmic improvement. Cached global allocations remain reachable.
  Final Massif line32 heap peak97850B and cumulative allocation/free traffic
 364632B match the initial candidate; baseline98370B/564912B. These are whole
  process measurements, not isolated per-request peak memory.
- Final linked comparison-driver size: text2535418->2536858 (+1440B),
  data221016->221064 (+48B), bss824->3432 (+2608B), aggregate+4096B. This
  includes all reachable curve/scalar machinery and is not a crate-only size.
  The baseline binary is preserved; final candidate and pre-prefix candidate
  are separate snapshots. Source comment now distinguishes denominator degree
  from rational-curve derivative order explicitly; no executable change.
- Concurrent user checkpoint was committed independently as5c63b8b, touching
  qualification artifacts only; audit production changes remain the two files
  rational_bezier_general.rs and hypercurve_derivative_completeness.rs. No
  audit commit or retention decision yet. Hyperreal remains clean at5de9a5f.
- Full release/all-feature gate50336 now closes exit0:46 result reports,
 1737 passed,0 failed,9 ignored (including891 unit passes and6 ignored unit
  cases); doctests contain0 cases. This is the proportional complete main
  Hypercurve gate, not a claim to have run wasm/demo builds or other crates'
  whole suites. Existing scalar/lattice/limit/solver/tri dependencies compile
  through the all-feature build; top-level curve consumers execute in this gate.
- Additional72 focused A/B samples (six per variant/case) after compilation:
  dense8/order1 warm+0.45%, cold+1.06%; dense32/order32 warm-0.20%, cold+4.45%;
  line/order32 warm-29.64%, cold-27.77%. All samples retained in
  /tmp/ireal-derivative-focused.tsv. The repeat does not establish a dense8
  low-order regression, but the dense32 cold result remains an explicit cost.
  Retention decision: worthwhile. Eliminating demonstrated artificial exact
  coefficient/factorial limits takes priority, and sparse high-order curves
  substantially improve time/allocation. The dense-case and4096B driver-size
  costs are accepted and recorded, not described as across-the-board speedups.
  Final diff/fmt and strict Clippy recheck precede the scoped audit commit.
- RETAINED Hypercurve commit d978852 (only source helper/recurrence and the new
  public regression file). No push. Final fmt and strict all-target/all-feature
  Clippy rechecks pass. The latter recheck also compiled newly appearing user
  Hyperreal edits in computable/node/algebra.rs and real/arithmetic/tests.rs;
  these are not audit changes and are not covered by the earlier full-suite
  attribution. The derivative A/B executables remain frozen on the recorded
  original clean scalar baseline. Summation experiments now use an independent
  clean local clone at5de9a5f, avoiding a moving scalar source during controls.

### ireal shared-prefix summation transfer comparison

- Re-read current Real sum construction and iterative computable Add evaluation:
  a size-hint threshold256 selects order-preserving binary-carry balancing;
  homogeneous exact/symbolic prefixes collapse without graph growth. Generic
  Add evaluation traverses left first with two child guard bits and publishes
  each node's finest cache. This prevents stack recursion but does not itself
  prove optimal evaluation order for successively shared prefix graphs.
- New independent public client lives in /tmp/ireal-audit.Sc4TzO/hyper-sum.
  It compares explicit left fold, current public sum, balanced forward, and
  balanced reverse evaluation on identical independent sine terms/cumulative
  prefixes. Cold timing reconstructs and destroys each whole graph; warm timing
  queries one retained graph. Both certified dyadic endpoints are materialized.
  An independent directed1024-bit MPFR weighted finite-sum oracle is reused
  through a small separate C executable, not Hyper expression identities.
  The initial driver compile exposed/fixed an unsigned numerator type mismatch;
  no numerical result has yet been credited. Clean pinned-source build running.
- Pinned Hyper baseline passes all24 independent MPFR128-bit controls:
  four sum forms, independent/dependent inputs,16/128/512 terms. Each returned
  certified interval has width2*2^-128. /tmp/ireal-hyper-sum-oracle.log.
- All192 CPU6 timing samples finish (4 samples ×4 modes ×2 shapes ×3 lengths
  ×cold/warm). Cold512 independent medians: left9.447ms, public1.987ms,
  balanced2.004ms, reverse2.019ms. Cold512 cumulative-prefix medians:
  left15.563ms, public1590.284ms, balanced1574.714ms, reverse16.213ms.
  Thus the current public path reproduces a102x scheduling penalty for shared
  prefixes while retaining4.75x benefit for independent inputs. At128 terms,
  the public API still uses a left fold and avoids the explicit balanced path's
 41.138ms versus1.137ms left cost. Warm root-cache controls stay sub-microsecond
  and do not characterize cold graph evaluation. No blanket reversal selected.
- Started an isolated scheduling prototype in a second clone
  /tmp/ireal-audit.Sc4TzO/hyperreal-schedule. A saturated16-bit additive-depth
  hint occupies unused bits of AtomicFacts; it adds no node field or serialized
  value. The iterative Add evaluator requests the deeper child first, retaining
  the identical child precisions and rounding rule. Unknown/deserialized hints
  default to ordinary order; this is an accelerator, never sign/magnitude proof.
  This prototype is not a production patch or a retained result. The clean
  baseline source and executable are separately preserved. Next compare time,
  allocation and small-case construction costs, then exactness/history/serde
  and full scalar gates only if the prototype remains worthwhile.
- Initial scheduling prototype passes the same24 independent MPFR controls.
  A96-observation wall-clock A/B run again removes the roughly100x shared-prefix
  penalty, but common-case observations are noisy during concurrent user scalar
  gates; no fine timing conclusion is drawn from that run. A separate CPU-clock
  driver now records process CPU and wall time, batching64 warm calls between
  clock reads. Fixed baseline/candidate executables isolate the pending runs
  from source edits. /tmp/ireal-hyper-sum-paired.tsv; CPU repeat in progress.
- Two completed Memcheck512-dependent public controls (warmup plus one fresh
  whole graph evaluation) have no errors/leaks: baseline32,506,023 allocations,
 2,679,068,382 total bytes; prototype193,782 allocations,43,467,389 bytes. These
  are cumulative process allocations, not resident/peak memory. Initial linked
  driver text/data/bss1378248/220832/3552 ->1376748/220832/5056 (aggregate+4B).
- All108 construction-only CPU samples finish: rational-add+0.7%, symbolic
  pi+e addition+3.9%, generic-add+1.2%, generic-mul+0.2%, sin construction+4.7%,
  sum2-3.1%, sum8+8.4%, sum32+14.4%, sum256+2.7% by separate medians. Some
  sample groups retain frequency/cache outliers despite excluding descheduling.
  Short-sum overhead merits mitigation rather than being hidden by the large
  dependent evaluation win. /tmp/ireal-hyper-sum-small.tsv.
- Prototype metadata preservation/saturation tests pass2/2. Further independent
  signed-root enclosure, precision-history, concurrent-query and serde omission
  tests have been added to the isolated clone and are being compiled; they are
  not yet production regressions. User Hyperreal edits now also include
  computable/node/tests.rs, so eventual transfer must preserve that overlap.
- All four isolated additive-depth tests now pass, including independent
  integer-square-root enclosures for257 signed/scaled shared-prefix terms under
  cold/ascending/descending precision histories and concurrent queries; serde
  omits the hint and restored zero hints remain numerically correct.
- The96-sample CPU-clock end-to-end repeat confirms baseline1598.092ms versus
  prototype15.867ms for512 dependent terms (~100.7x).512 independent terms
  are2.064ms versus2.075ms (+0.55%);128-term cold independent/dependent controls
  are effectively tied in this sample set. Frequency/cache outliers persist,
  especially in short/warm controls, and CPU-time recording alone is not a
  frequency normalization. /tmp/ireal-hyper-sum-cpu.tsv.
- Additional independent512 Memcheck controls keep exactly58,146 allocations;
  baseline/prototype2,560,256/2,560,255 bytes differ only by path length. No
  errors. Together with dependent controls, this supports reuse rather than
  blanket work inflation on the two audited DAG shapes.
- A refined prototype reads additive depth and the already-required inverse-
  trig-presence flag together, avoiding duplicate constructor atomic loads.
  The original eager-depth variant remains as saved executables and patch
  schedule-eager-v1.patch. The combined-read variant passes24 MPFR controls and
  the FULL isolated release/all-feature scalar gate:820 passed,0 ignored,
 0 failed across15 reports (721 unit cases and24 doctests included). Full
  all-feature/all-target strict Clippy and fmt checks also pass. This is a
  pinned-clone gate, not validation of concurrent production user changes.
- All162 three-way construction CPU samples finish. In this noisy run, sum8
  medians baseline540ns /original600ns /combined553ns, and sum32 baseline4061ns
  /original4516ns /combined4005ns. Other small groups traverse markedly different
  frequency bands; do not interpret their separate medians as stable effects.
  Hardware performance counters are available without escalation.108 paired
  counter samples now measure user-mode instructions and cycles per completed
  construction, keeping original CPU/wall observations too. No scheduling
  retention decision yet. New production Hypercurve bezier_offset.rs edits are
  concurrent user work and remain untouched.
- Completed108 hardware-counter controls (six per variant/case) normalize
  instructions/cycles by the driver's completed construction count. Most cases
  add about1.5–2.0% instructions; rational-add+0.5%, sin construction+7.8%.
  Median cycles: sum8+6.2%, sum256+3.4%, sum32-0.3%; the other sampled cycle
  medians do not regress. Startup is included, so these are normalized whole
  driver counters, not selectively bracketed instructions. Full CSV records
  are /tmp/ireal-audit.Sc4TzO/perf-small-1.csv through108.csv, joined by
  /tmp/ireal-hyper-sum-counters.tsv. The construction costs are accepted in
  light of the100x evaluated shared-prefix improvement and98.4% allocation-byte
  reduction. This is not a claim of universally faster scalar construction.
- Final combined-read CPU driver size: text1378608->1377036 (-1572B),
  data220856 unchanged, bss3160->4728 (+1568B), aggregate-4B. Object layouts
  remain within existing56B Node/16B Computable/40B Approximation tests.
- The combined-read candidate is now applied to production for final workspace
  validation. To preserve concurrent node/tests.rs edits, its four private
  regressions live in NEW node/additive_depth_tests.rs, included from node.rs;
  production changes otherwise touch only representation.rs and
  approximation_queries.rs. No user algebra/tests changes were copied or
  modified. An unused import carried over during test extraction was removed
  immediately; the new test file was formatted independently. Workspace
  release/all-feature gate63063 and all-target/all-feature Clippy are running.
  No scheduling commit yet; no broad goal completion or blocker claimed.
- Workspace release/all-feature gate63063 closes exit0:822 tests pass (723
  unit cases,24 doctests included),0 ignored/failed. Its compile log retains the
  subsequently fixed unused-import warning; strict workspace all-feature/
  all-target Clippy then closes exit0 without warnings on the corrected source.
  Production candidate remains uncommitted while additional scale-sensitive
  performance controls run: raw additive depth ignores Offset precision shifts,
  so exponentially decaying shared prefixes may reverse the beneficial demand
  order. This is a concrete hypothesis being tested before final retention,
  not a newly proven defect. The driver is hyper-sum/src/bin/scaled.rs; saved
  baseline/depth binaries use the same new driver and pinned scalar snapshots.
- SCALE TEST REJECTS DEPTH-ONLY RETENTION. All24 exploratory CPU samples finish
  for512 prefix terms scaled by2^(-step*(i-1)), steps0,1,2,4,8,-2. At step4,
  baseline3.864/3.874ms versus depth17.310/17.413ms (~4.5x slower); at step8,
  baseline3.850/3.830ms versus depth16.273/17.094ms. Step2 is roughly tied;
  unscaled/decreasing-step1 and growing-step-2 cases benefit substantially.
  /tmp/ireal-hyper-sum-scaled.tsv. These are timing probes with certified output
  materialization/positivity checks, not yet an independent scaled-sum oracle.
- The conditional retention decision above is SUPERSEDED: raw additive depth
  ignores binary Offset shifts, which can make a deeper prefix need less—not
  more—precision. This repeated-work regression is material despite passing
  numerical/full-suite gates. ALL audit scheduling edits have been removed from
  production with a scoped reverse patch. representation.rs,
  approximation_queries.rs and node.rs exactly match their tracked originals;
  the audit-only additive_depth_tests.rs is removed. Its tests and both candidate
  variants remain recoverable in the isolated clone/saved patches. Only the
  user's algebra.rs, node/tests.rs and real/arithmetic/tests.rs changes remain
  in production Hyperreal. No scheduler commit was made.
- Next candidate, still UNIMPLEMENTED: retain a signed saturated linear
  precision-demand hint, using max(child hints)+2 at Add, the same hint at
  Negate, and child hint+binary shift at Offset. Kernel boundaries can use0.
  Use it only for evaluation order, never to reduce requested accuracy or
  infer numerical facts. It needs scaled-prefix MPFR qualification, both
  dependency directions, saturation/serde/abort/thread checks, fresh common
  construction/counter controls and proportional full gates before transfer.
  Consider this alongside the still-pending RealLib difficult-child scheduling
  candidate; do not mark ireal transfer closure complete yet.
- TURN CHECKPOINT: PROGRESS. Retained Hypercurve d978852 after independent
  high-order exact closed forms,1737-test release/all-feature gate, strict
  Clippy/fmt,584 public timing samples,80 endpoint controls and16 Memcheck
  controls. A separate Hyper summation comparison reproduced a102x dependency-
  order bottleneck and qualified two depth-only prototypes extensively, then
  rejected both after scale-sensitive controls found a4.5x counter-regression.
  Their experimental production edits were fully removed; source, patches,
  executable snapshots and evidence remain in /tmp/ireal-audit.Sc4TzO.
- ALL started tool sessions are closed at this checkpoint. Hyperreal has only
  the three recorded concurrent user-edited files; Hypercurve has the retained
  derivative commit plus the user's bezier_offset.rs edit. No audit scheduler
  change is staged or committed, and nothing was pushed. Next: test a signed
  precision-demand scheduling hint including Offset shifts in the isolated
  clone, qualify scaled sums independently, then finish ireal transfer closure
  and proceed with remaining Haskell/reference targets and cross-reference
  candidates. Overall audit remains ACTIVE, neither complete nor blocked.

### ireal scale-aware demand scheduling experiment

- Previous goal turn classified PROGRESS. Current worktrees rechecked: only
  the recorded three user Hyperreal files and Hypercurve bezier_offset.rs are
  dirty. The retained derivative commit is unaffected; no audit scheduler code
  remains in production. Both rejected depth variants and their evidence are
  present in the isolated trial tree and saved patches/executables.
- Implemented signed saturated linear precision-demand ordering in the isolated
  clone only: Add uses max(child)+2 guard bits, Negate preserves demand, Offset
  adds its binary shift with checked saturation, other kernels use0. Both child
  approximation requests remain unchanged. The16-bit hint shares the existing
  atomic word and is excluded from serialization and numerical proof facts.
  Building the new comparison binaries; tests must be updated to signed demand
  semantics before claiming any new qualification.
- Signed hint preservation, positive/negative saturation, Offset shift accounting,
  independent257-term signed-root enclosures with cold/ascending/descending and
  concurrent histories, and serde omission tests all pass4/4 in the isolated
  release/serde build. No production scheduler edits applied.
- The24-sample forward pilot removes the depth-only counter-regression: step4
  demand4.019/4.031ms versus baseline4.472/4.131ms; step8 demand4.069/3.897ms
  versus baseline4.113/3.890ms. Unscaled demand15.907/15.728ms versus baseline
 1625.156/1559.346ms; growing step-2 demand42.247/42.101ms versus baseline
 6767.405/5499.920ms. Pilot timings are suggestive, not final retention data.
- Added an independent scaled-prefix oracle: exact GMP suffix weights convert
  the expression to a weighted finite sine sum; MPFR uses directed rounding and
  a global sine Lipschitz bound for input conversion. Precision2048+abs(step)*n
  handles absolute accuracy even for exponentially growing values. The driver
  supports both input orders and checks achieved interval width.126 cold oracle
  cases over16/128/512 terms, steps-4/-2/0/1/2/4/8, both orders,32/128/256 bits
  are running. Source scaled-sum-oracle.c and hyper-sum/src/bin/scaled.rs are
  preserved with the distinct before/depth/demand executable snapshots.
- All126 scaled MPFR oracle cases PASS; all24 unscaled finite-sum MPFR
  controls PASS. The isolated release/all-feature Hyperreal suite closes with
 820 passes,0 failures/ignored across15 reports. Strict all-target/all-feature
  Clippy, cargo fmt check and git diff check also pass. These are pinned-clone
  results, not yet validation of concurrent production edits.
- All96 paired scaled process-CPU samples complete (four per variant,512
  terms,128 bits,both orders,six scales; ABBA-style ordering on CPU6). Forward
  medians baseline->demand: step-2 5442.356->41.436ms; step0
 1563.101->15.764ms; step1 517.103->6.239ms; step2 3.885->3.996ms;
  step4 3.843->3.845ms; step8 3.834->3.882ms. Reverse step4 improves
 17.448->3.799ms and reverse step8 16.261->3.824ms; other reverse controls
  range-2.2% to+2.0%. This removes the rejected depth-only4.5x regression;
  it does not establish universally faster evaluation. Full CPU/wall records:
  /tmp/ireal-scaled-demand-bench.tsv.
- All108 small-construction counter controls complete (six per variant/case).
  Normalized whole-driver instruction deltas are+0.36% to+0.81% for rational,
  symbolic and generic addition, multiplication and sums2/8/32/256. Sine
  construction costs+7.81% instructions,+1.27% median cycles; other cycle
  medians range-0.96% to-5.07%. Startup and frequency/cache effects remain
  included. Evidence: /tmp/ireal-demand-small-counters.tsv and the108 joined
  perf-demand-small CSV files. These modest costs are acceptable for the
  demonstrated evaluation/allocation improvement; no new object field exists.
- All8 paired Memcheck controls close with0 errors and0 definitely/indirectly/
  possibly lost bytes. Reachable process-global caches are explicitly retained.
  For256 unscaled forward terms (warmup plus one fresh graph), cumulative
  allocations fall5,755,144->67,733 and bytes324,212,233->10,231,893.
  For512 decaying step4/8 forward terms, allocation counts are identical;
  the12-byte total difference is executable path length. Step4 reverse falls
 53,436->27,138 allocations. This measures cumulative allocation, not peak
  memory. /tmp/ireal-demand-memory.log and demand-mem-* logs preserve results.
- Same unscaled CPU-driver linked size: text1378608->1377212 (-1396B),
  data220856 unchanged,bss3160->4552 (+1392B),aggregate-4B. Existing
  Node56B/Computable16B/Approximation40B layout bounds pass. No crate-only
  binary-size reduction is claimed.
- CONDITIONAL RETENTION DECISION: the signed scale-aware candidate is worthwhile
  on the qualified shapes and will be transferred for final workspace gates.
  The raw-depth variants remain rejected. Only representation.rs,
  approximation_queries.rs,node.rs and a new separate linear_demand_tests.rs
  will change; user algebra.rs,node/tests.rs and real/arithmetic/tests.rs stay
  untouched. No scheduler commit or broad audit completion yet.
- Workspace candidate is applied in those four audit files. Added a fifth
  regression for aborted evaluation across both operand orders and signed
  scales: no aborted node publishes a cache, hints survive, and resumed
  evaluation agrees with an independently constructed coefficient expression.
  A test-only method-name compile error was corrected before qualification.
  Full workspace release/all-feature Hyperreal gate passes823 tests,0 ignored/
  failed (724 unit cases,24 doctests included); strict all-feature/all-target
  Clippy passes. Downstream five-crate gates are running. Hyperlattice's first
  build hit the sandbox's read-only compiler cache; an approved outside-sandbox
  retry is running. Concurrent Hypersolve algebraic_fiber.rs edits were found
  and are also excluded from this audit's patches.

### Haskell exact-real / Data.CReal audit opened

- Official repository expipiplus1/exact-real cloned unchanged and pinned at
 7bb2abaed01ac874d914c99919e81cfd00dd7d6d (2026-06-11). Official GitHub
  and Hackage pages reviewed. It still identifies as version0.12.5.1; GitHub
  HEAD includes newer source/build changes, so the2021 published artifact is
  not silently equated with this snapshot.
- All36 tracked text files are inventoried with bytes, physical lines and
  SHA-256 in HASKELL_EXACT_REAL_FILE_INVENTORY.tsv. No binary/symlink/submodule
  or nested AGENTS.md was found. All source, test, benchmark, build, workflow,
  license and prose lines have now been read; per-file dispositions and native/
  independent numerical qualification are next. Source reading is not yet
  full transfer closure.
- Initial comparison: a locked single-finest Cauchy cache resembles Hyper's
  retained cache, but holds its MVar while computing. Small constructor cache
  seeds and square-specific budgeting merit checking against current Hyper.
  Documented precision-limited Eq/Ord/conversions cannot replace exact evidence.
  signum and atan2 make precision-local branch decisions inside retained real
  objects; independent cold/history checks are needed. Converge stops on
  adjacent approximate equality/error stagnation without a tail modulus, so
  it is not by itself a general certified limit constructor. Native benchmark
  expressions are retained across repetitions and can mostly time warm caches.
- Native source builds unchanged in isolated /tmp/haskell-creal-audit.rafb8h
  with the existing GHC9.6.7/LTS22.44 environment. All499 native tests pass
  (seed142857,two threads,10s per-test limit; ordinary100-case properties),
  and all29 doctests pass. Modern compiler warnings are preserved. Missing
  test dependencies required an approved network retry. An initial default
  Stack-root probe began a redundant compiler download; it was interrupted
  after locating the already installed /tmp/aern2-stack.ryVCFK environment.
  No donor source changes or dependency-contract patches were needed.
- Independent ArithmeticProbe.hs passes337,456 exact Rational comparisons
  across addition/subtraction/multiplication/min/max, reciprocal,square,
  negate/abs,binary shifts and changing precision histories. The comparison
  checks abs(integer approximation - exact value*2^p)<=1, not donor Eq.
- Confirmed retained-value failures: signum(1/1024) at precisions2,16,2 gives
  integers0,65536,4; its first[-1/4,1/4] enclosure is disjoint from the later
  near1 enclosure. atan2(1/1024,1/1024) similarly gives0,51472,3, excluding
  the true pi/4 at the first precision and changing the repeated coarse result.
  Fresh root caches test both histories; this is not merely imprecise display
  or the documented approximate Eq. Converge([0,0,1,1,...]) gives0 at128 bits
  although the actual limit is1. Reciprocal/log zero hit the512MiB probe heap
  limit; negative sqrt raises its documented error. /tmp/haskell-creal-arithmetic.log.
- MPFR point-kernel probe covers20 general/bounded operations, dyadic grids,
  tiny/boundary/large inputs and cold-root/ascending/descending histories.
  Its independent C oracle needs PIC under this host linker; the client-only
  compile flag is corrected and the build is running. No MPFR pass claimed yet.
- All15,379 directed MPFR checks now PASS, with0 failures,timeouts,exceptions
  or unresolved cases. This includes20 general/bounded point operations plus
  pi at0/1/2/8/53/128/512 bits, tiny and endpoint dyadics, large allowed
  magnitudes and fresh-root/ascending/descending histories. The C oracle checks
  exact dyadic import and both directed endpoints against the claimed one-unit
  enclosure. This does not rehabilitate the separate signum/atan2/Converge
  failures. /tmp/haskell-creal-elementary.log.
- Current Hyper comparison read: certificate-bearing sign APIs and checked
  atan2 explicitly preserve Unknown/Exhausted and do not turn unresolved signs
  into axis values. Specialized Square uses one child approximation and an
  adaptive certified magnitude; exact/scaled square constructors already
  collapse structure. Donor bounded helpers and fixed guard formulas are not
  an evident completeness improvement. Cache-seeded unary construction and
  current cold/warm kernel costs still need targeted measurement before this
  donor's transfer disposition closes.
- Scheduling downstream gates completed so far: Hyperlattice200 passes after
  compiler-cache retry, Hyperlimit361, Hypersolve794 and Hypertri191, all with
 0 failed/ignored. Hypercurve's all-feature release build remains active; its
  two checked compiler processes are consuming CPU, not stalled. No scheduler
  commit yet. No audit changes made to downstream source in this follow-up.
- Completed64 CPU6 donor cache diagnostics: four samples each of seeded versus
  explicitly cleared result caches for negate,abs,integer-add,bounded square
  at128/512 bits. All inputs are independently qualified sin(17/32), warmed
 16 bits beyond output accuracy. IO reads keep each constructor application
  inside the measured loop; process CPU batches exceed200ms and RTS total
  allocation deltas are normalized by completed calls.
- Seeded512 medians for negate/abs/integer-add/square are555/317/620/524ns,
  versus cleared1217/1065/1393/902ns. Respective allocated bytes per call:
 1819/1387/2235/2691 versus3283/2931/4019/3859. This is only a diagnostic:
  clearing deliberately adds another MVar and donor unary callbacks bypass the
  original wrapper cache. It is NOT a fair isolated cache-policy A/B or a Hyper
  speedup prediction. Hyper's existing Negate traversal already uses its child
  cache, so transferring eager cache copies requires a separate current-Hyper
  measurement including unevaluated construction costs. Evidence:
  /tmp/haskell-creal-cache-bench.tsv,CacheBench.hs,run-cache-bench.sh. No new
  production cache-seeding change selected.
- Scheduling downstream qualification closes: Hypercurve1737 passes,9
  pre-existing ignores,0 failures. Across all six named crates, completed
  release/all-feature runs contain4106 passes. The first five-crate script
  exits1 only because its original Hyperlattice compiler-cache build failed;
  the separately completed approved retry passes200 tests. All other driver
  sessions are closed. Full evidence remains in /tmp/ireal-demand-workspace-*.log.
- Concurrent user Hyperreal edits were committed as55e142f while the audit
  gates ran. That commit changes only algebra.rs,node/tests.rs and
  real/arithmetic/tests.rs, and exactly matches their recorded diff hash
  be3991259628a012e194ff9790b352cefc39590afc358dadd07b62842372eba3.
  Only the four audit scheduling files are staged. A fresh scalar all-feature
  release gate on the combined state is running before audit commit; no user
  changes will be included in that commit. This preserves provenance rather
  than attributing concurrent source/test evolution to the scheduler.
- FINAL SCALAR RECHECK closes exit0 on the combined55e142f state:824 passes,
 0 failed/ignored across15 reports. Strict all-feature/all-target Clippy,
  cargo fmt check, direct new-test-file rustfmt check and diff checks pass.
  /tmp/ireal-demand-final-hyperreal.log and /tmp/ireal-demand-final-clippy.log.
- RETAINED Hyperreal commit21e76ead8351f5f770f9c97165606ec969310cf9:
  scale-aware Add evaluation ordering and five private regression tests in
  exactly four audit files. Parent is user commit55e142f; no user file is in
  the audit commit. Hyperreal is clean after commit. No push was performed.
  This closes ireal's shared-prefix transfer follow-up, not the broader
  RealLib nonlinear scheduling/cache-release candidates or ecosystem audit.
- TURN CHECKPOINT: PROGRESS. A material~99x shared-prefix scheduling benefit
  is retained after independent150 MPFR controls, signed-root exact enclosures,
  saturation/metadata/serde/thread/abort tests,96 scale-sensitive paired CPU
  samples,108 construction counters,8 Memcheck controls, linked size/layout
  checks,4106 cross-crate test passes and the final824-test scalar recheck.
  The prior Hypercurve derivative fix remains retained; depth-only scheduling
  remains rejected and recoverable in the saved trial artifacts.
- The next donor, Haskell Data.CReal, has complete36-file/2466-line coverage,
  reconciled hashes/ranges,499 native tests and29 doctests passing,337456
  independent arithmetic checks and15379 MPFR checks passing. Separate
  signum/atan2 history and false-convergence failures are confirmed.64 donor
  cache diagnostics are complete but do not justify eager cache transfer to
  Hyper; current-Hyper cache/constructor comparisons remain pending. Continue
  there, then the remaining Haskell and later reference tiers.
- ALL sessions started in this turn are closed. Sources/probes remain in
  /tmp/ireal-audit.Sc4TzO and /tmp/haskell-creal-audit.rafb8h; Rust target stays
  in workspace .audit-targets/ireal-derivative-18555. Hypercurve still has its
  user's bezier_offset.rs edit; Hypersolve's earlier user edit is now committed.
  No audit work is staged/uncommitted in Hyperreal. Overall goal remains ACTIVE,
  neither complete nor blocked.

### Data.CReal constructor-cache transfer follow-up

- Previous goal turn classified PROGRESS: retained scale-aware scheduling,
  completed Data.CReal source coverage and independent numerical qualification,
  and identified specific remaining transfer measurements. Current Hyperreal
  is clean at21e76ea; Hypercurve/Hypersolve user edits are now committed.
- Two isolated Hyperreal clones at21e76ea will compare existing lazy child-cache
  reuse with eager unary cache seeding. Production is untouched. Include
  retained warm inputs, unqueried constructors, fresh/cold inputs and requests
  finer than the transplanted approximation; measure process CPU and allocations.
  A donor benefit alone cannot justify eager work in Hyper's different cache
  architecture. Strong scalar/consumer gates remain mandatory only if a
  worthwhile candidate survives these scoped tests.
- The isolated seed prototype copies valid Negate/Offset approximations and
  derives a safely rounded Square seed from the child approximation's exact
  integer bit length; checked exponent conversions skip unrepresentable seeds.
  The linked before/seeded clients each pass216 independent directed MPFR
  controls through1024-bit output, including fine/coarse histories. Neither
  the production source nor its retained scheduler is modified.
- Running480 paired CPU6 observations (six operations,five construction/query
  phases,two precision bands,four samples per variant).120 allocation controls
  are complete: eager warmed Negate construction adds two allocations and80B
  at128 bits before any result is queried, while cold/unprimed controls preserve
  counts. Full disposition awaits completed paired timings and seed-invariant
  tests. Sources and frozen binaries: /tmp/haskell-creal-audit.rafb8h/hyper-cache*.
- Independent display checks close another native-suite gap:16,665 tests of
  showAtPrecision on exact rational and valid upward/downward Cauchy procedures
  yield128 advertised-error failures. For valid upward approximations to
 1/1024,6-bit display prints0.02, whose error exceeds1/64. The standalone
  rationalToDecimal kernel passes10,945 exact half-decimal-unit bounds; the
  issue is composing an already one-unit Cauchy approximation with another
  rounding step without guard accuracy. No donor formatter transfer selected.
  ConversionProbe.hs and /tmp/haskell-creal-conversion.log preserve evidence.
- Both seed-invariant tests pass, including900 independent exact endpoint
  checks across positive/negative/zero cached integers and positive/negative
  precisions, plus unrepresentable exponent cases. All480 paired timings and
 120 allocation controls are complete. Eager seeding speeds immediate warm
  Negate/Offset/Square queries by about11–23% at128 bits and14–17% at512,
  but unqueried warmed Negate construction costs+77–80%, warmed Square
  construction+290–486%, with additional cache allocations and integer work.
  Finer-than-seed requests do not show a reliable benefit and include slowdowns.
  Frequency/cache noise remains visible in controls; material constructor costs
  repeat at both bands. /tmp/haskell-creal-hyper-cache-bench.tsv and -alloc.tsv.
- REJECTED generic eager cache seeding. It trades Hyper's demand-driven cheap
  construction for speculative cache allocation/squaring before any query,
  despite already reusing the original child's approximation at evaluation.
  No measured consumer justifies that broad policy. The trial is preserved only
  in the isolated clone; production remains clean at21e76ea. No full consumer
  gate or production patch is warranted for this rejected experiment.
- Linked same-client text grows1832B, data unchanged,bss shrinks1856B,
  aggregate-24B; no meaningful binary-size win offsets the runtime/allocation
  tradeoff. Baseline client216 MPFR controls and candidate216 all pass.
  A baseline-only public semantic comparison is building to close decimal and
  tiny atan2/sign counterparts without changing Hyper production code.
- The baseline-only semantic client passes2345 exact-rational decimal error
  checks,120 independently MPFR-enclosed tiny-coordinate quadrant/history
  checks (scales through2^-2500), and20 certificate-bearing signs at a coarse
  refinement floor. Client-only Rational API/type mistakes were corrected
  before this passing build; production code remains untouched. Hyper's
  formatter already reserves extra precision before decimal rounding, unlike
  the donor's one-unit-to-decimal composition. Output:
  /tmp/haskell-creal-hyper-semantics.log; source hyper-cache/src/bin/semantics.rs.
- Data.CReal targeted transfer closure is COMPLETE. General approximate Eq/
  sign/limit policies are rejected as exact substitutes; magnitude/square/
  range-reduction methods are already represented in Hyper; eager unary cache
  seeding fails the measured tradeoff. No extra production change survives.
  Remaining ecosystem targets and unrelated cross-reference candidates remain
  fully in scope. All Data.CReal trial processes are now closed.

### haskell-fast-reals audit opened

- Official repository cloned and pinned at8fa09b2457d7099c75b7db62895b5c8f3391d9e3
  (2017-04-19). Its official README warns that the required custom MPFR binding
  depends on GHC7.8.4 internals. All24 tracked paths are inventoried:23 text
  files and one ELF benchmark artifact, identified without execution. No
  AGENTS.md, symlink or submodule is present. Read coverage is in progress.
- Main source coverage is now COMPLETE: all 23 text files / 2,954 physical
  lines, plus the identified unexecuted ELF. A truncated lecture-note read
  was repaired with an explicit 570–860 reread. Inventory bytes, hashes and
  every full read range reconcile. Per-file dispositions are in
  exact-real-references/FAST_REALS_FILE_NOTES.md and its inventory/coverage TSVs.
- The declared native build cannot construct a GHC 9.6.7 plan: base must be
  <4.8 and integer-gmp <0.6. The initial unsupported --offline invocation was
  corrected before collecting the real build-plan failure. No altered ABI or
  relaxed bounds are called a native pass. The original generic modules do
  compile unchanged and support independent semantic/numerical probes.
- 348,720 independent exact-Rational directional checks: 26,826/86,490
  failures each in dyadic add/sub, 25,596/86,490 in multiply, 855/4,650 in
  scaling; direct division passes 83,700 and inverse 900. Proper interval
  add/sub/mul each pass 2,025 small endpoint checks, while 228/900 interval
  divisions fail even at 128 bits due to signed normalization. These are
  enclosure failures, not just slow convergence or unusual equality semantics.
- Interval abs never tightens its zero lower bound for nonzero point inputs:
  32/36 refinement failures. Native generic checks also expose the default
  appMul2 arithmetic error, list-stage p+1 indexing, and unsound first/second
  Lipschitz bounds; finite Sierpinski forcing controls pass. These methods
  cannot replace Hyper's certified scalar, magnitude or derivative contracts.
- Separate stock-MPFR POLICY reconstructions (not historical native binding
  execution) produce 36/90 invalid frozen integer points and 580/1,542 inward
  abs upper bounds; MPFR's directed conversions themselves pass all 90.
  The declared appGetExp bound fails 256/257 powers-of-two checks. Evidence:
  /tmp/fast-reals-pure-probe.log, -semantic-probe.log and -mpfr-policy.log.
- Required backend comius/haskell-mpfr is pinned at b5d91ca5. Its 503 tracked
  paths include 478 vendored-MPFR files. Exactly 19 files / 3,039 lines of
  required binding/build/allocation context were read and recorded in
  FAST_REALS_BACKEND_READ_SLICE.tsv; no whole dependency/vendor coverage is
  claimed. No old binary or unsafe ABI reproduction was executed.
- Forty-eight complete pinned process-CPU/cache-storage observations show
  retained-list lookup rising from 77.76 ns at stage 64 to 103,877.55 ns at
  stage 65,536, versus roughly 14.3–14.7 ns for the function control. List
  live storage grows to 3,674,160 B even though only neighboring final stage
  results are demanded; function storage remains about 3.9 KB. Benchmarks
  force and checksum results, alternate variant order, separate setup/query,
  and flush allocation accounting outside timing. This measures indexing,
  not numerical throughput. /tmp/fast-reals-stage-bench.tsv contains all rows.
- Baseline Hyperreal 21e76ea passes 79,705 independent targeted controls:
  810 large-integer/history, 2,150 exact Real::abs, 57,680 dyadic-scale/history
  and 19,065 rational arithmetic/history. Source/frozen binary and donor
  probes are preserved under /tmp/fast-reals-audit.bwgyTf and the existing
  /tmp/haskell-creal-audit.rafb8h/hyper-cache client; production is untouched.
- REJECTED list-of-all-precisions cache transfer; Hyper already retains one
  finest approximation with direct coarser rounding. Dyadic offsets, compact
  approximations and explicit partial decisions are already represented.
  Generic compact search belongs to a future function-space layer, while
  current Hypersolve Bernstein subdivision already preserves depth-limited
  unknown outcomes and exact endpoint decisions. No new candidate survived;
  there is no production patch requiring another full cross-crate gate.
- haskell-fast-reals source and scoped transfer comparison are COMPLETE,
  with the original full native build explicitly unqualified. Overall
  ecosystem goal remains ACTIVE; HERA and later references remain pending.

### HERA audit opened and source coverage completed

- Official HERA-0.2 archive retrieved from the project download link, checked
  for safe paths/types, hashed and extracted into exact-real-references/hera-0.2.
  Archive SHA-256 is b4dcd62c876c5cb4cc5bb42465dfcc7dfe71695934b9887f9e7acbf29015eae9.
  All 14 text files / 2,849 physical lines are read. Inventory, hashes and
  complete read ranges reconcile; originals are unchanged. Per-file findings
  and explicit remaining work: exact-real-references/HERA_FILE_NOTES.md.
- Original GHC 9.6.7 configure succeeds, but build fails in hsc2hs on removed
  MPFR rounding tags. An isolated compatibility copy repairs current-host FFI
  imports/alignment/type widths, excludes obsolete unused tags/random2, moves
  unsafePerformIO's import and modernizes MonadFail constraints. No numerical
  formulas, constants, import policies, cache or limit implementation changed.
  The seven-module compatibility build passes with historical warnings;
  this is explicitly not a native original or warnings-clean build.
- Independent exact-Rational ball checks in the compatibility build:
  add/sub each pass 3,136; multiplication fails 1,620/3,136; division fails
  4/1,792; abs passes 112. Mathematical exp(0)=1 and log(1)=0 witnesses
  expose 30/36 exp and 6/9 log enclosure failures. Named constant controls
  fail 9/9, isZero 1/1, compose/decompose 15/15 and Ball.fromWord 2/5.
  These are reported numerical failures, not a passing test suite.
- Public Real import controls show frozen 32-bit Int/Word points: all six
  requests for 2^40+1 violate their 3/8/16-decimal error bounds. Parsing 0.1
  freezes 51/512 and violates 8/16-decimal requests. A valid eventually
  constant limit sequence (a0=0, ak=10 for k>=1, error 10*2^(1-k)) yields
  a false strict comparison at stage 1 because limRec intersects the initial
  seed point as though it enclosed the limit. Four later stages are inconclusive.
- Sources/probe/binary: /tmp/hera-audit.UOikl9/{native,compat,HeraProbe.hs,hera-probe}.
  Logs: /tmp/hera-{native-configure,native-build,compat-build,probe-build,probe}.log;
  /tmp/hera-compat-source.diff records every compatibility change. Stock MPFR
  is 4.2.2. A normal-workload Memcheck pass is being collected separately.
- HERA remains OPEN: broader ordinary-kernel and small-radius qualification,
  general limit/series checks, last-request-cache timing/storage and the
  supporting semantics/performance comparison are unfinished. Official RZ
  paper and both slide decks are located but not fully read or credited.
  No new Hyper production change is selected. Do not mistake source coverage
  or discovered donor failures for final transfer closure.
- Supplementary HeraImportProbe confirms all nine Int/Word/decimal requests
  return Right, not a limited-accuracy Left warning. Exact outputs are in
  /tmp/hera-import-probe.log. This strengthens the retained-object import
  contract counterexamples without altering any donor numerical code.
- Memcheck attempt is INCONCLUSIVE: after over three CPU-active minutes it
  still had only startup output, with no completed numerical output/error
  summary. It was interrupted (exit 130), and a host process check confirms
  it is gone. Do not report a zero-error pass. Check GHC/tool/runtime behavior
  before a bounded retry; preserve /tmp/hera-probe-memcheck*.log.
- TURN CHECKPOINT: PROGRESS. Data.CReal transfer closure and haskell-fast-reals
  file/line/qualification/transfer closure are recorded. HERA has complete
  source coverage plus explicit compatibility-build numerical counterexamples,
  but performance, broader qualification and supporting-paper review remain.
  No new production change is retained this turn. Hyperreal is clean at
  21e76ea and Hypersolve is clean. Hypercurve now has concurrent user edits
  in ten source files; those are untouched and excluded from this audit's
  changes and test attribution. All sessions started in this turn are closed.
  Overall goal remains ACTIVE, neither complete nor blocked.

### HERA qualification and targeted transfer closure

- PROGRESS: the follow-up closes HERA's source/qualification/transfer pass,
  not the overall ecosystem goal. All 14 originals / 2,849 lines still match
  their inventory hashes and complete read ranges. Detailed chronological
  per-file notes and final disposition: exact-real-references/HERA_FILE_NOTES.md.
- The compatibility build passes 684,162 exact-Rational ordinary dyadic
  controls (directed arithmetic and ternary signs, FMA, scaling, sqrt via
  rational squaring, extraction and exact Num arithmetic). The full numerical
  suite is not a pass: small-radius ball division fails 2,484/17,496 cases,
  including 64/128-bit signed-center cancellation witnesses. All 162 additional
  ball-sqrt enclosure controls pass. Num.fromInteger fails 32/160 offset-integer
  cases around/above 2^1024 because Double-based precision sizing fails.
- General lim/limRat/infSum/infSumRec geometric controls each pass 12 exact
  comparison/achieved-accuracy checks. Both series entry points fail to finish
  a finite exact-tail series in three seconds (exit 124); source proves the
  zero-error/zero-remainder continuation test remains true forever. The earlier
  false strict limRec result and frozen Int/Word/decimal imports remain defects.
- Both last-stage and isolated finest-stage cache variants pass 156 independent
  directed-MPFR radical-sum/history checks, covering every benchmark seed.
  108 CPU 6-pinned, alternating-order process-CPU/allocation observations show
  median same-accuracy requests falling from 524--645 microseconds to
  2.19--8.39 microseconds; mixed 8/fine-digit requests fall from 512--569 to
  4.11--9.55 microseconds. HERA approx restarts below the last successful stage
  even on repeated identical accuracy requests, evicting its useful result.
  Cold times are similar (variant 0.5--2.2% slower); cold allocations and linked
  size are effectively/fully unchanged. Hyper already preserves its finest
  approximation with synchronized coarser rounding, so no cache patch is needed.
- 48 compact-radius observations and 336 independent exact endpoint controls
  qualify the unchanged donor addition path versus a labelled full-radius
  formula reconstruction. At 4096 center bits, fixed-32 radius is about 1.24x
  faster and uses 19% less cumulative allocation; at 64 bits it is slightly
  slower. Managed live input storage is lower at high precision, not a claim
  about total RSS. Hyper's one-integer approximation has an implicit +/-1
  error unit and no second variable-size radius to compress. No representation
  replacement is justified by the donor's genuine ball-specific benefit.
- The full 21-page RZ paper / 1,135 extracted lines and 14/15-page slides / 251/215
  lines are now read, including all axiomatization appendices. The limit formula
  and spectrum diagram were also visually inspected. PDFs/text and SHA-256
  are preserved. Specifications extracted into comments are proof obligations,
  not evidence of a verified HERA implementation; historical OCaml Era speed
  claims are not current HERA measurements. The later RZ repository audit is
  still pending independently.
- General error-sequence limits are a real API expressiveness difference from
  Hyper's elementary tower, not falsely labelled subsumed. An unchecked caller
  convergence promise is not selected as a scalar-core transfer without a
  checked domain/convergence, cancellation, serialization and exact-fact
  contract plus a relevant consumer. Partial decisions, local proven series
  tails and multivalued near-integer choice already have Hyper counterparts.
- Two bounded ordinary Memcheck workloads now finish with zero errors: a
  large integer import and a qualified radical-sum/cache workload. These are
  scoped checks, not full leak or legacy-format safety qualification. The
  previously interrupted larger attempt remains inconclusive. No unsafe ABI
  or format vulnerability reproduction was run.
- Unchanged Hyperreal 21e76ea passes 227 new controls: 120 import/history,
  72 directed-MPFR radical-sum/history, 35 exact elementary zero/one identities.
  Probe sources, raw tables, logs and the compatibility diff are now preserved
  in exact-real-references/hera-qualification; frozen executables remain in
  /tmp/hera-audit.UOikl9. No new Hyper production patch is retained and no
  unrelated worktree changes are attributed to this pass. Remaining donors,
  starting with Few Digits, are still pending.
- TURN CHECKPOINT: PROGRESS. The final rebuilt kernel rerun reproduces the
  prior output byte-for-byte; all 108 cache / 48 radius rows reconcile, both
  156-check cache validations pass, and every session from this turn is closed.
  Hyperreal remains clean at 21e76ea and Hypersolve remains clean. Hypercurve
  has concurrent changes in the same ten source files plus
  tests/exact_structural_facts.rs; none was touched or counted as audit work.
  Overall goal remains ACTIVE, neither complete nor blocked.

### Few Digits source, qualification and targeted transfer closure

- PROGRESS: all 11 published 0.5.0 package files / 1,037 physical lines are
  read and hash-verified; all 21 pages of the linked completion-monad paper
  are visually read, including proofs, tables, bibliography and source appendix.
  The official and arXiv PDFs are byte-identical. Paper code is older than the
  archive: simplest-rational/Newton policies differ from dyadic/Wolfram code.
  Do not treat historical paper measurements or mathematical proofs as current
  archive performance or machine-verified implementation correctness.
- Original Cabal and direct GHC builds fail on undeclared dependencies and
  obsolete APIs. A separate compatibility copy typechecks all eight modules
  and preserves every numerical formula, dispatch decision and bound. The
  five-file compatibility diff and full source coverage are preserved in
  exact-real-references/FEW_DIGITS_FILE_NOTES.md and its inventory/read ledger.
- 655,415 exact-Rational controls and 216 independent directed-MPFR interior
  checks pass. The numerical suite as a whole is NOT a pass: ICReal cosine of
  opaque zero gives sine; sqrt(9/64) itself approximates 3/8 but stale [0,3/16]
  metadata clips its square to 9/256 and its product with one to 3/16. Twelve
  of 40 exact semantic checks fail. Generic exact/regular sqrt zero and asin(1)
  /acos(1) each time out at three seconds. Empty derivatives/lifted zero and
  ordinary multivariate dy/dxy throw tail []. Unchanged embedded properties
  independently reproduce these failures. A bitLength property's own shift
  boundary is defective; the independent dyadic approximation grid passes.
- 180 forced, CPU6-pinned donor sum observations compare balanced LCM,
  sequential LCM, left addition and balanced rational addition. Balanced LCM
  loses on shared/nested/dyadic and small independent inputs, while gaining
  modestly on large mixed denominators. At512 terms/512-bit parameter, mixed
  medians are 50.03ms balanced LCM versus 63.48ms sequential; balanced addition
  is faster still at37.55ms and uses2.42MB rather than50.56MB managed allocation.
  Results all pass exact Rational oracles; these are donor CPU/allocation data.
- 180 Hyper mean/schedule observations pass independent GMP rational oracles.
  An initial apparent mean failure was a HARNESS interchange defect: Hyper's
  mixed-fraction Display and GMP's whitespace-insensitive parsing differ.
  Direct signed numerator/denominator interchange corrected it; all final
  observations were rerun. The superseded partial table is preserved and not
  counted as qualified evidence. No Hyper arithmetic defect is claimed.
- 48 additional costly-case observations disable allocation counter increments.
  Balanced LCM is about1.48x/1.98x slower than current Hyper mean on the128-term
  mixed/independent families and still3.5--8.7% slower on256-term/512-bit cases,
  with greater cumulative allocation. External schedule controls share Hyper's
  public wide-GCD primitive; current mean also retains private structural and
  mixed-width shortcuts. This is a scoped schedule experiment, not a patched
  production A/B. The donor's balanced-LCM policy is not retained.
- 54 paired sine/Fibonacci-ratio observations compare unchanged dyadic
  compression with the older paper's simplest-rational policy reconstruction.
  Every result passes directed8192-bit MPFR, including a reserved budget for
  compact-report rounding. Dyadic compression is1.1--5.6x faster and allocates
  about72--80% less at512bits. It produces larger power-of-two denominators but
  cheaper arithmetic; the rational-compressed linked probe grows16,512 text
  and640 data bytes. A no-compression control times out at10 seconds and is
  not credited with an achieved-accuracy or finite performance result.
- Hyper already has implicit-error integer approximants, one-finest-result
  caches, specialized final-reduction rational schedules, balanced long Real
  sums, and scale-aware shared-DAG scheduling. Earlier ireal qualification
  covers independent/shared-prefix sum tradeoffs; no new n-ary node is selected.
  Bernstein bounds and zero derivatives are already represented in geometry/
  solving. Arbitrary completion/continuity-modulus closures remain a genuine
  broader API capability, not falsely called subsumed; importing them needs
  checked convergence/domain, cancellation, serialization and exact-fact
  contracts plus an actual consumer. No scalar-core expansion is justified.
- Unchanged Hyperreal21e76ea passes920 matching scalar/history controls. The
  unchanged Hypersolve symbolic module passes486 exact derivative/zero checks
  (a scoped module build, not the full integration suite). The donor semantic
  workload completes Memcheck with zero memory errors despite numerical
  failures. No full leak or whole legacy-runtime qualification is claimed.
- Durable sources, compatibility copy, commands, raw tables, logs and oracle
  manifest: exact-real-references/few-digits-qualification/. No new production
  change is retained. All numerical source and paper review for this published
  archive is closed; inaccessible Darcs history is not silently counted read.
  Escardo signed-binary reals and the other pending references remain open.
- TURN CHECKPOINT: PROGRESS. The durable five-bin oracle manifest builds
  offline; rebuilt scalar/history, symbolic-derivative, compression and dyadic
  mean controls pass. All180/180/48/54 benchmark tables have their expected
  unique row counts and columns. Every session from this turn is closed.
  Hyperreal remains clean at21e76ea; Hypersolve is clean. Concurrent Hypercurve
  work advanced tofb8aba8 and now leaves PERFORMANCE.md modified; neither that
  work nor the earlier source edits was touched or counted as audit output.
  Overall goal remains ACTIVE, neither complete nor blocked.

### Escardo signed-binary audit — source and native qualification

- All five repository files /1,995 lines at971ff20d are read independently;
  source inventory and exact ranges are in ESCARDO_FILE_INVENTORY.tsv and
  ESCARDO_READ_COVERAGE.tsv. The unchanged native program builds and runs.
  Generator output differs from checked-in source only in four indentation lines.
- Two linked foundational algorithm papers are fully read: Escardo/Simpson's
  27-page interval-object paper and Simpson's nine-page functional/integration
  paper (plus cover). Printed equations were checked visually where needed;
  explicit rescaling/constant-sequence formula defects are recorded in
  exact-real-references/ESCARDO_FILE_NOTES.md, not treated as proven algorithms.
- Independent exact-prefix grid:185,061 checks,2,700 failures. Selected mul2,
  imin/imax, and fixed55-digit Double conversion have concrete failures.
  Separate square-root controls fail9/18; zero and1/16 tend to -1. These are
  distinct from deliberately broken buggyMul. Alternate multiplication
  versions0,1,3 each pass the same19,683-check grid. The audit-only one-line
  mul2 correction passes26,244 controls; restricted-domain root passes35.
- 156 small functional/quantifier/affine-root controls pass. Extra normalization,
  corrected multiplication/root and continuous-gluing controls total78,407;
  fifteen fail from native Int overflow, while the other78,392 pass. The three
  documented partiality cases and unproductive naive bigMid time out at3s.
- Directed8192-bit MPFR:455 native results,115 failures across scaled
  elementary operations and Takano pi/4. BBP pi/32 passes all8 demands; all498
  printed native decimal digits pass an independent directed MPFR oracle.
  Takano2048bits exceeds90s; the completed455-row final table excludes this
  unachieved request. Hyper21e76ea passes2,060 matching controls/history.
- 243 CPU6-pinned forced-result kernel observations plus162 larger-batch
  normalization repeats all pass exact Rational output checks. Corrected
  zero-prefix multiply gives strong gains for sparse/terminating streams;
  specialized square roughly halves managed allocation. One-digit normalization
  has mixed runtime but real lookahead/productivity and allocation benefits.
  These are within-GHC comparisons, not cross-language scalar benchmarks.
  Normal-workload Memcheck completes with zero memory errors, not a claim
  of numerical correctness or full leak/runtime qualification.
- New isolated Hyper candidate: evaluate unresolved sqrt(Square(x)) using
  abs(x.approx(p)). Absolute value is1-Lipschitz, preserving the one-unit
  approximation contract without a new serialized node. The first build
  required an internal square-operand query to respect Node's private fields;
  probe-only integer/ownership/public-API mistakes are fixed separately.
  Candidate remains UNQUALIFIED and unretained pending oracles, paired
  measurements, regression/feature gates and size review. Production unchanged.
- Sources/raw evidence are in exact-real-references/escardo-qualification/.
  The tmpfs user's51,490MB quota was reached; this audit's26MB directory was
  moved to workspace .audit-escardo.A6Oxth, preserving its /tmp path via symlink.
  No evidence was deleted. Build temporary files now use that workspace path.
  Overall goal remains ACTIVE; donor transfer closure is still open for the
  isolated absolute-value experiment, not blocked by the resolved quota issue.

### Escardo transfer closure — retained sqrt-square evaluator

- RETAINED Hyperreal bd92d87, based on21e76ea. Runtime changes are14 lines:
  one private square-operand query and one sqrt-kernel branch returning the
  magnitude of a p-precision child approximation. The1-Lipschitz proof retains
  the one-unit error contract at every precision without a sign decision,
  squared intermediate, new node/field, public API or serialization change.
  Two regression functions (52 lines) and26 performance-note lines accompany it.
- Baseline and candidate each pass966 independent exact/MPFR/history/serde
  checks plus a public Real::abs lossy-zero control. New raw-node regressions
  cover385 rational/precision/history cases and explicitly test unresolved
  sign, one retained child approximation and abort-without-cache-publication.
- Final A/B:432 alternating-order CPU6-pinned timing samples plus48 separate
  allocation-count observations, all outputs forced and independently checked.
  Positive/negative unresolved abs improves2.68--3.70x with49--78% less cumulative
  allocation. Opaque zero +/-2^-256 improves2.01--2.69x when resolved by demand;
  exact zero is effectively unchanged at2048bits. Ordinary sqrt CPU remains
  within1.5% and allocations are identical. The336-row short pilot is not used
  for final claims. Linked serde/oracle text-368/data0/BSS+384 bytes is a scoped
  artifact result, not a claim that every downstream binary shrinks.
- Production826-test all-feature debug and release gates pass, as do all-target
  all-feature Clippy -D warnings, cargo fmt and diff checks. Default library-
  only downstream gates pass Hyperlattice19, Hyperlimit242, Hypertri3 and
  Hypersolve429. The native compiler-cache sandbox failure is retained with
  the approved CCACHE_DISABLE retry logs. No full downstream matrix claimed.
  Candidate oracle Memcheck completes with zero errors, without a full leak
  claim. Sources, patch, frozen worktrees, commands, raw tables and logs persist
  in exact-real-references/escardo-qualification/ and .audit-escardo.A6Oxth/.
- Source coverage/hashes reconcile: all five originals and both PDF/text pairs
  still match their inventories. Donor native numerical failures remain failures;
  alternate/corrected control successes do not reclassify the original program.
  General streaming replacement, arbitrary searchable-function closures and
  continuous-conditional nodes are not justified for the current consumers.
  Escardo's source/qualification/targeted transfer pass is CLOSED.
- TURN CHECKPOINT: PROGRESS. Hyperreal is clean atbd92d87. Concurrent Hypercurve
  work now modifies src/bezier_offset.rs; it was not touched or attributed to
  this audit. Hypersolve is clean. No evidence was deleted during quota recovery.
  The overall goal remains ACTIVE, neither complete nor blocked. Next pending
  inventory target is Plume; all other pending rows and cross-reference ideas
  remain open rather than silently counted read or rejected.

### Plume retrieval and report inventory started

- Previous goal turn is PROGRESS: Escardo source/qualification closure and
  retained Hyperreal bd92d87 are authoritative. Current Hyperreal is clean at
  that commit; Hypersolve is clean, concurrent Hypercurve src/bezier_offset.rs
  remains untouched. No new production edit is selected in this pass yet.
- The exact referenced Edinburgh index.html downloads successfully with
  Last-Modified2000-05-11, although the web reader rejects that URL spelling;
  its directory-form URL renders the1998 report and full contents. Three
  implementation/metadata pages node91,node97,node147 were retrieved and all
  HTML lines read. The module diagram image remains unread, so node91 visual
  coverage is not yet complete. Files are preserved in exact-real-references/Plume/.
- The report describes signed-binary and dyadic-stream layers, general
  mantissa/exponent, conversions, limits, transcendental and functional
  operations, parser/UI. Historical tools were Gofer, Hugs, GHC, Happy and Alex.
  No downloadable implementation archive has yet been found. Source retrieval
  continues through the supervisor's publication links; do not label the
  report itself a full implementation source audit. A transient DNS error
  was successfully retried outside the sandbox with approval.

### Plume recovered-source and numerical-kernel checkpoint — 2026-09-06

- Source retrieval succeeded after the preceding entry. The supervisor's
  public repository dbp/ directory preserves v1.2.tar.gz, cgi_source.tar.gz,
  performance.tar.gz and solaris_demo.tar.gz. Commit2f2aa72457d14f2363a741af1ebb52932a48d6ec
  pins the downloads; Git blob hashes match. SHA-256 values and the174-file
  inventory are in exact-real-references/PLUME_FILE_NOTES.md and
  PLUME_FILE_INVENTORY.tsv. Four SPARC executables are identified, not run.
- Main v1.2 numerical modules, calculator evaluators, grammar specifications,
  support and examples are now read; exact ranges are in PLUME_READ_COVERAGE.tsv.
  Generated parser/lexer tables are not claimed read. CGI/performance variants,
  bibliography and report remainder remain open. The133-page report text is
  read through1630/4871; physical PDF pages35 and40 were visually checked.
  The affine rescaling formula is correct despite text extraction; the printed
  dyadic-to-signed conversion really uses1/4 where source correctly uses1/2.
- Unmodified GHC9.6.7 build fails on obsolete Array/toInt APIs. An isolated
  compatibility copy restores toInt/fromInt, Data.Array/fmap and old Maybe zero
  spelling; no numerical formulas are changed. The numerical probe and ordinary
  calculator compile. Compatibility is not represented as a native build pass.
- Independent Data.Ratio Integer grid:212,860 checks,24,305 failures. Dyadic
  add/sub/shift, negative integer division, strict comparison, noncanonical
  equality and four constants fail. Signed average/add/sub/min/max and BOTH
  signed/dyadic-via-conversion multiplication paths pass their ordinary grids;
  each multiply path passes19,683 checks. Scaled division passes540 cases and
  valid nested interval limits pass42. Finite grids do not prove general safety.
- Boundaries confirm a cap2 normalizer consumes20 leading zeros, and exponent
  sum500 multiplication normalizes zero without producing a result while499
  remains productive. Several zero elementary operations exhaust the bounded
  probe heap. Exact-equality comparison is intentionally partial, separately
  classified from incorrect answers on strict inequalities.
-129 elementary requests yield116 finite prefixes; thirteen time/heap-limited
  requests are recorded rather than counted successful. Directed8192-bit MPFR
  rejects five returned log prefixes: ln(2) tends to0.625, and ln(8) also fails
  at higher precision. Unchanged Hyperreal bd92d87 passes464 corresponding
  precision/history observations and25,962 exact scalar/history controls.
- Initial multiplication performance pilot passes exact Rational output checks;
  final alternating-order CPU6 timing/allocation measurements are running.
  No Plume-derived Hyper change has been selected or retained. Whole Plume
  source/report/variant audit and broader inventory remain ACTIVE and open.

### Plume main-kernel checkpoint qualified; variant/report audit remains open

- Main v1.2 coverage is33 files/4,498 physical lines, including the379-line
  vendored Alex runtime. Four generated parser/lexer files are compatibility-
  compiled (ordinary tables also calculator-smoke-tested), not line-read or
  credited as handwritten scalar source. Performance README, LookInt and
  sbMul.fix add3 files/292 lines. Duplicate copies remain individually pending.
  All174 original-file hashes still match the inventory.
- Extra exact grid39,465 checks adds107 machine-Int division failures, while
  ordinary dyadic arithmetic/conversion and195 valid decimal imports pass.
  Combined with the first grid:252,325 checks,24,412 failures.270 compact-
  functional requests produce236 passing results and34 one-second timeouts;
  these timeouts are not classified as proofs of nontermination.
- Representation-specific controls refine the log finding: endpoint/redundant
  streams for2 converge to0.625, but decimal/terminating2 representations give
  correct ln(2), including ordinary calculator2^3. Equal mathematical inputs
  must not produce different real limits. All24 representation probes return
  prefixes and14 fail directed MPFR, including higher-demand ln(8). Native
  formulas remain unchanged in the compatibility copy.
- Final elementary/UI/boundary reruns outside the sandbox remove EPERM from
  child-process records;116 finite elementary prefixes match the original
  sandbox run exactly. Nine256MB zero-operation failures and four3s atan(+/-1)
  timeouts persist. Unchanged Hyper is now checked on ALL129 requested inputs,
  including donor non-results:516 precision/history and26,232 exact controls
  pass. Memcheck extra-grid run has zero memory errors, without a leak claim.
- Final benchmark evidence is216 observations from the frozen probe source;
  discard24 round0 warmups, leaving192 observations/eight per group with
  alternating pair order on CPU6. Every output passes its independent Rational
  oracle; no child permission errors remain. Prior pilot/sandbox observations
  are preserved separately and not mixed into final timing statistics.
  Dyadic-via-conversion multiply takes8.3--24.4x the signed-kernel CPU for dense
  inputs with10.5--14.1x managed allocation. Sparse and terminating families
  show still larger losses. These are pre-forced-input GHC kernel measurements,
  not a Hyper speed comparison, retained-RSS estimate, or binary-size claim.
  Submillisecond samples have limited timing resolution; full min/max data and
  allocation observations are retained in qualification-summary.json.
- Hyper's current magnitude-aware integer kernels, exact dyadic Offset shifts,
  specialized squares, single-finest synchronized cache and checked log range
  reduction already cover the applicable main-kernel patterns. Generic compact
  functional closures/nested-interval limits remain real broader capabilities,
  not falsely labeled already implemented; no new consumer/API expansion is
  justified in this partial pass. No production code change is selected.
- Durable commands, compatibility diff, frozen benchmark source, exact/MPFR
  oracles, raw tables, exit records and analysis script are in
  exact-real-references/plume-qualification/. PLUME_FILE_NOTES.md and
  PLUME_REPORT_COVERAGE.tsv distinguish real read ranges from generated-file
  validation and extraction-only text. Scratch .audit-plume.SZHOs8 is retained.
- TURN CHECKPOINT: PROGRESS. Hyperreal remains clean atbd92d87; Hypersolve is
  clean. Concurrent Hypercurve work advanced to065e9daa and now leaves
  benchmarks/checkpoints/2026-09-06-native-fiber-specialization.json untracked;
  neither its prior source edit nor its new commit/evidence was touched. No source
  reference or audit evidence was deleted. All work from this checkpoint is
  scoped numerical qualification, not closure of the whole Plume audit.
  Next: report1631-4871 and remaining visual equations/figures, every CGI and
  performance variant/support file, bibliography/Solaris fixtures, and remaining
  instrumentation/logistic-map/normalization/conversion/functional experiments.
  The overall goal remains ACTIVE, neither complete nor blocked.

### Plume performance-variant source read started

- Previous turn is PROGRESS: main-kernel qualification and final benchmark
  evidence are retained, with no production change. Current Hyperreal is still
  clean atbd92d87; concurrent Hypercurve has the untracked native-fiber
  checkpoint recorded above. No user work is touched.
- All fifteen Haskell files in versioned/perform are now read independently,
  including the distinct older IReal and mismatched module-name decimal/test
  files. Exact ranges are updated; the separate performance.tar.gz copies,
  plotting scripts and assets were initially not yet credited read. Subsequent
  reading covers all36 versioned/perform files/4365 physical lines, including
  every plot coordinate, generated EPS procedure and support-data row. The
  separate performance.tar.gz copies remain independently pending.
- These modules are not merely main-source instrumentation: one doubled-sign
  helper calls the opposite recurrence; division changes negative remainder
  handling; limits choose only an unboundedly normalized upper endpoint; sin
  and atan use ordered limits instead of main v1.2's between construction.
  Lookahead metadata drops some control dependencies. Numerical and actual-
  demand qualification is required before relying on performance claims.
- New isolated build scratch .audit-plume-perform.LVoj1D preserves the donor
  originals. Native build and explicitly scoped compatibility bridges are
  being checked; no numerical repair is selected or credited as native output.
- Separate numerical grid now completes200,017 checks with3841 failures and
  zero exceptions. The redundant-representation division grid alone catches
  870 bad prefixes missed by the simpler540 scaled canonical controls. Negative
  integer division, capped doubling, strict comparison and negative limits also
  fail. Full81-input-pair demand tests cover152,700 valid numerical outputs;
  486 signed-multiply rows under-report the maximum input-digit VALUE index.
  The independent oracle supplies infinite spines/tags with undefined values
  after a cutoff; it measures forced values, not list-spine traversal, time or
  allocation. All other measured tags match this simultaneous-prefix demand;
  this is not a proof of instrumentation completeness outside the finite grid.
-108 elementary requests produce100 finite prefixes; directed8192-bit MPFR
  rejects20 (nine sin, six atan, five ln). Negative sin/atan failures distinguish
  ordered-limit construction from main v1.2's between construction. Approved
  rerun has no EPERM; initial sandbox record is kept separately. All requests
  are a subset of the main Hyper oracle's129 already-passing input/history set.
- Extra tests complete1498 checks with266 failures and zero exceptions:
 195 decimal imports pass,96/585 output conversions fail; signed logistic
  variants and dyadic-float logistic map pass102 each, while90/102 dyadic-stream
  map prefixes fail. Even the missing-factor4 control fails74/102 because its
  subtraction is independently wrong. The older untagged IReal variant repeats
  the negative-limit failure (3/102); LookInt compare overflows in3/4 selected
  machine-boundary controls. No numerical donor formula was repaired.
- Two original EPS figures were rendered and visually read. The multiplier
  graph labels the25 tick as35. Modern fig2dev rejects the original FIG
  transparent-color headers. A scratch header-only bridge on test2.fig renders
  successfully but consumes its embedded raw debug rows as drawing coordinates,
  confirming plotfig output contamination; renderer success is not graph
  validity. Remaining qualification notes/reproducibility checks are in progress.

### Plume versioned/perform qualification checkpoint — 2026-09-06

- All36 files/4365 physical lines in this archive directory are read, including
  figure assets and data. No separate archive copy, CGI source, remaining report
  line, or broader inventory target is credited by this checkpoint. The report
  remains at1630/4871 extracted lines and the previously recorded visual pages.
- Final frozen InstrumentedProbe.hs SHA-256 is
  af10ffbec882d11aa60a2290a3e803f187e0833b88cd2d53d101b756b3bdfd2b.
  Compatibility-only bridges and exact build paths are preserved. All optimized
  and unoptimized grid, extra, demand and example outputs are byte-identical:
 201,515 numerical checks with4107 failures per build,152,700 numerically valid
  demand rows with486 under-reported multiplication cases. Explicit divisor-tag
  control also confirms omitted metadata for a forced divisor value. Neither
  tag is a sound dependency certificate. Memcheck extra run has zero memory
  errors; numerical failures are not relabeled successes and no leak claim.
- Final approved averaging benchmark has216 fresh-process observations on CPU6,
  192 after round0 warmups, eight per group and alternating pair order. Every
  output is independently Rational-qualified; source is frozen and all status/
  child-error checks pass. Dense parity-sensitive average takes1.18--1.30x
  median CPU and1.35--1.43x cumulative allocation of carry average, despite its
  lower lookahead. Other families are mixed. Input generation and output-tag
  forcing are outside the timed region; this is not a Hyper/RSS/binary-size
  comparison. Submillisecond timing limits and pair variability remain explicit.
- Current Hyper format.rs1-577 and linear_demand_tests.rs1-226 are read, with
  relevant representation/scheduler ranges recorded in PLUME_FILE_NOTES.md.
  Hyper's saturating hint changes operand order only; both operands retain the
  same required precision. It is not serialized numerical state or claimed as
  complete observed demand. No additional per-digit metadata is justified.
- All1170 new exact decimal/history checks pass for unchanged Hyper. The first
  audit oracle accidentally interpreted Hyper mixed-fraction display as a GMP
  simple fraction; its failure log is preserved and the corrected oracle parses
  decimal digits independently. This was an oracle bug, not a Hyper defect.
  Existing format23 and demand-scheduling5 tests pass in debug and release:
 56 scoped tests, not a new full-suite/downstream qualification claim.
- analyze-instrumented.mjs verifies all174 original-file hashes, complete36-file
  coverage ranges, frozen source, O0/O2 output hashes,108 elementary requests,
  100 finite outputs/20 MPFR failures,152,700 demand rows and216 benchmark rows.
  Durable commands, bridges, raw logs and limitations are in
  exact-real-references/plume-qualification/instrumented-README.md. Main data is
  preserved separately and its earlier benchmark source remains unchanged.
- No new Hyper production change is selected or retained. Hyperreal is still
  clean atbd92d87. Concurrent Hypercurve work advanced to40e94d69 and is clean;
  concurrent Hypersolve now modifies src/ordered_field_roots.rs. Neither was
  touched. No source reference or audit evidence was deleted.
- TURN CHECKPOINT: PROGRESS. Next: independently read the separate performance
  archive copies and every CGI variant/support file, continue report1631-4871
  with visual math/figure checks, bibliography/Solaris fixtures and remaining
  functional/representation experiments; then the rest of the broad inventory.
  The overall goal remains ACTIVE, neither complete nor blocked.

### Plume separate archive/CGI reading checkpoint — 2026-09-06

- Independently read all36 files/4365 physical lines in performance/perform,
  including every plot coordinate, EPS procedure and data row. These36 files
  are byte-identical to the qualified versioned/perform files. Existing tests
  and benchmarks apply by exact source identity, not by new measurements or
  an assumed correspondence. source-copy-map.mjs verifies all174 inventoried
  original hashes and emits the comparison manifest without granting coverage.
- All57 CGI files/8110 physical lines are now read, including all31 copies
  shared with main v1.2 and all26 different/new support files. Generated
  lexers/parsers are included; truncated combined lexer output was reread in
  smaller ranges before credit. The four main-archive generated copies have
  subsequently also been independently read in full (2187 lines), closing
  all37 main v1.2 files/6685 lines. Their bytes match the CGI copies exactly.
  The CGI-specific code is request/response, HTML, parser/pretty-printer and
  demonstration infrastructure, not another numerical implementation. The
  calculator wrapper resets state and defaults to ten digits per request.
- Native compile-only CGI check fails on obsolete System/Char imports and
  modern Prelude operator ambiguity. Scratch and failure log are preserved in
  .audit-plume-cgi.Dwlx5B. No CGI service, archived SPARC executable, environment
  display or registration-side-effect script was run. The separate pure
  expression-parser probe uses only Data.Array/fmap compatibility in Alex;
  no generated table, parser action or scalar formula was repaired.
- Both optimized/unoptimized parser probes pass33938 lexer checks (all ASCII
  strings through length2, keyword/identifier/number and position controls)
  plus88 AST/grammar checks. They confirm main power grammar restrictions and
  no power token in the integration grammar. This is syntax qualification,
  not a claim that all parsed functional expressions evaluate successfully.
- No Hyper change selected; Hyperreal remains clean atbd92d87. Current Hyper
  real/arithmetic/format_parse.rs1-106 and rational/arithmetic/format_parse.rs
  1-458 were read: Real parses scalar rational/scientific literals, not Plume's
  expression language; existing bounded scientific scaling and radix product
  tree machinery are distinct from donor per-digit decimal stream creation.
- In-progress checkpoint, not whole-Plume closure. Next reads are the main
  report, bibliography, Solaris fixtures and remaining
  functional experiments. Overall goal remains ACTIVE.

### Plume source-container closure and report continuation — 2026-09-06

- All170 inventoried text files/24049 physical lines are now independently
  read; all174 original hashes verified. Four SPARC ELF binaries identified,
  not executed. Bibliography citations do not confer reading credit on cited
  papers; Solaris Maple serialized plots were read, not rendered/executed.
  The three input fixtures add cancellation and compact-optimization cases.
- Pure parser probe, Alex-only compatibility diff, native/final build logs,
  reproducibility README and validation script are retained under
  exact-real-references/plume-qualification/. O0/O2 logs are byte-identical;
  each has33938 lexer and88 AST checks passing. analyze-source-parser.mjs
  verifies all originals,170-file coverage,57 compatibility copies and frozen
  probe/result hashes. No new parser benchmark or Hyper performance claim.
- Report reading has advanced through all4871 extracted lines, with selected rendered
  mathematical pages separately recorded in PLUME_REPORT_COVERAGE.tsv. Printed
  pages45,47-49,51,54,56 contain real pseudocode/notational discrepancies,
  not just missing PDF-extracted minus signs. Numerical source qualification
  remains authoritative; no report formula was silently substituted into code.
  Additional visual equations and remaining functional checks are in progress.
  No new production change selected; goal remains ACTIVE.
- Visually corroborated report errors also include the limit pseudocode's
  output-side test, sine/cosine/log series transcription, dyadic denominator/
  split-sum formulas and grammar labels. The source often differs correctly;
  report errors are not relabeled source errors. Detailed per-page dispositions
  and the figure review are in PLUME_FILE_NOTES.md. Report source and visual
  coverage remain separate; rendered-but-unseen pages are not credited.
- Original solaris/test1.era completes to30 decimal places in the modern
  compatibility calculator, correctly approximating -54767/66192. Independent
  GMP controls on100 neighboring/sign-varied input pairs and two coefficient
  decompositions pass200 Real exact comparisons plus1200 Computable precision/
  history checks in debug and release. Public folding is allowed; no raw-node
  or new full-suite/downstream claim. The single calculator run's timing/RSS
  is diagnostic, not benchmark evidence. No Maple or SPARC program was run.
- Exact squared-endpoint inequalities qualify the saved maximum prefix in
  test2.mws and its critical-point inclusion, without trusting the encoded
  plot. Additional exact identities pass6561 dyadic multiplication and189
  carry controls; the rejected printed variants fail6016 and88 respectively.
  Release/debug PASS lines are identical. Memcheck completes with zero memory
  errors; no leak claim. Two audit-only Rug compile errors were fixed before
  successful runs; original failure logs remain preserved.
- Current Hyper long-sum balancing, homogeneous symbolic folding, chunked
  rational products and mixed-product order were reread and compared against
  the report's tree-reassociation recommendation. Existing targeted reducers
  cover the applicable idea; no unconditional rewriting layer is justified.
  Durable report_fixture_controls.rs, README, hashes, raw outputs and validator
  are under exact-real-references/plume-qualification/. No Hyper source changed.
- TURN CHECKPOINT: PROGRESS. Hyperreal remains clean atbd92d87. Concurrent
  Hypercurve is now clean at017f9826 and Hypersolve clean at42f2bdb8; neither
  was changed by this audit. No original reference or audit evidence deleted.
  Next: remaining visual equations/figures, alternate author/report HTML
  containers as appropriate, higher-iteration logistic/functional and remaining
  representation/normalization claims, then the rest of the broad inventory.
  The overall goal remains ACTIVE, neither complete nor blocked.

### Plume higher-iteration and functional qualification — 2026-09-06

- Prior turn is PROGRESS: source/report coverage and exact fixture controls
  are durable. Current Hyperreal rechecked clean atbd92d87. No user work touched.
- New LateProbe builds in .audit-plume-late.4Yth3g. Its only additional donor
  bridge exposes Tests' hidden functions; all bodies remain unchanged and
  imported numerical modules use the already-qualified main compatibility copy.
  Frozen previous KernelProbe and InstrumentedProbe benchmarks remain intact.
- Short logistic grid:392 requests per O0/O2 build,350 finite prefixes and42
  750ms observations without results. Finite outputs are byte-identical across
  optimization modes;28 dyadic-stream prefixes fail independent directed
  interval qualification. Every returned prefix in the other six variants
  passes. This includes differing exact input representations and zero/fixed
  points; the normalized wrappers and unnormalized demo step are not conflated.
- Longer runs:42 requests at10/40/60 iterations and32 requested fractional bits;
  34 prefixes return, two dyadic-stream results fail. Eight2s caps affect dyadic
  variants at40/60 iterations. All signed/cross-representation returns pass.
  First-digit probes return for both half representations, but the huge
  greedy-stream numerator differs from the report and belongs to a known
  numerically wrong recurrence; it cannot validate a logistic performance claim.
- Four functional fixture families at0/4/6/8 bits: O2 returns12/16 prefixes,
  O0 returns10/16; all returned values pass exact or4096-bit outward MPFR
  controls. The3s caps are observations, not proofs of nontermination or a
  performance comparison. The positive narrow-interval reciprocal maximum
  is reproduced at8 fractional bits and matches the archived enclosure.
- A preliminary concern that eager rational folding could dominate Hyper's
  logistic construction was disproved for this actual workload: the factor4
  becomes an Offset node, retaining demand-driven computation. Bounded runs
  at10/20/60 iterations succeed. All160 Hyper precision/history checks over40
  input/iteration cases pass in release/debug. No guessed rational-policy edit
  was made. A91-case exact-rational self-check validates the interval oracle.
- All618 lines of report index.html now read. The two4871-line report text
  extractions differ in five small layout hunks, read explicitly; neither
  container comparison nor linked contents grants additional full read credit.
- Paired CPU6 benchmark of the correct signed/dyadic-float recurrences now
  has108 fresh-process, Rational-qualified observations (96 after12 warmups),
  two inputs and4/8/10 iterations at32-bit demand. Timing starts before input
  evaluation, ends after forcing output digits, excludes oracle/process startup;
  allocation is cumulative managed allocation, not RSS. Final analysis verifies
  all source hashes, exact bridge, process/result counts and108 benchmark rows.
  At10 iterations the dyadic/signed median CPU ratio is124.42--147.27 and
  allocation ratio90.65--103.30, reinforcing the existing representation choice.
  Full extrema, paired ratios and finite-sample bootstrap details are retained;
  submillisecond signed samples are explicitly treated as variable. No Hyper
  speed or binary-size comparison is inferred. Memcheck Hyper-grid run has
  zero errors, no leak claim. late-README.md records commands and limitations.
  No new Hyper production change selected; overall goal remains ACTIVE.
- TURN CHECKPOINT: PROGRESS. New probes, process/error records, independent
  oracles, frozen benchmark and reproducibility analysis are all durable.
  Main/instrumented/source-parser evidence revalidates without changes to donor
  originals or earlier frozen benchmarks. Hyperreal stays clean atbd92d87;
  Hypercurve now has a concurrent src/bezier_offset.rs edit, left untouched;
  Hypersolve is clean. No evidence/source was deleted. No processes remain
  running from this checkpoint. Next: remaining report visuals/alternate
  containers and residual representation/formatting claims, then the rest of
  the reference inventory. This is not whole-Plume or ecosystem closure.

### Plume residual representation and rendered report review — 2026-09-06

- Rechecked Hyperreal clean atbd92d87; no production edit. Reviewed all31
  previously rendered but uncredited pages, bringing visual report coverage
  to51/133: physical35,40,49-64,71-80,99-110,123-133. All report text was
  already read; neither text nor rendering implies unseen-page visual credit.
- Additional printed clipped-subtraction/residual cases disagree with the
  actual source. A repeated divisor factor, missing exp n>=1 restriction and
  false dyadic-division closure statement are separately recorded in
  PLUME_FILE_NOTES.md. Source and report defects remain distinct.
- Reread main SBinDec1-327, SBinStream1-105/415-440 and normalization
  definitions for new residual conversion/productivity controls. Work in
  progress; no new numeric or performance result claimed yet.
- Boundary qualification now has5128 independent checks perO0/O2,17 dyadic
  cap violations and no numeric exceptions.2080 decimal outputs/build contain
  128 errors beyond requested tolerance, independently BigInt-qualified.
 31 productivity requests/build give21 correct finite prefixes and10 caps;
 26 literal-helper requests/build give11 finite and15 explicit exceptions.
  Frozen sources, process records, exact controls and limitations are under
  plume-qualification/boundary-README.md. All174 original hashes unchanged.
- These controls exposed a Hyper fixed-decimal formatting issue: finite
  approximations ofsqrt(2)*sqrt(2)-2 return, but leading-nonzero-digit search
  reaches10s caps before formatting zero decimal places. Retained candidate
  bounds Display's search using its existing guarded output precision. Two
  executable lines replace one;37 regression-test lines check cancellation,
  tiny signed values and bounded cache demand. Scientific notation unchanged.
- Qualified72-row before/after CPU6 benchmark (64 after warmups) shows140.918x
  median CPU improvement and99.718% fewer requested bytes for tiny radicals.
  Ordinary-value intervals include parity, allocation unchanged. Audit binary
  file+64B, text+16B/bss-16B, loaded total unchanged. No general speedup/size
  claim. Full Hyperreal tests pass723 default-debug and828 release-all-features;
  2488 independent boundary checks pass perdebug/release; Memcheck0 errors.
  fmt check and all-feature library Clippy-D warnings pass. Downstream library
  tests pass696 across Hyperlattice/Hyperlimit/Hypertri/Hypersolve. Hypercurve
  full892-test run remains in progress, not yet credited. No commit/push.
- Physical report65-70 now also rendered and visually read:57/133 visual
  pages total, with76 remaining. Quotient residual equations and bound-driven
  limits corroborate the stated scoped contracts. Original author/HTML report
  containers remain open; this is not whole-Plume or ecosystem closure.
- TURN CHECKPOINT: PROGRESS. Retained uncommitted Hyperreal format.rs change,
  all qualification evidence and exact hashes are durable. Only owned ongoing
  process is the full Hypercurve library test session31037; its892-test run is
  still progressing and must be checked next turn before any completion claim.
  Hypercurve is concurrently updated and currently clean ate44cfbdd; Hypersolve
  clean at42f2bdb8. Neither repository was edited by this audit. Main,
  instrumented and source-parser prior evidence revalidates. No source or
  evidence removed. Overall goal remains ACTIVE, not complete or blocked.

### Plume report visual continuation — 2026-09-06

- Prior turn classified PROGRESS: qualified retained fixed-decimal formatting
  change plus source-boundary controls and57 visually read report pages.
  Current Hyperreal worktree still contains only the owned format.rs change.
  The existing Hypercurve test handle31037 is live on re-poll, with no new
  completion result; do not restart it or credit a full pass.
- Remaining76 report pages are being rendered for independent visual review.
  Rendering alone is not read credit. Boundary evidence revalidates unchanged.
- Hypercurve session31037 has now finished:891 passed,1 ignored,0 failures,
  exit0 in578.20s. Total scoped downstream library passes1587; ignored case
  excluded. Boundary README and validator now require this terminal evidence.
- Independently viewed46 more pages:physical1-34,36-39,41-48. Primary report
  visual coverage is103/133. Remaining30 are81-98 and111-122. Printed B-adic
  limit reverses its scale; exact counterexample and smaller notation defects
  are recorded separately from source findings. The report explicitly requires
  fractional-to-integer decimal carry, corroborating the already measured
  lost-fcarry source error. No additional Hyper edit selected.
- Remaining30 primary report pages independently visually read:81-98 and
 111-122. All133 primary report pages now visually covered, separately from
  the complete4871-line text read. Alternate author/HTML containers remain
  open. Implementation/analysis/conclusion ideas are compared in Plume notes;
  no new unqualified performance claim or production edit introduced.
- All ten printed correct-result logistic entries for43/64 through60 steps
  pass the4096-bit MPFR oracle and30 independent integer-only interval checks
  at1024/2048/4096 bits.39 exact-rational recurrence self-checks pass;36 exact
  counterexamples qualify the printed B-adic limit's scale error. Single/
  double historical columns/timings are not reproduced. Reused old Hyper
  binary also passes four approximation/history checks, explicitly separate
  from current Display qualification. No frozen benchmark changed.
- Acquired and visually read node91's585x256 img216.gif, matching primary
  figure6.1. Provenance/hash and controls are in report-visual-README.md;
  coverage ledger now records the image separately. Other HTML links and
  author-report remain uncredited/open.
- Hypercurve currently has a concurrent src/bezier_offset.rs edit, left
  untouched. The completed891-pass library run does not qualify edits made
  after compilation. Hyperreal still has only the owned Display change;
  Hypersolve is clean. No commit/push or source/evidence deletion performed.
- TURN CHECKPOINT: PROGRESS. Primary report visuals complete133/133; linked
  diagram and independent printed-table controls are durable. Main,
  instrumented, source-parser, late, boundary and report-visual validators all
  exit0. All174 original source hashes remain unchanged. Retained Hyperreal
  format.rs SHA4b9d969fb4020b45f78d1feb38dc6d254233165f06d839e1ea144c4f3df916cf
  is unchanged this turn. Hypercurve test and both rendering sessions have
  terminal exit0 results; no owned processes remain running. Next: independent
  author-report4871-line/container review and residual linked HTML inventory,
  then the remaining reference implementations. Overall goal remains ACTIVE;
  neither whole-Plume nor ecosystem closure is claimed.

### Plume alternate-report continuation — 2026-09-06

- Previous turn classified PROGRESS: primary visual review, diagram and
  printed-table controls completed. Current Hyperreal still has only the
  retained format.rs edit. No previous owned processes remain pending.
- Independently read author-report.txt1-1390. This corroborates the same
  representation and finite-decimal contracts; garbled extracted equations
  are not interpreted as new source bugs. One truncated combined output was
  repeated before crediting its source range. Remaining1391-4871 stays open.
- Completed independent author-report.txt1391-4871, now4871/4871. Alternate
  content repeats the same implementation contracts and documented defects.
  New suspects are printed69's claim of100 decimal digits from10^50 Leibniz
  terms and printed96's natural-log expression for binary-tree depth. These
  need rendered corroboration and exact controls before being recorded as
  confirmed report errors. No new Hyper change selected.
- Alternate text is now complete4871/4871; all5 text-difference pages and6
  further numerical pages independently visually read (11/133 alternate).
  Exact controls confirm the Leibniz and binary-depth report errors:1056
  denominator-cleared geometric identities and512 recursive-depth checks.
  All147 generated HTML section links inventoried;144 bodies and122 alternate
  visual pages have NO independent read credit. Under the established generated
  artifact scope convention, primary report133/133 visuals, both complete texts
  and all alternate text-difference pages close this source/report slice with
  those limitations, not a claim of every generated byte being read.
- Plume source, scoped numerical qualification and targeted transfer comparison
  COMPLETE with native/generated-container limits. Sole retained production
  edit is the qualified bounded Display search, unchanged this turn. Broad
  ecosystem goal remains ACTIVE.

### numbers / ERA continuation — 2026-09-06

- Corrected the supplied README-only haskellcats catalogue to the implementation
  named by official Hackage metadata: jwiegley/numbers. Both pinned and fully
  read: actual19 tracked files plus catalogue54lines. No original source edit.
  HEAD differs from the published version tag despite unchanged version text.
- Numerical qualification, native tests, benchmarks and Hyper transfer review
  remain OPEN. Static suspects are not yet presented as executed failures.
- All19 source files/1786lines plus the54-line catalogue now reconcile with
  pinned hashes. GHC9.6 native build fails missing Fixed deriving constraints;
  a constraint-only isolated bridge builds atO0/O2. Untouched upstream property
  bodies pass10000 generated checks each (two properties) in compatibility build.
- PerO0/O2,117304 ordinary approximation controls pass.99 basic failures cover
  fixed-precision decisions, rounded rather than truncated integer extraction,
  wrong atan derivatives, false constant-zero pruning and scaled-zero equality.
  All296 elementary requests finish,12 violate the computable approximation
  contract; all296 CF requests finish,63 exceed stated absolute epsilon.
  O0/O2 finite outputs and verdicts agree. DirectedMPFR oracle passes43 exact
  selfchecks. Unchanged Hyper passes1184 elementary/history and219 exact sign/
  truncation/history checks perdebug/release; releaseMemcheck0 errors.
- The transfer review found a separate Hyper bug: cold atan(4*sin(100)) enters
  PrescaledAtan with a negative argument whose magnitude exceeds1, since the
  rough dispatch checks only its upper bound. Five-second reproduction caps;
  frozen18-request cold/warm corpus has4 cold large-negative caps and14 MPFR
  passes, including all warm partners. Baseline binarya7f436d72b107a7ade1f4adab337af1baf14c3974485377eab682ec01ed69d74
  is retained. This is not a benchmark speedup or a retained fix yet.
- Next priority: signed atan range-dispatch regression and repair, including
  structural magnitude-only preconditions, followed by full gates and paired
  performance/allocation/size measurements. Numbers target and overall goal
  remain ACTIVE/OPEN. No new production edit selected this turn.
- TURN CHECKPOINT: PROGRESS. Plume closure validators (main, instrumented,
  source-parser, late, boundary, report-visuals, author-report) all exit0 and
  preserve frozen numerical/benchmark evidence. Numbers source inventory and
  numerical validator exit0. Every owned build/test/runner session has a terminal
  result; no owned process remains running. Hyperreal still has only the
  previously qualified Display edit, SHA4b9d969fb4020b45f78d1feb38dc6d254233165f06d839e1ea144c4f3df916cf.
  Concurrent Hypercurve edits to BLOCKER_REGRESSIONS.md and src/bezier_offset.rs
  were left untouched; Hypersolve remains clean. No commit/push, donor source
  edit or evidence deletion. Next turn begins with the frozen Hyper atan failure,
  not a restart of finished Plume/source reading. Overall goal remains ACTIVE.

### numbers / Hyper atan repair continuation — 2026-09-06

- Previous turn classified PROGRESS: complete source reading, independently
  qualified donor failures, and frozen Hyper cold/warm nontermination corpus.
  Current Hyperreal remains bd92d87 with only the owned Display edit; no pending
  owned process. Repair review also examines estimated versus exact magnitude
  shortcuts; no planning hint will be treated as a proved series domain.
- A second public baseline confirms estimated-domain nontermination:
  sqrt5/8+32*sqrt7/64 has retained planning magnitude<-1 despite value>1.
  Three fast dispatch regressions fail on old code: negative rough input,
  underestimated positive sum, and valid exact magnitude with unknown sign.
- Retained repair uses only exact magnitude for structural kernel shortcuts,
  requires positive sign for the large reciprocal shortcut, proves|x|<=1/2
  with |approx(-4)|<=7, and sends separated negative samples through odd
  symmetry. No new node/API/dependency/field. Five retained regressions include
  601valid coarse-cache variants/2404MPFR checks and aborted-cache protection.
- Full Hyperreal tests pass728default-debug and833release-all-features;
  independent public atan controls pass6456perbuild. All18prior cold/warm
  cases now pass perbuild, plus1184elementary/history and219exact controls.
  Release6456-grid Memcheck0errors, fmt and all-feature library Clippy pass.
  Downstream library passes19Hyperlattice+242Hyperlimit+3Hypertri+432Hypersolve
  +893Hypercurve=1589; Hypercurve1ignored, terminalexit0 in581.51s. Concurrent
  Hypercurve edits remain excluded from claims about post-compilation state.
- Frozen144benchmark process observations (128post-warmup),256qualified
  outputs/process, CPU6paired alternating order. Rational/small-opaque
  intervals include parity. Measured CPU cost increases about4.3%known-small,
  19.4%positive-opaque,15.4%warm-negative and10.3%small-estimated-sum. Additional
  allocation on two families is explicit in atan-README.md. File-64B/text-68B/
  bss+64B/loaded-4B is effectively unchanged size. No baseline timeout ratio,
  cross-language throughput or peak-memory claim. Retained for exactness and
  completeness under the requested priority order, not a claimed speedup.
- Adjacent range audit confirms more OPEN Hyper failures. For valid x≈.9909229
  built as sqrt5/64+740*sqrt7/2048, asin and atanh miss their32bit approximation
  bounds by about10.40 and3.59units. Frozen80requests/build yield42passes,
  debug37numerical failures+1coefficient-overflow exception, release38failures.
  Also77asinh/acosh log-boundary probes have8coarse-output failures from the
  unchecked ln_1p kernel. These methods are NOT repaired or claimed complete.
  Next: repair and qualify the remaining small-series/log1p domain gates.
- TURN CHECKPOINT: PROGRESS. Atan repair retained uncommitted with all owned
  builds/tests/benchmarks at terminal results. No process remains running.
  Existing Display edit is unchanged. Retained roots source SHAe63ecc1bdd838d6744c2c9b3324dea620c3c2fd1a31f179a6d02f54013efe11f;
  tests SHA67324eac110694b13d92722b1f778710923ffc76a47ebc8fb455d0f94e9be1b3;
  kernel-comment SHA65ceb888d05a40e7d8022aa8b27f4ac1ee4022329a867f65801217de8c39a110.
  Frozen before/after benchmarks, retained-source snapshots and neighboring
  failure baselines are preserved. Donor/atan validators exit0; no source or
  evidence deletion, no commit/push. Hypercurve's user-owned offset work was
  not edited. Overall goal stays ACTIVE, with numbers and remaining references
  open; next turn starts at the confirmed asin/atanh and log1p domain failures.

### numbers / inverse-series gate repair continuation — 2026-09-06

- Previous turn classification: PROGRESS (qualified atan fix retained; frozen
  neighboring failure corpora). Goal remains ACTIVE; no closure claim.
- Rechecked Hyperreal HEAD bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c and the four
  previously owned modified files. Read repository AGENTS.md. No unrelated
  edits, commits, pushes, source deletions, or external agents.
- Checked approximation/node representation and range-selection callers.
  Serde skips acceleration facts and caches but retains prescaled variants;
  valid new construction must therefore justify their arguments independently
  of planning hints. Generic ln_1p reaches PrescaledLn without reduction,
  including public Real::ln_1p, not only inverse-hyperbolic identities.
- Working first on asin/atanh certified-magnitude dispatch. Added a standalone
  public-API benchmark with 4096-bit outward MPFR qualification outside timing,
  frozen before binary and paired/fresh-process runner. Initial harness compile
  rejected private shift_right calls; replaced them with public exact-rational
  multiplication and rebuilt successfully. No donor source changed.
- ln_1p repair remains OPEN: planned demand-time range proof must precede the
  coarse zero shortcut and include overlap with normal ln's reduced residual
  interval to avoid boundary re-reduction loops. No log code change yet.
- RETAINED: sample-reusing asin/atanh domain guards, not constructor rejection
  of uncertain inputs. The initial exact-fact-only constructor candidate fixed
  numeric errors but failed two existing tiny-dispatch tests and substantially
  slowed a pilot small-sum control. Separate six/eight-bit probes also added
  avoidable fallback/work. All candidate source/binary/log evidence is preserved;
  final kernels reuse their already-required operand sample to prove |x|<=1/8
  using only its bit length. Estimates still choose a lazy schedule, never proof.
- Coarse asin zero is valid on its entire mathematical domain; coarse atanh
  now proves smallness or falls back. Direct prescaled/legacy serde nodes are
  covered, and abort does not poison caches. No new API/node/field/dependency.
  Incremental production +46/-3lines and204 test lines over the atan checkpoint.
- Final qualification: all80 original requests/build now pass (160 total),
  replacing debug37 numeric failures+1overflow and release38 numeric failures.
  Native directed-MPFR tests cover46050 coarse-cache boundary checks plus624
  reused working-sample checks/build. Full debug735 / release-allfeatures841
  passes incl19/24doctests; fmt and allfeatures--lib Clippy-Dwarnings exit0.
  Current-source --lib passes: Hyperlattice19, Hyperlimit242, Hypertri3,
  Hypersolve432 (696). Final debug Memcheck three tests pass in191.83s, exit0,
  zero errors; leak checking disabled and no peak-memory/leak-freedom claim.
- Benchmarks:270 frozen observations,240 post-warmup,256 qualified outputs per
  process, CPU6paired alternating order,8pairs/family,4096outwardMPFR outside
  timing. All14 control families have identical before/after allocations/bytes.
  CPU broadly near parity; noisy bootstrap intervals and concurrent host work
  disclosed. No general speedup or ratio against wrong results. Repaired256row
  CPU medians250.37ms asin/167.53ms atanh. Binary file+936B/text+816B/data+48B/
  bss-880B/loaded-16B; effectively unchanged total loaded size.
- Evidence: numbers-qualification/neighbor-README.md, analyze-neighbor.mjs,
  analyze-neighbor-bench.mjs and neighbor-* logs. New scalar validator and
  preserved donor/atan validators all exit0. Frozen selected source hashes:
  roots ec631190625f987238bf7d1f76a2f9e88633f42671c0e1870e63ad28f65282ae;
  inverse_trig 1e24d4ea4f51ef1340e3314a5d552dfbf3cc6bdcd3a34808f59eb12e488f03e2;
  inverse_hyperbolic b4ec61a2cd4f5da23893a5f3b1596a740fcaa169d39a875bd396c63e7fa875be;
  tests 4d34d411e181b77215d64045ce743ce3daa0e76c77a8966a344ec3edceec5a51.
- At18:52:36UTC the older-candidate Hypercurve run remains live in exec session
  96156, log numbers-qualification/neighbor-hypercurve.log, pinned CPU8. It
  began before sample reuse and must NOT be credited as final-source coverage.
  Last visible test: rank_independent_chord_normal_circle_partitions_folded_rational_overlap.
  Poll it to terminal, then qualify final source; preserve concurrent user
  BLOCKER_REGRESSIONS.md/src/bezier_offset.rs edits. All other owned processes
  are terminal. Broad goal ACTIVE; no commit/push/deletion. Next also repair
  the frozen ln_1p failures. Large-precision series coefficient/counter limits
  remain a separate unproven completeness question, not a closed claim.
- TURN CHECKPOINT: PROGRESS. Scalar asin/atanh repair retained uncommitted,
  Display/atan retained unchanged in semantics. Numbers and remaining reference
  queue stay OPEN; no complete/blocked goal update is warranted.

### numbers / ln_1p range repair continuation — 2026-09-06

- Previous turn classification: PROGRESS. Rechecked Hyperreal HEAD/worktree,
  retained changes, repository AGENTS.md and the actual ln/ln_1p kernels.
  Hypercurve session96156 was polled and remains authoritatively live; do not
  restart merely because it has not yet yielded a terminal result.
- Working on unchecked PrescaledLn for public Real::ln_1p and inverse-hyperbolic
  identities. Existing77case frozen baseline has8coarse-output failures.
  Plan: reuse the series operand sample as a domain certificate and use an
  explicitly contracting fallback, preserving lazy construction and avoiding
  extra sampling on ordinary fine-precision requests. No log production change
  yet. Full ecosystem scope remains ACTIVE, no agents/commits/pushes/deletions.
- Implemented candidate: certify |x|<=1/2 from the existing working sample's
  bit length, before either the odd-power series or coarse zero shortcut.
  Fallback2*ln(sqrt(1+x)) contracts ambiguous boundary residuals and uses normal
  ln scaling, without widening the local kernel's convergence contract or
  evaluating during construction. Public ln_1p(-255/256) regression fails before
  and passes after. No API, node, field, or dependency addition.
- Initial old-candidate Hypercurve process was verified live at PID558752,
  parentcargo554735, exact target9c41c5e0b21271d9, roughly28minutes at99%of one
  CPU. Stopped ONLY this obsolete audit-owned child with SIGTERM; session96156
  is terminalexit101 from signal15, not a numerical test failure. Log retained.
  Replaced with final-source run session3209, CPU8-15/eighttestthreads, log
  numbers-qualification/log-hypercurve-final.log. Hypercurve user commit now
  5156f67a4afc651a6f655d4ac2ba8a2317a7c3ff, clean at launch; offset source SHA
  f0cbf167f7e512efc74c6050f5db70096e52aee1dfb0e83526c950ce737fe88a and blocker
  doc SHA369d215907b8b451cc43bcba153dde69677c6cd0071b44d989bc83fa41cc8a26.
  Concurrent user changes were neither reverted nor edited.
- While native qualification ran, advanced the next donor in the full queue:
  cloned and pinned haskell-constructible46d760c, verified all4regulartracked
  files and noAGENTS/gitlinks/symlinks, independently read all490physical lines
  (module423+license29+setup2+manifest36), saved hashes/ranges and semantic notes.
  Inventory validator passes. Unchanged GHC9.6.7 initial check lacks the three
  declared external dependencies. Native arithmetic, field-join benchmarks and
  Hyper transfer comparison stay OPEN; no unexecuted hypothesis called a bug.
- RETAINED ln_1p repair: production +28/-11lines,203newtestlines. All77frozen
  asinh/acosh requests pass per build, replacing8coarse failures/build. Final
  build also passes the previous80asin/atanh cases/build. New MPFR checks/build:
  48public+624working-sample+224extreme/history+3200public/opaque grid+90estimated
  sums=4186. Debug741 / release-allfeatures848passes, incl19/24doctests. Format
  and Clippy allfeatures--lib AND --all-targets-Dwarnings exit0. Six log-domain
  Memcheck tests pass in38.35s, zeroerrors; leak checking disabled.
- Final-source --lib results: Hyperlattice19,Hyperlimit242,Hypertri3,Hypersolve432,
  Hypercurve897passed1ignored (604.24s); total1593passes. Session3209 terminal0.
  Hypercurve HEAD5156f67 and both launch/end source hashes matched; only a user
  PERFORMANCE.md edit existed at that end check. A later user src/bezier_offset.rs
  edit is outside this frozen qualification snapshot and was not touched.
  Initial downstream CPU16affinity attempt failed before executing tests;
  error retained, rerun on valid CPU7 passed. No numerical failure hidden.
- Log benchmark198frozen observations/176postwarmup,11families,8paired rounds,
  256outputs/process and4096directedMPFR outside timing. Selected moderate asinh
  ratio1.38860 CI1.25412–1.48961, calls72196→58820, bytes2516896→2245536;
  moderate acosh1.18307 CI1.04795–1.21573, calls54152→53384, bytes1940288→1930048.
  Ordinary controls near parity with unchanged allocations. No general speedup,
  cross-language or peak-memory claim. Binary file+888B/text+712B/data+24B/
  bss-696B/loaded+40B. No new node/API/field/dependency and no donor code copied.
- New log and preserved neighbor/atan/donor validators exit0. Frozen log source
  hashes: kernel eb05426db39726dfc3b5e5e121850656fe7e6fd03c6bade0fe0533779319952b;
  node 9874420245a129a67ab7e38fd2a87befa0bf418dcafa9b3f177600b30a2eb971;
  tests 222605dde1cf69b3d7eeb5d004cfe8c7a63133b9c8f470734d77256ec313bc6c.
  Details: numbers-qualification/log-README.md, analyze-log.mjs, log-* evidence.
- CONFIRMED NEXT DEFECT, not repaired: public asin(1/16) and asinh(1/16) at
 192000bits both panic in debug on i32coefficient multiplication (exit101);
  both release answers are numerically disjoint from the correct enclosure
  (probeexit2). All four184000bit controls pass. Precision-matched MPFR uses
  requestedbits+256, exact dyadic input verified. Eight probes, no60secondcaps.
  Frozen source SHA532e283afdbd070a6dd0d53b0def3c62293b97a4e2a32c94e0eb5e1cf5839069;
  beforedebug393332456d20a847d61b9216568dff99782bcb5084f90ddb21ac710f04b01546;
  beforerelease3baa4a47b160aa9fa1bc412a452318890183e5052d590942c0c7296264d55215.
  At n23171,46341² exceeds i32::MAX. Four recurrences (rational/generic
  asin/asinh) share the coefficient formulas. Next repair must cover all four,
  qualify with matching-precision oracles, and bench/size-check ordinary paths;
  do not impose an arbitrary precision cutoff. series-limit-README.md and
  analyze-series-limit.mjs preserve all evidence; validator exit0.
- TURN CHECKPOINT: PROGRESS. Log repair retained uncommitted, prior Display/
  atan/inverse-domain edits preserved. All owned processes are terminal; no
  live handle remains. No commit/push/source deletion. Native constructible
  dependencies/qualification and the verified coefficient defect are the next
  concrete actions. Numbers and the full reference queue remain OPEN; goal
  ACTIVE, no completion/blocked update justified.

### numbers / coefficient-width repair continuation — 2026-09-06

- Previous turn classification: PROGRESS. Rechecked Hyperreal HEADbd92d87 and
  the seven owned modified files, AGENTS.md, all four coefficient recurrences,
  and the frozen184000/192000bit probe evidence. No owned live process remains
  from that turn; current source still uses i32coefficient products.
- Plan: form coefficient magnitudes in u64 for the full positive i32index
  range; apply asinh's alternating sign at BigInt level. This avoids introducing
  a smaller artificial precision limit and permits a simple integer-bound
  proof. Add high-precision rational/generic regressions for all four paths,
  freeze fresh before controls, and qualify ordinary/high-precision costs.
  No coefficient production edit yet; full ecosystem scope remains ACTIVE.
- Candidate implemented in all four recurrences: one inline u64 magnitude
  helper, asinh sign applied to BigInt after positive division. Full-positive-
  i32 coefficient bound documented and tested against u128 at eight boundary
  indices. Four high-precision regression functions fail before in both builds
  (debug overflow, release non-enclosure); after all four pass for both signs.
  Independent public probes at184000/192000/256000/524288bits pass in both
  builds, sixteen terminal requests with no60secondcaps.
- Fresh benchmark baseline includes the retained log/domain repairs; all198
  alternating before/after observations completed and all outputs MPFR-enclosed.
  Initial full Hyperreal numerical tests pass, but doctest linking fails on
  /tmp Disk quota exceeded in both sandbox and escalated retries. Preserved
  failures; testing again with an isolated workspace TMPDIR, no deletion of
  shared temporary files. Clippy/wasm32 checks and benchmark analysis in progress.
- Scalar qualification complete:746debug/853release-allfeaturespasses, including
  19/24doctests after using workspace TMPDIR. Clippyallfeatures/alltargets and
  fmt/wasm32--lib checks exit0. Memcheck four new wide-coefficient regressions
  (eight signed requests) passes119.04s, zeroerrors, leak checking disabled.
  Four small downstream crates pass696tests. Current Hypercurve full run901tests
  remains live, with two failures on its user's edited HEAD149bea52c9c16797001fee2a95cf1dc739fe82e5.
- ISOLATED Hypercurve failures: same source copied to a private six-crate mirror,
  ONLY coefficient change restored there to the frozen prior hashes. Focused
  before AND after runs each fail the same two tests at the same assertions:
  nonrational-chamfer cache-entry sizeof check, transverse-chamfer comparison
  Uncertain(Predicate) versus Decided(Greater). These failures are not introduced
  by the coefficient repair. Original worktrees and user edits untouched.
- Benchmark analysis198observations/176postwarmup/41508MPFR-enclosedoutputs:
 184000bit allocation calls68995→46001 and cumulativebytes1323590176→1058953032
  for bothasin/asinh. Ordinary pairedmedianratios0.96817–1.02229; somewideCIs,
  no universal performanceclaim. Asinhighratio1.17321CI1.04452–1.23960;
  asinhhigh1.01235CI0.98525–1.11714. File+1064B,text+976B,dataunchanged,
  bss−992B,loaded−16B. Correctness/completeness and measuredallocationbenefits
  justify retention; full details coefficient-README.md and analyzer.
- RETAINED coefficient checkpoint validated: analyze-coefficient, its198row
  benchmark analyzer, and preserved series-limit/log validators pass. Current
  full Hypercurve ended898passed/2failed/1ignored in714.96s; both failures
  already isolated as unchanged before/after. A later user bezier_offset.rs
  edit changed its hash at finalcheck and is not claimed qualified. Source
  mirror and both testbinaries are frozen. With explicit approval, removed
  ONLY two duplicate UI target/ build caches accidentally copied into that
  mirror(~12GB); originals and source/evidence preserved, mirror now46MB.
- Constructible native progress: downloaded/pinned binary-search0.0,
  complex-generic0.1.1.1,integer-roots1.0.4.0. Original source/dependencies
  unchanged; direct GHC9.6.7 compile passes after disabling ccache and choosing
  modern ghc-bignum backend. This bypasses old complex-generic Cabal upper
  bounds, not a successful unmodified Cabal solver build. Selected dependency
  read credit5files466lines is separate from donor4files490lines.
- Both O0/O2 initial boundary probes99/109pass,10fail: five negative irrational
  properFraction sign violations and five nonfiniteDouble views of exactsqrt2
  represented as sqrt(2*10^(2k))/10^k,k154/155/160/200/500. All exact scaled
  identities pass; this latter finding is a floating-view limitation, not
  exact-object corruption. Hyper passes31matching controls/build, including
  exact128bit Computable and finitefloat checks against4096directedMPFR.
  No new Hyper production change warranted from those boundaries. Frozen
  source/binaries/logs and details in constructible-qualification/boundary-README.md.
- TURN CHECKPOINT: PROGRESS. All owned test/build/probe processes terminal.
  Coefficient repair retained uncommitted with earlier Display/atan/inverse/
  log repairs preserved. No agents/commits/pushes; only approved duplicate
  build-cache cleanup above. Next: constructible broader exact/field-join
  tests and benchmarks, norm/square-membership transfer review. Numbers and
  the full reference queue remain OPEN; goal ACTIVE, not complete or blocked.

### haskell-constructible / field qualification continuation — 2026-09-06

- Previous turn classification: PROGRESS. Rechecked Hyperreal HEADbd92d87,
  its seven owned modified files, AGENTS.md, clean constructible checkout,
  frozen coefficient validator and native boundary evidence. No live owned
  process remains. All retained scalar repairs preserved unchanged.
- Re-read the donor's exact field/norm/sign/square-membership kernels and the
  corresponding Hyper quadratic-surd and bounded algebraic-separation code.
  Next qualify expanded square identities, branch selection, independent
  towers, exceptional operations and repeated field joining before selecting
  any production idea. Native narrow-field decidability is not treated as
  a replacement contract for general computable reals.
- Generated/froze1,324independent integer-algebra field identities. DonorO0/O2
  passall. Hyperdebug/release1310PASS/14UNKNOWN/0WRONG at-2048; allunknowns
  depth5inversecases. At-4096release24/28pass; at-16384bothbuilds28/28pass.
  This is a bounded proof-cost difference, not a wrong answer or missing
  tested identity. Sources/binaries/logs verified by analyze-field.mjs.
- Correctednativecontracts126/126bothO0/O2; gold/Fibonacci,AGM,17thunity,
  close-radical finiteviewdocs also pass. Initialgolden defaultedtofloat;
  initialzeroexceptions expectedDivideByZero instead ofRatioZeroDenominator.
  Both were harness mistakes; initiallogs/binary kept separately.
- Pairedfieldbench126observations/112postwarmup/56measuredpairs,7families,
  1,103,040verifiedcomparisons,allterminal0,no caps/unresolved/wrong answers.
  Fresh filebytes/objects perrepeat,CPU6,alternatingorder,CPU+wall clocks;
  timed ingestion/parsing/construction/decision together. Hyper/native median
  CPUratios quadratic.2242,tower1.2136,tower2.6600,tower3.8111,tower4 1.2069,
  tower5 2.0862,independent.3637; bootstrapCIs and RSS/binary caveats in
  constructible-qualification/field-README.md. InitialsandboxEPERM sample
  rejected; approvedrerun usednewlogs. No production speedup claim.
- CurrentHypercurve alreadyuses sharedrecursivequadraticfields,positive-sheet
  reuse,and recursive squared-magnitude signs; Hypersolve already has selected
  local-field reduction/explicituncertainty. Partialread ranges/hashes recorded
  in field-README,not full-filecredit or newqualification of userdirtycurve.
  DonorQ-base irreducibility assumptions cannot be silently carried across.
  No eagerbackend transplanted; coldscalarproofcost remains a measured lead.
- Nativeclose-radical/Pell conversion accuracy now being checked independently
  with4096bitdirectedMPFR and publicHyper512bitapproximation. API read/show,
  deconstruction and enumeration probe build underway. Target/queue stillOPEN.
- Floatingqualification completed:194signedcases/nativebuild(close14radical
  sumand96Pellresidualsbothsigns),O0/O2outputsbyteidentical. Independent4096bit
  directedMPFR checks nativeDoubleandHyperlossyviews withinrelative2^-48 plus
  publicComputable512bitabsoluteenclosures;582checks ×4native/Hyperbuild
  combinations allpass,no caps. Notcorrectrounding; prior extreme-scale donor
  failure stillstands. Details/frozenhashes in float-README.md.
- APIqualification completed:8113/8113eachO0/O2,including7944read/show/
  deconstructionchecks,161enumerationchecks,7parsercontrols,typedlogBaseerror.
  Initialharnesssyntaxerrorfixedbeforeexecution;failurelogpreserved. API
  andfloatvalidators bind expectedlabels,casecoverage,source/binaryhashes.
  No production changes fromthisslice. Coldscalarproofcostfollow-up and
  the full referencequeue remainOPEN; no claimofwhole-target completion.
- TURN CHECKPOINT: PROGRESS. All owned native/build/oracle/benchmark processes
  terminal. Inventory,dependencybytes,boundary,field/bench,API/floatvalidators
  pass; retainedcoefficientvalidatorpasses; HyperrealHEADbd92d87andallthree
  coefficientrepairsourcehashes unchanged. gitdiff--checkpasses;originaldonor
  clean. No agents,commits,pushes,deletions or productionedits thisslice.
  Next useful experiment: boundthedepth-five inverseproofcost witha cold,
  evidence-preserving certificate prototype and pairedcontrols before any
  production adoption; do notdefaulttoa global eagerquadraticbackend. Then
  continue the remaining referencequeue,includingRuffiniandthelaterformal/
  algebraic targets. Goal ACTIVE,notcompleteorblocked.

### Constructible-inspired fractional separation certificate — 2026-09-06

- Previous turn: PROGRESS. Revalidated current Hyperreal HEADbd92d87, seven
  owned dirty files, full AGENTS.md, field/bench and API/float validators.
  No production edit yet. Investigating a cold certificate that retains an
  algebraic-integer numerator AND denominator instead of eagerly taking an
  integer norm at each inverse. This may avoid degree-amplified inverse bounds
  while preserving the same scalar graph and bounded certificate interface.
- Private source-only mirror `.audit-fractional-separation.qYmpNk` (5.5MB,
  tracked working-tree bytes, no target/.git copies) now carries the prototype.
  Metadata denotes beta/gamma with both integral and gamma nonzero. Inverse
  swaps conjugate bounds; root_n uses integral gamma*root_n(beta/gamma), whose
  nth power is beta*gamma^(n-1). Numerator norm yields the unchanged final
  separation formula. Existing inverse nonzero guard and all caps unchanged.
- Initial private results: existing4algebraic tests pass; field1324/1324 at
  -2048; deepest28/28alsoat-512. Added4tests pass:84inverse identities at
  depths1–6,336freshsigned128/512bitperturbations,245negative-denominator
  root/oracle cases, inverse-bound involution and invalid-denominator rejection.
  Largest depthwise bounds13,28,56,112,224,448bits. Full private debug and
  release/allfeature suites nowrunning; production remainsunchanged pending
  qualification and paired before/after measurements.
- Prototype pairedbench:162observations/144postwarmup/72pairs/4,508,640verified
  comparisons. After/beforeCPU ratios tower3.7427,tower4.4827,tower5.3037;
  shallow/nonzero controls near1withoverlappingCIs. Candidate promotedto
  production onlyafterprivate fullsuites passed. Runtimechange ismetadata
  invariant+inverseswap; no scalar/cache fields,dependencies,APIorcap changes.
  Separate includedtestmodule leaves earlier coefficienttesthashunchanged.
- Retainedsource tests:750debug and857release/allfeatures passed; Clippyall
  features/alltargets andWASMcheck pass. Full downstream(defaultincluding
  integrations/docs) Hyperlattice202,Hyperlimit348,Hypertri7,Hypersolve796pass.
  CurrentHypercurvefullsuite902pass/0fail/1ignored,632.74s; launchsourcehashes
  offsetf37d379,region81f7509,policy6f9ede2 unchangedatcompletion. No claimthat
  this alone repaired earlier failures fromdifferentuser-source snapshots.
- FuzzbuildinitialfailurewasccacheRead-onlyfilesystem; separatelyapproved
  retryCCACHE_DISABLE=1 passes. Originalfailurelogretained. Baseline6-crate
  source-onlymirror `.audit-fractional-control.pXHyjP`45MBpreserves current
  sourcewithoutbuildcaches. No edits todownstream/userfiles.
- Allocator-instrumentedbefore/afterchecks3repetitions×9families complete;
  analyzingcalls,cumulativebytes,andpeakrequestedlivebytes separatelyfrom
  RSS/timing. Finalproduction,identical-harnesspairedbench nowrunning; initial
  private-harnessbinarysize comparison isnotusedforretention claims.
- Final identicalproductionharness benchmark complete:72measuredpairs,
  4,508,640verifiedcomparisons;CPUafter/before tower3.7339[.7015,.7603],
  tower4.4865[.4786,.5030],tower5.3009[.2975,.3058]. Shallow/transverseCIs
  overlap1; no universalnoregressionclaim. Allocationchecks54observations/
  19,224comparisons stable:depth5calls115983→48239,bytes10919353→2147976,
  peakrequestedlive57265→22728. One-bytepathlengthdifferences inunchanged
  controls excludedfromsavingclaims. Finalbenchfile-160B,text-172B,data0,
  bss+160B,loadedtotal-12B; notagenerallibrarysizepromise.
- Candidate nowkept forboundedexactdecisioncompleteness plusdeepperformance/
  memory improvement. Public fullcorpusat-512:baseline1296PASS/28UNKNOWN;
  retaineddebug/release1324PASS/0UNKNOWN/0WRONG. Extraall-target/allfeature
  benchmark-smoke gate remains LIVE, session72439, currentlymillion-bitGCD
  cases. It regenerated benchmarks.md and dispatch_trace.md (cleanbefore);
  archiveandrestore onlytheseownedgeneratedreportchanges afteritfinishes.

- Extra all-target/allfeature gate terminal0:835tests and1439benchmark smoke
  cases acrossall10declaredbenchmarkexecutables. Generatedbenchmarkreports
  archivedSHA2566bc37b89afc807a2f503bf4ba6721aaca2566052d45227f67be5cd4902f4da97;
  onlytheownedgeneratedreportdiffs restoredwithapply_patch. Bothreportsmatch
  prerunbaselinebytes; no reportchurnleftinproduction. Allownedprocessesterminal.
- Constructible scopedclosure:4files490linesreadplusdeclarednative/transfer
  qualification complete; verify-closure.mjs aggregatesallstagevalidators.
  Retainquotientseparationcertificate,priorrepairsunchanged,donorunchanged.
  Fullreferencegoalnotcomplete; NumbersremainingquestionsandlaterqueueOPEN.

### Ruffini source audit opened — 2026-09-06

- Verified supplied public repository and cloned untouchedpin
  82d552fee22d92e493936183fab8672517694e56.245trackedfiles; noAGENTS.md.
  RootREADME,fullparentPOM,license,citation,ignore,bothworkflows,realsPOM and
  ConstructiveReal.java readcompletely. Gitblob/SHA256inventory and explicit
  source-readcredit in RUFFINI_FILE_INVENTORY.tsv/RUFFINI_READ_COVERAGE.tsv.
- Source-derived probes toqualify:negative-reciprocal search, signedmagnitude
  multiplication budget, cachedcoarsening,17bitequals/hashcontract,decimalvs
  binary import,andformatting/precisionarithmetic. No executedfailureclaim yet.
  Remainingfiles/nativebuild/tests/bench/HypertransferOPEN; fullgoalACTIVE.
- Inventoryvalidated245UTF8regularfiles/26,234physical lines; actualreadcredit
  9files594lines. InitialGitquotedUnicodefilenameinventoryfailed,correctedby
  separatecore.quotePath=falseindex; originalindexpreserved. No sourceedit.
- TURN CHECKPOINT: PROGRESS. Worthwhile scalartransferqualifiedandkept,
  pinnedConstructibletargetclosed,Ruffini auditopened. Allownedprocesses
  terminal. No agents,commits,pushes,deletions. Earliercoefficientrepairsand
  userdownstreamedits preserved. Next: Ruffinireals/common/integer source
  reading, native signed/cache/equality contracts andfull245-fileinventory.
  GoalACTIVE,notcompleteorblocked.

### Ruffini scalar and shared-contract continuation — 2026-09-06

- Revalidated retained fractional certificate qualification and current owned
  Hyperreal edits; no new production changes. Original Ruffini pin remains
  clean. Completed reading the other seven real-module Java files and common
  POM, then all fifteen common abstractions, Pair, Multiply and Power, with
  untruncated numbered source. Explicit inventory credit updated separately.
- ConstructiveReals delegates approximate equality into a Field interface whose
  Set contract explicitly says true iff equal. Shared Multiply has separate
  signed-int/BigInteger recursions requiring boundary probes; Power handles
  negative exponents but int MIN_VALUE negation needs qualification. Real and
  complex coordinate/norm/RK4 layers are ordinary double numerics, not exact
  integration. Pair is mutable; no claimed concurrency guarantee or race result.
- Native tooling: javac21.0.11 and Java25.0.4 available; mvn not on PATH. Plan
  direct --release16 compilation of unchanged donor sources with pinned parent
  dependency versions, not a claim of successful original Maven lifecycle.
  No deployment/signing/publication, donor edits, or agents authorized/used.
  Full 245-file Ruffini audit and broader reference goal remain ACTIVE/OPEN.
- Read coverage now76/245files,3,810/26,234physical lines. Full additional
  per-file notes in RUFFINI_FILE_NOTES.md; allrealmodule,sharedabstractions,
  selectedinteger/vector/algorithm/helper/test files read without truncation.
  169filesremainUNREAD,notexcluded. Compiledunreadfilesreceive no readcredit.
- Directnativebuild96unchangedcommon/realsJavafiles passes withpinnedGuava,
  CommonsMath,JUnit,Hamcrest. SandboxDNSandjavac/CargoEPERM failures retried
  withapproval; originalcompilerdiagnostics preserved. NativeJITand-Xint
  boundaryruns agree:1575rationalcases,1379pass/196requested-boundviolations
  (122callbackmultiply,74publicfactorymultiply); noadd/neg/positiveinverse
  failures. ExactBigIntoracles,notf64decisions. Errorsareprecisioncontracts,
  not blanket claims of nonconvergent exact-real sequences.
- Bothmodesconfirm nontransitive17bitequality,equals/hash mismatch,negative
  reciprocalpositive-onlysearch,5negativeintscalezeros/5BigIntegerstack
  overflows,andintMINembedding/powerrecursion. Decimalestimate20/91bound
  violations;7decimalimportcontrols pass. Binary-vs-decimal semantics differ
  fromHyperandare notthemselves classifiedasdefects.
- Rejectedinitialcache-coarseningsuspicion:integer-floorrescalepreserves<=1
  error byintegrality;300signedfloor/ceilcachecontrols per mode pass. This
  doesnotproveprecisioncounteroverfloworthreadsafety. equalsbypassescache:
  callbackcounts1,2,2,12afterconstructor/fine/10coarse/10equalsqueries.
- OriginalcommonJUnit:11run,10pass,1Jacobiargument-orderfailure inbothmodes.
  Sharednativeprobesconfirm loggingwrappersdropnonzero2^-17,Barrettnegative
  multiplesbreakcanonicalzeroequivalence,zeroFractioninverse1/0,BitLength
  positivebase-caseStackOverflow,andBinaryGCD(0,1)fixedpoint/2secondcap.
  Barrettpositivefast-range731/731pass;negativeboundedmultiples60fail;
  otherrange1035pass/179fail. Alltimedoutownedprocesseskilledandterminal.
- DeterministicDAGresourceexperiment:depth20shared21nodes retains4,194,301
  root-stringcharacters;leafestimator21constructioncalls,22afterrefinement,
  still22after100coarsereads. Stringexpansionrejected; usefulcacheeffect
  alreadyinHyper. Notalatency/RSSbenchmark orcross-languagespeedclaim.
- CurrentHyperpubliccontrolsdebug/releasebothpass1575matchingrationalcases
  plus584boundarychecks,including480signedirrational4096bitdirectedMPFR
  enclosuresafter768bitcachewarm. Full81filecurrentHyperhashsnapshot stable;
  no newproductionchanges. Previousretainedrepairqualification revalidated.
- TURN CHECKPOINT: PROGRESS. `ruffini-qualification/verify-checkpoint.mjs`
  aggregatesinventory,source/class/dependencyhashes,nativeexactoracles,and
  Hyperchecks. Nativeoriginalsourceunchanged; noagents,commits,pushes,deletions.
  Next:remainingcommonmatrix/FFT/utilandintegerfiles,thenrestofthe169-file
  inventory;qualifyremainingprecision/domain/vectorcontracts. Potential
  borrowed-cache-rescale optimization isunimplemented/unqualified,notretained.
  FullRuffiniandwholeecosystemgoalACTIVE/OPEN;notcompleteorblocked.
- Finalconcurrencycheck:oldfractionalaggregatepassedatturnentrybutnowcorrectly
  rejectschangeduserHypercurveoffset. HEADnowa46b906a7167542b9decc0bd27a72cca38eeb34c,
  offsetSHA2564e9f51163bff9b5a9e298ab58fe5e09f0cd8313cd4a7ed095fcafb108a16f082;
  region81f7509andpolicy6f9ede2remainoldbytes. NewRuffini/Hyperreal81filesnapshot
  validates;previousretainedproductionhashesunchangedandgitdiff--checkpasses.
  Noauditdownstreamedits.Noattributionofold902passresulttonewsource,andno
  modificationoffrozenvalidatorstorelabelthisasrequalified. Notablockerfor
  continuingdonoraudit. Allownedprocessesterminal.

### Ruffini matrices, transforms and remaining common source — 2026-09-06

- Previous turn classified PROGRESS. Revalidated the Ruffini scalar/shared
  checkpoint against current donor bytes, compiled classes/dependencies and all
  81 Hyperreal source files. Hyperreal HEAD bd92d87 and owned edits unchanged;
  donor checkout clean. Prior Hypercurve downstream snapshot caveat preserved.
- Read the remaining Fourier-transform, matrix algorithm/representation,
  collection/utility, functional and exception files in complete numbered
  chunks. Source-derived matrix dimension/equality, lazy-view, sparse-builder,
  QR/projection and integer/collection boundary questions require qualification.
  No production change; full reference scope remains ACTIVE/OPEN.
- Completed all common and integer source reads, then all parser, permutation
  and finite-field files. Explicit credit now148/245files,8,122/26,234physical
  lines;97files remain UNREAD, not excluded. Every newly read file has notes in
  RUFFINI_FILE_NOTES.md. Parser/finite-field hypotheses remain unqualified and
  must not be presented as executed findings. Donor checkout remains unchanged.
- Native matrix/transform/integer qualification:2,477probes per JIT/interpreter
  mode, identical outputs; original integer JUnit2/2pass in both modes. All
  1,296ordinary products,128Gram products,120nonempty determinants and69valid
  DFT/inverse cases pass. Strassen186/264pass,78padded cutoff>1cases throw;
  empty determinant returns0. Exact BigInteger oracles, not f64 decisions.
- Boundary qualification confirms rectangular equality/collapse failures,
  complex identity-inverse rejection, sparse-builder aliasing, complex
  projection sign, tall QR, vector/collection/dimension boundaries, dropped
  limb carry/null multiply and int MIN/division issues. One-variable congruence
  grid36zero-RHS pass/408nonzero-RHS wrong; square-input sqrt58pass/17unsupported
  p17exceptions. Public mutable-copy independence control passes; aliasing is
  not generalized to that separate path. Full logs and source/class hashes kept.
- Current Hyper matrix harness420checks each debug/release, all pass: exact
  determinants, Bareiss solves, Matrix3/4 products/determinants/transposes,
  complex reciprocal, tiny strict pivots and domain guards.201source/manifest
  files stable before/after. No new production change or claimed generic QR API.
  Initial Java overload and Rust missing-Default harness diagnostics preserved;
  fixes were harness-only. No full downstream rerun in this no-change slice.
- Donor matrix benchmark preliminary run completed, but post-JIT short batches
  expose process-CPU quantization. Source/logs/analysis preserved as preliminary,
  not precise timing claims. Separately versioned stable run warms before
  calibration and rejects measured batches below0.5CPU seconds; currently in
  progress. MATRIX_CHECKPOINT.md records methods, controls and limitations.
- Stable benchmark now completed and validated:6processes,216observations,
  162measured batches,2,435,328exactly checked measured products. Shortest
  measured batch1.04CPU seconds; fixed calibration and rotated positions pass
  validation. Default cutoff1 costs5.83–21.64×ordinary multiplication CPU on
  the tested power-of-two integer workloads; cutoff8 is ~1×when it dispatches
  to the ordinary base case and2.12–2.46×at size16. Wall times corroborate the
  large regressions. Shared-host/within-JVM descriptive intervals are not
  independent-process guarantees or universal crossover claims. No Hyper
  speedup claimed; no new production transfer retained. Original preliminary
  sources/classes/logs/analysis kept, not overwritten by the stable experiment.
- TURN CHECKPOINT: PROGRESS. Aggregate Ruffini validator now includes native
  scalar/shared/matrix, Hyper scalar/matrix, and preliminary/stable benchmark
  artifacts; final validation passes with97remaining UNREAD source files.
  Hyperreal HEADbd92d87 and all prior retained repair fingerprints unchanged;
  gitdiff--check passes. Hyperlattice/Hyperlimit/Hypersolve clean, donor clean.
  Earlier Hypercurve snapshot caveat retained; no old result relabeled current.
  All owned processes terminal, stable benchmark stderr empty. No agents,
  commits, pushes, donor edits or deletions. Full goal ACTIVE/OPEN, not complete
  or blocked. Next: polynomial representation/algorithms and their dependencies,
  then native parser/permutation/extension-field probes; remaining elliptic,
  class-group, demo/constant and root-SVG files remain explicitly in scope.

### Ruffini polynomial and dependent-contract continuation — 2026-09-06

- Previous goal turn classified PROGRESS. Revalidated complete existing
  Ruffini scalar/shared/matrix checkpoint and both benchmark artifact sets;
  current source/class/dependency/Hyper snapshots pass. Original donor clean.
- Read all29polynomial files/2,585physical lines, including commented Modulus,
  recursive representation and every test. One combined read was truncated;
  all affected files were rerendered completely before credit. Coverage now
  177/245files,10,707/26,234lines;68files remain UNREAD and in scope.
- Source-derived questions: sparse equality's reversed zero test, retained
  mutable builder maps, inconsistent empty/trailing-zero representation,
  reversal-width fast division, multivariate reduction sign and leading term,
  fixed-size FFT algebra versus full polynomial closure. Exact native probes
  and comparison with current Hyper paths are next; no production change yet.
- Native polynomial/dependent probes completed in both JVM modes:2,960each,
  identical outputs. All2,630valid-grid arithmetic checks pass, while dedicated
  representation/degree/division/parser/field boundaries expose documented
  failures. All10original polynomial/parser/permutation/finite-field JUnit
  tests pass; several polynomial tests only print core results, so this is
  not broad correctness certification. No donor or production source edits.
- Two public division processes hit2second caps per mode; separate exact
  callback wrappers verify1,025states of unchanged1+0x division and repeating
  F5 leading coefficients1,2,4,3. Both wrappers terminate at their explicit
  prefix budget. This separates source-confirmed non-progress from startup
  delay; all associated processes terminal. Full details in polynomial analysis.
- Hyper public polynomial controls1,520each debug/release all pass, stable
  201-file snapshot: evaluation, exact monic/nonmonic/sparse division, normalized
  gcd, bivariate division, domain and radical controls. No generic finite-field
  or parser equivalence inferred. Current existing retained divisor/strict
  trimming/balanced evaluation architecture inspected rather than duplicated.
- Polynomial benchmark initial dense family completed; next sparse family's
  ordinary multiplication was too fast for its1,048,576-loop calibration cap.
  The failure is a harness allowance, not a numerical defect. Original artifacts
  retained. A separately versioned full12-family run extends only that allowance
  to16,777,216, keeping prewarm, fixed calibration, exact verification, rotated
  pairs and minimum measured-duration gates; currently running.
- Read all4class-group files/423lines while timing ran, with per-file notes.
  Coverage now181/245files,11,130/26,234physical lines;64files UNREAD, not excluded.
  Invalid-discriminant principal construction, cache-dependent Object.equals,
  and misplaced/obsolete-import tests are pending native checks, not yet claims
  of executed failures. Full ecosystem goal remains ACTIVE/OPEN.
- Extended polynomial benchmark completed:12processes,288observations,
  216measured batches,108paired rounds,27,753,408exactly checked measured
  products; shortest batch1.15CPU seconds. No measured batch rejected.
  Cost/density matters: dense2048bit-parameter length64/128Karatsuba CPU ratios
  0.596[0.579,0.607] and0.447[0.441,0.458], wall0.595/0.451; all small-coefficient
  and all sparse families regress. These are donor-only within-JVM descriptive
  intervals, not Hyper speedups or universal crossover guarantees. Original
  calibration-cap failure preserved separately; full extended run validated.
- NEW CONCRETE CANDIDATE, UNIMPLEMENTED/UNQUALIFIED: polynomial-level Karatsuba
  for expensive dense Hyper coefficients, with measured density/cost dispatch.
  This is distinct from existing bigint Karatsuba. Next isolate the actual
  Hypersolve polynomial multiplication callers/current source; compare small,
  sparse, unbalanced and symbolic workloads as well as costly dense rationals.
  Verify exact coefficient/root/remainder behavior, allocation and binary/code
  size before retention. Do not blindly transfer the donor threshold or claim
  its speedup for Hyper. Existing borrowed-cache-rescale candidate also remains
  unimplemented/unqualified. Full reference scope is unchanged.
- Class-group native146probes per mode,80pass/66wrong, identical outputs:
  64valid-discriminant controls pass,64invalid-congruence inputs accepted;
  cache-dependent Object.equals and equal-hash failure confirmed. Separate
  coefficient Group equality, reduction, inverse/cube and order3/order7 controls
  pass. Unchanged original test direct compilation fails on obsolete imports;
  test source also lies outside default Maven test tree. No original JUnit
  pass claimed; sampling/general composition still unqualified.
- TURN CHECKPOINT: PROGRESS. Read33additional files; current181/245files,
  11,130/26,234physical lines,64UNREAD still in scope. Aggregate source/blob,
  native class/dependency, Hyper snapshot, test, progress-prefix and benchmark
  validators pass. Hyperreal HEADbd92d87, all prior retained repair fingerprints
  unchanged andgitdiff--check passes. Hyperlattice/Hyperlimit/Hypersolve clean;
  donor unchanged. Old Hypercurve snapshot caveat remains, without relabeling
  old downstream tests. All owned processes terminal; no new production change,
  agents, commits, pushes or deletions. Goal ACTIVE/OPEN, not complete or blocked.
  Next: qualify the dense/cost-sensitive Hyper polynomial candidate, then
  remaining elliptic/demos/constant/SVG source and outstanding native boundaries.

### Hyper polynomial transfer pilot — 2026-09-06

- Previous goal turn classified PROGRESS. Existing Ruffini aggregate initially
  revalidated;181/245files,11,130/26,234physical lines read,64UNREAD remain.
  Current Hypersolve clean at42f2bdb87e5f9905b95e4f95d4e5e1abdb65586e.
  Inspected actual curve_resultant convolution/trimming, square-root and
  discriminant callers, plus algebraic_mobius multiplication. No production
  change yet; donor-only crossover does not establish a Hyper improvement.
- Subsequent status shows concurrent/user Hyperreal edits to scalar_micro,
  rational/comparison.rs and rational/tests.rs beyond prior retained changes.
  Preserve them. New pilot will freeze its own current source snapshot; old
  qualification must not be relabeled as covering changed source. Full goal
  remains ACTIVE/OPEN; no agents, commits, pushes or deployment authorized.
- Frozen393-file current four-crate snapshot created for the private pilot;
  actual Hypersolve multiplication/zero predicate mechanically extracted and
  hash-checked. Three Karatsuba cutoffs and one rational density/cost gate pass
  3,245checks each debug/release, byte-identical outputs: independent integer/
  rational convolution, shape/degree/zero boundaries and radical controls.
- First allocation run fails its harness assertion that dropping a product
  must release all measured allocations. Hyper inputs retain arithmetic-cache
  results; this is not evidence of a production leak. Preserve the failed run
  and use lifetime-aware accounting. Preliminary timing is reused-input only;
  fresh-input measurements are required before any production decision.
- Read all17elliptic files/1,424lines while correctness ran, with per-file notes.
  Current198/245files,12,554/26,234physical lines;47UNREAD remain in scope.
  Elliptic findings are source-only, not executed/native or cryptographic
  qualification. No production edits or new transfer retained.
- Second-generation allocation accounting tracks whole allocation lifetimes;
  72fresh/reused profiles pass, with all measured input/output owners releasing
  their allocations. Example64×64dense2048bit gated multiplication allocates
  6,132,976bytes versus9,574,040baseline but peaks at622,848versus346,976bytes.
  This is a total-allocation/peak-live-memory tradeoff, not a blanket memory win.
  Original failed windowed-counter run retained unchanged.
- Read all14remaining executable/configuration demo files/1,030lines, including
  all documented constants in Java. Current212/245files,13,584/26,234lines;
  33UNREAD files are32numeric data resources plus root SVG, all still in scope.
  Demo findings are source-only. Repeated-node Cauchy solves are an unqualified
  structured-workload idea, not a demonstrated current Hyper improvement.
- Stable isolated Hyper study completed and validated:60processes,
  1,920observations,1,440measured batches,7,453,824fully compared products;
  minimum CPU batch0.150768seconds. All identical-code controls within10%.
  Fresh dense2048bit length64/128 gated CPU ratios0.608(0.593–0.615) and
  0.462(0.439–0.465); reused0.570and0.433. Parentheses are ranges of three
  process medians, not confidence intervals. Wall corroborates. No caller or
  whole-application speedup claimed, and no production transfer retained.
- Live rational/comparison.rs changed again during the run; frozen393-file
  pilot snapshot remains stable and valid. Historical aggregate correctly
  rejects current comparison hashfe4af8d...versus8a4138fa...; do not relabel old
  results or overwrite their snapshots. New coefficient-export oracle is running
  to check canonical numerical outputs without using Hyper's equality operation.
- Local elliptic coordinate/group-law follow-up completed:1,672probes per JVM
  mode,1,434PASS/232WRONG/6exceptions, identical output. Only7donor files compiled
  with frozen dependencies; no original crypto JUnit or general cryptographic
  assessment claimed. Four matrix resources additionally read in full (54lines)
  and all344nonempty square minors checked with independent integer determinants.
  Coverage216/245files,13,638/26,234lines;29UNREAD remain in scope.
- Read all12remaining matrix resources individually, fully and without
  truncation:1,730additional lines. Coverage228/245files,15,368/26,234lines;
  17UNREAD remain (16round-constant files and root SVG). These are numeric tables,
  not new scalar algorithms. The all-minor result above remains scoped to the
  first four matrices; no cryptographic or larger-matrix qualification implied.
- Separate larger-matrix structural checker passes expected dimensions/ranges,
  1,730unit entries,70,438two-by-two unit minors and12unit full determinants.
  Integer Bareiss and modular elimination agree;120independent Laplace controls
  also pass. No all-larger-minor or cryptographic conclusion. Strengthened local
  elliptic analyzer revalidated all frozen source/class/dependency hashes.
- Caller analysis prioritizes a smaller alternative before generic Karatsuba:
  square-root extraction currently computes Dfull(D+1)×(D+1)convolutions while
  using only coefficient D+p at step p. Root coefficients0..p are still exact
  zero then, so only indices p+1..D-1 can contribute, in unchanged ascending
  order. Across steps this is D(D-1)/2products instead of D(D+1)^2, before the
  unchanged final full-square identity certificate. This is an operation-count
  argument, NOT a timing or completeness qualification. Actual public caller
  parameter_component_bivariate_polynomial_system and existing quadratic-fiber
  bench provide an end-to-end gate; low-degree/symbolic geometry must be included.
  No implementation retained; live Hyperreal source drift must be revalidated.
- Equality-independent coefficient-export oracle complete:4,905PASS each debug
  and release, identical output. Original300-second debug timeout preserved;
  unchanged binary completes674.615seconds under1,200-second cap. Release29.750s.
  Canonical exported numerator/denominator compared through BigRational, never
  Hyper equality. Extended source/binary/lock/build/output records are validated.
- Fully read constants00andconstants01 (128+195lines), bringing source coverage
  to230/245files,15,691/26,234physical lines;15UNREAD still in scope. These opaque
  numeric round tables offer no scalar algorithm. Full ecosystem goal unchanged.
- Final transfer checkpoint at2026-09-07T02:19:42Z:12validation stages PASS;
  3historical scalar/matrix/polynomial stages reject live source drift and are
  explicitly NOT counted as passes. Native source/class/dependency evidence,
  donor timing, elliptic numerical controls, read-resource checks,393-file
  frozen pilot, correctness/export oracles and allocation/timing records pass.
  Concurrent work has restored comparison.rs to8a4138fa...but changed benchmark
  and rational test sources again. The pilot now differs from live source in
  scalar_micro.rs, comparison.rs and rational/tests.rs; historical guards now
  reject rational/tests.rs. New validator records exact expected/actual hashes
  and rejection errors; no old snapshot or guard was weakened or overwritten.
  Validator-only sandbox spawn failure and initial path-prefix/over-specific
  drift-check mistakes were corrected; none were production/numerical failures.
- TURN CHECKPOINT: PROGRESS. This turn read49additional donor files/4,561lines;
  Ruffini230/245files,15,691/26,234physical lines,15UNREAD remain. All owned
  processes terminal. Hyperreal HEADbd92d87, prior retained repair fingerprints
  unchanged, gitdiff--check passes; Hyperlattice/Hyperlimit/Hypersolve and Ruffini
  clean. User changes preserved. No new production change, agents, commits,
  pushes, deployment or deletions. Full goal ACTIVE/OPEN, not complete/blocked.
  Next: qualify caller-specific square-root diagonal work against a new current
  source snapshot and actual public quadratic-component callers; only revisit
  generic polynomial dispatch if caller timing and memory/size gates justify it.
  Then remaining14round-constant resources/root SVG, outstanding native
  boundaries, and all other unaudited references; full source scope unchanged.

### Square-root diagonal caller pilot — 2026-09-06

- Previous goal turn classified PROGRESS:49new donor-file reads and the
  polynomial transfer experiment validated, not just a status restatement.
  Current source re-inspected; concurrent work advanced Hyperreal through
  32d82589to da26e961bf76adf13829d8a26ced6cdaacb6b564. User changes preserved.
  Hypersolve clean at42f2bdb. New397-file four-crate snapshot frozen at02:28:45Z;
  actual square-root function and all seven storage/arithmetic helpers extracted
  unchanged. Candidate changes only the backward-loop known-coefficient sum,
  retaining leading-root/domain checks, strict reciprocal and final full-square
  certificate. Private experiment only; no new production change retained.
- Two /tmp build failures preserved (sandbox linker SIGBUS, then explicit host
  user quota exhaustion); unchanged code built successfully in private workspace
  .audit-square-root.C42NO5BC. No broad cleanup or user-file removal performed.
- Before running correctness, algebraic review caught a draft oracle typo:
  (sqrt2+sqrt3*x+x²)² has x² coefficient3+2sqrt2, not5. Original draft/binaries
  preserved, not claimed qualified. Separately versioned v2 oracle changes only
  validation, includes unchanged algorithms/timer and passes1,956checks each
  debug/release with identical output (652semantic cases×baseline/control/diagonal).
  Rational coefficients independently exported; accepted radicals/log identity,
  nonsquares, padding, empty/zero/negative-leading cases included.
- 48process isolated timing study completed; no owned heavy work concurrent.
  1,152observations,864measured batches,67,353,600verified calls; minimum CPU
  batch0.190103546seconds. All identical-code process controls within10%.
  Fresh D1/32dense diagonal/baseline CPU median0.947450; D8/32dense0.423598;
  D32/32dense0.118561. Reused medians0.829562,0.366636,0.111612 respectively.
  These are helper-only results, not whole-caller speedups.
- Separate lifetime-aware memory study passes96cases/288profiles; independent
  numerical oracle checked before allocation windows, all owners release tozero,
  baseline/control identical. Total allocated bytes never increase, but nine
  reused families have higher peak live allocation. D16/256sparse reused peak
  rises8,032to10,208bytes (+27.1%), calls53to108, while allocated bytes fall
  35,032to14,872. No blanket memory improvement claimed.
- Private whole-Hypersolve candidate differs only in Cargo package/paths and
  the qualified diagonal loop. Actual public quadratic-component caller passes
  81cases each debug/release and baseline/diagonal (324total), identical output.
  Independent BigRational full coefficient/factor-residual replay covers80cases;
  one radical case has certified full symbolic coefficient replay. Debugger
  probes confirm exactly one call to the intended helper in each implementation.
  Release .text falls128bytes but file size grows4,320bytes; package/symbol
  identities differ, so no production binary-size gain claimed.
- Public timing completed108sequential processes,864observations,648measured
  batches,3,876,864verified measured calls; minimum CPU batch0.478315654seconds.
  All same-binary controls within10%; no owned heavy concurrent work. Across
  twelve degree/cost/layout/lifetime families, diagonal/baseline CPU medians
  range0.982424to1.023095. D8/32terminal fresh1.007587,reused0.996176; D4/32sampled
  fresh1.015196,reused1.023095. Individual seed-group ratios0.952717–1.095397;
  same-code controls0.946142–1.044456. Wall corroborates. These are descriptive
  ratios/ranges, not confidence bounds or proof of universal equivalence.
  Scheduling uses only two of three cyclic orders: exploratory, not fully
  counterbalanced. The stated workflow includes report comparison/destruction.
- Retention decision: NO production transfer. Large helper-only gains have not
  established a worthwhile public-caller gain, and memory/size results are mixed.
  Keep private evidence, not a speculative production dispatch. Revisit only
  after representative profiling establishes a substantial helper cost; require
  fully counterbalanced timing and current-source/downstream qualification then.
  This does not claim that all possible workloads lack a benefit.
- Fully read constants02and03 individually, all256+340lines without truncation.
  Coverage232/245files,16,287/26,234physical lines;13UNREAD remain in scope.
  These numeric tables add no scalar algorithm or cryptographic qualification.
- Then fully read constants04and05 in bounded numbered chunks,408+497lines.
  Ruffini coverage234/245files,17,192/26,234physical lines;11UNREAD remain:
  constants06–15 and root SVG. Separate post-read checker for constants02–04
  passes1,004entries for dimensions, canonical decimal syntax and modulus range,
  plus15predicate controls. Its scope excludes constants05 and cryptographic
  qualification; the original constants00–01 evidence remains unchanged.
- Final scoped validator passes complete isolated/public manifests, binary/source/
  lock/output fingerprints,397frozen files,152candidate files and all234credited
  donor-file ranges. Current397-file source drift is empty. Checkpoint
  square-root-pilot/checkpoint-final-02.json preserves the no-transfer decision;
  the initial sandbox subprocess EPERM is separately preserved in final-01.
  Historical guards/snapshots were not weakened or relabeled as current passes.
- TURN CHECKPOINT: PROGRESS. This turn added4donor-file reads/1,501physical lines,
  independently qualified the square-root diagonal alternative in debug/release,
  measured48isolated+108public processes and96memory cases, and rejected its
  production transfer on the caller evidence. Ruffini234/245files,17,192/26,234
  lines;11UNREAD remain. Hyperreal HEADda26e961, Hypersolve HEAD42f2bdb; prior
  retained changes and user work preserved, gitdiff--check passes. All owned
  processes terminal; no agents, commits, pushes, deployment or deletions.
  Full ecosystem goal ACTIVE/OPEN, not complete/blocked. Next: remaining ten
  round-constant resources/root SVG, outstanding Ruffini native boundaries and
  all other unaudited references. No additional speculative timing is required
  merely to retain a candidate already rejected by this checkpoint.

### Ruffini finite-field boundaries and resource continuation — 2026-09-06

- Previous goal turn classified PROGRESS: independently qualified square-root
  alternative, completed156benchmark processes and rejected the production
  transfer on actual caller evidence; four new donor files read. Current worktree
  re-inspected, all prior/user changes preserved. No live process inherited.
- Bounded native finite-field follow-up now completes34JVM processes,3,206probes
  each default JIT/interpreter, byte-identical numerical outputs. All source,
  class, dependency, build, runner and output fingerprints revalidated. Independent
  BigInt carry-free coefficient packing, polynomial reduction, unit/square replay
  and exact trial-division primality checks validate the mathematical oracles.
  No process timeout or256draw randomized harness cap was reached.
- Canonical nonconstant extension inverse grid:546wrong values among682int/Big
  probes, consistent with the omitted constant-gcd normalization;136pass.
  Normalized AlgebraicFieldExtension control passes all332canonical probes.
  Independently certified fields of orders4,8,9,25,49 and all nonzero modulus
  scalings included. Separately,164nonzero trailing-zero representations throw
  NotInvertibleException;34zero inputs are rejected. Representation defects and
  canonical arithmetic defects are distinguished, not merged into one claim.
- Characteristic-two TonelliShanks:510elements across GF(2^n),n=1..8;16zero/one
  cases pass and494nontrivial values return incorrect roots. Its exponent is
  1<<(q/2), not2^(n-1). Separate seeded odd-prime grid passes549cases, including
  nonresidue rejection; arbitrary streams/odd extension fields are not certified.
- BigTonelliShanks:660supported small-prime representative cases give636passes
  and24rejections of zero. Four exact primes withv2(p-1)=30,31,32,35 and six
  authored squares each yield15passes/9StackOverflowErrors. The s=32 witness
  9mod184683593729 has explicit root3. Fixed-width exponent shifts feed the
  previously observed MIN_VALUE negation recursion in Power.apply(Integer).
  No cryptographic or all-large-prime claim follows from these local tests.
- Berlekamp-Rabin:249monic quadratic/seed cases over3,5,7 give133verified roots,
  110unresolved searches and6NullPointerExceptions.102unresolved inputs have no
  roots; eight have roots but finite randomized exhaustion alone is not a wrong
  answer. All six exceptions have roots; no wrong returned root observed.
  Separate249case diagnostic replay per mode confirms every iteration-limit
  exception message and traces null errors to Integers.negate. One high-square
  overflow replay per mode confirms Power recursion in JIT; interpreter top
  frames show Integer boxing. An initially over-specific diagnostic assertion
  was corrected to record that observed distinction, not to claim equal traces.
- Initial harness mistakenly admitted BigPrimeField(2/3), despite its explicit
  Barrett constructor restriction. Its failed run is retained/unqualified.
  V2 separates unsupported constructors and restricts arithmetic accordingly;
  it includes unchanged v1 oracles/other routines and adds the order49extension.
  Sandbox javac EPERM retry is separately recorded with identical class bytes.
  Repeated /tmp mount quota failures were handled through explicit approval;
  no user files or broad temporary directories were removed.
- Current Hyper source comparison confirms already-present zero/negative-root
  domain dispatch, unsigned_abs/BigInt exponent handling, eager-output budgets
  with retained exact fallback, and certified polynomial trimming/nonzero tests.
  These are inspected characteristic-zero contracts, not a new finite-field
  equivalence test. No new production transfer or performance benchmark claim.
- Fully read constants06 in untruncated numbered ranges1–144,145–288,289–432,
  433–576. Coverage235/245files,17,768/26,234physical lines; ten UNREAD resources
  remain:constants07–15 and root SVG. No algorithm in this opaque numeric table.
  Separate constants05–06 check passes all1,073entries for dimensions, canonical
  decimal syntax and modulus range; no generating/cryptographic qualification.
- Final finite-diagnostics-analysis.json validates the native/diagnostic evidence
  and235credited donor ranges; the latest397-file Hyper snapshot has no current
  source drift. Historical234-file and older validators/snapshots are preserved,
  not rewritten or relabeled as current aggregate passes.
- TURN CHECKPOINT: PROGRESS. One576line donor file completed; bounded native
  exactness questions resolved with3,206probes and250diagnostic replays per mode.
  Hyperreal HEADda26e961, Hypersolve HEAD42f2bdb; no new production change, agents,
  commits, pushes, deployment or deletions. All owned processes terminal.
  Full goal ACTIVE/OPEN, not complete/blocked. Next: remaining nine round tables
  and SVG, residual native/representation questions (including derangements and
  odd-characteristic extension behavior as relevant), then remaining references.
  Previous performance candidates stay unretained absent profile-driven evidence.

### Ruffini generated architecture resource — 2026-09-06

- Fully read the outer/mxfile/model envelopes and all103decoded SVG physical
  lines via327bounded chunks, including the PlantUML source comment. Verified
  the two encoded image copies are identical, source/chunk reconstruction and
  all artifact hashes. Rendered and viewed the554×926diagram. It has14interfaces,
  15method labels and16inheritance arrows; model metadata, SVG text/arrows and
  comment agree after documented comment escaping. Current interfaces retain
  all16edges and add OrderedSet, but package names, zero/identity/division method
  names and Euclidean norm type in the figure are stale. No new algorithm.
- REVIEWED_ASSET is distinct from READ:235source/data files,17,768lines remain
  credited; one generated12line asset is separately reviewed, nine resources
  UNREAD. This follows the original asset convention and does not claim manual
  interpretation of base64/deflate characters. The initial truncated metadata
  output received no read credit; all decoded-content reads were untruncated.
- No Hyper/donor production edit or new performance claim. Decoder, rendered
  preview and consistency evidence live in ruffini-qualification/abstractions-audit.
  Sandbox mount quota retries were explicitly authorized; no files deleted.
  Inkscape exited0 with four style warnings; preview was visually inspected.
  Full ecosystem goal remains ACTIVE/OPEN; continuing constants07–15 next.

### Ruffini resource continuation checkpoint — 2026-09-06

- Completed constants07 (639lines, four numbered ranges) and constants08
  (680lines, four numbered ranges); no truncation and no inferred read credit.
  All1,319entries pass independent BigInt/lexical canonical-decimal range checks
  and the fingerprinted loader's dimensions. All18predicate controls pass.
  No final newlines; no generating-procedure or cryptographic qualification.
- Inventory now has237source/data READ files,19,087physical lines, one separately
  REVIEWED_ASSET (12physical lines), and seven UNREAD tables/constants09–15
  (7,135lines), totaling245files/26,234physical lines. This turn added two full
  numeric-file reads and completed the generated diagram's decoded-source and
  visual review. No numeric-table algorithm or production transfer.
- Existing Hyperreal changes preserved; Hypersolve and donor clean. Hyperreal
  and Hypersolve gitdiff--check pass. Older234/235read-file checkpoint validators
  remain frozen historical evidence, not relabeled as current aggregate passes.
  Full ecosystem goal ACTIVE/OPEN. Next: constants09–15, relevant remaining
  Ruffini native questions, and the remaining source/reference queue.
- Final verification on2026-09-07: resource-checkpoint-07-08.json passes all new
  resource/diagram evidence,237credited read ranges and the397-file latest Hyper
  snapshot with zero current drift. This is not a rerun of historical numerical
  tests or performance experiments. TURN CHECKPOINT: PROGRESS; all owned
  processes terminal, no agents/commits/pushes/deployments/deletions. No new
  production changes. Full goal remains ACTIVE/OPEN, not complete or blocked.

### Ruffini round tables09–10 — 2026-09-07

- Previous turn classified PROGRESS: two full table reads and the separately
  credited diagram review. Current donor clean; Hyperreal retains only the
  previously recorded audit changes. No inherited running process or agent.
- Fully read constants09/814lines in five numbered ranges and constants10/
  816lines in four ranges. Every read untruncated. These1,630lines contain no
  new scalar algorithm. Inventory advances to239source/data files/20,717lines
  with one separately reviewed asset and five UNREAD tables/constants11–15,
  totaling5,505lines. No production edit or performance claim from numeric data.
- A reusable, explicitly indexed checker requires existing READ credit, checks
  pinned loader/resource fingerprints, and preserves existing result artifacts
  byte-for-byte. Historical validators are not modified for new coverage counts.
  Full ecosystem goal ACTIVE/OPEN; remaining tables/native questions/references
  are still in scope.
- constants09–10 validation passes all1,630entries and18predicate controls.
  Completed constants11 in four untruncated numbered ranges, adding949physical
  lines. Coverage240source/data files/21,666lines plus one reviewed asset;
  constants12–15 remain UNREAD (4,556lines). No new scalar algorithm in this data.
- Combined constants09–11 validation passes all2,579entries and the same18
  predicate controls (not36distinct controls). resource-checkpoint-240-files.json
  verifies240credited ranges, all245inventory fingerprints, four remaining tables,
  and the397-file current Hyper snapshot with zero drift. This does not rerun
  historical native arithmetic/performance qualification.

### Ruffini thirteenth round table — 2026-09-07

- Fully read constants12 in four untruncated numbered ranges:1–273,274–546,
  547–819,820–1092. This adds1,092physical lines of decimal data, no algorithm
  or new transfer candidate. Coverage241source/data files/22,758lines plus one
  separately reviewed12line generated asset. constants13–15 remain UNREAD,
  totaling3,464lines. Format/range qualification is separate from read credit.
- No new production change or performance claim. Full ecosystem goal ACTIVE/OPEN.

### Ruffini fourteenth round table — 2026-09-07

- constants12 passes all 1,092 shape/format/range checks and 18 predicate controls;
  no final newline. Evidence: read-round-constants-12-analysis.json.
- Fully read constants13 in four untruncated numbered ranges: 1–255, 256–510,
  511–765, 766–1020. Its 1,020 decimal entries contain no scalar algorithm.
  Coverage: 242 source/data files, 23,778 lines, one separately reviewed asset;
  two tables remain UNREAD (constants14–15, 2,444 lines). Full goal ACTIVE/OPEN.

### Ruffini final resource reads — 2026-09-07

- constants12–13 shape/format/range checks pass all 2,112 entries and the same
  18 predicate controls; both files lack a final newline.
- Fully read constants14 in four untruncated numbered ranges: 1–288, 289–576,
  577–864, 865–1152; constants15 likewise in ranges 1–323, 324–646, 647–969,
  970–1292. These add 2,444 physical lines of decimal data, no scalar algorithm.
- Ruffini read coverage is now complete at the pinned commit: 244 source/data
  files and 26,222 lines plus the separately REVIEWED_ASSET SVG (12 outer lines,
  decoded source and visual review documented above). Zero UNREAD files. This
  does not expand any native numerical, cryptographic, or performance claim.
- Full ecosystem goal remains ACTIVE/OPEN. No new Hyper/donor production change.

### Ruffini completed-read verification checkpoint — 2026-09-07

- The final four tables pass all 4,556 entry checks. The aggregate invocation
  over constants00–15 passes all 10,854 entries with independent BigInt and
  lexical predicates, pinned loader dimensions, and 18 positive/negative
  controls. All sixteen resources lack a final newline. These are integrity
  checks, not generating-procedure, cryptographic, or performance qualification.
- resource-checkpoint-244-files.json verifies all 244 credited source/data
  ranges, all 245 inventory fingerprints and physical counts, the separate
  reviewed SVG, zero unread files, and the latest 397-file Hyper snapshot with
  zero drift. Historical smaller-count checkpoints remain unchanged and are
  not relabeled as current tests. Final evidence under ruffini-qualification:

  | Artifact | SHA-256 |
  | --- | --- |
  | read-round-constants-12-analysis.json | 94579507a7477e2d1583871e9b02ef6385748872c383999848686e13f35ad38e |
  | read-round-constants-12-13-analysis.json | f3169d72e05316fefdfc997057b78ff706c84a31c0bcc3b9de06df6099482141 |
  | read-round-constants-12-13-14-15-analysis.json | 588adeabcefff606f99ea15aa3195a5b0a40c6c7429499f77c791b03dd18c0be |
  | read-round-constants-00-01-02-03-04-05-06-07-08-09-10-11-12-13-14-15-analysis.json | 6cdb8dcb4669f8d918f9b1009cfb6474cb8c44e0825d8d045796edd572d66ecb |
  | resource-checkpoint-244-files.json | ecbc7c8cabc2cae3923c508e7f0e9c7e72e1b6229a7ae763e07ce4577a3900c5 |

- Ruffini and Hypersolve worktrees are clean; existing Hyperreal audit edits
  are preserved. Hyperreal and Hypersolve git diff --check both pass. No new
  production edit or native arithmetic/performance rerun from these resources.
- TURN CHECKPOINT: PROGRESS. All owned processes terminal; no agents, commits,
  pushes, deployments, or deletions. Sandbox mount quota failures were retried
  through explicit escalation. The full ecosystem goal remains ACTIVE/OPEN,
  not complete or blocked. Next: reconcile remaining Ruffini candidate/native
  qualifications with their recorded dispositions, then continue the pending
  reference and cross-target experiment queue. Read completion does not erase
  the separately open numbers/constructible and cross-reference work.

### Ruffini cache transfer / ledger reconciliation — 2026-09-07

- Corrected the stale numbers and haskell-constructible summary rows against
  their detailed final records. The coefficient-width repair and Constructible
  quotient certificate are already retained and qualified; the prior summary
  descriptions of open repair/native hypotheses were obsolete. Their current
  production hashes match the frozen retained inverse-trig/inverse-hyperbolic
  and structural-analysis hashes. Historical downstream gates remain tied to
  their original snapshots, especially concurrent Hypercurve user edits. This
  correction does not close the entire numbers target or ecosystem queue.
- Re-read the relevant current cache, approximation dispatch, shared constants,
  precision type and scaling architecture. Read the installed num-bigint0.4.6
  signed/unsigned shift implementations completely for the candidate; this is
  a selected dependency slice, not whole-crate read credit. The existing cache
  clones before coarsening while holding the read lock; its comment understates
  that critical-section work. Borrowed ordinary shifts copy only surviving
  limbs, but the bigint's all-discarded branch still clones before zeroing.
- Confirmed a new public cache-hit completeness defect without modifying
  production: sqrt(17).approx(-128), then approx(i32::MAX), panics in debug at
  hyperreal/src/computable/node/representation.rs:197 on signed subtraction.
  The release overflowing call was deliberately not run, because wrapping can
  cause a huge left shift. No blanket full-i32 API or fresh-computation claim.
- Private cache-rescale-pilot compares the original formula with widened-gap,
  borrowed, and borrowed-plus-strict-zero-guard variants. Rounding remains
  nearest with ties toward positive infinity; miss/equal behavior is unchanged.
  The exact gap fits u32 after an i64 subtraction. Independent Euclidean
  quotient/remainder and enclosure checks pass149504original/149784per-candidate
  cases in debug, release and instrumented release.280unsafe baseline-overflow
  cases are explicitly skipped; all three candidates pass those cases. Small
  signed exhaustive grids and2310wide/boundary cases are not conflated with
  independent public-call qualification of a changed library.
- Frozen v2 evidence:1210CPU6-pinned timing observations,1100postwarmup,
  170170000checksum-checked calls;10measured samples per22signed workloads and
  five rotated/reversed variants including an identical-code control.264separate
  allocation observations reproduce exactly across three repeats, with zero
  retained-live deltas. Independent JavaScript reconstruction validates every
  recorded checksum and all1474raw measurement records reconcile.
- The borrowed/guarded million-bit-to128bit helper ratios are about.0260/.0212
  positive and.0268/.0270negative. Positive requested bytes per call fall
  131096→24, allocation calls2→1, additional peak requested live131072→24.
  Half-width results improve less and can be inconclusive for negative inputs.
  Crucially, guarded64bit gap-one timing regresses:1.1698[1.1051,1.2388]positive
  and1.2112[1.1442,1.2582]negative. This is not a universal speedup or a public
  cache/application timing claim. All uncertainty/control results are retained.
- The initial unqualified build needed dependency pinning and a local type
  annotation. v1 correctness passed, but its timing run stopped at an overly
  small oracle allocation threshold. Exact v1 sources/binaries/partial records
  are preserved, not mixed into corrected v2 measurements. The v2 analyzer's
  first run rejected a zero CPU delta in an allocation-only observation; the
  corrected gate excludes all instrumented timings, without changing raw data.
- Decision: NO NEW PRODUCTION CHANGE YET. The cache precision overflow needs a
  qualified repair, but a large helper win does not justify the measured small
  regressions. Next preserve the original gap-one direct-clone path in a private
  candidate, qualify actual synchronized cache/public history behavior, and run
  full correctness, caller performance, memory and binary/code-size gates.
  New details/proofs are in ruffini-qualification/cache-rescale-pilot/README.md.

  | v2 artifact under cache-rescale-pilot/evidence-v2 | SHA256 |
  | --- | --- |
  | prepared.json |34c7aadf91034ecbc72dc6d9fa3e7a261555a7b232475e567c66b7a7d30041b8|
  | measurements.json |89c6c3cfc16b13c17ac775311dc39d3ea626eb9105b70a902c21016e9dbdc1ff|
  | analysis.json |e43bfdb3b2ffdee92a4dcc462f4e3aa60f8e25f3e78328e6b6576764ada30c53|

- This turn is PROGRESS: a reproducible public defect and independently checked
  tradeoffs replace an untested transfer hypothesis. Ruffini read coverage stays
  at244source/data files26222lines plus one separately reviewed SVG; no new donor
  lines are credited. All397current Hyper snapshot files match after the pilot.
  Earlier retained repairs remain untouched. Full ecosystem goal ACTIVE/OPEN.

### Ruffini public cache repair retention — 2026-09-07

- Previous turn: PROGRESS. This turn preserves the original gap-one direct-clone
  path while using p.abs_diff(q), borrowed ordinary coarsening and a strict
  magnitude zero guard. Nearest/ties-up rounding is unchanged. No new API,
  scalar/cache field, dependency, synchronization or exact-fact rule. The cache
  read-lock comment is corrected. Detailed proof and every tradeoff are in
  ruffini-qualification/cache-public-pilot/README.md.
- The175-file private mirror and125registry packages match production. Two
  ordinary real-cache tests pass before/after; three extreme-gap/public/threaded
  tests fail before on signed subtraction and pass after in debug/release.
  Independent integer checks validate160ordinary and180extreme-inclusive public
  histories. All five tests and180public requests pass Memcheck, zero errors,
  leak checking off. Initial lock/include-path and Valgrind/tmp setup failures
  are preserved separately, not counted as numerical outcomes.
- Paired public evidence:2028observations/1872postwarmup/79872000checked calls,
  52signed warm/cold workloads;312separate allocation observations repeat exactly.
  65536→128bit queries improve about2.8–3.0x; requested bytes per call8224→32
  for sqrt(17),8208→24for1/3, calls2→1 and peak requested live8200→32/8192→24.
  A longer repeat adds800observations/768postwarmup/440000000calls with both-build
  identical-code controls. It confirms small rational-case costs: positive and
  negative65536bit gap-one ratios1.0236[1.0166,1.0305] and1.0294[1.0114,1.0440];
  some equal/cold rational pooled controls cost about1–2%. These are explicitly
  accepted for higher-priority completeness and substantial large-gap savings,
  not called a universal speedup/no-regression result.
- Same private driver: file+1200B,text+924,data unchanged,bss−928,loadedtotal−4.
  Private756debug/863release-all-feature tests, Clippyallfeatures/alltargets,
  fmt and WASMlib gates pass. Applied only the qualified runtime body/comment,
  one facade include and five formatted tests to Hyperreal, preserving prior
  edits. No commit/push/deployment/deletion. A947-file/45192934-byte six-crate
  before-source snapshot preserves exact pre-change sources and inventories.
  Hypercurve was clean at a3c1bb4582efbe337f37ef3e2c878787edf861d4.
- Retained-source756/863tests, Clippy,WASM,cargo fmt,new-test fmt and diff--check
  pass. Full downstream default tests pass:Hyperlattice202,Hyperlimit348,
  Hypertri7,Hypersolve796. Current Hypercurve's full library suite is RUNNING
  in exec session52381; evidence cache-public-pilot/retained-gates-DKveZ9.
  Poll the actual handle before calling it terminal. The verifier checks all
  frozen and current files/inventories before and after commands, allowing only
  the two intended edits and new test. Old397-file verifiers correctly reject
  this intentional live drift; their frozen results remain historical evidence.

  | Artifact under cache-public-pilot | SHA256 |
  | --- | --- |
  | paired-v1/analysis.json |fb79e0de09a4637789c09a32a58f67ce7a10b133434e36f0484e14a24ae35b37|
  | focused-v1/analysis.json |81a4b8df836517dcf2ba3faf6c0d0ac2e77f204b7bcf42975268d8ce0f06fbe4|
  | gates-ddSPOg/complete.json |59dc6dedfc9d82984a4621d3601981f5ca43f5a48ff3ee041b7926ce94d5067f|
  | memcheck-v2/complete.json |1ddf7d52f0237584942604b087c639ec60b5d82ae014cb47e202282f97c33425|
  | retention-baseline.json |bcfd719d324cf2570e63a0cdaaf12d9c9e224417ee5ef700e21165d576e60ca4|

- Runtime representation SHA6901ebf2795947942eea4356b0031fa2a5794187f79e30c076e5c6a01351d940;
  node facade c6cb5f5a1b1c8c471fedba18a7bb3e85b139c372fc65da3b033b39c4ecd49c6e;
  formatted tests699c725764121e771a693c5cd9090f654442e40f15156c70e53ccc3c70572a0a.
  This turn is PROGRESS; final retention closure awaits Hypercurve and final
  source/evidence reconciliation. Ruffini read counts are unchanged. Whole-i32
  arithmetic outside valid cache hits remains separate. Full goal ACTIVE/OPEN.

### Ruffini retained closure / next reference checkpoint — 2026-09-07

- Hypercurve's current full library gate completed:910passed,0failed,1ignored,
  755.61s. All11retained commands exit0. Source inventories and hashes stayed
  fixed through completion. Session52381 is terminal; no test remains running.
  Final verify-retained.mjs reconciles947baseline files plus the new test,
  all3140raw timing/allocation records, frozen binaries, Memcheck and exact
  test totals; diff--check passes in all six crates. The cache transfer is
  qualified and retained, uncommitted. Its1–3%small-case costs remain disclosed.
- retained-gates-DKveZ9/complete.json SHA256
  75900b1a1dc76ba4c79f323179326fa2c1a86b4abf8a0f518abd9ca221a3696a;
  cache-public-pilot/retained-checkpoint.json SHA256
  2dd4692766559e9891f8158cbc81fee28c0dae803a56d671e6d4cf03ac576e2e.
- While that verified live gate ran, started the pending Haskell comparison
  reference at7c53d4e4259f633606b18d655cdbbf813866ab81. All567files/7746380bytes
  are inventoried:14text/code files1534lines,525logs14025lines,14SVG/14PNGassets.
  All14 text/code files1534lines were completely read, numbered and untruncated,
  with per-file notes and hashed coverage. The initial truncated filename listing
  gave no read credit. Source questions include decimal/binary accuracy mapping,
  adapter demand consistency, omitted achieved-accuracy fields, stale log
  timestamps, and unqualified repetition/reporting. These are not executed
  native failures or validated performance comparisons. No donor code changed.
- Completed all.js (526 lines) and results.html (232 lines); report structure and historical data are recorded in HASKELL_COMPARISON_FILE_NOTES.md. Parsed all525 archived logs without modification: one-to-one with exported rows, all exits zero, RSS exact, 385 timing normalizations, and achieved-accuracy absent from all CDAR logs. This is provenance reconciliation only; no native benchmark or correctness claim. Parsed all14 SVG assets (static charts; no script/image elements) and inventoried all14 PNG assets (800x600 RGB); no transferable scalar idea.
- Next: qualify Haskell task/precision contracts against actual pinned dependencies;
  all archived logs and chart assets have already been reconciled at provenance level.
  Other references and cross-target work remain open; this turn is PROGRESS,
  not full-goal completion.
- Added Common Lisp computable-reals at607a5d5b95387c06f92a661aa5562a7be0f5cad8: all8 files/877 lines read and disposition recorded in COMPUTABLE_REALS_FILE_NOTES.md. No native runtime was available; no production transfer selected.
- Added OCaml Creal at0520c6351fdc431db3c0941b5cf111211826f88a: all13 files/1850 lines read and disposition recorded in CREAL_FILE_NOTES.md. No OCaml runtime was available; no production transfer selected.
- Added Julia ExactReals.jl at19d32155e69ad988cd68282d4e88b859552033e3: all11 files/382 lines read and disposition recorded in EXACTREALS_JL_FILE_NOTES.md. Direct Julia tests pass8/8; an independent zero-multiplication probe times out after3 seconds, and negative sqrt raises DivideError. No production transfer selected. No agents, commits, pushes, deployments or deletions.

- Added Julia DedekindCutArithmetic.jl at dc7a51b784a6d8f04be22fcca862656432a3d87e: inventoried 55 files and 2010 lines, read source/tests/docs in bounded chunks, and recorded hashes in DEDEKINDCUT_ARITHMETIC_FILE_INVENTORY.tsv. Static audit found undefined divisor error variable, exponent-zero round hazard, mutable unsynchronized cache, and recursive comparison/quantifier termination gaps. Native tests were blocked by missing Julia dependencies and quota-limited registry. No production transfer selected; no agents, commits, pushes, deployments or deletions.

- Started CRCalc.js audit at ea5d92921599f3666c6afbf86235864e66469da9: 63 tracked files; core TypeScript source is 3472 lines and tests 173 lines. Bounded audit reached CR caching, precision overflow checks, approximate comparisons, lazy arithmetic and elementary functions. Native npm test is blocked because dev dependency terser is absent; full source audit remains open.

- Completed CRCalc.js at ea5d92921599f3666c6afbf86235864e66469da9: all 63 files/19,537 lines inventoried, scalar/web TypeScript and metadata read in bounded chunks, and hashes recorded in CRCALC_JS_FILE_INVENTORY.tsv. Checked-in distribution test passes (`Success`); native npm rebuild is blocked by missing terser. Found coarse select-sign caching, unbounded worker maps, intentional equality divergence, and an always-true UnifiedReal.asin OR guard. No production transfer selected; no agents, commits, pushes, deployments or deletions.

- Completed Tcllib `math::exact` at 6093f8d6246572ae461bf344333483ca329413bc: all six exact-real artifacts/7,595 lines were read line by line and hashed in TCLLIB_MATH_EXACT_FILE_INVENTORY.tsv. The native Tcl suite passes 198/198 with no skips or failures, and a reference-counted sqrt/rational runtime probe passed. Mobius/tensor refinement and deferred destruction were compared with Hyper; no worthwhile production transfer selected. No agents, commits, pushes, deployments or deletions.

- Completed Spire exact-number slice at 0fe5a6a9714181a20fc9cef4c8b2af088ff2b4c9: five files/2,959 lines were read line by line and hashed in SPIRE_REAL_FILE_INVENTORY.tsv. ScalaCheck sources were fully audited but native tests could not run because sbt is unavailable. Separation-bound algebraic sign logic was compared with Hyper; no worthwhile production transfer selected. No agents, commits, pushes, deployments or deletions.

- Completed Numbas constructive-real extension at b840893fe89308ee2a3e5167a901295dc5c2851b: both files/2,253 lines were read line by line and hashed in NUMBAS_CONSTRUCTIVE_REAL_FILE_INVENTORY.tsv. A Node probe verified 20-digit sqrt2, pi, and e; Numbas browser integration was unavailable. The embedded CR design duplicates already audited AOSP/CRCalc patterns; no production transfer selected. No agents, commits, pushes, deployments or deletions.

- Completed Edalat prototype at 474b133ad9cd52d94000efcfcb6febb3722edf2f: all three files/2,095 lines were read line by line and hashed in EDALAT_FILE_INVENTORY.tsv. Direct Python execution is blocked by a raw 0xB2 source byte; in-memory normalized execution passes 32/32 tests. No production transfer selected. No agents, commits, pushes, deployments or deletions.

- Started Marshall audit at c9f1f6466e879e8db11a12b9bc030e62b07d8bd2. Repository contains OCaml exact-real language plus Haskell implementation and generated figures; core interval/approximant/dyadic/Newton files are being read in bounded chunks. Full-file coverage and tests remain open.

- Marshall audit completed at pin c9f1f6466e879e8db11a12b9bc030e62b07d8bd2. All tracked text/source files were read line-by-line in bounded reads; generated EPS/FIG/PDF figures were classified. OCaml, dune, GHC, and runghc were unavailable, so native tests/benchmarks could not run. Findings: explicit staged precision/rounding, Kaucher intervals, lower/upper approximants, searchable compact/overt semantics, and interval-Newton/Lipschitz ideas; no production Hyper change met the exactness-first bar.

- Rocq exact-real-arithmetic audit completed at d6bc1fee859c5b773dc46ef8efd4d43ba29697c5. All 35 tracked files and 5,471 lines were read. The source explicitly warns it is unsafe; native Coq tools are unavailable, and no production Hyper transfer was justified.

- Started RZ audit at d92cacaf78fb50d61fc6712c74b8fdaf5d2c6d28. The checkout is a 283-file, 148,131-line compiler/theory plus private historical corpus; full bounded line coverage remains open.
- Completed the RZ executable-source audit at d92cacaf78fb50d61fc6712c74b8fdaf5d2c6d28: all compiler/printer sources, examples/tests, and all 55 unique code-like private artifacts were read in bounded non-truncated chunks. Private non-code historical artifacts remain classified; no worthwhile Hyper change selected.
- Qualified the RealLib cache-release candidate against Hyperreal: existing cache microbenchmarks and the release counting-allocator profile show a material repeat-query hit benefit and no broad retained-memory problem after node destruction. No production change retained.
- Ran the existing deep evaluator qualification: four 5,000-node Add/Multiply/structural tests pass in debug; generic elementary/inverse/root depth remains unclosed, so no stack-safety change is retained.
- Completed `corn/metric2/Complete.v` (1,233 lines): regular-function completion, precision-tightening wrappers, map/join/bind monad laws, and locatedness transport compared with Hyperreal/Hyperlimit; no production change selected.
- Completed `corn/metric2/Limit.v` (423 lines): continuation-passing lazy termination and coinductive limit predicates compared with Hyperlimit; no production change selected.
- Completed `corn/algebra/COrdFields.v` (1,656 lines): constructive ordered-field axioms, apartness/order conversion, cancellation, positivity, division, and finite-sum sign propagation compared with Hyperreal structural sign semantics; no production change selected.
- Completed `corn/ftc/Integral.v` (1,498 lines): Darboux-sum integration, mesh/modulus convergence, partition joins, additivity, monotonicity, and norm bounds compared with Hyperlimit; no immediate production change selected.
- Completed `corn/transc/InvTrigonom.v` (1,274 lines): integral-defined inverse trig functions, IVT range construction, domains, and inverse identities compared with Hyperreal inverse-trig kernels; no production change selected.
- Completed `corn/transc/Exponential.v` (1,351 lines): Taylor-series exp/log laws, positivity, monotonicity, derivatives, and uniqueness compared with Hyperreal kernels; no production change selected.
- `cargo test --manifest-path hyperlimit/Cargo.toml --lib` passed all 242 tests (0 failed, 1.06s), including exact geometry predicates, structural-fact fallbacks, and Hyperreal/Hyperlattice integration.
- Related-crate validation: `cargo test --manifest-path hypersolve/Cargo.toml --lib` passed all 432 tests (0 failed, 3.68s), covering Hyperreal/Hyperlattice/Hyperlimit integration and exact polynomial/resultant/root-isolation paths.
- Completed `corn/algebra/CPolynomials.v` (2,786 lines): recursive Horner polynomials, derivatives, ring maps, and zero-trimming multiplication were compared with Hyperlimit/Hypersolve; no production change selected.
- Completed `corn/metric2/Compact.v` (1,824 lines): geometric compact-approximation streams and compact-image constructions were compared with Hyperlimit; no production change selected.
- Completed six additional compact CoRN support files (`CRing_as_Ring.v`, `N.v`, `Container.v`, `Ranges.v`, `Qclasses.v`, `Qposclasses.v`) totaling 85 lines; all are interface/typeclass glue or tiny helpers and yielded no Hyper change.
- Validation after the CoRN fast slice: `cargo test --manifest-path hyperreal/Cargo.toml --lib` passed all 679 tests (0 failed, 6.84s). Existing cache microbench remained stable for ratio cold/cached (28.04/19.07 ns) and pi cached (19.54 ns); pi cold measured 31.24 ns, a small +1.83% Criterion comparison marked as regression but not tied to any retained audit change.
- Started CoRN audit at `ada7c0b497ff15dd67cf7932c6f20e143a2aee2f`; created a 375-file/189,930-line source inventory and completed bounded reads of all 21 `reals/faster` files plus `reals/fast/CRexp.v`, `CRroot.v`, `CRsin.v`, `CRarctan.v`, `CRabs.v`, `CRarctan_small.v`, `CRcos.v`, `CRpi_fast.v`, `CRln.v`, `CRartanh_slow.v`, `CRpower.v`, `CRconst.v`, `CRpi_slow.v`, `CRball.v`, `CRsign.v`, `CRpi.v`, `CRGeometricSum.v`, `CRAlternatingSum.v`, `CRIR.v`, `CRGroupOps.v`, `CRFieldOps.v`, `CRArith.v`, `CRArith_alg.v`, `CRtrans.v`, `Compress.v`, `PowerBound.v`, `CRsum.v`, and `uneven_CRplus.v`, and `LazyNat.v`, and `ContinuousCorrect.v`, and `ModulusDerivative.v`, and `CRAlternatingSum_alg.v`, and `CRstreams.v`, and `CRcorrect.v`, and `Integration.v`, and `MultivariatePolynomials.v`, and `RasterQ.v`, and `Interval.v`, and `Plot.v`, and `RasterizeQ.v` plus five compact support/example files (27,623 lines total).
- Ran the full Hyper library regression suite: 679 tests passed, 0 failed in 6.84s.

- Re-ran the current Hyperreal library gate after the audit bookkeeping: 679 tests passed, 0 failed, 0 ignored in 6.72s.

- Completed CoRN `ftc/RefLemma.v` (1,163 lines), `ftc/RefSeparated.v` (840 lines), `ftc/RefSeparating.v` (1,235 lines), and `logic/CLogic.v` (1,662 lines), totaling 4,900 lines. These are exact partition-refinement/separation proofs and computational-proposition/induction foundations; all are proof-oriented/materialized and yielded no worthwhile Hyper change.
- Completed CoRN `logic/Classic.v` (173 lines): double-negation classical connectives and pigeonhole proofs only; no executable scalar or Hyper change.
- Completed CoRN `logic/CornBasics.v` (969 lines): arithmetic/coercion and well-founded proof infrastructure only; no worthwhile Hyper change.
- Completed CoRN `logic/Stability.v` (158 lines): double-negation/stability and decidability wrappers; no worthwhile Hyper change.
- Completed CoRN `metric2/CompleteProduct.v` (172 lines) and `metric2/DistanceMetricSpace.v` (167 lines), totaling 339 lines; completion/product and metric-interface glue yielded no worthwhile Hyper change.
- Completed CoRN `metric2/FinEnum.v` (1,000 lines): finite-enumeration Hausdorff metric, locatedness/prelength, and map/completion proofs; no worthwhile Hyper change.
- Completed CoRN `metric2/LocatedSubset.v` (248 lines), `Metric.v` (248 lines), and `ProductMetric.v` (385 lines), totaling 881 lines; locatedness, ball metrics, and product/completion combinators yielded no worthwhile Hyper change.
- Completed CoRN `metric2/Prelength.v` (664 lines): modulus-aware completion maps/binds, trails, and prelength laws; no worthwhile Hyper change.
- Completed CoRN `metric2/list_separates.v` (54 lines) and `Graph.v` (665 lines), totaling 719 lines; finite-enumeration and compact graph constructions yielded no worthwhile Hyper change.
- Completed CoRN `metric2/Classified.v` (1,187 lines): generalized Qinf balls, typeclass metric/function classification, and composition/currying infrastructure; canonical single-ball design confirmed, no worthwhile Hyper change.
- Completed CoRN `metric2/StepFunction.v` (890 lines): rational-cut step trees, split/mirror laws, and applicative composition; no worthwhile Hyper change.
- Completed CoRN `metric2/StepFunctionSetoid.v` (908 lines): setoid step-function equality, split/glue morphisms, applicative laws, and combinator evaluation; no worthwhile Hyper change.
- Completed CoRN `model/Zmod/Cmod.v` (135 lines) and `metrics/Equiv.v` (546 lines), totaling 681 lines; modulo/conversion and pseudo-metric equivalence infrastructure yielded no worthwhile Hyper change.
- Completed CoRN `metrics/CPseudoMSpaces.v` (340 lines) and `LipExt.v` (446 lines), totaling 786 lines; traditional CR metrics and McShane/Kirszbraun extension proofs yielded no worthwhile Hyper change.
- Completed CoRN `metrics/Prod_Sub.v` (412 lines): additive product and subspace pseudo-metrics; no worthwhile Hyper change.
- Completed CoRN `metrics/CPMSTheory.v` (784 lines): total boundedness, locatedness, infimum/supremum, compactness, and approximate selectors; no worthwhile Hyper change.
- Completed CoRN `metrics/ContFunctions.v` (712 lines) and `IR_CPMSpace.v` (498 lines), totaling 1,210 lines; continuity/Lipschitz predicates and CR real metric foundations yielded no worthwhile Hyper change.
- Completed CoRN `model/Zmod/ZBasics.v` (931 lines): nat/Z arithmetic, sign/abs, monotonicity, and triangle proof infrastructure; no worthwhile Hyper change.
- Completed CoRN `model/Zmod/ZDivides.v` (1,017 lines): integer divisibility, quotient/remainder, cancellation, and modulo-zero theory; no worthwhile Hyper change.
- Completed CoRN `model/Zmod/ZMod.v` (755 lines): exact modular arithmetic, congruence, inverses, and gcd identities; no worthwhile Hyper change.
- Completed CoRN `model/Zmod/ZGcd.v` (1785 lines): well-founded Euclidean gcd/Bezout and relative-prime/prime theory; no worthwhile Hyper change.
- Completed CoRN `model/Zmod/Zm.v` (729 lines): modular setoid/ring and prime-field construction; no worthwhile Hyper change.
- Completed CoRN `model/abgroups/{CRabgroup,QSposabgroup,Qabgroup,Qposabgroup}.v` (212 lines total): abelian-group typeclass wrappers for CR/Q/Qpos; no worthwhile Hyper change.
- Completed CoRN group/field wrappers (`Zabgroup`, `CRfield`, `Qfield`, `CRgroup`, `QSposgroup`, `Qgroup`, `Qposgroup`, `Zgroup`; 479 lines): typeclass lifts only; no worthwhile Hyper change.
- Completed CoRN lattice/metric/monoid models (`CRlattice`, `CRmetric`, `CRmonoid`, `Nm_to_cycm`, `Nm_to_freem`; 547 lines): structural completions and monoid morphisms; no worthwhile Hyper change.
- Completed CoRN `model/metric2/LinfMetric.v` (296 lines): fold-based step-function sup and Linf metric/embedding; no worthwhile Hyper change.
- Completed CoRN `model/metric2/LinfMetricMonad.v` (469 lines): pointwise sup-ball metric lifting and step-function bind/map continuity; no worthwhile Hyper change.
- Completed CoRN `model/metric2/L1metric.v` (560 lines): exact step-function integration/L1 metric and continuity; no worthwhile Hyper change.
- Completed CoRN basic monoids (`Nmonoid`, `Nposmonoid`, `QSposmonoid`, `Qmonoid`; 309 lines): unit/typeclass wrappers only; no worthwhile Hyper change.
- Completed CoRN monoid/order-field wrappers (`Qposmonoid`, `Zmonoid`, `freem_to_Nm`, `CRordfield`, `Qordfield`; 443 lines): structural lifts only; no worthwhile Hyper change.
- Completed CoRN ring/semigroup wrappers (`CRring`, `Qring`, `Zring`, `CRsemigroup`, `Npossemigroup`; 445 lines): algebraic structure only; no worthwhile Hyper change.
- Completed CoRN primitive semigroup wrappers (`Nsemigroup`, `Qsemigroup`, `Zsemigroup`; 185 lines): native operation packaging only; no worthwhile Hyper change.
- Completed CoRN setoid models (`CRsetoid`, `Nfinsetoid`, `Npossetoid`; 323 lines): constructive-real and bounded/positive-natural setoid wrappers; no worthwhile Hyper change.
- Completed CoRN setoids `Nsetoid.v` and `Qpossetoid.v` (441 lines): natural/positive-rational constructive setoids and operation wrappers; no worthwhile Hyper change.
- Completed CoRN setoids `Qsetoid.v` and `Zfinsetoid.v` (278 lines): rational arithmetic and bounded integer setoids; no worthwhile Hyper change.
- Completed CoRN `Zsetoid.v` and `decsetoid.v` (308 lines): integer arithmetic and generic decidable-setoid infrastructure; no worthwhile Hyper change.
- Completed CoRN `model/structures/Nsec.v` and `Npossec.v` (367 lines): natural apartness/decidability and positivity lemmas; no worthwhile Hyper change.
- Completed CoRN `QposInf.v`, `Qinf.v`, `QnnInf.v` (415 lines): tagged infinity/error-bound arithmetic and min/order; no worthwhile Hyper change.
- Completed CoRN `model/structures/Qpossec.v` (480 lines): proof-carrying positive rationals and exact arithmetic helpers; no worthwhile Hyper change.
- Completed CoRN `model/structures/StepQsec.v` (372 lines): exact step-function rational operations/ring and fold equality; no worthwhile Hyper change.
- Completed CoRN `model/structures/Zsec.v` (379 lines): signed-integer apartness and sign/divisibility lemmas; no worthwhile Hyper change.
- Completed CoRN `model/totalorder/QMinMax.v` (251 lines): rational total-order min/max and sign-aware distribution; no worthwhile Hyper change.
- Completed CoRN `model/totalorder/QposMinMax.v` (444 lines): positive-rational total order/min-max and sign-aware distribution; no worthwhile Hyper change.
- Completed CoRN order/algebra infrastructure (`ZMinMax`, `Transparent_algebra`, `Opaque_algebra`, `Lattice`, `PartialOrder`; 678 lines): generic order laws and reduction toggles; no worthwhile Hyper change.
- Completed CoRN `order/TotalOrder.v` (329 lines): generic total-order min/max, duality, and monotone/antitone laws; no worthwhile Hyper change.
- Completed CoRN real/metric/order infrastructure (`CMetricFields`, `CPoly_Contin`, `CReals`, `CSumsReals`; 599 lines): completeness/continuity contracts and sum identities; no worthwhile Hyper change.
- Completed CoRN `reals/Cauchy_CReals.v` (850 lines): Cauchy-sequence real completion, moduli, density, and diagonal limits; no worthwhile Hyper change.
- Completed CoRN `reals/Intervals.v` (1015 lines): compact intervals, total-bounded ε-nets, lub/glb limits, and interval partitioning; no worthwhile Hyper change.
- Completed CoRN `reals/NRootIR.v` (880 lines): IVT nth-root construction, absolute/triangle bounds, reciprocal Cauchy convergence, and partial-function lifting; no worthwhile Hyper change.
- Completed CoRN `reals/Q_dense.v` (918 lines): constructive rational-density intervals, 2/3 contraction, and convergent centers; Hyper adaptive refinement already subsumes the useful scheduling pattern, no change retained.
- Completed CoRN reals/R_morphism.v (673 lines): homomorphism laws, composition, limits, and surjectivity; no worthwhile Hyper change.
- Completed CoRN reals/RealCount.v (368 lines): nested-interval uncountability construction; no change.
- Completed CoRN reals/RealFuncts.v (260 lines): limit/continuity and monotonicity specifications; no change.
- Completed CoRN reals/RealLists.v (438 lines): list extrema and partial maps; no worthwhile change.
- Completed CoRN reals/stdlib/CMTDirac.v (85 lines): Dirac integration construction; no change.
- Completed CoRN reals/stdlib/CMTFullSets.v (1982 lines): full-set integration, diagonal completion, convergence theorems; no worthwhile Hyper change.
- Completed CoRN reals/stdlib/ConstructiveFastReals.v (374 lines): regular-function constructive-real adapter; no change.
- Completed CoRN reals/stdlib/ConstructiveFasterReals.v (419 lines): faster-real cast adapter and completion; no worthwhile change.
- Completed CoRN reals/stdlib/CMTbase.v (586 lines): constructive integration-space interfaces and bounds; no worthwhile Hyper change.
- Completed CoRN reals/stdlib/CMTPositivity.v (456 lines): constructive positivity/continuity and geometric series scheduling; no worthwhile Hyper change.
- Completed CoRN reals/stdlib/ConstructiveDiagonal.v (1016 lines): diagonal series rearrangement and tail bounds; no worthwhile Hyper change.
- Completed CoRN reals/stdlib/ConstructivePartialFunctions.v (861 lines): proof-carrying partial functions and arithmetic combinators; no worthwhile Hyper change.
- Completed CoRN reals/stdlib/Markov.v (131 lines): Markov principle/order apartness; noncomputable, no change.
- Completed CoRN stdlib_omissions/List.v (155 lines): generic list proof utilities; no change.
- Completed CoRN stdlib_omissions/P.v (53 lines): positive/nat conversions; no change.
- Completed CoRN stdlib_omissions/Q.v (547 lines): rational arithmetic/order/floor/ceiling utilities; no worthwhile Hyper change.
- Completed CoRN stdlib_omissions/Z.v (110 lines): integer conversion/order utilities; no change.
- Completed CoRN tactics/CornTac.v (59 lines): setoid replacement tacticals; no change.
- Completed CoRN tactics/DiffTactics1.v (49 lines): continuity/derivative Ltac wrappers; no change.
- Completed CoRN tactics/DiffTactics2.v (357 lines): symbolic derivative AST/tactics; no worthwhile Hyper change.
- Completed CoRN tactics/DiffTactics3.v (176 lines): interval derivative AST/tactics; no change.
- Completed CoRN tactics/AlgReflection.v (524 lines): reflected algebraic normalization; no worthwhile Hyper change.
- Completed CoRN reals/stdlib/CMTIntegrableFunctions.v (2373 lines): integrable representations, weaving, contractive operators, integral algebra and completion; no worthwhile Hyper change.
- Completed CoRN reals/stdlib/CMTIntegrableSets.v (1170 lines): characteristic integrable sets, measure algebra, countable unions/intersections, and restrictions; no worthwhile Hyper change.

- Completed CoRN reals/stdlib/CMTMeasurableFunctions.v (2034 lines): measurable closures, support/generator approximations, convergence in measure, dominated convergence; no worthwhile Hyper change.

- Completed CoRN reals/stdlib/CMTProductIntegral.v (2323 lines): product-space disjointization/Fubini and Riesz machinery; exponential list expansion is allocation-heavy; no worthwhile Hyper change.

- Completed CoRN reals/stdlib/CMTprofile.v (2239 lines): constructive profile/jump-point and inverse-image integrability machinery; no worthwhile Hyper change.

- Completed CoRN tactics/FieldReflection.v (958 lines): reflected field AST normalization and quotation tactics; proof-time only, no worthwhile Hyper change.

- Completed CoRN tactics/Qauto.v (51), Rational.v (77), Step.v (47), util/PointFree.v (74): proof automation/typeclass utilities; no worthwhile Hyper change.

- Completed CoRN RingReflection, Extract, Qdlog, Qgcd, Qsums, SetoidPermutation, WritePPM (111–890 lines): proof/extraction/rational utility and raster code; no worthwhile Hyper change.

- Completed CoRN tactics/csetoid_rewrite.v (1505 lines): reflective total/partial setoid rewriting and domain folding; proof/tactic-only, no worthwhile Hyper change.

- Completed iRRAM src/limits.cc (944 lines): adaptive limit/lipschitz retries, local error propagation, and iteration fallback; no worthwhile Hyper change.

- Completed iRRAM src/sqrt.cc (137) and exp_log.cc (178): MPFR/ Newton sqrt and range-reduced exp/log/AGM; no worthwhile Hyper change.

- Completed iRRAM src/sin_cos.cc (412 lines): grouped Taylor kernels, exact triple-angle/modulo-2pi reduction, and inverse/hyperbolic composition; no worthwhile Hyper change.
- Completed iRRAM src/STREAMS.cc (303 lines): iteration-safe stream I/O replay/cache semantics; no direct Hyper analogue or change.

- Completed iRRAM src/DYADIC.cc (262 lines): MPFR dyadic arithmetic and shift scaling; no worthwhile Hyper change.
- Completed iRRAM src/INTERVAL.cc (320 lines): endpoint interval operations and trig extrema/range reduction; source has hull/intersect endpoint defects, no transplant.
- Completed iRRAM src/LAZY_BOOLEAN.cc (113 lines): fail-closed three-valued decisions with replay caching; no worthwhile Hyper change.
- Completed iRRAM src/REALLIB.cc (319) and REALmain.cc (197): scalar helpers, matrix routines, and geometric precision initialization; no worthwhile Hyper change.

- Completed iRRAM RATIONAL.cc, GMP_int_ext.c, GMP_rat_ext.c (209–465 lines): canonical GMP rational/integer kernels, roots, powers, shifts; no worthwhile Hyper change.
- Completed iRRAM MPFR_ext.c, mpfr_extension.cc, convert.cc, errno.cc (8–79 lines): MPFR bridge, certified conversion, thread-local pools/error names; no worthwhile Hyper change.
- Completed iRRAM pi_ln2.cc and stack.cc (158, 172 lines): cached constants, precision stack, Lipschitz module search; no worthwhile Hyper change.

- Completed iRRAM src/COMPLEX.cc (620 lines): complex arithmetic, branch-selected sqrt, and composed elementary functions; branch-fragile formulas make it unsuitable for transplant.
- Completed iRRAM REALMATRIX.cc and SPARSEREALMATRIX.cc (295, 591 lines): dense/sparse Gaussian elimination, linked-list storage, hotspot lookup; no worthwhile scalar change.
- Completed iRRAM MPFR/MPFR_ext.h (459 lines): operand-size-aware MPFR precision, trailing-zero trimming, native shift/sqrt; Hyper already covers these policies.

- iRRAM audit complete: all 24 inventoried source/header files are READ with per-file notes; no source change met the exactness/completeness/performance bar.
- Verification: `cargo test --manifest-path hyperreal/Cargo.toml --lib` passed 679 tests, 0 failures in 6.82s.
- Haskell comparison harness source audit is complete; 525 archived logs and 28 static chart assets remain classified as data/provenance artifacts (not scalar source), with one-to-one parser reconciliation already documented in HASKELL_COMPARISON_FILE_NOTES.md.

- Completed CRCalc.js scalar source audit: `crcalc-js/src/cr.ts` (3,472 lines) read in four bounded non-truncated slices and the checked-in distribution test rerun successfully (`node .../dist/cr-test.mjs`: Success). The bounded-rational/UnifiedReal symbolic side channel, slow precision batching, and explicit comparison contracts are already represented by Hyper’s structural metadata or stronger indeterminate semantics; no production change selected.

- Reconciled Marshall audit status: MARSHALL_FILE_NOTES.md records complete bounded reads of all tracked text/source files (OCaml executable, Haskell prototypes, notes/slides, metadata) and classifies generated figures; native validation is unavailable because ocaml/dune/ghc are absent. No worthwhile Hyper change selected.
- Reconciled Ruffini status: the latest notes record complete coverage of all 244 source/data files plus the reviewed SVG and validated all constants00–15; older UNREAD checkpoint text is historical and superseded.

- Reconciled Plume status: PLUME_FILE_NOTES.md records complete reads of all 174 original source-container artifacts (24,049 lines), full report/primary visuals, and targeted numerical qualifications; indexed LaTeX2HTML section links remain provenance-only generated containers, not unreviewed scalar source. No additional transfer justified.

- Updated NUMBERS_FILE_NOTES.md to reconcile the completed high-precision coefficient-width repair: all four rational/generic asin/asinh recurrences are now qualified through 524288-bit probes, MPFR checks, full tests, allocation benchmarks, and Memcheck. The older OPEN wording is historical and superseded.

- Re-ran related-crate gates after the audit continuation: Hyperlimit 242/242 passed in 1.07s; Hypersolve 432/432 passed in 3.63s.

- Re-ran the Hyperreal cache microbench after related-crate gates. The first six-case run showed apparent +3–6% noise on ratio/pi cached paths; an immediate isolated rerun of `ratio_approx_cached_p128` measured 19.257 ns and −2.84% versus the stored baseline. The sign reversal demonstrates host/criterion variance, so no cache code change is justified. Benchmark metadata was refreshed in hyperreal/benchmarks.md.

- Traversed all 525 Haskell comparison `.log` artifacts line-by-line (14,025 lines; 2,940,721 bytes), recording per-file hashes, line counts, and recognized timing/RSS/exit/accuracy fields in `exact-real-references/HASKELL_COMPARISON_LOG_REVIEW.json`. Coverage entries are now `DATA_REVIEWED`, intentionally distinct from source `READ`; no implementation or correctness claim is inferred.

- Ran constructible closure/transfer verifiers. Source inventory/dependency hashes remain valid and report complete source reading, but the transfer verifier aborts on a changed `hyperreal/src/computable/node.rs` hash before numerical checks; the difference is test-module inclusion in the current user tree, not a production arithmetic mismatch. Existing field qualification remains bounded evidence, not a current-source closure claim.

- Current-source focused tests pass: fractional-separation suite 4/4 and cache-rescale suite 5/5, including extreme precision gaps, concurrent monotone cache publication, inverse/root identities, and MPFR cross-checks.

- Final current-tree library gate: Hyperreal 679/679, Hyperlattice 19/19, Hyperlimit 242/242, Hypertri 3/3, Hypercurve 910 passed/0 failed/1 ignored (865.99s), and Hypersolve 432/432. No new exactness or performance regression was exposed; no additional production change met the keep threshold.

- Reconciled the Haskell comparison coverage ledger with the completed data audit: all 525 archived logs now carry `DATA_REVIEWED` plus their exact `1-N` line ranges and preserved SHA-256 values in `HASKELL_COMPARISON_READ_COVERAGE.tsv`; implementation-source counts remain intentionally separate.

- Completed the remaining Haskell comparison asset audit: all 28 PNG/SVG charts passed binary/XML envelope checks, dimensions/line counts were recorded, and coverage is now zero `UNREVIEWED_ASSET` rows (`REVIEWED_ASSET` status). They remain visualization provenance, not scalar source.

- Propagated those reconciled statuses into the primary `HASKELL_COMPARISON_FILE_INVENTORY.tsv` as well: 525 logs are `DATA_REVIEWED` and 28 charts are `REVIEWED_ASSET`, eliminating stale `UNREAD` inventory claims.
