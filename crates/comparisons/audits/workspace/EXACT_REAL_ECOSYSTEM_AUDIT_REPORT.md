# Exact-real ecosystem audit — interim report

The audit has produced retained correctness, completeness and workload-specific
performance improvements. **The full requested file-by-file, line-by-line audit
is not complete.** This summarizes verified checkpoints, not a final completion
claim. Prior results belong to their frozen source versions; new user changes
are not retroactively qualified by old test runs.

## Final-report summary of verified results

- Keep Hyper's shared immutable scalar expressions, bounded refinement caches
  and explicit exact/refined/Unknown boundaries. Local proof preservation and
  demand-driven work have paid off; a wholesale representation replacement has not.
- Historical retained work fixes scalar enclosures/domains/series, solver
  certificates and exact curve coefficients. The Calcium/FLINT continuation has
  retained seven further changes, listed with benefits and costs below.
- Latest retention: certified divisor zero-factor removal recovers 184 answers
  in 16,692 paired queries while preserving the other 16,508 complete reports.
  Strict zero-exclusion, signed resultant orientation and proof replay remain
  enforced. Repeated policies/scales are not independent defects.
- In the measured same-result bounded-deflation groups, native time is
  0.210–0.809× baseline and WASM 0.216–0.790×; native allocation requests/bytes/peak
  decrease. Slower controls and extra work for new answers are accepted and
  disclosed. Two representative stripped binaries each grow 5,744 bytes.
- Latest retained-source qualification (checkpoint 75): 817 default tests; 818 all-feature debug and release
  each; Clippy, formatting and WASM compilation pass. The byte-identical frozen
  consumer passes 1,764 tests, with nine pre-existing ignored. Final verification
  binds 1,331 artifacts and the 957-file live map; earlier failed gates remain recorded.
- Latest isolated revision preserves 32 added twelfth-turn radical proofs and
  passes 766 default / 870 all-feature tests in debug/release, including new
  differential checks against the frozen evaluator. Direct tan/cot forms and
  sparse arithmetic cut selected hot queries to about 42% of the prior candidate's
  time. They still cost roughly 4x live baseline, with 26 versus 6 allocations;
  failed-proof penalties persist. This revision is not retained. Full certificates
  match the prior candidate; no live edit or loss of exact answers.
- The inverse-angle audit independently proves a FLINT completeness failure:
  `atan(tan(pi/3360))/pi` is exactly `1/3360`, but its one-candidate search returns
  not recognized. Exact final validation prevents a false accepted answer.
  Hyper preserves this identity through its compact angle form; equivalent
  radical inputs can still return Unknown. No new production change is retained.
- The scalar-boundary pass completes another 42 files / 1,902 lines.
  Independent checks validate float imports, reciprocal inverse branches and
  integer boundaries through +/-2^256 with offsets down to 2^-1024. Hyper's tested
  certified rounding results are correct; some reciprocal inverse proofs remain
  Unknown. A smaller rounding-candidate search is only an unmeasured idea.
- Quadratic extraction adds four complete files / 644 lines. All 99,587 native
  checks pass, including selected conjugates and reconstructed enclosures. Hyper
  proves all 934 normalized identities and 934 squared identities, with every
  ordering correct. Nonminimal residuals still admit certified equality; there
  is no demonstrated completeness reason to add full factorization.
- Symbolic conversion adds 26 complete files / 5,728 lines. Current FLINT can
  falsely accept nonzero radian arguments as Pi-scaled and report success while
  leaving large explicit-Pi results uncomputed. Independent oracles confirm both
  families. Rejected large powers leak temporary integers; a two-clear audit-only
  control removes the leak without changing results. These are donor defects,
  not Hyper defects. Complex-component native checks pass, but a reproducible
  Valgrind/LLL assertion leaves that collector's memory qualification failed.
- Hyper passes the corresponding angle and norm-enclosure checks in debug/release.
  Norm equality still returns Unknown on 24 exact inputs at two policies. A
  narrow complementary-angle certificate remains an unqualified candidate;
  no new production change or benchmark win is claimed.
- The remaining qqbar pass adds 37 complete files and completes one partial:
  5,324 new lines. Both pinned qqbar directories are now fully read (302 files,
  28,491 lines), including tests. Three more donor defect families are reproduced:
  large-exponent dispatch abort, signed root-degree overflow, and root-vector
  leakage on budget failure. Independent checks validate 1,078 returned algebraic
  values; the normal-input Memcheck collector is clean. Hyper's 160 enclosures
  are correct and full debug/release streams match. Three long-polynomial
  identities remain Unknown at two policies; no false equality or new retention.
- The coq-aern pass opens a previously pending reference: 21 complete files and
  one partial, 4,666 lines. Its continuous selection/limit contract differs from
  choosing a winning operand. Hyper already expresses continuous extrema through
  exact abs: all 96 independently checked enclosures pass, including 16 pairs
  whose bounded ordering is Unknown. The borrowed min/max behavior is documented,
  not a correctness defect. Coarse magnitude/rounding and erased proof assumptions
  are not suitable substitutes for certified exact decisions. No new retention.
- The next coq-aern pass adds 4,454 lines, bringing that reference to 30 complete
  text files / 9,120 lines. Two admitted complex-root lemmas prevent a fully proved
  construction claim. Independent rational checks validate the stated finite
  error/scaling/branch cases, and 23 existing Hyper sqrt tests pass in debug and
  release. A tighter ideal Newton schedule is not a proof that Hyper can lose
  rounded-arithmetic guard bits. No new code change or benchmark win is retained.
- The metric/benchmark pass adds six complete files / 2,864 lines: coq-aern now
  totals 36 complete text files / 11,984 lines. The hand-written benchmark sqrt
  permits zero outside its declared error bound; the proved/extracted threshold
  does not have that problem. Its shell runner also enables two commented-out
  dispatch cases. These are source/model findings, not reproduced Haskell output.
  Hyperlattice passes 27 selected integration tests in each debug/release profile.
  Its Euclidean norm is not interchangeable with the donor's maximum-coordinate
  metric. No new production change or performance claim is retained.
- No universal speed, memory or size improvement is claimed. Defective historical
  bootstrap intervals were corrected, not silently reused. Continuation coverage
  for Calcium/FLINT is 1,576 complete files, 19 partial files and 200,953 uniquely read lines—not
  whole-ecosystem completion. Remaining references and transfer experiments stay open.

Latest source/capability evidence: [checkpoint 87 findings](exactcore-hyper-comparison/audits/continuation/coq-aern/v87/findings-v87.md).
The completed scalar/vector metric proofs preserve coherent paths, dimensional
shape and closed predicates. Independent BigInt models check 35,937 scalar
triples, 512 vector cases and 8,448 finite fast-Cauchy pairs. The hand-written
sqrt threshold has 510 bound counterexamples; 16,320 formal-threshold controls
pass, with eight corruption controls. At n=4, zero can be selected for x=1/64
while claiming error <=1/16, although the root is 1/8. This is a contract
counterexample, not a native Haskell wrong-output run. The tracked runner's
enabled complex benchmark names lack active dispatcher entries; its logging
and CPU-time handling also require replacement before matched qualification.
Both selected Hyperlattice suites pass all 27 tests per profile, including
Euclidean norms and checked UnknownZero handling. Shared-target recompilation
is scoped to Hyperreal/Hyperlattice and their integration binaries. The old
GHC path is gone and PATH has no Coq/GHC; no new compiler, donor build, timing,
memory/size improvement, whole-stack/all-feature/WASM or production edit is claimed.
Failed/empty sandbox attempts and approved successes are preserved. Full audit open.
The four shared-target test executables total 16,705,520 bytes; this is artifact
storage, not an application-size measurement. The first seal caught and corrected
an audit-only 15-versus-14-line reread endpoint, with its failure preserved.
Final sealed verification passes 2026-09-12T06:46:31.490Z: 64 direct artifacts,
the previous evidence chain revalidated and 957 live files unchanged. All
checkpoint commands are terminal. [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/coq-aern-verify-v87.json).

Previous source/capability evidence: [checkpoint 86 findings](exactcore-hyper-comparison/audits/continuation/coq-aern/v86/findings-v86.md).
Eight new files are completed and one partial finished, including the full
extracted complex-root module. The source requires coherent multivalued paths and
closed target predicates; approximation alone does not preserve a selected root
or nonzero membership. Sqrt.v explicitly admits two lemmas used by general complex
sqrt. Models check 17,920 strict real-root error bounds, 65,589 planner indices,
signed scales and zero boundaries. A separate 2,048-branch model confirms the
multivalued, not principal-root, contract and rejects mixed-component results.
These models are not Coq/GHC execution or repairs of the admitted proofs.
The focused Hyper gate passes 23 tests in each debug/release profile, including
directed MPFR checks. One zero-test filter, one denied subprocess and one empty
sandbox capture remain classified as unqualified attempts. Cached executables
are reused with no new binaries or source-tree copies. No production edit,
new retention, matched benchmark, new full-stack/all-feature/WASM or size claim.
Final sealed verification passes 2026-09-12T06:25:36.241Z: 57 direct artifacts,
previous evidence revalidated, unchanged 957 live files, one record and empty
stderr. [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/coq-aern-verify-v86.json).
The full ecosystem audit remains open.

Previous source/capability evidence: [checkpoint 85 findings](exactcore-hyper-comparison/audits/continuation/coq-aern/findings-v85.md).
The coq-aern inventory has 135 artifacts, including 88 text files / 42,419 lines;
only the recorded 4,666 lines receive read credit. Its formal/extraction boundary
requires explicit preconditions, fair selection and a shared limit across choices.
Hyper debug/release match 49 records / 143,914 bytes, including certificates;
all 96 extrema enclosures and twelve corruption controls pass. A separate 38,025-pair
rational overlap model is not Coq/Haskell execution. Memcheck output matches with
zero errors/lost allocations and 3,008 reachable bytes in 25 blocks. Fmt, Clippy
and the default-feature 21-package graph pass. No available Coq/GHC compiler,
regeneration, donor native test, matched benchmark, new full-stack or WASM result.
Two empty sandbox captures are preserved as unusable alongside approved reruns.
Two new audit binaries use 5,150,272 bytes in the reused target; no source-tree
copy, broad cleanup or large toolchain installation. No production edit or new
retained transfer. Final sealed verification passes 2026-09-12T05:58:01.097Z:
62 directly bound artifacts, 4,498 prior artifacts revalidated, unchanged 957
live files, one result record and empty stderr. An audit-only path mistake in
the first sealing attempt was corrected and recorded, not a production failure.
[Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/coq-aern-verify-v85.json).
The full ecosystem audit remains open.

Previous source/capability evidence: [checkpoint 84 findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-remainder-v84-findings.md).
The public donor evaluator aborts on x^(2^64) at x=1 while the generic dispatcher
returns 1. Compiler trap instrumentation and debugger evidence identify an
unchecked signed degree product in the squarefree-root routine; enormous
uninstrumented cases were never run. A separate public root-wrapper failure
leaks 408 bytes per call on this build (52,224 bytes after 128 calls), while
success controls are clean. These are donor defects, not Hyper defects.
Normal multivariate/alias/matrix/root outputs match under native and Memcheck
execution with zero errors/live blocks. All 1,078 independently checked algebraic
values are correct; fourteen corruption controls pass. Hyper's 117-record,
33,952-byte debug/release streams match, including certificates. All 160
enclosures are correct; equality gives 38 Equal and six Unknown outcomes, with
twelve corruption controls. Fmt, Clippy and the 21-package dependency graph pass.
No new production change, matched benchmark, full-stack or WASM qualification.
Existing libraries and shared build storage are reused, without a source-tree
copy or cleanup. Six audit executables total 5,500,896 bytes, not a product-size
delta. Final sealed verification passes 2026-09-12T05:15:30.461Z: 4,498 artifacts /
101 captures (79 successful, eleven diagnostic failures, nine harness failures,
two environment failures), unchanged 957 live files and empty stderr.
[Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-remainder-verify-v84.json).
The full ecosystem audit remains open.

Previous source/capability evidence: [checkpoint 83 findings](exactcore-hyper-comparison/audits/continuation/calcium/symbolic-boundary-v83-findings.md).
The symbolic corpus has 112 stale-result and 24 missing-Pi false-success rows
(two defect families across repeated policies). Eight corruption controls pass.
The native component/round-trip corpus passes 15,456 independent checks and ten
corruption controls; all successful conversions have the correct selected root.
Its Memcheck SIGABRT is preserved, not counted as a clean gate. The isolated
absolute-value reproduction aborts under Valgrind-none as well as Memcheck;
the cause is not assigned to either implementation. Six separate large-power
Memcheck exits 97 become clean only in the audit-only two-clear causal control.
Hyper debug/release match all 585 records and certificates: 3,536 independent
checks, 14 corruption controls, 144 proven norm equalities and 48 Unknowns with
correct enclosures. Fmt, Clippy and the 22-package offline metadata graph pass.
Existing libraries and build caches are reused; no broad copy or cleanup.
One donor translation-unit copy is confined to the causal-control evidence.
Final sealed verification passes 2026-09-12T04:05:53.813Z: 4,150 artifacts /
55 captures (43 successful, ten diagnostic failures, two harness failures),
unchanged 957 live files, one record / 1,155 bytes and empty stderr.
[Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/symbolic-boundary-verify-v83.json).
The full ecosystem audit remains open.

Previous source/capability evidence: [checkpoint 82 findings](exactcore-hyper-comparison/audits/continuation/calcium/quadratic-extraction-v82-findings.md).
Four new complete files cover both pinned quadratic extraction implementations
and tests. Native/Memcheck streams match across 4,651 records, with zero errors
or live blocks. Hyper debug/release match across 935 records including certificates;
all queried identities and orderings are correct. Twenty native and thirteen
Hyper corruption controls pass. Bounded normalization is retained as the current
policy; no new production change or matched benchmark is claimed. Two empty
sandbox captures are preserved as unusable, with independently verified written
files and an approved driver confirmation. Existing builds are reused, with no
broad copy or cleanup. Combined evidence passes 2026-09-12T03:23:43.880Z:
one record / 12,042 bytes, empty stderr, code 0 / null signal.
Final sealed verification passes 2026-09-12T03:26:09.135Z: 3,945 artifacts /
19 captures (17 current successes, two unusable), unchanged 957 live files,
one record / 1,034 bytes and empty stderr. [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/quadratic-extraction-verify-v82.json).

Previous source/capability evidence: [checkpoint 81 findings](exactcore-hyper-comparison/audits/continuation/calcium/scalar-boundary-v81-findings.md).
The native corpus passes 163,328 independent checks over 19,924 records;
Memcheck reports zero errors/live blocks and an identical output stream. Hyper's
19,341 records match debug/release, including exact imported ratios, cached float
views, rounding values, errors and certificates. All 322 tested floors and 322
ceilings are certified correctly. Eighteen native and twelve Hyper corruption
controls pass. Four failed harness/checker captures and their corrections remain
preserved. No new production change, benchmark or full-stack regression is claimed.
Existing libraries/build caches are reused without a broad source copy or cleanup.
Final sealed verification passes 2026-09-12T03:02:20.667Z: 3,863 artifacts,
23 captures (17 current successes, two development successes, four preserved
failures), unchanged 957 live files, one result record / 1,200 bytes and empty
stderr. [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/scalar-boundary-verify-v81.json).

Previous source/capability evidence: [checkpoint 80 findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-inverse-v80-findings.md).
Nineteen new complete files / 1,738 lines cover both pinned inverse-angle
implementations/tests and one private header; two already-covered implementations
are rereads. The native corpus passes 25,216 mathematical checks and Memcheck
with no errors or live blocks. A separate exact polynomial/root proof establishes
the denominator-3360 counterexample. Hyper debug/release emit identical 1,849
records including full certificates: no false equality/inequality, and Unknown
remains explicit. This is not a matched benchmark or fresh whole-stack regression.
Two unusable empty captures and a corrected source-count reporting error remain
preserved. Existing builds are reused; no broad copy or cleanup.
Final sealed verification passes 2026-09-12T02:06:25.019Z: 3,764 artifacts,
25 captures (22 current, one development, two unusable), unchanged live sources,
one result record and empty stderr. [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-inverse-verify-v80.json).

Previous revision evidence: [checkpoint 79 findings](exactcore-hyper-comparison/audits/continuation/calcium/twelfth-revision-v79-findings.md).
The three-way campaign checks 65,664 observations and 3,456 preflight records.
Same mathematical answers and identical proof certificates are reported
separately; all prior candidate certificates are preserved. No new retention,
WASM runtime, downstream consumer or representative linked-size result is claimed.
Two new binary snapshots total 4,403,848 bytes; existing build storage is reused.
Combined verification passes against 3,661 bound artifacts and 149 captures;
failed, development and unusable empty captures remain explicitly classified.
Final sealed verification passes 2026-09-12T01:10:04.036Z with unchanged sources.

Previous candidate cost evidence: [checkpoint 78 findings](exactcore-hyper-comparison/audits/continuation/calcium/twelfth-cost-v78-findings.md).
All 40,608 main/isolated observations and 2,304 preflight records are checked;
changed-answer benefits are separated from same-result regressions. Final
verification passes 2026-09-12T00:13:42.729Z, binding 2,899 artifacts and 277
captures (274 current successes, two development successes, one preserved
failure). Four small binary snapshots total 8,750,456 bytes; the existing build
cache is reused, with no new Hyperreal tree copy or cleanup. The
[checkpoint 77 correctness qualification](exactcore-hyper-comparison/audits/continuation/calcium/twelfth-relation-v77-findings.md)
remains valid, with live sources unchanged.
The [preceding source audit](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-trig-v76-findings.md) remains valid.
The [seventh retention](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-retained-v75-findings.md) remains unchanged.

## Statistical qualification notice — checkpoints 60–64

A defect was found in this audit's bootstrap resampling code: the low bits of
its deterministic LCG force fixed index-residue counts in each twelve-draw
sample. That is not ordinary independent sampling with replacement. Historical
nominal bootstrap confidence intervals using this sampler, including their
directional/significance claims, are withdrawn pending corrected reanalysis.
Original raw timings, paired point estimates, marginal medians, numerical
correctness checks and allocation measurements are unchanged. Historical
artifact-integrity verification did not establish validity of the estimator.

The point-family campaigns (53/56/57/59) have been recomputed and independently
checked from all 151,296 raw observations: 3,920 interval comparisons, with every
point estimate preserved. A tested rejection-sampled pseudorandom bootstrap and
a separate order-statistic median interval replace the defective inference.
Across these comparisons, 269 formerly directional bootstrap claims now include
no change. A prominent apparent WASM slowdown's interval changes from
[1.107369,1.333005] to [0.973856,1.360412]; its paired point estimate remains
1.217574. The useful Unknown-control benefit and retry cost remain directional.

Checkpoint 61 corrects all five retained planner/derivative campaigns from
another 10,672 measured rows / 199 comparisons. Seven more formerly directional
comparisons become inconclusive, including a 4,096-bit fresh-WASM gain and the
derivative campaign's sole apparent slowdown. The large deep-precision planner
and high-order derivative benefits, and disclosed 2.7% native coarsening / 5.3%
warm-WASM costs, survive both corrected interval methods. Numerical and
allocation evidence is unchanged. Details:
[retained planner/derivative correction](exactcore-hyper-comparison/audits/continuation/calcium/retained-statistics-v61-findings.md).

Checkpoint 62 corrects seven accepted proof/reuse, polynomial-facts and monic
CPU campaigns: 24,048 rows / 675 comparisons. Thirty-nine more directional
comparisons become inconclusive. A separately reanalysed 1,728-row run remains
rejected for overlap with Memcheck. The large reuse/facts gains and substantial
unresolved-query costs remain; monic has six corrected above-one intervals,
five under both methods. Its allocation savings and completeness rationale are
unchanged. An additional 5,040 allocation rows recheck, without treating their
441 withdrawn instrumented timing intervals as CPU evidence. Details:
[proof/facts/monic correction](exactcore-hyper-comparison/audits/continuation/calcium/proof-facts-monic-v62-findings.md).

Checkpoint 63 corrects five unselected complex-product, rank and sign-filter
campaigns: 25,728 CPU rows / 536 comparisons, plus 3,216 separate allocation rows.
Thirty-four more directional comparisons become inconclusive. The major rank
cost, complex-product bypass/peak costs and sign-filter timing costs survive;
all five prototypes remain unselected. Thirty corruption controls pass. Details:
[prototype correction](exactcore-hyper-comparison/audits/continuation/calcium/prototype-statistics-v63-findings.md).

Checkpoint 64 corrects nine early datasets: 15,984 CPU rows / 333 comparisons.
Of these, 298 retain conditional historical qualification, 34 older log
comparisons remain source-limited, and one polynomial group stays excluded for
formatting overlap. Thirteen conditional and one source-limited directional
claims become inconclusive. Another 3,156 allocation rows recheck; their 263
instrumented timing intervals remain withdrawn. Missing pilots, timestamps,
earlier source versions and overwritten CPU executable paths are disclosed,
not reconstructed by assumption. Details:
[early scalar/proof correction](exactcore-hyper-comparison/audits/continuation/calcium/early-statistics-v64-findings.md).

Forty-five matching historical scripts, including retained-transfer measurements,
are inventoried for impact review. All now map to explicitly addressed dataset
scopes across checkpoints 60–64, not whole-workspace statistical closure. Checkpoint
65 reads all twenty additional matches in the frozen 370-file top-level inventory
and finds no independent estimator: they are wrappers, existing-interval consumers,
deterministic fixtures or prose. The 305 nonmatches and subdirectories are not
thereby semantically cleared; this is not a complete workspace statistical scan.
An initial filename-exclusion error and missing-output captures remain explicitly
preserved, not qualified by successful exit codes alone. Neither the remaining
intervals nor the full ecosystem audit are qualified by these corrections.
The new intervals still assume suitable
independent/common-distribution observations
and are not multiplicity-adjusted. Original evidence is preserved; no production
change or new timing campaign was launched in correction checkpoints 60–64.
Checkpoint 65 adds a separate focused diagnostic, described below. Details and corrected counts:
[checkpoint 60 statistical correction](exactcore-hyper-comparison/audits/continuation/calcium/point-statistics-v60-findings.md).

Earlier checkpoint descriptions below preserve their historical results. Their
original confidence-interval and significance statements are superseded by this
notice and the corrected analysis; artifact-integrity passes do not override it.

## Executive summary — through checkpoint 76

- **Exactness first:** retained historical work repairs scalar enclosures,
  domains and high-precision series, solver contraction certificates, and exact
  curve coefficients. Donor error-bound, domain and output defects reinforce
  the need for independent mathematical checks; passing upstream tests alone
  is not qualification.
- **Completeness:** the Calcium/FLINT continuation adds cache-preserving
  positive-exponential relation proofs, fact-first polynomial nonzero decisions,
  certified exact-one monic normalization, and demand-gated recovery of exact
  point-image witnesses in Hypersolve, followed by certified removal of unused
  divisor zero factors. Genuine undecided controls
  remain Unknown; none is resolved by heuristic equality.
- **Performance and memory:** its fourth retained change bounds polynomial
  derivative work by demand. One measured degree-eight/order-128 query falls
  from 155.317 to 22.623 microseconds and from 1,733 to five allocation requests.
  These are workload-specific results, with slower controls disclosed below.
  Rank-search and both constant-state sign-filter prototypes remain unselected;
  completeness or allocation gains did not justify their measured runtime costs.
  Both cold complex-product prototypes improve many wide cases but remain
  unselected because of bypass costs, higher peak demand and code-size costs.
  The fifth retained change is lower-factorial `e` planning: a fresh 65,536-bit
  public query falls from 3.405 to 1.294 ms and requested bytes from 32,693,112
  to 1,628,872 in the original campaign. Independent follow-up and WASM execution
  confirm deep cold-query gains. Public peak/live demand is unchanged; a deep native
  coarsening control is about 2.7% slower and a warm WASM control about 5.3% slower.
- **Size and architecture:** retain Hyper's shared immutable scalar expressions,
  bounded approximation caching and explicit certification boundaries. Selected
  correctness/completeness gains justify disclosed binary-size costs; no general
  binary-size or source-size reduction is claimed. No wholesale donor backend
  or representation replacement is justified.
- **Evidence:** the recorded 47-checkpoint integrity chain passed against its
  historical source map. Checkpoint 75's current-state verifier passes: 1,331 bound
  artifacts, fourteen gates (twelve successful, two preserved sandbox failures)
  and the 957-file live map, with only the qualified main-file/test-module change
  since checkpoint 67. Historical evidence explicitly preserves checkpoints 50 and 52's
  failed mathematical gates. Candidate qualification includes independent numerical
  oracles, debug/release regression suites, paired native/WASM measurements,
  allocation checks and representative binary sizes. The retained planner passes
  859 all-feature tests per live profile in its recorded campaign. Latest live
  Hypersolve passes 817 default and 818 per all-feature profile; the frozen candidate's Hypercurve
  consumer passes 1,764 tests, with nine curve tests still ignored.
  Earlier numerical failures, the unresolved LLL assertion and qualified
  thread-memory reports remain preserved. This is not an all-green history,
  full CI, or qualification of every platform and feature combination.
- **Statistical correction:** a defect in the audit's own resampler required
  withdrawing historical confidence claims. Four point-family and five retained
  planner/derivative campaigns, seven accepted proof/reuse, facts and monic
  campaigns, five unselected prototype campaigns, and the remaining early
  scalar/proof datasets are corrected rather than silently replaced. Main intended-path
  benefits and disclosed costs persist, but many smaller comparisons become
  inconclusive. The previously rejected overlapping run stays rejected.
  Raw timings, numerical qualification and allocations are unchanged. Both new
  interval methods remain conditional on sampling assumptions. Source-limited
  early records and broader estimator inventory remain open;
  no complete historical or universal retained-transfer qualification is claimed.
  Across the corrected conditionally qualified cohorts, 362 of 5,628 assessed
  comparisons lose their directional bootstrap claim; the point estimates are
  unchanged. The separately rejected overlaps and 34 source-limited early log
  comparisons are excluded from these totals. Known cache-loss/unresolved-proof
  costs survive the correction; early rejected versions remain unselected.
- **Retained point-witness completeness improvement:** Hypersolve's demand-gated fix
  recovers 825 previously lost point-image answers per policy while preserving
  the other 5,616 full records and strict proof obligations. These repeated
  cases expose one witness-loss gap, not 825 independent defects. Final consumer/
  size gates pass in checkpoint 66; the exact candidate is retained in checkpoint
  67. It avoids the unselected eager repair's measured unchanged-result allocation/
  peak penalties. Corrected native/WASM benchmarks and a four-pass diagnostic
  preserve a selected Unknown-control benefit of about 24–25% versus eager,
  but recovered answers require extra work/allocations and some retries are slower.
  Independent cold/history qualification passes 1,751,136 checks over 62,208
  full query records; native/WASM results agree. No universal speed or memory
  improvement is claimed. Accepted stripped-example growth is 1,568/1,536 bytes,
  with build-layout effects disclosed. Live adoption adds 52 net algorithm lines
  and 374 test/helper lines, including eight tests; the other 955 recorded files
  are unchanged. The separate power-sum candidate remains open.
- **Power-sum follow-up:** checkpoint 68 combines that independent witness fix
  with the deferred constructor in an isolated solver. The corrected candidate
  passes 814 tests per profile, 4,840 full-polynomial cases, both 6,441-record
  public-policy corpora and 384 nonrational/state records. All results match the
  already-repaired baseline: no new completeness gain is claimed. Focused
  memory totals improve, but per-query costs and consumer/size gates remain open;
  checkpoint 69's coefficient follow-up is below. The initial five-borrow Clippy failure is
  preserved. The power-sum constructor remains unretained and separate from
  the seventh transfer described below.
- **Wider coefficients and next completeness target:** checkpoint 69 adds
  all 23 admitted ordered degree pairs and coefficients beyond machine words.
  Both variants preserve all 3,680 query records; 81,610 assertions pass,
  including complete signed polynomials through 4,639-bit coefficients. A
  separate exact probe shows how unused zero factors in divisor carriers can
  account for the known division-resultant collapse. Seventy selected records
  pass the proposed factor-removal certificates; at that checkpoint it was not
  an implemented fix or seventy independent defects.
- **Isolated division-completeness trial:** checkpoint 70 implements certified
  divisor zero-factor removal separately from power sums. Across 16,692 paired
  queries, 184 previously Undecided/degree-rejected divisions become Transformed;
  the other 16,508 full records are unchanged. Repeated policies/scales are not
  independent bugs. The signed-polynomial/interval oracle passes 128,652 checks;
  384 extended state/history records also stay unchanged. Default tests pass
  817, all-feature debug/release 818 each; Clippy, formatting, WASM compilation
  and three native Memcheck runs pass. One new-result sign expectation and its
  formatting were corrected; the algorithm remained unchanged and all three
  failed captures are preserved. No live adoption; native cost qualification
  follows below; WASM correctness/costs and consumer/size are qualified in
  checkpoints 72–74. Exact live integration follows in checkpoint 75.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-v70-findings.md).
- **Native zero-factor costs:** checkpoint 71 records 21,888 CPU observations,
  7,296 separate allocation observations and all 1,824 pilots over 456 groups.
  All 196 unchanged-result bounded-deflation groups use 0.210–0.809× baseline
  time, with both corrected intervals below one and lower requests/bytes/peak.
  A representative existing quotient falls from 8.944 to 4.957 µs. Costs are
  disclosed: the worst early zero guard rises roughly 7.5 ns, and newly admitted
  degree-nine answers require roughly 6.8× the old rejection time. New answers
  are not equal-work comparisons. Small mixed-workload allocation increases
  vanish in 224 case-isolated paired controls; the original measurements remain
  intact and the precise state mechanism is not claimed. No universal speed/
  memory benefit or retention in that cost checkpoint. Intervals remain conditional and not
  multiplicity-adjusted. [Native findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-native-v71-findings.md).
- **WASM zero-factor correctness:** checkpoint 72 executes all 38,280 full
  observations across rational, cost-control and nonrational/history corpora.
  Every report matches qualified native evidence; the 184 new answers and
  16,508 unchanged paired rational results are preserved. Another 89,984
  independent history checks pass. Seventy-six ABI misuse controls trap as
  expected and all 21 deliberate record corruptions are rejected. Four builds,
  Clippy, formatting and dependency-identity checks pass. The final verifier
  passes 118 bound artifacts / 23 captured gates at 2026-09-11T21:08:01.072Z.
  No live adoption or WASM speed/allocation claim: qualification timings and
  linear-memory capacities do not establish those benefits. The four preserved
  modules total 5,818,881 bytes; existing source trees/build cache are reused.
  Matched WASM costs, consumer/size gates and live retention follow below.
  [WASM findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-wasm-v72-findings.md).
- **Matched WASM costs:** checkpoint 73 preserves 43,776 measured rows and
  3,648 pilots across 24 paired blocks per persistent/fresh instance mode.
  All 196 unchanged-result bounded-deflation groups per mode are faster under
  both corrected interval methods, with paired ratios 0.216–0.790 overall.
  A selected existing quotient falls from 13.209 to 7.407 µs persistent and
  12.630 to 7.245 µs fresh. Six bypass combinations show small slowdowns, up
  to a paired 3.25%; newly available answers' extra work is reported separately.
  All rows remain, including short batches; intervals are conditional and not
  multiplicity-adjusted. Fifty deliberate record/stream corruptions are rejected.
  Final independent statistical/source replay passes 430 artifacts / 111 gates.
  No new binaries, source copies, live edit or retention in that cost checkpoint.
  [WASM cost findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-wasm-cost-v73-findings.md).
- **Selection before live integration:** checkpoint 74 passes the final isolated
  consumer/size gates: 1,764 release tests, nine prior ignored, all 1,773 names/
  outcomes unchanged, Clippy, formatting and a WASM library build. Dependency
  metadata matches completely after path normalization. Both stripped examples
  run successfully and grow 5,744 bytes each (about 0.05%); build-layout caveats
  remain explicit. The certified additional answers and intended-path savings
  justify these measured costs. The exact candidate is selected, but not yet
  counted as a seventh retained transfer: live integration/regressions remain.
  Only the 355-file consumer is copied; dependencies/cache are shared. Final
  verification passes 84 artifacts / 19 gates at 2026-09-11T21:51:02.880Z.
  [Consumer/retention findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-consumer-v74-findings.md).
- **Seventh transfer retained:** checkpoint 75 promotes the exact qualified
  zero-factor main file and seven-test module. The other 955 previous source/
  support identities remain unchanged. Fresh live tests pass 817 default and
  818 all-feature debug/release each, with every name/outcome matching the
  candidate. Clippy, formatting, WASM build and complete 139-package/node
  dependency comparison pass. Final sealed verification at
  2026-09-11T22:10:23.914Z binds 1,331 artifacts / fourteen gates: twelve pass,
  two preserve a sandbox environment/driver failure before tests. No new
  numerical, consumer, timing or allocation campaign is implied by promotion;
  those qualify the frozen identical candidate. Existing cache reused, no
  dedicated executable snapshot or cleanup; post-build /tmp free 14,060,937,216 bytes.
  Use `node verify-zero-factor-retained-v75.mjs` from the Calcium audit directory.
  Older dynamic live-source flags describe historical maps, not this state.
  [Retention findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-retained-v75-findings.md).
- **Rational-angle follow-up:** checkpoint 76 reads 32 additional full donor
  files / 1,964 lines and passes 28,586 independent field/cyclotomic checks,
  including full minimal polynomials, both enclosure components, poles and root
  recognition. Native/Memcheck outputs match; zero errors/live blocks. Hyper
  debug/release capability probes agree on 1,728 observations each: 32 radical
  identities remain Unknown, repeated 192 times across budgets/shifts, while
  periodic/unequal/pole controls pass. This supports a certificate-preserving
  twelfth-turn relation candidate, not a production change or speed claim.
  Final verification binds 1,429 artifacts / twenty gates, preserving four
  harness failures. [Findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-trig-v76-findings.md).
- **Scope:** Calcium/FLINT coverage is 1,447 complete files, 20 partial files and
  185,617 uniquely counted read lines. These are continuation totals, not totals
  for the whole ecosystem. Remaining references, supporting kernels, transfer
  experiments and full inventory reconciliation still prevent audit completion.

The [latest full-chain evidence verification](exactcore-hyper-comparison/audits/continuation/calcium/results/mpoly-bridge-verify-full.json)
finished successfully on 2026-09-10 at 04:56:34 UTC. It checks recorded evidence
and source identity; it is separate from numerical tests and candidate benchmarks.
The [checkpoint 49 source/metadata check](exactcore-hyper-comparison/audits/continuation/calcium/results/fexpr-formatting-verify.json)
passed at 05:26:43 UTC without rerunning that full chain or adding any `/tmp` builds.
The [checkpoint 50 source/evidence check](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-decisions-verify.json)
passed at 05:49:16 UTC, preserving the independent mathematical failure and clean
focused memory result as distinct outcomes. It also does not rerun the full chain.
The earlier report-only recheck and all prior evidence remain preserved.

A report-only recheck on 2026-09-10 at approximately 06:00 UTC again passed the
checkpoint 50 verifier, including all 956 retained live-source identities and
the preserved mathematical failures. This was not a new numerical regression,
benchmark, memory test or full historical-chain run. No new `/tmp` files or
builds were required. At that handoff, checkpoint 51's arithmetic reads were
provisional. They are now included once in its completed selected source slice.
The power-sum candidate was subsequently implemented in checkpoint 52 but remains
isolated and unqualified for Hyper retention.

The [checkpoint 51 arithmetic/evidence check](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-arithmetic-verify.json)
passes at 18:08:04 UTC. It independently checks the new mathematical corpus and
rechecks source/evidence identity, including the preserved checkpoint 50 failures.
It does not rerun the full historical chain or the retained Hyper regression suites.

The [checkpoint 52 evidence check](exactcore-hyper-comparison/audits/continuation/calcium/results/power-sums-verify.json)
passes at 18:50:23 UTC, binding 55 artifacts, 957 isolated candidate source files,
five executables and ten terminal build/test/check/memory gates. Evidence integrity
passes while the public mathematical gate remains failed: 43,109 of 43,934 checks
pass, and all 825 failures identify lost point-image witnesses. Baseline, candidate
and Memcheck output match exactly. No matched timing/allocation campaign or new
production transfer occurred. See the [checkpoint 52 findings](exactcore-hyper-comparison/audits/continuation/calcium/power-sums-findings.md).
No donor coverage was added; the separate witness-preservation candidate followed.

The [checkpoint 53 evidence check](exactcore-hyper-comparison/audits/continuation/calcium/results/point-image-verify.json)
passes at 20:16:38 UTC, binding 154 artifacts and 33 terminal qualification/cost
captures while preserving all failed gates and 956 unchanged live-source hashes.
The guarded candidate certifies 5,126 returned roots and 1,794 exact witnesses.
The earlier witness variant passes 1,764 Hypercurve tests with nine ignored;
that is not yet consumer qualification of the final guard. Paired costs comprise
3,840 CPU observations, 160 calibration records and 480 allocation observations.
Three test/bookkeeping failures are preserved without changing a mathematical
oracle or measured output. No new production transfer or donor coverage is claimed.
Details and remaining gates: [checkpoint 53 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-image-findings.md).

The [checkpoint 54 evidence check](exactcore-hyper-comparison/audits/continuation/calcium/results/point-qualified-verify.json)
passes at 20:56:13 UTC, binding 179 artifacts and 45 successful qualification
captures. It rechecks the unchanged 956-file live snapshot and earlier failed
gates without rerunning the full historical 47-chain check. Guarded Hypercurve
release passes 1,764 tests, with nine ignored; all-target/all-feature Clippy and
a selected-feature WASM consumer build pass. The full 6,441-record corpus agrees
byte-for-byte across native/WASM and STRICT/APPROXIMATE_512, preserving the same
825 gains and 5,616 unchanged records each. Repeated outputs are not independent
new corpora or performance measurements. No new production transfer or donor
coverage is claimed. Details: [checkpoint 54 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-qualified-findings.md).

The [checkpoint 55 evidence check](exactcore-hyper-comparison/audits/continuation/calcium/results/point-extended-verify.json)
passes at 21:30:31 UTC, binding 73 artifacts and 15 completed captures, including
the preserved unsupported-constant decoder failure. A strengthened independent
interpreter passes 21,216 exact checks across 384 queries per variant, recovering
128 answers and preserving the other 256 query records. It checks the complete
serialized computation, exact authored interval bounds and carrier representation,
not a Hyper comparison or floating-point overlap. Both focused memory runs report
zero errors/lost blocks; 1,016 more bytes remain reachable in the candidate's whole
collector. No new CPU/allocator or WASM measurement is claimed. Details:
[checkpoint 55 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-extended-findings.md).

The [checkpoint 56 native cost verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-history-check.json)
passes at 22:11:50 UTC. All 3,072 qualification observations pass 84,864 independent
exact checks, with 256 gained and 512 unchanged policy/history/lifecycle groups.
The 36,864 CPU observations, 1,536 pilots and 4,608 allocation observations retain
every full final result. The largest unchanged-result paired slowdown is about
35%; median query times in that group are 18.242 versus 24.587 microseconds.
Same-result net return-live demand is unchanged, but 188 groups request more
allocations/bytes and 16 have a 176-byte higher measured peak. Newly successful
groups do additional work and are not equal-work timing comparisons. Per-group
confidence intervals are not multiplicity-adjusted. Four frozen binaries add
12.8 MB in `/tmp`, with existing source trees and build cache reused. No new
production transfer or donor coverage is claimed. The
[checkpoint 56 evidence check](exactcore-hyper-comparison/audits/continuation/calcium/results/point-history-verify.json)
and [findings](exactcore-hyper-comparison/audits/continuation/calcium/point-history-findings.md)
preserve these regressions and the next demand-gated experiment. The whole audit
and extended WASM cost qualification remain incomplete. The integrity capture
passes at 22:21:23 UTC, binding 91 artifacts and 15 successful captures; it
rechecks all 956 live hashes without rerunning the full historical 47-chain.

The [checkpoint 57 public qualification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-demand-public-check.json)
confirms that demand-gated recovery preserves all eager-repair public and extended
outputs. Full all-feature solver tests pass 811 per profile; focused Memcheck
runs have zero errors/lost blocks but nonzero reachable memory. The
[three-variant cost verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-demand-cost-check.json)
passes at 22:46:23 UTC: 55,296 CPU observations, 2,304 pilots and 6,912 separate
allocation observations. All 512 same-result groups match baseline request,
return-live and peak counts; paired timing ratios still span 0.882–1.169.
Against eager, the 768 groups have identical complete results and ratios
0.755–1.122. Retry allocations rise in 256 recovered-point groups, at most five
requests / 345 requested bytes per query. No universal speedup or new retained
transfer is claimed. The [integrity check](exactcore-hyper-comparison/audits/continuation/calcium/results/point-demand-verify.json)
passes at 22:52:56 UTC, binding 129 artifacts, 24 successful captures and 175
isolated candidate files, while rechecking all 956 unchanged live identities and
earlier failures. It is not a full historical 47-chain rerun. See the
[checkpoint 57 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-demand-findings.md)
for exact result/timing boundaries and the remaining cold-first-query, consumer,
size and WASM gates. The whole ecosystem audit remains incomplete.

The [checkpoint 58 cold-state check](exactcore-hyper-comparison/audits/continuation/calcium/results/point-cold-check.json)
passes at 23:13:17 UTC: 62,208 complete query records, 1,751,136 independent exact
checks and zero mathematical failures. Each of the three variants uses 1,152
fresh native processes and 1,152 fresh WASM instances, with nine recorded calls
per group and no unrecorded query warmups. Constructors and initial histories
are explicit setup, not a zero-cache assertion. Demand/eager records agree on
all 10,368 queries per platform; both recover 3,456 records and preserve 6,912
relative to baseline. The first-query gains are 384 repeated lifecycle groups,
not new independent defects. Native/WASM files match byte-for-byte.

Serialization after deep refinement changes four case/policy groups from a
certified result to Undecided/nonisolating in every variant, while serialized
inputs and independently proved values remain unchanged. This is existing
bounded-decision history dependence, not a demand-candidate regression.
The cold-first-query and extended WASM semantic gates are now closed for the
unchanged candidate. Matched WASM costs, final consumers, representative sizes
and a retention decision remain open. Six binaries add 14,358,510 dedicated
`/tmp` bytes; 201,468,576 bytes of raw observations and metadata are in the
workspace. The original lint failure and resume bookkeeping failure are
preserved. No new production transfer or donor coverage. See the
[checkpoint 58 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-cold-findings.md).

The [checkpoint 58 integrity verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-cold-verify.json)
passes at 23:22:40 UTC, binding 132 artifacts and 30 terminal captures (27
successful, three preserved failures). It rechecks the 175-file candidate,
all 956 unchanged live identities and earlier 48–57 evidence, with eleven output
records and empty stderr. It does not rerun the full historical 47-chain.

The [checkpoint 59 WASM cost verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-wasm-cost-check.json)
passes on 2026-09-10 at 23:59:16 UTC. All 4,608 qualification observations pass
129,856 independent checks and match native full records. The 55,296 measured
observations and 2,304 pilots preserve every final result. The shared Rust
workload is identical to native, with fresh instances, explicit preconditioning
and an optimizing WASM compiler selected before timing; this does not measure
default-tier startup or arbitrary browsers. Full ordering, calibration, raw
records and statistics are recomputed, without deleting slower samples.

Demand/baseline's 512 equal-result groups have paired ratios 0.801026–1.335991;
demand/eager's 768 equal-result groups span 0.752653–1.234868. A constructed
Unknown control has a 0.767287 ratio versus eager with interval
[0.736961, 0.782960], while a recovered-point retry control is 1.081198 with
interval [1.027356, 1.195473]. Other estimates are volatile: a paired 1.217574
baseline ratio accompanies opposite-direction marginal medians (121.079 versus
119.464 microseconds). Individual intervals are not multiplicity-adjusted, and
these statistics do not establish stable causal changes. Next perform a bounded
diagnostic replay of both unfavorable and favorable controls, preserving this
campaign, before final consumers/representative sizes and retention. Three
modules add 4,843,385 dedicated `/tmp` bytes; 222,399,157 raw evidence bytes are
in the workspace. No new production transfer, algorithm revision or donor reads.
Details: [checkpoint 59 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-wasm-findings.md).

The [checkpoint 59 integrity verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-wasm-verify.json)
passes on 2026-09-11 at 00:04:12 UTC, binding 104 artifacts / 19 successful
captures, 175 candidate files and all 956 unchanged live identities. It preserves
the earlier 48–58 evidence and failures, with twelve output records and empty
stderr; it is not a full historical 47-chain rerun. All commands are terminal.

The [checkpoint 60 statistical recheck](exactcore-hyper-comparison/audits/continuation/calcium/results/point-statistics-check.json)
passes at 00:21:38 UTC after recomputing all 3,920 corrected comparisons. The
old sampler fails an independently enumerated four-point bootstrap example;
the new helper passes it, rejection-boundary and reproducibility controls.
Of 3,920 endpoint pairs, 3,648 change; 269 directional claims become inconclusive.
Replica count also increases from 5,000 to 20,000, so endpoint changes are not
solely an isolated sampler effect. All 151,296 timings and point estimates are
preserved. No new `/tmp` files or binaries are needed; the analysis is 7,317,447
workspace bytes. Historical statistical-impact review takes priority before
the still-unlaunched diagnostic replay, final consumers/size and retention.

The [checkpoint 60 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-statistics-verify.json)
passes on 2026-09-11 at 00:46:19 UTC: 83 bound artifacts, four successful
captures, 175 candidate files and all 956 unchanged live identities, followed
by full corrected statistical recomputation. It records thirteen output records
/ 60,019 bytes, empty stderr and exit 0. Earlier 48–59 evidence remains preserved;
this is not a full historical 47-chain rerun or requalification of every older
confidence claim. All commands are terminal; no production changes or new
temporary files were required for this correction.

The [checkpoint 61 reanalysis check](exactcore-hyper-comparison/audits/continuation/calcium/results/retained-statistics-check.json)
passes on 2026-09-11 at 01:01:56 UTC. It covers every row in the five planner/
derivative timing campaigns, preserving all 199 paired point estimates and
marginal medians. Independent binomial-convolution controls verify order-statistic
coverage at 6/12/40 observations, and fourteen deliberately corrupted records
are rejected. The 40-block follow-up was affected by the original sampler too.
Corrected bootstrap below/above-one counts are 36/3 (original planner), 5/1
(native follow-up), 11/1 (WASM), 52/0 (derivatives), and 5/0 (endpoint consumers).
187 interval endpoint pairs change; seven directional claims become inconclusive.

Archived exact planner/enclosure oracles, Rust test membership, application sizes,
1,002 allocation rows and 320 retention rows recheck unchanged; this is not a new
Rust regression, timing, memory or size execution. Two retained decisions remain
supported on their documented workloads, not universally requalified. The
corrected analysis adds 353,998 workspace bytes and no new `/tmp` files. The
other retained/historical campaigns and full reference inventory remain open.

The [checkpoint 61 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/retained-statistics-verify.json)
passes on 2026-09-11 at 01:14:54 UTC, exit 0 / null signal: 48 bound artifacts,
three successful captures, one output record / 8,156 bytes and empty stderr.
All 956 live identities and the relevant frozen snapshots recheck unchanged.
The checkpoint 60 terminal evidence and its bound artifacts are checked; its
four-campaign recomputation and the historical 47-chain were not rerun. All
commands are terminal. No new production changes or temporary files.

The [checkpoint 62 recomputation check](exactcore-hyper-comparison/audits/continuation/calcium/results/proof-facts-monic-check.json)
passes on 2026-09-11 at 01:28:30 UTC. It preserves every point estimate in
25,776 CPU rows / 723 comparisons, including 48 comparisons in the separately
rejected overlap. Of 675 accepted comparisons, 626 interval endpoint pairs
change and 39 directional claims become inconclusive; none gains or reverses
direction. Matching recorded outcomes, separate numeric contracts and additional
answers remain distinct. Missing historical pilots are disclosed, not invented.

Twenty corruption controls pass, including outcome/pairing/count checks and
sixteen-query warm normalization. Three-block instrumented allocation intervals
remain withdrawn; a bounded min/max median interval at n=3 covers only 75% under
continuous IID sampling. All 5,040 allocation rows recheck, and the historical
eighteen-checkpoint output is reproduced verbatim before the corrected result.
This is not a new Rust, numerical-oracle, benchmark, memory, consumer or size
execution. The analysis adds 2,254,179 workspace bytes and no new `/tmp` files.
All three completeness-first retention decisions remain unchanged with their
known costs and limitations disclosed. Twenty-one known sampler matches and
the entire remaining ecosystem scope are still open.

The [checkpoint 62 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/proof-facts-monic-verify.json)
passes on 2026-09-11 at 01:38:44 UTC, exit 0 / null signal: 78 bound artifacts,
four successful captures, nineteen output records / 33,573 bytes and empty
stderr. The first eighteen records match the historical verifier output exactly;
the final record reports corrected statistics and their scope. All 956 current
live identities and relevant retained snapshots recheck unchanged. Checkpoints
60/61's complete statistical recomputations and the historical 47-chain were not
rerun. All commands are terminal; no new production changes or temporary files.

The [checkpoint 63 recomputation check](exactcore-hyper-comparison/audits/continuation/calcium/results/prototype-statistics-check.json)
passes on 2026-09-11 at 02:02:00 UTC. It reconstructs 25,728 CPU rows / 536
comparisons and 3,216 allocation rows, preserving every point estimate. Thirty-four
directional comparisons become inconclusive, with no new or reversed directional
claims. Thirty deliberate corruptions are rejected. The historical 23-checkpoint
rank output is preserved before the new result; four archived complex/sign
checkers also pass. Only the small 24-case exact BigInt sign oracle was newly
executed, not Rust suites, backend tests, Memcheck, consumers, sizes or benchmarks.

The unresolved width-32 retained rank control remains 48.006656 times baseline,
with corrected bootstrap [41.739881,51.477548]; requested bytes rise from 47,320
to 1,141,240 per query. Sixteen newly answered rank groups are different work,
not same-result speedups. Complex-product v2 retains 55 below-one intervals
among 60 candidate-reaching groups, but fourteen above-one bypass controls and
33 higher-peak groups. Sign summary/mask retain seven/five above-one intervals;
their allocation savings and mask's smaller benchmark binaries do not justify
those costs under the requested priorities. All five prototypes remain isolated.

The new analysis adds 1,638,145 workspace bytes, no new `/tmp` files or builds.
The manifest binds 95 artifacts and distinguishes six qualified gates from four
incomplete, failed or output-missing captures. The broader inventory recovers
fourteen wrongly excluded legacy filenames, including two known statistical
checkers. Nine known sampler scripts and twenty other potential text matches
remain for review; donor coverage and the full original scope are unchanged.

The [checkpoint 63 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/prototype-statistics-verify.json)
passes on 2026-09-11 at 02:06:57 UTC, exit 0 / null signal: 95 bound artifacts,
24 output records / 35,110 bytes and empty stderr. The first 23 records match
the historical rank chain exactly. All 956 current live identities and the
relevant frozen snapshots recheck unchanged; all commands are terminal. No full
60/61/62 statistical recomputation or historical 47-chain rerun is claimed.

The [checkpoint 64 recomputation check](exactcore-hyper-comparison/audits/continuation/calcium/results/early-statistics-check.json)
passes on 2026-09-11 at 02:20:00 UTC. All 333 paired point estimates are
preserved; the 298 conditionally usable historical comparisons contain thirteen
newly inconclusive claims. Thirty-four older log comparisons remain separately
source-limited, and the first polynomial group remains rejected for formatting
overlap. The 3,156 allocation rows are not CPU inference. Twenty-seven controls
reject malformed or invented records and three-block timing inference; a
pathological sample demonstrates why reproducing the old estimator is not
validation. The fourteen historical output records match the earlier chain.

Expanded log's warm unresolved control remains about 5.06 times baseline, early
sign-proof opaque work about 4.33 times baseline. Eager root/exp evaluation still
improves a fresh case to 0.429 times baseline but loses hot-operand cache reuse
at 4.55 times baseline. Erf's serialized-sign-fact failures remain uncorrected
in that isolated prototype. Polynomial v1's expensive new answer is different
work, not a same-result speedup; the later fact-first replacement stays retained.
No new retention or reversal of the later cache/fact-aware implementations follows.

The 83-artifact manifest binds five successful captures and a 990,238-byte
analysis. A captured path check finds sixteen early CPU binaries overwritten
at reusable build paths; only the two frozen polynomial executables match their
recorded hashes. Most pilots and all per-observation timestamps were not saved.
No new `/tmp` file/build, numerical/Rust/backend/Memcheck/consumer/size or benchmark
execution occurred. All 45 known sampler scripts now have addressed dataset
scopes, but twenty additional text matches and broader inventory reconciliation
remain open. Source coverage and the complete original objective are unchanged.

The [checkpoint 64 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/early-statistics-verify.json)
passes on 2026-09-11 at 02:32:09 UTC, exit 0 / null signal: 83 bound artifacts,
five successful captures, fifteen output records / 23,254 bytes and empty stderr.
The first fourteen records preserve the historical output exactly. All 956
current live identities and relevant frozen source bindings recheck; all commands
are terminal. No full 60/61/62/63 statistical recomputation or historical 47-chain
rerun is claimed.

The [checkpoint 65 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-attribution-v65-findings.md)
close semantic review of the twenty additional statistical text matches (1,371
tooling lines, no new estimator or donor credit). A separately predeclared
eight-group diagnostic reuses the three frozen WASM modules in four sequential
default/single-threaded/single-threaded/default GC processes. All 6,912 measured
batches and 192 qualification queries preserve their full native reference
records; three groups do additional certified work versus baseline.

Demand/eager wall ratios for the Unknown control are 0.7456/0.7466/0.7599/0.7553,
below one under both corrected interval methods in all passes. A small recovered-
point retry has ratios 1.0280/1.0188/1.0115/1.0110; its last order-statistic interval
includes one. Other small retry effects are inconsistent across passes. The
previously noisy case 17 slowdown is not reproduced, not proved equivalent.

The initial attempt failed before measured batches because a short query returned
zero thread CPU time. That failed capture, original code/plan and row remain
preserved. The amended protocol accepts zero readings without discarding them;
164 of 192 short qualification rows and 1,023 of 1,024 empty controls have zero
thread CPU, so it is not a fine-resolution clock oracle. No measured CPU reading
is zero. No overhead is subtracted or process-minus-thread time labelled GC time.
Default-GC post-boundary wall/thread medians are 2.23/2.36 versus 1.19/1.18 with
single-threaded GC; voluntary switch totals are 804/13/14/818. These are runtime-
sensitive observations, not a clean causal or production-setting recommendation.

Independent recomputation, 27 corruption controls, zero-counter/denominator
controls and an exact binomial-coverage check pass. The manifest binds 91 artifacts
and six successful captures. Raw completed query records occupy 30,365,957
workspace bytes; analysis occupies 1,603,578 bytes, with controls/captures extra.
No new /tmp artifact, Rust build, source-tree copy, production/donor change,
allocation/consumer/size test, retention or cleanup occurred. Original ecosystem
scope and donor coverage remain unchanged. Final consumer/size qualification
and the retention decision are next for the isolated demand repair; this bounded
runtime diagnostic does not need an indefinite search for a universal verdict.

The [checkpoint 65 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-attribution-verify.json)
passes at 2026-09-11T02:57:01.555Z, exit 0 / null signal: 91 bound artifacts,
three output records / 2,239 bytes, empty stderr. All 956 retained live identities
and 175 candidate files recheck. It recomputes the focused diagnostic, not the
historical 47-chain or 60–64 statistical campaigns. All commands are terminal.

The [checkpoint 66 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-consumer-v66-findings.md)
complete selected-demand consumer/size qualification. Only a 355-file Hypercurve
copy was added, reusing the frozen solver and scalar sources. Complete dependency
metadata matches baseline after the intended paths are normalized: 187 packages
and nodes, unchanged lockfile, one copy of each relevant Hyper crate. All 1,773
test names/outcomes match the prior consumer: 1,764 passed, nine ignored, none
failed. Formatting, warnings-denied all-target/all-feature Clippy and a selected-
feature release WASM library build pass; the latter is compile-only.

Six example executions agree. Stripped demand examples grow 1,568/1,536 bytes
versus baseline and 432/384 versus eager; BSS grows 2,528/2,568 versus baseline.
These representative observations include build-path/linker-layout effects,
not isolated function costs or a universal size/memory bound. Four preserved
example binaries occupy 50,279,984 bytes in /tmp; shared-cache growth is extra.
The [checkpoint 66 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-consumer-verify-v66.json)
passes at 2026-09-11T03:22:18.866Z: 87 bound artifacts / 21 gates, one output
record / 760 bytes, empty stderr, code 0 / null signal. This checkpoint selected
the candidate; it did not itself change production.

The [checkpoint 67 findings](exactcore-hyper-comparison/audits/continuation/calcium/point-retained-v67-findings.md)
record the sixth retained transfer. Only Hypersolve's algebraic_binary.rs changes,
byte-identically to the qualified candidate. The other 955 recorded live files
and existing resultant/root-isolation changes are preserved. The fix waits for
typed InvalidInterval, supplies a strictly proved point witness and retries the
unchanged refiner. Approximate multiplication/division must also replay the
whole image under STRICT; polynomial vanishing, containment, uniqueness and
half-open ownership are not weakened. The measured completeness benefit
outweighs disclosed retry/size costs under the requested priority order.

Fresh live debug/release solver tests pass 811 each, with zero failed/ignored
and identical names/outcomes to the isolated candidate. Warnings-denied Clippy,
formatting and all-feature release WASM compilation pass. Full candidate/live
dependency metadata matches after expected path normalization: 139 packages
and nodes, unchanged lockfile. Earlier numerical, memory, benchmark and
Hypercurve results remain evidence from the frozen qualified trees, not newly
executed live-path runs. No new dedicated /tmp artifact or source copy was needed;
the shared cache was reused, with 15,381,413,888 bytes available after live gates.

The [checkpoint 67 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-retained-verify-v67.json)
passes at 2026-09-11T03:36:46.453Z: 127 bound artifacts / ten gates, one output
record / 1,449 bytes, empty stderr, code 0 / null signal. It checks the current
956-file map and immutable historical/candidate/consumer bindings. Use
`node verify-point-retained-v67.mjs --point-live` from the Calcium audit directory
for this state. Older verification flags that assert an earlier live map are
historical, not current-state entry points after the one-file retention. The
historical 47-chain and complete statistical campaigns were not rerun. Donor
coverage is unchanged, and the full ecosystem objective remains incomplete.

The [checkpoint 68 findings](exactcore-hyper-comparison/audits/continuation/calcium/power-rebased-v68-findings.md)
resume the deferred power-sum constructor against the retained witness baseline.
Only the 175-file solver subset is copied (6,094,998 original logical bytes),
sharing frozen scalar dependencies. The original rebase passes the numerical
corpora but fails Clippy on five redundant references in the extracted fallback.
Its initial source, binaries, outputs and failed captures are preserved. Exactly
those references are removed, then corrected candidate qualification is rerun.

All-feature debug/release tests pass 814 each, zero failed/ignored; complete
names/outcomes preserve the baseline's 811 and add three power-sum tests. The
signed polynomial oracle passes 4,840 cases / 1,210 independent determinants,
including multiplicities, 136 fallbacks and 32 zero resultants. STRICT and
APPROXIMATE_512 public corpora each preserve all 6,441 records and pass 48,059
rational/Sturm assertions. All 384 nonrational/state records also match, with
11,248 independent field/serialized-value checks per variant. Genuine Unknown,
denominator, nonisolating and degree controls persist. These are repeated
qualification corpora, not newly discovered completeness gains.

Solver/harness Clippy with warnings denied, formatting and all-feature release
WASM compilation pass. Complete harness metadata matches after only intended
path normalization: 33 packages/nodes and identical lockfiles. Four serial
Memcheck runs report no errors or definite/indirect/possible lost bytes and
preserve native outputs. Public reachable bytes fall 1,840,544→1,364,672;
extended 48,648→48,296. Cumulative requested bytes fall 232,232,789→175,485,125
and 130,905,028→130,256,844. These include setup/collection/output, not isolated
per-query allocation, peak/RSS or bounded-retention evidence.

The [checkpoint 68 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/power-rebased-verify-v68.json)
passes at 2026-09-11T17:03:02.358Z: 169 artifacts / 43 gates, distinguishing 41
successful from two preserved failed captures, one output record / 1,233 bytes,
empty stderr, code 0 / null signal. Use `node verify-power-rebased-v68.mjs` from
the Calcium audit directory for this isolated state; retained live identity is
unchanged from checkpoint 67. Twelve dedicated binaries occupy 34,012,184 bytes;
the shared cache is reused, with 15,265,529,856 bytes available on /tmp after
verification. No performance campaign has run and no new production change is
retained. Broader coefficient/carrier coverage, matched costs, consumer/size
gates and the candidate's inherited path documentation remain outstanding.

The [checkpoint 69 findings](exactcore-hyper-comparison/audits/continuation/calcium/power-wide-v69-findings.md)
qualify a wider authored corpus: 1,840 cases, two policies, all 23 ordered degree
pairs with product at most nine, five construction heights and four carrier
families. Both variants emit byte-identical records: 3,540 Transformed, 80 zero-
divisor guards and 60 Undecided. The independent oracle checks full signed
polynomials including multiplicities, exact selected values, endpoints, witnesses
and metadata: 81,610 assertions / 1,791 distinct determinant constructions.
Input numerators reach 2,577 bits, denominators 258, primitive input coefficients
2,320 and resultant coefficients 4,639. Selected roots are rational points;
this is not exhaustive rational or arbitrary-interval qualification.

A separate probe revisits forty earlier division controls plus thirty new wide
cases before policy duplication. All divisor intervals exclude zero exactly.
Removing the divisor carrier's unused x^k factor preserves its selected root;
the independent resultants become nonzero and have one root in every authored
quotient image. All 810 assertions pass over seventy records / forty normalized
carrier pairs. This supports a separate completeness trial, not a retained
change or a new count of independent bugs. STRICT nonzero evidence, proof replay,
degree admission, transformed-carrier sharing and signed primitive orientation
must be addressed before implementation can be qualified.

Wide release harness builds/runs and Clippy pass; complete dependency metadata
matches after intended paths, with 33 packages/nodes and identical lockfiles.
No new solver suite, Memcheck, WASM execution, consumer or timing run is claimed.
Two executables total 5,616,216 bytes; no solver/scalar tree was copied. Input
data occupies 2,838,932 workspace bytes and paired outputs 28,016,100. The shared
cache is reused; recorded /tmp availability is 15,254,159,360 bytes.

Final evidence assembly catches an audit-metadata undefined/null mismatch.
Original code and both failed captures are preserved; making forty optional
slots explicitly null leaves serialized mathematical output byte-identical.
The [corrected checkpoint 69 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/power-wide-verify-canonical-v69.json)
passes at 2026-09-11T18:17:46.511Z: 85 artifacts / eighteen gates, sixteen
successful and two preserved failed captures; one record / 872 bytes, empty
stderr, code 0 / null signal. Use `node verify-power-wide-v69.mjs` from the
Calcium audit directory. Live sources, the isolated constructor, six retained
transfers and donor coverage remain unchanged. The separate completeness trial
now takes priority over the outstanding power-sum benchmarks and consumer/size
decision; the full ecosystem audit remains incomplete.

## Architecture conclusion

Keep Hyper's immutable shared scalar expressions, synchronized single-finest
dyadic caches, and explicit exact/refined/Unknown decisions. Hyperlattice's
specialized aggregate kernels, Hyperlimit's certification boundary, Hypersolve's
proof-carrying algebraic work, and Hypertri/Hypercurve's exact geometric decisions
already cover many useful donor patterns. The strongest transfers have been
local: preserve established mathematical facts, reuse exact work, and size
precision to demand. Whole-program replay, mutable refinement histories, and
heuristic equality are not justified replacements. This is a targeted
architecture assessment, not whole-source coverage of every Hyper crate.

## Retained results

Historical retained highlights are recorded in the
[original ledger](EXACT_REAL_ECOSYSTEM_AUDIT_PROGRESS.md):

- Scalar exactness: finite-value/binary64 enclosure corrections, negative-base
  power handling, exp/expm1 accuracy, inverse-function domain checks, and
  high-precision inverse-series coefficient-width repairs.
- Scalar completeness: guarded near-integer choice, removable sinc/cosc limits
  without deciding zero, algebraic-integer quotient separation, and evaluation
  of sqrt(square(x)) without first deciding x's sign.
- Solver/curve correctness: dimensionless Krawczyk contraction certification,
  monomial-preserving quadratic extraction, and exact high-order curve quotient
  coefficients/factorials.
- Performance and allocation demand: guarded demand-sized square-root seeds,
  scale-aware additive evaluation order, borrowed cache coarsening, bounded
  decimal formatting, joint inverse elimination and shared-scale integer
  Bernstein subdivision. Published gains are workload-specific; small-case
  regressions and binary costs remain in the ledger.

The active Calcium/FLINT continuation retained seven further changes:

| Retained change | Verified benefit | Accepted cost / qualification limit |
| --- | --- | --- |
| Cache-preserving positive-exponential relation proofs in Hyperreal | Additional exact equality/sign decisions without replacing numerical expressions or losing their operand caches | Some cold/unresolved queries cost more; bounded weak retention up to 2,304 requested bytes per touched worker, plus 400 native TLS bytes/thread; two stripped examples each grow 8,656 bytes |
| Fact-first polynomial nonzero decisions in Hypersolve | 120 more decided queries in the 630-query corpus; preserves 60 unresolved-leading controls | Same-outcome paired timing ratios 0.973–1.029 versus baseline; stripped examples grow 576/560 bytes; no blanket speedup |
| Certified exact-one monic normalization in Hypersolve | Three coefficient-comparison cases and two root checks become provable; allocation requests/bytes decrease in 56 of 162 measured groups, with no increases | Timing ratios 0.9007–1.0667 include slower controls; two stripped examples each grow 2,944 bytes; 13 sampled squarefree constructions remain unresolved |
| Demand-bounded polynomial derivatives in Hypercurve | Preserves every requested derivative; selected degree-eight/order-128 query falls from 155.317 to 22.623 µs and from 1,733 to five allocation requests | One earlier low-order control is slower; stripped examples grow 224/208 bytes; no universal speedup or memory bound |
| Lower-factorial `e` term planning in Hyperreal | Preserves the required term-count lower bound; original fresh 65,536-bit query falls from 3.405 to 1.294 ms and requested bytes from 32,693,112 to 1,628,872; stripped curve examples shrink 944/928 bytes | Public peak/live demand unchanged; follow-up deep native coarsening costs about 59 ns more and warm 4,096-bit WASM queries about 2.9 ns more; eight net algorithm lines plus four regression tests |
| Demand-gated exact point-witness recovery in Hypersolve | Recovers 825 answers per policy corpus while preserving the other 5,616 full records and strict proof obligations; avoids eager repair's measured unchanged-result allocation/peak penalties | Recovered answers add work/allocations and some retries are slower; stripped examples grow 1,568/1,536 bytes, BSS 2,528/2,568; 52 net algorithm lines plus 374 test/helper lines; repeated policies are not independent defects |
| Certified divisor zero-factor removal in Hypersolve | Recovers 184 answers in 16,692 paired queries, preserving 16,508 other full reports; all measured same-result bounded-deflation groups improve on native/WASM, with lower native requests/bytes/peak | Slower bypass controls and new-answer work; stripped examples each grow 5,744 bytes; 55 net main-file lines including docs/wiring plus a 260-line/seven-test module; no universal speed/memory/size benefit |

For the second change, a representative degree-16 retained log-self query falls
from 67.286 µs in the first candidate to 4.549 µs in the retained version, with
allocation requests falling from 1,117/160,976 bytes to 13/7,760 bytes. The original
baseline returned Unknown, so it is **not** an equal-work speedup over that
baseline. These are repeated degree/position/budget cases, not 120 independent
identities. See the [continuation checkpoint](exactcore-hyper-comparison/audits/continuation/calcium/README.md).

## Rejected or still-pending transfers

The eager root/exponential rewrite lost hot operand-cache reuse and regressed
selected workloads by roughly 5–8x. An erf rewrite lost four serialized sign facts
and slowed refinement. Log-relation prototypes gained decisions but incurred
severe repeated unresolved-query costs. None was retained as implemented; the
mathematical ideas remain open for safer scheduling and reuse.

Earlier comparisons also rejected unproved rounding, mutable/history-dependent
refinement, unbounded retained approximation lists, per-node OS threads, and
several eager-cache or representation replacements. Passing donor suites was
not sufficient: many assertions accepted Unknown equality or mere interval
overlap. Independent exact/MPFR, state, alias and boundary checks exposed defects
that those tests missed. No donor patch or external report was submitted by
this continuation.

## Polynomial findings and retained monic normalization

The polynomial source pass completes both polynomial directories, including their
tests: 74 archived Calcium and 70 current-FLINT files. Combined continuation
coverage, including the matrix/spectral/domain/LLL/nfloat/high-product/Arb passes,
is now **1,415 complete files, 20 partial files, 183,653 read lines**, with pinned
hashes and exact read ranges.
An append-only extension ledger preserves earlier frozen partial records while
counting new documentation reads without duplication.
This is not the entire reference
inventory or all recursively called supporting code.

Native controls pass 5,880 dense-series checks, 3,024 sparse-series checks, 4,608
composition/storage checks, 1,080 full-power checks and 81 root/multiplicity cases,
with clean focused Memcheck runs. Polynomial reference recurrences are
independent of the tested polynomial kernels, but share their
scalar backend; no complete branch-coverage claim is made.

The retained monic-normalization change preserves the leading coefficient as
proved exact one. In 81 known-factor Hyper cases, both versions return 68 results
and leave 13 undecided. All returned results have the expected degree and are
provably proportional to the reference. The change resolves three previously
Unknown coefficient-comparison cases and two root checks. An initial harness
failure was an overly strong normalization assertion, not an incorrect Hyper
polynomial; its evidence is preserved.

Live Hypersolve now passes 803 solver tests in each profile, including three
focused regressions. Each variant/profile also passes 1,932 state and expanded-
input queries covering serialization, warming, cancellation recovery, four-thread
use, higher multiplicities and rational scaling. All returned polynomials remain
provably proportional to the reference; state histories preserve the improvements
and unresolved controls. Both state Memchecks and a separate Rust-only thread
control report the same 48-byte possible loss in thread initialization, with no
definite/indirect loss or invalid accesses. These are not clean memory gates.

Qualification includes 7,776 CPU and 972 allocation observations: request demand
falls in 56 of 162 groups with no increases, while timing ratios span
0.9007–1.0667. Selected controls are slower, so no universal speedup is claimed.
Clippy and WASM compilation pass. Two stripped Hypercurve examples each grow
2,944 bytes and pass
their assertions; sizes include build-path/layout effects. Downstream Hypercurve
passed 1,761 tests, with the same nine pre-existing ignored tests and exact test
membership as baseline. That monic checkpoint matched all 954 live scalar/
consumer source and support files. The later derivative checkpoint below adds
one test module and changes only Hypercurve's two derivative loops. The
completeness and allocation benefits justify retention under the requested
priority order despite measured timing and size costs. Existing fraction-free
GCD machinery was also tested; it closes none of the remaining sampled
logarithmic gaps.

## Matrix findings: no production transfer

The initial matrix pass read 47 matrix/support files and 4,517 lines across both pins.
Pivot selection, recursive LU, fraction-free elimination, determinant dispatch,
Berkowitz construction and fixed-size cofactors offer scheduling ideas. Hyper
already has contiguous fixed matrices, exact integer cross-differences, and a
pivot-free Faddeev–LeVerrier determinant fallback; no representation replacement
or missing determinant-construction capability is established.

In 198 structured native cases, Berkowitz and Bareiss prove every expected
determinant. LU returns Unknown in 33 cases and default dispatch in ten, despite
certifiable singularity. All 28 donor matrix test functions pass, but permissive
Unknown assertions and their chosen dimensions/rank-check modes do not rule out
these gaps. Hyper constructs all 198 determinants and leaves 16 logarithmic
equality checks Unknown; no incorrect mathematical answer was observed in that corpus.

A separate comparison covers 310 inputs and 930 queries per build profile,
including dense matrices checked against an independent integer determinant
oracle. Bareiss/Faddeev each prove 260 equalities; the isolated Berkowitz probe
proves 266, gaining eight but losing two. **No direct replacement is retained.**
Alternate scheduling remains open. These are capability tests, not matched
performance or memory benchmarks; focused matrix Memchecks are clean.

The solve/rank pass adds 50 complete solve/rank/adjugate/test files and partial
documentation, 4,229 lines. New valid-input output controls expose stale zero-RREF
outputs and incorrect 2×2 in-place adjugate/inverse results: 153 incorrect rows
and four additional unresolved cofactor rows among 1,074 controls. These repeat
shapes, fields and routes, not 153 distinct defects. Native Memcheck is clean;
its exit code remains one because mathematical assertions fail. The archived
mechanisms were read but not executed. In-place discrepancies are documented
without assuming an unstated blanket alias guarantee.

An isolated Hyper affine-rank change continues past undecided minors of the same
size to find a certified nonzero witness, but never descends past an unresolved
larger order. In 828 queries per profile it increases certified results from 606
to 810 and preserves 18 unresolved controls. Existing 803 solver tests per profile,
formatting and focused memory checks pass. **This version is not selected as
implemented.** Its 2,688 CPU observations and 336 separate allocation observations
cover 56 groups. In a sampled 32-column retained-analysis case that remains
Unknown, the paired timing ratio is 48.01× baseline (per-group bootstrap interval
42.76–50.02×); requested bytes rise from 47,320 to 1,141,240 per query. Across the
corpus, allocation requests/bytes increase in 32 groups and decrease in none.
Known-zero timing controls also regress. The completeness idea remains open for
better search scheduling and reuse; no broader state/downstream qualification
or production transfer is claimed. Inputs are prechecked/warm, not cold scalar
construction, and new decisions are not equal-work speedups over Unknown.

The preceding support pass completes another 40 files and 2,521 lines. It examines
specialized rational/number-field multiplication, matrix powers, block polynomial
evaluation, three-valued matrix predicates and output construction. Hyper already
uses shared adjugate/determinant work for fixed-size division and specialized
repeated squaring. No missing capability or worthwhile new transfer is established.
All 1,528 new native multiplication/power/polynomial controls pass with clean
Memcheck. Integer-dot and Jordan-block closed forms avoid the tested matrix
kernels, but share their scalar backend. Sampled whole aliases, seeded outputs
and large denominators are not full branch or arbitrary-state coverage.

The solve/characteristic pass adds 32 complete files and supporting documentation ranges,
5,913 lines altogether. All 4,464 structured triangular/nonsingular solve controls
and 432 characteristic-polynomial coefficient checks pass. They include empty
shapes, seeded outputs, unit diagonals, valid whole RHS aliases, singular controls,
and rational, quadratic and logarithmic scalars. Integer/scalar RHS recipes and
known-spectrum coefficient recurrences avoid the tested matrix kernels but share
their scalar backend. Native and focused Memcheck outputs agree, with zero memory
errors and all allocations freed. These are correctness controls, not matched
performance evidence. Hyper also already has division-free Berkowitz machinery
for its algebraic-fiber polynomial domain; no new matrix transfer is retained.

The spectral pass completes both `ca_mat` source directories: 107 archived files
and 109 current files, including tests. Its 63 new complete files and supporting
ranges add 4,935 read lines. Called generic, algebraic and LLL support remains
incomplete. One donor Jordan-form test builds a dense similar matrix but calls
the routine on the original block matrix, leaving that intended case untested.

All 486 new Jordan controls pass, requiring correct block structure and, where
requested, an invertible rational transformation satisfying exact chain equations.
They cover dense/reversed similarities, repeated blocks, supplied decompositions,
nullable transformation output and supported input aliases. Focused Memcheck is
clean. In 648 matrix exp/log controls, 568 output equalities are proved, 66 singular
logarithms are correctly rejected, and 14 equalities remain Unknown. No incorrect
native value was observed. The function memory run aborts in an internal LLL
reduced-basis assertion during field-relation discovery; it is **not a clean
memory gate**. Logs and the crash dump are preserved. Its possible-loss reports
after termination do not establish normal-exit leaks; the underlying cause and
native reproducibility remain unresolved. No donor change or new production
matrix transfer is retained. This prompted the derivative evaluation experiment
below.

## Retained demand-bounded derivatives

The retained change bounds active polynomial derivative work during Horner evaluation,
skipping only entries that must still be zero. It preserves all requested output
orders, including arbitrarily high rational-curve derivatives. Three independent
exact-formula regressions pass in both baseline/candidate and debug/release.
The complete all-feature Hypercurve library/integration run passes 1,764 tests,
with the same nine pre-existing ignored tests. Its test names exactly match the
previous 1,770 outcomes plus the three new regressions.

Across 3,840 paired CPU observations in 80 groups, timing ratios range
0.1465–1.0587. Fifty-five per-group intervals favor the candidate; one low-order
control is slower at ratio 1.0420. A selected degree-eight/order-128 retained
polynomial query drops from 155.317 to 22.623 microseconds. These are warm,
prechecked public-API workloads, not cold scalar construction or universal gains.
A separate public endpoint-traversal campaign adds 768 observations in 16 groups:
ratios 0.9123–1.0121, five intervals below one and none above. This consumer
requests first-order derivatives at the affected end, so its gains are much
smaller. These per-group intervals are not multiplicity-adjusted.

In 480 separate allocation observations, requests/bytes fall in 16 groups and
increase in none. The selected case drops from 1,733 requests/223,984 bytes to
five requests/44,272 bytes. Peak and retained-byte deltas are unchanged in every
group. A 320-observation follow-up measures 1/8/32/128/512 additional queries:
peak/live measurements agree between variants and plateau by 32 through 512
in all 16 selected groups. The largest shared live delta is 196,376 bytes.
This is empirical behavior for those workloads, not a general memory bound.
Endpoint allocation measurements agree in all 16 groups, with one shared nonzero
live delta after eight queries. Focused public Memchecks
report no errors or lost blocks, with 264,184 bytes reachable in each version.

Both versions pass 1,584 exact state queries per debug/release/Memcheck run,
covering serialization, warming, cancellation recovery and four-worker sharing,
plus ten rejected boundary requests. Cancellation recovery does not prove that
the cancelled observation was interrupted. Threaded Memchecks report the same
48-byte possible loss in Rust thread initialization, matching the earlier
independent thread control; they are not clean memory gates. No definite or
indirect loss is reported. Endpoint Memchecks have zero errors/lost blocks,
with 63,176 bytes reachable each.

All-feature Clippy and formatting pass. Ordinary library features compile for
WASM; **all-feature WASM fails in both variants**, through the optional comparative-
benchmark dependency's getrandom configuration. No WASM execution is claimed.
Matched stripped basic/arrangement examples grow 224/208 bytes and pass their
assertions. The substantial high-order work savings, preserved exact outputs
and modest measured consumer/size costs justify retaining the two loop bounds
and regression module. No scalar, solver or donor source was changed in this pass.

The accompanying LLL support pass adds 2,980 read lines. Despite its name, the
current `is_reduced_mpfr` implementation uses FLINT's `nfloat` backend. All 3,360
new known-basis/scale/rounding-mode controls pass with clean Memcheck; fast binary64
checking leaves 80 reduced cases uncertified, which later routes decide.
These simple controls do not explain or resolve the earlier field-relation abort.
The next 1,653-line nfloat pass completes context setup, matrix dispatch, the
public header and directed-rounding tests, plus the first 310 lines of scalar
implementation. Word-sized fixed-precision storage and precision-dependent
cutoffs suggest localized experiments, not a wholesale exact-scalar replacement.
Experimental rounding, status-conditional tests and unresolved boundary handling
remain qualifications. At that checkpoint, called arithmetic and field-relation
support were still open; the following pass advances the arithmetic support.

## Fixed-precision support: preserve exact fallback

The next 7,568-line pass completes nfloat scalar arithmetic, its main arithmetic
tests, the manual, dot products, matrix multiplication and generic classical
matrix multiplication, plus part of the generic matrix header. The manual
explicitly distinguishes ordinary approximate results from an experimental
directed-rounding subset. Parsing, transcendental functions, most mixed-type
operations and complex arithmetic do not promise directed rounding. Public
directed real matrix multiplication uses the classical route, not the faster
fixed-point/block routes. These are important contract boundaries, not evidence
that an approximate scalar can replace Hyper's exact representation.

An independent GMP-rational driver passes 44,574 finite native cases and 74,922
exact comparisons: 31,680 scalar operations, 5,280 conversions, 6,912 forward/
reverse dots, and 702 matrix products containing 31,050 compared entries. Scalar
and conversion cases span every native word precision from 64 to 4,224 bits;
dots/matrices use nine selected precisions. Whole-input/output aliases, sparse
inputs, cancellation, exponent gaps and distributed caller rounding modes are
included. Comparisons check enclosure direction, not correctly rounded equality.
Native and Memcheck outputs match exactly. Memcheck reports zero errors and all
595,961 allocations freed. These bounded controls do not qualify exponent limits,
underflow flushing, nonfinite values, 32-bit builds, arbitrary concurrency or
the excluded/fixed-point/complex operations, and do not resolve the earlier LLL
field-relation assertion.

Hyper already has checked word-sized and six-limb exact dyadic accumulators with
arbitrary-precision fallback. Its three existing wide-alignment/borrow/fallback
regressions pass again in debug and release. Exact parsing and certified dyadic
enclosures also retain stronger contracts than ordinary nfloat approximations.
No new production change or performance claim is justified by this pass, so no
replacement-backend benchmark is presented as comparable exact work.

## Fixed-point bounds: defects and instrumentation qualifications

The following pass completes 13 fixed-point implementation, test and profiler
files, adding 3,286 read lines. An exact native-vector witness shows that a
Strassen intermediate can reach four times the input bound while the bound
routine accounts for three. Its largest value is only 2^-18: this checks the
bound without approaching overflow. The same witness fails in 24 precision/
rounding-mode configurations, not 24 independent defects. Another 52 sampled
cases expose inconsistent automatic versus explicit-cutoff bounds. These are
not claims of 52 incorrect final products.

Independent GMP-integer checks find no final-output-error-bound failures in
560 dots and 1,740 matrix products, totaling 1,922,900 comparisons per run.
The corpus covers classical, Waksman, automatic/forced-cutoff Strassen and public
fixed-point dispatch; inputs are conservatively scaled, with large precision
limited to small matrices. No overflow, raw empty-dot, output-alias, 32-bit or
complex-arithmetic qualification is claimed. Memcheck reports zero memory errors
and all 17,379,886 allocations freed, but the process exits unsuccessfully because
the intermediate-bound checks fail. Allocation totals include the exact oracle
and are not donor-only memory benchmarks.

Native and Memcheck outcome counts and reported maximum errors agree, but 908
rows differ in binary64 bound fields. A separate control without FLINT matches
MPFR in all 12 native multiplications; four differ under Memcheck even though
the reported rounding mode remains unchanged. Both outputs and the initial
byte-equality checker failure are preserved. This demonstrates a local
instrumentation qualification, not a general explanation of every discrepancy.
Each run is validated separately; numerical equality is not inferred from a
clean memory diagnostic.

No production or donor change is retained. Hyper's exact aggregate dispatch and
policy-visible uncertainty remain appropriate. A possible constant-state rewrite
of Hyperlimit's multi-term same-sign filter is recorded for a future isolated
experiment; the existing rational ring benchmark bypasses that filter, so it
cannot qualify the idea. Its two current regressions pass in debug and release.

## Complex arithmetic: existing architecture retained

The six remaining nfloat implementation/test/profiler files add 2,976 read lines
and complete all 26 tracked files in that directory. Called support remains open.
Hyperlattice already has the useful three-product idea, selected by exact-rational
reuse evidence, with separate fused cold and symbolic paths. FLINT's approximate
limb thresholds and midpoint conversions do not justify replacing those contracts.

Independent finite complex controls pass 89,628 cases: 177,144 exact rational
comparisons and 58,344 output-placement checks across all 66 word precisions,
including 37,488 actual input aliases; the others use an unused operand buffer.
Root checks use squared residuals and principal-branch signs, not an approximate
root oracle. The tolerance is explicitly corpus-scoped, not a general rounding or
enclosure guarantee. Native and Memcheck outputs match; Memcheck reports no
errors and all 2,601,723 allocations freed, including oracle allocations. All six
unchanged Hyperlattice complex tests pass in debug and release. No production
change or speedup is claimed.

A public ring-area trace probe now supplies a relevant workload for the pending
constant-state sign filter: all 15 near-cancellation queries reach its allocating
path, while 30 rational/well-separated controls bypass it. This establishes
reachability only. A replacement still needs isolated correctness/state checks
and matched performance, allocation and size measurements before retention.

## Constant-state sign filter: neither implementation retained

The first isolated implementation preserves the existing small-input path and
keeps at most two distinct nonzero signs for larger inputs. It passes 299,593
exhaustive short-sequence cases, 1,280 longer cases and trace comparisons.
Baseline and candidate pass identical 364-test suites in both profiles; public
traces match. An independent rational shoelace/Machin-series oracle proves the
benchmark rings' expected signs.

Across 48 workload groups, 2,304 paired CPU batches show mixed results:
candidate/base ratios 0.946–1.080; five unadjusted per-group intervals are below
parity, eleven above and 32 overlap it. Separate allocation measurements show
one saved request and 6–256 fewer requested bytes per filter-reaching query;
bypass controls are unchanged. Twelve groups reduce measured peak live bytes.
The unstripped CPU/allocation benchmark executables grow 2,536/2,552 bytes;
these are not representative application-size measurements.

Under the requested priorities, that does not justify retaining this version.
The second, bit-mask implementation also passes the same 364-test suites in
both profiles, including the exhaustive/long cases, and preserves all 45 public
traces. Its 2,304 CPU batches cover 8,346,816 repeated queries: ratios span
0.965–1.140, with two unadjusted per-group intervals below parity, seven above
and 39 overlapping. The allocation savings match the first prototype exactly.
Its unstripped CPU/allocation files shrink 1,176/1,264 bytes, but the mixed timing
results still do not establish a worthwhile transfer under the requested order
of priorities. Neither version is retained; production is unchanged and both
experiments' evidence is preserved.

## High-product support: documented bound does not hold generally

The next source pass reads six implementations and six tests completely, plus
header/documentation ranges: 2,196 lines. FLINT's precise high product retains
an extra guard limb but is not necessarily exact truncation. Omitted low products
can carry. The documented bound is n+2 guard-limb ulps; larger donor tests permit
2n, a weaker check for n>2, while smaller/normalised tests compare sibling
implementations. The subsequent independent qualification confirms that the
documented bound is too small for some valid inputs.

GMP full products check 53,248 outputs over 144 bounded lengths and 32 operand
patterns. Twelve public square/normalised-square outputs exceed the integer
n+2 guard-ulp bound; twelve internal naive/recursive multiplication outputs
exceed the same test bound. These repeat routes and normalisation forms, not
24 distinct defects. Twenty additional outputs fail only the stronger
full-product-residual interpretation. For example, public square has deficit
30 versus bound 25 at 23 limbs, and 124 versus 89 at 87 limbs.

A separate JavaScript BigInt reconstruction verifies 3,059 selected outputs,
including every discrepancy. It computes the exact omitted low-product sum,
explaining the excess carry error. Across the full GMP corpus there are no
overestimates, every output stays within the looser scaled 2n bound, and all
832 public full-product-fallback controls are exact. These findings do not
establish an incorrect Hyper or higher-level nfloat result.

Native and Memcheck numerical outputs match. Memcheck reports zero errors/live
blocks and all 628,419 allocations freed, including oracle allocations. Both
runs nevertheless exit one because the numerical bounds fail. These remain
failed mathematical gates, not all-green qualification.

Seven further complete source files and two header extensions add 1,708 lines:
ADX multiply/odd/even square basecases, ARF multiplication/MPFR bridging and
scratch cleanup. ARF has separate full-product rounding and bounded-size TLS
scratch dispatch; these are not the same contract as raw approximate high limbs.

Demand-sized multiplication and caller-owned scratch reuse are possible ideas,
but adopting approximate high limbs without certified tail handling would weaken
Hyper's contract. Hyper already plans approximation precision and exactly
multiplies integer approximations before final scaling. No backend replacement
or cutoff is selected, and no performance gain is claimed. The subsequent pass
reads the hardcoded/normalised/ARM assembly and selected rounding support below;
FFT and further supporting code remain open. Native controls do not qualify
other architectures or the whole exact-real stack.

## Correct-rounding boundary: no additional transfer selected

The next pass fully reads 20 assembly, ARF implementation and test files, with
two partial header/documentation records: 7,570 additional unique lines. Hard
high-product schedules exploit register-local accumulation and square symmetry;
normalisation shifts retained limbs but does not restore discarded carries.
ARM source was read, not executed or instruction-by-instruction proved.

All 612,000 finite ARF result/exactness checks pass, along with 2,880 input
checks. This is 61,200 results for each of ten routes: set/neg rounding in
separate and supported in-place storage, public/swapped/in-place multiplication,
explicit MPFR multiplication, public squaring and MPFR squaring. The corpus uses
18 operand-length pairs through 1,001 limbs, ten patterns, four sign forms,
17 precision positions and all five rounding modes. Counts include related and
repeated arithmetic, not 612,000 independent mathematical cases.

The oracle uses exact GMP integer products and quotient/remainder rounding,
decoding ARF output limbs directly. It does not use ARF rounding or MPFR as its
reference. A separate JavaScript BigInt implementation reconstructs 244,800
arithmetic roundings and matches all 7,200 result groups' decision counts and
compact fingerprints. Those fingerprints supplement the full GMP comparisons;
they are not collision-free certificates or a proof of every donor path.

Native and Memcheck outputs match byte-for-byte. Both exit zero; Memcheck reports
zero errors and zero live allocations, with all 303,090 allocations freed.
The 976,772,600 cumulative allocated bytes include oracle/setup work, not peak
RSS or donor-only cost. Earlier failed high-product bounds remain unchanged.
This pass does not qualify arbitrary exponents, thread histories, infinity/NaN,
all low-level helpers, fused/complex operations, ARM, 32-bit or FFT execution.

ARF's exact-product-then-round structure is a useful contract, not a replacement
for Hyper's constructive absolute-error approximation API. Hyper already plans
operand precision, shares square work and exactly multiplies integer
approximations. Hyperlattice already has cache-evidence-gated three-product
complex multiplication, fused cold exact-rational complex products, sparse
signed product sums and certified shared-scale dot dispatch. No donor cutoff,
new production change or new performance claim is justified by this pass.

## Fused and complex cancellation: qualification and an open performance candidate

The following pass fully reads 20 more files, including ARF addition, fused
multiply-add, sums, exact/approximate dots, memory management, donor tests and
supporting complex-integer kernels. Two header/documentation extensions bring
the increment to 4,131 unique lines. Exact sum/dot and approximate dot have
different contracts: the approximate API permits intermediate rounding and is
not qualified by a correct-rounding test.

All 829,440 scalar-component/result and exactness checks pass, with 19,584
before/after input checks. The 576 fixtures cover 12 base lengths through 251
limbs, 12 families, four selected sign masks, 12 precision positions and all
five modes. They include exact/near cancellation, finite zeros, balanced and
unbalanced components, ordinary exponent gaps and supported in-place outputs.
Complex calls contribute two checked components; counts include related and
repeated arithmetic. Checked APIs include complex multiplication/fallback and
squaring, fused add/sub-products, addition/subtraction, sum-of-squares,
four-term forward/reverse sums and two-term exact dots with/without an initial.

The reused GMP rounding oracle compares each complete integer expression with
the full decoded output. BigInt independently reconstructs 345,600 arithmetic
roundings and matches all 13,824 groups' decision counts and compact
fingerprints. Native and Memcheck output is identical (2,176,528 bytes each),
both exit zero, and Memcheck reports zero errors/live allocations. All 2,249,297
allocations are freed; 3,530,423,920 cumulative bytes include test/oracle work,
not peak RSS or donor-only allocation cost. This does not qualify arbitrary
exponents, thread histories, all aliases/lengths/strides, approximate/high
complex arithmetic, or ARM/32-bit/FFT execution.

One concrete performance candidate remains open. Hyperlattice already uses
three-product complex multiplication when cache evidence supports it, but the
cold wide-rational scalar fallback still forms four products. The donor's
shape-sensitive three-product selection and reduced scratch/copy layout suggest
an isolated common-scale candidate for balanced wide inputs. It needs matched
correctness, CPU, allocation and size measurements, including small, unbalanced,
cancellation and cache-state controls. No improvement is claimed or retained
from operation counts alone; production code remains unchanged in this pass.

## Cold wide-rational complex products: v1 not selected

The isolated common-scale candidate uses three signed integer numerator products
when both components of each operand share a denominator and a size estimate
favors the additional sum-product. The existing word path is unchanged; unequal
scales and unfavorable shapes fall back. Final reduction preserves exact signed
results, including conjugate products used by division. No donor or live Hyper
source was changed, and there is no fifth retained continuation transfer.

Both variants pass 82,944 exact component comparisons in 6,912 fixtures, with
27,648 input-preservation checks. The independent GMP rational oracle covers
16 widths through 2,048 bits, nine families, three scales and all 16 sign masks.
Direct multiply/divide and public Hyperlattice multiplication are each called
twice; related routes and repeats are included in those counts. Native and
Memcheck outputs agree. Both memory runs report zero errors or lost blocks and
the same 13,832 bytes reachable in 136 blocks, not zero live memory.

An additional 5,832 trace rows per variant reach 65,536-bit inputs, using exact
num-rational comparisons that share the Rust integer backend. The new scalar
path is selected 2,520 times. Public first-use/reuse labels agree between variants,
and repeated Hyperlattice calls retain their existing reuse schedule. This is
not arbitrary-state, concurrency or internal lazy-fraction qualification.

CPU measurements comprise 9,216 ABBA observations and 26,799,888 queries across
192 groups. Inputs are either separately constructed first-use values or
prewarmed values; construction, pool lifetime and oracle work are excluded.
Candidate/base ratios span 0.5043–1.4353, with 64 unadjusted per-group intervals
below parity, 43 above and 85 overlapping. Of the 75 candidate-reaching groups,
59 improve, nine regress (all at 192 bits) and seven overlap. Bypass and cached
controls also regress in 34 groups. Those controls prevent a general retention
claim despite the promising wide-input results.

In 1,152 separate allocation observations, requests fall in 72 groups and
requested bytes in 75, with no increases. Live deltas agree, but peak demand
rises in 66 groups, falls in nine and agrees in 117. One first-use 16,384-bit
public multiply improves from 109.536 to 81.903 microseconds while its measured
peak rises from 21,640 to 33,680 bytes. Unstripped CPU/allocation executables grow
8,864/8,792 bytes; these are benchmark sizes, not representative application sizes.

V1 remains isolated and unselected. It prompted the separate crossover,
dispatch and signed-sum lifetime revision below. No quotient CPU campaign, WASM,
other-architecture, source-size saving or universal performance/memory claim is
made. The eight preserved executables total 16,697,200 bytes; the source-only
candidate snapshot copied 45,445,675 bytes before its one-file, 68-line addition.
Baseline and existing build caches were reused; no evidence was deleted.

## Complex-product v2: better crossover, still not retained

The separate revision raises the experimental minimum to 256 bits, uses the
existing word scan for early bypass, moves the helper out of line and scopes
signed sums more tightly. It preserves the integer formulas and final reductions.
Both prototypes remain isolated; no production or donor source changes.

V2 passes the unchanged 82,944-component GMP oracle corpus, including threshold
boundaries, and 27,648 input-preservation checks. Native/Memcheck outputs match
the preserved baseline. Memory diagnostics show zero errors/lost blocks and
the same 13,832 reachable bytes. Its new 5,832-row trace through 65,536 bits
selects the candidate 1,800 times and preserves public first-use/reuse labels.
That large-input comparison uses num-rational and shares the Rust integer backend.
Baseline numerical, memory and trace evidence is reused, not rerun.

Both variants are rerun for CPU and allocation measurements. The same 192-group
CPU campaign produces 9,216 observations / 26,810,928 queries: ratios span
0.4962–1.2642, with 66 per-group intervals below parity, 17 above and 109
overlapping. Of the 60 candidate-reaching groups, 55 favor v2 and none has an
interval wholly above parity. All 17 above-parity intervals are bypass/reused
controls. These intervals are not multiplicity-adjusted or universal guarantees.
A selected first-use 16,384-bit public multiply improves from 114.657 to
79.976 microseconds in this campaign; this is not a paired v1/v2 timing result.

Across 1,152 allocation observations, requests fall in 57 groups and requested
bytes in 60, with no increases; live deltas match. Peak demand falls in 27 groups
but rises in 33. Among the same 60 selected groups, cumulative request demand
matches v1 and peak is lower in 36. The selected large public case still peaks
at 33,680 bytes versus baseline's 21,640. CPU/allocation executables grow
9,192/9,152 unstripped bytes; neither is a representative application-size result.

V2 is not selected as implemented. The wide-input benefit is established for
the measured corpus, but there is no new exactness/completeness capability to
justify the remaining general dispatch, peak-memory and size costs. The bundled
revision is not a factorial attribution experiment or proof that all possible
three-product implementations are unsuitable. Broader state/debug/crate/consumer
qualification was not pursued for this unselected version. Four new executables
total 8,367,856 bytes; four baseline executables and the shared build cache are reused.

The next audit work at checkpoint 37 returned to Arb dot products and error-bound support, then
the remaining original references. Revisit common-scale multiplication if a
measured application predominantly reaches that path; further generic variants
without new workload evidence are not justified by this campaign.

## Arb dot/error bounds: independent enclosure checks, no transfer

Checkpoint 38 reads 20 more complete donor files and three partial files,
adding 3,327 uniquely counted lines. This covers optimized, simple and precise
ball dot products, all five integer adapters, error inflation, fused update/FMA
support, relevant donor tests, and documented stride/alias contracts.

The architectural lesson is to keep midpoint truncation, propagated input
uncertainty and final rounding as separate error contributions. Arb limits work
using input uncertainty and actual product bottoms, accumulates at a common
fixed-point scale, and uses shallow normalized integer views. Its precise
reference materializes product vectors; it is neither an independent backend
nor a guarantee of the tightest possible interval.

All **198,288 results** pass **396,576 complete rational endpoint comparisons**
over 648 fixtures and 1,224 operation groups. The oracle takes exact rectangle
product extrema and sums them using GMP rationals; it does not use Arb overlap
or containment routines. There are also 21,349 successful checks of generated
Arb inputs, the initial value and fmpz mirrors. GMP still underlies some FLINT
integer work, so this is independent of the enclosure algorithm, not a second
integer backend or a formal proof.

The corpus covers twelve ball limb widths through 333, six exact/radius/
cancellation/sparse families, lengths 0/1/2/5, three valid stride layouts,
both subtraction choices, absent/separate/output-aliased initial values, and
nine precision positions. Typed wrappers use three ball widths and integer
coefficients reaching 258 bits. Counts include repeated routes and precision
positions. Source-relevant boundary inputs do not establish instrumented branch
coverage. Self-dot correlation, exceptional radius-only midpoint paths, huge
exponents, nonfinite values, direct FMA/add-error APIs, every alias and other
architectures remain unqualified. External MPFR high-helper source remains an
explicit dependency gap; its helpers are not conflated with earlier FLINT
high-product bound failures.

Native and focused Memcheck runs pass with identical 161,471-byte numerical
outputs. Memcheck reports zero errors and no live heap blocks: 687,105 allocations
and frees, 727,232,808 cumulative bytes including setup and oracle work—not peak
RSS or donor-only allocation cost. Native finishes at 22:37:45.201 UTC and
Memcheck at 22:41:02.317 UTC on 2026-09-09.

Hyper already carries certified exact factors into shared-scale reducers, plans
dyadic scale/headroom, and delays rational reduction. Truncated products cannot
replace those exact-value results without a separately certified enclosure
contract. No nonredundant transfer with measured benefit was established, so
there is **no production change or new performance claim**. The evidence is bound
in [arb-dot-experiment.json](exactcore-hyper-comparison/audits/continuation/calcium/arb-dot-experiment.json).
Its full chained verification passes with 38 JSON stdout records, 88,516 bytes,
empty stderr and exit zero. All 955 retained live hashes remain unchanged.

## Magnitude arithmetic and error inflation: no new transfer

Checkpoint 39 adds 3,978 uniquely counted source lines: 36 new full-file reads,
completion of the magnitude header, and three other partial extensions/reads.
Core upper/lower arithmetic, comparisons, constructors, integer bounds helpers,
root/power code, ten donor tests and the complete magnitude manual are covered.
At that checkpoint, remaining transcendental, tail, combinatorial and IO
implementations were not inferred complete from their declarations; the next
checkpoint reads their bodies explicitly.

The main lesson is bound polarity: an upper-bound quotient needs a lower-bound
denominator. These 30-bit magnitude objects do not promise correct rounding.
Fast paths also require finite inline destination exponents. Exact rational
checks pass for **505,048 magnitude outputs**, with equally many supplemental
quality checks and **14,688 comparisons**. There are 7,344 arithmetic input pairs,
680 constructor fixtures, 28 arithmetic and 16 conversion/composed-integer
routes. Tests include zero/neighboring mantissas, exponent gaps through 1,025,
integers through 513 bits, small signed powers and supported whole-object aliases.
Square-root bounds are checked by exact squaring. The supplemental relative
envelope is 1/1024, applied after squaring for root routes—not an ulp theorem.

The previously read Arb FMA/error APIs now pass **6,480 results / 12,960 endpoint
comparisons**: 5,760 fused results and 720 inflations across all five error APIs.
The inflation controls also prove midpoint preservation for each tested call.
There are 31,416 magnitude/conversion input checks and 1,584 ball input/midpoint
checks. GMP is independent of the enclosure algorithms but shares some FLINT
integer support; fingerprints supplement full comparisons, not formal proof.

Native finishes successfully at 23:02:08.299 UTC and focused Memcheck at
23:02:39.533 UTC on 2026-09-09. Their 197,917-byte numerical outputs match exactly.
Memcheck reports zero errors and no live heap blocks, with 9,839 allocations/frees
and 6,879,352 cumulative bytes including setup/oracle work. No peak-RSS or
donor-only allocation claim. Huge/promoted exponents, nonfinite sweeps, large
powers, direct hypot and other architectures/libm environments remain unqualified.

Hyper already distinguishes structural exact facts from approximate magnitude
planning and refines rounded-zero reciprocal divisors. A fixed 30-bit bound layer
would alter those contracts without demonstrated benefit. **No production change
or performance claim** is made. The reused build needs only a new 47,216-byte
audit executable; its size includes the unused previous harness main.

Two corrected audit failures remain preserved: formatting warnings in the first
C compile, and a read-ledger expected-total error (4,078 versus 3,978 actual lines)
caught before any coverage was applied. Formatting changed no non-whitespace
tokens; the ledger correction changed no read range. Evidence and limits are in
[magnitude-experiment.json](exactcore-hyper-comparison/audits/continuation/calcium/magnitude-experiment.json).
Its full chained verification passes: 39 JSON records / 95,243 stdout bytes,
empty stderr and exit zero. All 955 retained live-source hashes still match.

## Magnitude transcendental and tail bounds: no new transfer

Checkpoint 40 adds 44 complete-file reads and 4,660 lines: the remaining 29
top-level magnitude C implementations, two headers and thirteen donor tests.
All 52 top-level magnitude C files are now source-complete. That does not complete
all tests/profiles or recursively called conversion, table and MPFR support.
Every factorial/reciprocal table pair and logarithm constant was read; source
coverage does not imply new numerical qualification of those tables or I/O.

The finite native corpus passes **36,208 directed-bound checks**, **31,652
supplemental quality checks**, and **16,742 checks each** for input preservation
and supported whole-object alias agreement. At each of two reference precisions
(512/768 bits), it covers 371 unary inputs, 16 elementary-function routes, eight
positive root degrees, 99 exponential-tail inputs and 680 positive binary64
logarithm inputs, plus both pi bounds. Counts include repeated precisions and
aliases. Both reference precisions produce identical donor outputs.

Inputs are imported as exact 30-bit dyadics directly into MPFR, bypassing ARF
and avoiding huge integer materialization. Exponential-growth/decay inputs stop
at binary exponent 26; other selected functions cover exponents -1075..1024.
References use directed MPFR operations, not the donor's binary64 polynomials.
MPFR shares GMP integer support; two precisions are not independent backends or
a formal proof. A supplemental 1/1024 relative allowance is excluded for large
exponential-family inputs and all tails; no correct-rounding claim is made.

The donor exponential-tail test checks only a 50-term lower sum. The new oracle
sums terms N through N+256 with directed arithmetic and bounds the entire
remaining tail geometrically, for x <= 4 and eleven N positions through 257.
This certifies a full-tail enclosure for those cases, not merely a finite prefix.
Large lower-sinh's apparently mixed subtraction polarity passes the selected
transitions; compensating rounding slack still requires a general proof before
making an all-input claim. Planner log2 remains explicitly uncertified.

Native passes at 23:34:12.194 UTC and focused Memcheck at 23:34:34.109 UTC on
2026-09-09, with identical 866,255-byte numerical logs. Memcheck reports zero
errors and no live blocks; 116,716 allocations/frees and 22,125,312 cumulative
bytes include the oracle. These are not donor-only demand or peak-memory figures.
Default rounding mode only is checked; earlier FENV discrepancies remain open.
Combinatorial, conversion/I/O, other-tail, promoted/nonfinite, arbitrary-state,
thread and other-architecture qualification remains incomplete.

Hyper already checks local series domains, budgets precision and truncation,
reuses exact-rational range reductions, and obtains roots from integer bounds.
Replacing these with fixed-mantissa bounds would change its exact-real contract
without demonstrated benefit. **No fifth continuation transfer or new matched
performance claim is justified.** The evidence, corpus and limits are bound in
[mag-transcendental-experiment.json](exactcore-hyper-comparison/audits/continuation/calcium/mag-transcendental-experiment.json).

## Magnitude combinatorial, tail and conversion bounds

Checkpoint 41 adds 3,478 disjoint read lines: the remaining 28 magnitude test/
driver files, all of ARF I/O and the remaining ARF conversion ranges. All 104
inventoried files under `src/mag/` now have complete read records, along with
the previously read public header/manual. This is source completion at the
pinned revision, not full numerical or recursively called dependency coverage.

Native checks pass **48,890 exact comparisons**, **3,358 directed-MPFR checks**,
1,441 input-preservation checks, 1,240 supported alias agreements and 402 valid
string roundtrips. JavaScript BigInt independently rechecks 48,086 exact bounds
and 201 double exports; the 603 native rational/ceil/floor export comparisons
are not separately emitted for a second recomputation. Bernoulli uses different
exact recurrences in C and JavaScript, with sign conventions reconciled by the
absolute-value contract.

The corpus covers factorial/reciprocal bounds for every n from 0 through 4096,
including all 512 table pairs; 36,237 binomial inputs, 182 binomial-power inputs,
129 Bernoulli coefficients, 88 exact geometric tails, 264 finite double inputs
and 201 magnitude conversions. It deliberately avoids oversized conversions,
malformed serialization and file-error reproduction. Reading the upstream
random tests does not mean they were newly run.

Per MPFR precision (512/768 bits), full-tail controls cover 576 polylogarithm
and 527 Hurwitz-zeta inputs. Polylogarithm references use a positive prefix plus
a rigorous eventual geometric remainder; Hurwitz uses zeta minus the finite
prefix. Both precisions give identical donor outputs. **199 of the 576 convergent
polylogarithm inputs return conservative infinity**, while 377 return finite
bounds. The 796 repeated infinity outputs are included in the directed-check
count: they are valid but uninformative bounds, not finite-enclosure successes.

Native passes on 2026-09-09 at 23:59:33.497 UTC; focused Memcheck passes on
2026-09-10 at 00:00:14.166 UTC. Their 1,308,580-byte numerical outputs match.
Memcheck reports zero errors and no live heap blocks; 636,551 allocations/frees
and 73,880,835 cumulative bytes include all oracle/setup work. No peak-memory,
donor-only cost, all-FENV, arbitrary-state or other-architecture claim is made.

No new production transfer is retained. Hyper already constructs exact
combinatorial coefficients with product trees, word batching and structural
cancellation; a fixed-precision bound cannot replace those values. One narrow
planning candidate was opened: use a fixed-word **lower** factorial enclosure
to certify `e`'s term count without first constructing a growing exact factorial.
An upper bound would have the wrong polarity. At checkpoint 41 this candidate
was unimplemented and no speedup was claimed. Its isolated qualification follows
below; checkpoint 43 subsequently qualifies and retains it.

The evidence and limitations are bound in
[mag-series-experiment.json](exactcore-hyper-comparison/audits/continuation/calcium/mag-series-experiment.json).

## Lower-factorial `e` planner: isolated evidence at checkpoint 42

Checkpoint 42 replaces only the isolated candidate's growing factorial used for
term planning with a normalized 64-bit lower mantissa and exact 128-bit word
products. Downward truncation preserves the lower bound: its threshold cannot
stop earlier than the exact planner. The proof bounds the operations and
termination over the pre-existing supported precision arithmetic. Exact series
coefficients, binary splitting, rounding and shared-cache storage are unchanged.
The edit adds eight net source lines; no new donor-source coverage is claimed.

Both variants match exact GMP thresholds on **4,151 requests**, including
factorial-threshold neighbors and requests through 262,144 bits. No extra terms
occur in that corpus. Instrumentation checks all 20,367 lower-bound steps on
the deepest request against exact GMP. Each variant passes 279 directed MPFR
kernel/refinement/coarsening checks and 21 cancellation-recovery, serialization,
exp(1) and threaded-state checks. Independent JavaScript BigInt recomputes all
8,302 planner answers and 600 numerical bounds using a different exact series
recurrence with a complete geometric remainder—not just a finite prefix.

Four focused sequential Memchecks report no errors, lost or suppressed blocks.
They retain 544 runtime bytes for planner checks and 35,656 runtime/cache/MPFR
bytes for numerical checks in either variant; these are not zero-live-heap
results. The threaded state checks were not separately Memchecked here.
Native and Memcheck outputs match on all 8,864 emitted rows. All-feature
Hyperreal library/integration tests pass **855 tests in each variant/profile**,
with identical membership and no ignored tests. The initial audit build failed
on two Rug endpoint conversions and an unused import; its source and output
are preserved, and the corrected harness builds without warnings.

The predeclared 71-group benchmark records 3,408 CPU observations and 426
separate allocation observations. CPU 6 runs 12 alternating ABBA/BAAB blocks;
fresh/refining public calls use one query in each new process, while cached
queries and uncached kernels are measured separately. Per-group paired
bootstrap intervals are not multiplicity-adjusted. All ten planner and ten
kernel groups improve; across all groups 37 intervals lie below one, three
above, and 31 overlap. Public fresh-query examples are:

| Requested bits | Baseline → candidate time | Allocation requests | Requested bytes |
| --- | --- | --- | --- |
| 4,096 | 104.068 → 60.481 µs | 773 → 193 | 171,688 → 11,896 |
| 65,536 | 3.405 → 1.294 ms | 11,558 → 4,643 | 32,693,112 → 1,628,872 |
| 262,144 | 37.566 → 9.022 ms | 50,868 → 26,426 | 462,939,408 → 13,606,968 |

Requests and requested bytes decrease in 28 groups and never increase; live
demand is unchanged in all 71. Seven planner-only peak measurements decrease,
but **public and full-kernel peak demand is unchanged**. Public coarsening at
precision 0 and −262,144 has slower intervals, as does the unchanged warm-pi
control at −4096. Tiny/cached timings and unchanged-path controls expose
noise/layout sensitivity; no blanket speedup is claimed. The audit CPU and
allocation binaries shrink 1,304/1,472 bytes while the oracle executable grows
944 bytes. These are not representative application-size measurements.

At checkpoint 42 this was a pending candidate, not a retained change. Durable
in-crate regressions, default-feature/downstream qualification, target checks
(especially the u128 implementation cost), Clippy/fuzz/WASM and representative
sizes remained. Existing unsupported i32::MIN behavior is untouched. The evidence
and limits are bound in
[e-plan-experiment.json](exactcore-hyper-comparison/audits/continuation/calcium/e-plan-experiment.json).

## Qualified and retained `e` planner; ARF implementation source closure

Checkpoint 43 retains that exact production planner after qualification. Only
`hyperreal/src/computable/approximation/constants.rs`, four test-registration
lines in `approximation.rs`, and a new 112-line `e_plan_tests.rs` module change
relative to the previous retained snapshot. The tests cover exact factorial
thresholds, directed MPFR enclosures through 262,144 bits, shared refinement/
coarsening, and cancellation/serialization recovery. They do not assume a cold
process-wide cache. No runtime dependency, series coefficient, splitting,
rounding, cache or representation change is introduced.

Both profiles pass 752 default-feature baseline tests and 756 candidate tests;
the candidate and then the live tree pass 859 all-feature tests per profile.
Exact membership differs only by the four added tests. The candidate also passes
24 doctests, strict all-target/all-feature Clippy, formatting, default checks and
fuzz-target compilation. Baseline and candidate release consumer suites match
exactly: **803 Hypersolve tests and 1,764 Hypercurve tests pass**, with the same
nine pre-existing ignored curve tests unrun. Consumer Clippy and supported
WASM library builds pass. This is not full CI or every downstream feature matrix.

WASM is executed, not merely compiled, in local Node 22/V8. Across both variants,
8,302 planner answers match an independent exact-factorial oracle; 600 complete
integer outputs satisfy independently recomputed exact-series/full-tail bounds.
All 300 outputs per variant agree, and all 186 direct-kernel outputs agree with
their native counterparts. Fresh module instances, refinement and coarsening
are sampled. This does not qualify browser integration, WASM threads, or physical
ARM/RISC-V devices.

The native follow-up uses 40 alternating ABBA/BAAB blocks, 1,600 observations
and ten predeclared groups including all three previously slower controls.
Fresh 65,536-bit public `e` takes 3.650 → 1.575 ms; the 262,144-bit query takes
38.035 → 9.073 ms. Deep coarsening still has a slower paired ratio of **1.0271**
(95% interval 1.0235–1.0295), approximately **59 ns per query**. Earlier unfavorable
results remain preserved; this targeted campaign supplements, not replaces,
the original 71 groups.

The separate WASM campaign has 22 groups, 1,056 observations and 12 alternating
blocks. Twelve paired intervals are below parity, one above, nine overlap.
Fresh public 65,536-bit queries take 8.085 → 3.845 ms; 262,144-bit queries take
82.280 → 31.120 ms. Warm 4,096-bit queries are slower at ratio **1.0532**
(1.0309–1.0710), about **2.9 ns**. Intervals in both campaigns are per-group and
not multiplicity-adjusted. Large cold/uncached and allocation-demand benefits
justify retention despite these disclosed cached-query costs; no universal
speedup or reduction of public peak/live memory is claimed.

Three unchanged native examples per variant are built, stripped, measured and
run. Hypercurve `basic` and `arrangement` shrink **944 and 928 stripped bytes**;
Hyperreal's `readme_quickstart` is byte-identical. The WASM audit module shrinks
507 bytes. These are not whole-Alumina, all-feature/LTO or application-runtime
measurements; the scalar example need not link the changed planner.

The source pass reads **18 further ARF implementation files, 1,176 lines**,
completing all 41 inventoried top-level `src/arf/*.c` files. Tests, public-header
ranges and recursively called MPFR/GMP support remain separate. Finite-dyadic
rounding, sticky quotient correction and exponent rescaling offer
representation-specific ideas, not a demonstrated additional transfer. ARF's
nearest-even rounding, NaN ordering conventions and nonnegative root domain do
not substitute for Hyper's certified ties-away rounding, partial comparisons
and real odd-root contract. These new source reads have no new independent
numerical campaign in this checkpoint.

[The qualification manifest](exactcore-hyper-comparison/audits/continuation/calcium/e-qualified-experiment.json)
binds 211 evidence files, 55 successful captured gates, 956 retained source
hashes and 14 executable identities. The initial app-size command rejected an
invalid capture tag before spawning `strip`; its script and already-built
executable are preserved. A draft read range ending at line 4205 was corrected
to the physical end at 4200 before the successful draft verification; the
original draft remains explicitly unverified. Neither issue altered numerical
evidence. After retention, a separate
[versioned historical-source verifier](exactcore-hyper-comparison/audits/continuation/calcium/e-qualified-snapshot-verification.md)
preserves all original scripts and checks old claims against the frozen 955-file
snapshot, while checking the current 956 files independently. All 43 checkpoint
verifiers pass; the first 42 output records are byte-identical to the prior run.

## ARF contracts and test-source closure: stronger qualification, no new transfer

Checkpoint 44 completes the remaining 47 ARF test/driver files and header/manual
gaps: **5,783 disjoint lines**. All 104 inventoried files in `src/arf/`, `src/arf.h`
and its manual are now source-read. Two supporting random-helper excerpts add
66 lines and remain partial. This closes that source slice, not all recursive
integer/MPFR support, every numerical path or the ecosystem inventory.

The source audit identifies concrete weaknesses in the donor tests:

- Main add/sub tests overwrite random rounding choices with truncation.
- The signed-add wrapper's random upper limit of one makes its in-place branch
  unreachable; the helper's bitmask implementation confirms this directly.
- Two binary64-conversion switches have unreachable nearest-even defaults.
- Root/sqrt/reciprocal-sqrt tests omit nearest-even and share MPFR with their
  reference calculation.
- The approximate-dot reference error sum uses the first vector's orientation
  for both vectors, even when their tested orientations differ.

These are source-level qualification gaps, **not newly proved library output or
memory defects**. The added finite harness independently checks add/sub/div and
positive roots of degrees 1, 2, 3, 5, 7 and 17, plus reciprocal square root.
It explicitly enumerates five rounding modes, 14 precisions through 257 bits
including one bit and word neighbors, and supported whole-object aliases.
Inputs include exact powers, exact halfway roots and adjacent input values,
bit/exponent contrasts, cancellation and zero numerators. Denominators are nonzero.

Native GMP exact-rational scaling, integer roots and halfway powers validate
**518,490 full results and exactness flags**, 316,260 alias agreements and
64,518 input-preservation checks. There are 64,860 exact outputs and 890 repeated
halfway ties. A separate BigInt checker regenerates all fixtures and verifies
202,230 primary neighboring/halfway-power certificates plus every alias output
in full. This includes 103,698 nearest-even checks; it does not use MPFR output
agreement or reproduce GMP's root-search algorithm.

Native and focused Memcheck outputs match byte-for-byte, 18,296,286 bytes each.
Memcheck reports zero errors, suppressed reports or live heap blocks;
1,750,780 allocations are freed. Its 37,381,718 cumulative bytes include oracle
and setup work, not donor-only demand or peak/RSS measurement. The campaign is
finite sequential native-64-bit/FE_TONEAREST qualification, not all targets,
floating environments, operations or exceptional states.

Hyper already separates certified ties-away integer rounding from total
multivalued adjacent-integer choice, uses guarded integer/Newton square roots
and exact-power nth-root enclosures, supports negative odd roots, and uses sticky
round-to-odd compression for dyadic binary64 conversion. No additional
nonredundant production transfer or matched performance claim is established.
The five retained continuation improvements and all 956 live hashes are unchanged.
Nearest-binary64, remaining integer-rounding/comparison and approximate-dot
qualification gaps remain explicit follow-up work.

[The ARF contract manifest](exactcore-hyper-comparison/audits/continuation/calcium/arf-contract-experiment.json)
binds 27 files, five successful captured gates, 51 read records, current sources,
libraries/configuration and the 28,392-byte executable. All 44 checkpoint
verifiers pass; their earlier 44 output records (including the snapshot-binding
record) are byte-identical to the previous run. No donor/production patch,
cleanup, deletion, commit, push or external report occurred in this checkpoint.

## Finite ARF conversion and integer contracts — checkpoint 45

A bounded 2,401-fixture campaign closes the unreachable nearest-even test gap
identified in checkpoint 44. Signed dyadics span 257 mantissa bits and peak
exponents -1100 through 1100, with structured patterns and explicit midpoint
neighbors. The exact GMP grid oracle and independent BigInt ordered-bit search
pass 117,705 assertions. The latter regenerates every input, locates adjacent
binary64 values and compares exact rational midpoints; it does not reuse MPFR
or the native grid-selection algorithm.

Results include 12,005 complete binary64 outputs, 2,401 nearest-even cases,
164 binary midpoint fixtures, 11,281 finite imports, 12,005 integer values and
exactness flags, and 7,985 signed conversions whose expected values first prove
they fit. Full floor/ceil/nint outputs total 14,406; 9,604 public aliases cover
those operations and frexp. Decomposition, integrality, magnitude bounds, signed/
absolute comparisons and input preservation also pass. There are 1,848 subnormal
outputs, 460 negative zeros and 724 overflow infinities. These counts include
repeated mode/route cases, not independent mathematical identities.

The first harness failed one assertion because it expected zero's signed
magnitude bound to be zero; the donor correctly returns its documented negative
sentinel. A separately generated correction also skips two fmpz bound calls
whose zero results are unspecified and serializes the signed sentinel exactly.
The original source, executable and failed gate remain preserved. Every
non-bound numerical output is identical between versions. No donor bug is
inferred, and no fixture was dropped from the corrected inventory.

Corrected native and Memcheck outputs are byte-identical, 2,781,706 bytes each.
Focused Memcheck reports zero errors, suppressed contexts or live blocks; all
133,965 allocations are freed. The 3,847,142 cumulative allocated bytes include
the oracle and setup, not donor-only cost or peak memory. Qualification is
native64 ADX with FE_TONEAREST, not every hardware rounding mode or target.

The [conversion manifest](exactcore-hyper-comparison/audits/continuation/calcium/arf-conversion-experiment.json)
binds 38 files and eight captured gates: seven successful and the preserved
one-failure original harness. Source progress adds all 238 lines of fmpz/set.c;
ARF and fmpz/get.c rereads receive no new credit. Hyper already separates exact
and lossy conversion, uses sticky round-to-odd compression, and provides
certified ties-away rounding and multivalued near-integer choice. Those are
different contracts from finite ARF nearest-even/total ordering. No new
production transfer or performance benchmark is justified. The same 956 live
source hashes retain the prior five continuation improvements and qualification.

All 45 checkpoint verifiers pass at 2026-09-10T03:17:12.671Z, with empty stderr.
The first 45 output records (124,861 bytes, including the historical/current
binding record) exactly match checkpoint 44. Verification preserves the failed
original harness as failed; it does not turn the test history all green.

Two preserved executables total 57,336 bytes in `/tmp`; three numerical logs
total 8,340,310 workspace bytes. No Rust/whole-native rebuild or source copy was
needed. Approximate-dot reference pairing, signed-add alias qualification,
fixed-grid wrappers, recursive support and the remaining original reference
inventory remain open. This is progress, not whole-audit completion.

## Multivariate rational-function architecture — checkpoint 46

All 36 current FLINT rational-function implementation/test/header/manual files
are read, totaling 3,564 lines. The archived manual and supporting API/test-driver
reads bring this checkpoint to 3,955 newly credited lines: 38 complete files and
two partial files. Archived implementation, generic-ring/fexpr bridges and
recursive polynomial/GCD support remain separate requirements.

The main ideas are denominator-GCD staging, scalar-content shortcuts and
cross-cancellation before multiplication. Hyperreal already uses rational
cross-cancellation and restricted possible-divisor reduction; Hypersolve already
recognizes equal denominators, represents denominator one without a vector and
reduces products modulo its retained root polynomial. Its partial Real
coefficients and selected-root nonzero obligations differ from a total formal
rational-function field. Formal cancellation cannot remove the authored
denominator obligation at a selected root. Existing rational-image cancellation
is certificate-guarded. No new production transfer or matched performance claim
is justified by this pass.

The independent corpus spans twenty structured fractions in each of nine
contexts: 1/2/4 variables crossed with all three monomial orders, shared linear
factors, scalar denominators and 66/130-bit common integer content. Full BigInt
cross-polynomial identities and complete factor-based canonicality certificates
pass for 23,841 values, including 13,797 public aliases. The 10,044 primary
certificates check complete coefficients, not sampled point agreement or residues.
Known primitive linear denominator factors and integer content make canonicality
independently decidable for this corpus; this is not a general GCD proof.

All 15 upstream tests also pass, unfiltered at multiplier one. Their random
contexts use lexicographic order and their references share polynomial/GCD
machinery; the independent corpus supplies the separate oracle and explicit
scalar-wrapper alias coverage. Valid generated string roundtrips run, but no
malformed input or failure-output qualification is claimed.

Native and focused Memcheck numerical outputs are byte-identical, 3,087,444 bytes
each. Memcheck reports zero errors, suppressed contexts or live blocks; all
2,797,045 allocations are freed. Its 75,163,303 cumulative bytes include checks
and setup, not donor-only allocation or peak/RSS. The upstream suite runs
natively, not under this focused Memcheck. The initial missing-GMP-header compile
failure and original harness source remain preserved; the corrected source adds
only the required include prelude and a trailing newline.

The [rational-function manifest](exactcore-hyper-comparison/audits/continuation/calcium/mpoly-rational-experiment.json)
binds 46 files and ten captured gates: nine successful and the preserved failed
compile. All 956 live sources and the five retained continuation changes remain
unchanged. Two executables total 60,840 bytes in `/tmp`, reusing the existing
libraries; paired numerical logs total 6,174,888 workspace bytes. No Rust/whole-
native rebuild, source copy, cleanup, deletion, donor patch, commit or push.
All 46 evidence verifiers pass at 2026-09-10T03:47:27.858Z with empty stderr;
their first 46 records/129,552 bytes exactly match the checkpoint45 output,
including its historical/current binding record. This does not turn preserved
failed compile/numerical gates into passing tests.

## Archived rational functions and expression bridges — checkpoint 47

The archived Calcium rational-function implementation/test/header slice is now
read completely: 44 files and 3,564 lines. With its previously read manual, this
closes the 45-file inventoried slice, not its recursive dependencies. Both
generations of rational-expression bridges, the current generic-ring adapter/test
and four expression helpers add ten complete files. Documentation/API excerpts
bring this checkpoint to 5,029 new lines, 54 complete and two partial additions.

The archive already uses staged denominator/content GCD cancellation; current
FLINT adds scalar-denominator specializations listed as archived TODOs. Hyperreal
already has denominator-GCD staging and restricted final cancellation. Hypersolve
already handles unit/shared denominators and bounded local-field residues.

The important contract distinction is explicit in the donor manual: expression
normalization treats terminal expressions as independent indeterminates. It does
not prove their algebraic independence or preserve authored denominator domains
after cancellation. A formal normal form is therefore not sufficient evidence
for an exact-real rewrite or selected-root equality. Hyper's fiber reduction
contract explicitly retains the original nonzero-denominator obligation. No new
production transfer or matched performance claim is justified by this slice.

Independent BigInt full-polynomial and known-linear-factor canonicality checks
pass for 7,308 complete values, including 2,142 generic-operation aliases. The
2,052 primary certificates cover 1,062 power rows, 360 numerator/denominator
extraction rows, 180 roundtrips, 90 authored expression identities, and input/
preservation rows. Contexts use 1/2/4 variables and all three monomial orders;
signed powers are bounded to -3,-1,0,1,2,3. Direct expression conversion and
expanded normalization agree with independent mathematical targets. No point-
sampling oracle, malformed input, resource-boundary or old failure probe is used.

Native and focused Memcheck outputs are identical, 864,384 bytes each. Memcheck
reports zero errors/suppressed/live blocks; all 603,105 allocations are freed,
11,974,118 cumulative bytes including setup/checks. This is current native64
qualification, not archived runtime, the general generic-ring suite, all GCDs,
real-value domain validity, donor-only cost or peak/RSS. Archived tests and the
current generic test were read, not newly run.

The [bridge manifest](exactcore-hyper-comparison/audits/continuation/calcium/mpoly-bridge-experiment.json)
binds 30 artifacts, five passing gates, 56 read records and the unchanged
956-file live snapshot. All 47 verifiers pass at 2026-09-10T04:56:34.285Z:
48 records/139,322 bytes, empty stderr, and a byte-identical 135,764-byte
checkpoint46 prefix. One 28,488-byte executable reuses existing libraries in
`/tmp`; paired numerical logs occupy 1,728,768 workspace bytes. About 19 GB
remains free in `/tmp`. No production/donor edit, new retained transfer, broad
rebuild, cleanup/deletion, commit or push; prior failures remain preserved.

## Expression representation and numerical interfaces — checkpoint 48

Both expression headers and manuals, and every file in the two `fexpr/`
directories except the LaTeX implementations, are now source-read. This adds
8,321 unique lines in 88 records: 87 new files and completion of one partially
read manual. Builtin headers/tables, LaTeX implementations and recursive support
remain open. This is a source-only checkpoint, not a new test campaign.

Flat tagged-word storage offers compact syntax and indexed argument access, but
copies complete children on construction. Borrowed views and changed-only
replacement temporaries do not provide persistent subexpression sharing or
cached numeric evaluation. Hyper's actual implementation retains immutable
`Arc<Node>` sharing and a synchronized single-finest approximation cache. Its
README's lock-free-cache wording is stale; the comparison uses the code.

Exact integer/rational/dyadic constructors are useful syntax mechanisms, not a
missing capability in Hyper. The numerical display helper retries within a finite
precision cap and returns evaluation success independently of whether requested
accuracy was reached; it then prints without a radius. That is not a certified
approximation query. This is a control-flow finding, not a newly demonstrated
inaccurate output. Formal normalization likewise cannot discharge selected-root
or authored-denominator obligations.

No new production change or matched performance claim is justified. Source tests
were read, not run; prior numerical and memory checks qualify their earlier
stated corpora only. The new source/evidence verifier passes, checking 88 donor
records, all 956 live sources and unchanged checkpoint 47 evidence bindings.
It does not rerun the full historical verification chain. No new `/tmp` files,
builds, cleanup or deletion were needed. See the
[detailed source findings](exactcore-hyper-comparison/audits/continuation/calcium/fexpr-representation-findings.md).

## Expression formatting, builtin symbols and streams — checkpoint 49

Both complete LaTeX implementations, builtin headers/tables/lookup/manuals and
calcium support directories are now source-read. The new 25 records add 13,667
unique lines: 24 new complete files and completion of the archived header's
partial record. The combined expression/builtin/support slice is 60 archived
and 62 current files, 22,843 lines. Recursive support and the wider ecosystem
remain incomplete. Auxiliary configuration excerpts are separately hashed and
not counted as newly read donor lines.

Static checks pass for all 474 paired symbol-table entries, their enum indexes
and lexical ordering, 405 documented names, and all 35 distinct formatting
callbacks used by 269 rows. This checks metadata consistency, not numerical
support, rendered output or memory safety. A symbol for an equality, limit,
derivative or special function does not establish an implemented exact operation.

Formatting primarily uses borrowed views and geometric string-buffer growth,
but also allocates child strings, substitutes endpoint previews and can invoke
formal normalization. The table couples name lookup to presentation callbacks.
Current source requests size optimization and hides some helpers; no actual
linkage-size or runtime improvement for Hyper is established.

The source raises escaping, literal-text output and symbol-ownership concerns.
These are defensive review observations, not newly executed failure probes.
No safe-serializer, clean-memory or fully productive formatting claim is made.
Hyper's typed/literal output and demand-bounded fixed decimal formatting remain
the relevant existing mechanisms; scientific-format partiality is not erased.
No new implementation candidate warrants a matched benchmark. The
[detailed findings](exactcore-hyper-comparison/audits/continuation/calcium/fexpr-formatting-findings.md)
record the distinctions and remaining scope.

The source/metadata verifier passes, rechecking checkpoint 48 and the unchanged
956-file live snapshot plus recorded checkpoint 47 bindings. No new donor runtime,
numerical test, Memcheck, benchmark, full 47-checkpoint chain rerun or `/tmp` build
was performed. All five retained continuation improvements remain intact.

## Algebraic representation and exact decisions — checkpoint 50

Both complete qqbar headers/manuals and selected representation, comparison,
refinement, construction and test files add 92 complete-file reads / 8,646 unique
lines. Algebraic arithmetic, relation discovery, generic adapters and recursive
root-finding support remain incomplete. The twenty upstream test files were
read, not newly executed.

The independent corpus constructs 49 small valid algebraic values with signed
square-root real/imaginary coordinates. Every ordered pair is checked before
and after explicit cache polishing; enclosures are requested at 32, 128 and
512 bits. BigInt endpoint comparisons use signs and exact squares, not qqbar
arithmetic, floating-point roots or overlap as an oracle.

Of 35,826 assertions, 35,562 pass and 264 fail. There are two contract issues:

- Root ordering gives nonzero results for equal nonreal values: 84 same-object
  and 84 distinct-copy observations, covering 42 values in two states. Distinct-
  value orders pass; no incorrect root list or sorting failure is demonstrated.
- Exact +1/-1 components paired with irrational components remain inexact in
  96 polished outputs, spanning sixteen values, three precisions and two states.
  The source's candidate reconstruction does not undo its integer-grid scaling.
  All 588 component containment checks and 588 `prec-2` accuracy checks pass:
  this is an exact-component contract failure, not an incorrect enclosure claim.

All 28,812 ordinary pairwise equality/component/magnitude comparisons, unary
signs, copy equality/hash consistency and readonly input-preservation checks
pass. The mathematical gate exits one and remains preserved. Native and Memcheck
output match exactly; focused memory instrumentation reports zero errors and
zero live blocks. The original bookkeeping failure comparing JavaScript values
to JSON was corrected only at the serialization boundary; no oracle or result
was changed. Detailed analysis and limits are in the
[algebraic decision findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-decisions-findings.md).

Hyper already has bounded algebraic separation metadata, cheap-fact-first sign
dispatch, and represented-root comparison through strict local evidence checks,
Sturm refinement, common-root proofs and exact differences. Donor containment
shortcuts rely on canonical minimal-polynomial/isolation invariants that cannot
be assumed for arbitrary Hyper expressions. No new Hyper candidate or matched
performance/size claim is justified. All five retained transfers stay unchanged.

## Algebraic arithmetic and relation boundaries — checkpoint 51

54 selected full-file reads add 7,223 unique lines across both pinned arithmetic,
polynomial-evaluation, relation-discovery and test implementations. The 21
provisional files from the report handoff are counted once. The source separates
exact affine/deflation/surd shortcuts from general factor-and-root selection.
Numerical relation guesses require exact identity certificates; field-expression
discovery checks modular polynomial composition and the selected embedding.

The independent BigInt oracle operates in `Q(sqrt(2),sqrt(3))`, using exact
four-component rational arithmetic and sign-flip conjugates. It derives full
minimal polynomials and full Cartesian-root composed annihilators, then checks
root enclosures with directed integer-square-root bounds. It does not use qqbar
arithmetic, floating-point roots or interval overlap as a mathematical oracle.

All 10,945 records and 40,352 assertions pass: 7,344 arithmetic values each have
the expected minimal polynomial, root containment, ordered endpoints, checked
absolute width and exact real axis; 1,008 full composed polynomials and 2,560
relation decisions also pass, together with readonly-input preservation. The
corpus includes initial/explicitly cached states, aliases, cancellation, signed
powers, affine maps and polynomial evaluation. There are 44 true relation
controls and 2,516 false ones. This is not branch coverage, arbitrary algebraic
closure, direct LLL qualification, archived execution or a Hyper benchmark.

Native/Memcheck outputs are identical. Focused memory instrumentation reports
zero errors and zero live blocks. Complete evidence and limits are in the
[arithmetic findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-arithmetic-findings.md).

The complete live Hypersolve binary-construction and integer-interpolation
modules, plus relevant resultant ranges, confirm a worthwhile candidate to
investigate: power-sum/Newton construction instead of repeated exact determinant
samples. Existing code already has integer Bareiss and factorial-scaled
interpolation, so the comparison must use that optimized baseline. Preserve
reducible/repeated carriers, the divisor carrier's possible unused zero roots,
the degree cap, policy/Unknown behavior and root-validation boundaries. At
checkpoint 51 no Hyper prototype or matched cost campaign had run. The isolated
power-sum prototype followed in 52; it remains unretained and not performance-
qualified. Its public corpus motivated the separate completeness repair in 53.

## Point-image completeness — checkpoints 52–59

The retained binary-image helper discarded exact witnesses, including when an
image collapsed to a point. Refinement correctly refused those witness-free
singletons. This was one pre-existing completeness gap across 825 corpus cases,
not 825 independent defects or a demonstrated wrong returned value.

The guarded candidate preserves only certified point images and keeps all
containment, polynomial-vanishing and uniqueness replay. If approximate ordering
was allowed for multiplication/division, it first replays the whole point-image
construction under STRICT. Its public oracle recovers all 825 results and leaves
5,616 other records unchanged. Eight regressions cover rational/nonrational
points, zero collapse, Unknown/approximate controls, half-open ownership and
polynomial replay. That eager version has 20 net algorithm lines and 303 test lines.

Seventy same-result benchmark groups show unchanged allocation/requested-byte,
live and peak deltas, with mixed timing ratios. Ten groups gain certified answers
and therefore are not equal-work speed comparisons. The retained rational-sum
case has marginal medians 2.411 versus 2.707 microseconds while saving one
allocation/57 requested bytes per query. Whole-collector memory retains 728 more
reachable bytes; no error or definite/indirect/possible loss is reported. This is
not zero-live or peak-RSS qualification. At that stage, nonrational/Unknown endpoint
costs and broader cache histories were open; neither this repair nor the separate
power-sum optimization has been moved into production.

Checkpoint 54 qualifies the guarded release consumer, all-feature Clippy and
a selected-feature WASM consumer build. Full rational public values agree on
native and WASM under both STRICT and approximate policy, including the guard's
approximate-policy replay. This is not a full WASM consumer test-suite run or
nonrational/Unknown endpoint qualification. The earlier debug consumer result
still belongs to the unstrengthened v2 source.

Representative stripped curve examples grow by 1,136 and 1,152 bytes. The
unchanged scalar example shrinks by 784 bytes, demonstrating build-path/layout
effects; these are not isolated algorithm-only size differences. The active
arithmetic collectors grow by 1,152 native / 560 WASM bytes under STRICT and
1,080 native / 547 WASM bytes under approximate policy. No whole-application
timing, generalized size gain or new CPU/allocator campaign is claimed.

Checkpoint 55 extends exact-value qualification to radical and internally
Unknown trigonometric endpoints across four construction/refinement/serialization
histories. The independent model uses BigInt rationals in Q(sqrt(2),sqrt(3)),
nonzero pi/e divisors and a checked common-argument sine/cosine identity. All
carrier coefficients are checked after monic normalization, preserving
multiplicities but allowing a nonzero rational scale. It verifies 316 candidate
results and 252 exact witnesses, versus 188 results and 124 witnesses before
the repair. All 128 changed queries address the same point-witness gap; repeated
policies/histories and overlapping controls must not be counted as distinct bugs.

Deep refinement enables four tiny-offset case/policy combinations in both
variants. Other histories remain Undecided or nonisolating; this is a bounded
evidence-history effect, not a changed value or a new candidate defect. Thirty-two
input-evidence failures and 24 denominator guards remain unchanged. The focused
memory comparison retains 47,632 baseline versus 48,648 candidate bytes, with no
errors/lost blocks. These whole-collector counts include serialization and extra
certified answers and do not qualify marginal solver allocation costs.

Checkpoints 56–57 supply the matched native history-aware cost campaigns. Eager
recovery's avoidable nonpoint costs led to a demand-gated revision: try witness
recovery only after the unchanged refiner returns typed InvalidInterval, then
replay all original proof obligations. This preserves every qualified eager
result and restores baseline allocation/live/peak counts on all 512 unchanged-
result groups, but slower timing controls and recovered-point retry costs remain.
It is a completeness candidate with measured tradeoffs, not a universal speedup.

Checkpoint 58 qualifies the first query in fresh processes/instances and all
nine calls across retained, fresh and round-trip lifecycles. Demand/eager and
native/WASM results agree exactly across the full 62,208-query campaign. The
existing loss of bounded decision availability after serialization affects all
variants equally; exact input values persist. This closes the cold-state and
extended WASM semantic gates only. Checkpoint 59 supplies matched optimized-tier
WASM costs: an approximately 23% improvement versus eager on a constructed
Unknown control, disclosed retry costs, and volatile paired-versus-marginal
comparisons requiring a focused timing replay before algorithm-level attribution.
At checkpoint 59, final demand-version consumers/representative sizes and
retention remained open, with neither candidate transferred into live crates.
Checkpoints 60–65 subsequently correct statistics and complete the focused
diagnostic; 66 qualifies the selected consumer/sizes and 67 retains the demand
repair. The eager version stays unselected, and power sums remain separate.

## Scalar-domain findings: no production change needed

Reading the current generic CA wrapper exposes typed-domain and metadata defects.
A 580-row valid-input corpus records 12 successful non-real `asin`/`acos` outputs
in the real context and two incorrect real-vector-space property flags for
algebraic contexts. Another 18 successful algebraic-context argument outputs have
Unknown algebraicity predicates; a separate exact witness proves they equal
−π and therefore are transcendental. The 36 negative-rational principal-argument
failures reproduce the previously recorded −π versus documented +π defect;
they are not 36 new independent bugs. Original failures and the supplemental
value witness are both preserved. Failed-operation outputs are never inspected.

Hyper's public inverse-trig guards pass 102 domain/boundary/state controls in each
profile and in the focused release Memcheck. No errors or lost blocks are reported;
3,112 bytes remain reachable. This checks acceptance/rejection and refinability,
not independent numerical accuracy or arbitrary state histories. No new production
change is justified, and these memory results do not erase earlier failure reports.

## Validation and remaining work

The recorded 47-checkpoint historical verification passed, including archived rational-
function/bridge source reads and independent current expression qualification,
multivariate rational-function source/independent/upstream checks, finite conversion and
integer qualification with its preserved harness failure, the corrected ARF
manifest, explicitly separated historical source checks and the 956-file retained
live scalar/consumer snapshot at that checkpoint. Hyperreal passed 859 library/
integration tests per profile in its retained campaign. Checkpoint 67 passed
811 tests per live Hypersolve profile. Checkpoint 75 now passes 817 default and
818 per all-feature profile, including seven new regressions; checkpoint 74's
frozen-candidate Hypercurve qualification passes 1,764 tests with nine pre-existing
ignored tests unrun. The current verifier explicitly distinguishes the changed
957-file live map from historical qualification maps.
Exact/MPFR oracles, unequal controls, cancellation/retry, serialization,
concurrency, paired CPU campaigns, separate allocation measurements, Memcheck,
Clippy, formatting and representative binary sizes are recorded per candidate.
Older WASM gates are compile-only; checkpoint 43 also executes the scalar audit
module in Node/V8. This is not full CI, peak-RSS qualification, or a
universal speed/memory guarantee.

Checkpoints 48–49 separately pass source/evidence and static metadata checks for
the expression representation, numerical interface, formatting, builtin and stream
reads. They add no numerical test, benchmark, Memcheck or full-chain rerun, and
retain no new production change.

Checkpoint 50 adds independent bounded algebraic comparisons and enclosure
checks with two recorded mathematical contract failures. Its source/evidence
verifier passes by verifying those failures, not by treating them as successful
mathematical qualification. It rechecks 48–49 and the unchanged retained live
snapshot, without a full historical-chain rerun or new Hyper regression campaign.

Checkpoint 51 completes its selected arithmetic/relation source slice and
independent donor polynomial/root certificates. Checkpoint 52's isolated
power-sum prototype passes 4,840 full-polynomial cases, but public qualification
exposes the shared lost-witness gap. Checkpoint 53 qualifies a separate guarded
repair with independent root certificates, solver regressions and bounded paired
costs. Checkpoint 54 adds final guarded release-consumer qualification, native/WASM
full-value execution under two policies and representative size observations.
Checkpoint 55 adds exact nonrational/Unknown value and state-history qualification
with a strengthened independent oracle and focused memory checks. Checkpoints
56–65 add the demand-gated revision, native/WASM endpoint/history qualification,
corrected statistics and focused diagnostics; 66 completes consumer/size gates,
and 67 retains the exact candidate with live solver regressions. Retry/size costs
are accepted for completeness, not represented as a universal improvement.
Checkpoint 68 rebases power sums onto that retained witness baseline and passes
the existing independent corpora, regression and focused memory gates in isolation.
Checkpoint 69 extends coefficient/carrier qualification and certifies a separate
divisor-zero-factor opportunity. Checkpoint 70 implements that isolated trial and
passes independent full-polynomial/interval, regression and focused memory gates.
Its final verification passes at 2026-09-11T19:13:45.345Z: 145 bound artifacts /
thirty gates (27 successful, three preserved failed), one record / 692 bytes,
empty stderr, code 0 / null signal. Six continuation transfers remain retained.
Checkpoint 71 completes the isolated native matched CPU/allocation campaign and
its statistical/measurement checks. Final verification passes at
2026-09-11T19:54:15.448Z: 541 artifacts / 138 successful captured gates, one
record / 767 bytes, empty stderr, code 0 / null signal. An uncaptured sandboxed
recording attempt hit nm EPERM before creating a manifest; the unchanged
approved rerun succeeded. WASM execution and consumer/size gates were still open
at checkpoint 71; donor qualification does not substitute for Hyper gates.
[Checkpoint 70 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-verify-v70.json).
[Checkpoint 71 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-cost-verify-v71.json).

Checkpoint 72 completes WASM correctness qualification; final verification passes
2026-09-11T21:08:01.072Z: 118 bound artifacts / 23 successful captured gates,
one record / 707 bytes, empty stderr, code 0 / null signal. Checkpoint 73 completes
the matched WASM cost campaign. Its final independent statistical/source replay
passes 2026-09-11T21:25:57.740Z: 430 artifacts / 111 successful captured gates,
one record / 604 bytes, empty stderr, code 0 / null signal. All commands terminal.
Six retained transfers remain unchanged at checkpoint 73; the consumer/size
qualification and candidate-selection decision follow in checkpoint 74.
[Checkpoint 72 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-wasm-verify-v72.json).
[Checkpoint 73 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-wasm-cost-verify-v73.json).

Checkpoint 74 completes those isolated consumer/size gates and selects the exact
candidate for live integration. Its final verification passes at
2026-09-11T21:51:02.880Z: 84 artifacts / 19 successful captured gates, one
record / 881 bytes, empty stderr, code 0 / null signal. All commands terminal.
At that checkpoint, live integration/regression gates still preceded recording
a seventh retained transfer; all six earlier changes and donor coverage were unchanged.
[Checkpoint 74 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-consumer-verify-v74.json).

Checkpoint 75 completes exact live integration and the fresh regression, lint,
platform and dependency gates, retaining the seventh continuation transfer.
Final verification passes 2026-09-11T22:10:23.914Z: 1,331 artifacts / fourteen
gates (twelve successful, two preserved sandbox failures), one record / 1,275 bytes,
empty stderr, code 0 / null signal. All commands are terminal. Historical
numerical/consumer/cost evidence is inherited by exact source identity, not
presented as new live-path runs. Source coverage and remaining audit scope do
not change. [Checkpoint 75 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-retained-verify-v75.json).

The first ARF checkpoint verification caught a read-range bookkeeping error:
a Hyperlattice file was recorded through 255 but ends at 250. The corrected
manifest passes; the original manifest/verifier and failed run remain preserved.
No source, numerical evidence or donor coverage changed in that correction.

Checkpoint 76 completes the selected forward rational-angle source slice,
independent current-FLINT mathematical/memory qualification and the unchanged
Hyper capability comparison. Final verification passes 2026-09-11T22:38:30.116Z:
1,429 artifacts / twenty gates (sixteen successful, four preserved harness failures),
one record / 1,082 bytes, empty stderr, code 0 / null signal. All commands terminal.
There is no new retained transfer or performance claim. The new twelfth-turn
relation candidate still needs design, implementation and full qualification.
[Checkpoint 76 verification](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-trig-verify-v76.json).

Unfinished work includes further Calcium/FLINT supporting kernels, generic matrix
and algebraic/LLL support; substantial formal and symbolic references (including coq-aern,
Clerical, ternary reals, TypeTopology, LEDA/CGAL, Sage, LibPoly and Z3); historical
sources/surveys; and unresolved transfers from earlier source-complete targets.
The complete requested inventory must still be reconciled before a final audit
completion statement is justified.

Progress, next actions and evidence links are maintained in
[the workspace-root continuation ledger](EXACT_REAL_ECOSYSTEM_AUDIT_CONTINUATION.md).
Historical evidence is preserved, including the earlier 78 MB crash dump.
The derivative checkpoint's 148.7 MB of dedicated executable snapshots remain
preserved. The first filter experiment added six dedicated `/tmp` snapshots
totaling 12,712,664 bytes. The second added three totaling 6,349,368 bytes,
reusing the baseline binaries and shared Rust build cache; dedicated snapshot
size does not measure build-cache growth. The latest high-product qualification
adds one 22,616-byte executable. The ARF rounding pass adds one 22,856-byte
executable and about 2.3 MB of paired numerical logs, reusing the existing
native library. The fused/complex pass adds a 32,008-byte executable and about
4.4 MB of paired numerical logs. No new whole-tree native or Rust build was needed.
The first isolated complex-product experiment adds eight preserved executables
totaling 16,697,200 bytes. V2 adds four totaling 8,367,856 bytes and reuses the
four baseline executables. Both use the shared Rust build cache; source copies
are in the workspace. Arb dot adds one 27,768-byte audit executable and 322,942
bytes of paired numerical logs, reusing the same native libraries. Magnitude
qualification adds a 47,216-byte executable and 395,834 bytes of paired numerical
logs. The transcendental pass adds a 33,256-byte executable and 1,732,510 bytes
of paired numerical logs, again reusing existing native libraries.
The combinatorial/tail/conversion pass adds a 38,448-byte executable and
2,617,160 bytes of paired numerical logs using the same libraries. The isolated
`e` experiment copies only the 180-file Hyperreal subset (5,312,930 original
bytes) in the workspace and freezes six executables totaling 11,204,552 bytes
under `/tmp/calcium-e-plan.mOIsBs`, reusing the shared offline Rust build cache.
Qualification adds one 45,446,168-byte source copy in the workspace and 14 dedicated
native/WASM executable snapshots totaling 108,188,255 bytes in `/tmp`, reusing the
same build cache; shared-cache growth is separate. The versioned integrity path
adds only 82,863 bytes of derived scripts in the workspace, with original scripts
preserved. ARF contract qualification adds only a 28,392-byte executable in
`/tmp/calcium-arf-contracts.PNgYf4` and 36,592,572 bytes of paired numerical logs
in the workspace, reusing the native libraries without a Rust/whole-native
rebuild or source copy. Finite conversion qualification adds two preserved
executables totaling 57,336 bytes and 8,340,310 workspace bytes of numerical logs.
The rational-function pass adds two executables totaling 60,840 bytes and
6,174,888 workspace bytes of paired numerical logs, again reusing libraries.
About 17 GB remained available on `/tmp` at checkpoint 53, not a quota guarantee.
The algebraic decision pass adds one 18,152-byte executable and 1,316,178 workspace
bytes of paired numerical logs, reusing the existing native libraries.
The arithmetic pass adds one 22,960-byte executable and 4,064,396 workspace bytes
of paired numerical logs, again reusing those libraries without a broad rebuild.
Power-sum qualification adds five dedicated executables totaling 34,189,432 bytes;
the separate point-image work adds six totaling 56,943,680 bytes. Isolated source
copies are in the workspace, and the existing shared Cargo cache is reused.
Guarded consumer/platform qualification adds one 45,463,105-byte source copy in
the workspace and 14 native/WASM snapshots totaling 68,928,629 bytes in `/tmp`,
reusing all six baseline application files and the same shared cache. At its
recorded capacity snapshot, 16,642,371,584 bytes remained available on `/tmp`.
These dedicated-byte totals exclude cache growth and are not application-size
measurements. All earlier artifacts remain preserved.
The extended endpoint/history corpus adds two executables totaling 6,379,848
bytes and 4,791,668 workspace bytes of paired native/Memcheck output, without
another source copy. The shared cache was reused; about 16 GB remained free on
/tmp at that checkpoint.
The three scoped Hyperreal paths changed at checkpoint 43 remain intact;
checkpoints 44 through 55 add no production change. There is no donor change,
cleanup, deletion, commit or push. Rank v1, both sign-filter prototypes and both
complex-product versions remain isolated and unselected. Verification
preserves failures as well as successes; it does not mean every numerical or
memory gate passed.

Selected-demand qualification in checkpoint 66 adds only the 355-file consumer
copy in the workspace and four binaries totaling 50,279,984 bytes in /tmp,
reusing frozen dependencies and the shared cache. Checkpoint 67 needs no new
dedicated artifact or source copy for live adoption. Available /tmp space after
its live gates is 15,381,413,888 bytes; cache growth is separate from dedicated
artifact totals. The retained point-witness repair changes only Hypersolve's
algebraic_binary.rs. All earlier evidence is preserved; nothing was deleted,
committed or pushed in these checkpoints.
