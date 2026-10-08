# Exact-real ecosystem audit — active continuation

Started 2026-09-08. The objective remains completion of the **entire** supplied
reference inventory and disposition of every plausible transfer, prioritizing
exactness, completeness, performance, memory, binary size, and source size.

## Evidence and scope

- Historical ledger: [EXACT_REAL_ECOSYSTEM_AUDIT_PROGRESS.md](EXACT_REAL_ECOSYSTEM_AUDIT_PROGRESS.md).
- That ledger is now a compatibility link into the byte-verified historical
  archive. Preserve its recorded bytes; new work is tracked here and in
  `exactcore-hyper-comparison/audits/continuation/`.
- The previous report turn made progress by identifying that the completion
  claim was unsupported: numerous reference rows and experiments remain pending.
  No full-inventory completion is claimed. No subagents are being used.
- All still-pending historical targets and experiments remain in scope. This
  continuation supersedes a historical status only when explicitly documented.

## Current work: coq-aern; Calcium / FLINT remains open

- Coq-aern is pinned at `bc11353f450cf866b47c3985eee6150a5f99cf00`.
  Through checkpoint 86, read coverage is 30 complete files / 9,120 lines, no
  partial files. Native Coq/GHC qualification remains open. Two complex-root
  lemmas are explicitly admitted; the corresponding mathematical models do not
  fill their proofs. Hyper's focused sqrt regression passes 23 tests in each
  debug/release profile. No new production change or matched benchmark result.
- Latest Calcium/FLINT coverage is checkpoint 84: 1,576 complete files,
  19 partial and 200,953 unique lines. Both qqbar directories are fully read;
  wider source/support and transfer experiments remain open. The checkpoint 76
  coverage paragraph below is a preserved historical snapshot, not current totals.

### Calcium / FLINT context

- Standalone Calcium pinned to `8dbb16fc4fe92eaf3ebbc7478d629e994d39f944`
  (2023-11-15) at `exact-real-references/calcium`.
- Current FLINT pinned to `e269d38061d7a42070ddcffe6eb114466ed4aa7e`
  (2026-09-07) at `exact-real-references/flint`.
- Official project README confirms Calcium moved into FLINT in 2023; both the
  historical implementation and the current exact-number slice need coverage.
- Status: **in progress**. Scoped numerical qualification is recorded; neither
  whole-library qualification nor whole-inventory source completion is claimed.
- Historical read coverage (checkpoint 76): 568 standalone files / 61,301 lines
  (568 complete, none partial); 899 current-FLINT files / 124,316 lines
  (879 complete, 20 partial),
  with exact ranges and verified snapshot hashes. Both `ca_ext/` and `ca_field/`
  directories and all top-level `ca/*.c` files (114 archived, 108 current) are
  now source-read complete in both pins. Scalar tests and selected vector paths
  are also complete. Both complete ca_poly directories, including their tests,
  are now read (74 archived/70 current files). Both ca_mat directories are read
  (107 archived/109 current), as are all 26 tracked current nfloat files, all
  104 inventoried mag files and the full 104-file ARF implementation/test/header/
  manual slice. All36 current and45 archived multivariate rational-function
  implementation/test/header/manual files are now read, along with both rational-
  expression bridges, current generic adapter/test and four expression helpers.
  Both complete expression/builtin directories, headers and manuals are read,
  along with both calcium support directories and headers.
  Both qqbar headers/manuals and 87 implementation/test files per pin are now
  read; the remaining algebraic implementation/test and recursive support files
  are not covered by this selected slice.
  Polynomial/test-driver/generic headers remain partial, along with earlier
  random-helper excerpts.
  Supporting kernels, algebraic paths and whole libraries remain open.
  Full inventory: 684 standalone files; initial current exact-number slice 704
  files. Inventory alone is not credited as reading.
- Confirmed completeness candidate: `2*ln(a+sqrt(d)) = ln(a*a+d+2*a*sqrt(d))`.
  Thirty distinct positive formulas at three budgets produce 90 Unknown results
  in both frozen Hyper debug/release builds, while FLINT proves all 90. All 90
  algebraic controls and 90 unequal perturbations decide correctly in both;
  FLINT also rejects all 30 complex-branch counterexamples. No incorrect Hyper
  result was observed; no production transfer yet.
- Native FLINT builds with assertions; 29 scalar and 73 field/algebraic test
  functions pass. Both focused native/Hyper Memcheck runs have zero errors and
  no definite/indirect/possible leaks. These probes are not matched benchmarks.
- All 176 current Hyperreal source/support files were frozen and hash-checked
  for reproducible isolated work. Source HEAD is
  `da26e961bf76adf13829d8a26ced6cdaacb6b564` plus the pre-existing worktree delta.
- Hyperreal already has uncommitted scalar/cache/formatting changes. They are
  pre-existing, must be preserved, and must not be attributed to this continuation.
  Hypersolve was clean at entry. Experiments will be isolated.
- New qualification code and detailed coverage belong directly in
  `exactcore-hyper-comparison/audits/continuation/calcium/`.
- Detailed findings, commands, failed harness attempts, verified results and
  limitations: [Calcium checkpoint](exactcore-hyper-comparison/audits/continuation/calcium/README.md).
- Current production status: the cache-preserving exponential proof is retained
  in Hyperreal; fact-first nonzero-dominates-Unknown polynomial decisions and
  certified exact-one monic normalization are retained in Hypersolve, and
  demand-bounded polynomial derivative evaluation is retained in Hypercurve.
  Lower-factorial e term planning is the fifth retained continuation transfer,
  in Hyperreal. Demand-gated exact point-witness repair in Hypersolve is the
  sixth, retained at checkpoint 67. Certified divisor zero-factor removal is
  the seventh, retained at checkpoint 75. Its main-file/test-module promotion
  preserves the other 955 recorded source/support identities. Both sign-filter prototypes, both
  complex-product prototypes and the eager point repair remain unselected.
  Current verification, from
  `exactcore-hyper-comparison/audits/continuation/calcium/`, is
  `node verify-zero-factor-retained-v75.mjs`: 1,331 bound artifacts, fourteen
  gates (twelve successful, two preserved sandbox failures), the 957-file live
  map and complete candidate/consumer bindings. Fresh live solver tests pass
  817 default and 818 per all-feature profile; the frozen consumer passes 1,764 with nine
  ignored. Earlier independent mathematical/native/WASM/allocation/corrected
  timing campaigns qualify the identical candidate, not newly rerun live paths.
  The retained demand repair recovers 825 answers per policy while preserving
  5,616 other full records and strict proof obligations. It avoids eager's
  unchanged-result allocation/peak penalties but accepts retry and representative
  size costs for completeness. Power sums remain separate, unselected and untimed.
  Historical failures and superseded statistical intervals remain documented.
  The seventh transfer adds 184 qualified answers, preserves 16,508 other full
  reports and improves all measured same-result bounded-deflation groups on
  native/WASM, with disclosed control/new-answer costs. Representative stripped
  binaries grow 5,744 bytes each. No universal speed/memory/size claim.
  The recorded 47-chain is historical and was not rerun. Older live flags assert
  their original source maps; they are not valid current-state entry points.
- Historical isolated work: checkpoint 68 rebases the deferred power-sum constructor
  onto the then-retained witness baseline, without a live edit. The recorded
  `verify-power-rebased-v68.mjs` result is historical after checkpoint 75. The
  corrected candidate passes 814 tests per profile and the existing polynomial,
  public-policy and nonrational/state corpora; memory and WASM compile gates pass.
  Broader coefficient/carrier coverage, matched costs, consumer/size qualification
  and the retention decision remain open. The original five-borrow lint failure
  and initial candidate are preserved rather than relabelled as successful.
- Checkpoint 69 qualifies the wider point-carrier corpus without changing either
  solver: all 23 admitted ordered degree pairs, input numerators through 2,577
  bits and resultant coefficients through 4,639 bits. Full outputs agree. An
  independent zero-factor probe supports a separate completeness trial on the
  known division gap, which then took priority over timing. Historical isolated
  entry point at that time: `node verify-power-wide-v69.mjs`; initial failed metadata captures
  are distinct from its corrected final capture.
- The previous report turn verified a material benchmark regression, changing
  the next action: **PROGRESS**, not a completion claim. The interrupted follow-up
  preserved an audit-owned temporary build directory; its failed edits did not
  land. After the user freed temporary space, this checkpoint resumed normally.

## 2026-09-08 qualification and source continuation

- Isolated log prototype v1 proved all 90 identity queries but repeated unresolved
  queries regressed about 35x. Its exact source diff and raw evidence are preserved.
- V2 uses the sign of positive algebraic products to certify the sign of a rational
  log combination, instead of discarding a nonzero product result. All 270 public
  corpus rows pass in debug/release; all 773 library tests pass in each profile,
  including nine focused tests and an independent 2,000-case rational-product oracle.
- Expanded qualification records 1,056 CPU and 264 allocation observations across
  11 cases and fresh/shared lifecycles. Tiny rational and selected algebraic log
  differences gain decisions, but genuinely unresolved multiradical queries still
  regress about 171x warm (324 allocations / 32,008 requested bytes versus one /
  768); unresolved transcendental log queries regress about 5x warm. The matched
  linked CPU driver grows 8,432 file bytes. These are scoped observations, not a
  whole-library speed or memory claim.
- **No log production transfer is retained.** Rebuilding the entire proof on each
  unresolved query is not selected as implemented. The proof idea remains open;
  scheduling/reuse needs a bounded, history-safe design and its own measurements.
- Standalone v2 public Memcheck has zero errors and no definite/indirect/possible
  leaks. The nine-test runner reports a 48-byte possible leak in Rust's test-channel
  thread initialization; the unchanged baseline runner reproduces the same stack
  and exit status. Both raw failures remain recorded; no suppression was added.
- Current FLINT generic-qqbar and monomial contexts have inline state and no-op
  teardown. Their skipped cleanup calls are **not leaks in this pinned backend**;
  this closes the earlier lifetime questions, not the full backend audit.
- New independent error-function corpus: `erf(x)+erfc(x)=1`, erf oddness, and erfc
  reflection, at seven arguments and three budgets. Frozen Hyper debug/release
  each give 9 Equal / 54 Unknown identities and 63 correct NotEqual perturbations.
  FLINT proves all 63 identities and rejects all 63 perturbations; its focused
  Memcheck is clean. This is a separate completeness candidate, not a code transfer.
- `verify-checkpoint.mjs` verifies full corpus membership, outcome counts, source
  directory coverage, isolated trial footprint, test gates and benchmark groups.
  Original 176-file Hyper snapshot hashes and donor pins still verify. No live
  Hyper production file or historical archive ledger was changed by this checkpoint.
- Storage: the audit-owned 144 MB `/tmp/calcium-prototype-target` was moved, without
  deletion, to `.audit-builds/calcium-prototype-tmp-preserved`. Subsequent builds
  reused the existing two targets with incremental compilation disabled; native
  probes are small. The user's separate temporary cleanup restored normal edits.
- Next: read the remaining scalar construction/reduction paths, compare the new
  error-function identities with Hyper's current complement representation, and resolve
  log-proof scheduling/reuse before any retention decision. Continue all other
  pending ecosystem targets and experiments; none is closed by this checkpoint.

## 2026-09-08 scalar / error-function checkpoint

- Added complete independent reads of 13 more scalar files in each Calcium
  snapshot, plus two precisely bounded supporting-header reads. These cover
  equality/realness, erf/erfc/erfi/Gamma, complex normal form, exp/log, field
  merging, storage transition and demotion. Snapshot/range verification passes;
  whole Calcium/FLINT and the wider ecosystem are still incomplete.
- Confirmed a current-FLINT Gamma correctness defect: special-value predicates
  return `T_TRUE=0`, `T_FALSE=1`, `T_UNKNOWN=2`, but Gamma uses them as C booleans.
  Seven special-value controls fail, including `Gamma(+infinity)` becoming
  undefined and `Gamma(Unknown)` becoming positive infinity; three finite
  closed-form controls pass. The archived source has the same defect, but only
  the current native build was executed. Memcheck reports zero errors/all blocks
  freed; the probe exits 1 for mathematical failures. No donor patch was applied.
- Corrected an earlier description: Hyper's `Erfc` caches a complement and
  rebuilds `1 - erf_expanded(x)`; it is not a separate stable-tail kernel.
- An isolated erf prototype retains complement/reflection structure and adds
  symmetric bounded `x + (c - x)` cancellation. It proves all 63 sampled identity
  queries and rejects all 63 unequal controls, in debug/release and a clean
  focused Memcheck run. Its first incomplete constructor-only attempt is also
  preserved. These are 21 formulas at three budgets, not 63 distinct formulas.
- Both baseline and trial pass 5,856 directed-MPFR enclosure checks per profile
  over 160 inputs, including tiny inputs and deep tails through precision -1600,
  fresh/mixed/serialized states; 270 additional enclosure checks cover aborted
  work, concurrent refinement and opaque-zero inputs. Nearby distinct-argument
  controls reject false cancellation. All 764 existing all-feature library tests
  pass for the trial in debug and release. An initial oracle sign-import bug was
  corrected; both failed harness logs remain recorded, not blamed on Hyper.
- **The erf prototype is not selected as implemented.** Four reflected `erfc`
  expressions lose their structural positive signs after serialization; the
  baseline preserves all four. Paired measurements (2,304 CPU and 576 allocation
  observations, 12 cases/four lifecycles) also show roughly 1.5x ordinary erf
  refinement and 2.5x tiny-input refinement regressions. Construction can improve,
  but extra reconstruction loses useful intermediate caches. The linked CPU
  driver grows 2,064 file bytes; the source delta is 50 lines / 2,286 bytes.
- No live Hyper production source changed. The isolated prototype, source hashes,
  raw successes/failures and benchmark summaries remain under the Calcium
  continuation directory. `verify-erf-checkpoint.mjs` passes and also rechecks
  the earlier log checkpoint and original 176-file frozen Hyper snapshot.
- Next: retain symbolic special-function relations without losing range facts or
  expanded-kernel cache reuse; qualify general nested cancellation independently.
  Continue log-proof scheduling/reuse, remaining scalar paths and every other
  pending reference. A rejected combined prototype does not close these ideas.
- Storage: reused the existing two target directories with incremental builds
  disabled; no unrelated deletion or new large temporary checkout. At checkpoint,
  `/tmp` still has about 16 GB free and the workspace filesystem about 4.6 GB.

## 2026-09-08 arithmetic / root-exponential / trig checkpoint

- Added complete independent reads of 40 more source/test files across the two
  pinned snapshots, plus precisely bounded scalar documentation reads. Coverage
  now totals 138 complete files and five partial files; neither donor library
  nor the full reference inventory is complete. Both pins and all tracked source
  hashes still verify. Re-reading an already covered file receives no new credit.
- Arithmetic reads cover add/subtract/multiply/divide/invert, root factoring,
  powers, generic factorization, approximation, and trig representations.
  Useful distinctions: field-ideal reduction can allocate heavily; expression-size
  heuristics choose evaluation strategy, not truth; root branch selection first
  establishes an exact two-candidate identity; requested enclosure precision may
  not be achieved at the donor's precision cap. Hyper already has stronger fused
  rational/dyadic aggregate paths than Calcium's sequential dot implementation.
- New completeness corpus: `sqrt(exp(x)) = exp(x/2)` at eight arguments, both
  signs and three budgets. Frozen Hyper debug/release each give 12 Equal and
  36 Unknown identity queries; FLINT proves all 48. All 48 unequal perturbations
  are rejected by both. These are 16 signed formulas at three budgets, not
  48 distinct formulas; donor and Hyper budget semantics are not matched costs.
- An isolated 25-line / 1,394-byte root constructor rewrite closes all 36 missing
  queries, handling both direct and binary-range-reduced exponential nodes.
  All 764 existing all-feature tests pass in debug and release. Each baseline
  and trial profile passes 15,050 directed-MPFR enclosure checks over 43 dyadic
  inputs, optional sine composition, seven binary scales, five cache/serde
  lifecycles and five precisions; 261 more checks cover cancellation/retry,
  concurrent refinement and an opaque zero exponent. Structural positivity
  survives the tested serialization paths. Focused native and trial Memcheck
  runs report zero errors and no definite/indirect/possible losses.
- **No root/exponential production transfer retained as implemented.** Paired
  benchmarks contain 3,456 CPU and 864 allocation observations across 12 cases
  and six lifecycles. Fresh evaluation improves (trial/baseline about 0.365 for
  exponent 1/3 and 0.493 for sin(1)), but repeated fresh roots of an already-hot
  exponential regress about 5.14x and 6.80x respectively; the large-exponent case
  regresses 7.61x. The latter requests 7,224 allocation bytes versus 1,088.
  Rewriting discards the original exponential result cache. The linked CPU
  driver grows 1,056 file bytes. The identity and fresh-evaluation opportunities
  remain open for a design that preserves existing caches, not declared useless.
- Two additional current-FLINT correctness findings are independently reproduced:
  tangent/cotangent special handling reads the old destination's Unknown tag
  instead of the input (25 failures among 175 destination/special controls);
  in-place `atan_logarithm(2i)` selects the wrong principal branch (one failure
  among six direct/separate/in-place ±2i controls). The archived code has the
  same mechanisms, but only current FLINT was executed. Memcheck reports zero
  errors/all blocks freed; the native probe exits 1 for 26 mathematical failures.
  No donor patch or external issue was submitted. Hyper's owned finite-real
  scalar API does not share these output-alias/complex-branch mechanisms.
- All source changes remain in the isolated audit copy. Pre-existing live
  Hyperreal edits and the historical archive are preserved. Build targets were
  reused with incremental compilation disabled; no large temporary checkout or
  unrelated deletion. `/tmp` remains about 16 GB free, workspace about 4.5 GB.
- `root-exp-experiment.json` binds 20 source/support hashes and 103 evidence
  files. `verify-root-exp-checkpoint.mjs` passes, including the earlier log/erf
  checks and recomputation of benchmark medians from raw rows. A final live-tree
  comparison confirms all 176 frozen source/support files still match. Two
  nested sandbox runs lost verifier stdout despite exit 0; they are not used as
  completion evidence. Direct execution and the approved unsandboxed capture
  preserve all four verifier completion records.
- Next: preserve semantic root/exponential relations without abandoning useful
  numeric cache history; continue the open log and special-function work and all
  unread ecosystem references. Current checkpoint is **PROGRESS**, not completion.

## 2026-09-08 cache-preserving proof checkpoint

- Twelve additional full donor reads cover algebraic-conversion capability and
  execution, algebraicity/rationality/integrality checks, and all four ordering
  predicates: 1,981 lines independently read across both pinned snapshots.
  Total coverage is 150 complete files plus five partial files, not completion
  of Calcium/FLINT or the wider inventory. Conversion failure is not proof of
  irrationality; resource caps are selective, and several source TODOs are stale.
- A separate proof-only prototype recognizes logarithmic linear relations
  between positive exponential products, roots, inverses and binary scales.
  It leaves the original numerical DAG and operand caches intact, adds no node
  field or serialized state, and caches unsuccessful structural work through the
  existing exact-sign Unknown state. The six new tests include an independent
  512-case rational-coefficient oracle with unequal controls. Limits on terms,
  coefficients and collector visits do not bound all opaque structural-equality
  descendant work; that limitation remains explicit.
- All 48 public identity queries are Equal and all 48 perturbations NotEqual
  in debug/release. All 770 all-feature library tests pass in each profile;
  Clippy with warnings denied passes. Each profile also passes 15,050 directed
  MPFR enclosure checks and 261 cancellation/concurrency/opaque-input checks.
  Focused Memcheck has zero errors and no definite/indirect/possible losses
  (11,624 reachable bytes). Initial prototype and harness compile failures are preserved.
- Matched measurements add 5,040 CPU and 1,260 allocation observations. All
  72 numeric groups have identical allocation calls and requested bytes between
  baseline and proof trial. The prior multi-fold hot-operand regression is absent
  in this workload. This is not a general no-regression or memory-use guarantee.
- Public equality reconstructs its difference even when operands are retained.
  Warm-pair unresolved exponential queries cost about 1.20x and 1.30x; a separated
  exponential pair costs 1.55x. Reusing the difference itself caches failed proof
  work and is near parity for unresolved controls. New identity decisions can
  arrive much sooner than baseline Unknown, but those are not equal-work speedups.
  The linked numeric CPU driver grows 11,344 bytes, the query driver 8,776 bytes;
  production source delta is 143 lines / 5,782 bytes, plus 172 test lines.
- **Candidate pending, not rejected and not retained.** Completeness comes before
  performance under the requested priorities. The proof form currently discards
  a nonzero constant after opaque terms cancel; retaining its sign is a promising
  next qualification step. Query scheduling, opaque-comparison work limits and
  downstream consumer gates remain open. No production transfer is claimed.
- All 176 live Hyperreal source/support files still match the frozen baseline,
  including pre-existing user edits. Historical evidence remains unmodified.
  Existing build targets were reused with incremental compilation disabled;
  `/tmp` has about 16 GB free, the workspace about 4.3 GB. No deletion was needed.
- This checkpoint is **PROGRESS**. The full ecosystem goal remains active.

## 2026-09-08 scalar boundary continuation

- Another 21 files were read completely, independently across pinned snapshots:
  floor/ceil, binary64/complex import, conjugation, real/imaginary projections,
  phase/sign, root-of-unity logarithmic phase, and current arf conversion support.
  Supporting documentation/header ranges add 218 lines. This stage adds 2,216
  read lines: coverage now totals 171 complete and six partial files. The whole
  donor libraries and full ecosystem inventory remain incomplete.
- New current-FLINT correctness finding: `ca_arg` returns -pi for negative
  rationals, outside the documented (-pi,pi] principal range. Negative infinity
  follows this path through rational sign -1; negative sqrt(2) instead gives +pi.
  Independent expected pi fractions and special-state controls reproduce eight
  failures among 30 separate/in-place cases. Both snapshots have the same source
  mechanism, but only current FLINT was executed. No donor patch or issue filed.
- The same native probe passes 10,013 binary64 controls against GMP exact-rational
  conversion and special-value expectations. Arf storage remains unallocated for
  each of those inputs, confirming the current scalar import's omitted cleanup
  is not a leak on this path. All 84 simple dyadic real/complex rounding controls
  pass. Focused Memcheck reports zero errors and all blocks freed; its process
  exits 1 for the eight mathematical failures, not a memory error.
- Floor/ceiling of complex inputs intentionally use their real part. Their
  symbolic fallback is not equivalent to Hyper's certified integer/Exhausted API.
  Conjugation guards negative-real branch cuts before commuting through functions;
  transformed algebraic generators can change carriers, requiring reevaluation.
  These reads do not justify a numerical representation rewrite in Hyper.
- Hyper already imports finite IEEE values directly into exact dyadic storage,
  returns +pi on the negative axis, and preserves unresolved branch decisions as
  Exhausted in its checked atan2 API. Existing negative-axis regression tests
  passed in both frozen baseline profiles; no new production change is needed
  for this donor defect.
- `root-exp-proof-experiment.json` binds 21 source/support and 74 evidence files.
  Its verifier recomputes all four benchmark campaigns, including bootstrap
  intervals. The approved capture retains all five completion records; the
  sandboxed empty-output capture is explicitly not completion evidence.
  `verify-scalar-boundary-checkpoint.mjs` additionally checks the native boundary
  corpus, known failures, source coverage and Memcheck. Candidate disposition
  remains pending; no live Hyper source or historical bytes changed.
- Next: resolve the frozen proof candidate's nonzero-sign recovery and query work
  limits before downstream qualification, and continue unread scalar/import/
  evaluation paths and all remaining ecosystem references. **PROGRESS**, not
  completion. Existing targets reused; `/tmp` remains about 16 GB free.

## 2026-09-08 nonzero exponential proof / polynomial evaluation checkpoint

- Previous turn classified **PROGRESS**: additional complete source reads,
  verified cache-preserving candidate, and independently reproduced phase defect.
  This turn preserves the full goal; no narrowing or completion claim.
- Created `root-exp-sign-trial-hyperreal` from the verified 176-file baseline.
  The earlier zero-only candidate and its bound evidence remain unchanged.
  The new proof retains a nonzero rational log remainder after all opaque terms
  cancel: strict monotonicity gives sign(a-b) = sign(log(a)-log(b)) for proved
  positive a,b. Reversed subtraction is handled explicitly. Limits and numerical
  DAG/caches are unchanged; opaque structural-equality work is not wholly capped.
- All 772 all-feature library tests pass in debug/release, including the prior
  512-case rational oracle and two added tests for 30 signed tiny remainders,
  both Add/Negate orientations, no numerical approximation during proof, hot
  caches and serde recovery. Clippy with warnings denied passes. Each profile
  passes 15,050 directed-MPFR enclosure checks and 261 state checks. Focused
  ordinary and tiny-corpus Memcheck runs have zero errors/no lost blocks;
  reachable memory is 11,624 and 7,208 bytes respectively.
- Original corpus remains 48 Equal identities / 48 correct NotEqual controls.
  New public tiny-difference corpus: baseline gives 144 Unknown; trial gives 48
  Positive, 48 Negative and 48 Unknown, in both profiles. The 96 new decisions
  are 16 formulas at two orientations and three precision floors. The 48
  beyond-limit controls remain genuinely unresolved; no budget/input changed
  to conceal failed proofs.
- New matched campaigns record 5,040 CPU and 1,260 allocation observations.
  Separated exponential differences improve to 0.145x baseline CPU fresh and
  0.825x with retained operands, both with unchanged NotEqual outcomes. Warm
  unresolved pairs still cost 1.193x/1.234x; retained differences reuse Unknown
  cache work. All 72 numeric allocation groups match baseline exactly. None of
  the numeric groups has a bootstrap interval lower bound above 1.1 in this
  session; this is not a universal no-regression statement.
- Linked CPU driver growth: numeric +11,272 bytes, query +8,672 bytes. Non-test
  source-file delta is 150 lines / 6,102 bytes (includes a test-only wrapper);
  test file adds 248 lines / 10,155 bytes. No new dependency, node field or
  serialized state. **Candidate pending downstream/work-limit qualification,
  not rejected and not retained in production.**
- Ten more complete donor files independently read (1,992 lines) cover qqbar
  import and univariate/multivariate polynomial/rational-function evaluation.
  Coverage now totals 181 complete and six partial files. Generic sparse Horner
  uses an explicit stack and variable/power cost heuristic, with variable-by-term
  scratch storage and uncached powers. Hyper already has fused dense tensor-axis
  Horner and rational homogeneous Horner; no sparse-scheduler payoff established.
- Confirmed another current-FLINT correctness defect: generic non-Q
  `ca_fmpz_poly_evaluate` clears its Horner temporary without assigning `res`.
  Independent 64-case probe finds 24 stale-output failures: two nonconstant
  polynomials, three non-Q inputs, four destination states (including in-place).
  The 40 remaining integer-polynomial controls and all 64 rational-polynomial
  companion controls pass. Memcheck reports zero errors/all blocks freed;
  process exit 1 is mathematical failure. Archived source has the same missing
  assignment but was not executed. No donor patch or external issue submitted.
- `root-exp-sign-experiment.json` binds 25 source/support files and 101 evidence
  files. `verify-root-exp-sign-checkpoint.mjs` passes, including the prior chain,
  exact corpus membership and raw benchmark/interval recomputation. Donor pins
  and ranges still verify. Live Hyper source and historical bytes are untouched.
- Next: qualify consumer behavior and proof work costs before retention; continue
  remaining import/evaluation/serialization files and all other pending reference
  families. Existing targets reused, incremental builds disabled; no deletion.

## 2026-09-08 consumer / serialization checkpoint (qualification ongoing)

- Fifty-one additional donor files completely read, totaling 5,025 source lines,
  plus 41 new documentation lines: expression import/export/printing, copy and
  context transfer, representation hashing, field construction, factor storage,
  and scalar classification. Total coverage is 232 complete files and six partial
  files across these two donors, not full-library or ecosystem completion.
- Prepared identical snapshots of all five consumer crates: 773 files / 40,117,830
  bytes per variant. Only the sibling Hyperreal dependency changes. All-feature
  Cargo metadata resolves the intended six local crates in each variant; live
  sources and copied lockfiles are preserved. A first default-feature metadata
  check incorrectly expected optional Hypertri; the corrected all-feature check
  passes. This was a harness expectation error, not a dependency defect.
- All-feature library/integration tests pass in both variants for Hyperlimit
  (361 debug), Hypersolve (797 debug), Hypertri (187 debug), and Hyperlattice
  (203 release). Hypercurve's library suites each pass 914 tests with six existing
  ignored benchmark drivers; integration suites remain running at this update.
  This is not a full CI or both-profile downstream completion claim.
- New independent public consumer corpus: four arguments, seven exact/tiny
  exponent deltas, two orientations, four APIs. Baseline certifies 8 of 224
  queries; sign candidate certifies 160, adding 152 exact decisions. Strict
  scalar sign, ordering, 2D orientation and Hypersolve structural classification
  all benefit. All returned signs agree with exponential monotonicity; all 64
  beyond-cap controls remain Unknown. Focused release Memcheck is clean with
  7,208 reachable bytes and no definite/indirect/possible loss. Initial probe
  manifests used two spellings of the scalar path, causing lockfile collisions;
  unified-path builds pass. Failed harness logs are preserved.
- Confirmed current-FLINT expression round-trip loss: Arg(pi), Arg(pi+i), and
  Arg(1+pi*i) become Unknown under both export flags, although import succeeds.
  Six lost-value cases among 48; all 42 controls preserve known equality.
  Export lacks the Arg function head. Csgn's public implementation does not
  retain a formal node, so its absent exporter case is not a second demonstrated
  bug. Cross-context transfer uses this serialization bridge; that exposure is
  source-derived, not separately executed. Memcheck clean/all blocks freed.
- Confirmed current-FLINT factor-association error: merging a base at index i
  updates exponent0 instead of exponent i. An independent 90-case prime-factor
  corpus has 48 incorrect updates and 42 passing controls; all 90 initial
  reconstructions pass. Memcheck clean/all blocks freed. Archived source has
  the same two mechanisms, but only current FLINT was executed. No donor edits
  or external reports submitted.
- Other reads distinguish carrier cyclotomicity from value/root-of-unity facts,
  and representation hashes from exact equality. Hashing repeats the numerator
  and omits denominator/exponents, a distribution weakness, not itself false
  numerical equality. Q(i) +/-1 checks contain a wrong denominator pointer, but
  canonical rational demotion prevents claiming an ordinary public counterexample.
  Hyper already has stronger DAG sharing and exact aggregate mechanisms.
- Storage: /tmp's filesystem free count hid a per-user quota. The baseline cache
  moved to `/tmp/calcium-consumer-builds.X7kU2Y/calcium-baseline` with its original
  workspace path retained as a symlink. The prototype move left an incomplete
  duplicate; all 3,607 copied files were checked as prefixes of intact originals
  before approved removal of that 1.2 GB duplicate (no unique data removed).
  Baseline's 455 MB release cache then moved back to workspace
  `.audit-builds/calcium-baseline-release`, also linked from its prior path.
  The original quota-failed Hypercurve build and native compile remain recorded;
  unchanged retries succeed in compilation. Hyperlattice's native dependency
  required approved execution outside the sandbox for compiler-cache access.
  No unrelated cleanup. Last check: workspace about 1.7 GB free, /tmp user quota
  about 298 MB remaining despite roughly 13 GB filesystem free.
- Candidate still isolated and **pending work-cost/final downstream qualification**.
  The 128 collector visits do not bound opaque paired-DAG equality work. No new
  consumer CPU benchmark or live production transfer claimed. Finish running
  suites, bind the evidence, qualify that work cost, and continue every unread
  reference and historical candidate. Full goal remains active.

## 2026-09-08 completed consumer gates / opaque work-cost checkpoint

- Both Hypercurve all-feature debug library/integration runs finish successfully:
  45 suites, 1,761 passed and nine pre-existing ignored tests per variant.
  The ignored set is six benchmark drivers and three long stroke regressions;
  none is silently counted as passing. Exact test membership and Cargo's expected
  library/test target list agree across variants. Together with the previous four
  crates, completed consumer gates total 3,309 passing tests per variant.
  These are not full CI or both-profile consumer results. Their overlapping
  build/test wall clocks are not a controlled performance comparison.
- A new controlled CPU6 ABBA/BAAB campaign tests independently reconstructed
  nested-sine atoms at depths 1, 8, 32 and 128, for exact root/exponential
  identities and beyond-cap tiny perturbations, with retained operands or
  retained differences. It records 768 CPU and 192 allocation observations in
  16 groups. All expected identity/Unknown outcomes verify independently.
- For the depth128 unresolved retained pair, candidate/baseline CPU ratio is
  4.328 (bootstrap95% interval 4.259–4.490), roughly 9.233 us versus 2.160 us.
  Allocation calls rise25→39 and requested bytes1,768→11,220. Retaining the
  difference instead reuses its Unknown cache: CPU ratio1.030 and identical
  one allocation /768 requested bytes. Depth1/8/32 unresolved retained pairs
  cost about1.373/1.249/1.288x; the shallow identity cases gain exact decisions
  and lower latency, while depth128 identity costs3.302x but returns Equal
  instead of Unknown. Identity clocks are not equal-outcome speedups/regressions.
  Linked CPU driver grows8,696 bytes. Instrumented clocks are excluded from
  CPU conclusions; allocation requests are not peak heap.
- Source explanation: the128-visit collector and16-term normal form call the
  existing structural equality helper for opaque atoms. That helper uses a
  64-visit stack fast path followed by an iterative, memoized node-pair traversal.
  Its remaining work depends on the paired graphs and atom payload sizes;
  the collector limit is not a whole-query CPU/memory bound. Shared pointers
  shortcut comparison, but this benchmark intentionally rebuilds independent
  graphs. No universal asymptotic or all-sharing-pattern benchmark claim.
- The proof candidate remains isolated, **pending repeated opaque-comparison
  reuse/scheduling**. Do not lose the newly proved signs or identities to hide
  this cost. It is not discarded as a mathematical idea, and it is not retained
  in live production. No Hyper production edit from this continuation; all176
  original live source/support files still match the frozen user-worktree hashes.
- `sign-consumer-experiment.json` binds11 sources/support files and87 evidence
  files for the earlier completed four-crate checkpoint. The subsequent
  `sign-work-cost-experiment.json` binds nine sources/support and31 evidence
  files, including completed Hypercurve results, its quota failure, and both
  opaque benchmark campaigns. `verify-sign-work-cost-checkpoint.mjs` passes
  all nine verifier checkpoints and recomputes raw medians, bootstrap intervals,
  exact test/corpus membership, source snapshots and known-failure dispositions.
- Root ledger and detailed Calcium report are current; both donor pins and all
  tracked source hashes verify. Current slice coverage remains232 complete and
  six partial files, not full Calcium/FLINT or inventory completion. No processes
  from these qualification campaigns remain running. Next: improve proof reuse,
  qualify the revised candidate without reducing coverage, and continue all
  remaining references and historical experiments. Full goal stays active.

## 2026-09-08 scalar source closure / isolated weak-cache experiment

- Added 64 complete source reads (1,721 lines), covering all remaining top-level
  scalar wrappers, constructors, special predicates, and random generators, plus
  33 supporting FLINT header lines. Combined coverage: 296 complete and seven
  partial files. Hash/range verification passes, including the supporting source
  template range. An initial entry mistakenly named the generated `flint.h`;
  verification rejected it, and the tracked `flint.h.in` range was independently
  read and correctly recorded. The pinned inventory was not overwritten.
  This closes neither the scalar tests nor the full Calcium/FLINT libraries.
- Reproduced a random-constructor normalization defect in current FLINT:
  `ca_randtest_same_nf` produces 2,221 noncanonical rationals among 10,000 seeded
  four-bit samples. Exact `is_one` falsely rejects 107 of them (e.g. 2/2);
  all 10,000 independently canonicalized-copy controls pass. Native Memcheck
  reports zero errors/all blocks freed. This is a public test-data helper bug,
  not a claim that ordinary canonical rational arithmetic fails. Archived source
  has the same mechanism but was not executed. No donor patch submitted.
- A new isolated `root-exp-reuse-trial-hyperreal` keeps all v2 proof rules and
  adds a 16-entry thread-local weak-key structural-comparison cache. It adds no
  node fields or serialized state and retains no strong expression graph. Exact
  pair keys, not hashes alone, establish hits; unavailable/reentrantly borrowed
  cache falls back to the original comparison. Worst-case uncached work remains.
  All-feature library gates pass 780 tests each in debug and release, including
  eight cache lifetime/collision/reentrancy/deep-proof tests. Clippy lib/tests
  all features with warnings denied passes. Focused deep-fresh public Memcheck
  passes with zero errors/no definite, indirect or possible losses (1,928 bytes
  reachable); graph lifetime and thread-exit weak-key tests pass. Not production-qualified.
- A three-way CPU campaign was preserved but excluded because it overlapped
  Memcheck. A separate non-overlapping confirmation records 1,728 observations
  in 24 groups. For depth 128 unresolved retained pairs, v3 takes 2.640 us versus
  v2 9.271 us and baseline 2.214 us: v3/v2 ratio 0.2854 (95% 0.2723–0.2862),
  v3/baseline1.1944 (95% 1.1574–1.2414). The identity pair takes 0.629 us versus
  v2 7.099 us, with identical Equal outcomes. Baseline still returns Unknown.
  Fresh depth 128 cases are near v2 parity. Unresolved shallow retained pairs
  remain 1.18–1.36x baseline, and some retained-difference rows show small
  regressions; no universal no-regression claim.
  Linked CPU driver grows 3,080 bytes over v2, 11,776 over baseline; not whole-app size.
- Separate allocation qualification records 432 observations across the same 24
  groups (ten fresh or 100 retained queries per observation). Deep unresolved
  pairs fall from v2 39 calls/11,220 requested bytes to v3 30/2,392; baseline is
  still 25/1,768. Retained unresolved differences remain 1/768 in all variants.
  Fresh deep allocation counts match v2; weak retention does not appear as a new
  allocation request and requires separate live-memory qualification. Instrumented
  clocks are not used for CPU conclusions. Non-test-file source delta is 234
  lines/9,088 bytes versus baseline, including test-include directives.
- `root-exp-reuse-experiment.json` binds 20 source/support files and 60 evidence
  files. `verify-root-exp-reuse-checkpoint.mjs` passes all ten chained checkpoints,
  checking source footprint, all 780 test names, prior 772-test inclusion, complete
  scalar-C read ranges, native failure controls, every benchmark row/order/outcome,
  recomputed medians/intervals, and exclusion of the overlapping CPU campaign.
  Formatting and scoped diff whitespace checks pass. No qualification process
  from this checkpoint remains running; all 176 live Hyperreal snapshot files
  still match the original user-worktree baseline.
- V3 remains isolated pending directed-MPFR/public/consumer reruns, numeric-kernel
  workloads, cold-TLS/high-thread-count costs, wider sharing/collision patterns,
  and live-memory/application-size tradeoffs. Prior v2 qualification is not
  silently credited to this new cache. The current turn is **PROGRESS**, not
  completion or a production retention decision; every remaining donor and
  historical experiment stays in scope.
- After the user's cleanup, /tmp has about 23 GB remaining within the user quota
  (36 GB filesystem free); workspace remains about 1.7 GB free. New compiler
  artifacts use `/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse`, incremental
  compilation disabled. No existing evidence, source, or build cache was deleted.
  At checkpoint the new cache uses 647 MB; /tmp quota has about 22 GB remaining.

## 2026-09-08 weak-cache correctness / live-memory qualification checkpoint

- Read all remaining scalar `ca/test` sources independently at both pins, plus
  current `src/ca.h`; file notes and exact ranges are in `coverage.json`.
  Inventory hash/range verification passes: 348 complete/seven partial files,
  41,398 lines (52 new complete files/6,132 lines). Many randomized
  donor equalities permit Unknown or merely test ball overlap; strict special
  truth matrices are stronger. The Arg round-trip, random rational normalization,
  and in-place-state counterexamples are not covered by those random inputs.
- Frozen v3 passes 15,050 directed-MPFR checks over 43 inputs and 261
  state/serialization checks in each profile. Public 96-case and tiny-delta
  144-case reruns exactly match the previously verified v2 rows. New concurrent
  proof/approximation/serialization stress passes 1,732
  sign queries and 3,456 directed enclosure checks in debug and release.
  An initial harness used a private shift method and failed to compile; corrected
  harness uses public multiplication. No candidate-library edit was needed.
- Rechecked/copied all 773 frozen downstream files; corresponding live bytes are
  unchanged. This check does not inventory newly added live files. Hyperlimit,
  Hypersolve, Hypertri and approved Hyperlattice release reruns finish successfully.
  Hypercurve finishes at22:10:23 UTC: 1,761 passed/nine pre-existing ignored tests,
  45 suites. All five total3,309 passed. Exact names/statuses/ignored reasons and
  expected Cargo library/integration targets agree with the frozen baseline.
  These are not full CI/both-profile results; overlapping clocks are not benchmarks.
  Original Hyperlattice sandbox/compiler-cache failure is preserved separately.
- Live-memory campaign records 162 observations across three variants, 1/8/32
  threads, 1/64/512 rounds, and depths1/8. After worker expressions are dropped,
  baseline/v2 retain zero requested heap bytes; v3 retains at most 2,304 bytes
  (32 dead node allocations) per worker in this corpus. All workers' retained
  allocations disappear at thread exit. Eight-worker Memcheck passes with zero
  errors/no definite, indirect, or possible losses (1,344 reachable bytes).
  Stack, allocator overhead and process RSS are not measured. Separate linked
  ELF inspection shows static TLS848→1,248 bytes (+400 per thread, including
  untouched threads). Raw row/summary/binary-hash verification passes. The runner
  console captures are empty despite complete
  raw162 rows and summary; exit0 alone is not being used to credit coverage.
- Consumer public release/Memcheck rows exactly match all224 verified v2 queries
  (160 known/64 Unknown). Memcheck is clean, 7,208 reachable bytes. Concurrent
  proof-state Memcheck completes all numerical checks but returns99 for one
  48-byte possibly-lost Rust scoped-thread initialization record. A standalone
  std-only scoped-thread control reproduces that record without Hyper/GMP/MPFR.
  Zero definite/indirect loss; no suppression and no clean-run relabeling.
- Build artifacts remain under `/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse`.
  Cache uses3.4 GB; /tmp user quota has about20 GB headroom. No caches, historical
  evidence, or user source files were deleted. V3 remains
  isolated: no production retention decision or full audit completion claimed.
- `reuse-qualification-experiment.json` binds30 sources/support files and104
  evidence files. `verify-reuse-qualification-checkpoint.mjs` passes all11 chained
  checkpoints, including all prior immutable sources/results, new exact consumer
  target/test membership, public/tiny/consumer corpus equality, oracle/state counts,
  all162 live-memory rows and bounds, static TLS sizes, complete scalar-test/header
  read coverage, and explicitly classified failed harness/environment/control runs.
  All qualification handles for this checkpoint are terminal. Progress is
  substantive, not completion. Next: numeric-kernel and cold-TLS latency costs,
  broader sharing/collision and application-size qualification, then a justified
  retention decision; all remaining donor reads and earlier experiments stay open.

## 2026-09-08 retained exponential proof / cost and vector-layer checkpoint

- Previous goal turn classified PROGRESS: scalar-test source closure and the
  independently verified v3 correctness/consumer/live-memory checkpoint. All11
  prior checkpoint verifiers pass again before new work; frozen v3 is unchanged.
- Read22 additional vector source/header/documentation files independently at
  both pins (1,595 lines). Hash/range verification passes. Combined coverage:
  370 complete/seven partial files,42,993 lines. This closes the selected ca_vec
  directories/headers/docs, not all Calcium/FLINT or the full ecosystem.
- Reproduced public in-place vector negation no-op in current FLINT:24 wrong
  results among105 cases; separate-output/raw in-place and zero/empty controls
  pass. All27 vector zero/unknown/nonzero truth combinations pass. Memcheck zero
  errors/all blocks freed. Archived implementation has the same source mechanism
  but was not executed. Docs state negation without a per-function alias caveat;
  this is not a universal claim about arbitrary overlapping subranges. Hyper
  already has componentwise owned/borrowed negation, nonzero-dominates-Unknown
  norm certificates, and retained common-scale/fused product-sum paths.
- New sequential three-way campaigns finish without build/checker overlap:
  numerical72 groups (5,184 CPU/1,296 allocation observations) and first-touch24
  groups (1,728 CPU/432 allocation observations), rotating six-run blocks onCPU6.
  CPU and allocation binaries are separately preserved in
  `/tmp/calcium-reuse-costs.73SR1w`; no original cache/evidence overwritten.
  First-touch workers have fresh TLS but parent-preconditioned numeric operands;
  clocks separately measure first query,16 subsequent queries and total worker
  lifecycle. Shared/common-tail/independent argument graphs are included.
- Initial analysis: numerical allocation totals exactly match v2 for all72
  groups. Numerical median CPU ratios are close, with some few-percent changes;
  deep unresolved first queries still pay uncached traversal (v3 about15.8us,
  v2 about15.3us, baseline Unknown about4.2us). Warm repeats recover v3's reuse
  benefit. Detailed raw-row/interval verification and checkpoint binding pending.
  Capture consoles are empty despite complete raw/summary artifacts; exit0 alone
  is not coverage evidence. No instrumented-clock performance conclusions.
- Frozen Hypercurve basic/arrangement release-example size builds and v3
  all-feature WASM compile check finish successfully. Both stripped examples
  run their assertions; each grows8,656 bytes versus baseline, about0.08%, and
  about2.5 KB versus v2. These are representative executables, not all Alumina
  binaries or size-optimized/LTO builds. WASM is compile-only qualification.
- `reuse-cost-experiment.json` binds22 source/support files and132 evidence
  files. All12 chained checkpoint verifiers pass, including every new raw row,
  paired bootstrap interval, binary snapshot, no-overlap schedule, vector result,
  read range and example size/run. Numeric CPU paired ratios range0.927–1.048
  versus baseline and0.907–1.032 versus v2; no blanket no-regression claim.
  First-use v3/v2 ratios range1.008–1.102. Independent depth128 unresolved first
  query ratio is1.030 versus v2 /3.772 versus baseline; its16-repeat warm phase
  is0.284 versus v2 /1.217 versus baseline. All numeric allocation totals and
  first-query Rust allocation totals match v2. Clocks and bounds are host/corpus
  observations, not whole-query complexity or whole-process cold-start bounds.
- Selected v3 for retention under the requested exactness/completeness-first
  priority: it proves additional exact equalities/signs without altering numeric
  nodes, cache validity, public types or serialization, and avoids repeated deep
  paired-graph work. Accepted costs are the measured unresolved/cold-query work,
  bounded weak retention,400-byte native TLS increment and small linked size.
  No attempt is made to hide those costs by reducing the proof domain.
- Before transfer all176 baseline live source/support hashes still matched and
  four new filenames were unoccupied. Applied only two existing-file deltas and
  four new proof/cache/test files. All180 live source/support files now match
  frozen qualified v3 byte-for-byte, preserving every pre-existing user change.
  Live all-feature library/integration tests pass855 per profile:780 library and
  75 integration tests across14 suites, no ignored or filtered tests. Exact test
  membership agrees across profiles and all780 library names match frozen v3;
  expected Cargo library/integration targets also agree. Clippy all-features/
  all-targets with warnings denied, formatting, and WASM library compilation pass.
- **Retained in live Hyperreal.** `retained-exp-proof.json` binds the exact180-file
  source state and18 terminal gate artifacts; `verify-retained-exp-proof.mjs --live`
  passes all13 chained checkpoints and verifies current production bytes against
  qualified v3. Its default mode verifies the historical frozen transfer without
  blocking later authorized live changes. Both prior failed rewrites and all
  measured regressions remain recorded. This does not retain the separate rejected
  log/erf/eager-root prototypes or close their still-open mathematical ideas.
- All processes/qualification handles in this checkpoint are terminal. Build
  cache uses about4 GB; separately preserved CPU/allocation/example/native binaries
  use165 MB under `/tmp/calcium-reuse-costs.73SR1w`. User quota has about19 GB
  headroom. No cache/evidence deletion, commit, push or donor patch. Full ecosystem
  audit remains active: continue the unread algebraic/symbolic support, other
  references and unresolved historical transfers with the retained live baseline
  distinguished from the immutable pre-transfer snapshots.

## Polynomial decision continuation — in progress

- Previous goal turn classified NO PROGRESS: it summarized existing evidence.
  Revalidated live retention with all13 chained checkpoints before continuing.
- Independently read30 polynomial headers/docs/core/test files across both pins;
  exact ranges and per-file notes are added to coverage.json. This includes
  normalization, three-valued comparisons, division, GCD and relevant donor tests.
- New transfer hypothesis: Hypersolve's subresultant zero-polynomial helper
  returns on an unresolved low coefficient before a later exact nonzero one.
  Calcium's reducer instead lets known nonzero dominate Unknown. A public probe
  is being prepared; no Hypersolve production change or new retention claim.
- Numerical Sylvester coprimality certificates and rotating remainder storage
  are recorded for comparison with Hyper's existing exact resultant and certified
  divisor paths. Archived-only integer-division/header and cleanup differences
  are source observations, not executed archived defects.
- Ten additional polynomial storage files bring this turn's complete reads to40
  (4,159 lines). Native81 coefficient combinations pass405 truth/length/proper/
  monic checks, with zero Memcheck errors and all allocations freed.
- The public630-query corpus confirms120 unnecessarily undecided subresultant
  queries. Isolated fallback scans past Unknown, returns false on any proved
  nonzero coordinate, otherwise preserves the first unresolved index. Debug and
  release corpus results gain all120 decisions and retain60 unresolved-leading
  controls. These are repeated degree/position/budget cases, not120 identities.
- Added three isolated tests, including256 four-coordinate truth combinations.
  Baseline and candidate debug regression commands finish successfully. A command
  initially tagged trial-tests-release accidentally omitted --release; it is
  preserved as a debug run, not credited as release. A separately tagged corrected
  release command is running. Benchmarks and final retention decision remain open.
- All qualification processes are now terminal. Candidate passes800 tests across
  seven suites in debug and corrected release; baseline passes797 with exact
  membership matching the prior frozen qualification. The only new names are
  the three focused tests. All-feature/all-target Clippy and formatting pass.
  Public630 rows match exactly across each variant's profiles; candidate Memcheck
  passes all630 with zero errors/no definite, indirect or possible losses
  (3,240 reachable bytes). The initial debug probe preceded test-only additions;
  a final-source debug rerun is independently recorded and matches it.
- CPU campaign:1,728 observations,36 groups,12 alternating ABBA blocks,CPU6.
  First48 observations are excluded because formatting overlaps campaign start
  by65ms. The first group's286.879ms summed query clocks prove all later groups
  start after formatting; all other build/checker clocks are disjoint. Accepted
  scope is35 groups/1,680 observations, not the complete36-group campaign.
  All raw medians/paired bootstrap intervals and query outcomes reverify.
- Same-outcome controls span0.899–1.017 trial/baseline paired median ratios
  (17 eligible groups); no universal improvement is claimed. Newly decided
  degree16 retained log-self queries cost67.849us versus baseline2.918us Unknown
  (ratio23.072), and tiny-self43.967us versus2.287us Unknown (19.070). These are
  different proof outcomes, not equal-work regressions. Repeated lower unknown
  checks should be scheduled around already-certified leading nonzero facts
  before accepting this implementation. Proof domain must not be reduced to
  hide cost. Allocation/lifecycle/serialization, broader consumers and application
  size remain unqualified. Linked unstripped driver changes by72 file bytes,
  not an application-size conclusion.
- `polynomial-decision-experiment.json` binds74 source/support/evidence files,
  five binary snapshots,18 terminal gates and40 independently read donor files.
  `verify-polynomial-decision.mjs --live` passes all14 chained checkpoints,
  including exact630-query membership, test names, native81 truth combinations,
  every CPU row/interval, explicit exclusions and unchanged live source bytes.
  Inventory verification also passes at both pins:410 complete/seven partial
  files,47,152 read lines. This is not whole-library or ecosystem completion.
- No production or donor edits, commits, pushes, or deletions in this checkpoint.
  New binary evidence uses9.4MB in `/tmp/calcium-polynomial-decision.ItisYX`;
  shared target is5.5GB and /tmp quota has about17GB headroom. The339-file
  isolated three-crate snapshot is15MB. Preserve the v1 candidate/evidence;
  next experiment should separately qualify a fact-reuse schedule, while all
  other unread references and prior unresolved transfer ideas remain in scope.

## Polynomial fact-first continuation — retained scoped improvement

- Previous goal turn classified PROGRESS:40 complete source reads, a public
  completeness gap, isolated v1 and verified cost evidence. All14 checkpoint
  verifiers pass again against live bytes before the new experiment.
- A separate339-file v2 snapshot preserves v1. Its cold fallback scans existing
  structural nonzero facts from the leading end before the full three-valued
  refinement scan. It does not guess degree, drop Unknown terms, or narrow the
  proof domain. All630 debug public rows match v1 exactly. No live source edit.
- Three-way CPU campaign is running only after the new build, public probe and
  captured formatting check are terminal. Baseline/v1 binary snapshots are
  hash-checked, never rebuilt or overwritten; v2 uses a separate binary snapshot.
  Further correctness/state/consumer and allocation qualification remains open.
- The campaign finished with2,592 observations in all36 groups,12 rotating
  forward/reverse three-variant blocks onCPU6. All timed observations are clear
  of build/checker overlap. Same-outcome control paired ratios span0.973–1.029
  versus baseline. At degree16 retained, log-self v2 is4.549us versus v1
  67.286us (paired ratio0.06946); tiny-self4.323us versus42.688us (0.10301).
  Baseline returns Unknown in these cases, so v2/base ratios1.609/1.953 are not
  equal-work regressions or speedups. No universal no-regression claim.
- Candidate passes800 tests per profile,630 public queries per profile and
  1,090 state queries per profile, including aborted-observation recovery,
  later-refinement recovery, independent deserialization, shared four-worker
  queries, and64 unchanged serialization checks. Clippy, formatting and WASM
  compile-only gates pass. Public Memcheck has zero errors/no definite,
  indirect or possible loss. Downstream Hypercurve is still running.
- Initial allocation campaign stops after450 rows: a v1 fresh tiny case has
  equal request counts/bytes but live-byte totals2160 and2304. Preserved the
  failure. Existing scalar proof cache is pointer-indexed, with16 slots and
  two72-byte weak allocations per slot; occupancy need not be deterministic.
  Separate bounded campaign passes648 observations plus27 churn controls at
  100/1,000/10,000 fresh queries. All repeated request totals agree. Retained
  operands add zero live requested bytes; fresh tiny churn stays at/below2304.
  Other fresh controls retain0 or384 bytes identically across variants. These
  are requested-byte deltas, not peak RSS, native heap, or allocator overhead.
- Degree16 retained log-self allocation requests fall from1,117 /160,976 bytes
  in v1 to13 /7,760 bytes in v2; tiny-self589 /115,280 to13 /7,760. Same-outcome
  control totals match baseline. No numeric nodes, caches or public types added.
- Two qualified stripped Hypercurve release examples grow576 and560 bytes
  versus the retained scalar/unchanged solver baseline. Both run assertions.
  Unstripped file sizes shrink slightly due to layout; executable text does not
  justify a binary-size reduction claim. These are not full Alumina/LTO sizes.
- Independently read32 additional donor/source-support files,2,959 lines, now
  recorded with per-file notes and verified pinned hashes. Combined coverage:
  442 complete/seven partial files,50,111 lines. Includes multiplication and
  packed common-field convolution, derivative/integral, add/sub/neg, Horner,
  donor tests, and matrix/field conversion callees. Hyper already has primitive
  integer rational specialization, sparse polynomial collection, shared scales
  and specialized product sums. Common-field packing remains an open candidate,
  not a generic computable-real replacement. Borrowed-view source capacity
  initialization is a delimited source observation, not a reproduced defect.
- Native current-FLINT control passes180 cases /4,860 required-True checks:
  scalar-reference convolution, truncation, whole-input aliases, squaring,
  derivative/integral round trips, negation and evaluation. Integer, rational,
  quadratic, degree-four field and mixed-field inputs include short/empty and
  dispatch-threshold lengths. Memcheck has zero errors and all blocks freed.
  No branch-coverage instrumentation, archived execution or matched donor timing
  is claimed. No production transfer yet; binding/retention gate remains open.
- Downstream Hypercurve completes in26m29.5s:1,761 passed/nine pre-existing
  ignored tests across45 suites. Exact test membership matches the frozen
  retained-scalar baseline. The ignored tests were not run; no full-CI claim.
- `polynomial-facts-experiment.json` binds118 source/support/evidence files,
  29 terminal gates (one explicitly classified failed allocation assumption),
  17 binary artifacts and all32 read entries. The15-checkpoint verifier passes,
  independently recomputing every CPU interval/allocation range and confirming
  all773 live consumer baseline files before transfer. All180 retained live
  Hyperreal files also remain byte-identical to their qualified scalar snapshot.
- Retained v2 for its correct120 additional public decisions, preserved degree
  uncertainty and much cheaper repeated work than v1. Accepted the measured
  small same-outcome timing variation and576/560-byte stripped-example growth.
  Applied only `hypersolve/src/resultant.rs`:103 added/two removed lines including
  three focused tests. No dependency, scalar representation or cache change.
- Live all-feature library/integration tests pass800 per profile, with exact
  seven-target Cargo metadata and test membership matching frozen v2. Live
  all-feature/all-target Clippy with denied warnings, fmt and WASM library
  compilation pass. `retained-polynomial-facts.json` binds23 supporting/evidence
  files and the953-file scalar/consumer state; only the solver file differs
  from the prior retained baseline. `verify-retained-polynomial-facts.mjs
  --retained-live` passes all16 chained checkpoints and checks current bytes.
- All processes/handles for this checkpoint are terminal. Shared target7.4GB;
  new separately preserved binary evidence68MB. The /tmp quota has about15.3GB
  headroom. No source/evidence deletion, donor edit, commit or push. Full audit
  remains active and incomplete: unread polynomial/matrix/algebraic support,
  all pending references, and earlier unresolved transfer ideas remain in scope.

## Polynomial roots and series continuation — in progress

- Previous goal turn classified PROGRESS:32 complete reads, extensive new
  qualification and the retained fact-first solver change. All16 checkpoint
  verifiers pass again against the953 live source/support files before new work.
- Independently reading roots, squarefree factorization, root-product creation,
  inverse/division/log/exp series, integer powers and their donor tests at both
  pins. Root extraction separates squarefree multiplicities, exact rational
  algebraic roots and explicit quadratic/cubic formulas; higher nonrational
  degrees can fail. Hyper already has certified squarefree/GCD/degree carriers
  and exact interval-owned algebraic roots, so formulas are not a general upgrade.
- Series use rational denominator clearing, one reused reciprocal, sparse
  monomial recurrences and high-part Newton updates where field multiplication
  is fast. Donor tests allow Unknown equality and skip some failed decisions;
  independent required-True coefficient and multiplicity controls are next.
  No production change or new performance claim in this continuation yet.

- Completed the 34 independent donor reads (17 per pin), 4,493 lines. Exact
  ranges and file notes are now in coverage.json; both pinned inventories verify.
  Combined continuation coverage is 476 complete/seven partial files and 54,604
  lines. Remaining polynomial, matrix, algebraic and wider reference work stays open.
- Current-native dense series passes 280 cases/5,880 required-True comparisons:
  inversion, division, log, exp and truncated powers against scalar recurrences,
  including whole-input aliases. Rational/quadratic/quartic/mixed fields, orders
  0–17 and selected dispatch-boundary lengths are represented. A separate sparse
  monomial corpus passes 756 cases/3,024 required-True comparisons through order20.
  These are independent polynomial recurrences, not an independent scalar backend
  or instrumented proof that every internal branch executes.
- Native root control passes all81 known-factor cases: root-product construction,
  squarefree factor reconstruction, normalized squarefree part, and one-to-one
  root/multiplicity matching. Inputs use 0/1/2, 0/+sqrt(2)/-sqrt(2), and
  0/ln(2)/(ln(2)+1), with each multiplicity0–2 and both leading-scale signs.
  All three native Memcheck runs have zero errors and every allocation freed.
- Preserved the initial Hyper harness failure at logarithmic case2/5. It demanded
  exact equality of each unnormalized coefficient, stronger than the documented
  same-distinct-roots contract. A separate diagnostic completes all81 inputs:
  68 results have the correct degree and every projective coefficient comparison
  is Equal;13 logarithmic cases remain Unknown. No incorrect degree or disproved
  polynomial was observed. Three direct coefficient comparisons remain Unknown
  because a nonzero unit scale is represented as a*(1/a).
- Isolated a339-file monic trial based on the retained solver. Only its private
  gcd_monic_normalize helper changes: after obtaining the certified reciprocal,
  it scales lower coefficients and publishes the leading coefficient as exact
  one. The same81-case corpus still returns68 results/13 Unknown, but all three
  previously unresolved coordinate cases become Equal and two root checks also
  become Equal. Six other root-residual queries still remain Unknown.
- Baseline and candidate outputs each match exactly across debug, release and
  focused Memcheck. Both Memcheck runs have zero errors/no definite, indirect or
  possible loss; reachable bytes are46,688/46,408, not a peak-memory or comparative
  memory qualification. Candidate passes all800 solver tests in both profiles
  across the same seven suites and exact test membership as retained baseline;
  formatting passes. No new in-crate regression tests added yet.
- **Monic candidate remains isolated and unretained.** It has no matched timing,
  allocation/lifecycle, serialization/concurrency, downstream, application-size,
  Clippy or WASM qualification. Do not attribute the prior retained solver's
  benchmarks or consumer gates to this different source. Next qualify the proven
  leading-one fact and the remaining logarithmic GCD blocks independently.
- polynomial-closure-experiment.json binds74 source/support/evidence files,
  19 terminal gates (including the classified initial failed assertion), eight
  binary snapshots, the native FLINT library hash, all339 candidate source hashes
  and all34 read entries. verify-polynomial-closure.mjs --retained-live passes
  all17 chained checkpoints and rechecks953 live source/support files. The draft
  nested console capture was empty despite exit0 and is not completion evidence;
  direct verification completed, followed by an approved full-output capture.
- Preserved binaries total17,095,376 bytes in the existing small closure directory;
  builds reused the shared target with incremental compilation disabled. No new
  large checkout, deletion, donor/production edit, commit or push. All current
  qualification processes are terminal. An interim report is kept at
  [EXACT_REAL_ECOSYSTEM_AUDIT_REPORT.md](EXACT_REAL_ECOSYSTEM_AUDIT_REPORT.md).

## Monic cost and polynomial implementation checkpoint — candidate still isolated

- Previous goal turn classified PROGRESS:34 completed reads, native/Hyper
  qualification, a new monic candidate and the17-checkpoint report. Revalidated
  all17 checkpoints and953 live source/support hashes before further work.
- Added46 complete donor/support reads,2,740 lines, with exact ranges and pinned
  hashes. All remaining top-level ca_poly C files are now read:56 archived files/
  4,462 lines and53 current files/3,913 lines. This closes that implementation
  directory only, not its tests, all supporting kernels, whole libraries or the
  ecosystem. Combined continuation coverage:522 complete/seven partial files,
  57,344 lines. Both inventories verify.
- New reads cover composition and parity-selected scratch buffers, sparse
  affine/binomial substitution, Taylor shifts, export/transfer, reversal/shifts,
  setters and capacity-wide polynomial-vector lifetime. Generic composition and
  binary-power callees are independently read, not inferred from wrapper shrinkage.
  Clarification of an earlier wrapper note: binexp means binary exponentiation,
  not binomial expansion. Hyper already has affine Horner conversion, exact
  positive-scale integer carriers and owned Vec reuse. General fast composition
  remains workload-gated; no unconditional port is selected.
- Native composition/storage control passes384 cases/4,608 required-True checks,
  using scalar-convolution powers as the composition reference. Tests cover
  zero/constant/short/threshold outer lengths through21, dense and sparse inner
  forms, rational/quadratic/quartic/mixed coefficients, both whole-input aliases,
  padded reversal, shifts and same-/cross-context transfer. Memcheck reports zero
  errors and all allocations freed. Not a separate scalar oracle, instrumented
  branch proof, archived execution or special-value/alias-completeness claim.
- Frozen monic v1 remains unchanged. New uninstrumented CPU campaign has7,776
  observations:all81 known-factor recipes,two lifecycles,12 alternating ABBA
  blocks per group,CPU6. Recorded build/checker gates do not overlap timing.
  Eight preconditioning queries precede measurement; retained includes required
  input Vec cloning, fresh includes construction and teardown. Same68 known/13
  unresolved split and expected degrees hold throughout both variants.
- All162 group medians/paired bootstrap intervals reverify from raw rows. Ratios
  range0.9007–1.0667, with selected controls about2–7% slower; no blanket speedup.
  Logarithmic case5 retained is2.530us baseline/2.303us trial (paired0.9153), and
  case11 is2.069us/1.839us (0.9078). Several wider intervals include parity.
  These measure squarefree construction, not subsequent coefficient/root proofs;
  equal availability does not mean identical output representation. Intervals
  are per-group host/corpus estimates, not multiple-comparison-adjusted bounds.
- Separate allocation campaign passes972 observations over the same162 groups.
  Requested counts/bytes decrease in56 groups and never increase in this corpus;
  incremental requested peaks decrease in44 groups and never increase. All
  repeat totals agree; live deltas match between variants. Representative case11
  retained falls14→11 requests and1,400→1,184 requested bytes per query. Largest
  observed post-loop live delta is1,640 bytes; this is not RSS or a general bound.
- A separate108-observation churn probe covers nine recipes, both lifecycles and
  1/100/1,000 queries. Live deltas do not grow with query count in those cases;
  retained inputs add0 bytes, and fresh deltas are at most384. It does not cover
  every higher-retention recipe or establish a universal heap/lifetime result.
  Linked CPU driver file grows1,400 bytes,text36,loaded total4; no application
  size conclusion. Candidate Clippy all-targets/all-features with warnings denied
  and WASM all-feature library compilation pass; WASM execution remains untested.
- Tested the existing public fraction-free chain as a possible alternative to
  Euclidean GCD on the same known-factor families, at two precision floors. Debug
  and release each give128 known/34 Unknown across162 queries, with all known
  degrees correct; focused Memcheck is clean with49,448 reachable bytes. It
  resolves none of the13 original gaps and leaves four more family cases Unknown.
  Thus neither direct replacement nor fallback closes these sampled gaps; no
  production routing change. This is a scoped capability comparison, not a
  matched-cost comparison or proof that fraction-free methods are useless.
- monic-cost-experiment.json binds65 source/support/evidence files,13 terminal
  gates,seven binaries (15,597,136 bytes) and46 read entries. The18-checkpoint
  verifier recomputes every timing/allocation/churn row and interval, checks
  implementation-directory coverage, native/Hyper corpus membership and unchanged
  candidate/live source hashes. Historical snapshots and failures stay immutable.
- No production or donor edit, deletion, commit or push. Builds reuse the7.8GB
  shared target; new preserved binaries occupy about15.6MB. The whole goal stays
  active. Before monic retention, add focused in-crate regressions and independently
  qualify lifecycle/serde/concurrency, higher-degree/scale controls, downstream
  callers and representative application size. Continue unread polynomial tests,
  remaining donor support and every pending ecosystem target/transfer in parallel
  with useful local qualification work (no subagents have been used).

## 2026-09-09 monic state/size and polynomial-directory closure

- Previous goal turn classified PROGRESS:46 reads and verified monic cost/native
  evidence, not a retention or full-audit completion. Revalidated all18 prior
  checkpoints and953 retained live files before this work.
- Completed the eight remaining ca_poly test/support files,579 lines, independently
  at both pins. Entire directory coverage is74 archived files/6,371 lines and70
  current files/5,571 lines. Combined continuation coverage is530 complete/seven
  partial files,57,923 read lines. Inventories and every selected range/hash verify.
  This does not close recursively called generic kernels, matrix/algebraic modules,
  all donor libraries or the original ecosystem inventory.
- Archived atan-series test has a TODO instead of a mathematical assertion.
  Composition and full-power tests fail only on T_FALSE and accept Unknown.
  Current15-function polynomial test registry was read and all15 functions run
  successfully; their permissive assertions remain explicit limitations. Added
  an independent scalar repeated-product full-power probe:540 cases/1,080 required-
  True checks, exponents0–8, lengths0/1/2/4/8, constants−1/0/1 and four coefficient
  field modes, with separate/in-place outputs. Native and Memcheck output agree;
  zero memory errors/all allocations freed. Polynomial oracle is independent of
  powering/convolution kernels, not an independent scalar backend or branch proof.
- New qualification snapshot adds only a cfg(test) module and three regressions
  to the immutable monic implementation. Runtime source is byte-identical to
  the benchmarked checkpoint17 candidate after removing that test-module inclusion.
  Of the773 retained consumer source/support paths, only root_isolation.rs differs;
  the added test file brings the consumer snapshot to774 files.
  Tests cover scalar/empty/failed-reciprocal boundaries, lower-coefficient preservation
  through degree64 with seven leading forms, and three logarithmic unit regressions.
- All803 solver tests pass in debug and release, with exact membership equal to
  the old800 plus those three tests. Eight suite summaries include a zero-test
  doctest suite. All-feature/all-target Clippy with denied warnings, formatting,
  and all-feature WASM library compilation pass; no WASM execution or full-CI claim.
- Public state probe has1,932 queries per variant/profile:81 base recipes ×22
  states plus150 expanded inputs. States include input/output serialization,
  cache warming at three floors, cancellation/retry and four simultaneous workers
  alternating shared input and independent deserialization. Aborted numeric
  observations are explicitly discarded; no correctness during cancellation claim.
  Expanded rational/radical cases have multiplicities3/4/8 (total degree through24)
  and scales1,−7,3/7,2^-300,2^300. Each input serialization stays unchanged.
- Baseline/trial each produce1,646 known/286 Unknown results. All4,836 projective
  coefficient and4,836 output-roundtrip comparisons per variant are proved Equal.
  Across22 repeated states, candidate clears154 formerly Unknown coefficient
  comparisons and44 root-residual comparisons;132 residual comparisons remain
  unresolved. These repeat the prior three coefficient/two root improvements,
  not154/44 independent new identities. All debug/release/Memcheck rows match.
- Both state Memchecks exit97 solely for48 possible-loss bytes in std thread
  initialization, with zero definite/indirect loss and no invalid/uninitialized
  access reports. A Hyper-free scoped-thread control reproduces the same48-byte
  allocation stack and exit97. No suppression or clean-Memcheck claim. Initial
  parallel checks also produced a vgdb pipe-unlink warning in the baseline log;
  sequential --vgdb=no reruns remove that warning and reproduce the same mathematical
  rows and memory result. All original outcomes remain preserved.
- Representative stripped Hypercurve basic/arrangement examples each grow2,944
  file bytes and execute their built-in assertions successfully. text/data deltas
  are+3,008/−64 each; bss deltas−2,944/+1,152 yield loaded-total deltas0/+4,096.
  Build-path/layout effects are included: not whole-Alumina, size-optimized/LTO,
  pure logic-size attribution or timing evidence. Prior scoped CPU/allocation
  findings still apply to the unchanged implementation, including slower controls.
- monic-state-experiment.json binds102 files,28 terminal gates (including five
  expected exit97 memory outcomes),774 candidate files,11 preserved binaries
  totaling71,443,536 bytes and8 read entries. The19-checkpoint verifier checks
  exact corpus/test membership, full polynomial-directory coverage, std control,
  example sizes and all953 retained live hashes. Historical evidence is immutable.
- **No new production transfer.** The full Hypercurve downstream gate is running
  under capture tag monic-qualified-consumer-hypercurve-debug and is not credited
  by this checkpoint. Finish it and compare exact membership before the retention
  decision; then continue remaining donor support and full-inventory work.
  No production/donor edit, deletion, commit or push. Reused shared Rust target
  is now9.4GB; frozen new binary evidence is about71.4MB. /tmp reports26GB filesystem
  availability; that is not a quota guarantee. Workspace filesystem remains tight.

## 2026-09-09 matrix capability audit and retained monic normalization

- Previous goal turn classified PROGRESS: eight source reads and monic state/size
  qualification, with downstream still pending. Revalidated all19 historical
  checkpoints and953 live source/support hashes before the production transfer.
  This turn is also PROGRESS, not a whole-audit completion claim.
- Completed47 additional matrix/support files,4,517 lines: paired ownership,
  windows, pivot, FFLU, LU/recursive LU, nonsingularity, determinant/characteristic
  dispatch, Berkowitz and cofactor files; two current generic kernels; the current
  28-function test registry and paired determinant/three LU tests. Each read has
  pinned hashes, exact full-file ranges and individual notes. Combined coverage
  is577 complete/seven partial files,62,440 lines. Both inventories verify.
  Other matrix implementations/tests, generic/algebraic support and the entire
  remaining reference inventory are still open.
- Current matrix storage replaces archived row-pointer tables with a stride:
  allocation-free windows trade against coefficient-moving row swaps. Hyper's
  fixed matrices are already contiguous and dynamic solver Vec rows swap cheaply;
  no blanket storage port is selected. Donor pivots rank representation simplicity,
  certify a cheap candidate, and continue past Unknown to a known nonzero. Hyper
  already continues past Unknown; cheapest-fact scheduling remains a cost question.
- Current default determinants use small cofactors, rational backends and either
  number-field LU or Berkowitz. The recursive rank-check initial-half branch may
  return Unknown for a certified singular matrix; default number-field dispatch
  preserves this Unknown. Source-read archived differences are not archived runtime
  claims. Existing Hyper integer cross-difference/exact-quotient paths and its
  Faddeev–LeVerrier pivot-free fallback already cover substantial donor machinery.
- Native structured determinant probe passes198 cases over integer, quadratic and
  logarithmic bases,dimensions0–10,diagonal/zero-pivot/row-swap/repeated-row shapes.
  Berkowitz and Bareiss prove all198 expected results; LU has33 Unknown and default
  dispatch ten Unknown. These are incomplete decisions, not incorrect values.
  Memcheck has zero errors/all allocations freed. All28 registered donor matrix
  tests pass, but determinant tests accept Unknown and avoid default dimensions
  above4; LU tests use rank_check=0 and permissive reconstruction equality. Only
  the read constituent tests, not every executed test's source, receive coverage.
- Hyper public determinant construction succeeds on all198 structured cases and
  proves182 equalities;16 logarithmic equality comparisons remain Unknown. Every
  singular case is proved zero. Debug/release/clean focused Memcheck rows agree.
  All calls use the existing fraction-free route in this corpus, so it does not
  exercise public pivot-free fallback dispatch.
- Separate isolated method probe covers310 inputs ×three algorithms=930 queries
  per profile. Structured198 cases are supplemented by112 dense integer-scaled
  matrices, dimensions0–6, with a separate i128 permutation determinant oracle.
  Bareiss and standalone Faddeev each prove260 equalities and leave50 Unknown;
  standalone Berkowitz proves266/leaves44. Berkowitz gains eight unpermuted
  logarithmic diagonal comparisons but loses two dense logarithmic comparisons.
  No mathematical disproof occurs; debug/release/clean Memcheck rows agree.
  No production matrix change: this is neither a monotone completeness replacement
  nor a matched CPU/allocation benchmark. Alternate scheduling remains open.
- Full frozen monic Hypercurve gate finishes with1,761 passed/nine pre-existing
  ignored tests across45 suites, exactly matching baseline membership. Retained
  certified-one normalization in live Hypersolve after all preceding correctness,
  state, timing, allocation and size qualifications. Three coefficient cases and
  two root checks gain proofs; the68-known/13-unresolved split stays unchanged.
  Accepted timing ratios0.9007–1.0667 include slower controls; allocation requests
  decrease in56/162 groups without increases, and two stripped examples grow2,944
  bytes each. Completeness-first priorities justify this narrow transfer.
- Production delta: root_isolation.rs14 added/six removed lines including the
  cfg(test) inclusion, plus101 lines in root_isolation_monic_tests.rs with three
  regressions. No public API/dependency/cache-field change. Previous resultant.rs
  and Hyperreal changes are preserved. All954 live scalar/consumer source/support
  files match the frozen qualified snapshot; only those two monic paths differ
  from the previous953-file live snapshot. Downstream was not redundantly rerun
  at the live path; its entire recorded source/dependency snapshot is identical.
- Live Hypersolve has803 passing tests in debug and release, exact qualified
  membership, seven library/integration targets and an eighth zero-test doctest
  suite. Live Clippy all-targets/all-features with denied warnings, formatting,
  metadata and WASM all-feature library compilation pass. WASM execution/full CI
  remain unclaimed. Prior state Memcheck's unsuppressed48-byte std thread possible
  loss, reproduced in both variants and a Hyper-free control, remains a non-clean
  gate; clean matrix runs do not supersede it.
- matrix-pivot-experiment.json binds47 files,11 terminal gates,47 read entries and
  six binaries totaling12,399,752 bytes. retained-monic.json binds25 files,six live
  gates and954 source hashes. All21 chained verifiers pass with --monic-live.
  The first nested final capture returned0 but had empty stdout: preserved, not
  credited. Approved full-output rerun retained-monic-verify-full has all21 JSON
  records,24,525 stdout bytes,empty stderr and terminal0. Inventory verification
  likewise reran successfully after a sandbox nested-git EPERM. Historical bound
  snapshots/scripts/manifests are unchanged; old live flags keep old contracts.
- Reused the shared non-incremental Rust target and preserved about12.4MB of new
  matrix binary evidence in /tmp/calcium-matrix-pivots.X3ulwj. No donor edit,
  deletion, commit or push. The filesystem reports about26GB available in /tmp;
  availability is not a quota guarantee. Root report remains explicitly interim.
  Next: unread matrix/support source, unresolved scheduling/capability experiments,
  and all remaining ecosystem targets. No checkpoint narrows the original scope.

## 2026-09-09 matrix solve/rank source and output checkpoint

- Previous goal turn: PROGRESS, with matrix capability evidence and retained monic
  normalization. Rechecked all954 retained live source/support hashes at entry.
- Completed50 more independent full-file reads plus172 documentation lines,
  4,229 lines altogether. Includes both pinned solve/inverse/rank/RREF/kernel
  implementations, adjugate paths, seven corresponding tests at each pin and
  two full current generic triangular-solve kernels. One aggregate output was
  truncated; current rref_lu.c was reread in full before receiving coverage.
  Exact selection/ranges/notes are in matrix-solve-read-selection.json and
  coverage.json. Combined coverage is627 complete/eight partial files,66,669
  lines; both full pinned inventories verify. Full ecosystem scope is unchanged.
- Current generic triangular substitution precomputes diagonal reciprocals once
  for all RHS columns, with direct-division fallback for nonfield contexts; the
  archived classical code divides per column. Both use block recursion only
  above thresholds in both dimensions. Hyper already shares augmented fraction-
  free elimination and residual replays across RHS; inverse-node reuse and
  workload costs must be measured before a reciprocal scheduling transfer.
- Shared adjugate/determinant construction may reduce repeated Cramer work for
  multiple RHS; cofactor/characteristic-polynomial routes have different symbolic
  behavior. RREF/kernel implementations rediscover pivot positions and note that
  retaining pivot metadata could avoid repeated Unknown decisions. Hyper carries
  certified solve pivots but uses an independent all-minors affine rank diagnostic.
  Matrix polynomial evaluation, other support and remaining matrix tests stay open.
- Added1,074 native valid-input output rows:882 zero-RREF controls across all
  row/column dimensions0–6, three initial-output states, separate/whole-input
  output and default/LU/FFLU algorithms;144 adjugate and48 inverse controls across
  dimensions0–7, three coefficient fields and separate/whole-input output.
  Upper-bidiagonal closed-form inverse/adjugate coefficients are constructed from
  integer products, without calling a matrix determinant/inverse oracle. Scalar
  operations still share the donor backend; not independent scalar qualification.
- Zero-RREF default/LU returns success/rank0 but leaves seeded nonzero output
  untouched in144 rows; FFLU, fresh-zero output and whole-input controls pass.
  Order2 in-place cofactor/default adjugate has six incorrect rows, and inverse
  three; charpoly in-place controls pass. These are repeated routes/fields/shapes,
  not153 distinct defects. The explicit larger-order alias guard establishes an
  intended alias mechanism; documentation does not state a blanket alias promise,
  so in-place discrepancies are kept separate from unconditional API guarantees.
- Four higher-order quadratic cofactor rows return Unknown for determinant and
  adjugate, consistent with the already qualified minor-LU dispatch gap. They
  are completeness failures, not incorrect values. Native probe and Memcheck
  each exit1 for157 required-True failures (153 incorrect/four unresolved), with
  identical rows; Memcheck reports zero errors and all allocations freed. No
  invalid pointer, failed-rank-state or uninitialized-state probe was performed.
  Archived mechanisms are source-read only; no donor patch or external report.
- Read donor tests explain missing coverage: inverse/adjugate always use separate
  output; RREF separate output starts zero and alias mode starts from input.
  Equality checks accept Unknown and many assertions are conditional on success.
  The existing28-function native gate remains tied to the identical pinned code;
  it does not negate the new independent output controls or credit unread files.
- Isolated rank candidate changes only hypersolve/src/rank.rs in a774-file
  consumer snapshot. It continues past Unknown minors at the same order, returns
  on a proved nonzero witness, and refuses to descend if no witness resolves that
  order. Thus it does not infer a smaller rank from an undecided larger minor.
  Live production remains the unchanged954-file retained monic snapshot.
- Public rank probe has828 queries per variant/profile across three precision
  floors, rational/radical/logarithmic/tiny-positive/Pythagorean-zero/literal-zero
  entries, row/column permutations, two-row full-rank witnesses and unresolved
  higher-order controls. Baseline606 certified/222 Undecided becomes810/18.
  All204 additional decisions have expected coefficient/augmented rank and DOF;
  all18 unresolved controls are preserved. These repeat witness patterns across
  positions/floors, not204 independent identities. Scalar states are prechecked
  and shared within each kind/floor; not arbitrary-state or cold-start coverage.
- Debug/release/standalone Memcheck rows agree for each variant. Both Hyper
  Memchecks have zero errors/no definite,indirect or possible losses;2,272 bytes
  remain reachable in this focused run. Candidate existing803 tests pass in both
  profiles with exact baseline membership and eight suite summaries including
  an empty doctest. Formatting passes. No new in-crate tests or production edit.
- **Rank candidate remains pending, not retained or rejected.** Next qualification
  must measure the cost of scanning many unresolved minors, then address state/
  serde/concurrency, independent larger-matrix controls, focused regressions,
  downstream use, Clippy/WASM and representative size as warranted. No matched
  benchmark or speed/allocation benefit is claimed at this checkpoint.
- matrix-solve-experiment.json binds50 source/support/evidence files,12 terminal
  gates,51 read entries,774 candidate source hashes and five binaries totaling
  14,316,952 bytes. All22 chained verifiers pass with --monic-live. Full-output
  capture matrix-solve-verify-full contains22 JSON records/25,983 stdout bytes,
  empty stderr and terminal0; both inventories pass through the approved nested-
  git environment. Historical bound evidence, failures and prior costs remain
  immutable. This goal turn is PROGRESS; the entire objective remains active.
- Reused the shared non-incremental target, now9.8GB, and preserved about14.3MB of
  new binary evidence in /tmp/calcium-matrix-solves.Pl5RBm. About26GB filesystem
  availability remains in /tmp, not a quota guarantee. No production/donor edit,
  deletion, commit or push. Continue all unread reference targets and unresolved
  transfers; this checkpoint does not redefine audit completion.

## 2026-09-09 rank cost and matrix support checkpoint

- Previous goal turn classified PROGRESS:50 completed reads, native output
  controls and verified isolated rank improvements. Rechecked 954 live and 774
  candidate source hashes before starting this work. No production change.
- Measured immutable rank v1 with separate uninstrumented CPU and requested-
  allocation builds, both terminal 0. Seven patterns distinguish newly certified
  witnesses, same-outcome unresolved rows/minors, rational controls and exact-zero
  controls over widths 4,8,16,32 and fresh-problem/retained-analysis lifecycles.
  Both share a prechecked tiny-positive scalar and eight warmups; fresh_problem
  rebuilds/analyzes the Problem, not cold scalar construction or process timing.
- CPU has 2,688 observations in 56 groups,12 alternating ABBA blocks/group on CPU 6,
  adaptive common iteration count, paired median ratios and 5,000 deterministic
  bootstrap resamples/group. Per-group intervals are not multiplicity-adjusted.
  CPU window 02:32:05.395–02:32:34.688 UTC overlaps no recorded build/test/memory
  gate; light source reads ran separately. Allocation 336 observations follow at
  02:32:57.743–02:33:01.108, three 16-query pairs/group. Allocation clocks are not
  CPU evidence. Complete raw streams, pilots, dates and manifests are preserved.
- Retained-analysis width 32 still-unresolved row and two-row cases have paired
  ratios 7.9745 and 48.0067. The latter 95% bootstrap interval is 42.7626–50.0205;
  requested bytes/query rise 47,320→1,141,240 and requests 527→5,597. The timing
  ratio is median paired ratios, not ratio of pooled median clocks. New one-/
  two-row decisions cost 14.7715×/31.0499× baseline Unknown and are different work,
  not equal-work regression/speedup comparisons. Known-zero controls also slow
  (1.0783–1.2180); other known controls span 0.9877–1.1177.
- Allocation requests/bytes are higher in 32 groups, equal in 24, lower in none;
  peaks higher in 24/equal in 32, with zero measured live deltas. Worst requested-
  bytes ratio 25.0965 is a newly certified witness group. Counters measure requested
  Rust bytes/calls, not native/allocator overhead, RSS or stacks. Driver binary
  deltas−312 CPU/−296 allocation bytes and text−416/bss+416 are build-path/layout-
  inclusive, not application-size or pure code-size wins.
- **Rank v1 not selected as implemented.** Its 204 exact-decision gains remain
  credible; exhaustive unresolved scans need better scheduling/reuse before
  retention. The mathematical idea and full original completeness requirements
  remain open, with no arbitrary search cap or corpus-based scope reduction.
  No v2, production rank edit, broader state/serde/concurrency, focused in-crate
  regressions, downstream, Clippy or WASM qualification in this checkpoint.
- Completed 40 independent full donor files/2,521 lines: paired multiplication,
  powers, matrix polynomial evaluation, three related tests, scalar-matrix
  operations, predicates, transpose and constructors. Shared denominator/common
  field dispatch and Paterson–Stockmeyer evaluation offer scheduling/workspace
  ideas. Donor polynomial evaluation allocates an unused t matrix, but no matching
  Hyper defect is inferred. Borrowed mutable C headers are not an ownership port.
- Checked Hyperlattice core.rs 1338–1448,1740–1820,5300–5360,5905–5985: fixed 3/4
  right division already shares adjugate, determinant and inverse with exact-
  rational gating; matrix powers already specialize small powers/repeated
  squaring. Thus no blanket missing shared-adjugate or power capability is claimed.
  Source-range checks refer to the unchanged retained snapshot, not all of core.rs.
- Native valid-input support probe passes 1,528 required-True rows:408 products,
  560 powers,560 matrix polynomials. Empty/rectangular/square dimensions 0–8,
  initialized nonzero separate outputs and valid whole-input aliases are sampled.
  Rational, quadratic and logarithmic fields plus large denominators around the
  common-field cutoff are included; no full branch-coverage claim. Whole-input
  classical multiplication can delegate to default multiplication, and the both-
  inputs-alias case necessarily squares one input.
- Integer dot coefficients and Jordan-block binomial closed forms avoid donor
  matrix multiplication/power/polynomial-evaluation oracles but share ca scalar
  operations. Powers/lengths 0,1,2,3,4,7,8,15,16,31 are sampled. Native and Memcheck
  outputs are identical, exit 0, zero memory errors/all allocations freed
  (739,893 allocs/frees,37,263,684 cumulative bytes). These new clean gates do not
  erase earlier mathematical output failures or the Rust-thread 48-byte possible
  loss. Archived counterparts are independently read, not executed.
- Exact reads/notes are bound in rank-cost-read-selection.json/coverage.json:
  combined 667 complete/eight partial files,69,190 read lines. Both full pinned
  inventories verify. Matrix/algebraic/generic support, entire remaining ecosystem
  and earlier unresolved transfers stay in scope; no completion claim.
- rank-cost-experiment.json binds 41 source/support/evidence files,seven terminal
  gates,40 read entries and five binaries totaling 10,956,864 bytes. All 23 chained
  verifiers pass with --monic-live, recomputing every cost summary and native row,
  while checking 954 live source/support hashes against retained monic. Earlier
  774-file rank candidate is frozen unchanged. Full output, old failures and
  all historical evidence are preserved; this goal turn is PROGRESS.
- Reused 9.8 GB shared non-incremental build target and preserved about 11 MB binary
  evidence in /tmp/calcium-rank-costs.lbsszv. About 26 GB /tmp filesystem availability
  remains, not a quota guarantee. No production/donor edit, deletion, commit or
  push. Root summary remains explicitly interim. Continue remaining support reads
  and qualify better rank scheduling independently before any transfer.

## 2026-09-09 matrix solve/characteristic certificates and scalar domains — checkpoint 24

- Previous goal turn classified PROGRESS: 40 complete reads, 1,528 required-True
  native controls and verified rank-cost evidence. Rechecked all 954 retained
  live source/support hashes before this pass. Rank v1 remains isolated and
  unselected; full ecosystem scope and unresolved transfer ideas remain open.
- Completed 32 independent full-file reads: both pins' remaining triangular/
  nonsingular solve tests, characteristic/Danilevsky tests and implementation,
  companion, eigenvalue and diagonalization paths/tests; current generic
  Danilevsky, full 1,831-line generic CA wrapper, its test registration and vector
  header. Partial generic vector/dot/status and domain documentation reads are
  explicitly bounded. The initial truncated vector-header read was repaired;
  reread ca_arg/phase documentation was already credited and is not counted again.
- This adds 5,913 actual read lines. Archived Calcium now has 347 complete/one
  partial files, 34,875 lines; current FLINT 352 complete/ten partial, 40,228 lines.
  Combined 699 complete/11 partial files, 75,103 read lines. Exact source hashes,
  ranges and per-file notes are recorded, including the previous generic partial
  entry before extension. Entire donor/support and ecosystem scope remains open.
- Danilevsky's division/pivot requirements, block splits, generic status handling
  and dot temporary ordering contrast with division-free construction. The CA
  method table has no vector-dot override, so generic strided dot is a scalar
  loop, not a specialized joint CA dot. Unused donor temporaries are observations,
  not measured Hyper wins. Hyper already has pivot-free Faddeev–LeVerrier and
  algebraic-fiber Berkowitz support. No missing global determinant capability or
  worthwhile new workspace/ownership transfer is established.
- All 4,464 native solve controls pass: 3,024 default/classical/recursive lower/
  upper triangular rows, including ignored unit-diagonal storage and whole RHS
  aliases; 1,440 default/LU/FFLU/adjugate solve rows, with 768 nonsingular successes
  and 672 required singular failures. Dimensions include empty and threshold-size
  shapes, multiple RHS counts, rational 2/3, sqrt(2) and log(2). Initialized
  separate outputs and independent integer/scalar RHS recipes avoid same-backend
  matrix multiplication as the oracle. Failed outputs are never inspected.
- All 432 default/Danilevsky characteristic-coefficient controls pass. Dimensions
  0,1,2,3,4,5,6,8,10; known diagonal/triangular and explicit dense similar matrices,
  reversed basis, fresh or length-17 seeded output, three scalar fields. Integer
  product-of-linear-factors recurrence supplies every expected coefficient and
  required output length. Avoids tested charpoly/determinant/matrix polynomial
  oracles, but shares scalar arithmetic. Sampled corpus, not full branch coverage.
- Both native matrix probes match their Memcheck rows and exit zero, zero errors/
  all blocks freed: solve 2,499,970 allocations/frees, 70,612,552 cumulative bytes;
  characteristic 377,500 allocations/frees, 7,215,480 bytes. These are correctness
  and memory-safety controls, not matched performance/allocation improvement
  evidence. Archived mechanisms were independently read but not executed.
- New generic CA type/property findings: 580 valid rational-input rows record
  12 successful non-real asin/acos results in the RR context and two incorrect
  real-vector-space flags for AA/QQbar. A further 18 successful algebraic-context
  Arg results have Unknown algebraicity predicates. Independent exact equality
  to −pi resolves their actual value as transcendental; Unknown was not converted
  into False. All operation outputs are inspected only after GR_SUCCESS.
- Across four contexts, 36 negative-rational Arg rows fail the mathematically
  correct +pi principal oracle and match −pi in a separate diagnostic. This
  reproduces checkpoint six's documented (−pi,pi] phase defect, not a fresh count
  of distinct bugs. Three negative rational values and three initialized/alias
  lifecycles repeat the mechanisms. The failed +pi witness and supplemental −pi
  witness are both preserved. No change to the required principal convention.
- Domain and +pi witness gates exit one for mathematical failures; Memcheck
  nevertheless has zero errors/all blocks freed. The −pi diagnostic exits zero,
  also clean. Initial domain compile failure used nonexistent ca_ctx_ptr; original
  source and error output are frozen separately from the corrected pointer type.
  No invalid pointer, failed-state access or donor mutation was attempted.
- Existing Hyper public asin/acos guards pass 102 rows per debug/release profile:
  exact/near rational boundaries, radicals and ±pi across cloned, serde and
  explicitly warmed inputs. In-domain outputs refine to −80 bits; out-of-domain
  values return NotANumber. Shared originals are not guaranteed cold; this is
  domain/refinability testing, not an independent numerical accuracy oracle.
  Release Memcheck matches all rows, zero errors/no lost blocks, 3,112 bytes in
  28 reachable blocks. This does not supersede old Rust-thread possible losses.
- charpoly-domain-experiment.json binds 72 source/support/evidence files, 19
  terminal gates (including expected failures), 36 read entries and seven binary
  artifacts totaling 6,203,896 bytes. All 24 chained verifiers pass with
  --monic-live, preserving known failure counts and verifying all 954 retained
  live files. Full capture charpoly-domain-verify-full has 24 JSON records,
  29,455 stdout bytes, empty stderr and terminal zero. Both pinned inventories
  verify in charpoly-domain-inventory (two records, 388 bytes, empty stderr).
- No new production candidate, performance/size claim or transfer. Rank v1 is
  still isolated, immutable and unselected as implemented; better scheduling
  remains open. Root report updated and explicitly interim. Reused the 9.8 GB
  build target, with 6.2 MB extra binary evidence in
  /tmp/calcium-charpoly-solves.lbZlN8. About 26 GB /tmp filesystem availability
  remains, not a quota guarantee. No deletion, production/donor edit, commit or
  push. This checkpoint is PROGRESS, not completion or a scope reduction.
- Next: remaining matrix spectral/Jordan and algebraic/generic supporting code,
  remaining original references and unresolved transfer experiments. Reconcile
  the full inventory before any final-completion claim.

## 2026-09-09 matrix spectral/Jordan source closure — checkpoint 25

- Previous goal turn classified PROGRESS: checkpoint 24 adds 32 complete reads,
  exact matrix/domain controls and frozen failure witnesses. Rechecked its 72
  bound files and all 954 retained live source/support hashes before proceeding.
  Rank v1 remains unselected and immutable; full ecosystem scope is unchanged.
- Completed the 27 remaining ca_mat files independently at each pin, plus nine
  current generic matrix implementations: 63 complete reads. Both directories
  now close at 107 archived files/8,834 lines and 109 current files/7,486 lines,
  including tests. This is directory source closure, not all recursively called
  generic/rational/algebraic/LLL support or full ecosystem completion.
- With supporting documentation, method-registration and LLL-tail ranges, this
  pass adds 4,935 lines. Effective cumulative coverage: archived 374 complete/one
  partial, 37,085 lines; current 388 complete/12 partial, 42,953 lines. Combined
  762 complete/13 partial files, 80,038 lines. Source hashes and ranges verified.
- Added an append-only coverage-extensions.json because checkpoint 22 binds its
  earlier partial ca_mat documentation record exactly. New 515..624 reads are
  recorded without changing that historical entry. effective-coverage.mjs unions
  base/extension ranges without duplicate line credit; the historical inventory
  reporter still reports base-only totals (110 fewer lines). Current generic
  registration and new partial documentation/LLL reads remain in coverage.json.
- Jordan blocks arise from eigenvalue multiplicities and Ferrers partitions of
  rank-power differences. Transformation uses generalized-kernel differences,
  RREF residual witnesses and reversed chains. Current shared function evaluator
  computes one maximal normalized jet per distinct eigenvalue, reusing prefixes
  across blocks; log inverse powers use a balanced multiplication schedule.
  Context/mutable ownership layouts and spectral-domain limitations do not justify
  replacing Hyper's scalar representation or adding an unqualified generic API.
- Hyper comparison read exact integer Newton/falling-factorial interpolation,
  derivative Horner/endpoint factorial handling and fixed-size matrix schedules.
  Existing exact recurrences and zero tails already avoid finite-word factorial
  truncation. Bounding active derivative orders in the remaining general Horner
  loops is a concrete follow-up idea, not yet implemented/benchmarked. Keep it in
  scope alongside better rank scheduling and previous unresolved transfers.
- Test audit: known-block jordan_form test builds dense similar B but computes
  on original A. Its inverse/equality checks accept Unknown and are conditional
  on success. The Jordan-block rational family does require success; generic
  cases remain conditional. DFT requires True in square inverse products 0..16.
  Matrix exp/log tests use dimensions 0..2 and conditional same-backend identities,
  not independent high-order jets or mandatory branch/domain coverage.
- All 486 new Jordan controls pass: three scalar shifts (2/3, sqrt(2), log(2)),
  nine block patterns through dimension six, direct/dense/reversed similarities,
  six routes including both supported input aliases, null P and precomputed
  metadata. Require exact block multisets; the 324 P-producing routes also require
  direct-sum AP=PJ and exact rational determinant nonzero. Inputs and chain sums
  avoid CA matrix construction/multiply/inverse/determinant oracles; scalar and
  rational arithmetic remain shared. No full branch/arbitrary-state claim.
- Jordan native/Memcheck rows agree, exit zero, zero errors/all blocks freed:
  1,092,304 allocations/frees, 54,643,520 cumulative allocated bytes. Current FLINT
  executed; archived code independently read only. These are numerical and memory
  correctness controls, not a matched performance campaign.
- Native matrix exp/log has 648 rows over six shifts, the same patterns/shapes,
  separate/whole-alias output and direct finite Jordan coefficient formulas.
  All existence statuses match: 66 correctly rejected singular logarithms and
  582 successes. Of the latter, 568 exact equalities are proved and 14 remain
  Unknown in the three-distinct-eigenvalue pattern with quadratic/log shifts.
  No incorrect value observed. Shared scalar functions and exp factorial
  recurrence limit oracle independence; Unknown is not silently accepted as True.
- Function Memcheck terminates SIGABRT in fmpz_lll_is_reduced_d's exact reduced-
  basis assertion while CA discovers field relations. The 4,096-byte stdout is
  only a buffered native-prefix match: 168 complete rows plus a fragment, not
  an exact localization or full run. Last validation tail 765..821 read, but
  preceding floating-bound/setup and exact-validation support remain open.
  Native run finishes; root cause/environment sensitivity and native reproduction
  are not established. No donor assertion or relation check was disabled.
- Aborted Memcheck has no invalid-access/uninitialized-use diagnostic, but 15
  possible-loss contexts: 136,296 bytes/4,065 blocks, plus 120,712 reachable bytes.
  These are post-abort observations, not clean memory evidence or proof of a
  normal-exit leak. Original logs and 78,024,704-byte generated crash dump retained.
  Dump moved intact from comparison root to results/matrix-function-memcheck.vgcore
  in the workspace; no minimized memory-failure reproducer or donor mutation.
- spectral-experiment.json binds 28 source/support/evidence files, six terminal
  gates including failures, 66 read entries, one extension and two binaries
  totaling 40,736 bytes, plus the crash dump. All 25 chained verifiers pass with
  --monic-live and all 954 retained live files unchanged. Both complete pinned
  inventories and effective cumulative coverage verify. Evidence verification
  preserves the aborted run; it does not declare all numerical/memory gates green.
- No production candidate/change, CPU/allocation/binary-improvement claim or
  donor edit. Rank v1 remains immutable and unselected; shared build targets were
  not rebuilt. About 41 KB additional /tmp binary evidence in
  /tmp/calcium-spectral.nqRkRT, with the 78 MB crash dump on the workspace filesystem.
  About 26 GB /tmp availability remains, not a quota guarantee. No deletion,
  commit or push. Root report remains explicitly interim. This turn is PROGRESS.
- Next: audit remaining LLL/field-relation validation support; independently
  qualify demand-bounded Hyper derivative work and better rank scheduling; continue
  every remaining original reference and reconcile the full inventory before
  any ecosystem completion statement.

## 2026-09-09 derivative-demand costs / LLL support — checkpoint 26

- Previous turn classified PROGRESS: checkpoint 25 closes both ca_mat source
  directories, records 486 passing Jordan controls and preserves the aborted
  matrix-function memory run. Rechecked its 28 bound files and all 954 retained
  live source/support files before this pass. Entire ecosystem scope stays open.
- Qualifying an active-order bound in Hypercurve polynomial derivative Horner
  loops: retain all requested output entries, but omit arithmetic on derivative
  orders above the degree of the polynomial prefix processed so far. Rational
  quotient derivatives are not truncated. Candidate stays isolated until exact
  oracles, matched costs and proportional regression gates justify retention.
- Continuing the full floating LLL validation implementation and relevant exact
  support. The earlier instrumented assertion is not presumed a native failure,
  and its crash evidence remains intact.
- Completed the remaining 764 lines of is_reduced_d.c and 14 complete supporting
  files: nfloat validator, exact/generic reduced-basis checks, dispatch/context
  setup, matrix float imports, integer/limb exports, double matrix kernels and
  tests. New unique reads total 2,980 lines. Effective cumulative coverage is
  777 complete/12 partial files, 83,018 lines: archived 374 complete/one partial,
  37,085 lines; current 403 complete/11 partial, 45,933 lines. These are Calcium/
  FLINT continuation totals, not the complete ecosystem inventory.
- The function named is_reduced_mpfr now uses nfloat nearest/floor/ceil contexts,
  not MPFR arithmetic. The binary64 validator changes and restores caller FENV;
  its imports reject values beyond DBL_MAX but do not certify exact conversion.
  Larger integer-to-double conversion truncates toward zero independently of
  hardware rounding. Generic/exact checking keeps operation failure separate
  from True/False/Unknown. Called nfloat arithmetic/rounding proofs and broader
  field-relation support remain open; no root cause of the previous abort is
  established, and no assertion or relation check was changed.
- All 3,360 numerical LLL controls pass: dimensions 0..6, four closed-form
  reduced/non-reduced basis patterns, scales 1 through 2^1200, basis/Gram inputs,
  four caller rounding modes and three validation routes. Explicit integer
  Gram dots and known size/Lovasz conditions supply the expected answers.
  Binary64 leaves 80 reduced cases uncertified; the nfloat and complete
  dispatcher decide all 720 reduced and 400 non-reduced cases each. Every call
  preserves its caller rounding mode. These are simple numerical controls, not
  a reproduction or resolution of the earlier field-relation assertion.
- LLL native/Memcheck rows match, zero errors/all allocations freed: 179,867
  allocations/frees and 42,930,504 cumulative requested bytes. This clean control
  does not supersede the failed matrix-function Memcheck or its retained core.
- Isolated derivative-demand-baseline/candidate copy only Hypercurve's 354
  retained source/support files each; five dependencies reference immutable
  qualified snapshots. Added identical 170-line regression modules to both.
  The sole candidate production difference is two active-order Horner bounds,
  six added/four removed lines. Every requested output and all rational quotient
  orders remain intact. Source validator checks 955 files per variant and exact
  allowed edits; all 954 retained live files remain unchanged.
- Three new tests pass in each variant and debug/release profile: independent
  BigRational monomial sums across empty/dense/sparse/leading-zero inputs,
  rational/pi/sqrt(2)/log(2) scales and derivatives through order 128; exact
  rational-curve quotient formulas, public/private/endpoint routes and clone
  reuse; checked capacity overflow and exact zero tails. The initial candidate
  compilation failed on a test-only rational constructor typo; original source
  and terminal failure are preserved, and the corrected harness passes. These
  are default-feature focused tests, not full candidate consumer qualification.
- CPU campaign: 3,840 observations in 80 groups, 12 alternating ABBA blocks,
  pinned to CPU 6; two curve families, degrees 1/3/8/24, orders 0/1/3/24/128 and
  retained/fresh-curve lifecycles. Each process independently prechecks public
  results, then warms eight queries. Parameter is 1/3; fresh construction clones
  prebuilt controls/weights, not cold scalar construction. No endpoint-helper
  CPU claim. Recorded builds/Memchecks do not overlap the timing campaign.
- Paired CPU ratios range 0.146542..1.058681. Per-group bootstrap intervals are
  below one in 55 groups and above one in one low-order control: pi-scaled
  degree-one/order-zero retained curve, ratio 1.042047, interval
  1.016616..1.222695. These intervals are not multiplicity-adjusted or universal.
  The selected degree-eight/order-128 retained polynomial falls from 155.317 to
  22.623 microseconds/query. No blanket speedup or workload-frequency claim.
- Separate allocation campaign has 480 observations/80 groups/three pairs:
  requests/bytes decrease in 16 groups, equal in 64, increase in none. The
  selected degree-eight case falls from 1,733 requests/223,984 bytes to five
  requests/44,272 bytes per query. Measured peak/live deltas are identical in all
  groups. Fourteen groups still retain bytes after eight measured queries, up to
  126,728 bytes in both variants; warming is not a steady-state/leak diagnosis.
  Counters measure requested Rust bytes, not RSS or allocator/native overhead.
- Matching high-order pi-scaled public Memchecks pass with zero errors or lost
  blocks; each retains 264,184 reachable bytes in 2,137 blocks. Candidate format
  check passes. Linked probe files are 112/88 bytes smaller but text decreases
  316 bytes and BSS increases 320 bytes each; this is not representative stripped
  application-size qualification. Broader state, endpoint-consumer, full-feature/
  regression, Clippy, WASM compilation and representative size gates remain.
- Candidate remains immutable and isolated, promising but not retained. Full
  consumer qualification is the next step before any production transfer; the
  measured low-order control and shared retained-state growth stay explicit.
  Rank v1 remains unselected. No production/donor edits, deletion, commit or push.
- derivative-demand-experiment.json binds 70 source/support/evidence files,
  15 terminal gates (including the initial harness compilation failure), 15
  source-read records, both 955-file snapshots and five binaries/14,802,448 bytes.
  All 26 chained verifiers pass with --monic-live; both full pinned inventories
  and deduplicated coverage verify. Verification preserves prior failures.
  Full verification records 26 JSON lines/33,868 bytes, empty stderr and exit 0;
  inventory records two lines/388 bytes, and effective coverage one line/159 bytes.
- Reused the shared Rust target (now about 11 GB); new dedicated /tmp binaries
  occupy about 14.8 MB in /tmp/calcium-derivative-costs.FA5c2M. The two 23.8 MB
  source copies are in the workspace. About 25 GB /tmp availability remains,
  not a quota guarantee. Original snapshots, failed logs and crash dump retained.
  Root report remains explicitly interim. This turn is PROGRESS, not completion.
- Next: full derivative candidate consumer/state/size qualification and retention
  decision; remaining nfloat/field-relation validation support and every original
  reference still open; reconcile the entire inventory before final completion.

## 2026-09-09 derivative consumer/state qualification — checkpoint 27 retained

- Previous goal turn classified PROGRESS. Revalidated checkpoint 26's 70 bound
  files and all 954 retained live source/support hashes before this pass.
- Retained the exact isolated v1 into live Hypercurve after qualification:
  two active-order Horner bounds, algorithm +6/-4 lines, plus four test-module
  registration lines and the existing qualified 170-line module. Every requested
  polynomial output and rational quotient order remains intact. No other live
  source changes; all 955 source/support hashes equal the frozen candidate.
- Complete candidate all-feature library/integration suite passed in 1,550.717
  seconds including compilation: 1,764 passed, nine pre-existing ignored, 45
  suites. All 1,770 previous named outcomes remain, with exactly three added
  passing regressions. Live focused default-feature debug/release tests and
  formatting pass. All-feature/all-target Clippy passes. No full release
  consumer-suite or full CI qualification is claimed.
- State probe: 72 cases from four scalar scales, three rational parameters and
  six requested orders, each with 22 fresh/serialized/warm/abort-recovery/four-
  worker states. All 1,584 exact quotient-formula queries per variant/profile/
  Memcheck run agree, including output serialization and unchanged input encoding.
  Ten additional invalid parameter, capacity and pole requests are rejected.
  The cancellation observation is discarded; only recovery is checked, not
  guaranteed interruption or exact error-variant identity.
- Both state Memchecks return 97 for one 48-byte possible loss in
  std::thread::thread::Thread -> std::thread::current::init_current, the same
  allocation stack already present in the independent Rust thread control.
  No definite/indirect losses; 347,608 bytes remain reachable in 3,357 blocks.
  These are preserved failing memory gates, not a clean/all-freed claim.
- All-feature WASM fails in both variants in getrandom 0.4.2. Recorded inverse
  dependency tree reaches the optional comparative-benchmarks curvo/rand path.
  Separate checks with dispatch-trace,triangulation,svg,hershey pass in both
  variants. No dependency feature was modified, failure suppressed, all-feature
  success asserted or WASM execution claimed.
- Endpoint consumer uses known four-fragment closed-square rational curves
  with small inward control bends, degrees 1/3/8/24 and rational/pi scales.
  Retained/fresh graph lifecycles use prebuilt shared scalar controls. Every
  public tangent-order traversal must certify one closed chain [0,1,2,3].
  Source review confirms end jets request order one and start jets order three;
  the latter uses the unchanged at-zero specialization. This is a complete
  low-order consumer measurement, not an isolated high-order endpoint benchmark.
- Endpoint CPU: 768 observations, 16 groups, 12 alternating ABBA blocks on CPU 6
  after an exact precheck and eight warm queries. Ratios 0.912326..1.012088;
  five per-group intervals below one, none above, eleven straddling. Intervals
  are not multiplicity-adjusted. Initial harness compilation failed on mistaken
  public classification handling; original source and both failures retained,
  corrected builds used. All builds/state memory checks finish before CPU timing.
- Endpoint allocation: 96 observations, three pairs/group, eight measured
  queries. Request/byte/peak/live counts exactly match between variants in all
  16 groups; one shared nonzero live delta remains. Focused degree-24 pi-scaled
  fresh-graph Memchecks pass with zero errors/losses and 63,176 reachable bytes
  each. No arbitrary-state or steady-state endpoint memory claim.
- Generic-query retention staircase: 320 observations, two repeats of each
  variant across 16 groups and 1/8/32/128/512 measured queries. Counters repeat
  exactly. Peak/live match between variants and plateau by 32 through 512 in
  every group; largest shared live delta 196,376 bytes. Request/byte incremental
  rates match over 32->128 and 128->512 per variant. This diagnostic overlaps
  consumer testing and is not CPU evidence, RSS or a general retention bound.
- Re-read Hyper rational weak-key/strong-result write-once product caches and
  bounded small-number tables. They explain why reachable shared state warrants
  explicit measurement, not an inference of either a leak or globally bounded
  transitive storage. No cache implementation was changed.
- Representative default-release basic/arrangement examples both pass assertions.
  Stripped file sizes grow 224/208 bytes; text +252/+228, data -32/-40, BSS
  -224/-200. Unstripped files each grow 304 bytes. These are matched local
  examples, not full Alumina/LTO/all-feature binaries. Large high-order request/
  CPU savings and preserved exact outputs justify these small costs. The
  earlier degree-one/order-zero slowdown remains explicit; no universal win.
- Actual reference reads: nfloat/ctx.c 238, nfloat/mat.c 171,
  nfloat/test/t-nfloat_directed.c 364, nfloat.h 570, nfloat/nfloat.c 1..310.
  Five new records/four complete files add 1,653 lines. Effective coverage:
  Calcium 375 reviewed/374 complete/one partial/37,085 lines; FLINT 419 reviewed/
  407 complete/12 partial/47,586 lines; total 781 complete/13 partial/84,671 lines.
  Fixed-limb storage and precision-dependent dispatch are not replacements for
  the exact scalar tower. Unflagged MPFR/ARF adapter modes truncate toward zero;
  checkpoint 26's shorthand "nearest" for that context is corrected here.
  Experimental directed arithmetic, status-conditional tests, suspicious overflow macro and remaining
  conversion/dot/matrix support stay open. No new donor numerical claim or patch.
- derivative-qualification-experiment.json binds 150 evidence/support files,
  40 terminal gates (34 successful and six preserved failures), five read
  records, two 955-file snapshots and 14 binaries/148,651,888 bytes. Draft
  verification passed before live edits. Final 27 chained verifiers pass with
  --derivative-live, 27 JSON lines/37,650 stdout bytes, empty stderr, exit 0,
  2026-09-09 15:54:23.459Z..15:54:33.156Z. Both full pinned inventories verify
  (two lines/388 bytes); deduplicated coverage verifies (one line/159 bytes).
- New dedicated snapshots use about 148.7 MB in
  /tmp/calcium-derivative-apps.5K06wk and /tmp/calcium-derivative-endpoint.XO0A0Q.
  Shared Rust target reused, about 13 GB; about 23 GB /tmp availability remains,
  not a quota guarantee. No cleanup, donor edit, commit or push. Original
  snapshots/failures/core preserved; root report stays explicitly interim.
- This goal turn is PROGRESS, not completion. Next: nfloat scalar implementation
  from line 311 and called conversion/dot/matrix/field-relation support, remaining
  source and documents in both pinned inventories, unresolved transfer candidates,
  and every original ecosystem target. Rank v1 remains isolated/unselected.

## 2026-09-09 checkpoint 28 — nfloat arithmetic/conversion support

- Previous goal turn classified PROGRESS. Revalidated all 150 checkpoint-27
  bound files and all 955 retained live source/support hashes before continuing.
- Actual donor reads: nfloat/nfloat.c 311..3795 completes the file while the
  frozen 1..310 record is preserved; nfloat/test/t-nfloat.c 271 lines,
  doc/source/nfloat.rst 563, nfloat/dot.c 1187, nfloat/mat_mul.c 1852,
  gr_mat/mul_classical.c 115, gr_mat.h 1..95. Seven records add 7,568 lines,
  six newly completed files and one new partial header. The old scalar partial
  is extended through coverage-extensions.json, not overwritten.
- Effective coverage: Calcium 375 reviewed/374 complete/one partial/37,085
  lines; FLINT 425 reviewed/413 complete/12 partial/55,154 lines. Combined
  800 reviewed/787 complete/13 partial/92,239 unique read lines. This is the
  continuation's source coverage, not the full ecosystem total or completion.
- Contracts: unflagged fixed-precision arithmetic permits arbitrary rounding,
  typically within 1-2 ULP, not a certified outward interval. Directed modes
  are experimental and have an explicit limited operation list. Parsing,
  generic transcendental adapters, most mixed operations and complex arithmetic
  are documented exclusions. Underflow flushing is explicitly warned about.
  An ignored conversion status and the earlier suspicious overflow macro remain
  source-level boundary concerns, not qualified numerical defect reproductions.
- Real dot products choose a common exponent and bounded accumulator, with
  conservative directed error padding. This promises direction, not bitwise
  correctly rounded equality. Public directed matrix multiplication always
  dispatches classical; fixed-point/error-bound and block-fmpz speed paths are
  not covered by the directed matrix controls. Complex three-product kernels,
  fixed-point support, huge-length 32-bit concerns and called support stay open.
- Hyper reads bound to the unchanged 955-file candidate: format_parse.rs 1..106,
  facts.rs 1360..1430; rational aggregate_products.rs 1..240, 699..890,
  3335..3665, 5580..5680; its tests.rs 1..70 and 3930..4088. Checked word/six-
  limb exact accumulation with arbitrary-precision fallback already supplies
  the useful bounded-local-storage pattern without discarding exact terms.
  Exact rational parsing, lazy scientific powers after resource exhaustion,
  and rational/abort-aware certified dyadic intervals retain stronger contracts.
- New 273-line native driver compares documented finite directed operations
  with independently decoded exact GMP rationals. Scalar inputs are checked
  exactly representable. Dots/matrices use the actual represented input values
  in explicit rational sums. Root/reciprocal-root direction is checked by exact
  squared inequalities and nonnegative output. Failed outputs are not inspected.
- All 44,574 native cases pass, with 74,922 exact comparisons: 31,680 scalar
  cases, 5,280 conversions, 6,912 dots, and 702 matrices/31,050 entries. Scalar
  and conversion controls use all 66 native word precisions from 64..4224 bits,
  eight patterns and both rounding directions. Ten scalar operations and three
  whole-output alias routes; five conversion kinds including cross precision.
  Four caller FENV modes are distributed among patterns and always preserved,
  not tested as an independent full Cartesian product.
- Dot/matrix controls use limb counts 1/2/3/4/5/8/16/32/66. Dots cover lengths
  0/1/2/3/8/17, sparse/sign-cancelling/exponent-gap patterns, initial/subtract/
  reverse toggles and initial-output alias. Empty reverse-dot vectors use
  padded valid storage. Matrix shapes include zero inner dimension, rectangular
  cases and whole A/B aliases on square cases. No invalid partial overlap or
  nonfinite/boundary/failed-output/field-relation assertion reproduction.
- Memcheck repeats the identical output and passes: zero errors, zero live
  bytes/blocks, 595,961 allocations and frees, 299,063,144 cumulative requested
  bytes. This is not peak RSS, a Hyper allocation comparison, a 32-bit result,
  arbitrary-random-input validation, or a fix for the earlier field-relation
  abort. The original abort logs/core and all earlier failing memory gates remain.
- Existing Hyperreal dyadic dot word/stack alignment/borrow/fallback regressions
  pass again: three tests per debug/release profile, 691 filtered, none ignored.
  These add no new production tests and are not a new full-suite qualification.
- Preserved initial driver compile failure: arf.h must precede nfloat.h for
  its conditional ARF declaration. The original source is retained alongside
  the corrected source; its independent finite-decoder bound was also enlarged
  before any run to admit the intended benign exponent-gap inputs. No donor
  header changed. Nested-sandbox inventory run failed with spawnSync git EPERM;
  escalated read-only retry verified both complete pinned inventories.
- nfloat-support-experiment.json binds 39 support/evidence files, nine terminal
  gates (seven successful, two preserved infrastructure/harness failures), seven
  read records, the unchanged 955-file source maps, the 27,928-byte executable
  and linked libraries. No new production/donor change or performance claim.
  No backend replacement benchmark is justified across these unequal contracts.
- All 28 chained verifiers pass with --derivative-live: 28 JSON lines/39,741
  stdout bytes, empty stderr, exit 0, 2026-09-09 16:21:45.868Z..16:21:55.972Z.
  Both pinned inventories verify (two lines/388 bytes); effective coverage
  verifies (one line/159 bytes). Source hashes still match the retained four
  continuation transfers; no unrelated worktree changes were overwritten.
- New dedicated /tmp use is one 27,928-byte executable in
  /tmp/calcium-nfloat-controls.A4BEA9. Existing FLINT library and shared Rust
  target reused; about 23 GB /tmp availability remains, not a quota guarantee.
  No cleanup, deletion, commit, push or external report. Root report updated
  but explicitly interim. This goal turn is PROGRESS, not completion.
- Next: called nfixed/high-product and general matrix/field-relation support,
  remaining Calcium/FLINT inventory, unresolved transfer candidates and all
  remaining original ecosystem references. Do not repeat these completed reads
  or infer that passing finite controls resolves the preserved assertion.

## 2026-09-09 checkpoint 29 — fixed-point kernels and bounds

- Previous goal turn classified PROGRESS. Revalidated all 39 checkpoint-28
  bound files and all 955 retained live source/support hashes.
- Actual complete donor reads: nfloat/nfixed.c 1626; test/t-nfixed_dot.c 159;
  test/t-nfixed_mat_mul.c 111, t-nfixed_mat_mul_classical.c 111,
  t-nfixed_mat_mul_strassen.c 114, t-nfixed_mat_mul_waksman.c 111;
  profile/p-nfixed_mat_mul.c 268, profile/p-mat_mul.c 418;
  test/t-mat_mul.c 120, t-add_sub_n.c 96, t-addmul_submul.c 91, main.c 47;
  nfloat/inlines.c 14. Thirteen records add 3,286 lines and complete 13 files.
- Effective coverage: Calcium 375 reviewed/374 complete/one partial/37,085
  lines; FLINT 438 reviewed/426 complete/12 partial/58,440 lines. Combined
  813 reviewed/800 complete/13 partial/95,525 unique lines. This does not
  close either inventory or the full original ecosystem request.
- Fixed-point algorithms require caller scaling and have no overflow handling.
  Vector add/sub specialise 2..8 limbs; short dot kernels use signed truncated
  high products, wider ones split positive/negative accumulators. Waksman uses
  row/column correction storage and half shifts; Strassen uses matrix windows,
  two reusable temporaries and classical odd-border corrections. Duplicated
  cutoff logic must agree with intermediate-range and ULP error calculators.
- Concrete range-bound defect: S4=A22-A21+A12-A11 can equal 4A, but the bound
  routine uses 3A. Native vector operations on exact q=2^-20 inputs confirm
  the S4 witness is exactly 2^-18, above the reported bound in all six precision/
  four FENV configurations. This is one identity repeated 24 times, not 24
  independent bugs, an overflow test or an observed wrong final matrix product.
- Automatic Strassen cutoff selection uses parity while its bound helper passes
  the positive inner dimension in that position. Across six precisions and
  dimensions 24..60, 52 of 222 automatic/explicit-cutoff bound pairs differ;
  both reported range and error bounds are smaller in those sampled automatic
  cases. Source-supported dispatch mismatch, not 52 failed product computations.
- Donor tests mostly compare against another approximate high-product or
  extra-limb classical path, often choosing padding using the bound helper
  under test. The automatic fixed-point test's dimensions 1..20 miss the
  default Strassen thresholds. Explicit Strassen tests use cutoffs 0..5 and
  ordinary classical output despite an extra-precision-named allowance variable.
  Sequential profiler thresholds have no matched exact-work, allocation or
  binary qualification; no donor timing table was adopted as a Hyper cutoff.
- Independent integer driver: 560 nonempty raw dots and 1,740 matrix products,
  1,922,900 exact output-error comparisons, all within the stated output-error
  bounds. Dots use 2..8 limbs, lengths 1/2/3/8/17, four sign/sparse/randomized
  patterns and four contiguous/strided/reverse layouts. Matrices use five
  algorithms, 348 input cases each, 384,468 entries per algorithm. Fifteen
  positive shapes reach 58x57x59 with 2/3/4/8/12 limbs; 22/47/66 limbs use
  only four small shapes. Matrix inputs <2^-20 and dot inputs <2^-12. Four
  caller FENV modes are distributed among patterns, not full independent
  Cartesian matrix repetition. No output aliases or raw empty dots tested.
- Native and Memcheck each terminate normally with code 1 for the 24 range-bound
  witness failures, not for output-error failures. Memcheck has zero errors,
  zero live bytes/blocks and 17,379,886 allocations/frees totaling 1,052,400,360
  cumulative requested bytes. These totals include the GMP oracle and are not
  peak RSS, disk storage or donor-only performance measurements.
- Initial output checker incorrectly required identical full native/Memcheck
  logs. Preserved its source and failed run. Exactly 908 rows differ in 1,225
  binary64 bound fields (450 matrix range bounds, 769 matrix error bounds,
  six witness range bounds); case membership, decision flags and reported
  maximum errors agree. Both runs now undergo independent full validation.
- Independent no-FLINT binary64 control uses volatile operands, rounding-aware
  compilation with contraction disabled, and a 53-bit MPFR multiplication
  oracle. All 12 native cases match. Four instrumented cases differ; the
  instrumented values equal native nearest-mode controls while FENV still
  reports the requested modes. Its Memcheck has zero memory errors/all blocks
  freed but exits 1 for numerical disagreement. This demonstrates a local
  instrumentation limitation, not proof of a general cause for every donor
  discrepancy or permission to erase failed numerical evidence.
- Hyper reads: hyperlattice matrix/ops.rs 1..4, matrix/core.rs 6460..6590;
  hyperlimit predicates/filters.rs 1..225, resolve.rs 1..55 and 240..430,
  predicates/ring.rs 275..367, benches/predicates.rs 1830..1885, Cargo.toml
  1..43. Small exact dense/sparse matrix kernels, exact interval/ball sign
  predicates, structural sign-only filtering and explicit Unknown fallback
  contrast with unsupported approximate dominance. Existing two same/mixed
  sign regressions pass in debug/release, 240 filtered, none ignored.
- Potential local transfer recorded, not implemented: >4-term same-sign
  filtering collects a Vec although a constant-size summary may suffice.
  Preserve Unknown short-circuit/trace behavior and qualify realistic public
  nonrational consumers. The existing rational ring benchmark bypasses this
  filter; using it alone would not establish a candidate benefit.
- nfixed-experiment.json binds 47 evidence/support files, 12 terminal gates
  (seven successes, five preserved compile/checker/numerical failures), 13
  complete read records, the unchanged source maps, both binaries and linked
  libraries. Initial C compile warning was corrected only by separating cleanup
  loops; original source and log kept. No production or donor change.
- All 29 chained verifiers pass with --derivative-live: 29 JSON lines/43,065
  stdout bytes, empty stderr, exit 0, 2026-09-09 16:50:56.411Z..16:51:06.856Z.
  Both full pinned inventories and effective coverage also verify. This checks
  preservation/classification of failures, not that all numerical gates passed.
- Dedicated /tmp additions: nfixed-controls 27,352 bytes and independent
  fenv-product-control 12,960 bytes in /tmp/calcium-nfixed-controls.qZn5o6,
  total 40,312 bytes. Existing native library and shared Rust target reused;
  about 23 GB /tmp space remains, not a quota guarantee. No deletion, cleanup,
  commit, push or external report. All earlier evidence/core stays preserved.
- This goal turn is PROGRESS, not completion. Next: remaining complex nfloat
  implementation/tests/profilers, high-product and general field/matrix support,
  the possible constant-state sign-filter experiment, remaining inventories and
  every remaining original reference. The earlier field-relation assertion is
  not resolved by the present bounds or instrumentation findings.

## 2026-09-09 complex nfloat closure and sign-filter reachability — checkpoint 30

- Previous goal turn classified PROGRESS. Revalidated all 47 checkpoint-29
  bound files and all 955 retained live source/support hashes.
- Read all six remaining files under src/nfloat: complex.c 1..2068,
  profile/p-complex_mat_mul.c 1..422, profile/p-vs_acf.c 1..174,
  profile/p-vs_arf.c 1..160, test/t-complex_mat_mul.c 1..76 and
  test/t-nfloat_complex.c 1..76. All 26 tracked files in that directory now
  have complete source coverage. Called high-product/ARF/generic matrix/field
  support is not thereby closed. Six new records add 2,976 unique lines.
- Effective continuation coverage: Calcium 374 complete/one partial/37,085
  lines; FLINT 432 complete/12 partial/61,416 lines; combined 806 complete,
  13 partial, 98,501 lines. Both pinned full inventories verify.
- Complex contract: packed approximate real/imaginary pair, radius-dropping
  ACB import, encoded-value export, axis shortcuts, guard-limb four-product
  and aligned three-product kernels, 12-limb multiplication and 20-limb square
  thresholds with exponent-gap fallbacks. Principal roots choose formulas by
  real-part sign to reduce cancellation. Absolute comparison separates exponents,
  applies a padded binary64 filter and then compares exact squares via ARF.
  Comparing stored finite dyadics is narrower than computable-real equality.
  No directed complex enclosure or exact-field laws are promised.
- Donor tests use approximate ACB/max-norm comparison with low-precision-heavy
  repetition counts. Profilers use sequential 100ms timing and do not equate
  exact work or certify output errors. ARF/ACF vector addmul timings mutate
  earlier output across repetitions, potentially changing values/iteration
  counts between backends. No profiler cutoff adopted as a Hyper threshold.
- Hyperlattice complex.rs, tests/complex.rs, benches/mathbench/complex_ops.rs
  and fuzz/fuzz_targets/complex_ops.rs read completely (555/89/509/193 lines).
  Already has reuse-sensitive exact-rational three-product dispatch, fused cold
  kernels, shared inverses, checked Unknown-zero handling and all ownership
  forms. Existing complex square/power scheduling and symbolic fallbacks differ
  from raw limb arithmetic. No direct replacement justified. Additional exact
  Hyper read ranges are bound in nfloat-complex-experiment.json.
- Independent finite complex driver: 87,516 arithmetic cases plus 2,112
  squared-norm comparisons = 89,628 rows; 177,144 exact rational comparisons
  and 58,344 output-placement equality checks. The raw driver's `aliases`
  counter includes 37,488 actual input aliases and 20,856 unary outputs placed
  in the otherwise-unused second operand buffer; these are not all input aliases.
  All 66 word precisions, 32 deterministic
  sign/zero/axis/equality/cancellation/exponent-gap patterns, 14 arithmetic
  operations and three output placements. 1,188 zero-divisor placements are
  skipped, not counted as passed arithmetic. Inputs are bounded finite dyadics;
  no failed-operation output is read.
- Oracle directly decodes packed significands into GMP rationals. Component
  tolerance is 64*2^-p*max(1,absolute expected components); abs/sqrt/rsqrt
  check exact squared residuals plus principal-branch signs. Copy/conjugate/
  negation/projection checks are exact. Independent JS BigInt reconstruction
  confirms all 2,112 comparison signs: 528 negative, 330 zero, 1,254 positive.
  Not a correct-rounding, directed-enclosure or general relative-error proof.
- Native and Memcheck both exit 0 with identical full numerical stdout.
  Memcheck: zero errors, zero live blocks, 2,601,723 allocations/frees and
  655,390,264 cumulative allocated bytes including the GMP oracle. Not a
  donor-only memory benchmark or peak RSS. No complex matrix/dot, concurrent,
  32-bit, nonfinite or extreme-exponent qualification. Four FENV modes are
  distributed among patterns, not a full mode/operand Cartesian product.
  This pass does not erase checkpoint 29's independent FENV discrepancy.
- All six existing Hyperlattice complex integration tests pass in debug/release,
  no failures/ignored/filtered tests. Their expected values share Real arithmetic;
  they are regression checks, not a new independent scalar numerical oracle.
- Public strict ring-area reachability probe: five vertex counts 3/4/8/16/32,
  rational/well-separated-pi/near-pi fixtures, three same-object queries each.
  Subdividing a straight edge preserves twice-area pi-103993/33102. All 15
  near-pi queries reach the >4-term mixed-sign filter (four zero terms each),
  then return exact Positive by refinement. All 30 controls bypass that filter.
  Exact trace membership/counts are checked per query. Repeat zero is the first
  local query, not a cold process; no performance or general state claim.
- The possible constant-state filter remains unimplemented. Public reachability
  is now established, but correctness, trace/Unknown order, matched CPU/allocations,
  retention and binary costs must qualify any isolated prototype before retention.
- Initial C indentation-warning failure and Rust probe denominator-type failure
  are preserved with their original sources. Corrections only put the switch-case
  break after its loop on a separate line and use the unsigned denominator type;
  successful builds retain full logs. No production or donor patch.
- nfloat-complex-experiment.json binds 49 evidence/support files, 12 terminal
  gates (ten successes/two preserved compile failures), six complete read
  records, unchanged 955-file source maps, two executable snapshots and native
  linked-library hashes. All 30 chained verifiers pass with --derivative-live:
  30 JSON lines/46,512 stdout bytes, empty stderr, exit 0, 2026-09-09
  17:16:06.182Z..17:16:17.009Z. Verification preserves failures, not all-green
  qualification of every historical numerical or memory gate.
- Dedicated snapshots: /tmp/calcium-complex-controls.8yAaAz/nfloat-complex-controls
  22,504 bytes and sign-filter-probe 2,219,752 bytes, total 2,242,256. Native
  library and shared Rust target reused. About 22 GB remains on /tmp, not a
  quota guarantee or claim that cache growth equals dedicated snapshot size.
  No deletion, cleanup, commit, push or external report; old evidence/core preserved.
- This goal turn is PROGRESS, not completion. Next: isolate and qualify the
  constant-state sign filter using the now-proven public workload, then continue
  called high-product/ARF/generic matrix/field support, remaining inventories and
  every remaining original reference. Full ecosystem scope remains unchanged.

## 2026-09-09 constant-state sign-filter experiment — checkpoint 31, unselected

- Previous goal turn classified PROGRESS. Revalidated all 49 checkpoint-30
  bound files and all 955 retained live hashes. Full ecosystem scope remains.
- Created two 956-file source/support maps from the retained 955-file snapshot.
  Only Hyperlimit is copied (80 old files per variant); other crates point to
  the frozen prior snapshot. Both add the same 125-line test module and four
  registration lines. Candidate production-function delta is +13/-3 lines:
  two distinct nonzero representatives replace the Vec, with ordered continued
  inspection after mixed signs and unchanged <=4-term handling. No new donor
  read coverage claimed; continuation remains 806 complete/13 partial/98,501 lines.
- Independent sign table checks all 299,593 sequences of eight term/multiplier
  cases through length six, including known-zero, both signs, zero multipliers
  and unresolved signs even with zero multipliers. Another 1,280 cases reach
  lengths 5/8/32/128/512 and early/late Unknown/mixed/zero combinations. Reference
  Vec and tested implementation agree. Thirty-two selected cases preserve full
  trace snapshots, including Unknown after mixed signs and skipped later zeros.
- Baseline and candidate each pass 364 all-feature library/integration tests in
  debug/release: identical membership across 16 suites, no failed/ignored/filtered
  tests. The focused candidate run passes three tests with 252 filtered. Tests
  are finite qualification, not all possible scalar histories or full-stack CI.
- All 45 public ring traces match byte-for-byte between variants and the prior
  checkpoint. Independent JS BigInt rational shoelace sums verify affine-in-pi
  area formulas for 24 fixtures: four shapes/sign cases and six vertex counts.
  Alternating atan series plus the Machin tangent identity supply strict rational
  pi bounds, proving pi-103993/33102 positive without using Hyper arithmetic.
  Reversed rings have the negated area. Oracle ran before the CPU campaign.
- CPU: 2,304 measured batches/8,884,320 repeated queries across 48 groups,
  12 alternating ABBA blocks per group, affinity CPU 6, separate executable
  processes and six-millisecond-target calibrated batches. Four fixtures
  (rational, separated pi, near pi, reversed near pi), 3/4/8/16/32/128 vertices,
  retained or cloned prebuilt rings. Sixteen warmups; cloned-ring timing includes
  clone/drop, not cold scalar construction. Each measured result checks complete
  value/certainty/stage. Pilots are retained separately from measured counts.
- Candidate/base paired median ratios span 0.946269..1.080495. Five per-group
  bootstrap intervals lie below one, eleven above and 32 straddle one. Intervals
  are not multiplicity-adjusted; bypass controls also vary. No universal slowdown
  or speedup claimed. This does not establish a worthwhile performance win.
- Separate allocation campaign: 288 measured batches/4,608 queries, three
  repetitions per variant/group, 16 queries each. All 24 filter-reaching groups
  save exactly one allocation and 2*vertex_count bytes per query (6..256 bytes).
  The 24 bypass groups are identical. Peak requested live bytes fall in 12 groups,
  remain equal in 36; all post-batch live deltas are zero. No allocation/peak
  regression observed in these samples. Counts exclude RSS, stack and allocator
  overhead and do not prove arbitrary retention bounds.
- Unstripped CPU/allocation driver files grow 2,536/2,552 bytes; text sections
  +1,920/+1,912, data -8 each, BSS +2,232/+2,168. These are observed benchmark
  artifacts, not representative application size or an established codegen cause.
  Trace executable size instead changes -648 bytes; do not conflate feature sets.
- Decision: first representative-array implementation stays isolated and
  UNSELECTED. No exactness/completeness gain; small allocation savings do not
  justify its mixed runtime results and nontrace benchmark growth under the
  requested priorities. The mathematical constant-state idea is not rejected.
- sign-filter-experiment.json binds 67 support/evidence files, 15 successful
  terminal gates, both 956-file maps, unchanged 955-file live map and six
  executable snapshots. All 31 chained verifiers pass with --derivative-live:
  31 JSON lines/51,122 stdout bytes, empty stderr, exit 0, 2026-09-09
  17:40:15.396Z..17:40:26.853Z. Full raw measurements and exact test membership
  are independently rechecked, including bootstrap reconstruction.
- Dedicated six snapshots in /tmp/calcium-sign-filter.TCGoGs total 12,712,664
  bytes. Shared Rust target reused; about 21 GB /tmp remains, not a cache-growth
  measure or quota guarantee. No memory-sanitizer, WASM, concurrency, serialization,
  abort-recovery or representative downstream binary qualification is newly
  claimed for this unselected trial. No production change, deletion, commit,
  push or external report. All prior successful and failed evidence preserved.
- This goal turn is PROGRESS. Next: independently isolate a leaner bit-mask
  summary, preserve the same Unknown/trace contract, and compare against the
  baseline and controls before retention. If no worthwhile implementation
  survives, reject the transfer and continue high-product/ARF/generic matrix/
  field support and the remaining original references. Full goal remains open.

## 2026-09-09 bit-mask filter and high-product reads — checkpoint 32, unselected

- Previous goal turn classified PROGRESS. Revalidated 67 checkpoint-31 files
  and 955 live hashes. Preserve the unselected array prototype and its evidence.
- Isolated a bit-mask representation of the same four summary states, copying
  only the 81-file baseline Hyperlimit snapshot and reusing the other frozen
  crates. Both 956-file maps are checked; only the intended dynamic filter
  differs. The <=4 path, term visit order, Unknown short circuit and final
  trace helper remain unchanged. No assumption about Sign enum discriminants.
- Candidate debug/release each pass the identical 364 tests in 16 suites,
  including 299,593 exhaustive short sequences, 1,280 long sequences and
  32 private trace cases. No failed, ignored or filtered tests. Forty-five
  public traces match the frozen baseline and checkpoint 30 byte-for-byte.
  Cargo lockfile and application manifest differ only by the application/
  snapshot name. The prior independent 24-ring sign oracle is reused.
- CPU campaign: 48 groups, 2,304 timed batches / 8,346,816 repeated queries,
  12 alternating ABBA blocks/group on CPU 6; separately stored pilots and
  sixteen warmups. Candidate/base median ratios span 0.964811..1.139742;
  two per-group bootstrap intervals below one, seven above, 39 straddling.
  The near-pi three-vertex retained/cloned cases are slower in this sample.
  Bypass controls also vary. Intervals are not multiplicity-adjusted; these
  are warm prebuilt-ring queries, not cold scalar or universal performance.
- Separate allocation campaign: 288 batches / 4,608 queries. All recorded
  request/byte/live/peak measurements equal the first prototype campaign.
  Each of the 24 reaching groups saves one request and 2*vertex_count bytes
  per query (6..256); 24 bypass groups are unchanged. Peak requested bytes
  improve in 12 groups, are equal in 36; every post-batch live delta is zero.
  Not RSS, stack/allocator overhead or a general retention bound.
- Unstripped CPU/allocation files shrink 1,176/1,264 bytes; text changes
  -576/-584, data -8 each, BSS +600 each. Trace executable shrinks 2,304 bytes.
  Artifact-specific observations, not representative application sizes or a
  proven codegen explanation. Despite these savings, mixed CPU results with
  no exactness/completeness gain do not justify retention. Both prototypes
  stay isolated and UNSELECTED; stop this transfer in its tested forms.
- High-product support: six implementations (1,214 lines) and six tests
  (614 lines) fully read, plus 368 header/documentation lines. Exact ranges
  and hashes recorded: 12 complete files/two partial files, 2,196 new lines.
  Combined continuation: 818 complete/15 partial files, 100,697 unique lines.
  Generated tuning tables are read; assembly declarations do not credit bodies.
- Precise high multiplication returns n high limbs plus a guard but is not
  exact truncation: omitted low products can carry. Documentation states a
  one-sided n+2 guard-ulp bound; larger donor tests allow 2n, which is weaker
  for n>2. Small and normalised tests use sibling consistency, not independent
  full-product bounds. Recursive terminology and native restriction comments
  need reconciliation. These are source findings, not new numerical failures.
- Possible ideas: demand-sized high products with certified discarded-tail
  correction, and scratch reuse when the caller already supplies 2n limbs.
  Hyper's exact/certified contract is stronger than raw approximate high limbs;
  no donor cutoff or backend replacement is justified by this source pass.
  Donor tests were read, not newly executed; no high-product performance,
  independent accuracy or memory qualification is claimed here.
- sign-filter-mask-experiment.json binds 41 evidence/support files, eight
  successful gates, both 956-file maps, unchanged 955-file retained live map,
  14 read records and six binaries (three reused baseline/three new candidate).
  All 32 chained verifiers pass with --derivative-live: 32 JSON lines / 55,226
  stdout bytes, empty stderr, exit 0, 2026-09-09T19:21:12.616Z..19:21:24.711Z.
  Raw ABBA/calibration/bootstrap and allocation results are reconstructed;
  exact ordered test membership is checked. Both pinned inventories verify.
  The read-only inventory check initially hit sandbox subprocess EPERM; its
  approved rerun passed. This was an environment issue, not donor failure.
- Three new dedicated snapshots in /tmp/calcium-sign-filter-mask.ctu9Hl
  total 6,349,368 bytes; baseline binaries and shared build cache reused.
  About 21 GB remains on /tmp; snapshot bytes do not measure cache growth.
  No production/donor edits, deletion, commit, push or external report.
  Earlier successful/failed evidence and the crash core remain preserved.
- This goal turn is PROGRESS, not completion. Next: bounded valid-input
  high-product checks against independent GMP full products, then supporting
  assembly/FFT/ARF/generic matrix/field code and remaining original references.
  Do not reproduce invalid-size/alias or earlier assertion failures. The full
  requested ecosystem audit and final inventory reconciliation remain open.

## 2026-09-09 independent high-product qualification — checkpoint 33

- Previous goal turn classified PROGRESS. Revalidated checkpoint 32's 41 bound
  files and all 955 retained live hashes. No active process from that checkpoint.
- Reused the existing assertion-enabled, 64-bit ADX native FLINT build with
  FFT-small disabled; linked library hashes match checkpoint 30. New finite
  corpus uses positive bounded lengths, disjoint output buffers, seeded outputs
  and top-bit-set inputs for normalised routes. No invalid-size/alias experiment.
- Independent GMP full products check 53,248 outputs from 9,216 input cases:
  144 lengths (1..128 plus 16 selected lengths through 2,049), 32 deterministic
  patterns, two normalisation input forms. Seven routes: public multiply,
  square, caller-scratch multiply, naive/recursive multiply limited to 128 limbs,
  and normalised multiply/square only on appropriately normalised inputs.
  Includes public table/basecase/Mulders/full-product transitions, but not FFT,
  alternate architectures, all operand patterns or arbitrary storage histories.
- Let B=2^64, q=B^(n-1), and R be the n high output limbs plus guard limb.
  Check the integer deficit floor(P*2^shift/q)-R against (n+2)*2^shift plus
  the possible shift/truncation bit. Also check the stronger full residual
  P*2^shift-R*q against (n+2)*2^shift*q. These are separate interpretations;
  no approximate floating-point oracle is used.
- Both native and Memcheck terminate normally with exit 1: 24 integer-bound
  failures and 44 full-residual failures (20 additional fractional-only rows).
  Twelve integer failures are public square/normalised-square routes at four
  lengths; twelve are internal naive/recursive multiplication at three lengths.
  Counts repeat routes and normalisation forms, not independent defects. Public
  ordinary multiplication has no integer-bound failure in this corpus, though
  its n=40 rows fail the stronger fractional interpretation.
- Examples: public square integer deficit 30 versus bound 25 at n=23, and
  124 versus 89 at n=87. Internal reference multiply has deficit 62 versus 61
  at n=59. All discrepancies occur in the nearly-all-ones pattern 13; inputs
  are finite and valid. No incorrect Hyper or higher-level nfloat answer is
  established by these low-level bound discrepancies.
- Independent JavaScript BigInt reconstruction verifies 3,059 output rows /
  1,180 arithmetic cases, including all 24/44 failures. It separately computes
  full integer products, the retained triangular sum, and the exact omitted
  tail: low halves on diagonal n-2 plus all lower full products. This proves
  the sampled deficits are omitted-carry effects, not a GMP/decoder discrepancy.
  The triangular omitted tail is bounded conservatively below 2n-3 guard ulps
  for n>=3; n+2 need not hold. This formula is not a proof for every recursive,
  hardcoded assembly or FFT path. The weaker 2n donor test bound admits these
  examples, explaining why sibling/bounded donor tests need not expose them.
- Across the entire GMP corpus: no overestimate, all results within the looser
  scaled 2n bound, 15,809 exact truncated outputs, 4,727 one-bit normalisations,
  and all 832 public full-product-fallback controls exact. Both inputs remain
  unchanged in 18,432 checks. These are finite numerical results, not full
  correct-rounding, all-input error proofs, or an exact-real replacement.
- Native/Memcheck full numerical stdout is byte-identical (8,545,563 bytes
  each). Memcheck reports zero errors/live blocks, all 628,419 allocations
  freed, 891,786,227 cumulative bytes including the oracle. Not peak RSS or a
  donor-only allocation benchmark. Clean memory diagnostics do not turn the
  failed numerical runs into successful gates; earlier failures remain intact.
- Seven source files fully read: three ADX basecase assembly files (multiply,
  odd/even square), ARF round-down/other-mode multiplication, MPFR bridge and
  TLS scratch cleanup. Two append-only header extensions cover reverse-dot
  macros and ARF multiplication/temp dispatch. Adds 1,708 unique lines;
  combined continuation coverage is 825 complete/15 partial files, 102,405 lines.
  Hardcoded/normalised/arm assembly, rounding helpers and full inventories remain.
- ARF uses full products plus rounding, or a demand-sensitive MPFR bridge;
  its scratch macros select up to 40 stack limbs, up to 1,000 grow-only TLS
  limbs with registered cleanup, and larger transient allocations. Conditional
  source caps are 320/8,000 bytes on this build, not measured peak or arbitrary
  thread/reentrancy guarantees. No threshold is imported into Hyper.
- Current Hyper arithmetic_kernels.rs (264 lines) and scale.rs (20 lines)
  read fully, with selected multiplication tests. Hyper already sizes operand
  approximations from magnitude facts, reuses a single square operand and
  exactly multiplies BigInts before final scaling. Blindly substituting raw
  approximate high limbs would alter that error budget. No Hyper candidate,
  production/donor change, or new performance claim is justified by this pass.
- high-product-experiment.json binds 30 evidence/support files, seven terminal
  gates (five successes/two preserved numerical failures), 955 unchanged live
  hashes, nine read records, native/configuration hashes and one binary. The
  initial binding draft is separately hash-preserved: before verification, a
  Hyper AGENTS.md read-range typo was corrected from 1..18 to its actual 1..14
  lines. Donor coverage and numerical evidence were unaffected.
- All 33 chained verifiers pass with --derivative-live: 33 JSON lines / 60,998
  stdout bytes, empty stderr, exit 0, 2026-09-09T19:55:35.822Z..19:55:54.616Z.
  Both pinned source inventories verify; this preserves failed mathematical
  results rather than asserting that every historical gate passed.
- New /tmp/calcium-high-product.03dkHN/high-product-controls is 22,616 bytes.
  Existing native/Rust builds and all earlier logs/snapshots/core are preserved.
  About 21 GB remains on /tmp; cumulative oracle allocations do not represent
  retained space. No cleanup, deletion, commit, push or external report.
- This goal turn is PROGRESS, not completion. Continue hardcoded/normalised/arm
  high-product source and ARF rounding/temp contracts, then generic matrix/field
  support and every remaining original reference. Demand-sized multiplication
  or scratch reuse needs a certified carry/error design and matched Hyper
  workloads before retention. Full ecosystem audit and reconciliation remain open.

## 2026-09-09 specialised high products and ARF rounding — checkpoint 34

- Revalidated checkpoint 33's 30 bound files, separately preserved initial draft,
  955 live source hashes and five linked libraries before continuing. No active
  process remains from that checkpoint; the full goal is still incomplete.
- Fully read seven remaining high-product assembly files: Broadwell hard
  multiply (911 lines), hard square (565), normalised hard multiply (1,033),
  normalised hard square (671), and arm64 basecase multiply (419), hard multiply
  (793), hard square (528): 4,920 lines. All are now bound to pinned hashes.
- Both architectures omit lower product contributions and carry information;
  normalisation shifts the retained approximation, not the omitted tail. The
  square schedules exploit symmetry. Assembly ABI, carry scheduling, duplicated
  normalisation bodies and size-related TODOs are source observations, not
  instruction-level proofs or measured Hyper benefits. ARM was not run.
- Fully read ARF set/neg rounding, special multiplication, complex multiply/
  square, fused add/sub-product implementations and seven donor rounding tests:
  another 1,944 lines. Partial header/docs add 706 new lines after excluding
  previous overlaps. Combined increment is 7,570 lines, 20 complete source
  files and two partial records. Continuation coverage is now 845 complete /
  16 partial files, 109,975 lines. set.c was reread, not double-counted.
- ARF rounding retains significant bits and uses trailing-zero/sticky/parity
  information for exactness, nearest ties and directed rounding; increment
  scans handle carry runs and power-of-two rollover. The public in-place
  wrapper preserves its input while the internal limb helper requires disjoint
  storage. Specialized ui/uiui helpers have distinct preconditions.
- Complex/fused source preserves exact intermediate products before one final
  rounding, with balanced-size/exponent crossovers for three-product complex
  multiplication and fmpzi square. Donor multiplication tests often use exact
  arf_mul then arf_set_round as their reference, sharing implementation pieces.
  Read tests are not claimed as newly run; direct ui/uiui/mpz helper, fused and
  complex numeric qualification remain separate.
- Added an audit-only finite C/GMP oracle and 22,856-byte executable using the
  existing 64-bit ADX assertion-enabled FLINT build with FFT-small disabled.
  All 612,000 output/value/exactness checks pass, 61,200 for each of ten routes:
  set/neg rounding in disjoint and supported in-place output, public/swapped/
  in-place multiply, explicit MPFR multiply, public square and MPFR square.
  All 2,880 before/after input checks pass. No invalid raw overlap experiment.
- Corpus: 18 length pairs through 1,001 limbs, ten deterministic patterns,
  four sign forms, 17 precision positions including exact precision and
  coincident duplicate positions, and all five modes. Finite zeros, powers of
  two, all ones, sparse/tie cases and dense words with bounded ordinary exponents.
  Counts repeat related arithmetic/routes; not 612,000 independent identities.
  The positions straddle limb, full-product and source dispatch/scratch
  thresholds, without claiming complete instruction/branch coverage.
- GMP oracle uses full integer product and quotient/remainder rounding, then
  full comparison against directly decoded ARF limbs and exponent. No ARF
  rounding/conversion helper or MPFR reference. Integer machinery is shared
  with parts of the backend, so a second JavaScript BigInt implementation
  reconstructs 244,800 arithmetic roundings. It checks membership/order,
  exact/tie/carry counts and compact fingerprints for all 7,200 output groups.
  Fingerprints are supplemental consistency checks, not collision-free proofs.
- Native exit 0, 2026-09-09T20:11:38.158Z..20:11:40.005Z. Memcheck exit 0,
  20:12:29.772Z..20:13:30.573Z. Full numerical stdout matches (1,142,532 bytes
  each). Memcheck reports zero errors/contexts/suppressions, zero live bytes /
  blocks, all 303,090 allocations freed; 976,772,600 cumulative bytes include
  setup/oracle work. Not a native speed comparison, donor-only memory benchmark
  or peak RSS. BigInt captured run exit 0, 20:26:06.947Z..20:26:16.922Z;
  the intervening approval delay is not computation time.
- No infinity/NaN sweep, arbitrary fmpz exponents, thread/scratch lifecycle,
  invalid precisions, raw alias, all-helper, fused/complex, ARM, 32-bit or FFT
  runtime qualification. Earlier high-product 24 integer / 44 full-residual
  failures remain preserved; success under ARF's different contract does not
  erase those discrepancies or qualify all higher-level exact-real consumers.
- Read selected current Hyper multiplication/square/linear-combination and
  Hyperlattice complex/kernel/shared-scale dot ranges, with source hashes.
  Hyper already plans precision from magnitude facts and exactly multiplies
  integer approximations before scaling; squares share child work. Lattice
  already selects three-product exact-rational complex multiplication from
  cache reuse evidence, fused cold rational products, sparse signed product
  sums and certified shared-scale dot routes. No donor cutoff or generic
  representation replacement is justified. No new Hyper candidate/benchmark,
  production/donor edits or fifth retained continuation transfer.
- Initial full verification passed all 33 earlier checkpoints, then failed a
  Hyper read-range assertion: kernels.rs was recorded as 110..255, but its
  actual end is 250. The read returned through EOF; all numerical results and
  donor coverage are unchanged. Original 30-file manifest, binder/verifier and
  failed run (33 stdout lines / 60,998 bytes, 413 stderr bytes, exit 1) remain
  immutable. Corrected arf-rounding-experiment-v2.json binds 36 files and eight
  gates (seven successes plus that metadata failure), 22 read records, all 955
  unchanged live hashes, native/configuration hashes and the same binary.
  Use verify-arf-rounding-v2.mjs --derivative-live; v1 is preserved failed evidence.
- Corrected full chain passes: 34 JSON stdout lines / 67,196 bytes, empty
  stderr, exit 0, 2026-09-09T20:35:10.632Z..20:35:39.246Z. Both pinned
  inventories verify. Passing evidence verification preserves earlier failures;
  it does not assert that all historical mathematical or memory gates passed.
- Existing results, snapshots, binaries and crash evidence remain preserved.
  /tmp has about 21 GB available. Reused the native build; no new whole-tree
  native/Rust build, cleanup, deletion, commit, push or external report.
- This goal turn is PROGRESS, not completion. Next: finite fused/complex
  cancellation qualification as needed; remaining ARF/addition/temp/generic
  matrix/field support and every original reference. Demand-sized product
  ideas require certified omitted-tail/error handling and matched Hyper
  workloads. Full ecosystem audit and inventory reconciliation remain open.

## 2026-09-09 ARF fused/complex cancellation — checkpoint 35

- Previous goal turn classified PROGRESS: 7,570 source lines read, finite ARF
  rounding independently qualified, and corrected checkpoint 34 bound/verified.
  Revalidated all 36 bound files, 955 retained live hashes, five libraries,
  generated configuration and binary. Its final verifier is terminal exit zero;
  no process from checkpoint 34 remains active.
- Fully read 20 further source/test files: ARF addition/subtraction, limb
  addition, FMA, sum-of-squares, sum, exact/approximate dot, scalar/vector clear,
  memory manager; seven donor fused/complex/sum tests; Gaussian-integer square,
  exact shifted-limb import and all 1,014 lines of complex limb multiplication.
  Two header/docs extensions cover addition scratch, significant/bottom bits
  and exact versus approximate sum/dot contracts. Adds 4,131 unique lines;
  effective continuation coverage is 865 complete / 16 partial files, 114,106
  read lines. Both pinned tracked inventories verify.
- Exact fused products preserve cancellation before final rounding. The limb
  adder has small-word paths, aligned carry/borrow/sign recovery and a signed
  epsilon substitution only for sufficiently separated tails. Exact sums merge
  near blocks, remove exact zeros and use the sign of separated remainder;
  exact dot materializes exact products before summing. Approximate dot permits
  intermediate rounding and is excluded from all new correct-rounding claims.
- Addition uses a separate 40-limb stack / 1,000-limb grow-only TLS / larger
  transient scratch split. The mantissa free-list cache is explicitly disabled
  (ARF_USE_CACHE=0), unlike scratch reuse. These source facts do not prove peak
  memory, arbitrary lifetime, concurrency or reentrancy guarantees.
- Complex limb source contains shape-cost three-product selection, exact
  two-product square, selective input normalization, direct accumulation into
  outputs and compact scratch layouts. Transformed-domain reuse/exports and
  high variants were read, but FFT-small is disabled and approximate high
  complex paths were not newly executed. Source complexity/budget comments are
  not adopted as measured Hyper benefits or all-input error proofs.
- Added a 32,008-byte finite C harness, including the frozen checkpoint-34
  rounding/decoding oracle unchanged. Its old main is compiled but not called.
  No native library/Rust rebuild. All 829,440 scalar-component/value/exactness
  comparisons pass, with 19,584 before/after input checks on original scalars,
  initial, sum/reverse arrays and dot vectors; return flag ranges also pass.
- Corpus: 576 fixtures = 12 base lengths (1,2,3,19,20,21,111,112,113,249,250,251)
  times 12 families times sign masks 0/1/6/15. Twelve precision positions and
  five modes yield 34,560 results per route, with 24 component/result routes.
  Complex calls contribute two components each; counts repeat related
  arithmetic, precision positions and storage routes, not independent identities.
- Families include exact/near real and imaginary cancellation, finite zeros,
  exponent gaps 63/64/65/129/257 bits, and component limb imbalances two/three.
  Shared dyadic exponents are ordinary bounded values; exact precision results
  remain small. Main/fallback/left-/right-in-place complex products, ordinary/
  in-place squares, separate/x-/initial-aliased FMA, addmul/submul, sosq,
  add/sub, four-term forward/reverse sums and two-term exact dots (with/without
  a subtracted sum and initial) all use a complete integer-expression oracle.
  No raw invalid shape/alias, huge exponent or prior assertion reproduction.
- GMP compares every complete decoded value and exactness flag after a single
  quotient/remainder rounding; no ARF rounded-expression or MPFR oracle.
  BigInt independently reconstructs 345,600 expression roundings and matches
  all 13,824 groups' membership/order, exact/tie/carry counts and compact
  fingerprints. Fingerprints are not collision-free certificates. Its 468 zero
  and 600 unit-magnitude integer reference positions include repetition and
  zero controls; unit integer magnitude is not necessarily real magnitude one
  after dyadic scaling. No exhaustive branch or representation proof.
- Native exit 0, 2026-09-09T20:45:24.422Z..20:45:28.313Z. Memcheck exit 0,
  20:46:20.123Z..20:48:41.219Z. Full numerical stdout matches byte-for-byte
  (2,176,528 bytes each). Memcheck reports zero errors/contexts/suppressions,
  zero live bytes/blocks and all 2,249,297 allocations freed; 3,530,423,920
  cumulative bytes include setup/oracles/product vectors. Not donor-only cost,
  peak RSS or a matched native speed measurement. BigInt captured run exit 0,
  20:47:49.724Z..20:47:55.487Z. Approval delays are not computation timings.
- Current Hyper rational complex fallback, Real fact wrapper and Hyperlattice
  complex dispatch ranges read with actual EOF validation before binding.
  Lattice already uses cache-evidence-gated three products. The scalar cold
  rational path shares a word scan but uses four products for wider values.
  A shape-gated common-scale three-product candidate for balanced wide exact
  rationals is therefore a concrete next experiment, not a retained benefit.
  Preserve word-sized/unbalanced/cache-sensitive paths and measure correctness,
  state/trace, CPU, allocation and representative code/binary costs first.
- arf-fused-experiment.json binds 31 files, seven successful gates, 22 read
  records, all 955 unchanged live hashes, five libraries, generated configuration
  and the binary. No metadata correction was needed for this checkpoint.
  All 35 chained verifiers pass with --derivative-live: 35 JSON stdout lines /
  75,395 bytes, empty stderr, exit 0, 2026-09-09T21:09:11.071Z..21:09:45.554Z.
  Earlier metadata/numerical/memory failures remain preserved in the chain.
- /tmp has about 21 GB available. Added only the small executable there;
  paired numerical logs occupy about 4.4 MB in the workspace audit directory.
  Existing builds, binary/source snapshots and crash evidence remain preserved.
  No production/donor change, fifth retained transfer, cleanup, deletion,
  commit, push or external report; no performance improvement claimed yet.
- This goal turn is PROGRESS, not completion. Next investigate the isolated
  wide balanced rational complex candidate, and continue remaining ARF/Arb-dot,
  generic matrix/field/algebraic support and every original reference. Full
  original ecosystem scope and inventory reconciliation remain open.

## 2026-09-09 report summary and checkpoint-35 evidence recheck

- Added a concise executive summary to the workspace-root
  EXACT_REAL_ECOSYSTEM_AUDIT_REPORT.md, ordered by the requested retention
  priorities. It separates historical retained results, the four continuation
  transfers, rejected prototypes, mathematical/memory qualifications and the
  unfinished original scope. The report remains explicitly interim.
- Re-ran `node verify-arf-fused.mjs --derivative-live` through the exclusive-output
  capture helper. `results/report-checkpoint35-reverify.{json,stdout,stderr}`
  preserves the new run: exit zero, no signal, 35 JSON stdout lines / 75,395 bytes,
  empty stderr, 2026-09-09T21:18:41.422Z..21:19:15.840Z. All 955 retained live
  source/support hashes match. This revalidates evidence, including preserved
  failures; it is not a rerun of native suites, Memcheck or CPU benchmarks.
- Recomputed effective coverage without adding reads: Calcium 374 complete /
  one partial file, 37,085 lines; FLINT 491 complete / 15 partial files, 77,021
  lines; combined 865 complete / 16 partial files, 114,106 lines. No global
  ecosystem coverage total or new completed source checkpoint is claimed.
- No production/donor change, new candidate, retained transfer, build, cleanup,
  deletion, commit or push. `/tmp` still has about 21 GB available; this report
  pass adds only small workspace evidence logs and documentation. Existing
  builds, snapshots and crash evidence remain preserved. The cold wide-rational
  complex-product experiment is still pending, not a measured improvement.
- The full audit goal remains active and incomplete; all requirements below
  and the checkpoint-35 next actions are unchanged.

## 2026-09-09 cold wide-rational complex-product v1 — checkpoint 36

- Previous turn classified PROGRESS: the requested report summary was added and
  the complete checkpoint-35 evidence chain revalidated. Rechecked its terminal
  exit zero, all 31 bound checkpoint-35 files and all 955 retained live hashes
  before starting this experiment. No preceding process remains active.
- Read scalar construction/reduction, product-sum dispatch, complex multiply/
  divide, selected existing tests, multiplication-backend selection and
  Hyperlattice reuse gates. The 955-file retained live map still matches.
  A source-only candidate copy adds 45,445,675 bytes in the workspace; baseline
  reuses the retained derivative snapshot. Copying all six crate trees preserves
  candidate Cargo path dependency identity; each lock has one Hyperreal/Lattice.
- Candidate changes one isolated Hyperreal file, adding 68 lines. Common raw
  per-operand denominators plus a shape estimate select three signed integer
  numerator products after the word path. Unequal scales, narrow components and
  unfavorable shapes bypass it. Both ordinary and conjugate products receive
  final canonical reduction; no approximation or LCM inflation. Existing selected
  integer multiplication is reused. This is an arithmetic idea, not an adopted
  donor cutoff or a general cost proof.
- Both release probes pass 82,944 component/canonicality comparisons in 6,912
  fixtures, plus 27,648 input-preservation checks. Sixteen widths from 32 through
  2,048 bits, nine families, three scales and all 16 sign masks cover dense,
  sparse, zero, unbalanced, mixed-denominator, exact/near cancellation and
  zero-sum inputs. Direct multiply/divide and public Hyperlattice multiply are
  each called twice. Counts include repeated routes and 4,608 zero-component
  observations, not independent identities. Full GMP rationals are the oracle.
- Native and Memcheck numerical stdout agree for both variants. Native exits
  zero at 21:38:04.379Z / 21:38:08.638Z; Memchecks exit zero at 21:40:26.574Z /
  21:40:31.675Z on 2026-09-09. Zero errors and definite/indirect/possible losses;
  both retain 13,832 reachable bytes in 136 blocks. Baseline/candidate cumulative
  totals are 12,412,763 / 12,328,283 allocation calls and 1,879,323,436 /
  1,870,305,676 bytes, including test setup/oracle work. Not donor-only cost,
  zero live memory, peak RSS or an arbitrary lifecycle proof.
- Separate traces pass 5,832 rows per variant through 65,536-bit inputs against
  exact num-rational arithmetic, which shares the Rust integer backend. The
  scalar candidate is reached 2,520 times. Public first-use/reuse dispatch labels
  match exactly between variants; repeated lattice calls use the existing
  retained schedule. First/repeated trace controls are not serialization,
  cancellation, concurrency or internally unreduced-storage qualification.
- Uninstrumented CPU runs begin only after native/memory/trace/allocation gates
  finish. CPU 6 on the recorded Ryzen 7 5800X3D; rustc 1.97.0 / LLVM 22.1.6.
  Twelve alternating ABBA/BAAB blocks yield 9,216 observations / 26,799,888
  queries over 192 groups: six widths through 16,384 bits, eight scenarios,
  direct/public multiplication, separately constructed first-use or four-query
  prewarmed inputs. Construction, pool lifetime and exact preflight are excluded;
  result drops are included. No quotient timing or native donor comparison.
- Ratios span 0.5042667..1.4353356. Per-group 5,000-resample paired median
  intervals: 64 below parity, 43 above, 85 overlapping; not multiplicity-adjusted.
  Of 75 candidate-reaching groups, 59 improve, nine regress (all 192-bit cases),
  seven overlap. Of 117 bypass/reused controls, five improve, 34 regress and 78
  overlap. One 16,384-bit first-use public multiply drops from 109.536 to
  81.903 microseconds, but these selected gains do not justify keeping v1.
- Separate allocation campaign: 1,152 observations / 18,432 queries. Requests
  fall in 72 groups, requested bytes in 75, with no increases; live deltas match
  in all 192. Peak rises in 66, falls in nine, agrees in 117. The selected public
  case's peak rises from 21,640 to 33,680 bytes. Its requested bytes fall from
  123,536 to 104,888 per query, requests from 170 to 167. No general memory bound.
- Unstripped CPU/allocation executables grow 8,864 / 8,792 bytes (text +8,312 /
  +8,296; data unchanged, BSS -112). Trace grows 9,264 bytes. Benchmark sizes are
  not representative application sizes. Eight exclusive executable snapshots
  total 16,697,200 bytes in /tmp/calcium-complex-product.0bkAWH. Existing shared
  Rust/native builds and all earlier snapshots/crash evidence remain preserved.
- The output checker recomputes complete corpus membership, native/Memcheck
  equality, trace relationships, raw batch order, paired ratios/bootstrap
  intervals and allocation summaries. complex-product-experiment.json binds
  71 files, 16 successful gates, both 955-file source maps, live hashes, selected
  Hyper reads and all eight executables. The donor shape helper was reread;
  no new donor coverage is credited. Coverage stays 865 complete / 16 partial
  files and 114,106 read lines, not whole-ecosystem completion.
- Full chained verification passes: 36 JSON stdout lines / 78,703 bytes, empty
  stderr, exit zero, 2026-09-09T21:52:15.296Z..21:52:51.195Z, preserved as
  results/complex-product-verify-full.{json,stdout,stderr}. This validates the
  evidence chain, including earlier failures; it is not an all-green historical
  mathematical/memory gate claim. All current process handles are terminal.
- V1 is NOT SELECTED AS IMPLEMENTED. Next test a measured crossover and lower-
  overhead dispatch plus shorter signed-sum temporary lifetimes in a separate
  v2. Broader crate/debug/state/downstream and representative size qualification
  remains necessary before retention. No new production/donor change, fifth
  continuation transfer, cleanup, deletion, commit or push. /tmp remains about
  21 GB available; dedicated snapshots do not measure total build-cache growth.
- This turn is PROGRESS, not audit completion. All original references,
  remaining Calcium/FLINT/supporting reads and inventory reconciliation remain
  in scope. The pending v2 does not replace those full requirements.

## 2026-09-09 complex-product crossover/dispatch/lifetime revision — checkpoint 37

- Previous goal turn classified PROGRESS: v1 was numerically qualified, measured
  and left unselected for recorded regressions. Revalidated all 71 bound files,
  955 unchanged live hashes and terminal checkpoint-36 verifier exit zero.
- V2 is isolated in complex-product-v2-candidate. The source copy is 955 files /
  45,448,836 bytes before its edit. One Hyperreal file differs from retained
  baseline, adding 78 physical lines overall (ten more than v1). It uses a
  measured 256-bit minimum, the existing word-scan result plus a first-width
  check for early bypass, an out-of-line helper and scoped signed-sum temporaries.
  It retains the exact integer formulas and final reductions. Changes are
  bundled; this is not a factorial attribution experiment or universal cutoff.
- Reused unchanged Rust corpus/oracle/CPU/allocation/trace source, baseline
  source and four frozen baseline executables. Both fresh CPU and allocation
  campaigns rerun the baseline; only its native/Memcheck/trace evidence is
  reused. No old evidence, candidate, binary or live file is overwritten.
- V2's release GMP oracle again passes 82,944 component/canonicality checks in
  6,912 fixtures, plus 27,648 input-preservation checks. This includes all sign
  masks and the 255/256/257-bit threshold controls. Native exit zero at
  2026-09-09T22:02:10.524Z; Memcheck exit zero at 22:03:24.894Z. Numerical stdout
  matches the frozen baseline exactly, including repeated-route counts.
- Memcheck reports zero errors or definite/indirect/possible losses, with the
  same 13,832 bytes reachable in 136 blocks. V2 totals 12,364,643 allocations /
  12,364,507 frees / 1,871,964,076 cumulative bytes including setup/oracle work.
  These are not zero live memory, donor-only allocation cost or peak RSS.
- New candidate trace passes 5,832 rows through 65,536 bits against exact
  num-rational arithmetic (shared Rust integer backend). It selects the new
  scalar path 1,800 times, and bypasses the former 129/192-bit selections.
  External first-use/reuse labels match every frozen baseline row. Trace
  finishes at 22:09:37.590Z, before uninstrumented CPU timing starts.
- Compiler version and CPU identity match checkpoint 36: rustc 1.97.0 /
  LLVM 22.1.6, Ryzen 7 5800X3D, CPU 6 for measurements. This does not assert
  fixed clock frequency or identical system load between campaigns.
- Fresh CPU campaign: 22:11:22.043Z..22:12:57.456Z, 9,216 ABBA observations /
  26,810,928 queries, same 192 groups and workload construction as v1. Ratios
  span 0.4961558..1.2641581. Per-group bootstrap intervals: 66 below parity,
  17 above, 109 overlapping. No multiplicity adjustment or universal gain claim.
  Of 60 candidate-reaching groups, 55 improve, none has an interval wholly above
  parity and five overlap; among 132 bypass/reused controls, 11 improve, 17 have
  intervals above parity and 104 overlap. A selected 16,384-bit fresh public
  multiply goes from 114.657 to 79.976 microseconds in this campaign. Those
  absolute timings are not a paired v1-versus-v2 comparison.
- Allocation: 1,152 observations / 18,432 queries. Requests fall in 57 groups,
  requested bytes in 60, neither increases. Live deltas match in all 192 groups.
  Peak falls in 27, rises in 33 and matches in 132 versus retained baseline.
  Across the same 60 selected groups, cumulative requests/bytes equal v1; peak
  is lower in 36 and unchanged in 24. The selected 16,384-bit public case still
  peaks at 33,680 versus baseline 21,640 bytes. Shorter sum lifetimes do not
  eliminate every larger temporary peak.
- Unstripped CPU/allocation executables grow 9,192 / 9,152 bytes over baseline;
  text grows 8,256 bytes in each, data is unchanged and BSS drops 64 / 80 bytes.
  Trace grows 9,712 bytes. Names/build layout can affect unstripped sizes; no
  representative application-size benefit or source-size saving is claimed.
  Four new binaries total 8,367,856 bytes in /tmp/calcium-complex-product-v2.jy5gpf;
  four reused baseline binaries total 8,330,768 bytes. No whole native rebuild.
- Read the remaining gaps in Hyperlattice's 555-line complex module, including
  checked-zero boundaries, powers, scalar division and all four ownership
  multiply/divide wrappers. They share the component helpers, but those other
  workload lifecycles are not newly benchmarked or broadly qualified here.
- complex-product-v2-experiment.json binds 68 files, 12 successful new gates,
  three reused baseline numerical/memory/trace gates, both 955-file maps and
  eight binaries. Its checker recomputes memberships, raw batch order, paired
  bootstrap summaries, exact output agreement and trace controls. No new donor
  lines: coverage remains 865 complete / 16 partial files, 114,106 lines.
- Full chained verification passes: 37 JSON stdout lines / 82,500 bytes, empty
  stderr, exit zero, 2026-09-09T22:16:24.890Z..22:17:02.172Z, preserved in
  results/complex-product-v2-verify-full.{json,stdout,stderr}. All task handles
  are terminal. This validates the evidence chain without erasing earlier
  failed mathematical/memory gates or claiming they all passed.
- V2 is NOT SELECTED AS IMPLEMENTED. Both prototypes have recorded dispositions.
  Wide-input gains are real in the measured corpus, but there is no exactness or
  completeness gain, and bypass costs plus peak/code-size costs remain. Do not
  continue generic dispatch tuning without new application workload evidence.
  No new full crate/debug/state-history/concurrency/internal-unreduced-storage/
  quotient timing/WASM/application-size qualification or production transfer.
- This turn is PROGRESS. Next return to remaining Arb dot/error-bound support
  (dot.c, dot_precise.c, dot_simple.c and typed wrappers, add_error/fused support),
  then other supporting and original references. Full original inventory
  reconciliation remains open. No fifth retained continuation transfer, donor
  edit, cleanup, deletion, commit, push or external report. Prior failed evidence,
  snapshots and crash dump remain preserved; /tmp has about 21 GB available.

## Checkpoint 38 — Arb dot/error-bound support

- Revalidated checkpoint 37's 68 bound files, 955 unchanged live hashes and
  terminal successful verifier before starting. About 21 GB was available in
  /tmp; reuse the existing native FLINT build and add only bounded audit tools.
- Read dot.c, dot_precise.c, dot_simple.c, all five integer dot wrappers,
  add_error.c, addmul.c, fma.c, internal declarations and eight donor tests
  completely, plus public declarations, magnitude representation and dot/stride/
  alias documentation ranges. Read selection freezes 20 complete / three partial
  files, 3,327 new lines. Previously credited ARF header rereads add no credit.
  Effective total: Calcium 374 complete / one partial / 37,085 lines; FLINT
  511 complete / 18 partial / 80,348 lines. Combined 885 / 19 / 117,433.
- Optimized dot separates midpoint truncation, propagated input uncertainty and
  final rounding errors, with precision capped by input radius and actual
  product bottoms. Integer adapters use shallow normalized views and a batched
  shift buffer when big coefficients need normalization. Precise dot allocates
  exact product vectors and rounds its radius outward; it is not an independent
  backend or the tightest rectangle enclosure. MPFR high-product symbols are
  external; their source is still an explicit supporting audit gap, not assumed
  to be the same FLINT high-product kernels as earlier bound failures.
- Built one 27,768-byte audit executable in /tmp/calcium-arb-dot.zsWvwv, reusing
  existing native FLINT/MPFR/GMP libraries. No full native or Rust rebuild.
  The bounded public corpus uses complete independent GMP rational rectangle
  extrema and full decoded endpoint comparisons, not Arb sibling overlap.
  GMP also underlies some FLINT integer operations; no second integer backend
  or formal proof is claimed. Compact fingerprints supplement the full oracle.
- Native exit zero at 2026-09-09T22:37:45.201Z: 648 fixtures / 1,224 operation
  groups / 198,288 results / 396,576 endpoint comparisons and 21,349 input checks.
  Each ball routine has 46,656 calls; each integer wrapper 11,664. Input checks
  cover Arb arrays, initial and fmpz mirrors, not every word-array byte directly.
- Twelve ball limb widths [1,2,3,11,12,13,24,25,26,331,332,333], six families,
  lengths [0,1,2,5], layouts [(1,1),(-1,1),(2,-2)], both subtraction choices,
  absent/separate/output-aliased initial and nine precision positions. Integer
  wrappers use x widths 1,3,26; coefficient sizes reach 258 bits. Counts include
  related routes and repeated precision/alias positions. No instrumented branch
  coverage, every-integer, arbitrary-stride/length or correct-rounding claim.
- Focused Memcheck exit zero at 22:41:02.317Z; numerical stdout matches native
  exactly (161,471 bytes each). Zero errors, suppressed reports or live heap
  blocks; 687,105 allocations/frees, 727,232,808 cumulative bytes including
  input construction and oracle work. Not donor-only allocations or peak RSS.
  All memory/numerical histories from previous checkpoints remain preserved.
- No invalid raw shape/overlap, self-dot correlation, negative length, nonfinite,
  huge exponent or exceptional radius-only midpoint branch was exercised.
  Direct FMA/add-error APIs and unexecuted 32-bit/FFT/other architecture branches
  are source-only. No memory-safety reproduction or donor patch was attempted.
- Read current Hyperlattice shared-scale dot boundary and Hyperreal dyadic
  scale/headroom and exact product-sum planning. Those paths already preserve
  certified exact factors, word/stack/wide schedules and delayed normalization.
  Truncated midpoint arithmetic cannot replace an exact-value result without
  a separately certified enclosure contract. No concrete additional transfer
  with measured benefit was found; no matched Hyper timing/size claim is made.
- arb-dot-experiment.json binds 27 files, six successful gates, 23 read records,
  955 unchanged live source hashes, five shared library identities, configuration
  and the new executable. Full chained verification passes: 38 JSON records /
  88,516 stdout bytes, empty stderr, exit zero at 2026-09-09T22:45:24.102Z,
  preserved in results/arb-dot-verify-full.{json,stdout,stderr}. This validates
  the evidence chain, not universal numerical correctness of earlier failed
  gates. All command handles are terminal and /tmp still has about 21 GB free.
- This is PROGRESS, not audit completion. Four retained continuation transfers
  remain unchanged. Next continue remaining Arb/magnitude and scalar support,
  supporting MPFR source, then remaining generic field/matrix/algebraic kernels
  and every original reference. Complete inventory reconciliation remains open.
  No new production change, cleanup, deletion, commit, push or external report;
  source snapshots, all prior gates, binaries and crash evidence remain intact.

## Checkpoint 39 — magnitude bounds and error inflation

- Previous goal turn is PROGRESS: 3,327 donor lines added, complete rational
  enclosure checks and a successful chained verifier. Revalidated all 27 bound
  checkpoint-38 files, 955 live source hashes and its terminal verifier before
  continuing; /tmp remains about 21 GB available.
- Read 36 new complete donor files, including the magnitude manual, 23 core
  magnitude implementations, two integer limb-window helpers and ten donor
  tests. Four additional range records complete mag.h and extend ARF/Arb
  headers and ARF get.c. Preserved old header overlap. New actual coverage is
  3,978 lines; 37 additional files are now complete including the formerly
  partial mag.h. Totals: Calcium 374 complete / one partial / 37,085 lines;
  FLINT 548 complete / 18 partial / 84,326 lines. Combined 922 / 19 / 121,411.
- Upper/lower magnitude APIs are directed bounds, not best-rounding promises.
  Decreasing inputs, notably divisors, require the opposite bound polarity.
  Fast paths require finite inline exponents in destinations as well as inputs.
  Shifted arithmetic accounts for discarded tails; deep-cancellation lower
  subtraction uses ARF. Root code pads binary64 estimates before normalization.
  Integer constructors handle zero before nonzero-only limb-window helpers.
- Added a 47,216-byte audit executable in /tmp/calcium-magnitude.ipqBZw, reusing
  native libraries and the frozen checkpoint-38 rational decoder/rectangle
  oracle. Its previous main is compiled but not executed. No whole native/Rust
  rebuild or production transfer. Binary size is audit-tool storage, not a
  donor/Hyper size comparison.
- Native passes at 2026-09-09T23:02:08.299Z: 505,048 exact rational magnitude
  direction checks and equally many supplemental relative quality checks,
  14,688 comparisons, 31,416 magnitude/conversion input checks, 6,480 Arb results
  (12,960 endpoint comparisons) and 1,584 ball input/midpoint-preservation checks.
  Square-root and reciprocal-root bounds are checked by exact squaring, not
  an approximate root oracle. GMP still underlies some FLINT integer work;
  no second integer backend, formal proof or universal bound theorem is claimed.
- Magnitudes: 7,344 input pairs, twelve normalized-or-zero mantissas, three base
  scales and 17 signed exponent gaps through 1,025. Twenty-eight arithmetic
  routes with supported whole-object aliases and small positive/signed powers.
  Constructors: 680 fixtures, 17 width positions through 513 bits, four patterns,
  both signs and five ordinary scales, across 16 routes. Ball controls: 144
  fixtures, six widths through 331 limbs, six families, four sign masks and
  eight precisions; 5,760 FMA/addmul results and 720 five-API radius inflations.
  Counts include repeated aliases/precision positions. Relative quality uses
  1023/1024..1025/1024, after squaring for roots; not correct rounding.
- Memcheck passes at 23:02:39.533Z with identical 197,917-byte numerical stdout.
  Zero errors, suppressed reports or live heap blocks; 9,839 allocations/frees,
  6,879,352 cumulative bytes including setup/oracle work. Not donor-only demand,
  peak RSS or qualification of earlier failed memory histories.
- Preserved two corrected audit failures: initial compile rejected misleading
  indentation; formatting alone fixed it, with identical whitespace-stripped
  C source. Initial read-count assertion expected 4,078 rather than the actual
  3,978 disjoint lines. Its one-constant correction preceded applying any new
  read record. Both initial sources and failed gate logs remain bound.
- Read Hyper's inverse/multiply/add precision planning and BoundInfo distinctions
  among structural Zero, Unknown, NonZero and approximate versus exact MSDs.
  Importing a fixed 30-bit bound layer would change error/representation contracts
  without an established exactness/completeness/workload gain. No nonredundant
  transfer, matched Hyper benchmark, production improvement or fifth retained
  change is claimed. The four retained continuation transfers remain unchanged.
- magnitude-experiment.json binds 35 files, eight gates (six successful, two
  preserved corrected failures), 40 read records, 955 live hashes, five shared
  library identities, native configuration and the executable. Full chained
  verification passes: 39 JSON records / 95,243 stdout bytes, empty stderr,
  exit zero at 2026-09-09T23:12:01.924Z. Complete capture is preserved in
  results/magnitude-verify-full.{json,stdout,stderr}. All command handles are
  terminal; chain validity does not relabel prior failed numerical gates.
- Source-only gaps include hypot, large/promoted powers, extreme/nonfinite
  exponents, arbitrary libm/rounding/architecture/thread environments, remaining
  magnitude transcendental/tail/combinatorial/IO routines and unread ARF get.c.
  No invalid raw shape/overlap or prior safety failures were reproduced. No new
  full Rust/debug/state/serialization/concurrency/WASM/application-size gates.
- This turn is PROGRESS; full ecosystem completion and inventory reconciliation
  remain open. Next continue remaining magnitude and scalar support, including
  external MPFR high-helper source, then remaining generic field/matrix/algebraic
  and original references. No cleanup, deletion, donor patch, commit, push or
  external report. All snapshots, prior gates, binaries and crash evidence stay
  preserved; /tmp still has about 21 GB available.

## Checkpoint 40 — remaining magnitude source / transcendental bounds

- Revalidated checkpoint 39's 35 bound files, 955 live hashes and terminal
  successful verifier before continuing. /tmp has about 21 GB available; reused
  the existing FLINT/MPFR/GMP build and added only a small audit executable.
- Read all 29 remaining top-level mag/*.c implementations and the 21-line
  internal header completely, including every factorial/reciprocal/log table
  entry. Also read the complete 108-line double-extras header and thirteen
  transcendental/root/exp-tail donor tests: 44 new complete files, 4,660 disjoint
  lines. All 52 top-level mag C implementations now have complete read records,
  not all tests/profiles or recursively called support. Effective coverage:
  Calcium 374 complete / one partial / 37,085 lines; FLINT 592 complete /
  18 partial / 88,986 lines. Combined 966 complete / 19 partial / 126,071 lines.
- Ordinary donor tests compare through ARF/MPFR at magnitude precision; exp-tail
  tests only a finite 50-term lower sum. The new oracle uses direct exact dyadic
  imports to MPFR, avoiding ARF and huge integer expansion. Exponential-tail
  references sum N..N+256 with directed arithmetic and bound all remaining
  positive terms by t_(N+257)/(1-x/(N+258)), for x <= 4 and N <= 257.
- Source distinctions: finite 30-bit magnitudes are bounds, not exact-real
  values; logarithms clamp the unsupported sign, planner log2 is explicitly
  approximate, and large exponential/tail paths can deliberately be coarse.
  Binary64 polynomial padding is an implementation claim, not a new all-FENV
  proof. Large lower-sinh's mixed subtraction polarity passes the selected
  transitions, but compensating slack needs more than a function-name argument
  for a general proof. Source-only I/O observations were not fault-reproduced.
- Native passes at 2026-09-09T23:34:12.194Z: 36,208 bound checks, 31,652
  supplemental relative-quality checks, 16,742 input-preservation checks and
  16,742 supported whole-object alias agreements, all zero failures. Per
  reference precision: 371 unary inputs from ten mantissas / 37 exponents plus
  zero, 16 elementary-function routes, eight positive root degrees through 31,
  99 exp-tail inputs, 680 positive binary64 log inputs and both pi bounds.
  Counts include repeated precisions/routes/aliases, not distinct identities.
- References use 512 and 768 bits and yield identical donor outputs. Exponential
  growth/decay input exponents stop at 26; selected other routes span -1075..1024.
  All 33,488 magnitude outputs are finite. Relative quality uses 1/1024 slack,
  excluded for exponential-family input exponents > 10 and all tails. MPFR
  shares GMP integer support; two reference precisions are not independent
  implementations, formal proof, correct rounding or full branch coverage.
- Focused Memcheck passes at 23:34:34.109Z with identical 866,255-byte numerical
  stdout. Zero errors, suppressed reports or live heap blocks; 116,716
  allocations/frees, 22,125,312 cumulative bytes including oracle/setup work.
  Not donor-only costs or peak RSS. Existing FENV/LLL/thread-memory histories
  remain unresolved as previously recorded; these new passes do not erase them.
- Read Hyper's complete exp/sqrt/nth-root and logarithm approximation modules.
  Hyper already checks local series domains before coarse shortcuts, sizes
  guard/truncation work to demand, reuses exact-rational reductions, and uses
  integer root enclosures. No fixed-30-bit representation replacement or other
  nonredundant candidate with demonstrated benefit was found. The four retained
  continuation transfers remain unchanged; no new matched Hyper benchmark claim.
- The new 33,256-byte executable is in /tmp/calcium-mag-transcendental.p8Wy83;
  no whole native/Rust rebuild. mag-transcendental-experiment.json binds 26 files,
  six successful gates, 44 full read records, 955 unchanged live hashes, five
  library identities, native configuration and the executable. Full chained
  verification passes at 2026-09-09T23:47:37.528Z: 40 JSON records / 100,963
  stdout bytes, empty stderr, exit zero. Capture is preserved in
  results/mag-transcendental-verify-full.{json,stdout,stderr}. All handles terminal.
- This turn is PROGRESS, not ecosystem completion. Next qualify source-read
  combinatorial/conversion/other-tail routines with bounded independent oracles
  as warranted; finish mag tests/profiles, unread ARF conversion and scalar
  support, external MPFR high helpers, then generic field/matrix/algebraic and
  every original reference. Full inventory reconciliation remains open. No new
  production/donor edit, cleanup, deletion, commit, push or external report;
  all prior gates, snapshots, binaries and crash evidence remain preserved.
  About 21 GB remains available on /tmp, not a quota guarantee.

## Checkpoint 41 — combinatorial/tail bounds and conversion support

- Previous turn is PROGRESS: 4,660 source lines, bounded independent numerical
  qualification and a complete 40-checkpoint integrity pass. Revalidated all
  26 bound files, 955 live hashes and terminal verifier before continuing.
- /tmp has about 21 GB available. Reused native libraries; one new 38,448-byte
  executable is preserved in /tmp/calcium-mag-series.o08QL0. No whole-tree
  native/Rust rebuild, cleanup or deletion. Source snapshots and failed evidence
  remain preserved. No new production or donor change.
- Read the remaining 28 mag test/driver files, complete 251-line ARF I/O and
  ARF get.c gaps 1..439 and 531..604. Preserved old 440..530 credit. New coverage
  is 29 fresh complete files plus one completed existing file, 3,478 disjoint
  lines. All 104 inventoried files under src/mag/ are now read, plus the earlier
  mag.h/manual; not all numerical paths or recursive support. Calcium totals
  374 complete / one partial / 37,085 lines; FLINT 622 complete / 17 partial /
  92,464 lines. Combined 996 complete / 18 partial / 129,549 read lines.
- Donor tail tests compare only finite prefixes. The new geometric oracle is
  exact x^N/(1-x); polylog sums N..N+512 with directed arithmetic and bounds all
  remaining terms using q=x*(1+1/(N+513))^(max(-sigma,0)+d). For subsequent
  indices log(k)>=1, so this bounds the logarithmic and polynomial ratios.
  Hurwitz uses directed zeta(s) minus its finite prefix, not a truncated tail.
- Native passes at 2026-09-09T23:59:33.497Z: 48,890 exact comparisons, 3,358
  directed comparisons, 1,441 input checks, 1,240 whole-object alias agreements,
  402 valid string roundtrips, zero failures. Exact corpus: every factorial and
  reciprocal factorial from 0..4096, all 512 table pairs, 36,237 binomial inputs,
  182 binomial powers, 129 Bernoulli coefficients, 88 geometric tails, 264 finite
  binary64 inputs with five scales and 201 magnitude conversion fixtures.
- Independent JavaScript BigInt rechecks all 48,086 emitted rational bounds and
  201 double exports. Bernoulli uses a different defining recurrence from C's
  Akiyama-Tanigawa recurrence, with absolute-value sign reconciliation. Native
  603 fmpq/ceil/floor comparisons are not separately emitted for JS recomputation.
  Source-only file I/O and malformed/exceptional parser cases are not reproduced.
- At each MPFR precision 512/768, 576 polylog inputs and 527 Hurwitz inputs have
  complete full-tail references. Two precisions give identical donor outputs,
  not independent transcendental backends. Of 576 known-convergent polylog
  inputs, 199 yield valid but uninformative infinity and 377 finite bounds.
  The 796 repeated infinity outputs are included in the directed-check count;
  they are not described as finite-enclosure successes.
- Focused Memcheck passes at 2026-09-10T00:00:14.166Z. Native/Memcheck numerical
  stdout matches exactly: 1,308,580 bytes each. Zero errors, suppressed reports
  or live heap blocks; 636,551 allocations/frees, 73,880,835 cumulative bytes
  including oracle/setup. No donor-only allocation or peak-memory measurement.
  Bounded valid inputs, default FE_TONEAREST only; earlier failed numerical,
  FENV/LLL/thread-memory histories remain preserved and unresolved as recorded.
- Hyper comparison reads cover exact factorial/binomial product trees, word
  batching, gamma/beta structural cancellation, exact dyadic float import and
  e's term planning/binary splitting. Fixed-precision bounds cannot replace
  exact coefficients. One unimplemented planning-only candidate is identified:
  replace e_terms_for_precision's growing factorial solely with a normalized
  fixed-word LOWER enclosure to certify the same or conservatively larger term
  count. An upper factorial estimate is wrong polarity. Requires proof,
  independent exact/MPFR/state checks and matched CPU/allocation/size benchmarks.
  No gain, fifth retained transfer or source-size benefit is claimed yet.
- mag-series-experiment.json binds 26 files, six successful gates, 30 read records,
  955 unchanged live hashes, five library identities, configuration and executable.
  Full chained verification passes at 2026-09-10T00:10:30.054Z: 41 JSON records /
  106,842 stdout bytes, empty stderr, exit zero. Preserved capture is in
  results/mag-series-verify-full.{json,stdout,stderr}. All handles are terminal.
- This turn is PROGRESS. Next isolate and qualify the e planning candidate;
  continue remaining ARF/scalar/MPFR high-helper, generic field/matrix/algebraic
  and every original reference. Full inventory reconciliation and older open
  transfer experiments remain requirements. No production/donor edit, cleanup,
  deletion, commit, push or external report. About 21 GB remains on /tmp.

## Checkpoint 42 — isolated lower-factorial `e` planner, not selected

- Previous turn is PROGRESS: 3,478 source lines, independent BigInt/full-tail
  qualification and a completed 41-checkpoint integrity pass. Revalidated its
  26 bound files, 955 live hashes and terminal verifier before starting.
- Read the complete current constant kernels and relevant shared-cache/constructor
  boundaries. The candidate changes only factorial-size planning, not exact
  series coefficients, binary splitting, final rounding or shared cache storage.
- Test a normalized 64-bit lower mantissa with exact 128-bit word products.
  Truncation is downward, so crossing the bit threshold certifies the true
  factorial crosses it too. The result can select extra terms but cannot omit
  required terms. Prove bounded arithmetic/termination and compare exact
  thresholds, direct numerical kernels, public/cache histories and costs.
- The candidate adds eight net source lines and changes only term planning.
  A proof covers downward truncation, word-product bounds, termination and the
  original one-unit approximation contract. Unsupported i32::MIN handling is
  unchanged. No claim of identical term counts for every possible request.
- Exact GMP/independent BigInt thresholds agree on 4,151 requests per variant,
  including factorial neighbors and precision through 262,144 bits. No sampled
  extra terms. GMP checks all 20,367 candidate-loop invariant steps at the deepest
  request. Each variant passes 279 directed MPFR kernel/refinement/coarsening
  outputs plus 21 cancellation, serde, exp(1), four-thread and post-thread checks.
  JavaScript BigInt independently rechecks 8,302 plans and 600 output enclosures
  with a different exact series recurrence and complete geometric remainder.
- Four sequential Memchecks pass; 8,864 numerical/summary output rows match
  native. Zero errors/lost/suppressed blocks, but 544 runtime bytes remain in
  planner runs and 35,656 runtime/cache/MPFR bytes in numerical runs. Not
  zero-live-heap, thread Memcheck or peak/RSS qualification. Oracle/setup costs
  are included in Memcheck cumulative allocation figures.
- The initial audit build failed on two Rug integer-to-float endpoint conversions
  and an unused import. Preserved results/e-plan-check.initial-build.rs plus
  failed build gate; fixed only the harness before successful baseline/candidate
  builds. No warning remains in those builds. Failed evidence was not removed.
- All-feature Hyperreal library/integration tests pass 855 tests in each variant
  and each debug/release profile, 14 suites with identical membership and no
  ignored/failed/filtered tests. Last candidate release gate finishes
  2026-09-10T00:45:40.856Z. Not default-only/full-CI/downstream qualification.
- CPU 6: 71 predeclared groups / 3,408 observations, 12 alternating ABBA/BAAB
  blocks, separate fresh/refine/warm/coarsen lifecycle timing. All ten planner
  and ten kernel intervals improve; overall 37 below one, three above, 31 overlap.
  Ratios span 0.003568..1.084187; per-group intervals are not multiplicity-adjusted.
  Public coarsening at 0 and -262144 and unchanged warm pi at -4096 have slower
  intervals. Tiny/cached and unchanged-path timing sensitivity is disclosed.
- Fresh 65,536-bit public e median: 3.4048715 to 1.2938615 ms. Allocation requests
  11,558 to 4,643, requested bytes 32,693,112 to 1,628,872, peak 103,248 bytes
  unchanged. Separate 426 allocation observations: request/byte demand lower
  in 28 groups, unchanged in 43, no increases; live unchanged in all 71. Seven
  planner-only peaks decrease; public and full-kernel peaks do not. No blanket
  memory/runtime gain. Audit CPU/allocation binaries shrink 1,304/1,472 bytes,
  oracle grows 944; not representative application size.
- Only the 180-file Hyperreal subset is copied, 5,312,930 original bytes in the
  workspace. Six frozen executables total 11,204,552 bytes in
  /tmp/calcium-e-plan.mOIsBs. Existing offline Rust cache reused; about 21 GB
  remains on /tmp. No cleanup, deletion, donor/production edit, commit or push.
- e-plan-experiment.json binds 94 files, 21 gates (20 successful, one corrected
  initial failure), 955 unchanged live hashes, 180 candidate hashes and six
  executable identities. No new donor-source credit: Calcium/FLINT stays
  996 complete / 18 partial files / 129,549 read lines. The fifth continuation
  transfer is still pending, not retained.
- Full chained verification passes at 2026-09-10T00:50:00.566Z: 42 JSON records /
  112,213 stdout bytes, empty stderr, exit zero. Preserved capture:
  results/e-plan-verify-full.{json,stdout,stderr}. After report edits, rechecked
  all 94 bound files, 955 live hashes, 180 candidate hashes, six executables and
  the report's nine/root ledger's three local links. All handles are terminal.
- This checkpoint is PROGRESS. Next add durable regressions and qualify default
  features, downstream, target/u128 behavior and representative sizes before
  retention; resolve control tradeoffs without dropping unfavorable evidence.
  Continue remaining ARF/scalar/MPFR high helpers, generic field/matrix/algebraic,
  every original reference and full inventory reconciliation. All older failed
  evidence, snapshots, crash core and unfinished requirements remain preserved.

## Checkpoint 43 — qualified and retained e planner; ARF implementation source closure

- Previous turn is PROGRESS: isolated proof/numerical/state, matched CPU and
  allocation evidence, both-profile regressions and the 42-checkpoint verifier.
  Revalidated all 94 bound files, 955 live hashes, 180 candidate hashes, six
  binaries and the terminal 42-row verifier before continuing. About 21 GB is
  available on /tmp; reuse the offline Rust build cache and preserve evidence.
- Kept checkpoint 42 immutable. Prepared one 956-file qualified candidate,
  starting from a 955-file / 45,446,168-byte source copy, and reused the existing
  baseline. Added four permanent tests in a 112-line module and four cfg(test)
  registration lines. The production planner is byte-identical to checkpoint 42;
  it adds eight net lines. No numerical series, splitting, rounding, shared
  cache, representation or runtime dependency change.
- Live Hyperreal has pre-existing tracked/untracked changes, all included in
  the retained hash inventory where in scope. Preserve them. No production
  edit occurred before qualification. Subsequently applied only constants.rs,
  the registration in approximation.rs and the new e_plan_tests.rs module.
  All other prior source hashes and pre-existing user changes remain unchanged.
- Both profiles: baseline default 752 tests, candidate default 756, candidate
  all-feature 859. The four new names account for the only membership difference;
  zero ignored/failed scalar tests. Candidate passes 24 doctests, formatting,
  strict all-target/all-feature Clippy, default checks and fuzz compilation.
  The tests independently compare exact factorial thresholds and directed e
  enclosures through 262144 bits, plus public refinement/coarsening and
  cancellation/conditional-serde recovery without assuming exclusive cold cache.
- Both unchanged consumer snapshots pass the same complete release all-feature
  memberships: 803 Hypersolve, 1764 Hypercurve, nine prior ignored curve tests
  unrun. Last candidate curve test finishes 2026-09-10T01:28:38.269Z; consumer
  Clippy finishes 01:29:10.265Z. Supported scalar/consumer WASM library builds
  pass; no full CI/default-only downstream/browser-demo claim.
- WASM numerical execution in Node22/V8 checks 4151 exact plans and 300 complete
  integer outputs per variant: kernel, public refinement/coarsening, exp(1),
  and fresh module instances. Independent BigInt uses an exact series recurrence
  and complete geometric remainder. All 8302 plans and 600 enclosures pass;
  300 outputs match across variants, and all 186 direct kernel outputs match
  native checkpoint 42. No physical ARM/RISC-V, WASM thread/cancel/serde or
  arbitrary-state qualification. Functional gate finishes 01:22:58.292Z.
- Native targeted follow-up: ten predeclared groups, 40 alternating ABBA/BAAB
  blocks, 1600 observations, CPU6. All three earlier slower controls included;
  original 71-group evidence is preserved, not replaced. Six intervals below
  one, one above, three overlap, ratio range 0.241813..1.027059. Fresh e medians
  at 65536/262144 bits: 3.6496 to 1.57498 ms / 38.03495 to 9.07335 ms.
  Deep coarsening remains slower at 1.027059 [1.023461,1.029521], about 59ns/query.
  Gate finishes 01:46:45.445Z; per-group bootstrap intervals are unadjusted.
- WASM timing: 22 predeclared groups, 12 alternating blocks, 1056 observations,
  CPU6, module instantiation/GC/setup outside timing and documented wrapper/drop
  costs inside. Twelve intervals below one, one above, nine overlap. Cold public
  e medians at 65536/262144 bits: 8.08497 to 3.84452 ms / 82.27973 to 31.12011 ms.
  Warm4096 costs 1.053223 [1.030935,1.071027], approximately 2.9ns/query more.
  Gate finishes 01:48:44.084Z. Module linear memory agrees; not live-heap/RSS.
  Both native and target cached-query costs are accepted and disclosed, not
  dismissed. Strong cold/uncached and allocation-demand gains justify retention;
  public/kernel peak and live demand remain unchanged from checkpoint 42.
- Built, stripped, measured and ran three unchanged native examples per variant.
  Hyperreal readme_quickstart is byte-identical; Hypercurve basic/arrangement
  shrink 944/928 stripped bytes (original files shrink 272/208). WASM audit
  module shrinks 507 bytes. Not whole-Alumina/all-feature/LTO/runtime measures.
  Initial app-size capture rejected an underscore tag before opening outputs
  or spawning strip. Preserved original script and already-built executable;
  corrected tag-only/resumable script reuses its validated directory and build.
- Fourteen dedicated binaries total 108188255 bytes: native examples106789744
  in /tmp/calcium-e-qualified-apps.AHiwfn and WASM1398511 in
  /tmp/calcium-e-qualified-wasm.cxqM1G. Reused offline Rust build cache, whose
  growth is separate from snapshots. About 19GB available on /tmp. No cleanup,
  deletion, donor edit, commit, push or external report; all old evidence retained.
- Read 18 complete ARF implementation files, 1176 new lines: abs_bound_lt_2exp_si,
  call_mpfr_func, ceil, cmp, debug, div, equal, floor, frexp, inlines, is_int,
  is_int_2exp_si, nint, randtest, root, rsqrt, sqrt and urandom (.c). All41 top-level
  ARF C files now source-complete, not all tests/header/recursive GMP/MPFR support.
  Calcium 374 complete/1 partial/37085 lines; FLINT640 complete/17 partial/93640
  lines; combined1014 complete/18 partial/130725 lines. Append-only read records
  preserve all older partial credit. No new independent numerical campaign for
  these 18 source files. Dyadic rounding, NaN convention, nearest-even vs Hyper
  ties-away and root-domain differences do not justify another scalar transfer.
- Corrected draft Hyper read range4170..4205 to4170..4200 after checking the
  physical end. Original unverified draft and its three bound script versions
  are preserved. Corrected full draft verification passes 02:09:25.581Z before
  retention. Then the live tree passes four focused tests and all859 all-feature
  library/integration tests in both profiles, plus formatting; last live gate
  finishes 02:11:49.459Z. New retained source map matches all956 qualified files.
- Final e-qualified-experiment.json binds211 files,55 successful gates,18 full
  read records,956 live hashes and14 binaries. Inspection found that historical
  verifiers still unconditionally read their old Hyper ranges from live sources.
  Added15 versioned modules (82863 bytes) with only explicit frozen-source paths
  and import substitutions. Original scripts, manifests, every math assertion,
  failed gate and raw observation remain unchanged. The new binding verifies
  the old955-file snapshot separately from the current956-file map pre/post.
- All43 checkpoint verifiers pass at 2026-09-10T02:25:08.473Z, exit0, empty
  stderr. results/e-qualified-verify-full.{json,stdout,stderr}:44 records/120223
  stdout bytes (43 checkpoints plus one snapshot-binding record). The first42
  records/112213 bytes are byte-identical to the prior successful run. This is
  integrity recomputation, not a rerun of every historical numerical executable.
- Updated the root report and checkpoint README. After documentation edits,
  rechecked all211 bound files,955 historical/956 live sources,15 versioned
  modules,14 binary identities and the report's11/root ledger's3 local links.
  Scoped production diff passes whitespace checks. All launched handles are
  terminal; no verifier/test/benchmark process remains from this checkpoint.
- This checkpoint is PROGRESS, not ecosystem completion. The fifth continuation
  transfer is retained. Continue unread ARF tests/header and recursive MPFR/GMP
  high helpers with bounded independent oracles as warranted, then generic
  field/matrix/algebraic and all original formal/symbolic/historical references.
  Full inventory reconciliation and every older open experiment remain required.

## Checkpoint 44 — ARF contracts and test-source closure

- Previous turn is PROGRESS: fifth transfer retained, 43-checkpoint historical/
  current verification passes, report and source coverage updated. Revalidated
  all211 bound files,956 live source hashes,15 versioned modules,955 historical
  snapshot identities and the terminal verifier before continuing.
- About19GB remains on /tmp. Reuse native libraries and offline build caches;
  preserve all prior binaries, failed gates, snapshots and crash evidence.
- Continue the unread ARF public-header/manual ranges and tests, comparing
  exact finite rounding, comparison, division and root contracts with Hyper.
  Add bounded independent numerical qualification where it closes a real
  evidence gap; no malformed/invalid-memory or historical crash reproduction.
  No new production transfer or whole-ecosystem completion is assumed.
- Read all47 remaining test/driver files and the442/403 remaining header/manual
  lines:5783 disjoint ARF lines. All104 inventoried files in src/arf/, src/arf.h
  and doc/source/arf.rst are now source-complete. Read66 additional support lines
  in ulong_extras.h70..126 and ulong_extras/randomisation.c35..43; those two
  files remain partial. The earlier attempted randint.c path did not exist and
  receives no credit. Total51 new read records/5849 lines;47 new complete files,
  two completed existing files and two new partial files. Combined Calcium/FLINT:
  1063 complete/18 partial/136574 uniquely counted lines. Recursive support and
  every other original reference remain separate requirements.
- Source-level test gaps: add/sub overwrite randomly selected modes with
  truncation; add_si's random upper limit1 makes its in-place branch unreachable;
  get_d's first/third limit4 switches make nearest-even defaults unreachable;
  root/sqrt/rsqrt references share MPFR and omit nearest-even. The helper's
  bitmask semantics prove the unreachable cases without relying on random runs.
  Approximate-dot reference absolute-product bounds index both vectors using
  revx, ignoring revy for the second. These are qualification gaps, not newly
  proved library numerical or memory defects. No upstream test or donor patched.
- Hyper comparison reads cover certified ties-away integer rounding versus
  multivalued near_integer, negative odd roots, guarded integer/Newton square
  roots, exact-power nth-root enclosures and sticky round-to-odd dyadic f64
  conversion. Existing positive/negative half-tie and wide-dyadic GMP tests
  cover its distinct contracts. No nonredundant production candidate identified;
  do not infer a performance claim from the audit harness's elapsed time.
- New bounded native oracle:543 signed arithmetic fixtures (add/sub/div),180
  positive-root fixtures (degrees1,2,3,5,7,17 and reciprocal square root),14
  precisions1..257 including word neighbors, all five modes and public aliases.
  Includes exact root midpoints (odd^k+delta)/2^k with delta−1/0/+1, exact powers,
  finite exponent/bit-pattern contrasts, cancellation and zero numerators.
  No zero denominator, invalid/raw-memory or excessive exponent probe.
- GMP exact rational scaling, integer quotient/root and halfway-power oracle
  passes518490 full results/flags,316260 aliases and64518 preservation checks;
  64860 exact results and890 repeated halfway ties. Native gate finishes
  2026-09-10T02:45:08.576Z. Output has40447 rows/18296286 bytes including summary.
- Independent BigInt regenerates every fixture and validates202230 primary
  neighboring/halfway-power certificates and every alias result in full. All
  518490 outputs pass, including103698 nearest-even checks. It uses no MPFR
  output oracle or GMP root-search algorithm. Gate finishes02:47:45.324Z.
- Focused Memcheck finishes02:45:50.266Z: zero errors/suppressed/live blocks;
  all1750780 allocations freed,37381718 cumulative bytes including oracle/setup.
  Native/Memcheck numerical stdout is byte-identical. This is finite sequential
  native64 ADX/FE_TONEAREST qualification, not all FENV/targets, peak RSS or
  donor-only allocation/performance. Prior failed evidence remains preserved.
- One28392-byte executable in /tmp/calcium-arf-contracts.PNgYf4, reusing the same
  five shared libraries and native configuration. Paired numerical logs total
  36592572 bytes in the workspace. No Rust/whole-native rebuild or source copy;
  about19GB remains on /tmp. All956 retained source/support hashes unchanged;
  five prior continuation transfers remain retained. No new production/donor
  edit, cleanup, deletion, commit, push or external report.
- arf-contract-experiment.json binds27 files, five successful captured gates,
  51 read records,956 live hashes, five libraries, native configuration and the
  executable. Full44-checkpoint verification passes at2026-09-10T02:54:02.872Z:
  exit0, empty stderr,45 records/124861 stdout bytes (44 checkpoints plus the
  historical/current binding record). Its first44 records/120223 bytes are
  byte-identical to checkpoint43. Capture:
  results/arf-contract-verify-full.{json,stdout,stderr}. All handles terminal.
- Updated the report and checkpoint README. Post-documentation checks match
  all27 bound files,956 live sources, five libraries/configuration, the executable
  and effective coverage. The report's12/root ledger's3 local links resolve.
  Scoped Hyperreal whitespace checks pass; no new production changes this turn.
- Next independently qualify the remaining finite conversion, integer-rounding,
  comparison and approximate-dot contracts where existing evidence is weak.
  Continue recursive scalar/high-product helpers, generic field/matrix/algebraic,
  all formal/symbolic/historical references and full inventory reconciliation.
  This source closure/numerical qualification is PROGRESS, not audit completion.

## Checkpoint 45 — finite conversion and integer contracts

- Revalidated checkpoint44's bound artifacts,956 live hashes, libraries and
  configuration, executable, effective coverage and terminal full verification.
  The104-file ARF source slice is complete; the full ecosystem is not.
- Close the explicit nearest-even binary64 test gap with bounded finite exact
  numerical oracles, then integer rounding, decomposition and comparisons as
  justified. No malformed inputs, invalid memory contracts or old crash reruns.
- Reuse the current FLINT libraries; about19GB remains on /tmp. Preserve all
  earlier evidence and production edits. No new production transfer is assumed.
- Rereading ARF get.c adds no source credit. Initial guesses fmpz/get_d.c and
  fmpz/get_d_2exp.c did not exist; actual definitions are in fmpz/get.c. Failed
  read-only path lookups add no coverage.
- Added all238 lines of fmpz/set.c, including inline/heap promotion, bounded
  floating and GMP imports and source-only raw-array contracts. fmpz/get.c and
  the ARF conversion/integer/comparison/decomposition rereads were already
  covered and add no credit. Another guessed arf/int.c path did not exist.
  New combined coverage:1064 complete/18 partial/136812 unique read lines.
- Bounded finite corpus:2401 signed dyadic fixtures, mantissas through257 bits,
  peak exponents−1100..1100 and explicit midpoint/quarter-grid neighbors, all
  five modes. Exact GMP grid oracle; independent BigInt monotone binary64
  bit-search and exact midpoint/integer certificates regenerate every input
  and check complete outputs. No MPFR output oracle or residue-only comparison.
- Original harness native exit1 at03:08:19.762Z: exactly one wrong expectation
  for zero's signed magnitude exponent. Source/manual specify−ARF_PREC_EXACT;
  two fmpz magnitude bounds leave zero unspecified. Preserve original source,
  controls executable and failed gate. Mechanically derived v2 changes only
  that block: skip the two unspecified zero calls, check the documented signed
  sentinel, serialize it exactly as decimal text. Every non-bound numerical
  result matches v1; no fixture dropped and no donor defect inferred.
- Corrected native finishes03:10:30.644Z, exit0. Independent checker finishes
  03:12:46.453Z, exit0:117705 assertions,12005 full binary64 results including
  2401 nearest-even and164 midpoint fixtures;11281 finite imports;12005 integer
  outputs/flags,7985 proved-in-range signed conversions,14406 floor/ceil/nint
  outputs and9604 public aliases including frexp. Decomposition, integrality,
  bounds, signed/absolute power/previous-value comparisons and preservation
  pass. There are1848 subnormal outputs,460 negative zeros,724 overflow
  infinities; counts repeat modes/routes and are not independent identities.
- Focused Memcheck finishes03:10:39.199Z: zero errors/suppressed/live blocks;
  all133965 allocations freed,3847142 cumulative bytes including oracle/setup.
  Native/Memcheck stdout identical2781706 bytes each. Native64 ADX and hardware
  FE_TONEAREST only, not all targets/FENV, donor-only cost or peak/RSS metrics.
  No old numerical/FENV/LLL/thread-memory failure is rerun or discarded.
- Hyper comparison confirms existing exact/lossy dyadic boundaries, sticky
  round-to-odd, certified ties-away integer operations and total multivalued
  near_integer. Finite ARF nearest-even/ordering is not a replacement. No new
  production candidate, transfer or benchmark claim. The956 live source hashes
  match retained checkpoint43; all five continuation transfers remain intact.
- Two binaries in /tmp/calcium-arf-conversion.ZnmLMf: original28664 bytes,
  corrected28672 bytes, total57336. Three numerical logs total8340310 workspace
  bytes. Existing five native libraries/configuration reused; no Rust/whole-
  native build or source copy. About19GB remains on /tmp. No cleanup/deletion,
  donor/production patch, commit, push or external report.
- arf-conversion-experiment.json binds38 files, eight captured gates (seven
  successful and the preserved one-failure original), one source-read record,
  956 live identities, shared libraries/configuration and both executables.
  Full45-checkpoint verification passes2026-09-10T03:17:12.671Z: exit0, empty
  stderr,46 records/129552 bytes. First45 records/124861 bytes exactly match44.
  Capture:results/arf-conversion-verify-full.{json,stdout,stderr}. All handles
  terminal. Original/generated harness derivation is rechecked mechanically.
- Updated root report and checkpoint README. One rejected report patch had
  unmatched context and made no edit; the corrected patch applied normally.
- Post-documentation revalidation matches all38 bound artifacts,956 live files,
  five libraries, one configuration file, both executable hashes/sizes and
  effective coverage. Terminal45 verification and124861-byte predecessor prefix
  match. All13 report links and three root-ledger links resolve; scoped audit
  and Hyperreal whitespace checks pass. No new production edits this checkpoint.
- Follow-ups remain add_si alias and approximate-dot reference pairing, fixed-
  grid wrappers and other uncovered scalar contracts where useful; recursive
  support, generic field/matrix/algebraic work, all original formal/symbolic/
  historical references, unresolved experiments and inventory reconciliation.
  This qualification is PROGRESS, not full ecosystem completion.

## Checkpoint 46 — multivariate rational-function architecture

- Checkpoint45 is PROGRESS: independent finite conversion qualification,
 238 new donor lines and updated report. Revalidated38 bound artifacts,956
 live hashes, libraries/configuration, both executables, coverage and terminal
 full45 verification before continuing. No earlier live session needs restarting.
- Next substantive slice: current FLINT fmpz_mpoly_q implementation, header,
 manual and tests; compare canonicalization/cross-cancellation with Hyper's
 rational and retained algebraic-field paths. Archived Calcium equivalents,
 generic wrappers, recursive polynomial support and all other original targets
 remain required, not declared complete by this slice.
- Keep finite valid-input numerical checks independent where needed; no parser,
 invalid-memory or old crash reproduction. Reuse existing libraries/caches and
 preserve all original evidence. Do not infer a production transfer in advance.
- A guessed coverage-notes.md path did not exist. Broad inventory/search output
 was truncated and is used only for navigation, never as source-read credit.
- Read all36 current fmpz_mpoly_q implementation/test/header/manual files,
 3564 lines; the complete archived201-line manual;87 fmpz_mpoly.h excerpts;
 66 test-driver header lines; and the full37-line multiplier helper. Total40
 records/3955 new lines,38 new complete/two new partial files. Combined coverage
 is1102 complete/20 partial/140767 unique lines. Archived implementation,
 generic-ring/fexpr bridges and recursive polynomial/GCD support remain open.
- Architecture: denominator-GCD staged addition/subtraction, restricted second
 cancellation, scalar-content GCD, opposite-side pre-cancellation, alias-aware
 temporaries and leading-sign normalization. All15 upstream tests use ORD_LEX
 and shared polynomial/GCD references; scalar-wrapper tests do not explicitly
 alias the tested wrapper's output. Valid generated strings are not malformed-
 input qualification. No newly proved donor arithmetic defect in this slice.
- Hyper reads confirm integer rational cross-cancellation/possible-divisor
 reduction, shared local-field denominators, allocation-free one and bounded
 residue products. Formal rational functions differ from selected-root values;
 the fiber API retains authored nonzero-denominator obligations and rational-
 image tests already guard cancellation outside an isolated root. No new
 production transfer or matched performance claim justified.
- New finite corpus has20 recipes in each of9 contexts:1/2/4 variables crossed
 with all three monomial orders; common linear factors, scalar denominators,
 cancellation/signs/zero and66/130-bit contents2^65+7/2^129+7. Canonicalize
 valid raw polynomial pairs before operations; all polynomial divisors nonzero.
 Complete outputs include coefficients, flags, used-variable masks and content.
- Independent BigInt regenerates every input, verifies full cross-polynomial
 identities, completely factors corpus denominators into known primitive linear
 factors, rejects common numerator factors and checks integer-content GCD one.
 This gives10044 primary identity/canonicality certificates and checks all23841
 values including13797 complete aliases. Final full input emissions preserve
 inputs. Not a general multivariate GCD or selected-root domain proof.
- Initial compile failed03:32:23.898Z: omitted GMP header. Preserve original
 source and failed gate; no executable existed from that attempt. Separately
 derived corrected source adds only stdio/GMP include prelude and one trailing
 newline. Corrected compile passes03:35:07.035Z; native passes03:35:34.255Z.
 Independent checker passes03:39:05.872Z. No numerical fixture removed.
- Focused Memcheck passes03:36:05.203Z with zero errors/suppressed/live blocks;
 all2797045 allocations freed,75163303 cumulative bytes including checks/setup.
 Native/Memcheck stdout identical3087444 bytes each,6174888 paired workspace
 bytes. Not donor-only allocation, peak/RSS, all-target or performance evidence.
- All15 upstream tests additionally pass natively03:40:46.806Z, no filters,
 explicit FLINT_TEST_MULTIPLIER=1. Source-read driver and multiplier confirm
 all registered tests execute. Exact pass membership verified03:42:43.200Z.
 These include valid generated string roundtrips; no malformed-input probes.
 Upstream suite is not the focused Memcheck target. Original failed numerical/
 FENV/LLL/thread-memory campaigns remain preserved and unrerun.
- Existing native libraries/configuration reused. Two executable sizes24280
 and36560 bytes,60840 total, in/tmp/calcium-mpoly-rational.Hmk8Is. About19GB
 remains on/tmp. No Rust/whole-native rebuild, source copy, cleanup/deletion,
 production/donor edit, commit, push or external report.956 live hashes remain
 unchanged and all five retained continuation improvements stay intact.
- mpoly-rational-experiment.json binds46 files,10 captured gates (nine successful,
 one preserved failed compile),40 read records,956 live sources, five libraries,
 configuration and both executables. Full46-checkpoint verification passes at
 2026-09-10T03:47:27.858Z: exit0, empty stderr,47 records/135764 bytes. First46
 records/129552 bytes exactly match checkpoint45. Capture:
 results/mpoly-rational-verify-full.{json,stdout,stderr}. All handles terminal.
- Updated report/root status and checkpoint README. One report patch had a
 mismatched no-op context and was rejected without edits; corrected patch applied.
 Earlier guessed split rational-ops filenames did not exist; actual ops.rs
 ranges were subsequently located, read and bound. No false coverage credit.
- Post-documentation revalidation matches all46 bound artifacts,956 live files,
 five libraries, one configuration file, both executable hashes/sizes and
 effective coverage. Terminal46 verification and129552-byte predecessor prefix
 match. All14 report links and three root-ledger links resolve; scoped audit,
 Hyperreal and Hypersolve whitespace checks pass. No new production edits.
- Next archived rational-function implementation/tests, generic-ring/fexpr
 bridges, recursive polynomial/GCD and remaining scalar contracts, then every
 other original formal/symbolic/historical target, unresolved transfer and full
 inventory reconciliation. This checkpoint is PROGRESS, not audit completion.

## Checkpoint 47 — archived rational functions and expression bridges

- Previous turn is PROGRESS: checkpoint46 report/ledger and verified evidence
  were completed. Revalidated all46 bound artifacts,956 live sources, libraries,
  configuration, executables, effective coverage and terminal full46 capture.
- Read archived Calcium rational-function implementation/tests/header and both
  generations of expression bridges, plus the current generic-ring adapter and
  test. Compare mathematical domains and cancellation contracts before proposing
  any transfer. Preserve earlier failures and all donor/production files.
- Initial broad filename discovery was truncated; navigation only, no read credit.
  No applicable AGENTS.md found along the audit/donor ancestor paths. About19GB
  remains on/tmp; reuse libraries and avoid whole-workspace rebuilds.
- Read all44 archived rational-function implementation/test/header files,
  3564 lines; the manual was already read at46. Both rational-expression bridges,
  current generic adapter/test and four fexpr helpers add10 complete files.
  With96 manual and52 generic-header excerpt lines:56 records/5029 new lines,
  54 complete/two partial additions. Combined1156 complete/22 partial/145796
  unique lines. Recursive support and every original outstanding target remain.
- A draft recorder omitted the inv.c note and stopped before any output write.
  Added the already-read file's note; the single successful append records all56
  ranges with matching donor hashes. A later broad Hyper search was truncated;
  actual bound source ranges were read separately. No false read credit.
- Archive/current comparison confirms staged denominator/content cancellation;
  current FLINT adds the archive's TODO scalar-denominator paths. Hyperreal
  already stages denominator GCD and restricts residual cancellation. Hypersolve
  retains unit/shared denominators and explicit selected-root domain obligations.
  Formal expression normalization does not prove terminal independence or
  preserve authored denominator domains. No new production candidate selected.
- Native finite expression/generic-ring corpus passes04:42:50.071Z; independent
  BigInt checker passes04:51:21.271Z.2052 complete polynomial/canonicality
  certificates cover7308 values,2142 generic-operation aliases,1062 power rows,
  360 part-extraction rows,180 roundtrips and90 authored identities, plus inputs/
  preservation.1/2/4 variables, all three orders and small signed powers only.
  No malformed inputs, boundary/resource probes or old failure reproduction.
- Focused Memcheck passes04:43:19.739Z with zero errors/suppressed/live blocks,
  all603105 allocations freed,11974118 cumulative bytes including checks/setup.
  Native/Memcheck numerical stdout identical864384 bytes each. No donor-only
  timing/allocation/peak/RSS or archived/general-generic-suite qualification.
- Reused existing five libraries/configuration. One28488-byte executable in
  /tmp/calcium-mpoly-bridge.7pQl1d; no Rust/whole-native rebuild, source copy,
  cleanup/deletion, production/donor edit, commit, push or external report.
  All956 live source hashes unchanged; five retained improvements preserved.
- mpoly-bridge-experiment.json binds30 artifacts, five successful captured gates,
  56 read records, all956 live hashes, libraries/configuration and executable.
  Generated C/helper-math prefixes are byte-verified against unchanged46 files.
  Full47 chain ran04:55:37.481Z..04:56:34.285Z and passes: exit0, empty stderr,
  48 records/139322 bytes, first47 records/135764 bytes exactly match46.
  Capture results/mpoly-bridge-verify-full.{json,stdout,stderr}. All handles
  terminal. Updated root report/status and checkpoint README; no retained change.
- Post-documentation revalidation matches all30 bound artifacts,956 live files,
  five libraries, one configuration, executable hash/28488-byte size, effective
  coverage and terminal full47 capture with135764-byte unchanged46 prefix.
  All15 report and three ledger links resolve. Scoped audit, Hyperreal and
  Hypersolve whitespace checks pass; latest report/status/README agree on47.
- Continue recursive polynomial/GCD support, generic/default conversion helpers,
  remaining fexpr/algebraic paths, every original uncompleted reference and
  unresolved transfer, then full inventory reconciliation. This is PROGRESS,
  not full ecosystem completion. About19GB remains free on/tmp.

## Checkpoint 48 — expression representation and numerical interfaces

- Previous turn is PROGRESS:54 complete/two partial source additions, independent
  expression qualification, full47 verification and report updates. Revalidated
  all30 bound artifacts,956 live sources, libraries/configuration/executable,
  terminal47 capture and effective coverage before continuing. No live handle.
- Continue the remaining fexpr representation, conversion and numerical code in
  both pinned generations; keep formatting and recursive support explicitly open
  until their lines are read. Compare immutable/flat storage and exact-versus-
  enclosure contracts with Hyper before proposing any transfer. Reuse libraries;
  preserve existing files and failed evidence. About19GB remains free on/tmp.
- Finished the pending current replacement test and five archived tests. Together
  with the earlier48 reads, both headers/manuals and every file in both fexpr
  directories except write_latex.c are read.88 records credit8321 new lines:
  87 new files and completion of the current manual's partial record. Exact
  coverage now1244 complete/21 partial/154117 unique lines; source hashes match.
  No credit for truncated navigation, unread LaTeX implementations or builtin
  headers/tables. Two previously truncated manual chunks were reread completely.
- Flat tagged syntax copies complete children; indexed arguments and borrowed
  replacement views do not supply DAG sharing or cached numeric evaluation.
  Exact rational/dyadic constructors overlap existing Hyper capabilities.
  Capped decimal retry returns evaluation success separately from requested
  accuracy and omits the radius; not a certified approximation-query replacement.
  Formal normalization still cannot discharge authored/selected-root domains.
- Compared actual Hyper representation.rs1..330 and README1..170. The code uses
  a lazily allocated RwLock single-finest cache; README lock-free-swap wording is
  stale and is not used for a lock-free claim. No bound production file changed.
- No plausible new transfer selected. This is explicitly source-only: tests read,
  not run; no new benchmark, numerical, allocation, Memcheck or size claim. Prior
  numerical results retain their own corpus/snapshot limits. No old failure
  reproduction, donor edit, new/tmp file/build, cleanup/deletion, commit or push.
- New recorder preflights all inputs and appends only new ranges. Manifest binds
  ten artifacts,88 donor hashes and records, and points to unchanged47 evidence.
  Source/evidence verifier passes2026-09-10T05:13:02.958Z: exit0, null signal,
  checking all956 live files,30 previous bound files, libraries/configuration,
  existing executable and terminal47 capture with unchanged46 prefix. Capture:
  results/fexpr-representation-verify.{json,stdout,stderr}. No full historical
  chain rerun; latest full47 capture remains04:56:34.285Z. All handles terminal.
- Condensed the report executive evidence summary while preserving detailed
  qualification sections; added48 findings and updated README/root coverage.
  All five continuation transfers remain retained. Entire ecosystem remains
  incomplete; continue formatting/builtin and recursive support, then remaining
  formal/symbolic/historical references and full inventory reconciliation.
- Post-documentation source/evidence recheck passes with96 selected expression
  files complete,88 new records and all956 live hashes unchanged. All17 report
  and three ledger local links resolve; report/ledger/findings whitespace and
  scoped audit diff checks pass. No/tmp growth from this checkpoint;19GB free.

## Checkpoint 49 — expression formatting, builtin symbols and streams

- Previous turn is PROGRESS:88 expression read records/8321 unique new lines,
  source/evidence verification and report updates. Revalidated48 bindings,
  all956 live sources and unchanged47 evidence; no active handle remains.
- Continue the remaining LaTeX formatting implementations, builtin headers,
  tables/lookup and manuals in both pins. Track actual read ranges without
  crediting navigation or inferred source similarity. No numerical or benchmark
  claim from source review alone. Preserve all earlier evidence and production
  files; about19GB remains free on/tmp, with no new build planned.
- Read both full4096/4103-line LaTeX implementations; both builtin headers,
  tables, lookup/inline files and1293-line manuals; then both calcium support
  directories and global headers. Current header already complete, archived
  header76..108 already credited; new archived credit1..75/109..183 only.
  One combined output exceeded its budget by74 tokens; reread archived2101..2160
  before credit. No source similarity or truncated-navigation credit.
- 25 new records add13667 unique lines:24 new complete files and completion of
  one old partial. Combined1269 complete/20 partial/167784 lines. Closed combined
  expression/builtin/support slice is60 archived/62 current files and22843 lines.
  Config template200..226 and generatedconfig201..227 are separately hashed
  auxiliary reads, not extra donor coverage; existing inventory stays unchanged.
- Both474-entry symbol tables match exactly in IDs/spelling/LaTeX/callbacks.
  Independent static metadata checker validates enum indexes, ASCII ordering,
  405 documented names and35 callback definitions used by269 rows. This is not
  C runtime execution, mathematical implementation coverage or safe-formatting
  qualification. No arbitrary-syntax test, old failure or memory-error repro.
- Formatter uses borrowed views/geometric string growth but also child strings,
  endpoint substitution and explicit formal normalization. Builtin metadata
  directly couples lookup with display callbacks. Current helper internal
  linkage/Os configuration are source choices, not measured Hyper size gains.
  Displayed assertions and symbolic vocabulary are not proofs or implemented
  exact-real closure. Output/escaping/ownership concerns remain source-only.
- Compared Hyper computable/format.rs1..330 and real/arithmetic/format_parse.rs
  1..106: typed literal output and bounded fixed-decimal demand already exist;
  scientific formatting still uses iter_msd. No all-modes productivity or
  correct-decimal-ties claim. No new candidate justifies matched benchmarking.
- fexpr-formatting-manifest.json binds11 artifacts,25 donor records/hashes,
  auxiliary configuration hashes, prior48 manifest/capture and metadata result.
  Verifier imports48 and passes2026-09-10T05:26:43.602Z: exit0/nullsignal,
  two records, checking all956 live files and unchanged previous evidence.
  Capture results/fexpr-formatting-verify.{json,stdout,stderr}. This is not
  a full47 historical-chain rerun; that latest capture remains04:56:34.285Z.
- Updated report, root coverage and README. No production/donor change, new
  retained transfer, numerical/Memcheck/benchmark/size claim, new/tmp file/build,
  cleanup/deletion, commit, push or external report. All handles terminal.
  Continue algebraic/qqbar and recursive polynomial/GCD/numerical support,
  remaining formal/symbolic/historical targets and full inventory reconciliation.

## Checkpoint 50 — algebraic representation and exact decisions

- The previous turn completed the requested report summary and revalidated
  checkpoint49, all956 retained live hashes and report links. It added no donor
  coverage or numerical qualification. Resume the open algebraic source work;
  do not count the unchanged-state recheck as a new audit completion.
- Inspect both pinned qqbar representations, enclosure refinement and exact
  equality/sign decisions, then compare their contracts with Hyper. Record only
  actually read ranges. Existing49 evidence and all production files remain
  unchanged; no subagents, broad rebuild or cleanup is planned.
- Completed92 new full-file reads /8646 unique lines:46 archived/4321 lines,
  46 current/4325 lines. Both full headers/manuals,34 implementation files and
  ten test files per pin. Combined1361 complete/20 partial/176430 lines. All
  source outputs were untruncated; truncated rg navigation was not credited.
- Compared Hyperreal structural_analysis.rs1..270, approximation_queries.rs
  330..460, real/arithmetic/comparison.rs1..81; Hypersolve algebraic.rs365..940
  and3570..3645. Existing bounded algebraic separation and represented-root
  proof/refinement/difference machinery already cover the useful patterns.
  No new production candidate or matched benchmark is justified.
- Added a valid-input collector for49 signed-square-root complex values,
  all4802 ordered pairs across cold/explicitly cached states,294 polished
  enclosures at32/128/512 bits, copies and readonly input preservation.
  Independent BigInt signs/squares certify endpoints and derive exact orders;
  no qqbar-based oracle, primitive square-root oracle or overlap-only check.
- Mathematical gate:35826 assertions,35562 pass/264 fail. Root ordering fails
  equal-nonreal reflexivity in84 same-object plus84 copy queries (42 values,
  two states). Exact-component polishing fails96 outputs for16 values with
  a +/-1 component and an irrational other component. These are two contract
  issues, not264 defects. All588 component containment and588 prec-2 accuracy
  checks,28812 ordinary pair comparisons, signs/copy/hash/preservation pass.
  No wrong enclosure, distinct-value root order or root list is demonstrated.
- Native collector terminal0 at05:41:13.296Z; independent math checker terminal1
  at05:41:30.898Z. Memcheck terminal0 at05:42:08.592Z, zero errors/suppressed/
  live blocks,2729471 allocations/frees and154454444 cumulative requested bytes
  including setup/queries/output. Native/Memcheck stdout byte-identical658089
  each; collection/memory success does not repair mathematical failures.
- One18152-byte executable at/tmp/calcium-qqbar-decisions.xUeflv/controls reuses
  existing five libraries and configuration; linked paths captured separately.
  No broad rebuild, Rust build, source copy, cleanup/deletion or old failure repro.
- Initial evidence recorder failed before writing coverage/manifest because
  raw JavaScript optional undefined fields/-0 differ from parsed JSON. Preserved
  both initial scripts and full failed capture. Corrected only binding to compare
  JSON wire forms; original C/checker and every mathematical assertion unchanged.
- qqbar-decisions-manifest.json binds33 artifacts,92 source records/hashes,
  five execution/check gates (math gate1, others0), initial bookkeeping failure,
  live956 sources through the retained manifest, libraries/config and executable.
  Source/evidence verification passes05:49:16.590Z, terminal0, while explicitly
  verifying the264 mathematical failures. It imports49/48, not the full47 chain;
  latest full47 capture still04:56:34.285Z. All handles terminal.
- Report and README updated. All five retained transfers unchanged. Continue
  algebraic arithmetic/relations/field expressions, generic adapters and recursive
  polynomial/root-finding support, every remaining original target and full
  inventory reconciliation. This checkpoint is PROGRESS, not goal completion.
- Post-documentation verification passes again: all956 retained live hashes,
  source records, captures and preserved failures match. Captured verifier has
  three records/3611 stdout bytes, exit0/nullsignal and empty stderr. All20 local
  report links and three ledger links resolve; scoped whitespace checks pass.
  About19GB remains free on/tmp. No active session or outstanding tool handle.

## Checkpoint 51 — algebraic arithmetic and relation boundaries

The initial read/report-handoff entries below are preserved chronologically.
The later qualification entries bind those reads and the completed selected
source slice; the Hyper power-sum candidate remains open.

- Previous goal turn is PROGRESS:92 new complete reads/8646 unique lines and
  independent algebraic qualification, with two contract issues preserved.
  Revalidated50 source/evidence bindings and all956 retained live files before
  continuing. No active process or missing terminal handle remains.
- Continue both pinned qqbar arithmetic implementations, including fused affine
  transforms, composed annihilators, cancellation, powers and polynomial/field
  relation boundaries. Compare the useful algorithms with current Hyper before
  selecting a candidate. Keep old failures and all production changes intact.
- Read progress before the report-only handoff: the following full files were
  actually read in untruncated output. Paths are `qqbar/<name>` in archived
  Calcium and `src/qqbar/<name>` in current FLINT. Ranges start at line1 and end
  at the listed physical EOF. Their bytes were rechecked against the existing
  pinned inventory during reporting; this metadata check is not a source read.

  | Name | Calcium EOF | FLINT EOF |
  | --- | ---: | ---: |
  | affine_transform.c | 170 | 171 |
  | composed_op.c | 364 | 364 |
  | add.c | 96 | 95 |
  | sub.c | 145 | 145 |
  | mul.c | 149 | 150 |
  | div.c | 191 | 191 |
  | inv.c | 79 | 78 |
  | mul_2exp_si.c | 64 | 63 |
  | pow.c | 337 | 338 |
  | fmpq_pow_si_ui.c | 49 | 50 |
  | guess.c | Not yet read in checkpoint 51 | 125 |

- These 21 files / 3,414 lines are provisional checkpoint 51 progress, not yet appended
  to the coverage extension or bound by a new checkpoint manifest. Completed
  coverage remains 1,361 files / 20 partial / 176,430 unique lines through 50. Both
  pinned hashes remain those in inventory.json; no new source version is used.
- Source observations: affine transforms preserve root identity through exact
  polynomial substitution plus enclosure validation. Composed operations build
  annihilators through power sums (Hadamard products for products/quotients,
  factorial-scaled convolution for sums/differences), then factor and certify
  the selected root. Rational guesses are candidates requiring exact checking;
  overlap or a zero-containing residual alone is not an identity proof. Powers
  exploit rational/surd/root-of-unity cases and polynomial deflation before
  generic evaluation. These observations are not runtime qualification.
- Open transfer candidate: compare power-sum/Newton/Borel annihilator
  construction against the live hypersolve/src/algebraic_binary.rs resultant
  kernel. Do not dismiss it as already covered without that comparison. A
  bounded independent Q(sqrt(2),sqrt(3)) coefficient/conjugate oracle could
  qualify full polynomials and root embeddings before any isolated prototype
  or matched benchmark. No collector, checker, prototype or benchmark for 51
  has yet been implemented or run.
- Next unread source: both polynomial evaluators, polynomial-value equality,
  field-expression and acb_lindep implementations, archived guess.c, and the
  associated arithmetic/field/polynomial tests. Continue source and mathematical
  work without rerunning earlier assertion/memory failures or malformed-input
  probes. No new retained production change is claimed.
- Report-only recheck at approximately 2026-09-10T06:00Z: direct
  `node verify-qqbar-decisions.mjs` terminates with exit 0, checking 48/49/50 evidence and
  all 956 live identities, while preserving 35,826 assertions / 264 failures across
  the two already recorded contract issues. This is not a new captured gate,
  full 47-chain rerun, numerical regression, Memcheck or performance experiment.
  Updated the root report to separate qualified checkpoint 50 results from
  provisional checkpoint 51 work. No production/donor edit, new `/tmp` file/build,
  cleanup, deletion, commit, push or external report. About 19 GB remains free
  on `/tmp`; 116 GB in the workspace. All 20 local report links and three ledger
  links resolve, with no trailing whitespace in either root document.

- Continued after the report handoff. Classified the previous turn as PROGRESS
  for the report/ledger update, not numerical qualification. Revalidated 50 and
  the 956 retained live identities before source or experiment work. No active
  process remained from that handoff.
- Finished both remaining polynomial evaluators, polynomial-value equality,
  field-expression, guess and acb_lindep implementations, plus eleven associated
  test files per pin. Together with the provisional reads, this adds 54 full
  files / 7,223 unique lines: 27 archived / 3,654 lines and 27 current / 3,569.
  All source-read output was untruncated. No duplicate credit for the earlier
  21 provisional files. Combined coverage: 1,415 complete / 20 partial /
  183,653 lines (Calcium 552 complete / 60,286 lines; FLINT 863 complete /
  20 partial / 123,367 lines). Coverage extension is append-only.
- Read live hypersolve/src/algebraic_binary.rs 1..1089 and
  integer_interpolation.rs 1..750 completely; resultant.rs 185..340,
  640..740 and 1215..1275. The binary path samples exact resultants through
  flat BigInt Bareiss and factorial-scaled integer interpolation. Power sums
  are a genuine alternative, not already implemented. Preserve reducible/
  repeated carriers, zero-root divisor fallback, degree/policy/validation
  contracts and full output certificates in the next prototype.
- Added a bounded valid-input C collector and independent BigInt biquadratic
  field oracle. Sixteen small exact values, two cache states, binary/unary
  aliases, signed powers, dyadic scaling, affine maps, rational polynomial
  evaluation, full composed annihilators and true/false relation controls.
  No malformed inputs, arbitrary objects, extreme sizes, direct LLL experiment
  or earlier failure reproduction. The 22 upstream tests were read, not run.
- All 10,945 expected records and 40,352 assertions pass: 7,344 minimal
  polynomials/root enclosures each, 1,008 full composed polynomials, 2,560
  relation decisions (44 true / 2,516 false), ordered endpoints, width/real-axis
  checks and 64 readonly-preservation checks. The exact oracle uses Galois
  conjugates, integer-square-root bounds and BigInt cross-products, not qqbar
  operations or overlap. This is a bounded real field corpus, not general
  algebraic closure, branch coverage, archived execution or Hyper qualification.
- Compile terminal 0 at 18:00:56.109Z; native terminal 0 at 18:01:37.896Z;
  independent checker terminal 0 at 18:01:56.413Z; focused Memcheck terminal 0
  at 18:02:12.310Z; linked-library capture terminal 0 at 18:02:54.303Z.
  Native/Memcheck stdout is byte-identical, 2,032,198 bytes each. Memory reports
  zero errors/suppressed errors/live blocks, 3,180,130 allocations/frees and
  150,670,769 cumulative requested bytes including collection/setup/output.
  These are not donor-only performance or peak-memory measurements.
- One 22,960-byte executable at /tmp/calcium-qqbar-arithmetic.oUgkCO/controls,
  SHA256 6852225233a77921f4c5bdd58e726e3a7ef082b078f1ff2783417df9dae0a518.
  Existing five libraries and one configuration file are reused and hashed.
  No broad native/Rust build, source copy, cleanup/deletion or production edit.
- qqbar-arithmetic-manifest.json binds 28 artifacts, 54 read records/hashes,
  five successful gates, library/config/executable identities and 956 unchanged
  retained live files. The captured verifier passes at 18:08:04.634Z, exit 0 /
  null signal, checking the new oracle and 48–50 evidence including the old
  mathematical failures. Capture results/qqbar-arithmetic-verify.{json,stdout,stderr}.
  This is not a full historical-chain rerun; the latest full 47 capture remains
  04:56:34.285Z. All command/session handles are terminal.
- Updated root report and continuation README. All five retained changes stay
  unchanged. The next concrete action is an isolated power-sum/Newton candidate
  with expanded carrier certificates and matched kernel/public-query costs,
  followed by proportional regression/size gates before any retention. Further
  qqbar/recursive support and all remaining original references remain in scope.
  This checkpoint is PROGRESS, not completion of the full audit. About 19 GB
  remains free on /tmp and 116 GB in the workspace; no deletion, commit, push
  or external report occurred.
- Post-documentation verification passes again, including all 956 retained
  live hashes and the new/previous mathematical evidence. The captured verifier
  contains four records / 5,208 stdout bytes, exit 0 / null signal and empty
  stderr. All 22 local report links and three ledger links resolve; scoped
  whitespace checks pass. All session handles are terminal.

## Checkpoint 52 — isolated power sums; shared point-image completeness gap

- Previous turn is PROGRESS: 54 source files / 7,223 new lines and 40,352
  independent arithmetic assertions completed and bound. Revalidated checkpoint
  51 and all 956 retained live files before continuing. No live session remains
  from that checkpoint.
- Prototype integer-scaled power sums and Newton reconstruction in an isolated
  Hypersolve copy. Reuse the existing retained frozen baseline and shared build
  cache; preserve production files and every prior artifact. Keep signed output
  orientation, reducible/repeated carriers, the zero-root divisor fallback,
  degree/policy/validation contracts, and compare full polynomials before timing.
- Implemented integer-scaled root power sums, binomial/Hadamard image sums and
  exact Newton reconstruction with signed primitive orientation. Original
  sampled resultant fallback remains for unused zero-root divisor carriers.
  One existing candidate file changes, one 276-line private module is added.
  All 956 live files and the frozen retained baseline remain unchanged. The
  isolated copy had 45,450,206 logical bytes before editing; shared cache reused.
- Independent polynomial-ring Sylvester/Leibniz oracle passes all 4,840 full
  signed-polynomial cases for baseline and candidate: 4,704 direct, 136 fallback,
  32 zero resultants, 1,210 independently computed determinants. No sampled,
  interpolated, Newton or donor result is used as the oracle. All 19 focused
  debug tests and 805 default-feature debug library/integration tests pass.
  Not a fresh matched baseline, release/all-feature, Clippy, WASM or full CI gate.
- Paired public collectors each emit 6,441 complete records. Independent exact
  rational/Sturm checks: 43,109 pass / 825 fail out of 43,934. All 4,301 returned
  results and 969 exact witnesses pass. The 825 failures are identical baseline
  and candidate InvalidTransformedEvidence results where an arithmetic image
  contains exactly one root. All report a collapsed interval without a witness.
  Source isolation, degree/nonzero guards, 65 nonisolating controls and 40 zero-
  resultant Undecided controls pass; the latter remain a separate open issue.
- Root cause: binary_image_interval always sets exact_root to None, losing
  collapsed image witnesses. root_isolation.rs 693..840 was read and correctly
  requires/replays the witness. This is lost completeness, not a wrong returned
  value or power-sum regression. The oracle uses closed intervals; broader
  half-open endpoint/policy qualification is still needed. Original checker and
  failed mathematical capture remain unchanged, exit 1 at 18:37:41.988Z.
- Candidate focused Memcheck exits 0 at 18:37:55.736Z: zero errors and definite/
  indirect/possible lost bytes; 1,364,464 reachable bytes in 11,605 blocks.
  2,634,556 allocations / 2,622,951 frees / 156,378,772 cumulative requested bytes
  include collection/setup/output. Some stacks show Sturm-cache allocation;
  full ownership/boundedness and a baseline memory comparison remain unqualified.
  Native baseline/candidate/Memcheck stdout match, 4,332,578 bytes each. Not a
  zero-live, peak-memory or timing result. No earlier crash/assertion reproduction.
- Five dedicated executables total 34,189,432 bytes in the bounded
  /tmp/calcium-power-sums.2CGTky directory. CPU and allocation drivers were built
  separately, but NO timing/allocation campaign ran. All ten captures are terminal.
  No broad rebuild, cleanup/deletion, commit/push, external report or donor edit.
- power-sums-manifest.json binds 55 artifacts, 957 candidate source hashes,
  five executable identities and all ten gates, preserving the public failure.
  Captured integrity verification passes at 18:50:23.896Z, exit 0 / null signal;
  rechecks 48–51 and 956 unchanged live files, not the full 47 historical chain.
  No new donor credit: 1,415 complete / 20 partial / 183,653 unique read lines.
  Root report and continuation README updated; all five retained transfers
  unchanged. This is PROGRESS, not full audit completion.
- Next actual action: separate isolated point-image witness repair off the
  retained baseline. Keep exact equality, containment, vanishing, uniqueness,
  policy and degree guards; qualify nonrational/Unknown/half-open controls and
  public/consumer effects before retention. Preserve the unqualified power-sum
  candidate separately. Resume its cost/size qualification only after resolving
  this higher-priority completeness issue. About 19 GB remains free on /tmp.

## Checkpoint 53 — isolated certified point-image witnesses and bounded costs

- Checkpoint 52 made PROGRESS: isolated power sums pass the full-polynomial
  kernel gate, while a shared public completeness failure was identified and
  preserved. All retained live sources and prior evidence remain unchanged.
- A separate candidate starts from the retained frozen baseline, not the
  power-sum candidate. Only certified endpoint equality may synthesize a point
  witness. Reading the predicate resolver shows APPROXIMATE_512 can label an
  interval overlapping zero as approximate equality; use STRICT for witness
  synthesis even when the caller permits approximation. Keep downstream replay.
- Reuse the shared Cargo cache and existing bounded public/baseline drivers.
  Qualify rational/nonrational points, exact-zero collapse, strict/approximate
  Unknown controls, half-open ownership and polynomial replay before retention.
- Initial candidate changes six algorithm lines plus witness assignment and
  adds six regressions. Strict equality supplies the point witness, with all
  containment/vanishing/uniqueness replay retained. Focused 22 tests pass; all-
  feature Hypersolve baseline passes 803 tests per debug/release profile and
  candidate passes 809. The documentation command succeeds with zero doctests.
- Public collector/oracle passes: exactly 825 former InvalidTransformedEvidence
  results become Transformed, with all 5,616 other records unchanged (including
  terminal). There are 5,126 returned roots and 1,794 exact witnesses. The
  unchanged independent oracle checks 48,059 assertions, including 6,441
  self-paired record checks; a separate full baseline/candidate delta check
  establishes the actual improvements. The original checkpoint-52 gate remains
  failed and unchanged. Candidate stdout is 4,428,994 bytes.
- Baseline and candidate focused Memchecks pass with zero errors and no
  definite/indirect/possible lost blocks. Each matches its own native stdout.
  Reachable bytes/blocks: baseline 1,839,816 / 15,311; candidate 1,840,544 /
  15,318 (+728 bytes / seven blocks). Cumulative requests include collection:
  baseline 2,837,158 allocations / 213,126,963 bytes; candidate 2,908,903 /
  215,408,740 bytes. Not equal-work throughput, peak memory, RSS or a leak-free
  lifetime proof. The candidate returns additional certified answers.
- Clippy initially fails on one redundant u64-to-u64 cast in a new test.
  Original source is frozen as point-image-v1-algebraic_binary.rs and the
  failed code-101 capture is preserved. Removing that cast changes only
  cfg(test) code; source-prefix and exact replacement checks pass. V2 again
  passes 809 all-feature tests per profile, Clippy and changed-file rustfmt.
  Its full all-feature Hypercurve debug suite remains running; no pass assumed.
- A source-level proof obligation remains when approximate ordering chooses
  multiply/divide extrema: strict equality of chosen endpoints alone does not
  certify the whole image. A solver-only 175-file / 6,088,204-byte copy keeps the
  v2 consumer source unchanged while testing this boundary. First probe fails
  its own endpoint-equality precondition (Unknown, not Equal), so it does NOT
  demonstrate a candidate defect. The original source/capture are preserved.
  Two corrected controls withhold witnesses; all eight point-image tests pass.
- A guarded variant now replays multiply/divide image construction under STRICT
  before adding a point witness when approximate ordering was permitted. That
  replay is one-level bounded, and addition/subtraction need no extrema ordering.
  No approximate-extrema witness defect has been reproduced; this strengthens
  the proof boundary. All 24 focused tests pass. Guarded debug/release/lint and
  public-driver builds are running separately; their final results are pending.
- Main candidate executables occupy 28,465,872 dedicated bytes in
  /tmp/calcium-point-image.4f6mja. Existing baseline and power-sum executables are
  reused and unchanged. About 18 GB remains free on /tmp after shared-cache
  growth. No cleanup/deletion, production/donor edit, commit, push or external
  report. Prepared timing/allocation runners have NOT run. No new donor credit;
  checkpoint 53 is not yet evidence-bound or a retained production transfer.
- Final guarded variant passes 811 all-feature solver tests per debug/release
  profile, Clippy/all-targets and changed-file rustfmt; 24 focused tests pass.
  Guarded public and Memcheck output match the initial candidate byte-for-byte,
  4,428,994 bytes each. The independent checker again proves exactly 825 gains
  and 5,616 unchanged records. Guarded Memcheck has the same errors/lost/reachable/
  allocation counts as v1 but 215,408,746 cumulative bytes (+six, consistent with
  the longer executable path). Whole-image proof guard remains isolated.
- V2 consumer terminates 0 / null signal at 19:52:18.576Z: 1,764 passed,
  nine pre-existing ignored, 45 suites. About 26 minutes including build; this
  is regression execution, not a benchmark or final guarded-source qualification.
  All regression/memory work ended before CPU measurements began.
- Guarded CPU campaign terminates 0 at 19:54:57.714Z; allocation campaign 0 at
  19:55:01.586Z. Forty queries, two lifecycles, core 6, twelve alternating ABBA/
  BAAB blocks: 3,840 observations / 160 calibration rows. Separate allocation
  binary: 480 observations. Every full report/checksum is checked; statistics
  and actual observation order replay successfully at 19:59:56.044Z.
- Seventy same-result groups: paired median ratios 0.930234..1.080410; five
  per-group bootstrap intervals wholly above one, seventeen below. These are
  not multiplicity-adjusted or universal. All seventy groups have identical
  request/byte/live/peak counts. Ten groups gain answers, with additional-work
  ratios 0.986576..1.167275, not equal-work speedups. Six save one request/57
  bytes per query; four request 119 or 223 more bytes, one adds one request.
  All eighty groups have unchanged measured live/peak deltas. Rational endpoints
  and preconditioned process-local caches only; nonrational/Unknown costs pending.
- Recorder draft initially required identical whole-collector memory totals;
  it fails because guarded cumulative bytes differ by six. Original evidence/
  recorder/findings sources and code-1 capture are preserved. Corrected binding
  records both exact totals; no mathematical oracle or measured output changes.
- point-image-manifest.json binds 154 artifacts, 33 terminal gates, initial/v2/
  guarded source identities, six dedicated executables and independent public/
  cost results. Captured verifier passes at 20:16:38.251Z, code 0 / null signal,
  preserving all prior failures and rechecking all 956 unchanged live files.
  It imports 48–52 checks, not the entire historical 47 chain; that latest full
  capture remains 04:56:34.285Z. All command/function/session handles are terminal.
- Root report and continuation README updated through 53. Guarded source has
  20 net algorithm lines and 303 net test lines versus retained baseline.
  Six dedicated executables total 56,943,680 bytes; shared-cache growth is
  separate. About 17 GB remains free on /tmp. No deletion, production/donor edit,
  commit, push or external report. Five retained transfers and donor coverage
  remain unchanged. This is PROGRESS, not completion of the ecosystem audit.
- Next actual action: guarded-consumer, nonrational/Unknown endpoint/state-history
  cost and representative native/WASM qualification, then a justified retention
  decision. Only afterward resume separate power-sum performance qualification.
- Post-documentation verification passes again, including all 956 unchanged
  live identities and preserved mathematical/test/bookkeeping failures. All 26
  local report links and three ledger links resolve; scoped whitespace checks
  pass. The historical ledger remains a symlink. All handles are terminal.

## Checkpoint 54 — guarded witness consumer/platform qualification; extended costs pending

- Previous goal turn is PROGRESS: 52–53 evidence is bound, the shared point-image
  completeness gap is identified, an isolated guarded candidate recovers 825
  answers, and bounded native costs are measured. Revalidated 53 and all 956
  unchanged retained live identities before continuing. All old handles are
  terminal; neither whole-inventory completion nor production retention is claimed.
- Prepare one full isolated qualification tree from the retained baseline with
  only the byte-identical guarded solver file substituted. This permits exact
  guarded consumer/application qualification without modifying any old snapshot.
  Reuse the shared Cargo cache and previously qualified baseline application
  executables wherever source/build identities match. About 17 GB is free on /tmp.
- Next gates are the exact guarded consumer, representative native/WASM builds
  and execution, plus nonrational/Unknown endpoint and state-history cost checks.
  All original references and the separate power-sum optimization remain in scope.
- Full guarded qualification tree prepared: 956 files, 45,463,105 copied bytes;
  only hypersolve/src/algebraic_binary.rs differs from the retained baseline.
  Consumer release/all-features gate terminates 0 / null signal at
  20:40:47.732Z: 1,764 passed, nine pre-existing ignored, 46 suites including
  the zero-test documentation suite. Its 507.123 s includes build/test work,
  not a benchmark. Previous v2 debug consumer gate is not relabeled guarded.
- Native example and native/WASM full-public-corpus builds now use the shared
  Cargo cache. Matched retained-baseline example binaries are reused by exact
  source/build/hash identity and rerun. No source/donor retention or deletion.
- Final guarded consumer Clippy (all targets/all features, warnings denied)
  passes at 20:45:43.300Z; WASM library build with default plus triangulation,
  SVG and Hershey features passes at 20:45:54.655Z. Build-only consumer WASM is
  distinguished from the actually executed arithmetic modules.
- Native and WASM full-value collectors each run both baseline and candidate:
  6,441 records, byte-identical to the checkpoint-52/53 native outputs. A separate
  mechanically derived collector changes exactly two public call sites to
  APPROXIMATE_512 and repeats both platforms. This exercises the guarded policy
  replay with exact-rational inputs; no nonrational/Unknown or instrumented branch
  coverage claim. STRICT check ends 20:44:15.752Z; approximate run ends
  20:49:44.963Z. All captures terminate 0 / null signal.
- Each policy/platform retains exactly 825 gains and 5,616 unchanged records,
  with all 5,126 returned roots / 1,794 exact witnesses certified by the unchanged
  oracle. Its 48,059 checks include 6,441 candidate self-pairs; separate full-record
  baseline comparisons prove the delta. Reexecution is not an independent new
  oracle or expanded mathematical corpus. Forty zero-resultant Unknowns persist.
- New example assertions pass and stdout matches reused baselines. Stripped
  curve examples grow 1,136 / 1,152 bytes; unchanged scalar code shrinks 784 bytes,
  demonstrating build-path/linker-layout effects. No algorithm-only byte claim.
  Active collectors grow 1,152 native / 560 WASM bytes under STRICT and
  1,080 native / 547 WASM bytes under approximate policy. No new timings or
  allocator/Memcheck campaign; command elapsed times are not benchmarks.
- point-qualified-manifest.json binds 179 artifacts / 45 successful captures.
  Captured verifier passes 20:56:13.407Z, code 0 / null signal, seven output
  records / 11,617 bytes and empty stderr. It rechecks all 956 live hashes and
  prior failures through 48–53, not the full historical 47 chain; its latest full
  capture remains 04:56:34.285Z. All launched commands are terminal.
- Fourteen dedicated files total 68,928,629 bytes in three bounded /tmp
  directories; six old baseline app files and the shared Cargo cache are reused.
  Source/log evidence stays in the workspace. /tmp had 16,642,371,584 available
  bytes at the captured capacity check. No deletion, cleanup, production/donor
  edit, commit, push or external submission. Five retained transfers and donor
  coverage (1,415 complete / 20 partial / 183,653 unique lines) are unchanged.
- This is PROGRESS, not whole-audit completion or repair retention. The originally
  planned nonrational/Unknown endpoint, broader state-history and WASM cost gates
  carry forward explicitly; this checkpoint does not claim they ran. Next actual
  action is that extended qualification, then a justified retention decision,
  followed by the separate power-sum performance experiment and full inventory.
- Post-documentation verification passes again, including all 956 unchanged live
  identities and preserved failures. All 28 local report links and three ledger
  links resolve; whitespace/scoped diff checks pass. The historical ledger remains
  a symlink. No command is still running; /tmp remains about 16 GB free.

## Checkpoint 55 — nonrational/Unknown exact values and histories; matched costs pending

- Previous turn is PROGRESS: final guarded consumer/platform and size gates are
  bound through 54. Revalidated its evidence and all 956 unchanged live sources
  before continuing. All previous commands are terminal; /tmp remains about 16 GB
  free. The full inventory and separate power-sum transfer remain in scope.
- Reuse the retained and guarded full snapshots directly, with no additional
  whole-source copy. Add a full-value serialized corpus over known radical,
  algebraically obscured and trigonometric-Unknown endpoints, with explicit
  constructed/refined/deserialized histories. Separate exact mathematical
  certificates from status-only and cost observations; no checksum-only proof.
- Collected 384 queries per variant: 48 operations, STRICT/APPROXIMATE_512 and
  constructed/−64-refined/−4096-refined/round-trip histories. Complete serialized
  inputs/outputs are retained and input records stay unchanged around every call.
  Shared process constants are not cold between rows. Two release executables
  total 6,379,848 bytes; no additional whole-source copy or cache target.
- Independent BigInt field interpreter evaluates the actual computational graph
  in Q(sqrt(2),sqrt(3)), admitting nonzero pi/e monomials and the explicit shared
  sin(e−pi)^2+cos(e−pi)^2=1 identity. Initial decoder fails at baseline case 40 /
  STRICT / constructed history, after 4,236 values, because InvPi/Sqrt2 cases were
  omitted. Original decoder and code-1 capture are preserved. Source-defined
  support added; completed decoding covers 10,664 values without changing output.
- Initial mathematical oracle passes 17,640 checks. Preserved its exact source
  and capture, then strengthened it with exact authored input bounds/witnesses
  and rational carrier-shape checks. Final gate passes 21,216 checks at
  21:23:32.789Z: 9,968 baseline / 11,248 candidate. Full carrier coefficients are
  checked after monic normalization (nonzero rational scale allowed), including
  multiplicity, exact signs, selected roots, witnesses and interval containment.
  Negative in-memory controls reject modified witness/polynomial/endpoint values.
- Guarded corpus: 316 Transformed / 252 exact witnesses versus 188 / 124 before.
  Exactly 128 query records improve; 256 others plus the terminal record are
  byte-identical. Repeated policies/histories and overlapping controls do not
  make these 128 independent bugs or a disjoint addition to the earlier 825.
  Both variants retain 24 denominator guards, six Undecided, six nonisolating
  reports and 32 input-evidence failures.
- Deep refinement enables cases 36/37 under both policies in both variants;
  constructed/coarse/round-trip states remain Undecided or nonisolating. These
  four case/policy combinations change available evidence, not numerical values.
  They must remain separate in the matched-cost design.
- Focused Memcheck outputs match native byte-for-byte; both runs have zero errors
  and zero definite/indirect/possible losses. Reachable baseline 47,632 bytes /
  346 blocks, candidate 48,648 / 353; +1,016 bytes is not a leak diagnosis or
  per-query cost. Cumulative requests are 123,272,240 versus 131,069,337 bytes,
  including all constructors, serialization, histories and extra certified output.
  Whole memory campaign ends 21:19:27.205Z. No suppression or zero-live claim.
- point-extended-manifest.json binds 73 artifacts and 15 terminal captures,
  including the failed decoder probe. Captured verification passes at
  21:30:31.317Z, code 0 / null signal: eight records / 12,917 bytes, empty stderr.
  It checks all 956 unchanged live sources and 48–54 evidence, not a full
  historical 47-chain rerun. That latest full capture remains 04:56:34.285Z.
- Both collector builds preserve one unnecessary-mut warning. No new Clippy,
  full-suite, CPU/allocator or WASM gate is claimed. Existing draft timing modes
  are unexecuted and not accepted cost evidence: finalize final-full-report
  validation outside the clock and explicit cache-history accounting first.
- No production/donor edit, retained transfer, deletion, cleanup, commit, push
  or external submission. All earlier artifacts, five retained transfers and
  donor coverage (1,415 complete / 20 partial / 183,653 lines) are unchanged.
  Four native/Memcheck output files total 4,791,668 workspace bytes; /tmp stays
  about 16 GB free. All commands are terminal. This is PROGRESS, not completion.
- Next actual action: finalize/qualify the matched full-result CPU/allocation
  harness, measure the nonrational/Unknown and state-history matrix, execute
  relevant WASM checks/costs and make an explicit retention decision. Then resume
  separate power-sum performance work and the remaining full reference inventory.
- Post-documentation verification passes again, including 73 bound artifacts,
  all 956 unchanged live sources and prior failed gates. All 30 local report
  links and three ledger links resolve; whitespace/scoped diff checks pass.
  The historical ledger remains a symlink. All launched commands are terminal.

## Checkpoint 56 — matched native costs expose eager-probe regressions; not retained

- Previous turn is PROGRESS: 55 binds independent exact-value/history checks and
  focused memory observations. Revalidate 55 and the unchanged 956-file live map
  before further work. Reuse both existing snapshots/cache; /tmp has about 16 GB
  free. No scope is dropped and no production retention is assumed.
- Qualify a new benchmark wrapper, preserving the old unqualified timing draft.
  Use all 48 cases × two policies × four histories × retained/fresh lifecycles.
  Validate the final actual full report outside the timed window, keep allocation
  separate from CPU, and explicitly account for a final returned result retained
  at the allocation snapshot. WASM costs and full inventory remain open.
- Both baseline/candidate release builds and all-target Clippy with warnings
  denied pass. The build capture ends 21:41:00.627Z, code 0; four frozen native
  executables total 12,772,864 bytes in /tmp/calcium-point-history.L38XCJ. No new
  whole-source copy or shared-cache target. Revalidation of 55 passes again,
  including all 956 unchanged live files and preserved earlier failures.
- New qualification ends 21:55:25.081Z, code 0: all 3,072 final observations
  (768 groups × two variants × CPU/allocation) pass 84,864 independent exact
  checks. CPU qualification uses one measured iteration, allocation eight;
  intermediate results have only a polynomial-length checksum, not full proofs.
  Nine preconditioning calls do not change any prior complete input/report.
  There are 256 gained groups and 512 unchanged groups; these are lifecycle,
  policy and history repetitions of the existing gap, not new independent bugs.
- Native campaign started 21:57:01.122Z. All groups are retained in the matrix;
  twelve alternating ABBA/BAAB CPU blocks use a common calibrated iteration
  count, followed by three separate paired allocation blocks. No CPU results
  are accepted before the campaign and offline full-record/statistical checks
  finish. No own build/regression/Memcheck runs overlap the CPU campaign.
- Campaign finishes 22:03:33.420Z, code 0: 36,864 CPU observations / 1,536 pilots
  and 4,608 separate allocation observations. Offline verification passes
  22:11:50.280Z, checking every full final result against qualified records,
  actual pair ordering, calibrated iteration counts and recomputed statistics.
  No final-result mismatch, discarded timing outlier or failed child occurred.
- The 512 same-result groups have paired median ratios 0.902928–1.348403;
  individual bootstrap intervals are below one in 14 and above in 192, without
  multiplicity adjustment. Largest paired slowdown: case 37 / approximate /
  round-trip / retained, 18.242 vs 24.587 microseconds, same nonisolating report.
  Allocation requests/bytes rise in 188 groups and remain equal in 324. Across
  eight-query batches, request delta is at most 456 and requested-byte delta
  at most 46,848 (different maxima may belong to different groups). Return-live
  is unchanged in all 512; peak rises 176 bytes in 16 retained case-20/21 groups.
- The 256 newly successful groups do additional certified work: paired ratios
  0.914530–3.506376 are not equal-work speedups/regressions. Requests/bytes fall
  in 80 groups and rise in 176. With the final result alive, net return-live
  rises 143–1,663 bytes; peak is unchanged in 165 and higher in 91, at most
  1,104 bytes. These are requested-byte counters, not RSS, leaks or universal
  memory bounds. Checkpoint 55's whole-collector reachability remains separate.
- Do not retain the eager variant at this checkpoint. Source inspection of
  algebraic_binary.rs (1–290, 300–415) and root_isolation.rs (495–795) shows a
  plausible demand gate: the existing refiner already compares endpoints and
  rejects a collapsed interval without a witness. Investigate deferring the
  recovery probe until needed, without weakening strict whole-image, vanishing,
  containment or uniqueness obligations. This is not yet an implementation or
  measured optimization. Preserve all eager-version controls and measurements.
- point-history-manifest.json binds 91 artifacts and 15 successful captures.
  Captured verification passes 22:21:23.086Z, code 0 / null signal, with empty
  stderr. All 956 live hashes and prior 48–55 evidence/failures are rechecked;
  this is not a full historical 47-chain rerun (latest remains 04:56:34.285Z).
- Raw qualification/CPU/pilot/allocation evidence totals 158,331,284 workspace
  bytes. Four dedicated executables add 12,772,864 /tmp bytes; existing snapshots
  and shared build cache are reused. Capacity capture: 16,467,509,248 free /tmp
  bytes. No cleanup/deletion, production/donor edit, retained transfer, commit,
  push or external report. All launched commands are terminal; five retained
  transfers and donor coverage (1,415 complete / 20 partial / 183,653 lines)
  remain unchanged. New wrapper rustfmt checks and scoped diff checks pass.
- This is PROGRESS, not whole-audit completion. Next actual experiment is an
  isolated demand-gated repair with independent qualification and matched costs;
  relevant extended WASM execution/costs and explicit retention decision still
  follow. Separate power-sum optimization, remaining reference/supporting reads
  and full inventory reconciliation remain in scope. The root report is updated;
  its 33 local links and the ledger's three links resolve. The historical ledger
  remains a symlink.

## Checkpoint 57 — demand-gated recovery removes nonpoint allocations; retry costs remain

- Previous turn is PROGRESS: 56 completed matched native costs and exposed
  unchanged-result regressions, leaving the eager variant unselected. Revalidate
  its full evidence and all 956 live identities before changing the experiment.
  All prior processes are terminal; /tmp remains about 16 GB free.
- Prototype recovery after the existing refiner rejects an interval, using its
  typed status rather than a diagnostic-string match. Require strict point-image
  proof and preserve the approximate multiplication/division whole-image replay;
  then rerun the unchanged refiner's witness/vanishing/uniqueness checks. This
  candidate is isolated, not a retained change. Copy only the solver slice and
  reuse unchanged dependency snapshots and the shared build cache.
- Keep all earlier sources, failures and benchmark rows. Qualify full public
  results and histories before comparing costs; existing eager/baseline timings
  are controls, not substitutes for paired measurements of the new version.
  Extended WASM, power-sum work and the full original inventory remain open.
- The isolated solver copy contains 175 files / 6,091,527 original bytes. Only
  algebraic_binary.rs changes algorithm/tests; Cargo.toml paths mechanically
  reuse the retained dependencies. The public refiner is unchanged. Strengthened
  private-helper controls still exercise strict equality, approximate extrema,
  half-open ownership and rejection of a witness that fails the polynomial.
- Build ends 22:30:55.684Z, code 0: five executables total 14,950,736 bytes in
  /tmp/calcium-point-demand.BLEmYy. Library-only debug/release gates pass 447 tests
  each; separate full all-feature solver gates then pass 811 tests per profile,
  eight suites, zero failed/ignored. App and all-target/all-feature solver Clippy
  with warnings denied pass. No final Hypercurve consumer qualification yet.
- Public qualification ends 22:34:23.356Z, code 0. Both 6,441-record policy
  outputs equal the eager variant exactly and independently certify its same
  825 gains / 5,616 unchanged records. The large checker has 48,059 checks per
  policy including self-pairs; repeated policies are not independent corpora.
  Extended output also matches exactly, passing 11,248 independent checks over
  384 demand-candidate queries. No mathematical result is weakened to pass.
- Focused extended and approximate-public Memchecks have zero errors and zero
  definite/indirect/possible losses; output equals native. Extended reachability
  is 48,648 bytes / 353 blocks; large collector 1,840,544 / 15,318. They are not
  zero-live gates. Whole-collector cumulative bytes are 130,905,028 and
  232,232,789 respectively, not marginal costs or RSS.
- The 768-group history/lifecycle qualification passes 1,536 observations and
  44,992 independent exact checks at 22:35:15.923Z. Every full final result equals
  the eager guarded reference in both instrumentation modes. CPU campaign began
  22:35:52.121Z using all three frozen variants together, twelve balanced
  six-observation blocks per group and subsequent separate allocation runs.
  CPU results remain provisional until terminal captures and offline verification.
- Before retention, explicitly compare fresh-process first queries without
  preconditioning as well: ordered public collectors and nine-preconditioned
  benchmark children do not prove every cold cache-enabled decision is unchanged.
  This semantic gate supplements, rather than replaces, pending extended WASM,
  final consumer/size and full-inventory work. No new cold-first-query harness yet.
- Three-variant campaign ends 22:45:35.556Z, code 0. CPU: 55,296 observations /
  2,304 pilots; allocation: 6,912 observations. Offline verification passes
  22:46:23.662Z, checking complete final results, all pairing/calibration/time
  records and recomputed statistics. No discarded outlier or result mismatch.
- Demand versus retained baseline: all 512 same-result groups now have identical
  request/byte/return-live/peak counts. Paired timing ratios still span
  0.881863–1.168626; eight individual CIs are below one and 134 above, without
  multiplicity adjustment. Slower controls remain; this is not a universal
  speedup or timing-equivalence result.
- Demand versus eager: all 768 results match; ratios 0.754745–1.122420, with 119
  individual CIs below one and 184 above. Retained case 37 / approximate /
  constructed improves from 23.931 to 18.937 microseconds (baseline 19.091),
  paired ratio 0.754745. Requests/bytes decrease in 188, equal in 324, increase in
  the 256 recovered-point groups. Eight-query deltas range −456 to +40 requests,
  −46,848 to +2,760 bytes. Return-live equals eager in all 768; peak is lower in
  16, equal in 722, higher in 30, ranging −176 to +201 bytes. Retrying constructs
  extra rejected-report state; do not hide that cost.
- Versus baseline, 256 newly successful groups perform different work; their
  timing ratios are not equal-work comparisons. Request/byte totals are equal
  in 80 and higher in 176; net return-live rises 143–1,663 bytes with the final
  result alive. Peak is equal in 150 and higher in 106, at most 1,104 bytes.
- The revised file is 1,515 lines / 53,846 bytes: +32 net algorithm and +71 net
  test lines versus eager. Five binaries total 14,950,736 dedicated /tmp bytes;
  raw history/CPU/pilot/allocation rows total 230,193,025 workspace bytes.
  Capacity capture: 16,393,678,848 /tmp bytes available. Existing snapshots,
  baseline/eager binaries and shared cache reused; no cleanup or deletion.
- Recorder binds 129 artifacts / 24 successful captures and all 175 candidate
  source files. No new production/donor edit, sixth retained transfer, commit,
  push or external report. The five retained transfers and donor coverage
  (1,415 complete / 20 partial / 183,653 uniquely read lines) are unchanged.
  Candidate/wrapper rustfmt and scoped diff checks pass.
- This remains PROGRESS, not a whole-audit completion or retention claim. Next
  verify cold first-query decisions before choosing whether to refine the retry
  implementation; any selected final version still requires applicable extended
  WASM, consumer and representative-size gates. Separate power-sum optimization,
  all remaining references/supporting reads and inventory reconciliation stay open.
- Captured integrity verification passes 22:52:56.395Z, code 0 / null signal,
  ten stdout records / 34,729 bytes and empty stderr. It binds 129 artifacts /
  24 successful captures, checks all 175 candidate files and 956 unchanged live
  identities, and preserves 48–56 evidence/failures. The full historical 47-chain
  was not rerun; its latest complete capture remains 04:56:34.285Z. All launched
  commands are terminal. Root report and continuation README are updated.

## Checkpoint 58 — cold first-query and repeated-state semantics

- Previous checkpoint remains PROGRESS, not retention. The 175-file isolated
  demand candidate and all 956 live identities are unchanged. No algorithm or
  donor-source edit is made. Existing baseline/eager/demand sources and shared
  Cargo cache are reused without another source copy.
- Shared cold-sequence harness covers 48 cases × two policies × four initial
  histories × retained/fresh/round-trip lifecycles. Each group executes nine
  fully recorded calls in one fresh native process or import-free WASM instance,
  with no unrecorded query preconditioning. Constructors/initial histories may
  initialize caches. Fresh replacement drops old inputs before rebuilding;
  round-trip serializes all scalar input fields between calls 0 and 1 onward.
- Initial build passes but Clippy rejects an unnecessary unwrap (Cargo 101).
  First resume stops on an incorrect expected exit-code assertion (1 vs 101).
  Failure captures and exact control-flow patch are preserved. Corrected builds
  complete 23:10:02.953Z; all native/WASM build and lint gates are warning-free.
- Collection completes 23:11:18.240Z: 3,456 native processes, 3,456 WASM instances,
  62,208 full query records and 6,912 terminal records. WASM uses Node 22.22.2 /
  V8 12.4.254.21-node.39, one compiled module per variant, no imports, explicit
  host GC every eight completed instances, with no old instances retained.
- Independent exact-value check passes 23:13:17.230Z, code 0 / null signal:
  1,751,136 checks, zero mathematical failures, and three rejected witness/
  polynomial/endpoint negative controls. Native/WASM raw records are byte-equal.
- Per platform, demand/eager agree on all 10,368 records. Both gain 3,456 answers
  with exact witnesses versus baseline and preserve the other 6,912 records.
  The 1,152 first calls gain 384 and preserve 768. Those 384 are the same 128
  case/policy/history combinations across three lifecycles, not independent bugs.
  Baseline transforms 5,044 queries / 3,316 witnesses; either repair transforms
  8,500 / 6,772. Denominator, Undecided, nonisolating and invalid-input controls
  remain unchanged. No lost answer/witness or other candidate difference.
- Four deeply refined round-trip groups change in every variant/platform:
  cases 36/37 (Add/Subtract), both policies, history 2. Call 0 is Transformed
  with a witness; calls 1–8 after serialization are Undecided (STRICT) or
  NonIsolatingImageInterval (approximate). Inputs' full serialized records and
  independently interpreted values remain unchanged. This is existing bounded
  decision history dependence, not a candidate regression. No other group
  changes any full record within its nine calls.
- Six frozen binaries in /tmp/calcium-point-cold.3RcRIa use 14,358,510 bytes;
  raw output and group metadata use 201,468,576 workspace bytes. Capacity capture
  records 16,247,853,056 available /tmp bytes. WASM post-collection memory is
  1,310,720–1,638,400 bytes, not per-query peak/RSS/allocator/CPU evidence.
- Cold-first-query and extended native/WASM semantic gates close for this
  unchanged candidate. No new CPU/allocation/Memcheck/full consumer/size result,
  source copy, production transfer, cleanup/deletion, commit, push or external
  report. Five retained transfers and donor coverage (1,415 complete / 20 partial
  / 183,653 uniquely read lines) are unchanged. Bound findings:
  [checkpoint 58](exactcore-hyper-comparison/audits/continuation/calcium/point-cold-findings.md).
- Next run matched extended WASM costs and final consumer/representative-size
  qualification before the retention decision, considering checkpoint 57's
  disclosed retry and slower-control costs. No further algorithm revision is
  selected. Separate power-sum work and the entire original reference inventory
  remain open; this is not whole-audit completion.
- Captured integrity verification passes 23:22:40.971Z, code 0 / null signal:
  132 bound artifacts, 30 terminal captures (27 successful / three preserved
  failures), 11 stdout records / 38,662 bytes, empty stderr. It rechecks all
  175 candidate files and 956 unchanged live identities plus 48–57 evidence,
  not the full historical 47-chain. All launched processes are terminal.
  Root report/local-link checks, harness rustfmt and scoped diff checks pass.

## Checkpoint 59 — matched extended WASM costs

- Previous turn is PROGRESS. Revalidation checks all 132 checkpoint 58 artifacts,
  175 candidate files and 956 live identities. No algorithm, live production or
  donor-source edit. Baseline/eager/demand snapshots and shared Cargo cache reused.
- `point-wasm-work.rs` is the exact extracted native HistoryWork prefix; compare
  its entire contents mechanically. Wrapper separates prepare/batch/finish;
  every observation gets a fresh import-free WASM instance. Module compilation,
  instance creation, nine preconditioning queries, final report drop, JSON,
  validation and host GC are outside timing. Host call boundary/state bookkeeping
  are timed, with the final report alive. Fresh-mode reference roots remain alive
  exactly as in native 56/57, unlike 58's fresh-only cold-state lifetime protocol.
- Release WASM builds and Clippy pass warning-free for all three variants,
  completed 2026-09-10T23:31:33.615Z. Rustfmt passes. Dependency lockfiles agree
  exactly after normalizing only the harness package name. No full consumer or
  solver-suite rerun at this checkpoint.
- Qualification: all 768 groups × three variants × one/eight batch sizes =
  4,608 observations, 129,856 independent exact checks, full native-reference
  agreement. Capture completes 23:38:14.937Z, offline recheck 23:48:08.455Z.
- Runtime Node 22.22.2 / V8 12.4.254.21-node.39, core 6 affinity, no Liftoff,
  no WASM tier-up and no lazy WASM compilation. Optimizing WASM tier selected
  before collection; host JS/runtime scheduling is not similarly fixed. Explicit
  host GC every eight completed instances; no earlier instances retained.
- Campaign 23:48:05.369Z–23:57:48.384Z, timing runner
  23:48:08.761Z–23:57:47.759Z: 55,296 observations / 2,304 pilots, all full final
  records match. Three 16-iteration pilots calibrate a common 4–4096 iteration
  count targeting 4 ms. Twelve six-sample palindromic blocks cover all variant
  permutations/reverses twice. Every sample is preserved; final full-result checks
  do not imply each intermediate batch query had more than a checksum.
- Offline check passes 23:59:16.257Z, code 0 / null signal / empty stderr. It
  recomputes calibration, order, time ranges, full records and statistics.
  Individual 5,000-resample bootstrap CIs are not multiplicity-adjusted.
- Demand/baseline: 512 equal-result groups, ratios 0.801026–1.335991, individual
  CIs 165 below / 34 above one. Another 256 groups return additional answers:
  ratios 0.912081–3.960350, CIs eight below / 163 above; different work, not equal-
  work performance. Demand/eager: all 768 results equal, ratios 0.752653–1.234868,
  CIs 159 below / 33 above one.
- Retained case 37 / approximate / constructed improves eager 32.331 to demand
  24.882 microseconds (baseline 25.104), paired ratio 0.767287,
  CI [0.736961,0.782960]. Retained case 3 / approximate / round-trip retry costs
  eager 15.388 versus demand 15.883 microseconds, paired 1.081198,
  CI [1.027356,1.195473]. These support workload-specific tradeoffs only.
- Timing attribution remains uncertain for volatile controls: case 17 / STRICT /
  deep / fresh paired baseline ratio 1.217574, CI [1.107369,1.333005], contrasts
  with marginal medians 121.079 baseline / 119.464 demand microseconds. Case 44 /
  approximate / round-trip / retained is an early InvalidEvidence control with
  ratio 1.335991, CI [0.994250,1.671217], marginal medians 10.113/10.137 us.
  Do not silently choose one estimator, discard samples or infer code causality.
  Host powersave governor, SMT sibling 14; load snapshots 0.90/0.79/0.97 to
  3.12/3.04/2.15 do not explain individual interruptions or fix frequency.
- Three frozen modules in /tmp/calcium-point-wasm.jPk2WY: 4,843,385 bytes;
  raw qualification/CPU/pilot files: 222,399,157 workspace bytes. Capacity:
  16,237,924,352 available /tmp bytes. No new source-tree copy, cleanup/deletion,
  memory/allocator/Memcheck result, representative application-size result,
  production retention, commit, push or external report.
- Recorder binds 104 artifacts and 19 successful captures, unchanged 175-file
  candidate and all 956 live identities. Five retained transfers and donor
  coverage (1,415 complete / 20 partial / 183,653 unique read lines) are unchanged.
  Findings: [checkpoint 59](exactcore-hyper-comparison/audits/continuation/calcium/point-wasm-findings.md).
- Next: bounded diagnostic replay of volatile paired/marginal controls, including
  unfavorable and favorable cases, with wall/CPU time and order/GC context.
  Preserve this campaign unchanged; no algorithm revision yet. Then final
  selected-version consumer/representative-size qualification and retention.
  Separate power-sum work and the entire original reference inventory stay open.
- Captured integrity verification passes 2026-09-11T00:04:12.443Z, code 0 / null
  signal, 12 stdout records / 54,712 bytes, empty stderr. It binds 104 artifacts /
  19 successful captures, rechecks all 175 candidate and 956 live identities, and
  preserves prior 48–58 evidence/failures. Full historical 47-chain is not rerun.
  All launched commands are terminal. Root report/local links, harness rustfmt
  and scoped diff checks pass. This remains PROGRESS, not whole-audit completion.

## Checkpoint 60 — statistical qualification correction

- Previous turn is PROGRESS. Revalidation checks checkpoint 59's 104 artifacts,
  all 175 candidate files and 956 live identities. No new temporary files,
  binaries, benchmark runs, source copies or production/donor edits.
- While preparing wall/process/thread-CPU replay, find a bug in this audit's
  bootstrap helper. LCG multiplier = 1 mod 4, increment = 3 mod 4; reducing to
  12 indices forces residue cycling, three from each class in every sample.
  This is not ordinary IID sampling with replacement. Historical bootstrap
  confidence/significance claims using it are withdrawn pending correction.
  Raw observations, point estimates, mathematical tests and allocations persist.
- Independent exhaustive test: all 256 four-draw samples of [0,0,0,1] give median
  counts [189,54,13] for [0,.5,1], central percentile interval [0,1]. The old
  modulo-four sampler always returns median zero and interval [0,0]. New helper
  passes the exhaustive-distribution control, rejection boundaries, repeatability
  across refill and non-forced residue counts. This is an audit-tooling defect,
  not an exact-real/donor value error.
- New deterministic AES-256-CTR pseudorandom stream with SHA-256-domain/group
  keys and rejection to an unbiased bounded range; 20,000 replicates versus
  historical 5,000. Both stream and replicate count change, so individual
  endpoint differences are not an isolated RNG-only effect. Independent exact
  order-statistic interval uses ranks 3 and 10 of twelve, coverage 3938/4096
  under IID/common-distribution assumptions. Neither method proves those
  assumptions for timings or adjusts for multiple comparisons.
- Reanalysis from 151,296 raw measured rows preserves complete per-variant
  reports, block order/counts, iteration counts, all paired ratios/medians and
  marginal medians. Four campaigns: 53 (80 comparisons), 56 (768), 57 (1,536),
  59 (1,536), totaling 3,920. 3,648 endpoint pairs change and 269 formerly
  directional claims include one; no opposite transition in this reanalysis.
- Equal-result below/above-one counts, old → corrected bootstrap → order CI:
  53 baseline 17/5 → 11/2 → 9/1; 56 baseline 14/192 → 9/151 → 5/114;
  57 baseline 8/134 → 4/96 → 2/73, eager 119/184 → 95/141 → 79/111;
  59 baseline 165/34 → 140/19 → 113/6, eager 159/33 → 134/24 → 118/23.
  Newly successful baseline strata remain separate, doing additional work.
- WASM case 17 / STRICT / deep / fresh keeps paired 1.217574 and opposite-
  direction marginal medians 121.079/119.464 us; corrected CI
  [0.973856,1.360412] and order CI [0.965811,1.402592] now include one. Old
  [1.107369,1.333005] was unsupported. Unknown-control 37/approx/constructed/
  retained benefit remains: 0.767287, corrected [0.733356,0.782960], order
  [0.729751,0.790640]. Retry 3/approx/round-trip/retained remains: 1.081198,
  corrected [1.025073,1.211209], order [1.017033,1.226945]. No uniform claim.
- Inventory binds 45 matching top-level historical scripts and line/hash
  matches, including retained exponential, monic, derivative and e-planner
  work. It is potential impact, not completed file-by-file statistical closure,
  nor a scan of the full ecosystem. Remaining intervals/inference and retention
  implications need source/raw-data review. Do not silently revert the five
  retained changes or treat this subset as their complete requalification.
- Reanalysis capture passes 2026-09-11T00:16:48.384Z; sampler self-test capture
  00:19:47.747Z; independent full recomputation 00:21:38.479Z, all code 0 and
  empty stderr. Corrected analysis is 7,317,447 workspace bytes, no new /tmp
  files; capacity capture 16,237,924,352 available /tmp bytes.
- Root report prominently qualifies all affected historical confidence claims.
  Findings: [checkpoint 60](exactcore-hyper-comparison/audits/continuation/calcium/point-statistics-v60-findings.md).
  Five retained code changes and donor coverage (1,415 complete / 20 partial /
  183,653 unique read lines) are unchanged; confidence-based justifications are
  qualified pending impact review. No commit, push, cleanup or external report.
- Captured final verification passes 2026-09-11T00:46:19.463Z, code 0 / null
  signal: 83 bound artifacts, four successful captures, 175 candidate files and
  all 956 unchanged live identities, including complete corrected reanalysis.
  Thirteen output records / 60,019 bytes, empty stderr. Earlier 48–59 evidence
  remains preserved, not a full historical 47-chain rerun. All tool sessions
  are terminal; no new /tmp files or builds. Capture:
  [final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-statistics-verify.json).
- Next correct remaining historical statistical records and reassess inference,
  then the disclosed controlled runtime replay, final consumers/size/retention.
  Replay was not launched; locally available Node thread CPU and GC controls
  are not measurement evidence. Separate power sums and the complete original
  reference inventory remain open. This is not whole-audit completion.

## Checkpoint 61 — retained planner/derivative statistical correction

- Previous turn is PROGRESS: checkpoint 60 corrected four statistical campaigns,
  preserving all raw observations and qualifying historical confidence claims.
- Read all nine planner/derivative timing runners and checkers completely.
  The five timing campaigns contain 199 comparisons / 10,672 measured rows:
  original planner (71), native follow-up (10), WASM (22), derivative (80),
  derivative endpoint consumer (16). The forty-block native planner follow-up
  is also affected; more blocks did not remove the low-bit resampling defect.
- Full reanalysis passes 2026-09-11T00:59:50.331Z; complete recomputation and
  historical-output/independent-control check passes 01:01:56.864Z, both code 0 /
  null signal with empty stderr. Every paired point estimate, marginal median,
  raw observation and old interval is preserved. 187 of 199 interval endpoint
  pairs change; seven directional claims become inconclusive, none reverses or
  changes from inconclusive to directional. Corrected bootstrap below/above-one
  counts: original planner 36/3, native follow-up 5/1, WASM 11/1, derivatives
  52/0, endpoint consumers 5/0. Both interval methods remain conditional on
  suitable independent/common-distribution timing blocks, without multiplicity
  adjustment or algorithm-level attribution.
- BigInt Bernoulli convolution independently verifies order-interval ranks and
  coverage at n=6/12/40; n=40 uses ranks 14 and 27, coverage
  1057205379912/1099511627776. Fourteen deliberately corrupted raw/summary records
  are rejected. Original checkers reproduce old intervals only as archival
  integrity evidence, not as statistical validation.
- Deep native/WASM planner and degree-8/order-128 derivative benefits remain
  directional with both methods. The known 2.7% native coarsening and 5.3% warm-
  WASM costs persist. The 4,096-bit fresh-WASM ratio 0.689714 now has bootstrap
  [0.668933,1.256309]; the sole apparent derivative slowdown ratio 1.042047 has
  [0.976620,1.222695]. Both become inconclusive, not proven unchanged.
- Existing independent exact planner/enclosure oracles, test membership, sizes,
  1,002 allocation rows and 320 retention rows recheck unchanged. This is offline
  evidence rechecking, not new Rust regression/benchmark/Memcheck/size execution.
  Source checks cover all 956 live identities, 175 point-candidate files, full
  planner/derivative snapshots and the four bound historical manifests. No
  reason to reverse those two retained decisions is established; their scope
  and unfavorable controls remain disclosed. Other retained statistics remain
  unqualified pending review.
- Recorder binds 48 artifacts / three successful captures. Findings:
  [checkpoint 61](exactcore-hyper-comparison/audits/continuation/calcium/retained-statistics-v61-findings.md).
  Analysis adds 353,998 workspace bytes; no new /tmp files. Capacity capture:
  16,237,924,352 /tmp bytes available. No source-tree copy, production/donor edit,
  new retained change, cleanup/deletion, commit or push. Donor coverage unchanged.
- Captured final verification passes 2026-09-11T01:14:54.853Z, code 0 / null
  signal: 48 bound artifacts / three successful captures, one output record /
  8,156 bytes, empty stderr. All commands are terminal. Checkpoint 60's terminal
  evidence and artifacts recheck; its full four-campaign recomputation and the
  historical 47-chain were not rerun. Capture:
  [final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/retained-statistics-verify.json).
- The other retained transfers, remaining historical experiments, full source
  inventory and unselected point/power-sum candidates remain open. This is not
  final audit completion or a blanket performance requalification.

## Checkpoint 62 — proof/reuse, facts and monic statistical correction

- Previous turn is PROGRESS: five planner/derivative campaigns corrected,
  original observations preserved and final verification passed. Revalidation
  checks all 48 checkpoint 61 artifacts and all 956 live identities unchanged.
- Read twelve relevant runners/verifiers completely (1,222 lines). Eight CPU
  campaigns contain 25,776 measured rows / 723 paired comparisons, including
  the previously rejected overlapping reuse run. Do not rehabilitate that run
  by correcting its estimator. Additional-answer versus Unknown comparisons
  stay separate from matching recorded outcomes; monic known/degree equality
  alone does not prove identical representations or downstream proof availability.
- Read supporting bounded-allocation runner completely (77 more lines), for
  1,299 tooling lines total and no new donor credit. Revalidate all source/
  evidence entries in eight relevant manifests and retained snapshots of
  180/953/954 files in their historical versions.
- Historical eighteen-checkpoint verification passes 2026-09-11T01:19:25.861Z,
  18 records / 19,692 bytes. Corrected reanalysis passes 01:25:53.698Z; complete
  recomputation and twenty corruption controls pass 01:28:30.694Z, preserving
  all eighteen historical output records verbatim before the new result. All
  exit 0 / null signal with empty stderr. This is offline evidence checking,
  not new Rust/numerical/benchmark/memory/consumer/size execution.
- Accepted CPU: 24,048 rows / 675 comparisons; 626 interval endpoint pairs
  change, 39 directional comparisons become inconclusive, none gains or
  reverses direction. Rejected overlap: 1,728 rows / 48 comparisons, still
  ineligible despite re-estimation. Accepted below/above-one counts are
  proof-query 15/6, proof-numeric 17/6, reuse-confirm 18/13, reuse-numeric 33/9,
  first-touch 44/57, facts 28/8 and monic 48/6. Relations remain separated:
  216 numeric-contract, 378 matching-recorded-outcome, 81 additional-answer
  comparisons. Matching known/degree does not prove full representation equality.
- Rechecked 5,040 separate allocation rows. The 441 instrumented three-block
  timing intervals remain withdrawn, not CPU evidence. An independent eight-
  case sign enumeration verifies only 75% coverage for a bounded min/max median
  interval at n=3 under continuous IID sampling. Twenty deliberately corrupted
  records/summaries fail closed, including invented baseline equality and
  incorrect per-worker/sixteen-warm-query normalization. Missing historical
  pilots/timestamps are disclosed; no fabricated calibration reconstruction.
- Deep unresolved first-touch still has ratio 3.772397 to baseline, corrected
  [3.682088,3.864299]; warm ratio to v2 is 0.284191 with [0.279352,0.288462],
  while baseline warm remains cheaper. Degree-16 retained log-self/v1 ratio
  0.069457 has [0.065662,0.071966]; baseline Unknown is different work. Worst
  monic point ratio 1.066729 retains [1.025035,1.148326]. Monic allocation/byte
  savings persist in 56/162 groups, peak savings in 44, never higher in this
  measured corpus and equal live deltas. No reason to reverse the three
  completeness-first decisions is established; unfavorable controls persist.
- Recorder binds 78 artifacts / four successful captures. Findings:
  [checkpoint 62](exactcore-hyper-comparison/audits/continuation/calcium/proof-facts-monic-v62-findings.md).
  Analysis adds 2,254,179 workspace bytes, no new /tmp files; capacity capture
  records 16,237,924,352 /tmp bytes available. No production/donor edit, new
  retained transfer, source copy, binary, benchmark, cleanup/deletion, commit or
  push. Donor coverage remains 1,415 complete / 20 partial / 183,653 read lines.
- Captured final verification passes 2026-09-11T01:38:44.027Z, code 0 / null
  signal: 78 bound artifacts, four successful captures, 19 output records /
  33,573 bytes and empty stderr. First eighteen records match the historical
  output exactly; final record reports corrected statistics and limits.
  All 956 live identities and retained snapshots recheck unchanged; all commands
  are terminal. No checkpoint 60/61 full statistical recomputation or historical
  47-chain rerun. Capture:
  [final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/proof-facts-monic-verify.json).
- Twenty-one of the known 45 sampler matches remain outside addressed campaign
  scopes: log/erf, early root-exp/sign/opaque, polynomial-decision, rank, sign
  filters/mask and complex-product variants plus archived checker copies. This
  is not exhaustive workspace statistical inventory. The full historical impact
  review, source inventory, isolated point follow-up and power sums remain open.

## Checkpoint 63 — complex products, rank and sign-filter statistics

- Previous turn is PROGRESS. Revalidate all 78 checkpoint 62 artifacts and
  current 956 live identities. The full original objective is unchanged.
- Read twelve relevant runners/checkers and archived checker copies completely,
  covering five CPU campaigns: two complex products (192 groups each), rank
  (56), sign summary (48), sign mask (48). Total 536 comparisons / 25,728 CPU
  rows, with 3,216 separate allocation rows. Rank's sixteen additional-answer
  groups remain separate from baseline Unknown; sign outcomes include certainty
  and proof stage, not only the sign.
- Use the already verified archived complex-product source bindings and prove
  their only checker change is the binding import. Later retained e-planner
  source changes must not be confused with changes to the historical candidates.
- Complete reads cover 1,034 lines in twelve statistical scripts plus 228 lines
  in seven source/oracle helpers. No new donor credit. All five historical
  manifests recheck (71/68/41/67/41 artifacts), along with both 955-file complex
  pairs, both 956-file sign pairs and 774 rank-candidate source entries.
- Corrected reanalysis passes 2026-09-11T01:59:05.450Z; complete recomputation,
  four archived checkers and thirty corruption controls pass 02:02:00.860Z.
  All point estimates and marginal medians persist. Of 536 intervals, 486 change
  endpoints and 34 directional comparisons become inconclusive; none gains or
  reverses direction. Corrected below/above counts: complex v1 61/35, v2 60/14,
  rank 0/40, sign summary 1/7, sign mask 1/5. The sixteen newly answered rank
  groups stay separate from forty matching-known-status groups. Full sign
  outcomes, including certainty/stage, are checked against the exact oracle.
- All 3,216 allocation rows recheck, without CPU inference from instrumented
  clocks. Unresolved width-32 retained rank remains 48.006656 times baseline,
  corrected interval [41.739881,51.477548], requested bytes 47,320→1,141,240/query.
  V2's 60 candidate-reaching groups retain 55 below-one intervals, but bypass
  controls have fourteen above-one intervals and 33 groups retain higher peak
  demand. Sign savings remain one allocation in 24 groups per variant, but
  sampled costs survive. All five prototypes remain unselected.
- Historical 23-checkpoint rank verification passes 01:46:49.369Z; its output
  is reproduced verbatim before the new checker result. The exact 24-case
  BigInt shoelace/Machin oracle reruns and agrees byte-for-byte. No fresh Rust,
  backend, Memcheck, consumer, size or benchmark executions; not the full 47-chain.
- Expanded frozen top-level .mjs inventory: 370 files / 65 text matches, all
  45 known LCG matches plus twenty other potential matches. The initial overly
  broad filename exclusion omitted fourteen legacy scripts, including two known
  checkers; the incomplete snapshot is preserved. A correction's too-narrow
  membership assertion failed; its source/capture is preserved. Two nested
  sandbox captures exited 0 with missing stdout and are not qualified. Separate
  approved read-only inventory/oracle captures have full output. Six qualified
  gates are distinguished from those four incomplete/failed/output-missing
  captures; no false all-green claim or semantic inventory closure.
- Recorder binds 95 artifacts. Findings:
  [checkpoint 63](exactcore-hyper-comparison/audits/continuation/calcium/prototype-statistics-v63-findings.md).
  Analysis is 1,638,145 workspace bytes; /tmp available 16,237,924,352 bytes.
  No new /tmp files, source-tree copy, binary, build, production/donor edit,
  retained transfer, cleanup/deletion, commit or push. Donor coverage unchanged
  at 1,415 complete / 20 partial / 183,653 unique read lines.
- Captured final verification passes 2026-09-11T02:06:57.692Z, code 0 / null
  signal: 95 bound artifacts, 24 records / 35,110 bytes, empty stderr. First
  23 records match the historical rank output exactly. All 956 live identities
  and frozen source bindings recheck; all commands are terminal. No full
  60/61/62 statistical recomputation or historical 47-chain rerun. Capture:
  [final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/prototype-statistics-verify.json).
- Nine known sampler matches remain outside corrected scopes: log/erf, early
  root-exp/opaque/sign, polynomial-decision and their relevant checkers. The
  twenty other potential matches need semantic classification. Continue point
  candidate attribution/final consumer/size qualification, power sums, remaining
  donor reads and full reference reconciliation. The original objective is open.

## Checkpoint 64 — early scalar/proof and polynomial statistics

- Previous turn is PROGRESS. Revalidate all 95 checkpoint 63 artifacts and
  current 956 live identities; the original full objective remains unchanged.
- Read the remaining nine known sampler scripts completely (880 lines), plus
  four early evidence verifiers. Discovered nine CPU datasets, including three
  successive log corpora, erf, eager root/exp, sign query/numeric, opaque costs
  and polynomial decisions: 333 comparisons / 15,984 CPU rows. Six allocation
  datasets contain 263 groups / 3,156 rows; their three-block timing intervals
  remain withdrawn, not CPU evidence.
- The first polynomial CPU group remains excluded for recorded formatting
  overlap. Earlier log datasets have fewer cases than the surviving expanded
  runner; preserve those historic memberships and disclose unavailable earlier
  source/pilot/timestamp provenance rather than inventing it. Review underlying
  capture/source evidence before assigning any inference eligibility.
- Four supporting verifiers were read completely (462 lines), for 1,342 tooling
  lines and no new donor credit. Revalidate five historical manifest maps
  (5/123/126/40/74 entries), the 176-file initial Hyperreal baseline and two
  frozen polynomial CPU binaries, as well as current live identities.
- Historical fourteen-checkpoint verification passes 2026-09-11T02:12:49.192Z;
  reanalysis 02:17:18.331Z; full recomputation and 27 rejection controls
  02:20:00.240Z. All exit 0 / null signal with empty stderr. Historical output
  exactly matches the first fourteen records of checkpoint 63's rank chain and
  precedes the new result. The old estimator is reproduced only as withdrawn
  archival evidence; a pathological four-block control exposes its degenerate
  [0,0] interval, versus corrected [0,1].
- Nine datasets total 333 comparisons / 15,984 CPU rows. Conditional historical
  stratum: 298 comparisons / 14,304 rows, 284 changed interval pairs, thirteen
  directional claims become inconclusive. Older log source-limited stratum:
  34 comparisons / 1,632 rows, thirty changed pairs, one newly inconclusive.
  Rejected polynomial overlap: one comparison / 48 rows, never rehabilitated.
  None gains or reverses direction. Conditional relations: 192 separate numeric
  contracts, 59 matching recorded outcomes, 47 additional certified answers.
- All 3,156 allocation rows recheck. Their 263 instrumented three-block timing
  intervals remain withdrawn, not CPU evidence. No polynomial allocation
  campaign or unrecorded pilot/timestamp is invented. Numeric sign-proof
  request/byte counts remain matched; opaque unresolved warm-difference counts
  also match. These are cumulative requested counts, not peak/RSS bounds.
- Expanded-log warm unresolved ratio 5.057751 retains bootstrap
  [4.548175,5.211756], requested bytes 768→2,936/query. Eager-root fresh sqrt(2)
  exponent ratio 0.428908 improves, but hot-operand ratio 4.545358 retains
  [4.419047,4.742386], bytes 1,016→4,104. Opaque depth-128 warm-pair ratio
  4.327541 retains [4.220225,4.687004], bytes 1,768→11,220. Erf's four lost
  serialized sign facts persist. Polynomial v1's new degree-16 answer is
  different work from baseline Unknown. Early versions remain unselected;
  later retained cache/fact-aware replacements remain unchanged.
- A separately captured read-only check finds sixteen of eighteen recorded CPU
  executable paths no longer match their historical hash; only two frozen
  polynomial executables match. Reusable paths were overwritten by subsequent
  work, not replacements for historical qualification. No separate root-eager
  CPU command capture was found; existing raw/summary metadata is not silently
  strengthened. Most early pilots and every per-observation timestamp are
  missing; only polynomial pilot calibration is fully reconstructed.
- Recorder binds 83 artifacts / five successful captures. Findings:
  [checkpoint 64](exactcore-hyper-comparison/audits/continuation/calcium/early-statistics-v64-findings.md).
  Analysis 990,238 workspace bytes; /tmp available 16,237,924,352 bytes. No new
  /tmp file, source-tree copy, build, binary, production/donor edit, retained
  transfer, cleanup/deletion, commit or push. No fresh Rust/backend/numerical/
  Memcheck/consumer/size/benchmark execution; historical verifier rewrites an
  identical existing qualification-summary.json. Donor coverage unchanged.
- Captured final verification passes 2026-09-11T02:32:09.608Z, code 0 / null
  signal: 83 bound artifacts, five successful captures, fifteen output records /
  23,254 bytes, empty stderr. First fourteen records preserve the historical
  output exactly. All 956 live identities and relevant frozen bindings recheck;
  all commands are terminal. No full 60/61/62/63 statistical recomputation or
  historical 47-chain rerun. Capture:
  [final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/early-statistics-verify.json).
- All 45 original known sampler matches now map to addressed dataset scopes
  across 60–64. This is not whole-workspace semantic statistical closure:
  twenty additional top-level text matches require classification. Continue
  point runtime/consumer/size follow-up, power sums, donor/reference reads and
  complete inventory reconciliation. The original full objective remains open.

## Checkpoint 65 — residual estimator classification and focused WASM diagnostic

- Revalidated checkpoint 64's 83 artifacts and 956 live identities. Read all
  twenty additional statistical text matches completely: 1,371 tooling lines,
  no donor-source credit. No independent estimator was found. These are
  corrected-helper consumers, interval classifiers, deterministic correctness
  fixture loops, prose, or the inventory scanner itself. The scope is the frozen
  370-file top-level inventory, not whole-workspace semantic closure.
- Reused the three frozen WASM modules for a predeclared eight-group, four-pass
  diagnostic: default/single-threaded/single-threaded/default GC, CPU 6, unchanged
  batch lengths, 36 balanced blocks/pass. Measured wall time inside enclosing
  thread/process CPU and resource counters; no overhead subtraction, dropped
  slow rows, pass pooling or multiplicity-adjusted inference.
- The initial attempt failed before measured batches: a 71,495 ns qualification
  query returned zero thread CPU, invalidating the strictly-positive-counter
  assumption. Preserve its original code/plan, full row, 128 empty controls,
  failure log and exit-one capture. The amendment was registered at
  2026-09-11T02:47:32.433Z; legal zeros remain in the data and a zero paired
  block median makes the whole metric inference unavailable. No completed
  measured CPU reading is zero; 164 of 192 qualification thread readings are.
- Four completed captures run 02:48:03.978–02:49:12.089Z, all code 0 / null
  signal, seven output records each, empty stderr/failure logs. They preserve
  6,912 measured batches, 192 qualification queries and 1,024 empty controls.
  All full final values match frozen native records. Three selected groups
  recover baseline's uncertified answer: different work, not same-result costs.
- The Unknown control's four demand/eager paired wall ratios are
  0.745623/0.746558/0.759917/0.755285, below one under both interval methods.
  Small retry case 42 ratios are 1.028024/1.018765/1.011498/1.010959; all bootstrap
  intervals are above one, but the last order-statistic interval includes one.
  Cases 3/4 show inconsistent small effects. The old case-17 baseline slowdown
  is not reproduced; no equivalence or universal performance claim.
- Empty thread CPU is zero in 1,023/1,024 controls, not a high-resolution oracle.
  Median wall/process envelope controls are about 0.11–0.13/3 microseconds.
  Default post-GC wall/thread ratios 2.23/2.36 versus single-threaded 1.19/1.18,
  and voluntary switches 804/13/14/818, support a runtime-noise concern. Fixed
  pass order, counter granularity and host/runtime effects prevent clean causal
  attribution; process-minus-thread time is not exact GC time.
- Analysis passes 02:50:29.677Z; full recomputation and 27 rejection controls
  pass 02:52:18.380Z. Legal-zero preservation, whole-metric unavailability,
  constant-ratio and independent n=36 Pascal/binomial coverage checks pass.
  The latter uses ranks 12/25 and coverage 66739206840/68719476736. There are
  192 correlated group/reference/channel comparisons, not independent studies.
- Findings:
  [checkpoint 65](exactcore-hyper-comparison/audits/continuation/calcium/point-attribution-v65-findings.md).
  Manifest binds 91 artifacts and six successful captures. Completed raw query
  records total 30,365,957 workspace bytes; analysis 1,603,578 bytes, with
  controls/captures extra. /tmp available 16,237,899,776 bytes at recording.
  No new /tmp artifact, build, source copy, production/donor edit, allocation/
  consumer/size gate, retention, cleanup/deletion, commit or push. Five retained
  transfers and donor coverage (1,415 complete / 20 partial / 183,653 lines)
  remain unchanged.
- Proceed to final selected-demand consumer/size qualification and the retention
  decision, preserving retry costs. No indefinite timing-attribution expansion.
  Power sums, remaining donor/reference reads and complete inventory
  reconciliation remain required by the original full objective.
- Captured final verification passes 2026-09-11T02:57:01.555Z, code 0 / null
  signal: 91 bound artifacts, six successful captures, three output records /
  2,239 bytes, empty stderr. Recomputes the focused diagnostic, rechecks all
  956 retained live identities and 175 candidate files, and preserves the initial
  failed attempt. All commands are terminal. No full historical 47-chain or
  60–64 statistical recomputation. Capture:
  [checkpoint 65 final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-attribution-verify.json).

## Checkpoint 66 — selected-demand consumer and size qualification

- Previous turn is PROGRESS. Revalidated all 91 checkpoint-65 artifacts, its
  terminal successful capture, and 956 live identities. No previous process
  remains active. Original full ecosystem objective remains unchanged.
- Qualify the already measured demand-gated point-witness repair in Hypercurve.
  Copy only the 355-file Hypercurve consumer (23,841,357 original bytes), retaining
  the frozen scalar dependencies and 175-file solver candidate in place. Only
  the copied consumer manifest's dependency paths change. Reuse the shared
  two-job, nonincremental Cargo target and existing baseline application binaries.
- Required gates: dependency-graph identity, all-feature release consumer tests,
  all-target/all-feature Clippy, selected-feature WASM build, unchanged example
  execution and representative native sizes. Preserve the prior numerical,
  cold/history, allocation and corrected native/WASM timing evidence; do not
  confuse these consumer gates with an independent new oracle or universal
  performance/size proof. No retention until the evidence supports it.
- All gates pass. Metadata is identical after only two intended path
  normalizations: 187 packages/nodes, one of each Hyper crate, unchanged lock.
  Release consumer passes 1,764 / zero failed / nine ignored in 46 suites,
  with all 1,773 names/outcomes matching the prior guarded consumer.
  Formatting, warnings-denied all-target/all-feature Clippy and selected-feature
  WASM library build pass. Six frozen/new example executions agree.
- Stripped demand example deltas versus baseline: basic +1,568 bytes,
  arrangement +1,536; versus eager +432/+384. ELF BSS grows +2,528/+2,568
  versus baseline; no universal memory/size claim. Frozen cold collector sizes
  are rechecked, not newly rebuilt: native +4,712, WASM +793 versus baseline;
  history WASM +702. Build-path/layout contributions remain disclosed.
- Four new dedicated executable files total 50,279,984 bytes under
  /tmp/calcium-point-consumer.qyV59b. Shared cache growth is additional;
  recorded /tmp availability 15,424,610,304 bytes. No cleanup/deletion.
- Findings:
  [checkpoint 66](exactcore-hyper-comparison/audits/continuation/calcium/point-consumer-v66-findings.md).
  Final verification passes 2026-09-11T03:22:18.866Z, code 0 / null signal,
  87 artifacts / 21 gates, one record / 760 bytes, empty stderr. All commands
  are terminal. Source coverage and five retained transfers remain unchanged
  at this pre-integration checkpoint. The evidence supports selecting demand
  for live integration; no sixth retention is claimed yet.

## Checkpoint 67 — live demand-gated witness repair retained

- Apply only the exact qualified algebraic_binary.rs from the immutable demand
  candidate. Before integration the live file is clean in Git and matches the
  956-file pre-retention map; existing resultant.rs, root_isolation.rs and
  root_isolation_monic_tests.rs changes are preserved.
- Run live all-feature debug/release solver tests, Clippy, formatting, WASM
  build and candidate/live dependency-graph checks. Preserve all earlier
  source versions/captures: their historical live flags describe their original
  state, not the new retention. The new retained binding must distinguish
  frozen historical qualification from current live identity explicitly.
- Retained the exact qualified file as the sixth continuation transfer. Its
  SHA-256 is e97877da6029f26e256943ca70f4005576574689f33288b71874165b4c8e87e6;
  the other 955 recorded live source/support identities are unchanged. The
  diff adds 52 net algorithm lines and 374 test/helper lines, including eight
  tests, with no public API, dependency, scalar-layout or cache-field change.
- The repair waits for typed InvalidInterval, proves endpoint equality under
  STRICT, and retries the same refiner. Approximate multiplication/division
  also needs strict reconstruction of the entire image. Polynomial vanishing,
  containment, uniqueness and half-open ownership obligations remain intact.
  Earlier public corpora recover 825 answers per policy and preserve all other
  5,616 records; repeated policies/histories are not independent defect counts.
- Live gates run 2026-09-11T03:26:15.670Z–03:28:55.616Z, all passing. Debug
  and release each pass 811 tests in eight suites, zero failed/ignored; all
  names/outcomes match the qualified candidate. All-target/all-feature Clippy
  with warnings denied, formatting and all-feature release WASM compilation
  pass. Complete candidate/live metadata matches after only expected path
  normalization: 139 packages/nodes, unchanged lockfile, no duplicate Hyper crate.
- Hypercurve's 1,764 passed/nine ignored consumer tests and the prior numerical,
  memory, native/WASM and corrected timing campaigns remain qualification of
  the frozen identical candidate, not newly rerun live-path campaigns. The
  unchanged-result allocation/peak penalties of eager repair are avoided, but
  recovered-answer retries add work and allocations. Accept the representative
  stripped-example increases of 1,568/1,536 bytes and disclosed BSS costs for
  the completeness gain; no universal performance/memory/size improvement.
- Retention evidence check passes at 03:30:15.722Z. Final captured verification
  passes at 03:36:46.453Z, code 0 / null signal: 127 bound artifacts, ten gates,
  one output record / 1,449 bytes, empty stderr. It verifies both 956-file
  source maps, the candidate's 175 files and the consumer's 355 files. All
  commands are terminal. Findings:
  [checkpoint 67](exactcore-hyper-comparison/audits/continuation/calcium/point-retained-v67-findings.md);
  [final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/point-retained-verify-v67.json).
- Current entry point, from the Calcium audit directory:
  `node verify-point-retained-v67.mjs --point-live`. Earlier scripts asserting
  a pre-retention live map are historical, not valid current-state checks.
  The new verifier preserves/hashes earlier evidence; it does not rerun the
  historical 47-chain or the complete 60–65 statistical campaigns.
- Reused the existing two-job nonincremental build cache. No new dedicated
  /tmp artifact or source copy for live adoption; cache growth is additional.
  Recorded /tmp availability after live gates: 15,381,413,888 bytes. No deletion,
  donor edit, commit or push. Coverage remains 1,415 complete / 20 partial /
  183,653 read lines. Continue separate power-sum work, donor/reference reads
  and the full inventory reconciliation; the original objective remains open.

## Checkpoint 68 — isolated power sums qualified on the retained witness baseline

- Previous turn is PROGRESS: checkpoint 67's final verification and report
  updates completed. Revalidated the current 956-file map before proceeding;
  no prior process remains active. The full original objective is unchanged.
- Rebase only checkpoint 52's private integer-scaled power-sum constructor on
  the frozen, now-retained demand-gated solver. Copy only the 175 solver files,
  share the existing frozen scalar dependencies and shared two-job Cargo cache,
  and preserve the original power-sum candidate/evidence unchanged. No live edit.
- Recheck the entire signed polynomial oracle, public STRICT/approximate full
  records, nonrational/state controls and debug/release regression gates before
  timing. The old 825 witness failures must not be treated as an optimization
  benefit; the comparison baseline already contains that independent repair.
  Preserve genuine Unknown, zero-resultant, nonzero and degree controls.
- Power-sum timing/allocation, broader coefficients, consumer/platform/size gates
  and a retention decision remain required if this candidate stays promising.
  No new donor-source reading credit is claimed for this transfer checkpoint.
- Copied 175 solver files / 6,094,998 original logical bytes; added the unchanged
  276-line private module and 24-line wrapper from checkpoint 52. Scalar trees
  and the original candidate are reused, not copied again. The first run passes
  numerical/tests but Clippy rejects five redundant references in the extracted
  fallback. Preserve its exit-101 capture and exit-one outer run, exact initial
  source, bindings and binaries. The correction removes only those references.
- Initial gates run 2026-09-11T03:50:06.627Z–03:52:15.469Z; corrected gates run
  16:54:37.074Z–16:56:49.433Z. The gap is explicit; collection/build elapsed
  times are not benchmark data. Supplemental gates finish at 16:58:46.885Z.
  All processes are terminal, including the failed first run.
- Corrected debug/release all-feature tests pass 814 each in eight suites,
  zero failed/ignored. Full names/outcomes preserve the retained 811 tests and
  add exactly three power-sum tests. Solver and both harness Clippy gates pass
  with warnings denied; formatting and all-feature release WASM compilation
  pass. WASM execution remains unqualified for this candidate.
- Signed polynomial oracle passes all 4,840 cases / 1,210 independent
  determinants, including 136 fallbacks and 32 zero resultants. Public STRICT
  and APPROXIMATE_512 each preserve all 6,441 full records and pass 48,059
  rational/Sturm assertions / 1,178 independent resultants. Outcomes remain
  5,126 Transformed, 241 denominator guards, 40 Undecided, 65 nonisolating and
  968 unsupported degree. These are repeated corpora, not new defect counts.
- All 384 nonrational/state records match the retained baseline: 11,248
  independent field/serialized-value assertions per variant; 316 Transformed,
  24 denominator guards, six Undecided, six nonisolating and 32 InvalidEvidence.
  Complete harness metadata matches after only intended path normalization:
  33 packages/nodes, one copy of each relevant Hyper crate, identical lockfiles.
- Four serial Memcheck runs preserve full native outputs and report zero
  errors/definite/indirect/possible lost bytes. Public reachable bytes fall
  1,840,544→1,364,672; extended 48,648→48,296. Cumulative requested bytes fall
  232,232,789→175,485,125 and 130,905,028→130,256,844 respectively. These include
  collector/setup/output work, not isolated allocation or peak/RSS/boundedness
  evidence. Per-query allocation and CPU campaigns remain unrun.
- Twelve dedicated files, including four preserved initial candidate binaries,
  total 34,012,184 bytes in /tmp/calcium-power-rebased.xUuvlb. The correction
  archives only the changed source file, not another whole source tree. Cache
  growth is additional; post-verification /tmp availability 15,265,529,856 bytes.
  No live/donor change, new retention, deletion, commit or push.
- Findings:
  [checkpoint 68](exactcore-hyper-comparison/audits/continuation/calcium/power-rebased-v68-findings.md).
  Evidence check passes at 17:00:40.885Z. Final captured verification passes
  17:03:02.358Z, code 0 / null signal: 169 bound artifacts / 43 gates, including
  41 successful and two preserved failed captures; one record / 1,233 bytes,
  empty stderr. It checks the current 956-file live map and isolated 176-file
  map, preserving historical failures without rerunning the full 47-chain.
  [Final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/power-rebased-verify-v68.json).
- Next: broader coefficient/carrier qualification, then matched native/WASM
  timing and per-query allocation with retained/fresh/bypass controls. Require
  consumer/size gates and corrected public path documentation before retention.
  Six retained transfers and coverage 1,415 complete / 20 partial / 183,653
  unique read lines remain unchanged. Full inventory/reference work stays open.

## Checkpoint 69 — wide coefficients qualified; zero-factor division is next

- Previous turn is PROGRESS. Revalidated checkpoint 68's 169 bound artifacts
  and current live/candidate maps; all previous processes are terminal.
- Reuse the retained baseline and corrected isolated constructor without a
  new solver copy or production edit. Add public point-root cases covering
  every admitted ordered degree pair, coefficient scales beyond machine words,
  repeated roots, unused zero roots and real carriers with nonreal conjugates.
  Check complete signed polynomials against the independent determinant oracle
  and selected values by exact rational arithmetic under both policies.
- Keep zero-divisor and zero-resultant controls explicit. The latter remain a
  separate known completeness gap, not a reason to weaken the oracle or credit
  this optimization with the earlier witness repair. Exactness/completeness
  findings take precedence over starting the pending timing campaign.
- Preserve all input/output/source identities and use the existing two-job
  nonincremental cache. No donor reading credit or new retained change yet.
- The corpus contains 1,840 arithmetic cases, both policies and every one of
  the 23 ordered degree pairs with product at most nine. Five height parameters
  (1/31/65/129/257), signed rational scales and four carrier families cover
  distinct, repeated, unused-zero and complex-conjugate cases. Selected roots
  are rational points; this is not arbitrary interval/state or exhaustive input
  coverage. Both variants emit 3,680 query records plus a terminal record.
- All full outputs agree: 3,540 Transformed, 80 denominator guards and 60
  Undecided per variant. The independent signed full-polynomial/selected-value
  oracle passes 81,610 assertions with 1,791 distinct determinant constructions.
  Multiplicities, endpoints, exact witnesses, vanishing and metadata are checked.
  Maximum widths: input numerator 2,577 bits, denominator 258, primitive input
  coefficient 2,320 and resultant coefficient 4,639. Height parameters are not
  final coefficient widths. Release harness builds/runs and Clippy pass; full
  metadata agrees after intended paths: 33 packages/nodes, identical locks.
- Independently examined forty earlier zero-resultant controls and thirty new
  wide cases before policy duplication: seventy records / forty normalized
  carrier pairs. Every divisor interval excludes zero exactly. Removing its
  unused x^k carrier factor preserves the selected divisor root, yields a
  nonzero resultant and gives one root in every quotient image. All 810 probe
  assertions pass. This is a mathematical opportunity, not an implemented fix,
  seventy independent bugs or a timing result. Existing literal zero divisors
  remain guarded. Earlier public input bytes also recheck against the immutable
  checkpoint-68 baseline output (SHA-256 7b628db1453275c3ad4700523e7bdf36d1384904dd92f1afc78ad651a81ae350).
- Before implementation, preserve STRICT nonzero evidence and original
  validation/proof replay. Audit degree admission and the shared-square-free
  shortcut: original-source equality is not equality of a deflated internal
  carrier. Removing x^k can also change the scalar sign of an already nonzero
  resultant; do not hide orientation changes by weakening full-polynomial tests.
- Execution gates finish 2026-09-11T17:11:25.768Z, wide oracle 17:14:48.039Z,
  independent deflation probe 17:55:16.338Z. Final evidence assembly catches an
  audit-only undefined/null mismatch in forty optional metadata slots. Preserve
  both exit-one captures and original probe/evidence/verifier code; an uncaptured
  record attempt hits the same assertion and creates no manifest. Explicit null
  slots leave serialized mathematical output byte-identical. Fresh corrected
  probe passes at 18:14:54.114Z, evidence check at 18:16:29.151Z. No mathematical
  assertion or Hyper source is changed by that correction.
- Findings:
  [checkpoint 69](exactcore-hyper-comparison/audits/continuation/calcium/power-wide-v69-findings.md).
  Corrected final verification passes at 18:17:46.511Z, code 0 / null signal:
  85 bound artifacts / eighteen gates, sixteen successful and two preserved
  failed captures; one record / 872 bytes, empty stderr. All commands are
  terminal. [Final verification](exactcore-hyper-comparison/audits/continuation/calcium/results/power-wide-verify-canonical-v69.json).
- No solver/scalar copy: two new executables total 5,616,216 bytes under
  /tmp/calcium-power-wide.ijyxgC. Input occupies 2,838,932 workspace bytes;
  paired output 28,016,100. Recorded /tmp availability 15,254,159,360 bytes;
  shared-cache growth is additional. No new Rust suite, Memcheck, WASM execution,
  consumer/size/timing gate, live/donor edit, retention, deletion, commit or push.
- Next is an isolated certified divisor-zero-factor completeness trial, ahead
  of power-sum timing. Keep six retained changes and donor coverage unchanged
  (1,415 complete / 20 partial / 183,653 uniquely read lines). The remaining
  reference inventory and original full ecosystem objective remain open.
- Implementation follow-up should include an oversized shared-source control
  such as x(x-1)^8 divided by itself at selected root 1. Deflating only the right
  carrier makes its square-free reduction differ from the left, even though the
  original sources were equal. Also check signed primitive outputs for cases
  whose original resultant was nonzero: Res(P,x^k S)=Res(P,x)^k Res(P,S), with
  Res(P,x)=(-1)^deg(P) P(0), can introduce a sign factor. These are prospective
  regression requirements, not claims that a new implementation has passed.

## Checkpoint 70 — certified divisor zero-factor trial (numerically qualified, isolated)

- Rechecked checkpoint 69's 85-artifact manifest and current retained sources.
  No live edit or seventh retention. Created a solver-only exclusive copy of the
  retained point-demand baseline: 175 files / 6,094,998 bytes, sharing frozen
  scalar dependencies/cache. The separate power-sum candidate remains unchanged.
- Read current binary-transform admission and payload validation: source
  leading coefficients must be STRICT-certified nonzero. Stored polynomial
  degree therefore supports the resultant sign identity. Preserved the existing
  oversized square-free stage before deflating only the divisor's unused x^k.
- Implemented exact zero coefficients plus STRICT divisor-interval exclusion
  of zero, even under approximate policy. Preserved signed primitive outputs
  when the original resultant is nonzero, and kept original source evidence,
  image construction, containment/vanishing/uniqueness replay unchanged.
- Ordinary carriers borrow a suffix; square-free owned storage is drained in
  place. Production file diff is 56 added / one removed line, including docs
  and test declaration; new test module is 260 lines / seven tests. No runtime
  API/dependency/cache/layout change. All 956 live identities remain unchanged.
- Paired results: earlier STRICT and approximate corpora each recover forty
  Undecided plus twelve degree-rejected divisions, preserving 6,388 other full
  records per policy; wide corpus recovers sixty and preserves 3,620; new
  degree/shared/sign corpus recovers twenty and preserves 112. Total 16,692
  queries, 184 new answers / 16,508 unchanged records, not 184 distinct defects.
  Independent signed-polynomial/interval checks: 128,652 assertions / 6,441
  distinct determinants, including 14,168 complete successful polynomial
  comparisons. New controls include shared x(x-1)^8, 3×4→3×3 admission and
  still-refused 4×4→4×3. Literal zero and strict-Unknown guards persist.
- Extended nonrational/state/history outputs remain byte-identical: 384 queries,
  11,248 independent checks. Fresh default tests pass 817; all-feature debug and
  release pass 818 each, eight suites, no failed/ignored tests. All prior 811
  all-feature names persist with seven additions. The default-only omission is
  the existing feature-gated tensor-resultant opaque-zero pruning test.
- Solver/collector Clippy, formatting, release all-feature WASM compilation
  pass. WASM execution remains open. All 32 dependency packages/nodes and root
  edges match after the solver-path normalization, 33 total packages/nodes;
  collector name/targets intentionally differ, locks match after name change.
- First unit capture: six pass / one fail, caused by a wrong positive-leading
  expectation for a negatively scaled shared carrier. Monic-GCD division keeps
  that sign; an independent determinant confirms both expected signs. Corrected
  only the test. Subsequent full tests/Clippy pass but the assertion needs a
  rustfmt wrap; formatting and enclosing gate fail. Both old test versions and
  all three failed captures remain. Production algorithm bytes never changed
  after the initial trial; fresh final gates rerun on the formatted test source.
- Final execution gates finish 2026-09-11T19:07:41.047Z, independent oracle
  19:07:59.070Z, extra gates 19:08:36.656Z, evidence assembly 19:10:45.469Z.
  Three native Memcheck runs have zero errors and no definite/indirect/possible
  loss, with exact matching outputs. Reachable public/extended/new-degree bytes
  are 1,843,616 / 48,648 / 28,424, not peak/RSS, boundedness or matched savings.
- Final verification passes 19:13:45.345Z: 145 artifacts / thirty gates,
  27 successful and three preserved failed; one record / 692 bytes, empty stderr,
  code 0 / null signal. All commands terminal.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-v70-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-verify-v70.json).
  Current entry point is zero-factor-final-sources-v70.mjs; initial and
  signed-only source checkers intentionally describe historical test states.
- Four new executables total 11,393,464 bytes in /tmp/calcium-zero-factor.1YCn5a;
  one old baseline executable is reused. /tmp initial available 15,254,134,784
  bytes, recorded post-build 15,054,901,248; shared-cache growth is additional.
  No deletion, commit or push. No timing/consumer/representative-size claim.
- Next: matched native/WASM CPU and separate per-query allocations for bypass,
  signed unchanged results, new-answer and oversized/shared cases; consumer and
  stripped-size gates before deciding retention. Do not count extra new-answer
  work as an equal-result speedup. Six retained changes, donor coverage unchanged
  (1,415 complete / 20 partial / 183,653 uniquely read lines). Remaining donor
  references and the full original inventory remain open.

## Checkpoint 71 — isolated zero-factor native cost qualification

- Previous turn: progress, with isolated implementation and independent
  numerical qualification, not retention. Rechecked checkpoint 70's 145 bound
  artifacts, all 956 live identities and the 176-file candidate.
- Completed matched native CPU and separate allocation qualification. Reused
  both solver trees and shared cache; no new source-tree copy. Covered forty
  cost cases, all sixty-six degree/sign/shared controls, and selected wide cases
  under both policies and retained/fresh-source lifecycles. Fresh source does
  not imply a cold process or empty global caches.
- Predeclared balanced variant order, matched iteration counts from preserved
  pilots, randomized common group order and no outlier removal. Full records
  checked before/after each measured group; equal-result and new-answer work
  stay separate. Corrected 20,000-replicate bootstrap and independent binomial
  order-statistic intervals remain conditional and not multiplicity-adjusted.
- Four release collectors pass 1,824 full-report preflight checks, Clippy and
  final formatting. CPU executables contain no allocation-instrumentation
  symbols; allocation hook is unchanged. Complete dependency metadata agrees
  after two intended paths, 33 packages/nodes, identical locks. All 956 live
  and 176 candidate source identities persist; no solver edit or new Rust suite.
- Campaign 2026-09-11T19:30:58.448Z–19:32:34.424Z is terminal: 1,824 preserved
  pilot rows, 21,888 CPU rows (24 balanced process pairs), 7,296 allocation rows
  (four separate balanced pairs, one/sixteen iterations). Common CPU counts
  span 3–20,000, targeting three milliseconds from the slower pilot; 1,636 CPU
  rows remain below one millisecond, not dropped. CPU 2 / sibling 10 measured,
  orchestration CPU 0; powersave governor and changing load recorded, not a
  reserved/fixed-frequency or continuously observed contention environment.
- 408 groups have unchanged full results; 48 produce new answers. All 196
  same-result bounded-deflation groups have paired medians 0.210–0.809× baseline,
  both intervals below one, and lower requests/bytes/peak at both batch sizes.
  Existing sqrt(2)/selected-1 quotient: 8.944→4.957 µs, 55→43 requests,
  9,000→5,240 requested bytes, 2,584→1,816 peak above starting live bytes.
- Disclosed costs: nondivision groups span 0.974–1.038, nonzero-constant bypass
  0.986–1.017, zero guards 0.996–1.074 and still-rejected oversized inputs
  1.004–1.027. Worst early literal-zero guard is 100.2→107.7 ns; no unmeasured
  code-layout explanation is asserted. Newly admitted degree-nine example is
  6.574→44.508 µs and adds 5,568 peak bytes; new-answer work is not an equal-work
  slowdown. Wide newly supported answers reduce cumulative allocations but can
  add 544 peak bytes. Batch peak/live deltas are not total peak/RSS/boundedness.
- Mixed-workload allocation rows retain 28 unchanged-path groups with small
  positive paired-median differences (0.5–1.5 requests / 52–192 bytes). An
  independent post-campaign case-isolated diagnostic covers all seven affected
  cases: 56 fresh processes, 448 full records, 224 paired request/byte/peak/live
  comparisons exactly equal. Supports prior-work/state dependence, not a named
  cache mechanism, universal zero cost, or removal of original observations.
  Diagnostic finishes 19:37:47.135Z; its elapsed values are not CPU evidence.
- One positive and nine deliberate corruption controls pass: altered signed
  polynomial/endpoint, completion/count/policy, zero time, backward clock and
  instrumented-CPU records are correctly rejected. Fresh complete statistical
  recomputation and evidence assembly pass at 19:47:55.425Z, code 0 / null signal.
- An uncaptured sandboxed manifest-record attempt subsequently fails with
  spawnSync nm EPERM (terminal exit one; tool output truncated), before writing
  a manifest. The unchanged command succeeds with approval outside the sandbox.
  No source/data/check is altered and no failure capture is invented. All 138
  captured execution gates pass; prior checkpoint failures remain preserved.
- Final verification passes 2026-09-11T19:54:15.448Z: 541 bound artifacts /
  138 successful captured gates, one record / 767 bytes, empty stderr, code 0 /
  null signal. All commands are terminal; the uncaptured sandbox denial above
  is distinct from those captured gates.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-native-v71-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-cost-verify-v71.json).
- Four new executables total 11,304,616 bytes in /tmp/calcium-zero-cost.cEHSth.
  Raw pilot/CPU/allocation output occupies 25,135,254 workspace bytes; auxiliary
  preflight/diagnostic/evidence files are additional. Initial /tmp available
  15,054,876,672 bytes, recorded post-campaign
  15,031,971,840. No new solver/scalar copy, deletion, commit or push. Native
  costs justify continuing qualification, not retention. Six retained changes
  and donor coverage (1,415 complete / 20 partial / 183,653 uniquely read lines)
  are unchanged. Next: WASM execution, representative consumer/stripped-size
  gates; power sums and the complete remaining reference inventory stay open.

## Checkpoint 72 — isolated zero-factor WASM qualification (complete slice)

- Previous turn: progress (native cost qualification), not retention. Rechecked
  checkpoint 71's 541 artifacts, 138 captured gates, all 956 live and 176
  candidate source identities. Six continuation transfers remain retained.
- Built four import-free WASM collectors against the same retained baseline and
  isolated candidate, sharing existing source trees/cache. All four offline
  release builds and Clippy gates pass; Rust formatting passes. Both 33-package/
  33-node dependency graphs match after path normalization within each bridge;
  all four lockfiles are identical.
- Read the existing history bridge/work and current native rational importer;
  reuse their checked wire formats and preserve complete report comparisons.
  No live edit, donor credit or source-tree copy. Initial /tmp available:
  15,031,947,264 bytes.
- Actual execution passes 2026-09-11T20:46:20.889Z–20:50:12.611Z, code 0 /
  null signal, empty stderr/failure log: 33,384 full rational observations,
  1,824 cost controls and 3,072 nonrational/history observations. Full native
  source/report records match throughout. The 16,692 paired rational queries
  preserve 184 new answers and 16,508 unchanged reports. The extended-field
  oracle passes 89,984 checks; the prior rational checker is freshly rerun.
- One persistent rational instance per variant handles the full corpus;
  cost/history observations each use a fresh instance. Node v22.22.2 / V8
  12.4.254.21-node.39, CPU 2, explicit no-Liftoff/no-tier-up/no-lazy-compilation
  flags and host GC are recorded. Qualification elapsed values are not benchmarks.
  Rational batches drop all reports; history batches retain the last report.
- Seventy-six disposable-instance ABI negative controls trap as expected;
  four legal sequences pass, with two additional expected reprepare traps.
  History finish is intentionally repeatable. Two valid records pass and all
  21 deliberate corruptions are rejected. Saved records are independently
  streamed/rechecked in full, including corpus order and instance continuity.
- Full-corpus linear-memory capacity stays 33,816,576 bytes per instance after
  initialization; cost instances use 1,703,936 bytes. History final capacity
  ranges 1,310,720–1,572,864 bytes for both variants. These are capacities, not
  allocation/peak/RSS measurements or proof of equal memory costs.
- Four new modules total 5,818,881 bytes in /tmp/calcium-zero-wasm.ATQP07;
  full raw observations occupy 53,098,727 workspace bytes. Recorded post-check
  /tmp availability is 15,003,926,528 bytes. Existing cache reused; no deletion,
  commit or push. No duplicated solver/scalar source tree.
- Final sealed verification passes 2026-09-11T21:08:01.072Z: 118 bound artifacts /
  23 successful captured gates, one record / 707 bytes, empty stderr, code 0 /
  null signal. All commands terminal. Prior captured failures remain preserved;
  expected ABI traps are passing controls, not failed execution gates.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-wasm-v72-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-wasm-verify-v72.json).
- Six retained changes and donor coverage (1,415 complete / 20 partial /
  183,653 uniquely read lines) are unchanged. Next: matched WASM cost campaign,
  representative consumer/stripped-size gates before retention. Separate power
  sums, remaining references and full inventory reconciliation remain open.

## Checkpoint 73 — matched isolated zero-factor WASM costs (complete slice)

- Previous turn: progress, completing actual WASM correctness qualification.
  Freshly revalidated checkpoint 72: 118 artifacts / 23 successful gates,
  all 956 live and 176 candidate files; six retained transfers unchanged.
- Reused the four frozen modules (only the rational pair is measured), current
  cost corpus and exact importer. No build or source-tree copy. Predeclared
  24 balanced paired blocks for each of persistent/fresh module lifetimes,
  common pilot-derived counts and identical per-pair group orders. Keep source
  lifecycle distinct from module lifetime and recovered answers distinct from
  equal-result comparisons. Qualification and timing remain separate datasets.
- All 104 workers pass 2026-09-11T21:14:13.958Z–21:21:26.803Z: 3,648 pilots
  and 43,776 measured rows, 912 group/mode combinations, 24 paired blocks each.
  All complete reports and source wires match qualified native evidence.
  Pilot-derived common counts stay fixed at 1–13,715 persistent / 1–12,940 fresh.
  All 3,604 measured sub-millisecond batches and other observations are retained.
- Every one of the 196 unchanged-result bounded-deflation groups per mode has
  both corrected intervals below one: persistent ratios 0.219561–0.788387,
  fresh 0.216277–0.789963. The selected existing quotient drops from 13.209 to
  7.407 µs persistent, 12.630 to 7.245 µs fresh. Six bypass combinations have
  both intervals above one, largest paired ratio 1.032490. No blanket speedup
  or unmeasured cache/code-layout attribution. Intervals remain conditional and
  not multiplicity-adjusted.
- New-answer costs are separate: the selected degree-nine result takes 75.004
  versus the old rejection's 17.024 µs persistent, 71.211 versus 16.465 µs fresh.
  All 48 new-answer groups per mode remain visible; they are not equal-work
  slowdowns/speedups. The candidate is still isolated and unretained.
- Node/V8, flags, CPU 2 affinity, CPU 0 orchestration, balanced variant/mode
  orders, complete plans and environmental snapshots are verified. Powersave
  governor, 3.46–3.50 GHz snapshots and changing load are disclosed; no reserved
  CPU or continuous contention/independence guarantee. Setup/warmup/GC/output
  remain outside timing; fresh instances are preconditioned, not cold startups.
- Independent analysis and repeated recomputation pass; 36 record corruptions
  and 14 small stream/order/terminal corruptions are rejected, with four valid
  controls. Stream fixtures are explicitly authored two-row excerpts, not
  invented short execution histories. No earlier failure is overwritten.
- Final statistical/source replay passes 2026-09-11T21:25:57.740Z: 430 bound
  artifacts / 111 successful captured gates, one record / 604 bytes, empty
  stderr, code 0 / null signal. All commands terminal. The record-only path
  binds evidence; the final ordinary verifier freshly recomputes statistics.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-wasm-cost-v73-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-wasm-cost-verify-v73.json).
- No new binaries/source-tree copies; raw observations occupy 49,537,622
  workspace bytes. Initial /tmp availability 15,003,901,952 bytes, post-campaign
  15,003,926,528. Sampled persistent linear capacity is 1,703,936–1,835,008 bytes,
  fresh 1,703,936, for both variants: not allocation/peak/RSS measurements.
  No cleanup, live edit, retention, commit or push. Six retained transfers and
  donor coverage remain unchanged. Re-read the candidate diff/seven tests and
  prior consumer harness; this adds no donor credit. Consumer/stripped-size
  qualification is next; power sums and full remaining inventory stay open.

## Checkpoint 74 — isolated zero-factor consumer and size gates (complete slice)

- Previous turn: progress, completing matched WASM costs. Rechecked checkpoint
  73's 430 artifact hashes / 111 gates and successful final capture; current
  source bindings are revalidated before the new consumer copy/build.
- Copy only the 355-file qualified Hypercurve consumer and substitute its two
  Hypersolve paths to the isolated zero-factor candidate. Share frozen scalar
  dependencies and the existing two-job/nonincremental cache. Run full consumer
  release regressions, Clippy, formatting and a WASM build; compare all test
  names/outcomes and metadata, not aggregate counts alone. Measure and execute
  representative basic/arrangement stripped examples against the preserved
  retained-baseline binaries, whose hashes have been rechecked.
- Audit caller paths and keep actual branch coverage separate from representative
  consumer/size evidence. No live adoption is implied by a passing consumer build.
  Initial /tmp availability: 15,003,926,528 bytes. Six transfers remain retained;
  full donor/reference inventory and power sums are still open.
- Consumer-only copy contains 355 files / 23,841,619 original logical bytes
  (the already-qualified manifest is longer than the earlier unmodified source).
  Only its two Hypersolve paths change. Both 187-package/187-node metadata graphs
  match after the intended path normalization. Full release regressions pass at
  2026-09-11T21:45:04.866Z: 1,764 passed, nine prior ignored, 46 suites; all 1,773
  names/outcomes independently match checkpoint 66. Formatting passes.
- All-target/all-feature Clippy passes with warnings denied at 21:45:36.384Z.
  Release WASM library compilation with triangulation/svg/hershey passes; it is
  not new WASM geometry execution. Both unchanged default-feature examples and
  their preserved retained-baseline binaries run successfully with matching
  complete outputs. Seventeen nested gates and their wrapper finish successfully
  at 2026-09-11T21:47:57.925Z, code 0 / null signal.
- Stripped basic/arrangement sizes each grow 5,744 bytes (about 0.05%);
  ordinary sizes grow 9,680/9,752 bytes. Text +5,596/+5,580, data +144/+144,
  BSS -1,648/-1,648. File/section changes are not runtime allocation or isolated
  function size. Rust/Cargo/strip/size versions match the preserved baseline;
  build paths, linker layout and dead-code selection remain confounders.
- Selected caller ranges are recorded separately: strict-first adapter policy,
  exact-point/scalar bypasses, independent-root construction and the indirect
  rational-image quotient route. The simple geometry examples are not credited
  with direct divisor-deflation branch coverage. No donor coverage added.
- Final sealed verification passes 2026-09-11T21:51:02.880Z: 84 bound artifacts /
  19 successful captured gates, one record / 881 bytes, empty stderr, code 0 /
  null signal. All commands terminal; all earlier failed captures remain intact.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-consumer-v74-findings.md),
  [caller review](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-consumer-caller-review-v74.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-consumer-verify-v74.json).
- Four dedicated files total 50,310,904 bytes in /tmp/calcium-zero-consumer.LrNEZu;
  prior baseline files are reused untouched. Recorded post-build /tmp availability
  is 14,190,510,080 bytes; shared-cache growth is additional to dedicated files.
  No broad stack copy, cleanup, live edit, commit or push.
- Decision: select this exact candidate for live integration. Its 184 additional
  qualified answers, unchanged full records and intended-path native/WASM gains
  justify the disclosed bypass/new-answer and representative size costs under
  the user's priorities. This is not yet a seventh retention. Next: promote only
  the tested main-file diff and seven-test module, bind the changed live map and
  run live regressions/lint/platform gates, preserving pre-existing solver edits.
  Six retained changes and 1,415 complete / 20 partial / 183,653 uniquely read
  donor lines remain unchanged. Power sums and the full remaining inventory stay open.

## Checkpoint 75 — certified zero-factor removal retained

- Previous turn: progress, completing consumer/size qualification and selecting
  the candidate. Fresh pre-adoption verification passes checkpoint 74's 84
  artifacts / 19 gates and all current 956 live / 176 solver / 355 consumer files.
- Promote only the tested algebraic_binary.rs diff and new seven-test module.
  Preserve the existing resultant/root-isolation edits. Freeze before/after maps
  before changing live source; the post-change map has 957 source/support files.
  Historical current-source verifiers must not be mistaken for this new state.
- Rerun default and all-feature debug/release solver regressions, compare every
  test name/outcome with the qualified candidate, then Clippy/format/WASM and
  live/candidate metadata. Do not record the seventh transfer before those pass.
  Initial /tmp availability: 14,190,485,504 bytes; reuse the existing cache.
  No new whole-stack copy or dedicated executable snapshot is planned.
- Promoted exactly the qualified main file and 260-line/seven-test module.
  Both live hashes match checkpoint 70; all other 955 previous identities and
  the frozen 176-file solver / 355-file consumer remain unchanged. The source
  checker also rechecks 1,279 historical artifacts without invoking stale live flags.
- All fresh live gates pass: 817 default tests, 818 all-feature debug and release
  each, eight suites/configuration and zero failed/ignored. Every name/outcome
  matches the qualified candidate; the one feature-only tensor test is expected.
  Clippy all-target/all-feature with warnings denied, formatting and all-feature
  release WASM library build pass. Full Cargo metadata matches after only the
  intended solver/scalar path normalization: 139 packages/nodes, identical locks,
  exactly one each hyperlattice/hyperlimit/hyperreal/hypersolve. Tool versions match.
- The initial sandbox environment query failed spawnSync rustc EPERM before
  tests, also failing its outer driver. Both captures and original runner/origin
  are preserved. The separately captured approved driver passes at
  2026-09-11T22:07:12.984Z; independent evidence check passes 22:07:33.772Z.
- Final sealed verification passes 2026-09-11T22:10:23.914Z, code 0 / null signal,
  one record / 1,275 bytes, empty stderr: 1,331 artifacts / fourteen gates,
  twelve successful and two preserved sandbox failures. All commands terminal.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/zero-factor-retained-v75-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/zero-factor-retained-verify-v75.json).
- Retained as the seventh continuation transfer: 184 additional qualified
  answers and intended-path native/WASM gains justify disclosed bypass/new-answer
  costs and representative stripped-example growth of 5,744 bytes each. Earlier
  numerical/consumer/cost runs qualify the byte-identical candidate; they are
  not relabeled as fresh live-path measurements. No universal improvement claim.
- Current entry point: `node verify-zero-factor-retained-v75.mjs`. Checkpoint
  67 and 68–74 dynamic live-source flags describe older states; preserve their
  recorded evidence and do not use them as current-state checks. Any further
  power-sum experiment must account for the new zero-factor baseline.
- Shared-cache reuse only: post-build /tmp availability 14,060,937,216 bytes.
  No dedicated executable snapshot, source-tree copy, cleanup, commit or push.
  No new donor credit: 1,415 complete / 20 partial / 183,653 uniquely read lines
  unchanged. Remaining source/reference reads, unresolved transfers and full
  inventory reconciliation still prevent completion of the original audit.
- Report, README and root ledger updated. Both scoped Git whitespace checks
  pass. An optional post-report tool dispatch referenced an undefined local
  variable before launching its verifier; the subsequent direct verifier passes
  unchanged. This was not a failed numerical gate or a change to sealed evidence.

## Checkpoint 76 — rational-angle source qualification and completeness candidate

- Previous turn is progress: checkpoint 75 retained the exact zero-factor change
  and sealed its live qualification. Fresh checkpoint-75 verification passes;
  the 957-file live map is unchanged. No subagents or production edits planned.
- Continue the previously unread qqbar roots-of-unity and forward rational-angle
  trigonometric implementation/tests in both pinned repositories. Read both
  copies in full, record exact ranges/hashes, and compare against Hyper's actual
  scalar paths. Navigation/inventory and similarity do not count as reading.
- Independently qualify useful algebraic/value/branch contracts before proposing
  a transfer. Reuse existing FLINT libraries and build resources; no broad source
  copies. The separate power-sum candidate and full remaining inventory stay open.
- Completed 32 full-file reads / 1,964 new lines: eight implementations and
  eight tests per pin. Archived +16 / 1,015 lines; current +16 / 949 lines.
  Root-of-unity, exp(pi*i*x), sin/cos/tan/cot/sec/csc source and test paths are
  covered; inverse trig, remaining qqbar and recursive support remain open.
  Both manuals explicitly exclude near-word-boundary inputs. The initial
  doubling concern is a documented limit, not an in-contract defect; no
  excluded-boundary probe or donor patch was made.
- Independent field/cyclotomic oracle passes 28,586 checks over 3,787 records:
  3,408 values, 368 pole observations, 944 root-recognition observations and ten
  non-root controls. Complete minimal polynomials, both exact dyadic component
  enclosures, width, canonical root indices and NULL-output behavior checked.
  Ten record/stream corruptions rejected. Native/Memcheck outputs identical,
  910,636 bytes each; zero errors/live blocks, 397,808 allocations/frees and
  41,257,680 requested bytes including setup/caching/output. Not a benchmark.
- Live Hyper comparison uses exact pi/12 formulas, certified equality budgets
  -64/-256 and period shifts 0/5/2^1024. Debug/release outputs match, 1,728 rows
  each: 360 radical Equal, 192 radical Unknown, 552 periodic Equal, 552 perturbed
  NotEqual and 72 repeated poles. Unknown covers 32 function/residue identities,
  not 192 separate defects. Six checker corruptions rejected. Audit-package
  Clippy/format and full ten-package/node dependency checks pass.
- Candidate: preserve compact SinPi/TanPi expressions and add exact twelfth-turn
  radical relation certificates. No implementation, matched timing or retention
  yet. Existing arbitrary-precision reduction and exact pole behavior already
  cover the donor's basic design. No numerical misdecision observed in this probe.
- Four failed captures remain: initial formatting, the i32-to-i64 probe compile
  error, its outer driver, and the first evidence verifier's overly strict
  treatment of rustfmt's optional trailing-comma removal. Original sources,
  runners/origins and outputs preserved. A follow-up read of failed empty stdout
  also failed before useful work; corrected verification passed without changing
  live source or mathematical records.
- Final sealed verification passes 2026-09-11T22:38:30.116Z: 1,429 artifacts /
  twenty gates (sixteen successful, four preserved failed), one record / 1,082
  bytes, empty stderr, code 0 / null signal. All commands terminal.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-trig-v76-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-trig-verify-v76.json).
  Current combined entry point: `node verify-qqbar-trig-v76.mjs`; checkpoint 75's
  retained-state verifier also remains valid because no live file changed.
- One dedicated 13,872-byte C binary; two Rust probe products remain in the
  existing shared target (3,138,880 / 1,943,208 bytes), without snapshot copies.
  Recorded /tmp available 14,025,854,976 bytes. No library rebuild, broad source
  copy, cleanup, production/donor edit, external report, commit or push.
  Seven retained transfers unchanged. New continuation coverage: 1,447 complete /
  20 partial / 185,617 uniquely read lines. Full inventory completion remains open.

## Checkpoint 77 — isolated twelfth-turn relation initial qualification

- Checkpoint 76's current verifier passes; the retained 957-file live map is
  unchanged. No new donor read credit or production change in this checkpoint.
- Investigate a bounded proof-only Q(sqrt(2), sqrt(3)) normal form for the 32
  unresolved twelfth-turn identities. Preserve compact SinPi/TanPi expressions,
  approximation caches, exact pole rejection and legitimate Unknown outcomes.
  Place any proof before caching an unresolved sum, including descendant sums.
- Candidate isolation is Hyperreal-only (181 original source/support files),
  with source hashes checked before copying. Reuse the existing build target;
  /tmp has about 14 GiB available. No cleanup or broad stack snapshot planned.
- Required before retention: independent field/sign/branch checks, bounded-work
  and cache/query-order controls, full regression suites, matched bypass and
  intended-path timings, allocation/size checks and relevant consumer gates.
  This entry records initial qualification, not a retained improvement.
- Implemented bounded proof-only field arithmetic, exact pi/12 admission and
  a fallback before unresolved sum caching. Two new files / two minimal existing
  file edits in the isolated copy only; 307 implementation and 442 test lines.
- Independent checks pass: 6,561 field signs, 512 polynomial/matrix pairs,
  256 Pell convergents plus 512 mixed cases, 336 rational square-class cases.
  Exact branch/domain/budget, cold/warm/failed/descendant and JSON/CBOR controls pass.
- Unchanged public probe: 1,728 rows/profile, debug/release byte-identical;
  192 repeated Unknowns covering 32 identities become Equal, other 1,536 rows
  unchanged. Six stream corruptions rejected; no pole or unequal-control regression.
- Matched default debug/release: baseline 756, candidate 763 tests each.
  Matched all-feature debug/release: baseline 859, candidate 867 each. Every
  baseline name/outcome retained, no failed/ignored tests. Clippy/format/WASM
  compile pass; all 126 metadata packages/nodes equal after root normalization.
- Memcheck output matches native: zero errors/lost bytes; 44,002 allocations,
  43,834 frees, 4,474,572 requested bytes; 21,584 bytes/168 blocks still reachable.
  Whole-process control only, not matched memory/peak or timing evidence.
- Three failed development captures retained (missing sqrt helper, two test-only
  Clippy loops, stopped driver). Four status-zero/empty Node completion captures
  are rejected; approved verifiers/drivers produce expected records and reuse
  individual successful gates. Original/pre-correction test sources preserved.
- Combined evidence passes 2026-09-11T23:31:55.908Z. Manifest: 1,759 artifacts /
  41 captures, comprising 25 accepted final-source gates, nine development
  successes, three failed and four unusable empty captures.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/twelfth-relation-v77-findings.md).
- Final sealed verification passes 2026-09-11T23:34:29.411Z, code 0 / null
  signal, one record / 1,532 bytes and empty stderr. All commands terminal.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/twelfth-verify-v77.json).
- Current checker: `node verify-twelfth-relation-v77.mjs`; 75/76 also remain
  valid. Live 957-file state unchanged; seven retained transfers unchanged.
  No new donor credit: 1,447 complete / 20 partial / 185,617 unique lines.
- Existing build cache reused; new public binaries 3,198,496 / 1,970,744 bytes
  without snapshot copies. Recorded /tmp available 13,994,549,248 bytes. No
  deletion, donor/production edit, external report, commit or push. Both scoped
  Git whitespace checks pass. Cost/consumer gates and full inventory remain open.

## Checkpoint 78 — native cost qualified; current version not selected

- Previous turn is progress: checkpoint 77 implemented and independently
  qualified an isolated candidate. Fresh 77 verification passes; all recorded
  live/candidate sources remain unchanged. No new donor credit or retention.
- Predeclare a native public-predicate matrix covering the 32 identities,
  unequal perturbations, already-supported/structural controls and failed-proof
  controls, with construction-inclusive, fresh-object first-query and warmed
  retained-object lifecycles at two precision budgets. Separate changed-answer
  comparisons from same-result comparisons; do not assume all cases improve.
- Reuse the existing 183-file candidate and live 181-file baseline, with hashes
  before/after builds and measurements. Only small harness packages/binaries
  are added. CPU and allocation binaries are separate. Use the corrected
  deterministic rejection-sampled statistics, balanced paired order and raw
  timestamps; no benchmark overlap or universal performance claim.
- WASM runtime, consumer/size and remaining whole-inventory work stay open.
- 96 cases / 576 groups; four binaries pass 2,304 preflight records. Full
  21-package/node metadata graphs and locks match. CPU binaries have no counting
  allocator symbols; final rebuilds match the preserved products byte-for-byte.
- Main collection completes 2026-09-11T23:51:05.460Z: 2,304 pilot, 27,648 CPU
  and 9,216 allocation rows; 24 balanced randomized CPU pairs, four allocation
  pairs at 1/16 iterations. All raw rows, calibration, order and timestamps checked.
- Recomputed corrected bootstrap and independent median-order intervals; twelve
  corrupted record/stream controls rejected. All 115 capped groups and 92 short
  median batches are preserved; conditional intervals are not multiplicity-adjusted.
- The 32 target identities retain their new Equal answers. Construction-inclusive
  ratios 0.110–0.503; cold-predicate 0.075–0.389; retained 0.760–5.353, with
  48/64 retained groups slower under both intervals. These are changed answers,
  not same-work speedups. Perturbed same-result ratios span 0.586–9.188.
- Worst repeated unequal cotangent: 0.874→8.029 us, paired ratio 9.188;
  6→41 allocation requests, 520→4,744 requested bytes, 408→1,752 peak above
  starting live bytes, zero live delta. Perturbed request/byte counts increase
  in all 192 groups at both allocation iteration counts.
- Case-isolated replication completes 2026-09-11T23:59:29.302Z: six selected
  cases, 192 processes, 864 CPU / 576 allocation rows. Hot tan/cot ratios
  reproduce at 8.712/8.613 with identical allocation counts to the main run;
  failed-proof controls also regress. Post-selection, not universal inference.
- Decision: do not retain this version. Keep its proof domain and numeric nodes
  intact while reducing field-arithmetic and failed-proof overhead. No production
  edit or new retention. Preserve the existing candidate and all cost evidence.
- Initial unused imports corrected; one failed patch context applied no edit.
  Uncaptured initial build stopped on sandbox nm EPERM after one binary copy.
  Approved resume reused it but failed the 20-versus-21 package-count assumption;
  all mathematical preflights passed. Correct finalization/rebuild identity passes.
- Combined evidence passes 2026-09-12T00:06:59.557Z. Four snapshots total
  8,750,456 bytes; no new Hyperreal tree copy, cleanup, donor/live edit, external
  report, commit or push. Seven retained transfers and donor coverage unchanged.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/twelfth-cost-v78-findings.md).
- Final sealed verification passes 2026-09-12T00:13:42.729Z: 2,899 artifacts /
  277 captures (274 current successes, two development successes, one failed).
  One record / 1,407 bytes, empty stderr, code 0 / null signal. All campaign
  commands terminal. Post-compaction polling of the already-closed final session
  reports an unknown process id; the persisted terminal capture and output are
  independently checked. Both tracked donor worktrees remain clean.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/twelfth-cost-verify-v78.json).
  Current verifier: `node verify-twelfth-cost-v78.mjs`; recorded /tmp available
  13,970,505,728 bytes. No source change since this verification.
- Next revision hypotheses only: direct exact twelfth-angle tan/cot field forms,
  skipping exact-zero field coefficients, and reducing failed-proof work without
  losing descendant/cached exact facts. Any revision needs a new isolated version
  and complete correctness/cost qualification; checkpoint 77's candidate and
  checkpoint 78's evidence must remain immutable. No new implementation or
  performance claim is made here.

## Checkpoint 79 — cheaper proof revision, still not retained

- Previous goal turn is progress: final checkpoint 78 evidence is verified and
  the report records rejection of the measured candidate version. Full scope
  remains unchanged; no new donor credit or production change is authorized by
  this narrower experiment.
- Revalidate checkpoint 78, then create an exclusive Hyperreal-only candidate
  from the frozen 183-file checkpoint 77 version. Preserve all original evidence.
  Reuse the shared build target; no broad copy or cleanup.
- Investigate direct exact tan/cot forms and sparse coefficient arithmetic.
  Preserve the existing proof domain, bounded admission, pole rejection, numeric
  expression graph and descendant/cache behavior. Differential old/new evaluator
  tests supplement the independent field and public-predicate checks.
- Requalify every original cost-corpus case, including same-result failed-proof
  controls. Changes remain isolated unless complete correctness, measured cost
  and relevant consumer/size gates justify retention. Remaining references and
  full inventory reconciliation stay open.
- Exclusive copy: 183 files / 5,345,646 bytes, now 184 candidate files. Direct
  exact tan/cot forms, sparse coefficient work and unused-value avoidance retain
  the proof domain. Helper 307→340 lines; new test-only reference module 498 lines.
- Independent original tests preserved; differential checks cover 1,024 sparse
  pairs, 160 coefficient boundaries, 485 tan/cot arguments, 8,760 expressions and
  670 unsupported/deep cases. No admission/value loss observed.
- Matched default baseline/candidate 756/766, all-feature 859/870, in both
  debug/release; all previous test membership/outcomes preserved. Clippy/fmt/WASM
  compile pass, full 126-package graphs match. Public 1,728 rows/profile remain
  byte-identical to 77, including all 32 added identities.
- Three-way native matrix uses unchanged 78 harness/inputs; baseline/prior
  rebuilds are byte-identical to frozen binaries. Full 21-package benchmark
  graphs and locks match; six binaries pass 3,456 preflight records.
- Native collection finishes 2026-09-12T00:53:29.959Z: 65,664 rows comprising
  3,456 pilot / 41,472 CPU / 20,736 allocation. Twenty-four balanced CPU triples,
  six allocation triples at 1/16 iterations. Full membership, timestamps,
  calibration and certificates checked; revised certificates equal prior.
- Hot unequal tan/cot now 0.416/0.422 of prior cost but 3.883/3.963 of live
  baseline in selected -64 retained-pair queries. Requests 41→26, still above
  baseline 6; requested bytes 4,744→2,608 versus 520; peak 1,752→928 versus 408.
  Six rounds agree at both iteration counts. Failed-proof CPU penalties persist.
- Do not retain this version. Preserve the cheaper revision and all exact
  answers; proof scheduling/bypass costs remain unresolved. No new production
  edit, retained transfer or donor credit. No WASM runtime, consumer, representative
  size or new case-isolated replication claim. Seven retentions unchanged.
- The initial summary's identical-certificate flag mislabeled unchanged
  NotEqual outcomes as changed answers. Final summary separates decision and
  certificate identity; all raw samples/statistics and the original summary
  are preserved. Twelve stream corruptions and statistical self-tests pass.
  All 103 capped groups / 92 short baseline median batches remain in the results.
- Memcheck native output matches: zero errors/lost bytes; 41,200 allocations,
  41,052 frees, 4,092,804 requested bytes; 19,000 bytes / 148 blocks still reachable.
- Two development failures (test constructors, overly strict source-format
  comparison), four unusable empty Node captures and their source histories
  remain. Approved confirmation reuses valid individual gates; no exclusive
  preparation/binding is rerun. Uncaptured empty-evidence parsing also failed.
- New frozen benchmark binaries total 4,403,848 bytes; shared cache reused.
  No cleanup, live/donor edit, external report, commit or push.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/twelfth-revision-v79-findings.md).
- Combined evidence passes 2026-09-12T01:03:24.740Z, one record / 58,616 bytes,
  empty stderr, code 0 / null signal. Manifest binds 3,661 artifacts / 149
  captures: 140 current successes, three development successes, two failed and
  four unusable empty captures. Recorded /tmp available 13,929,951,232 bytes.
  Both tracked donor worktrees remain clean; scoped Git whitespace checks pass.
- Follow-up architecture read confirms exact-sign traversal can use a separated
  cached approximation, but an unresolved newly constructed sum does not get
  that shortcut from its children's signs alone. Cached-enclosure proof scheduling
  is a hypothesis to investigate, not a measured fix or permission to discard
  descendant proofs. Remaining qqbar/inverse-trig reads and full inventory remain
  open; do not substitute further prototype tuning for completing those reads.
- Next donor queue: both pins' `qqbar/{asin_pi,acos_pi,atan_pi,acot_pi,log_pi_i}.c`
  and their tests. The current FLINT paths exist and no matching extension entry
  was found in this check. This is queue identification only, not source-read
  credit or an assertion that all historical coverage inventories are reconciled.
- Final sealed verification passes 2026-09-12T01:10:04.036Z: one record / 1,293
  bytes, empty stderr, code 0 / null signal. All commands terminal. Source maps
  remain unchanged; current checker `node verify-twelfth-revision-v79.mjs`.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/twelfth-revision-verify-v79.json).

## Checkpoint 80 — inverse rational-angle source and capability audit

- Previous goal turn is progress: checkpoint 79 independently qualified a cheaper
  isolated proof and rejected production retention on measured costs. Its sealed
  verifier is being rechecked; no live/candidate edit or new retention planned.
- Resume the donor queue at both pins' full `qqbar` asin_pi, acos_pi, atan_pi,
  acot_pi and log_pi_i implementations and tests: twenty files / 1,783 source
  lines by inventory. Confirm prior coverage before claiming new reads.
- Audit branch/domain and recognition contracts, nullable outputs, reduction,
  intermediate expression growth and failure behavior. Build an independent
  bounded corpus using existing native libraries and compare corresponding Hyper
  behavior; source coverage and execution qualification remain separate.
- Reuse existing build storage; no library rebuild, broad source copy or cleanup.
  Remaining support, formal/symbolic/historical references and full inventory
  reconciliation stay in scope.
- Fresh checkpoint 79 verification passes 2026-09-12T01:12:44.144Z with unchanged
  live/candidate maps. The twenty queued source/test files are fully read at both
  pins. Two 33-line log implementations were already covered, so add no duplicate
  credit. The fully read 21-line current private header adds one new file.
- Actual new credit: 19 complete files / 1,738 lines (878 Calcium, 860 FLINT).
  Read records and coverage extensions published; cumulative continuation totals
  are 1,466 complete files, 20 partial, 187,355 unique lines. The manuals are
  rereads; declarations do not credit unexamined callees. Full inventory remains open.
- The bounded inverse-angle corpus emits 2,162 rows + terminal. Independent full
  polynomial and both-component endpoint checks pass: 25,216 checks, 15 rejected
  corruptions, 2,042 recognized rows, 72 nonrecognized and 48 poles. Sixteen
  near-match rows overlap at proposal precision but are rejected exactly.
- Native/Memcheck output is byte-identical (593,765 bytes). Zero errors/live
  blocks; 1,492,610 allocations/frees, 74,269,892 total requested bytes. These
  are collector totals, not per-operation or peak-memory measurements.
- Public degree-384 `x=tan(pi/3360)` returns no recognized angle. Exact integer
  polynomial division and a rational root bracket independently establish the
  true result 1/3360. The helper accepts nearby 1/3359 first; its exact overlap
  rejection stops the search. This violates the documented recognition completeness
  contract, not accepted-result soundness. Current FLINT executed, archived source
  only. Counterexample native process capped at 1 GiB / 30 CPU / 45 wall seconds.
- Original normalized-rational proof and independent homogeneous-Horner replay
  agree; eight primary corruptions and 420 evaluator differential cases pass.
  No floating-point or same-library inverse/forward oracle supplies the proof.
- Hyper capability check produces 1,848 rows + terminal per debug/release,
  byte-identical including full certificates: 584 Equal, 896 NotEqual, 312
  Unknown, 48 poles and eight domain errors. Eleven corruptions rejected. All
  eight denominator-3360 identity rows prove structural equality, including
  signed inputs and huge exact period shifts. Equivalent radicals can stay Unknown.
- Derived acot is explicitly atan(1/x), +pi/2 at zero; there is no claimed Hyper
  acot API. Its structure loss is separated from direct atan behavior. Hyper's
  cubic input is a cos_pi expression; complex logarithm is outside this probe.
- Fmt, warning-denied Clippy, locked offline 21-package metadata and both profiles
  pass. Unchanged 957-file live map and 184-file isolated candidate map rechecked.
  No new production transfer, benchmark, full-stack regression, WASM runtime or
  representative product-size claim. Seven retained continuation transfers remain.
- Two sandboxed zero-exit/empty-result captures are unusable and preserved;
  approved confirmations carry actual result records. Original combined summary
  counted 14 metadata fields instead of 957 files; final wrapper fixes only that
  reporting field and preserves the original script/capture. Mathematical checks
  and source hashes are unchanged.
- Two new C executables total 36,720 bytes; unique shared-target Rust executables
  total 5,596,264 bytes. No source copy, library rebuild or cleanup. Post-build
  /tmp available 13,910,597,632 bytes. Both tracked donor worktrees remain clean;
  scoped Git whitespace checks pass. No external report, commit or push.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-inverse-v80-findings.md).
- Final corrected combined evidence passes 2026-09-12T01:56:58.920Z. Manifest
  records 3,764 artifacts / 25 captures: 22 current successes, one development
  summary and two unusable empty captures. Final sealed verification passes
  2026-09-12T02:06:25.019Z: one record / 1,228 bytes, empty stderr, code 0 / null
  signal. All commands terminal; no production changes. Current checker:
  `node verify-qqbar-inverse-v80.mjs`.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-inverse-verify-v80.json).
- Next verified unread queue: both pins' `qqbar/{asec_pi,acsc_pi}.c` and their
  tests, eight files / 388 lines. This is inventory identification, not read
  credit. Both `root_ui.c` files are already fully covered; do not double count
  them. Remaining support/formal/symbolic/historical scope is unchanged.

## Checkpoint 81 — reciprocal inverses and scalar boundaries

- Previous goal turn is progress: checkpoint 80 added verified source coverage,
  independently proved a recognition-completeness defect and qualified live Hyper
  behavior without a production edit. Recheck its sealed evidence before proceeding.
- Continue both pins' unread asec/acsc implementations and tests, then exact
  floor/ceiling and binary-floating import boundaries. Read the supporting scalar
  extraction/normalization files as needed. Count only actually completed reads;
  separate source coverage, native behavior and proposed Hyper transfers.
- Reuse pinned libraries and the existing build target. No broad copy or cleanup.
  Keep remaining qqbar, support, formal, symbolic and historical references and
  full inventory reconciliation in scope. Retain a production change only after
  independent correctness and proportionate measured-cost qualification.
- Fresh checkpoint 80 verification passes 2026-09-12T02:09:02.571Z. Both donor
  HEADs still match the pinned inventory; all qualified live/candidate sources
  remain unchanged.
- Read all 42 selected files completely: 21 per pin, 979 archived / 923 current
  lines, total 1,902 new lines. Includes the six implementation/test pairs for
  reciprocal inverses, floor/ceil and binary imports, plus nine scalar helpers.
  Manuals and Hyper excerpts are rereads. Coverage records/extensions published:
  cumulative 1,508 complete files, 20 partial, 189,257 uniquely read lines.
- Native collector emits 19,923 rows + terminal, 9,799,577 bytes. All 163,328
  independent checks pass, plus 18 corruptions. Every float exponent/both signs
  with four significands; complex boundary cross-product; reciprocal branches;
  rational/quadratic real-part rounding near integers through +/-2^256 with
  2^-1024 offsets. Denominator/height and numerator polynomials are checked;
  numerator selected-root enclosures are not independently qualified here.
- Native/Memcheck streams match exactly. Zero errors/live blocks, 1,682,521
  allocations/frees, 2,220,444,799 cumulative requested bytes. No peak-memory,
  per-operation allocation or matched performance claim. Both bounded commands
  terminate successfully; upstream tests and archive were not newly executed.
- Hyper debug/release streams match, 19,340 rows + terminal / 4,819,943 bytes each.
  All 18,416 finite imports have exact canonical ratios and expected cached f64
  bits; four infinities and twelve NaNs retain distinct errors. All 322 floors,
  322 ceilings and 322 adjacent-integer results are correct, stable on repeat.
- Derived reciprocal inverses: 336 Equal, 192 Unknown, 48 DivideByZero; controls
  add four Equal, two DivideByZero and four NotANumber. Unknown rows are sixteen
  op/residue inputs repeated across periods/factors/floors, not independent defects.
  Both forward and inverse reciprocal layers are explicitly composed APIs.
  Twelve Hyper corruption controls pass. Fmt/Clippy/21-package metadata pass.
- Four failed captures are preserved: native decoder's inherited exponent guard;
  Rust probe's three absent API names and enclosing driver; Hyper checker's wrong
  error-class assumption. Fixed decoder accepts bounded 16K-bit enclosures without
  weakening containment/order/width checks, with 30 shared-range comparisons.
  Rust source corrections and DivideByZero distinction are bound separately.
  Two earlier successful formatter/metadata captures are development gates.
- Combined evidence passes 2026-09-12T02:55:02.191Z: one record / 34,324 bytes,
  empty stderr, code 0 / null signal. No production edit; seven retentions remain.
  Existing near_integer could reduce generic floor's nine candidates to two;
  this remains unimplemented/unmeasured pending proof/cache differential tests,
  matched costs and appropriate consumer gates. Do not transfer total algebraic
  rounding as a total general computable-real decision procedure.
- One C executable is 22,880 bytes; shared-target Rust executables total 5,628,112
  bytes. Post-build /tmp available 13,904,842,752 bytes. No source copy, library
  rebuild or cleanup. Tracked donor trees clean; scoped Git whitespace checks pass.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/scalar-boundary-v81-findings.md).
- Final sealed verification passes 2026-09-12T03:02:20.667Z: 3,863 artifacts /
  23 captures (17 current successes, two development successes, four preserved
  failures), unchanged 957 live files. One record / 1,200 bytes, empty stderr,
  code 0 / null signal. All commands terminal; no production changes.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/scalar-boundary-verify-v81.json).

## Checkpoint 82 — quadratic extraction

- Previous goal turn is progress: checkpoint 81 completes source and capability
  qualification without adding a production change. Its sealed verification
  passes before this next source pass.
- Continue both pins' unread quadratic extraction implementations and tests;
  compare normalization, selected-root identity and factorization policy against
  Hyper. Keep source reads separate from native qualification and retained ideas.
  Reuse existing libraries and build storage; no broad copy or cleanup.
- Both get_quadratic implementations and both tests are read completely:
  326 archived / 318 current lines, four files / 644 lines. Manuals and recorded
  Hyper excerpts are contextual rereads. Coverage records/extensions published:
  cumulative 1,512 complete files, 20 partial, 189,901 unique lines.
- Native 775-input corpus tests three factorization modes and two cache histories:
  4,650 rows + terminal / 22,104,752 bytes. All 99,587 independent checks pass,
  including original/reconstructed minimal polynomials and both selected
  components, normalization, denominator and radical identities. Twenty
  corruptions rejected. Native and Memcheck streams match; zero errors/live
  blocks, 1,276,837 allocations/frees, 280,998,786 cumulative requested bytes.
  This is whole-collector accounting, not per-operation cost or peak memory.
- Mode 0 leaves 412 nonminimal radicands, as documented. Full factoring yields
  every expected minimal radicand; smooth factoring also does on this corpus,
  without a general guarantee. All repeated coefficients are stable. The second
  query follows the first 2048-bit output request, not a calibrated cache benchmark.
- Hyper public probe covers 467 nonnegative-radicand inputs at two precision
  policies: 934 rows + terminal / 303,998 bytes per profile. Debug/release streams
  and full certificates match. All 934 normalized equalities, 934 squared
  identities and 934 orderings are correct; no Unknown in this bounded corpus.
  Seventy-two nonminimal residual rows still prove equality by bounded refinement.
  Thirteen corruptions pass; fmt, 21-package metadata and Clippy pass.
- Retain bounded square extraction as current policy. Broad factor stripping is
  only a possible cost tradeoff, not a demonstrated completeness improvement.
  No production edit, matched benchmark, new full-stack regression or WASM result.
  Seven retained continuation transfers and the isolated candidate stay unchanged.
- Two zero-exit empty sandbox captures are preserved as unusable (preparation and
  initial driver). Exclusive preparation wrote its files and was not replayed;
  a separate approved confirmation validates hashes/input/oracle/timestamps. The
  approved driver confirms the existing successful native captures. Neither
  native stream was empty. An ad-hoc JSON inspection used the wrong Node file
  loader; corrected read-only inspection succeeded, with no artifact mutation.
- Combined evidence passes 2026-09-12T03:23:43.880Z: one record / 12,042 bytes,
  empty stderr, code 0 / null signal. Both pinned tracked donor worktrees are
  clean. The 957-file live map and 184-file isolated candidate remain unchanged.
  Scoped Git whitespace check passes. No external report, commit or push.
- One C executable / 14,024 bytes and shared-target Rust executables /
  5,557,984 bytes, total 5,572,008 bytes. Post-build /tmp available
  13,910,568,960 bytes. No source tree copy, library rebuild or cleanup.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/quadratic-extraction-v82-findings.md).
- Final sealed verification passes 2026-09-12T03:26:09.135Z: 3,945 artifacts /
  19 captures (17 current successes, two unusable). One record / 1,034 bytes,
  empty stderr, code 0 / null signal. All commands terminal; no production edits.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/quadratic-extraction-verify-v82.json).
- Next verified unread queue: both pins' abs, im, re, re_im and sgn implementations
  plus abs/abs2/re_im/sgn tests: eighteen files / 866 lines. This identifies
  inventory only; no new read credit. Expression conversion, other references
  and full inventory reconciliation remain open.

## Checkpoint 83 — complex components and symbolic conversion

- Previous goal turn is progress: four source files fully read and checkpoint 82
  sealed and verified, with independent quadratic/native/Hyper evidence. Recheck
  the current sealed files before this continuation; do not treat past intent as
  evidence of current source identity.
- Read remaining complex component wrappers/tests and proceed to symbolic
  expression conversion. Separate source coverage, native behavior and Hyper
  transfer decisions. Reuse libraries/build storage; no broad copy or cleanup.
  Keep the complete ecosystem inventory and open transfer hypotheses in scope.

Checkpoint 83 verified results:

- Read all 26 selected component/conversion implementations and tests across
  both pins: 5,728 new lines. Published exact ranges and snapshot hashes;
  coverage reaches 1,538 complete files, 20 partial, 195,629 unique lines.
- Public symbolic conversion confirms two false-success families: nonzero
  arguments lacking Pi can be interpreted as Pi-scaled, and explicit large-Pi
  arguments can leave the old destination unchanged while reporting success.
  Independent exact-field/Taylor checks classify 136 false-success rows across
  repeated policies (112 stale outputs, 24 missing-Pi); not 136 independent bugs.
- Rejected large powers leak temporary integers. Nine native/Memcheck pairs
  include six retained diagnostic exits 97. A two-clear, renamed audit-only
  translation unit produces identical results and zero errors/live blocks at
  256 repetitions for all three controls. No live donor or Hyper edit.
- Native component/serialization corpus passes 15,456 mathematical checks over
  2,496 cases; ten corruption controls pass. Full Memcheck aborts in donor LLL
  assertion. Isolated abs(-7/3 * exp(pi*i/12)) succeeds natively but aborts under
  both Valgrind-none and Memcheck; cause remains unresolved. No clean-memory
  claim, and abort-time live allocations are not called normal-exit leaks.
- Hyper debug/release match all 585 records including certificates. Independent
  validation passes 3,536 checks and 14 corruption controls: 304 Pi, 56 radian
  and 32 pole rows; all 192 norm/magnitude enclosures correct. Norm equality
  proves 144 rows and returns Unknown on 48; no false equality/inequality.
  Fmt, offline 22-package metadata and Clippy pass. No new full-stack/WASM test,
  matched benchmark or production retention; seven continuation changes remain.
- Reuse existing libraries and shared build target. One small audit-only donor
  translation-unit copy was needed for the leak causal control; original no-copy
  protocol remains frozen and this deviation is documented in the addendum.
  Root report and detailed findings are updated. Four C and two Rust audit
  executables total 5,971,960 bytes; observed /tmp available after builds is
  13,888,749,568 bytes. These are audit artifacts, not product-size deltas.
- Combined evidence passes 2026-09-12T04:04:03.653Z: one record / 33,720 bytes,
  empty stderr, code 0 / null signal. Both tracked donor worktrees are clean;
  all 957 live and 184 isolated-candidate files match. Scoped whitespace check
  passes. One truncated ad-hoc coverage inspection failed before making edits;
  the bounded retry succeeds. No external report, commit or push.
- Final sealed verification passes 2026-09-12T04:05:53.813Z: 4,150 artifacts /
  55 captures (43 successful, ten diagnostic failures, two harness failures).
  One record / 1,155 bytes, empty stderr, code 0 / null signal. All checkpoint
  commands terminal; the full ecosystem goal remains active and incomplete.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/symbolic-boundary-v83-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/symbolic-boundary-verify-v83.json).
- Next inventory-only qqbar queue: 16 archive and 19 current files, including
  multivariate evaluation, matrix eigenvalue/root wrappers, random generation,
  printing, remaining power/composed-op tests and current squarefree roots/test
  registration. These have not received new read credit. Complementary-angle
  norm certificates, prior rounding/proof-scheduling hypotheses, other references
  and full requested-inventory reconciliation remain open.

## Checkpoint 84 — remaining qqbar source and test coverage

- Previous goal turn is progress: checkpoint 83 completed 26 source files and
  independently classified donor correctness/memory defects and Hyper capability,
  then sealed and verified the evidence. Recheck that evidence against current
  files before continuing, rather than treating prior intent as current identity.
- Finish the remaining qqbar implementation/support/test files in both pins;
  compare evaluation, root and power strategies with Hyper. Scope remains the
  whole original ecosystem inventory, not only qqbar or the easiest tests.
  Keep the complementary-angle norm proof and previous hypotheses open pending
  rigorous qualification. Reuse existing libraries and shared build storage.

Checkpoint 84 verified results:

- Completed the remaining 35 qqbar files across both pins, two generic evaluator
  files, and the unread middle of gr/qqbar.c: 37 new complete files, one partial
  completed, 5,324 new unique lines. Exact ranges are published in the read
  records and cumulative coverage extensions. Execution qualification is separate.
- Large-exponent monomials at x=1 abort in the public wrapper but succeed in the
  width-aware generic dispatcher. Five bounded trap-instrumented root inputs
  stop; debugger evidence identifies roots_poly_squarefree.c:158, the unchecked
  signed degree product. No enormous uninstrumented root calculation was run.
- Three failed build attempts are preserved: harness const mismatch, donor
  signedness warning under Werror, and missing system UBSan runtime. Trap
  instrumentation requires no package installation or library rebuild.
- Initial quadratic cleanup controls reject before root-vector allocation and
  are clean. The cubic follow-up reaches it: limit-4 failures leak 408 bytes per
  call, giving 408/13,056/52,224 live bytes at 1/32/128 repetitions. All three
  Memcheck diagnostics exit 97. Limit-8 success controls are clean. No general
  root-wrapper memory guarantee is claimed.
- Normal-input corpus: 244 cases plus terminal, three monomial orders, aliases,
  eigenvalue/rational-root wrappers and repeated roots. Native/Memcheck streams
  match; zero errors/live blocks. Combined independent checker validates 1,078
  algebraic values, their minimal polynomials and both selected components;
  fourteen corruptions are rejected. Original wrong packed-width checker
  assumption and missing audit-only polynomial header are preserved as failures.
- Hyper matches all 117 records / 33,952 bytes in debug/release, including full
  certificates. All 160 enclosures are independently correct. Equal 38, Unknown
  six: polynomial lengths 127/128/129 at two policies. Twelve corruption controls
  pass. Fmt, Clippy and offline 21-package graph pass after removing an audit-only
  redundant cast; original source/failure and identical numeric streams retained.
- Both qqbar inventories are fully read: archive 149 files / 14,405 lines;
  current 153 files / 14,086 lines. Cumulative continuation: 1,576 complete files,
  19 partial, 200,953 unique lines. Wider donor support, other ecosystem references
  and relation/proof-scheduling transfer experiments remain open.
- Combined evidence passes 2026-09-12T05:13:26.237Z; both tracked donor worktrees
  are clean, and 957 live / 184 isolated-candidate files match. Scoped whitespace
  check passes. One ad-hoc gate listing was truncated; a bounded summary retry
  succeeded. No production edit, new retention, matched benchmark or new full-stack
  or WASM qualification. Seven continuation transfers remain retained.
- Final sealed verification passes 2026-09-12T05:15:30.461Z: 4,498 artifacts /
  101 captures (79 successful, eleven diagnostic failures, nine harness failures,
  two environment failures), one result record, empty stderr, code 0 / null
  signal. All commands terminal. Four C and two Rust executables total 5,500,896
  bytes; observed /tmp available after builds is 13,883,142,144 bytes. Reused
  existing libraries/shared target; no source-tree copy, broad cleanup, commit
  or push. The whole original ecosystem goal remains active and incomplete.
  [Findings](exactcore-hyper-comparison/audits/continuation/calcium/qqbar-remainder-v84-findings.md),
  [final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/qqbar-remainder-verify-v84.json).

## Checkpoint 85 — broader inventory and coq-aern

- Previous turn is progress: checkpoint 84 completed both qqbar source-directory
  inventories and independently qualified bounded native/Hyper results, then
  sealed three donor defect families. Fresh verification passes before this turn.
- Historical queue review confirms coq-aern, ternary reals, Clerical,
  TypeTopology, Bishop, Lean ComputableReal, Acorn, ROSCoq and the adjacent
  algebraic systems remain unaudited. Historical closed rows are not newly
  qualified by this review; unresolved experiments and unavailable native gates
  remain separate from source-read coverage.
- Begin coq-aern from its public upstream, pin before reading credit, and inspect
  its formal/extraction/runtime contracts. No large toolchain install or broad
  build is preauthorized by this plan. Reuse existing tooling where available.
- Calcium remains open: 54 inventoried unread text files, including pycalcium,
  examples, documentation/build support and utils_flint algorithms. The empty
  Python test initializer is included in that file count. Finishing qqbar did
  not close those files, current FLINT's wider support, or transfer experiments.
- No agents are used. The original objective, priority order and full inventory
  remain unchanged; this checkpoint is not a completion claim.

Checkpoint 85 verified results:

- Pinned coq-aern `bc11353f450cf866b47c3985eee6150a5f99cf00`: 135 tracked artifacts,
  including 88 text files / 42,419 lines. Actual read credit: 21 complete files,
  one partial, 4,666 lines; 66 wholly unread text files and 877 lines of the
  partial remain. Four existing AERN2 runtime files (413 lines) are rereads,
  not additional coq-aern coverage or proof of dependency-version identity.
- Continuous max via overlapping selection/limit is different from choosing
  a winning operand. Coarse magnitude and rounding contracts cannot replace
  exact logarithm/floor predicates. Extraction erases substantial assumptions;
  no Coq proof check, regenerated extraction or donor native test is claimed.
  The explicitly broken fixed-precision extraction is not transferred.
- Existing Hyper APIs already express continuous extrema via exact abs.
  Debug/release match 49 records / 143,914 bytes with 96 independently correct
  enclosures. Ordering: 24 known unequal, eight Equal, 16 Unknown. Borrowed
  min/max keep self in the unresolved cases, as documented; not a correctness
  defect. Twelve corruption controls and 38,025 rational overlap-model pairs
  pass; the mathematical model is not donor execution.
- Memcheck returns identical numerical output, zero errors and no lost
  allocations; 3,008 reachable bytes / 25 blocks remain. Collector totals:
  39,929 allocations, 39,904 frees, 5,175,511 requested bytes—not isolated
  operation costs or a memory-improvement claim. Fmt, Clippy and the offline
  default-feature 21-package graph pass. No new full-stack, WASM or benchmark gate.
- Two zero-output sandbox captures are preserved as unusable, with successful
  approved checker/environment reruns. The initial inventory subprocess EPERM
  preceded its successful approved record. PATH/cache inspection finds no Coq/GHC;
  three benchmark symlink targets are absent. No large toolchain installed.
- Reused the shared Rust target; two audit binaries total 5,150,272 bytes, with
  13,877,915,648 bytes observed available on /tmp. No broad copy/cleanup, donor
  library rebuild, production edit, new retention, external report, commit or push.
  Seven previous continuation transfers remain retained. Full inventory open.
  [Findings](exactcore-hyper-comparison/audits/continuation/coq-aern/findings-v85.md),
  [read records](exactcore-hyper-comparison/audits/continuation/coq-aern/read-records-v85.json).
- Final sealed verification passes 2026-09-12T05:58:01.097Z: 62 directly bound
  artifacts, 4,498 prior artifacts revalidated, 135 coq-aern source artifacts,
  unchanged 957 live and 184 isolated-candidate files. Twelve earlier captures
  are classified (ten successful, two unusable empty sandbox captures); the
  final verification is a separate successful capture, one record / 635 bytes,
  empty stderr, code 0 / null signal. The first seal attempt's audit-only
  relative-path failure is recorded and corrected; it created no manifest.
  Scoped whitespace check passes; all checkpoint commands are terminal.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/coq-aern-verify-v85.json).
  The full original goal remains active and incomplete.

## Checkpoint 86 — coq-aern proof dependencies and precision planning

- Previous turn is progress: 21 files completed, one partially read, and an
  independently checked Hyper capability probe sealed. Revalidate those source
  identities and results before continuing. The ordinary sandbox retry fails
  on a denied subprocess; retain it and use the approved verification path.
- Continue the unread multivalue-monad implementation, scalar limit dependencies
  and formal square-root algorithm. Compare precision/error contracts with Hyper
  before proposing any code transfer. Native Coq/GHC qualification and benchmark
  claims require their own executable evidence, not just a mathematical model.
- Preserve checkpoint 85's immutable files; checkpoint 86 artifacts live in its
  own subdirectory. Reuse existing toolchains/build storage, with no broad copies
  or cleanup. Whole coq-aern, Calcium and original ecosystem scope remain open.

Checkpoint 86 qualification results:

- Read 4,454 new lines: eight newly complete files and the remaining 877 lines
  of MultivalueMonad.v. Coq-aern totals are 30 complete text files / 9,120 lines;
  58 text files and other artifact classes remain open. Complete extracted CSqrt
  reading includes all integer/vector helpers and erased-equality scaffolding.
- Sqrt.v admits csqrt_solutions at 628 and csqrt_small at 710. General complex
  sqrt uses both; these are source-level proof gaps, not reproduced wrong output.
  The real sqrt construction is separate, with its own assumptions/build limits.
- Completed monad/limit dependencies require coherent paths, closed predicates
  and unique/common limits. Alternating roots and positive values tending to zero
  show why those conditions cannot be dropped. Hyperlimit's geometric predicates
  and Hyperlattice's checked divisors retain their explicit uncertainty contracts.
- Ideal Newton scheduling admits one fewer iteration at selected power boundaries,
  but does not justify removing Hyper's rounded-arithmetic guards. Independent
  rational models pass 350 normalized-step and 17,920 strict requested-error
  checks, 65,589 planner indices, 1,161 signed-scale cases and near-zero controls.
  The finite corpus saves 315 ideal iterations, not measured runtime. Six negative
  controls and two semantic countermodels pass; no guard-bit transfer retained.
- The complex model validates 2,048 admissible formulas across 1,088 nonzero
  inputs, including 240 inputs with two legitimate results and five corruption
  controls. At -3-4i, branches can yield 1-2i or -1+2i; mixing components fails.
  This is not a principal-root contract, native Haskell run or proof-hole repair.
- Cached default Hyper library tests pass 23 in debug and 23 in release with the
  same names, including the directed-MPFR seed test and cache/abort checks. The
  initial exact-name filter ran zero tests and is explicitly unusable. The initial
  verification subprocess EPERM and one empty branch capture are preserved;
  approved reruns pass. No new full-stack, all-feature, WASM or benchmark result.
- No production edit, new retained transfer, new audit binary, broad source copy,
  donor rebuild, toolchain installation or cleanup. Observed /tmp available:
  13,877,891,072 bytes. All original references and unresolved experiments remain
  in scope. [Findings](exactcore-hyper-comparison/audits/continuation/coq-aern/v86/findings-v86.md),
  [read records](exactcore-hyper-comparison/audits/continuation/coq-aern/v86/read-records-v86.json).
- Final sealed verification passes 2026-09-12T06:25:36.241Z: 57 direct artifacts,
  the previous 62 direct / 4,498 older artifacts revalidated, and unchanged 957
  live / 184 isolated-candidate files. Eight prior captures are classified as
  five successes, one denied-subprocess failure, one unusable zero-test filter
  and one unusable empty capture. The final verification is a separate successful
  capture: one record / 658 bytes, empty stderr, code 0 / null signal. Scoped
  whitespace check and clean donor tracked-worktree check pass. All checkpoint
  commands are terminal; no external report, commit or push. Full goal active.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/coq-aern-verify-v86.json).

## Checkpoint 87 — metric contracts and native-toolchain reuse

- Previous turn is progress. Checkpoint 86 revalidation passes at
  2026-09-12T06:28:19.590Z after an ordinary sandbox subprocess EPERM; preserve
  both captures. No production changes or new retained candidates yet.
- Read the scalar metric and dimension-indexed vector dependencies in full,
  recording exact ranges separately from prior coverage. Investigate existing
  Haskell qualification tooling before downloading or installing anything.
  Old native-build records are not evidence that a compiler is still available.
- Keep artifacts in coq-aern/v87, leaving both earlier sealed directories intact.
  Reuse storage and build products; no broad copies or cleanup. The full original
  ecosystem inventory and all unresolved qualification requirements remain open.

Checkpoint 87 qualification results:

- Six files completed, 2,864 new unique lines: scalar metric, Euclidean vectors
  and all four textual benchmark harness files. Coq-aern totals 36 complete
  text files / 11,984 lines; 52 text files and other artifact classes remain open.
- The donor's vector metric is maximum coordinate distance. Hyperlattice's
  Euclidean norm cannot be replaced without changing the contract. Coordinatewise
  limits still require coherent paths, dimension consistency and closed predicates.
- Hand-written benchmark sqrt uses 2^-n where formal/extracted sqrt uses
  2^(-2n-1), permitting a zero approximation outside the promised bound. An
  independent rational model confirms 510 counterexamples and 16,320 valid
  formal-threshold controls. Not a reproduced native Haskell wrong answer.
- Enabled csqrt3rE/csqrt5rE runner names have no active dispatcher entries.
  Repeated parameters overwrite logs; failed logs are deleted; CPU 0.00 readings
  become 0.01. Only a static/syntax check ran; no donor benchmark was executed.
- Model also checks 35,937 scalar triples, 71,874 interval equivalences, 2,176
  convex cases, 512 vector cases and 8,448 fast-Cauchy pairs; eight corruption
  controls pass. Finite models are not a Coq proof check or Haskell execution.
- Offline/locked Hyperlattice vector and API-surface suites pass 27 tests in
  each debug/release profile, with matching names and no ignored/filtered tests.
  Shared-target recompilation is limited to Hyperreal/Hyperlattice and four test
  executables. No new whole-stack/all-feature/WASM, memory or benchmark result.
- The historical GHC path is absent; PATH lacks GHC/runghc/Coq/Rocq/opam/Cabal,
  while Stack is available. This checks known locations, not all possible disk
  locations. No download/install, source copy, broad cleanup or production edit.
  Observed /tmp available: 13,861,122,048 bytes after tests. All seven retained
  continuation transfers remain unchanged. Three denied-subprocess attempts and
  one empty model capture are preserved alongside approved successful reruns.
  [Findings](exactcore-hyper-comparison/audits/continuation/coq-aern/v87/findings-v87.md),
  [read records](exactcore-hyper-comparison/audits/continuation/coq-aern/v87/read-records-v87.json).
- The first seal rejects an audit-only reread range of 1-15 for the 14-line
  Hyperreal AGENTS.md. The corrected range is 1-14; no manifest was written by
  that failed attempt and no source/model/native result changed. The command,
  assertion and correction are preserved in the checkpoint harness-failure note.
- Corrected sealing succeeds 2026-09-12T06:45:22.865Z, binding 64 direct artifacts
  plus the earlier evidence chain. Four shared-target test executables total
  16,705,520 bytes, not a measured application-size change. No broad copy/cleanup.
- Final sealed verification passes 2026-09-12T06:46:31.490Z: 64 direct artifacts,
  previous 57 / 62 / 4,498 artifacts revalidated, unchanged 957 live and 184
  isolated-candidate files. Ten earlier captures classify as six successes,
  three environment failures and one unusable empty capture. Final verification
  is separate, code 0 / null signal. Scoped whitespace and clean donor checks pass.
  All checkpoint commands are terminal; no commit, push or external report.
  [Final capture](exactcore-hyper-comparison/audits/continuation/calcium/results/coq-aern-verify-v87.json).
  Previous/current turn classification: progress. Full goal active, not complete.
- Next open coq-aern work includes classical monads/Nabla, order/relator proofs,
  IVT, analytic and hyperspace algorithms, their remaining extractions and the
  binary/data artifacts. Native compiler/dependency acquisition and the earlier
  complex-root proof gaps remain pending, not a reason to stop source progress.

## Checkpoint 88 — classical choice, IVT and live-source drift

- Previous turn is progress: six source files and independent metric/benchmark
  contracts completed, with 27 native Hyperlattice tests in each profile.
- Fresh ordinary revalidation is denied a subprocess. The approved retry reaches
  a genuine live-source mismatch: hypercurve/src/bezier_region.rs changed after
  checkpoint 87. Preserve the edits and both failed captures; do not rewrite old
  manifests or claim that their tests qualify the edited curve implementation.
- Continue ClassicalMonads/Nabla and the IVT dependencies, while separating
  immutable historical evidence from current source identity. Investigate the
  remaining Haskell cache before deciding how much native setup is needed.
  Checkpoint 88 files live in a new sibling directory; the full scope stays open.

## Open requirements (unchanged full scope)

1. Finish source and supporting-reference reads, with exact ranges and snapshot hashes.
2. Compare candidate ideas against current Hyper implementation and its contracts.
3. Run native independent correctness checks, matched benchmarks, allocation and
   size measurements for plausible candidates; document unavailable environments.
4. Keep only justified improvements, with proportional regression gates.
5. Continue remaining ecosystem targets and unresolved historical transfer experiments.
6. Reconcile the full requested inventory before claiming completion.
