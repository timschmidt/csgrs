# Checkpoint 68 — power sums resumed on the retained witness baseline

The isolated power-sum constructor now passes the existing mathematical/public
corpora against the retained point-witness baseline. It is not retained or
performance-qualified. The full ecosystem audit remains incomplete.

## Source scope and the preserved first failure

Copied only the 175-file solver subset (6,094,998 original logical bytes),
sharing the existing frozen scalar dependencies. Added the exact 276-line
checkpoint-52 private power-sum module and its 24-line dispatch/fallback wrapper
to the already-qualified demand-gated witness repair. No live or donor file
changes; all 956 retained live identities still match checkpoint 67. The
original power-sum and point candidates remain unchanged.

Integer-scaled roots, Newton sums, binomial convolution and exact-divisibility
reconstruction replace repeated sampled resultants on the admitted path. The
unused-zero divisor-carrier fallback, signed primitive orientation, degree
admission, validation, interval construction and proof replay remain separate.
The baseline already contains the witness repair: none of the old 825 recovered
answers is a new power-sum completeness gain.

The first rebase passes both test profiles and the polynomial/public oracles,
but warnings-denied Clippy fails on five redundant references in the extracted
sampled fallback. Its exit-101 capture, exit-one outer run, initial source,
bindings, binaries and outputs are preserved. The correction removes exactly
those five references; its source checker reconstructs both parent versions
and verifies that no other implementation or test change occurred. Corrected
candidate tests and public/oracle runs are fresh, not inferred from the first run.

Initial gates run 2026-09-11T03:50:06.627Z–03:52:15.469Z; corrected gates run
16:54:37.074Z–16:56:49.433Z. The time gap is explicit. Build/test/collection elapsed
times are not matched benchmark observations. Supplemental gates finish at
16:58:46.885Z and the independent evidence check at 17:00:40.885Z.

## Mathematical and regression qualification

- Signed full-polynomial oracle: 4,840 cases and 1,210 independent polynomial-
  ring Sylvester determinants. All full candidate/baseline polynomials match,
  including multiplicities, 136 fallback cases and 32 zero resultants. This
  oracle uses neither Newton sums nor sampling/interpolation or a donor backend.
- Public STRICT and APPROXIMATE_512: 6,441 complete records per variant/policy;
  all agree with the retained demand baseline. Each policy passes 48,059 exact
  rational/Sturm checks with 1,178 independent resultants: 5,126 Transformed,
  241 denominator guards, 40 Undecided zero-resultant controls, 65 nonisolating
  controls and 968 unsupported-degree outcomes. The inherited rational checker
  labels its original scope STRICT; the same exact certificates here also check
  the separately built approximate-policy collector. These are repeated corpora,
  not independent defect or identity counts.
- Nonrational/state corpus: all 384 records match, across 48 operations, two
  policies and four ordered construction/refinement/serialization histories.
  Independent field/serialized-value checks pass 11,248 assertions per variant;
  316 Transformed, 24 denominator guards, six Undecided, six nonisolating and
  32 InvalidEvidence controls persist. Closed-interval rational checks alone
  are not used as proof of every half-open/nonrational case.
- Corrected all-feature debug/release tests pass 814 each in eight suites,
  zero failed/ignored. Exact names/outcomes match the initial rebase; the only
  additions to the retained baseline's 811 tests are the three power-sum tests.
- Warnings-denied all-target/all-feature solver Clippy, both harness Clippy
  runs and solver formatting pass. All-feature release WASM library compilation
  passes; no new WASM execution is claimed.
- Complete harness dependency metadata matches after only the two intended
  source-path normalizations: 33 packages/nodes, one of each relevant Hyper
  crate, identical lockfiles. No dependency upgrade is attributed as a benefit.

## Focused memory qualification

Four serial Memcheck runs preserve complete native outputs and report zero
errors and zero definite, indirect or possible lost bytes. Reachable memory
remains nonzero:

| Corpus | Baseline reachable bytes / blocks | Candidate reachable bytes / blocks | Baseline cumulative requested bytes | Candidate cumulative requested bytes |
| --- | ---: | ---: | ---: | ---: |
| Approximate public | 1,840,544 / 15,318 | 1,364,672 / 11,607 | 232,232,789 | 175,485,125 |
| Extended state | 48,648 / 353 | 48,296 / 351 | 130,905,028 | 130,256,844 |

Public allocation calls are 2,909,745 versus 2,707,146; extended calls are
727,683 versus 725,143. These process totals include setup, collection and output,
not isolated per-query allocation, peak/RSS measurements or a proof of bounded
retention. The observed reductions are promising, not a universal memory claim.

## Resources, evidence and next action

Twelve dedicated files, including the four preserved initial candidate binaries,
total 34,012,184 bytes under /tmp/calcium-power-rebased.xUuvlb. No second source
tree was copied for the lint correction; only its initial changed file was
archived in the workspace. The existing two-job nonincremental Cargo cache was
reused. Recorded /tmp availability after corrected gates is 15,304,589,312 bytes;
cache growth is additional to dedicated executable totals. Nothing was deleted,
committed or pushed. Unstripped driver sizes are not representative application
sizes or a retention argument.

The independent checker verifies 42 captured gates: forty successful, plus the
two preserved failed initial captures. Its own successful capture is separate.
It rechecks complete source maps, metadata, test inventories, full public/state
records, mathematical oracles, binary identities and memory summaries. Historical
52's failed mathematical gate and 67's live source evidence remain intact; this
is not a fresh full historical-chain execution.

Current isolated verification, from this directory:

```sh
node verify-power-rebased-v68.mjs
```

Next qualify broader coefficient heights and carrier shapes, then run matched
native/WASM CPU and separate per-query allocation campaigns with retained/fresh
and bypass controls. Consumer and representative-size gates, and updating the
candidate's inherited sample-only public documentation, remain required before
any retention. No timing campaign has run yet. Six retained continuation changes
and donor coverage remain unchanged: 1,415 complete files, twenty partial and
183,653 uniquely counted read lines. Remaining donor/reference reads and full
inventory reconciliation stay in scope.
