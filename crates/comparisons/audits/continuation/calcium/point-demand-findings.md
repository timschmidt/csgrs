# Checkpoint 57 — demand-gated recovery: native benefit with retry costs

The isolated revision preserves the eager repair's complete outputs and removes
its measured allocation/peak penalties in all 512 unchanged-result groups. The
expensive Unknown-endpoint controls become roughly 25% faster than the eager
version. This is not an unconditional win: retries add allocations to recovered
point cases, some paired timings are slower, and no version is retained here.
Cold-first-query semantics, applicable WASM execution/costs, final consumers and
representative sizes remain qualification requirements.

## Change and proof obligations

`point-demand-candidate` copies only the 175-file solver slice (6,091,527 original
bytes), reusing unchanged retained dependency snapshots. Its Cargo manifest has
mechanical path changes. The only algorithm/test edit is `algebraic_binary.rs`:
1,515 lines / 53,846 bytes versus the eager file's 1,412 / 50,465. This is 32 net
algorithm lines and 71 net test lines beyond the eager candidate, not beyond the
retained baseline. No public API, scalar representation, resultant algorithm,
degree cap or public-refiner contract changes.

Interval construction again carries no witness. The existing refiner runs
first; only its typed InvalidInterval result triggers attempted recovery. No
diagnostic text is matched. Recovery must strictly prove singleton endpoints.
Approximate-policy multiplication/division also reconstructs and certifies the
whole image under STRICT. On success, the rejected report is dropped, a witness
is attached, and the unchanged refiner replays containment, polynomial vanishing
and uniqueness. This is a bounded retry, not recursive refinement or approximate
equality promoted to exactness.

Private-helper tests were strengthened so the new always-empty construction
field cannot make old negative tests vacuous. They check unresolved endpoint
equality, approximate extrema, symmetric products and half-open ownership. A
point without a witness still fails the public refiner; after recovery, a
witness that fails the polynomial is still rejected.

## Qualification

- Build finishes 2026-09-10T22:30:55.684Z. Library-only debug/release tests pass
  447 each. Separate full all-feature runs pass 811 tests each across eight
  suites, with zero failed/ignored. App Clippy and all-target/all-feature solver
  Clippy pass with warnings denied. This is not new Hypercurve consumer testing.
- STRICT and APPROXIMATE_512 public collectors each emit 6,441 complete records,
  byte-identical to the eager repair. The unchanged rational oracle passes
  48,059 checks per policy, including disclosed candidate self-pairs; a separate
  baseline comparison establishes the same 825 gains and 5,616 unchanged records.
  Repeated policies are not independent corpora or additional defect counts.
- The 384-query extended collector also matches the eager result byte-for-byte.
  Its independent serialized-value interpreter passes 11,248 checks. The separate
  768-group history/lifecycle qualification passes 1,536 CPU/allocation
  observations and 44,992 exact checks, ending 22:35:15.923Z. Final actual full
  results match, not merely statuses or checksums.
- Extended and large approximate-public Memcheck outputs equal native exactly;
  both report zero errors and zero definite/indirect/possible losses. Extended
  reachability is 48,648 bytes / 353 blocks, with 727,683 allocations, 727,330 frees
  and 130,905,028 cumulative requested bytes. The large collector has 1,840,544
  reachable bytes / 15,318 blocks, 2,909,745 allocations, 2,894,427 frees and
  232,232,789 cumulative bytes. These are nonzero-live whole-collector observations,
  not marginal query costs, RSS or general lifetime bounds. Qualification ends
  22:34:23.356Z; all failed historical gates remain preserved.

## Matched three-variant campaign

Baseline, eager and demand binaries are measured together, not compared by ratios
of separate campaigns. Each of 768 groups includes all 48 cases, both policies,
four histories and retained/fresh input lifecycles. Three 16-iteration pilots
choose a common count targeting 4 ms at the slowest rate, clamped to 4–4,096.
Each of twelve CPU blocks is one permutation followed by its reverse; all six
permutations occur twice. Three rotated allocation blocks measure eight queries
per variant separately. All observations are retained; no outlier is deleted.

There are 55,296 CPU observations and 2,304 pilots, 22:35:52.520–22:44:48.336Z;
then 6,912 allocation observations, 22:44:48.437–22:45:35.341Z. Offline verification
passes at 22:46:23.662Z, replaying independent history checks, complete output
comparisons, actual order, calibration, timing ranges and all statistics.

All children make nine preconditioning calls. Fresh rebuilds and drops input
graphs during the batch but does not mean cold shared constants. The final result
remains alive at the counter snapshot; its drop, serialization and full checking
are outside timing. Intermediate iterations have a polynomial-length checksum
only. Core 6 affinity is used on a Ryzen 7 5800X3D (SMT sibling 14), powersave
governor, with before/after environment snapshots. No own build/regression/
Memcheck runs overlap CPU measurement; fixed frequency or an idle host is not
guaranteed. Individual 95% bootstrap intervals are not multiplicity-adjusted.
Paired median ratios need not equal ratios of the overall sample medians.

## Results

Against the retained baseline, all 512 same-result groups now have identical
allocation requests, requested bytes, return-live and peak deltas. This removes
the eager version's 188 request/byte increases and 16 peak increases in the same
corpus. Paired CPU ratios nevertheless range 0.881863–1.168626: eight individual
intervals lie below one and 134 above. No blanket timing equivalence or speedup
is established. The maximum ratio's interval crosses one; a slower retained
case 29 / STRICT / round-trip control has ratio 1.147938 [1.087594, 1.228912], with
overall sample medians 4.065 versus 4.326 microseconds per query.

Against the eager repair, all 768 full results match. Paired ratios range
0.754745–1.122420; 119 intervals lie below one and 184 above. Retained case 37 /
approximate / constructed history improves from an eager median 23.931 to
18.937 microseconds, paired ratio 0.754745 [0.743588, 0.808935]; its retained
baseline median is 19.091 microseconds. Related Unknown-width controls improve
similarly. These are workload-specific improvements, not a whole-library speedup.

Versus eager, requests/bytes decrease in 188 groups, stay equal in 324, and rise
in the 256 recovered-point groups. Eight-query deltas range −456 to +40 requests
and −46,848 to +2,760 requested bytes. The largest positive increment is five
requests / 345 bytes per query. Return-live is equal in all 768; peak is lower in
16, equal in 722 and higher in 30, ranging −176 to +201 bytes. Peak/net-live
deltas must not be divided by the iteration count as if they were request totals.

Against baseline, the 256 new-answer groups do different certified work; ratios
0.983638–3.530992 are not equal-work speedups/regressions. Request/byte totals are
equal in 80 and higher in 176, with eight-query maxima +1,608 requests / +82,624
bytes. Return-live rises 143–1,663 bytes with a successful result alive. Peak is
equal in 150 and higher in 106, at most 1,104 bytes.

## Decision and remaining work

Keep the candidate isolated. Demand gating solves the measured nonpoint
allocation problem, but retrying through a rejected refinement report is not
free and the remaining runtime/size tradeoffs are explicit. No production or
donor edit or sixth retained transfer is claimed. A lower-overhead integration
may be worth testing, but no such implementation exists at this checkpoint.

Next verify fresh-process first queries without preconditioning. The current
ordered collectors and warmed benchmark children do not prove every cold
cache-enabled decision is unchanged. Then qualify any chosen final revision on
the relevant extended WASM corpus/costs, consumers and representative binaries,
before deciding retention. Separate power-sum performance work, remaining donor
and supporting reads, formal/algebraic references and full inventory
reconciliation remain in scope.

## Resources and reproducibility

Five frozen executables occupy 14,950,736 bytes in
`/tmp/calcium-point-demand.BLEmYy`; the four baseline/eager benchmark binaries and
shared build cache are reused. Demand CPU/allocation harnesses grow 2,232/2,136
bytes over eager, including build-path/layout effects. The large collectors use
the app's serde-enabled dependency graph; their size difference from earlier
non-serde collectors is not an algorithm-only measurement.

History/CPU/pilot/allocation raw files total 230,193,025 workspace bytes, excluding
public/Memcheck outputs and summaries. Capacity capture records 16,393,678,848
available `/tmp` bytes. Dedicated sizes exclude changes within the reused cache.
No cleanup/deletion, commit, push or external report occurred.

`point-demand-manifest.json` binds this note and the source, execution and cost
evidence. `verify-point-demand.mjs` rechecks all 956 unchanged live sources and
175 candidate files, importing 56 through 48 and preserving earlier failures.
It does not rerun the full historical 47-chain, whose latest complete capture is
still 2026-09-10T04:56:34.285Z. Donor coverage remains 1,415 complete files, 20 partial
files and 183,653 uniquely read lines for Calcium/FLINT, not ecosystem completion.
