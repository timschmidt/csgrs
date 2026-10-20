# Changelog

## [0.3.0] - Unreleased

### Breaking

- `Candidate::replay_policy` and the `ReplayPolicy` struct are replaced by
  `Candidate::proposal_seed: u64`. The former `require_exact_replay` flag was
  never consulted: archive admission always requires an accepted replay on the
  candidate's `FitnessReport`.
- `select_exact_best` and `select_tournament_by_indices` return the new
  `SelectionError::CandidateReportIdMismatch` when a fitness report does not
  name the candidate at the same index.
- `classify_simulated_annealing_neighbor` returns `InvalidSchedule` for a
  cooling ratio above one, and the new `AnnealingAcceptance::UnknownSchedule`
  when the temperature or cooling-ratio checks cannot be certified.
- `SelectionError` and `AnnealingAcceptance` are exhaustive, so `match`
  expressions over them must handle the new variants.
- `FitnessValue::compare_total` returns `FitnessComparison::Unknown` for
  lexicographic vectors of different lengths instead of ranking the shorter
  vector as worse.
- `eval_gp_batch` looks up sparse inputs by key. An index missing from the
  input map now yields `GpValidationIssue::MissingInput` instead of evaluating
  as zero.

### Added

- `dispatch-trace` feature, which forwards the exact-dispatch instrumentation
  of `hyperreal` and `hyperlimit`.

### Changed

- Fitness, interval-endpoint, and annealing-schedule comparisons use
  `hyperlimit`'s strict predicate policy (`PredicatePolicy::STRICT`) instead
  of `Real`'s `PartialOrd`, and interval equality is certified rather than
  structural. Comparisons the policy cannot certify are still reported as
  unknown. `hyperlimit` is a new dependency.
- First-improvement hill climbing builds neighbors lazily and stops at the
  first accepted improvement. GP validation computes depth, node count, and
  issues in one traversal. `PERFORMANCE.md` records local benchmark medians
  before and after these changes, for example 54.216 µs to 3.562 µs for a
  32-gene first-improvement step.
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.2.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.

### Removed

- `surrogate_allows_archive_promotion`, which always returned `false`. Use the
  `SurrogateDecision` in `SurrogateScreenReport` and send candidates to exact
  replay instead.
- The empty placeholder `hyperevolution` binary target.

For 0.2.0 and earlier, see [crates.io](https://crates.io/crates/hyperevolution/versions).
