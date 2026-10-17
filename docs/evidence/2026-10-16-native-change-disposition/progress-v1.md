# Disposition of the 35 pending native edits

**Rejected as a complete integration.** The 29 tracked files are restored to commit `99b88f707b04053de7f40a86d6d029e3c07460d9`; the six added source/test files are removed from production after exact preservation. All 35 candidate files remain under [pending-source-v1](pending-source-v1/), with [tracked](preparation-v1/tracked.patch) and [untracked](preparation-v1/untracked.patch) patches. No native edit remains indefinitely pending in the worktree.

This implements the user’s request for a retain-or-reject decision under [the declared acceptance contract](contract-v1.md). The [current source review](source-review-v1.md) identifies real accuracy, evidence and failure-handling improvements. The eight new curve regressions and all 26 new DRC regressions pass. Those benefits do not make the complete integration satisfy its unchanged workload contract.

| Original gate | Candidate result | Committed alternative | Unchanged criterion |
| --- | --- | --- | --- |
| full-board | 193.401087205 seconds; failed | passed | < 120 test-worker thread CPU seconds |
| copper-neck | 86.070668709 seconds; failed | passed | < 60 check-body wall seconds |
| gerber | passed | passed | original smoke assertions |

The test source, policies, complete archives and geometry resolution are unchanged. Four independent unzip extractions verify both complete board entries in both snapshots. Gates run without other builds from this task. Candidate failures and complete logs are retained in [candidate validation](candidate-validation-v1/commands.json). Whole command times are separate from the asserted worker-CPU/check-body metrics. The representations do different work, so these observations establish no speedup or performance-only optimization.

The restored configuration passed the [native matrix](committed-validation-v1/commands.json), including root and curve feature combinations, DRC tests and all three original gates, all-target Clippy with warnings denied, docs with warnings denied and formatting. The [real WASM matrix](wasm-validation-v1/commands.json) passed all twelve new STL/error tests, both earlier JavaScript consumers, the prior TypeScript check, the compiled-and-executed new TypeScript consumer, and WASM Clippy. Main-tree formatting passed after byte-identical restoration.

| Native check | Passed | Failed | Existing ignored |
| --- | ---: | ---: | ---: |
| root-all | 429 | 0 | 1 |
| root-minimal | 132 | 0 | 0 |
| curve-default | 2431 | 0 | 28 |
| curve-all | 2468 | 0 | 33 |
| native-other-tests | 1614 | 0 | 0 |

Counts overlap across feature configurations and are not a coverage percentage.

The complete unchanged proposed DRC regression file was also [replayed against the committed libraries](committed-proposed-regressions-v1/result.json): **4 passed, 22 failed, 0 ignored**. These failures are preserved as unmet mathematical claims, not hidden by the legacy suite. Rejection is not a correctness repair and does not certify the legacy handling of exact track tokens, round caps, extreme finite strokes or construction failures.

A failed test stops at its first assertion. Several budget/policy-named tests fail first on construction or area, so their later budget/policy assertions are not independently established as failures.

Failed proposed regressions:

- `ambient_approximate_policy_does_not_change_stroke_geometry`
- `coincident_track_endpoints_keep_the_width_disk`
- `denied_stroke_construction_retains_failure_after_scope_exit`
- `denied_track_import_cannot_disappear_or_become_a_valid_origin`
- `exact_diagonal_track_area_includes_caps_without_rounding_the_width`
- `finite_endpoint_difference_can_exceed_float_range`
- `finite_length_squaring_cannot_overflow`
- `finite_length_squaring_cannot_underflow_to_zero`
- `fresh_zero_budget_admits_rational_butt_strokes`
- `irrational_diagonal_retains_native_surd_width`
- `irrational_diagonal_track_keeps_line_and_arc_geometry`
- `kicad_linear_tracks_include_both_round_caps`
- `large_finite_stroke_does_not_collapse_its_width`
- `malformed_track_fields_retain_source_failure`
- `pythagorean_diagonal_has_exact_rational_area`
- `subnormal_width_is_halved_after_lifting`
- `tangent_track_caps_do_not_create_filled_overlap`
- `track_caps_drive_actual_short_readiness`
- `track_decimal_endpoints_and_width_are_never_rounded_first`
- `track_subrange_width_and_length_remain_nonzero`
- `track_superrange_coordinates_stay_native`
- `warmed_stroke_queries_recheck_exhausted_scope`

The next useful work is smaller independently qualified repairs using these preserved regressions and the original full-scale gates. No individual optimization is assigned the integration’s costs without a controlled comparison. The complete candidate, original hard fixtures and all adverse results remain available.

Only the raw `current_work` ledger value changes; bytes and decoded keys outside it stay bound to revision `9025de62`. The historical artifact audit remains failed/incomplete, retaining 947,270 earlier expectations and 27,093 missing historical paths. New physical paths are hashed independently; no whole-history audit or zero-loss claim is made. Full distribution, pre-push, internal resource-bound, coverage and other properties work remains open. The full goal continues.

See [the disposition decision](decision-v1.json), [exact restoration](application-v1.json), [retained logs](collection-v1.json) and [checkpoint](checkpoint-v1.json).
