The proposed stored-bound shortcut has **zero coverage at the measured native
call site**. All **11,075,136** exact-sign quadratic fallbacks entered with
`BoundCache::Invalid`. Every observation completed; no stored positive,
negative, zero, unsigned-nonzero or anomalous bound was observed there.
The [decision](decision-v1.json) closes this shortcut for the measured boards.
No bypass prototype or runtime change is admitted.

| Original fallback result | Count |
| --- | ---: |
| Unknown | 842,331 |
| Negative | 3,637,633 |
| Zero | 1,665,238 |
| Positive | 4,929,934 |

The diagnostic reads only existing packed root-bound metadata immediately
before the original call, then counts its already-computed result. It adds no
scalar comparisons, owner clones, cache writes, arithmetic or recognition
passes. Removing the declared insertions recovers the parent runtime bytes
exactly. Normalization, result publication, memo lifetime, node budgets and the
mandatory original retry remain unchanged.

[Qualification and observation](assessment-v2.json) verify 14 commands. Three
byte-identical reference tests pass in separate parent and instrumented copies.
Three counter calibrations and five site controls pass, including valid bound
signs, zero cancellation, the original multiradical decline, cached-sign
shortcuts, mismatches and unfinished observations. The 885-test scalar library,
28 doctests, 11 P4 regressions, six cache-frontier tests and five endpoint
regressions pass; focused counts overlap the library. Workspace formatting,
strict all-feature/all-target Clippy for four crates, and warning-denied scalar
docs also pass. This is not the repository's complete pre-push workflow matrix.

One fixed test execution covers both original complete boards. The original
assertion fails at **143.517755723 worker-CPU seconds / 120**; both reports finish
with 12 and 197 findings. Instrumentation changes execution cost, so this is
not a matched speedup or slowdown and does not replace the parent's release
failure. The separate 60-second copper-neck check-body gate was not rerun.
Finding counts do not establish complete report-byte identity.

V1 preparation stopped before compilation because one shared-test assertion
needed wrapping under the crate's rustfmt configuration. V2 changes only that
test formatting. The failed source trees, runners and
[preparation failure](preparation-failure-v1.json) remain hash-checked. The
generated diff's final blank context line also triggered the commit whitespace
check; [independent reconstruction](commit-whitespace-review-v1.json) proves
the retained patch is byte-identical to the frozen source difference.

The result does not claim that bound certificates are absent elsewhere or that
computing new bounds would be free. It rules out adding this particular stored
bound check to skip the observed normalizations. The multiplication-helper and
storage experiments remain rejected.

The next review concerns allocation costs at the DRC process boundary. The
existing unchanged-parent profile has a declared allocator-leaf subset at
29.580% of sampled cycle weight; that is not predicted savings. Pinned mimalloc
and wrapper sources are already present locally, but their integration, target
support, ownership and applicable allocation paths still need review. No
allocator prototype or dependency change is prepared. Any trial must preserve
the mathematical source and proof behavior, qualify whole-application timing
and memory, replay full reports, and pass both original native gates before
admission.

All 35 pending native paths are preserved. All 39 properties, 14 targets and
crate future work remain in scope; the plan is active and incomplete.
