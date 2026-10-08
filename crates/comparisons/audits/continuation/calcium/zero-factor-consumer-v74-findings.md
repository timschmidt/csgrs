# Zero-factor final consumer and size qualification — checkpoint 74

Outcome: the isolated candidate passes the remaining consumer and representative
size gates. Its certified completeness gain and intended-path cost reductions
justify the observed bypass and size costs. Select it for exact live integration
and live-path regression checks; it is not yet a recorded retained change.
The six retained continuation transfers remain unchanged at this checkpoint.
The full ecosystem source/reference audit remains incomplete.

## Consumer identity and regressions

Only the already-qualified Hypercurve consumer is copied: 355 files / 23,841,619
original logical bytes in zero-factor-consumer-v74/hypercurve. That byte count
includes checkpoint 66's path-expanded manifest, unlike the earlier unmodified
consumer count. Exactly two Hypersolve paths now point to the zero-factor solver;
every other byte, including Cargo.lock, is unchanged. Scalar and triangulation
dependencies remain shared with the existing frozen source trees/cache.

Complete all-feature Cargo metadata matches after normalizing only consumer and
solver paths: 187 packages/nodes, exactly one instance of each of the six Hyper
crates, with unchanged features/dependencies/targets and lockfile. The current
956-file live map and 176-file isolated solver remain bound and unchanged.

All-feature release tests pass at 2026-09-11T21:45:04.866Z: 1,764 passed,
zero failed and nine pre-existing ignored, across 46 suites. All 1,773 individual
test names and outcomes match checkpoint 66, not merely aggregate totals.
Formatting passes; all-target/all-feature Clippy passes with warnings denied.
The release WASM library builds with triangulation, svg and hershey. This is
WASM compile qualification, not new geometry execution in a WASM runtime.

Both new default-feature basic/arrangement examples and both preserved baseline
examples execute successfully; complete outputs match. The seventeen nested
gates and qualification wrapper pass by 21:47:57.925Z. Independent evidence
checking passes at 21:48:29.000Z, code 0 / null signal with empty stderr.

The selected-range caller review is recorded separately. The public arithmetic
dispatcher can bypass the changed constructor through point/scalar/identity
routes. Independent represented roots and the rational-image quotient fallback
provide routes to it; Hypercurve retains its strict-first policy boundary.
The simple geometry examples do not establish direct divisor-deflation coverage.
That mathematical/branch qualification remains the earlier independently checked
binary-constructor, nonrational/history and native/WASM corpora. No new donor
lines or whole-file consumer-audit credit are claimed for this selected re-read.

## Representative sizes

| Stripped example | Retained baseline | Candidate | File delta |
| --- | ---: | ---: | ---: |
| basic | 11,014,424 bytes | 11,020,168 bytes | +5,744 bytes |
| arrangement | 11,588,728 bytes | 11,594,472 bytes | +5,744 bytes |

Ordinary executable deltas are +9,680 / +9,752 bytes. ELF text changes
+5,596 / +5,580, data +144 / +144, and BSS -1,648 / -1,648. These are linked
representative file/section observations, not isolated function sizes or a
runtime allocation/peak/RSS improvement. Rust, Cargo, strip and size versions
match the preserved baseline and remain unchanged during this campaign. Source,
features and command configuration are matched; differing build paths, linker
layout and dead-code selection remain confounders. No universal size claim.

The four dedicated new files total 50,310,904 bytes in
/tmp/calcium-zero-consumer.LrNEZu; all prior binaries are preserved. The existing
nonincremental two-job cache is reused, and its growth is separate. Recorded
post-build /tmp availability is 14,190,510,080 bytes, versus 15,003,926,528 at
checkpoint start. No cleanup, deletion, commit or push.

## Retention decision and remaining work

Select the isolated zero-factor candidate. It removes only an exact divisor
factor x^k when STRICT interval evidence excludes zero, after the existing
oversized square-free reduction. It preserves original source evidence,
signed primitive orientation for existing nonzero resultants, strict guard
semantics, downstream interval/polynomial/witness replay and the reduced degree
bound. It neither promises a total algebraic field nor replaces Hyper's scalar
representation or evaluation architecture.

Earlier qualification obtains 184 additional Transformed results among 16,692
paired queries while preserving the other 16,508 full records. Repeated policies,
scales and carriers are not independent bugs. Native and WASM independent
correctness, regression and memory gates are preserved, including the original
failed test-expectation/formatting captures. All 196 unchanged-result bounded-
deflation groups show native timing and allocation benefits; their WASM timing
benefit persists in both instance modes. Small native guard and WASM bypass
costs remain disclosed, and newly available answers can require more time,
requests and peak memory. The corrected statistical intervals are conditional
and not multiplicity-adjusted. No universal speed or memory improvement.

Under the requested priority, certified additional answers and those intended-
path savings justify the measured approximately 0.05% stripped-example growth
and bounded observed bypass costs. The main solver file adds 56 lines/removes
one (including documentation/test wiring); the separate seven-test module adds
260 lines. This is a modest code addition for completeness, not a code-size
reduction. No new dependency or public type field is introduced.

Next promote only the exact tested main-file diff and its new test module to
live Hypersolve, preserving pre-existing resultant/root-isolation edits. Bind
the new live source map and rerun proportional live regression/lint/platform
gates before recording a seventh retained transfer. Keep the independent power-
sum candidate separate. Remaining donor/kernel reads, formal/symbolic references,
historical transfers and full inventory reconciliation remain open.

Recheck this pre-adoption state with `node verify-zero-factor-consumer-v74.mjs`.
The independent verifier recomputes consumer metadata/test/size evidence and
checks current source identities; it does not rerun all earlier benchmarks.
