# Checkpoint 79 — cheaper exact proof, still not retained

The isolated revision preserves all prior exact answers and certificates while
substantially reducing the expensive tan/cot proof arithmetic. It is still not
selected for production retention: repeated unequal tan/cot comparisons remain
about four times the live baseline, with higher allocation demand. Failed-proof
overhead also remains. Seven previously retained continuation transfers are
unchanged; no live or donor source was edited.

## Change and correctness

An exclusive 183-file / 5,345,646-byte copy preserves the checkpoint 77 candidate.
The revision uses direct exact Q(sqrt(3)) tangent/cotangent forms, avoids unused
sine/cosine construction, skips exact-zero multiplication/scale coefficients and
avoids multiplication by one. The proof remains bounded, does not request
approximations or rewrite numeric nodes, and still rejects exact poles before
cancellation. Its helper grows from 307 to 340 lines; this is not a code-size win.
The new test-only reference module is 498 lines, including the frozen evaluator.

Three differential tests supplement the unchanged independent mathematical
oracles: 1,024 sparse field pairs, 160 coefficient-boundary cases, 485 tangent and
cotangent arguments, 8,760 expression cases, and 670 unsupported/deep cases.
They check admission as well as values, including negative residues, poles,
large periods, zero products, inverse/square operations and work bounds.
The frozen reference differs only by verified formatting of one match arm.

Matched library/integration suites pass in debug and release:

| Features | Live baseline | Revised candidate |
| --- | ---: | ---: |
| Default | 756 | 766 |
| All | 859 | 870 |

Every previous baseline/candidate test name and outcome is preserved. Clippy,
formatting and WASM compilation pass; full 126-package/node graphs match after
path normalization. The unchanged public probe produces 1,728 rows per profile,
byte-identical to checkpoint 77: all 32 new identities remain proved, without a
pole or unequal-control regression. This is not a full CI or universal closure
claim.

Memcheck's public release output is identical to native output: zero errors or
lost bytes, 41,200 allocations, 41,052 frees, 4,092,804 requested bytes, with
19,000 bytes / 148 blocks still reachable. This is a whole-process diagnostic,
not isolated per-query memory or RSS evidence.

## Native costs

The unchanged checkpoint 78 harness and 96-case / 576-group corpus compare
baseline, prior candidate and revision. Baseline/prior rebuilt products match
their frozen executables byte-for-byte. All three full benchmark dependency
graphs (21 packages/nodes) and locks match; six CPU/allocation binaries pass
3,456 complete preflight records. CPU binaries lack counting-allocator symbols.

Collection finishes 2026-09-12T00:53:29.959Z: 3,456 pilot, 41,472 CPU and 20,736
allocation rows, totaling 65,664. Twenty-four CPU triples balance all six variant
orders four times; six allocation triples balance them once, at one and sixteen
iterations. All plan membership, order, calibration, timestamps and complete
certificates are rechecked. All revised certificates equal the prior candidate's.
Twelve existing corrupted-stream controls and corrected-statistics self-tests
pass again. Details: [protocol](twelfth-revision-cost-protocol-v79.md),
[final recomputation](twelfth-revision-native-final-summary-v79.json).

Selected retained-pair queries at precision -64, from this same campaign:

| Case | Baseline / prior / revised median time (us) | Revised/baseline paired ratio | Revised/prior paired ratio |
| --- | ---: | ---: | ---: |
| Unequal tan(pi/12) | 0.866 / 8.064 / 3.371 | 3.883 | 0.416 |
| Unequal cot(5pi/12) | 0.865 / 8.109 / 3.414 | 3.963 | 0.422 |
| Unsupported seventh-angle identity | 1.462 / 1.638 / 1.652 | 1.130 | 1.011 |
| Unsupported 64-term identity | 3.382 / 4.017 / 4.108 | 1.201 | 1.008 |

Paired ratios are medians of per-triple ratios, not ratios of the marginal
medians. Both interval methods retain the large tan/cot benefit versus prior and
penalty versus baseline. Failed-proof comparisons versus prior are inconclusive;
their penalty versus baseline persists. No instruction-level attribution or new
case-isolated replication is claimed.

The selected unequal tan/cot queries use 6 / 41 / 26 allocation requests,
520 / 4,744 / 2,608 requested bytes, and 408 / 1,752 / 928 peak bytes above starting
live demand, respectively. Live delta is zero. All six allocation rounds agree
at both one and sixteen iterations. The revision is cheaper than prior, not
cheaper than the live baseline. Small supported/periodic controls also include
slower measurements; no universal no-regression claim is made.

For the 32 target identities, revised/baseline ratios span 0.071–0.504 with
construction included, 0.036–0.374 on first queries of preconstructed pairs, and
0.766–2.279 on repeated retained pairs. Forty-eight of 64 repeated groups are
slower under both intervals. These compare new Equal answers against Unknown,
not equal-work speedups. Perturbed comparisons retain NotEqual with changed
certificates and span 0.380–3.965. Same mathematical answer does not imply an
identical proof certificate.

## Qualifications, corrections and storage

All 103 capped groups and 92 short median batches in the baseline comparison
are retained. Confidence intervals are conditional on sampling assumptions and
not multiplicity-adjusted. CPU affinity and host snapshots do not establish a
reserved or fixed-frequency CPU. Fresh means construction in a warmed process;
cold setup/destruction remain outside timing. Allocation peak is not RSS.

The first summary classified identical certificate text as an identical answer,
placing unchanged NotEqual decisions under changed answers. The final summary
separates decision equality and certificate equality. Original summary, checker,
samples, ratios and intervals remain intact; no measurement was altered.

Preserved failures: test-only nonexistent sqrt2/sqrt3 constructors; a source
comparison that initially disallowed rustfmt's match-arm wrapping; four
status-zero/empty Node captures; and an uncaptured failed parse of empty evidence.
Approved confirmation checks source maps and reuses individual successful gates
without repeating exclusive source preparation or binding. No mathematical
misdecision was observed. Original failing source/checker versions are retained.

The shared build cache is reused. Two new frozen benchmark binaries total
4,403,848 bytes; no broad stack snapshot or cleanup. Source maps remain 957 live
files and 184 revised-candidate files. No representative linked-size, downstream
consumer or WASM runtime retention qualification is claimed.

Current verifier: `node verify-twelfth-revision-v79.mjs`. Keep this revision as
isolated evidence, not a retained production change. Proof scheduling/bypass
costs remain open, alongside remaining donor reads, transfer experiments and
full inventory reconciliation. No new donor read credit: continuation coverage
remains 1,447 complete files, 20 partial files and 185,617 uniquely read lines.

Combined evidence passes 2026-09-12T01:03:24.740Z: one record / 58,616 bytes,
empty stderr, code 0 / null signal. The [manifest](twelfth-revision-v79-manifest.json)
binds 3,661 artifacts and 149 captures: 140 current successes, three development
successes, two failed and four unusable empty captures. Recorded /tmp available:
13,929,951,232 bytes. Both tracked donor worktrees remain clean.

[Final sealed verification](results/twelfth-revision-verify-v79.json) passes
2026-09-12T01:10:04.036Z: one record / 1,293 bytes, empty stderr, code 0 / null
signal. All commands are terminal; the live and candidate source maps are unchanged.
