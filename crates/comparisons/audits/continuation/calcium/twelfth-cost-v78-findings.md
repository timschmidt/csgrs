# Checkpoint 78 — native costs: do not retain this version

The twelfth-turn proof remains mathematically useful, but its repeated-query
overhead is too large to select the current implementation for retention.
The strongest same-result regression is a warmed unequal cotangent comparison:
0.874 to 8.029 microseconds, paired median ratio 9.188. Case-isolated replication
also finds a large slowdown (paired ratio 8.613). Allocation requests rise from
6 to 41 per query, requested bytes from 520 to 4,744, and peak above starting
live bytes from 408 to 1,752. Live delta is zero. These counts reproduce in
case-isolated runs at both 1 and 16 iterations.

No production change was made. Keep the original candidate and evidence intact,
but reduce its proof-arithmetic and failed-proof costs before reconsidering it.
Do not obtain a speedup by abandoning the new exact answers, changing compact
numeric nodes or treating Unknown as a decision.

## Scope and protocol

96 authored cases, two precision budgets and three lifecycles give 576 groups.
The matrix covers all 32 target identities, 32 unequal perturbations, supported
angles, periodic equivalents, rational/radical/exponential controls, unsupported
trig identities, deep expressions and two large coefficient scales. The four
CPU/allocation binaries pass all 2,304 preflight records. Their full 21-package
dependency graphs and locks match after intended path normalization.

The main campaign records 2,304 pilot, 27,648 CPU and 9,216 allocation rows:
39,168 total, including all samples. CPU measurement uses 24 balanced randomized
pairs, CPU 2 affinity and the same group order within each pair. Allocation
uses four pairs at both 1 and 16 iterations. Instrumentation symbols are absent
from the CPU binaries. Statistical reanalysis uses the corrected rejection-sampled
bootstrap and a separate median order-statistic interval; its self-tests pass.
Twelve corrupted records/streams are rejected, with one valid control accepted.

Post-selection replication runs six cases in separate processes: 192 processes,
864 CPU and 576 allocation rows. These 1,440 additional records reproduce the
large hot-path penalty without preceding unrelated cases in the same process.
They are not unconditioned confirmatory significance claims.

See [protocol](twelfth-cost-protocol-v78.md),
[main recomputation](twelfth-native-summary-v78.json),
[isolated recomputation](twelfth-isolated-summary-v78.json), and
[combined evidence](results/twelfth-cost-evidence-v78.stdout).

## Benefits and costs

For the original 32 identities, the candidate returns Equal where the baseline
returns Unknown. The elapsed ratios below therefore compare different answers,
not equal-work speedups; each row contains 64 groups across both precisions.

| Lifecycle | Candidate / baseline elapsed ratio | Groups slower under both intervals |
| --- | ---: | ---: |
| Construction included, warmed process | 0.110–0.503 | 0 / 64 |
| First predicate on preconstructed pairs | 0.075–0.389 | 0 / 64 |
| Repeated predicate on warmed retained pairs | 0.760–5.353 | 48 / 64 |

The 192 same-result perturbed comparisons span 0.586–9.188. Their allocation
requests and requested bytes increase in every group at both measured iteration
counts. First-query benefits do not erase the repeated-query tax. The selected
tan/cot cases reproduce paired ratios 8.712 and 8.613 in isolated processes.

Failed-proof controls also regress. In isolated retained -64 queries, the
seventh-angle unit-circle identity is 1.165x baseline and the 64-term variant
1.258x, with unchanged allocation counts. This supports a CPU-work cost beyond
allocation alone; it does not establish a precise instruction-level cause.

Supported-angle and periodic controls have identical measured allocation counts;
most timing intervals are inconclusive. Their short rows are not evidence of a
universal no-regression result. In total, 115 groups hit the 2,000-iteration cap;
92 groups have median timed batches below 100 microseconds. The cap bounds the
preconstructed-pair memory footprint. Every short row is retained and identified.

The 1,000-bit scaled target gains a decision in six groups; the 1,030-bit variant
remains outside this bounded proof's admitted work in six groups. These are
representation/scale controls, not six or twelve additional independent identities.

## Limits, failures and storage

Fresh means construction inside the measured loop in a warmed process, not
process startup. Cold-pair construction and final destruction are outside timing.
The result histogram is inside timing. Allocation peak is above starting live
bytes, not RSS. Host snapshots do not prove an idle, reserved or fixed-frequency
CPU. Confidence intervals are conditional and not multiplicity-adjusted.

Initial harness-only unused-import warnings were fixed. One patch context
mismatch applied no edit. An uncaptured initial build stopped on sandbox `nm`
EPERM after copying the first binary; approved continuation verified/reused it.
The continuation then failed a bookkeeping assumption of 20 packages rather
than 21 including the benchmark root. All mathematical preflight records had
passed. Correct finalization compares both rebuilt products byte-for-byte with
the existing snapshots. Failed/development captures and source versions remain.

The four frozen binaries total 8,750,456 bytes in one narrow /tmp directory.
Existing build cache reused; no new Hyperreal tree copy or cleanup. Live 957-file
and candidate 183-file maps remain unchanged. Combined evidence passes
2026-09-12T00:06:59.557Z. No donor, production or external report edit, commit,
push or new retained transfer. Seven continuation transfers remain retained.

Final sealed verification passes 2026-09-12T00:13:42.729Z: 2,899 bound artifacts
and 277 captures, comprising 274 current successes, two development successes
and one preserved failure. Output is one record / 1,407 bytes, empty stderr,
code 0 / null signal. All campaign commands are terminal; both tracked donor
worktrees remain clean. Recorded /tmp available is 13,970,505,728 bytes.
See [final capture](results/twelfth-cost-verify-v78.json) and
[manifest](twelfth-cost-v78-manifest.json).

No WASM runtime, downstream consumer or representative linked-size qualification
is claimed for this candidate. Revise and retest its cost while preserving its
proof domain; then complete those gates if retention is justified. Continue the
remaining references and full inventory reconciliation. Donor coverage is
unchanged: 1,447 complete, 20 partial, 185,617 uniquely read lines.

Current verifier: `node verify-twelfth-cost-v78.mjs`. The earlier 75/76/77 verifiers
remain valid because their recorded source states are unchanged.
