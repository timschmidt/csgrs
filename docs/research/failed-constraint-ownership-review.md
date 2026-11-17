# Failed-constraint diagnostic ownership

At `21aa18c88`, all 826 lines and 33 declarations in
`crates/hypersolve/src/failed_constraints.rs` were reviewed. The module had no
inventory rows; all 33 are now mapped, preserving the previous 1,611 rows.
The resulting 1,644-row inventory remains partial. This closes a mapping gap,
not the outstanding implementation or resource gaps.

Hypersolve owns these category-5 reports, certificates and provenance under
plan section 7.1 and P6/P8/P9/P19/P20. There is no independent interval endpoint
carrier or generic interval arithmetic to extract. The module consumes exact
point-valued residual certification and affine-rank evidence; it retains source
constraint identities, diagnostic distinctions and the contributing reports.

| Role | Contract retained in Hypersolve |
| --- | --- |
| Status and summary queries | Separate contradiction, candidate violation, redundancy, lossy proposal, domain failure and unresolved evidence. Only redundancy is nonblocking. |
| Existing-report consumer | Compute affine rank, classify failure rows, then consider rank redundancy only for `CertifiedZero` rows; sort by source index. |
| Redundancy probe | Require an active affine equality/soft row, clone the problem, deactivate that source row and compare certified rank reports. |
| Single/pair removal | Replay diagnostics after each deterministic source-row removal; keep whether current blocking rows remain. |
| Bounded set removal | Enumerate increasing-index combinations through the requested cardinality, capped by the blocking-row count. Zero cardinality still produces the original diagnostics. |
| Minimal removal | Search cardinalities in order; retain the first clearing cardinality's probes, or all attempted probes if none clears within the bound. |

Clearing the current candidate's blocking diagnostics is not a global
satisfiability proof or a globally minimal unsatisfiable core. Removal searches
rerun scalar certification from the problem/context with explicit precision
inputs; they do not consume an existing residual-ball report. The supplied-report
entry point does consume such reports, which is where the separately reproduced
approximate-certainty defect matters.

The isolated residual-certainty repair changes only that failure-classification
guard in this module. The lossy-proposal arm remains before the broader unknown
check, preserving its distinct diagnostic status. Original predicate/rank order
is unchanged. See the [certainty review](residual-certification-ownership-review.md)
for the failing public witnesses, candidate qualification and open admission
gates. This inventory change does not adopt the repair.

Reports allocate vectors and cloned names/evidence. Searches clone and reanalyze
problems, enumerate combinations and retain probe lists. Cardinality and scalar
precision bounds are not total-work, allocation, retained-memory, stack or
cancellation budgets. Minimal search can retain lower-cardinality probes while
constructing the next set. No new resource bound is inferred from this audit.
There is no serde derive or exact replay codec here; public report fields are
not independently validated replay proof.

Public reexports, the sketch diagnostic adapter, six maintained smoke tests,
the generated residual-property cases, and certification/dispatch/retained-fuzz
benchmarks are mapped. Prior isolated-candidate owner logs retain their passing
results; this review does not rerun those tests or infer a new timing result.
The native/WASM direct-diagnostic consumer pilot is additional finite evidence,
not qualification of every removal search on every target.

Declaration coverage, source hashes, unchanged unrelated inventory rows and
accepted ledgers, spelling, whitespace, archive identity and repository-budget
checks qualify this documentation change. No production source, public API,
predicate sequence or resource account changes. All plan phases remain open.

## Native and WebAssembly report consumers

The residual-certainty consumer campaign completed fourteen final artifacts
and 16,096 checks across fixed `c9f1f9ac`, current `4192d8c726` and candidate
`aad7409a59`. The fixture and operation/validation sources are copied unchanged
from the earlier resource campaign. All 113 cases and 196 policy configurations
are exercised with reused/rebuilt inputs, cold/eight-call warmup and
absent/passive/active tracing. Added corrected outcomes run only on the candidate;
an incorrect baseline certificate is not treated as completed functionality.

There are 13,744 full report/trace comparisons, including native/WASM equality
and equality across revisions on unchanged contracts. Candidate-only corrected
cases are compared across their candidate artifacts. Traces are nonempty in
5,792 active rows; empty cases remain in the corpus. Independent fixture
assertions validate reports and source identities before producing output.
This is full textual comparison of constructed reports, not an exact replay
codec or a general proof of cross-platform equivalence.

| Final consumer | Candidate file bytes | Cumulative positive file/code charge |
| --- | ---: | ---: |
| Native, tracing absent | 2,749,040 | 0 |
| Native, tracing compiled | 2,905,864 | 0 |
| WASM, tracing absent | 1,727,867 | 310 bytes |
| WASM, tracing compiled | 1,885,704 | 310 bytes |

All original prospective image caps and reserves pass, preserving separate
fixed/current/candidate accounts. Native allocated-section, executable and
named-code accounts have no positive charge. WASM file and code-section growth
both retain their 310-byte charges. These are measured fixture images, not new
accepted accounts for a shared application binary.

WASM modules have zero imports. The largest observed linear-memory size is
1,441,792 bytes after report materialization and explicit output-buffer cleanup,
within the declared 256 MiB capacity/64 MiB reserve. Linear-memory pages do not
measure live allocator payload or imply pages return to the host. Native memory
cap failures from the earlier campaign remain open.

Strict Clippy passes for both feature sets on both targets, and formatting
passes. The isolated worktree is restored to the candidate branch. The original
quiet queue and its finite watcher are resumed unchanged after the build hold;
these under-load builds and controls provide no runtime evidence. Owner artifacts,
public package replay, complete consumer capacity, cancellation/retry, runtime
and combination with the four earlier repairs remain required. No source or
resource account is admitted.

Frozen evidence: `docs/evidence/2026-11-17-interval-failed-constraint-ownership`, local commit
`8bc1f0256c0d8fe919cd7c0fd182ac94341145ac`; 139 files / 55,660,343 bytes, SHA-256
`ef3852c2547fec0b9019c3f552951d36f88d9819ff6b8244ee49eea74e7b8568`. All prior archive paths and manifest rows are
preserved. The ownership review is frozen; the separate live residual admission
stage remains mutable beyond this finite snapshot. No push.
