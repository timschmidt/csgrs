# Solver domain-certification ownership and certainty review

On `425c28eeb`, the domain preflight adapter can publish a policy-authorized
approximation as an exact domain certificate. The review reproduces this through
the public API for all ten domain-check kinds, including two incorrect domain
classifications. No repair is retained by this record.

All 532 lines and 15 declarations in
`crates/hypersolve/src/domain_certification.rs` were reviewed. Hypersolve owns
the authored-expression traversal, mathematical-domain conditions and diagnostic
reports. Hyperlimit owns the underlying comparisons and their certainty.
The two helpers named `classify_*_unit_interval` compare a point-valued `Real`
against scalar bounds; they do not implement interval-set arithmetic for
extraction into Hyperinterval. Computational adapters remain category 4 and
report/status types remain category 5 under plan section 7.1.

The inventory updates two previously lexical entries and adds thirteen missing
declarations, preserving every other row. Its 1,578 rows remain a partial
workspace inventory. Public report fields and status counters are mutable;
this API has no serialization or independent native-replay validator. The
review does not claim that a supplied report independently certifies its source.

## Reproduced missing acceptance witnesses

The independent identity
`z = sin(e + 1) - sin(e) cos(1) - cos(e) sin(1) = 0` supplies a retained
terminal-policy case already used elsewhere in the repository. On this revision,
the direct Hyperlimit comparison is `Unknown` under `STRICT` and a decided
equality with `Certainty::Approximate` under `APPROXIMATE_512`.

The domain adapter calls `.value()` and discards that certainty. At the relevant
boundary, all ten check kinds then return `CertifiedValid` or `CertifiedInvalid`
under the approximate policy. The report counts the result as certified, and
its aggregate certification queries consume that count. The strict policy
keeps these same witnesses unknown.

| Independent mathematical input | Approximate domain report | Required distinction |
| --- | --- | --- |
| Denominator `z + 2^-2048`, which is positive | `CertifiedInvalid` | A terminal approximation is not a proof that this denominator is zero. |
| Square-root operand `z - 2^-2048`, which is negative | `CertifiedValid` | A terminal approximation is not a proof that this operand is nonnegative. |

The expanded probe covers division, negative and zero powers, square root,
both logarithms, asin, acos, acosh and atanh, plus the two signed-offset cases.
Twelve exact rational boundary/offset controls pass under both policies.
Twelve acceptance invocations fail because an approximate answer is counted
as a proof. These are deliberately failing witnesses, not passing conformance
tests. A separate initial square-root probe reproduces the same failure.

Both external harnesses build against the current repository, pass strict
Clippy and formatting, and retain full dispatch snapshots. They run on native
x86_64 with Rust 1.95 and the `dispatch-trace` feature. The initial harness used
the nonexistent `EvaluationContext::new`; that compile failure is preserved,
followed by the correction to the existing `Default` constructor. No owner
implementation changed during reproduction.

## Required repair boundary

P8, P9, P19 and P30 require the qualified predicate answer to remain distinct
from an exact certificate. A future repair must preserve certified exact and
filtered decisions, explicit uncertainty and evaluation failures, and retain
the policy-authorized approximate answer with its receipt. Disabling the
approximate policy, guessing an exact answer, or merely weakening the witness
would not repair that contract.

Preserve the existing expression traversal and comparison order: left child
before right child, checks after their operand traversal, and the lower unit
bound before the upper bound. Hyperlimit already supplies certainty-preserving
logical conjunction for predicate outcomes; Hypersolve has shared certainty
support in `residual_replay`. Reuse the relevant owning-layer machinery rather
than adding another numerical decision kernel. The shared solver test helper
`terminal_zero` retains the hard symbolic identity for maintained regressions.

The `certify_candidate_domains` owner benchmark and `domain_certification`
dispatch fixture are mapped, but were not timed. Existing smoke, property and
representation tests cover rational or certifiably positive controls; this
review does not claim a fresh whole-owner run. Allocation, stack, binary size,
total-work bounds, cancellation and downstream release qualification were not
measured. Recursive traversal and repeated operand evaluation remain visible
resource obligations, not limits established by the predicate precision policy.

The frozen source, public probes, original failures, trace records and review
checks are registered in `docs/evidence-archive.tsv` under
`docs/evidence/2026-11-17-interval-domain-certainty-review`. All 57 size accounts
and zero adopted memory accounts remain unchanged. All fifteen phases remain
active; this record neither admits a repair nor establishes full closure.
