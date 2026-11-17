# Residual certification ownership and certainty review

At `4192d8c726`, all 446 lines and twenty declarations in
`crates/hypersolve/src/certification.rs` were reviewed with the ball filter,
interval-box, batch and failed-constraint consumers. Two lexical inventory rows
are replaced and eighteen missing declarations added, preserving the other
1,591 rows. The resulting 1,611-row workspace inventory remains partial.

Hypersolve owns this solver adapter and its reports under plan section 7.1,
P6, P8, P9 and P19. `CandidateResidualBall` selects an active-row ordinal and
radius; its exact residual center comes from replay. It is not a standalone
generic interval carrier. Scalar sign proofs remain in Hyperreal and ball
predicate policy/certainty in Hyperlimit. No arithmetic migration is proposed.

| Role | Required distinction |
| --- | --- |
| Exact residual replay | Preserve active source order, source identifiers, inequality normalization and evaluation failures. |
| Ball overlay | Preserve supplied selection order, original center, radius validation and raw predicate receipt. |
| Report summaries | Approximate decisions must not become exact satisfaction or violation claims. |
| Proposal-only reports | Preserve requested/used engine and precision, separately from approximate ball evidence. |
| Subsequent consumers | Interval-box and failed-constraint reports must retain the contributing uncertainty. |

The current ball adapter preserves `Certainty::Approximate` in its row payload,
but `is_certified_satisfied` and `is_certified_violation` ignore that field.
`is_unknown` omits approximate ball receipts. Recomputed summary counts can
therefore promote a qualified decision to a claimed certificate.

Eight public witnesses reproduce the problem. The exact angle-addition identity
`sin(e+1) - sin(e)cos(1) - cos(e)sin(1) = 0`, shifted by either sign of
`2^-2048`, supplies independently nonzero residuals for equality, soft and both
inequality kinds. Rational controls pass under both policies. Strict symbolic
witnesses remain unknown. Approximate witnesses retain a zero/approximate ball
receipt but all eight count as certified satisfaction; some of those rows are
mathematically violated. The affine interval-box facade reports `Certified`.
The quadratic facades remain `Unknown` in this corpus; their behavior is not
inferred from the affine path.

Four new regression tests also fail against the unchanged public baseline:
the full certainty/sign/satisfaction status matrix, residual summaries,
interval-box continuation and failed-constraint diagnostic continuation. The
isolated repair requires exact or filtered evidence for certified-summary
helpers, treats approximate ball receipts as unknown and transports them into
uncertain diagnostics. It retains the original public row payload and every
predicate call. Eight exact-control stdout records and all sixteen two-policy
public predicate traces match the baseline. This is dispatch evidence for the
probe corpus, not a full performance qualification.

The original adapter skips out-of-range selectors and rows without replayed
centers, and repeated selections overwrite earlier statuses in supplied order.
Those behaviors are recorded, not changed by this repair. The scalar precision
floor does not bound replay work, allocation, graph size, nested proof cost or
cleanup. Reports allocate vectors, names and diagnostic strings and retain exact
scalar data. This module has no serialization derive or native replay codec;
publicly constructed fields do not constitute independently validated proof.

The repair remains isolated and unaccepted. No memory/size allowance, runtime
claim, packaging qualification or phase completion follows from these checks.
The source and resource accounts in the primary tree remain unchanged.

Hyperpath feed, jerk, lookahead and rest-machining reports also retain candidate
certification reports. The inspected constructors use exact scalar replay; the
batch entry point does too. The ball-receipt defect is not thereby reproduced in
those paths. Their consumer checks remain part of the repair qualification.

Local candidate `aad7409a5928e309812d9a5cc361d46313685266` passed the four
regressions, 1,524 tests across 64 all-feature owner suites, 839 no-default
owner tests, strict all-feature/all-target Clippy, formatting, 444 Hyperpath
tests and 155 Hyperdynamics tests. These are correctness/build checks under
concurrent host load, not runtime measurements. Existing tests were unchanged.
The initial probe Clippy failure is retained. Initial Cargo cache warnings from
a full system disk are retained too; all 2,246 downloaded archives were verified
and relocated losslessly to the development SSD, restoring writable cache
space. No third-party archive payload is included in this evidence record.

Frozen review evidence: `docs/evidence/2026-11-17-interval-residual-certainty-review`, local commit
`ae36f8d84f038d5789b690e41b0615c5929b7ca5`; 138 files / 2,686,921 bytes, SHA-256
`a5ce2987cfdc4ae978cbe44cc1cbbc3c85b61e8105819a93f80dfd8a9ab8ef20`. Prior archive records and manifest rows are preserved.
No push.
