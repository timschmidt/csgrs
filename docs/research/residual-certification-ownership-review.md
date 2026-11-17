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

## Finite memory qualification

The isolated repair now has an initial resource qualification over 113 cases
and 196 policy configurations: direct residual reports, interval-box consumers
and failed-constraint diagnostics. Fixed `c9f1f9ac`, current `4192d8c726` and
candidate `aad7409a59` use identical fixtures, locks and build paths. Corrected
approximate outcomes are measured separately from unchanged completed contracts.

The operation campaign completed 77,760 processes across 2,352 accounts; 113
accounts exceeded the prospective caps. Complete input/report/cleanup lifetimes
completed 9,720 processes and 311,040 cycles across 1,176 accounts; 68 accounts
exceeded caps. Full reports and dispatch traces match on unchanged contracts.
Neither campaign's failures are erased by later observations.

Fresh repetitions of those exact binaries add 11,136 operation/stack processes
and 6,528 lifetime processes, including another 208,896 cycles. They reproduce
within-binary allocation and watermark variation. All original positive charges
remain, together with additional baseline drift and unchanged denominators.
Overlapping observations do not establish zero candidate cost. Static-address
trace hashing followed by B-tree insertion is a possible contributor to
temporary allocation variation, not a proven attribution.

A separate, unadopted proposal raises only two warm, reused-input, no-tracing
STRICT stack configurations from 64 to 128 bytes AND 2%: the unknown negative
inequality residual and unknown equality diagnostic. Their cumulative reserved
charges are 80 and 104 bytes, respectively, against 11,704-byte baselines. The
32 KiB total and 1 MiB observed worker reserve remain unchanged. Every other
configuration retains its original cap; temporary allocation failures remain
open. No broader percentage allowance is proposed.

Across both lifetime campaigns, no positive persistent payload increment occurs
after cycle two, and retained-output/persistent-growth caps pass. The monitors
measure requested and glibc usable payload, excluding allocator arenas and RSS.
Stack watermarks cover the callback below actual RSP, including the callee red
zone, with two fill patterns and a guarded 2 MiB pthread stack. These finite
observations do not establish universal stack or cache bounds.

Both instrumented consumer families pass their original final-binary caps and
capacity checks. Required uninstrumented public/WASM, owning-crate, complete
consumer capacity, runtime, packaging, cancellation/retry and shared-composition
qualification remain open. The initial harness compile failure and analyzer
output-path error are retained with their corrected checks. No source or
resource account is admitted, and no runtime claim comes from these runs.

Frozen finite resource evidence: `docs/evidence/2026-11-17-interval-residual-certainty-resources`, local commit
`d62cded53e3df97f5c77ada6a040df93cda848c0`; 13 archive files / 134,998,864 bytes, SHA-256
`5e6547a66f1263debda2a5f5a794ccaef9aec58034c1de61b570926a80de2dc6`. The verified split archive contains 109,301 files /
2,875,171,645 uncompressed bytes, including all fourteen instrumented binaries.
The live admission stage remains mutable; later work is not sealed by this
snapshot. Prior archive records and manifest rows are unchanged. No push.
