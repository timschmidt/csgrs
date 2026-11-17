# Five-repair solver composition qualification

The isolated candidate `4a04dd9744bfecb578963b42fa3bffef5b7062b5` combines the four pending solver
repairs with the residual-certainty repair. It passes a new composition witness
and the affected functional qualification below. It remains unaccepted:
combined resource, runtime and distribution evidence is still required.

## Contract and reproduced failure

Hypersolve owns the P6/P8/P9/P19 certification summaries and diagnostics. This
extends the [residual-certification review](residual-certification-ownership-review.md).
The four-repair baseline is `f65589b44e1a010de75f8129b371ecbf503cc052`;
the added repair is `aad7409a5928e309812d9a5cc361d46313685266`.

On that baseline, all four unchanged residual regressions and all eight public
residual witnesses reproduce their expected failures. The additive composition
witness also fails at its residual-uncertainty assertion. This reproduces the
defect on the combined target revision before changing production code.

The existing composition test already checks context-centered finite
differences, finite linear-adapter failures, a lossy numerical proposal, exact
alpha qualification and domain certainty. Its extension feeds the same exact
context into residual-ball certification and then consumes that report in
failed-constraint diagnostics. The policy sequence is STRICT, APPROXIMATE_512,
STRICT again with warmed caches. A near-root claim and an approximate zero
residual must not become an exact root or exact violation. The diagnostic must
retain the original certainty-bearing row, and context bindings/generation
must remain unchanged.

Every existing assertion and input is preserved. The original repair's two
production files and residual test file are byte-identical to the standalone
candidate. The only additional source change is 50 lines in the existing
composition test. Other four-repair source files remain unchanged. The isolated
branch is `interval-solver-five-repair`; neither earlier candidate nor the
primary source is rewritten.

## Executed qualification

Rust 1.95.0 locked offline builds use a dedicated tmpfs target and six build
jobs. Each command has its recorded bound under one total elapsed deadline.
The initial Rustfmt rejection required only wrapping an added assertion; its
source, log and failed result remain preserved. The retry keeps the original
total deadline and every assertion.

| Candidate check | Passed tests | Suites |
| --- | ---: | ---: |
| Owner all-feature library/integration tests | 1,538 | 64 |
| Owner no-default library, smoke and residual tests | 853 | 3 |
| Original four-repair public witnesses | 36 | 1 |
| Owner doctests | 1 | 1 |
| Hyperpath all-feature library/integration tests | 444 | 4 |
| Hyperdynamics all-feature library/integration tests | 155 | 19 |

No tests were ignored in these runs. Both owner feature configurations pass
all-target Clippy with `-D warnings`; both external public harnesses also pass
strict Clippy. Strict owner documentation, final workspace formatting and
whitespace checks pass. There are no compiler warnings in the recorded commands.
These are affected checks, not the complete repository workflow matrix.

All 16 residual public cases pass across 32 policy rows. The eight exact-control
full outputs match the fresh four-repair baseline. Every residual predicate
trace matches, including the witnesses whose summary claims intentionally
change. All eleven owner dispatch workloads preserve full report and stdout
bytes. Existing count assertions are unchanged. The independent verifier checks
raw outcomes, all four observer image hashes, source equivalence, copied public
fixture/lock bytes, assertion preservation and the original total deadline.

## Admission remains open

This completes the specified functional composition check, not resource or
release admission. The earlier individual and four-repair measurements do not
establish final five-repair costs. Preserve all original failures, positive
charges and fixed program baselines. Final combined image, memory/lifetime,
capacity/headroom, quiet runtime, successive-operation, package and applicable
validation evidence remain required before retention.

The original quiet watcher resumes with its finite horizon and frozen command
plan. The two recorded DRC failures and 36 queued commands remain intact.
Primary production source, the partial 1,644-row inventory and accepted accounts are unchanged:
57 size accounts and zero memory exceptions. The full fifteen-phase plan remains
active.

Frozen evidence: `docs/evidence/2026-11-17-interval-five-repair-composition`, local commit
`db6870e1c3109c4e540d084e3f34039dc412158f`; 122 files / 7,822,613 bytes, SHA-256
`ca2012ee7e29ee64f569a2bf38e82cf71e917ca45c07dad2d5daef74329598e6`. Original observer executables, command outputs and patches
are preserved with lossless gzip encoding and source/stored digests. Earlier
archive paths and manifest rows remain unchanged. The qualification and initial
formatting-attempt stages are frozen. No push.
