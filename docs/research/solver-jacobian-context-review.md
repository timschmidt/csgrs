# Solver Jacobian context review

The required phase-7 review of `hypersolve/src/jacobian.rs` found a public
forward-difference bug at `557a349f9`. For `x²`, model value `1`, context value
`3`, and step `1`, the implementation returns `-5` instead of `7`: it evaluates
the baseline at the context but perturbs the model value. A weighted case with
fixed, unused and inactive roles also fails. Missing used bindings still return
the existing typed evaluation error.

All six original declarations were read, along with `EvaluationContext`,
residual evaluation and the immediate solver caller. They belong to the solver's
numerical proposal boundary, category 4 in plan section 7.1. None stores an
interval or supplies general interval arithmetic to extract. The ordinary solver
reconstructs its context from its current model each iteration; that synchronized
path is an unchanged control. The returned `f64` matrix is not a certificate.

Three isolated repairs remain unaccepted:

| Candidate | Change | Whole-call stack result against original |
| --- | --- | --- |
| `9e88db0ca` | Read the context binding before perturbing | +32 bytes in all 16 groups, with and without dispatch instrumentation |
| `89b58eb57` | Shift through one context-map entry | −80 bytes in 14 groups; +128 bytes in both empty-row groups |
| `4bd27bdb9` | Return the complete owned, perturbed context | −128 bytes in 14 groups; +176 bytes in both empty-row groups |

The eight-case corpus includes synchronized quadratic and linear inputs, a
mixed two-variable polynomial, weighted/fixed/unused/inactive roles, an empty
row set, and both inequality hinges. Four campaigns run 1,536 fresh processes,
including two stack-fill patterns and four repetitions. All 768 paired matrix
outputs match independent expected binary values; 256 allocation and output-drop
pairs match. The first candidate also preserves 64 full dispatch snapshots.
These are finite-corpus observations, not universal bounds or latency evidence.
The stack regressions prevent retention; binary-size permission does not waive
that gate. No timing campaign or size-account change follows from this review.

At `9e88db0ca`, the full all-feature owner library/integration suite, doctests,
strict docs, Clippy and formatting pass on Rust 1.95. Stable-toolchain focused
tests and all-target strict Clippy pass. The later ownership variants pass four
focused tests, including generation, source-context preservation and the
original independent regressions; they were stopped at the resource screen
before repeating full qualification. Tests assert scalar operation counts and
zero Hyperlimit decisions. The probe's initial dead-code warning is preserved;
an explicit statistics accessor resolves it without suppressing the warning.

The original source is restored clean. Three original Jacobian tests, strict
all-feature/all-target Clippy and formatting pass. Both restored probe binaries
are byte-identical to their baseline artifacts and pass their functional checks.
No production repair or new test is retained.
Six reviewed dispositions are added to the partial inventory. Zero/underflowed
step validation, missing numeric-estimate handling and signed symbolic
inequality conventions remain separate review questions. Neither this audit nor
the preserved failing fixtures establishes broader Jacobian correctness.

Evidence: `docs/evidence/2026-11-17-interval-jacobian-context`, local commit
`a099a4fd21fd8d106acfb874bb943dedc628794d`; 1,792 files / 1,280,121 bytes,
SHA-256 `33911caecf0a7cd0d30a440b434f9bbff6331f88c514a7a711b71a4dbaeb42ce`.
Commands, source patches, locks, failed witnesses and all raw observations are
retained. Earlier archive paths and manifest rows are unchanged. No push.
