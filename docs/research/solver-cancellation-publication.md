# Cancellation before result publication

The isolated candidate `9457d39b8b03e58ebabeb7871549b362371f5f7b` extends
Hypersolve's cancellable work scope with one final exhaustion check after its
callback returns. It is based on the unadmitted five-repair composition
`4a04dd9744bfecb578963b42fa3bffef5b7062b5`. Functional qualification passes;
complete-consumer cost and remaining admission checks are still required.

Hypersolve owns this P5/P6/P20/P36 boundary and plan item 14.4. The old documented
contract allowed a callback to cancel its token and still return `Ok(value)`
when no later instrumented check observed cancellation. A public native witness
reproduces that behavior using the verified extracted five-repair package. Five
of eight added owner tests fail on the original implementation; three controls
pass. The original assertions remain byte-identical prefixes of both test files.

The candidate calls the existing `exhaustion` helper immediately after the
callback. Ordinary `with_budget` and executable Hypercurve adapter code are
unchanged. Cancellation observed there prevents the callback value from being
published. The migration documents this intentional behavior extension; it is
not a behavior-preserving refactor. The callback still executes with a token
cancelled before entry. Earlier resource failures retain precedence. Nested
usage, parent/child isolation, cross-thread requests, unwind cleanup and fresh
token retries have explicit acceptance tests. Arc ownership checks verify that
an unpublished output is dropped.

This remains cooperative cancellation. It does not preempt uninstrumented work,
undo callback side effects or impose a total-work, latency or heap bound. A
request arriving after the final check may race with successful publication.

## Executed qualification

The initial campaign uses Rust 1.95, locked offline builds, six jobs, a total
1,800-second deadline, 600-second build/owner command limits and 20-second native
witness limits. The WASM campaign completes about 1,548 seconds after the
original start. All listed candidate suites have zero failures and zero ignored
tests; 1,321 is the number of test invocations, including repeated feature runs.

| Check | Result |
| --- | --- |
| Hypersolve release library, no default features | 642 passed |
| Hypersolve release library, all features | 645 passed |
| Hypersolve ordered-field resultant integrations | 16 + 10 passed |
| Hypercurve work-budget integration | 7 passed |
| Existing Hypercurve cancellation heap-release test | 1 passed |
| Public native cancellation witness | Baseline fails; candidate passes |
| WASM uncancelled / callback-cancelled / pre-cancelled witnesses | Baseline 1 success + 2 expected traps; candidate 3 successes |
| Hypersolve strict Clippy, all targets and features | Passed |
| Hypercurve strict Clippy, both affected test targets, all features | Passed |
| Owner and direct-consumer strict documentation | Passed |
| Affected formatting, whitespace and spelling | Passed |

The unchanged heap-release test retains its original assertions. Its successful
harness captures numerical diagnostics, so this record makes no exact heap-byte
claim. WASM has no imports and each case uses a fresh instance. Its Rust witness
checks exact cancellation fields, zero usage, callback invocation, scope cleanup
and a fresh-token retry. Baseline traps reproduce assertion failures; they are
not typed resource exhaustion. These are diagnostic witnesses, not a production
WASM ABI or complete consumer size qualification.

Independent verification checks command/log hashes, exact counts, preserved
original assertions, the five-path patch, the one executable owner change,
unchanged adapter code, identical WASM witness source and all four witness image
hashes. A spelling-tool invocation initially failed because the executable was
absent from PATH; the explicit installed path passed. Both logs are preserved.
The isolated candidate is committed and clean. This is not whole-workspace
validation or an admission decision.

## Remaining admission

The [complete-consumer cost campaign](solver-cancellation-consumer-cost.md)
now supplies functional and lifetime evidence but fails initial stack and some
active-trace growth caps. Those failures remain pending.

Complete-consumer output and dispatch comparisons, complete-lifetime memory and
stack evidence, final-image accounting, quiet performance, successive operations,
distribution and applicable downstream validation remain. The prior five-repair
memory failures and all positive charges remain intact. The bounded-memory
exception permits a justified allowance; it does not waive these measurements
or another gate. Accepted totals remain 57 size accounts and zero memory
exceptions. The fifteen-phase goal stays active; no push.

## Frozen evidence

Record `docs/evidence/2026-11-17-interval-cancellation-boundary`, evidence commit
`f36a509a3e69917406dab41ebc9c1c087119ed8f`: 79 files / 571,103 bytes,
SHA-256 `963fbacbf2f8aec16d500627514e2990071e4b1886639b17743d599cebdb33c2`.
The record preserves original failures, raw commands/logs, protocols, verifiers,
source patch and four losslessly compressed witness images. Source/stored hashes,
lengths and modes are recorded. Earlier archive paths and index rows are unchanged.
