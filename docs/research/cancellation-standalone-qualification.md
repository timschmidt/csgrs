# Standalone cancellation publication qualification

The publication check can be evaluated independently of the unadmitted solver
repairs. Candidate `8a454c86054ae4d5a4b0813d8e9edd1e6775ec47`, based on primary
`e63cbf68aea24a5939332c738c4a6652be881627`, passes the affected functional and
platform checks below. It remains isolated: 17 resource accounts exceed the
existing growth allowances after historical charges are retained, and runtime,
distribution and remaining admission work are open. No numerical cap or accepted
exception count changes in this record.

This follows the [combined consumer qualification](solver-cancellation-qualification.md)
under Hypersolve work-budget ownership, P5/P6/P20/P36 and plan 14.4. The four
affected owner/adapter source and test files on primary were byte-identical to
the earlier five-repair source before its publication fix. Only the scope-local
implementation, unchanged eight owner tests, unchanged adapter regression and
a standalone migration note are imported. No solver migration or trace collector
optimization is included.

## Contract and reproduced witnesses

Previously documented behavior allowed a callback to cancel its token and return
`Ok(value)` if it reached no later instrumented check. The candidate intentionally
extends that contract: cancellable scopes call the existing exhaustion check after
the callback returns and before removing the scope or publishing its value.
Ordinary scopes use the private false const parameter. Hypercurve's executable
adapter code is unchanged; its documentation and regression describe the extended
owner behavior.

The callback still runs for a token cancelled before entry. Earlier failure reasons
retain precedence; nested scopes, cross-thread requests, cleanup, unwind restoration
and fresh-token retry are covered. Cancellation remains cooperative. The check
neither preempts arbitrary callback work nor rolls back side effects or establishes
a time/heap/work bound. A request after the final check can race with success.

On current primary, the unchanged native public witness prints `Ok(7)` and fails
its publication assertion. The added owner tests reproduce five failures and three
passing controls; the adapter test fails as expected. The isolated candidate passes
1,301 affected test invocations: 632 no-default and 635 all-feature owner library
tests, 16 and 10 ordered-field integrations, and eight downstream budget/release
tests. Three strict Clippy configurations, strict owner/adapter docs, owner example
and bench compilation, and formatting pass. These are affected checks, not complete
workspace or benchmark execution.

The unchanged WASM witness uses a fresh no-import instance for each case. Primary
passes the uncancelled case and traps at the two expected cancellation assertions;
the candidate passes all three cases, including exact diagnostics, callback
invocation, scope cleanup and fresh retry. Both target Clippy configurations and
formatting pass. This witness does not qualify a production WASM ABI or its memory.

## Complete consumer and preserved feature failure

The first consumer build exposes an inherited feature connection: `cancel-retry`
also enables `added`, compiling three fixtures that require unmerged solver APIs.
That failed build, 12 images and 12 ordinary smoke controls remain preserved.

A reviewed standalone configuration removes only that feature dependency and widens
the two existing `Fixture.case` declaration/construction selectors to either feature.
All assertions and operation code remain intact. Cases 0–3 cover 1/4/16 variables
and separate 64-variable stress. Cases 4–6 and their original seven-case bundle
retain their source and outstanding obligations; this configuration does not claim
to complete them. Its cancellation/retry output also omits that bundle's unrelated
report fields, so their complete-workload costs are not interchangeable.

The corrected collection preserves the original absolute deadline and aggregate
child ceiling. Thirty final native images cover fixed program baseline
`c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`, current primary and the candidate,
with default/trace configurations and separate added cancellation/retry controls.
Constant launch names are shared by consumer kind. All original exact expectations,
both policies, reused/rebuilt input, cold/warm states, operation/destruction stack
probes, both stack fill patterns and three heap repetitions remain.

There are 3,294 observations: 3,288 successes and six expected primary publication
failures. All 2,232 full output/trace comparisons and 2,760 output comparisons pass.
The 592 fresh, 32-cycle histories cover 18,944 cycles, including setup, retained
state, cleanup and teardown. Four consumer Clippy configurations and formatting
pass. Failed primary attempts are not successful cost baselines for added retry.

## Resource and historical accounts

Twelve of 288 observed accounts exceed an incremental or cumulative allowance.
Historical reconciliation raises that count to 17; all use active tracing. Absolute
capacities, required headrooms, stack allowances and retention after cycle two pass.
The twelve final-image accounts have no failed growth or headroom checks; added
retry uses absolute capacity rather than an invented successful-old denominator.

Maximum observed requested/usable peaks are 577,406 / 613,136 bytes. Persistent
footprints reach 12,696 / 12,888 bytes; measured operation and cleanup stack reach
18,768 / 1,768 bytes. The guarded mapping, excluded caller/runtime frames and
measurement resolution remain defined by the consumer protocol. These are finite
single-worker payload measurements, not RSS, firmware or arbitrary-input bounds.

A source review confirms that only test additions separate the fixed baseline,
historical primary `ee4932b4cf8482e6d4d4672833395fd937e8047f`, and current primary
in this consumer's four kernel dependencies and root build configuration. The
production trace prefix is identical. All 1,644 corresponding historical/current
full output and trace comparisons pass.

The reconciliation retains 1,632 historical metric floors and 254 positive
historical observations for that accepted source path, with their denominators
and failed obligations. New positive charges remain too. For example, a preserved
1,264-byte peak charge still fails even where the latest candidate peak is lower.
Neither source identity nor a later smaller measurement erases that obligation.
The earlier ten-boundary solver/wrapper bundle keeps its separate complete ledger;
none of its failures or charges is declared resolved by taking this distinct path.

The next admission work is to assess justified per-target bounded allowances for
this useful functionality, together with runtime, owning benchmarks, distribution
and remaining validation. The optional [trace refactor](trace-borrowed-runtime.md)
is not a prerequisite. No allocation mechanism is inferred for every failed account,
and the memory exception cannot waive performance or other acceptance gates.

## Execution and disposition

The original functional run finishes in about 827 seconds with 47 aggregate direct
children, including twelve source preparations, within 3,600 seconds / 160 children.
Cargo reports a full system disk while updating cache metadata. The first strict
audit preserves that warning as a failure. Five identical affected checks then pass
without warnings using an isolated RAM Cargo metadata home and unchanged registry
sources. The revised audit requires those clean checks and preserves the original
warning; no test is rerun or assertion weakened by this environmental correction.

WASM uses 19 children / about 113 seconds. The two consumer attempts together use
3,608 children / about 1,157 seconds, within the original 4,500 / 5,400 limits.
The historical review uses eight read-only Git children and about 17 seconds.
Independent audits pass within their 300-second bounds. No runtime improvement is
claimed from these instrumented campaigns.

All producers are terminal and the source worktree is restored to its original
control branch. The workflow queue remains held and its original watcher remains
terminal. Primary production code is unchanged. Accepted totals remain 57 size
accounts and zero memory exceptions; inventory remains 1,644 rows. No push; the
full fifteen-phase goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-standalone`,
archive commit `a66bfd0ab72be6c91812d60ee6a0fc6993fe0c56`; 3 files / 28,577,694 bytes,
SHA-256 `5455967c5b5a948f2ca896e0a017a7ed26c1983be92ab4538b56b50814e2ba7e`.
The lossless payload contains 3,861 members / 259,497,686 raw bytes,
including 46 validated images. All original, decoded and committed hashes are
verified; earlier archive paths are unchanged.
