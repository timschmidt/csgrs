# Scope-local cancellation publication pilot

Isolated candidate `ab81708aa8c44eb895043879162803a964f6ebc5` preserves the
publication contract and passes the initial stack comparisons that its wrapper
predecessor failed. The change is one Hypersolve source file: seven insertions
and eight deletions relative to `9457d39b8`. It is committed and functionally
qualified, but remains unadmitted pending full cost and downstream evidence.

The [complete-consumer wrapper review](solver-cancellation-consumer-cost.md)
reproduced 168 operation-stack failures, with 848–1,104 bytes of growth. That
failing witness motivates this P5/P6/P20/P36 and plan-14.4 change. The existing
private `enter` function now checks exhaustion immediately after its callback
and before popping its scope. A private const flag enables that check only for
cancellable scopes; ordinary and coefficient-limited scopes select false.
The added closure around the callback is removed. Callback/check/scope-pop and
predicate order remain the same. No public API, assertion or source family changes.

The original publication migration still applies: cancellation is cooperative,
pre-cancelled callbacks still execute, prior resource failure keeps precedence,
and cancellation after the final check may race with success. Side effects are
not undone; this adds no total-work, latency or heap bound.

## Executed checks

The same 1,321 test invocations pass: Hypersolve release library 642 without
default features and 645 with all features; ordered-field integrations 16 + 10;
Hypercurve work-budget tests 7 and the unchanged cancellation heap-release test 1.
The unchanged native witness and all three no-import WASM witness cases pass.
Strict Clippy covers every Hypersolve target/feature, every complete-consumer
target/feature, and both affected Hypercurve test targets with all features.
Strict owner/direct-consumer documentation, formatting, whitespace and spelling
pass. These are affected checks, not complete workspace/release qualification.

The complete consumer preserves every original Rust/C source file; only dependency
paths differ. Four native resource images exercise 882 configurations, including
all seven cases, both policies, rebuilt/reused inputs, cold/eight-call-warm heap
observations, passive/active trace, two stack patterns, cold/one-call-warm stack
callbacks and added cancellation/retry. All 1,722 corresponding full output/trace
comparisons pass: ordinary results match both five-repair and wrapper controls,
and added results match the wrapper's successful cancel/retry controls.

Independent verification checks every command/log hash, original source and
witness bytes, the one-file patch, six final image hashes, exact test counts,
all complete output/trace comparisons and stack observations. All checks follow
the final source edit; the committed patch matches its tested hash.

## Stack result and limits

Each row summarizes 168 configuration groups, taking the maximum over both fill
patterns. These are the same complete operation and destruction boundaries used
by the wrapper campaign.

| Comparison | Observed delta | Initial allowance | Failed groups |
| --- | ---: | --- | ---: |
| Operation, alternative versus five-repair | −160 to +160 bytes | 256 bytes AND 4% | 0 |
| Cleanup, alternative versus five-repair | −40 to +24 bytes | 256 bytes AND 4% | 0 |
| Operation, alternative versus wrapper | −1,056 to −944 bytes | Diagnostic comparison | — |

All 336 initial comparisons and observed stack headroom checks pass without
changing an allowance. The measured improvement supports this implementation
choice for the finite corpus. It does not bound arbitrary return types or input
complexity. The monitor excludes recorded caller/thread-runtime frames above
its RSP boundary; heap warmup and stack warmup remain distinct.

Earlier wrapper charges and all prior five-repair failures remain intact. This
pilot has only one heap observation per configuration and does not supply the
alternative's complete 32-cycle lifetime accounts, three-repeat heap measurements,
monomorphized/final-image accounts, fixed-program cumulative charges, complete
consumer WASM/packages, quiet runtime or full release validation. No exception
or implementation is admitted. Accepted totals remain 57 size accounts and zero
memory exceptions. The full fifteen-phase goal remains active. No push.

## Preserved interruption and storage handling

The first driver stopped before its next command when the development-volume
20 GiB guard was reached during an independent archive commit. Its 442 completed
commands and the stop record remain preserved. After the archive writer ended,
the reviewed continuation verified and reused those commands without executing
them again. Source bytes and the original 1,800-second deadline stayed unchanged.
All 900 executed commands, including supplemental checks, complete within that
deadline (about 962 seconds after the recorded start). Independent
non-geometric verification has a separate 300-second cap.

Thirteen verified files from an older archive were already outside the evidence
repository's empty sparse selection. Their redundant checkout copies were
removed, recovering 134,998,864 bytes. Index contents and committed blobs remain
verified; the record includes its materialization command. Queue commands contain
no evidence-checkout paths. This leaves the archive available through Git and its
existing recovery scripts. The new archive waits until timed validation ends
and uses a stronger three-copy staging margin plus 20 MiB above the same reserve.
No source capacity, cost cap or queued artifact changes.

## Frozen evidence

Record `docs/evidence/2026-11-17-interval-cancellation-scope`, archive commit
`335b51a405c4fcb6be800aa41df4464a3c9d930e`: 968 files / 14,485,688 bytes,
SHA-256 `e5f317e42656b088fb53d1cf3d90b440496cac806caf1825d2c1c16ea339a92f`.
It retains raw observations, all six images, protocols, original failure and
continuation, verifiers, source patch/commit and storage verification. Original
and stored hashes, lengths, modes and compression are recorded. Earlier archive
paths and index rows remain unchanged.

The subsequent [complete qualification](solver-cancellation-qualification.md)
adds full lifetime/cumulative, native/WASM and package evidence. Distribution
and image checks pass; memory allowances remain open. A matching-launch study
isolates one allocation confound while preserving historical failures. The
original native runner also exceeded its command cap; the review records that
limitation and the corrected runner's separate evidence.
