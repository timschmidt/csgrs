# Complete cancellation consumer: initial cost qualification

The wrapper candidate `9457d39b8b03e58ebabeb7871549b362371f5f7b` passes the
complete consumer's functional comparisons but exceeds its initial stack and
some active-trace memory caps. It remains unadmitted. This extends the
[publication-boundary functional review](solver-cancellation-publication.md)
with actual complete-consumer measurements, not a change to an allowance.

## Contract and corpus

Hypersolve owns P5/P6/P20/P36 and plan item 14.4. A separate module wraps the
existing complete solver operation in an unlimited cancellable scope. Its added
path validates the complete report, cancels without another instrumented check,
requires the exact cancellation error, then retries with a fresh token. The
fixture, independent assertions, allocator monitor, guarded stack monitor and
three lifetime runners retain their original bytes: seven inherited files in
all. The callback/check/scope-pop semantics remain those documented in the
candidate migration.

The five-repair `4a04dd974` baseline and wrapper candidate each build default,
trace, cancel/retry and trace+cancel/retry features, producing 24 native images.
Cases retain 1/4/16 variables, separate 64-variable stress, rebound input,
invalid-adapter retry and STRICT → APPROXIMATE_512 → STRICT qualification.
Both policies, rebuilt/reused inputs, cold/eight-call-warm heap measurements,
passive/active trace and operation/destruction stack boundaries are covered.
Stack observations use both fill patterns and cold/one-call-warm callbacks;
that warmup differs from the heap runner's eight calls.

The driver completes 4,047 commands in about 839 seconds within its 5,400-second
limit. Among 4,032 executed consumer processes, 3,906 succeed and 126 reproduce
the baseline publication assertion failure. Failed results never supply cost
baselines. All corresponding normal full outputs and dispatch traces match;
added retry outputs match their ordinary counterparts. The independent verifier
checks the exact command multiset, 1,848 full output/trace domains and 924 retry
output domains. Four strict Clippy feature configurations and formatting pass.
Analysis and independent verification finish within a separate 600-second
non-geometric handling limit.

## Memory observations

Three fresh heap observations per configuration retain the initial control too.
Both rebuilt and reused full lifetimes cover 252 successful 32-cycle histories,
or 8,064 cycles. Epoch sums include required reports, cleanup and input teardown;
reused histories include initial input setup. Requested bytes and glibc usable
payload are separate. Signed single-epoch post-drop deltas are observations,
not invented nonnegative percentage baselines. Debug serialization remains
outside measured epochs, as in the inherited harness.

The initial comparison is five-repair → wrapper for this newly scoped consumer.
Fixed `c9f1f9ac` cumulative accounts and intermediate positive charges remain
required before admission. Existing unscoped consumer accounts, failures and
charges are unchanged. Added cancel/retry has no correct baseline and uses
absolute total capacity/headroom checks.

Of 504 resource groups, 172 fail one or more initial growth comparisons:

| Failed comparison | Checks |
| --- | ---: |
| Operation touched-stack growth | 168 |
| Requested / usable peak growth | 9 / 9 |
| Requested / usable traffic growth | 5 / 5 |
| Allocation / deallocation count growth | 1 / 1 |

All 168 normal operation-stack groups grow by 848–1,104 bytes, above the
256-byte AND 4% allowance. Maximum relative stack growth is about 6.96%.
Cleanup stack growth is at most 40 bytes and passes. The largest requested and
usable peak increment is 1,264 bytes; active-trace groups can still exceed the
2% cap. History traffic and count failures retain their exact rows and limits.
No cap is enlarged to reinterpret these failures as passes.

All absolute totals, observed stack/payload headroom, nonnegative complete
footprints and no-additional-retention-after-cycle-two checks pass. Maximum
observed requested/usable peak is 577,406/613,104 bytes; persistent payload is
24,976/25,424 bytes. Maximum touched operation stack is 23,912 bytes and cleanup
stack 2,344 bytes. Stack observations cover the called computation beneath the
monitor's recorded RSP, including the callee red zone; caller/thread-runtime
frames above that boundary are excluded and reported. Each mapping is 2 MiB
with a guard page and at least 1 MiB observed untouched reserve.

These finite single-worker capacities are not firmware partitions, RSS or
allocator-metadata bounds, total-work guarantees or quiet runtime measurements.
A source-level hypothesis is that retaining a large callback result across the
additional wrapper boundary causes avoidable stack growth. A separate scoped
implementation can test that hypothesis; it cannot erase this record or its
positive charges. The active-trace heap failures remain separate obligations.

## Images and remaining gates

A [scope-local alternative](solver-cancellation-scope-pilot.md) subsequently
passes the same initial stack limits. That pilot preserves this wrapper record
and does not clear its other pending accounts or admission requirements.

All initial native image caps and 16 MiB capacity/2 MiB reserve checks pass.
Ordinary file deltas range from −12,096 to +9,168 bytes. Default resource allocated
sections grow four bytes despite its smaller file. Trace allocated growth is
4,080–4,100 bytes, and named code growth 2,314–4,292 bytes. These positive metrics
remain explicit; savings elsewhere do not offset them. These are incremental
comparisons, not completed fixed-program cumulative size accounts.

Neither source nor an exception is admitted. Earlier memory failures, all prior
positive charges, complete cumulative accounting, quiet timing, successive
operations, WASM/package consumers and applicable downstream/release checks
remain open. Accepted accounts remain 57 size and zero memory; the full
fifteen-phase goal remains active. No push.

## Frozen evidence and recovery

Record `docs/evidence/2026-11-17-interval-cancellation-consumer`, archive commit
`20d859de23726bf3c05c48ac5b25686be9c5ff2c`: 14 files / 57,952,780 bytes,
SHA-256 `f49b4b07ece73452600445c65339605e0e9f24ef3734530be3bb5375da5cb065`.
The lossless XZ/tar container holds 4,143 prepared members, including every raw
log, all 24 images, original assertions, protocols, scripts and failed cost
checks. Its member manifest and inner source/encoding inventory are verified.
Prior archive paths and index rows remain unchanged.

Initial per-file archival and container attempts stopped at the unchanged
storage guard before writing the evidence record. Their failures are retained.
Fourteen previously archived residual resource images (55,204,520 bytes) and
twelve packaged images (27,818,345 bytes) were verified against committed archive
bytes and relocated to RAM through stable original-path symlinks. Both restore
scripts populated fresh alternate directories and verified every recovered file.
Maps, modes, hashes, durable references and the initial manifest-digest correction
are retained. Queue plan, held parent and queued DRC image remain unchanged;
no unrelated job was interrupted. This recovery check is not a restarted runtime
qualification. The final archive preserves the 20 GiB reserve and staging margin.
