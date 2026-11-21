# Residual-certainty usable allocation review

The isolated Hypersolve certainty repair remains unaccepted. A finite diagnostic
now explains allocator-usable-byte variation in one previously failing memory
configuration while preserving every original failure and positive charge.
It extends the [owner and trace review](residual-certainty-owner-and-trace-review.md)
under the [bounded memory exception](../admission.md#bounded-memory-growth-exceptions).
Hypersolve owns the P8/P9 certainty repair; this review addresses its missing
resource attribution under P36/P37, without changing production arithmetic.

The fixed program revision is `c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`,
the measured pre-repair revision is `4192d8c726d3333a31cb7eddc30cc0d464d61411`,
and the candidate is `aad7409a5928e309812d9a5cc361d46313685266`. Their original
SHA-verified no-feature resource executables are reused. Fixed and pre-repair
images have identical bytes. The target is case 24: an equality residual with
the angle-addition-zero construction plus a tiny positive value, STRICT policy,
passive trace, cold state and rebuilt inputs. Its maintained assertion requires
the qualified unknown result; this is no new exact-completion claim.

## Executed diagnostic

The prospective protocol bounds process count, output, time and storage. A
process-local glibc interposer reads the original allocator measurement flag
through its verified one-byte ELF symbol; it never writes that flag. It records
allocation, reallocation and free events during the operation, output cleanup
and input cleanup. Original executable bytes, global ASLR settings and original
measurement boundaries remain unchanged. Compilation with
`cc -shared -fPIC -O2 -Wall -Wextra -Werror` passes.

All 146 processes preserve complete output and trace records. The 73 instrumented
processes reconcile all 219 epochs exactly against the original executable's
allocation/deallocation/reallocation counts, requested allocation traffic and
usable traffic, peak and live-byte counters. A separate verifier reconstructs
the raw event logs, compares corresponding event ordinals and verifies every
original image digest.

All instrumented operations have the same 6,312-event request sequence. Both
plain and instrumented operations have 2,910 allocations, 2,809 deallocations
and 593 reallocations. Requested traffic is 193,309 bytes,
requested peak is 14,100 bytes and requested live payload is 9,088 bytes in
both plain and instrumented processes. Seventy-three corresponding allocation
or reallocation event positions exhibit different usable sizes across the
instrumented runs despite identical request sizes. Their recorded usable-size
differences account exactly for the traffic differences. For example, some
one-byte requests receive 24 usable bytes and others 40.

Plain processes also reproduce usable-traffic variation on each unchanged
binary: 215,928–216,104 bytes for fixed, 215,928–216,168 for pre-repair and
215,992–216,136 for candidate. These are observed ranges, not statistical bounds
or a universal allocator model. Interposition may perturb allocation layout;
its measurements cannot replace original admission costs or be subtracted from
earlier charges. Requested sizes of pre-epoch allocations freed during cleanup
are not inferred from usable sizes. This diagnostic supplies no runtime result.

## Disposition

The sampled passive usable-byte difference is attributable to differing usable
sizes for unchanged requests. This does not explain every configuration in the
memory campaign, establish a zero-cost repair or admit a memory exception.
The two known stack cases, full consumer capacity, cancellation/retry, quiet
runtime, successive operations, applicable validation and combined-repair
qualification remain open. Any cap revision still needs its own bounded,
cumulative justification and preserved headroom.

No production source, inventory row or accepted resource account changes.
The inventory remains partial at 1,644 rows; 57 size accounts and zero memory
exceptions remain accepted. The existing quiet watcher resumes after each
finite hold with its original horizon and command plan. All fifteen plan
phases remain active. No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-residual-usable-allocation`,
local commit `6de0aab87eb0c23c9f8883dd88c6f1411c1c83b7`; 171 files /
2,662,214 bytes, SHA-256
`fae97e7a8a16aebbd67440cf3f4a5ab665fcffbe42a845df79f431ec648bcb9e`.
Raw outputs and the diagnostic shared object are preserved with lossless gzip
encoding and original/stored digests. Earlier archive paths and manifest rows
remain unchanged. The diagnostic source stage and snapshot are frozen.
