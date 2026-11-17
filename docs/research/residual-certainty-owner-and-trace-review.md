# Residual-certainty owner artifacts and trace allocation

Hypersolve owns the certification-summary and diagnostic repair under
P6/P8/P9/P19. This review extends the [certainty qualification](residual-certification-ownership-review.md)
and [packaged-consumer evidence](residual-certainty-packaged-consumers.md).
Candidate `aad7409a5928e309812d9a5cc361d46313685266` remains isolated and
unaccepted. The missing witness here was its final owning-crate release images
and their completed controls, including the original size gates.

## Final owner images

Fixed `c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`, measured current
`4192d8c726d3333a31cb7eddc30cc0d464d61411`, and candidate are built at the same
checkout and dedicated target path, using Rust 1.95.0, locked offline resolution
and the unmodified repository release profile. This is separate from the earlier
portable consumer's thin-LTO profile. Both no-default and all-feature builds
are measured; dispatch tracing requires the latter.

Thirty-three ordinary images and two candidate-only regression images complete
5,264 functional checks. Complete library/smoke test identities and outcomes,
selected benchmark identities, and full dispatch report/stdout bytes agree
across revisions. Existing tests and count assertions are unchanged.

| Executed artifact | No-default cases | All-feature cases | Revisions |
| --- | ---: | ---: | --- |
| Library tests | 624 | 627 | fixed/current/candidate |
| Smoke tests | 211 | 211 | fixed/current/candidate |
| Affected certification benchmark controls | 12 | 12 | fixed/current/candidate |
| Complete representation benchmark controls | 22 | 22 | fixed/current/candidate |
| Dispatch workloads | — | 11 | fixed/current/candidate |
| New residual-certainty regressions | 4 | 4 | candidate only |

Six retained-fuzz images are also measured and start successfully in metadata
mode. That harness returns before executing its workload in this mode. These
are not six completed fuzz campaigns or benchmark timing results. The remaining
owner integration suites retain their earlier qualification; this campaign
does not claim to rerun all of them in release mode.

Independent verification checks all 35 image hashes, like-image release
profiles, final section totals, 246,429 per-image unique-address text symbols,
31 selected source snapshots against Git objects, and the raw functional logs.
Both no-default/all-feature all-target Clippy configurations pass with
`-D warnings`; workspace formatting passes. There are no compiler warnings.
All final artifacts and their original symbol/section listings are archived.

## Original file-size failure and separate proposal

There is one failed original metric: the no-default unit-test executable's
file grows from 12,974,744 to 12,990,200 bytes, a positive charge of 15,456 bytes.
It exceeds the original 4 KiB incremental cap while remaining within the
16 KiB cumulative cap. Every other measured metric and all declared image
capacity/reserve checks pass. Historical positive charges remain reserved,
including those on images whose fresh measurements are smaller.

For that same executable, text/executable sections shrink by 288 bytes and
unique-address named text shrinks by 695 bytes. Total allocated sections are
unchanged. File growth comprises 10,780 symbol-string bytes, 2,688 symbol-entry
bytes, 1,984 net file-backed allocated-section bytes and four other file-layout
bytes. The extra file-backed allocated bytes are balanced by 1,984 fewer NOBITS
RELRO padding bytes. These size observations do not prove unchanged runtime.

A separate, unadopted proposal would raise only this configuration's incremental
file cap from 4 to 16 KiB, retaining the 1% incremental cap and the existing
16 KiB AND 2% cumulative cap. It allows the independently demonstrated
correctness repair to consume the existing cumulative file budget in one
increment. It does not enlarge that total budget or borrow another target's
allowance. The original 15,456-byte charge remains, leaving 928 bytes; every
other cap remains unchanged. The 16 MiB fixture capacity and 2 MiB reserve also
remain unchanged, with 3,787,016 bytes left by the candidate image.

The original campaign still records the file failure. Neither the proposal nor
passing functional checks admit the source or a resource account. These are
test-image capacities, not firmware partitions or complete application bounds.

## Direct attribution of one trace-allocation variation

The earlier memory campaign found repeated 632-byte differences within
unchanged binaries. A separate host-glibc diagnostic now captures size-632
allocation backtraces from the original fixed/current/candidate trace binaries,
without changing their bytes or global ASLR settings. It uses one exact-zero
residual case with STRICT policy, rebuilt inputs and eight warmup calls.

The 108 processes comprise 48 plain and 48 interposed active-trace runs, plus
six plain and six interposed passive controls. Full output and trace strings
agree within each recording configuration across modes and source revisions.
The original fixture assertions still execute. All captured size-632 stacks
resolve to B-tree insertion inside Hyperreal's `collect_dispatch_counts`.

In the 48 interposed active processes, the measured operation contains either
two or three such allocations. The extra node corresponds exactly to one more
allocation/deallocation, 632 more requested traffic bytes and 632 more requested
peak bytes; retained requested payload stays at 1,824 bytes. Plain processes
reproduce both allocation states on each binary. Passive controls are constant
and have no captured 632-byte event.

This establishes the allocation site for this finite active-trace case.
Static-address hashing followed by B-tree insertion remains a plausible
explanation of insertion-order variation; its ASLR causality is not separately
proved. The diagnostic does not explain passive-trace or stack failures.
Interposition can perturb process state and cost, so these are attribution
results rather than unchanged resource measurements or performance evidence.
No earlier positive charge is subtracted, baseline reset or memory cap changed.

## Remaining admission and evidence

The candidate branch is restored. The original quiet watcher resumes after
the build/archive holds with the same finite horizon and frozen command plan.
External routing work overlaps these checks, so their durations provide no
quiet-runtime comparison. The two earlier DRC failures remain recorded.
Native memory failures, complete consumer capacity, cancellation/retry, quiet
runtime, successive operations, full applicable validation and composition with
the four earlier repairs remain open. No production source, inventory row,
accepted size account or memory account changes: 57 size accounts and zero
memory exceptions remain accepted, and the fifteen-phase goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-residual-certainty-owner`, local
commit `1da0eebf99e52060d5f08dd1f7c80cf8e28def6d`; 548 files / 149,749,754 bytes,
SHA-256 `984f4104fc268c76d6c94dcbc0e74b98741f43fbc8b6c436614807a21b3f6cef`.
Original executable and command-output bytes are preserved with lossless gzip
encoding and source/stored digests. The initial raw-log whitespace rejection
and provenance-collector metadata error are preserved alongside their corrected
verification. All previous archive paths and manifest rows remain unchanged.
The source stages and this snapshot are frozen. No push.
