# Complete-chain trace allocation attribution

The complete five-repair solver consumer reproduces allocation variation in
trace snapshot construction. In this finite diagnostic, each additional B-tree
node accounts for one allocation/deallocation and 632 requested bytes of peak
and traffic. Operation-retained requested payload stays at 10,381 bytes. This
extends the [standalone residual attribution](residual-certainty-owner-and-trace-review.md)
to the complete consumer; it does not remove any resource charge or failure.

The [resource campaign](solver-five-repair-resource-controls.md) has unresolved
requested-peak growth up to 1,896 bytes. The diagnostic uses its original fixed
`c9f1f9ac` and final `4a04dd974` trace resource images, case zero, STRICT, reused
input and eight warmup calls. Images, fixture assertions and allocator monitor
are unchanged. The existing host-glibc interposer captures size-632 allocation
stacks in owned child processes; system ASLR and unrelated jobs are untouched.

All 56 processes complete within the declared 120-second bound: twelve active
plain/interposed pairs and two passive pairs on each image. Full output and trace
strings agree in each recording mode across both images and interposition,
providing 54 additional full-string comparisons. Every original fixture
assertion passes. Independent verification re-parses raw streams, re-symbolizes
all captured binary offsets, checks image/interposer hashes and reconstructs
the allocation relationships.

All captured size-632 allocations resolve to B-tree insertion inside Hyperreal's
`collect_dispatch_counts`. The original `allocation::observe` frame identifies
those belonging to the measured operation; warmup captures remain separate.
The 24 interposed active processes show four classes:

| Observed nodes | Allocations | Deallocations | Requested traffic | Requested peak | Operation-retained requested bytes |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 6 | 177 | 117 | 26,033 | 14,901 | 10,381 |
| 7 | 178 | 118 | 26,665 | 15,533 | 10,381 |
| 8 | 179 | 119 | 27,297 | 16,165 | 10,381 |
| 9 | 180 | 120 | 27,929 | 16,797 | 10,381 |

Reallocations stay at eight. Plain active processes also show variation on each
unchanged executable. All eight passive controls have the same allocation tuple
and no captured size-632 event. This attributes the observed requested-byte
variation in this fixture; it does not establish unchanged glibc usable bytes,
process RSS, whole-input lifetime cost or every earlier positive source charge.
The retained column is operation-local, not total retained geometry.

Static-address hashing followed by B-tree insertion remains a possible source
of insertion-order variation. The diagnostic does not separately prove ASLR
causality. Interposition may perturb allocation state and cost, so these are
site-attribution observations, not unchanged resource measurements or timing
comparisons. No source optimization or revised allowance is adopted. Earlier
failed caps, cumulative charges, passive-trace failures and other workloads
remain intact; resource/cancellation and quiet-runtime requirements stay open.

Frozen evidence: `docs/evidence/2026-11-17-interval-five-trace-attribution`, local
archive commit `fbaef1fe81d4ccf1a9ba1acfd3f43473e5718300`; 129 files / 306,365
bytes, SHA-256 `21c21fcdc02ca3244204f681749cfffe4710524a7f3ff00ef14e0e2ab811ad1a`.
The archive preserves all raw streams, resolved stacks, scripts, the unchanged
interposer source/binary and independent verification. Original executable
images remain hash-bound to the earlier resource archive. All previous archive
paths remain unchanged. No source or memory account is admitted.
