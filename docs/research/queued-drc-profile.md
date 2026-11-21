# Diagnostic profile of the queued DRC failure

The exact queued HyperDRC image still fails the unchanged 120-second worker-CPU
assertion. This diagnostic does not qualify a timing run or clear either prior
failed replay. Source `955593aa9e25727c8e8cfbd0ab1e599d6a09dd53`, the executable,
the complex board fixtures and the frozen queue are unchanged after verification.

The bounded debugger run collected 66 samples at two-second intervals with up
to 31 worker stack frames. It completed within its 600-second outer limit; the
inferior exited 101 after reporting 132.669405418 seconds of worker CPU. The
debugger exited normally. Instrumentation and external work prevent comparing
this time with the two original failed replays as a performance result.

Inclusive sample counts overlap: native region admission appears in 31 samples,
profile aggregation in 26, regularization in 31, candidate scheduling in 15,
intersection preparation in 12, fragment classification in nine and rational
GCD work in 16. These counts locate work; they are not exclusive CPU shares or
proof of a particular optimization's benefit. Raw stacks and test output remain
available, including the failure.

Review found that the DRC check runner already shares copper-layer construction
through a `OnceCell`. Another board cache is not justified by this profile.
The borrowed quadratic-promotion refactor remains rejected: its earlier owner
benchmark was slower in 19 of 20 balanced pairs, with median ratio
1.0170898908727681. Bounded memory growth does not waive that runtime gate.
No production optimization, predicate reordering, scalar-core restructuring or
spatial-index change follows from this diagnostic.

The original quiet watcher is terminal. The workflow parent remains held with
36 commands pending; no watcher horizon, command budget or earlier outcome is
reset. Fresh review is still required before queue resumption. This record does
not admit source or resource accounts or complete the interval-refactor goal.

Frozen evidence: `docs/evidence/2026-11-17-interval-queued-drc-profile`, local
archive commit `fa3894a4d4ded73fb61c389b1781f8a68453a00d`; 12 files / 200,433
bytes, SHA-256 `e02a9ed79f7ff2c116f0156bd5b877d461a76831944f1d86d262f158fafba85a`.
It includes the sampler, raw debugger stream, all samples, source/fixture/image
hashes and independent verification. The image remains at its hash-bound path;
it is not duplicated in this small diagnostic record. Fixture equality was
checked during the diagnostic and again afterward, not in a newly collected
pre-run fixture snapshot. Earlier archive paths remain unchanged.
