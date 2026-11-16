# Taylor scheduling clone removal: bounded-size reconsideration

Target revision: `924bef14429b31229c82fc20de43deadd35a13ce`.
Owner: Hypersolve's demanded coefficient derivative scheduler, under
P1/P4/P5/P6/P8–P10/P18–P20/P36 and the interval refactor plan.
The production candidate is rejected and removed. The bounded size exception
resolves its former size-only objection, but does not waive its newly measured
local-stack regression. No new size allowance is consumed.

## Candidate and exact controls

The scheduling visit needs only the node's two child indices. The candidate
borrows those indices and delays cloning until the ready/construction visit.
Budget checks and charges, memo access, child push order, derivative rules,
predicate order, and the retained graph remain in their original order.
The same two-line change was previously rejected in the wrapping diagnostic.
This review reproduced it on the current revision rather than reusing that
historical assessment as acceptance evidence.

The current fixture has a 200-byte coordinate name, a large exact rational,
aliasing output rows, and nine demanded coefficient requests. All 448 exact
output, serialized definition, work/size-boundary, both-policy certainty and
full dispatch records match. Independent polynomial expectations check the
values. All 36 downstream two-rate wrapping records also match after excluding
only elapsed-time fields.

Across 100 constructions, allocations/deallocations decrease from 20,300 to
20,200 and allocated bytes from 1,430,000 to 1,410,000. Reallocations remain
1,100, peak live bytes 3,404, and final live bytes zero. These are warmed
construction measurements, including output destruction, not a claim about
all Taylor workloads.

## Timing and final native size

A separate System-allocator probe shares the fixture with the counting probe.
Four ABBA blocks, pinned to CPU 0 with ASLR disabled, each use 30 samples of
2,000 complete operations per mode. The construct/evaluate/classify/serialize/
drop modes improve by 0.94–1.35% under STRICT and 1.10–1.25% under
APPROXIMATE_512. Construction-only blocks improve by 1.37–5.84%; the first
baseline run is noticeably slower than the later runs, so that upper value
must not be presented as a stable speedup. The remaining three construction
blocks improve by 1.37–1.49%. No builds or tests ran during these timings.
This is evidence for this fixture, not the unexecuted broader benchmark gate.

Forty-two existing final native release artifacts and three complete probes
were measured. Existing affected examples/benchmarks gain 48 bytes of text;
the probes gain 58–80 bytes of unique named text. Existing final files grow by
at most 136 bytes, probes by at most 96. Every measured native artifact passes
the already declared byte and percentage caps, individually and against the
fixed `c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb` baseline. The earlier lattice
harness consumption is preserved; it is not reset. This is not an accepted
exception or a complete cross-target cost qualification.

## Independent rejection: stack

Disassembly of the actual all-feature `flow_noise` executable shows six saved
registers in both versions, but its derivative scheduler's stack reservation
increases from `0x138` (312) to `0x148` (328) bytes. The coefficient and timing
probes independently show the same 16-byte increase. The wrapping probe's
reservation stays at 328 bytes. This is a local-function measurement, not a
measurement of maximum whole-program stack depth.

The unchanged memory/stack gate rejects the production candidate. No inlining,
layout, or unrelated code changes were tried to disguise the increase. Heap
savings and the bounded binary allowance do not waive this gate.

An attempted in-repository trace assertion also failed: the all-feature debug
test's first-policy counts differ from the standalone release fixture. That
new test and fixture are removed, with the failure and patch retained in the
evidence. This does not establish a production behavior regression; it shows
that the proposed fixture was not yet qualified for that build configuration.
The passing baseline/candidate standalone comparisons remain separate evidence.

## Scope and remaining qualification

The candidate WASM coefficient consumer and a candidate `successive_operations`
executable built successfully. Fixed/current WASM comparisons, WASM execution,
broader owner/downstream timing, successive-operation runs, and the proposed
trace-test size allowance were not completed after the independent rejection.
No claims of passing those gates are made. Current default root/Interface
feature trees do not enable `taylor-model`; the firmware tree does not include
Hypersolve. No firmware flash-fit or whole-image claim is made.

After removing the candidate and attempted test, all 1,676 owner/downstream
tests and doctests pass across 84 reported suites (263.06 seconds),
with no failures or ignored cases. Strict all-feature/all-target Clippy and
whole-workspace formatting pass. All 42 restored release artifacts match their
current-baseline SHA-256 hashes exactly. The failed candidate run remains
recorded separately: 940 passed and the one new unqualified test failed before
the remaining Hypersolve suites ran. The existing cumulative size account is
unchanged. Full-plan completion, automatic noise orchestration, and global
sensitivity remain open; this review completes only this reconsideration.

Raw evidence: `docs/evidence/2026-11-16-interval-taylor-scheduling-reconsideration` in local evidence archive commit
`4b31e7d22673bce20812bd2aa952f11074595a0f`; manifest entry in
[`docs/evidence-archive.tsv`](../evidence-archive.tsv).
