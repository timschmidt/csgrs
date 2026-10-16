# Fixed distribution and concatenation cost comparison

Use the original complete linear-8, grid-4x4, arc-12 and feature-merge-22 inputs,
plus duplicate concatenation. Preserve their exact position and triangle rows
from `reproduction-v2`; compare candidate serialization separately from
independent semantic acceptance tests. No fixture, policy or validation may be
weakened to produce a completed row. This is a correctness extension, not an
optimization claim or a substitute for the full supported corpus.

Build standalone baseline/candidate drivers against pinned libraries, with the
same toolchain/optimization. Use three paired timing repetitions per fixture
in fresh cold processes and in fresh processes with one identical untimed
warmup. Alternate variant order within pairs. Measure allocation in separate
instrumented binaries, once per fixture, variant and temperature. The matrix
therefore has 60 timing rows and 20 allocation rows. Retain every attempt and
unfavorable result; a failed child is not completed cost evidence.

The fixed probe interval includes constructing the unit cube/input copies and
the complete native distribution/concatenation result. Input construction is
identical between variants; this scope differs from the kernel benchmark's
preconstructed input. Actual benchmark consumers are run separately. Exact
serialization occurs in an untimed replay after all allocation and post-drop
retention snapshots. It cannot contribute work to the timed or instrumented
construction. Record local numerical certainty separately from retained PWN
facts and topology/fidelity tests.

Allocator-requested counts/bytes, incremental peak live requested bytes and
post-drop retained deltas exclude allocator overhead and are not a total heap
limit. Preserve process-resource records, driver/library sizes, source deltas,
dependency/toolchain/hardware pins, policies and cache conditions. All prior
failed diagnostic phases and external containment outcomes remain retained.
No internal total-work, heap-byte or cancellation bound is established here.
