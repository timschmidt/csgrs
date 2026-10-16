# Preserved-workload resource disclosure

This is a typed-failure and certainty migration, not an optimization claim.
Use the five original `profile_primitives/constructor` cases: involute gear,
cycloidal gear, involute rack, cycloidal rack and NACA airfoil. Their dimensions,
sample counts and materialized outputs remain unchanged. Exact coordinate
records and independent mathematical tests establish preservation separately
from the benchmark's count/checksum fields.

Fix five alternating baseline/candidate process pairs for each condition:

- Cold: fresh process, no explicit warmup, one timed iteration per row.
- Warm: fresh process, one eight-iteration warmup, eight timed iterations.

Each process selects all five named cases. “Cold” does not mean that shared
scalar or OS caches are evicted between cases. Both all-feature binaries use
the existing dispatch-trace harness. Timed work includes parameter creation,
construction, output materialization, component counting and result disposal.
Keep every row and raw dispatch/resource log; do not extend or select the matrix
based on favorable results. Run both affected feature catalog cases once on
each executable as separate completion checks, not a performance comparison.

Also compile the same standalone `resource-probe/allocation_v1.rs` against the
pinned baseline and final candidate libraries. Each case/condition/variant
gets its own process, for twenty allocation rows. A 4096-byte allocation and
deallocation calibrates the counter before every measurement. Cold has no
construction warmup; warm has one complete construction and drop before the
measured construction. Assert successful nonempty output in both binaries.

Allocator counts cover successful Rust global allocator requests, including
zeroed allocations, with reallocations reported separately. Requested bytes
exclude allocator metadata and allocations bypassing Rust's global allocator.
Peak additional requested bytes are measured above the pre-construction live
baseline; retained delta is measured after dropping the returned region and
includes native/scalar/modeling cache effects. These values are not heap-byte
limits. Whole-process `/usr/bin/time` RSS includes untimed setup and is separate.
No affinity, fixed-frequency or unrelated-process isolation is claimed.

The allocation driver is additional evidence code, written after production
sources were frozen. The broader production qualification does not validate
that driver; its own warning-denied builds, calibration and guarded executions
do. The resource-phase runner guards it explicitly. Production/test sources
must still match the qualified snapshot byte for byte.

Retain revisions, full source snapshots, dependency locks, executable hashes,
toolchain, host snapshot, environment, timing boundaries and raw observations.
Any adverse timing/allocation result remains visible. Failure/capacity paths
are established by acceptance tests rather than credited as faster geometry.
