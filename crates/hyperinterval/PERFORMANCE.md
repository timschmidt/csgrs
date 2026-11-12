# Performance and resource ledger

No arithmetic optimization or consumer migration is qualified yet. The
phase-1 relations allocate nothing: their only storage is an optional pair
of primitive endpoints, and they call a fixed number of integer comparisons.
Production source forbids unsafe Rust and imports neither alloc nor std.
Cargo feature closure, tests and layout checks must validate that claim.

On a target with i64 alignment 8, the explicit-empty carrier uses 12 bytes
for i32 endpoints and 24 bytes for i64 endpoints. Alumina's nonempty Q30
wrapper is 8 bytes; therefore this carrier must not silently replace that
wrapper's layout. Delegation and retained representation need measured review
in phase 9. Source sharing alone is not evidence of improved firmware cost.

Baseline commands, raw outputs and pending gates are recorded in the root
progress record. No speedup, firmware cycle count, or binary-size improvement
is claimed by the skeleton.
