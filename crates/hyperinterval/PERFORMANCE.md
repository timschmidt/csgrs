# Performance and resource ledger

No consumer migration is qualified yet. The
primitive relations and arithmetic allocate nothing: their only storage is an optional pair
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

## Phase 2 focused Q30 probe

The maintained `conformance/fixed_cost.rs` client materializes both endpoints
for the same four successful inputs. It includes construction, uses out-of-line
operation wrappers and alternates legacy/shared sample order. Rust 1.88 release,
x86_64 Linux, 30 samples/backend/operation, 65,536 completed operations/sample,
and separate warmup; the firmware comparison source is
`bdee397d0b80841f16bf17e7bcd029b4feb2c212`. Exact differential tests precede timing.
Raw output and hardware/toolchain details are retained in the evidence record
linked from the root progress ledger.

The first candidate kept directed quotient helpers out of line. It was rejected
as a performance candidate: median shared/legacy ratios were 2.044 for multiply,
1.301 for divide and 3.004 for square. Inspection found dynamic i128 division
instead of constant-scale specialization. Coverage work was also active during
that first exploratory probe, so it is not a controlled cross-run speedup claim.
The corrected candidate marks those two helpers inline, preserving every
arithmetic branch and operation. With coverage finished, the paired probe was:

| Operation | Legacy median ns/op | Shared median ns/op | Shared / legacy |
| --- | ---: | ---: | ---: |
| Add | 21.233 | 19.167 | 0.903 |
| Subtract | 18.126 | 16.429 | 0.906 |
| Multiply | 33.980 | 20.185 | 0.594 |
| Divide | 114.711 | 86.655 | 0.755 |
| Square | 22.300 | 17.799 | 0.798 |

These are finite probe medians, not general performance guarantees or evidence
of FOC timing equivalence. The shared five-operation wrapper is 1,086 text
bytes versus 488 for the legacy wrapper; callees differ, so those symbol sizes
alone do not compare complete backends. The combined harness has 346,782 text
bytes, 16,208 data bytes and 296 BSS bytes. Its initial non-inline version had
347,526 text bytes. No firmware binary/cycle/stack or HIL result is inferred;
those remain migration gates. Host cycle counters were not available here.
