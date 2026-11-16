# Interface compiler and display repair

Interface commit `5b532c803af86dd9e3d7a66f3651163fd2f7c68b` is retained.
The coordinated release qualification exposed fixed-size byte-iterator lints,
test assertion lints and overflowing display midpoint formulas in Alumina
Interface. The retained five-file repair preserves machine bytes, validation
order, failures and original test assertions. Display wire bends at finite
coordinates of positive or negative 2^127 now remain finite, verified by an
independent exact-bit regression. Owners and pre-change failures are recorded
in the archived admission notes (P6/P28/P29/P31–P33/P36/P37).

Stable and Rust 1.95 workspace tests, strict native/WASM Clippy, formatting,
source policy, 109 release UI tests, native/WASM release builds, WASM validation
and compressed-file checks pass. Both canonical fixture modes are byte-identical;
valid and invalid CLI outcomes retain their expectations. Complete trace
comparisons for three affected core workloads agree. Display arithmetic and
byte views add no scalar-policy dispatch. Original compiler failures, the
rejected slice-view alternative, and initial size-cap failures remain archived.

The nine-workload campaign completes all 2,496 measured invocations. Stable
packaging improves by about 1.25%; the other paired 95% CPU intervals include
no change. Peak process RSS is unchanged in every matched case. Production
allocation sites and capacities are unchanged on source review; native unwind
frame summaries match in all three shipped ELF artifacts. RSS includes inherited
pre-exec memory and is not an allocation count; frame summaries are not a general
peak-stack bound. No universal zero-slowdown or arbitrary-input memory claim is
made.

| Final artifact | File change | Complete emitted-code change |
| --- | ---: | ---: |
| Desktop | +64 B | +64 B |
| Shared library | 0 B (identical) | 0 B |
| Fixture CLI | −16 B | −16 B |
| Raw WASM | +18 B | See section ledger |
| Delivered WASM | +17 B | See section ledger |
| Gzip | +26 B | — |
| Brotli | −326 B | — |
| UI release test | +2,624 B | +2,480 B |

Each artifact and relevant section passes its own byte and percentage caps in
the [cumulative ledger](interval-size-accounts.json). Explicit supplementary
RELRO-padding revisions retain the original failed comparisons and cap the
NOBITS alignment section at 4,095 bytes; other caps are unchanged. The Brotli
asset leaves 688,115 bytes beneath its 4 MiB delivery ceiling, exceeding the
512 KiB minimum. No firmware image or flash partition is affected.

The Rust 1.95 core test compares actual fixed, previous and candidate sources in
one layout. Fixed-to-candidate file/text changes are +352/+176 bytes; incremental
changes are −48/−48 bytes. Earlier accepted +432/+224 byte charges remain intact
in their original account. Supplemental measurements do not reset the fixed
program baseline or recover previously consumed allowance. Eight new accounts
bring the total to 54; all preceding accounts retain their accepted charges.

Evidence: `docs/evidence/2026-11-16-interval-interface-compiler-display` in
csgrs-evidence, commit `efc1e26c633fa6e7004d583b4981b94876afde18`: 608 files,
857,548,059 bytes, SHA-256
`7f94dd7541f665d233bb8d41494e2a278d38fe3531c79dcebb5a5728c1583395`.
It preserves linked artifacts, build configurations, raw samples, original
failures and source snapshot `0db15b4a749692cac610496ec4f1c1f8bdd73ee0`.
This is a frozen Interface subrecord of ongoing Stage 92. The other validation
lanes and all fifteen phases remain open; no full release, push or publication
is credited.
