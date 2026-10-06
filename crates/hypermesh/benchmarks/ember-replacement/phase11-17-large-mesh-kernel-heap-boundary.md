# Phase 11/17 large-mesh kernel-heap boundary checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hypermesh `6d0ad0311bb47f0a5faf7b770a896c136edab057`.

This checkpoint closes the direct input-versus-Boolean requested-payload
lifetime measurement gap. It expands the permanent heap corpus and measurement
infrastructure; it does not claim Phase 11 corpus completion, Phase 17
performance completion, Phase 18 completion, or CGAL EPECK parity.

## Result

Every large or XL heap fixture now has a unique executable selector shared by
the ordinary process profiler and a separate allocator-instrumented kernel
probe. The latter measures:

1. the process baseline after argument parsing;
2. live prepared-input payload after raw fixture storage and optional PWN
   priming temporaries are gone;
3. peak live payload while the production `boolean` call runs;
4. incremental kernel peak above the retained-input boundary;
5. live payload after `boolean` returns;
6. payload released with the output;
7. input-attached fact/cache growth after output drop; and
8. residual live payload after the inputs also drop.

The 6,144-triangle control peaks 15,806,112 bytes above its native prepared
inputs and 15,865,832 bytes above its raw/general prepared inputs. The
23,788-triangle full rotated YeahRight intersection peaks 158,262,120 bytes
above its inputs. Its exact empty result owns only 56 bytes, while 24,389,848
bytes remain attached to the still-live input scalar identities after output
drop and are released when those inputs drop.

Every measured byte count, allocation count, certainty, and topology is
identical under `STRICT` and `APPROXIMATE_512`. No approximate terminal is
consumed: all rows are `Certified`.

## Measurement boundary

`large_mesh_kernel_heap_probe` is the only executable that installs the
tracking global allocator. It delegates every allocation to `System` and
records successful requested layout bytes, live blocks, allocation,
deallocation, and reallocation calls, byte growth/release, and the live-byte
high-water mark. Reallocation changes live payload only by the size delta.
Internal conservation assertions require interval byte and block changes to
match the recorded events.

The authoritative probe is serialized on one pinned CPU. The peak is reset
only after fixture preparation, optional native PWN priming, and view
construction. The result is captured before any printing. Output and input are
then explicitly dropped and sampled independently.

This metric is exact Rust allocator requested payload, not RSS or allocator
overhead. It complements rather than replaces Heaptrack/Massif and `/usr/bin/time`.
The ordinary `large_mesh_heap_probe` has no tracking allocator, so hardware
counters and external heap profilers retain their former allocation path.

## Permanent heap corpus

The monotonic manifest now requires every `heap` record to be `large` or `xl`
and to name at least one unique `heap_probe_modes` selector. The current
selectors are:

| Selector | Fixture/path | Requested operation |
| --- | --- | --- |
| `boxes-3072` | 6,144 triangles, native views with policy-qualified PWN facts primed | union |
| `boxes-3072-general` | the same 6,144 triangles through raw borrowed views | union |
| `yeahright` | 852-triangle control-hull/box scale row | union |
| `yeahright-4` | 3,372-triangle deterministic scale sibling | union |
| `yeahright-8` | 13,452-triangle deterministic scale sibling | union |
| `yeahright-full-rotated` | 11,894-by-11,894 full source/rotated-source hard case | intersection |

The fixture selector chooses only input construction/ownership and the
requested public operation. Both probes call the same sole production
`hypermesh::boolean` entry point. There is no fixture, coordinate, triangle
count, expected result, competitor, or measurement-state dispatch inside the
Boolean engine.

## Direct requested-payload results

Each row below represents two separate processes, one per policy. Both policy
runs produced the displayed identical measurement.

| Selector | Input triangles | Output V/T | Prepared input | Total live peak | Incremental kernel peak | Output-live payload | Input fact growth | Alloc calls | Realloc calls |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `boxes-3072` | 6,144 | 2,410 / 4,816 | 655,616 B | 16,462,358 B | 15,806,112 B | 520,472 B | 13,536 B | 251,626 | 32,892 |
| `boxes-3072-general` | 6,144 | 2,410 / 4,816 | 595,896 B | 16,462,366 B | 15,865,832 B | 520,472 B | 73,256 B | 252,377 | 32,892 |
| `yeahright` | 852 | 317 / 630 | 915,792 B | 5,970,009 B | 5,053,588 B | 232,536 B | 284,528 B | 453,382 | 22,241 |
| `yeahright-4` | 3,372 | 1,054 / 2,104 | 3,637,392 B | 22,489,145 B | 18,851,122 B | 571,168 B | 1,079,024 B | 1,623,390 | 70,301 |
| `yeahright-8` | 13,452 | 3,820 / 7,636 | 14,523,792 B | 85,635,443 B | 71,111,020 B | 1,512,560 B | 4,196,032 B | 6,296,027 | 250,529 |
| `yeahright-full-rotated` | 23,788 | 0 / 0 | 7,122,688 B | 165,385,450 B | 158,262,120 B | 56 B | 24,389,848 B | 39,528,604 | 4,148,399 |

The corresponding byte churn is 85,396,932/84,862,924 added/removed for the
native box, 86,866,020/86,272,292 for the general box, 28,204,955/27,687,891,
103,117,256/101,467,064, and 406,485,522/400,776,930 across the three YeahRight
scale rows, and 2,623,230,460/2,598,840,556 for full rotated YeahRight.

After output drop, the table's input-fact payload remains on the borrowed input
identities. After input drop, residual payload above the process baseline is
78,576 bytes for both box paths, 7,320 bytes for each control-hull scale row,
and 10,792 bytes for full rotated YeahRight. The large retained-fact rows are
therefore input-lifetime storage, not an unbounded process leak.

## Independent external-profiler agreement

The preceding checkpoint's sequential Heaptrack peaks were 16,497,378 bytes
for the native box and 16,497,738 bytes for the general box. The direct
requested-payload totals are lower by 35,020 bytes (0.2123%) and 35,372 bytes
(0.2144%). That narrow, expected difference is allocator/profiler overhead
outside the requested layout payload and validates the direct boundary.

The full hard case's 165,385,450-byte requested-payload peak is 157.72 MiB.
The preceding ordinary process row reached 192,376 KiB maximum RSS; executable
mapping, stacks, allocator arenas, and non-Rust process memory explain why RSS
is larger. Neither metric is substituted for the other.

## Scaling and retained-fact interpretation

The deterministic control-hull family scales input triangles approximately
4x at each step. Prepared input payload grows 3.972x then 3.993x; incremental
kernel peak grows 3.730x then 3.772x; input-attached fact growth grows 3.792x
then 3.889x. Incremental peak per input triangle falls from 5,931 to 5,590 to
5,286 bytes, so this family is close to linear and improves slightly with
scale. Larger dense/pathological siblings remain required.

Hyperreal's retained arithmetic facts are a performance asset and are not
classified as waste merely because their lifetime is now visible. In
particular, global removal or fixture-specific cache suppression would violate
the clean-algorithm and performance gates. The measured 23.26 MiB full-case
input-fact row instead gives future profile-led work a precise target: prefer
general fused construction and sign-only predicate schedules that avoid
materializing unused intermediate products, and retain facts whose repeated
consumer benefit wins end-to-end runtime without excessive lifetime storage.
Warm repeated-input measurements must accompany any retention-policy change.

## Validation and footprint

At implementation commit `6d0ad0311bb47f0a5faf7b770a896c136edab057`:

- 112 unit, 8 Boolean, 5 executed competitive, 7 manifest, 2 intersection, 9
  policy, and 2 README tests pass (145 executed total); six opt-in/manual tests
  remain ignored by the ordinary suite;
- both probe executables build in release mode;
- all twelve large/XL selector-policy executions complete with exact expected
  topology and `Certified` certainty;
- the allocator conservation assertions pass on every execution;
- all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
  formatting, and diff checks pass.

No production module, dependency, feature, or Cargo profile changes. The
ordinary and tracking probes share one fixture-preparation module, deleting 84
lines from the former probe rather than copying its fixture logic. The 386 net
new lines are measurement/example, manifest, documentation, and test code and
do not link into canonical native or WASM consumers.

## Open work

The whole Boolean kernel lifetime is now isolated from prepared inputs and
outputs on every manifested large/XL heap fixture. Phase 11 still needs broader
pathological/real-world and medium/large/XL siblings, and stage-specific arena
lifetime attribution remains useful. Phase 17 still needs profile-led reduction
of the 150.93 MiB full-case incremental peak and 23.26 MiB retained-input fact
row without sacrificing exactness or material runtime. CGAL per-case heap/RSS
parity and the Phase 18 completion audit remain open.

## Reproduction

```sh
cargo build --release \
  --example large_mesh_heap_probe \
  --example large_mesh_kernel_heap_probe

taskset -c 11 \
  target/release/examples/large_mesh_kernel_heap_probe \
  boxes-3072-general strict

taskset -c 11 env YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated approximate-512

heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe \
  boxes-3072-general strict

cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo fmt --all -- --check
```
