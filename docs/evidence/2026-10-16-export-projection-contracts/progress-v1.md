# Exact export projections and consumable geometry

Status: **kept as a correctness change**. The full properties goal remains active.
The preceding goal turn made progress in c9410688b0c7854fba6c9a90f34cae5b3382318a
(native STL import validation). This patch is qualified against that revision
plus the same 35 in-scope native worktree files, whose bytes remain unchanged.
It does not admit those native changes or establish a clean published release.

CSGRS owns projection and decimal spelling. Hypermesh/Hyperlimit own the
nondegeneracy decision. The [contract](contract-v1.md) applies the existing
`properties.md` and `docs/admission.md`; this evidence does not replace them.

## Behavior and migration

OBJ and ASCII STL now write each chosen finite binary64 value as an exact
terminating decimal. The shared formatter preserves the old 17 fractional
places whenever those already represent the value exactly. Otherwise text,
lengths and checksums intentionally change. Binary64 1.1, for example, is not
the exact decimal `1.10000000000000009` emitted by the former formatter.
Consumers should regenerate affected golden text deliberately. ASCII normals
use the same formatter and preserve small nonzero computed components.

For a finite binary64 significand `m` and biased exponent `E`, removing the
trailing binary zeros leaves at most
`max(0, 1075 - max(E, 1) - trailing_zeros(m))` fractional decimal places.
Zero is handled separately. Since `n / 2^d = (n * 5^d) / 10^d`, that many
places suffice for an exact terminating decimal; binary64 bounds `d` at 1074.
Standard Rust formatting supplies the digits. No second scalar or geometric
kernel is introduced.

Binary STL now promotes its actual finite binary32 vertices exactly and calls
the existing strict native nondegeneracy predicate before publishing or caching
bytes. Coincident vertices and three distinct collinear projected vertices
produce `IoError::Geometry`; native predicate errors retain their cause through
`IoError::Construction`. Open nondegenerate surfaces remain supported. Valid
binary output bytes, normal calculation, and all five full-resolution example
outputs are unchanged in the recorded comparisons.

These are separate claims: exact decimal spelling of the chosen float,
nondegeneracy of an encoded triangle, and topology of a complete model.
Neither export certifies an analytic source or preserves an unrepresentable
native scalar. ASCII STL import still quantizes to binary32. Reimported geometry
can earn its own proof; it does not certify original intent or provenance.

## Executed validation

The unchanged original format probe improved from **9 passed / 3 failed** to
**12 passed / 0 failed**. The original topology probe remains **4 passed**.
The new public export matrix improved from **3 passed / 6 failed** to **9 passed**.
It covers ordinary 1.1, signed zero, extreme and fixed-seed finite bit patterns,
tiny text coordinates/normals, zero-area projection, binary32 subnormals,
independent binary bytes, cache eviction/mutation/recovery and downstream
strict composition. Native exact-value comparisons are supplemented by
independent JavaScript BigInt ratios and independent box boundary/volume checks.

Final native v2 ran 15 commands successfully: formatting, the focused matrix,
original probes, all-feature tests (**429 passed, one existing ignored README
image-render test**), STL-only tests (**151 passed**), OBJ-only tests
(**150 passed**), all five original basic_shapes outputs, four warning-denied
Clippy configurations and warning-denied all-feature documentation.
Counts overlap across configurations and focused suites.

The initial v1 suite passed semantics but failed two Clippy configurations on
`chunks_exact_to_as_chunks` in the new decoder test. The corrected fixed-size
slice iteration retains the exact length assertion, every other assertion and
all fixtures. Its fresh source/target and the original failed run are both kept.

Real generated WASM bindings passed the existing certainty consumer, the new
export consumer and locked TypeScript 5.7.3 checking. WASM-target Clippy passed
with warnings denied. The new consumer verifies exact decimal ratios, visible
failed exports, no returned geometry on failure, copied-buffer ownership,
eviction and strict composition of independently decoded tiny STL fields.
There is no JS STL importer: decoding plus `fromArrays` is identified explicitly.
The pinned wasm-bindgen CLI is 0.2.126; its installation reports third-party
future-incompatibility warnings for buf_redux 0.8.4 and multipart 0.18.0.
Those warnings are retained, separately from the warning-clean project checks.

Native v2 and WASM v1 use byte-identical production Rust, Cargo manifests and
dependencies. WASM v1 includes the final JS/harness and an uncompiled v1 native
integration test; native v2 compiles the corrected final test. The merged input
manifest and application record verify this partition and the applied bytes.

## Cost of the same complete outputs

[Protocol](cost-contract-v1.md), [raw samples](cost-v1/samples.json),
[results](cost-v1/result.json). Five alternating pairs for each of nine workloads
produced 90 successful processes and 180 first/warm observations. Every complete
input, serialized output and reimport row comparison matched byte for byte.
Independent integer box boundaries, orientation, per-component volume and native
topology checks qualify the outputs separately from byte agreement. Timings
include production validation and complete output materialization/retention;
construction and post-export oracle checks are outside timing.

The table reports candidate/baseline median paired ratios, retaining slower
results. First means an empty binary-output cache, after scalar construction.
Warm binary calls reuse the proven bytes. Small empty/cached timings are noisy;
this is an allocator-instrumented focused measurement, not an optimization or
whole-application speed claim.

| Encoding | Cubes | First ratio | Warm ratio | Candidate warm µs/export | Output bytes |
| --- | ---: | ---: | ---: | ---: | ---: |
| binary | 0 | 1.124305 | 1.004689 | 0.063 | 84 |
| binary | 1 | 1.637904 | 0.980650 | 0.451 | 684 |
| binary | 256 | 2.547992 | 1.002386 | 50.176 | 153684 |
| ascii | 0 | 1.057307 | 0.919132 | 0.183 | 25 |
| ascii | 1 | 1.113405 | 1.050585 | 27.433 | 4015 |
| ascii | 256 | 1.124661 | 1.000641 | 6298.521 | 1038565 |
| obj | 0 | 2.073006 | 1.032303 | 0.104 | 28 |
| obj | 1 | 1.272124 | 1.065370 | 19.848 | 2383 |
| obj | 256 | 0.994139 | 1.020214 | 4397.264 | 639637 |

For the first 256-cube binary export, the added proof requests 25,272 extra allocations and 1,213,056 extra cumulative bytes. Observed additional live peak remains 612,648 bytes and retained delta remains 416,040 bytes, equal to baseline in these samples. The measured first-call slowdown is retained openly as the cost of the stricter correctness contract.

Probe executable sizes: baseline 8,959,376 bytes, candidate 8,980,400 bytes.

All allocation calls, requested bytes, peak additional live bytes, retained
bytes, output capacities and process RSS remain in the raw records. Requested
allocation sizes do not measure allocator overhead or actual heap/RSS.
Correctness takes priority over the measured cost of the added projection proof.

## Absolute cost of newly correct fractional text

[Protocol](fractional-cost-contract-v1.md),
[raw samples](fractional-cost-v1/samples.json),
[results](fractional-cost-v1/result.json). Five candidate processes yielded 40
observations across four fixtures; all complete repeated outputs were identical.
The old lossy text is not counted as the same completed contract. This table
records absolute candidate warm costs and retained string sizes.

| Fixture | µs/export | Output bytes | String capacity | Allocations/export | Requested bytes/export |
| --- | ---: | ---: | ---: | ---: | ---: |
| ascii-fractional | 3.353 | 498 | 560 | 17 | 1944 |
| ascii-minimum-f32 | 3.266 | 489 | 688 | 19 | 2504 |
| obj-fractional-extremes | 135.035 | 3184 | 5010 | 26 | 19831 |
| obj-ordinary-cube | 8.996 | 1028 | 1536 | 52 | 5636 |

## Limits and retained work

Per-coordinate decimal precision is bounded; total output work and retained
memory are not internally budgeted. No public heap-byte, total-work or
cancellation contract was added. External elapsed/address-space containment
limits the experiments. The binary cache remains bounded by entry count only.
WASM still flattens typed IO errors into strings: diagnostics now survive the
new failure, but the typed error receipt remains an explicit P30 follow-up.

The 35 native worktree changes remain unadmitted. Their last reproduced original
full-board gate used 191.224524234 worker CPU seconds against 120, and copper-neck
used 86.843039677 wall seconds against 60. Neither those fixtures nor policies
were weakened. Clean external distribution, minimum compiler and the complete
pre-push workflow matrix remain open; no push or release occurred.

Completed sources, targets, tools, probes and measurements remain frozen in new
physical ext4 directories. The checkpoint adds per-path hashes while retaining
906112 inherited expectations and 27093 previously missing historical artifacts.
It does not re-audit that history or claim zero loss. Intentional package-manager
symlinks are pinned separately, and their targets are read per path. Only the
raw `current_work` ledger value changes; every other byte and decoded key stays
bound to revision 9025de62.

The next changes should preserve typed IO failure through the actual WASM
boundary and continue the remaining admission/release work. The full properties
goal remains active.
