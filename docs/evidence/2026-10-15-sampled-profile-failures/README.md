# Sampled-profile failures and composition

This slice applies `properties.md` and `docs/admission.md`, chiefly P1, P4,
P6, P8–P9, P17–P20, P25–P30 and P35–P39. CSGRS owns sampled-profile parameter
validation and construction facts; Hypercurve owns topology. The public Rust
constructors, teardrop solid composition, WASM, examples, benchmarks and fuzz
target share the same implementation. The previous goal turn made progress in
`d41fc3601`, `573bdcb8f` and `2271ad6c5`. The full plan remains active.

The migration covers arrow, trapezoid, star, teardrop, egg, squircle, pie_slice,
heart and supershape. Preserve existing positive dimensions and range/sample
restrictions, the full README and benchmark resolutions, and successful exact
sample rows. Primary APIs return `Result<CurveRegion2, ValidationError>`;
context variants retain the consumed certainty. Do not publish scalar, predicate,
capacity or native failure as empty success. Teardrop-cylinder composition must
propagate construction errors and join its certainty with extrusion certainty.
The four WASM constructors retain their signatures and expose typed error codes.

Equal-width positive arrows have duplicate shoulder endpoints in the old sampler;
clamp-free equality evidence permits omitting only those duplicates. The intended
shaft-plus-head area must survive. Other successful sample formulas remain
unchanged. These are sampled sets, with no analytic-fidelity or end-to-end source
provenance claim. Precision, total work, scalar growth, heap bytes and cancellation
remain different contracts: only primary buffer capacity/reservation is checked
here. Large trials use external wall/address-space containment.

## Preserved evidence

- `baseline-v1`, based on `2271ad6c5`: nine full README fixtures pass and fifteen
  assertions fail. Fourteen are actual error/completeness defects. One oracle
  incorrectly demanded an error from the valid collinear one-chord semicircle.
- `candidate-v1`: 23 of 24 tests pass. Investigation establishes that Hypercurve
  successfully regularizes that collinear set to empty; it is not an error
  swallowed by a wrapper. Its assertion is corrected to require Certified
  empty geometry and downstream union/extrusion composition. No production
  behavior is weakened to hide a native failure; the original failed oracle
  and trial remain preserved.
- `candidate-v2`: all 30 expanded tests and the existing all-feature library
  tests pass. Independent rational/algebraic areas cover every migrated family.
  Exact volume 9 and a separate manifold check cover an arrow with a hole and
  teardrop extrusion; the dyadic arrow survives OBJ value-for-value. Approximate
  arrow shoulder equality remains qualified even when a strict identical ring
  is cached. Every input-controlled primary buffer rejects capacity overflow.
- All nine original README fixtures preserve their exact serialized coordinate
  rows byte-for-byte. The serializers retain exact scalar expressions and omit
  accelerator caches. This comparison supplements independent semantic oracles;
  matching rows/checksums alone would not establish the intended set.

`compiler-consolidation-v1` shares 19,367,511,727 bytes of identical non-executable
compiler artifacts in terminal trials, retaining every path and byte and checking
all 2,485 protected artifacts before and after. Executables, unique outputs,
source snapshots and logs remain untouched. These closed targets must never be
resumed with Cargo; future trials copy seeds with separate inodes.

## Final qualification

Implementation commit `fcb269765` changes the nine primary Rust signatures to
`Result` and adds context variants. Callers must handle typed errors. The four
WASM signatures stay the same and now throw `GeometryError` with a diagnostic
`code`. Equal-width arrows are the explicit geometry correction: five distinct
boundary vertices retain the shaft and head instead of failure becoming empty.

`native-v1` passes all 13 commands: root/fuzz formatting; all-target strict Clippy
for all features, minimal and curve-only configurations; library/integration
tests; fuzz-target Clippy; warning-free docs/doctests; all nine original benchmark
consumers; and the full renderer build. All-feature tests pass 293 with one
pre-existing full image-corpus test ignored. Minimal tests pass 38, curve-only
108, and one doctest passes: 440 passing executions across overlapping feature
configurations. Both `wasm-v1` commands pass: target/profile Clippy and the actual
WASM build with Node assertions and TypeScript checking. New binding checks
cover 13 rejection cases and four successful recovery/classification/extrusion
fixtures. `boundary-toolchain-v1.json` retains actual tool versions and Node
package inputs. There is no direct C/C++/Python/Go exposure for these constructors.

Final native and WASM source hashes agree. Nine original full README fixtures
retain all 729 exact coordinate pairs across baseline, both candidates and final
native qualification. No fixture resolution, policy or renderer formula changed.

## Focused costs on completed outputs

`cost-v1` is retained as a failed harness build: the independent sector-area
expression borrowed and moved the same scalar (`E0505`). `cost-v2` borrows both
factors. No production source, workload, oracle or timing boundary changed.

All 128 commands in `cost-v2` pass, including 108 fresh-process timing samples:
three repetitions of nine constructors, warm and ring-FIFO-miss sequences, on
both revisions. Every sample completes 128 strict operations (13,824 timed
operations in total), each with an independent exact-area proof, materialized
hole-free boundary, exact-expression serialization, checksum and output drop.
All 128 serialized variants for each family match byte-for-byte between revisions
(1,152 variants per revision); every paired timing checksum/output size agrees.
The original full README and benchmark resolutions are separately preserved.

The table reports medians per completed operation. Ratios above one are slower.
These are three instrumented samples on a shared, unpinned host, with unchanged
locks/toolchain recorded in the protocol; they do not establish a broad speedup.
Warm sequences repeat one profile. Miss sequences cycle 128 dimension variants
to exceed the 64-entry ring FIFO, without claiming globally cold scalar caches.

| Profile | Cache | Baseline µs | Candidate µs | Time ratio | Allocations baseline → candidate |
| --- | --- | ---: | ---: | ---: | ---: |
| arrow | warm | 12.798 | 12.105 | 0.946 | 87 → 88 |
| arrow | miss | 117.207 | 117.215 | 1.000 | 822.312 → 823.312 |
| trapezoid | warm | 6.775 | 6.788 | 1.002 | 48 → 48 |
| trapezoid | miss | 86.823 | 80.600 | 0.928 | 512.602 → 512.602 |
| star | warm | 107.122 | 100.244 | 0.936 | 1886 → 1885 |
| star | miss | 657.583 | 664.279 | 1.010 | 11400.6 → 11399.6 |
| teardrop | warm | 9.705 | 8.848 | 0.912 | 92 → 92 |
| teardrop | miss | 74.089 | 79.131 | 1.068 | 533.062 → 533.062 |
| egg | warm | 12.678 | 14.625 | 1.154 | 152 → 152 |
| egg | miss | 89.046 | 83.296 | 0.935 | 637.898 → 637.898 |
| squircle | warm | 11.556 | 11.822 | 1.023 | 119 → 119 |
| squircle | miss | 76.104 | 84.556 | 1.111 | 606.125 → 606.125 |
| sector | warm | 8.741 | 7.063 | 0.808 | 90 → 89 |
| sector | miss | 59.030 | 64.352 | 1.090 | 468.172 → 467.172 |
| heart | warm | 350.918 | 338.192 | 0.964 | 5853 → 5848 |
| heart | miss | 1808.040 | 1785.632 | 0.988 | 28067.2 → 28062.2 |
| supershape | warm | 17.007 | 18.316 | 1.077 | 238 → 238 |
| supershape | miss | 97.350 | 94.192 | 0.968 | 958.492 → 958.492 |

Unfavorable results remain: warm egg median is 15.4% higher and miss squircle
11.1% higher. Arrow adds one Rust allocation and 672 requested bytes per
operation. Star removes one allocation/384 requested bytes, sector one/192,
and heart five/544; other measured allocation counts match. Retained-byte
deltas agree between revisions. Arrow's miss-sequence peak rises by 672 bytes;
sector's falls by 96. These allocator counts exclude native allocations and do
not establish a heap bound. All raw samples and process resource reports remain.

The focused harness executable changes from 35,365,056 to 35,344,888 bytes;
all-feature `libcsgrs.rlib` from 10,020,198 to 9,869,558 bytes. These are artifacts
of this API/correctness migration, not a performance-only claim. Closure also
records the three production source-file sizes. Exactness and failure visibility
justify the change despite the measured costs.

## Preservation and open work

`source-input-migrations-v1.json` records one earlier evidence-input migration:
the prior renderer hashed live `examples/readme_renders.rs`. Its original bytes
are archived under `prior-inputs/`; the live caller now handles `Result` without
changing inputs. The prior closure and all historical records remain untouched.

`close_v1.py` verifies phase streams, qualified source snapshots, old and new
artifacts, exact outputs, compiler-preservation manifests and the 35 pending
native files. It changes only `current_work` in `docs/properties-status.json`;
all other decoded keys and raw bytes outside that value remain fixed. The
baseline evidence is committed as `1916037c7`; the implementation is `fcb269765`.

The original 35 pending native files remain byte-identical and uncommitted,
awaiting separate admission. Gear, rack, airfoil, text and other legacy paths
still need failure migration. Source provenance, analytic fidelity, internal
work/heap/cancellation budgets, full CI/pre-push, external-package/MSRV/release,
directed fuzz/sanitizers, complete image-corpus validation and the remaining
properties plan stay open. This is worktree qualification. No push or CI edits.
