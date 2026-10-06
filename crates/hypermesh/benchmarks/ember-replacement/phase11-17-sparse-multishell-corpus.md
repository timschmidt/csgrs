# Phase 11/17 sparse multi-shell corpus checkpoint

Captured 2026-08-04 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hypermesh `c66aac67367eec607bfa03259b6a14118a40f27d`.

This checkpoint closes the first fixed-local-topology sparse multi-shell
scaling gap. It adds three exact-integer inputs whose operands each contain
8, 64, or 512 disconnected closed shells. Corresponding shells intersect in
the same way, while shells in different grid cells are exactly disjoint. The
family therefore grows component count and sparse broad-phase/assembly work
without changing the local Boolean, authored scalar class, or triangles per
component.

It also records exact both-policy oracles, reduced-rational CGAL EPECK inputs,
shared-arrangement and single-union runtimes, hardware-counter slopes, direct
kernel heap, fresh-process RSS, and canonical native/WASM size. It does not
claim Phase 11 corpus completion, Phase 17 performance completion, Phase 18
completion, or CGAL parity.

## Result

For every shell, the left operand contains a size-four tetrahedron at an
integer grid origin and the right operand contains the same tetrahedron
translated by `[1, 1, 1]`. Grid origins are spaced by eight in each axis. The
closest cross-cell pair therefore has an exact AABB gap of three. A complete
2x2x2, 4x4x4, or 8x8x8 grid supplies the three permanent members.

Each operand is one indexed `RawMesh`, not a batch of independent calls. It
contains four vertices and four triangles per shell. All coordinates are
small exact integers, reaching only 13, 29, and 61, so the family does not
conflate component growth with arbitrary-rational width or changing local
intersection topology.

There is no production-code, dependency, feature, or Cargo change. Fixture
selection exists only in deterministic corpus support, examples, tests, and
the manifest. The production Boolean engine does not inspect fixture identity,
component count, triangle count, coordinate magnitude, operation, policy,
expected result, competitor, or measurement state.

## Permanent exact and policy contract

| Shells per operand | Total input triangles | Shared result vertices | Union / intersection / difference / reverse / XOR triangles |
| ---: | ---: | ---: | --- |
| 8 | 64 | 88 | 128 / 32 / 96 / 64 / 160 |
| 64 | 512 | 704 | 1,024 / 256 / 768 / 512 / 1,280 |
| 512 | 4,096 | 5,632 | 8,192 / 2,048 / 6,144 / 4,096 / 10,240 |

One arrangement evaluates all five expressions. Their exact signed
six-volumes are `[127, 1, 63, 63, 126] * shell_count`. Every result has exact
directed-edge balance and the expected componentwise topology. `STRICT` and
`APPROXIMATE_512` return exactly equal batches at all three scales with
`Certified` certainty; no terminal 512-bit decision is consumed. This family
therefore tests policy propagation even though its rational predicates finish
before the terminal policy boundary.

The 8-shell member is in the shared Hypermesh/Boolmesh/Manifold-rust corpus and
the tri-mesh half-edge adapter suite. The 64- and 512-shell members remain
explicit scaling jobs so normal tests stay bounded. The 512-shell member has
one unique both-policy process/kernel heap selector.

## Identical exact CGAL inputs and outputs

`export_cgal_exact_off` writes reduced numerator/denominator tokens from
`Real::exact_rational`; it does not round through decimal text or binary64.
CGAL 6.0.3 EPECK therefore receives the same exact authored integers.

| Shells | Left SHA-256 | Right SHA-256 |
| ---: | --- | --- |
| 8 | `d65d0990dbf511a4c97f72ef46c77ad811acc2097f094d631145cd5d2ca3bcfc` | `1d8dc6bf249a6738f6ed7dacbad36f1c61aa775d12de8d6bd7caca3e97857e13` |
| 64 | `e1919b96595326b21f3cb2628fef97aa238fe0263a0e917bee2dd1ebd2290d03` | `4d6fbd9408e4c3f36fd011aec0513b1094b86e976cd5f918b34242c6a4dc74d8` |
| 512 | `b3115b50f2f23ad56513640d53bacd2d7e984e50d55db91df6f98a2bff1c572a` | `26e69cd18c3a613e8a86aeeb7f90cdda3a02e1803026cf1c85e675e4e45f9725` |

CGAL reports every union, intersection, difference, and reverse difference
valid, closed, and structurally valid at every scale. It reproduces the exact
triangle counts above and union vertex counts of 10 per shell. Its XOR adapter
requests both complementary differences; their combined triangle and volume
counts reproduce Hypermesh's XOR oracle.

## Shared-arrangement runtime versus CGAL EPECK

Both engines use one thread pinned to CPU 11. Each Hypermesh row is the median
of seven independent process aggregates after exact import and policy-qualified
PWN priming. Aggregates contain 50, 20, and 5 calls for 8, 64, and 512 shells.
Each call constructs one arrangement and returns union, intersection,
difference, and reverse difference together. CGAL is the median of 21 calls to
`corefine_and_compute_boolean_operations`; the table uses its required mutable
input copy outside the timed interval and also reports the copy-inside median.

| Shells | Hypermesh STRICT median (min-max) | Hypermesh APPROXIMATE_512 median (min-max) | CGAL outside median (min-max) | CGAL inside median | STRICT / CGAL outside |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 8 | 1.063 ms (1.052-1.176) | 1.057 ms (1.047-1.235) | 0.268 ms (0.259-0.409) | 0.282 ms | 3.97x |
| 64 | 8.473 ms (8.408-8.752) | 8.434 ms (8.361-8.541) | 2.010 ms (1.983-2.351) | 2.048 ms | 4.22x |
| 512 | 78.087 ms (76.781-83.489) | 78.303 ms (76.580-79.869) | 17.304 ms (16.870-18.978) | 17.520 ms | 4.51x |

Hypermesh loses every row. Its `STRICT` ratio worsens from 3.97x to 4.51x as
shells grow 64x. That is a Phase 17 failure gate, not a linear-scaling claim.
The two policies have identical work and outputs here; small timing differences
are measurement variation, not an approximate-policy fast path.

## Single-union runtime

A second fair boundary requests only union. Hypermesh again uses medians of
seven independent aggregates; CGAL uses 21 in-process calls with both copy
boundaries.

| Shells | Hypermesh STRICT | Hypermesh APPROXIMATE_512 | CGAL outside | CGAL inside | STRICT / CGAL outside |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 8 | 1.282 ms | 1.186 ms | 0.226 ms | 0.232 ms | 5.67x |
| 64 | 8.758 ms | 8.953 ms | 1.709 ms | 1.709 ms | 5.12x |
| 512 | 80.978 ms | 76.047 ms | 14.531 ms | 14.124 ms | 5.57x |

The union boundary confirms that shared-arrangement reporting is not hiding an
individual-operation loss. Hypermesh remains approximately 5.1-5.7x slower.
The nonmonotonic policy/copy ordering is retained as observed host variation;
exact topology and deterministic retired work are unchanged.

## Retired-work slope

Three `perf stat` repetitions run 100, 20, and 5 shared-arrangement calls per
process. Counts below divide the averaged whole-process total by calls. They
include one deterministic fixture construction, exact import, and PWN-prime
pass per process, amortized across the repeated production Boolean boundary;
they are not isolated Boolean-only counters.

| Shells | Instructions per arrangement | Branches per arrangement | Instructions per shell |
| ---: | ---: | ---: | ---: |
| 8 | 11.682M | 2.118M | 1.460M |
| 64 | 101.883M | 18.493M | 1.592M |
| 512 | 946.847M | 171.994M | 1.849M |

For each 8x component increase, instructions grow 8.721x and then 9.293x;
branches grow 8.732x and then 9.300x. Per-shell instructions rise 26.64% from
8 to 512 shells. This points to clean general targets in component-local
candidate scheduling, arrangement assembly, and retained exact ownership. It
does not justify a fixture-count branch or bypassing the complete arrangement.

## Direct 512-shell heap and process RSS

The allocator-instrumented probe wraps `System` only in the measurement
executable and counts successful requested payload. Input generation, exact
import, and policy-qualified PWN priming finish before the Boolean interval.
`STRICT` and `APPROXIMATE_512` produce byte-identical values:

| Metric | Bytes / calls |
| --- | ---: |
| Prepared input payload | 699,312 B |
| Incremental kernel peak | 15,017,410 B |
| Post-Boolean retained incremental | 1,077,200 B |
| Output-live payload | 1,032,248 B |
| Input-attached fact growth | 44,952 B |
| Post-input-drop incremental | 55,960 B |
| Allocation / deallocation / reallocation calls | 516,658 / 516,030 / 14,099 |
| Added / removed requested bytes | 53,812,724 / 52,735,524 B |

The peak is 3,666 requested bytes per input triangle or 29,331 bytes per shell.
Only 44,952 bytes remain attached as newly learned input facts, showing that
Hyperreal fact retention is compact here; the main memory target is temporary
arrangement ownership and allocation churn.

Fresh-process maximum RSS is 21,156/21,136 KiB for Hypermesh
`STRICT`/`APPROXIMATE_512`. CGAL union is 13,544 KiB with input copies outside
and 13,392 KiB with copies inside the timed interval. Against the copy-outside
row, Hypermesh uses about 1.56x RSS. RSS includes different exact carriers,
fixture front ends, allocators, and executable maps; the direct requested-
payload boundary above is the authoritative Hypermesh kernel measurement.

## Historical comparison

There is no historical EMBER measurement for this family because it was added
after the production cutover. No historical speedup is inferred and the
deleted engine is not rebuilt or linked to manufacture one. The established
overlapping-box and full-YeahRight historical scorecards remain pinned in the
earlier replacement evidence. This checkpoint adds a current competitive
shape and leaves every losing CGAL row open.

## Call graph, source, and linked footprint

The latest workspace call graph over Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh remains the direct-ordering graph: 14,846 nodes and
24,749 edges, with the canonical Hypertri-to-Hyperlimit Real-ordering edge and
no EMBER, compatibility, or alternate engine route. Hypercurve and HyperSolve
remain excluded. This checkpoint changes no production source, so it adds no
production graph node or edge; the fixture/test graph is not treated as
runtime reachability evidence.

The implementation commit adds 351 and removes 8 lines in corpus support,
examples, manifests, documentation, and tests. It changes zero production Rust
modules and no Cargo metadata. A fresh canonical size harness reproduces the
preceding checkpoint exactly:

| Profile / consumer | Native `.text` | `wasm-opt -Oz` |
| --- | ---: | ---: |
| release / general | 1,970,782 B | 1,394,510 B |
| release / immediate | 1,973,926 B | 1,396,354 B |
| size / general | 1,072,367 B | 668,271 B |
| size / immediate | 1,073,331 B | 668,678 B |

All eight file hashes are also byte-identical to the direct Real-ordering
checkpoint. The permanent test/source growth is accepted because it closes a
named corpus gap without entering canonical native or WASM consumers.

## Validation

Validation at `c66aac67` includes 138 unit, 8 Boolean, 9 executed competitive,
12 manifest, 3 intersection, 9 policy, and 2 README tests under all features
(181 executed, six documented opt-in/manual ignores). The default suite
executes 180 tests. It also includes no-default checking, warning-denied
all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
benchmark compilation, release probe/exporter/heap builds, both-policy large
heap execution, exact CGAL output validation including XOR, the complete
native/WASM size harness, formatting, and diff checks.

## Open work

Phase 11 still needs legally distributable external real-world pathologies,
deeper non-rational symbolic scaling, further adversarial/self-intersecting
scale families, and stage-specific arena attribution. This new family makes
the sparse component-scheduling deficit measurable: Phase 17 must reduce the
4.0-4.5x shared-arrangement runtime loss, 5.1-5.7x union loss, 1.56x RSS loss,
and superlinear retired-work slope with clean general scheduling and compact
ownership. Production specialization by fixture, component count, expected
topology, or competitor remains forbidden. Phase 18 still requires the final
path and exit audit.

## Reproduction

```sh
cargo build --locked --release \
  --example competitive_arrangement_probe \
  --example export_cgal_exact_off \
  --example large_mesh_heap_probe \
  --example large_mesh_kernel_heap_probe

target/release/examples/export_cgal_exact_off \
  sparse_multishell_tetrahedra_512 /tmp/hypermesh-sparse-multishell

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 5
taskset -c 11 target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  /tmp/hypermesh-sparse-multishell/sparse_multishell_tetrahedra_512-left.off \
  /tmp/hypermesh-sparse-multishell/sparse_multishell_tetrahedra_512-right.off \
  all 21 outside

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  sparse-shells-512 approximate-512
/usr/bin/time -v taskset -c 11 \
  target/release/examples/large_mesh_heap_probe sparse-shells-512 strict

taskset -c 11 perf stat -r 3 -x, -e instructions,branches \
  target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 5

cargo test --locked --all-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --all-features --no-run
cargo fmt --all -- --check
benchmarks/size-harness/measure.sh default
```
