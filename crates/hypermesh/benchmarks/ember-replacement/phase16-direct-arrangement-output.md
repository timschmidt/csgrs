# Phase 16 checkpoint: direct exact arrangement output

Captured 2026-08-03 at Hypermesh implementation
`7d9fac56f37d70893fbbc819560ca46ab3b779a8`.

## Scope and decision

Retain the first output stage of the replacement surface-arrangement engine.
One `ExactSurfaceArrangement` now selects cell-boundary facets for built-in or
arbitrary truth-table rows, compacts only referenced arrangement points, emits
deterministically ordered triangles plus source provenance, and certifies the
selected exact facets without using output repair, subdivision, local BSP, or
segment tracing.

The module remains `cfg(test)`. This is deliberately not the Phase 16 atomic
production cutover: EMBER is still shipped, the public multi-expression API is
not yet installed, and no caller has been migrated. There is no compatibility
wrapper, feature-selectable second production engine, or edge in either
direction between the replacement graph and the historical kernel.

## Output architecture

- Built-in union, intersection, left difference, and symmetric difference
  classifications are computed from the existing cell truth table. Arbitrary
  expression rows, including reverse difference, enter the same materializer.
- The first pass validates every classification and facet point, counts the
  exact output, and marks used points in a bit-packed `Vec<bool>`.
- A checked `u32` remap compacts only selected points. Triangles and
  `TriangleSource` rows are reserved exactly and written once.
- Facet contributions are already sorted by facet, source face, and
  orientation. The first contribution therefore supplies deterministic
  provenance; output orientation is the product of the cell classification
  and that source contribution's orientation.
- Contribution CSR offsets, source faces, orientations, compact remaps, and
  capacities are all validated before indexing or mutation can escape.
- Certification operates on the arrangement's original `Point3` values and
  compact `u32` facet IDs. It independently checks exact nondegeneracy,
  duplicate triangle geometry, directed edge balance, singleton boundaries,
  balanced nonmanifold PWN valence, checked multiplicity, and closure.
- No T-junction repair or retry is invoked. A malformed or open selection is a
  typed error rather than input to historical cleanup machinery.

The first implementation certified the newly cloned `OutputVertex` rows with
machine-word keys. Profiling showed that rebuilding three owned `Point3`
values per triangle was unnecessary. Certifying the already-owned arrangement
facets removes those clones, changes triangle/edge certificate keys from
`usize` to `u32`, and preserves the independent proof.

## Exactness and policy

Every topology and degeneracy decision still flows through the operation's
`DecisionContext` and Hyperlimit policy. Exact-rational fixtures remain
`Certified` under both `STRICT` and `APPROXIMATE_512`.

A terminal-equality degeneracy row uses `(pi + e)` and `(e + pi)` in distinct
expression trees. `STRICT` returns `PredicateUndecided`; `APPROXIMATE_512`
consumes Hyperlimit's terminal 512-bit equality, records
`Approximate512Consumed`, and then correctly rejects the approximately
degenerate triangle. The output layer neither introduces a terminal nor
relables approximate evidence as certified.

## Permanent cases exercised

The focused surface-arrangement suite now has 29 tests. New and extended rows
cover:

1. one transverse arrangement reused for all four built-in operations under
   both policies, with deterministic repeat output and positive exact signed
   volume;
2. disjoint intersection as a certified empty mesh;
3. edge-tangent shells as a balanced closed PWN with one nonmanifold edge;
4. seven arbitrary three-operand expression rows, including constant-empty,
   union, intersection, difference, parity, and a mixed expression;
5. reverse difference across a coincident interface;
6. one 40-operand arrangement materializing union, empty intersection, and
   parity without rebuilding intersections or corefinement;
7. a configurable large disconnected-shell row that now retains the
   arrangement while materializing all selected vertices, triangles, and
   provenance;
8. malformed classification dimensions/values, absent and degenerate points,
   empty/reversed/out-of-range contribution CSR, absent sources, invalid
   contribution orientation, duplicate facets, open selections, capacity and
   edge-multiplicity paths; and
9. strict versus approximate-512 output-degeneracy evidence.

The full validation matrix passes:

| Gate | Result |
| --- | ---: |
| Unit tests | 1,123 passed |
| Integration tests | 134 passed |
| Documented manual/benchmark ignores | 7 |
| Failures | 0 |
| Clippy, all targets/features, warnings denied | pass |
| Rustdoc, all features, warnings denied | pass |
| rustfmt and `git diff --check` | pass |

## Runtime

Callgrind 3.27.0 was pinned to CPU 11. A detached worktree at the committed
Phase 15 evidence revision was rebuilt against the same current Hyperreal,
Hyperlattice, Hyperlimit, and Hypertri checkouts and Rust 1.97.0. This is the
comparable A/B; the older 1,542,502,286-instruction Phase 15 recording predates
the current dependency/codegen state and is retained only as historical
evidence.

| Facets | Policies | Phase 15 test | Direct output test | Delta |
| ---: | ---: | ---: | ---: | ---: |
| 1,024 | 2 | 260,748,448 | 266,135,938 | +5,387,490 (+2.0662%) |
| 6,144 | 2 | 1,632,810,238 | 1,666,850,813 | +34,040,575 (+2.0848%) |

The complete candidate grows 6.2632x for 6x the facets, or 4.386% per-facet
growth. The isolated whole-test increment grows 6.3184x; its per-facet cost
rises 5.307%, remaining near-linear on this hierarchy-heavy fixture.

Callgrind attributes 23,015,654 inclusive instructions across both
materializations, 1,873.0 instructions per emitted triangle. Independent exact
certification accounts for 18,242,789 of those instructions, 1,484.6 per
triangle. Direct arrangement-facet certification lowers the initial candidate
from 1,679,730,462 to 1,666,850,813 instructions, removing 12,879,649
instructions (0.7668% of the whole test and 27.450% of the initial output
increment).

This disconnected tetrahedron family is a scaling and lifetime instrument, not
a shared CGAL contract fixture. It does not close any CGAL EPECK row. The
historical full-resolution and common-case competitive ledger remains open for
the production cutover and Phase 17.

## Large-fixture heap

Massif 3.27.0 used `--stacks=yes` on 1,536 disconnected shells (6,144 facets),
running both policies in one release test process.

| Contemporary A/B | Useful maximum | Total maximum |
| --- | ---: | ---: |
| Phase 15 worktree | 21,859,979 | 23,536,696 |
| Direct arrangement output | 21,859,979 | 23,534,896 |

The useful process maximum is byte-identical. The 1,800-byte total difference
is allocator/stack sampling noise and is not claimed as an improvement. The
detailed output-stage snapshot is 19,745,883 useful bytes (21,202,000 including
allocator and stack), below the earlier arrangement peak. Its tree exposes the
expected 884,736-byte exact output-vertex allocation for 6,144 retained
vertices. Thus retaining the arrangement during materialization adds a real
local lifetime but does not move the governing large-fixture heap maximum.

## Source, linked size, and call graph

The implementation commit adds 691 lines and removes 16, primarily permanent
path/failure/policy fixtures. Because `surface_arrangement` remains test-gated,
all eight canonical production artifacts are exactly unchanged from the shared
Phase 15 checkpoint:

| Consumer/profile | Native `.text` | `wasm-opt -Oz` |
| --- | ---: | ---: |
| General/release | 4,118,204 | 2,779,870 |
| Immediate/release | 4,151,428 | 2,794,424 |
| General/size | 1,899,546 | 1,189,866 |
| Immediate/size | 1,911,694 | 1,200,027 |

The five-crate source call graph has 20,964 nodes and 41,667 edges. It contains
655 surface-arrangement nodes and the unchanged 4,437 historical
subdivision/segment-trace/local-BSP nodes, with zero edge in either direction.
The six materializer/certifier nodes have 90 incident edges. This proves the
checkpoint extends one replacement orchestrator rather than calling into EMBER.

## Remaining Phase 16 work

- Move the complete orchestrator into production ownership and design the
  minimal shared multi-expression result/API.
- Preserve certainty and provenance through the public `MeshOutcome` boundary.
- Atomically route every general/specialization decline to the arrangement
  engine, migrate controlled callers, and remove all EMBER configuration and
  low-level exports without a shim.
- Delete subdivision, segment-trace, and local-BSP source/module registration
  in that same cutover.
- Run the full corpus, fuzz/sanitizers, controlled callers, dispatch trace,
  production large-mesh heap, source/native/WASM size, call graph, and pinned
  CGAL EPECK per-case protocol after the cutover.

## Reproduction

```sh
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo test --locked --release --all-features --no-run --lib
HYPERMESH_TOPOLOGY_SHELLS=1536 taskset -c 11 \
  target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::disconnected_shell_scaling_preserves_every_exact_component --exact
HYPERMESH_TOPOLOGY_SHELLS=1536 valgrind --tool=massif --stacks=yes \
  target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::disconnected_shell_scaling_preserves_every_exact_component --exact
HYPERMESH_TOPOLOGY_SHELLS=1536 taskset -c 11 \
  valgrind --tool=callgrind target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::disconnected_shell_scaling_preserves_every_exact_component --exact
cargo run --manifest-path ../tools/hyper-callgraph/Cargo.toml --release -- \
  --root .. --out-dir target/phase16-materialization-callgraph \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh --format json
./benchmarks/size-harness/measure.sh default
```
