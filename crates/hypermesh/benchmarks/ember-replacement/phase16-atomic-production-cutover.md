# Phase 16 checkpoint: atomic production cutover

Captured 2026-08-03 at Hypermesh implementation `ef431941` and controlled
CSGRS caller commit `b34a2f4`.

## Outcome

Hypermesh now has one production Boolean engine and one public entry point:

```text
boolean(&MeshContext, &[TriangleMeshRef], BooleanProgram)
  -> MeshOutcome<BooleanMeshBatch>
```

The call validates finite closed PWN input, constructs one shared exact
source-face arrangement and radial cell complex, evaluates either a built-in
operation or an arbitrary topologically ordered expression DAG, and directly
selects closure-certified boundary facets. Requested results share one compact
exact `Point3<Real>` arena; each result retains compact triangles, source
provenance, and whether its exterior cell is inside.

There is no old/new selector, feature flag, retry, deprecated forwarding API,
or compatibility shim. The operations, subdivision, segment-trace, local-BSP,
and old half-space modules are deleted. Raw borrowed views and native mesh
views enter the same arrangement engine. Native owners may cache only the
policy-qualified closed-PWN validation fact; convexity is not a trusted engine
selector.

## Exactness and output contract

- `hyperreal::Real` remains the scalar at every topology boundary.
- One `DecisionContext` carries `STRICT` or `APPROXIMATE_512` through input
  preparation, intersections, Hypertri, radial ordering, winding truth, and
  output certification.
- Approximate cached PWN evidence cannot answer a later strict operation. A
  strict recomputation can monotonically upgrade the immutable fact.
- Output is selected from cell truth and independently checked for valid
  indices, exact nondegeneracy, duplicate facets, directed edge balance,
  balanced nonmanifold multiplicity, and source provenance. Failed output is a
  typed error, never input to a repair pass.
- Exterior-containing expressions remain representable as oriented finite
  boundaries, but conversion to a finite `TriangleMesh` rejects them as
  unbounded rather than silently changing their meaning.

## Deletion and corpus accounting

The implementation checkpoint changes 59 files, adds 2,279 lines, and removes
83,934 lines, a net reduction of 81,655 lines. The deleted integration and
engine-control tests are not silently discarded:
`benchmarks/corpus/implementation-test-migration.toml` pins eight historical
sources at parent commit `f56371ec` by SHA-256 and test count. Its catch-all and
semantic mappings account for 1,113 removed tests and point to current public
behavior fixtures or stronger arrangement invariants. The corpus manifest test
rejects a missing source, mapping, current invariant, or permanent fixture.

## Current validation

| Gate | Result |
| --- | ---: |
| Unit tests | 107 passed |
| Integration tests | 29 passed |
| Documented external/manual ignores | 6 |
| Failures | 0 |
| Main all-target/all-feature Clippy, warnings denied | pass |
| Rustdoc, warnings denied | pass |
| Fuzz binaries, size harness, and UI Clippy, warnings denied | pass |
| rustfmt and `git diff --check` | pass |

Hypermesh examples, benches, fuzz targets, size harness, and UI compile against
the canonical API. CSGRS consumes it directly at `b34a2f4` and contains no
removed name. A complete CSGRS check currently stops at concurrent Hypercurve
signature changes in `src/curve/native.rs`; it emits no Hypermesh API error.
Hypercurve and HyperSolve were not edited.

## Call graph

The workspace utility was regenerated over Hypermesh, Hyperreal, Hyperlimit,
Hyperlattice, Hypertri, and CSGRS:

| Graph | Nodes | Edges | Hypermesh functions | Hypermesh internal edges | Removed namespace nodes |
| --- | ---: | ---: | ---: | ---: | ---: |
| Production | 17,932 | 29,377 | 2,573 | 4,063 | 0 |
| Tests/benches/examples/fuzz | 25,022 | 39,535 | 3,470 | 5,126 | 0 |

The production `boolean` node calls source preparation, program validation,
surface-arrangement construction, and shared result materialization directly.
Historical evidence documents still mention removed names by design; current
source and graph namespaces do not.

## Still open

This checkpoint establishes the implementation cutover, not the Phase 17/18
finish. Current production large-mesh heap and retained-lifetime rows under
both policies, dispatch/runtime coverage, rustdoc API coverage, fuzz runtime
and sanitizers, native/WASM size, expanded pathological fixtures, and the
per-case historical/CGAL EPECK campaign remain required. CSGRS also needs a
complete validation rerun after its concurrent Hypercurve work stabilizes.

## Reproduction

```sh
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps
cargo clippy --manifest-path fuzz/Cargo.toml --bins -- -D warnings
cargo clippy --manifest-path benchmarks/size-harness/Cargo.toml --all-targets -- -D warnings
cargo clippy --manifest-path examples/hypermesh_ui/Cargo.toml --all-targets -- -D warnings
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir target/hypermesh-path-callgraph-phase16-cutover-production \
  --crate-name hypermesh,hyperreal,hyperlimit,hyperlattice,hypertri,csgrs \
  --format json,dot --per-library
```
