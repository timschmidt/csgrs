# Surface-arrangement replacement Phase 10 inventory

Status: in progress

Source checkpoint: `f57526e05d9bad63f481695c289d2d743fd13d73`

The machine-readable companion is `phase10-inventory.toml`. This checkpoint
freezes the replacement boundary before the new engine changes production
behavior.

## Contract

The current crate contract is finite, closed, piecewise-winding-number triangle
meshes over canonical `hyperreal::Real` coordinates. Degenerate triangles,
empty/open meshes, invalid indices, and non-PWN directed boundaries are rejected
before the general Boolean path. `STRICT` consumes certified Hyperlimit
decisions only. `APPROXIMATE_512` alone may consume Hyperlimit's terminal
512-bit decision, and the operation returns that consumption through aggregate
`MeshCertainty`.

The replacement preserves this contract first. Any later expansion to exact
self-intersecting or nonmanifold PWN input is additive and must have an explicit
semantic fixture matrix. It cannot weaken or silently reinterpret an existing
input path.

## Atomic replacement boundary

`tools/hyper-callgraph` found 21,266 production-scan nodes and 41,499 edges
across Hypermesh, Hyperreal, Hyperlimit, Hyperlattice, Hypertri, and Hypervoxel.
Only one call-graph edge enters the historical implementation from outside its
modules:

```text
hypermesh::operations::compute_boolean
  -> hypermesh::subdivision::subdivide_boolean_with_certified_convex_inputs
```

The evidence scan, including tests, benches, examples, and fuzz targets, has
28,062 nodes and 52,049 edges. Its numerous low-level callers are Hypermesh
implementation tests and diagnostic tooling, not independent production
engines. This supports one atomic top-level cutover rather than capability
routing between two general engines.

Outside Hypermesh, the only discovered controlled production use of the EMBER
API is CSGRS passing `EmberConfig::default()` to
`boolean_triangle_meshes`. Hypervoxel uses only `polygon_soup` validation.
Hypercurve and HyperSolve were excluded from searches and remain untouched.

## Deletion baseline

The historical general machinery occupies 30,578 physical production lines:

| Family | Physical lines |
| --- | ---: |
| Subdivision and split | 13,537 |
| Segment trace production modules | 16,400 |
| Local BSP | 641 |

Its two dedicated implementation-test files add 31,599 lines. Those test lines
are not disposable coverage: Phase 11 maps behavior into the permanent fixture
corpus before implementation-specific tests are removed.

## Reproducible evidence commands

Production call graph:

```sh
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-ember-replacement-callgraph-production \
  --crate-name hypermesh,hyperreal,hyperlimit,hyperlattice,hypertri,hypervoxel \
  --format json,dot \
  --per-library
```

Evidence call graph:

```sh
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-ember-replacement-callgraph-evidence \
  --crate-name hypermesh,hyperreal,hyperlimit,hyperlattice,hypertri,hypervoxel \
  --include-tests \
  --include-bench \
  --include-examples \
  --include-fuzz \
  --format json \
  --per-library
```

The current unit baseline is 1,068 passed tests from `cargo test --lib` in
2.00 seconds. Full integration, feature, fuzz, caller, size, heap, and
competitive matrices remain required before Phase 10 closes.

## Competitive pin

The installed competitive implementation is CGAL 6.0.3 at git hash
`cefe3007d59cf4292a09da4fa8a35f38478a4e0b`, using
`Exact_predicates_exact_constructions_kernel` and GCC 15.2.1. The dedicated
adapter under `competitive/cgal-epeck` accepts exact rational OFF input and
emits one raw JSON record per repetition. Hypermesh and CGAL consume the same
fixture files; fixture import, input copies, Boolean work, output validation,
and process RSS can therefore be isolated rather than inferred.

The starting full-resolution hard row remains 3,312.66 seconds and 329,352 KiB
maximum RSS for Hypermesh versus approximately 0.09 seconds and 15,516 KiB for
CGAL EPECK. This is an open 36,807x runtime and 21.23x RSS deficit.
