# Explicit-policy baseline — 2026-07-30

This baseline covers Hypertri commit `214aaa0a6b6a`, with Hyperlimit commit
`972c4a66e53e`. It is the Phase 2 evidence for the workspace-root
`HYPERMESH_PATH_COMPLETE_IMPLEMENTATION_PLAN.md`. Machine-readable values are
in `policy-context-2026-07-30.toml`.

The host was an AMD Ryzen 7 5800X3D running Fedora kernel
`7.0.4-100.fc43.x86_64`, Rust 1.97.0, and LLVM 22.1.6. Runtime rows were
serialized and pinned to CPU 0.

## Policy and correctness

- Every public predicate-bearing operation requires `TriangulationContext`.
- `TriangulationContext` and `TriangulationCertainty` are each one byte.
- Earcut, 2D Delaunay, and the N-D oracle all reject a shared terminal-boundary
  fixture under `STRICT`; under `APPROXIMATE_512` they complete and report
  `Approximate512Consumed`.
- Exact-rational controls remain `Certified` under both policies.
- The all-feature suite passed 106 runtime tests and 4 doctests. Featureless
  tests, warning-denied Clippy/rustdoc, the standalone fuzz package, and the
  separate UI package also passed.

## Runtime against retained history

| Pinned Criterion row | Historical | Current | Change |
| --- | ---: | ---: | ---: |
| Exact rational-spike Earcut, runtime policy | 2.662 µs | 2.650 µs | 0.5% faster |
| Runtime-selected polygon triangulation | 2.650 µs | 2.711 µs | 2.3% slower; retain as watch row |
| 400-point located Delaunay | 39.026 ms | 17.575 ms | 55.0% faster |
| Small exact 4-D Delaunay oracle | 2.7994 ms | 22.287 µs | 99.2% faster |

The direct dynamic-policy row is the policy-plumbing control and remains inside
the 1% gate. The runtime-selection row includes fact inspection and dispatch,
showed wider run-to-run variance, and is retained rather than normalized away.

## Dependency-only artifact size

The checked-in `benchmarks/size-harness` materializes every selected algorithm
family and excludes Criterion, fuzzing, the UI, and `earcutr`.

### Speed-oriented release profile

| Features | Native file | Native `.text` | WASM raw | `wasm-opt -Oz` |
| --- | ---: | ---: | ---: | ---: |
| Core | 435,760 | 324,770 | 48,371 | 33,767 |
| Earcut | 1,483,592 | 1,076,750 | 872,382 | 645,828 |
| CDT | 1,458,672 | 1,057,018 | 850,088 | 637,276 |
| N-D | 1,332,352 | 941,120 | 709,834 | 533,900 |
| All algorithms + runtime selection | 1,663,568 | 1,240,786 | 1,058,780 | 747,934 |

### Size-oriented profile

| Features | Native file | Native `.text` | WASM raw | `wasm-opt -Oz` |
| --- | ---: | ---: | ---: | ---: |
| Core | 291,936 | 278,412 | 32,030 | 28,821 |
| Earcut | 855,680 | 634,995 | 385,207 | 327,442 |
| CDT | 835,992 | 616,859 | 370,825 | 313,922 |
| N-D | 773,224 | 555,629 | 304,272 | 257,387 |
| All algorithms + runtime selection | 922,552 | 699,483 | 445,799 | 379,670 |

## Runtime path evidence

`cargo bench --bench dispatch_trace --features
all-algorithms,runtime-select,dispatch-trace` passed under `STRICT`:

| Family | Dispatch events | Predicate events | Refinements |
| --- | ---: | ---: | ---: |
| Earcut | 80 | 51 | 0 |
| Delaunay | 209 | 58 | 0 |
| N-D Delaunay | 175 | 10 | 0 |
| Runtime selection | 73 | 51 | 0 |

The regenerated five-crate source graph has 18,481 nodes and 36,338 edges; the
all-evidence graph has 24,706 nodes and 46,024 edges. Hypertri has 25 direct
syntactic edges to 16 Hyperlimit targets. Every predicate target is the single
canonical policy-requiring API; no legacy `_with_policy` or policy-free edge
remains.
