# Path-complete baseline — 2026-07-30

This is the starting evidence for the workspace-root
`HYPERMESH_PATH_COMPLETE_IMPLEMENTATION_PLAN.md`. Machine-readable values are
in `path-complete-2026-07-30.toml`.

The host was an AMD Ryzen 7 5800X3D running Fedora's
`7.0.4-100.fc43.x86_64` kernel, Rust 1.97.0, and LLVM 22.1.6. Runtime
benchmarks were serialized and pinned to CPU 0.

## Workspace identity

| Crate | Revision |
| --- | --- |
| Hyperreal | `4ebfa43f8b48` |
| Hyperlattice | `91d300239500` |
| Hyperlimit | `681096cabf9a` |
| Hypertri | `67d106bd7704` |
| Hypermesh | `1314d54714fd` |

The workspace contained pre-existing tracked edits in Hyperlattice and
Hypertri. Their tracked-diff SHA-256 values were respectively
`067a31c0fc19b1221e317290d0eb1a45e4b39030b4f937e1f7322c4aa3746fcf`
and
`170c6c31c013a84873203d278166b7991831d0e0d0d840273ca644ab44d352b8`.
Comparisons must use these fingerprints or explicitly rebaseline.

## Canonical artifact size

`benchmarks/size-harness` is a dependency-only consumer. It selects the Boolean
operation at runtime and materializes certified triangle output, without
linking Hypermesh's benchmarks, fuzzing support, UI, or competitors.

### Speed-oriented release profile

| Features | Target | Raw bytes | Text bytes | Gzip -9 | Brotli 11 | `wasm-opt -Oz` |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Default | Native | 3,923,824 | 3,345,473 | 1,557,668 | 1,161,694 | — |
| All | Native | 4,053,088 | 3,470,590 | 1,608,895 | 1,201,615 | — |
| Default | WASM | 2,919,617 | — | 859,383 | 580,299 | 2,229,994 |
| All | WASM | 3,011,956 | — | 878,064 | 592,695 | 2,297,018 |

All features add 3.294% native file bytes, 3.740% native text, 3.163% raw
WASM, and 3.006% optimized WASM.

### Size-oriented profile

| Features | Target | Raw bytes | Text bytes | Gzip -9 | Brotli 11 | `wasm-opt -Oz` |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Default | Native | 1,699,872 | 1,467,875 | 694,445 | 568,073 | — |
| All | Native | 1,700,096 | 1,468,051 | 694,441 | 567,954 | — |
| Default | WASM | 1,102,459 | — | 395,734 | 307,157 | 929,484 |
| All | WASM | 1,100,662 | — | 395,739 | 306,737 | 927,046 |

At `opt-level = "z"` with fat LTO and stripping, the all-feature diagnostics
are almost entirely eliminated for this consumer. Small negative compressed
WASM deltas are alignment/compression effects, not evidence of a semantic
improvement.

Reproduce with:

```sh
./benchmarks/size-harness/measure.sh default
./benchmarks/size-harness/measure.sh all
```

The harness built directly for `wasm32-unknown-unknown`; the prior Hypermesh
example failure was caused by its dev-dependency graph's `getrandom`
configuration, not the core Hyper stack.

## Current direct runtime

| Workload | Median |
| --- | ---: |
| Certified-convex overlapping-box union, immediate mesh | 618.60 µs |
| boolmesh equivalent row | 64.777 µs |
| manifold-rust equivalent row | 62.279 µs |
| General cube union, polygon result | 2.3912 ms |
| General 192-triangle-per-input subdivided-cube union | 80.019 ms |

Hypermesh is 9.55× slower than boolmesh and 9.93× slower than manifold-rust on
the selected row. Those competitors are throughput references, not exactness
oracles.

## Historical memory gate

The retained 1,140-facet YeahRight arrangement gate is:

- 944.8 ms median;
- no more than 82.5 MiB peak RSS;
- 67.74 MiB peak heap;
- 5,020,891 allocations;
- 5,152 output polygons; and
- checksum 675,298,388.

The full-resolution 11,894 × 11,894 triangle Boolean previously approached
116 GiB RSS and remains a required optimization target.

## Graph and evidence

The five-crate source graph had 18,591 syntactic nodes and 36,384 edges. The
all-source-class graph had 24,793 nodes and 46,039 edges.

Fresh rustdoc evidence found 333 Hypermesh public items and 162 callables. Only
21 callables had qualified test evidence, 7 benchmark evidence, 7 fuzz
evidence, 16 dispatch-trace evidence, and 4 qualified evidence in all four
classes. These static counts are navigation evidence and must be confirmed by
runtime paths.
