# Phase 11/17 thin-dyadic near-degenerate corpus

Date: 2026-08-05

Status: accepted corpus checkpoint; Phases 11, 17, and 18 remain open

Implementation: Hypermesh `8b8f8509`

Direct parent/evidence: Hypermesh `d5093a9d`

## Result

The permanent corpus now contains a fixed-topology exact family for the
near-degenerate and extreme-exponent cells that were not represented by the
well-conditioned wide-rational similarity family. The three new large cases
apply

```text
(x, y, z) -> (x + z, y, z / 2^shift)
```

to the same 6,144-triangle overlapping-box surface grid at shifts 64, 512, and
2,048. The affine determinant is exactly `2^-shift`. Connectivity and Boolean
truth remain fixed while the smallest geometric singular value decreases with
the exact dyadic exponent. At shift 2,048 the thin coordinate's binary64 value
is zero, but the `hyperreal::Real` coordinate remains an exact nonzero rational.

This raises the monotonic fixture registry from 46 to 49 cases and the set of
distinct large-mesh heap selectors from 15 to 18. The new tags are
`exact-affine-embedding`, `near-degenerate`, `thin-shell`, `sliver-triangles`,
and `extreme-exponent`.

The generator, exact-OFF exporter, competitive probe, and heap probes share one
case constructor. Overlapping-box generation was also factored out directly,
so exact/scaling probes no longer construct and discard the rest of the corpus
merely to select its first case. Six repeated five-output test programs were
collapsed into one constant expression DAG. These are support/test
consolidations; no production module, Cargo feature, dependency, Boolean
dispatch, or scalar implementation changed.

## Exactness, topology, and policy

The small correctness sibling uses 96 total input triangles and evaluates
union, intersection, both differences, and XOR under both policies. For every
shift it proves:

- aggregate certainty is `Certified` under `STRICT` and
  `APPROXIMATE_512`; no terminal approximation is consumed;
- directed output boundaries are balanced;
- exact six-volumes are `[504, 72, 312, 120, 432] * 2^-shift`;
- the exact inverse map `(x', y', z') -> (x' - z'/scale, y', z'/scale)`
  recovers byte-equal rational vertex rows across all three shifts;
- all five oriented triangle arrays are identical across shifts; and
- both policies produce byte-identical complete batches.

Every large member has 6,144 input triangles and produces the same 2,410
vertices and 4,816 union triangles. The 2,048-bit result's exact volume remains
`84 / 2^2048`; no floating volume or epsilon is a correctness oracle.

There is no fixture recognition in the production engine. The only selectors
live in development probes and choose input data, policy, and operation. All
three cases enter the same public Boolean arrangement route as every other
fixture.

## Deterministic scaling

CPU-11-pinned `perf stat -r 3` wraps a fresh release probe process and one
union. Setup is included. The policy rows are effectively identical in retired
work and all results remain `Certified`.

| Shift / policy | Instructions | Branches | Cycles |
| --- | ---: | ---: | ---: |
| 64 / `STRICT` | 1,194,400,383 | 205,221,436 | 457,250,995 |
| 64 / `APPROXIMATE_512` | 1,194,305,160 | 205,197,738 | 465,779,452 |
| 512 / `STRICT` | 3,429,452,625 | 590,847,897 | 1,118,393,189 |
| 512 / `APPROXIMATE_512` | 3,429,367,787 | 590,826,709 | 1,129,486,526 |
| 2,048 / `STRICT` | 2,330,629,672 | 445,770,817 | 870,224,758 |
| 2,048 / `APPROXIMATE_512` | 2,330,650,122 | 445,775,854 | 871,309,014 |

The 512-bit row executes 2.871x the 64-bit instructions, while the 2,048-bit
row executes only 1.951x the 64-bit work and 0.680x the 512-bit work. That
nonmonotonic boundary is real and useful: Hyperreal's retained dyadic/general
exact schedules have different cost shapes. It is the next general scalar and
predicate optimization target, not a reason to remove or relabel a corpus
point.

## Large-fixture heap and RSS

Each new large selector ran in a fresh instrumented process under both
policies. Within each shift, the two policy rows are byte-identical for input,
peak, allocation/reallocation counts, cumulative bytes, output, and certainty.

| Shift | Input payload | Total peak | Incremental kernel peak | Calls | Reallocations | Added bytes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 795,560 | 19,495,340 | 18,699,146 | 206,411 | 572 | 24,443,798 |
| 512 | 798,040 | 23,610,989 | 22,812,314 | 1,043,381 | 9,236 | 113,027,238 |
| 2,048 | 695,152 | 38,676,638 | 37,980,850 | 1,037,872 | 9,213 | 356,577,310 |

Every output-live payload is 520,472 bytes. Input-attached fact growth is
27,736, 28,240, and 139,520 bytes. The requested-payload peak grows 1.220x at
512 bits and 2.031x at 2,048 bits relative to 64 bits; cumulative traffic grows
much faster and remains open.

Fresh-process maximum RSS was:

| Shift | Hypermesh `STRICT` / `APPROXIMATE_512` | CGAL EPECK | Hypermesh / CGAL (`STRICT`) |
| --- | ---: | ---: | ---: |
| 64 | 25,204 / 25,232 KiB | 9,948 KiB | 2.534x |
| 512 | 29,328 / 29,160 KiB | 10,016 KiB | 2.928x |
| 2,048 | 43,944 / 43,748 KiB | 10,912 KiB | 4.027x |

All three RSS gaps remain open. The 2,048-bit runtime win below does not close
its memory loss.

## Pinned CGAL EPECK boundary

The exporter writes every coordinate as a reduced exact rational. CGAL 6.0.3
EPECK reports the same 2,410 vertices and 4,816 triangles, with valid, closed,
structurally valid output on every repetition. Its binary64 diagnostic volume
underflows to zero at shift 2,048; that diagnostic is not used as an oracle.

CGAL used 21 internal repetitions for each copy mode. Hypermesh used 21
repetitions over one retained input pair. Host frequency changed during the
series, so raw wall windows are advisory and no favorable subset is selected.

| Shift | CGAL outside / inside median | Hypermesh `STRICT` observed mean | Hypermesh `APPROXIMATE_512` observed mean | Per-case result |
| --- | ---: | ---: | ---: | --- |
| 64 | 5.793 / 5.851 ms | 100.44–102.94 ms | 100.91–184.94 ms | Hypermesh loss |
| 512 | 6.295 / 6.483 ms | 258.56–370.05 ms | 257.50 ms | Hypermesh loss |
| 2,048 | 871.742 / 875.465 ms | 199.36–200.38 ms | 199.09 ms | Hypermesh 4.35–4.38x win |

To avoid depending on frequency state, both engines were also measured as
complete 21-operation processes with retired work. Exact input generation or
OFF parsing is included once and therefore amortized over the 21 unions.

| Shift | Hypermesh instructions / branches / cycles | CGAL instructions / branches / cycles | Deterministic result |
| --- | ---: | ---: | --- |
| 64 | 23.707B / 4.077B / 9.126B | 4.354B / 0.821B / 1.693B | Hypermesh loses 5.444x / 4.966x / 5.389x |
| 512 | 70.639B / 12.177B / 32.046B | 4.855B / 0.894B / 1.894B | Hypermesh loses 14.550x / 13.623x / 16.921x |
| 2,048 | 47.868B / 9.179B / 17.706B | 244.171B / 38.680B / 80.126B | Hypermesh wins 5.101x / 4.214x / 4.525x |

This is a per-case Hyperreal-strength result, not an aggregate claim. The
64/512 runtime losses and all three RSS losses stay in the open ledger.

## Profile boundary

A 2,644-sample 512-bit profile records 14.86% self cycles in Hyperreal's
four-product/two-side signed ordering, 10.76% in exact-rational coordinate
classification, 6.39% in lossy rational export, 5.24% in `memmove`, 4.40% in
four-value rational linear-form normalization, and 3.90% in point-query
construction. The comparable 2,048-bit profile falls to 6.62%, 7.46%, 3.16%,
3.84%, below 0.5%, and 1.39% respectively; shifted-BigUint comparison and
trailing-zero discovery become its largest self costs.

Artifacts are `target/phase11-17-thin-dyadic-{512,2048}.data`. The next pass
should inspect the clean fixed-wide dyadic ordering/normalization boundary and
reuse Hyperreal's retained exponent/content facts. It must preserve the
complete arbitrary-rational and symbolic fallbacks and win broad existing
controls; no exponent-512 fixture branch is acceptable.

## Exact-OFF identity

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| 64 left | 88,504 | `5e82ea3d5abd60ab58efe7991afecf5c004e6e5f7140a7d5c7a56533618f5b7c` |
| 64 right | 95,107 | `f2d5b83d5ba1c8a7d4f4e7f8bd5a9620f63f52727b17246f2bc74f9e3e5ce9bd` |
| 512 left | 257,119 | `ca31f8fc287ab7f2b4110faa47dd23289088dd4bb167611f0af4633cae162702` |
| 512 right | 302,737 | `957f7168683832d581eaf750226890c1c551f4d7a5f7d3e15ed83bee8b366766` |
| 2,048 left | 834,733 | `88728f4fc20d7dba600721a192b0715c90137c3b3f15f726a1254705b069d75a` |
| 2,048 right | 1,013,869 | `ff43d7fc5107e643cdbb61bb5fea5c66cbff51b3d2e9691571b609dc18135b26` |

## Source, binary size, and call graph

The implementation changes only corpus metadata, competitive/support code,
examples, and tests: 381 insertions and 129 deletions. Production `src/`, Cargo
manifests, dependencies, and shipped API are untouched. All sixteen default
and all-feature release/size native/WASM artifact hashes and sizes are
byte-identical to `d5093a9d`.

The regenerated five-crate production graph is unchanged at 15,140 nodes /
25,245 edges. Examples/tests contain 17,477 / 28,586; all tests, examples,
benches, and fuzz targets contain 21,522 / 34,686. Every thin-dyadic node is a
generator, test, exporter, or probe node. There is no production thin-dyadic
node, selector, predicate, alternate engine, or Hyperlimit edge.

## Validation and reproduction

The committed source passes 202 default tests with six ignores, 203
all-feature tests with six ignores, and 153 minimal library tests. Warning-
denied all-target/all-feature Clippy and rustdoc, fuzz-target checks,
bench/example checks, formatting, both size matrices, the three new two-policy
large-heap rows, and all three call graphs pass.

```sh
cargo test --locked
cargo test --locked --all-features
cargo test --locked --lib --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --examples --all-features
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all

target/release/examples/export_cgal_exact_off \
  thin_dyadic_boxes_2048 /tmp/hypermesh-thin-dyadic
target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  /tmp/hypermesh-thin-dyadic/thin_dyadic_boxes_2048-left.off \
  /tmp/hypermesh-thin-dyadic/thin_dyadic_boxes_2048-right.off \
  union 21 outside
target/release/examples/large_mesh_kernel_heap_probe \
  thin-dyadic-2048 strict
```

Phases 11, 17, and 18 remain open for external real-world pathology,
unexplained semantic cells, the new 512-bit schedule loss, every other per-case
CGAL gap, and the final requirement audit.
