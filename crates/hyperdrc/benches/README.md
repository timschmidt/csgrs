# hyperdrc Benches

This folder contains optimized smoke-performance targets for `hyperdrc`. They
are broad regression anchors, not claims about statistically stable throughput.
Correctness belongs in unit and property tests beside the owning module.

## Targets

- [`parser_geometry_smoke.rs`](parser_geometry_smoke.rs) exercises synthetic
  parser, source-grid, geometry, check, policy, reporting, waiver, and package
  paths. It includes both dense workloads and sparse cases that should benefit
  from broad-phase candidate filtering.
- [`fixture_smoke.rs`](fixture_smoke.rs) measures bounded end-to-end work over
  the repository board fixtures: archive loading, KiCad/Gerber parsing, report
  generation, minimum-copper-neck review, and drill spacing.
- [`spatial_index_audit.rs`](spatial_index_audit.rs) isolates a sparse 10,003-drill
  workload for comparing deterministic broad-phase index implementations.

The benchmark source names each measured operation. Add a case when a new hot
path needs a stable workload; keep fixtures small enough for routine local runs.

## Usage

Compile the targets without executing them:

```sh
cargo check --benches --locked
```

Run one target:

```sh
cargo bench --bench parser_geometry_smoke
cargo bench --bench fixture_smoke
cargo bench --bench spatial_index_audit
```

Return to the [repository README](../README.md).
