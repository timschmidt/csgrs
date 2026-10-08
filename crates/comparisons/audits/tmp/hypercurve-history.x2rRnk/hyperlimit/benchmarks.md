# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1788058531`.

## Commands

Run the default benchmark suite and update this file:

```sh
cargo bench --all-features --bench predicates
```

Run dispatch tracing separately and update `dispatch_trace.md`:

```sh
cargo bench --all-features --bench predicates -- --write-dispatch-trace-md
```

Regenerate this file from existing Criterion output:

```sh
cargo run --example write_benchmarks_md
```

Open Criterion's detailed HTML report at `target/criterion/report/index.html`.

Core predicate rows process 512 prebuilt cases per iteration. Hyperlimit rows accept exact `Real` coordinates and return policy/provenance-bearing outcomes; `robust`, `geometry-predicates`, and `apfp` rows accept binary64 coordinates and return only a determinant sign. The timings therefore compare end-to-end predicate APIs on equivalent binary64 values, not identical output semantics. `apfp` currently supplies only the 2D rows. Parallel batch rows process 8,192 cases.

## Latest Results

| Predicate | Representation | Workload | Mean | 95% CI | Median | Change vs Baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `batch_parallel/incircle2d/near_degenerate` | `rayon` | `near_degenerate` | 135.66 us | 133.90 us - 137.41 us | 135.66 us | - |
| `batch_parallel/incircle2d/near_degenerate` | `sequential` | `near_degenerate` | 385.39 us | 380.32 us - 390.46 us | 385.39 us | - |
| `batch_parallel/insphere3d/near_degenerate` | `rayon` | `near_degenerate` | 275.90 us | 275.64 us - 276.17 us | 275.90 us | - |
| `batch_parallel/insphere3d/near_degenerate` | `sequential` | `near_degenerate` | 1.17 ms | 1.16 ms - 1.18 ms | 1.17 ms | - |
| `batch_parallel/orient2d/near_degenerate` | `rayon` | `near_degenerate` | 118.23 us | 118.19 us - 118.27 us | 118.23 us | - |
| `batch_parallel/orient2d/near_degenerate` | `sequential` | `near_degenerate` | 368.01 us | 358.47 us - 377.55 us | 368.01 us | - |
| `batch_parallel/orient3d/near_degenerate` | `rayon` | `near_degenerate` | 173.44 us | 172.19 us - 174.68 us | 173.44 us | - |
| `batch_parallel/orient3d/near_degenerate` | `sequential` | `near_degenerate` | 658.70 us | 645.72 us - 671.69 us | 658.70 us | - |
| `incircle2d` | `apfp` | `easy` | 2.50 us | 2.47 us - 2.53 us | 2.50 us | - |
| `incircle2d` | `apfp` | `near_degenerate` | 2.63 us | 2.63 us - 2.63 us | 2.63 us | - |
| `incircle2d` | `geometry_predicates` | `easy` | 2.92 us | 2.90 us - 2.94 us | 2.92 us | - |
| `incircle2d` | `geometry_predicates` | `near_degenerate` | 3.09 us | 3.06 us - 3.13 us | 3.09 us | - |
| `incircle2d` | `hyperreal` | `easy` | 24.98 us | 24.34 us - 25.62 us | 24.98 us | - |
| `incircle2d` | `hyperreal` | `near_degenerate` | 24.92 us | 24.71 us - 25.13 us | 24.92 us | - |
| `incircle2d` | `hyperreal_evidence` | `easy` | 18.06 us | 18.00 us - 18.12 us | 18.06 us | - |
| `incircle2d` | `hyperreal_evidence` | `near_degenerate` | 18.10 us | 17.94 us - 18.26 us | 18.10 us | - |
| `incircle2d` | `robust` | `easy` | 2.91 us | 2.87 us - 2.95 us | 2.91 us | - |
| `incircle2d` | `robust` | `near_degenerate` | 3.01 us | 2.99 us - 3.02 us | 3.01 us | - |
| `insphere3d` | `geometry_predicates` | `easy` | 10.99 us | 10.90 us - 11.07 us | 10.99 us | - |
| `insphere3d` | `geometry_predicates` | `near_degenerate` | 10.82 us | 10.78 us - 10.86 us | 10.82 us | - |
| `insphere3d` | `hyperreal` | `easy` | 71.31 us | 71.11 us - 71.51 us | 71.31 us | - |
| `insphere3d` | `hyperreal` | `near_degenerate` | 74.25 us | 72.95 us - 75.54 us | 74.25 us | - |
| `insphere3d` | `hyperreal_evidence` | `easy` | 41.97 us | 41.86 us - 42.09 us | 41.97 us | - |
| `insphere3d` | `hyperreal_evidence` | `near_degenerate` | 42.78 us | 42.77 us - 42.79 us | 42.78 us | - |
| `insphere3d` | `robust` | `easy` | 14.21 us | 13.97 us - 14.45 us | 14.21 us | - |
| `insphere3d` | `robust` | `near_degenerate` | 13.98 us | 13.94 us - 14.01 us | 13.98 us | - |
| `orient2d` | `apfp` | `easy` | 986.89 ns | 962.76 ns - 1.01 us | 986.89 ns | - |
| `orient2d` | `apfp` | `near_degenerate` | 966.41 ns | 958.87 ns - 973.94 ns | 966.41 ns | - |
| `orient2d` | `geometry_predicates` | `easy` | 1.23 us | 1.21 us - 1.25 us | 1.23 us | - |
| `orient2d` | `geometry_predicates` | `near_degenerate` | 1.52 us | 1.51 us - 1.53 us | 1.52 us | - |
| `orient2d` | `hyperreal` | `easy` | 13.25 us | 13.04 us - 13.45 us | 13.25 us | - |
| `orient2d` | `hyperreal` | `near_degenerate` | 13.15 us | 12.79 us - 13.52 us | 13.15 us | - |
| `orient2d` | `robust` | `easy` | 1.30 us | 1.30 us - 1.30 us | 1.30 us | - |
| `orient2d` | `robust` | `near_degenerate` | 1.36 us | 1.35 us - 1.36 us | 1.36 us | - |
| `orient3d` | `geometry_predicates` | `easy` | 3.49 us | 3.46 us - 3.53 us | 3.49 us | - |
| `orient3d` | `geometry_predicates` | `near_degenerate` | 3.58 us | 3.54 us - 3.62 us | 3.58 us | - |
| `orient3d` | `hyperreal` | `easy` | 47.41 us | 46.20 us - 48.62 us | 47.41 us | - |
| `orient3d` | `hyperreal` | `near_degenerate` | 41.38 us | 41.28 us - 41.47 us | 41.38 us | - |
| `orient3d` | `robust` | `easy` | 8.40 us | 8.34 us - 8.45 us | 8.40 us | - |
| `orient3d` | `robust` | `near_degenerate` | 8.75 us | 8.72 us - 8.77 us | 8.75 us | - |
