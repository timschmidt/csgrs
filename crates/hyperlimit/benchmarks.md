# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1785399044`.

## Commands

Run the default benchmark suite and update this file:

```sh
cargo bench --bench predicates
```

Run dispatch tracing separately and update `dispatch_trace.md`:

```sh
cargo bench --bench predicates --features dispatch-trace -- --write-dispatch-trace-md
```

Regenerate this file from existing Criterion output:

```sh
cargo run --example write_benchmarks_md
```

Open Criterion's detailed HTML report at `target/criterion/report/index.html`.

## Latest Results

| Predicate | Representation | Workload | Mean | 95% CI | Median | Change vs Baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `classify_point_line` | `hyperreal` | `easy` | 23.17 us | 22.33 us - 23.95 us | 25.39 us | - |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 14.35 us | 14.31 us - 14.40 us | 14.36 us | - |
| `classify_point_line_fixed` | `hyperreal` | `easy` | 15.41 us | 15.37 us - 15.45 us | 15.38 us | - |
| `classify_point_line_fixed` | `hyperreal` | `near_degenerate` | 13.84 us | 13.81 us - 13.86 us | 13.82 us | - |
| `classify_point_line_fixed` | `hyperreal_oriented` | `easy` | 9.55 us | 9.54 us - 9.56 us | 9.54 us | - |
| `classify_point_line_fixed` | `hyperreal_oriented` | `near_degenerate` | 9.06 us | 9.05 us - 9.07 us | 9.04 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 39.52 us | 39.30 us - 39.78 us | 39.11 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 39.09 us | 38.84 us - 39.44 us | 38.74 us | - |
| `classify_point_oriented_plane` | `hyperreal_evidence` | `easy` | 21.14 us | 20.91 us - 21.40 us | 20.74 us | - |
| `classify_point_oriented_plane` | `hyperreal_evidence` | `near_degenerate` | 20.15 us | 20.13 us - 20.17 us | 20.14 us | - |
| `classify_point_plane` | `hyperreal` | `easy` | 21.50 us | 21.44 us - 21.56 us | 21.55 us | - |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 32.22 us | 32.09 us - 32.36 us | 32.40 us | - |
| `classify_point_plane` | `hyperreal_evidence` | `easy` | 9.65 us | 9.21 us - 10.13 us | 8.73 us | - |
| `classify_point_plane` | `hyperreal_evidence` | `near_degenerate` | 11.81 us | 11.08 us - 12.56 us | 9.86 us | - |
| `incircle2d` | `hyperreal` | `easy` | 32.09 us | 29.88 us - 34.35 us | 24.16 us | - |
| `incircle2d` | `hyperreal` | `near_degenerate` | 22.33 us | 22.29 us - 22.37 us | 22.27 us | - |
| `incircle2d` | `hyperreal_evidence` | `easy` | 17.89 us | 17.87 us - 17.91 us | 17.87 us | - |
| `incircle2d` | `hyperreal_evidence` | `near_degenerate` | 17.79 us | 17.67 us - 17.94 us | 17.54 us | - |
| `incircle2d` | `robust` | `easy` | 2.84 us | 2.83 us - 2.84 us | 2.83 us | - |
| `incircle2d` | `robust` | `near_degenerate` | 2.89 us | 2.88 us - 2.89 us | 2.88 us | - |
| `insphere3d` | `hyperreal` | `easy` | 66.37 us | 66.11 us - 66.68 us | 65.86 us | - |
| `insphere3d` | `hyperreal` | `near_degenerate` | 67.82 us | 67.54 us - 68.16 us | 67.35 us | - |
| `insphere3d` | `hyperreal_evidence` | `easy` | 41.01 us | 40.87 us - 41.19 us | 40.76 us | - |
| `insphere3d` | `hyperreal_evidence` | `near_degenerate` | 41.37 us | 41.22 us - 41.56 us | 41.12 us | - |
| `insphere3d` | `robust` | `easy` | 13.70 us | 13.64 us - 13.76 us | 13.59 us | - |
| `insphere3d` | `robust` | `near_degenerate` | 13.79 us | 13.74 us - 13.84 us | 13.66 us | - |
| `orient2d` | `hyperreal` | `easy` | 13.84 us | 13.82 us - 13.86 us | 13.81 us | - |
| `orient2d` | `hyperreal` | `near_degenerate` | 14.24 us | 14.17 us - 14.33 us | 14.10 us | - |
| `orient2d` | `robust` | `easy` | 1.23 us | 1.22 us - 1.23 us | 1.23 us | - |
| `orient2d` | `robust` | `near_degenerate` | 1.28 us | 1.27 us - 1.28 us | 1.27 us | - |
| `orient3d` | `hyperreal` | `easy` | 43.19 us | 42.81 us - 43.63 us | 42.97 us | - |
| `orient3d` | `hyperreal` | `near_degenerate` | 39.47 us | 39.31 us - 39.65 us | 39.22 us | - |
| `orient3d` | `robust` | `easy` | 8.27 us | 8.25 us - 8.31 us | 8.26 us | - |
| `orient3d` | `robust` | `near_degenerate` | 8.17 us | 8.16 us - 8.19 us | 8.18 us | - |
