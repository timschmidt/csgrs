# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1785184305`.

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
| `classify_point_line` | `hyperreal` | `easy` | 13.41 us | 13.40 us - 13.43 us | 13.39 us | - |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 13.61 us | 13.57 us - 13.65 us | 13.55 us | - |
| `classify_point_line_fixed` | `hyperreal` | `easy` | 14.07 us | 14.05 us - 14.08 us | 14.05 us | - |
| `classify_point_line_fixed` | `hyperreal` | `near_degenerate` | 13.19 us | 13.13 us - 13.27 us | 13.07 us | - |
| `classify_point_line_fixed` | `hyperreal_prepared` | `easy` | 8.77 us | 8.75 us - 8.80 us | 8.73 us | - |
| `classify_point_line_fixed` | `hyperreal_prepared` | `near_degenerate` | 8.13 us | 8.12 us - 8.14 us | 8.11 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 38.14 us | 38.02 us - 38.26 us | 38.47 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 38.29 us | 38.19 us - 38.39 us | 38.10 us | - |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `easy` | 20.61 us | 20.58 us - 20.65 us | 20.55 us | - |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `near_degenerate` | 21.08 us | 21.05 us - 21.12 us | 21.05 us | - |
| `classify_point_plane` | `hyperreal` | `easy` | 21.25 us | 21.23 us - 21.28 us | 21.22 us | - |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 19.24 us | 19.22 us - 19.28 us | 19.20 us | - |
| `classify_point_plane` | `hyperreal_prepared` | `easy` | 13.85 us | 13.75 us - 13.94 us | 14.17 us | - |
| `classify_point_plane` | `hyperreal_prepared` | `near_degenerate` | 12.24 us | 12.21 us - 12.28 us | 12.19 us | - |
| `incircle2d` | `hyperreal` | `easy` | 23.43 us | 23.34 us - 23.51 us | 23.56 us | - |
| `incircle2d` | `hyperreal` | `near_degenerate` | 23.38 us | 23.35 us - 23.42 us | 23.34 us | - |
| `incircle2d` | `hyperreal_prepared` | `easy` | 15.92 us | 15.90 us - 15.94 us | 15.89 us | - |
| `incircle2d` | `hyperreal_prepared` | `near_degenerate` | 16.08 us | 16.04 us - 16.12 us | 16.01 us | - |
| `incircle2d` | `robust` | `easy` | 2.81 us | 2.81 us - 2.82 us | 2.80 us | - |
| `incircle2d` | `robust` | `near_degenerate` | 2.91 us | 2.91 us - 2.92 us | 2.91 us | - |
| `insphere3d` | `hyperreal` | `easy` | 64.25 us | 63.76 us - 64.82 us | 63.41 us | - |
| `insphere3d` | `hyperreal` | `near_degenerate` | 64.69 us | 64.49 us - 64.92 us | 64.40 us | - |
| `insphere3d` | `hyperreal_prepared` | `easy` | 39.84 us | 39.73 us - 39.95 us | 39.71 us | - |
| `insphere3d` | `hyperreal_prepared` | `near_degenerate` | 40.62 us | 40.46 us - 40.79 us | 41.01 us | - |
| `insphere3d` | `robust` | `easy` | 13.05 us | 13.03 us - 13.08 us | 13.01 us | - |
| `insphere3d` | `robust` | `near_degenerate` | 13.10 us | 13.08 us - 13.12 us | 13.08 us | - |
| `orient2d` | `hyperreal` | `easy` | 12.44 us | 12.43 us - 12.45 us | 12.43 us | - |
| `orient2d` | `hyperreal` | `near_degenerate` | 12.54 us | 12.52 us - 12.56 us | 12.50 us | - |
| `orient2d` | `robust` | `easy` | 1.22 us | 1.21 us - 1.22 us | 1.21 us | - |
| `orient2d` | `robust` | `near_degenerate` | 1.27 us | 1.27 us - 1.27 us | 1.27 us | - |
| `orient3d` | `hyperreal` | `easy` | 43.34 us | 43.28 us - 43.39 us | 43.27 us | - |
| `orient3d` | `hyperreal` | `near_degenerate` | 39.28 us | 39.18 us - 39.40 us | 39.12 us | - |
| `orient3d` | `robust` | `easy` | 8.94 us | 8.92 us - 8.95 us | 8.93 us | - |
| `orient3d` | `robust` | `near_degenerate` | 8.94 us | 8.90 us - 9.00 us | 8.85 us | - |
