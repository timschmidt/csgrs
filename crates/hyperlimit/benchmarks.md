# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1784463127`.

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
| `classify_point_line` | `hyperreal` | `easy` | 29.63 us | 29.48 us - 29.80 us | 29.49 us | - |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 30.72 us | 30.50 us - 31.00 us | 30.52 us | - |
| `classify_point_line_fixed` | `hyperreal` | `easy` | 22.74 us | 22.64 us - 22.87 us | 22.67 us | - |
| `classify_point_line_fixed` | `hyperreal` | `near_degenerate` | 21.04 us | 20.99 us - 21.11 us | 21.00 us | - |
| `classify_point_line_fixed` | `hyperreal_prepared` | `easy` | 11.03 us | 10.97 us - 11.09 us | 10.98 us | - |
| `classify_point_line_fixed` | `hyperreal_prepared` | `near_degenerate` | 10.41 us | 10.40 us - 10.43 us | 10.41 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 54.27 us | 53.01 us - 55.87 us | 52.73 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 54.69 us | 53.89 us - 55.60 us | 53.45 us | - |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `easy` | 23.50 us | 23.34 us - 23.69 us | 23.33 us | - |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `near_degenerate` | 24.27 us | 24.05 us - 24.56 us | 24.01 us | - |
| `classify_point_plane` | `hyperreal` | `easy` | 31.34 us | 31.17 us - 31.55 us | 31.15 us | - |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 29.42 us | 29.23 us - 29.62 us | 29.23 us | - |
| `classify_point_plane` | `hyperreal_prepared` | `easy` | 24.07 us | 23.74 us - 24.45 us | 23.82 us | - |
| `classify_point_plane` | `hyperreal_prepared` | `near_degenerate` | 21.33 us | 21.13 us - 21.56 us | 21.13 us | - |
| `incircle2d` | `hyperreal` | `easy` | 24.46 us | 24.30 us - 24.65 us | 24.33 us | -82.90% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 25.09 us | 24.64 us - 25.68 us | 24.65 us | -82.46% |
| `incircle2d` | `hyperreal_prepared` | `easy` | 16.38 us | 16.06 us - 16.77 us | 15.92 us | -72.40% |
| `incircle2d` | `hyperreal_prepared` | `near_degenerate` | 15.67 us | 15.60 us - 15.75 us | 15.57 us | -73.34% |
| `incircle2d` | `robust` | `easy` | 2.85 us | 2.83 us - 2.87 us | 2.82 us | -0.58% |
| `incircle2d` | `robust` | `near_degenerate` | 2.94 us | 2.93 us - 2.94 us | 2.93 us | +0.60% |
| `insphere3d` | `hyperreal` | `easy` | 98.97 us | 98.70 us - 99.30 us | 98.85 us | -58.97% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 100.94 us | 100.31 us - 101.69 us | 100.64 us | -59.43% |
| `insphere3d` | `hyperreal_prepared` | `easy` | 59.26 us | 58.57 us - 60.02 us | 59.19 us | -50.04% |
| `insphere3d` | `hyperreal_prepared` | `near_degenerate` | 61.35 us | 60.44 us - 62.52 us | 60.30 us | -48.54% |
| `insphere3d` | `robust` | `easy` | 13.80 us | 13.73 us - 13.90 us | 13.78 us | +0.43% |
| `insphere3d` | `robust` | `near_degenerate` | 13.82 us | 13.78 us - 13.86 us | 13.78 us | +1.05% |
| `orient2d` | `hyperreal` | `easy` | 13.27 us | 13.23 us - 13.34 us | 13.23 us | -90.19% |
| `orient2d` | `hyperreal` | `near_degenerate` | 12.97 us | 12.95 us - 12.99 us | 12.94 us | -89.67% |
| `orient2d` | `robust` | `easy` | 1.22 us | 1.22 us - 1.23 us | 1.22 us | -0.63% |
| `orient2d` | `robust` | `near_degenerate` | 1.27 us | 1.27 us - 1.28 us | 1.27 us | -0.73% |
| `orient3d` | `hyperreal` | `easy` | 45.21 us | 44.42 us - 46.06 us | 44.88 us | -84.82% |
| `orient3d` | `hyperreal` | `near_degenerate` | 40.20 us | 39.62 us - 40.85 us | 39.56 us | -75.04% |
| `orient3d` | `robust` | `easy` | 8.31 us | 8.27 us - 8.35 us | 8.28 us | -6.69% |
| `orient3d` | `robust` | `near_degenerate` | 8.14 us | 8.11 us - 8.16 us | 8.13 us | -8.67% |
