# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1784476968`.

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
| `incircle2d` | `hyperreal` | `easy` | 24.01 us | 23.89 us - 24.16 us | 23.87 us | -1.84% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 24.23 us | 24.05 us - 24.42 us | 23.88 us | -3.41% |
| `incircle2d` | `hyperreal_prepared` | `easy` | 16.28 us | 16.20 us - 16.37 us | 16.20 us | -0.60% |
| `incircle2d` | `hyperreal_prepared` | `near_degenerate` | 15.98 us | 15.94 us - 16.02 us | 15.92 us | +1.97% |
| `incircle2d` | `robust` | `easy` | 2.88 us | 2.87 us - 2.90 us | 2.86 us | +1.25% |
| `incircle2d` | `robust` | `near_degenerate` | 2.95 us | 2.95 us - 2.96 us | 2.95 us | +0.59% |
| `insphere3d` | `hyperreal` | `easy` | 65.62 us | 65.49 us - 65.77 us | 65.45 us | -33.70% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 67.39 us | 67.22 us - 67.58 us | 67.26 us | -33.24% |
| `insphere3d` | `hyperreal_prepared` | `easy` | 42.17 us | 41.94 us - 42.47 us | 41.89 us | -28.84% |
| `insphere3d` | `hyperreal_prepared` | `near_degenerate` | 44.27 us | 44.05 us - 44.52 us | 43.98 us | -27.83% |
| `insphere3d` | `robust` | `easy` | 13.89 us | 13.84 us - 13.95 us | 13.80 us | +0.67% |
| `insphere3d` | `robust` | `near_degenerate` | 13.97 us | 13.92 us - 14.02 us | 13.89 us | +1.08% |
| `orient2d` | `hyperreal` | `easy` | 12.89 us | 12.84 us - 12.94 us | 12.83 us | -2.94% |
| `orient2d` | `hyperreal` | `near_degenerate` | 12.77 us | 12.72 us - 12.84 us | 12.71 us | -1.50% |
| `orient2d` | `robust` | `easy` | 1.24 us | 1.23 us - 1.24 us | 1.23 us | +1.35% |
| `orient2d` | `robust` | `near_degenerate` | 1.31 us | 1.30 us - 1.32 us | 1.29 us | +2.71% |
| `orient3d` | `hyperreal` | `easy` | 41.19 us | 40.95 us - 41.48 us | 40.87 us | -8.89% |
| `orient3d` | `hyperreal` | `near_degenerate` | 39.73 us | 39.45 us - 40.06 us | 39.30 us | -1.18% |
| `orient3d` | `robust` | `easy` | 8.30 us | 8.29 us - 8.31 us | 8.30 us | -0.04% |
| `orient3d` | `robust` | `near_degenerate` | 8.22 us | 8.20 us - 8.24 us | 8.20 us | +0.99% |
