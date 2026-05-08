# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1778244273`.

## Commands

Run the default benchmark suite and update this file:

```sh
cargo bench --bench predicates
```

Run dispatch tracing separately and update `dispatch_trace.md`:

```sh
cargo bench --bench predicates --features dispatch-trace,realistic-blas -- --write-dispatch-trace-md
```

Regenerate this file from existing Criterion output:

```sh
cargo run --example write_benchmarks_md
```

Run optional scalar representation benchmarks:

```sh
RUSTFLAGS='-Ctarget-cpu=haswell' cargo bench --bench predicates --features hyperreal,realistic-blas,interval
```

Open Criterion's detailed HTML report at `target/criterion/report/index.html`.

## Latest Results

| Predicate | Representation | Workload | Mean | 95% CI | Median | Change vs Baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `classify_point_line` | `f32` | `easy` | 25.12 us | 24.49 us - 25.98 us | 24.46 us | - |
| `classify_point_line` | `f32` | `near_degenerate` | 52.15 us | 48.96 us - 54.08 us | 53.77 us | - |
| `classify_point_line` | `f64` | `easy` | 23.88 us | 23.50 us - 24.28 us | 23.61 us | - |
| `classify_point_line` | `f64` | `near_degenerate` | 26.01 us | 24.69 us - 27.43 us | 25.42 us | - |
| `classify_point_line` | `hyperreal` | `easy` | 172.42 us | 161.33 us - 185.05 us | 171.00 us | -22.51% |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 145.45 us | 139.10 us - 153.74 us | 142.25 us | -35.37% |
| `classify_point_line` | `realistic_blas` | `easy` | 117.15 us | 116.14 us - 118.21 us | 115.84 us | -5.60% |
| `classify_point_line` | `realistic_blas` | `near_degenerate` | 119.91 us | 118.94 us - 121.04 us | 119.28 us | -6.78% |
| `classify_point_oriented_plane` | `f32` | `easy` | 43.73 us | 42.73 us - 44.87 us | 42.90 us | - |
| `classify_point_oriented_plane` | `f32` | `near_degenerate` | 55.81 us | 52.84 us - 58.64 us | 57.85 us | - |
| `classify_point_oriented_plane` | `f64` | `easy` | 54.66 us | 47.53 us - 63.15 us | 50.54 us | - |
| `classify_point_oriented_plane` | `f64` | `near_degenerate` | 43.66 us | 41.78 us - 46.44 us | 41.66 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 290.32 us | 282.12 us - 298.32 us | 290.55 us | -26.38% |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 288.97 us | 284.00 us - 295.46 us | 287.71 us | -42.55% |
| `classify_point_oriented_plane` | `realistic_blas` | `easy` | 247.96 us | 244.43 us - 251.79 us | 241.56 us | -4.29% |
| `classify_point_oriented_plane` | `realistic_blas` | `near_degenerate` | 238.29 us | 236.22 us - 240.55 us | 234.02 us | -8.46% |
| `classify_point_plane` | `f32` | `easy` | 24.93 us | 23.30 us - 26.63 us | 23.22 us | +395.38% |
| `classify_point_plane` | `f32` | `near_degenerate` | 25.60 us | 24.52 us - 26.75 us | 25.61 us | +436.20% |
| `classify_point_plane` | `f64` | `easy` | 23.93 us | 23.29 us - 25.08 us | 23.36 us | +382.61% |
| `classify_point_plane` | `f64` | `near_degenerate` | 23.17 us | 22.82 us - 23.66 us | 23.00 us | +395.59% |
| `classify_point_plane` | `hyperreal` | `easy` | 209.15 us | 187.20 us - 232.13 us | 187.23 us | +24.14% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 171.69 us | 170.17 us - 173.52 us | 170.74 us | +7.16% |
| `classify_point_plane` | `realistic_blas` | `easy` | 144.03 us | 143.52 us - 144.59 us | 143.66 us | -2.81% |
| `classify_point_plane` | `realistic_blas` | `near_degenerate` | 144.03 us | 143.57 us - 144.51 us | 143.55 us | -1.45% |
| `incircle2d` | `f32` | `easy` | 37.71 us | 36.44 us - 39.22 us | 36.51 us | +1213.80% |
| `incircle2d` | `f32` | `near_degenerate` | 38.65 us | 37.71 us - 39.77 us | 38.71 us | +1291.37% |
| `incircle2d` | `f64` | `easy` | 40.95 us | 40.74 us - 41.17 us | 40.87 us | +1415.16% |
| `incircle2d` | `f64` | `near_degenerate` | 42.10 us | 41.46 us - 42.81 us | 42.31 us | +1452.88% |
| `incircle2d` | `hyperreal` | `easy` | 136.07 us | 134.50 us - 138.34 us | 134.98 us | +5.70% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 138.65 us | 135.94 us - 141.70 us | 136.46 us | +7.92% |
| `incircle2d` | `realistic_blas` | `easy` | 123.12 us | 121.75 us - 124.48 us | 123.94 us | -5.29% |
| `incircle2d` | `realistic_blas` | `near_degenerate` | 122.70 us | 121.10 us - 124.28 us | 124.76 us | -7.38% |
| `insphere3d` | `f32` | `easy` | 79.95 us | 78.74 us - 81.43 us | 79.40 us | +334.92% |
| `insphere3d` | `f32` | `near_degenerate` | 80.36 us | 78.07 us - 83.29 us | 78.42 us | +295.07% |
| `insphere3d` | `f64` | `easy` | 81.44 us | 79.71 us - 83.89 us | 79.99 us | +940.05% |
| `insphere3d` | `f64` | `near_degenerate` | 81.03 us | 79.75 us - 82.69 us | 80.52 us | +923.57% |
| `insphere3d` | `hyperreal` | `easy` | 232.22 us | 227.24 us - 236.79 us | 230.75 us | +13.76% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 309.62 us | 283.16 us - 334.02 us | 324.31 us | +44.08% |
| `insphere3d` | `realistic_blas` | `easy` | 181.29 us | 178.70 us - 183.87 us | 181.44 us | -0.95% |
| `insphere3d` | `realistic_blas` | `near_degenerate` | 192.33 us | 190.29 us - 194.26 us | 196.83 us | +2.22% |
| `orient2d` | `f32` | `easy` | 25.23 us | 24.79 us - 25.70 us | 25.09 us | +203.53% |
| `orient2d` | `f32` | `near_degenerate` | 32.72 us | 32.34 us - 33.12 us | 32.69 us | +99.57% |
| `orient2d` | `f64` | `easy` | 26.77 us | 24.52 us - 29.25 us | 23.98 us | +729.79% |
| `orient2d` | `f64` | `near_degenerate` | 25.07 us | 24.29 us - 25.88 us | 24.91 us | +537.98% |
| `orient2d` | `hyperreal` | `easy` | 122.06 us | 120.90 us - 123.22 us | 122.03 us | -14.70% |
| `orient2d` | `hyperreal` | `near_degenerate` | 123.28 us | 121.29 us - 125.31 us | 122.40 us | -8.57% |
| `orient2d` | `realistic_blas` | `easy` | 124.81 us | 123.15 us - 126.76 us | 122.79 us | +5.78% |
| `orient2d` | `realistic_blas` | `near_degenerate` | 132.23 us | 128.85 us - 135.94 us | 125.40 us | +2.35% |
| `orient3d` | `f32` | `easy` | 73.94 us | 65.58 us - 83.81 us | 68.35 us | +722.77% |
| `orient3d` | `f32` | `near_degenerate` | 43.82 us | 42.07 us - 46.53 us | 42.32 us | +392.46% |
| `orient3d` | `f64` | `easy` | 59.16 us | 51.93 us - 66.35 us | 57.78 us | +581.24% |
| `orient3d` | `f64` | `near_degenerate` | 41.62 us | 40.57 us - 43.11 us | 40.74 us | +376.17% |
| `orient3d` | `hyperreal` | `easy` | 244.36 us | 237.07 us - 252.46 us | 243.26 us | -9.70% |
| `orient3d` | `hyperreal` | `near_degenerate` | 143.09 us | 141.18 us - 145.12 us | 142.57 us | -20.34% |
| `orient3d` | `realistic_blas` | `easy` | 247.13 us | 243.28 us - 251.69 us | 240.93 us | -0.86% |
| `orient3d` | `realistic_blas` | `near_degenerate` | 143.64 us | 142.81 us - 144.50 us | 142.46 us | +1.75% |
