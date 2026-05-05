# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1777944858`.

## Commands

Run the default benchmark suite and update this file:

```sh
cargo bench --bench predicates
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
| `classify_point_line` | `f32` | `easy` | 2.38 us | 2.37 us - 2.39 us | 2.36 us | -24.07% |
| `classify_point_line` | `f32` | `near_degenerate` | 1.80 us | 1.79 us - 1.82 us | 1.79 us | -33.85% |
| `classify_point_line` | `f64` | `easy` | 2.06 us | 2.06 us - 2.07 us | 2.06 us | -24.69% |
| `classify_point_line` | `f64` | `near_degenerate` | 2.11 us | 2.10 us - 2.12 us | 2.09 us | -24.55% |
| `classify_point_line` | `hyperreal` | `easy` | 3.29 ms | 3.27 ms - 3.30 ms | 3.26 ms | - |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 1.92 ms | 1.90 ms - 1.93 ms | 1.88 ms | - |
| `classify_point_line` | `interval_singleton` | `easy` | 32.81 us | 32.70 us - 32.94 us | 32.65 us | - |
| `classify_point_line` | `interval_singleton` | `near_degenerate` | 32.93 us | 32.82 us - 33.07 us | 32.75 us | - |
| `classify_point_line` | `realistic_blas` | `easy` | 3.31 ms | 3.29 ms - 3.33 ms | 3.26 ms | - |
| `classify_point_line` | `realistic_blas` | `near_degenerate` | 1.94 ms | 1.92 ms - 1.95 ms | 1.92 ms | - |
| `classify_point_oriented_plane` | `f32` | `easy` | 6.10 us | 6.09 us - 6.11 us | 6.09 us | -7.99% |
| `classify_point_oriented_plane` | `f32` | `near_degenerate` | 6.12 us | 6.11 us - 6.13 us | 6.10 us | -9.04% |
| `classify_point_oriented_plane` | `f64` | `easy` | 5.80 us | 5.78 us - 5.83 us | 5.74 us | -3.15% |
| `classify_point_oriented_plane` | `f64` | `near_degenerate` | 5.76 us | 5.74 us - 5.78 us | 5.74 us | -5.95% |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 12.39 ms | 12.26 ms - 12.53 ms | 12.12 ms | - |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 11.99 ms | 11.91 ms - 12.08 ms | 11.85 ms | - |
| `classify_point_oriented_plane` | `interval_singleton` | `easy` | 170.39 us | 169.94 us - 170.88 us | 169.94 us | - |
| `classify_point_oriented_plane` | `interval_singleton` | `near_degenerate` | 167.03 us | 166.63 us - 167.47 us | 166.32 us | - |
| `classify_point_oriented_plane` | `realistic_blas` | `easy` | 12.14 ms | 12.06 ms - 12.23 ms | 12.02 ms | - |
| `classify_point_oriented_plane` | `realistic_blas` | `near_degenerate` | 12.01 ms | 11.96 ms - 12.07 ms | 11.96 ms | - |
| `classify_point_plane` | `f32` | `easy` | 2.96 us | 2.96 us - 2.97 us | 2.96 us | -33.15% |
| `classify_point_plane` | `f32` | `near_degenerate` | 2.96 us | 2.96 us - 2.97 us | 2.96 us | -33.43% |
| `classify_point_plane` | `f64` | `easy` | 2.83 us | 2.81 us - 2.86 us | 2.80 us | -34.04% |
| `classify_point_plane` | `f64` | `near_degenerate` | 2.85 us | 2.83 us - 2.87 us | 2.84 us | -38.84% |
| `classify_point_plane` | `hyperreal` | `easy` | 2.88 ms | 2.86 ms - 2.90 ms | 2.85 ms | - |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 2.79 ms | 2.76 ms - 2.81 ms | 2.74 ms | - |
| `classify_point_plane` | `interval_singleton` | `easy` | 27.87 us | 27.79 us - 27.97 us | 27.78 us | - |
| `classify_point_plane` | `interval_singleton` | `near_degenerate` | 28.27 us | 28.19 us - 28.35 us | 28.23 us | - |
| `classify_point_plane` | `realistic_blas` | `easy` | 2.94 ms | 2.93 ms - 2.96 ms | 2.92 ms | - |
| `classify_point_plane` | `realistic_blas` | `near_degenerate` | 2.82 ms | 2.81 ms - 2.84 ms | 2.79 ms | - |
| `incircle2d` | `f32` | `easy` | 729.83 ns | 728.86 ns - 730.86 ns | 728.22 ns | -3.42% |
| `incircle2d` | `f32` | `near_degenerate` | 735.72 ns | 733.91 ns - 737.76 ns | 732.65 ns | -3.17% |
| `incircle2d` | `f64` | `easy` | 765.26 ns | 751.99 ns - 781.75 ns | 737.12 ns | +1.85% |
| `incircle2d` | `f64` | `near_degenerate` | 757.09 ns | 749.92 ns - 765.43 ns | 743.05 ns | -2.02% |
| `incircle2d` | `hyperreal` | `easy` | 15.98 ms | 15.93 ms - 16.04 ms | 15.87 ms | - |
| `incircle2d` | `hyperreal` | `near_degenerate` | 16.23 ms | 16.13 ms - 16.35 ms | 16.19 ms | - |
| `incircle2d` | `interval_cells` | `strict` | 231.99 us | 231.44 us - 232.60 us | 230.99 us | - |
| `incircle2d` | `interval_singleton` | `easy` | 226.09 us | 225.05 us - 227.16 us | 225.64 us | - |
| `incircle2d` | `interval_singleton` | `near_degenerate` | 219.59 us | 218.72 us - 220.54 us | 217.79 us | - |
| `incircle2d` | `realistic_blas` | `easy` | 16.22 ms | 16.15 ms - 16.30 ms | 16.10 ms | - |
| `incircle2d` | `realistic_blas` | `near_degenerate` | 16.20 ms | 16.09 ms - 16.31 ms | 16.03 ms | - |
| `insphere3d` | `f32` | `easy` | 4.98 us | 4.92 us - 5.05 us | 4.87 us | -20.40% |
| `insphere3d` | `f32` | `near_degenerate` | 4.78 us | 4.77 us - 4.79 us | 4.77 us | -20.43% |
| `insphere3d` | `f64` | `easy` | 4.71 us | 4.69 us - 4.73 us | 4.68 us | -14.36% |
| `insphere3d` | `f64` | `near_degenerate` | 4.72 us | 4.71 us - 4.74 us | 4.70 us | -13.70% |
| `insphere3d` | `hyperreal` | `easy` | 49.85 ms | 49.39 ms - 50.36 ms | 49.16 ms | - |
| `insphere3d` | `hyperreal` | `near_degenerate` | 49.30 ms | 48.95 ms - 49.69 ms | 48.88 ms | - |
| `insphere3d` | `interval_singleton` | `easy` | 751.92 us | 749.18 us - 755.23 us | 747.82 us | - |
| `insphere3d` | `interval_singleton` | `near_degenerate` | 748.96 us | 746.70 us - 751.37 us | 745.88 us | - |
| `insphere3d` | `realistic_blas` | `easy` | 49.35 ms | 49.04 ms - 49.68 ms | 48.93 ms | - |
| `insphere3d` | `realistic_blas` | `near_degenerate` | 48.16 ms | 47.91 ms - 48.45 ms | 47.62 ms | - |
| `orient2d` | `f32` | `easy` | 2.36 us | 2.35 us - 2.37 us | 2.35 us | -24.72% |
| `orient2d` | `f32` | `near_degenerate` | 1.78 us | 1.77 us - 1.78 us | 1.77 us | -34.42% |
| `orient2d` | `f64` | `easy` | 2.10 us | 2.09 us - 2.11 us | 2.09 us | -21.21% |
| `orient2d` | `f64` | `near_degenerate` | 2.09 us | 2.08 us - 2.10 us | 2.07 us | -23.36% |
| `orient2d` | `hyperreal` | `easy` | 3.24 ms | 3.23 ms - 3.25 ms | 3.22 ms | - |
| `orient2d` | `hyperreal` | `near_degenerate` | 1.86 ms | 1.86 ms - 1.87 ms | 1.85 ms | - |
| `orient2d` | `interval_cells` | `strict` | 33.08 us | 32.96 us - 33.22 us | 32.93 us | - |
| `orient2d` | `interval_singleton` | `easy` | 32.93 us | 32.84 us - 33.02 us | 32.84 us | - |
| `orient2d` | `interval_singleton` | `near_degenerate` | 33.46 us | 33.31 us - 33.62 us | 33.39 us | - |
| `orient2d` | `realistic_blas` | `easy` | 3.32 ms | 3.30 ms - 3.33 ms | 3.30 ms | - |
| `orient2d` | `realistic_blas` | `near_degenerate` | 1.92 ms | 1.91 ms - 1.93 ms | 1.90 ms | - |
| `orient3d` | `f32` | `easy` | 6.18 us | 6.16 us - 6.20 us | 6.15 us | -8.22% |
| `orient3d` | `f32` | `near_degenerate` | 6.14 us | 6.13 us - 6.16 us | 6.11 us | -8.96% |
| `orient3d` | `f64` | `easy` | 5.69 us | 5.69 us - 5.70 us | 5.69 us | -5.26% |
| `orient3d` | `f64` | `near_degenerate` | 5.84 us | 5.80 us - 5.88 us | 5.76 us | -5.81% |
| `orient3d` | `hyperreal` | `easy` | 12.54 ms | 12.39 ms - 12.71 ms | 12.25 ms | - |
| `orient3d` | `hyperreal` | `near_degenerate` | 9.47 ms | 9.43 ms - 9.52 ms | 9.37 ms | - |
| `orient3d` | `interval_singleton` | `easy` | 161.18 us | 160.82 us - 161.58 us | 160.75 us | - |
| `orient3d` | `interval_singleton` | `near_degenerate` | 170.63 us | 170.42 us - 170.86 us | 170.40 us | - |
| `orient3d` | `realistic_blas` | `easy` | 12.17 ms | 12.11 ms - 12.24 ms | 12.04 ms | - |
| `orient3d` | `realistic_blas` | `near_degenerate` | 9.87 ms | 9.81 ms - 9.93 ms | 9.85 ms | - |
