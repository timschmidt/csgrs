# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1778031392`.

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
| `classify_point_plane` | `f32` | `easy` | 5.03 us | 4.91 us - 5.17 us | 5.00 us | - |
| `classify_point_plane` | `f32` | `near_degenerate` | 4.77 us | 4.76 us - 4.79 us | 4.77 us | - |
| `classify_point_plane` | `f64` | `easy` | 4.96 us | 4.70 us - 5.21 us | 5.14 us | - |
| `classify_point_plane` | `f64` | `near_degenerate` | 4.68 us | 4.34 us - 5.17 us | 4.52 us | - |
| `classify_point_plane` | `hyperreal` | `easy` | 1.55 ms | 1.50 ms - 1.61 ms | 1.52 ms | +8.82% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 1.52 ms | 1.48 ms - 1.57 ms | 1.48 ms | +4.39% |
| `classify_point_plane` | `realistic_blas` | `easy` | 1.53 ms | 1.50 ms - 1.55 ms | 1.52 ms | -3.37% |
| `classify_point_plane` | `realistic_blas` | `near_degenerate` | 1.52 ms | 1.51 ms - 1.53 ms | 1.51 ms | +1.50% |
| `incircle2d` | `f32` | `easy` | 6.97 us | 6.90 us - 7.04 us | 6.95 us | - |
| `incircle2d` | `f32` | `near_degenerate` | 7.07 us | 7.04 us - 7.09 us | 7.04 us | - |
| `incircle2d` | `f64` | `easy` | 4.87 us | 4.77 us - 5.00 us | 4.82 us | - |
| `incircle2d` | `f64` | `near_degenerate` | 4.69 us | 4.66 us - 4.73 us | 4.71 us | - |
| `incircle2d` | `hyperreal` | `easy` | 1.36 ms | 1.34 ms - 1.38 ms | 1.35 ms | -9.23% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 1.33 ms | 1.32 ms - 1.35 ms | 1.33 ms | -12.89% |
| `incircle2d` | `realistic_blas` | `easy` | 1.39 ms | 1.36 ms - 1.41 ms | 1.37 ms | +8.75% |
| `incircle2d` | `realistic_blas` | `near_degenerate` | 1.37 ms | 1.34 ms - 1.41 ms | 1.36 ms | +6.12% |
| `insphere3d` | `f32` | `easy` | 18.38 us | 18.12 us - 18.73 us | 18.20 us | - |
| `insphere3d` | `f32` | `near_degenerate` | 20.34 us | 20.20 us - 20.50 us | 20.33 us | - |
| `insphere3d` | `f64` | `easy` | 7.83 us | 7.61 us - 8.11 us | 7.74 us | - |
| `insphere3d` | `f64` | `near_degenerate` | 7.92 us | 7.54 us - 8.37 us | 7.55 us | - |
| `insphere3d` | `hyperreal` | `easy` | 192.97 us | 191.44 us - 194.53 us | 190.34 us | -90.67% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 2.08 ms | 2.02 ms - 2.16 ms | 2.03 ms | +4.03% |
| `insphere3d` | `realistic_blas` | `easy` | 201.12 us | 199.35 us - 202.90 us | 200.49 us | -90.20% |
| `insphere3d` | `realistic_blas` | `near_degenerate` | 2.10 ms | 2.08 ms - 2.12 ms | 2.09 ms | +0.32% |
| `orient2d` | `f32` | `easy` | 8.31 us | 8.19 us - 8.45 us | 8.27 us | +3.08% |
| `orient2d` | `f32` | `near_degenerate` | 16.39 us | 16.26 us - 16.54 us | 16.33 us | -2.15% |
| `orient2d` | `f64` | `easy` | 3.23 us | 3.21 us - 3.24 us | 3.20 us | -13.54% |
| `orient2d` | `f64` | `near_degenerate` | 3.93 us | 3.86 us - 4.00 us | 3.89 us | -50.50% |
| `orient2d` | `hyperreal` | `easy` | 115.71 us | 115.16 us - 116.32 us | 114.96 us | -91.03% |
| `orient2d` | `hyperreal` | `near_degenerate` | 752.52 us | 746.73 us - 758.64 us | 751.36 us | +3.86% |
| `orient2d` | `realistic_blas` | `easy` | 116.71 us | 114.79 us - 119.30 us | 114.53 us | -91.26% |
| `orient2d` | `realistic_blas` | `near_degenerate` | 756.66 us | 749.86 us - 765.04 us | 753.79 us | -2.90% |
| `orient3d` | `hyperreal` | `easy` | 2.64 ms | 2.59 ms - 2.70 ms | 2.63 ms | - |
| `orient3d` | `hyperreal` | `near_degenerate` | 1.05 ms | 985.68 us - 1.13 ms | 1.00 ms | - |
| `orient3d` | `realistic_blas` | `easy` | 2.52 ms | 2.51 ms - 2.54 ms | 2.52 ms | - |
| `orient3d` | `realistic_blas` | `near_degenerate` | 975.78 us | 964.12 us - 992.06 us | 964.52 us | - |
