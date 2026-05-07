# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1778119727`.

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
| `classify_point_line` | `hyperreal` | `easy` | 222.51 us | 212.28 us - 234.43 us | 213.55 us | - |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 225.06 us | 209.54 us - 250.05 us | 213.70 us | - |
| `classify_point_line` | `realistic_blas` | `easy` | 130.72 us | 115.09 us - 157.86 us | 115.39 us | - |
| `classify_point_line` | `realistic_blas` | `near_degenerate` | 133.85 us | 128.87 us - 139.54 us | 133.02 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 394.36 us | 322.59 us - 475.86 us | 365.69 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 502.99 us | 411.62 us - 664.32 us | 424.08 us | - |
| `classify_point_oriented_plane` | `realistic_blas` | `easy` | 574.52 us | 450.92 us - 712.66 us | 478.43 us | - |
| `classify_point_oriented_plane` | `realistic_blas` | `near_degenerate` | 502.13 us | 421.73 us - 609.70 us | 446.40 us | - |
| `classify_point_plane` | `f32` | `easy` | 5.03 us | 4.91 us - 5.17 us | 5.00 us | - |
| `classify_point_plane` | `f32` | `near_degenerate` | 4.77 us | 4.76 us - 4.79 us | 4.77 us | - |
| `classify_point_plane` | `f64` | `easy` | 4.96 us | 4.70 us - 5.21 us | 5.14 us | - |
| `classify_point_plane` | `f64` | `near_degenerate` | 4.68 us | 4.34 us - 5.17 us | 4.52 us | - |
| `classify_point_plane` | `hyperreal` | `easy` | 168.48 us | 163.67 us - 175.93 us | 164.70 us | -42.57% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 160.21 us | 156.96 us - 163.42 us | 159.28 us | -1.03% |
| `classify_point_plane` | `realistic_blas` | `easy` | 160.41 us | 159.27 us - 161.43 us | 160.69 us | -5.08% |
| `classify_point_plane` | `realistic_blas` | `near_degenerate` | 162.69 us | 160.75 us - 164.84 us | 161.99 us | -12.82% |
| `incircle2d` | `f32` | `easy` | 2.87 us | 2.83 us - 2.92 us | 2.85 us | +1.53% |
| `incircle2d` | `f32` | `near_degenerate` | 2.78 us | 2.76 us - 2.79 us | 2.76 us | -1.39% |
| `incircle2d` | `f64` | `easy` | 2.70 us | 2.69 us - 2.72 us | 2.70 us | -4.80% |
| `incircle2d` | `f64` | `near_degenerate` | 2.71 us | 2.70 us - 2.73 us | 2.70 us | -2.43% |
| `incircle2d` | `hyperreal` | `easy` | 128.72 us | 127.36 us - 130.07 us | 129.29 us | -19.78% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 128.47 us | 126.88 us - 130.39 us | 127.48 us | -23.37% |
| `incircle2d` | `realistic_blas` | `easy` | 127.98 us | 126.87 us - 129.08 us | 128.16 us | -24.87% |
| `incircle2d` | `realistic_blas` | `near_degenerate` | 128.53 us | 125.72 us - 131.66 us | 126.69 us | -23.68% |
| `insphere3d` | `f32` | `easy` | 18.38 us | 18.12 us - 18.73 us | 18.20 us | - |
| `insphere3d` | `f32` | `near_degenerate` | 20.34 us | 20.20 us - 20.50 us | 20.33 us | - |
| `insphere3d` | `f64` | `easy` | 7.83 us | 7.61 us - 8.11 us | 7.74 us | - |
| `insphere3d` | `f64` | `near_degenerate` | 7.92 us | 7.54 us - 8.37 us | 7.55 us | - |
| `insphere3d` | `hyperreal` | `easy` | 204.14 us | 200.40 us - 207.57 us | 205.27 us | +0.25% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 214.89 us | 209.52 us - 221.67 us | 210.84 us | +0.94% |
| `insphere3d` | `realistic_blas` | `easy` | 202.16 us | 196.24 us - 209.67 us | 197.83 us | -5.37% |
| `insphere3d` | `realistic_blas` | `near_degenerate` | 338.21 us | 298.85 us - 380.00 us | 332.28 us | +50.11% |
| `orient2d` | `f32` | `easy` | 8.31 us | 8.19 us - 8.45 us | 8.27 us | +3.08% |
| `orient2d` | `f32` | `near_degenerate` | 16.39 us | 16.26 us - 16.54 us | 16.33 us | -2.15% |
| `orient2d` | `f64` | `easy` | 3.23 us | 3.21 us - 3.24 us | 3.20 us | -13.54% |
| `orient2d` | `f64` | `near_degenerate` | 3.93 us | 3.86 us - 4.00 us | 3.89 us | -50.50% |
| `orient2d` | `hyperreal` | `easy` | 117.43 us | 115.23 us - 119.75 us | 117.32 us | -8.87% |
| `orient2d` | `hyperreal` | `near_degenerate` | 133.09 us | 130.75 us - 135.55 us | 132.39 us | -3.56% |
| `orient2d` | `realistic_blas` | `easy` | 120.46 us | 117.98 us - 123.27 us | 118.28 us | -4.78% |
| `orient2d` | `realistic_blas` | `near_degenerate` | 126.35 us | 124.86 us - 127.87 us | 126.13 us | -23.54% |
| `orient3d` | `f32` | `easy` | 8.99 us | 8.90 us - 9.10 us | 8.96 us | -1.76% |
| `orient3d` | `f32` | `near_degenerate` | 8.90 us | 8.78 us - 9.06 us | 8.80 us | -0.76% |
| `orient3d` | `f64` | `easy` | 8.68 us | 8.64 us - 8.73 us | 8.69 us | -0.51% |
| `orient3d` | `f64` | `near_degenerate` | 8.74 us | 8.67 us - 8.83 us | 8.73 us | -0.48% |
| `orient3d` | `hyperreal` | `easy` | 232.94 us | 229.55 us - 237.09 us | 231.40 us | -15.21% |
| `orient3d` | `hyperreal` | `near_degenerate` | 157.19 us | 155.93 us - 158.84 us | 156.70 us | -6.61% |
| `orient3d` | `realistic_blas` | `easy` | 234.69 us | 232.03 us - 237.42 us | 233.50 us | +0.28% |
| `orient3d` | `realistic_blas` | `near_degenerate` | 158.96 us | 157.80 us - 160.15 us | 158.95 us | -1.47% |
