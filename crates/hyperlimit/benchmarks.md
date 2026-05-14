# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1778725993`.

## Commands

Run the default benchmark suite and update this file:

```sh
cargo bench --bench predicates
```

Run dispatch tracing separately and update `dispatch_trace.md`:

```sh
cargo bench --bench predicates --features dispatch-trace,hyperlattice -- --write-dispatch-trace-md
```

Regenerate this file from existing Criterion output:

```sh
cargo run --example write_benchmarks_md
```

Run optional scalar representation benchmarks:

```sh
RUSTFLAGS='-Ctarget-cpu=haswell' cargo bench --bench predicates --features hyperreal,hyperlattice,interval
```

Open Criterion's detailed HTML report at `target/criterion/report/index.html`.

## Latest Results

| Predicate | Representation | Workload | Mean | 95% CI | Median | Change vs Baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `classify_point_line` | `f32` | `easy` | 3.42 us | 3.41 us - 3.42 us | 3.41 us | +22.94% |
| `classify_point_line` | `f32` | `near_degenerate` | 3.08 us | 3.06 us - 3.10 us | 3.05 us | +25.63% |
| `classify_point_line` | `f64` | `easy` | 3.07 us | 3.07 us - 3.08 us | 3.06 us | +20.72% |
| `classify_point_line` | `f64` | `near_degenerate` | 3.06 us | 3.06 us - 3.07 us | 3.06 us | +23.85% |
| `classify_point_line` | `hyperlattice` | `easy` | 113.18 us | 113.03 us - 113.36 us | 113.04 us | -86.25% |
| `classify_point_line` | `hyperreal` | `easy` | 113.79 us | 113.72 us - 113.87 us | 113.73 us | -86.49% |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 214.61 us | 213.83 us - 215.40 us | 214.50 us | +68.11% |
| `classify_point_line` | `realistic_blas` | `easy` | 161.10 us | 154.14 us - 168.56 us | 143.11 us | +14.64% |
| `classify_point_line` | `realistic_blas` | `near_degenerate` | 140.25 us | 139.73 us - 140.81 us | 140.05 us | -2.04% |
| `classify_point_oriented_plane` | `f32` | `easy` | 5.52 us | 5.51 us - 5.53 us | 5.52 us | -25.96% |
| `classify_point_oriented_plane` | `f32` | `near_degenerate` | 5.57 us | 5.55 us - 5.60 us | 5.54 us | -29.29% |
| `classify_point_oriented_plane` | `f64` | `easy` | 5.10 us | 5.08 us - 5.12 us | 5.08 us | -29.93% |
| `classify_point_oriented_plane` | `f64` | `near_degenerate` | 5.13 us | 5.12 us - 5.15 us | 5.10 us | -22.20% |
| `classify_point_oriented_plane` | `hyperlattice` | `easy` | 223.48 us | 223.29 us - 223.68 us | 223.34 us | - |
| `classify_point_oriented_plane` | `hyperlattice` | `near_degenerate` | 223.24 us | 222.99 us - 223.54 us | 223.11 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 221.92 us | 221.49 us - 222.42 us | 221.64 us | -9.58% |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 228.24 us | 228.07 us - 228.43 us | 228.11 us | -8.38% |
| `classify_point_oriented_plane` | `realistic_blas` | `easy` | 276.56 us | 274.85 us - 278.76 us | 274.83 us | +0.43% |
| `classify_point_oriented_plane` | `realistic_blas` | `near_degenerate` | 275.64 us | 274.77 us - 276.56 us | 274.75 us | +15.67% |
| `classify_point_plane` | `f32` | `easy` | 4.00 us | 4.00 us - 4.01 us | 3.99 us | +3.09% |
| `classify_point_plane` | `f32` | `near_degenerate` | 4.06 us | 4.03 us - 4.10 us | 3.99 us | +18.58% |
| `classify_point_plane` | `f64` | `easy` | 3.91 us | 3.89 us - 3.94 us | 3.87 us | -36.21% |
| `classify_point_plane` | `f64` | `near_degenerate` | 3.89 us | 3.87 us - 3.90 us | 3.86 us | -33.60% |
| `classify_point_plane` | `hyperlattice` | `easy` | 116.44 us | 116.14 us - 116.79 us | 115.92 us | -17.14% |
| `classify_point_plane` | `hyperlattice` | `near_degenerate` | 117.12 us | 116.80 us - 117.48 us | 116.50 us | -16.87% |
| `classify_point_plane` | `hyperreal` | `easy` | 116.95 us | 116.32 us - 117.79 us | 115.91 us | -16.10% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 116.41 us | 116.29 us - 116.55 us | 116.16 us | -16.81% |
| `classify_point_plane` | `realistic_blas` | `easy` | 172.77 us | 172.03 us - 173.58 us | 172.11 us | +1.00% |
| `classify_point_plane` | `realistic_blas` | `near_degenerate` | 169.44 us | 168.84 us - 170.06 us | 169.14 us | +17.64% |
| `incircle2d` | `f32` | `easy` | 726.58 ns | 724.21 ns - 729.35 ns | 723.39 ns | +24.61% |
| `incircle2d` | `f32` | `near_degenerate` | 732.15 ns | 726.90 ns - 738.48 ns | 724.11 ns | +21.50% |
| `incircle2d` | `f64` | `easy` | 723.55 ns | 721.43 ns - 726.17 ns | 721.40 ns | +24.27% |
| `incircle2d` | `f64` | `near_degenerate` | 723.71 ns | 721.44 ns - 726.42 ns | 721.32 ns | +26.54% |
| `incircle2d` | `hyperreal` | `easy` | 123.66 us | 122.99 us - 124.37 us | 122.95 us | -9.12% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 123.28 us | 122.51 us - 124.17 us | 122.43 us | -11.09% |
| `incircle2d` | `realistic_blas` | `easy` | 139.36 us | 138.36 us - 140.71 us | 138.41 us | +0.47% |
| `incircle2d` | `realistic_blas` | `near_degenerate` | 138.76 us | 138.02 us - 139.60 us | 137.84 us | +13.09% |
| `insphere3d` | `f32` | `easy` | 8.53 us | 8.51 us - 8.56 us | 8.52 us | -20.88% |
| `insphere3d` | `f32` | `near_degenerate` | 8.53 us | 8.51 us - 8.55 us | 8.53 us | -30.62% |
| `insphere3d` | `f64` | `easy` | 7.46 us | 7.45 us - 7.48 us | 7.46 us | +0.27% |
| `insphere3d` | `f64` | `near_degenerate` | 7.45 us | 7.44 us - 7.47 us | 7.44 us | -22.20% |
| `insphere3d` | `hyperlattice` | `easy` | 135.40 us | 134.38 us - 137.13 us | 134.08 us | -14.89% |
| `insphere3d` | `hyperlattice` | `near_degenerate` | 135.18 us | 134.67 us - 135.74 us | 133.97 us | -12.80% |
| `insphere3d` | `hyperreal` | `easy` | 140.01 us | 139.51 us - 140.51 us | 140.70 us | -10.16% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 137.99 us | 137.31 us - 138.72 us | 137.79 us | -11.22% |
| `insphere3d` | `realistic_blas` | `easy` | 246.36 us | 232.23 us - 261.91 us | 210.78 us | +16.33% |
| `insphere3d` | `realistic_blas` | `near_degenerate` | 209.99 us | 208.99 us - 211.09 us | 209.29 us | +9.18% |
| `orient2d` | `f32` | `easy` | 3.40 us | 3.40 us - 3.41 us | 3.40 us | +8.92% |
| `orient2d` | `f32` | `near_degenerate` | 3.08 us | 3.07 us - 3.08 us | 3.08 us | +22.86% |
| `orient2d` | `f64` | `easy` | 3.08 us | 3.07 us - 3.08 us | 3.08 us | +45.42% |
| `orient2d` | `f64` | `near_degenerate` | 3.05 us | 3.05 us - 3.06 us | 3.05 us | +18.01% |
| `orient2d` | `hyperlattice` | `easy` | 113.39 us | 113.18 us - 113.69 us | 113.10 us | -86.71% |
| `orient2d` | `hyperreal` | `easy` | 112.49 us | 112.45 us - 112.53 us | 112.47 us | -86.05% |
| `orient2d` | `hyperreal` | `near_degenerate` | 129.66 us | 128.54 us - 130.93 us | 127.49 us | +1.49% |
| `orient2d` | `realistic_blas` | `easy` | 163.99 us | 154.80 us - 174.02 us | 139.28 us | +16.61% |
| `orient2d` | `realistic_blas` | `near_degenerate` | 246.43 us | 243.33 us - 251.09 us | 243.08 us | +72.74% |
| `orient3d` | `f32` | `easy` | 5.61 us | 5.59 us - 5.64 us | 5.58 us | -23.49% |
| `orient3d` | `f32` | `near_degenerate` | 5.61 us | 5.59 us - 5.63 us | 5.57 us | -22.16% |
| `orient3d` | `f64` | `easy` | 5.09 us | 5.08 us - 5.10 us | 5.08 us | -0.61% |
| `orient3d` | `f64` | `near_degenerate` | 5.09 us | 5.08 us - 5.10 us | 5.08 us | -38.35% |
| `orient3d` | `hyperlattice` | `easy` | 189.15 us | 188.41 us - 190.01 us | 188.83 us | -15.71% |
| `orient3d` | `hyperlattice` | `near_degenerate` | 123.57 us | 123.43 us - 123.72 us | 123.38 us | -10.06% |
| `orient3d` | `hyperreal` | `easy` | 186.73 us | 186.08 us - 187.48 us | 186.18 us | -15.81% |
| `orient3d` | `hyperreal` | `near_degenerate` | 123.10 us | 122.85 us - 123.38 us | 122.82 us | -9.80% |
| `orient3d` | `realistic_blas` | `easy` | 274.78 us | 272.60 us - 277.72 us | 271.80 us | +0.98% |
| `orient3d` | `realistic_blas` | `near_degenerate` | 181.59 us | 180.60 us - 182.62 us | 181.14 us | +26.42% |
