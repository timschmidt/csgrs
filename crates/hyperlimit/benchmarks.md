# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1778386676`.

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
| `classify_point_line` | `f32` | `easy` | 4.39 us | 4.15 us - 4.64 us | 3.64 us | +22.94% |
| `classify_point_line` | `f32` | `near_degenerate` | 4.12 us | 3.88 us - 4.40 us | 3.50 us | +25.63% |
| `classify_point_line` | `f64` | `easy` | 4.59 us | 4.28 us - 4.93 us | 3.74 us | +20.72% |
| `classify_point_line` | `f64` | `near_degenerate` | 4.89 us | 4.65 us - 5.12 us | 5.36 us | +23.85% |
| `classify_point_line` | `hyperreal` | `easy` | 129.48 us | 128.71 us - 130.34 us | 128.48 us | +0.98% |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 214.61 us | 213.83 us - 215.40 us | 214.50 us | +68.11% |
| `classify_point_line` | `hyperlattice` | `easy` | 161.10 us | 154.14 us - 168.56 us | 143.11 us | +14.64% |
| `classify_point_line` | `hyperlattice` | `near_degenerate` | 140.25 us | 139.73 us - 140.81 us | 140.05 us | -2.04% |
| `classify_point_oriented_plane` | `f32` | `easy` | 11.35 us | 10.68 us - 12.04 us | 9.40 us | +15.21% |
| `classify_point_oriented_plane` | `f32` | `near_degenerate` | 11.88 us | 11.12 us - 12.69 us | 9.46 us | +36.59% |
| `classify_point_oriented_plane` | `f64` | `easy` | 11.83 us | 11.04 us - 12.69 us | 9.33 us | +42.12% |
| `classify_point_oriented_plane` | `f64` | `near_degenerate` | 10.61 us | 10.04 us - 11.24 us | 9.18 us | +28.32% |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 245.44 us | 244.56 us - 246.40 us | 244.65 us | -15.46% |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 249.11 us | 247.19 us - 251.30 us | 246.04 us | -13.79% |
| `classify_point_oriented_plane` | `hyperlattice` | `easy` | 276.56 us | 274.85 us - 278.76 us | 274.83 us | +0.43% |
| `classify_point_oriented_plane` | `hyperlattice` | `near_degenerate` | 275.64 us | 274.77 us - 276.56 us | 274.75 us | +15.67% |
| `classify_point_plane` | `f32` | `easy` | 4.73 us | 4.52 us - 4.98 us | 4.35 us | +3.09% |
| `classify_point_plane` | `f32` | `near_degenerate` | 5.82 us | 5.43 us - 6.23 us | 4.46 us | +18.58% |
| `classify_point_plane` | `f64` | `easy` | 7.56 us | 7.02 us - 8.15 us | 7.69 us | +48.46% |
| `classify_point_plane` | `f64` | `near_degenerate` | 6.84 us | 6.22 us - 7.51 us | 5.66 us | +68.11% |
| `classify_point_plane` | `hyperreal` | `easy` | 155.45 us | 154.75 us - 156.17 us | 154.55 us | -25.68% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 156.56 us | 155.69 us - 157.52 us | 155.62 us | -8.81% |
| `classify_point_plane` | `hyperlattice` | `easy` | 172.77 us | 172.03 us - 173.58 us | 172.11 us | +1.00% |
| `classify_point_plane` | `hyperlattice` | `near_degenerate` | 169.44 us | 168.84 us - 170.06 us | 169.14 us | +17.64% |
| `incircle2d` | `f32` | `easy` | 1.02 us | 969.03 ns - 1.08 us | 859.74 ns | +24.61% |
| `incircle2d` | `f32` | `near_degenerate` | 1.01 us | 932.38 ns - 1.10 us | 810.70 ns | +21.50% |
| `incircle2d` | `f64` | `easy` | 911.28 ns | 866.38 ns - 959.44 ns | 797.83 ns | +24.27% |
| `incircle2d` | `f64` | `near_degenerate` | 982.73 ns | 926.07 ns - 1.04 us | 812.50 ns | +26.54% |
| `incircle2d` | `hyperreal` | `easy` | 123.66 us | 122.99 us - 124.37 us | 122.95 us | -9.12% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 123.28 us | 122.51 us - 124.17 us | 122.43 us | -11.09% |
| `incircle2d` | `hyperlattice` | `easy` | 139.36 us | 138.36 us - 140.71 us | 138.41 us | +0.47% |
| `incircle2d` | `hyperlattice` | `near_degenerate` | 138.76 us | 138.02 us - 139.60 us | 137.84 us | +13.09% |
| `insphere3d` | `f32` | `easy` | 7.32 us | 6.98 us - 7.68 us | 6.29 us | +16.76% |
| `insphere3d` | `f32` | `near_degenerate` | 8.46 us | 7.99 us - 8.97 us | 8.73 us | +35.74% |
| `insphere3d` | `f64` | `easy` | 5.73 us | 5.60 us - 5.90 us | 5.50 us | +6.11% |
| `insphere3d` | `f64` | `near_degenerate` | 6.57 us | 6.23 us - 6.94 us | 5.61 us | +18.71% |
| `insphere3d` | `hyperreal` | `easy` | 187.52 us | 186.39 us - 188.82 us | 185.75 us | -19.25% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 184.80 us | 184.33 us - 185.30 us | 184.25 us | -40.31% |
| `insphere3d` | `hyperlattice` | `easy` | 246.36 us | 232.23 us - 261.91 us | 210.78 us | +16.33% |
| `insphere3d` | `hyperlattice` | `near_degenerate` | 209.99 us | 208.99 us - 211.09 us | 209.29 us | +9.18% |
| `orient2d` | `f32` | `easy` | 3.82 us | 3.68 us - 3.98 us | 3.52 us | +8.92% |
| `orient2d` | `f32` | `near_degenerate` | 4.04 us | 3.84 us - 4.25 us | 3.49 us | +22.86% |
| `orient2d` | `f64` | `easy` | 5.15 us | 4.92 us - 5.39 us | 5.34 us | +45.42% |
| `orient2d` | `f64` | `near_degenerate` | 4.90 us | 4.64 us - 5.17 us | 5.28 us | +18.01% |
| `orient2d` | `hyperreal` | `easy` | 217.34 us | 216.25 us - 218.74 us | 216.77 us | +67.18% |
| `orient2d` | `hyperreal` | `near_degenerate` | 129.66 us | 128.54 us - 130.93 us | 127.49 us | +1.49% |
| `orient2d` | `hyperlattice` | `easy` | 163.99 us | 154.80 us - 174.02 us | 139.28 us | +16.61% |
| `orient2d` | `hyperlattice` | `near_degenerate` | 246.43 us | 243.33 us - 251.09 us | 243.08 us | +72.74% |
| `orient3d` | `f32` | `easy` | 10.99 us | 10.37 us - 11.67 us | 9.31 us | +21.51% |
| `orient3d` | `f32` | `near_degenerate` | 10.87 us | 10.33 us - 11.46 us | 9.42 us | +16.65% |
| `orient3d` | `f64` | `easy` | 11.81 us | 11.08 us - 12.57 us | 9.49 us | +15.70% |
| `orient3d` | `f64` | `near_degenerate` | 13.35 us | 12.60 us - 14.12 us | 14.50 us | +15.14% |
| `orient3d` | `hyperreal` | `easy` | 245.95 us | 244.84 us - 247.15 us | 244.45 us | +0.65% |
| `orient3d` | `hyperreal` | `near_degenerate` | 155.85 us | 154.74 us - 157.06 us | 154.11 us | +8.92% |
| `orient3d` | `hyperlattice` | `easy` | 274.78 us | 272.60 us - 277.72 us | 271.80 us | +0.98% |
| `orient3d` | `hyperlattice` | `near_degenerate` | 181.59 us | 180.60 us - 182.62 us | 181.14 us | +26.42% |
