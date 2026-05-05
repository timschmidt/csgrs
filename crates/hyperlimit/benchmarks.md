# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1777943976`.

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
| `classify_point_line` | `f32` | `easy` | 3.14 us | 3.05 us - 3.28 us | 3.07 us | -2.04% |
| `classify_point_line` | `f32` | `near_degenerate` | 2.72 us | 2.69 us - 2.76 us | 2.71 us | +7.36% |
| `classify_point_line` | `f64` | `easy` | 2.74 us | 2.72 us - 2.77 us | 2.73 us | +2.97% |
| `classify_point_line` | `f64` | `near_degenerate` | 2.80 us | 2.75 us - 2.84 us | 2.79 us | +8.40% |
| `classify_point_oriented_plane` | `f32` | `easy` | 6.63 us | 6.46 us - 6.80 us | 6.59 us | +7.66% |
| `classify_point_oriented_plane` | `f32` | `near_degenerate` | 6.73 us | 6.47 us - 7.05 us | 6.49 us | +3.64% |
| `classify_point_oriented_plane` | `f64` | `easy` | 5.99 us | 5.80 us - 6.31 us | 5.88 us | +6.07% |
| `classify_point_oriented_plane` | `f64` | `near_degenerate` | 6.12 us | 5.98 us - 6.28 us | 6.06 us | +7.56% |
| `classify_point_plane` | `f32` | `easy` | 4.43 us | 4.33 us - 4.55 us | 4.36 us | +5.68% |
| `classify_point_plane` | `f32` | `near_degenerate` | 4.45 us | 4.31 us - 4.61 us | 4.32 us | +3.68% |
| `classify_point_plane` | `f64` | `easy` | 4.29 us | 4.23 us - 4.38 us | 4.29 us | -0.80% |
| `classify_point_plane` | `f64` | `near_degenerate` | 4.66 us | 4.23 us - 5.26 us | 4.24 us | +11.37% |
| `incircle2d` | `f32` | `easy` | 755.71 ns | 746.29 ns - 765.91 ns | 747.71 ns | +110.67% |
| `incircle2d` | `f32` | `near_degenerate` | 759.77 ns | 749.70 ns - 770.79 ns | 757.20 ns | +103.85% |
| `incircle2d` | `f64` | `easy` | 751.33 ns | 742.58 ns - 762.28 ns | 743.33 ns | +105.82% |
| `incircle2d` | `f64` | `near_degenerate` | 772.68 ns | 756.85 ns - 789.69 ns | 767.00 ns | +110.75% |
| `insphere3d` | `f32` | `easy` | 6.25 us | 6.12 us - 6.38 us | 6.27 us | +2.91% |
| `insphere3d` | `f32` | `near_degenerate` | 6.01 us | 5.92 us - 6.11 us | 5.98 us | +5.64% |
| `insphere3d` | `f64` | `easy` | 5.50 us | 5.44 us - 5.57 us | 5.45 us | +3.96% |
| `insphere3d` | `f64` | `near_degenerate` | 5.47 us | 5.45 us - 5.49 us | 5.47 us | +3.98% |
| `orient2d` | `f32` | `easy` | 3.13 us | 3.07 us - 3.22 us | 3.10 us | +4.33% |
| `orient2d` | `f32` | `near_degenerate` | 2.71 us | 2.65 us - 2.80 us | 2.66 us | +6.69% |
| `orient2d` | `f64` | `easy` | 2.67 us | 2.65 us - 2.68 us | 2.66 us | -3.12% |
| `orient2d` | `f64` | `near_degenerate` | 2.73 us | 2.71 us - 2.75 us | 2.74 us | +4.01% |
| `orient3d` | `f32` | `easy` | 6.73 us | 6.48 us - 7.13 us | 6.54 us | +7.45% |
| `orient3d` | `f32` | `near_degenerate` | 6.75 us | 6.52 us - 7.01 us | 6.60 us | +9.22% |
| `orient3d` | `f64` | `easy` | 6.01 us | 5.91 us - 6.10 us | 6.08 us | +7.86% |
| `orient3d` | `f64` | `near_degenerate` | 6.20 us | 5.92 us - 6.51 us | 6.02 us | +7.58% |
