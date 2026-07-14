# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1784033982`.

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
| `classify_point_line` | `hyperreal` | `easy` | 772.09 us | 768.08 us - 776.71 us | 767.58 us | -0.71% |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 526.32 us | 525.04 us - 527.70 us | 524.94 us | -22.59% |
| `classify_point_line_fixed` | `hyperreal` | `easy` | 526.88 us | 524.26 us - 529.91 us | 522.63 us | -16.52% |
| `classify_point_line_fixed` | `hyperreal` | `near_degenerate` | 519.12 us | 516.67 us - 521.98 us | 515.62 us | -21.20% |
| `classify_point_line_fixed` | `hyperreal_prepared` | `easy` | 526.27 us | 524.42 us - 528.72 us | 523.99 us | -18.44% |
| `classify_point_line_fixed` | `hyperreal_prepared` | `near_degenerate` | 518.36 us | 514.98 us - 523.31 us | 512.57 us | -22.77% |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 1.86 ms | 1.85 ms - 1.86 ms | 1.85 ms | +3.78% |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 1.91 ms | 1.89 ms - 1.92 ms | 1.89 ms | +6.80% |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `easy` | 480.93 us | 479.82 us - 482.28 us | 479.59 us | +40.82% |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `near_degenerate` | 487.70 us | 485.94 us - 489.73 us | 484.72 us | +43.27% |
| `classify_point_plane` | `hyperreal` | `easy` | 259.44 us | 258.63 us - 260.34 us | 258.60 us | +10.10% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 148.62 us | 148.28 us - 148.98 us | 148.62 us | -42.03% |
| `classify_point_plane` | `hyperreal_prepared` | `easy` | 342.16 us | 341.05 us - 343.40 us | 340.70 us | -34.07% |
| `classify_point_plane` | `hyperreal_prepared` | `near_degenerate` | 234.61 us | 232.84 us - 236.85 us | 232.43 us | -29.00% |
| `incircle2d` | `hyperreal` | `easy` | 3.75 ms | 3.74 ms - 3.76 ms | 3.75 ms | +129.44% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 3.83 ms | 3.82 ms - 3.84 ms | 3.82 ms | +139.86% |
| `incircle2d` | `hyperreal_prepared` | `easy` | 4.06 ms | 3.98 ms - 4.14 ms | 3.94 ms | +154.01% |
| `incircle2d` | `hyperreal_prepared` | `near_degenerate` | 3.78 ms | 3.77 ms - 3.80 ms | 3.76 ms | +136.40% |
| `insphere3d` | `hyperreal` | `easy` | 9.94 ms | 9.91 ms - 9.99 ms | 9.88 ms | +130.29% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 9.83 ms | 9.81 ms - 9.85 ms | 9.82 ms | +125.27% |
| `insphere3d` | `hyperreal_prepared` | `easy` | 10.04 ms | 9.98 ms - 10.11 ms | 9.92 ms | +131.13% |
| `insphere3d` | `hyperreal_prepared` | `near_degenerate` | 9.83 ms | 9.82 ms - 9.84 ms | 9.83 ms | +129.51% |
| `orient2d` | `hyperreal` | `easy` | 767.95 us | 763.22 us - 773.59 us | 760.25 us | +1.98% |
| `orient2d` | `hyperreal` | `near_degenerate` | 530.41 us | 528.24 us - 532.92 us | 526.73 us | -20.22% |
| `orient3d` | `hyperreal` | `easy` | 1.92 ms | 1.91 ms - 1.93 ms | 1.91 ms | +10.22% |
| `orient3d` | `hyperreal` | `near_degenerate` | 1.41 ms | 1.41 ms - 1.42 ms | 1.40 ms | +39.13% |
