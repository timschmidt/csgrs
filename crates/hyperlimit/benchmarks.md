# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1784047647`.

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
| `incircle2d` | `hyperreal` | `easy` | 119.44 us | 116.45 us - 124.02 us | 116.42 us | -5.69% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 121.61 us | 118.55 us - 126.32 us | 118.61 us | -4.63% |
| `incircle2d` | `hyperreal_prepared` | `easy` | 128.48 us | 124.68 us - 133.15 us | 122.43 us | -96.60% |
| `incircle2d` | `hyperreal_prepared` | `near_degenerate` | 121.54 us | 121.12 us - 121.99 us | 121.22 us | -96.84% |
| `insphere3d` | `hyperreal` | `easy` | 195.40 us | 193.89 us - 197.07 us | 194.19 us | -11.16% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 193.22 us | 192.34 us - 194.29 us | 192.32 us | -6.87% |
| `insphere3d` | `hyperreal_prepared` | `easy` | 202.07 us | 201.15 us - 203.24 us | 200.85 us | -97.99% |
| `insphere3d` | `hyperreal_prepared` | `near_degenerate` | 214.85 us | 207.01 us - 224.35 us | 202.98 us | -97.81% |
| `orient2d` | `hyperreal` | `easy` | 121.13 us | 120.77 us - 121.58 us | 120.81 us | -11.95% |
| `orient2d` | `hyperreal` | `near_degenerate` | 111.50 us | 111.30 us - 111.73 us | 111.61 us | -3.59% |
| `orient3d` | `hyperreal` | `easy` | 258.51 us | 257.41 us - 260.11 us | 257.38 us | -3.51% |
| `orient3d` | `hyperreal` | `near_degenerate` | 138.25 us | 131.70 us - 146.05 us | 129.45 us | -1.23% |
