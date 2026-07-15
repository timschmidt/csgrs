# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1784101626`.

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
| `classify_point_line` | `hyperreal` | `easy` | 123.56 us | 123.33 us - 123.81 us | 123.18 us | +1.37% |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 114.54 us | 114.14 us - 115.03 us | 113.77 us | +1.63% |
| `classify_point_line_fixed` | `hyperreal` | `easy` | 111.78 us | 111.59 us - 111.98 us | 111.31 us | -1.32% |
| `classify_point_line_fixed` | `hyperreal` | `near_degenerate` | 111.49 us | 111.22 us - 111.78 us | 110.91 us | +1.01% |
| `classify_point_line_fixed` | `hyperreal_prepared` | `easy` | 40.37 us | 40.08 us - 40.74 us | 39.90 us | -3.31% |
| `classify_point_line_fixed` | `hyperreal_prepared` | `near_degenerate` | 41.00 us | 40.80 us - 41.24 us | 40.61 us | -6.70% |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 230.51 us | 229.61 us - 231.53 us | 229.38 us | -4.17% |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 235.23 us | 233.83 us - 236.71 us | 231.62 us | +0.94% |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `easy` | 71.95 us | 71.44 us - 72.63 us | 70.94 us | -1.28% |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `near_degenerate` | 72.75 us | 72.27 us - 73.30 us | 71.95 us | +1.47% |
| `classify_point_plane` | `hyperreal` | `easy` | 139.78 us | 139.23 us - 140.39 us | 138.87 us | -45.48% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 141.60 us | 141.04 us - 142.20 us | 140.11 us | -4.51% |
| `classify_point_plane` | `hyperreal_prepared` | `easy` | 73.23 us | 71.62 us - 75.19 us | 70.33 us | -78.41% |
| `classify_point_plane` | `hyperreal_prepared` | `near_degenerate` | 67.00 us | 66.88 us - 67.13 us | 66.94 us | -71.04% |
| `incircle2d` | `hyperreal` | `easy` | 110.54 us | 110.10 us - 111.03 us | 110.18 us | -0.19% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 111.70 us | 111.39 us - 112.05 us | 111.09 us | -5.35% |
| `incircle2d` | `hyperreal_prepared` | `easy` | 50.79 us | 50.74 us - 50.85 us | 50.72 us | -1.43% |
| `incircle2d` | `hyperreal_prepared` | `near_degenerate` | 50.08 us | 50.02 us - 50.15 us | 50.05 us | -0.08% |
| `insphere3d` | `hyperreal` | `easy` | 188.59 us | 188.06 us - 189.22 us | 188.09 us | -1.60% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 190.87 us | 190.27 us - 191.59 us | 190.02 us | -0.53% |
| `insphere3d` | `hyperreal_prepared` | `easy` | 101.04 us | 100.95 us - 101.13 us | 100.98 us | -0.10% |
| `insphere3d` | `hyperreal_prepared` | `near_degenerate` | 102.70 us | 102.38 us - 103.06 us | 101.99 us | +0.45% |
| `orient2d` | `hyperreal` | `easy` | 122.87 us | 122.61 us - 123.13 us | 122.88 us | +2.06% |
| `orient2d` | `hyperreal` | `near_degenerate` | 112.39 us | 112.27 us - 112.53 us | 112.26 us | +0.24% |
| `orient3d` | `hyperreal` | `easy` | 266.14 us | 264.84 us - 267.66 us | 263.73 us | -0.15% |
| `orient3d` | `hyperreal` | `near_degenerate` | 134.53 us | 134.16 us - 134.96 us | 134.02 us | -3.82% |
