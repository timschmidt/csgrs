# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1785221176`.

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
| `classify_point_line` | `hyperreal` | `easy` | 13.41 us | 13.40 us - 13.43 us | 13.39 us | - |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 13.61 us | 13.57 us - 13.65 us | 13.55 us | - |
| `classify_point_line_fixed` | `hyperreal` | `easy` | 14.07 us | 14.05 us - 14.08 us | 14.05 us | - |
| `classify_point_line_fixed` | `hyperreal` | `near_degenerate` | 13.19 us | 13.13 us - 13.27 us | 13.07 us | - |
| `classify_point_line_fixed` | `hyperreal_oriented` | `easy` | 8.69 us | 8.67 us - 8.70 us | 8.67 us | - |
| `classify_point_line_fixed` | `hyperreal_oriented` | `near_degenerate` | 8.08 us | 8.06 us - 8.11 us | 8.05 us | - |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 37.18 us | 37.09 us - 37.29 us | 37.09 us | -3.59% |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 36.85 us | 36.80 us - 36.89 us | 36.77 us | -5.60% |
| `classify_point_oriented_plane` | `hyperreal_evidence` | `easy` | 19.12 us | 19.07 us - 19.18 us | 19.02 us | - |
| `classify_point_oriented_plane` | `hyperreal_evidence` | `near_degenerate` | 19.39 us | 19.37 us - 19.41 us | 19.36 us | - |
| `classify_point_plane` | `hyperreal` | `easy` | 20.86 us | 20.82 us - 20.92 us | 20.87 us | -4.24% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 19.21 us | 19.17 us - 19.25 us | 19.14 us | -2.34% |
| `classify_point_plane` | `hyperreal_evidence` | `easy` | 8.83 us | 8.81 us - 8.84 us | 8.80 us | +0.42% |
| `classify_point_plane` | `hyperreal_evidence` | `near_degenerate` | 7.53 us | 7.50 us - 7.57 us | 7.48 us | -2.01% |
| `incircle2d` | `hyperreal` | `easy` | 23.19 us | 23.12 us - 23.26 us | 23.13 us | -2.94% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 23.36 us | 23.19 us - 23.55 us | 23.16 us | -0.48% |
| `incircle2d` | `hyperreal_evidence` | `easy` | 16.86 us | 16.81 us - 16.91 us | 16.80 us | -0.15% |
| `incircle2d` | `hyperreal_evidence` | `near_degenerate` | 16.83 us | 16.79 us - 16.88 us | 16.73 us | -1.09% |
| `incircle2d` | `robust` | `easy` | 2.81 us | 2.81 us - 2.82 us | 2.80 us | - |
| `incircle2d` | `robust` | `near_degenerate` | 2.91 us | 2.91 us - 2.92 us | 2.91 us | - |
| `insphere3d` | `hyperreal` | `easy` | 64.48 us | 64.10 us - 64.94 us | 63.95 us | +0.77% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 65.63 us | 65.42 us - 65.87 us | 65.29 us | -2.50% |
| `insphere3d` | `hyperreal_evidence` | `easy` | 40.56 us | 40.45 us - 40.69 us | 40.35 us | -0.36% |
| `insphere3d` | `hyperreal_evidence` | `near_degenerate` | 41.89 us | 41.56 us - 42.26 us | 41.28 us | +0.95% |
| `insphere3d` | `robust` | `easy` | 13.05 us | 13.03 us - 13.08 us | 13.01 us | - |
| `insphere3d` | `robust` | `near_degenerate` | 13.10 us | 13.08 us - 13.12 us | 13.08 us | - |
| `orient2d` | `hyperreal` | `easy` | 12.39 us | 12.26 us - 12.56 us | 12.16 us | -1.60% |
| `orient2d` | `hyperreal` | `near_degenerate` | 12.25 us | 12.24 us - 12.26 us | 12.23 us | -2.04% |
| `orient2d` | `robust` | `easy` | 1.22 us | 1.21 us - 1.22 us | 1.21 us | - |
| `orient2d` | `robust` | `near_degenerate` | 1.27 us | 1.27 us - 1.27 us | 1.27 us | - |
| `orient3d` | `hyperreal` | `easy` | 43.54 us | 43.33 us - 43.80 us | 43.50 us | -0.44% |
| `orient3d` | `hyperreal` | `near_degenerate` | 38.34 us | 38.23 us - 38.46 us | 38.45 us | +1.10% |
| `orient3d` | `robust` | `easy` | 8.94 us | 8.92 us - 8.95 us | 8.93 us | - |
| `orient3d` | `robust` | `near_degenerate` | 8.94 us | 8.90 us - 9.00 us | 8.85 us | - |
