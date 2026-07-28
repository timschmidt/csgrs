# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1785196416`.

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
| `classify_point_plane` | `hyperreal` | `easy` | 20.87 us | 20.77 us - 20.97 us | 21.13 us | +0.94% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 19.55 us | 19.46 us - 19.65 us | 19.77 us | +0.84% |
| `classify_point_plane` | `hyperreal_evidence` | `easy` | 8.89 us | 8.85 us - 8.94 us | 8.84 us | - |
| `classify_point_plane` | `hyperreal_evidence` | `near_degenerate` | 7.42 us | 7.41 us - 7.44 us | 7.42 us | - |
| `incircle2d` | `hyperreal` | `easy` | 23.43 us | 23.34 us - 23.51 us | 23.56 us | - |
| `incircle2d` | `hyperreal` | `near_degenerate` | 23.38 us | 23.35 us - 23.42 us | 23.34 us | - |
| `incircle2d` | `hyperreal_evidence` | `easy` | 18.71 us | 18.62 us - 18.83 us | 18.64 us | - |
| `incircle2d` | `hyperreal_evidence` | `near_degenerate` | 17.64 us | 17.51 us - 17.82 us | 17.42 us | - |
| `incircle2d` | `robust` | `easy` | 2.81 us | 2.81 us - 2.82 us | 2.80 us | - |
| `incircle2d` | `robust` | `near_degenerate` | 2.91 us | 2.91 us - 2.92 us | 2.91 us | - |
| `insphere3d` | `hyperreal` | `easy` | 64.25 us | 63.76 us - 64.82 us | 63.41 us | - |
| `insphere3d` | `hyperreal` | `near_degenerate` | 64.69 us | 64.49 us - 64.92 us | 64.40 us | - |
| `insphere3d` | `hyperreal_evidence` | `easy` | 39.96 us | 39.93 us - 40.01 us | 39.90 us | - |
| `insphere3d` | `hyperreal_evidence` | `near_degenerate` | 40.29 us | 40.24 us - 40.36 us | 40.17 us | - |
| `insphere3d` | `robust` | `easy` | 13.05 us | 13.03 us - 13.08 us | 13.01 us | - |
| `insphere3d` | `robust` | `near_degenerate` | 13.10 us | 13.08 us - 13.12 us | 13.08 us | - |
| `orient2d` | `hyperreal` | `easy` | 12.44 us | 12.43 us - 12.45 us | 12.43 us | - |
| `orient2d` | `hyperreal` | `near_degenerate` | 12.54 us | 12.52 us - 12.56 us | 12.50 us | - |
| `orient2d` | `robust` | `easy` | 1.22 us | 1.21 us - 1.22 us | 1.21 us | - |
| `orient2d` | `robust` | `near_degenerate` | 1.27 us | 1.27 us - 1.27 us | 1.27 us | - |
| `orient3d` | `hyperreal` | `easy` | 43.34 us | 43.28 us - 43.39 us | 43.27 us | - |
| `orient3d` | `hyperreal` | `near_degenerate` | 39.28 us | 39.18 us - 39.40 us | 39.12 us | - |
| `orient3d` | `robust` | `easy` | 8.94 us | 8.92 us - 8.95 us | 8.93 us | - |
| `orient3d` | `robust` | `near_degenerate` | 8.94 us | 8.90 us - 9.00 us | 8.85 us | - |
