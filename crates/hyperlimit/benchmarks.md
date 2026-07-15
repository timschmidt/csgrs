# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1784102736`.

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
| `classify_point_line` | `hyperreal` | `easy` | 124.47 us | 123.65 us - 125.43 us | 123.47 us | +0.73% |
| `classify_point_line` | `hyperreal` | `near_degenerate` | 115.65 us | 115.36 us - 116.00 us | 115.21 us | +0.97% |
| `classify_point_line_fixed` | `hyperreal` | `easy` | 117.69 us | 116.62 us - 118.85 us | 116.17 us | +5.29% |
| `classify_point_line_fixed` | `hyperreal` | `near_degenerate` | 118.08 us | 117.00 us - 119.26 us | 115.87 us | +5.92% |
| `classify_point_line_fixed` | `hyperreal_prepared` | `easy` | 41.33 us | 41.12 us - 41.57 us | 40.99 us | +5.00% |
| `classify_point_line_fixed` | `hyperreal_prepared` | `near_degenerate` | 42.15 us | 41.88 us - 42.45 us | 41.69 us | +4.08% |
| `classify_point_oriented_plane` | `hyperreal` | `easy` | 249.69 us | 245.30 us - 254.74 us | 239.47 us | +8.32% |
| `classify_point_oriented_plane` | `hyperreal` | `near_degenerate` | 241.22 us | 239.31 us - 243.57 us | 237.92 us | +2.55% |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `easy` | 73.04 us | 72.52 us - 73.66 us | 72.33 us | +1.51% |
| `classify_point_oriented_plane` | `hyperreal_prepared` | `near_degenerate` | 76.11 us | 75.31 us - 77.05 us | 74.78 us | +4.18% |
| `classify_point_plane` | `hyperreal` | `easy` | 148.44 us | 146.17 us - 151.45 us | 144.22 us | +6.86% |
| `classify_point_plane` | `hyperreal` | `near_degenerate` | 146.98 us | 145.62 us - 148.52 us | 144.35 us | +4.81% |
| `classify_point_plane` | `hyperreal_prepared` | `easy` | 74.67 us | 73.71 us - 75.73 us | 73.00 us | +8.58% |
| `classify_point_plane` | `hyperreal_prepared` | `near_degenerate` | 73.53 us | 72.95 us - 74.20 us | 72.55 us | +6.46% |
| `incircle2d` | `hyperreal` | `easy` | 114.22 us | 113.36 us - 115.18 us | 112.50 us | +3.33% |
| `incircle2d` | `hyperreal` | `near_degenerate` | 116.04 us | 115.59 us - 116.56 us | 115.41 us | +3.89% |
| `incircle2d` | `hyperreal_prepared` | `easy` | 52.93 us | 52.47 us - 53.44 us | 52.31 us | +4.22% |
| `incircle2d` | `hyperreal_prepared` | `near_degenerate` | 52.15 us | 51.71 us - 52.63 us | 51.26 us | +4.14% |
| `insphere3d` | `hyperreal` | `easy` | 199.09 us | 198.34 us - 200.00 us | 198.11 us | +5.57% |
| `insphere3d` | `hyperreal` | `near_degenerate` | 201.15 us | 197.43 us - 205.69 us | 194.90 us | +5.39% |
| `insphere3d` | `hyperreal_prepared` | `easy` | 109.35 us | 107.62 us - 111.59 us | 106.75 us | +8.22% |
| `insphere3d` | `hyperreal_prepared` | `near_degenerate` | 107.00 us | 106.31 us - 107.77 us | 105.57 us | +4.19% |
| `orient2d` | `hyperreal` | `easy` | 122.61 us | 122.11 us - 123.11 us | 122.88 us | -0.21% |
| `orient2d` | `hyperreal` | `near_degenerate` | 114.49 us | 114.09 us - 115.02 us | 113.99 us | +1.87% |
| `orient3d` | `hyperreal` | `easy` | 273.09 us | 271.62 us - 274.77 us | 270.58 us | +2.61% |
| `orient3d` | `hyperreal` | `near_degenerate` | 141.61 us | 140.56 us - 142.81 us | 139.82 us | +5.26% |
