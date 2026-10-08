# exactCorelib / Hyper complete memory-sweep results

Run date: 2026-08-21 (America/Detroit)

This is the exhaustive generated appendix for the full allocator-instrumented
campaign. It records all 160 fixed comparable operations plus the 36 retained
geometry scaling configurations. Each exactCorelib/Hyper measurement ran in an
isolated worker and traversed its retained fixture twice.

Raw TSV: [`results/memory-sweep-2026-08-21.tsv`](results/memory-sweep-2026-08-21.tsv)
SHA-256: `ac62c41a0d60d6a670e7c4535b56303f73b870f8d7992eee39da50058c5599ac`

The curated interpretation, methodology, and limits are in
[MEMORY_BENCHMARK_REPORT.md](MEMORY_BENCHMARK_REPORT.md). Values below are
logical requested payload bytes unless the column explicitly names RSS or HWM.

## Generated memory-sweep analysis

Input: `results/memory-sweep-2026-08-21.tsv`
392 worker measurements / 196 paired configurations.

### Largest streaming fixtures

| Workload | Cases | Retained exact / Hyper | Execution allocation traffic per item, exact / Hyper | Post-operation growth exact / Hyper | Ephemeral peak exact / Hyper | Residual exact / Hyper |
|---|---:|---:|---:|---:|---:|---:|
| line-point | 8192 | 21.06 MiB / 6.96 MiB (3.03×) | 1.53 KiB / 0 B (∞) | 648 B / 0 B | 2.50 KiB / 0 B | 0 B / 1.93 KiB |
| triangle-point | 4096 | 18.91 MiB / 5.29 MiB (3.58×) | 14.99 KiB / 659 B (23.29×) | 648 B / 5.15 MiB | 11.02 KiB / 0 B | 0 B / 15.15 KiB |
| triangle-pair | 1024 | 7.09 MiB / 1.80 MiB (3.95×) | 164.34 KiB / 3.59 KiB (45.73×) | 113.11 MiB / 2.71 MiB | 5.73 KiB / 904 B | 120.00 MiB / 43.72 KiB |

### Fitted retained/residual scaling

| Workload | Library | Retained slope | Residual slope per fixture case | Residual per processed item |
|---|---|---:|---:|---:|
| line-point | exactCorelib | 2696.0 B/case | 0.0 B/case | 0.0 B/result |
| line-point | Hyper | 895.1 B/case | 0.1 B/case | 0.1 B/result |
| triangle-point | exactCorelib | 4840.0 B/case | 0.0 B/case | 0.0 B/result |
| triangle-point | Hyper | 1358.5 B/case | 1.1 B/case | 0.6 B/result |
| triangle-pair | exactCorelib | 7264.0 B/case | 122879.7 B/case | 61439.8 B/result |
| triangle-pair | Hyper | 1840.7 B/case | 19.9 B/case | 10.0 B/result |

### Materialized native-output footprint

| Workload | Cases | exactCore | Hyper |
|---|---:|---:|---:|
| line-point | 8192 | 4.0 B/result | 3.0 B/result |
| triangle-point | 4096 | 4.0 B/result | 3.0 B/result |
| triangle-pair | 1024 | 4.0 B/result | 56.0 B/result |

### Comparable-operation coverage

160 concrete retained operations were measured.

| Operation | Retained exact / Hyper | Traffic per iteration exact / Hyper | Post-op growth exact / Hyper | Ephemeral peak exact / Hyper | Residual exact / Hyper |
|---|---:|---:|---:|---:|---:|
| bivariate.evaluate | 608 B / 992 B | 96 B / 480 B | 0 B / 768 B | 80 B / 96 B | 0 B / 1.46 KiB |
| bivariate.resultant | 608 B / 1.23 KiB | 12.74 KiB / 2.68 KiB | 512 B / 840 B | 1.23 KiB / 984 B | 760 B / 1.53 KiB |
| complex.add | 304 B / 616 B | 176 B / 384 B | 0 B / 560 B | 144 B / 0 B | 0 B / 976 B |
| complex.divide | 304 B / 616 B | 880 B / 208 B | 0 B / 0 B | 432 B / 208 B | 0 B / 416 B |
| complex.multiply | 304 B / 616 B | 480 B / 872 B | 0 B / 1.40 KiB | 288 B / 104 B | 0 B / 1.80 KiB |
| complex.norm_squared | 304 B / 304 B | 240 B / 156 B | 0 B / 104 B | 216 B / 104 B | 0 B / 312 B |
| complex.subtract | 304 B / 616 B | 176 B / 384 B | 0 B / 560 B | 144 B / 0 B | 0 B / 976 B |
| curve.circle_circle_relation | 3.31 KiB / 3.01 KiB | 1.23 KiB / 668 B | 0 B / 1008 B | 1.20 KiB / 216 B | 0 B / 2.65 KiB |
| curve.circle_point_distance | 3.31 KiB / 2.18 KiB | 776 B / 880 B | 0 B / 0 B | 776 B / 600 B | 0 B / 1.39 KiB |
| curve.line_contains_point | 3.33 KiB / 2.19 KiB | 920 B / 0 B | 0 B / 0 B | 920 B / 0 B | 0 B / 1.80 KiB |
| curve.line_intersection_topology | 3.33 KiB / 3.02 KiB | 7.39 KiB / 644 B | 448 B / 664 B | 2.71 KiB / 312 B | 0 B / 3.13 KiB |
| curve.line_intersection_witness | 3.83 KiB / 3.02 KiB | 3.56 KiB / 644 B | 0 B / 664 B | 1.91 KiB / 312 B | 0 B / 3.13 KiB |
| curve.line_length | 1.66 KiB / 1.97 KiB | 664 B / 456 B | 0 B / 0 B | 664 B / 352 B | 0 B / 1.70 KiB |
| curve.line_side | 3.83 KiB / 2.19 KiB | 672 B / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 1.80 KiB |
| curve.segment_dispatch | 3.33 KiB / 3.02 KiB | 7.39 KiB / 644 B | 448 B / 664 B | 2.71 KiB / 312 B | 0 B / 3.13 KiB |
| curve.supporting_line_circle | 3.57 KiB / 2.88 KiB | 2.35 KiB / 868 B | 0 B / 488 B | 1.96 KiB / 624 B | 0 B / 2.41 KiB |
| geometry2.area | 2.41 KiB / 808 B | 784 B / 296 B | 0 B / 488 B | 784 B / 0 B | 0 B / 1008 B |
| geometry2.between | 2.41 KiB / 808 B | 672 B / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 520 B |
| geometry2.circle_circle_distance | 3.31 KiB / 656 B | 1.23 KiB / 1.68 KiB | 0 B / 104 B | 1.20 KiB / 968 B | 0 B / 520 B |
| geometry2.circle_line | 3.57 KiB / 856 B | 2.35 KiB / 1.11 KiB | 0 B / 2.02 KiB | 1.96 KiB / 208 B | 0 B / 2.52 KiB |
| geometry2.circle_point_distance | 3.31 KiB / 656 B | 776 B / 932 B | 0 B / 104 B | 776 B / 600 B | 0 B / 520 B |
| geometry2.circle_segment | 2.87 KiB / 856 B | 5.27 KiB / 1.93 KiB | 0 B / 3.14 KiB | 4.50 KiB / 416 B | 0 B / 3.65 KiB |
| geometry2.incircle | 3.30 KiB / 1008 B | 3.89 KiB / 0 B | 0 B / 0 B | 3.50 KiB / 0 B | 0 B / 624 B |
| geometry2.line_intersection | 3.83 KiB / 1.09 KiB | 3.56 KiB / 208 B | 0 B / 0 B | 1.91 KiB / 208 B | 0 B / 728 B |
| geometry2.line_point_distance | 3.83 KiB / 808 B | 1.98 KiB / 1.09 KiB | 0 B / 0 B | 1.95 KiB / 1016 B | 0 B / 520 B |
| geometry2.line_relation_0 | 3.83 KiB / 1.09 KiB | 672 B / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 728 B |
| geometry2.line_relation_1 | 3.83 KiB / 1.09 KiB | 672 B / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 728 B |
| geometry2.line_relation_2 | 3.83 KiB / 1.09 KiB | 952 B / 384 B | 0 B / 560 B | 920 B / 104 B | 0 B / 1.26 KiB |
| geometry2.line_relation_3 | 3.83 KiB / 1.09 KiB | 1.59 KiB / 384 B | 0 B / 560 B | 920 B / 104 B | 0 B / 1.26 KiB |
| geometry2.line_relation_4 | 3.83 KiB / 1.09 KiB | 672 B / 384 B | 0 B / 560 B | 672 B / 104 B | 0 B / 1.26 KiB |
| geometry2.line_relation_5 | 3.83 KiB / 1.09 KiB | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 728 B |
| geometry2.line_relation_6 | 3.83 KiB / 1.09 KiB | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 728 B |
| geometry2.orientation | 2.41 KiB / 808 B | 672 B / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 520 B |
| geometry2.point_distance | 1.66 KiB / 504 B | 664 B / 804 B | 0 B / 1.37 KiB | 664 B / 0 B | 0 B / 1.67 KiB |
| geometry2.segment_point_distance | 3.33 KiB / 808 B | 5.26 KiB / 1.30 KiB | 0 B / 0 B | 4.60 KiB / 1.09 KiB | 0 B / 520 B |
| geometry2.segment_relation_0 | 3.33 KiB / 1.09 KiB | 920 B / 0 B | 0 B / 0 B | 920 B / 0 B | 0 B / 728 B |
| geometry2.segment_relation_1 | 3.33 KiB / 1.09 KiB | 7.39 KiB / 0 B | 448 B / 0 B | 2.71 KiB / 0 B | 0 B / 728 B |
| geometry2.segment_relation_2 | 3.33 KiB / 1.09 KiB | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 728 B |
| geometry2.segment_relation_3 | 3.33 KiB / 1.09 KiB | 1.41 KiB / 384 B | 0 B / 560 B | 1.38 KiB / 104 B | 0 B / 1.26 KiB |
| geometry3.line_point_distance | 5.66 KiB / 1.13 KiB | 3.72 KiB / 2.36 KiB | 0 B / 1.16 KiB | 3.66 KiB / 1.71 KiB | 0 B / 1.87 KiB |
| geometry3.line_relation_0 | 5.66 KiB / 1.59 KiB | 1.73 KiB / 680 B | 0 B / 1.23 KiB | 1.70 KiB / 0 B | 0 B / 2.24 KiB |
| geometry3.line_relation_1 | 5.66 KiB / 1.59 KiB | 1.41 KiB / 1.09 KiB | 0 B / 1.30 KiB | 1.38 KiB / 592 B | 0 B / 2.31 KiB |
| geometry3.line_relation_2 | 5.66 KiB / 1.59 KiB | 2.78 KiB / 0 B | 0 B / 0 B | 2.63 KiB / 0 B | 0 B / 1.02 KiB |
| geometry3.line_relation_3 | 5.66 KiB / 1.59 KiB | 3.23 KiB / 0 B | 0 B / 0 B | 2.84 KiB / 0 B | 0 B / 1.02 KiB |
| geometry3.line_relation_4 | 5.66 KiB / 1.59 KiB | 1.73 KiB / 1.09 KiB | 0 B / 1.30 KiB | 1.70 KiB / 592 B | 0 B / 2.31 KiB |
| geometry3.orientation | 4.77 KiB / 1.58 KiB | 3.23 KiB / 0 B | 0 B / 0 B | 2.84 KiB / 0 B | 0 B / 1.02 KiB |
| geometry3.plane_point_distance | 13.27 KiB / 856 B | 1.88 KiB / 296 B | 0 B / 384 B | 1.85 KiB / 0 B | 0 B / 904 B |
| geometry3.plane_relation_0 | 13.27 KiB / 1.53 KiB | 1.02 KiB / 0 B | 0 B / 0 B | 1008 B / 0 B | 0 B / 936 B |
| geometry3.plane_relation_1 | 13.27 KiB / 1.53 KiB | 672 B / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 936 B |
| geometry3.plane_relation_2 | 13.27 KiB / 1.53 KiB | 1.02 KiB / 0 B | 0 B / 0 B | 1008 B / 0 B | 0 B / 936 B |
| geometry3.plane_relation_3 | 13.27 KiB / 1.53 KiB | 2.39 KiB / 0 B | 0 B / 0 B | 1.34 KiB / 0 B | 0 B / 936 B |
| geometry3.plane_relation_4 | 13.27 KiB / 1.53 KiB | 1.02 KiB / 0 B | 0 B / 0 B | 1008 B / 0 B | 0 B / 936 B |
| geometry3.plane_relation_5 | 13.27 KiB / 1.53 KiB | 2.80 KiB / 0 B | 0 B / 0 B | 1.75 KiB / 0 B | 0 B / 936 B |
| geometry3.plane_relation_6 | 13.27 KiB / 1.53 KiB | 1.38 KiB / 0 B | 0 B / 0 B | 1.34 KiB / 0 B | 0 B / 936 B |
| geometry3.plane_relation_7 | 13.27 KiB / 1.53 KiB | 5.55 KiB / 0 B | 6.83 KiB / 0 B | 1.39 KiB / 0 B | 0 B / 936 B |
| geometry3.point_distance | 2.45 KiB / 704 B | 1000 B / 1.38 KiB | 0 B / 2.56 KiB | 1000 B / 0 B | 0 B / 2.97 KiB |
| geometry3.segment_point_distance | 4.92 KiB / 1.13 KiB | 8.02 KiB / 2.56 KiB | 0 B / 1.16 KiB | 7.30 KiB / 1.81 KiB | 0 B / 1.87 KiB |
| geometry3.segment_relation_0 | 4.92 KiB / 1.59 KiB | 2.09 KiB / 680 B | 0 B / 1.23 KiB | 2.06 KiB / 0 B | 0 B / 2.24 KiB |
| geometry3.segment_relation_1 | 4.92 KiB / 1.59 KiB | 3.95 KiB / 1012 B | 0 B / 1.57 KiB | 3.56 KiB / 208 B | 0 B / 2.59 KiB |
| geometry3.segment_relation_2 | 4.92 KiB / 1.59 KiB | 0 B / 1012 B | 0 B / 1.57 KiB | 0 B / 208 B | 0 B / 2.59 KiB |
| geometry3.segment_relation_3 | 4.92 KiB / 1.59 KiB | 3.23 KiB / 0 B | 0 B / 0 B | 2.84 KiB / 0 B | 0 B / 1.02 KiB |
| geometry3.triangle_relation_0 | 7.85 KiB / 1.56 KiB | 3.23 KiB / 52 B | 0 B / 104 B | 2.84 KiB / 0 B | 0 B / 832 B |
| geometry3.triangle_relation_1 | 7.85 KiB / 1.56 KiB | 13.41 KiB / 52 B | 448 B / 104 B | 4.48 KiB / 0 B | 0 B / 832 B |
| geometry3.triangle_relation_2 | 7.85 KiB / 1.56 KiB | 8.00 KiB / 52 B | 0 B / 104 B | 3.42 KiB / 0 B | 0 B / 832 B |
| geometry3.triangle_relation_3 | 7.85 KiB / 1.56 KiB | 13.41 KiB / 52 B | 448 B / 104 B | 4.48 KiB / 0 B | 0 B / 832 B |
| geometry3.triangle_relation_4 | 7.85 KiB / 1.56 KiB | 18.65 KiB / 2.47 KiB | 0 B / 4.54 KiB | 4.51 KiB / 104 B | 0 B / 5.25 KiB |
| geometry3.triangle_relation_5 | 7.85 KiB / 1.56 KiB | 12.94 KiB / 2.47 KiB | 0 B / 4.54 KiB | 2.84 KiB / 104 B | 0 B / 5.25 KiB |
| geometry3.triangle_relation_6 | 7.85 KiB / 1.56 KiB | 22.83 KiB / 15.39 KiB | 13.91 KiB / 22.77 KiB | 5.27 KiB / 104 B | 19.65 KiB / 23.48 KiB |
| geometry3.volume | 4.77 KiB / 1.58 KiB | 6.68 KiB / 1.43 KiB | 0 B / 1.84 KiB | 5.46 KiB / 520 B | 0 B / 2.86 KiB |
| matrix3.add | 1.21 KiB / 1.99 KiB | 1.27 KiB / 576 B | 0 B / 528 B | 728 B / 312 B | 0 B / 1.63 KiB |
| matrix3.determinant | 1.21 KiB / 2.00 KiB | 2.10 KiB / 384 B | 0 B / 768 B | 724 B / 0 B | 0 B / 1.87 KiB |
| matrix3.inverse | 1.21 KiB / 1.99 KiB | 15.75 KiB / 1008 B | 0 B / 1.97 KiB | 3.11 KiB / 0 B | 0 B / 3.09 KiB |
| matrix3.multiply | 1.21 KiB / 1.99 KiB | 3.15 KiB / 416 B | 0 B / 832 B | 848 B / 0 B | 0 B / 1.93 KiB |
| matrix3.subtract | 1.21 KiB / 1.99 KiB | 1.27 KiB / 820 B | 0 B / 1016 B | 728 B / 312 B | 0 B / 2.11 KiB |
| matrix3.transpose | 1.21 KiB / 1.99 KiB | 512 B / 0 B | 0 B / 0 B | 512 B / 0 B | 0 B / 1.12 KiB |
| matrix4.add | 2.05 KiB / 2.44 KiB | 2.23 KiB / 784 B | 0 B / 528 B | 1.26 KiB / 520 B | 0 B / 1.43 KiB |
| matrix4.determinant | 2.05 KiB / 2.45 KiB | 5.17 KiB / 1.52 KiB | 0 B / 2.84 KiB | 1.12 KiB / 104 B | 0 B / 3.75 KiB |
| matrix4.inverse | 2.05 KiB / 2.44 KiB | 37.33 KiB / 1.63 KiB | 0 B / 0 B | 5.41 KiB / 1.63 KiB | 0 B / 936 B |
| matrix4.multiply | 2.05 KiB / 2.45 KiB | 6.38 KiB / 468 B | 0 B / 936 B | 1.38 KiB / 0 B | 0 B / 1.83 KiB |
| matrix4.subtract | 2.05 KiB / 2.45 KiB | 2.23 KiB / 1.54 KiB | 0 B / 2.05 KiB | 1.26 KiB / 520 B | 0 B / 2.97 KiB |
| matrix4.transpose | 2.05 KiB / 2.45 KiB | 904 B / 0 B | 0 B / 0 B | 904 B / 0 B | 0 B / 936 B |
| mesh.plane_point_classification | 13.27 KiB / 1.55 KiB | 3.71 KiB / 352 B | 3.40 KiB / 0 B | 2.02 KiB / 352 B | 0 B / 1.12 KiB |
| mesh.triangle_boundary_point | 7.85 KiB / 4.74 KiB | 13.08 KiB / 704 B | 448 B / 0 B | 5.91 KiB / 352 B | 0 B / 2.11 KiB |
| mesh.triangle_contains_point | 7.85 KiB / 4.84 KiB | 13.41 KiB / 352 B | 448 B / 0 B | 4.48 KiB / 352 B | 0 B / 2.21 KiB |
| mesh.triangle_contains_point_strictly | 7.85 KiB / 4.84 KiB | 13.41 KiB / 352 B | 448 B / 0 B | 4.48 KiB / 352 B | 0 B / 2.21 KiB |
| mesh.triangle_triangle_intersection | 7.85 KiB / 8.70 KiB | 34.74 KiB / 2.68 KiB | 32.52 KiB / 0 B | 5.75 KiB / 2.38 KiB | 37.82 KiB / 3.61 KiB |
| path.circle_circle_relation | 3.31 KiB / 2.71 KiB | 1.23 KiB / 208 B | 0 B / 208 B | 1.20 KiB / 104 B | 0 B / 1.66 KiB |
| path.circle_point_membership | 3.31 KiB / 2.01 KiB | 776 B / 0 B | 0 B / 0 B | 776 B / 0 B | 0 B / 1.29 KiB |
| path.circle_segment_intersection | 2.87 KiB / 2.68 KiB | 6.85 KiB / 2.54 KiB | 0 B / 3.16 KiB | 6.09 KiB / 576 B | 0 B / 4.73 KiB |
| path.line_axis_classification | 3.83 KiB / 1.58 KiB | 488 B / 0 B | 0 B / 0 B | 416 B / 0 B | 0 B / 1.09 KiB |
| path.line_endpoint_equality | 3.33 KiB / 2.72 KiB | 2.42 KiB / 0 B | 448 B / 0 B | 464 B / 0 B | 0 B / 1.73 KiB |
| path.line_length | 1.66 KiB / 1.95 KiB | 664 B / 400 B | 0 B / 176 B | 664 B / 208 B | 0 B / 1.63 KiB |
| path.line_parameter_order | 2.41 KiB / 1.59 KiB | 3.43 KiB / 0 B | 0 B / 0 B | 1.79 KiB / 0 B | 0 B / 936 B |
| polynomial.binary_0 | 416 B / 864 B | 488 B / 820 B | 0 B / 1.50 KiB | 296 B / 0 B | 0 B / 2.01 KiB |
| polynomial.binary_1 | 416 B / 864 B | 488 B / 820 B | 0 B / 1.50 KiB | 296 B / 0 B | 0 B / 2.01 KiB |
| polynomial.binary_2 | 416 B / 864 B | 1.38 KiB / 680 B | 0 B / 1.33 KiB | 496 B / 0 B | 0 B / 1.84 KiB |
| polynomial.binary_3 | 416 B / 864 B | 2.98 KiB / 2.36 KiB | 368 B / 1016 B | 608 B / 1.20 KiB | 368 B / 1.50 KiB |
| polynomial.binary_4 | 416 B / 864 B | 4.25 KiB / 1.04 KiB | 0 B / 1.98 KiB | 824 B / 0 B | 0 B / 2.48 KiB |
| polynomial.derivative | 416 B / 824 B | 736 B / 244 B | 0 B / 488 B | 280 B / 0 B | 0 B / 1.19 KiB |
| polynomial.discriminant | 416 B / 1.04 KiB | 7.07 KiB / 6.79 KiB | 128 B / 104 B | 1.00 KiB / 4.91 KiB | 128 B / 832 B |
| polynomial.evaluate | 416 B / 712 B | 48 B / 384 B | 0 B / 768 B | 40 B / 0 B | 0 B / 1.26 KiB |
| polynomial.gcd | 416 B / 856 B | 3.47 KiB / 2.30 KiB | 368 B / 912 B | 736 B / 1.20 KiB | 368 B / 1.40 KiB |
| polynomial.resultant | 416 B / 856 B | 3.33 KiB / 5.74 KiB | 368 B / 104 B | 640 B / 4.71 KiB | 368 B / 624 B |
| polynomial.root_count | 416 B / 2.36 KiB | 22.43 KiB / 13.86 KiB | 432 B / 11.42 KiB | 1.59 KiB / 2.88 KiB | 432 B / 12.03 KiB |
| polynomial.root_count_interval | 416 B / 2.36 KiB | 20.72 KiB / 13.86 KiB | 432 B / 11.42 KiB | 1.34 KiB / 2.88 KiB | 432 B / 12.03 KiB |
| polynomial.root_isolation | 416 B / 2.36 KiB | 69.04 KiB / 13.86 KiB | 432 B / 11.42 KiB | 1.59 KiB / 2.88 KiB | 432 B / 12.03 KiB |
| polynomial.square_free | 416 B / 712 B | 10.06 KiB / 1.06 KiB | 304 B / 312 B | 1.27 KiB / 816 B | 304 B / 832 B |
| rational.absolute | 176 B / 120 B | 56 B / 0 B | 0 B / 0 B | 56 B / 0 B | 0 B / 0 B |
| rational.add | 176 B / 232 B | 80 B / 192 B | 0 B / 280 B | 72 B / 0 B | 0 B / 0 B |
| rational.compare | 176 B / 224 B | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B |
| rational.divide | 176 B / 232 B | 80 B / 104 B | 0 B / 0 B | 72 B / 104 B | 0 B / 0 B |
| rational.multiply | 176 B / 232 B | 80 B / 52 B | 0 B / 104 B | 72 B / 0 B | 0 B / 0 B |
| rational.negate | 176 B / 120 B | 56 B / 280 B | 0 B / 560 B | 56 B / 0 B | 0 B / 104 B |
| rational.parts | 176 B / 112 B | 64 B / 0 B | 0 B / 0 B | 64 B / 0 B | 0 B / 0 B |
| rational.reciprocal | 176 B / 120 B | 56 B / 280 B | 0 B / 560 B | 56 B / 0 B | 0 B / 104 B |
| rational.subtract | 176 B / 232 B | 80 B / 192 B | 0 B / 280 B | 72 B / 0 B | 0 B / 0 B |
| real.absolute | 1.22 KiB / 160 B | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 104 B |
| real.acos | 1.22 KiB / 160 B | 1.72 KiB / 248 B | 0 B / 208 B | 888 B / 144 B | 0 B / 312 B |
| real.add | 1.22 KiB / 312 B | 112 B / 192 B | 0 B / 280 B | 112 B / 0 B | 0 B / 488 B |
| real.asin | 1.22 KiB / 160 B | 1.72 KiB / 300 B | 0 B / 312 B | 888 B / 144 B | 0 B / 416 B |
| real.atan | 1.22 KiB / 160 B | 472 B / 248 B | 0 B / 208 B | 336 B / 144 B | 0 B / 312 B |
| real.cbrt | 1.22 KiB / 160 B | 104 B / 992 B | 0 B / 208 B | 104 B / 600 B | 0 B / 312 B |
| real.ceil | 1.22 KiB / 152 B | 1.38 KiB / 208 B | 0 B / 0 B | 1.13 KiB / 104 B | 0 B / 104 B |
| real.cos | 1.22 KiB / 160 B | 104 B / 392 B | 0 B / 208 B | 104 B / 72 B | 0 B / 312 B |
| real.cot | 1.22 KiB / 160 B | 864 B / 1.03 KiB | 80 B / 664 B | 584 B / 288 B | 0 B / 768 B |
| real.divide | 1.22 KiB / 312 B | 112 B / 104 B | 0 B / 0 B | 112 B / 104 B | 0 B / 208 B |
| real.e | 16 B / 0 B | 96 B / 160 B | 0 B / 176 B | 96 B / 72 B | 0 B / 176 B |
| real.exp | 1.22 KiB / 160 B | 104 B / 196 B | 0 B / 104 B | 104 B / 144 B | 0 B / 208 B |
| real.exp10 | 1.22 KiB / 160 B | 104 B / 6.04 KiB | 0 B / 368 B | 104 B / 3.42 KiB | 0 B / 472 B |
| real.exp2 | 1.22 KiB / 160 B | 104 B / 4.87 KiB | 0 B / 208 B | 104 B / 1.25 KiB | 0 B / 312 B |
| real.floor | 1.22 KiB / 152 B | 1.02 KiB / 104 B | 0 B / 0 B | 824 B / 104 B | 0 B / 104 B |
| real.ln | 1.22 KiB / 160 B | 104 B / 544 B | 0 B / 944 B | 104 B / 72 B | 0 B / 1.02 KiB |
| real.log10 | 1.22 KiB / 160 B | 104 B / 724 B | 0 B / 872 B | 104 B / 288 B | 0 B / 976 B |
| real.log2 | 1.22 KiB / 160 B | 104 B / 724 B | 0 B / 872 B | 104 B / 288 B | 0 B / 976 B |
| real.multiply | 1.22 KiB / 312 B | 112 B / 52 B | 0 B / 104 B | 112 B / 0 B | 0 B / 312 B |
| real.negate | 1.22 KiB / 160 B | 104 B / 280 B | 0 B / 560 B | 104 B / 0 B | 0 B / 664 B |
| real.pi | 16 B / 0 B | 96 B / 160 B | 0 B / 176 B | 96 B / 72 B | 0 B / 176 B |
| real.powi | 1.22 KiB / 312 B | 272 B / 104 B | 0 B / 0 B | 216 B / 104 B | 0 B / 208 B |
| real.root_n | 1.22 KiB / 160 B | 272 B / 992 B | 40 B / 208 B | 224 B / 600 B | 0 B / 312 B |
| real.sign | 1.22 KiB / 152 B | 648 B / 392 B | 80 B / 208 B | 368 B / 72 B | 0 B / 312 B |
| real.sin | 1.22 KiB / 160 B | 104 B / 392 B | 0 B / 208 B | 104 B / 72 B | 0 B / 312 B |
| real.sqrt | 1.22 KiB / 160 B | 104 B / 632 B | 0 B / 768 B | 104 B / 144 B | 0 B / 872 B |
| real.square | 1.22 KiB / 160 B | 112 B / 52 B | 0 B / 104 B | 112 B / 0 B | 0 B / 208 B |
| real.subtract | 1.22 KiB / 312 B | 112 B / 140 B | 0 B / 280 B | 112 B / 0 B | 0 B / 488 B |
| real.tan | 1.22 KiB / 160 B | 816 B / 124 B | 80 B / 104 B | 584 B / 72 B | 0 B / 208 B |
| triangulation.delaunay_complex | 4.78 KiB / 1.95 KiB | 148.65 KiB / 80.58 KiB | 0 B / 56.75 KiB | 4.00 KiB / 1.61 KiB | 0 B / 57.97 KiB |
| vector2.add | 352 B / 635 B | 296 B / 228 B | 0 B / 456 B | 168 B / 0 B | 0 B / 872 B |
| vector2.dot | 352 B / 635 B | 240 B / 104 B | 0 B / 208 B | 144 B / 0 B | 0 B / 624 B |
| vector2.norm | 872 B / 636 B | 920 B / 400 B | 0 B / 592 B | 888 B / 0 B | 0 B / 1008 B |
| vector2.subtract | 352 B / 640 B | 296 B / 228 B | 0 B / 456 B | 168 B / 0 B | 0 B / 872 B |
| vector2.wedge | 352 B / 637 B | 240 B / 104 B | 0 B / 208 B | 216 B / 0 B | 0 B / 624 B |
| vector3.add | 480 B / 939 B | 440 B / 368 B | 0 B / 736 B | 248 B / 0 B | 0 B / 1.33 KiB |
| vector3.cross | 480 B / 941 B | 896 B / 260 B | 0 B / 312 B | 824 B / 104 B | 0 B / 936 B |
| vector3.dot | 480 B / 939 B | 320 B / 104 B | 0 B / 0 B | 144 B / 104 B | 0 B / 624 B |
| vector3.norm | 1.25 KiB / 940 B | 1.12 KiB / 908 B | 0 B / 1.43 KiB | 1.09 KiB / 72 B | 0 B / 2.04 KiB |
| vector3.subtract | 480 B / 944 B | 440 B / 368 B | 0 B / 736 B | 248 B / 0 B | 0 B / 1.33 KiB |
| vector4.add | 608 B / 1.21 KiB | 584 B / 456 B | 0 B / 912 B | 328 B / 0 B | 0 B / 1.70 KiB |
| vector4.dot | 608 B / 1.21 KiB | 400 B / 104 B | 0 B / 0 B | 144 B / 104 B | 0 B / 832 B |
| vector4.norm | 1.63 KiB / 1.21 KiB | 1.34 KiB / 892 B | 0 B / 976 B | 1.30 KiB / 352 B | 0 B / 1.77 KiB |
| vector4.subtract | 608 B / 1.22 KiB | 584 B / 508 B | 0 B / 1016 B | 328 B / 0 B | 0 B / 1.80 KiB |

### Complete paired table

| Workload / operation | Mode | Cases | Retained exact / Hyper | Operation traffic exact / Hyper | Post-op growth exact / Hyper | Ephemeral peak exact / Hyper | Residual exact / Hyper | RSS Δ exact / Hyper | HWM Δ exact / Hyper |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| rational.add | streaming | 1 | 176 B / 232 B | 160 B / 384 B | 0 B / 280 B | 72 B / 0 B | 0 B / 0 B | 304.00 KiB / 420.00 KiB | 304.00 KiB / 420.00 KiB |
| rational.subtract | streaming | 1 | 176 B / 232 B | 160 B / 384 B | 0 B / 280 B | 72 B / 0 B | 0 B / 0 B | 292.00 KiB / 600.00 KiB | 292.00 KiB / 600.00 KiB |
| rational.multiply | streaming | 1 | 176 B / 232 B | 160 B / 104 B | 0 B / 104 B | 72 B / 0 B | 0 B / 0 B | 244.00 KiB / 468.00 KiB | 244.00 KiB / 468.00 KiB |
| rational.divide | streaming | 1 | 176 B / 232 B | 160 B / 208 B | 0 B / 0 B | 72 B / 104 B | 0 B / 0 B | 324.00 KiB / 452.00 KiB | 324.00 KiB / 452.00 KiB |
| rational.negate | streaming | 1 | 176 B / 120 B | 112 B / 560 B | 0 B / 560 B | 56 B / 0 B | 0 B / 104 B | 236.00 KiB / 336.00 KiB | 236.00 KiB / 464.00 KiB |
| rational.absolute | streaming | 1 | 176 B / 120 B | 112 B / 0 B | 0 B / 0 B | 56 B / 0 B | 0 B / 0 B | 248.00 KiB / 196.00 KiB | 248.00 KiB / 324.00 KiB |
| rational.reciprocal | streaming | 1 | 176 B / 120 B | 112 B / 560 B | 0 B / 560 B | 56 B / 0 B | 0 B / 104 B | 244.00 KiB / 324.00 KiB | 244.00 KiB / 452.00 KiB |
| rational.compare | streaming | 1 | 176 B / 224 B | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 212.00 KiB / 132.00 KiB | 276.00 KiB / 260.00 KiB |
| rational.parts | streaming | 1 | 176 B / 112 B | 128 B / 0 B | 0 B / 0 B | 64 B / 0 B | 0 B / 0 B | 244.00 KiB / 324.00 KiB | 244.00 KiB / 452.00 KiB |
| real.negate | streaming | 1 | 1.22 KiB / 160 B | 208 B / 560 B | 0 B / 560 B | 104 B / 0 B | 0 B / 664 B | 324.00 KiB / 660.00 KiB | 324.00 KiB / 660.00 KiB |
| real.absolute | streaming | 1 | 1.22 KiB / 160 B | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 104 B | 296.00 KiB / 812.00 KiB | 296.00 KiB / 812.00 KiB |
| real.sqrt | streaming | 1 | 1.22 KiB / 160 B | 208 B / 1.23 KiB | 0 B / 768 B | 104 B / 144 B | 0 B / 872 B | 296.00 KiB / 916.00 KiB | 296.00 KiB / 916.00 KiB |
| real.cbrt | streaming | 1 | 1.22 KiB / 160 B | 208 B / 1.94 KiB | 0 B / 208 B | 104 B / 600 B | 0 B / 312 B | 364.00 KiB / 1.07 MiB | 364.00 KiB / 1.07 MiB |
| real.root_n | streaming | 1 | 1.22 KiB / 160 B | 544 B / 1.94 KiB | 40 B / 208 B | 224 B / 600 B | 0 B / 312 B | 316.00 KiB / 924.00 KiB | 352.00 KiB / 924.00 KiB |
| real.exp | streaming | 1 | 1.22 KiB / 160 B | 208 B / 392 B | 0 B / 104 B | 104 B / 144 B | 0 B / 208 B | 296.00 KiB / 956.00 KiB | 296.00 KiB / 956.00 KiB |
| real.ln | streaming | 1 | 1.22 KiB / 160 B | 208 B / 1.06 KiB | 0 B / 944 B | 104 B / 72 B | 0 B / 1.02 KiB | 324.00 KiB / 992.00 KiB | 324.00 KiB / 992.00 KiB |
| real.log2 | streaming | 1 | 1.22 KiB / 160 B | 208 B / 1.41 KiB | 0 B / 872 B | 104 B / 288 B | 0 B / 976 B | 296.00 KiB / 920.00 KiB | 296.00 KiB / 920.00 KiB |
| real.log10 | streaming | 1 | 1.22 KiB / 160 B | 208 B / 1.41 KiB | 0 B / 872 B | 104 B / 288 B | 0 B / 976 B | 236.00 KiB / 832.00 KiB | 236.00 KiB / 832.00 KiB |
| real.sin | streaming | 1 | 1.22 KiB / 160 B | 208 B / 784 B | 0 B / 208 B | 104 B / 72 B | 0 B / 312 B | 292.00 KiB / 1020.00 KiB | 292.00 KiB / 1020.00 KiB |
| real.cos | streaming | 1 | 1.22 KiB / 160 B | 208 B / 784 B | 0 B / 208 B | 104 B / 72 B | 0 B / 312 B | 336.00 KiB / 1012.00 KiB | 336.00 KiB / 1012.00 KiB |
| real.tan | streaming | 1 | 1.22 KiB / 160 B | 1.59 KiB / 248 B | 80 B / 104 B | 584 B / 72 B | 0 B / 208 B | 288.00 KiB / 852.00 KiB | 288.00 KiB / 852.00 KiB |
| real.asin | streaming | 1 | 1.22 KiB / 160 B | 3.44 KiB / 600 B | 0 B / 312 B | 888 B / 144 B | 0 B / 416 B | 324.00 KiB / 1.13 MiB | 324.00 KiB / 1.13 MiB |
| real.acos | streaming | 1 | 1.22 KiB / 160 B | 3.44 KiB / 496 B | 0 B / 208 B | 888 B / 144 B | 0 B / 312 B | 340.00 KiB / 1012.00 KiB | 340.00 KiB / 1012.00 KiB |
| real.atan | streaming | 1 | 1.22 KiB / 160 B | 944 B / 496 B | 0 B / 208 B | 336 B / 144 B | 0 B / 312 B | 324.00 KiB / 1020.00 KiB | 324.00 KiB / 1020.00 KiB |
| real.square | streaming | 1 | 1.22 KiB / 160 B | 224 B / 104 B | 0 B / 104 B | 112 B / 0 B | 0 B / 208 B | 296.00 KiB / 812.00 KiB | 296.00 KiB / 812.00 KiB |
| real.exp2 | streaming | 1 | 1.22 KiB / 160 B | 208 B / 9.73 KiB | 0 B / 208 B | 104 B / 1.25 KiB | 0 B / 312 B | 324.00 KiB / 1.16 MiB | 324.00 KiB / 1.16 MiB |
| real.exp10 | streaming | 1 | 1.22 KiB / 160 B | 208 B / 12.07 KiB | 0 B / 368 B | 104 B / 3.42 KiB | 0 B / 472 B | 296.00 KiB / 1.23 MiB | 296.00 KiB / 1.23 MiB |
| real.cot | streaming | 1 | 1.22 KiB / 160 B | 1.69 KiB / 2.05 KiB | 80 B / 664 B | 584 B / 288 B | 0 B / 768 B | 324.00 KiB / 900.00 KiB | 324.00 KiB / 900.00 KiB |
| real.add | streaming | 1 | 1.22 KiB / 312 B | 224 B / 384 B | 0 B / 280 B | 112 B / 0 B | 0 B / 488 B | 324.00 KiB / 844.00 KiB | 324.00 KiB / 844.00 KiB |
| real.subtract | streaming | 1 | 1.22 KiB / 312 B | 224 B / 280 B | 0 B / 280 B | 112 B / 0 B | 0 B / 488 B | 324.00 KiB / 912.00 KiB | 324.00 KiB / 912.00 KiB |
| real.multiply | streaming | 1 | 1.22 KiB / 312 B | 224 B / 104 B | 0 B / 104 B | 112 B / 0 B | 0 B / 312 B | 340.00 KiB / 788.00 KiB | 340.00 KiB / 788.00 KiB |
| real.divide | streaming | 1 | 1.22 KiB / 312 B | 224 B / 208 B | 0 B / 0 B | 112 B / 104 B | 0 B / 208 B | 296.00 KiB / 648.00 KiB | 296.00 KiB / 648.00 KiB |
| real.powi | streaming | 1 | 1.22 KiB / 312 B | 544 B / 208 B | 0 B / 0 B | 216 B / 104 B | 0 B / 208 B | 324.00 KiB / 836.00 KiB | 324.00 KiB / 836.00 KiB |
| real.pi | streaming | 1 | 16 B / 0 B | 192 B / 320 B | 0 B / 176 B | 96 B / 72 B | 0 B / 176 B | 296.00 KiB / 768.00 KiB | 296.00 KiB / 768.00 KiB |
| real.e | streaming | 1 | 16 B / 0 B | 192 B / 320 B | 0 B / 176 B | 96 B / 72 B | 0 B / 176 B | 368.00 KiB / 676.00 KiB | 368.00 KiB / 676.00 KiB |
| real.sign | streaming | 1 | 1.22 KiB / 152 B | 1.27 KiB / 784 B | 80 B / 208 B | 368 B / 72 B | 0 B / 312 B | 404.00 KiB / 1.13 MiB | 404.00 KiB / 1.13 MiB |
| real.floor | streaming | 1 | 1.22 KiB / 152 B | 2.05 KiB / 208 B | 0 B / 0 B | 824 B / 104 B | 0 B / 104 B | 324.00 KiB / 836.00 KiB | 324.00 KiB / 836.00 KiB |
| real.ceil | streaming | 1 | 1.22 KiB / 152 B | 2.77 KiB / 416 B | 0 B / 0 B | 1.13 KiB / 104 B | 0 B / 104 B | 324.00 KiB / 676.00 KiB | 328.00 KiB / 676.00 KiB |
| complex.add | streaming | 1 | 304 B / 616 B | 352 B / 768 B | 0 B / 560 B | 144 B / 0 B | 0 B / 976 B | 304.00 KiB / 1016.00 KiB | 304.00 KiB / 1016.00 KiB |
| complex.subtract | streaming | 1 | 304 B / 616 B | 352 B / 768 B | 0 B / 560 B | 144 B / 0 B | 0 B / 976 B | 296.00 KiB / 1.07 MiB | 296.00 KiB / 1.07 MiB |
| complex.multiply | streaming | 1 | 304 B / 616 B | 960 B / 1.70 KiB | 0 B / 1.40 KiB | 288 B / 104 B | 0 B / 1.80 KiB | 296.00 KiB / 1012.00 KiB | 296.00 KiB / 1012.00 KiB |
| complex.divide | streaming | 1 | 304 B / 616 B | 1.72 KiB / 416 B | 0 B / 0 B | 432 B / 208 B | 0 B / 416 B | 324.00 KiB / 700.00 KiB | 324.00 KiB / 700.00 KiB |
| complex.norm_squared | streaming | 1 | 304 B / 304 B | 480 B / 312 B | 0 B / 104 B | 216 B / 104 B | 0 B / 312 B | 312.00 KiB / 824.00 KiB | 312.00 KiB / 824.00 KiB |
| vector2.add | streaming | 1 | 352 B / 635 B | 592 B / 456 B | 0 B / 456 B | 168 B / 0 B | 0 B / 872 B | 296.00 KiB / 932.00 KiB | 296.00 KiB / 932.00 KiB |
| vector2.subtract | streaming | 1 | 352 B / 640 B | 592 B / 456 B | 0 B / 456 B | 168 B / 0 B | 0 B / 872 B | 308.00 KiB / 932.00 KiB | 308.00 KiB / 932.00 KiB |
| vector2.dot | streaming | 1 | 352 B / 635 B | 480 B / 208 B | 0 B / 208 B | 144 B / 0 B | 0 B / 624 B | 212.00 KiB / 896.00 KiB | 212.00 KiB / 896.00 KiB |
| vector2.norm | streaming | 1 | 872 B / 636 B | 1.80 KiB / 800 B | 0 B / 592 B | 888 B / 0 B | 0 B / 1008 B | 276.00 KiB / 1.04 MiB | 276.00 KiB / 1.04 MiB |
| vector2.wedge | streaming | 1 | 352 B / 637 B | 480 B / 208 B | 0 B / 208 B | 216 B / 0 B | 0 B / 624 B | 296.00 KiB / 868.00 KiB | 296.00 KiB / 868.00 KiB |
| vector3.add | streaming | 1 | 480 B / 939 B | 880 B / 736 B | 0 B / 736 B | 248 B / 0 B | 0 B / 1.33 KiB | 296.00 KiB / 888.00 KiB | 296.00 KiB / 888.00 KiB |
| vector3.subtract | streaming | 1 | 480 B / 944 B | 880 B / 736 B | 0 B / 736 B | 248 B / 0 B | 0 B / 1.33 KiB | 296.00 KiB / 944.00 KiB | 296.00 KiB / 944.00 KiB |
| vector3.dot | streaming | 1 | 480 B / 939 B | 640 B / 208 B | 0 B / 0 B | 144 B / 104 B | 0 B / 624 B | 364.00 KiB / 856.00 KiB | 364.00 KiB / 856.00 KiB |
| vector3.cross | streaming | 1 | 480 B / 941 B | 1.75 KiB / 520 B | 0 B / 312 B | 824 B / 104 B | 0 B / 936 B | 300.00 KiB / 804.00 KiB | 340.00 KiB / 804.00 KiB |
| vector3.norm | streaming | 1 | 1.25 KiB / 940 B | 2.23 KiB / 1.77 KiB | 0 B / 1.43 KiB | 1.09 KiB / 72 B | 0 B / 2.04 KiB | 236.00 KiB / 1.02 MiB | 236.00 KiB / 1.02 MiB |
| vector4.add | streaming | 1 | 608 B / 1.21 KiB | 1.14 KiB / 912 B | 0 B / 912 B | 328 B / 0 B | 0 B / 1.70 KiB | 308.00 KiB / 980.00 KiB | 308.00 KiB / 980.00 KiB |
| vector4.subtract | streaming | 1 | 608 B / 1.22 KiB | 1.14 KiB / 1016 B | 0 B / 1016 B | 328 B / 0 B | 0 B / 1.80 KiB | 220.00 KiB / 932.00 KiB | 220.00 KiB / 932.00 KiB |
| vector4.dot | streaming | 1 | 608 B / 1.21 KiB | 800 B / 208 B | 0 B / 0 B | 144 B / 104 B | 0 B / 832 B | 308.00 KiB / 868.00 KiB | 308.00 KiB / 868.00 KiB |
| vector4.norm | streaming | 1 | 1.63 KiB / 1.21 KiB | 2.67 KiB / 1.74 KiB | 0 B / 976 B | 1.30 KiB / 352 B | 0 B / 1.77 KiB | 216.00 KiB / 1.07 MiB | 216.00 KiB / 1.07 MiB |
| matrix3.add | streaming | 1 | 1.21 KiB / 1.99 KiB | 2.53 KiB / 1.13 KiB | 0 B / 528 B | 728 B / 312 B | 0 B / 1.63 KiB | 312.00 KiB / 1.02 MiB | 312.00 KiB / 1.02 MiB |
| matrix3.subtract | streaming | 1 | 1.21 KiB / 1.99 KiB | 2.53 KiB / 1.60 KiB | 0 B / 1016 B | 728 B / 312 B | 0 B / 2.11 KiB | 312.00 KiB / 948.00 KiB | 376.00 KiB / 952.00 KiB |
| matrix3.multiply | streaming | 1 | 1.21 KiB / 1.99 KiB | 6.30 KiB / 832 B | 0 B / 832 B | 848 B / 0 B | 0 B / 1.93 KiB | 300.00 KiB / 1.18 MiB | 300.00 KiB / 1.18 MiB |
| matrix3.transpose | streaming | 1 | 1.21 KiB / 1.99 KiB | 1.00 KiB / 0 B | 0 B / 0 B | 512 B / 0 B | 0 B / 1.12 KiB | 172.00 KiB / 948.00 KiB | 172.00 KiB / 948.00 KiB |
| matrix3.determinant | streaming | 1 | 1.21 KiB / 2.00 KiB | 4.20 KiB / 768 B | 0 B / 768 B | 724 B / 0 B | 0 B / 1.87 KiB | 228.00 KiB / 1.10 MiB | 228.00 KiB / 1.10 MiB |
| matrix3.inverse | streaming | 1 | 1.21 KiB / 1.99 KiB | 31.50 KiB / 1.97 KiB | 0 B / 1.97 KiB | 3.11 KiB / 0 B | 0 B / 3.09 KiB | 320.00 KiB / 1.04 MiB | 320.00 KiB / 1.04 MiB |
| matrix4.add | streaming | 1 | 2.05 KiB / 2.44 KiB | 4.47 KiB / 1.53 KiB | 0 B / 528 B | 1.26 KiB / 520 B | 0 B / 1.43 KiB | 316.00 KiB / 944.00 KiB | 316.00 KiB / 948.00 KiB |
| matrix4.subtract | streaming | 1 | 2.05 KiB / 2.45 KiB | 4.47 KiB / 3.07 KiB | 0 B / 2.05 KiB | 1.26 KiB / 520 B | 0 B / 2.97 KiB | 292.00 KiB / 952.00 KiB | 292.00 KiB / 984.00 KiB |
| matrix4.multiply | streaming | 1 | 2.05 KiB / 2.45 KiB | 12.77 KiB / 936 B | 0 B / 936 B | 1.38 KiB / 0 B | 0 B / 1.83 KiB | 304.00 KiB / 1.07 MiB | 304.00 KiB / 1.08 MiB |
| matrix4.transpose | streaming | 1 | 2.05 KiB / 2.45 KiB | 1.77 KiB / 0 B | 0 B / 0 B | 904 B / 0 B | 0 B / 936 B | 176.00 KiB / 1.03 MiB | 176.00 KiB / 1.03 MiB |
| matrix4.determinant | streaming | 1 | 2.05 KiB / 2.45 KiB | 10.34 KiB / 3.04 KiB | 0 B / 2.84 KiB | 1.12 KiB / 104 B | 0 B / 3.75 KiB | 376.00 KiB / 1.12 MiB | 376.00 KiB / 1.12 MiB |
| matrix4.inverse | streaming | 1 | 2.05 KiB / 2.44 KiB | 74.66 KiB / 3.25 KiB | 0 B / 0 B | 5.41 KiB / 1.63 KiB | 0 B / 936 B | 344.00 KiB / 992.00 KiB | 344.00 KiB / 996.00 KiB |
| geometry2.orientation | streaming | 1 | 2.41 KiB / 808 B | 1.31 KiB / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 520 B | 232.00 KiB / 868.00 KiB | 232.00 KiB / 868.00 KiB |
| geometry2.area | streaming | 1 | 2.41 KiB / 808 B | 1.53 KiB / 592 B | 0 B / 488 B | 784 B / 0 B | 0 B / 1008 B | 256.00 KiB / 1.08 MiB | 256.00 KiB / 1.08 MiB |
| geometry2.between | streaming | 1 | 2.41 KiB / 808 B | 1.31 KiB / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 520 B | 248.00 KiB / 964.00 KiB | 248.00 KiB / 964.00 KiB |
| geometry2.line_relation_0 | streaming | 1 | 3.83 KiB / 1.09 KiB | 1.31 KiB / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 728 B | 248.00 KiB / 828.00 KiB | 252.00 KiB / 828.00 KiB |
| geometry2.line_relation_1 | streaming | 1 | 3.83 KiB / 1.09 KiB | 1.31 KiB / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 728 B | 248.00 KiB / 864.00 KiB | 252.00 KiB / 864.00 KiB |
| geometry2.line_relation_2 | streaming | 1 | 3.83 KiB / 1.09 KiB | 1.86 KiB / 768 B | 0 B / 560 B | 920 B / 104 B | 0 B / 1.26 KiB | 296.00 KiB / 1.35 MiB | 300.00 KiB / 1.35 MiB |
| geometry2.line_relation_3 | streaming | 1 | 3.83 KiB / 1.09 KiB | 3.17 KiB / 768 B | 0 B / 560 B | 920 B / 104 B | 0 B / 1.26 KiB | 300.00 KiB / 1.07 MiB | 304.00 KiB / 1.07 MiB |
| geometry2.line_relation_4 | streaming | 1 | 3.83 KiB / 1.09 KiB | 1.31 KiB / 768 B | 0 B / 560 B | 672 B / 104 B | 0 B / 1.26 KiB | 292.00 KiB / 1.29 MiB | 296.00 KiB / 1.29 MiB |
| geometry2.line_relation_5 | streaming | 1 | 3.83 KiB / 1.09 KiB | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 728 B | 312.00 KiB / 808.00 KiB | 312.00 KiB / 808.00 KiB |
| geometry2.line_relation_6 | streaming | 1 | 3.83 KiB / 1.09 KiB | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 728 B | 272.00 KiB / 780.00 KiB | 272.00 KiB / 780.00 KiB |
| geometry2.line_intersection | streaming | 1 | 3.83 KiB / 1.09 KiB | 7.13 KiB / 416 B | 0 B / 0 B | 1.91 KiB / 208 B | 0 B / 728 B | 288.00 KiB / 844.00 KiB | 288.00 KiB / 844.00 KiB |
| geometry2.segment_relation_0 | streaming | 1 | 3.33 KiB / 1.09 KiB | 1.80 KiB / 0 B | 0 B / 0 B | 920 B / 0 B | 0 B / 728 B | 312.00 KiB / 1.08 MiB | 312.00 KiB / 1.08 MiB |
| geometry2.segment_relation_1 | streaming | 1 | 3.33 KiB / 1.09 KiB | 14.78 KiB / 0 B | 448 B / 0 B | 2.71 KiB / 0 B | 0 B / 728 B | 436.00 KiB / 1.04 MiB | 440.00 KiB / 1.04 MiB |
| geometry2.segment_relation_2 | streaming | 1 | 3.33 KiB / 1.09 KiB | 0 B / 0 B | 0 B / 0 B | 0 B / 0 B | 0 B / 728 B | 300.00 KiB / 1.11 MiB | 300.00 KiB / 1.11 MiB |
| geometry2.segment_relation_3 | streaming | 1 | 3.33 KiB / 1.09 KiB | 2.83 KiB / 768 B | 0 B / 560 B | 1.38 KiB / 104 B | 0 B / 1.26 KiB | 216.00 KiB / 1.10 MiB | 220.00 KiB / 1.10 MiB |
| geometry2.incircle | streaming | 1 | 3.30 KiB / 1008 B | 7.78 KiB / 0 B | 0 B / 0 B | 3.50 KiB / 0 B | 0 B / 624 B | 268.00 KiB / 1.10 MiB | 268.00 KiB / 1.10 MiB |
| geometry2.circle_line | streaming | 1 | 3.57 KiB / 856 B | 4.70 KiB / 2.22 KiB | 0 B / 2.02 KiB | 1.96 KiB / 208 B | 0 B / 2.52 KiB | 244.00 KiB / 1.23 MiB | 244.00 KiB / 1.24 MiB |
| geometry2.circle_segment | streaming | 1 | 2.87 KiB / 856 B | 10.53 KiB / 3.85 KiB | 0 B / 3.14 KiB | 4.50 KiB / 416 B | 0 B / 3.65 KiB | 296.00 KiB / 1.10 MiB | 296.00 KiB / 1.10 MiB |
| geometry2.circle_point_distance | streaming | 1 | 3.31 KiB / 656 B | 1.52 KiB / 1.82 KiB | 0 B / 104 B | 776 B / 600 B | 0 B / 520 B | 308.00 KiB / 1.22 MiB | 308.00 KiB / 1.22 MiB |
| geometry2.circle_circle_distance | streaming | 1 | 3.31 KiB / 656 B | 2.45 KiB / 3.35 KiB | 0 B / 104 B | 1.20 KiB / 968 B | 0 B / 520 B | 252.00 KiB / 1.18 MiB | 252.00 KiB / 1.18 MiB |
| geometry2.point_distance | streaming | 1 | 1.66 KiB / 504 B | 1.30 KiB / 1.57 KiB | 0 B / 1.37 KiB | 664 B / 0 B | 0 B / 1.67 KiB | 308.00 KiB / 1.04 MiB | 312.00 KiB / 1.04 MiB |
| geometry2.line_point_distance | streaming | 1 | 3.83 KiB / 808 B | 3.97 KiB / 2.19 KiB | 0 B / 0 B | 1.95 KiB / 1016 B | 0 B / 520 B | 296.00 KiB / 1.18 MiB | 296.00 KiB / 1.18 MiB |
| geometry2.segment_point_distance | streaming | 1 | 3.33 KiB / 808 B | 10.52 KiB / 2.59 KiB | 0 B / 0 B | 4.60 KiB / 1.09 KiB | 0 B / 520 B | 244.00 KiB / 1.20 MiB | 244.00 KiB / 1.20 MiB |
| geometry3.orientation | streaming | 1 | 4.77 KiB / 1.58 KiB | 6.47 KiB / 0 B | 0 B / 0 B | 2.84 KiB / 0 B | 0 B / 1.02 KiB | 240.00 KiB / 728.00 KiB | 240.00 KiB / 728.00 KiB |
| geometry3.volume | streaming | 1 | 4.77 KiB / 1.58 KiB | 13.36 KiB / 2.86 KiB | 0 B / 1.84 KiB | 5.46 KiB / 520 B | 0 B / 2.86 KiB | 272.00 KiB / 1.11 MiB | 272.00 KiB / 1.11 MiB |
| geometry3.line_relation_0 | streaming | 1 | 5.66 KiB / 1.59 KiB | 3.47 KiB / 1.33 KiB | 0 B / 1.23 KiB | 1.70 KiB / 0 B | 0 B / 2.24 KiB | 220.00 KiB / 1.04 MiB | 224.00 KiB / 1.04 MiB |
| geometry3.line_relation_1 | streaming | 1 | 5.66 KiB / 1.59 KiB | 2.81 KiB / 2.18 KiB | 0 B / 1.30 KiB | 1.38 KiB / 592 B | 0 B / 2.31 KiB | 228.00 KiB / 1.05 MiB | 232.00 KiB / 1.05 MiB |
| geometry3.line_relation_2 | streaming | 1 | 5.66 KiB / 1.59 KiB | 5.56 KiB / 0 B | 0 B / 0 B | 2.63 KiB / 0 B | 0 B / 1.02 KiB | 300.00 KiB / 848.00 KiB | 300.00 KiB / 848.00 KiB |
| geometry3.line_relation_3 | streaming | 1 | 5.66 KiB / 1.59 KiB | 6.47 KiB / 0 B | 0 B / 0 B | 2.84 KiB / 0 B | 0 B / 1.02 KiB | 308.00 KiB / 868.00 KiB | 308.00 KiB / 868.00 KiB |
| geometry3.line_relation_4 | streaming | 1 | 5.66 KiB / 1.59 KiB | 3.47 KiB / 2.18 KiB | 0 B / 1.30 KiB | 1.70 KiB / 592 B | 0 B / 2.31 KiB | 240.00 KiB / 1.05 MiB | 244.00 KiB / 1.05 MiB |
| geometry3.segment_relation_0 | streaming | 1 | 4.92 KiB / 1.59 KiB | 4.19 KiB / 1.33 KiB | 0 B / 1.23 KiB | 2.06 KiB / 0 B | 0 B / 2.24 KiB | 240.00 KiB / 1004.00 KiB | 244.00 KiB / 1004.00 KiB |
| geometry3.segment_relation_1 | streaming | 1 | 4.92 KiB / 1.59 KiB | 7.91 KiB / 1.98 KiB | 0 B / 1.57 KiB | 3.56 KiB / 208 B | 0 B / 2.59 KiB | 240.00 KiB / 1.23 MiB | 244.00 KiB / 1.30 MiB |
| geometry3.segment_relation_2 | streaming | 1 | 4.92 KiB / 1.59 KiB | 0 B / 1.98 KiB | 0 B / 1.57 KiB | 0 B / 208 B | 0 B / 2.59 KiB | 228.00 KiB / 1.23 MiB | 228.00 KiB / 1.23 MiB |
| geometry3.segment_relation_3 | streaming | 1 | 4.92 KiB / 1.59 KiB | 6.47 KiB / 0 B | 0 B / 0 B | 2.84 KiB / 0 B | 0 B / 1.02 KiB | 244.00 KiB / 908.00 KiB | 244.00 KiB / 908.00 KiB |
| geometry3.plane_relation_0 | streaming | 1 | 13.27 KiB / 1.53 KiB | 2.03 KiB / 0 B | 0 B / 0 B | 1008 B / 0 B | 0 B / 936 B | 276.00 KiB / 900.00 KiB | 280.00 KiB / 900.00 KiB |
| geometry3.plane_relation_1 | streaming | 1 | 13.27 KiB / 1.53 KiB | 1.31 KiB / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 936 B | 332.00 KiB / 924.00 KiB | 332.00 KiB / 924.00 KiB |
| geometry3.plane_relation_2 | streaming | 1 | 13.27 KiB / 1.53 KiB | 2.03 KiB / 0 B | 0 B / 0 B | 1008 B / 0 B | 0 B / 936 B | 252.00 KiB / 1.20 MiB | 256.00 KiB / 1.20 MiB |
| geometry3.plane_relation_3 | streaming | 1 | 13.27 KiB / 1.53 KiB | 4.78 KiB / 0 B | 0 B / 0 B | 1.34 KiB / 0 B | 0 B / 936 B | 252.00 KiB / 1.14 MiB | 256.00 KiB / 1.14 MiB |
| geometry3.plane_relation_4 | streaming | 1 | 13.27 KiB / 1.53 KiB | 2.03 KiB / 0 B | 0 B / 0 B | 1008 B / 0 B | 0 B / 936 B | 248.00 KiB / 1.10 MiB | 252.00 KiB / 1.10 MiB |
| geometry3.plane_relation_5 | streaming | 1 | 13.27 KiB / 1.53 KiB | 5.59 KiB / 0 B | 0 B / 0 B | 1.75 KiB / 0 B | 0 B / 936 B | 252.00 KiB / 1.23 MiB | 256.00 KiB / 1.23 MiB |
| geometry3.plane_relation_6 | streaming | 1 | 13.27 KiB / 1.53 KiB | 2.75 KiB / 0 B | 0 B / 0 B | 1.34 KiB / 0 B | 0 B / 936 B | 324.00 KiB / 1.05 MiB | 328.00 KiB / 1.05 MiB |
| geometry3.plane_relation_7 | streaming | 1 | 13.27 KiB / 1.53 KiB | 11.09 KiB / 0 B | 6.83 KiB / 0 B | 1.39 KiB / 0 B | 0 B / 936 B | 468.00 KiB / 1.03 MiB | 468.00 KiB / 1.03 MiB |
| geometry3.point_distance | streaming | 1 | 2.45 KiB / 704 B | 1.95 KiB / 2.77 KiB | 0 B / 2.56 KiB | 1000 B / 0 B | 0 B / 2.97 KiB | 248.00 KiB / 1012.00 KiB | 248.00 KiB / 1016.00 KiB |
| geometry3.line_point_distance | streaming | 1 | 5.66 KiB / 1.13 KiB | 7.44 KiB / 4.71 KiB | 0 B / 1.16 KiB | 3.66 KiB / 1.71 KiB | 0 B / 1.87 KiB | 232.00 KiB / 1.21 MiB | 232.00 KiB / 1.21 MiB |
| geometry3.segment_point_distance | streaming | 1 | 4.92 KiB / 1.13 KiB | 16.03 KiB / 5.12 KiB | 0 B / 1.16 KiB | 7.30 KiB / 1.81 KiB | 0 B / 1.87 KiB | 260.00 KiB / 1.16 MiB | 260.00 KiB / 1.16 MiB |
| geometry3.plane_point_distance | streaming | 1 | 13.27 KiB / 856 B | 3.77 KiB / 592 B | 0 B / 384 B | 1.85 KiB / 0 B | 0 B / 904 B | 296.00 KiB / 1.41 MiB | 300.00 KiB / 1.41 MiB |
| geometry3.triangle_relation_0 | streaming | 1 | 7.85 KiB / 1.56 KiB | 6.47 KiB / 104 B | 0 B / 104 B | 2.84 KiB / 0 B | 0 B / 832 B | 300.00 KiB / 1.06 MiB | 300.00 KiB / 1.06 MiB |
| geometry3.triangle_relation_1 | streaming | 1 | 7.85 KiB / 1.56 KiB | 26.83 KiB / 104 B | 448 B / 104 B | 4.48 KiB / 0 B | 0 B / 832 B | 372.00 KiB / 1004.00 KiB | 372.00 KiB / 1004.00 KiB |
| geometry3.triangle_relation_2 | streaming | 1 | 7.85 KiB / 1.56 KiB | 16.00 KiB / 104 B | 0 B / 104 B | 3.42 KiB / 0 B | 0 B / 832 B | 292.00 KiB / 1.03 MiB | 292.00 KiB / 1.03 MiB |
| geometry3.triangle_relation_3 | streaming | 1 | 7.85 KiB / 1.56 KiB | 26.83 KiB / 104 B | 448 B / 104 B | 4.48 KiB / 0 B | 0 B / 832 B | 408.00 KiB / 1.11 MiB | 408.00 KiB / 1.11 MiB |
| geometry3.triangle_relation_4 | streaming | 1 | 7.85 KiB / 1.56 KiB | 37.30 KiB / 4.95 KiB | 0 B / 4.54 KiB | 4.51 KiB / 104 B | 0 B / 5.25 KiB | 248.00 KiB / 1.42 MiB | 248.00 KiB / 1.42 MiB |
| geometry3.triangle_relation_5 | streaming | 1 | 7.85 KiB / 1.56 KiB | 25.88 KiB / 4.95 KiB | 0 B / 4.54 KiB | 2.84 KiB / 104 B | 0 B / 5.25 KiB | 288.00 KiB / 1.41 MiB | 288.00 KiB / 1.41 MiB |
| geometry3.triangle_relation_6 | streaming | 1 | 7.85 KiB / 1.56 KiB | 45.66 KiB / 30.79 KiB | 13.91 KiB / 22.77 KiB | 5.27 KiB / 104 B | 19.65 KiB / 23.48 KiB | 272.00 KiB / 1.50 MiB | 272.00 KiB / 1.50 MiB |
| polynomial.evaluate | streaming | 1 | 416 B / 712 B | 96 B / 768 B | 0 B / 768 B | 40 B / 0 B | 0 B / 1.26 KiB | 308.00 KiB / 740.00 KiB | 308.00 KiB / 740.00 KiB |
| polynomial.binary_0 | streaming | 1 | 416 B / 864 B | 976 B / 1.60 KiB | 0 B / 1.50 KiB | 296 B / 0 B | 0 B / 2.01 KiB | 228.00 KiB / 804.00 KiB | 228.00 KiB / 804.00 KiB |
| polynomial.binary_1 | streaming | 1 | 416 B / 864 B | 976 B / 1.60 KiB | 0 B / 1.50 KiB | 296 B / 0 B | 0 B / 2.01 KiB | 144.00 KiB / 788.00 KiB | 144.00 KiB / 788.00 KiB |
| polynomial.binary_2 | streaming | 1 | 416 B / 864 B | 2.77 KiB / 1.33 KiB | 0 B / 1.33 KiB | 496 B / 0 B | 0 B / 1.84 KiB | 224.00 KiB / 804.00 KiB | 224.00 KiB / 804.00 KiB |
| polynomial.binary_3 | streaming | 1 | 416 B / 864 B | 5.95 KiB / 4.71 KiB | 368 B / 1016 B | 608 B / 1.20 KiB | 368 B / 1.50 KiB | 324.00 KiB / 1.22 MiB | 328.00 KiB / 1.23 MiB |
| polynomial.binary_4 | streaming | 1 | 416 B / 864 B | 8.50 KiB / 2.08 KiB | 0 B / 1.98 KiB | 824 B / 0 B | 0 B / 2.48 KiB | 232.00 KiB / 836.00 KiB | 236.00 KiB / 840.00 KiB |
| polynomial.derivative | streaming | 1 | 416 B / 824 B | 1.44 KiB / 488 B | 0 B / 488 B | 280 B / 0 B | 0 B / 1.19 KiB | 232.00 KiB / 828.00 KiB | 232.00 KiB / 828.00 KiB |
| polynomial.resultant | streaming | 1 | 416 B / 856 B | 6.66 KiB / 11.48 KiB | 368 B / 104 B | 640 B / 4.71 KiB | 368 B / 624 B | 328.00 KiB / 1.23 MiB | 332.00 KiB / 1.23 MiB |
| polynomial.discriminant | streaming | 1 | 416 B / 1.04 KiB | 14.14 KiB / 13.57 KiB | 128 B / 104 B | 1.00 KiB / 4.91 KiB | 128 B / 832 B | 328.00 KiB / 1.09 MiB | 328.00 KiB / 1.09 MiB |
| polynomial.gcd | streaming | 1 | 416 B / 856 B | 6.94 KiB / 4.61 KiB | 368 B / 912 B | 736 B / 1.20 KiB | 368 B / 1.40 KiB | 324.00 KiB / 1.21 MiB | 328.00 KiB / 1.22 MiB |
| polynomial.square_free | streaming | 1 | 416 B / 712 B | 20.13 KiB / 2.12 KiB | 304 B / 312 B | 1.27 KiB / 816 B | 304 B / 832 B | 328.00 KiB / 1.08 MiB | 380.00 KiB / 1.08 MiB |
| polynomial.root_count | streaming | 1 | 416 B / 2.36 KiB | 44.86 KiB / 27.73 KiB | 432 B / 11.42 KiB | 1.59 KiB / 2.88 KiB | 432 B / 12.03 KiB | 456.00 KiB / 1.38 MiB | 460.00 KiB / 1.38 MiB |
| polynomial.root_count_interval | streaming | 1 | 416 B / 2.36 KiB | 41.44 KiB / 27.73 KiB | 432 B / 11.42 KiB | 1.34 KiB / 2.88 KiB | 432 B / 12.03 KiB | 392.00 KiB / 1.46 MiB | 392.00 KiB / 1.47 MiB |
| polynomial.root_isolation | streaming | 1 | 416 B / 2.36 KiB | 138.08 KiB / 27.73 KiB | 432 B / 11.42 KiB | 1.59 KiB / 2.88 KiB | 432 B / 12.03 KiB | 436.00 KiB / 1.37 MiB | 440.00 KiB / 1.38 MiB |
| bivariate.evaluate | streaming | 1 | 608 B / 992 B | 192 B / 960 B | 0 B / 768 B | 80 B / 96 B | 0 B / 1.46 KiB | 232.00 KiB / 872.00 KiB | 232.00 KiB / 872.00 KiB |
| bivariate.resultant | streaming | 1 | 608 B / 1.23 KiB | 25.48 KiB / 5.37 KiB | 512 B / 840 B | 1.23 KiB / 984 B | 760 B / 1.53 KiB | 328.00 KiB / 1.18 MiB | 328.00 KiB / 1.18 MiB |
| triangulation.delaunay_complex | streaming | 1 | 4.78 KiB / 1.95 KiB | 297.30 KiB / 161.16 KiB | 0 B / 56.75 KiB | 4.00 KiB / 1.61 KiB | 0 B / 57.97 KiB | 316.00 KiB / 1.29 MiB | 380.00 KiB / 1.29 MiB |
| curve.line_length | streaming | 1 | 1.66 KiB / 1.97 KiB | 1.30 KiB / 912 B | 0 B / 0 B | 664 B / 352 B | 0 B / 1.70 KiB | 308.00 KiB / 1.20 MiB | 312.00 KiB / 1.20 MiB |
| curve.line_side | streaming | 1 | 3.83 KiB / 2.19 KiB | 1.31 KiB / 0 B | 0 B / 0 B | 672 B / 0 B | 0 B / 1.80 KiB | 236.00 KiB / 1.29 MiB | 236.00 KiB / 1.29 MiB |
| curve.line_contains_point | streaming | 1 | 3.33 KiB / 2.19 KiB | 1.80 KiB / 0 B | 0 B / 0 B | 920 B / 0 B | 0 B / 1.80 KiB | 236.00 KiB / 1.22 MiB | 236.00 KiB / 1.22 MiB |
| curve.line_intersection_topology | streaming | 1 | 3.33 KiB / 3.02 KiB | 14.78 KiB / 1.26 KiB | 448 B / 664 B | 2.71 KiB / 312 B | 0 B / 3.13 KiB | 424.00 KiB / 1.14 MiB | 464.00 KiB / 1.14 MiB |
| curve.line_intersection_witness | streaming | 1 | 3.83 KiB / 3.02 KiB | 7.13 KiB / 1.26 KiB | 0 B / 664 B | 1.91 KiB / 312 B | 0 B / 3.13 KiB | 244.00 KiB / 1.25 MiB | 244.00 KiB / 1.25 MiB |
| curve.segment_dispatch | streaming | 1 | 3.33 KiB / 3.02 KiB | 14.78 KiB / 1.26 KiB | 448 B / 664 B | 2.71 KiB / 312 B | 0 B / 3.13 KiB | 436.00 KiB / 1.16 MiB | 440.00 KiB / 1.16 MiB |
| curve.supporting_line_circle | streaming | 1 | 3.57 KiB / 2.88 KiB | 4.70 KiB / 1.70 KiB | 0 B / 488 B | 1.96 KiB / 624 B | 0 B / 2.41 KiB | 304.00 KiB / 1.40 MiB | 304.00 KiB / 1.40 MiB |
| curve.circle_point_distance | streaming | 1 | 3.31 KiB / 2.18 KiB | 1.52 KiB / 1.72 KiB | 0 B / 0 B | 776 B / 600 B | 0 B / 1.39 KiB | 240.00 KiB / 1.23 MiB | 240.00 KiB / 1.23 MiB |
| curve.circle_circle_relation | streaming | 1 | 3.31 KiB / 3.01 KiB | 2.45 KiB / 1.30 KiB | 0 B / 1008 B | 1.20 KiB / 216 B | 0 B / 2.65 KiB | 204.00 KiB / 1.16 MiB | 204.00 KiB / 1.16 MiB |
| mesh.plane_point_classification | streaming | 1 | 13.27 KiB / 1.55 KiB | 7.43 KiB / 704 B | 3.40 KiB / 0 B | 2.02 KiB / 352 B | 0 B / 1.12 KiB | 324.00 KiB / 988.00 KiB | 328.00 KiB / 988.00 KiB |
| mesh.triangle_contains_point | streaming | 1 | 7.85 KiB / 4.84 KiB | 26.83 KiB / 704 B | 448 B / 0 B | 4.48 KiB / 352 B | 0 B / 2.21 KiB | 384.00 KiB / 1.00 MiB | 384.00 KiB / 1.00 MiB |
| mesh.triangle_contains_point_strictly | streaming | 1 | 7.85 KiB / 4.84 KiB | 26.83 KiB / 704 B | 448 B / 0 B | 4.48 KiB / 352 B | 0 B / 2.21 KiB | 372.00 KiB / 1.13 MiB | 372.00 KiB / 1.13 MiB |
| mesh.triangle_boundary_point | streaming | 1 | 7.85 KiB / 4.74 KiB | 26.16 KiB / 1.38 KiB | 448 B / 0 B | 5.91 KiB / 352 B | 0 B / 2.11 KiB | 496.00 KiB / 1.20 MiB | 496.00 KiB / 1.20 MiB |
| mesh.triangle_triangle_intersection | streaming | 1 | 7.85 KiB / 8.70 KiB | 69.48 KiB / 5.36 KiB | 32.52 KiB / 0 B | 5.75 KiB / 2.38 KiB | 37.82 KiB / 3.61 KiB | 480.00 KiB / 1.24 MiB | 480.00 KiB / 1.24 MiB |
| path.line_length | streaming | 1 | 1.66 KiB / 1.95 KiB | 1.30 KiB / 800 B | 0 B / 176 B | 664 B / 208 B | 0 B / 1.63 KiB | 244.00 KiB / 1.15 MiB | 308.00 KiB / 1.15 MiB |
| path.line_axis_classification | streaming | 1 | 3.83 KiB / 1.58 KiB | 976 B / 0 B | 0 B / 0 B | 416 B / 0 B | 0 B / 1.09 KiB | 432.00 KiB / 1012.00 KiB | 432.00 KiB / 1012.00 KiB |
| path.line_endpoint_equality | streaming | 1 | 3.33 KiB / 2.72 KiB | 4.84 KiB / 0 B | 448 B / 0 B | 464 B / 0 B | 0 B / 1.73 KiB | 424.00 KiB / 1.09 MiB | 428.00 KiB / 1.09 MiB |
| path.line_parameter_order | streaming | 1 | 2.41 KiB / 1.59 KiB | 6.86 KiB / 0 B | 0 B / 0 B | 1.79 KiB / 0 B | 0 B / 936 B | 248.00 KiB / 1.17 MiB | 252.00 KiB / 1.17 MiB |
| path.circle_point_membership | streaming | 1 | 3.31 KiB / 2.01 KiB | 1.52 KiB / 0 B | 0 B / 0 B | 776 B / 0 B | 0 B / 1.29 KiB | 296.00 KiB / 868.00 KiB | 296.00 KiB / 868.00 KiB |
| path.circle_segment_intersection | streaming | 1 | 2.87 KiB / 2.68 KiB | 13.70 KiB / 5.07 KiB | 0 B / 3.16 KiB | 6.09 KiB / 576 B | 0 B / 4.73 KiB | 312.00 KiB / 1.17 MiB | 312.00 KiB / 1.17 MiB |
| path.circle_circle_relation | streaming | 1 | 3.31 KiB / 2.71 KiB | 2.45 KiB / 416 B | 0 B / 208 B | 1.20 KiB / 104 B | 0 B / 1.66 KiB | 240.00 KiB / 968.00 KiB | 240.00 KiB / 972.00 KiB |
| line-point | streaming | 8 | 21.09 KiB / 5.09 KiB | 25.48 KiB / 0 B | 448 B / 0 B | 2.47 KiB / 0 B | 0 B / 1.02 KiB | 476.00 KiB / 852.00 KiB | 476.00 KiB / 856.00 KiB |
| line-point | streaming | 32 | 84.28 KiB / 19.16 KiB | 95.73 KiB / 0 B | 448 B / 0 B | 2.47 KiB / 0 B | 0 B / 1.02 KiB | 524.00 KiB / 888.00 KiB | 528.00 KiB / 892.00 KiB |
| line-point | streaming | 128 | 337.03 KiB / 75.41 KiB | 376.73 KiB / 0 B | 448 B / 0 B | 2.47 KiB / 0 B | 0 B / 1.02 KiB | 888.00 KiB / 960.00 KiB | 896.00 KiB / 964.00 KiB |
| line-point | streaming | 512 | 1.32 MiB / 300.51 KiB | 1.50 MiB / 0 B | 648 B / 0 B | 2.50 KiB / 0 B | 0 B / 1.73 KiB | 2.82 MiB / 1.13 MiB | 2.82 MiB / 1012.00 KiB |
| line-point | streaming | 2048 | 5.27 MiB / 1.63 MiB | 6.08 MiB / 0 B | 648 B / 0 B | 2.50 KiB / 0 B | 0 B / 1.93 KiB | 9.99 MiB / 2.95 MiB | 9.96 MiB / 2.79 MiB |
| line-point | streaming | 8192 | 21.06 MiB / 6.96 MiB | 24.42 MiB / 0 B | 648 B / 0 B | 2.50 KiB / 0 B | 0 B / 1.93 KiB | 38.66 MiB / 10.39 MiB | 38.56 MiB / 10.15 MiB |
| line-point | materialized | 8 | 21.09 KiB / 5.09 KiB | 25.55 KiB / 48 B | 448 B / 0 B | 2.53 KiB / 48 B | 0 B / 1.02 KiB | 476.00 KiB / 872.00 KiB | 476.00 KiB / 872.00 KiB |
| line-point | materialized | 32 | 84.28 KiB / 19.16 KiB | 95.98 KiB / 192 B | 448 B / 0 B | 2.72 KiB / 192 B | 0 B / 1.02 KiB | 576.00 KiB / 1004.00 KiB | 580.00 KiB / 1008.00 KiB |
| line-point | materialized | 128 | 337.03 KiB / 75.41 KiB | 377.73 KiB / 768 B | 448 B / 0 B | 3.47 KiB / 768 B | 0 B / 1.02 KiB | 892.00 KiB / 1.00 MiB | 896.00 KiB / 1.00 MiB |
| line-point | materialized | 512 | 1.32 MiB / 300.51 KiB | 1.50 MiB / 3.00 KiB | 648 B / 0 B | 6.50 KiB / 3.00 KiB | 0 B / 1.73 KiB | 2.75 MiB / 1.21 MiB | 2.75 MiB / 1.07 MiB |
| line-point | materialized | 2048 | 5.27 MiB / 1.63 MiB | 6.09 MiB / 12.00 KiB | 648 B / 0 B | 18.50 KiB / 12.00 KiB | 0 B / 1.93 KiB | 10.01 MiB / 2.98 MiB | 9.96 MiB / 2.93 MiB |
| line-point | materialized | 8192 | 21.06 MiB / 6.96 MiB | 24.48 MiB / 48.00 KiB | 648 B / 0 B | 66.50 KiB / 48.00 KiB | 0 B / 1.93 KiB | 38.79 MiB / 10.50 MiB | 38.60 MiB / 10.46 MiB |
| triangle-point | streaming | 4 | 18.94 KiB / 4.38 KiB | 116.42 KiB / 9.19 KiB | 448 B / 8.78 KiB | 9.47 KiB / 0 B | 0 B / 8.73 KiB | 460.00 KiB / 932.00 KiB | 460.00 KiB / 936.00 KiB |
| triangle-point | streaming | 16 | 75.66 KiB / 17.02 KiB | 474.80 KiB / 19.16 KiB | 448 B / 18.75 KiB | 9.47 KiB / 0 B | 0 B / 11.46 KiB | 628.00 KiB / 976.00 KiB | 676.00 KiB / 984.00 KiB |
| triangle-point | streaming | 64 | 302.53 KiB / 63.63 KiB | 1.86 MiB / 52.33 KiB | 448 B / 51.92 KiB | 9.47 KiB / 0 B | 0 B / 11.73 KiB | 992.00 KiB / 1.16 MiB | 996.00 KiB / 1.17 MiB |
| triangle-point | streaming | 256 | 1.18 MiB / 249.63 KiB | 7.46 MiB / 184.33 KiB | 648 B / 183.92 KiB | 9.49 KiB / 0 B | 0 B / 11.73 KiB | 2.56 MiB / 1.55 MiB | 2.56 MiB / 1.45 MiB |
| triangle-point | streaming | 1024 | 4.73 MiB / 1.17 MiB | 29.95 MiB / 1.04 MiB | 648 B / 1.04 MiB | 11.01 KiB / 0 B | 0 B / 15.15 KiB | 9.20 MiB / 3.74 MiB | 9.08 MiB / 3.62 MiB |
| triangle-point | streaming | 4096 | 18.91 MiB / 5.29 MiB | 119.88 MiB / 5.15 MiB | 648 B / 5.15 MiB | 11.02 KiB / 0 B | 0 B / 15.15 KiB | 35.52 MiB / 14.53 MiB | 35.46 MiB / 14.38 MiB |
| triangle-point | materialized | 4 | 18.94 KiB / 4.38 KiB | 116.45 KiB / 9.21 KiB | 448 B / 8.78 KiB | 9.50 KiB / 24 B | 0 B / 8.73 KiB | 416.00 KiB / 948.00 KiB | 416.00 KiB / 948.00 KiB |
| triangle-point | materialized | 16 | 75.66 KiB / 17.02 KiB | 474.92 KiB / 19.25 KiB | 448 B / 18.75 KiB | 9.59 KiB / 96 B | 0 B / 11.46 KiB | 584.00 KiB / 972.00 KiB | 588.00 KiB / 976.00 KiB |
| triangle-point | materialized | 64 | 302.53 KiB / 63.63 KiB | 1.86 MiB / 52.70 KiB | 448 B / 51.92 KiB | 9.97 KiB / 384 B | 0 B / 11.73 KiB | 940.00 KiB / 1.11 MiB | 944.00 KiB / 1.12 MiB |
| triangle-point | materialized | 256 | 1.18 MiB / 249.63 KiB | 7.46 MiB / 185.83 KiB | 648 B / 183.92 KiB | 11.49 KiB / 1.50 KiB | 0 B / 11.73 KiB | 2.63 MiB / 1.51 MiB | 2.63 MiB / 1.45 MiB |
| triangle-point | materialized | 1024 | 4.73 MiB / 1.17 MiB | 29.95 MiB / 1.04 MiB | 648 B / 1.04 MiB | 19.01 KiB / 6.00 KiB | 0 B / 15.15 KiB | 9.14 MiB / 3.73 MiB | 9.01 MiB / 3.59 MiB |
| triangle-point | materialized | 4096 | 18.91 MiB / 5.29 MiB | 119.92 MiB / 5.17 MiB | 648 B / 5.15 MiB | 43.02 KiB / 24.00 KiB | 0 B / 15.15 KiB | 35.52 MiB / 14.50 MiB | 35.46 MiB / 14.42 MiB |
| triangle-pair | streaming | 1 | 7.13 KiB / 1.55 KiB | 339.59 KiB / 14.33 KiB | 114.47 KiB / 13.41 KiB | 7.59 KiB / 104 B | 120.92 KiB / 14.13 KiB | 596.00 KiB / 1.43 MiB | 596.00 KiB / 1.43 MiB |
| triangle-pair | streaming | 4 | 28.41 KiB / 6.42 KiB | 1.20 MiB / 27.73 KiB | 452.30 KiB / 22.27 KiB | 5.70 KiB / 800 B | 479.42 KiB / 19.96 KiB | 1.09 MiB / 1.49 MiB | 1.10 MiB / 1.50 MiB |
| triangle-pair | streaming | 16 | 113.53 KiB / 24.98 KiB | 4.81 MiB / 94.46 KiB | 1.77 MiB / 51.12 KiB | 5.70 KiB / 904 B | 1.87 MiB / 30.20 KiB | 3.63 MiB / 1.49 MiB | 3.63 MiB / 1.50 MiB |
| triangle-pair | streaming | 64 | 454.03 KiB / 94.73 KiB | 19.29 MiB / 323.67 KiB | 7.06 MiB / 141.87 KiB | 5.70 KiB / 1.05 KiB | 7.49 MiB / 37.51 KiB | 13.52 MiB / 1.75 MiB | 13.52 MiB / 1.75 MiB |
| triangle-pair | streaming | 256 | 1.77 MiB / 373.73 KiB | 80.39 MiB / 1.19 MiB | 28.26 MiB / 476.95 KiB | 5.73 KiB / 1.05 KiB | 29.98 MiB / 37.71 KiB | 52.80 MiB / 2.48 MiB | 52.80 MiB / 2.36 MiB |
| triangle-pair | streaming | 1024 | 7.09 MiB / 1.80 MiB | 328.68 MiB / 7.19 MiB | 113.11 MiB / 2.71 MiB | 5.73 KiB / 904 B | 120.00 MiB / 43.72 KiB | 210.08 MiB / 7.42 MiB | 209.93 MiB / 7.31 MiB |
| triangle-pair | materialized | 1 | 7.13 KiB / 1.55 KiB | 339.60 KiB / 14.44 KiB | 114.47 KiB / 13.41 KiB | 7.59 KiB / 216 B | 120.92 KiB / 14.13 KiB | 556.00 KiB / 1.48 MiB | 556.00 KiB / 1.48 MiB |
| triangle-pair | materialized | 4 | 28.41 KiB / 6.42 KiB | 1.20 MiB / 28.16 KiB | 452.30 KiB / 22.27 KiB | 5.73 KiB / 1.22 KiB | 479.42 KiB / 19.96 KiB | 1.24 MiB / 1.51 MiB | 1.24 MiB / 1.52 MiB |
| triangle-pair | materialized | 16 | 113.53 KiB / 24.98 KiB | 4.81 MiB / 96.21 KiB | 1.77 MiB / 51.12 KiB | 5.83 KiB / 2.63 KiB | 1.87 MiB / 30.20 KiB | 3.71 MiB / 1.57 MiB | 3.71 MiB / 1.58 MiB |
| triangle-pair | materialized | 64 | 454.03 KiB / 94.73 KiB | 19.29 MiB / 330.67 KiB | 7.06 MiB / 141.87 KiB | 6.20 KiB / 8.05 KiB | 7.49 MiB / 37.51 KiB | 13.48 MiB / 1.70 MiB | 13.48 MiB / 1.70 MiB |
| triangle-pair | materialized | 256 | 1.77 MiB / 373.73 KiB | 80.39 MiB / 1.22 MiB | 28.26 MiB / 476.95 KiB | 7.73 KiB / 29.05 KiB | 29.98 MiB / 37.71 KiB | 52.87 MiB / 2.47 MiB | 52.87 MiB / 2.46 MiB |
| triangle-pair | materialized | 1024 | 7.09 MiB / 1.80 MiB | 328.69 MiB / 7.30 MiB | 113.11 MiB / 2.71 MiB | 13.73 KiB / 112.88 KiB | 120.00 MiB / 43.72 KiB | 210.17 MiB / 7.50 MiB | 209.96 MiB / 7.41 MiB |

RSS Δ is current rollup RSS after execution minus the pre-fixture baseline; HWM Δ is the process high-water delta.
