<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for Hypertri's retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 73998 -->
<!-- promoted_slow_previous_score_nanos: 70269 -->
<!-- promoted_slow_score_delta_nanos: 3729 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 73.998 us |
| Delta | 3.729 us |
| Delta derivative | 3.729 us |

| Rank | Current Time | Fuzz target | Input |
| ---: | ---: | --- | --- |
| 1 | 84.504 us | `hyperreal_representations` | `seed[8287]` |
| 2 | 83.625 us | `hyperreal_representations` | `seed[820]` |
| 3 | 82.495 us | `hyperreal_representations` | `seed[415]` |
| 4 | 81.934 us | `hyperreal_representations` | `seed[403]` |
| 5 | 80.544 us | `hyperreal_representations` | `seed[835]` |
| 6 | 79.954 us | `topology_invariants` | `seed[569]` |
| 7 | 79.595 us | `hyperreal_representations` | `seed[496]` |
| 8 | 78.865 us | `hyperreal_representations` | `seed[679]` |
| 9 | 78.794 us | `hyperreal_representations` | `seed[994]` |
| 10 | 78.435 us | `hyperreal_representations` | `seed[598]` |

<!-- END promoted_slow_offender_score -->







# Hypertri Benchmarks

This file is updated automatically by the benchmark binaries.

<!-- BEGIN COMPLETE BENCHMARK REPORT -->
## Complete generated benchmark report

Every registered benchmark target is catalogued below. Every Criterion result found under `target/criterion` is included without a name or implementation filter; non-Criterion targets write their own linked reports. Each timing binary refreshes this section after it runs.

Run the complete non-instrumented timing set with:

```sh
cargo bench --features all-algorithms,runtime-select,f64-interop
```

Regenerate this Markdown from stored Criterion data without rerunning benchmarks:

```sh
cargo run --example write_benchmarks_md
```

### Registered benchmark suites

| Target | Kind | Required features | Command | Generated report |
| --- | --- | --- | --- | --- |
| `competitive` | Criterion timing | `all-algorithms, f64-interop` | `cargo bench --bench competitive --features all-algorithms,f64-interop` | this file |
| `delaunay` | Criterion timing | `cdt, f64-interop` | `cargo bench --bench delaunay --features cdt,f64-interop` | this file |
| `dispatch_trace` | diagnostic | `all-algorithms, runtime-select, dispatch-trace` | `cargo bench --bench dispatch_trace --features all-algorithms,runtime-select,dispatch-trace` | [dispatch_trace.md](dispatch_trace.md) |
| `earcut` | Criterion timing | `earcut, f64-interop` | `cargo bench --bench earcut --features earcut,f64-interop` | this file |
| `exact` | Criterion timing | `earcut, cdt, nd, runtime-select` | `cargo bench --bench exact --features earcut,cdt,nd,runtime-select` | this file |
| `representations` | Criterion timing | `all-algorithms, runtime-select` | `cargo bench --bench representations --features all-algorithms,runtime-select` | this file |
| `retained_fuzz` | Criterion timing | `all-algorithms, runtime-select` | `cargo bench --bench retained_fuzz --features all-algorithms,runtime-select` | this file |

### Comparative results

Rows sharing a Criterion group and input are compared when they expose distinct implementations. Ratios are elapsed time relative to the fastest stored row; they do not imply identical guarantees or output semantics.

| Group | Input | Implementation | Mean | Relative to fastest |
| --- | --- | --- | ---: | ---: |
| `competitive/delaunay` | `400` | `delaunator_f64` | 52.64 us | 1.00x |
| `competitive/delaunay` | `400` | `hypertri_exact_spatial` | 3.41 ms | 64.71x |
| `competitive/delaunay` | `400` | `hypertri_exact_prelifted` | 3.50 ms | 66.45x |
| `competitive/delaunay` | `400` | `hypertri_f64_boundary` | 3.53 ms | 67.06x |
| `competitive/delaunay` | `64` | `delaunator_f64` | 6.89 us | 1.00x |
| `competitive/delaunay` | `64` | `hypertri_exact_spatial` | 157.38 us | 22.85x |
| `competitive/delaunay` | `64` | `hypertri_f64_boundary` | 192.12 us | 27.89x |
| `competitive/delaunay` | `64` | `hypertri_exact_prelifted` | 197.70 us | 28.71x |
| `competitive/earcut` | `128` | `earcutr_f64` | 6.00 us | 1.00x |
| `competitive/earcut` | `128` | `hypertri_exact_prelifted` | 101.26 us | 16.87x |
| `competitive/earcut` | `128` | `hypertri_f64_boundary` | 132.21 us | 22.03x |
| `competitive/earcut` | `32` | `earcutr_f64` | 1.55 us | 1.00x |
| `competitive/earcut` | `32` | `hypertri_exact_prelifted` | 22.93 us | 14.81x |
| `competitive/earcut` | `32` | `hypertri_f64_boundary` | 32.37 us | 20.90x |

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `competitive/delaunay/delaunator_f64/400` | 52.64 us | 51.67 us - 53.79 us | 51.75 us | - | 400 elements |
| `competitive/delaunay/delaunator_f64/64` | 6.89 us | 6.73 us - 7.07 us | 6.73 us | - | 64 elements |
| `competitive/delaunay/hypertri_exact_prelifted/400` | 3.50 ms | 3.46 ms - 3.54 ms | 3.48 ms | - | 400 elements |
| `competitive/delaunay/hypertri_exact_prelifted/64` | 197.70 us | 180.02 us - 221.43 us | 182.28 us | - | 64 elements |
| `competitive/delaunay/hypertri_exact_spatial/400` | 3.41 ms | 3.35 ms - 3.50 ms | 3.36 ms | - | 400 elements |
| `competitive/delaunay/hypertri_exact_spatial/64` | 157.38 us | 156.26 us - 158.50 us | 157.33 us | - | 64 elements |
| `competitive/delaunay/hypertri_f64_boundary/400` | 3.53 ms | 3.49 ms - 3.57 ms | 3.51 ms | - | 400 elements |
| `competitive/delaunay/hypertri_f64_boundary/64` | 192.12 us | 190.98 us - 193.21 us | 192.45 us | - | 64 elements |
| `competitive/earcut/earcutr_f64/128` | 6.00 us | 5.99 us - 6.02 us | 5.99 us | - | 128 elements |
| `competitive/earcut/earcutr_f64/32` | 1.55 us | 1.52 us - 1.58 us | 1.52 us | - | 32 elements |
| `competitive/earcut/hypertri_exact_prelifted/128` | 101.26 us | 101.08 us - 101.45 us | 101.23 us | - | 128 elements |
| `competitive/earcut/hypertri_exact_prelifted/32` | 22.93 us | 22.83 us - 23.05 us | 22.92 us | - | 32 elements |
| `competitive/earcut/hypertri_f64_boundary/128` | 132.21 us | 130.35 us - 134.51 us | 130.44 us | - | 128 elements |
| `competitive/earcut/hypertri_f64_boundary/32` | 32.37 us | 32.08 us - 32.74 us | 32.21 us | - | 32 elements |
| `f64_exact_lifted_concave_earcut` | 2.79 us | 2.63 us - 2.97 us | 2.61 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_1` | 114.27 us | 112.87 us - 115.68 us | 113.79 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_10` | 641.75 us | 128.30 us - 1.39 ms | 127.67 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_1096` | 82.71 us | 76.08 us - 90.71 us | 78.71 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_11` | 67.00 us | 65.87 us - 68.22 us | 66.55 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_13` | 176.56 us | 101.37 us - 269.02 us | 102.22 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_14` | 71.11 us | 66.31 us - 77.70 us | 69.92 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_16` | 139.56 us | 127.85 us - 153.83 us | 139.94 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_17` | 64.45 us | 62.27 us - 66.91 us | 63.55 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_19` | 367.43 us | 186.73 us - 584.41 us | 172.17 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_2` | 66.07 us | 62.27 us - 72.57 us | 63.62 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_20` | 123.59 us | 78.30 us - 175.71 us | 78.63 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_22` | 163.45 us | 123.10 us - 213.09 us | 119.89 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_23` | 111.26 us | 98.49 us - 131.04 us | 102.72 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_25` | 167.12 us | 147.70 us - 189.73 us | 160.16 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_26` | 94.94 us | 89.30 us - 100.78 us | 89.45 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_28` | 157.56 us | 126.68 us - 195.17 us | 136.00 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_29` | 65.25 us | 63.89 us - 66.95 us | 64.36 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_31` | 145.29 us | 120.06 us - 176.90 us | 124.32 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_32` | 264.41 us | 152.08 us - 402.91 us | 169.94 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_34` | 270.18 us | 158.61 us - 403.42 us | 144.77 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_3403` | 81.43 us | 77.90 us - 85.59 us | 78.11 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_35` | 136.09 us | 84.44 us - 236.01 us | 87.26 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_355` | 76.97 us | 76.36 us - 77.57 us | 77.31 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_37` | 116.68 us | 115.17 us - 118.42 us | 115.98 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_38` | 131.09 us | 69.54 us - 237.26 us | 70.69 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_397` | 87.19 us | 80.62 us - 94.24 us | 84.54 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_4` | 175.87 us | 119.46 us - 237.04 us | 157.76 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_40` | 146.51 us | 115.67 us - 187.63 us | 116.90 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_403` | 83.35 us | 80.75 us - 86.02 us | 83.57 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_406` | 82.33 us | 79.83 us - 85.19 us | 80.27 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_409` | 78.15 us | 76.68 us - 80.12 us | 76.94 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_41` | 75.20 us | 70.83 us - 79.73 us | 74.93 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_412` | 70.62 us | 69.11 us - 72.92 us | 69.14 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_415` | 87.87 us | 79.84 us - 99.98 us | 82.45 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_421` | 75.81 us | 73.57 us - 78.37 us | 73.79 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_4267` | 75.39 us | 72.76 us - 78.40 us | 73.33 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_427` | 72.45 us | 72.01 us - 72.91 us | 72.46 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_43` | 154.12 us | 123.49 us - 189.29 us | 124.54 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_436` | 75.72 us | 74.54 us - 77.63 us | 74.82 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_44` | 65.88 us | 64.67 us - 67.11 us | 64.80 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_442` | 71.86 us | 71.38 us - 72.37 us | 71.87 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_445` | 82.63 us | 76.46 us - 90.68 us | 76.62 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_448` | 82.01 us | 79.89 us - 84.81 us | 80.09 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_451` | 82.31 us | 77.51 us - 88.19 us | 77.85 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_454` | 75.64 us | 72.81 us - 79.02 us | 74.20 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_457` | 71.39 us | 70.88 us - 72.08 us | 71.09 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_46` | 125.11 us | 114.22 us - 139.25 us | 117.28 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_460` | 76.40 us | 74.09 us - 79.65 us | 75.01 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_466` | 75.92 us | 73.81 us - 78.26 us | 74.47 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_469` | 85.61 us | 78.51 us - 93.62 us | 80.46 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_472` | 89.65 us | 80.43 us - 100.44 us | 83.01 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_478` | 80.19 us | 77.95 us - 82.60 us | 79.53 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_484` | 78.49 us | 77.23 us - 80.17 us | 78.28 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_487` | 72.73 us | 71.98 us - 73.69 us | 72.07 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_49` | 132.62 us | 122.19 us - 143.97 us | 121.27 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_490` | 76.95 us | 74.63 us - 79.51 us | 75.41 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_493` | 75.51 us | 73.61 us - 77.99 us | 74.34 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_496` | 84.72 us | 80.98 us - 88.87 us | 81.77 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_499` | 78.26 us | 76.22 us - 80.51 us | 76.70 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_5` | 93.79 us | 75.87 us - 121.17 us | 78.13 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_50` | 67.65 us | 65.02 us - 70.79 us | 66.26 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_52` | 200.73 us | 132.53 us - 304.25 us | 131.55 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_520` | 78.79 us | 76.92 us - 80.97 us | 77.55 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_523` | 74.20 us | 69.63 us - 81.42 us | 69.98 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_526` | 73.42 us | 71.61 us - 75.77 us | 71.97 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_5287` | 71.73 us | 71.16 us - 72.31 us | 71.58 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_529` | 77.92 us | 77.29 us - 78.57 us | 77.87 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_53` | 109.43 us | 70.02 us - 161.73 us | 67.98 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_541` | 83.33 us | 76.87 us - 91.48 us | 78.17 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_544` | 75.23 us | 74.47 us - 76.02 us | 75.20 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_55` | 445.56 us | 134.38 us - 875.95 us | 130.92 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_556` | 78.61 us | 78.12 us - 79.13 us | 78.40 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_56` | 74.72 us | 62.77 us - 89.32 us | 62.80 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_58` | 179.51 us | 139.76 us - 221.29 us | 156.34 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_580` | 82.12 us | 80.09 us - 84.58 us | 80.77 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_59` | 99.19 us | 88.97 us - 110.99 us | 89.30 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_592` | 78.76 us | 78.10 us - 79.43 us | 78.80 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_598` | 79.48 us | 78.88 us - 80.14 us | 79.45 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_607` | 79.96 us | 74.92 us - 87.21 us | 77.46 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_61` | 130.01 us | 126.22 us - 134.79 us | 129.70 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_613` | 72.92 us | 71.17 us - 75.33 us | 71.32 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_62` | 251.40 us | 103.04 us - 422.80 us | 99.63 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_622` | 77.81 us | 76.84 us - 79.22 us | 77.29 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_625` | 81.66 us | 78.57 us - 85.60 us | 80.01 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_634` | 82.44 us | 79.17 us - 86.63 us | 80.59 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_637` | 86.09 us | 81.75 us - 92.16 us | 84.36 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_64` | 218.59 us | 124.83 us - 337.14 us | 128.79 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_655` | 76.86 us | 75.71 us - 78.23 us | 76.28 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_667` | 76.58 us | 73.43 us - 80.33 us | 74.56 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_67` | 391.81 us | 213.13 us - 654.81 us | 343.37 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_673` | 82.55 us | 79.79 us - 85.45 us | 81.96 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_679` | 80.02 us | 75.33 us - 86.48 us | 75.53 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_68` | 76.63 us | 67.07 us - 89.18 us | 69.39 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_682` | 74.79 us | 73.85 us - 75.84 us | 74.66 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_691` | 77.90 us | 75.47 us - 80.81 us | 76.28 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_7` | 121.03 us | 114.42 us - 129.31 us | 115.20 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_70` | 100.11 us | 85.99 us - 122.78 us | 88.93 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_706` | 74.91 us | 72.66 us - 77.94 us | 73.01 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_709` | 83.80 us | 80.20 us - 87.61 us | 83.19 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_71` | 277.97 us | 151.58 us - 436.97 us | 259.75 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_712` | 78.09 us | 76.51 us - 79.89 us | 77.24 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_718` | 71.48 us | 70.55 us - 72.39 us | 71.82 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_73` | 130.08 us | 99.16 us - 168.98 us | 101.86 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_733` | 72.77 us | 70.06 us - 76.21 us | 70.78 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_736` | 83.42 us | 79.01 us - 88.48 us | 79.48 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_74` | 83.75 us | 63.07 us - 112.10 us | 63.48 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_745` | 78.23 us | 77.08 us - 79.28 us | 78.88 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_748` | 73.35 us | 72.03 us - 75.53 us | 72.32 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_751` | 77.34 us | 73.15 us - 82.09 us | 73.33 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_754` | 74.53 us | 73.06 us - 76.67 us | 73.30 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_76` | 169.71 us | 125.08 us - 219.87 us | 127.92 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_766` | 71.13 us | 68.90 us - 73.61 us | 68.48 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_769` | 77.11 us | 74.40 us - 80.81 us | 75.35 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_77` | 462.52 us | 177.35 us - 775.27 us | 90.50 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_775` | 77.16 us | 76.41 us - 78.17 us | 76.68 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_778` | 76.47 us | 75.98 us - 77.06 us | 76.20 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_787` | 69.71 us | 69.26 us - 70.22 us | 69.61 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_79` | 155.59 us | 125.98 us - 205.89 us | 131.92 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_796` | 76.90 us | 73.90 us - 80.09 us | 74.98 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_799` | 73.19 us | 72.78 us - 73.61 us | 73.15 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_8` | 426.99 us | 278.73 us - 566.62 us | 495.49 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_80` | 66.95 us | 63.69 us - 72.58 us | 64.30 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_805` | 80.31 us | 78.46 us - 82.31 us | 79.82 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_808` | 77.39 us | 76.32 us - 78.61 us | 76.47 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_817` | 73.21 us | 72.59 us - 74.00 us | 73.11 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_82` | 144.00 us | 112.30 us - 192.65 us | 111.75 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_820` | 79.49 us | 78.19 us - 81.49 us | 78.40 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_823` | 82.68 us | 74.59 us - 93.21 us | 75.88 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_826` | 74.24 us | 73.41 us - 75.21 us | 74.08 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_8287` | 87.48 us | 84.78 us - 90.76 us | 85.79 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_829` | 75.05 us | 72.36 us - 78.29 us | 74.45 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_83` | 63.66 us | 62.80 us - 64.83 us | 63.24 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_832` | 71.50 us | 70.87 us - 72.07 us | 71.86 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_835` | 86.88 us | 82.46 us - 91.51 us | 83.66 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_838` | 70.39 us | 70.02 us - 70.79 us | 70.28 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_841` | 74.71 us | 72.12 us - 77.45 us | 74.16 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_847` | 69.66 us | 68.14 us - 72.18 us | 68.41 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_85` | 74.69 us | 74.31 us - 75.04 us | 74.77 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_850` | 77.01 us | 76.24 us - 77.86 us | 76.73 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_853` | 71.12 us | 70.35 us - 71.90 us | 71.18 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_859` | 72.11 us | 70.30 us - 74.74 us | 70.60 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_862` | 75.45 us | 73.98 us - 77.17 us | 74.47 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_865` | 71.75 us | 69.94 us - 74.13 us | 70.19 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_868` | 80.02 us | 77.10 us - 84.28 us | 77.74 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_871` | 79.40 us | 74.30 us - 85.26 us | 75.22 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_874` | 79.18 us | 77.03 us - 81.93 us | 77.45 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_877` | 77.12 us | 75.59 us - 79.31 us | 76.45 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_88` | 132.42 us | 108.51 us - 165.19 us | 109.90 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_89` | 110.83 us | 101.00 us - 121.61 us | 107.16 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_892` | 69.68 us | 69.14 us - 70.20 us | 69.72 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_895` | 71.74 us | 71.04 us - 72.47 us | 71.46 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_898` | 71.50 us | 70.30 us - 73.10 us | 71.19 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_901` | 75.35 us | 72.86 us - 77.93 us | 74.94 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_91` | 139.92 us | 115.72 us - 178.99 us | 127.12 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_94` | 223.99 us | 149.78 us - 307.71 us | 134.34 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_95` | 75.38 us | 65.02 us - 86.53 us | 65.51 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_97` | 418.51 us | 195.86 us - 683.97 us | 173.34 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_985` | 79.10 us | 75.57 us - 83.47 us | 75.71 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_991` | 74.42 us | 71.16 us - 78.23 us | 71.34 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_994` | 83.09 us | 80.66 us - 86.01 us | 81.77 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_11` | 64.97 us | 51.01 us - 79.45 us | 47.24 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_13` | 87.60 us | 68.15 us - 109.08 us | 64.33 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_19` | 46.93 us | 42.31 us - 54.07 us | 42.62 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_21` | 184.73 us | 97.67 us - 294.55 us | 116.42 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_23` | 72.28 us | 64.60 us - 80.14 us | 71.56 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_25` | 60.98 us | 60.58 us - 61.42 us | 60.82 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_27` | 41.38 us | 36.79 us - 45.27 us | 44.91 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_29` | 51.49 us | 49.65 us - 53.94 us | 50.19 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_3` | 41.52 us | 41.14 us - 41.98 us | 41.44 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_31` | 96.71 us | 57.45 us - 142.37 us | 52.06 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_33` | 161.55 us | 72.62 us - 273.26 us | 64.30 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_35` | 61.62 us | 56.53 us - 70.08 us | 57.79 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_3693` | 252.06 us | 228.61 us - 277.61 us | 243.02 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_39` | 62.61 us | 60.55 us - 64.72 us | 62.28 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_41` | 62.05 us | 47.80 us - 84.07 us | 57.26 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_43` | 81.23 us | 66.34 us - 95.79 us | 84.99 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_47` | 81.08 us | 60.11 us - 104.82 us | 56.52 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_49` | 500.50 us | 235.41 us - 832.78 us | 280.94 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_51` | 78.07 us | 65.98 us - 90.32 us | 76.33 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_53` | 107.34 us | 84.64 us - 130.03 us | 116.17 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_55` | 71.35 us | 60.46 us - 86.73 us | 60.95 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_569` | 84.25 us | 81.62 us - 86.82 us | 83.92 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_59` | 154.54 us | 127.64 us - 180.74 us | 158.18 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_63` | 110.42 us | 77.55 us - 150.97 us | 99.83 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_65` | 68.22 us | 59.59 us - 78.12 us | 61.58 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_67` | 57.32 us | 53.31 us - 62.63 us | 54.29 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_69` | 131.45 us | 102.25 us - 159.75 us | 138.95 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_7` | 49.93 us | 46.11 us - 54.86 us | 46.76 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_71` | 64.25 us | 63.33 us - 65.27 us | 64.13 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_75` | 108.85 us | 87.20 us - 128.70 us | 118.15 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_77` | 65.88 us | 63.81 us - 69.60 us | 64.34 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_79` | 89.61 us | 67.60 us - 116.84 us | 69.73 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_81` | 101.01 us | 73.90 us - 132.32 us | 76.10 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_83` | 86.73 us | 73.33 us - 101.92 us | 73.34 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_87` | 64.13 us | 58.78 us - 70.31 us | 60.83 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_9` | 139.39 us | 97.87 us - 185.66 us | 130.29 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_91` | 135.55 us | 86.91 us - 199.17 us | 113.57 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_93` | 112.67 us | 93.47 us - 131.62 us | 119.80 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_95` | 46.68 us | 45.51 us - 48.02 us | 46.40 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_99` | 62.44 us | 53.41 us - 71.87 us | 61.24 us | - | - |
| `promoted_slow_offender_score/replay_promoted_100` | 8.35 ms | 7.96 ms - 8.86 ms | 7.98 ms | +50.16% | - |
| `real_representations/full_topology/ConstOffset` | 5.37 ms | 4.04 ms - 6.74 ms | 5.26 ms | - | - |
| `real_representations/full_topology/ConstProduct` | 6.14 ms | 4.94 ms - 7.39 ms | 5.95 ms | - | - |
| `real_representations/full_topology/ConstProductSqrt` | 5.27 ms | 5.25 ms - 5.28 ms | 5.28 ms | - | - |
| `real_representations/full_topology/Exp` | 1.74 ms | 1.71 ms - 1.77 ms | 1.73 ms | - | - |
| `real_representations/full_topology/Irrational` | 1.53 ms | 1.51 ms - 1.56 ms | 1.53 ms | - | - |
| `real_representations/full_topology/Ln` | 1.41 ms | 1.39 ms - 1.43 ms | 1.39 ms | - | - |
| `real_representations/full_topology/LnAffine` | 2.07 ms | 1.83 ms - 2.35 ms | 1.78 ms | - | - |
| `real_representations/full_topology/LnProduct` | 4.60 ms | 3.73 ms - 5.45 ms | 5.43 ms | - | - |
| `real_representations/full_topology/Log10` | 6.12 ms | 4.71 ms - 7.52 ms | 6.47 ms | - | - |
| `real_representations/full_topology/Log2` | 3.06 ms | 2.98 ms - 3.13 ms | 3.06 ms | - | - |
| `real_representations/full_topology/One` | 28.59 us | 28.19 us - 29.06 us | 28.18 us | - | - |
| `real_representations/full_topology/Pi` | 1.43 ms | 1.41 ms - 1.45 ms | 1.43 ms | - | - |
| `real_representations/full_topology/PiExp` | 2.58 ms | 2.50 ms - 2.65 ms | 2.62 ms | - | - |
| `real_representations/full_topology/PiInv` | 4.44 ms | 3.51 ms - 5.48 ms | 4.40 ms | - | - |
| `real_representations/full_topology/PiInvExp` | 1.93 ms | 1.88 ms - 1.98 ms | 1.92 ms | - | - |
| `real_representations/full_topology/PiPow` | 2.91 ms | 2.80 ms - 3.01 ms | 2.93 ms | - | - |
| `real_representations/full_topology/PiSqrt` | 2.22 ms | 2.13 ms - 2.34 ms | 2.19 ms | - | - |
| `real_representations/full_topology/Pow10` | 2.96 ms | 2.84 ms - 3.13 ms | 2.86 ms | - | - |
| `real_representations/full_topology/Pow2` | 2.54 ms | 2.52 ms - 2.56 ms | 2.54 ms | - | - |
| `real_representations/full_topology/SinPi` | 2.60 ms | 2.55 ms - 2.66 ms | 2.58 ms | - | - |
| `real_representations/full_topology/Sqrt` | 1.06 ms | 1.03 ms - 1.10 ms | 1.05 ms | - | - |
| `real_representations/full_topology/TanPi` | 5.64 ms | 5.55 ms - 5.73 ms | 5.65 ms | - | - |

<!-- END COMPLETE BENCHMARK REPORT -->
