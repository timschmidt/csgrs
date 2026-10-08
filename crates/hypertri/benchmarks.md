<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for Hypertri's retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 70810 -->
<!-- promoted_slow_previous_score_nanos: 73998 -->
<!-- promoted_slow_score_delta_nanos: -3188 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 70.810 us |
| Delta | -3.188 us |
| Delta derivative | -6.917 us |

| Rank | Current Time | Fuzz target | Input |
| ---: | ---: | --- | --- |
| 1 | 78.115 us | `hyperreal_representations` | `seed[8287]` |
| 2 | 77.355 us | `hyperreal_representations` | `seed[403]` |
| 3 | 76.225 us | `hyperreal_representations` | `seed[448]` |
| 4 | 76.115 us | `hyperreal_representations` | `seed[580]` |
| 5 | 75.945 us | `hyperreal_representations` | `seed[994]` |
| 6 | 75.295 us | `topology_invariants` | `seed[569]` |
| 7 | 75.224 us | `hyperreal_representations` | `seed[445]` |
| 8 | 75.085 us | `hyperreal_representations` | `seed[556]` |
| 9 | 75.075 us | `hyperreal_representations` | `seed[592]` |
| 10 | 74.985 us | `hyperreal_representations` | `seed[598]` |

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
| `competitive/delaunay` | `400` | `delaunator_f64` | 49.75 us | 1.00x |
| `competitive/delaunay` | `400` | `hypertri_exact_spatial` | 3.18 ms | 63.85x |
| `competitive/delaunay` | `400` | `hypertri_exact_prelifted` | 3.21 ms | 64.49x |
| `competitive/delaunay` | `400` | `hypertri_f64_boundary` | 3.24 ms | 65.21x |
| `competitive/delaunay` | `64` | `delaunator_f64` | 6.58 us | 1.00x |
| `competitive/delaunay` | `64` | `hypertri_exact_spatial` | 143.45 us | 21.79x |
| `competitive/delaunay` | `64` | `hypertri_exact_prelifted` | 155.23 us | 23.58x |
| `competitive/delaunay` | `64` | `hypertri_f64_boundary` | 169.04 us | 25.68x |
| `competitive/earcut` | `128` | `earcutr_f64` | 6.07 us | 1.00x |
| `competitive/earcut` | `128` | `hypertri_exact_prelifted` | 121.27 us | 19.98x |
| `competitive/earcut` | `128` | `hypertri_f64_boundary` | 153.16 us | 25.23x |
| `competitive/earcut` | `32` | `earcutr_f64` | 1.46 us | 1.00x |
| `competitive/earcut` | `32` | `hypertri_exact_prelifted` | 26.41 us | 18.10x |
| `competitive/earcut` | `32` | `hypertri_f64_boundary` | 35.51 us | 24.33x |

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `competitive/delaunay/delaunator_f64/400` | 49.75 us | 49.67 us - 49.84 us | 49.73 us | - | 400 elements |
| `competitive/delaunay/delaunator_f64/64` | 6.58 us | 6.55 us - 6.62 us | 6.58 us | - | 64 elements |
| `competitive/delaunay/hypertri_exact_prelifted/400` | 3.21 ms | 3.19 ms - 3.23 ms | 3.20 ms | - | 400 elements |
| `competitive/delaunay/hypertri_exact_prelifted/64` | 155.23 us | 154.80 us - 155.85 us | 154.95 us | - | 64 elements |
| `competitive/delaunay/hypertri_exact_spatial/400` | 3.18 ms | 3.17 ms - 3.19 ms | 3.17 ms | - | 400 elements |
| `competitive/delaunay/hypertri_exact_spatial/64` | 143.45 us | 142.30 us - 145.26 us | 142.42 us | - | 64 elements |
| `competitive/delaunay/hypertri_f64_boundary/400` | 3.24 ms | 3.24 ms - 3.25 ms | 3.24 ms | - | 400 elements |
| `competitive/delaunay/hypertri_f64_boundary/64` | 169.04 us | 168.59 us - 169.55 us | 168.88 us | - | 64 elements |
| `competitive/earcut/earcutr_f64/128` | 6.07 us | 6.06 us - 6.09 us | 6.06 us | - | 128 elements |
| `competitive/earcut/earcutr_f64/32` | 1.46 us | 1.45 us - 1.47 us | 1.45 us | - | 32 elements |
| `competitive/earcut/hypertri_exact_prelifted/128` | 121.27 us | 121.00 us - 121.54 us | 121.31 us | - | 128 elements |
| `competitive/earcut/hypertri_exact_prelifted/32` | 26.41 us | 26.08 us - 26.82 us | 25.98 us | - | 32 elements |
| `competitive/earcut/hypertri_f64_boundary/128` | 153.16 us | 152.74 us - 153.65 us | 153.06 us | - | 128 elements |
| `competitive/earcut/hypertri_f64_boundary/32` | 35.51 us | 35.47 us - 35.56 us | 35.50 us | - | 32 elements |
| `exact_cdt_crossing_constraint_split` | 9.85 us | 9.84 us - 9.87 us | 9.83 us | - | - |
| `exact_cdt_nonconvex_cavity_recovery` | 11.76 us | 11.75 us - 11.77 us | 11.75 us | - | - |
| `exact_cdt_separated_cycles_general_pslg` | 15.39 us | 15.35 us - 15.45 us | 15.33 us | - | - |
| `exact_cdt_validate_crossing_split` | 753.08 ns | 751.04 ns - 755.54 ns | 750.04 ns | - | - |
| `exact_delaunay_400_located_insertions` | 3.22 ms | 3.21 ms - 3.23 ms | 3.21 ms | - | - |
| `exact_delaunay_400_scattered_insertions` | 3.17 ms | 3.16 ms - 3.18 ms | 3.16 ms | - | - |
| `exact_delaunay_64_located_insertions` | 165.30 us | 165.09 us - 165.56 us | 165.03 us | - | - |
| `exact_delaunay_spatial_400_located_input` | 3.23 ms | 3.22 ms - 3.24 ms | 3.22 ms | - | - |
| `exact_delaunay_spatial_400_scattered_input` | 3.19 ms | 3.18 ms - 3.19 ms | 3.18 ms | - | - |
| `exact_delaunay_spatial_64_located_input` | 149.26 us | 149.04 us - 149.58 us | 149.12 us | - | - |
| `exact_delaunay_supertriangle_expansion` | 10.05 us | 10.03 us - 10.08 us | 10.01 us | - | - |
| `exact_nd_4d_delaunay_complex` | 21.06 us | 21.04 us - 21.09 us | 21.02 us | - | - |
| `exact_nd_4d_oracle_insertion_report` | 28.20 us | 28.13 us - 28.28 us | 28.09 us | - | - |
| `exact_nd_bistellar_flip_oracle_apply` | 4.32 us | 4.30 us - 4.36 us | 4.29 us | - | - |
| `exact_nd_bistellar_flip_validate` | 2.11 us | 2.10 us - 2.12 us | 2.10 us | - | - |
| `exact_nd_tds_combinatorial_report` | 161.66 ns | 161.19 ns - 162.19 ns | 160.71 ns | - | - |
| `exact_nd_tds_combinatorial_validate` | 164.23 ns | 163.84 ns - 164.78 ns | 163.65 ns | - | - |
| `exact_nd_tds_geometric_report` | 538.76 ns | 538.34 ns - 539.29 ns | 538.18 ns | - | - |
| `exact_nd_tds_manifold_report` | 381.43 ns | 380.55 ns - 382.48 ns | 379.90 ns | - | - |
| `exact_polygon_input_shared_denominator_facts` | 5.73 ns | 5.72 ns - 5.74 ns | 5.73 ns | - | - |
| `exact_rational_spike_earcut` | 1.58 us | 1.58 us - 1.58 us | 1.58 us | - | - |
| `exact_rational_spike_earcut_diagnostics` | 1.57 us | 1.57 us - 1.57 us | 1.57 us | - | - |
| `exact_rational_spike_earcut_dynamic_policy` | 1.59 us | 1.58 us - 1.59 us | 1.58 us | - | - |
| `exact_sawtooth_earcut_candidate_pressure` | 33.61 us | 33.51 us - 33.73 us | 33.47 us | - | - |
| `f64_exact_lifted_cdt_crossing_split` | 9.67 us | 9.66 us - 9.69 us | 9.66 us | - | - |
| `f64_exact_lifted_cdt_edge_flip_recovery` | 8.55 us | 8.55 us - 8.56 us | 8.55 us | - | - |
| `f64_exact_lifted_cdt_existing_vertex_split` | 1.87 us | 1.86 us - 1.87 us | 1.86 us | - | - |
| `f64_exact_lifted_closed_ring_cdt_hole` | 7.67 us | 7.65 us - 7.70 us | 7.64 us | - | - |
| `f64_exact_lifted_concave_earcut` | 2.98 us | 2.97 us - 2.98 us | 2.97 us | - | - |
| `f64_exact_lifted_holed_earcut` | 20.16 us | 20.14 us - 20.18 us | 20.16 us | - | - |
| `f64_exact_lifted_incremental_delaunay` | 12.31 us | 12.28 us - 12.34 us | 12.26 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_1096` | 71.75 us | 71.48 us - 72.04 us | 71.50 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_3400` | 73.69 us | 73.52 us - 73.90 us | 73.62 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_3403` | 73.71 us | 72.59 us - 75.13 us | 72.63 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_355` | 74.58 us | 73.35 us - 75.83 us | 74.52 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_397` | 72.41 us | 72.21 us - 72.64 us | 72.29 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_403` | 76.34 us | 76.13 us - 76.59 us | 76.28 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_406` | 76.21 us | 75.89 us - 76.55 us | 76.18 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_409` | 72.30 us | 72.00 us - 72.62 us | 72.14 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_412` | 69.17 us | 68.88 us - 69.53 us | 68.99 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_415` | 74.42 us | 74.31 us - 74.52 us | 74.48 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_421` | 71.39 us | 70.06 us - 72.96 us | 70.15 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_4267` | 70.41 us | 70.12 us - 70.82 us | 70.20 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_427` | 72.87 us | 69.20 us - 78.43 us | 69.41 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_436` | 72.10 us | 71.54 us - 73.01 us | 71.72 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_442` | 69.57 us | 69.49 us - 69.63 us | 69.58 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_445` | 75.32 us | 75.13 us - 75.49 us | 75.36 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_448` | 75.43 us | 75.27 us - 75.63 us | 75.35 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_451` | 71.40 us | 70.70 us - 72.46 us | 70.73 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_457` | 68.14 us | 67.91 us - 68.39 us | 68.03 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_460` | 68.83 us | 68.66 us - 69.06 us | 68.79 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_466` | 72.74 us | 72.55 us - 72.91 us | 72.83 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_469` | 71.97 us | 71.89 us - 72.07 us | 71.93 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_472` | 69.25 us | 67.55 us - 71.63 us | 67.83 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_478` | 74.56 us | 73.98 us - 75.46 us | 74.27 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_484` | 78.28 us | 74.57 us - 82.35 us | 76.53 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_487` | 68.90 us | 68.57 us - 69.17 us | 69.09 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_490` | 73.07 us | 71.47 us - 75.24 us | 71.66 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_493` | 72.60 us | 71.64 us - 73.85 us | 71.67 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_496` | 74.83 us | 74.66 us - 75.00 us | 74.83 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_499` | 75.62 us | 73.84 us - 78.00 us | 73.93 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_520` | 72.94 us | 72.83 us - 73.07 us | 72.88 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_523` | 67.49 us | 67.35 us - 67.66 us | 67.42 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_526` | 68.83 us | 68.66 us - 69.07 us | 68.69 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_5287` | 72.18 us | 69.23 us - 76.08 us | 69.52 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_529` | 74.44 us | 72.58 us - 77.09 us | 72.93 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_541` | 69.98 us | 69.85 us - 70.12 us | 69.92 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_544` | 74.20 us | 71.50 us - 77.53 us | 71.67 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_556` | 78.78 us | 75.74 us - 82.95 us | 75.90 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_580` | 75.05 us | 74.90 us - 75.21 us | 74.99 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_592` | 74.72 us | 74.05 us - 75.76 us | 74.26 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_598` | 75.16 us | 74.78 us - 75.55 us | 75.13 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_607` | 67.89 us | 67.81 us - 68.00 us | 67.84 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_613` | 68.14 us | 67.86 us - 68.45 us | 68.13 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_622` | 73.21 us | 72.99 us - 73.51 us | 73.05 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_625` | 74.99 us | 74.22 us - 75.86 us | 74.73 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_634` | 72.43 us | 72.21 us - 72.66 us | 72.29 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_637` | 73.18 us | 72.86 us - 73.61 us | 73.01 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_655` | 76.49 us | 74.78 us - 78.51 us | 74.73 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_667` | 69.55 us | 69.06 us - 70.12 us | 69.34 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_673` | 72.88 us | 72.36 us - 73.74 us | 72.53 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_679` | 78.45 us | 74.99 us - 82.91 us | 75.08 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_682` | 72.07 us | 71.88 us - 72.34 us | 71.97 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_691` | 70.85 us | 70.70 us - 71.01 us | 70.79 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_706` | 68.59 us | 68.43 us - 68.76 us | 68.51 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_709` | 73.54 us | 72.65 us - 75.11 us | 72.74 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_712` | 71.27 us | 71.14 us - 71.42 us | 71.28 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_718` | 68.84 us | 68.58 us - 69.05 us | 68.93 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_733` | 69.27 us | 69.14 us - 69.39 us | 69.30 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_736` | 73.42 us | 73.30 us - 73.57 us | 73.36 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_745` | 81.44 us | 77.60 us - 85.45 us | 76.85 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_748` | 72.38 us | 71.27 us - 73.89 us | 71.48 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_751` | 70.41 us | 70.13 us - 70.79 us | 70.36 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_754` | 69.59 us | 69.30 us - 69.91 us | 69.54 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_766` | 66.85 us | 66.19 us - 67.61 us | 66.61 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_769` | 69.43 us | 69.10 us - 69.80 us | 69.31 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_775` | 72.93 us | 72.80 us - 73.09 us | 72.88 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_778` | 73.23 us | 72.99 us - 73.45 us | 73.27 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_787` | 68.73 us | 68.56 us - 68.91 us | 68.66 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_796` | 68.13 us | 68.04 us - 68.23 us | 68.07 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_799` | 68.57 us | 68.41 us - 68.73 us | 68.51 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_805` | 73.76 us | 73.61 us - 73.90 us | 73.84 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_808` | 72.22 us | 72.12 us - 72.33 us | 72.22 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_817` | 69.52 us | 69.32 us - 69.70 us | 69.53 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_820` | 74.69 us | 74.42 us - 75.03 us | 74.52 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_823` | 70.53 us | 68.85 us - 72.50 us | 69.06 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_826` | 69.33 us | 68.55 us - 70.73 us | 68.64 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_8287` | 82.61 us | 80.96 us - 85.17 us | 81.16 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_829` | 71.13 us | 68.82 us - 74.24 us | 68.39 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_832` | 69.66 us | 69.53 us - 69.78 us | 69.64 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_835` | 75.46 us | 75.23 us - 75.79 us | 75.35 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_838` | 68.06 us | 67.84 us - 68.39 us | 67.95 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_841` | 66.66 us | 66.41 us - 66.90 us | 66.67 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_847` | 65.77 us | 65.42 us - 66.17 us | 65.56 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_850` | 78.49 us | 74.85 us - 82.75 us | 74.74 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_853` | 69.20 us | 68.81 us - 69.70 us | 68.91 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_859` | 70.07 us | 68.33 us - 73.39 us | 68.49 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_862` | 70.43 us | 70.01 us - 70.92 us | 70.27 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_865` | 71.31 us | 69.43 us - 74.57 us | 69.57 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_868` | 75.10 us | 73.99 us - 76.82 us | 74.09 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_871` | 70.44 us | 69.67 us - 71.45 us | 70.35 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_874` | 75.58 us | 73.30 us - 79.09 us | 73.48 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_877` | 72.89 us | 72.70 us - 73.19 us | 72.75 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_892` | 69.66 us | 69.51 us - 69.85 us | 69.67 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_895` | 68.78 us | 68.56 us - 69.06 us | 68.58 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_898` | 69.32 us | 68.87 us - 69.80 us | 69.18 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_901` | 67.81 us | 67.26 us - 68.69 us | 67.34 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_985` | 71.43 us | 70.25 us - 73.17 us | 70.47 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_991` | 69.52 us | 69.16 us - 69.94 us | 69.39 us | - | - |
| `promoted_fuzz_worst_performers/hyperreal_representations_seed_994` | 78.23 us | 75.62 us - 82.24 us | 75.59 us | - | - |
| `promoted_fuzz_worst_performers/topology_invariants_seed_569` | 74.56 us | 74.29 us - 74.84 us | 74.47 us | - | - |
| `promoted_slow_offender_score/replay_promoted_100` | 7.47 ms | 7.46 ms - 7.49 ms | 7.46 ms | - | - |
| `real_representations/full_topology/ConstOffset` | 1.41 ms | 1.40 ms - 1.44 ms | 1.40 ms | - | - |
| `real_representations/full_topology/ConstProduct` | 2.14 ms | 2.14 ms - 2.16 ms | 2.14 ms | - | - |
| `real_representations/full_topology/ConstProductSqrt` | 2.07 ms | 2.07 ms - 2.08 ms | 2.07 ms | - | - |
| `real_representations/full_topology/Exp` | 1.70 ms | 1.69 ms - 1.70 ms | 1.69 ms | - | - |
| `real_representations/full_topology/Irrational` | 1.45 ms | 1.45 ms - 1.46 ms | 1.45 ms | - | - |
| `real_representations/full_topology/Ln` | 1.44 ms | 1.44 ms - 1.46 ms | 1.44 ms | - | - |
| `real_representations/full_topology/LnAffine` | 1.59 ms | 1.59 ms - 1.60 ms | 1.59 ms | - | - |
| `real_representations/full_topology/LnProduct` | 1.51 ms | 1.50 ms - 1.52 ms | 1.50 ms | - | - |
| `real_representations/full_topology/Log10` | 1.54 ms | 1.54 ms - 1.54 ms | 1.54 ms | - | - |
| `real_representations/full_topology/Log2` | 1.52 ms | 1.52 ms - 1.52 ms | 1.52 ms | - | - |
| `real_representations/full_topology/One` | 28.18 us | 27.71 us - 28.89 us | 27.77 us | - | - |
| `real_representations/full_topology/Pi` | 1.27 ms | 1.27 ms - 1.28 ms | 1.27 ms | - | - |
| `real_representations/full_topology/PiExp` | 1.95 ms | 1.92 ms - 1.98 ms | 1.92 ms | - | - |
| `real_representations/full_topology/PiInv` | 1.12 ms | 1.11 ms - 1.14 ms | 1.11 ms | - | - |
| `real_representations/full_topology/PiInvExp` | 1.85 ms | 1.85 ms - 1.86 ms | 1.85 ms | - | - |
| `real_representations/full_topology/PiPow` | 1.29 ms | 1.29 ms - 1.30 ms | 1.29 ms | - | - |
| `real_representations/full_topology/PiSqrt` | 1.44 ms | 1.44 ms - 1.45 ms | 1.44 ms | - | - |
| `real_representations/full_topology/Pow10` | 1.52 ms | 1.51 ms - 1.54 ms | 1.51 ms | - | - |
| `real_representations/full_topology/Pow2` | 1.51 ms | 1.50 ms - 1.51 ms | 1.51 ms | - | - |
| `real_representations/full_topology/SinPi` | 1.47 ms | 1.47 ms - 1.47 ms | 1.47 ms | - | - |
| `real_representations/full_topology/Sqrt` | 1.21 ms | 1.21 ms - 1.21 ms | 1.21 ms | - | - |
| `real_representations/full_topology/TanPi` | 1.47 ms | 1.47 ms - 1.47 ms | 1.47 ms | - | - |
| `runtime_polygon_triangulation` | 1.62 us | 1.60 us - 1.63 us | 1.60 us | - | - |
| `runtime_polygon_triangulation_report` | 1.62 us | 1.62 us - 1.62 us | 1.62 us | - | - |

<!-- END COMPLETE BENCHMARK REPORT -->
