# Hypersolve Algebraic Fiber Benchmarks

Generated automatically by `cargo bench --bench algebraic_fiber`. These are deterministic wall-clock throughput probes with exact result checks, not Criterion statistical estimates. Override the default iteration count with `HYPERSOLVE_FIBER_BENCH_ITERATIONS`.

| Benchmark | Iterations | Total | Mean per iteration | Validation checksum |
| --- | ---: | ---: | ---: | --- |
| `algebraic_fiber_even_multiplicity` | 1000 | 0.012 s | 12.30 us | root=1000; refinement=4000 |
| `algebraic_fiber_eight_independent_intervals` | 1000 | 0.087 s | 87.29 us | root=4000 |
| `algebraic_fiber_eight_batched_intervals` | 1000 | 0.050 s | 49.91 us | root=4000 |
| `algebraic_common_fiber_degree_drop` | 1000 | 0.009 s | 9.18 us | root=1000; refinement=4000 |
| `algebraic_image_policy_zero_content` | 1000 | 0.006 s | 5.85 us | coefficient=2000 |
| `rational_quadratic_common_fiber_two_components` | 1000 | 0.127 s | 127.15 us | component=4000 |
| `implicit_quadratic_common_fiber` | 1000 | 0.070 s | 69.64 us | component=5000 |
| `implicit_quadratic_high_cofactor` | 1000 | 0.244 s | 243.70 us | component=7000 |
| `rational_repeated_cubic_common_fiber_three_components` | 1000 | 0.289 s | 289.07 us | component=6000 |
