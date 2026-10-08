# Hypersolve Algebraic Fiber Benchmarks

Generated automatically by `cargo bench --bench algebraic_fiber`. These are deterministic wall-clock throughput probes with exact result checks, not Criterion statistical estimates. Override the default iteration count with `HYPERSOLVE_FIBER_BENCH_ITERATIONS`.

| Benchmark | Iterations | Total | Mean per iteration | Validation checksum |
| --- | ---: | ---: | ---: | --- |
| `algebraic_fiber_even_multiplicity` | 1000 | 0.111 s | 110.55 us | root=1000; refinement=4000 |
| `algebraic_fiber_eight_independent_intervals` | 1000 | 0.755 s | 754.74 us | root=4000 |
| `algebraic_fiber_eight_batched_intervals` | 1000 | 0.461 s | 460.90 us | root=4000 |
| `algebraic_common_fiber_degree_drop` | 1000 | 0.081 s | 80.59 us | root=1000; refinement=4000 |
| `algebraic_image_policy_zero_content` | 1000 | 0.061 s | 61.14 us | coefficient=2000 |
| `rational_quadratic_common_fiber_two_components` | 1000 | 0.763 s | 762.56 us | component=4000 |
| `implicit_quadratic_common_fiber` | 1000 | 0.456 s | 456.40 us | component=5000 |
| `implicit_quadratic_high_cofactor` | 1000 | 3.067 s | 3.07 ms | component=7000 |
| `rational_repeated_cubic_common_fiber_three_components` | 1000 | 1.900 s | 1.90 ms | component=6000 |
