# Hypersolve Algebraic Fiber Benchmarks

Generated automatically by `cargo bench --bench algebraic_fiber`. These are deterministic wall-clock throughput probes with exact result checks, not Criterion statistical estimates. Override the default iteration count with `HYPERSOLVE_FIBER_BENCH_ITERATIONS`.

| Benchmark | Iterations | Total | Mean per iteration | Validation checksum |
| --- | ---: | ---: | ---: | --- |
| `algebraic_fiber_even_multiplicity` | 1000 | 0.336 s | 335.52 us | root=1000; refinement=4000 |
| `algebraic_fiber_eight_independent_intervals` | 1000 | 2.405 s | 2.40 ms | root=4000 |
| `algebraic_fiber_eight_batched_intervals` | 1000 | 1.095 s | 1.10 ms | root=4000 |
| `algebraic_common_fiber_degree_drop` | 1000 | 0.605 s | 605.09 us | root=1000; refinement=4000 |
| `algebraic_image_policy_zero_content` | 1000 | 0.165 s | 164.87 us | coefficient=2000 |
| `rational_quadratic_common_fiber_two_components` | 1000 | 1.618 s | 1.62 ms | component=4000 |
| `implicit_quadratic_common_fiber` | 1000 | 0.905 s | 904.85 us | component=5000 |
| `implicit_quadratic_high_cofactor` | 1000 | 2.648 s | 2.65 ms | component=7000 |
| `rational_repeated_cubic_common_fiber_three_components` | 1000 | 3.488 s | 3.49 ms | component=6000 |
