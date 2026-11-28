# Hypersolve Algebraic Fiber Benchmarks

Generated automatically by `cargo bench --bench algebraic_fiber`. These are deterministic wall-clock throughput probes with exact result checks, not Criterion statistical estimates. Override the default iteration count with `HYPERSOLVE_FIBER_BENCH_ITERATIONS`.

| Benchmark | Iterations | Total | Mean per iteration | Validation checksum |
| --- | ---: | ---: | ---: | --- |
| `algebraic_fiber_even_multiplicity` | 1000 | 0.015 s | 14.88 us | root=1000; refinement=4000 |
| `algebraic_fiber_eight_independent_intervals` | 1000 | 0.084 s | 84.01 us | root=4000 |
| `algebraic_fiber_eight_batched_intervals` | 1000 | 0.045 s | 44.77 us | root=4000 |
| `algebraic_common_fiber_degree_drop` | 1000 | 0.038 s | 38.31 us | root=1000; refinement=4000 |
| `algebraic_image_policy_zero_content` | 1000 | 0.005 s | 5.35 us | coefficient=2000 |
| `rational_quadratic_common_fiber_two_components` | 1000 | 0.104 s | 103.58 us | component=4000 |
| `implicit_quadratic_common_fiber` | 1000 | 0.058 s | 57.77 us | component=5000 |
| `implicit_quadratic_high_cofactor` | 1000 | 0.174 s | 174.15 us | component=7000 |
| `rational_repeated_cubic_common_fiber_three_components` | 1000 | 0.221 s | 221.11 us | component=6000 |
