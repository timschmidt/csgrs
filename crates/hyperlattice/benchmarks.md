<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for Hyperlattice's retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 3706 -->
<!-- promoted_slow_previous_score_nanos: 3706 -->
<!-- promoted_slow_score_delta_nanos: 0 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 3.706 us |
| Delta | 0 ns |
| Delta derivative | 0 ns |

| Rank | Current Time | Fuzz target | Input |
| ---: | ---: | --- | --- |
| 1 | 4.090 us | `matrix_ops` | `seed[3199]` |
| 2 | 4.080 us | `matrix_ops` | `seed[2329]` |
| 3 | 3.980 us | `matrix_ops` | `seed[2419]` |
| 4 | 3.870 us | `matrix_ops` | `seed[3100]` |
| 5 | 3.859 us | `matrix_ops` | `seed[3641]` |
| 6 | 3.840 us | `matrix_ops` | `seed[804]` |
| 7 | 3.830 us | `matrix_ops` | `seed[3717]` |
| 8 | 3.829 us | `matrix_ops` | `seed[2946]` |
| 9 | 3.810 us | `matrix_ops` | `seed[2962]` |
| 10 | 3.809 us | `matrix_ops` | `seed[2186]` |

<!-- END promoted_slow_offender_score -->

# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
```

Run dispatch path tracing separately:

```sh
cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md
```

Refresh this file from existing Criterion estimates without rerunning the full suite:

```sh
cargo bench --bench mathbench -- --update-benchmarks-md
```

The `mathbench` suite benchmarks the Real-primary crate path and writes this file from Criterion's median estimates after a real benchmark run. The exact-dyadic column imports each finite binary64 fixture as its exact dyadic rational value; it does not perform binary64 arithmetic. The explicit-rational column constructs the corresponding authored rational inputs directly. The `numerica128` comparison column runs at 128-bit precision, `gmp_mpfr128` uses Rug's GMP/MPFR stack with 128-bit MPFR scalars, and the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated.

Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, range-reduction-heavy trigonometric arguments, and boundary-adjacent inverse trigonometric and inverse hyperbolic values.

## Operation Coverage

- Real construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.
- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.
- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Real Operations

#### Real Trigonometric And Inverse Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 219.60 ns | 219.70 ns | 753.66 ns | 761.58 ns | 2.58 us | 0.29x | 0.29x | 0.09x |
| `cos 0.1` | 210.04 ns | 212.46 ns | 491.37 ns | 485.59 ns | 2.41 us | 0.43x | 0.43x | 0.09x |
| `sin 1.23456789` | 218.94 ns | 219.24 ns | 796.45 ns | 791.33 ns | 2.54 us | 0.27x | 0.28x | 0.09x |
| `cos 1.23456789` | 211.39 ns | 210.79 ns | 580.15 ns | 578.59 ns | 2.38 us | 0.36x | 0.37x | 0.09x |
| `sin 1e6` | 358.04 ns | 356.23 ns | 1.08 us | 1.07 us | 2.77 us | 0.33x | 0.33x | 0.13x |
| `cos 1e6` | 355.55 ns | 355.82 ns | 816.73 ns | 814.00 ns | 2.57 us | 0.44x | 0.44x | 0.14x |
| `sin 1e30` | 328.82 ns | 328.56 ns | 2.86 us | 2.87 us | 4.29 us | 0.12x | 0.11x | 0.08x |
| `cos 1e30` | 330.15 ns | 327.66 ns | 971.09 ns | 963.27 ns | 3.82 us | 0.34x | 0.34x | 0.09x |
| `sin pi_7` | 218.92 ns | 756.93 ns | 734.97 ns | 738.78 ns | 2.61 us | 0.30x | 0.30x | 0.08x |
| `cos pi_7` | 210.91 ns | 640.22 ns | 533.71 ns | 527.49 ns | 2.44 us | 0.40x | 0.40x | 0.09x |
| `sin 1000pi_eps` | 198.28 ns | 1.80 us | 2.30 us | 2.26 us | 3.60 us | 0.09x | 0.09x | 0.06x |
| `cos 1000pi_eps` | 191.60 ns | 1.79 us | 582.03 ns | 577.24 ns | 2.42 us | 0.33x | 0.33x | 0.08x |
| `asin 0.5` | 35.64 ns | 34.74 ns | 3.01 us | 2.99 us | 17.25 us | 0.01x | 0.01x | 0.00x |
| `acos 0.5` | 36.51 ns | 34.86 ns | 3.02 us | 3.01 us | 17.08 us | 0.01x | 0.01x | 0.00x |
| `atanh 0.5` | 35.21 ns | 35.10 ns | 1.62 us | 1.62 us | 16.01 us | 0.02x | 0.02x | 0.00x |
| `asin neg_0.999999` | 216.35 ns | 201.51 ns | 2.52 us | 2.51 us | 16.89 us | 0.09x | 0.09x | 0.01x |
| `acos neg_0.999999` | 238.53 ns | 219.85 ns | 2.65 us | 2.65 us | 17.01 us | 0.09x | 0.09x | 0.01x |
| `atanh neg_0.999999` | 260.29 ns | 229.05 ns | 1.58 us | 1.58 us | 15.79 us | 0.17x | 0.16x | 0.02x |
| `asin 0.999999` | 185.26 ns | 185.99 ns | 2.53 us | 2.53 us | 16.83 us | 0.07x | 0.07x | 0.01x |
| `acos 0.999999` | 179.56 ns | 179.30 ns | 2.73 us | 2.73 us | 16.86 us | 0.07x | 0.07x | 0.01x |
| `atanh 0.999999` | 211.91 ns | 196.84 ns | 1.58 us | 1.58 us | 15.68 us | 0.13x | 0.13x | 0.01x |
| `asin 1e-12` | 168.70 ns | 183.22 ns | 1.41 us | 1.40 us | 19.25 us | 0.12x | 0.12x | 0.01x |
| `acos 1e-12` | 1.09 us | 1.12 us | 1.42 us | 1.42 us | 19.24 us | 0.77x | 0.77x | 0.06x |
| `atanh 1e-12` | 163.61 ns | 177.09 ns | 170.83 ns | 171.07 ns | 23.22 us | 0.96x | 0.96x | 0.01x |
| `atan 0.5` | 252.28 ns | 252.09 ns | 2.82 us | 2.72 us | 21.69 us | 0.09x | 0.09x | 0.01x |
| `asinh 0.5` | 188.82 ns | 187.13 ns | 1.58 us | 1.59 us | 10.18 us | 0.12x | 0.12x | 0.02x |
| `atan neg_1e-12` | 316.59 ns | 324.47 ns | 1.10 us | 1.04 us | 19.35 us | 0.29x | 0.30x | 0.02x |
| `asinh neg_1e-12` | 294.70 ns | 306.82 ns | 8.39 us | 8.39 us | 14.59 us | 0.04x | 0.04x | 0.02x |
| `atan 1e6` | 370.30 ns | 370.69 ns | 1.44 us | 1.39 us | 21.98 us | 0.26x | 0.27x | 0.02x |
| `asinh 1e6` | 191.33 ns | 191.11 ns | 1.58 us | 1.59 us | 9.90 us | 0.12x | 0.12x | 0.02x |
| `atan neg_1e6` | 452.23 ns | 453.31 ns | 1.44 us | 1.38 us | 21.88 us | 0.31x | 0.33x | 0.02x |
| `asinh neg_1e6` | 246.40 ns | 250.13 ns | 1.59 us | 1.59 us | 9.77 us | 0.16x | 0.15x | 0.03x |
| `acosh 9` | 156.54 ns | 157.50 ns | 1.59 us | 1.59 us | 13.67 us | 0.10x | 0.10x | 0.01x |
| `acosh 1_plus_1e-12` | 201.61 ns | 201.00 ns | 8.22 us | 8.24 us | 15.18 us | 0.02x | 0.02x | 0.01x |
| `acosh 1e6` | 156.78 ns | 156.92 ns | 1.57 us | 1.57 us | 13.67 us | 0.10x | 0.10x | 0.01x |
| `acosh e` | 146.44 ns | 146.93 ns | 1.62 us | 1.61 us | 13.60 us | 0.09x | 0.09x | 0.01x |

#### Forward Hyperbolic Construction Cases

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | 405.28 ns | 403.76 ns | 1.13 us | 1.13 us | 13.26 us | 0.36x | 0.36x | 0.03x |
| `cosh half` | 358.43 ns | 358.83 ns | 1.14 us | 1.13 us | 12.01 us | 0.32x | 0.32x | 0.03x |
| `tanh half` | 543.98 ns | 547.77 ns | 1.19 us | 1.19 us | 28.63 us | 0.46x | 0.46x | 0.02x |
| `sinh negative_tiny` | 366.23 ns | 401.74 ns | 912.40 ns | 920.69 ns | 13.71 us | 0.40x | 0.40x | 0.03x |
| `cosh negative_tiny` | 349.07 ns | 382.41 ns | 629.71 ns | 622.73 ns | 12.35 us | 0.55x | 0.56x | 0.03x |
| `tanh negative_tiny` | 514.28 ns | 552.45 ns | 840.48 ns | 830.34 ns | 28.40 us | 0.61x | 0.62x | 0.02x |
| `sinh positive_20` | 729.62 ns | 729.30 ns | 1.21 us | 1.22 us | 13.23 us | 0.60x | 0.60x | 0.06x |
| `cosh positive_20` | 819.46 ns | 817.26 ns | 1.22 us | 1.23 us | 12.00 us | 0.67x | 0.67x | 0.07x |
| `tanh positive_20` | 581.27 ns | 582.45 ns | 1.35 us | 1.29 us | 28.38 us | 0.43x | 0.45x | 0.02x |
| `sinh negative_20` | 807.45 ns | 806.85 ns | 1.22 us | 1.22 us | 13.08 us | 0.66x | 0.66x | 0.06x |
| `cosh negative_20` | 872.46 ns | 868.33 ns | 1.22 us | 1.23 us | 11.92 us | 0.71x | 0.71x | 0.07x |
| `tanh negative_20` | 646.18 ns | 649.23 ns | 1.35 us | 1.29 us | 28.30 us | 0.48x | 0.50x | 0.02x |

#### Forward Hyperbolic Explicit f64 Output Cases

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | 402.51 ns | 404.41 ns | 1.16 us | 1.15 us | - | 0.35x | 0.35x | - |
| `cosh half` | 352.65 ns | 353.77 ns | 1.16 us | 1.16 us | - | 0.30x | 0.30x | - |
| `tanh half` | 556.59 ns | 547.85 ns | 1.23 us | 1.22 us | - | 0.45x | 0.46x | - |
| `sinh negative_tiny` | 364.25 ns | 406.70 ns | 930.94 ns | 937.69 ns | - | 0.39x | 0.39x | - |
| `cosh negative_tiny` | 347.47 ns | 387.80 ns | 637.97 ns | 645.73 ns | - | 0.54x | 0.54x | - |
| `tanh negative_tiny` | 527.88 ns | 557.39 ns | 859.08 ns | 857.21 ns | - | 0.61x | 0.62x | - |
| `sinh positive_20` | 725.49 ns | 732.61 ns | 1.25 us | 1.24 us | - | 0.58x | 0.59x | - |
| `cosh positive_20` | 821.13 ns | 816.75 ns | 1.24 us | 1.24 us | - | 0.66x | 0.66x | - |
| `tanh positive_20` | 575.03 ns | 577.81 ns | 1.36 us | 1.29 us | - | 0.42x | 0.45x | - |
| `sinh negative_20` | 818.47 ns | 813.78 ns | 1.24 us | 1.23 us | - | 0.66x | 0.66x | - |
| `cosh negative_20` | 878.92 ns | 868.49 ns | 1.24 us | 1.25 us | - | 0.71x | 0.70x | - |
| `tanh negative_20` | 659.68 ns | 652.53 ns | 1.36 us | 1.29 us | - | 0.48x | 0.51x | - |

#### Real API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 11.91 ns | 11.76 ns | 15.55 ns | 8.97 ns | 0.94 ns | 0.77x | 1.33x | 12.61x |
| `one` | 12.14 ns | 12.14 ns | 30.49 ns | 21.50 ns | 29.74 ns | 0.40x | 0.56x | 0.41x |
| `e` | 22.45 ns | 22.46 ns | 1.07 us | 1.02 us | 225.24 ns | 0.02x | 0.02x | 0.10x |
| `pi` | 17.26 ns | 17.23 ns | 48.46 ns | 20.14 ns | 223.78 ns | 0.36x | 0.86x | 0.08x |
| `tau` | 17.41 ns | 17.40 ns | 98.96 ns | 68.89 ns | 2.31 us | 0.18x | 0.25x | 0.01x |
| `add` | 31.65 ns | 32.06 ns | 42.24 ns | 31.36 ns | 1.74 us | 0.75x | 1.01x | 0.02x |
| `sub` | 32.30 ns | 32.74 ns | 45.58 ns | 31.44 ns | 2.94 us | 0.71x | 1.03x | 0.01x |
| `neg` | 18.56 ns | 19.12 ns | 21.63 ns | 21.62 ns | 1.53 us | 0.86x | 0.86x | 0.01x |
| `mul` | 32.96 ns | 34.86 ns | 44.98 ns | 42.02 ns | 1.96 us | 0.73x | 0.78x | 0.02x |
| `div` | 64.41 ns | 62.49 ns | 62.62 ns | 59.91 ns | 3.04 us | 1.03x | 1.08x | 0.02x |
| `reciprocal` | 17.28 ns | 17.38 ns | 59.61 ns | 59.08 ns | 2.03 us | 0.29x | 0.29x | 0.01x |
| `reciprocal checked` | 29.06 ns | 29.29 ns | 60.35 ns | 59.42 ns | 2.02 us | 0.48x | 0.49x | 0.01x |
| `reciprocal checked abort` | 44.12 ns | 44.41 ns | 59.32 ns | 59.03 ns | 2.01 us | 0.74x | 0.75x | 0.02x |
| `pow` | 1.38 us | 1.94 us | 2.96 us | 2.79 us | 2.76 us | 0.47x | 0.50x | 0.50x |
| `powi` | 53.78 ns | 57.73 ns | 84.43 ns | 90.41 ns | 1.99 us | 0.64x | 0.59x | 0.03x |
| `exp` | 96.21 ns | 98.53 ns | 931.38 ns | 882.13 ns | 2.65 us | 0.10x | 0.11x | 0.04x |
| `exp 128` | 295.26 ns | 292.93 ns | 1.05 us | 1.03 us | 2.60 us | 0.28x | 0.29x | 0.11x |
| `ln` | 1.16 us | 375.00 ns | 1.31 us | 1.33 us | 2.55 us | 0.89x | 0.88x | 0.46x |
| `log10` | 1.41 us | 571.45 ns | 2.78 us | 3.85 us | 8.61 us | 0.51x | 0.37x | 0.16x |
| `log10 abort` | 1.43 us | 601.56 ns | 2.76 us | 3.90 us | 8.62 us | 0.52x | 0.37x | 0.17x |
| `sqrt` | 67.85 ns | 50.51 ns | 95.09 ns | 108.39 ns | 2.23 us | 0.71x | 0.63x | 0.03x |
| `sin` | 257.07 ns | 259.09 ns | 1.31 us | 1.26 us | 2.99 us | 0.20x | 0.20x | 0.09x |
| `cos` | 256.57 ns | 258.95 ns | 629.39 ns | 627.68 ns | 2.54 us | 0.41x | 0.41x | 0.10x |
| `tan` | 50.33 ns | 50.53 ns | 1.59 us | 1.58 us | 8.71 us | 0.03x | 0.03x | 0.01x |
| `sinh` | 609.96 ns | 621.67 ns | 1.13 us | 1.15 us | 13.85 us | 0.54x | 0.53x | 0.04x |
| `cosh` | 629.21 ns | 654.70 ns | 1.07 us | 1.07 us | 12.47 us | 0.59x | 0.59x | 0.05x |
| `tanh` | 595.34 ns | 618.99 ns | 1.20 us | 1.18 us | 29.02 us | 0.50x | 0.50x | 0.02x |
| `asin` | 157.00 ns | 159.67 ns | 2.46 us | 2.45 us | 18.30 us | 0.06x | 0.06x | 0.01x |
| `asin abort` | 184.88 ns | 185.17 ns | 2.45 us | 2.46 us | 18.07 us | 0.08x | 0.08x | 0.01x |
| `acos` | 409.95 ns | 413.98 ns | 2.57 us | 2.56 us | 18.15 us | 0.16x | 0.16x | 0.02x |
| `acos abort` | 435.59 ns | 441.03 ns | 2.58 us | 2.57 us | 18.39 us | 0.17x | 0.17x | 0.02x |
| `atan` | 285.78 ns | 290.56 ns | 2.29 us | 2.23 us | 23.08 us | 0.12x | 0.13x | 0.01x |
| `atan abort` | 316.30 ns | 318.57 ns | 2.30 us | 2.24 us | 23.13 us | 0.14x | 0.14x | 0.01x |
| `asinh` | 191.45 ns | 191.70 ns | 1.61 us | 1.62 us | 10.40 us | 0.12x | 0.12x | 0.02x |
| `asinh abort` | 219.74 ns | 223.48 ns | 1.61 us | 1.62 us | 10.64 us | 0.14x | 0.14x | 0.02x |
| `acosh` | 169.03 ns | 168.45 ns | 3.36 us | 3.30 us | 14.60 us | 0.05x | 0.05x | 0.01x |
| `acosh abort` | 196.53 ns | 197.81 ns | 3.33 us | 3.30 us | 14.59 us | 0.06x | 0.06x | 0.01x |
| `atanh` | 167.06 ns | 162.54 ns | 1.26 us | 1.26 us | 18.13 us | 0.13x | 0.13x | 0.01x |
| `atanh abort` | 194.17 ns | 187.65 ns | 1.28 us | 1.31 us | 18.06 us | 0.15x | 0.15x | 0.01x |
| `zero status` | 1.55 ns | 1.59 ns | 7.14 ns | 1.28 ns | 7.92 ns | 0.22x | 1.21x | 0.20x |
| `zero status abort` | - | - | 7.33 ns | 1.30 ns | 7.96 ns | - | - | - |

### Complex Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 15.84 ns | 15.82 ns | 23.78 ns | 19.92 ns | 1.87 ns | 0.67x | 0.80x | 8.46x |
| `one` | 16.27 ns | 16.29 ns | 41.43 ns | 27.59 ns | 30.60 ns | 0.39x | 0.59x | 0.53x |
| `i` | 16.16 ns | 16.20 ns | 42.44 ns | 28.17 ns | 29.79 ns | 0.38x | 0.57x | 0.54x |
| `free i` | 16.24 ns | 16.24 ns | 42.56 ns | 28.12 ns | 29.74 ns | 0.38x | 0.58x | 0.55x |
| `conjugate` | 37.57 ns | 37.57 ns | 34.13 ns | 33.88 ns | 1.55 us | 1.10x | 1.11x | 0.02x |
| `norm squared` | 81.29 ns | 94.11 ns | 122.16 ns | 97.71 ns | 5.68 us | 0.67x | 0.83x | 0.01x |
| `reciprocal` | 180.00 ns | 184.31 ns | 248.54 ns | 217.81 ns | 13.56 us | 0.72x | 0.83x | 0.01x |
| `reciprocal checked` | 180.99 ns | 183.26 ns | 247.82 ns | 217.54 ns | 13.63 us | 0.73x | 0.83x | 0.01x |
| `powi` | 643.33 ns | 655.66 ns | 1.24 us | 987.05 ns | 58.12 us | 0.52x | 0.65x | 0.01x |
| `powi checked` | 641.87 ns | 656.61 ns | 1.24 us | 986.30 ns | 58.07 us | 0.52x | 0.65x | 0.01x |
| `div checked` | 250.96 ns | 252.71 ns | 546.71 ns | 453.23 ns | 27.23 us | 0.46x | 0.55x | 0.01x |
| `div real checked` | 98.16 ns | 99.20 ns | 120.12 ns | 108.56 ns | 6.14 us | 0.82x | 0.90x | 0.02x |
| `from scalar` | 22.45 ns | 22.36 ns | 31.05 ns | 27.07 ns | 10.31 ns | 0.72x | 0.83x | 2.18x |
| `add` | 64.27 ns | 64.51 ns | 84.29 ns | 55.78 ns | 3.46 us | 0.76x | 1.15x | 0.02x |
| `sub` | 65.31 ns | 65.64 ns | 93.78 ns | 56.44 ns | 5.83 us | 0.70x | 1.16x | 0.01x |
| `neg` | 45.86 ns | 45.83 ns | 36.43 ns | 36.60 ns | 3.02 us | 1.26x | 1.25x | 0.02x |
| `mul` | 195.67 ns | 198.74 ns | 240.94 ns | 195.14 ns | 12.60 us | 0.81x | 1.00x | 0.02x |
| `div` | 251.46 ns | 259.99 ns | 548.93 ns | 455.51 ns | 27.42 us | 0.46x | 0.55x | 0.01x |
| `div real` | 121.34 ns | 122.59 ns | 118.79 ns | 109.64 ns | 6.09 us | 1.02x | 1.11x | 0.02x |

#### Cold Complex Multiplication

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `varying exact inputs` | 219.33 ns | 262.27 ns | 289.25 ns | 244.53 ns | 12.51 us | 0.76x | 0.90x | 0.02x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 231.00 ns | 169.80 ns | 251.71 ns | 201.71 ns | 9.36 us | 0.92x | 1.15x | 0.02x |
| `vec3 magnitude` | 240.13 ns | 177.79 ns | 343.28 ns | 303.42 ns | 11.63 us | 0.70x | 0.79x | 0.02x |
| `vec3 normalize` | 531.47 ns | 1.93 us | 590.85 ns | 467.38 ns | 20.85 us | 0.90x | 1.14x | 0.03x |

#### Vector API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 187.61 ns | 503.84 ns | 57.01 ns | 55.05 ns | 723.80 ns | 3.29x | 3.41x | 0.26x |
| `vec3 zero` | 45.55 ns | 45.33 ns | 29.83 ns | 26.40 ns | 2.82 ns | 1.53x | 1.73x | 16.14x |
| `vec3 dot abort` | 234.14 ns | 145.80 ns | 198.79 ns | 150.59 ns | 9.29 us | 1.18x | 1.55x | 0.03x |
| `vec3 magnitude abort` | 264.57 ns | 268.01 ns | 320.78 ns | 276.68 ns | 11.64 us | 0.82x | 0.96x | 0.02x |
| `vec3 normalize checked` | 536.73 ns | 1.38 us | 544.24 ns | 415.57 ns | 21.32 us | 0.99x | 1.29x | 0.03x |
| `vec3 normalize checked abort` | 550.32 ns | 551.54 ns | 543.38 ns | 419.36 ns | 21.33 us | 1.01x | 1.31x | 0.03x |
| `vec3 div scalar checked` | 173.39 ns | 177.03 ns | 171.57 ns | 162.68 ns | 9.19 us | 1.01x | 1.07x | 0.02x |
| `vec3 div scalar checked abort` | 199.22 ns | 206.26 ns | 172.65 ns | 162.57 ns | 9.09 us | 1.15x | 1.23x | 0.02x |
| `vec3 add` | 123.60 ns | 125.10 ns | 125.54 ns | 84.68 ns | 5.32 us | 0.98x | 1.46x | 0.02x |
| `vec3 add scalar` | 173.19 ns | 171.70 ns | 134.57 ns | 81.60 ns | 5.13 us | 1.29x | 2.12x | 0.03x |
| `vec3 sub` | 125.43 ns | 124.57 ns | 136.19 ns | 81.30 ns | 8.76 us | 0.92x | 1.54x | 0.01x |
| `vec3 sub scalar` | 216.69 ns | 224.22 ns | 124.85 ns | 82.97 ns | 8.55 us | 1.74x | 2.61x | 0.03x |
| `vec3 neg` | 87.05 ns | 85.87 ns | 50.67 ns | 51.38 ns | 4.44 us | 1.72x | 1.69x | 0.02x |
| `vec3 mul scalar` | 250.18 ns | 332.82 ns | 123.00 ns | 114.92 ns | 5.70 us | 2.03x | 2.18x | 0.04x |
| `vec3 div scalar` | 133.25 ns | 135.70 ns | 173.21 ns | 162.80 ns | 9.10 us | 0.77x | 0.82x | 0.01x |
| `vec4 dot` | 228.61 ns | 136.98 ns | 312.23 ns | 243.20 ns | 12.73 us | 0.73x | 0.94x | 0.02x |
| `vec4 magnitude` | 237.02 ns | 251.61 ns | 403.42 ns | 343.41 ns | 15.03 us | 0.59x | 0.69x | 0.02x |
| `vec4 normalize` | 456.40 ns | 998.93 ns | 692.34 ns | 533.78 ns | 27.85 us | 0.66x | 0.86x | 0.02x |
| `vec4 add` | 178.19 ns | 172.53 ns | 172.82 ns | 97.93 ns | 7.03 us | 1.03x | 1.82x | 0.03x |
| `vec4 add scalar` | 290.84 ns | 202.59 ns | 175.45 ns | 96.75 ns | 6.84 us | 1.66x | 3.01x | 0.04x |
| `vec4 sub` | 184.69 ns | 185.54 ns | 174.75 ns | 102.59 ns | 11.69 us | 1.06x | 1.80x | 0.02x |
| `vec4 sub scalar` | 316.92 ns | 337.52 ns | 167.85 ns | 96.66 ns | 11.35 us | 1.89x | 3.28x | 0.03x |
| `vec4 neg` | 106.43 ns | 107.20 ns | 63.87 ns | 66.16 ns | 5.80 us | 1.67x | 1.61x | 0.02x |
| `vec4 mul scalar` | 357.62 ns | 459.85 ns | 154.37 ns | 150.48 ns | 7.40 us | 2.32x | 2.38x | 0.05x |
| `vec4 div scalar` | 251.35 ns | 271.74 ns | 220.79 ns | 216.90 ns | 11.84 us | 1.14x | 1.16x | 0.02x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 478.34 ns | 407.09 ns | 826.96 ns | 682.18 ns | 28.52 us | 0.58x | 0.70x | 0.02x |
| `mat3 inverse` | 4.55 us | 2.05 us | 2.44 us | 2.05 us | 104.69 us | 1.86x | 2.22x | 0.04x |
| `mat3 mul mat3` | 1.41 us | 1.03 us | 2.31 us | 1.77 us | 80.40 us | 0.61x | 0.80x | 0.02x |
| `mat3 transform vec3` | 679.78 ns | 483.77 ns | 866.96 ns | 715.68 ns | 26.53 us | 0.78x | 0.95x | 0.03x |
| `mat4 determinant` | 1.04 us | 528.69 ns | 4.02 us | 3.31 us | 121.43 us | 0.26x | 0.31x | 0.01x |
| `mat4 inverse` | 7.05 us | 5.71 us | 8.94 us | 7.37 us | 433.95 us | 0.79x | 0.96x | 0.02x |
| `mat4 mul mat4` | 2.16 us | 1.77 us | 5.22 us | 4.04 us | 187.68 us | 0.41x | 0.54x | 0.01x |
| `mat4 transform vec4` | 959.77 ns | 567.99 ns | 1.61 us | 1.28 us | 47.56 us | 0.60x | 0.75x | 0.02x |

#### Matrix API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 472.73 ns | 993.45 ns | 242.09 ns | 224.38 ns | 2.10 us | 1.95x | 2.11x | 0.22x |
| `mat3 zero` | 222.74 ns | 220.52 ns | 210.79 ns | 226.27 ns | 21.03 ns | 1.06x | 0.98x | 10.59x |
| `mat3 identity` | 244.53 ns | 241.85 ns | 258.22 ns | 256.93 ns | 174.18 ns | 0.95x | 0.95x | 1.40x |
| `mat3 transpose` | 219.73 ns | 228.93 ns | 201.25 ns | 200.51 ns | 109.73 ns | 1.09x | 1.10x | 2.00x |
| `mat3 reciprocal` | 4.53 us | 2.47 us | 2.28 us | 1.88 us | 104.07 us | 1.99x | 2.41x | 0.04x |
| `mat3 reciprocal checked` | 4.50 us | 2.47 us | 2.27 us | 1.87 us | 103.69 us | 1.98x | 2.40x | 0.04x |
| `mat3 inverse checked` | 4.50 us | 2.48 us | 2.28 us | 1.87 us | 104.68 us | 1.97x | 2.41x | 0.04x |
| `mat3 inverse checked abort` | 4.67 us | 2.77 us | 2.28 us | 1.90 us | 104.40 us | 2.05x | 2.45x | 0.04x |
| `mat3 powi` | 3.96 us | 4.34 us | 6.17 us | 4.74 us | 206.07 us | 0.64x | 0.84x | 0.02x |
| `mat3 powi checked` | 3.94 us | 4.33 us | 6.19 us | 4.74 us | 204.79 us | 0.64x | 0.83x | 0.02x |
| `mat3 powi checked abort` | 3.96 us | 4.35 us | 6.17 us | 4.75 us | 204.36 us | 0.64x | 0.83x | 0.02x |
| `mat3 div scalar checked` | 518.41 ns | 641.00 ns | 823.84 ns | 766.54 ns | 26.30 us | 0.63x | 0.68x | 0.02x |
| `mat3 div scalar checked abort` | 560.87 ns | 688.04 ns | 818.67 ns | 765.63 ns | 26.35 us | 0.69x | 0.73x | 0.02x |
| `mat3 div matrix checked` | 23.15 us | 5.82 us | 4.40 us | 3.49 us | 199.50 us | 5.26x | 6.63x | 0.12x |
| `mat3 div matrix checked abort` | 23.24 us | 5.90 us | 4.39 us | 3.42 us | 200.21 us | 5.30x | 6.79x | 0.12x |
| `mat3 add` | 357.61 ns | 375.01 ns | 494.57 ns | 363.87 ns | 15.45 us | 0.72x | 0.98x | 0.02x |
| `mat3 add scalar` | 606.25 ns | 591.04 ns | 726.29 ns | 540.71 ns | 15.86 us | 0.83x | 1.12x | 0.04x |
| `mat3 sub` | 426.63 ns | 440.95 ns | 520.72 ns | 362.80 ns | 25.17 us | 0.82x | 1.18x | 0.02x |
| `mat3 sub scalar` | 793.96 ns | 712.99 ns | 718.53 ns | 544.87 ns | 25.64 us | 1.10x | 1.46x | 0.03x |
| `mat3 neg` | 219.38 ns | 219.45 ns | 475.83 ns | 453.34 ns | 12.35 us | 0.46x | 0.48x | 0.02x |
| `mat3 mul scalar` | 657.28 ns | 700.45 ns | 679.46 ns | 650.37 ns | 15.98 us | 0.97x | 1.01x | 0.04x |
| `mat3 div scalar` | 313.42 ns | 435.44 ns | 819.41 ns | 775.41 ns | 26.20 us | 0.38x | 0.40x | 0.01x |
| `mat3 div matrix` | 23.21 us | 5.74 us | 4.38 us | 3.45 us | 198.56 us | 5.30x | 6.73x | 0.12x |
| `mat3 bitxor` | 3.95 us | 4.33 us | 6.19 us | 4.74 us | 204.75 us | 0.64x | 0.83x | 0.02x |
| `mat4 zero` | 212.92 ns | 229.64 ns | 323.11 ns | 338.94 ns | 14.62 ns | 0.66x | 0.63x | 14.57x |
| `mat4 identity` | 279.09 ns | 297.73 ns | 383.12 ns | 350.32 ns | 245.92 ns | 0.73x | 0.80x | 1.13x |
| `mat4 transpose` | 242.19 ns | 225.29 ns | 335.73 ns | 343.78 ns | 162.15 ns | 0.72x | 0.70x | 1.49x |
| `mat4 reciprocal` | 7.19 us | 6.09 us | 8.80 us | 7.19 us | 437.20 us | 0.82x | 1.00x | 0.02x |
| `mat4 reciprocal checked` | 7.10 us | 5.95 us | 8.81 us | 7.15 us | 436.76 us | 0.81x | 0.99x | 0.02x |
| `mat4 powi` | 5.02 us | 6.74 us | 13.69 us | 10.85 us | 481.54 us | 0.37x | 0.46x | 0.01x |
| `mat4 powi checked` | 4.99 us | 6.78 us | 13.73 us | 10.85 us | 482.84 us | 0.36x | 0.46x | 0.01x |
| `mat4 add` | 686.55 ns | 682.21 ns | 821.00 ns | 605.64 ns | 26.21 us | 0.84x | 1.13x | 0.03x |
| `mat4 add scalar` | 915.90 ns | 968.19 ns | 1.18 us | 914.01 ns | 27.21 us | 0.78x | 1.00x | 0.03x |
| `mat4 sub` | 779.14 ns | 771.03 ns | 873.06 ns | 602.13 ns | 42.84 us | 0.89x | 1.29x | 0.02x |
| `mat4 sub scalar` | 1.25 us | 1.24 us | 1.17 us | 913.42 ns | 45.12 us | 1.06x | 1.37x | 0.03x |
| `mat4 neg` | 370.99 ns | 353.71 ns | 759.64 ns | 753.53 ns | 20.74 us | 0.49x | 0.49x | 0.02x |
| `mat4 mul scalar` | 1.06 us | 1.30 us | 1.11 us | 1.09 us | 27.04 us | 0.95x | 0.97x | 0.04x |
| `mat4 div scalar` | 638.64 ns | 931.61 ns | 1.40 us | 1.38 us | 45.35 us | 0.46x | 0.46x | 0.01x |
| `mat4 div matrix` | 36.77 us | 9.22 us | 14.17 us | 10.80 us | 677.17 us | 2.60x | 3.41x | 0.05x |
| `mat4 bitxor` | 5.21 us | 6.75 us | 13.74 us | 10.83 us | 483.47 us | 0.38x | 0.48x | 0.01x |

### Borrowed API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 18.85 ns | 19.86 ns | 44.55 ns | 34.11 ns | 1.72 us | 0.42x | 0.55x | 0.01x |
| `scalar add ref_owned` | 23.42 ns | 24.82 ns | 44.62 ns | 34.15 ns | 1.72 us | 0.52x | 0.69x | 0.01x |
| `scalar add refs` | 20.86 ns | 19.71 ns | 44.72 ns | 34.15 ns | 1.73 us | 0.47x | 0.61x | 0.01x |
| `scalar add owned_ref_with_clone` | 21.97 ns | 23.15 ns | 59.04 ns | 49.34 ns | 1.75 us | 0.37x | 0.45x | 0.01x |
| `scalar add ref_owned_with_clone` | 23.81 ns | 24.89 ns | 56.77 ns | 44.90 ns | 1.74 us | 0.42x | 0.53x | 0.01x |
| `scalar sub owned_ref` | 19.08 ns | 20.16 ns | 47.17 ns | 34.76 ns | 2.93 us | 0.40x | 0.55x | 0.01x |
| `scalar sub ref_owned` | 23.87 ns | 24.83 ns | 47.11 ns | 34.84 ns | 2.93 us | 0.51x | 0.69x | 0.01x |
| `scalar sub refs` | 21.44 ns | 19.94 ns | 47.19 ns | 34.79 ns | 2.90 us | 0.45x | 0.62x | 0.01x |
| `scalar sub owned_ref_with_clone` | 22.68 ns | 23.62 ns | 62.54 ns | 49.77 ns | 2.93 us | 0.36x | 0.46x | 0.01x |
| `scalar sub ref_owned_with_clone` | 22.67 ns | 23.45 ns | 58.96 ns | 45.53 ns | 2.95 us | 0.38x | 0.50x | 0.01x |
| `scalar mul owned_ref` | 22.52 ns | 26.01 ns | 47.63 ns | 44.97 ns | 1.99 us | 0.47x | 0.50x | 0.01x |
| `scalar mul ref_owned` | 24.48 ns | 27.01 ns | 47.84 ns | 44.98 ns | 1.98 us | 0.51x | 0.54x | 0.01x |
| `scalar mul refs` | 22.33 ns | 24.79 ns | 47.65 ns | 44.91 ns | 1.98 us | 0.47x | 0.50x | 0.01x |
| `scalar mul owned_ref_with_clone` | 26.22 ns | 29.90 ns | 62.77 ns | 60.41 ns | 1.99 us | 0.42x | 0.43x | 0.01x |
| `scalar mul ref_owned_with_clone` | 25.96 ns | 29.71 ns | 59.37 ns | 56.05 ns | 1.99 us | 0.44x | 0.46x | 0.01x |
| `scalar div owned_ref` | 54.46 ns | 52.45 ns | 66.16 ns | 62.11 ns | 2.99 us | 0.82x | 0.88x | 0.02x |
| `scalar div ref_owned` | 57.44 ns | 54.06 ns | 65.92 ns | 62.37 ns | 3.00 us | 0.87x | 0.92x | 0.02x |
| `scalar div refs` | 57.78 ns | 55.69 ns | 65.98 ns | 62.07 ns | 3.00 us | 0.88x | 0.93x | 0.02x |
| `scalar div owned_ref_with_clone` | 60.46 ns | 58.59 ns | 80.86 ns | 78.24 ns | 3.01 us | 0.75x | 0.77x | 0.02x |
| `scalar div ref_owned_with_clone` | 60.37 ns | 58.60 ns | 77.08 ns | 73.18 ns | 3.03 us | 0.78x | 0.83x | 0.02x |
| `vec3 add refs` | 75.21 ns | 74.27 ns | 124.04 ns | 84.91 ns | 5.28 us | 0.61x | 0.89x | 0.01x |
| `vec3 sub refs` | 77.85 ns | 76.40 ns | 137.02 ns | 78.48 ns | 8.79 us | 0.57x | 0.99x | 0.01x |
| `vec3 neg ref` | 50.06 ns | 49.64 ns | 49.70 ns | 49.21 ns | 4.44 us | 1.01x | 1.02x | 0.01x |
| `vec3 add_scalar_ref` | 184.28 ns | 194.56 ns | 134.63 ns | 78.68 ns | 5.13 us | 1.37x | 2.34x | 0.04x |
| `vec3 sub_scalar_ref` | 201.11 ns | 217.49 ns | 125.46 ns | 84.66 ns | 8.65 us | 1.60x | 2.38x | 0.02x |
| `vec3 mul_scalar_ref` | 113.72 ns | 117.61 ns | 116.94 ns | 113.42 ns | 5.72 us | 0.97x | 1.00x | 0.02x |
| `vec3 div_scalar_ref` | 242.62 ns | 296.25 ns | 168.03 ns | 157.78 ns | 9.04 us | 1.44x | 1.54x | 0.03x |
| `vec4 add refs` | 103.79 ns | 104.74 ns | 170.86 ns | 98.61 ns | 7.04 us | 0.61x | 1.05x | 0.01x |
| `vec4 sub refs` | 112.36 ns | 112.47 ns | 172.77 ns | 105.34 ns | 11.67 us | 0.65x | 1.07x | 0.01x |
| `vec4 neg ref` | 56.01 ns | 58.35 ns | 58.16 ns | 58.88 ns | 5.82 us | 0.96x | 0.95x | 0.01x |
| `vec4 add_scalar_ref` | 301.22 ns | 331.10 ns | 175.30 ns | 97.95 ns | 6.87 us | 1.72x | 3.08x | 0.04x |
| `vec4 sub_scalar_ref` | 303.96 ns | 323.54 ns | 168.41 ns | 97.34 ns | 11.38 us | 1.80x | 3.12x | 0.03x |
| `vec4 mul_scalar_ref` | 155.65 ns | 162.25 ns | 157.74 ns | 148.08 ns | 7.39 us | 0.99x | 1.05x | 0.02x |
| `vec4 div_scalar_ref` | 304.89 ns | 303.37 ns | 223.59 ns | 211.49 ns | 11.93 us | 1.36x | 1.44x | 0.03x |
| `mat3 add refs` | 367.64 ns | 370.94 ns | 473.78 ns | 351.32 ns | 15.43 us | 0.78x | 1.05x | 0.02x |
| `mat3 sub refs` | 415.08 ns | 429.14 ns | 503.74 ns | 349.18 ns | 25.32 us | 0.82x | 1.19x | 0.02x |
| `mat3 mul refs` | 1.24 us | 1.09 us | 2.14 us | 1.61 us | 80.10 us | 0.58x | 0.77x | 0.02x |
| `mat3 div refs` | 23.53 us | 5.93 us | 4.35 us | 3.46 us | 201.02 us | 5.41x | 6.79x | 0.12x |
| `mat3 neg ref` | 140.87 ns | 141.28 ns | 455.84 ns | 445.86 ns | 12.31 us | 0.31x | 0.32x | 0.01x |
| `mat3 add_scalar_ref` | 833.88 ns | 982.74 ns | 702.87 ns | 532.35 ns | 15.85 us | 1.19x | 1.57x | 0.05x |
| `mat3 sub_scalar_ref` | 902.09 ns | 1.05 us | 711.19 ns | 534.44 ns | 25.78 us | 1.27x | 1.69x | 0.03x |
| `mat3 mul_scalar_ref` | 575.03 ns | 693.47 ns | 662.29 ns | 638.31 ns | 15.90 us | 0.87x | 0.90x | 0.04x |
| `mat3 div_scalar_ref` | 1.06 us | 1.00 us | 810.79 ns | 794.88 ns | 26.38 us | 1.31x | 1.34x | 0.04x |
| `mat4 add refs` | 539.88 ns | 561.96 ns | 826.04 ns | 597.40 ns | 26.38 us | 0.65x | 0.90x | 0.02x |
| `mat4 sub refs` | 630.28 ns | 642.08 ns | 874.19 ns | 594.12 ns | 43.27 us | 0.72x | 1.06x | 0.01x |
| `mat4 mul refs` | 1.94 us | 2.27 us | 4.92 us | 3.76 us | 188.28 us | 0.40x | 0.52x | 0.01x |
| `mat4 div refs` | 36.09 us | 9.00 us | 14.13 us | 11.01 us | 674.31 us | 2.55x | 3.28x | 0.05x |
| `mat4 neg ref` | 241.37 ns | 233.23 ns | 731.53 ns | 749.85 ns | 20.88 us | 0.33x | 0.32x | 0.01x |
| `mat4 add_scalar_ref` | 1.27 us | 1.25 us | 1.19 us | 906.06 ns | 27.47 us | 1.07x | 1.40x | 0.05x |
| `mat4 sub_scalar_ref` | 1.23 us | 1.22 us | 1.17 us | 902.95 ns | 44.77 us | 1.05x | 1.36x | 0.03x |
| `mat4 mul_scalar_ref` | 966.39 ns | 1.11 us | 1.11 us | 1.07 us | 26.90 us | 0.87x | 0.90x | 0.04x |
| `mat4 div_scalar_ref` | 1.50 us | 1.61 us | 1.38 us | 1.34 us | 45.23 us | 1.08x | 1.12x | 0.03x |
| `mat3 transform_vec refs` | 561.02 ns | 413.65 ns | 681.82 ns | 542.94 ns | 26.43 us | 0.82x | 1.03x | 0.02x |
| `mat4 transform_vec refs` | 795.21 ns | 614.17 ns | 1.25 us | 954.05 ns | 47.54 us | 0.64x | 0.83x | 0.02x |
| `complex add refs` | 34.07 ns | 36.76 ns | 84.40 ns | 55.70 ns | 3.44 us | 0.40x | 0.61x | 0.01x |
| `complex sub refs` | 35.12 ns | 36.39 ns | 91.71 ns | 56.43 ns | 5.73 us | 0.38x | 0.62x | 0.01x |
| `complex mul refs` | 300.94 ns | 350.67 ns | 241.15 ns | 195.78 ns | 12.61 us | 1.25x | 1.54x | 0.02x |
| `complex div refs` | 227.47 ns | 235.40 ns | 536.38 ns | 457.07 ns | 26.98 us | 0.42x | 0.50x | 0.01x |
| `complex neg ref` | 29.59 ns | 29.98 ns | 35.12 ns | 34.90 ns | 3.05 us | 0.84x | 0.85x | 0.01x |
| `complex div_real_ref` | 127.65 ns | 144.26 ns | 115.30 ns | 108.83 ns | 6.14 us | 1.11x | 1.17x | 0.02x |

<!-- BEGIN COMPLETE BENCHMARK REPORT -->
## Complete generated benchmark report

Every registered benchmark target is catalogued below. Every Criterion result found under `target/criterion` is included without a name or implementation filter; non-Criterion targets write their own linked reports. Each timing binary refreshes this section after it runs.

Run the complete non-instrumented timing set with:

```sh
cargo bench
```

Regenerate this Markdown from stored Criterion data without rerunning benchmarks:

```sh
cargo run --example write_benchmarks_md
```

### Registered benchmark suites

| Target | Kind | Required features | Command | Generated report |
| --- | --- | --- | --- | --- |
| `api_dispatch_trace` | diagnostic | `hyperreal-dispatch-trace` | `cargo bench --bench api_dispatch_trace --features hyperreal-dispatch-trace` | [api_dispatch_trace.md](api_dispatch_trace.md) |
| `mathbench` | Criterion timing | `default` | `cargo bench --bench mathbench` | this file |
| `regression_sentinels` | Criterion timing | `default` | `cargo bench --bench regression_sentinels` | this file |
| `retained_fuzz` | Criterion timing | `default` | `cargo bench --bench retained_fuzz` | this file |
| `mathbench` trace mode | diagnostic | `hyperreal-dispatch-trace` | `cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md` | [dispatch_trace.md](dispatch_trace.md) |

### Comparative results

Rows sharing a Criterion group and input are compared when they expose distinct implementations. Ratios are elapsed time relative to the fastest stored row; they do not imply identical guarantees or output semantics.

No paired Criterion results are currently stored.

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `borrowed_ops/gmp_mpfr128/complex add refs` | 55.84 ns | 55.77 ns - 55.92 ns | 55.70 ns | - | - |
| `borrowed_ops/gmp_mpfr128/complex div refs` | 461.25 ns | 458.61 ns - 464.34 ns | 457.07 ns | - | - |
| `borrowed_ops/gmp_mpfr128/complex div_real_ref` | 109.13 ns | 108.94 ns - 109.36 ns | 108.83 ns | - | - |
| `borrowed_ops/gmp_mpfr128/complex mul refs` | 196.65 ns | 196.15 ns - 197.27 ns | 195.78 ns | - | - |
| `borrowed_ops/gmp_mpfr128/complex neg ref` | 34.95 ns | 34.91 ns - 34.99 ns | 34.90 ns | - | - |
| `borrowed_ops/gmp_mpfr128/complex sub refs` | 56.86 ns | 56.66 ns - 57.07 ns | 56.43 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 add refs` | 353.73 ns | 352.46 ns - 355.26 ns | 351.32 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 add_scalar_ref` | 533.71 ns | 532.80 ns - 534.74 ns | 532.35 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 div refs` | 3.48 us | 3.47 us - 3.49 us | 3.46 us | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 div_scalar_ref` | 798.89 ns | 796.23 ns - 801.93 ns | 794.88 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 mul refs` | 1.62 us | 1.62 us - 1.62 us | 1.61 us | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 mul_scalar_ref` | 645.49 ns | 642.08 ns - 649.45 ns | 638.31 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 neg ref` | 448.60 ns | 447.03 ns - 450.66 ns | 445.86 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 sub refs` | 354.44 ns | 351.77 ns - 357.88 ns | 349.18 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 sub_scalar_ref` | 539.80 ns | 537.09 ns - 543.01 ns | 534.44 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat3 transform_vec refs` | 548.46 ns | 544.99 ns - 552.60 ns | 542.94 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 add refs` | 602.70 ns | 600.11 ns - 605.57 ns | 597.40 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 add_scalar_ref` | 913.32 ns | 909.71 ns - 917.35 ns | 906.06 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 div refs` | 11.18 us | 11.10 us - 11.27 us | 11.01 us | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 div_scalar_ref` | 1.35 us | 1.35 us - 1.36 us | 1.34 us | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 mul refs` | 3.80 us | 3.78 us - 3.82 us | 3.76 us | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 mul_scalar_ref` | 1.08 us | 1.08 us - 1.09 us | 1.07 us | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 neg ref` | 758.08 ns | 753.99 ns - 762.75 ns | 749.85 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 sub refs` | 598.31 ns | 596.13 ns - 600.75 ns | 594.12 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 sub_scalar_ref` | 911.95 ns | 907.88 ns - 916.57 ns | 902.95 ns | - | - |
| `borrowed_ops/gmp_mpfr128/mat4 transform_vec refs` | 958.13 ns | 955.74 ns - 961.13 ns | 954.05 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar add owned_ref` | 34.20 ns | 34.15 ns - 34.27 ns | 34.11 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar add owned_ref_with_clone` | 49.53 ns | 49.43 ns - 49.66 ns | 49.34 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar add ref_owned` | 34.41 ns | 34.25 ns - 34.60 ns | 34.15 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar add ref_owned_with_clone` | 45.11 ns | 44.98 ns - 45.27 ns | 44.90 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar add refs` | 34.32 ns | 34.22 ns - 34.43 ns | 34.15 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar div owned_ref` | 62.24 ns | 62.16 ns - 62.35 ns | 62.11 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar div owned_ref_with_clone` | 78.47 ns | 78.31 ns - 78.67 ns | 78.24 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar div ref_owned` | 62.97 ns | 62.71 ns - 63.27 ns | 62.37 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar div ref_owned_with_clone` | 73.65 ns | 73.42 ns - 73.90 ns | 73.18 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar div refs` | 62.58 ns | 62.31 ns - 62.92 ns | 62.07 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar mul owned_ref` | 45.14 ns | 44.99 ns - 45.33 ns | 44.97 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar mul owned_ref_with_clone` | 60.52 ns | 60.44 ns - 60.61 ns | 60.41 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar mul ref_owned` | 45.27 ns | 45.15 ns - 45.41 ns | 44.98 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar mul ref_owned_with_clone` | 56.32 ns | 56.15 ns - 56.53 ns | 56.05 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar mul refs` | 45.07 ns | 44.95 ns - 45.23 ns | 44.91 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar sub owned_ref` | 34.93 ns | 34.84 ns - 35.04 ns | 34.76 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar sub owned_ref_with_clone` | 50.35 ns | 50.07 ns - 50.70 ns | 49.77 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar sub ref_owned` | 34.98 ns | 34.91 ns - 35.06 ns | 34.84 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar sub ref_owned_with_clone` | 45.87 ns | 45.69 ns - 46.07 ns | 45.53 ns | - | - |
| `borrowed_ops/gmp_mpfr128/scalar sub refs` | 34.89 ns | 34.83 ns - 34.96 ns | 34.79 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec3 add refs` | 84.94 ns | 84.85 ns - 85.02 ns | 84.91 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec3 add_scalar_ref` | 78.93 ns | 78.73 ns - 79.14 ns | 78.68 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec3 div_scalar_ref` | 158.74 ns | 158.25 ns - 159.31 ns | 157.78 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec3 mul_scalar_ref` | 113.91 ns | 113.61 ns - 114.24 ns | 113.42 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec3 neg ref` | 49.26 ns | 49.19 ns - 49.35 ns | 49.21 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec3 sub refs` | 79.04 ns | 78.69 ns - 79.45 ns | 78.48 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec3 sub_scalar_ref` | 84.66 ns | 84.52 ns - 84.81 ns | 84.66 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec4 add refs` | 99.37 ns | 98.99 ns - 99.80 ns | 98.61 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec4 add_scalar_ref` | 98.38 ns | 98.16 ns - 98.64 ns | 97.95 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec4 div_scalar_ref` | 214.84 ns | 213.24 ns - 216.94 ns | 211.49 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec4 mul_scalar_ref` | 148.52 ns | 148.18 ns - 148.92 ns | 148.08 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec4 neg ref` | 59.58 ns | 59.22 ns - 59.99 ns | 58.88 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec4 sub refs` | 106.15 ns | 105.70 ns - 106.66 ns | 105.34 ns | - | - |
| `borrowed_ops/gmp_mpfr128/vec4 sub_scalar_ref` | 100.04 ns | 98.93 ns - 101.29 ns | 97.34 ns | - | - |
| `borrowed_ops/hyperreal-rational/complex add refs` | 37.06 ns | 36.89 ns - 37.26 ns | 36.76 ns | - | - |
| `borrowed_ops/hyperreal-rational/complex div refs` | 238.54 ns | 237.00 ns - 240.29 ns | 235.40 ns | - | - |
| `borrowed_ops/hyperreal-rational/complex div_real_ref` | 146.98 ns | 145.70 ns - 148.61 ns | 144.26 ns | - | - |
| `borrowed_ops/hyperreal-rational/complex mul refs` | 357.24 ns | 354.34 ns - 360.60 ns | 350.67 ns | - | - |
| `borrowed_ops/hyperreal-rational/complex neg ref` | 30.34 ns | 30.15 ns - 30.57 ns | 29.98 ns | - | - |
| `borrowed_ops/hyperreal-rational/complex sub refs` | 36.68 ns | 36.49 ns - 36.95 ns | 36.39 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat3 add refs` | 371.34 ns | 370.92 ns - 371.82 ns | 370.94 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat3 add_scalar_ref` | 986.20 ns | 984.13 ns - 988.78 ns | 982.74 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat3 div refs` | 5.97 us | 5.95 us - 6.00 us | 5.93 us | - | - |
| `borrowed_ops/hyperreal-rational/mat3 div_scalar_ref` | 1.01 us | 1.00 us - 1.01 us | 1.00 us | - | - |
| `borrowed_ops/hyperreal-rational/mat3 mul refs` | 1.09 us | 1.09 us - 1.09 us | 1.09 us | - | - |
| `borrowed_ops/hyperreal-rational/mat3 mul_scalar_ref` | 696.82 ns | 695.03 ns - 698.94 ns | 693.47 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat3 neg ref` | 141.90 ns | 141.57 ns - 142.26 ns | 141.28 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat3 sub refs` | 431.10 ns | 429.81 ns - 432.56 ns | 429.14 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat3 sub_scalar_ref` | 1.06 us | 1.05 us - 1.06 us | 1.05 us | - | - |
| `borrowed_ops/hyperreal-rational/mat3 transform_vec refs` | 415.95 ns | 414.62 ns - 417.57 ns | 413.65 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat4 add refs` | 566.85 ns | 564.04 ns - 570.02 ns | 561.96 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat4 add_scalar_ref` | 1.26 us | 1.26 us - 1.27 us | 1.25 us | - | - |
| `borrowed_ops/hyperreal-rational/mat4 div refs` | 9.07 us | 9.04 us - 9.11 us | 9.00 us | - | - |
| `borrowed_ops/hyperreal-rational/mat4 div_scalar_ref` | 1.63 us | 1.62 us - 1.64 us | 1.61 us | - | - |
| `borrowed_ops/hyperreal-rational/mat4 mul refs` | 2.30 us | 2.28 us - 2.31 us | 2.27 us | - | - |
| `borrowed_ops/hyperreal-rational/mat4 mul_scalar_ref` | 1.12 us | 1.12 us - 1.13 us | 1.11 us | - | - |
| `borrowed_ops/hyperreal-rational/mat4 neg ref` | 233.49 ns | 232.98 ns - 234.02 ns | 233.23 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat4 sub refs` | 644.22 ns | 642.49 ns - 646.23 ns | 642.08 ns | - | - |
| `borrowed_ops/hyperreal-rational/mat4 sub_scalar_ref` | 1.22 us | 1.22 us - 1.23 us | 1.22 us | - | - |
| `borrowed_ops/hyperreal-rational/mat4 transform_vec refs` | 626.56 ns | 621.62 ns - 632.25 ns | 614.17 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar add owned_ref` | 20.17 ns | 20.05 ns - 20.31 ns | 19.86 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar add owned_ref_with_clone` | 23.19 ns | 23.14 ns - 23.24 ns | 23.15 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar add ref_owned` | 25.14 ns | 24.94 ns - 25.42 ns | 24.82 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar add ref_owned_with_clone` | 25.16 ns | 25.02 ns - 25.33 ns | 24.89 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar add refs` | 19.82 ns | 19.77 ns - 19.88 ns | 19.71 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar div owned_ref` | 52.70 ns | 52.53 ns - 52.92 ns | 52.45 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar div owned_ref_with_clone` | 58.67 ns | 58.56 ns - 58.80 ns | 58.59 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar div ref_owned` | 54.36 ns | 54.19 ns - 54.57 ns | 54.06 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar div ref_owned_with_clone` | 58.83 ns | 58.68 ns - 59.02 ns | 58.60 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar div refs` | 57.64 ns | 56.65 ns - 58.76 ns | 55.69 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar mul owned_ref` | 26.29 ns | 26.11 ns - 26.58 ns | 26.01 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar mul owned_ref_with_clone` | 30.23 ns | 30.08 ns - 30.41 ns | 29.90 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar mul ref_owned` | 27.30 ns | 27.18 ns - 27.44 ns | 27.01 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar mul ref_owned_with_clone` | 30.18 ns | 29.97 ns - 30.42 ns | 29.71 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar mul refs` | 24.86 ns | 24.81 ns - 24.91 ns | 24.79 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar sub owned_ref` | 21.01 ns | 20.28 ns - 22.38 ns | 20.16 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar sub owned_ref_with_clone` | 23.76 ns | 23.67 ns - 23.85 ns | 23.62 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar sub ref_owned` | 25.32 ns | 25.03 ns - 25.75 ns | 24.83 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar sub ref_owned_with_clone` | 23.55 ns | 23.50 ns - 23.61 ns | 23.45 ns | - | - |
| `borrowed_ops/hyperreal-rational/scalar sub refs` | 20.11 ns | 20.01 ns - 20.21 ns | 19.94 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec3 add refs` | 74.49 ns | 74.34 ns - 74.66 ns | 74.27 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec3 add_scalar_ref` | 195.59 ns | 194.98 ns - 196.30 ns | 194.56 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec3 div_scalar_ref` | 298.90 ns | 297.66 ns - 300.32 ns | 296.25 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec3 mul_scalar_ref` | 119.92 ns | 119.00 ns - 120.94 ns | 117.61 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec3 neg ref` | 50.38 ns | 50.03 ns - 50.79 ns | 49.64 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec3 sub refs` | 76.75 ns | 76.58 ns - 76.97 ns | 76.40 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec3 sub_scalar_ref` | 221.88 ns | 219.84 ns - 224.23 ns | 217.49 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec4 add refs` | 106.53 ns | 105.82 ns - 107.32 ns | 104.74 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec4 add_scalar_ref` | 333.07 ns | 331.83 ns - 334.54 ns | 331.10 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec4 div_scalar_ref` | 305.84 ns | 304.55 ns - 307.26 ns | 303.37 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec4 mul_scalar_ref` | 162.90 ns | 162.49 ns - 163.38 ns | 162.25 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec4 neg ref` | 58.73 ns | 58.52 ns - 58.98 ns | 58.35 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec4 sub refs` | 113.62 ns | 113.12 ns - 114.19 ns | 112.47 ns | - | - |
| `borrowed_ops/hyperreal-rational/vec4 sub_scalar_ref` | 326.91 ns | 325.28 ns - 328.75 ns | 323.54 ns | - | - |
| `borrowed_ops/hyperreal/complex add refs` | 34.13 ns | 34.07 ns - 34.21 ns | 34.07 ns | - | - |
| `borrowed_ops/hyperreal/complex div refs` | 228.63 ns | 227.84 ns - 229.76 ns | 227.47 ns | - | - |
| `borrowed_ops/hyperreal/complex div_real_ref` | 128.37 ns | 127.97 ns - 128.84 ns | 127.65 ns | - | - |
| `borrowed_ops/hyperreal/complex mul refs` | 301.73 ns | 301.16 ns - 302.46 ns | 300.94 ns | - | - |
| `borrowed_ops/hyperreal/complex neg ref` | 29.84 ns | 29.68 ns - 30.03 ns | 29.59 ns | - | - |
| `borrowed_ops/hyperreal/complex sub refs` | 35.16 ns | 35.12 ns - 35.22 ns | 35.12 ns | - | - |
| `borrowed_ops/hyperreal/mat3 add refs` | 370.29 ns | 368.64 ns - 372.17 ns | 367.64 ns | - | - |
| `borrowed_ops/hyperreal/mat3 add_scalar_ref` | 839.28 ns | 836.37 ns - 842.76 ns | 833.88 ns | - | - |
| `borrowed_ops/hyperreal/mat3 div refs` | 23.76 us | 23.63 us - 23.90 us | 23.53 us | - | - |
| `borrowed_ops/hyperreal/mat3 div_scalar_ref` | 1.06 us | 1.06 us - 1.06 us | 1.06 us | - | - |
| `borrowed_ops/hyperreal/mat3 mul refs` | 1.25 us | 1.24 us - 1.25 us | 1.24 us | - | - |
| `borrowed_ops/hyperreal/mat3 mul_scalar_ref` | 579.53 ns | 576.44 ns - 582.90 ns | 575.03 ns | - | - |
| `borrowed_ops/hyperreal/mat3 neg ref` | 141.92 ns | 141.47 ns - 142.40 ns | 140.87 ns | - | - |
| `borrowed_ops/hyperreal/mat3 sub refs` | 418.06 ns | 416.47 ns - 419.88 ns | 415.08 ns | - | - |
| `borrowed_ops/hyperreal/mat3 sub_scalar_ref` | 912.17 ns | 905.87 ns - 919.29 ns | 902.09 ns | - | - |
| `borrowed_ops/hyperreal/mat3 transform_vec refs` | 561.34 ns | 560.85 ns - 561.87 ns | 561.02 ns | - | - |
| `borrowed_ops/hyperreal/mat4 add refs` | 543.92 ns | 541.59 ns - 546.67 ns | 539.88 ns | - | - |
| `borrowed_ops/hyperreal/mat4 add_scalar_ref` | 1.27 us | 1.27 us - 1.27 us | 1.27 us | - | - |
| `borrowed_ops/hyperreal/mat4 div refs` | 36.30 us | 36.19 us - 36.43 us | 36.09 us | - | - |
| `borrowed_ops/hyperreal/mat4 div_scalar_ref` | 1.51 us | 1.50 us - 1.51 us | 1.50 us | - | - |
| `borrowed_ops/hyperreal/mat4 mul refs` | 1.97 us | 1.96 us - 1.98 us | 1.94 us | - | - |
| `borrowed_ops/hyperreal/mat4 mul_scalar_ref` | 972.92 ns | 968.95 ns - 978.10 ns | 966.39 ns | - | - |
| `borrowed_ops/hyperreal/mat4 neg ref` | 240.14 ns | 239.18 ns - 241.09 ns | 241.37 ns | - | - |
| `borrowed_ops/hyperreal/mat4 sub refs` | 636.84 ns | 633.36 ns - 640.85 ns | 630.28 ns | - | - |
| `borrowed_ops/hyperreal/mat4 sub_scalar_ref` | 1.23 us | 1.23 us - 1.23 us | 1.23 us | - | - |
| `borrowed_ops/hyperreal/mat4 transform_vec refs` | 799.92 ns | 797.30 ns - 803.05 ns | 795.21 ns | - | - |
| `borrowed_ops/hyperreal/scalar add owned_ref` | 19.03 ns | 18.94 ns - 19.14 ns | 18.85 ns | - | - |
| `borrowed_ops/hyperreal/scalar add owned_ref_with_clone` | 22.08 ns | 22.02 ns - 22.15 ns | 21.97 ns | - | - |
| `borrowed_ops/hyperreal/scalar add ref_owned` | 30.37 ns | 23.72 ns - 43.47 ns | 23.42 ns | - | - |
| `borrowed_ops/hyperreal/scalar add ref_owned_with_clone` | 23.96 ns | 23.89 ns - 24.05 ns | 23.81 ns | - | - |
| `borrowed_ops/hyperreal/scalar add refs` | 21.02 ns | 20.95 ns - 21.10 ns | 20.86 ns | - | - |
| `borrowed_ops/hyperreal/scalar div owned_ref` | 54.95 ns | 54.69 ns - 55.25 ns | 54.46 ns | - | - |
| `borrowed_ops/hyperreal/scalar div owned_ref_with_clone` | 61.56 ns | 61.15 ns - 62.01 ns | 60.46 ns | - | - |
| `borrowed_ops/hyperreal/scalar div ref_owned` | 58.51 ns | 58.01 ns - 59.08 ns | 57.44 ns | - | - |
| `borrowed_ops/hyperreal/scalar div ref_owned_with_clone` | 60.76 ns | 60.54 ns - 61.00 ns | 60.37 ns | - | - |
| `borrowed_ops/hyperreal/scalar div refs` | 58.15 ns | 57.93 ns - 58.39 ns | 57.78 ns | - | - |
| `borrowed_ops/hyperreal/scalar mul owned_ref` | 24.15 ns | 22.81 ns - 26.36 ns | 22.52 ns | - | - |
| `borrowed_ops/hyperreal/scalar mul owned_ref_with_clone` | 26.31 ns | 26.24 ns - 26.38 ns | 26.22 ns | - | - |
| `borrowed_ops/hyperreal/scalar mul ref_owned` | 24.89 ns | 24.69 ns - 25.12 ns | 24.48 ns | - | - |
| `borrowed_ops/hyperreal/scalar mul ref_owned_with_clone` | 26.45 ns | 26.25 ns - 26.68 ns | 25.96 ns | - | - |
| `borrowed_ops/hyperreal/scalar mul refs` | 22.85 ns | 22.58 ns - 23.18 ns | 22.33 ns | - | - |
| `borrowed_ops/hyperreal/scalar sub owned_ref` | 20.83 ns | 19.41 ns - 23.47 ns | 19.08 ns | - | - |
| `borrowed_ops/hyperreal/scalar sub owned_ref_with_clone` | 22.80 ns | 22.73 ns - 22.89 ns | 22.68 ns | - | - |
| `borrowed_ops/hyperreal/scalar sub ref_owned` | 24.15 ns | 24.01 ns - 24.31 ns | 23.87 ns | - | - |
| `borrowed_ops/hyperreal/scalar sub ref_owned_with_clone` | 22.71 ns | 22.67 ns - 22.75 ns | 22.67 ns | - | - |
| `borrowed_ops/hyperreal/scalar sub refs` | 21.57 ns | 21.48 ns - 21.67 ns | 21.44 ns | - | - |
| `borrowed_ops/hyperreal/vec3 add refs` | 75.39 ns | 75.25 ns - 75.57 ns | 75.21 ns | - | - |
| `borrowed_ops/hyperreal/vec3 add_scalar_ref` | 185.45 ns | 184.85 ns - 186.16 ns | 184.28 ns | - | - |
| `borrowed_ops/hyperreal/vec3 div_scalar_ref` | 244.72 ns | 243.72 ns - 245.83 ns | 242.62 ns | - | - |
| `borrowed_ops/hyperreal/vec3 mul_scalar_ref` | 114.36 ns | 114.00 ns - 114.80 ns | 113.72 ns | - | - |
| `borrowed_ops/hyperreal/vec3 neg ref` | 50.85 ns | 50.56 ns - 51.16 ns | 50.06 ns | - | - |
| `borrowed_ops/hyperreal/vec3 sub refs` | 78.74 ns | 78.31 ns - 79.25 ns | 77.85 ns | - | - |
| `borrowed_ops/hyperreal/vec3 sub_scalar_ref` | 203.01 ns | 202.26 ns - 203.85 ns | 201.11 ns | - | - |
| `borrowed_ops/hyperreal/vec4 add refs` | 105.28 ns | 104.54 ns - 106.08 ns | 103.79 ns | - | - |
| `borrowed_ops/hyperreal/vec4 add_scalar_ref` | 304.31 ns | 302.95 ns - 305.82 ns | 301.22 ns | - | - |
| `borrowed_ops/hyperreal/vec4 div_scalar_ref` | 305.79 ns | 305.33 ns - 306.33 ns | 304.89 ns | - | - |
| `borrowed_ops/hyperreal/vec4 mul_scalar_ref` | 156.55 ns | 155.89 ns - 157.39 ns | 155.65 ns | - | - |
| `borrowed_ops/hyperreal/vec4 neg ref` | 56.77 ns | 56.43 ns - 57.15 ns | 56.01 ns | - | - |
| `borrowed_ops/hyperreal/vec4 sub refs` | 114.34 ns | 113.49 ns - 115.29 ns | 112.36 ns | - | - |
| `borrowed_ops/hyperreal/vec4 sub_scalar_ref` | 306.40 ns | 305.34 ns - 307.59 ns | 303.96 ns | - | - |
| `borrowed_ops/numerica128/complex add refs` | 85.29 ns | 84.86 ns - 85.78 ns | 84.40 ns | - | - |
| `borrowed_ops/numerica128/complex div refs` | 546.54 ns | 542.61 ns - 550.97 ns | 536.38 ns | - | - |
| `borrowed_ops/numerica128/complex div_real_ref` | 117.19 ns | 116.48 ns - 117.96 ns | 115.30 ns | - | - |
| `borrowed_ops/numerica128/complex mul refs` | 241.38 ns | 241.24 ns - 241.54 ns | 241.15 ns | - | - |
| `borrowed_ops/numerica128/complex neg ref` | 35.52 ns | 35.31 ns - 35.76 ns | 35.12 ns | - | - |
| `borrowed_ops/numerica128/complex sub refs` | 92.24 ns | 91.96 ns - 92.56 ns | 91.71 ns | - | - |
| `borrowed_ops/numerica128/mat3 add refs` | 476.42 ns | 474.10 ns - 480.49 ns | 473.78 ns | - | - |
| `borrowed_ops/numerica128/mat3 add_scalar_ref` | 709.88 ns | 706.12 ns - 714.56 ns | 702.87 ns | - | - |
| `borrowed_ops/numerica128/mat3 div refs` | 4.37 us | 4.36 us - 4.39 us | 4.35 us | - | - |
| `borrowed_ops/numerica128/mat3 div_scalar_ref` | 821.07 ns | 816.17 ns - 827.05 ns | 810.79 ns | - | - |
| `borrowed_ops/numerica128/mat3 mul refs` | 2.17 us | 2.16 us - 2.18 us | 2.14 us | - | - |
| `borrowed_ops/numerica128/mat3 mul_scalar_ref` | 667.84 ns | 665.50 ns - 670.47 ns | 662.29 ns | - | - |
| `borrowed_ops/numerica128/mat3 neg ref` | 460.37 ns | 458.13 ns - 462.86 ns | 455.84 ns | - | - |
| `borrowed_ops/numerica128/mat3 sub refs` | 504.16 ns | 503.63 ns - 504.74 ns | 503.74 ns | - | - |
| `borrowed_ops/numerica128/mat3 sub_scalar_ref` | 717.32 ns | 713.75 ns - 721.08 ns | 711.19 ns | - | - |
| `borrowed_ops/numerica128/mat3 transform_vec refs` | 687.05 ns | 684.06 ns - 690.43 ns | 681.82 ns | - | - |
| `borrowed_ops/numerica128/mat4 add refs` | 837.13 ns | 832.23 ns - 842.58 ns | 826.04 ns | - | - |
| `borrowed_ops/numerica128/mat4 add_scalar_ref` | 1.20 us | 1.19 us - 1.21 us | 1.19 us | - | - |
| `borrowed_ops/numerica128/mat4 div refs` | 14.33 us | 14.24 us - 14.42 us | 14.13 us | - | - |
| `borrowed_ops/numerica128/mat4 div_scalar_ref` | 1.39 us | 1.39 us - 1.40 us | 1.38 us | - | - |
| `borrowed_ops/numerica128/mat4 mul refs` | 4.98 us | 4.96 us - 5.01 us | 4.92 us | - | - |
| `borrowed_ops/numerica128/mat4 mul_scalar_ref` | 1.11 us | 1.11 us - 1.11 us | 1.11 us | - | - |
| `borrowed_ops/numerica128/mat4 neg ref` | 742.09 ns | 737.57 ns - 747.32 ns | 731.53 ns | - | - |
| `borrowed_ops/numerica128/mat4 sub refs` | 878.79 ns | 875.78 ns - 882.22 ns | 874.19 ns | - | - |
| `borrowed_ops/numerica128/mat4 sub_scalar_ref` | 1.18 us | 1.17 us - 1.18 us | 1.17 us | - | - |
| `borrowed_ops/numerica128/mat4 transform_vec refs` | 1.25 us | 1.25 us - 1.26 us | 1.25 us | - | - |
| `borrowed_ops/numerica128/scalar add owned_ref` | 44.98 ns | 44.81 ns - 45.18 ns | 44.55 ns | - | - |
| `borrowed_ops/numerica128/scalar add owned_ref_with_clone` | 59.97 ns | 59.58 ns - 60.43 ns | 59.04 ns | - | - |
| `borrowed_ops/numerica128/scalar add ref_owned` | 44.77 ns | 44.69 ns - 44.86 ns | 44.62 ns | - | - |
| `borrowed_ops/numerica128/scalar add ref_owned_with_clone` | 57.54 ns | 57.12 ns - 58.02 ns | 56.77 ns | - | - |
| `borrowed_ops/numerica128/scalar add refs` | 45.16 ns | 44.96 ns - 45.39 ns | 44.72 ns | - | - |
| `borrowed_ops/numerica128/scalar div owned_ref` | 66.63 ns | 66.35 ns - 66.95 ns | 66.16 ns | - | - |
| `borrowed_ops/numerica128/scalar div owned_ref_with_clone` | 82.26 ns | 81.65 ns - 82.95 ns | 80.86 ns | - | - |
| `borrowed_ops/numerica128/scalar div ref_owned` | 66.24 ns | 66.05 ns - 66.47 ns | 65.92 ns | - | - |
| `borrowed_ops/numerica128/scalar div ref_owned_with_clone` | 77.58 ns | 77.31 ns - 77.89 ns | 77.08 ns | - | - |
| `borrowed_ops/numerica128/scalar div refs` | 66.25 ns | 66.13 ns - 66.40 ns | 65.98 ns | - | - |
| `borrowed_ops/numerica128/scalar mul owned_ref` | 47.72 ns | 47.64 ns - 47.83 ns | 47.63 ns | - | - |
| `borrowed_ops/numerica128/scalar mul owned_ref_with_clone` | 63.20 ns | 62.97 ns - 63.48 ns | 62.77 ns | - | - |
| `borrowed_ops/numerica128/scalar mul ref_owned` | 48.30 ns | 48.11 ns - 48.52 ns | 47.84 ns | - | - |
| `borrowed_ops/numerica128/scalar mul ref_owned_with_clone` | 59.68 ns | 59.54 ns - 59.84 ns | 59.37 ns | - | - |
| `borrowed_ops/numerica128/scalar mul refs` | 47.75 ns | 47.68 ns - 47.84 ns | 47.65 ns | - | - |
| `borrowed_ops/numerica128/scalar sub owned_ref` | 47.46 ns | 47.30 ns - 47.66 ns | 47.17 ns | - | - |
| `borrowed_ops/numerica128/scalar sub owned_ref_with_clone` | 62.94 ns | 62.76 ns - 63.13 ns | 62.54 ns | - | - |
| `borrowed_ops/numerica128/scalar sub ref_owned` | 47.33 ns | 47.20 ns - 47.49 ns | 47.11 ns | - | - |
| `borrowed_ops/numerica128/scalar sub ref_owned_with_clone` | 59.70 ns | 59.40 ns - 60.04 ns | 58.96 ns | - | - |
| `borrowed_ops/numerica128/scalar sub refs` | 47.45 ns | 47.31 ns - 47.63 ns | 47.19 ns | - | - |
| `borrowed_ops/numerica128/vec3 add refs` | 125.22 ns | 124.65 ns - 125.85 ns | 124.04 ns | - | - |
| `borrowed_ops/numerica128/vec3 add_scalar_ref` | 135.09 ns | 134.78 ns - 135.46 ns | 134.63 ns | - | - |
| `borrowed_ops/numerica128/vec3 div_scalar_ref` | 168.90 ns | 168.36 ns - 169.50 ns | 168.03 ns | - | - |
| `borrowed_ops/numerica128/vec3 mul_scalar_ref` | 116.99 ns | 116.85 ns - 117.15 ns | 116.94 ns | - | - |
| `borrowed_ops/numerica128/vec3 neg ref` | 49.91 ns | 49.73 ns - 50.15 ns | 49.70 ns | - | - |
| `borrowed_ops/numerica128/vec3 sub refs` | 138.79 ns | 138.06 ns - 139.60 ns | 137.02 ns | - | - |
| `borrowed_ops/numerica128/vec3 sub_scalar_ref` | 125.86 ns | 125.58 ns - 126.18 ns | 125.46 ns | - | - |
| `borrowed_ops/numerica128/vec4 add refs` | 171.33 ns | 170.98 ns - 171.78 ns | 170.86 ns | - | - |
| `borrowed_ops/numerica128/vec4 add_scalar_ref` | 175.62 ns | 175.35 ns - 175.92 ns | 175.30 ns | - | - |
| `borrowed_ops/numerica128/vec4 div_scalar_ref` | 224.33 ns | 223.86 ns - 224.86 ns | 223.59 ns | - | - |
| `borrowed_ops/numerica128/vec4 mul_scalar_ref` | 158.16 ns | 157.84 ns - 158.54 ns | 157.74 ns | - | - |
| `borrowed_ops/numerica128/vec4 neg ref` | 58.29 ns | 58.19 ns - 58.41 ns | 58.16 ns | - | - |
| `borrowed_ops/numerica128/vec4 sub refs` | 174.08 ns | 173.49 ns - 174.76 ns | 172.77 ns | - | - |
| `borrowed_ops/numerica128/vec4 sub_scalar_ref` | 169.32 ns | 168.82 ns - 169.89 ns | 168.41 ns | - | - |
| `borrowed_ops/symbolica/complex add refs` | 3.47 us | 3.45 us - 3.49 us | 3.44 us | - | - |
| `borrowed_ops/symbolica/complex div refs` | 27.18 us | 27.07 us - 27.30 us | 26.98 us | - | - |
| `borrowed_ops/symbolica/complex div_real_ref` | 6.16 us | 6.14 us - 6.19 us | 6.14 us | - | - |
| `borrowed_ops/symbolica/complex mul refs` | 12.69 us | 12.65 us - 12.75 us | 12.61 us | - | - |
| `borrowed_ops/symbolica/complex neg ref` | 3.07 us | 3.05 us - 3.08 us | 3.05 us | - | - |
| `borrowed_ops/symbolica/complex sub refs` | 5.77 us | 5.75 us - 5.79 us | 5.73 us | - | - |
| `borrowed_ops/symbolica/mat3 add refs` | 15.49 us | 15.44 us - 15.55 us | 15.43 us | - | - |
| `borrowed_ops/symbolica/mat3 add_scalar_ref` | 15.91 us | 15.87 us - 15.96 us | 15.85 us | - | - |
| `borrowed_ops/symbolica/mat3 div refs` | 202.43 us | 201.80 us - 203.12 us | 201.02 us | - | - |
| `borrowed_ops/symbolica/mat3 div_scalar_ref` | 26.76 us | 26.60 us - 26.94 us | 26.38 us | - | - |
| `borrowed_ops/symbolica/mat3 mul refs` | 80.31 us | 80.17 us - 80.48 us | 80.10 us | - | - |
| `borrowed_ops/symbolica/mat3 mul_scalar_ref` | 15.93 us | 15.91 us - 15.95 us | 15.90 us | - | - |
| `borrowed_ops/symbolica/mat3 neg ref` | 12.33 us | 12.30 us - 12.36 us | 12.31 us | - | - |
| `borrowed_ops/symbolica/mat3 sub refs` | 25.45 us | 25.40 us - 25.51 us | 25.32 us | - | - |
| `borrowed_ops/symbolica/mat3 sub_scalar_ref` | 25.97 us | 25.88 us - 26.07 us | 25.78 us | - | - |
| `borrowed_ops/symbolica/mat3 transform_vec refs` | 26.74 us | 26.57 us - 26.95 us | 26.43 us | - | - |
| `borrowed_ops/symbolica/mat4 add refs` | 26.72 us | 26.55 us - 26.92 us | 26.38 us | - | - |
| `borrowed_ops/symbolica/mat4 add_scalar_ref` | 27.85 us | 27.67 us - 28.06 us | 27.47 us | - | - |
| `borrowed_ops/symbolica/mat4 div refs` | 676.67 us | 673.68 us - 680.04 us | 674.31 us | - | - |
| `borrowed_ops/symbolica/mat4 div_scalar_ref` | 45.28 us | 45.24 us - 45.33 us | 45.23 us | - | - |
| `borrowed_ops/symbolica/mat4 mul refs` | 188.61 us | 188.25 us - 188.98 us | 188.28 us | - | - |
| `borrowed_ops/symbolica/mat4 mul_scalar_ref` | 27.03 us | 26.95 us - 27.13 us | 26.90 us | - | - |
| `borrowed_ops/symbolica/mat4 neg ref` | 20.98 us | 20.92 us - 21.06 us | 20.88 us | - | - |
| `borrowed_ops/symbolica/mat4 sub refs` | 43.38 us | 43.25 us - 43.54 us | 43.27 us | - | - |
| `borrowed_ops/symbolica/mat4 sub_scalar_ref` | 44.99 us | 44.83 us - 45.20 us | 44.77 us | - | - |
| `borrowed_ops/symbolica/mat4 transform_vec refs` | 47.87 us | 47.61 us - 48.15 us | 47.54 us | - | - |
| `borrowed_ops/symbolica/scalar add owned_ref` | 1.73 us | 1.73 us - 1.74 us | 1.72 us | - | - |
| `borrowed_ops/symbolica/scalar add owned_ref_with_clone` | 1.75 us | 1.75 us - 1.76 us | 1.75 us | - | - |
| `borrowed_ops/symbolica/scalar add ref_owned` | 1.73 us | 1.73 us - 1.73 us | 1.72 us | - | - |
| `borrowed_ops/symbolica/scalar add ref_owned_with_clone` | 1.76 us | 1.75 us - 1.78 us | 1.74 us | - | - |
| `borrowed_ops/symbolica/scalar add refs` | 1.74 us | 1.74 us - 1.75 us | 1.73 us | - | - |
| `borrowed_ops/symbolica/scalar div owned_ref` | 3.01 us | 3.00 us - 3.03 us | 2.99 us | - | - |
| `borrowed_ops/symbolica/scalar div owned_ref_with_clone` | 3.01 us | 3.01 us - 3.02 us | 3.01 us | - | - |
| `borrowed_ops/symbolica/scalar div ref_owned` | 3.04 us | 3.02 us - 3.06 us | 3.00 us | - | - |
| `borrowed_ops/symbolica/scalar div ref_owned_with_clone` | 3.04 us | 3.03 us - 3.05 us | 3.03 us | - | - |
| `borrowed_ops/symbolica/scalar div refs` | 3.04 us | 3.02 us - 3.06 us | 3.00 us | - | - |
| `borrowed_ops/symbolica/scalar mul owned_ref` | 2.01 us | 2.00 us - 2.02 us | 1.99 us | - | - |
| `borrowed_ops/symbolica/scalar mul owned_ref_with_clone` | 2.01 us | 2.00 us - 2.02 us | 1.99 us | - | - |
| `borrowed_ops/symbolica/scalar mul ref_owned` | 1.99 us | 1.98 us - 2.00 us | 1.98 us | - | - |
| `borrowed_ops/symbolica/scalar mul ref_owned_with_clone` | 2.01 us | 2.00 us - 2.02 us | 1.99 us | - | - |
| `borrowed_ops/symbolica/scalar mul refs` | 1.99 us | 1.98 us - 2.00 us | 1.98 us | - | - |
| `borrowed_ops/symbolica/scalar sub owned_ref` | 2.94 us | 2.93 us - 2.95 us | 2.93 us | - | - |
| `borrowed_ops/symbolica/scalar sub owned_ref_with_clone` | 2.95 us | 2.94 us - 2.96 us | 2.93 us | - | - |
| `borrowed_ops/symbolica/scalar sub ref_owned` | 2.96 us | 2.94 us - 2.98 us | 2.93 us | - | - |
| `borrowed_ops/symbolica/scalar sub ref_owned_with_clone` | 2.97 us | 2.96 us - 2.99 us | 2.95 us | - | - |
| `borrowed_ops/symbolica/scalar sub refs` | 2.92 us | 2.91 us - 2.94 us | 2.90 us | - | - |
| `borrowed_ops/symbolica/vec3 add refs` | 5.32 us | 5.30 us - 5.35 us | 5.28 us | - | - |
| `borrowed_ops/symbolica/vec3 add_scalar_ref` | 5.18 us | 5.15 us - 5.20 us | 5.13 us | - | - |
| `borrowed_ops/symbolica/vec3 div_scalar_ref` | 9.09 us | 9.06 us - 9.13 us | 9.04 us | - | - |
| `borrowed_ops/symbolica/vec3 mul_scalar_ref` | 5.75 us | 5.73 us - 5.78 us | 5.72 us | - | - |
| `borrowed_ops/symbolica/vec3 neg ref` | 4.48 us | 4.46 us - 4.50 us | 4.44 us | - | - |
| `borrowed_ops/symbolica/vec3 sub refs` | 8.87 us | 8.83 us - 8.92 us | 8.79 us | - | - |
| `borrowed_ops/symbolica/vec3 sub_scalar_ref` | 8.73 us | 8.68 us - 8.79 us | 8.65 us | - | - |
| `borrowed_ops/symbolica/vec4 add refs` | 7.07 us | 7.06 us - 7.09 us | 7.04 us | - | - |
| `borrowed_ops/symbolica/vec4 add_scalar_ref` | 6.91 us | 6.89 us - 6.94 us | 6.87 us | - | - |
| `borrowed_ops/symbolica/vec4 div_scalar_ref` | 11.99 us | 11.96 us - 12.03 us | 11.93 us | - | - |
| `borrowed_ops/symbolica/vec4 mul_scalar_ref` | 7.49 us | 7.44 us - 7.54 us | 7.39 us | - | - |
| `borrowed_ops/symbolica/vec4 neg ref` | 5.86 us | 5.84 us - 5.88 us | 5.82 us | - | - |
| `borrowed_ops/symbolica/vec4 sub refs` | 11.81 us | 11.74 us - 11.89 us | 11.67 us | - | - |
| `borrowed_ops/symbolica/vec4 sub_scalar_ref` | 11.42 us | 11.40 us - 11.45 us | 11.38 us | - | - |
| `complex_mul_cold/gmp_mpfr128/varying` | 245.51 ns | 243.03 ns - 248.07 ns | 244.53 ns | - | - |
| `complex_mul_cold/hyperreal-rational/varying` | 264.11 ns | 263.10 ns - 265.23 ns | 262.27 ns | - | - |
| `complex_mul_cold/hyperreal/varying` | 221.10 ns | 220.15 ns - 222.20 ns | 219.33 ns | - | - |
| `complex_mul_cold/numerica128/varying` | 288.66 ns | 287.32 ns - 289.83 ns | 289.25 ns | - | - |
| `complex_mul_cold/symbolica/varying` | 12.55 us | 12.51 us - 12.60 us | 12.51 us | - | - |
| `complex_ops/gmp_mpfr128/add` | 56.72 ns | 56.24 ns - 57.27 ns | 55.78 ns | - | - |
| `complex_ops/gmp_mpfr128/conjugate` | 33.92 ns | 33.88 ns - 33.97 ns | 33.88 ns | - | - |
| `complex_ops/gmp_mpfr128/div` | 457.80 ns | 456.52 ns - 459.33 ns | 455.51 ns | - | - |
| `complex_ops/gmp_mpfr128/div_checked` | 454.18 ns | 453.43 ns - 455.09 ns | 453.23 ns | - | - |
| `complex_ops/gmp_mpfr128/div_real` | 110.73 ns | 110.20 ns - 111.32 ns | 109.64 ns | - | - |
| `complex_ops/gmp_mpfr128/div_real_checked` | 108.56 ns | 108.50 ns - 108.62 ns | 108.56 ns | - | - |
| `complex_ops/gmp_mpfr128/free_i` | 28.13 ns | 28.11 ns - 28.16 ns | 28.12 ns | - | - |
| `complex_ops/gmp_mpfr128/from_scalar` | 27.14 ns | 27.09 ns - 27.20 ns | 27.07 ns | - | - |
| `complex_ops/gmp_mpfr128/i` | 28.27 ns | 28.20 ns - 28.34 ns | 28.17 ns | - | - |
| `complex_ops/gmp_mpfr128/mul` | 196.44 ns | 195.74 ns - 197.23 ns | 195.14 ns | - | - |
| `complex_ops/gmp_mpfr128/neg` | 37.01 ns | 36.81 ns - 37.24 ns | 36.60 ns | - | - |
| `complex_ops/gmp_mpfr128/norm_squared` | 97.89 ns | 97.73 ns - 98.10 ns | 97.71 ns | - | - |
| `complex_ops/gmp_mpfr128/one` | 27.93 ns | 27.73 ns - 28.18 ns | 27.59 ns | - | - |
| `complex_ops/gmp_mpfr128/powi` | 989.63 ns | 987.24 ns - 992.53 ns | 987.05 ns | - | - |
| `complex_ops/gmp_mpfr128/powi_checked` | 987.19 ns | 986.53 ns - 987.94 ns | 986.30 ns | - | - |
| `complex_ops/gmp_mpfr128/powi_checked_negative_one` | 453.12 ns | 452.79 ns - 453.47 ns | 452.77 ns | - | - |
| `complex_ops/gmp_mpfr128/powi_negative_one` | 455.44 ns | 454.02 ns - 457.05 ns | 452.84 ns | - | - |
| `complex_ops/gmp_mpfr128/reciprocal` | 218.65 ns | 218.13 ns - 219.25 ns | 217.81 ns | - | - |
| `complex_ops/gmp_mpfr128/reciprocal_checked` | 217.80 ns | 217.58 ns - 218.05 ns | 217.54 ns | - | - |
| `complex_ops/gmp_mpfr128/sub` | 56.88 ns | 56.69 ns - 57.08 ns | 56.44 ns | - | - |
| `complex_ops/gmp_mpfr128/zero` | 20.03 ns | 19.96 ns - 20.14 ns | 19.92 ns | - | - |
| `complex_ops/hyperreal-rational/add` | 64.84 ns | 64.63 ns - 65.09 ns | 64.51 ns | - | - |
| `complex_ops/hyperreal-rational/conjugate` | 37.61 ns | 37.58 ns - 37.65 ns | 37.57 ns | - | - |
| `complex_ops/hyperreal-rational/div` | 260.80 ns | 260.25 ns - 261.41 ns | 259.99 ns | - | - |
| `complex_ops/hyperreal-rational/div_checked` | 253.89 ns | 253.33 ns - 254.53 ns | 252.71 ns | - | - |
| `complex_ops/hyperreal-rational/div_real` | 122.98 ns | 122.70 ns - 123.32 ns | 122.59 ns | - | - |
| `complex_ops/hyperreal-rational/div_real_checked` | 99.61 ns | 99.38 ns - 99.86 ns | 99.20 ns | - | - |
| `complex_ops/hyperreal-rational/free_i` | 16.26 ns | 16.24 ns - 16.29 ns | 16.24 ns | - | - |
| `complex_ops/hyperreal-rational/from_scalar` | 22.39 ns | 22.36 ns - 22.44 ns | 22.36 ns | - | - |
| `complex_ops/hyperreal-rational/i` | 16.21 ns | 16.19 ns - 16.23 ns | 16.20 ns | - | - |
| `complex_ops/hyperreal-rational/mul` | 200.27 ns | 199.53 ns - 201.17 ns | 198.74 ns | - | - |
| `complex_ops/hyperreal-rational/neg` | 46.05 ns | 45.91 ns - 46.22 ns | 45.83 ns | - | - |
| `complex_ops/hyperreal-rational/norm_squared` | 94.31 ns | 94.14 ns - 94.53 ns | 94.11 ns | - | - |
| `complex_ops/hyperreal-rational/one` | 16.33 ns | 16.30 ns - 16.38 ns | 16.29 ns | - | - |
| `complex_ops/hyperreal-rational/powi` | 658.58 ns | 656.77 ns - 660.79 ns | 655.66 ns | - | - |
| `complex_ops/hyperreal-rational/powi_checked` | 657.19 ns | 655.94 ns - 658.70 ns | 656.61 ns | - | - |
| `complex_ops/hyperreal-rational/powi_checked_negative_one` | 205.52 ns | 204.64 ns - 206.51 ns | 203.86 ns | - | - |
| `complex_ops/hyperreal-rational/powi_negative_one` | 186.44 ns | 185.91 ns - 187.02 ns | 185.19 ns | - | - |
| `complex_ops/hyperreal-rational/reciprocal` | 185.54 ns | 184.88 ns - 186.30 ns | 184.31 ns | - | - |
| `complex_ops/hyperreal-rational/reciprocal_checked` | 184.03 ns | 183.58 ns - 184.60 ns | 183.26 ns | - | - |
| `complex_ops/hyperreal-rational/sub` | 66.09 ns | 65.84 ns - 66.39 ns | 65.64 ns | - | - |
| `complex_ops/hyperreal-rational/zero` | 15.82 ns | 15.80 ns - 15.85 ns | 15.82 ns | - | - |
| `complex_ops/hyperreal/add` | 64.76 ns | 64.55 ns - 64.99 ns | 64.27 ns | - | - |
| `complex_ops/hyperreal/conjugate` | 37.64 ns | 37.59 ns - 37.71 ns | 37.57 ns | - | - |
| `complex_ops/hyperreal/div` | 251.96 ns | 251.68 ns - 252.28 ns | 251.46 ns | - | - |
| `complex_ops/hyperreal/div_checked` | 252.57 ns | 251.77 ns - 253.51 ns | 250.96 ns | - | - |
| `complex_ops/hyperreal/div_real` | 121.55 ns | 121.39 ns - 121.74 ns | 121.34 ns | - | - |
| `complex_ops/hyperreal/div_real_checked` | 98.93 ns | 98.55 ns - 99.34 ns | 98.16 ns | - | - |
| `complex_ops/hyperreal/free_i` | 16.25 ns | 16.23 ns - 16.26 ns | 16.24 ns | - | - |
| `complex_ops/hyperreal/from_scalar` | 22.51 ns | 22.47 ns - 22.56 ns | 22.45 ns | - | - |
| `complex_ops/hyperreal/i` | 16.17 ns | 16.15 ns - 16.18 ns | 16.16 ns | - | - |
| `complex_ops/hyperreal/mul` | 195.83 ns | 195.63 ns - 196.04 ns | 195.67 ns | - | - |
| `complex_ops/hyperreal/neg` | 46.02 ns | 45.94 ns - 46.11 ns | 45.86 ns | - | - |
| `complex_ops/hyperreal/norm_squared` | 81.83 ns | 81.46 ns - 82.32 ns | 81.29 ns | - | - |
| `complex_ops/hyperreal/one` | 16.35 ns | 16.30 ns - 16.40 ns | 16.27 ns | - | - |
| `complex_ops/hyperreal/powi` | 650.72 ns | 646.98 ns - 655.14 ns | 643.33 ns | - | - |
| `complex_ops/hyperreal/powi_checked` | 643.40 ns | 642.37 ns - 644.61 ns | 641.87 ns | - | - |
| `complex_ops/hyperreal/powi_checked_negative_one` | 201.96 ns | 200.76 ns - 203.42 ns | 199.66 ns | - | - |
| `complex_ops/hyperreal/powi_negative_one` | 183.66 ns | 183.12 ns - 184.32 ns | 183.06 ns | - | - |
| `complex_ops/hyperreal/reciprocal` | 180.65 ns | 180.18 ns - 181.19 ns | 180.00 ns | - | - |
| `complex_ops/hyperreal/reciprocal_checked` | 181.48 ns | 181.08 ns - 181.96 ns | 180.99 ns | - | - |
| `complex_ops/hyperreal/sub` | 65.59 ns | 65.42 ns - 65.79 ns | 65.31 ns | - | - |
| `complex_ops/hyperreal/zero` | 16.07 ns | 15.93 ns - 16.25 ns | 15.84 ns | - | - |
| `complex_ops/numerica128/add` | 84.50 ns | 84.39 ns - 84.63 ns | 84.29 ns | - | - |
| `complex_ops/numerica128/conjugate` | 34.23 ns | 34.17 ns - 34.29 ns | 34.13 ns | - | - |
| `complex_ops/numerica128/div` | 552.69 ns | 550.59 ns - 555.03 ns | 548.93 ns | - | - |
| `complex_ops/numerica128/div_checked` | 548.65 ns | 547.34 ns - 550.21 ns | 546.71 ns | - | - |
| `complex_ops/numerica128/div_real` | 118.86 ns | 118.70 ns - 119.04 ns | 118.79 ns | - | - |
| `complex_ops/numerica128/div_real_checked` | 121.08 ns | 120.48 ns - 121.77 ns | 120.12 ns | - | - |
| `complex_ops/numerica128/free_i` | 42.87 ns | 42.71 ns - 43.06 ns | 42.56 ns | - | - |
| `complex_ops/numerica128/from_scalar` | 31.07 ns | 31.05 ns - 31.08 ns | 31.05 ns | - | - |
| `complex_ops/numerica128/i` | 42.58 ns | 42.48 ns - 42.72 ns | 42.44 ns | - | - |
| `complex_ops/numerica128/mul` | 241.94 ns | 241.41 ns - 242.58 ns | 240.94 ns | - | - |
| `complex_ops/numerica128/neg` | 36.44 ns | 36.42 ns - 36.46 ns | 36.43 ns | - | - |
| `complex_ops/numerica128/norm_squared` | 123.18 ns | 122.77 ns - 123.65 ns | 122.16 ns | - | - |
| `complex_ops/numerica128/one` | 41.60 ns | 41.48 ns - 41.73 ns | 41.43 ns | - | - |
| `complex_ops/numerica128/powi` | 1.24 us | 1.24 us - 1.24 us | 1.24 us | - | - |
| `complex_ops/numerica128/powi_checked` | 1.24 us | 1.24 us - 1.25 us | 1.24 us | - | - |
| `complex_ops/numerica128/powi_checked_negative_one` | 535.94 ns | 535.28 ns - 536.69 ns | 534.64 ns | - | - |
| `complex_ops/numerica128/powi_negative_one` | 538.00 ns | 535.36 ns - 542.43 ns | 533.96 ns | - | - |
| `complex_ops/numerica128/reciprocal` | 250.07 ns | 249.39 ns - 250.80 ns | 248.54 ns | - | - |
| `complex_ops/numerica128/reciprocal_checked` | 248.32 ns | 248.04 ns - 248.66 ns | 247.82 ns | - | - |
| `complex_ops/numerica128/sub` | 94.16 ns | 93.94 ns - 94.48 ns | 93.78 ns | - | - |
| `complex_ops/numerica128/zero` | 23.98 ns | 23.88 ns - 24.10 ns | 23.78 ns | - | - |
| `complex_ops/symbolica/add` | 3.51 us | 3.49 us - 3.54 us | 3.46 us | - | - |
| `complex_ops/symbolica/conjugate` | 1.56 us | 1.55 us - 1.57 us | 1.55 us | - | - |
| `complex_ops/symbolica/div` | 27.74 us | 27.59 us - 27.92 us | 27.42 us | - | - |
| `complex_ops/symbolica/div_checked` | 27.48 us | 27.32 us - 27.67 us | 27.23 us | - | - |
| `complex_ops/symbolica/div_real` | 6.19 us | 6.15 us - 6.25 us | 6.09 us | - | - |
| `complex_ops/symbolica/div_real_checked` | 6.25 us | 6.18 us - 6.34 us | 6.14 us | - | - |
| `complex_ops/symbolica/free_i` | 29.88 ns | 29.80 ns - 29.97 ns | 29.74 ns | - | - |
| `complex_ops/symbolica/from_scalar` | 10.34 ns | 10.32 ns - 10.36 ns | 10.31 ns | - | - |
| `complex_ops/symbolica/i` | 29.85 ns | 29.79 ns - 29.91 ns | 29.79 ns | - | - |
| `complex_ops/symbolica/mul` | 12.64 us | 12.61 us - 12.67 us | 12.60 us | - | - |
| `complex_ops/symbolica/neg` | 3.06 us | 3.04 us - 3.07 us | 3.02 us | - | - |
| `complex_ops/symbolica/norm_squared` | 5.77 us | 5.72 us - 5.83 us | 5.68 us | - | - |
| `complex_ops/symbolica/one` | 30.74 ns | 30.67 ns - 30.82 ns | 30.60 ns | - | - |
| `complex_ops/symbolica/powi` | 58.60 us | 58.31 us - 58.91 us | 58.12 us | - | - |
| `complex_ops/symbolica/powi_checked` | 58.64 us | 58.34 us - 58.98 us | 58.07 us | - | - |
| `complex_ops/symbolica/powi_checked_negative_one` | 21.10 us | 20.98 us - 21.24 us | 20.86 us | - | - |
| `complex_ops/symbolica/powi_negative_one` | 20.99 us | 20.89 us - 21.11 us | 20.83 us | - | - |
| `complex_ops/symbolica/reciprocal` | 13.73 us | 13.64 us - 13.84 us | 13.56 us | - | - |
| `complex_ops/symbolica/reciprocal_checked` | 13.79 us | 13.71 us - 13.88 us | 13.63 us | - | - |
| `complex_ops/symbolica/sub` | 5.92 us | 5.88 us - 5.97 us | 5.83 us | - | - |
| `complex_ops/symbolica/zero` | 1.88 ns | 1.88 ns - 1.89 ns | 1.87 ns | - | - |
| `matrix3/gmp_mpfr128/mat3 determinant` | 689.56 ns | 686.30 ns - 693.21 ns | 682.18 ns | -3.70% | - |
| `matrix3/gmp_mpfr128/mat3 inverse` | 2.06 us | 2.05 us - 2.06 us | 2.05 us | -3.47% | - |
| `matrix3/gmp_mpfr128/mat3 mul mat3` | 1.77 us | 1.77 us - 1.78 us | 1.77 us | -2.64% | - |
| `matrix3/gmp_mpfr128/mat3 transform vec3` | 716.13 ns | 715.69 ns - 716.60 ns | 715.68 ns | -3.04% | - |
| `matrix3/hyperreal-rational/mat3 determinant` | 407.28 ns | 406.82 ns - 407.81 ns | 407.09 ns | -6.16% | - |
| `matrix3/hyperreal-rational/mat3 inverse` | 2.05 us | 2.05 us - 2.05 us | 2.05 us | -4.11% | - |
| `matrix3/hyperreal-rational/mat3 mul mat3` | 1.03 us | 1.03 us - 1.03 us | 1.03 us | -5.83% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3` | 484.65 ns | 484.09 ns - 485.32 ns | 483.77 ns | -4.42% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3 all-coord approx` | 599.26 ns | 596.08 ns - 602.77 ns | 592.20 ns | -4.01% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3 batch` | 1.57 us | 1.57 us - 1.58 us | 1.56 us | -4.91% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3 batch all-coord approx` | 1.95 us | 1.95 us - 1.96 us | 1.95 us | -4.01% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3 batch one-coord approx` | 1.61 us | 1.60 us - 1.61 us | 1.61 us | -6.01% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3 batch structural facts` | 1.69 us | 1.69 us - 1.69 us | 1.69 us | -2.33% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3 one-coord approx` | 518.97 ns | 517.34 ns - 520.80 ns | 514.46 ns | -4.04% | - |
| `matrix3/hyperreal-rational/mat3 transform vec3 sign/zero facts` | 536.17 ns | 535.57 ns - 536.82 ns | 535.47 ns | -3.61% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3` | 2.60 us | 2.59 us - 2.62 us | 2.57 us | -4.48% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3 all-coord approx` | 11.20 us | 11.17 us - 11.24 us | 11.14 us | -2.48% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3 batch` | 9.62 us | 9.60 us - 9.63 us | 9.60 us | -5.82% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3 batch all-coord approx` | 35.05 us | 35.01 us - 35.10 us | 35.01 us | -2.48% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3 batch one-coord approx` | 13.29 us | 13.27 us - 13.30 us | 13.28 us | -5.51% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3 batch structural facts` | 16.96 us | 16.93 us - 17.00 us | 16.92 us | -3.79% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3 one-coord approx` | 5.84 us | 5.83 us - 5.84 us | 5.83 us | -4.42% | - |
| `matrix3/hyperreal-symbolic/mat3 transform vec3 sign refinement` | 5.29 us | 5.26 us - 5.31 us | 5.24 us | -2.08% | - |
| `matrix3/hyperreal/mat3 determinant` | 478.92 ns | 478.23 ns - 479.76 ns | 478.34 ns | +1.69% | - |
| `matrix3/hyperreal/mat3 inverse` | 4.57 us | 4.56 us - 4.59 us | 4.55 us | +0.05% | - |
| `matrix3/hyperreal/mat3 mul mat3` | 1.41 us | 1.41 us - 1.42 us | 1.41 us | -2.24% | - |
| `matrix3/hyperreal/mat3 transform vec3` | 680.56 ns | 679.88 ns - 681.29 ns | 679.78 ns | -2.73% | - |
| `matrix3/numerica128/mat3 determinant` | 828.16 ns | 827.37 ns - 829.06 ns | 826.96 ns | -3.12% | - |
| `matrix3/numerica128/mat3 inverse` | 2.45 us | 2.44 us - 2.45 us | 2.44 us | -2.21% | - |
| `matrix3/numerica128/mat3 mul mat3` | 2.31 us | 2.31 us - 2.31 us | 2.31 us | -2.28% | - |
| `matrix3/numerica128/mat3 transform vec3` | 868.57 ns | 867.51 ns - 869.88 ns | 866.96 ns | -2.59% | - |
| `matrix3/symbolica/mat3 determinant` | 28.58 us | 28.53 us - 28.64 us | 28.52 us | -2.40% | - |
| `matrix3/symbolica/mat3 inverse` | 105.89 us | 105.31 us - 106.55 us | 104.69 us | -2.12% | - |
| `matrix3/symbolica/mat3 mul mat3` | 80.67 us | 80.50 us - 80.86 us | 80.40 us | -4.01% | - |
| `matrix3/symbolica/mat3 transform vec3` | 26.55 us | 26.53 us - 26.57 us | 26.53 us | -4.41% | - |
| `matrix4/gmp_mpfr128/mat4 determinant` | 3.33 us | 3.32 us - 3.34 us | 3.31 us | -2.25% | - |
| `matrix4/gmp_mpfr128/mat4 inverse` | 7.40 us | 7.38 us - 7.43 us | 7.37 us | -1.56% | - |
| `matrix4/gmp_mpfr128/mat4 mul mat4` | 4.05 us | 4.04 us - 4.06 us | 4.04 us | -2.25% | - |
| `matrix4/gmp_mpfr128/mat4 transform vec4` | 1.29 us | 1.28 us - 1.29 us | 1.28 us | -2.27% | - |
| `matrix4/hyperreal-rational/mat4 determinant` | 531.04 ns | 529.72 ns - 532.66 ns | 528.69 ns | -2.51% | - |
| `matrix4/hyperreal-rational/mat4 inverse` | 5.75 us | 5.73 us - 5.78 us | 5.71 us | -1.07% | - |
| `matrix4/hyperreal-rational/mat4 mul mat4` | 1.77 us | 1.77 us - 1.77 us | 1.77 us | -1.80% | - |
| `matrix4/hyperreal-rational/mat4 transform direction vec4` | 540.24 ns | 540.00 ns - 540.48 ns | 539.84 ns | -3.13% | - |
| `matrix4/hyperreal-rational/mat4 transform direction vec4 structural facts` | 597.61 ns | 596.14 ns - 599.36 ns | 594.59 ns | -1.86% | - |
| `matrix4/hyperreal-rational/mat4 transform point vec4` | 9.58 us | 9.57 us - 9.59 us | 9.57 us | +2.02% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4` | 568.81 ns | 568.10 ns - 569.72 ns | 567.99 ns | -0.52% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 all-coord approx` | 607.58 ns | 606.86 ns - 608.44 ns | 607.00 ns | -3.22% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch` | 4.35 us | 4.34 us - 4.36 us | 4.35 us | -0.48% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch all-coord approx` | 4.61 us | 4.60 us - 4.63 us | 4.61 us | +0.18% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch direction` | 4.13 us | 4.12 us - 4.14 us | 4.12 us | -3.15% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch direction all-coord approx` | 4.58 us | 4.57 us - 4.60 us | 4.57 us | -1.97% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch direction one-coord approx` | 4.21 us | 4.19 us - 4.22 us | 4.20 us | -1.55% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch direction structural facts` | 4.41 us | 4.40 us - 4.43 us | 4.41 us | -0.55% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch one-coord approx` | 4.38 us | 4.36 us - 4.40 us | 4.38 us | +0.10% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 batch structural facts` | 4.64 us | 4.62 us - 4.66 us | 4.65 us | -0.35% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 no-translation` | 571.21 ns | 569.68 ns - 573.13 ns | 568.76 ns | -1.18% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 no-translation all-coord approx` | 662.17 ns | 661.84 ns - 662.52 ns | 662.01 ns | -2.69% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 no-translation one-coord approx` | 603.39 ns | 601.83 ns - 605.27 ns | 601.09 ns | -2.73% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 no-translation structural facts` | 619.86 ns | 618.49 ns - 621.54 ns | 617.64 ns | +0.30% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 one-coord approx` | 575.03 ns | 574.60 ns - 575.49 ns | 575.05 ns | -1.15% | - |
| `matrix4/hyperreal-rational/mat4 transform vec4 sign/zero facts` | 618.07 ns | 617.47 ns - 618.76 ns | 617.69 ns | -3.02% | - |
| `matrix4/hyperreal-symbolic/mat4 transform direction vec4` | 4.08 us | 4.08 us - 4.09 us | 4.07 us | -5.55% | - |
| `matrix4/hyperreal-symbolic/mat4 transform direction vec4 all-coord approx` | 9.33 us | 9.31 us - 9.34 us | 9.33 us | -2.64% | - |
| `matrix4/hyperreal-symbolic/mat4 transform direction vec4 one-coord approx` | 7.43 us | 7.42 us - 7.44 us | 7.43 us | -3.33% | - |
| `matrix4/hyperreal-symbolic/mat4 transform direction vec4 structural facts` | 5.41 us | 5.40 us - 5.42 us | 5.40 us | -5.05% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4` | 7.61 us | 7.59 us - 7.64 us | 7.59 us | -6.27% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 all-coord approx` | 20.13 us | 20.11 us - 20.15 us | 20.11 us | -4.65% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch` | 31.77 us | 31.69 us - 31.87 us | 31.66 us | -5.36% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch all-coord approx` | 81.48 us | 81.33 us - 81.65 us | 81.26 us | -5.77% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction` | 18.30 us | 18.26 us - 18.35 us | 18.26 us | -5.36% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction all-coord approx` | 37.76 us | 37.69 us - 37.84 us | 37.62 us | -4.37% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction one-coord approx` | 21.81 us | 21.79 us - 21.85 us | 21.78 us | -5.35% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction structural facts` | 23.13 us | 23.10 us - 23.17 us | 23.09 us | -6.41% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch one-coord approx` | 36.32 us | 36.20 us - 36.48 us | 36.15 us | -4.18% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 batch structural facts` | 43.70 us | 43.67 us - 43.73 us | 43.67 us | -4.14% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation` | 4.07 us | 4.06 us - 4.09 us | 4.06 us | -4.69% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation all-coord approx` | 14.55 us | 14.54 us - 14.57 us | 14.55 us | -4.30% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation one-coord approx` | 7.52 us | 7.51 us - 7.53 us | 7.50 us | -2.90% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation structural facts` | 6.70 us | 6.70 us - 6.71 us | 6.70 us | -3.22% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 one-coord approx` | 11.58 us | 11.55 us - 11.62 us | 11.54 us | -3.90% | - |
| `matrix4/hyperreal-symbolic/mat4 transform vec4 sign refinement` | 10.84 us | 10.83 us - 10.85 us | 10.83 us | -4.68% | - |
| `matrix4/hyperreal/mat4 determinant` | 1.05 us | 1.04 us - 1.05 us | 1.04 us | -1.26% | - |
| `matrix4/hyperreal/mat4 determinant sparse` | 152.94 ns | 152.76 ns - 153.13 ns | 152.80 ns | -3.13% | - |
| `matrix4/hyperreal/mat4 inverse` | 7.07 us | 7.05 us - 7.08 us | 7.05 us | -2.57% | - |
| `matrix4/hyperreal/mat4 inverse sparse` | 2.52 us | 2.52 us - 2.52 us | 2.52 us | -1.50% | - |
| `matrix4/hyperreal/mat4 mul mat4` | 2.17 us | 2.16 us - 2.19 us | 2.16 us | -1.08% | - |
| `matrix4/hyperreal/mat4 mul mat4 sparse` | 697.47 ns | 696.51 ns - 698.53 ns | 696.18 ns | -2.35% | - |
| `matrix4/hyperreal/mat4 transform vec4` | 962.63 ns | 960.89 ns - 964.87 ns | 959.77 ns | -3.58% | - |
| `matrix4/numerica128/mat4 determinant` | 4.03 us | 4.02 us - 4.03 us | 4.02 us | +0.27% | - |
| `matrix4/numerica128/mat4 inverse` | 8.98 us | 8.96 us - 9.00 us | 8.94 us | +0.50% | - |
| `matrix4/numerica128/mat4 mul mat4` | 5.23 us | 5.22 us - 5.24 us | 5.22 us | +0.23% | - |
| `matrix4/numerica128/mat4 transform vec4` | 1.62 us | 1.61 us - 1.62 us | 1.61 us | -0.53% | - |
| `matrix4/symbolica/mat4 determinant` | 122.06 us | 121.59 us - 122.60 us | 121.43 us | -2.46% | - |
| `matrix4/symbolica/mat4 inverse` | 433.71 us | 432.98 us - 434.33 us | 433.95 us | -0.42% | - |
| `matrix4/symbolica/mat4 mul mat4` | 188.63 us | 188.02 us - 189.32 us | 187.68 us | -1.02% | - |
| `matrix4/symbolica/mat4 transform vec4` | 47.83 us | 47.64 us - 48.06 us | 47.56 us | +0.92% | - |
| `matrix_forms/hyperreal-rational/dyadic_dense/mat3 div_matrix` | 4.45 us | 4.45 us - 4.45 us | 4.44 us | - | - |
| `matrix_forms/hyperreal-rational/dyadic_dense/mat3 powi_negative` | 2.92 us | 2.91 us - 2.93 us | 2.90 us | - | - |
| `matrix_forms/hyperreal-rational/dyadic_dense/mat3 reciprocal` | 1.68 us | 1.68 us - 1.69 us | 1.68 us | - | - |
| `matrix_forms/hyperreal-rational/dyadic_dense/mat4 div_matrix` | 6.30 us | 6.28 us - 6.32 us | 6.27 us | - | - |
| `matrix_forms/hyperreal-rational/dyadic_dense/mat4 powi_negative` | 6.91 us | 6.88 us - 6.94 us | 6.85 us | - | - |
| `matrix_forms/hyperreal-rational/dyadic_dense/mat4 reciprocal` | 4.11 us | 4.10 us - 4.13 us | 4.08 us | - | - |
| `matrix_forms/hyperreal-rational/equal_decimal_den/mat3 div_matrix` | 4.92 us | 4.89 us - 4.94 us | 4.87 us | - | - |
| `matrix_forms/hyperreal-rational/equal_decimal_den/mat3 powi_negative` | 3.29 us | 3.27 us - 3.31 us | 3.25 us | - | - |
| `matrix_forms/hyperreal-rational/equal_decimal_den/mat3 reciprocal` | 2.05 us | 2.04 us - 2.05 us | 2.04 us | - | - |
| `matrix_forms/hyperreal-rational/equal_decimal_den/mat4 div_matrix` | 8.04 us | 8.00 us - 8.08 us | 7.96 us | - | - |
| `matrix_forms/hyperreal-rational/equal_decimal_den/mat4 powi_negative` | 7.94 us | 7.88 us - 8.00 us | 7.87 us | - | - |
| `matrix_forms/hyperreal-rational/equal_decimal_den/mat4 reciprocal` | 4.87 us | 4.85 us - 4.88 us | 4.87 us | - | - |
| `matrix_forms/hyperreal-rational/mixed_prime_den/mat3 div_matrix` | 5.62 us | 5.58 us - 5.66 us | 5.54 us | - | - |
| `matrix_forms/hyperreal-rational/mixed_prime_den/mat3 powi_negative` | 4.48 us | 4.46 us - 4.51 us | 4.44 us | - | - |
| `matrix_forms/hyperreal-rational/mixed_prime_den/mat3 reciprocal` | 2.59 us | 2.59 us - 2.60 us | 2.58 us | - | - |
| `matrix_forms/hyperreal-rational/mixed_prime_den/mat4 div_matrix` | 12.02 us | 11.91 us - 12.14 us | 11.84 us | - | - |
| `matrix_forms/hyperreal-rational/mixed_prime_den/mat4 powi_negative` | 39.26 us | 39.10 us - 39.45 us | 38.97 us | - | - |
| `matrix_forms/hyperreal-rational/mixed_prime_den/mat4 reciprocal` | 7.50 us | 7.46 us - 7.54 us | 7.42 us | - | - |
| `matrix_forms/hyperreal-rational/sparse_integer/mat3 div_matrix` | 3.01 us | 2.99 us - 3.03 us | 2.97 us | - | - |
| `matrix_forms/hyperreal-rational/sparse_integer/mat3 powi_negative` | 2.40 us | 2.38 us - 2.41 us | 2.36 us | - | - |
| `matrix_forms/hyperreal-rational/sparse_integer/mat3 reciprocal` | 1.85 us | 1.84 us - 1.86 us | 1.84 us | - | - |
| `matrix_forms/hyperreal-rational/sparse_integer/mat4 div_matrix` | 5.36 us | 5.33 us - 5.39 us | 5.31 us | - | - |
| `matrix_forms/hyperreal-rational/sparse_integer/mat4 powi_negative` | 5.81 us | 5.79 us - 5.85 us | 5.76 us | - | - |
| `matrix_forms/hyperreal-rational/sparse_integer/mat4 reciprocal` | 3.83 us | 3.81 us - 3.86 us | 3.79 us | - | - |
| `matrix_forms/hyperreal/dyadic_dense/mat3 div_matrix` | 4.41 us | 4.41 us - 4.42 us | 4.41 us | - | - |
| `matrix_forms/hyperreal/dyadic_dense/mat3 powi_negative` | 2.86 us | 2.85 us - 2.87 us | 2.85 us | - | - |
| `matrix_forms/hyperreal/dyadic_dense/mat3 reciprocal` | 1.64 us | 1.64 us - 1.64 us | 1.63 us | - | - |
| `matrix_forms/hyperreal/dyadic_dense/mat4 div_matrix` | 6.13 us | 6.12 us - 6.14 us | 6.10 us | - | - |
| `matrix_forms/hyperreal/dyadic_dense/mat4 powi_negative` | 6.79 us | 6.78 us - 6.79 us | 6.78 us | - | - |
| `matrix_forms/hyperreal/dyadic_dense/mat4 reciprocal` | 4.06 us | 4.05 us - 4.06 us | 4.04 us | - | - |
| `matrix_forms/hyperreal/equal_decimal_den/mat3 div_matrix` | 49.26 us | 49.04 us - 49.53 us | 48.85 us | - | - |
| `matrix_forms/hyperreal/equal_decimal_den/mat3 powi_negative` | 27.11 us | 27.04 us - 27.20 us | 27.00 us | - | - |
| `matrix_forms/hyperreal/equal_decimal_den/mat3 reciprocal` | 4.77 us | 4.76 us - 4.78 us | 4.76 us | - | - |
| `matrix_forms/hyperreal/equal_decimal_den/mat4 div_matrix` | 108.61 us | 108.43 us - 108.81 us | 108.34 us | - | - |
| `matrix_forms/hyperreal/equal_decimal_den/mat4 powi_negative` | 68.51 us | 68.28 us - 68.77 us | 68.11 us | - | - |
| `matrix_forms/hyperreal/equal_decimal_den/mat4 reciprocal` | 11.62 us | 11.60 us - 11.64 us | 11.60 us | - | - |
| `matrix_forms/hyperreal/mixed_prime_den/mat3 div_matrix` | 63.87 us | 63.69 us - 64.12 us | 63.59 us | - | - |
| `matrix_forms/hyperreal/mixed_prime_den/mat3 powi_negative` | 26.97 us | 26.93 us - 27.02 us | 26.90 us | - | - |
| `matrix_forms/hyperreal/mixed_prime_den/mat3 reciprocal` | 4.95 us | 4.94 us - 4.95 us | 4.94 us | - | - |
| `matrix_forms/hyperreal/mixed_prime_den/mat4 div_matrix` | 120.80 us | 120.35 us - 121.38 us | 120.13 us | - | - |
| `matrix_forms/hyperreal/mixed_prime_den/mat4 powi_negative` | 57.23 us | 57.17 us - 57.31 us | 57.18 us | - | - |
| `matrix_forms/hyperreal/mixed_prime_den/mat4 reciprocal` | 11.44 us | 11.40 us - 11.50 us | 11.40 us | - | - |
| `matrix_forms/hyperreal/sparse_integer/mat3 div_matrix` | 3.00 us | 2.98 us - 3.03 us | 2.97 us | - | - |
| `matrix_forms/hyperreal/sparse_integer/mat3 powi_negative` | 2.44 us | 2.43 us - 2.44 us | 2.43 us | - | - |
| `matrix_forms/hyperreal/sparse_integer/mat3 reciprocal` | 1.91 us | 1.91 us - 1.92 us | 1.91 us | - | - |
| `matrix_forms/hyperreal/sparse_integer/mat4 div_matrix` | 5.54 us | 5.51 us - 5.58 us | 5.50 us | - | - |
| `matrix_forms/hyperreal/sparse_integer/mat4 powi_negative` | 6.19 us | 6.17 us - 6.21 us | 6.15 us | - | - |
| `matrix_forms/hyperreal/sparse_integer/mat4 reciprocal` | 4.19 us | 4.17 us - 4.21 us | 4.15 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 add` | 365.26 ns | 364.40 ns - 366.25 ns | 363.87 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 add_scalar` | 541.52 ns | 541.00 ns - 542.12 ns | 540.71 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 bitxor` | 4.74 us | 4.74 us - 4.75 us | 4.74 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 div_matrix` | 3.48 us | 3.46 us - 3.49 us | 3.45 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 div_matrix_checked` | 3.54 us | 3.52 us - 3.57 us | 3.49 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 div_matrix_checked_abort` | 3.44 us | 3.43 us - 3.45 us | 3.42 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 div_scalar` | 786.09 ns | 780.86 ns - 791.89 ns | 775.41 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 div_scalar_checked` | 771.40 ns | 769.08 ns - 774.01 ns | 766.54 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 div_scalar_checked_abort` | 766.98 ns | 765.86 ns - 768.32 ns | 765.63 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 identity` | 258.40 ns | 257.62 ns - 259.33 ns | 256.93 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 inverse_checked` | 1.88 us | 1.87 us - 1.88 us | 1.87 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 inverse_checked_abort` | 1.92 us | 1.91 us - 1.93 us | 1.90 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 mul_scalar` | 654.20 ns | 652.38 ns - 656.32 ns | 650.37 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 neg` | 456.23 ns | 455.07 ns - 457.51 ns | 453.34 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 new` | 225.77 ns | 224.97 ns - 226.70 ns | 224.38 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 powi` | 4.74 us | 4.74 us - 4.75 us | 4.74 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 powi_checked` | 4.79 us | 4.77 us - 4.81 us | 4.74 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 powi_checked_abort` | 4.78 us | 4.76 us - 4.79 us | 4.75 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 reciprocal` | 1.88 us | 1.88 us - 1.88 us | 1.88 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 reciprocal_checked` | 1.89 us | 1.88 us - 1.90 us | 1.87 us | - | - |
| `matrix_ops/gmp_mpfr128/mat3 sub` | 364.31 ns | 363.18 ns - 365.58 ns | 362.80 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 sub_scalar` | 548.64 ns | 546.76 ns - 550.78 ns | 544.87 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 transpose` | 202.44 ns | 201.51 ns - 203.46 ns | 200.51 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat3 zero` | 228.50 ns | 227.50 ns - 229.60 ns | 226.27 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 add` | 608.11 ns | 606.57 ns - 609.92 ns | 605.64 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 add_scalar` | 917.57 ns | 915.36 ns - 920.16 ns | 914.01 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 bitxor` | 10.86 us | 10.84 us - 10.88 us | 10.83 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 div_matrix` | 10.83 us | 10.81 us - 10.84 us | 10.80 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 div_scalar` | 1.39 us | 1.38 us - 1.39 us | 1.38 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 identity` | 352.76 ns | 351.02 ns - 354.70 ns | 350.32 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 mul_scalar` | 1.09 us | 1.09 us - 1.09 us | 1.09 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 neg` | 755.91 ns | 754.14 ns - 758.53 ns | 753.53 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 powi` | 10.90 us | 10.87 us - 10.95 us | 10.85 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 powi_checked` | 10.93 us | 10.88 us - 10.99 us | 10.85 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 reciprocal` | 7.22 us | 7.20 us - 7.24 us | 7.19 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 reciprocal_checked` | 7.17 us | 7.16 us - 7.18 us | 7.15 us | - | - |
| `matrix_ops/gmp_mpfr128/mat4 sub` | 606.67 ns | 603.86 ns - 610.23 ns | 602.13 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 sub_scalar` | 918.26 ns | 914.86 ns - 922.36 ns | 913.42 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 transpose` | 346.16 ns | 344.67 ns - 347.85 ns | 343.78 ns | - | - |
| `matrix_ops/gmp_mpfr128/mat4 zero` | 344.31 ns | 341.69 ns - 347.41 ns | 338.94 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 add` | 377.34 ns | 376.10 ns - 378.73 ns | 375.01 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 add_scalar` | 598.72 ns | 594.44 ns - 603.37 ns | 591.04 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 affine_div_matrix` | 3.50 us | 3.49 us - 3.53 us | 3.47 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 affine_div_matrix_translation` | 2.78 us | 2.75 us - 2.82 us | 2.73 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 affine_inverse` | 1.70 us | 1.69 us - 1.70 us | 1.69 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 bitxor` | 4.37 us | 4.34 us - 4.39 us | 4.33 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 direct_div_matrix` | 5.29 us | 5.25 us - 5.32 us | 5.21 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 direct_div_matrix_checked` | 5.22 us | 5.20 us - 5.24 us | 5.18 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 direct_div_matrix_checked_abort` | 5.26 us | 5.22 us - 5.30 us | 5.19 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 div_matrix` | 5.77 us | 5.75 us - 5.79 us | 5.74 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 div_matrix_checked` | 5.90 us | 5.84 us - 5.98 us | 5.82 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 div_matrix_checked_abort` | 6.00 us | 5.94 us - 6.06 us | 5.90 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 div_scalar` | 439.34 ns | 437.09 ns - 441.93 ns | 435.44 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 div_scalar_checked` | 644.03 ns | 641.92 ns - 646.52 ns | 641.00 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 div_scalar_checked_abort` | 696.22 ns | 692.19 ns - 700.85 ns | 688.04 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 identity` | 242.39 ns | 241.87 ns - 242.99 ns | 241.85 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 inverse_checked` | 2.51 us | 2.50 us - 2.53 us | 2.48 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 inverse_checked_abort` | 2.79 us | 2.78 us - 2.81 us | 2.77 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_diagonal_div_matrix` | 386.95 ns | 384.51 ns - 389.67 ns | 382.61 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_diagonal_div_vector` | 686.89 ns | 683.49 ns - 690.92 ns | 680.31 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_diagonal_inverse` | 164.92 ns | 163.91 ns - 166.02 ns | 162.58 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_lower_triangular_div_matrix` | 1.33 us | 1.32 us - 1.35 us | 1.30 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_lower_triangular_inverse` | 488.78 ns | 486.50 ns - 491.31 ns | 485.32 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_lower_triangular_inverse_checked` | 503.91 ns | 500.77 ns - 507.20 ns | 504.09 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_uniform_diagonal_div_vector` | 538.40 ns | 536.60 ns - 540.54 ns | 534.63 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_uniform_scale_inverse` | 111.76 ns | 111.24 ns - 112.32 ns | 110.74 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_upper_triangular_div_matrix` | 1.21 us | 1.20 us - 1.22 us | 1.19 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_upper_triangular_inverse` | 431.26 ns | 428.29 ns - 434.64 ns | 424.52 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 known_upper_triangular_inverse_checked` | 428.97 ns | 427.59 ns - 430.48 ns | 426.26 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 mul_scalar` | 705.48 ns | 703.41 ns - 707.84 ns | 700.45 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 neg` | 220.15 ns | 219.69 ns - 220.65 ns | 219.45 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 new` | 1.01 us | 1.00 us - 1.02 us | 993.45 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 powi` | 4.38 us | 4.35 us - 4.41 us | 4.34 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 powi_checked` | 4.36 us | 4.34 us - 4.38 us | 4.33 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 powi_checked_abort` | 4.39 us | 4.37 us - 4.42 us | 4.35 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 powi_checked_negative` | 8.24 us | 8.15 us - 8.34 us | 8.08 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 powi_negative` | 8.15 us | 8.10 us - 8.20 us | 8.05 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 powi_negative_one` | 2.49 us | 2.48 us - 2.50 us | 2.46 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 reciprocal` | 2.51 us | 2.49 us - 2.52 us | 2.47 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 reciprocal_checked` | 2.51 us | 2.49 us - 2.52 us | 2.47 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 sub` | 441.79 ns | 440.58 ns - 443.11 ns | 440.95 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 sub_scalar` | 718.67 ns | 715.90 ns - 721.81 ns | 712.99 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 transpose` | 229.43 ns | 228.87 ns - 230.14 ns | 228.93 ns | - | - |
| `matrix_ops/hyperreal-rational/mat3 uniform_scale_reciprocal` | 1.22 us | 1.21 us - 1.22 us | 1.21 us | - | - |
| `matrix_ops/hyperreal-rational/mat3 zero` | 224.20 ns | 222.52 ns - 226.12 ns | 220.52 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 add` | 682.67 ns | 681.91 ns - 683.46 ns | 682.21 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 add_scalar` | 968.71 ns | 968.13 ns - 969.34 ns | 968.19 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 affine_div_matrix_checked` | 7.79 us | 7.75 us - 7.83 us | 7.75 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 affine_div_matrix_checked_abort` | 7.89 us | 7.87 us - 7.91 us | 7.88 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 affine_div_matrix_translation` | 4.60 us | 4.60 us - 4.61 us | 4.60 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 affine_div_matrix_translation_checked` | 4.61 us | 4.60 us - 4.63 us | 4.59 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 affine_div_matrix_translation_checked_abort` | 4.63 us | 4.62 us - 4.64 us | 4.62 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 bitxor` | 6.78 us | 6.76 us - 6.81 us | 6.75 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 diagonal_reciprocal` | 2.03 us | 2.01 us - 2.04 us | 1.99 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_div_matrix` | 9.15 us | 9.14 us - 9.16 us | 9.14 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_div_matrix_checked` | 9.19 us | 9.19 us - 9.20 us | 9.19 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_div_matrix_checked_abort` | 9.24 us | 9.23 us - 9.25 us | 9.24 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_div_matrix_exact_left` | 9.20 us | 9.19 us - 9.22 us | 9.19 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_inverse` | 6.31 us | 6.30 us - 6.32 us | 6.30 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_inverse_checked` | 6.32 us | 6.32 us - 6.33 us | 6.31 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_inverse_checked_abort` | 6.46 us | 6.39 us - 6.56 us | 6.35 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_powi_negative` | 8.73 us | 8.72 us - 8.74 us | 8.73 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_powi_negative_one` | 6.33 us | 6.32 us - 6.34 us | 6.32 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_reciprocal` | 6.33 us | 6.32 us - 6.35 us | 6.31 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_reciprocal_checked` | 6.32 us | 6.31 us - 6.33 us | 6.31 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 direct_reciprocal_checked_abort` | 6.37 us | 6.35 us - 6.38 us | 6.34 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 div_matrix` | 9.29 us | 9.25 us - 9.33 us | 9.22 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 div_matrix_checked` | 9.23 us | 9.21 us - 9.24 us | 9.20 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 div_matrix_checked_abort` | 9.31 us | 9.30 us - 9.31 us | 9.30 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 div_scalar` | 938.24 ns | 934.34 ns - 942.85 ns | 931.61 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 div_scalar_checked` | 1.13 us | 1.13 us - 1.14 us | 1.13 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 div_scalar_checked_abort` | 1.16 us | 1.16 us - 1.17 us | 1.16 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity` | 302.67 ns | 300.39 ns - 305.33 ns | 297.73 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_direction_batch_assumed` | 334.09 ns | 333.07 ns - 335.23 ns | 333.29 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_direction_transform` | 194.40 ns | 193.68 ns - 195.28 ns | 193.29 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_direction_transform_direct` | 197.11 ns | 196.15 ns - 198.16 ns | 196.60 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_direction_transform_generic` | 305.25 ns | 304.66 ns - 305.93 ns | 304.26 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_point_batch_assumed` | 2.32 us | 2.31 us - 2.33 us | 2.30 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_point_transform` | 158.67 ns | 157.78 ns - 159.61 ns | 156.44 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_point_transform_direct` | 156.68 ns | 155.74 ns - 157.90 ns | 155.19 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 identity_point_transform_generic` | 307.59 ns | 306.12 ns - 309.37 ns | 304.95 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 inverse_checked` | 5.99 us | 5.98 us - 6.01 us | 5.97 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 inverse_checked_abort` | 6.00 us | 6.00 us - 6.01 us | 5.99 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_diagonal_div_matrix` | 666.66 ns | 663.30 ns - 670.48 ns | 659.59 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_diagonal_div_vector` | 968.77 ns | 964.45 ns - 973.44 ns | 959.81 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_diagonal_div_vector_direction` | 772.95 ns | 768.85 ns - 777.92 ns | 765.68 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_diagonal_div_vector_direction_only` | 806.48 ns | 801.03 ns - 813.65 ns | 796.98 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_diagonal_div_vector_point` | 1.36 us | 1.35 us - 1.37 us | 1.34 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_diagonal_inverse` | 223.85 ns | 222.83 ns - 225.03 ns | 221.76 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_lower_triangular_div_matrix` | 3.44 us | 3.42 us - 3.46 us | 3.40 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_lower_triangular_inverse` | 949.68 ns | 944.09 ns - 955.88 ns | 936.86 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_lower_triangular_inverse_checked` | 950.64 ns | 944.45 ns - 957.45 ns | 936.38 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_orthonormal_div_matrix` | 1.48 us | 1.48 us - 1.49 us | 1.47 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_orthonormal_inverse` | 481.48 ns | 478.27 ns - 484.97 ns | 474.55 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_signed_permutation_batch` | 274.14 ns | 273.53 ns - 274.83 ns | 273.26 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_signed_permutation_div_matrix` | 414.15 ns | 412.96 ns - 415.56 ns | 412.44 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_signed_permutation_inverse` | 170.28 ns | 169.80 ns - 170.82 ns | 169.23 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_signed_permutation_transform` | 61.76 ns | 61.64 ns - 61.90 ns | 61.50 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_translation_div_matrix` | 555.36 ns | 549.65 ns - 562.00 ns | 541.87 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_translation_inverse` | 182.81 ns | 182.03 ns - 183.73 ns | 181.19 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_uniform_diagonal_div_vector` | 929.78 ns | 922.12 ns - 938.42 ns | 913.20 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_uniform_diagonal_div_vector_direction` | 572.51 ns | 568.85 ns - 576.56 ns | 563.16 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_uniform_diagonal_div_vector_point` | 1.03 us | 1.03 us - 1.03 us | 1.03 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_uniform_scale_inverse` | 149.05 ns | 148.27 ns - 149.94 ns | 147.34 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_upper_triangular_div_matrix` | 2.93 us | 2.91 us - 2.95 us | 2.89 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_upper_triangular_inverse` | 982.11 ns | 978.78 ns - 985.91 ns | 976.19 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 known_upper_triangular_inverse_checked` | 994.25 ns | 986.94 ns - 1.00 us | 973.84 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 mul_scalar` | 1.30 us | 1.30 us - 1.30 us | 1.30 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 neg` | 354.02 ns | 353.67 ns - 354.41 ns | 353.71 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 powi` | 6.77 us | 6.76 us - 6.79 us | 6.74 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 powi_checked` | 6.82 us | 6.79 us - 6.84 us | 6.78 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 powi_checked_abort` | 6.73 us | 6.72 us - 6.74 us | 6.71 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 powi_checked_negative` | 19.61 us | 19.46 us - 19.80 us | 19.30 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 powi_negative` | 19.29 us | 19.25 us - 19.36 us | 19.25 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 powi_negative_one` | 5.99 us | 5.97 us - 6.01 us | 5.97 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 reciprocal` | 6.19 us | 6.14 us - 6.23 us | 6.09 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 reciprocal_checked` | 5.99 us | 5.97 us - 6.01 us | 5.95 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 sub` | 772.25 ns | 771.40 ns - 773.34 ns | 771.03 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 sub_scalar` | 1.25 us | 1.24 us - 1.26 us | 1.24 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_direction_batch` | 2.48 us | 2.47 us - 2.49 us | 2.46 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_direction_batch_assumed` | 485.36 ns | 483.83 ns - 487.88 ns | 483.54 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_direction_batch_public_assumed` | 487.60 ns | 485.89 ns - 489.55 ns | 483.89 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_direction_transform` | 211.58 ns | 211.28 ns - 211.90 ns | 211.21 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_direction_transform_generic` | 267.12 ns | 266.61 ns - 267.76 ns | 266.82 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_point_batch` | 2.85 us | 2.84 us - 2.86 us | 2.83 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_point_batch_assumed` | 2.61 us | 2.60 us - 2.62 us | 2.60 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_point_batch_public_assumed` | 2.64 us | 2.63 us - 2.65 us | 2.62 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 translated_diagonal_point_transform_generic` | 337.57 ns | 336.88 ns - 338.32 ns | 336.58 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 transpose` | 227.73 ns | 226.30 ns - 229.25 ns | 225.29 ns | - | - |
| `matrix_ops/hyperreal-rational/mat4 uniform_scale_reciprocal` | 1.97 us | 1.96 us - 1.97 us | 1.95 us | - | - |
| `matrix_ops/hyperreal-rational/mat4 zero` | 231.38 ns | 230.25 ns - 232.64 ns | 229.64 ns | - | - |
| `matrix_ops/hyperreal/mat3 add` | 360.62 ns | 359.06 ns - 362.42 ns | 357.61 ns | - | - |
| `matrix_ops/hyperreal/mat3 add_scalar` | 606.95 ns | 606.23 ns - 607.76 ns | 606.25 ns | - | - |
| `matrix_ops/hyperreal/mat3 affine_div_matrix` | 4.26 us | 4.26 us - 4.27 us | 4.26 us | - | - |
| `matrix_ops/hyperreal/mat3 affine_div_matrix_translation` | 2.73 us | 2.71 us - 2.74 us | 2.70 us | - | - |
| `matrix_ops/hyperreal/mat3 affine_inverse` | 1.67 us | 1.66 us - 1.68 us | 1.66 us | - | - |
| `matrix_ops/hyperreal/mat3 bitxor` | 3.97 us | 3.96 us - 3.98 us | 3.95 us | - | - |
| `matrix_ops/hyperreal/mat3 direct_div_matrix` | 42.23 us | 42.15 us - 42.32 us | 42.13 us | - | - |
| `matrix_ops/hyperreal/mat3 direct_div_matrix_checked` | 42.40 us | 42.23 us - 42.59 us | 42.05 us | - | - |
| `matrix_ops/hyperreal/mat3 direct_div_matrix_checked_abort` | 43.09 us | 42.88 us - 43.33 us | 42.66 us | - | - |
| `matrix_ops/hyperreal/mat3 div_matrix` | 23.22 us | 23.20 us - 23.24 us | 23.21 us | - | - |
| `matrix_ops/hyperreal/mat3 div_matrix_checked` | 23.17 us | 23.15 us - 23.20 us | 23.15 us | - | - |
| `matrix_ops/hyperreal/mat3 div_matrix_checked_abort` | 23.34 us | 23.30 us - 23.39 us | 23.24 us | - | - |
| `matrix_ops/hyperreal/mat3 div_scalar` | 313.65 ns | 313.41 ns - 313.99 ns | 313.42 ns | - | - |
| `matrix_ops/hyperreal/mat3 div_scalar_checked` | 519.74 ns | 518.86 ns - 520.83 ns | 518.41 ns | - | - |
| `matrix_ops/hyperreal/mat3 div_scalar_checked_abort` | 561.18 ns | 560.67 ns - 561.74 ns | 560.87 ns | - | - |
| `matrix_ops/hyperreal/mat3 identity` | 245.03 ns | 244.34 ns - 245.95 ns | 244.53 ns | - | - |
| `matrix_ops/hyperreal/mat3 inverse_checked` | 4.54 us | 4.53 us - 4.56 us | 4.50 us | - | - |
| `matrix_ops/hyperreal/mat3 inverse_checked_abort` | 4.69 us | 4.68 us - 4.70 us | 4.67 us | - | - |
| `matrix_ops/hyperreal/mat3 known_diagonal_div_matrix` | 379.07 ns | 378.36 ns - 379.95 ns | 378.11 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_diagonal_div_vector` | 941.08 ns | 940.03 ns - 942.28 ns | 940.50 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_diagonal_inverse` | 156.56 ns | 156.23 ns - 156.96 ns | 156.06 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_lower_triangular_div_matrix` | 1.47 us | 1.47 us - 1.47 us | 1.47 us | - | - |
| `matrix_ops/hyperreal/mat3 known_lower_triangular_inverse` | 482.09 ns | 480.74 ns - 483.56 ns | 479.16 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_lower_triangular_inverse_checked` | 483.95 ns | 482.82 ns - 485.32 ns | 482.68 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_uniform_diagonal_div_vector` | 585.40 ns | 584.92 ns - 585.96 ns | 584.44 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_uniform_scale_inverse` | 109.43 ns | 109.26 ns - 109.64 ns | 109.21 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_upper_triangular_div_matrix` | 1.55 us | 1.55 us - 1.56 us | 1.55 us | - | - |
| `matrix_ops/hyperreal/mat3 known_upper_triangular_inverse` | 428.14 ns | 426.37 ns - 429.96 ns | 423.34 ns | - | - |
| `matrix_ops/hyperreal/mat3 known_upper_triangular_inverse_checked` | 425.05 ns | 423.80 ns - 426.41 ns | 422.32 ns | - | - |
| `matrix_ops/hyperreal/mat3 mul_scalar` | 660.02 ns | 658.49 ns - 661.73 ns | 657.28 ns | - | - |
| `matrix_ops/hyperreal/mat3 neg` | 220.50 ns | 219.94 ns - 221.18 ns | 219.38 ns | - | - |
| `matrix_ops/hyperreal/mat3 new` | 474.95 ns | 473.47 ns - 476.76 ns | 472.73 ns | - | - |
| `matrix_ops/hyperreal/mat3 powi` | 3.99 us | 3.97 us - 4.01 us | 3.96 us | - | - |
| `matrix_ops/hyperreal/mat3 powi_checked` | 3.95 us | 3.94 us - 3.96 us | 3.94 us | - | - |
| `matrix_ops/hyperreal/mat3 powi_checked_abort` | 3.98 us | 3.97 us - 4.00 us | 3.96 us | - | - |
| `matrix_ops/hyperreal/mat3 powi_checked_negative` | 18.30 us | 18.28 us - 18.33 us | 18.27 us | - | - |
| `matrix_ops/hyperreal/mat3 powi_negative` | 18.30 us | 18.26 us - 18.36 us | 18.22 us | - | - |
| `matrix_ops/hyperreal/mat3 powi_negative_one` | 4.54 us | 4.51 us - 4.57 us | 4.50 us | - | - |
| `matrix_ops/hyperreal/mat3 reciprocal` | 4.54 us | 4.53 us - 4.55 us | 4.53 us | - | - |
| `matrix_ops/hyperreal/mat3 reciprocal_checked` | 4.50 us | 4.50 us - 4.51 us | 4.50 us | - | - |
| `matrix_ops/hyperreal/mat3 sub` | 426.90 ns | 426.50 ns - 427.48 ns | 426.63 ns | - | - |
| `matrix_ops/hyperreal/mat3 sub_scalar` | 795.54 ns | 794.45 ns - 796.77 ns | 793.96 ns | - | - |
| `matrix_ops/hyperreal/mat3 transpose` | 220.01 ns | 219.74 ns - 220.31 ns | 219.73 ns | - | - |
| `matrix_ops/hyperreal/mat3 uniform_scale_reciprocal` | 1.20 us | 1.20 us - 1.21 us | 1.20 us | - | - |
| `matrix_ops/hyperreal/mat3 zero` | 223.01 ns | 222.65 ns - 223.41 ns | 222.74 ns | - | - |
| `matrix_ops/hyperreal/mat4 add` | 690.35 ns | 686.82 ns - 694.07 ns | 686.55 ns | - | - |
| `matrix_ops/hyperreal/mat4 add_scalar` | 923.24 ns | 920.10 ns - 926.82 ns | 915.90 ns | - | - |
| `matrix_ops/hyperreal/mat4 affine_div_matrix_checked` | 20.37 us | 20.29 us - 20.47 us | 20.24 us | - | - |
| `matrix_ops/hyperreal/mat4 affine_div_matrix_checked_abort` | 20.20 us | 20.12 us - 20.28 us | 20.09 us | - | - |
| `matrix_ops/hyperreal/mat4 affine_div_matrix_translation` | 4.69 us | 4.67 us - 4.71 us | 4.66 us | - | - |
| `matrix_ops/hyperreal/mat4 affine_div_matrix_translation_checked` | 4.66 us | 4.64 us - 4.68 us | 4.63 us | - | - |
| `matrix_ops/hyperreal/mat4 affine_div_matrix_translation_checked_abort` | 4.67 us | 4.66 us - 4.69 us | 4.65 us | - | - |
| `matrix_ops/hyperreal/mat4 bitxor` | 5.24 us | 5.22 us - 5.27 us | 5.21 us | - | - |
| `matrix_ops/hyperreal/mat4 diagonal_reciprocal` | 2.05 us | 2.03 us - 2.07 us | 2.01 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_div_matrix` | 8.89 us | 8.85 us - 8.94 us | 8.79 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_div_matrix_checked` | 8.81 us | 8.79 us - 8.84 us | 8.76 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_div_matrix_checked_abort` | 8.83 us | 8.81 us - 8.86 us | 8.79 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_div_matrix_exact_left` | 8.76 us | 8.75 us - 8.79 us | 8.75 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_inverse` | 6.34 us | 6.33 us - 6.36 us | 6.32 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_inverse_checked` | 6.61 us | 6.56 us - 6.66 us | 6.49 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_inverse_checked_abort` | 6.47 us | 6.44 us - 6.51 us | 6.42 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_powi_negative` | 9.06 us | 9.00 us - 9.14 us | 8.90 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_powi_negative_one` | 6.58 us | 6.53 us - 6.63 us | 6.47 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_reciprocal` | 6.37 us | 6.35 us - 6.38 us | 6.34 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_reciprocal_checked` | 6.45 us | 6.42 us - 6.48 us | 6.40 us | - | - |
| `matrix_ops/hyperreal/mat4 direct_reciprocal_checked_abort` | 6.49 us | 6.46 us - 6.52 us | 6.44 us | - | - |
| `matrix_ops/hyperreal/mat4 div_matrix` | 37.10 us | 36.93 us - 37.29 us | 36.77 us | - | - |
| `matrix_ops/hyperreal/mat4 div_matrix_checked` | 36.66 us | 36.58 us - 36.75 us | 36.60 us | - | - |
| `matrix_ops/hyperreal/mat4 div_matrix_checked_abort` | 37.19 us | 37.01 us - 37.39 us | 36.82 us | - | - |
| `matrix_ops/hyperreal/mat4 div_scalar` | 644.00 ns | 641.15 ns - 647.13 ns | 638.64 ns | - | - |
| `matrix_ops/hyperreal/mat4 div_scalar_checked` | 889.23 ns | 885.07 ns - 893.80 ns | 880.80 ns | - | - |
| `matrix_ops/hyperreal/mat4 div_scalar_checked_abort` | 950.43 ns | 943.12 ns - 958.47 ns | 937.19 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity` | 280.21 ns | 279.51 ns - 281.03 ns | 279.09 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity_direction_batch_assumed` | 343.14 ns | 341.90 ns - 344.45 ns | 342.29 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity_direction_transform` | 205.21 ns | 203.79 ns - 206.83 ns | 202.63 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity_direction_transform_direct` | 201.19 ns | 200.41 ns - 202.08 ns | 199.46 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity_direction_transform_generic` | 306.82 ns | 304.87 ns - 309.27 ns | 303.40 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity_point_batch_assumed` | 2.37 us | 2.36 us - 2.38 us | 2.34 us | - | - |
| `matrix_ops/hyperreal/mat4 identity_point_transform` | 159.81 ns | 158.41 ns - 161.31 ns | 155.39 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity_point_transform_direct` | 162.77 ns | 161.64 ns - 163.89 ns | 164.75 ns | - | - |
| `matrix_ops/hyperreal/mat4 identity_point_transform_generic` | 305.50 ns | 304.58 ns - 306.51 ns | 304.17 ns | - | - |
| `matrix_ops/hyperreal/mat4 inverse_checked` | 7.08 us | 7.07 us - 7.10 us | 7.05 us | - | - |
| `matrix_ops/hyperreal/mat4 inverse_checked_abort` | 7.07 us | 7.05 us - 7.09 us | 7.04 us | - | - |
| `matrix_ops/hyperreal/mat4 known_diagonal_div_matrix` | 657.63 ns | 656.21 ns - 659.35 ns | 655.35 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_diagonal_div_vector` | 957.53 ns | 955.52 ns - 959.75 ns | 955.39 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_diagonal_div_vector_direction` | 764.74 ns | 762.89 ns - 766.92 ns | 762.12 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_diagonal_div_vector_direction_only` | 794.90 ns | 789.73 ns - 800.56 ns | 783.61 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_diagonal_div_vector_point` | 1.39 us | 1.38 us - 1.39 us | 1.37 us | - | - |
| `matrix_ops/hyperreal/mat4 known_diagonal_inverse` | 211.03 ns | 210.37 ns - 211.77 ns | 209.83 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_lower_triangular_div_matrix` | 3.38 us | 3.37 us - 3.40 us | 3.37 us | - | - |
| `matrix_ops/hyperreal/mat4 known_lower_triangular_inverse` | 916.12 ns | 914.59 ns - 917.81 ns | 914.05 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_lower_triangular_inverse_checked` | 938.81 ns | 937.57 ns - 940.28 ns | 937.49 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_orthonormal_div_matrix` | 1.45 us | 1.44 us - 1.46 us | 1.44 us | - | - |
| `matrix_ops/hyperreal/mat4 known_orthonormal_inverse` | 492.21 ns | 488.00 ns - 497.02 ns | 482.53 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_signed_permutation_batch` | 269.86 ns | 268.89 ns - 270.96 ns | 268.00 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_signed_permutation_div_matrix` | 434.93 ns | 432.93 ns - 437.12 ns | 430.26 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_signed_permutation_inverse` | 173.89 ns | 172.71 ns - 175.26 ns | 171.17 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_signed_permutation_transform` | 61.76 ns | 61.40 ns - 62.16 ns | 61.05 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_translation_div_matrix` | 553.60 ns | 548.34 ns - 559.48 ns | 544.38 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_translation_inverse` | 172.64 ns | 171.75 ns - 173.68 ns | 170.65 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_uniform_diagonal_div_vector` | 939.80 ns | 934.66 ns - 945.81 ns | 930.36 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_uniform_diagonal_div_vector_direction` | 555.82 ns | 554.83 ns - 557.04 ns | 554.50 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_uniform_diagonal_div_vector_point` | 1.02 us | 1.02 us - 1.03 us | 1.02 us | - | - |
| `matrix_ops/hyperreal/mat4 known_uniform_scale_inverse` | 141.19 ns | 140.76 ns - 141.66 ns | 140.24 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_upper_triangular_div_matrix` | 2.90 us | 2.89 us - 2.91 us | 2.88 us | - | - |
| `matrix_ops/hyperreal/mat4 known_upper_triangular_inverse` | 1.01 us | 997.36 ns - 1.02 us | 988.19 ns | - | - |
| `matrix_ops/hyperreal/mat4 known_upper_triangular_inverse_checked` | 1.02 us | 1.01 us - 1.03 us | 992.68 ns | - | - |
| `matrix_ops/hyperreal/mat4 mul_scalar` | 1.06 us | 1.06 us - 1.07 us | 1.06 us | - | - |
| `matrix_ops/hyperreal/mat4 neg` | 376.40 ns | 373.70 ns - 379.29 ns | 370.99 ns | - | - |
| `matrix_ops/hyperreal/mat4 powi` | 5.02 us | 5.00 us - 5.03 us | 5.02 us | - | - |
| `matrix_ops/hyperreal/mat4 powi_checked` | 4.98 us | 4.97 us - 5.00 us | 4.99 us | - | - |
| `matrix_ops/hyperreal/mat4 powi_checked_abort` | 4.99 us | 4.97 us - 5.01 us | 4.96 us | - | - |
| `matrix_ops/hyperreal/mat4 powi_checked_negative` | 30.82 us | 30.72 us - 30.94 us | 30.69 us | - | - |
| `matrix_ops/hyperreal/mat4 powi_negative` | 30.68 us | 30.60 us - 30.76 us | 30.54 us | - | - |
| `matrix_ops/hyperreal/mat4 powi_negative_one` | 7.18 us | 7.15 us - 7.22 us | 7.12 us | - | - |
| `matrix_ops/hyperreal/mat4 reciprocal` | 7.28 us | 7.23 us - 7.33 us | 7.19 us | - | - |
| `matrix_ops/hyperreal/mat4 reciprocal_checked` | 7.14 us | 7.12 us - 7.17 us | 7.10 us | - | - |
| `matrix_ops/hyperreal/mat4 sub` | 784.93 ns | 781.86 ns - 788.41 ns | 779.14 ns | - | - |
| `matrix_ops/hyperreal/mat4 sub_scalar` | 1.26 us | 1.25 us - 1.27 us | 1.25 us | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_direction_batch` | 2.49 us | 2.48 us - 2.49 us | 2.48 us | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_direction_batch_assumed` | 500.77 ns | 497.15 ns - 504.92 ns | 492.58 ns | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_direction_batch_public_assumed` | 493.97 ns | 492.49 ns - 495.74 ns | 491.33 ns | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_direction_transform` | 212.69 ns | 212.25 ns - 213.20 ns | 211.87 ns | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_direction_transform_generic` | 265.43 ns | 264.79 ns - 266.16 ns | 264.62 ns | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_point_batch` | 2.89 us | 2.87 us - 2.90 us | 2.86 us | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_point_batch_assumed` | 2.68 us | 2.67 us - 2.70 us | 2.66 us | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_point_batch_public_assumed` | 2.66 us | 2.64 us - 2.67 us | 2.63 us | - | - |
| `matrix_ops/hyperreal/mat4 translated_diagonal_point_transform_generic` | 343.09 ns | 339.80 ns - 346.77 ns | 335.56 ns | - | - |
| `matrix_ops/hyperreal/mat4 transpose` | 244.32 ns | 243.13 ns - 245.65 ns | 242.19 ns | - | - |
| `matrix_ops/hyperreal/mat4 uniform_scale_reciprocal` | 2.04 us | 2.01 us - 2.07 us | 2.00 us | - | - |
| `matrix_ops/hyperreal/mat4 zero` | 213.63 ns | 213.10 ns - 214.23 ns | 212.92 ns | - | - |
| `matrix_ops/numerica128/mat3 add` | 497.02 ns | 495.74 ns - 498.45 ns | 494.57 ns | - | - |
| `matrix_ops/numerica128/mat3 add_scalar` | 730.61 ns | 728.06 ns - 733.63 ns | 726.29 ns | - | - |
| `matrix_ops/numerica128/mat3 bitxor` | 6.23 us | 6.21 us - 6.25 us | 6.19 us | - | - |
| `matrix_ops/numerica128/mat3 div_matrix` | 4.40 us | 4.40 us - 4.41 us | 4.38 us | - | - |
| `matrix_ops/numerica128/mat3 div_matrix_checked` | 4.42 us | 4.41 us - 4.44 us | 4.40 us | - | - |
| `matrix_ops/numerica128/mat3 div_matrix_checked_abort` | 4.41 us | 4.39 us - 4.44 us | 4.39 us | - | - |
| `matrix_ops/numerica128/mat3 div_scalar` | 823.52 ns | 821.35 ns - 825.90 ns | 819.41 ns | - | - |
| `matrix_ops/numerica128/mat3 div_scalar_checked` | 830.88 ns | 827.52 ns - 834.54 ns | 823.84 ns | - | - |
| `matrix_ops/numerica128/mat3 div_scalar_checked_abort` | 825.02 ns | 822.41 ns - 827.90 ns | 818.67 ns | - | - |
| `matrix_ops/numerica128/mat3 identity` | 258.78 ns | 258.40 ns - 259.27 ns | 258.22 ns | - | - |
| `matrix_ops/numerica128/mat3 inverse_checked` | 2.29 us | 2.28 us - 2.29 us | 2.28 us | - | - |
| `matrix_ops/numerica128/mat3 inverse_checked_abort` | 2.29 us | 2.28 us - 2.30 us | 2.28 us | - | - |
| `matrix_ops/numerica128/mat3 mul_scalar` | 681.40 ns | 680.21 ns - 682.79 ns | 679.46 ns | - | - |
| `matrix_ops/numerica128/mat3 neg` | 477.48 ns | 476.40 ns - 478.70 ns | 475.83 ns | - | - |
| `matrix_ops/numerica128/mat3 new` | 243.12 ns | 242.46 ns - 243.86 ns | 242.09 ns | - | - |
| `matrix_ops/numerica128/mat3 powi` | 6.18 us | 6.17 us - 6.20 us | 6.17 us | - | - |
| `matrix_ops/numerica128/mat3 powi_checked` | 6.22 us | 6.20 us - 6.25 us | 6.19 us | - | - |
| `matrix_ops/numerica128/mat3 powi_checked_abort` | 6.19 us | 6.17 us - 6.21 us | 6.17 us | - | - |
| `matrix_ops/numerica128/mat3 reciprocal` | 2.30 us | 2.29 us - 2.31 us | 2.28 us | - | - |
| `matrix_ops/numerica128/mat3 reciprocal_checked` | 2.29 us | 2.28 us - 2.29 us | 2.27 us | - | - |
| `matrix_ops/numerica128/mat3 sub` | 524.40 ns | 522.88 ns - 526.07 ns | 520.72 ns | - | - |
| `matrix_ops/numerica128/mat3 sub_scalar` | 721.73 ns | 719.88 ns - 723.74 ns | 718.53 ns | - | - |
| `matrix_ops/numerica128/mat3 transpose` | 201.68 ns | 201.33 ns - 202.10 ns | 201.25 ns | - | - |
| `matrix_ops/numerica128/mat3 zero` | 211.53 ns | 211.03 ns - 212.12 ns | 210.79 ns | - | - |
| `matrix_ops/numerica128/mat4 add` | 830.86 ns | 826.07 ns - 836.21 ns | 821.00 ns | - | - |
| `matrix_ops/numerica128/mat4 add_scalar` | 1.18 us | 1.18 us - 1.19 us | 1.18 us | - | - |
| `matrix_ops/numerica128/mat4 bitxor` | 13.83 us | 13.79 us - 13.88 us | 13.74 us | - | - |
| `matrix_ops/numerica128/mat4 div_matrix` | 14.52 us | 14.36 us - 14.72 us | 14.17 us | - | - |
| `matrix_ops/numerica128/mat4 div_scalar` | 1.42 us | 1.41 us - 1.43 us | 1.40 us | - | - |
| `matrix_ops/numerica128/mat4 identity` | 385.05 ns | 384.11 ns - 386.15 ns | 383.12 ns | - | - |
| `matrix_ops/numerica128/mat4 mul_scalar` | 1.11 us | 1.11 us - 1.11 us | 1.11 us | - | - |
| `matrix_ops/numerica128/mat4 neg` | 764.23 ns | 761.96 ns - 766.66 ns | 759.64 ns | - | - |
| `matrix_ops/numerica128/mat4 powi` | 13.73 us | 13.70 us - 13.77 us | 13.69 us | - | - |
| `matrix_ops/numerica128/mat4 powi_checked` | 13.84 us | 13.79 us - 13.90 us | 13.73 us | - | - |
| `matrix_ops/numerica128/mat4 reciprocal` | 8.91 us | 8.84 us - 8.99 us | 8.80 us | - | - |
| `matrix_ops/numerica128/mat4 reciprocal_checked` | 8.87 us | 8.84 us - 8.91 us | 8.81 us | - | - |
| `matrix_ops/numerica128/mat4 sub` | 879.12 ns | 876.37 ns - 882.21 ns | 873.06 ns | - | - |
| `matrix_ops/numerica128/mat4 sub_scalar` | 1.19 us | 1.18 us - 1.19 us | 1.17 us | - | - |
| `matrix_ops/numerica128/mat4 transpose` | 338.02 ns | 336.88 ns - 339.28 ns | 335.73 ns | - | - |
| `matrix_ops/numerica128/mat4 zero` | 327.13 ns | 324.91 ns - 329.76 ns | 323.11 ns | - | - |
| `matrix_ops/symbolica/mat3 add` | 15.46 us | 15.45 us - 15.47 us | 15.45 us | - | - |
| `matrix_ops/symbolica/mat3 add_scalar` | 15.90 us | 15.88 us - 15.92 us | 15.86 us | - | - |
| `matrix_ops/symbolica/mat3 bitxor` | 205.42 us | 204.97 us - 205.93 us | 204.75 us | - | - |
| `matrix_ops/symbolica/mat3 div_matrix` | 199.04 us | 198.64 us - 199.47 us | 198.56 us | - | - |
| `matrix_ops/symbolica/mat3 div_matrix_checked` | 199.43 us | 199.13 us - 199.77 us | 199.50 us | - | - |
| `matrix_ops/symbolica/mat3 div_matrix_checked_abort` | 200.88 us | 200.32 us - 201.84 us | 200.21 us | - | - |
| `matrix_ops/symbolica/mat3 div_scalar` | 26.26 us | 26.22 us - 26.31 us | 26.20 us | - | - |
| `matrix_ops/symbolica/mat3 div_scalar_checked` | 26.42 us | 26.36 us - 26.49 us | 26.30 us | - | - |
| `matrix_ops/symbolica/mat3 div_scalar_checked_abort` | 26.49 us | 26.40 us - 26.60 us | 26.35 us | - | - |
| `matrix_ops/symbolica/mat3 identity` | 174.65 ns | 174.35 ns - 174.99 ns | 174.18 ns | - | - |
| `matrix_ops/symbolica/mat3 inverse_checked` | 105.74 us | 105.24 us - 106.29 us | 104.68 us | - | - |
| `matrix_ops/symbolica/mat3 inverse_checked_abort` | 105.34 us | 104.81 us - 105.95 us | 104.40 us | - | - |
| `matrix_ops/symbolica/mat3 mul_scalar` | 16.00 us | 15.99 us - 16.02 us | 15.98 us | - | - |
| `matrix_ops/symbolica/mat3 neg` | 12.48 us | 12.39 us - 12.61 us | 12.35 us | - | - |
| `matrix_ops/symbolica/mat3 new` | 2.14 us | 2.12 us - 2.15 us | 2.10 us | - | - |
| `matrix_ops/symbolica/mat3 powi` | 206.83 us | 206.39 us - 207.32 us | 206.07 us | - | - |
| `matrix_ops/symbolica/mat3 powi_checked` | 205.39 us | 204.95 us - 205.87 us | 204.79 us | - | - |
| `matrix_ops/symbolica/mat3 powi_checked_abort` | 204.99 us | 204.44 us - 205.64 us | 204.36 us | - | - |
| `matrix_ops/symbolica/mat3 reciprocal` | 104.38 us | 104.16 us - 104.61 us | 104.07 us | - | - |
| `matrix_ops/symbolica/mat3 reciprocal_checked` | 104.26 us | 103.89 us - 104.81 us | 103.69 us | - | - |
| `matrix_ops/symbolica/mat3 sub` | 25.17 us | 25.16 us - 25.19 us | 25.17 us | - | - |
| `matrix_ops/symbolica/mat3 sub_scalar` | 25.66 us | 25.64 us - 25.67 us | 25.64 us | - | - |
| `matrix_ops/symbolica/mat3 transpose` | 109.84 ns | 109.67 ns - 110.03 ns | 109.73 ns | - | - |
| `matrix_ops/symbolica/mat3 zero` | 21.05 ns | 21.02 ns - 21.09 ns | 21.03 ns | - | - |
| `matrix_ops/symbolica/mat4 add` | 26.23 us | 26.21 us - 26.25 us | 26.21 us | - | - |
| `matrix_ops/symbolica/mat4 add_scalar` | 27.33 us | 27.26 us - 27.42 us | 27.21 us | - | - |
| `matrix_ops/symbolica/mat4 bitxor` | 482.48 us | 481.30 us - 483.64 us | 483.47 us | - | - |
| `matrix_ops/symbolica/mat4 div_matrix` | 676.03 us | 674.06 us - 677.54 us | 677.17 us | - | - |
| `matrix_ops/symbolica/mat4 div_scalar` | 45.37 us | 45.32 us - 45.43 us | 45.35 us | - | - |
| `matrix_ops/symbolica/mat4 identity` | 246.29 ns | 245.93 ns - 246.74 ns | 245.92 ns | - | - |
| `matrix_ops/symbolica/mat4 mul_scalar` | 27.06 us | 27.04 us - 27.09 us | 27.04 us | - | - |
| `matrix_ops/symbolica/mat4 neg` | 20.78 us | 20.75 us - 20.83 us | 20.74 us | - | - |
| `matrix_ops/symbolica/mat4 powi` | 480.69 us | 479.56 us - 481.71 us | 481.54 us | - | - |
| `matrix_ops/symbolica/mat4 powi_checked` | 482.69 us | 481.47 us - 484.03 us | 482.84 us | - | - |
| `matrix_ops/symbolica/mat4 reciprocal` | 437.10 us | 436.49 us - 437.69 us | 437.20 us | - | - |
| `matrix_ops/symbolica/mat4 reciprocal_checked` | 437.41 us | 436.34 us - 438.53 us | 436.76 us | - | - |
| `matrix_ops/symbolica/mat4 sub` | 42.88 us | 42.83 us - 42.93 us | 42.84 us | - | - |
| `matrix_ops/symbolica/mat4 sub_scalar` | 45.10 us | 45.05 us - 45.17 us | 45.12 us | - | - |
| `matrix_ops/symbolica/mat4 transpose` | 162.19 ns | 161.44 ns - 163.09 ns | 162.15 ns | - | - |
| `matrix_ops/symbolica/mat4 zero` | 14.63 ns | 14.61 ns - 14.65 ns | 14.62 ns | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_1014` | 4.00 us | 3.87 us - 4.14 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_1022` | 4.04 us | 3.99 us - 4.08 us | 4.05 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_1749` | 3.95 us | 3.89 us - 4.05 us | 3.89 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2185` | 3.75 us | 3.73 us - 3.78 us | 3.75 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2186` | 3.84 us | 3.82 us - 3.86 us | 3.83 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2196` | 4.06 us | 3.93 us - 4.24 us | 4.03 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2225` | 4.25 us | 3.95 us - 4.60 us | 3.92 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2234` | 3.88 us | 3.85 us - 3.92 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2247` | 3.96 us | 3.91 us - 4.02 us | 3.93 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2258` | 3.78 us | 3.72 us - 3.90 us | 3.73 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2277` | 3.97 us | 3.93 us - 4.04 us | 3.95 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2283` | 4.32 us | 3.91 us - 4.81 us | 3.86 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2287` | 3.86 us | 3.85 us - 3.88 us | 3.85 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2289` | 3.85 us | 3.80 us - 3.90 us | 3.82 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2293` | 4.20 us | 3.89 us - 4.65 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2310` | 3.89 us | 3.87 us - 3.90 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2321` | 3.73 us | 3.72 us - 3.75 us | 3.73 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2323` | 3.84 us | 3.82 us - 3.86 us | 3.83 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2324` | 3.94 us | 3.77 us - 4.20 us | 3.78 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2328` | 4.01 us | 3.98 us - 4.03 us | 4.00 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2329` | 3.95 us | 3.91 us - 3.98 us | 3.92 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2332` | 3.90 us | 3.86 us - 3.95 us | 3.90 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2335` | 3.91 us | 3.86 us - 3.97 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2346` | 3.85 us | 3.83 us - 3.86 us | 3.85 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2369` | 3.93 us | 3.90 us - 3.95 us | 3.93 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2374` | 3.87 us | 3.85 us - 3.90 us | 3.86 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2381` | 4.05 us | 3.93 us - 4.20 us | 3.95 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2418` | 4.06 us | 3.88 us - 4.28 us | 3.89 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2419` | 3.94 us | 3.91 us - 3.96 us | 3.94 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2422` | 4.03 us | 3.82 us - 4.29 us | 3.84 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2430` | 3.94 us | 3.80 us - 4.13 us | 3.80 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2454` | 3.85 us | 3.79 us - 3.96 us | 3.80 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2489` | 3.88 us | 3.87 us - 3.89 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2492` | 4.04 us | 3.87 us - 4.22 us | 3.86 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2521` | 3.87 us | 3.85 us - 3.90 us | 3.86 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2881` | 3.85 us | 3.83 us - 3.87 us | 3.84 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2907` | 3.97 us | 3.87 us - 4.11 us | 3.89 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2913` | 3.86 us | 3.81 us - 3.92 us | 3.81 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2921` | 3.87 us | 3.86 us - 3.89 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2946` | 3.90 us | 3.89 us - 3.92 us | 3.90 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2953` | 4.19 us | 4.01 us - 4.40 us | 4.04 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2955` | 4.26 us | 4.03 us - 4.52 us | 4.15 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2956` | 3.87 us | 3.85 us - 3.90 us | 3.86 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2958` | 4.05 us | 3.86 us - 4.29 us | 3.95 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2959` | 3.78 us | 3.76 us - 3.80 us | 3.76 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2960` | 3.92 us | 3.89 us - 3.94 us | 3.92 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2962` | 3.82 us | 3.81 us - 3.84 us | 3.82 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2972` | 3.97 us | 3.84 us - 4.12 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2974` | 4.17 us | 4.10 us - 4.24 us | 4.17 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_2994` | 3.93 us | 3.83 us - 4.06 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3011` | 3.98 us | 3.92 us - 4.04 us | 3.97 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3017` | 3.99 us | 3.83 us - 4.19 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3033` | 3.85 us | 3.84 us - 3.86 us | 3.86 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3035` | 3.76 us | 3.74 us - 3.77 us | 3.76 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3038` | 3.71 us | 3.70 us - 3.73 us | 3.72 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3039` | 3.87 us | 3.82 us - 3.92 us | 3.85 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3048` | 4.61 us | 4.38 us - 4.86 us | 4.48 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3087` | 3.90 us | 3.86 us - 3.96 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3095` | 3.76 us | 3.75 us - 3.78 us | 3.75 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3100` | 4.22 us | 3.98 us - 4.51 us | 4.01 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3141` | 3.99 us | 3.86 us - 4.15 us | 3.91 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3194` | 3.91 us | 3.87 us - 3.96 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3199` | 4.05 us | 4.01 us - 4.09 us | 4.04 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3203` | 3.90 us | 3.84 us - 3.97 us | 3.84 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3214` | 4.02 us | 3.93 us - 4.20 us | 3.93 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3219` | 3.97 us | 3.86 us - 4.08 us | 3.89 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3225` | 3.94 us | 3.90 us - 3.98 us | 3.92 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3227` | 4.20 us | 3.99 us - 4.47 us | 3.97 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3234` | 4.17 us | 4.00 us - 4.37 us | 4.09 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3381` | 3.91 us | 3.89 us - 3.95 us | 3.90 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3566` | 3.94 us | 3.89 us - 3.99 us | 3.91 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3607` | 4.13 us | 3.92 us - 4.37 us | 4.06 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3614` | 3.82 us | 3.80 us - 3.83 us | 3.82 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3617` | 4.10 us | 3.94 us - 4.30 us | 3.94 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3639` | 3.90 us | 3.80 us - 4.02 us | 3.81 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3641` | 4.06 us | 3.91 us - 4.26 us | 3.92 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3687` | 3.87 us | 3.83 us - 3.91 us | 3.84 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3717` | 3.96 us | 3.94 us - 3.99 us | 3.96 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3740` | 4.07 us | 3.98 us - 4.18 us | 4.03 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3742` | 3.87 us | 3.78 us - 4.01 us | 3.83 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3745` | 3.92 us | 3.86 us - 4.01 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3754` | 3.77 us | 3.76 us - 3.78 us | 3.77 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3774` | 3.88 us | 3.78 us - 4.00 us | 3.82 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3779` | 4.17 us | 3.96 us - 4.42 us | 4.13 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3787` | 3.85 us | 3.83 us - 3.86 us | 3.83 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3799` | 4.04 us | 3.89 us - 4.20 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3813` | 3.94 us | 3.86 us - 4.04 us | 3.85 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3814` | 3.89 us | 3.86 us - 3.94 us | 3.88 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3867` | 3.97 us | 3.88 us - 4.10 us | 3.92 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3874` | 3.91 us | 3.89 us - 3.92 us | 3.90 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3881` | 4.03 us | 3.96 us - 4.14 us | 3.97 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3883` | 3.89 us | 3.83 us - 3.96 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3887` | 4.24 us | 3.99 us - 4.51 us | 4.18 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3889` | 3.78 us | 3.76 us - 3.81 us | 3.77 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3894` | 4.15 us | 3.96 us - 4.43 us | 4.00 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_39` | 3.86 us | 3.84 us - 3.88 us | 3.87 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3908` | 4.07 us | 3.98 us - 4.19 us | 3.99 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_3919` | 4.04 us | 3.92 us - 4.21 us | 3.93 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_40` | 3.67 us | 3.66 us - 3.68 us | 3.67 us | - | - |
| `promoted_fuzz_worst_performers/matrix_ops_seed_804` | 3.97 us | 3.92 us - 4.05 us | 3.92 us | - | - |
| `promoted_slow_offender_score/replay_promoted_100` | 481.80 us | 457.29 us - 513.65 us | 472.44 us | - | - |
| `real_representations/ConstOffset` | 4.72 us | 4.64 us - 4.81 us | 4.62 us | - | - |
| `real_representations/ConstProduct` | 5.15 us | 5.08 us - 5.24 us | 5.08 us | - | - |
| `real_representations/ConstProductSqrt` | 5.68 us | 5.53 us - 5.90 us | 5.55 us | - | - |
| `real_representations/Exp` | 3.72 us | 3.67 us - 3.77 us | 3.66 us | - | - |
| `real_representations/Irrational` | 2.17 us | 2.16 us - 2.18 us | 2.17 us | - | - |
| `real_representations/Ln` | 3.00 us | 2.98 us - 3.03 us | 2.98 us | - | - |
| `real_representations/LnAffine` | 3.34 us | 3.32 us - 3.38 us | 3.32 us | - | - |
| `real_representations/LnProduct` | 2.33 us | 2.32 us - 2.34 us | 2.32 us | - | - |
| `real_representations/Log10` | 2.23 us | 2.22 us - 2.24 us | 2.23 us | - | - |
| `real_representations/Log2` | 2.21 us | 2.21 us - 2.22 us | 2.21 us | - | - |
| `real_representations/One` | 1.04 us | 1.03 us - 1.05 us | 1.03 us | - | - |
| `real_representations/Pi` | 4.11 us | 4.09 us - 4.13 us | 4.09 us | - | - |
| `real_representations/PiExp` | 4.54 us | 4.52 us - 4.56 us | 4.53 us | - | - |
| `real_representations/PiInv` | 3.35 us | 3.29 us - 3.43 us | 3.29 us | - | - |
| `real_representations/PiInvExp` | 4.06 us | 4.04 us - 4.10 us | 4.04 us | - | - |
| `real_representations/PiPow` | 3.53 us | 3.52 us - 3.56 us | 3.51 us | - | - |
| `real_representations/PiSqrt` | 3.70 us | 3.65 us - 3.77 us | 3.65 us | - | - |
| `real_representations/Pow10` | 2.26 us | 2.20 us - 2.35 us | 2.19 us | - | - |
| `real_representations/Pow2` | 2.23 us | 2.21 us - 2.26 us | 2.21 us | - | - |
| `real_representations/SinPi` | 2.48 us | 2.45 us - 2.51 us | 2.45 us | - | - |
| `real_representations/Sqrt` | 1.52 us | 1.50 us - 1.54 us | 1.50 us | - | - |
| `real_representations/TanPi` | 2.24 us | 2.23 us - 2.26 us | 2.23 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/cosh/half` | 1.14 us | 1.13 us - 1.14 us | 1.13 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/cosh/negative_20` | 1.23 us | 1.23 us - 1.23 us | 1.23 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/cosh/negative_tiny` | 623.91 ns | 622.99 ns - 624.97 ns | 622.73 ns | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/cosh/positive_20` | 1.23 us | 1.23 us - 1.23 us | 1.23 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/sinh/half` | 1.13 us | 1.13 us - 1.13 us | 1.13 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/sinh/negative_20` | 1.22 us | 1.22 us - 1.23 us | 1.22 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/sinh/negative_tiny` | 925.79 ns | 923.14 ns - 928.83 ns | 920.69 ns | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/sinh/positive_20` | 1.22 us | 1.22 us - 1.22 us | 1.22 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/tanh/half` | 1.19 us | 1.19 us - 1.19 us | 1.19 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/tanh/negative_20` | 1.29 us | 1.29 us - 1.29 us | 1.29 us | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/tanh/negative_tiny` | 832.86 ns | 830.38 ns - 836.77 ns | 830.34 ns | - | - |
| `scalar_hyperbolic_cases/gmp_mpfr128/tanh/positive_20` | 1.29 us | 1.29 us - 1.29 us | 1.29 us | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/cosh/half` | 360.62 ns | 359.68 ns - 361.67 ns | 358.83 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/cosh/negative_20` | 869.65 ns | 868.61 ns - 870.92 ns | 868.33 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/cosh/negative_tiny` | 383.56 ns | 383.01 ns - 384.13 ns | 382.41 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/cosh/positive_20` | 818.01 ns | 817.26 ns - 818.81 ns | 817.26 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/sinh/half` | 404.46 ns | 403.91 ns - 405.23 ns | 403.76 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/sinh/negative_20` | 807.65 ns | 806.67 ns - 808.73 ns | 806.85 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/sinh/negative_tiny` | 402.52 ns | 401.67 ns - 403.56 ns | 401.74 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/sinh/positive_20` | 729.79 ns | 729.25 ns - 730.35 ns | 729.30 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/tanh/half` | 548.19 ns | 547.79 ns - 548.61 ns | 547.77 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/tanh/negative_20` | 652.55 ns | 650.77 ns - 654.55 ns | 649.23 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/tanh/negative_tiny` | 552.94 ns | 551.81 ns - 554.35 ns | 552.45 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal-rational/tanh/positive_20` | 583.80 ns | 582.58 ns - 585.43 ns | 582.45 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/cosh/half` | 361.59 ns | 359.85 ns - 363.84 ns | 358.43 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/cosh/negative_20` | 876.16 ns | 873.81 ns - 878.83 ns | 872.46 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/cosh/negative_tiny` | 349.70 ns | 348.96 ns - 350.53 ns | 349.07 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/cosh/positive_20` | 822.05 ns | 819.93 ns - 825.10 ns | 819.46 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/sinh/half` | 405.53 ns | 405.18 ns - 405.93 ns | 405.28 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/sinh/negative_20` | 808.12 ns | 807.10 ns - 809.28 ns | 807.45 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/sinh/negative_tiny` | 367.02 ns | 366.46 ns - 367.65 ns | 366.23 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/sinh/positive_20` | 730.10 ns | 729.44 ns - 730.83 ns | 729.62 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/tanh/half` | 548.78 ns | 545.83 ns - 552.64 ns | 543.98 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/tanh/negative_20` | 647.97 ns | 646.74 ns - 649.37 ns | 646.18 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/tanh/negative_tiny` | 514.89 ns | 514.18 ns - 515.73 ns | 514.28 ns | - | - |
| `scalar_hyperbolic_cases/hyperreal/tanh/positive_20` | 583.08 ns | 581.84 ns - 584.50 ns | 581.27 ns | - | - |
| `scalar_hyperbolic_cases/numerica128/cosh/half` | 1.14 us | 1.13 us - 1.14 us | 1.14 us | - | - |
| `scalar_hyperbolic_cases/numerica128/cosh/negative_20` | 1.23 us | 1.22 us - 1.23 us | 1.22 us | - | - |
| `scalar_hyperbolic_cases/numerica128/cosh/negative_tiny` | 629.70 ns | 628.83 ns - 630.75 ns | 629.71 ns | - | - |
| `scalar_hyperbolic_cases/numerica128/cosh/positive_20` | 1.22 us | 1.22 us - 1.22 us | 1.22 us | - | - |
| `scalar_hyperbolic_cases/numerica128/sinh/half` | 1.13 us | 1.13 us - 1.13 us | 1.13 us | - | - |
| `scalar_hyperbolic_cases/numerica128/sinh/negative_20` | 1.22 us | 1.22 us - 1.22 us | 1.22 us | - | - |
| `scalar_hyperbolic_cases/numerica128/sinh/negative_tiny` | 912.26 ns | 911.88 ns - 912.63 ns | 912.40 ns | - | - |
| `scalar_hyperbolic_cases/numerica128/sinh/positive_20` | 1.21 us | 1.21 us - 1.21 us | 1.21 us | - | - |
| `scalar_hyperbolic_cases/numerica128/tanh/half` | 1.19 us | 1.19 us - 1.19 us | 1.19 us | - | - |
| `scalar_hyperbolic_cases/numerica128/tanh/negative_20` | 1.35 us | 1.35 us - 1.36 us | 1.35 us | - | - |
| `scalar_hyperbolic_cases/numerica128/tanh/negative_tiny` | 842.80 ns | 841.15 ns - 844.71 ns | 840.48 ns | - | - |
| `scalar_hyperbolic_cases/numerica128/tanh/positive_20` | 1.35 us | 1.34 us - 1.35 us | 1.35 us | - | - |
| `scalar_hyperbolic_cases/symbolica/cosh/half` | 12.01 us | 12.00 us - 12.03 us | 12.01 us | - | - |
| `scalar_hyperbolic_cases/symbolica/cosh/negative_20` | 11.94 us | 11.92 us - 11.96 us | 11.92 us | - | - |
| `scalar_hyperbolic_cases/symbolica/cosh/negative_tiny` | 12.36 us | 12.35 us - 12.37 us | 12.35 us | - | - |
| `scalar_hyperbolic_cases/symbolica/cosh/positive_20` | 12.05 us | 12.02 us - 12.09 us | 12.00 us | - | - |
| `scalar_hyperbolic_cases/symbolica/sinh/half` | 13.30 us | 13.28 us - 13.33 us | 13.26 us | - | - |
| `scalar_hyperbolic_cases/symbolica/sinh/negative_20` | 13.10 us | 13.08 us - 13.12 us | 13.08 us | - | - |
| `scalar_hyperbolic_cases/symbolica/sinh/negative_tiny` | 13.73 us | 13.70 us - 13.76 us | 13.71 us | - | - |
| `scalar_hyperbolic_cases/symbolica/sinh/positive_20` | 13.23 us | 13.22 us - 13.24 us | 13.23 us | - | - |
| `scalar_hyperbolic_cases/symbolica/tanh/half` | 28.73 us | 28.66 us - 28.82 us | 28.63 us | - | - |
| `scalar_hyperbolic_cases/symbolica/tanh/negative_20` | 28.29 us | 28.25 us - 28.33 us | 28.30 us | - | - |
| `scalar_hyperbolic_cases/symbolica/tanh/negative_tiny` | 28.41 us | 28.40 us - 28.44 us | 28.40 us | - | - |
| `scalar_hyperbolic_cases/symbolica/tanh/positive_20` | 28.54 us | 28.45 us - 28.65 us | 28.38 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/cosh/half` | 1.16 us | 1.16 us - 1.17 us | 1.16 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/cosh/negative_20` | 1.25 us | 1.25 us - 1.26 us | 1.25 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/cosh/negative_tiny` | 652.79 ns | 649.51 ns - 656.35 ns | 645.73 ns | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/cosh/positive_20` | 1.24 us | 1.24 us - 1.25 us | 1.24 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/sinh/half` | 1.15 us | 1.15 us - 1.16 us | 1.15 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/sinh/negative_20` | 1.24 us | 1.24 us - 1.24 us | 1.23 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/sinh/negative_tiny` | 947.76 ns | 942.12 ns - 954.81 ns | 937.69 ns | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/sinh/positive_20` | 1.26 us | 1.25 us - 1.27 us | 1.24 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/tanh/half` | 1.23 us | 1.22 us - 1.23 us | 1.22 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/tanh/negative_20` | 1.30 us | 1.29 us - 1.30 us | 1.29 us | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/tanh/negative_tiny` | 861.08 ns | 858.67 ns - 863.90 ns | 857.21 ns | - | - |
| `scalar_hyperbolic_f64_cases/gmp_mpfr128/tanh/positive_20` | 1.29 us | 1.29 us - 1.29 us | 1.29 us | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/cosh/half` | 355.58 ns | 354.57 ns - 356.83 ns | 353.77 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/cosh/negative_20` | 870.12 ns | 868.60 ns - 871.83 ns | 868.49 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/cosh/negative_tiny` | 388.65 ns | 387.99 ns - 389.42 ns | 387.80 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/cosh/positive_20` | 819.72 ns | 817.38 ns - 822.44 ns | 816.75 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/sinh/half` | 406.35 ns | 405.11 ns - 407.97 ns | 404.41 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/sinh/negative_20` | 816.79 ns | 814.98 ns - 818.95 ns | 813.78 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/sinh/negative_tiny` | 407.45 ns | 406.95 ns - 408.07 ns | 406.70 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/sinh/positive_20` | 743.81 ns | 737.76 ns - 750.79 ns | 732.61 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/tanh/half` | 553.21 ns | 550.63 ns - 556.23 ns | 547.85 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/tanh/negative_20` | 653.48 ns | 652.31 ns - 654.75 ns | 652.53 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/tanh/negative_tiny` | 560.76 ns | 558.33 ns - 564.06 ns | 557.39 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal-rational/tanh/positive_20` | 583.08 ns | 580.30 ns - 586.16 ns | 577.81 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/cosh/half` | 354.87 ns | 353.73 ns - 356.19 ns | 352.65 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/cosh/negative_20` | 880.88 ns | 879.55 ns - 882.46 ns | 878.92 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/cosh/negative_tiny` | 351.61 ns | 349.33 ns - 354.12 ns | 347.47 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/cosh/positive_20` | 831.03 ns | 824.86 ns - 838.46 ns | 821.13 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/sinh/half` | 403.12 ns | 402.38 ns - 404.24 ns | 402.51 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/sinh/negative_20` | 820.84 ns | 819.50 ns - 822.35 ns | 818.47 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/sinh/negative_tiny` | 367.70 ns | 365.94 ns - 369.66 ns | 364.25 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/sinh/positive_20` | 726.34 ns | 725.36 ns - 727.48 ns | 725.49 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/tanh/half` | 556.78 ns | 555.96 ns - 557.70 ns | 556.59 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/tanh/negative_20` | 662.45 ns | 660.38 ns - 665.01 ns | 659.68 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/tanh/negative_tiny` | 532.85 ns | 529.89 ns - 536.27 ns | 527.88 ns | - | - |
| `scalar_hyperbolic_f64_cases/hyperreal/tanh/positive_20` | 578.18 ns | 576.12 ns - 580.61 ns | 575.03 ns | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/cosh/half` | 1.17 us | 1.16 us - 1.17 us | 1.16 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/cosh/negative_20` | 1.24 us | 1.24 us - 1.25 us | 1.24 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/cosh/negative_tiny` | 641.45 ns | 639.53 ns - 643.60 ns | 637.97 ns | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/cosh/positive_20` | 1.24 us | 1.24 us - 1.24 us | 1.24 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/sinh/half` | 1.16 us | 1.16 us - 1.17 us | 1.16 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/sinh/negative_20` | 1.24 us | 1.24 us - 1.24 us | 1.24 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/sinh/negative_tiny` | 930.83 ns | 930.09 ns - 931.65 ns | 930.94 ns | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/sinh/positive_20` | 1.26 us | 1.26 us - 1.27 us | 1.25 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/tanh/half` | 1.25 us | 1.24 us - 1.27 us | 1.23 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/tanh/negative_20` | 1.37 us | 1.36 us - 1.37 us | 1.36 us | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/tanh/negative_tiny` | 859.39 ns | 858.40 ns - 860.52 ns | 859.08 ns | - | - |
| `scalar_hyperbolic_f64_cases/numerica128/tanh/positive_20` | 1.37 us | 1.37 us - 1.38 us | 1.36 us | - | - |
| `scalar_large_integer_exp/gmp_mpfr128/exp_128` | 1.03 us | 1.03 us - 1.04 us | 1.03 us | - | - |
| `scalar_large_integer_exp/hyperreal-rational/exp_128` | 295.63 ns | 294.62 ns - 296.74 ns | 292.93 ns | - | - |
| `scalar_large_integer_exp/hyperreal/exp_128` | 296.96 ns | 296.02 ns - 298.10 ns | 295.26 ns | - | - |
| `scalar_large_integer_exp/numerica128/exp_128` | 1.06 us | 1.05 us - 1.06 us | 1.05 us | - | - |
| `scalar_large_integer_exp/symbolica/exp_128` | 2.63 us | 2.62 us - 2.65 us | 2.60 us | - | - |
| `scalar_ops/gmp_mpfr128/acos` | 2.56 us | 2.56 us - 2.57 us | 2.56 us | - | - |
| `scalar_ops/gmp_mpfr128/acos_abort` | 2.59 us | 2.58 us - 2.60 us | 2.57 us | - | - |
| `scalar_ops/gmp_mpfr128/acosh` | 3.34 us | 3.32 us - 3.37 us | 3.30 us | - | - |
| `scalar_ops/gmp_mpfr128/acosh_abort` | 3.34 us | 3.32 us - 3.35 us | 3.30 us | - | - |
| `scalar_ops/gmp_mpfr128/add` | 31.52 ns | 31.45 ns - 31.61 ns | 31.36 ns | - | - |
| `scalar_ops/gmp_mpfr128/asin` | 2.46 us | 2.46 us - 2.47 us | 2.45 us | - | - |
| `scalar_ops/gmp_mpfr128/asin_abort` | 2.48 us | 2.47 us - 2.49 us | 2.46 us | - | - |
| `scalar_ops/gmp_mpfr128/asinh` | 1.63 us | 1.62 us - 1.63 us | 1.62 us | - | - |
| `scalar_ops/gmp_mpfr128/asinh_abort` | 1.64 us | 1.63 us - 1.65 us | 1.62 us | - | - |
| `scalar_ops/gmp_mpfr128/atan` | 2.25 us | 2.24 us - 2.27 us | 2.23 us | - | - |
| `scalar_ops/gmp_mpfr128/atan_abort` | 2.29 us | 2.27 us - 2.32 us | 2.24 us | - | - |
| `scalar_ops/gmp_mpfr128/atanh` | 1.27 us | 1.26 us - 1.27 us | 1.26 us | - | - |
| `scalar_ops/gmp_mpfr128/atanh_abort` | 1.35 us | 1.33 us - 1.37 us | 1.31 us | - | - |
| `scalar_ops/gmp_mpfr128/cos` | 631.31 ns | 629.48 ns - 633.49 ns | 627.68 ns | - | - |
| `scalar_ops/gmp_mpfr128/cosh` | 1.08 us | 1.07 us - 1.08 us | 1.07 us | - | - |
| `scalar_ops/gmp_mpfr128/div` | 60.34 ns | 60.08 ns - 60.67 ns | 59.91 ns | - | - |
| `scalar_ops/gmp_mpfr128/e` | 1.03 us | 1.03 us - 1.04 us | 1.02 us | - | - |
| `scalar_ops/gmp_mpfr128/exp` | 891.44 ns | 887.37 ns - 895.92 ns | 882.13 ns | - | - |
| `scalar_ops/gmp_mpfr128/ln` | 1.33 us | 1.33 us - 1.34 us | 1.33 us | - | - |
| `scalar_ops/gmp_mpfr128/log10` | 3.86 us | 3.85 us - 3.87 us | 3.85 us | - | - |
| `scalar_ops/gmp_mpfr128/log10_abort` | 3.94 us | 3.92 us - 3.96 us | 3.90 us | - | - |
| `scalar_ops/gmp_mpfr128/mul` | 42.27 ns | 42.13 ns - 42.44 ns | 42.02 ns | - | - |
| `scalar_ops/gmp_mpfr128/neg` | 21.62 ns | 21.62 ns - 21.63 ns | 21.62 ns | - | - |
| `scalar_ops/gmp_mpfr128/one` | 21.66 ns | 21.57 ns - 21.77 ns | 21.50 ns | - | - |
| `scalar_ops/gmp_mpfr128/pi` | 20.24 ns | 20.17 ns - 20.33 ns | 20.14 ns | - | - |
| `scalar_ops/gmp_mpfr128/pow` | 2.81 us | 2.80 us - 2.82 us | 2.79 us | - | - |
| `scalar_ops/gmp_mpfr128/powi` | 90.74 ns | 90.56 ns - 90.97 ns | 90.41 ns | - | - |
| `scalar_ops/gmp_mpfr128/powi_negative_one` | 59.86 ns | 59.51 ns - 60.26 ns | 59.08 ns | - | - |
| `scalar_ops/gmp_mpfr128/reciprocal` | 59.40 ns | 59.26 ns - 59.56 ns | 59.08 ns | - | - |
| `scalar_ops/gmp_mpfr128/reciprocal_checked` | 60.34 ns | 59.92 ns - 60.82 ns | 59.42 ns | - | - |
| `scalar_ops/gmp_mpfr128/reciprocal_checked_abort` | 59.25 ns | 59.10 ns - 59.46 ns | 59.03 ns | - | - |
| `scalar_ops/gmp_mpfr128/sin` | 1.27 us | 1.26 us - 1.27 us | 1.26 us | - | - |
| `scalar_ops/gmp_mpfr128/sinh` | 1.17 us | 1.16 us - 1.18 us | 1.15 us | - | - |
| `scalar_ops/gmp_mpfr128/sqrt` | 110.36 ns | 109.51 ns - 111.30 ns | 108.39 ns | - | - |
| `scalar_ops/gmp_mpfr128/sub` | 31.49 ns | 31.45 ns - 31.54 ns | 31.44 ns | - | - |
| `scalar_ops/gmp_mpfr128/tan` | 1.59 us | 1.59 us - 1.60 us | 1.58 us | - | - |
| `scalar_ops/gmp_mpfr128/tanh` | 1.19 us | 1.18 us - 1.19 us | 1.18 us | - | - |
| `scalar_ops/gmp_mpfr128/tau` | 68.98 ns | 68.85 ns - 69.12 ns | 68.89 ns | - | - |
| `scalar_ops/gmp_mpfr128/zero` | 9.02 ns | 8.99 ns - 9.06 ns | 8.97 ns | - | - |
| `scalar_ops/gmp_mpfr128/zero_status` | 1.29 ns | 1.28 ns - 1.30 ns | 1.28 ns | - | - |
| `scalar_ops/gmp_mpfr128/zero_status_abort` | 1.31 ns | 1.30 ns - 1.32 ns | 1.30 ns | - | - |
| `scalar_ops/hyperreal-rational/acos` | 419.00 ns | 416.16 ns - 422.54 ns | 413.98 ns | - | - |
| `scalar_ops/hyperreal-rational/acos_abort` | 444.50 ns | 442.23 ns - 447.34 ns | 441.03 ns | - | - |
| `scalar_ops/hyperreal-rational/acosh` | 170.51 ns | 169.41 ns - 171.78 ns | 168.45 ns | - | - |
| `scalar_ops/hyperreal-rational/acosh_abort` | 199.81 ns | 198.88 ns - 200.82 ns | 197.81 ns | - | - |
| `scalar_ops/hyperreal-rational/add` | 32.18 ns | 32.10 ns - 32.29 ns | 32.06 ns | - | - |
| `scalar_ops/hyperreal-rational/asin` | 161.04 ns | 160.21 ns - 161.98 ns | 159.67 ns | - | - |
| `scalar_ops/hyperreal-rational/asin_abort` | 186.21 ns | 185.57 ns - 187.01 ns | 185.17 ns | - | - |
| `scalar_ops/hyperreal-rational/asinh` | 194.08 ns | 193.25 ns - 194.99 ns | 191.70 ns | - | - |
| `scalar_ops/hyperreal-rational/asinh_abort` | 229.07 ns | 225.81 ns - 233.55 ns | 223.48 ns | - | - |
| `scalar_ops/hyperreal-rational/atan` | 294.10 ns | 292.72 ns - 295.60 ns | 290.56 ns | - | - |
| `scalar_ops/hyperreal-rational/atan_abort` | 320.50 ns | 319.59 ns - 321.52 ns | 318.57 ns | - | - |
| `scalar_ops/hyperreal-rational/atanh` | 164.65 ns | 163.55 ns - 165.91 ns | 162.54 ns | - | - |
| `scalar_ops/hyperreal-rational/atanh_abort` | 189.52 ns | 188.55 ns - 190.67 ns | 187.65 ns | - | - |
| `scalar_ops/hyperreal-rational/cos` | 261.06 ns | 259.98 ns - 262.23 ns | 258.95 ns | - | - |
| `scalar_ops/hyperreal-rational/cosh` | 660.55 ns | 656.86 ns - 664.55 ns | 654.70 ns | - | - |
| `scalar_ops/hyperreal-rational/div` | 63.01 ns | 62.76 ns - 63.29 ns | 62.49 ns | - | - |
| `scalar_ops/hyperreal-rational/e` | 22.50 ns | 22.47 ns - 22.53 ns | 22.46 ns | - | - |
| `scalar_ops/hyperreal-rational/exp` | 99.21 ns | 98.88 ns - 99.60 ns | 98.53 ns | - | - |
| `scalar_ops/hyperreal-rational/ln` | 383.89 ns | 380.04 ns - 388.46 ns | 375.00 ns | - | - |
| `scalar_ops/hyperreal-rational/log10` | 572.86 ns | 571.77 ns - 574.18 ns | 571.45 ns | - | - |
| `scalar_ops/hyperreal-rational/log10_abort` | 605.87 ns | 603.03 ns - 609.53 ns | 601.56 ns | - | - |
| `scalar_ops/hyperreal-rational/mul` | 35.01 ns | 34.92 ns - 35.12 ns | 34.86 ns | - | - |
| `scalar_ops/hyperreal-rational/neg` | 19.19 ns | 19.14 ns - 19.24 ns | 19.12 ns | - | - |
| `scalar_ops/hyperreal-rational/one` | 12.17 ns | 12.15 ns - 12.20 ns | 12.14 ns | - | - |
| `scalar_ops/hyperreal-rational/pi` | 17.36 ns | 17.29 ns - 17.43 ns | 17.23 ns | - | - |
| `scalar_ops/hyperreal-rational/pow` | 1.95 us | 1.94 us - 1.95 us | 1.94 us | - | - |
| `scalar_ops/hyperreal-rational/powi` | 58.04 ns | 57.87 ns - 58.23 ns | 57.73 ns | - | - |
| `scalar_ops/hyperreal-rational/powi_negative_one` | 28.14 ns | 28.04 ns - 28.25 ns | 27.99 ns | - | - |
| `scalar_ops/hyperreal-rational/reciprocal` | 17.46 ns | 17.41 ns - 17.53 ns | 17.38 ns | - | - |
| `scalar_ops/hyperreal-rational/reciprocal_checked` | 29.71 ns | 29.52 ns - 29.93 ns | 29.29 ns | - | - |
| `scalar_ops/hyperreal-rational/reciprocal_checked_abort` | 44.77 ns | 44.59 ns - 44.98 ns | 44.41 ns | - | - |
| `scalar_ops/hyperreal-rational/sin` | 260.12 ns | 259.51 ns - 260.81 ns | 259.09 ns | - | - |
| `scalar_ops/hyperreal-rational/sinh` | 627.87 ns | 624.90 ns - 631.23 ns | 621.67 ns | - | - |
| `scalar_ops/hyperreal-rational/sqrt` | 51.03 ns | 50.81 ns - 51.26 ns | 50.51 ns | - | - |
| `scalar_ops/hyperreal-rational/sub` | 32.84 ns | 32.80 ns - 32.89 ns | 32.74 ns | - | - |
| `scalar_ops/hyperreal-rational/tan` | 50.71 ns | 50.57 ns - 50.89 ns | 50.53 ns | - | - |
| `scalar_ops/hyperreal-rational/tanh` | 623.15 ns | 619.89 ns - 626.63 ns | 618.99 ns | - | - |
| `scalar_ops/hyperreal-rational/tau` | 17.43 ns | 17.40 ns - 17.47 ns | 17.40 ns | - | - |
| `scalar_ops/hyperreal-rational/zero` | 11.80 ns | 11.76 ns - 11.86 ns | 11.76 ns | - | - |
| `scalar_ops/hyperreal-rational/zero_status` | 1.58 ns | 1.57 ns - 1.60 ns | 1.59 ns | - | - |
| `scalar_ops/hyperreal/acos` | 411.79 ns | 410.52 ns - 413.33 ns | 409.95 ns | - | - |
| `scalar_ops/hyperreal/acos_abort` | 435.91 ns | 435.38 ns - 436.47 ns | 435.59 ns | - | - |
| `scalar_ops/hyperreal/acosh` | 169.87 ns | 169.44 ns - 170.36 ns | 169.03 ns | - | - |
| `scalar_ops/hyperreal/acosh_abort` | 196.88 ns | 196.49 ns - 197.34 ns | 196.53 ns | - | - |
| `scalar_ops/hyperreal/add` | 31.83 ns | 31.74 ns - 31.93 ns | 31.65 ns | - | - |
| `scalar_ops/hyperreal/asin` | 157.57 ns | 157.22 ns - 157.99 ns | 157.00 ns | - | - |
| `scalar_ops/hyperreal/asin_abort` | 186.21 ns | 185.56 ns - 186.96 ns | 184.88 ns | - | - |
| `scalar_ops/hyperreal/asinh` | 192.26 ns | 191.72 ns - 192.89 ns | 191.45 ns | - | - |
| `scalar_ops/hyperreal/asinh_abort` | 219.91 ns | 219.41 ns - 220.45 ns | 219.74 ns | - | - |
| `scalar_ops/hyperreal/atan` | 287.41 ns | 286.56 ns - 288.39 ns | 285.78 ns | - | - |
| `scalar_ops/hyperreal/atan_abort` | 318.90 ns | 317.62 ns - 320.37 ns | 316.30 ns | - | - |
| `scalar_ops/hyperreal/atanh` | 167.97 ns | 167.47 ns - 168.57 ns | 167.06 ns | - | - |
| `scalar_ops/hyperreal/atanh_abort` | 196.20 ns | 195.14 ns - 197.43 ns | 194.17 ns | - | - |
| `scalar_ops/hyperreal/cos` | 256.72 ns | 256.44 ns - 257.05 ns | 256.57 ns | - | - |
| `scalar_ops/hyperreal/cosh` | 635.24 ns | 632.22 ns - 638.58 ns | 629.21 ns | - | - |
| `scalar_ops/hyperreal/div` | 64.48 ns | 64.42 ns - 64.54 ns | 64.41 ns | - | - |
| `scalar_ops/hyperreal/e` | 22.48 ns | 22.46 ns - 22.51 ns | 22.45 ns | - | - |
| `scalar_ops/hyperreal/exp` | 96.20 ns | 96.14 ns - 96.27 ns | 96.21 ns | - | - |
| `scalar_ops/hyperreal/ln` | 1.17 us | 1.17 us - 1.17 us | 1.16 us | - | - |
| `scalar_ops/hyperreal/log10` | 1.41 us | 1.41 us - 1.42 us | 1.41 us | - | - |
| `scalar_ops/hyperreal/log10_abort` | 1.43 us | 1.43 us - 1.44 us | 1.43 us | - | - |
| `scalar_ops/hyperreal/mul` | 33.01 ns | 32.97 ns - 33.06 ns | 32.96 ns | - | - |
| `scalar_ops/hyperreal/neg` | 18.64 ns | 18.59 ns - 18.71 ns | 18.56 ns | - | - |
| `scalar_ops/hyperreal/one` | 12.16 ns | 12.15 ns - 12.18 ns | 12.14 ns | - | - |
| `scalar_ops/hyperreal/pi` | 17.27 ns | 17.26 ns - 17.28 ns | 17.26 ns | - | - |
| `scalar_ops/hyperreal/pow` | 1.38 us | 1.38 us - 1.38 us | 1.38 us | - | - |
| `scalar_ops/hyperreal/powi` | 53.85 ns | 53.79 ns - 53.94 ns | 53.78 ns | - | - |
| `scalar_ops/hyperreal/powi_negative_one` | 28.03 ns | 27.99 ns - 28.07 ns | 28.00 ns | - | - |
| `scalar_ops/hyperreal/reciprocal` | 17.31 ns | 17.29 ns - 17.34 ns | 17.28 ns | - | - |
| `scalar_ops/hyperreal/reciprocal_checked` | 29.10 ns | 29.08 ns - 29.12 ns | 29.06 ns | - | - |
| `scalar_ops/hyperreal/reciprocal_checked_abort` | 44.23 ns | 44.12 ns - 44.44 ns | 44.12 ns | - | - |
| `scalar_ops/hyperreal/sin` | 257.79 ns | 257.34 ns - 258.33 ns | 257.07 ns | - | - |
| `scalar_ops/hyperreal/sinh` | 615.98 ns | 612.84 ns - 620.17 ns | 609.96 ns | - | - |
| `scalar_ops/hyperreal/sqrt` | 68.15 ns | 67.98 ns - 68.33 ns | 67.85 ns | - | - |
| `scalar_ops/hyperreal/sub` | 32.43 ns | 32.35 ns - 32.54 ns | 32.30 ns | - | - |
| `scalar_ops/hyperreal/tan` | 50.36 ns | 50.32 ns - 50.40 ns | 50.33 ns | - | - |
| `scalar_ops/hyperreal/tanh` | 599.01 ns | 596.49 ns - 601.86 ns | 595.34 ns | - | - |
| `scalar_ops/hyperreal/tau` | 17.44 ns | 17.41 ns - 17.49 ns | 17.41 ns | - | - |
| `scalar_ops/hyperreal/zero` | 11.91 ns | 11.89 ns - 11.94 ns | 11.91 ns | - | - |
| `scalar_ops/hyperreal/zero_status` | 1.55 ns | 1.54 ns - 1.57 ns | 1.55 ns | - | - |
| `scalar_ops/numerica128/acos` | 2.61 us | 2.59 us - 2.63 us | 2.57 us | - | - |
| `scalar_ops/numerica128/acos_abort` | 2.58 us | 2.58 us - 2.59 us | 2.58 us | - | - |
| `scalar_ops/numerica128/acosh` | 3.42 us | 3.38 us - 3.47 us | 3.36 us | - | - |
| `scalar_ops/numerica128/acosh_abort` | 3.39 us | 3.36 us - 3.43 us | 3.33 us | - | - |
| `scalar_ops/numerica128/add` | 42.68 ns | 42.48 ns - 42.90 ns | 42.24 ns | - | - |
| `scalar_ops/numerica128/asin` | 2.51 us | 2.49 us - 2.54 us | 2.46 us | - | - |
| `scalar_ops/numerica128/asin_abort` | 2.46 us | 2.45 us - 2.47 us | 2.45 us | - | - |
| `scalar_ops/numerica128/asinh` | 1.63 us | 1.62 us - 1.64 us | 1.61 us | - | - |
| `scalar_ops/numerica128/asinh_abort` | 1.62 us | 1.61 us - 1.62 us | 1.61 us | - | - |
| `scalar_ops/numerica128/atan` | 2.30 us | 2.29 us - 2.30 us | 2.29 us | - | - |
| `scalar_ops/numerica128/atan_abort` | 2.30 us | 2.30 us - 2.31 us | 2.30 us | - | - |
| `scalar_ops/numerica128/atanh` | 1.28 us | 1.27 us - 1.29 us | 1.26 us | - | - |
| `scalar_ops/numerica128/atanh_abort` | 1.31 us | 1.29 us - 1.32 us | 1.28 us | - | - |
| `scalar_ops/numerica128/cos` | 631.11 ns | 629.89 ns - 632.52 ns | 629.39 ns | - | - |
| `scalar_ops/numerica128/cosh` | 1.08 us | 1.08 us - 1.09 us | 1.07 us | - | - |
| `scalar_ops/numerica128/div` | 62.92 ns | 62.72 ns - 63.17 ns | 62.62 ns | - | - |
| `scalar_ops/numerica128/e` | 1.08 us | 1.07 us - 1.09 us | 1.07 us | - | - |
| `scalar_ops/numerica128/exp` | 938.51 ns | 934.77 ns - 942.68 ns | 931.38 ns | - | - |
| `scalar_ops/numerica128/ln` | 1.32 us | 1.32 us - 1.34 us | 1.31 us | - | - |
| `scalar_ops/numerica128/log10` | 2.81 us | 2.79 us - 2.84 us | 2.78 us | - | - |
| `scalar_ops/numerica128/log10_abort` | 2.77 us | 2.77 us - 2.78 us | 2.76 us | - | - |
| `scalar_ops/numerica128/mul` | 45.41 ns | 45.08 ns - 45.83 ns | 44.98 ns | - | - |
| `scalar_ops/numerica128/neg` | 21.76 ns | 21.70 ns - 21.82 ns | 21.63 ns | - | - |
| `scalar_ops/numerica128/one` | 30.87 ns | 30.67 ns - 31.08 ns | 30.49 ns | - | - |
| `scalar_ops/numerica128/pi` | 49.06 ns | 48.71 ns - 49.46 ns | 48.46 ns | - | - |
| `scalar_ops/numerica128/pow` | 2.99 us | 2.97 us - 3.02 us | 2.96 us | - | - |
| `scalar_ops/numerica128/powi` | 85.41 ns | 84.86 ns - 86.01 ns | 84.43 ns | - | - |
| `scalar_ops/numerica128/powi_negative_one` | 59.97 ns | 59.63 ns - 60.36 ns | 59.16 ns | - | - |
| `scalar_ops/numerica128/reciprocal` | 60.73 ns | 60.21 ns - 61.33 ns | 59.61 ns | - | - |
| `scalar_ops/numerica128/reciprocal_checked` | 61.08 ns | 60.63 ns - 61.59 ns | 60.35 ns | - | - |
| `scalar_ops/numerica128/reciprocal_checked_abort` | 59.81 ns | 59.59 ns - 60.05 ns | 59.32 ns | - | - |
| `scalar_ops/numerica128/sin` | 1.34 us | 1.32 us - 1.36 us | 1.31 us | - | - |
| `scalar_ops/numerica128/sinh` | 1.14 us | 1.14 us - 1.15 us | 1.13 us | - | - |
| `scalar_ops/numerica128/sqrt` | 95.89 ns | 95.45 ns - 96.41 ns | 95.09 ns | - | - |
| `scalar_ops/numerica128/sub` | 45.79 ns | 45.67 ns - 45.93 ns | 45.58 ns | - | - |
| `scalar_ops/numerica128/tan` | 1.61 us | 1.60 us - 1.62 us | 1.59 us | - | - |
| `scalar_ops/numerica128/tanh` | 1.20 us | 1.20 us - 1.21 us | 1.20 us | - | - |
| `scalar_ops/numerica128/tau` | 100.47 ns | 99.76 ns - 101.27 ns | 98.96 ns | - | - |
| `scalar_ops/numerica128/zero` | 15.62 ns | 15.59 ns - 15.66 ns | 15.55 ns | - | - |
| `scalar_ops/numerica128/zero_status` | 7.17 ns | 7.16 ns - 7.19 ns | 7.14 ns | - | - |
| `scalar_ops/numerica128/zero_status_abort` | 7.46 ns | 7.40 ns - 7.52 ns | 7.33 ns | - | - |
| `scalar_ops/symbolica/acos` | 18.35 us | 18.24 us - 18.48 us | 18.15 us | - | - |
| `scalar_ops/symbolica/acos_abort` | 18.80 us | 18.61 us - 19.02 us | 18.39 us | - | - |
| `scalar_ops/symbolica/acosh` | 14.82 us | 14.71 us - 14.95 us | 14.60 us | - | - |
| `scalar_ops/symbolica/acosh_abort` | 14.84 us | 14.73 us - 14.96 us | 14.59 us | - | - |
| `scalar_ops/symbolica/add` | 1.77 us | 1.76 us - 1.78 us | 1.74 us | - | - |
| `scalar_ops/symbolica/asin` | 18.69 us | 18.51 us - 18.90 us | 18.30 us | - | - |
| `scalar_ops/symbolica/asin_abort` | 18.20 us | 18.13 us - 18.27 us | 18.07 us | - | - |
| `scalar_ops/symbolica/asinh` | 10.44 us | 10.42 us - 10.47 us | 10.40 us | - | - |
| `scalar_ops/symbolica/asinh_abort` | 10.77 us | 10.68 us - 10.86 us | 10.64 us | - | - |
| `scalar_ops/symbolica/atan` | 23.30 us | 23.19 us - 23.43 us | 23.08 us | - | - |
| `scalar_ops/symbolica/atan_abort` | 23.26 us | 23.18 us - 23.35 us | 23.13 us | - | - |
| `scalar_ops/symbolica/atanh` | 18.27 us | 18.20 us - 18.35 us | 18.13 us | - | - |
| `scalar_ops/symbolica/atanh_abort` | 18.16 us | 18.09 us - 18.24 us | 18.06 us | - | - |
| `scalar_ops/symbolica/cos` | 2.56 us | 2.55 us - 2.57 us | 2.54 us | - | - |
| `scalar_ops/symbolica/cosh` | 12.69 us | 12.57 us - 12.84 us | 12.47 us | - | - |
| `scalar_ops/symbolica/div` | 3.06 us | 3.05 us - 3.07 us | 3.04 us | - | - |
| `scalar_ops/symbolica/e` | 227.42 ns | 226.45 ns - 228.52 ns | 225.24 ns | - | - |
| `scalar_ops/symbolica/exp` | 2.67 us | 2.66 us - 2.68 us | 2.65 us | - | - |
| `scalar_ops/symbolica/ln` | 2.56 us | 2.56 us - 2.57 us | 2.55 us | - | - |
| `scalar_ops/symbolica/log10` | 8.62 us | 8.61 us - 8.64 us | 8.61 us | - | - |
| `scalar_ops/symbolica/log10_abort` | 8.64 us | 8.62 us - 8.65 us | 8.62 us | - | - |
| `scalar_ops/symbolica/mul` | 1.96 us | 1.96 us - 1.96 us | 1.96 us | - | - |
| `scalar_ops/symbolica/neg` | 1.54 us | 1.53 us - 1.54 us | 1.53 us | - | - |
| `scalar_ops/symbolica/one` | 29.95 ns | 29.84 ns - 30.08 ns | 29.74 ns | - | - |
| `scalar_ops/symbolica/pi` | 225.22 ns | 224.39 ns - 226.24 ns | 223.78 ns | - | - |
| `scalar_ops/symbolica/pow` | 2.77 us | 2.76 us - 2.78 us | 2.76 us | - | - |
| `scalar_ops/symbolica/powi` | 2.01 us | 2.00 us - 2.02 us | 1.99 us | - | - |
| `scalar_ops/symbolica/powi_negative_one` | 2.05 us | 2.04 us - 2.06 us | 2.03 us | - | - |
| `scalar_ops/symbolica/reciprocal` | 2.05 us | 2.04 us - 2.07 us | 2.03 us | - | - |
| `scalar_ops/symbolica/reciprocal_checked` | 2.03 us | 2.03 us - 2.04 us | 2.02 us | - | - |
| `scalar_ops/symbolica/reciprocal_checked_abort` | 2.02 us | 2.02 us - 2.03 us | 2.01 us | - | - |
| `scalar_ops/symbolica/sin` | 3.00 us | 2.99 us - 3.01 us | 2.99 us | - | - |
| `scalar_ops/symbolica/sinh` | 14.15 us | 13.96 us - 14.40 us | 13.85 us | - | - |
| `scalar_ops/symbolica/sqrt` | 2.24 us | 2.23 us - 2.25 us | 2.23 us | - | - |
| `scalar_ops/symbolica/sub` | 2.96 us | 2.95 us - 2.98 us | 2.94 us | - | - |
| `scalar_ops/symbolica/tan` | 8.79 us | 8.75 us - 8.83 us | 8.71 us | - | - |
| `scalar_ops/symbolica/tanh` | 29.53 us | 29.30 us - 29.78 us | 29.02 us | - | - |
| `scalar_ops/symbolica/tau` | 2.34 us | 2.33 us - 2.36 us | 2.31 us | - | - |
| `scalar_ops/symbolica/zero` | 0.95 ns | 0.95 ns - 0.96 ns | 0.94 ns | - | - |
| `scalar_ops/symbolica/zero_status` | 7.94 ns | 7.93 ns - 7.96 ns | 7.92 ns | - | - |
| `scalar_ops/symbolica/zero_status_abort` | 8.02 ns | 7.99 ns - 8.05 ns | 7.96 ns | - | - |
| `scalar_sqrt_cases/gmp_mpfr128/e_import` | 110.72 ns | 110.64 ns - 110.82 ns | 110.66 ns | - | - |
| `scalar_sqrt_cases/gmp_mpfr128/perfect_1e12` | 103.52 ns | 102.99 ns - 104.19 ns | 102.59 ns | - | - |
| `scalar_sqrt_cases/gmp_mpfr128/perfect_9` | 102.89 ns | 102.55 ns - 103.32 ns | 102.37 ns | - | - |
| `scalar_sqrt_cases/gmp_mpfr128/tiny_1e_12` | 106.20 ns | 105.70 ns - 106.80 ns | 105.24 ns | - | - |
| `scalar_sqrt_cases/hyperreal-rational/e_import` | 105.43 ns | 105.08 ns - 105.85 ns | 104.95 ns | - | - |
| `scalar_sqrt_cases/hyperreal-rational/perfect_1e12` | 31.39 ns | 31.22 ns - 31.57 ns | 31.03 ns | - | - |
| `scalar_sqrt_cases/hyperreal-rational/perfect_9` | 31.42 ns | 31.36 ns - 31.48 ns | 31.37 ns | - | - |
| `scalar_sqrt_cases/hyperreal-rational/tiny_1e_12` | 31.52 ns | 31.35 ns - 31.70 ns | 31.18 ns | - | - |
| `scalar_sqrt_cases/hyperreal/e_import` | 105.20 ns | 104.71 ns - 105.78 ns | 104.29 ns | - | - |
| `scalar_sqrt_cases/hyperreal/perfect_1e12` | 31.68 ns | 31.49 ns - 31.89 ns | 31.26 ns | - | - |
| `scalar_sqrt_cases/hyperreal/perfect_9` | 31.58 ns | 31.49 ns - 31.67 ns | 31.45 ns | - | - |
| `scalar_sqrt_cases/hyperreal/tiny_1e_12` | 103.47 ns | 103.20 ns - 103.77 ns | 103.04 ns | - | - |
| `scalar_sqrt_cases/numerica128/e_import` | 100.79 ns | 100.48 ns - 101.13 ns | 100.12 ns | - | - |
| `scalar_sqrt_cases/numerica128/perfect_1e12` | 91.09 ns | 90.89 ns - 91.31 ns | 90.67 ns | - | - |
| `scalar_sqrt_cases/numerica128/perfect_9` | 92.01 ns | 91.51 ns - 92.59 ns | 90.94 ns | - | - |
| `scalar_sqrt_cases/numerica128/tiny_1e_12` | 95.41 ns | 94.76 ns - 96.12 ns | 94.25 ns | - | - |
| `scalar_sqrt_cases/symbolica/e_import` | 2.14 us | 2.14 us - 2.15 us | 2.14 us | - | - |
| `scalar_sqrt_cases/symbolica/perfect_1e12` | 2.20 us | 2.19 us - 2.21 us | 2.19 us | - | - |
| `scalar_sqrt_cases/symbolica/perfect_9` | 2.18 us | 2.16 us - 2.20 us | 2.15 us | - | - |
| `scalar_sqrt_cases/symbolica/tiny_1e_12` | 2.38 us | 2.37 us - 2.38 us | 2.38 us | - | - |
| `scalar_trig/gmp_mpfr128/0.1/cos` | 487.07 ns | 485.87 ns - 488.64 ns | 485.59 ns | - | - |
| `scalar_trig/gmp_mpfr128/0.1/sin` | 763.47 ns | 761.95 ns - 765.25 ns | 761.58 ns | - | - |
| `scalar_trig/gmp_mpfr128/0.5/acos` | 3.01 us | 3.01 us - 3.02 us | 3.01 us | - | - |
| `scalar_trig/gmp_mpfr128/0.5/asin` | 2.99 us | 2.99 us - 3.00 us | 2.99 us | - | - |
| `scalar_trig/gmp_mpfr128/0.5/asinh` | 1.59 us | 1.59 us - 1.59 us | 1.59 us | - | - |
| `scalar_trig/gmp_mpfr128/0.5/atan` | 2.73 us | 2.72 us - 2.73 us | 2.72 us | - | - |
| `scalar_trig/gmp_mpfr128/0.5/atanh` | 1.62 us | 1.62 us - 1.63 us | 1.62 us | - | - |
| `scalar_trig/gmp_mpfr128/0.999999/acos` | 2.77 us | 2.75 us - 2.80 us | 2.73 us | - | - |
| `scalar_trig/gmp_mpfr128/0.999999/asin` | 2.55 us | 2.54 us - 2.56 us | 2.53 us | - | - |
| `scalar_trig/gmp_mpfr128/0.999999/atanh` | 1.58 us | 1.58 us - 1.58 us | 1.58 us | - | - |
| `scalar_trig/gmp_mpfr128/1.23456789/cos` | 582.90 ns | 580.25 ns - 586.36 ns | 578.59 ns | - | - |
| `scalar_trig/gmp_mpfr128/1.23456789/sin` | 792.78 ns | 791.12 ns - 794.72 ns | 791.33 ns | - | - |
| `scalar_trig/gmp_mpfr128/1000pi_eps/cos` | 578.00 ns | 577.56 ns - 578.47 ns | 577.24 ns | - | - |
| `scalar_trig/gmp_mpfr128/1000pi_eps/sin` | 2.26 us | 2.26 us - 2.26 us | 2.26 us | - | - |
| `scalar_trig/gmp_mpfr128/1_plus_1e-12/acosh` | 8.29 us | 8.26 us - 8.32 us | 8.24 us | - | - |
| `scalar_trig/gmp_mpfr128/1e-12/acos` | 1.44 us | 1.43 us - 1.45 us | 1.42 us | - | - |
| `scalar_trig/gmp_mpfr128/1e-12/asin` | 1.41 us | 1.40 us - 1.41 us | 1.40 us | - | - |
| `scalar_trig/gmp_mpfr128/1e-12/atanh` | 171.28 ns | 171.12 ns - 171.47 ns | 171.07 ns | - | - |
| `scalar_trig/gmp_mpfr128/1e30/cos` | 964.92 ns | 963.94 ns - 966.01 ns | 963.27 ns | - | - |
| `scalar_trig/gmp_mpfr128/1e30/sin` | 2.88 us | 2.87 us - 2.88 us | 2.87 us | - | - |
| `scalar_trig/gmp_mpfr128/1e6/acosh` | 1.57 us | 1.57 us - 1.57 us | 1.57 us | - | - |
| `scalar_trig/gmp_mpfr128/1e6/asinh` | 1.60 us | 1.59 us - 1.60 us | 1.59 us | - | - |
| `scalar_trig/gmp_mpfr128/1e6/atan` | 1.39 us | 1.39 us - 1.39 us | 1.39 us | - | - |
| `scalar_trig/gmp_mpfr128/1e6/cos` | 816.89 ns | 815.01 ns - 819.03 ns | 814.00 ns | - | - |
| `scalar_trig/gmp_mpfr128/1e6/sin` | 1.08 us | 1.07 us - 1.08 us | 1.07 us | - | - |
| `scalar_trig/gmp_mpfr128/9/acosh` | 1.59 us | 1.58 us - 1.59 us | 1.59 us | - | - |
| `scalar_trig/gmp_mpfr128/e/acosh` | 1.61 us | 1.61 us - 1.62 us | 1.61 us | - | - |
| `scalar_trig/gmp_mpfr128/neg_0.999999/acos` | 2.67 us | 2.66 us - 2.67 us | 2.65 us | - | - |
| `scalar_trig/gmp_mpfr128/neg_0.999999/asin` | 2.51 us | 2.51 us - 2.52 us | 2.51 us | - | - |
| `scalar_trig/gmp_mpfr128/neg_0.999999/atanh` | 1.59 us | 1.58 us - 1.60 us | 1.58 us | - | - |
| `scalar_trig/gmp_mpfr128/neg_1e-12/asinh` | 8.40 us | 8.39 us - 8.41 us | 8.39 us | - | - |
| `scalar_trig/gmp_mpfr128/neg_1e-12/atan` | 1.04 us | 1.04 us - 1.05 us | 1.04 us | - | - |
| `scalar_trig/gmp_mpfr128/neg_1e6/asinh` | 1.60 us | 1.59 us - 1.60 us | 1.59 us | - | - |
| `scalar_trig/gmp_mpfr128/neg_1e6/atan` | 1.38 us | 1.38 us - 1.38 us | 1.38 us | - | - |
| `scalar_trig/gmp_mpfr128/pi_7/cos` | 528.87 ns | 527.88 ns - 529.98 ns | 527.49 ns | - | - |
| `scalar_trig/gmp_mpfr128/pi_7/sin` | 742.60 ns | 740.00 ns - 745.99 ns | 738.78 ns | - | - |
| `scalar_trig/hyperreal-rational/0.1/cos` | 212.60 ns | 212.39 ns - 212.86 ns | 212.46 ns | +2.59% | - |
| `scalar_trig/hyperreal-rational/0.1/sin` | 219.61 ns | 219.35 ns - 219.89 ns | 219.70 ns | +5.37% | - |
| `scalar_trig/hyperreal-rational/0.5/acos` | 35.01 ns | 34.91 ns - 35.14 ns | 34.86 ns | - | - |
| `scalar_trig/hyperreal-rational/0.5/asin` | 35.06 ns | 34.90 ns - 35.25 ns | 34.74 ns | - | - |
| `scalar_trig/hyperreal-rational/0.5/asinh` | 188.90 ns | 187.82 ns - 190.23 ns | 187.13 ns | - | - |
| `scalar_trig/hyperreal-rational/0.5/atan` | 252.51 ns | 252.19 ns - 252.88 ns | 252.09 ns | - | - |
| `scalar_trig/hyperreal-rational/0.5/atanh` | 35.19 ns | 35.10 ns - 35.31 ns | 35.10 ns | - | - |
| `scalar_trig/hyperreal-rational/0.999999/acos` | 179.90 ns | 179.50 ns - 180.39 ns | 179.30 ns | - | - |
| `scalar_trig/hyperreal-rational/0.999999/asin` | 187.07 ns | 186.63 ns - 187.58 ns | 185.99 ns | - | - |
| `scalar_trig/hyperreal-rational/0.999999/atanh` | 196.90 ns | 196.69 ns - 197.18 ns | 196.84 ns | - | - |
| `scalar_trig/hyperreal-rational/1.23456789/cos` | 210.93 ns | 210.77 ns - 211.09 ns | 210.79 ns | +1.98% | - |
| `scalar_trig/hyperreal-rational/1.23456789/sin` | 219.56 ns | 219.15 ns - 220.17 ns | 219.24 ns | +5.71% | - |
| `scalar_trig/hyperreal-rational/1000pi_eps/cos` | 1.79 us | 1.79 us - 1.79 us | 1.79 us | - | - |
| `scalar_trig/hyperreal-rational/1000pi_eps/sin` | 1.80 us | 1.80 us - 1.81 us | 1.80 us | - | - |
| `scalar_trig/hyperreal-rational/1_plus_1e-12/acosh` | 202.02 ns | 201.37 ns - 202.84 ns | 201.00 ns | - | - |
| `scalar_trig/hyperreal-rational/1e-12/acos` | 1.13 us | 1.12 us - 1.13 us | 1.12 us | - | - |
| `scalar_trig/hyperreal-rational/1e-12/asin` | 183.40 ns | 183.06 ns - 183.76 ns | 183.22 ns | - | - |
| `scalar_trig/hyperreal-rational/1e-12/atanh` | 177.51 ns | 177.19 ns - 177.91 ns | 177.09 ns | - | - |
| `scalar_trig/hyperreal-rational/1e30/cos` | 327.80 ns | 327.57 ns - 328.04 ns | 327.66 ns | - | - |
| `scalar_trig/hyperreal-rational/1e30/sin` | 328.96 ns | 328.51 ns - 329.51 ns | 328.56 ns | - | - |
| `scalar_trig/hyperreal-rational/1e6/acosh` | 156.88 ns | 156.80 ns - 156.96 ns | 156.92 ns | - | - |
| `scalar_trig/hyperreal-rational/1e6/asinh` | 192.13 ns | 191.55 ns - 192.76 ns | 191.11 ns | - | - |
| `scalar_trig/hyperreal-rational/1e6/atan` | 371.42 ns | 371.07 ns - 371.82 ns | 370.69 ns | - | - |
| `scalar_trig/hyperreal-rational/1e6/cos` | 355.75 ns | 355.44 ns - 356.12 ns | 355.82 ns | - | - |
| `scalar_trig/hyperreal-rational/1e6/sin` | 356.80 ns | 356.31 ns - 357.40 ns | 356.23 ns | +1.72% | - |
| `scalar_trig/hyperreal-rational/9/acosh` | 157.93 ns | 157.63 ns - 158.28 ns | 157.50 ns | - | - |
| `scalar_trig/hyperreal-rational/e/acosh` | 147.59 ns | 147.17 ns - 148.10 ns | 146.93 ns | - | - |
| `scalar_trig/hyperreal-rational/neg_0.999999/acos` | 220.93 ns | 220.24 ns - 221.84 ns | 219.85 ns | - | - |
| `scalar_trig/hyperreal-rational/neg_0.999999/asin` | 202.64 ns | 202.01 ns - 203.39 ns | 201.51 ns | - | - |
| `scalar_trig/hyperreal-rational/neg_0.999999/atanh` | 230.60 ns | 229.78 ns - 231.52 ns | 229.05 ns | - | - |
| `scalar_trig/hyperreal-rational/neg_1e-12/asinh` | 308.78 ns | 307.63 ns - 310.39 ns | 306.82 ns | - | - |
| `scalar_trig/hyperreal-rational/neg_1e-12/atan` | 325.29 ns | 324.73 ns - 325.91 ns | 324.47 ns | - | - |
| `scalar_trig/hyperreal-rational/neg_1e6/asinh` | 251.88 ns | 250.77 ns - 253.13 ns | 250.13 ns | - | - |
| `scalar_trig/hyperreal-rational/neg_1e6/atan` | 454.63 ns | 453.42 ns - 456.37 ns | 453.31 ns | - | - |
| `scalar_trig/hyperreal-rational/pi_7/cos` | 647.23 ns | 643.53 ns - 651.77 ns | 640.22 ns | - | - |
| `scalar_trig/hyperreal-rational/pi_7/sin` | 762.77 ns | 759.49 ns - 766.60 ns | 756.93 ns | - | - |
| `scalar_trig/hyperreal/0.1/cos` | 210.08 ns | 209.98 ns - 210.19 ns | 210.04 ns | +0.32% | - |
| `scalar_trig/hyperreal/0.1/sin` | 221.11 ns | 220.29 ns - 222.05 ns | 219.60 ns | +5.63% | - |
| `scalar_trig/hyperreal/0.5/acos` | 36.74 ns | 36.63 ns - 36.88 ns | 36.51 ns | -17.04% | - |
| `scalar_trig/hyperreal/0.5/asin` | 35.69 ns | 35.65 ns - 35.76 ns | 35.64 ns | +1.88% | - |
| `scalar_trig/hyperreal/0.5/asinh` | 189.45 ns | 189.07 ns - 189.88 ns | 188.82 ns | -0.35% | - |
| `scalar_trig/hyperreal/0.5/atan` | 252.72 ns | 252.38 ns - 253.12 ns | 252.28 ns | -0.99% | - |
| `scalar_trig/hyperreal/0.5/atanh` | 35.29 ns | 35.17 ns - 35.45 ns | 35.21 ns | +2.65% | - |
| `scalar_trig/hyperreal/0.999999/acos` | 179.81 ns | 179.64 ns - 180.01 ns | 179.56 ns | -1.67% | - |
| `scalar_trig/hyperreal/0.999999/asin` | 186.36 ns | 185.79 ns - 187.06 ns | 185.26 ns | +0.08% | - |
| `scalar_trig/hyperreal/0.999999/atanh` | 212.21 ns | 211.93 ns - 212.57 ns | 211.91 ns | -0.50% | - |
| `scalar_trig/hyperreal/1.23456789/cos` | 211.39 ns | 211.24 ns - 211.55 ns | 211.39 ns | +2.32% | - |
| `scalar_trig/hyperreal/1.23456789/sin` | 219.33 ns | 218.96 ns - 219.88 ns | 218.94 ns | +5.11% | - |
| `scalar_trig/hyperreal/1000pi_eps/cos` | 191.63 ns | 191.48 ns - 191.80 ns | 191.60 ns | +0.32% | - |
| `scalar_trig/hyperreal/1000pi_eps/sin` | 198.80 ns | 198.45 ns - 199.23 ns | 198.28 ns | +4.11% | - |
| `scalar_trig/hyperreal/1_plus_1e-12/acosh` | 202.51 ns | 201.83 ns - 203.45 ns | 201.61 ns | +3.20% | - |
| `scalar_trig/hyperreal/1e-12/acos` | 1.10 us | 1.09 us - 1.10 us | 1.09 us | -1.33% | - |
| `scalar_trig/hyperreal/1e-12/asin` | 169.60 ns | 169.05 ns - 170.21 ns | 168.70 ns | -0.34% | - |
| `scalar_trig/hyperreal/1e-12/atanh` | 163.62 ns | 163.25 ns - 164.02 ns | 163.61 ns | +4.26% | - |
| `scalar_trig/hyperreal/1e30/cos` | 330.83 ns | 330.19 ns - 331.61 ns | 330.15 ns | -3.91% | - |
| `scalar_trig/hyperreal/1e30/sin` | 330.33 ns | 329.58 ns - 331.17 ns | 328.82 ns | -2.89% | - |
| `scalar_trig/hyperreal/1e6/acosh` | 158.00 ns | 157.49 ns - 158.59 ns | 156.78 ns | -0.80% | - |
| `scalar_trig/hyperreal/1e6/asinh` | 191.53 ns | 191.33 ns - 191.74 ns | 191.33 ns | +0.24% | - |
| `scalar_trig/hyperreal/1e6/atan` | 371.36 ns | 370.76 ns - 372.04 ns | 370.30 ns | -1.75% | - |
| `scalar_trig/hyperreal/1e6/cos` | 355.97 ns | 355.70 ns - 356.27 ns | 355.55 ns | -4.71% | - |
| `scalar_trig/hyperreal/1e6/sin` | 358.72 ns | 358.09 ns - 359.42 ns | 358.04 ns | +1.73% | - |
| `scalar_trig/hyperreal/9/acosh` | 156.71 ns | 156.55 ns - 156.92 ns | 156.54 ns | +0.46% | - |
| `scalar_trig/hyperreal/e/acosh` | 147.76 ns | 146.99 ns - 148.75 ns | 146.44 ns | -0.94% | - |
| `scalar_trig/hyperreal/neg_0.999999/acos` | 239.80 ns | 239.19 ns - 240.50 ns | 238.53 ns | -0.41% | - |
| `scalar_trig/hyperreal/neg_0.999999/asin` | 218.44 ns | 217.45 ns - 219.57 ns | 216.35 ns | +1.47% | - |
| `scalar_trig/hyperreal/neg_0.999999/atanh` | 260.47 ns | 260.22 ns - 260.76 ns | 260.29 ns | +1.33% | - |
| `scalar_trig/hyperreal/neg_1e-12/asinh` | 297.04 ns | 296.16 ns - 297.98 ns | 294.70 ns | +0.65% | - |
| `scalar_trig/hyperreal/neg_1e-12/atan` | 317.48 ns | 316.97 ns - 318.04 ns | 316.59 ns | +1.17% | - |
| `scalar_trig/hyperreal/neg_1e6/asinh` | 246.80 ns | 246.51 ns - 247.11 ns | 246.40 ns | +0.13% | - |
| `scalar_trig/hyperreal/neg_1e6/atan` | 453.32 ns | 452.27 ns - 454.64 ns | 452.23 ns | +0.92% | - |
| `scalar_trig/hyperreal/pi_7/cos` | 212.04 ns | 211.37 ns - 212.86 ns | 210.91 ns | +0.96% | - |
| `scalar_trig/hyperreal/pi_7/sin` | 219.94 ns | 219.39 ns - 220.57 ns | 218.92 ns | +5.35% | - |
| `scalar_trig/numerica128/0.1/cos` | 491.51 ns | 491.21 ns - 491.82 ns | 491.37 ns | - | - |
| `scalar_trig/numerica128/0.1/sin` | 756.68 ns | 753.83 ns - 760.53 ns | 753.66 ns | - | - |
| `scalar_trig/numerica128/0.5/acos` | 3.03 us | 3.02 us - 3.05 us | 3.02 us | - | - |
| `scalar_trig/numerica128/0.5/asin` | 3.02 us | 3.01 us - 3.02 us | 3.01 us | - | - |
| `scalar_trig/numerica128/0.5/asinh` | 1.58 us | 1.58 us - 1.59 us | 1.58 us | - | - |
| `scalar_trig/numerica128/0.5/atan` | 2.82 us | 2.81 us - 2.82 us | 2.82 us | - | - |
| `scalar_trig/numerica128/0.5/atanh` | 1.62 us | 1.62 us - 1.63 us | 1.62 us | - | - |
| `scalar_trig/numerica128/0.999999/acos` | 2.73 us | 2.72 us - 2.73 us | 2.73 us | - | - |
| `scalar_trig/numerica128/0.999999/asin` | 2.55 us | 2.54 us - 2.57 us | 2.53 us | - | - |
| `scalar_trig/numerica128/0.999999/atanh` | 1.59 us | 1.58 us - 1.60 us | 1.58 us | - | - |
| `scalar_trig/numerica128/1.23456789/cos` | 580.65 ns | 580.11 ns - 581.25 ns | 580.15 ns | - | - |
| `scalar_trig/numerica128/1.23456789/sin` | 796.51 ns | 795.67 ns - 797.49 ns | 796.45 ns | - | - |
| `scalar_trig/numerica128/1000pi_eps/cos` | 582.70 ns | 581.76 ns - 583.78 ns | 582.03 ns | - | - |
| `scalar_trig/numerica128/1000pi_eps/sin` | 2.30 us | 2.30 us - 2.31 us | 2.30 us | - | - |
| `scalar_trig/numerica128/1_plus_1e-12/acosh` | 8.24 us | 8.23 us - 8.26 us | 8.22 us | - | - |
| `scalar_trig/numerica128/1e-12/acos` | 1.42 us | 1.42 us - 1.42 us | 1.42 us | - | - |
| `scalar_trig/numerica128/1e-12/asin` | 1.41 us | 1.41 us - 1.41 us | 1.41 us | - | - |
| `scalar_trig/numerica128/1e-12/atanh` | 171.62 ns | 170.98 ns - 172.72 ns | 170.83 ns | - | - |
| `scalar_trig/numerica128/1e30/cos` | 975.54 ns | 972.67 ns - 978.79 ns | 971.09 ns | - | - |
| `scalar_trig/numerica128/1e30/sin` | 2.86 us | 2.86 us - 2.87 us | 2.86 us | - | - |
| `scalar_trig/numerica128/1e6/acosh` | 1.57 us | 1.57 us - 1.57 us | 1.57 us | - | - |
| `scalar_trig/numerica128/1e6/asinh` | 1.59 us | 1.58 us - 1.60 us | 1.58 us | - | - |
| `scalar_trig/numerica128/1e6/atan` | 1.44 us | 1.44 us - 1.45 us | 1.44 us | - | - |
| `scalar_trig/numerica128/1e6/cos` | 818.83 ns | 817.27 ns - 820.61 ns | 816.73 ns | - | - |
| `scalar_trig/numerica128/1e6/sin` | 1.08 us | 1.08 us - 1.08 us | 1.08 us | - | - |
| `scalar_trig/numerica128/9/acosh` | 1.59 us | 1.59 us - 1.60 us | 1.59 us | - | - |
| `scalar_trig/numerica128/e/acosh` | 1.62 us | 1.62 us - 1.63 us | 1.62 us | - | - |
| `scalar_trig/numerica128/neg_0.999999/acos` | 2.65 us | 2.64 us - 2.65 us | 2.65 us | - | - |
| `scalar_trig/numerica128/neg_0.999999/asin` | 2.53 us | 2.52 us - 2.54 us | 2.52 us | - | - |
| `scalar_trig/numerica128/neg_0.999999/atanh` | 1.58 us | 1.57 us - 1.58 us | 1.58 us | - | - |
| `scalar_trig/numerica128/neg_1e-12/asinh` | 8.41 us | 8.40 us - 8.43 us | 8.39 us | - | - |
| `scalar_trig/numerica128/neg_1e-12/atan` | 1.11 us | 1.10 us - 1.11 us | 1.10 us | - | - |
| `scalar_trig/numerica128/neg_1e6/asinh` | 1.59 us | 1.59 us - 1.59 us | 1.59 us | - | - |
| `scalar_trig/numerica128/neg_1e6/atan` | 1.45 us | 1.45 us - 1.46 us | 1.44 us | - | - |
| `scalar_trig/numerica128/pi_7/cos` | 534.74 ns | 534.06 ns - 535.46 ns | 533.71 ns | - | - |
| `scalar_trig/numerica128/pi_7/sin` | 739.58 ns | 737.66 ns - 741.66 ns | 734.97 ns | - | - |
| `scalar_trig/symbolica/0.1/cos` | 2.42 us | 2.41 us - 2.42 us | 2.41 us | - | - |
| `scalar_trig/symbolica/0.1/sin` | 2.58 us | 2.58 us - 2.58 us | 2.58 us | - | - |
| `scalar_trig/symbolica/0.5/acos` | 17.16 us | 17.12 us - 17.21 us | 17.08 us | - | - |
| `scalar_trig/symbolica/0.5/asin` | 17.25 us | 17.24 us - 17.27 us | 17.25 us | - | - |
| `scalar_trig/symbolica/0.5/asinh` | 10.19 us | 10.18 us - 10.21 us | 10.18 us | - | - |
| `scalar_trig/symbolica/0.5/atan` | 21.79 us | 21.72 us - 21.88 us | 21.69 us | - | - |
| `scalar_trig/symbolica/0.5/atanh` | 16.06 us | 16.02 us - 16.10 us | 16.01 us | - | - |
| `scalar_trig/symbolica/0.999999/acos` | 17.00 us | 16.93 us - 17.07 us | 16.86 us | - | - |
| `scalar_trig/symbolica/0.999999/asin` | 16.93 us | 16.88 us - 17.00 us | 16.83 us | - | - |
| `scalar_trig/symbolica/0.999999/atanh` | 15.84 us | 15.75 us - 15.94 us | 15.68 us | - | - |
| `scalar_trig/symbolica/1.23456789/cos` | 2.38 us | 2.38 us - 2.38 us | 2.38 us | - | - |
| `scalar_trig/symbolica/1.23456789/sin` | 2.55 us | 2.55 us - 2.56 us | 2.54 us | - | - |
| `scalar_trig/symbolica/1000pi_eps/cos` | 2.43 us | 2.43 us - 2.43 us | 2.42 us | - | - |
| `scalar_trig/symbolica/1000pi_eps/sin` | 3.62 us | 3.61 us - 3.63 us | 3.60 us | - | - |
| `scalar_trig/symbolica/1_plus_1e-12/acosh` | 15.23 us | 15.19 us - 15.28 us | 15.18 us | - | - |
| `scalar_trig/symbolica/1e-12/acos` | 19.28 us | 19.25 us - 19.31 us | 19.24 us | - | - |
| `scalar_trig/symbolica/1e-12/asin` | 19.33 us | 19.28 us - 19.39 us | 19.25 us | - | - |
| `scalar_trig/symbolica/1e-12/atanh` | 23.43 us | 23.31 us - 23.56 us | 23.22 us | - | - |
| `scalar_trig/symbolica/1e30/cos` | 3.82 us | 3.81 us - 3.82 us | 3.82 us | - | - |
| `scalar_trig/symbolica/1e30/sin` | 4.33 us | 4.31 us - 4.35 us | 4.29 us | - | - |
| `scalar_trig/symbolica/1e6/acosh` | 13.69 us | 13.68 us - 13.72 us | 13.67 us | - | - |
| `scalar_trig/symbolica/1e6/asinh` | 9.92 us | 9.91 us - 9.94 us | 9.90 us | - | - |
| `scalar_trig/symbolica/1e6/atan` | 22.15 us | 22.04 us - 22.27 us | 21.98 us | - | - |
| `scalar_trig/symbolica/1e6/cos` | 2.57 us | 2.57 us - 2.58 us | 2.57 us | - | - |
| `scalar_trig/symbolica/1e6/sin` | 2.79 us | 2.78 us - 2.81 us | 2.77 us | - | - |
| `scalar_trig/symbolica/9/acosh` | 13.81 us | 13.73 us - 13.90 us | 13.67 us | - | - |
| `scalar_trig/symbolica/e/acosh` | 13.63 us | 13.61 us - 13.67 us | 13.60 us | - | - |
| `scalar_trig/symbolica/neg_0.999999/acos` | 17.16 us | 17.08 us - 17.26 us | 17.01 us | - | - |
| `scalar_trig/symbolica/neg_0.999999/asin` | 16.97 us | 16.92 us - 17.04 us | 16.89 us | - | - |
| `scalar_trig/symbolica/neg_0.999999/atanh` | 15.87 us | 15.82 us - 15.94 us | 15.79 us | - | - |
| `scalar_trig/symbolica/neg_1e-12/asinh` | 14.60 us | 14.59 us - 14.63 us | 14.59 us | - | - |
| `scalar_trig/symbolica/neg_1e-12/atan` | 19.42 us | 19.38 us - 19.48 us | 19.35 us | - | - |
| `scalar_trig/symbolica/neg_1e6/asinh` | 9.80 us | 9.79 us - 9.82 us | 9.77 us | - | - |
| `scalar_trig/symbolica/neg_1e6/atan` | 22.14 us | 21.97 us - 22.36 us | 21.88 us | - | - |
| `scalar_trig/symbolica/pi_7/cos` | 2.45 us | 2.45 us - 2.46 us | 2.44 us | - | - |
| `scalar_trig/symbolica/pi_7/sin` | 2.62 us | 2.61 us - 2.63 us | 2.61 us | - | - |
| `sentinel/complex/powi_zero_known_nonzero` | 30.79 ns | 30.75 ns - 30.83 ns | 30.72 ns | - | - |
| `sentinel/matrix3/dense_transform_batch_public` | 1.91 us | 1.90 us - 1.92 us | 1.90 us | - | - |
| `sentinel/matrix3/inverse_fractional` | 1.83 us | 1.82 us - 1.85 us | 1.80 us | - | - |
| `sentinel/matrix3/sparse_mask_product` | 374.18 ns | 372.17 ns - 376.45 ns | 370.20 ns | - | - |
| `sentinel/matrix4/determinant_fractional` | 1.16 us | 1.16 us - 1.17 us | 1.16 us | - | - |
| `sentinel/matrix4/diagonal_direction_batch` | 342.41 ns | 341.62 ns - 343.31 ns | 340.84 ns | - | - |
| `sentinel/matrix4/diagonal_point_batch` | 2.16 us | 2.16 us - 2.17 us | 2.15 us | - | - |
| `sentinel/matrix4/diagonal_unknown_batch` | 2.22 us | 2.20 us - 2.23 us | 2.20 us | - | - |
| `sentinel/matrix4/division_fractional` | 7.51 us | 7.50 us - 7.52 us | 7.50 us | - | - |
| `sentinel/matrix4/exact_facts` | 2.39 us | 2.38 us - 2.40 us | 2.37 us | - | - |
| `sentinel/matrix4/sparse_mask_product` | 574.30 ns | 573.45 ns - 575.25 ns | 573.07 ns | - | - |
| `sentinel/matrix4/translated_diagonal_direction_batch_public` | 2.32 us | 2.31 us - 2.33 us | 2.30 us | - | - |
| `sentinel/matrix4/translated_diagonal_direction_transform_public` | 140.74 ns | 140.35 ns - 141.20 ns | 140.04 ns | - | - |
| `sentinel/matrix4/translated_diagonal_point_batch_public` | 3.06 us | 3.04 us - 3.08 us | 3.04 us | - | - |
| `sentinel/matrix4/translated_diagonal_point_transform_public` | 192.90 ns | 191.89 ns - 194.00 ns | 193.97 ns | - | - |
| `sentinel/scalar/cancellation_zero_status` | 0.48 ns | 0.47 ns - 0.48 ns | 0.47 ns | - | - |
| `sentinel/scalar/sqrt2_minus_convergent_sign` | 7.52 ns | 7.48 ns - 7.55 ns | 7.54 ns | - | - |
| `sentinel/vector/dot_equal_distinct_cold_dyadic` | 340.92 ns | 338.27 ns - 343.40 ns | 340.80 ns | - | - |
| `sentinel/vector/dot_sparse_symbolic` | 35.63 ns | 35.54 ns - 35.73 ns | 35.47 ns | - | - |
| `sentinel/vector/inverse_magnitude_retained_dyadic` | 187.01 ns | 186.38 ns - 187.71 ns | 185.59 ns | - | - |
| `sentinel/vector/magnitude_retained_dyadic` | 165.55 ns | 165.09 ns - 166.04 ns | 164.77 ns | - | - |
| `sentinel/vector/normalize_cold_dyadic` | 1.45 us | 1.42 us - 1.51 us | 1.41 us | - | - |
| `sentinel/vector/normalize_retained_dyadic` | 389.66 ns | 387.54 ns - 391.99 ns | 386.00 ns | - | - |
| `sentinel/vector/self_dot_cold_dyadic` | 231.94 ns | 230.91 ns - 232.84 ns | 231.83 ns | - | - |
| `sentinel/vector/self_dot_retained_dyadic` | 90.26 ns | 88.49 ns - 91.96 ns | 92.78 ns | - | - |
| `sentinel/vector/shared_scale_owned_common_denominator` | 223.28 ns | 221.04 ns - 225.91 ns | 220.93 ns | - | - |
| `sentinel/vector/shared_scale_view_common_denominator` | 159.45 ns | 158.80 ns - 160.29 ns | 158.36 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 add` | 84.78 ns | 84.66 ns - 84.94 ns | 84.68 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 add_scalar` | 80.11 ns | 79.36 ns - 80.86 ns | 81.60 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 div_scalar` | 163.09 ns | 162.90 ns - 163.31 ns | 162.80 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 div_scalar_checked` | 162.93 ns | 162.62 ns - 163.30 ns | 162.68 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 div_scalar_checked_abort` | 162.63 ns | 162.43 ns - 162.87 ns | 162.57 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 dot_abort` | 150.78 ns | 150.61 ns - 150.98 ns | 150.59 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 magnitude_abort` | 277.40 ns | 276.92 ns - 277.98 ns | 276.68 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 mul_scalar` | 115.03 ns | 114.91 ns - 115.16 ns | 114.92 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 neg` | 51.61 ns | 51.25 ns - 52.05 ns | 51.38 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 new` | 55.34 ns | 55.23 ns - 55.46 ns | 55.05 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 normalize_checked` | 416.10 ns | 415.34 ns - 416.94 ns | 415.57 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 normalize_checked_abort` | 420.37 ns | 419.06 ns - 421.91 ns | 419.36 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 sub` | 81.50 ns | 81.32 ns - 81.70 ns | 81.30 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 sub_scalar` | 83.25 ns | 82.90 ns - 83.67 ns | 82.97 ns | - | - |
| `vector_ops/gmp_mpfr128/vec3 zero` | 26.65 ns | 26.54 ns - 26.78 ns | 26.40 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 add` | 98.68 ns | 98.15 ns - 99.34 ns | 97.93 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 add_scalar` | 97.06 ns | 96.89 ns - 97.28 ns | 96.75 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 div_scalar` | 217.79 ns | 217.32 ns - 218.34 ns | 216.90 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 dot` | 243.96 ns | 243.43 ns - 244.59 ns | 243.20 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 magnitude` | 344.03 ns | 343.64 ns - 344.49 ns | 343.41 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 mul_scalar` | 150.91 ns | 150.55 ns - 151.38 ns | 150.48 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 neg` | 66.35 ns | 66.20 ns - 66.52 ns | 66.16 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 normalize` | 535.57 ns | 534.58 ns - 536.72 ns | 533.78 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 sub` | 103.23 ns | 102.85 ns - 103.69 ns | 102.59 ns | - | - |
| `vector_ops/gmp_mpfr128/vec4 sub_scalar` | 97.49 ns | 97.12 ns - 97.92 ns | 96.66 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 add` | 126.59 ns | 125.81 ns - 127.43 ns | 125.10 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 add_scalar` | 172.96 ns | 172.40 ns - 173.58 ns | 171.70 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 div_scalar` | 136.27 ns | 135.88 ns - 136.71 ns | 135.70 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 div_scalar_checked` | 178.63 ns | 177.84 ns - 179.52 ns | 177.03 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 div_scalar_checked_abort` | 208.29 ns | 207.21 ns - 209.47 ns | 206.26 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 dot_abort` | 146.58 ns | 146.08 ns - 147.18 ns | 145.80 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 dot_abort_dense` | 320.05 ns | 319.12 ns - 321.06 ns | 319.46 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 dot_abort_sparse` | 93.15 ns | 92.73 ns - 93.82 ns | 92.60 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 dot_sparse` | 51.53 ns | 51.47 ns - 51.59 ns | 51.44 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 magnitude_abort` | 271.00 ns | 269.44 ns - 272.83 ns | 268.01 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 mul_scalar` | 334.20 ns | 333.28 ns - 335.32 ns | 332.82 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 neg` | 86.21 ns | 86.01 ns - 86.45 ns | 85.87 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 new` | 505.11 ns | 503.90 ns - 506.45 ns | 503.84 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 normalize_checked` | 1.39 us | 1.38 us - 1.40 us | 1.38 us | - | - |
| `vector_ops/hyperreal-rational/vec3 normalize_checked_abort` | 558.82 ns | 555.08 ns - 562.95 ns | 551.54 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 sub` | 125.26 ns | 124.92 ns - 125.66 ns | 124.57 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 sub_scalar` | 227.27 ns | 225.58 ns - 229.33 ns | 224.22 ns | - | - |
| `vector_ops/hyperreal-rational/vec3 zero` | 45.50 ns | 45.38 ns - 45.63 ns | 45.33 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 add` | 173.29 ns | 172.91 ns - 173.71 ns | 172.53 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 add_scalar` | 203.84 ns | 203.13 ns - 204.67 ns | 202.59 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 div_scalar` | 273.81 ns | 272.86 ns - 274.86 ns | 271.74 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 dot` | 137.72 ns | 137.24 ns - 138.46 ns | 136.98 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 dot_abort_dense` | 390.56 ns | 390.04 ns - 391.16 ns | 389.59 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 dot_abort_sparse` | 122.54 ns | 122.38 ns - 122.71 ns | 122.25 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 dot_sparse` | 49.74 ns | 49.67 ns - 49.82 ns | 49.65 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 magnitude` | 251.97 ns | 251.35 ns - 252.63 ns | 251.61 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 mul_scalar` | 464.31 ns | 462.03 ns - 466.78 ns | 459.85 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 neg` | 108.34 ns | 107.82 ns - 108.91 ns | 107.20 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 normalize` | 999.39 ns | 998.08 ns - 1.00 us | 998.93 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 sub` | 187.84 ns | 186.43 ns - 189.98 ns | 185.54 ns | - | - |
| `vector_ops/hyperreal-rational/vec4 sub_scalar` | 338.38 ns | 337.64 ns - 339.28 ns | 337.52 ns | - | - |
| `vector_ops/hyperreal/vec3 add` | 124.85 ns | 124.26 ns - 125.55 ns | 123.60 ns | - | - |
| `vector_ops/hyperreal/vec3 add_scalar` | 174.50 ns | 173.72 ns - 175.40 ns | 173.19 ns | - | - |
| `vector_ops/hyperreal/vec3 div_scalar` | 133.37 ns | 133.17 ns - 133.59 ns | 133.25 ns | - | - |
| `vector_ops/hyperreal/vec3 div_scalar_checked` | 174.71 ns | 173.99 ns - 175.54 ns | 173.39 ns | - | - |
| `vector_ops/hyperreal/vec3 div_scalar_checked_abort` | 200.07 ns | 199.56 ns - 200.67 ns | 199.22 ns | - | - |
| `vector_ops/hyperreal/vec3 dot_abort` | 234.81 ns | 234.43 ns - 235.29 ns | 234.14 ns | - | - |
| `vector_ops/hyperreal/vec3 dot_abort_dense` | 321.46 ns | 320.76 ns - 322.25 ns | 320.67 ns | - | - |
| `vector_ops/hyperreal/vec3 dot_abort_sparse` | 93.94 ns | 93.47 ns - 94.46 ns | 92.73 ns | - | - |
| `vector_ops/hyperreal/vec3 dot_sparse` | 51.79 ns | 51.62 ns - 51.99 ns | 51.46 ns | - | - |
| `vector_ops/hyperreal/vec3 magnitude_abort` | 265.72 ns | 264.80 ns - 266.78 ns | 264.57 ns | - | - |
| `vector_ops/hyperreal/vec3 mul_scalar` | 251.98 ns | 250.81 ns - 253.53 ns | 250.18 ns | - | - |
| `vector_ops/hyperreal/vec3 neg` | 87.50 ns | 87.27 ns - 87.75 ns | 87.05 ns | - | - |
| `vector_ops/hyperreal/vec3 new` | 187.95 ns | 187.68 ns - 188.24 ns | 187.61 ns | - | - |
| `vector_ops/hyperreal/vec3 normalize_checked` | 537.45 ns | 536.92 ns - 538.02 ns | 536.73 ns | - | - |
| `vector_ops/hyperreal/vec3 normalize_checked_abort` | 551.52 ns | 550.40 ns - 552.69 ns | 550.32 ns | - | - |
| `vector_ops/hyperreal/vec3 sub` | 126.15 ns | 125.65 ns - 126.77 ns | 125.43 ns | - | - |
| `vector_ops/hyperreal/vec3 sub_scalar` | 216.76 ns | 216.47 ns - 217.07 ns | 216.69 ns | - | - |
| `vector_ops/hyperreal/vec3 zero` | 45.87 ns | 45.72 ns - 46.04 ns | 45.55 ns | - | - |
| `vector_ops/hyperreal/vec4 add` | 179.28 ns | 178.79 ns - 179.82 ns | 178.19 ns | - | - |
| `vector_ops/hyperreal/vec4 add_scalar` | 292.57 ns | 291.33 ns - 293.96 ns | 290.84 ns | - | - |
| `vector_ops/hyperreal/vec4 div_scalar` | 252.94 ns | 252.13 ns - 253.85 ns | 251.35 ns | - | - |
| `vector_ops/hyperreal/vec4 dot` | 230.79 ns | 229.80 ns - 231.86 ns | 228.61 ns | - | - |
| `vector_ops/hyperreal/vec4 dot_abort_dense` | 421.33 ns | 420.08 ns - 422.86 ns | 419.56 ns | - | - |
| `vector_ops/hyperreal/vec4 dot_abort_sparse` | 122.45 ns | 122.33 ns - 122.58 ns | 122.35 ns | - | - |
| `vector_ops/hyperreal/vec4 dot_sparse` | 49.73 ns | 49.69 ns - 49.78 ns | 49.72 ns | - | - |
| `vector_ops/hyperreal/vec4 magnitude` | 238.57 ns | 237.65 ns - 239.63 ns | 237.02 ns | - | - |
| `vector_ops/hyperreal/vec4 mul_scalar` | 359.78 ns | 358.54 ns - 361.25 ns | 357.62 ns | - | - |
| `vector_ops/hyperreal/vec4 neg` | 106.68 ns | 106.50 ns - 106.90 ns | 106.43 ns | - | - |
| `vector_ops/hyperreal/vec4 normalize` | 459.98 ns | 457.23 ns - 463.38 ns | 456.40 ns | - | - |
| `vector_ops/hyperreal/vec4 sub` | 185.38 ns | 184.84 ns - 185.96 ns | 184.69 ns | - | - |
| `vector_ops/hyperreal/vec4 sub_scalar` | 318.26 ns | 317.24 ns - 319.55 ns | 316.92 ns | - | - |
| `vector_ops/numerica128/vec3 add` | 127.46 ns | 126.70 ns - 128.28 ns | 125.54 ns | - | - |
| `vector_ops/numerica128/vec3 add_scalar` | 136.70 ns | 135.62 ns - 137.95 ns | 134.57 ns | - | - |
| `vector_ops/numerica128/vec3 div_scalar` | 173.88 ns | 173.46 ns - 174.35 ns | 173.21 ns | - | - |
| `vector_ops/numerica128/vec3 div_scalar_checked` | 172.42 ns | 172.00 ns - 172.90 ns | 171.57 ns | - | - |
| `vector_ops/numerica128/vec3 div_scalar_checked_abort` | 174.10 ns | 173.41 ns - 174.87 ns | 172.65 ns | - | - |
| `vector_ops/numerica128/vec3 dot_abort` | 199.27 ns | 198.93 ns - 199.69 ns | 198.79 ns | - | - |
| `vector_ops/numerica128/vec3 magnitude_abort` | 322.70 ns | 321.55 ns - 324.03 ns | 320.78 ns | - | - |
| `vector_ops/numerica128/vec3 mul_scalar` | 123.89 ns | 123.40 ns - 124.46 ns | 123.00 ns | - | - |
| `vector_ops/numerica128/vec3 neg` | 51.61 ns | 51.18 ns - 52.12 ns | 50.67 ns | - | - |
| `vector_ops/numerica128/vec3 new` | 57.45 ns | 57.23 ns - 57.69 ns | 57.01 ns | - | - |
| `vector_ops/numerica128/vec3 normalize_checked` | 549.46 ns | 547.23 ns - 551.93 ns | 544.24 ns | - | - |
| `vector_ops/numerica128/vec3 normalize_checked_abort` | 548.48 ns | 545.66 ns - 551.81 ns | 543.38 ns | - | - |
| `vector_ops/numerica128/vec3 sub` | 136.88 ns | 136.47 ns - 137.34 ns | 136.19 ns | - | - |
| `vector_ops/numerica128/vec3 sub_scalar` | 125.70 ns | 125.24 ns - 126.22 ns | 124.85 ns | - | - |
| `vector_ops/numerica128/vec3 zero` | 29.92 ns | 29.80 ns - 30.07 ns | 29.83 ns | - | - |
| `vector_ops/numerica128/vec4 add` | 174.18 ns | 173.37 ns - 175.10 ns | 172.82 ns | - | - |
| `vector_ops/numerica128/vec4 add_scalar` | 175.90 ns | 175.59 ns - 176.27 ns | 175.45 ns | - | - |
| `vector_ops/numerica128/vec4 div_scalar` | 223.99 ns | 222.63 ns - 225.53 ns | 220.79 ns | - | - |
| `vector_ops/numerica128/vec4 dot` | 314.73 ns | 313.52 ns - 316.08 ns | 312.23 ns | - | - |
| `vector_ops/numerica128/vec4 magnitude` | 405.96 ns | 404.73 ns - 407.32 ns | 403.42 ns | - | - |
| `vector_ops/numerica128/vec4 mul_scalar` | 154.65 ns | 154.41 ns - 154.92 ns | 154.37 ns | - | - |
| `vector_ops/numerica128/vec4 neg` | 63.97 ns | 63.85 ns - 64.12 ns | 63.87 ns | - | - |
| `vector_ops/numerica128/vec4 normalize` | 695.20 ns | 693.29 ns - 697.43 ns | 692.34 ns | - | - |
| `vector_ops/numerica128/vec4 sub` | 176.48 ns | 175.56 ns - 177.57 ns | 174.75 ns | - | - |
| `vector_ops/numerica128/vec4 sub_scalar` | 168.10 ns | 167.87 ns - 168.40 ns | 167.85 ns | - | - |
| `vector_ops/symbolica/vec3 add` | 5.39 us | 5.35 us - 5.44 us | 5.32 us | - | - |
| `vector_ops/symbolica/vec3 add_scalar` | 5.16 us | 5.15 us - 5.18 us | 5.13 us | - | - |
| `vector_ops/symbolica/vec3 div_scalar` | 9.18 us | 9.14 us - 9.22 us | 9.10 us | - | - |
| `vector_ops/symbolica/vec3 div_scalar_checked` | 9.36 us | 9.28 us - 9.45 us | 9.19 us | - | - |
| `vector_ops/symbolica/vec3 div_scalar_checked_abort` | 9.13 us | 9.10 us - 9.16 us | 9.09 us | - | - |
| `vector_ops/symbolica/vec3 dot_abort` | 9.34 us | 9.30 us - 9.38 us | 9.29 us | - | - |
| `vector_ops/symbolica/vec3 magnitude_abort` | 11.70 us | 11.66 us - 11.73 us | 11.64 us | - | - |
| `vector_ops/symbolica/vec3 mul_scalar` | 5.74 us | 5.71 us - 5.79 us | 5.70 us | - | - |
| `vector_ops/symbolica/vec3 neg` | 4.48 us | 4.46 us - 4.51 us | 4.44 us | - | - |
| `vector_ops/symbolica/vec3 new` | 726.10 ns | 724.30 ns - 728.24 ns | 723.80 ns | - | - |
| `vector_ops/symbolica/vec3 normalize_checked` | 21.69 us | 21.51 us - 21.89 us | 21.32 us | - | - |
| `vector_ops/symbolica/vec3 normalize_checked_abort` | 21.42 us | 21.36 us - 21.49 us | 21.33 us | - | - |
| `vector_ops/symbolica/vec3 sub` | 8.80 us | 8.77 us - 8.83 us | 8.76 us | - | - |
| `vector_ops/symbolica/vec3 sub_scalar` | 8.62 us | 8.58 us - 8.65 us | 8.55 us | - | - |
| `vector_ops/symbolica/vec3 zero` | 2.84 ns | 2.83 ns - 2.85 ns | 2.82 ns | - | - |
| `vector_ops/symbolica/vec4 add` | 7.10 us | 7.07 us - 7.13 us | 7.03 us | - | - |
| `vector_ops/symbolica/vec4 add_scalar` | 6.86 us | 6.84 us - 6.87 us | 6.84 us | - | - |
| `vector_ops/symbolica/vec4 div_scalar` | 11.87 us | 11.84 us - 11.91 us | 11.84 us | - | - |
| `vector_ops/symbolica/vec4 dot` | 12.81 us | 12.76 us - 12.87 us | 12.73 us | - | - |
| `vector_ops/symbolica/vec4 magnitude` | 15.14 us | 15.07 us - 15.23 us | 15.03 us | - | - |
| `vector_ops/symbolica/vec4 mul_scalar` | 7.43 us | 7.41 us - 7.45 us | 7.40 us | - | - |
| `vector_ops/symbolica/vec4 neg` | 5.85 us | 5.82 us - 5.87 us | 5.80 us | - | - |
| `vector_ops/symbolica/vec4 normalize` | 28.11 us | 27.98 us - 28.25 us | 27.85 us | - | - |
| `vector_ops/symbolica/vec4 sub` | 11.85 us | 11.77 us - 11.95 us | 11.69 us | - | - |
| `vector_ops/symbolica/vec4 sub_scalar` | 11.49 us | 11.42 us - 11.57 us | 11.35 us | - | - |
| `vectors/gmp_mpfr128/vec3 dot` | 204.85 ns | 203.38 ns - 206.56 ns | 201.71 ns | +1.72% | - |
| `vectors/gmp_mpfr128/vec3 magnitude` | 305.95 ns | 304.76 ns - 307.41 ns | 303.42 ns | -0.70% | - |
| `vectors/gmp_mpfr128/vec3 normalize` | 473.44 ns | 470.98 ns - 476.05 ns | 467.38 ns | -1.25% | - |
| `vectors/hyperreal-rational/vec3 dot` | 170.67 ns | 170.20 ns - 171.22 ns | 169.80 ns | +0.49% | - |
| `vectors/hyperreal-rational/vec3 magnitude` | 178.98 ns | 178.23 ns - 180.00 ns | 177.79 ns | -2.19% | - |
| `vectors/hyperreal-rational/vec3 normalize` | 1.94 us | 1.94 us - 1.95 us | 1.93 us | +1.60% | - |
| `vectors/hyperreal/vec3 dot` | 231.04 ns | 230.87 ns - 231.22 ns | 231.00 ns | -1.30% | - |
| `vectors/hyperreal/vec3 magnitude` | 244.87 ns | 242.85 ns - 247.12 ns | 240.13 ns | +3.54% | - |
| `vectors/hyperreal/vec3 normalize` | 533.16 ns | 532.07 ns - 534.48 ns | 531.47 ns | +4.59% | - |
| `vectors/numerica128/vec3 dot` | 252.33 ns | 251.88 ns - 252.89 ns | 251.71 ns | +0.42% | - |
| `vectors/numerica128/vec3 magnitude` | 345.16 ns | 344.21 ns - 346.19 ns | 343.28 ns | -1.65% | - |
| `vectors/numerica128/vec3 normalize` | 596.89 ns | 593.65 ns - 600.71 ns | 590.85 ns | -0.42% | - |
| `vectors/symbolica/vec3 dot` | 9.44 us | 9.41 us - 9.49 us | 9.36 us | -0.04% | - |
| `vectors/symbolica/vec3 magnitude` | 11.66 us | 11.64 us - 11.69 us | 11.63 us | -0.78% | - |
| `vectors/symbolica/vec3 normalize` | 20.92 us | 20.88 us - 20.97 us | 20.85 us | -2.12% | - |

<!-- END COMPLETE BENCHMARK REPORT -->
