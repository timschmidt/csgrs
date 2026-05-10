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

The `mathbench` suite benchmarks both crate backends and writes this file from Criterion's median estimates after a real benchmark run. The `astro-float 128` and `numerica128` comparison columns run at 128-bit precision, while the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated, or that the external library does not expose a directly comparable operation in this suite.

Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, range-reduction-heavy trigonometric arguments, and boundary-adjacent inverse trigonometric and inverse hyperbolic values.

## Operation Coverage

- Scalar construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.
- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.
- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Scalar Operations

#### Scalar Trigonometric And Inverse Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 10.97 ns | 145.84 ns | 144.60 ns | 10.65 us | 784.55 ns | 1.82 us | 13.29x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 15.12 ns | 145.83 ns | 145.31 ns | 10.34 us | 498.48 ns | 1.66 us | 9.64x | 0.01x | 0.29x | 0.09x |
| `sin 1.23456789` | 13.98 ns | 199.81 ns | 183.64 ns | 12.38 us | 812.39 ns | 1.80 us | 14.29x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 12.19 ns | 193.53 ns | 178.93 ns | 10.59 us | 586.58 ns | 1.65 us | 15.88x | 0.02x | 0.33x | 0.12x |
| `sin 1e6` | 15.06 ns | 87.44 ns | 92.08 ns | 15.99 us | 1.09 us | 1.97 us | 5.81x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 15.11 ns | 88.19 ns | 88.68 ns | 13.69 us | 815.26 ns | 1.79 us | 5.84x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 65.48 ns | 87.72 ns | 89.43 ns | 18.22 us | 2.86 us | 3.54 us | 1.34x | 0.00x | 0.03x | 0.02x |
| `cos 1e30` | 71.69 ns | 88.46 ns | 88.32 ns | 15.39 us | 986.22 ns | 3.08 us | 1.23x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 14.68 ns | 145.54 ns | 348.44 ns | 12.39 us | 748.45 ns | 1.85 us | 9.92x | 0.01x | 0.19x | 0.08x |
| `cos pi_7` | 15.05 ns | 145.68 ns | 766.78 ns | 10.69 us | 533.42 ns | 1.69 us | 9.68x | 0.01x | 0.27x | 0.09x |
| `sin 1000pi_eps` | 11.92 ns | 87.40 ns | 795.37 ns | 16.10 us | 2.28 us | 2.79 us | 7.33x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 16.40 ns | 87.96 ns | 814.21 ns | 13.64 us | 568.25 ns | 1.68 us | 5.36x | 0.01x | 0.15x | 0.05x |
| `asin 0.5` | 10.85 ns | 452.80 ns | 451.06 ns | 49.62 us | 2.90 us | 13.09 us | 41.74x | 0.01x | 0.16x | 0.03x |
| `acos 0.5` | 11.03 ns | 987.61 ns | 993.15 ns | 59.08 us | 2.87 us | 12.90 us | 89.56x | 0.02x | 0.34x | 0.08x |
| `atanh 0.5` | 14.32 ns | 1.41 us | 1.42 us | 34.81 us | 1.67 us | 12.71 us | 98.19x | 0.04x | 0.84x | 0.11x |
| `asin neg_0.999999` | 13.85 ns | 757.40 ns | 1.20 us | 13.88 us | 2.44 us | 12.65 us | 54.67x | 0.05x | 0.31x | 0.06x |
| `acos neg_0.999999` | 14.97 ns | 861.11 ns | 1.72 us | 18.05 us | 2.66 us | 12.88 us | 57.53x | 0.05x | 0.32x | 0.07x |
| `atanh neg_0.999999` | 14.21 ns | 369.30 ns | 575.53 ns | 36.82 us | 1.58 us | 12.51 us | 25.99x | 0.01x | 0.23x | 0.03x |
| `asin 0.999999` | 13.58 ns | 729.21 ns | 1.40 us | 13.84 us | 2.46 us | 12.63 us | 53.72x | 0.05x | 0.30x | 0.06x |
| `acos 0.999999` | 13.93 ns | 609.15 ns | 1.50 us | 18.24 us | 2.70 us | 12.65 us | 43.73x | 0.03x | 0.23x | 0.05x |
| `atanh 0.999999` | 14.30 ns | 429.55 ns | 856.79 ns | 31.74 us | 1.58 us | 12.37 us | 30.03x | 0.01x | 0.27x | 0.03x |
| `asin 1e-12` | 9.23 ns | 448.99 ns | 1.09 us | 8.06 us | 1.38 us | 14.95 us | 48.67x | 0.06x | 0.32x | 0.03x |
| `acos 1e-12` | 9.79 ns | 708.64 ns | 1.56 us | 9.82 us | 1.40 us | 14.93 us | 72.36x | 0.07x | 0.51x | 0.05x |
| `atanh 1e-12` | 9.47 ns | 424.79 ns | 824.69 ns | 36.12 us | 171.24 ns | 19.91 us | 44.88x | 0.01x | 2.48x | 0.02x |
| `atan 0.5` | 14.66 ns | 286.76 ns | 288.84 ns | 36.52 us | 2.71 us | 17.07 us | 19.56x | 0.01x | 0.11x | 0.02x |
| `asinh 0.5` | 26.43 ns | 415.47 ns | 421.77 ns | 39.51 us | 1.65 us | 7.42 us | 15.72x | 0.01x | 0.25x | 0.06x |
| `atan neg_1e-12` | 14.12 ns | 321.01 ns | 545.05 ns | 1.65 us | 1.12 us | 14.98 us | 22.73x | 0.19x | 0.29x | 0.02x |
| `asinh neg_1e-12` | 15.61 ns | 468.26 ns | 383.81 ns | 42.22 us | 8.53 us | 11.94 us | 29.99x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 19.27 ns | 176.29 ns | 177.39 ns | 3.02 us | 1.39 us | 17.55 us | 9.15x | 0.06x | 0.13x | 0.01x |
| `asinh 1e6` | 26.30 ns | 294.68 ns | 299.25 ns | 36.80 us | 1.61 us | 7.23 us | 11.20x | 0.01x | 0.18x | 0.04x |
| `atan neg_1e6` | 18.98 ns | 278.07 ns | 277.61 ns | 2.91 us | 1.39 us | 17.62 us | 14.65x | 0.10x | 0.20x | 0.02x |
| `asinh neg_1e6` | 26.70 ns | 341.87 ns | 340.28 ns | 36.98 us | 1.63 us | 7.12 us | 12.80x | 0.01x | 0.21x | 0.05x |
| `acosh 9` | 13.03 ns | 169.96 ns | 171.41 ns | 42.46 us | 1.60 us | 9.68 us | 13.05x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 12.05 ns | 510.16 ns | 1.43 us | 42.25 us | 8.30 us | 11.55 us | 42.35x | 0.01x | 0.06x | 0.04x |
| `acosh 1e6` | 12.92 ns | 170.52 ns | 171.29 ns | 37.51 us | 1.58 us | 9.76 us | 13.20x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 13.13 ns | 508.32 ns | 2.31 us | 41.90 us | 1.62 us | 9.59 us | 38.72x | 0.01x | 0.31x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.53 ns | 18.58 ns | 26.82 ns | 15.33 ns | 0.94 ns | 39.52x | 0.69x | 1.21x | 19.73x |
| `one` | 0.47 ns | 23.43 ns | 24.33 ns | 44.81 ns | 30.20 ns | 29.81 ns | 50.34x | 0.52x | 0.78x | 0.79x |
| `e` | 0.48 ns | 88.23 ns | 87.08 ns | 56.26 ns | 1.04 us | 225.49 ns | 184.41x | 1.57x | 0.08x | 0.39x |
| `pi` | 0.47 ns | 50.54 ns | 49.00 ns | 46.24 ns | 47.78 ns | 227.17 ns | 106.62x | 1.09x | 1.06x | 0.22x |
| `tau` | 0.48 ns | 50.88 ns | 49.11 ns | 118.59 ns | 98.72 ns | 1.86 us | 107.09x | 0.43x | 0.52x | 0.03x |
| `add` | 5.11 ns | 212.55 ns | 327.87 ns | 52.32 ns | 45.22 ns | 1.26 us | 41.56x | 4.06x | 4.70x | 0.17x |
| `sub` | 5.25 ns | 219.49 ns | 377.63 ns | 56.64 ns | 46.63 ns | 2.38 us | 41.77x | 3.88x | 4.71x | 0.09x |
| `neg` | 5.12 ns | 38.98 ns | 38.71 ns | 23.07 ns | 21.35 ns | 1.05 us | 7.61x | 1.69x | 1.83x | 0.04x |
| `mul` | 5.41 ns | 137.80 ns | 408.18 ns | 57.68 ns | 45.19 ns | 1.51 us | 25.48x | 2.39x | 3.05x | 0.09x |
| `div` | 8.51 ns | 313.44 ns | 361.39 ns | 135.07 ns | 62.81 ns | 2.53 us | 36.83x | 2.32x | 4.99x | 0.12x |
| `reciprocal` | 8.88 ns | 80.94 ns | 81.76 ns | 158.87 ns | 57.94 ns | 1.51 us | 9.11x | 0.51x | 1.40x | 0.05x |
| `reciprocal checked` | 9.14 ns | 81.84 ns | 83.39 ns | 158.81 ns | 58.37 ns | 1.51 us | 8.95x | 0.52x | 1.40x | 0.05x |
| `reciprocal checked abort` | 24.21 ns | 90.79 ns | 87.73 ns | 159.23 ns | 58.25 ns | 1.50 us | 3.75x | 0.57x | 1.56x | 0.06x |
| `pow` | 30.07 ns | 8.31 us | 7.38 us | 55.07 us | 2.84 us | 2.31 us | 276.32x | 0.15x | 2.92x | 3.60x |
| `powi` | 6.00 ns | 372.17 ns | 2.64 us | 278.36 ns | 83.95 ns | 1.61 us | 62.07x | 1.34x | 4.43x | 0.23x |
| `exp` | 19.69 ns | 1.49 us | 2.12 us | 13.95 us | 908.71 ns | 1.85 us | 75.53x | 0.11x | 1.64x | 0.81x |
| `ln` | 11.27 ns | 1.56 us | 2.43 us | 29.43 us | 1.32 us | 1.79 us | 138.60x | 0.05x | 1.18x | 0.87x |
| `log10` | 13.79 ns | 4.59 us | 5.89 us | 36.63 us | 2.78 us | 6.55 us | 332.76x | 0.13x | 1.65x | 0.70x |
| `log10 abort` | 16.77 ns | 4.70 us | 5.91 us | 35.21 us | 2.77 us | 6.53 us | 280.29x | 0.13x | 1.70x | 0.72x |
| `sqrt` | 8.44 ns | 1.62 us | 1.65 us | 4.93 us | 99.90 ns | 1.45 us | 191.97x | 0.33x | 16.22x | 1.12x |
| `sin` | 14.82 ns | 118.97 ns | 122.38 ns | 13.88 us | 1.23 us | 2.19 us | 8.03x | 0.01x | 0.10x | 0.05x |
| `cos` | 18.01 ns | 118.36 ns | 120.75 ns | 12.05 us | 627.52 ns | 1.73 us | 6.57x | 0.01x | 0.19x | 0.07x |
| `tan` | 24.71 ns | 169.33 ns | 172.12 ns | 29.16 us | 1.58 us | 6.53 us | 6.85x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.12 ns | 4.13 us | 4.15 us | 3.41 us | 1.11 us | 10.62 us | 227.64x | 1.21x | 3.73x | 0.39x |
| `cosh` | 18.24 ns | 3.99 us | 4.16 us | 7.96 us | 1.05 us | 9.43 us | 218.68x | 0.50x | 3.82x | 0.42x |
| `tanh` | 25.35 ns | 8.29 us | 8.25 us | 3.42 us | 1.18 us | 22.71 us | 326.94x | 2.42x | 7.01x | 0.37x |
| `asin` | 13.17 ns | 630.58 ns | 1.48 us | 20.97 us | 2.37 us | 13.38 us | 47.88x | 0.03x | 0.27x | 0.05x |
| `asin abort` | 17.25 ns | 631.27 ns | 1.51 us | 20.99 us | 2.33 us | 13.44 us | 36.60x | 0.03x | 0.27x | 0.05x |
| `acos` | 14.64 ns | 837.02 ns | 2.12 us | 26.32 us | 2.50 us | 13.47 us | 57.16x | 0.03x | 0.33x | 0.06x |
| `acos abort` | 17.41 ns | 852.18 ns | 2.15 us | 26.34 us | 2.47 us | 13.50 us | 48.94x | 0.03x | 0.35x | 0.06x |
| `atan` | 15.03 ns | 273.12 ns | 943.46 ns | 18.42 us | 2.21 us | 18.49 us | 18.17x | 0.01x | 0.12x | 0.01x |
| `atan abort` | 21.43 ns | 280.42 ns | 951.92 ns | 18.25 us | 2.20 us | 18.54 us | 13.09x | 0.02x | 0.13x | 0.02x |
| `asinh` | 30.50 ns | 403.99 ns | 1.08 us | 38.24 us | 1.66 us | 7.37 us | 13.24x | 0.01x | 0.24x | 0.05x |
| `asinh abort` | 31.11 ns | 422.21 ns | 1.11 us | 38.83 us | 1.65 us | 7.52 us | 13.57x | 0.01x | 0.26x | 0.06x |
| `acosh` | 12.65 ns | 340.46 ns | 1.20 us | 39.78 us | 3.35 us | 10.38 us | 26.91x | 0.01x | 0.10x | 0.03x |
| `acosh abort` | 16.73 ns | 351.22 ns | 1.14 us | 40.03 us | 3.34 us | 10.43 us | 21.00x | 0.01x | 0.11x | 0.03x |
| `atanh` | 13.63 ns | 688.66 ns | 1.39 us | 34.82 us | 1.28 us | 14.57 us | 50.52x | 0.02x | 0.54x | 0.05x |
| `atanh abort` | 16.84 ns | 692.28 ns | 1.40 us | 34.71 us | 1.28 us | 14.60 us | 41.10x | 0.02x | 0.54x | 0.05x |
| `zero status` | 1.20 ns | 1.95 ns | 1.94 ns | 1.05 ns | 6.72 ns | 8.14 ns | 1.62x | 1.85x | 0.29x | 0.24x |
| `zero status abort` | 1.41 ns | 3.26 ns | 3.28 ns | 1.06 ns | 6.70 ns | 8.13 ns | 2.31x | 3.08x | 0.49x | 0.40x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 35.77 ns | 35.46 ns | 49.17 ns | - | 1.86 ns | 38.38x | 0.73x | - | 19.22x |
| `one` | 5.32 ns | 38.89 ns | 38.89 ns | 63.90 ns | - | 30.68 ns | 7.30x | 0.61x | - | 1.27x |
| `i` | 0.94 ns | 40.35 ns | 41.93 ns | 63.12 ns | - | 29.61 ns | 43.09x | 0.64x | - | 1.36x |
| `free i` | 0.93 ns | 42.11 ns | 42.18 ns | 63.16 ns | - | 29.74 ns | 45.05x | 0.67x | - | 1.42x |
| `conjugate` | 2.09 ns | 84.69 ns | 84.76 ns | 41.00 ns | - | 1.06 us | 40.48x | 2.07x | - | 0.08x |
| `norm squared` | 5.74 ns | 322.95 ns | 1.44 us | 150.33 ns | - | 4.25 us | 56.24x | 2.15x | - | 0.08x |
| `reciprocal` | 16.79 ns | 1.72 us | 2.92 us | 439.82 ns | - | 10.63 us | 102.30x | 3.90x | - | 0.16x |
| `reciprocal checked` | 15.54 ns | 1.76 us | 2.91 us | 439.80 ns | - | 10.58 us | 113.00x | 3.99x | - | 0.17x |
| `powi` | 17.37 ns | 2.04 us | 13.49 us | 1.40 us | - | 43.08 us | 117.36x | 1.45x | - | 0.05x |
| `powi checked` | 17.86 ns | 2.04 us | 13.56 us | 1.40 us | - | 43.08 us | 114.31x | 1.45x | - | 0.05x |
| `div checked` | 18.30 ns | 2.71 us | 6.32 us | 789.96 ns | - | 21.50 us | 147.96x | 3.43x | - | 0.13x |
| `div real checked` | 9.27 ns | 674.56 ns | 642.43 ns | 264.40 ns | - | 5.12 us | 72.79x | 2.55x | - | 0.13x |
| `from scalar` | 1.41 ns | 56.12 ns | 54.89 ns | 44.37 ns | - | 9.92 ns | 39.80x | 1.26x | - | 5.66x |
| `add` | 6.03 ns | 468.95 ns | 843.67 ns | 105.74 ns | - | 2.55 us | 77.74x | 4.43x | - | 0.18x |
| `sub` | 6.09 ns | 473.03 ns | 867.06 ns | 117.91 ns | - | 4.68 us | 77.67x | 4.01x | - | 0.10x |
| `neg` | 2.57 ns | 86.20 ns | 84.24 ns | 42.22 ns | - | 2.10 us | 33.57x | 2.04x | - | 0.04x |
| `mul` | 7.69 ns | 861.30 ns | 3.15 us | 312.84 ns | - | 9.81 us | 112.07x | 2.75x | - | 0.09x |
| `div` | 18.20 ns | 2.71 us | 6.35 us | 809.63 ns | - | 21.16 us | 148.88x | 3.35x | - | 0.13x |
| `div real` | 9.85 ns | 655.20 ns | 642.66 ns | 265.57 ns | - | 5.02 us | 66.54x | 2.47x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.78 ns | 204.88 ns | 2.38 us | 305.48 ns | 248.88 ns | 7.01 us | 30.24x | 0.67x | 0.82x | 0.03x |
| `vec3 magnitude` | 9.34 ns | 4.13 us | 5.84 us | 5.44 us | 344.68 ns | 8.54 us | 442.00x | 0.76x | 11.97x | 0.48x |
| `vec3 normalize` | 25.18 ns | 8.42 us | 11.96 us | 5.95 us | 588.02 ns | 16.41 us | 334.33x | 1.42x | 14.32x | 0.51x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.05 ns | 221.54 ns | 1.47 us | 69.37 ns | 55.95 ns | 710.86 ns | 72.57x | 3.19x | 3.96x | 0.31x |
| `vec3 zero` | 1.42 ns | 148.49 ns | 147.25 ns | 63.83 ns | 30.44 ns | 2.80 ns | 104.37x | 2.33x | 4.88x | 53.04x |
| `vec3 dot abort` | 7.44 ns | 208.52 ns | 901.18 ns | 256.24 ns | 199.65 ns | 7.06 us | 28.03x | 0.81x | 1.04x | 0.03x |
| `vec3 magnitude abort` | 15.40 ns | 4.18 us | 3.54 us | 5.44 us | 320.84 ns | 8.54 us | 271.05x | 0.77x | 13.01x | 0.49x |
| `vec3 normalize checked` | 25.63 ns | 8.47 us | 7.78 us | 5.92 us | 554.25 ns | 16.53 us | 330.54x | 1.43x | 15.29x | 0.51x |
| `vec3 normalize checked abort` | 28.73 ns | 8.39 us | 7.71 us | 5.93 us | 549.56 ns | 16.50 us | 292.06x | 1.42x | 15.27x | 0.51x |
| `vec3 div scalar checked` | 15.79 ns | 1.36 us | 1.61 us | 397.70 ns | - | - | 86.03x | 3.42x | - | - |
| `vec3 div scalar checked abort` | 18.42 ns | 1.35 us | 1.62 us | 393.03 ns | - | - | 73.03x | 3.42x | - | - |
| `vec3 add` | 6.74 ns | 946.84 ns | 1.74 us | 150.82 ns | 126.72 ns | 3.83 us | 140.56x | 6.28x | 7.47x | 0.25x |
| `vec3 add scalar` | 6.41 ns | 759.53 ns | 1.44 us | 153.69 ns | 132.24 ns | 3.69 us | 118.41x | 4.94x | 5.74x | 0.21x |
| `vec3 sub` | 6.77 ns | 944.86 ns | 1.77 us | 166.01 ns | 137.06 ns | 7.24 us | 139.51x | 5.69x | 6.89x | 0.13x |
| `vec3 sub scalar` | 6.34 ns | 750.21 ns | 1.24 us | 146.88 ns | 125.44 ns | 6.95 us | 118.33x | 5.11x | 5.98x | 0.11x |
| `vec3 neg` | 3.70 ns | 142.04 ns | 141.84 ns | 57.44 ns | 49.64 ns | 3.07 us | 38.39x | 2.47x | 2.86x | 0.05x |
| `vec3 mul scalar` | 6.88 ns | 518.65 ns | 2.04 us | 164.69 ns | 124.44 ns | 4.29 us | 75.42x | 3.15x | 4.17x | 0.12x |
| `vec3 div scalar` | 10.05 ns | 1.35 us | 1.61 us | 393.43 ns | 179.80 ns | 7.36 us | 134.64x | 3.44x | 7.53x | 0.18x |
| `vec4 dot` | 7.26 ns | 245.43 ns | 627.87 ns | 444.19 ns | 322.33 ns | 9.51 us | 33.82x | 0.55x | 0.76x | 0.03x |
| `vec4 magnitude` | 12.86 ns | 3.88 us | 2.84 us | 5.62 us | 408.13 ns | 10.95 us | 302.06x | 0.69x | 9.51x | 0.35x |
| `vec4 normalize` | 35.67 ns | 8.74 us | 6.79 us | 6.18 us | 770.59 ns | 21.47 us | 244.90x | 1.41x | 11.34x | 0.41x |
| `vec4 add` | 7.32 ns | 1.18 us | 1.80 us | 206.92 ns | 176.58 ns | 5.10 us | 161.15x | 5.70x | 6.68x | 0.23x |
| `vec4 add scalar` | 6.85 ns | 1.03 us | 1.73 us | 218.29 ns | 182.00 ns | 4.93 us | 150.61x | 4.73x | 5.67x | 0.21x |
| `vec4 sub` | 4.99 ns | 1.17 us | 1.72 us | 213.05 ns | 182.32 ns | 9.36 us | 234.76x | 5.50x | 6.42x | 0.13x |
| `vec4 sub scalar` | 4.41 ns | 1.01 us | 1.52 us | 203.22 ns | 175.53 ns | 9.10 us | 228.33x | 4.95x | 5.73x | 0.11x |
| `vec4 neg` | 4.90 ns | 195.89 ns | 194.84 ns | 79.07 ns | 62.15 ns | 3.94 us | 39.95x | 2.48x | 3.15x | 0.05x |
| `vec4 mul scalar` | 7.30 ns | 711.02 ns | 2.12 us | 216.24 ns | 158.81 ns | 5.51 us | 97.43x | 3.29x | 4.48x | 0.13x |
| `vec4 div scalar` | 14.36 ns | 1.75 us | 1.56 us | 523.88 ns | 227.94 ns | 9.60 us | 121.79x | 3.34x | 7.68x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.31 ns | 730.73 ns | 2.15 us | 934.04 ns | 841.33 ns | 22.09 us | 59.34x | 0.78x | 0.87x | 0.03x |
| `mat3 inverse` | 78.18 ns | 15.08 us | 8.98 us | 3.10 us | 2.44 us | 80.72 us | 192.91x | 4.87x | 6.19x | 0.19x |
| `mat3 mul mat3` | 35.09 ns | 3.00 us | 7.03 us | 2.74 us | 2.35 us | 59.87 us | 85.39x | 1.09x | 1.27x | 0.05x |
| `mat3 transform vec3` | 15.98 ns | 1.36 us | 4.53 us | 1.03 us | 881.38 ns | 19.44 us | 85.40x | 1.33x | 1.55x | 0.07x |
| `mat4 determinant` | 35.83 ns | 2.20 us | 1.76 us | 4.59 us | 4.16 us | 93.33 us | 61.48x | 0.48x | 0.53x | 0.02x |
| `mat4 inverse` | 177.12 ns | 24.71 us | 8.46 us | 11.22 us | 9.11 us | 329.60 us | 139.52x | 2.20x | 2.71x | 0.07x |
| `mat4 mul mat4` | 77.84 ns | 5.51 us | 4.18 us | 6.38 us | 5.28 us | 136.30 us | 70.78x | 0.86x | 1.04x | 0.04x |
| `mat4 transform vec4` | 26.75 ns | 2.19 us | 1.75 us | 1.89 us | 1.62 us | 34.36 us | 81.79x | 1.16x | 1.35x | 0.06x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.25 ns | 1.09 us | 4.22 us | 213.91 ns | 193.11 ns | 2.06 us | 30.85x | 5.08x | 5.63x | 0.53x |
| `mat3 zero` | 16.09 ns | 508.37 ns | 513.94 ns | 265.13 ns | 162.58 ns | 12.25 ns | 31.60x | 1.92x | 3.13x | 41.49x |
| `mat3 identity` | 9.49 ns | 661.54 ns | 662.33 ns | 325.32 ns | 207.81 ns | 140.31 ns | 69.74x | 2.03x | 3.18x | 4.71x |
| `mat3 transpose` | 8.80 ns | 722.93 ns | 712.08 ns | 242.28 ns | 182.99 ns | 120.17 ns | 82.15x | 2.98x | 3.95x | 6.02x |
| `mat3 reciprocal` | 78.73 ns | 15.25 us | 25.37 us | 2.90 us | 2.26 us | 80.97 us | 193.64x | 5.25x | 6.73x | 0.19x |
| `mat3 reciprocal checked` | 78.56 ns | 15.04 us | 25.35 us | 2.94 us | 2.27 us | 80.73 us | 191.50x | 5.12x | 6.64x | 0.19x |
| `mat3 inverse checked` | 78.45 ns | 15.25 us | 25.31 us | 2.93 us | 2.26 us | 80.19 us | 194.34x | 5.21x | 6.74x | 0.19x |
| `mat3 inverse checked abort` | 114.00 ns | 15.36 us | 25.32 us | 2.89 us | 2.27 us | 80.97 us | 134.74x | 5.32x | 6.76x | 0.19x |
| `mat3 powi` | 90.08 ns | 5.83 us | 44.42 us | 6.81 us | 6.08 us | 147.79 us | 64.69x | 0.86x | 0.96x | 0.04x |
| `mat3 powi checked` | 89.05 ns | 5.68 us | 44.61 us | 6.83 us | 6.05 us | 147.70 us | 63.74x | 0.83x | 0.94x | 0.04x |
| `mat3 powi checked abort` | 89.55 ns | 5.65 us | 46.31 us | 6.81 us | 6.09 us | 147.68 us | 63.04x | 0.83x | 0.93x | 0.04x |
| `mat3 div scalar checked` | 24.45 ns | 4.33 us | 5.55 us | 1.43 us | 800.91 ns | 21.71 us | 177.22x | 3.02x | 5.41x | 0.20x |
| `mat3 div scalar checked abort` | 29.87 ns | 4.45 us | 5.72 us | 1.42 us | 798.27 ns | 21.42 us | 148.84x | 3.14x | 5.57x | 0.21x |
| `mat3 div matrix checked` | 117.77 ns | 18.63 us | 58.04 us | 5.31 us | 4.36 us | 152.52 us | 158.22x | 3.51x | 4.28x | 0.12x |
| `mat3 div matrix checked abort` | 129.51 ns | 18.73 us | 58.16 us | 5.34 us | 4.39 us | 154.75 us | 144.62x | 3.50x | 4.27x | 0.12x |
| `mat3 add` | 14.57 ns | 3.22 us | 5.27 us | 508.48 ns | 477.22 ns | 11.31 us | 220.83x | 6.33x | 6.74x | 0.28x |
| `mat3 add scalar` | 11.99 ns | 2.48 us | 4.56 us | 774.75 ns | 692.11 ns | 11.59 us | 206.76x | 3.20x | 3.58x | 0.21x |
| `mat3 sub` | 12.98 ns | 3.20 us | 5.16 us | 535.46 ns | 507.15 ns | 20.84 us | 246.60x | 5.98x | 6.31x | 0.15x |
| `mat3 sub scalar` | 10.74 ns | 2.50 us | 4.44 us | 788.45 ns | 697.48 ns | 20.98 us | 232.92x | 3.17x | 3.59x | 0.12x |
| `mat3 neg` | 10.12 ns | 593.33 ns | 584.53 ns | 470.44 ns | 444.87 ns | 8.38 us | 58.63x | 1.26x | 1.33x | 0.07x |
| `mat3 mul scalar` | 13.53 ns | 1.81 us | 5.86 us | 753.91 ns | 655.85 ns | 11.74 us | 134.15x | 2.41x | 2.77x | 0.15x |
| `mat3 div scalar` | 24.38 ns | 4.36 us | 5.64 us | 1.41 us | 800.96 ns | 21.59 us | 178.99x | 3.08x | 5.45x | 0.20x |
| `mat3 div matrix` | 128.10 ns | 18.65 us | 58.37 us | 5.36 us | 4.38 us | 155.32 us | 145.55x | 3.48x | 4.25x | 0.12x |
| `mat3 bitxor` | 89.40 ns | 5.66 us | 45.13 us | 6.75 us | 6.09 us | 147.74 us | 63.35x | 0.84x | 0.93x | 0.04x |
| `mat4 zero` | 11.54 ns | 925.80 ns | 912.15 ns | 447.14 ns | 353.96 ns | 14.24 ns | 80.22x | 2.07x | 2.62x | 65.01x |
| `mat4 identity` | 10.69 ns | 1.12 us | 1.10 us | 569.20 ns | 422.28 ns | 212.78 ns | 105.01x | 1.97x | 2.66x | 5.28x |
| `mat4 transpose` | 9.28 ns | 1.09 us | 1.08 us | 419.01 ns | 369.87 ns | 181.91 ns | 117.26x | 2.60x | 2.94x | 5.98x |
| `mat4 reciprocal` | 160.75 ns | 24.85 us | 42.04 us | 10.67 us | 8.77 us | 333.13 us | 154.59x | 2.33x | 2.83x | 0.07x |
| `mat4 reciprocal checked` | 152.80 ns | 24.92 us | 42.17 us | 10.67 us | 8.80 us | 330.18 us | 163.08x | 2.34x | 2.83x | 0.08x |
| `mat4 powi` | 168.12 ns | 10.36 us | 53.38 us | 15.50 us | 13.91 us | 339.74 us | 61.65x | 0.67x | 0.75x | 0.03x |
| `mat4 powi checked` | 168.24 ns | 10.36 us | 53.38 us | 15.50 us | 13.96 us | 340.51 us | 61.58x | 0.67x | 0.74x | 0.03x |
| `mat4 add` | 52.02 ns | 5.00 us | 5.85 us | 965.94 ns | 837.22 ns | 18.97 us | 96.08x | 5.17x | 5.97x | 0.26x |
| `mat4 add scalar` | 20.36 ns | 4.06 us | 6.64 us | 1.45 us | 1.18 us | 19.85 us | 199.64x | 2.81x | 3.44x | 0.20x |
| `mat4 sub` | 38.54 ns | 5.00 us | 5.87 us | 998.01 ns | 902.22 ns | 35.19 us | 129.75x | 5.01x | 5.54x | 0.14x |
| `mat4 sub scalar` | 14.99 ns | 4.08 us | 6.50 us | 1.46 us | 1.16 us | 36.14 us | 272.41x | 2.80x | 3.52x | 0.11x |
| `mat4 neg` | 14.39 ns | 997.96 ns | 994.17 ns | 937.50 ns | 730.18 ns | 13.85 us | 69.37x | 1.06x | 1.37x | 0.07x |
| `mat4 mul scalar` | 24.39 ns | 2.93 us | 7.56 us | 1.42 us | 1.11 us | 19.71 us | 120.27x | 2.06x | 2.64x | 0.15x |
| `mat4 div scalar` | 33.38 ns | 7.50 us | 7.58 us | 2.61 us | 1.43 us | 36.58 us | 224.81x | 2.88x | 5.24x | 0.21x |
| `mat4 div matrix` | 190.99 ns | 28.79 us | 86.75 us | 16.66 us | 14.34 us | 516.77 us | 150.76x | 1.73x | 2.01x | 0.06x |
| `mat4 bitxor` | 167.99 ns | 10.30 us | 53.79 us | 15.47 us | 14.08 us | 339.36 us | 61.34x | 0.67x | 0.73x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.62 ns | 175.99 ns | 279.50 ns | - | - | 1.26 us | 48.58x | - | - | 0.14x |
| `scalar add ref_owned` | 12.17 ns | 173.97 ns | 280.60 ns | - | - | 1.26 us | 14.29x | - | - | 0.14x |
| `scalar add refs` | 5.30 ns | 158.62 ns | 262.12 ns | - | - | 1.25 us | 29.93x | - | - | 0.13x |
| `scalar add owned_ref_with_clone` | 8.94 ns | 192.57 ns | 302.47 ns | - | - | - | 21.55x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.49 ns | 183.52 ns | 296.57 ns | - | - | - | 33.44x | - | - | - |
| `scalar sub owned_ref` | 3.96 ns | 181.51 ns | 342.14 ns | - | - | 2.37 us | 45.80x | - | - | 0.08x |
| `scalar sub ref_owned` | 12.24 ns | 179.79 ns | 351.00 ns | - | - | 2.42 us | 14.69x | - | - | 0.07x |
| `scalar sub refs` | 5.52 ns | 162.31 ns | 323.73 ns | - | - | 2.38 us | 29.41x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 8.94 ns | 199.20 ns | 356.87 ns | - | - | - | 22.28x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.65 ns | 192.21 ns | 354.95 ns | - | - | - | 34.01x | - | - | - |
| `scalar mul owned_ref` | 4.52 ns | 95.11 ns | 388.29 ns | - | - | 1.51 us | 21.03x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.31 ns | 96.63 ns | 393.67 ns | - | - | 1.51 us | 7.26x | - | - | 0.06x |
| `scalar mul refs` | 6.11 ns | 80.20 ns | 357.11 ns | - | - | 1.50 us | 13.13x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.67 ns | 108.92 ns | 385.81 ns | - | - | - | 11.26x | - | - | - |
| `scalar mul ref_owned_with_clone` | 7.76 ns | 109.31 ns | 382.66 ns | - | - | - | 14.08x | - | - | - |
| `scalar div owned_ref` | 5.92 ns | 258.89 ns | 319.75 ns | - | - | 2.50 us | 43.74x | - | - | 0.10x |
| `scalar div ref_owned` | 17.07 ns | 261.95 ns | 325.91 ns | - | - | 2.51 us | 15.34x | - | - | 0.10x |
| `scalar div refs` | 6.84 ns | 248.35 ns | 306.32 ns | - | - | 2.50 us | 36.30x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.36 ns | 277.26 ns | 337.20 ns | - | - | - | 18.05x | - | - | - |
| `scalar div ref_owned_with_clone` | 8.05 ns | 274.36 ns | 341.20 ns | - | - | - | 34.09x | - | - | - |
| `vec3 add refs` | 6.10 ns | 637.16 ns | 1.40 us | - | - | 3.87 us | 104.46x | - | - | 0.16x |
| `vec3 sub refs` | 6.08 ns | 629.15 ns | 1.45 us | - | - | 7.16 us | 103.54x | - | - | 0.09x |
| `vec3 neg ref` | 3.26 ns | 171.80 ns | 158.88 ns | - | - | 3.12 us | 52.68x | - | - | 0.06x |
| `vec3 add_scalar_ref` | 6.44 ns | 794.52 ns | 1.46 us | - | - | 3.72 us | 123.45x | - | - | 0.21x |
| `vec3 sub_scalar_ref` | 6.59 ns | 781.19 ns | 1.25 us | - | - | 6.89 us | 118.60x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 6.83 ns | 534.95 ns | 2.09 us | - | - | 4.35 us | 78.27x | - | - | 0.12x |
| `vec3 div_scalar_ref` | 8.27 ns | 1.29 us | 1.58 us | - | - | 7.46 us | 156.13x | - | - | 0.17x |
| `vec4 add refs` | 6.75 ns | 756.56 ns | 1.39 us | - | - | 5.22 us | 112.12x | - | - | 0.14x |
| `vec4 sub refs` | 3.05 ns | 754.67 ns | 1.29 us | - | - | 9.43 us | 247.16x | - | - | 0.08x |
| `vec4 neg ref` | 4.21 ns | 203.99 ns | 203.81 ns | - | - | 4.03 us | 48.50x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 7.52 ns | 984.53 ns | 1.71 us | - | - | 4.99 us | 130.84x | - | - | 0.20x |
| `vec4 sub_scalar_ref` | 4.18 ns | 981.11 ns | 1.48 us | - | - | 9.15 us | 234.87x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.33 ns | 656.36 ns | 2.10 us | - | - | 5.59 us | 89.51x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.69 ns | 1.75 us | 1.56 us | - | - | 10.15 us | 149.26x | - | - | 0.17x |
| `mat3 add refs` | 11.60 ns | 1.68 us | 3.82 us | - | - | 11.33 us | 144.69x | - | - | 0.15x |
| `mat3 sub refs` | 11.00 ns | 1.72 us | 3.76 us | - | - | 20.69 us | 156.54x | - | - | 0.08x |
| `mat3 mul refs` | 31.51 ns | 1.89 us | 13.00 us | - | - | 60.25 us | 59.99x | - | - | 0.03x |
| `mat3 div refs` | 116.22 ns | 17.46 us | 58.46 us | - | - | 159.18 us | 150.21x | - | - | 0.11x |
| `mat3 neg ref` | 9.53 ns | 498.42 ns | 482.49 ns | - | - | 8.47 us | 52.30x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 43.95 ns | 2.40 us | 4.57 us | - | - | 11.77 us | 54.63x | - | - | 0.20x |
| `mat3 sub_scalar_ref` | 44.21 ns | 2.40 us | 4.42 us | - | - | 21.05 us | 54.28x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 45.82 ns | 1.72 us | 5.85 us | - | - | 11.93 us | 37.43x | - | - | 0.14x |
| `mat3 div_scalar_ref` | 21.43 ns | 4.28 us | 5.59 us | - | - | 21.54 us | 199.81x | - | - | 0.20x |
| `mat4 add refs` | 17.44 ns | 2.63 us | 3.46 us | - | - | 19.04 us | 150.72x | - | - | 0.14x |
| `mat4 sub refs` | 16.38 ns | 2.66 us | 3.55 us | - | - | 35.42 us | 162.61x | - | - | 0.08x |
| `mat4 mul refs` | 69.86 ns | 3.65 us | 18.01 us | - | - | 140.33 us | 52.19x | - | - | 0.03x |
| `mat4 div refs` | 200.21 ns | 27.08 us | 85.77 us | - | - | 522.04 us | 135.24x | - | - | 0.05x |
| `mat4 neg ref` | 12.51 ns | 837.85 ns | 840.29 ns | - | - | 13.81 us | 67.00x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 51.60 ns | 4.14 us | 6.69 us | - | - | 19.96 us | 80.26x | - | - | 0.21x |
| `mat4 sub_scalar_ref` | 39.39 ns | 4.14 us | 6.59 us | - | - | 36.36 us | 104.98x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 55.40 ns | 3.04 us | 7.52 us | - | - | 20.84 us | 54.88x | - | - | 0.15x |
| `mat4 div_scalar_ref` | 29.03 ns | 7.30 us | 7.72 us | - | - | 36.43 us | 251.56x | - | - | 0.20x |
| `mat3 transform_vec refs` | 14.30 ns | 721.70 ns | 4.02 us | - | - | 19.47 us | 50.46x | - | - | 0.04x |
| `mat4 transform_vec refs` | 24.48 ns | 1.11 us | 4.88 us | - | - | 34.60 us | 45.48x | - | - | 0.03x |
| `complex add refs` | 7.63 ns | 326.12 ns | 697.83 ns | - | - | 2.53 us | 42.75x | - | - | 0.13x |
| `complex sub refs` | 7.89 ns | 327.81 ns | 719.58 ns | - | - | 4.70 us | 41.53x | - | - | 0.07x |
| `complex mul refs` | 7.88 ns | 719.84 ns | 3.09 us | - | - | 9.80 us | 91.33x | - | - | 0.07x |
| `complex div refs` | 16.36 ns | 2.62 us | 6.22 us | - | - | 21.34 us | 159.91x | - | - | 0.12x |
| `complex neg ref` | 2.31 ns | 71.42 ns | 76.08 ns | - | - | 2.11 us | 30.94x | - | - | 0.03x |
| `complex div_real_ref` | 7.17 ns | 634.41 ns | 614.23 ns | - | - | 5.08 us | 88.51x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.64 us |
| `astro sin 160` | 13.76 us |
| `astro sin 192` | 13.76 us |
| `astro sin 256` | 15.60 us |
