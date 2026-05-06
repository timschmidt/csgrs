# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
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
| `sin 0.1` | 11.10 ns | 224.07 ns | 222.68 ns | 11.10 us | 781.23 ns | 1.85 us | 20.18x | 0.02x | 0.29x | 0.12x |
| `cos 0.1` | 11.76 ns | 221.46 ns | 217.40 ns | 10.94 us | 504.31 ns | 1.67 us | 18.83x | 0.02x | 0.44x | 0.13x |
| `sin 1.23456789` | 11.97 ns | 661.69 ns | 672.98 ns | 12.88 us | 812.59 ns | 1.84 us | 55.27x | 0.05x | 0.81x | 0.36x |
| `cos 1.23456789` | 12.16 ns | 977.39 ns | 990.00 ns | 11.05 us | 602.00 ns | 1.67 us | 80.40x | 0.09x | 1.62x | 0.58x |
| `sin 1e6` | 12.80 ns | 2.69 us | 2.72 us | 15.92 us | 1.09 us | 2.08 us | 210.32x | 0.17x | 2.46x | 1.30x |
| `cos 1e6` | 12.67 ns | 2.79 us | 2.74 us | 13.98 us | 834.53 ns | 1.84 us | 220.51x | 0.20x | 3.35x | 1.52x |
| `sin 1e30` | 66.76 ns | 2.98 us | 3.00 us | 18.91 us | 2.91 us | 3.57 us | 44.58x | 0.16x | 1.02x | 0.83x |
| `cos 1e30` | 69.38 ns | 2.96 us | 3.04 us | 15.75 us | 963.82 ns | 3.08 us | 42.62x | 0.19x | 3.07x | 0.96x |
| `sin pi_7` | 12.15 ns | 222.92 ns | 524.48 ns | 12.38 us | 757.85 ns | 1.90 us | 18.34x | 0.02x | 0.29x | 0.12x |
| `cos pi_7` | 11.82 ns | 217.73 ns | 995.77 ns | 11.43 us | 547.37 ns | 1.73 us | 18.41x | 0.02x | 0.40x | 0.13x |
| `sin 1000pi_eps` | 11.96 ns | 2.83 us | 4.18 us | 16.09 us | 2.26 us | 2.85 us | 236.46x | 0.18x | 1.25x | 0.99x |
| `cos 1000pi_eps` | 12.81 ns | 2.89 us | 4.18 us | 13.85 us | 576.27 ns | 1.71 us | 225.38x | 0.21x | 5.01x | 1.69x |
| `asin 0.5` | 11.14 ns | 522.21 ns | 526.50 ns | 48.89 us | 3.00 us | 13.45 us | 46.89x | 0.01x | 0.17x | 0.04x |
| `acos 0.5` | 11.46 ns | 1.12 us | 1.09 us | 59.53 us | 2.91 us | 13.31 us | 97.62x | 0.02x | 0.38x | 0.08x |
| `atanh 0.5` | 14.61 ns | 1.44 us | 1.44 us | 34.33 us | 1.71 us | 13.24 us | 98.23x | 0.04x | 0.84x | 0.11x |
| `asin neg_0.999999` | 14.28 ns | 6.78 us | 6.39 us | 13.73 us | 2.55 us | 13.15 us | 474.54x | 0.49x | 2.66x | 0.52x |
| `acos neg_0.999999` | 15.65 ns | 7.55 us | 7.60 us | 18.42 us | 2.70 us | 13.09 us | 482.20x | 0.41x | 2.80x | 0.58x |
| `atanh neg_0.999999` | 14.65 ns | 4.17 us | 3.64 us | 36.17 us | 1.64 us | 12.98 us | 284.40x | 0.12x | 2.53x | 0.32x |
| `asin 0.999999` | 14.05 ns | 6.22 us | 6.03 us | 13.56 us | 2.53 us | 13.04 us | 442.93x | 0.46x | 2.46x | 0.48x |
| `acos 0.999999` | 14.44 ns | 5.43 us | 4.92 us | 18.36 us | 2.74 us | 13.12 us | 375.84x | 0.30x | 1.98x | 0.41x |
| `atanh 0.999999` | 14.45 ns | 4.11 us | 3.56 us | 31.48 us | 1.65 us | 12.79 us | 284.04x | 0.13x | 2.48x | 0.32x |
| `asin 1e-12` | 9.48 ns | 7.57 us | 5.26 us | 7.99 us | 1.47 us | 15.37 us | 798.14x | 0.95x | 5.15x | 0.49x |
| `acos 1e-12` | 10.03 ns | 12.58 us | 11.32 us | 9.79 us | 1.45 us | 15.29 us | 1253.50x | 1.28x | 8.65x | 0.82x |
| `atanh 1e-12` | 9.76 ns | 5.78 us | 3.27 us | 36.53 us | 170.97 ns | 20.16 us | 591.99x | 0.16x | 33.79x | 0.29x |
| `atan 0.5` | 14.98 ns | 429.32 ns | 434.76 ns | 35.83 us | 2.73 us | 17.86 us | 28.65x | 0.01x | 0.16x | 0.02x |
| `asinh 0.5` | 27.43 ns | 1.65 us | 1.63 us | 39.64 us | 1.63 us | 7.51 us | 60.09x | 0.04x | 1.01x | 0.22x |
| `atan neg_1e-12` | 14.33 ns | 474.80 ns | 739.46 ns | 1.61 us | 1.14 us | 15.51 us | 33.14x | 0.30x | 0.42x | 0.03x |
| `asinh neg_1e-12` | 16.03 ns | 5.47 us | 4.46 us | 43.10 us | 8.68 us | 11.99 us | 341.50x | 0.13x | 0.63x | 0.46x |
| `atan 1e6` | 15.30 ns | 647.73 ns | 655.28 ns | 2.85 us | 1.45 us | 18.35 us | 42.33x | 0.23x | 0.45x | 0.04x |
| `asinh 1e6` | 26.86 ns | 3.35 us | 3.32 us | 36.83 us | 1.69 us | 7.33 us | 124.86x | 0.09x | 1.99x | 0.46x |
| `atan neg_1e6` | 15.54 ns | 747.51 ns | 727.13 ns | 2.85 us | 1.42 us | 18.34 us | 48.10x | 0.26x | 0.53x | 0.04x |
| `asinh neg_1e6` | 27.27 ns | 3.33 us | 3.29 us | 37.03 us | 1.69 us | 7.17 us | 122.11x | 0.09x | 1.98x | 0.46x |
| `acosh 9` | 12.68 ns | 2.97 us | 2.87 us | 42.94 us | 1.67 us | 9.98 us | 234.41x | 0.07x | 1.78x | 0.30x |
| `acosh 1_plus_1e-12` | 12.54 ns | 3.75 us | 5.10 us | 42.63 us | 8.41 us | 11.50 us | 298.95x | 0.09x | 0.45x | 0.33x |
| `acosh 1e6` | 12.59 ns | 3.74 us | 3.65 us | 37.63 us | 1.66 us | 10.04 us | 297.15x | 0.10x | 2.26x | 0.37x |
| `acosh e` | 12.58 ns | 4.02 us | 1.22 us | 41.69 us | 1.67 us | 9.86 us | 319.42x | 0.10x | 2.40x | 0.41x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 55.64 ns | 53.68 ns | 24.97 ns | 15.65 ns | 0.97 ns | 117.36x | 2.23x | 3.55x | 57.59x |
| `one` | 0.47 ns | 63.74 ns | 59.41 ns | 43.47 ns | 30.92 ns | 32.90 ns | 134.90x | 1.47x | 2.06x | 1.94x |
| `e` | 0.48 ns | 73.93 ns | 70.20 ns | 56.12 ns | 1.07 us | 227.91 ns | 154.28x | 1.32x | 0.07x | 0.32x |
| `pi` | 0.48 ns | 57.26 ns | 52.96 ns | 45.66 ns | 50.10 ns | 233.71 ns | 119.20x | 1.25x | 1.14x | 0.25x |
| `tau` | 0.48 ns | 201.91 ns | 193.23 ns | 120.68 ns | 104.75 ns | 1.89 us | 423.63x | 1.67x | 1.93x | 0.11x |
| `add` | 5.35 ns | 301.77 ns | 412.43 ns | 53.53 ns | 42.97 ns | 1.34 us | 56.44x | 5.64x | 7.02x | 0.23x |
| `sub` | 5.33 ns | 301.41 ns | 465.26 ns | 59.08 ns | 45.07 ns | 2.51 us | 56.51x | 5.10x | 6.69x | 0.12x |
| `neg` | 5.17 ns | 63.71 ns | 61.68 ns | 24.05 ns | 20.34 ns | 1.10 us | 12.32x | 2.65x | 3.13x | 0.06x |
| `mul` | 5.58 ns | 217.36 ns | 481.29 ns | 59.97 ns | 45.16 ns | 1.55 us | 38.97x | 3.62x | 4.81x | 0.14x |
| `div` | 9.50 ns | 396.33 ns | 464.58 ns | 150.92 ns | 62.37 ns | 2.63 us | 41.74x | 2.63x | 6.35x | 0.15x |
| `reciprocal` | 8.79 ns | 115.95 ns | 111.45 ns | 168.05 ns | 59.48 ns | 1.60 us | 13.19x | 0.69x | 1.95x | 0.07x |
| `reciprocal checked` | 9.29 ns | 119.45 ns | 112.96 ns | 163.87 ns | 59.52 ns | 1.59 us | 12.86x | 0.73x | 2.01x | 0.07x |
| `reciprocal checked abort` | 25.10 ns | 136.05 ns | 132.35 ns | 164.28 ns | 59.45 ns | 1.62 us | 5.42x | 0.83x | 2.29x | 0.08x |
| `pow` | 31.13 ns | 10.57 us | 10.92 us | 54.44 us | 2.90 us | 2.53 us | 339.65x | 0.19x | 3.64x | 4.18x |
| `powi` | 6.20 ns | 578.62 ns | 2.88 us | 288.24 ns | 85.74 ns | 1.63 us | 93.28x | 2.01x | 6.75x | 0.35x |
| `exp` | 10.30 ns | 1.15 us | 1.86 us | 14.42 us | 934.06 ns | 1.90 us | 111.71x | 0.08x | 1.23x | 0.61x |
| `ln` | 10.84 ns | 1.54 us | 1.54 us | 30.75 us | 1.34 us | 1.87 us | 142.30x | 0.05x | 1.15x | 0.82x |
| `log10` | 13.67 ns | 2.74 us | 2.40 us | 36.05 us | 2.84 us | 6.74 us | 200.81x | 0.08x | 0.97x | 0.41x |
| `log10 abort` | 17.65 ns | 2.74 us | 2.40 us | 36.11 us | 2.83 us | 6.79 us | 155.49x | 0.08x | 0.97x | 0.40x |
| `sqrt` | 20.60 ns | 1.60 us | 1.72 us | 5.27 us | 97.34 ns | 1.49 us | 77.83x | 0.30x | 16.47x | 1.08x |
| `sin` | 15.29 ns | 1.52 us | 1.73 us | 14.20 us | 1.26 us | 2.29 us | 99.14x | 0.11x | 1.21x | 0.66x |
| `cos` | 18.74 ns | 1.54 us | 1.74 us | 12.34 us | 644.78 ns | 1.79 us | 82.12x | 0.12x | 2.39x | 0.86x |
| `tan` | 24.98 ns | 1.35 us | 1.55 us | 31.19 us | 1.59 us | 6.94 us | 54.16x | 0.04x | 0.85x | 0.19x |
| `sinh` | 19.53 ns | 3.22 us | 3.44 us | 3.36 us | 1.15 us | 10.97 us | 165.05x | 0.96x | 2.81x | 0.29x |
| `cosh` | 19.63 ns | 3.10 us | 3.37 us | 8.20 us | 1.07 us | 9.67 us | 157.94x | 0.38x | 2.90x | 0.32x |
| `tanh` | 24.02 ns | 4.71 us | 5.02 us | 3.37 us | 1.22 us | 23.37 us | 196.12x | 1.40x | 3.88x | 0.20x |
| `asin` | 13.49 ns | 5.39 us | 6.54 us | 21.80 us | 2.43 us | 14.03 us | 399.86x | 0.25x | 2.21x | 0.38x |
| `asin abort` | 17.67 ns | 5.40 us | 6.33 us | 22.10 us | 2.44 us | 13.98 us | 305.77x | 0.24x | 2.21x | 0.39x |
| `acos` | 14.53 ns | 6.93 us | 8.57 us | 26.98 us | 2.56 us | 13.91 us | 477.07x | 0.26x | 2.71x | 0.50x |
| `acos abort` | 18.04 ns | 6.81 us | 8.57 us | 26.99 us | 2.58 us | 13.90 us | 377.24x | 0.25x | 2.64x | 0.49x |
| `atan` | 14.97 ns | 625.19 ns | 1.27 us | 18.76 us | 2.27 us | 19.13 us | 41.77x | 0.03x | 0.27x | 0.03x |
| `atan abort` | 21.78 ns | 613.94 ns | 1.28 us | 18.42 us | 2.29 us | 19.13 us | 28.19x | 0.03x | 0.27x | 0.03x |
| `asinh` | 49.09 ns | 3.62 us | 5.50 us | 39.71 us | 1.69 us | 7.77 us | 73.72x | 0.09x | 2.14x | 0.47x |
| `asinh abort` | 31.54 ns | 3.64 us | 5.49 us | 39.40 us | 1.67 us | 7.65 us | 115.34x | 0.09x | 2.18x | 0.48x |
| `acosh` | 12.77 ns | 3.69 us | 5.38 us | 40.28 us | 3.43 us | 10.58 us | 289.13x | 0.09x | 1.08x | 0.35x |
| `acosh abort` | 15.85 ns | 3.68 us | 5.32 us | 40.74 us | 3.42 us | 10.54 us | 232.47x | 0.09x | 1.08x | 0.35x |
| `atanh` | 14.12 ns | 4.05 us | 4.14 us | 35.21 us | 1.32 us | 15.06 us | 286.61x | 0.11x | 3.06x | 0.27x |
| `atanh abort` | 17.68 ns | 4.12 us | 4.02 us | 35.24 us | 1.32 us | 15.11 us | 233.28x | 0.12x | 3.11x | 0.27x |
| `zero status` | 1.24 ns | 1.83 ns | 1.79 ns | 0.99 ns | 6.88 ns | 8.29 ns | 1.48x | 1.84x | 0.27x | 0.22x |
| `zero status abort` | 1.45 ns | 3.03 ns | 3.01 ns | 1.07 ns | 6.81 ns | 8.09 ns | 2.10x | 2.82x | 0.45x | 0.38x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 115.44 ns | 116.58 ns | 48.26 ns | - | 1.92 ns | 122.79x | 2.39x | - | 60.09x |
| `one` | 5.15 ns | 116.87 ns | 117.04 ns | 65.71 ns | - | 31.41 ns | 22.71x | 1.78x | - | 3.72x |
| `i` | 0.96 ns | 123.60 ns | 123.06 ns | 64.88 ns | - | 33.68 ns | 129.31x | 1.91x | - | 3.67x |
| `free i` | 0.94 ns | 124.01 ns | 123.33 ns | 64.93 ns | - | 33.44 ns | 131.59x | 1.91x | - | 3.71x |
| `conjugate` | 2.14 ns | 134.71 ns | 132.50 ns | 42.40 ns | - | 1.12 us | 62.81x | 3.18x | - | 0.12x |
| `norm squared` | 5.90 ns | 403.71 ns | 1.53 us | 153.66 ns | - | 4.44 us | 68.47x | 2.63x | - | 0.09x |
| `reciprocal` | 18.97 ns | 1.93 us | 3.07 us | 464.41 ns | - | 11.01 us | 101.80x | 4.16x | - | 0.18x |
| `reciprocal checked` | 14.42 ns | 1.95 us | 3.04 us | 459.94 ns | - | 11.23 us | 135.07x | 4.24x | - | 0.17x |
| `powi` | 18.36 ns | 2.74 us | 14.00 us | 1.50 us | - | 45.00 us | 149.27x | 1.83x | - | 0.06x |
| `powi checked` | 18.35 ns | 2.75 us | 14.41 us | 1.54 us | - | 45.04 us | 149.91x | 1.79x | - | 0.06x |
| `div checked` | 19.67 ns | 3.09 us | 6.76 us | 822.51 ns | - | 22.34 us | 156.95x | 3.75x | - | 0.14x |
| `div real checked` | 9.51 ns | 804.36 ns | 777.45 ns | 263.86 ns | - | 5.26 us | 84.62x | 3.05x | - | 0.15x |
| `from scalar` | 1.44 ns | 130.61 ns | 128.66 ns | 47.81 ns | - | 10.74 ns | 90.63x | 2.73x | - | 12.16x |
| `add` | 6.12 ns | 620.90 ns | 1.02 us | 106.75 ns | - | 2.66 us | 101.50x | 5.82x | - | 0.23x |
| `sub` | 6.17 ns | 634.61 ns | 1.02 us | 125.15 ns | - | 4.87 us | 102.85x | 5.07x | - | 0.13x |
| `neg` | 2.65 ns | 135.70 ns | 132.32 ns | 44.26 ns | - | 2.27 us | 51.26x | 3.07x | - | 0.06x |
| `mul` | 7.67 ns | 1.13 us | 3.59 us | 307.27 ns | - | 10.07 us | 147.56x | 3.68x | - | 0.11x |
| `div` | 18.21 ns | 3.08 us | 6.82 us | 805.50 ns | - | 21.99 us | 169.03x | 3.82x | - | 0.14x |
| `div real` | 10.23 ns | 781.55 ns | 758.61 ns | 269.26 ns | - | 5.33 us | 76.43x | 2.90x | - | 0.15x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.83 ns | 833.44 ns | 6.17 us | 319.88 ns | 255.31 ns | 7.45 us | 121.98x | 2.61x | 3.26x | 0.11x |
| `vec3 magnitude` | 9.63 ns | 4.75 us | 8.85 us | 5.41 us | 353.62 ns | 8.91 us | 492.93x | 0.88x | 13.43x | 0.53x |
| `vec3 normalize` | 25.78 ns | 8.96 us | 11.20 us | 6.15 us | 591.18 ns | 17.14 us | 347.52x | 1.46x | 15.15x | 0.52x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.08 ns | 374.22 ns | 1.54 us | 71.42 ns | 56.41 ns | 725.77 ns | 121.35x | 5.24x | 6.63x | 0.52x |
| `vec3 zero` | 1.44 ns | 241.07 ns | 236.93 ns | 59.09 ns | 30.86 ns | 2.82 ns | 167.92x | 4.08x | 7.81x | 85.36x |
| `vec3 dot abort` | 27.93 ns | 1.28 us | 4.60 us | 257.09 ns | 204.27 ns | 7.33 us | 45.99x | 5.00x | 6.29x | 0.18x |
| `vec3 magnitude abort` | 39.30 ns | 5.20 us | 6.85 us | 5.40 us | 320.77 ns | 9.06 us | 132.24x | 0.96x | 16.20x | 0.57x |
| `vec3 normalize checked` | 26.28 ns | 8.98 us | 8.96 us | 5.82 us | 541.10 ns | 17.12 us | 341.69x | 1.54x | 16.60x | 0.52x |
| `vec3 normalize checked abort` | 55.49 ns | 9.51 us | 9.43 us | 5.84 us | 538.98 ns | 17.09 us | 171.40x | 1.63x | 17.64x | 0.56x |
| `vec3 div scalar checked` | 10.52 ns | 1.47 us | 1.70 us | 389.19 ns | - | - | 139.95x | 3.78x | - | - |
| `vec3 div scalar checked abort` | 18.26 ns | 1.52 us | 1.75 us | 396.42 ns | - | - | 82.98x | 3.82x | - | - |
| `vec3 add` | 6.84 ns | 1.22 us | 1.98 us | 152.18 ns | 125.88 ns | 4.16 us | 178.00x | 8.00x | 9.67x | 0.29x |
| `vec3 add scalar` | 6.48 ns | 982.08 ns | 1.67 us | 156.51 ns | 133.53 ns | 3.97 us | 151.59x | 6.27x | 7.35x | 0.25x |
| `vec3 sub` | 6.78 ns | 1.21 us | 2.04 us | 166.49 ns | 136.60 ns | 7.45 us | 177.78x | 7.24x | 8.83x | 0.16x |
| `vec3 sub scalar` | 6.55 ns | 980.90 ns | 1.48 us | 151.36 ns | 124.94 ns | 7.23 us | 149.74x | 6.48x | 7.85x | 0.14x |
| `vec3 neg` | 3.84 ns | 231.77 ns | 230.42 ns | 57.12 ns | 46.40 ns | 3.19 us | 60.36x | 4.06x | 5.00x | 0.07x |
| `vec3 mul scalar` | 7.08 ns | 689.92 ns | 2.17 us | 166.77 ns | 119.27 ns | 4.45 us | 97.42x | 4.14x | 5.78x | 0.16x |
| `vec3 div scalar` | 18.85 ns | 1.46 us | 1.74 us | 402.41 ns | 183.92 ns | 7.79 us | 77.37x | 3.62x | 7.93x | 0.19x |
| `vec4 dot` | 7.36 ns | 992.71 ns | 3.47 us | 445.92 ns | 313.40 ns | 9.85 us | 134.94x | 2.23x | 3.17x | 0.10x |
| `vec4 magnitude` | 13.15 ns | 4.61 us | 5.59 us | 5.49 us | 407.60 ns | 11.48 us | 350.63x | 0.84x | 11.31x | 0.40x |
| `vec4 normalize` | 36.55 ns | 9.12 us | 7.61 us | 6.10 us | 702.67 ns | 22.68 us | 249.55x | 1.50x | 12.98x | 0.40x |
| `vec4 add` | 8.09 ns | 1.49 us | 2.14 us | 209.42 ns | 172.21 ns | 5.47 us | 183.98x | 7.11x | 8.65x | 0.27x |
| `vec4 add scalar` | 7.05 ns | 1.24 us | 1.96 us | 220.93 ns | 176.51 ns | 5.28 us | 175.87x | 5.61x | 7.02x | 0.23x |
| `vec4 sub` | 5.26 ns | 1.50 us | 2.04 us | 216.58 ns | 176.98 ns | 9.81 us | 285.02x | 6.92x | 8.47x | 0.15x |
| `vec4 sub scalar` | 4.51 ns | 1.23 us | 1.74 us | 209.96 ns | 168.14 ns | 9.64 us | 273.77x | 5.88x | 7.34x | 0.13x |
| `vec4 neg` | 4.98 ns | 298.17 ns | 298.88 ns | 78.11 ns | 65.33 ns | 4.15 us | 59.91x | 3.82x | 4.56x | 0.07x |
| `vec4 mul scalar` | 7.59 ns | 866.78 ns | 2.27 us | 228.61 ns | 159.91 ns | 5.69 us | 114.19x | 3.79x | 5.42x | 0.15x |
| `vec4 div scalar` | 14.34 ns | 1.95 us | 1.79 us | 530.59 ns | 242.79 ns | 10.17 us | 135.72x | 3.67x | 8.02x | 0.19x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.48 ns | 1.86 us | 3.57 us | 1.00 us | 867.76 ns | 22.97 us | 149.29x | 1.86x | 2.15x | 0.08x |
| `mat3 inverse` | 80.71 ns | 18.71 us | 12.23 us | 3.22 us | 2.62 us | 84.00 us | 231.76x | 5.81x | 7.15x | 0.22x |
| `mat3 mul mat3` | 74.12 ns | 8.22 us | 12.81 us | 2.89 us | 2.42 us | 62.43 us | 110.92x | 2.85x | 3.39x | 0.13x |
| `mat3 transform vec3` | 16.34 ns | 3.43 us | 11.12 us | 1.07 us | 896.98 ns | 21.06 us | 209.69x | 3.20x | 3.82x | 0.16x |
| `mat4 determinant` | 45.96 ns | 5.44 us | 4.32 us | 4.85 us | 4.24 us | 99.83 us | 118.43x | 1.12x | 1.28x | 0.05x |
| `mat4 inverse` | 153.51 ns | 35.53 us | 17.12 us | 11.46 us | 9.40 us | 355.32 us | 231.47x | 3.10x | 3.78x | 0.10x |
| `mat4 mul mat4` | 117.96 ns | 17.15 us | 14.35 us | 6.68 us | 5.49 us | 145.96 us | 145.40x | 2.57x | 3.12x | 0.12x |
| `mat4 transform vec4` | 25.57 ns | 5.61 us | 5.11 us | 1.93 us | 1.73 us | 36.34 us | 219.58x | 2.91x | 3.25x | 0.15x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.52 ns | 1.52 us | 4.46 us | 207.73 ns | 205.14 ns | 2.07 us | 43.97x | 7.31x | 7.40x | 0.73x |
| `mat3 zero` | 16.90 ns | 870.52 ns | 874.84 ns | 259.83 ns | 167.28 ns | 11.40 ns | 51.52x | 3.35x | 5.20x | 76.36x |
| `mat3 identity` | 10.03 ns | 946.11 ns | 943.59 ns | 329.04 ns | 208.65 ns | 152.17 ns | 94.37x | 2.88x | 4.53x | 6.22x |
| `mat3 transpose` | 9.13 ns | 892.61 ns | 887.91 ns | 229.40 ns | 181.27 ns | 130.11 ns | 97.74x | 3.89x | 4.92x | 6.86x |
| `mat3 reciprocal` | 78.53 ns | 18.03 us | 35.92 us | 2.97 us | 2.23 us | 84.44 us | 229.62x | 6.08x | 8.09x | 0.21x |
| `mat3 reciprocal checked` | 78.74 ns | 18.29 us | 36.40 us | 2.95 us | 2.24 us | 83.23 us | 232.25x | 6.21x | 8.17x | 0.22x |
| `mat3 inverse checked` | 79.11 ns | 18.08 us | 36.07 us | 2.96 us | 2.22 us | 83.25 us | 228.49x | 6.11x | 8.13x | 0.22x |
| `mat3 inverse checked abort` | 115.09 ns | 17.99 us | 35.96 us | 2.96 us | 2.23 us | 83.74 us | 156.32x | 6.07x | 8.07x | 0.21x |
| `mat3 powi` | 144.24 ns | 16.48 us | 88.67 us | 6.77 us | 6.23 us | 152.02 us | 114.25x | 2.43x | 2.65x | 0.11x |
| `mat3 powi checked` | 144.12 ns | 16.40 us | 88.68 us | 6.80 us | 6.16 us | 151.69 us | 113.82x | 2.41x | 2.66x | 0.11x |
| `mat3 powi checked abort` | 144.61 ns | 17.03 us | 88.67 us | 6.78 us | 6.18 us | 151.29 us | 117.75x | 2.51x | 2.75x | 0.11x |
| `mat3 div scalar checked` | 24.66 ns | 4.75 us | 6.09 us | 1.45 us | 816.41 ns | 22.55 us | 192.72x | 3.29x | 5.82x | 0.21x |
| `mat3 div scalar checked abort` | 29.29 ns | 4.84 us | 6.21 us | 1.45 us | 816.10 ns | 22.51 us | 165.15x | 3.34x | 5.93x | 0.21x |
| `mat3 div matrix checked` | 147.79 ns | 65.89 us | 65.50 us | 5.52 us | 4.36 us | 160.38 us | 445.84x | 11.93x | 15.12x | 0.41x |
| `mat3 div matrix checked abort` | 160.62 ns | 65.56 us | 65.60 us | 5.49 us | 4.37 us | 161.01 us | 408.18x | 11.93x | 14.99x | 0.41x |
| `mat3 add` | 14.77 ns | 3.93 us | 6.16 us | 520.15 ns | 484.75 ns | 11.95 us | 265.70x | 7.55x | 8.10x | 0.33x |
| `mat3 add scalar` | 12.22 ns | 2.91 us | 5.06 us | 800.58 ns | 704.10 ns | 12.30 us | 238.24x | 3.64x | 4.14x | 0.24x |
| `mat3 sub` | 13.70 ns | 3.90 us | 5.92 us | 547.24 ns | 518.35 ns | 21.44 us | 284.41x | 7.12x | 7.52x | 0.18x |
| `mat3 sub scalar` | 11.35 ns | 2.94 us | 4.92 us | 808.59 ns | 702.27 ns | 21.92 us | 259.12x | 3.64x | 4.19x | 0.13x |
| `mat3 neg` | 10.74 ns | 934.56 ns | 933.89 ns | 485.69 ns | 461.64 ns | 8.73 us | 86.98x | 1.92x | 2.02x | 0.11x |
| `mat3 mul scalar` | 13.86 ns | 2.17 us | 6.10 us | 774.55 ns | 646.13 ns | 12.23 us | 156.39x | 2.80x | 3.35x | 0.18x |
| `mat3 div scalar` | 24.84 ns | 4.70 us | 6.02 us | 1.45 us | 814.11 ns | 22.49 us | 189.14x | 3.25x | 5.77x | 0.21x |
| `mat3 div matrix` | 148.52 ns | 66.09 us | 65.97 us | 5.50 us | 4.35 us | 160.58 us | 444.97x | 12.01x | 15.18x | 0.41x |
| `mat3 bitxor` | 144.35 ns | 16.39 us | 88.55 us | 6.75 us | 6.20 us | 150.81 us | 113.56x | 2.43x | 2.64x | 0.11x |
| `mat4 zero` | 13.23 ns | 1.36 us | 1.38 us | 514.16 ns | 348.52 ns | 14.01 ns | 102.85x | 2.65x | 3.90x | 97.09x |
| `mat4 identity` | 10.22 ns | 1.53 us | 1.53 us | 545.06 ns | 412.02 ns | 229.41 ns | 149.99x | 2.81x | 3.72x | 6.68x |
| `mat4 transpose` | 9.82 ns | 1.59 us | 1.58 us | 442.16 ns | 369.37 ns | 174.86 ns | 161.55x | 3.59x | 4.29x | 9.07x |
| `mat4 reciprocal` | 159.55 ns | 35.26 us | 64.66 us | 10.85 us | 9.12 us | 347.45 us | 221.00x | 3.25x | 3.87x | 0.10x |
| `mat4 reciprocal checked` | 162.19 ns | 35.21 us | 65.41 us | 10.70 us | 8.99 us | 347.24 us | 217.07x | 3.29x | 3.92x | 0.10x |
| `mat4 powi` | 240.76 ns | 34.12 us | 112.60 us | 15.67 us | 13.88 us | 355.30 us | 141.73x | 2.18x | 2.46x | 0.10x |
| `mat4 powi checked` | 241.32 ns | 34.42 us | 114.16 us | 15.75 us | 14.04 us | 354.24 us | 142.62x | 2.19x | 2.45x | 0.10x |
| `mat4 add` | 50.87 ns | 6.64 us | 7.47 us | 882.39 ns | 857.61 ns | 19.96 us | 130.61x | 7.53x | 7.75x | 0.33x |
| `mat4 add scalar` | 20.66 ns | 4.92 us | 7.45 us | 1.44 us | 1.20 us | 21.23 us | 238.06x | 3.41x | 4.10x | 0.23x |
| `mat4 sub` | 37.23 ns | 6.61 us | 7.52 us | 938.35 ns | 901.06 ns | 36.36 us | 177.59x | 7.05x | 7.34x | 0.18x |
| `mat4 sub scalar` | 15.01 ns | 4.96 us | 7.34 us | 1.45 us | 1.18 us | 38.02 us | 330.46x | 3.43x | 4.19x | 0.13x |
| `mat4 neg` | 13.60 ns | 1.62 us | 1.63 us | 936.61 ns | 783.57 ns | 14.58 us | 119.29x | 1.73x | 2.07x | 0.11x |
| `mat4 mul scalar` | 24.02 ns | 3.65 us | 8.08 us | 1.44 us | 1.13 us | 20.57 us | 152.15x | 2.53x | 3.22x | 0.18x |
| `mat4 div scalar` | 32.25 ns | 8.52 us | 8.45 us | 2.70 us | 1.41 us | 38.56 us | 264.08x | 3.15x | 6.03x | 0.22x |
| `mat4 div matrix` | 217.16 ns | 133.68 us | 107.38 us | 16.88 us | 14.36 us | 538.64 us | 615.59x | 7.92x | 9.31x | 0.25x |
| `mat4 bitxor` | 242.56 ns | 34.07 us | 112.96 us | 15.86 us | 13.81 us | 353.96 us | 140.44x | 2.15x | 2.47x | 0.10x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.66 ns | 216.20 ns | 316.87 ns | - | - | 1.32 us | 59.11x | - | - | 0.16x |
| `scalar add ref_owned` | 12.12 ns | 213.80 ns | 321.65 ns | - | - | 1.33 us | 17.65x | - | - | 0.16x |
| `scalar add refs` | 5.32 ns | 200.03 ns | 302.14 ns | - | - | 1.32 us | 37.59x | - | - | 0.15x |
| `scalar add owned_ref_with_clone` | 8.97 ns | 551.57 ns | 656.63 ns | - | - | - | 61.49x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.47 ns | 548.17 ns | 671.06 ns | - | - | - | 100.24x | - | - | - |
| `scalar sub owned_ref` | 3.92 ns | 216.56 ns | 378.27 ns | - | - | 2.50 us | 55.21x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.27 ns | 214.22 ns | 375.23 ns | - | - | 2.50 us | 17.46x | - | - | 0.09x |
| `scalar sub refs` | 5.57 ns | 203.35 ns | 357.96 ns | - | - | 2.48 us | 36.50x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 8.96 ns | 554.09 ns | 723.07 ns | - | - | - | 61.84x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.68 ns | 573.25 ns | 730.75 ns | - | - | - | 100.88x | - | - | - |
| `scalar mul owned_ref` | 4.74 ns | 123.68 ns | 392.99 ns | - | - | 1.54 us | 26.10x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.48 ns | 122.62 ns | 398.29 ns | - | - | 1.55 us | 9.10x | - | - | 0.08x |
| `scalar mul refs` | 6.23 ns | 111.64 ns | 376.50 ns | - | - | 1.54 us | 17.93x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 9.67 ns | 444.64 ns | 732.36 ns | - | - | - | 45.96x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.26 ns | 451.51 ns | 739.90 ns | - | - | - | 72.16x | - | - | - |
| `scalar div owned_ref` | 5.81 ns | 289.03 ns | 336.14 ns | - | - | 2.59 us | 49.78x | - | - | 0.11x |
| `scalar div ref_owned` | 17.18 ns | 295.41 ns | 343.83 ns | - | - | 2.57 us | 17.20x | - | - | 0.11x |
| `scalar div refs` | 6.74 ns | 275.25 ns | 324.48 ns | - | - | 2.57 us | 40.81x | - | - | 0.11x |
| `scalar div owned_ref_with_clone` | 15.49 ns | 612.41 ns | 666.74 ns | - | - | - | 39.53x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.84 ns | 620.97 ns | 676.39 ns | - | - | - | 39.21x | - | - | - |
| `vec3 add refs` | 6.26 ns | 997.09 ns | 1.81 us | - | - | 4.09 us | 159.29x | - | - | 0.24x |
| `vec3 sub refs` | 6.17 ns | 994.95 ns | 1.88 us | - | - | 7.41 us | 161.32x | - | - | 0.13x |
| `vec3 neg ref` | 3.28 ns | 540.11 ns | 533.99 ns | - | - | 3.19 us | 164.52x | - | - | 0.17x |
| `vec3 add_scalar_ref` | 6.44 ns | 1.23 us | 1.94 us | - | - | 3.91 us | 190.31x | - | - | 0.31x |
| `vec3 sub_scalar_ref` | 6.51 ns | 1.23 us | 1.74 us | - | - | 7.22 us | 188.95x | - | - | 0.17x |
| `vec3 mul_scalar_ref` | 6.91 ns | 945.30 ns | 2.50 us | - | - | 4.41 us | 136.86x | - | - | 0.21x |
| `vec3 div_scalar_ref` | 8.04 ns | 1.76 us | 2.06 us | - | - | 7.68 us | 218.71x | - | - | 0.23x |
| `vec4 add refs` | 6.44 ns | 1.19 us | 1.82 us | - | - | 5.44 us | 185.28x | - | - | 0.22x |
| `vec4 sub refs` | 3.06 ns | 1.16 us | 1.71 us | - | - | 9.81 us | 379.52x | - | - | 0.12x |
| `vec4 neg ref` | 4.21 ns | 622.23 ns | 624.54 ns | - | - | 4.14 us | 147.69x | - | - | 0.15x |
| `vec4 add_scalar_ref` | 6.90 ns | 1.49 us | 2.25 us | - | - | 5.19 us | 215.64x | - | - | 0.29x |
| `vec4 sub_scalar_ref` | 4.23 ns | 1.49 us | 2.02 us | - | - | 9.70 us | 353.41x | - | - | 0.15x |
| `vec4 mul_scalar_ref` | 7.33 ns | 1.10 us | 2.54 us | - | - | 5.72 us | 150.27x | - | - | 0.19x |
| `vec4 div_scalar_ref` | 12.50 ns | 2.25 us | 2.08 us | - | - | 10.05 us | 180.24x | - | - | 0.22x |
| `mat3 add refs` | 11.24 ns | 2.34 us | 4.55 us | - | - | 11.93 us | 207.96x | - | - | 0.20x |
| `mat3 sub refs` | 10.81 ns | 2.35 us | 4.41 us | - | - | 21.69 us | 217.08x | - | - | 0.11x |
| `mat3 mul refs` | 56.97 ns | 6.93 us | 29.55 us | - | - | 62.23 us | 121.56x | - | - | 0.11x |
| `mat3 div refs` | 150.90 ns | 67.38 us | 67.26 us | - | - | 160.56 us | 446.51x | - | - | 0.42x |
| `mat3 neg ref` | 9.84 ns | 1.05 us | 1.01 us | - | - | 8.80 us | 106.75x | - | - | 0.12x |
| `mat3 add_scalar_ref` | 44.04 ns | 3.26 us | 5.52 us | - | - | 12.30 us | 73.98x | - | - | 0.26x |
| `mat3 sub_scalar_ref` | 44.51 ns | 3.30 us | 5.32 us | - | - | 21.76 us | 74.12x | - | - | 0.15x |
| `mat3 mul_scalar_ref` | 47.43 ns | 2.52 us | 6.58 us | - | - | 12.35 us | 53.06x | - | - | 0.20x |
| `mat3 div_scalar_ref` | 22.03 ns | 5.20 us | 6.45 us | - | - | 22.53 us | 236.19x | - | - | 0.23x |
| `mat4 add refs` | 16.62 ns | 3.47 us | 4.34 us | - | - | 20.16 us | 208.68x | - | - | 0.17x |
| `mat4 sub refs` | 16.37 ns | 3.45 us | 4.48 us | - | - | 37.90 us | 210.58x | - | - | 0.09x |
| `mat4 mul refs` | 100.97 ns | 14.36 us | 30.68 us | - | - | 144.83 us | 142.24x | - | - | 0.10x |
| `mat4 div refs` | 217.08 ns | 134.67 us | 108.01 us | - | - | 542.71 us | 620.40x | - | - | 0.25x |
| `mat4 neg ref` | 12.00 ns | 1.59 us | 1.61 us | - | - | 14.65 us | 132.14x | - | - | 0.11x |
| `mat4 add_scalar_ref` | 48.82 ns | 5.43 us | 8.13 us | - | - | 21.01 us | 111.18x | - | - | 0.26x |
| `mat4 sub_scalar_ref` | 37.04 ns | 5.52 us | 8.07 us | - | - | 38.06 us | 149.00x | - | - | 0.15x |
| `mat4 mul_scalar_ref` | 52.49 ns | 4.12 us | 8.74 us | - | - | 20.95 us | 78.57x | - | - | 0.20x |
| `mat4 div_scalar_ref` | 27.32 ns | 8.75 us | 8.96 us | - | - | 38.66 us | 320.21x | - | - | 0.23x |
| `mat3 transform_vec refs` | 14.42 ns | 2.68 us | 11.83 us | - | - | 20.33 us | 185.84x | - | - | 0.13x |
| `mat4 transform_vec refs` | 22.97 ns | 4.34 us | 12.15 us | - | - | 35.50 us | 189.08x | - | - | 0.12x |
| `complex add refs` | 7.72 ns | 691.01 ns | 1.08 us | - | - | 2.64 us | 89.56x | - | - | 0.26x |
| `complex sub refs` | 8.01 ns | 689.14 ns | 1.11 us | - | - | 4.83 us | 86.04x | - | - | 0.14x |
| `complex mul refs` | 8.00 ns | 1.20 us | 3.58 us | - | - | 10.04 us | 149.72x | - | - | 0.12x |
| `complex div refs` | 17.31 ns | 3.16 us | 6.74 us | - | - | 21.90 us | 182.49x | - | - | 0.14x |
| `complex neg ref` | 2.35 ns | 400.57 ns | 403.84 ns | - | - | 2.19 us | 170.20x | - | - | 0.18x |
| `complex div_real_ref` | 7.24 ns | 1.04 us | 1.01 us | - | - | 5.22 us | 144.24x | - | - | 0.20x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.42 us |
| `astro sin 160` | 14.08 us |
| `astro sin 192` | 13.90 us |
| `astro sin 256` | 16.03 us |
