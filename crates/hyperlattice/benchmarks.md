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
| `sin 0.1` | 11.10 ns | 177.38 ns | 178.18 ns | 11.26 us | 765.13 ns | 1.86 us | 15.98x | 0.02x | 0.23x | 0.10x |
| `cos 0.1` | 11.92 ns | 180.52 ns | 178.84 ns | 11.06 us | 521.31 ns | 1.71 us | 15.15x | 0.02x | 0.35x | 0.11x |
| `sin 1.23456789` | 12.08 ns | 1.94 us | 1.95 us | 13.06 us | 829.79 ns | 1.82 us | 160.72x | 0.15x | 2.34x | 1.07x |
| `cos 1.23456789` | 12.16 ns | 2.10 us | 2.27 us | 11.04 us | 593.16 ns | 1.67 us | 172.35x | 0.19x | 3.53x | 1.25x |
| `sin 1e6` | 13.31 ns | 5.95 us | 6.07 us | 16.05 us | 1.09 us | 2.11 us | 447.46x | 0.37x | 5.45x | 2.82x |
| `cos 1e6` | 12.89 ns | 6.10 us | 5.99 us | 13.76 us | 828.57 ns | 1.95 us | 473.13x | 0.44x | 7.36x | 3.13x |
| `sin 1e30` | 66.66 ns | 8.16 us | 8.64 us | 18.52 us | 2.92 us | 3.67 us | 122.41x | 0.44x | 2.80x | 2.22x |
| `cos 1e30` | 70.33 ns | 8.75 us | 8.45 us | 15.67 us | 1.29 us | 3.15 us | 124.45x | 0.56x | 6.78x | 2.78x |
| `sin pi_7` | 11.86 ns | 173.73 ns | 527.31 ns | 12.16 us | 746.91 ns | 1.93 us | 14.65x | 0.01x | 0.23x | 0.09x |
| `cos pi_7` | 11.92 ns | 178.27 ns | 988.46 ns | 11.29 us | 548.50 ns | 1.73 us | 14.95x | 0.02x | 0.33x | 0.10x |
| `sin 1000pi_eps` | 12.23 ns | 6.04 us | 3.68 us | 16.27 us | 2.37 us | 2.91 us | 493.50x | 0.37x | 2.55x | 2.07x |
| `cos 1000pi_eps` | 12.74 ns | 6.69 us | 3.63 us | 14.23 us | 590.48 ns | 1.70 us | 524.87x | 0.47x | 11.33x | 3.93x |
| `asin 0.5` | 11.48 ns | 619.58 ns | 565.77 ns | 51.14 us | 2.96 us | 13.38 us | 53.97x | 0.01x | 0.21x | 0.05x |
| `acos 0.5` | 11.95 ns | 1.24 us | 1.15 us | 60.40 us | 2.95 us | 13.39 us | 103.36x | 0.02x | 0.42x | 0.09x |
| `atanh 0.5` | 14.82 ns | 1.57 us | 1.49 us | 36.02 us | 1.75 us | 12.83 us | 105.71x | 0.04x | 0.89x | 0.12x |
| `asin neg_0.999999` | 14.26 ns | 5.89 us | 5.13 us | 13.83 us | 2.57 us | 13.42 us | 413.26x | 0.43x | 2.29x | 0.44x |
| `acos neg_0.999999` | 15.39 ns | 6.54 us | 5.95 us | 18.37 us | 2.68 us | 13.60 us | 424.82x | 0.36x | 2.44x | 0.48x |
| `atanh neg_0.999999` | 14.70 ns | 4.17 us | 4.00 us | 36.31 us | 1.56 us | 13.39 us | 283.57x | 0.11x | 2.67x | 0.31x |
| `asin 0.999999` | 14.12 ns | 5.92 us | 5.15 us | 13.62 us | 2.50 us | 13.12 us | 419.10x | 0.43x | 2.36x | 0.45x |
| `acos 0.999999` | 14.51 ns | 5.57 us | 5.32 us | 18.38 us | 2.80 us | 13.31 us | 383.70x | 0.30x | 1.99x | 0.42x |
| `atanh 0.999999` | 14.85 ns | 4.36 us | 3.81 us | 31.51 us | 1.64 us | 12.78 us | 293.89x | 0.14x | 2.66x | 0.34x |
| `asin 1e-12` | 9.85 ns | 557.90 ns | 1.19 us | 8.05 us | 1.40 us | 15.56 us | 56.61x | 0.07x | 0.40x | 0.04x |
| `acos 1e-12` | 10.45 ns | 957.60 ns | 1.96 us | 9.88 us | 1.44 us | 15.25 us | 91.66x | 0.10x | 0.67x | 0.06x |
| `atanh 1e-12` | 9.89 ns | 539.83 ns | 967.27 ns | 37.58 us | 171.26 ns | 20.58 us | 54.59x | 0.01x | 3.15x | 0.03x |
| `atan 0.5` | 15.30 ns | 455.84 ns | 458.64 ns | 35.39 us | 2.79 us | 18.29 us | 29.80x | 0.01x | 0.16x | 0.02x |
| `asinh 0.5` | 27.06 ns | 1.67 us | 1.66 us | 40.66 us | 1.61 us | 7.48 us | 61.70x | 0.04x | 1.04x | 0.22x |
| `atan neg_1e-12` | 14.66 ns | 471.33 ns | 701.44 ns | 1.60 us | 1.14 us | 15.62 us | 32.14x | 0.29x | 0.41x | 0.03x |
| `asinh neg_1e-12` | 15.93 ns | 5.53 us | 4.61 us | 42.90 us | 8.55 us | 12.33 us | 346.85x | 0.13x | 0.65x | 0.45x |
| `atan 1e6` | 15.66 ns | 613.63 ns | 623.34 ns | 2.87 us | 1.45 us | 18.12 us | 39.18x | 0.21x | 0.42x | 0.03x |
| `asinh 1e6` | 27.12 ns | 3.30 us | 3.36 us | 39.19 us | 1.68 us | 7.20 us | 121.49x | 0.08x | 1.96x | 0.46x |
| `atan neg_1e6` | 16.83 ns | 711.30 ns | 753.08 ns | 2.82 us | 1.48 us | 17.97 us | 42.27x | 0.25x | 0.48x | 0.04x |
| `asinh neg_1e6` | 27.11 ns | 3.35 us | 3.50 us | 39.80 us | 1.65 us | 7.04 us | 123.55x | 0.08x | 2.03x | 0.48x |
| `acosh 9` | 12.81 ns | 2.98 us | 3.22 us | 42.27 us | 1.63 us | 9.75 us | 232.24x | 0.07x | 1.83x | 0.31x |
| `acosh 1_plus_1e-12` | 11.71 ns | 4.06 us | 5.84 us | 41.88 us | 8.36 us | 11.39 us | 346.33x | 0.10x | 0.49x | 0.36x |
| `acosh 1e6` | 12.84 ns | 3.85 us | 3.76 us | 37.65 us | 1.62 us | 10.28 us | 300.25x | 0.10x | 2.37x | 0.38x |
| `acosh e` | 12.86 ns | 4.24 us | 4.12 us | 40.51 us | 1.63 us | 10.27 us | 329.71x | 0.10x | 2.61x | 0.41x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.49 ns | 58.15 ns | 55.85 ns | 25.15 ns | 15.65 ns | 0.95 ns | 118.40x | 2.31x | 3.71x | 61.27x |
| `one` | 0.49 ns | 66.53 ns | 65.17 ns | 42.58 ns | 31.15 ns | 30.26 ns | 136.78x | 1.56x | 2.14x | 2.20x |
| `e` | 0.49 ns | 76.53 ns | 75.76 ns | 54.92 ns | 1.07 us | 226.26 ns | 155.08x | 1.39x | 0.07x | 0.34x |
| `pi` | 0.48 ns | 61.10 ns | 55.15 ns | 43.51 ns | 48.75 ns | 227.35 ns | 126.39x | 1.40x | 1.25x | 0.27x |
| `tau` | 0.48 ns | 191.89 ns | 196.86 ns | 120.49 ns | 100.29 ns | 1.85 us | 399.84x | 1.59x | 1.91x | 0.10x |
| `add` | 5.38 ns | 316.95 ns | 393.49 ns | 53.40 ns | 42.35 ns | 1.40 us | 58.91x | 5.93x | 7.48x | 0.23x |
| `sub` | 5.30 ns | 322.83 ns | 456.62 ns | 59.25 ns | 45.89 ns | 2.53 us | 60.92x | 5.45x | 7.04x | 0.13x |
| `neg` | 5.16 ns | 65.53 ns | 61.49 ns | 23.69 ns | 22.06 ns | 1.07 us | 12.69x | 2.77x | 2.97x | 0.06x |
| `mul` | 5.59 ns | 228.33 ns | 524.81 ns | 58.92 ns | 51.67 ns | 1.54 us | 40.83x | 3.87x | 4.42x | 0.15x |
| `div` | 9.19 ns | 408.61 ns | 474.71 ns | 139.73 ns | 64.72 ns | 2.55 us | 44.45x | 2.92x | 6.31x | 0.16x |
| `reciprocal` | 8.96 ns | 141.37 ns | 131.37 ns | 164.17 ns | 60.46 ns | 1.53 us | 15.78x | 0.86x | 2.34x | 0.09x |
| `reciprocal checked` | 9.30 ns | 125.18 ns | 135.21 ns | 163.82 ns | 61.18 ns | 1.53 us | 13.46x | 0.76x | 2.05x | 0.08x |
| `reciprocal checked abort` | 24.65 ns | 131.00 ns | 146.75 ns | 165.88 ns | 61.91 ns | 1.56 us | 5.31x | 0.79x | 2.12x | 0.08x |
| `pow` | 20.48 ns | 10.95 us | 11.14 us | 54.36 us | 2.90 us | 2.42 us | 534.57x | 0.20x | 3.77x | 4.52x |
| `powi` | 6.25 ns | 572.87 ns | 3.08 us | 292.24 ns | 85.82 ns | 1.73 us | 91.73x | 1.96x | 6.68x | 0.33x |
| `exp` | 20.51 ns | 1.81 us | 3.15 us | 13.94 us | 947.89 ns | 2.04 us | 88.05x | 0.13x | 1.91x | 0.89x |
| `ln` | 11.03 ns | 1.64 us | 1.60 us | 30.02 us | 1.31 us | 1.85 us | 148.53x | 0.05x | 1.25x | 0.88x |
| `log10` | 13.55 ns | 2.89 us | 2.43 us | 35.62 us | 2.76 us | 6.89 us | 213.63x | 0.08x | 1.05x | 0.42x |
| `log10 abort` | 17.35 ns | 2.88 us | 2.45 us | 35.39 us | 2.74 us | 6.75 us | 165.84x | 0.08x | 1.05x | 0.43x |
| `sqrt` | 8.53 ns | 1.63 us | 1.75 us | 5.23 us | 98.31 ns | 1.47 us | 190.61x | 0.31x | 16.53x | 1.11x |
| `sin` | 15.44 ns | 3.25 us | 3.69 us | 14.32 us | 1.28 us | 2.32 us | 210.31x | 0.23x | 2.54x | 1.40x |
| `cos` | 19.11 ns | 3.13 us | 3.74 us | 12.30 us | 649.67 ns | 1.76 us | 164.01x | 0.25x | 4.82x | 1.79x |
| `tan` | 25.28 ns | 2.94 us | 3.33 us | 29.89 us | 1.64 us | 6.73 us | 116.45x | 0.10x | 1.80x | 0.44x |
| `sinh` | 18.37 ns | 4.53 us | 4.53 us | 3.38 us | 1.16 us | 11.16 us | 246.61x | 1.34x | 3.89x | 0.41x |
| `cosh` | 18.56 ns | 5.28 us | 4.79 us | 7.99 us | 1.09 us | 9.92 us | 284.53x | 0.66x | 4.83x | 0.53x |
| `tanh` | 23.41 ns | 6.49 us | 6.17 us | 3.41 us | 1.23 us | 23.28 us | 276.99x | 1.90x | 5.26x | 0.28x |
| `asin` | 13.74 ns | 3.26 us | 4.79 us | 21.58 us | 2.42 us | 14.04 us | 237.33x | 0.15x | 1.35x | 0.23x |
| `asin abort` | 17.33 ns | 3.28 us | 4.79 us | 21.19 us | 2.48 us | 14.03 us | 189.52x | 0.16x | 1.33x | 0.23x |
| `acos` | 14.36 ns | 3.69 us | 6.17 us | 26.37 us | 2.60 us | 14.03 us | 256.81x | 0.14x | 1.42x | 0.26x |
| `acos abort` | 17.84 ns | 3.59 us | 5.81 us | 26.40 us | 2.69 us | 13.92 us | 201.44x | 0.14x | 1.33x | 0.26x |
| `atan` | 15.02 ns | 575.93 ns | 1.39 us | 18.83 us | 2.34 us | 19.46 us | 38.35x | 0.03x | 0.25x | 0.03x |
| `atan abort` | 22.11 ns | 576.80 ns | 1.32 us | 18.55 us | 2.33 us | 19.54 us | 26.08x | 0.03x | 0.25x | 0.03x |
| `asinh` | 34.31 ns | 3.72 us | 5.55 us | 39.07 us | 1.72 us | 7.67 us | 108.45x | 0.10x | 2.16x | 0.49x |
| `asinh abort` | 31.59 ns | 3.69 us | 5.43 us | 39.41 us | 1.64 us | 7.77 us | 116.67x | 0.09x | 2.25x | 0.47x |
| `acosh` | 12.66 ns | 3.75 us | 5.25 us | 41.29 us | 3.34 us | 10.50 us | 296.19x | 0.09x | 1.12x | 0.36x |
| `acosh abort` | 17.19 ns | 3.88 us | 5.45 us | 42.44 us | 3.32 us | 10.76 us | 225.83x | 0.09x | 1.17x | 0.36x |
| `atanh` | 13.80 ns | 2.75 us | 3.32 us | 34.90 us | 1.26 us | 15.38 us | 199.01x | 0.08x | 2.18x | 0.18x |
| `atanh abort` | 17.33 ns | 2.72 us | 3.41 us | 34.63 us | 1.25 us | 14.88 us | 156.89x | 0.08x | 2.17x | 0.18x |
| `zero status` | 1.27 ns | 1.81 ns | 1.82 ns | 0.98 ns | 6.79 ns | 8.24 ns | 1.42x | 1.85x | 0.27x | 0.22x |
| `zero status abort` | 1.42 ns | 3.17 ns | 3.54 ns | 1.09 ns | 7.07 ns | 8.25 ns | 2.23x | 2.92x | 0.45x | 0.38x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 120.43 ns | 115.86 ns | 48.97 ns | - | 1.90 ns | 128.00x | 2.46x | - | 63.44x |
| `one` | 5.45 ns | 127.56 ns | 121.73 ns | 66.96 ns | - | 30.60 ns | 23.42x | 1.90x | - | 4.17x |
| `i` | 0.97 ns | 129.25 ns | 126.06 ns | 66.89 ns | - | 30.00 ns | 133.29x | 1.93x | - | 4.31x |
| `free i` | 0.96 ns | 124.58 ns | 125.64 ns | 66.00 ns | - | 31.28 ns | 129.75x | 1.89x | - | 3.98x |
| `conjugate` | 2.18 ns | 128.63 ns | 128.24 ns | 41.81 ns | - | 1.14 us | 58.90x | 3.08x | - | 0.11x |
| `norm squared` | 6.06 ns | 402.57 ns | 1.57 us | 153.29 ns | - | 4.43 us | 66.40x | 2.63x | - | 0.09x |
| `reciprocal` | 16.00 ns | 2.03 us | 3.15 us | 471.24 ns | - | 10.90 us | 126.83x | 4.31x | - | 0.19x |
| `reciprocal checked` | 14.91 ns | 1.95 us | 3.10 us | 470.97 ns | - | 11.00 us | 130.68x | 4.14x | - | 0.18x |
| `powi` | 18.12 ns | 2.76 us | 14.56 us | 1.44 us | - | 43.65 us | 152.50x | 1.92x | - | 0.06x |
| `powi checked` | 17.78 ns | 2.81 us | 14.27 us | 1.45 us | - | 44.56 us | 158.32x | 1.95x | - | 0.06x |
| `div checked` | 21.20 ns | 3.30 us | 6.78 us | 813.00 ns | - | 21.84 us | 155.65x | 4.06x | - | 0.15x |
| `div real checked` | 9.51 ns | 818.39 ns | 780.88 ns | 268.59 ns | - | 5.44 us | 86.09x | 3.05x | - | 0.15x |
| `from scalar` | 1.42 ns | 120.59 ns | 129.34 ns | 46.44 ns | - | 11.52 ns | 84.72x | 2.60x | - | 10.46x |
| `add` | 6.27 ns | 670.68 ns | 1.12 us | 105.38 ns | - | 2.58 us | 106.93x | 6.36x | - | 0.26x |
| `sub` | 6.33 ns | 611.34 ns | 1.04 us | 127.14 ns | - | 4.95 us | 96.63x | 4.81x | - | 0.12x |
| `neg` | 2.66 ns | 128.38 ns | 141.32 ns | 42.33 ns | - | 2.14 us | 48.19x | 3.03x | - | 0.06x |
| `mul` | 7.99 ns | 1.14 us | 3.49 us | 308.86 ns | - | 10.32 us | 142.55x | 3.69x | - | 0.11x |
| `div` | 19.05 ns | 3.10 us | 6.89 us | 782.09 ns | - | 22.12 us | 162.80x | 3.97x | - | 0.14x |
| `div real` | 10.28 ns | 786.35 ns | 774.91 ns | 266.28 ns | - | 5.26 us | 76.53x | 2.95x | - | 0.15x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.98 ns | 847.87 ns | 6.29 us | 312.36 ns | 271.45 ns | 7.39 us | 121.52x | 2.71x | 3.12x | 0.11x |
| `vec3 magnitude` | 9.68 ns | 4.82 us | 9.51 us | 5.64 us | 366.54 ns | 9.21 us | 498.24x | 0.86x | 13.16x | 0.52x |
| `vec3 normalize` | 26.31 ns | 9.32 us | 11.68 us | 6.33 us | 624.00 ns | 17.08 us | 354.38x | 1.47x | 14.94x | 0.55x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.10 ns | 308.73 ns | 1.95 us | 71.45 ns | 61.11 ns | 743.31 ns | 99.70x | 4.32x | 5.05x | 0.42x |
| `vec3 zero` | 1.41 ns | 254.61 ns | 255.49 ns | 67.09 ns | 32.25 ns | 2.85 ns | 180.36x | 3.79x | 7.89x | 89.31x |
| `vec3 dot abort` | 27.87 ns | 1.27 us | 4.88 us | 271.09 ns | 209.57 ns | 7.17 us | 45.40x | 4.67x | 6.04x | 0.18x |
| `vec3 magnitude abort` | 40.46 ns | 5.14 us | 7.23 us | 5.77 us | 338.89 ns | 8.91 us | 126.95x | 0.89x | 15.16x | 0.58x |
| `vec3 normalize checked` | 26.02 ns | 8.83 us | 9.89 us | 6.04 us | 573.36 ns | 18.20 us | 339.40x | 1.46x | 15.40x | 0.49x |
| `vec3 normalize checked abort` | 57.35 ns | 9.35 us | 9.64 us | 6.04 us | 572.73 ns | 17.44 us | 163.05x | 1.55x | 16.33x | 0.54x |
| `vec3 div scalar checked` | 17.65 ns | 1.56 us | 1.93 us | 399.88 ns | - | - | 88.56x | 3.91x | - | - |
| `vec3 div scalar checked abort` | 19.02 ns | 1.61 us | 1.94 us | 405.26 ns | - | - | 84.41x | 3.96x | - | - |
| `vec3 add` | 6.75 ns | 1.26 us | 2.03 us | 154.34 ns | 128.14 ns | 4.11 us | 187.07x | 8.18x | 9.85x | 0.31x |
| `vec3 add scalar` | 6.45 ns | 999.04 ns | 1.70 us | 154.55 ns | 139.24 ns | 3.89 us | 154.82x | 6.46x | 7.18x | 0.26x |
| `vec3 sub` | 6.82 ns | 1.22 us | 2.11 us | 177.96 ns | 138.50 ns | 7.64 us | 178.68x | 6.85x | 8.80x | 0.16x |
| `vec3 sub scalar` | 6.42 ns | 956.04 ns | 1.44 us | 146.33 ns | 130.29 ns | 7.54 us | 148.99x | 6.53x | 7.34x | 0.13x |
| `vec3 neg` | 3.79 ns | 256.23 ns | 263.50 ns | 61.51 ns | 52.94 ns | 3.22 us | 67.61x | 4.17x | 4.84x | 0.08x |
| `vec3 mul scalar` | 7.05 ns | 739.06 ns | 2.39 us | 175.54 ns | 128.61 ns | 4.54 us | 104.79x | 4.21x | 5.75x | 0.16x |
| `vec3 div scalar` | 10.51 ns | 1.55 us | 2.00 us | 401.60 ns | 181.09 ns | 7.84 us | 147.07x | 3.85x | 8.54x | 0.20x |
| `vec4 dot` | 7.48 ns | 1.02 us | 3.70 us | 459.12 ns | 332.85 ns | 9.81 us | 136.48x | 2.22x | 3.07x | 0.10x |
| `vec4 magnitude` | 13.63 ns | 4.72 us | 5.82 us | 5.80 us | 433.49 ns | 11.47 us | 345.95x | 0.81x | 10.88x | 0.41x |
| `vec4 normalize` | 39.73 ns | 9.19 us | 7.92 us | 6.45 us | 756.21 ns | 22.36 us | 231.32x | 1.43x | 12.15x | 0.41x |
| `vec4 add` | 7.69 ns | 1.61 us | 2.25 us | 203.31 ns | 177.86 ns | 5.41 us | 209.31x | 7.92x | 9.05x | 0.30x |
| `vec4 add scalar` | 7.36 ns | 1.32 us | 2.04 us | 233.67 ns | 180.86 ns | 5.33 us | 178.96x | 5.63x | 7.28x | 0.25x |
| `vec4 sub` | 5.15 ns | 1.61 us | 2.25 us | 220.30 ns | 179.32 ns | 9.90 us | 312.66x | 7.31x | 8.98x | 0.16x |
| `vec4 sub scalar` | 4.67 ns | 1.32 us | 1.87 us | 209.11 ns | 174.23 ns | 9.59 us | 283.37x | 6.33x | 7.60x | 0.14x |
| `vec4 neg` | 5.04 ns | 336.51 ns | 336.41 ns | 80.44 ns | 67.72 ns | 4.17 us | 66.80x | 4.18x | 4.97x | 0.08x |
| `vec4 mul scalar` | 7.50 ns | 949.33 ns | 2.40 us | 219.33 ns | 166.54 ns | 5.92 us | 126.52x | 4.33x | 5.70x | 0.16x |
| `vec4 div scalar` | 14.51 ns | 2.15 us | 1.86 us | 555.97 ns | 232.83 ns | 10.36 us | 148.01x | 3.86x | 9.22x | 0.21x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.37 ns | 1.98 us | 3.58 us | 974.72 ns | 901.18 ns | 22.71 us | 160.22x | 2.03x | 2.20x | 0.09x |
| `mat3 inverse` | 83.61 ns | 18.89 us | 12.78 us | 3.28 us | 2.55 us | 84.99 us | 226.00x | 5.76x | 7.42x | 0.22x |
| `mat3 mul mat3` | 72.96 ns | 8.15 us | 13.15 us | 2.86 us | 2.41 us | 63.53 us | 111.75x | 2.85x | 3.38x | 0.13x |
| `mat3 transform vec3` | 16.51 ns | 3.43 us | 11.71 us | 1.51 us | 908.51 ns | 20.63 us | 207.74x | 2.28x | 3.78x | 0.17x |
| `mat4 determinant` | 46.84 ns | 5.59 us | 4.17 us | 4.71 us | 4.20 us | 96.81 us | 119.42x | 1.19x | 1.33x | 0.06x |
| `mat4 inverse` | 156.57 ns | 36.74 us | 17.19 us | 11.45 us | 9.35 us | 349.94 us | 234.64x | 3.21x | 3.93x | 0.10x |
| `mat4 mul mat4` | 128.04 ns | 16.96 us | 14.29 us | 6.55 us | 5.44 us | 146.14 us | 132.46x | 2.59x | 3.12x | 0.12x |
| `mat4 transform vec4` | 26.28 ns | 5.82 us | 4.43 us | 1.94 us | 1.73 us | 37.67 us | 221.45x | 3.00x | 3.36x | 0.15x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.63 ns | 1.36 us | 4.65 us | 218.93 ns | 244.54 ns | 2.09 us | 38.24x | 6.22x | 5.57x | 0.65x |
| `mat3 zero` | 16.48 ns | 874.02 ns | 859.27 ns | 250.46 ns | 210.97 ns | 12.49 ns | 53.03x | 3.49x | 4.14x | 69.98x |
| `mat3 identity` | 10.04 ns | 974.27 ns | 992.92 ns | 313.13 ns | 241.04 ns | 152.40 ns | 97.03x | 3.11x | 4.04x | 6.39x |
| `mat3 transpose` | 9.16 ns | 950.78 ns | 929.20 ns | 243.00 ns | 212.19 ns | 138.80 ns | 103.77x | 3.91x | 4.48x | 6.85x |
| `mat3 reciprocal` | 84.22 ns | 18.98 us | 36.76 us | 3.00 us | 2.41 us | 89.17 us | 225.33x | 6.32x | 7.88x | 0.21x |
| `mat3 reciprocal checked` | 81.82 ns | 18.93 us | 39.29 us | 3.15 us | 2.42 us | 84.90 us | 231.36x | 6.01x | 7.83x | 0.22x |
| `mat3 inverse checked` | 81.76 ns | 19.14 us | 38.57 us | 3.02 us | 2.36 us | 88.76 us | 234.05x | 6.34x | 8.11x | 0.22x |
| `mat3 inverse checked abort` | 122.83 ns | 18.84 us | 38.21 us | 3.05 us | 2.38 us | 85.51 us | 153.42x | 6.18x | 7.93x | 0.22x |
| `mat3 powi` | 160.11 ns | 18.17 us | 100.34 us | 6.78 us | 6.20 us | 152.32 us | 113.50x | 2.68x | 2.93x | 0.12x |
| `mat3 powi checked` | 160.42 ns | 17.01 us | 93.22 us | 6.69 us | 6.15 us | 153.88 us | 106.02x | 2.54x | 2.77x | 0.11x |
| `mat3 powi checked abort` | 148.02 ns | 17.09 us | 107.79 us | 6.74 us | 6.33 us | 152.83 us | 115.47x | 2.54x | 2.70x | 0.11x |
| `mat3 div scalar checked` | 25.40 ns | 4.94 us | 6.42 us | 1.48 us | 833.02 ns | 23.24 us | 194.60x | 3.34x | 5.93x | 0.21x |
| `mat3 div scalar checked abort` | 29.52 ns | 4.90 us | 6.36 us | 1.51 us | 827.77 ns | 22.68 us | 166.13x | 3.24x | 5.92x | 0.22x |
| `mat3 div matrix checked` | 154.25 ns | 66.94 us | 65.99 us | 5.53 us | 4.63 us | 164.33 us | 433.96x | 12.10x | 14.45x | 0.41x |
| `mat3 div matrix checked abort` | 163.88 ns | 73.16 us | 66.03 us | 5.51 us | 4.58 us | 165.27 us | 446.39x | 13.28x | 15.97x | 0.44x |
| `mat3 add` | 14.66 ns | 4.19 us | 6.62 us | 540.37 ns | 492.99 ns | 11.86 us | 285.42x | 7.75x | 8.49x | 0.35x |
| `mat3 add scalar` | 12.23 ns | 3.14 us | 5.28 us | 800.79 ns | 740.72 ns | 12.19 us | 256.33x | 3.92x | 4.23x | 0.26x |
| `mat3 sub` | 13.37 ns | 4.29 us | 6.16 us | 572.60 ns | 526.32 ns | 23.10 us | 320.96x | 7.50x | 8.16x | 0.19x |
| `mat3 sub scalar` | 11.13 ns | 3.11 us | 5.00 us | 815.91 ns | 719.61 ns | 22.48 us | 279.09x | 3.81x | 4.32x | 0.14x |
| `mat3 neg` | 10.43 ns | 956.94 ns | 877.03 ns | 504.88 ns | 464.28 ns | 9.08 us | 91.73x | 1.90x | 2.06x | 0.11x |
| `mat3 mul scalar` | 13.89 ns | 2.33 us | 6.44 us | 793.23 ns | 691.01 ns | 12.38 us | 167.92x | 2.94x | 3.38x | 0.19x |
| `mat3 div scalar` | 25.23 ns | 5.01 us | 6.21 us | 1.51 us | 821.14 ns | 23.53 us | 198.56x | 3.31x | 6.10x | 0.21x |
| `mat3 div matrix` | 150.78 ns | 67.23 us | 66.05 us | 5.49 us | 4.59 us | 164.15 us | 445.87x | 12.24x | 14.66x | 0.41x |
| `mat3 bitxor` | 158.45 ns | 18.97 us | 93.99 us | 7.08 us | 6.63 us | 151.39 us | 119.75x | 2.68x | 2.86x | 0.13x |
| `mat4 zero` | 11.29 ns | 1.78 us | 1.79 us | 476.09 ns | 364.06 ns | 15.19 ns | 157.86x | 3.74x | 4.89x | 117.28x |
| `mat4 identity` | 11.37 ns | 1.92 us | 1.97 us | 590.61 ns | 433.36 ns | 245.02 ns | 168.92x | 3.25x | 4.43x | 7.84x |
| `mat4 transpose` | 9.45 ns | 1.67 us | 1.68 us | 432.47 ns | 381.04 ns | 211.70 ns | 176.58x | 3.86x | 4.38x | 7.88x |
| `mat4 reciprocal` | 156.53 ns | 36.55 us | 68.02 us | 10.93 us | 9.10 us | 354.68 us | 233.50x | 3.34x | 4.01x | 0.10x |
| `mat4 reciprocal checked` | 167.58 ns | 35.83 us | 65.37 us | 10.99 us | 9.13 us | 347.92 us | 213.83x | 3.26x | 3.92x | 0.10x |
| `mat4 powi` | 245.25 ns | 34.47 us | 118.31 us | 16.17 us | 14.66 us | 361.90 us | 140.56x | 2.13x | 2.35x | 0.10x |
| `mat4 powi checked` | 246.07 ns | 37.29 us | 113.84 us | 16.48 us | 15.19 us | 352.22 us | 151.56x | 2.26x | 2.46x | 0.11x |
| `mat4 add` | 51.69 ns | 6.80 us | 7.47 us | 949.57 ns | 878.32 ns | 19.76 us | 131.48x | 7.16x | 7.74x | 0.34x |
| `mat4 add scalar` | 24.50 ns | 5.10 us | 7.69 us | 1.50 us | 1.25 us | 21.31 us | 208.13x | 3.41x | 4.08x | 0.24x |
| `mat4 sub` | 39.40 ns | 6.75 us | 7.70 us | 950.22 ns | 944.78 ns | 37.83 us | 171.23x | 7.10x | 7.14x | 0.18x |
| `mat4 sub scalar` | 19.07 ns | 5.54 us | 7.62 us | 1.45 us | 1.19 us | 39.71 us | 290.77x | 3.82x | 4.64x | 0.14x |
| `mat4 neg` | 14.53 ns | 1.59 us | 1.57 us | 920.91 ns | 743.29 ns | 14.17 us | 109.28x | 1.72x | 2.14x | 0.11x |
| `mat4 mul scalar` | 26.03 ns | 4.06 us | 8.65 us | 1.45 us | 1.12 us | 20.42 us | 156.06x | 2.81x | 3.62x | 0.20x |
| `mat4 div scalar` | 39.19 ns | 8.65 us | 8.74 us | 2.73 us | 1.43 us | 40.06 us | 220.70x | 3.17x | 6.06x | 0.22x |
| `mat4 div matrix` | 228.07 ns | 136.53 us | 105.84 us | 17.23 us | 14.59 us | 594.20 us | 598.62x | 7.92x | 9.36x | 0.23x |
| `mat4 bitxor` | 246.52 ns | 34.20 us | 124.52 us | 15.80 us | 15.05 us | 363.38 us | 138.73x | 2.16x | 2.27x | 0.09x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.67 ns | 228.66 ns | 316.94 ns | - | - | 1.33 us | 62.37x | - | - | 0.17x |
| `scalar add ref_owned` | 12.29 ns | 207.74 ns | 321.13 ns | - | - | 1.32 us | 16.90x | - | - | 0.16x |
| `scalar add refs` | 5.57 ns | 188.42 ns | 295.13 ns | - | - | 1.32 us | 33.82x | - | - | 0.14x |
| `scalar add owned_ref_with_clone` | 9.13 ns | 270.03 ns | 352.42 ns | - | - | - | 29.59x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.57 ns | 257.73 ns | 351.39 ns | - | - | - | 46.28x | - | - | - |
| `scalar sub owned_ref` | 4.17 ns | 221.27 ns | 381.55 ns | - | - | 2.54 us | 53.06x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.62 ns | 223.70 ns | 372.87 ns | - | - | 2.54 us | 17.73x | - | - | 0.09x |
| `scalar sub refs` | 5.76 ns | 193.63 ns | 362.64 ns | - | - | 2.51 us | 33.63x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.19 ns | 272.95 ns | 407.21 ns | - | - | - | 29.70x | - | - | - |
| `scalar sub ref_owned_with_clone` | 6.50 ns | 264.85 ns | 407.70 ns | - | - | - | 40.75x | - | - | - |
| `scalar mul owned_ref` | 4.93 ns | 128.54 ns | 422.83 ns | - | - | 1.61 us | 26.06x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.61 ns | 127.13 ns | 403.95 ns | - | - | 1.56 us | 9.34x | - | - | 0.08x |
| `scalar mul refs` | 6.28 ns | 107.61 ns | 387.72 ns | - | - | 1.58 us | 17.12x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 9.80 ns | 174.98 ns | 443.38 ns | - | - | - | 17.86x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.58 ns | 177.23 ns | 433.93 ns | - | - | - | 26.92x | - | - | - |
| `scalar div owned_ref` | 6.29 ns | 312.87 ns | 352.48 ns | - | - | 2.59 us | 49.74x | - | - | 0.12x |
| `scalar div ref_owned` | 17.88 ns | 315.54 ns | 367.08 ns | - | - | 2.69 us | 17.65x | - | - | 0.12x |
| `scalar div refs` | 8.90 ns | 274.49 ns | 341.66 ns | - | - | 2.60 us | 30.83x | - | - | 0.11x |
| `scalar div owned_ref_with_clone` | 16.11 ns | 345.56 ns | 380.21 ns | - | - | - | 21.44x | - | - | - |
| `scalar div ref_owned_with_clone` | 14.11 ns | 354.02 ns | 399.07 ns | - | - | - | 25.09x | - | - | - |
| `vec3 add refs` | 6.10 ns | 722.58 ns | 1.51 us | - | - | 4.13 us | 118.47x | - | - | 0.18x |
| `vec3 sub refs` | 6.16 ns | 750.60 ns | 1.54 us | - | - | 7.34 us | 121.76x | - | - | 0.10x |
| `vec3 neg ref` | 3.31 ns | 284.51 ns | 268.67 ns | - | - | 3.24 us | 86.01x | - | - | 0.09x |
| `vec3 add_scalar_ref` | 6.75 ns | 959.52 ns | 1.77 us | - | - | 4.04 us | 142.08x | - | - | 0.24x |
| `vec3 sub_scalar_ref` | 6.74 ns | 946.76 ns | 1.47 us | - | - | 7.12 us | 140.51x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 7.04 ns | 688.24 ns | 2.21 us | - | - | 4.58 us | 97.72x | - | - | 0.15x |
| `vec3 div_scalar_ref` | 8.46 ns | 1.62 us | 1.73 us | - | - | 8.01 us | 191.34x | - | - | 0.20x |
| `vec4 add refs` | 7.01 ns | 945.83 ns | 1.59 us | - | - | 5.34 us | 135.02x | - | - | 0.18x |
| `vec4 sub refs` | 3.17 ns | 903.13 ns | 1.40 us | - | - | 10.12 us | 284.78x | - | - | 0.09x |
| `vec4 neg ref` | 4.27 ns | 348.92 ns | 347.67 ns | - | - | 4.33 us | 81.68x | - | - | 0.08x |
| `vec4 add_scalar_ref` | 7.01 ns | 1.25 us | 1.98 us | - | - | 5.12 us | 178.72x | - | - | 0.25x |
| `vec4 sub_scalar_ref` | 4.25 ns | 1.26 us | 1.77 us | - | - | 9.68 us | 297.26x | - | - | 0.13x |
| `vec4 mul_scalar_ref` | 7.52 ns | 912.59 ns | 2.30 us | - | - | 5.81 us | 121.33x | - | - | 0.16x |
| `vec4 div_scalar_ref` | 12.28 ns | 1.97 us | 1.79 us | - | - | 10.10 us | 160.77x | - | - | 0.20x |
| `mat3 add refs` | 11.87 ns | 2.15 us | 4.15 us | - | - | 11.71 us | 180.87x | - | - | 0.18x |
| `mat3 sub refs` | 11.47 ns | 1.98 us | 4.05 us | - | - | 21.51 us | 172.68x | - | - | 0.09x |
| `mat3 mul refs` | 33.66 ns | 6.78 us | 29.49 us | - | - | 63.01 us | 201.33x | - | - | 0.11x |
| `mat3 div refs` | 141.38 ns | 65.79 us | 63.66 us | - | - | 166.43 us | 465.35x | - | - | 0.40x |
| `mat3 neg ref` | 9.71 ns | 800.50 ns | 775.96 ns | - | - | 8.70 us | 82.41x | - | - | 0.09x |
| `mat3 add_scalar_ref` | 45.70 ns | 2.95 us | 5.13 us | - | - | 12.61 us | 64.52x | - | - | 0.23x |
| `mat3 sub_scalar_ref` | 45.67 ns | 2.97 us | 4.92 us | - | - | 21.95 us | 65.01x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 47.07 ns | 2.21 us | 6.23 us | - | - | 12.62 us | 47.05x | - | - | 0.18x |
| `mat3 div_scalar_ref` | 21.61 ns | 4.88 us | 6.25 us | - | - | 23.19 us | 225.87x | - | - | 0.21x |
| `mat4 add refs` | 17.63 ns | 3.29 us | 4.06 us | - | - | 19.75 us | 186.44x | - | - | 0.17x |
| `mat4 sub refs` | 17.06 ns | 3.21 us | 4.19 us | - | - | 36.67 us | 188.34x | - | - | 0.09x |
| `mat4 mul refs` | 73.36 ns | 13.72 us | 29.58 us | - | - | 149.29 us | 187.03x | - | - | 0.09x |
| `mat4 div refs` | 178.36 ns | 129.26 us | 104.82 us | - | - | 542.19 us | 724.73x | - | - | 0.24x |
| `mat4 neg ref` | 13.49 ns | 1.27 us | 1.23 us | - | - | 14.40 us | 94.43x | - | - | 0.09x |
| `mat4 add_scalar_ref` | 53.58 ns | 5.22 us | 7.62 us | - | - | 20.54 us | 97.39x | - | - | 0.25x |
| `mat4 sub_scalar_ref` | 41.67 ns | 5.61 us | 7.44 us | - | - | 38.83 us | 134.67x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 57.74 ns | 4.21 us | 8.38 us | - | - | 20.50 us | 72.88x | - | - | 0.21x |
| `mat4 div_scalar_ref` | 28.35 ns | 8.63 us | 8.86 us | - | - | 37.84 us | 304.60x | - | - | 0.23x |
| `mat3 transform_vec refs` | 14.57 ns | 2.48 us | 11.71 us | - | - | 20.51 us | 170.15x | - | - | 0.12x |
| `mat4 transform_vec refs` | 23.54 ns | 4.03 us | 11.62 us | - | - | 37.45 us | 171.07x | - | - | 0.11x |
| `complex add refs` | 7.76 ns | 386.96 ns | 757.58 ns | - | - | 2.59 us | 49.87x | - | - | 0.15x |
| `complex sub refs` | 8.01 ns | 398.66 ns | 795.19 ns | - | - | 4.94 us | 49.74x | - | - | 0.08x |
| `complex mul refs` | 8.12 ns | 908.79 ns | 3.39 us | - | - | 10.26 us | 111.92x | - | - | 0.09x |
| `complex div refs` | 17.89 ns | 2.91 us | 6.56 us | - | - | 22.48 us | 162.84x | - | - | 0.13x |
| `complex neg ref` | 2.79 ns | 129.10 ns | 128.20 ns | - | - | 2.21 us | 46.26x | - | - | 0.06x |
| `complex div_real_ref` | 7.39 ns | 774.47 ns | 751.60 ns | - | - | 5.48 us | 104.80x | - | - | 0.14x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 13.09 us |
| `astro sin 160` | 14.35 us |
| `astro sin 192` | 14.54 us |
| `astro sin 256` | 16.20 us |
