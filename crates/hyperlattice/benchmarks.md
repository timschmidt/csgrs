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
| `sin 0.1` | 11.25 ns | 215.52 ns | 206.31 ns | 11.24 us | 788.21 ns | 1.92 us | 19.16x | 0.02x | 0.27x | 0.11x |
| `cos 0.1` | 12.02 ns | 215.80 ns | 211.21 ns | 10.66 us | 507.27 ns | 1.73 us | 17.95x | 0.02x | 0.43x | 0.12x |
| `sin 1.23456789` | 12.10 ns | 647.75 ns | 646.44 ns | 12.83 us | 809.42 ns | 1.87 us | 53.55x | 0.05x | 0.80x | 0.35x |
| `cos 1.23456789` | 12.15 ns | 563.85 ns | 547.50 ns | 10.85 us | 602.71 ns | 1.68 us | 46.40x | 0.05x | 0.94x | 0.34x |
| `sin 1e6` | 12.74 ns | 2.80 us | 2.74 us | 16.20 us | 1.08 us | 2.07 us | 220.18x | 0.17x | 2.59x | 1.35x |
| `cos 1e6` | 12.72 ns | 2.34 us | 2.34 us | 13.74 us | 821.82 ns | 1.85 us | 183.76x | 0.17x | 2.84x | 1.26x |
| `sin 1e30` | 67.80 ns | 2.97 us | 2.94 us | 18.80 us | 2.91 us | 3.68 us | 43.86x | 0.16x | 1.02x | 0.81x |
| `cos 1e30` | 70.86 ns | 2.51 us | 2.51 us | 15.75 us | 974.85 ns | 3.16 us | 35.49x | 0.16x | 2.58x | 0.79x |
| `sin pi_7` | 11.98 ns | 209.67 ns | 837.73 ns | 12.45 us | 750.52 ns | 1.97 us | 17.50x | 0.02x | 0.28x | 0.11x |
| `cos pi_7` | 12.03 ns | 211.42 ns | 1.87 us | 10.97 us | 544.80 ns | 1.77 us | 17.58x | 0.02x | 0.39x | 0.12x |
| `sin 1000pi_eps` | 11.92 ns | 3.21 us | 4.08 us | 16.32 us | 2.33 us | 2.92 us | 269.29x | 0.20x | 1.38x | 1.10x |
| `cos 1000pi_eps` | 12.64 ns | 2.83 us | 3.66 us | 13.89 us | 572.20 ns | 1.69 us | 224.35x | 0.20x | 4.95x | 1.68x |
| `asin 0.5` | 11.19 ns | 560.39 ns | 551.63 ns | 49.49 us | 3.03 us | 13.82 us | 50.08x | 0.01x | 0.19x | 0.04x |
| `acos 0.5` | 11.65 ns | 1.26 us | 1.26 us | 60.82 us | 2.95 us | 13.63 us | 107.78x | 0.02x | 0.43x | 0.09x |
| `atanh 0.5` | 14.94 ns | 1.52 us | 1.51 us | 34.90 us | 1.71 us | 13.09 us | 101.92x | 0.04x | 0.89x | 0.12x |
| `asin neg_0.999999` | 14.35 ns | 13.27 us | 7.80 us | 13.79 us | 2.53 us | 13.46 us | 925.13x | 0.96x | 5.26x | 0.99x |
| `acos neg_0.999999` | 15.65 ns | 17.04 us | 10.81 us | 18.47 us | 2.73 us | 13.39 us | 1088.62x | 0.92x | 6.23x | 1.27x |
| `atanh neg_0.999999` | 15.01 ns | 4.76 us | 3.53 us | 36.89 us | 1.65 us | 13.07 us | 317.15x | 0.13x | 2.88x | 0.36x |
| `asin 0.999999` | 14.14 ns | 15.06 us | 7.94 us | 13.81 us | 2.53 us | 13.22 us | 1065.76x | 1.09x | 5.96x | 1.14x |
| `acos 0.999999` | 14.58 ns | 18.02 us | 10.26 us | 18.46 us | 2.83 us | 13.11 us | 1236.10x | 0.98x | 6.36x | 1.37x |
| `atanh 0.999999` | 14.99 ns | 4.71 us | 3.54 us | 31.78 us | 1.67 us | 12.82 us | 314.30x | 0.15x | 2.83x | 0.37x |
| `asin 1e-12` | 9.57 ns | 24.21 us | 10.15 us | 7.93 us | 1.45 us | 15.55 us | 2530.43x | 3.05x | 16.73x | 1.56x |
| `acos 1e-12` | 10.35 ns | 26.27 us | 12.46 us | 9.84 us | 1.44 us | 15.52 us | 2538.39x | 2.67x | 18.29x | 1.69x |
| `atanh 1e-12` | 9.75 ns | 7.15 us | 3.46 us | 36.71 us | 171.89 ns | 20.45 us | 732.79x | 0.19x | 41.58x | 0.35x |
| `atan 0.5` | 15.51 ns | 681.07 ns | 694.47 ns | 36.13 us | 2.78 us | 17.84 us | 43.92x | 0.02x | 0.25x | 0.04x |
| `asinh 0.5` | 27.26 ns | 7.90 us | 7.70 us | 39.44 us | 1.61 us | 7.54 us | 289.88x | 0.20x | 4.92x | 1.05x |
| `atan neg_1e-12` | 14.57 ns | 820.98 ns | 742.40 ns | 1.59 us | 1.14 us | 15.62 us | 56.37x | 0.52x | 0.72x | 0.05x |
| `asinh neg_1e-12` | 16.34 ns | 10.43 us | 7.87 us | 41.88 us | 8.70 us | 12.21 us | 638.36x | 0.25x | 1.20x | 0.85x |
| `atan 1e6` | 15.76 ns | 631.71 ns | 652.15 ns | 2.88 us | 1.41 us | 18.19 us | 40.08x | 0.22x | 0.45x | 0.03x |
| `asinh 1e6` | 27.33 ns | 4.17 us | 4.12 us | 37.24 us | 1.69 us | 7.44 us | 152.65x | 0.11x | 2.47x | 0.56x |
| `atan neg_1e6` | 15.60 ns | 742.44 ns | 741.76 ns | 2.91 us | 1.45 us | 18.35 us | 47.59x | 0.26x | 0.51x | 0.04x |
| `asinh neg_1e6` | 27.91 ns | 7.17 us | 6.98 us | 37.09 us | 1.70 us | 7.20 us | 256.77x | 0.19x | 4.21x | 0.99x |
| `acosh 9` | 12.83 ns | 3.77 us | 3.66 us | 42.92 us | 1.67 us | 10.05 us | 293.54x | 0.09x | 2.26x | 0.37x |
| `acosh 1_plus_1e-12` | 13.42 ns | 5.92 us | 7.11 us | 42.65 us | 8.55 us | 11.66 us | 441.16x | 0.14x | 0.69x | 0.51x |
| `acosh 1e6` | 13.16 ns | 5.27 us | 5.01 us | 37.45 us | 1.66 us | 10.02 us | 400.30x | 0.14x | 3.18x | 0.53x |
| `acosh e` | 12.90 ns | 9.53 us | 24.54 us | 41.10 us | 1.68 us | 9.91 us | 738.88x | 0.23x | 5.68x | 0.96x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 61.27 ns | 54.82 ns | 24.96 ns | 15.76 ns | 0.95 ns | 126.52x | 2.45x | 3.89x | 64.68x |
| `one` | 0.48 ns | 71.57 ns | 61.44 ns | 41.80 ns | 30.99 ns | 32.28 ns | 148.12x | 1.71x | 2.31x | 2.22x |
| `e` | 0.49 ns | 75.63 ns | 68.24 ns | 54.24 ns | 1.08 us | 222.54 ns | 154.43x | 1.39x | 0.07x | 0.34x |
| `pi` | 0.49 ns | 58.86 ns | 54.76 ns | 44.20 ns | 50.72 ns | 225.21 ns | 120.42x | 1.33x | 1.16x | 0.26x |
| `tau` | 0.49 ns | 204.29 ns | 187.64 ns | 121.26 ns | 101.70 ns | 1.90 us | 419.88x | 1.68x | 2.01x | 0.11x |
| `add` | 5.41 ns | 437.62 ns | 404.53 ns | 52.25 ns | 42.42 ns | 1.31 us | 80.83x | 8.38x | 10.32x | 0.33x |
| `sub` | 5.44 ns | 704.90 ns | 472.21 ns | 56.85 ns | 45.80 ns | 2.48 us | 129.49x | 12.40x | 15.39x | 0.28x |
| `neg` | 5.27 ns | 68.64 ns | 64.43 ns | 24.17 ns | 20.28 ns | 1.09 us | 13.03x | 2.84x | 3.39x | 0.06x |
| `mul` | 5.59 ns | 482.97 ns | 355.43 ns | 59.25 ns | 44.67 ns | 1.56 us | 86.38x | 8.15x | 10.81x | 0.31x |
| `div` | 8.20 ns | 426.54 ns | 289.64 ns | 138.48 ns | 63.07 ns | 2.61 us | 52.04x | 3.08x | 6.76x | 0.16x |
| `reciprocal` | 8.86 ns | 122.11 ns | 112.36 ns | 160.21 ns | 59.09 ns | 1.57 us | 13.78x | 0.76x | 2.07x | 0.08x |
| `reciprocal checked` | 9.10 ns | 161.56 ns | 149.32 ns | 160.28 ns | 59.05 ns | 1.56 us | 17.76x | 1.01x | 2.74x | 0.10x |
| `reciprocal checked abort` | 28.16 ns | 258.52 ns | 236.19 ns | 160.37 ns | 59.34 ns | 1.56 us | 9.18x | 1.61x | 4.36x | 0.17x |
| `pow` | 31.56 ns | 13.47 us | 9.31 us | 54.97 us | 2.96 us | 2.38 us | 426.71x | 0.24x | 4.55x | 5.66x |
| `powi` | 6.17 ns | 2.65 us | 2.83 us | 292.93 ns | 85.97 ns | 1.58 us | 429.74x | 9.05x | 30.83x | 1.68x |
| `exp` | 10.36 ns | 1.87 us | 1.83 us | 14.36 us | 935.38 ns | 1.90 us | 180.31x | 0.13x | 2.00x | 0.98x |
| `ln` | 11.18 ns | 1.71 us | 1.57 us | 30.10 us | 1.36 us | 1.85 us | 153.22x | 0.06x | 1.26x | 0.93x |
| `log10` | 13.96 ns | 2.94 us | 2.38 us | 35.43 us | 2.84 us | 6.74 us | 210.44x | 0.08x | 1.03x | 0.44x |
| `log10 abort` | 17.86 ns | 2.96 us | 2.37 us | 35.45 us | 2.89 us | 6.81 us | 165.89x | 0.08x | 1.03x | 0.44x |
| `sqrt` | 20.89 ns | 1.89 us | 1.38 us | 5.13 us | 96.62 ns | 1.48 us | 90.63x | 0.37x | 19.59x | 1.28x |
| `sin` | 15.29 ns | 1.63 us | 1.69 us | 14.23 us | 1.26 us | 2.30 us | 106.52x | 0.11x | 1.29x | 0.71x |
| `cos` | 18.83 ns | 1.46 us | 1.51 us | 12.23 us | 643.47 ns | 1.82 us | 77.30x | 0.12x | 2.26x | 0.80x |
| `tan` | 25.28 ns | 1.50 us | 1.56 us | 29.71 us | 1.61 us | 6.79 us | 59.19x | 0.05x | 0.93x | 0.22x |
| `sinh` | 18.88 ns | 3.51 us | 3.54 us | 3.32 us | 1.14 us | 10.98 us | 185.76x | 1.06x | 3.07x | 0.32x |
| `cosh` | 18.61 ns | 3.49 us | 3.41 us | 8.19 us | 1.08 us | 9.69 us | 187.42x | 0.43x | 3.22x | 0.36x |
| `tanh` | 23.96 ns | 4.89 us | 4.85 us | 3.40 us | 1.25 us | 23.55 us | 204.13x | 1.44x | 3.91x | 0.21x |
| `asin` | 13.82 ns | 13.58 us | 10.08 us | 21.55 us | 2.44 us | 14.19 us | 983.04x | 0.63x | 5.57x | 0.96x |
| `asin abort` | 17.79 ns | 13.90 us | 10.02 us | 21.72 us | 2.46 us | 14.17 us | 781.52x | 0.64x | 5.65x | 0.98x |
| `acos` | 14.49 ns | 16.13 us | 12.45 us | 26.84 us | 2.54 us | 14.28 us | 1112.99x | 0.60x | 6.35x | 1.13x |
| `acos abort` | 18.35 ns | 16.04 us | 13.03 us | 26.76 us | 2.57 us | 14.30 us | 874.10x | 0.60x | 6.23x | 1.12x |
| `atan` | 15.40 ns | 1.24 us | 1.69 us | 18.42 us | 2.32 us | 19.10 us | 80.73x | 0.07x | 0.54x | 0.07x |
| `atan abort` | 21.97 ns | 1.28 us | 1.69 us | 18.51 us | 2.30 us | 19.10 us | 58.35x | 0.07x | 0.56x | 0.07x |
| `asinh` | 49.79 ns | 8.69 us | 10.11 us | 40.25 us | 1.68 us | 7.66 us | 174.54x | 0.22x | 5.18x | 1.13x |
| `asinh abort` | 35.24 ns | 8.93 us | 10.23 us | 39.59 us | 1.69 us | 7.63 us | 253.34x | 0.23x | 5.29x | 1.17x |
| `acosh` | 12.60 ns | 6.30 us | 7.48 us | 40.53 us | 3.37 us | 10.58 us | 499.64x | 0.16x | 1.87x | 0.60x |
| `acosh abort` | 17.39 ns | 6.25 us | 7.63 us | 40.50 us | 3.41 us | 10.65 us | 359.39x | 0.15x | 1.83x | 0.59x |
| `atanh` | 14.06 ns | 4.77 us | 3.96 us | 35.34 us | 1.33 us | 15.21 us | 338.96x | 0.13x | 3.58x | 0.31x |
| `atanh abort` | 17.47 ns | 4.81 us | 3.87 us | 35.28 us | 1.32 us | 15.12 us | 275.42x | 0.14x | 3.63x | 0.32x |
| `zero status` | 1.23 ns | 38.55 ns | 34.27 ns | 1.01 ns | 6.84 ns | 7.94 ns | 31.28x | 38.22x | 5.63x | 4.85x |
| `zero status abort` | 3.46 ns | 96.69 ns | 91.30 ns | 1.00 ns | 6.82 ns | 8.04 ns | 27.93x | 96.93x | 14.18x | 12.03x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 118.83 ns | 122.40 ns | 48.01 ns | - | 1.90 ns | 127.64x | 2.47x | - | 62.41x |
| `one` | 5.09 ns | 122.08 ns | 123.52 ns | 64.31 ns | - | 31.96 ns | 23.99x | 1.90x | - | 3.82x |
| `i` | 0.94 ns | 129.25 ns | 131.36 ns | 64.41 ns | - | 34.05 ns | 138.16x | 2.01x | - | 3.80x |
| `free i` | 0.93 ns | 129.50 ns | 131.98 ns | 64.28 ns | - | 33.90 ns | 138.86x | 2.01x | - | 3.82x |
| `conjugate` | 2.12 ns | 129.81 ns | 132.71 ns | 41.77 ns | - | 1.15 us | 61.20x | 3.11x | - | 0.11x |
| `norm squared` | 5.79 ns | 1.58 us | 1.63 us | 155.25 ns | - | 4.62 us | 272.19x | 10.15x | - | 0.34x |
| `reciprocal` | 18.88 ns | 3.10 us | 3.07 us | 457.40 ns | - | 11.33 us | 164.08x | 6.77x | - | 0.27x |
| `reciprocal checked` | 14.98 ns | 3.19 us | 3.16 us | 445.20 ns | - | 11.26 us | 212.78x | 7.16x | - | 0.28x |
| `powi` | 17.99 ns | 12.36 us | 15.22 us | 1.48 us | - | 45.90 us | 686.83x | 8.33x | - | 0.27x |
| `powi checked` | 18.14 ns | 12.21 us | 15.29 us | 1.49 us | - | 45.82 us | 673.26x | 8.18x | - | 0.27x |
| `div checked` | 20.20 ns | 6.37 us | 6.82 us | 796.94 ns | - | 22.70 us | 315.30x | 7.99x | - | 0.28x |
| `div real checked` | 9.35 ns | 864.43 ns | 823.03 ns | 269.62 ns | - | 5.50 us | 92.49x | 3.21x | - | 0.16x |
| `from scalar` | 1.42 ns | 126.36 ns | 121.28 ns | 46.53 ns | - | 10.51 ns | 89.05x | 2.72x | - | 12.03x |
| `add` | 6.11 ns | 877.38 ns | 916.94 ns | 107.63 ns | - | 2.65 us | 143.70x | 8.15x | - | 0.33x |
| `sub` | 6.06 ns | 1.00 us | 891.58 ns | 119.18 ns | - | 5.05 us | 165.42x | 8.41x | - | 0.20x |
| `neg` | 2.64 ns | 133.70 ns | 132.07 ns | 43.58 ns | - | 2.20 us | 50.64x | 3.07x | - | 0.06x |
| `mul` | 7.55 ns | 3.27 us | 3.58 us | 304.77 ns | - | 10.45 us | 433.59x | 10.74x | - | 0.31x |
| `div` | 17.73 ns | 6.34 us | 6.78 us | 813.98 ns | - | 22.54 us | 357.37x | 7.79x | - | 0.28x |
| `div real` | 10.00 ns | 785.03 ns | 764.40 ns | 268.25 ns | - | 5.46 us | 78.54x | 2.93x | - | 0.14x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.86 ns | 3.58 us | 6.25 us | 317.00 ns | 252.04 ns | 7.44 us | 521.37x | 11.28x | 14.19x | 0.48x |
| `vec3 magnitude` | 9.68 ns | 7.43 us | 10.19 us | 5.43 us | 352.47 ns | 9.09 us | 767.39x | 1.37x | 21.08x | 0.82x |
| `vec3 normalize` | 25.23 ns | 12.98 us | 12.69 us | 6.10 us | 591.32 ns | 17.30 us | 514.36x | 2.13x | 21.95x | 0.75x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.09 ns | 1.17 us | 2.14 us | 73.75 ns | 60.54 ns | 724.59 ns | 379.61x | 15.90x | 19.37x | 1.62x |
| `vec3 zero` | 1.43 ns | 236.78 ns | 239.80 ns | 59.55 ns | 31.09 ns | 2.90 ns | 165.60x | 3.98x | 7.62x | 81.67x |
| `vec3 dot abort` | 28.28 ns | 3.87 us | 4.63 us | 260.42 ns | 205.59 ns | 7.54 us | 136.99x | 14.87x | 18.84x | 0.51x |
| `vec3 magnitude abort` | 39.43 ns | 7.87 us | 7.21 us | 5.40 us | 324.78 ns | 9.19 us | 199.59x | 1.46x | 24.23x | 0.86x |
| `vec3 normalize checked` | 26.10 ns | 12.66 us | 10.25 us | 5.99 us | 551.66 ns | 17.97 us | 484.83x | 2.11x | 22.94x | 0.70x |
| `vec3 normalize checked abort` | 55.42 ns | 13.73 us | 10.41 us | 6.08 us | 561.47 ns | 18.30 us | 247.73x | 2.26x | 24.45x | 0.75x |
| `vec3 div scalar checked` | 16.19 ns | 1.83 us | 1.69 us | 397.49 ns | - | - | 113.27x | 4.61x | - | - |
| `vec3 div scalar checked abort` | 18.15 ns | 1.82 us | 1.69 us | 401.55 ns | - | - | 100.24x | 4.53x | - | - |
| `vec3 add` | 6.87 ns | 2.07 us | 1.86 us | 153.30 ns | 126.57 ns | 4.09 us | 301.24x | 13.49x | 16.34x | 0.51x |
| `vec3 add scalar` | 6.59 ns | 1.84 us | 1.71 us | 154.11 ns | 134.73 ns | 4.05 us | 278.73x | 11.92x | 13.64x | 0.45x |
| `vec3 sub` | 6.80 ns | 2.10 us | 1.95 us | 169.08 ns | 136.81 ns | 7.69 us | 308.15x | 12.39x | 15.32x | 0.27x |
| `vec3 sub scalar` | 6.44 ns | 1.62 us | 1.56 us | 147.73 ns | 125.35 ns | 7.35 us | 252.11x | 10.98x | 12.95x | 0.22x |
| `vec3 neg` | 3.91 ns | 240.32 ns | 240.80 ns | 57.19 ns | 47.58 ns | 3.34 us | 61.46x | 4.20x | 5.05x | 0.07x |
| `vec3 mul scalar` | 7.10 ns | 2.00 us | 2.13 us | 167.04 ns | 121.38 ns | 4.63 us | 281.12x | 11.95x | 16.45x | 0.43x |
| `vec3 div scalar` | 10.45 ns | 1.80 us | 1.70 us | 407.44 ns | 170.94 ns | 8.19 us | 172.15x | 4.42x | 10.52x | 0.22x |
| `vec4 dot` | 7.41 ns | 3.61 us | 3.30 us | 439.14 ns | 325.28 ns | 10.35 us | 486.88x | 8.21x | 11.09x | 0.35x |
| `vec4 magnitude` | 13.21 ns | 7.14 us | 5.93 us | 5.64 us | 416.56 ns | 11.96 us | 540.52x | 1.27x | 17.14x | 0.60x |
| `vec4 normalize` | 36.28 ns | 12.61 us | 8.46 us | 6.28 us | 739.09 ns | 24.19 us | 347.40x | 2.01x | 17.05x | 0.52x |
| `vec4 add` | 8.16 ns | 2.35 us | 2.08 us | 212.52 ns | 172.52 ns | 5.57 us | 287.56x | 11.04x | 13.60x | 0.42x |
| `vec4 add scalar` | 7.11 ns | 2.03 us | 1.78 us | 217.50 ns | 176.52 ns | 5.46 us | 285.37x | 9.33x | 11.49x | 0.37x |
| `vec4 sub` | 5.24 ns | 2.24 us | 1.96 us | 216.34 ns | 175.43 ns | 10.47 us | 426.93x | 10.34x | 12.75x | 0.21x |
| `vec4 sub scalar` | 4.56 ns | 1.77 us | 1.54 us | 206.28 ns | 168.57 ns | 9.96 us | 388.92x | 8.60x | 10.53x | 0.18x |
| `vec4 neg` | 5.07 ns | 304.91 ns | 309.48 ns | 78.17 ns | 66.77 ns | 4.31 us | 60.16x | 3.90x | 4.57x | 0.07x |
| `vec4 mul scalar` | 7.45 ns | 2.26 us | 2.22 us | 230.08 ns | 162.50 ns | 6.04 us | 302.72x | 9.81x | 13.88x | 0.37x |
| `vec4 div scalar` | 14.21 ns | 2.18 us | 1.86 us | 531.53 ns | 235.28 ns | 10.56 us | 153.58x | 4.11x | 9.28x | 0.21x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 11.77 ns | 8.53 us | 3.72 us | 953.53 ns | 871.44 ns | 23.66 us | 724.49x | 8.94x | 9.79x | 0.36x |
| `mat3 inverse` | 90.93 ns | 32.57 us | 12.13 us | 3.14 us | 2.51 us | 85.24 us | 358.18x | 10.36x | 12.97x | 0.38x |
| `mat3 mul mat3` | 74.63 ns | 24.14 us | 11.19 us | 2.84 us | 2.43 us | 62.73 us | 323.41x | 8.50x | 9.93x | 0.38x |
| `mat3 transform vec3` | 16.32 ns | 10.93 us | 11.47 us | 1.06 us | 897.12 ns | 20.79 us | 670.10x | 10.33x | 12.19x | 0.53x |
| `mat4 determinant` | 47.15 ns | 16.43 us | 5.21 us | 4.69 us | 4.29 us | 99.90 us | 348.48x | 3.51x | 3.83x | 0.16x |
| `mat4 inverse` | 161.17 ns | 73.87 us | 19.32 us | 11.56 us | 9.01 us | 354.28 us | 458.34x | 6.39x | 8.20x | 0.21x |
| `mat4 mul mat4` | 117.86 ns | 32.40 us | 11.45 us | 6.65 us | 5.51 us | 150.84 us | 274.93x | 4.88x | 5.89x | 0.21x |
| `mat4 transform vec4` | 25.90 ns | 13.63 us | 3.77 us | 1.99 us | 1.74 us | 36.91 us | 526.36x | 6.83x | 7.82x | 0.37x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.13 ns | 2.88 us | 5.91 us | 217.90 ns | 241.52 ns | 2.08 us | 82.02x | 13.22x | 11.93x | 1.38x |
| `mat3 zero` | 17.32 ns | 892.94 ns | 883.57 ns | 277.11 ns | 212.62 ns | 11.51 ns | 51.57x | 3.22x | 4.20x | 77.58x |
| `mat3 identity` | 10.31 ns | 1.03 us | 1.02 us | 340.80 ns | 251.62 ns | 155.03 ns | 99.82x | 3.02x | 4.09x | 6.64x |
| `mat3 transpose` | 9.38 ns | 987.63 ns | 955.39 ns | 239.60 ns | 219.88 ns | 126.27 ns | 105.33x | 4.12x | 4.49x | 7.82x |
| `mat3 reciprocal` | 91.03 ns | 33.01 us | 37.88 us | 2.94 us | 2.39 us | 88.38 us | 362.68x | 11.23x | 13.81x | 0.37x |
| `mat3 reciprocal checked` | 112.39 ns | 32.89 us | 37.80 us | 2.94 us | 2.35 us | 87.15 us | 292.69x | 11.21x | 14.01x | 0.38x |
| `mat3 inverse checked` | 114.91 ns | 32.42 us | 37.78 us | 2.99 us | 2.32 us | 85.54 us | 282.11x | 10.83x | 14.00x | 0.38x |
| `mat3 inverse checked abort` | 120.60 ns | 32.08 us | 37.81 us | 2.95 us | 2.34 us | 86.48 us | 266.01x | 10.88x | 13.71x | 0.37x |
| `mat3 powi` | 151.61 ns | 68.89 us | 95.64 us | 6.92 us | 6.42 us | 154.89 us | 454.39x | 9.95x | 10.73x | 0.44x |
| `mat3 powi checked` | 150.70 ns | 70.12 us | 99.28 us | 6.98 us | 6.42 us | 155.50 us | 465.29x | 10.05x | 10.92x | 0.45x |
| `mat3 powi checked abort` | 150.31 ns | 68.71 us | 95.60 us | 6.85 us | 6.52 us | 155.59 us | 457.16x | 10.03x | 10.55x | 0.44x |
| `mat3 div scalar checked` | 25.57 ns | 5.54 us | 5.71 us | 1.60 us | 832.02 ns | 22.82 us | 216.74x | 3.46x | 6.66x | 0.24x |
| `mat3 div scalar checked abort` | 31.27 ns | 5.77 us | 5.71 us | 1.58 us | 815.60 ns | 23.29 us | 184.44x | 3.66x | 7.07x | 0.25x |
| `mat3 div matrix checked` | 151.04 ns | 71.72 us | 67.12 us | 5.65 us | 4.58 us | 169.66 us | 474.82x | 12.68x | 15.68x | 0.42x |
| `mat3 div matrix checked abort` | 165.71 ns | 70.92 us | 67.46 us | 5.50 us | 4.54 us | 169.68 us | 427.96x | 12.90x | 15.64x | 0.42x |
| `mat3 add` | 14.69 ns | 5.08 us | 5.80 us | 555.26 ns | 501.38 ns | 11.85 us | 345.46x | 9.14x | 10.12x | 0.43x |
| `mat3 add scalar` | 11.05 ns | 5.26 us | 6.16 us | 836.61 ns | 709.66 ns | 12.49 us | 476.21x | 6.29x | 7.42x | 0.42x |
| `mat3 sub` | 14.12 ns | 5.24 us | 5.71 us | 568.31 ns | 518.61 ns | 21.94 us | 371.02x | 9.22x | 10.10x | 0.24x |
| `mat3 sub scalar` | 11.42 ns | 5.92 us | 5.94 us | 832.08 ns | 715.48 ns | 22.02 us | 518.34x | 7.12x | 8.28x | 0.27x |
| `mat3 neg` | 11.23 ns | 992.21 ns | 958.29 ns | 511.63 ns | 466.41 ns | 8.79 us | 88.39x | 1.94x | 2.13x | 0.11x |
| `mat3 mul scalar` | 12.66 ns | 5.36 us | 5.99 us | 813.75 ns | 680.99 ns | 12.60 us | 423.63x | 6.59x | 7.87x | 0.43x |
| `mat3 div scalar` | 25.21 ns | 5.63 us | 5.56 us | 1.60 us | 834.50 ns | 22.76 us | 223.33x | 3.51x | 6.75x | 0.25x |
| `mat3 div matrix` | 146.49 ns | 72.76 us | 66.38 us | 5.53 us | 4.52 us | 167.25 us | 496.71x | 13.15x | 16.09x | 0.44x |
| `mat3 bitxor` | 147.65 ns | 70.70 us | 95.41 us | 6.80 us | 6.48 us | 156.34 us | 478.85x | 10.39x | 10.92x | 0.45x |
| `mat4 zero` | 13.33 ns | 1.40 us | 1.41 us | 512.70 ns | 365.65 ns | 14.30 ns | 105.26x | 2.74x | 3.84x | 98.14x |
| `mat4 identity` | 10.44 ns | 1.59 us | 1.59 us | 565.94 ns | 429.55 ns | 234.60 ns | 152.37x | 2.81x | 3.70x | 6.78x |
| `mat4 transpose` | 9.91 ns | 1.68 us | 1.68 us | 449.84 ns | 373.01 ns | 178.29 ns | 169.68x | 3.74x | 4.51x | 9.44x |
| `mat4 reciprocal` | 187.38 ns | 73.44 us | 73.34 us | 11.42 us | 8.74 us | 351.98 us | 391.93x | 6.43x | 8.40x | 0.21x |
| `mat4 reciprocal checked` | 180.38 ns | 74.96 us | 73.88 us | 11.77 us | 8.72 us | 349.57 us | 415.55x | 6.37x | 8.60x | 0.21x |
| `mat4 powi` | 243.45 ns | 103.49 us | 120.36 us | 16.30 us | 14.09 us | 360.34 us | 425.12x | 6.35x | 7.35x | 0.29x |
| `mat4 powi checked` | 242.35 ns | 103.81 us | 119.59 us | 16.69 us | 14.16 us | 362.43 us | 428.33x | 6.22x | 7.33x | 0.29x |
| `mat4 add` | 51.53 ns | 6.46 us | 6.75 us | 982.08 ns | 872.22 ns | 19.80 us | 125.29x | 6.57x | 7.40x | 0.33x |
| `mat4 add scalar` | 16.46 ns | 7.36 us | 8.13 us | 1.59 us | 1.23 us | 21.20 us | 447.16x | 4.62x | 5.98x | 0.35x |
| `mat4 sub` | 37.84 ns | 6.84 us | 6.88 us | 990.66 ns | 898.22 ns | 36.80 us | 180.82x | 6.91x | 7.62x | 0.19x |
| `mat4 sub scalar` | 15.21 ns | 8.82 us | 7.93 us | 1.56 us | 1.22 us | 38.14 us | 579.63x | 5.67x | 7.25x | 0.23x |
| `mat4 neg` | 14.07 ns | 1.72 us | 1.74 us | 1.09 us | 760.73 ns | 14.55 us | 122.47x | 1.58x | 2.27x | 0.12x |
| `mat4 mul scalar` | 21.25 ns | 7.58 us | 7.90 us | 1.53 us | 1.13 us | 21.08 us | 356.51x | 4.94x | 6.68x | 0.36x |
| `mat4 div scalar` | 33.56 ns | 8.76 us | 8.04 us | 2.85 us | 1.42 us | 39.90 us | 260.86x | 3.07x | 6.16x | 0.22x |
| `mat4 div matrix` | 219.33 ns | 145.34 us | 113.70 us | 18.37 us | 13.97 us | 553.01 us | 662.64x | 7.91x | 10.40x | 0.26x |
| `mat4 bitxor` | 244.71 ns | 105.52 us | 119.60 us | 16.40 us | 14.12 us | 358.37 us | 431.20x | 6.43x | 7.47x | 0.29x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 9.18 ns | 661.48 ns | 655.18 ns | - | - | 1.32 us | 72.07x | - | - | 0.50x |
| `scalar add ref_owned` | 5.60 ns | 663.42 ns | 659.64 ns | - | - | 1.32 us | 118.39x | - | - | 0.50x |
| `scalar add refs` | 5.41 ns | 667.49 ns | 646.20 ns | - | - | 1.31 us | 123.39x | - | - | 0.51x |
| `scalar sub owned_ref` | 9.10 ns | 942.39 ns | 730.17 ns | - | - | 2.48 us | 103.56x | - | - | 0.38x |
| `scalar sub ref_owned` | 5.75 ns | 1.01 us | 795.28 ns | - | - | 2.53 us | 175.70x | - | - | 0.40x |
| `scalar sub refs` | 5.62 ns | 960.45 ns | 726.43 ns | - | - | 2.50 us | 171.04x | - | - | 0.38x |
| `scalar mul owned_ref` | 9.84 ns | 744.84 ns | 601.45 ns | - | - | 1.57 us | 75.72x | - | - | 0.47x |
| `scalar mul ref_owned` | 6.27 ns | 731.61 ns | 606.34 ns | - | - | 1.58 us | 116.74x | - | - | 0.46x |
| `scalar mul refs` | 6.35 ns | 727.56 ns | 604.24 ns | - | - | 1.58 us | 114.52x | - | - | 0.46x |
| `scalar div owned_ref` | 15.73 ns | 622.13 ns | 508.48 ns | - | - | 2.63 us | 39.55x | - | - | 0.24x |
| `scalar div ref_owned` | 15.96 ns | 700.56 ns | 567.58 ns | - | - | 2.62 us | 43.89x | - | - | 0.27x |
| `scalar div refs` | 7.05 ns | 629.04 ns | 494.40 ns | - | - | 2.67 us | 89.17x | - | - | 0.24x |
| `vec3 add refs` | 6.38 ns | 2.01 us | 1.91 us | - | - | 4.13 us | 315.60x | - | - | 0.49x |
| `vec3 sub refs` | 6.40 ns | 2.09 us | 1.97 us | - | - | 7.45 us | 326.59x | - | - | 0.28x |
| `vec3 neg ref` | 3.36 ns | 517.92 ns | 534.53 ns | - | - | 3.21 us | 154.25x | - | - | 0.16x |
| `vec3 add_scalar_ref` | 6.69 ns | 2.11 us | 2.07 us | - | - | 3.88 us | 315.66x | - | - | 0.54x |
| `vec3 sub_scalar_ref` | 6.58 ns | 1.88 us | 1.89 us | - | - | 7.20 us | 285.60x | - | - | 0.26x |
| `vec3 mul_scalar_ref` | 7.23 ns | 2.27 us | 2.35 us | - | - | 4.64 us | 314.27x | - | - | 0.49x |
| `vec3 div_scalar_ref` | 9.99 ns | 2.21 us | 2.01 us | - | - | 7.88 us | 220.97x | - | - | 0.28x |
| `vec4 add refs` | 6.51 ns | 2.23 us | 1.92 us | - | - | 5.42 us | 341.91x | - | - | 0.41x |
| `vec4 sub refs` | 3.10 ns | 2.13 us | 1.78 us | - | - | 10.11 us | 684.78x | - | - | 0.21x |
| `vec4 neg ref` | 4.27 ns | 666.30 ns | 650.97 ns | - | - | 4.19 us | 156.17x | - | - | 0.16x |
| `vec4 add_scalar_ref` | 7.13 ns | 2.28 us | 1.99 us | - | - | 5.17 us | 319.57x | - | - | 0.44x |
| `vec4 sub_scalar_ref` | 4.38 ns | 2.04 us | 1.75 us | - | - | 9.59 us | 466.28x | - | - | 0.21x |
| `vec4 mul_scalar_ref` | 7.73 ns | 2.50 us | 2.38 us | - | - | 6.05 us | 323.47x | - | - | 0.41x |
| `vec4 div_scalar_ref` | 13.70 ns | 2.41 us | 2.09 us | - | - | 10.35 us | 175.68x | - | - | 0.23x |
| `mat3 add refs` | 11.41 ns | 4.17 us | 4.73 us | - | - | 12.06 us | 365.60x | - | - | 0.35x |
| `mat3 sub refs` | 10.83 ns | 4.26 us | 4.64 us | - | - | 21.78 us | 393.14x | - | - | 0.20x |
| `mat3 mul refs` | 55.00 ns | 23.94 us | 32.99 us | - | - | 64.41 us | 435.33x | - | - | 0.37x |
| `mat3 div refs` | 146.98 ns | 71.25 us | 68.20 us | - | - | 164.32 us | 484.78x | - | - | 0.43x |
| `mat3 neg ref` | 10.18 ns | 1.01 us | 1.03 us | - | - | 8.99 us | 98.86x | - | - | 0.11x |
| `mat3 add_scalar_ref` | 10.57 ns | 5.30 us | 6.28 us | - | - | 12.31 us | 501.60x | - | - | 0.43x |
| `mat3 sub_scalar_ref` | 12.88 ns | 6.03 us | 6.10 us | - | - | 22.34 us | 467.81x | - | - | 0.27x |
| `mat3 mul_scalar_ref` | 11.79 ns | 5.54 us | 6.23 us | - | - | 12.64 us | 470.16x | - | - | 0.44x |
| `mat3 div_scalar_ref` | 23.93 ns | 5.94 us | 5.95 us | - | - | 22.95 us | 248.05x | - | - | 0.26x |
| `mat4 add refs` | 17.25 ns | 4.26 us | 4.61 us | - | - | 19.80 us | 246.93x | - | - | 0.22x |
| `mat4 sub refs` | 16.86 ns | 4.59 us | 4.73 us | - | - | 36.89 us | 271.99x | - | - | 0.12x |
| `mat4 mul refs` | 104.14 ns | 34.27 us | 40.98 us | - | - | 145.38 us | 329.09x | - | - | 0.24x |
| `mat4 div refs` | 218.55 ns | 141.49 us | 110.74 us | - | - | 540.47 us | 647.41x | - | - | 0.26x |
| `mat4 neg ref` | 12.21 ns | 1.52 us | 1.63 us | - | - | 14.58 us | 124.58x | - | - | 0.10x |
| `mat4 add_scalar_ref` | 14.38 ns | 7.87 us | 8.45 us | - | - | 20.88 us | 547.10x | - | - | 0.38x |
| `mat4 sub_scalar_ref` | 15.40 ns | 9.10 us | 8.44 us | - | - | 38.12 us | 590.71x | - | - | 0.24x |
| `mat4 mul_scalar_ref` | 49.35 ns | 7.90 us | 8.26 us | - | - | 20.77 us | 160.05x | - | - | 0.38x |
| `mat4 div_scalar_ref` | 29.61 ns | 9.26 us | 8.43 us | - | - | 39.04 us | 312.82x | - | - | 0.24x |
| `mat3 transform_vec refs` | 15.73 ns | 11.33 us | 12.55 us | - | - | 20.78 us | 720.17x | - | - | 0.55x |
| `mat4 transform_vec refs` | 24.87 ns | 13.63 us | 12.54 us | - | - | 36.59 us | 548.03x | - | - | 0.37x |
| `complex add refs` | 7.79 ns | 1.07 us | 1.07 us | - | - | 2.65 us | 137.99x | - | - | 0.41x |
| `complex sub refs` | 7.96 ns | 1.20 us | 1.08 us | - | - | 4.90 us | 150.22x | - | - | 0.24x |
| `complex mul refs` | 8.05 ns | 3.49 us | 3.67 us | - | - | 10.28 us | 433.50x | - | - | 0.34x |
| `complex div refs` | 17.62 ns | 6.55 us | 6.93 us | - | - | 22.38 us | 372.03x | - | - | 0.29x |
| `complex neg ref` | 2.38 ns | 400.13 ns | 391.90 ns | - | - | 2.19 us | 168.24x | - | - | 0.18x |
| `complex div_real_ref` | 10.05 ns | 1.17 us | 1.10 us | - | - | 5.33 us | 116.14x | - | - | 0.22x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.50 us |
| `astro sin 160` | 14.15 us |
| `astro sin 192` | 14.21 us |
| `astro sin 256` | 16.08 us |
