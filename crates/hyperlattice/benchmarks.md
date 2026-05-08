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
| `sin 0.1` | 11.11 ns | 193.39 ns | 191.36 ns | 11.32 us | 775.18 ns | 1.88 us | 17.40x | 0.02x | 0.25x | 0.10x |
| `cos 0.1` | 12.03 ns | 191.08 ns | 188.88 ns | 11.12 us | 505.02 ns | 1.68 us | 15.89x | 0.02x | 0.38x | 0.11x |
| `sin 1.23456789` | 11.98 ns | 250.71 ns | 233.58 ns | 13.07 us | 826.38 ns | 1.85 us | 20.93x | 0.02x | 0.30x | 0.14x |
| `cos 1.23456789` | 12.35 ns | 238.57 ns | 226.70 ns | 11.45 us | 603.06 ns | 1.68 us | 19.31x | 0.02x | 0.40x | 0.14x |
| `sin 1e6` | 13.09 ns | 101.90 ns | 103.55 ns | 16.03 us | 1.09 us | 2.04 us | 7.79x | 0.01x | 0.09x | 0.05x |
| `cos 1e6` | 12.65 ns | 103.68 ns | 102.74 ns | 13.94 us | 832.23 ns | 1.85 us | 8.20x | 0.01x | 0.12x | 0.06x |
| `sin 1e30` | 67.62 ns | 101.49 ns | 103.34 ns | 18.94 us | 2.89 us | 3.64 us | 1.50x | 0.01x | 0.04x | 0.03x |
| `cos 1e30` | 69.55 ns | 100.94 ns | 100.19 ns | 16.41 us | 990.54 ns | 3.12 us | 1.45x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.77 ns | 192.78 ns | 499.31 ns | 12.34 us | 744.09 ns | 1.88 us | 16.38x | 0.02x | 0.26x | 0.10x |
| `cos pi_7` | 11.62 ns | 189.78 ns | 904.90 ns | 11.43 us | 538.53 ns | 1.71 us | 16.33x | 0.02x | 0.35x | 0.11x |
| `sin 1000pi_eps` | 12.16 ns | 101.13 ns | 4.06 us | 16.48 us | 2.25 us | 2.86 us | 8.32x | 0.01x | 0.05x | 0.04x |
| `cos 1000pi_eps` | 12.56 ns | 100.66 ns | 4.12 us | 13.88 us | 568.88 ns | 1.69 us | 8.01x | 0.01x | 0.18x | 0.06x |
| `asin 0.5` | 11.14 ns | 521.24 ns | 519.01 ns | 50.41 us | 3.02 us | 13.32 us | 46.77x | 0.01x | 0.17x | 0.04x |
| `acos 0.5` | 11.45 ns | 1.11 us | 1.10 us | 60.85 us | 3.06 us | 13.17 us | 96.54x | 0.02x | 0.36x | 0.08x |
| `atanh 0.5` | 14.88 ns | 1.45 us | 1.42 us | 35.73 us | 1.65 us | 13.08 us | 97.19x | 0.04x | 0.88x | 0.11x |
| `asin neg_0.999999` | 14.87 ns | 869.68 ns | 1.50 us | 14.27 us | 2.59 us | 13.17 us | 58.47x | 0.06x | 0.34x | 0.07x |
| `acos neg_0.999999` | 15.58 ns | 977.90 ns | 2.03 us | 18.80 us | 2.81 us | 13.45 us | 62.78x | 0.05x | 0.35x | 0.07x |
| `atanh neg_0.999999` | 14.62 ns | 432.69 ns | 609.53 ns | 37.43 us | 1.60 us | 13.22 us | 29.59x | 0.01x | 0.27x | 0.03x |
| `asin 0.999999` | 13.87 ns | 858.69 ns | 1.70 us | 14.13 us | 2.60 us | 13.13 us | 61.91x | 0.06x | 0.33x | 0.07x |
| `acos 0.999999` | 14.12 ns | 727.78 ns | 1.84 us | 18.86 us | 2.81 us | 12.97 us | 51.55x | 0.04x | 0.26x | 0.06x |
| `atanh 0.999999` | 14.51 ns | 470.30 ns | 878.74 ns | 32.35 us | 1.59 us | 12.82 us | 32.42x | 0.01x | 0.30x | 0.04x |
| `asin 1e-12` | 9.63 ns | 481.57 ns | 1.12 us | 8.14 us | 1.47 us | 15.44 us | 50.01x | 0.06x | 0.33x | 0.03x |
| `acos 1e-12` | 10.02 ns | 807.30 ns | 1.88 us | 9.68 us | 1.48 us | 15.33 us | 80.53x | 0.08x | 0.54x | 0.05x |
| `atanh 1e-12` | 9.95 ns | 486.57 ns | 867.27 ns | 37.15 us | 171.16 ns | 20.34 us | 48.92x | 0.01x | 2.84x | 0.02x |
| `atan 0.5` | 15.52 ns | 449.87 ns | 436.80 ns | 35.55 us | 2.79 us | 18.22 us | 28.99x | 0.01x | 0.16x | 0.02x |
| `asinh 0.5` | 27.43 ns | 453.76 ns | 450.30 ns | 40.28 us | 1.66 us | 7.64 us | 16.54x | 0.01x | 0.27x | 0.06x |
| `atan neg_1e-12` | 14.61 ns | 415.49 ns | 584.70 ns | 1.61 us | 1.20 us | 15.50 us | 28.44x | 0.26x | 0.35x | 0.03x |
| `asinh neg_1e-12` | 15.95 ns | 477.36 ns | 375.82 ns | 42.39 us | 8.92 us | 12.36 us | 29.93x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 15.46 ns | 390.62 ns | 375.61 ns | 2.92 us | 1.50 us | 19.13 us | 25.26x | 0.13x | 0.26x | 0.02x |
| `asinh 1e6` | 27.36 ns | 326.05 ns | 318.17 ns | 38.00 us | 1.72 us | 7.39 us | 11.92x | 0.01x | 0.19x | 0.04x |
| `atan neg_1e6` | 15.73 ns | 509.65 ns | 489.84 ns | 2.92 us | 1.50 us | 18.56 us | 32.39x | 0.17x | 0.34x | 0.03x |
| `asinh neg_1e6` | 27.38 ns | 346.63 ns | 344.47 ns | 37.45 us | 1.70 us | 7.33 us | 12.66x | 0.01x | 0.20x | 0.05x |
| `acosh 9` | 12.85 ns | 190.44 ns | 187.87 ns | 42.68 us | 1.66 us | 10.17 us | 14.82x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 13.71 ns | 542.04 ns | 1.47 us | 41.39 us | 8.53 us | 11.91 us | 39.53x | 0.01x | 0.06x | 0.05x |
| `acosh 1e6` | 12.62 ns | 189.82 ns | 184.87 ns | 36.64 us | 1.61 us | 10.11 us | 15.04x | 0.01x | 0.12x | 0.02x |
| `acosh e` | 12.72 ns | 545.68 ns | 2.20 us | 40.80 us | 1.68 us | 9.97 us | 42.92x | 0.01x | 0.32x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 33.42 ns | 32.62 ns | 27.10 ns | 15.58 ns | 0.95 ns | 69.81x | 1.23x | 2.14x | 35.25x |
| `one` | 0.49 ns | 41.17 ns | 39.02 ns | 43.26 ns | 31.38 ns | 30.69 ns | 83.58x | 0.95x | 1.31x | 1.34x |
| `e` | 0.49 ns | 69.65 ns | 66.07 ns | 55.51 ns | 1.11 us | 229.44 ns | 141.61x | 1.25x | 0.06x | 0.30x |
| `pi` | 0.50 ns | 51.89 ns | 50.97 ns | 44.74 ns | 50.15 ns | 231.05 ns | 104.12x | 1.16x | 1.03x | 0.22x |
| `tau` | 0.49 ns | 197.56 ns | 193.48 ns | 119.18 ns | 103.34 ns | 1.93 us | 401.85x | 1.66x | 1.91x | 0.10x |
| `add` | 5.36 ns | 272.60 ns | 375.45 ns | 55.20 ns | 43.10 ns | 1.30 us | 50.82x | 4.94x | 6.33x | 0.21x |
| `sub` | 5.37 ns | 290.09 ns | 440.96 ns | 58.74 ns | 46.16 ns | 2.53 us | 54.00x | 4.94x | 6.28x | 0.11x |
| `neg` | 5.33 ns | 54.34 ns | 50.38 ns | 23.16 ns | 21.70 ns | 1.11 us | 10.19x | 2.35x | 2.50x | 0.05x |
| `mul` | 5.53 ns | 198.99 ns | 465.89 ns | 58.50 ns | 46.69 ns | 1.59 us | 35.97x | 3.40x | 4.26x | 0.13x |
| `div` | 8.56 ns | 375.52 ns | 418.52 ns | 137.26 ns | 63.64 ns | 2.54 us | 43.85x | 2.74x | 5.90x | 0.15x |
| `reciprocal` | 9.07 ns | 111.26 ns | 105.15 ns | 161.47 ns | 67.78 ns | 1.55 us | 12.27x | 0.69x | 1.64x | 0.07x |
| `reciprocal checked` | 9.44 ns | 112.05 ns | 110.29 ns | 161.14 ns | 66.83 ns | 1.53 us | 11.86x | 0.70x | 1.68x | 0.07x |
| `reciprocal checked abort` | 25.29 ns | 120.18 ns | 119.45 ns | 160.31 ns | 65.82 ns | 1.53 us | 4.75x | 0.75x | 1.83x | 0.08x |
| `pow` | 20.89 ns | 10.45 us | 10.96 us | 55.53 us | 2.94 us | 2.39 us | 500.20x | 0.19x | 3.55x | 4.37x |
| `powi` | 6.05 ns | 509.34 ns | 2.84 us | 294.99 ns | 85.29 ns | 1.64 us | 84.14x | 1.73x | 5.97x | 0.31x |
| `exp` | 10.53 ns | 1.78 us | 3.20 us | 14.26 us | 926.05 ns | 1.93 us | 169.40x | 0.13x | 1.93x | 0.92x |
| `ln` | 11.15 ns | 1.50 us | 1.52 us | 29.92 us | 1.32 us | 1.85 us | 134.32x | 0.05x | 1.13x | 0.81x |
| `log10` | 13.66 ns | 2.74 us | 2.30 us | 35.87 us | 2.76 us | 6.88 us | 200.75x | 0.08x | 0.99x | 0.40x |
| `log10 abort` | 17.09 ns | 2.84 us | 2.35 us | 35.97 us | 2.79 us | 6.85 us | 166.38x | 0.08x | 1.02x | 0.41x |
| `sqrt` | 20.33 ns | 1.61 us | 1.66 us | 4.98 us | 97.71 ns | 1.49 us | 79.08x | 0.32x | 16.45x | 1.08x |
| `sin` | 15.49 ns | 149.11 ns | 150.94 ns | 14.10 us | 1.24 us | 2.25 us | 9.62x | 0.01x | 0.12x | 0.07x |
| `cos` | 18.85 ns | 148.83 ns | 150.27 ns | 12.51 us | 640.34 ns | 1.77 us | 7.90x | 0.01x | 0.23x | 0.08x |
| `tan` | 25.70 ns | 2.84 us | 3.32 us | 29.69 us | 1.58 us | 6.68 us | 110.42x | 0.10x | 1.80x | 0.42x |
| `sinh` | 18.86 ns | 4.30 us | 4.53 us | 3.44 us | 1.15 us | 10.81 us | 227.91x | 1.25x | 3.74x | 0.40x |
| `cosh` | 18.61 ns | 4.26 us | 4.51 us | 8.28 us | 1.05 us | 9.60 us | 228.77x | 0.51x | 4.05x | 0.44x |
| `tanh` | 23.67 ns | 6.04 us | 6.17 us | 3.45 us | 1.19 us | 23.48 us | 255.25x | 1.75x | 5.08x | 0.26x |
| `asin` | 13.51 ns | 694.47 ns | 1.76 us | 21.89 us | 2.46 us | 13.96 us | 51.39x | 0.03x | 0.28x | 0.05x |
| `asin abort` | 17.36 ns | 685.30 ns | 1.77 us | 21.89 us | 2.51 us | 14.29 us | 39.48x | 0.03x | 0.27x | 0.05x |
| `acos` | 13.71 ns | 903.13 ns | 2.53 us | 27.00 us | 2.66 us | 14.07 us | 65.85x | 0.03x | 0.34x | 0.06x |
| `acos abort` | 17.88 ns | 926.56 ns | 2.55 us | 26.58 us | 2.70 us | 14.54 us | 51.83x | 0.03x | 0.34x | 0.06x |
| `atan` | 15.53 ns | 408.25 ns | 1.09 us | 18.78 us | 2.34 us | 18.96 us | 26.29x | 0.02x | 0.17x | 0.02x |
| `atan abort` | 21.70 ns | 427.20 ns | 1.14 us | 18.67 us | 2.44 us | 19.44 us | 19.69x | 0.02x | 0.17x | 0.02x |
| `asinh` | 31.03 ns | 437.44 ns | 1.09 us | 39.52 us | 1.66 us | 7.79 us | 14.10x | 0.01x | 0.26x | 0.06x |
| `asinh abort` | 31.92 ns | 451.48 ns | 1.07 us | 39.95 us | 1.65 us | 7.91 us | 14.14x | 0.01x | 0.27x | 0.06x |
| `acosh` | 12.90 ns | 366.90 ns | 1.13 us | 42.00 us | 3.42 us | 10.59 us | 28.44x | 0.01x | 0.11x | 0.03x |
| `acosh abort` | 17.28 ns | 377.08 ns | 1.14 us | 41.90 us | 3.45 us | 10.83 us | 21.82x | 0.01x | 0.11x | 0.03x |
| `atanh` | 13.91 ns | 734.12 ns | 1.37 us | 35.73 us | 1.31 us | 15.23 us | 52.78x | 0.02x | 0.56x | 0.05x |
| `atanh abort` | 17.42 ns | 738.85 ns | 1.42 us | 34.84 us | 1.33 us | 15.28 us | 42.41x | 0.02x | 0.56x | 0.05x |
| `zero status` | 1.21 ns | 1.96 ns | 1.97 ns | 1.09 ns | 6.83 ns | 8.17 ns | 1.63x | 1.80x | 0.29x | 0.24x |
| `zero status abort` | 1.48 ns | 3.51 ns | 3.62 ns | 1.09 ns | 7.14 ns | 8.24 ns | 2.38x | 3.21x | 0.49x | 0.43x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.96 ns | 67.63 ns | 67.01 ns | 50.97 ns | - | 1.92 ns | 70.41x | 1.33x | - | 35.22x |
| `one` | 5.54 ns | 71.40 ns | 71.90 ns | 66.49 ns | - | 32.95 ns | 12.90x | 1.07x | - | 2.17x |
| `i` | 0.99 ns | 71.71 ns | 73.00 ns | 67.32 ns | - | 30.13 ns | 72.68x | 1.07x | - | 2.38x |
| `free i` | 0.96 ns | 71.32 ns | 73.35 ns | 68.04 ns | - | 30.31 ns | 74.11x | 1.05x | - | 2.35x |
| `conjugate` | 2.17 ns | 113.51 ns | 112.55 ns | 41.85 ns | - | 1.09 us | 52.19x | 2.71x | - | 0.10x |
| `norm squared` | 6.03 ns | 369.88 ns | 1.47 us | 162.95 ns | - | 4.37 us | 61.38x | 2.27x | - | 0.08x |
| `reciprocal` | 18.13 ns | 1.89 us | 2.94 us | 470.48 ns | - | 10.96 us | 104.32x | 4.02x | - | 0.17x |
| `reciprocal checked` | 15.92 ns | 1.88 us | 2.98 us | 466.79 ns | - | 11.05 us | 117.95x | 4.02x | - | 0.17x |
| `powi` | 18.65 ns | 2.56 us | 13.88 us | 1.53 us | - | 45.21 us | 137.07x | 1.67x | - | 0.06x |
| `powi checked` | 18.14 ns | 2.55 us | 14.27 us | 1.47 us | - | 45.61 us | 140.39x | 1.73x | - | 0.06x |
| `div checked` | 20.42 ns | 2.97 us | 6.47 us | 822.34 ns | - | 22.27 us | 145.63x | 3.62x | - | 0.13x |
| `div real checked` | 9.51 ns | 743.43 ns | 727.00 ns | 267.02 ns | - | 5.31 us | 78.18x | 2.78x | - | 0.14x |
| `from scalar` | 1.46 ns | 88.89 ns | 91.94 ns | 44.58 ns | - | 10.51 ns | 61.06x | 1.99x | - | 8.46x |
| `add` | 6.28 ns | 557.70 ns | 967.36 ns | 105.63 ns | - | 2.60 us | 88.74x | 5.28x | - | 0.21x |
| `sub` | 6.03 ns | 586.07 ns | 964.81 ns | 131.43 ns | - | 4.82 us | 97.23x | 4.46x | - | 0.12x |
| `neg` | 2.59 ns | 118.75 ns | 115.05 ns | 42.97 ns | - | 2.21 us | 45.86x | 2.76x | - | 0.05x |
| `mul` | 7.51 ns | 1.04 us | 3.35 us | 327.31 ns | - | 10.12 us | 137.85x | 3.16x | - | 0.10x |
| `div` | 18.71 ns | 2.99 us | 6.50 us | 833.42 ns | - | 21.94 us | 159.79x | 3.59x | - | 0.14x |
| `div real` | 10.11 ns | 740.96 ns | 708.43 ns | 271.91 ns | - | 5.24 us | 73.27x | 2.73x | - | 0.14x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.86 ns | 792.09 ns | 5.96 us | 334.12 ns | 256.95 ns | 7.49 us | 115.43x | 2.37x | 3.08x | 0.11x |
| `vec3 magnitude` | 10.10 ns | 4.69 us | 8.86 us | 5.65 us | 343.63 ns | 9.17 us | 464.56x | 0.83x | 13.66x | 0.51x |
| `vec3 normalize` | 25.95 ns | 8.94 us | 14.73 us | 6.25 us | 593.89 ns | 17.30 us | 344.31x | 1.43x | 15.05x | 0.52x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.18 ns | 291.46 ns | 1.56 us | 69.96 ns | 58.38 ns | 753.18 ns | 91.73x | 4.17x | 4.99x | 0.39x |
| `vec3 zero` | 1.42 ns | 183.95 ns | 180.44 ns | 62.85 ns | 31.13 ns | 2.84 ns | 129.31x | 2.93x | 5.91x | 64.71x |
| `vec3 dot abort` | 28.32 ns | 1.16 us | 4.74 us | 281.27 ns | 203.19 ns | 7.42 us | 40.94x | 4.12x | 5.71x | 0.16x |
| `vec3 magnitude abort` | 39.34 ns | 5.05 us | 6.61 us | 5.59 us | 327.65 ns | 9.42 us | 128.30x | 0.90x | 15.41x | 0.54x |
| `vec3 normalize checked` | 27.02 ns | 8.94 us | 10.41 us | 5.98 us | 561.03 ns | 17.74 us | 331.02x | 1.49x | 15.94x | 0.50x |
| `vec3 normalize checked abort` | 56.58 ns | 9.30 us | 10.68 us | 6.08 us | 541.52 ns | 17.68 us | 164.30x | 1.53x | 17.17x | 0.53x |
| `vec3 div scalar checked` | 10.57 ns | 1.49 us | 1.69 us | 422.45 ns | - | - | 141.17x | 3.53x | - | - |
| `vec3 div scalar checked abort` | 18.06 ns | 1.46 us | 1.73 us | 418.07 ns | - | - | 81.01x | 3.50x | - | - |
| `vec3 add` | 7.30 ns | 1.16 us | 1.89 us | 158.55 ns | 132.17 ns | 4.10 us | 158.84x | 7.31x | 8.77x | 0.28x |
| `vec3 add scalar` | 6.49 ns | 898.02 ns | 1.57 us | 175.73 ns | 136.33 ns | 3.88 us | 138.42x | 5.11x | 6.59x | 0.23x |
| `vec3 sub` | 7.28 ns | 1.17 us | 2.05 us | 189.80 ns | 139.95 ns | 7.36 us | 161.31x | 6.19x | 8.39x | 0.16x |
| `vec3 sub scalar` | 6.49 ns | 918.11 ns | 1.36 us | 172.34 ns | 128.07 ns | 7.12 us | 141.39x | 5.33x | 7.17x | 0.13x |
| `vec3 neg` | 3.79 ns | 218.91 ns | 220.30 ns | 59.09 ns | 52.46 ns | 3.27 us | 57.74x | 3.70x | 4.17x | 0.07x |
| `vec3 mul scalar` | 7.07 ns | 633.42 ns | 2.21 us | 168.03 ns | 125.49 ns | 4.51 us | 89.53x | 3.77x | 5.05x | 0.14x |
| `vec3 div scalar` | 18.06 ns | 1.45 us | 1.78 us | 396.22 ns | 177.12 ns | 7.94 us | 80.35x | 3.66x | 8.19x | 0.18x |
| `vec4 dot` | 7.42 ns | 946.10 ns | 3.42 us | 467.92 ns | 328.49 ns | 10.14 us | 127.46x | 2.02x | 2.88x | 0.09x |
| `vec4 magnitude` | 13.04 ns | 4.55 us | 5.70 us | 5.70 us | 407.77 ns | 11.67 us | 349.19x | 0.80x | 11.17x | 0.39x |
| `vec4 normalize` | 37.03 ns | 9.14 us | 9.18 us | 6.41 us | 700.37 ns | 22.55 us | 246.85x | 1.43x | 13.05x | 0.41x |
| `vec4 add` | 7.40 ns | 1.44 us | 2.07 us | 221.17 ns | 178.01 ns | 5.29 us | 194.58x | 6.51x | 8.09x | 0.27x |
| `vec4 add scalar` | 6.97 ns | 1.21 us | 1.90 us | 241.66 ns | 180.59 ns | 5.09 us | 173.55x | 5.01x | 6.70x | 0.24x |
| `vec4 sub` | 5.22 ns | 1.44 us | 1.97 us | 247.70 ns | 185.74 ns | 9.62 us | 274.99x | 5.80x | 7.73x | 0.15x |
| `vec4 sub scalar` | 4.66 ns | 1.18 us | 1.65 us | 224.03 ns | 179.69 ns | 9.33 us | 252.45x | 5.25x | 6.54x | 0.13x |
| `vec4 neg` | 5.10 ns | 299.84 ns | 315.74 ns | 80.13 ns | 64.97 ns | 4.13 us | 58.75x | 3.74x | 4.62x | 0.07x |
| `vec4 mul scalar` | 7.38 ns | 857.56 ns | 2.28 us | 227.16 ns | 160.91 ns | 5.76 us | 116.15x | 3.78x | 5.33x | 0.15x |
| `vec4 div scalar` | 14.47 ns | 1.96 us | 1.80 us | 564.38 ns | 224.39 ns | 10.22 us | 135.15x | 3.46x | 8.71x | 0.19x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.55 ns | 1.77 us | 3.33 us | 1.02 us | 871.84 ns | 22.88 us | 140.81x | 1.74x | 2.03x | 0.08x |
| `mat3 inverse` | 81.84 ns | 17.71 us | 11.47 us | 3.24 us | 2.54 us | 83.52 us | 216.43x | 5.47x | 6.97x | 0.21x |
| `mat3 mul mat3` | 78.88 ns | 7.53 us | 12.05 us | 2.98 us | 2.39 us | 62.44 us | 95.47x | 2.53x | 3.15x | 0.12x |
| `mat3 transform vec3` | 15.83 ns | 3.25 us | 10.67 us | 1.06 us | 893.63 ns | 21.41 us | 205.10x | 3.05x | 3.63x | 0.15x |
| `mat4 determinant` | 46.01 ns | 5.10 us | 3.86 us | 4.77 us | 4.21 us | 95.65 us | 110.77x | 1.07x | 1.21x | 0.05x |
| `mat4 inverse` | 176.11 ns | 33.37 us | 15.25 us | 11.72 us | 9.28 us | 356.54 us | 189.50x | 2.85x | 3.60x | 0.09x |
| `mat4 mul mat4` | 126.49 ns | 14.83 us | 13.00 us | 6.77 us | 5.56 us | 141.72 us | 117.23x | 2.19x | 2.67x | 0.10x |
| `mat4 transform vec4` | 25.61 ns | 5.24 us | 4.15 us | 1.98 us | 1.71 us | 36.23 us | 204.78x | 2.64x | 3.07x | 0.14x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 36.56 ns | 1.29 us | 4.37 us | 224.63 ns | 235.23 ns | 2.20 us | 35.36x | 5.75x | 5.50x | 0.59x |
| `mat3 zero` | 22.41 ns | 674.93 ns | 663.81 ns | 270.94 ns | 209.91 ns | 11.97 ns | 30.12x | 2.49x | 3.22x | 56.39x |
| `mat3 identity` | 9.88 ns | 767.09 ns | 769.74 ns | 333.83 ns | 264.36 ns | 142.08 ns | 77.65x | 2.30x | 2.90x | 5.40x |
| `mat3 transpose` | 9.06 ns | 879.93 ns | 870.56 ns | 244.76 ns | 219.70 ns | 145.01 ns | 97.18x | 3.60x | 4.01x | 6.07x |
| `mat3 reciprocal` | 91.29 ns | 17.77 us | 36.66 us | 3.10 us | 2.42 us | 86.49 us | 194.61x | 5.72x | 7.35x | 0.21x |
| `mat3 reciprocal checked` | 92.02 ns | 17.68 us | 37.18 us | 3.01 us | 2.43 us | 86.68 us | 192.11x | 5.87x | 7.26x | 0.20x |
| `mat3 inverse checked` | 91.27 ns | 18.02 us | 37.03 us | 3.05 us | 2.41 us | 84.54 us | 197.46x | 5.91x | 7.48x | 0.21x |
| `mat3 inverse checked abort` | 134.29 ns | 17.87 us | 37.89 us | 3.08 us | 2.46 us | 86.95 us | 133.10x | 5.81x | 7.27x | 0.21x |
| `mat3 powi` | 164.88 ns | 15.75 us | 88.58 us | 7.03 us | 6.43 us | 156.42 us | 95.56x | 2.24x | 2.45x | 0.10x |
| `mat3 powi checked` | 164.88 ns | 15.87 us | 89.96 us | 7.07 us | 6.37 us | 154.96 us | 96.24x | 2.24x | 2.49x | 0.10x |
| `mat3 powi checked abort` | 167.24 ns | 16.30 us | 93.58 us | 7.06 us | 6.46 us | 157.23 us | 97.45x | 2.31x | 2.52x | 0.10x |
| `mat3 div scalar checked` | 28.77 ns | 4.89 us | 6.25 us | 1.52 us | 821.01 ns | 22.23 us | 169.81x | 3.22x | 5.95x | 0.22x |
| `mat3 div scalar checked abort` | 33.99 ns | 5.02 us | 6.39 us | 1.51 us | 837.58 ns | 22.12 us | 147.72x | 3.33x | 6.00x | 0.23x |
| `mat3 div matrix checked` | 161.31 ns | 61.44 us | 62.78 us | 5.64 us | 4.68 us | 161.55 us | 380.91x | 10.89x | 13.12x | 0.38x |
| `mat3 div matrix checked abort` | 176.80 ns | 61.93 us | 62.96 us | 5.71 us | 4.56 us | 163.48 us | 350.27x | 10.85x | 13.59x | 0.38x |
| `mat3 add` | 14.93 ns | 3.88 us | 6.23 us | 560.49 ns | 533.96 ns | 11.99 us | 260.07x | 6.93x | 7.27x | 0.32x |
| `mat3 add scalar` | 12.49 ns | 2.85 us | 5.03 us | 844.05 ns | 741.29 ns | 12.18 us | 228.17x | 3.38x | 3.84x | 0.23x |
| `mat3 sub` | 18.42 ns | 3.90 us | 6.23 us | 578.81 ns | 570.50 ns | 21.23 us | 211.80x | 6.74x | 6.84x | 0.18x |
| `mat3 sub scalar` | 10.89 ns | 2.94 us | 4.78 us | 839.30 ns | 753.67 ns | 21.51 us | 269.56x | 3.50x | 3.90x | 0.14x |
| `mat3 neg` | 15.95 ns | 830.58 ns | 824.76 ns | 491.31 ns | 476.52 ns | 8.83 us | 52.09x | 1.69x | 1.74x | 0.09x |
| `mat3 mul scalar` | 13.71 ns | 2.24 us | 6.28 us | 790.03 ns | 691.27 ns | 12.43 us | 163.16x | 2.83x | 3.24x | 0.18x |
| `mat3 div scalar` | 27.72 ns | 4.89 us | 6.20 us | 1.48 us | 827.32 ns | 22.38 us | 176.42x | 3.30x | 5.91x | 0.22x |
| `mat3 div matrix` | 162.55 ns | 63.16 us | 62.26 us | 5.62 us | 4.52 us | 163.62 us | 388.54x | 11.24x | 13.99x | 0.39x |
| `mat3 bitxor` | 167.93 ns | 16.06 us | 90.24 us | 6.98 us | 6.46 us | 154.13 us | 95.61x | 2.30x | 2.49x | 0.10x |
| `mat4 zero` | 17.70 ns | 1.23 us | 1.23 us | 460.89 ns | 391.80 ns | 14.99 ns | 69.66x | 2.67x | 3.15x | 82.22x |
| `mat4 identity` | 15.04 ns | 1.40 us | 1.40 us | 574.12 ns | 449.98 ns | 218.07 ns | 93.16x | 2.44x | 3.11x | 6.43x |
| `mat4 transpose` | 13.65 ns | 1.46 us | 1.44 us | 434.26 ns | 387.57 ns | 188.53 ns | 106.92x | 3.36x | 3.77x | 7.74x |
| `mat4 reciprocal` | 176.48 ns | 34.71 us | 63.93 us | 11.48 us | 9.01 us | 348.16 us | 196.66x | 3.02x | 3.85x | 0.10x |
| `mat4 reciprocal checked` | 170.50 ns | 33.68 us | 64.95 us | 11.11 us | 9.21 us | 344.44 us | 197.53x | 3.03x | 3.66x | 0.10x |
| `mat4 powi` | 253.51 ns | 32.12 us | 112.01 us | 16.40 us | 14.65 us | 361.23 us | 126.69x | 1.96x | 2.19x | 0.09x |
| `mat4 powi checked` | 253.83 ns | 32.24 us | 111.67 us | 16.20 us | 14.59 us | 357.79 us | 127.00x | 1.99x | 2.21x | 0.09x |
| `mat4 add` | 58.78 ns | 6.25 us | 7.03 us | 964.08 ns | 897.67 ns | 19.79 us | 106.37x | 6.49x | 6.97x | 0.32x |
| `mat4 add scalar` | 20.78 ns | 5.22 us | 7.89 us | 1.50 us | 1.21 us | 20.44 us | 251.04x | 3.48x | 4.31x | 0.26x |
| `mat4 sub` | 44.29 ns | 6.20 us | 7.26 us | 1.03 us | 915.83 ns | 35.77 us | 140.06x | 6.01x | 6.77x | 0.17x |
| `mat4 sub scalar` | 15.49 ns | 5.25 us | 7.61 us | 1.55 us | 1.23 us | 37.05 us | 338.64x | 3.39x | 4.26x | 0.14x |
| `mat4 neg` | 14.54 ns | 1.40 us | 1.35 us | 908.81 ns | 765.90 ns | 14.02 us | 96.28x | 1.54x | 1.83x | 0.10x |
| `mat4 mul scalar` | 24.59 ns | 3.90 us | 8.20 us | 1.41 us | 1.15 us | 21.06 us | 158.75x | 2.76x | 3.40x | 0.19x |
| `mat4 div scalar` | 33.96 ns | 8.16 us | 8.47 us | 2.64 us | 1.42 us | 38.35 us | 240.34x | 3.09x | 5.73x | 0.21x |
| `mat4 div matrix` | 235.94 ns | 124.64 us | 101.31 us | 17.34 us | 14.58 us | 537.85 us | 528.25x | 7.19x | 8.55x | 0.23x |
| `mat4 bitxor` | 253.13 ns | 31.92 us | 114.81 us | 16.58 us | 14.65 us | 377.63 us | 126.10x | 1.93x | 2.18x | 0.08x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.72 ns | 219.40 ns | 301.47 ns | - | - | 1.31 us | 58.98x | - | - | 0.17x |
| `scalar add ref_owned` | 12.74 ns | 208.51 ns | 300.82 ns | - | - | 1.30 us | 16.36x | - | - | 0.16x |
| `scalar add refs` | 5.55 ns | 186.96 ns | 280.65 ns | - | - | 1.31 us | 33.71x | - | - | 0.14x |
| `scalar add owned_ref_with_clone` | 9.24 ns | 234.54 ns | 332.33 ns | - | - | - | 25.39x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.50 ns | 234.07 ns | 329.65 ns | - | - | - | 42.59x | - | - | - |
| `scalar sub owned_ref` | 3.88 ns | 212.33 ns | 371.56 ns | - | - | 2.42 us | 54.71x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.73 ns | 209.78 ns | 368.32 ns | - | - | 2.47 us | 16.47x | - | - | 0.08x |
| `scalar sub refs` | 5.82 ns | 189.42 ns | 348.08 ns | - | - | 2.43 us | 32.53x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.33 ns | 235.61 ns | 399.71 ns | - | - | - | 25.26x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.86 ns | 230.89 ns | 399.62 ns | - | - | - | 39.41x | - | - | - |
| `scalar mul owned_ref` | 4.69 ns | 117.94 ns | 419.33 ns | - | - | 1.54 us | 25.17x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.95 ns | 123.90 ns | 399.27 ns | - | - | 1.58 us | 8.88x | - | - | 0.08x |
| `scalar mul refs` | 6.38 ns | 100.40 ns | 373.58 ns | - | - | 1.57 us | 15.74x | - | - | 0.06x |
| `scalar mul owned_ref_with_clone` | 9.92 ns | 148.58 ns | 427.13 ns | - | - | - | 14.97x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.46 ns | 147.86 ns | 414.06 ns | - | - | - | 22.89x | - | - | - |
| `scalar div owned_ref` | 6.08 ns | 284.96 ns | 343.68 ns | - | - | 2.58 us | 46.85x | - | - | 0.11x |
| `scalar div ref_owned` | 17.29 ns | 296.80 ns | 377.90 ns | - | - | 2.60 us | 17.17x | - | - | 0.11x |
| `scalar div refs` | 6.83 ns | 270.21 ns | 334.58 ns | - | - | 2.62 us | 39.56x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.92 ns | 318.37 ns | 376.87 ns | - | - | - | 20.00x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.91 ns | 321.79 ns | 379.81 ns | - | - | - | 20.22x | - | - | - |
| `vec3 add refs` | 6.11 ns | 704.89 ns | 1.48 us | - | - | 3.96 us | 115.33x | - | - | 0.18x |
| `vec3 sub refs` | 6.25 ns | 698.53 ns | 1.53 us | - | - | 7.36 us | 111.68x | - | - | 0.09x |
| `vec3 neg ref` | 3.31 ns | 239.05 ns | 239.68 ns | - | - | 3.18 us | 72.33x | - | - | 0.08x |
| `vec3 add_scalar_ref` | 6.64 ns | 927.98 ns | 1.64 us | - | - | 3.92 us | 139.73x | - | - | 0.24x |
| `vec3 sub_scalar_ref` | 6.61 ns | 926.40 ns | 1.40 us | - | - | 7.15 us | 140.22x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 7.07 ns | 651.86 ns | 2.23 us | - | - | 4.46 us | 92.23x | - | - | 0.15x |
| `vec3 div_scalar_ref` | 8.33 ns | 1.71 us | 2.02 us | - | - | 7.89 us | 204.75x | - | - | 0.22x |
| `vec4 add refs` | 6.75 ns | 837.35 ns | 1.48 us | - | - | 5.44 us | 124.09x | - | - | 0.15x |
| `vec4 sub refs` | 3.23 ns | 849.72 ns | 1.37 us | - | - | 10.27 us | 263.25x | - | - | 0.08x |
| `vec4 neg ref` | 4.25 ns | 336.14 ns | 329.95 ns | - | - | 4.20 us | 79.02x | - | - | 0.08x |
| `vec4 add_scalar_ref` | 6.94 ns | 1.48 us | 2.21 us | - | - | 5.15 us | 212.88x | - | - | 0.29x |
| `vec4 sub_scalar_ref` | 4.25 ns | 1.46 us | 1.96 us | - | - | 9.33 us | 344.34x | - | - | 0.16x |
| `vec4 mul_scalar_ref` | 7.44 ns | 1.12 us | 2.66 us | - | - | 5.70 us | 151.25x | - | - | 0.20x |
| `vec4 div_scalar_ref` | 11.79 ns | 2.16 us | 2.03 us | - | - | 10.02 us | 183.39x | - | - | 0.22x |
| `mat3 add refs` | 11.34 ns | 2.17 us | 4.50 us | - | - | 11.83 us | 191.80x | - | - | 0.18x |
| `mat3 sub refs` | 11.09 ns | 2.27 us | 4.22 us | - | - | 21.80 us | 204.65x | - | - | 0.10x |
| `mat3 mul refs` | 32.36 ns | 6.02 us | 29.14 us | - | - | 63.93 us | 186.06x | - | - | 0.09x |
| `mat3 div refs` | 141.43 ns | 62.31 us | 61.96 us | - | - | 166.07 us | 440.60x | - | - | 0.38x |
| `mat3 neg ref` | 9.71 ns | 979.91 ns | 988.09 ns | - | - | 8.87 us | 100.93x | - | - | 0.11x |
| `mat3 add_scalar_ref` | 45.70 ns | 3.22 us | 5.27 us | - | - | 12.13 us | 70.45x | - | - | 0.27x |
| `mat3 sub_scalar_ref` | 45.09 ns | 3.08 us | 5.14 us | - | - | 21.61 us | 68.36x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 48.14 ns | 2.36 us | 6.59 us | - | - | 12.21 us | 49.12x | - | - | 0.19x |
| `mat3 div_scalar_ref` | 22.12 ns | 5.00 us | 6.30 us | - | - | 22.58 us | 225.99x | - | - | 0.22x |
| `mat4 add refs` | 17.65 ns | 3.33 us | 4.29 us | - | - | 20.01 us | 188.81x | - | - | 0.17x |
| `mat4 sub refs` | 16.37 ns | 3.37 us | 4.29 us | - | - | 36.67 us | 205.72x | - | - | 0.09x |
| `mat4 mul refs` | 97.81 ns | 12.89 us | 29.04 us | - | - | 146.86 us | 131.82x | - | - | 0.09x |
| `mat4 div refs` | 178.01 ns | 127.10 us | 96.04 us | - | - | 546.02 us | 713.99x | - | - | 0.23x |
| `mat4 neg ref` | 12.63 ns | 1.52 us | 1.41 us | - | - | 14.45 us | 120.37x | - | - | 0.11x |
| `mat4 add_scalar_ref` | 60.69 ns | 5.28 us | 7.69 us | - | - | 20.80 us | 86.93x | - | - | 0.25x |
| `mat4 sub_scalar_ref` | 48.11 ns | 5.30 us | 7.62 us | - | - | 37.73 us | 110.07x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 64.75 ns | 3.92 us | 8.29 us | - | - | 20.59 us | 60.61x | - | - | 0.19x |
| `mat4 div_scalar_ref` | 28.55 ns | 8.49 us | 8.75 us | - | - | 38.84 us | 297.33x | - | - | 0.22x |
| `mat3 transform_vec refs` | 14.90 ns | 2.81 us | 11.82 us | - | - | 20.71 us | 188.52x | - | - | 0.14x |
| `mat4 transform_vec refs` | 23.33 ns | 4.00 us | 11.96 us | - | - | 35.84 us | 171.34x | - | - | 0.11x |
| `complex add refs` | 7.77 ns | 365.20 ns | 746.99 ns | - | - | 2.68 us | 46.97x | - | - | 0.14x |
| `complex sub refs` | 8.00 ns | 381.23 ns | 768.04 ns | - | - | 4.87 us | 47.66x | - | - | 0.08x |
| `complex mul refs` | 8.08 ns | 860.60 ns | 3.20 us | - | - | 10.08 us | 106.52x | - | - | 0.09x |
| `complex div refs` | 17.45 ns | 3.16 us | 6.91 us | - | - | 22.72 us | 181.09x | - | - | 0.14x |
| `complex neg ref` | 2.36 ns | 116.14 ns | 107.69 ns | - | - | 2.21 us | 49.20x | - | - | 0.05x |
| `complex div_real_ref` | 7.56 ns | 721.69 ns | 701.50 ns | - | - | 5.39 us | 95.42x | - | - | 0.13x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 13.10 us |
| `astro sin 160` | 14.67 us |
| `astro sin 192` | 15.22 us |
| `astro sin 256` | 17.06 us |
