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
| `sin 0.1` | 11.04 ns | 187.16 ns | 194.35 ns | 11.23 us | 777.20 ns | 1.86 us | 16.96x | 0.02x | 0.24x | 0.10x |
| `cos 0.1` | 11.98 ns | 193.58 ns | 190.15 ns | 10.84 us | 500.25 ns | 1.70 us | 16.15x | 0.02x | 0.39x | 0.11x |
| `sin 1.23456789` | 11.94 ns | 252.52 ns | 242.12 ns | 13.03 us | 808.53 ns | 1.86 us | 21.14x | 0.02x | 0.31x | 0.14x |
| `cos 1.23456789` | 12.15 ns | 238.48 ns | 229.01 ns | 10.99 us | 596.22 ns | 1.69 us | 19.62x | 0.02x | 0.40x | 0.14x |
| `sin 1e6` | 12.85 ns | 5.52 us | 5.38 us | 16.08 us | 1.08 us | 2.07 us | 429.60x | 0.34x | 5.09x | 2.66x |
| `cos 1e6` | 12.69 ns | 5.29 us | 5.45 us | 13.78 us | 830.56 ns | 1.87 us | 416.81x | 0.38x | 6.37x | 2.83x |
| `sin 1e30` | 67.49 ns | 7.41 us | 7.72 us | 18.83 us | 2.87 us | 3.64 us | 109.75x | 0.39x | 2.58x | 2.03x |
| `cos 1e30` | 70.00 ns | 7.50 us | 7.55 us | 15.60 us | 989.05 ns | 3.26 us | 107.19x | 0.48x | 7.59x | 2.30x |
| `sin pi_7` | 12.00 ns | 192.25 ns | 527.56 ns | 12.19 us | 754.35 ns | 1.89 us | 16.03x | 0.02x | 0.25x | 0.10x |
| `cos pi_7` | 11.98 ns | 195.96 ns | 940.79 ns | 11.14 us | 539.43 ns | 1.72 us | 16.36x | 0.02x | 0.36x | 0.11x |
| `sin 1000pi_eps` | 12.04 ns | 5.53 us | 4.27 us | 16.49 us | 2.31 us | 3.02 us | 459.59x | 0.34x | 2.40x | 1.83x |
| `cos 1000pi_eps` | 12.71 ns | 5.34 us | 4.20 us | 13.83 us | 578.67 ns | 1.68 us | 420.45x | 0.39x | 9.23x | 3.17x |
| `asin 0.5` | 11.02 ns | 465.98 ns | 466.09 ns | 49.32 us | 2.90 us | 13.18 us | 42.30x | 0.01x | 0.16x | 0.04x |
| `acos 0.5` | 11.17 ns | 1.06 us | 1.08 us | 59.06 us | 2.96 us | 13.24 us | 95.18x | 0.02x | 0.36x | 0.08x |
| `atanh 0.5` | 14.68 ns | 1.44 us | 1.41 us | 34.58 us | 1.68 us | 12.84 us | 98.16x | 0.04x | 0.86x | 0.11x |
| `asin neg_0.999999` | 14.11 ns | 855.96 ns | 1.50 us | 14.13 us | 2.57 us | 13.07 us | 60.66x | 0.06x | 0.33x | 0.07x |
| `acos neg_0.999999` | 15.47 ns | 988.30 ns | 2.09 us | 18.79 us | 2.81 us | 13.66 us | 63.88x | 0.05x | 0.35x | 0.07x |
| `atanh neg_0.999999` | 14.75 ns | 436.58 ns | 631.00 ns | 37.20 us | 1.62 us | 12.96 us | 29.61x | 0.01x | 0.27x | 0.03x |
| `asin 0.999999` | 14.44 ns | 863.06 ns | 1.79 us | 14.09 us | 2.53 us | 12.99 us | 59.76x | 0.06x | 0.34x | 0.07x |
| `acos 0.999999` | 14.19 ns | 720.77 ns | 1.90 us | 18.64 us | 2.83 us | 13.29 us | 50.79x | 0.04x | 0.26x | 0.05x |
| `atanh 0.999999` | 14.83 ns | 483.34 ns | 908.88 ns | 31.74 us | 1.59 us | 12.75 us | 32.59x | 0.02x | 0.30x | 0.04x |
| `asin 1e-12` | 9.55 ns | 481.41 ns | 1.12 us | 8.06 us | 1.40 us | 15.16 us | 50.40x | 0.06x | 0.34x | 0.03x |
| `acos 1e-12` | 9.92 ns | 858.66 ns | 1.82 us | 9.79 us | 1.47 us | 15.31 us | 86.57x | 0.09x | 0.58x | 0.06x |
| `atanh 1e-12` | 9.72 ns | 472.18 ns | 854.90 ns | 36.40 us | 175.53 ns | 20.63 us | 48.60x | 0.01x | 2.69x | 0.02x |
| `atan 0.5` | 14.91 ns | 412.75 ns | 407.27 ns | 35.48 us | 2.77 us | 18.06 us | 27.68x | 0.01x | 0.15x | 0.02x |
| `asinh 0.5` | 27.27 ns | 454.33 ns | 446.84 ns | 39.00 us | 1.67 us | 7.47 us | 16.66x | 0.01x | 0.27x | 0.06x |
| `atan neg_1e-12` | 14.38 ns | 413.59 ns | 600.41 ns | 1.73 us | 1.13 us | 15.30 us | 28.77x | 0.24x | 0.37x | 0.03x |
| `asinh neg_1e-12` | 15.98 ns | 463.96 ns | 367.13 ns | 42.04 us | 8.78 us | 12.10 us | 29.03x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 15.67 ns | 390.85 ns | 379.94 ns | 2.89 us | 1.44 us | 18.19 us | 24.94x | 0.14x | 0.27x | 0.02x |
| `asinh 1e6` | 27.19 ns | 309.63 ns | 299.65 ns | 38.44 us | 1.68 us | 7.40 us | 11.39x | 0.01x | 0.18x | 0.04x |
| `atan neg_1e6` | 15.06 ns | 525.35 ns | 501.49 ns | 2.92 us | 1.43 us | 17.97 us | 34.88x | 0.18x | 0.37x | 0.03x |
| `asinh neg_1e6` | 27.26 ns | 344.32 ns | 336.12 ns | 36.33 us | 1.71 us | 7.14 us | 12.63x | 0.01x | 0.20x | 0.05x |
| `acosh 9` | 12.91 ns | 180.63 ns | 176.91 ns | 42.02 us | 1.63 us | 9.91 us | 13.99x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 11.54 ns | 521.99 ns | 1.40 us | 42.11 us | 8.53 us | 11.64 us | 45.24x | 0.01x | 0.06x | 0.04x |
| `acosh 1e6` | 12.41 ns | 184.21 ns | 176.64 ns | 36.97 us | 1.64 us | 10.21 us | 14.84x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.38 ns | 537.71 ns | 2.32 us | 42.26 us | 1.67 us | 9.81 us | 43.45x | 0.01x | 0.32x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.49 ns | 32.80 ns | 31.48 ns | 25.19 ns | 15.85 ns | 0.98 ns | 67.37x | 1.30x | 2.07x | 33.43x |
| `one` | 0.49 ns | 38.55 ns | 37.82 ns | 44.07 ns | 31.22 ns | 33.19 ns | 78.55x | 0.87x | 1.23x | 1.16x |
| `e` | 0.48 ns | 68.68 ns | 66.48 ns | 58.05 ns | 1.10 us | 227.01 ns | 143.38x | 1.18x | 0.06x | 0.30x |
| `pi` | 0.49 ns | 52.83 ns | 50.85 ns | 45.97 ns | 49.59 ns | 226.95 ns | 107.80x | 1.15x | 1.07x | 0.23x |
| `tau` | 0.49 ns | 199.40 ns | 190.44 ns | 118.10 ns | 105.11 ns | 1.84 us | 408.81x | 1.69x | 1.90x | 0.11x |
| `add` | 5.28 ns | 269.11 ns | 371.41 ns | 55.12 ns | 43.35 ns | 1.29 us | 51.00x | 4.88x | 6.21x | 0.21x |
| `sub` | 5.39 ns | 268.77 ns | 418.06 ns | 60.57 ns | 47.30 ns | 2.45 us | 49.90x | 4.44x | 5.68x | 0.11x |
| `neg` | 5.30 ns | 53.54 ns | 51.46 ns | 23.43 ns | 20.46 ns | 1.07 us | 10.09x | 2.29x | 2.62x | 0.05x |
| `mul` | 5.59 ns | 188.69 ns | 449.83 ns | 59.11 ns | 44.78 ns | 1.55 us | 33.75x | 3.19x | 4.21x | 0.12x |
| `div` | 9.15 ns | 363.73 ns | 433.80 ns | 141.16 ns | 62.52 ns | 2.65 us | 39.76x | 2.58x | 5.82x | 0.14x |
| `reciprocal` | 8.89 ns | 109.53 ns | 107.15 ns | 162.07 ns | 58.47 ns | 1.61 us | 12.33x | 0.68x | 1.87x | 0.07x |
| `reciprocal checked` | 8.99 ns | 108.43 ns | 106.02 ns | 160.10 ns | 58.85 ns | 1.62 us | 12.07x | 0.68x | 1.84x | 0.07x |
| `reciprocal checked abort` | 24.44 ns | 121.31 ns | 117.05 ns | 157.20 ns | 60.48 ns | 1.59 us | 4.96x | 0.77x | 2.01x | 0.08x |
| `pow` | 30.73 ns | 10.59 us | 10.61 us | 54.53 us | 2.93 us | 2.44 us | 344.54x | 0.19x | 3.62x | 4.33x |
| `powi` | 6.03 ns | 499.24 ns | 2.74 us | 288.22 ns | 87.12 ns | 1.64 us | 82.73x | 1.73x | 5.73x | 0.30x |
| `exp` | 19.87 ns | 1.82 us | 2.97 us | 14.26 us | 963.26 ns | 1.94 us | 91.69x | 0.13x | 1.89x | 0.94x |
| `ln` | 10.72 ns | 1.53 us | 1.49 us | 30.51 us | 1.36 us | 1.85 us | 142.56x | 0.05x | 1.12x | 0.83x |
| `log10` | 13.70 ns | 2.67 us | 2.36 us | 36.71 us | 2.89 us | 6.90 us | 194.57x | 0.07x | 0.92x | 0.39x |
| `log10 abort` | 17.60 ns | 2.70 us | 2.33 us | 36.37 us | 2.78 us | 6.82 us | 153.49x | 0.07x | 0.97x | 0.40x |
| `sqrt` | 8.45 ns | 1.57 us | 1.70 us | 5.32 us | 97.87 ns | 1.53 us | 185.89x | 0.30x | 16.05x | 1.03x |
| `sin` | 15.18 ns | 2.92 us | 3.22 us | 14.45 us | 1.26 us | 2.25 us | 192.52x | 0.20x | 2.32x | 1.30x |
| `cos` | 18.61 ns | 2.97 us | 3.19 us | 12.25 us | 658.55 ns | 1.78 us | 159.86x | 0.24x | 4.52x | 1.67x |
| `tan` | 24.52 ns | 2.88 us | 3.29 us | 29.82 us | 1.60 us | 6.94 us | 117.52x | 0.10x | 1.80x | 0.42x |
| `sinh` | 18.41 ns | 4.36 us | 4.58 us | 3.50 us | 1.15 us | 11.12 us | 236.74x | 1.25x | 3.78x | 0.39x |
| `cosh` | 18.35 ns | 4.31 us | 4.59 us | 8.12 us | 1.11 us | 9.93 us | 235.06x | 0.53x | 3.88x | 0.43x |
| `tanh` | 25.25 ns | 5.99 us | 6.22 us | 3.40 us | 1.27 us | 23.88 us | 237.08x | 1.76x | 4.73x | 0.25x |
| `asin` | 13.41 ns | 689.66 ns | 1.82 us | 21.68 us | 2.46 us | 14.31 us | 51.43x | 0.03x | 0.28x | 0.05x |
| `asin abort` | 17.88 ns | 691.26 ns | 1.82 us | 22.41 us | 2.44 us | 13.72 us | 38.66x | 0.03x | 0.28x | 0.05x |
| `acos` | 14.35 ns | 926.55 ns | 2.52 us | 27.01 us | 2.70 us | 14.15 us | 64.57x | 0.03x | 0.34x | 0.07x |
| `acos abort` | 18.24 ns | 931.01 ns | 2.61 us | 27.45 us | 2.63 us | 13.81 us | 51.04x | 0.03x | 0.35x | 0.07x |
| `atan` | 15.18 ns | 421.14 ns | 1.15 us | 18.67 us | 2.31 us | 18.92 us | 27.74x | 0.02x | 0.18x | 0.02x |
| `atan abort` | 21.85 ns | 438.31 ns | 1.11 us | 18.66 us | 2.36 us | 19.48 us | 20.06x | 0.02x | 0.19x | 0.02x |
| `asinh` | 35.58 ns | 425.13 ns | 1.10 us | 40.46 us | 1.68 us | 7.78 us | 11.95x | 0.01x | 0.25x | 0.05x |
| `asinh abort` | 32.26 ns | 441.36 ns | 1.12 us | 40.74 us | 1.70 us | 7.82 us | 13.68x | 0.01x | 0.26x | 0.06x |
| `acosh` | 12.54 ns | 364.64 ns | 1.18 us | 40.55 us | 3.50 us | 11.03 us | 29.08x | 0.01x | 0.10x | 0.03x |
| `acosh abort` | 16.10 ns | 360.29 ns | 1.14 us | 40.63 us | 3.35 us | 10.65 us | 22.37x | 0.01x | 0.11x | 0.03x |
| `atanh` | 13.86 ns | 712.42 ns | 1.39 us | 35.29 us | 1.29 us | 14.92 us | 51.38x | 0.02x | 0.55x | 0.05x |
| `atanh abort` | 17.32 ns | 740.50 ns | 1.39 us | 35.81 us | 1.33 us | 15.11 us | 42.76x | 0.02x | 0.56x | 0.05x |
| `zero status` | 1.24 ns | 1.89 ns | 1.87 ns | 1.06 ns | 7.06 ns | 8.31 ns | 1.52x | 1.78x | 0.27x | 0.23x |
| `zero status abort` | 1.44 ns | 2.91 ns | 2.79 ns | 0.97 ns | 6.98 ns | 8.36 ns | 2.02x | 2.99x | 0.42x | 0.35x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.95 ns | 65.41 ns | 66.09 ns | 47.56 ns | - | 1.93 ns | 69.04x | 1.38x | - | 33.91x |
| `one` | 5.17 ns | 72.10 ns | 69.12 ns | 64.66 ns | - | 31.65 ns | 13.94x | 1.11x | - | 2.28x |
| `i` | 0.98 ns | 70.15 ns | 68.91 ns | 63.18 ns | - | 34.32 ns | 71.66x | 1.11x | - | 2.04x |
| `free i` | 0.97 ns | 69.15 ns | 69.87 ns | 63.65 ns | - | 33.68 ns | 71.16x | 1.09x | - | 2.05x |
| `conjugate` | 2.17 ns | 116.66 ns | 115.74 ns | 42.69 ns | - | 1.08 us | 53.73x | 2.73x | - | 0.11x |
| `norm squared` | 6.03 ns | 378.19 ns | 1.47 us | 152.22 ns | - | 4.33 us | 62.74x | 2.48x | - | 0.09x |
| `reciprocal` | 14.35 ns | 1.88 us | 2.92 us | 462.92 ns | - | 10.79 us | 130.75x | 4.05x | - | 0.17x |
| `reciprocal checked` | 14.39 ns | 1.95 us | 2.96 us | 446.59 ns | - | 10.78 us | 135.54x | 4.37x | - | 0.18x |
| `powi` | 18.37 ns | 2.53 us | 14.38 us | 1.48 us | - | 43.10 us | 137.56x | 1.71x | - | 0.06x |
| `powi checked` | 18.57 ns | 2.52 us | 14.09 us | 1.47 us | - | 44.43 us | 135.78x | 1.72x | - | 0.06x |
| `div checked` | 18.60 ns | 3.02 us | 6.64 us | 786.03 ns | - | 21.89 us | 162.26x | 3.84x | - | 0.14x |
| `div real checked` | 9.49 ns | 759.15 ns | 715.89 ns | 265.16 ns | - | 5.61 us | 80.00x | 2.86x | - | 0.14x |
| `from scalar` | 1.44 ns | 87.47 ns | 87.79 ns | 47.24 ns | - | 10.73 ns | 60.57x | 1.85x | - | 8.15x |
| `add` | 6.13 ns | 563.50 ns | 927.01 ns | 112.38 ns | - | 2.84 us | 91.85x | 5.01x | - | 0.20x |
| `sub` | 6.16 ns | 587.89 ns | 957.30 ns | 124.26 ns | - | 4.81 us | 95.50x | 4.73x | - | 0.12x |
| `neg` | 2.59 ns | 121.65 ns | 114.66 ns | 44.01 ns | - | 2.21 us | 46.95x | 2.76x | - | 0.06x |
| `mul` | 11.33 ns | 1.06 us | 3.39 us | 307.25 ns | - | 9.94 us | 93.72x | 3.46x | - | 0.11x |
| `div` | 18.44 ns | 2.95 us | 6.69 us | 788.66 ns | - | 21.69 us | 160.10x | 3.74x | - | 0.14x |
| `div real` | 10.38 ns | 738.12 ns | 707.38 ns | 267.58 ns | - | 5.41 us | 71.09x | 2.76x | - | 0.14x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.82 ns | 770.80 ns | 6.07 us | 322.04 ns | 252.44 ns | 7.54 us | 113.05x | 2.39x | 3.05x | 0.10x |
| `vec3 magnitude` | 9.50 ns | 4.68 us | 8.89 us | 5.53 us | 356.85 ns | 9.05 us | 492.69x | 0.85x | 13.12x | 0.52x |
| `vec3 normalize` | 25.78 ns | 9.43 us | 15.61 us | 6.44 us | 603.67 ns | 17.08 us | 365.91x | 1.46x | 15.63x | 0.55x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.17 ns | 295.63 ns | 1.48 us | 71.84 ns | 57.65 ns | 730.44 ns | 93.38x | 4.12x | 5.13x | 0.40x |
| `vec3 zero` | 1.42 ns | 195.74 ns | 194.49 ns | 59.04 ns | 30.70 ns | 2.88 ns | 137.54x | 3.32x | 6.38x | 68.01x |
| `vec3 dot abort` | 27.91 ns | 1.21 us | 4.53 us | 258.37 ns | 205.33 ns | 7.26 us | 43.36x | 4.68x | 5.89x | 0.17x |
| `vec3 magnitude abort` | 38.71 ns | 5.16 us | 6.72 us | 5.50 us | 324.96 ns | 9.07 us | 133.20x | 0.94x | 15.87x | 0.57x |
| `vec3 normalize checked` | 26.06 ns | 8.75 us | 10.63 us | 6.04 us | 557.71 ns | 17.52 us | 335.72x | 1.45x | 15.68x | 0.50x |
| `vec3 normalize checked abort` | 55.37 ns | 9.12 us | 10.85 us | 6.00 us | 546.26 ns | 17.54 us | 164.80x | 1.52x | 16.70x | 0.52x |
| `vec3 div scalar checked` | 15.92 ns | 1.43 us | 1.70 us | 403.94 ns | - | - | 90.10x | 3.55x | - | - |
| `vec3 div scalar checked abort` | 18.11 ns | 1.43 us | 1.70 us | 409.06 ns | - | - | 78.94x | 3.50x | - | - |
| `vec3 add` | 6.77 ns | 1.07 us | 1.86 us | 159.61 ns | 131.34 ns | 4.09 us | 158.41x | 6.72x | 8.17x | 0.26x |
| `vec3 add scalar` | 6.63 ns | 923.70 ns | 1.58 us | 162.09 ns | 133.00 ns | 3.89 us | 139.30x | 5.70x | 6.95x | 0.24x |
| `vec3 sub` | 6.95 ns | 1.09 us | 1.89 us | 169.43 ns | 138.46 ns | 7.30 us | 157.53x | 6.46x | 7.91x | 0.15x |
| `vec3 sub scalar` | 6.49 ns | 895.95 ns | 1.34 us | 156.47 ns | 128.87 ns | 7.01 us | 137.96x | 5.73x | 6.95x | 0.13x |
| `vec3 neg` | 3.80 ns | 222.71 ns | 218.15 ns | 56.06 ns | 47.12 ns | 3.17 us | 58.65x | 3.97x | 4.73x | 0.07x |
| `vec3 mul scalar` | 6.94 ns | 660.14 ns | 2.16 us | 164.88 ns | 121.09 ns | 4.51 us | 95.18x | 4.00x | 5.45x | 0.15x |
| `vec3 div scalar` | 10.11 ns | 1.40 us | 1.67 us | 405.38 ns | 175.26 ns | 7.87 us | 138.55x | 3.45x | 7.99x | 0.18x |
| `vec4 dot` | 7.27 ns | 940.00 ns | 3.33 us | 441.29 ns | 318.35 ns | 9.86 us | 129.25x | 2.13x | 2.95x | 0.10x |
| `vec4 magnitude` | 13.05 ns | 4.55 us | 5.45 us | 5.72 us | 405.98 ns | 11.39 us | 348.76x | 0.80x | 11.21x | 0.40x |
| `vec4 normalize` | 36.49 ns | 9.02 us | 9.09 us | 6.22 us | 748.07 ns | 22.17 us | 247.24x | 1.45x | 12.06x | 0.41x |
| `vec4 add` | 8.29 ns | 1.47 us | 2.02 us | 225.14 ns | 176.28 ns | 5.45 us | 177.71x | 6.55x | 8.36x | 0.27x |
| `vec4 add scalar` | 7.15 ns | 1.21 us | 1.86 us | 228.67 ns | 182.05 ns | 5.29 us | 169.71x | 5.30x | 6.66x | 0.23x |
| `vec4 sub` | 5.35 ns | 1.49 us | 1.99 us | 216.23 ns | 179.77 ns | 9.97 us | 277.91x | 6.87x | 8.26x | 0.15x |
| `vec4 sub scalar` | 4.61 ns | 1.17 us | 1.65 us | 218.34 ns | 168.91 ns | 9.54 us | 254.26x | 5.37x | 6.94x | 0.12x |
| `vec4 neg` | 4.97 ns | 280.07 ns | 273.94 ns | 79.85 ns | 65.21 ns | 4.01 us | 56.39x | 3.51x | 4.30x | 0.07x |
| `vec4 mul scalar` | 7.48 ns | 792.75 ns | 2.17 us | 227.47 ns | 163.84 ns | 5.74 us | 106.02x | 3.49x | 4.84x | 0.14x |
| `vec4 div scalar` | 14.42 ns | 1.87 us | 1.67 us | 550.27 ns | 228.10 ns | 10.40 us | 129.74x | 3.40x | 8.20x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.44 ns | 1.80 us | 3.41 us | 973.00 ns | 870.91 ns | 22.49 us | 144.44x | 1.85x | 2.06x | 0.08x |
| `mat3 inverse` | 80.07 ns | 17.91 us | 11.10 us | 3.23 us | 2.53 us | 83.98 us | 223.66x | 5.54x | 7.09x | 0.21x |
| `mat3 mul mat3` | 73.21 ns | 7.60 us | 11.82 us | 2.95 us | 2.47 us | 63.37 us | 103.83x | 2.58x | 3.08x | 0.12x |
| `mat3 transform vec3` | 16.35 ns | 3.26 us | 10.68 us | 1.10 us | 911.68 ns | 20.23 us | 199.45x | 2.96x | 3.58x | 0.16x |
| `mat4 determinant` | 46.52 ns | 5.02 us | 3.75 us | 4.64 us | 4.27 us | 95.73 us | 107.85x | 1.08x | 1.17x | 0.05x |
| `mat4 inverse` | 163.30 ns | 34.93 us | 15.84 us | 11.25 us | 9.52 us | 348.73 us | 213.90x | 3.10x | 3.67x | 0.10x |
| `mat4 mul mat4` | 117.12 ns | 15.71 us | 12.61 us | 6.54 us | 5.43 us | 145.87 us | 134.16x | 2.40x | 2.89x | 0.11x |
| `mat4 transform vec4` | 25.14 ns | 5.13 us | 4.10 us | 1.96 us | 1.69 us | 35.86 us | 204.20x | 2.62x | 3.04x | 0.14x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.90 ns | 1.31 us | 4.47 us | 213.82 ns | 237.22 ns | 2.10 us | 37.43x | 6.11x | 5.51x | 0.62x |
| `mat3 zero` | 17.11 ns | 680.36 ns | 685.77 ns | 265.84 ns | 208.20 ns | 11.45 ns | 39.76x | 2.56x | 3.27x | 59.44x |
| `mat3 identity` | 10.21 ns | 780.40 ns | 790.06 ns | 335.01 ns | 242.22 ns | 156.83 ns | 76.45x | 2.33x | 3.22x | 4.98x |
| `mat3 transpose` | 9.25 ns | 895.58 ns | 865.13 ns | 231.77 ns | 213.01 ns | 116.16 ns | 96.81x | 3.86x | 4.20x | 7.71x |
| `mat3 reciprocal` | 82.09 ns | 18.34 us | 36.51 us | 2.99 us | 2.32 us | 83.92 us | 223.43x | 6.13x | 7.89x | 0.22x |
| `mat3 reciprocal checked` | 82.70 ns | 18.50 us | 36.50 us | 3.04 us | 2.32 us | 84.94 us | 223.63x | 6.08x | 7.99x | 0.22x |
| `mat3 inverse checked` | 81.78 ns | 18.39 us | 36.41 us | 3.06 us | 2.32 us | 83.71 us | 224.90x | 6.01x | 7.93x | 0.22x |
| `mat3 inverse checked abort` | 119.87 ns | 17.67 us | 37.26 us | 3.11 us | 2.28 us | 83.35 us | 147.43x | 5.68x | 7.73x | 0.21x |
| `mat3 powi` | 146.75 ns | 15.59 us | 87.22 us | 7.00 us | 6.23 us | 153.45 us | 106.23x | 2.23x | 2.50x | 0.10x |
| `mat3 powi checked` | 145.85 ns | 15.42 us | 85.74 us | 7.07 us | 6.23 us | 154.81 us | 105.75x | 2.18x | 2.48x | 0.10x |
| `mat3 powi checked abort` | 155.73 ns | 15.15 us | 85.69 us | 6.89 us | 6.25 us | 152.14 us | 97.27x | 2.20x | 2.43x | 0.10x |
| `mat3 div scalar checked` | 25.31 ns | 4.55 us | 6.03 us | 1.45 us | 816.05 ns | 22.18 us | 179.64x | 3.13x | 5.57x | 0.20x |
| `mat3 div scalar checked abort` | 30.25 ns | 4.68 us | 6.11 us | 1.48 us | 821.95 ns | 22.31 us | 154.80x | 3.15x | 5.70x | 0.21x |
| `mat3 div matrix checked` | 159.30 ns | 63.67 us | 62.11 us | 5.56 us | 4.50 us | 164.94 us | 399.69x | 11.44x | 14.16x | 0.39x |
| `mat3 div matrix checked abort` | 166.96 ns | 62.48 us | 63.25 us | 5.51 us | 4.48 us | 160.33 us | 374.19x | 11.34x | 13.94x | 0.39x |
| `mat3 add` | 14.63 ns | 3.79 us | 5.85 us | 540.24 ns | 496.50 ns | 11.83 us | 259.09x | 7.01x | 7.63x | 0.32x |
| `mat3 add scalar` | 12.21 ns | 2.76 us | 4.99 us | 795.98 ns | 722.91 ns | 12.13 us | 226.43x | 3.47x | 3.82x | 0.23x |
| `mat3 sub` | 14.15 ns | 3.83 us | 5.71 us | 566.46 ns | 540.34 ns | 22.07 us | 270.95x | 6.77x | 7.09x | 0.17x |
| `mat3 sub scalar` | 11.43 ns | 2.84 us | 4.67 us | 818.99 ns | 720.76 ns | 21.84 us | 248.30x | 3.47x | 3.94x | 0.13x |
| `mat3 neg` | 10.97 ns | 866.50 ns | 848.09 ns | 489.87 ns | 477.64 ns | 8.65 us | 79.01x | 1.77x | 1.81x | 0.10x |
| `mat3 mul scalar` | 13.71 ns | 2.06 us | 6.05 us | 765.50 ns | 679.96 ns | 12.67 us | 150.01x | 2.69x | 3.02x | 0.16x |
| `mat3 div scalar` | 25.35 ns | 4.65 us | 5.96 us | 1.49 us | 840.11 ns | 22.70 us | 183.63x | 3.13x | 5.54x | 0.21x |
| `mat3 div matrix` | 154.46 ns | 62.14 us | 65.18 us | 5.59 us | 4.43 us | 161.97 us | 402.31x | 11.12x | 14.02x | 0.38x |
| `mat3 bitxor` | 147.03 ns | 15.43 us | 88.44 us | 6.85 us | 6.30 us | 152.45 us | 104.92x | 2.25x | 2.45x | 0.10x |
| `mat4 zero` | 13.46 ns | 1.05 us | 1.05 us | 498.72 ns | 350.27 ns | 14.58 ns | 78.38x | 2.12x | 3.01x | 72.37x |
| `mat4 identity` | 10.15 ns | 1.23 us | 1.20 us | 570.44 ns | 419.09 ns | 237.49 ns | 121.51x | 2.16x | 2.94x | 5.19x |
| `mat4 transpose` | 9.83 ns | 1.51 us | 1.52 us | 432.81 ns | 372.03 ns | 179.91 ns | 153.55x | 3.49x | 4.06x | 8.39x |
| `mat4 reciprocal` | 146.69 ns | 34.28 us | 63.51 us | 10.96 us | 9.06 us | 336.43 us | 233.69x | 3.13x | 3.78x | 0.10x |
| `mat4 reciprocal checked` | 172.65 ns | 33.85 us | 64.59 us | 10.75 us | 8.93 us | 338.32 us | 196.06x | 3.15x | 3.79x | 0.10x |
| `mat4 powi` | 247.36 ns | 32.76 us | 111.76 us | 15.86 us | 14.18 us | 351.10 us | 132.43x | 2.07x | 2.31x | 0.09x |
| `mat4 powi checked` | 248.50 ns | 31.82 us | 110.17 us | 15.79 us | 14.54 us | 357.10 us | 128.03x | 2.01x | 2.19x | 0.09x |
| `mat4 add` | 52.29 ns | 6.37 us | 7.26 us | 960.30 ns | 848.35 ns | 20.16 us | 121.89x | 6.64x | 7.51x | 0.32x |
| `mat4 add scalar` | 20.80 ns | 4.77 us | 7.33 us | 1.45 us | 1.18 us | 21.28 us | 229.22x | 3.30x | 4.03x | 0.22x |
| `mat4 sub` | 38.51 ns | 6.31 us | 7.18 us | 1.00 us | 900.07 ns | 36.32 us | 163.94x | 6.31x | 7.01x | 0.17x |
| `mat4 sub scalar` | 15.50 ns | 4.70 us | 7.12 us | 1.45 us | 1.17 us | 37.65 us | 303.06x | 3.25x | 4.01x | 0.12x |
| `mat4 neg` | 13.83 ns | 1.48 us | 1.47 us | 910.42 ns | 767.82 ns | 14.28 us | 107.00x | 1.63x | 1.93x | 0.10x |
| `mat4 mul scalar` | 24.37 ns | 3.73 us | 7.98 us | 1.41 us | 1.17 us | 20.69 us | 152.98x | 2.64x | 3.18x | 0.18x |
| `mat4 div scalar` | 33.06 ns | 8.21 us | 8.25 us | 2.67 us | 1.38 us | 38.45 us | 248.48x | 3.07x | 5.97x | 0.21x |
| `mat4 div matrix` | 234.98 ns | 125.57 us | 98.17 us | 17.11 us | 14.48 us | 530.74 us | 534.39x | 7.34x | 8.67x | 0.24x |
| `mat4 bitxor` | 245.84 ns | 32.51 us | 112.74 us | 16.03 us | 14.20 us | 354.69 us | 132.24x | 2.03x | 2.29x | 0.09x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.73 ns | 197.53 ns | 297.97 ns | - | - | 1.30 us | 52.92x | - | - | 0.15x |
| `scalar add ref_owned` | 12.26 ns | 202.85 ns | 306.42 ns | - | - | 1.31 us | 16.54x | - | - | 0.16x |
| `scalar add refs` | 5.47 ns | 181.63 ns | 278.22 ns | - | - | 1.31 us | 33.19x | - | - | 0.14x |
| `scalar add owned_ref_with_clone` | 9.27 ns | 224.91 ns | 321.42 ns | - | - | - | 24.27x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.67 ns | 222.21 ns | 324.77 ns | - | - | - | 39.18x | - | - | - |
| `scalar sub owned_ref` | 3.89 ns | 208.19 ns | 363.87 ns | - | - | 2.43 us | 53.56x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.53 ns | 215.07 ns | 373.32 ns | - | - | 2.43 us | 17.16x | - | - | 0.09x |
| `scalar sub refs` | 5.69 ns | 187.27 ns | 338.90 ns | - | - | 2.45 us | 32.93x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.16 ns | 224.09 ns | 392.53 ns | - | - | - | 24.46x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.81 ns | 234.57 ns | 392.36 ns | - | - | - | 40.34x | - | - | - |
| `scalar mul owned_ref` | 4.66 ns | 117.53 ns | 407.22 ns | - | - | 1.55 us | 25.22x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.32 ns | 122.89 ns | 386.03 ns | - | - | 1.54 us | 9.23x | - | - | 0.08x |
| `scalar mul refs` | 6.31 ns | 103.22 ns | 370.33 ns | - | - | 1.56 us | 16.35x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 9.74 ns | 145.60 ns | 407.30 ns | - | - | - | 14.95x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.21 ns | 143.82 ns | 408.91 ns | - | - | - | 23.15x | - | - | - |
| `scalar div owned_ref` | 6.13 ns | 297.55 ns | 346.77 ns | - | - | 2.67 us | 48.52x | - | - | 0.11x |
| `scalar div ref_owned` | 17.56 ns | 301.03 ns | 359.23 ns | - | - | 2.61 us | 17.14x | - | - | 0.12x |
| `scalar div refs` | 7.07 ns | 265.37 ns | 318.87 ns | - | - | 2.60 us | 37.52x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.62 ns | 301.48 ns | 369.16 ns | - | - | - | 22.14x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.93 ns | 313.33 ns | 376.88 ns | - | - | - | 19.67x | - | - | - |
| `vec3 add refs` | 6.41 ns | 680.72 ns | 1.44 us | - | - | 4.05 us | 106.22x | - | - | 0.17x |
| `vec3 sub refs` | 6.43 ns | 706.49 ns | 1.53 us | - | - | 7.35 us | 109.82x | - | - | 0.10x |
| `vec3 neg ref` | 3.34 ns | 245.82 ns | 241.17 ns | - | - | 3.07 us | 73.55x | - | - | 0.08x |
| `vec3 add_scalar_ref` | 6.66 ns | 900.20 ns | 1.67 us | - | - | 3.93 us | 135.26x | - | - | 0.23x |
| `vec3 sub_scalar_ref` | 6.68 ns | 926.27 ns | 1.40 us | - | - | 7.17 us | 138.60x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 7.05 ns | 638.80 ns | 2.17 us | - | - | 4.46 us | 90.56x | - | - | 0.14x |
| `vec3 div_scalar_ref` | 8.30 ns | 1.71 us | 2.00 us | - | - | 7.75 us | 205.88x | - | - | 0.22x |
| `vec4 add refs` | 6.56 ns | 830.69 ns | 1.47 us | - | - | 5.41 us | 126.58x | - | - | 0.15x |
| `vec4 sub refs` | 3.13 ns | 813.38 ns | 1.36 us | - | - | 9.81 us | 259.54x | - | - | 0.08x |
| `vec4 neg ref` | 4.36 ns | 321.94 ns | 310.70 ns | - | - | 4.06 us | 73.78x | - | - | 0.08x |
| `vec4 add_scalar_ref` | 7.13 ns | 1.51 us | 2.18 us | - | - | 5.19 us | 211.66x | - | - | 0.29x |
| `vec4 sub_scalar_ref` | 4.28 ns | 1.51 us | 1.94 us | - | - | 9.40 us | 353.77x | - | - | 0.16x |
| `vec4 mul_scalar_ref` | 7.58 ns | 1.15 us | 2.52 us | - | - | 5.78 us | 151.84x | - | - | 0.20x |
| `vec4 div_scalar_ref` | 12.84 ns | 2.14 us | 1.96 us | - | - | 10.60 us | 166.50x | - | - | 0.20x |
| `mat3 add refs` | 11.93 ns | 2.16 us | 4.31 us | - | - | 11.69 us | 181.16x | - | - | 0.18x |
| `mat3 sub refs` | 10.99 ns | 2.24 us | 4.37 us | - | - | 21.60 us | 204.12x | - | - | 0.10x |
| `mat3 mul refs` | 33.90 ns | 6.06 us | 29.34 us | - | - | 62.70 us | 178.92x | - | - | 0.10x |
| `mat3 div refs` | 137.68 ns | 62.19 us | 61.66 us | - | - | 161.66 us | 451.70x | - | - | 0.38x |
| `mat3 neg ref` | 9.92 ns | 963.68 ns | 973.04 ns | - | - | 8.87 us | 97.16x | - | - | 0.11x |
| `mat3 add_scalar_ref` | 44.80 ns | 3.09 us | 5.17 us | - | - | 12.22 us | 68.86x | - | - | 0.25x |
| `mat3 sub_scalar_ref` | 44.74 ns | 3.07 us | 5.06 us | - | - | 21.72 us | 68.52x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 48.87 ns | 2.45 us | 6.30 us | - | - | 12.53 us | 50.12x | - | - | 0.20x |
| `mat3 div_scalar_ref` | 23.27 ns | 4.90 us | 6.61 us | - | - | 23.06 us | 210.71x | - | - | 0.21x |
| `mat4 add refs` | 17.17 ns | 3.34 us | 4.23 us | - | - | 19.88 us | 194.46x | - | - | 0.17x |
| `mat4 sub refs` | 16.71 ns | 3.30 us | 4.29 us | - | - | 36.61 us | 197.69x | - | - | 0.09x |
| `mat4 mul refs` | 72.22 ns | 12.54 us | 29.13 us | - | - | 141.74 us | 173.69x | - | - | 0.09x |
| `mat4 div refs` | 181.47 ns | 122.73 us | 96.44 us | - | - | 532.46 us | 676.30x | - | - | 0.23x |
| `mat4 neg ref` | 12.20 ns | 1.51 us | 1.50 us | - | - | 14.06 us | 123.50x | - | - | 0.11x |
| `mat4 add_scalar_ref` | 49.68 ns | 5.31 us | 7.72 us | - | - | 21.56 us | 106.96x | - | - | 0.25x |
| `mat4 sub_scalar_ref` | 37.42 ns | 5.39 us | 7.51 us | - | - | 37.61 us | 144.06x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 53.58 ns | 4.14 us | 8.26 us | - | - | 20.46 us | 77.36x | - | - | 0.20x |
| `mat4 div_scalar_ref` | 27.65 ns | 8.49 us | 8.70 us | - | - | 39.52 us | 306.93x | - | - | 0.21x |
| `mat3 transform_vec refs` | 14.72 ns | 2.55 us | 11.72 us | - | - | 20.64 us | 173.13x | - | - | 0.12x |
| `mat4 transform_vec refs` | 23.52 ns | 4.14 us | 12.41 us | - | - | 35.90 us | 175.97x | - | - | 0.12x |
| `complex add refs` | 7.99 ns | 360.93 ns | 743.43 ns | - | - | 2.62 us | 45.18x | - | - | 0.14x |
| `complex sub refs` | 8.21 ns | 368.22 ns | 758.71 ns | - | - | 4.85 us | 44.86x | - | - | 0.08x |
| `complex mul refs` | 8.37 ns | 869.19 ns | 3.20 us | - | - | 10.28 us | 103.84x | - | - | 0.08x |
| `complex div refs` | 18.99 ns | 3.18 us | 6.58 us | - | - | 21.70 us | 167.19x | - | - | 0.15x |
| `complex neg ref` | 2.42 ns | 112.00 ns | 113.08 ns | - | - | 2.11 us | 46.25x | - | - | 0.05x |
| `complex div_real_ref` | 7.51 ns | 733.46 ns | 679.82 ns | - | - | 5.28 us | 97.62x | - | - | 0.14x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.43 us |
| `astro sin 160` | 14.44 us |
| `astro sin 192` | 14.16 us |
| `astro sin 256` | 16.36 us |
