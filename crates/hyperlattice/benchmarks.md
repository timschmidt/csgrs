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
| `sin 0.1` | 10.91 ns | 217.18 ns | 212.82 ns | 11.09 us | 784.47 ns | 1.90 us | 19.91x | 0.02x | 0.28x | 0.11x |
| `cos 0.1` | 11.64 ns | 217.05 ns | 214.69 ns | 10.64 us | 500.72 ns | 1.76 us | 18.64x | 0.02x | 0.43x | 0.12x |
| `sin 1.23456789` | 11.76 ns | 618.43 ns | 634.18 ns | 12.81 us | 807.82 ns | 1.88 us | 52.59x | 0.05x | 0.77x | 0.33x |
| `cos 1.23456789` | 11.98 ns | 543.03 ns | 519.08 ns | 10.79 us | 587.08 ns | 1.73 us | 45.33x | 0.05x | 0.92x | 0.31x |
| `sin 1e6` | 12.39 ns | 2.58 us | 2.58 us | 16.18 us | 1.08 us | 2.06 us | 208.12x | 0.16x | 2.38x | 1.25x |
| `cos 1e6` | 12.43 ns | 2.20 us | 2.19 us | 13.58 us | 839.18 ns | 1.85 us | 176.68x | 0.16x | 2.62x | 1.19x |
| `sin 1e30` | 66.81 ns | 2.86 us | 2.83 us | 18.39 us | 2.86 us | 3.65 us | 42.73x | 0.16x | 1.00x | 0.78x |
| `cos 1e30` | 72.26 ns | 2.46 us | 2.41 us | 15.37 us | 981.13 ns | 3.18 us | 34.04x | 0.16x | 2.51x | 0.77x |
| `sin pi_7` | 11.74 ns | 216.35 ns | 514.92 ns | 12.25 us | 755.04 ns | 1.92 us | 18.43x | 0.02x | 0.29x | 0.11x |
| `cos pi_7` | 11.71 ns | 218.10 ns | 975.50 ns | 11.06 us | 533.53 ns | 1.76 us | 18.63x | 0.02x | 0.41x | 0.12x |
| `sin 1000pi_eps` | 11.73 ns | 3.09 us | 3.99 us | 16.24 us | 2.29 us | 2.96 us | 263.16x | 0.19x | 1.35x | 1.04x |
| `cos 1000pi_eps` | 12.30 ns | 2.66 us | 3.59 us | 13.58 us | 595.57 ns | 1.73 us | 216.13x | 0.20x | 4.46x | 1.53x |
| `asin 0.5` | 11.09 ns | 560.24 ns | 559.65 ns | 49.35 us | 2.90 us | 13.53 us | 50.50x | 0.01x | 0.19x | 0.04x |
| `acos 0.5` | 11.23 ns | 1.30 us | 1.28 us | 58.77 us | 2.90 us | 13.60 us | 115.59x | 0.02x | 0.45x | 0.10x |
| `atanh 0.5` | 14.49 ns | 1.67 us | 1.65 us | 35.02 us | 1.71 us | 13.01 us | 115.01x | 0.05x | 0.97x | 0.13x |
| `asin neg_0.999999` | 14.46 ns | 9.03 us | 6.41 us | 13.79 us | 2.51 us | 12.94 us | 624.65x | 0.65x | 3.59x | 0.70x |
| `acos neg_0.999999` | 15.18 ns | 12.41 us | 9.10 us | 18.38 us | 2.77 us | 13.17 us | 817.92x | 0.68x | 4.48x | 0.94x |
| `atanh neg_0.999999` | 14.32 ns | 4.82 us | 3.71 us | 37.06 us | 1.64 us | 12.85 us | 336.34x | 0.13x | 2.94x | 0.37x |
| `asin 0.999999` | 13.92 ns | 8.93 us | 6.10 us | 13.74 us | 2.55 us | 12.91 us | 641.45x | 0.65x | 3.50x | 0.69x |
| `acos 0.999999` | 14.16 ns | 11.89 us | 8.29 us | 18.34 us | 2.79 us | 13.09 us | 839.51x | 0.65x | 4.26x | 0.91x |
| `atanh 0.999999` | 14.56 ns | 4.80 us | 3.58 us | 32.20 us | 1.69 us | 12.82 us | 330.00x | 0.15x | 2.85x | 0.37x |
| `asin 1e-12` | 9.33 ns | 9.72 us | 5.26 us | 8.04 us | 1.43 us | 15.23 us | 1041.60x | 1.21x | 6.79x | 0.64x |
| `acos 1e-12` | 9.97 ns | 12.01 us | 7.15 us | 9.52 us | 1.45 us | 15.24 us | 1204.14x | 1.26x | 8.29x | 0.79x |
| `atanh 1e-12` | 9.60 ns | 6.19 us | 3.33 us | 36.94 us | 171.36 ns | 20.23 us | 644.86x | 0.17x | 36.11x | 0.31x |
| `atan 0.5` | 14.77 ns | 521.10 ns | 519.46 ns | 35.02 us | 2.77 us | 17.72 us | 35.28x | 0.01x | 0.19x | 0.03x |
| `asinh 0.5` | 26.99 ns | 1.80 us | 1.77 us | 40.03 us | 1.65 us | 7.56 us | 66.76x | 0.05x | 1.09x | 0.24x |
| `atan neg_1e-12` | 14.19 ns | 798.25 ns | 691.72 ns | 1.55 us | 1.14 us | 15.25 us | 56.26x | 0.51x | 0.70x | 0.05x |
| `asinh neg_1e-12` | 16.03 ns | 7.55 us | 4.45 us | 43.07 us | 8.76 us | 12.26 us | 470.91x | 0.18x | 0.86x | 0.62x |
| `atan 1e6` | 15.29 ns | 605.14 ns | 595.33 ns | 2.81 us | 1.46 us | 17.95 us | 39.58x | 0.22x | 0.42x | 0.03x |
| `asinh 1e6` | 26.51 ns | 3.26 us | 3.26 us | 37.48 us | 1.68 us | 7.21 us | 123.03x | 0.09x | 1.94x | 0.45x |
| `atan neg_1e6` | 15.22 ns | 684.04 ns | 680.09 ns | 2.82 us | 1.44 us | 17.94 us | 44.94x | 0.24x | 0.47x | 0.04x |
| `asinh neg_1e6` | 26.99 ns | 3.28 us | 3.26 us | 38.29 us | 1.69 us | 7.10 us | 121.58x | 0.09x | 1.94x | 0.46x |
| `acosh 9` | 12.45 ns | 2.85 us | 2.83 us | 42.91 us | 1.68 us | 10.01 us | 229.26x | 0.07x | 1.70x | 0.29x |
| `acosh 1_plus_1e-12` | 11.51 ns | 4.24 us | 5.06 us | 42.05 us | 8.62 us | 11.48 us | 368.45x | 0.10x | 0.49x | 0.37x |
| `acosh 1e6` | 12.50 ns | 3.61 us | 3.57 us | 36.88 us | 1.66 us | 9.90 us | 289.28x | 0.10x | 2.18x | 0.37x |
| `acosh e` | 12.48 ns | 5.77 us | 1.18 us | 40.88 us | 1.71 us | 9.91 us | 462.49x | 0.14x | 3.38x | 0.58x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 57.76 ns | 54.52 ns | 26.63 ns | 15.77 ns | 0.98 ns | 120.78x | 2.17x | 3.66x | 58.71x |
| `one` | 0.47 ns | 67.38 ns | 62.10 ns | 41.23 ns | 30.80 ns | 30.63 ns | 142.25x | 1.63x | 2.19x | 2.20x |
| `e` | 0.47 ns | 79.08 ns | 70.59 ns | 58.81 ns | 1.08 us | 230.94 ns | 166.65x | 1.34x | 0.07x | 0.34x |
| `pi` | 0.48 ns | 59.85 ns | 56.49 ns | 47.25 ns | 49.78 ns | 233.10 ns | 124.30x | 1.27x | 1.20x | 0.26x |
| `tau` | 0.48 ns | 200.08 ns | 184.65 ns | 119.44 ns | 102.15 ns | 1.92 us | 419.31x | 1.68x | 1.96x | 0.10x |
| `add` | 5.20 ns | 348.00 ns | 391.01 ns | 53.40 ns | 43.43 ns | 1.34 us | 66.89x | 6.52x | 8.01x | 0.26x |
| `sub` | 5.19 ns | 431.18 ns | 447.68 ns | 55.54 ns | 46.52 ns | 2.50 us | 83.08x | 7.76x | 9.27x | 0.17x |
| `neg` | 5.10 ns | 67.45 ns | 64.34 ns | 23.52 ns | 21.92 ns | 1.10 us | 13.24x | 2.87x | 3.08x | 0.06x |
| `mul` | 5.52 ns | 481.94 ns | 471.30 ns | 58.53 ns | 45.85 ns | 1.55 us | 87.38x | 8.23x | 10.51x | 0.31x |
| `div` | 7.86 ns | 420.77 ns | 435.43 ns | 138.93 ns | 63.69 ns | 2.64 us | 53.55x | 3.03x | 6.61x | 0.16x |
| `reciprocal` | 8.74 ns | 119.08 ns | 111.75 ns | 161.39 ns | 59.84 ns | 1.64 us | 13.62x | 0.74x | 1.99x | 0.07x |
| `reciprocal checked` | 9.23 ns | 123.15 ns | 115.14 ns | 161.97 ns | 60.21 ns | 1.61 us | 13.34x | 0.76x | 2.05x | 0.08x |
| `reciprocal checked abort` | 27.18 ns | 207.96 ns | 189.51 ns | 162.92 ns | 60.72 ns | 1.59 us | 7.65x | 1.28x | 3.42x | 0.13x |
| `pow` | 31.35 ns | 12.57 us | 10.96 us | 56.12 us | 3.03 us | 2.47 us | 401.05x | 0.22x | 4.15x | 5.09x |
| `powi` | 5.84 ns | 2.53 us | 2.83 us | 281.16 ns | 87.58 ns | 1.64 us | 433.67x | 9.00x | 28.90x | 1.54x |
| `exp` | 10.34 ns | 1.82 us | 1.82 us | 14.29 us | 941.98 ns | 1.95 us | 175.67x | 0.13x | 1.93x | 0.93x |
| `ln` | 10.96 ns | 1.61 us | 1.52 us | 29.45 us | 1.35 us | 1.92 us | 146.66x | 0.05x | 1.19x | 0.84x |
| `log10` | 13.47 ns | 2.89 us | 2.32 us | 35.37 us | 2.83 us | 6.92 us | 214.86x | 0.08x | 1.02x | 0.42x |
| `log10 abort` | 17.03 ns | 2.86 us | 2.31 us | 35.78 us | 2.82 us | 6.89 us | 168.17x | 0.08x | 1.02x | 0.42x |
| `sqrt` | 20.70 ns | 1.78 us | 1.71 us | 5.16 us | 101.34 ns | 1.51 us | 86.20x | 0.35x | 17.61x | 1.18x |
| `sin` | 15.06 ns | 1.56 us | 1.63 us | 14.30 us | 1.27 us | 2.30 us | 103.24x | 0.11x | 1.22x | 0.67x |
| `cos` | 18.37 ns | 1.34 us | 1.43 us | 12.14 us | 651.14 ns | 1.79 us | 73.19x | 0.11x | 2.06x | 0.75x |
| `tan` | 24.62 ns | 1.42 us | 1.49 us | 29.46 us | 1.62 us | 6.78 us | 57.56x | 0.05x | 0.87x | 0.21x |
| `sinh` | 18.28 ns | 3.37 us | 3.35 us | 3.42 us | 1.16 us | 11.05 us | 184.59x | 0.99x | 2.91x | 0.31x |
| `cosh` | 18.26 ns | 3.30 us | 3.35 us | 7.96 us | 1.08 us | 9.89 us | 180.75x | 0.41x | 3.04x | 0.33x |
| `tanh` | 23.39 ns | 4.87 us | 4.85 us | 3.40 us | 1.21 us | 23.76 us | 208.35x | 1.43x | 4.04x | 0.21x |
| `asin` | 13.32 ns | 7.08 us | 6.32 us | 21.65 us | 2.50 us | 14.15 us | 531.49x | 0.33x | 2.84x | 0.50x |
| `asin abort` | 17.15 ns | 7.13 us | 6.43 us | 21.20 us | 2.50 us | 14.27 us | 415.80x | 0.34x | 2.85x | 0.50x |
| `acos` | 13.69 ns | 9.64 us | 8.81 us | 26.42 us | 2.56 us | 14.22 us | 704.29x | 0.36x | 3.77x | 0.68x |
| `acos abort` | 17.84 ns | 9.59 us | 8.83 us | 26.76 us | 2.57 us | 14.77 us | 537.88x | 0.36x | 3.73x | 0.65x |
| `atan` | 15.24 ns | 961.37 ns | 1.27 us | 18.53 us | 2.35 us | 19.33 us | 63.08x | 0.05x | 0.41x | 0.05x |
| `atan abort` | 21.32 ns | 966.67 ns | 1.27 us | 18.46 us | 2.35 us | 19.63 us | 45.34x | 0.05x | 0.41x | 0.05x |
| `asinh` | 41.73 ns | 4.51 us | 5.42 us | 40.24 us | 1.70 us | 7.83 us | 108.02x | 0.11x | 2.66x | 0.58x |
| `asinh abort` | 31.39 ns | 4.53 us | 5.46 us | 39.95 us | 1.69 us | 7.82 us | 144.36x | 0.11x | 2.68x | 0.58x |
| `acosh` | 12.59 ns | 4.24 us | 5.32 us | 41.89 us | 3.43 us | 11.70 us | 337.23x | 0.10x | 1.24x | 0.36x |
| `acosh abort` | 15.55 ns | 4.22 us | 5.25 us | 41.08 us | 3.41 us | 10.89 us | 271.43x | 0.10x | 1.24x | 0.39x |
| `atanh` | 13.79 ns | 4.36 us | 4.12 us | 35.35 us | 1.31 us | 15.59 us | 316.42x | 0.12x | 3.34x | 0.28x |
| `atanh abort` | 17.24 ns | 4.44 us | 4.12 us | 36.29 us | 1.34 us | 15.83 us | 257.80x | 0.12x | 3.32x | 0.28x |
| `zero status` | 1.24 ns | 1.77 ns | 1.77 ns | 1.02 ns | 6.86 ns | 8.29 ns | 1.43x | 1.73x | 0.26x | 0.21x |
| `zero status abort` | 3.37 ns | 61.08 ns | 62.27 ns | 1.02 ns | 6.91 ns | 8.21 ns | 18.10x | 59.72x | 8.84x | 7.44x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.98 ns | 119.08 ns | 120.54 ns | 51.32 ns | - | 1.91 ns | 121.73x | 2.32x | - | 62.35x |
| `one` | 5.48 ns | 121.54 ns | 119.65 ns | 67.55 ns | - | 33.89 ns | 22.16x | 1.80x | - | 3.59x |
| `i` | 0.96 ns | 130.67 ns | 126.40 ns | 68.51 ns | - | 30.20 ns | 135.61x | 1.91x | - | 4.33x |
| `free i` | 0.97 ns | 127.28 ns | 130.11 ns | 68.87 ns | - | 30.06 ns | 131.88x | 1.85x | - | 4.23x |
| `conjugate` | 2.19 ns | 133.71 ns | 132.04 ns | 41.66 ns | - | 1.10 us | 61.11x | 3.21x | - | 0.12x |
| `norm squared` | 5.98 ns | 1.49 us | 1.59 us | 163.62 ns | - | 4.48 us | 248.40x | 9.08x | - | 0.33x |
| `reciprocal` | 18.62 ns | 3.02 us | 3.14 us | 466.76 ns | - | 11.06 us | 162.43x | 6.48x | - | 0.27x |
| `reciprocal checked` | 14.79 ns | 3.03 us | 3.07 us | 462.27 ns | - | 10.96 us | 204.61x | 6.55x | - | 0.28x |
| `powi` | 19.73 ns | 10.75 us | 14.45 us | 1.49 us | - | 44.91 us | 545.01x | 7.20x | - | 0.24x |
| `powi checked` | 19.70 ns | 10.52 us | 14.42 us | 1.49 us | - | 45.13 us | 533.90x | 7.06x | - | 0.23x |
| `div checked` | 20.76 ns | 6.16 us | 6.82 us | 791.61 ns | - | 22.55 us | 296.66x | 7.78x | - | 0.27x |
| `div real checked` | 9.66 ns | 845.02 ns | 827.18 ns | 269.21 ns | - | 5.38 us | 87.43x | 3.14x | - | 0.16x |
| `from scalar` | 1.46 ns | 122.57 ns | 121.31 ns | 45.67 ns | - | 10.68 ns | 83.68x | 2.68x | - | 11.47x |
| `add` | 6.04 ns | 928.17 ns | 1.01 us | 108.38 ns | - | 2.70 us | 153.56x | 8.56x | - | 0.34x |
| `sub` | 6.02 ns | 1.00 us | 1.05 us | 121.18 ns | - | 5.01 us | 166.78x | 8.29x | - | 0.20x |
| `neg` | 2.69 ns | 134.44 ns | 133.69 ns | 43.03 ns | - | 2.24 us | 49.97x | 3.12x | - | 0.06x |
| `mul` | 11.46 ns | 3.15 us | 3.56 us | 321.08 ns | - | 10.27 us | 274.59x | 9.80x | - | 0.31x |
| `div` | 18.15 ns | 6.29 us | 6.65 us | 813.07 ns | - | 22.39 us | 346.54x | 7.74x | - | 0.28x |
| `div real` | 10.63 ns | 797.43 ns | 773.01 ns | 274.55 ns | - | 5.84 us | 75.01x | 2.90x | - | 0.14x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.75 ns | 3.41 us | 6.09 us | 315.13 ns | 253.38 ns | 7.28 us | 505.68x | 10.83x | 13.48x | 0.47x |
| `vec3 magnitude` | 9.65 ns | 7.41 us | 10.02 us | 5.52 us | 343.22 ns | 8.90 us | 768.09x | 1.34x | 21.59x | 0.83x |
| `vec3 normalize` | 25.10 ns | 12.13 us | 12.31 us | 6.09 us | 602.12 ns | 16.98 us | 483.36x | 1.99x | 20.15x | 0.71x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.16 ns | 1.15 us | 1.63 us | 70.47 ns | 58.62 ns | 744.77 ns | 365.92x | 16.38x | 19.70x | 1.55x |
| `vec3 zero` | 1.46 ns | 246.37 ns | 248.38 ns | 62.59 ns | 33.23 ns | 2.91 ns | 168.91x | 3.94x | 7.41x | 84.59x |
| `vec3 dot abort` | 28.49 ns | 4.04 us | 4.75 us | 272.83 ns | 214.75 ns | 7.41 us | 141.75x | 14.80x | 18.81x | 0.54x |
| `vec3 magnitude abort` | 40.35 ns | 8.17 us | 7.40 us | 5.56 us | 336.66 ns | 9.21 us | 202.35x | 1.47x | 24.25x | 0.89x |
| `vec3 normalize checked` | 26.43 ns | 12.37 us | 9.56 us | 6.31 us | 561.63 ns | 17.65 us | 468.04x | 1.96x | 22.02x | 0.70x |
| `vec3 normalize checked abort` | 56.78 ns | 12.84 us | 9.93 us | 6.26 us | 570.02 ns | 17.33 us | 226.22x | 2.05x | 22.53x | 0.74x |
| `vec3 div scalar checked` | 16.81 ns | 1.83 us | 1.80 us | 408.45 ns | - | - | 108.74x | 4.47x | - | - |
| `vec3 div scalar checked abort` | 18.41 ns | 1.85 us | 1.79 us | 402.56 ns | - | - | 100.26x | 4.59x | - | - |
| `vec3 add` | 7.51 ns | 2.02 us | 2.06 us | 156.31 ns | 134.03 ns | 4.21 us | 268.87x | 12.91x | 15.06x | 0.48x |
| `vec3 add scalar` | 6.58 ns | 1.57 us | 1.70 us | 158.88 ns | 140.34 ns | 4.02 us | 238.31x | 9.87x | 11.17x | 0.39x |
| `vec3 sub` | 7.46 ns | 2.05 us | 2.14 us | 176.41 ns | 146.09 ns | 7.64 us | 275.12x | 11.63x | 14.04x | 0.27x |
| `vec3 sub scalar` | 6.51 ns | 1.39 us | 1.46 us | 152.33 ns | 133.45 ns | 7.45 us | 213.70x | 9.13x | 10.42x | 0.19x |
| `vec3 neg` | 3.83 ns | 227.60 ns | 225.84 ns | 59.77 ns | 51.04 ns | 3.22 us | 59.40x | 3.81x | 4.46x | 0.07x |
| `vec3 mul scalar` | 6.97 ns | 2.03 us | 2.23 us | 169.86 ns | 128.88 ns | 4.46 us | 291.21x | 11.95x | 15.75x | 0.45x |
| `vec3 div scalar` | 18.98 ns | 1.80 us | 1.80 us | 405.61 ns | 182.73 ns | 7.98 us | 94.78x | 4.44x | 9.85x | 0.23x |
| `vec4 dot` | 7.36 ns | 3.60 us | 3.64 us | 442.36 ns | 332.75 ns | 9.92 us | 489.57x | 8.14x | 10.82x | 0.36x |
| `vec4 magnitude` | 12.87 ns | 7.34 us | 6.19 us | 5.68 us | 437.35 ns | 11.67 us | 570.19x | 1.29x | 16.78x | 0.63x |
| `vec4 normalize` | 37.45 ns | 12.19 us | 7.97 us | 6.45 us | 746.70 ns | 22.96 us | 325.41x | 1.89x | 16.32x | 0.53x |
| `vec4 add` | 7.50 ns | 2.28 us | 2.14 us | 209.36 ns | 183.31 ns | 5.45 us | 304.10x | 10.89x | 12.44x | 0.42x |
| `vec4 add scalar` | 6.99 ns | 2.13 us | 2.04 us | 226.23 ns | 186.65 ns | 5.29 us | 304.58x | 9.41x | 11.41x | 0.40x |
| `vec4 sub` | 5.35 ns | 2.19 us | 2.08 us | 226.98 ns | 181.18 ns | 9.93 us | 409.46x | 9.65x | 12.09x | 0.22x |
| `vec4 sub scalar` | 4.59 ns | 1.90 us | 1.80 us | 211.43 ns | 178.16 ns | 9.94 us | 413.36x | 8.97x | 10.65x | 0.19x |
| `vec4 neg` | 5.02 ns | 327.20 ns | 323.71 ns | 79.98 ns | 65.85 ns | 4.20 us | 65.18x | 4.09x | 4.97x | 0.08x |
| `vec4 mul scalar` | 7.49 ns | 2.31 us | 2.34 us | 226.07 ns | 164.93 ns | 5.83 us | 308.52x | 10.22x | 14.01x | 0.40x |
| `vec4 div scalar` | 14.45 ns | 2.06 us | 1.85 us | 543.79 ns | 238.21 ns | 10.28 us | 142.71x | 3.79x | 8.66x | 0.20x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.42 ns | 7.30 us | 3.53 us | 990.64 ns | 872.67 ns | 22.63 us | 587.71x | 7.37x | 8.36x | 0.32x |
| `mat3 inverse` | 79.61 ns | 29.47 us | 11.76 us | 3.17 us | 2.55 us | 83.89 us | 370.23x | 9.29x | 11.57x | 0.35x |
| `mat3 mul mat3` | 71.31 ns | 20.24 us | 12.45 us | 2.81 us | 2.38 us | 61.74 us | 283.89x | 7.21x | 8.50x | 0.33x |
| `mat3 transform vec3` | 16.24 ns | 10.11 us | 11.13 us | 1.02 us | 886.44 ns | 20.65 us | 622.53x | 9.91x | 11.41x | 0.49x |
| `mat4 determinant` | 45.26 ns | 12.95 us | 3.95 us | 4.59 us | 4.18 us | 96.56 us | 286.14x | 2.82x | 3.10x | 0.13x |
| `mat4 inverse` | 164.44 ns | 62.96 us | 16.23 us | 11.30 us | 9.37 us | 347.11 us | 382.89x | 5.57x | 6.72x | 0.18x |
| `mat4 mul mat4` | 118.46 ns | 24.94 us | 13.84 us | 6.57 us | 5.45 us | 143.36 us | 210.56x | 3.80x | 4.58x | 0.17x |
| `mat4 transform vec4` | 25.17 ns | 12.90 us | 4.30 us | 1.94 us | 1.69 us | 35.95 us | 512.57x | 6.65x | 7.64x | 0.36x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 36.67 ns | 2.89 us | 4.48 us | 224.66 ns | 248.75 ns | 2.20 us | 78.93x | 12.89x | 11.64x | 1.32x |
| `mat3 zero` | 16.64 ns | 869.54 ns | 842.33 ns | 268.56 ns | 206.58 ns | 12.21 ns | 52.26x | 3.24x | 4.21x | 71.24x |
| `mat3 identity` | 10.15 ns | 973.81 ns | 966.68 ns | 330.36 ns | 239.05 ns | 153.77 ns | 95.92x | 2.95x | 4.07x | 6.33x |
| `mat3 transpose` | 9.26 ns | 971.09 ns | 966.57 ns | 247.99 ns | 204.68 ns | 140.81 ns | 104.85x | 3.92x | 4.74x | 6.90x |
| `mat3 reciprocal` | 80.80 ns | 30.02 us | 37.20 us | 3.06 us | 2.65 us | 86.44 us | 371.47x | 9.82x | 11.33x | 0.35x |
| `mat3 reciprocal checked` | 80.06 ns | 29.93 us | 37.02 us | 3.02 us | 2.70 us | 85.12 us | 373.85x | 9.90x | 11.08x | 0.35x |
| `mat3 inverse checked` | 79.87 ns | 29.84 us | 37.87 us | 3.03 us | 2.68 us | 85.32 us | 373.59x | 9.85x | 11.15x | 0.35x |
| `mat3 inverse checked abort` | 117.61 ns | 30.40 us | 37.30 us | 3.02 us | 2.68 us | 85.99 us | 258.48x | 10.08x | 11.35x | 0.35x |
| `mat3 powi` | 150.20 ns | 62.67 us | 91.53 us | 7.02 us | 7.73 us | 157.26 us | 417.23x | 8.93x | 8.10x | 0.40x |
| `mat3 powi checked` | 148.14 ns | 62.47 us | 91.80 us | 6.96 us | 7.88 us | 157.30 us | 421.67x | 8.98x | 7.92x | 0.40x |
| `mat3 powi checked abort` | 148.01 ns | 62.57 us | 91.55 us | 6.87 us | 7.87 us | 158.47 us | 422.72x | 9.10x | 7.95x | 0.39x |
| `mat3 div scalar checked` | 25.21 ns | 5.66 us | 6.23 us | 1.50 us | 955.35 ns | 22.91 us | 224.74x | 3.78x | 5.93x | 0.25x |
| `mat3 div scalar checked abort` | 30.82 ns | 5.91 us | 6.47 us | 1.52 us | 981.57 ns | 22.89 us | 191.63x | 3.88x | 6.02x | 0.26x |
| `mat3 div matrix checked` | 154.20 ns | 70.82 us | 67.66 us | 5.63 us | 5.41 us | 168.14 us | 459.29x | 12.59x | 13.09x | 0.42x |
| `mat3 div matrix checked abort` | 171.44 ns | 71.44 us | 69.09 us | 5.57 us | 5.34 us | 166.64 us | 416.73x | 12.82x | 13.38x | 0.43x |
| `mat3 add` | 14.95 ns | 5.45 us | 6.25 us | 526.42 ns | 525.13 ns | 11.91 us | 364.48x | 10.35x | 10.37x | 0.46x |
| `mat3 add scalar` | 12.22 ns | 3.77 us | 5.28 us | 796.14 ns | 855.85 ns | 12.51 us | 308.15x | 4.73x | 4.40x | 0.30x |
| `mat3 sub` | 13.33 ns | 5.59 us | 6.22 us | 550.08 ns | 554.39 ns | 22.14 us | 419.05x | 10.15x | 10.08x | 0.25x |
| `mat3 sub scalar` | 11.01 ns | 3.67 us | 5.13 us | 779.72 ns | 847.59 ns | 22.94 us | 333.57x | 4.71x | 4.33x | 0.16x |
| `mat3 neg` | 10.49 ns | 942.57 ns | 953.54 ns | 473.60 ns | 513.74 ns | 9.04 us | 89.87x | 1.99x | 1.83x | 0.10x |
| `mat3 mul scalar` | 14.04 ns | 5.36 us | 6.31 us | 790.12 ns | 820.72 ns | 12.90 us | 381.79x | 6.78x | 6.53x | 0.42x |
| `mat3 div scalar` | 25.35 ns | 5.48 us | 6.07 us | 1.49 us | 958.79 ns | 23.15 us | 216.09x | 3.67x | 5.71x | 0.24x |
| `mat3 div matrix` | 151.56 ns | 68.68 us | 67.59 us | 5.54 us | 5.37 us | 165.65 us | 453.17x | 12.39x | 12.79x | 0.41x |
| `mat3 bitxor` | 148.11 ns | 61.49 us | 93.21 us | 6.86 us | 7.86 us | 160.59 us | 415.16x | 8.96x | 7.82x | 0.38x |
| `mat4 zero` | 11.81 ns | 1.52 us | 1.54 us | 473.55 ns | 392.19 ns | 15.17 ns | 128.63x | 3.21x | 3.87x | 100.11x |
| `mat4 identity` | 11.21 ns | 1.73 us | 1.76 us | 587.05 ns | 480.48 ns | 220.67 ns | 153.90x | 2.94x | 3.59x | 7.82x |
| `mat4 transpose` | 9.42 ns | 1.58 us | 1.59 us | 444.88 ns | 434.88 ns | 183.66 ns | 168.25x | 3.56x | 3.64x | 8.63x |
| `mat4 reciprocal` | 168.02 ns | 62.93 us | 65.04 us | 10.90 us | 11.12 us | 364.59 us | 374.52x | 5.77x | 5.66x | 0.17x |
| `mat4 reciprocal checked` | 174.52 ns | 63.13 us | 65.30 us | 11.02 us | 11.19 us | 361.44 us | 361.72x | 5.73x | 5.64x | 0.17x |
| `mat4 powi` | 247.63 ns | 93.11 us | 114.59 us | 16.05 us | 16.94 us | 363.93 us | 376.02x | 5.80x | 5.50x | 0.26x |
| `mat4 powi checked` | 250.38 ns | 92.26 us | 114.34 us | 16.06 us | 16.86 us | 374.57 us | 368.47x | 5.74x | 5.47x | 0.25x |
| `mat4 add` | 53.00 ns | 7.06 us | 7.46 us | 947.31 ns | 956.47 ns | 20.51 us | 133.25x | 7.45x | 7.38x | 0.34x |
| `mat4 add scalar` | 21.29 ns | 5.94 us | 7.84 us | 1.46 us | 1.45 us | 21.25 us | 278.97x | 4.07x | 4.10x | 0.28x |
| `mat4 sub` | 41.28 ns | 7.32 us | 7.47 us | 982.22 ns | 1.03 us | 37.93 us | 177.32x | 7.45x | 7.08x | 0.19x |
| `mat4 sub scalar` | 15.58 ns | 5.93 us | 7.59 us | 1.47 us | 1.45 us | 39.57 us | 380.59x | 4.04x | 4.10x | 0.15x |
| `mat4 neg` | 14.82 ns | 1.55 us | 1.56 us | 916.74 ns | 890.67 ns | 14.48 us | 104.57x | 1.69x | 1.74x | 0.11x |
| `mat4 mul scalar` | 25.02 ns | 7.71 us | 8.56 us | 1.43 us | 1.33 us | 20.76 us | 308.29x | 5.39x | 5.79x | 0.37x |
| `mat4 div scalar` | 34.38 ns | 8.56 us | 8.83 us | 2.60 us | 1.63 us | 39.91 us | 248.92x | 3.29x | 5.25x | 0.21x |
| `mat4 div matrix` | 229.79 ns | 137.80 us | 113.19 us | 17.25 us | 17.63 us | 545.36 us | 599.71x | 7.99x | 7.82x | 0.25x |
| `mat4 bitxor` | 244.90 ns | 92.69 us | 115.70 us | 16.29 us | 16.48 us | 366.31 us | 378.46x | 5.69x | 5.62x | 0.25x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 9.16 ns | 604.38 ns | 664.13 ns | - | - | 1.34 us | 65.98x | - | - | 0.45x |
| `scalar add ref_owned` | 5.57 ns | 603.07 ns | 670.61 ns | - | - | 1.35 us | 108.27x | - | - | 0.45x |
| `scalar add refs` | 5.35 ns | 240.46 ns | 301.99 ns | - | - | 1.34 us | 44.95x | - | - | 0.18x |
| `scalar sub owned_ref` | 9.03 ns | 706.25 ns | 730.03 ns | - | - | 2.53 us | 78.19x | - | - | 0.28x |
| `scalar sub ref_owned` | 5.82 ns | 708.02 ns | 748.14 ns | - | - | 2.54 us | 121.68x | - | - | 0.28x |
| `scalar sub refs` | 5.77 ns | 331.61 ns | 364.69 ns | - | - | 2.53 us | 57.42x | - | - | 0.13x |
| `scalar mul owned_ref` | 9.94 ns | 728.51 ns | 744.63 ns | - | - | 1.58 us | 73.26x | - | - | 0.46x |
| `scalar mul ref_owned` | 6.36 ns | 741.80 ns | 765.48 ns | - | - | 1.57 us | 116.68x | - | - | 0.47x |
| `scalar mul refs` | 6.32 ns | 385.96 ns | 378.43 ns | - | - | 1.56 us | 61.04x | - | - | 0.25x |
| `scalar div owned_ref` | 15.62 ns | 650.87 ns | 691.09 ns | - | - | 2.61 us | 41.67x | - | - | 0.25x |
| `scalar div ref_owned` | 16.15 ns | 654.38 ns | 691.05 ns | - | - | 2.61 us | 40.52x | - | - | 0.25x |
| `scalar div refs` | 6.84 ns | 293.57 ns | 328.80 ns | - | - | 2.62 us | 42.89x | - | - | 0.11x |
| `vec3 add refs` | 6.09 ns | 1.85 us | 1.90 us | - | - | 4.09 us | 302.80x | - | - | 0.45x |
| `vec3 sub refs` | 6.11 ns | 1.90 us | 1.95 us | - | - | 7.55 us | 311.48x | - | - | 0.25x |
| `vec3 neg ref` | 3.33 ns | 525.75 ns | 526.13 ns | - | - | 3.22 us | 157.95x | - | - | 0.16x |
| `vec3 add_scalar_ref` | 6.57 ns | 1.88 us | 1.97 us | - | - | 3.89 us | 285.70x | - | - | 0.48x |
| `vec3 sub_scalar_ref` | 6.50 ns | 1.66 us | 1.73 us | - | - | 7.18 us | 255.52x | - | - | 0.23x |
| `vec3 mul_scalar_ref` | 7.00 ns | 2.28 us | 2.49 us | - | - | 4.45 us | 326.03x | - | - | 0.51x |
| `vec3 div_scalar_ref` | 9.76 ns | 2.11 us | 2.12 us | - | - | 7.79 us | 216.58x | - | - | 0.27x |
| `vec4 add refs` | 6.66 ns | 1.93 us | 1.88 us | - | - | 5.32 us | 289.05x | - | - | 0.36x |
| `vec4 sub refs` | 3.09 ns | 1.86 us | 1.77 us | - | - | 9.86 us | 603.60x | - | - | 0.19x |
| `vec4 neg ref` | 4.24 ns | 611.15 ns | 645.29 ns | - | - | 4.19 us | 143.99x | - | - | 0.15x |
| `vec4 add_scalar_ref` | 6.93 ns | 2.38 us | 2.26 us | - | - | 5.16 us | 343.50x | - | - | 0.46x |
| `vec4 sub_scalar_ref` | 4.24 ns | 2.16 us | 2.02 us | - | - | 9.58 us | 510.69x | - | - | 0.23x |
| `vec4 mul_scalar_ref` | 7.46 ns | 2.51 us | 2.58 us | - | - | 5.80 us | 336.70x | - | - | 0.43x |
| `vec4 div_scalar_ref` | 13.02 ns | 2.30 us | 2.26 us | - | - | 10.25 us | 176.77x | - | - | 0.22x |
| `mat3 add refs` | 10.95 ns | 3.67 us | 4.53 us | - | - | 11.98 us | 335.58x | - | - | 0.31x |
| `mat3 sub refs` | 10.66 ns | 3.71 us | 4.55 us | - | - | 21.78 us | 348.53x | - | - | 0.17x |
| `mat3 mul refs` | 57.43 ns | 19.19 us | 31.23 us | - | - | 63.52 us | 334.17x | - | - | 0.30x |
| `mat3 div refs` | 148.76 ns | 69.78 us | 69.25 us | - | - | 167.39 us | 469.06x | - | - | 0.42x |
| `mat3 neg ref` | 9.71 ns | 1.04 us | 1.05 us | - | - | 8.76 us | 106.68x | - | - | 0.12x |
| `mat3 add_scalar_ref` | 10.87 ns | 3.86 us | 5.81 us | - | - | 12.37 us | 355.07x | - | - | 0.31x |
| `mat3 sub_scalar_ref` | 12.02 ns | 3.79 us | 5.65 us | - | - | 22.32 us | 315.13x | - | - | 0.17x |
| `mat3 mul_scalar_ref` | 11.51 ns | 5.56 us | 6.63 us | - | - | 12.62 us | 483.08x | - | - | 0.44x |
| `mat3 div_scalar_ref` | 23.32 ns | 5.97 us | 6.47 us | - | - | 22.97 us | 255.75x | - | - | 0.26x |
| `mat4 add refs` | 17.56 ns | 3.97 us | 4.38 us | - | - | 20.45 us | 226.04x | - | - | 0.19x |
| `mat4 sub refs` | 16.35 ns | 4.37 us | 4.48 us | - | - | 37.76 us | 267.17x | - | - | 0.12x |
| `mat4 mul refs` | 102.32 ns | 22.57 us | 30.89 us | - | - | 146.80 us | 220.62x | - | - | 0.15x |
| `mat4 div refs` | 225.35 ns | 137.39 us | 111.87 us | - | - | 555.54 us | 609.67x | - | - | 0.25x |
| `mat4 neg ref` | 12.46 ns | 1.59 us | 1.52 us | - | - | 14.76 us | 127.91x | - | - | 0.11x |
| `mat4 add_scalar_ref` | 14.24 ns | 6.36 us | 8.49 us | - | - | 21.47 us | 446.75x | - | - | 0.30x |
| `mat4 sub_scalar_ref` | 16.43 ns | 6.25 us | 8.29 us | - | - | 38.54 us | 380.49x | - | - | 0.16x |
| `mat4 mul_scalar_ref` | 48.98 ns | 7.96 us | 9.02 us | - | - | 20.83 us | 162.43x | - | - | 0.38x |
| `mat4 div_scalar_ref` | 31.07 ns | 9.17 us | 9.50 us | - | - | 39.57 us | 295.08x | - | - | 0.23x |
| `mat3 transform_vec refs` | 14.53 ns | 9.37 us | 11.97 us | - | - | 21.17 us | 644.65x | - | - | 0.44x |
| `mat4 transform_vec refs` | 23.13 ns | 11.73 us | 12.35 us | - | - | 36.39 us | 507.34x | - | - | 0.32x |
| `complex add refs` | 7.76 ns | 1.00 us | 1.10 us | - | - | 2.65 us | 129.29x | - | - | 0.38x |
| `complex sub refs` | 8.05 ns | 1.11 us | 1.12 us | - | - | 4.92 us | 137.32x | - | - | 0.22x |
| `complex mul refs` | 7.97 ns | 3.22 us | 3.58 us | - | - | 10.40 us | 404.61x | - | - | 0.31x |
| `complex div refs` | 17.33 ns | 6.19 us | 6.96 us | - | - | 22.15 us | 357.37x | - | - | 0.28x |
| `complex neg ref` | 2.38 ns | 407.27 ns | 391.53 ns | - | - | 2.17 us | 171.10x | - | - | 0.19x |
| `complex div_real_ref` | 10.12 ns | 1.12 us | 1.13 us | - | - | 5.37 us | 110.83x | - | - | 0.21x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 13.14 us |
| `astro sin 160` | 14.35 us |
| `astro sin 192` | 14.34 us |
| `astro sin 256` | 16.55 us |
