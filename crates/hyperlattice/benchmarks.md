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
| `sin 0.1` | 11.06 ns | 181.25 ns | 175.86 ns | 11.72 us | 762.12 ns | 1.92 us | 16.39x | 0.02x | 0.24x | 0.09x |
| `cos 0.1` | 11.71 ns | 177.93 ns | 177.20 ns | 10.90 us | 501.56 ns | 1.75 us | 15.20x | 0.02x | 0.35x | 0.10x |
| `sin 1.23456789` | 11.90 ns | 1.85 us | 2.03 us | 12.90 us | 818.38 ns | 1.85 us | 155.46x | 0.14x | 2.26x | 1.00x |
| `cos 1.23456789` | 12.19 ns | 2.22 us | 2.32 us | 11.05 us | 603.66 ns | 1.65 us | 182.13x | 0.20x | 3.68x | 1.34x |
| `sin 1e6` | 12.75 ns | 6.17 us | 6.00 us | 16.12 us | 1.09 us | 2.02 us | 484.35x | 0.38x | 5.67x | 3.06x |
| `cos 1e6` | 12.47 ns | 6.05 us | 5.96 us | 13.59 us | 823.65 ns | 1.83 us | 485.36x | 0.45x | 7.35x | 3.31x |
| `sin 1e30` | 68.49 ns | 8.13 us | 8.33 us | 18.94 us | 2.98 us | 3.67 us | 118.66x | 0.43x | 2.73x | 2.22x |
| `cos 1e30` | 72.70 ns | 8.10 us | 8.27 us | 15.47 us | 991.77 ns | 3.15 us | 111.41x | 0.52x | 8.17x | 2.57x |
| `sin pi_7` | 11.96 ns | 175.78 ns | 518.19 ns | 12.43 us | 749.95 ns | 1.93 us | 14.70x | 0.01x | 0.23x | 0.09x |
| `cos pi_7` | 11.86 ns | 175.33 ns | 951.20 ns | 11.20 us | 550.74 ns | 1.80 us | 14.78x | 0.02x | 0.32x | 0.10x |
| `sin 1000pi_eps` | 12.13 ns | 5.96 us | 4.28 us | 16.17 us | 2.32 us | 2.85 us | 491.14x | 0.37x | 2.57x | 2.09x |
| `cos 1000pi_eps` | 12.50 ns | 6.09 us | 4.27 us | 13.88 us | 592.04 ns | 1.63 us | 487.45x | 0.44x | 10.29x | 3.73x |
| `asin 0.5` | 11.21 ns | 572.71 ns | 556.39 ns | 49.13 us | 3.00 us | 13.60 us | 51.09x | 0.01x | 0.19x | 0.04x |
| `acos 0.5` | 11.57 ns | 1.18 us | 1.15 us | 59.97 us | 3.08 us | 13.34 us | 102.20x | 0.02x | 0.38x | 0.09x |
| `atanh 0.5` | 14.84 ns | 1.47 us | 1.46 us | 34.68 us | 1.67 us | 13.25 us | 98.98x | 0.04x | 0.88x | 0.11x |
| `asin neg_0.999999` | 14.24 ns | 5.72 us | 4.73 us | 14.18 us | 2.68 us | 13.45 us | 401.65x | 0.40x | 2.14x | 0.43x |
| `acos neg_0.999999` | 15.66 ns | 5.93 us | 5.32 us | 18.90 us | 2.73 us | 13.90 us | 378.54x | 0.31x | 2.17x | 0.43x |
| `atanh neg_0.999999` | 14.70 ns | 4.18 us | 3.66 us | 37.77 us | 1.60 us | 12.71 us | 284.69x | 0.11x | 2.61x | 0.33x |
| `asin 0.999999` | 13.77 ns | 5.75 us | 4.82 us | 13.71 us | 2.60 us | 13.17 us | 417.31x | 0.42x | 2.21x | 0.44x |
| `acos 0.999999` | 14.24 ns | 6.04 us | 5.30 us | 18.32 us | 2.69 us | 13.12 us | 424.25x | 0.33x | 2.25x | 0.46x |
| `atanh 0.999999` | 14.95 ns | 4.25 us | 3.58 us | 31.69 us | 1.59 us | 12.63 us | 284.25x | 0.13x | 2.67x | 0.34x |
| `asin 1e-12` | 9.45 ns | 551.05 ns | 1.19 us | 7.96 us | 1.40 us | 15.50 us | 58.32x | 0.07x | 0.39x | 0.04x |
| `acos 1e-12` | 10.45 ns | 900.25 ns | 1.94 us | 9.89 us | 1.43 us | 15.38 us | 86.11x | 0.09x | 0.63x | 0.06x |
| `atanh 1e-12` | 9.85 ns | 541.38 ns | 931.00 ns | 37.34 us | 171.15 ns | 20.35 us | 54.99x | 0.01x | 3.16x | 0.03x |
| `atan 0.5` | 15.15 ns | 463.25 ns | 465.98 ns | 37.01 us | 2.84 us | 18.27 us | 30.58x | 0.01x | 0.16x | 0.03x |
| `asinh 0.5` | 26.78 ns | 1.69 us | 1.67 us | 40.41 us | 1.64 us | 7.58 us | 63.24x | 0.04x | 1.03x | 0.22x |
| `atan neg_1e-12` | 14.49 ns | 430.71 ns | 631.32 ns | 1.65 us | 1.12 us | 15.72 us | 29.72x | 0.26x | 0.38x | 0.03x |
| `asinh neg_1e-12` | 16.10 ns | 6.27 us | 4.87 us | 43.69 us | 8.82 us | 12.04 us | 389.65x | 0.14x | 0.71x | 0.52x |
| `atan 1e6` | 15.13 ns | 435.58 ns | 424.64 ns | 2.86 us | 1.46 us | 17.80 us | 28.79x | 0.15x | 0.30x | 0.02x |
| `asinh 1e6` | 26.89 ns | 3.46 us | 3.49 us | 37.29 us | 1.62 us | 7.14 us | 128.77x | 0.09x | 2.14x | 0.48x |
| `atan neg_1e6` | 15.04 ns | 534.36 ns | 527.92 ns | 2.85 us | 1.43 us | 17.89 us | 35.52x | 0.19x | 0.37x | 0.03x |
| `asinh neg_1e6` | 26.85 ns | 3.51 us | 3.66 us | 39.80 us | 1.62 us | 7.06 us | 130.79x | 0.09x | 2.17x | 0.50x |
| `acosh 9` | 12.38 ns | 2.95 us | 2.99 us | 42.96 us | 1.59 us | 9.76 us | 237.86x | 0.07x | 1.85x | 0.30x |
| `acosh 1_plus_1e-12` | 11.54 ns | 3.79 us | 5.13 us | 43.07 us | 8.54 us | 11.40 us | 328.17x | 0.09x | 0.44x | 0.33x |
| `acosh 1e6` | 12.52 ns | 3.72 us | 3.79 us | 38.28 us | 1.58 us | 9.69 us | 297.48x | 0.10x | 2.36x | 0.38x |
| `acosh e` | 12.65 ns | 4.06 us | 4.23 us | 42.62 us | 1.65 us | 9.71 us | 320.77x | 0.10x | 2.46x | 0.42x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 63.69 ns | 58.02 ns | 24.67 ns | 16.07 ns | 0.95 ns | 131.73x | 2.58x | 3.96x | 67.25x |
| `one` | 0.48 ns | 68.74 ns | 63.45 ns | 45.23 ns | 30.85 ns | 32.56 ns | 143.07x | 1.52x | 2.23x | 2.11x |
| `e` | 0.50 ns | 80.16 ns | 71.20 ns | 55.64 ns | 1.07 us | 221.47 ns | 161.90x | 1.44x | 0.08x | 0.36x |
| `pi` | 0.49 ns | 63.46 ns | 56.65 ns | 45.63 ns | 49.35 ns | 222.32 ns | 130.24x | 1.39x | 1.29x | 0.29x |
| `tau` | 0.49 ns | 201.38 ns | 191.44 ns | 119.70 ns | 101.03 ns | 1.84 us | 413.82x | 1.68x | 1.99x | 0.11x |
| `add` | 5.27 ns | 306.17 ns | 398.39 ns | 53.53 ns | 41.83 ns | 1.33 us | 58.09x | 5.72x | 7.32x | 0.23x |
| `sub` | 5.24 ns | 308.37 ns | 472.20 ns | 57.99 ns | 46.72 ns | 2.46 us | 58.80x | 5.32x | 6.60x | 0.13x |
| `neg` | 5.17 ns | 70.46 ns | 64.75 ns | 24.15 ns | 20.66 ns | 1.08 us | 13.62x | 2.92x | 3.41x | 0.07x |
| `mul` | 5.38 ns | 226.73 ns | 500.16 ns | 57.97 ns | 45.06 ns | 1.55 us | 42.17x | 3.91x | 5.03x | 0.15x |
| `div` | 9.09 ns | 399.49 ns | 459.51 ns | 138.31 ns | 63.79 ns | 2.63 us | 43.97x | 2.89x | 6.26x | 0.15x |
| `reciprocal` | 8.89 ns | 133.43 ns | 119.19 ns | 159.08 ns | 60.27 ns | 1.53 us | 15.01x | 0.84x | 2.21x | 0.09x |
| `reciprocal checked` | 9.01 ns | 126.54 ns | 124.21 ns | 162.23 ns | 58.88 ns | 1.55 us | 14.04x | 0.78x | 2.15x | 0.08x |
| `reciprocal checked abort` | 24.81 ns | 132.95 ns | 128.94 ns | 161.45 ns | 60.70 ns | 1.54 us | 5.36x | 0.82x | 2.19x | 0.09x |
| `pow` | 20.24 ns | 10.56 us | 10.74 us | 55.84 us | 2.94 us | 2.36 us | 521.77x | 0.19x | 3.59x | 4.48x |
| `powi` | 6.00 ns | 577.13 ns | 2.92 us | 291.77 ns | 86.84 ns | 1.64 us | 96.24x | 1.98x | 6.65x | 0.35x |
| `exp` | 19.86 ns | 1.72 us | 3.06 us | 14.14 us | 940.44 ns | 1.94 us | 86.78x | 0.12x | 1.83x | 0.89x |
| `ln` | 10.70 ns | 1.53 us | 1.54 us | 29.94 us | 1.37 us | 1.79 us | 142.76x | 0.05x | 1.11x | 0.85x |
| `log10` | 13.41 ns | 2.74 us | 2.35 us | 35.19 us | 2.75 us | 6.51 us | 204.29x | 0.08x | 1.00x | 0.42x |
| `log10 abort` | 17.23 ns | 2.76 us | 2.43 us | 35.22 us | 2.73 us | 6.67 us | 160.25x | 0.08x | 1.01x | 0.41x |
| `sqrt` | 20.54 ns | 1.58 us | 1.70 us | 5.21 us | 96.57 ns | 1.49 us | 76.98x | 0.30x | 16.37x | 1.06x |
| `sin` | 15.22 ns | 3.10 us | 3.59 us | 14.10 us | 1.24 us | 2.28 us | 203.41x | 0.22x | 2.50x | 1.36x |
| `cos` | 18.74 ns | 3.11 us | 3.71 us | 12.31 us | 626.81 ns | 1.76 us | 166.05x | 0.25x | 4.97x | 1.77x |
| `tan` | 25.33 ns | 2.84 us | 3.34 us | 30.75 us | 1.57 us | 6.68 us | 111.97x | 0.09x | 1.80x | 0.42x |
| `sinh` | 19.10 ns | 4.30 us | 4.55 us | 3.30 us | 1.14 us | 10.90 us | 225.25x | 1.30x | 3.79x | 0.39x |
| `cosh` | 19.15 ns | 4.27 us | 4.60 us | 8.04 us | 1.07 us | 9.65 us | 223.03x | 0.53x | 3.98x | 0.44x |
| `tanh` | 24.87 ns | 5.90 us | 6.20 us | 3.42 us | 1.23 us | 23.68 us | 237.42x | 1.73x | 4.81x | 0.25x |
| `asin` | 13.41 ns | 3.12 us | 4.70 us | 21.59 us | 2.48 us | 14.24 us | 232.82x | 0.14x | 1.26x | 0.22x |
| `asin abort` | 17.73 ns | 3.11 us | 4.83 us | 21.81 us | 2.50 us | 13.98 us | 175.58x | 0.14x | 1.25x | 0.22x |
| `acos` | 14.45 ns | 3.46 us | 5.71 us | 26.93 us | 2.52 us | 14.00 us | 239.21x | 0.13x | 1.37x | 0.25x |
| `acos abort` | 36.22 ns | 3.47 us | 5.44 us | 26.80 us | 2.52 us | 14.01 us | 95.73x | 0.13x | 1.38x | 0.25x |
| `atan` | 15.17 ns | 463.57 ns | 1.14 us | 18.80 us | 2.27 us | 19.06 us | 30.55x | 0.02x | 0.20x | 0.02x |
| `atan abort` | 21.95 ns | 484.80 ns | 1.18 us | 18.67 us | 2.26 us | 18.90 us | 22.09x | 0.03x | 0.21x | 0.03x |
| `asinh` | 34.87 ns | 3.69 us | 5.48 us | 39.65 us | 1.61 us | 7.53 us | 105.87x | 0.09x | 2.29x | 0.49x |
| `asinh abort` | 31.32 ns | 3.72 us | 5.67 us | 39.31 us | 1.65 us | 7.51 us | 118.81x | 0.09x | 2.26x | 0.50x |
| `acosh` | 12.34 ns | 3.77 us | 5.41 us | 39.84 us | 3.38 us | 10.38 us | 305.87x | 0.09x | 1.12x | 0.36x |
| `acosh abort` | 16.24 ns | 3.73 us | 5.38 us | 39.77 us | 3.37 us | 10.50 us | 229.87x | 0.09x | 1.11x | 0.36x |
| `atanh` | 14.14 ns | 2.70 us | 3.36 us | 35.98 us | 1.27 us | 14.89 us | 191.20x | 0.08x | 2.13x | 0.18x |
| `atanh abort` | 17.44 ns | 2.80 us | 3.31 us | 35.83 us | 1.25 us | 14.77 us | 160.61x | 0.08x | 2.23x | 0.19x |
| `zero status` | 1.23 ns | 1.94 ns | 1.98 ns | 1.12 ns | 6.82 ns | 8.28 ns | 1.58x | 1.73x | 0.28x | 0.23x |
| `zero status abort` | 1.44 ns | 3.57 ns | 3.44 ns | 1.01 ns | 6.78 ns | 8.33 ns | 2.48x | 3.53x | 0.53x | 0.43x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.95 ns | 105.99 ns | 107.74 ns | 48.07 ns | - | 1.91 ns | 111.96x | 2.20x | - | 55.45x |
| `one` | 5.07 ns | 112.58 ns | 111.60 ns | 64.45 ns | - | 31.86 ns | 22.19x | 1.75x | - | 3.53x |
| `i` | 0.94 ns | 113.80 ns | 114.34 ns | 63.07 ns | - | 33.73 ns | 120.91x | 1.80x | - | 3.37x |
| `free i` | 0.95 ns | 115.65 ns | 114.80 ns | 63.53 ns | - | 32.93 ns | 121.29x | 1.82x | - | 3.51x |
| `conjugate` | 2.16 ns | 135.83 ns | 133.15 ns | 41.35 ns | - | 1.11 us | 62.99x | 3.28x | - | 0.12x |
| `norm squared` | 5.85 ns | 404.21 ns | 1.56 us | 151.85 ns | - | 4.37 us | 69.15x | 2.66x | - | 0.09x |
| `reciprocal` | 19.38 ns | 1.92 us | 3.09 us | 466.57 ns | - | 10.84 us | 99.04x | 4.11x | - | 0.18x |
| `reciprocal checked` | 15.10 ns | 1.96 us | 3.11 us | 457.08 ns | - | 10.82 us | 129.63x | 4.28x | - | 0.18x |
| `powi` | 18.61 ns | 2.78 us | 13.97 us | 1.49 us | - | 44.02 us | 149.68x | 1.86x | - | 0.06x |
| `powi checked` | 18.27 ns | 2.78 us | 14.06 us | 1.46 us | - | 43.96 us | 152.02x | 1.90x | - | 0.06x |
| `div checked` | 18.03 ns | 3.21 us | 6.75 us | 800.08 ns | - | 21.82 us | 178.03x | 4.01x | - | 0.15x |
| `div real checked` | 9.36 ns | 830.76 ns | 795.54 ns | 267.89 ns | - | 5.31 us | 88.78x | 3.10x | - | 0.16x |
| `from scalar` | 1.43 ns | 133.58 ns | 128.35 ns | 46.85 ns | - | 10.64 ns | 93.15x | 2.85x | - | 12.56x |
| `add` | 6.10 ns | 619.06 ns | 1.02 us | 110.65 ns | - | 2.62 us | 101.54x | 5.59x | - | 0.24x |
| `sub` | 6.15 ns | 610.06 ns | 1.01 us | 119.60 ns | - | 4.80 us | 99.27x | 5.10x | - | 0.13x |
| `neg` | 2.62 ns | 138.50 ns | 142.35 ns | 43.98 ns | - | 2.13 us | 52.82x | 3.15x | - | 0.06x |
| `mul` | 7.61 ns | 1.10 us | 3.55 us | 306.15 ns | - | 10.27 us | 144.85x | 3.60x | - | 0.11x |
| `div` | 18.48 ns | 3.09 us | 6.84 us | 791.59 ns | - | 21.82 us | 167.43x | 3.91x | - | 0.14x |
| `div real` | 10.38 ns | 821.58 ns | 794.04 ns | 268.99 ns | - | 5.21 us | 79.12x | 3.05x | - | 0.16x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.93 ns | 846.07 ns | 5.99 us | 329.34 ns | 250.05 ns | 7.22 us | 122.15x | 2.57x | 3.38x | 0.12x |
| `vec3 magnitude` | 9.44 ns | 4.76 us | 9.25 us | 5.55 us | 349.86 ns | 8.92 us | 504.56x | 0.86x | 13.61x | 0.53x |
| `vec3 normalize` | 25.29 ns | 8.95 us | 15.40 us | 6.09 us | 590.87 ns | 17.48 us | 354.06x | 1.47x | 15.15x | 0.51x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.11 ns | 303.00 ns | 1.55 us | 77.31 ns | 58.93 ns | 717.06 ns | 97.29x | 3.92x | 5.14x | 0.42x |
| `vec3 zero` | 1.43 ns | 247.91 ns | 246.04 ns | 58.86 ns | 32.32 ns | 2.84 ns | 173.81x | 4.21x | 7.67x | 87.40x |
| `vec3 dot abort` | 27.89 ns | 1.26 us | 4.54 us | 261.41 ns | 207.05 ns | 7.39 us | 45.21x | 4.82x | 6.09x | 0.17x |
| `vec3 magnitude abort` | 39.10 ns | 5.24 us | 7.15 us | 5.44 us | 330.82 ns | 9.04 us | 134.15x | 0.96x | 15.85x | 0.58x |
| `vec3 normalize checked` | 26.19 ns | 9.21 us | 10.47 us | 5.86 us | 550.14 ns | 17.80 us | 351.71x | 1.57x | 16.74x | 0.52x |
| `vec3 normalize checked abort` | 55.08 ns | 9.69 us | 11.36 us | 5.94 us | 548.27 ns | 17.74 us | 175.85x | 1.63x | 17.67x | 0.55x |
| `vec3 div scalar checked` | 16.49 ns | 1.51 us | 1.83 us | 412.28 ns | - | - | 91.68x | 3.67x | - | - |
| `vec3 div scalar checked abort` | 18.17 ns | 1.51 us | 1.75 us | 407.16 ns | - | - | 83.36x | 3.72x | - | - |
| `vec3 add` | 6.81 ns | 1.19 us | 1.99 us | 159.42 ns | 126.80 ns | 3.99 us | 174.69x | 7.46x | 9.38x | 0.30x |
| `vec3 add scalar` | 6.42 ns | 947.13 ns | 1.63 us | 154.80 ns | 137.54 ns | 3.84 us | 147.55x | 6.12x | 6.89x | 0.25x |
| `vec3 sub` | 6.80 ns | 1.18 us | 1.97 us | 164.51 ns | 139.75 ns | 7.33 us | 173.02x | 7.15x | 8.41x | 0.16x |
| `vec3 sub scalar` | 6.46 ns | 924.83 ns | 1.42 us | 149.75 ns | 127.41 ns | 7.05 us | 143.11x | 6.18x | 7.26x | 0.13x |
| `vec3 neg` | 3.79 ns | 237.40 ns | 233.30 ns | 56.25 ns | 47.33 ns | 3.15 us | 62.72x | 4.22x | 5.02x | 0.08x |
| `vec3 mul scalar` | 7.01 ns | 682.40 ns | 2.18 us | 162.27 ns | 120.92 ns | 4.52 us | 97.35x | 4.21x | 5.64x | 0.15x |
| `vec3 div scalar` | 10.06 ns | 1.48 us | 1.80 us | 419.51 ns | 170.17 ns | 7.92 us | 147.15x | 3.53x | 8.70x | 0.19x |
| `vec4 dot` | 7.28 ns | 1.00 us | 3.62 us | 452.13 ns | 323.96 ns | 10.20 us | 137.42x | 2.21x | 3.09x | 0.10x |
| `vec4 magnitude` | 13.02 ns | 4.71 us | 5.65 us | 5.62 us | 425.86 ns | 11.32 us | 361.76x | 0.84x | 11.06x | 0.42x |
| `vec4 normalize` | 37.14 ns | 9.12 us | 9.28 us | 6.15 us | 714.24 ns | 22.32 us | 245.42x | 1.48x | 12.76x | 0.41x |
| `vec4 add` | 8.13 ns | 1.48 us | 2.18 us | 216.53 ns | 177.11 ns | 5.31 us | 182.07x | 6.84x | 8.36x | 0.28x |
| `vec4 add scalar` | 7.01 ns | 1.23 us | 1.95 us | 224.61 ns | 178.34 ns | 5.12 us | 174.99x | 5.46x | 6.88x | 0.24x |
| `vec4 sub` | 5.34 ns | 1.48 us | 2.04 us | 214.98 ns | 178.49 ns | 10.09 us | 277.75x | 6.90x | 8.31x | 0.15x |
| `vec4 sub scalar` | 4.74 ns | 1.25 us | 1.74 us | 209.02 ns | 170.17 ns | 9.71 us | 263.38x | 5.97x | 7.33x | 0.13x |
| `vec4 neg` | 5.16 ns | 309.69 ns | 313.77 ns | 82.87 ns | 66.13 ns | 4.11 us | 60.02x | 3.74x | 4.68x | 0.08x |
| `vec4 mul scalar` | 7.54 ns | 868.03 ns | 2.32 us | 228.01 ns | 161.89 ns | 5.77 us | 115.20x | 3.81x | 5.36x | 0.15x |
| `vec4 div scalar` | 14.18 ns | 2.05 us | 1.92 us | 534.95 ns | 226.87 ns | 10.33 us | 144.63x | 3.83x | 9.04x | 0.20x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.40 ns | 1.89 us | 3.67 us | 984.04 ns | 856.84 ns | 22.32 us | 152.78x | 1.93x | 2.21x | 0.08x |
| `mat3 inverse` | 93.21 ns | 18.91 us | 12.20 us | 3.18 us | 2.50 us | 82.57 us | 202.90x | 5.94x | 7.57x | 0.23x |
| `mat3 mul mat3` | 86.22 ns | 8.28 us | 13.24 us | 2.85 us | 2.45 us | 61.67 us | 96.02x | 2.91x | 3.38x | 0.13x |
| `mat3 transform vec3` | 16.13 ns | 3.39 us | 11.35 us | 1.12 us | 905.76 ns | 20.51 us | 210.42x | 3.04x | 3.75x | 0.17x |
| `mat4 determinant` | 46.31 ns | 5.47 us | 4.18 us | 4.67 us | 4.20 us | 95.90 us | 118.15x | 1.17x | 1.30x | 0.06x |
| `mat4 inverse` | 145.02 ns | 35.47 us | 17.26 us | 11.35 us | 9.09 us | 341.47 us | 244.56x | 3.12x | 3.90x | 0.10x |
| `mat4 mul mat4` | 117.98 ns | 16.52 us | 14.58 us | 6.54 us | 5.38 us | 141.72 us | 140.03x | 2.53x | 3.07x | 0.12x |
| `mat4 transform vec4` | 25.90 ns | 5.56 us | 4.70 us | 1.93 us | 1.67 us | 35.99 us | 214.80x | 2.88x | 3.33x | 0.15x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.93 ns | 1.38 us | 4.59 us | 208.32 ns | 236.84 ns | 2.06 us | 39.47x | 6.62x | 5.82x | 0.67x |
| `mat3 zero` | 16.89 ns | 904.32 ns | 893.57 ns | 267.15 ns | 216.85 ns | 12.07 ns | 53.56x | 3.39x | 4.17x | 74.94x |
| `mat3 identity` | 10.17 ns | 1.03 us | 1.02 us | 339.12 ns | 244.53 ns | 153.26 ns | 101.31x | 3.04x | 4.22x | 6.73x |
| `mat3 transpose` | 9.16 ns | 992.71 ns | 969.72 ns | 231.72 ns | 215.07 ns | 115.90 ns | 108.36x | 4.28x | 4.62x | 8.57x |
| `mat3 reciprocal` | 80.60 ns | 18.43 us | 37.60 us | 2.99 us | 2.34 us | 82.04 us | 228.71x | 6.16x | 7.88x | 0.22x |
| `mat3 reciprocal checked` | 81.76 ns | 18.31 us | 37.21 us | 3.02 us | 2.37 us | 83.57 us | 224.00x | 6.07x | 7.73x | 0.22x |
| `mat3 inverse checked` | 81.96 ns | 18.61 us | 36.86 us | 3.05 us | 2.30 us | 84.44 us | 227.04x | 6.09x | 8.10x | 0.22x |
| `mat3 inverse checked abort` | 119.04 ns | 18.54 us | 37.74 us | 2.95 us | 2.29 us | 83.06 us | 155.71x | 6.28x | 8.10x | 0.22x |
| `mat3 powi` | 148.13 ns | 16.89 us | 90.98 us | 6.85 us | 6.22 us | 158.50 us | 113.99x | 2.47x | 2.71x | 0.11x |
| `mat3 powi checked` | 146.59 ns | 17.59 us | 90.06 us | 6.84 us | 6.19 us | 156.52 us | 119.99x | 2.57x | 2.84x | 0.11x |
| `mat3 powi checked abort` | 146.32 ns | 17.52 us | 90.93 us | 6.78 us | 6.24 us | 153.46 us | 119.74x | 2.58x | 2.81x | 0.11x |
| `mat3 div scalar checked` | 24.50 ns | 4.90 us | 6.25 us | 1.44 us | 814.72 ns | 22.61 us | 200.18x | 3.40x | 6.02x | 0.22x |
| `mat3 div scalar checked abort` | 30.10 ns | 4.84 us | 6.22 us | 1.46 us | 825.85 ns | 22.50 us | 160.76x | 3.31x | 5.86x | 0.22x |
| `mat3 div matrix checked` | 152.50 ns | 65.30 us | 65.96 us | 5.65 us | 4.60 us | 164.11 us | 428.18x | 11.56x | 14.20x | 0.40x |
| `mat3 div matrix checked abort` | 168.24 ns | 66.11 us | 64.47 us | 5.54 us | 4.43 us | 166.08 us | 392.94x | 11.94x | 14.91x | 0.40x |
| `mat3 add` | 14.91 ns | 4.11 us | 6.13 us | 522.95 ns | 491.24 ns | 11.94 us | 276.04x | 7.87x | 8.38x | 0.34x |
| `mat3 add scalar` | 12.66 ns | 2.94 us | 5.10 us | 813.39 ns | 712.72 ns | 12.12 us | 232.08x | 3.61x | 4.12x | 0.24x |
| `mat3 sub` | 14.02 ns | 4.04 us | 6.11 us | 549.49 ns | 521.40 ns | 21.08 us | 287.98x | 7.35x | 7.74x | 0.19x |
| `mat3 sub scalar` | 11.58 ns | 2.97 us | 5.04 us | 850.75 ns | 727.92 ns | 21.83 us | 256.33x | 3.49x | 4.08x | 0.14x |
| `mat3 neg` | 10.95 ns | 905.87 ns | 896.71 ns | 524.06 ns | 472.25 ns | 8.57 us | 82.70x | 1.73x | 1.92x | 0.11x |
| `mat3 mul scalar` | 13.68 ns | 2.28 us | 6.16 us | 797.80 ns | 693.96 ns | 12.31 us | 166.66x | 2.86x | 3.29x | 0.19x |
| `mat3 div scalar` | 24.92 ns | 4.85 us | 6.23 us | 1.49 us | 834.61 ns | 22.47 us | 194.81x | 3.26x | 5.82x | 0.22x |
| `mat3 div matrix` | 151.49 ns | 65.57 us | 65.54 us | 5.44 us | 4.42 us | 164.29 us | 432.82x | 12.06x | 14.84x | 0.40x |
| `mat3 bitxor` | 147.67 ns | 17.26 us | 92.35 us | 6.82 us | 6.40 us | 155.98 us | 116.88x | 2.53x | 2.70x | 0.11x |
| `mat4 zero` | 13.71 ns | 1.40 us | 1.38 us | 509.25 ns | 354.03 ns | 14.51 ns | 102.16x | 2.75x | 3.96x | 96.50x |
| `mat4 identity` | 10.45 ns | 1.57 us | 1.61 us | 560.33 ns | 442.02 ns | 231.48 ns | 150.50x | 2.81x | 3.56x | 6.79x |
| `mat4 transpose` | 9.93 ns | 1.67 us | 1.71 us | 420.24 ns | 379.10 ns | 178.40 ns | 168.20x | 3.97x | 4.41x | 9.36x |
| `mat4 reciprocal` | 145.25 ns | 36.23 us | 67.13 us | 10.76 us | 8.86 us | 340.71 us | 249.43x | 3.37x | 4.09x | 0.11x |
| `mat4 reciprocal checked` | 155.73 ns | 37.10 us | 66.83 us | 10.82 us | 9.07 us | 343.87 us | 238.23x | 3.43x | 4.09x | 0.11x |
| `mat4 powi` | 243.15 ns | 34.15 us | 117.95 us | 16.14 us | 14.22 us | 352.62 us | 140.44x | 2.12x | 2.40x | 0.10x |
| `mat4 powi checked` | 244.38 ns | 33.85 us | 116.12 us | 16.29 us | 14.34 us | 348.94 us | 138.53x | 2.08x | 2.36x | 0.10x |
| `mat4 add` | 53.37 ns | 6.94 us | 7.65 us | 912.60 ns | 857.05 ns | 19.17 us | 130.05x | 7.61x | 8.10x | 0.36x |
| `mat4 add scalar` | 21.17 ns | 4.92 us | 7.52 us | 1.47 us | 1.22 us | 20.50 us | 232.56x | 3.36x | 4.05x | 0.24x |
| `mat4 sub` | 37.78 ns | 6.62 us | 7.68 us | 979.45 ns | 906.79 ns | 36.22 us | 175.15x | 6.76x | 7.30x | 0.18x |
| `mat4 sub scalar` | 15.46 ns | 4.93 us | 7.58 us | 1.51 us | 1.18 us | 37.28 us | 318.60x | 3.27x | 4.16x | 0.13x |
| `mat4 neg` | 13.74 ns | 1.66 us | 1.62 us | 957.15 ns | 777.40 ns | 14.10 us | 120.44x | 1.73x | 2.13x | 0.12x |
| `mat4 mul scalar` | 24.16 ns | 3.81 us | 8.35 us | 1.47 us | 1.16 us | 20.30 us | 157.48x | 2.58x | 3.29x | 0.19x |
| `mat4 div scalar` | 32.57 ns | 8.61 us | 8.58 us | 2.77 us | 1.41 us | 39.28 us | 264.25x | 3.11x | 6.10x | 0.22x |
| `mat4 div matrix` | 229.40 ns | 134.06 us | 104.97 us | 17.30 us | 14.70 us | 536.57 us | 584.39x | 7.75x | 9.12x | 0.25x |
| `mat4 bitxor` | 252.93 ns | 35.34 us | 113.82 us | 16.42 us | 13.93 us | 358.54 us | 139.72x | 2.15x | 2.54x | 0.10x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.72 ns | 218.06 ns | 323.49 ns | - | - | 1.29 us | 58.61x | - | - | 0.17x |
| `scalar add ref_owned` | 12.56 ns | 216.30 ns | 323.20 ns | - | - | 1.32 us | 17.23x | - | - | 0.16x |
| `scalar add refs` | 5.44 ns | 194.40 ns | 293.93 ns | - | - | 1.30 us | 35.75x | - | - | 0.15x |
| `scalar add owned_ref_with_clone` | 9.17 ns | 241.22 ns | 347.03 ns | - | - | - | 26.30x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.59 ns | 246.76 ns | 344.94 ns | - | - | - | 44.11x | - | - | - |
| `scalar sub owned_ref` | 3.89 ns | 217.23 ns | 376.36 ns | - | - | 2.45 us | 55.88x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.54 ns | 215.84 ns | 376.29 ns | - | - | 2.47 us | 17.22x | - | - | 0.09x |
| `scalar sub refs` | 5.71 ns | 190.14 ns | 349.46 ns | - | - | 2.51 us | 33.27x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.20 ns | 246.86 ns | 410.82 ns | - | - | - | 26.82x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.80 ns | 243.19 ns | 407.85 ns | - | - | - | 41.91x | - | - | - |
| `scalar mul owned_ref` | 4.80 ns | 126.44 ns | 411.98 ns | - | - | 1.56 us | 26.36x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.90 ns | 130.48 ns | 417.28 ns | - | - | 1.60 us | 9.38x | - | - | 0.08x |
| `scalar mul refs` | 6.35 ns | 108.87 ns | 392.28 ns | - | - | 1.59 us | 17.13x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 10.01 ns | 155.81 ns | 440.47 ns | - | - | - | 15.57x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.41 ns | 154.30 ns | 430.22 ns | - | - | - | 24.07x | - | - | - |
| `scalar div owned_ref` | 5.99 ns | 294.54 ns | 349.33 ns | - | - | 2.56 us | 49.20x | - | - | 0.12x |
| `scalar div ref_owned` | 17.50 ns | 304.06 ns | 362.57 ns | - | - | 2.61 us | 17.37x | - | - | 0.12x |
| `scalar div refs` | 6.88 ns | 271.81 ns | 325.40 ns | - | - | 2.67 us | 39.51x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.84 ns | 320.51 ns | 389.40 ns | - | - | - | 20.23x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.71 ns | 331.73 ns | 393.80 ns | - | - | - | 21.11x | - | - | - |
| `vec3 add refs` | 6.21 ns | 739.86 ns | 1.49 us | - | - | 4.04 us | 119.08x | - | - | 0.18x |
| `vec3 sub refs` | 6.35 ns | 718.51 ns | 1.52 us | - | - | 7.61 us | 113.12x | - | - | 0.09x |
| `vec3 neg ref` | 3.37 ns | 264.16 ns | 254.97 ns | - | - | 3.25 us | 78.40x | - | - | 0.08x |
| `vec3 add_scalar_ref` | 6.66 ns | 952.36 ns | 1.64 us | - | - | 3.84 us | 142.96x | - | - | 0.25x |
| `vec3 sub_scalar_ref` | 6.59 ns | 918.98 ns | 1.44 us | - | - | 7.06 us | 139.41x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 7.09 ns | 672.65 ns | 2.16 us | - | - | 4.55 us | 94.90x | - | - | 0.15x |
| `vec3 div_scalar_ref` | 8.27 ns | 1.51 us | 1.77 us | - | - | 7.75 us | 182.45x | - | - | 0.19x |
| `vec4 add refs` | 6.69 ns | 890.74 ns | 1.53 us | - | - | 5.33 us | 133.12x | - | - | 0.17x |
| `vec4 sub refs` | 3.20 ns | 889.24 ns | 1.40 us | - | - | 9.73 us | 278.20x | - | - | 0.09x |
| `vec4 neg ref` | 4.30 ns | 361.94 ns | 361.36 ns | - | - | 4.15 us | 84.13x | - | - | 0.09x |
| `vec4 add_scalar_ref` | 7.10 ns | 1.21 us | 1.91 us | - | - | 5.09 us | 170.98x | - | - | 0.24x |
| `vec4 sub_scalar_ref` | 4.24 ns | 1.21 us | 1.70 us | - | - | 9.35 us | 286.01x | - | - | 0.13x |
| `vec4 mul_scalar_ref` | 7.41 ns | 873.81 ns | 2.33 us | - | - | 5.94 us | 117.87x | - | - | 0.15x |
| `vec4 div_scalar_ref` | 12.52 ns | 2.02 us | 1.74 us | - | - | 10.31 us | 161.36x | - | - | 0.20x |
| `mat3 add refs` | 22.01 ns | 1.96 us | 4.04 us | - | - | 11.68 us | 89.05x | - | - | 0.17x |
| `mat3 sub refs` | 22.75 ns | 1.97 us | 4.01 us | - | - | 21.58 us | 86.57x | - | - | 0.09x |
| `mat3 mul refs` | 45.41 ns | 6.17 us | 29.21 us | - | - | 61.51 us | 135.92x | - | - | 0.10x |
| `mat3 div refs` | 149.45 ns | 64.62 us | 64.39 us | - | - | 160.15 us | 432.41x | - | - | 0.40x |
| `mat3 neg ref` | 10.17 ns | 830.13 ns | 804.08 ns | - | - | 8.83 us | 81.60x | - | - | 0.09x |
| `mat3 add_scalar_ref` | 60.95 ns | 3.01 us | 5.26 us | - | - | 12.22 us | 49.42x | - | - | 0.25x |
| `mat3 sub_scalar_ref` | 59.74 ns | 2.98 us | 4.99 us | - | - | 21.96 us | 49.89x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 63.81 ns | 2.24 us | 6.43 us | - | - | 12.28 us | 35.07x | - | - | 0.18x |
| `mat3 div_scalar_ref` | 26.95 ns | 4.76 us | 6.46 us | - | - | 22.30 us | 176.63x | - | - | 0.21x |
| `mat4 add refs` | 19.43 ns | 3.17 us | 4.11 us | - | - | 19.66 us | 162.94x | - | - | 0.16x |
| `mat4 sub refs` | 23.35 ns | 3.14 us | 4.21 us | - | - | 36.38 us | 134.63x | - | - | 0.09x |
| `mat4 mul refs` | 80.50 ns | 13.19 us | 29.71 us | - | - | 145.59 us | 163.84x | - | - | 0.09x |
| `mat4 div refs` | 184.64 ns | 131.43 us | 100.11 us | - | - | 524.45 us | 711.83x | - | - | 0.25x |
| `mat4 neg ref` | 17.79 ns | 1.39 us | 1.35 us | - | - | 14.08 us | 78.10x | - | - | 0.10x |
| `mat4 add_scalar_ref` | 59.60 ns | 5.15 us | 7.75 us | - | - | 20.56 us | 86.47x | - | - | 0.25x |
| `mat4 sub_scalar_ref` | 43.40 ns | 5.14 us | 7.51 us | - | - | 37.20 us | 118.41x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 56.58 ns | 3.83 us | 8.41 us | - | - | 20.49 us | 67.75x | - | - | 0.19x |
| `mat4 div_scalar_ref` | 28.77 ns | 8.76 us | 8.77 us | - | - | 39.01 us | 304.37x | - | - | 0.22x |
| `mat3 transform_vec refs` | 15.20 ns | 2.44 us | 11.66 us | - | - | 20.48 us | 160.58x | - | - | 0.12x |
| `mat4 transform_vec refs` | 24.10 ns | 3.90 us | 11.75 us | - | - | 36.00 us | 161.72x | - | - | 0.11x |
| `complex add refs` | 7.94 ns | 387.66 ns | 772.10 ns | - | - | 2.61 us | 48.84x | - | - | 0.15x |
| `complex sub refs` | 8.10 ns | 423.04 ns | 790.96 ns | - | - | 4.85 us | 52.22x | - | - | 0.09x |
| `complex mul refs` | 8.03 ns | 872.38 ns | 3.18 us | - | - | 10.03 us | 108.63x | - | - | 0.09x |
| `complex div refs` | 17.48 ns | 2.89 us | 6.83 us | - | - | 22.47 us | 165.42x | - | - | 0.13x |
| `complex neg ref` | 2.36 ns | 135.53 ns | 118.08 ns | - | - | 2.17 us | 57.35x | - | - | 0.06x |
| `complex div_real_ref` | 7.31 ns | 753.78 ns | 720.61 ns | - | - | 5.29 us | 103.16x | - | - | 0.14x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.42 us |
| `astro sin 160` | 13.75 us |
| `astro sin 192` | 13.83 us |
| `astro sin 256` | 15.87 us |
