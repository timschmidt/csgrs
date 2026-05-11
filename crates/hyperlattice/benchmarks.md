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
| `sin 0.1` | 10.86 ns | 144.55 ns | 145.14 ns | 11.08 us | 760.24 ns | 1.80 us | 13.31x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.77 ns | 142.40 ns | 142.66 ns | 10.68 us | 493.36 ns | 1.64 us | 12.10x | 0.01x | 0.29x | 0.09x |
| `sin 1.23456789` | 11.51 ns | 201.08 ns | 187.09 ns | 12.61 us | 813.26 ns | 1.78 us | 17.47x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 11.86 ns | 201.62 ns | 190.90 ns | 10.90 us | 581.69 ns | 1.62 us | 17.00x | 0.02x | 0.35x | 0.12x |
| `sin 1e6` | 12.56 ns | 90.99 ns | 90.02 ns | 16.23 us | 1.08 us | 2.00 us | 7.25x | 0.01x | 0.08x | 0.05x |
| `cos 1e6` | 12.43 ns | 90.21 ns | 90.11 ns | 13.56 us | 812.90 ns | 1.81 us | 7.26x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 65.12 ns | 90.81 ns | 91.02 ns | 18.71 us | 2.96 us | 3.68 us | 1.39x | 0.00x | 0.03x | 0.02x |
| `cos 1e30` | 68.44 ns | 90.83 ns | 90.25 ns | 15.27 us | 1.02 us | 3.20 us | 1.33x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 11.74 ns | 143.98 ns | 351.90 ns | 12.23 us | 747.38 ns | 1.84 us | 12.26x | 0.01x | 0.19x | 0.08x |
| `cos pi_7` | 11.45 ns | 143.53 ns | 693.71 ns | 11.00 us | 536.95 ns | 1.68 us | 12.53x | 0.01x | 0.27x | 0.09x |
| `sin 1000pi_eps` | 11.60 ns | 91.03 ns | 714.18 ns | 16.03 us | 2.31 us | 2.86 us | 7.85x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.52 ns | 90.38 ns | 721.61 ns | 13.65 us | 582.01 ns | 1.65 us | 7.22x | 0.01x | 0.16x | 0.05x |
| `asin 0.5` | 11.01 ns | 135.65 ns | 136.96 ns | 48.72 us | 2.92 us | 13.08 us | 12.32x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.27 ns | 393.59 ns | 387.26 ns | 58.47 us | 2.90 us | 12.92 us | 34.94x | 0.01x | 0.14x | 0.03x |
| `atanh 0.5` | 14.25 ns | 832.85 ns | 837.09 ns | 34.16 us | 1.64 us | 12.67 us | 58.46x | 0.02x | 0.51x | 0.07x |
| `asin neg_0.999999` | 13.86 ns | 563.07 ns | 540.67 ns | 13.40 us | 2.53 us | 12.65 us | 40.63x | 0.04x | 0.22x | 0.04x |
| `acos neg_0.999999` | 15.08 ns | 511.08 ns | 472.39 ns | 17.96 us | 2.67 us | 12.71 us | 33.90x | 0.03x | 0.19x | 0.04x |
| `atanh neg_0.999999` | 14.22 ns | 205.69 ns | 192.90 ns | 36.19 us | 1.60 us | 12.42 us | 14.46x | 0.01x | 0.13x | 0.02x |
| `asin 0.999999` | 13.55 ns | 509.48 ns | 728.38 ns | 13.34 us | 2.52 us | 12.58 us | 37.59x | 0.04x | 0.20x | 0.04x |
| `acos 0.999999` | 14.07 ns | 296.48 ns | 535.09 ns | 17.92 us | 2.73 us | 12.70 us | 21.07x | 0.02x | 0.11x | 0.02x |
| `atanh 0.999999` | 14.16 ns | 210.33 ns | 198.95 ns | 31.44 us | 1.59 us | 12.37 us | 14.85x | 0.01x | 0.13x | 0.02x |
| `asin 1e-12` | 9.21 ns | 277.95 ns | 482.39 ns | 7.84 us | 1.43 us | 14.86 us | 30.18x | 0.04x | 0.19x | 0.02x |
| `acos 1e-12` | 9.98 ns | 468.88 ns | 654.72 ns | 9.64 us | 1.43 us | 14.91 us | 46.97x | 0.05x | 0.33x | 0.03x |
| `atanh 1e-12` | 9.82 ns | 242.95 ns | 216.71 ns | 35.93 us | 167.38 ns | 19.99 us | 24.73x | 0.01x | 1.45x | 0.01x |
| `atan 0.5` | 14.62 ns | 162.33 ns | 162.10 ns | 35.59 us | 2.76 us | 17.21 us | 11.10x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.25 ns | 269.36 ns | 270.72 ns | 38.79 us | 1.62 us | 7.27 us | 10.26x | 0.01x | 0.17x | 0.04x |
| `atan neg_1e-12` | 14.03 ns | 278.53 ns | 260.70 ns | 1.58 us | 1.12 us | 15.00 us | 19.85x | 0.18x | 0.25x | 0.02x |
| `asinh neg_1e-12` | 15.60 ns | 466.46 ns | 377.11 ns | 41.78 us | 8.71 us | 11.76 us | 29.89x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 15.09 ns | 155.81 ns | 155.91 ns | 2.79 us | 1.44 us | 17.57 us | 10.32x | 0.06x | 0.11x | 0.01x |
| `asinh 1e6` | 26.14 ns | 262.74 ns | 266.22 ns | 36.86 us | 1.66 us | 7.00 us | 10.05x | 0.01x | 0.16x | 0.04x |
| `atan neg_1e6` | 15.24 ns | 256.41 ns | 257.39 ns | 2.82 us | 1.43 us | 17.68 us | 16.83x | 0.09x | 0.18x | 0.01x |
| `asinh neg_1e6` | 26.14 ns | 387.31 ns | 387.48 ns | 36.71 us | 1.67 us | 6.90 us | 14.81x | 0.01x | 0.23x | 0.06x |
| `acosh 9` | 12.76 ns | 158.89 ns | 159.06 ns | 41.75 us | 1.61 us | 9.56 us | 12.45x | 0.00x | 0.10x | 0.02x |
| `acosh 1_plus_1e-12` | 11.26 ns | 232.03 ns | 232.52 ns | 41.87 us | 8.41 us | 11.18 us | 20.61x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.17 ns | 158.13 ns | 160.28 ns | 37.26 us | 1.59 us | 9.66 us | 12.99x | 0.00x | 0.10x | 0.02x |
| `acosh e` | 12.15 ns | 229.20 ns | 2.25 us | 40.93 us | 1.63 us | 9.41 us | 18.86x | 0.01x | 0.14x | 0.02x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 26.29 ns | 18.83 ns | 25.00 ns | 15.58 ns | 0.94 ns | 55.41x | 1.05x | 1.69x | 27.96x |
| `one` | 0.47 ns | 33.70 ns | 23.98 ns | 41.19 ns | 30.83 ns | 29.75 ns | 71.74x | 0.82x | 1.09x | 1.13x |
| `e` | 0.47 ns | 108.62 ns | 90.02 ns | 54.46 ns | 1.06 us | 226.69 ns | 229.96x | 1.99x | 0.10x | 0.48x |
| `pi` | 0.48 ns | 66.61 ns | 50.73 ns | 43.02 ns | 48.72 ns | 225.37 ns | 139.41x | 1.55x | 1.37x | 0.30x |
| `tau` | 0.47 ns | 66.28 ns | 50.74 ns | 126.13 ns | 104.22 ns | 1.85 us | 141.29x | 0.53x | 0.64x | 0.04x |
| `add` | 5.18 ns | 207.24 ns | 311.20 ns | 51.76 ns | 42.22 ns | 1.29 us | 40.03x | 4.00x | 4.91x | 0.16x |
| `sub` | 5.24 ns | 214.13 ns | 370.46 ns | 55.35 ns | 45.54 ns | 2.42 us | 40.89x | 3.87x | 4.70x | 0.09x |
| `neg` | 5.07 ns | 44.52 ns | 39.27 ns | 23.25 ns | 21.58 ns | 1.05 us | 8.78x | 1.91x | 2.06x | 0.04x |
| `mul` | 5.33 ns | 136.13 ns | 395.75 ns | 57.10 ns | 46.49 ns | 1.51 us | 25.52x | 2.38x | 2.93x | 0.09x |
| `div` | 9.02 ns | 311.73 ns | 357.52 ns | 134.73 ns | 65.02 ns | 2.52 us | 34.56x | 2.31x | 4.79x | 0.12x |
| `reciprocal` | 8.62 ns | 95.56 ns | 87.64 ns | 160.36 ns | 58.63 ns | 1.52 us | 11.08x | 0.60x | 1.63x | 0.06x |
| `reciprocal checked` | 8.93 ns | 101.32 ns | 80.47 ns | 160.12 ns | 58.82 ns | 1.51 us | 11.35x | 0.63x | 1.72x | 0.07x |
| `reciprocal checked abort` | 24.82 ns | 92.02 ns | 94.45 ns | 160.42 ns | 58.80 ns | 1.51 us | 3.71x | 0.57x | 1.56x | 0.06x |
| `pow` | 20.02 ns | 6.64 us | 5.82 us | 54.31 us | 2.86 us | 2.32 us | 331.60x | 0.12x | 2.32x | 2.86x |
| `powi` | 5.88 ns | 407.03 ns | 2.55 us | 279.15 ns | 86.80 ns | 1.56 us | 69.28x | 1.46x | 4.69x | 0.26x |
| `exp` | 19.48 ns | 261.33 ns | 248.20 ns | 14.01 us | 947.50 ns | 1.85 us | 13.42x | 0.02x | 0.28x | 0.14x |
| `ln` | 10.66 ns | 1.42 us | 1.38 us | 29.33 us | 1.32 us | 1.77 us | 132.84x | 0.05x | 1.07x | 0.80x |
| `log10` | 13.29 ns | 1.60 us | 1.56 us | 34.89 us | 2.81 us | 6.64 us | 120.05x | 0.05x | 0.57x | 0.24x |
| `log10 abort` | 17.96 ns | 1.58 us | 1.55 us | 35.01 us | 2.78 us | 6.55 us | 87.99x | 0.05x | 0.57x | 0.24x |
| `sqrt` | 8.13 ns | 1.57 us | 1.71 us | 4.94 us | 96.70 ns | 1.42 us | 193.53x | 0.32x | 16.27x | 1.10x |
| `sin` | 15.10 ns | 125.65 ns | 125.82 ns | 13.93 us | 1.27 us | 2.22 us | 8.32x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.04 ns | 125.30 ns | 126.21 ns | 11.98 us | 633.08 ns | 1.73 us | 6.95x | 0.01x | 0.20x | 0.07x |
| `tan` | 24.51 ns | 183.26 ns | 178.48 ns | 29.62 us | 1.61 us | 6.53 us | 7.48x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.19 ns | 3.58 us | 3.56 us | 3.29 us | 1.12 us | 10.68 us | 196.71x | 1.09x | 3.19x | 0.34x |
| `cosh` | 18.11 ns | 3.53 us | 3.58 us | 7.78 us | 1.05 us | 9.44 us | 195.12x | 0.45x | 3.36x | 0.37x |
| `tanh` | 23.08 ns | 7.29 us | 7.28 us | 3.33 us | 1.22 us | 22.79 us | 315.87x | 2.19x | 6.00x | 0.32x |
| `asin` | 13.17 ns | 402.45 ns | 612.56 ns | 20.92 us | 2.40 us | 13.67 us | 30.55x | 0.02x | 0.17x | 0.03x |
| `asin abort` | 17.06 ns | 389.06 ns | 611.02 ns | 20.85 us | 2.41 us | 13.55 us | 22.81x | 0.02x | 0.16x | 0.03x |
| `acos` | 13.57 ns | 463.12 ns | 663.30 ns | 26.34 us | 2.50 us | 13.59 us | 34.13x | 0.02x | 0.18x | 0.03x |
| `acos abort` | 17.74 ns | 453.93 ns | 662.68 ns | 26.24 us | 2.51 us | 13.70 us | 25.58x | 0.02x | 0.18x | 0.03x |
| `atan` | 14.76 ns | 179.84 ns | 170.63 ns | 18.38 us | 2.24 us | 18.36 us | 12.18x | 0.01x | 0.08x | 0.01x |
| `atan abort` | 21.13 ns | 170.00 ns | 171.06 ns | 18.48 us | 2.24 us | 18.65 us | 8.04x | 0.01x | 0.08x | 0.01x |
| `asinh` | 33.86 ns | 284.11 ns | 297.49 ns | 38.57 us | 1.66 us | 7.35 us | 8.39x | 0.01x | 0.17x | 0.04x |
| `asinh abort` | 31.09 ns | 280.98 ns | 290.83 ns | 38.59 us | 1.64 us | 7.30 us | 9.04x | 0.01x | 0.17x | 0.04x |
| `acosh` | 12.93 ns | 214.14 ns | 208.22 ns | 40.33 us | 3.33 us | 10.24 us | 16.56x | 0.01x | 0.06x | 0.02x |
| `acosh abort` | 15.66 ns | 220.14 ns | 226.54 ns | 40.30 us | 3.33 us | 10.27 us | 14.06x | 0.01x | 0.07x | 0.02x |
| `atanh` | 13.62 ns | 403.08 ns | 535.59 ns | 34.31 us | 1.28 us | 14.73 us | 29.59x | 0.01x | 0.32x | 0.03x |
| `atanh abort` | 16.89 ns | 391.55 ns | 532.88 ns | 34.31 us | 1.28 us | 14.65 us | 23.18x | 0.01x | 0.31x | 0.03x |
| `zero status` | 1.20 ns | 1.04 ns | 1.03 ns | 1.03 ns | 6.71 ns | 8.19 ns | 0.87x | 1.01x | 0.15x | 0.13x |
| `zero status abort` | 1.40 ns | 1.14 ns | 1.14 ns | 1.04 ns | 6.99 ns | 8.22 ns | 0.81x | 1.09x | 0.16x | 0.14x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 36.20 ns | 36.33 ns | 50.51 ns | - | 1.88 ns | 38.94x | 0.72x | - | 19.27x |
| `one` | 5.27 ns | 39.23 ns | 39.83 ns | 66.38 ns | - | 32.58 ns | 7.44x | 0.59x | - | 1.20x |
| `i` | 0.94 ns | 41.88 ns | 42.37 ns | 67.66 ns | - | 29.29 ns | 44.70x | 0.62x | - | 1.43x |
| `free i` | 0.94 ns | 41.92 ns | 41.97 ns | 66.64 ns | - | 29.30 ns | 44.62x | 0.63x | - | 1.43x |
| `conjugate` | 2.12 ns | 84.61 ns | 84.27 ns | 40.32 ns | - | 1.07 us | 39.99x | 2.10x | - | 0.08x |
| `norm squared` | 5.74 ns | 163.14 ns | 549.03 ns | 155.47 ns | - | 4.26 us | 28.41x | 1.05x | - | 0.04x |
| `reciprocal` | 14.30 ns | 1.74 us | 2.75 us | 448.10 ns | - | 10.60 us | 122.00x | 3.89x | - | 0.16x |
| `reciprocal checked` | 14.10 ns | 1.73 us | 2.81 us | 443.61 ns | - | 10.60 us | 122.56x | 3.90x | - | 0.16x |
| `powi` | 18.92 ns | 2.06 us | 13.04 us | 1.48 us | - | 42.40 us | 109.11x | 1.40x | - | 0.05x |
| `powi checked` | 18.94 ns | 2.07 us | 12.93 us | 1.48 us | - | 43.14 us | 109.13x | 1.39x | - | 0.05x |
| `div checked` | 22.31 ns | 2.66 us | 6.13 us | 793.96 ns | - | 21.24 us | 119.37x | 3.35x | - | 0.13x |
| `div real checked` | 9.38 ns | 648.00 ns | 631.26 ns | 264.26 ns | - | 5.10 us | 69.10x | 2.45x | - | 0.13x |
| `from scalar` | 1.43 ns | 56.10 ns | 55.75 ns | 44.38 ns | - | 9.97 ns | 39.11x | 1.26x | - | 5.63x |
| `add` | 5.81 ns | 462.96 ns | 817.27 ns | 105.36 ns | - | 2.53 us | 79.69x | 4.39x | - | 0.18x |
| `sub` | 5.76 ns | 467.13 ns | 860.90 ns | 113.53 ns | - | 4.69 us | 81.10x | 4.11x | - | 0.10x |
| `neg` | 2.57 ns | 86.79 ns | 85.06 ns | 41.68 ns | - | 2.10 us | 33.77x | 2.08x | - | 0.04x |
| `mul` | 7.48 ns | 849.49 ns | 3.11 us | 309.74 ns | - | 9.81 us | 113.57x | 2.74x | - | 0.09x |
| `div` | 18.16 ns | 2.66 us | 6.12 us | 811.50 ns | - | 21.30 us | 146.76x | 3.28x | - | 0.13x |
| `div real` | 9.97 ns | 662.76 ns | 637.98 ns | 264.08 ns | - | 5.08 us | 66.49x | 2.51x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.50 ns | 215.98 ns | 2.42 us | 305.81 ns | 251.54 ns | 7.28 us | 25.42x | 0.71x | 0.86x | 0.03x |
| `vec3 magnitude` | 15.79 ns | 4.03 us | 5.59 us | 5.38 us | 337.56 ns | 8.67 us | 254.93x | 0.75x | 11.92x | 0.46x |
| `vec3 normalize` | 27.58 ns | 8.17 us | 11.40 us | 5.92 us | 587.41 ns | 16.31 us | 296.10x | 1.38x | 13.90x | 0.50x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.03 ns | 289.00 ns | 1.48 us | 71.44 ns | 56.16 ns | 721.51 ns | 95.40x | 4.05x | 5.15x | 0.40x |
| `vec3 zero` | 1.41 ns | 140.54 ns | 138.11 ns | 61.77 ns | 30.76 ns | 2.81 ns | 99.87x | 2.28x | 4.57x | 49.93x |
| `vec3 dot abort` | 9.19 ns | 224.16 ns | 881.41 ns | 261.11 ns | 202.84 ns | 7.09 us | 24.39x | 0.86x | 1.11x | 0.03x |
| `vec3 magnitude abort` | 17.81 ns | 4.07 us | 3.28 us | 5.30 us | 318.48 ns | 8.62 us | 228.30x | 0.77x | 12.77x | 0.47x |
| `vec3 normalize checked` | 28.13 ns | 8.29 us | 7.29 us | 5.95 us | 530.61 ns | 16.80 us | 294.69x | 1.39x | 15.62x | 0.49x |
| `vec3 normalize checked abort` | 30.41 ns | 8.25 us | 7.34 us | 5.94 us | 532.10 ns | 16.72 us | 271.26x | 1.39x | 15.50x | 0.49x |
| `vec3 div scalar checked` | 10.13 ns | 1.29 us | 1.52 us | 391.41 ns | - | - | 127.29x | 3.29x | - | - |
| `vec3 div scalar checked abort` | 17.65 ns | 1.31 us | 1.52 us | 393.00 ns | - | - | 74.09x | 3.33x | - | - |
| `vec3 add` | 6.62 ns | 932.66 ns | 1.67 us | 150.12 ns | 126.03 ns | 3.90 us | 140.88x | 6.21x | 7.40x | 0.24x |
| `vec3 add scalar` | 6.40 ns | 757.39 ns | 1.41 us | 152.32 ns | 131.21 ns | 3.75 us | 118.30x | 4.97x | 5.77x | 0.20x |
| `vec3 sub` | 6.58 ns | 925.79 ns | 1.71 us | 166.70 ns | 137.02 ns | 7.23 us | 140.72x | 5.55x | 6.76x | 0.13x |
| `vec3 sub scalar` | 6.37 ns | 750.49 ns | 1.22 us | 146.56 ns | 121.75 ns | 6.90 us | 117.88x | 5.12x | 6.16x | 0.11x |
| `vec3 neg` | 3.76 ns | 146.26 ns | 143.74 ns | 55.76 ns | 45.86 ns | 3.10 us | 38.89x | 2.62x | 3.19x | 0.05x |
| `vec3 mul scalar` | 6.89 ns | 523.81 ns | 1.97 us | 164.06 ns | 116.09 ns | 4.31 us | 76.03x | 3.19x | 4.51x | 0.12x |
| `vec3 div scalar` | 10.11 ns | 1.26 us | 1.50 us | 402.58 ns | 174.05 ns | 7.47 us | 124.88x | 3.14x | 7.25x | 0.17x |
| `vec4 dot` | 9.59 ns | 255.19 ns | 629.62 ns | 444.99 ns | 317.80 ns | 9.55 us | 26.61x | 0.57x | 0.80x | 0.03x |
| `vec4 magnitude` | 17.05 ns | 3.82 us | 2.71 us | 5.51 us | 412.14 ns | 11.02 us | 224.32x | 0.69x | 9.28x | 0.35x |
| `vec4 normalize` | 33.00 ns | 8.44 us | 6.38 us | 6.05 us | 709.14 ns | 21.45 us | 255.86x | 1.39x | 11.91x | 0.39x |
| `vec4 add` | 7.72 ns | 1.18 us | 1.79 us | 207.67 ns | 174.13 ns | 5.19 us | 153.14x | 5.69x | 6.79x | 0.23x |
| `vec4 add scalar` | 6.85 ns | 966.42 ns | 1.65 us | 213.69 ns | 177.11 ns | 5.01 us | 141.15x | 4.52x | 5.46x | 0.19x |
| `vec4 sub` | 5.17 ns | 1.17 us | 1.69 us | 213.81 ns | 175.60 ns | 9.45 us | 226.30x | 5.47x | 6.67x | 0.12x |
| `vec4 sub scalar` | 4.43 ns | 962.75 ns | 1.43 us | 202.46 ns | 169.14 ns | 9.19 us | 217.48x | 4.76x | 5.69x | 0.10x |
| `vec4 neg` | 4.91 ns | 186.80 ns | 187.96 ns | 78.05 ns | 64.88 ns | 3.97 us | 38.05x | 2.39x | 2.88x | 0.05x |
| `vec4 mul scalar` | 7.32 ns | 641.71 ns | 1.98 us | 227.35 ns | 159.93 ns | 5.53 us | 87.71x | 2.82x | 4.01x | 0.12x |
| `vec4 div scalar` | 14.08 ns | 1.76 us | 1.53 us | 521.68 ns | 227.24 ns | 10.27 us | 124.68x | 3.37x | 7.73x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.59 ns | 743.31 ns | 2.23 us | 950.24 ns | 837.15 ns | 21.77 us | 37.94x | 0.78x | 0.89x | 0.03x |
| `mat3 inverse` | 94.78 ns | 14.95 us | 8.99 us | 3.11 us | 2.43 us | 79.99 us | 157.77x | 4.81x | 6.14x | 0.19x |
| `mat3 mul mat3` | 52.24 ns | 2.97 us | 6.92 us | 2.81 us | 2.31 us | 59.79 us | 56.79x | 1.06x | 1.29x | 0.05x |
| `mat3 transform vec3` | 22.79 ns | 1.35 us | 4.46 us | 1.03 us | 881.66 ns | 19.78 us | 59.06x | 1.31x | 1.53x | 0.07x |
| `mat4 determinant` | 42.79 ns | 2.21 us | 1.57 us | 4.67 us | 4.04 us | 93.13 us | 51.68x | 0.47x | 0.55x | 0.02x |
| `mat4 inverse` | 164.94 ns | 25.24 us | 9.25 us | 11.46 us | 9.15 us | 333.86 us | 153.04x | 2.20x | 2.76x | 0.08x |
| `mat4 mul mat4` | 113.83 ns | 6.24 us | 6.95 us | 6.44 us | 5.35 us | 139.19 us | 54.83x | 0.97x | 1.17x | 0.04x |
| `mat4 transform vec4` | 40.55 ns | 2.42 us | 2.46 us | 1.95 us | 1.62 us | 34.50 us | 59.64x | 1.24x | 1.50x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.47 ns | 1.32 us | 4.51 us | 208.69 ns | 229.87 ns | 2.07 us | 38.28x | 6.32x | 5.74x | 0.64x |
| `mat3 zero` | 15.80 ns | 537.87 ns | 540.29 ns | 274.94 ns | 203.64 ns | 11.24 ns | 34.05x | 1.96x | 2.64x | 47.86x |
| `mat3 identity` | 9.94 ns | 649.11 ns | 643.52 ns | 319.35 ns | 246.53 ns | 154.25 ns | 65.29x | 2.03x | 2.63x | 4.21x |
| `mat3 transpose` | 9.30 ns | 689.34 ns | 685.03 ns | 231.75 ns | 205.27 ns | 120.67 ns | 74.14x | 2.97x | 3.36x | 5.71x |
| `mat3 reciprocal` | 92.89 ns | 14.91 us | 24.71 us | 2.96 us | 2.25 us | 81.45 us | 160.53x | 5.04x | 6.62x | 0.18x |
| `mat3 reciprocal checked` | 124.42 ns | 14.95 us | 24.51 us | 2.93 us | 2.26 us | 81.19 us | 120.15x | 5.11x | 6.61x | 0.18x |
| `mat3 inverse checked` | 124.86 ns | 14.98 us | 24.72 us | 2.94 us | 2.25 us | 81.17 us | 119.95x | 5.09x | 6.66x | 0.18x |
| `mat3 inverse checked abort` | 143.21 ns | 14.83 us | 24.98 us | 2.93 us | 2.25 us | 79.75 us | 103.56x | 5.06x | 6.59x | 0.19x |
| `mat3 powi` | 103.10 ns | 5.62 us | 39.16 us | 6.99 us | 6.16 us | 146.30 us | 54.54x | 0.80x | 0.91x | 0.04x |
| `mat3 powi checked` | 103.00 ns | 5.64 us | 39.24 us | 6.99 us | 6.15 us | 146.76 us | 54.72x | 0.81x | 0.92x | 0.04x |
| `mat3 powi checked abort` | 103.04 ns | 5.62 us | 39.29 us | 6.98 us | 6.16 us | 146.30 us | 54.55x | 0.81x | 0.91x | 0.04x |
| `mat3 div scalar checked` | 24.75 ns | 4.21 us | 5.55 us | 1.43 us | 795.50 ns | 21.45 us | 169.95x | 2.94x | 5.29x | 0.20x |
| `mat3 div scalar checked abort` | 30.07 ns | 4.14 us | 5.45 us | 1.44 us | 791.75 ns | 21.40 us | 137.55x | 2.88x | 5.22x | 0.19x |
| `mat3 div matrix checked` | 116.16 ns | 18.68 us | 56.44 us | 5.49 us | 4.34 us | 156.98 us | 160.80x | 3.40x | 4.30x | 0.12x |
| `mat3 div matrix checked abort` | 127.66 ns | 18.70 us | 56.45 us | 5.43 us | 4.35 us | 154.68 us | 146.50x | 3.45x | 4.30x | 0.12x |
| `mat3 add` | 14.61 ns | 3.16 us | 5.26 us | 503.34 ns | 487.05 ns | 11.35 us | 216.32x | 6.28x | 6.49x | 0.28x |
| `mat3 add scalar` | 12.16 ns | 2.39 us | 4.47 us | 768.78 ns | 696.83 ns | 11.70 us | 196.42x | 3.11x | 3.43x | 0.20x |
| `mat3 sub` | 13.80 ns | 3.15 us | 5.10 us | 522.45 ns | 515.08 ns | 20.76 us | 228.66x | 6.04x | 6.12x | 0.15x |
| `mat3 sub scalar` | 11.18 ns | 2.39 us | 4.30 us | 780.79 ns | 691.67 ns | 21.20 us | 214.12x | 3.06x | 3.46x | 0.11x |
| `mat3 neg` | 10.70 ns | 609.80 ns | 607.31 ns | 466.72 ns | 465.18 ns | 8.37 us | 57.00x | 1.31x | 1.31x | 0.07x |
| `mat3 mul scalar` | 14.05 ns | 1.74 us | 5.58 us | 781.56 ns | 663.51 ns | 11.78 us | 123.74x | 2.22x | 2.62x | 0.15x |
| `mat3 div scalar` | 24.78 ns | 4.09 us | 5.46 us | 1.43 us | 795.79 ns | 21.51 us | 165.18x | 2.85x | 5.14x | 0.19x |
| `mat3 div matrix` | 126.35 ns | 18.62 us | 58.08 us | 5.48 us | 4.35 us | 154.16 us | 147.33x | 3.40x | 4.28x | 0.12x |
| `mat3 bitxor` | 103.85 ns | 5.64 us | 39.46 us | 6.98 us | 6.15 us | 146.37 us | 54.27x | 0.81x | 0.92x | 0.04x |
| `mat4 zero` | 13.32 ns | 779.77 ns | 780.97 ns | 527.01 ns | 341.79 ns | 14.05 ns | 58.55x | 1.48x | 2.28x | 55.49x |
| `mat4 identity` | 10.32 ns | 963.21 ns | 966.96 ns | 579.14 ns | 415.38 ns | 226.03 ns | 93.31x | 1.66x | 2.32x | 4.26x |
| `mat4 transpose` | 9.93 ns | 1.16 us | 1.14 us | 482.10 ns | 358.35 ns | 172.23 ns | 116.36x | 2.40x | 3.22x | 6.71x |
| `mat4 reciprocal` | 145.64 ns | 25.44 us | 45.53 us | 10.92 us | 8.68 us | 338.03 us | 174.68x | 2.33x | 2.93x | 0.08x |
| `mat4 reciprocal checked` | 168.50 ns | 25.46 us | 44.59 us | 10.95 us | 8.68 us | 337.73 us | 151.09x | 2.32x | 2.93x | 0.08x |
| `mat4 powi` | 213.56 ns | 11.61 us | 54.86 us | 16.03 us | 13.69 us | 342.13 us | 54.35x | 0.72x | 0.85x | 0.03x |
| `mat4 powi checked` | 216.57 ns | 11.62 us | 54.86 us | 16.19 us | 13.81 us | 336.58 us | 53.64x | 0.72x | 0.84x | 0.03x |
| `mat4 add` | 51.85 ns | 5.48 us | 6.29 us | 933.85 ns | 850.83 ns | 19.53 us | 105.68x | 5.87x | 6.44x | 0.28x |
| `mat4 add scalar` | 20.54 ns | 3.96 us | 6.47 us | 1.35 us | 1.16 us | 20.24 us | 192.80x | 2.93x | 3.40x | 0.20x |
| `mat4 sub` | 37.22 ns | 5.55 us | 6.44 us | 998.71 ns | 897.45 ns | 35.66 us | 149.15x | 5.56x | 6.19x | 0.16x |
| `mat4 sub scalar` | 14.97 ns | 4.02 us | 6.38 us | 1.38 us | 1.17 us | 36.36 us | 268.58x | 2.91x | 3.44x | 0.11x |
| `mat4 neg` | 13.71 ns | 1.08 us | 1.10 us | 860.21 ns | 752.12 ns | 13.91 us | 79.05x | 1.26x | 1.44x | 0.08x |
| `mat4 mul scalar` | 23.79 ns | 2.85 us | 7.22 us | 1.40 us | 1.13 us | 19.93 us | 119.86x | 2.03x | 2.53x | 0.14x |
| `mat4 div scalar` | 32.96 ns | 7.31 us | 7.45 us | 2.55 us | 1.40 us | 36.96 us | 221.85x | 2.87x | 5.24x | 0.20x |
| `mat4 div matrix` | 187.98 ns | 30.64 us | 86.68 us | 17.29 us | 13.93 us | 524.48 us | 162.97x | 1.77x | 2.20x | 0.06x |
| `mat4 bitxor` | 222.83 ns | 11.73 us | 54.93 us | 16.07 us | 13.69 us | 333.75 us | 52.62x | 0.73x | 0.86x | 0.04x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.60 ns | 172.64 ns | 276.18 ns | - | - | 1.30 us | 47.99x | - | - | 0.13x |
| `scalar add ref_owned` | 12.30 ns | 173.92 ns | 273.45 ns | - | - | 1.29 us | 14.14x | - | - | 0.13x |
| `scalar add refs` | 5.32 ns | 154.48 ns | 268.34 ns | - | - | 1.29 us | 29.05x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 8.94 ns | 187.59 ns | 295.21 ns | - | - | - | 20.97x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.43 ns | 182.09 ns | 288.03 ns | - | - | - | 33.53x | - | - | - |
| `scalar sub owned_ref` | 3.93 ns | 179.49 ns | 332.89 ns | - | - | 2.42 us | 45.67x | - | - | 0.07x |
| `scalar sub ref_owned` | 12.70 ns | 178.77 ns | 327.37 ns | - | - | 2.42 us | 14.08x | - | - | 0.07x |
| `scalar sub refs` | 5.58 ns | 165.97 ns | 318.51 ns | - | - | 2.41 us | 29.72x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 9.04 ns | 194.06 ns | 347.01 ns | - | - | - | 21.46x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.62 ns | 189.89 ns | 346.21 ns | - | - | - | 33.80x | - | - | - |
| `scalar mul owned_ref` | 4.51 ns | 97.18 ns | 364.55 ns | - | - | 1.51 us | 21.55x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.55 ns | 98.49 ns | 365.97 ns | - | - | 1.52 us | 7.27x | - | - | 0.06x |
| `scalar mul refs` | 6.25 ns | 82.48 ns | 360.78 ns | - | - | 1.52 us | 13.20x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.73 ns | 116.25 ns | 377.02 ns | - | - | - | 11.94x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.17 ns | 106.23 ns | 378.88 ns | - | - | - | 17.22x | - | - | - |
| `scalar div owned_ref` | 5.91 ns | 264.34 ns | 310.10 ns | - | - | 2.51 us | 44.74x | - | - | 0.11x |
| `scalar div ref_owned` | 17.11 ns | 255.22 ns | 310.86 ns | - | - | 2.54 us | 14.91x | - | - | 0.10x |
| `scalar div refs` | 6.82 ns | 243.78 ns | 304.57 ns | - | - | 2.52 us | 35.73x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.14 ns | 269.90 ns | 328.86 ns | - | - | - | 20.54x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.19 ns | 276.16 ns | 325.09 ns | - | - | - | 18.18x | - | - | - |
| `vec3 add refs` | 6.05 ns | 627.95 ns | 1.37 us | - | - | 3.93 us | 103.78x | - | - | 0.16x |
| `vec3 sub refs` | 6.10 ns | 621.70 ns | 1.44 us | - | - | 7.21 us | 101.89x | - | - | 0.09x |
| `vec3 neg ref` | 3.27 ns | 158.42 ns | 172.79 ns | - | - | 3.11 us | 48.42x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.47 ns | 792.12 ns | 1.44 us | - | - | 3.76 us | 122.39x | - | - | 0.21x |
| `vec3 sub_scalar_ref` | 6.53 ns | 767.88 ns | 1.22 us | - | - | 6.92 us | 117.50x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 6.96 ns | 539.38 ns | 1.95 us | - | - | 4.32 us | 77.45x | - | - | 0.12x |
| `vec3 div_scalar_ref` | 8.04 ns | 1.25 us | 1.49 us | - | - | 7.44 us | 155.62x | - | - | 0.17x |
| `vec4 add refs` | 6.68 ns | 737.58 ns | 1.35 us | - | - | 5.18 us | 110.39x | - | - | 0.14x |
| `vec4 sub refs` | 3.06 ns | 733.33 ns | 1.26 us | - | - | 9.51 us | 239.61x | - | - | 0.08x |
| `vec4 neg ref` | 4.23 ns | 197.29 ns | 200.24 ns | - | - | 4.00 us | 46.68x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 6.89 ns | 1.00 us | 1.73 us | - | - | 5.01 us | 145.29x | - | - | 0.20x |
| `vec4 sub_scalar_ref` | 4.20 ns | 980.99 ns | 1.46 us | - | - | 9.22 us | 233.71x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.41 ns | 661.01 ns | 2.03 us | - | - | 5.66 us | 89.20x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.60 ns | 1.70 us | 1.46 us | - | - | 9.81 us | 146.43x | - | - | 0.17x |
| `mat3 add refs` | 10.86 ns | 1.66 us | 3.67 us | - | - | 11.58 us | 153.12x | - | - | 0.14x |
| `mat3 sub refs` | 10.40 ns | 1.65 us | 3.72 us | - | - | 21.79 us | 158.31x | - | - | 0.08x |
| `mat3 mul refs` | 38.85 ns | 1.92 us | 12.76 us | - | - | 60.42 us | 49.37x | - | - | 0.03x |
| `mat3 div refs` | 117.29 ns | 17.47 us | 56.37 us | - | - | 157.30 us | 148.91x | - | - | 0.11x |
| `mat3 neg ref` | 9.70 ns | 483.10 ns | 480.72 ns | - | - | 8.42 us | 49.81x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 44.19 ns | 2.34 us | 4.57 us | - | - | 11.79 us | 53.07x | - | - | 0.20x |
| `mat3 sub_scalar_ref` | 43.92 ns | 2.36 us | 4.42 us | - | - | 20.91 us | 53.80x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 46.66 ns | 1.72 us | 5.73 us | - | - | 11.81 us | 36.89x | - | - | 0.15x |
| `mat3 div_scalar_ref` | 21.57 ns | 4.22 us | 5.45 us | - | - | 21.34 us | 195.87x | - | - | 0.20x |
| `mat4 add refs` | 17.52 ns | 2.64 us | 3.42 us | - | - | 19.09 us | 150.75x | - | - | 0.14x |
| `mat4 sub refs` | 16.22 ns | 2.63 us | 3.49 us | - | - | 35.38 us | 162.25x | - | - | 0.07x |
| `mat4 mul refs` | 101.76 ns | 4.42 us | 17.98 us | - | - | 138.80 us | 43.48x | - | - | 0.03x |
| `mat4 div refs` | 176.93 ns | 28.81 us | 83.25 us | - | - | 520.98 us | 162.86x | - | - | 0.06x |
| `mat4 neg ref` | 12.42 ns | 831.63 ns | 833.00 ns | - | - | 13.80 us | 66.99x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 52.20 ns | 3.97 us | 6.60 us | - | - | 20.40 us | 75.98x | - | - | 0.19x |
| `mat4 sub_scalar_ref` | 38.86 ns | 4.02 us | 6.44 us | - | - | 36.55 us | 103.57x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 55.44 ns | 2.86 us | 7.25 us | - | - | 19.91 us | 51.61x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 28.99 ns | 7.33 us | 7.39 us | - | - | 36.83 us | 252.68x | - | - | 0.20x |
| `mat3 transform_vec refs` | 21.09 ns | 750.15 ns | 4.00 us | - | - | 19.83 us | 35.58x | - | - | 0.04x |
| `mat4 transform_vec refs` | 33.53 ns | 1.36 us | 5.14 us | - | - | 34.69 us | 40.46x | - | - | 0.04x |
| `complex add refs` | 7.61 ns | 322.80 ns | 675.64 ns | - | - | 2.56 us | 42.43x | - | - | 0.13x |
| `complex sub refs` | 7.97 ns | 323.80 ns | 715.11 ns | - | - | 4.69 us | 40.64x | - | - | 0.07x |
| `complex mul refs` | 7.88 ns | 725.76 ns | 2.93 us | - | - | 9.87 us | 92.15x | - | - | 0.07x |
| `complex div refs` | 16.30 ns | 2.53 us | 5.99 us | - | - | 21.51 us | 155.35x | - | - | 0.12x |
| `complex neg ref` | 2.32 ns | 72.58 ns | 75.14 ns | - | - | 2.15 us | 31.34x | - | - | 0.03x |
| `complex div_real_ref` | 7.18 ns | 608.52 ns | 581.17 ns | - | - | 5.10 us | 84.76x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.46 us |
| `astro sin 160` | 13.49 us |
| `astro sin 192` | 13.53 us |
| `astro sin 256` | 15.83 us |
