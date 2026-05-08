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
| `sin 0.1` | 14.72 ns | 228.67 ns | 270.68 ns | 11.31 us | 772.78 ns | 1.87 us | 15.54x | 0.02x | 0.30x | 0.12x |
| `cos 0.1` | 20.79 ns | 231.13 ns | 230.52 ns | 10.83 us | 499.08 ns | 1.69 us | 11.12x | 0.02x | 0.46x | 0.14x |
| `sin 1.23456789` | 17.73 ns | 359.38 ns | 282.74 ns | 13.74 us | 835.53 ns | 1.79 us | 20.27x | 0.03x | 0.43x | 0.20x |
| `cos 1.23456789` | 25.20 ns | 280.94 ns | 272.41 ns | 11.22 us | 684.89 ns | 1.67 us | 11.15x | 0.03x | 0.41x | 0.17x |
| `sin 1e6` | 17.94 ns | 119.90 ns | 120.78 ns | 16.94 us | 1.20 us | 2.03 us | 6.69x | 0.01x | 0.10x | 0.06x |
| `cos 1e6` | 16.84 ns | 119.71 ns | 122.50 ns | 14.02 us | 1.06 us | 1.81 us | 7.11x | 0.01x | 0.11x | 0.07x |
| `sin 1e30` | 74.40 ns | 126.80 ns | 118.85 ns | 19.52 us | 2.90 us | 3.53 us | 1.70x | 0.01x | 0.04x | 0.04x |
| `cos 1e30` | 74.63 ns | 119.98 ns | 121.94 ns | 15.72 us | 973.34 ns | 3.18 us | 1.61x | 0.01x | 0.12x | 0.04x |
| `sin pi_7` | 15.39 ns | 227.22 ns | 387.61 ns | 12.61 us | 750.42 ns | 3.20 us | 14.76x | 0.02x | 0.30x | 0.07x |
| `cos pi_7` | 16.50 ns | 228.80 ns | 772.45 ns | 11.21 us | 536.48 ns | 1.73 us | 13.87x | 0.02x | 0.43x | 0.13x |
| `sin 1000pi_eps` | 15.60 ns | 118.92 ns | 3.99 us | 16.76 us | 2.28 us | 4.57 us | 7.63x | 0.01x | 0.05x | 0.03x |
| `cos 1000pi_eps` | 18.41 ns | 119.43 ns | 4.07 us | 14.17 us | 578.46 ns | 1.79 us | 6.49x | 0.01x | 0.21x | 0.07x |
| `asin 0.5` | 20.99 ns | 502.71 ns | 515.56 ns | 50.43 us | 2.99 us | 13.46 us | 23.95x | 0.01x | 0.17x | 0.04x |
| `acos 0.5` | 23.35 ns | 1.12 us | 1.12 us | 76.38 us | 2.95 us | 18.62 us | 47.85x | 0.01x | 0.38x | 0.06x |
| `atanh 0.5` | 30.75 ns | 1.47 us | 1.50 us | 36.12 us | 1.70 us | 12.91 us | 47.84x | 0.04x | 0.87x | 0.11x |
| `asin neg_0.999999` | 25.45 ns | 800.71 ns | 1.26 us | 14.11 us | 4.93 us | 12.84 us | 31.46x | 0.06x | 0.16x | 0.06x |
| `acos neg_0.999999` | 24.90 ns | 1.01 us | 1.81 us | 18.25 us | 2.77 us | 13.51 us | 40.36x | 0.06x | 0.36x | 0.07x |
| `atanh neg_0.999999` | 25.52 ns | 440.84 ns | 675.09 ns | 37.20 us | 1.63 us | 12.69 us | 17.27x | 0.01x | 0.27x | 0.03x |
| `asin 0.999999` | 24.78 ns | 785.25 ns | 1.53 us | 14.17 us | 2.57 us | 12.81 us | 31.70x | 0.06x | 0.31x | 0.06x |
| `acos 0.999999` | 24.97 ns | 650.06 ns | 1.71 us | 18.78 us | 2.74 us | 13.72 us | 26.03x | 0.03x | 0.24x | 0.05x |
| `atanh 0.999999` | 26.09 ns | 511.69 ns | 965.31 ns | 32.50 us | 1.63 us | 12.74 us | 19.61x | 0.02x | 0.31x | 0.04x |
| `asin 1e-12` | 14.31 ns | 501.65 ns | 1.17 us | 8.21 us | 1.76 us | 15.23 us | 35.06x | 0.06x | 0.28x | 0.03x |
| `acos 1e-12` | 15.77 ns | 1.23 us | 1.98 us | 10.19 us | 1.43 us | 19.95 us | 77.97x | 0.12x | 0.86x | 0.06x |
| `atanh 1e-12` | 25.74 ns | 525.73 ns | 915.25 ns | 38.33 us | 173.87 ns | 20.31 us | 20.43x | 0.01x | 3.02x | 0.03x |
| `atan 0.5` | 25.74 ns | 445.14 ns | 437.74 ns | 37.10 us | 2.77 us | 17.86 us | 17.29x | 0.01x | 0.16x | 0.02x |
| `asinh 0.5` | 37.98 ns | 477.14 ns | 486.82 ns | 41.68 us | 1.63 us | 7.66 us | 12.56x | 0.01x | 0.29x | 0.06x |
| `atan neg_1e-12` | 18.59 ns | 483.72 ns | 626.58 ns | 2.49 us | 1.17 us | 21.02 us | 26.01x | 0.19x | 0.41x | 0.02x |
| `asinh neg_1e-12` | 30.42 ns | 499.90 ns | 409.42 ns | 43.80 us | 9.84 us | 12.57 us | 16.43x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 31.96 ns | 416.40 ns | 452.34 ns | 3.61 us | 1.44 us | 18.61 us | 13.03x | 0.12x | 0.29x | 0.02x |
| `asinh 1e6` | 37.45 ns | 405.35 ns | 338.44 ns | 41.46 us | 1.73 us | 7.26 us | 10.82x | 0.01x | 0.23x | 0.06x |
| `atan neg_1e6` | 32.89 ns | 527.53 ns | 541.38 ns | 2.85 us | 1.44 us | 17.87 us | 16.04x | 0.19x | 0.37x | 0.03x |
| `asinh neg_1e6` | 40.44 ns | 370.12 ns | 387.75 ns | 38.16 us | 1.70 us | 7.03 us | 9.15x | 0.01x | 0.22x | 0.05x |
| `acosh 9` | 22.62 ns | 196.11 ns | 307.72 ns | 44.65 us | 2.46 us | 10.02 us | 8.67x | 0.00x | 0.08x | 0.02x |
| `acosh 1_plus_1e-12` | 21.47 ns | 554.66 ns | 1.56 us | 44.36 us | 9.16 us | 11.71 us | 25.84x | 0.01x | 0.06x | 0.05x |
| `acosh 1e6` | 24.75 ns | 196.93 ns | 200.14 ns | 39.11 us | 2.10 us | 10.03 us | 7.96x | 0.01x | 0.09x | 0.02x |
| `acosh e` | 22.55 ns | 556.86 ns | 2.33 us | 43.94 us | 1.79 us | 9.94 us | 24.69x | 0.01x | 0.31x | 0.06x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 6.63 ns | 54.45 ns | 37.37 ns | 25.43 ns | 16.14 ns | 0.95 ns | 8.21x | 2.14x | 3.37x | 57.13x |
| `one` | 7.22 ns | 54.49 ns | 42.88 ns | 44.69 ns | 43.18 ns | 31.36 ns | 7.55x | 1.22x | 1.26x | 1.74x |
| `e` | 7.07 ns | 105.68 ns | 76.13 ns | 56.32 ns | 1.05 us | 230.52 ns | 14.94x | 1.88x | 0.10x | 0.46x |
| `pi` | 7.02 ns | 86.63 ns | 56.69 ns | 45.75 ns | 50.26 ns | 223.34 ns | 12.35x | 1.89x | 1.72x | 0.39x |
| `tau` | 9.69 ns | 69.82 ns | 56.57 ns | 123.36 ns | 106.75 ns | 1.83 us | 7.20x | 0.57x | 0.65x | 0.04x |
| `add` | 6.71 ns | 295.27 ns | 382.60 ns | 55.19 ns | 43.09 ns | 1.50 us | 44.03x | 5.35x | 6.85x | 0.20x |
| `sub` | 7.66 ns | 311.39 ns | 447.17 ns | 57.96 ns | 46.71 ns | 3.21 us | 40.67x | 5.37x | 6.67x | 0.10x |
| `neg` | 6.53 ns | 62.89 ns | 62.71 ns | 25.29 ns | 22.57 ns | 1.08 us | 9.63x | 2.49x | 2.79x | 0.06x |
| `mul` | 7.35 ns | 243.57 ns | 471.34 ns | 59.44 ns | 47.37 ns | 1.55 us | 33.15x | 4.10x | 5.14x | 0.16x |
| `div` | 17.78 ns | 371.25 ns | 431.48 ns | 138.27 ns | 65.29 ns | 3.51 us | 20.88x | 2.68x | 5.69x | 0.11x |
| `reciprocal` | 18.58 ns | 141.02 ns | 117.80 ns | 172.75 ns | 62.03 ns | 1.92 us | 7.59x | 0.82x | 2.27x | 0.07x |
| `reciprocal checked` | 29.43 ns | 144.49 ns | 128.81 ns | 165.78 ns | 62.52 ns | 1.68 us | 4.91x | 0.87x | 2.31x | 0.09x |
| `reciprocal checked abort` | 38.93 ns | 145.03 ns | 168.23 ns | 168.58 ns | 61.83 ns | 1.71 us | 3.72x | 0.86x | 2.35x | 0.08x |
| `pow` | 29.27 ns | 11.93 us | 10.03 us | 58.12 us | 3.10 us | 2.40 us | 407.60x | 0.21x | 3.85x | 4.96x |
| `powi` | 21.83 ns | 572.43 ns | 3.04 us | 302.71 ns | 87.12 ns | 1.70 us | 26.23x | 1.89x | 6.57x | 0.34x |
| `exp` | 17.03 ns | 1.54 us | 2.32 us | 14.22 us | 942.19 ns | 2.62 us | 90.61x | 0.11x | 1.64x | 0.59x |
| `ln` | 20.37 ns | 1.19 us | 1.06 us | 31.19 us | 1.34 us | 3.71 us | 58.50x | 0.04x | 0.89x | 0.32x |
| `log10` | 24.85 ns | 2.38 us | 2.86 us | 37.19 us | 2.80 us | 6.84 us | 95.85x | 0.06x | 0.85x | 0.35x |
| `log10 abort` | 41.47 ns | 3.02 us | 1.95 us | 36.59 us | 2.77 us | 6.82 us | 72.74x | 0.08x | 1.09x | 0.44x |
| `sqrt` | 22.01 ns | 1.68 us | 1.61 us | 5.45 us | 143.39 ns | 2.38 us | 76.17x | 0.31x | 11.69x | 0.71x |
| `sin` | 20.80 ns | 177.48 ns | 177.17 ns | 14.31 us | 1.79 us | 2.86 us | 8.53x | 0.01x | 0.10x | 0.06x |
| `cos` | 17.18 ns | 180.44 ns | 178.92 ns | 12.31 us | 650.51 ns | 1.76 us | 10.50x | 0.01x | 0.28x | 0.10x |
| `tan` | 36.98 ns | 2.92 us | 3.10 us | 29.08 us | 1.57 us | 6.76 us | 78.95x | 0.10x | 1.85x | 0.43x |
| `sinh` | 63.53 ns | 5.22 us | 4.35 us | 3.48 us | 1.20 us | 11.19 us | 82.21x | 1.50x | 4.36x | 0.47x |
| `cosh` | 61.10 ns | 4.34 us | 4.31 us | 8.37 us | 1.09 us | 14.75 us | 71.03x | 0.52x | 3.99x | 0.29x |
| `tanh` | 66.51 ns | 6.23 us | 8.31 us | 3.43 us | 1.44 us | 24.52 us | 93.72x | 1.82x | 4.34x | 0.25x |
| `asin` | 22.53 ns | 715.24 ns | 1.57 us | 21.96 us | 2.48 us | 14.03 us | 31.74x | 0.03x | 0.29x | 0.05x |
| `asin abort` | 37.79 ns | 693.71 ns | 1.62 us | 22.16 us | 2.49 us | 14.92 us | 18.36x | 0.03x | 0.28x | 0.05x |
| `acos` | 22.40 ns | 1.13 us | 2.29 us | 34.06 us | 2.56 us | 23.09 us | 50.24x | 0.03x | 0.44x | 0.05x |
| `acos abort` | 40.52 ns | 923.91 ns | 2.26 us | 43.62 us | 2.73 us | 13.93 us | 22.80x | 0.02x | 0.34x | 0.07x |
| `atan` | 20.09 ns | 478.62 ns | 1.15 us | 19.57 us | 2.31 us | 19.95 us | 23.82x | 0.02x | 0.21x | 0.02x |
| `atan abort` | 36.33 ns | 467.33 ns | 1.13 us | 20.04 us | 2.69 us | 19.46 us | 12.86x | 0.02x | 0.17x | 0.02x |
| `asinh` | 49.88 ns | 579.49 ns | 1.07 us | 41.69 us | 1.77 us | 10.11 us | 11.62x | 0.01x | 0.33x | 0.06x |
| `asinh abort` | 71.06 ns | 471.61 ns | 1.16 us | 40.14 us | 1.66 us | 7.56 us | 6.64x | 0.01x | 0.28x | 0.06x |
| `acosh` | 39.52 ns | 413.29 ns | 1.36 us | 41.90 us | 3.40 us | 11.14 us | 10.46x | 0.01x | 0.12x | 0.04x |
| `acosh abort` | 43.97 ns | 402.51 ns | 1.17 us | 43.70 us | 3.42 us | 17.55 us | 9.15x | 0.01x | 0.12x | 0.02x |
| `atanh` | 26.40 ns | 972.59 ns | 1.50 us | 35.35 us | 1.29 us | 24.84 us | 36.84x | 0.03x | 0.76x | 0.04x |
| `atanh abort` | 44.26 ns | 840.35 ns | 1.50 us | 35.62 us | 1.31 us | 24.63 us | 18.99x | 0.02x | 0.64x | 0.03x |
| `zero status` | 7.38 ns | 10.85 ns | 7.39 ns | 0.98 ns | 6.96 ns | 8.72 ns | 1.47x | 11.09x | 1.56x | 1.24x |
| `zero status abort` | 9.93 ns | 10.43 ns | 10.43 ns | 1.01 ns | 7.90 ns | 8.35 ns | 1.05x | 10.29x | 1.32x | 1.25x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 11.74 ns | 96.96 ns | 82.16 ns | 48.72 ns | - | 1.94 ns | 8.26x | 1.99x | - | 50.08x |
| `one` | 11.48 ns | 83.27 ns | 133.11 ns | 65.24 ns | - | 31.74 ns | 7.25x | 1.28x | - | 2.62x |
| `i` | 11.88 ns | 84.50 ns | 85.68 ns | 65.12 ns | - | 36.93 ns | 7.11x | 1.30x | - | 2.29x |
| `free i` | 11.95 ns | 84.60 ns | 84.64 ns | 65.96 ns | - | 35.06 ns | 7.08x | 1.28x | - | 2.41x |
| `conjugate` | 6.59 ns | 152.84 ns | 143.49 ns | 43.79 ns | - | 1.15 us | 23.19x | 3.49x | - | 0.13x |
| `norm squared` | 11.89 ns | 457.78 ns | 1.47 us | 210.81 ns | - | 4.49 us | 38.49x | 2.17x | - | 0.10x |
| `reciprocal` | 59.55 ns | 1.96 us | 3.30 us | 496.77 ns | - | 11.32 us | 32.83x | 3.94x | - | 0.17x |
| `reciprocal checked` | 68.04 ns | 2.43 us | 3.16 us | 506.42 ns | - | 11.13 us | 35.69x | 4.80x | - | 0.22x |
| `powi` | 96.16 ns | 2.81 us | 14.57 us | 1.52 us | - | 48.23 us | 29.19x | 1.85x | - | 0.06x |
| `powi checked` | 86.64 ns | 2.86 us | 14.74 us | 1.48 us | - | 44.27 us | 32.99x | 1.93x | - | 0.06x |
| `div checked` | 103.74 ns | 3.23 us | 9.80 us | 855.19 ns | - | 27.10 us | 31.11x | 3.77x | - | 0.12x |
| `div real checked` | 40.45 ns | 817.16 ns | 760.82 ns | 272.96 ns | - | 5.75 us | 20.20x | 2.99x | - | 0.14x |
| `from scalar` | 9.32 ns | 96.77 ns | 92.51 ns | 49.08 ns | - | 10.48 ns | 10.38x | 1.97x | - | 9.23x |
| `add` | 11.80 ns | 631.07 ns | 1.20 us | 173.85 ns | - | 3.40 us | 53.49x | 3.63x | - | 0.19x |
| `sub` | 11.21 ns | 633.53 ns | 2.00 us | 120.31 ns | - | 4.88 us | 56.51x | 5.27x | - | 0.13x |
| `neg` | 9.31 ns | 164.06 ns | 161.73 ns | 45.25 ns | - | 2.18 us | 17.63x | 3.63x | - | 0.08x |
| `mul` | 25.50 ns | 1.13 us | 3.54 us | 491.74 ns | - | 10.75 us | 44.35x | 2.30x | - | 0.11x |
| `div` | 76.16 ns | 3.18 us | 6.91 us | 919.53 ns | - | 21.64 us | 41.80x | 3.46x | - | 0.15x |
| `div real` | 33.94 ns | 777.09 ns | 797.27 ns | 377.23 ns | - | 5.27 us | 22.90x | 2.06x | - | 0.15x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 9.23 ns | 829.33 ns | 5.73 us | 305.48 ns | 259.21 ns | 7.23 us | 89.83x | 2.71x | 3.20x | 0.11x |
| `vec3 magnitude` | 24.04 ns | 4.83 us | 8.98 us | 5.69 us | 358.29 ns | 9.07 us | 200.99x | 0.85x | 13.49x | 0.53x |
| `vec3 normalize` | 64.12 ns | 9.14 us | 15.70 us | 6.13 us | 642.62 ns | 19.44 us | 142.60x | 1.49x | 14.23x | 0.47x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 30.06 ns | 283.40 ns | 1.56 us | 85.58 ns | 68.11 ns | 772.64 ns | 9.43x | 3.31x | 4.16x | 0.37x |
| `vec3 zero` | 15.14 ns | 205.81 ns | 212.69 ns | 60.49 ns | 31.54 ns | 2.94 ns | 13.60x | 3.40x | 6.53x | 69.93x |
| `vec3 dot abort` | 60.44 ns | 1.46 us | 5.41 us | 379.20 ns | 207.09 ns | 7.52 us | 24.21x | 3.86x | 7.07x | 0.19x |
| `vec3 magnitude abort` | 88.04 ns | 5.26 us | 11.43 us | 5.92 us | 345.98 ns | 9.73 us | 59.73x | 0.89x | 15.20x | 0.54x |
| `vec3 normalize checked` | 81.60 ns | 8.84 us | 10.94 us | 6.28 us | 556.88 ns | 18.77 us | 108.34x | 1.41x | 15.88x | 0.47x |
| `vec3 normalize checked abort` | 125.65 ns | 9.45 us | 11.20 us | 6.47 us | 634.87 ns | 18.00 us | 75.20x | 1.46x | 14.88x | 0.52x |
| `vec3 div scalar checked` | 48.87 ns | 1.50 us | 1.77 us | 396.09 ns | - | - | 30.75x | 3.79x | - | - |
| `vec3 div scalar checked abort` | 57.72 ns | 1.68 us | 1.82 us | 402.69 ns | - | - | 29.18x | 4.18x | - | - |
| `vec3 add` | 14.17 ns | 1.59 us | 1.97 us | 158.87 ns | 134.62 ns | 4.98 us | 112.19x | 10.00x | 11.81x | 0.32x |
| `vec3 add scalar` | 17.33 ns | 1.64 us | 1.70 us | 158.96 ns | 141.12 ns | 3.92 us | 94.43x | 10.29x | 11.60x | 0.42x |
| `vec3 sub` | 14.07 ns | 1.23 us | 2.13 us | 298.17 ns | 143.20 ns | 7.49 us | 87.23x | 4.12x | 8.57x | 0.16x |
| `vec3 sub scalar` | 17.01 ns | 1.05 us | 1.49 us | 151.36 ns | 129.78 ns | 7.20 us | 62.03x | 6.97x | 8.13x | 0.15x |
| `vec3 neg` | 12.08 ns | 499.64 ns | 473.23 ns | 61.25 ns | 50.20 ns | 3.22 us | 41.36x | 8.16x | 9.95x | 0.16x |
| `vec3 mul scalar` | 17.48 ns | 1.15 us | 2.15 us | 191.94 ns | 136.13 ns | 4.69 us | 65.65x | 5.98x | 8.43x | 0.24x |
| `vec3 div scalar` | 41.08 ns | 1.51 us | 1.82 us | 419.04 ns | 183.40 ns | 8.11 us | 36.64x | 3.59x | 8.21x | 0.19x |
| `vec4 dot` | 9.79 ns | 983.18 ns | 5.04 us | 444.50 ns | 326.26 ns | 9.79 us | 100.43x | 2.21x | 3.01x | 0.10x |
| `vec4 magnitude` | 36.04 ns | 4.68 us | 5.85 us | 5.70 us | 418.86 ns | 12.09 us | 129.79x | 0.82x | 11.17x | 0.39x |
| `vec4 normalize` | 71.17 ns | 9.20 us | 9.28 us | 6.35 us | 786.94 ns | 23.22 us | 129.29x | 1.45x | 11.69x | 0.40x |
| `vec4 add` | 29.56 ns | 1.48 us | 2.15 us | 209.30 ns | 179.85 ns | 5.34 us | 50.06x | 7.07x | 8.23x | 0.28x |
| `vec4 add scalar` | 23.53 ns | 1.40 us | 2.04 us | 223.50 ns | 186.29 ns | 10.90 us | 59.36x | 6.25x | 7.50x | 0.13x |
| `vec4 sub` | 15.87 ns | 1.52 us | 3.30 us | 395.23 ns | 175.92 ns | 19.82 us | 95.64x | 3.84x | 8.63x | 0.08x |
| `vec4 sub scalar` | 21.77 ns | 1.30 us | 1.71 us | 352.63 ns | 172.57 ns | 17.42 us | 59.70x | 3.69x | 7.53x | 0.07x |
| `vec4 neg` | 15.19 ns | 515.97 ns | 496.03 ns | 80.65 ns | 67.50 ns | 7.97 us | 33.97x | 6.40x | 7.64x | 0.06x |
| `vec4 mul scalar` | 22.05 ns | 861.39 ns | 2.21 us | 231.48 ns | 166.04 ns | 9.85 us | 39.07x | 3.72x | 5.19x | 0.09x |
| `vec4 div scalar` | 53.58 ns | 3.39 us | 1.86 us | 700.70 ns | 240.14 ns | 19.03 us | 63.31x | 4.84x | 14.12x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 38.46 ns | 1.91 us | 3.53 us | 961.80 ns | 882.14 ns | 22.73 us | 49.58x | 1.98x | 2.16x | 0.08x |
| `mat3 inverse` | 226.92 ns | 17.96 us | 10.81 us | 3.20 us | 2.55 us | 83.53 us | 79.16x | 5.62x | 7.05x | 0.22x |
| `mat3 mul mat3` | 120.91 ns | 6.76 us | 11.59 us | 2.86 us | 2.46 us | 64.54 us | 55.92x | 2.36x | 2.75x | 0.10x |
| `mat3 transform vec3` | 24.32 ns | 3.40 us | 10.52 us | 1.07 us | 920.27 ns | 20.58 us | 139.73x | 3.17x | 3.69x | 0.17x |
| `mat4 determinant` | 155.21 ns | 5.64 us | 4.42 us | 5.91 us | 4.28 us | 95.82 us | 36.34x | 0.96x | 1.32x | 0.06x |
| `mat4 inverse` | 629.46 ns | 32.73 us | 13.85 us | 12.02 us | 9.50 us | 351.00 us | 52.00x | 2.72x | 3.45x | 0.09x |
| `mat4 mul mat4` | 158.65 ns | 13.30 us | 10.74 us | 6.56 us | 5.63 us | 144.30 us | 83.83x | 2.03x | 2.36x | 0.09x |
| `mat4 transform vec4` | 33.49 ns | 5.68 us | 4.39 us | 1.94 us | 1.73 us | 35.45 us | 169.61x | 2.93x | 3.29x | 0.16x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 136.40 ns | 1.46 us | 5.43 us | 214.05 ns | 199.31 ns | 2.55 us | 10.74x | 6.84x | 7.35x | 0.57x |
| `mat3 zero` | 59.70 ns | 717.03 ns | 769.83 ns | 271.99 ns | 180.59 ns | 11.58 ns | 12.01x | 2.64x | 3.97x | 61.91x |
| `mat3 identity` | 53.61 ns | 821.48 ns | 820.62 ns | 427.41 ns | 212.99 ns | 184.28 ns | 15.32x | 1.92x | 3.86x | 4.46x |
| `mat3 transpose` | 12.29 ns | 936.00 ns | 942.60 ns | 232.49 ns | 211.79 ns | 115.80 ns | 76.16x | 4.03x | 4.42x | 8.08x |
| `mat3 reciprocal` | 389.12 ns | 17.93 us | 36.18 us | 2.99 us | 2.30 us | 84.78 us | 46.08x | 6.00x | 7.78x | 0.21x |
| `mat3 reciprocal checked` | 410.74 ns | 17.55 us | 36.92 us | 3.02 us | 2.32 us | 84.42 us | 42.72x | 5.81x | 7.56x | 0.21x |
| `mat3 inverse checked` | 394.33 ns | 19.37 us | 54.76 us | 5.41 us | 2.35 us | 131.74 us | 49.11x | 3.58x | 8.26x | 0.15x |
| `mat3 inverse checked abort` | 425.33 ns | 18.80 us | 37.46 us | 3.27 us | 2.38 us | 142.20 us | 44.20x | 5.75x | 7.88x | 0.13x |
| `mat3 powi` | 297.93 ns | 14.05 us | 89.97 us | 6.88 us | 6.34 us | 249.03 us | 47.16x | 2.04x | 2.22x | 0.06x |
| `mat3 powi checked` | 294.14 ns | 13.99 us | 88.93 us | 7.72 us | 7.76 us | 154.95 us | 47.58x | 1.81x | 1.80x | 0.09x |
| `mat3 powi checked abort` | 282.71 ns | 13.88 us | 89.10 us | 7.02 us | 6.57 us | 189.63 us | 49.08x | 1.98x | 2.11x | 0.07x |
| `mat3 div scalar checked` | 131.90 ns | 4.79 us | 6.18 us | 1.47 us | 850.84 ns | 22.42 us | 36.30x | 3.25x | 5.63x | 0.21x |
| `mat3 div scalar checked abort` | 190.97 ns | 4.71 us | 6.20 us | 1.50 us | 829.49 ns | 27.47 us | 24.67x | 3.14x | 5.68x | 0.17x |
| `mat3 div matrix checked` | 785.00 ns | 61.04 us | 61.84 us | 5.64 us | 4.58 us | 171.08 us | 77.76x | 10.82x | 13.32x | 0.36x |
| `mat3 div matrix checked abort` | 997.51 ns | 61.49 us | 61.82 us | 5.52 us | 4.87 us | 163.53 us | 61.65x | 11.15x | 12.62x | 0.38x |
| `mat3 add` | 58.54 ns | 3.97 us | 6.10 us | 529.23 ns | 490.61 ns | 11.91 us | 67.89x | 7.51x | 8.10x | 0.33x |
| `mat3 add scalar` | 78.18 ns | 3.03 us | 5.14 us | 792.96 ns | 723.31 ns | 13.32 us | 38.73x | 3.82x | 4.19x | 0.23x |
| `mat3 sub` | 59.15 ns | 4.09 us | 6.10 us | 561.21 ns | 757.02 ns | 39.30 us | 69.22x | 7.30x | 5.41x | 0.10x |
| `mat3 sub scalar` | 43.36 ns | 3.05 us | 5.11 us | 917.20 ns | 1.04 us | 21.85 us | 70.36x | 3.33x | 2.95x | 0.14x |
| `mat3 neg` | 32.94 ns | 1.59 us | 1.60 us | 486.06 ns | 665.86 ns | 8.69 us | 48.35x | 3.28x | 2.39x | 0.18x |
| `mat3 mul scalar` | 56.59 ns | 2.25 us | 6.05 us | 768.59 ns | 1.00 us | 12.46 us | 39.84x | 2.93x | 2.25x | 0.18x |
| `mat3 div scalar` | 153.78 ns | 4.75 us | 6.16 us | 1.68 us | 816.93 ns | 23.78 us | 30.89x | 2.82x | 5.81x | 0.20x |
| `mat3 div matrix` | 556.62 ns | 61.06 us | 61.04 us | 8.90 us | 4.52 us | 166.46 us | 109.70x | 6.86x | 13.52x | 0.37x |
| `mat3 bitxor` | 197.58 ns | 17.02 us | 90.91 us | 7.03 us | 8.96 us | 158.71 us | 86.17x | 2.42x | 1.90x | 0.11x |
| `mat4 zero` | 69.44 ns | 1.28 us | 1.27 us | 513.33 ns | 598.88 ns | 14.53 ns | 18.48x | 2.50x | 2.14x | 88.27x |
| `mat4 identity` | 65.59 ns | 1.45 us | 1.68 us | 576.44 ns | 669.10 ns | 241.64 ns | 22.17x | 2.52x | 2.17x | 6.02x |
| `mat4 transpose` | 10.58 ns | 1.60 us | 1.74 us | 441.28 ns | 629.80 ns | 197.10 ns | 151.66x | 3.64x | 2.55x | 8.14x |
| `mat4 reciprocal` | 632.66 ns | 32.79 us | 63.70 us | 10.93 us | 13.99 us | 331.00 us | 51.84x | 3.00x | 2.34x | 0.10x |
| `mat4 reciprocal checked` | 694.82 ns | 32.73 us | 62.58 us | 10.90 us | 10.63 us | 356.75 us | 47.10x | 3.00x | 3.08x | 0.09x |
| `mat4 powi` | 335.17 ns | 27.77 us | 107.75 us | 15.95 us | 19.09 us | 344.55 us | 82.85x | 1.74x | 1.45x | 0.08x |
| `mat4 powi checked` | 330.28 ns | 27.89 us | 110.13 us | 16.74 us | 21.56 us | 337.81 us | 84.45x | 1.67x | 1.29x | 0.08x |
| `mat4 add` | 85.87 ns | 6.75 us | 7.60 us | 939.92 ns | 1.33 us | 20.59 us | 78.65x | 7.19x | 5.07x | 0.33x |
| `mat4 add scalar` | 83.38 ns | 5.23 us | 7.76 us | 2.22 us | 1.29 us | 24.78 us | 62.71x | 2.35x | 4.05x | 0.21x |
| `mat4 sub` | 84.62 ns | 6.68 us | 7.78 us | 2.03 us | 1.17 us | 57.11 us | 78.90x | 3.29x | 5.72x | 0.12x |
| `mat4 sub scalar` | 81.41 ns | 5.24 us | 7.47 us | 1.47 us | 1.20 us | 79.03 us | 64.40x | 3.57x | 4.37x | 0.07x |
| `mat4 neg` | 68.86 ns | 2.88 us | 2.79 us | 935.78 ns | 1.01 us | 14.48 us | 41.88x | 3.08x | 2.86x | 0.20x |
| `mat4 mul scalar` | 82.42 ns | 4.00 us | 7.88 us | 1.44 us | 1.19 us | 21.77 us | 48.53x | 2.77x | 3.37x | 0.18x |
| `mat4 div scalar` | 121.10 ns | 8.65 us | 8.51 us | 2.56 us | 1.43 us | 40.00 us | 71.44x | 3.38x | 6.03x | 0.22x |
| `mat4 div matrix` | 1.18 us | 123.05 us | 96.79 us | 17.09 us | 18.02 us | 455.10 us | 104.23x | 7.20x | 6.83x | 0.27x |
| `mat4 bitxor` | 322.37 ns | 35.10 us | 113.35 us | 16.22 us | 14.14 us | 426.79 us | 108.87x | 2.16x | 2.48x | 0.08x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 6.35 ns | 207.99 ns | 364.69 ns | - | - | 1.39 us | 32.74x | - | - | 0.15x |
| `scalar add ref_owned` | 14.52 ns | 204.12 ns | 322.30 ns | - | - | 1.32 us | 14.06x | - | - | 0.15x |
| `scalar add refs` | 7.30 ns | 194.94 ns | 359.42 ns | - | - | 1.36 us | 26.71x | - | - | 0.14x |
| `scalar add owned_ref_with_clone` | 7.07 ns | 237.87 ns | 360.67 ns | - | - | - | 33.65x | - | - | - |
| `scalar add ref_owned_with_clone` | 8.46 ns | 425.97 ns | 534.46 ns | - | - | - | 50.37x | - | - | - |
| `scalar sub owned_ref` | 6.46 ns | 217.02 ns | 444.51 ns | - | - | 2.49 us | 33.60x | - | - | 0.09x |
| `scalar sub ref_owned` | 14.83 ns | 219.59 ns | 369.41 ns | - | - | 2.71 us | 14.80x | - | - | 0.08x |
| `scalar sub refs` | 7.35 ns | 237.40 ns | 352.73 ns | - | - | 3.12 us | 32.28x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 11.41 ns | 261.65 ns | 400.74 ns | - | - | - | 22.94x | - | - | - |
| `scalar sub ref_owned_with_clone` | 7.52 ns | 253.49 ns | 424.81 ns | - | - | - | 33.70x | - | - | - |
| `scalar mul owned_ref` | 8.68 ns | 128.01 ns | 398.46 ns | - | - | 1.56 us | 14.75x | - | - | 0.08x |
| `scalar mul ref_owned` | 15.69 ns | 119.28 ns | 399.52 ns | - | - | 1.58 us | 7.60x | - | - | 0.08x |
| `scalar mul refs` | 8.07 ns | 102.97 ns | 386.38 ns | - | - | 1.60 us | 12.76x | - | - | 0.06x |
| `scalar mul owned_ref_with_clone` | 8.24 ns | 145.51 ns | 459.43 ns | - | - | - | 17.65x | - | - | - |
| `scalar mul ref_owned_with_clone` | 7.82 ns | 152.87 ns | 419.99 ns | - | - | - | 19.55x | - | - | - |
| `scalar div owned_ref` | 19.70 ns | 308.71 ns | 376.00 ns | - | - | 2.68 us | 15.67x | - | - | 0.12x |
| `scalar div ref_owned` | 28.96 ns | 298.64 ns | 409.40 ns | - | - | 2.74 us | 10.31x | - | - | 0.11x |
| `scalar div refs` | 22.43 ns | 309.83 ns | 324.24 ns | - | - | 3.00 us | 13.81x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 18.97 ns | 428.71 ns | 385.95 ns | - | - | - | 22.60x | - | - | - |
| `scalar div ref_owned_with_clone` | 16.55 ns | 345.49 ns | 376.98 ns | - | - | - | 20.88x | - | - | - |
| `vec3 add refs` | 13.22 ns | 770.22 ns | 1.49 us | - | - | 4.14 us | 58.25x | - | - | 0.19x |
| `vec3 sub refs` | 13.01 ns | 744.88 ns | 1.54 us | - | - | 9.30 us | 57.24x | - | - | 0.08x |
| `vec3 neg ref` | 11.46 ns | 303.21 ns | 299.01 ns | - | - | 5.00 us | 26.46x | - | - | 0.06x |
| `vec3 add_scalar_ref` | 17.08 ns | 932.99 ns | 1.64 us | - | - | 3.91 us | 54.62x | - | - | 0.24x |
| `vec3 sub_scalar_ref` | 17.13 ns | 933.48 ns | 1.41 us | - | - | 7.06 us | 54.50x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 18.79 ns | 714.60 ns | 2.17 us | - | - | 5.45 us | 38.04x | - | - | 0.13x |
| `vec3 div_scalar_ref` | 60.55 ns | 3.07 us | 2.76 us | - | - | 8.56 us | 50.66x | - | - | 0.36x |
| `vec4 add refs` | 16.57 ns | 1.43 us | 2.60 us | - | - | 6.05 us | 86.53x | - | - | 0.24x |
| `vec4 sub refs` | 16.02 ns | 878.94 ns | 1.44 us | - | - | 9.83 us | 54.87x | - | - | 0.09x |
| `vec4 neg ref` | 17.64 ns | 375.98 ns | 403.11 ns | - | - | 4.20 us | 21.32x | - | - | 0.09x |
| `vec4 add_scalar_ref` | 23.26 ns | 1.23 us | 1.96 us | - | - | 5.17 us | 52.80x | - | - | 0.24x |
| `vec4 sub_scalar_ref` | 23.82 ns | 1.41 us | 2.80 us | - | - | 10.17 us | 59.41x | - | - | 0.14x |
| `vec4 mul_scalar_ref` | 22.49 ns | 843.01 ns | 2.32 us | - | - | 7.24 us | 37.48x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 50.71 ns | 1.93 us | 1.79 us | - | - | 11.65 us | 37.98x | - | - | 0.17x |
| `mat3 add refs` | 70.24 ns | 2.02 us | 4.38 us | - | - | 14.88 us | 28.78x | - | - | 0.14x |
| `mat3 sub refs` | 70.10 ns | 3.07 us | 4.55 us | - | - | 24.27 us | 43.74x | - | - | 0.13x |
| `mat3 mul refs` | 58.49 ns | 5.05 us | 28.11 us | - | - | 62.98 us | 86.32x | - | - | 0.08x |
| `mat3 div refs` | 561.16 ns | 61.54 us | 59.94 us | - | - | 169.35 us | 109.66x | - | - | 0.36x |
| `mat3 neg ref` | 33.69 ns | 925.66 ns | 778.24 ns | - | - | 9.02 us | 27.48x | - | - | 0.10x |
| `mat3 add_scalar_ref` | 67.55 ns | 2.99 us | 5.19 us | - | - | 12.31 us | 44.32x | - | - | 0.24x |
| `mat3 sub_scalar_ref` | 67.86 ns | 2.99 us | 7.30 us | - | - | 22.02 us | 44.12x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 71.61 ns | 2.34 us | 7.30 us | - | - | 13.62 us | 32.70x | - | - | 0.17x |
| `mat3 div_scalar_ref` | 88.34 ns | 4.83 us | 9.92 us | - | - | 27.82 us | 54.72x | - | - | 0.17x |
| `mat4 add refs` | 89.53 ns | 3.06 us | 4.06 us | - | - | 21.40 us | 34.23x | - | - | 0.14x |
| `mat4 sub refs` | 83.36 ns | 3.52 us | 4.10 us | - | - | 43.21 us | 42.17x | - | - | 0.08x |
| `mat4 mul refs` | 110.97 ns | 10.82 us | 27.56 us | - | - | 151.39 us | 97.53x | - | - | 0.07x |
| `mat4 div refs` | 1.14 us | 122.24 us | 92.24 us | - | - | 474.17 us | 107.48x | - | - | 0.26x |
| `mat4 neg ref` | 80.21 ns | 1.40 us | 1.34 us | - | - | 14.36 us | 17.42x | - | - | 0.10x |
| `mat4 add_scalar_ref` | 96.79 ns | 8.08 us | 8.42 us | - | - | 24.24 us | 83.46x | - | - | 0.33x |
| `mat4 sub_scalar_ref` | 91.04 ns | 8.17 us | 7.52 us | - | - | 42.03 us | 89.69x | - | - | 0.19x |
| `mat4 mul_scalar_ref` | 108.93 ns | 3.78 us | 8.46 us | - | - | 21.14 us | 34.67x | - | - | 0.18x |
| `mat4 div_scalar_ref` | 121.40 ns | 8.26 us | 9.03 us | - | - | 39.76 us | 68.03x | - | - | 0.21x |
| `mat3 transform_vec refs` | 22.42 ns | 2.37 us | 12.06 us | - | - | 20.74 us | 105.50x | - | - | 0.11x |
| `mat4 transform_vec refs` | 32.39 ns | 5.13 us | 11.45 us | - | - | 39.22 us | 158.23x | - | - | 0.13x |
| `complex add refs` | 11.54 ns | 412.41 ns | 775.99 ns | - | - | 2.65 us | 35.75x | - | - | 0.16x |
| `complex sub refs` | 11.71 ns | 404.00 ns | 872.38 ns | - | - | 4.96 us | 34.51x | - | - | 0.08x |
| `complex mul refs` | 25.60 ns | 899.72 ns | 3.32 us | - | - | 10.26 us | 35.15x | - | - | 0.09x |
| `complex div refs` | 75.96 ns | 3.19 us | 9.87 us | - | - | 21.99 us | 42.01x | - | - | 0.15x |
| `complex neg ref` | 9.71 ns | 139.56 ns | 218.00 ns | - | - | 2.26 us | 14.37x | - | - | 0.06x |
| `complex div_real_ref` | 42.84 ns | 868.19 ns | 1.42 us | - | - | 5.65 us | 20.27x | - | - | 0.15x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.94 us |
| `astro sin 160` | 13.86 us |
| `astro sin 192` | 13.81 us |
| `astro sin 256` | 17.12 us |
