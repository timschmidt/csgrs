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
| `sin 0.1` | 10.77 ns | 145.08 ns | 144.74 ns | 10.93 us | 758.96 ns | 1.88 us | 13.48x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.43 ns | 144.80 ns | 143.08 ns | 10.64 us | 491.98 ns | 1.69 us | 12.67x | 0.01x | 0.29x | 0.09x |
| `sin 1.23456789` | 11.49 ns | 199.31 ns | 185.04 ns | 12.60 us | 814.95 ns | 1.85 us | 17.34x | 0.02x | 0.24x | 0.11x |
| `cos 1.23456789` | 11.81 ns | 193.20 ns | 177.92 ns | 10.74 us | 595.83 ns | 1.68 us | 16.36x | 0.02x | 0.32x | 0.11x |
| `sin 1e6` | 12.24 ns | 88.50 ns | 88.46 ns | 15.73 us | 1.07 us | 2.04 us | 7.23x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 12.22 ns | 88.55 ns | 88.04 ns | 13.46 us | 811.63 ns | 1.80 us | 7.24x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 65.30 ns | 88.64 ns | 88.73 ns | 18.39 us | 2.83 us | 3.55 us | 1.36x | 0.00x | 0.03x | 0.02x |
| `cos 1e30` | 67.29 ns | 88.62 ns | 88.40 ns | 15.02 us | 946.80 ns | 3.11 us | 1.32x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 11.43 ns | 142.53 ns | 338.73 ns | 12.03 us | 729.33 ns | 1.92 us | 12.47x | 0.01x | 0.20x | 0.07x |
| `cos pi_7` | 11.54 ns | 144.35 ns | 751.07 ns | 10.89 us | 534.08 ns | 1.75 us | 12.51x | 0.01x | 0.27x | 0.08x |
| `sin 1000pi_eps` | 11.44 ns | 88.35 ns | 825.91 ns | 15.83 us | 2.24 us | 2.85 us | 7.72x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.18 ns | 88.07 ns | 802.48 ns | 13.38 us | 566.84 ns | 1.72 us | 7.23x | 0.01x | 0.16x | 0.05x |
| `asin 0.5` | 10.74 ns | 468.75 ns | 469.06 ns | 48.01 us | 2.90 us | 13.24 us | 43.63x | 0.01x | 0.16x | 0.04x |
| `acos 0.5` | 11.02 ns | 1.03 us | 1.03 us | 58.81 us | 2.88 us | 13.20 us | 93.62x | 0.02x | 0.36x | 0.08x |
| `atanh 0.5` | 14.35 ns | 1.44 us | 1.41 us | 34.48 us | 1.65 us | 12.81 us | 100.41x | 0.04x | 0.87x | 0.11x |
| `asin neg_0.999999` | 13.69 ns | 730.24 ns | 1.20 us | 13.51 us | 2.48 us | 12.89 us | 53.35x | 0.05x | 0.29x | 0.06x |
| `acos neg_0.999999` | 14.97 ns | 836.92 ns | 1.75 us | 18.16 us | 2.61 us | 12.89 us | 55.91x | 0.05x | 0.32x | 0.06x |
| `atanh neg_0.999999` | 14.13 ns | 374.93 ns | 575.45 ns | 35.96 us | 1.60 us | 12.59 us | 26.53x | 0.01x | 0.23x | 0.03x |
| `asin 0.999999` | 13.50 ns | 734.53 ns | 1.42 us | 13.47 us | 2.49 us | 12.82 us | 54.40x | 0.05x | 0.29x | 0.06x |
| `acos 0.999999` | 13.96 ns | 618.11 ns | 1.52 us | 18.09 us | 2.68 us | 12.84 us | 44.27x | 0.03x | 0.23x | 0.05x |
| `atanh 0.999999` | 14.31 ns | 440.76 ns | 853.15 ns | 31.71 us | 1.60 us | 12.34 us | 30.81x | 0.01x | 0.28x | 0.04x |
| `asin 1e-12` | 9.18 ns | 461.32 ns | 1.09 us | 7.85 us | 1.39 us | 15.05 us | 50.23x | 0.06x | 0.33x | 0.03x |
| `acos 1e-12` | 9.72 ns | 702.86 ns | 1.56 us | 9.53 us | 1.43 us | 14.83 us | 72.29x | 0.07x | 0.49x | 0.05x |
| `atanh 1e-12` | 9.42 ns | 474.39 ns | 847.72 ns | 35.79 us | 169.99 ns | 19.83 us | 50.37x | 0.01x | 2.79x | 0.02x |
| `atan 0.5` | 14.61 ns | 298.26 ns | 300.62 ns | 35.21 us | 2.67 us | 17.52 us | 20.42x | 0.01x | 0.11x | 0.02x |
| `asinh 0.5` | 26.23 ns | 428.26 ns | 429.05 ns | 38.57 us | 1.59 us | 7.30 us | 16.32x | 0.01x | 0.27x | 0.06x |
| `atan neg_1e-12` | 14.04 ns | 332.83 ns | 546.87 ns | 1.56 us | 1.10 us | 14.95 us | 23.70x | 0.21x | 0.30x | 0.02x |
| `asinh neg_1e-12` | 15.55 ns | 461.46 ns | 399.80 ns | 43.17 us | 8.41 us | 11.64 us | 29.67x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 14.99 ns | 183.73 ns | 185.82 ns | 2.81 us | 1.40 us | 17.66 us | 12.26x | 0.07x | 0.13x | 0.01x |
| `asinh 1e6` | 26.55 ns | 300.23 ns | 308.51 ns | 37.44 us | 1.64 us | 7.06 us | 11.31x | 0.01x | 0.18x | 0.04x |
| `atan neg_1e6` | 15.01 ns | 290.68 ns | 289.27 ns | 2.79 us | 1.41 us | 17.79 us | 19.37x | 0.10x | 0.21x | 0.02x |
| `asinh neg_1e6` | 26.13 ns | 347.08 ns | 345.86 ns | 36.45 us | 1.61 us | 6.94 us | 13.28x | 0.01x | 0.22x | 0.05x |
| `acosh 9` | 12.79 ns | 178.38 ns | 182.63 ns | 42.61 us | 1.59 us | 9.87 us | 13.95x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 12.12 ns | 516.92 ns | 1.41 us | 41.75 us | 8.10 us | 11.31 us | 42.65x | 0.01x | 0.06x | 0.05x |
| `acosh 1e6` | 12.82 ns | 181.63 ns | 181.64 ns | 36.82 us | 1.60 us | 9.76 us | 14.17x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.76 ns | 531.66 ns | 2.29 us | 40.79 us | 1.61 us | 9.61 us | 41.65x | 0.01x | 0.33x | 0.06x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.57 ns | 18.60 ns | 26.66 ns | 15.34 ns | 0.93 ns | 39.49x | 0.70x | 1.21x | 20.04x |
| `one` | 0.47 ns | 23.67 ns | 24.13 ns | 41.42 ns | 29.98 ns | 29.51 ns | 50.05x | 0.57x | 0.79x | 0.80x |
| `e` | 0.47 ns | 90.07 ns | 89.67 ns | 54.57 ns | 1.05 us | 222.89 ns | 190.02x | 1.65x | 0.09x | 0.40x |
| `pi` | 0.47 ns | 50.52 ns | 49.27 ns | 42.83 ns | 47.30 ns | 223.74 ns | 106.78x | 1.18x | 1.07x | 0.23x |
| `tau` | 0.47 ns | 50.49 ns | 49.05 ns | 115.20 ns | 98.83 ns | 1.81 us | 106.65x | 0.44x | 0.51x | 0.03x |
| `add` | 5.09 ns | 209.25 ns | 316.92 ns | 52.49 ns | 42.13 ns | 1.28 us | 41.08x | 3.99x | 4.97x | 0.16x |
| `sub` | 5.12 ns | 215.38 ns | 374.07 ns | 54.46 ns | 44.36 ns | 2.42 us | 42.07x | 3.95x | 4.86x | 0.09x |
| `neg` | 5.02 ns | 39.83 ns | 38.87 ns | 23.38 ns | 21.15 ns | 1.03 us | 7.93x | 1.70x | 1.88x | 0.04x |
| `mul` | 5.34 ns | 134.31 ns | 406.94 ns | 56.93 ns | 44.72 ns | 1.51 us | 25.17x | 2.36x | 3.00x | 0.09x |
| `div` | 8.88 ns | 309.76 ns | 369.17 ns | 133.05 ns | 62.29 ns | 2.58 us | 34.88x | 2.33x | 4.97x | 0.12x |
| `reciprocal` | 8.65 ns | 80.37 ns | 81.38 ns | 157.90 ns | 58.52 ns | 1.53 us | 9.29x | 0.51x | 1.37x | 0.05x |
| `reciprocal checked` | 9.09 ns | 83.15 ns | 83.25 ns | 158.44 ns | 58.99 ns | 1.54 us | 9.15x | 0.52x | 1.41x | 0.05x |
| `reciprocal checked abort` | 26.18 ns | 85.77 ns | 86.80 ns | 158.44 ns | 58.76 ns | 1.53 us | 3.28x | 0.54x | 1.46x | 0.06x |
| `pow` | 30.01 ns | 10.01 us | 9.33 us | 53.99 us | 2.82 us | 2.35 us | 333.74x | 0.19x | 3.55x | 4.26x |
| `powi` | 5.79 ns | 395.72 ns | 2.65 us | 276.88 ns | 83.41 ns | 1.58 us | 68.31x | 1.43x | 4.74x | 0.25x |
| `exp` | 19.69 ns | 1.48 us | 2.12 us | 13.69 us | 904.75 ns | 1.87 us | 75.10x | 0.11x | 1.63x | 0.79x |
| `ln` | 10.87 ns | 1.20 us | 1.20 us | 29.44 us | 1.31 us | 1.84 us | 110.53x | 0.04x | 0.92x | 0.65x |
| `log10` | 13.36 ns | 3.81 us | 3.39 us | 35.17 us | 2.76 us | 6.59 us | 284.78x | 0.11x | 1.38x | 0.58x |
| `log10 abort` | 16.90 ns | 3.82 us | 3.33 us | 34.93 us | 2.75 us | 6.58 us | 226.31x | 0.11x | 1.39x | 0.58x |
| `sqrt` | 8.43 ns | 1.50 us | 1.50 us | 4.87 us | 93.30 ns | 1.48 us | 178.46x | 0.31x | 16.13x | 1.01x |
| `sin` | 14.79 ns | 129.35 ns | 120.15 ns | 13.68 us | 1.21 us | 2.24 us | 8.75x | 0.01x | 0.11x | 0.06x |
| `cos` | 18.46 ns | 129.11 ns | 121.29 ns | 11.85 us | 621.36 ns | 1.75 us | 6.99x | 0.01x | 0.21x | 0.07x |
| `tan` | 24.86 ns | 166.91 ns | 172.26 ns | 28.73 us | 1.57 us | 6.54 us | 6.71x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.46 ns | 4.03 us | 4.06 us | 3.28 us | 1.10 us | 10.62 us | 218.32x | 1.23x | 3.67x | 0.38x |
| `cosh` | 18.47 ns | 3.99 us | 4.02 us | 7.91 us | 1.05 us | 9.31 us | 216.02x | 0.50x | 3.80x | 0.43x |
| `tanh` | 22.74 ns | 8.19 us | 8.07 us | 3.37 us | 1.17 us | 22.82 us | 360.20x | 2.43x | 6.97x | 0.36x |
| `asin` | 13.12 ns | 630.38 ns | 1.49 us | 20.88 us | 2.34 us | 13.76 us | 48.04x | 0.03x | 0.27x | 0.05x |
| `asin abort` | 17.17 ns | 627.38 ns | 1.46 us | 20.75 us | 2.35 us | 13.62 us | 36.54x | 0.03x | 0.27x | 0.05x |
| `acos` | 14.18 ns | 831.65 ns | 2.10 us | 26.16 us | 2.45 us | 13.69 us | 58.65x | 0.03x | 0.34x | 0.06x |
| `acos abort` | 17.40 ns | 844.12 ns | 2.16 us | 26.39 us | 2.45 us | 13.84 us | 48.50x | 0.03x | 0.34x | 0.06x |
| `atan` | 14.95 ns | 268.45 ns | 931.68 ns | 18.10 us | 2.20 us | 18.68 us | 17.95x | 0.01x | 0.12x | 0.01x |
| `atan abort` | 21.04 ns | 272.15 ns | 942.12 ns | 18.26 us | 2.23 us | 18.61 us | 12.94x | 0.01x | 0.12x | 0.01x |
| `asinh` | 29.31 ns | 405.07 ns | 1.09 us | 38.30 us | 1.63 us | 7.51 us | 13.82x | 0.01x | 0.25x | 0.05x |
| `asinh abort` | 30.42 ns | 412.73 ns | 1.07 us | 38.66 us | 1.62 us | 7.49 us | 13.57x | 0.01x | 0.25x | 0.06x |
| `acosh` | 12.45 ns | 359.79 ns | 1.13 us | 40.17 us | 3.33 us | 10.52 us | 28.90x | 0.01x | 0.11x | 0.03x |
| `acosh abort` | 16.23 ns | 343.11 ns | 1.13 us | 39.55 us | 3.33 us | 10.40 us | 21.14x | 0.01x | 0.10x | 0.03x |
| `atanh` | 13.61 ns | 685.35 ns | 1.37 us | 34.08 us | 1.29 us | 14.83 us | 50.36x | 0.02x | 0.53x | 0.05x |
| `atanh abort` | 16.85 ns | 705.91 ns | 1.36 us | 33.87 us | 1.29 us | 14.73 us | 41.90x | 0.02x | 0.55x | 0.05x |
| `zero status` | 1.19 ns | 1.71 ns | 1.70 ns | 0.94 ns | 6.71 ns | 7.91 ns | 1.44x | 1.81x | 0.25x | 0.22x |
| `zero status abort` | 1.41 ns | 3.05 ns | 3.13 ns | 0.94 ns | 6.67 ns | 7.84 ns | 2.16x | 3.24x | 0.46x | 0.39x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.92 ns | 35.88 ns | 35.65 ns | 47.56 ns | - | 1.85 ns | 38.82x | 0.75x | - | 19.41x |
| `one` | 5.28 ns | 39.20 ns | 38.56 ns | 62.97 ns | - | 30.02 ns | 7.42x | 0.62x | - | 1.31x |
| `i` | 0.93 ns | 41.36 ns | 41.12 ns | 63.39 ns | - | 29.46 ns | 44.54x | 0.65x | - | 1.40x |
| `free i` | 0.92 ns | 41.56 ns | 40.98 ns | 63.39 ns | - | 29.20 ns | 45.00x | 0.66x | - | 1.42x |
| `conjugate` | 2.13 ns | 83.28 ns | 83.35 ns | 40.45 ns | - | 1.05 us | 39.03x | 2.06x | - | 0.08x |
| `norm squared` | 5.74 ns | 324.03 ns | 1.41 us | 153.75 ns | - | 4.31 us | 56.45x | 2.11x | - | 0.08x |
| `reciprocal` | 14.00 ns | 1.76 us | 2.81 us | 441.16 ns | - | 10.65 us | 125.48x | 3.98x | - | 0.16x |
| `reciprocal checked` | 15.44 ns | 1.77 us | 2.82 us | 439.56 ns | - | 10.66 us | 114.76x | 4.03x | - | 0.17x |
| `powi` | 18.23 ns | 2.07 us | 13.63 us | 1.40 us | - | 44.56 us | 113.26x | 1.47x | - | 0.05x |
| `powi checked` | 17.37 ns | 2.05 us | 13.42 us | 1.41 us | - | 43.07 us | 118.27x | 1.46x | - | 0.05x |
| `div checked` | 18.00 ns | 2.76 us | 6.34 us | 778.28 ns | - | 21.64 us | 153.20x | 3.54x | - | 0.13x |
| `div real checked` | 9.33 ns | 671.00 ns | 645.17 ns | 261.64 ns | - | 5.14 us | 71.91x | 2.56x | - | 0.13x |
| `from scalar` | 1.40 ns | 56.25 ns | 55.62 ns | 44.41 ns | - | 10.11 ns | 40.06x | 1.27x | - | 5.56x |
| `add` | 6.01 ns | 469.07 ns | 840.25 ns | 105.29 ns | - | 2.55 us | 78.11x | 4.45x | - | 0.18x |
| `sub` | 6.05 ns | 479.27 ns | 862.97 ns | 113.23 ns | - | 4.71 us | 79.26x | 4.23x | - | 0.10x |
| `neg` | 2.56 ns | 85.31 ns | 85.08 ns | 41.29 ns | - | 2.15 us | 33.36x | 2.07x | - | 0.04x |
| `mul` | 11.05 ns | 850.79 ns | 3.15 us | 305.63 ns | - | 10.03 us | 76.99x | 2.78x | - | 0.08x |
| `div` | 17.63 ns | 2.71 us | 6.34 us | 767.95 ns | - | 21.91 us | 153.48x | 3.52x | - | 0.12x |
| `div real` | 9.98 ns | 663.72 ns | 629.44 ns | 262.82 ns | - | 5.22 us | 66.47x | 2.53x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.71 ns | 212.49 ns | 2.46 us | 309.17 ns | 248.99 ns | 7.10 us | 31.66x | 0.69x | 0.85x | 0.03x |
| `vec3 magnitude` | 9.41 ns | 3.97 us | 5.44 us | 5.45 us | 342.83 ns | 8.61 us | 421.49x | 0.73x | 11.57x | 0.46x |
| `vec3 normalize` | 24.71 ns | 8.15 us | 11.38 us | 5.96 us | 592.33 ns | 16.72 us | 329.71x | 1.37x | 13.76x | 0.49x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.04 ns | 215.55 ns | 1.44 us | 68.48 ns | 56.70 ns | 708.38 ns | 70.99x | 3.15x | 3.80x | 0.30x |
| `vec3 zero` | 1.39 ns | 131.24 ns | 131.21 ns | 65.37 ns | 30.51 ns | 2.76 ns | 94.34x | 2.01x | 4.30x | 47.51x |
| `vec3 dot abort` | 7.23 ns | 214.98 ns | 873.92 ns | 259.81 ns | 209.89 ns | 7.12 us | 29.73x | 0.83x | 1.02x | 0.03x |
| `vec3 magnitude abort` | 15.23 ns | 4.00 us | 3.21 us | 5.34 us | 320.02 ns | 8.60 us | 262.31x | 0.75x | 12.49x | 0.46x |
| `vec3 normalize checked` | 25.62 ns | 8.23 us | 7.19 us | 5.85 us | 541.79 ns | 16.70 us | 321.14x | 1.41x | 15.19x | 0.49x |
| `vec3 normalize checked abort` | 28.64 ns | 8.22 us | 7.24 us | 5.89 us | 547.47 ns | 16.87 us | 287.14x | 1.40x | 15.02x | 0.49x |
| `vec3 div scalar checked` | 15.70 ns | 1.35 us | 1.61 us | 396.05 ns | - | - | 85.87x | 3.40x | - | - |
| `vec3 div scalar checked abort` | 18.26 ns | 1.33 us | 1.58 us | 396.87 ns | - | - | 73.01x | 3.36x | - | - |
| `vec3 add` | 6.67 ns | 921.52 ns | 1.69 us | 147.36 ns | 126.07 ns | 3.98 us | 138.13x | 6.25x | 7.31x | 0.23x |
| `vec3 add scalar` | 6.33 ns | 745.33 ns | 1.45 us | 149.34 ns | 134.79 ns | 3.82 us | 117.78x | 4.99x | 5.53x | 0.20x |
| `vec3 sub` | 6.72 ns | 937.68 ns | 1.75 us | 166.48 ns | 134.80 ns | 7.26 us | 139.56x | 5.63x | 6.96x | 0.13x |
| `vec3 sub scalar` | 6.35 ns | 730.17 ns | 1.22 us | 142.85 ns | 126.14 ns | 7.06 us | 114.95x | 5.11x | 5.79x | 0.10x |
| `vec3 neg` | 3.70 ns | 140.74 ns | 140.11 ns | 57.99 ns | 50.06 ns | 3.09 us | 37.99x | 2.43x | 2.81x | 0.05x |
| `vec3 mul scalar` | 6.88 ns | 507.84 ns | 2.00 us | 165.23 ns | 123.41 ns | 4.38 us | 73.83x | 3.07x | 4.12x | 0.12x |
| `vec3 div scalar` | 18.59 ns | 1.34 us | 1.59 us | 391.80 ns | 176.94 ns | 7.89 us | 72.28x | 3.43x | 7.60x | 0.17x |
| `vec4 dot` | 7.20 ns | 235.71 ns | 618.51 ns | 442.57 ns | 316.51 ns | 10.25 us | 32.76x | 0.53x | 0.74x | 0.02x |
| `vec4 magnitude` | 12.57 ns | 3.74 us | 2.75 us | 5.58 us | 409.32 ns | 11.32 us | 297.38x | 0.67x | 9.13x | 0.33x |
| `vec4 normalize` | 35.96 ns | 8.46 us | 6.50 us | 6.26 us | 703.33 ns | 22.06 us | 235.24x | 1.35x | 12.03x | 0.38x |
| `vec4 add` | 7.21 ns | 1.16 us | 1.79 us | 210.79 ns | 178.73 ns | 5.23 us | 160.62x | 5.50x | 6.48x | 0.22x |
| `vec4 add scalar` | 6.93 ns | 1.00 us | 1.70 us | 223.25 ns | 180.29 ns | 5.08 us | 144.89x | 4.50x | 5.57x | 0.20x |
| `vec4 sub` | 5.03 ns | 1.16 us | 1.68 us | 221.51 ns | 180.67 ns | 9.71 us | 230.32x | 5.23x | 6.41x | 0.12x |
| `vec4 sub scalar` | 4.55 ns | 996.11 ns | 1.46 us | 208.70 ns | 172.49 ns | 9.69 us | 219.06x | 4.77x | 5.77x | 0.10x |
| `vec4 neg` | 4.94 ns | 185.59 ns | 190.14 ns | 90.33 ns | 61.83 ns | 3.98 us | 37.53x | 2.05x | 3.00x | 0.05x |
| `vec4 mul scalar` | 7.34 ns | 673.31 ns | 2.08 us | 226.64 ns | 155.08 ns | 5.63 us | 91.77x | 2.97x | 4.34x | 0.12x |
| `vec4 div scalar` | 14.31 ns | 1.76 us | 1.57 us | 536.24 ns | 222.20 ns | 10.00 us | 122.90x | 3.28x | 7.91x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.36 ns | 727.46 ns | 2.18 us | 955.81 ns | 846.86 ns | 21.91 us | 58.87x | 0.76x | 0.86x | 0.03x |
| `mat3 inverse` | 86.27 ns | 15.27 us | 8.99 us | 3.11 us | 2.46 us | 81.28 us | 176.98x | 4.91x | 6.19x | 0.19x |
| `mat3 mul mat3` | 40.25 ns | 2.98 us | 6.93 us | 2.74 us | 2.39 us | 60.73 us | 73.99x | 1.09x | 1.25x | 0.05x |
| `mat3 transform vec3` | 15.71 ns | 1.32 us | 4.42 us | 998.04 ns | 878.84 ns | 19.95 us | 84.29x | 1.33x | 1.51x | 0.07x |
| `mat4 determinant` | 35.43 ns | 2.17 us | 1.78 us | 4.54 us | 4.13 us | 93.54 us | 61.24x | 0.48x | 0.53x | 0.02x |
| `mat4 inverse` | 139.80 ns | 24.58 us | 8.59 us | 11.13 us | 9.16 us | 336.20 us | 175.85x | 2.21x | 2.68x | 0.07x |
| `mat4 mul mat4` | 76.57 ns | 5.47 us | 4.11 us | 6.43 us | 5.32 us | 140.43 us | 71.46x | 0.85x | 1.03x | 0.04x |
| `mat4 transform vec4` | 24.91 ns | 2.06 us | 1.64 us | 1.89 us | 1.62 us | 34.95 us | 82.58x | 1.09x | 1.27x | 0.06x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.62 ns | 1.13 us | 4.27 us | 213.28 ns | 233.84 ns | 2.06 us | 32.67x | 5.30x | 4.84x | 0.55x |
| `mat3 zero` | 16.10 ns | 563.63 ns | 563.11 ns | 247.49 ns | 207.73 ns | 11.80 ns | 35.01x | 2.28x | 2.71x | 47.75x |
| `mat3 identity` | 9.74 ns | 611.18 ns | 609.81 ns | 307.31 ns | 239.65 ns | 139.83 ns | 62.78x | 1.99x | 2.55x | 4.37x |
| `mat3 transpose` | 8.90 ns | 701.39 ns | 691.30 ns | 234.63 ns | 209.47 ns | 120.15 ns | 78.78x | 2.99x | 3.35x | 5.84x |
| `mat3 reciprocal` | 78.27 ns | 15.22 us | 25.26 us | 2.86 us | 2.30 us | 81.93 us | 194.50x | 5.32x | 6.63x | 0.19x |
| `mat3 reciprocal checked` | 79.38 ns | 15.25 us | 25.25 us | 2.91 us | 2.30 us | 82.46 us | 192.09x | 5.25x | 6.64x | 0.18x |
| `mat3 inverse checked` | 78.47 ns | 15.22 us | 25.19 us | 2.91 us | 2.30 us | 81.61 us | 193.94x | 5.23x | 6.61x | 0.19x |
| `mat3 inverse checked abort` | 114.63 ns | 15.46 us | 25.31 us | 2.90 us | 2.30 us | 82.26 us | 134.86x | 5.33x | 6.73x | 0.19x |
| `mat3 powi` | 89.53 ns | 5.57 us | 39.63 us | 6.96 us | 6.13 us | 150.91 us | 62.24x | 0.80x | 0.91x | 0.04x |
| `mat3 powi checked` | 88.47 ns | 5.60 us | 39.97 us | 6.94 us | 6.12 us | 151.50 us | 63.25x | 0.81x | 0.91x | 0.04x |
| `mat3 powi checked abort` | 90.70 ns | 5.65 us | 40.36 us | 6.95 us | 6.11 us | 151.73 us | 62.24x | 0.81x | 0.92x | 0.04x |
| `mat3 div scalar checked` | 24.09 ns | 4.36 us | 5.64 us | 1.42 us | 805.22 ns | 22.39 us | 181.12x | 3.07x | 5.42x | 0.19x |
| `mat3 div scalar checked abort` | 29.30 ns | 4.26 us | 5.53 us | 1.43 us | 805.57 ns | 22.79 us | 145.50x | 2.99x | 5.29x | 0.19x |
| `mat3 div matrix checked` | 116.52 ns | 18.58 us | 58.03 us | 5.31 us | 4.42 us | 161.15 us | 159.48x | 3.50x | 4.20x | 0.12x |
| `mat3 div matrix checked abort` | 129.79 ns | 18.45 us | 58.34 us | 5.36 us | 4.43 us | 160.77 us | 142.17x | 3.44x | 4.16x | 0.11x |
| `mat3 add` | 14.44 ns | 3.15 us | 5.22 us | 510.69 ns | 488.66 ns | 11.79 us | 218.25x | 6.17x | 6.45x | 0.27x |
| `mat3 add scalar` | 11.95 ns | 2.40 us | 4.53 us | 770.82 ns | 700.80 ns | 12.13 us | 200.44x | 3.11x | 3.42x | 0.20x |
| `mat3 sub` | 12.68 ns | 3.13 us | 5.12 us | 531.58 ns | 509.99 ns | 21.26 us | 246.94x | 5.89x | 6.14x | 0.15x |
| `mat3 sub scalar` | 10.77 ns | 2.40 us | 4.35 us | 771.51 ns | 698.61 ns | 21.57 us | 223.38x | 3.12x | 3.44x | 0.11x |
| `mat3 neg` | 10.18 ns | 603.63 ns | 595.60 ns | 473.43 ns | 445.39 ns | 8.55 us | 59.32x | 1.28x | 1.36x | 0.07x |
| `mat3 mul scalar` | 13.34 ns | 1.74 us | 5.70 us | 767.64 ns | 660.01 ns | 12.17 us | 130.60x | 2.27x | 2.64x | 0.14x |
| `mat3 div scalar` | 23.74 ns | 4.38 us | 5.65 us | 1.42 us | 805.46 ns | 22.45 us | 184.70x | 3.08x | 5.44x | 0.20x |
| `mat3 div matrix` | 130.22 ns | 18.62 us | 58.39 us | 5.35 us | 4.46 us | 160.69 us | 143.02x | 3.48x | 4.18x | 0.12x |
| `mat3 bitxor` | 89.32 ns | 5.56 us | 39.96 us | 6.94 us | 6.12 us | 152.02 us | 62.24x | 0.80x | 0.91x | 0.04x |
| `mat4 zero` | 11.42 ns | 897.06 ns | 892.03 ns | 468.14 ns | 350.76 ns | 14.81 ns | 78.57x | 1.92x | 2.56x | 60.59x |
| `mat4 identity` | 10.98 ns | 1.09 us | 1.07 us | 557.40 ns | 418.80 ns | 214.45 ns | 99.62x | 1.96x | 2.61x | 5.10x |
| `mat4 transpose` | 9.21 ns | 1.10 us | 1.09 us | 419.14 ns | 361.48 ns | 179.78 ns | 119.41x | 2.62x | 3.04x | 6.11x |
| `mat4 reciprocal` | 142.03 ns | 24.91 us | 42.01 us | 10.49 us | 8.86 us | 342.55 us | 175.39x | 2.37x | 2.81x | 0.07x |
| `mat4 reciprocal checked` | 145.50 ns | 25.06 us | 42.46 us | 10.66 us | 8.89 us | 343.53 us | 172.26x | 2.35x | 2.82x | 0.07x |
| `mat4 powi` | 166.41 ns | 10.45 us | 53.66 us | 15.56 us | 13.97 us | 344.75 us | 62.82x | 0.67x | 0.75x | 0.03x |
| `mat4 powi checked` | 165.93 ns | 10.41 us | 53.48 us | 15.56 us | 14.02 us | 347.33 us | 62.71x | 0.67x | 0.74x | 0.03x |
| `mat4 add` | 51.13 ns | 5.02 us | 5.80 us | 899.66 ns | 871.14 ns | 19.79 us | 98.08x | 5.57x | 5.76x | 0.25x |
| `mat4 add scalar` | 20.53 ns | 4.03 us | 6.59 us | 1.41 us | 1.19 us | 20.71 us | 196.38x | 2.85x | 3.39x | 0.19x |
| `mat4 sub` | 38.65 ns | 4.91 us | 5.86 us | 943.57 ns | 901.05 ns | 35.35 us | 127.12x | 5.21x | 5.45x | 0.14x |
| `mat4 sub scalar` | 15.17 ns | 4.07 us | 6.47 us | 1.41 us | 1.16 us | 37.41 us | 268.03x | 2.87x | 3.50x | 0.11x |
| `mat4 neg` | 14.37 ns | 1.02 us | 1.00 us | 903.89 ns | 743.90 ns | 13.86 us | 71.22x | 1.13x | 1.38x | 0.07x |
| `mat4 mul scalar` | 24.35 ns | 2.91 us | 7.51 us | 1.41 us | 1.12 us | 20.50 us | 119.32x | 2.06x | 2.59x | 0.14x |
| `mat4 div scalar` | 33.66 ns | 7.49 us | 7.75 us | 2.60 us | 1.36 us | 38.16 us | 222.44x | 2.88x | 5.52x | 0.20x |
| `mat4 div matrix` | 190.95 ns | 28.62 us | 85.74 us | 16.56 us | 14.29 us | 528.33 us | 149.90x | 1.73x | 2.00x | 0.05x |
| `mat4 bitxor` | 166.04 ns | 10.44 us | 54.05 us | 15.56 us | 14.02 us | 348.76 us | 62.86x | 0.67x | 0.74x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.58 ns | 175.83 ns | 276.94 ns | - | - | 1.27 us | 49.14x | - | - | 0.14x |
| `scalar add ref_owned` | 12.09 ns | 172.91 ns | 276.14 ns | - | - | 1.29 us | 14.30x | - | - | 0.13x |
| `scalar add refs` | 5.37 ns | 154.16 ns | 260.47 ns | - | - | 1.29 us | 28.71x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 9.10 ns | 192.85 ns | 297.27 ns | - | - | - | 21.20x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.39 ns | 182.56 ns | 289.98 ns | - | - | - | 33.84x | - | - | - |
| `scalar sub owned_ref` | 3.94 ns | 180.64 ns | 329.50 ns | - | - | 2.38 us | 45.80x | - | - | 0.08x |
| `scalar sub ref_owned` | 12.36 ns | 183.10 ns | 336.58 ns | - | - | 2.41 us | 14.81x | - | - | 0.08x |
| `scalar sub refs` | 5.55 ns | 159.51 ns | 314.65 ns | - | - | 2.43 us | 28.72x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 8.89 ns | 200.89 ns | 355.70 ns | - | - | - | 22.59x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.65 ns | 188.23 ns | 347.38 ns | - | - | - | 33.32x | - | - | - |
| `scalar mul owned_ref` | 4.20 ns | 97.80 ns | 369.15 ns | - | - | 1.52 us | 23.31x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.17 ns | 95.63 ns | 372.09 ns | - | - | 1.53 us | 7.26x | - | - | 0.06x |
| `scalar mul refs` | 6.36 ns | 79.87 ns | 355.42 ns | - | - | 1.53 us | 12.56x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.78 ns | 112.00 ns | 384.92 ns | - | - | - | 11.45x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.16 ns | 109.54 ns | 383.74 ns | - | - | - | 17.79x | - | - | - |
| `scalar div owned_ref` | 5.84 ns | 263.72 ns | 310.94 ns | - | - | 2.64 us | 45.13x | - | - | 0.10x |
| `scalar div ref_owned` | 17.15 ns | 262.24 ns | 313.64 ns | - | - | 2.59 us | 15.29x | - | - | 0.10x |
| `scalar div refs` | 6.86 ns | 251.15 ns | 298.55 ns | - | - | 2.56 us | 36.60x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.30 ns | 277.28 ns | 337.98 ns | - | - | - | 18.12x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.13 ns | 275.48 ns | 335.85 ns | - | - | - | 18.21x | - | - | - |
| `vec3 add refs` | 6.02 ns | 619.26 ns | 1.38 us | - | - | 4.09 us | 102.80x | - | - | 0.15x |
| `vec3 sub refs` | 6.05 ns | 619.29 ns | 1.47 us | - | - | 7.34 us | 102.38x | - | - | 0.08x |
| `vec3 neg ref` | 3.27 ns | 160.49 ns | 162.22 ns | - | - | 3.08 us | 49.13x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.40 ns | 773.30 ns | 1.46 us | - | - | 3.94 us | 120.86x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.54 ns | 749.67 ns | 1.24 us | - | - | 7.52 us | 114.58x | - | - | 0.10x |
| `vec3 mul_scalar_ref` | 6.90 ns | 506.19 ns | 2.03 us | - | - | 4.45 us | 73.36x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.03 ns | 1.32 us | 1.53 us | - | - | 7.86 us | 164.40x | - | - | 0.17x |
| `vec4 add refs` | 6.68 ns | 764.81 ns | 1.39 us | - | - | 5.48 us | 114.57x | - | - | 0.14x |
| `vec4 sub refs` | 3.07 ns | 754.94 ns | 1.28 us | - | - | 9.67 us | 245.71x | - | - | 0.08x |
| `vec4 neg ref` | 4.21 ns | 230.16 ns | 229.44 ns | - | - | 4.02 us | 54.70x | - | - | 0.06x |
| `vec4 add_scalar_ref` | 6.95 ns | 992.48 ns | 1.70 us | - | - | 5.20 us | 142.87x | - | - | 0.19x |
| `vec4 sub_scalar_ref` | 4.21 ns | 1.01 us | 1.45 us | - | - | 9.37 us | 239.12x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.45 ns | 655.69 ns | 2.06 us | - | - | 5.64 us | 87.98x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.79 ns | 1.75 us | 1.51 us | - | - | 10.11 us | 148.74x | - | - | 0.17x |
| `mat3 add refs` | 11.63 ns | 1.73 us | 3.80 us | - | - | 11.78 us | 149.13x | - | - | 0.15x |
| `mat3 sub refs` | 15.94 ns | 1.74 us | 3.82 us | - | - | 21.99 us | 109.11x | - | - | 0.08x |
| `mat3 mul refs` | 31.74 ns | 1.91 us | 12.77 us | - | - | 62.04 us | 60.16x | - | - | 0.03x |
| `mat3 div refs` | 121.42 ns | 17.23 us | 57.73 us | - | - | 162.68 us | 141.88x | - | - | 0.11x |
| `mat3 neg ref` | 9.54 ns | 493.22 ns | 484.55 ns | - | - | 8.56 us | 51.68x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 49.35 ns | 2.46 us | 4.54 us | - | - | 12.14 us | 49.80x | - | - | 0.20x |
| `mat3 sub_scalar_ref` | 50.06 ns | 2.48 us | 4.39 us | - | - | 21.62 us | 49.49x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 51.61 ns | 1.80 us | 5.62 us | - | - | 12.24 us | 34.90x | - | - | 0.15x |
| `mat3 div_scalar_ref` | 24.85 ns | 4.25 us | 5.58 us | - | - | 22.36 us | 171.08x | - | - | 0.19x |
| `mat4 add refs` | 17.97 ns | 2.64 us | 3.43 us | - | - | 19.30 us | 146.68x | - | - | 0.14x |
| `mat4 sub refs` | 20.29 ns | 2.64 us | 3.60 us | - | - | 35.44 us | 130.25x | - | - | 0.07x |
| `mat4 mul refs` | 74.48 ns | 3.75 us | 17.39 us | - | - | 143.53 us | 50.30x | - | - | 0.03x |
| `mat4 div refs` | 187.95 ns | 27.08 us | 84.97 us | - | - | 539.32 us | 144.06x | - | - | 0.05x |
| `mat4 neg ref` | 12.49 ns | 846.05 ns | 848.60 ns | - | - | 13.99 us | 67.75x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 58.64 ns | 4.06 us | 6.60 us | - | - | 20.75 us | 69.24x | - | - | 0.20x |
| `mat4 sub_scalar_ref` | 44.72 ns | 4.08 us | 6.47 us | - | - | 37.20 us | 91.22x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 62.50 ns | 2.93 us | 7.40 us | - | - | 20.31 us | 46.85x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 28.75 ns | 7.36 us | 7.58 us | - | - | 37.92 us | 256.02x | - | - | 0.19x |
| `mat3 transform_vec refs` | 14.45 ns | 727.87 ns | 3.96 us | - | - | 20.13 us | 50.36x | - | - | 0.04x |
| `mat4 transform_vec refs` | 22.90 ns | 1.05 us | 4.91 us | - | - | 35.88 us | 45.88x | - | - | 0.03x |
| `complex add refs` | 7.67 ns | 320.82 ns | 691.38 ns | - | - | 2.59 us | 41.85x | - | - | 0.12x |
| `complex sub refs` | 7.98 ns | 325.34 ns | 725.50 ns | - | - | 4.76 us | 40.79x | - | - | 0.07x |
| `complex mul refs` | 7.95 ns | 720.76 ns | 3.05 us | - | - | 10.00 us | 90.64x | - | - | 0.07x |
| `complex div refs` | 16.45 ns | 2.55 us | 6.15 us | - | - | 21.91 us | 154.83x | - | - | 0.12x |
| `complex neg ref` | 2.34 ns | 72.16 ns | 70.48 ns | - | - | 2.11 us | 30.87x | - | - | 0.03x |
| `complex div_real_ref` | 7.19 ns | 630.37 ns | 588.44 ns | - | - | 5.21 us | 87.68x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.39 us |
| `astro sin 160` | 14.05 us |
| `astro sin 192` | 13.99 us |
| `astro sin 256` | 15.85 us |
