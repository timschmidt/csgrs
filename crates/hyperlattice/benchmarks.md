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
| `sin 0.1` | 10.97 ns | 143.60 ns | 144.83 ns | 10.87 us | 747.30 ns | 1.88 us | 13.09x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.70 ns | 142.18 ns | 143.40 ns | 10.49 us | 489.55 ns | 1.71 us | 12.15x | 0.01x | 0.29x | 0.08x |
| `sin 1.23456789` | 11.73 ns | 214.95 ns | 202.14 ns | 12.23 us | 801.59 ns | 1.82 us | 18.32x | 0.02x | 0.27x | 0.12x |
| `cos 1.23456789` | 12.08 ns | 192.53 ns | 179.50 ns | 10.63 us | 583.71 ns | 1.64 us | 15.94x | 0.02x | 0.33x | 0.12x |
| `sin 1e6` | 12.56 ns | 87.44 ns | 88.94 ns | 15.81 us | 1.06 us | 2.05 us | 6.96x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 12.50 ns | 86.89 ns | 88.81 ns | 13.79 us | 818.63 ns | 1.85 us | 6.95x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 66.38 ns | 85.73 ns | 88.38 ns | 18.28 us | 2.83 us | 3.57 us | 1.29x | 0.00x | 0.03x | 0.02x |
| `cos 1e30` | 68.89 ns | 87.85 ns | 87.44 ns | 15.42 us | 944.39 ns | 3.10 us | 1.28x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 11.78 ns | 145.11 ns | 337.01 ns | 11.90 us | 740.93 ns | 1.87 us | 12.32x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.72 ns | 142.29 ns | 784.70 ns | 10.62 us | 529.10 ns | 1.71 us | 12.14x | 0.01x | 0.27x | 0.08x |
| `sin 1000pi_eps` | 11.47 ns | 88.35 ns | 790.47 ns | 15.63 us | 2.26 us | 2.92 us | 7.71x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.39 ns | 87.35 ns | 780.03 ns | 13.69 us | 571.00 ns | 1.71 us | 7.05x | 0.01x | 0.15x | 0.05x |
| `asin 0.5` | 10.82 ns | 469.52 ns | 479.75 ns | 48.07 us | 2.96 us | 13.41 us | 43.41x | 0.01x | 0.16x | 0.04x |
| `acos 0.5` | 11.11 ns | 1.01 us | 1.03 us | 58.13 us | 2.92 us | 13.25 us | 90.76x | 0.02x | 0.35x | 0.08x |
| `atanh 0.5` | 14.27 ns | 1.38 us | 1.41 us | 34.14 us | 1.64 us | 12.91 us | 96.93x | 0.04x | 0.85x | 0.11x |
| `asin neg_0.999999` | 13.80 ns | 735.42 ns | 1.20 us | 13.65 us | 2.51 us | 12.98 us | 53.29x | 0.05x | 0.29x | 0.06x |
| `acos neg_0.999999` | 15.23 ns | 852.77 ns | 1.73 us | 17.86 us | 2.64 us | 12.99 us | 55.98x | 0.05x | 0.32x | 0.07x |
| `atanh neg_0.999999` | 14.36 ns | 364.64 ns | 566.28 ns | 36.07 us | 1.59 us | 12.84 us | 25.40x | 0.01x | 0.23x | 0.03x |
| `asin 0.999999` | 14.17 ns | 736.59 ns | 1.40 us | 13.44 us | 2.49 us | 12.85 us | 51.99x | 0.05x | 0.30x | 0.06x |
| `acos 0.999999` | 14.11 ns | 622.70 ns | 1.53 us | 17.89 us | 2.74 us | 12.98 us | 44.13x | 0.03x | 0.23x | 0.05x |
| `atanh 0.999999` | 14.35 ns | 443.56 ns | 868.16 ns | 31.04 us | 1.61 us | 12.71 us | 30.92x | 0.01x | 0.28x | 0.03x |
| `asin 1e-12` | 9.36 ns | 460.80 ns | 1.12 us | 7.79 us | 1.44 us | 15.27 us | 49.21x | 0.06x | 0.32x | 0.03x |
| `acos 1e-12` | 9.76 ns | 710.26 ns | 1.55 us | 9.53 us | 1.42 us | 15.34 us | 72.78x | 0.07x | 0.50x | 0.05x |
| `atanh 1e-12` | 9.63 ns | 434.47 ns | 827.37 ns | 35.27 us | 169.43 ns | 20.21 us | 45.11x | 0.01x | 2.56x | 0.02x |
| `atan 0.5` | 14.79 ns | 296.96 ns | 297.47 ns | 34.77 us | 2.67 us | 17.65 us | 20.09x | 0.01x | 0.11x | 0.02x |
| `asinh 0.5` | 26.44 ns | 421.56 ns | 421.78 ns | 38.33 us | 1.59 us | 7.50 us | 15.94x | 0.01x | 0.26x | 0.06x |
| `atan neg_1e-12` | 14.09 ns | 325.31 ns | 566.51 ns | 1.59 us | 1.11 us | 15.28 us | 23.09x | 0.20x | 0.29x | 0.02x |
| `asinh neg_1e-12` | 15.70 ns | 474.49 ns | 382.33 ns | 40.66 us | 8.52 us | 11.70 us | 30.23x | 0.01x | 0.06x | 0.04x |
| `atan 1e6` | 15.48 ns | 187.28 ns | 186.04 ns | 2.84 us | 1.39 us | 17.76 us | 12.10x | 0.07x | 0.13x | 0.01x |
| `asinh 1e6` | 26.38 ns | 295.99 ns | 293.37 ns | 35.96 us | 1.65 us | 7.21 us | 11.22x | 0.01x | 0.18x | 0.04x |
| `atan neg_1e6` | 15.07 ns | 273.31 ns | 272.34 ns | 2.77 us | 1.42 us | 17.81 us | 18.13x | 0.10x | 0.19x | 0.02x |
| `asinh neg_1e6` | 26.54 ns | 339.43 ns | 333.78 ns | 36.17 us | 1.64 us | 7.04 us | 12.79x | 0.01x | 0.21x | 0.05x |
| `acosh 9` | 13.14 ns | 172.29 ns | 171.62 ns | 41.80 us | 1.62 us | 9.70 us | 13.11x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 12.12 ns | 518.26 ns | 1.40 us | 40.81 us | 8.27 us | 11.62 us | 42.77x | 0.01x | 0.06x | 0.04x |
| `acosh 1e6` | 12.90 ns | 172.16 ns | 171.69 ns | 36.53 us | 1.62 us | 9.75 us | 13.34x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.94 ns | 512.81 ns | 2.30 us | 40.73 us | 1.64 us | 9.67 us | 39.62x | 0.01x | 0.31x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 18.67 ns | 19.24 ns | 26.76 ns | 15.53 ns | 0.94 ns | 38.86x | 0.70x | 1.20x | 19.84x |
| `one` | 0.48 ns | 23.83 ns | 25.03 ns | 40.01 ns | 30.60 ns | 30.02 ns | 49.87x | 0.60x | 0.78x | 0.79x |
| `e` | 0.48 ns | 96.71 ns | 91.89 ns | 57.30 ns | 1.06 us | 224.95 ns | 202.59x | 1.69x | 0.09x | 0.43x |
| `pi` | 0.47 ns | 57.32 ns | 49.68 ns | 45.85 ns | 48.24 ns | 226.25 ns | 122.28x | 1.25x | 1.19x | 0.25x |
| `tau` | 0.47 ns | 57.47 ns | 49.58 ns | 117.25 ns | 98.23 ns | 1.88 us | 122.77x | 0.49x | 0.59x | 0.03x |
| `add` | 5.14 ns | 216.30 ns | 325.31 ns | 52.25 ns | 42.74 ns | 1.32 us | 42.11x | 4.14x | 5.06x | 0.16x |
| `sub` | 5.15 ns | 220.86 ns | 383.02 ns | 55.56 ns | 44.86 ns | 2.48 us | 42.91x | 3.98x | 4.92x | 0.09x |
| `neg` | 5.04 ns | 48.75 ns | 38.81 ns | 23.02 ns | 21.55 ns | 1.08 us | 9.68x | 2.12x | 2.26x | 0.05x |
| `mul` | 5.41 ns | 136.54 ns | 395.60 ns | 55.59 ns | 46.59 ns | 1.55 us | 25.25x | 2.46x | 2.93x | 0.09x |
| `div` | 8.87 ns | 316.68 ns | 361.01 ns | 134.40 ns | 66.23 ns | 2.59 us | 35.70x | 2.36x | 4.78x | 0.12x |
| `reciprocal` | 8.91 ns | 81.53 ns | 82.02 ns | 159.04 ns | 59.63 ns | 1.53 us | 9.15x | 0.51x | 1.37x | 0.05x |
| `reciprocal checked` | 9.04 ns | 95.07 ns | 83.28 ns | 157.17 ns | 59.50 ns | 1.53 us | 10.51x | 0.60x | 1.60x | 0.06x |
| `reciprocal checked abort` | 24.07 ns | 100.71 ns | 88.14 ns | 158.39 ns | 59.55 ns | 1.54 us | 4.18x | 0.64x | 1.69x | 0.07x |
| `pow` | 30.48 ns | 8.21 us | 7.31 us | 53.95 us | 2.85 us | 2.34 us | 269.34x | 0.15x | 2.88x | 3.51x |
| `powi` | 5.73 ns | 388.65 ns | 2.59 us | 296.82 ns | 82.90 ns | 1.59 us | 67.80x | 1.31x | 4.69x | 0.24x |
| `exp` | 19.58 ns | 1.51 us | 2.14 us | 13.82 us | 921.99 ns | 1.92 us | 77.02x | 0.11x | 1.64x | 0.79x |
| `ln` | 10.86 ns | 1.55 us | 2.37 us | 29.49 us | 1.34 us | 1.83 us | 142.98x | 0.05x | 1.16x | 0.85x |
| `log10` | 13.35 ns | 4.56 us | 5.67 us | 34.62 us | 2.82 us | 6.74 us | 341.44x | 0.13x | 1.62x | 0.68x |
| `log10 abort` | 16.69 ns | 4.60 us | 5.72 us | 34.60 us | 2.80 us | 6.91 us | 275.62x | 0.13x | 1.64x | 0.67x |
| `sqrt` | 8.41 ns | 1.59 us | 1.58 us | 5.05 us | 94.91 ns | 1.49 us | 188.60x | 0.31x | 16.71x | 1.06x |
| `sin` | 14.81 ns | 133.05 ns | 118.40 ns | 13.75 us | 1.33 us | 2.27 us | 8.98x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.62 ns | 134.00 ns | 118.46 ns | 12.33 us | 635.96 ns | 1.77 us | 7.20x | 0.01x | 0.21x | 0.08x |
| `tan` | 24.69 ns | 170.76 ns | 173.23 ns | 29.96 us | 1.71 us | 6.66 us | 6.91x | 0.01x | 0.10x | 0.03x |
| `sinh` | 18.01 ns | 4.06 us | 4.11 us | 3.34 us | 1.13 us | 10.82 us | 225.60x | 1.22x | 3.58x | 0.38x |
| `cosh` | 18.08 ns | 4.03 us | 4.00 us | 7.93 us | 1.07 us | 9.75 us | 223.03x | 0.51x | 3.76x | 0.41x |
| `tanh` | 22.46 ns | 8.21 us | 8.26 us | 3.35 us | 1.20 us | 23.55 us | 365.83x | 2.45x | 6.85x | 0.35x |
| `asin` | 13.29 ns | 623.36 ns | 1.48 us | 20.98 us | 2.44 us | 14.10 us | 46.90x | 0.03x | 0.26x | 0.04x |
| `asin abort` | 16.95 ns | 620.87 ns | 1.52 us | 21.09 us | 2.44 us | 14.24 us | 36.63x | 0.03x | 0.25x | 0.04x |
| `acos` | 14.82 ns | 832.19 ns | 2.11 us | 26.43 us | 2.54 us | 14.16 us | 56.17x | 0.03x | 0.33x | 0.06x |
| `acos abort` | 17.63 ns | 839.56 ns | 2.13 us | 26.72 us | 2.53 us | 14.05 us | 47.61x | 0.03x | 0.33x | 0.06x |
| `atan` | 15.00 ns | 276.21 ns | 951.21 ns | 18.78 us | 2.23 us | 19.20 us | 18.41x | 0.01x | 0.12x | 0.01x |
| `atan abort` | 21.27 ns | 277.65 ns | 957.18 ns | 18.81 us | 2.24 us | 19.01 us | 13.06x | 0.01x | 0.12x | 0.01x |
| `asinh` | 30.99 ns | 409.20 ns | 1.10 us | 38.73 us | 1.67 us | 7.67 us | 13.20x | 0.01x | 0.25x | 0.05x |
| `asinh abort` | 31.46 ns | 411.14 ns | 1.10 us | 38.51 us | 1.67 us | 7.67 us | 13.07x | 0.01x | 0.25x | 0.05x |
| `acosh` | 12.65 ns | 345.65 ns | 1.11 us | 41.24 us | 3.37 us | 10.53 us | 27.33x | 0.01x | 0.10x | 0.03x |
| `acosh abort` | 15.85 ns | 345.69 ns | 1.11 us | 41.03 us | 3.34 us | 10.69 us | 21.81x | 0.01x | 0.10x | 0.03x |
| `atanh` | 13.58 ns | 694.58 ns | 1.37 us | 34.72 us | 1.28 us | 15.03 us | 51.14x | 0.02x | 0.54x | 0.05x |
| `atanh abort` | 17.46 ns | 686.20 ns | 1.34 us | 34.28 us | 1.28 us | 15.25 us | 39.31x | 0.02x | 0.54x | 0.04x |
| `zero status` | 1.19 ns | 1.66 ns | 1.73 ns | 0.99 ns | 6.87 ns | 8.05 ns | 1.39x | 1.67x | 0.24x | 0.21x |
| `zero status abort` | 1.40 ns | 3.36 ns | 3.45 ns | 1.00 ns | 6.78 ns | 8.06 ns | 2.41x | 3.37x | 0.50x | 0.42x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 36.01 ns | 36.12 ns | 48.62 ns | - | 1.96 ns | 38.31x | 0.74x | - | 18.36x |
| `one` | 5.35 ns | 40.01 ns | 39.86 ns | 64.17 ns | - | 30.70 ns | 7.48x | 0.62x | - | 1.30x |
| `i` | 0.95 ns | 44.43 ns | 45.89 ns | 64.49 ns | - | 30.33 ns | 46.91x | 0.69x | - | 1.46x |
| `free i` | 0.94 ns | 45.46 ns | 44.97 ns | 64.25 ns | - | 29.68 ns | 48.30x | 0.71x | - | 1.53x |
| `conjugate` | 2.12 ns | 87.16 ns | 87.25 ns | 41.34 ns | - | 1.09 us | 41.02x | 2.11x | - | 0.08x |
| `norm squared` | 5.75 ns | 342.10 ns | 1.41 us | 151.99 ns | - | 4.39 us | 59.46x | 2.25x | - | 0.08x |
| `reciprocal` | 16.80 ns | 1.74 us | 2.80 us | 443.61 ns | - | 10.88 us | 103.29x | 3.91x | - | 0.16x |
| `reciprocal checked` | 15.46 ns | 1.75 us | 2.82 us | 447.34 ns | - | 11.14 us | 113.03x | 3.91x | - | 0.16x |
| `powi` | 17.95 ns | 2.07 us | 13.29 us | 1.44 us | - | 44.25 us | 115.36x | 1.43x | - | 0.05x |
| `powi checked` | 17.52 ns | 2.08 us | 13.28 us | 1.43 us | - | 44.60 us | 118.67x | 1.45x | - | 0.05x |
| `div checked` | 18.44 ns | 2.70 us | 6.14 us | 784.64 ns | - | 21.94 us | 146.42x | 3.44x | - | 0.12x |
| `div real checked` | 9.41 ns | 677.25 ns | 643.10 ns | 266.86 ns | - | 5.25 us | 71.97x | 2.54x | - | 0.13x |
| `from scalar` | 1.41 ns | 56.81 ns | 56.40 ns | 45.37 ns | - | 10.46 ns | 40.16x | 1.25x | - | 5.43x |
| `add` | 6.14 ns | 477.16 ns | 837.41 ns | 106.09 ns | - | 2.63 us | 77.65x | 4.50x | - | 0.18x |
| `sub` | 6.11 ns | 478.41 ns | 855.15 ns | 116.46 ns | - | 4.93 us | 78.28x | 4.11x | - | 0.10x |
| `neg` | 2.59 ns | 86.76 ns | 85.67 ns | 42.28 ns | - | 2.16 us | 33.51x | 2.05x | - | 0.04x |
| `mul` | 7.75 ns | 856.91 ns | 3.14 us | 306.14 ns | - | 10.31 us | 110.50x | 2.80x | - | 0.08x |
| `div` | 18.00 ns | 2.68 us | 6.13 us | 794.60 ns | - | 21.87 us | 148.76x | 3.37x | - | 0.12x |
| `div real` | 10.15 ns | 668.39 ns | 622.54 ns | 266.99 ns | - | 5.25 us | 65.86x | 2.50x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.71 ns | 209.62 ns | 2.39 us | 299.85 ns | 251.19 ns | 7.21 us | 31.22x | 0.70x | 0.83x | 0.03x |
| `vec3 magnitude` | 9.42 ns | 4.04 us | 5.71 us | 5.42 us | 334.43 ns | 8.83 us | 429.22x | 0.75x | 12.09x | 0.46x |
| `vec3 normalize` | 24.70 ns | 8.31 us | 11.44 us | 5.86 us | 577.71 ns | 16.96 us | 336.43x | 1.42x | 14.38x | 0.49x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.06 ns | 219.71 ns | 1.45 us | 69.08 ns | 56.69 ns | 720.09 ns | 71.78x | 3.18x | 3.88x | 0.31x |
| `vec3 zero` | 1.41 ns | 148.15 ns | 147.29 ns | 64.67 ns | 30.98 ns | 2.82 ns | 105.21x | 2.29x | 4.78x | 52.51x |
| `vec3 dot abort` | 7.44 ns | 212.96 ns | 880.65 ns | 256.57 ns | 200.20 ns | 7.29 us | 28.63x | 0.83x | 1.06x | 0.03x |
| `vec3 magnitude abort` | 15.51 ns | 4.12 us | 3.54 us | 5.53 us | 321.50 ns | 8.93 us | 265.69x | 0.75x | 12.81x | 0.46x |
| `vec3 normalize checked` | 25.81 ns | 8.33 us | 7.49 us | 5.94 us | 536.23 ns | 17.25 us | 322.65x | 1.40x | 15.53x | 0.48x |
| `vec3 normalize checked abort` | 28.89 ns | 8.32 us | 7.51 us | 5.95 us | 536.04 ns | 17.10 us | 288.03x | 1.40x | 15.52x | 0.49x |
| `vec3 div scalar checked` | 10.36 ns | 1.36 us | 1.56 us | 407.91 ns | - | - | 130.94x | 3.33x | - | - |
| `vec3 div scalar checked abort` | 18.56 ns | 1.35 us | 1.57 us | 407.54 ns | - | - | 72.61x | 3.31x | - | - |
| `vec3 add` | 6.76 ns | 945.20 ns | 1.70 us | 150.53 ns | 125.89 ns | 4.01 us | 139.84x | 6.28x | 7.51x | 0.24x |
| `vec3 add scalar` | 6.45 ns | 776.78 ns | 1.44 us | 152.28 ns | 133.41 ns | 3.86 us | 120.38x | 5.10x | 5.82x | 0.20x |
| `vec3 sub` | 6.75 ns | 1.01 us | 1.81 us | 164.06 ns | 137.20 ns | 7.46 us | 149.47x | 6.15x | 7.35x | 0.14x |
| `vec3 sub scalar` | 6.43 ns | 767.96 ns | 1.24 us | 147.79 ns | 124.66 ns | 7.21 us | 119.49x | 5.20x | 6.16x | 0.11x |
| `vec3 neg` | 3.76 ns | 154.57 ns | 154.85 ns | 58.12 ns | 50.05 ns | 3.24 us | 41.09x | 2.66x | 3.09x | 0.05x |
| `vec3 mul scalar` | 6.97 ns | 531.00 ns | 1.99 us | 166.93 ns | 124.52 ns | 4.42 us | 76.15x | 3.18x | 4.26x | 0.12x |
| `vec3 div scalar` | 10.08 ns | 1.32 us | 1.56 us | 396.44 ns | 178.17 ns | 7.70 us | 130.71x | 3.32x | 7.40x | 0.17x |
| `vec4 dot` | 7.28 ns | 246.05 ns | 623.04 ns | 442.80 ns | 317.64 ns | 9.85 us | 33.82x | 0.56x | 0.77x | 0.02x |
| `vec4 magnitude` | 12.70 ns | 3.82 us | 2.82 us | 5.76 us | 400.62 ns | 11.49 us | 300.74x | 0.66x | 9.53x | 0.33x |
| `vec4 normalize` | 36.32 ns | 8.53 us | 6.59 us | 6.20 us | 679.65 ns | 22.32 us | 235.00x | 1.38x | 12.56x | 0.38x |
| `vec4 add` | 7.36 ns | 1.19 us | 1.83 us | 204.47 ns | 172.63 ns | 5.34 us | 162.42x | 5.84x | 6.92x | 0.22x |
| `vec4 add scalar` | 6.96 ns | 1.01 us | 1.69 us | 214.64 ns | 176.23 ns | 5.18 us | 145.70x | 4.72x | 5.75x | 0.20x |
| `vec4 sub` | 5.14 ns | 1.18 us | 1.71 us | 213.30 ns | 174.38 ns | 9.82 us | 230.48x | 5.55x | 6.79x | 0.12x |
| `vec4 sub scalar` | 4.60 ns | 1.01 us | 1.47 us | 205.06 ns | 169.19 ns | 9.53 us | 218.97x | 4.91x | 5.95x | 0.11x |
| `vec4 neg` | 4.94 ns | 205.82 ns | 204.32 ns | 80.46 ns | 62.39 ns | 4.09 us | 41.70x | 2.56x | 3.30x | 0.05x |
| `vec4 mul scalar` | 7.51 ns | 670.77 ns | 2.06 us | 216.79 ns | 159.96 ns | 5.70 us | 89.26x | 3.09x | 4.19x | 0.12x |
| `vec4 div scalar` | 14.27 ns | 1.70 us | 1.50 us | 542.14 ns | 223.77 ns | 10.09 us | 119.39x | 3.14x | 7.61x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.50 ns | 748.94 ns | 2.25 us | 990.46 ns | 844.63 ns | 22.38 us | 59.91x | 0.76x | 0.89x | 0.03x |
| `mat3 inverse` | 77.85 ns | 14.96 us | 9.04 us | 3.13 us | 2.47 us | 82.30 us | 192.19x | 4.78x | 6.05x | 0.18x |
| `mat3 mul mat3` | 35.08 ns | 3.12 us | 7.11 us | 2.72 us | 2.32 us | 61.47 us | 88.95x | 1.15x | 1.35x | 0.05x |
| `mat3 transform vec3` | 15.94 ns | 1.38 us | 4.51 us | 1.00 us | 870.33 ns | 20.30 us | 86.74x | 1.38x | 1.59x | 0.07x |
| `mat4 determinant` | 35.77 ns | 2.32 us | 1.97 us | 4.59 us | 4.09 us | 96.24 us | 64.97x | 0.51x | 0.57x | 0.02x |
| `mat4 inverse` | 144.94 ns | 24.61 us | 8.89 us | 11.15 us | 9.13 us | 350.09 us | 169.82x | 2.21x | 2.70x | 0.07x |
| `mat4 mul mat4` | 78.46 ns | 5.52 us | 4.21 us | 6.42 us | 5.44 us | 143.77 us | 70.42x | 0.86x | 1.02x | 0.04x |
| `mat4 transform vec4` | 26.67 ns | 2.18 us | 1.77 us | 1.91 us | 1.64 us | 36.24 us | 81.89x | 1.15x | 1.33x | 0.06x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 36.17 ns | 1.11 us | 4.27 us | 214.98 ns | 227.58 ns | 2.08 us | 30.68x | 5.16x | 4.88x | 0.53x |
| `mat3 zero` | 16.26 ns | 530.99 ns | 530.20 ns | 271.12 ns | 203.47 ns | 11.93 ns | 32.66x | 1.96x | 2.61x | 44.50x |
| `mat3 identity` | 9.80 ns | 680.33 ns | 682.47 ns | 328.47 ns | 235.71 ns | 140.54 ns | 69.44x | 2.07x | 2.89x | 4.84x |
| `mat3 transpose` | 9.10 ns | 718.08 ns | 711.87 ns | 239.04 ns | 202.92 ns | 117.03 ns | 78.87x | 3.00x | 3.54x | 6.14x |
| `mat3 reciprocal` | 79.62 ns | 15.20 us | 24.98 us | 2.91 us | 2.29 us | 83.86 us | 190.93x | 5.22x | 6.63x | 0.18x |
| `mat3 reciprocal checked` | 79.39 ns | 15.09 us | 24.92 us | 2.93 us | 2.30 us | 84.10 us | 190.13x | 5.15x | 6.55x | 0.18x |
| `mat3 inverse checked` | 79.44 ns | 15.51 us | 24.92 us | 2.94 us | 2.28 us | 83.63 us | 195.29x | 5.28x | 6.80x | 0.19x |
| `mat3 inverse checked abort` | 115.79 ns | 15.09 us | 24.89 us | 2.91 us | 2.29 us | 83.11 us | 130.36x | 5.19x | 6.59x | 0.18x |
| `mat3 powi` | 90.21 ns | 5.77 us | 39.09 us | 6.88 us | 6.10 us | 151.20 us | 63.94x | 0.84x | 0.95x | 0.04x |
| `mat3 powi checked` | 92.43 ns | 5.76 us | 38.99 us | 6.74 us | 6.09 us | 151.57 us | 62.36x | 0.86x | 0.95x | 0.04x |
| `mat3 powi checked abort` | 92.04 ns | 5.86 us | 39.24 us | 6.84 us | 6.10 us | 152.34 us | 63.65x | 0.86x | 0.96x | 0.04x |
| `mat3 div scalar checked` | 24.53 ns | 4.18 us | 5.53 us | 1.46 us | 803.82 ns | 22.34 us | 170.24x | 2.86x | 5.20x | 0.19x |
| `mat3 div scalar checked abort` | 29.62 ns | 4.25 us | 5.57 us | 1.49 us | 807.61 ns | 22.22 us | 143.60x | 2.86x | 5.27x | 0.19x |
| `mat3 div matrix checked` | 118.03 ns | 18.60 us | 58.47 us | 5.41 us | 4.48 us | 162.22 us | 157.54x | 3.43x | 4.15x | 0.11x |
| `mat3 div matrix checked abort` | 131.10 ns | 18.57 us | 58.30 us | 5.43 us | 4.46 us | 161.66 us | 141.65x | 3.42x | 4.16x | 0.11x |
| `mat3 add` | 14.70 ns | 3.21 us | 5.23 us | 517.29 ns | 486.87 ns | 11.99 us | 218.23x | 6.20x | 6.59x | 0.27x |
| `mat3 add scalar` | 12.10 ns | 2.49 us | 4.66 us | 767.70 ns | 702.70 ns | 12.20 us | 206.21x | 3.25x | 3.55x | 0.20x |
| `mat3 sub` | 13.16 ns | 3.19 us | 5.17 us | 535.34 ns | 512.46 ns | 21.62 us | 242.16x | 5.95x | 6.22x | 0.15x |
| `mat3 sub scalar` | 10.80 ns | 2.49 us | 4.50 us | 776.66 ns | 703.77 ns | 21.77 us | 230.87x | 3.21x | 3.54x | 0.11x |
| `mat3 neg` | 10.40 ns | 597.58 ns | 594.88 ns | 471.33 ns | 451.96 ns | 8.56 us | 57.48x | 1.27x | 1.32x | 0.07x |
| `mat3 mul scalar` | 13.96 ns | 1.80 us | 5.68 us | 762.72 ns | 672.60 ns | 12.25 us | 129.17x | 2.36x | 2.68x | 0.15x |
| `mat3 div scalar` | 24.43 ns | 4.26 us | 5.58 us | 1.45 us | 803.74 ns | 22.29 us | 174.50x | 2.94x | 5.30x | 0.19x |
| `mat3 div matrix` | 128.43 ns | 18.73 us | 58.81 us | 5.45 us | 4.47 us | 162.18 us | 145.86x | 3.44x | 4.19x | 0.12x |
| `mat3 bitxor` | 90.61 ns | 5.84 us | 39.02 us | 6.70 us | 6.10 us | 151.42 us | 64.48x | 0.87x | 0.96x | 0.04x |
| `mat4 zero` | 11.53 ns | 925.01 ns | 929.38 ns | 477.15 ns | 357.37 ns | 14.60 ns | 80.23x | 1.94x | 2.59x | 63.37x |
| `mat4 identity` | 10.90 ns | 1.12 us | 1.13 us | 569.47 ns | 416.04 ns | 216.87 ns | 102.96x | 1.97x | 2.70x | 5.17x |
| `mat4 transpose` | 9.30 ns | 1.13 us | 1.12 us | 453.30 ns | 370.42 ns | 176.29 ns | 121.70x | 2.50x | 3.05x | 6.42x |
| `mat4 reciprocal` | 144.15 ns | 24.88 us | 42.50 us | 10.79 us | 8.84 us | 347.31 us | 172.60x | 2.31x | 2.82x | 0.07x |
| `mat4 reciprocal checked` | 143.44 ns | 24.80 us | 42.72 us | 10.76 us | 8.84 us | 344.63 us | 172.89x | 2.31x | 2.80x | 0.07x |
| `mat4 powi` | 166.00 ns | 10.54 us | 53.13 us | 15.56 us | 14.15 us | 352.16 us | 63.50x | 0.68x | 0.74x | 0.03x |
| `mat4 powi checked` | 167.34 ns | 10.49 us | 52.99 us | 15.48 us | 14.18 us | 350.02 us | 62.70x | 0.68x | 0.74x | 0.03x |
| `mat4 add` | 51.88 ns | 5.23 us | 6.05 us | 952.73 ns | 843.12 ns | 20.11 us | 100.82x | 5.49x | 6.20x | 0.26x |
| `mat4 add scalar` | 20.57 ns | 4.20 us | 6.79 us | 1.40 us | 1.17 us | 20.90 us | 204.31x | 3.01x | 3.58x | 0.20x |
| `mat4 sub` | 39.99 ns | 5.18 us | 6.08 us | 976.04 ns | 899.63 ns | 36.42 us | 129.54x | 5.31x | 5.76x | 0.14x |
| `mat4 sub scalar` | 15.07 ns | 4.16 us | 6.60 us | 1.42 us | 1.17 us | 37.88 us | 276.26x | 2.94x | 3.57x | 0.11x |
| `mat4 neg` | 14.47 ns | 1.05 us | 1.05 us | 886.03 ns | 735.84 ns | 14.18 us | 72.57x | 1.19x | 1.43x | 0.07x |
| `mat4 mul scalar` | 24.66 ns | 2.93 us | 7.31 us | 1.39 us | 1.11 us | 20.69 us | 118.97x | 2.11x | 2.65x | 0.14x |
| `mat4 div scalar` | 33.32 ns | 7.59 us | 7.43 us | 2.62 us | 1.35 us | 38.42 us | 227.91x | 2.90x | 5.61x | 0.20x |
| `mat4 div matrix` | 189.26 ns | 29.03 us | 86.47 us | 16.90 us | 14.34 us | 531.97 us | 153.38x | 1.72x | 2.02x | 0.05x |
| `mat4 bitxor` | 166.25 ns | 10.61 us | 53.14 us | 15.50 us | 14.23 us | 351.66 us | 63.83x | 0.68x | 0.75x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.61 ns | 180.24 ns | 285.84 ns | - | - | 1.33 us | 49.90x | - | - | 0.14x |
| `scalar add ref_owned` | 12.28 ns | 177.10 ns | 286.67 ns | - | - | 1.39 us | 14.42x | - | - | 0.13x |
| `scalar add refs` | 5.34 ns | 164.39 ns | 273.38 ns | - | - | 1.34 us | 30.81x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 9.09 ns | 204.48 ns | 298.55 ns | - | - | - | 22.49x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.50 ns | 193.93 ns | 299.89 ns | - | - | - | 35.29x | - | - | - |
| `scalar sub owned_ref` | 3.86 ns | 184.62 ns | 341.64 ns | - | - | 2.49 us | 47.82x | - | - | 0.07x |
| `scalar sub ref_owned` | 12.32 ns | 186.66 ns | 340.50 ns | - | - | 2.49 us | 15.15x | - | - | 0.08x |
| `scalar sub refs` | 5.64 ns | 166.44 ns | 324.32 ns | - | - | 2.53 us | 29.50x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 8.96 ns | 205.37 ns | 354.08 ns | - | - | - | 22.91x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.72 ns | 203.73 ns | 360.91 ns | - | - | - | 35.60x | - | - | - |
| `scalar mul owned_ref` | 4.62 ns | 98.49 ns | 372.24 ns | - | - | 1.55 us | 21.32x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.39 ns | 98.94 ns | 365.17 ns | - | - | 1.56 us | 7.39x | - | - | 0.06x |
| `scalar mul refs` | 6.35 ns | 80.73 ns | 355.44 ns | - | - | 1.56 us | 12.72x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.81 ns | 114.84 ns | 378.17 ns | - | - | - | 11.71x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.26 ns | 107.24 ns | 377.40 ns | - | - | - | 17.14x | - | - | - |
| `scalar div owned_ref` | 5.88 ns | 267.71 ns | 311.51 ns | - | - | 2.60 us | 45.51x | - | - | 0.10x |
| `scalar div ref_owned` | 17.25 ns | 261.17 ns | 316.12 ns | - | - | 2.62 us | 15.14x | - | - | 0.10x |
| `scalar div refs` | 6.89 ns | 247.40 ns | 299.32 ns | - | - | 4.42 us | 35.89x | - | - | 0.06x |
| `scalar div owned_ref_with_clone` | 13.37 ns | 279.29 ns | 326.94 ns | - | - | - | 20.88x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.59 ns | 291.69 ns | 348.29 ns | - | - | - | 18.71x | - | - | - |
| `vec3 add refs` | 6.05 ns | 642.27 ns | 1.39 us | - | - | 4.28 us | 106.19x | - | - | 0.15x |
| `vec3 sub refs` | 6.10 ns | 654.02 ns | 1.44 us | - | - | 7.44 us | 107.14x | - | - | 0.09x |
| `vec3 neg ref` | 3.29 ns | 174.16 ns | 177.56 ns | - | - | 3.15 us | 52.92x | - | - | 0.06x |
| `vec3 add_scalar_ref` | 6.55 ns | 808.01 ns | 1.49 us | - | - | 4.02 us | 123.37x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.64 ns | 789.64 ns | 1.26 us | - | - | 7.45 us | 118.87x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 6.91 ns | 537.03 ns | 2.02 us | - | - | 4.44 us | 77.75x | - | - | 0.12x |
| `vec3 div_scalar_ref` | 8.03 ns | 1.29 us | 1.53 us | - | - | 7.83 us | 160.39x | - | - | 0.16x |
| `vec4 add refs` | 6.68 ns | 765.91 ns | 1.38 us | - | - | 5.40 us | 114.63x | - | - | 0.14x |
| `vec4 sub refs` | 3.09 ns | 768.50 ns | 1.27 us | - | - | 16.99 us | 248.72x | - | - | 0.05x |
| `vec4 neg ref` | 4.23 ns | 221.40 ns | 223.28 ns | - | - | 4.08 us | 52.37x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 6.95 ns | 1.02 us | 1.71 us | - | - | 5.23 us | 146.12x | - | - | 0.19x |
| `vec4 sub_scalar_ref` | 4.22 ns | 1.02 us | 1.59 us | - | - | 9.50 us | 240.75x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.50 ns | 663.59 ns | 2.20 us | - | - | 5.93 us | 88.49x | - | - | 0.11x |
| `vec4 div_scalar_ref` | 12.13 ns | 1.72 us | 1.68 us | - | - | 10.45 us | 141.77x | - | - | 0.16x |
| `mat3 add refs` | 11.77 ns | 1.71 us | 3.90 us | - | - | 12.05 us | 145.19x | - | - | 0.14x |
| `mat3 sub refs` | 11.00 ns | 1.71 us | 3.82 us | - | - | 21.63 us | 155.52x | - | - | 0.08x |
| `mat3 mul refs` | 31.85 ns | 1.97 us | 12.79 us | - | - | 62.58 us | 61.69x | - | - | 0.03x |
| `mat3 div refs` | 117.50 ns | 17.60 us | 57.84 us | - | - | 166.63 us | 149.81x | - | - | 0.11x |
| `mat3 neg ref` | 9.58 ns | 485.71 ns | 495.61 ns | - | - | 8.97 us | 50.72x | - | - | 0.05x |
| `mat3 add_scalar_ref` | 44.18 ns | 2.48 us | 5.07 us | - | - | 12.49 us | 56.18x | - | - | 0.20x |
| `mat3 sub_scalar_ref` | 44.55 ns | 2.41 us | 4.72 us | - | - | 21.72 us | 54.15x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 46.88 ns | 1.83 us | 5.82 us | - | - | 12.25 us | 38.94x | - | - | 0.15x |
| `mat3 div_scalar_ref` | 21.84 ns | 4.23 us | 5.64 us | - | - | 22.44 us | 193.81x | - | - | 0.19x |
| `mat4 add refs` | 17.69 ns | 2.64 us | 3.46 us | - | - | 20.14 us | 149.52x | - | - | 0.13x |
| `mat4 sub refs` | 16.78 ns | 2.65 us | 3.72 us | - | - | 36.31 us | 158.12x | - | - | 0.07x |
| `mat4 mul refs` | 71.06 ns | 3.77 us | 17.68 us | - | - | 144.13 us | 53.11x | - | - | 0.03x |
| `mat4 div refs` | 178.30 ns | 28.21 us | 87.84 us | - | - | 532.59 us | 158.24x | - | - | 0.05x |
| `mat4 neg ref` | 12.74 ns | 878.07 ns | 857.02 ns | - | - | 14.17 us | 68.95x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 52.26 ns | 6.79 us | 6.76 us | - | - | 21.25 us | 129.86x | - | - | 0.32x |
| `mat4 sub_scalar_ref` | 39.79 ns | 4.24 us | 6.56 us | - | - | 37.94 us | 106.67x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 56.17 ns | 3.06 us | 7.38 us | - | - | 20.39 us | 54.48x | - | - | 0.15x |
| `mat4 div_scalar_ref` | 27.81 ns | 7.45 us | 7.43 us | - | - | 38.15 us | 267.97x | - | - | 0.20x |
| `mat3 transform_vec refs` | 14.43 ns | 782.46 ns | 4.03 us | - | - | 20.24 us | 54.24x | - | - | 0.04x |
| `mat4 transform_vec refs` | 25.08 ns | 1.12 us | 4.83 us | - | - | 35.78 us | 44.66x | - | - | 0.03x |
| `complex add refs` | 7.71 ns | 335.30 ns | 721.16 ns | - | - | 2.63 us | 43.50x | - | - | 0.13x |
| `complex sub refs` | 7.89 ns | 642.34 ns | 725.53 ns | - | - | 4.84 us | 81.41x | - | - | 0.13x |
| `complex mul refs` | 8.05 ns | 853.23 ns | 3.09 us | - | - | 10.15 us | 106.00x | - | - | 0.08x |
| `complex div refs` | 17.27 ns | 2.77 us | 6.12 us | - | - | 22.96 us | 160.26x | - | - | 0.12x |
| `complex neg ref` | 2.36 ns | 75.10 ns | 91.64 ns | - | - | 2.17 us | 31.79x | - | - | 0.03x |
| `complex div_real_ref` | 7.39 ns | 624.69 ns | 585.43 ns | - | - | 5.22 us | 84.51x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.14 us |
| `astro sin 160` | 13.90 us |
| `astro sin 192` | 13.90 us |
| `astro sin 256` | 16.18 us |
