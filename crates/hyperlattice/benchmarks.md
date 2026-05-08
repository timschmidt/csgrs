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
| `sin 0.1` | 11.11 ns | 152.28 ns | 146.87 ns | 11.15 us | 762.91 ns | 1.85 us | 13.70x | 0.01x | 0.20x | 0.08x |
| `cos 0.1` | 11.63 ns | 148.27 ns | 146.55 ns | 10.91 us | 505.49 ns | 1.71 us | 12.75x | 0.01x | 0.29x | 0.09x |
| `sin 1.23456789` | 11.89 ns | 210.53 ns | 191.65 ns | 12.97 us | 850.77 ns | 1.89 us | 17.70x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 12.23 ns | 194.33 ns | 185.32 ns | 11.41 us | 600.64 ns | 1.70 us | 15.88x | 0.02x | 0.32x | 0.11x |
| `sin 1e6` | 12.79 ns | 95.72 ns | 96.36 ns | 16.18 us | 1.15 us | 2.08 us | 7.48x | 0.01x | 0.08x | 0.05x |
| `cos 1e6` | 12.64 ns | 94.87 ns | 97.14 ns | 13.65 us | 850.16 ns | 1.90 us | 7.50x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 68.46 ns | 97.19 ns | 97.42 ns | 18.93 us | 2.86 us | 3.60 us | 1.42x | 0.01x | 0.03x | 0.03x |
| `cos 1e30` | 68.92 ns | 97.33 ns | 95.31 ns | 16.19 us | 970.94 ns | 3.11 us | 1.41x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.92 ns | 149.28 ns | 360.15 ns | 12.24 us | 757.56 ns | 1.92 us | 12.52x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.74 ns | 146.57 ns | 768.71 ns | 11.42 us | 539.65 ns | 1.74 us | 12.48x | 0.01x | 0.27x | 0.08x |
| `sin 1000pi_eps` | 11.90 ns | 93.24 ns | 827.11 ns | 15.92 us | 2.28 us | 2.82 us | 7.83x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.49 ns | 94.53 ns | 846.19 ns | 13.88 us | 579.17 ns | 1.73 us | 7.57x | 0.01x | 0.16x | 0.05x |
| `asin 0.5` | 11.15 ns | 447.31 ns | 457.97 ns | 50.84 us | 2.99 us | 13.82 us | 40.11x | 0.01x | 0.15x | 0.03x |
| `acos 0.5` | 11.62 ns | 1.01 us | 1.05 us | 61.16 us | 3.01 us | 13.33 us | 87.16x | 0.02x | 0.34x | 0.08x |
| `atanh 0.5` | 15.03 ns | 1.46 us | 1.46 us | 36.17 us | 1.67 us | 13.13 us | 97.38x | 0.04x | 0.88x | 0.11x |
| `asin neg_0.999999` | 14.75 ns | 797.32 ns | 1.22 us | 14.45 us | 2.54 us | 13.16 us | 54.05x | 0.06x | 0.31x | 0.06x |
| `acos neg_0.999999` | 16.57 ns | 900.59 ns | 1.81 us | 18.87 us | 2.66 us | 13.25 us | 54.34x | 0.05x | 0.34x | 0.07x |
| `atanh neg_0.999999` | 14.57 ns | 382.84 ns | 596.06 ns | 37.53 us | 1.59 us | 13.24 us | 26.28x | 0.01x | 0.24x | 0.03x |
| `asin 0.999999` | 14.93 ns | 749.67 ns | 1.46 us | 14.18 us | 2.52 us | 12.95 us | 50.20x | 0.05x | 0.30x | 0.06x |
| `acos 0.999999` | 14.65 ns | 630.54 ns | 1.57 us | 18.83 us | 2.73 us | 13.32 us | 43.03x | 0.03x | 0.23x | 0.05x |
| `atanh 0.999999` | 14.54 ns | 463.11 ns | 851.11 ns | 33.19 us | 1.61 us | 13.21 us | 31.85x | 0.01x | 0.29x | 0.04x |
| `asin 1e-12` | 9.45 ns | 461.45 ns | 1.09 us | 8.31 us | 1.46 us | 15.51 us | 48.81x | 0.06x | 0.32x | 0.03x |
| `acos 1e-12` | 10.16 ns | 757.25 ns | 1.55 us | 9.75 us | 1.40 us | 15.25 us | 74.52x | 0.08x | 0.54x | 0.05x |
| `atanh 1e-12` | 9.67 ns | 456.10 ns | 853.02 ns | 37.05 us | 169.22 ns | 20.39 us | 47.16x | 0.01x | 2.70x | 0.02x |
| `atan 0.5` | 15.03 ns | 303.43 ns | 304.11 ns | 35.37 us | 2.76 us | 17.81 us | 20.19x | 0.01x | 0.11x | 0.02x |
| `asinh 0.5` | 26.95 ns | 443.68 ns | 433.22 ns | 40.64 us | 1.64 us | 7.48 us | 16.46x | 0.01x | 0.27x | 0.06x |
| `atan neg_1e-12` | 14.36 ns | 337.36 ns | 569.40 ns | 1.60 us | 1.11 us | 15.41 us | 23.49x | 0.21x | 0.30x | 0.02x |
| `asinh neg_1e-12` | 16.06 ns | 483.35 ns | 398.32 ns | 43.60 us | 8.62 us | 12.07 us | 30.09x | 0.01x | 0.06x | 0.04x |
| `atan 1e6` | 15.07 ns | 184.45 ns | 186.84 ns | 2.92 us | 1.42 us | 18.29 us | 12.24x | 0.06x | 0.13x | 0.01x |
| `asinh 1e6` | 26.68 ns | 308.48 ns | 313.61 ns | 40.51 us | 1.69 us | 7.42 us | 11.56x | 0.01x | 0.18x | 0.04x |
| `atan neg_1e6` | 15.27 ns | 294.82 ns | 299.04 ns | 2.87 us | 1.45 us | 18.56 us | 19.30x | 0.10x | 0.20x | 0.02x |
| `asinh neg_1e6` | 27.61 ns | 351.05 ns | 363.48 ns | 38.53 us | 1.69 us | 7.27 us | 12.72x | 0.01x | 0.21x | 0.05x |
| `acosh 9` | 12.89 ns | 176.97 ns | 175.10 ns | 44.17 us | 1.61 us | 10.08 us | 13.73x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 13.64 ns | 531.57 ns | 1.41 us | 42.07 us | 8.37 us | 11.41 us | 38.96x | 0.01x | 0.06x | 0.05x |
| `acosh 1e6` | 12.80 ns | 176.43 ns | 178.89 ns | 37.36 us | 1.60 us | 9.89 us | 13.78x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.78 ns | 517.91 ns | 2.36 us | 41.69 us | 1.62 us | 9.89 us | 40.51x | 0.01x | 0.32x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 18.72 ns | 18.57 ns | 25.32 ns | 15.78 ns | 0.96 ns | 38.95x | 0.74x | 1.19x | 19.42x |
| `one` | 0.48 ns | 27.44 ns | 23.74 ns | 41.93 ns | 32.77 ns | 32.57 ns | 57.09x | 0.65x | 0.84x | 0.84x |
| `e` | 0.49 ns | 92.47 ns | 89.72 ns | 55.28 ns | 1.07 us | 233.57 ns | 190.24x | 1.67x | 0.09x | 0.40x |
| `pi` | 0.49 ns | 51.17 ns | 50.21 ns | 47.50 ns | 49.48 ns | 238.04 ns | 104.81x | 1.08x | 1.03x | 0.21x |
| `tau` | 0.49 ns | 51.68 ns | 50.73 ns | 118.69 ns | 106.03 ns | 1.89 us | 105.70x | 0.44x | 0.49x | 0.03x |
| `add` | 5.31 ns | 214.06 ns | 320.78 ns | 54.19 ns | 43.86 ns | 1.32 us | 40.28x | 3.95x | 4.88x | 0.16x |
| `sub` | 5.31 ns | 222.35 ns | 384.95 ns | 59.58 ns | 45.50 ns | 2.49 us | 41.87x | 3.73x | 4.89x | 0.09x |
| `neg` | 5.20 ns | 42.44 ns | 40.80 ns | 23.61 ns | 20.29 ns | 1.09 us | 8.16x | 1.80x | 2.09x | 0.04x |
| `mul` | 5.53 ns | 138.43 ns | 413.00 ns | 58.07 ns | 45.08 ns | 1.58 us | 25.01x | 2.38x | 3.07x | 0.09x |
| `div` | 9.09 ns | 313.53 ns | 377.39 ns | 139.84 ns | 62.36 ns | 2.64 us | 34.50x | 2.24x | 5.03x | 0.12x |
| `reciprocal` | 8.70 ns | 80.09 ns | 83.21 ns | 157.79 ns | 58.87 ns | 1.56 us | 9.21x | 0.51x | 1.36x | 0.05x |
| `reciprocal checked` | 9.34 ns | 83.72 ns | 84.55 ns | 158.57 ns | 59.45 ns | 1.57 us | 8.96x | 0.53x | 1.41x | 0.05x |
| `reciprocal checked abort` | 24.29 ns | 93.28 ns | 92.98 ns | 158.07 ns | 58.83 ns | 1.56 us | 3.84x | 0.59x | 1.59x | 0.06x |
| `pow` | 30.89 ns | 10.17 us | 9.60 us | 54.82 us | 2.88 us | 2.38 us | 329.16x | 0.19x | 3.53x | 4.28x |
| `powi` | 6.11 ns | 413.48 ns | 2.61 us | 288.14 ns | 85.90 ns | 1.60 us | 67.71x | 1.44x | 4.81x | 0.26x |
| `exp` | 10.20 ns | 1.51 us | 2.10 us | 13.99 us | 944.42 ns | 1.93 us | 147.59x | 0.11x | 1.59x | 0.78x |
| `ln` | 10.77 ns | 1.15 us | 1.19 us | 30.13 us | 1.35 us | 1.83 us | 107.23x | 0.04x | 0.85x | 0.63x |
| `log10` | 13.62 ns | 3.88 us | 3.39 us | 36.07 us | 2.87 us | 6.72 us | 284.45x | 0.11x | 1.35x | 0.58x |
| `log10 abort` | 17.14 ns | 3.81 us | 3.40 us | 36.94 us | 2.77 us | 6.82 us | 222.42x | 0.10x | 1.38x | 0.56x |
| `sqrt` | 20.44 ns | 1.59 us | 1.55 us | 5.22 us | 96.05 ns | 1.51 us | 77.81x | 0.30x | 16.56x | 1.06x |
| `sin` | 15.33 ns | 125.06 ns | 121.69 ns | 14.25 us | 1.25 us | 2.26 us | 8.16x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.89 ns | 124.40 ns | 121.17 ns | 12.18 us | 635.48 ns | 1.81 us | 6.59x | 0.01x | 0.20x | 0.07x |
| `tan` | 24.79 ns | 179.64 ns | 172.15 ns | 29.43 us | 1.58 us | 6.70 us | 7.25x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.26 ns | 4.10 us | 4.16 us | 3.29 us | 1.15 us | 10.90 us | 224.49x | 1.25x | 3.55x | 0.38x |
| `cosh` | 18.24 ns | 4.06 us | 4.11 us | 8.18 us | 1.06 us | 10.04 us | 222.58x | 0.50x | 3.85x | 0.40x |
| `tanh` | 23.01 ns | 8.29 us | 8.52 us | 3.43 us | 1.22 us | 23.76 us | 360.29x | 2.42x | 6.82x | 0.35x |
| `asin` | 13.54 ns | 651.58 ns | 1.51 us | 22.04 us | 2.42 us | 14.12 us | 48.13x | 0.03x | 0.27x | 0.05x |
| `asin abort` | 17.72 ns | 654.73 ns | 1.50 us | 22.29 us | 2.44 us | 13.87 us | 36.94x | 0.03x | 0.27x | 0.05x |
| `acos` | 14.01 ns | 860.26 ns | 2.15 us | 27.93 us | 2.53 us | 13.93 us | 61.41x | 0.03x | 0.34x | 0.06x |
| `acos abort` | 18.03 ns | 897.80 ns | 2.11 us | 27.45 us | 2.55 us | 14.84 us | 49.79x | 0.03x | 0.35x | 0.06x |
| `atan` | 15.47 ns | 284.12 ns | 948.97 ns | 18.70 us | 2.29 us | 19.04 us | 18.37x | 0.02x | 0.12x | 0.01x |
| `atan abort` | 22.09 ns | 287.49 ns | 967.86 ns | 18.65 us | 2.26 us | 19.02 us | 13.02x | 0.02x | 0.13x | 0.02x |
| `asinh` | 49.88 ns | 415.60 ns | 1.10 us | 39.98 us | 1.66 us | 7.69 us | 8.33x | 0.01x | 0.25x | 0.05x |
| `asinh abort` | 32.00 ns | 436.71 ns | 1.09 us | 40.01 us | 1.67 us | 7.79 us | 13.65x | 0.01x | 0.26x | 0.06x |
| `acosh` | 12.53 ns | 362.39 ns | 1.11 us | 40.40 us | 3.41 us | 10.58 us | 28.93x | 0.01x | 0.11x | 0.03x |
| `acosh abort` | 17.25 ns | 356.37 ns | 1.15 us | 40.68 us | 3.42 us | 10.74 us | 20.65x | 0.01x | 0.10x | 0.03x |
| `atanh` | 13.77 ns | 693.91 ns | 1.37 us | 35.98 us | 1.31 us | 15.06 us | 50.39x | 0.02x | 0.53x | 0.05x |
| `atanh abort` | 17.34 ns | 716.54 ns | 1.39 us | 35.41 us | 1.31 us | 15.31 us | 41.33x | 0.02x | 0.55x | 0.05x |
| `zero status` | 1.21 ns | 2.37 ns | 1.65 ns | 0.99 ns | 6.82 ns | 8.22 ns | 1.96x | 2.39x | 0.35x | 0.29x |
| `zero status abort` | 1.44 ns | 3.18 ns | 3.26 ns | 1.07 ns | 6.81 ns | 8.32 ns | 2.21x | 2.97x | 0.47x | 0.38x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.95 ns | 49.04 ns | 48.56 ns | 48.07 ns | - | 1.92 ns | 51.56x | 1.02x | - | 25.51x |
| `one` | 5.17 ns | 56.11 ns | 55.47 ns | 64.67 ns | - | 30.43 ns | 10.85x | 0.87x | - | 1.84x |
| `i` | 0.97 ns | 58.39 ns | 58.15 ns | 65.19 ns | - | 32.13 ns | 60.41x | 0.90x | - | 1.82x |
| `free i` | 0.96 ns | 57.55 ns | 57.31 ns | 62.99 ns | - | 32.64 ns | 59.74x | 0.91x | - | 1.76x |
| `conjugate` | 2.19 ns | 103.49 ns | 104.93 ns | 41.92 ns | - | 1.14 us | 47.26x | 2.47x | - | 0.09x |
| `norm squared` | 5.96 ns | 345.29 ns | 1.46 us | 156.75 ns | - | 4.39 us | 57.98x | 2.20x | - | 0.08x |
| `reciprocal` | 18.73 ns | 1.76 us | 2.83 us | 450.38 ns | - | 10.92 us | 93.74x | 3.90x | - | 0.16x |
| `reciprocal checked` | 15.66 ns | 1.80 us | 2.92 us | 453.80 ns | - | 10.90 us | 114.90x | 3.96x | - | 0.16x |
| `powi` | 18.51 ns | 2.13 us | 13.79 us | 1.48 us | - | 44.48 us | 115.23x | 1.44x | - | 0.05x |
| `powi checked` | 18.21 ns | 2.11 us | 13.64 us | 1.52 us | - | 44.84 us | 115.82x | 1.39x | - | 0.05x |
| `div checked` | 18.08 ns | 2.71 us | 6.26 us | 798.85 ns | - | 22.75 us | 149.94x | 3.39x | - | 0.12x |
| `div real checked` | 9.41 ns | 667.81 ns | 657.87 ns | 271.35 ns | - | 5.25 us | 70.93x | 2.46x | - | 0.13x |
| `from scalar` | 1.42 ns | 70.83 ns | 71.01 ns | 45.21 ns | - | 10.16 ns | 50.00x | 1.57x | - | 6.97x |
| `add` | 6.29 ns | 468.92 ns | 846.15 ns | 104.69 ns | - | 2.68 us | 74.50x | 4.48x | - | 0.18x |
| `sub` | 6.41 ns | 483.74 ns | 913.21 ns | 117.29 ns | - | 5.07 us | 75.43x | 4.12x | - | 0.10x |
| `neg` | 2.67 ns | 103.99 ns | 105.97 ns | 43.48 ns | - | 2.22 us | 38.97x | 2.39x | - | 0.05x |
| `mul` | 7.72 ns | 859.87 ns | 3.17 us | 311.14 ns | - | 10.28 us | 111.37x | 2.76x | - | 0.08x |
| `div` | 17.67 ns | 2.73 us | 6.23 us | 800.04 ns | - | 22.42 us | 154.43x | 3.41x | - | 0.12x |
| `div real` | 10.44 ns | 659.42 ns | 653.65 ns | 267.69 ns | - | 5.39 us | 63.14x | 2.46x | - | 0.12x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.96 ns | 694.95 ns | 5.71 us | 324.73 ns | 256.62 ns | 7.43 us | 99.84x | 2.14x | 2.71x | 0.09x |
| `vec3 magnitude` | 9.86 ns | 4.61 us | 8.69 us | 5.54 us | 352.86 ns | 9.01 us | 467.39x | 0.83x | 13.06x | 0.51x |
| `vec3 normalize` | 25.33 ns | 8.64 us | 14.86 us | 6.26 us | 595.49 ns | 17.30 us | 341.04x | 1.38x | 14.51x | 0.50x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.08 ns | 219.42 ns | 1.46 us | 73.43 ns | 58.21 ns | 742.56 ns | 71.34x | 2.99x | 3.77x | 0.30x |
| `vec3 zero` | 1.42 ns | 140.77 ns | 141.41 ns | 61.33 ns | 31.68 ns | 2.87 ns | 99.44x | 2.30x | 4.44x | 49.03x |
| `vec3 dot abort` | 27.84 ns | 972.45 ns | 4.18 us | 261.95 ns | 213.70 ns | 7.20 us | 34.93x | 3.71x | 4.55x | 0.14x |
| `vec3 magnitude abort` | 39.65 ns | 4.82 us | 6.47 us | 5.53 us | 330.11 ns | 9.13 us | 121.61x | 0.87x | 14.61x | 0.53x |
| `vec3 normalize checked` | 26.65 ns | 8.65 us | 10.28 us | 6.04 us | 550.29 ns | 17.61 us | 324.75x | 1.43x | 15.73x | 0.49x |
| `vec3 normalize checked abort` | 55.62 ns | 9.04 us | 10.43 us | 6.04 us | 548.22 ns | 17.14 us | 162.54x | 1.50x | 16.49x | 0.53x |
| `vec3 div scalar checked` | 10.69 ns | 1.34 us | 1.61 us | 392.67 ns | - | - | 125.67x | 3.42x | - | - |
| `vec3 div scalar checked abort` | 18.61 ns | 1.33 us | 1.62 us | 394.32 ns | - | - | 71.32x | 3.37x | - | - |
| `vec3 add` | 6.83 ns | 947.34 ns | 1.68 us | 153.11 ns | 127.58 ns | 4.20 us | 138.66x | 6.19x | 7.43x | 0.23x |
| `vec3 add scalar` | 6.41 ns | 768.12 ns | 1.44 us | 157.11 ns | 133.97 ns | 4.09 us | 119.84x | 4.89x | 5.73x | 0.19x |
| `vec3 sub` | 6.64 ns | 934.85 ns | 1.80 us | 171.72 ns | 137.71 ns | 7.48 us | 140.85x | 5.44x | 6.79x | 0.13x |
| `vec3 sub scalar` | 6.59 ns | 752.26 ns | 1.24 us | 152.83 ns | 129.00 ns | 7.46 us | 114.22x | 4.92x | 5.83x | 0.10x |
| `vec3 neg` | 3.78 ns | 150.99 ns | 147.32 ns | 57.18 ns | 47.74 ns | 3.26 us | 39.91x | 2.64x | 3.16x | 0.05x |
| `vec3 mul scalar` | 6.93 ns | 519.40 ns | 2.04 us | 165.55 ns | 123.38 ns | 4.45 us | 75.00x | 3.14x | 4.21x | 0.12x |
| `vec3 div scalar` | 10.41 ns | 1.30 us | 1.56 us | 412.20 ns | 178.71 ns | 7.75 us | 125.13x | 3.16x | 7.29x | 0.17x |
| `vec4 dot` | 7.50 ns | 840.35 ns | 3.24 us | 448.70 ns | 326.83 ns | 10.25 us | 112.02x | 1.87x | 2.57x | 0.08x |
| `vec4 magnitude` | 13.20 ns | 4.48 us | 5.45 us | 5.59 us | 425.88 ns | 11.56 us | 339.40x | 0.80x | 10.52x | 0.39x |
| `vec4 normalize` | 37.25 ns | 9.00 us | 9.26 us | 6.32 us | 731.80 ns | 22.71 us | 241.56x | 1.42x | 12.30x | 0.40x |
| `vec4 add` | 7.98 ns | 1.19 us | 1.80 us | 210.83 ns | 179.01 ns | 5.68 us | 149.66x | 5.67x | 6.67x | 0.21x |
| `vec4 add scalar` | 6.95 ns | 1.02 us | 1.75 us | 222.50 ns | 182.42 ns | 5.44 us | 146.68x | 4.58x | 5.59x | 0.19x |
| `vec4 sub` | 5.35 ns | 1.15 us | 1.70 us | 214.49 ns | 183.02 ns | 10.12 us | 214.68x | 5.35x | 6.27x | 0.11x |
| `vec4 sub scalar` | 4.49 ns | 980.60 ns | 1.46 us | 212.63 ns | 171.96 ns | 9.63 us | 218.20x | 4.61x | 5.70x | 0.10x |
| `vec4 neg` | 4.98 ns | 191.21 ns | 188.05 ns | 77.97 ns | 68.44 ns | 4.11 us | 38.36x | 2.45x | 2.79x | 0.05x |
| `vec4 mul scalar` | 7.37 ns | 646.40 ns | 2.04 us | 225.24 ns | 166.87 ns | 5.71 us | 87.73x | 2.87x | 3.87x | 0.11x |
| `vec4 div scalar` | 14.23 ns | 1.77 us | 1.55 us | 524.32 ns | 230.44 ns | 10.14 us | 124.11x | 3.37x | 7.66x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.74 ns | 1.51 us | 3.17 us | 964.87 ns | 886.41 ns | 22.73 us | 118.88x | 1.57x | 1.71x | 0.07x |
| `mat3 inverse` | 81.31 ns | 17.31 us | 10.92 us | 3.22 us | 2.57 us | 84.54 us | 212.91x | 5.38x | 6.75x | 0.20x |
| `mat3 mul mat3` | 73.39 ns | 6.48 us | 10.99 us | 2.81 us | 2.45 us | 62.75 us | 88.34x | 2.31x | 2.64x | 0.10x |
| `mat3 transform vec3` | 16.20 ns | 2.66 us | 10.10 us | 1.05 us | 929.78 ns | 20.58 us | 164.51x | 2.54x | 2.87x | 0.13x |
| `mat4 determinant` | 45.83 ns | 4.30 us | 3.13 us | 4.76 us | 4.25 us | 100.75 us | 93.77x | 0.90x | 1.01x | 0.04x |
| `mat4 inverse` | 145.23 ns | 31.66 us | 13.66 us | 11.25 us | 9.18 us | 354.71 us | 217.97x | 2.81x | 3.45x | 0.09x |
| `mat4 mul mat4` | 119.63 ns | 13.08 us | 10.69 us | 6.59 us | 5.43 us | 149.95 us | 109.32x | 1.99x | 2.41x | 0.09x |
| `mat4 transform vec4` | 26.05 ns | 4.15 us | 3.31 us | 1.91 us | 1.73 us | 36.29 us | 159.22x | 2.17x | 2.40x | 0.11x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.97 ns | 1.13 us | 4.51 us | 208.16 ns | 240.42 ns | 2.17 us | 32.34x | 5.43x | 4.70x | 0.52x |
| `mat3 zero` | 16.87 ns | 535.18 ns | 542.59 ns | 265.21 ns | 219.02 ns | 11.59 ns | 31.72x | 2.02x | 2.44x | 46.18x |
| `mat3 identity` | 10.22 ns | 647.55 ns | 668.33 ns | 333.41 ns | 258.24 ns | 153.86 ns | 63.38x | 1.94x | 2.51x | 4.21x |
| `mat3 transpose` | 9.27 ns | 699.78 ns | 713.03 ns | 235.66 ns | 217.36 ns | 116.18 ns | 75.52x | 2.97x | 3.22x | 6.02x |
| `mat3 reciprocal` | 79.63 ns | 16.79 us | 35.04 us | 3.06 us | 2.33 us | 84.88 us | 210.90x | 5.49x | 7.20x | 0.20x |
| `mat3 reciprocal checked` | 79.61 ns | 16.89 us | 34.99 us | 3.08 us | 2.32 us | 86.05 us | 212.17x | 5.48x | 7.28x | 0.20x |
| `mat3 inverse checked` | 81.69 ns | 17.25 us | 34.81 us | 3.12 us | 2.36 us | 86.54 us | 211.15x | 5.53x | 7.30x | 0.20x |
| `mat3 inverse checked abort` | 116.38 ns | 17.07 us | 35.59 us | 3.06 us | 2.36 us | 84.06 us | 146.65x | 5.57x | 7.23x | 0.20x |
| `mat3 powi` | 146.88 ns | 13.27 us | 85.53 us | 6.97 us | 6.47 us | 154.71 us | 90.35x | 1.90x | 2.05x | 0.09x |
| `mat3 powi checked` | 147.63 ns | 13.27 us | 89.12 us | 6.98 us | 6.47 us | 155.71 us | 89.87x | 1.90x | 2.05x | 0.09x |
| `mat3 powi checked abort` | 147.81 ns | 13.26 us | 88.30 us | 6.84 us | 6.44 us | 156.03 us | 89.71x | 1.94x | 2.06x | 0.08x |
| `mat3 div scalar checked` | 25.67 ns | 4.36 us | 5.82 us | 1.43 us | 803.08 ns | 23.00 us | 169.85x | 3.05x | 5.43x | 0.19x |
| `mat3 div scalar checked abort` | 30.57 ns | 4.29 us | 5.64 us | 1.44 us | 815.13 ns | 23.06 us | 140.28x | 2.98x | 5.26x | 0.19x |
| `mat3 div matrix checked` | 193.30 ns | 25.00 us | 59.25 us | 5.50 us | 4.80 us | 165.67 us | 129.33x | 4.54x | 5.20x | 0.15x |
| `mat3 div matrix checked abort` | 203.23 ns | 24.84 us | 59.03 us | 5.54 us | 4.64 us | 164.74 us | 122.24x | 4.48x | 5.35x | 0.15x |
| `mat3 add` | 14.54 ns | 3.31 us | 5.33 us | 526.21 ns | 501.07 ns | 12.45 us | 227.90x | 6.30x | 6.61x | 0.27x |
| `mat3 add scalar` | 11.99 ns | 2.55 us | 4.80 us | 793.87 ns | 717.05 ns | 12.63 us | 212.41x | 3.21x | 3.55x | 0.20x |
| `mat3 sub` | 13.60 ns | 3.23 us | 5.39 us | 563.32 ns | 515.90 ns | 22.12 us | 237.85x | 5.74x | 6.27x | 0.15x |
| `mat3 sub scalar` | 11.70 ns | 2.51 us | 4.62 us | 816.61 ns | 703.53 ns | 23.63 us | 214.88x | 3.08x | 3.57x | 0.11x |
| `mat3 neg` | 11.38 ns | 632.27 ns | 647.26 ns | 481.89 ns | 464.23 ns | 9.46 us | 55.57x | 1.31x | 1.36x | 0.07x |
| `mat3 mul scalar` | 14.16 ns | 1.74 us | 5.82 us | 800.67 ns | 693.15 ns | 12.72 us | 122.75x | 2.17x | 2.51x | 0.14x |
| `mat3 div scalar` | 24.90 ns | 4.37 us | 5.73 us | 1.47 us | 816.06 ns | 22.52 us | 175.60x | 2.98x | 5.36x | 0.19x |
| `mat3 div matrix` | 163.30 ns | 25.28 us | 60.22 us | 5.53 us | 4.66 us | 163.11 us | 154.79x | 4.57x | 5.43x | 0.15x |
| `mat3 bitxor` | 145.65 ns | 13.63 us | 85.03 us | 6.80 us | 6.38 us | 156.92 us | 93.56x | 2.01x | 2.14x | 0.09x |
| `mat4 zero` | 13.50 ns | 824.00 ns | 792.17 ns | 531.23 ns | 346.82 ns | 14.46 ns | 61.05x | 1.55x | 2.38x | 56.99x |
| `mat4 identity` | 10.21 ns | 995.66 ns | 970.71 ns | 580.75 ns | 419.30 ns | 238.02 ns | 97.54x | 1.71x | 2.37x | 4.18x |
| `mat4 transpose` | 10.08 ns | 1.18 us | 1.17 us | 465.29 ns | 382.25 ns | 177.18 ns | 116.85x | 2.53x | 3.08x | 6.64x |
| `mat4 reciprocal` | 147.06 ns | 31.61 us | 61.80 us | 11.04 us | 8.99 us | 359.53 us | 214.91x | 2.86x | 3.51x | 0.09x |
| `mat4 reciprocal checked` | 167.40 ns | 31.68 us | 62.92 us | 10.98 us | 8.78 us | 349.73 us | 189.26x | 2.89x | 3.61x | 0.09x |
| `mat4 powi` | 247.30 ns | 26.61 us | 105.42 us | 15.98 us | 14.11 us | 369.44 us | 107.60x | 1.66x | 1.89x | 0.07x |
| `mat4 powi checked` | 248.09 ns | 27.35 us | 104.13 us | 16.00 us | 14.06 us | 358.82 us | 110.25x | 1.71x | 1.94x | 0.08x |
| `mat4 add` | 51.77 ns | 5.40 us | 6.14 us | 965.52 ns | 860.34 ns | 20.80 us | 104.39x | 5.60x | 6.28x | 0.26x |
| `mat4 add scalar` | 20.88 ns | 4.08 us | 6.65 us | 1.38 us | 1.19 us | 21.96 us | 195.61x | 2.95x | 3.45x | 0.19x |
| `mat4 sub` | 37.86 ns | 5.42 us | 6.23 us | 1.01 us | 919.89 ns | 38.41 us | 143.18x | 5.36x | 5.89x | 0.14x |
| `mat4 sub scalar` | 15.00 ns | 4.06 us | 6.65 us | 1.41 us | 1.18 us | 38.55 us | 270.54x | 2.89x | 3.44x | 0.11x |
| `mat4 neg` | 14.15 ns | 1.12 us | 1.12 us | 892.44 ns | 756.91 ns | 14.95 us | 78.88x | 1.25x | 1.47x | 0.07x |
| `mat4 mul scalar` | 24.28 ns | 2.89 us | 7.38 us | 1.44 us | 1.15 us | 21.33 us | 119.15x | 2.01x | 2.51x | 0.14x |
| `mat4 div scalar` | 33.67 ns | 7.52 us | 7.64 us | 2.63 us | 1.40 us | 38.54 us | 223.31x | 2.85x | 5.37x | 0.20x |
| `mat4 div matrix` | 239.66 ns | 44.80 us | 90.79 us | 17.40 us | 14.36 us | 542.75 us | 186.92x | 2.57x | 3.12x | 0.08x |
| `mat4 bitxor` | 248.64 ns | 26.65 us | 103.79 us | 16.20 us | 14.09 us | 360.30 us | 107.20x | 1.65x | 1.89x | 0.07x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.63 ns | 180.16 ns | 294.30 ns | - | - | 1.35 us | 49.63x | - | - | 0.13x |
| `scalar add ref_owned` | 12.53 ns | 188.55 ns | 298.05 ns | - | - | 1.37 us | 15.04x | - | - | 0.14x |
| `scalar add refs` | 5.43 ns | 161.57 ns | 272.88 ns | - | - | 1.35 us | 29.74x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 9.33 ns | 213.65 ns | 308.95 ns | - | - | - | 22.91x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.68 ns | 204.65 ns | 303.13 ns | - | - | - | 36.06x | - | - | - |
| `scalar sub owned_ref` | 4.00 ns | 191.24 ns | 339.80 ns | - | - | 2.53 us | 47.80x | - | - | 0.08x |
| `scalar sub ref_owned` | 12.69 ns | 190.85 ns | 339.69 ns | - | - | 2.50 us | 15.04x | - | - | 0.08x |
| `scalar sub refs` | 5.67 ns | 165.27 ns | 336.48 ns | - | - | 2.52 us | 29.15x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 9.06 ns | 212.65 ns | 372.80 ns | - | - | - | 23.47x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.86 ns | 200.37 ns | 365.14 ns | - | - | - | 34.17x | - | - | - |
| `scalar mul owned_ref` | 4.36 ns | 106.53 ns | 381.51 ns | - | - | 1.56 us | 24.44x | - | - | 0.07x |
| `scalar mul ref_owned` | 13.43 ns | 107.56 ns | 392.02 ns | - | - | 1.57 us | 8.01x | - | - | 0.07x |
| `scalar mul refs` | 6.12 ns | 84.01 ns | 373.80 ns | - | - | 1.58 us | 13.72x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.93 ns | 113.74 ns | 395.65 ns | - | - | - | 11.45x | - | - | - |
| `scalar mul ref_owned_with_clone` | 8.01 ns | 110.24 ns | 401.89 ns | - | - | - | 13.77x | - | - | - |
| `scalar div owned_ref` | 6.12 ns | 267.47 ns | 326.09 ns | - | - | 2.70 us | 43.71x | - | - | 0.10x |
| `scalar div ref_owned` | 17.62 ns | 279.97 ns | 329.76 ns | - | - | 2.66 us | 15.89x | - | - | 0.11x |
| `scalar div refs` | 7.25 ns | 256.52 ns | 301.32 ns | - | - | 2.67 us | 35.38x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.59 ns | 282.14 ns | 330.68 ns | - | - | - | 20.75x | - | - | - |
| `scalar div ref_owned_with_clone` | 8.06 ns | 277.59 ns | 333.36 ns | - | - | - | 34.43x | - | - | - |
| `vec3 add refs` | 6.29 ns | 635.30 ns | 1.43 us | - | - | 4.21 us | 101.06x | - | - | 0.15x |
| `vec3 sub refs` | 6.33 ns | 632.24 ns | 1.46 us | - | - | 7.64 us | 99.96x | - | - | 0.08x |
| `vec3 neg ref` | 3.31 ns | 159.42 ns | 166.99 ns | - | - | 3.26 us | 48.21x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.65 ns | 773.55 ns | 1.46 us | - | - | 4.02 us | 116.37x | - | - | 0.19x |
| `vec3 sub_scalar_ref` | 6.72 ns | 765.96 ns | 1.25 us | - | - | 7.28 us | 113.95x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 7.04 ns | 527.54 ns | 2.08 us | - | - | 4.62 us | 74.98x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.44 ns | 1.32 us | 1.54 us | - | - | 7.75 us | 156.68x | - | - | 0.17x |
| `vec4 add refs` | 6.65 ns | 770.10 ns | 1.39 us | - | - | 5.47 us | 115.76x | - | - | 0.14x |
| `vec4 sub refs` | 3.20 ns | 775.76 ns | 1.27 us | - | - | 10.13 us | 242.49x | - | - | 0.08x |
| `vec4 neg ref` | 4.33 ns | 235.38 ns | 233.97 ns | - | - | 4.15 us | 54.32x | - | - | 0.06x |
| `vec4 add_scalar_ref` | 7.53 ns | 1.02 us | 1.75 us | - | - | 5.20 us | 134.83x | - | - | 0.20x |
| `vec4 sub_scalar_ref` | 4.24 ns | 987.59 ns | 1.50 us | - | - | 9.90 us | 232.69x | - | - | 0.10x |
| `vec4 mul_scalar_ref` | 7.46 ns | 659.18 ns | 2.12 us | - | - | 5.77 us | 88.32x | - | - | 0.11x |
| `vec4 div_scalar_ref` | 11.97 ns | 1.69 us | 1.52 us | - | - | 10.23 us | 141.28x | - | - | 0.17x |
| `mat3 add refs` | 11.15 ns | 1.80 us | 3.94 us | - | - | 12.21 us | 161.29x | - | - | 0.15x |
| `mat3 sub refs` | 10.65 ns | 1.83 us | 3.75 us | - | - | 22.66 us | 171.65x | - | - | 0.08x |
| `mat3 mul refs` | 32.21 ns | 4.97 us | 27.85 us | - | - | 63.44 us | 154.20x | - | - | 0.08x |
| `mat3 div refs` | 146.74 ns | 24.33 us | 60.36 us | - | - | 166.08 us | 165.80x | - | - | 0.15x |
| `mat3 neg ref` | 10.24 ns | 510.77 ns | 533.48 ns | - | - | 8.77 us | 49.90x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 45.25 ns | 2.38 us | 4.97 us | - | - | 12.45 us | 52.53x | - | - | 0.19x |
| `mat3 sub_scalar_ref` | 45.58 ns | 2.38 us | 4.47 us | - | - | 22.19 us | 52.29x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 48.51 ns | 1.71 us | 5.90 us | - | - | 12.71 us | 35.22x | - | - | 0.13x |
| `mat3 div_scalar_ref` | 23.08 ns | 4.35 us | 5.71 us | - | - | 22.76 us | 188.70x | - | - | 0.19x |
| `mat4 add refs` | 16.80 ns | 2.78 us | 3.60 us | - | - | 21.30 us | 165.50x | - | - | 0.13x |
| `mat4 sub refs` | 16.69 ns | 2.87 us | 3.72 us | - | - | 37.98 us | 172.07x | - | - | 0.08x |
| `mat4 mul refs` | 74.21 ns | 10.45 us | 27.26 us | - | - | 149.26 us | 140.82x | - | - | 0.07x |
| `mat4 div refs` | 182.54 ns | 44.42 us | 90.47 us | - | - | 552.21 us | 243.33x | - | - | 0.08x |
| `mat4 neg ref` | 12.22 ns | 769.22 ns | 723.15 ns | - | - | 14.25 us | 62.97x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 49.95 ns | 4.14 us | 6.85 us | - | - | 21.41 us | 82.88x | - | - | 0.19x |
| `mat4 sub_scalar_ref` | 37.93 ns | 4.13 us | 6.63 us | - | - | 39.46 us | 108.90x | - | - | 0.10x |
| `mat4 mul_scalar_ref` | 53.95 ns | 2.94 us | 7.53 us | - | - | 21.30 us | 54.59x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 27.67 ns | 7.65 us | 8.00 us | - | - | 38.97 us | 276.56x | - | - | 0.20x |
| `mat3 transform_vec refs` | 14.87 ns | 2.04 us | 11.15 us | - | - | 21.23 us | 137.45x | - | - | 0.10x |
| `mat4 transform_vec refs` | 23.23 ns | 3.21 us | 10.94 us | - | - | 37.01 us | 138.26x | - | - | 0.09x |
| `complex add refs` | 7.91 ns | 342.79 ns | 699.71 ns | - | - | 2.76 us | 43.34x | - | - | 0.12x |
| `complex sub refs` | 8.22 ns | 343.58 ns | 716.49 ns | - | - | 4.94 us | 41.79x | - | - | 0.07x |
| `complex mul refs` | 8.18 ns | 743.50 ns | 3.03 us | - | - | 10.45 us | 90.86x | - | - | 0.07x |
| `complex div refs` | 17.52 ns | 2.61 us | 6.26 us | - | - | 22.91 us | 148.91x | - | - | 0.11x |
| `complex neg ref` | 2.36 ns | 77.23 ns | 73.72 ns | - | - | 2.21 us | 32.69x | - | - | 0.03x |
| `complex div_real_ref` | 7.32 ns | 654.55 ns | 607.26 ns | - | - | 5.22 us | 89.37x | - | - | 0.13x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.61 us |
| `astro sin 160` | 14.41 us |
| `astro sin 192` | 14.45 us |
| `astro sin 256` | 16.47 us |
