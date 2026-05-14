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
| `sin 0.1` | 10.78 ns | 145.72 ns | 143.44 ns | 10.64 us | 759.23 ns | 1.79 us | 13.51x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.56 ns | 146.07 ns | 142.38 ns | 10.18 us | 499.01 ns | 1.64 us | 12.64x | 0.01x | 0.29x | 0.09x |
| `sin 1.23456789` | 11.61 ns | 203.52 ns | 189.82 ns | 12.12 us | 812.33 ns | 1.77 us | 17.52x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 11.87 ns | 194.04 ns | 180.15 ns | 10.31 us | 597.32 ns | 1.64 us | 16.35x | 0.02x | 0.32x | 0.12x |
| `sin 1e6` | 12.51 ns | 94.53 ns | 92.97 ns | 15.68 us | 1.12 us | 1.98 us | 7.56x | 0.01x | 0.08x | 0.05x |
| `cos 1e6` | 12.22 ns | 91.90 ns | 92.09 ns | 13.59 us | 826.26 ns | 1.80 us | 7.52x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 65.36 ns | 93.56 ns | 93.68 ns | 18.24 us | 2.86 us | 3.47 us | 1.43x | 0.01x | 0.03x | 0.03x |
| `cos 1e30` | 71.87 ns | 92.96 ns | 93.01 ns | 15.40 us | 960.09 ns | 3.01 us | 1.29x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.55 ns | 146.37 ns | 363.25 ns | 11.77 us | 742.46 ns | 1.83 us | 12.67x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.40 ns | 145.38 ns | 682.26 ns | 10.45 us | 538.16 ns | 1.67 us | 12.75x | 0.01x | 0.27x | 0.09x |
| `sin 1000pi_eps` | 11.71 ns | 93.46 ns | 722.17 ns | 15.80 us | 2.30 us | 2.75 us | 7.98x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.17 ns | 91.83 ns | 710.24 ns | 13.37 us | 566.43 ns | 1.64 us | 7.54x | 0.01x | 0.16x | 0.06x |
| `asin 0.5` | 10.62 ns | 138.34 ns | 140.42 ns | 50.54 us | 2.90 us | 13.01 us | 13.02x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.12 ns | 412.49 ns | 412.46 ns | 59.05 us | 2.90 us | 12.91 us | 37.08x | 0.01x | 0.14x | 0.03x |
| `atanh 0.5` | 14.27 ns | 235.49 ns | 237.17 ns | 34.17 us | 1.63 us | 12.91 us | 16.50x | 0.01x | 0.14x | 0.02x |
| `asin neg_0.999999` | 14.27 ns | 560.86 ns | 536.00 ns | 13.72 us | 2.53 us | 12.79 us | 39.31x | 0.04x | 0.22x | 0.04x |
| `acos neg_0.999999` | 16.06 ns | 534.73 ns | 509.10 ns | 18.01 us | 2.68 us | 12.89 us | 33.29x | 0.03x | 0.20x | 0.04x |
| `atanh neg_0.999999` | 14.38 ns | 207.09 ns | 190.25 ns | 36.39 us | 1.57 us | 12.79 us | 14.40x | 0.01x | 0.13x | 0.02x |
| `asin 0.999999` | 14.18 ns | 511.20 ns | 709.28 ns | 13.84 us | 2.52 us | 12.80 us | 36.06x | 0.04x | 0.20x | 0.04x |
| `acos 0.999999` | 14.39 ns | 298.67 ns | 537.51 ns | 18.29 us | 2.71 us | 12.95 us | 20.76x | 0.02x | 0.11x | 0.02x |
| `atanh 0.999999` | 14.42 ns | 212.25 ns | 202.10 ns | 31.47 us | 1.57 us | 12.40 us | 14.71x | 0.01x | 0.14x | 0.02x |
| `asin 1e-12` | 9.29 ns | 272.75 ns | 492.39 ns | 8.08 us | 1.46 us | 15.00 us | 29.35x | 0.03x | 0.19x | 0.02x |
| `acos 1e-12` | 9.91 ns | 472.96 ns | 675.79 ns | 9.53 us | 1.47 us | 14.90 us | 47.74x | 0.05x | 0.32x | 0.03x |
| `atanh 1e-12` | 9.54 ns | 255.55 ns | 218.37 ns | 36.29 us | 174.56 ns | 19.53 us | 26.79x | 0.01x | 1.46x | 0.01x |
| `atan 0.5` | 14.45 ns | 167.01 ns | 167.59 ns | 34.80 us | 2.75 us | 17.16 us | 11.56x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.19 ns | 263.39 ns | 269.59 ns | 39.18 us | 1.61 us | 7.46 us | 10.06x | 0.01x | 0.16x | 0.04x |
| `atan neg_1e-12` | 14.09 ns | 292.99 ns | 278.82 ns | 1.57 us | 1.15 us | 15.12 us | 20.80x | 0.19x | 0.26x | 0.02x |
| `asinh neg_1e-12` | 15.51 ns | 461.79 ns | 387.31 ns | 42.30 us | 8.53 us | 11.81 us | 29.78x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 14.82 ns | 158.14 ns | 161.75 ns | 2.80 us | 1.43 us | 17.69 us | 10.67x | 0.06x | 0.11x | 0.01x |
| `asinh 1e6` | 25.93 ns | 260.44 ns | 262.87 ns | 37.13 us | 1.62 us | 7.13 us | 10.04x | 0.01x | 0.16x | 0.04x |
| `atan neg_1e6` | 14.60 ns | 272.76 ns | 271.36 ns | 2.86 us | 1.41 us | 17.74 us | 18.69x | 0.10x | 0.19x | 0.02x |
| `asinh neg_1e6` | 26.96 ns | 368.30 ns | 378.05 ns | 37.00 us | 1.59 us | 6.94 us | 13.66x | 0.01x | 0.23x | 0.05x |
| `acosh 9` | 12.52 ns | 161.73 ns | 162.53 ns | 41.70 us | 1.59 us | 9.87 us | 12.91x | 0.00x | 0.10x | 0.02x |
| `acosh 1_plus_1e-12` | 11.54 ns | 247.57 ns | 244.98 ns | 41.21 us | 8.22 us | 11.31 us | 21.46x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.22 ns | 161.58 ns | 161.41 ns | 37.15 us | 1.54 us | 9.70 us | 13.23x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.44 ns | 240.75 ns | 2.24 us | 40.79 us | 1.61 us | 9.71 us | 19.35x | 0.01x | 0.15x | 0.02x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.02 ns | 18.97 ns | 24.81 ns | 15.25 ns | 0.92 ns | 38.41x | 0.73x | 1.18x | 19.51x |
| `one` | 0.47 ns | 22.34 ns | 26.78 ns | 40.09 ns | 29.92 ns | 29.69 ns | 48.01x | 0.56x | 0.75x | 0.75x |
| `e` | 0.47 ns | 87.63 ns | 88.82 ns | 56.76 ns | 1.05 us | 221.61 ns | 187.56x | 1.54x | 0.08x | 0.40x |
| `pi` | 0.47 ns | 49.82 ns | 51.56 ns | 47.45 ns | 47.37 ns | 221.23 ns | 107.04x | 1.05x | 1.05x | 0.23x |
| `tau` | 0.46 ns | 49.79 ns | 51.42 ns | 115.73 ns | 100.06 ns | 1.82 us | 107.55x | 0.43x | 0.50x | 0.03x |
| `add` | 5.06 ns | 206.49 ns | 319.13 ns | 52.18 ns | 42.27 ns | 1.25 us | 40.84x | 3.96x | 4.88x | 0.16x |
| `sub` | 5.10 ns | 249.06 ns | 499.51 ns | 56.12 ns | 43.86 ns | 2.36 us | 48.87x | 4.44x | 5.68x | 0.11x |
| `neg` | 5.02 ns | 39.02 ns | 39.83 ns | 22.85 ns | 21.58 ns | 1.04 us | 7.78x | 1.71x | 1.81x | 0.04x |
| `mul` | 5.31 ns | 134.94 ns | 401.71 ns | 55.37 ns | 45.22 ns | 1.48 us | 25.41x | 2.44x | 2.98x | 0.09x |
| `div` | 7.73 ns | 308.22 ns | 365.04 ns | 132.26 ns | 61.49 ns | 2.49 us | 39.85x | 2.33x | 5.01x | 0.12x |
| `reciprocal` | 8.45 ns | 73.03 ns | 75.56 ns | 160.96 ns | 58.51 ns | 1.51 us | 8.64x | 0.45x | 1.25x | 0.05x |
| `reciprocal checked` | 8.77 ns | 67.78 ns | 70.33 ns | 161.24 ns | 58.78 ns | 1.51 us | 7.73x | 0.42x | 1.15x | 0.04x |
| `reciprocal checked abort` | 23.13 ns | 81.48 ns | 77.80 ns | 157.54 ns | 60.02 ns | 1.50 us | 3.52x | 0.52x | 1.36x | 0.05x |
| `pow` | 29.69 ns | 6.51 us | 5.77 us | 53.51 us | 2.78 us | 2.29 us | 219.29x | 0.12x | 2.34x | 2.84x |
| `powi` | 5.84 ns | 371.27 ns | 2.64 us | 276.91 ns | 83.56 ns | 1.52 us | 63.58x | 1.34x | 4.44x | 0.24x |
| `exp` | 19.14 ns | 231.40 ns | 231.69 ns | 13.77 us | 919.47 ns | 1.84 us | 12.09x | 0.02x | 0.25x | 0.13x |
| `ln` | 10.39 ns | 1.48 us | 1.45 us | 29.48 us | 1.28 us | 1.76 us | 142.63x | 0.05x | 1.16x | 0.84x |
| `log10` | 13.10 ns | 1.61 us | 1.54 us | 34.84 us | 2.70 us | 6.43 us | 122.89x | 0.05x | 0.60x | 0.25x |
| `log10 abort` | 18.09 ns | 1.66 us | 1.54 us | 34.12 us | 2.70 us | 6.45 us | 91.66x | 0.05x | 0.62x | 0.26x |
| `sqrt` | 8.14 ns | 1.51 us | 1.66 us | 4.90 us | 94.85 ns | 1.43 us | 184.98x | 0.31x | 15.88x | 1.05x |
| `sin` | 14.78 ns | 121.95 ns | 120.60 ns | 13.77 us | 1.24 us | 2.16 us | 8.25x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.04 ns | 122.47 ns | 119.70 ns | 12.03 us | 631.12 ns | 1.71 us | 6.79x | 0.01x | 0.19x | 0.07x |
| `tan` | 24.22 ns | 175.26 ns | 181.81 ns | 28.99 us | 1.55 us | 6.40 us | 7.24x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.16 ns | 3.59 us | 3.51 us | 3.28 us | 1.11 us | 10.45 us | 197.84x | 1.09x | 3.23x | 0.34x |
| `cosh` | 17.80 ns | 3.50 us | 3.45 us | 7.76 us | 1.03 us | 9.40 us | 196.86x | 0.45x | 3.40x | 0.37x |
| `tanh` | 19.71 ns | 6.60 us | 6.66 us | 3.34 us | 1.19 us | 22.45 us | 335.04x | 1.98x | 5.56x | 0.29x |
| `asin` | 13.00 ns | 389.19 ns | 595.61 ns | 21.01 us | 2.41 us | 13.53 us | 29.93x | 0.02x | 0.16x | 0.03x |
| `asin abort` | 16.78 ns | 389.49 ns | 597.45 ns | 21.19 us | 2.41 us | 13.47 us | 23.21x | 0.02x | 0.16x | 0.03x |
| `acos` | 13.98 ns | 448.69 ns | 652.06 ns | 25.91 us | 2.50 us | 13.58 us | 32.10x | 0.02x | 0.18x | 0.03x |
| `acos abort` | 17.36 ns | 449.60 ns | 646.96 ns | 26.26 us | 2.53 us | 13.56 us | 25.91x | 0.02x | 0.18x | 0.03x |
| `atan` | 14.87 ns | 177.44 ns | 164.47 ns | 18.15 us | 2.21 us | 18.27 us | 11.94x | 0.01x | 0.08x | 0.01x |
| `atan abort` | 21.02 ns | 181.27 ns | 163.75 ns | 18.18 us | 2.21 us | 18.29 us | 8.62x | 0.01x | 0.08x | 0.01x |
| `asinh` | 48.11 ns | 265.84 ns | 273.40 ns | 38.67 us | 1.59 us | 7.37 us | 5.53x | 0.01x | 0.17x | 0.04x |
| `asinh abort` | 30.43 ns | 272.09 ns | 274.52 ns | 37.65 us | 1.62 us | 7.36 us | 8.94x | 0.01x | 0.17x | 0.04x |
| `acosh` | 12.12 ns | 198.04 ns | 205.64 ns | 40.12 us | 3.30 us | 10.38 us | 16.34x | 0.00x | 0.06x | 0.02x |
| `acosh abort` | 16.73 ns | 203.15 ns | 201.71 ns | 39.59 us | 3.30 us | 10.47 us | 12.14x | 0.01x | 0.06x | 0.02x |
| `atanh` | 13.56 ns | 226.04 ns | 253.60 ns | 34.18 us | 1.25 us | 14.50 us | 16.67x | 0.01x | 0.18x | 0.02x |
| `atanh abort` | 16.78 ns | 226.75 ns | 253.84 ns | 33.64 us | 1.23 us | 14.43 us | 13.51x | 0.01x | 0.19x | 0.02x |
| `zero status` | 1.20 ns | 1.13 ns | 1.07 ns | 0.99 ns | 6.66 ns | 7.87 ns | 0.94x | 1.14x | 0.17x | 0.14x |
| `zero status abort` | 1.38 ns | 0.97 ns | 1.03 ns | 1.02 ns | 6.66 ns | 7.84 ns | 0.71x | 0.96x | 0.15x | 0.12x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.91 ns | 46.13 ns | 46.21 ns | 47.25 ns | - | 1.87 ns | 50.89x | 0.98x | - | 24.70x |
| `one` | 4.91 ns | 53.84 ns | 52.91 ns | 62.59 ns | - | 29.15 ns | 10.97x | 0.86x | - | 1.85x |
| `i` | 0.91 ns | 53.10 ns | 52.50 ns | 62.60 ns | - | 31.55 ns | 58.56x | 0.85x | - | 1.68x |
| `free i` | 0.91 ns | 52.81 ns | 52.77 ns | 62.37 ns | - | 31.14 ns | 57.95x | 0.85x | - | 1.70x |
| `conjugate` | 2.07 ns | 103.33 ns | 99.72 ns | 40.88 ns | - | 1.06 us | 49.87x | 2.53x | - | 0.10x |
| `norm squared` | 5.61 ns | 168.44 ns | 564.18 ns | 153.62 ns | - | 4.24 us | 30.00x | 1.10x | - | 0.04x |
| `reciprocal` | 18.27 ns | 1.71 us | 2.75 us | 443.36 ns | - | 10.47 us | 93.87x | 3.87x | - | 0.16x |
| `reciprocal checked` | 14.96 ns | 1.69 us | 2.82 us | 440.28 ns | - | 10.49 us | 112.84x | 3.83x | - | 0.16x |
| `powi` | 17.56 ns | 2.04 us | 14.37 us | 1.39 us | - | 42.73 us | 116.19x | 1.46x | - | 0.05x |
| `powi checked` | 17.33 ns | 2.07 us | 14.66 us | 1.43 us | - | 43.38 us | 119.46x | 1.44x | - | 0.05x |
| `div checked` | 21.97 ns | 2.30 us | 4.57 us | 777.91 ns | - | 21.36 us | 104.47x | 2.95x | - | 0.11x |
| `div real checked` | 9.13 ns | 643.01 ns | 616.07 ns | 263.84 ns | - | 5.12 us | 70.40x | 2.44x | - | 0.13x |
| `from scalar` | 1.40 ns | 68.89 ns | 69.31 ns | 43.65 ns | - | 9.87 ns | 49.18x | 1.58x | - | 6.98x |
| `add` | 6.07 ns | 460.73 ns | 841.10 ns | 105.28 ns | - | 2.53 us | 75.93x | 4.38x | - | 0.18x |
| `sub` | 6.07 ns | 508.48 ns | 1.13 us | 112.75 ns | - | 4.70 us | 83.75x | 4.51x | - | 0.11x |
| `neg` | 2.54 ns | 99.09 ns | 100.90 ns | 42.11 ns | - | 2.11 us | 39.01x | 2.35x | - | 0.05x |
| `mul` | 7.38 ns | 879.31 ns | 3.51 us | 298.05 ns | - | 9.88 us | 119.09x | 2.95x | - | 0.09x |
| `div` | 18.62 ns | 2.26 us | 4.64 us | 778.09 ns | - | 21.46 us | 121.59x | 2.91x | - | 0.11x |
| `div real` | 9.88 ns | 636.49 ns | 637.72 ns | 258.73 ns | - | 5.13 us | 64.42x | 2.46x | - | 0.12x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.57 ns | 217.55 ns | 2.43 us | 314.77 ns | 251.70 ns | 7.23 us | 25.38x | 0.69x | 0.86x | 0.03x |
| `vec3 magnitude` | 11.57 ns | 4.00 us | 5.44 us | 5.41 us | 351.53 ns | 8.86 us | 345.80x | 0.74x | 11.38x | 0.45x |
| `vec3 normalize` | 25.38 ns | 8.38 us | 11.48 us | 5.94 us | 589.08 ns | 17.48 us | 330.04x | 1.41x | 14.22x | 0.48x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.05 ns | 310.40 ns | 1.45 us | 69.09 ns | 57.54 ns | 704.75 ns | 101.82x | 4.49x | 5.39x | 0.44x |
| `vec3 zero` | 1.39 ns | 131.01 ns | 130.70 ns | 64.15 ns | 30.85 ns | 2.80 ns | 94.24x | 2.04x | 4.25x | 46.76x |
| `vec3 dot abort` | 8.28 ns | 211.04 ns | 889.27 ns | 259.62 ns | 199.70 ns | 6.96 us | 25.49x | 0.81x | 1.06x | 0.03x |
| `vec3 magnitude abort` | 16.74 ns | 4.00 us | 3.15 us | 5.35 us | 316.16 ns | 8.59 us | 238.90x | 0.75x | 12.65x | 0.47x |
| `vec3 normalize checked` | 26.14 ns | 8.08 us | 7.19 us | 5.90 us | 527.21 ns | 16.63 us | 308.90x | 1.37x | 15.32x | 0.49x |
| `vec3 normalize checked abort` | 30.66 ns | 8.24 us | 7.18 us | 5.81 us | 529.82 ns | 16.57 us | 268.66x | 1.42x | 15.55x | 0.50x |
| `vec3 div scalar checked` | 10.26 ns | 1.30 us | 1.60 us | 398.89 ns | - | - | 127.20x | 3.27x | - | - |
| `vec3 div scalar checked abort` | 18.49 ns | 1.34 us | 1.61 us | 397.84 ns | - | - | 72.53x | 3.37x | - | - |
| `vec3 add` | 6.63 ns | 940.59 ns | 1.71 us | 151.62 ns | 121.93 ns | 3.89 us | 141.78x | 6.20x | 7.71x | 0.24x |
| `vec3 add scalar` | 6.18 ns | 751.11 ns | 1.41 us | 151.23 ns | 128.86 ns | 3.85 us | 121.63x | 4.97x | 5.83x | 0.20x |
| `vec3 sub` | 6.69 ns | 1.05 us | 2.51 us | 162.80 ns | 135.22 ns | 7.34 us | 157.43x | 6.47x | 7.79x | 0.14x |
| `vec3 sub scalar` | 6.52 ns | 860.28 ns | 1.90 us | 147.70 ns | 120.32 ns | 7.36 us | 131.88x | 5.82x | 7.15x | 0.12x |
| `vec3 neg` | 3.69 ns | 142.72 ns | 140.59 ns | 57.39 ns | 50.42 ns | 3.13 us | 38.72x | 2.49x | 2.83x | 0.05x |
| `vec3 mul scalar` | 6.80 ns | 514.88 ns | 2.00 us | 163.42 ns | 119.33 ns | 4.33 us | 75.72x | 3.15x | 4.31x | 0.12x |
| `vec3 div scalar` | 9.79 ns | 1.39 us | 1.63 us | 389.34 ns | 170.92 ns | 7.56 us | 141.93x | 3.57x | 8.13x | 0.18x |
| `vec4 dot` | 9.51 ns | 254.41 ns | 638.55 ns | 429.06 ns | 327.36 ns | 9.83 us | 26.75x | 0.59x | 0.78x | 0.03x |
| `vec4 magnitude` | 16.41 ns | 3.66 us | 2.53 us | 5.59 us | 397.46 ns | 11.24 us | 222.86x | 0.65x | 9.20x | 0.33x |
| `vec4 normalize` | 36.23 ns | 8.50 us | 6.30 us | 6.16 us | 665.69 ns | 21.74 us | 234.64x | 1.38x | 12.77x | 0.39x |
| `vec4 add` | 7.13 ns | 1.20 us | 1.83 us | 206.85 ns | 170.93 ns | 5.25 us | 167.92x | 5.79x | 7.00x | 0.23x |
| `vec4 add scalar` | 6.82 ns | 970.63 ns | 1.70 us | 213.53 ns | 179.04 ns | 5.07 us | 142.31x | 4.55x | 5.42x | 0.19x |
| `vec4 sub` | 4.97 ns | 1.26 us | 2.20 us | 211.04 ns | 173.01 ns | 9.44 us | 253.98x | 5.98x | 7.30x | 0.13x |
| `vec4 sub scalar` | 4.47 ns | 1.06 us | 2.08 us | 198.56 ns | 167.74 ns | 9.29 us | 237.80x | 5.36x | 6.34x | 0.11x |
| `vec4 neg` | 4.82 ns | 188.44 ns | 188.77 ns | 79.07 ns | 63.27 ns | 3.93 us | 39.07x | 2.38x | 2.98x | 0.05x |
| `vec4 mul scalar` | 7.11 ns | 671.64 ns | 2.05 us | 212.27 ns | 155.53 ns | 5.57 us | 94.48x | 3.16x | 4.32x | 0.12x |
| `vec4 div scalar` | 14.14 ns | 1.69 us | 1.54 us | 535.54 ns | 222.62 ns | 9.86 us | 119.82x | 3.16x | 7.61x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.40 ns | 729.76 ns | 2.25 us | 945.33 ns | 842.90 ns | 22.07 us | 37.61x | 0.77x | 0.87x | 0.03x |
| `mat3 inverse` | 110.06 ns | 14.77 us | 8.67 us | 3.12 us | 2.48 us | 81.73 us | 134.22x | 4.73x | 5.94x | 0.18x |
| `mat3 mul mat3` | 67.34 ns | 3.43 us | 7.87 us | 2.77 us | 2.34 us | 60.70 us | 50.87x | 1.24x | 1.46x | 0.06x |
| `mat3 transform vec3` | 28.04 ns | 1.56 us | 5.02 us | 1.03 us | 901.83 ns | 19.96 us | 55.60x | 1.51x | 1.73x | 0.08x |
| `mat4 determinant` | 42.71 ns | 2.25 us | 1.58 us | 4.73 us | 4.11 us | 96.68 us | 52.67x | 0.48x | 0.55x | 0.02x |
| `mat4 inverse` | 150.58 ns | 24.66 us | 8.56 us | 11.01 us | 9.30 us | 340.97 us | 163.76x | 2.24x | 2.65x | 0.07x |
| `mat4 mul mat4` | 133.76 ns | 5.98 us | 5.06 us | 6.50 us | 5.40 us | 139.58 us | 44.73x | 0.92x | 1.11x | 0.04x |
| `mat4 transform vec4` | 45.36 ns | 2.57 us | 2.45 us | 1.91 us | 1.93 us | 35.27 us | 56.71x | 1.35x | 1.33x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 40.47 ns | 1.28 us | 4.18 us | 213.26 ns | 228.03 ns | 2.12 us | 31.74x | 6.02x | 5.63x | 0.61x |
| `mat3 zero` | 16.02 ns | 515.77 ns | 502.87 ns | 257.11 ns | 198.30 ns | 11.87 ns | 32.20x | 2.01x | 2.60x | 43.44x |
| `mat3 identity` | 9.47 ns | 623.76 ns | 628.06 ns | 312.16 ns | 231.88 ns | 139.58 ns | 65.90x | 2.00x | 2.69x | 4.47x |
| `mat3 transpose` | 8.82 ns | 669.93 ns | 671.81 ns | 236.75 ns | 201.69 ns | 117.93 ns | 75.93x | 2.83x | 3.32x | 5.68x |
| `mat3 reciprocal` | 91.84 ns | 14.71 us | 24.81 us | 2.89 us | 2.25 us | 81.91 us | 160.19x | 5.09x | 6.55x | 0.18x |
| `mat3 reciprocal checked` | 93.72 ns | 14.47 us | 24.31 us | 2.91 us | 2.32 us | 80.68 us | 154.36x | 4.96x | 6.24x | 0.18x |
| `mat3 inverse checked` | 93.24 ns | 14.76 us | 24.96 us | 2.93 us | 2.26 us | 81.88 us | 158.27x | 5.04x | 6.52x | 0.18x |
| `mat3 inverse checked abort` | 116.08 ns | 14.79 us | 24.72 us | 2.89 us | 2.27 us | 81.01 us | 127.46x | 5.11x | 6.53x | 0.18x |
| `mat3 powi` | 107.14 ns | 4.45 us | 38.47 us | 6.66 us | 6.12 us | 144.88 us | 41.50x | 0.67x | 0.73x | 0.03x |
| `mat3 powi checked` | 106.71 ns | 4.47 us | 38.64 us | 6.67 us | 6.02 us | 148.38 us | 41.89x | 0.67x | 0.74x | 0.03x |
| `mat3 powi checked abort` | 107.53 ns | 4.37 us | 38.92 us | 6.67 us | 6.13 us | 147.16 us | 40.66x | 0.66x | 0.71x | 0.03x |
| `mat3 div scalar checked` | 24.40 ns | 4.20 us | 5.62 us | 1.48 us | 797.83 ns | 21.82 us | 171.97x | 2.84x | 5.26x | 0.19x |
| `mat3 div scalar checked abort` | 29.27 ns | 4.27 us | 5.60 us | 1.44 us | 783.33 ns | 21.79 us | 145.98x | 2.97x | 5.46x | 0.20x |
| `mat3 div matrix checked` | 143.88 ns | 17.71 us | 44.05 us | 5.25 us | 4.49 us | 156.47 us | 123.07x | 3.37x | 3.95x | 0.11x |
| `mat3 div matrix checked abort` | 156.04 ns | 17.87 us | 43.18 us | 5.26 us | 4.47 us | 156.77 us | 114.53x | 3.40x | 4.00x | 0.11x |
| `mat3 add` | 14.48 ns | 3.10 us | 5.18 us | 501.60 ns | 472.02 ns | 11.61 us | 214.06x | 6.18x | 6.57x | 0.27x |
| `mat3 add scalar` | 11.89 ns | 2.38 us | 4.75 us | 746.19 ns | 679.91 ns | 12.27 us | 200.07x | 3.19x | 3.50x | 0.19x |
| `mat3 sub` | 12.94 ns | 3.39 us | 7.06 us | 537.73 ns | 511.49 ns | 21.38 us | 262.40x | 6.31x | 6.64x | 0.16x |
| `mat3 sub scalar` | 10.56 ns | 2.63 us | 6.52 us | 793.45 ns | 692.34 ns | 21.24 us | 249.01x | 3.31x | 3.80x | 0.12x |
| `mat3 neg` | 10.11 ns | 592.91 ns | 629.77 ns | 458.21 ns | 448.38 ns | 8.43 us | 58.62x | 1.29x | 1.32x | 0.07x |
| `mat3 mul scalar` | 13.23 ns | 1.72 us | 5.73 us | 761.03 ns | 642.00 ns | 12.00 us | 130.32x | 2.27x | 2.69x | 0.14x |
| `mat3 div scalar` | 24.16 ns | 4.21 us | 5.50 us | 1.45 us | 798.78 ns | 21.87 us | 174.06x | 2.90x | 5.26x | 0.19x |
| `mat3 div matrix` | 141.32 ns | 17.68 us | 43.52 us | 5.23 us | 4.35 us | 156.72 us | 125.09x | 3.38x | 4.06x | 0.11x |
| `mat3 bitxor` | 105.99 ns | 4.34 us | 37.61 us | 6.67 us | 6.13 us | 144.94 us | 40.99x | 0.65x | 0.71x | 0.03x |
| `mat4 zero` | 11.24 ns | 893.47 ns | 918.88 ns | 512.96 ns | 350.62 ns | 14.54 ns | 79.50x | 1.74x | 2.55x | 61.45x |
| `mat4 identity` | 10.76 ns | 1.09 us | 1.09 us | 617.78 ns | 418.47 ns | 236.38 ns | 101.17x | 1.76x | 2.60x | 4.60x |
| `mat4 transpose` | 9.07 ns | 1.05 us | 1.04 us | 472.73 ns | 370.97 ns | 181.18 ns | 115.84x | 2.22x | 2.83x | 5.80x |
| `mat4 reciprocal` | 159.62 ns | 24.87 us | 41.31 us | 10.59 us | 8.65 us | 339.64 us | 155.82x | 2.35x | 2.87x | 0.07x |
| `mat4 reciprocal checked` | 171.19 ns | 24.80 us | 41.83 us | 10.76 us | 8.84 us | 336.98 us | 144.85x | 2.30x | 2.80x | 0.07x |
| `mat4 powi` | 247.90 ns | 8.69 us | 52.02 us | 15.29 us | 14.06 us | 349.87 us | 35.06x | 0.57x | 0.62x | 0.02x |
| `mat4 powi checked` | 249.34 ns | 8.59 us | 52.13 us | 14.99 us | 13.93 us | 357.91 us | 34.46x | 0.57x | 0.62x | 0.02x |
| `mat4 add` | 50.32 ns | 5.11 us | 5.63 us | 912.65 ns | 885.67 ns | 19.18 us | 101.56x | 5.60x | 5.77x | 0.27x |
| `mat4 add scalar` | 23.76 ns | 4.01 us | 6.59 us | 1.40 us | 1.17 us | 20.23 us | 168.82x | 2.87x | 3.42x | 0.20x |
| `mat4 sub` | 39.01 ns | 5.21 us | 6.78 us | 955.44 ns | 938.36 ns | 35.42 us | 133.65x | 5.46x | 5.56x | 0.15x |
| `mat4 sub scalar` | 18.76 ns | 4.24 us | 7.36 us | 1.42 us | 1.14 us | 37.18 us | 225.78x | 2.98x | 3.72x | 0.11x |
| `mat4 neg` | 18.89 ns | 963.12 ns | 949.61 ns | 900.23 ns | 731.48 ns | 14.05 us | 50.98x | 1.07x | 1.32x | 0.07x |
| `mat4 mul scalar` | 25.84 ns | 2.96 us | 7.39 us | 1.40 us | 1.10 us | 19.75 us | 114.41x | 2.11x | 2.68x | 0.15x |
| `mat4 div scalar` | 32.95 ns | 7.60 us | 7.48 us | 2.59 us | 1.36 us | 36.64 us | 230.72x | 2.93x | 5.59x | 0.21x |
| `mat4 div matrix` | 196.75 ns | 28.15 us | 63.85 us | 16.34 us | 13.92 us | 529.53 us | 143.05x | 1.72x | 2.02x | 0.05x |
| `mat4 bitxor` | 250.98 ns | 8.60 us | 51.25 us | 15.30 us | 14.35 us | 351.93 us | 34.25x | 0.56x | 0.60x | 0.02x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.60 ns | 172.59 ns | 281.06 ns | - | - | 1.26 us | 47.93x | - | - | 0.14x |
| `scalar add ref_owned` | 11.92 ns | 176.97 ns | 275.50 ns | - | - | 1.26 us | 14.85x | - | - | 0.14x |
| `scalar add refs` | 5.31 ns | 153.43 ns | 256.89 ns | - | - | 1.25 us | 28.91x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 8.82 ns | 186.88 ns | 289.27 ns | - | - | - | 21.20x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.31 ns | 182.14 ns | 292.30 ns | - | - | - | 34.29x | - | - | - |
| `scalar sub owned_ref` | 3.86 ns | 216.61 ns | 454.48 ns | - | - | 2.38 us | 56.09x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.10 ns | 222.88 ns | 452.74 ns | - | - | 2.38 us | 18.41x | - | - | 0.09x |
| `scalar sub refs` | 5.54 ns | 193.91 ns | 435.82 ns | - | - | 2.38 us | 35.03x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 8.95 ns | 223.44 ns | 467.30 ns | - | - | - | 24.97x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.53 ns | 224.95 ns | 467.21 ns | - | - | - | 40.71x | - | - | - |
| `scalar mul owned_ref` | 4.45 ns | 95.43 ns | 363.57 ns | - | - | 1.53 us | 21.43x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.31 ns | 104.13 ns | 370.48 ns | - | - | 1.52 us | 7.82x | - | - | 0.07x |
| `scalar mul refs` | 6.26 ns | 80.73 ns | 349.56 ns | - | - | 1.52 us | 12.89x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.64 ns | 110.01 ns | 378.20 ns | - | - | - | 11.41x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.14 ns | 106.67 ns | 383.89 ns | - | - | - | 17.38x | - | - | - |
| `scalar div owned_ref` | 5.87 ns | 255.37 ns | 310.33 ns | - | - | 2.51 us | 43.47x | - | - | 0.10x |
| `scalar div ref_owned` | 16.71 ns | 273.30 ns | 324.55 ns | - | - | 2.51 us | 16.35x | - | - | 0.11x |
| `scalar div refs` | 6.70 ns | 246.03 ns | 299.01 ns | - | - | 2.49 us | 36.75x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.05 ns | 269.16 ns | 326.17 ns | - | - | - | 17.88x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.11 ns | 268.69 ns | 326.08 ns | - | - | - | 17.78x | - | - | - |
| `vec3 add refs` | 6.06 ns | 601.47 ns | 1.37 us | - | - | 3.82 us | 99.18x | - | - | 0.16x |
| `vec3 sub refs` | 6.14 ns | 709.38 ns | 2.13 us | - | - | 7.12 us | 115.48x | - | - | 0.10x |
| `vec3 neg ref` | 3.25 ns | 160.33 ns | 156.96 ns | - | - | 3.07 us | 49.39x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.45 ns | 728.53 ns | 1.41 us | - | - | 3.70 us | 112.87x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.50 ns | 827.19 ns | 1.89 us | - | - | 6.88 us | 127.35x | - | - | 0.12x |
| `vec3 mul_scalar_ref` | 6.92 ns | 497.12 ns | 2.02 us | - | - | 4.38 us | 71.88x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.02 ns | 1.26 us | 1.51 us | - | - | 7.54 us | 156.50x | - | - | 0.17x |
| `vec4 add refs` | 6.29 ns | 755.76 ns | 1.38 us | - | - | 5.20 us | 120.08x | - | - | 0.15x |
| `vec4 sub refs` | 3.06 ns | 846.88 ns | 1.78 us | - | - | 9.52 us | 277.13x | - | - | 0.09x |
| `vec4 neg ref` | 4.20 ns | 213.79 ns | 215.01 ns | - | - | 3.99 us | 50.89x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 7.01 ns | 958.16 ns | 1.68 us | - | - | 5.14 us | 136.75x | - | - | 0.19x |
| `vec4 sub_scalar_ref` | 4.19 ns | 1.07 us | 2.02 us | - | - | 9.27 us | 254.65x | - | - | 0.12x |
| `vec4 mul_scalar_ref` | 7.44 ns | 648.37 ns | 2.05 us | - | - | 5.72 us | 87.16x | - | - | 0.11x |
| `vec4 div_scalar_ref` | 12.16 ns | 1.68 us | 1.50 us | - | - | 10.07 us | 138.19x | - | - | 0.17x |
| `mat3 add refs` | 11.00 ns | 1.72 us | 3.82 us | - | - | 11.53 us | 156.33x | - | - | 0.15x |
| `mat3 sub refs` | 10.40 ns | 1.97 us | 5.57 us | - | - | 21.09 us | 189.21x | - | - | 0.09x |
| `mat3 mul refs` | 53.68 ns | 2.28 us | 14.46 us | - | - | 60.61 us | 42.46x | - | - | 0.04x |
| `mat3 div refs` | 133.00 ns | 17.43 us | 41.75 us | - | - | 157.69 us | 131.09x | - | - | 0.11x |
| `mat3 neg ref` | 10.00 ns | 484.12 ns | 497.30 ns | - | - | 8.44 us | 48.40x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 44.56 ns | 2.29 us | 4.44 us | - | - | 11.95 us | 51.40x | - | - | 0.19x |
| `mat3 sub_scalar_ref` | 43.81 ns | 2.57 us | 6.42 us | - | - | 21.20 us | 58.57x | - | - | 0.12x |
| `mat3 mul_scalar_ref` | 47.40 ns | 1.67 us | 5.50 us | - | - | 11.90 us | 35.17x | - | - | 0.14x |
| `mat3 div_scalar_ref` | 22.16 ns | 4.20 us | 5.47 us | - | - | 21.53 us | 189.32x | - | - | 0.19x |
| `mat4 add refs` | 16.53 ns | 2.68 us | 3.42 us | - | - | 19.13 us | 161.96x | - | - | 0.14x |
| `mat4 sub refs` | 16.20 ns | 2.96 us | 4.61 us | - | - | 35.73 us | 182.54x | - | - | 0.08x |
| `mat4 mul refs` | 121.61 ns | 4.30 us | 19.15 us | - | - | 141.15 us | 35.36x | - | - | 0.03x |
| `mat4 div refs` | 185.57 ns | 26.88 us | 61.08 us | - | - | 528.01 us | 144.86x | - | - | 0.05x |
| `mat4 neg ref` | 12.02 ns | 738.95 ns | 699.92 ns | - | - | 13.84 us | 61.48x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 47.77 ns | 3.93 us | 6.48 us | - | - | 20.48 us | 82.24x | - | - | 0.19x |
| `mat4 sub_scalar_ref` | 36.23 ns | 4.16 us | 7.40 us | - | - | 36.89 us | 114.87x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 52.35 ns | 2.82 us | 7.16 us | - | - | 19.99 us | 53.86x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 27.15 ns | 7.30 us | 7.41 us | - | - | 37.34 us | 268.90x | - | - | 0.20x |
| `mat3 transform_vec refs` | 26.01 ns | 912.97 ns | 4.91 us | - | - | 19.75 us | 35.11x | - | - | 0.05x |
| `mat4 transform_vec refs` | 42.72 ns | 1.51 us | 5.80 us | - | - | 34.19 us | 35.30x | - | - | 0.04x |
| `complex add refs` | 7.63 ns | 317.39 ns | 684.55 ns | - | - | 2.52 us | 41.62x | - | - | 0.13x |
| `complex sub refs` | 7.94 ns | 365.45 ns | 940.02 ns | - | - | 4.71 us | 46.05x | - | - | 0.08x |
| `complex mul refs` | 7.81 ns | 751.24 ns | 3.31 us | - | - | 10.16 us | 96.23x | - | - | 0.07x |
| `complex div refs` | 16.20 ns | 2.20 us | 4.45 us | - | - | 21.37 us | 135.59x | - | - | 0.10x |
| `complex neg ref` | 2.31 ns | 71.71 ns | 71.32 ns | - | - | 2.11 us | 31.08x | - | - | 0.03x |
| `complex div_real_ref` | 7.12 ns | 627.90 ns | 585.53 ns | - | - | 5.15 us | 88.20x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 11.76 us |
| `astro sin 160` | 13.68 us |
| `astro sin 192` | 13.75 us |
| `astro sin 256` | 15.53 us |
