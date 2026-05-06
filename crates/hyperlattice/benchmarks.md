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
| `sin 0.1` | 11.10 ns | 170.54 ns | 169.06 ns | 11.26 us | 765.13 ns | 1.86 us | 15.36x | 0.02x | 0.22x | 0.09x |
| `cos 0.1` | 11.92 ns | 174.71 ns | 170.29 ns | 11.06 us | 521.31 ns | 1.71 us | 14.66x | 0.02x | 0.34x | 0.10x |
| `sin 1.23456789` | 12.08 ns | 1.75 us | 1.98 us | 13.06 us | 829.79 ns | 1.82 us | 145.27x | 0.13x | 2.11x | 0.97x |
| `cos 1.23456789` | 12.16 ns | 2.09 us | 2.27 us | 11.04 us | 593.16 ns | 1.67 us | 171.77x | 0.19x | 3.52x | 1.25x |
| `sin 1e6` | 13.31 ns | 5.92 us | 5.97 us | 16.05 us | 1.09 us | 2.11 us | 444.49x | 0.37x | 5.41x | 2.80x |
| `cos 1e6` | 12.89 ns | 5.83 us | 5.97 us | 13.76 us | 828.57 ns | 1.95 us | 452.71x | 0.42x | 7.04x | 2.99x |
| `sin 1e30` | 66.66 ns | 7.96 us | 8.39 us | 18.52 us | 2.92 us | 3.67 us | 119.39x | 0.43x | 2.73x | 2.17x |
| `cos 1e30` | 70.33 ns | 8.09 us | 8.21 us | 15.67 us | 1.29 us | 3.15 us | 115.08x | 0.52x | 6.27x | 2.57x |
| `sin pi_7` | 11.86 ns | 176.09 ns | 493.37 ns | 12.16 us | 746.91 ns | 1.93 us | 14.85x | 0.01x | 0.24x | 0.09x |
| `cos pi_7` | 11.92 ns | 177.90 ns | 904.44 ns | 11.29 us | 548.50 ns | 1.73 us | 14.92x | 0.02x | 0.32x | 0.10x |
| `sin 1000pi_eps` | 12.23 ns | 5.96 us | 3.50 us | 16.27 us | 2.37 us | 2.91 us | 487.59x | 0.37x | 2.52x | 2.05x |
| `cos 1000pi_eps` | 12.74 ns | 5.82 us | 3.49 us | 14.23 us | 590.48 ns | 1.70 us | 456.34x | 0.41x | 9.85x | 3.42x |
| `asin 0.5` | 11.48 ns | 526.72 ns | 517.07 ns | 51.14 us | 2.96 us | 13.38 us | 45.88x | 0.01x | 0.18x | 0.04x |
| `acos 0.5` | 11.95 ns | 1.16 us | 1.17 us | 60.40 us | 2.95 us | 13.39 us | 96.89x | 0.02x | 0.39x | 0.09x |
| `atanh 0.5` | 14.82 ns | 1.48 us | 1.47 us | 36.02 us | 1.75 us | 12.83 us | 99.60x | 0.04x | 0.84x | 0.12x |
| `asin neg_0.999999` | 14.26 ns | 5.68 us | 4.66 us | 13.83 us | 2.57 us | 13.42 us | 398.75x | 0.41x | 2.21x | 0.42x |
| `acos neg_0.999999` | 15.39 ns | 6.78 us | 5.84 us | 18.37 us | 2.68 us | 13.60 us | 440.92x | 0.37x | 2.53x | 0.50x |
| `atanh neg_0.999999` | 14.70 ns | 4.33 us | 3.66 us | 36.31 us | 1.56 us | 13.39 us | 294.29x | 0.12x | 2.77x | 0.32x |
| `asin 0.999999` | 14.12 ns | 5.81 us | 4.76 us | 13.62 us | 2.50 us | 13.12 us | 411.50x | 0.43x | 2.32x | 0.44x |
| `acos 0.999999` | 14.51 ns | 5.42 us | 4.99 us | 18.38 us | 2.80 us | 13.31 us | 373.73x | 0.30x | 1.94x | 0.41x |
| `atanh 0.999999` | 14.85 ns | 4.08 us | 3.69 us | 31.51 us | 1.64 us | 12.78 us | 274.85x | 0.13x | 2.49x | 0.32x |
| `asin 1e-12` | 9.85 ns | 496.75 ns | 1.14 us | 8.05 us | 1.40 us | 15.56 us | 50.41x | 0.06x | 0.35x | 0.03x |
| `acos 1e-12` | 10.45 ns | 851.57 ns | 1.92 us | 9.88 us | 1.44 us | 15.25 us | 81.52x | 0.09x | 0.59x | 0.06x |
| `atanh 1e-12` | 9.89 ns | 489.10 ns | 942.29 ns | 37.58 us | 171.26 ns | 20.58 us | 49.46x | 0.01x | 2.86x | 0.02x |
| `atan 0.5` | 15.30 ns | 427.28 ns | 431.13 ns | 35.39 us | 2.79 us | 18.29 us | 27.93x | 0.01x | 0.15x | 0.02x |
| `asinh 0.5` | 27.06 ns | 1.62 us | 1.68 us | 40.66 us | 1.61 us | 7.48 us | 59.90x | 0.04x | 1.01x | 0.22x |
| `atan neg_1e-12` | 14.66 ns | 436.99 ns | 680.27 ns | 1.60 us | 1.14 us | 15.62 us | 29.80x | 0.27x | 0.38x | 0.03x |
| `asinh neg_1e-12` | 15.93 ns | 5.61 us | 4.55 us | 42.90 us | 8.55 us | 12.33 us | 351.87x | 0.13x | 0.66x | 0.45x |
| `atan 1e6` | 15.66 ns | 604.62 ns | 595.38 ns | 2.87 us | 1.45 us | 18.12 us | 38.61x | 0.21x | 0.42x | 0.03x |
| `asinh 1e6` | 27.12 ns | 3.21 us | 3.28 us | 39.19 us | 1.68 us | 7.20 us | 118.51x | 0.08x | 1.92x | 0.45x |
| `atan neg_1e6` | 16.83 ns | 683.64 ns | 741.57 ns | 2.82 us | 1.48 us | 17.97 us | 40.63x | 0.24x | 0.46x | 0.04x |
| `asinh neg_1e6` | 27.11 ns | 3.29 us | 3.34 us | 39.80 us | 1.65 us | 7.04 us | 121.34x | 0.08x | 2.00x | 0.47x |
| `acosh 9` | 12.81 ns | 2.83 us | 2.89 us | 42.27 us | 1.63 us | 9.75 us | 221.04x | 0.07x | 1.74x | 0.29x |
| `acosh 1_plus_1e-12` | 11.71 ns | 3.73 us | 5.11 us | 41.88 us | 8.36 us | 11.39 us | 318.09x | 0.09x | 0.45x | 0.33x |
| `acosh 1e6` | 12.84 ns | 3.66 us | 3.57 us | 37.65 us | 1.62 us | 10.28 us | 284.89x | 0.10x | 2.25x | 0.36x |
| `acosh e` | 12.86 ns | 4.08 us | 3.84 us | 40.51 us | 1.63 us | 10.27 us | 317.66x | 0.10x | 2.51x | 0.40x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.49 ns | 57.49 ns | 54.74 ns | 25.15 ns | 15.65 ns | 0.95 ns | 117.07x | 2.29x | 3.67x | 60.58x |
| `one` | 0.49 ns | 66.60 ns | 63.30 ns | 42.58 ns | 31.15 ns | 30.26 ns | 136.93x | 1.56x | 2.14x | 2.20x |
| `e` | 0.49 ns | 75.10 ns | 71.54 ns | 54.92 ns | 1.07 us | 226.26 ns | 152.17x | 1.37x | 0.07x | 0.33x |
| `pi` | 0.48 ns | 60.21 ns | 57.62 ns | 43.51 ns | 48.75 ns | 227.35 ns | 124.55x | 1.38x | 1.24x | 0.26x |
| `tau` | 0.48 ns | 197.59 ns | 194.86 ns | 120.49 ns | 100.29 ns | 1.85 us | 411.72x | 1.64x | 1.97x | 0.11x |
| `add` | 5.38 ns | 297.41 ns | 409.58 ns | 53.40 ns | 42.35 ns | 1.40 us | 55.27x | 5.57x | 7.02x | 0.21x |
| `sub` | 5.30 ns | 297.93 ns | 465.65 ns | 59.25 ns | 45.89 ns | 2.53 us | 56.22x | 5.03x | 6.49x | 0.12x |
| `neg` | 5.16 ns | 60.69 ns | 57.77 ns | 23.69 ns | 22.06 ns | 1.07 us | 11.76x | 2.56x | 2.75x | 0.06x |
| `mul` | 5.59 ns | 219.89 ns | 469.68 ns | 58.92 ns | 51.67 ns | 1.54 us | 39.33x | 3.73x | 4.26x | 0.14x |
| `div` | 9.19 ns | 405.13 ns | 441.08 ns | 139.73 ns | 64.72 ns | 2.55 us | 44.07x | 2.90x | 6.26x | 0.16x |
| `reciprocal` | 8.96 ns | 149.71 ns | 137.27 ns | 164.17 ns | 60.46 ns | 1.53 us | 16.71x | 0.91x | 2.48x | 0.10x |
| `reciprocal checked` | 9.30 ns | 126.96 ns | 121.84 ns | 163.82 ns | 61.18 ns | 1.53 us | 13.65x | 0.77x | 2.08x | 0.08x |
| `reciprocal checked abort` | 24.65 ns | 130.87 ns | 127.47 ns | 165.88 ns | 61.91 ns | 1.56 us | 5.31x | 0.79x | 2.11x | 0.08x |
| `pow` | 20.48 ns | 10.66 us | 10.80 us | 54.36 us | 2.90 us | 2.42 us | 520.80x | 0.20x | 3.67x | 4.40x |
| `powi` | 6.25 ns | 546.01 ns | 2.92 us | 292.24 ns | 85.82 ns | 1.73 us | 87.43x | 1.87x | 6.36x | 0.32x |
| `exp` | 20.51 ns | 1.75 us | 3.01 us | 13.94 us | 947.89 ns | 2.04 us | 85.45x | 0.13x | 1.85x | 0.86x |
| `ln` | 11.03 ns | 1.56 us | 1.59 us | 30.02 us | 1.31 us | 1.85 us | 141.05x | 0.05x | 1.19x | 0.84x |
| `log10` | 13.55 ns | 2.69 us | 2.44 us | 35.62 us | 2.76 us | 6.89 us | 198.89x | 0.08x | 0.98x | 0.39x |
| `log10 abort` | 17.35 ns | 2.82 us | 2.40 us | 35.39 us | 2.74 us | 6.75 us | 162.29x | 0.08x | 1.03x | 0.42x |
| `sqrt` | 8.53 ns | 1.62 us | 1.74 us | 5.23 us | 98.31 ns | 1.47 us | 190.08x | 0.31x | 16.49x | 1.10x |
| `sin` | 15.44 ns | 3.09 us | 3.52 us | 14.32 us | 1.28 us | 2.32 us | 200.24x | 0.22x | 2.41x | 1.33x |
| `cos` | 19.11 ns | 3.03 us | 3.57 us | 12.30 us | 649.67 ns | 1.76 us | 158.54x | 0.25x | 4.66x | 1.73x |
| `tan` | 25.28 ns | 2.83 us | 3.27 us | 29.89 us | 1.64 us | 6.73 us | 111.76x | 0.09x | 1.72x | 0.42x |
| `sinh` | 18.37 ns | 4.41 us | 4.50 us | 3.38 us | 1.16 us | 11.16 us | 239.87x | 1.30x | 3.79x | 0.39x |
| `cosh` | 18.56 ns | 4.36 us | 4.56 us | 7.99 us | 1.09 us | 9.92 us | 234.66x | 0.55x | 3.99x | 0.44x |
| `tanh` | 23.41 ns | 5.73 us | 5.96 us | 3.41 us | 1.23 us | 23.28 us | 244.53x | 1.68x | 4.65x | 0.25x |
| `asin` | 13.74 ns | 3.22 us | 4.60 us | 21.58 us | 2.42 us | 14.04 us | 234.24x | 0.15x | 1.33x | 0.23x |
| `asin abort` | 17.33 ns | 3.16 us | 4.77 us | 21.19 us | 2.48 us | 14.03 us | 182.49x | 0.15x | 1.28x | 0.23x |
| `acos` | 14.36 ns | 3.53 us | 5.89 us | 26.37 us | 2.60 us | 14.03 us | 245.52x | 0.13x | 1.35x | 0.25x |
| `acos abort` | 17.84 ns | 3.48 us | 5.79 us | 26.40 us | 2.69 us | 13.92 us | 194.85x | 0.13x | 1.29x | 0.25x |
| `atan` | 15.02 ns | 597.66 ns | 1.29 us | 18.83 us | 2.34 us | 19.46 us | 39.80x | 0.03x | 0.26x | 0.03x |
| `atan abort` | 22.11 ns | 578.18 ns | 1.29 us | 18.55 us | 2.33 us | 19.54 us | 26.15x | 0.03x | 0.25x | 0.03x |
| `asinh` | 34.31 ns | 3.64 us | 5.56 us | 39.07 us | 1.72 us | 7.67 us | 106.07x | 0.09x | 2.11x | 0.47x |
| `asinh abort` | 31.59 ns | 3.69 us | 5.44 us | 39.41 us | 1.64 us | 7.77 us | 116.69x | 0.09x | 2.25x | 0.47x |
| `acosh` | 12.66 ns | 3.75 us | 5.34 us | 41.29 us | 3.34 us | 10.50 us | 296.28x | 0.09x | 1.12x | 0.36x |
| `acosh abort` | 17.19 ns | 3.80 us | 5.34 us | 42.44 us | 3.32 us | 10.76 us | 220.95x | 0.09x | 1.14x | 0.35x |
| `atanh` | 13.80 ns | 2.79 us | 3.28 us | 34.90 us | 1.26 us | 15.38 us | 201.83x | 0.08x | 2.22x | 0.18x |
| `atanh abort` | 17.33 ns | 2.58 us | 3.33 us | 34.63 us | 1.25 us | 14.88 us | 148.76x | 0.07x | 2.06x | 0.17x |
| `zero status` | 1.27 ns | 1.96 ns | 2.72 ns | 0.98 ns | 6.79 ns | 8.24 ns | 1.54x | 2.00x | 0.29x | 0.24x |
| `zero status abort` | 1.42 ns | 3.37 ns | 3.35 ns | 1.09 ns | 7.07 ns | 8.25 ns | 2.37x | 3.10x | 0.48x | 0.41x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 116.86 ns | 115.78 ns | 48.97 ns | - | 1.90 ns | 124.20x | 2.39x | - | 61.55x |
| `one` | 5.45 ns | 120.69 ns | 126.89 ns | 66.96 ns | - | 30.60 ns | 22.16x | 1.80x | - | 3.94x |
| `i` | 0.97 ns | 120.77 ns | 121.59 ns | 66.89 ns | - | 30.00 ns | 124.55x | 1.81x | - | 4.03x |
| `free i` | 0.96 ns | 124.41 ns | 121.81 ns | 66.00 ns | - | 31.28 ns | 129.58x | 1.89x | - | 3.98x |
| `conjugate` | 2.18 ns | 131.89 ns | 125.46 ns | 41.81 ns | - | 1.14 us | 60.39x | 3.15x | - | 0.12x |
| `norm squared` | 6.06 ns | 405.62 ns | 1.51 us | 153.29 ns | - | 4.43 us | 66.91x | 2.65x | - | 0.09x |
| `reciprocal` | 16.00 ns | 1.96 us | 3.06 us | 471.24 ns | - | 10.90 us | 122.30x | 4.15x | - | 0.18x |
| `reciprocal checked` | 14.91 ns | 1.96 us | 3.09 us | 470.97 ns | - | 11.00 us | 131.31x | 4.16x | - | 0.18x |
| `powi` | 18.12 ns | 2.98 us | 14.70 us | 1.44 us | - | 43.65 us | 164.20x | 2.06x | - | 0.07x |
| `powi checked` | 17.78 ns | 2.70 us | 14.42 us | 1.45 us | - | 44.56 us | 152.01x | 1.87x | - | 0.06x |
| `div checked` | 21.20 ns | 3.14 us | 6.88 us | 813.00 ns | - | 21.84 us | 148.17x | 3.86x | - | 0.14x |
| `div real checked` | 9.51 ns | 782.30 ns | 775.64 ns | 268.59 ns | - | 5.44 us | 82.29x | 2.91x | - | 0.14x |
| `from scalar` | 1.42 ns | 119.39 ns | 114.06 ns | 46.44 ns | - | 11.52 ns | 83.88x | 2.57x | - | 10.36x |
| `add` | 6.27 ns | 622.51 ns | 1.01 us | 105.38 ns | - | 2.58 us | 99.25x | 5.91x | - | 0.24x |
| `sub` | 6.33 ns | 627.38 ns | 1.03 us | 127.14 ns | - | 4.95 us | 99.17x | 4.93x | - | 0.13x |
| `neg` | 2.66 ns | 136.01 ns | 131.89 ns | 42.33 ns | - | 2.14 us | 51.05x | 3.21x | - | 0.06x |
| `mul` | 7.99 ns | 1.13 us | 3.51 us | 308.86 ns | - | 10.32 us | 141.09x | 3.65x | - | 0.11x |
| `div` | 19.05 ns | 3.25 us | 6.93 us | 782.09 ns | - | 22.12 us | 170.69x | 4.16x | - | 0.15x |
| `div real` | 10.28 ns | 799.74 ns | 755.57 ns | 266.28 ns | - | 5.26 us | 77.83x | 3.00x | - | 0.15x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.98 ns | 842.19 ns | 6.01 us | 312.36 ns | 271.45 ns | 7.39 us | 120.71x | 2.70x | 3.10x | 0.11x |
| `vec3 magnitude` | 9.68 ns | 4.81 us | 9.69 us | 5.64 us | 366.54 ns | 9.21 us | 496.44x | 0.85x | 13.12x | 0.52x |
| `vec3 normalize` | 26.31 ns | 9.17 us | 11.60 us | 6.33 us | 624.00 ns | 17.08 us | 348.51x | 1.45x | 14.69x | 0.54x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.10 ns | 310.81 ns | 1.62 us | 71.45 ns | 61.11 ns | 743.31 ns | 100.37x | 4.35x | 5.09x | 0.42x |
| `vec3 zero` | 1.41 ns | 259.85 ns | 253.06 ns | 67.09 ns | 32.25 ns | 2.85 ns | 184.07x | 3.87x | 8.06x | 91.15x |
| `vec3 dot abort` | 27.87 ns | 1.33 us | 4.69 us | 271.09 ns | 209.57 ns | 7.17 us | 47.74x | 4.91x | 6.35x | 0.19x |
| `vec3 magnitude abort` | 40.46 ns | 5.44 us | 7.01 us | 5.77 us | 338.89 ns | 8.91 us | 134.35x | 0.94x | 16.04x | 0.61x |
| `vec3 normalize checked` | 26.02 ns | 8.92 us | 8.88 us | 6.04 us | 573.36 ns | 18.20 us | 342.90x | 1.48x | 15.56x | 0.49x |
| `vec3 normalize checked abort` | 57.35 ns | 9.55 us | 9.53 us | 6.04 us | 572.73 ns | 17.44 us | 166.58x | 1.58x | 16.68x | 0.55x |
| `vec3 div scalar checked` | 17.65 ns | 1.49 us | 1.75 us | 399.88 ns | - | - | 84.25x | 3.72x | - | - |
| `vec3 div scalar checked abort` | 19.02 ns | 1.50 us | 1.82 us | 405.26 ns | - | - | 79.10x | 3.71x | - | - |
| `vec3 add` | 6.75 ns | 1.25 us | 2.10 us | 154.34 ns | 128.14 ns | 4.11 us | 185.27x | 8.10x | 9.75x | 0.30x |
| `vec3 add scalar` | 6.45 ns | 966.71 ns | 1.67 us | 154.55 ns | 139.24 ns | 3.89 us | 149.81x | 6.26x | 6.94x | 0.25x |
| `vec3 sub` | 6.82 ns | 1.24 us | 2.13 us | 177.96 ns | 138.50 ns | 7.64 us | 182.08x | 6.98x | 8.97x | 0.16x |
| `vec3 sub scalar` | 6.42 ns | 1.04 us | 1.50 us | 146.33 ns | 130.29 ns | 7.54 us | 161.35x | 7.08x | 7.95x | 0.14x |
| `vec3 neg` | 3.79 ns | 240.18 ns | 242.18 ns | 61.51 ns | 52.94 ns | 3.22 us | 63.37x | 3.90x | 4.54x | 0.07x |
| `vec3 mul scalar` | 7.05 ns | 705.33 ns | 2.22 us | 175.54 ns | 128.61 ns | 4.54 us | 100.01x | 4.02x | 5.48x | 0.16x |
| `vec3 div scalar` | 10.51 ns | 1.54 us | 1.76 us | 401.60 ns | 181.09 ns | 7.84 us | 146.94x | 3.85x | 8.53x | 0.20x |
| `vec4 dot` | 7.48 ns | 1.03 us | 3.50 us | 459.12 ns | 332.85 ns | 9.81 us | 137.57x | 2.24x | 3.09x | 0.10x |
| `vec4 magnitude` | 13.63 ns | 4.58 us | 5.61 us | 5.80 us | 433.49 ns | 11.47 us | 335.67x | 0.79x | 10.56x | 0.40x |
| `vec4 normalize` | 39.73 ns | 9.42 us | 7.65 us | 6.45 us | 756.21 ns | 22.36 us | 237.01x | 1.46x | 12.45x | 0.42x |
| `vec4 add` | 7.69 ns | 1.59 us | 2.21 us | 203.31 ns | 177.86 ns | 5.41 us | 206.61x | 7.81x | 8.93x | 0.29x |
| `vec4 add scalar` | 7.36 ns | 1.28 us | 2.06 us | 233.67 ns | 180.86 ns | 5.33 us | 173.95x | 5.48x | 7.08x | 0.24x |
| `vec4 sub` | 5.15 ns | 1.54 us | 2.09 us | 220.30 ns | 179.32 ns | 9.90 us | 298.94x | 6.99x | 8.58x | 0.16x |
| `vec4 sub scalar` | 4.67 ns | 1.30 us | 1.88 us | 209.11 ns | 174.23 ns | 9.59 us | 278.81x | 6.23x | 7.47x | 0.14x |
| `vec4 neg` | 5.04 ns | 327.88 ns | 323.42 ns | 80.44 ns | 67.72 ns | 4.17 us | 65.08x | 4.08x | 4.84x | 0.08x |
| `vec4 mul scalar` | 7.50 ns | 910.59 ns | 2.45 us | 219.33 ns | 166.54 ns | 5.92 us | 121.36x | 4.15x | 5.47x | 0.15x |
| `vec4 div scalar` | 14.51 ns | 2.02 us | 1.81 us | 555.97 ns | 232.83 ns | 10.36 us | 139.07x | 3.63x | 8.66x | 0.19x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.37 ns | 1.94 us | 3.62 us | 974.72 ns | 901.18 ns | 22.71 us | 156.50x | 1.99x | 2.15x | 0.09x |
| `mat3 inverse` | 83.61 ns | 19.39 us | 11.77 us | 3.28 us | 2.55 us | 84.99 us | 231.96x | 5.91x | 7.61x | 0.23x |
| `mat3 mul mat3` | 72.96 ns | 8.40 us | 12.83 us | 2.86 us | 2.41 us | 63.53 us | 115.08x | 2.94x | 3.49x | 0.13x |
| `mat3 transform vec3` | 16.51 ns | 3.49 us | 11.19 us | 1.51 us | 908.51 ns | 20.63 us | 211.30x | 2.32x | 3.84x | 0.17x |
| `mat4 determinant` | 46.84 ns | 5.56 us | 4.17 us | 4.71 us | 4.20 us | 96.81 us | 118.67x | 1.18x | 1.32x | 0.06x |
| `mat4 inverse` | 156.57 ns | 36.85 us | 17.38 us | 11.45 us | 9.35 us | 349.94 us | 235.39x | 3.22x | 3.94x | 0.11x |
| `mat4 mul mat4` | 128.04 ns | 17.90 us | 18.58 us | 6.55 us | 5.44 us | 146.14 us | 139.76x | 2.73x | 3.29x | 0.12x |
| `mat4 transform vec4` | 26.28 ns | 5.69 us | 4.62 us | 1.94 us | 1.73 us | 37.67 us | 216.61x | 2.93x | 3.29x | 0.15x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.63 ns | 1.47 us | 4.81 us | 218.93 ns | 244.54 ns | 2.09 us | 41.29x | 6.72x | 6.02x | 0.71x |
| `mat3 zero` | 16.48 ns | 881.76 ns | 892.21 ns | 250.46 ns | 210.97 ns | 12.49 ns | 53.50x | 3.52x | 4.18x | 70.60x |
| `mat3 identity` | 10.04 ns | 959.52 ns | 948.52 ns | 313.13 ns | 241.04 ns | 152.40 ns | 95.56x | 3.06x | 3.98x | 6.30x |
| `mat3 transpose` | 9.16 ns | 951.82 ns | 935.59 ns | 243.00 ns | 212.19 ns | 138.80 ns | 103.88x | 3.92x | 4.49x | 6.86x |
| `mat3 reciprocal` | 84.22 ns | 19.18 us | 37.88 us | 3.00 us | 2.41 us | 89.17 us | 227.76x | 6.39x | 7.97x | 0.22x |
| `mat3 reciprocal checked` | 81.82 ns | 18.49 us | 36.59 us | 3.15 us | 2.42 us | 84.90 us | 225.91x | 5.87x | 7.65x | 0.22x |
| `mat3 inverse checked` | 81.76 ns | 18.80 us | 37.72 us | 3.02 us | 2.36 us | 88.76 us | 230.00x | 6.23x | 7.97x | 0.21x |
| `mat3 inverse checked abort` | 122.83 ns | 18.65 us | 37.38 us | 3.05 us | 2.38 us | 85.51 us | 151.85x | 6.11x | 7.85x | 0.22x |
| `mat3 powi` | 160.11 ns | 18.29 us | 92.06 us | 6.78 us | 6.20 us | 152.32 us | 114.26x | 2.70x | 2.95x | 0.12x |
| `mat3 powi checked` | 160.42 ns | 17.98 us | 91.05 us | 6.69 us | 6.15 us | 153.88 us | 112.08x | 2.69x | 2.92x | 0.12x |
| `mat3 powi checked abort` | 148.02 ns | 17.18 us | 91.11 us | 6.74 us | 6.33 us | 152.83 us | 116.04x | 2.55x | 2.71x | 0.11x |
| `mat3 div scalar checked` | 25.40 ns | 4.96 us | 6.50 us | 1.48 us | 833.02 ns | 23.24 us | 195.33x | 3.35x | 5.96x | 0.21x |
| `mat3 div scalar checked abort` | 29.52 ns | 4.90 us | 6.43 us | 1.51 us | 827.77 ns | 22.68 us | 165.91x | 3.24x | 5.92x | 0.22x |
| `mat3 div matrix checked` | 154.25 ns | 66.62 us | 68.58 us | 5.53 us | 4.63 us | 164.33 us | 431.89x | 12.04x | 14.38x | 0.41x |
| `mat3 div matrix checked abort` | 163.88 ns | 67.61 us | 68.43 us | 5.51 us | 4.58 us | 165.27 us | 412.57x | 12.27x | 14.76x | 0.41x |
| `mat3 add` | 14.66 ns | 4.15 us | 6.52 us | 540.37 ns | 492.99 ns | 11.86 us | 283.14x | 7.68x | 8.42x | 0.35x |
| `mat3 add scalar` | 12.23 ns | 3.24 us | 5.53 us | 800.79 ns | 740.72 ns | 12.19 us | 264.69x | 4.04x | 4.37x | 0.27x |
| `mat3 sub` | 13.37 ns | 4.22 us | 6.27 us | 572.60 ns | 526.32 ns | 23.10 us | 315.66x | 7.37x | 8.02x | 0.18x |
| `mat3 sub scalar` | 11.13 ns | 3.37 us | 5.45 us | 815.91 ns | 719.61 ns | 22.48 us | 302.49x | 4.13x | 4.68x | 0.15x |
| `mat3 neg` | 10.43 ns | 870.95 ns | 892.41 ns | 504.88 ns | 464.28 ns | 9.08 us | 83.49x | 1.73x | 1.88x | 0.10x |
| `mat3 mul scalar` | 13.89 ns | 2.55 us | 6.41 us | 793.23 ns | 691.01 ns | 12.38 us | 183.32x | 3.21x | 3.69x | 0.21x |
| `mat3 div scalar` | 25.23 ns | 4.98 us | 6.23 us | 1.51 us | 821.14 ns | 23.53 us | 197.35x | 3.29x | 6.06x | 0.21x |
| `mat3 div matrix` | 150.78 ns | 66.74 us | 68.04 us | 5.49 us | 4.59 us | 164.15 us | 442.65x | 12.15x | 14.55x | 0.41x |
| `mat3 bitxor` | 158.45 ns | 18.29 us | 90.05 us | 7.08 us | 6.63 us | 151.39 us | 115.46x | 2.59x | 2.76x | 0.12x |
| `mat4 zero` | 11.29 ns | 1.65 us | 1.55 us | 476.09 ns | 364.06 ns | 15.19 ns | 146.16x | 3.46x | 4.53x | 108.59x |
| `mat4 identity` | 11.37 ns | 2.03 us | 1.91 us | 590.61 ns | 433.36 ns | 245.02 ns | 178.64x | 3.44x | 4.69x | 8.29x |
| `mat4 transpose` | 9.45 ns | 1.56 us | 1.58 us | 432.47 ns | 381.04 ns | 211.70 ns | 165.18x | 3.61x | 4.10x | 7.37x |
| `mat4 reciprocal` | 156.53 ns | 35.72 us | 67.76 us | 10.93 us | 9.10 us | 354.68 us | 228.24x | 3.27x | 3.92x | 0.10x |
| `mat4 reciprocal checked` | 167.58 ns | 35.68 us | 69.50 us | 10.99 us | 9.13 us | 347.92 us | 212.93x | 3.25x | 3.91x | 0.10x |
| `mat4 powi` | 245.25 ns | 36.25 us | 117.73 us | 16.17 us | 14.66 us | 361.90 us | 147.82x | 2.24x | 2.47x | 0.10x |
| `mat4 powi checked` | 246.07 ns | 36.63 us | 118.45 us | 16.48 us | 15.19 us | 352.22 us | 148.86x | 2.22x | 2.41x | 0.10x |
| `mat4 add` | 51.69 ns | 6.77 us | 7.64 us | 949.57 ns | 878.32 ns | 19.76 us | 130.92x | 7.13x | 7.70x | 0.34x |
| `mat4 add scalar` | 24.50 ns | 5.21 us | 7.88 us | 1.50 us | 1.25 us | 21.31 us | 212.52x | 3.48x | 4.16x | 0.24x |
| `mat4 sub` | 39.40 ns | 6.88 us | 7.93 us | 950.22 ns | 944.78 ns | 37.83 us | 174.67x | 7.24x | 7.28x | 0.18x |
| `mat4 sub scalar` | 19.07 ns | 5.23 us | 7.72 us | 1.45 us | 1.19 us | 39.71 us | 274.03x | 3.60x | 4.38x | 0.13x |
| `mat4 neg` | 14.53 ns | 1.53 us | 1.50 us | 920.91 ns | 743.29 ns | 14.17 us | 105.15x | 1.66x | 2.06x | 0.11x |
| `mat4 mul scalar` | 26.03 ns | 3.83 us | 8.23 us | 1.45 us | 1.12 us | 20.42 us | 147.10x | 2.65x | 3.42x | 0.19x |
| `mat4 div scalar` | 39.19 ns | 8.34 us | 8.59 us | 2.73 us | 1.43 us | 40.06 us | 212.81x | 3.05x | 5.84x | 0.21x |
| `mat4 div matrix` | 228.07 ns | 142.59 us | 109.09 us | 17.23 us | 14.59 us | 594.20 us | 625.20x | 8.27x | 9.77x | 0.24x |
| `mat4 bitxor` | 246.52 ns | 35.44 us | 116.52 us | 15.80 us | 15.05 us | 363.38 us | 143.75x | 2.24x | 2.35x | 0.10x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.67 ns | 223.33 ns | 324.16 ns | - | - | 1.33 us | 60.91x | - | - | 0.17x |
| `scalar add ref_owned` | 12.29 ns | 232.81 ns | 319.86 ns | - | - | 1.32 us | 18.94x | - | - | 0.18x |
| `scalar add refs` | 5.57 ns | 200.15 ns | 301.93 ns | - | - | 1.32 us | 35.93x | - | - | 0.15x |
| `scalar add owned_ref_with_clone` | 9.13 ns | 261.06 ns | 382.88 ns | - | - | - | 28.61x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.57 ns | 245.89 ns | 382.10 ns | - | - | - | 44.16x | - | - | - |
| `scalar sub owned_ref` | 4.17 ns | 227.25 ns | 383.52 ns | - | - | 2.54 us | 54.49x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.62 ns | 225.14 ns | 424.74 ns | - | - | 2.54 us | 17.84x | - | - | 0.09x |
| `scalar sub refs` | 5.76 ns | 193.64 ns | 369.86 ns | - | - | 2.51 us | 33.63x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.19 ns | 252.51 ns | 443.47 ns | - | - | - | 27.48x | - | - | - |
| `scalar sub ref_owned_with_clone` | 6.50 ns | 252.33 ns | 450.84 ns | - | - | - | 38.82x | - | - | - |
| `scalar mul owned_ref` | 4.93 ns | 131.68 ns | 416.42 ns | - | - | 1.61 us | 26.70x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.61 ns | 128.06 ns | 401.91 ns | - | - | 1.56 us | 9.41x | - | - | 0.08x |
| `scalar mul refs` | 6.28 ns | 110.35 ns | 388.90 ns | - | - | 1.58 us | 17.56x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 9.80 ns | 160.60 ns | 468.74 ns | - | - | - | 16.39x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.58 ns | 158.73 ns | 463.64 ns | - | - | - | 24.11x | - | - | - |
| `scalar div owned_ref` | 6.29 ns | 295.10 ns | 345.07 ns | - | - | 2.59 us | 46.91x | - | - | 0.11x |
| `scalar div ref_owned` | 17.88 ns | 304.40 ns | 367.92 ns | - | - | 2.69 us | 17.02x | - | - | 0.11x |
| `scalar div refs` | 8.90 ns | 275.69 ns | 332.50 ns | - | - | 2.60 us | 30.97x | - | - | 0.11x |
| `scalar div owned_ref_with_clone` | 16.11 ns | 320.76 ns | 408.25 ns | - | - | - | 19.91x | - | - | - |
| `scalar div ref_owned_with_clone` | 14.11 ns | 343.39 ns | 408.04 ns | - | - | - | 24.34x | - | - | - |
| `vec3 add refs` | 6.10 ns | 735.72 ns | 1.52 us | - | - | 4.13 us | 120.62x | - | - | 0.18x |
| `vec3 sub refs` | 6.16 ns | 761.60 ns | 1.58 us | - | - | 7.34 us | 123.55x | - | - | 0.10x |
| `vec3 neg ref` | 3.31 ns | 278.19 ns | 251.81 ns | - | - | 3.24 us | 84.10x | - | - | 0.09x |
| `vec3 add_scalar_ref` | 6.75 ns | 977.71 ns | 1.67 us | - | - | 4.04 us | 144.78x | - | - | 0.24x |
| `vec3 sub_scalar_ref` | 6.74 ns | 960.96 ns | 1.48 us | - | - | 7.12 us | 142.62x | - | - | 0.14x |
| `vec3 mul_scalar_ref` | 7.04 ns | 700.82 ns | 2.30 us | - | - | 4.58 us | 99.50x | - | - | 0.15x |
| `vec3 div_scalar_ref` | 8.46 ns | 1.46 us | 1.80 us | - | - | 8.01 us | 172.13x | - | - | 0.18x |
| `vec4 add refs` | 7.01 ns | 896.96 ns | 1.54 us | - | - | 5.34 us | 128.04x | - | - | 0.17x |
| `vec4 sub refs` | 3.17 ns | 890.83 ns | 1.41 us | - | - | 10.12 us | 280.90x | - | - | 0.09x |
| `vec4 neg ref` | 4.27 ns | 375.44 ns | 398.51 ns | - | - | 4.33 us | 87.89x | - | - | 0.09x |
| `vec4 add_scalar_ref` | 7.01 ns | 1.25 us | 1.97 us | - | - | 5.12 us | 178.44x | - | - | 0.24x |
| `vec4 sub_scalar_ref` | 4.25 ns | 1.22 us | 1.73 us | - | - | 9.68 us | 287.19x | - | - | 0.13x |
| `vec4 mul_scalar_ref` | 7.52 ns | 887.01 ns | 2.28 us | - | - | 5.81 us | 117.93x | - | - | 0.15x |
| `vec4 div_scalar_ref` | 12.28 ns | 1.99 us | 1.80 us | - | - | 10.10 us | 161.63x | - | - | 0.20x |
| `mat3 add refs` | 11.87 ns | 2.08 us | 4.26 us | - | - | 11.71 us | 175.12x | - | - | 0.18x |
| `mat3 sub refs` | 11.47 ns | 2.03 us | 4.13 us | - | - | 21.51 us | 177.23x | - | - | 0.09x |
| `mat3 mul refs` | 58.39 ns | 6.84 us | 31.04 us | - | - | 63.01 us | 117.18x | - | - | 0.11x |
| `mat3 div refs` | 150.38 ns | 70.42 us | 68.04 us | - | - | 166.43 us | 468.32x | - | - | 0.42x |
| `mat3 neg ref` | 9.71 ns | 815.17 ns | 811.55 ns | - | - | 8.70 us | 83.92x | - | - | 0.09x |
| `mat3 add_scalar_ref` | 45.70 ns | 2.96 us | 5.34 us | - | - | 12.61 us | 64.83x | - | - | 0.23x |
| `mat3 sub_scalar_ref` | 45.67 ns | 3.02 us | 5.26 us | - | - | 21.95 us | 66.09x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 47.07 ns | 2.33 us | 6.23 us | - | - | 12.62 us | 49.51x | - | - | 0.18x |
| `mat3 div_scalar_ref` | 21.61 ns | 4.99 us | 6.20 us | - | - | 23.19 us | 230.80x | - | - | 0.22x |
| `mat4 add refs` | 17.63 ns | 3.31 us | 4.08 us | - | - | 19.75 us | 187.54x | - | - | 0.17x |
| `mat4 sub refs` | 17.06 ns | 3.35 us | 4.18 us | - | - | 36.67 us | 196.39x | - | - | 0.09x |
| `mat4 mul refs` | 102.87 ns | 14.78 us | 30.60 us | - | - | 149.29 us | 143.65x | - | - | 0.10x |
| `mat4 div refs` | 227.03 ns | 137.23 us | 108.80 us | - | - | 542.19 us | 604.46x | - | - | 0.25x |
| `mat4 neg ref` | 13.49 ns | 1.29 us | 1.22 us | - | - | 14.40 us | 95.29x | - | - | 0.09x |
| `mat4 add_scalar_ref` | 53.58 ns | 5.35 us | 8.48 us | - | - | 20.54 us | 99.90x | - | - | 0.26x |
| `mat4 sub_scalar_ref` | 41.67 ns | 5.30 us | 7.83 us | - | - | 38.83 us | 127.22x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 57.74 ns | 3.99 us | 8.49 us | - | - | 20.50 us | 69.11x | - | - | 0.19x |
| `mat4 div_scalar_ref` | 28.35 ns | 9.70 us | 8.68 us | - | - | 37.84 us | 342.31x | - | - | 0.26x |
| `mat3 transform_vec refs` | 14.57 ns | 2.65 us | 11.82 us | - | - | 20.51 us | 182.20x | - | - | 0.13x |
| `mat4 transform_vec refs` | 23.54 ns | 4.15 us | 12.01 us | - | - | 37.45 us | 176.11x | - | - | 0.11x |
| `complex add refs` | 7.76 ns | 418.44 ns | 790.88 ns | - | - | 2.59 us | 53.93x | - | - | 0.16x |
| `complex sub refs` | 8.01 ns | 499.14 ns | 801.37 ns | - | - | 4.94 us | 62.28x | - | - | 0.10x |
| `complex mul refs` | 8.12 ns | 890.32 ns | 3.25 us | - | - | 10.26 us | 109.65x | - | - | 0.09x |
| `complex div refs` | 17.89 ns | 2.87 us | 6.50 us | - | - | 22.48 us | 160.59x | - | - | 0.13x |
| `complex neg ref` | 2.79 ns | 124.66 ns | 120.18 ns | - | - | 2.21 us | 44.67x | - | - | 0.06x |
| `complex div_real_ref` | 7.39 ns | 734.77 ns | 722.37 ns | - | - | 5.48 us | 99.42x | - | - | 0.13x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 13.09 us |
| `astro sin 160` | 14.35 us |
| `astro sin 192` | 14.54 us |
| `astro sin 256` | 16.20 us |
