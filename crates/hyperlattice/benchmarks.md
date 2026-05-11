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
| `sin 0.1` | 10.84 ns | 147.30 ns | 147.12 ns | 10.61 us | 753.37 ns | 1.82 us | 13.58x | 0.01x | 0.20x | 0.08x |
| `cos 0.1` | 11.54 ns | 146.88 ns | 147.00 ns | 10.30 us | 491.68 ns | 1.67 us | 12.72x | 0.01x | 0.30x | 0.09x |
| `sin 1.23456789` | 11.73 ns | 201.59 ns | 189.00 ns | 12.17 us | 800.69 ns | 1.79 us | 17.18x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 11.94 ns | 194.46 ns | 182.49 ns | 10.39 us | 586.84 ns | 1.62 us | 16.28x | 0.02x | 0.33x | 0.12x |
| `sin 1e6` | 12.38 ns | 89.66 ns | 89.47 ns | 15.78 us | 1.09 us | 2.01 us | 7.24x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 12.43 ns | 97.45 ns | 91.15 ns | 13.45 us | 820.74 ns | 1.80 us | 7.84x | 0.01x | 0.12x | 0.05x |
| `sin 1e30` | 67.09 ns | 90.37 ns | 91.38 ns | 18.25 us | 2.85 us | 3.65 us | 1.35x | 0.00x | 0.03x | 0.02x |
| `cos 1e30` | 67.83 ns | 92.48 ns | 92.86 ns | 15.06 us | 975.90 ns | 3.09 us | 1.36x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 11.67 ns | 147.65 ns | 340.64 ns | 11.78 us | 730.37 ns | 1.88 us | 12.65x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.55 ns | 146.26 ns | 738.79 ns | 10.63 us | 533.17 ns | 1.70 us | 12.66x | 0.01x | 0.27x | 0.09x |
| `sin 1000pi_eps` | 11.64 ns | 88.92 ns | 776.54 ns | 15.80 us | 2.28 us | 2.82 us | 7.64x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.56 ns | 91.16 ns | 777.92 ns | 13.48 us | 558.91 ns | 1.68 us | 7.26x | 0.01x | 0.16x | 0.05x |
| `asin 0.5` | 10.91 ns | 445.75 ns | 444.97 ns | 49.15 us | 2.94 us | 13.12 us | 40.84x | 0.01x | 0.15x | 0.03x |
| `acos 0.5` | 11.33 ns | 978.54 ns | 994.25 ns | 58.68 us | 2.93 us | 12.99 us | 86.38x | 0.02x | 0.33x | 0.08x |
| `atanh 0.5` | 14.41 ns | 1.42 us | 1.40 us | 34.71 us | 1.63 us | 12.73 us | 98.27x | 0.04x | 0.87x | 0.11x |
| `asin neg_0.999999` | 13.92 ns | 756.75 ns | 1.18 us | 13.59 us | 2.55 us | 12.78 us | 54.35x | 0.06x | 0.30x | 0.06x |
| `acos neg_0.999999` | 15.25 ns | 839.81 ns | 1.67 us | 17.93 us | 2.70 us | 12.88 us | 55.06x | 0.05x | 0.31x | 0.07x |
| `atanh neg_0.999999` | 14.29 ns | 385.39 ns | 575.70 ns | 36.21 us | 1.56 us | 12.57 us | 26.96x | 0.01x | 0.25x | 0.03x |
| `asin 0.999999` | 13.92 ns | 723.68 ns | 1.36 us | 13.70 us | 2.55 us | 12.92 us | 52.00x | 0.05x | 0.28x | 0.06x |
| `acos 0.999999` | 14.15 ns | 612.38 ns | 1.50 us | 17.99 us | 2.74 us | 12.90 us | 43.29x | 0.03x | 0.22x | 0.05x |
| `atanh 0.999999` | 14.41 ns | 449.58 ns | 855.14 ns | 31.19 us | 1.56 us | 12.42 us | 31.20x | 0.01x | 0.29x | 0.04x |
| `asin 1e-12` | 9.35 ns | 450.42 ns | 1.08 us | 7.89 us | 1.42 us | 14.98 us | 48.19x | 0.06x | 0.32x | 0.03x |
| `acos 1e-12` | 10.06 ns | 723.47 ns | 1.51 us | 9.62 us | 1.43 us | 14.92 us | 71.88x | 0.08x | 0.51x | 0.05x |
| `atanh 1e-12` | 9.79 ns | 434.48 ns | 829.75 ns | 36.10 us | 168.06 ns | 19.74 us | 44.39x | 0.01x | 2.59x | 0.02x |
| `atan 0.5` | 14.68 ns | 297.37 ns | 298.27 ns | 35.17 us | 2.79 us | 17.52 us | 20.26x | 0.01x | 0.11x | 0.02x |
| `asinh 0.5` | 26.79 ns | 429.00 ns | 430.17 ns | 38.75 us | 1.60 us | 7.41 us | 16.02x | 0.01x | 0.27x | 0.06x |
| `atan neg_1e-12` | 14.29 ns | 332.60 ns | 542.41 ns | 1.59 us | 1.14 us | 15.10 us | 23.28x | 0.21x | 0.29x | 0.02x |
| `asinh neg_1e-12` | 15.56 ns | 478.45 ns | 382.68 ns | 41.40 us | 8.49 us | 11.78 us | 30.75x | 0.01x | 0.06x | 0.04x |
| `atan 1e6` | 15.33 ns | 183.09 ns | 183.16 ns | 2.83 us | 1.44 us | 17.73 us | 11.95x | 0.06x | 0.13x | 0.01x |
| `asinh 1e6` | 26.39 ns | 306.57 ns | 309.19 ns | 36.30 us | 1.59 us | 7.13 us | 11.62x | 0.01x | 0.19x | 0.04x |
| `atan neg_1e6` | 15.47 ns | 284.35 ns | 287.30 ns | 2.91 us | 1.45 us | 17.82 us | 18.38x | 0.10x | 0.20x | 0.02x |
| `asinh neg_1e6` | 26.34 ns | 347.48 ns | 348.65 ns | 36.69 us | 1.60 us | 6.95 us | 13.19x | 0.01x | 0.22x | 0.05x |
| `acosh 9` | 13.12 ns | 173.14 ns | 171.53 ns | 42.01 us | 1.59 us | 9.70 us | 13.20x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 12.18 ns | 512.31 ns | 1.37 us | 41.59 us | 8.32 us | 11.23 us | 42.05x | 0.01x | 0.06x | 0.05x |
| `acosh 1e6` | 13.12 ns | 172.57 ns | 172.50 ns | 36.90 us | 1.59 us | 9.81 us | 13.15x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.93 ns | 511.33 ns | 2.33 us | 41.01 us | 1.63 us | 9.67 us | 39.55x | 0.01x | 0.31x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 19.71 ns | 18.65 ns | 25.03 ns | 15.56 ns | 0.94 ns | 41.20x | 0.79x | 1.27x | 20.86x |
| `one` | 0.47 ns | 24.95 ns | 23.55 ns | 41.76 ns | 30.32 ns | 29.90 ns | 52.82x | 0.60x | 0.82x | 0.83x |
| `e` | 0.48 ns | 90.62 ns | 89.28 ns | 53.69 ns | 1.07 us | 225.08 ns | 189.07x | 1.69x | 0.08x | 0.40x |
| `pi` | 0.48 ns | 49.83 ns | 50.92 ns | 45.08 ns | 48.57 ns | 225.57 ns | 104.76x | 1.11x | 1.03x | 0.22x |
| `tau` | 0.47 ns | 49.99 ns | 50.74 ns | 118.73 ns | 99.69 ns | 1.86 us | 105.74x | 0.42x | 0.50x | 0.03x |
| `add` | 5.12 ns | 232.53 ns | 317.76 ns | 52.48 ns | 42.38 ns | 1.26 us | 45.46x | 4.43x | 5.49x | 0.18x |
| `sub` | 5.15 ns | 235.38 ns | 375.29 ns | 56.62 ns | 45.18 ns | 2.38 us | 45.71x | 4.16x | 5.21x | 0.10x |
| `neg` | 5.04 ns | 39.15 ns | 39.40 ns | 23.71 ns | 21.51 ns | 1.05 us | 7.76x | 1.65x | 1.82x | 0.04x |
| `mul` | 5.47 ns | 160.67 ns | 402.55 ns | 57.52 ns | 46.49 ns | 1.53 us | 29.39x | 2.79x | 3.46x | 0.11x |
| `div` | 8.34 ns | 316.48 ns | 383.39 ns | 136.10 ns | 64.25 ns | 2.51 us | 37.94x | 2.33x | 4.93x | 0.13x |
| `reciprocal` | 8.69 ns | 94.57 ns | 82.69 ns | 162.58 ns | 59.98 ns | 1.53 us | 10.89x | 0.58x | 1.58x | 0.06x |
| `reciprocal checked` | 8.99 ns | 84.10 ns | 80.96 ns | 163.28 ns | 59.59 ns | 1.54 us | 9.36x | 0.52x | 1.41x | 0.05x |
| `reciprocal checked abort` | 23.79 ns | 107.98 ns | 88.29 ns | 162.46 ns | 59.90 ns | 1.55 us | 4.54x | 0.66x | 1.80x | 0.07x |
| `pow` | 19.97 ns | 8.41 us | 7.41 us | 54.56 us | 2.87 us | 2.35 us | 421.12x | 0.15x | 2.93x | 3.58x |
| `powi` | 5.78 ns | 390.20 ns | 2.58 us | 281.75 ns | 85.42 ns | 1.58 us | 67.53x | 1.38x | 4.57x | 0.25x |
| `exp` | 19.60 ns | 1.53 us | 2.14 us | 13.91 us | 932.64 ns | 1.87 us | 78.15x | 0.11x | 1.64x | 0.82x |
| `ln` | 10.63 ns | 1.63 us | 2.43 us | 29.46 us | 1.31 us | 1.80 us | 153.55x | 0.06x | 1.25x | 0.90x |
| `log10` | 13.34 ns | 4.65 us | 5.79 us | 34.98 us | 2.74 us | 6.59 us | 348.82x | 0.13x | 1.70x | 0.71x |
| `log10 abort` | 16.92 ns | 4.68 us | 5.80 us | 35.03 us | 2.73 us | 6.63 us | 276.48x | 0.13x | 1.71x | 0.71x |
| `sqrt` | 20.00 ns | 1.61 us | 1.59 us | 4.98 us | 95.35 ns | 1.44 us | 80.69x | 0.32x | 16.93x | 1.12x |
| `sin` | 15.20 ns | 134.46 ns | 121.64 ns | 13.72 us | 1.24 us | 2.21 us | 8.85x | 0.01x | 0.11x | 0.06x |
| `cos` | 18.24 ns | 133.77 ns | 123.01 ns | 11.99 us | 628.05 ns | 1.74 us | 7.33x | 0.01x | 0.21x | 0.08x |
| `tan` | 24.68 ns | 181.69 ns | 176.96 ns | 29.33 us | 1.59 us | 6.59 us | 7.36x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.11 ns | 4.14 us | 4.21 us | 3.29 us | 1.15 us | 10.52 us | 228.77x | 1.26x | 3.61x | 0.39x |
| `cosh` | 18.11 ns | 4.10 us | 4.16 us | 7.92 us | 1.07 us | 9.47 us | 226.23x | 0.52x | 3.82x | 0.43x |
| `tanh` | 23.09 ns | 8.32 us | 8.33 us | 3.35 us | 1.21 us | 22.84 us | 360.57x | 2.48x | 6.88x | 0.36x |
| `asin` | 13.15 ns | 640.46 ns | 1.47 us | 20.93 us | 2.42 us | 13.64 us | 48.69x | 0.03x | 0.26x | 0.05x |
| `asin abort` | 17.10 ns | 635.77 ns | 1.48 us | 21.05 us | 2.43 us | 14.05 us | 37.18x | 0.03x | 0.26x | 0.05x |
| `acos` | 15.74 ns | 850.99 ns | 2.14 us | 26.30 us | 2.52 us | 13.76 us | 54.05x | 0.03x | 0.34x | 0.06x |
| `acos abort` | 17.67 ns | 854.20 ns | 2.15 us | 26.34 us | 2.53 us | 13.79 us | 48.33x | 0.03x | 0.34x | 0.06x |
| `atan` | 14.85 ns | 290.11 ns | 931.60 ns | 18.39 us | 2.24 us | 18.71 us | 19.54x | 0.02x | 0.13x | 0.02x |
| `atan abort` | 21.08 ns | 302.00 ns | 941.08 ns | 18.47 us | 2.27 us | 18.87 us | 14.33x | 0.02x | 0.13x | 0.02x |
| `asinh` | 33.81 ns | 431.83 ns | 1.07 us | 38.55 us | 1.65 us | 7.46 us | 12.77x | 0.01x | 0.26x | 0.06x |
| `asinh abort` | 31.25 ns | 435.37 ns | 1.08 us | 38.52 us | 1.64 us | 7.40 us | 13.93x | 0.01x | 0.27x | 0.06x |
| `acosh` | 12.76 ns | 357.71 ns | 1.12 us | 39.88 us | 3.31 us | 10.34 us | 28.03x | 0.01x | 0.11x | 0.03x |
| `acosh abort` | 16.04 ns | 374.27 ns | 1.13 us | 40.24 us | 3.30 us | 10.50 us | 23.33x | 0.01x | 0.11x | 0.04x |
| `atanh` | 13.61 ns | 708.34 ns | 1.37 us | 34.74 us | 1.27 us | 14.80 us | 52.04x | 0.02x | 0.56x | 0.05x |
| `atanh abort` | 17.11 ns | 716.27 ns | 1.34 us | 34.32 us | 1.25 us | 14.71 us | 41.86x | 0.02x | 0.57x | 0.05x |
| `zero status` | 1.20 ns | 1.07 ns | 1.04 ns | 1.03 ns | 6.75 ns | 8.22 ns | 0.89x | 1.05x | 0.16x | 0.13x |
| `zero status abort` | 1.42 ns | 1.12 ns | 1.11 ns | 1.05 ns | 6.74 ns | 8.17 ns | 0.79x | 1.08x | 0.17x | 0.14x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 36.39 ns | 36.36 ns | 51.26 ns | - | 1.91 ns | 38.94x | 0.71x | - | 19.08x |
| `one` | 5.38 ns | 39.60 ns | 39.41 ns | 66.02 ns | - | 33.29 ns | 7.36x | 0.60x | - | 1.19x |
| `i` | 0.94 ns | 41.92 ns | 41.44 ns | 65.85 ns | - | 29.49 ns | 44.37x | 0.64x | - | 1.42x |
| `free i` | 0.94 ns | 41.47 ns | 41.14 ns | 65.86 ns | - | 30.26 ns | 44.03x | 0.63x | - | 1.37x |
| `conjugate` | 2.14 ns | 85.50 ns | 84.82 ns | 41.07 ns | - | 1.06 us | 40.01x | 2.08x | - | 0.08x |
| `norm squared` | 5.76 ns | 320.00 ns | 1.42 us | 151.64 ns | - | 4.25 us | 55.52x | 2.11x | - | 0.08x |
| `reciprocal` | 14.37 ns | 1.74 us | 2.77 us | 447.21 ns | - | 10.67 us | 120.93x | 3.89x | - | 0.16x |
| `reciprocal checked` | 14.39 ns | 1.74 us | 2.79 us | 449.57 ns | - | 10.63 us | 121.25x | 3.88x | - | 0.16x |
| `powi` | 18.14 ns | 2.06 us | 13.21 us | 1.44 us | - | 42.92 us | 113.58x | 1.43x | - | 0.05x |
| `powi checked` | 18.30 ns | 2.06 us | 13.30 us | 1.48 us | - | 43.59 us | 112.76x | 1.40x | - | 0.05x |
| `div checked` | 20.44 ns | 2.66 us | 6.17 us | 793.48 ns | - | 21.41 us | 130.06x | 3.35x | - | 0.12x |
| `div real checked` | 9.35 ns | 718.68 ns | 668.96 ns | 265.26 ns | - | 5.24 us | 76.85x | 2.71x | - | 0.14x |
| `from scalar` | 1.41 ns | 57.21 ns | 55.69 ns | 45.27 ns | - | 9.96 ns | 40.56x | 1.26x | - | 5.74x |
| `add` | 5.90 ns | 473.39 ns | 834.81 ns | 108.06 ns | - | 2.52 us | 80.23x | 4.38x | - | 0.19x |
| `sub` | 5.86 ns | 477.66 ns | 846.21 ns | 117.70 ns | - | 4.70 us | 81.57x | 4.06x | - | 0.10x |
| `neg` | 2.59 ns | 87.13 ns | 85.99 ns | 42.01 ns | - | 2.12 us | 33.70x | 2.07x | - | 0.04x |
| `mul` | 7.63 ns | 876.88 ns | 3.15 us | 312.61 ns | - | 10.33 us | 114.90x | 2.80x | - | 0.08x |
| `div` | 18.42 ns | 2.73 us | 6.16 us | 792.97 ns | - | 21.35 us | 148.19x | 3.44x | - | 0.13x |
| `div real` | 9.86 ns | 653.78 ns | 630.73 ns | 266.11 ns | - | 5.05 us | 66.30x | 2.46x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.90 ns | 223.15 ns | 2.44 us | 307.92 ns | 253.15 ns | 7.15 us | 25.07x | 0.72x | 0.88x | 0.03x |
| `vec3 magnitude` | 16.10 ns | 4.15 us | 5.82 us | 5.47 us | 342.27 ns | 8.66 us | 257.46x | 0.76x | 12.11x | 0.48x |
| `vec3 normalize` | 27.58 ns | 8.24 us | 11.61 us | 6.00 us | 593.04 ns | 16.54 us | 298.74x | 1.37x | 13.89x | 0.50x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.12 ns | 213.07 ns | 1.46 us | 78.94 ns | 56.60 ns | 741.28 ns | 68.27x | 2.70x | 3.76x | 0.29x |
| `vec3 zero` | 1.41 ns | 155.94 ns | 154.05 ns | 60.31 ns | 30.86 ns | 2.83 ns | 110.39x | 2.59x | 5.05x | 55.14x |
| `vec3 dot abort` | 9.27 ns | 215.73 ns | 889.53 ns | 255.06 ns | 204.98 ns | 7.06 us | 23.26x | 0.85x | 1.05x | 0.03x |
| `vec3 magnitude abort` | 18.05 ns | 4.10 us | 3.50 us | 5.46 us | 321.48 ns | 8.65 us | 227.15x | 0.75x | 12.75x | 0.47x |
| `vec3 normalize checked` | 28.59 ns | 8.28 us | 7.54 us | 5.85 us | 542.78 ns | 16.69 us | 289.57x | 1.41x | 15.25x | 0.50x |
| `vec3 normalize checked abort` | 30.66 ns | 8.38 us | 7.55 us | 5.85 us | 544.13 ns | 16.56 us | 273.48x | 1.43x | 15.41x | 0.51x |
| `vec3 div scalar checked` | 10.43 ns | 1.30 us | 1.56 us | 394.64 ns | - | - | 124.92x | 3.30x | - | - |
| `vec3 div scalar checked abort` | 18.17 ns | 1.31 us | 1.56 us | 392.65 ns | - | - | 72.15x | 3.34x | - | - |
| `vec3 add` | 6.63 ns | 951.53 ns | 1.70 us | 155.08 ns | 126.35 ns | 3.87 us | 143.55x | 6.14x | 7.53x | 0.25x |
| `vec3 add scalar` | 6.43 ns | 824.75 ns | 1.50 us | 155.84 ns | 132.66 ns | 3.73 us | 128.24x | 5.29x | 6.22x | 0.22x |
| `vec3 sub` | 6.54 ns | 956.74 ns | 1.74 us | 167.30 ns | 135.89 ns | 7.19 us | 146.35x | 5.72x | 7.04x | 0.13x |
| `vec3 sub scalar` | 6.43 ns | 816.81 ns | 1.28 us | 148.02 ns | 123.96 ns | 6.93 us | 127.05x | 5.52x | 6.59x | 0.12x |
| `vec3 neg` | 3.78 ns | 162.74 ns | 161.47 ns | 58.20 ns | 46.49 ns | 3.07 us | 43.08x | 2.80x | 3.50x | 0.05x |
| `vec3 mul scalar` | 6.94 ns | 603.82 ns | 2.05 us | 162.55 ns | 119.29 ns | 4.34 us | 86.95x | 3.71x | 5.06x | 0.14x |
| `vec3 div scalar` | 10.09 ns | 1.37 us | 1.62 us | 402.33 ns | 170.02 ns | 7.47 us | 136.03x | 3.41x | 8.07x | 0.18x |
| `vec4 dot` | 9.69 ns | 298.79 ns | 712.89 ns | 429.71 ns | 323.36 ns | 9.55 us | 30.82x | 0.70x | 0.92x | 0.03x |
| `vec4 magnitude` | 17.67 ns | 3.86 us | 2.88 us | 5.48 us | 406.37 ns | 11.17 us | 218.22x | 0.70x | 9.49x | 0.35x |
| `vec4 normalize` | 33.71 ns | 8.57 us | 6.70 us | 6.09 us | 691.56 ns | 22.24 us | 254.17x | 1.41x | 12.39x | 0.39x |
| `vec4 add` | 7.84 ns | 1.20 us | 1.82 us | 205.51 ns | 171.42 ns | 5.16 us | 153.15x | 5.84x | 7.00x | 0.23x |
| `vec4 add scalar` | 6.80 ns | 981.42 ns | 1.71 us | 215.32 ns | 179.05 ns | 4.98 us | 144.29x | 4.56x | 5.48x | 0.20x |
| `vec4 sub` | 5.20 ns | 1.19 us | 1.70 us | 215.09 ns | 176.68 ns | 9.47 us | 227.88x | 5.51x | 6.71x | 0.13x |
| `vec4 sub scalar` | 4.45 ns | 968.23 ns | 1.45 us | 206.31 ns | 170.09 ns | 9.29 us | 217.59x | 4.69x | 5.69x | 0.10x |
| `vec4 neg` | 4.93 ns | 186.16 ns | 187.38 ns | 78.16 ns | 64.92 ns | 3.97 us | 37.74x | 2.38x | 2.87x | 0.05x |
| `vec4 mul scalar` | 7.27 ns | 661.05 ns | 2.02 us | 223.98 ns | 163.05 ns | 5.60 us | 90.93x | 2.95x | 4.05x | 0.12x |
| `vec4 div scalar` | 14.14 ns | 1.72 us | 1.54 us | 524.70 ns | 230.35 ns | 9.88 us | 121.58x | 3.28x | 7.47x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.22 ns | 1.16 us | 2.65 us | 952.95 ns | 848.70 ns | 21.93 us | 60.14x | 1.21x | 1.36x | 0.05x |
| `mat3 inverse` | 95.04 ns | 16.27 us | 9.87 us | 3.26 us | 2.44 us | 81.33 us | 171.22x | 5.00x | 6.68x | 0.20x |
| `mat3 mul mat3` | 52.27 ns | 2.99 us | 7.17 us | 2.87 us | 2.35 us | 60.46 us | 57.27x | 1.04x | 1.28x | 0.05x |
| `mat3 transform vec3` | 23.12 ns | 1.39 us | 4.52 us | 1.03 us | 887.52 ns | 19.82 us | 60.08x | 1.34x | 1.56x | 0.07x |
| `mat4 determinant` | 45.95 ns | 3.33 us | 2.13 us | 4.59 us | 4.16 us | 93.49 us | 72.45x | 0.72x | 0.80x | 0.04x |
| `mat4 inverse` | 170.98 ns | 27.45 us | 10.11 us | 11.58 us | 9.32 us | 333.89 us | 160.55x | 2.37x | 2.94x | 0.08x |
| `mat4 mul mat4` | 108.94 ns | 6.55 us | 7.08 us | 6.42 us | 5.40 us | 139.68 us | 60.16x | 1.02x | 1.21x | 0.05x |
| `mat4 transform vec4` | 41.19 ns | 2.32 us | 2.57 us | 1.95 us | 1.66 us | 34.60 us | 56.39x | 1.19x | 1.40x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.66 ns | 1.14 us | 4.27 us | 220.70 ns | 230.88 ns | 2.11 us | 32.91x | 5.17x | 4.94x | 0.54x |
| `mat3 zero` | 16.22 ns | 561.23 ns | 566.56 ns | 250.47 ns | 205.10 ns | 12.16 ns | 34.60x | 2.24x | 2.74x | 46.15x |
| `mat3 identity` | 10.27 ns | 611.94 ns | 615.64 ns | 307.69 ns | 236.20 ns | 144.18 ns | 59.59x | 1.99x | 2.59x | 4.24x |
| `mat3 transpose` | 8.90 ns | 709.35 ns | 697.40 ns | 239.62 ns | 205.48 ns | 118.40 ns | 79.66x | 2.96x | 3.45x | 5.99x |
| `mat3 reciprocal` | 109.19 ns | 15.72 us | 26.60 us | 2.95 us | 2.30 us | 82.14 us | 143.95x | 5.32x | 6.82x | 0.19x |
| `mat3 reciprocal checked` | 109.75 ns | 15.59 us | 26.47 us | 2.95 us | 2.30 us | 81.22 us | 142.07x | 5.29x | 6.77x | 0.19x |
| `mat3 inverse checked` | 126.20 ns | 16.24 us | 31.52 us | 2.93 us | 2.34 us | 80.88 us | 128.68x | 5.54x | 6.95x | 0.20x |
| `mat3 inverse checked abort` | 132.94 ns | 16.29 us | 31.76 us | 2.95 us | 2.32 us | 80.97 us | 122.51x | 5.53x | 7.02x | 0.20x |
| `mat3 powi` | 101.94 ns | 5.54 us | 39.41 us | 6.95 us | 6.18 us | 146.88 us | 54.32x | 0.80x | 0.90x | 0.04x |
| `mat3 powi checked` | 102.63 ns | 5.56 us | 39.32 us | 6.99 us | 6.17 us | 147.54 us | 54.19x | 0.80x | 0.90x | 0.04x |
| `mat3 powi checked abort` | 104.26 ns | 5.54 us | 39.27 us | 6.96 us | 6.15 us | 150.36 us | 53.15x | 0.80x | 0.90x | 0.04x |
| `mat3 div scalar checked` | 24.42 ns | 4.33 us | 5.68 us | 1.45 us | 830.35 ns | 21.85 us | 177.14x | 2.97x | 5.21x | 0.20x |
| `mat3 div scalar checked abort` | 29.64 ns | 4.21 us | 5.65 us | 1.45 us | 830.33 ns | 21.69 us | 142.05x | 2.91x | 5.07x | 0.19x |
| `mat3 div matrix checked` | 118.71 ns | 19.97 us | 63.78 us | 5.40 us | 4.47 us | 156.10 us | 168.22x | 3.70x | 4.47x | 0.13x |
| `mat3 div matrix checked abort` | 131.85 ns | 19.98 us | 63.99 us | 5.46 us | 4.49 us | 156.90 us | 151.56x | 3.66x | 4.45x | 0.13x |
| `mat3 add` | 14.56 ns | 3.13 us | 5.25 us | 519.04 ns | 484.14 ns | 11.33 us | 214.79x | 6.03x | 6.46x | 0.28x |
| `mat3 add scalar` | 12.15 ns | 2.39 us | 4.53 us | 780.79 ns | 702.86 ns | 11.74 us | 196.67x | 3.06x | 3.40x | 0.20x |
| `mat3 sub` | 13.09 ns | 3.14 us | 5.25 us | 540.99 ns | 513.81 ns | 20.82 us | 240.21x | 5.81x | 6.12x | 0.15x |
| `mat3 sub scalar` | 10.94 ns | 2.37 us | 4.31 us | 796.54 ns | 704.59 ns | 21.03 us | 216.26x | 2.97x | 3.36x | 0.11x |
| `mat3 neg` | 10.16 ns | 603.77 ns | 604.09 ns | 479.11 ns | 455.26 ns | 8.76 us | 59.43x | 1.26x | 1.33x | 0.07x |
| `mat3 mul scalar` | 13.82 ns | 1.75 us | 5.68 us | 770.65 ns | 663.65 ns | 11.99 us | 126.41x | 2.27x | 2.63x | 0.15x |
| `mat3 div scalar` | 24.08 ns | 4.26 us | 5.57 us | 1.45 us | 822.24 ns | 21.63 us | 177.12x | 2.94x | 5.19x | 0.20x |
| `mat3 div matrix` | 129.30 ns | 19.97 us | 63.44 us | 5.40 us | 4.46 us | 156.45 us | 154.49x | 3.70x | 4.48x | 0.13x |
| `mat3 bitxor` | 103.60 ns | 5.57 us | 39.20 us | 6.97 us | 6.25 us | 146.68 us | 53.73x | 0.80x | 0.89x | 0.04x |
| `mat4 zero` | 11.32 ns | 901.88 ns | 912.06 ns | 469.04 ns | 354.39 ns | 14.77 ns | 79.69x | 1.92x | 2.54x | 61.04x |
| `mat4 identity` | 11.35 ns | 1.08 us | 1.08 us | 563.70 ns | 412.78 ns | 217.94 ns | 95.40x | 1.92x | 2.62x | 4.97x |
| `mat4 transpose` | 9.29 ns | 1.10 us | 1.09 us | 424.65 ns | 368.25 ns | 186.51 ns | 118.33x | 2.59x | 2.99x | 5.90x |
| `mat4 reciprocal` | 176.36 ns | 26.27 us | 46.82 us | 16.52 us | 9.07 us | 338.37 us | 148.93x | 1.59x | 2.90x | 0.08x |
| `mat4 reciprocal checked` | 174.85 ns | 25.84 us | 46.60 us | 10.89 us | 9.09 us | 341.10 us | 147.77x | 2.37x | 2.84x | 0.08x |
| `mat4 powi` | 222.85 ns | 11.85 us | 54.68 us | 15.50 us | 14.10 us | 338.90 us | 53.19x | 0.76x | 0.84x | 0.03x |
| `mat4 powi checked` | 217.89 ns | 11.89 us | 54.95 us | 15.63 us | 14.14 us | 344.64 us | 54.55x | 0.76x | 0.84x | 0.03x |
| `mat4 add` | 51.73 ns | 5.01 us | 5.89 us | 917.15 ns | 864.45 ns | 19.29 us | 96.86x | 5.46x | 5.80x | 0.26x |
| `mat4 add scalar` | 20.94 ns | 4.01 us | 6.61 us | 1.44 us | 1.21 us | 20.34 us | 191.68x | 2.79x | 3.32x | 0.20x |
| `mat4 sub` | 38.95 ns | 5.00 us | 5.88 us | 965.46 ns | 906.68 ns | 35.44 us | 128.33x | 5.18x | 5.51x | 0.14x |
| `mat4 sub scalar` | 15.18 ns | 4.05 us | 6.45 us | 1.47 us | 1.21 us | 37.41 us | 267.09x | 2.77x | 3.36x | 0.11x |
| `mat4 neg` | 14.40 ns | 1.01 us | 1.00 us | 904.46 ns | 774.15 ns | 13.90 us | 70.04x | 1.11x | 1.30x | 0.07x |
| `mat4 mul scalar` | 24.63 ns | 2.95 us | 7.34 us | 1.43 us | 1.14 us | 20.09 us | 119.82x | 2.06x | 2.58x | 0.15x |
| `mat4 div scalar` | 34.04 ns | 7.36 us | 7.65 us | 2.68 us | 1.41 us | 37.54 us | 216.30x | 2.75x | 5.24x | 0.20x |
| `mat4 div matrix` | 187.49 ns | 31.97 us | 89.16 us | 17.04 us | 14.50 us | 526.18 us | 170.54x | 1.88x | 2.21x | 0.06x |
| `mat4 bitxor` | 218.79 ns | 12.04 us | 54.47 us | 15.48 us | 14.76 us | 343.44 us | 55.01x | 0.78x | 0.82x | 0.04x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.64 ns | 171.53 ns | 275.69 ns | - | - | 1.26 us | 47.12x | - | - | 0.14x |
| `scalar add ref_owned` | 12.47 ns | 172.73 ns | 269.94 ns | - | - | 1.27 us | 13.85x | - | - | 0.14x |
| `scalar add refs` | 5.33 ns | 154.79 ns | 256.29 ns | - | - | 1.28 us | 29.05x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 8.97 ns | 211.30 ns | 294.36 ns | - | - | - | 23.55x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.47 ns | 196.04 ns | 281.77 ns | - | - | - | 35.82x | - | - | - |
| `scalar sub owned_ref` | 3.91 ns | 177.05 ns | 329.76 ns | - | - | 2.39 us | 45.27x | - | - | 0.07x |
| `scalar sub ref_owned` | 12.66 ns | 178.29 ns | 333.30 ns | - | - | 2.40 us | 14.09x | - | - | 0.07x |
| `scalar sub refs` | 5.67 ns | 162.27 ns | 320.19 ns | - | - | 2.40 us | 28.63x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 9.03 ns | 221.08 ns | 343.87 ns | - | - | - | 24.48x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.69 ns | 203.79 ns | 342.41 ns | - | - | - | 35.81x | - | - | - |
| `scalar mul owned_ref` | 4.53 ns | 97.22 ns | 362.88 ns | - | - | 1.52 us | 21.46x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.69 ns | 98.29 ns | 370.60 ns | - | - | 1.52 us | 7.18x | - | - | 0.06x |
| `scalar mul refs` | 6.32 ns | 82.71 ns | 351.73 ns | - | - | 1.54 us | 13.08x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.68 ns | 129.82 ns | 380.35 ns | - | - | - | 13.41x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.16 ns | 125.57 ns | 377.32 ns | - | - | - | 20.40x | - | - | - |
| `scalar div owned_ref` | 5.81 ns | 255.29 ns | 308.96 ns | - | - | 2.51 us | 43.90x | - | - | 0.10x |
| `scalar div ref_owned` | 17.39 ns | 255.87 ns | 308.79 ns | - | - | 2.52 us | 14.72x | - | - | 0.10x |
| `scalar div refs` | 6.82 ns | 243.22 ns | 297.16 ns | - | - | 2.52 us | 35.68x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.13 ns | 287.15 ns | 327.44 ns | - | - | - | 21.87x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.50 ns | 289.11 ns | 333.53 ns | - | - | - | 18.65x | - | - | - |
| `vec3 add refs` | 6.13 ns | 617.35 ns | 1.38 us | - | - | 3.90 us | 100.79x | - | - | 0.16x |
| `vec3 sub refs` | 6.06 ns | 608.59 ns | 1.48 us | - | - | 7.30 us | 100.36x | - | - | 0.08x |
| `vec3 neg ref` | 3.31 ns | 160.00 ns | 172.24 ns | - | - | 3.08 us | 48.38x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.44 ns | 757.77 ns | 1.54 us | - | - | 3.73 us | 117.72x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.51 ns | 744.87 ns | 1.24 us | - | - | 6.99 us | 114.40x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 6.86 ns | 510.52 ns | 2.02 us | - | - | 4.35 us | 74.37x | - | - | 0.12x |
| `vec3 div_scalar_ref` | 8.08 ns | 1.32 us | 1.53 us | - | - | 7.50 us | 163.60x | - | - | 0.18x |
| `vec4 add refs` | 6.64 ns | 780.61 ns | 1.37 us | - | - | 5.17 us | 117.51x | - | - | 0.15x |
| `vec4 sub refs` | 3.07 ns | 746.47 ns | 1.26 us | - | - | 10.18 us | 243.54x | - | - | 0.07x |
| `vec4 neg ref` | 4.23 ns | 213.84 ns | 201.65 ns | - | - | 4.01 us | 50.59x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 6.93 ns | 1.02 us | 1.71 us | - | - | 4.95 us | 147.89x | - | - | 0.21x |
| `vec4 sub_scalar_ref` | 4.22 ns | 1.03 us | 1.48 us | - | - | 9.78 us | 242.99x | - | - | 0.10x |
| `vec4 mul_scalar_ref` | 7.31 ns | 687.75 ns | 2.07 us | - | - | 5.58 us | 94.02x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.51 ns | 1.74 us | 1.47 us | - | - | 9.77 us | 151.06x | - | - | 0.18x |
| `mat3 add refs` | 10.92 ns | 1.67 us | 3.69 us | - | - | 11.42 us | 153.19x | - | - | 0.15x |
| `mat3 sub refs` | 10.51 ns | 1.68 us | 3.67 us | - | - | 20.98 us | 159.88x | - | - | 0.08x |
| `mat3 mul refs` | 39.01 ns | 1.93 us | 12.51 us | - | - | 60.69 us | 49.52x | - | - | 0.03x |
| `mat3 div refs` | 117.86 ns | 19.13 us | 56.78 us | - | - | 157.28 us | 162.30x | - | - | 0.12x |
| `mat3 neg ref` | 9.72 ns | 487.02 ns | 487.99 ns | - | - | 8.40 us | 50.13x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 44.79 ns | 2.51 us | 4.52 us | - | - | 11.68 us | 56.03x | - | - | 0.21x |
| `mat3 sub_scalar_ref` | 43.96 ns | 2.40 us | 4.33 us | - | - | 21.03 us | 54.59x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 47.22 ns | 1.78 us | 5.68 us | - | - | 11.94 us | 37.63x | - | - | 0.15x |
| `mat3 div_scalar_ref` | 21.53 ns | 4.27 us | 5.62 us | - | - | 21.53 us | 198.10x | - | - | 0.20x |
| `mat4 add refs` | 17.58 ns | 2.65 us | 3.46 us | - | - | 18.57 us | 150.97x | - | - | 0.14x |
| `mat4 sub refs` | 16.34 ns | 2.64 us | 3.54 us | - | - | 35.20 us | 161.45x | - | - | 0.07x |
| `mat4 mul refs` | 95.40 ns | 4.29 us | 18.42 us | - | - | 139.25 us | 44.93x | - | - | 0.03x |
| `mat4 div refs` | 186.32 ns | 30.57 us | 83.69 us | - | - | 521.41 us | 164.10x | - | - | 0.06x |
| `mat4 neg ref` | 12.49 ns | 839.82 ns | 854.73 ns | - | - | 13.82 us | 67.25x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 52.81 ns | 4.05 us | 6.63 us | - | - | 19.97 us | 76.66x | - | - | 0.20x |
| `mat4 sub_scalar_ref` | 39.17 ns | 4.10 us | 6.50 us | - | - | 36.49 us | 104.61x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 56.23 ns | 2.96 us | 7.44 us | - | - | 19.73 us | 52.67x | - | - | 0.15x |
| `mat4 div_scalar_ref` | 28.57 ns | 7.48 us | 7.68 us | - | - | 36.81 us | 261.77x | - | - | 0.20x |
| `mat3 transform_vec refs` | 21.45 ns | 736.75 ns | 3.94 us | - | - | 19.65 us | 34.35x | - | - | 0.04x |
| `mat4 transform_vec refs` | 31.83 ns | 1.28 us | 5.02 us | - | - | 34.74 us | 40.14x | - | - | 0.04x |
| `complex add refs` | 7.80 ns | 321.00 ns | 685.33 ns | - | - | 2.51 us | 41.17x | - | - | 0.13x |
| `complex sub refs` | 8.02 ns | 326.93 ns | 703.96 ns | - | - | 4.67 us | 40.78x | - | - | 0.07x |
| `complex mul refs` | 8.04 ns | 722.46 ns | 3.00 us | - | - | 9.75 us | 89.86x | - | - | 0.07x |
| `complex div refs` | 16.46 ns | 2.50 us | 6.10 us | - | - | 21.40 us | 151.76x | - | - | 0.12x |
| `complex neg ref` | 2.35 ns | 78.19 ns | 70.71 ns | - | - | 2.11 us | 33.31x | - | - | 0.04x |
| `complex div_real_ref` | 7.25 ns | 622.52 ns | 586.23 ns | - | - | 5.02 us | 85.90x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.27 us |
| `astro sin 160` | 14.05 us |
| `astro sin 192` | 14.00 us |
| `astro sin 256` | 15.87 us |
