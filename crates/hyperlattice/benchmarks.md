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
| `sin 0.1` | 10.85 ns | 146.42 ns | 146.90 ns | 11.65 us | 771.53 ns | 1.85 us | 13.50x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.56 ns | 146.11 ns | 146.26 ns | 10.75 us | 491.87 ns | 1.68 us | 12.64x | 0.01x | 0.30x | 0.09x |
| `sin 1.23456789` | 11.63 ns | 204.53 ns | 189.60 ns | 12.82 us | 814.40 ns | 1.92 us | 17.59x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 11.91 ns | 196.87 ns | 180.75 ns | 10.56 us | 585.21 ns | 1.73 us | 16.54x | 0.02x | 0.34x | 0.11x |
| `sin 1e6` | 12.60 ns | 90.98 ns | 91.75 ns | 15.86 us | 1.16 us | 2.08 us | 7.22x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 12.45 ns | 90.79 ns | 91.47 ns | 13.74 us | 1.55 us | 1.83 us | 7.30x | 0.01x | 0.06x | 0.05x |
| `sin 1e30` | 65.33 ns | 92.24 ns | 91.71 ns | 19.09 us | 2.80 us | 3.65 us | 1.41x | 0.00x | 0.03x | 0.03x |
| `cos 1e30` | 68.07 ns | 92.52 ns | 92.24 ns | 15.95 us | 952.94 ns | 3.08 us | 1.36x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.67 ns | 147.56 ns | 359.65 ns | 12.50 us | 755.68 ns | 1.88 us | 12.65x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.55 ns | 147.33 ns | 762.88 ns | 10.76 us | 538.76 ns | 1.74 us | 12.75x | 0.01x | 0.27x | 0.08x |
| `sin 1000pi_eps` | 11.65 ns | 91.30 ns | 792.66 ns | 16.12 us | 2.30 us | 2.79 us | 7.84x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.80 ns | 91.59 ns | 742.78 ns | 13.70 us | 617.85 ns | 1.69 us | 7.16x | 0.01x | 0.15x | 0.05x |
| `asin 0.5` | 11.34 ns | 146.34 ns | 145.52 ns | 51.12 us | 2.94 us | 13.57 us | 12.91x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.44 ns | 145.49 ns | 147.86 ns | 59.50 us | 2.96 us | 13.55 us | 12.71x | 0.00x | 0.05x | 0.01x |
| `atanh 0.5` | 14.67 ns | 149.64 ns | 152.14 ns | 34.59 us | 1.77 us | 12.94 us | 10.20x | 0.00x | 0.08x | 0.01x |
| `asin neg_0.999999` | 14.11 ns | 537.71 ns | 535.56 ns | 14.24 us | 2.52 us | 13.12 us | 38.12x | 0.04x | 0.21x | 0.04x |
| `acos neg_0.999999` | 15.14 ns | 369.75 ns | 334.59 ns | 18.61 us | 2.70 us | 13.20 us | 24.42x | 0.02x | 0.14x | 0.03x |
| `atanh neg_0.999999` | 14.51 ns | 283.03 ns | 254.51 ns | 36.66 us | 1.68 us | 12.95 us | 19.51x | 0.01x | 0.17x | 0.02x |
| `asin 0.999999` | 13.88 ns | 474.89 ns | 1.45 us | 15.91 us | 2.52 us | 13.05 us | 34.20x | 0.03x | 0.19x | 0.04x |
| `acos 0.999999` | 14.25 ns | 345.13 ns | 837.28 ns | 32.92 us | 2.71 us | 13.69 us | 24.22x | 0.01x | 0.13x | 0.03x |
| `atanh 0.999999` | 14.84 ns | 300.44 ns | 549.72 ns | 31.85 us | 1.79 us | 12.77 us | 20.24x | 0.01x | 0.17x | 0.02x |
| `asin 1e-12` | 9.56 ns | 272.73 ns | 506.48 ns | 8.14 us | 1.56 us | 15.47 us | 28.53x | 0.03x | 0.17x | 0.02x |
| `acos 1e-12` | 10.29 ns | 542.49 ns | 964.30 ns | 9.84 us | 2.90 us | 15.39 us | 52.73x | 0.06x | 0.19x | 0.04x |
| `atanh 1e-12` | 9.77 ns | 314.17 ns | 501.52 ns | 36.54 us | 175.25 ns | 20.45 us | 32.14x | 0.01x | 1.79x | 0.02x |
| `atan 0.5` | 14.99 ns | 166.05 ns | 172.28 ns | 34.76 us | 2.87 us | 17.89 us | 11.08x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.80 ns | 215.59 ns | 214.89 ns | 39.12 us | 1.66 us | 7.59 us | 8.04x | 0.01x | 0.13x | 0.03x |
| `atan neg_1e-12` | 14.36 ns | 282.80 ns | 274.37 ns | 1.62 us | 1.20 us | 15.90 us | 19.70x | 0.17x | 0.24x | 0.02x |
| `asinh neg_1e-12` | 15.92 ns | 473.97 ns | 390.00 ns | 42.00 us | 8.82 us | 12.29 us | 29.78x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 15.30 ns | 160.95 ns | 161.17 ns | 2.87 us | 1.43 us | 18.06 us | 10.52x | 0.06x | 0.11x | 0.01x |
| `asinh 1e6` | 26.65 ns | 210.09 ns | 212.21 ns | 36.99 us | 1.76 us | 7.39 us | 7.88x | 0.01x | 0.12x | 0.03x |
| `atan neg_1e6` | 15.48 ns | 269.44 ns | 269.80 ns | 2.90 us | 2.89 us | 18.43 us | 17.40x | 0.09x | 0.09x | 0.01x |
| `asinh neg_1e6` | 26.50 ns | 253.29 ns | 255.02 ns | 36.95 us | 1.83 us | 9.09 us | 9.56x | 0.01x | 0.14x | 0.03x |
| `acosh 9` | 12.41 ns | 146.46 ns | 143.40 ns | 42.25 us | 3.37 us | 10.00 us | 11.80x | 0.00x | 0.04x | 0.01x |
| `acosh 1_plus_1e-12` | 11.56 ns | 285.08 ns | 270.84 ns | 41.87 us | 9.00 us | 11.66 us | 24.67x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.43 ns | 144.51 ns | 144.78 ns | 36.84 us | 1.72 us | 10.12 us | 11.63x | 0.00x | 0.08x | 0.01x |
| `acosh e` | 12.39 ns | 185.60 ns | 2.22 us | 41.32 us | 1.71 us | 9.80 us | 14.98x | 0.00x | 0.11x | 0.02x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.44 ns | 18.82 ns | 26.64 ns | 15.50 ns | 0.96 ns | 38.87x | 0.69x | 1.19x | 19.27x |
| `one` | 0.77 ns | 22.82 ns | 24.55 ns | 42.12 ns | 30.51 ns | 30.11 ns | 29.77x | 0.54x | 0.75x | 0.76x |
| `e` | 0.48 ns | 88.98 ns | 90.94 ns | 57.11 ns | 1.08 us | 231.93 ns | 184.81x | 1.56x | 0.08x | 0.38x |
| `pi` | 0.48 ns | 49.74 ns | 49.80 ns | 44.84 ns | 48.56 ns | 236.59 ns | 103.85x | 1.11x | 1.02x | 0.21x |
| `tau` | 0.48 ns | 55.38 ns | 49.42 ns | 118.11 ns | 100.45 ns | 2.22 us | 114.84x | 0.47x | 0.55x | 0.02x |
| `add` | 5.33 ns | 226.25 ns | 325.42 ns | 52.62 ns | 42.23 ns | 1.40 us | 42.44x | 4.30x | 5.36x | 0.16x |
| `sub` | 5.28 ns | 263.59 ns | 496.61 ns | 55.89 ns | 44.90 ns | 2.62 us | 49.92x | 4.72x | 5.87x | 0.10x |
| `neg` | 5.17 ns | 40.61 ns | 41.15 ns | 23.51 ns | 21.68 ns | 1.14 us | 7.86x | 1.73x | 1.87x | 0.04x |
| `mul` | 5.52 ns | 136.13 ns | 405.11 ns | 58.14 ns | 46.48 ns | 1.61 us | 24.68x | 2.34x | 2.93x | 0.08x |
| `div` | 8.47 ns | 323.74 ns | 366.59 ns | 136.91 ns | 63.15 ns | 2.72 us | 38.23x | 2.36x | 5.13x | 0.12x |
| `reciprocal` | 16.87 ns | 73.60 ns | 75.23 ns | 161.53 ns | 58.81 ns | 1.58 us | 4.36x | 0.46x | 1.25x | 0.05x |
| `reciprocal checked` | 9.06 ns | 68.76 ns | 73.68 ns | 166.57 ns | 58.82 ns | 1.51 us | 7.59x | 0.41x | 1.17x | 0.05x |
| `reciprocal checked abort` | 22.20 ns | 87.53 ns | 79.08 ns | 160.75 ns | 58.88 ns | 1.52 us | 3.94x | 0.54x | 1.49x | 0.06x |
| `pow` | 20.57 ns | 10.04 us | 5.24 us | 55.56 us | 2.91 us | 2.38 us | 488.31x | 0.18x | 3.46x | 4.21x |
| `powi` | 5.99 ns | 376.71 ns | 2.61 us | 277.95 ns | 84.37 ns | 2.86 us | 62.93x | 1.36x | 4.46x | 0.13x |
| `exp` | 20.04 ns | 233.84 ns | 244.18 ns | 14.41 us | 935.23 ns | 2.10 us | 11.67x | 0.02x | 0.25x | 0.11x |
| `ln` | 11.02 ns | 1.03 us | 1.12 us | 29.08 us | 1.35 us | 1.98 us | 93.19x | 0.04x | 0.76x | 0.52x |
| `log10` | 17.52 ns | 1.21 us | 1.31 us | 34.84 us | 2.82 us | 7.04 us | 69.34x | 0.03x | 0.43x | 0.17x |
| `log10 abort` | 18.31 ns | 1.24 us | 1.31 us | 34.98 us | 2.80 us | 6.88 us | 67.56x | 0.04x | 0.44x | 0.18x |
| `sqrt` | 20.58 ns | 1.49 us | 1.65 us | 4.94 us | 95.38 ns | 1.53 us | 72.38x | 0.30x | 15.62x | 0.97x |
| `sin` | 15.05 ns | 123.52 ns | 120.89 ns | 14.58 us | 1.27 us | 2.22 us | 8.21x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.47 ns | 176.68 ns | 119.81 ns | 12.08 us | 635.31 ns | 1.76 us | 9.57x | 0.01x | 0.28x | 0.10x |
| `tan` | 25.30 ns | 169.64 ns | 179.54 ns | 30.71 us | 1.56 us | 6.66 us | 6.70x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.32 ns | 3.60 us | 3.59 us | 3.36 us | 1.13 us | 10.98 us | 196.41x | 1.07x | 3.17x | 0.33x |
| `cosh` | 18.17 ns | 3.55 us | 3.52 us | 7.92 us | 1.06 us | 10.00 us | 195.24x | 0.45x | 3.35x | 0.35x |
| `tanh` | 40.04 ns | 5.96 us | 5.92 us | 3.35 us | 1.20 us | 23.52 us | 148.88x | 1.78x | 4.95x | 0.25x |
| `asin` | 13.52 ns | 363.03 ns | 577.41 ns | 21.84 us | 4.84 us | 14.21 us | 26.86x | 0.02x | 0.08x | 0.03x |
| `asin abort` | 17.21 ns | 373.94 ns | 578.36 ns | 21.48 us | 2.40 us | 14.83 us | 21.72x | 0.02x | 0.16x | 0.03x |
| `acos` | 14.87 ns | 357.54 ns | 741.85 ns | 26.59 us | 2.52 us | 15.53 us | 24.04x | 0.01x | 0.14x | 0.02x |
| `acos abort` | 17.69 ns | 394.60 ns | 734.41 ns | 26.75 us | 2.56 us | 14.05 us | 22.31x | 0.01x | 0.15x | 0.03x |
| `atan` | 15.13 ns | 221.11 ns | 169.09 ns | 18.62 us | 2.26 us | 19.18 us | 14.61x | 0.01x | 0.10x | 0.01x |
| `atan abort` | 21.34 ns | 170.79 ns | 167.73 ns | 18.64 us | 2.22 us | 19.20 us | 8.00x | 0.01x | 0.08x | 0.01x |
| `asinh` | 30.72 ns | 211.13 ns | 221.91 ns | 38.62 us | 1.69 us | 8.44 us | 6.87x | 0.01x | 0.12x | 0.03x |
| `asinh abort` | 31.74 ns | 224.83 ns | 224.58 ns | 38.99 us | 1.69 us | 8.00 us | 7.08x | 0.01x | 0.13x | 0.03x |
| `acosh` | 19.39 ns | 191.17 ns | 182.99 ns | 40.72 us | 3.49 us | 10.66 us | 9.86x | 0.00x | 0.05x | 0.02x |
| `acosh abort` | 16.40 ns | 195.10 ns | 189.23 ns | 40.32 us | 3.45 us | 10.58 us | 11.89x | 0.00x | 0.06x | 0.02x |
| `atanh` | 13.93 ns | 259.75 ns | 467.08 ns | 34.61 us | 1.33 us | 15.09 us | 18.65x | 0.01x | 0.20x | 0.02x |
| `atanh abort` | 17.00 ns | 266.60 ns | 460.01 ns | 34.21 us | 1.34 us | 14.97 us | 15.68x | 0.01x | 0.20x | 0.02x |
| `zero status` | 1.20 ns | 0.96 ns | 0.94 ns | 1.03 ns | 6.77 ns | 8.03 ns | 0.80x | 0.93x | 0.14x | 0.12x |
| `zero status abort` | 1.42 ns | 1.78 ns | 0.97 ns | 1.03 ns | 6.82 ns | 8.05 ns | 1.25x | 1.73x | 0.26x | 0.22x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.95 ns | 36.83 ns | 36.49 ns | 52.36 ns | - | 1.94 ns | 38.93x | 0.70x | - | 18.94x |
| `one` | 5.42 ns | 39.59 ns | 39.79 ns | 67.17 ns | - | 41.71 ns | 7.30x | 0.59x | - | 0.95x |
| `i` | 0.96 ns | 43.81 ns | 42.96 ns | 66.94 ns | - | 31.31 ns | 45.44x | 0.65x | - | 1.40x |
| `free i` | 0.95 ns | 42.88 ns | 66.83 ns | 66.79 ns | - | 30.51 ns | 45.37x | 0.64x | - | 1.41x |
| `conjugate` | 2.16 ns | 85.71 ns | 136.53 ns | 48.99 ns | - | 1.11 us | 39.71x | 1.75x | - | 0.08x |
| `norm squared` | 5.86 ns | 173.82 ns | 580.83 ns | 155.26 ns | - | 4.54 us | 29.68x | 1.12x | - | 0.04x |
| `reciprocal` | 17.19 ns | 1.71 us | 2.87 us | 475.39 ns | - | 11.35 us | 99.48x | 3.60x | - | 0.15x |
| `reciprocal checked` | 15.02 ns | 1.76 us | 4.79 us | 462.30 ns | - | 10.97 us | 116.91x | 3.80x | - | 0.16x |
| `powi` | 17.60 ns | 1.30 us | 6.88 us | 1.47 us | - | 44.73 us | 73.65x | 0.88x | - | 0.03x |
| `powi checked` | 17.93 ns | 1.32 us | 6.50 us | 1.51 us | - | 45.58 us | 73.71x | 0.87x | - | 0.03x |
| `div checked` | 16.80 ns | 2.43 us | 4.60 us | 805.42 ns | - | 22.07 us | 144.52x | 3.01x | - | 0.11x |
| `div real checked` | 17.57 ns | 657.93 ns | 632.40 ns | 272.84 ns | - | 5.63 us | 37.44x | 2.41x | - | 0.12x |
| `from scalar` | 1.44 ns | 58.48 ns | 55.95 ns | 53.18 ns | - | 11.49 ns | 40.71x | 1.10x | - | 5.09x |
| `add` | 5.85 ns | 485.58 ns | 861.18 ns | 111.68 ns | - | 2.71 us | 82.95x | 4.35x | - | 0.18x |
| `sub` | 5.87 ns | 543.79 ns | 1.16 us | 124.16 ns | - | 4.99 us | 92.67x | 4.38x | - | 0.11x |
| `neg` | 2.62 ns | 87.88 ns | 84.59 ns | 49.90 ns | - | 2.17 us | 33.54x | 1.76x | - | 0.04x |
| `mul` | 7.69 ns | 927.87 ns | 3.56 us | 313.76 ns | - | 10.27 us | 120.61x | 2.96x | - | 0.09x |
| `div` | 15.57 ns | 2.37 us | 4.61 us | 818.48 ns | - | 22.10 us | 151.92x | 2.89x | - | 0.11x |
| `div real` | 10.28 ns | 662.56 ns | 635.64 ns | 274.38 ns | - | 5.26 us | 64.46x | 2.41x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.62 ns | 220.63 ns | 2.38 us | 317.98 ns | 256.25 ns | 7.25 us | 25.60x | 0.69x | 0.86x | 0.03x |
| `vec3 magnitude` | 11.60 ns | 4.05 us | 5.72 us | 5.33 us | 351.49 ns | 8.94 us | 349.50x | 0.76x | 11.54x | 0.45x |
| `vec3 normalize` | 25.92 ns | 8.18 us | 11.63 us | 5.96 us | 596.78 ns | 16.91 us | 315.45x | 1.37x | 13.70x | 0.48x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.06 ns | 283.58 ns | 3.08 us | 72.25 ns | 57.56 ns | 766.80 ns | 92.54x | 3.92x | 4.93x | 0.37x |
| `vec3 zero` | 1.42 ns | 139.43 ns | 146.13 ns | 61.65 ns | 31.00 ns | 2.83 ns | 98.47x | 2.26x | 4.50x | 49.33x |
| `vec3 dot abort` | 8.74 ns | 213.25 ns | 892.08 ns | 268.31 ns | 203.27 ns | 7.27 us | 24.40x | 0.79x | 1.05x | 0.03x |
| `vec3 magnitude abort` | 17.17 ns | 3.98 us | 3.15 us | 5.58 us | 335.86 ns | 9.44 us | 231.68x | 0.71x | 11.84x | 0.42x |
| `vec3 normalize checked` | 25.05 ns | 8.19 us | 7.03 us | 6.01 us | 557.22 ns | 17.09 us | 326.82x | 1.36x | 14.69x | 0.48x |
| `vec3 normalize checked abort` | 29.18 ns | 8.09 us | 7.15 us | 5.95 us | 561.78 ns | 17.23 us | 277.28x | 1.36x | 14.40x | 0.47x |
| `vec3 div scalar checked` | 9.72 ns | 1.32 us | 1.55 us | 402.99 ns | - | - | 135.61x | 3.27x | - | - |
| `vec3 div scalar checked abort` | 18.12 ns | 1.33 us | 1.56 us | 402.95 ns | - | - | 73.19x | 3.29x | - | - |
| `vec3 add` | 7.25 ns | 952.98 ns | 1.87 us | 151.18 ns | 131.70 ns | 4.02 us | 131.46x | 6.30x | 7.24x | 0.24x |
| `vec3 add scalar` | 6.38 ns | 779.84 ns | 1.48 us | 157.47 ns | 136.24 ns | 3.86 us | 122.23x | 4.95x | 5.72x | 0.20x |
| `vec3 sub` | 7.22 ns | 1.08 us | 2.49 us | 342.68 ns | 139.43 ns | 7.51 us | 149.46x | 3.15x | 7.74x | 0.14x |
| `vec3 sub scalar` | 6.42 ns | 803.43 ns | 1.28 us | 323.12 ns | 127.59 ns | 7.18 us | 125.22x | 2.49x | 6.30x | 0.11x |
| `vec3 neg` | 3.76 ns | 143.21 ns | 142.06 ns | 59.11 ns | 50.25 ns | 3.16 us | 38.07x | 2.42x | 2.85x | 0.05x |
| `vec3 mul scalar` | 6.94 ns | 520.26 ns | 2.00 us | 169.84 ns | 130.88 ns | 4.52 us | 74.99x | 3.06x | 3.98x | 0.12x |
| `vec3 div scalar` | 9.58 ns | 1.29 us | 1.54 us | 406.10 ns | 177.32 ns | 7.63 us | 134.97x | 3.18x | 7.29x | 0.17x |
| `vec4 dot` | 9.73 ns | 262.66 ns | 629.98 ns | 442.74 ns | 332.97 ns | 9.85 us | 26.99x | 0.59x | 0.79x | 0.03x |
| `vec4 magnitude` | 16.77 ns | 3.81 us | 2.56 us | 5.57 us | 434.48 ns | 11.51 us | 226.86x | 0.68x | 8.76x | 0.33x |
| `vec4 normalize` | 36.68 ns | 8.55 us | 6.40 us | 6.12 us | 759.32 ns | 22.44 us | 233.17x | 1.40x | 11.26x | 0.38x |
| `vec4 add` | 7.58 ns | 1.22 us | 1.84 us | 212.34 ns | 178.21 ns | 5.40 us | 160.94x | 5.75x | 6.85x | 0.23x |
| `vec4 add scalar` | 6.86 ns | 1.04 us | 1.75 us | 224.69 ns | 182.28 ns | 5.23 us | 152.01x | 4.64x | 5.72x | 0.20x |
| `vec4 sub` | 5.21 ns | 1.28 us | 2.28 us | 218.67 ns | 180.00 ns | 9.68 us | 245.97x | 5.86x | 7.12x | 0.13x |
| `vec4 sub scalar` | 4.58 ns | 1.01 us | 1.47 us | 211.43 ns | 173.66 ns | 9.38 us | 219.68x | 4.76x | 5.79x | 0.11x |
| `vec4 neg` | 4.91 ns | 202.81 ns | 203.79 ns | 79.04 ns | 66.75 ns | 4.06 us | 41.27x | 2.57x | 3.04x | 0.05x |
| `vec4 mul scalar` | 7.35 ns | 1.11 us | 2.06 us | 221.68 ns | 158.30 ns | 5.73 us | 150.83x | 5.00x | 7.00x | 0.19x |
| `vec4 div scalar` | 13.21 ns | 3.00 us | 1.56 us | 539.70 ns | 228.63 ns | 10.11 us | 226.94x | 5.56x | 13.12x | 0.30x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.61 ns | 698.42 ns | 2.08 us | 1.95 us | 855.34 ns | 22.36 us | 35.62x | 0.36x | 0.82x | 0.03x |
| `mat3 inverse` | 87.32 ns | 14.30 us | 7.89 us | 3.22 us | 2.51 us | 155.79 us | 163.81x | 4.44x | 5.71x | 0.09x |
| `mat3 mul mat3` | 67.64 ns | 3.39 us | 7.77 us | 2.94 us | 2.35 us | 126.88 us | 50.16x | 1.15x | 1.44x | 0.03x |
| `mat3 transform vec3` | 27.63 ns | 1.57 us | 4.87 us | 1.07 us | 880.86 ns | 21.94 us | 56.69x | 1.46x | 1.78x | 0.07x |
| `mat4 determinant` | 44.29 ns | 2.15 us | 1.51 us | 4.71 us | 4.13 us | 97.19 us | 48.64x | 0.46x | 0.52x | 0.02x |
| `mat4 inverse` | 170.12 ns | 24.35 us | 8.45 us | 11.27 us | 9.19 us | 367.60 us | 143.14x | 2.16x | 2.65x | 0.07x |
| `mat4 mul mat4` | 138.68 ns | 5.81 us | 4.81 us | 6.43 us | 5.43 us | 146.58 us | 41.91x | 0.90x | 1.07x | 0.04x |
| `mat4 transform vec4` | 47.51 ns | 2.59 us | 2.40 us | 1.94 us | 1.67 us | 36.00 us | 54.45x | 1.34x | 1.55x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.82 ns | 1.30 us | 4.36 us | 216.40 ns | 237.13 ns | 2.15 us | 36.30x | 6.01x | 5.48x | 0.60x |
| `mat3 zero` | 16.05 ns | 515.07 ns | 546.89 ns | 251.45 ns | 210.94 ns | 11.94 ns | 32.09x | 2.05x | 2.44x | 43.13x |
| `mat3 identity` | 9.64 ns | 610.86 ns | 629.34 ns | 309.49 ns | 239.93 ns | 143.21 ns | 63.40x | 1.97x | 2.55x | 4.27x |
| `mat3 transpose` | 8.87 ns | 678.73 ns | 660.36 ns | 240.88 ns | 210.28 ns | 117.93 ns | 76.54x | 2.82x | 3.23x | 5.76x |
| `mat3 reciprocal` | 91.44 ns | 14.65 us | 24.48 us | 3.01 us | 3.43 us | 84.20 us | 160.25x | 4.87x | 4.27x | 0.17x |
| `mat3 reciprocal checked` | 89.14 ns | 14.32 us | 24.44 us | 2.96 us | 2.28 us | 83.51 us | 160.66x | 4.84x | 6.27x | 0.17x |
| `mat3 inverse checked` | 87.21 ns | 15.25 us | 25.24 us | 2.97 us | 2.29 us | 82.91 us | 174.90x | 5.14x | 6.65x | 0.18x |
| `mat3 inverse checked abort` | 94.56 ns | 15.44 us | 24.31 us | 3.09 us | 2.30 us | 85.02 us | 163.32x | 5.00x | 6.70x | 0.18x |
| `mat3 powi` | 111.34 ns | 6.31 us | 41.54 us | 6.85 us | 6.13 us | 154.56 us | 56.63x | 0.92x | 1.03x | 0.04x |
| `mat3 powi checked` | 108.14 ns | 4.42 us | 37.85 us | 6.81 us | 6.12 us | 154.18 us | 40.91x | 0.65x | 0.72x | 0.03x |
| `mat3 powi checked abort` | 108.77 ns | 4.41 us | 38.24 us | 7.09 us | 6.11 us | 152.66 us | 40.53x | 0.62x | 0.72x | 0.03x |
| `mat3 div scalar checked` | 23.38 ns | 4.23 us | 5.45 us | 1.46 us | 820.76 ns | 22.26 us | 180.83x | 2.90x | 5.15x | 0.19x |
| `mat3 div scalar checked abort` | 28.33 ns | 4.23 us | 5.50 us | 1.45 us | 814.93 ns | 22.25 us | 149.28x | 2.91x | 5.19x | 0.19x |
| `mat3 div matrix checked` | 135.31 ns | 18.07 us | 43.46 us | 5.49 us | 4.44 us | 162.70 us | 133.52x | 3.29x | 4.07x | 0.11x |
| `mat3 div matrix checked abort` | 146.04 ns | 17.92 us | 43.14 us | 9.61 us | 4.48 us | 162.16 us | 122.69x | 1.86x | 4.00x | 0.11x |
| `mat3 add` | 14.62 ns | 3.15 us | 5.21 us | 509.74 ns | 494.00 ns | 11.86 us | 215.25x | 6.18x | 6.37x | 0.27x |
| `mat3 add scalar` | 12.02 ns | 2.52 us | 4.64 us | 800.03 ns | 727.77 ns | 12.20 us | 209.94x | 3.15x | 3.47x | 0.21x |
| `mat3 sub` | 13.15 ns | 3.45 us | 7.06 us | 533.98 ns | 528.12 ns | 21.70 us | 262.10x | 6.45x | 6.53x | 0.16x |
| `mat3 sub scalar` | 10.83 ns | 2.57 us | 4.48 us | 800.31 ns | 718.45 ns | 21.73 us | 237.50x | 3.21x | 3.58x | 0.12x |
| `mat3 neg` | 10.28 ns | 603.38 ns | 621.03 ns | 486.88 ns | 452.04 ns | 8.60 us | 58.71x | 1.24x | 1.33x | 0.07x |
| `mat3 mul scalar` | 13.69 ns | 1.81 us | 5.74 us | 802.33 ns | 669.15 ns | 12.26 us | 132.56x | 2.26x | 2.71x | 0.15x |
| `mat3 div scalar` | 24.23 ns | 4.18 us | 5.48 us | 1.48 us | 828.71 ns | 22.44 us | 172.70x | 2.82x | 5.05x | 0.19x |
| `mat3 div matrix` | 135.61 ns | 18.50 us | 44.38 us | 5.49 us | 4.46 us | 161.12 us | 136.40x | 3.37x | 4.15x | 0.11x |
| `mat3 bitxor` | 111.72 ns | 4.41 us | 38.26 us | 6.92 us | 6.25 us | 151.95 us | 39.50x | 0.64x | 0.71x | 0.03x |
| `mat4 zero` | 11.56 ns | 903.44 ns | 784.35 ns | 481.59 ns | 355.93 ns | 14.85 ns | 78.16x | 1.88x | 2.54x | 60.84x |
| `mat4 identity` | 10.81 ns | 1.08 us | 966.15 ns | 568.91 ns | 420.98 ns | 215.28 ns | 99.62x | 1.89x | 2.56x | 5.00x |
| `mat4 transpose` | 9.34 ns | 1.11 us | 1.22 us | 433.73 ns | 367.07 ns | 180.78 ns | 118.98x | 2.56x | 3.03x | 6.15x |
| `mat4 reciprocal` | 163.30 ns | 25.72 us | 42.63 us | 10.94 us | 9.00 us | 349.92 us | 157.52x | 2.35x | 2.86x | 0.07x |
| `mat4 reciprocal checked` | 169.92 ns | 24.20 us | 40.75 us | 10.95 us | 8.91 us | 345.90 us | 142.43x | 2.21x | 2.72x | 0.07x |
| `mat4 powi` | 246.39 ns | 9.74 us | 65.47 us | 15.62 us | 13.94 us | 350.15 us | 39.55x | 0.62x | 0.70x | 0.03x |
| `mat4 powi checked` | 240.48 ns | 8.78 us | 51.70 us | 15.83 us | 14.67 us | 354.64 us | 36.53x | 0.55x | 0.60x | 0.02x |
| `mat4 add` | 52.53 ns | 5.17 us | 6.17 us | 909.57 ns | 868.69 ns | 19.48 us | 98.36x | 5.68x | 5.95x | 0.27x |
| `mat4 add scalar` | 20.61 ns | 4.10 us | 6.67 us | 1.43 us | 1.19 us | 20.99 us | 198.86x | 2.87x | 3.45x | 0.20x |
| `mat4 sub` | 38.80 ns | 5.46 us | 7.37 us | 971.81 ns | 909.52 ns | 36.83 us | 140.77x | 5.62x | 6.01x | 0.15x |
| `mat4 sub scalar` | 15.28 ns | 4.16 us | 6.53 us | 1.43 us | 1.68 us | 37.68 us | 271.98x | 2.90x | 2.48x | 0.11x |
| `mat4 neg` | 14.60 ns | 994.85 ns | 1.16 us | 884.95 ns | 744.48 ns | 14.12 us | 68.15x | 1.12x | 1.34x | 0.07x |
| `mat4 mul scalar` | 24.65 ns | 2.89 us | 7.32 us | 1.41 us | 1.12 us | 20.51 us | 117.31x | 2.05x | 2.57x | 0.14x |
| `mat4 div scalar` | 31.81 ns | 7.23 us | 9.63 us | 4.00 us | 1.37 us | 37.83 us | 227.45x | 1.81x | 5.29x | 0.19x |
| `mat4 div matrix` | 202.88 ns | 29.15 us | 78.14 us | 17.00 us | 14.43 us | 543.05 us | 143.68x | 1.72x | 2.02x | 0.05x |
| `mat4 bitxor` | 241.39 ns | 8.73 us | 51.60 us | 16.45 us | 13.97 us | 352.55 us | 36.16x | 0.53x | 0.62x | 0.02x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.63 ns | 189.12 ns | 291.40 ns | - | - | 1.38 us | 52.13x | - | - | 0.14x |
| `scalar add ref_owned` | 12.15 ns | 181.95 ns | 276.00 ns | - | - | 1.34 us | 14.97x | - | - | 0.14x |
| `scalar add refs` | 5.36 ns | 168.36 ns | 262.16 ns | - | - | 1.33 us | 31.40x | - | - | 0.13x |
| `scalar add owned_ref_with_clone` | 9.00 ns | 206.20 ns | 297.87 ns | - | - | - | 22.90x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.47 ns | 193.27 ns | 294.01 ns | - | - | - | 35.32x | - | - | - |
| `scalar sub owned_ref` | 3.97 ns | 228.17 ns | 463.88 ns | - | - | 2.57 us | 57.52x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.34 ns | 220.63 ns | 450.48 ns | - | - | 2.54 us | 17.88x | - | - | 0.09x |
| `scalar sub refs` | 5.81 ns | 204.86 ns | 436.88 ns | - | - | 2.51 us | 35.23x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.10 ns | 244.34 ns | 472.01 ns | - | - | - | 26.85x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.70 ns | 386.18 ns | 796.64 ns | - | - | - | 67.71x | - | - | - |
| `scalar mul owned_ref` | 4.29 ns | 103.23 ns | 363.55 ns | - | - | 1.56 us | 24.09x | - | - | 0.07x |
| `scalar mul ref_owned` | 13.34 ns | 95.65 ns | 375.66 ns | - | - | 1.57 us | 7.17x | - | - | 0.06x |
| `scalar mul refs` | 6.14 ns | 82.66 ns | 347.91 ns | - | - | 1.57 us | 13.47x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.97 ns | 110.76 ns | 381.65 ns | - | - | - | 11.11x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.52 ns | 109.11 ns | 381.73 ns | - | - | - | 16.74x | - | - | - |
| `scalar div owned_ref` | 6.21 ns | 260.87 ns | 316.69 ns | - | - | 2.58 us | 42.01x | - | - | 0.10x |
| `scalar div ref_owned` | 17.79 ns | 259.50 ns | 317.03 ns | - | - | 2.61 us | 14.59x | - | - | 0.10x |
| `scalar div refs` | 7.08 ns | 247.71 ns | 304.54 ns | - | - | 2.60 us | 35.01x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.79 ns | 280.25 ns | 335.37 ns | - | - | - | 17.75x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.66 ns | 279.19 ns | 342.05 ns | - | - | - | 17.82x | - | - | - |
| `vec3 add refs` | 6.41 ns | 636.78 ns | 1.41 us | - | - | 4.06 us | 99.38x | - | - | 0.16x |
| `vec3 sub refs` | 6.37 ns | 775.46 ns | 2.21 us | - | - | 7.57 us | 121.79x | - | - | 0.10x |
| `vec3 neg ref` | 3.30 ns | 161.82 ns | 159.48 ns | - | - | 3.17 us | 49.00x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.78 ns | 766.65 ns | 1.45 us | - | - | 3.97 us | 113.01x | - | - | 0.19x |
| `vec3 sub_scalar_ref` | 6.50 ns | 807.43 ns | 1.27 us | - | - | 7.10 us | 124.26x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 7.47 ns | 515.57 ns | 1.98 us | - | - | 4.52 us | 69.04x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.20 ns | 1.29 us | 1.56 us | - | - | 7.66 us | 156.90x | - | - | 0.17x |
| `vec4 add refs` | 6.76 ns | 789.52 ns | 1.91 us | - | - | 5.33 us | 116.84x | - | - | 0.15x |
| `vec4 sub refs` | 3.26 ns | 862.38 ns | 1.83 us | - | - | 9.90 us | 264.60x | - | - | 0.09x |
| `vec4 neg ref` | 4.29 ns | 234.53 ns | 239.01 ns | - | - | 4.09 us | 54.64x | - | - | 0.06x |
| `vec4 add_scalar_ref` | 6.98 ns | 1.02 us | 1.72 us | - | - | 8.77 us | 146.69x | - | - | 0.12x |
| `vec4 sub_scalar_ref` | 4.26 ns | 1.05 us | 1.51 us | - | - | 9.63 us | 246.81x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.54 ns | 638.10 ns | 2.05 us | - | - | 5.85 us | 84.59x | - | - | 0.11x |
| `vec4 div_scalar_ref` | 12.94 ns | 1.66 us | 1.49 us | - | - | 10.22 us | 128.07x | - | - | 0.16x |
| `mat3 add refs` | 11.61 ns | 1.75 us | 3.79 us | - | - | 11.91 us | 150.66x | - | - | 0.15x |
| `mat3 sub refs` | 10.60 ns | 2.02 us | 5.64 us | - | - | 21.41 us | 190.31x | - | - | 0.09x |
| `mat3 mul refs` | 55.01 ns | 2.26 us | 19.87 us | - | - | 64.11 us | 41.12x | - | - | 0.04x |
| `mat3 div refs` | 130.61 ns | 16.79 us | 42.25 us | - | - | 163.19 us | 128.53x | - | - | 0.10x |
| `mat3 neg ref` | 9.99 ns | 484.75 ns | 481.09 ns | - | - | 8.66 us | 48.51x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 44.46 ns | 2.47 us | 4.60 us | - | - | 12.89 us | 55.61x | - | - | 0.19x |
| `mat3 sub_scalar_ref` | 11.73 ns | 2.61 us | 4.44 us | - | - | 21.81 us | 222.46x | - | - | 0.12x |
| `mat3 mul_scalar_ref` | 47.99 ns | 1.78 us | 5.61 us | - | - | 12.37 us | 37.04x | - | - | 0.14x |
| `mat3 div_scalar_ref` | 21.56 ns | 4.26 us | 5.64 us | - | - | 22.52 us | 197.37x | - | - | 0.19x |
| `mat4 add refs` | 16.99 ns | 2.83 us | 3.59 us | - | - | 19.81 us | 166.79x | - | - | 0.14x |
| `mat4 sub refs` | 16.76 ns | 3.12 us | 7.37 us | - | - | 36.86 us | 186.19x | - | - | 0.08x |
| `mat4 mul refs` | 125.09 ns | 6.74 us | 19.19 us | - | - | 144.46 us | 53.89x | - | - | 0.05x |
| `mat4 div refs` | 180.50 ns | 26.60 us | 60.65 us | - | - | 530.31 us | 147.35x | - | - | 0.05x |
| `mat4 neg ref` | 12.28 ns | 728.64 ns | 720.77 ns | - | - | 14.01 us | 59.34x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 49.81 ns | 4.28 us | 6.64 us | - | - | 20.99 us | 85.89x | - | - | 0.20x |
| `mat4 sub_scalar_ref` | 15.39 ns | 4.39 us | 6.84 us | - | - | 37.39 us | 285.09x | - | - | 0.12x |
| `mat4 mul_scalar_ref` | 53.50 ns | 2.87 us | 7.63 us | - | - | 20.65 us | 53.64x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 25.26 ns | 7.52 us | 8.74 us | - | - | 37.60 us | 297.89x | - | - | 0.20x |
| `mat3 transform_vec refs` | 26.66 ns | 963.93 ns | 8.45 us | - | - | 20.38 us | 36.16x | - | - | 0.05x |
| `mat4 transform_vec refs` | 43.41 ns | 1.50 us | 6.02 us | - | - | 36.32 us | 34.50x | - | - | 0.04x |
| `complex add refs` | 7.72 ns | 331.67 ns | 697.48 ns | - | - | 2.63 us | 42.94x | - | - | 0.13x |
| `complex sub refs` | 7.97 ns | 381.16 ns | 990.74 ns | - | - | 4.92 us | 47.84x | - | - | 0.08x |
| `complex mul refs` | 8.12 ns | 768.08 ns | 3.35 us | - | - | 10.47 us | 94.63x | - | - | 0.07x |
| `complex div refs` | 16.05 ns | 2.18 us | 4.47 us | - | - | 21.81 us | 135.67x | - | - | 0.10x |
| `complex neg ref` | 2.36 ns | 75.06 ns | 72.34 ns | - | - | 2.13 us | 31.84x | - | - | 0.04x |
| `complex div_real_ref` | 7.27 ns | 618.74 ns | 591.66 ns | - | - | 5.34 us | 85.05x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.60 us |
| `astro sin 160` | 13.94 us |
| `astro sin 192` | 14.01 us |
| `astro sin 256` | 16.28 us |
