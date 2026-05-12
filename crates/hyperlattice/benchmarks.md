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
| `sin 0.1` | 10.60 ns | 141.26 ns | 145.81 ns | 10.94 us | 766.63 ns | 1.82 us | 13.33x | 0.01x | 0.18x | 0.08x |
| `cos 0.1` | 11.27 ns | 141.77 ns | 144.72 ns | 10.50 us | 507.61 ns | 1.64 us | 12.58x | 0.01x | 0.28x | 0.09x |
| `sin 1.23456789` | 11.43 ns | 196.03 ns | 187.16 ns | 12.56 us | 810.53 ns | 1.81 us | 17.15x | 0.02x | 0.24x | 0.11x |
| `cos 1.23456789` | 11.74 ns | 188.78 ns | 195.05 ns | 10.63 us | 640.84 ns | 1.61 us | 16.09x | 0.02x | 0.29x | 0.12x |
| `sin 1e6` | 12.27 ns | 87.81 ns | 98.30 ns | 15.90 us | 1.10 us | 2.02 us | 7.16x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 12.01 ns | 87.84 ns | 95.90 ns | 13.64 us | 831.36 ns | 1.81 us | 7.31x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 63.87 ns | 88.58 ns | 94.82 ns | 18.38 us | 2.88 us | 3.58 us | 1.39x | 0.00x | 0.03x | 0.02x |
| `cos 1e30` | 66.41 ns | 88.03 ns | 106.63 ns | 15.34 us | 970.70 ns | 3.06 us | 1.33x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 11.37 ns | 141.17 ns | 377.43 ns | 12.13 us | 750.50 ns | 1.85 us | 12.42x | 0.01x | 0.19x | 0.08x |
| `cos pi_7` | 11.47 ns | 142.40 ns | 860.15 ns | 10.76 us | 539.74 ns | 1.68 us | 12.41x | 0.01x | 0.26x | 0.08x |
| `sin 1000pi_eps` | 11.50 ns | 87.79 ns | 981.05 ns | 15.96 us | 2.36 us | 2.82 us | 7.63x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.16 ns | 87.62 ns | 829.48 ns | 13.77 us | 569.51 ns | 1.66 us | 7.21x | 0.01x | 0.15x | 0.05x |
| `asin 0.5` | 10.91 ns | 132.76 ns | 138.52 ns | 49.66 us | 2.91 us | 13.25 us | 12.17x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.06 ns | 392.69 ns | 423.04 ns | 59.03 us | 2.91 us | 12.96 us | 35.49x | 0.01x | 0.14x | 0.03x |
| `atanh 0.5` | 14.03 ns | 859.62 ns | 903.57 ns | 33.93 us | 1.75 us | 13.00 us | 61.27x | 0.03x | 0.49x | 0.07x |
| `asin neg_0.999999` | 13.75 ns | 551.97 ns | 542.80 ns | 13.51 us | 2.52 us | 12.71 us | 40.15x | 0.04x | 0.22x | 0.04x |
| `acos neg_0.999999` | 14.98 ns | 490.63 ns | 482.84 ns | 18.09 us | 2.70 us | 12.83 us | 32.76x | 0.03x | 0.18x | 0.04x |
| `atanh neg_0.999999` | 13.99 ns | 198.11 ns | 193.43 ns | 35.72 us | 1.56 us | 12.76 us | 14.16x | 0.01x | 0.13x | 0.02x |
| `asin 0.999999` | 13.45 ns | 516.26 ns | 759.35 ns | 13.41 us | 2.53 us | 12.83 us | 38.39x | 0.04x | 0.20x | 0.04x |
| `acos 0.999999` | 13.94 ns | 290.16 ns | 533.95 ns | 18.26 us | 2.72 us | 12.72 us | 20.81x | 0.02x | 0.11x | 0.02x |
| `atanh 0.999999` | 14.04 ns | 204.74 ns | 199.21 ns | 30.96 us | 1.56 us | 12.48 us | 14.58x | 0.01x | 0.13x | 0.02x |
| `asin 1e-12` | 9.13 ns | 265.84 ns | 516.97 ns | 7.90 us | 1.42 us | 14.91 us | 29.13x | 0.03x | 0.19x | 0.02x |
| `acos 1e-12` | 9.80 ns | 460.51 ns | 674.04 ns | 9.60 us | 1.43 us | 14.92 us | 46.99x | 0.05x | 0.32x | 0.03x |
| `atanh 1e-12` | 9.40 ns | 238.00 ns | 218.27 ns | 35.70 us | 169.37 ns | 19.75 us | 25.33x | 0.01x | 1.41x | 0.01x |
| `atan 0.5` | 14.30 ns | 160.74 ns | 163.33 ns | 35.23 us | 2.92 us | 17.47 us | 11.24x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.07 ns | 257.59 ns | 263.50 ns | 38.52 us | 1.59 us | 7.32 us | 9.88x | 0.01x | 0.16x | 0.04x |
| `atan neg_1e-12` | 13.80 ns | 271.48 ns | 272.07 ns | 1.60 us | 1.18 us | 15.10 us | 19.68x | 0.17x | 0.23x | 0.02x |
| `asinh neg_1e-12` | 15.53 ns | 457.46 ns | 374.46 ns | 41.17 us | 8.47 us | 11.64 us | 29.46x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 14.93 ns | 153.16 ns | 160.08 ns | 2.84 us | 1.50 us | 17.90 us | 10.26x | 0.05x | 0.10x | 0.01x |
| `asinh 1e6` | 25.76 ns | 249.53 ns | 258.71 ns | 36.36 us | 1.64 us | 7.09 us | 9.69x | 0.01x | 0.15x | 0.04x |
| `atan neg_1e6` | 14.86 ns | 261.00 ns | 268.42 ns | 2.89 us | 1.49 us | 17.62 us | 17.56x | 0.09x | 0.17x | 0.01x |
| `asinh neg_1e6` | 25.77 ns | 361.99 ns | 371.90 ns | 36.61 us | 1.62 us | 6.91 us | 14.05x | 0.01x | 0.22x | 0.05x |
| `acosh 9` | 11.96 ns | 157.89 ns | 158.53 ns | 41.93 us | 1.60 us | 9.62 us | 13.20x | 0.00x | 0.10x | 0.02x |
| `acosh 1_plus_1e-12` | 12.88 ns | 233.61 ns | 233.44 ns | 41.23 us | 8.27 us | 11.21 us | 18.13x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.05 ns | 160.37 ns | 160.34 ns | 37.08 us | 1.58 us | 9.77 us | 13.30x | 0.00x | 0.10x | 0.02x |
| `acosh e` | 12.12 ns | 237.96 ns | 2.29 us | 41.50 us | 1.63 us | 9.50 us | 19.64x | 0.01x | 0.15x | 0.03x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.87 ns | 18.53 ns | 24.89 ns | 15.55 ns | 0.92 ns | 40.29x | 0.76x | 1.21x | 20.48x |
| `one` | 0.47 ns | 24.79 ns | 23.76 ns | 41.31 ns | 31.10 ns | 29.21 ns | 52.62x | 0.60x | 0.80x | 0.85x |
| `e` | 0.47 ns | 88.56 ns | 87.26 ns | 53.49 ns | 1.05 us | 221.58 ns | 188.46x | 1.66x | 0.08x | 0.40x |
| `pi` | 0.47 ns | 48.77 ns | 49.81 ns | 42.57 ns | 48.66 ns | 220.98 ns | 104.42x | 1.15x | 1.00x | 0.22x |
| `tau` | 0.47 ns | 48.64 ns | 50.06 ns | 114.87 ns | 100.98 ns | 1.83 us | 103.79x | 0.42x | 0.48x | 0.03x |
| `add` | 5.17 ns | 208.08 ns | 313.55 ns | 54.26 ns | 42.65 ns | 1.25 us | 40.27x | 3.84x | 4.88x | 0.17x |
| `sub` | 5.04 ns | 213.95 ns | 375.92 ns | 56.23 ns | 45.37 ns | 2.35 us | 42.47x | 3.81x | 4.72x | 0.09x |
| `neg` | 5.02 ns | 39.03 ns | 38.76 ns | 23.06 ns | 21.21 ns | 1.04 us | 7.78x | 1.69x | 1.84x | 0.04x |
| `mul` | 5.38 ns | 134.78 ns | 402.51 ns | 57.38 ns | 45.50 ns | 1.50 us | 25.07x | 2.35x | 2.96x | 0.09x |
| `div` | 8.87 ns | 307.35 ns | 359.64 ns | 134.12 ns | 62.13 ns | 2.45 us | 34.66x | 2.29x | 4.95x | 0.13x |
| `reciprocal` | 8.37 ns | 81.75 ns | 82.64 ns | 157.91 ns | 58.97 ns | 1.48 us | 9.77x | 0.52x | 1.39x | 0.06x |
| `reciprocal checked` | 8.79 ns | 77.86 ns | 81.00 ns | 160.41 ns | 59.43 ns | 1.49 us | 8.86x | 0.49x | 1.31x | 0.05x |
| `reciprocal checked abort` | 24.09 ns | 87.95 ns | 88.10 ns | 158.05 ns | 57.57 ns | 1.49 us | 3.65x | 0.56x | 1.53x | 0.06x |
| `pow` | 30.21 ns | 6.53 us | 5.92 us | 54.74 us | 2.82 us | 2.30 us | 216.17x | 0.12x | 2.32x | 2.84x |
| `powi` | 6.25 ns | 375.24 ns | 2.70 us | 286.48 ns | 83.07 ns | 1.54 us | 60.00x | 1.31x | 4.52x | 0.24x |
| `exp` | 9.96 ns | 231.42 ns | 233.80 ns | 13.81 us | 919.39 ns | 1.81 us | 23.24x | 0.02x | 0.25x | 0.13x |
| `ln` | 10.52 ns | 1.45 us | 1.40 us | 29.18 us | 1.27 us | 1.76 us | 138.23x | 0.05x | 1.14x | 0.83x |
| `log10` | 13.13 ns | 1.66 us | 1.61 us | 34.90 us | 2.68 us | 6.48 us | 126.58x | 0.05x | 0.62x | 0.26x |
| `log10 abort` | 16.56 ns | 1.64 us | 1.61 us | 34.09 us | 2.68 us | 6.51 us | 98.93x | 0.05x | 0.61x | 0.25x |
| `sqrt` | 19.56 ns | 1.48 us | 1.66 us | 4.84 us | 94.67 ns | 1.40 us | 75.67x | 0.31x | 15.63x | 1.06x |
| `sin` | 14.53 ns | 123.18 ns | 120.21 ns | 13.75 us | 1.23 us | 2.18 us | 8.48x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.02 ns | 123.97 ns | 120.12 ns | 11.88 us | 620.72 ns | 1.72 us | 6.88x | 0.01x | 0.20x | 0.07x |
| `tan` | 24.11 ns | 166.94 ns | 172.51 ns | 28.54 us | 1.54 us | 6.40 us | 6.92x | 0.01x | 0.11x | 0.03x |
| `sinh` | 17.93 ns | 3.54 us | 3.56 us | 3.24 us | 1.12 us | 10.49 us | 197.24x | 1.09x | 3.15x | 0.34x |
| `cosh` | 17.84 ns | 3.47 us | 3.44 us | 7.69 us | 1.06 us | 9.28 us | 194.71x | 0.45x | 3.28x | 0.37x |
| `tanh` | 22.56 ns | 7.14 us | 7.19 us | 3.31 us | 1.17 us | 22.16 us | 316.45x | 2.16x | 6.12x | 0.32x |
| `asin` | 12.88 ns | 380.98 ns | 589.06 ns | 21.04 us | 2.36 us | 13.37 us | 29.57x | 0.02x | 0.16x | 0.03x |
| `asin abort` | 16.81 ns | 385.59 ns | 596.91 ns | 21.02 us | 2.35 us | 13.52 us | 22.93x | 0.02x | 0.16x | 0.03x |
| `acos` | 14.52 ns | 444.15 ns | 650.21 ns | 26.17 us | 2.45 us | 13.55 us | 30.59x | 0.02x | 0.18x | 0.03x |
| `acos abort` | 17.27 ns | 452.58 ns | 652.56 ns | 26.13 us | 2.45 us | 13.36 us | 26.20x | 0.02x | 0.18x | 0.03x |
| `atan` | 14.54 ns | 165.70 ns | 163.00 ns | 18.15 us | 2.20 us | 18.19 us | 11.39x | 0.01x | 0.08x | 0.01x |
| `atan abort` | 21.00 ns | 164.23 ns | 158.34 ns | 18.00 us | 2.21 us | 18.26 us | 7.82x | 0.01x | 0.07x | 0.01x |
| `asinh` | 42.46 ns | 266.48 ns | 264.60 ns | 37.63 us | 1.61 us | 7.42 us | 6.28x | 0.01x | 0.17x | 0.04x |
| `asinh abort` | 30.68 ns | 259.46 ns | 269.98 ns | 37.46 us | 1.61 us | 7.38 us | 8.46x | 0.01x | 0.16x | 0.04x |
| `acosh` | 12.29 ns | 197.91 ns | 199.53 ns | 39.27 us | 3.23 us | 10.11 us | 16.11x | 0.01x | 0.06x | 0.02x |
| `acosh abort` | 15.27 ns | 198.91 ns | 200.54 ns | 39.25 us | 3.23 us | 10.10 us | 13.03x | 0.01x | 0.06x | 0.02x |
| `atanh` | 14.12 ns | 387.44 ns | 538.74 ns | 33.43 us | 1.22 us | 14.52 us | 27.43x | 0.01x | 0.32x | 0.03x |
| `atanh abort` | 17.31 ns | 384.66 ns | 539.63 ns | 34.29 us | 1.24 us | 14.50 us | 22.22x | 0.01x | 0.31x | 0.03x |
| `zero status` | 1.22 ns | 1.04 ns | 1.11 ns | 1.03 ns | 6.87 ns | 7.85 ns | 0.85x | 1.01x | 0.15x | 0.13x |
| `zero status abort` | 1.38 ns | 1.14 ns | 1.08 ns | 1.02 ns | 6.76 ns | 7.87 ns | 0.83x | 1.12x | 0.17x | 0.15x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.91 ns | 37.55 ns | 38.13 ns | 50.51 ns | - | 1.87 ns | 41.19x | 0.74x | - | 20.11x |
| `one` | 5.25 ns | 43.38 ns | 43.67 ns | 65.58 ns | - | 32.72 ns | 8.27x | 0.66x | - | 1.33x |
| `i` | 0.91 ns | 44.93 ns | 45.70 ns | 66.19 ns | - | 29.99 ns | 49.14x | 0.68x | - | 1.50x |
| `free i` | 0.92 ns | 43.67 ns | 45.61 ns | 66.39 ns | - | 29.56 ns | 47.66x | 0.66x | - | 1.48x |
| `conjugate` | 2.07 ns | 88.76 ns | 88.21 ns | 40.85 ns | - | 1.07 us | 42.81x | 2.17x | - | 0.08x |
| `norm squared` | 5.61 ns | 157.26 ns | 555.89 ns | 153.65 ns | - | 4.20 us | 28.01x | 1.02x | - | 0.04x |
| `reciprocal` | 18.31 ns | 1.71 us | 2.86 us | 449.41 ns | - | 10.55 us | 93.15x | 3.80x | - | 0.16x |
| `reciprocal checked` | 14.87 ns | 1.71 us | 2.90 us | 438.31 ns | - | 10.56 us | 115.02x | 3.90x | - | 0.16x |
| `powi` | 17.52 ns | 2.10 us | 13.47 us | 1.43 us | - | 43.04 us | 119.95x | 1.46x | - | 0.05x |
| `powi checked` | 17.61 ns | 2.09 us | 13.54 us | 1.45 us | - | 42.39 us | 118.72x | 1.44x | - | 0.05x |
| `div checked` | 22.16 ns | 2.66 us | 6.35 us | 812.56 ns | - | 21.14 us | 120.07x | 3.27x | - | 0.13x |
| `div real checked` | 9.18 ns | 665.49 ns | 639.45 ns | 266.84 ns | - | 5.08 us | 72.49x | 2.49x | - | 0.13x |
| `from scalar` | 1.39 ns | 61.14 ns | 60.22 ns | 44.95 ns | - | 10.40 ns | 44.05x | 1.36x | - | 5.88x |
| `add` | 5.85 ns | 477.57 ns | 853.03 ns | 110.24 ns | - | 2.60 us | 81.62x | 4.33x | - | 0.18x |
| `sub` | 5.67 ns | 478.25 ns | 883.43 ns | 115.16 ns | - | 4.66 us | 84.41x | 4.15x | - | 0.10x |
| `neg` | 2.55 ns | 90.73 ns | 89.68 ns | 41.86 ns | - | 2.12 us | 35.62x | 2.17x | - | 0.04x |
| `mul` | 7.28 ns | 853.68 ns | 3.18 us | 309.87 ns | - | 9.84 us | 117.29x | 2.75x | - | 0.09x |
| `div` | 17.19 ns | 2.72 us | 6.24 us | 774.69 ns | - | 21.62 us | 158.00x | 3.51x | - | 0.13x |
| `div real` | 9.84 ns | 669.39 ns | 635.28 ns | 260.77 ns | - | 5.11 us | 68.05x | 2.57x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.76 ns | 211.26 ns | 2.35 us | 296.10 ns | 253.33 ns | 6.92 us | 24.11x | 0.71x | 0.83x | 0.03x |
| `vec3 magnitude` | 12.98 ns | 3.95 us | 5.44 us | 5.35 us | 336.87 ns | 8.40 us | 304.15x | 0.74x | 11.72x | 0.47x |
| `vec3 normalize` | 32.03 ns | 8.04 us | 11.11 us | 5.94 us | 583.70 ns | 16.26 us | 250.97x | 1.35x | 13.77x | 0.49x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.06 ns | 292.59 ns | 1.45 us | 70.98 ns | 55.48 ns | 773.19 ns | 95.66x | 4.12x | 5.27x | 0.38x |
| `vec3 zero` | 1.39 ns | 135.92 ns | 135.29 ns | 58.76 ns | 30.44 ns | 2.76 ns | 97.58x | 2.31x | 4.46x | 49.30x |
| `vec3 dot abort` | 8.42 ns | 213.62 ns | 883.37 ns | 248.07 ns | 205.52 ns | 6.89 us | 25.36x | 0.86x | 1.04x | 0.03x |
| `vec3 magnitude abort` | 17.31 ns | 3.91 us | 3.12 us | 5.24 us | 313.81 ns | 8.43 us | 225.69x | 0.75x | 12.45x | 0.46x |
| `vec3 normalize checked` | 32.38 ns | 8.08 us | 7.06 us | 5.79 us | 522.78 ns | 16.31 us | 249.39x | 1.39x | 15.45x | 0.50x |
| `vec3 normalize checked abort` | 29.01 ns | 8.09 us | 7.17 us | 5.81 us | 521.15 ns | 16.28 us | 278.96x | 1.39x | 15.53x | 0.50x |
| `vec3 div scalar checked` | 10.39 ns | 1.28 us | 1.55 us | 390.94 ns | - | - | 123.32x | 3.28x | - | - |
| `vec3 div scalar checked abort` | 17.72 ns | 1.27 us | 1.55 us | 390.03 ns | - | - | 71.88x | 3.26x | - | - |
| `vec3 add` | 6.51 ns | 944.91 ns | 1.69 us | 148.85 ns | 124.30 ns | 3.78 us | 145.24x | 6.35x | 7.60x | 0.25x |
| `vec3 add scalar` | 6.29 ns | 767.80 ns | 1.45 us | 147.74 ns | 130.65 ns | 3.66 us | 121.98x | 5.20x | 5.88x | 0.21x |
| `vec3 sub` | 6.49 ns | 942.48 ns | 1.74 us | 163.52 ns | 134.58 ns | 7.04 us | 145.13x | 5.76x | 7.00x | 0.13x |
| `vec3 sub scalar` | 6.28 ns | 766.87 ns | 1.24 us | 143.57 ns | 120.84 ns | 6.74 us | 122.03x | 5.34x | 6.35x | 0.11x |
| `vec3 neg` | 3.74 ns | 168.63 ns | 163.41 ns | 55.15 ns | 45.66 ns | 3.02 us | 45.10x | 3.06x | 3.69x | 0.06x |
| `vec3 mul scalar` | 6.84 ns | 524.34 ns | 2.01 us | 166.41 ns | 117.72 ns | 4.24 us | 76.71x | 3.15x | 4.45x | 0.12x |
| `vec3 div scalar` | 18.35 ns | 1.27 us | 1.51 us | 393.57 ns | 166.87 ns | 7.34 us | 69.47x | 3.24x | 7.64x | 0.17x |
| `vec4 dot` | 9.89 ns | 249.76 ns | 630.10 ns | 422.07 ns | 315.08 ns | 9.30 us | 25.25x | 0.59x | 0.79x | 0.03x |
| `vec4 magnitude` | 16.26 ns | 3.63 us | 2.52 us | 5.37 us | 403.08 ns | 10.87 us | 223.05x | 0.68x | 9.00x | 0.33x |
| `vec4 normalize` | 37.15 ns | 8.31 us | 6.25 us | 6.04 us | 689.11 ns | 21.25 us | 223.59x | 1.38x | 12.05x | 0.39x |
| `vec4 add` | 7.79 ns | 1.15 us | 1.79 us | 204.65 ns | 169.07 ns | 5.08 us | 147.33x | 5.61x | 6.79x | 0.23x |
| `vec4 add scalar` | 6.83 ns | 972.01 ns | 1.68 us | 208.65 ns | 171.91 ns | 4.91 us | 142.30x | 4.66x | 5.65x | 0.20x |
| `vec4 sub` | 5.19 ns | 1.14 us | 1.68 us | 206.88 ns | 170.69 ns | 9.29 us | 219.55x | 5.51x | 6.68x | 0.12x |
| `vec4 sub scalar` | 4.44 ns | 969.93 ns | 1.45 us | 201.15 ns | 165.15 ns | 9.06 us | 218.56x | 4.82x | 5.87x | 0.11x |
| `vec4 neg` | 4.92 ns | 185.64 ns | 182.10 ns | 75.72 ns | 63.79 ns | 3.91 us | 37.70x | 2.45x | 2.91x | 0.05x |
| `vec4 mul scalar` | 7.24 ns | 641.30 ns | 2.03 us | 219.66 ns | 159.70 ns | 5.55 us | 88.63x | 2.92x | 4.02x | 0.12x |
| `vec4 div scalar` | 14.10 ns | 1.70 us | 1.53 us | 512.68 ns | 224.35 ns | 9.57 us | 120.41x | 3.31x | 7.57x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 18.97 ns | 703.84 ns | 2.17 us | 933.12 ns | 824.69 ns | 21.44 us | 37.11x | 0.75x | 0.85x | 0.03x |
| `mat3 inverse` | 94.13 ns | 14.70 us | 8.93 us | 3.06 us | 2.39 us | 80.13 us | 156.12x | 4.80x | 6.16x | 0.18x |
| `mat3 mul mat3` | 66.12 ns | 3.36 us | 7.68 us | 2.72 us | 2.28 us | 59.03 us | 50.78x | 1.23x | 1.47x | 0.06x |
| `mat3 transform vec3` | 26.69 ns | 1.51 us | 4.91 us | 1.01 us | 862.43 ns | 19.34 us | 56.65x | 1.50x | 1.75x | 0.08x |
| `mat4 determinant` | 42.12 ns | 2.13 us | 1.53 us | 4.55 us | 4.01 us | 92.39 us | 50.50x | 0.47x | 0.53x | 0.02x |
| `mat4 inverse` | 173.43 ns | 25.27 us | 9.37 us | 11.67 us | 8.76 us | 329.98 us | 145.70x | 2.17x | 2.88x | 0.08x |
| `mat4 mul mat4` | 157.62 ns | 6.55 us | 6.79 us | 6.27 us | 5.14 us | 136.58 us | 41.54x | 1.04x | 1.28x | 0.05x |
| `mat4 transform vec4` | 44.73 ns | 2.43 us | 2.64 us | 1.84 us | 1.61 us | 34.10 us | 54.41x | 1.32x | 1.52x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.26 ns | 1.30 us | 4.23 us | 202.50 ns | 203.32 ns | 2.03 us | 37.87x | 6.41x | 6.38x | 0.64x |
| `mat3 zero` | 19.00 ns | 537.18 ns | 549.48 ns | 263.94 ns | 167.75 ns | 11.23 ns | 28.27x | 2.04x | 3.20x | 47.85x |
| `mat3 identity` | 9.82 ns | 633.16 ns | 616.19 ns | 327.41 ns | 211.00 ns | 151.76 ns | 64.50x | 1.93x | 3.00x | 4.17x |
| `mat3 transpose` | 9.30 ns | 666.04 ns | 651.74 ns | 231.92 ns | 180.41 ns | 117.20 ns | 71.62x | 2.87x | 3.69x | 5.68x |
| `mat3 reciprocal` | 87.02 ns | 14.52 us | 24.19 us | 2.90 us | 2.24 us | 80.64 us | 166.81x | 5.01x | 6.49x | 0.18x |
| `mat3 reciprocal checked` | 87.03 ns | 14.94 us | 24.36 us | 2.85 us | 2.24 us | 81.01 us | 171.70x | 5.25x | 6.67x | 0.18x |
| `mat3 inverse checked` | 87.61 ns | 14.93 us | 24.31 us | 2.85 us | 2.23 us | 81.13 us | 170.37x | 5.23x | 6.68x | 0.18x |
| `mat3 inverse checked abort` | 98.04 ns | 14.63 us | 24.59 us | 2.84 us | 2.23 us | 80.32 us | 149.19x | 5.15x | 6.56x | 0.18x |
| `mat3 powi` | 127.48 ns | 6.37 us | 46.04 us | 6.58 us | 6.28 us | 145.85 us | 50.00x | 0.97x | 1.01x | 0.04x |
| `mat3 powi checked` | 127.94 ns | 6.44 us | 44.56 us | 6.56 us | 6.22 us | 145.71 us | 50.33x | 0.98x | 1.04x | 0.04x |
| `mat3 powi checked abort` | 132.14 ns | 6.43 us | 44.70 us | 6.62 us | 6.30 us | 146.31 us | 48.67x | 0.97x | 1.02x | 0.04x |
| `mat3 div scalar checked` | 25.76 ns | 4.34 us | 5.53 us | 1.43 us | 799.38 ns | 21.58 us | 168.39x | 3.04x | 5.43x | 0.20x |
| `mat3 div scalar checked abort` | 30.97 ns | 4.21 us | 5.65 us | 1.46 us | 796.42 ns | 21.53 us | 135.93x | 2.89x | 5.29x | 0.20x |
| `mat3 div matrix checked` | 158.04 ns | 18.58 us | 58.86 us | 5.66 us | 4.35 us | 155.91 us | 117.59x | 3.29x | 4.28x | 0.12x |
| `mat3 div matrix checked abort` | 165.45 ns | 18.65 us | 58.81 us | 5.39 us | 4.35 us | 155.69 us | 112.75x | 3.46x | 4.29x | 0.12x |
| `mat3 add` | 14.15 ns | 3.09 us | 5.24 us | 514.00 ns | 482.89 ns | 11.29 us | 218.04x | 6.00x | 6.39x | 0.27x |
| `mat3 add scalar` | 11.83 ns | 2.59 us | 4.57 us | 783.22 ns | 698.98 ns | 11.83 us | 218.63x | 3.30x | 3.70x | 0.22x |
| `mat3 sub` | 18.10 ns | 3.13 us | 5.18 us | 537.21 ns | 512.02 ns | 20.70 us | 172.85x | 5.83x | 6.11x | 0.15x |
| `mat3 sub scalar` | 11.06 ns | 2.63 us | 4.46 us | 796.75 ns | 696.95 ns | 20.99 us | 237.99x | 3.31x | 3.78x | 0.13x |
| `mat3 neg` | 15.84 ns | 629.97 ns | 597.44 ns | 493.37 ns | 460.80 ns | 8.38 us | 39.77x | 1.28x | 1.37x | 0.08x |
| `mat3 mul scalar` | 13.46 ns | 1.79 us | 5.66 us | 784.80 ns | 668.15 ns | 11.77 us | 133.26x | 2.29x | 2.68x | 0.15x |
| `mat3 div scalar` | 25.81 ns | 4.24 us | 5.60 us | 1.48 us | 812.29 ns | 21.55 us | 164.16x | 2.86x | 5.22x | 0.20x |
| `mat3 div matrix` | 154.21 ns | 18.60 us | 58.03 us | 5.33 us | 4.33 us | 155.04 us | 120.60x | 3.49x | 4.29x | 0.12x |
| `mat3 bitxor` | 127.18 ns | 6.45 us | 45.87 us | 6.69 us | 6.19 us | 146.14 us | 50.72x | 0.96x | 1.04x | 0.04x |
| `mat4 zero` | 17.53 ns | 777.24 ns | 821.26 ns | 512.54 ns | 348.67 ns | 13.87 ns | 44.33x | 1.52x | 2.23x | 56.04x |
| `mat4 identity` | 10.26 ns | 952.81 ns | 988.53 ns | 574.49 ns | 409.39 ns | 226.68 ns | 92.84x | 1.66x | 2.33x | 4.20x |
| `mat4 transpose` | 10.02 ns | 1.19 us | 1.18 us | 439.32 ns | 369.22 ns | 184.06 ns | 118.50x | 2.70x | 3.22x | 6.45x |
| `mat4 reciprocal` | 179.78 ns | 25.77 us | 44.35 us | 10.67 us | 8.75 us | 332.18 us | 143.34x | 2.42x | 2.95x | 0.08x |
| `mat4 reciprocal checked` | 184.56 ns | 25.45 us | 44.33 us | 10.65 us | 8.62 us | 335.02 us | 137.91x | 2.39x | 2.95x | 0.08x |
| `mat4 powi` | 280.07 ns | 12.49 us | 58.57 us | 15.56 us | 13.55 us | 339.12 us | 44.59x | 0.80x | 0.92x | 0.04x |
| `mat4 powi checked` | 279.90 ns | 12.41 us | 58.37 us | 15.60 us | 13.71 us | 339.49 us | 44.33x | 0.80x | 0.91x | 0.04x |
| `mat4 add` | 50.78 ns | 5.20 us | 6.05 us | 913.40 ns | 838.01 ns | 19.08 us | 102.39x | 5.69x | 6.20x | 0.27x |
| `mat4 add scalar` | 20.36 ns | 3.92 us | 6.50 us | 1.39 us | 1.17 us | 19.94 us | 192.64x | 2.81x | 3.36x | 0.20x |
| `mat4 sub` | 36.57 ns | 5.17 us | 6.10 us | 955.37 ns | 880.94 ns | 34.75 us | 141.39x | 5.41x | 5.87x | 0.15x |
| `mat4 sub scalar` | 14.73 ns | 3.95 us | 6.43 us | 1.41 us | 1.15 us | 36.09 us | 268.12x | 2.80x | 3.44x | 0.11x |
| `mat4 neg` | 13.46 ns | 1.10 us | 1.15 us | 923.55 ns | 740.96 ns | 13.90 us | 81.51x | 1.19x | 1.48x | 0.08x |
| `mat4 mul scalar` | 23.47 ns | 2.82 us | 7.21 us | 1.41 us | 1.12 us | 19.48 us | 120.25x | 2.00x | 2.53x | 0.14x |
| `mat4 div scalar` | 32.52 ns | 7.37 us | 7.50 us | 2.66 us | 1.36 us | 37.19 us | 226.76x | 2.77x | 5.42x | 0.20x |
| `mat4 div matrix` | 204.41 ns | 30.01 us | 85.06 us | 16.92 us | 13.96 us | 511.92 us | 146.81x | 1.77x | 2.15x | 0.06x |
| `mat4 bitxor` | 282.79 ns | 12.49 us | 58.35 us | 15.45 us | 13.60 us | 337.55 us | 44.17x | 0.81x | 0.92x | 0.04x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.63 ns | 185.39 ns | 287.44 ns | - | - | 1.27 us | 51.02x | - | - | 0.15x |
| `scalar add ref_owned` | 12.27 ns | 185.43 ns | 281.30 ns | - | - | 1.27 us | 15.12x | - | - | 0.15x |
| `scalar add refs` | 5.27 ns | 157.97 ns | 268.12 ns | - | - | 1.27 us | 30.00x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 8.91 ns | 191.60 ns | 300.34 ns | - | - | - | 21.51x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.47 ns | 184.51 ns | 293.69 ns | - | - | - | 33.73x | - | - | - |
| `scalar sub owned_ref` | 3.95 ns | 189.67 ns | 347.43 ns | - | - | 2.40 us | 48.06x | - | - | 0.08x |
| `scalar sub ref_owned` | 12.36 ns | 188.74 ns | 337.10 ns | - | - | 2.39 us | 15.27x | - | - | 0.08x |
| `scalar sub refs` | 5.47 ns | 162.37 ns | 322.72 ns | - | - | 2.38 us | 29.71x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 8.97 ns | 198.76 ns | 352.60 ns | - | - | - | 22.17x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.69 ns | 192.49 ns | 351.91 ns | - | - | - | 33.85x | - | - | - |
| `scalar mul owned_ref` | 4.27 ns | 107.00 ns | 370.62 ns | - | - | 1.50 us | 25.07x | - | - | 0.07x |
| `scalar mul ref_owned` | 13.49 ns | 105.18 ns | 362.34 ns | - | - | 1.51 us | 7.79x | - | - | 0.07x |
| `scalar mul refs` | 6.01 ns | 81.28 ns | 347.62 ns | - | - | 1.50 us | 13.53x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.63 ns | 106.38 ns | 377.52 ns | - | - | - | 11.05x | - | - | - |
| `scalar mul ref_owned_with_clone` | 7.60 ns | 103.88 ns | 379.77 ns | - | - | - | 13.68x | - | - | - |
| `scalar div owned_ref` | 5.80 ns | 274.41 ns | 313.95 ns | - | - | 2.51 us | 47.35x | - | - | 0.11x |
| `scalar div ref_owned` | 16.98 ns | 260.52 ns | 318.18 ns | - | - | 2.50 us | 15.34x | - | - | 0.10x |
| `scalar div refs` | 6.84 ns | 247.74 ns | 299.58 ns | - | - | 2.49 us | 36.23x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.19 ns | 274.00 ns | 323.41 ns | - | - | - | 18.04x | - | - | - |
| `scalar div ref_owned_with_clone` | 7.95 ns | 276.46 ns | 327.85 ns | - | - | - | 34.77x | - | - | - |
| `vec3 add refs` | 6.00 ns | 617.05 ns | 1.39 us | - | - | 3.87 us | 102.89x | - | - | 0.16x |
| `vec3 sub refs` | 6.00 ns | 620.03 ns | 1.44 us | - | - | 7.16 us | 103.36x | - | - | 0.09x |
| `vec3 neg ref` | 3.27 ns | 171.81 ns | 172.12 ns | - | - | 3.10 us | 52.60x | - | - | 0.06x |
| `vec3 add_scalar_ref` | 6.42 ns | 762.30 ns | 1.43 us | - | - | 3.74 us | 118.76x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.40 ns | 745.69 ns | 1.23 us | - | - | 6.87 us | 116.59x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 6.81 ns | 495.50 ns | 2.02 us | - | - | 4.33 us | 72.78x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.00 ns | 1.27 us | 1.56 us | - | - | 7.42 us | 158.87x | - | - | 0.17x |
| `vec4 add refs` | 6.55 ns | 758.65 ns | 1.40 us | - | - | 5.18 us | 115.78x | - | - | 0.15x |
| `vec4 sub refs` | 3.06 ns | 756.88 ns | 1.29 us | - | - | 9.46 us | 247.57x | - | - | 0.08x |
| `vec4 neg ref` | 4.16 ns | 246.00 ns | 247.76 ns | - | - | 4.00 us | 59.09x | - | - | 0.06x |
| `vec4 add_scalar_ref` | 6.86 ns | 1.02 us | 1.73 us | - | - | 5.01 us | 148.58x | - | - | 0.20x |
| `vec4 sub_scalar_ref` | 4.15 ns | 1.01 us | 1.47 us | - | - | 9.16 us | 241.98x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.37 ns | 670.64 ns | 2.07 us | - | - | 5.61 us | 90.99x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.31 ns | 1.67 us | 1.49 us | - | - | 9.78 us | 147.73x | - | - | 0.17x |
| `mat3 add refs` | 10.83 ns | 1.70 us | 3.83 us | - | - | 11.38 us | 156.55x | - | - | 0.15x |
| `mat3 sub refs` | 10.49 ns | 1.69 us | 3.75 us | - | - | 20.96 us | 161.36x | - | - | 0.08x |
| `mat3 mul refs` | 54.15 ns | 2.26 us | 14.76 us | - | - | 59.09 us | 41.67x | - | - | 0.04x |
| `mat3 div refs` | 134.12 ns | 17.75 us | 57.97 us | - | - | 157.18 us | 132.37x | - | - | 0.11x |
| `mat3 neg ref` | 9.60 ns | 501.55 ns | 506.32 ns | - | - | 8.44 us | 52.25x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 43.35 ns | 2.43 us | 4.66 us | - | - | 11.69 us | 56.11x | - | - | 0.21x |
| `mat3 sub_scalar_ref` | 43.71 ns | 2.43 us | 4.53 us | - | - | 21.08 us | 55.70x | - | - | 0.12x |
| `mat3 mul_scalar_ref` | 46.02 ns | 1.74 us | 5.87 us | - | - | 11.94 us | 37.75x | - | - | 0.15x |
| `mat3 div_scalar_ref` | 21.55 ns | 4.34 us | 5.60 us | - | - | 21.34 us | 201.45x | - | - | 0.20x |
| `mat4 add refs` | 17.33 ns | 2.60 us | 3.51 us | - | - | 18.96 us | 149.89x | - | - | 0.14x |
| `mat4 sub refs` | 16.05 ns | 2.63 us | 3.57 us | - | - | 35.12 us | 164.05x | - | - | 0.07x |
| `mat4 mul refs` | 115.20 ns | 4.79 us | 20.11 us | - | - | 138.80 us | 41.58x | - | - | 0.03x |
| `mat4 div refs` | 190.02 ns | 29.06 us | 86.03 us | - | - | 516.40 us | 152.94x | - | - | 0.06x |
| `mat4 neg ref` | 12.39 ns | 836.03 ns | 836.28 ns | - | - | 13.80 us | 67.47x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 51.40 ns | 4.00 us | 6.67 us | - | - | 20.12 us | 77.90x | - | - | 0.20x |
| `mat4 sub_scalar_ref` | 38.60 ns | 4.00 us | 6.55 us | - | - | 37.55 us | 103.68x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 55.21 ns | 2.88 us | 7.41 us | - | - | 19.90 us | 52.10x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 28.50 ns | 7.37 us | 7.74 us | - | - | 37.06 us | 258.72x | - | - | 0.20x |
| `mat3 transform_vec refs` | 25.44 ns | 914.79 ns | 5.02 us | - | - | 19.92 us | 35.95x | - | - | 0.05x |
| `mat4 transform_vec refs` | 42.74 ns | 1.49 us | 5.94 us | - | - | 34.12 us | 34.98x | - | - | 0.04x |
| `complex add refs` | 7.62 ns | 349.45 ns | 708.88 ns | - | - | 2.50 us | 45.88x | - | - | 0.14x |
| `complex sub refs` | 7.89 ns | 378.38 ns | 729.12 ns | - | - | 4.65 us | 47.93x | - | - | 0.08x |
| `complex mul refs` | 7.84 ns | 745.04 ns | 3.08 us | - | - | 10.02 us | 95.02x | - | - | 0.07x |
| `complex div refs` | 17.12 ns | 2.57 us | 6.17 us | - | - | 21.81 us | 150.24x | - | - | 0.12x |
| `complex neg ref` | 2.33 ns | 72.52 ns | 70.33 ns | - | - | 2.10 us | 31.08x | - | - | 0.03x |
| `complex div_real_ref` | 7.16 ns | 643.90 ns | 596.63 ns | - | - | 5.24 us | 89.94x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.43 us |
| `astro sin 160` | 13.98 us |
| `astro sin 192` | 13.96 us |
| `astro sin 256` | 16.12 us |
