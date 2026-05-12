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
| `sin 0.1` | 10.76 ns | 148.65 ns | 146.20 ns | 10.70 us | 785.74 ns | 1.83 us | 13.82x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.48 ns | 145.58 ns | 146.04 ns | 10.62 us | 494.43 ns | 1.63 us | 12.68x | 0.01x | 0.29x | 0.09x |
| `sin 1.23456789` | 11.53 ns | 200.36 ns | 187.25 ns | 12.31 us | 802.36 ns | 1.83 us | 17.37x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 11.89 ns | 193.10 ns | 181.01 ns | 10.67 us | 605.88 ns | 1.65 us | 16.24x | 0.02x | 0.32x | 0.12x |
| `sin 1e6` | 12.48 ns | 90.78 ns | 91.93 ns | 15.63 us | 1.12 us | 2.02 us | 7.28x | 0.01x | 0.08x | 0.05x |
| `cos 1e6` | 12.15 ns | 89.16 ns | 89.21 ns | 13.45 us | 843.39 ns | 1.79 us | 7.34x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 65.24 ns | 93.43 ns | 92.43 ns | 18.38 us | 2.82 us | 3.64 us | 1.43x | 0.01x | 0.03x | 0.03x |
| `cos 1e30` | 67.81 ns | 91.67 ns | 92.45 ns | 15.62 us | 947.88 ns | 3.15 us | 1.35x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.58 ns | 145.62 ns | 350.64 ns | 12.16 us | 744.08 ns | 1.88 us | 12.58x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.54 ns | 145.82 ns | 727.27 ns | 10.91 us | 558.44 ns | 1.64 us | 12.63x | 0.01x | 0.26x | 0.09x |
| `sin 1000pi_eps` | 11.55 ns | 90.88 ns | 763.10 ns | 15.77 us | 2.30 us | 2.81 us | 7.87x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.21 ns | 89.12 ns | 742.25 ns | 13.76 us | 589.93 ns | 1.62 us | 7.30x | 0.01x | 0.15x | 0.05x |
| `asin 0.5` | 10.75 ns | 149.22 ns | 147.34 ns | 49.28 us | 2.92 us | 13.00 us | 13.88x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.14 ns | 420.91 ns | 415.48 ns | 58.51 us | 2.90 us | 12.81 us | 37.78x | 0.01x | 0.15x | 0.03x |
| `atanh 0.5` | 14.31 ns | 236.75 ns | 240.81 ns | 33.95 us | 1.66 us | 12.72 us | 16.54x | 0.01x | 0.14x | 0.02x |
| `asin neg_0.999999` | 13.88 ns | 577.32 ns | 556.04 ns | 13.77 us | 2.48 us | 12.60 us | 41.59x | 0.04x | 0.23x | 0.05x |
| `acos neg_0.999999` | 15.11 ns | 517.49 ns | 494.24 ns | 17.71 us | 2.65 us | 12.77 us | 34.25x | 0.03x | 0.20x | 0.04x |
| `atanh neg_0.999999` | 14.16 ns | 211.38 ns | 197.97 ns | 36.10 us | 1.66 us | 12.45 us | 14.93x | 0.01x | 0.13x | 0.02x |
| `asin 0.999999` | 13.57 ns | 522.48 ns | 764.49 ns | 13.68 us | 2.48 us | 12.45 us | 38.51x | 0.04x | 0.21x | 0.04x |
| `acos 0.999999` | 13.95 ns | 300.01 ns | 533.08 ns | 17.74 us | 2.66 us | 12.54 us | 21.51x | 0.02x | 0.11x | 0.02x |
| `atanh 0.999999` | 14.29 ns | 216.59 ns | 204.09 ns | 31.08 us | 1.66 us | 12.24 us | 15.16x | 0.01x | 0.13x | 0.02x |
| `asin 1e-12` | 9.33 ns | 277.49 ns | 494.87 ns | 8.11 us | 1.41 us | 14.71 us | 29.75x | 0.03x | 0.20x | 0.02x |
| `acos 1e-12` | 9.91 ns | 470.19 ns | 682.35 ns | 9.59 us | 1.41 us | 14.88 us | 47.42x | 0.05x | 0.33x | 0.03x |
| `atanh 1e-12` | 9.63 ns | 245.89 ns | 224.93 ns | 36.19 us | 168.10 ns | 20.09 us | 25.53x | 0.01x | 1.46x | 0.01x |
| `atan 0.5` | 14.67 ns | 172.79 ns | 174.97 ns | 34.93 us | 2.70 us | 17.49 us | 11.78x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.34 ns | 276.46 ns | 279.02 ns | 39.27 us | 1.64 us | 7.33 us | 10.50x | 0.01x | 0.17x | 0.04x |
| `atan neg_1e-12` | 14.24 ns | 291.86 ns | 274.06 ns | 1.57 us | 1.10 us | 14.80 us | 20.50x | 0.19x | 0.26x | 0.02x |
| `asinh neg_1e-12` | 15.77 ns | 482.58 ns | 399.41 ns | 42.30 us | 8.75 us | 11.80 us | 30.61x | 0.01x | 0.06x | 0.04x |
| `atan 1e6` | 15.05 ns | 171.09 ns | 167.79 ns | 2.75 us | 1.42 us | 17.58 us | 11.37x | 0.06x | 0.12x | 0.01x |
| `asinh 1e6` | 26.39 ns | 269.05 ns | 266.87 ns | 37.18 us | 1.70 us | 7.08 us | 10.19x | 0.01x | 0.16x | 0.04x |
| `atan neg_1e6` | 14.78 ns | 272.57 ns | 271.41 ns | 2.83 us | 1.44 us | 17.37 us | 18.44x | 0.10x | 0.19x | 0.02x |
| `asinh neg_1e6` | 26.31 ns | 387.31 ns | 391.01 ns | 36.43 us | 1.68 us | 6.79 us | 14.72x | 0.01x | 0.23x | 0.06x |
| `acosh 9` | 12.21 ns | 167.88 ns | 166.73 ns | 40.82 us | 1.67 us | 9.46 us | 13.75x | 0.00x | 0.10x | 0.02x |
| `acosh 1_plus_1e-12` | 12.78 ns | 246.83 ns | 241.20 ns | 40.55 us | 8.44 us | 11.20 us | 19.32x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.19 ns | 167.72 ns | 167.76 ns | 36.29 us | 1.67 us | 9.61 us | 13.76x | 0.00x | 0.10x | 0.02x |
| `acosh e` | 12.17 ns | 245.33 ns | 2.30 us | 40.01 us | 1.67 us | 9.56 us | 20.16x | 0.01x | 0.15x | 0.03x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.54 ns | 18.11 ns | 24.70 ns | 21.86 ns | 0.94 ns | 39.75x | 0.75x | 0.85x | 19.76x |
| `one` | 0.47 ns | 23.36 ns | 26.61 ns | 39.16 ns | 37.27 ns | 31.83 ns | 50.10x | 0.60x | 0.63x | 0.73x |
| `e` | 0.47 ns | 91.66 ns | 88.02 ns | 53.32 ns | 1.08 us | 220.13 ns | 196.52x | 1.72x | 0.08x | 0.42x |
| `pi` | 0.47 ns | 50.40 ns | 50.47 ns | 45.39 ns | 55.15 ns | 223.37 ns | 107.83x | 1.11x | 0.91x | 0.23x |
| `tau` | 0.46 ns | 49.83 ns | 50.78 ns | 119.32 ns | 116.99 ns | 1.82 us | 107.72x | 0.42x | 0.43x | 0.03x |
| `add` | 5.19 ns | 211.92 ns | 317.32 ns | 49.90 ns | 45.57 ns | 1.24 us | 40.84x | 4.25x | 4.65x | 0.17x |
| `sub` | 5.28 ns | 261.74 ns | 521.91 ns | 54.97 ns | 47.73 ns | 2.36 us | 49.56x | 4.76x | 5.48x | 0.11x |
| `neg` | 4.96 ns | 40.47 ns | 39.78 ns | 23.49 ns | 22.27 ns | 1.02 us | 8.16x | 1.72x | 1.82x | 0.04x |
| `mul` | 5.28 ns | 135.64 ns | 420.89 ns | 57.89 ns | 52.85 ns | 1.48 us | 25.69x | 2.34x | 2.57x | 0.09x |
| `div` | 7.67 ns | 318.02 ns | 384.93 ns | 134.00 ns | 66.43 ns | 2.51 us | 41.44x | 2.37x | 4.79x | 0.13x |
| `reciprocal` | 8.50 ns | 79.38 ns | 80.77 ns | 154.05 ns | 59.02 ns | 1.46 us | 9.34x | 0.52x | 1.34x | 0.05x |
| `reciprocal checked` | 8.77 ns | 76.68 ns | 78.53 ns | 156.46 ns | 59.07 ns | 1.47 us | 8.74x | 0.49x | 1.30x | 0.05x |
| `reciprocal checked abort` | 23.63 ns | 90.15 ns | 92.68 ns | 153.98 ns | 58.87 ns | 1.47 us | 3.82x | 0.59x | 1.53x | 0.06x |
| `pow` | 20.20 ns | 6.60 us | 5.92 us | 53.73 us | 2.89 us | 2.38 us | 326.78x | 0.12x | 2.28x | 2.78x |
| `powi` | 5.73 ns | 398.18 ns | 2.67 us | 277.44 ns | 92.58 ns | 1.51 us | 69.51x | 1.44x | 4.30x | 0.26x |
| `exp` | 18.98 ns | 233.09 ns | 227.18 ns | 13.85 us | 903.21 ns | 1.85 us | 12.28x | 0.02x | 0.26x | 0.13x |
| `ln` | 10.40 ns | 1.53 us | 1.43 us | 30.54 us | 1.34 us | 1.81 us | 146.90x | 0.05x | 1.14x | 0.84x |
| `log10` | 13.19 ns | 1.72 us | 1.63 us | 35.26 us | 2.82 us | 6.55 us | 130.27x | 0.05x | 0.61x | 0.26x |
| `log10 abort` | 17.05 ns | 1.72 us | 1.60 us | 35.22 us | 2.80 us | 6.53 us | 100.80x | 0.05x | 0.61x | 0.26x |
| `sqrt` | 8.08 ns | 1.51 us | 1.63 us | 5.02 us | 107.30 ns | 1.42 us | 186.88x | 0.30x | 14.07x | 1.07x |
| `sin` | 14.84 ns | 126.64 ns | 121.19 ns | 13.84 us | 1.27 us | 2.24 us | 8.54x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.03 ns | 123.87 ns | 118.98 ns | 11.86 us | 651.96 ns | 1.70 us | 6.87x | 0.01x | 0.19x | 0.07x |
| `tan` | 24.31 ns | 168.67 ns | 167.60 ns | 28.95 us | 1.59 us | 6.43 us | 6.94x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.17 ns | 3.64 us | 3.66 us | 3.27 us | 1.10 us | 10.20 us | 200.47x | 1.12x | 3.32x | 0.36x |
| `cosh` | 18.27 ns | 3.57 us | 3.58 us | 7.95 us | 1.05 us | 9.19 us | 195.54x | 0.45x | 3.41x | 0.39x |
| `tanh` | 27.77 ns | 7.43 us | 7.37 us | 3.34 us | 1.19 us | 22.22 us | 267.73x | 2.22x | 6.22x | 0.33x |
| `asin` | 13.14 ns | 397.18 ns | 621.22 ns | 21.10 us | 2.38 us | 13.51 us | 30.23x | 0.02x | 0.17x | 0.03x |
| `asin abort` | 17.29 ns | 407.29 ns | 615.89 ns | 21.41 us | 2.39 us | 13.50 us | 23.56x | 0.02x | 0.17x | 0.03x |
| `acos` | 14.34 ns | 452.53 ns | 665.02 ns | 26.26 us | 2.52 us | 13.43 us | 31.57x | 0.02x | 0.18x | 0.03x |
| `acos abort` | 17.59 ns | 468.01 ns | 675.52 ns | 26.19 us | 2.49 us | 13.44 us | 26.61x | 0.02x | 0.19x | 0.03x |
| `atan` | 14.71 ns | 167.84 ns | 158.07 ns | 18.04 us | 2.22 us | 18.42 us | 11.41x | 0.01x | 0.08x | 0.01x |
| `atan abort` | 21.18 ns | 178.33 ns | 163.37 ns | 17.83 us | 2.23 us | 18.32 us | 8.42x | 0.01x | 0.08x | 0.01x |
| `asinh` | 33.17 ns | 276.30 ns | 270.92 ns | 38.09 us | 1.68 us | 7.28 us | 8.33x | 0.01x | 0.16x | 0.04x |
| `asinh abort` | 30.50 ns | 278.26 ns | 273.96 ns | 38.88 us | 1.69 us | 7.29 us | 9.12x | 0.01x | 0.16x | 0.04x |
| `acosh` | 12.34 ns | 199.22 ns | 193.03 ns | 39.53 us | 3.37 us | 10.30 us | 16.15x | 0.01x | 0.06x | 0.02x |
| `acosh abort` | 15.45 ns | 212.38 ns | 200.61 ns | 39.74 us | 3.41 us | 10.19 us | 13.74x | 0.01x | 0.06x | 0.02x |
| `atanh` | 13.54 ns | 229.18 ns | 248.59 ns | 34.73 us | 1.29 us | 14.63 us | 16.92x | 0.01x | 0.18x | 0.02x |
| `atanh abort` | 16.71 ns | 233.44 ns | 249.47 ns | 34.79 us | 1.31 us | 14.57 us | 13.97x | 0.01x | 0.18x | 0.02x |
| `zero status` | 1.19 ns | 0.95 ns | 0.97 ns | 1.01 ns | 6.67 ns | 8.23 ns | 0.80x | 0.95x | 0.14x | 0.12x |
| `zero status abort` | 1.40 ns | 1.10 ns | 1.12 ns | 1.01 ns | 6.70 ns | 8.21 ns | 0.78x | 1.09x | 0.16x | 0.13x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.92 ns | 30.35 ns | 30.44 ns | 46.72 ns | - | 1.85 ns | 33.06x | 0.65x | - | 16.39x |
| `one` | 5.00 ns | 37.12 ns | 37.23 ns | 61.80 ns | - | 29.28 ns | 7.42x | 0.60x | - | 1.27x |
| `i` | 0.93 ns | 39.70 ns | 39.37 ns | 61.50 ns | - | 31.94 ns | 42.67x | 0.65x | - | 1.24x |
| `free i` | 0.92 ns | 38.65 ns | 39.68 ns | 61.67 ns | - | 32.02 ns | 42.00x | 0.63x | - | 1.21x |
| `conjugate` | 2.08 ns | 90.30 ns | 89.92 ns | 40.87 ns | - | 1.04 us | 43.42x | 2.21x | - | 0.09x |
| `norm squared` | 5.69 ns | 161.19 ns | 553.52 ns | 152.19 ns | - | 4.16 us | 28.34x | 1.06x | - | 0.04x |
| `reciprocal` | 17.95 ns | 1.72 us | 2.83 us | 441.48 ns | - | 10.49 us | 95.84x | 3.90x | - | 0.16x |
| `reciprocal checked` | 14.80 ns | 1.79 us | 2.81 us | 437.75 ns | - | 10.50 us | 120.92x | 4.09x | - | 0.17x |
| `powi` | 17.54 ns | 2.08 us | 14.09 us | 1.45 us | - | 42.44 us | 118.84x | 1.44x | - | 0.05x |
| `powi checked` | 17.39 ns | 2.08 us | 14.20 us | 1.46 us | - | 42.69 us | 119.74x | 1.43x | - | 0.05x |
| `div checked` | 17.83 ns | 2.73 us | 6.43 us | 782.79 ns | - | 21.00 us | 152.95x | 3.48x | - | 0.13x |
| `div real checked` | 9.21 ns | 639.88 ns | 626.36 ns | 264.99 ns | - | 5.02 us | 69.50x | 2.41x | - | 0.13x |
| `from scalar` | 1.40 ns | 68.27 ns | 68.95 ns | 44.92 ns | - | 10.12 ns | 48.88x | 1.52x | - | 6.74x |
| `add` | 6.12 ns | 478.24 ns | 853.87 ns | 103.33 ns | - | 2.49 us | 78.14x | 4.63x | - | 0.19x |
| `sub` | 6.15 ns | 528.69 ns | 1.11 us | 114.57 ns | - | 4.65 us | 85.91x | 4.61x | - | 0.11x |
| `neg` | 2.54 ns | 93.15 ns | 92.26 ns | 43.39 ns | - | 2.05 us | 36.65x | 2.15x | - | 0.05x |
| `mul` | 11.24 ns | 889.52 ns | 3.43 us | 307.14 ns | - | 9.73 us | 79.11x | 2.90x | - | 0.09x |
| `div` | 17.67 ns | 2.70 us | 6.46 us | 783.85 ns | - | 20.95 us | 152.81x | 3.44x | - | 0.13x |
| `div real` | 9.95 ns | 640.60 ns | 627.13 ns | 262.56 ns | - | 5.05 us | 64.37x | 2.44x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.63 ns | 210.89 ns | 2.47 us | 313.82 ns | 244.69 ns | 7.06 us | 24.45x | 0.67x | 0.86x | 0.03x |
| `vec3 magnitude` | 11.24 ns | 3.94 us | 5.48 us | 5.37 us | 343.64 ns | 8.52 us | 350.59x | 0.73x | 11.47x | 0.46x |
| `vec3 normalize` | 25.36 ns | 8.20 us | 11.37 us | 6.04 us | 591.09 ns | 16.34 us | 323.42x | 1.36x | 13.87x | 0.50x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.06 ns | 283.98 ns | 1.43 us | 68.95 ns | 56.83 ns | 713.35 ns | 92.89x | 4.12x | 5.00x | 0.40x |
| `vec3 zero` | 1.38 ns | 130.10 ns | 130.24 ns | 64.31 ns | 30.82 ns | 2.85 ns | 94.47x | 2.02x | 4.22x | 45.71x |
| `vec3 dot abort` | 8.39 ns | 212.14 ns | 890.28 ns | 256.69 ns | 200.84 ns | 7.04 us | 25.28x | 0.83x | 1.06x | 0.03x |
| `vec3 magnitude abort` | 17.05 ns | 3.98 us | 3.22 us | 5.51 us | 321.39 ns | 8.57 us | 233.64x | 0.72x | 12.39x | 0.47x |
| `vec3 normalize checked` | 26.35 ns | 8.05 us | 7.30 us | 5.93 us | 562.23 ns | 16.57 us | 305.59x | 1.36x | 14.32x | 0.49x |
| `vec3 normalize checked abort` | 30.48 ns | 8.30 us | 7.30 us | 5.93 us | 563.67 ns | 16.63 us | 272.20x | 1.40x | 14.72x | 0.50x |
| `vec3 div scalar checked` | 15.55 ns | 1.31 us | 1.61 us | 401.50 ns | - | - | 84.20x | 3.26x | - | - |
| `vec3 div scalar checked abort` | 18.67 ns | 1.37 us | 1.59 us | 397.44 ns | - | - | 73.18x | 3.44x | - | - |
| `vec3 add` | 6.70 ns | 961.27 ns | 1.72 us | 152.14 ns | 125.68 ns | 3.85 us | 143.40x | 6.32x | 7.65x | 0.25x |
| `vec3 add scalar` | 6.36 ns | 765.58 ns | 1.44 us | 149.67 ns | 131.25 ns | 3.72 us | 120.41x | 5.12x | 5.83x | 0.21x |
| `vec3 sub` | 6.69 ns | 1.08 us | 2.52 us | 163.76 ns | 135.71 ns | 7.21 us | 160.67x | 6.57x | 7.92x | 0.15x |
| `vec3 sub scalar` | 6.37 ns | 902.74 ns | 1.97 us | 146.15 ns | 123.28 ns | 6.89 us | 141.72x | 6.18x | 7.32x | 0.13x |
| `vec3 neg` | 3.74 ns | 142.67 ns | 142.31 ns | 58.17 ns | 50.45 ns | 3.04 us | 38.18x | 2.45x | 2.83x | 0.05x |
| `vec3 mul scalar` | 6.84 ns | 513.48 ns | 2.00 us | 168.63 ns | 122.69 ns | 4.31 us | 75.12x | 3.04x | 4.19x | 0.12x |
| `vec3 div scalar` | 9.83 ns | 1.33 us | 1.56 us | 396.81 ns | 178.77 ns | 7.37 us | 134.86x | 3.34x | 7.41x | 0.18x |
| `vec4 dot` | 9.55 ns | 250.27 ns | 645.25 ns | 441.38 ns | 322.27 ns | 9.33 us | 26.22x | 0.57x | 0.78x | 0.03x |
| `vec4 magnitude` | 16.34 ns | 3.72 us | 2.57 us | 5.65 us | 431.93 ns | 10.92 us | 227.62x | 0.66x | 8.61x | 0.34x |
| `vec4 normalize` | 36.82 ns | 8.26 us | 6.44 us | 6.16 us | 725.27 ns | 21.52 us | 224.47x | 1.34x | 11.40x | 0.38x |
| `vec4 add` | 7.23 ns | 1.14 us | 1.80 us | 200.65 ns | 175.11 ns | 5.09 us | 157.62x | 5.68x | 6.51x | 0.22x |
| `vec4 add scalar` | 6.78 ns | 1.03 us | 1.75 us | 214.78 ns | 177.88 ns | 4.94 us | 152.12x | 4.80x | 5.80x | 0.21x |
| `vec4 sub` | 5.02 ns | 1.28 us | 2.28 us | 207.38 ns | 178.21 ns | 9.44 us | 254.33x | 6.15x | 7.16x | 0.14x |
| `vec4 sub scalar` | 4.52 ns | 1.15 us | 2.15 us | 204.21 ns | 169.18 ns | 9.18 us | 254.19x | 5.63x | 6.79x | 0.13x |
| `vec4 neg` | 4.88 ns | 188.68 ns | 191.44 ns | 79.13 ns | 63.37 ns | 3.89 us | 38.64x | 2.38x | 2.98x | 0.05x |
| `vec4 mul scalar` | 7.34 ns | 696.99 ns | 2.11 us | 219.34 ns | 158.13 ns | 5.54 us | 95.00x | 3.18x | 4.41x | 0.13x |
| `vec4 div scalar` | 14.20 ns | 1.75 us | 1.57 us | 533.77 ns | 227.67 ns | 9.57 us | 123.42x | 3.28x | 7.70x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 18.87 ns | 703.71 ns | 2.37 us | 969.06 ns | 843.31 ns | 21.84 us | 37.29x | 0.73x | 0.83x | 0.03x |
| `mat3 inverse` | 86.84 ns | 14.36 us | 8.59 us | 3.15 us | 2.43 us | 81.45 us | 165.36x | 4.56x | 5.90x | 0.18x |
| `mat3 mul mat3` | 66.60 ns | 3.30 us | 7.92 us | 2.78 us | 2.33 us | 60.29 us | 49.63x | 1.19x | 1.42x | 0.05x |
| `mat3 transform vec3` | 28.13 ns | 1.56 us | 5.07 us | 1.02 us | 879.60 ns | 19.90 us | 55.48x | 1.53x | 1.77x | 0.08x |
| `mat4 determinant` | 41.73 ns | 2.25 us | 1.55 us | 4.53 us | 4.11 us | 93.30 us | 53.81x | 0.50x | 0.55x | 0.02x |
| `mat4 inverse` | 176.63 ns | 24.66 us | 8.46 us | 11.09 us | 9.13 us | 337.31 us | 139.60x | 2.22x | 2.70x | 0.07x |
| `mat4 mul mat4` | 150.61 ns | 5.81 us | 4.83 us | 6.35 us | 5.36 us | 140.34 us | 38.55x | 0.91x | 1.08x | 0.04x |
| `mat4 transform vec4` | 44.97 ns | 2.53 us | 2.43 us | 1.89 us | 1.65 us | 35.03 us | 56.20x | 1.33x | 1.53x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.03 ns | 1.31 us | 4.26 us | 208.95 ns | 195.56 ns | 2.08 us | 38.36x | 6.25x | 6.67x | 0.63x |
| `mat3 zero` | 16.00 ns | 503.48 ns | 511.42 ns | 263.24 ns | 162.34 ns | 12.31 ns | 31.46x | 1.91x | 3.10x | 40.89x |
| `mat3 identity` | 9.89 ns | 605.77 ns | 618.50 ns | 319.62 ns | 206.56 ns | 139.46 ns | 61.28x | 1.90x | 2.93x | 4.34x |
| `mat3 transpose` | 8.78 ns | 660.67 ns | 661.43 ns | 237.28 ns | 169.72 ns | 117.79 ns | 75.24x | 2.78x | 3.89x | 5.61x |
| `mat3 reciprocal` | 87.78 ns | 14.55 us | 24.65 us | 2.89 us | 2.23 us | 81.25 us | 165.81x | 5.03x | 6.54x | 0.18x |
| `mat3 reciprocal checked` | 89.01 ns | 14.67 us | 24.67 us | 2.91 us | 2.28 us | 80.85 us | 164.81x | 5.04x | 6.44x | 0.18x |
| `mat3 inverse checked` | 88.40 ns | 14.46 us | 24.61 us | 2.90 us | 2.28 us | 81.20 us | 163.57x | 4.99x | 6.35x | 0.18x |
| `mat3 inverse checked abort` | 96.13 ns | 14.54 us | 24.54 us | 2.95 us | 2.27 us | 80.70 us | 151.27x | 4.92x | 6.41x | 0.18x |
| `mat3 powi` | 125.34 ns | 6.34 us | 45.31 us | 6.81 us | 6.21 us | 147.28 us | 50.61x | 0.93x | 1.02x | 0.04x |
| `mat3 powi checked` | 124.66 ns | 6.32 us | 46.03 us | 6.71 us | 6.10 us | 146.84 us | 50.66x | 0.94x | 1.04x | 0.04x |
| `mat3 powi checked abort` | 126.49 ns | 6.36 us | 45.27 us | 6.80 us | 6.09 us | 147.98 us | 50.32x | 0.94x | 1.04x | 0.04x |
| `mat3 div scalar checked` | 23.85 ns | 4.21 us | 5.61 us | 1.47 us | 835.87 ns | 21.48 us | 176.76x | 2.88x | 5.04x | 0.20x |
| `mat3 div scalar checked abort` | 29.07 ns | 4.23 us | 5.70 us | 1.47 us | 832.46 ns | 21.52 us | 145.54x | 2.88x | 5.08x | 0.20x |
| `mat3 div matrix checked` | 151.27 ns | 18.23 us | 66.78 us | 5.43 us | 4.54 us | 157.08 us | 120.53x | 3.36x | 4.02x | 0.12x |
| `mat3 div matrix checked abort` | 162.89 ns | 18.39 us | 67.77 us | 5.43 us | 4.57 us | 156.87 us | 112.92x | 3.39x | 4.03x | 0.12x |
| `mat3 add` | 14.41 ns | 3.07 us | 5.29 us | 499.63 ns | 487.88 ns | 11.48 us | 213.32x | 6.15x | 6.30x | 0.27x |
| `mat3 add scalar` | 12.01 ns | 2.36 us | 4.56 us | 755.98 ns | 701.78 ns | 11.73 us | 196.52x | 3.12x | 3.36x | 0.20x |
| `mat3 sub` | 12.76 ns | 3.40 us | 7.63 us | 537.14 ns | 515.25 ns | 20.82 us | 266.74x | 6.33x | 6.60x | 0.16x |
| `mat3 sub scalar` | 10.79 ns | 2.62 us | 6.44 us | 779.55 ns | 704.50 ns | 21.12 us | 242.72x | 3.36x | 3.72x | 0.12x |
| `mat3 neg` | 10.54 ns | 587.06 ns | 601.37 ns | 475.75 ns | 447.56 ns | 8.46 us | 55.70x | 1.23x | 1.31x | 0.07x |
| `mat3 mul scalar` | 13.67 ns | 1.71 us | 5.69 us | 772.03 ns | 663.92 ns | 11.95 us | 124.79x | 2.21x | 2.57x | 0.14x |
| `mat3 div scalar` | 24.40 ns | 4.26 us | 5.60 us | 1.50 us | 830.75 ns | 21.67 us | 174.57x | 2.83x | 5.13x | 0.20x |
| `mat3 div matrix` | 151.32 ns | 18.45 us | 67.44 us | 5.41 us | 4.55 us | 156.15 us | 121.92x | 3.41x | 4.06x | 0.12x |
| `mat3 bitxor` | 125.75 ns | 6.30 us | 45.58 us | 6.81 us | 6.09 us | 148.18 us | 50.12x | 0.93x | 1.04x | 0.04x |
| `mat4 zero` | 11.20 ns | 902.96 ns | 897.89 ns | 452.96 ns | 359.55 ns | 14.76 ns | 80.61x | 1.99x | 2.51x | 61.19x |
| `mat4 identity` | 11.02 ns | 1.07 us | 1.09 us | 565.77 ns | 417.20 ns | 217.23 ns | 97.49x | 1.90x | 2.58x | 4.95x |
| `mat4 transpose` | 9.23 ns | 1.06 us | 1.05 us | 424.90 ns | 368.49 ns | 186.75 ns | 114.99x | 2.50x | 2.88x | 5.68x |
| `mat4 reciprocal` | 183.15 ns | 24.10 us | 42.34 us | 10.73 us | 8.65 us | 335.46 us | 131.58x | 2.25x | 2.79x | 0.07x |
| `mat4 reciprocal checked` | 179.26 ns | 24.70 us | 42.33 us | 10.70 us | 8.70 us | 335.55 us | 137.77x | 2.31x | 2.84x | 0.07x |
| `mat4 powi` | 262.71 ns | 11.29 us | 57.90 us | 15.20 us | 14.00 us | 344.11 us | 42.96x | 0.74x | 0.81x | 0.03x |
| `mat4 powi checked` | 265.27 ns | 11.36 us | 58.30 us | 15.27 us | 13.94 us | 343.19 us | 42.84x | 0.74x | 0.81x | 0.03x |
| `mat4 add` | 50.40 ns | 5.08 us | 5.70 us | 934.28 ns | 848.82 ns | 19.28 us | 100.74x | 5.44x | 5.98x | 0.26x |
| `mat4 add scalar` | 20.40 ns | 3.98 us | 6.75 us | 1.40 us | 1.18 us | 20.03 us | 194.95x | 2.83x | 3.36x | 0.20x |
| `mat4 sub` | 38.08 ns | 5.30 us | 6.98 us | 996.41 ns | 889.83 ns | 35.61 us | 139.23x | 5.32x | 5.96x | 0.15x |
| `mat4 sub scalar` | 15.10 ns | 4.24 us | 7.61 us | 1.44 us | 1.18 us | 36.95 us | 280.93x | 2.95x | 3.61x | 0.11x |
| `mat4 neg` | 14.32 ns | 963.98 ns | 977.79 ns | 926.10 ns | 736.10 ns | 13.67 us | 67.31x | 1.04x | 1.31x | 0.07x |
| `mat4 mul scalar` | 24.12 ns | 2.87 us | 7.38 us | 1.45 us | 1.11 us | 19.84 us | 118.94x | 1.98x | 2.59x | 0.14x |
| `mat4 div scalar` | 33.81 ns | 7.38 us | 7.55 us | 2.62 us | 1.44 us | 36.99 us | 218.24x | 2.82x | 5.13x | 0.20x |
| `mat4 div matrix` | 205.70 ns | 28.05 us | 95.14 us | 17.05 us | 14.39 us | 526.21 us | 136.36x | 1.65x | 1.95x | 0.05x |
| `mat4 bitxor` | 263.67 ns | 11.42 us | 58.29 us | 15.37 us | 13.98 us | 339.68 us | 43.32x | 0.74x | 0.82x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.57 ns | 175.93 ns | 279.11 ns | - | - | 1.25 us | 49.32x | - | - | 0.14x |
| `scalar add ref_owned` | 12.21 ns | 172.67 ns | 277.20 ns | - | - | 1.24 us | 14.14x | - | - | 0.14x |
| `scalar add refs` | 5.23 ns | 154.83 ns | 257.63 ns | - | - | 1.25 us | 29.61x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 8.83 ns | 189.53 ns | 296.15 ns | - | - | - | 21.47x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.44 ns | 183.89 ns | 292.44 ns | - | - | - | 33.80x | - | - | - |
| `scalar sub owned_ref` | 3.92 ns | 217.83 ns | 466.05 ns | - | - | 2.35 us | 55.52x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.21 ns | 214.99 ns | 460.93 ns | - | - | 2.37 us | 17.60x | - | - | 0.09x |
| `scalar sub refs` | 5.58 ns | 198.73 ns | 439.00 ns | - | - | 2.36 us | 35.61x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 8.67 ns | 227.28 ns | 472.75 ns | - | - | - | 26.21x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.60 ns | 227.38 ns | 468.78 ns | - | - | - | 40.62x | - | - | - |
| `scalar mul owned_ref` | 4.18 ns | 95.54 ns | 363.13 ns | - | - | 1.49 us | 22.86x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.11 ns | 95.11 ns | 365.73 ns | - | - | 1.49 us | 7.26x | - | - | 0.06x |
| `scalar mul refs` | 6.00 ns | 81.00 ns | 352.20 ns | - | - | 1.48 us | 13.49x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.63 ns | 106.63 ns | 387.73 ns | - | - | - | 11.07x | - | - | - |
| `scalar mul ref_owned_with_clone` | 7.67 ns | 105.49 ns | 387.10 ns | - | - | - | 13.76x | - | - | - |
| `scalar div owned_ref` | 5.88 ns | 258.77 ns | 310.12 ns | - | - | 2.49 us | 44.00x | - | - | 0.10x |
| `scalar div ref_owned` | 16.84 ns | 257.88 ns | 310.00 ns | - | - | 2.48 us | 15.31x | - | - | 0.10x |
| `scalar div refs` | 6.78 ns | 249.06 ns | 300.76 ns | - | - | 2.47 us | 36.76x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 15.02 ns | 274.67 ns | 330.00 ns | - | - | - | 18.29x | - | - | - |
| `scalar div ref_owned_with_clone` | 7.85 ns | 271.22 ns | 333.73 ns | - | - | - | 34.56x | - | - | - |
| `vec3 add refs` | 6.16 ns | 611.40 ns | 1.38 us | - | - | 3.84 us | 99.22x | - | - | 0.16x |
| `vec3 sub refs` | 6.16 ns | 720.09 ns | 2.17 us | - | - | 7.17 us | 116.86x | - | - | 0.10x |
| `vec3 neg ref` | 3.26 ns | 150.93 ns | 157.95 ns | - | - | 3.02 us | 46.35x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.45 ns | 741.96 ns | 1.44 us | - | - | 3.69 us | 115.11x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.37 ns | 849.17 ns | 1.94 us | - | - | 6.87 us | 133.25x | - | - | 0.12x |
| `vec3 mul_scalar_ref` | 6.78 ns | 490.05 ns | 2.00 us | - | - | 4.26 us | 72.29x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.00 ns | 1.21 us | 1.50 us | - | - | 7.35 us | 151.53x | - | - | 0.16x |
| `vec4 add refs` | 6.43 ns | 765.81 ns | 1.41 us | - | - | 5.07 us | 119.16x | - | - | 0.15x |
| `vec4 sub refs` | 3.02 ns | 853.27 ns | 1.85 us | - | - | 9.24 us | 282.25x | - | - | 0.09x |
| `vec4 neg ref` | 4.17 ns | 201.42 ns | 201.23 ns | - | - | 3.88 us | 48.32x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 6.92 ns | 957.38 ns | 1.66 us | - | - | 4.92 us | 138.34x | - | - | 0.19x |
| `vec4 sub_scalar_ref` | 4.16 ns | 1.07 us | 2.02 us | - | - | 9.04 us | 257.17x | - | - | 0.12x |
| `vec4 mul_scalar_ref` | 7.29 ns | 627.31 ns | 2.02 us | - | - | 5.52 us | 86.05x | - | - | 0.11x |
| `vec4 div_scalar_ref` | 12.14 ns | 1.69 us | 1.51 us | - | - | 9.64 us | 139.17x | - | - | 0.18x |
| `mat3 add refs` | 13.98 ns | 1.68 us | 3.81 us | - | - | 11.16 us | 120.44x | - | - | 0.15x |
| `mat3 sub refs` | 13.55 ns | 1.92 us | 5.67 us | - | - | 20.36 us | 141.91x | - | - | 0.09x |
| `mat3 mul refs` | 55.82 ns | 2.24 us | 14.72 us | - | - | 59.38 us | 40.05x | - | - | 0.04x |
| `mat3 div refs` | 142.49 ns | 17.06 us | 65.70 us | - | - | 155.52 us | 119.70x | - | - | 0.11x |
| `mat3 neg ref` | 9.86 ns | 480.80 ns | 499.09 ns | - | - | 8.33 us | 48.76x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 50.88 ns | 2.36 us | 4.53 us | - | - | 11.60 us | 46.33x | - | - | 0.20x |
| `mat3 sub_scalar_ref` | 51.06 ns | 2.64 us | 6.38 us | - | - | 21.13 us | 51.76x | - | - | 0.13x |
| `mat3 mul_scalar_ref` | 53.81 ns | 1.70 us | 5.77 us | - | - | 11.90 us | 31.65x | - | - | 0.14x |
| `mat3 div_scalar_ref` | 24.28 ns | 4.20 us | 5.54 us | - | - | 21.15 us | 173.04x | - | - | 0.20x |
| `mat4 add refs` | 26.62 ns | 2.75 us | 3.61 us | - | - | 18.94 us | 103.13x | - | - | 0.14x |
| `mat4 sub refs` | 27.95 ns | 3.01 us | 4.72 us | - | - | 35.01 us | 107.68x | - | - | 0.09x |
| `mat4 mul refs` | 126.00 ns | 4.09 us | 19.38 us | - | - | 136.24 us | 32.43x | - | - | 0.03x |
| `mat4 div refs` | 195.56 ns | 27.10 us | 93.58 us | - | - | 515.65 us | 138.57x | - | - | 0.05x |
| `mat4 neg ref` | 11.93 ns | 724.89 ns | 714.41 ns | - | - | 13.58 us | 60.74x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 52.99 ns | 3.92 us | 6.79 us | - | - | 19.82 us | 73.91x | - | - | 0.20x |
| `mat4 sub_scalar_ref` | 39.05 ns | 4.17 us | 7.66 us | - | - | 36.04 us | 106.73x | - | - | 0.12x |
| `mat4 mul_scalar_ref` | 55.07 ns | 2.80 us | 7.41 us | - | - | 19.56 us | 50.92x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 26.64 ns | 7.45 us | 7.48 us | - | - | 36.29 us | 279.64x | - | - | 0.21x |
| `mat3 transform_vec refs` | 26.89 ns | 925.48 ns | 4.97 us | - | - | 19.44 us | 34.42x | - | - | 0.05x |
| `mat4 transform_vec refs` | 42.97 ns | 1.43 us | 5.78 us | - | - | 33.53 us | 33.18x | - | - | 0.04x |
| `complex add refs` | 7.56 ns | 327.37 ns | 699.32 ns | - | - | 2.47 us | 43.28x | - | - | 0.13x |
| `complex sub refs` | 7.92 ns | 373.04 ns | 960.99 ns | - | - | 4.62 us | 47.12x | - | - | 0.08x |
| `complex mul refs` | 7.95 ns | 754.54 ns | 3.28 us | - | - | 9.66 us | 94.88x | - | - | 0.08x |
| `complex div refs` | 16.40 ns | 2.55 us | 6.25 us | - | - | 20.82 us | 155.19x | - | - | 0.12x |
| `complex neg ref` | 2.32 ns | 70.94 ns | 70.49 ns | - | - | 2.02 us | 30.55x | - | - | 0.04x |
| `complex div_real_ref` | 7.12 ns | 605.21 ns | 588.62 ns | - | - | 5.00 us | 84.98x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.02 us |
| `astro sin 160` | 13.14 us |
| `astro sin 192` | 13.04 us |
| `astro sin 256` | 15.89 us |
