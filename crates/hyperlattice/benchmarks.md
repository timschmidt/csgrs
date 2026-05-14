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
| `sin 0.1` | 10.81 ns | 161.95 ns | 162.27 ns | 10.57 us | 757.13 ns | 1.89 us | 14.99x | 0.02x | 0.21x | 0.09x |
| `cos 0.1` | 11.57 ns | 162.55 ns | 163.11 ns | 10.15 us | 495.36 ns | 1.73 us | 14.05x | 0.02x | 0.33x | 0.09x |
| `sin 1.23456789` | 11.55 ns | 217.36 ns | 204.55 ns | 12.08 us | 802.64 ns | 1.84 us | 18.82x | 0.02x | 0.27x | 0.12x |
| `cos 1.23456789` | 11.98 ns | 211.04 ns | 197.32 ns | 10.25 us | 587.35 ns | 1.68 us | 17.62x | 0.02x | 0.36x | 0.13x |
| `sin 1e6` | 12.59 ns | 105.72 ns | 106.00 ns | 15.78 us | 1.09 us | 2.05 us | 8.39x | 0.01x | 0.10x | 0.05x |
| `cos 1e6` | 12.34 ns | 107.03 ns | 106.38 ns | 13.41 us | 822.97 ns | 1.85 us | 8.67x | 0.01x | 0.13x | 0.06x |
| `sin 1e30` | 65.71 ns | 105.26 ns | 105.40 ns | 18.10 us | 2.87 us | 3.55 us | 1.60x | 0.01x | 0.04x | 0.03x |
| `cos 1e30` | 68.52 ns | 106.74 ns | 107.30 ns | 15.13 us | 979.22 ns | 3.14 us | 1.56x | 0.01x | 0.11x | 0.03x |
| `sin pi_7` | 11.57 ns | 163.18 ns | 379.12 ns | 11.75 us | 742.59 ns | 1.92 us | 14.10x | 0.01x | 0.22x | 0.09x |
| `cos pi_7` | 11.55 ns | 162.64 ns | 724.40 ns | 10.44 us | 539.38 ns | 1.76 us | 14.08x | 0.02x | 0.30x | 0.09x |
| `sin 1000pi_eps` | 11.56 ns | 106.08 ns | 750.18 ns | 15.86 us | 2.27 us | 2.79 us | 9.18x | 0.01x | 0.05x | 0.04x |
| `cos 1000pi_eps` | 12.24 ns | 107.01 ns | 743.59 ns | 13.45 us | 570.45 ns | 1.68 us | 8.75x | 0.01x | 0.19x | 0.06x |
| `asin 0.5` | 10.83 ns | 144.42 ns | 144.43 ns | 50.00 us | 2.91 us | 13.14 us | 13.33x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.12 ns | 429.26 ns | 429.90 ns | 58.92 us | 2.91 us | 13.16 us | 38.59x | 0.01x | 0.15x | 0.03x |
| `atanh 0.5` | 14.65 ns | 240.31 ns | 239.51 ns | 34.27 us | 1.63 us | 12.88 us | 16.41x | 0.01x | 0.15x | 0.02x |
| `asin neg_0.999999` | 13.91 ns | 574.39 ns | 554.66 ns | 13.82 us | 2.47 us | 12.89 us | 41.30x | 0.04x | 0.23x | 0.04x |
| `acos neg_0.999999` | 15.07 ns | 530.70 ns | 501.78 ns | 18.10 us | 2.67 us | 13.02 us | 35.21x | 0.03x | 0.20x | 0.04x |
| `atanh neg_0.999999` | 14.56 ns | 212.45 ns | 198.90 ns | 36.68 us | 1.59 us | 12.59 us | 14.59x | 0.01x | 0.13x | 0.02x |
| `asin 0.999999` | 13.57 ns | 520.91 ns | 750.73 ns | 14.17 us | 2.48 us | 12.74 us | 38.40x | 0.04x | 0.21x | 0.04x |
| `acos 0.999999` | 14.08 ns | 311.53 ns | 530.49 ns | 18.43 us | 2.72 us | 12.88 us | 22.12x | 0.02x | 0.11x | 0.02x |
| `atanh 0.999999` | 14.70 ns | 220.39 ns | 205.70 ns | 31.57 us | 1.58 us | 12.56 us | 14.99x | 0.01x | 0.14x | 0.02x |
| `asin 1e-12` | 9.29 ns | 275.92 ns | 482.04 ns | 7.99 us | 1.40 us | 15.19 us | 29.70x | 0.03x | 0.20x | 0.02x |
| `acos 1e-12` | 9.99 ns | 475.36 ns | 662.56 ns | 9.50 us | 1.42 us | 15.18 us | 47.61x | 0.05x | 0.34x | 0.03x |
| `atanh 1e-12` | 9.58 ns | 246.33 ns | 220.64 ns | 36.28 us | 167.39 ns | 20.11 us | 25.72x | 0.01x | 1.47x | 0.01x |
| `atan 0.5` | 14.69 ns | 167.17 ns | 166.90 ns | 34.38 us | 2.74 us | 17.68 us | 11.38x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.29 ns | 275.21 ns | 272.27 ns | 39.20 us | 1.61 us | 7.51 us | 10.47x | 0.01x | 0.17x | 0.04x |
| `atan neg_1e-12` | 14.04 ns | 287.54 ns | 274.55 ns | 1.58 us | 1.12 us | 15.22 us | 20.48x | 0.18x | 0.26x | 0.02x |
| `asinh neg_1e-12` | 15.68 ns | 482.05 ns | 393.38 ns | 42.03 us | 8.55 us | 11.92 us | 30.74x | 0.01x | 0.06x | 0.04x |
| `atan 1e6` | 15.23 ns | 165.54 ns | 166.91 ns | 2.80 us | 1.41 us | 17.81 us | 10.87x | 0.06x | 0.12x | 0.01x |
| `asinh 1e6` | 26.42 ns | 263.88 ns | 264.32 ns | 36.93 us | 1.67 us | 7.15 us | 9.99x | 0.01x | 0.16x | 0.04x |
| `atan neg_1e6` | 15.10 ns | 272.77 ns | 271.96 ns | 2.86 us | 1.42 us | 18.02 us | 18.07x | 0.10x | 0.19x | 0.02x |
| `asinh neg_1e6` | 26.18 ns | 378.82 ns | 379.68 ns | 36.97 us | 1.66 us | 7.08 us | 14.47x | 0.01x | 0.23x | 0.05x |
| `acosh 9` | 12.33 ns | 166.84 ns | 166.78 ns | 43.03 us | 1.64 us | 9.75 us | 13.53x | 0.00x | 0.10x | 0.02x |
| `acosh 1_plus_1e-12` | 13.49 ns | 242.61 ns | 243.02 ns | 42.05 us | 8.38 us | 11.31 us | 17.99x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.28 ns | 166.40 ns | 166.79 ns | 37.14 us | 1.58 us | 9.80 us | 13.55x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.17 ns | 241.22 ns | 2.30 us | 41.30 us | 1.65 us | 9.66 us | 19.81x | 0.01x | 0.15x | 0.02x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.46 ns | 18.71 ns | 24.90 ns | 15.59 ns | 0.94 ns | 39.16x | 0.74x | 1.18x | 19.59x |
| `one` | 0.47 ns | 22.93 ns | 24.23 ns | 42.12 ns | 30.47 ns | 29.84 ns | 48.76x | 0.54x | 0.75x | 0.77x |
| `e` | 0.47 ns | 90.04 ns | 90.33 ns | 53.79 ns | 1.08 us | 225.89 ns | 189.88x | 1.67x | 0.08x | 0.40x |
| `pi` | 0.47 ns | 51.02 ns | 52.11 ns | 47.97 ns | 48.29 ns | 224.89 ns | 107.70x | 1.06x | 1.06x | 0.23x |
| `tau` | 0.48 ns | 51.77 ns | 51.50 ns | 115.50 ns | 100.86 ns | 1.89 us | 108.96x | 0.45x | 0.51x | 0.03x |
| `add` | 5.14 ns | 219.04 ns | 327.07 ns | 52.47 ns | 43.36 ns | 1.30 us | 42.63x | 4.17x | 5.05x | 0.17x |
| `sub` | 5.17 ns | 258.37 ns | 513.76 ns | 56.64 ns | 46.12 ns | 2.43 us | 50.00x | 4.56x | 5.60x | 0.11x |
| `neg` | 5.09 ns | 39.79 ns | 39.65 ns | 23.15 ns | 21.53 ns | 1.06 us | 7.81x | 1.72x | 1.85x | 0.04x |
| `mul` | 5.42 ns | 137.77 ns | 415.66 ns | 64.23 ns | 46.35 ns | 1.54 us | 25.42x | 2.15x | 2.97x | 0.09x |
| `div` | 9.00 ns | 312.61 ns | 366.52 ns | 160.67 ns | 62.93 ns | 2.55 us | 34.72x | 1.95x | 4.97x | 0.12x |
| `reciprocal` | 8.09 ns | 73.96 ns | 76.14 ns | 159.51 ns | 58.37 ns | 1.60 us | 9.14x | 0.46x | 1.27x | 0.05x |
| `reciprocal checked` | 8.61 ns | 68.66 ns | 72.52 ns | 160.73 ns | 58.37 ns | 1.55 us | 7.97x | 0.43x | 1.18x | 0.04x |
| `reciprocal checked abort` | 21.77 ns | 81.37 ns | 79.33 ns | 161.32 ns | 58.34 ns | 1.56 us | 3.74x | 0.50x | 1.39x | 0.05x |
| `pow` | 29.93 ns | 6.72 us | 5.83 us | 55.04 us | 2.85 us | 2.37 us | 224.63x | 0.12x | 2.36x | 2.84x |
| `powi` | 5.90 ns | 374.68 ns | 2.68 us | 280.42 ns | 82.77 ns | 1.56 us | 63.46x | 1.34x | 4.53x | 0.24x |
| `exp` | 19.40 ns | 236.16 ns | 235.42 ns | 14.01 us | 915.79 ns | 1.91 us | 12.17x | 0.02x | 0.26x | 0.12x |
| `ln` | 10.71 ns | 1.49 us | 1.42 us | 30.24 us | 1.32 us | 1.83 us | 138.84x | 0.05x | 1.13x | 0.81x |
| `log10` | 13.29 ns | 1.65 us | 1.57 us | 35.21 us | 2.76 us | 6.68 us | 124.33x | 0.05x | 0.60x | 0.25x |
| `log10 abort` | 16.85 ns | 1.66 us | 1.59 us | 36.31 us | 2.77 us | 6.59 us | 98.47x | 0.05x | 0.60x | 0.25x |
| `sqrt` | 20.03 ns | 1.51 us | 1.65 us | 4.98 us | 95.34 ns | 1.47 us | 75.20x | 0.30x | 15.80x | 1.03x |
| `sin` | 14.95 ns | 122.70 ns | 119.76 ns | 13.88 us | 1.23 us | 2.27 us | 8.21x | 0.01x | 0.10x | 0.05x |
| `cos` | 18.38 ns | 122.54 ns | 119.41 ns | 12.07 us | 631.77 ns | 1.77 us | 6.67x | 0.01x | 0.19x | 0.07x |
| `tan` | 24.70 ns | 166.94 ns | 173.85 ns | 29.39 us | 1.57 us | 6.64 us | 6.76x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.12 ns | 3.66 us | 3.62 us | 3.36 us | 1.12 us | 10.67 us | 201.88x | 1.09x | 3.27x | 0.34x |
| `cosh` | 18.18 ns | 3.61 us | 3.62 us | 7.97 us | 1.06 us | 9.57 us | 198.76x | 0.45x | 3.42x | 0.38x |
| `tanh` | 20.32 ns | 6.08 us | 6.37 us | 3.32 us | 1.20 us | 23.18 us | 299.17x | 1.83x | 5.07x | 0.26x |
| `asin` | 13.10 ns | 381.56 ns | 614.06 ns | 21.28 us | 2.42 us | 13.90 us | 29.12x | 0.02x | 0.16x | 0.03x |
| `asin abort` | 17.27 ns | 383.32 ns | 605.42 ns | 21.35 us | 2.41 us | 13.90 us | 22.20x | 0.02x | 0.16x | 0.03x |
| `acos` | 13.89 ns | 451.32 ns | 680.33 ns | 27.41 us | 2.51 us | 13.81 us | 32.50x | 0.02x | 0.18x | 0.03x |
| `acos abort` | 17.71 ns | 459.35 ns | 666.91 ns | 26.88 us | 2.50 us | 13.85 us | 25.93x | 0.02x | 0.18x | 0.03x |
| `atan` | 14.85 ns | 167.72 ns | 164.12 ns | 18.89 us | 2.29 us | 18.73 us | 11.29x | 0.01x | 0.07x | 0.01x |
| `atan abort` | 21.24 ns | 167.55 ns | 164.17 ns | 18.71 us | 2.24 us | 18.87 us | 7.89x | 0.01x | 0.07x | 0.01x |
| `asinh` | 32.77 ns | 264.77 ns | 270.25 ns | 39.04 us | 1.63 us | 7.57 us | 8.08x | 0.01x | 0.16x | 0.03x |
| `asinh abort` | 30.66 ns | 264.67 ns | 274.47 ns | 39.25 us | 1.63 us | 7.52 us | 8.63x | 0.01x | 0.16x | 0.04x |
| `acosh` | 12.29 ns | 197.38 ns | 199.15 ns | 40.78 us | 3.32 us | 10.35 us | 16.06x | 0.00x | 0.06x | 0.02x |
| `acosh abort` | 15.88 ns | 201.88 ns | 201.66 ns | 40.62 us | 3.31 us | 10.46 us | 12.71x | 0.00x | 0.06x | 0.02x |
| `atanh` | 13.78 ns | 227.08 ns | 250.74 ns | 34.99 us | 1.30 us | 14.82 us | 16.48x | 0.01x | 0.17x | 0.02x |
| `atanh abort` | 16.90 ns | 229.76 ns | 249.65 ns | 34.82 us | 1.27 us | 14.84 us | 13.60x | 0.01x | 0.18x | 0.02x |
| `zero status` | 1.20 ns | 1.04 ns | 1.00 ns | 1.07 ns | 6.82 ns | 7.94 ns | 0.86x | 0.97x | 0.15x | 0.13x |
| `zero status abort` | 1.41 ns | 1.12 ns | 1.12 ns | 1.05 ns | 6.75 ns | 7.94 ns | 0.79x | 1.07x | 0.17x | 0.14x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 38.37 ns | 38.23 ns | 51.04 ns | - | 1.88 ns | 40.86x | 0.75x | - | 20.37x |
| `one` | 5.25 ns | 44.44 ns | 43.66 ns | 66.49 ns | - | 32.29 ns | 8.46x | 0.67x | - | 1.38x |
| `i` | 0.94 ns | 45.19 ns | 44.77 ns | 65.07 ns | - | 29.52 ns | 48.20x | 0.69x | - | 1.53x |
| `free i` | 0.94 ns | 45.42 ns | 44.79 ns | 64.83 ns | - | 29.43 ns | 48.56x | 0.70x | - | 1.54x |
| `conjugate` | 2.12 ns | 87.32 ns | 86.40 ns | 40.57 ns | - | 1.07 us | 41.09x | 2.15x | - | 0.08x |
| `norm squared` | 5.74 ns | 165.59 ns | 562.48 ns | 154.04 ns | - | 4.28 us | 28.83x | 1.07x | - | 0.04x |
| `reciprocal` | 12.51 ns | 1.73 us | 2.83 us | 450.93 ns | - | 10.70 us | 138.61x | 3.85x | - | 0.16x |
| `reciprocal checked` | 14.76 ns | 1.75 us | 2.84 us | 450.25 ns | - | 10.71 us | 118.63x | 3.89x | - | 0.16x |
| `powi` | 18.34 ns | 1.34 us | 6.75 us | 1.43 us | - | 43.43 us | 73.12x | 0.94x | - | 0.03x |
| `powi checked` | 18.00 ns | 1.34 us | 6.74 us | 1.43 us | - | 44.09 us | 74.57x | 0.94x | - | 0.03x |
| `div checked` | 16.79 ns | 2.39 us | 4.70 us | 817.54 ns | - | 21.69 us | 142.34x | 2.92x | - | 0.11x |
| `div real checked` | 17.54 ns | 665.32 ns | 647.11 ns | 265.81 ns | - | 5.28 us | 37.93x | 2.50x | - | 0.13x |
| `from scalar` | 1.42 ns | 59.78 ns | 58.67 ns | 44.96 ns | - | 10.40 ns | 42.19x | 1.33x | - | 5.75x |
| `add` | 5.84 ns | 480.71 ns | 845.76 ns | 108.46 ns | - | 2.56 us | 82.27x | 4.43x | - | 0.19x |
| `sub` | 5.83 ns | 532.02 ns | 1.13 us | 116.36 ns | - | 4.78 us | 91.18x | 4.57x | - | 0.11x |
| `neg` | 2.59 ns | 88.25 ns | 87.25 ns | 41.99 ns | - | 2.13 us | 34.04x | 2.10x | - | 0.04x |
| `mul` | 7.43 ns | 932.86 ns | 3.61 us | 307.29 ns | - | 10.00 us | 125.49x | 3.04x | - | 0.09x |
| `div` | 15.30 ns | 2.39 us | 4.72 us | 818.21 ns | - | 21.52 us | 156.33x | 2.92x | - | 0.11x |
| `div real` | 9.99 ns | 668.02 ns | 645.23 ns | 266.83 ns | - | 5.17 us | 66.84x | 2.50x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.70 ns | 229.16 ns | 2.39 us | 312.51 ns | 252.69 ns | 7.16 us | 26.34x | 0.73x | 0.91x | 0.03x |
| `vec3 magnitude` | 11.23 ns | 3.98 us | 5.44 us | 5.35 us | 348.35 ns | 8.65 us | 354.31x | 0.74x | 11.43x | 0.46x |
| `vec3 normalize` | 24.12 ns | 8.18 us | 11.50 us | 6.00 us | 582.96 ns | 16.63 us | 339.05x | 1.36x | 14.03x | 0.49x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.05 ns | 285.32 ns | 1.44 us | 68.72 ns | 57.19 ns | 721.51 ns | 93.41x | 4.15x | 4.99x | 0.40x |
| `vec3 zero` | 1.40 ns | 132.54 ns | 131.47 ns | 64.41 ns | 31.00 ns | 2.81 ns | 94.94x | 2.06x | 4.27x | 47.12x |
| `vec3 dot abort` | 8.51 ns | 228.57 ns | 906.60 ns | 259.80 ns | 200.09 ns | 7.08 us | 26.85x | 0.88x | 1.14x | 0.03x |
| `vec3 magnitude abort` | 17.17 ns | 4.05 us | 3.19 us | 5.48 us | 326.19 ns | 8.74 us | 235.76x | 0.74x | 12.41x | 0.46x |
| `vec3 normalize checked` | 25.33 ns | 8.30 us | 7.25 us | 5.87 us | 535.45 ns | 16.85 us | 327.48x | 1.41x | 15.49x | 0.49x |
| `vec3 normalize checked abort` | 29.57 ns | 8.31 us | 7.25 us | 5.87 us | 536.22 ns | 16.70 us | 281.08x | 1.42x | 15.50x | 0.50x |
| `vec3 div scalar checked` | 9.70 ns | 1.34 us | 1.63 us | 398.65 ns | - | - | 138.26x | 3.37x | - | - |
| `vec3 div scalar checked abort` | 18.14 ns | 1.35 us | 1.56 us | 399.61 ns | - | - | 74.55x | 3.39x | - | - |
| `vec3 add` | 6.73 ns | 961.97 ns | 1.72 us | 150.78 ns | 124.68 ns | 3.92 us | 142.99x | 6.38x | 7.72x | 0.25x |
| `vec3 add scalar` | 6.35 ns | 765.16 ns | 1.44 us | 154.27 ns | 131.79 ns | 3.78 us | 120.49x | 4.96x | 5.81x | 0.20x |
| `vec3 sub` | 6.72 ns | 1.08 us | 2.53 us | 164.33 ns | 135.75 ns | 7.32 us | 160.77x | 6.58x | 7.96x | 0.15x |
| `vec3 sub scalar` | 6.38 ns | 881.52 ns | 1.95 us | 147.00 ns | 123.07 ns | 7.04 us | 138.23x | 6.00x | 7.16x | 0.13x |
| `vec3 neg` | 3.76 ns | 142.16 ns | 141.81 ns | 58.61 ns | 50.27 ns | 3.13 us | 37.81x | 2.43x | 2.83x | 0.05x |
| `vec3 mul scalar` | 6.93 ns | 524.04 ns | 2.00 us | 168.40 ns | 124.97 ns | 4.30 us | 75.66x | 3.11x | 4.19x | 0.12x |
| `vec3 div scalar` | 9.49 ns | 1.35 us | 1.61 us | 394.07 ns | 175.03 ns | 7.57 us | 142.55x | 3.43x | 7.73x | 0.18x |
| `vec4 dot` | 9.59 ns | 265.48 ns | 648.45 ns | 442.43 ns | 314.76 ns | 9.77 us | 27.68x | 0.60x | 0.84x | 0.03x |
| `vec4 magnitude` | 16.88 ns | 3.84 us | 2.59 us | 5.59 us | 406.27 ns | 11.24 us | 227.67x | 0.69x | 9.46x | 0.34x |
| `vec4 normalize` | 36.27 ns | 8.51 us | 6.36 us | 6.18 us | 699.27 ns | 21.89 us | 234.69x | 1.38x | 12.17x | 0.39x |
| `vec4 add` | 7.24 ns | 1.18 us | 1.82 us | 203.90 ns | 174.93 ns | 5.25 us | 163.40x | 5.81x | 6.77x | 0.23x |
| `vec4 add scalar` | 6.88 ns | 1.06 us | 1.80 us | 211.65 ns | 176.09 ns | 5.06 us | 154.09x | 5.01x | 6.02x | 0.21x |
| `vec4 sub` | 5.03 ns | 1.29 us | 2.27 us | 209.30 ns | 173.38 ns | 9.56 us | 256.26x | 6.16x | 7.44x | 0.13x |
| `vec4 sub scalar` | 4.44 ns | 1.15 us | 2.10 us | 203.01 ns | 168.99 ns | 9.32 us | 260.21x | 5.69x | 6.83x | 0.12x |
| `vec4 neg` | 4.91 ns | 193.76 ns | 190.47 ns | 79.27 ns | 64.27 ns | 4.01 us | 39.46x | 2.44x | 3.01x | 0.05x |
| `vec4 mul scalar` | 7.26 ns | 716.25 ns | 2.12 us | 219.51 ns | 157.70 ns | 5.66 us | 98.60x | 3.26x | 4.54x | 0.13x |
| `vec4 div scalar` | 13.34 ns | 1.76 us | 1.54 us | 537.71 ns | 221.69 ns | 9.92 us | 131.81x | 3.27x | 7.93x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.27 ns | 772.51 ns | 2.28 us | 949.35 ns | 844.43 ns | 22.02 us | 40.08x | 0.81x | 0.91x | 0.04x |
| `mat3 inverse` | 85.64 ns | 14.72 us | 8.73 us | 3.13 us | 2.43 us | 80.49 us | 171.86x | 4.70x | 6.06x | 0.18x |
| `mat3 mul mat3` | 75.62 ns | 3.40 us | 7.89 us | 2.80 us | 2.35 us | 60.48 us | 44.92x | 1.21x | 1.45x | 0.06x |
| `mat3 transform vec3` | 27.95 ns | 1.65 us | 5.01 us | 1.05 us | 883.73 ns | 19.94 us | 59.13x | 1.58x | 1.87x | 0.08x |
| `mat4 determinant` | 42.64 ns | 2.24 us | 1.55 us | 4.57 us | 4.12 us | 93.94 us | 52.48x | 0.49x | 0.54x | 0.02x |
| `mat4 inverse` | 178.10 ns | 24.92 us | 8.48 us | 11.19 us | 9.14 us | 344.37 us | 139.93x | 2.23x | 2.73x | 0.07x |
| `mat4 mul mat4` | 156.82 ns | 6.00 us | 5.08 us | 6.43 us | 5.34 us | 142.60 us | 38.28x | 0.93x | 1.12x | 0.04x |
| `mat4 transform vec4` | 45.99 ns | 2.61 us | 2.45 us | 1.91 us | 1.62 us | 35.00 us | 56.80x | 1.37x | 1.61x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.24 ns | 1.30 us | 4.39 us | 214.21 ns | 230.55 ns | 2.15 us | 36.87x | 6.07x | 5.64x | 0.60x |
| `mat3 zero` | 16.09 ns | 518.05 ns | 525.88 ns | 262.55 ns | 210.79 ns | 11.63 ns | 32.20x | 1.97x | 2.46x | 44.53x |
| `mat3 identity` | 9.76 ns | 615.94 ns | 623.58 ns | 319.59 ns | 242.61 ns | 139.94 ns | 63.09x | 1.93x | 2.54x | 4.40x |
| `mat3 transpose` | 8.99 ns | 682.00 ns | 687.43 ns | 238.98 ns | 206.56 ns | 120.89 ns | 75.90x | 2.85x | 3.30x | 5.64x |
| `mat3 reciprocal` | 86.55 ns | 14.69 us | 25.83 us | 2.91 us | 2.30 us | 82.30 us | 169.68x | 5.04x | 6.39x | 0.18x |
| `mat3 reciprocal checked` | 87.38 ns | 14.78 us | 24.83 us | 2.93 us | 2.30 us | 82.70 us | 169.17x | 5.05x | 6.43x | 0.18x |
| `mat3 inverse checked` | 87.01 ns | 14.89 us | 24.87 us | 2.90 us | 2.30 us | 82.39 us | 171.10x | 5.13x | 6.46x | 0.18x |
| `mat3 inverse checked abort` | 93.15 ns | 14.86 us | 25.67 us | 2.92 us | 2.32 us | 81.79 us | 159.50x | 5.09x | 6.42x | 0.18x |
| `mat3 powi` | 107.57 ns | 4.60 us | 38.41 us | 6.70 us | 6.23 us | 147.21 us | 42.72x | 0.69x | 0.74x | 0.03x |
| `mat3 powi checked` | 106.97 ns | 4.62 us | 38.33 us | 6.72 us | 6.11 us | 148.67 us | 43.21x | 0.69x | 0.76x | 0.03x |
| `mat3 powi checked abort` | 107.60 ns | 4.68 us | 38.14 us | 6.71 us | 6.12 us | 151.69 us | 43.48x | 0.70x | 0.76x | 0.03x |
| `mat3 div scalar checked` | 23.22 ns | 4.33 us | 5.77 us | 1.49 us | 817.33 ns | 22.02 us | 186.49x | 2.90x | 5.30x | 0.20x |
| `mat3 div scalar checked abort` | 28.36 ns | 4.33 us | 5.71 us | 1.48 us | 816.94 ns | 21.78 us | 152.75x | 2.92x | 5.30x | 0.20x |
| `mat3 div matrix checked` | 133.58 ns | 18.43 us | 43.74 us | 5.37 us | 4.51 us | 157.71 us | 138.00x | 3.43x | 4.09x | 0.12x |
| `mat3 div matrix checked abort` | 145.28 ns | 18.45 us | 44.00 us | 5.40 us | 4.50 us | 159.73 us | 127.00x | 3.42x | 4.10x | 0.12x |
| `mat3 add` | 14.58 ns | 3.28 us | 5.21 us | 517.92 ns | 487.26 ns | 11.65 us | 225.07x | 6.34x | 6.73x | 0.28x |
| `mat3 add scalar` | 11.99 ns | 2.56 us | 4.55 us | 770.38 ns | 703.70 ns | 11.94 us | 213.69x | 3.32x | 3.64x | 0.21x |
| `mat3 sub` | 12.97 ns | 3.71 us | 7.19 us | 537.05 ns | 512.76 ns | 21.14 us | 286.00x | 6.91x | 7.23x | 0.18x |
| `mat3 sub scalar` | 10.84 ns | 2.90 us | 6.39 us | 784.03 ns | 705.49 ns | 21.56 us | 267.74x | 3.70x | 4.12x | 0.13x |
| `mat3 neg` | 10.12 ns | 626.01 ns | 616.43 ns | 475.71 ns | 451.25 ns | 8.60 us | 61.85x | 1.32x | 1.39x | 0.07x |
| `mat3 mul scalar` | 13.48 ns | 1.83 us | 5.88 us | 763.76 ns | 663.13 ns | 12.16 us | 135.66x | 2.39x | 2.76x | 0.15x |
| `mat3 div scalar` | 23.34 ns | 4.25 us | 5.66 us | 1.48 us | 819.04 ns | 21.97 us | 182.12x | 2.86x | 5.19x | 0.19x |
| `mat3 div matrix` | 131.07 ns | 18.36 us | 43.52 us | 5.44 us | 4.54 us | 157.20 us | 140.05x | 3.37x | 4.05x | 0.12x |
| `mat3 bitxor` | 107.43 ns | 4.58 us | 38.50 us | 6.74 us | 6.15 us | 148.51 us | 42.66x | 0.68x | 0.75x | 0.03x |
| `mat4 zero` | 11.42 ns | 900.97 ns | 909.49 ns | 503.89 ns | 358.78 ns | 14.42 ns | 78.89x | 1.79x | 2.51x | 62.47x |
| `mat4 identity` | 10.76 ns | 1.09 us | 1.09 us | 621.79 ns | 421.24 ns | 215.00 ns | 101.74x | 1.76x | 2.60x | 5.09x |
| `mat4 transpose` | 9.23 ns | 1.06 us | 1.06 us | 472.71 ns | 367.94 ns | 180.43 ns | 115.22x | 2.25x | 2.89x | 5.89x |
| `mat4 reciprocal` | 165.14 ns | 24.78 us | 42.15 us | 10.90 us | 8.85 us | 345.46 us | 150.07x | 2.27x | 2.80x | 0.07x |
| `mat4 reciprocal checked` | 169.36 ns | 24.71 us | 42.44 us | 10.72 us | 8.82 us | 339.89 us | 145.91x | 2.30x | 2.80x | 0.07x |
| `mat4 powi` | 241.39 ns | 9.03 us | 53.04 us | 15.46 us | 14.09 us | 342.09 us | 37.41x | 0.58x | 0.64x | 0.03x |
| `mat4 powi checked` | 242.45 ns | 9.07 us | 52.61 us | 15.47 us | 13.94 us | 348.22 us | 37.39x | 0.59x | 0.65x | 0.03x |
| `mat4 add` | 51.45 ns | 5.14 us | 6.13 us | 956.30 ns | 854.94 ns | 19.71 us | 99.92x | 5.38x | 6.01x | 0.26x |
| `mat4 add scalar` | 20.48 ns | 4.04 us | 6.82 us | 1.40 us | 1.17 us | 20.62 us | 197.34x | 2.88x | 3.45x | 0.20x |
| `mat4 sub` | 38.64 ns | 5.43 us | 7.45 us | 986.86 ns | 906.50 ns | 35.77 us | 140.53x | 5.50x | 5.99x | 0.15x |
| `mat4 sub scalar` | 15.02 ns | 4.33 us | 7.61 us | 1.43 us | 1.17 us | 37.10 us | 288.48x | 3.03x | 3.71x | 0.12x |
| `mat4 neg` | 14.42 ns | 983.54 ns | 987.62 ns | 887.00 ns | 743.82 ns | 14.25 us | 68.22x | 1.11x | 1.32x | 0.07x |
| `mat4 mul scalar` | 24.30 ns | 2.94 us | 7.33 us | 1.42 us | 1.12 us | 20.01 us | 120.86x | 2.07x | 2.63x | 0.15x |
| `mat4 div scalar` | 31.68 ns | 7.48 us | 7.63 us | 2.61 us | 1.36 us | 37.59 us | 236.14x | 2.87x | 5.48x | 0.20x |
| `mat4 div matrix` | 197.94 ns | 29.68 us | 64.39 us | 16.88 us | 14.27 us | 537.28 us | 149.94x | 1.76x | 2.08x | 0.06x |
| `mat4 bitxor` | 242.38 ns | 9.52 us | 54.43 us | 15.47 us | 13.97 us | 349.06 us | 39.27x | 0.62x | 0.68x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 4.02 ns | 187.08 ns | 284.14 ns | - | - | 1.28 us | 46.59x | - | - | 0.15x |
| `scalar add ref_owned` | 12.47 ns | 184.44 ns | 278.29 ns | - | - | 1.30 us | 14.79x | - | - | 0.14x |
| `scalar add refs` | 5.52 ns | 188.54 ns | 266.47 ns | - | - | 1.29 us | 34.14x | - | - | 0.15x |
| `scalar add owned_ref_with_clone` | 9.24 ns | 235.09 ns | 301.51 ns | - | - | - | 25.45x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.64 ns | 232.55 ns | 296.12 ns | - | - | - | 41.21x | - | - | - |
| `scalar sub owned_ref` | 4.40 ns | 284.78 ns | 462.52 ns | - | - | 2.42 us | 64.74x | - | - | 0.12x |
| `scalar sub ref_owned` | 12.62 ns | 265.46 ns | 460.06 ns | - | - | 2.43 us | 21.03x | - | - | 0.11x |
| `scalar sub refs` | 5.78 ns | 239.87 ns | 442.60 ns | - | - | 2.42 us | 41.47x | - | - | 0.10x |
| `scalar sub owned_ref_with_clone` | 9.10 ns | 276.87 ns | 480.83 ns | - | - | - | 30.41x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.94 ns | 264.02 ns | 480.30 ns | - | - | - | 44.45x | - | - | - |
| `scalar mul owned_ref` | 4.57 ns | 126.29 ns | 371.66 ns | - | - | 1.52 us | 27.64x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.33 ns | 125.75 ns | 369.05 ns | - | - | 1.52 us | 9.43x | - | - | 0.08x |
| `scalar mul refs` | 6.37 ns | 94.42 ns | 359.52 ns | - | - | 1.55 us | 14.82x | - | - | 0.06x |
| `scalar mul owned_ref_with_clone` | 9.78 ns | 128.60 ns | 390.71 ns | - | - | - | 13.15x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.16 ns | 128.04 ns | 384.84 ns | - | - | - | 20.78x | - | - | - |
| `scalar div owned_ref` | 5.97 ns | 342.00 ns | 313.55 ns | - | - | 2.53 us | 57.26x | - | - | 0.14x |
| `scalar div ref_owned` | 17.29 ns | 355.93 ns | 314.90 ns | - | - | 2.56 us | 20.59x | - | - | 0.14x |
| `scalar div refs` | 7.52 ns | 317.73 ns | 305.14 ns | - | - | 2.53 us | 42.25x | - | - | 0.13x |
| `scalar div owned_ref_with_clone` | 15.90 ns | 324.06 ns | 333.91 ns | - | - | - | 20.38x | - | - | - |
| `scalar div ref_owned_with_clone` | 14.25 ns | 312.37 ns | 335.77 ns | - | - | - | 21.93x | - | - | - |
| `vec3 add refs` | 6.43 ns | 787.15 ns | 1.39 us | - | - | 3.93 us | 122.49x | - | - | 0.20x |
| `vec3 sub refs` | 6.47 ns | 742.49 ns | 2.19 us | - | - | 7.37 us | 114.84x | - | - | 0.10x |
| `vec3 neg ref` | 3.41 ns | 155.05 ns | 171.12 ns | - | - | 3.12 us | 45.51x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.76 ns | 776.53 ns | 1.45 us | - | - | 3.79 us | 114.85x | - | - | 0.21x |
| `vec3 sub_scalar_ref` | 6.77 ns | 871.11 ns | 1.98 us | - | - | 7.03 us | 128.58x | - | - | 0.12x |
| `vec3 mul_scalar_ref` | 7.01 ns | 501.80 ns | 2.00 us | - | - | 4.32 us | 71.63x | - | - | 0.12x |
| `vec3 div_scalar_ref` | 7.93 ns | 1.26 us | 1.50 us | - | - | 7.58 us | 158.48x | - | - | 0.17x |
| `vec4 add refs` | 6.48 ns | 757.21 ns | 1.39 us | - | - | 5.25 us | 116.82x | - | - | 0.14x |
| `vec4 sub refs` | 3.46 ns | 848.28 ns | 1.83 us | - | - | 9.82 us | 245.10x | - | - | 0.09x |
| `vec4 neg ref` | 4.42 ns | 201.78 ns | 203.48 ns | - | - | 4.04 us | 45.64x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 7.30 ns | 972.53 ns | 1.71 us | - | - | 5.15 us | 133.19x | - | - | 0.19x |
| `vec4 sub_scalar_ref` | 4.47 ns | 1.08 us | 2.05 us | - | - | 9.38 us | 241.93x | - | - | 0.12x |
| `vec4 mul_scalar_ref` | 7.72 ns | 628.36 ns | 2.06 us | - | - | 5.59 us | 81.35x | - | - | 0.11x |
| `vec4 div_scalar_ref` | 12.63 ns | 1.74 us | 1.55 us | - | - | 9.89 us | 138.07x | - | - | 0.18x |
| `mat3 add refs` | 11.75 ns | 1.78 us | 3.86 us | - | - | 11.70 us | 151.37x | - | - | 0.15x |
| `mat3 sub refs` | 10.96 ns | 2.02 us | 5.69 us | - | - | 21.31 us | 183.98x | - | - | 0.09x |
| `mat3 mul refs` | 55.75 ns | 2.37 us | 14.75 us | - | - | 60.94 us | 42.42x | - | - | 0.04x |
| `mat3 div refs` | 134.42 ns | 17.24 us | 42.60 us | - | - | 162.99 us | 128.23x | - | - | 0.11x |
| `mat3 neg ref` | 10.27 ns | 484.86 ns | 479.26 ns | - | - | 8.96 us | 47.22x | - | - | 0.05x |
| `mat3 add_scalar_ref` | 45.30 ns | 2.39 us | 4.57 us | - | - | 12.58 us | 52.78x | - | - | 0.19x |
| `mat3 sub_scalar_ref` | 45.42 ns | 2.68 us | 6.43 us | - | - | 24.43 us | 59.05x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 49.52 ns | 1.68 us | 5.65 us | - | - | 13.42 us | 33.90x | - | - | 0.13x |
| `mat3 div_scalar_ref` | 23.30 ns | 4.19 us | 5.52 us | - | - | 25.39 us | 179.91x | - | - | 0.17x |
| `mat4 add refs` | 17.38 ns | 2.79 us | 3.55 us | - | - | 22.22 us | 160.71x | - | - | 0.13x |
| `mat4 sub refs` | 17.15 ns | 2.98 us | 4.66 us | - | - | 38.86 us | 173.84x | - | - | 0.08x |
| `mat4 mul refs` | 149.18 ns | 4.44 us | 19.59 us | - | - | 155.35 us | 29.73x | - | - | 0.03x |
| `mat4 div refs` | 191.30 ns | 27.21 us | 63.03 us | - | - | 601.10 us | 142.23x | - | - | 0.05x |
| `mat4 neg ref` | 12.79 ns | 735.68 ns | 735.84 ns | - | - | 15.30 us | 57.51x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 51.31 ns | 4.05 us | 6.89 us | - | - | 23.00 us | 78.87x | - | - | 0.18x |
| `mat4 sub_scalar_ref` | 38.08 ns | 4.38 us | 7.59 us | - | - | 40.33 us | 114.91x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 54.12 ns | 2.86 us | 7.42 us | - | - | 21.42 us | 52.83x | - | - | 0.13x |
| `mat4 div_scalar_ref` | 26.87 ns | 7.45 us | 7.57 us | - | - | 41.50 us | 277.09x | - | - | 0.18x |
| `mat3 transform_vec refs` | 29.34 ns | 948.31 ns | 5.06 us | - | - | 22.05 us | 32.32x | - | - | 0.04x |
| `mat4 transform_vec refs` | 45.94 ns | 1.53 us | 5.93 us | - | - | 38.47 us | 33.24x | - | - | 0.04x |
| `complex add refs` | 7.58 ns | 330.91 ns | 700.23 ns | - | - | 2.77 us | 43.65x | - | - | 0.12x |
| `complex sub refs` | 7.99 ns | 381.66 ns | 994.53 ns | - | - | 5.46 us | 47.77x | - | - | 0.07x |
| `complex mul refs` | 8.09 ns | 800.19 ns | 3.44 us | - | - | 11.17 us | 98.86x | - | - | 0.07x |
| `complex div refs` | 16.21 ns | 2.26 us | 4.64 us | - | - | 24.43 us | 139.68x | - | - | 0.09x |
| `complex neg ref` | 2.40 ns | 72.45 ns | 71.97 ns | - | - | 2.28 us | 30.16x | - | - | 0.03x |
| `complex div_real_ref` | 7.34 ns | 630.55 ns | 608.01 ns | - | - | 6.09 us | 85.94x | - | - | 0.10x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 13.39 us |
| `astro sin 160` | 14.77 us |
| `astro sin 192` | 15.14 us |
| `astro sin 256` | 17.66 us |
