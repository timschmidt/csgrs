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
| `sin 0.1` | 11.05 ns | 143.68 ns | 143.54 ns | 10.81 us | 759.12 ns | 1.84 us | 13.00x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.60 ns | 143.68 ns | 143.78 ns | 10.63 us | 492.99 ns | 1.71 us | 12.39x | 0.01x | 0.29x | 0.08x |
| `sin 1.23456789` | 11.65 ns | 199.22 ns | 185.95 ns | 12.41 us | 796.65 ns | 1.83 us | 17.10x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 11.87 ns | 190.51 ns | 177.81 ns | 10.88 us | 588.44 ns | 1.66 us | 16.05x | 0.02x | 0.32x | 0.11x |
| `sin 1e6` | 12.52 ns | 87.76 ns | 88.13 ns | 16.08 us | 1.08 us | 2.00 us | 7.01x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 12.47 ns | 88.50 ns | 89.38 ns | 13.84 us | 813.62 ns | 1.82 us | 7.10x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 66.77 ns | 87.70 ns | 87.55 ns | 19.08 us | 2.82 us | 3.58 us | 1.31x | 0.00x | 0.03x | 0.02x |
| `cos 1e30` | 68.81 ns | 88.68 ns | 91.49 ns | 15.69 us | 962.78 ns | 3.05 us | 1.29x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 11.56 ns | 143.75 ns | 351.85 ns | 12.08 us | 744.16 ns | 1.87 us | 12.44x | 0.01x | 0.19x | 0.08x |
| `cos pi_7` | 11.60 ns | 144.06 ns | 712.34 ns | 10.95 us | 541.69 ns | 1.70 us | 12.42x | 0.01x | 0.27x | 0.08x |
| `sin 1000pi_eps` | 11.59 ns | 87.74 ns | 736.73 ns | 16.14 us | 2.29 us | 2.77 us | 7.57x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.48 ns | 88.77 ns | 742.75 ns | 13.93 us | 557.44 ns | 1.65 us | 7.12x | 0.01x | 0.16x | 0.05x |
| `asin 0.5` | 10.88 ns | 140.55 ns | 142.05 ns | 49.74 us | 2.91 us | 13.00 us | 12.92x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.22 ns | 416.15 ns | 421.02 ns | 59.39 us | 2.92 us | 13.02 us | 37.09x | 0.01x | 0.14x | 0.03x |
| `atanh 0.5` | 14.68 ns | 233.38 ns | 236.08 ns | 33.92 us | 1.61 us | 12.84 us | 15.90x | 0.01x | 0.15x | 0.02x |
| `asin neg_0.999999` | 13.88 ns | 572.64 ns | 547.46 ns | 14.04 us | 2.49 us | 12.75 us | 41.25x | 0.04x | 0.23x | 0.04x |
| `acos neg_0.999999` | 15.08 ns | 519.53 ns | 492.20 ns | 18.24 us | 2.65 us | 13.36 us | 34.46x | 0.03x | 0.20x | 0.04x |
| `atanh neg_0.999999` | 14.54 ns | 204.96 ns | 193.00 ns | 36.13 us | 1.58 us | 12.43 us | 14.10x | 0.01x | 0.13x | 0.02x |
| `asin 0.999999` | 13.60 ns | 510.91 ns | 726.78 ns | 13.71 us | 2.48 us | 12.49 us | 37.58x | 0.04x | 0.21x | 0.04x |
| `acos 0.999999` | 14.00 ns | 300.19 ns | 527.89 ns | 18.20 us | 2.71 us | 12.82 us | 21.45x | 0.02x | 0.11x | 0.02x |
| `atanh 0.999999` | 14.63 ns | 210.88 ns | 198.24 ns | 31.29 us | 1.56 us | 12.43 us | 14.42x | 0.01x | 0.13x | 0.02x |
| `asin 1e-12` | 9.29 ns | 269.87 ns | 472.62 ns | 7.90 us | 1.41 us | 15.35 us | 29.05x | 0.03x | 0.19x | 0.02x |
| `acos 1e-12` | 9.99 ns | 471.18 ns | 672.45 ns | 9.88 us | 1.40 us | 14.87 us | 47.17x | 0.05x | 0.34x | 0.03x |
| `atanh 1e-12` | 9.54 ns | 244.00 ns | 217.67 ns | 36.20 us | 170.69 ns | 19.90 us | 25.58x | 0.01x | 1.43x | 0.01x |
| `atan 0.5` | 14.57 ns | 163.78 ns | 165.36 ns | 35.23 us | 2.75 us | 17.22 us | 11.24x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.44 ns | 266.75 ns | 267.47 ns | 38.83 us | 1.59 us | 7.28 us | 10.09x | 0.01x | 0.17x | 0.04x |
| `atan neg_1e-12` | 14.18 ns | 284.73 ns | 280.40 ns | 1.60 us | 1.13 us | 14.93 us | 20.09x | 0.18x | 0.25x | 0.02x |
| `asinh neg_1e-12` | 15.60 ns | 470.19 ns | 378.50 ns | 41.82 us | 8.66 us | 11.70 us | 30.14x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 15.10 ns | 156.19 ns | 156.56 ns | 2.80 us | 1.41 us | 17.25 us | 10.34x | 0.06x | 0.11x | 0.01x |
| `asinh 1e6` | 26.59 ns | 257.74 ns | 259.24 ns | 36.91 us | 1.61 us | 7.03 us | 9.69x | 0.01x | 0.16x | 0.04x |
| `atan neg_1e6` | 15.08 ns | 273.48 ns | 277.48 ns | 2.93 us | 1.56 us | 17.42 us | 18.14x | 0.09x | 0.18x | 0.02x |
| `asinh neg_1e6` | 26.20 ns | 372.01 ns | 374.67 ns | 36.63 us | 1.60 us | 6.91 us | 14.20x | 0.01x | 0.23x | 0.05x |
| `acosh 9` | 12.33 ns | 159.61 ns | 161.34 ns | 41.98 us | 1.60 us | 9.61 us | 12.95x | 0.00x | 0.10x | 0.02x |
| `acosh 1_plus_1e-12` | 12.92 ns | 233.76 ns | 235.33 ns | 41.86 us | 8.35 us | 11.09 us | 18.10x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.28 ns | 158.90 ns | 160.83 ns | 37.12 us | 1.58 us | 9.52 us | 12.94x | 0.00x | 0.10x | 0.02x |
| `acosh e` | 12.28 ns | 232.52 ns | 2.27 us | 41.03 us | 1.61 us | 9.49 us | 18.94x | 0.01x | 0.14x | 0.02x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.73 ns | 18.56 ns | 24.96 ns | 15.55 ns | 0.94 ns | 39.76x | 0.75x | 1.20x | 19.92x |
| `one` | 0.48 ns | 24.71 ns | 23.61 ns | 40.80 ns | 30.31 ns | 30.27 ns | 51.72x | 0.61x | 0.82x | 0.82x |
| `e` | 0.48 ns | 91.82 ns | 88.92 ns | 53.97 ns | 1.04 us | 233.15 ns | 192.80x | 1.70x | 0.09x | 0.39x |
| `pi` | 0.47 ns | 49.35 ns | 50.86 ns | 45.27 ns | 48.67 ns | 233.23 ns | 103.98x | 1.09x | 1.01x | 0.21x |
| `tau` | 0.47 ns | 48.89 ns | 50.28 ns | 116.66 ns | 100.66 ns | 1.84 us | 103.76x | 0.42x | 0.49x | 0.03x |
| `add` | 5.16 ns | 209.18 ns | 319.61 ns | 51.64 ns | 43.86 ns | 1.28 us | 40.53x | 4.05x | 4.77x | 0.16x |
| `sub` | 5.24 ns | 255.01 ns | 498.32 ns | 55.33 ns | 45.63 ns | 2.47 us | 48.70x | 4.61x | 5.59x | 0.10x |
| `neg` | 5.14 ns | 38.90 ns | 40.02 ns | 23.30 ns | 21.48 ns | 1.05 us | 7.57x | 1.67x | 1.81x | 0.04x |
| `mul` | 5.42 ns | 137.57 ns | 411.87 ns | 58.62 ns | 45.90 ns | 1.49 us | 25.39x | 2.35x | 3.00x | 0.09x |
| `div` | 8.32 ns | 313.84 ns | 365.84 ns | 136.43 ns | 62.70 ns | 2.47 us | 37.74x | 2.30x | 5.01x | 0.13x |
| `reciprocal` | 8.66 ns | 75.62 ns | 74.59 ns | 161.27 ns | 58.52 ns | 1.50 us | 8.73x | 0.47x | 1.29x | 0.05x |
| `reciprocal checked` | 9.07 ns | 72.33 ns | 71.53 ns | 161.88 ns | 58.75 ns | 1.49 us | 7.98x | 0.45x | 1.23x | 0.05x |
| `reciprocal checked abort` | 24.36 ns | 81.70 ns | 78.67 ns | 161.78 ns | 58.87 ns | 1.49 us | 3.35x | 0.51x | 1.39x | 0.05x |
| `pow` | 20.13 ns | 6.69 us | 5.84 us | 54.57 us | 2.81 us | 2.32 us | 332.50x | 0.12x | 2.38x | 2.89x |
| `powi` | 5.80 ns | 378.88 ns | 2.66 us | 283.10 ns | 83.04 ns | 1.59 us | 65.29x | 1.34x | 4.56x | 0.24x |
| `exp` | 19.34 ns | 240.81 ns | 238.21 ns | 14.09 us | 913.77 ns | 1.86 us | 12.45x | 0.02x | 0.26x | 0.13x |
| `ln` | 10.62 ns | 1.44 us | 1.41 us | 29.50 us | 1.31 us | 1.78 us | 135.55x | 0.05x | 1.10x | 0.81x |
| `log10` | 13.28 ns | 1.63 us | 1.59 us | 34.85 us | 2.71 us | 6.58 us | 122.81x | 0.05x | 0.60x | 0.25x |
| `log10 abort` | 16.76 ns | 1.65 us | 1.58 us | 34.92 us | 2.73 us | 6.55 us | 98.40x | 0.05x | 0.60x | 0.25x |
| `sqrt` | 19.90 ns | 1.52 us | 1.66 us | 4.96 us | 94.75 ns | 1.44 us | 76.30x | 0.31x | 16.02x | 1.06x |
| `sin` | 14.87 ns | 122.29 ns | 123.04 ns | 14.00 us | 1.23 us | 2.20 us | 8.22x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.37 ns | 123.93 ns | 121.77 ns | 12.54 us | 621.26 ns | 1.73 us | 6.75x | 0.01x | 0.20x | 0.07x |
| `tan` | 24.58 ns | 170.63 ns | 172.70 ns | 29.53 us | 1.57 us | 6.47 us | 6.94x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.26 ns | 3.67 us | 3.62 us | 3.29 us | 1.11 us | 10.48 us | 200.96x | 1.12x | 3.31x | 0.35x |
| `cosh` | 18.18 ns | 3.60 us | 3.59 us | 7.95 us | 1.04 us | 9.39 us | 197.97x | 0.45x | 3.45x | 0.38x |
| `tanh` | 24.79 ns | 7.41 us | 7.62 us | 3.34 us | 1.18 us | 22.37 us | 298.76x | 2.22x | 6.28x | 0.33x |
| `asin` | 12.94 ns | 387.74 ns | 597.44 ns | 21.14 us | 2.38 us | 13.45 us | 29.96x | 0.02x | 0.16x | 0.03x |
| `asin abort` | 17.29 ns | 391.99 ns | 600.62 ns | 21.24 us | 2.37 us | 13.35 us | 22.67x | 0.02x | 0.17x | 0.03x |
| `acos` | 14.70 ns | 458.66 ns | 656.64 ns | 26.63 us | 2.48 us | 13.51 us | 31.20x | 0.02x | 0.18x | 0.03x |
| `acos abort` | 17.72 ns | 466.99 ns | 662.36 ns | 26.56 us | 2.48 us | 13.50 us | 26.35x | 0.02x | 0.19x | 0.03x |
| `atan` | 15.08 ns | 164.76 ns | 163.92 ns | 18.55 us | 2.24 us | 18.36 us | 10.93x | 0.01x | 0.07x | 0.01x |
| `atan abort` | 21.15 ns | 166.86 ns | 163.12 ns | 18.57 us | 2.26 us | 18.26 us | 7.89x | 0.01x | 0.07x | 0.01x |
| `asinh` | 33.37 ns | 265.14 ns | 272.94 ns | 38.92 us | 1.61 us | 7.34 us | 7.94x | 0.01x | 0.16x | 0.04x |
| `asinh abort` | 31.08 ns | 266.71 ns | 277.47 ns | 38.79 us | 1.61 us | 7.35 us | 8.58x | 0.01x | 0.17x | 0.04x |
| `acosh` | 12.48 ns | 202.13 ns | 202.35 ns | 40.43 us | 3.31 us | 10.15 us | 16.19x | 0.00x | 0.06x | 0.02x |
| `acosh abort` | 15.44 ns | 200.85 ns | 204.10 ns | 40.18 us | 3.30 us | 10.22 us | 13.01x | 0.00x | 0.06x | 0.02x |
| `atanh` | 13.74 ns | 231.43 ns | 253.08 ns | 34.46 us | 1.25 us | 14.59 us | 16.85x | 0.01x | 0.19x | 0.02x |
| `atanh abort` | 16.79 ns | 228.55 ns | 252.20 ns | 34.38 us | 1.25 us | 14.52 us | 13.61x | 0.01x | 0.18x | 0.02x |
| `zero status` | 1.20 ns | 0.99 ns | 1.00 ns | 1.05 ns | 6.73 ns | 8.02 ns | 0.83x | 0.95x | 0.15x | 0.12x |
| `zero status abort` | 1.41 ns | 1.12 ns | 1.13 ns | 1.02 ns | 6.70 ns | 8.01 ns | 0.79x | 1.09x | 0.17x | 0.14x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 36.97 ns | 36.92 ns | 51.16 ns | - | 1.88 ns | 39.76x | 0.72x | - | 19.67x |
| `one` | 5.29 ns | 42.47 ns | 42.17 ns | 64.22 ns | - | 32.10 ns | 8.03x | 0.66x | - | 1.32x |
| `i` | 0.93 ns | 43.14 ns | 43.86 ns | 65.83 ns | - | 29.15 ns | 46.23x | 0.66x | - | 1.48x |
| `free i` | 0.94 ns | 43.75 ns | 43.39 ns | 65.87 ns | - | 29.40 ns | 46.56x | 0.66x | - | 1.49x |
| `conjugate` | 2.11 ns | 85.93 ns | 85.17 ns | 41.31 ns | - | 1.06 us | 40.70x | 2.08x | - | 0.08x |
| `norm squared` | 5.73 ns | 163.18 ns | 551.22 ns | 153.08 ns | - | 4.20 us | 28.46x | 1.07x | - | 0.04x |
| `reciprocal` | 18.32 ns | 1.75 us | 2.86 us | 460.36 ns | - | 10.39 us | 95.45x | 3.80x | - | 0.17x |
| `reciprocal checked` | 15.07 ns | 1.77 us | 2.87 us | 460.86 ns | - | 10.45 us | 117.69x | 3.85x | - | 0.17x |
| `powi` | 18.45 ns | 2.15 us | 14.38 us | 1.44 us | - | 42.48 us | 116.53x | 1.49x | - | 0.05x |
| `powi checked` | 18.61 ns | 2.15 us | 14.43 us | 1.43 us | - | 42.30 us | 115.37x | 1.50x | - | 0.05x |
| `div checked` | 18.21 ns | 2.75 us | 6.40 us | 793.06 ns | - | 21.15 us | 151.08x | 3.47x | - | 0.13x |
| `div real checked` | 9.31 ns | 669.14 ns | 630.13 ns | 267.33 ns | - | 5.03 us | 71.89x | 2.50x | - | 0.13x |
| `from scalar` | 1.40 ns | 72.06 ns | 70.15 ns | 44.73 ns | - | 10.17 ns | 51.33x | 1.61x | - | 7.09x |
| `add` | 6.05 ns | 486.19 ns | 850.94 ns | 103.63 ns | - | 2.50 us | 80.32x | 4.69x | - | 0.19x |
| `sub` | 5.79 ns | 533.47 ns | 1.14 us | 113.49 ns | - | 4.63 us | 92.17x | 4.70x | - | 0.12x |
| `neg` | 2.57 ns | 87.82 ns | 85.87 ns | 41.87 ns | - | 2.08 us | 34.17x | 2.10x | - | 0.04x |
| `mul` | 7.58 ns | 939.78 ns | 3.62 us | 307.30 ns | - | 9.73 us | 124.04x | 3.06x | - | 0.10x |
| `div` | 18.30 ns | 2.75 us | 6.41 us | 784.80 ns | - | 20.98 us | 150.32x | 3.51x | - | 0.13x |
| `div real` | 9.99 ns | 661.74 ns | 623.37 ns | 267.60 ns | - | 5.04 us | 66.24x | 2.47x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.77 ns | 211.91 ns | 2.38 us | 307.16 ns | 251.00 ns | 7.03 us | 24.15x | 0.69x | 0.84x | 0.03x |
| `vec3 magnitude` | 11.24 ns | 3.95 us | 5.33 us | 5.42 us | 350.71 ns | 8.51 us | 351.08x | 0.73x | 11.25x | 0.46x |
| `vec3 normalize` | 25.73 ns | 8.43 us | 11.66 us | 5.92 us | 605.53 ns | 16.37 us | 327.66x | 1.42x | 13.92x | 0.52x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.03 ns | 318.50 ns | 1.46 us | 72.47 ns | 57.43 ns | 729.37 ns | 105.25x | 4.40x | 5.55x | 0.44x |
| `vec3 zero` | 1.40 ns | 136.41 ns | 136.59 ns | 59.73 ns | 31.55 ns | 2.82 ns | 97.40x | 2.28x | 4.32x | 48.41x |
| `vec3 dot abort` | 8.41 ns | 212.74 ns | 886.79 ns | 252.54 ns | 205.89 ns | 7.00 us | 25.30x | 0.84x | 1.03x | 0.03x |
| `vec3 magnitude abort` | 17.33 ns | 3.95 us | 3.16 us | 5.48 us | 329.42 ns | 8.61 us | 227.98x | 0.72x | 11.99x | 0.46x |
| `vec3 normalize checked` | 26.16 ns | 8.31 us | 7.27 us | 5.91 us | 548.82 ns | 16.74 us | 317.77x | 1.41x | 15.15x | 0.50x |
| `vec3 normalize checked abort` | 29.95 ns | 8.23 us | 7.26 us | 5.91 us | 546.34 ns | 16.82 us | 274.64x | 1.39x | 15.06x | 0.49x |
| `vec3 div scalar checked` | 10.33 ns | 1.32 us | 1.55 us | 394.00 ns | - | - | 127.77x | 3.35x | - | - |
| `vec3 div scalar checked abort` | 17.74 ns | 1.30 us | 1.54 us | 391.67 ns | - | - | 73.49x | 3.33x | - | - |
| `vec3 add` | 6.55 ns | 947.17 ns | 1.73 us | 149.08 ns | 126.05 ns | 3.83 us | 144.53x | 6.35x | 7.51x | 0.25x |
| `vec3 add scalar` | 6.31 ns | 761.85 ns | 1.47 us | 151.47 ns | 131.22 ns | 3.72 us | 120.82x | 5.03x | 5.81x | 0.20x |
| `vec3 sub` | 6.98 ns | 1.04 us | 2.48 us | 162.82 ns | 135.72 ns | 7.18 us | 149.00x | 6.38x | 7.66x | 0.14x |
| `vec3 sub scalar` | 6.38 ns | 892.17 ns | 1.97 us | 144.98 ns | 123.96 ns | 6.87 us | 139.91x | 6.15x | 7.20x | 0.13x |
| `vec3 neg` | 3.75 ns | 150.37 ns | 145.12 ns | 56.15 ns | 46.52 ns | 3.08 us | 40.09x | 2.68x | 3.23x | 0.05x |
| `vec3 mul scalar` | 6.84 ns | 516.69 ns | 2.00 us | 161.85 ns | 120.67 ns | 4.34 us | 75.54x | 3.19x | 4.28x | 0.12x |
| `vec3 div scalar` | 17.25 ns | 1.27 us | 1.51 us | 401.82 ns | 172.26 ns | 7.41 us | 73.68x | 3.16x | 7.38x | 0.17x |
| `vec4 dot` | 9.76 ns | 252.10 ns | 634.63 ns | 426.41 ns | 319.68 ns | 9.43 us | 25.83x | 0.59x | 0.79x | 0.03x |
| `vec4 magnitude` | 16.19 ns | 3.74 us | 2.58 us | 5.48 us | 405.76 ns | 11.13 us | 230.82x | 0.68x | 9.21x | 0.34x |
| `vec4 normalize` | 37.50 ns | 8.51 us | 6.39 us | 6.08 us | 721.24 ns | 21.49 us | 227.03x | 1.40x | 11.80x | 0.40x |
| `vec4 add` | 7.81 ns | 1.16 us | 1.79 us | 204.47 ns | 172.46 ns | 5.12 us | 147.88x | 5.65x | 6.70x | 0.23x |
| `vec4 add scalar` | 6.77 ns | 978.06 ns | 1.68 us | 211.80 ns | 175.31 ns | 4.98 us | 144.48x | 4.62x | 5.58x | 0.20x |
| `vec4 sub` | 5.17 ns | 1.25 us | 2.23 us | 205.44 ns | 176.63 ns | 9.37 us | 241.82x | 6.09x | 7.08x | 0.13x |
| `vec4 sub scalar` | 4.41 ns | 1.07 us | 2.04 us | 205.35 ns | 168.23 ns | 9.17 us | 243.25x | 5.23x | 6.38x | 0.12x |
| `vec4 neg` | 4.92 ns | 187.73 ns | 188.34 ns | 77.98 ns | 65.48 ns | 3.99 us | 38.19x | 2.41x | 2.87x | 0.05x |
| `vec4 mul scalar` | 7.24 ns | 646.71 ns | 2.04 us | 225.24 ns | 166.32 ns | 5.54 us | 89.32x | 2.87x | 3.89x | 0.12x |
| `vec4 div scalar` | 14.17 ns | 1.77 us | 1.52 us | 523.95 ns | 227.46 ns | 9.72 us | 124.54x | 3.37x | 7.76x | 0.18x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.16 ns | 715.28 ns | 2.23 us | 957.12 ns | 847.82 ns | 21.80 us | 37.33x | 0.75x | 0.84x | 0.03x |
| `mat3 inverse` | 88.50 ns | 14.58 us | 8.57 us | 3.16 us | 2.46 us | 79.82 us | 164.73x | 4.61x | 5.92x | 0.18x |
| `mat3 mul mat3` | 74.69 ns | 3.29 us | 7.81 us | 2.75 us | 2.30 us | 59.60 us | 44.06x | 1.20x | 1.43x | 0.06x |
| `mat3 transform vec3` | 28.22 ns | 1.60 us | 4.95 us | 1.00 us | 880.51 ns | 19.60 us | 56.83x | 1.60x | 1.82x | 0.08x |
| `mat4 determinant` | 42.99 ns | 2.21 us | 1.54 us | 4.55 us | 4.07 us | 92.40 us | 51.39x | 0.49x | 0.54x | 0.02x |
| `mat4 inverse` | 182.74 ns | 24.80 us | 8.56 us | 10.99 us | 8.97 us | 331.83 us | 135.69x | 2.26x | 2.76x | 0.07x |
| `mat4 mul mat4` | 134.49 ns | 6.02 us | 4.64 us | 6.38 us | 5.25 us | 139.87 us | 44.79x | 0.94x | 1.15x | 0.04x |
| `mat4 transform vec4` | 44.89 ns | 2.44 us | 2.51 us | 1.89 us | 1.63 us | 34.52 us | 54.38x | 1.29x | 1.50x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 38.50 ns | 1.31 us | 4.29 us | 205.89 ns | 228.55 ns | 2.13 us | 34.12x | 6.38x | 5.75x | 0.62x |
| `mat3 zero` | 16.07 ns | 543.82 ns | 537.92 ns | 261.96 ns | 205.39 ns | 11.24 ns | 33.83x | 2.08x | 2.65x | 48.40x |
| `mat3 identity` | 10.12 ns | 628.81 ns | 628.13 ns | 327.35 ns | 237.80 ns | 152.32 ns | 62.11x | 1.92x | 2.64x | 4.13x |
| `mat3 transpose` | 9.45 ns | 673.01 ns | 661.26 ns | 229.08 ns | 208.08 ns | 108.49 ns | 71.21x | 2.94x | 3.23x | 6.20x |
| `mat3 reciprocal` | 88.50 ns | 14.62 us | 24.73 us | 2.90 us | 2.27 us | 82.04 us | 165.21x | 5.04x | 6.43x | 0.18x |
| `mat3 reciprocal checked` | 88.05 ns | 14.67 us | 24.87 us | 2.88 us | 2.26 us | 80.84 us | 166.65x | 5.10x | 6.48x | 0.18x |
| `mat3 inverse checked` | 87.91 ns | 14.77 us | 24.89 us | 2.90 us | 2.27 us | 80.68 us | 167.96x | 5.09x | 6.50x | 0.18x |
| `mat3 inverse checked abort` | 96.04 ns | 14.74 us | 24.95 us | 2.89 us | 2.27 us | 80.38 us | 153.50x | 5.10x | 6.48x | 0.18x |
| `mat3 powi` | 108.83 ns | 4.36 us | 37.97 us | 6.80 us | 6.23 us | 146.83 us | 40.10x | 0.64x | 0.70x | 0.03x |
| `mat3 powi checked` | 107.95 ns | 4.38 us | 38.15 us | 6.82 us | 6.22 us | 148.74 us | 40.59x | 0.64x | 0.70x | 0.03x |
| `mat3 powi checked abort` | 122.05 ns | 4.37 us | 38.16 us | 6.78 us | 6.20 us | 147.80 us | 35.82x | 0.64x | 0.70x | 0.03x |
| `mat3 div scalar checked` | 24.97 ns | 4.25 us | 5.56 us | 1.44 us | 802.81 ns | 21.55 us | 170.39x | 2.96x | 5.30x | 0.20x |
| `mat3 div scalar checked abort` | 29.70 ns | 4.29 us | 5.55 us | 1.45 us | 813.68 ns | 21.59 us | 144.47x | 2.96x | 5.27x | 0.20x |
| `mat3 div matrix checked` | 155.72 ns | 18.10 us | 43.80 us | 5.40 us | 4.36 us | 156.81 us | 116.21x | 3.35x | 4.15x | 0.12x |
| `mat3 div matrix checked abort` | 167.02 ns | 18.18 us | 43.78 us | 5.39 us | 4.37 us | 157.29 us | 108.84x | 3.37x | 4.16x | 0.12x |
| `mat3 add` | 14.46 ns | 3.34 us | 5.17 us | 508.87 ns | 492.04 ns | 11.50 us | 230.85x | 6.56x | 6.78x | 0.29x |
| `mat3 add scalar` | 11.98 ns | 2.39 us | 4.56 us | 777.09 ns | 696.65 ns | 11.73 us | 199.12x | 3.07x | 3.42x | 0.20x |
| `mat3 sub` | 14.02 ns | 3.50 us | 7.12 us | 558.69 ns | 514.03 ns | 20.81 us | 249.79x | 6.27x | 6.81x | 0.17x |
| `mat3 sub scalar` | 11.21 ns | 2.68 us | 6.43 us | 787.62 ns | 694.83 ns | 21.11 us | 239.26x | 3.41x | 3.86x | 0.13x |
| `mat3 neg` | 10.79 ns | 620.51 ns | 632.15 ns | 476.24 ns | 460.66 ns | 8.38 us | 57.50x | 1.30x | 1.35x | 0.07x |
| `mat3 mul scalar` | 13.65 ns | 1.71 us | 5.96 us | 775.57 ns | 675.20 ns | 11.94 us | 125.50x | 2.21x | 2.54x | 0.14x |
| `mat3 div scalar` | 24.97 ns | 4.29 us | 5.64 us | 1.46 us | 800.83 ns | 21.83 us | 171.93x | 2.94x | 5.36x | 0.20x |
| `mat3 div matrix` | 153.66 ns | 18.16 us | 43.82 us | 5.41 us | 4.36 us | 157.46 us | 118.16x | 3.36x | 4.16x | 0.12x |
| `mat3 bitxor` | 107.85 ns | 4.33 us | 38.27 us | 6.73 us | 6.22 us | 147.32 us | 40.19x | 0.64x | 0.70x | 0.03x |
| `mat4 zero` | 13.30 ns | 784.54 ns | 784.76 ns | 550.21 ns | 350.14 ns | 14.17 ns | 58.99x | 1.43x | 2.24x | 55.37x |
| `mat4 identity` | 10.48 ns | 968.90 ns | 968.85 ns | 607.08 ns | 415.18 ns | 232.06 ns | 92.45x | 1.60x | 2.33x | 4.18x |
| `mat4 transpose` | 10.10 ns | 1.16 us | 1.14 us | 479.58 ns | 367.71 ns | 175.83 ns | 114.47x | 2.41x | 3.14x | 6.58x |
| `mat4 reciprocal` | 160.22 ns | 24.67 us | 42.70 us | 10.54 us | 8.71 us | 333.54 us | 153.98x | 2.34x | 2.83x | 0.07x |
| `mat4 reciprocal checked` | 169.86 ns | 24.59 us | 42.80 us | 10.53 us | 8.70 us | 332.83 us | 144.76x | 2.33x | 2.82x | 0.07x |
| `mat4 powi` | 238.08 ns | 8.84 us | 51.99 us | 15.49 us | 13.79 us | 342.95 us | 37.13x | 0.57x | 0.64x | 0.03x |
| `mat4 powi checked` | 238.02 ns | 8.80 us | 51.91 us | 15.50 us | 13.76 us | 340.12 us | 36.98x | 0.57x | 0.64x | 0.03x |
| `mat4 add` | 52.15 ns | 5.32 us | 6.05 us | 930.23 ns | 851.04 ns | 19.35 us | 101.93x | 5.71x | 6.25x | 0.27x |
| `mat4 add scalar` | 20.67 ns | 4.06 us | 6.68 us | 1.37 us | 1.23 us | 20.10 us | 196.59x | 2.96x | 3.29x | 0.20x |
| `mat4 sub` | 38.67 ns | 5.57 us | 7.21 us | 1.01 us | 892.68 ns | 35.12 us | 143.93x | 5.51x | 6.23x | 0.16x |
| `mat4 sub scalar` | 14.89 ns | 4.32 us | 7.56 us | 1.40 us | 1.15 us | 36.21 us | 290.37x | 3.09x | 3.74x | 0.12x |
| `mat4 neg` | 13.67 ns | 1.10 us | 1.08 us | 878.81 ns | 761.46 ns | 13.95 us | 80.21x | 1.25x | 1.44x | 0.08x |
| `mat4 mul scalar` | 23.67 ns | 2.96 us | 7.36 us | 1.44 us | 1.12 us | 19.70 us | 124.93x | 2.05x | 2.63x | 0.15x |
| `mat4 div scalar` | 32.52 ns | 7.50 us | 7.59 us | 2.67 us | 1.40 us | 36.84 us | 230.56x | 2.81x | 5.36x | 0.20x |
| `mat4 div matrix` | 208.38 ns | 28.39 us | 64.31 us | 16.73 us | 13.95 us | 518.58 us | 136.26x | 1.70x | 2.04x | 0.05x |
| `mat4 bitxor` | 238.98 ns | 8.83 us | 51.64 us | 15.49 us | 13.96 us | 341.98 us | 36.94x | 0.57x | 0.63x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.64 ns | 173.85 ns | 290.33 ns | - | - | 1.26 us | 47.74x | - | - | 0.14x |
| `scalar add ref_owned` | 12.50 ns | 179.87 ns | 283.13 ns | - | - | 1.26 us | 14.39x | - | - | 0.14x |
| `scalar add refs` | 5.33 ns | 156.56 ns | 263.17 ns | - | - | 1.26 us | 29.35x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 8.94 ns | 191.28 ns | 296.42 ns | - | - | - | 21.40x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.41 ns | 183.23 ns | 291.77 ns | - | - | - | 33.87x | - | - | - |
| `scalar sub owned_ref` | 3.97 ns | 218.27 ns | 470.89 ns | - | - | 2.36 us | 54.94x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.50 ns | 223.44 ns | 467.39 ns | - | - | 2.37 us | 17.87x | - | - | 0.09x |
| `scalar sub refs` | 5.57 ns | 202.48 ns | 445.16 ns | - | - | 2.36 us | 36.32x | - | - | 0.09x |
| `scalar sub owned_ref_with_clone` | 9.00 ns | 229.92 ns | 478.34 ns | - | - | - | 25.55x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.65 ns | 228.94 ns | 471.63 ns | - | - | - | 40.51x | - | - | - |
| `scalar mul owned_ref` | 4.27 ns | 96.63 ns | 377.24 ns | - | - | 1.50 us | 22.65x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.54 ns | 100.70 ns | 376.65 ns | - | - | 1.50 us | 7.44x | - | - | 0.07x |
| `scalar mul refs` | 6.06 ns | 83.03 ns | 355.52 ns | - | - | 1.49 us | 13.70x | - | - | 0.06x |
| `scalar mul owned_ref_with_clone` | 9.83 ns | 108.97 ns | 383.00 ns | - | - | - | 11.08x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.23 ns | 106.47 ns | 386.41 ns | - | - | - | 17.10x | - | - | - |
| `scalar div owned_ref` | 5.81 ns | 262.58 ns | 314.39 ns | - | - | 2.48 us | 45.17x | - | - | 0.11x |
| `scalar div ref_owned` | 17.08 ns | 267.98 ns | 312.46 ns | - | - | 2.45 us | 15.69x | - | - | 0.11x |
| `scalar div refs` | 6.89 ns | 247.47 ns | 300.20 ns | - | - | 2.48 us | 35.92x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.29 ns | 278.91 ns | 326.06 ns | - | - | - | 20.99x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.63 ns | 278.46 ns | 328.63 ns | - | - | - | 17.82x | - | - | - |
| `vec3 add refs` | 6.05 ns | 621.82 ns | 1.40 us | - | - | 3.82 us | 102.80x | - | - | 0.16x |
| `vec3 sub refs` | 6.12 ns | 736.10 ns | 2.20 us | - | - | 7.10 us | 120.35x | - | - | 0.10x |
| `vec3 neg ref` | 3.29 ns | 162.52 ns | 158.27 ns | - | - | 3.06 us | 49.46x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.45 ns | 764.46 ns | 1.46 us | - | - | 3.70 us | 118.55x | - | - | 0.21x |
| `vec3 sub_scalar_ref` | 6.52 ns | 875.97 ns | 1.96 us | - | - | 6.80 us | 134.29x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 6.88 ns | 512.19 ns | 2.00 us | - | - | 4.29 us | 74.48x | - | - | 0.12x |
| `vec3 div_scalar_ref` | 8.13 ns | 1.26 us | 1.56 us | - | - | 7.43 us | 155.00x | - | - | 0.17x |
| `vec4 add refs` | 6.66 ns | 789.71 ns | 1.39 us | - | - | 5.13 us | 118.56x | - | - | 0.15x |
| `vec4 sub refs` | 3.07 ns | 878.55 ns | 1.82 us | - | - | 9.40 us | 286.17x | - | - | 0.09x |
| `vec4 neg ref` | 4.23 ns | 211.84 ns | 201.39 ns | - | - | 3.93 us | 50.14x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 6.91 ns | 1.00 us | 1.72 us | - | - | 4.95 us | 144.80x | - | - | 0.20x |
| `vec4 sub_scalar_ref` | 4.21 ns | 1.11 us | 2.07 us | - | - | 9.18 us | 263.87x | - | - | 0.12x |
| `vec4 mul_scalar_ref` | 7.50 ns | 656.07 ns | 2.09 us | - | - | 5.51 us | 87.47x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.65 ns | 1.68 us | 1.44 us | - | - | 9.67 us | 144.49x | - | - | 0.17x |
| `mat3 add refs` | 10.99 ns | 1.67 us | 3.75 us | - | - | 11.36 us | 152.22x | - | - | 0.15x |
| `mat3 sub refs` | 10.48 ns | 1.98 us | 5.69 us | - | - | 20.74 us | 188.68x | - | - | 0.10x |
| `mat3 mul refs` | 53.82 ns | 2.32 us | 14.86 us | - | - | 59.48 us | 43.09x | - | - | 0.04x |
| `mat3 div refs` | 138.44 ns | 17.01 us | 42.49 us | - | - | 154.64 us | 122.89x | - | - | 0.11x |
| `mat3 neg ref` | 9.64 ns | 520.20 ns | 481.58 ns | - | - | 8.37 us | 53.95x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 44.37 ns | 2.44 us | 4.61 us | - | - | 11.69 us | 54.88x | - | - | 0.21x |
| `mat3 sub_scalar_ref` | 44.41 ns | 2.73 us | 6.44 us | - | - | 21.01 us | 61.46x | - | - | 0.13x |
| `mat3 mul_scalar_ref` | 46.51 ns | 1.78 us | 5.70 us | - | - | 11.90 us | 38.19x | - | - | 0.15x |
| `mat3 div_scalar_ref` | 21.60 ns | 4.20 us | 5.59 us | - | - | 21.50 us | 194.30x | - | - | 0.20x |
| `mat4 add refs` | 17.56 ns | 2.63 us | 3.46 us | - | - | 19.18 us | 149.98x | - | - | 0.14x |
| `mat4 sub refs` | 16.41 ns | 2.98 us | 4.62 us | - | - | 35.59 us | 181.48x | - | - | 0.08x |
| `mat4 mul refs` | 121.12 ns | 4.18 us | 19.14 us | - | - | 138.59 us | 34.50x | - | - | 0.03x |
| `mat4 div refs` | 197.00 ns | 26.50 us | 62.88 us | - | - | 519.43 us | 134.53x | - | - | 0.05x |
| `mat4 neg ref` | 12.52 ns | 830.64 ns | 830.82 ns | - | - | 13.84 us | 66.35x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 52.46 ns | 4.04 us | 6.71 us | - | - | 19.98 us | 77.06x | - | - | 0.20x |
| `mat4 sub_scalar_ref` | 39.19 ns | 4.30 us | 7.62 us | - | - | 36.33 us | 109.60x | - | - | 0.12x |
| `mat4 mul_scalar_ref` | 56.03 ns | 2.90 us | 7.31 us | - | - | 20.03 us | 51.78x | - | - | 0.14x |
| `mat4 div_scalar_ref` | 28.54 ns | 7.46 us | 7.51 us | - | - | 36.70 us | 261.54x | - | - | 0.20x |
| `mat3 transform_vec refs` | 27.10 ns | 920.74 ns | 4.93 us | - | - | 19.44 us | 33.98x | - | - | 0.05x |
| `mat4 transform_vec refs` | 43.34 ns | 1.51 us | 5.76 us | - | - | 33.97 us | 34.73x | - | - | 0.04x |
| `complex add refs` | 7.79 ns | 321.93 ns | 696.10 ns | - | - | 2.46 us | 41.34x | - | - | 0.13x |
| `complex sub refs` | 8.03 ns | 395.97 ns | 944.93 ns | - | - | 4.66 us | 49.29x | - | - | 0.09x |
| `complex mul refs` | 7.95 ns | 776.39 ns | 3.47 us | - | - | 9.72 us | 97.69x | - | - | 0.08x |
| `complex div refs` | 17.33 ns | 2.60 us | 6.25 us | - | - | 21.05 us | 150.15x | - | - | 0.12x |
| `complex neg ref` | 2.35 ns | 72.29 ns | 69.95 ns | - | - | 2.10 us | 30.73x | - | - | 0.03x |
| `complex div_real_ref` | 7.22 ns | 612.39 ns | 598.69 ns | - | - | 5.05 us | 84.86x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.10 us |
| `astro sin 160` | 13.41 us |
| `astro sin 192` | 13.40 us |
| `astro sin 256` | 15.99 us |
