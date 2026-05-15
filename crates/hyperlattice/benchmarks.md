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
| `sin 0.1` | 10.90 ns | 147.55 ns | 148.93 ns | 10.99 us | 760.73 ns | 1.82 us | 13.54x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.68 ns | 146.37 ns | 146.88 ns | 10.44 us | 491.01 ns | 1.67 us | 12.53x | 0.01x | 0.30x | 0.09x |
| `sin 1.23456789` | 11.67 ns | 209.86 ns | 196.82 ns | 12.53 us | 800.84 ns | 1.79 us | 17.98x | 0.02x | 0.26x | 0.12x |
| `cos 1.23456789` | 12.00 ns | 211.68 ns | 201.12 ns | 10.64 us | 588.45 ns | 1.64 us | 17.65x | 0.02x | 0.36x | 0.13x |
| `sin 1e6` | 12.57 ns | 90.13 ns | 90.51 ns | 15.96 us | 1.09 us | 2.00 us | 7.17x | 0.01x | 0.08x | 0.05x |
| `cos 1e6` | 12.35 ns | 91.50 ns | 91.48 ns | 13.60 us | 822.45 ns | 1.81 us | 7.41x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 66.17 ns | 92.85 ns | 92.65 ns | 18.44 us | 2.84 us | 3.53 us | 1.40x | 0.01x | 0.03x | 0.03x |
| `cos 1e30` | 68.52 ns | 94.65 ns | 92.77 ns | 15.41 us | 963.82 ns | 3.07 us | 1.38x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.69 ns | 147.06 ns | 372.68 ns | 12.13 us | 735.75 ns | 1.86 us | 12.58x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.63 ns | 146.96 ns | 708.26 ns | 10.82 us | 530.67 ns | 1.70 us | 12.63x | 0.01x | 0.28x | 0.09x |
| `sin 1000pi_eps` | 11.81 ns | 90.01 ns | 731.26 ns | 16.00 us | 2.24 us | 2.81 us | 7.62x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.43 ns | 91.18 ns | 730.31 ns | 13.65 us | 581.48 ns | 1.67 us | 7.33x | 0.01x | 0.16x | 0.05x |
| `asin 0.5` | 10.95 ns | 137.81 ns | 137.48 ns | 49.32 us | 2.95 us | 13.27 us | 12.59x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.14 ns | 136.22 ns | 138.00 ns | 59.30 us | 2.92 us | 13.09 us | 12.23x | 0.00x | 0.05x | 0.01x |
| `atanh 0.5` | 14.49 ns | 149.83 ns | 149.32 ns | 34.07 us | 1.63 us | 12.84 us | 10.34x | 0.00x | 0.09x | 0.01x |
| `asin neg_0.999999` | 13.92 ns | 537.37 ns | 509.52 ns | 13.64 us | 2.54 us | 12.86 us | 38.61x | 0.04x | 0.21x | 0.04x |
| `acos neg_0.999999` | 15.19 ns | 366.47 ns | 335.66 ns | 18.04 us | 2.65 us | 12.95 us | 24.12x | 0.02x | 0.14x | 0.03x |
| `atanh neg_0.999999` | 14.46 ns | 271.22 ns | 246.85 ns | 36.12 us | 1.57 us | 12.56 us | 18.76x | 0.01x | 0.17x | 0.02x |
| `asin 0.999999` | 13.86 ns | 470.76 ns | 688.31 ns | 13.55 us | 2.52 us | 12.83 us | 33.96x | 0.03x | 0.19x | 0.04x |
| `acos 0.999999` | 14.16 ns | 349.07 ns | 792.55 ns | 18.07 us | 2.72 us | 12.92 us | 24.66x | 0.02x | 0.13x | 0.03x |
| `atanh 0.999999` | 14.52 ns | 289.80 ns | 499.30 ns | 31.40 us | 1.57 us | 12.50 us | 19.95x | 0.01x | 0.18x | 0.02x |
| `asin 1e-12` | 9.37 ns | 271.18 ns | 465.17 ns | 7.93 us | 1.41 us | 15.05 us | 28.95x | 0.03x | 0.19x | 0.02x |
| `acos 1e-12` | 9.86 ns | 549.58 ns | 923.82 ns | 9.59 us | 1.41 us | 15.12 us | 55.73x | 0.06x | 0.39x | 0.04x |
| `atanh 1e-12` | 9.60 ns | 309.20 ns | 492.47 ns | 35.99 us | 168.98 ns | 19.89 us | 32.19x | 0.01x | 1.83x | 0.02x |
| `atan 0.5` | 14.87 ns | 162.03 ns | 162.31 ns | 35.40 us | 2.74 us | 17.55 us | 10.89x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.63 ns | 210.47 ns | 209.93 ns | 39.06 us | 1.61 us | 7.40 us | 7.90x | 0.01x | 0.13x | 0.03x |
| `atan neg_1e-12` | 14.27 ns | 278.69 ns | 269.51 ns | 1.60 us | 1.11 us | 15.19 us | 19.53x | 0.17x | 0.25x | 0.02x |
| `asinh neg_1e-12` | 15.58 ns | 469.04 ns | 369.68 ns | 41.86 us | 8.60 us | 11.74 us | 30.11x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 15.11 ns | 163.89 ns | 163.93 ns | 2.81 us | 1.41 us | 18.00 us | 10.85x | 0.06x | 0.12x | 0.01x |
| `asinh 1e6` | 26.46 ns | 207.01 ns | 206.76 ns | 36.51 us | 1.65 us | 7.09 us | 7.82x | 0.01x | 0.13x | 0.03x |
| `atan neg_1e6` | 15.26 ns | 267.17 ns | 266.46 ns | 2.86 us | 1.42 us | 18.06 us | 17.51x | 0.09x | 0.19x | 0.01x |
| `asinh neg_1e6` | 26.84 ns | 247.46 ns | 247.57 ns | 36.79 us | 1.65 us | 6.96 us | 9.22x | 0.01x | 0.15x | 0.04x |
| `acosh 9` | 12.32 ns | 142.52 ns | 142.19 ns | 42.66 us | 1.59 us | 9.78 us | 11.57x | 0.00x | 0.09x | 0.01x |
| `acosh 1_plus_1e-12` | 11.41 ns | 280.11 ns | 263.04 ns | 42.03 us | 8.29 us | 11.27 us | 24.55x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.33 ns | 142.78 ns | 142.10 ns | 37.76 us | 1.59 us | 9.75 us | 11.58x | 0.00x | 0.09x | 0.01x |
| `acosh e` | 12.34 ns | 182.52 ns | 2.19 us | 41.61 us | 1.61 us | 9.63 us | 14.80x | 0.00x | 0.11x | 0.02x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 18.59 ns | 18.39 ns | 24.97 ns | 15.67 ns | 0.94 ns | 38.80x | 0.74x | 1.19x | 19.72x |
| `one` | 0.48 ns | 23.62 ns | 23.01 ns | 41.21 ns | 30.65 ns | 32.63 ns | 49.30x | 0.57x | 0.77x | 0.72x |
| `e` | 0.47 ns | 89.38 ns | 90.77 ns | 54.03 ns | 1.05 us | 224.28 ns | 189.65x | 1.65x | 0.08x | 0.40x |
| `pi` | 0.48 ns | 50.44 ns | 50.11 ns | 51.94 ns | 48.64 ns | 221.12 ns | 104.87x | 0.97x | 1.04x | 0.23x |
| `tau` | 0.48 ns | 50.37 ns | 49.88 ns | 116.16 ns | 101.82 ns | 1.85 us | 104.84x | 0.43x | 0.49x | 0.03x |
| `add` | 5.16 ns | 239.19 ns | 320.44 ns | 53.20 ns | 42.38 ns | 1.26 us | 46.40x | 4.50x | 5.64x | 0.19x |
| `sub` | 5.20 ns | 264.62 ns | 495.72 ns | 55.82 ns | 45.27 ns | 2.41 us | 50.85x | 4.74x | 5.85x | 0.11x |
| `neg` | 5.09 ns | 39.13 ns | 40.86 ns | 23.73 ns | 20.13 ns | 1.05 us | 7.68x | 1.65x | 1.94x | 0.04x |
| `mul` | 5.38 ns | 146.02 ns | 405.54 ns | 58.17 ns | 44.63 ns | 1.50 us | 27.13x | 2.51x | 3.27x | 0.10x |
| `div` | 9.07 ns | 310.94 ns | 363.46 ns | 138.12 ns | 62.39 ns | 2.55 us | 34.29x | 2.25x | 4.98x | 0.12x |
| `reciprocal` | 8.22 ns | 73.65 ns | 72.69 ns | 158.10 ns | 59.15 ns | 1.51 us | 8.96x | 0.47x | 1.25x | 0.05x |
| `reciprocal checked` | 8.63 ns | 71.62 ns | 69.58 ns | 158.09 ns | 58.86 ns | 1.51 us | 8.30x | 0.45x | 1.22x | 0.05x |
| `reciprocal checked abort` | 22.12 ns | 78.89 ns | 80.93 ns | 157.88 ns | 58.74 ns | 1.51 us | 3.57x | 0.50x | 1.34x | 0.05x |
| `pow` | 20.36 ns | 6.61 us | 5.84 us | 54.32 us | 2.83 us | 2.33 us | 324.81x | 0.12x | 2.33x | 2.84x |
| `powi` | 5.91 ns | 377.34 ns | 2.58 us | 286.94 ns | 84.77 ns | 1.54 us | 63.81x | 1.32x | 4.45x | 0.24x |
| `exp` | 19.59 ns | 238.29 ns | 230.28 ns | 14.14 us | 918.05 ns | 1.86 us | 12.16x | 0.02x | 0.26x | 0.13x |
| `ln` | 10.73 ns | 1.40 us | 1.37 us | 30.17 us | 1.32 us | 1.81 us | 130.81x | 0.05x | 1.07x | 0.77x |
| `log10` | 13.43 ns | 1.60 us | 1.58 us | 35.78 us | 2.75 us | 6.65 us | 119.50x | 0.04x | 0.58x | 0.24x |
| `log10 abort` | 18.77 ns | 1.60 us | 1.56 us | 35.98 us | 2.76 us | 6.63 us | 85.26x | 0.04x | 0.58x | 0.24x |
| `sqrt` | 8.17 ns | 1.50 us | 1.61 us | 5.15 us | 96.92 ns | 1.45 us | 184.12x | 0.29x | 15.52x | 1.04x |
| `sin` | 15.06 ns | 120.91 ns | 119.27 ns | 14.15 us | 1.24 us | 2.21 us | 8.03x | 0.01x | 0.10x | 0.05x |
| `cos` | 18.28 ns | 122.25 ns | 119.17 ns | 12.26 us | 629.09 ns | 1.75 us | 6.69x | 0.01x | 0.19x | 0.07x |
| `tan` | 24.53 ns | 169.99 ns | 167.59 ns | 29.21 us | 1.59 us | 6.53 us | 6.93x | 0.01x | 0.11x | 0.03x |
| `sinh` | 18.14 ns | 3.65 us | 3.64 us | 3.30 us | 1.11 us | 10.62 us | 201.01x | 1.10x | 3.29x | 0.34x |
| `cosh` | 18.42 ns | 3.56 us | 3.54 us | 7.97 us | 1.05 us | 9.41 us | 193.19x | 0.45x | 3.40x | 0.38x |
| `tanh` | 20.36 ns | 5.99 us | 5.92 us | 3.34 us | 1.19 us | 22.67 us | 294.11x | 1.79x | 5.02x | 0.26x |
| `asin` | 13.18 ns | 365.27 ns | 567.60 ns | 21.44 us | 2.41 us | 13.80 us | 27.71x | 0.02x | 0.15x | 0.03x |
| `asin abort` | 17.23 ns | 366.31 ns | 571.55 ns | 21.48 us | 2.42 us | 13.80 us | 21.26x | 0.02x | 0.15x | 0.03x |
| `acos` | 13.95 ns | 356.16 ns | 740.68 ns | 26.65 us | 2.54 us | 13.79 us | 25.54x | 0.01x | 0.14x | 0.03x |
| `acos abort` | 17.73 ns | 354.72 ns | 756.01 ns | 26.47 us | 2.55 us | 13.75 us | 20.01x | 0.01x | 0.14x | 0.03x |
| `atan` | 14.93 ns | 168.87 ns | 160.67 ns | 18.31 us | 2.27 us | 18.83 us | 11.31x | 0.01x | 0.07x | 0.01x |
| `atan abort` | 21.17 ns | 174.69 ns | 162.19 ns | 18.28 us | 2.27 us | 18.82 us | 8.25x | 0.01x | 0.08x | 0.01x |
| `asinh` | 33.99 ns | 213.26 ns | 215.08 ns | 39.10 us | 1.65 us | 7.41 us | 6.27x | 0.01x | 0.13x | 0.03x |
| `asinh abort` | 31.93 ns | 213.06 ns | 220.02 ns | 39.23 us | 1.65 us | 7.40 us | 6.67x | 0.01x | 0.13x | 0.03x |
| `acosh` | 12.30 ns | 190.51 ns | 180.24 ns | 39.74 us | 3.30 us | 10.31 us | 15.49x | 0.00x | 0.06x | 0.02x |
| `acosh abort` | 15.44 ns | 195.40 ns | 189.00 ns | 39.83 us | 3.30 us | 10.34 us | 12.65x | 0.00x | 0.06x | 0.02x |
| `atanh` | 13.80 ns | 260.72 ns | 459.91 ns | 34.82 us | 1.25 us | 14.73 us | 18.90x | 0.01x | 0.21x | 0.02x |
| `atanh abort` | 17.00 ns | 260.89 ns | 460.13 ns | 34.86 us | 1.26 us | 14.62 us | 15.34x | 0.01x | 0.21x | 0.02x |
| `zero status` | 1.20 ns | 1.06 ns | 1.05 ns | 1.00 ns | 6.73 ns | 8.47 ns | 0.88x | 1.06x | 0.16x | 0.13x |
| `zero status abort` | 1.41 ns | 1.02 ns | 1.03 ns | 0.95 ns | 6.75 ns | 8.45 ns | 0.72x | 1.07x | 0.15x | 0.12x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 46.45 ns | 46.37 ns | 47.28 ns | - | 1.88 ns | 49.19x | 0.98x | - | 24.66x |
| `one` | 5.10 ns | 53.22 ns | 53.31 ns | 64.14 ns | - | 30.02 ns | 10.44x | 0.83x | - | 1.77x |
| `i` | 0.94 ns | 54.56 ns | 53.36 ns | 64.22 ns | - | 31.81 ns | 58.30x | 0.85x | - | 1.72x |
| `free i` | 0.94 ns | 54.56 ns | 54.36 ns | 63.92 ns | - | 31.82 ns | 57.84x | 0.85x | - | 1.71x |
| `conjugate` | 2.12 ns | 103.20 ns | 101.61 ns | 41.17 ns | - | 1.09 us | 48.58x | 2.51x | - | 0.09x |
| `norm squared` | 5.81 ns | 158.50 ns | 543.76 ns | 159.09 ns | - | 4.27 us | 27.30x | 1.00x | - | 0.04x |
| `reciprocal` | 12.72 ns | 1.72 us | 2.81 us | 451.04 ns | - | 10.67 us | 135.55x | 3.82x | - | 0.16x |
| `reciprocal checked` | 14.89 ns | 1.76 us | 2.82 us | 449.63 ns | - | 10.64 us | 118.28x | 3.92x | - | 0.17x |
| `powi` | 17.44 ns | 1.28 us | 6.45 us | 1.46 us | - | 43.29 us | 73.26x | 0.88x | - | 0.03x |
| `powi checked` | 17.57 ns | 1.28 us | 6.42 us | 1.46 us | - | 43.35 us | 72.87x | 0.88x | - | 0.03x |
| `div checked` | 16.52 ns | 2.39 us | 4.60 us | 791.02 ns | - | 21.53 us | 144.52x | 3.02x | - | 0.11x |
| `div real checked` | 17.54 ns | 652.27 ns | 627.16 ns | 266.37 ns | - | 5.16 us | 37.18x | 2.45x | - | 0.13x |
| `from scalar` | 1.41 ns | 69.89 ns | 69.59 ns | 45.12 ns | - | 11.10 ns | 49.39x | 1.55x | - | 6.30x |
| `add` | 6.31 ns | 486.62 ns | 846.08 ns | 106.34 ns | - | 2.54 us | 77.08x | 4.58x | - | 0.19x |
| `sub` | 6.24 ns | 539.75 ns | 1.13 us | 116.21 ns | - | 4.73 us | 86.44x | 4.64x | - | 0.11x |
| `neg` | 2.58 ns | 101.75 ns | 100.51 ns | 43.20 ns | - | 2.13 us | 39.36x | 2.36x | - | 0.05x |
| `mul` | 7.65 ns | 930.29 ns | 3.47 us | 304.62 ns | - | 9.95 us | 121.58x | 3.05x | - | 0.09x |
| `div` | 15.31 ns | 2.32 us | 4.61 us | 797.63 ns | - | 21.51 us | 151.24x | 2.90x | - | 0.11x |
| `div real` | 10.03 ns | 671.43 ns | 628.47 ns | 265.89 ns | - | 5.15 us | 66.91x | 2.53x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.74 ns | 212.38 ns | 2.40 us | 306.84 ns | 253.37 ns | 7.20 us | 24.30x | 0.69x | 0.84x | 0.03x |
| `vec3 magnitude` | 11.34 ns | 3.95 us | 5.36 us | 5.49 us | 341.99 ns | 8.55 us | 347.94x | 0.72x | 11.54x | 0.46x |
| `vec3 normalize` | 24.75 ns | 8.12 us | 11.35 us | 6.00 us | 599.56 ns | 16.59 us | 328.04x | 1.35x | 13.54x | 0.49x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.06 ns | 278.68 ns | 1.45 us | 72.02 ns | 57.18 ns | 741.00 ns | 91.17x | 3.87x | 4.87x | 0.38x |
| `vec3 zero` | 1.41 ns | 157.38 ns | 156.83 ns | 59.88 ns | 30.85 ns | 2.82 ns | 111.95x | 2.63x | 5.10x | 55.76x |
| `vec3 dot abort` | 8.47 ns | 212.89 ns | 882.43 ns | 253.48 ns | 203.35 ns | 7.08 us | 25.14x | 0.84x | 1.05x | 0.03x |
| `vec3 magnitude abort` | 17.44 ns | 3.98 us | 3.12 us | 5.42 us | 320.57 ns | 8.61 us | 228.10x | 0.73x | 12.41x | 0.46x |
| `vec3 normalize checked` | 25.15 ns | 8.22 us | 7.20 us | 6.00 us | 533.42 ns | 16.81 us | 327.00x | 1.37x | 15.42x | 0.49x |
| `vec3 normalize checked abort` | 28.82 ns | 8.27 us | 7.23 us | 6.00 us | 530.88 ns | 16.83 us | 286.85x | 1.38x | 15.57x | 0.49x |
| `vec3 div scalar checked` | 9.64 ns | 1.34 us | 1.55 us | 390.42 ns | - | - | 138.61x | 3.42x | - | - |
| `vec3 div scalar checked abort` | 17.66 ns | 1.27 us | 1.51 us | 393.06 ns | - | - | 72.03x | 3.24x | - | - |
| `vec3 add` | 6.55 ns | 961.61 ns | 1.72 us | 149.30 ns | 126.80 ns | 3.96 us | 146.71x | 6.44x | 7.58x | 0.24x |
| `vec3 add scalar` | 6.58 ns | 800.86 ns | 1.46 us | 153.64 ns | 132.50 ns | 3.80 us | 121.65x | 5.21x | 6.04x | 0.21x |
| `vec3 sub` | 6.61 ns | 1.06 us | 2.49 us | 166.74 ns | 134.61 ns | 7.26 us | 160.48x | 6.37x | 7.89x | 0.15x |
| `vec3 sub scalar` | 6.38 ns | 893.07 ns | 1.97 us | 146.50 ns | 123.94 ns | 6.98 us | 140.03x | 6.10x | 7.21x | 0.13x |
| `vec3 neg` | 3.77 ns | 145.44 ns | 145.35 ns | 56.25 ns | 46.24 ns | 3.11 us | 38.62x | 2.59x | 3.15x | 0.05x |
| `vec3 mul scalar` | 6.93 ns | 526.53 ns | 1.98 us | 162.87 ns | 120.10 ns | 4.31 us | 75.99x | 3.23x | 4.38x | 0.12x |
| `vec3 div scalar` | 9.28 ns | 1.27 us | 1.53 us | 406.10 ns | 168.78 ns | 7.54 us | 137.17x | 3.13x | 7.54x | 0.17x |
| `vec4 dot` | 9.63 ns | 268.11 ns | 641.27 ns | 428.32 ns | 316.48 ns | 9.58 us | 27.84x | 0.63x | 0.85x | 0.03x |
| `vec4 magnitude` | 16.83 ns | 3.76 us | 2.54 us | 5.49 us | 405.82 ns | 10.96 us | 223.20x | 0.68x | 9.26x | 0.34x |
| `vec4 normalize` | 36.57 ns | 8.56 us | 6.45 us | 6.11 us | 716.24 ns | 21.78 us | 234.16x | 1.40x | 11.95x | 0.39x |
| `vec4 add` | 7.79 ns | 1.17 us | 1.81 us | 208.71 ns | 171.06 ns | 5.18 us | 150.58x | 5.62x | 6.86x | 0.23x |
| `vec4 add scalar` | 6.81 ns | 1.00 us | 1.70 us | 219.07 ns | 175.19 ns | 4.99 us | 147.08x | 4.57x | 5.72x | 0.20x |
| `vec4 sub` | 5.23 ns | 1.27 us | 2.25 us | 216.08 ns | 172.90 ns | 9.44 us | 243.39x | 5.89x | 7.36x | 0.13x |
| `vec4 sub scalar` | 4.58 ns | 1.10 us | 2.02 us | 203.70 ns | 166.19 ns | 9.26 us | 241.43x | 5.42x | 6.65x | 0.12x |
| `vec4 neg` | 4.94 ns | 188.74 ns | 188.73 ns | 76.86 ns | 64.82 ns | 4.02 us | 38.24x | 2.46x | 2.91x | 0.05x |
| `vec4 mul scalar` | 7.37 ns | 646.87 ns | 2.01 us | 225.80 ns | 160.51 ns | 5.60 us | 87.75x | 2.86x | 4.03x | 0.12x |
| `vec4 div scalar` | 13.30 ns | 1.71 us | 1.49 us | 526.79 ns | 226.13 ns | 9.86 us | 128.46x | 3.24x | 7.56x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.57 ns | 727.83 ns | 2.15 us | 989.23 ns | 864.65 ns | 22.32 us | 37.19x | 0.74x | 0.84x | 0.03x |
| `mat3 inverse` | 92.05 ns | 14.56 us | 8.05 us | 3.27 us | 2.48 us | 81.79 us | 158.22x | 4.45x | 5.88x | 0.18x |
| `mat3 mul mat3` | 74.81 ns | 3.33 us | 7.79 us | 2.85 us | 2.31 us | 60.67 us | 44.45x | 1.17x | 1.44x | 0.05x |
| `mat3 transform vec3` | 28.56 ns | 1.57 us | 4.99 us | 1.04 us | 875.53 ns | 19.89 us | 55.10x | 1.52x | 1.80x | 0.08x |
| `mat4 determinant` | 42.67 ns | 2.16 us | 1.52 us | 4.60 us | 4.13 us | 94.72 us | 50.56x | 0.47x | 0.52x | 0.02x |
| `mat4 inverse` | 180.86 ns | 24.41 us | 8.41 us | 11.04 us | 9.00 us | 337.49 us | 134.96x | 2.21x | 2.71x | 0.07x |
| `mat4 mul mat4` | 138.50 ns | 6.29 us | 4.73 us | 6.41 us | 5.26 us | 140.12 us | 45.40x | 0.98x | 1.19x | 0.04x |
| `mat4 transform vec4` | 46.16 ns | 2.53 us | 2.51 us | 1.86 us | 1.67 us | 35.04 us | 54.81x | 1.36x | 1.51x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.53 ns | 1.30 us | 4.26 us | 206.68 ns | 235.54 ns | 2.11 us | 36.62x | 6.30x | 5.52x | 0.62x |
| `mat3 zero` | 16.28 ns | 538.16 ns | 520.61 ns | 273.72 ns | 207.38 ns | 11.44 ns | 33.06x | 1.97x | 2.60x | 47.04x |
| `mat3 identity` | 9.74 ns | 641.93 ns | 612.99 ns | 332.80 ns | 240.60 ns | 153.43 ns | 65.94x | 1.93x | 2.67x | 4.18x |
| `mat3 transpose` | 9.04 ns | 687.31 ns | 668.68 ns | 233.72 ns | 206.71 ns | 108.65 ns | 76.01x | 2.94x | 3.32x | 6.33x |
| `mat3 reciprocal` | 87.15 ns | 14.71 us | 24.50 us | 2.93 us | 2.26 us | 81.55 us | 168.74x | 5.02x | 6.52x | 0.18x |
| `mat3 reciprocal checked` | 88.17 ns | 14.52 us | 24.26 us | 2.90 us | 2.26 us | 81.62 us | 164.65x | 5.00x | 6.43x | 0.18x |
| `mat3 inverse checked` | 88.31 ns | 14.61 us | 24.30 us | 2.95 us | 2.26 us | 81.46 us | 165.39x | 4.95x | 6.46x | 0.18x |
| `mat3 inverse checked abort` | 93.91 ns | 14.53 us | 24.33 us | 2.91 us | 2.25 us | 81.41 us | 154.73x | 5.00x | 6.46x | 0.18x |
| `mat3 powi` | 108.62 ns | 4.41 us | 37.93 us | 6.77 us | 6.21 us | 152.91 us | 40.57x | 0.65x | 0.71x | 0.03x |
| `mat3 powi checked` | 108.52 ns | 4.38 us | 38.10 us | 6.78 us | 6.23 us | 150.33 us | 40.32x | 0.65x | 0.70x | 0.03x |
| `mat3 powi checked abort` | 108.74 ns | 4.39 us | 38.10 us | 6.77 us | 6.25 us | 149.21 us | 40.33x | 0.65x | 0.70x | 0.03x |
| `mat3 div scalar checked` | 23.63 ns | 4.37 us | 5.65 us | 1.48 us | 801.00 ns | 22.44 us | 185.12x | 2.96x | 5.46x | 0.19x |
| `mat3 div scalar checked abort` | 27.66 ns | 4.20 us | 5.65 us | 1.52 us | 803.06 ns | 22.03 us | 151.81x | 2.77x | 5.23x | 0.19x |
| `mat3 div matrix checked` | 133.83 ns | 17.87 us | 43.54 us | 5.41 us | 4.38 us | 158.49 us | 133.52x | 3.30x | 4.08x | 0.11x |
| `mat3 div matrix checked abort` | 147.02 ns | 17.85 us | 43.42 us | 5.50 us | 4.37 us | 159.97 us | 121.41x | 3.25x | 4.09x | 0.11x |
| `mat3 add` | 14.68 ns | 3.32 us | 5.33 us | 506.95 ns | 490.73 ns | 11.49 us | 226.02x | 6.54x | 6.76x | 0.29x |
| `mat3 add scalar` | 12.00 ns | 2.46 us | 5.23 us | 774.95 ns | 700.85 ns | 11.77 us | 205.18x | 3.18x | 3.51x | 0.21x |
| `mat3 sub` | 13.17 ns | 3.64 us | 7.27 us | 540.89 ns | 510.58 ns | 21.09 us | 276.17x | 6.73x | 7.13x | 0.17x |
| `mat3 sub scalar` | 10.84 ns | 2.73 us | 7.32 us | 785.49 ns | 696.22 ns | 21.33 us | 252.17x | 3.48x | 3.93x | 0.13x |
| `mat3 neg` | 10.21 ns | 619.16 ns | 621.05 ns | 475.92 ns | 463.19 ns | 8.47 us | 60.63x | 1.30x | 1.34x | 0.07x |
| `mat3 mul scalar` | 13.56 ns | 1.76 us | 6.01 us | 788.07 ns | 676.13 ns | 12.42 us | 129.58x | 2.23x | 2.60x | 0.14x |
| `mat3 div scalar` | 23.22 ns | 4.37 us | 5.67 us | 1.51 us | 803.51 ns | 21.74 us | 188.26x | 2.89x | 5.44x | 0.20x |
| `mat3 div matrix` | 133.96 ns | 18.33 us | 43.31 us | 5.55 us | 4.36 us | 158.09 us | 136.85x | 3.30x | 4.20x | 0.12x |
| `mat3 bitxor` | 108.21 ns | 4.39 us | 38.44 us | 6.72 us | 6.26 us | 148.76 us | 40.57x | 0.65x | 0.70x | 0.03x |
| `mat4 zero` | 11.53 ns | 792.36 ns | 913.79 ns | 554.12 ns | 348.35 ns | 14.12 ns | 68.73x | 1.43x | 2.27x | 56.13x |
| `mat4 identity` | 11.04 ns | 982.48 ns | 1.11 us | 611.36 ns | 413.96 ns | 232.41 ns | 89.01x | 1.61x | 2.37x | 4.23x |
| `mat4 transpose` | 9.34 ns | 1.18 us | 1.10 us | 481.70 ns | 366.45 ns | 178.36 ns | 126.07x | 2.44x | 3.21x | 6.60x |
| `mat4 reciprocal` | 176.44 ns | 24.51 us | 40.98 us | 10.60 us | 8.73 us | 337.88 us | 138.89x | 2.31x | 2.81x | 0.07x |
| `mat4 reciprocal checked` | 172.36 ns | 24.62 us | 41.63 us | 10.63 us | 8.70 us | 341.64 us | 142.82x | 2.32x | 2.83x | 0.07x |
| `mat4 powi` | 239.07 ns | 9.32 us | 52.11 us | 15.72 us | 13.84 us | 343.78 us | 39.01x | 0.59x | 0.67x | 0.03x |
| `mat4 powi checked` | 239.95 ns | 9.33 us | 52.30 us | 15.83 us | 13.74 us | 347.09 us | 38.89x | 0.59x | 0.68x | 0.03x |
| `mat4 add` | 52.34 ns | 5.72 us | 6.18 us | 909.87 ns | 847.73 ns | 19.82 us | 109.21x | 6.28x | 6.74x | 0.29x |
| `mat4 add scalar` | 20.58 ns | 4.29 us | 6.75 us | 1.41 us | 1.18 us | 20.31 us | 208.28x | 3.04x | 3.64x | 0.21x |
| `mat4 sub` | 38.76 ns | 5.93 us | 7.41 us | 952.94 ns | 893.54 ns | 35.55 us | 153.09x | 6.23x | 6.64x | 0.17x |
| `mat4 sub scalar` | 15.11 ns | 4.60 us | 7.78 us | 1.42 us | 1.16 us | 37.01 us | 304.15x | 3.23x | 3.95x | 0.12x |
| `mat4 neg` | 14.49 ns | 1.11 us | 1.01 us | 929.45 ns | 761.99 ns | 14.11 us | 76.89x | 1.20x | 1.46x | 0.08x |
| `mat4 mul scalar` | 24.62 ns | 3.05 us | 7.31 us | 1.45 us | 1.14 us | 20.62 us | 123.89x | 2.10x | 2.68x | 0.15x |
| `mat4 div scalar` | 31.56 ns | 7.48 us | 7.50 us | 2.66 us | 1.38 us | 37.36 us | 236.96x | 2.81x | 5.43x | 0.20x |
| `mat4 div matrix` | 191.48 ns | 28.04 us | 63.15 us | 16.62 us | 14.11 us | 526.74 us | 146.42x | 1.69x | 1.99x | 0.05x |
| `mat4 bitxor` | 239.84 ns | 9.35 us | 52.27 us | 15.91 us | 13.71 us | 350.53 us | 38.98x | 0.59x | 0.68x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.68 ns | 182.41 ns | 282.58 ns | - | - | 1.26 us | 49.57x | - | - | 0.14x |
| `scalar add ref_owned` | 12.47 ns | 183.11 ns | 283.97 ns | - | - | 1.26 us | 14.68x | - | - | 0.14x |
| `scalar add refs` | 5.35 ns | 164.85 ns | 265.83 ns | - | - | 1.27 us | 30.84x | - | - | 0.13x |
| `scalar add owned_ref_with_clone` | 8.97 ns | 203.10 ns | 301.51 ns | - | - | - | 22.63x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.50 ns | 197.18 ns | 298.18 ns | - | - | - | 35.84x | - | - | - |
| `scalar sub owned_ref` | 3.92 ns | 222.92 ns | 456.78 ns | - | - | 2.41 us | 56.94x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.56 ns | 223.50 ns | 457.44 ns | - | - | 2.40 us | 17.79x | - | - | 0.09x |
| `scalar sub refs` | 5.63 ns | 204.37 ns | 436.38 ns | - | - | 2.40 us | 36.29x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.05 ns | 237.66 ns | 475.96 ns | - | - | - | 26.27x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.74 ns | 233.04 ns | 470.43 ns | - | - | - | 40.63x | - | - | - |
| `scalar mul owned_ref` | 4.26 ns | 96.49 ns | 360.69 ns | - | - | 1.50 us | 22.65x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.65 ns | 98.08 ns | 365.46 ns | - | - | 1.51 us | 7.18x | - | - | 0.06x |
| `scalar mul refs` | 6.41 ns | 84.23 ns | 348.87 ns | - | - | 1.51 us | 13.14x | - | - | 0.06x |
| `scalar mul owned_ref_with_clone` | 9.75 ns | 109.31 ns | 375.33 ns | - | - | - | 11.21x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.25 ns | 107.86 ns | 377.03 ns | - | - | - | 17.27x | - | - | - |
| `scalar div owned_ref` | 5.78 ns | 259.67 ns | 311.02 ns | - | - | 2.53 us | 44.93x | - | - | 0.10x |
| `scalar div ref_owned` | 17.22 ns | 258.24 ns | 318.04 ns | - | - | 2.52 us | 15.00x | - | - | 0.10x |
| `scalar div refs` | 6.92 ns | 247.65 ns | 305.96 ns | - | - | 2.52 us | 35.77x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.34 ns | 278.08 ns | 333.37 ns | - | - | - | 20.84x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.44 ns | 276.11 ns | 329.81 ns | - | - | - | 17.88x | - | - | - |
| `vec3 add refs` | 6.06 ns | 630.94 ns | 1.41 us | - | - | 3.84 us | 104.08x | - | - | 0.16x |
| `vec3 sub refs` | 6.12 ns | 730.99 ns | 2.21 us | - | - | 7.24 us | 119.42x | - | - | 0.10x |
| `vec3 neg ref` | 3.30 ns | 158.91 ns | 159.09 ns | - | - | 3.08 us | 48.16x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.54 ns | 830.26 ns | 1.48 us | - | - | 3.71 us | 127.04x | - | - | 0.22x |
| `vec3 sub_scalar_ref` | 6.54 ns | 877.51 ns | 1.96 us | - | - | 6.95 us | 134.22x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 6.89 ns | 520.30 ns | 1.98 us | - | - | 4.32 us | 75.55x | - | - | 0.12x |
| `vec3 div_scalar_ref` | 7.86 ns | 1.27 us | 1.53 us | - | - | 7.49 us | 161.52x | - | - | 0.17x |
| `vec4 add refs` | 6.65 ns | 799.38 ns | 1.41 us | - | - | 5.20 us | 120.16x | - | - | 0.15x |
| `vec4 sub refs` | 3.07 ns | 879.18 ns | 1.85 us | - | - | 9.55 us | 286.29x | - | - | 0.09x |
| `vec4 neg ref` | 4.23 ns | 266.75 ns | 248.67 ns | - | - | 3.99 us | 63.03x | - | - | 0.07x |
| `vec4 add_scalar_ref` | 6.93 ns | 1.04 us | 1.80 us | - | - | 5.05 us | 150.21x | - | - | 0.21x |
| `vec4 sub_scalar_ref` | 4.22 ns | 1.17 us | 2.13 us | - | - | 9.26 us | 277.81x | - | - | 0.13x |
| `vec4 mul_scalar_ref` | 7.42 ns | 689.87 ns | 2.06 us | - | - | 5.55 us | 93.00x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.29 ns | 1.66 us | 1.45 us | - | - | 9.82 us | 146.76x | - | - | 0.17x |
| `mat3 add refs` | 10.98 ns | 1.75 us | 3.83 us | - | - | 11.37 us | 159.68x | - | - | 0.15x |
| `mat3 sub refs` | 10.50 ns | 2.05 us | 5.76 us | - | - | 21.05 us | 195.48x | - | - | 0.10x |
| `mat3 mul refs` | 54.22 ns | 2.27 us | 14.59 us | - | - | 60.22 us | 41.86x | - | - | 0.04x |
| `mat3 div refs` | 128.47 ns | 16.68 us | 42.42 us | - | - | 156.69 us | 129.86x | - | - | 0.11x |
| `mat3 neg ref` | 9.75 ns | 484.92 ns | 508.12 ns | - | - | 8.39 us | 49.71x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 44.30 ns | 2.49 us | 4.56 us | - | - | 11.75 us | 56.18x | - | - | 0.21x |
| `mat3 sub_scalar_ref` | 44.46 ns | 2.76 us | 6.53 us | - | - | 21.35 us | 62.00x | - | - | 0.13x |
| `mat3 mul_scalar_ref` | 46.56 ns | 1.78 us | 5.64 us | - | - | 11.94 us | 38.22x | - | - | 0.15x |
| `mat3 div_scalar_ref` | 20.74 ns | 4.14 us | 5.58 us | - | - | 21.82 us | 199.83x | - | - | 0.19x |
| `mat4 add refs` | 18.51 ns | 2.74 us | 3.58 us | - | - | 19.13 us | 148.08x | - | - | 0.14x |
| `mat4 sub refs` | 16.83 ns | 3.00 us | 4.76 us | - | - | 35.07 us | 178.40x | - | - | 0.09x |
| `mat4 mul refs` | 131.43 ns | 4.42 us | 19.31 us | - | - | 139.08 us | 33.63x | - | - | 0.03x |
| `mat4 div refs` | 186.35 ns | 26.20 us | 60.52 us | - | - | 521.93 us | 140.59x | - | - | 0.05x |
| `mat4 neg ref` | 12.47 ns | 874.96 ns | 846.74 ns | - | - | 13.98 us | 70.14x | - | - | 0.06x |
| `mat4 add_scalar_ref` | 52.51 ns | 4.22 us | 6.70 us | - | - | 20.15 us | 80.34x | - | - | 0.21x |
| `mat4 sub_scalar_ref` | 39.12 ns | 4.52 us | 7.62 us | - | - | 36.63 us | 115.65x | - | - | 0.12x |
| `mat4 mul_scalar_ref` | 56.31 ns | 2.98 us | 7.34 us | - | - | 19.85 us | 52.91x | - | - | 0.15x |
| `mat4 div_scalar_ref` | 26.34 ns | 7.30 us | 7.52 us | - | - | 37.16 us | 277.04x | - | - | 0.20x |
| `mat3 transform_vec refs` | 27.36 ns | 916.61 ns | 4.95 us | - | - | 19.58 us | 33.50x | - | - | 0.05x |
| `mat4 transform_vec refs` | 43.44 ns | 1.53 us | 5.80 us | - | - | 34.40 us | 35.29x | - | - | 0.04x |
| `complex add refs` | 7.75 ns | 337.44 ns | 702.77 ns | - | - | 2.49 us | 43.56x | - | - | 0.14x |
| `complex sub refs` | 8.02 ns | 391.19 ns | 970.65 ns | - | - | 4.76 us | 48.78x | - | - | 0.08x |
| `complex mul refs` | 7.97 ns | 788.03 ns | 3.33 us | - | - | 9.95 us | 98.92x | - | - | 0.08x |
| `complex div refs` | 16.00 ns | 2.20 us | 4.41 us | - | - | 21.42 us | 137.28x | - | - | 0.10x |
| `complex neg ref` | 2.36 ns | 72.31 ns | 70.01 ns | - | - | 2.10 us | 30.69x | - | - | 0.03x |
| `complex div_real_ref` | 7.26 ns | 646.38 ns | 607.72 ns | - | - | 5.12 us | 88.99x | - | - | 0.13x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.94 us |
| `astro sin 160` | 14.88 us |
| `astro sin 192` | 14.20 us |
| `astro sin 256` | 16.16 us |
