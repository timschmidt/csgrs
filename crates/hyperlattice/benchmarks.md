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
| `sin 0.1` | 10.77 ns | 143.91 ns | 143.88 ns | 10.65 us | 734.94 ns | 1.80 us | 13.37x | 0.01x | 0.20x | 0.08x |
| `cos 0.1` | 11.55 ns | 141.33 ns | 142.07 ns | 10.11 us | 482.50 ns | 1.64 us | 12.23x | 0.01x | 0.29x | 0.09x |
| `sin 1.23456789` | 11.57 ns | 198.37 ns | 183.66 ns | 12.18 us | 786.89 ns | 1.77 us | 17.15x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 11.83 ns | 190.53 ns | 175.17 ns | 10.22 us | 574.86 ns | 1.62 us | 16.10x | 0.02x | 0.33x | 0.12x |
| `sin 1e6` | 12.18 ns | 88.15 ns | 88.11 ns | 15.89 us | 1.08 us | 1.99 us | 7.24x | 0.01x | 0.08x | 0.04x |
| `cos 1e6` | 12.11 ns | 88.86 ns | 88.42 ns | 13.48 us | 798.03 ns | 1.81 us | 7.34x | 0.01x | 0.11x | 0.05x |
| `sin 1e30` | 65.27 ns | 88.34 ns | 88.35 ns | 18.09 us | 2.82 us | 3.51 us | 1.35x | 0.00x | 0.03x | 0.03x |
| `cos 1e30` | 67.27 ns | 88.73 ns | 88.90 ns | 15.17 us | 961.05 ns | 3.01 us | 1.32x | 0.01x | 0.09x | 0.03x |
| `sin pi_7` | 11.55 ns | 144.47 ns | 346.16 ns | 11.66 us | 724.68 ns | 1.84 us | 12.51x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.41 ns | 142.45 ns | 754.91 ns | 10.52 us | 521.44 ns | 1.68 us | 12.49x | 0.01x | 0.27x | 0.08x |
| `sin 1000pi_eps` | 11.37 ns | 88.26 ns | 780.65 ns | 15.70 us | 2.24 us | 2.79 us | 7.76x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.11 ns | 88.68 ns | 776.89 ns | 13.55 us | 556.75 ns | 1.67 us | 7.33x | 0.01x | 0.16x | 0.05x |
| `asin 0.5` | 10.72 ns | 441.13 ns | 441.49 ns | 49.13 us | 2.85 us | 12.90 us | 41.13x | 0.01x | 0.16x | 0.03x |
| `acos 0.5` | 10.90 ns | 983.09 ns | 978.62 ns | 56.89 us | 2.84 us | 12.87 us | 90.21x | 0.02x | 0.35x | 0.08x |
| `atanh 0.5` | 14.17 ns | 1.43 us | 1.41 us | 33.54 us | 1.63 us | 12.69 us | 101.13x | 0.04x | 0.88x | 0.11x |
| `asin neg_0.999999` | 13.90 ns | 755.51 ns | 1.18 us | 13.60 us | 2.42 us | 12.75 us | 54.36x | 0.06x | 0.31x | 0.06x |
| `acos neg_0.999999` | 14.91 ns | 873.84 ns | 1.71 us | 17.89 us | 2.61 us | 12.76 us | 58.61x | 0.05x | 0.34x | 0.07x |
| `atanh neg_0.999999` | 14.03 ns | 387.55 ns | 567.89 ns | 35.47 us | 1.56 us | 12.39 us | 27.63x | 0.01x | 0.25x | 0.03x |
| `asin 0.999999` | 13.50 ns | 728.97 ns | 1.37 us | 13.48 us | 2.44 us | 12.55 us | 53.99x | 0.05x | 0.30x | 0.06x |
| `acos 0.999999` | 13.99 ns | 612.95 ns | 1.48 us | 17.69 us | 2.65 us | 12.70 us | 43.81x | 0.03x | 0.23x | 0.05x |
| `atanh 0.999999` | 14.26 ns | 448.47 ns | 838.27 ns | 30.52 us | 1.57 us | 12.28 us | 31.45x | 0.01x | 0.29x | 0.04x |
| `asin 1e-12` | 9.19 ns | 463.54 ns | 1.06 us | 7.82 us | 1.37 us | 14.79 us | 50.47x | 0.06x | 0.34x | 0.03x |
| `acos 1e-12` | 9.69 ns | 717.67 ns | 1.52 us | 9.31 us | 1.39 us | 14.82 us | 74.03x | 0.08x | 0.52x | 0.05x |
| `atanh 1e-12` | 9.63 ns | 456.45 ns | 829.29 ns | 35.75 us | 165.36 ns | 19.53 us | 47.40x | 0.01x | 2.76x | 0.02x |
| `atan 0.5` | 14.52 ns | 291.03 ns | 288.77 ns | 34.04 us | 2.67 us | 17.43 us | 20.05x | 0.01x | 0.11x | 0.02x |
| `asinh 0.5` | 26.25 ns | 419.13 ns | 419.13 ns | 38.58 us | 1.56 us | 7.23 us | 15.97x | 0.01x | 0.27x | 0.06x |
| `atan neg_1e-12` | 14.38 ns | 369.63 ns | 584.76 ns | 1.52 us | 1.09 us | 15.18 us | 25.71x | 0.24x | 0.34x | 0.02x |
| `asinh neg_1e-12` | 15.36 ns | 473.43 ns | 379.00 ns | 41.31 us | 8.34 us | 11.48 us | 30.83x | 0.01x | 0.06x | 0.04x |
| `atan 1e6` | 14.86 ns | 180.24 ns | 178.41 ns | 2.69 us | 1.40 us | 17.54 us | 12.13x | 0.07x | 0.13x | 0.01x |
| `asinh 1e6` | 26.12 ns | 297.21 ns | 299.41 ns | 35.95 us | 1.59 us | 6.99 us | 11.38x | 0.01x | 0.19x | 0.04x |
| `atan neg_1e6` | 15.23 ns | 317.92 ns | 317.93 ns | 2.74 us | 1.39 us | 17.71 us | 20.88x | 0.12x | 0.23x | 0.02x |
| `asinh neg_1e6` | 26.38 ns | 335.23 ns | 336.62 ns | 36.10 us | 1.57 us | 6.83 us | 12.71x | 0.01x | 0.21x | 0.05x |
| `acosh 9` | 12.90 ns | 171.66 ns | 172.21 ns | 40.79 us | 1.56 us | 9.52 us | 13.31x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 11.86 ns | 515.88 ns | 1.39 us | 40.36 us | 8.12 us | 11.07 us | 43.50x | 0.01x | 0.06x | 0.05x |
| `acosh 1e6` | 12.72 ns | 174.15 ns | 172.87 ns | 35.62 us | 1.58 us | 9.56 us | 13.69x | 0.00x | 0.11x | 0.02x |
| `acosh e` | 12.72 ns | 515.11 ns | 2.34 us | 39.54 us | 1.58 us | 9.40 us | 40.49x | 0.01x | 0.33x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 18.00 ns | 17.97 ns | 24.57 ns | 15.44 ns | 0.92 ns | 38.52x | 0.73x | 1.17x | 19.51x |
| `one` | 0.46 ns | 26.27 ns | 26.36 ns | 40.64 ns | 30.47 ns | 32.36 ns | 56.66x | 0.65x | 0.86x | 0.81x |
| `e` | 0.47 ns | 87.34 ns | 90.28 ns | 52.21 ns | 1.04 us | 226.33 ns | 185.88x | 1.67x | 0.08x | 0.39x |
| `pi` | 0.46 ns | 49.30 ns | 50.50 ns | 42.99 ns | 49.37 ns | 225.89 ns | 106.85x | 1.15x | 1.00x | 0.22x |
| `tau` | 0.47 ns | 49.04 ns | 50.52 ns | 116.95 ns | 104.90 ns | 1.85 us | 104.15x | 0.42x | 0.47x | 0.03x |
| `add` | 5.04 ns | 205.50 ns | 309.28 ns | 51.08 ns | 41.93 ns | 1.24 us | 40.80x | 4.02x | 4.90x | 0.17x |
| `sub` | 5.10 ns | 210.82 ns | 366.12 ns | 55.30 ns | 44.90 ns | 2.39 us | 41.32x | 3.81x | 4.69x | 0.09x |
| `neg` | 5.04 ns | 40.02 ns | 39.51 ns | 23.35 ns | 19.76 ns | 1.03 us | 7.94x | 1.71x | 2.03x | 0.04x |
| `mul` | 5.35 ns | 133.77 ns | 390.69 ns | 57.11 ns | 44.34 ns | 1.52 us | 25.02x | 2.34x | 3.02x | 0.09x |
| `div` | 8.69 ns | 312.32 ns | 353.67 ns | 133.33 ns | 61.59 ns | 2.50 us | 35.92x | 2.34x | 5.07x | 0.12x |
| `reciprocal` | 8.61 ns | 77.38 ns | 79.53 ns | 155.00 ns | 58.34 ns | 1.49 us | 8.99x | 0.50x | 1.33x | 0.05x |
| `reciprocal checked` | 8.67 ns | 75.15 ns | 77.22 ns | 157.05 ns | 58.16 ns | 1.51 us | 8.67x | 0.48x | 1.29x | 0.05x |
| `reciprocal checked abort` | 24.19 ns | 90.27 ns | 89.54 ns | 154.44 ns | 58.09 ns | 1.51 us | 3.73x | 0.58x | 1.55x | 0.06x |
| `pow` | 19.68 ns | 8.24 us | 7.29 us | 52.65 us | 2.79 us | 2.34 us | 418.65x | 0.16x | 2.96x | 3.52x |
| `powi` | 5.66 ns | 367.33 ns | 2.54 us | 279.49 ns | 85.71 ns | 1.56 us | 64.94x | 1.31x | 4.29x | 0.24x |
| `exp` | 19.36 ns | 1.49 us | 2.07 us | 13.84 us | 922.68 ns | 1.84 us | 77.01x | 0.11x | 1.62x | 0.81x |
| `ln` | 10.57 ns | 1.56 us | 2.37 us | 28.66 us | 1.29 us | 1.79 us | 147.42x | 0.05x | 1.21x | 0.87x |
| `log10` | 13.30 ns | 4.60 us | 5.72 us | 34.72 us | 2.68 us | 6.54 us | 346.10x | 0.13x | 1.72x | 0.70x |
| `log10 abort` | 16.85 ns | 4.53 us | 5.77 us | 34.55 us | 2.69 us | 6.52 us | 269.03x | 0.13x | 1.69x | 0.70x |
| `sqrt` | 8.06 ns | 1.53 us | 1.54 us | 4.96 us | 94.56 ns | 1.43 us | 190.29x | 0.31x | 16.22x | 1.07x |
| `sin` | 14.78 ns | 118.45 ns | 122.85 ns | 13.57 us | 1.22 us | 2.19 us | 8.01x | 0.01x | 0.10x | 0.05x |
| `cos` | 18.22 ns | 121.48 ns | 118.96 ns | 11.76 us | 613.92 ns | 1.73 us | 6.67x | 0.01x | 0.20x | 0.07x |
| `tan` | 24.08 ns | 162.82 ns | 165.68 ns | 29.63 us | 1.55 us | 6.53 us | 6.76x | 0.01x | 0.10x | 0.02x |
| `sinh` | 17.88 ns | 3.98 us | 4.00 us | 3.21 us | 1.11 us | 10.62 us | 222.83x | 1.24x | 3.59x | 0.38x |
| `cosh` | 17.81 ns | 3.92 us | 3.99 us | 7.80 us | 1.04 us | 9.55 us | 220.06x | 0.50x | 3.76x | 0.41x |
| `tanh` | 23.28 ns | 8.20 us | 8.22 us | 3.26 us | 1.17 us | 22.76 us | 352.42x | 2.52x | 7.04x | 0.36x |
| `asin` | 12.93 ns | 614.76 ns | 1.45 us | 20.77 us | 2.34 us | 13.66 us | 47.54x | 0.03x | 0.26x | 0.04x |
| `asin abort` | 17.04 ns | 640.20 ns | 1.45 us | 20.81 us | 2.37 us | 13.62 us | 37.57x | 0.03x | 0.27x | 0.05x |
| `acos` | 13.84 ns | 838.63 ns | 2.09 us | 25.62 us | 2.49 us | 13.43 us | 60.60x | 0.03x | 0.34x | 0.06x |
| `acos abort` | 17.57 ns | 850.58 ns | 2.07 us | 25.73 us | 2.48 us | 13.68 us | 48.40x | 0.03x | 0.34x | 0.06x |
| `atan` | 14.69 ns | 269.59 ns | 912.90 ns | 17.93 us | 2.18 us | 18.78 us | 18.36x | 0.02x | 0.12x | 0.01x |
| `atan abort` | 21.22 ns | 278.20 ns | 925.10 ns | 17.57 us | 2.18 us | 18.59 us | 13.11x | 0.02x | 0.13x | 0.01x |
| `asinh` | 33.49 ns | 416.96 ns | 1.07 us | 38.28 us | 1.60 us | 7.32 us | 12.45x | 0.01x | 0.26x | 0.06x |
| `asinh abort` | 30.45 ns | 416.48 ns | 1.06 us | 38.36 us | 1.58 us | 7.35 us | 13.68x | 0.01x | 0.26x | 0.06x |
| `acosh` | 12.35 ns | 343.30 ns | 1.09 us | 38.67 us | 3.23 us | 10.16 us | 27.80x | 0.01x | 0.11x | 0.03x |
| `acosh abort` | 16.23 ns | 344.87 ns | 1.11 us | 39.12 us | 3.22 us | 10.11 us | 21.25x | 0.01x | 0.11x | 0.03x |
| `atanh` | 13.51 ns | 674.80 ns | 1.34 us | 33.66 us | 1.22 us | 14.40 us | 49.94x | 0.02x | 0.55x | 0.05x |
| `atanh abort` | 16.84 ns | 690.51 ns | 1.33 us | 33.97 us | 1.22 us | 14.45 us | 41.01x | 0.02x | 0.57x | 0.05x |
| `zero status` | 1.19 ns | 1.06 ns | 1.04 ns | 1.04 ns | 6.66 ns | 7.93 ns | 0.89x | 1.01x | 0.16x | 0.13x |
| `zero status abort` | 1.39 ns | 1.16 ns | 1.09 ns | 0.99 ns | 6.66 ns | 7.94 ns | 0.83x | 1.17x | 0.17x | 0.15x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.92 ns | 30.87 ns | 30.81 ns | 46.75 ns | - | 1.87 ns | 33.61x | 0.66x | - | 16.51x |
| `one` | 5.02 ns | 37.84 ns | 37.78 ns | 63.33 ns | - | 29.51 ns | 7.54x | 0.60x | - | 1.28x |
| `i` | 0.93 ns | 38.37 ns | 38.25 ns | 64.15 ns | - | 31.68 ns | 41.34x | 0.60x | - | 1.21x |
| `free i` | 0.93 ns | 39.91 ns | 38.51 ns | 64.49 ns | - | 31.21 ns | 42.95x | 0.62x | - | 1.28x |
| `conjugate` | 2.12 ns | 84.61 ns | 83.81 ns | 44.43 ns | - | 1.06 us | 40.00x | 1.90x | - | 0.08x |
| `norm squared` | 5.69 ns | 154.67 ns | 540.62 ns | 148.80 ns | - | 4.19 us | 27.16x | 1.04x | - | 0.04x |
| `reciprocal` | 18.09 ns | 1.71 us | 2.75 us | 440.48 ns | - | 10.47 us | 94.73x | 3.89x | - | 0.16x |
| `reciprocal checked` | 14.89 ns | 1.73 us | 2.78 us | 440.27 ns | - | 10.54 us | 116.23x | 3.93x | - | 0.16x |
| `powi` | 17.45 ns | 2.02 us | 13.14 us | 1.45 us | - | 43.77 us | 115.80x | 1.39x | - | 0.05x |
| `powi checked` | 17.83 ns | 2.04 us | 13.00 us | 1.44 us | - | 43.65 us | 114.56x | 1.42x | - | 0.05x |
| `div checked` | 23.32 ns | 2.63 us | 6.05 us | 775.81 ns | - | 21.40 us | 112.81x | 3.39x | - | 0.12x |
| `div real checked` | 9.28 ns | 665.49 ns | 617.99 ns | 260.57 ns | - | 5.16 us | 71.72x | 2.55x | - | 0.13x |
| `from scalar` | 1.38 ns | 57.98 ns | 57.42 ns | 47.22 ns | - | 11.45 ns | 41.92x | 1.23x | - | 5.06x |
| `add` | 6.15 ns | 459.94 ns | 824.17 ns | 105.91 ns | - | 2.51 us | 74.74x | 4.34x | - | 0.18x |
| `sub` | 6.13 ns | 460.97 ns | 847.74 ns | 114.26 ns | - | 4.71 us | 75.24x | 4.03x | - | 0.10x |
| `neg` | 2.57 ns | 88.62 ns | 87.27 ns | 46.01 ns | - | 2.13 us | 34.50x | 1.93x | - | 0.04x |
| `mul` | 7.53 ns | 857.95 ns | 3.11 us | 298.37 ns | - | 9.93 us | 113.96x | 2.88x | - | 0.09x |
| `div` | 18.04 ns | 2.64 us | 6.12 us | 764.66 ns | - | 21.43 us | 146.57x | 3.46x | - | 0.12x |
| `div real` | 9.80 ns | 653.45 ns | 618.60 ns | 259.51 ns | - | 5.16 us | 66.68x | 2.52x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.47 ns | 206.56 ns | 2.34 us | 302.89 ns | 246.97 ns | 7.12 us | 24.38x | 0.68x | 0.84x | 0.03x |
| `vec3 magnitude` | 15.88 ns | 3.95 us | 5.72 us | 5.27 us | 341.17 ns | 8.65 us | 248.75x | 0.75x | 11.57x | 0.46x |
| `vec3 normalize` | 26.93 ns | 8.19 us | 11.37 us | 5.92 us | 569.98 ns | 16.54 us | 304.08x | 1.38x | 14.37x | 0.50x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.06 ns | 223.01 ns | 1.43 us | 66.93 ns | 58.01 ns | 700.83 ns | 72.99x | 3.33x | 3.84x | 0.32x |
| `vec3 zero` | 1.40 ns | 131.58 ns | 131.45 ns | 63.70 ns | 30.30 ns | 2.76 ns | 94.24x | 2.07x | 4.34x | 47.62x |
| `vec3 dot abort` | 9.12 ns | 215.90 ns | 873.72 ns | 250.68 ns | 197.49 ns | 7.08 us | 23.66x | 0.86x | 1.09x | 0.03x |
| `vec3 magnitude abort` | 18.00 ns | 4.08 us | 3.51 us | 5.63 us | 319.54 ns | 8.60 us | 226.46x | 0.72x | 12.76x | 0.47x |
| `vec3 normalize checked` | 27.75 ns | 8.25 us | 7.56 us | 5.81 us | 532.32 ns | 16.51 us | 297.43x | 1.42x | 15.51x | 0.50x |
| `vec3 normalize checked abort` | 30.76 ns | 8.27 us | 7.56 us | 5.91 us | 530.10 ns | 16.56 us | 269.03x | 1.40x | 15.61x | 0.50x |
| `vec3 div scalar checked` | 10.43 ns | 1.33 us | 1.60 us | 390.82 ns | - | - | 127.70x | 3.41x | - | - |
| `vec3 div scalar checked abort` | 18.58 ns | 1.36 us | 1.56 us | 388.11 ns | - | - | 73.19x | 3.50x | - | - |
| `vec3 add` | 6.79 ns | 933.92 ns | 1.70 us | 148.14 ns | 124.56 ns | 3.83 us | 137.45x | 6.30x | 7.50x | 0.24x |
| `vec3 add scalar` | 6.28 ns | 749.39 ns | 1.45 us | 147.52 ns | 129.59 ns | 3.65 us | 119.30x | 5.08x | 5.78x | 0.21x |
| `vec3 sub` | 6.74 ns | 941.66 ns | 1.75 us | 162.70 ns | 135.16 ns | 7.14 us | 139.67x | 5.79x | 6.97x | 0.13x |
| `vec3 sub scalar` | 6.31 ns | 746.40 ns | 1.24 us | 142.65 ns | 121.63 ns | 6.96 us | 118.28x | 5.23x | 6.14x | 0.11x |
| `vec3 neg` | 3.74 ns | 141.33 ns | 140.25 ns | 57.08 ns | 49.83 ns | 3.08 us | 37.80x | 2.48x | 2.84x | 0.05x |
| `vec3 mul scalar` | 6.90 ns | 523.54 ns | 1.98 us | 163.98 ns | 124.61 ns | 4.32 us | 75.87x | 3.19x | 4.20x | 0.12x |
| `vec3 div scalar` | 18.42 ns | 1.33 us | 1.60 us | 386.15 ns | 177.70 ns | 7.61 us | 72.05x | 3.44x | 7.47x | 0.17x |
| `vec4 dot` | 9.56 ns | 245.65 ns | 619.78 ns | 433.68 ns | 313.56 ns | 9.49 us | 25.69x | 0.57x | 0.78x | 0.03x |
| `vec4 magnitude` | 17.40 ns | 3.84 us | 2.75 us | 5.48 us | 412.13 ns | 11.06 us | 220.65x | 0.70x | 9.32x | 0.35x |
| `vec4 normalize` | 33.74 ns | 8.54 us | 6.59 us | 6.14 us | 674.10 ns | 21.66 us | 253.15x | 1.39x | 12.67x | 0.39x |
| `vec4 add` | 7.21 ns | 1.15 us | 1.74 us | 198.28 ns | 169.56 ns | 5.12 us | 159.04x | 5.78x | 6.76x | 0.22x |
| `vec4 add scalar` | 6.92 ns | 1.01 us | 1.68 us | 206.15 ns | 173.44 ns | 4.97 us | 145.79x | 4.89x | 5.81x | 0.20x |
| `vec4 sub` | 5.08 ns | 1.14 us | 1.65 us | 206.29 ns | 171.92 ns | 9.47 us | 223.90x | 5.52x | 6.62x | 0.12x |
| `vec4 sub scalar` | 4.56 ns | 998.17 ns | 1.45 us | 198.90 ns | 167.27 ns | 9.23 us | 219.06x | 5.02x | 5.97x | 0.11x |
| `vec4 neg` | 4.96 ns | 197.49 ns | 194.10 ns | 77.83 ns | 59.49 ns | 3.99 us | 39.80x | 2.54x | 3.32x | 0.05x |
| `vec4 mul scalar` | 7.32 ns | 690.59 ns | 2.04 us | 213.93 ns | 154.42 ns | 5.50 us | 94.34x | 3.23x | 4.47x | 0.13x |
| `vec4 div scalar` | 14.30 ns | 1.71 us | 1.53 us | 521.66 ns | 222.40 ns | 9.89 us | 119.41x | 3.27x | 7.68x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.21 ns | 683.01 ns | 2.11 us | 944.86 ns | 827.68 ns | 21.97 us | 35.55x | 0.72x | 0.83x | 0.03x |
| `mat3 inverse` | 95.90 ns | 14.56 us | 8.57 us | 3.11 us | 2.47 us | 80.45 us | 151.87x | 4.68x | 5.91x | 0.18x |
| `mat3 mul mat3` | 51.48 ns | 2.91 us | 6.78 us | 2.67 us | 2.33 us | 59.97 us | 56.58x | 1.09x | 1.25x | 0.05x |
| `mat3 transform vec3` | 23.19 ns | 1.35 us | 4.41 us | 988.18 ns | 861.16 ns | 19.68 us | 58.10x | 1.36x | 1.56x | 0.07x |
| `mat4 determinant` | 42.02 ns | 2.05 us | 1.48 us | 4.47 us | 4.01 us | 93.30 us | 48.76x | 0.46x | 0.51x | 0.02x |
| `mat4 inverse` | 163.08 ns | 25.31 us | 9.32 us | 11.01 us | 8.82 us | 334.21 us | 155.22x | 2.30x | 2.87x | 0.08x |
| `mat4 mul mat4` | 112.32 ns | 5.98 us | 6.60 us | 6.30 us | 5.22 us | 138.50 us | 53.23x | 0.95x | 1.15x | 0.04x |
| `mat4 transform vec4` | 39.78 ns | 2.39 us | 2.64 us | 1.85 us | 1.63 us | 34.44 us | 59.97x | 1.29x | 1.46x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.16 ns | 1.06 us | 4.10 us | 212.71 ns | 233.50 ns | 2.03 us | 31.02x | 4.98x | 4.54x | 0.52x |
| `mat3 zero` | 15.76 ns | 504.68 ns | 505.37 ns | 245.16 ns | 202.70 ns | 11.47 ns | 32.02x | 2.06x | 2.49x | 44.01x |
| `mat3 identity` | 9.74 ns | 603.06 ns | 603.22 ns | 301.76 ns | 240.25 ns | 138.80 ns | 61.92x | 2.00x | 2.51x | 4.34x |
| `mat3 transpose` | 8.87 ns | 661.95 ns | 652.79 ns | 236.04 ns | 208.07 ns | 125.48 ns | 74.66x | 2.80x | 3.18x | 5.28x |
| `mat3 reciprocal` | 95.54 ns | 14.68 us | 24.57 us | 2.83 us | 2.29 us | 80.77 us | 153.65x | 5.18x | 6.42x | 0.18x |
| `mat3 reciprocal checked` | 126.07 ns | 14.64 us | 24.60 us | 2.84 us | 2.28 us | 81.21 us | 116.10x | 5.15x | 6.41x | 0.18x |
| `mat3 inverse checked` | 126.05 ns | 14.69 us | 24.43 us | 2.85 us | 2.27 us | 80.60 us | 116.52x | 5.15x | 6.46x | 0.18x |
| `mat3 inverse checked abort` | 130.85 ns | 14.72 us | 24.92 us | 2.86 us | 2.27 us | 80.97 us | 112.47x | 5.14x | 6.48x | 0.18x |
| `mat3 powi` | 101.88 ns | 5.51 us | 38.76 us | 6.56 us | 6.11 us | 147.55 us | 54.03x | 0.84x | 0.90x | 0.04x |
| `mat3 powi checked` | 101.05 ns | 5.51 us | 38.52 us | 6.55 us | 6.10 us | 144.98 us | 54.57x | 0.84x | 0.90x | 0.04x |
| `mat3 powi checked abort` | 102.86 ns | 5.45 us | 38.52 us | 6.59 us | 6.13 us | 148.11 us | 53.03x | 0.83x | 0.89x | 0.04x |
| `mat3 div scalar checked` | 24.10 ns | 4.20 us | 5.44 us | 1.41 us | 805.54 ns | 21.63 us | 174.28x | 2.99x | 5.21x | 0.19x |
| `mat3 div scalar checked abort` | 28.86 ns | 4.15 us | 5.47 us | 1.41 us | 802.62 ns | 21.63 us | 143.67x | 2.94x | 5.17x | 0.19x |
| `mat3 div matrix checked` | 118.18 ns | 18.80 us | 56.64 us | 5.23 us | 4.46 us | 157.47 us | 159.10x | 3.59x | 4.21x | 0.12x |
| `mat3 div matrix checked abort` | 130.21 ns | 18.52 us | 56.99 us | 5.24 us | 4.42 us | 157.00 us | 142.20x | 3.53x | 4.19x | 0.12x |
| `mat3 add` | 14.46 ns | 3.19 us | 5.23 us | 493.50 ns | 487.60 ns | 11.35 us | 220.22x | 6.45x | 6.53x | 0.28x |
| `mat3 add scalar` | 11.99 ns | 2.39 us | 4.52 us | 748.91 ns | 694.70 ns | 11.61 us | 199.61x | 3.20x | 3.45x | 0.21x |
| `mat3 sub` | 12.66 ns | 3.22 us | 5.12 us | 515.87 ns | 510.38 ns | 20.97 us | 254.30x | 6.24x | 6.31x | 0.15x |
| `mat3 sub scalar` | 10.79 ns | 2.40 us | 4.36 us | 770.69 ns | 696.78 ns | 21.03 us | 222.25x | 3.11x | 3.44x | 0.11x |
| `mat3 neg` | 10.17 ns | 595.27 ns | 587.16 ns | 466.54 ns | 459.34 ns | 8.47 us | 58.54x | 1.28x | 1.30x | 0.07x |
| `mat3 mul scalar` | 13.62 ns | 1.76 us | 5.58 us | 753.95 ns | 684.35 ns | 12.02 us | 129.44x | 2.34x | 2.58x | 0.15x |
| `mat3 div scalar` | 23.68 ns | 4.13 us | 5.40 us | 1.41 us | 799.47 ns | 21.76 us | 174.46x | 2.94x | 5.17x | 0.19x |
| `mat3 div matrix` | 127.45 ns | 18.40 us | 56.67 us | 5.27 us | 4.45 us | 155.27 us | 144.38x | 3.49x | 4.14x | 0.12x |
| `mat3 bitxor` | 101.69 ns | 5.51 us | 38.51 us | 6.58 us | 6.13 us | 147.58 us | 54.14x | 0.84x | 0.90x | 0.04x |
| `mat4 zero` | 11.34 ns | 979.63 ns | 992.44 ns | 466.11 ns | 349.19 ns | 14.54 ns | 86.36x | 2.10x | 2.81x | 67.36x |
| `mat4 identity` | 10.94 ns | 1.15 us | 1.16 us | 551.71 ns | 429.71 ns | 214.29 ns | 105.51x | 2.09x | 2.69x | 5.39x |
| `mat4 transpose` | 9.26 ns | 1.14 us | 1.14 us | 416.56 ns | 369.73 ns | 179.06 ns | 123.57x | 2.75x | 3.09x | 6.39x |
| `mat4 reciprocal` | 160.58 ns | 25.15 us | 43.80 us | 10.70 us | 8.89 us | 331.92 us | 156.63x | 2.35x | 2.83x | 0.08x |
| `mat4 reciprocal checked` | 163.73 ns | 24.96 us | 43.85 us | 10.61 us | 8.93 us | 333.06 us | 152.43x | 2.35x | 2.79x | 0.07x |
| `mat4 powi` | 224.57 ns | 11.78 us | 53.33 us | 15.23 us | 14.15 us | 339.31 us | 52.46x | 0.77x | 0.83x | 0.03x |
| `mat4 powi checked` | 223.31 ns | 11.74 us | 53.44 us | 15.18 us | 13.97 us | 337.95 us | 52.59x | 0.77x | 0.84x | 0.03x |
| `mat4 add` | 50.81 ns | 4.95 us | 5.72 us | 872.46 ns | 863.28 ns | 18.97 us | 97.39x | 5.67x | 5.73x | 0.26x |
| `mat4 add scalar` | 20.30 ns | 3.96 us | 6.56 us | 1.40 us | 1.20 us | 19.84 us | 194.98x | 2.84x | 3.29x | 0.20x |
| `mat4 sub` | 38.03 ns | 4.92 us | 5.81 us | 932.08 ns | 905.98 ns | 35.54 us | 129.48x | 5.28x | 5.44x | 0.14x |
| `mat4 sub scalar` | 15.00 ns | 4.01 us | 6.53 us | 1.43 us | 1.19 us | 36.70 us | 267.61x | 2.82x | 3.37x | 0.11x |
| `mat4 neg` | 14.26 ns | 986.89 ns | 998.62 ns | 887.98 ns | 757.95 ns | 13.72 us | 69.20x | 1.11x | 1.30x | 0.07x |
| `mat4 mul scalar` | 24.08 ns | 2.93 us | 7.35 us | 1.42 us | 1.13 us | 20.00 us | 121.74x | 2.07x | 2.59x | 0.15x |
| `mat4 div scalar` | 33.52 ns | 7.31 us | 7.55 us | 2.59 us | 1.37 us | 37.30 us | 217.97x | 2.82x | 5.32x | 0.20x |
| `mat4 div matrix` | 186.25 ns | 29.87 us | 84.01 us | 16.48 us | 14.28 us | 520.84 us | 160.37x | 1.81x | 2.09x | 0.06x |
| `mat4 bitxor` | 226.59 ns | 11.80 us | 54.20 us | 15.47 us | 14.06 us | 340.33 us | 52.06x | 0.76x | 0.84x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.61 ns | 171.67 ns | 276.77 ns | - | - | 1.27 us | 47.59x | - | - | 0.14x |
| `scalar add ref_owned` | 12.07 ns | 170.40 ns | 281.59 ns | - | - | 1.26 us | 14.12x | - | - | 0.14x |
| `scalar add refs` | 5.29 ns | 152.45 ns | 258.25 ns | - | - | 1.25 us | 28.82x | - | - | 0.12x |
| `scalar add owned_ref_with_clone` | 8.90 ns | 189.86 ns | 287.05 ns | - | - | - | 21.34x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.40 ns | 185.33 ns | 284.53 ns | - | - | - | 34.29x | - | - | - |
| `scalar sub owned_ref` | 3.84 ns | 176.85 ns | 333.79 ns | - | - | 2.41 us | 46.02x | - | - | 0.07x |
| `scalar sub ref_owned` | 12.24 ns | 173.99 ns | 331.82 ns | - | - | 2.37 us | 14.21x | - | - | 0.07x |
| `scalar sub refs` | 5.55 ns | 158.94 ns | 310.96 ns | - | - | 2.38 us | 28.62x | - | - | 0.07x |
| `scalar sub owned_ref_with_clone` | 8.90 ns | 194.86 ns | 344.45 ns | - | - | - | 21.91x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.59 ns | 184.18 ns | 342.56 ns | - | - | - | 32.97x | - | - | - |
| `scalar mul owned_ref` | 4.57 ns | 96.44 ns | 357.25 ns | - | - | 1.50 us | 21.12x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.29 ns | 93.18 ns | 364.27 ns | - | - | 1.51 us | 7.01x | - | - | 0.06x |
| `scalar mul refs` | 6.21 ns | 81.28 ns | 339.96 ns | - | - | 1.50 us | 13.09x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 9.72 ns | 107.72 ns | 368.74 ns | - | - | - | 11.08x | - | - | - |
| `scalar mul ref_owned_with_clone` | 7.62 ns | 105.37 ns | 368.38 ns | - | - | - | 13.82x | - | - | - |
| `scalar div owned_ref` | 5.80 ns | 258.88 ns | 308.50 ns | - | - | 2.51 us | 44.64x | - | - | 0.10x |
| `scalar div ref_owned` | 17.06 ns | 255.84 ns | 307.62 ns | - | - | 2.52 us | 15.00x | - | - | 0.10x |
| `scalar div refs` | 6.80 ns | 239.02 ns | 294.55 ns | - | - | 2.53 us | 35.15x | - | - | 0.09x |
| `scalar div owned_ref_with_clone` | 13.04 ns | 265.11 ns | 318.96 ns | - | - | - | 20.33x | - | - | - |
| `scalar div ref_owned_with_clone` | 7.90 ns | 267.27 ns | 322.81 ns | - | - | - | 33.84x | - | - | - |
| `vec3 add refs` | 6.26 ns | 609.70 ns | 1.36 us | - | - | 3.88 us | 97.37x | - | - | 0.16x |
| `vec3 sub refs` | 6.21 ns | 617.73 ns | 1.40 us | - | - | 7.17 us | 99.50x | - | - | 0.09x |
| `vec3 neg ref` | 3.24 ns | 152.31 ns | 156.76 ns | - | - | 3.12 us | 47.07x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.40 ns | 733.98 ns | 1.42 us | - | - | 3.71 us | 114.77x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.50 ns | 719.14 ns | 1.22 us | - | - | 6.97 us | 110.67x | - | - | 0.10x |
| `vec3 mul_scalar_ref` | 6.81 ns | 491.34 ns | 1.94 us | - | - | 4.28 us | 72.17x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.03 ns | 1.27 us | 1.52 us | - | - | 7.54 us | 158.35x | - | - | 0.17x |
| `vec4 add refs` | 6.44 ns | 735.89 ns | 1.34 us | - | - | 5.08 us | 114.30x | - | - | 0.14x |
| `vec4 sub refs` | 3.05 ns | 732.94 ns | 1.23 us | - | - | 9.51 us | 240.70x | - | - | 0.08x |
| `vec4 neg ref` | 4.16 ns | 221.15 ns | 219.81 ns | - | - | 3.96 us | 53.13x | - | - | 0.06x |
| `vec4 add_scalar_ref` | 6.90 ns | 990.90 ns | 1.64 us | - | - | 4.94 us | 143.68x | - | - | 0.20x |
| `vec4 sub_scalar_ref` | 4.16 ns | 979.10 ns | 1.41 us | - | - | 9.23 us | 235.37x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.35 ns | 640.67 ns | 2.00 us | - | - | 5.51 us | 87.15x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.75 ns | 1.70 us | 1.49 us | - | - | 9.85 us | 144.83x | - | - | 0.17x |
| `mat3 add refs` | 10.84 ns | 1.67 us | 3.67 us | - | - | 11.38 us | 153.56x | - | - | 0.15x |
| `mat3 sub refs` | 10.26 ns | 1.66 us | 3.66 us | - | - | 20.83 us | 161.68x | - | - | 0.08x |
| `mat3 mul refs` | 38.07 ns | 1.84 us | 12.28 us | - | - | 60.22 us | 48.43x | - | - | 0.03x |
| `mat3 div refs` | 116.82 ns | 17.53 us | 56.09 us | - | - | 155.69 us | 150.08x | - | - | 0.11x |
| `mat3 neg ref` | 9.81 ns | 497.25 ns | 475.25 ns | - | - | 8.39 us | 50.70x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 43.88 ns | 2.34 us | 4.47 us | - | - | 11.54 us | 53.29x | - | - | 0.20x |
| `mat3 sub_scalar_ref` | 44.31 ns | 2.32 us | 4.38 us | - | - | 21.15 us | 52.46x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 47.40 ns | 1.71 us | 5.56 us | - | - | 11.92 us | 36.10x | - | - | 0.14x |
| `mat3 div_scalar_ref` | 22.09 ns | 4.22 us | 5.59 us | - | - | 21.51 us | 190.99x | - | - | 0.20x |
| `mat4 add refs` | 17.61 ns | 2.69 us | 3.50 us | - | - | 18.94 us | 152.70x | - | - | 0.14x |
| `mat4 sub refs` | 16.92 ns | 2.70 us | 3.62 us | - | - | 35.18 us | 159.33x | - | - | 0.08x |
| `mat4 mul refs` | 98.39 ns | 4.24 us | 17.78 us | - | - | 139.83 us | 43.06x | - | - | 0.03x |
| `mat4 div refs` | 174.22 ns | 28.43 us | 83.30 us | - | - | 518.01 us | 163.18x | - | - | 0.05x |
| `mat4 neg ref` | 12.91 ns | 717.83 ns | 727.43 ns | - | - | 13.82 us | 55.58x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 48.19 ns | 4.18 us | 6.69 us | - | - | 19.89 us | 86.77x | - | - | 0.21x |
| `mat4 sub_scalar_ref` | 36.02 ns | 4.20 us | 6.58 us | - | - | 36.43 us | 116.61x | - | - | 0.12x |
| `mat4 mul_scalar_ref` | 51.69 ns | 2.98 us | 7.44 us | - | - | 19.74 us | 57.62x | - | - | 0.15x |
| `mat4 div_scalar_ref` | 27.56 ns | 7.38 us | 7.54 us | - | - | 37.04 us | 267.77x | - | - | 0.20x |
| `mat3 transform_vec refs` | 21.25 ns | 713.58 ns | 3.84 us | - | - | 19.53 us | 33.57x | - | - | 0.04x |
| `mat4 transform_vec refs` | 34.46 ns | 1.36 us | 5.00 us | - | - | 34.36 us | 39.54x | - | - | 0.04x |
| `complex add refs` | 7.65 ns | 317.10 ns | 681.34 ns | - | - | 2.47 us | 41.47x | - | - | 0.13x |
| `complex sub refs` | 7.94 ns | 319.37 ns | 710.54 ns | - | - | 4.72 us | 40.20x | - | - | 0.07x |
| `complex mul refs` | 7.84 ns | 721.28 ns | 3.06 us | - | - | 9.80 us | 91.98x | - | - | 0.07x |
| `complex div refs` | 17.20 ns | 2.51 us | 5.92 us | - | - | 21.27 us | 146.10x | - | - | 0.12x |
| `complex neg ref` | 2.31 ns | 70.44 ns | 70.37 ns | - | - | 2.09 us | 30.55x | - | - | 0.03x |
| `complex div_real_ref` | 7.18 ns | 631.33 ns | 593.69 ns | - | - | 5.05 us | 87.89x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 11.98 us |
| `astro sin 160` | 13.31 us |
| `astro sin 192` | 13.33 us |
| `astro sin 256` | 15.73 us |
