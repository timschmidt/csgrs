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
| `sin 0.1` | 11.07 ns | 149.79 ns | 148.35 ns | 11.24 us | 793.22 ns | 1.85 us | 13.54x | 0.01x | 0.19x | 0.08x |
| `cos 0.1` | 11.78 ns | 149.75 ns | 150.13 ns | 10.87 us | 498.57 ns | 1.65 us | 12.71x | 0.01x | 0.30x | 0.09x |
| `sin 1.23456789` | 12.06 ns | 207.76 ns | 197.15 ns | 13.02 us | 818.30 ns | 1.84 us | 17.22x | 0.02x | 0.25x | 0.11x |
| `cos 1.23456789` | 12.64 ns | 201.10 ns | 192.52 ns | 10.97 us | 603.86 ns | 1.69 us | 15.91x | 0.02x | 0.33x | 0.12x |
| `sin 1e6` | 13.26 ns | 96.88 ns | 96.47 ns | 16.41 us | 1.13 us | 2.02 us | 7.31x | 0.01x | 0.09x | 0.05x |
| `cos 1e6` | 12.93 ns | 95.07 ns | 96.17 ns | 14.17 us | 817.41 ns | 1.87 us | 7.35x | 0.01x | 0.12x | 0.05x |
| `sin 1e30` | 68.91 ns | 94.63 ns | 93.89 ns | 18.39 us | 2.96 us | 3.70 us | 1.37x | 0.01x | 0.03x | 0.03x |
| `cos 1e30` | 69.54 ns | 95.79 ns | 95.97 ns | 15.84 us | 970.07 ns | 3.28 us | 1.38x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.82 ns | 148.34 ns | 361.80 ns | 12.33 us | 755.44 ns | 1.95 us | 12.55x | 0.01x | 0.20x | 0.08x |
| `cos pi_7` | 11.77 ns | 151.18 ns | 792.28 ns | 11.53 us | 545.36 ns | 1.72 us | 12.84x | 0.01x | 0.28x | 0.09x |
| `sin 1000pi_eps` | 12.06 ns | 93.16 ns | 811.52 ns | 16.39 us | 2.39 us | 2.90 us | 7.73x | 0.01x | 0.04x | 0.03x |
| `cos 1000pi_eps` | 12.58 ns | 97.02 ns | 821.49 ns | 13.91 us | 575.46 ns | 1.76 us | 7.71x | 0.01x | 0.17x | 0.06x |
| `asin 0.5` | 11.56 ns | 470.29 ns | 467.90 ns | 50.46 us | 3.02 us | 13.53 us | 40.69x | 0.01x | 0.16x | 0.03x |
| `acos 0.5` | 11.80 ns | 1.05 us | 1.02 us | 59.78 us | 3.09 us | 13.50 us | 88.93x | 0.02x | 0.34x | 0.08x |
| `atanh 0.5` | 15.04 ns | 1.51 us | 1.47 us | 35.86 us | 1.69 us | 12.93 us | 100.33x | 0.04x | 0.89x | 0.12x |
| `asin neg_0.999999` | 14.30 ns | 800.53 ns | 1.28 us | 13.98 us | 2.71 us | 13.02 us | 55.97x | 0.06x | 0.30x | 0.06x |
| `acos neg_0.999999` | 15.70 ns | 894.23 ns | 1.82 us | 18.53 us | 2.82 us | 13.62 us | 56.96x | 0.05x | 0.32x | 0.07x |
| `atanh neg_0.999999` | 14.98 ns | 394.86 ns | 594.62 ns | 37.41 us | 1.63 us | 12.97 us | 26.37x | 0.01x | 0.24x | 0.03x |
| `asin 0.999999` | 13.93 ns | 774.56 ns | 1.48 us | 14.09 us | 2.74 us | 13.21 us | 55.59x | 0.05x | 0.28x | 0.06x |
| `acos 0.999999` | 14.50 ns | 642.09 ns | 1.58 us | 18.68 us | 2.89 us | 13.08 us | 44.28x | 0.03x | 0.22x | 0.05x |
| `atanh 0.999999` | 15.17 ns | 486.67 ns | 908.77 ns | 31.84 us | 1.60 us | 12.96 us | 32.08x | 0.02x | 0.30x | 0.04x |
| `asin 1e-12` | 9.90 ns | 482.21 ns | 1.17 us | 8.08 us | 1.54 us | 15.56 us | 48.72x | 0.06x | 0.31x | 0.03x |
| `acos 1e-12` | 10.52 ns | 739.21 ns | 1.63 us | 9.76 us | 1.53 us | 15.13 us | 70.28x | 0.08x | 0.48x | 0.05x |
| `atanh 1e-12` | 10.17 ns | 468.48 ns | 897.70 ns | 37.07 us | 172.96 ns | 20.68 us | 46.05x | 0.01x | 2.71x | 0.02x |
| `atan 0.5` | 15.34 ns | 303.94 ns | 315.09 ns | 36.04 us | 2.91 us | 18.00 us | 19.81x | 0.01x | 0.10x | 0.02x |
| `asinh 0.5` | 27.15 ns | 435.02 ns | 431.68 ns | 40.02 us | 1.62 us | 7.55 us | 16.02x | 0.01x | 0.27x | 0.06x |
| `atan neg_1e-12` | 14.93 ns | 335.45 ns | 567.37 ns | 1.65 us | 1.23 us | 15.35 us | 22.47x | 0.20x | 0.27x | 0.02x |
| `asinh neg_1e-12` | 16.07 ns | 485.54 ns | 413.06 ns | 43.07 us | 8.78 us | 12.18 us | 30.22x | 0.01x | 0.06x | 0.04x |
| `atan 1e6` | 15.78 ns | 194.80 ns | 195.41 ns | 2.85 us | 1.55 us | 18.19 us | 12.34x | 0.07x | 0.13x | 0.01x |
| `asinh 1e6` | 27.61 ns | 317.52 ns | 308.38 ns | 37.45 us | 1.64 us | 7.15 us | 11.50x | 0.01x | 0.19x | 0.04x |
| `atan neg_1e6` | 16.38 ns | 300.28 ns | 297.40 ns | 2.95 us | 1.52 us | 18.54 us | 18.33x | 0.10x | 0.20x | 0.02x |
| `asinh neg_1e6` | 28.12 ns | 365.91 ns | 350.61 ns | 38.36 us | 1.63 us | 7.09 us | 13.01x | 0.01x | 0.22x | 0.05x |
| `acosh 9` | 12.85 ns | 185.08 ns | 185.03 ns | 43.82 us | 1.63 us | 9.75 us | 14.40x | 0.00x | 0.11x | 0.02x |
| `acosh 1_plus_1e-12` | 12.62 ns | 525.87 ns | 1.46 us | 41.75 us | 8.39 us | 11.44 us | 41.67x | 0.01x | 0.06x | 0.05x |
| `acosh 1e6` | 12.78 ns | 183.84 ns | 187.26 ns | 36.72 us | 1.62 us | 10.12 us | 14.39x | 0.01x | 0.11x | 0.02x |
| `acosh e` | 12.77 ns | 530.80 ns | 2.39 us | 40.73 us | 1.65 us | 9.99 us | 41.57x | 0.01x | 0.32x | 0.05x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.49 ns | 18.66 ns | 18.68 ns | 24.98 ns | 16.50 ns | 0.97 ns | 37.89x | 0.75x | 1.13x | 19.28x |
| `one` | 0.49 ns | 24.43 ns | 26.85 ns | 41.31 ns | 30.87 ns | 32.94 ns | 49.46x | 0.59x | 0.79x | 0.74x |
| `e` | 0.50 ns | 91.48 ns | 90.08 ns | 53.17 ns | 1.07 us | 226.00 ns | 184.67x | 1.72x | 0.09x | 0.40x |
| `pi` | 0.48 ns | 53.14 ns | 49.78 ns | 50.20 ns | 50.69 ns | 223.96 ns | 110.32x | 1.06x | 1.05x | 0.24x |
| `tau` | 0.49 ns | 51.55 ns | 49.99 ns | 117.93 ns | 106.48 ns | 1.90 us | 105.41x | 0.44x | 0.48x | 0.03x |
| `add` | 5.29 ns | 214.46 ns | 323.47 ns | 54.74 ns | 43.57 ns | 1.30 us | 40.54x | 3.92x | 4.92x | 0.17x |
| `sub` | 5.24 ns | 216.93 ns | 395.27 ns | 57.53 ns | 46.84 ns | 2.51 us | 41.38x | 3.77x | 4.63x | 0.09x |
| `neg` | 5.19 ns | 40.33 ns | 41.98 ns | 24.15 ns | 20.37 ns | 1.10 us | 7.77x | 1.67x | 1.98x | 0.04x |
| `mul` | 5.63 ns | 138.52 ns | 412.09 ns | 58.87 ns | 44.87 ns | 1.58 us | 24.61x | 2.35x | 3.09x | 0.09x |
| `div` | 9.26 ns | 327.42 ns | 386.28 ns | 141.41 ns | 63.70 ns | 2.56 us | 35.34x | 2.32x | 5.14x | 0.13x |
| `reciprocal` | 9.11 ns | 87.77 ns | 81.56 ns | 162.52 ns | 59.39 ns | 1.54 us | 9.64x | 0.54x | 1.48x | 0.06x |
| `reciprocal checked` | 9.16 ns | 88.20 ns | 83.63 ns | 167.11 ns | 60.03 ns | 1.57 us | 9.63x | 0.53x | 1.47x | 0.06x |
| `reciprocal checked abort` | 25.59 ns | 91.51 ns | 105.96 ns | 170.75 ns | 59.85 ns | 1.56 us | 3.58x | 0.54x | 1.53x | 0.06x |
| `pow` | 20.62 ns | 10.16 us | 9.44 us | 54.35 us | 2.97 us | 2.35 us | 492.68x | 0.19x | 3.42x | 4.32x |
| `powi` | 6.31 ns | 379.51 ns | 2.70 us | 286.00 ns | 87.95 ns | 1.64 us | 60.16x | 1.33x | 4.32x | 0.23x |
| `exp` | 19.87 ns | 1.53 us | 2.22 us | 14.00 us | 982.98 ns | 1.91 us | 77.22x | 0.11x | 1.56x | 0.80x |
| `ln` | 11.51 ns | 1.22 us | 1.24 us | 29.78 us | 1.31 us | 1.84 us | 105.87x | 0.04x | 0.93x | 0.66x |
| `log10` | 14.68 ns | 4.06 us | 3.37 us | 35.49 us | 2.84 us | 7.15 us | 276.66x | 0.11x | 1.43x | 0.57x |
| `log10 abort` | 17.13 ns | 3.94 us | 3.36 us | 36.21 us | 2.83 us | 6.72 us | 230.08x | 0.11x | 1.39x | 0.59x |
| `sqrt` | 8.36 ns | 1.58 us | 1.49 us | 5.25 us | 97.97 ns | 1.44 us | 189.41x | 0.30x | 16.16x | 1.10x |
| `sin` | 15.18 ns | 127.24 ns | 119.98 ns | 14.57 us | 1.31 us | 2.21 us | 8.38x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.49 ns | 124.92 ns | 121.04 ns | 12.17 us | 652.88 ns | 1.74 us | 6.76x | 0.01x | 0.19x | 0.07x |
| `tan` | 25.25 ns | 176.80 ns | 176.06 ns | 30.30 us | 1.64 us | 6.68 us | 7.00x | 0.01x | 0.11x | 0.03x |
| `sinh` | 19.45 ns | 4.18 us | 4.22 us | 3.30 us | 1.15 us | 11.24 us | 214.96x | 1.27x | 3.64x | 0.37x |
| `cosh` | 19.07 ns | 4.03 us | 4.22 us | 8.08 us | 1.08 us | 9.71 us | 211.43x | 0.50x | 3.75x | 0.42x |
| `tanh` | 26.12 ns | 8.34 us | 8.71 us | 3.44 us | 1.20 us | 24.51 us | 319.11x | 2.43x | 6.94x | 0.34x |
| `asin` | 13.67 ns | 654.01 ns | 1.55 us | 22.06 us | 2.49 us | 14.34 us | 47.85x | 0.03x | 0.26x | 0.05x |
| `asin abort` | 18.06 ns | 664.50 ns | 1.57 us | 21.63 us | 2.51 us | 14.27 us | 36.79x | 0.03x | 0.27x | 0.05x |
| `acos` | 14.45 ns | 872.10 ns | 2.24 us | 26.67 us | 2.61 us | 13.88 us | 60.35x | 0.03x | 0.33x | 0.06x |
| `acos abort` | 18.06 ns | 884.17 ns | 2.26 us | 27.04 us | 2.65 us | 13.87 us | 48.96x | 0.03x | 0.33x | 0.06x |
| `atan` | 15.25 ns | 278.58 ns | 915.90 ns | 18.61 us | 2.46 us | 18.95 us | 18.27x | 0.01x | 0.11x | 0.01x |
| `atan abort` | 21.76 ns | 278.21 ns | 955.06 ns | 18.71 us | 2.47 us | 18.85 us | 12.79x | 0.01x | 0.11x | 0.01x |
| `asinh` | 35.12 ns | 434.82 ns | 1.14 us | 40.34 us | 1.69 us | 7.69 us | 12.38x | 0.01x | 0.26x | 0.06x |
| `asinh abort` | 32.55 ns | 408.58 ns | 1.09 us | 38.91 us | 1.65 us | 7.48 us | 12.55x | 0.01x | 0.25x | 0.05x |
| `acosh` | 13.64 ns | 350.62 ns | 1.20 us | 40.93 us | 3.32 us | 10.42 us | 25.70x | 0.01x | 0.11x | 0.03x |
| `acosh abort` | 16.75 ns | 369.43 ns | 1.18 us | 40.83 us | 3.33 us | 10.66 us | 22.06x | 0.01x | 0.11x | 0.03x |
| `atanh` | 14.19 ns | 746.62 ns | 1.41 us | 35.48 us | 1.27 us | 15.07 us | 52.62x | 0.02x | 0.59x | 0.05x |
| `atanh abort` | 17.17 ns | 745.83 ns | 1.43 us | 36.03 us | 1.29 us | 14.90 us | 43.44x | 0.02x | 0.58x | 0.05x |
| `zero status` | 1.21 ns | 1.78 ns | 1.71 ns | 1.02 ns | 6.92 ns | 8.07 ns | 1.47x | 1.74x | 0.26x | 0.22x |
| `zero status abort` | 1.44 ns | 3.54 ns | 3.44 ns | 0.99 ns | 7.07 ns | 8.01 ns | 2.45x | 3.56x | 0.50x | 0.44x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.97 ns | 47.56 ns | 46.16 ns | 47.93 ns | - | 1.96 ns | 49.15x | 0.99x | - | 24.22x |
| `one` | 5.35 ns | 38.66 ns | 38.12 ns | 64.36 ns | - | 32.05 ns | 7.23x | 0.60x | - | 1.21x |
| `i` | 0.98 ns | 40.21 ns | 39.25 ns | 65.29 ns | - | 33.54 ns | 40.86x | 0.62x | - | 1.20x |
| `free i` | 0.98 ns | 39.05 ns | 38.97 ns | 66.32 ns | - | 34.39 ns | 39.86x | 0.59x | - | 1.14x |
| `conjugate` | 2.13 ns | 86.61 ns | 87.28 ns | 43.16 ns | - | 1.11 us | 40.58x | 2.01x | - | 0.08x |
| `norm squared` | 5.87 ns | 322.00 ns | 1.52 us | 156.39 ns | - | 4.38 us | 54.82x | 2.06x | - | 0.07x |
| `reciprocal` | 18.86 ns | 1.79 us | 3.04 us | 457.30 ns | - | 10.76 us | 94.63x | 3.90x | - | 0.17x |
| `reciprocal checked` | 14.58 ns | 1.79 us | 3.04 us | 443.31 ns | - | 10.86 us | 122.76x | 4.04x | - | 0.16x |
| `powi` | 18.61 ns | 2.16 us | 14.18 us | 1.44 us | - | 44.55 us | 116.17x | 1.50x | - | 0.05x |
| `powi checked` | 18.62 ns | 2.13 us | 14.03 us | 1.46 us | - | 44.72 us | 114.55x | 1.46x | - | 0.05x |
| `div checked` | 26.74 ns | 2.74 us | 6.44 us | 788.41 ns | - | 21.88 us | 102.47x | 3.48x | - | 0.13x |
| `div real checked` | 9.73 ns | 687.19 ns | 656.25 ns | 260.65 ns | - | 5.24 us | 70.64x | 2.64x | - | 0.13x |
| `from scalar` | 1.48 ns | 57.43 ns | 55.68 ns | 47.59 ns | - | 10.91 ns | 38.83x | 1.21x | - | 5.27x |
| `add` | 6.18 ns | 490.97 ns | 896.68 ns | 110.25 ns | - | 2.62 us | 79.51x | 4.45x | - | 0.19x |
| `sub` | 6.14 ns | 478.80 ns | 902.22 ns | 119.46 ns | - | 4.89 us | 77.95x | 4.01x | - | 0.10x |
| `neg` | 2.61 ns | 93.75 ns | 91.68 ns | 44.85 ns | - | 2.29 us | 35.94x | 2.09x | - | 0.04x |
| `mul` | 7.52 ns | 878.49 ns | 3.35 us | 306.65 ns | - | 10.48 us | 116.81x | 2.86x | - | 0.08x |
| `div` | 17.84 ns | 2.81 us | 6.52 us | 786.90 ns | - | 22.06 us | 157.36x | 3.57x | - | 0.13x |
| `div real` | 10.36 ns | 669.77 ns | 636.43 ns | 262.13 ns | - | 5.39 us | 64.67x | 2.56x | - | 0.12x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.93 ns | 689.46 ns | 5.79 us | 323.72 ns | 256.93 ns | 7.37 us | 99.48x | 2.13x | 2.68x | 0.09x |
| `vec3 magnitude` | 9.81 ns | 4.57 us | 8.76 us | 5.55 us | 350.84 ns | 9.01 us | 466.09x | 0.82x | 13.03x | 0.51x |
| `vec3 normalize` | 25.61 ns | 8.87 us | 15.30 us | 6.16 us | 598.68 ns | 17.30 us | 346.20x | 1.44x | 14.81x | 0.51x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.16 ns | 221.43 ns | 1.47 us | 73.45 ns | 58.69 ns | 738.30 ns | 70.18x | 3.01x | 3.77x | 0.30x |
| `vec3 zero` | 1.44 ns | 138.32 ns | 140.36 ns | 59.34 ns | 31.46 ns | 2.84 ns | 95.73x | 2.33x | 4.40x | 48.68x |
| `vec3 dot abort` | 28.11 ns | 1.03 us | 4.41 us | 259.32 ns | 208.83 ns | 7.20 us | 36.48x | 3.95x | 4.91x | 0.14x |
| `vec3 magnitude abort` | 39.39 ns | 5.19 us | 6.73 us | 5.48 us | 331.33 ns | 8.77 us | 131.62x | 0.95x | 15.65x | 0.59x |
| `vec3 normalize checked` | 25.94 ns | 9.11 us | 10.64 us | 5.98 us | 558.58 ns | 17.15 us | 351.05x | 1.52x | 16.30x | 0.53x |
| `vec3 normalize checked abort` | 55.06 ns | 9.13 us | 10.98 us | 6.01 us | 572.58 ns | 17.15 us | 165.82x | 1.52x | 15.94x | 0.53x |
| `vec3 div scalar checked` | 10.53 ns | 1.34 us | 1.67 us | 400.92 ns | - | - | 127.39x | 3.34x | - | - |
| `vec3 div scalar checked abort` | 18.31 ns | 1.33 us | 1.68 us | 405.06 ns | - | - | 72.72x | 3.29x | - | - |
| `vec3 add` | 6.75 ns | 950.07 ns | 1.71 us | 154.97 ns | 125.79 ns | 4.01 us | 140.69x | 6.13x | 7.55x | 0.24x |
| `vec3 add scalar` | 6.43 ns | 805.91 ns | 1.53 us | 152.79 ns | 136.89 ns | 3.83 us | 125.25x | 5.27x | 5.89x | 0.21x |
| `vec3 sub` | 6.83 ns | 942.36 ns | 1.83 us | 171.14 ns | 139.77 ns | 7.42 us | 138.02x | 5.51x | 6.74x | 0.13x |
| `vec3 sub scalar` | 6.52 ns | 808.58 ns | 1.29 us | 150.43 ns | 126.85 ns | 7.14 us | 124.00x | 5.38x | 6.37x | 0.11x |
| `vec3 neg` | 3.89 ns | 147.85 ns | 147.61 ns | 58.13 ns | 48.18 ns | 3.26 us | 38.02x | 2.54x | 3.07x | 0.05x |
| `vec3 mul scalar` | 7.12 ns | 541.01 ns | 2.09 us | 164.55 ns | 122.06 ns | 4.54 us | 75.96x | 3.29x | 4.43x | 0.12x |
| `vec3 div scalar` | 10.16 ns | 1.34 us | 1.62 us | 412.59 ns | 170.80 ns | 7.80 us | 132.39x | 3.26x | 7.87x | 0.17x |
| `vec4 dot` | 7.36 ns | 807.93 ns | 3.46 us | 440.80 ns | 338.86 ns | 9.80 us | 109.72x | 1.83x | 2.38x | 0.08x |
| `vec4 magnitude` | 13.23 ns | 4.61 us | 5.58 us | 5.64 us | 434.03 ns | 11.23 us | 348.53x | 0.82x | 10.62x | 0.41x |
| `vec4 normalize` | 36.53 ns | 9.53 us | 9.24 us | 6.19 us | 735.95 ns | 22.37 us | 260.85x | 1.54x | 12.95x | 0.43x |
| `vec4 add` | 8.20 ns | 1.17 us | 1.80 us | 206.40 ns | 176.51 ns | 5.47 us | 143.16x | 5.69x | 6.65x | 0.21x |
| `vec4 add scalar` | 7.10 ns | 1.01 us | 1.71 us | 222.81 ns | 181.22 ns | 5.18 us | 142.60x | 4.55x | 5.59x | 0.20x |
| `vec4 sub` | 5.37 ns | 1.16 us | 1.73 us | 218.80 ns | 179.28 ns | 9.89 us | 216.57x | 5.32x | 6.49x | 0.12x |
| `vec4 sub scalar` | 4.57 ns | 991.15 ns | 1.57 us | 205.80 ns | 170.61 ns | 9.71 us | 216.83x | 4.82x | 5.81x | 0.10x |
| `vec4 neg` | 5.05 ns | 187.50 ns | 191.01 ns | 83.02 ns | 66.42 ns | 4.14 us | 37.10x | 2.26x | 2.82x | 0.05x |
| `vec4 mul scalar` | 7.38 ns | 687.52 ns | 2.14 us | 230.04 ns | 168.97 ns | 5.83 us | 93.12x | 2.99x | 4.07x | 0.12x |
| `vec4 div scalar` | 14.14 ns | 1.80 us | 1.63 us | 534.97 ns | 238.40 ns | 10.32 us | 127.22x | 3.36x | 7.55x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.67 ns | 1.59 us | 3.18 us | 970.28 ns | 895.16 ns | 22.27 us | 125.47x | 1.64x | 1.78x | 0.07x |
| `mat3 inverse` | 80.14 ns | 17.01 us | 10.88 us | 3.21 us | 2.55 us | 83.16 us | 212.22x | 5.30x | 6.66x | 0.20x |
| `mat3 mul mat3` | 73.77 ns | 6.64 us | 10.69 us | 2.79 us | 2.45 us | 61.38 us | 90.00x | 2.38x | 2.71x | 0.11x |
| `mat3 transform vec3` | 15.92 ns | 2.71 us | 10.44 us | 1.04 us | 904.57 ns | 20.18 us | 170.30x | 2.60x | 3.00x | 0.13x |
| `mat4 determinant` | 47.17 ns | 4.35 us | 3.25 us | 4.60 us | 4.31 us | 97.55 us | 92.25x | 0.95x | 1.01x | 0.04x |
| `mat4 inverse` | 149.70 ns | 32.31 us | 13.42 us | 11.04 us | 9.66 us | 357.31 us | 215.83x | 2.93x | 3.34x | 0.09x |
| `mat4 mul mat4` | 120.35 ns | 12.73 us | 10.18 us | 6.68 us | 6.94 us | 142.82 us | 105.80x | 1.91x | 1.84x | 0.09x |
| `mat4 transform vec4` | 26.26 ns | 4.29 us | 3.22 us | 1.99 us | 1.71 us | 35.51 us | 163.48x | 2.16x | 2.51x | 0.12x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 36.37 ns | 1.19 us | 4.50 us | 222.15 ns | 243.85 ns | 2.15 us | 32.72x | 5.36x | 4.88x | 0.55x |
| `mat3 zero` | 16.18 ns | 553.61 ns | 548.01 ns | 276.41 ns | 211.45 ns | 11.58 ns | 34.22x | 2.00x | 2.62x | 47.83x |
| `mat3 identity` | 10.20 ns | 645.50 ns | 643.61 ns | 332.44 ns | 238.29 ns | 161.82 ns | 63.27x | 1.94x | 2.71x | 3.99x |
| `mat3 transpose` | 9.66 ns | 686.20 ns | 688.69 ns | 243.41 ns | 210.82 ns | 117.98 ns | 71.04x | 2.82x | 3.25x | 5.82x |
| `mat3 reciprocal` | 81.15 ns | 17.32 us | 35.94 us | 2.96 us | 2.31 us | 84.98 us | 213.45x | 5.84x | 7.49x | 0.20x |
| `mat3 reciprocal checked` | 81.17 ns | 17.32 us | 35.95 us | 2.98 us | 2.28 us | 83.52 us | 213.44x | 5.81x | 7.59x | 0.21x |
| `mat3 inverse checked` | 82.96 ns | 17.93 us | 35.42 us | 3.00 us | 2.28 us | 87.22 us | 216.19x | 5.98x | 7.88x | 0.21x |
| `mat3 inverse checked abort` | 121.63 ns | 17.14 us | 35.74 us | 3.03 us | 2.42 us | 84.97 us | 140.95x | 5.66x | 7.09x | 0.20x |
| `mat3 powi` | 150.14 ns | 14.15 us | 91.41 us | 6.85 us | 6.38 us | 155.66 us | 94.26x | 2.07x | 2.22x | 0.09x |
| `mat3 powi checked` | 149.07 ns | 13.93 us | 90.79 us | 7.01 us | 6.45 us | 158.65 us | 93.45x | 1.99x | 2.16x | 0.09x |
| `mat3 powi checked abort` | 157.23 ns | 13.67 us | 89.61 us | 6.95 us | 6.47 us | 152.19 us | 86.97x | 1.97x | 2.11x | 0.09x |
| `mat3 div scalar checked` | 25.70 ns | 4.44 us | 5.84 us | 1.44 us | 830.41 ns | 22.52 us | 172.81x | 3.08x | 5.35x | 0.20x |
| `mat3 div scalar checked abort` | 30.96 ns | 4.34 us | 6.03 us | 1.42 us | 825.87 ns | 22.12 us | 140.21x | 3.05x | 5.26x | 0.20x |
| `mat3 div matrix checked` | 155.98 ns | 25.05 us | 61.22 us | 5.71 us | 4.43 us | 163.44 us | 160.63x | 4.39x | 5.65x | 0.15x |
| `mat3 div matrix checked abort` | 163.42 ns | 25.31 us | 60.49 us | 5.38 us | 4.42 us | 170.07 us | 154.91x | 4.71x | 5.72x | 0.15x |
| `mat3 add` | 14.79 ns | 3.12 us | 5.50 us | 519.31 ns | 511.46 ns | 12.00 us | 210.97x | 6.01x | 6.10x | 0.26x |
| `mat3 add scalar` | 12.19 ns | 2.38 us | 4.61 us | 794.04 ns | 728.15 ns | 12.19 us | 195.36x | 3.00x | 3.27x | 0.20x |
| `mat3 sub` | 14.40 ns | 3.27 us | 5.23 us | 556.49 ns | 535.34 ns | 21.69 us | 226.89x | 5.87x | 6.11x | 0.15x |
| `mat3 sub scalar` | 11.57 ns | 2.46 us | 4.41 us | 809.05 ns | 734.76 ns | 22.01 us | 212.42x | 3.04x | 3.34x | 0.11x |
| `mat3 neg` | 11.11 ns | 647.24 ns | 630.38 ns | 484.64 ns | 481.90 ns | 8.94 us | 58.27x | 1.34x | 1.34x | 0.07x |
| `mat3 mul scalar` | 13.97 ns | 1.74 us | 5.77 us | 794.88 ns | 682.60 ns | 12.75 us | 124.41x | 2.19x | 2.55x | 0.14x |
| `mat3 div scalar` | 26.14 ns | 4.38 us | 5.70 us | 1.47 us | 827.71 ns | 22.20 us | 167.48x | 2.97x | 5.29x | 0.20x |
| `mat3 div matrix` | 150.94 ns | 25.33 us | 61.46 us | 5.45 us | 4.47 us | 160.03 us | 167.85x | 4.65x | 5.67x | 0.16x |
| `mat3 bitxor` | 151.25 ns | 13.46 us | 89.45 us | 6.72 us | 6.33 us | 154.01 us | 88.98x | 2.00x | 2.12x | 0.09x |
| `mat4 zero` | 13.97 ns | 786.13 ns | 789.60 ns | 497.65 ns | 366.64 ns | 14.56 ns | 56.28x | 1.58x | 2.14x | 53.99x |
| `mat4 identity` | 10.78 ns | 967.90 ns | 976.58 ns | 544.75 ns | 411.41 ns | 237.80 ns | 89.77x | 1.78x | 2.35x | 4.07x |
| `mat4 transpose` | 10.32 ns | 1.18 us | 1.15 us | 429.96 ns | 366.14 ns | 197.41 ns | 113.92x | 2.73x | 3.21x | 5.95x |
| `mat4 reciprocal` | 144.98 ns | 32.84 us | 62.59 us | 11.05 us | 9.07 us | 363.34 us | 226.54x | 2.97x | 3.62x | 0.09x |
| `mat4 reciprocal checked` | 169.02 ns | 32.18 us | 61.99 us | 10.73 us | 9.08 us | 350.69 us | 190.40x | 3.00x | 3.54x | 0.09x |
| `mat4 powi` | 252.15 ns | 27.64 us | 108.68 us | 16.36 us | 14.10 us | 349.12 us | 109.60x | 1.69x | 1.96x | 0.08x |
| `mat4 powi checked` | 260.82 ns | 27.29 us | 108.76 us | 16.25 us | 14.22 us | 355.47 us | 104.64x | 1.68x | 1.92x | 0.08x |
| `mat4 add` | 54.09 ns | 5.47 us | 6.16 us | 947.06 ns | 871.11 ns | 19.64 us | 101.19x | 5.78x | 6.28x | 0.28x |
| `mat4 add scalar` | 21.90 ns | 4.12 us | 6.77 us | 1.41 us | 1.40 us | 21.17 us | 188.00x | 2.92x | 2.94x | 0.19x |
| `mat4 sub` | 40.10 ns | 5.31 us | 6.39 us | 992.58 ns | 933.81 ns | 37.24 us | 132.35x | 5.35x | 5.68x | 0.14x |
| `mat4 sub scalar` | 15.32 ns | 4.09 us | 6.68 us | 1.45 us | 1.41 us | 38.59 us | 266.97x | 2.82x | 2.91x | 0.11x |
| `mat4 neg` | 13.95 ns | 1.11 us | 1.12 us | 886.50 ns | 937.92 ns | 14.12 us | 79.58x | 1.25x | 1.18x | 0.08x |
| `mat4 mul scalar` | 23.99 ns | 3.01 us | 7.87 us | 1.39 us | 1.30 us | 20.29 us | 125.40x | 2.16x | 2.32x | 0.15x |
| `mat4 div scalar` | 33.32 ns | 7.87 us | 7.80 us | 2.61 us | 1.68 us | 37.66 us | 236.09x | 3.02x | 4.67x | 0.21x |
| `mat4 div matrix` | 231.56 ns | 45.45 us | 90.87 us | 17.06 us | 15.30 us | 534.70 us | 196.26x | 2.66x | 2.97x | 0.08x |
| `mat4 bitxor` | 256.59 ns | 28.20 us | 110.39 us | 16.20 us | 13.96 us | 346.47 us | 109.89x | 1.74x | 2.02x | 0.08x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.62 ns | 192.32 ns | 284.03 ns | - | - | 1.34 us | 53.16x | - | - | 0.14x |
| `scalar add ref_owned` | 12.32 ns | 182.69 ns | 281.60 ns | - | - | 1.35 us | 14.83x | - | - | 0.14x |
| `scalar add refs` | 5.53 ns | 165.86 ns | 270.25 ns | - | - | 1.30 us | 29.98x | - | - | 0.13x |
| `scalar add owned_ref_with_clone` | 9.26 ns | 199.62 ns | 297.94 ns | - | - | - | 21.55x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.70 ns | 192.39 ns | 303.56 ns | - | - | - | 33.78x | - | - | - |
| `scalar sub owned_ref` | 4.01 ns | 192.67 ns | 352.29 ns | - | - | 2.49 us | 47.99x | - | - | 0.08x |
| `scalar sub ref_owned` | 12.58 ns | 179.58 ns | 350.51 ns | - | - | 2.48 us | 14.27x | - | - | 0.07x |
| `scalar sub refs` | 5.64 ns | 160.65 ns | 328.56 ns | - | - | 2.49 us | 28.48x | - | - | 0.06x |
| `scalar sub owned_ref_with_clone` | 9.18 ns | 202.96 ns | 361.54 ns | - | - | - | 22.11x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.81 ns | 201.32 ns | 364.34 ns | - | - | - | 34.67x | - | - | - |
| `scalar mul owned_ref` | 4.52 ns | 105.89 ns | 384.98 ns | - | - | 1.53 us | 23.43x | - | - | 0.07x |
| `scalar mul ref_owned` | 13.50 ns | 102.30 ns | 383.39 ns | - | - | 1.58 us | 7.58x | - | - | 0.06x |
| `scalar mul refs` | 6.20 ns | 83.87 ns | 366.48 ns | - | - | 1.56 us | 13.52x | - | - | 0.05x |
| `scalar mul owned_ref_with_clone` | 10.10 ns | 115.56 ns | 405.41 ns | - | - | - | 11.44x | - | - | - |
| `scalar mul ref_owned_with_clone` | 8.06 ns | 114.47 ns | 412.89 ns | - | - | - | 14.20x | - | - | - |
| `scalar div owned_ref` | 6.03 ns | 267.87 ns | 318.86 ns | - | - | 2.56 us | 44.41x | - | - | 0.10x |
| `scalar div ref_owned` | 17.73 ns | 265.37 ns | 319.58 ns | - | - | 2.60 us | 14.97x | - | - | 0.10x |
| `scalar div refs` | 7.12 ns | 250.87 ns | 307.48 ns | - | - | 2.57 us | 35.24x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.75 ns | 285.63 ns | 334.78 ns | - | - | - | 20.77x | - | - | - |
| `scalar div ref_owned_with_clone` | 8.20 ns | 291.53 ns | 345.66 ns | - | - | - | 35.56x | - | - | - |
| `vec3 add refs` | 6.30 ns | 647.61 ns | 1.47 us | - | - | 4.01 us | 102.72x | - | - | 0.16x |
| `vec3 sub refs` | 6.40 ns | 669.95 ns | 1.50 us | - | - | 7.63 us | 104.76x | - | - | 0.09x |
| `vec3 neg ref` | 3.41 ns | 167.60 ns | 186.33 ns | - | - | 3.22 us | 49.11x | - | - | 0.05x |
| `vec3 add_scalar_ref` | 6.72 ns | 772.70 ns | 1.54 us | - | - | 3.85 us | 114.96x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.62 ns | 759.83 ns | 1.31 us | - | - | 7.16 us | 114.73x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 7.15 ns | 518.52 ns | 2.08 us | - | - | 4.51 us | 72.48x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 8.21 ns | 1.30 us | 1.58 us | - | - | 7.66 us | 158.51x | - | - | 0.17x |
| `vec4 add refs` | 6.65 ns | 772.59 ns | 1.43 us | - | - | 5.39 us | 116.11x | - | - | 0.14x |
| `vec4 sub refs` | 3.17 ns | 746.09 ns | 1.30 us | - | - | 10.15 us | 235.63x | - | - | 0.07x |
| `vec4 neg ref` | 4.28 ns | 205.78 ns | 208.77 ns | - | - | 4.19 us | 48.12x | - | - | 0.05x |
| `vec4 add_scalar_ref` | 7.00 ns | 976.62 ns | 1.73 us | - | - | 5.12 us | 139.49x | - | - | 0.19x |
| `vec4 sub_scalar_ref` | 4.31 ns | 1.03 us | 1.51 us | - | - | 9.64 us | 239.67x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.61 ns | 643.63 ns | 2.18 us | - | - | 5.86 us | 84.52x | - | - | 0.11x |
| `vec4 div_scalar_ref` | 12.50 ns | 1.80 us | 1.66 us | - | - | 10.42 us | 143.65x | - | - | 0.17x |
| `mat3 add refs` | 11.77 ns | 1.71 us | 3.90 us | - | - | 12.34 us | 145.59x | - | - | 0.14x |
| `mat3 sub refs` | 10.63 ns | 1.74 us | 3.85 us | - | - | 22.60 us | 163.31x | - | - | 0.08x |
| `mat3 mul refs` | 33.56 ns | 5.14 us | 28.26 us | - | - | 63.49 us | 153.11x | - | - | 0.08x |
| `mat3 div refs` | 140.99 ns | 23.38 us | 59.11 us | - | - | 167.37 us | 165.82x | - | - | 0.14x |
| `mat3 neg ref` | 10.19 ns | 503.47 ns | 483.49 ns | - | - | 8.65 us | 49.42x | - | - | 0.06x |
| `mat3 add_scalar_ref` | 45.27 ns | 2.38 us | 4.57 us | - | - | 12.10 us | 52.57x | - | - | 0.20x |
| `mat3 sub_scalar_ref` | 44.65 ns | 2.37 us | 4.40 us | - | - | 22.62 us | 53.00x | - | - | 0.10x |
| `mat3 mul_scalar_ref` | 48.59 ns | 1.73 us | 5.94 us | - | - | 12.46 us | 35.51x | - | - | 0.14x |
| `mat3 div_scalar_ref` | 22.72 ns | 4.41 us | 5.79 us | - | - | 22.17 us | 194.09x | - | - | 0.20x |
| `mat4 add refs` | 17.21 ns | 2.82 us | 3.60 us | - | - | 19.72 us | 163.76x | - | - | 0.14x |
| `mat4 sub refs` | 16.74 ns | 2.73 us | 3.64 us | - | - | 38.11 us | 162.99x | - | - | 0.07x |
| `mat4 mul refs` | 72.70 ns | 10.51 us | 26.80 us | - | - | 146.86 us | 144.58x | - | - | 0.07x |
| `mat4 div refs` | 177.86 ns | 44.95 us | 90.89 us | - | - | 546.42 us | 252.71x | - | - | 0.08x |
| `mat4 neg ref` | 12.28 ns | 761.91 ns | 756.02 ns | - | - | 14.65 us | 62.02x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 50.64 ns | 4.10 us | 6.80 us | - | - | 20.86 us | 80.99x | - | - | 0.20x |
| `mat4 sub_scalar_ref` | 38.21 ns | 4.11 us | 6.64 us | - | - | 37.93 us | 107.65x | - | - | 0.11x |
| `mat4 mul_scalar_ref` | 53.53 ns | 3.01 us | 7.91 us | - | - | 20.57 us | 56.30x | - | - | 0.15x |
| `mat4 div_scalar_ref` | 29.24 ns | 7.86 us | 7.77 us | - | - | 39.94 us | 268.85x | - | - | 0.20x |
| `mat3 transform_vec refs` | 14.65 ns | 2.01 us | 11.38 us | - | - | 19.95 us | 137.38x | - | - | 0.10x |
| `mat4 transform_vec refs` | 23.63 ns | 3.19 us | 11.07 us | - | - | 36.31 us | 135.07x | - | - | 0.09x |
| `complex add refs` | 7.70 ns | 329.67 ns | 719.28 ns | - | - | 2.68 us | 42.83x | - | - | 0.12x |
| `complex sub refs` | 7.92 ns | 335.00 ns | 754.75 ns | - | - | 4.84 us | 42.28x | - | - | 0.07x |
| `complex mul refs` | 8.41 ns | 738.85 ns | 3.11 us | - | - | 10.25 us | 87.87x | - | - | 0.07x |
| `complex div refs` | 17.80 ns | 2.68 us | 6.31 us | - | - | 22.91 us | 150.59x | - | - | 0.12x |
| `complex neg ref` | 2.39 ns | 79.94 ns | 72.65 ns | - | - | 2.18 us | 33.50x | - | - | 0.04x |
| `complex div_real_ref` | 7.38 ns | 653.03 ns | 634.79 ns | - | - | 5.38 us | 88.45x | - | - | 0.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 13.02 us |
| `astro sin 160` | 14.43 us |
| `astro sin 192` | 14.05 us |
| `astro sin 256` | 16.14 us |
