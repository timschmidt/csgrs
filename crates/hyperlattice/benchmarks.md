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
| `sin 0.1` | 11.11 ns | 155.65 ns | 155.25 ns | 10.89 us | 765.99 ns | 1.85 us | 14.01x | 0.01x | 0.20x | 0.08x |
| `cos 0.1` | 11.83 ns | 152.93 ns | 153.33 ns | 10.98 us | 497.32 ns | 1.75 us | 12.93x | 0.01x | 0.31x | 0.09x |
| `sin 1.23456789` | 13.04 ns | 210.62 ns | 197.68 ns | 12.60 us | 808.49 ns | 1.84 us | 16.15x | 0.02x | 0.26x | 0.11x |
| `cos 1.23456789` | 28.54 ns | 203.10 ns | 190.70 ns | 10.96 us | 587.38 ns | 1.67 us | 7.12x | 0.02x | 0.35x | 0.12x |
| `sin 1e6` | 17.77 ns | 101.98 ns | 101.00 ns | 16.22 us | 1.08 us | 2.12 us | 5.74x | 0.01x | 0.09x | 0.05x |
| `cos 1e6` | 12.63 ns | 101.46 ns | 103.56 ns | 13.50 us | 837.21 ns | 1.84 us | 8.03x | 0.01x | 0.12x | 0.06x |
| `sin 1e30` | 72.40 ns | 97.91 ns | 98.72 ns | 18.60 us | 2.83 us | 3.62 us | 1.35x | 0.01x | 0.03x | 0.03x |
| `cos 1e30` | 69.40 ns | 97.31 ns | 98.59 ns | 15.17 us | 975.92 ns | 3.09 us | 1.40x | 0.01x | 0.10x | 0.03x |
| `sin pi_7` | 11.89 ns | 155.15 ns | 349.22 ns | 12.16 us | 748.05 ns | 1.90 us | 13.05x | 0.01x | 0.21x | 0.08x |
| `cos pi_7` | 11.67 ns | 152.25 ns | 705.53 ns | 10.93 us | 583.80 ns | 1.74 us | 13.04x | 0.01x | 0.26x | 0.09x |
| `sin 1000pi_eps` | 11.75 ns | 100.62 ns | 766.14 ns | 16.36 us | 2.27 us | 2.86 us | 8.56x | 0.01x | 0.04x | 0.04x |
| `cos 1000pi_eps` | 12.41 ns | 101.95 ns | 753.54 ns | 13.76 us | 577.56 ns | 1.75 us | 8.22x | 0.01x | 0.18x | 0.06x |
| `asin 0.5` | 11.15 ns | 149.34 ns | 149.47 ns | 50.55 us | 2.96 us | 13.31 us | 13.40x | 0.00x | 0.05x | 0.01x |
| `acos 0.5` | 11.55 ns | 142.57 ns | 143.50 ns | 60.08 us | 2.92 us | 13.34 us | 12.34x | 0.00x | 0.05x | 0.01x |
| `atanh 0.5` | 14.44 ns | 153.26 ns | 153.60 ns | 34.46 us | 1.63 us | 12.93 us | 10.61x | 0.00x | 0.09x | 0.01x |
| `asin neg_0.999999` | 13.93 ns | 556.36 ns | 537.82 ns | 13.75 us | 2.54 us | 13.11 us | 39.94x | 0.04x | 0.22x | 0.04x |
| `acos neg_0.999999` | 15.19 ns | 366.72 ns | 335.12 ns | 18.15 us | 2.68 us | 13.12 us | 24.14x | 0.02x | 0.14x | 0.03x |
| `atanh neg_0.999999` | 14.31 ns | 273.75 ns | 247.31 ns | 37.21 us | 1.58 us | 12.71 us | 19.13x | 0.01x | 0.17x | 0.02x |
| `asin 0.999999` | 13.93 ns | 488.21 ns | 711.69 ns | 13.74 us | 2.56 us | 12.74 us | 35.06x | 0.04x | 0.19x | 0.04x |
| `acos 0.999999` | 14.09 ns | 352.30 ns | 821.13 ns | 18.00 us | 2.79 us | 13.09 us | 25.01x | 0.02x | 0.13x | 0.03x |
| `atanh 0.999999` | 14.52 ns | 297.43 ns | 516.72 ns | 32.85 us | 1.57 us | 12.57 us | 20.49x | 0.01x | 0.19x | 0.02x |
| `asin 1e-12` | 9.39 ns | 272.78 ns | 487.30 ns | 8.00 us | 1.41 us | 15.27 us | 29.06x | 0.03x | 0.19x | 0.02x |
| `acos 1e-12` | 10.15 ns | 537.66 ns | 967.04 ns | 9.53 us | 1.41 us | 15.18 us | 53.00x | 0.06x | 0.38x | 0.04x |
| `atanh 1e-12` | 9.61 ns | 313.26 ns | 509.04 ns | 36.58 us | 169.38 ns | 20.12 us | 32.59x | 0.01x | 1.85x | 0.02x |
| `atan 0.5` | 14.97 ns | 167.23 ns | 164.52 ns | 35.47 us | 2.75 us | 17.56 us | 11.17x | 0.00x | 0.06x | 0.01x |
| `asinh 0.5` | 26.86 ns | 226.90 ns | 214.17 ns | 39.53 us | 1.61 us | 7.59 us | 8.45x | 0.01x | 0.14x | 0.03x |
| `atan neg_1e-12` | 14.34 ns | 283.70 ns | 272.67 ns | 1.57 us | 1.14 us | 15.38 us | 19.78x | 0.18x | 0.25x | 0.02x |
| `asinh neg_1e-12` | 15.67 ns | 465.05 ns | 381.52 ns | 42.56 us | 8.76 us | 11.88 us | 29.69x | 0.01x | 0.05x | 0.04x |
| `atan 1e6` | 15.13 ns | 162.20 ns | 161.34 ns | 2.78 us | 1.44 us | 17.62 us | 10.72x | 0.06x | 0.11x | 0.01x |
| `asinh 1e6` | 26.39 ns | 211.73 ns | 211.15 ns | 36.99 us | 1.62 us | 7.13 us | 8.02x | 0.01x | 0.13x | 0.03x |
| `atan neg_1e6` | 15.32 ns | 269.18 ns | 266.95 ns | 2.82 us | 1.42 us | 17.97 us | 17.57x | 0.10x | 0.19x | 0.01x |
| `asinh neg_1e6` | 26.65 ns | 251.26 ns | 251.35 ns | 37.18 us | 1.63 us | 6.95 us | 9.43x | 0.01x | 0.15x | 0.04x |
| `acosh 9` | 12.30 ns | 144.54 ns | 144.64 ns | 41.94 us | 1.60 us | 9.83 us | 11.75x | 0.00x | 0.09x | 0.01x |
| `acosh 1_plus_1e-12` | 11.41 ns | 283.27 ns | 267.34 ns | 42.42 us | 8.64 us | 11.47 us | 24.83x | 0.01x | 0.03x | 0.02x |
| `acosh 1e6` | 12.23 ns | 144.70 ns | 144.98 ns | 37.60 us | 1.58 us | 9.81 us | 11.83x | 0.00x | 0.09x | 0.01x |
| `acosh e` | 12.27 ns | 186.11 ns | 2.27 us | 40.87 us | 1.63 us | 9.67 us | 15.16x | 0.00x | 0.11x | 0.02x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 18.49 ns | 18.73 ns | 25.05 ns | 15.73 ns | 0.94 ns | 38.22x | 0.74x | 1.18x | 19.58x |
| `one` | 0.48 ns | 24.39 ns | 23.80 ns | 43.13 ns | 31.19 ns | 30.92 ns | 50.65x | 0.57x | 0.78x | 0.79x |
| `e` | 0.48 ns | 89.33 ns | 99.77 ns | 54.06 ns | 1.07 us | 222.05 ns | 187.88x | 1.65x | 0.08x | 0.40x |
| `pi` | 0.48 ns | 51.04 ns | 57.88 ns | 42.55 ns | 48.76 ns | 220.35 ns | 105.78x | 1.20x | 1.05x | 0.23x |
| `tau` | 0.48 ns | 51.58 ns | 57.87 ns | 118.44 ns | 99.67 ns | 1.88 us | 106.56x | 0.44x | 0.52x | 0.03x |
| `add` | 5.30 ns | 224.29 ns | 333.30 ns | 54.25 ns | 42.14 ns | 1.31 us | 42.33x | 4.13x | 5.32x | 0.17x |
| `sub` | 5.32 ns | 264.76 ns | 511.80 ns | 60.48 ns | 44.98 ns | 2.46 us | 49.80x | 4.38x | 5.89x | 0.11x |
| `neg` | 5.18 ns | 39.13 ns | 57.96 ns | 23.54 ns | 21.53 ns | 1.07 us | 7.55x | 1.66x | 1.82x | 0.04x |
| `mul` | 5.53 ns | 147.03 ns | 423.80 ns | 58.45 ns | 46.63 ns | 1.54 us | 26.57x | 2.52x | 3.15x | 0.10x |
| `div` | 9.23 ns | 331.14 ns | 378.90 ns | 135.88 ns | 62.72 ns | 2.55 us | 35.87x | 2.44x | 5.28x | 0.13x |
| `reciprocal` | 8.30 ns | 75.12 ns | 81.07 ns | 163.60 ns | 58.59 ns | 1.54 us | 9.06x | 0.46x | 1.28x | 0.05x |
| `reciprocal checked` | 8.72 ns | 69.15 ns | 87.35 ns | 164.05 ns | 58.68 ns | 1.54 us | 7.93x | 0.42x | 1.18x | 0.04x |
| `reciprocal checked abort` | 22.06 ns | 82.26 ns | 78.60 ns | 164.90 ns | 58.83 ns | 1.52 us | 3.73x | 0.50x | 1.40x | 0.05x |
| `pow` | 20.46 ns | 6.75 us | 5.86 us | 61.44 us | 2.83 us | 2.41 us | 329.99x | 0.11x | 2.39x | 2.80x |
| `powi` | 6.21 ns | 374.77 ns | 2.73 us | 282.45 ns | 85.00 ns | 1.55 us | 60.37x | 1.33x | 4.41x | 0.24x |
| `exp` | 19.90 ns | 233.66 ns | 240.37 ns | 13.86 us | 927.71 ns | 1.93 us | 11.74x | 0.02x | 0.25x | 0.12x |
| `ln` | 10.79 ns | 1.42 us | 1.40 us | 29.12 us | 1.30 us | 1.81 us | 131.62x | 0.05x | 1.09x | 0.78x |
| `log10` | 13.60 ns | 1.62 us | 1.57 us | 34.92 us | 2.74 us | 6.69 us | 118.94x | 0.05x | 0.59x | 0.24x |
| `log10 abort` | 17.27 ns | 1.64 us | 1.59 us | 35.36 us | 2.75 us | 6.70 us | 94.86x | 0.05x | 0.60x | 0.24x |
| `sqrt` | 8.28 ns | 1.50 us | 1.65 us | 4.95 us | 98.97 ns | 1.48 us | 181.86x | 0.30x | 15.21x | 1.01x |
| `sin` | 15.18 ns | 124.54 ns | 135.34 ns | 14.33 us | 1.24 us | 2.26 us | 8.20x | 0.01x | 0.10x | 0.06x |
| `cos` | 18.46 ns | 129.57 ns | 135.34 ns | 12.49 us | 627.36 ns | 1.79 us | 7.02x | 0.01x | 0.21x | 0.07x |
| `tan` | 24.97 ns | 167.06 ns | 180.37 ns | 30.31 us | 1.56 us | 6.71 us | 6.69x | 0.01x | 0.11x | 0.02x |
| `sinh` | 18.32 ns | 3.67 us | 3.66 us | 3.43 us | 1.11 us | 10.90 us | 200.36x | 1.07x | 3.30x | 0.34x |
| `cosh` | 18.56 ns | 3.63 us | 3.61 us | 8.91 us | 1.06 us | 9.65 us | 195.84x | 0.41x | 3.43x | 0.38x |
| `tanh` | 20.55 ns | 6.10 us | 6.06 us | 5.87 us | 1.19 us | 22.97 us | 296.63x | 1.04x | 5.13x | 0.27x |
| `asin` | 13.18 ns | 372.78 ns | 599.97 ns | 36.20 us | 2.41 us | 13.87 us | 28.28x | 0.01x | 0.15x | 0.03x |
| `asin abort` | 17.29 ns | 385.30 ns | 615.73 ns | 53.36 us | 2.39 us | 13.68 us | 22.28x | 0.01x | 0.16x | 0.03x |
| `acos` | 13.88 ns | 367.67 ns | 769.75 ns | 27.72 us | 2.57 us | 13.75 us | 26.49x | 0.01x | 0.14x | 0.03x |
| `acos abort` | 17.89 ns | 367.26 ns | 770.60 ns | 26.55 us | 2.54 us | 13.93 us | 20.53x | 0.01x | 0.14x | 0.03x |
| `atan` | 15.13 ns | 168.45 ns | 169.86 ns | 18.61 us | 2.22 us | 19.07 us | 11.13x | 0.01x | 0.08x | 0.01x |
| `atan abort` | 21.45 ns | 167.70 ns | 167.09 ns | 18.41 us | 2.24 us | 19.02 us | 7.82x | 0.01x | 0.07x | 0.01x |
| `asinh` | 34.11 ns | 212.52 ns | 233.96 ns | 39.06 us | 1.61 us | 7.57 us | 6.23x | 0.01x | 0.13x | 0.03x |
| `asinh abort` | 32.76 ns | 212.91 ns | 221.65 ns | 39.11 us | 1.61 us | 7.71 us | 6.50x | 0.01x | 0.13x | 0.03x |
| `acosh` | 12.45 ns | 187.85 ns | 204.19 ns | 41.13 us | 3.37 us | 10.43 us | 15.09x | 0.00x | 0.06x | 0.02x |
| `acosh abort` | 15.59 ns | 190.66 ns | 190.15 ns | 40.57 us | 3.31 us | 10.49 us | 12.23x | 0.00x | 0.06x | 0.02x |
| `atanh` | 13.81 ns | 267.52 ns | 508.08 ns | 34.84 us | 1.25 us | 15.05 us | 19.37x | 0.01x | 0.21x | 0.02x |
| `atanh abort` | 17.35 ns | 267.22 ns | 488.14 ns | 34.48 us | 1.24 us | 14.94 us | 15.40x | 0.01x | 0.21x | 0.02x |
| `zero status` | 1.21 ns | 0.98 ns | 1.01 ns | 1.00 ns | 6.74 ns | 8.09 ns | 0.81x | 0.99x | 0.15x | 0.12x |
| `zero status abort` | 1.41 ns | 1.10 ns | 1.12 ns | 1.02 ns | 6.80 ns | 8.13 ns | 0.78x | 1.07x | 0.16x | 0.14x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.97 ns | 48.77 ns | 36.40 ns | 48.19 ns | - | 1.91 ns | 50.42x | 1.01x | - | 25.55x |
| `one` | 5.99 ns | 39.01 ns | 43.12 ns | 64.51 ns | - | 31.07 ns | 6.51x | 0.60x | - | 1.26x |
| `i` | 0.98 ns | 42.45 ns | 43.16 ns | 65.29 ns | - | 29.68 ns | 43.48x | 0.65x | - | 1.43x |
| `free i` | 0.94 ns | 42.92 ns | 57.51 ns | 65.43 ns | - | 29.69 ns | 45.64x | 0.66x | - | 1.45x |
| `conjugate` | 2.12 ns | 91.79 ns | 90.96 ns | 41.56 ns | - | 1.10 us | 43.25x | 2.21x | - | 0.08x |
| `norm squared` | 5.89 ns | 160.21 ns | 573.83 ns | 161.51 ns | - | 4.34 us | 27.20x | 0.99x | - | 0.04x |
| `reciprocal` | 17.16 ns | 1.73 us | 2.86 us | 782.55 ns | - | 10.84 us | 100.74x | 2.21x | - | 0.16x |
| `reciprocal checked` | 14.92 ns | 1.73 us | 2.93 us | 783.71 ns | - | 11.08 us | 116.12x | 2.21x | - | 0.16x |
| `powi` | 17.46 ns | 1.31 us | 6.71 us | 2.59 us | - | 44.70 us | 75.33x | 0.51x | - | 0.03x |
| `powi checked` | 17.60 ns | 1.37 us | 7.44 us | 1.44 us | - | 44.01 us | 77.78x | 0.95x | - | 0.03x |
| `div checked` | 16.52 ns | 2.40 us | 5.10 us | 789.20 ns | - | 22.11 us | 145.46x | 3.04x | - | 0.11x |
| `div real checked` | 17.46 ns | 681.78 ns | 990.17 ns | 268.61 ns | - | 5.23 us | 39.05x | 2.54x | - | 0.13x |
| `from scalar` | 1.41 ns | 56.07 ns | 83.09 ns | 45.49 ns | - | 10.32 ns | 39.83x | 1.23x | - | 5.43x |
| `add` | 6.06 ns | 480.58 ns | 875.42 ns | 106.59 ns | - | 2.55 us | 79.25x | 4.51x | - | 0.19x |
| `sub` | 6.16 ns | 530.53 ns | 1.17 us | 120.33 ns | - | 4.77 us | 86.15x | 4.41x | - | 0.11x |
| `neg` | 2.62 ns | 89.33 ns | 85.56 ns | 42.03 ns | - | 2.15 us | 34.14x | 2.13x | - | 0.04x |
| `mul` | 7.75 ns | 946.95 ns | 3.81 us | 319.56 ns | - | 10.12 us | 122.13x | 2.96x | - | 0.09x |
| `div` | 15.95 ns | 2.53 us | 4.80 us | 800.35 ns | - | 21.94 us | 158.39x | 3.16x | - | 0.12x |
| `div real` | 10.08 ns | 685.62 ns | 659.08 ns | 267.35 ns | - | 5.25 us | 68.04x | 2.56x | - | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 8.75 ns | 214.40 ns | 2.47 us | 312.68 ns | 249.36 ns | 7.15 us | 24.50x | 0.69x | 0.86x | 0.03x |
| `vec3 magnitude` | 11.31 ns | 3.98 us | 5.46 us | 5.49 us | 349.17 ns | 8.80 us | 351.97x | 0.72x | 11.40x | 0.45x |
| `vec3 normalize` | 24.37 ns | 8.37 us | 11.52 us | 6.07 us | 581.16 ns | 16.82 us | 343.52x | 1.38x | 14.40x | 0.50x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.07 ns | 284.65 ns | 1.53 us | 70.98 ns | 57.24 ns | 734.10 ns | 92.70x | 4.01x | 4.97x | 0.39x |
| `vec3 zero` | 1.41 ns | 132.04 ns | 131.37 ns | 65.17 ns | 35.75 ns | 2.85 ns | 93.44x | 2.03x | 3.69x | 46.26x |
| `vec3 dot abort` | 8.54 ns | 212.29 ns | 917.18 ns | 265.84 ns | 200.54 ns | 7.44 us | 24.86x | 0.80x | 1.06x | 0.03x |
| `vec3 magnitude abort` | 17.32 ns | 4.05 us | 3.28 us | 5.53 us | 328.65 ns | 8.87 us | 234.12x | 0.73x | 12.34x | 0.46x |
| `vec3 normalize checked` | 25.11 ns | 8.58 us | 7.35 us | 5.97 us | 610.07 ns | 17.17 us | 341.75x | 1.44x | 14.07x | 0.50x |
| `vec3 normalize checked abort` | 29.26 ns | 8.36 us | 7.36 us | 5.98 us | 539.99 ns | 17.00 us | 285.64x | 1.40x | 15.48x | 0.49x |
| `vec3 div scalar checked` | 9.68 ns | 1.34 us | 1.62 us | 402.89 ns | - | - | 138.25x | 3.32x | - | - |
| `vec3 div scalar checked abort` | 18.51 ns | 1.36 us | 1.62 us | 408.00 ns | - | - | 73.69x | 3.34x | - | - |
| `vec3 add` | 6.72 ns | 970.57 ns | 1.79 us | 150.24 ns | 125.38 ns | 3.93 us | 144.52x | 6.46x | 7.74x | 0.25x |
| `vec3 add scalar` | 6.62 ns | 785.46 ns | 1.64 us | 151.61 ns | 132.65 ns | 3.84 us | 118.67x | 5.18x | 5.92x | 0.20x |
| `vec3 sub` | 6.85 ns | 1.06 us | 2.58 us | 168.88 ns | 142.42 ns | 7.66 us | 154.88x | 6.28x | 7.45x | 0.14x |
| `vec3 sub scalar` | 6.45 ns | 943.56 ns | 2.03 us | 148.13 ns | 125.29 ns | 7.14 us | 146.22x | 6.37x | 7.53x | 0.13x |
| `vec3 neg` | 3.78 ns | 142.83 ns | 143.52 ns | 60.07 ns | 52.23 ns | 3.18 us | 37.75x | 2.38x | 2.73x | 0.04x |
| `vec3 mul scalar` | 7.00 ns | 529.02 ns | 3.77 us | 169.94 ns | 122.31 ns | 4.57 us | 75.55x | 3.11x | 4.33x | 0.12x |
| `vec3 div scalar` | 9.41 ns | 1.34 us | 1.60 us | 404.81 ns | 173.22 ns | 7.76 us | 142.39x | 3.31x | 7.74x | 0.17x |
| `vec4 dot` | 9.64 ns | 248.45 ns | 659.81 ns | 458.74 ns | 315.42 ns | 9.93 us | 25.76x | 0.54x | 0.79x | 0.03x |
| `vec4 magnitude` | 16.91 ns | 3.76 us | 2.59 us | 5.98 us | 423.45 ns | 11.69 us | 222.49x | 0.63x | 8.88x | 0.32x |
| `vec4 normalize` | 36.68 ns | 13.45 us | 6.46 us | 6.17 us | 721.99 ns | 22.07 us | 366.59x | 2.18x | 18.63x | 0.61x |
| `vec4 add` | 7.36 ns | 1.20 us | 1.88 us | 209.47 ns | 271.16 ns | 5.25 us | 162.48x | 5.71x | 4.41x | 0.23x |
| `vec4 add scalar` | 7.11 ns | 1.60 us | 1.82 us | 397.62 ns | 178.70 ns | 5.06 us | 225.56x | 4.04x | 8.98x | 0.32x |
| `vec4 sub` | 5.06 ns | 1.31 us | 2.32 us | 392.49 ns | 177.06 ns | 9.64 us | 259.97x | 3.35x | 7.43x | 0.14x |
| `vec4 sub scalar` | 4.58 ns | 1.13 us | 2.16 us | 361.17 ns | 176.19 ns | 9.47 us | 247.32x | 3.14x | 6.43x | 0.12x |
| `vec4 neg` | 4.94 ns | 192.57 ns | 224.54 ns | 122.38 ns | 106.50 ns | 4.19 us | 38.95x | 1.57x | 1.81x | 0.05x |
| `vec4 mul scalar` | 7.38 ns | 703.93 ns | 2.17 us | 217.31 ns | 219.41 ns | 5.70 us | 95.39x | 3.24x | 3.21x | 0.12x |
| `vec4 div scalar` | 13.55 ns | 1.73 us | 2.13 us | 532.78 ns | 333.46 ns | 10.23 us | 127.94x | 3.25x | 5.20x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 19.67 ns | 704.32 ns | 2.09 us | 965.33 ns | 843.78 ns | 23.02 us | 35.81x | 0.73x | 0.83x | 0.03x |
| `mat3 inverse` | 86.55 ns | 14.84 us | 8.12 us | 3.13 us | 2.44 us | 82.28 us | 171.45x | 4.75x | 6.07x | 0.18x |
| `mat3 mul mat3` | 67.72 ns | 3.36 us | 7.81 us | 2.88 us | 2.51 us | 61.28 us | 49.58x | 1.17x | 1.34x | 0.05x |
| `mat3 transform vec3` | 28.45 ns | 1.57 us | 5.09 us | 1.04 us | 884.74 ns | 20.48 us | 55.35x | 1.51x | 1.78x | 0.08x |
| `mat4 determinant` | 42.99 ns | 2.15 us | 1.50 us | 4.75 us | 6.99 us | 100.16 us | 50.02x | 0.45x | 0.31x | 0.02x |
| `mat4 inverse` | 163.54 ns | 27.16 us | 8.45 us | 12.01 us | 9.62 us | 339.69 us | 166.10x | 2.26x | 2.82x | 0.08x |
| `mat4 mul mat4` | 142.76 ns | 5.88 us | 4.85 us | 6.35 us | 5.48 us | 141.66 us | 41.16x | 0.93x | 1.07x | 0.04x |
| `mat4 transform vec4` | 46.33 ns | 2.60 us | 2.41 us | 1.91 us | 1.69 us | 36.11 us | 56.18x | 1.36x | 1.54x | 0.07x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 36.65 ns | 1.37 us | 4.41 us | 212.81 ns | 238.07 ns | 2.08 us | 37.25x | 6.41x | 5.73x | 0.66x |
| `mat3 zero` | 19.30 ns | 563.12 ns | 553.78 ns | 267.33 ns | 205.72 ns | 11.94 ns | 29.17x | 2.11x | 2.74x | 47.17x |
| `mat3 identity` | 11.78 ns | 629.36 ns | 629.26 ns | 323.61 ns | 237.51 ns | 141.01 ns | 53.41x | 1.94x | 2.65x | 4.46x |
| `mat3 transpose` | 9.82 ns | 672.04 ns | 667.99 ns | 239.38 ns | 204.20 ns | 118.67 ns | 68.44x | 2.81x | 3.29x | 5.66x |
| `mat3 reciprocal` | 145.09 ns | 14.55 us | 24.58 us | 2.92 us | 2.29 us | 83.21 us | 100.26x | 4.98x | 6.34x | 0.17x |
| `mat3 reciprocal checked` | 112.54 ns | 14.66 us | 24.38 us | 2.97 us | 2.32 us | 82.17 us | 130.22x | 4.94x | 6.32x | 0.18x |
| `mat3 inverse checked` | 113.63 ns | 14.71 us | 24.50 us | 2.93 us | 2.32 us | 82.90 us | 129.44x | 5.02x | 6.33x | 0.18x |
| `mat3 inverse checked abort` | 103.32 ns | 14.73 us | 24.33 us | 2.94 us | 2.28 us | 82.50 us | 142.52x | 5.00x | 6.46x | 0.18x |
| `mat3 powi` | 110.29 ns | 4.42 us | 39.69 us | 12.85 us | 6.41 us | 150.37 us | 40.03x | 0.34x | 0.69x | 0.03x |
| `mat3 powi checked` | 112.68 ns | 4.51 us | 39.76 us | 6.77 us | 6.44 us | 149.01 us | 40.01x | 0.67x | 0.70x | 0.03x |
| `mat3 powi checked abort` | 114.40 ns | 4.41 us | 40.69 us | 6.79 us | 6.39 us | 150.88 us | 38.53x | 0.65x | 0.69x | 0.03x |
| `mat3 div scalar checked` | 26.68 ns | 4.42 us | 5.69 us | 1.46 us | 802.73 ns | 21.92 us | 165.49x | 3.02x | 5.50x | 0.20x |
| `mat3 div scalar checked abort` | 32.66 ns | 4.50 us | 5.68 us | 1.47 us | 801.51 ns | 21.94 us | 137.88x | 3.06x | 5.62x | 0.21x |
| `mat3 div matrix checked` | 136.17 ns | 18.15 us | 43.31 us | 5.42 us | 4.39 us | 158.29 us | 133.28x | 3.35x | 4.13x | 0.11x |
| `mat3 div matrix checked abort` | 149.46 ns | 18.31 us | 43.33 us | 5.36 us | 4.40 us | 159.48 us | 122.49x | 3.42x | 4.16x | 0.11x |
| `mat3 add` | 19.24 ns | 3.19 us | 5.51 us | 505.25 ns | 484.96 ns | 11.39 us | 165.57x | 6.30x | 6.57x | 0.28x |
| `mat3 add scalar` | 17.19 ns | 2.47 us | 4.71 us | 829.17 ns | 697.68 ns | 11.95 us | 143.60x | 2.98x | 3.54x | 0.21x |
| `mat3 sub` | 26.39 ns | 3.46 us | 7.39 us | 542.63 ns | 510.00 ns | 21.22 us | 131.00x | 6.37x | 6.78x | 0.16x |
| `mat3 sub scalar` | 16.87 ns | 2.77 us | 12.35 us | 793.24 ns | 696.94 ns | 21.48 us | 163.93x | 3.49x | 3.97x | 0.13x |
| `mat3 neg` | 10.90 ns | 601.25 ns | 902.06 ns | 477.28 ns | 452.82 ns | 8.66 us | 55.18x | 1.26x | 1.33x | 0.07x |
| `mat3 mul scalar` | 18.40 ns | 1.77 us | 10.58 us | 794.93 ns | 659.56 ns | 12.11 us | 96.10x | 2.22x | 2.68x | 0.15x |
| `mat3 div scalar` | 25.52 ns | 4.38 us | 10.37 us | 1.48 us | 811.96 ns | 22.00 us | 171.44x | 2.95x | 5.39x | 0.20x |
| `mat3 div matrix` | 133.02 ns | 18.54 us | 43.36 us | 5.34 us | 4.45 us | 158.96 us | 139.40x | 3.47x | 4.17x | 0.12x |
| `mat3 bitxor` | 108.56 ns | 4.42 us | 39.90 us | 6.87 us | 6.40 us | 150.45 us | 40.73x | 0.64x | 0.69x | 0.03x |
| `mat4 zero` | 19.06 ns | 908.78 ns | 790.89 ns | 457.41 ns | 356.55 ns | 14.77 ns | 47.67x | 1.99x | 2.55x | 61.54x |
| `mat4 identity` | 10.55 ns | 1.11 us | 980.52 ns | 569.70 ns | 420.78 ns | 215.25 ns | 104.90x | 1.94x | 2.63x | 5.14x |
| `mat4 transpose` | 10.22 ns | 1.09 us | 1.16 us | 427.58 ns | 384.22 ns | 192.00 ns | 106.28x | 2.54x | 2.83x | 5.65x |
| `mat4 reciprocal` | 168.09 ns | 24.56 us | 40.47 us | 10.88 us | 8.91 us | 344.08 us | 146.13x | 2.26x | 2.76x | 0.07x |
| `mat4 reciprocal checked` | 180.39 ns | 24.77 us | 40.97 us | 10.61 us | 8.89 us | 342.00 us | 137.29x | 2.33x | 2.79x | 0.07x |
| `mat4 powi` | 248.37 ns | 8.72 us | 53.25 us | 16.09 us | 13.95 us | 348.51 us | 35.13x | 0.54x | 0.63x | 0.03x |
| `mat4 powi checked` | 249.82 ns | 8.68 us | 53.40 us | 15.73 us | 13.97 us | 347.95 us | 34.76x | 0.55x | 0.62x | 0.02x |
| `mat4 add` | 52.73 ns | 5.15 us | 6.25 us | 913.97 ns | 848.45 ns | 19.49 us | 97.66x | 5.63x | 6.07x | 0.26x |
| `mat4 add scalar` | 20.67 ns | 4.12 us | 6.85 us | 1.35 us | 1.17 us | 20.41 us | 199.52x | 3.05x | 3.53x | 0.20x |
| `mat4 sub` | 38.37 ns | 5.43 us | 7.41 us | 973.71 ns | 897.66 ns | 35.97 us | 141.52x | 5.58x | 6.05x | 0.15x |
| `mat4 sub scalar` | 15.00 ns | 4.41 us | 7.81 us | 1.40 us | 1.17 us | 37.58 us | 293.70x | 3.15x | 3.78x | 0.12x |
| `mat4 neg` | 13.68 ns | 978.58 ns | 1.10 us | 948.58 ns | 732.09 ns | 14.26 us | 71.54x | 1.03x | 1.34x | 0.07x |
| `mat4 mul scalar` | 23.85 ns | 2.90 us | 7.56 us | 1.37 us | 1.11 us | 20.48 us | 121.77x | 2.11x | 2.61x | 0.14x |
| `mat4 div scalar` | 31.04 ns | 7.52 us | 7.81 us | 2.62 us | 1.36 us | 37.72 us | 242.31x | 2.87x | 5.54x | 0.20x |
| `mat4 div matrix` | 193.90 ns | 28.13 us | 61.68 us | 16.99 us | 14.21 us | 535.30 us | 145.07x | 1.66x | 1.98x | 0.05x |
| `mat4 bitxor` | 248.99 ns | 8.78 us | 53.14 us | 16.03 us | 13.96 us | 348.67 us | 35.27x | 0.55x | 0.63x | 0.03x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.66 ns | 182.19 ns | 305.16 ns | - | - | 2.28 us | 49.83x | - | - | 0.08x |
| `scalar add ref_owned` | 12.24 ns | 181.31 ns | 343.50 ns | - | - | 1.32 us | 14.81x | - | - | 0.14x |
| `scalar add refs` | 5.40 ns | 179.87 ns | 274.89 ns | - | - | 1.32 us | 33.28x | - | - | 0.14x |
| `scalar add owned_ref_with_clone` | 8.93 ns | 199.39 ns | 308.10 ns | - | - | - | 22.32x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.52 ns | 196.59 ns | 303.71 ns | - | - | - | 35.59x | - | - | - |
| `scalar sub owned_ref` | 3.89 ns | 224.40 ns | 473.40 ns | - | - | 2.50 us | 57.62x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.33 ns | 232.66 ns | 848.14 ns | - | - | 2.55 us | 18.86x | - | - | 0.09x |
| `scalar sub refs` | 5.65 ns | 211.90 ns | 549.49 ns | - | - | 2.51 us | 37.47x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.00 ns | 237.58 ns | 520.54 ns | - | - | - | 26.40x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.73 ns | 239.26 ns | 503.76 ns | - | - | - | 41.75x | - | - | - |
| `scalar mul owned_ref` | 4.25 ns | 102.02 ns | 392.40 ns | - | - | 1.58 us | 23.99x | - | - | 0.06x |
| `scalar mul ref_owned` | 13.32 ns | 95.75 ns | 387.88 ns | - | - | 1.57 us | 7.19x | - | - | 0.06x |
| `scalar mul refs` | 6.42 ns | 86.14 ns | 358.92 ns | - | - | 1.56 us | 13.41x | - | - | 0.06x |
| `scalar mul owned_ref_with_clone` | 9.72 ns | 112.91 ns | 414.78 ns | - | - | - | 11.62x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.09 ns | 108.53 ns | 421.55 ns | - | - | - | 17.82x | - | - | - |
| `scalar div owned_ref` | 5.88 ns | 278.28 ns | 337.32 ns | - | - | 4.45 us | 47.32x | - | - | 0.06x |
| `scalar div ref_owned` | 17.32 ns | 270.17 ns | 328.37 ns | - | - | 2.82 us | 15.60x | - | - | 0.10x |
| `scalar div refs` | 6.93 ns | 272.26 ns | 318.91 ns | - | - | 2.56 us | 39.27x | - | - | 0.11x |
| `scalar div owned_ref_with_clone` | 13.32 ns | 317.44 ns | 424.35 ns | - | - | - | 23.84x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.31 ns | 292.04 ns | 351.85 ns | - | - | - | 19.08x | - | - | - |
| `vec3 add refs` | 6.32 ns | 675.02 ns | 1.47 us | - | - | 3.99 us | 106.87x | - | - | 0.17x |
| `vec3 sub refs` | 6.34 ns | 751.06 ns | 2.31 us | - | - | 7.55 us | 118.51x | - | - | 0.10x |
| `vec3 neg ref` | 3.30 ns | 182.00 ns | 160.92 ns | - | - | 3.20 us | 55.13x | - | - | 0.06x |
| `vec3 add_scalar_ref` | 6.57 ns | 786.54 ns | 1.49 us | - | - | 3.88 us | 119.71x | - | - | 0.20x |
| `vec3 sub_scalar_ref` | 6.65 ns | 775.08 ns | 1.28 us | - | - | 7.26 us | 116.57x | - | - | 0.11x |
| `vec3 mul_scalar_ref` | 6.96 ns | 520.30 ns | 2.00 us | - | - | 4.53 us | 74.79x | - | - | 0.11x |
| `vec3 div_scalar_ref` | 7.88 ns | 1.36 us | 1.59 us | - | - | 7.58 us | 172.98x | - | - | 0.18x |
| `vec4 add refs` | 6.43 ns | 792.60 ns | 1.50 us | - | - | 5.24 us | 123.21x | - | - | 0.15x |
| `vec4 sub refs` | 3.10 ns | 880.43 ns | 1.95 us | - | - | 9.63 us | 283.86x | - | - | 0.09x |
| `vec4 neg ref` | 4.25 ns | 232.27 ns | 219.25 ns | - | - | 4.08 us | 54.65x | - | - | 0.06x |
| `vec4 add_scalar_ref` | 6.98 ns | 1.04 us | 1.79 us | - | - | 5.59 us | 149.52x | - | - | 0.19x |
| `vec4 sub_scalar_ref` | 4.25 ns | 1.05 us | 1.50 us | - | - | 9.85 us | 247.46x | - | - | 0.11x |
| `vec4 mul_scalar_ref` | 7.47 ns | 674.79 ns | 2.15 us | - | - | 5.87 us | 90.35x | - | - | 0.12x |
| `vec4 div_scalar_ref` | 11.23 ns | 1.79 us | 1.50 us | - | - | 13.71 us | 159.41x | - | - | 0.13x |
| `mat3 add refs` | 11.16 ns | 1.76 us | 4.00 us | - | - | 11.63 us | 157.75x | - | - | 0.15x |
| `mat3 sub refs` | 10.60 ns | 2.02 us | 6.19 us | - | - | 21.43 us | 190.70x | - | - | 0.09x |
| `mat3 mul refs` | 52.62 ns | 2.25 us | 15.24 us | - | - | 62.13 us | 42.69x | - | - | 0.04x |
| `mat3 div refs` | 127.65 ns | 16.78 us | 41.61 us | - | - | 166.32 us | 131.42x | - | - | 0.10x |
| `mat3 neg ref` | 9.97 ns | 514.07 ns | 491.77 ns | - | - | 9.63 us | 51.56x | - | - | 0.05x |
| `mat3 add_scalar_ref` | 44.93 ns | 2.50 us | 4.56 us | - | - | 24.62 us | 55.75x | - | - | 0.10x |
| `mat3 sub_scalar_ref` | 12.00 ns | 2.47 us | 4.41 us | - | - | 22.87 us | 205.70x | - | - | 0.11x |
| `mat3 mul_scalar_ref` | 48.13 ns | 1.79 us | 6.02 us | - | - | 12.78 us | 37.27x | - | - | 0.14x |
| `mat3 div_scalar_ref` | 21.53 ns | 4.33 us | 5.48 us | - | - | 22.93 us | 201.08x | - | - | 0.19x |
| `mat4 add refs` | 17.43 ns | 2.95 us | 3.88 us | - | - | 20.10 us | 169.01x | - | - | 0.15x |
| `mat4 sub refs` | 16.96 ns | 3.22 us | 5.07 us | - | - | 36.34 us | 190.00x | - | - | 0.09x |
| `mat4 mul refs` | 132.21 ns | 4.40 us | 27.17 us | - | - | 142.13 us | 33.27x | - | - | 0.03x |
| `mat4 div refs` | 183.27 ns | 27.17 us | 61.36 us | - | - | 532.87 us | 148.27x | - | - | 0.05x |
| `mat4 neg ref` | 12.11 ns | 746.69 ns | 1.22 us | - | - | 14.63 us | 61.65x | - | - | 0.05x |
| `mat4 add_scalar_ref` | 49.06 ns | 4.23 us | 6.69 us | - | - | 37.76 us | 86.17x | - | - | 0.11x |
| `mat4 sub_scalar_ref` | 15.48 ns | 4.30 us | 6.69 us | - | - | 77.57 us | 277.59x | - | - | 0.06x |
| `mat4 mul_scalar_ref` | 52.60 ns | 2.99 us | 7.34 us | - | - | 23.26 us | 56.92x | - | - | 0.13x |
| `mat4 div_scalar_ref` | 25.63 ns | 7.45 us | 7.49 us | - | - | 39.34 us | 290.86x | - | - | 0.19x |
| `mat3 transform_vec refs` | 27.77 ns | 919.75 ns | 5.53 us | - | - | 20.94 us | 33.13x | - | - | 0.04x |
| `mat4 transform_vec refs` | 44.32 ns | 1.51 us | 6.12 us | - | - | 35.60 us | 33.99x | - | - | 0.04x |
| `complex add refs` | 7.66 ns | 332.08 ns | 696.93 ns | - | - | 2.58 us | 43.33x | - | - | 0.13x |
| `complex sub refs` | 8.00 ns | 380.12 ns | 965.56 ns | - | - | 4.87 us | 47.51x | - | - | 0.08x |
| `complex mul refs` | 8.09 ns | 766.69 ns | 3.32 us | - | - | 11.06 us | 94.73x | - | - | 0.07x |
| `complex div refs` | 16.14 ns | 2.21 us | 4.48 us | - | - | 22.23 us | 136.79x | - | - | 0.10x |
| `complex neg ref` | 2.37 ns | 94.04 ns | 73.64 ns | - | - | 2.21 us | 39.68x | - | - | 0.04x |
| `complex div_real_ref` | 7.32 ns | 674.72 ns | 627.21 ns | - | - | 5.26 us | 92.19x | - | - | 0.13x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.43 us |
| `astro sin 160` | 14.02 us |
| `astro sin 192` | 13.94 us |
| `astro sin 256` | 16.17 us |
