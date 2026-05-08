# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
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
| `sin 0.1` | 10.99 ns | 193.21 ns | 190.95 ns | 10.86 us | 778.50 ns | 1.85 us | 17.58x | 0.02x | 0.25x | 0.10x |
| `cos 0.1` | 11.92 ns | 197.79 ns | 196.57 ns | 10.65 us | 496.90 ns | 1.69 us | 16.60x | 0.02x | 0.40x | 0.12x |
| `sin 1.23456789` | 11.77 ns | 253.38 ns | 233.66 ns | 12.55 us | 821.14 ns | 1.84 us | 21.52x | 0.02x | 0.31x | 0.14x |
| `cos 1.23456789` | 12.02 ns | 241.48 ns | 226.53 ns | 10.59 us | 615.90 ns | 1.66 us | 20.09x | 0.02x | 0.39x | 0.15x |
| `sin 1e6` | 13.10 ns | 6.02 us | 6.24 us | 16.01 us | 1.11 us | 2.07 us | 459.47x | 0.38x | 5.41x | 2.91x |
| `cos 1e6` | 12.66 ns | 6.12 us | 6.18 us | 13.78 us | 848.10 ns | 1.84 us | 483.02x | 0.44x | 7.21x | 3.33x |
| `sin 1e30` | 67.54 ns | 8.16 us | 8.23 us | 18.75 us | 2.90 us | 3.68 us | 120.87x | 0.44x | 2.82x | 2.22x |
| `cos 1e30` | 69.60 ns | 8.37 us | 8.49 us | 15.66 us | 975.12 ns | 3.15 us | 120.24x | 0.53x | 8.58x | 2.66x |
| `sin pi_7` | 11.95 ns | 194.20 ns | 548.21 ns | 12.26 us | 774.84 ns | 1.93 us | 16.25x | 0.02x | 0.25x | 0.10x |
| `cos pi_7` | 11.88 ns | 199.56 ns | 1.01 us | 10.89 us | 550.59 ns | 1.76 us | 16.80x | 0.02x | 0.36x | 0.11x |
| `sin 1000pi_eps` | 12.03 ns | 6.03 us | 4.36 us | 15.84 us | 2.29 us | 2.81 us | 501.03x | 0.38x | 2.63x | 2.15x |
| `cos 1000pi_eps` | 12.95 ns | 6.07 us | 4.35 us | 13.60 us | 582.15 ns | 1.66 us | 468.57x | 0.45x | 10.42x | 3.65x |
| `asin 0.5` | 11.31 ns | 510.94 ns | 509.44 ns | 50.14 us | 2.95 us | 13.24 us | 45.17x | 0.01x | 0.17x | 0.04x |
| `acos 0.5` | 11.42 ns | 1.09 us | 1.10 us | 59.28 us | 3.03 us | 13.32 us | 95.66x | 0.02x | 0.36x | 0.08x |
| `atanh 0.5` | 14.67 ns | 1.45 us | 1.45 us | 35.39 us | 1.65 us | 13.38 us | 99.07x | 0.04x | 0.88x | 0.11x |
| `asin neg_0.999999` | 14.29 ns | 5.71 us | 4.60 us | 14.33 us | 2.54 us | 13.24 us | 399.38x | 0.40x | 2.24x | 0.43x |
| `acos neg_0.999999` | 15.64 ns | 5.80 us | 5.16 us | 19.45 us | 2.72 us | 13.23 us | 370.60x | 0.30x | 2.13x | 0.44x |
| `atanh neg_0.999999` | 14.81 ns | 4.13 us | 3.64 us | 37.78 us | 1.66 us | 12.66 us | 278.63x | 0.11x | 2.48x | 0.33x |
| `asin 0.999999` | 14.37 ns | 5.59 us | 4.91 us | 14.51 us | 2.54 us | 12.87 us | 389.13x | 0.39x | 2.20x | 0.43x |
| `acos 0.999999` | 14.59 ns | 5.51 us | 5.07 us | 18.66 us | 2.73 us | 13.21 us | 377.98x | 0.30x | 2.02x | 0.42x |
| `atanh 0.999999` | 14.59 ns | 4.15 us | 3.54 us | 32.12 us | 1.59 us | 12.71 us | 284.70x | 0.13x | 2.61x | 0.33x |
| `asin 1e-12` | 9.72 ns | 510.24 ns | 1.12 us | 8.14 us | 1.42 us | 15.87 us | 52.48x | 0.06x | 0.36x | 0.03x |
| `acos 1e-12` | 10.19 ns | 862.51 ns | 1.87 us | 10.01 us | 1.45 us | 15.88 us | 84.64x | 0.09x | 0.59x | 0.05x |
| `atanh 1e-12` | 9.90 ns | 515.20 ns | 941.70 ns | 36.97 us | 172.33 ns | 20.55 us | 52.05x | 0.01x | 2.99x | 0.03x |
| `atan 0.5` | 15.42 ns | 462.16 ns | 453.20 ns | 36.95 us | 2.84 us | 18.02 us | 29.97x | 0.01x | 0.16x | 0.03x |
| `asinh 0.5` | 27.61 ns | 1.64 us | 1.65 us | 39.80 us | 1.62 us | 7.60 us | 59.35x | 0.04x | 1.01x | 0.22x |
| `atan neg_1e-12` | 14.83 ns | 433.79 ns | 618.21 ns | 1.63 us | 1.13 us | 15.20 us | 29.26x | 0.27x | 0.38x | 0.03x |
| `asinh neg_1e-12` | 16.14 ns | 5.49 us | 4.48 us | 42.77 us | 8.65 us | 11.77 us | 339.85x | 0.13x | 0.63x | 0.47x |
| `atan 1e6` | 15.76 ns | 397.14 ns | 396.20 ns | 2.88 us | 1.45 us | 18.13 us | 25.21x | 0.14x | 0.27x | 0.02x |
| `asinh 1e6` | 27.38 ns | 3.34 us | 3.39 us | 37.52 us | 1.63 us | 7.26 us | 121.91x | 0.09x | 2.05x | 0.46x |
| `atan neg_1e6` | 15.83 ns | 501.05 ns | 498.95 ns | 2.84 us | 1.44 us | 17.56 us | 31.65x | 0.18x | 0.35x | 0.03x |
| `asinh neg_1e6` | 27.18 ns | 3.55 us | 3.41 us | 36.95 us | 1.66 us | 7.20 us | 130.70x | 0.10x | 2.14x | 0.49x |
| `acosh 9` | 12.80 ns | 2.92 us | 2.98 us | 42.47 us | 1.63 us | 9.88 us | 228.13x | 0.07x | 1.79x | 0.30x |
| `acosh 1_plus_1e-12` | 11.60 ns | 3.75 us | 5.20 us | 42.60 us | 8.69 us | 11.40 us | 323.55x | 0.09x | 0.43x | 0.33x |
| `acosh 1e6` | 12.71 ns | 3.73 us | 3.65 us | 37.73 us | 1.59 us | 10.08 us | 293.09x | 0.10x | 2.34x | 0.37x |
| `acosh e` | 12.60 ns | 4.13 us | 4.19 us | 41.63 us | 1.63 us | 9.89 us | 327.99x | 0.10x | 2.53x | 0.42x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.49 ns | 33.08 ns | 32.86 ns | 26.04 ns | 15.75 ns | 0.96 ns | 67.33x | 1.27x | 2.10x | 34.40x |
| `one` | 0.49 ns | 40.68 ns | 39.36 ns | 45.74 ns | 30.99 ns | 30.18 ns | 83.19x | 0.89x | 1.31x | 1.35x |
| `e` | 0.48 ns | 73.20 ns | 68.64 ns | 55.90 ns | 1.06 us | 227.61 ns | 152.24x | 1.31x | 0.07x | 0.32x |
| `pi` | 0.48 ns | 54.16 ns | 52.71 ns | 49.28 ns | 49.22 ns | 229.45 ns | 112.01x | 1.10x | 1.10x | 0.24x |
| `tau` | 0.48 ns | 192.70 ns | 187.60 ns | 117.44 ns | 101.54 ns | 1.86 us | 402.84x | 1.64x | 1.90x | 0.10x |
| `add` | 5.44 ns | 273.29 ns | 366.53 ns | 53.01 ns | 43.60 ns | 1.30 us | 50.24x | 5.16x | 6.27x | 0.21x |
| `sub` | 5.34 ns | 273.32 ns | 418.55 ns | 56.68 ns | 45.80 ns | 2.43 us | 51.18x | 4.82x | 5.97x | 0.11x |
| `neg` | 5.20 ns | 54.70 ns | 51.95 ns | 23.41 ns | 22.16 ns | 1.08 us | 10.53x | 2.34x | 2.47x | 0.05x |
| `mul` | 5.54 ns | 187.39 ns | 446.36 ns | 57.45 ns | 46.42 ns | 1.51 us | 33.82x | 3.26x | 4.04x | 0.12x |
| `div` | 9.00 ns | 367.81 ns | 409.55 ns | 140.48 ns | 64.82 ns | 2.62 us | 40.88x | 2.62x | 5.67x | 0.14x |
| `reciprocal` | 8.92 ns | 109.69 ns | 105.82 ns | 166.94 ns | 59.77 ns | 1.53 us | 12.30x | 0.66x | 1.84x | 0.07x |
| `reciprocal checked` | 9.16 ns | 111.36 ns | 108.95 ns | 162.28 ns | 59.15 ns | 1.54 us | 12.16x | 0.69x | 1.88x | 0.07x |
| `reciprocal checked abort` | 24.25 ns | 111.72 ns | 113.58 ns | 161.94 ns | 59.98 ns | 1.52 us | 4.61x | 0.69x | 1.86x | 0.07x |
| `pow` | 30.06 ns | 10.51 us | 10.74 us | 54.84 us | 2.91 us | 2.34 us | 349.71x | 0.19x | 3.62x | 4.50x |
| `powi` | 6.22 ns | 533.05 ns | 2.80 us | 284.14 ns | 85.62 ns | 1.54 us | 85.73x | 1.88x | 6.23x | 0.35x |
| `exp` | 19.85 ns | 1.83 us | 3.06 us | 14.01 us | 931.12 ns | 1.94 us | 91.96x | 0.13x | 1.96x | 0.94x |
| `ln` | 10.86 ns | 1.53 us | 1.53 us | 29.57 us | 1.34 us | 1.85 us | 140.51x | 0.05x | 1.14x | 0.82x |
| `log10` | 13.48 ns | 2.71 us | 2.32 us | 35.37 us | 2.75 us | 6.75 us | 201.01x | 0.08x | 0.98x | 0.40x |
| `log10 abort` | 17.16 ns | 2.70 us | 2.33 us | 35.46 us | 2.75 us | 6.63 us | 157.35x | 0.08x | 0.98x | 0.41x |
| `sqrt` | 8.31 ns | 1.58 us | 1.68 us | 5.09 us | 95.41 ns | 1.49 us | 189.80x | 0.31x | 16.54x | 1.06x |
| `sin` | 15.24 ns | 3.16 us | 3.69 us | 13.78 us | 1.30 us | 2.21 us | 207.42x | 0.23x | 2.44x | 1.43x |
| `cos` | 18.39 ns | 3.18 us | 3.64 us | 11.91 us | 628.48 ns | 1.75 us | 173.17x | 0.27x | 5.07x | 1.82x |
| `tan` | 24.68 ns | 2.90 us | 3.34 us | 29.75 us | 1.57 us | 6.66 us | 117.61x | 0.10x | 1.85x | 0.44x |
| `sinh` | 18.72 ns | 4.33 us | 4.74 us | 3.33 us | 1.14 us | 10.70 us | 231.34x | 1.30x | 3.81x | 0.40x |
| `cosh` | 18.25 ns | 4.31 us | 4.56 us | 8.07 us | 1.06 us | 9.72 us | 236.32x | 0.53x | 4.07x | 0.44x |
| `tanh` | 25.03 ns | 5.84 us | 6.14 us | 3.40 us | 1.20 us | 23.16 us | 233.20x | 1.72x | 4.87x | 0.25x |
| `asin` | 13.11 ns | 3.07 us | 4.57 us | 21.87 us | 2.42 us | 14.08 us | 234.55x | 0.14x | 1.27x | 0.22x |
| `asin abort` | 17.31 ns | 3.13 us | 4.50 us | 21.50 us | 2.42 us | 14.10 us | 180.69x | 0.15x | 1.29x | 0.22x |
| `acos` | 14.70 ns | 3.49 us | 5.23 us | 26.94 us | 2.54 us | 14.09 us | 237.38x | 0.13x | 1.37x | 0.25x |
| `acos abort` | 17.84 ns | 3.40 us | 5.23 us | 27.26 us | 2.58 us | 14.28 us | 190.36x | 0.12x | 1.32x | 0.24x |
| `atan` | 15.33 ns | 478.39 ns | 1.13 us | 18.59 us | 2.27 us | 18.80 us | 31.20x | 0.03x | 0.21x | 0.03x |
| `atan abort` | 22.48 ns | 445.50 ns | 1.10 us | 18.85 us | 2.29 us | 18.82 us | 19.82x | 0.02x | 0.19x | 0.02x |
| `asinh` | 35.20 ns | 3.69 us | 5.47 us | 39.58 us | 1.66 us | 7.44 us | 104.69x | 0.09x | 2.22x | 0.50x |
| `asinh abort` | 31.61 ns | 3.82 us | 5.45 us | 38.84 us | 1.64 us | 7.44 us | 120.76x | 0.10x | 2.33x | 0.51x |
| `acosh` | 12.54 ns | 3.68 us | 5.39 us | 41.33 us | 3.35 us | 10.34 us | 293.62x | 0.09x | 1.10x | 0.36x |
| `acosh abort` | 15.57 ns | 3.63 us | 5.13 us | 41.23 us | 3.38 us | 10.34 us | 232.95x | 0.09x | 1.07x | 0.35x |
| `atanh` | 13.78 ns | 2.57 us | 3.20 us | 35.34 us | 1.27 us | 15.07 us | 186.63x | 0.07x | 2.02x | 0.17x |
| `atanh abort` | 17.16 ns | 2.56 us | 3.24 us | 35.01 us | 1.27 us | 14.95 us | 149.02x | 0.07x | 2.01x | 0.17x |
| `zero status` | 1.22 ns | 2.56 ns | 1.91 ns | 1.08 ns | 6.76 ns | 8.07 ns | 2.10x | 2.37x | 0.38x | 0.32x |
| `zero status abort` | 1.46 ns | 3.07 ns | 3.17 ns | 1.06 ns | 6.82 ns | 8.03 ns | 2.10x | 2.89x | 0.45x | 0.38x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.95 ns | 64.96 ns | 65.66 ns | 48.46 ns | - | 1.89 ns | 68.59x | 1.34x | - | 34.39x |
| `one` | 5.53 ns | 69.32 ns | 69.33 ns | 65.67 ns | - | 30.73 ns | 12.53x | 1.06x | - | 2.26x |
| `i` | 0.96 ns | 72.85 ns | 71.48 ns | 64.92 ns | - | 30.59 ns | 75.67x | 1.12x | - | 2.38x |
| `free i` | 0.95 ns | 73.77 ns | 71.70 ns | 66.37 ns | - | 30.00 ns | 77.90x | 1.11x | - | 2.46x |
| `conjugate` | 2.14 ns | 122.28 ns | 118.48 ns | 42.72 ns | - | 1.12 us | 57.23x | 2.86x | - | 0.11x |
| `norm squared` | 5.91 ns | 389.42 ns | 1.45 us | 154.75 ns | - | 4.47 us | 65.93x | 2.52x | - | 0.09x |
| `reciprocal` | 15.32 ns | 1.86 us | 2.94 us | 451.74 ns | - | 10.99 us | 121.63x | 4.13x | - | 0.17x |
| `reciprocal checked` | 14.57 ns | 1.87 us | 2.92 us | 451.84 ns | - | 10.80 us | 128.68x | 4.15x | - | 0.17x |
| `powi` | 17.59 ns | 2.49 us | 14.02 us | 1.53 us | - | 43.51 us | 141.41x | 1.62x | - | 0.06x |
| `powi checked` | 17.90 ns | 2.50 us | 13.85 us | 1.53 us | - | 43.03 us | 139.44x | 1.63x | - | 0.06x |
| `div checked` | 18.88 ns | 2.90 us | 6.58 us | 786.50 ns | - | 22.03 us | 153.43x | 3.68x | - | 0.13x |
| `div real checked` | 9.55 ns | 720.40 ns | 706.46 ns | 277.26 ns | - | 5.21 us | 75.40x | 2.60x | - | 0.14x |
| `from scalar` | 1.42 ns | 86.96 ns | 86.40 ns | 48.69 ns | - | 11.86 ns | 61.22x | 1.79x | - | 7.33x |
| `add` | 6.17 ns | 589.48 ns | 956.53 ns | 113.86 ns | - | 2.58 us | 95.60x | 5.18x | - | 0.23x |
| `sub` | 6.25 ns | 585.87 ns | 961.95 ns | 121.37 ns | - | 4.76 us | 93.78x | 4.83x | - | 0.12x |
| `neg` | 2.64 ns | 117.18 ns | 115.71 ns | 42.98 ns | - | 2.11 us | 44.40x | 2.73x | - | 0.06x |
| `mul` | 11.43 ns | 1.03 us | 3.30 us | 304.19 ns | - | 9.92 us | 89.68x | 3.37x | - | 0.10x |
| `div` | 18.42 ns | 3.01 us | 6.40 us | 808.98 ns | - | 21.48 us | 163.34x | 3.72x | - | 0.14x |
| `div real` | 10.21 ns | 738.35 ns | 700.16 ns | 271.73 ns | - | 5.15 us | 72.34x | 2.72x | - | 0.14x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.85 ns | 790.45 ns | 5.74 us | 311.36 ns | 276.03 ns | 7.21 us | 115.39x | 2.54x | 2.86x | 0.11x |
| `vec3 magnitude` | 9.56 ns | 4.71 us | 8.73 us | 5.61 us | 376.63 ns | 9.00 us | 492.85x | 0.84x | 12.50x | 0.52x |
| `vec3 normalize` | 25.80 ns | 8.80 us | 14.57 us | 6.04 us | 617.87 ns | 16.70 us | 341.23x | 1.46x | 14.25x | 0.53x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.13 ns | 278.06 ns | 1.51 us | 71.70 ns | 56.15 ns | 732.61 ns | 88.84x | 3.88x | 4.95x | 0.38x |
| `vec3 zero` | 1.41 ns | 176.49 ns | 176.32 ns | 64.64 ns | 30.83 ns | 2.83 ns | 124.94x | 2.73x | 5.72x | 62.44x |
| `vec3 dot abort` | 28.13 ns | 1.21 us | 4.34 us | 264.19 ns | 200.46 ns | 7.11 us | 43.18x | 4.60x | 6.06x | 0.17x |
| `vec3 magnitude abort` | 39.54 ns | 5.08 us | 6.69 us | 5.63 us | 324.14 ns | 8.79 us | 128.55x | 0.90x | 15.68x | 0.58x |
| `vec3 normalize checked` | 25.86 ns | 8.71 us | 10.17 us | 6.08 us | 548.41 ns | 17.20 us | 336.88x | 1.43x | 15.89x | 0.51x |
| `vec3 normalize checked abort` | 56.92 ns | 9.19 us | 10.68 us | 6.10 us | 554.06 ns | 17.24 us | 161.45x | 1.51x | 16.59x | 0.53x |
| `vec3 div scalar checked` | 14.31 ns | 1.47 us | 1.69 us | 408.95 ns | - | - | 102.52x | 3.59x | - | - |
| `vec3 div scalar checked abort` | 18.66 ns | 1.43 us | 1.67 us | 409.36 ns | - | - | 76.35x | 3.48x | - | - |
| `vec3 add` | 6.83 ns | 1.14 us | 1.95 us | 153.01 ns | 128.52 ns | 4.14 us | 166.19x | 7.42x | 8.83x | 0.27x |
| `vec3 add scalar` | 6.53 ns | 907.46 ns | 1.60 us | 156.81 ns | 135.20 ns | 3.85 us | 138.92x | 5.79x | 6.71x | 0.24x |
| `vec3 sub` | 6.74 ns | 1.17 us | 1.99 us | 169.09 ns | 136.90 ns | 7.56 us | 173.65x | 6.92x | 8.55x | 0.15x |
| `vec3 sub scalar` | 6.44 ns | 880.73 ns | 1.39 us | 149.64 ns | 126.16 ns | 7.13 us | 136.69x | 5.89x | 6.98x | 0.12x |
| `vec3 neg` | 3.86 ns | 203.55 ns | 203.27 ns | 58.32 ns | 51.33 ns | 3.21 us | 52.68x | 3.49x | 3.97x | 0.06x |
| `vec3 mul scalar` | 7.10 ns | 635.81 ns | 2.15 us | 170.56 ns | 123.82 ns | 4.38 us | 89.58x | 3.73x | 5.13x | 0.15x |
| `vec3 div scalar` | 10.25 ns | 1.46 us | 1.71 us | 406.95 ns | 177.52 ns | 7.73 us | 142.54x | 3.59x | 8.23x | 0.19x |
| `vec4 dot` | 7.36 ns | 924.96 ns | 3.36 us | 439.55 ns | 321.37 ns | 9.64 us | 125.62x | 2.10x | 2.88x | 0.10x |
| `vec4 magnitude` | 12.72 ns | 4.51 us | 5.53 us | 5.64 us | 420.23 ns | 11.32 us | 354.51x | 0.80x | 10.73x | 0.40x |
| `vec4 normalize` | 36.21 ns | 8.96 us | 8.91 us | 6.26 us | 720.08 ns | 22.34 us | 247.55x | 1.43x | 12.45x | 0.40x |
| `vec4 add` | 7.40 ns | 1.46 us | 2.13 us | 207.15 ns | 186.65 ns | 5.28 us | 197.12x | 7.04x | 7.82x | 0.28x |
| `vec4 add scalar` | 7.20 ns | 1.20 us | 1.90 us | 219.00 ns | 186.53 ns | 5.13 us | 166.48x | 5.48x | 6.43x | 0.23x |
| `vec4 sub` | 5.14 ns | 1.46 us | 2.00 us | 219.10 ns | 184.91 ns | 9.76 us | 283.02x | 6.64x | 7.87x | 0.15x |
| `vec4 sub scalar` | 4.63 ns | 1.20 us | 1.68 us | 205.92 ns | 178.28 ns | 9.44 us | 258.14x | 5.81x | 6.71x | 0.13x |
| `vec4 neg` | 5.01 ns | 280.00 ns | 277.29 ns | 82.91 ns | 61.10 ns | 4.16 us | 55.84x | 3.38x | 4.58x | 0.07x |
| `vec4 mul scalar` | 7.59 ns | 824.02 ns | 2.19 us | 218.86 ns | 159.81 ns | 5.69 us | 108.56x | 3.77x | 5.16x | 0.14x |
| `vec4 div scalar` | 14.68 ns | 1.87 us | 1.67 us | 558.57 ns | 228.83 ns | 10.09 us | 127.66x | 3.36x | 8.19x | 0.19x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.28 ns | 1.79 us | 3.40 us | 974.06 ns | 895.86 ns | 22.78 us | 145.90x | 1.84x | 2.00x | 0.08x |
| `mat3 inverse` | 84.85 ns | 17.97 us | 11.29 us | 3.20 us | 2.52 us | 83.30 us | 211.73x | 5.61x | 7.12x | 0.22x |
| `mat3 mul mat3` | 71.41 ns | 7.82 us | 11.88 us | 2.81 us | 2.37 us | 62.48 us | 109.47x | 2.78x | 3.29x | 0.13x |
| `mat3 transform vec3` | 16.51 ns | 3.26 us | 10.62 us | 1.05 us | 935.05 ns | 20.28 us | 197.18x | 3.10x | 3.48x | 0.16x |
| `mat4 determinant` | 45.33 ns | 5.13 us | 3.94 us | 4.60 us | 4.33 us | 96.86 us | 113.19x | 1.11x | 1.18x | 0.05x |
| `mat4 inverse` | 163.61 ns | 34.67 us | 15.68 us | 11.38 us | 9.43 us | 344.91 us | 211.88x | 3.05x | 3.68x | 0.10x |
| `mat4 mul mat4` | 120.01 ns | 15.76 us | 13.21 us | 6.50 us | 5.83 us | 143.38 us | 131.37x | 2.43x | 2.70x | 0.11x |
| `mat4 transform vec4` | 25.61 ns | 5.43 us | 4.15 us | 1.98 us | 1.73 us | 35.70 us | 212.17x | 2.74x | 3.14x | 0.15x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.93 ns | 1.28 us | 4.56 us | 215.32 ns | 229.88 ns | 2.14 us | 35.63x | 5.95x | 5.57x | 0.60x |
| `mat3 zero` | 16.34 ns | 721.78 ns | 708.35 ns | 250.89 ns | 204.02 ns | 12.25 ns | 44.19x | 2.88x | 3.54x | 58.91x |
| `mat3 identity` | 10.28 ns | 815.83 ns | 809.05 ns | 314.17 ns | 242.19 ns | 143.79 ns | 79.34x | 2.60x | 3.37x | 5.67x |
| `mat3 transpose` | 8.82 ns | 937.40 ns | 920.29 ns | 236.94 ns | 202.55 ns | 125.51 ns | 106.22x | 3.96x | 4.63x | 7.47x |
| `mat3 reciprocal` | 80.16 ns | 17.83 us | 36.24 us | 2.91 us | 2.30 us | 83.31 us | 222.47x | 6.12x | 7.77x | 0.21x |
| `mat3 reciprocal checked` | 79.34 ns | 17.79 us | 36.14 us | 2.91 us | 2.34 us | 81.88 us | 224.17x | 6.12x | 7.59x | 0.22x |
| `mat3 inverse checked` | 79.39 ns | 17.83 us | 35.44 us | 2.95 us | 2.31 us | 81.29 us | 224.62x | 6.04x | 7.70x | 0.22x |
| `mat3 inverse checked abort` | 116.22 ns | 17.68 us | 36.45 us | 2.94 us | 2.33 us | 81.47 us | 152.13x | 6.01x | 7.58x | 0.22x |
| `mat3 powi` | 150.76 ns | 15.70 us | 85.95 us | 6.87 us | 6.13 us | 148.92 us | 104.12x | 2.29x | 2.56x | 0.11x |
| `mat3 powi checked` | 148.80 ns | 15.43 us | 86.09 us | 6.96 us | 6.24 us | 150.88 us | 103.66x | 2.22x | 2.47x | 0.10x |
| `mat3 powi checked abort` | 147.58 ns | 15.53 us | 86.23 us | 6.99 us | 6.17 us | 149.89 us | 105.20x | 2.22x | 2.52x | 0.10x |
| `mat3 div scalar checked` | 25.31 ns | 4.72 us | 5.87 us | 1.52 us | 804.98 ns | 22.39 us | 186.38x | 3.11x | 5.86x | 0.21x |
| `mat3 div scalar checked abort` | 29.53 ns | 4.65 us | 5.97 us | 1.48 us | 801.24 ns | 22.07 us | 157.59x | 3.15x | 5.81x | 0.21x |
| `mat3 div matrix checked` | 150.99 ns | 61.71 us | 62.94 us | 5.47 us | 4.50 us | 160.88 us | 408.71x | 11.29x | 13.71x | 0.38x |
| `mat3 div matrix checked abort` | 162.24 ns | 61.54 us | 64.94 us | 5.48 us | 4.80 us | 160.74 us | 379.30x | 11.23x | 12.83x | 0.38x |
| `mat3 add` | 14.77 ns | 3.99 us | 5.99 us | 520.43 ns | 496.32 ns | 11.77 us | 270.09x | 7.67x | 8.04x | 0.34x |
| `mat3 add scalar` | 12.21 ns | 2.84 us | 5.04 us | 797.69 ns | 727.03 ns | 12.70 us | 232.54x | 3.56x | 3.90x | 0.22x |
| `mat3 sub` | 13.29 ns | 3.87 us | 5.82 us | 546.15 ns | 515.66 ns | 21.61 us | 291.15x | 7.09x | 7.50x | 0.18x |
| `mat3 sub scalar` | 10.94 ns | 2.87 us | 4.71 us | 820.75 ns | 705.48 ns | 21.49 us | 262.34x | 3.50x | 4.07x | 0.13x |
| `mat3 neg` | 10.26 ns | 818.78 ns | 818.83 ns | 479.49 ns | 455.00 ns | 8.90 us | 79.80x | 1.71x | 1.80x | 0.09x |
| `mat3 mul scalar` | 14.19 ns | 2.17 us | 6.08 us | 792.65 ns | 665.93 ns | 12.07 us | 153.03x | 2.74x | 3.26x | 0.18x |
| `mat3 div scalar` | 24.85 ns | 4.75 us | 5.89 us | 1.48 us | 807.72 ns | 22.42 us | 191.16x | 3.22x | 5.88x | 0.21x |
| `mat3 div matrix` | 151.43 ns | 61.99 us | 61.09 us | 5.47 us | 4.55 us | 159.37 us | 409.35x | 11.32x | 13.62x | 0.39x |
| `mat3 bitxor` | 149.84 ns | 15.75 us | 87.98 us | 6.93 us | 6.14 us | 153.14 us | 105.14x | 2.27x | 2.57x | 0.10x |
| `mat4 zero` | 11.65 ns | 1.17 us | 1.20 us | 481.24 ns | 363.35 ns | 14.62 ns | 100.68x | 2.44x | 3.23x | 80.25x |
| `mat4 identity` | 11.60 ns | 1.34 us | 1.38 us | 568.13 ns | 433.45 ns | 216.39 ns | 115.83x | 2.36x | 3.10x | 6.21x |
| `mat4 transpose` | 9.35 ns | 1.43 us | 1.44 us | 423.51 ns | 366.32 ns | 183.00 ns | 153.31x | 3.39x | 3.91x | 7.84x |
| `mat4 reciprocal` | 150.81 ns | 34.88 us | 62.47 us | 11.21 us | 9.02 us | 340.36 us | 231.26x | 3.11x | 3.87x | 0.10x |
| `mat4 reciprocal checked` | 170.25 ns | 35.12 us | 62.71 us | 10.93 us | 8.85 us | 351.05 us | 206.28x | 3.21x | 3.97x | 0.10x |
| `mat4 powi` | 242.03 ns | 32.68 us | 108.73 us | 15.57 us | 13.99 us | 353.35 us | 135.02x | 2.10x | 2.34x | 0.09x |
| `mat4 powi checked` | 240.56 ns | 32.27 us | 108.73 us | 15.83 us | 14.04 us | 348.15 us | 134.13x | 2.04x | 2.30x | 0.09x |
| `mat4 add` | 52.15 ns | 6.14 us | 6.96 us | 1.05 us | 854.19 ns | 19.60 us | 117.69x | 5.82x | 7.19x | 0.31x |
| `mat4 add scalar` | 21.21 ns | 5.12 us | 7.71 us | 1.40 us | 1.20 us | 20.68 us | 241.52x | 3.65x | 4.26x | 0.25x |
| `mat4 sub` | 39.60 ns | 6.17 us | 7.06 us | 1.12 us | 931.42 ns | 36.00 us | 155.76x | 5.52x | 6.62x | 0.17x |
| `mat4 sub scalar` | 15.62 ns | 4.76 us | 7.19 us | 1.43 us | 1.19 us | 37.58 us | 304.76x | 3.32x | 3.99x | 0.13x |
| `mat4 neg` | 14.66 ns | 1.37 us | 1.34 us | 963.73 ns | 745.03 ns | 14.25 us | 93.30x | 1.42x | 1.84x | 0.10x |
| `mat4 mul scalar` | 24.82 ns | 3.61 us | 7.82 us | 1.40 us | 1.11 us | 20.30 us | 145.62x | 2.59x | 3.25x | 0.18x |
| `mat4 div scalar` | 33.97 ns | 8.15 us | 8.06 us | 2.69 us | 1.39 us | 37.38 us | 239.89x | 3.03x | 5.84x | 0.22x |
| `mat4 div matrix` | 224.48 ns | 126.77 us | 96.23 us | 17.00 us | 14.57 us | 542.20 us | 564.72x | 7.46x | 8.70x | 0.23x |
| `mat4 bitxor` | 241.52 ns | 32.75 us | 109.99 us | 15.75 us | 14.32 us | 348.00 us | 135.61x | 2.08x | 2.29x | 0.09x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.65 ns | 202.46 ns | 310.54 ns | - | - | 1.32 us | 55.48x | - | - | 0.15x |
| `scalar add ref_owned` | 12.26 ns | 195.86 ns | 298.90 ns | - | - | 1.29 us | 15.97x | - | - | 0.15x |
| `scalar add refs` | 5.47 ns | 186.51 ns | 286.09 ns | - | - | 1.29 us | 34.08x | - | - | 0.14x |
| `scalar add owned_ref_with_clone` | 9.09 ns | 227.93 ns | 323.77 ns | - | - | - | 25.07x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.98 ns | 225.46 ns | 322.27 ns | - | - | - | 37.73x | - | - | - |
| `scalar sub owned_ref` | 4.04 ns | 218.27 ns | 357.92 ns | - | - | 2.43 us | 54.01x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.60 ns | 206.53 ns | 351.58 ns | - | - | 2.45 us | 16.39x | - | - | 0.08x |
| `scalar sub refs` | 5.92 ns | 189.71 ns | 345.32 ns | - | - | 2.42 us | 32.07x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 9.13 ns | 234.83 ns | 388.29 ns | - | - | - | 25.72x | - | - | - |
| `scalar sub ref_owned_with_clone` | 6.47 ns | 237.58 ns | 394.09 ns | - | - | - | 36.75x | - | - | - |
| `scalar mul owned_ref` | 4.29 ns | 119.72 ns | 392.38 ns | - | - | 1.53 us | 27.92x | - | - | 0.08x |
| `scalar mul ref_owned` | 13.59 ns | 115.24 ns | 387.52 ns | - | - | 1.63 us | 8.48x | - | - | 0.07x |
| `scalar mul refs` | 6.22 ns | 100.13 ns | 360.64 ns | - | - | 1.53 us | 16.10x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 9.84 ns | 140.29 ns | 408.10 ns | - | - | - | 14.26x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.41 ns | 143.02 ns | 408.72 ns | - | - | - | 22.31x | - | - | - |
| `scalar div owned_ref` | 5.84 ns | 278.41 ns | 329.72 ns | - | - | 2.55 us | 47.66x | - | - | 0.11x |
| `scalar div ref_owned` | 17.27 ns | 293.74 ns | 342.09 ns | - | - | 2.51 us | 17.01x | - | - | 0.12x |
| `scalar div refs` | 8.47 ns | 269.10 ns | 314.63 ns | - | - | 2.62 us | 31.76x | - | - | 0.10x |
| `scalar div owned_ref_with_clone` | 13.36 ns | 301.90 ns | 354.05 ns | - | - | - | 22.61x | - | - | - |
| `scalar div ref_owned_with_clone` | 13.77 ns | 307.71 ns | 366.65 ns | - | - | - | 22.34x | - | - | - |
| `vec3 add refs` | 6.11 ns | 674.84 ns | 1.48 us | - | - | 3.99 us | 110.44x | - | - | 0.17x |
| `vec3 sub refs` | 6.12 ns | 678.86 ns | 1.56 us | - | - | 7.27 us | 110.89x | - | - | 0.09x |
| `vec3 neg ref` | 3.35 ns | 237.29 ns | 243.94 ns | - | - | 3.11 us | 70.75x | - | - | 0.08x |
| `vec3 add_scalar_ref` | 6.61 ns | 936.68 ns | 1.61 us | - | - | 3.90 us | 141.67x | - | - | 0.24x |
| `vec3 sub_scalar_ref` | 6.55 ns | 889.69 ns | 1.41 us | - | - | 7.17 us | 135.80x | - | - | 0.12x |
| `vec3 mul_scalar_ref` | 6.96 ns | 630.75 ns | 2.09 us | - | - | 4.36 us | 90.64x | - | - | 0.14x |
| `vec3 div_scalar_ref` | 8.16 ns | 1.66 us | 1.94 us | - | - | 7.66 us | 203.84x | - | - | 0.22x |
| `vec4 add refs` | 6.68 ns | 831.89 ns | 1.48 us | - | - | 5.47 us | 124.55x | - | - | 0.15x |
| `vec4 sub refs` | 3.09 ns | 849.55 ns | 1.35 us | - | - | 9.80 us | 275.12x | - | - | 0.09x |
| `vec4 neg ref` | 4.31 ns | 373.98 ns | 370.61 ns | - | - | 4.11 us | 86.82x | - | - | 0.09x |
| `vec4 add_scalar_ref` | 7.01 ns | 1.58 us | 2.22 us | - | - | 5.24 us | 224.71x | - | - | 0.30x |
| `vec4 sub_scalar_ref` | 4.32 ns | 1.58 us | 1.93 us | - | - | 9.47 us | 365.93x | - | - | 0.17x |
| `vec4 mul_scalar_ref` | 7.58 ns | 1.17 us | 2.51 us | - | - | 5.58 us | 154.78x | - | - | 0.21x |
| `vec4 div_scalar_ref` | 12.33 ns | 2.19 us | 1.97 us | - | - | 9.96 us | 177.97x | - | - | 0.22x |
| `mat3 add refs` | 11.94 ns | 2.34 us | 4.31 us | - | - | 11.72 us | 195.69x | - | - | 0.20x |
| `mat3 sub refs` | 11.08 ns | 2.29 us | 4.26 us | - | - | 21.15 us | 206.87x | - | - | 0.11x |
| `mat3 mul refs` | 32.38 ns | 6.22 us | 28.42 us | - | - | 61.14 us | 192.05x | - | - | 0.10x |
| `mat3 div refs` | 139.30 ns | 61.66 us | 61.56 us | - | - | 163.16 us | 442.62x | - | - | 0.38x |
| `mat3 neg ref` | 15.12 ns | 973.07 ns | 974.89 ns | - | - | 8.65 us | 64.37x | - | - | 0.11x |
| `mat3 add_scalar_ref` | 44.91 ns | 3.14 us | 5.32 us | - | - | 12.07 us | 69.99x | - | - | 0.26x |
| `mat3 sub_scalar_ref` | 45.61 ns | 3.28 us | 5.11 us | - | - | 21.37 us | 72.01x | - | - | 0.15x |
| `mat3 mul_scalar_ref` | 47.45 ns | 2.44 us | 6.28 us | - | - | 12.06 us | 51.46x | - | - | 0.20x |
| `mat3 div_scalar_ref` | 22.08 ns | 5.02 us | 6.28 us | - | - | 21.98 us | 227.47x | - | - | 0.23x |
| `mat4 add refs` | 17.78 ns | 3.51 us | 4.32 us | - | - | 20.00 us | 197.24x | - | - | 0.18x |
| `mat4 sub refs` | 16.79 ns | 3.35 us | 4.38 us | - | - | 36.06 us | 199.61x | - | - | 0.09x |
| `mat4 mul refs` | 73.92 ns | 12.81 us | 28.57 us | - | - | 141.43 us | 173.37x | - | - | 0.09x |
| `mat4 div refs` | 179.31 ns | 121.10 us | 95.30 us | - | - | 525.51 us | 675.35x | - | - | 0.23x |
| `mat4 neg ref` | 18.19 ns | 1.38 us | 1.55 us | - | - | 14.08 us | 76.04x | - | - | 0.10x |
| `mat4 add_scalar_ref` | 53.52 ns | 5.20 us | 8.01 us | - | - | 20.75 us | 97.07x | - | - | 0.25x |
| `mat4 sub_scalar_ref` | 41.14 ns | 5.24 us | 7.76 us | - | - | 37.26 us | 127.35x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 56.73 ns | 3.93 us | 8.39 us | - | - | 19.87 us | 69.24x | - | - | 0.20x |
| `mat4 div_scalar_ref` | 29.25 ns | 8.49 us | 8.50 us | - | - | 38.36 us | 290.18x | - | - | 0.22x |
| `mat3 transform_vec refs` | 14.44 ns | 2.61 us | 11.61 us | - | - | 20.37 us | 180.98x | - | - | 0.13x |
| `mat4 transform_vec refs` | 23.32 ns | 4.11 us | 11.46 us | - | - | 35.43 us | 176.35x | - | - | 0.12x |
| `complex add refs` | 7.86 ns | 367.81 ns | 744.85 ns | - | - | 2.57 us | 46.82x | - | - | 0.14x |
| `complex sub refs` | 8.14 ns | 378.86 ns | 769.10 ns | - | - | 4.82 us | 46.53x | - | - | 0.08x |
| `complex mul refs` | 8.29 ns | 883.53 ns | 3.14 us | - | - | 9.95 us | 106.64x | - | - | 0.09x |
| `complex div refs` | 17.85 ns | 3.19 us | 6.64 us | - | - | 21.48 us | 178.69x | - | - | 0.15x |
| `complex neg ref` | 2.40 ns | 131.06 ns | 112.39 ns | - | - | 2.13 us | 54.65x | - | - | 0.06x |
| `complex div_real_ref` | 7.38 ns | 697.74 ns | 683.87 ns | - | - | 5.14 us | 94.49x | - | - | 0.14x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.05 us |
| `astro sin 160` | 14.24 us |
| `astro sin 192` | 13.93 us |
| `astro sin 256` | 16.23 us |
