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
| `sin 0.1` | 11.10 ns | 223.15 ns | 225.67 ns | 11.10 us | 781.23 ns | 1.85 us | 20.10x | 0.02x | 0.29x | 0.12x |
| `cos 0.1` | 11.76 ns | 216.07 ns | 217.78 ns | 10.94 us | 504.31 ns | 1.67 us | 18.37x | 0.02x | 0.43x | 0.13x |
| `sin 1.23456789` | 11.97 ns | 2.06 us | 2.27 us | 12.88 us | 812.59 ns | 1.84 us | 171.68x | 0.16x | 2.53x | 1.12x |
| `cos 1.23456789` | 12.16 ns | 2.39 us | 2.61 us | 11.05 us | 602.00 ns | 1.67 us | 196.97x | 0.22x | 3.98x | 1.43x |
| `sin 1e6` | 12.80 ns | 5.89 us | 5.92 us | 15.92 us | 1.09 us | 2.08 us | 459.70x | 0.37x | 5.38x | 2.83x |
| `cos 1e6` | 12.67 ns | 5.93 us | 5.99 us | 13.98 us | 834.53 ns | 1.84 us | 468.04x | 0.42x | 7.11x | 3.22x |
| `sin 1e30` | 66.76 ns | 8.09 us | 8.31 us | 18.91 us | 2.91 us | 3.57 us | 121.19x | 0.43x | 2.78x | 2.26x |
| `cos 1e30` | 69.38 ns | 8.07 us | 8.27 us | 15.75 us | 963.82 ns | 3.08 us | 116.36x | 0.51x | 8.38x | 2.62x |
| `sin pi_7` | 12.15 ns | 222.61 ns | 514.42 ns | 12.38 us | 757.85 ns | 1.90 us | 18.32x | 0.02x | 0.29x | 0.12x |
| `cos pi_7` | 11.82 ns | 214.18 ns | 941.38 ns | 11.43 us | 547.37 ns | 1.73 us | 18.11x | 0.02x | 0.39x | 0.12x |
| `sin 1000pi_eps` | 11.96 ns | 5.98 us | 3.96 us | 16.09 us | 2.26 us | 2.85 us | 499.59x | 0.37x | 2.64x | 2.10x |
| `cos 1000pi_eps` | 12.81 ns | 5.99 us | 4.05 us | 13.85 us | 576.27 ns | 1.71 us | 467.98x | 0.43x | 10.40x | 3.50x |
| `asin 0.5` | 11.14 ns | 521.97 ns | 526.22 ns | 48.89 us | 3.00 us | 13.45 us | 46.87x | 0.01x | 0.17x | 0.04x |
| `acos 0.5` | 11.46 ns | 1.13 us | 1.14 us | 59.53 us | 2.91 us | 13.31 us | 98.30x | 0.02x | 0.39x | 0.08x |
| `atanh 0.5` | 14.61 ns | 1.48 us | 1.48 us | 34.33 us | 1.71 us | 13.24 us | 101.26x | 0.04x | 0.87x | 0.11x |
| `asin neg_0.999999` | 14.28 ns | 5.69 us | 4.69 us | 13.73 us | 2.55 us | 13.15 us | 398.52x | 0.41x | 2.23x | 0.43x |
| `acos neg_0.999999` | 15.65 ns | 6.27 us | 5.72 us | 18.42 us | 2.70 us | 13.09 us | 400.48x | 0.34x | 2.33x | 0.48x |
| `atanh neg_0.999999` | 14.65 ns | 4.15 us | 3.70 us | 36.17 us | 1.64 us | 12.98 us | 283.53x | 0.11x | 2.53x | 0.32x |
| `asin 0.999999` | 14.05 ns | 5.72 us | 4.84 us | 13.56 us | 2.53 us | 13.04 us | 407.24x | 0.42x | 2.26x | 0.44x |
| `acos 0.999999` | 14.44 ns | 5.65 us | 4.96 us | 18.36 us | 2.74 us | 13.12 us | 390.99x | 0.31x | 2.06x | 0.43x |
| `atanh 0.999999` | 14.45 ns | 4.25 us | 3.58 us | 31.48 us | 1.65 us | 12.79 us | 293.85x | 0.13x | 2.57x | 0.33x |
| `asin 1e-12` | 9.48 ns | 506.80 ns | 1.15 us | 7.99 us | 1.47 us | 15.37 us | 53.44x | 0.06x | 0.34x | 0.03x |
| `acos 1e-12` | 10.03 ns | 909.10 ns | 1.94 us | 9.79 us | 1.45 us | 15.29 us | 90.61x | 0.09x | 0.63x | 0.06x |
| `atanh 1e-12` | 9.76 ns | 514.33 ns | 883.11 ns | 36.53 us | 170.97 ns | 20.16 us | 52.70x | 0.01x | 3.01x | 0.03x |
| `atan 0.5` | 14.98 ns | 481.76 ns | 451.20 ns | 35.83 us | 2.73 us | 17.86 us | 32.15x | 0.01x | 0.18x | 0.03x |
| `asinh 0.5` | 27.43 ns | 1.69 us | 1.70 us | 39.64 us | 1.63 us | 7.51 us | 61.73x | 0.04x | 1.04x | 0.23x |
| `atan neg_1e-12` | 14.33 ns | 453.59 ns | 723.36 ns | 1.61 us | 1.14 us | 15.51 us | 31.66x | 0.28x | 0.40x | 0.03x |
| `asinh neg_1e-12` | 16.03 ns | 5.46 us | 4.61 us | 43.10 us | 8.68 us | 11.99 us | 340.97x | 0.13x | 0.63x | 0.46x |
| `atan 1e6` | 15.30 ns | 616.07 ns | 625.23 ns | 2.85 us | 1.45 us | 18.35 us | 40.26x | 0.22x | 0.43x | 0.03x |
| `asinh 1e6` | 26.86 ns | 3.28 us | 3.26 us | 36.83 us | 1.69 us | 7.33 us | 122.05x | 0.09x | 1.94x | 0.45x |
| `atan neg_1e6` | 15.54 ns | 725.15 ns | 716.02 ns | 2.85 us | 1.42 us | 18.34 us | 46.66x | 0.25x | 0.51x | 0.04x |
| `asinh neg_1e6` | 27.27 ns | 3.31 us | 3.31 us | 37.03 us | 1.69 us | 7.17 us | 121.41x | 0.09x | 1.96x | 0.46x |
| `acosh 9` | 12.68 ns | 2.89 us | 2.92 us | 42.94 us | 1.67 us | 9.98 us | 227.94x | 0.07x | 1.73x | 0.29x |
| `acosh 1_plus_1e-12` | 12.54 ns | 3.80 us | 5.22 us | 42.63 us | 8.41 us | 11.50 us | 303.10x | 0.09x | 0.45x | 0.33x |
| `acosh 1e6` | 12.59 ns | 3.63 us | 3.65 us | 37.63 us | 1.66 us | 10.04 us | 288.31x | 0.10x | 2.19x | 0.36x |
| `acosh e` | 12.58 ns | 4.02 us | 3.77 us | 41.69 us | 1.67 us | 9.86 us | 319.40x | 0.10x | 2.40x | 0.41x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 55.49 ns | 55.51 ns | 24.97 ns | 15.65 ns | 0.97 ns | 117.03x | 2.22x | 3.55x | 57.43x |
| `one` | 0.47 ns | 64.15 ns | 66.39 ns | 43.47 ns | 30.92 ns | 32.90 ns | 135.76x | 1.48x | 2.08x | 1.95x |
| `e` | 0.48 ns | 73.36 ns | 74.54 ns | 56.12 ns | 1.07 us | 227.91 ns | 153.10x | 1.31x | 0.07x | 0.32x |
| `pi` | 0.48 ns | 57.04 ns | 57.64 ns | 45.66 ns | 50.10 ns | 233.71 ns | 118.74x | 1.25x | 1.14x | 0.24x |
| `tau` | 0.48 ns | 192.17 ns | 190.81 ns | 120.68 ns | 104.75 ns | 1.89 us | 403.19x | 1.59x | 1.83x | 0.10x |
| `add` | 5.35 ns | 291.20 ns | 396.95 ns | 53.53 ns | 42.97 ns | 1.34 us | 54.46x | 5.44x | 6.78x | 0.22x |
| `sub` | 5.33 ns | 296.00 ns | 453.41 ns | 59.08 ns | 45.07 ns | 2.51 us | 55.50x | 5.01x | 6.57x | 0.12x |
| `neg` | 5.17 ns | 62.32 ns | 59.80 ns | 24.05 ns | 20.34 ns | 1.10 us | 12.06x | 2.59x | 3.06x | 0.06x |
| `mul` | 5.58 ns | 206.25 ns | 475.78 ns | 59.97 ns | 45.16 ns | 1.55 us | 36.97x | 3.44x | 4.57x | 0.13x |
| `div` | 9.50 ns | 396.57 ns | 453.06 ns | 150.92 ns | 62.37 ns | 2.63 us | 41.76x | 2.63x | 6.36x | 0.15x |
| `reciprocal` | 8.79 ns | 128.01 ns | 122.17 ns | 168.05 ns | 59.48 ns | 1.60 us | 14.56x | 0.76x | 2.15x | 0.08x |
| `reciprocal checked` | 9.29 ns | 135.72 ns | 133.63 ns | 163.87 ns | 59.52 ns | 1.59 us | 14.61x | 0.83x | 2.28x | 0.09x |
| `reciprocal checked abort` | 25.10 ns | 138.95 ns | 137.71 ns | 164.28 ns | 59.45 ns | 1.62 us | 5.54x | 0.85x | 2.34x | 0.09x |
| `pow` | 31.13 ns | 10.73 us | 10.99 us | 54.44 us | 2.90 us | 2.53 us | 344.54x | 0.20x | 3.69x | 4.24x |
| `powi` | 6.20 ns | 559.22 ns | 2.83 us | 288.24 ns | 85.74 ns | 1.63 us | 90.16x | 1.94x | 6.52x | 0.34x |
| `exp` | 10.30 ns | 1.72 us | 3.08 us | 14.42 us | 934.06 ns | 1.90 us | 167.24x | 0.12x | 1.84x | 0.91x |
| `ln` | 10.84 ns | 1.55 us | 1.56 us | 30.75 us | 1.34 us | 1.87 us | 142.58x | 0.05x | 1.15x | 0.83x |
| `log10` | 13.67 ns | 2.73 us | 2.39 us | 36.05 us | 2.84 us | 6.74 us | 199.92x | 0.08x | 0.96x | 0.41x |
| `log10 abort` | 17.65 ns | 2.72 us | 2.39 us | 36.11 us | 2.83 us | 6.79 us | 154.38x | 0.08x | 0.96x | 0.40x |
| `sqrt` | 20.60 ns | 1.59 us | 1.73 us | 5.27 us | 97.34 ns | 1.49 us | 77.39x | 0.30x | 16.37x | 1.07x |
| `sin` | 15.29 ns | 3.17 us | 3.65 us | 14.20 us | 1.26 us | 2.29 us | 207.57x | 0.22x | 2.53x | 1.39x |
| `cos` | 18.74 ns | 3.14 us | 3.65 us | 12.34 us | 644.78 ns | 1.79 us | 167.71x | 0.25x | 4.87x | 1.76x |
| `tan` | 24.98 ns | 2.90 us | 3.31 us | 31.19 us | 1.59 us | 6.94 us | 116.08x | 0.09x | 1.83x | 0.42x |
| `sinh` | 19.53 ns | 4.40 us | 4.53 us | 3.36 us | 1.15 us | 10.97 us | 225.44x | 1.31x | 3.84x | 0.40x |
| `cosh` | 19.63 ns | 4.33 us | 4.46 us | 8.20 us | 1.07 us | 9.67 us | 220.45x | 0.53x | 4.05x | 0.45x |
| `tanh` | 24.02 ns | 5.92 us | 6.07 us | 3.37 us | 1.22 us | 23.37 us | 246.58x | 1.76x | 4.87x | 0.25x |
| `asin` | 13.49 ns | 3.26 us | 4.74 us | 21.80 us | 2.43 us | 14.03 us | 241.71x | 0.15x | 1.34x | 0.23x |
| `asin abort` | 17.67 ns | 3.21 us | 4.79 us | 22.10 us | 2.44 us | 13.98 us | 181.82x | 0.15x | 1.32x | 0.23x |
| `acos` | 14.53 ns | 3.57 us | 5.53 us | 26.98 us | 2.56 us | 13.91 us | 245.91x | 0.13x | 1.39x | 0.26x |
| `acos abort` | 18.04 ns | 3.60 us | 5.73 us | 26.99 us | 2.58 us | 13.90 us | 199.44x | 0.13x | 1.40x | 0.26x |
| `atan` | 14.97 ns | 603.69 ns | 1.25 us | 18.76 us | 2.27 us | 19.13 us | 40.34x | 0.03x | 0.27x | 0.03x |
| `atan abort` | 21.78 ns | 617.07 ns | 1.29 us | 18.42 us | 2.29 us | 19.13 us | 28.34x | 0.03x | 0.27x | 0.03x |
| `asinh` | 49.09 ns | 3.62 us | 5.46 us | 39.71 us | 1.69 us | 7.77 us | 73.84x | 0.09x | 2.14x | 0.47x |
| `asinh abort` | 31.54 ns | 3.69 us | 5.44 us | 39.40 us | 1.67 us | 7.65 us | 116.95x | 0.09x | 2.21x | 0.48x |
| `acosh` | 12.77 ns | 3.79 us | 5.31 us | 40.28 us | 3.43 us | 10.58 us | 296.84x | 0.09x | 1.10x | 0.36x |
| `acosh abort` | 15.85 ns | 3.75 us | 5.40 us | 40.74 us | 3.42 us | 10.54 us | 236.75x | 0.09x | 1.10x | 0.36x |
| `atanh` | 14.12 ns | 2.64 us | 3.32 us | 35.21 us | 1.32 us | 15.06 us | 186.68x | 0.07x | 1.99x | 0.18x |
| `atanh abort` | 17.68 ns | 2.72 us | 3.41 us | 35.24 us | 1.32 us | 15.11 us | 153.96x | 0.08x | 2.06x | 0.18x |
| `zero status` | 1.24 ns | 1.97 ns | 1.99 ns | 0.99 ns | 6.88 ns | 8.29 ns | 1.59x | 1.98x | 0.29x | 0.24x |
| `zero status abort` | 1.45 ns | 3.39 ns | 3.42 ns | 1.07 ns | 6.81 ns | 8.09 ns | 2.34x | 3.16x | 0.50x | 0.42x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 111.01 ns | 109.63 ns | 48.26 ns | - | 1.92 ns | 118.08x | 2.30x | - | 57.78x |
| `one` | 5.15 ns | 115.48 ns | 117.04 ns | 65.71 ns | - | 31.41 ns | 22.44x | 1.76x | - | 3.68x |
| `i` | 0.96 ns | 118.35 ns | 117.81 ns | 64.88 ns | - | 33.68 ns | 123.82x | 1.82x | - | 3.51x |
| `free i` | 0.94 ns | 118.82 ns | 117.69 ns | 64.93 ns | - | 33.44 ns | 126.09x | 1.83x | - | 3.55x |
| `conjugate` | 2.14 ns | 131.69 ns | 129.02 ns | 42.40 ns | - | 1.12 us | 61.40x | 3.11x | - | 0.12x |
| `norm squared` | 5.90 ns | 409.82 ns | 1.54 us | 153.66 ns | - | 4.44 us | 69.50x | 2.67x | - | 0.09x |
| `reciprocal` | 18.97 ns | 2.01 us | 3.10 us | 464.41 ns | - | 11.01 us | 106.08x | 4.33x | - | 0.18x |
| `reciprocal checked` | 14.42 ns | 2.03 us | 3.11 us | 459.94 ns | - | 11.23 us | 140.44x | 4.40x | - | 0.18x |
| `powi` | 18.36 ns | 2.79 us | 14.09 us | 1.50 us | - | 45.00 us | 151.76x | 1.86x | - | 0.06x |
| `powi checked` | 18.35 ns | 2.76 us | 14.17 us | 1.54 us | - | 45.04 us | 150.21x | 1.79x | - | 0.06x |
| `div checked` | 19.67 ns | 3.19 us | 6.66 us | 822.51 ns | - | 22.34 us | 162.29x | 3.88x | - | 0.14x |
| `div real checked` | 9.51 ns | 810.37 ns | 778.09 ns | 263.86 ns | - | 5.26 us | 85.25x | 3.07x | - | 0.15x |
| `from scalar` | 1.44 ns | 115.00 ns | 115.34 ns | 47.81 ns | - | 10.74 ns | 79.79x | 2.41x | - | 10.71x |
| `add` | 6.12 ns | 651.46 ns | 1.01 us | 106.75 ns | - | 2.66 us | 106.49x | 6.10x | - | 0.24x |
| `sub` | 6.17 ns | 637.49 ns | 1.03 us | 125.15 ns | - | 4.87 us | 103.31x | 5.09x | - | 0.13x |
| `neg` | 2.65 ns | 131.88 ns | 130.82 ns | 44.26 ns | - | 2.27 us | 49.82x | 2.98x | - | 0.06x |
| `mul` | 7.67 ns | 1.16 us | 3.54 us | 307.27 ns | - | 10.07 us | 151.55x | 3.78x | - | 0.12x |
| `div` | 18.21 ns | 3.16 us | 6.70 us | 805.50 ns | - | 21.99 us | 173.39x | 3.92x | - | 0.14x |
| `div real` | 10.23 ns | 800.89 ns | 785.23 ns | 269.26 ns | - | 5.33 us | 78.32x | 2.97x | - | 0.15x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.83 ns | 862.49 ns | 6.01 us | 319.88 ns | 255.31 ns | 7.45 us | 126.23x | 2.70x | 3.38x | 0.12x |
| `vec3 magnitude` | 9.63 ns | 4.63 us | 8.91 us | 5.41 us | 353.62 ns | 8.91 us | 480.18x | 0.85x | 13.08x | 0.52x |
| `vec3 normalize` | 25.78 ns | 8.87 us | 11.07 us | 6.15 us | 591.18 ns | 17.14 us | 344.13x | 1.44x | 15.01x | 0.52x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.08 ns | 310.77 ns | 1.59 us | 71.42 ns | 56.41 ns | 725.77 ns | 100.78x | 4.35x | 5.51x | 0.43x |
| `vec3 zero` | 1.44 ns | 262.29 ns | 263.16 ns | 59.09 ns | 30.86 ns | 2.82 ns | 182.70x | 4.44x | 8.50x | 92.88x |
| `vec3 dot abort` | 27.93 ns | 1.29 us | 4.59 us | 257.09 ns | 204.27 ns | 7.33 us | 46.05x | 5.00x | 6.30x | 0.18x |
| `vec3 magnitude abort` | 39.30 ns | 5.22 us | 6.97 us | 5.40 us | 320.77 ns | 9.06 us | 132.79x | 0.97x | 16.27x | 0.58x |
| `vec3 normalize checked` | 26.28 ns | 8.80 us | 9.02 us | 5.82 us | 541.10 ns | 17.12 us | 334.78x | 1.51x | 16.26x | 0.51x |
| `vec3 normalize checked abort` | 55.49 ns | 9.42 us | 9.42 us | 5.84 us | 538.98 ns | 17.09 us | 169.73x | 1.61x | 17.47x | 0.55x |
| `vec3 div scalar checked` | 10.52 ns | 1.53 us | 1.77 us | 389.19 ns | - | - | 145.88x | 3.94x | - | - |
| `vec3 div scalar checked abort` | 18.26 ns | 1.52 us | 1.79 us | 396.42 ns | - | - | 83.33x | 3.84x | - | - |
| `vec3 add` | 6.84 ns | 1.21 us | 2.01 us | 152.18 ns | 125.88 ns | 4.16 us | 177.49x | 7.98x | 9.64x | 0.29x |
| `vec3 add scalar` | 6.48 ns | 980.82 ns | 1.69 us | 156.51 ns | 133.53 ns | 3.97 us | 151.40x | 6.27x | 7.35x | 0.25x |
| `vec3 sub` | 6.78 ns | 1.19 us | 2.03 us | 166.49 ns | 136.60 ns | 7.45 us | 175.81x | 7.16x | 8.73x | 0.16x |
| `vec3 sub scalar` | 6.55 ns | 980.40 ns | 1.49 us | 151.36 ns | 124.94 ns | 7.23 us | 149.67x | 6.48x | 7.85x | 0.14x |
| `vec3 neg` | 3.84 ns | 245.44 ns | 241.24 ns | 57.12 ns | 46.40 ns | 3.19 us | 63.92x | 4.30x | 5.29x | 0.08x |
| `vec3 mul scalar` | 7.08 ns | 698.88 ns | 2.23 us | 166.77 ns | 119.27 ns | 4.45 us | 98.69x | 4.19x | 5.86x | 0.16x |
| `vec3 div scalar` | 18.85 ns | 1.50 us | 1.80 us | 402.41 ns | 183.92 ns | 7.79 us | 79.78x | 3.74x | 8.18x | 0.19x |
| `vec4 dot` | 7.36 ns | 1.02 us | 3.52 us | 445.92 ns | 313.40 ns | 9.85 us | 138.67x | 2.29x | 3.26x | 0.10x |
| `vec4 magnitude` | 13.15 ns | 4.61 us | 5.65 us | 5.49 us | 407.60 ns | 11.48 us | 350.37x | 0.84x | 11.30x | 0.40x |
| `vec4 normalize` | 36.55 ns | 9.17 us | 7.68 us | 6.10 us | 702.67 ns | 22.68 us | 250.87x | 1.50x | 13.05x | 0.40x |
| `vec4 add` | 8.09 ns | 1.56 us | 2.26 us | 209.42 ns | 172.21 ns | 5.47 us | 192.89x | 7.46x | 9.07x | 0.29x |
| `vec4 add scalar` | 7.05 ns | 1.24 us | 1.99 us | 220.93 ns | 176.51 ns | 5.28 us | 175.24x | 5.59x | 7.00x | 0.23x |
| `vec4 sub` | 5.26 ns | 1.54 us | 2.10 us | 216.58 ns | 176.98 ns | 9.81 us | 292.18x | 7.10x | 8.69x | 0.16x |
| `vec4 sub scalar` | 4.51 ns | 1.25 us | 1.75 us | 209.96 ns | 168.14 ns | 9.64 us | 276.46x | 5.94x | 7.41x | 0.13x |
| `vec4 neg` | 4.98 ns | 325.69 ns | 333.30 ns | 78.11 ns | 65.33 ns | 4.15 us | 65.44x | 4.17x | 4.99x | 0.08x |
| `vec4 mul scalar` | 7.59 ns | 895.65 ns | 2.28 us | 228.61 ns | 159.91 ns | 5.69 us | 118.00x | 3.92x | 5.60x | 0.16x |
| `vec4 div scalar` | 14.34 ns | 2.03 us | 1.86 us | 530.59 ns | 242.79 ns | 10.17 us | 141.21x | 3.82x | 8.34x | 0.20x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.48 ns | 1.88 us | 3.56 us | 1.00 us | 867.76 ns | 22.97 us | 150.75x | 1.88x | 2.17x | 0.08x |
| `mat3 inverse` | 80.71 ns | 18.46 us | 12.00 us | 3.22 us | 2.62 us | 84.00 us | 228.65x | 5.74x | 7.05x | 0.22x |
| `mat3 mul mat3` | 74.12 ns | 8.20 us | 12.84 us | 2.89 us | 2.42 us | 62.43 us | 110.65x | 2.84x | 3.38x | 0.13x |
| `mat3 transform vec3` | 16.34 ns | 3.45 us | 11.25 us | 1.07 us | 896.98 ns | 21.06 us | 211.15x | 3.22x | 3.85x | 0.16x |
| `mat4 determinant` | 45.96 ns | 5.45 us | 4.29 us | 4.85 us | 4.24 us | 99.83 us | 118.50x | 1.12x | 1.28x | 0.05x |
| `mat4 inverse` | 153.51 ns | 36.23 us | 17.29 us | 11.46 us | 9.40 us | 355.32 us | 236.02x | 3.16x | 3.85x | 0.10x |
| `mat4 mul mat4` | 117.96 ns | 17.00 us | 14.16 us | 6.68 us | 5.49 us | 145.96 us | 144.10x | 2.55x | 3.10x | 0.12x |
| `mat4 transform vec4` | 25.57 ns | 5.62 us | 4.62 us | 1.93 us | 1.73 us | 36.34 us | 219.63x | 2.91x | 3.25x | 0.15x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.52 ns | 1.38 us | 4.68 us | 207.73 ns | 205.14 ns | 2.07 us | 39.96x | 6.64x | 6.72x | 0.67x |
| `mat3 zero` | 16.90 ns | 881.09 ns | 884.45 ns | 259.83 ns | 167.28 ns | 11.40 ns | 52.15x | 3.39x | 5.27x | 77.28x |
| `mat3 identity` | 10.03 ns | 993.65 ns | 995.40 ns | 329.04 ns | 208.65 ns | 152.17 ns | 99.12x | 3.02x | 4.76x | 6.53x |
| `mat3 transpose` | 9.13 ns | 932.13 ns | 920.33 ns | 229.40 ns | 181.27 ns | 130.11 ns | 102.07x | 4.06x | 5.14x | 7.16x |
| `mat3 reciprocal` | 78.53 ns | 18.31 us | 36.68 us | 2.97 us | 2.23 us | 84.44 us | 233.19x | 6.17x | 8.22x | 0.22x |
| `mat3 reciprocal checked` | 78.74 ns | 18.55 us | 36.85 us | 2.95 us | 2.24 us | 83.23 us | 235.59x | 6.30x | 8.29x | 0.22x |
| `mat3 inverse checked` | 79.11 ns | 19.16 us | 37.06 us | 2.96 us | 2.22 us | 83.25 us | 242.21x | 6.47x | 8.61x | 0.23x |
| `mat3 inverse checked abort` | 115.09 ns | 18.31 us | 37.42 us | 2.96 us | 2.23 us | 83.74 us | 159.06x | 6.18x | 8.21x | 0.22x |
| `mat3 powi` | 144.24 ns | 16.82 us | 92.18 us | 6.77 us | 6.23 us | 152.02 us | 116.63x | 2.48x | 2.70x | 0.11x |
| `mat3 powi checked` | 144.12 ns | 16.86 us | 91.84 us | 6.80 us | 6.16 us | 151.69 us | 117.00x | 2.48x | 2.74x | 0.11x |
| `mat3 powi checked abort` | 144.61 ns | 17.23 us | 90.54 us | 6.78 us | 6.18 us | 151.29 us | 119.14x | 2.54x | 2.79x | 0.11x |
| `mat3 div scalar checked` | 24.66 ns | 4.94 us | 6.34 us | 1.45 us | 816.41 ns | 22.55 us | 200.13x | 3.41x | 6.05x | 0.22x |
| `mat3 div scalar checked abort` | 29.29 ns | 4.88 us | 6.35 us | 1.45 us | 816.10 ns | 22.51 us | 166.71x | 3.37x | 5.98x | 0.22x |
| `mat3 div matrix checked` | 147.79 ns | 68.61 us | 67.98 us | 5.52 us | 4.36 us | 160.38 us | 464.25x | 12.42x | 15.75x | 0.43x |
| `mat3 div matrix checked abort` | 160.62 ns | 67.93 us | 67.65 us | 5.49 us | 4.37 us | 161.01 us | 422.91x | 12.37x | 15.53x | 0.42x |
| `mat3 add` | 14.77 ns | 4.17 us | 6.31 us | 520.15 ns | 484.75 ns | 11.95 us | 282.16x | 8.01x | 8.60x | 0.35x |
| `mat3 add scalar` | 12.22 ns | 3.11 us | 5.26 us | 800.58 ns | 704.10 ns | 12.30 us | 254.52x | 3.89x | 4.42x | 0.25x |
| `mat3 sub` | 13.70 ns | 4.22 us | 6.26 us | 547.24 ns | 518.35 ns | 21.44 us | 307.91x | 7.71x | 8.14x | 0.20x |
| `mat3 sub scalar` | 11.35 ns | 3.12 us | 5.19 us | 808.59 ns | 702.27 ns | 21.92 us | 275.30x | 3.86x | 4.45x | 0.14x |
| `mat3 neg` | 10.74 ns | 951.40 ns | 958.78 ns | 485.69 ns | 461.64 ns | 8.73 us | 88.54x | 1.96x | 2.06x | 0.11x |
| `mat3 mul scalar` | 13.86 ns | 2.37 us | 6.36 us | 774.55 ns | 646.13 ns | 12.23 us | 170.98x | 3.06x | 3.67x | 0.19x |
| `mat3 div scalar` | 24.84 ns | 5.00 us | 6.48 us | 1.45 us | 814.11 ns | 22.49 us | 201.30x | 3.46x | 6.14x | 0.22x |
| `mat3 div matrix` | 148.52 ns | 68.80 us | 67.63 us | 5.50 us | 4.35 us | 160.58 us | 463.25x | 12.50x | 15.80x | 0.43x |
| `mat3 bitxor` | 144.35 ns | 16.80 us | 91.97 us | 6.75 us | 6.20 us | 150.81 us | 116.35x | 2.49x | 2.71x | 0.11x |
| `mat4 zero` | 13.23 ns | 1.38 us | 1.41 us | 514.16 ns | 348.52 ns | 14.01 ns | 104.15x | 2.68x | 3.95x | 98.32x |
| `mat4 identity` | 10.22 ns | 1.63 us | 1.65 us | 545.06 ns | 412.02 ns | 229.41 ns | 159.42x | 2.99x | 3.95x | 7.10x |
| `mat4 transpose` | 9.82 ns | 1.68 us | 1.70 us | 442.16 ns | 369.37 ns | 174.86 ns | 171.23x | 3.80x | 4.55x | 9.61x |
| `mat4 reciprocal` | 159.55 ns | 36.12 us | 66.63 us | 10.85 us | 9.12 us | 347.45 us | 226.40x | 3.33x | 3.96x | 0.10x |
| `mat4 reciprocal checked` | 162.19 ns | 36.24 us | 65.28 us | 10.70 us | 8.99 us | 347.24 us | 223.45x | 3.39x | 4.03x | 0.10x |
| `mat4 powi` | 240.76 ns | 34.71 us | 115.66 us | 15.67 us | 13.88 us | 355.30 us | 144.17x | 2.22x | 2.50x | 0.10x |
| `mat4 powi checked` | 241.32 ns | 34.48 us | 114.40 us | 15.75 us | 14.04 us | 354.24 us | 142.89x | 2.19x | 2.46x | 0.10x |
| `mat4 add` | 50.87 ns | 7.06 us | 8.09 us | 882.39 ns | 857.61 ns | 19.96 us | 138.83x | 8.00x | 8.23x | 0.35x |
| `mat4 add scalar` | 20.66 ns | 5.02 us | 7.82 us | 1.44 us | 1.20 us | 21.23 us | 243.02x | 3.48x | 4.19x | 0.24x |
| `mat4 sub` | 37.23 ns | 7.10 us | 8.10 us | 938.35 ns | 901.06 ns | 36.36 us | 190.71x | 7.57x | 7.88x | 0.20x |
| `mat4 sub scalar` | 15.01 ns | 5.13 us | 7.73 us | 1.45 us | 1.18 us | 38.02 us | 341.61x | 3.54x | 4.34x | 0.13x |
| `mat4 neg` | 13.60 ns | 1.62 us | 1.63 us | 936.61 ns | 783.57 ns | 14.58 us | 119.34x | 1.73x | 2.07x | 0.11x |
| `mat4 mul scalar` | 24.02 ns | 3.79 us | 8.52 us | 1.44 us | 1.13 us | 20.57 us | 157.61x | 2.62x | 3.34x | 0.18x |
| `mat4 div scalar` | 32.25 ns | 8.47 us | 8.88 us | 2.70 us | 1.41 us | 38.56 us | 262.71x | 3.14x | 6.00x | 0.22x |
| `mat4 div matrix` | 217.16 ns | 137.28 us | 111.39 us | 16.88 us | 14.36 us | 538.64 us | 632.16x | 8.13x | 9.56x | 0.25x |
| `mat4 bitxor` | 242.56 ns | 35.19 us | 116.28 us | 15.86 us | 13.81 us | 353.96 us | 145.06x | 2.22x | 2.55x | 0.10x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.66 ns | 220.93 ns | 319.08 ns | - | - | 1.32 us | 60.40x | - | - | 0.17x |
| `scalar add ref_owned` | 12.12 ns | 212.61 ns | 316.12 ns | - | - | 1.33 us | 17.55x | - | - | 0.16x |
| `scalar add refs` | 5.32 ns | 207.38 ns | 296.64 ns | - | - | 1.32 us | 38.97x | - | - | 0.16x |
| `scalar add owned_ref_with_clone` | 8.97 ns | 248.32 ns | 355.01 ns | - | - | - | 27.68x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.47 ns | 246.98 ns | 355.15 ns | - | - | - | 45.16x | - | - | - |
| `scalar sub owned_ref` | 3.92 ns | 223.24 ns | 382.72 ns | - | - | 2.50 us | 56.91x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.27 ns | 218.87 ns | 381.55 ns | - | - | 2.50 us | 17.84x | - | - | 0.09x |
| `scalar sub refs` | 5.57 ns | 203.41 ns | 362.07 ns | - | - | 2.48 us | 36.51x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 8.96 ns | 252.98 ns | 410.70 ns | - | - | - | 28.23x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.68 ns | 254.66 ns | 406.75 ns | - | - | - | 44.82x | - | - | - |
| `scalar mul owned_ref` | 4.74 ns | 130.70 ns | 406.53 ns | - | - | 1.54 us | 27.58x | - | - | 0.09x |
| `scalar mul ref_owned` | 13.48 ns | 128.10 ns | 402.78 ns | - | - | 1.55 us | 9.51x | - | - | 0.08x |
| `scalar mul refs` | 6.23 ns | 108.31 ns | 376.74 ns | - | - | 1.54 us | 17.40x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 9.67 ns | 157.50 ns | 437.57 ns | - | - | - | 16.28x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.26 ns | 156.35 ns | 432.11 ns | - | - | - | 24.99x | - | - | - |
| `scalar div owned_ref` | 5.81 ns | 290.82 ns | 356.39 ns | - | - | 2.59 us | 50.08x | - | - | 0.11x |
| `scalar div ref_owned` | 17.18 ns | 301.35 ns | 355.10 ns | - | - | 2.57 us | 17.54x | - | - | 0.12x |
| `scalar div refs` | 6.74 ns | 278.97 ns | 333.08 ns | - | - | 2.57 us | 41.36x | - | - | 0.11x |
| `scalar div owned_ref_with_clone` | 15.49 ns | 319.25 ns | 387.11 ns | - | - | - | 20.61x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.84 ns | 333.43 ns | 387.22 ns | - | - | - | 21.05x | - | - | - |
| `vec3 add refs` | 6.26 ns | 740.57 ns | 1.53 us | - | - | 4.09 us | 118.31x | - | - | 0.18x |
| `vec3 sub refs` | 6.17 ns | 749.41 ns | 1.59 us | - | - | 7.41 us | 121.51x | - | - | 0.10x |
| `vec3 neg ref` | 3.28 ns | 266.61 ns | 268.40 ns | - | - | 3.19 us | 81.21x | - | - | 0.08x |
| `vec3 add_scalar_ref` | 6.44 ns | 1.03 us | 1.70 us | - | - | 3.91 us | 160.52x | - | - | 0.26x |
| `vec3 sub_scalar_ref` | 6.51 ns | 1.03 us | 1.47 us | - | - | 7.22 us | 157.58x | - | - | 0.14x |
| `vec3 mul_scalar_ref` | 6.91 ns | 768.01 ns | 2.22 us | - | - | 4.41 us | 111.19x | - | - | 0.17x |
| `vec3 div_scalar_ref` | 8.04 ns | 1.48 us | 1.75 us | - | - | 7.68 us | 184.11x | - | - | 0.19x |
| `vec4 add refs` | 6.44 ns | 902.46 ns | 1.55 us | - | - | 5.44 us | 140.04x | - | - | 0.17x |
| `vec4 sub refs` | 3.06 ns | 916.04 ns | 1.43 us | - | - | 9.81 us | 299.18x | - | - | 0.09x |
| `vec4 neg ref` | 4.21 ns | 330.02 ns | 327.56 ns | - | - | 4.14 us | 78.34x | - | - | 0.08x |
| `vec4 add_scalar_ref` | 6.90 ns | 1.25 us | 1.96 us | - | - | 5.19 us | 180.64x | - | - | 0.24x |
| `vec4 sub_scalar_ref` | 4.23 ns | 1.23 us | 1.72 us | - | - | 9.70 us | 290.68x | - | - | 0.13x |
| `vec4 mul_scalar_ref` | 7.33 ns | 859.54 ns | 2.28 us | - | - | 5.72 us | 117.23x | - | - | 0.15x |
| `vec4 div_scalar_ref` | 12.50 ns | 1.93 us | 1.73 us | - | - | 10.05 us | 154.72x | - | - | 0.19x |
| `mat3 add refs` | 11.24 ns | 2.04 us | 4.21 us | - | - | 11.93 us | 181.44x | - | - | 0.17x |
| `mat3 sub refs` | 10.81 ns | 2.02 us | 4.06 us | - | - | 21.69 us | 186.72x | - | - | 0.09x |
| `mat3 mul refs` | 56.97 ns | 6.64 us | 29.57 us | - | - | 62.23 us | 116.51x | - | - | 0.11x |
| `mat3 div refs` | 150.90 ns | 68.31 us | 67.10 us | - | - | 160.56 us | 452.65x | - | - | 0.43x |
| `mat3 neg ref` | 9.84 ns | 804.51 ns | 768.98 ns | - | - | 8.80 us | 81.72x | - | - | 0.09x |
| `mat3 add_scalar_ref` | 44.04 ns | 3.06 us | 6.01 us | - | - | 12.30 us | 69.59x | - | - | 0.25x |
| `mat3 sub_scalar_ref` | 44.51 ns | 3.07 us | 5.83 us | - | - | 21.76 us | 68.95x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 47.43 ns | 2.48 us | 6.44 us | - | - | 12.35 us | 52.36x | - | - | 0.20x |
| `mat3 div_scalar_ref` | 22.03 ns | 4.98 us | 6.24 us | - | - | 22.53 us | 226.06x | - | - | 0.22x |
| `mat4 add refs` | 16.62 ns | 3.46 us | 4.20 us | - | - | 20.16 us | 207.94x | - | - | 0.17x |
| `mat4 sub refs` | 16.37 ns | 3.18 us | 4.30 us | - | - | 37.90 us | 194.47x | - | - | 0.08x |
| `mat4 mul refs` | 100.97 ns | 17.59 us | 30.21 us | - | - | 144.83 us | 174.20x | - | - | 0.12x |
| `mat4 div refs` | 217.08 ns | 138.37 us | 108.22 us | - | - | 542.71 us | 637.40x | - | - | 0.25x |
| `mat4 neg ref` | 12.00 ns | 1.39 us | 1.38 us | - | - | 14.65 us | 115.47x | - | - | 0.09x |
| `mat4 add_scalar_ref` | 48.82 ns | 5.25 us | 7.90 us | - | - | 21.01 us | 107.43x | - | - | 0.25x |
| `mat4 sub_scalar_ref` | 37.04 ns | 5.25 us | 7.84 us | - | - | 38.06 us | 141.78x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 52.49 ns | 3.97 us | 8.52 us | - | - | 20.95 us | 75.72x | - | - | 0.19x |
| `mat4 div_scalar_ref` | 27.32 ns | 8.86 us | 8.64 us | - | - | 38.66 us | 324.21x | - | - | 0.23x |
| `mat3 transform_vec refs` | 14.42 ns | 2.49 us | 11.60 us | - | - | 20.33 us | 172.77x | - | - | 0.12x |
| `mat4 transform_vec refs` | 22.97 ns | 4.06 us | 11.78 us | - | - | 35.50 us | 176.63x | - | - | 0.11x |
| `complex add refs` | 7.72 ns | 400.62 ns | 784.84 ns | - | - | 2.64 us | 51.92x | - | - | 0.15x |
| `complex sub refs` | 8.01 ns | 395.39 ns | 799.60 ns | - | - | 4.83 us | 49.36x | - | - | 0.08x |
| `complex mul refs` | 8.00 ns | 893.89 ns | 3.27 us | - | - | 10.04 us | 111.73x | - | - | 0.09x |
| `complex div refs` | 17.31 ns | 2.82 us | 6.54 us | - | - | 21.90 us | 162.88x | - | - | 0.13x |
| `complex neg ref` | 2.35 ns | 118.46 ns | 120.19 ns | - | - | 2.19 us | 50.33x | - | - | 0.05x |
| `complex div_real_ref` | 7.24 ns | 749.69 ns | 711.07 ns | - | - | 5.22 us | 103.51x | - | - | 0.14x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.42 us |
| `astro sin 160` | 14.08 us |
| `astro sin 192` | 13.90 us |
| `astro sin 256` | 16.03 us |
