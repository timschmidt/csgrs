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
| `sin 0.1` | 11.01 ns | 205.10 ns | 201.24 ns | 11.00 us | 772.25 ns | 1.87 us | 18.63x | 0.02x | 0.27x | 0.11x |
| `cos 0.1` | 11.81 ns | 208.01 ns | 202.85 ns | 10.66 us | 495.62 ns | 1.70 us | 17.62x | 0.02x | 0.42x | 0.12x |
| `sin 1.23456789` | 11.71 ns | 620.97 ns | 633.16 ns | 12.77 us | 839.62 ns | 1.83 us | 53.01x | 0.05x | 0.74x | 0.34x |
| `cos 1.23456789` | 12.14 ns | 538.98 ns | 533.54 ns | 10.78 us | 595.21 ns | 1.66 us | 44.38x | 0.05x | 0.91x | 0.32x |
| `sin 1e6` | 12.92 ns | 2.60 us | 2.64 us | 15.91 us | 1.13 us | 2.07 us | 201.24x | 0.16x | 2.31x | 1.26x |
| `cos 1e6` | 12.37 ns | 2.24 us | 2.23 us | 13.91 us | 849.97 ns | 1.84 us | 180.93x | 0.16x | 2.63x | 1.21x |
| `sin 1e30` | 66.18 ns | 2.82 us | 2.86 us | 18.57 us | 2.89 us | 3.57 us | 42.56x | 0.15x | 0.97x | 0.79x |
| `cos 1e30` | 72.41 ns | 2.38 us | 2.41 us | 15.66 us | 974.76 ns | 3.13 us | 32.87x | 0.15x | 2.44x | 0.76x |
| `sin pi_7` | 11.72 ns | 223.67 ns | 523.41 ns | 11.98 us | 758.44 ns | 1.88 us | 19.09x | 0.02x | 0.29x | 0.12x |
| `cos pi_7` | 11.71 ns | 225.12 ns | 1.00 us | 11.05 us | 565.44 ns | 1.76 us | 19.22x | 0.02x | 0.40x | 0.13x |
| `sin 1000pi_eps` | 11.88 ns | 3.15 us | 4.15 us | 15.94 us | 2.32 us | 2.84 us | 265.24x | 0.20x | 1.36x | 1.11x |
| `cos 1000pi_eps` | 12.60 ns | 2.68 us | 3.65 us | 13.62 us | 584.17 ns | 1.69 us | 212.62x | 0.20x | 4.59x | 1.59x |
| `asin 0.5` | 11.02 ns | 529.16 ns | 535.83 ns | 49.28 us | 2.95 us | 13.58 us | 48.04x | 0.01x | 0.18x | 0.04x |
| `acos 0.5` | 11.42 ns | 1.20 us | 1.24 us | 59.40 us | 3.02 us | 13.05 us | 105.36x | 0.02x | 0.40x | 0.09x |
| `atanh 0.5` | 14.53 ns | 1.72 us | 1.72 us | 35.04 us | 1.70 us | 12.95 us | 118.34x | 0.05x | 1.01x | 0.13x |
| `asin neg_0.999999` | 14.21 ns | 8.83 us | 6.20 us | 13.82 us | 2.59 us | 12.90 us | 621.30x | 0.64x | 3.41x | 0.68x |
| `acos neg_0.999999` | 15.25 ns | 12.49 us | 8.90 us | 18.39 us | 2.74 us | 12.96 us | 818.96x | 0.68x | 4.56x | 0.96x |
| `atanh neg_0.999999` | 14.40 ns | 4.61 us | 3.58 us | 37.14 us | 1.61 us | 12.73 us | 319.98x | 0.12x | 2.86x | 0.36x |
| `asin 0.999999` | 14.05 ns | 8.79 us | 6.03 us | 13.71 us | 2.61 us | 12.80 us | 625.55x | 0.64x | 3.37x | 0.69x |
| `acos 0.999999` | 14.37 ns | 11.65 us | 8.13 us | 18.50 us | 2.74 us | 12.93 us | 810.67x | 0.63x | 4.25x | 0.90x |
| `atanh 0.999999` | 14.57 ns | 4.48 us | 3.50 us | 31.56 us | 1.57 us | 12.51 us | 307.67x | 0.14x | 2.85x | 0.36x |
| `asin 1e-12` | 9.42 ns | 9.56 us | 5.06 us | 8.02 us | 1.44 us | 15.04 us | 1015.22x | 1.19x | 6.62x | 0.64x |
| `acos 1e-12` | 10.23 ns | 11.80 us | 7.00 us | 9.88 us | 1.47 us | 15.21 us | 1154.29x | 1.20x | 8.05x | 0.78x |
| `atanh 1e-12` | 10.13 ns | 5.94 us | 3.23 us | 36.92 us | 181.70 ns | 20.18 us | 586.84x | 0.16x | 32.71x | 0.29x |
| `atan 0.5` | 15.05 ns | 521.99 ns | 515.10 ns | 36.62 us | 2.80 us | 17.62 us | 34.68x | 0.01x | 0.19x | 0.03x |
| `asinh 0.5` | 26.95 ns | 1.79 us | 1.75 us | 39.78 us | 1.63 us | 7.40 us | 66.39x | 0.04x | 1.10x | 0.24x |
| `atan neg_1e-12` | 14.27 ns | 777.16 ns | 674.50 ns | 1.60 us | 1.12 us | 15.24 us | 54.45x | 0.49x | 0.69x | 0.05x |
| `asinh neg_1e-12` | 16.05 ns | 7.41 us | 4.34 us | 42.49 us | 8.67 us | 11.90 us | 461.85x | 0.17x | 0.85x | 0.62x |
| `atan 1e6` | 15.29 ns | 594.20 ns | 581.45 ns | 2.87 us | 1.49 us | 18.34 us | 38.87x | 0.21x | 0.40x | 0.03x |
| `asinh 1e6` | 27.21 ns | 4.26 us | 4.18 us | 36.84 us | 1.66 us | 7.14 us | 156.54x | 0.12x | 2.56x | 0.60x |
| `atan neg_1e6` | 15.27 ns | 691.66 ns | 676.16 ns | 2.88 us | 1.45 us | 19.64 us | 45.30x | 0.24x | 0.48x | 0.04x |
| `asinh neg_1e6` | 26.87 ns | 4.28 us | 4.31 us | 37.33 us | 1.71 us | 7.10 us | 159.40x | 0.11x | 2.50x | 0.60x |
| `acosh 9` | 12.71 ns | 3.76 us | 3.69 us | 42.76 us | 1.65 us | 9.85 us | 296.13x | 0.09x | 2.28x | 0.38x |
| `acosh 1_plus_1e-12` | 13.56 ns | 5.63 us | 7.14 us | 42.43 us | 8.53 us | 11.46 us | 414.77x | 0.13x | 0.66x | 0.49x |
| `acosh 1e6` | 12.71 ns | 5.42 us | 5.17 us | 38.32 us | 1.62 us | 9.97 us | 426.20x | 0.14x | 3.34x | 0.54x |
| `acosh e` | 12.86 ns | 6.21 us | 1.21 us | 42.22 us | 1.63 us | 10.74 us | 483.02x | 0.15x | 3.81x | 0.58x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 55.53 ns | 52.62 ns | 25.02 ns | 15.68 ns | 0.94 ns | 115.28x | 2.22x | 3.54x | 58.94x |
| `one` | 0.47 ns | 63.18 ns | 58.87 ns | 41.27 ns | 31.11 ns | 31.99 ns | 133.03x | 1.53x | 2.03x | 1.98x |
| `e` | 0.49 ns | 72.59 ns | 68.21 ns | 53.98 ns | 1.07 us | 217.47 ns | 149.50x | 1.34x | 0.07x | 0.33x |
| `pi` | 0.48 ns | 54.18 ns | 53.37 ns | 44.07 ns | 48.82 ns | 219.57 ns | 113.58x | 1.23x | 1.11x | 0.25x |
| `tau` | 0.48 ns | 184.53 ns | 173.27 ns | 121.84 ns | 102.39 ns | 1.87 us | 383.48x | 1.51x | 1.80x | 0.10x |
| `add` | 5.26 ns | 329.78 ns | 379.30 ns | 53.11 ns | 42.19 ns | 1.28 us | 62.72x | 6.21x | 7.82x | 0.26x |
| `sub` | 5.29 ns | 418.66 ns | 435.73 ns | 57.31 ns | 45.29 ns | 2.40 us | 79.12x | 7.31x | 9.24x | 0.17x |
| `neg` | 5.17 ns | 67.28 ns | 65.24 ns | 23.85 ns | 20.71 ns | 1.07 us | 13.02x | 2.82x | 3.25x | 0.06x |
| `mul` | 5.42 ns | 464.22 ns | 464.01 ns | 58.13 ns | 44.66 ns | 1.52 us | 85.68x | 7.99x | 10.40x | 0.31x |
| `div` | 8.16 ns | 414.44 ns | 439.35 ns | 138.88 ns | 62.25 ns | 2.51 us | 50.78x | 2.98x | 6.66x | 0.17x |
| `reciprocal` | 8.72 ns | 117.30 ns | 111.19 ns | 158.31 ns | 59.48 ns | 1.54 us | 13.46x | 0.74x | 1.97x | 0.08x |
| `reciprocal checked` | 9.11 ns | 117.00 ns | 111.99 ns | 157.67 ns | 59.71 ns | 1.51 us | 12.85x | 0.74x | 1.96x | 0.08x |
| `reciprocal checked abort` | 27.43 ns | 201.03 ns | 190.55 ns | 158.51 ns | 59.45 ns | 1.53 us | 7.33x | 1.27x | 3.38x | 0.13x |
| `pow` | 30.65 ns | 12.43 us | 10.82 us | 55.51 us | 2.87 us | 2.35 us | 405.38x | 0.22x | 4.34x | 5.29x |
| `powi` | 5.90 ns | 2.48 us | 2.81 us | 291.10 ns | 85.96 ns | 1.54 us | 419.41x | 8.51x | 28.80x | 1.61x |
| `exp` | 10.33 ns | 1.80 us | 1.78 us | 14.10 us | 922.89 ns | 1.87 us | 174.26x | 0.13x | 1.95x | 0.96x |
| `ln` | 10.77 ns | 1.59 us | 1.49 us | 30.17 us | 1.31 us | 1.82 us | 147.57x | 0.05x | 1.22x | 0.87x |
| `log10` | 13.49 ns | 2.81 us | 2.29 us | 36.05 us | 2.75 us | 6.67 us | 208.63x | 0.08x | 1.02x | 0.42x |
| `log10 abort` | 17.17 ns | 2.86 us | 2.30 us | 37.15 us | 2.75 us | 6.73 us | 166.26x | 0.08x | 1.04x | 0.42x |
| `sqrt` | 20.40 ns | 1.80 us | 1.73 us | 5.24 us | 97.99 ns | 1.45 us | 88.43x | 0.34x | 18.41x | 1.24x |
| `sin` | 14.97 ns | 1.56 us | 1.65 us | 14.44 us | 1.24 us | 2.22 us | 104.32x | 0.11x | 1.25x | 0.70x |
| `cos` | 18.63 ns | 1.34 us | 1.42 us | 12.22 us | 643.11 ns | 1.74 us | 72.08x | 0.11x | 2.09x | 0.77x |
| `tan` | 24.89 ns | 1.43 us | 1.55 us | 29.83 us | 1.58 us | 6.53 us | 57.59x | 0.05x | 0.91x | 0.22x |
| `sinh` | 18.35 ns | 3.35 us | 3.33 us | 3.32 us | 1.12 us | 10.50 us | 182.55x | 1.01x | 2.98x | 0.32x |
| `cosh` | 18.48 ns | 3.37 us | 3.30 us | 8.09 us | 1.06 us | 9.50 us | 182.50x | 0.42x | 3.17x | 0.36x |
| `tanh` | 23.39 ns | 4.84 us | 4.86 us | 3.37 us | 1.20 us | 22.64 us | 207.05x | 1.44x | 4.03x | 0.21x |
| `asin` | 13.26 ns | 7.04 us | 6.21 us | 21.56 us | 2.42 us | 13.65 us | 530.66x | 0.33x | 2.91x | 0.52x |
| `asin abort` | 17.44 ns | 7.10 us | 6.33 us | 22.35 us | 2.46 us | 13.69 us | 407.26x | 0.32x | 2.89x | 0.52x |
| `acos` | 14.58 ns | 9.42 us | 8.62 us | 27.15 us | 2.53 us | 13.86 us | 645.84x | 0.35x | 3.72x | 0.68x |
| `acos abort` | 17.89 ns | 9.32 us | 8.49 us | 27.08 us | 2.51 us | 13.79 us | 521.03x | 0.34x | 3.71x | 0.68x |
| `atan` | 15.01 ns | 928.40 ns | 1.20 us | 18.63 us | 2.25 us | 18.90 us | 61.86x | 0.05x | 0.41x | 0.05x |
| `atan abort` | 21.54 ns | 941.40 ns | 1.23 us | 18.56 us | 2.25 us | 19.13 us | 43.70x | 0.05x | 0.42x | 0.05x |
| `asinh` | 48.80 ns | 5.34 us | 6.56 us | 40.56 us | 1.63 us | 7.49 us | 109.38x | 0.13x | 3.28x | 0.71x |
| `asinh abort` | 31.69 ns | 5.35 us | 6.64 us | 40.32 us | 1.63 us | 7.47 us | 168.83x | 0.13x | 3.28x | 0.72x |
| `acosh` | 12.32 ns | 5.74 us | 7.41 us | 41.76 us | 3.33 us | 10.65 us | 465.83x | 0.14x | 1.72x | 0.54x |
| `acosh abort` | 15.55 ns | 5.72 us | 7.36 us | 41.12 us | 3.31 us | 10.51 us | 367.66x | 0.14x | 1.73x | 0.54x |
| `atanh` | 13.76 ns | 4.31 us | 4.05 us | 35.07 us | 1.26 us | 14.71 us | 313.21x | 0.12x | 3.43x | 0.29x |
| `atanh abort` | 17.05 ns | 4.35 us | 4.01 us | 35.07 us | 1.26 us | 14.89 us | 254.89x | 0.12x | 3.45x | 0.29x |
| `zero status` | 1.22 ns | 1.77 ns | 1.78 ns | 0.99 ns | 7.23 ns | 8.08 ns | 1.46x | 1.80x | 0.25x | 0.22x |
| `zero status abort` | 3.38 ns | 58.36 ns | 58.45 ns | 1.02 ns | 7.19 ns | 7.99 ns | 17.28x | 57.14x | 8.12x | 7.31x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 113.62 ns | 113.17 ns | 47.27 ns | - | 1.89 ns | 120.82x | 2.40x | - | 60.00x |
| `one` | 5.15 ns | 115.75 ns | 115.56 ns | 64.06 ns | - | 29.79 ns | 22.49x | 1.81x | - | 3.89x |
| `i` | 0.94 ns | 123.18 ns | 122.75 ns | 64.33 ns | - | 32.03 ns | 130.64x | 1.91x | - | 3.85x |
| `free i` | 0.93 ns | 122.76 ns | 123.18 ns | 64.92 ns | - | 32.25 ns | 131.78x | 1.89x | - | 3.81x |
| `conjugate` | 2.13 ns | 132.84 ns | 130.36 ns | 41.36 ns | - | 1.08 us | 62.46x | 3.21x | - | 0.12x |
| `norm squared` | 5.79 ns | 1.41 us | 1.52 us | 154.04 ns | - | 4.34 us | 242.76x | 9.12x | - | 0.32x |
| `reciprocal` | 17.96 ns | 2.83 us | 2.95 us | 446.44 ns | - | 10.89 us | 157.60x | 6.34x | - | 0.26x |
| `reciprocal checked` | 14.31 ns | 2.86 us | 2.93 us | 445.19 ns | - | 10.81 us | 200.04x | 6.43x | - | 0.26x |
| `powi` | 17.50 ns | 10.18 us | 13.86 us | 1.46 us | - | 43.81 us | 581.77x | 6.97x | - | 0.23x |
| `powi checked` | 17.39 ns | 10.17 us | 13.80 us | 1.48 us | - | 43.59 us | 584.79x | 6.87x | - | 0.23x |
| `div checked` | 18.87 ns | 5.85 us | 6.43 us | 789.23 ns | - | 21.86 us | 310.01x | 7.41x | - | 0.27x |
| `div real checked` | 9.41 ns | 782.16 ns | 770.07 ns | 260.54 ns | - | 5.29 us | 83.13x | 3.00x | - | 0.15x |
| `from scalar` | 1.41 ns | 119.74 ns | 118.27 ns | 44.76 ns | - | 10.46 ns | 84.63x | 2.68x | - | 11.45x |
| `add` | 6.28 ns | 884.92 ns | 962.11 ns | 105.82 ns | - | 2.57 us | 140.85x | 8.36x | - | 0.34x |
| `sub` | 6.19 ns | 967.96 ns | 979.84 ns | 116.75 ns | - | 4.77 us | 156.41x | 8.29x | - | 0.20x |
| `neg` | 2.58 ns | 134.56 ns | 131.08 ns | 43.63 ns | - | 2.14 us | 52.10x | 3.08x | - | 0.06x |
| `mul` | 7.59 ns | 3.04 us | 3.37 us | 306.10 ns | - | 10.22 us | 399.94x | 9.92x | - | 0.30x |
| `div` | 18.32 ns | 5.96 us | 6.42 us | 791.23 ns | - | 21.60 us | 325.22x | 7.53x | - | 0.28x |
| `div real` | 10.12 ns | 775.84 ns | 761.46 ns | 260.87 ns | - | 5.16 us | 76.67x | 2.97x | - | 0.15x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.79 ns | 3.39 us | 5.73 us | 317.43 ns | 253.59 ns | 7.14 us | 498.62x | 10.67x | 13.35x | 0.47x |
| `vec3 magnitude` | 9.56 ns | 7.33 us | 10.07 us | 5.49 us | 351.75 ns | 8.78 us | 766.85x | 1.34x | 20.85x | 0.84x |
| `vec3 normalize` | 25.60 ns | 12.15 us | 12.00 us | 6.06 us | 587.69 ns | 17.11 us | 474.72x | 2.01x | 20.68x | 0.71x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.08 ns | 1.13 us | 1.54 us | 72.96 ns | 59.31 ns | 708.20 ns | 366.08x | 15.46x | 19.02x | 1.59x |
| `vec3 zero` | 1.42 ns | 244.06 ns | 245.66 ns | 60.06 ns | 31.02 ns | 2.84 ns | 172.17x | 4.06x | 7.87x | 85.91x |
| `vec3 dot abort` | 27.96 ns | 3.84 us | 4.53 us | 255.46 ns | 207.34 ns | 7.20 us | 137.39x | 15.04x | 18.53x | 0.53x |
| `vec3 magnitude abort` | 39.40 ns | 7.82 us | 7.33 us | 5.46 us | 325.52 ns | 8.73 us | 198.46x | 1.43x | 24.02x | 0.90x |
| `vec3 normalize checked` | 26.01 ns | 12.27 us | 9.22 us | 6.01 us | 554.62 ns | 16.99 us | 471.90x | 2.04x | 22.13x | 0.72x |
| `vec3 normalize checked abort` | 55.06 ns | 12.51 us | 9.98 us | 6.03 us | 548.34 ns | 17.12 us | 227.30x | 2.07x | 22.82x | 0.73x |
| `vec3 div scalar checked` | 16.11 ns | 1.73 us | 1.71 us | 391.21 ns | - | - | 107.51x | 4.43x | - | - |
| `vec3 div scalar checked abort` | 17.95 ns | 1.72 us | 1.65 us | 391.05 ns | - | - | 95.70x | 4.39x | - | - |
| `vec3 add` | 6.67 ns | 1.93 us | 1.92 us | 148.78 ns | 128.51 ns | 4.05 us | 290.18x | 13.00x | 15.06x | 0.48x |
| `vec3 add scalar` | 6.37 ns | 1.56 us | 1.62 us | 153.39 ns | 133.90 ns | 3.79 us | 244.22x | 10.15x | 11.63x | 0.41x |
| `vec3 sub` | 6.73 ns | 2.01 us | 1.98 us | 167.52 ns | 136.22 ns | 7.34 us | 298.95x | 12.01x | 14.77x | 0.27x |
| `vec3 sub scalar` | 6.48 ns | 1.36 us | 1.47 us | 147.38 ns | 126.73 ns | 7.03 us | 209.83x | 9.22x | 10.73x | 0.19x |
| `vec3 neg` | 3.77 ns | 235.58 ns | 242.50 ns | 56.38 ns | 47.58 ns | 3.11 us | 62.55x | 4.18x | 4.95x | 0.08x |
| `vec3 mul scalar` | 6.95 ns | 1.96 us | 2.14 us | 163.46 ns | 119.40 ns | 4.45 us | 282.28x | 12.00x | 16.44x | 0.44x |
| `vec3 div scalar` | 10.15 ns | 1.66 us | 1.66 us | 408.88 ns | 174.00 ns | 7.57 us | 164.04x | 4.07x | 9.57x | 0.22x |
| `vec4 dot` | 7.27 ns | 3.51 us | 3.42 us | 431.99 ns | 323.33 ns | 9.87 us | 483.22x | 8.13x | 10.86x | 0.36x |
| `vec4 magnitude` | 12.89 ns | 7.15 us | 5.91 us | 5.64 us | 411.89 ns | 11.41 us | 554.20x | 1.27x | 17.35x | 0.63x |
| `vec4 normalize` | 36.31 ns | 12.05 us | 7.81 us | 6.14 us | 713.44 ns | 22.15 us | 331.77x | 1.96x | 16.89x | 0.54x |
| `vec4 add` | 7.85 ns | 2.23 us | 2.13 us | 208.99 ns | 181.65 ns | 5.33 us | 284.65x | 10.69x | 12.30x | 0.42x |
| `vec4 add scalar` | 6.99 ns | 2.04 us | 1.95 us | 214.61 ns | 183.85 ns | 5.09 us | 291.27x | 9.48x | 11.07x | 0.40x |
| `vec4 sub` | 5.25 ns | 2.15 us | 2.06 us | 211.23 ns | 181.94 ns | 9.66 us | 409.84x | 10.18x | 11.82x | 0.22x |
| `vec4 sub scalar` | 4.46 ns | 1.84 us | 1.71 us | 206.29 ns | 174.05 ns | 9.51 us | 413.09x | 8.94x | 10.59x | 0.19x |
| `vec4 neg` | 4.96 ns | 295.91 ns | 295.36 ns | 77.69 ns | 68.09 ns | 4.03 us | 59.66x | 3.81x | 4.35x | 0.07x |
| `vec4 mul scalar` | 7.40 ns | 2.19 us | 2.24 us | 225.90 ns | 161.17 ns | 5.69 us | 296.37x | 9.71x | 13.61x | 0.39x |
| `vec4 div scalar` | 14.31 ns | 1.96 us | 1.76 us | 537.82 ns | 228.67 ns | 10.04 us | 136.90x | 3.64x | 8.57x | 0.20x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.26 ns | 7.14 us | 3.34 us | 962.40 ns | 868.06 ns | 22.52 us | 582.60x | 7.42x | 8.23x | 0.32x |
| `mat3 inverse` | 80.31 ns | 28.87 us | 11.28 us | 3.22 us | 2.50 us | 83.11 us | 359.54x | 8.98x | 11.55x | 0.35x |
| `mat3 mul mat3` | 73.88 ns | 19.94 us | 12.38 us | 2.88 us | 2.51 us | 61.35 us | 269.94x | 6.92x | 7.95x | 0.33x |
| `mat3 transform vec3` | 16.07 ns | 9.96 us | 10.80 us | 1.05 us | 899.34 ns | 20.15 us | 619.36x | 9.51x | 11.07x | 0.49x |
| `mat4 determinant` | 45.59 ns | 12.54 us | 3.98 us | 4.71 us | 4.22 us | 94.25 us | 275.18x | 2.67x | 2.97x | 0.13x |
| `mat4 inverse` | 144.60 ns | 65.42 us | 19.32 us | 11.17 us | 9.32 us | 343.17 us | 452.40x | 5.86x | 7.02x | 0.19x |
| `mat4 mul mat4` | 116.78 ns | 25.19 us | 15.09 us | 6.55 us | 5.46 us | 143.67 us | 215.67x | 3.84x | 4.62x | 0.18x |
| `mat4 transform vec4` | 25.63 ns | 12.64 us | 4.52 us | 1.94 us | 1.69 us | 35.27 us | 493.28x | 6.51x | 7.48x | 0.36x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.10 ns | 2.96 us | 4.79 us | 219.37 ns | 239.01 ns | 2.04 us | 84.36x | 13.50x | 12.39x | 1.45x |
| `mat3 zero` | 16.09 ns | 866.35 ns | 865.15 ns | 279.36 ns | 210.00 ns | 11.49 ns | 53.86x | 3.10x | 4.13x | 75.38x |
| `mat3 identity` | 10.17 ns | 940.21 ns | 934.96 ns | 339.87 ns | 247.78 ns | 156.45 ns | 92.45x | 2.77x | 3.79x | 6.01x |
| `mat3 transpose` | 9.65 ns | 888.95 ns | 874.64 ns | 237.31 ns | 210.84 ns | 114.69 ns | 92.08x | 3.75x | 4.22x | 7.75x |
| `mat3 reciprocal` | 80.19 ns | 28.87 us | 35.37 us | 2.96 us | 2.29 us | 82.59 us | 360.04x | 9.76x | 12.62x | 0.35x |
| `mat3 reciprocal checked` | 80.16 ns | 28.96 us | 35.88 us | 2.96 us | 2.28 us | 82.97 us | 361.32x | 9.77x | 12.71x | 0.35x |
| `mat3 inverse checked` | 79.99 ns | 29.15 us | 36.87 us | 3.00 us | 2.32 us | 82.78 us | 364.43x | 9.70x | 12.58x | 0.35x |
| `mat3 inverse checked abort` | 117.58 ns | 29.17 us | 37.03 us | 3.00 us | 2.29 us | 83.60 us | 248.10x | 9.74x | 12.73x | 0.35x |
| `mat3 powi` | 146.25 ns | 60.41 us | 87.86 us | 6.98 us | 6.40 us | 151.82 us | 413.04x | 8.66x | 9.43x | 0.40x |
| `mat3 powi checked` | 145.31 ns | 60.35 us | 87.78 us | 6.87 us | 6.46 us | 151.41 us | 415.33x | 8.79x | 9.34x | 0.40x |
| `mat3 powi checked abort` | 147.96 ns | 59.86 us | 88.82 us | 6.96 us | 6.35 us | 152.00 us | 404.58x | 8.60x | 9.43x | 0.39x |
| `mat3 div scalar checked` | 25.09 ns | 5.35 us | 6.02 us | 1.47 us | 804.42 ns | 21.90 us | 213.13x | 3.64x | 6.65x | 0.24x |
| `mat3 div scalar checked abort` | 30.50 ns | 5.40 us | 5.97 us | 1.46 us | 822.10 ns | 22.35 us | 177.10x | 3.70x | 6.57x | 0.24x |
| `mat3 div matrix checked` | 190.31 ns | 68.45 us | 66.31 us | 5.53 us | 4.48 us | 160.63 us | 359.67x | 12.38x | 15.27x | 0.43x |
| `mat3 div matrix checked abort` | 215.25 ns | 67.96 us | 66.71 us | 5.52 us | 4.47 us | 159.48 us | 315.72x | 12.32x | 15.19x | 0.43x |
| `mat3 add` | 14.56 ns | 5.21 us | 6.04 us | 521.40 ns | 508.19 ns | 11.70 us | 357.71x | 9.99x | 10.25x | 0.45x |
| `mat3 add scalar` | 12.15 ns | 3.94 us | 5.46 us | 784.73 ns | 710.27 ns | 12.12 us | 324.55x | 5.03x | 5.55x | 0.33x |
| `mat3 sub` | 14.31 ns | 5.24 us | 5.98 us | 544.83 ns | 521.05 ns | 21.03 us | 366.30x | 9.62x | 10.06x | 0.25x |
| `mat3 sub scalar` | 11.35 ns | 3.83 us | 5.31 us | 810.38 ns | 735.15 ns | 21.75 us | 337.69x | 4.73x | 5.21x | 0.18x |
| `mat3 neg` | 11.04 ns | 1.01 us | 1.03 us | 487.91 ns | 463.36 ns | 8.57 us | 91.33x | 2.07x | 2.18x | 0.12x |
| `mat3 mul scalar` | 13.91 ns | 5.49 us | 6.46 us | 802.29 ns | 682.00 ns | 12.21 us | 394.69x | 6.84x | 8.05x | 0.45x |
| `mat3 div scalar` | 25.53 ns | 5.39 us | 6.16 us | 1.47 us | 815.41 ns | 21.98 us | 211.08x | 3.67x | 6.61x | 0.25x |
| `mat3 div matrix` | 148.29 ns | 67.26 us | 67.84 us | 5.53 us | 4.42 us | 158.92 us | 453.58x | 12.16x | 15.21x | 0.42x |
| `mat3 bitxor` | 145.50 ns | 59.69 us | 88.81 us | 6.93 us | 6.41 us | 150.49 us | 410.26x | 8.61x | 9.32x | 0.40x |
| `mat4 zero` | 13.48 ns | 1.34 us | 1.39 us | 494.19 ns | 354.04 ns | 14.09 ns | 99.77x | 2.72x | 3.80x | 95.43x |
| `mat4 identity` | 10.83 ns | 1.51 us | 1.54 us | 547.04 ns | 415.99 ns | 228.77 ns | 139.47x | 2.76x | 3.63x | 6.60x |
| `mat4 transpose` | 10.19 ns | 1.60 us | 1.65 us | 429.28 ns | 376.35 ns | 173.45 ns | 157.09x | 3.73x | 4.25x | 9.23x |
| `mat4 reciprocal` | 144.66 ns | 61.18 us | 63.33 us | 11.12 us | 9.01 us | 339.47 us | 422.89x | 5.50x | 6.79x | 0.18x |
| `mat4 reciprocal checked` | 167.92 ns | 61.49 us | 64.06 us | 10.90 us | 8.79 us | 340.78 us | 366.21x | 5.64x | 7.00x | 0.18x |
| `mat4 powi` | 244.58 ns | 89.51 us | 111.28 us | 15.80 us | 13.92 us | 351.62 us | 365.96x | 5.67x | 6.43x | 0.25x |
| `mat4 powi checked` | 243.70 ns | 89.48 us | 113.22 us | 15.96 us | 13.89 us | 354.07 us | 367.20x | 5.61x | 6.44x | 0.25x |
| `mat4 add` | 52.78 ns | 7.29 us | 7.81 us | 950.15 ns | 868.09 ns | 19.37 us | 138.20x | 7.68x | 8.40x | 0.38x |
| `mat4 add scalar` | 21.25 ns | 5.64 us | 7.64 us | 1.43 us | 1.20 us | 20.71 us | 265.66x | 3.95x | 4.69x | 0.27x |
| `mat4 sub` | 39.24 ns | 7.57 us | 7.76 us | 977.50 ns | 906.83 ns | 35.56 us | 193.02x | 7.75x | 8.35x | 0.21x |
| `mat4 sub scalar` | 15.02 ns | 5.60 us | 7.51 us | 1.46 us | 1.18 us | 37.99 us | 372.75x | 3.84x | 4.74x | 0.15x |
| `mat4 neg` | 13.83 ns | 1.69 us | 1.71 us | 941.09 ns | 757.28 ns | 14.67 us | 122.02x | 1.79x | 2.23x | 0.12x |
| `mat4 mul scalar` | 23.78 ns | 7.27 us | 8.35 us | 1.44 us | 1.14 us | 20.71 us | 305.93x | 5.04x | 6.38x | 0.35x |
| `mat4 div scalar` | 33.50 ns | 8.44 us | 8.74 us | 2.67 us | 1.39 us | 37.87 us | 251.93x | 3.16x | 6.06x | 0.22x |
| `mat4 div matrix` | 214.38 ns | 132.80 us | 108.24 us | 16.89 us | 14.09 us | 537.78 us | 619.46x | 7.86x | 9.43x | 0.25x |
| `mat4 bitxor` | 242.64 ns | 90.34 us | 114.44 us | 16.58 us | 13.86 us | 354.70 us | 372.29x | 5.45x | 6.52x | 0.25x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 9.11 ns | 603.69 ns | 373.29 ns | - | - | 1.32 us | 66.25x | - | - | 0.46x |
| `scalar add ref_owned` | 5.51 ns | 611.75 ns | 361.30 ns | - | - | 1.30 us | 110.96x | - | - | 0.47x |
| `scalar add refs` | 5.46 ns | 245.05 ns | 300.25 ns | - | - | 1.30 us | 44.90x | - | - | 0.19x |
| `scalar sub owned_ref` | 9.16 ns | 691.13 ns | 442.03 ns | - | - | 2.47 us | 75.44x | - | - | 0.28x |
| `scalar sub ref_owned` | 5.78 ns | 690.08 ns | 438.15 ns | - | - | 2.49 us | 119.42x | - | - | 0.28x |
| `scalar sub refs` | 5.69 ns | 321.03 ns | 366.23 ns | - | - | 2.43 us | 56.42x | - | - | 0.13x |
| `scalar mul owned_ref` | 9.74 ns | 726.38 ns | 443.91 ns | - | - | 1.54 us | 74.57x | - | - | 0.47x |
| `scalar mul ref_owned` | 6.20 ns | 736.10 ns | 428.87 ns | - | - | 1.55 us | 118.78x | - | - | 0.47x |
| `scalar mul refs` | 6.45 ns | 369.10 ns | 367.42 ns | - | - | 1.54 us | 57.25x | - | - | 0.24x |
| `scalar div owned_ref` | 15.51 ns | 635.35 ns | 380.15 ns | - | - | 2.56 us | 40.96x | - | - | 0.25x |
| `scalar div ref_owned` | 15.96 ns | 645.57 ns | 405.33 ns | - | - | 2.59 us | 40.46x | - | - | 0.25x |
| `scalar div refs` | 6.96 ns | 286.45 ns | 320.43 ns | - | - | 2.55 us | 41.16x | - | - | 0.11x |
| `vec3 add refs` | 6.31 ns | 1.84 us | 1.79 us | - | - | 3.94 us | 292.10x | - | - | 0.47x |
| `vec3 sub refs` | 6.35 ns | 1.86 us | 1.84 us | - | - | 7.39 us | 293.71x | - | - | 0.25x |
| `vec3 neg ref` | 3.30 ns | 576.16 ns | 542.27 ns | - | - | 3.20 us | 174.39x | - | - | 0.18x |
| `vec3 add_scalar_ref` | 6.63 ns | 1.94 us | 1.92 us | - | - | 3.80 us | 292.61x | - | - | 0.51x |
| `vec3 sub_scalar_ref` | 6.64 ns | 1.76 us | 1.70 us | - | - | 7.21 us | 265.57x | - | - | 0.24x |
| `vec3 mul_scalar_ref` | 7.01 ns | 2.29 us | 2.42 us | - | - | 4.41 us | 326.48x | - | - | 0.52x |
| `vec3 div_scalar_ref` | 9.61 ns | 2.09 us | 2.08 us | - | - | 7.61 us | 217.56x | - | - | 0.27x |
| `vec4 add refs` | 6.52 ns | 1.93 us | 1.79 us | - | - | 5.30 us | 296.66x | - | - | 0.37x |
| `vec4 sub refs` | 3.11 ns | 1.85 us | 1.72 us | - | - | 9.59 us | 593.67x | - | - | 0.19x |
| `vec4 neg ref` | 4.27 ns | 649.55 ns | 662.57 ns | - | - | 4.08 us | 152.21x | - | - | 0.16x |
| `vec4 add_scalar_ref` | 7.09 ns | 2.36 us | 2.22 us | - | - | 5.20 us | 333.56x | - | - | 0.45x |
| `vec4 sub_scalar_ref` | 4.25 ns | 2.13 us | 1.99 us | - | - | 9.49 us | 500.90x | - | - | 0.22x |
| `vec4 mul_scalar_ref` | 7.56 ns | 2.47 us | 2.45 us | - | - | 5.71 us | 326.68x | - | - | 0.43x |
| `vec4 div_scalar_ref` | 13.33 ns | 2.34 us | 2.10 us | - | - | 10.15 us | 175.43x | - | - | 0.23x |
| `mat3 add refs` | 11.37 ns | 3.56 us | 4.41 us | - | - | 11.65 us | 313.41x | - | - | 0.31x |
| `mat3 sub refs` | 10.72 ns | 3.67 us | 4.29 us | - | - | 21.17 us | 342.15x | - | - | 0.17x |
| `mat3 mul refs` | 57.73 ns | 18.96 us | 29.55 us | - | - | 62.89 us | 328.43x | - | - | 0.30x |
| `mat3 div refs` | 152.94 ns | 68.47 us | 67.64 us | - | - | 161.89 us | 447.69x | - | - | 0.42x |
| `mat3 neg ref` | 10.23 ns | 1.05 us | 1.05 us | - | - | 8.66 us | 102.96x | - | - | 0.12x |
| `mat3 add_scalar_ref` | 10.38 ns | 4.12 us | 5.44 us | - | - | 12.06 us | 396.69x | - | - | 0.34x |
| `mat3 sub_scalar_ref` | 13.18 ns | 3.99 us | 5.14 us | - | - | 21.41 us | 302.67x | - | - | 0.19x |
| `mat3 mul_scalar_ref` | 11.81 ns | 5.67 us | 6.38 us | - | - | 12.29 us | 479.83x | - | - | 0.46x |
| `mat3 div_scalar_ref` | 23.95 ns | 5.95 us | 6.45 us | - | - | 22.53 us | 248.38x | - | - | 0.26x |
| `mat4 add refs` | 16.80 ns | 4.02 us | 4.23 us | - | - | 19.56 us | 239.62x | - | - | 0.21x |
| `mat4 sub refs` | 16.69 ns | 4.30 us | 4.41 us | - | - | 36.08 us | 257.97x | - | - | 0.12x |
| `mat4 mul refs` | 101.84 ns | 22.28 us | 29.81 us | - | - | 143.91 us | 218.73x | - | - | 0.15x |
| `mat4 div refs` | 212.91 ns | 135.56 us | 110.22 us | - | - | 537.74 us | 636.69x | - | - | 0.25x |
| `mat4 neg ref` | 12.59 ns | 1.50 us | 1.60 us | - | - | 14.48 us | 119.08x | - | - | 0.10x |
| `mat4 add_scalar_ref` | 14.45 ns | 6.24 us | 8.31 us | - | - | 20.60 us | 431.66x | - | - | 0.30x |
| `mat4 sub_scalar_ref` | 15.46 ns | 6.19 us | 8.38 us | - | - | 37.27 us | 400.03x | - | - | 0.17x |
| `mat4 mul_scalar_ref` | 49.81 ns | 7.79 us | 8.55 us | - | - | 20.73 us | 156.30x | - | - | 0.38x |
| `mat4 div_scalar_ref` | 30.41 ns | 8.80 us | 9.03 us | - | - | 38.01 us | 289.32x | - | - | 0.23x |
| `mat3 transform_vec refs` | 14.60 ns | 9.23 us | 11.82 us | - | - | 20.11 us | 632.17x | - | - | 0.46x |
| `mat4 transform_vec refs` | 23.97 ns | 11.86 us | 11.74 us | - | - | 35.69 us | 494.84x | - | - | 0.33x |
| `complex add refs` | 7.88 ns | 985.93 ns | 1.07 us | - | - | 2.65 us | 125.18x | - | - | 0.37x |
| `complex sub refs` | 8.09 ns | 1.07 us | 1.13 us | - | - | 4.80 us | 132.35x | - | - | 0.22x |
| `complex mul refs` | 8.06 ns | 3.19 us | 3.54 us | - | - | 10.06 us | 396.37x | - | - | 0.32x |
| `complex div refs` | 17.99 ns | 6.12 us | 6.76 us | - | - | 22.21 us | 340.25x | - | - | 0.28x |
| `complex neg ref` | 2.40 ns | 399.02 ns | 401.05 ns | - | - | 2.16 us | 166.25x | - | - | 0.19x |
| `complex div_real_ref` | 10.16 ns | 1.10 us | 1.07 us | - | - | 5.19 us | 108.71x | - | - | 0.21x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.60 us |
| `astro sin 160` | 13.91 us |
| `astro sin 192` | 13.71 us |
| `astro sin 256` | 16.02 us |
