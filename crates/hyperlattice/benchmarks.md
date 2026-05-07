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
| `sin 0.1` | 11.08 ns | 218.02 ns | 216.07 ns | 10.96 us | 777.78 ns | 1.91 us | 19.68x | 0.02x | 0.28x | 0.11x |
| `cos 0.1` | 11.78 ns | 216.46 ns | 216.33 ns | 10.93 us | 498.31 ns | 1.66 us | 18.38x | 0.02x | 0.43x | 0.13x |
| `sin 1.23456789` | 11.86 ns | 304.36 ns | 289.84 ns | 12.78 us | 818.13 ns | 1.86 us | 25.67x | 0.02x | 0.37x | 0.16x |
| `cos 1.23456789` | 12.26 ns | 294.97 ns | 271.31 ns | 10.94 us | 614.35 ns | 1.64 us | 24.07x | 0.03x | 0.48x | 0.18x |
| `sin 1e6` | 12.87 ns | 6.14 us | 6.20 us | 16.13 us | 1.15 us | 2.11 us | 477.36x | 0.38x | 5.36x | 2.91x |
| `cos 1e6` | 12.50 ns | 6.10 us | 6.07 us | 13.81 us | 896.59 ns | 1.85 us | 488.11x | 0.44x | 6.81x | 3.31x |
| `sin 1e30` | 67.34 ns | 8.18 us | 8.39 us | 18.60 us | 2.90 us | 3.69 us | 121.53x | 0.44x | 2.82x | 2.22x |
| `cos 1e30` | 72.93 ns | 8.43 us | 8.33 us | 15.49 us | 984.22 ns | 3.12 us | 115.56x | 0.54x | 8.56x | 2.70x |
| `sin pi_7` | 11.89 ns | 218.58 ns | 543.55 ns | 12.17 us | 743.81 ns | 1.94 us | 18.38x | 0.02x | 0.29x | 0.11x |
| `cos pi_7` | 11.83 ns | 216.43 ns | 969.88 ns | 11.03 us | 541.27 ns | 1.72 us | 18.30x | 0.02x | 0.40x | 0.13x |
| `sin 1000pi_eps` | 11.85 ns | 6.10 us | 4.31 us | 16.09 us | 2.30 us | 2.95 us | 514.69x | 0.38x | 2.65x | 2.07x |
| `cos 1000pi_eps` | 12.29 ns | 6.00 us | 4.26 us | 13.78 us | 587.47 ns | 1.77 us | 488.34x | 0.44x | 10.21x | 3.40x |
| `asin 0.5` | 10.94 ns | 553.60 ns | 554.14 ns | 49.27 us | 2.95 us | 13.12 us | 50.60x | 0.01x | 0.19x | 0.04x |
| `acos 0.5` | 11.44 ns | 1.20 us | 1.16 us | 59.64 us | 3.05 us | 13.55 us | 104.62x | 0.02x | 0.39x | 0.09x |
| `atanh 0.5` | 14.94 ns | 1.55 us | 1.51 us | 35.12 us | 1.74 us | 13.17 us | 103.95x | 0.04x | 0.89x | 0.12x |
| `asin neg_0.999999` | 14.53 ns | 5.90 us | 4.76 us | 13.95 us | 2.55 us | 12.97 us | 405.72x | 0.42x | 2.32x | 0.45x |
| `acos neg_0.999999` | 15.74 ns | 6.08 us | 5.34 us | 18.63 us | 2.69 us | 13.36 us | 386.45x | 0.33x | 2.26x | 0.46x |
| `atanh neg_0.999999` | 15.12 ns | 4.29 us | 3.72 us | 36.80 us | 1.67 us | 12.79 us | 283.71x | 0.12x | 2.57x | 0.34x |
| `asin 0.999999` | 14.06 ns | 5.88 us | 4.89 us | 13.99 us | 2.52 us | 12.65 us | 418.26x | 0.42x | 2.33x | 0.46x |
| `acos 0.999999` | 14.53 ns | 5.69 us | 5.05 us | 18.73 us | 2.71 us | 13.05 us | 391.74x | 0.30x | 2.10x | 0.44x |
| `atanh 0.999999` | 14.79 ns | 4.19 us | 3.59 us | 31.93 us | 1.64 us | 12.53 us | 283.62x | 0.13x | 2.55x | 0.33x |
| `asin 1e-12` | 9.58 ns | 523.59 ns | 1.18 us | 8.14 us | 1.51 us | 15.65 us | 54.63x | 0.06x | 0.35x | 0.03x |
| `acos 1e-12` | 10.23 ns | 892.51 ns | 1.97 us | 9.83 us | 1.46 us | 15.37 us | 87.21x | 0.09x | 0.61x | 0.06x |
| `atanh 1e-12` | 9.81 ns | 499.36 ns | 925.22 ns | 37.02 us | 172.60 ns | 21.47 us | 50.91x | 0.01x | 2.89x | 0.02x |
| `atan 0.5` | 14.85 ns | 476.61 ns | 460.89 ns | 36.09 us | 2.78 us | 17.86 us | 32.09x | 0.01x | 0.17x | 0.03x |
| `asinh 0.5` | 26.94 ns | 1.73 us | 1.69 us | 39.51 us | 1.65 us | 7.40 us | 64.24x | 0.04x | 1.05x | 0.23x |
| `atan neg_1e-12` | 14.77 ns | 429.13 ns | 621.97 ns | 1.68 us | 1.11 us | 15.12 us | 29.06x | 0.25x | 0.39x | 0.03x |
| `asinh neg_1e-12` | 15.99 ns | 5.61 us | 4.53 us | 42.69 us | 8.66 us | 11.92 us | 350.84x | 0.13x | 0.65x | 0.47x |
| `atan 1e6` | 14.97 ns | 413.87 ns | 412.54 ns | 2.90 us | 1.45 us | 18.50 us | 27.66x | 0.14x | 0.29x | 0.02x |
| `asinh 1e6` | 27.40 ns | 3.38 us | 3.43 us | 37.13 us | 1.70 us | 7.18 us | 123.46x | 0.09x | 1.99x | 0.47x |
| `atan neg_1e6` | 15.37 ns | 504.65 ns | 511.86 ns | 3.02 us | 1.46 us | 18.57 us | 32.83x | 0.17x | 0.34x | 0.03x |
| `asinh neg_1e6` | 26.77 ns | 3.44 us | 3.46 us | 36.85 us | 1.71 us | 7.09 us | 128.39x | 0.09x | 2.00x | 0.49x |
| `acosh 9` | 12.46 ns | 2.93 us | 2.95 us | 43.49 us | 1.70 us | 10.04 us | 234.86x | 0.07x | 1.73x | 0.29x |
| `acosh 1_plus_1e-12` | 13.01 ns | 3.82 us | 5.38 us | 41.88 us | 8.43 us | 12.12 us | 293.84x | 0.09x | 0.45x | 0.32x |
| `acosh 1e6` | 12.43 ns | 3.76 us | 3.72 us | 37.02 us | 1.67 us | 9.75 us | 302.43x | 0.10x | 2.25x | 0.39x |
| `acosh e` | 12.63 ns | 4.09 us | 4.08 us | 41.23 us | 1.67 us | 9.83 us | 323.52x | 0.10x | 2.45x | 0.42x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.49 ns | 57.17 ns | 52.86 ns | 24.60 ns | 15.78 ns | 0.98 ns | 115.74x | 2.32x | 3.62x | 58.58x |
| `one` | 0.49 ns | 66.40 ns | 62.77 ns | 44.42 ns | 31.50 ns | 32.00 ns | 136.49x | 1.49x | 2.11x | 2.08x |
| `e` | 0.49 ns | 76.07 ns | 73.23 ns | 55.49 ns | 1.07 us | 224.80 ns | 154.12x | 1.37x | 0.07x | 0.34x |
| `pi` | 0.49 ns | 59.40 ns | 55.50 ns | 44.73 ns | 49.22 ns | 224.40 ns | 120.66x | 1.33x | 1.21x | 0.26x |
| `tau` | 0.48 ns | 205.24 ns | 189.03 ns | 119.18 ns | 102.74 ns | 1.84 us | 425.04x | 1.72x | 2.00x | 0.11x |
| `add` | 5.35 ns | 289.50 ns | 379.59 ns | 54.53 ns | 42.37 ns | 1.25 us | 54.12x | 5.31x | 6.83x | 0.23x |
| `sub` | 5.46 ns | 316.95 ns | 446.97 ns | 57.39 ns | 45.31 ns | 2.38 us | 58.01x | 5.52x | 7.00x | 0.13x |
| `neg` | 5.33 ns | 64.56 ns | 61.92 ns | 23.85 ns | 20.45 ns | 1.06 us | 12.12x | 2.71x | 3.16x | 0.06x |
| `mul` | 5.71 ns | 209.17 ns | 475.08 ns | 59.25 ns | 44.36 ns | 1.49 us | 36.65x | 3.53x | 4.72x | 0.14x |
| `div` | 8.06 ns | 403.76 ns | 462.71 ns | 139.79 ns | 62.49 ns | 2.50 us | 50.08x | 2.89x | 6.46x | 0.16x |
| `reciprocal` | 9.05 ns | 135.25 ns | 122.25 ns | 157.01 ns | 59.11 ns | 1.55 us | 14.94x | 0.86x | 2.29x | 0.09x |
| `reciprocal checked` | 9.04 ns | 132.82 ns | 131.33 ns | 161.82 ns | 59.70 ns | 1.52 us | 14.70x | 0.82x | 2.22x | 0.09x |
| `reciprocal checked abort` | 25.42 ns | 131.08 ns | 128.52 ns | 158.04 ns | 59.52 ns | 1.53 us | 5.16x | 0.83x | 2.20x | 0.09x |
| `pow` | 20.37 ns | 10.81 us | 10.80 us | 54.35 us | 3.04 us | 2.35 us | 530.61x | 0.20x | 3.56x | 4.59x |
| `powi` | 6.05 ns | 563.16 ns | 2.84 us | 300.23 ns | 86.43 ns | 1.55 us | 93.15x | 1.88x | 6.52x | 0.36x |
| `exp` | 19.74 ns | 1.95 us | 3.10 us | 14.09 us | 930.83 ns | 1.83 us | 98.96x | 0.14x | 2.10x | 1.07x |
| `ln` | 10.75 ns | 1.58 us | 1.59 us | 30.67 us | 1.36 us | 1.78 us | 146.77x | 0.05x | 1.16x | 0.89x |
| `log10` | 13.94 ns | 2.73 us | 2.50 us | 35.80 us | 2.82 us | 6.66 us | 196.18x | 0.08x | 0.97x | 0.41x |
| `log10 abort` | 19.51 ns | 2.76 us | 2.51 us | 36.01 us | 2.82 us | 6.67 us | 141.48x | 0.08x | 0.98x | 0.41x |
| `sqrt` | 8.36 ns | 1.59 us | 1.73 us | 5.18 us | 96.00 ns | 1.46 us | 190.59x | 0.31x | 16.59x | 1.09x |
| `sin` | 15.43 ns | 3.20 us | 3.75 us | 14.04 us | 1.26 us | 2.26 us | 207.46x | 0.23x | 2.53x | 1.42x |
| `cos` | 18.65 ns | 3.24 us | 3.73 us | 12.12 us | 627.12 ns | 1.75 us | 173.90x | 0.27x | 5.17x | 1.86x |
| `tan` | 24.85 ns | 3.01 us | 3.51 us | 30.05 us | 1.61 us | 6.66 us | 121.21x | 0.10x | 1.88x | 0.45x |
| `sinh` | 18.76 ns | 4.55 us | 4.80 us | 3.36 us | 1.13 us | 10.69 us | 242.43x | 1.35x | 4.03x | 0.43x |
| `cosh` | 19.07 ns | 4.46 us | 4.67 us | 8.09 us | 1.08 us | 9.46 us | 233.88x | 0.55x | 4.14x | 0.47x |
| `tanh` | 27.24 ns | 5.97 us | 6.30 us | 3.35 us | 1.19 us | 22.64 us | 218.98x | 1.78x | 5.00x | 0.26x |
| `asin` | 13.59 ns | 3.18 us | 4.92 us | 21.63 us | 2.47 us | 13.70 us | 234.17x | 0.15x | 1.29x | 0.23x |
| `asin abort` | 17.59 ns | 3.19 us | 4.86 us | 22.19 us | 2.44 us | 13.69 us | 181.29x | 0.14x | 1.31x | 0.23x |
| `acos` | 14.26 ns | 3.48 us | 5.70 us | 26.88 us | 2.54 us | 13.64 us | 243.88x | 0.13x | 1.37x | 0.25x |
| `acos abort` | 17.68 ns | 3.54 us | 5.76 us | 26.88 us | 2.61 us | 13.70 us | 200.36x | 0.13x | 1.36x | 0.26x |
| `atan` | 15.11 ns | 467.01 ns | 1.17 us | 18.33 us | 2.29 us | 19.07 us | 30.91x | 0.03x | 0.20x | 0.02x |
| `atan abort` | 21.42 ns | 477.28 ns | 1.23 us | 18.43 us | 2.28 us | 19.58 us | 22.28x | 0.03x | 0.21x | 0.02x |
| `asinh` | 35.06 ns | 3.68 us | 5.67 us | 40.04 us | 1.70 us | 7.40 us | 104.94x | 0.09x | 2.17x | 0.50x |
| `asinh abort` | 32.30 ns | 3.68 us | 5.71 us | 39.98 us | 1.66 us | 7.47 us | 113.99x | 0.09x | 2.22x | 0.49x |
| `acosh` | 12.68 ns | 3.91 us | 5.59 us | 41.11 us | 3.48 us | 10.54 us | 308.38x | 0.10x | 1.12x | 0.37x |
| `acosh abort` | 16.47 ns | 3.75 us | 5.53 us | 40.19 us | 3.41 us | 10.38 us | 227.97x | 0.09x | 1.10x | 0.36x |
| `atanh` | 14.16 ns | 2.78 us | 3.58 us | 35.19 us | 1.32 us | 14.92 us | 196.57x | 0.08x | 2.11x | 0.19x |
| `atanh abort` | 17.13 ns | 2.71 us | 3.39 us | 35.52 us | 1.30 us | 14.86 us | 158.01x | 0.08x | 2.08x | 0.18x |
| `zero status` | 1.22 ns | 1.91 ns | 1.91 ns | 0.99 ns | 6.78 ns | 8.50 ns | 1.56x | 1.92x | 0.28x | 0.22x |
| `zero status abort` | 1.44 ns | 3.35 ns | 3.46 ns | 0.97 ns | 6.93 ns | 8.40 ns | 2.32x | 3.46x | 0.48x | 0.40x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 109.51 ns | 106.48 ns | 47.69 ns | - | 1.90 ns | 115.94x | 2.30x | - | 57.50x |
| `one` | 5.09 ns | 112.99 ns | 113.94 ns | 63.51 ns | - | 30.76 ns | 22.21x | 1.78x | - | 3.67x |
| `i` | 0.96 ns | 115.90 ns | 118.93 ns | 64.36 ns | - | 32.87 ns | 120.38x | 1.80x | - | 3.53x |
| `free i` | 1.01 ns | 115.99 ns | 118.19 ns | 64.67 ns | - | 32.73 ns | 114.81x | 1.79x | - | 3.54x |
| `conjugate` | 2.17 ns | 138.39 ns | 140.17 ns | 41.78 ns | - | 1.09 us | 63.91x | 3.31x | - | 0.13x |
| `norm squared` | 5.92 ns | 393.85 ns | 1.56 us | 149.72 ns | - | 4.39 us | 66.51x | 2.63x | - | 0.09x |
| `reciprocal` | 18.26 ns | 1.94 us | 3.14 us | 451.01 ns | - | 10.75 us | 105.99x | 4.29x | - | 0.18x |
| `reciprocal checked` | 15.48 ns | 1.98 us | 3.13 us | 456.22 ns | - | 10.59 us | 128.24x | 4.35x | - | 0.19x |
| `powi` | 17.99 ns | 2.76 us | 14.53 us | 1.43 us | - | 43.00 us | 153.47x | 1.93x | - | 0.06x |
| `powi checked` | 17.66 ns | 2.76 us | 14.05 us | 1.43 us | - | 43.42 us | 156.27x | 1.93x | - | 0.06x |
| `div checked` | 18.28 ns | 3.09 us | 6.64 us | 836.56 ns | - | 21.16 us | 169.29x | 3.70x | - | 0.15x |
| `div real checked` | 9.51 ns | 809.67 ns | 762.08 ns | 273.08 ns | - | 5.19 us | 85.18x | 2.96x | - | 0.16x |
| `from scalar` | 1.42 ns | 133.87 ns | 132.92 ns | 45.69 ns | - | 10.74 ns | 94.41x | 2.93x | - | 12.46x |
| `add` | 6.41 ns | 638.34 ns | 999.07 ns | 108.92 ns | - | 2.59 us | 99.63x | 5.86x | - | 0.25x |
| `sub` | 6.38 ns | 633.28 ns | 1.06 us | 118.76 ns | - | 4.91 us | 99.20x | 5.33x | - | 0.13x |
| `neg` | 2.60 ns | 136.92 ns | 138.85 ns | 44.00 ns | - | 2.14 us | 52.67x | 3.11x | - | 0.06x |
| `mul` | 7.73 ns | 1.11 us | 3.57 us | 308.39 ns | - | 10.01 us | 144.11x | 3.61x | - | 0.11x |
| `div` | 17.98 ns | 3.11 us | 6.78 us | 814.16 ns | - | 21.55 us | 172.75x | 3.82x | - | 0.14x |
| `div real` | 10.26 ns | 789.63 ns | 758.60 ns | 268.69 ns | - | 5.12 us | 77.00x | 2.94x | - | 0.15x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.96 ns | 849.20 ns | 6.15 us | 327.11 ns | 277.02 ns | 7.13 us | 121.98x | 2.60x | 3.07x | 0.12x |
| `vec3 magnitude` | 9.96 ns | 4.78 us | 9.02 us | 5.58 us | 372.94 ns | 8.73 us | 480.17x | 0.86x | 12.82x | 0.55x |
| `vec3 normalize` | 25.86 ns | 9.09 us | 15.20 us | 6.35 us | 594.82 ns | 16.79 us | 351.34x | 1.43x | 15.28x | 0.54x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.10 ns | 311.89 ns | 1.58 us | 71.96 ns | 59.66 ns | 739.28 ns | 100.71x | 4.33x | 5.23x | 0.42x |
| `vec3 zero` | 1.42 ns | 257.28 ns | 244.55 ns | 62.78 ns | 31.92 ns | 2.88 ns | 181.01x | 4.10x | 8.06x | 89.47x |
| `vec3 dot abort` | 28.42 ns | 1.27 us | 4.60 us | 255.41 ns | 206.59 ns | 7.23 us | 44.86x | 4.99x | 6.17x | 0.18x |
| `vec3 magnitude abort` | 39.92 ns | 5.24 us | 6.99 us | 5.53 us | 326.63 ns | 8.74 us | 131.15x | 0.95x | 16.03x | 0.60x |
| `vec3 normalize checked` | 26.71 ns | 9.00 us | 10.61 us | 5.90 us | 535.21 ns | 17.00 us | 336.90x | 1.53x | 16.81x | 0.53x |
| `vec3 normalize checked abort` | 55.18 ns | 9.26 us | 10.94 us | 6.02 us | 543.14 ns | 17.17 us | 167.80x | 1.54x | 17.05x | 0.54x |
| `vec3 div scalar checked` | 10.53 ns | 1.51 us | 1.76 us | 401.98 ns | - | - | 143.44x | 3.76x | - | - |
| `vec3 div scalar checked abort` | 18.41 ns | 1.54 us | 1.79 us | 394.27 ns | - | - | 83.54x | 3.90x | - | - |
| `vec3 add` | 6.73 ns | 1.21 us | 2.00 us | 153.05 ns | 127.40 ns | 3.87 us | 179.67x | 7.90x | 9.49x | 0.31x |
| `vec3 add scalar` | 6.60 ns | 945.62 ns | 1.66 us | 157.81 ns | 136.06 ns | 3.69 us | 143.35x | 5.99x | 6.95x | 0.26x |
| `vec3 sub` | 6.81 ns | 1.20 us | 2.06 us | 166.17 ns | 137.06 ns | 7.34 us | 176.64x | 7.24x | 8.78x | 0.16x |
| `vec3 sub scalar` | 6.49 ns | 940.12 ns | 1.46 us | 150.34 ns | 123.90 ns | 7.07 us | 144.91x | 6.25x | 7.59x | 0.13x |
| `vec3 neg` | 3.77 ns | 233.03 ns | 236.92 ns | 58.11 ns | 47.01 ns | 3.09 us | 61.86x | 4.01x | 4.96x | 0.08x |
| `vec3 mul scalar` | 7.04 ns | 682.06 ns | 2.19 us | 166.16 ns | 118.17 ns | 4.49 us | 96.83x | 4.10x | 5.77x | 0.15x |
| `vec3 div scalar` | 10.16 ns | 1.47 us | 1.75 us | 424.09 ns | 168.94 ns | 7.49 us | 144.52x | 3.46x | 8.69x | 0.20x |
| `vec4 dot` | 7.29 ns | 974.81 ns | 3.61 us | 454.59 ns | 316.34 ns | 9.66 us | 133.80x | 2.14x | 3.08x | 0.10x |
| `vec4 magnitude` | 13.06 ns | 4.77 us | 5.64 us | 5.59 us | 424.62 ns | 11.27 us | 364.94x | 0.85x | 11.22x | 0.42x |
| `vec4 normalize` | 36.08 ns | 9.51 us | 9.57 us | 6.09 us | 689.95 ns | 21.77 us | 263.70x | 1.56x | 13.79x | 0.44x |
| `vec4 add` | 8.00 ns | 1.52 us | 2.21 us | 213.66 ns | 176.36 ns | 5.28 us | 189.96x | 7.11x | 8.61x | 0.29x |
| `vec4 add scalar` | 7.01 ns | 1.20 us | 1.95 us | 220.42 ns | 179.11 ns | 5.05 us | 170.98x | 5.44x | 6.69x | 0.24x |
| `vec4 sub` | 5.41 ns | 1.47 us | 2.07 us | 221.42 ns | 178.08 ns | 9.44 us | 271.25x | 6.63x | 8.24x | 0.16x |
| `vec4 sub scalar` | 4.51 ns | 1.19 us | 1.73 us | 210.50 ns | 173.34 ns | 9.29 us | 264.20x | 5.66x | 6.88x | 0.13x |
| `vec4 neg` | 4.99 ns | 299.20 ns | 293.28 ns | 77.96 ns | 66.87 ns | 4.07 us | 59.93x | 3.84x | 4.47x | 0.07x |
| `vec4 mul scalar` | 7.48 ns | 883.12 ns | 2.31 us | 224.32 ns | 160.97 ns | 5.54 us | 118.11x | 3.94x | 5.49x | 0.16x |
| `vec4 div scalar` | 14.26 ns | 2.07 us | 1.86 us | 539.49 ns | 229.03 ns | 9.86 us | 145.06x | 3.83x | 9.03x | 0.21x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 12.68 ns | 1.91 us | 3.48 us | 959.02 ns | 872.25 ns | 22.10 us | 150.24x | 1.99x | 2.18x | 0.09x |
| `mat3 inverse` | 80.75 ns | 19.02 us | 11.95 us | 3.14 us | 2.58 us | 82.18 us | 235.51x | 6.06x | 7.38x | 0.23x |
| `mat3 mul mat3` | 73.89 ns | 8.13 us | 12.86 us | 2.87 us | 2.46 us | 60.30 us | 109.99x | 2.83x | 3.31x | 0.13x |
| `mat3 transform vec3` | 16.22 ns | 3.46 us | 11.20 us | 1.04 us | 906.99 ns | 19.78 us | 213.17x | 3.32x | 3.81x | 0.17x |
| `mat4 determinant` | 45.60 ns | 5.49 us | 4.39 us | 4.64 us | 4.24 us | 93.63 us | 120.42x | 1.18x | 1.30x | 0.06x |
| `mat4 inverse` | 166.62 ns | 36.04 us | 17.10 us | 11.19 us | 9.02 us | 339.77 us | 216.32x | 3.22x | 4.00x | 0.11x |
| `mat4 mul mat4` | 118.26 ns | 17.76 us | 13.98 us | 6.67 us | 5.37 us | 140.06 us | 150.18x | 2.66x | 3.31x | 0.13x |
| `mat4 transform vec4` | 26.06 ns | 5.69 us | 4.54 us | 1.95 us | 1.68 us | 34.60 us | 218.26x | 2.92x | 3.39x | 0.16x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.48 ns | 1.42 us | 4.73 us | 210.04 ns | 241.28 ns | 2.08 us | 39.97x | 6.75x | 5.88x | 0.68x |
| `mat3 zero` | 16.21 ns | 903.68 ns | 908.07 ns | 276.98 ns | 210.73 ns | 11.53 ns | 55.76x | 3.26x | 4.29x | 78.35x |
| `mat3 identity` | 10.22 ns | 1.07 us | 1.08 us | 324.64 ns | 245.11 ns | 156.62 ns | 105.12x | 3.31x | 4.38x | 6.86x |
| `mat3 transpose` | 9.54 ns | 914.37 ns | 913.76 ns | 245.17 ns | 218.71 ns | 131.48 ns | 95.84x | 3.73x | 4.18x | 6.95x |
| `mat3 reciprocal` | 82.52 ns | 18.49 us | 36.53 us | 3.00 us | 2.33 us | 82.71 us | 224.04x | 6.17x | 7.92x | 0.22x |
| `mat3 reciprocal checked` | 81.84 ns | 18.66 us | 37.03 us | 3.06 us | 2.26 us | 81.66 us | 228.01x | 6.11x | 8.25x | 0.23x |
| `mat3 inverse checked` | 81.96 ns | 18.54 us | 37.00 us | 3.00 us | 2.26 us | 82.44 us | 226.22x | 6.18x | 8.20x | 0.22x |
| `mat3 inverse checked abort` | 120.99 ns | 18.14 us | 36.23 us | 3.00 us | 2.28 us | 84.54 us | 149.94x | 6.05x | 7.96x | 0.21x |
| `mat3 powi` | 146.52 ns | 17.77 us | 91.00 us | 7.00 us | 6.20 us | 152.63 us | 121.29x | 2.54x | 2.87x | 0.12x |
| `mat3 powi checked` | 145.34 ns | 17.23 us | 95.02 us | 6.76 us | 6.30 us | 151.36 us | 118.57x | 2.55x | 2.73x | 0.11x |
| `mat3 powi checked abort` | 145.87 ns | 16.80 us | 94.12 us | 6.73 us | 6.33 us | 151.40 us | 115.20x | 2.50x | 2.65x | 0.11x |
| `mat3 div scalar checked` | 25.30 ns | 5.05 us | 6.49 us | 1.45 us | 829.27 ns | 21.88 us | 199.46x | 3.49x | 6.08x | 0.23x |
| `mat3 div scalar checked abort` | 30.08 ns | 4.86 us | 6.34 us | 1.46 us | 817.94 ns | 21.77 us | 161.43x | 3.32x | 5.94x | 0.22x |
| `mat3 div matrix checked` | 152.16 ns | 65.12 us | 64.82 us | 5.47 us | 4.43 us | 157.59 us | 427.97x | 11.91x | 14.69x | 0.41x |
| `mat3 div matrix checked abort` | 167.85 ns | 65.36 us | 64.24 us | 5.48 us | 4.50 us | 163.53 us | 389.39x | 11.93x | 14.53x | 0.40x |
| `mat3 add` | 14.87 ns | 4.10 us | 6.01 us | 525.01 ns | 490.20 ns | 11.42 us | 275.59x | 7.80x | 8.36x | 0.36x |
| `mat3 add scalar` | 12.31 ns | 3.00 us | 5.04 us | 783.59 ns | 737.41 ns | 11.91 us | 243.79x | 3.83x | 4.07x | 0.25x |
| `mat3 sub` | 14.27 ns | 3.98 us | 6.04 us | 544.56 ns | 530.42 ns | 20.82 us | 279.16x | 7.31x | 7.51x | 0.19x |
| `mat3 sub scalar` | 11.23 ns | 2.98 us | 4.93 us | 821.07 ns | 708.19 ns | 20.95 us | 265.04x | 3.63x | 4.20x | 0.14x |
| `mat3 neg` | 11.09 ns | 907.89 ns | 905.59 ns | 488.33 ns | 469.13 ns | 8.61 us | 81.83x | 1.86x | 1.94x | 0.11x |
| `mat3 mul scalar` | 14.38 ns | 2.21 us | 6.29 us | 782.39 ns | 693.23 ns | 11.98 us | 153.49x | 2.82x | 3.18x | 0.18x |
| `mat3 div scalar` | 24.92 ns | 4.86 us | 6.26 us | 1.47 us | 813.13 ns | 22.64 us | 195.10x | 3.32x | 5.98x | 0.21x |
| `mat3 div matrix` | 150.33 ns | 64.75 us | 65.92 us | 5.67 us | 4.52 us | 160.24 us | 430.72x | 11.42x | 14.34x | 0.40x |
| `mat3 bitxor` | 145.89 ns | 17.28 us | 91.81 us | 6.73 us | 6.23 us | 150.44 us | 118.43x | 2.57x | 2.77x | 0.11x |
| `mat4 zero` | 13.41 ns | 1.36 us | 1.34 us | 496.77 ns | 358.19 ns | 14.20 ns | 101.07x | 2.73x | 3.78x | 95.45x |
| `mat4 identity` | 10.57 ns | 1.58 us | 1.56 us | 599.04 ns | 417.12 ns | 232.96 ns | 149.70x | 2.64x | 3.79x | 6.79x |
| `mat4 transpose` | 10.24 ns | 1.67 us | 1.68 us | 427.26 ns | 373.18 ns | 191.92 ns | 163.26x | 3.91x | 4.48x | 8.71x |
| `mat4 reciprocal` | 143.80 ns | 37.00 us | 66.32 us | 10.72 us | 8.94 us | 341.07 us | 257.28x | 3.45x | 4.14x | 0.11x |
| `mat4 reciprocal checked` | 171.19 ns | 36.02 us | 67.82 us | 10.72 us | 8.81 us | 341.08 us | 210.39x | 3.36x | 4.09x | 0.11x |
| `mat4 powi` | 247.13 ns | 33.90 us | 118.95 us | 15.60 us | 14.05 us | 360.43 us | 137.17x | 2.17x | 2.41x | 0.09x |
| `mat4 powi checked` | 245.10 ns | 33.38 us | 115.18 us | 16.11 us | 14.18 us | 355.29 us | 136.21x | 2.07x | 2.35x | 0.09x |
| `mat4 add` | 54.55 ns | 7.01 us | 7.78 us | 971.16 ns | 859.15 ns | 19.64 us | 128.59x | 7.22x | 8.16x | 0.36x |
| `mat4 add scalar` | 21.31 ns | 4.93 us | 7.60 us | 1.46 us | 1.18 us | 20.02 us | 231.60x | 3.39x | 4.17x | 0.25x |
| `mat4 sub` | 38.28 ns | 6.87 us | 7.84 us | 968.06 ns | 929.35 ns | 35.26 us | 179.35x | 7.09x | 7.39x | 0.19x |
| `mat4 sub scalar` | 15.14 ns | 5.18 us | 7.57 us | 1.47 us | 1.21 us | 37.57 us | 341.96x | 3.52x | 4.28x | 0.14x |
| `mat4 neg` | 13.80 ns | 1.64 us | 1.68 us | 923.54 ns | 755.94 ns | 13.70 us | 118.62x | 1.77x | 2.17x | 0.12x |
| `mat4 mul scalar` | 24.20 ns | 3.93 us | 8.53 us | 1.43 us | 1.17 us | 20.39 us | 162.28x | 2.75x | 3.35x | 0.19x |
| `mat4 div scalar` | 33.41 ns | 8.63 us | 8.54 us | 2.72 us | 1.39 us | 37.07 us | 258.29x | 3.17x | 6.22x | 0.23x |
| `mat4 div matrix` | 225.43 ns | 133.17 us | 104.90 us | 17.14 us | 13.98 us | 525.31 us | 590.75x | 7.77x | 9.52x | 0.25x |
| `mat4 bitxor` | 243.81 ns | 35.30 us | 112.86 us | 16.15 us | 13.98 us | 351.38 us | 144.80x | 2.19x | 2.52x | 0.10x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 3.68 ns | 211.13 ns | 327.09 ns | - | - | 1.27 us | 57.30x | - | - | 0.17x |
| `scalar add ref_owned` | 12.36 ns | 209.71 ns | 310.16 ns | - | - | 1.25 us | 16.97x | - | - | 0.17x |
| `scalar add refs` | 5.46 ns | 183.02 ns | 286.99 ns | - | - | 1.26 us | 33.54x | - | - | 0.15x |
| `scalar add owned_ref_with_clone` | 9.23 ns | 240.51 ns | 344.21 ns | - | - | - | 26.07x | - | - | - |
| `scalar add ref_owned_with_clone` | 5.70 ns | 238.68 ns | 342.29 ns | - | - | - | 41.85x | - | - | - |
| `scalar sub owned_ref` | 3.96 ns | 217.09 ns | 371.78 ns | - | - | 2.39 us | 54.85x | - | - | 0.09x |
| `scalar sub ref_owned` | 12.51 ns | 223.56 ns | 386.06 ns | - | - | 2.44 us | 17.87x | - | - | 0.09x |
| `scalar sub refs` | 5.77 ns | 195.58 ns | 343.59 ns | - | - | 2.36 us | 33.90x | - | - | 0.08x |
| `scalar sub owned_ref_with_clone` | 8.99 ns | 248.14 ns | 414.92 ns | - | - | - | 27.60x | - | - | - |
| `scalar sub ref_owned_with_clone` | 5.78 ns | 244.49 ns | 400.36 ns | - | - | - | 42.27x | - | - | - |
| `scalar mul owned_ref` | 4.58 ns | 130.56 ns | 418.61 ns | - | - | 1.47 us | 28.51x | - | - | 0.09x |
| `scalar mul ref_owned` | 13.45 ns | 134.57 ns | 418.30 ns | - | - | 1.52 us | 10.01x | - | - | 0.09x |
| `scalar mul refs` | 6.55 ns | 105.12 ns | 380.59 ns | - | - | 1.54 us | 16.04x | - | - | 0.07x |
| `scalar mul owned_ref_with_clone` | 10.10 ns | 153.62 ns | 424.90 ns | - | - | - | 15.20x | - | - | - |
| `scalar mul ref_owned_with_clone` | 6.41 ns | 152.55 ns | 423.41 ns | - | - | - | 23.78x | - | - | - |
| `scalar div owned_ref` | 6.09 ns | 293.33 ns | 346.89 ns | - | - | 2.73 us | 48.14x | - | - | 0.11x |
| `scalar div ref_owned` | 17.35 ns | 299.73 ns | 355.52 ns | - | - | 2.52 us | 17.27x | - | - | 0.12x |
| `scalar div refs` | 6.87 ns | 274.77 ns | 331.34 ns | - | - | 2.57 us | 39.97x | - | - | 0.11x |
| `scalar div owned_ref_with_clone` | 15.82 ns | 330.05 ns | 381.01 ns | - | - | - | 20.87x | - | - | - |
| `scalar div ref_owned_with_clone` | 15.78 ns | 336.04 ns | 390.70 ns | - | - | - | 21.30x | - | - | - |
| `vec3 add refs` | 6.34 ns | 727.20 ns | 1.50 us | - | - | 3.91 us | 114.74x | - | - | 0.19x |
| `vec3 sub refs` | 6.51 ns | 727.29 ns | 1.54 us | - | - | 7.39 us | 111.79x | - | - | 0.10x |
| `vec3 neg ref` | 3.44 ns | 260.10 ns | 253.32 ns | - | - | 3.10 us | 75.51x | - | - | 0.08x |
| `vec3 add_scalar_ref` | 6.69 ns | 958.67 ns | 1.65 us | - | - | 3.74 us | 143.25x | - | - | 0.26x |
| `vec3 sub_scalar_ref` | 6.78 ns | 941.75 ns | 1.46 us | - | - | 7.08 us | 138.84x | - | - | 0.13x |
| `vec3 mul_scalar_ref` | 6.95 ns | 682.92 ns | 2.19 us | - | - | 4.32 us | 98.27x | - | - | 0.16x |
| `vec3 div_scalar_ref` | 8.16 ns | 1.50 us | 1.75 us | - | - | 7.50 us | 183.20x | - | - | 0.20x |
| `vec4 add refs` | 6.42 ns | 873.88 ns | 1.51 us | - | - | 5.16 us | 136.02x | - | - | 0.17x |
| `vec4 sub refs` | 3.07 ns | 878.31 ns | 1.44 us | - | - | 9.38 us | 286.26x | - | - | 0.09x |
| `vec4 neg ref` | 4.25 ns | 346.76 ns | 342.84 ns | - | - | 4.05 us | 81.58x | - | - | 0.09x |
| `vec4 add_scalar_ref` | 7.19 ns | 1.22 us | 1.95 us | - | - | 5.11 us | 169.54x | - | - | 0.24x |
| `vec4 sub_scalar_ref` | 4.32 ns | 1.23 us | 1.70 us | - | - | 9.39 us | 283.83x | - | - | 0.13x |
| `vec4 mul_scalar_ref` | 7.63 ns | 881.01 ns | 2.36 us | - | - | 5.59 us | 115.46x | - | - | 0.16x |
| `vec4 div_scalar_ref` | 12.40 ns | 2.09 us | 1.84 us | - | - | 9.94 us | 168.14x | - | - | 0.21x |
| `mat3 add refs` | 11.11 ns | 2.04 us | 4.38 us | - | - | 11.30 us | 183.89x | - | - | 0.18x |
| `mat3 sub refs` | 10.49 ns | 1.99 us | 4.27 us | - | - | 20.78 us | 189.69x | - | - | 0.10x |
| `mat3 mul refs` | 32.39 ns | 6.11 us | 29.38 us | - | - | 60.36 us | 188.66x | - | - | 0.10x |
| `mat3 div refs` | 135.66 ns | 65.02 us | 65.43 us | - | - | 160.30 us | 479.29x | - | - | 0.41x |
| `mat3 neg ref` | 10.01 ns | 827.33 ns | 783.58 ns | - | - | 8.63 us | 82.69x | - | - | 0.10x |
| `mat3 add_scalar_ref` | 45.60 ns | 2.95 us | 5.15 us | - | - | 11.92 us | 64.71x | - | - | 0.25x |
| `mat3 sub_scalar_ref` | 45.47 ns | 3.02 us | 5.00 us | - | - | 21.19 us | 66.44x | - | - | 0.14x |
| `mat3 mul_scalar_ref` | 48.73 ns | 2.34 us | 6.47 us | - | - | 12.05 us | 48.08x | - | - | 0.19x |
| `mat3 div_scalar_ref` | 23.66 ns | 4.84 us | 6.23 us | - | - | 22.72 us | 204.61x | - | - | 0.21x |
| `mat4 add refs` | 16.66 ns | 3.13 us | 4.02 us | - | - | 18.95 us | 188.03x | - | - | 0.17x |
| `mat4 sub refs` | 16.70 ns | 3.21 us | 4.07 us | - | - | 35.21 us | 192.17x | - | - | 0.09x |
| `mat4 mul refs` | 72.71 ns | 13.50 us | 29.66 us | - | - | 140.85 us | 185.70x | - | - | 0.10x |
| `mat4 div refs` | 177.71 ns | 132.00 us | 100.95 us | - | - | 523.29 us | 742.79x | - | - | 0.25x |
| `mat4 neg ref` | 12.40 ns | 1.54 us | 1.39 us | - | - | 13.86 us | 124.24x | - | - | 0.11x |
| `mat4 add_scalar_ref` | 50.53 ns | 5.30 us | 7.67 us | - | - | 19.93 us | 104.83x | - | - | 0.27x |
| `mat4 sub_scalar_ref` | 38.21 ns | 5.30 us | 7.61 us | - | - | 37.07 us | 138.62x | - | - | 0.14x |
| `mat4 mul_scalar_ref` | 52.92 ns | 4.05 us | 8.44 us | - | - | 20.44 us | 76.47x | - | - | 0.20x |
| `mat4 div_scalar_ref` | 27.17 ns | 8.71 us | 8.50 us | - | - | 37.44 us | 320.61x | - | - | 0.23x |
| `mat3 transform_vec refs` | 14.69 ns | 2.39 us | 11.59 us | - | - | 19.93 us | 162.67x | - | - | 0.12x |
| `mat4 transform_vec refs` | 23.01 ns | 3.89 us | 11.86 us | - | - | 35.46 us | 169.21x | - | - | 0.11x |
| `complex add refs` | 7.85 ns | 390.99 ns | 749.66 ns | - | - | 2.52 us | 49.79x | - | - | 0.16x |
| `complex sub refs` | 8.17 ns | 386.73 ns | 785.13 ns | - | - | 4.80 us | 47.34x | - | - | 0.08x |
| `complex mul refs` | 8.16 ns | 891.28 ns | 3.38 us | - | - | 9.94 us | 109.28x | - | - | 0.09x |
| `complex div refs` | 17.77 ns | 2.95 us | 6.57 us | - | - | 21.32 us | 166.09x | - | - | 0.14x |
| `complex neg ref` | 2.43 ns | 123.87 ns | 121.02 ns | - | - | 2.11 us | 51.05x | - | - | 0.06x |
| `complex div_real_ref` | 7.47 ns | 741.10 ns | 755.50 ns | - | - | 5.07 us | 99.20x | - | - | 0.15x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 11.99 us |
| `astro sin 160` | 13.92 us |
| `astro sin 192` | 14.41 us |
| `astro sin 256` | 16.11 us |
