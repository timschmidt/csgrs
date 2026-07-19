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

The `mathbench` suite benchmarks the Real-primary crate path and writes this file from Criterion's median estimates after a real benchmark run. The `numerica128` comparison column runs at 128-bit precision, while the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated.

Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, range-reduction-heavy trigonometric arguments, and boundary-adjacent inverse trigonometric and inverse hyperbolic values.

## Operation Coverage

- Real construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.
- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.
- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Real Operations

#### Real Trigonometric And Inverse Comparisons

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 55.28 ns | 55.52 ns | 766.98 ns | 1.91 us | 0.07x | 0.03x |
| `cos 0.1` | 53.07 ns | 52.97 ns | 501.60 ns | 1.69 us | 0.11x | 0.03x |
| `sin 1.23456789` | 60.71 ns | 57.03 ns | 836.61 ns | 1.88 us | 0.07x | 0.03x |
| `cos 1.23456789` | 59.49 ns | 57.42 ns | 599.26 ns | 1.68 us | 0.10x | 0.04x |
| `sin 1e6` | 42.05 ns | 42.04 ns | 1.10 us | 2.12 us | 0.04x | 0.02x |
| `cos 1e6` | 41.92 ns | 41.84 ns | 834.84 ns | 1.87 us | 0.05x | 0.02x |
| `sin 1e30` | 42.19 ns | 42.00 ns | 2.89 us | 3.67 us | 0.01x | 0.01x |
| `cos 1e30` | 42.32 ns | 41.89 ns | 975.84 ns | 3.14 us | 0.04x | 0.01x |
| `sin pi_7` | 53.52 ns | 216.04 ns | 745.91 ns | 1.94 us | 0.07x | 0.03x |
| `cos pi_7` | 52.26 ns | 493.56 ns | 541.59 ns | 1.74 us | 0.10x | 0.03x |
| `sin 1000pi_eps` | 42.09 ns | 182.13 ns | 2.30 us | 2.91 us | 0.02x | 0.01x |
| `cos 1000pi_eps` | 41.84 ns | 183.84 ns | 583.01 ns | 1.71 us | 0.07x | 0.02x |
| `asin 0.5` | 99.93 ns | 100.97 ns | 2.99 us | 13.56 us | 0.03x | 0.01x |
| `acos 0.5` | 101.36 ns | 100.88 ns | 3.00 us | 13.19 us | 0.03x | 0.01x |
| `atanh 0.5` | 60.32 ns | 60.20 ns | 1.66 us | 13.28 us | 0.04x | 0.00x |
| `asin neg_0.999999` | 156.91 ns | 152.54 ns | 2.58 us | 13.12 us | 0.06x | 0.01x |
| `acos neg_0.999999` | 289.15 ns | 279.60 ns | 2.68 us | 13.12 us | 0.11x | 0.02x |
| `atanh neg_0.999999` | 185.43 ns | 167.40 ns | 1.59 us | 12.93 us | 0.12x | 0.01x |
| `asin 0.999999` | 176.38 ns | 156.22 ns | 2.57 us | 12.88 us | 0.07x | 0.01x |
| `acos 0.999999` | 151.12 ns | 170.20 ns | 2.74 us | 13.15 us | 0.06x | 0.01x |
| `atanh 0.999999` | 143.39 ns | 138.00 ns | 1.58 us | 12.96 us | 0.09x | 0.01x |
| `asin 1e-12` | 127.14 ns | 136.14 ns | 1.43 us | 15.12 us | 0.09x | 0.01x |
| `acos 1e-12` | 213.74 ns | 231.72 ns | 1.40 us | 15.30 us | 0.15x | 0.01x |
| `atanh 1e-12` | 98.32 ns | 101.89 ns | 169.10 ns | 21.70 us | 0.58x | 0.00x |
| `atan 0.5` | 67.64 ns | 67.95 ns | 2.77 us | 21.79 us | 0.02x | 0.00x |
| `asinh 0.5` | 95.31 ns | 96.74 ns | 1.62 us | 7.58 us | 0.06x | 0.01x |
| `atan neg_1e-12` | 158.85 ns | 156.45 ns | 1.14 us | 15.72 us | 0.14x | 0.01x |
| `asinh neg_1e-12` | 195.12 ns | 184.68 ns | 8.65 us | 12.01 us | 0.02x | 0.02x |
| `atan 1e6` | 118.67 ns | 118.84 ns | 1.45 us | 17.98 us | 0.08x | 0.01x |
| `asinh 1e6` | 114.32 ns | 113.91 ns | 1.66 us | 7.24 us | 0.07x | 0.02x |
| `atan neg_1e6` | 255.94 ns | 252.79 ns | 1.45 us | 17.94 us | 0.18x | 0.01x |
| `asinh neg_1e6` | 185.90 ns | 185.51 ns | 1.67 us | 7.21 us | 0.11x | 0.03x |
| `acosh 9` | 127.81 ns | 127.69 ns | 1.62 us | 9.85 us | 0.08x | 0.01x |
| `acosh 1_plus_1e-12` | 260.30 ns | 280.08 ns | 8.45 us | 11.33 us | 0.03x | 0.02x |
| `acosh 1e6` | 126.46 ns | 126.00 ns | 1.61 us | 9.97 us | 0.08x | 0.01x |
| `acosh e` | 115.93 ns | 1.13 us | 1.65 us | 9.81 us | 0.07x | 0.01x |

#### Forward Hyperbolic Construction Cases

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | 580.21 ns | 595.51 ns | 1.12 us | 10.78 us | 0.52x | 0.05x |
| `cosh half` | 548.85 ns | 285.76 ns | 1.13 us | 9.35 us | 0.49x | 0.06x |
| `tanh half` | 423.80 ns | 587.20 ns | 1.17 us | 22.92 us | 0.36x | 0.02x |
| `sinh negative_tiny` | 282.77 ns | 289.00 ns | 906.14 ns | 10.89 us | 0.31x | 0.03x |
| `cosh negative_tiny` | 282.10 ns | 283.28 ns | 612.47 ns | 9.70 us | 0.46x | 0.03x |
| `tanh negative_tiny` | 399.00 ns | 402.57 ns | 839.45 ns | 22.45 us | 0.48x | 0.02x |
| `sinh positive_20` | 526.06 ns | 541.96 ns | 1.20 us | 10.48 us | 0.44x | 0.05x |
| `cosh positive_20` | 535.78 ns | 534.75 ns | 1.20 us | 9.32 us | 0.45x | 0.06x |
| `tanh positive_20` | 402.98 ns | 406.15 ns | 1.34 us | 22.44 us | 0.30x | 0.02x |
| `sinh negative_20` | 576.62 ns | 585.82 ns | 1.19 us | 10.27 us | 0.48x | 0.06x |
| `cosh negative_20` | 558.27 ns | 558.47 ns | 1.20 us | 9.29 us | 0.47x | 0.06x |
| `tanh negative_20` | 395.11 ns | 399.57 ns | 1.34 us | 22.39 us | 0.30x | 0.02x |

#### Forward Hyperbolic Explicit f64 Output Cases

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | 355.01 ns | 350.60 ns | 1.15 us | - | 0.31x | - |
| `cosh half` | 328.48 ns | 326.70 ns | 1.18 us | - | 0.28x | - |
| `tanh half` | 451.12 ns | 457.64 ns | 1.22 us | - | 0.37x | - |
| `sinh negative_tiny` | 347.72 ns | 330.97 ns | 953.15 ns | - | 0.36x | - |
| `cosh negative_tiny` | 336.39 ns | 330.67 ns | 662.83 ns | - | 0.51x | - |
| `tanh negative_tiny` | 456.85 ns | 455.89 ns | 890.71 ns | - | 0.51x | - |
| `sinh positive_20` | 617.43 ns | 629.91 ns | 1.23 us | - | 0.50x | - |
| `cosh positive_20` | 603.05 ns | 601.14 ns | 1.24 us | - | 0.49x | - |
| `tanh positive_20` | 538.85 ns | 541.64 ns | 1.39 us | - | 0.39x | - |
| `sinh negative_20` | 715.36 ns | 707.73 ns | 1.24 us | - | 0.58x | - |
| `cosh negative_20` | 662.91 ns | 665.14 ns | 1.24 us | - | 0.54x | - |
| `tanh negative_20` | 622.43 ns | 627.57 ns | 1.39 us | - | 0.45x | - |

#### Real API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 12.20 ns | 11.90 ns | 15.63 ns | 0.95 ns | 0.78x | 12.86x |
| `one` | 12.31 ns | 11.91 ns | 30.65 ns | 31.22 ns | 0.40x | 0.39x |
| `e` | 48.57 ns | 48.23 ns | 1.10 us | 226.87 ns | 0.04x | 0.21x |
| `pi` | 36.60 ns | 36.36 ns | 48.37 ns | 228.97 ns | 0.76x | 0.16x |
| `tau` | 36.66 ns | 36.34 ns | 100.25 ns | 1.91 us | 0.37x | 0.02x |
| `add` | 29.68 ns | 31.27 ns | 42.36 ns | 1.29 us | 0.70x | 0.02x |
| `sub` | 31.14 ns | 32.72 ns | 44.66 ns | 2.43 us | 0.70x | 0.01x |
| `neg` | 57.60 ns | 57.65 ns | 24.34 ns | 1.06 us | 2.37x | 0.05x |
| `mul` | 32.98 ns | 32.90 ns | 44.16 ns | 1.53 us | 0.75x | 0.02x |
| `div` | 107.27 ns | 78.26 ns | 63.43 ns | 2.56 us | 1.69x | 0.04x |
| `reciprocal` | 62.98 ns | 62.80 ns | 60.57 ns | 1.58 us | 1.04x | 0.04x |
| `reciprocal checked` | 70.17 ns | 70.39 ns | 60.44 ns | 1.64 us | 1.16x | 0.04x |
| `reciprocal checked abort` | 82.04 ns | 84.04 ns | 60.75 ns | 1.58 us | 1.35x | 0.05x |
| `pow` | 6.41 us | 2.96 us | 2.93 us | 2.38 us | 2.19x | 2.69x |
| `powi` | 43.44 ns | 75.84 ns | 83.31 ns | 1.507 us | 0.52x | 0.03x |
| `exp` | 86.71 ns | 91.45 ns | 990.72 ns | 1.92 us | 0.09x | 0.05x |
| `exp 128` | 251.06 ns | 252.53 ns | 1.04 us | 1.91 us | 0.24x | 0.13x |
| `ln` | 1.23 us | 734.31 ns | 1.34 us | 1.84 us | 0.92x | 0.67x |
| `log10` | 1.41 us | 916.18 ns | 2.89 us | 6.76 us | 0.49x | 0.21x |
| `log10 abort` | 1.44 us | 941.40 ns | 3.00 us | 7.06 us | 0.48x | 0.20x |
| `sqrt` | 49.18 ns | 34.07 ns | 96.34 ns | 1.478 us | 0.51x | 0.03x |
| `sin` | 47.07 ns | 47.36 ns | 1.27 us | 2.27 us | 0.04x | 0.02x |
| `cos` | 46.65 ns | 47.09 ns | 655.93 ns | 1.78 us | 0.07x | 0.03x |
| `tan` | 48.06 ns | 50.83 ns | 1.62 us | 7.32 us | 0.03x | 0.01x |
| `sinh` | 543.51 ns | 532.01 ns | 1.13 us | 10.84 us | 0.48x | 0.05x |
| `cosh` | 498.27 ns | 481.75 ns | 1.06 us | 9.57 us | 0.47x | 0.05x |
| `tanh` | 537.20 ns | 521.00 ns | 1.21 us | 23.04 us | 0.45x | 0.02x |
| `asin` | 199.32 ns | 213.29 ns | 2.44 us | 14.32 us | 0.08x | 0.01x |
| `asin abort` | 219.02 ns | 231.39 ns | 2.46 us | 14.16 us | 0.09x | 0.02x |
| `acos` | 193.48 ns | 202.00 ns | 2.60 us | 14.14 us | 0.07x | 0.01x |
| `acos abort` | 212.41 ns | 220.29 ns | 2.57 us | 14.27 us | 0.08x | 0.01x |
| `atan` | 86.15 ns | 67.81 ns | 2.29 us | 19.83 us | 0.04x | 0.00x |
| `atan abort` | 109.95 ns | 98.68 ns | 2.32 us | 22.27 us | 0.05x | 0.00x |
| `asinh` | 100.01 ns | 96.27 ns | 1.65 us | 7.69 us | 0.06x | 0.01x |
| `asinh abort` | 131.39 ns | 183.99 ns | 1.64 us | 7.80 us | 0.08x | 0.02x |
| `acosh` | 168.42 ns | 164.87 ns | 3.51 us | 10.66 us | 0.05x | 0.02x |
| `acosh abort` | 186.27 ns | 184.26 ns | 3.35 us | 10.64 us | 0.06x | 0.02x |
| `atanh` | 122.05 ns | 117.39 ns | 1.29 us | 15.30 us | 0.09x | 0.01x |
| `atanh abort` | 145.48 ns | 142.55 ns | 1.28 us | 15.39 us | 0.11x | 0.01x |
| `zero status` | 1.09 ns | 1.04 ns | 6.87 ns | 8.19 ns | 0.16x | 0.13x |
| `zero status abort` | 1.17 ns | 1.18 ns | 7.01 ns | 8.20 ns | 0.17x | 0.14x |

#### Retained Square-root Cases

| Input | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica |
| --- | ---: | ---: | ---: | ---: |
| `9` | 23.52 ns | 24.69 ns | 90.16 ns | 1.360 us |
| `1e-12` | 83.26 ns | 23.86 ns | 94.73 ns | 1.619 us |
| `1e12` | 23.05 ns | 23.37 ns | 90.45 ns | 1.431 us |
| imported `e` | 63.00 ns | 62.78 ns | 100.52 ns | 1.408 us |

### Complex Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 16.12 ns | 16.18 ns | 22.78 ns | 1.91 ns | 0.71x | 8.46x |
| `one` | 16.52 ns | 16.70 ns | 42.62 ns | 30.02 ns | 0.39x | 0.55x |
| `i` | 19.74 ns | 16.58 ns | 42.43 ns | 33.00 ns | 0.47x | 0.60x |
| `free i` | 16.48 ns | 16.82 ns | 43.74 ns | 32.88 ns | 0.38x | 0.50x |
| `conjugate` | 75.21 ns | 78.23 ns | 35.81 ns | 1.09 us | 2.10x | 0.07x |
| `norm squared` | 263.22 ns | 229.85 ns | 122.43 ns | 4.32 us | 2.15x | 0.06x |
| `reciprocal` | 1.38 us | 1.19 us | 260.09 ns | 10.84 us | 5.30x | 0.13x |
| `reciprocal checked` | 1.40 us | 1.15 us | 260.37 ns | 10.84 us | 5.39x | 0.13x |
| `powi` | 679.67 ns | 788.02 ns | 1.23 us | 45.33 us | 0.55x | 0.01x |
| `powi checked` | 677.20 ns | 792.04 ns | 1.23 us | 44.95 us | 0.55x | 0.02x |
| `div checked` | 1.97 us | 1.70 us | 550.99 ns | 22.97 us | 3.58x | 0.09x |
| `div real checked` | 221.87 ns | 233.70 ns | 118.57 ns | 5.30 us | 1.87x | 0.04x |
| `from scalar` | 32.19 ns | 32.72 ns | 30.79 ns | 10.19 ns | 1.05x | 3.16x |
| `add` | 62.13 ns | 62.54 ns | 84.50 ns | 2.59 us | 0.74x | 0.02x |
| `sub` | 63.04 ns | 63.76 ns | 93.07 ns | 5.20 us | 0.68x | 0.01x |
| `neg` | 95.66 ns | 95.89 ns | 32.34 ns | 2.19 us | 2.96x | 0.04x |
| `mul` | 157.85 ns | 162.59 ns | 247.72 ns | 10.11 us | 0.64x | 0.02x |
| `div` | 2.01 us | 1.71 us | 562.90 ns | 22.59 us | 3.56x | 0.09x |
| `div real` | 220.68 ns | 230.48 ns | 117.91 ns | 5.29 us | 1.87x | 0.04x |

#### Cold Complex Multiplication

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `varying exact inputs` | 222.77 ns | 280.76 ns | 286.27 ns | 10.24 us | 0.78x | 0.02x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 453.02 ns | 422.30 ns | 257.12 ns | 7.40 us | 1.76x | 0.06x |
| `vec3 magnitude` | 796.04 ns | 1.99 us | 354.38 ns | 8.84 us | 2.25x | 0.09x |
| `vec3 normalize` | 2.57 us | 4.67 us | 591.76 ns | 16.62 us | 4.34x | 0.15x |

#### Vector API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 235.93 ns | 1.37 us | 56.41 ns | 825.22 ns | 4.18x | 0.29x |
| `vec3 zero` | 47.89 ns | 47.57 ns | 30.95 ns | 2.82 ns | 1.55x | 16.97x |
| `vec3 dot abort` | 426.71 ns | 347.07 ns | 204.28 ns | 7.19 us | 2.09x | 0.06x |
| `vec3 magnitude abort` | 822.02 ns | 1.92 us | 324.35 ns | 8.97 us | 2.53x | 0.09x |
| `vec3 normalize checked` | 2.56 us | 3.60 us | 528.75 ns | 17.43 us | 4.85x | 0.15x |
| `vec3 normalize checked abort` | 2.60 us | 3.63 us | - | - | - | - |
| `vec3 div scalar checked` | 462.83 ns | 429.43 ns | 171.88 ns | 7.69 us | 2.69x | 0.06x |
| `vec3 div scalar checked abort` | 478.61 ns | 452.38 ns | 172.41 ns | 7.68 us | 2.78x | 0.06x |
| `vec3 add` | 118.64 ns | 120.05 ns | 123.73 ns | 3.98 us | 0.96x | 0.03x |
| `vec3 add scalar` | 484.22 ns | 414.07 ns | 133.24 ns | 3.96 us | 3.63x | 0.12x |
| `vec3 sub` | 117.81 ns | 118.09 ns | 135.87 ns | 7.52 us | 0.87x | 0.02x |
| `vec3 sub scalar` | 481.92 ns | 403.79 ns | 123.86 ns | 7.01 us | 3.89x | 0.07x |
| `vec3 neg` | 89.59 ns | 88.44 ns | 50.34 ns | 3.13 us | 1.78x | 0.03x |
| `vec3 mul scalar` | 91.63 ns | 88.89 ns | 119.31 ns | 4.34 us | 0.77x | 0.02x |
| `vec3 div scalar` | 118.51 ns | 120.47 ns | 169.34 ns | 7.65 us | 0.70x | 0.02x |
| `vec4 dot` | 408.77 ns | 251.40 ns | 326.88 ns | 9.68 us | 1.25x | 0.04x |
| `vec4 magnitude` | 832.44 ns | 1.30 us | 409.78 ns | 11.65 us | 2.03x | 0.07x |
| `vec4 normalize` | 3.16 us | 2.67 us | 736.42 ns | 22.07 us | 4.29x | 0.14x |
| `vec4 add` | 159.83 ns | 162.23 ns | 174.44 ns | 5.24 us | 0.92x | 0.03x |
| `vec4 add scalar` | 526.86 ns | 437.35 ns | 177.42 ns | 5.26 us | 2.97x | 0.10x |
| `vec4 sub` | 159.87 ns | 161.66 ns | 176.50 ns | 9.80 us | 0.91x | 0.02x |
| `vec4 sub scalar` | 500.94 ns | 406.39 ns | 170.91 ns | 9.52 us | 2.93x | 0.05x |
| `vec4 neg` | 108.93 ns | 109.83 ns | 63.70 ns | 4.02 us | 1.71x | 0.03x |
| `vec4 mul scalar` | 108.07 ns | 112.90 ns | 158.24 ns | 5.65 us | 0.68x | 0.02x |
| `vec4 div scalar` | 147.80 ns | 146.40 ns | 225.42 ns | 10.15 us | 0.66x | 0.01x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 894.96 ns | 396.21 ns | 949.80 ns | 22.87 us | 0.94x | 0.04x |
| `mat3 inverse` | 12.15 us | 2.46 us | 2.51 us | 84.21 us | 4.84x | 0.14x |
| `mat3 mul mat3` | 5.52 us | 4.23 us | 2.39 us | 62.00 us | 2.31x | 0.09x |
| `mat3 transform vec3` | 2.77 us | 2.26 us | 906.03 ns | 21.08 us | 3.05x | 0.13x |
| `mat4 determinant` | 1.99 us | 637.81 ns | 4.09 us | 96.50 us | 0.49x | 0.02x |
| `mat4 inverse` | 33.93 us | 5.91 us | 9.29 us | 344.99 us | 3.65x | 0.10x |
| `mat4 mul mat4` | 9.35 us | 6.97 us | 5.46 us | 145.36 us | 1.71x | 0.06x |
| `mat4 transform vec4` | 7.49 us | 5.74 us | 1.66 us | 35.91 us | 4.52x | 0.21x |

#### Matrix API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 764.93 ns | 2.21 us | 222.81 ns | 2.38 us | 3.43x | 0.32x |
| `mat3 zero` | 223.45 ns | 218.56 ns | 200.35 ns | 11.85 ns | 1.12x | 18.86x |
| `mat3 identity` | 240.55 ns | 238.99 ns | 251.74 ns | 180.71 ns | 0.96x | 1.33x |
| `mat3 transpose` | 270.70 ns | 264.70 ns | 191.91 ns | 107.12 ns | 1.41x | 2.53x |
| `mat3 reciprocal` | 12.01 us | 3.61 us | 2.28 us | 82.16 us | 5.28x | 0.15x |
| `mat3 reciprocal checked` | 11.93 us | 3.68 us | 2.27 us | 81.25 us | 5.25x | 0.15x |
| `mat3 inverse checked` | 12.05 us | 3.67 us | 2.27 us | 82.01 us | 5.30x | 0.15x |
| `mat3 inverse checked abort` | 12.04 us | 3.73 us | 2.28 us | 81.05 us | 5.29x | 0.15x |
| `mat3 powi` | 6.97 us | 11.51 us | 6.19 us | 154.11 us | 1.13x | 0.05x |
| `mat3 powi checked` | 6.99 us | 11.55 us | 6.21 us | 151.43 us | 1.12x | 0.05x |
| `mat3 powi checked abort` | 6.85 us | 11.47 us | 6.19 us | 149.78 us | 1.11x | 0.05x |
| `mat3 div scalar checked` | 1.45 us | 1.39 us | 809.02 ns | 22.39 us | 1.79x | 0.06x |
| `mat3 div scalar checked abort` | 1.45 us | 1.40 us | 807.49 ns | 22.04 us | 1.79x | 0.07x |
| `mat3 div matrix checked` | 17.14 us | 11.94 us | 4.46 us | 177.35 us | 3.84x | 0.10x |
| `mat3 div matrix checked abort` | 17.57 us | 11.88 us | 4.42 us | 159.34 us | 3.98x | 0.11x |
| `mat3 add` | 332.77 ns | 331.28 ns | 452.86 ns | 11.72 us | 0.73x | 0.03x |
| `mat3 add scalar` | 1.27 us | 1.27 us | 700.42 ns | 11.90 us | 1.81x | 0.11x |
| `mat3 sub` | 347.12 ns | 342.75 ns | 482.26 ns | 20.92 us | 0.72x | 0.02x |
| `mat3 sub scalar` | 1.29 us | 1.31 us | 695.68 ns | 21.35 us | 1.85x | 0.06x |
| `mat3 neg` | 209.49 ns | 219.40 ns | 423.07 ns | 9.02 us | 0.50x | 0.02x |
| `mat3 mul scalar` | 184.44 ns | 192.68 ns | 647.22 ns | 11.89 us | 0.28x | 0.02x |
| `mat3 div scalar` | 229.99 ns | 233.38 ns | 787.66 ns | 22.43 us | 0.29x | 0.01x |
| `mat3 div matrix` | 17.29 us | 11.86 us | 4.43 us | 157.16 us | 3.90x | 0.11x |
| `mat3 bitxor` | 6.84 us | 11.47 us | 6.21 us | 152.68 us | 1.10x | 0.04x |
| `mat4 zero` | 217.98 ns | 205.93 ns | 325.06 ns | 15.79 ns | 0.67x | 13.81x |
| `mat4 identity` | 281.48 ns | 274.47 ns | 384.33 ns | 253.41 ns | 0.73x | 1.11x |
| `mat4 transpose` | 325.56 ns | 326.84 ns | 335.75 ns | 162.96 ns | 0.97x | 2.00x |
| `mat4 reciprocal` | 24.32 us | 20.56 us | 8.78 us | 360.93 us | 2.77x | 0.07x |
| `mat4 reciprocal checked` | 24.57 us | 20.46 us | 8.79 us | 343.75 us | 2.80x | 0.07x |
| `mat4 powi` | 12.68 us | 21.73 us | 13.70 us | 667.14 us | 0.93x | 0.02x |
| `mat4 powi checked` | 12.73 us | 21.54 us | 13.79 us | 685.21 us | 0.92x | 0.02x |
| `mat4 add` | 635.13 ns | 638.16 ns | 758.16 ns | 40.57 us | 0.84x | 0.02x |
| `mat4 add scalar` | 1.98 us | 1.79 us | 1.17 us | 42.79 us | 1.69x | 0.05x |
| `mat4 sub` | 665.25 ns | 683.60 ns | 816.63 ns | 76.37 us | 0.81x | 0.01x |
| `mat4 sub scalar` | 2.02 us | 1.80 us | 1.17 us | 79.13 us | 1.72x | 0.03x |
| `mat4 neg` | 357.86 ns | 374.92 ns | 728.99 ns | 13.98 us | 0.49x | 0.03x |
| `mat4 mul scalar` | 305.44 ns | 309.18 ns | 1.124 us | 20.00 us | 0.27x | 0.02x |
| `mat4 div scalar` | 381.16 ns | 377.44 ns | 1.37 us | 38.21 us | 0.28x | 0.01x |
| `mat4 div matrix` | 29.33 us | 32.86 us | 14.10 us | 533.86 us | 2.08x | 0.05x |
| `mat4 bitxor` | 12.46 us | 21.39 us | 13.74 us | 721.23 us | 0.91x | 0.02x |

### Borrowed API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 18.35 ns | 19.03 ns | 44.37 ns | 1.29 us | 0.41x | 0.01x |
| `scalar add ref_owned` | 24.06 ns | 23.77 ns | 44.42 ns | 1.28 us | 0.54x | 0.02x |
| `scalar add refs` | 19.74 ns | 20.69 ns | 44.28 ns | 1.275 us | 0.45x | 0.02x |
| `scalar add owned_ref_with_clone` | 170.09 ns | 99.00 ns | 58.47 ns | 1.27 us | 2.91x | 0.13x |
| `scalar add ref_owned_with_clone` | 171.25 ns | 94.34 ns | 55.92 ns | 1.31 us | 3.06x | 0.13x |
| `scalar sub owned_ref` | 19.55 ns | 20.13 ns | 46.93 ns | 2.46 us | 0.42x | 0.01x |
| `scalar sub ref_owned` | 25.24 ns | 24.64 ns | 47.05 ns | 2.45 us | 0.54x | 0.01x |
| `scalar sub refs` | 19.97 ns | 22.50 ns | 47.00 ns | 2.468 us | 0.42x | 0.01x |
| `scalar sub owned_ref_with_clone` | 123.04 ns | 111.03 ns | 62.11 ns | 2.46 us | 1.98x | 0.05x |
| `scalar sub ref_owned_with_clone` | 122.22 ns | 107.63 ns | 59.09 ns | 2.48 us | 2.07x | 0.05x |
| `scalar mul owned_ref` | 141.12 ns | 114.06 ns | 49.08 ns | 1.54 us | 2.88x | 0.09x |
| `scalar mul ref_owned` | 142.71 ns | 115.52 ns | 49.04 ns | 1.55 us | 2.91x | 0.09x |
| `scalar mul refs` | 155.11 ns | 130.22 ns | 49.05 ns | 1.54 us | 3.16x | 0.10x |
| `scalar mul owned_ref_with_clone` | 165.05 ns | 136.66 ns | 62.67 ns | 1.56 us | 2.63x | 0.11x |
| `scalar mul ref_owned_with_clone` | 157.22 ns | 132.80 ns | 61.53 ns | 1.57 us | 2.56x | 0.10x |
| `scalar div owned_ref` | 85.21 ns | 56.10 ns | 65.84 ns | 2.59 us | 1.29x | 0.03x |
| `scalar div ref_owned` | 87.41 ns | 57.67 ns | 65.68 ns | 2.58 us | 1.33x | 0.03x |
| `scalar div refs` | 87.68 ns | 59.48 ns | 65.69 ns | 2.60 us | 1.33x | 0.03x |
| `scalar div owned_ref_with_clone` | 96.96 ns | 68.35 ns | 80.40 ns | 4.92 us | 1.21x | 0.02x |
| `scalar div ref_owned_with_clone` | 97.54 ns | 66.92 ns | 76.61 ns | 2.91 us | 1.27x | 0.03x |
| `vec3 add refs` | 72.61 ns | 73.56 ns | 122.07 ns | 4.003 us | 0.59x | 0.02x |
| `vec3 sub refs` | 76.22 ns | 76.11 ns | 136.57 ns | 7.340 us | 0.56x | 0.01x |
| `vec3 neg ref` | 42.82 ns | 42.93 ns | 46.09 ns | 3.09 us | 0.93x | 0.01x |
| `vec3 add_scalar_ref` | 471.89 ns | 381.17 ns | 134.25 ns | 3.97 us | 3.52x | 0.12x |
| `vec3 sub_scalar_ref` | 467.29 ns | 399.44 ns | 126.26 ns | 7.17 us | 3.70x | 0.07x |
| `vec3 mul_scalar_ref` | 774.49 ns | 716.01 ns | 154.91 ns | 4.44 us | 5.00x | 0.17x |
| `vec3 div_scalar_ref` | 441.90 ns | 399.86 ns | 184.56 ns | 7.76 us | 2.39x | 0.06x |
| `vec4 add refs` | 93.99 ns | 93.21 ns | 176.08 ns | 5.19 us | 0.53x | 0.02x |
| `vec4 sub refs` | 98.57 ns | 97.45 ns | 179.82 ns | 9.52 us | 0.55x | 0.01x |
| `vec4 neg ref` | 50.90 ns | 47.82 ns | 63.57 ns | 4.04 us | 0.80x | 0.01x |
| `vec4 add_scalar_ref` | 514.59 ns | 406.30 ns | 175.46 ns | 5.17 us | 2.93x | 0.10x |
| `vec4 sub_scalar_ref` | 507.54 ns | 423.29 ns | 167.11 ns | 9.27 us | 3.04x | 0.05x |
| `vec4 mul_scalar_ref` | 899.76 ns | 823.85 ns | 161.07 ns | 5.65 us | 5.59x | 0.16x |
| `vec4 div_scalar_ref` | 493.06 ns | 463.92 ns | 227.41 ns | 10.03 us | 2.17x | 0.05x |
| `mat3 add refs` | 355.33 ns | 342.07 ns | 439.02 ns | 11.51 us | 0.81x | 0.03x |
| `mat3 sub refs` | 385.40 ns | 373.34 ns | 470.46 ns | 21.09 us | 0.82x | 0.02x |
| `mat3 mul refs` | 4.77 us | 4.20 us | 2.16 us | 61.15 us | 2.21x | 0.08x |
| `mat3 div refs` | 16.67 us | 11.24 us | 4.36 us | 160.10 us | 3.82x | 0.10x |
| `mat3 neg ref` | 118.77 ns | 131.67 ns | 416.49 ns | 8.43 us | 0.29x | 0.01x |
| `mat3 add_scalar_ref` | 1.25 us | 1.27 us | 697.18 ns | 12.03 us | 1.80x | 0.10x |
| `mat3 sub_scalar_ref` | 1.32 us | 1.26 us | 688.32 ns | 21.48 us | 1.92x | 0.06x |
| `mat3 mul_scalar_ref` | 1.62 us | 1.62 us | 659.98 ns | 12.13 us | 2.45x | 0.13x |
| `mat3 div_scalar_ref` | 1.39 us | 1.36 us | 806.63 ns | 22.50 us | 1.73x | 0.06x |
| `mat4 add refs` | 493.29 ns | 516.71 ns | 765.62 ns | 19.07 us | 0.64x | 0.03x |
| `mat4 sub refs` | 555.35 ns | 573.42 ns | 817.04 ns | 35.64 us | 0.68x | 0.02x |
| `mat4 mul refs` | 8.13 us | 7.62 us | 4.92 us | 144.85 us | 1.65x | 0.06x |
| `mat4 div refs` | 28.26 us | 32.85 us | 14.10 us | 540.77 us | 2.00x | 0.05x |
| `mat4 neg ref` | 197.26 ns | 213.41 ns | 748.76 ns | 13.80 us | 0.26x | 0.01x |
| `mat4 add_scalar_ref` | 1.75 us | 1.77 us | 1.17 us | 20.94 us | 1.49x | 0.08x |
| `mat4 sub_scalar_ref` | 1.82 us | 1.79 us | 1.16 us | 37.12 us | 1.57x | 0.05x |
| `mat4 mul_scalar_ref` | 2.14 us | 2.26 us | 1.11 us | 20.21 us | 1.92x | 0.11x |
| `mat4 div_scalar_ref` | 2.00 us | 1.97 us | 1.37 us | 37.99 us | 1.46x | 0.05x |
| `mat3 transform_vec refs` | 2.44 us | 2.05 us | 677.53 ns | 20.54 us | 3.60x | 0.12x |
| `mat4 transform_vec refs` | 3.79 us | 3.19 us | 1.28 us | 35.26 us | 2.96x | 0.11x |
| `complex add refs` | 34.44 ns | 35.62 ns | 84.70 ns | 2.55 us | 0.41x | 0.01x |
| `complex sub refs` | 35.33 ns | 36.48 ns | 93.02 ns | 4.78 us | 0.38x | 0.01x |
| `complex mul refs` | 138.77 ns | 140.11 ns | 235.87 ns | 10.11 us | 0.59x | 0.01x |
| `complex div refs` | 1.91 us | 1.63 us | 543.99 ns | 21.88 us | 3.50x | 0.09x |
| `complex neg ref` | 93.45 ns | 95.17 ns | 35.42 ns | 2.13 us | 2.64x | 0.04x |
| `complex div_real_ref` | 219.38 ns | 238.92 ns | 115.42 ns | 5.23 us | 1.90x | 0.04x |
