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

The `mathbench` suite benchmarks the Real-primary crate path and writes this file from Criterion's median estimates after a real benchmark run. The exact-dyadic column imports each finite binary64 fixture as its exact dyadic rational value; it does not perform binary64 arithmetic. The explicit-rational column constructs the corresponding authored rational inputs directly. The `numerica128` comparison column runs at 128-bit precision, `gmp_mpfr128` uses Rug's GMP/MPFR stack with 128-bit MPFR scalars, and the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated.

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

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 49.58 ns | 61.44 ns | 795.59 ns | 771.90 ns | 1.88 us | 0.06x | 0.06x | 0.03x |
| `cos 0.1` | 49.36 ns | 61.36 ns | 486.57 ns | 493.99 ns | 1.71 us | 0.10x | 0.10x | 0.03x |
| `sin 1.23456789` | 55.44 ns | 67.06 ns | 822.92 ns | 825.39 ns | 1.91 us | 0.07x | 0.07x | 0.03x |
| `cos 1.23456789` | 71.75 ns | 66.98 ns | 583.61 ns | 600.89 ns | 1.76 us | 0.12x | 0.12x | 0.04x |
| `sin 1e6` | 44.16 ns | 44.86 ns | 1.09 us | 1.87 us | 2.06 us | 0.04x | 0.02x | 0.02x |
| `cos 1e6` | 44.28 ns | 44.81 ns | 822.62 ns | 870.94 ns | 1.85 us | 0.05x | 0.05x | 0.02x |
| `sin 1e30` | 44.55 ns | 44.16 ns | 2.86 us | 2.89 us | 3.60 us | 0.02x | 0.02x | 0.01x |
| `cos 1e30` | 44.56 ns | 46.46 ns | 968.41 ns | 962.86 ns | 3.13 us | 0.05x | 0.05x | 0.01x |
| `sin pi_7` | 61.62 ns | 246.32 ns | 731.09 ns | 737.01 ns | 2.02 us | 0.08x | 0.08x | 0.03x |
| `cos pi_7` | 63.09 ns | 145.02 ns | 519.26 ns | 523.10 ns | 1.83 us | 0.12x | 0.12x | 0.03x |
| `sin 1000pi_eps` | 45.15 ns | 216.15 ns | 2.26 us | 2.22 us | 2.88 us | 0.02x | 0.02x | 0.02x |
| `cos 1000pi_eps` | 44.11 ns | 216.95 ns | 579.98 ns | 597.24 ns | 1.72 us | 0.08x | 0.07x | 0.03x |
| `asin 0.5` | 102.60 ns | 103.86 ns | 2.96 us | 2.97 us | 13.43 us | 0.03x | 0.03x | 0.01x |
| `acos 0.5` | 103.52 ns | 102.22 ns | 2.98 us | 2.96 us | 13.34 us | 0.03x | 0.03x | 0.01x |
| `atanh 0.5` | 55.88 ns | 55.53 ns | 1.71 us | 1.69 us | 13.16 us | 0.03x | 0.03x | 0.00x |
| `asin neg_0.999999` | 286.24 ns | 149.09 ns | 2.56 us | 2.52 us | 13.20 us | 0.11x | 0.11x | 0.02x |
| `acos neg_0.999999` | 446.45 ns | 214.47 ns | 2.69 us | 2.70 us | 13.22 us | 0.17x | 0.17x | 0.03x |
| `atanh neg_0.999999` | 192.55 ns | 160.71 ns | 1.64 us | 1.69 us | 13.01 us | 0.12x | 0.11x | 0.01x |
| `asin 0.999999` | 153.18 ns | 145.94 ns | 2.61 us | 4.67 us | 13.24 us | 0.06x | 0.03x | 0.01x |
| `acos 0.999999` | 165.15 ns | 157.87 ns | 2.76 us | 5.02 us | 13.28 us | 0.06x | 0.03x | 0.01x |
| `atanh 0.999999` | 147.61 ns | 132.62 ns | 1.65 us | 1.68 us | 12.81 us | 0.09x | 0.09x | 0.01x |
| `asin 1e-12` | 125.78 ns | 143.23 ns | 1.45 us | 1.45 us | 15.39 us | 0.09x | 0.09x | 0.01x |
| `acos 1e-12` | 219.89 ns | 252.56 ns | 2.83 us | 1.47 us | 15.58 us | 0.08x | 0.15x | 0.01x |
| `atanh 1e-12` | 97.42 ns | 108.97 ns | 183.18 ns | 173.20 ns | 20.24 us | 0.53x | 0.56x | 0.00x |
| `atan 0.5` | 69.53 ns | 71.27 ns | 3.03 us | 2.74 us | 17.94 us | 0.02x | 0.03x | 0.00x |
| `asinh 0.5` | 109.47 ns | 106.90 ns | 1.62 us | 1.60 us | 7.63 us | 0.07x | 0.07x | 0.01x |
| `atan neg_1e-12` | 136.74 ns | 132.48 ns | 1.11 us | 1.06 us | 15.70 us | 0.12x | 0.13x | 0.01x |
| `asinh neg_1e-12` | 163.29 ns | 170.14 ns | 8.61 us | 8.57 us | 11.93 us | 0.02x | 0.02x | 0.01x |
| `atan 1e6` | 115.11 ns | 120.18 ns | 1.45 us | 1.40 us | 18.27 us | 0.08x | 0.08x | 0.01x |
| `asinh 1e6` | 123.14 ns | 123.44 ns | 1.66 us | 1.66 us | 7.40 us | 0.07x | 0.07x | 0.02x |
| `atan neg_1e6` | 258.89 ns | 260.64 ns | 1.45 us | 1.39 us | 18.31 us | 0.18x | 0.19x | 0.01x |
| `asinh neg_1e6` | 170.55 ns | 172.19 ns | 1.70 us | 1.69 us | 7.16 us | 0.10x | 0.10x | 0.02x |
| `acosh 9` | 87.71 ns | 89.00 ns | 1.64 us | 1.65 us | 10.01 us | 0.05x | 0.05x | 0.01x |
| `acosh 1_plus_1e-12` | 124.03 ns | 123.30 ns | 8.46 us | 8.53 us | 11.53 us | 0.01x | 0.01x | 0.01x |
| `acosh 1e6` | 87.51 ns | 89.48 ns | 1.69 us | 1.87 us | 11.32 us | 0.05x | 0.05x | 0.01x |
| `acosh e` | 81.33 ns | 100.14 ns | 1.69 us | 1.87 us | 10.24 us | 0.05x | 0.04x | 0.01x |

#### Forward Hyperbolic Construction Cases

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | 389.11 ns | 394.39 ns | 1.23 us | 1.20 us | 11.76 us | 0.32x | 0.32x | 0.03x |
| `cosh half` | 397.28 ns | 377.63 ns | 1.20 us | 1.26 us | 10.48 us | 0.33x | 0.32x | 0.04x |
| `tanh half` | 544.05 ns | 524.18 ns | 1.36 us | 1.31 us | 51.70 us | 0.40x | 0.42x | 0.01x |
| `sinh negative_tiny` | 696.32 ns | 399.68 ns | 980.55 ns | 1.00 us | 11.86 us | 0.71x | 0.69x | 0.06x |
| `cosh negative_tiny` | 371.79 ns | 404.56 ns | 639.60 ns | 639.58 ns | 10.54 us | 0.58x | 0.58x | 0.04x |
| `tanh negative_tiny` | 479.46 ns | 527.63 ns | 905.38 ns | 920.80 ns | 24.65 us | 0.53x | 0.52x | 0.02x |
| `sinh positive_20` | 677.80 ns | 678.55 ns | 1.26 us | 2.23 us | 11.31 us | 0.54x | 0.30x | 0.06x |
| `cosh positive_20` | 647.13 ns | 631.20 ns | 1.26 us | 1.27 us | 9.71 us | 0.51x | 0.51x | 0.07x |
| `tanh positive_20` | 513.93 ns | 507.18 ns | 1.35 us | 1.29 us | 23.16 us | 0.38x | 0.40x | 0.02x |
| `sinh negative_20` | 728.37 ns | 735.70 ns | 1.21 us | 1.22 us | 11.07 us | 0.60x | 0.59x | 0.07x |
| `cosh negative_20` | 692.29 ns | 678.18 ns | 1.22 us | 1.23 us | 9.45 us | 0.57x | 0.56x | 0.07x |
| `tanh negative_20` | 582.67 ns | 593.97 ns | 1.35 us | 1.29 us | 22.86 us | 0.43x | 0.45x | 0.03x |

#### Forward Hyperbolic Explicit f64 Output Cases

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | 369.64 ns | 373.63 ns | 1.13 us | 1.15 us | - | 0.33x | 0.32x | - |
| `cosh half` | 368.61 ns | 370.91 ns | 1.14 us | 1.19 us | - | 0.32x | 0.31x | - |
| `tanh half` | 512.86 ns | 666.59 ns | 1.21 us | 1.22 us | - | 0.42x | 0.42x | - |
| `sinh negative_tiny` | 336.13 ns | 375.25 ns | 947.94 ns | 945.75 ns | - | 0.35x | 0.36x | - |
| `cosh negative_tiny` | 354.40 ns | 384.95 ns | 642.31 ns | 1.16 us | - | 0.55x | 0.31x | - |
| `tanh negative_tiny` | 481.01 ns | 512.80 ns | 1.51 us | 884.96 ns | - | 0.32x | 0.54x | - |
| `sinh positive_20` | 652.23 ns | 662.41 ns | 1.23 us | 1.25 us | - | 0.53x | 0.52x | - |
| `cosh positive_20` | 658.18 ns | 671.91 ns | 1.24 us | 1.28 us | - | 0.53x | 0.51x | - |
| `tanh positive_20` | 510.09 ns | 506.88 ns | 1.39 us | 1.30 us | - | 0.37x | 0.39x | - |
| `sinh negative_20` | 738.56 ns | 737.33 ns | 1.23 us | 1.23 us | - | 0.60x | 0.60x | - |
| `cosh negative_20` | 677.68 ns | 678.76 ns | 1.48 us | 2.22 us | - | 0.46x | 0.30x | - |
| `tanh negative_20` | 1.10 us | 1.11 us | 2.36 us | 1.81 us | - | 0.46x | 0.61x | - |

#### Real API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 12.93 ns | 11.77 ns | 15.94 ns | 9.18 ns | 0.95 ns | 0.81x | 1.41x | 13.64x |
| `one` | 13.19 ns | 12.09 ns | 33.58 ns | 23.49 ns | 31.04 ns | 0.39x | 0.56x | 0.43x |
| `e` | 41.39 ns | 44.61 ns | 1.07 us | 1.01 us | 229.23 ns | 0.04x | 0.04x | 0.18x |
| `pi` | 31.97 ns | 33.13 ns | 50.21 ns | 20.16 ns | 228.34 ns | 0.64x | 1.59x | 0.14x |
| `tau` | 31.94 ns | 33.02 ns | 110.03 ns | 68.61 ns | 1.96 us | 0.29x | 0.47x | 0.02x |
| `add` | 29.77 ns | 36.78 ns | 49.79 ns | 31.13 ns | 1.34 us | 0.60x | 0.96x | 0.02x |
| `sub` | 31.40 ns | 34.89 ns | 47.63 ns | 37.94 ns | 2.48 us | 0.66x | 0.83x | 0.01x |
| `neg` | 18.17 ns | 22.03 ns | 21.45 ns | 27.09 ns | 1.13 us | 0.85x | 0.67x | 0.02x |
| `mul` | 31.52 ns | 61.13 ns | 45.34 ns | 42.54 ns | 1.55 us | 0.70x | 0.74x | 0.02x |
| `div` | 71.07 ns | 113.13 ns | 62.90 ns | 59.37 ns | 2.60 us | 1.13x | 1.20x | 0.03x |
| `reciprocal` | 27.76 ns | 28.86 ns | 58.62 ns | 58.63 ns | 1.57 us | 0.47x | 0.47x | 0.02x |
| `reciprocal checked` | 27.71 ns | 29.41 ns | 58.80 ns | 58.59 ns | 1.56 us | 0.47x | 0.47x | 0.02x |
| `reciprocal checked abort` | 47.81 ns | 46.73 ns | 58.74 ns | 59.12 ns | 1.57 us | 0.81x | 0.81x | 0.03x |
| `pow` | 2.87 us | 3.37 us | 2.87 us | 2.82 us | 2.34 us | 1.00x | 1.02x | 1.23x |
| `powi` | 58.96 ns | 61.61 ns | 85.85 ns | 90.91 ns | 1.56 us | 0.69x | 0.65x | 0.04x |
| `exp` | 93.88 ns | 96.00 ns | 917.69 ns | 896.26 ns | 1.92 us | 0.10x | 0.10x | 0.05x |
| `exp 128` | 430.33 ns | 411.97 ns | 1.13 us | 1.10 us | 1.99 us | 0.38x | 0.39x | 0.22x |
| `ln` | 1.25 us | 445.29 ns | 1.33 us | 1.36 us | 1.86 us | 0.94x | 0.92x | 0.68x |
| `log10` | 1.43 us | 602.64 ns | 2.80 us | 4.01 us | 6.79 us | 0.51x | 0.36x | 0.21x |
| `log10 abort` | 1.50 us | 610.05 ns | 2.85 us | 4.00 us | 6.76 us | 0.53x | 0.37x | 0.22x |
| `sqrt` | 58.06 ns | 43.85 ns | 95.19 ns | 107.08 ns | 3.09 us | 0.61x | 0.54x | 0.02x |
| `sin` | 55.57 ns | 55.49 ns | 1.23 us | 1.24 us | 2.32 us | 0.04x | 0.04x | 0.02x |
| `cos` | 54.34 ns | 56.17 ns | 658.62 ns | 621.09 ns | 1.83 us | 0.08x | 0.09x | 0.03x |
| `tan` | 56.81 ns | 58.39 ns | 1.58 us | 1.57 us | 7.24 us | 0.04x | 0.04x | 0.01x |
| `sinh` | 562.77 ns | 582.50 ns | 1.12 us | 1.17 us | 10.94 us | 0.50x | 0.48x | 0.05x |
| `cosh` | 549.67 ns | 568.70 ns | 1.07 us | 1.12 us | 10.62 us | 0.51x | 0.49x | 0.05x |
| `tanh` | 514.31 ns | 566.18 ns | 1.20 us | 1.18 us | 23.55 us | 0.43x | 0.43x | 0.02x |
| `asin` | 142.63 ns | 143.85 ns | 2.41 us | 2.57 us | 15.20 us | 0.06x | 0.06x | 0.01x |
| `asin abort` | 161.43 ns | 165.69 ns | 2.43 us | 2.44 us | 14.70 us | 0.07x | 0.07x | 0.01x |
| `acos` | 199.82 ns | 209.13 ns | 2.54 us | 2.84 us | 14.72 us | 0.08x | 0.07x | 0.01x |
| `acos abort` | 214.03 ns | 208.37 ns | 2.56 us | 2.61 us | 14.44 us | 0.08x | 0.08x | 0.01x |
| `atan` | 83.46 ns | 84.74 ns | 2.28 us | 4.29 us | 19.34 us | 0.04x | 0.02x | 0.00x |
| `atan abort` | 108.72 ns | 114.57 ns | 2.29 us | 2.36 us | 20.56 us | 0.05x | 0.05x | 0.01x |
| `asinh` | 111.15 ns | 112.74 ns | 1.67 us | 1.68 us | 14.51 us | 0.07x | 0.07x | 0.01x |
| `asinh abort` | 136.98 ns | 135.27 ns | 1.73 us | 1.69 us | 8.34 us | 0.08x | 0.08x | 0.02x |
| `acosh` | 95.77 ns | 97.97 ns | 3.36 us | 3.39 us | 13.81 us | 0.03x | 0.03x | 0.01x |
| `acosh abort` | 121.44 ns | 123.16 ns | 3.45 us | 3.36 us | 10.74 us | 0.04x | 0.04x | 0.01x |
| `atanh` | 126.76 ns | 118.59 ns | 1.31 us | 1.30 us | 15.89 us | 0.10x | 0.10x | 0.01x |
| `atanh abort` | 146.73 ns | 141.30 ns | 1.33 us | 1.30 us | 15.33 us | 0.11x | 0.11x | 0.01x |
| `zero status` | 2.19 ns | 2.21 ns | 6.72 ns | 0.94 ns | 8.10 ns | 0.33x | 2.32x | 0.27x |
| `zero status abort` | 2.65 ns | 2.67 ns | 6.74 ns | 0.94 ns | 8.64 ns | 0.39x | 2.81x | 0.31x |

### Complex Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 18.73 ns | 18.59 ns | 22.87 ns | 21.00 ns | 1.90 ns | 0.82x | 0.89x | 9.86x |
| `one` | 20.78 ns | 18.36 ns | 46.75 ns | 52.12 ns | 29.90 ns | 0.44x | 0.40x | 0.69x |
| `i` | 18.83 ns | 18.30 ns | 47.74 ns | 36.24 ns | 37.38 ns | 0.39x | 0.52x | 0.50x |
| `free i` | 19.07 ns | 18.29 ns | 45.89 ns | 35.82 ns | 44.56 ns | 0.42x | 0.53x | 0.43x |
| `conjugate` | 42.86 ns | 40.23 ns | 35.33 ns | 45.00 ns | 1.11 us | 1.21x | 0.95x | 0.04x |
| `norm squared` | 92.41 ns | 102.86 ns | 122.72 ns | 137.22 ns | 5.40 us | 0.75x | 0.67x | 0.02x |
| `reciprocal` | 195.95 ns | 194.19 ns | 257.17 ns | 219.10 ns | 11.45 us | 0.76x | 0.89x | 0.02x |
| `reciprocal checked` | 387.23 ns | 205.38 ns | 257.49 ns | 218.06 ns | 11.16 us | 1.50x | 1.78x | 0.03x |
| `powi` | 692.72 ns | 691.76 ns | 1.29 us | 1.03 us | 45.01 us | 0.54x | 0.67x | 0.02x |
| `powi checked` | 682.00 ns | 734.80 ns | 1.28 us | 1.03 us | 44.66 us | 0.53x | 0.66x | 0.02x |
| `div checked` | 267.58 ns | 266.45 ns | 548.96 ns | 460.40 ns | 21.94 us | 0.49x | 0.58x | 0.01x |
| `div real checked` | 107.33 ns | 109.12 ns | 119.65 ns | 112.09 ns | 5.25 us | 0.90x | 0.96x | 0.02x |
| `from scalar` | 25.08 ns | 24.86 ns | 30.51 ns | 28.10 ns | 10.40 ns | 0.82x | 0.89x | 2.41x |
| `add` | 68.75 ns | 67.76 ns | 86.98 ns | 55.15 ns | 2.64 us | 0.79x | 1.25x | 0.03x |
| `sub` | 69.32 ns | 67.84 ns | 135.06 ns | 55.04 ns | 4.88 us | 0.51x | 1.26x | 0.01x |
| `neg` | 50.13 ns | 49.91 ns | 54.78 ns | 33.00 ns | 2.19 us | 0.92x | 1.52x | 0.02x |
| `mul` | 248.33 ns | 226.23 ns | 256.49 ns | 203.01 ns | 10.31 us | 0.97x | 1.22x | 0.02x |
| `div` | 274.62 ns | 262.12 ns | 586.85 ns | 466.48 ns | 22.06 us | 0.47x | 0.59x | 0.01x |
| `div real` | 181.05 ns | 107.79 ns | 123.57 ns | 112.54 ns | 5.30 us | 1.47x | 1.61x | 0.03x |

#### Cold Complex Multiplication

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `varying exact inputs` | 220.33 ns | 268.91 ns | 479.05 ns | 368.44 ns | 10.18 us | 0.46x | 0.60x | 0.02x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 237.39 ns | 173.38 ns | 249.49 ns | 200.90 ns | 7.32 us | 0.95x | 1.18x | 0.03x |
| `vec3 magnitude` | 200.13 ns | 129.19 ns | 345.97 ns | 306.38 ns | 9.01 us | 0.58x | 0.65x | 0.02x |
| `vec3 normalize` | 473.77 ns | 1.86 us | 596.70 ns | 463.72 ns | 17.07 us | 0.79x | 1.02x | 0.03x |

#### Vector API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 198.93 ns | 506.21 ns | 65.21 ns | 65.32 ns | 756.94 ns | 3.05x | 3.05x | 0.26x |
| `vec3 zero` | 49.60 ns | 49.83 ns | 31.30 ns | 28.64 ns | 2.87 ns | 1.58x | 1.73x | 17.30x |
| `vec3 dot abort` | 255.25 ns | 148.62 ns | 207.24 ns | 156.08 ns | 7.45 us | 1.23x | 1.64x | 0.03x |
| `vec3 magnitude abort` | 226.92 ns | 223.93 ns | 321.39 ns | 282.88 ns | 9.07 us | 0.71x | 0.80x | 0.03x |
| `vec3 normalize checked` | 512.69 ns | 1.35 us | 542.83 ns | 428.14 ns | 17.44 us | 0.94x | 1.20x | 0.03x |
| `vec3 normalize checked abort` | 524.07 ns | 547.26 ns | 545.84 ns | 424.07 ns | 17.40 us | 0.96x | 1.24x | 0.03x |
| `vec3 div scalar checked` | 181.55 ns | 183.72 ns | 182.09 ns | 170.27 ns | 7.77 us | 1.00x | 1.07x | 0.02x |
| `vec3 div scalar checked abort` | 201.73 ns | 204.06 ns | 175.89 ns | 161.41 ns | 7.71 us | 1.15x | 1.25x | 0.03x |
| `vec3 add` | 122.60 ns | 128.07 ns | 126.71 ns | 86.18 ns | 4.10 us | 0.97x | 1.42x | 0.03x |
| `vec3 add scalar` | 147.43 ns | 192.82 ns | 133.87 ns | 113.19 ns | 3.93 us | 1.10x | 1.30x | 0.04x |
| `vec3 sub` | 126.04 ns | 209.05 ns | 137.49 ns | 118.05 ns | 7.52 us | 0.92x | 1.07x | 0.02x |
| `vec3 sub scalar` | 200.27 ns | 407.68 ns | 124.98 ns | 111.14 ns | 7.18 us | 1.60x | 1.80x | 0.03x |
| `vec3 neg` | 94.44 ns | 127.15 ns | 47.49 ns | 83.80 ns | 3.21 us | 1.99x | 1.13x | 0.03x |
| `vec3 mul scalar` | 251.70 ns | 556.21 ns | 122.44 ns | 126.13 ns | 4.42 us | 2.06x | 2.00x | 0.06x |
| `vec3 div scalar` | 140.19 ns | 228.52 ns | 176.25 ns | 161.09 ns | 7.80 us | 0.80x | 0.87x | 0.02x |
| `vec4 dot` | 421.52 ns | 255.59 ns | 325.76 ns | 249.68 ns | 9.89 us | 1.29x | 1.69x | 0.04x |
| `vec4 magnitude` | 408.25 ns | 377.53 ns | 410.58 ns | 369.81 ns | 22.38 us | 0.99x | 1.10x | 0.02x |
| `vec4 normalize` | 524.46 ns | 1.09 us | 716.38 ns | 535.37 ns | 23.59 us | 0.73x | 0.98x | 0.02x |
| `vec4 add` | 177.62 ns | 312.46 ns | 278.37 ns | 101.54 ns | 5.42 us | 0.64x | 1.75x | 0.03x |
| `vec4 add scalar` | 294.42 ns | 407.21 ns | 277.75 ns | 100.67 ns | 5.24 us | 1.06x | 2.92x | 0.06x |
| `vec4 sub` | 175.78 ns | 193.97 ns | 188.62 ns | 108.92 ns | 9.95 us | 0.93x | 1.61x | 0.02x |
| `vec4 sub scalar` | 345.19 ns | 364.19 ns | 171.97 ns | 99.89 ns | 16.64 us | 2.01x | 3.46x | 0.02x |
| `vec4 neg` | 154.34 ns | 103.06 ns | 68.39 ns | 61.90 ns | 4.13 us | 2.26x | 2.49x | 0.04x |
| `vec4 mul scalar` | 404.37 ns | 458.92 ns | 171.31 ns | 151.08 ns | 5.82 us | 2.36x | 2.68x | 0.07x |
| `vec4 div scalar` | 263.90 ns | 257.36 ns | 229.89 ns | 219.99 ns | 10.18 us | 1.15x | 1.20x | 0.03x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 524.84 ns | 410.88 ns | 840.84 ns | 705.06 ns | 22.53 us | 0.62x | 0.74x | 0.02x |
| `mat3 inverse` | 4.66 us | 2.11 us | 2.49 us | 2.07 us | 83.52 us | 1.87x | 2.25x | 0.06x |
| `mat3 mul mat3` | 1.43 us | 1.04 us | 2.38 us | 1.79 us | 61.69 us | 0.60x | 0.80x | 0.02x |
| `mat3 transform vec3` | 674.79 ns | 491.28 ns | 872.62 ns | 720.11 ns | 20.62 us | 0.77x | 0.94x | 0.03x |
| `mat4 determinant` | 1.20 us | 629.00 ns | 4.18 us | 4.37 us | 97.19 us | 0.29x | 0.27x | 0.01x |
| `mat4 inverse` | 7.46 us | 6.02 us | 9.16 us | 9.51 us | 348.82 us | 0.82x | 0.78x | 0.02x |
| `mat4 mul mat4` | 2.30 us | 1.88 us | 5.47 us | 5.94 us | 144.28 us | 0.42x | 0.39x | 0.02x |
| `mat4 transform vec4` | 997.54 ns | 583.87 ns | 1.67 us | 1.33 us | 35.72 us | 0.60x | 0.75x | 0.03x |

#### Matrix API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 724.14 ns | 975.32 ns | - | - | - | - | - | - |
| `mat3 zero` | 225.30 ns | 226.34 ns | - | - | - | - | - | - |
| `mat3 identity` | 245.62 ns | 247.07 ns | - | - | - | - | - | - |
| `mat3 transpose` | 226.17 ns | 219.94 ns | - | - | - | - | - | - |
| `mat3 reciprocal` | 4.79 us | 2.69 us | - | - | - | - | - | - |
| `mat3 reciprocal checked` | 4.79 us | 2.69 us | - | - | - | - | - | - |
| `mat3 inverse checked` | 4.74 us | 2.68 us | - | - | - | - | - | - |
| `mat3 inverse checked abort` | 4.87 us | 2.92 us | - | - | - | - | - | - |
| `mat3 powi` | 4.32 us | 4.27 us | - | - | - | - | - | - |
| `mat3 powi checked` | 4.24 us | 4.27 us | - | - | - | - | - | - |
| `mat3 powi checked abort` | 4.26 us | 4.27 us | - | - | - | - | - | - |
| `mat3 div scalar checked` | 562.57 ns | 594.65 ns | - | - | - | - | - | - |
| `mat3 div scalar checked abort` | 576.21 ns | 632.25 ns | - | - | - | - | - | - |
| `mat3 div matrix checked` | 23.46 us | 6.07 us | - | - | - | - | - | - |
| `mat3 div matrix checked abort` | 23.49 us | 6.23 us | - | - | - | - | - | - |
| `mat3 add` | 348.54 ns | 364.93 ns | - | - | - | - | - | - |
| `mat3 add scalar` | 584.41 ns | 603.83 ns | - | - | - | - | - | - |
| `mat3 sub` | 395.49 ns | 459.26 ns | - | - | - | - | - | - |
| `mat3 sub scalar` | 818.50 ns | 761.56 ns | - | - | - | - | - | - |
| `mat3 neg` | 226.74 ns | 228.34 ns | - | - | - | - | - | - |
| `mat3 mul scalar` | 567.75 ns | 704.57 ns | - | - | - | - | - | - |
| `mat3 div scalar` | 282.78 ns | 309.53 ns | - | - | - | - | - | - |
| `mat3 div matrix` | 23.70 us | 6.13 us | - | - | - | - | - | - |
| `mat3 bitxor` | 4.68 us | 4.29 us | - | - | - | - | - | - |
| `mat4 zero` | 241.72 ns | 220.14 ns | - | - | - | - | - | - |
| `mat4 identity` | 306.14 ns | 293.20 ns | - | - | - | - | - | - |
| `mat4 transpose` | 223.21 ns | 240.35 ns | - | - | - | - | - | - |
| `mat4 reciprocal` | 8.00 us | 6.61 us | - | - | - | - | - | - |
| `mat4 reciprocal checked` | 7.73 us | 6.64 us | - | - | - | - | - | - |
| `mat4 powi` | 5.39 us | 7.15 us | - | - | - | - | - | - |
| `mat4 powi checked` | 5.41 us | 10.57 us | - | - | - | - | - | - |
| `mat4 add` | 646.03 ns | 705.96 ns | - | - | - | - | - | - |
| `mat4 add scalar` | 816.91 ns | 1.01 us | - | - | - | - | - | - |
| `mat4 sub` | 744.29 ns | 833.47 ns | - | - | - | - | - | - |
| `mat4 sub scalar` | 1.28 us | 1.31 us | - | - | - | - | - | - |
| `mat4 neg` | 399.37 ns | 376.63 ns | - | - | - | - | - | - |
| `mat4 mul scalar` | 1.11 us | 1.32 us | - | - | - | - | - | - |
| `mat4 div scalar` | 474.51 ns | 651.27 ns | - | - | - | - | - | - |
| `mat4 div matrix` | 36.99 us | 9.95 us | - | - | - | - | - | - |
| `mat4 bitxor` | 5.33 us | - | - | - | - | - | - | - |

### Borrowed API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | - | - | - | - | - | - | - | - |
| `scalar add ref_owned` | - | - | - | - | - | - | - | - |
| `scalar add refs` | - | - | - | - | - | - | - | - |
| `scalar add owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar add ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `scalar sub owned_ref` | - | - | - | - | - | - | - | - |
| `scalar sub ref_owned` | - | - | - | - | - | - | - | - |
| `scalar sub refs` | - | - | - | - | - | - | - | - |
| `scalar sub owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar sub ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `scalar mul owned_ref` | - | - | - | - | - | - | - | - |
| `scalar mul ref_owned` | - | - | - | - | - | - | - | - |
| `scalar mul refs` | - | - | - | - | - | - | - | - |
| `scalar mul owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar mul ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `scalar div owned_ref` | - | - | - | - | - | - | - | - |
| `scalar div ref_owned` | - | - | - | - | - | - | - | - |
| `scalar div refs` | - | - | - | - | - | - | - | - |
| `scalar div owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar div ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `vec3 add refs` | - | - | - | - | - | - | - | - |
| `vec3 sub refs` | - | - | - | - | - | - | - | - |
| `vec3 neg ref` | - | - | - | - | - | - | - | - |
| `vec3 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec3 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec3 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec3 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 add refs` | - | - | - | - | - | - | - | - |
| `vec4 sub refs` | - | - | - | - | - | - | - | - |
| `vec4 neg ref` | - | - | - | - | - | - | - | - |
| `vec4 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 add refs` | - | - | - | - | - | - | - | - |
| `mat3 sub refs` | - | - | - | - | - | - | - | - |
| `mat3 mul refs` | - | - | - | - | - | - | - | - |
| `mat3 div refs` | - | - | - | - | - | - | - | - |
| `mat3 neg ref` | - | - | - | - | - | - | - | - |
| `mat3 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 add refs` | - | - | - | - | - | - | - | - |
| `mat4 sub refs` | - | - | - | - | - | - | - | - |
| `mat4 mul refs` | - | - | - | - | - | - | - | - |
| `mat4 div refs` | - | - | - | - | - | - | - | - |
| `mat4 neg ref` | - | - | - | - | - | - | - | - |
| `mat4 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 transform_vec refs` | - | - | - | - | - | - | - | - |
| `mat4 transform_vec refs` | - | - | - | - | - | - | - | - |
| `complex add refs` | - | - | - | - | - | - | - | - |
| `complex sub refs` | - | - | - | - | - | - | - | - |
| `complex mul refs` | - | - | - | - | - | - | - | - |
| `complex div refs` | - | - | - | - | - | - | - | - |
| `complex neg ref` | - | - | - | - | - | - | - | - |
| `complex div_real_ref` | - | - | - | - | - | - | - | - |
