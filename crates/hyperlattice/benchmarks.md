# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
```

The `mathbench` suite benchmarks both crate backends and writes this file from Criterion's median estimates after a real benchmark run. The `astro-float` and `arpfloat` comparison columns run at 128-bit precision. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated, or that the external library does not expose a directly comparable operation in this suite.

Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, and range-reduction-heavy trigonometric arguments.

## Operation Coverage

- Scalar construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.
- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Scalar Operations

#### Scalar Trigonometric Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 10.90 ns | 2.30 us | 2.31 us | 10.39 us | 123.69 us | 211.29x | 0.22x | 0.02x |
| `cos 0.1` | 11.61 ns | 225.30 ns | 224.89 ns | 9.99 us | 25.31 us | 19.40x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.68 ns | 1.80 us | 1.79 us | 11.82 us | 262.18 us | 153.97x | 0.15x | 0.01x |
| `cos 1.23456789` | 11.97 ns | 446.90 ns | 401.66 ns | 10.09 us | 171.76 us | 37.33x | 0.04x | 0.00x |
| `sin 1e6` | 12.57 ns | 35.74 us | 36.13 us | 15.65 us | 266.97 us | 2842.98x | 2.28x | 0.13x |
| `cos 1e6` | 12.28 ns | 21.93 us | 22.18 us | 13.94 us | 172.02 us | 1785.94x | 1.57x | 0.13x |
| `sin 1e30` | 65.84 ns | 193.02 us | 191.57 us | 17.98 us | 271.55 us | 2931.75x | 10.73x | 0.71x |
| `cos 1e30` | 68.13 ns | 141.84 us | 140.43 us | 15.56 us | 171.41 us | 2081.78x | 9.11x | 0.83x |
| `sin pi_7` | 11.65 ns | 2.42 us | 4.34 us | 11.46 us | 122.80 us | 208.07x | 0.21x | 0.02x |
| `cos pi_7` | 11.57 ns | 213.33 ns | 4.56 us | 10.24 us | 27.42 us | 18.44x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.71 ns | 23.29 us | 34.65 us | 15.77 us | 261.52 us | 1988.71x | 1.48x | 0.09x |
| `cos 1000pi_eps` | 12.30 ns | 13.78 us | 25.21 us | 13.89 us | 154.96 us | 1120.20x | 0.99x | 0.09x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 39.97 ns | - | - | - | 84.36x | - | - |
| `one` | 0.48 ns | 69.48 ns | - | - | - | 146.19x | - | - |
| `e` | 0.47 ns | 1.20 us | - | - | - | 2546.90x | - | - |
| `pi` | 0.47 ns | 225.04 ns | - | - | - | 474.84x | - | - |
| `tau` | 0.47 ns | 358.23 ns | - | - | - | 761.58x | - | - |
| `add` | 5.08 ns | 303.78 ns | - | - | - | 59.82x | - | - |
| `sub` | 5.06 ns | 291.12 ns | - | - | - | 57.51x | - | - |
| `neg` | 4.83 ns | 60.99 ns | - | - | - | 12.63x | - | - |
| `mul` | 5.08 ns | 223.48 ns | - | - | - | 44.03x | - | - |
| `div` | 10.89 ns | 270.89 ns | - | - | - | 24.88x | - | - |
| `reciprocal` | 8.22 ns | 95.07 ns | - | - | - | 11.56x | - | - |
| `reciprocal checked` | 8.82 ns | 95.05 ns | - | - | - | 10.78x | - | - |
| `reciprocal checked abort` | 26.50 ns | 167.61 ns | - | - | - | 6.32x | - | - |
| `pow` | 19.83 ns | 6.31 us | - | - | - | 318.26x | - | - |
| `powi` | 7.59 ns | 949.16 ns | - | - | - | 125.03x | - | - |
| `exp` | 10.10 ns | 1.41 us | - | - | - | 139.21x | - | - |
| `ln` | 11.45 ns | 1.94 us | - | - | - | 169.07x | - | - |
| `log10` | 13.15 ns | 8.73 us | - | - | - | 664.13x | - | - |
| `log10 abort` | 18.84 ns | 8.80 us | - | - | - | 466.87x | - | - |
| `sqrt` | 8.17 ns | 222.62 ns | - | - | - | 27.24x | - | - |
| `sin` | 11.63 ns | 2.15 us | - | - | - | 184.79x | - | - |
| `cos` | 11.58 ns | 214.93 ns | - | - | - | 18.55x | - | - |
| `tan` | 22.87 ns | 2.40 us | - | - | - | 104.94x | - | - |
| `sinh` | 17.36 ns | 3.06 us | - | - | - | 175.98x | - | - |
| `cosh` | 17.35 ns | 3.03 us | - | - | - | 174.72x | - | - |
| `tanh` | 44.96 ns | 6.66 us | - | - | - | 148.21x | - | - |
| `asin` | 7.04 ns | 879.11 ns | - | - | - | 124.90x | - | - |
| `asin abort` | 11.98 ns | 883.80 ns | - | - | - | 73.80x | - | - |
| `acos` | 7.35 ns | 885.59 ns | - | - | - | 120.43x | - | - |
| `acos abort` | 11.99 ns | 891.16 ns | - | - | - | 74.31x | - | - |
| `atan` | 11.45 ns | 913.92 ns | - | - | - | 79.82x | - | - |
| `atan abort` | 16.37 ns | 922.65 ns | - | - | - | 56.38x | - | - |
| `asinh` | 20.25 ns | 956.07 ns | - | - | - | 47.22x | - | - |
| `asinh abort` | 24.56 ns | 930.32 ns | - | - | - | 37.88x | - | - |
| `acosh` | 25.68 ns | 687.75 ns | - | - | - | 26.78x | - | - |
| `acosh abort` | 12.00 ns | 691.78 ns | - | - | - | 57.66x | - | - |
| `atanh` | 9.68 ns | 910.12 ns | - | - | - | 94.05x | - | - |
| `atanh abort` | 12.35 ns | 915.69 ns | - | - | - | 74.14x | - | - |
| `zero status` | 1.01 ns | 2.58 ns | - | - | - | 2.56x | - | - |
| `zero status abort` | 3.32 ns | 57.40 ns | - | - | - | 17.27x | - | - |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.94 ns | 85.71 ns | - | - | - | 91.52x | - | - |
| `one` | 5.27 ns | 117.60 ns | - | - | - | 22.32x | - | - |
| `i` | 0.94 ns | 113.82 ns | - | - | - | 121.54x | - | - |
| `free i` | 1.18 ns | 112.62 ns | - | - | - | 95.52x | - | - |
| `conjugate` | 1.87 ns | 134.44 ns | - | - | - | 71.99x | - | - |
| `norm squared` | 5.53 ns | 353.98 ns | - | - | - | 63.97x | - | - |
| `reciprocal` | 10.73 ns | 875.90 ns | - | - | - | 81.64x | - | - |
| `reciprocal checked` | 11.68 ns | 884.08 ns | - | - | - | 75.67x | - | - |
| `powi` | 18.73 ns | 2.78 us | - | - | - | 148.51x | - | - |
| `powi checked` | 18.73 ns | 2.78 us | - | - | - | 148.25x | - | - |
| `div checked` | 28.06 ns | 2.09 us | - | - | - | 74.54x | - | - |
| `div real checked` | 17.96 ns | 581.37 ns | - | - | - | 32.37x | - | - |
| `from scalar` | 1.18 ns | 100.20 ns | - | - | - | 85.02x | - | - |
| `add` | 5.92 ns | 446.29 ns | - | - | - | 75.41x | - | - |
| `sub` | 5.90 ns | 467.26 ns | - | - | - | 79.15x | - | - |
| `neg` | 2.34 ns | 142.83 ns | - | - | - | 61.09x | - | - |
| `mul` | 7.31 ns | 1.08 us | - | - | - | 148.30x | - | - |
| `div` | 22.37 ns | 2.11 us | - | - | - | 94.19x | - | - |
| `div real` | 17.63 ns | 600.46 ns | - | - | - | 34.06x | - | - |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.52 ns | 4.43 us | 6.41 us | 311.44 ns | 728.66 ns | 679.41x | 14.22x | 6.08x |
| `vec3 magnitude` | 12.12 ns | 7.70 us | 10.38 us | 5.94 us | 15.02 us | 635.18x | 1.30x | 0.51x |
| `vec3 normalize` | 32.14 ns | 13.12 us | 12.94 us | 6.44 us | 19.85 us | 408.14x | 2.04x | 0.66x |

#### Vector API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 1.40 ns | 673.68 ns | - | - | - | 482.81x | - | - |
| `vec3 zero` | 1.40 ns | 157.33 ns | - | - | - | 112.40x | - | - |
| `vec3 dot abort` | 27.55 ns | 4.60 us | - | - | - | 166.95x | - | - |
| `vec3 magnitude abort` | 38.77 ns | 7.89 us | - | - | - | 203.51x | - | - |
| `vec3 normalize checked` | 32.87 ns | 13.24 us | - | - | - | 402.71x | - | - |
| `vec3 normalize checked abort` | 61.16 ns | 13.42 us | - | - | - | 219.43x | - | - |
| `vec3 div scalar checked` | 10.00 ns | 2.00 us | - | - | - | 200.15x | - | - |
| `vec3 div scalar checked abort` | 29.23 ns | 2.11 us | - | - | - | 72.12x | - | - |
| `vec3 add` | 7.23 ns | 2.95 us | - | - | - | 408.42x | - | - |
| `vec3 add scalar` | 6.16 ns | 2.13 us | - | - | - | 345.74x | - | - |
| `vec3 sub` | 7.25 ns | 2.72 us | - | - | - | 374.87x | - | - |
| `vec3 sub scalar` | 6.23 ns | 2.18 us | - | - | - | 349.34x | - | - |
| `vec3 neg` | 3.52 ns | 510.44 ns | - | - | - | 144.99x | - | - |
| `vec3 mul scalar` | 6.42 ns | 2.12 us | - | - | - | 330.53x | - | - |
| `vec3 div scalar` | 10.01 ns | 2.03 us | - | - | - | 203.01x | - | - |
| `vec4 dot` | 7.36 ns | 781.72 ns | - | - | - | 106.14x | - | - |
| `vec4 magnitude` | 13.81 ns | 2.83 us | - | - | - | 204.68x | - | - |
| `vec4 normalize` | 37.41 ns | 4.88 us | - | - | - | 130.49x | - | - |
| `vec4 add` | 7.10 ns | 1.50 us | - | - | - | 211.77x | - | - |
| `vec4 add scalar` | 6.60 ns | 1.21 us | - | - | - | 183.40x | - | - |
| `vec4 sub` | 4.75 ns | 1.50 us | - | - | - | 316.38x | - | - |
| `vec4 sub scalar` | 4.98 ns | 1.16 us | - | - | - | 233.71x | - | - |
| `vec4 neg` | 4.67 ns | 725.78 ns | - | - | - | 155.51x | - | - |
| `vec4 mul scalar` | 7.06 ns | 1.18 us | - | - | - | 166.52x | - | - |
| `vec4 div scalar` | 14.03 ns | 1.36 us | - | - | - | 97.27x | - | - |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 15.88 ns | 14.06 us | 3.55 us | 1.05 us | 2.73 us | 885.26x | 13.38x | 5.16x |
| `mat3 inverse` | 63.73 ns | 59.08 us | 14.64 us | 3.24 us | 10.91 us | 927.03x | 18.23x | 5.42x |
| `mat3 mul mat3` | 73.10 ns | 41.96 us | 13.61 us | 2.87 us | 8.33 us | 574.03x | 14.64x | 5.04x |
| `mat3 transform vec3` | 18.50 ns | 15.36 us | 12.28 us | 1.11 us | 2.68 us | 830.33x | 13.90x | 5.72x |
| `mat4 determinant` | 46.79 ns | 5.97 us | 5.98 us | 4.55 us | 8.91 us | 127.53x | 1.31x | 0.67x |
| `mat4 inverse` | 162.09 ns | 21.71 us | 21.63 us | 11.86 us | 34.27 us | 133.93x | 1.83x | 0.63x |
| `mat4 mul mat4` | 119.48 ns | 16.29 us | 16.29 us | 6.14 us | 16.54 us | 136.36x | 2.65x | 0.98x |
| `mat4 transform vec4` | 29.20 ns | 5.12 us | 5.13 us | 1.89 us | 4.39 us | 175.19x | 2.70x | 1.17x |

#### Matrix API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 10.15 ns | 5.01 us | - | - | - | 493.84x | - | - |
| `mat3 zero` | 16.87 ns | 764.59 ns | - | - | - | 45.33x | - | - |
| `mat3 identity` | 10.17 ns | 845.45 ns | - | - | - | 83.16x | - | - |
| `mat3 transpose` | 9.55 ns | 965.81 ns | - | - | - | 101.15x | - | - |
| `mat3 reciprocal` | 63.54 ns | 59.35 us | - | - | - | 933.96x | - | - |
| `mat3 reciprocal checked` | 141.60 ns | 83.86 us | - | - | - | 592.26x | - | - |
| `mat3 inverse checked` | 141.73 ns | 84.45 us | - | - | - | 595.83x | - | - |
| `mat3 inverse checked abort` | 161.82 ns | 84.85 us | - | - | - | 524.33x | - | - |
| `mat3 powi` | 164.16 ns | 122.19 us | - | - | - | 744.30x | - | - |
| `mat3 powi checked` | 162.79 ns | 121.92 us | - | - | - | 748.96x | - | - |
| `mat3 powi checked abort` | 164.37 ns | 122.57 us | - | - | - | 745.66x | - | - |
| `mat3 div scalar checked` | 24.50 ns | 5.77 us | - | - | - | 235.42x | - | - |
| `mat3 div scalar checked abort` | 39.66 ns | 5.76 us | - | - | - | 145.32x | - | - |
| `mat3 div matrix checked` | 237.47 ns | 193.15 us | - | - | - | 813.38x | - | - |
| `mat3 div matrix checked abort` | 241.58 ns | 194.00 us | - | - | - | 803.04x | - | - |
| `mat3 add` | 12.65 ns | 9.12 us | - | - | - | 721.15x | - | - |
| `mat3 add scalar` | 10.22 ns | 6.41 us | - | - | - | 626.79x | - | - |
| `mat3 sub` | 13.77 ns | 8.57 us | - | - | - | 621.93x | - | - |
| `mat3 sub scalar` | 10.67 ns | 6.45 us | - | - | - | 604.22x | - | - |
| `mat3 neg` | 10.93 ns | 2.11 us | - | - | - | 192.85x | - | - |
| `mat3 mul scalar` | 11.93 ns | 6.48 us | - | - | - | 543.57x | - | - |
| `mat3 div scalar` | 23.96 ns | 5.78 us | - | - | - | 241.38x | - | - |
| `mat3 div matrix` | 141.59 ns | 168.54 us | - | - | - | 1190.36x | - | - |
| `mat3 bitxor` | 163.73 ns | 122.66 us | - | - | - | 749.16x | - | - |
| `mat4 zero` | 13.21 ns | 1.36 us | - | - | - | 102.84x | - | - |
| `mat4 identity` | 10.12 ns | 1.50 us | - | - | - | 148.40x | - | - |
| `mat4 transpose` | 8.93 ns | 1.74 us | - | - | - | 195.28x | - | - |
| `mat4 reciprocal` | 133.68 ns | 21.96 us | - | - | - | 164.30x | - | - |
| `mat4 reciprocal checked` | 197.71 ns | 41.83 us | - | - | - | 211.59x | - | - |
| `mat4 powi` | 324.31 ns | 47.71 us | - | - | - | 147.11x | - | - |
| `mat4 powi checked` | 325.27 ns | 48.93 us | - | - | - | 150.43x | - | - |
| `mat4 add` | 50.20 ns | 6.77 us | - | - | - | 134.92x | - | - |
| `mat4 add scalar` | 16.23 ns | 5.16 us | - | - | - | 317.69x | - | - |
| `mat4 sub` | 36.67 ns | 6.93 us | - | - | - | 188.89x | - | - |
| `mat4 sub scalar` | 14.60 ns | 5.33 us | - | - | - | 364.81x | - | - |
| `mat4 neg` | 13.59 ns | 3.59 us | - | - | - | 264.35x | - | - |
| `mat4 mul scalar` | 49.43 ns | 5.11 us | - | - | - | 103.36x | - | - |
| `mat4 div scalar` | 32.39 ns | 5.60 us | - | - | - | 172.77x | - | - |
| `mat4 div matrix` | 286.95 ns | 49.76 us | - | - | - | 173.42x | - | - |
| `mat4 bitxor` | 323.36 ns | 47.44 us | - | - | - | 146.71x | - | - |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 11.85 us |
| `astro sin 160` | 12.94 us |
| `astro sin 192` | 12.91 us |
| `astro sin 256` | 15.40 us |
| `arp sin 128` | 262.20 us |
| `arp sin 160` | 329.60 us |
| `arp sin 192` | 392.35 us |
| `arp sin 256` | 598.59 us |
