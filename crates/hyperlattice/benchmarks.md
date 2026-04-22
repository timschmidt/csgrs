# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
cargo bench --bench mathbench --no-default-features --features approx-backend
```

The `mathbench` suite has two layers:

- Broad `realistic_blas` operation coverage for scalar, complex, vector, and
  matrix APIs.
- Cross-backend comparison coverage for vector dot/magnitude/normalize, matrix
  determinant/inverse/multiplication/transforms, and scalar trigonometric
  functions against `astro-float` and `arpfloat`.

The comparison backends run at 128-bit precision.

## Operation Coverage

The suite includes benchmarks for:

- Scalar construction/constants, arithmetic, reciprocal, checked reciprocal,
  power, exponentials, logarithms, square root, trigonometric functions,
  hyperbolic functions, inverse trigonometric and hyperbolic helpers,
  zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, checked
  reciprocal, powers, checked division, scalar conversion, arithmetic, and real
  scalar division.
- Vector construction, zero, dot product, abort-aware dot product, magnitude,
  abort-aware magnitude, normalization, checked normalization, abort-aware
  checked normalization, vector/vector arithmetic, vector/scalar arithmetic,
  and scalar division for both 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse,
  checked inverse, reciprocal, checked reciprocal, powers, checked powers,
  matrix/matrix arithmetic, matrix/scalar arithmetic, matrix multiplication,
  matrix/vector transformation, scalar division, matrix division, checked
  division, abort-aware checked operations, and `^` integer powers for 3x3 and
  4x4 matrices.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7
5800X3D on Fedora. They compare this crate's two scalar backends with
`astro-float` and `arpfloat` comparison backends. The `mathbench` comparison
suite runs both comparison backends at 128-bit precision.

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.53 ns | 4.44 us | 6.41 us | 313.11 ns | 743.93 ns | 679.03x | 14.17x | 5.96x |
| `vec3 mag` | 12.18 ns | 7.73 us | 10.31 us | 5.84 us | 15.05 us | 634.46x | 1.32x | 0.51x |
| `vec3 norm` | 32.12 ns | 13.07 us | 12.97 us | 6.46 us | 19.65 us | 406.98x | 2.02x | 0.67x |
| `mat3 det` | 15.93 ns | 14.10 us | 3.56 us | 1.06 us | 2.75 us | 885.47x | 13.35x | 5.12x |
| `mat3 inv` | 64.01 ns | 59.18 us | 14.68 us | 3.34 us | 11.32 us | 924.56x | 17.72x | 5.23x |
| `mat3 mul` | 73.44 ns | 42.10 us | 13.66 us | 2.89 us | 8.37 us | 573.27x | 14.58x | 5.03x |
| `mat3 x vec3` | 18.57 ns | 15.49 us | 12.33 us | 1.12 us | 2.75 us | 833.94x | 13.81x | 5.64x |
| `mat4 det` | 47.18 ns | 6.00 us | 6.01 us | 4.60 us | 9.20 us | 127.22x | 1.30x | 0.65x |
| `mat4 inv` | 162.16 ns | 21.91 us | 21.77 us | 12.10 us | 34.96 us | 135.14x | 1.81x | 0.63x |
| `mat4 mul` | 120.50 ns | 16.37 us | 16.39 us | 6.18 us | 16.49 us | 135.86x | 2.65x | 0.99x |
| `mat4 x vec4` | 29.51 ns | 5.16 us | 5.15 us | 1.87 us | 4.47 us | 174.70x | 2.76x | 1.15x |
| `sin 0.1` | 10.90 ns | 2.31 us | 2.32 us | 10.70 us | 125.74 us | 212.01x | 0.22x | 0.02x |
| `cos 0.1` | 11.61 ns | 226.48 ns | 225.86 ns | 10.03 us | 25.39 us | 19.50x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.71 ns | 1.81 us | 1.79 us | 11.95 us | 266.55 us | 154.40x | 0.15x | 0.01x |
| `cos 1.23456789` | 12.04 ns | 448.25 ns | 404.03 ns | 10.04 us | 173.18 us | 37.23x | 0.04x | 0.00x |
| `sin 1e6` | 12.60 ns | 35.99 us | 36.31 us | 15.69 us | 270.72 us | 2856.21x | 2.29x | 0.13x |
| `cos 1e6` | 12.27 ns | 22.16 us | 22.32 us | 13.97 us | 174.91 us | 1805.90x | 1.59x | 0.13x |
| `sin 1e30` | 66.03 ns | 194.06 us | 192.59 us | 17.86 us | 276.75 us | 2938.79x | 10.86x | 0.70x |
| `cos 1e30` | 68.46 ns | 143.12 us | 141.68 us | 15.77 us | 175.16 us | 2090.56x | 9.08x | 0.82x |
| `sin pi_7` | 11.65 ns | 2.43 us | 4.36 us | 11.72 us | 124.51 us | 208.94x | 0.21x | 0.02x |
| `cos pi_7` | 11.55 ns | 214.27 ns | 4.59 us | 10.63 us | 27.96 us | 18.55x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.75 ns | 23.50 us | 34.99 us | 15.70 us | 265.79 us | 2000.68x | 1.50x | 0.09x |
| `cos 1000pi_eps` | 12.30 ns | 13.99 us | 25.54 us | 13.95 us | 155.48 us | 1137.31x | 1.00x | 0.09x |

## Operation Benchmark Results

The following tables cover the remaining `realistic_blas` API operation
benchmarks. These benchmarks compare only the crate's `approx` and
`realistic` scalar backends.

### Scalar Operations

| Benchmark | Approx | Realistic from f64 | Realistic f64 / approx |
| --- | ---: | ---: | ---: |
| `zero` | 0.47 ns | 39.97 ns | 84.36x |
| `one` | 0.48 ns | 69.48 ns | 146.19x |
| `e` | 0.47 ns | 1.20 us | 2546.90x |
| `pi` | 0.47 ns | 225.04 ns | 474.84x |
| `tau` | 0.47 ns | 358.23 ns | 761.58x |
| `add` | 5.08 ns | 303.78 ns | 59.82x |
| `sub` | 5.06 ns | 291.12 ns | 57.51x |
| `neg` | 4.83 ns | 60.99 ns | 12.63x |
| `mul` | 5.08 ns | 223.48 ns | 44.03x |
| `div` | 10.89 ns | 270.89 ns | 24.88x |
| `reciprocal` | 8.22 ns | 95.07 ns | 11.56x |
| `reciprocal checked` | 8.82 ns | 95.05 ns | 10.78x |
| `reciprocal checked abort` | 26.50 ns | 167.61 ns | 6.32x |
| `pow` | 19.83 ns | 6.31 us | 318.26x |
| `powi` | 7.59 ns | 949.16 ns | 125.03x |
| `exp` | 10.10 ns | 1.41 us | 139.21x |
| `ln` | 11.45 ns | 1.94 us | 169.07x |
| `log10` | 13.15 ns | 8.73 us | 664.13x |
| `log10 abort` | 18.84 ns | 8.80 us | 466.87x |
| `sqrt` | 8.17 ns | 222.62 ns | 27.24x |
| `sin` | 11.63 ns | 2.15 us | 184.79x |
| `cos` | 11.58 ns | 214.93 ns | 18.55x |
| `tan` | 22.87 ns | 2.40 us | 104.94x |
| `sinh` | 17.36 ns | 3.06 us | 175.98x |
| `cosh` | 17.35 ns | 3.03 us | 174.72x |
| `tanh` | 44.96 ns | 6.66 us | 148.21x |
| `asin` | 7.04 ns | 879.11 ns | 124.90x |
| `asin abort` | 11.98 ns | 883.80 ns | 73.80x |
| `acos` | 7.35 ns | 885.59 ns | 120.43x |
| `acos abort` | 11.99 ns | 891.16 ns | 74.31x |
| `atan` | 11.45 ns | 913.92 ns | 79.82x |
| `atan abort` | 16.37 ns | 922.65 ns | 56.38x |
| `asinh` | 20.25 ns | 956.07 ns | 47.22x |
| `asinh abort` | 24.56 ns | 930.32 ns | 37.88x |
| `acosh` | 25.68 ns | 687.75 ns | 26.78x |
| `acosh abort` | 12.00 ns | 691.78 ns | 57.66x |
| `atanh` | 9.68 ns | 910.12 ns | 94.05x |
| `atanh abort` | 12.35 ns | 915.69 ns | 74.14x |
| `zero status` | 1.01 ns | 2.58 ns | 2.56x |
| `zero status abort` | 3.32 ns | 57.40 ns | 17.27x |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic f64 / approx |
| --- | ---: | ---: | ---: |
| `zero` | 0.94 ns | 85.71 ns | 91.52x |
| `one` | 5.27 ns | 117.60 ns | 22.32x |
| `i` | 0.94 ns | 113.82 ns | 121.54x |
| `free i` | 1.18 ns | 112.62 ns | 95.52x |
| `conjugate` | 1.87 ns | 134.44 ns | 71.99x |
| `norm squared` | 5.53 ns | 353.98 ns | 63.97x |
| `reciprocal` | 10.73 ns | 875.90 ns | 81.64x |
| `reciprocal checked` | 11.68 ns | 884.08 ns | 75.67x |
| `powi` | 18.73 ns | 2.78 us | 148.51x |
| `powi checked` | 18.73 ns | 2.78 us | 148.25x |
| `div checked` | 28.06 ns | 2.09 us | 74.54x |
| `div real checked` | 17.96 ns | 581.37 ns | 32.37x |
| `from scalar` | 1.18 ns | 100.20 ns | 85.02x |
| `add` | 5.92 ns | 446.29 ns | 75.41x |
| `sub` | 5.90 ns | 467.26 ns | 79.15x |
| `neg` | 2.34 ns | 142.83 ns | 61.09x |
| `mul` | 7.31 ns | 1.08 us | 148.30x |
| `div` | 22.37 ns | 2.11 us | 94.19x |
| `div real` | 17.63 ns | 600.46 ns | 34.06x |

### Vector Operations

| Benchmark | Approx | Realistic from f64 | Realistic f64 / approx |
| --- | ---: | ---: | ---: |
| `vec3 new` | 1.40 ns | 673.68 ns | 482.81x |
| `vec3 zero` | 1.40 ns | 157.33 ns | 112.40x |
| `vec3 dot abort` | 27.55 ns | 4.60 us | 166.95x |
| `vec3 magnitude abort` | 38.77 ns | 7.89 us | 203.51x |
| `vec3 normalize checked` | 32.87 ns | 13.24 us | 402.71x |
| `vec3 normalize checked abort` | 61.16 ns | 13.42 us | 219.43x |
| `vec3 div scalar checked` | 10.00 ns | 2.00 us | 200.15x |
| `vec3 div scalar checked abort` | 29.23 ns | 2.11 us | 72.12x |
| `vec3 add` | 7.23 ns | 2.95 us | 408.42x |
| `vec3 add scalar` | 6.16 ns | 2.13 us | 345.74x |
| `vec3 sub` | 7.25 ns | 2.72 us | 374.87x |
| `vec3 sub scalar` | 6.23 ns | 2.18 us | 349.34x |
| `vec3 neg` | 3.52 ns | 510.44 ns | 144.99x |
| `vec3 mul scalar` | 6.42 ns | 2.12 us | 330.53x |
| `vec3 div scalar` | 10.01 ns | 2.03 us | 203.01x |
| `vec4 dot` | 7.36 ns | 781.72 ns | 106.14x |
| `vec4 magnitude` | 13.81 ns | 2.83 us | 204.68x |
| `vec4 normalize` | 37.41 ns | 4.88 us | 130.49x |
| `vec4 add` | 7.10 ns | 1.50 us | 211.77x |
| `vec4 add scalar` | 6.60 ns | 1.21 us | 183.40x |
| `vec4 sub` | 4.75 ns | 1.50 us | 316.38x |
| `vec4 sub scalar` | 4.98 ns | 1.16 us | 233.71x |
| `vec4 neg` | 4.67 ns | 725.78 ns | 155.51x |
| `vec4 mul scalar` | 7.06 ns | 1.18 us | 166.52x |
| `vec4 div scalar` | 14.03 ns | 1.36 us | 97.27x |

### Matrix Operations

| Benchmark | Approx | Realistic from f64 | Realistic f64 / approx |
| --- | ---: | ---: | ---: |
| `mat3 new` | 10.15 ns | 5.01 us | 493.84x |
| `mat3 zero` | 16.87 ns | 764.59 ns | 45.33x |
| `mat3 identity` | 10.17 ns | 845.45 ns | 83.16x |
| `mat3 transpose` | 9.55 ns | 965.81 ns | 101.15x |
| `mat3 reciprocal` | 63.54 ns | 59.35 us | 933.96x |
| `mat3 reciprocal checked` | 141.60 ns | 83.86 us | 592.26x |
| `mat3 inverse checked` | 141.73 ns | 84.45 us | 595.83x |
| `mat3 inverse checked abort` | 161.82 ns | 84.85 us | 524.33x |
| `mat3 powi` | 164.16 ns | 122.19 us | 744.30x |
| `mat3 powi checked` | 162.79 ns | 121.92 us | 748.96x |
| `mat3 powi checked abort` | 164.37 ns | 122.57 us | 745.66x |
| `mat3 div scalar checked` | 24.50 ns | 5.77 us | 235.42x |
| `mat3 div scalar checked abort` | 39.66 ns | 5.76 us | 145.32x |
| `mat3 div matrix checked` | 237.47 ns | 193.15 us | 813.38x |
| `mat3 div matrix checked abort` | 241.58 ns | 194.00 us | 803.04x |
| `mat3 add` | 12.65 ns | 9.12 us | 721.15x |
| `mat3 add scalar` | 10.22 ns | 6.41 us | 626.79x |
| `mat3 sub` | 13.77 ns | 8.57 us | 621.93x |
| `mat3 sub scalar` | 10.67 ns | 6.45 us | 604.22x |
| `mat3 neg` | 10.93 ns | 2.11 us | 192.85x |
| `mat3 mul scalar` | 11.93 ns | 6.48 us | 543.57x |
| `mat3 div scalar` | 23.96 ns | 5.78 us | 241.38x |
| `mat3 div matrix` | 141.59 ns | 168.54 us | 1190.36x |
| `mat3 bitxor` | 163.73 ns | 122.66 us | 749.16x |
| `mat4 zero` | 13.21 ns | 1.36 us | 102.84x |
| `mat4 identity` | 10.12 ns | 1.50 us | 148.40x |
| `mat4 transpose` | 8.93 ns | 1.74 us | 195.28x |
| `mat4 reciprocal` | 133.68 ns | 21.96 us | 164.30x |
| `mat4 reciprocal checked` | 197.71 ns | 41.83 us | 211.59x |
| `mat4 powi` | 324.31 ns | 47.71 us | 147.11x |
| `mat4 powi checked` | 325.27 ns | 48.93 us | 150.43x |
| `mat4 add` | 50.20 ns | 6.77 us | 134.92x |
| `mat4 add scalar` | 16.23 ns | 5.16 us | 317.69x |
| `mat4 sub` | 36.67 ns | 6.93 us | 188.89x |
| `mat4 sub scalar` | 14.60 ns | 5.33 us | 364.81x |
| `mat4 neg` | 13.59 ns | 3.59 us | 264.35x |
| `mat4 mul scalar` | 49.43 ns | 5.11 us | 103.36x |
| `mat4 div scalar` | 32.39 ns | 5.60 us | 172.77x |
| `mat4 div matrix` | 286.95 ns | 49.76 us | 173.42x |
| `mat4 bitxor` | 323.36 ns | 47.44 us | 146.71x |
