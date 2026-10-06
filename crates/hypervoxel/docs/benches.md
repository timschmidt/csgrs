# Voxelis Benchmarks

This document contains benchmark results for the Voxelis voxel engine, measuring the performance of various operations on the VoxTree (SVO DAG) structure. The tests compare the performance of single operations versus batch operations, as well as performance across different LOD (Level of Detail) levels.

## Methodology

Benchmarks were performed using the Rust `criterion` library. Each test was run on four different octree sizes (8³, 16³, 32³, 64³), corresponding to tree depths 3-6. The hardware specifications for these benchmarks are as follows:

- CPU: Apple M3 Max
- RAM: 128 GB
- OS: macOS Sequoia 15.4.1
- Rust Version: 1.86.0
- RUSTFLAGS: `-C target-cpu=native`
- Profile: `final`
- To run the benchmarks on your machine, use the command:

  ```bash
  cargo bench --profile=final --bench voxtree_bench
  ```

- or if you want to run specific benchmarks:

  ```bash
  cargo bench --profile=final --bench voxtree_bench -- voxtree_set_terrain_
  ```

## Table of Contents

- [Voxelis Benchmarks](#voxelis-benchmarks)
  - [Methodology](#methodology)
  - [Table of Contents](#table-of-contents)
  - [Basic Operations](#basic-operations)
    - [voxtree\_create](#voxtree_create)
    - [voxtree\_fill](#voxtree_fill)
    - [voxtree\_set\_single\_voxel](#voxtree_set_single_voxel)
  - [Modification Operations](#modification-operations)
    - [voxtree\_fill\_then\_set\_single\_voxel](#voxtree_fill_then_set_single_voxel)
    - [voxtree\_set\_uniform](#voxtree_set_uniform)
    - [voxtree\_set\_uniform\_half](#voxtree_set_uniform_half)
    - [voxtree\_set\_sum](#voxtree_set_sum)
    - [voxtree\_set\_checkerboard](#voxtree_set_checkerboard)
    - [voxtree\_set\_sparse\_fill](#voxtree_set_sparse_fill)
    - [voxtree\_set\_gradient\_fill](#voxtree_set_gradient_fill)
    - [voxtree\_set\_hollow\_cube](#voxtree_set_hollow_cube)
    - [voxtree\_set\_diagonal](#voxtree_set_diagonal)
    - [voxtree\_set\_sphere](#voxtree_set_sphere)
    - [voxtree\_set\_terrain\_surface\_only](#voxtree_set_terrain_surface_only)
    - [voxtree\_set\_terrain\_surface\_and\_below](#voxtree_set_terrain_surface_and_below)
    - [voxtree\_set\_random\_position\_same\_value](#voxtree_set_random_position_same_value)
    - [voxtree\_set\_random\_position\_and\_value](#voxtree_set_random_position_and_value)
  - [Read Operations](#read-operations)
    - [voxtree\_get\_empty](#voxtree_get_empty)
    - [voxtree\_get\_sphere\_uniform](#voxtree_get_sphere_uniform)
    - [voxtree\_get\_sphere\_sum](#voxtree_get_sphere_sum)
    - [voxtree\_get\_full\_uniform](#voxtree_get_full_uniform)
    - [voxtree\_get\_full\_sum](#voxtree_get_full_sum)
  - [State Operations](#state-operations)
    - [voxtree\_is\_empty\_empty](#voxtree_is_empty_empty)
    - [voxtree\_is\_empty\_not\_empty](#voxtree_is_empty_not_empty)
    - [voxtree\_clear\_empty](#voxtree_clear_empty)
    - [voxtree\_clear\_sphere](#voxtree_clear_sphere)
    - [voxtree\_clear\_filled](#voxtree_clear_filled)
  - [LOD Operations](#lod-operations)
    - [voxtree\_to\_vec\_empty](#voxtree_to_vec_empty)
    - [voxtree\_to\_vec\_sphere](#voxtree_to_vec_sphere)
    - [voxtree\_to\_vec\_uniform](#voxtree_to_vec_uniform)
    - [voxtree\_to\_vec\_sum](#voxtree_to_vec_sum)
    - [voxtree\_to\_vec\_terrain](#voxtree_to_vec_terrain)

## Basic Operations

### voxtree_create

Creates a new VoxTree instance.

```rust
// Example code
let tree = VoxTree::new(depth);
```

```accesslog
voxtree_create           time:   [542.40 ps 543.23 ps 544.15 ps]
```

### voxtree_fill

Fills the entire VoxTree with a single value. Compares the performance of single operations versus batch operations for different octree sizes.

```rust
// Single operation
tree.fill(&mut interner, value);

// Batch operation
let mut batch = tree.create_batch();
batch.fill(&mut interner, value);
tree.apply_batch(&mut interner, &batch);
```

```accesslog
voxtree_fill/8/single    time:   [9.1261 ns 9.2114 ns 9.3173 ns]
voxtree_fill/8/batch     time:   [10.386 ns 10.461 ns 10.543 ns]
voxtree_fill/16/single   time:   [8.8781 ns 8.9194 ns 8.9640 ns]
voxtree_fill/16/batch    time:   [10.355 ns 10.429 ns 10.511 ns]
voxtree_fill/32/single   time:   [8.9483 ns 8.9841 ns 9.0285 ns]
voxtree_fill/32/batch    time:   [10.384 ns 10.604 ns 10.907 ns]
voxtree_fill/64/single   time:   [8.9025 ns 8.9338 ns 8.9657 ns]
voxtree_fill/64/batch    time:   [10.267 ns 10.305 ns 10.353 ns]
```

### voxtree_set_single_voxel

Sets a single voxel in the VoxTree to a specific value. This tests the overhead of the Copy-on-Write mechanism for modifying a single point.

```rust
// Single operation
tree.set(&mut interner, IVec3::new(0, 0, 0), value);

// Batch operation
let mut batch = tree.create_batch();
batch.set(&mut interner, IVec3::new(0, 0, 0), value);
tree.apply_batch(&mut interner, &batch);
```

```accesslog
voxtree_set_single_voxel/8/single
                        time:   [75.327 ns 75.512 ns 75.695 ns]
voxtree_set_single_voxel/8/batch
                        time:   [244.62 ns 245.83 ns 247.01 ns]
voxtree_set_single_voxel/16/single
                        time:   [92.713 ns 92.986 ns 93.296 ns]
voxtree_set_single_voxel/16/batch
                        time:   [471.02 ns 478.77 ns 486.45 ns]
voxtree_set_single_voxel/32/single
                        time:   [113.59 ns 113.89 ns 114.19 ns]
voxtree_set_single_voxel/32/batch
                        time:   [2.1568 µs 2.1902 µs 2.2273 µs]
voxtree_set_single_voxel/64/single
                        time:   [130.89 ns 131.24 ns 131.65 ns]
voxtree_set_single_voxel/64/batch
                        time:   [14.622 µs 15.158 µs 15.771 µs]
```

## Modification Operations

### voxtree_fill_then_set_single_voxel

Fills the entire tree with one value and then sets a single voxel to a different value. Tests the performance impact of modifying a fully populated tree.

```rust
// Single operation example
tree.fill(&mut interner, value);
tree.set(&mut interner, IVec3::new(0, 0, 0), different_value);

// Batch operation example
let mut batch = tree.create_batch();
batch.fill(&mut interner, value);
batch.set(&mut interner, IVec3::new(0, 0, 0), different_value);
tree.apply_batch(&mut interner, &batch);
```

```accesslog
voxtree_fill_then_set_single_voxel/8/single
                        time:   [97.504 ns 97.710 ns 97.921 ns]
voxtree_fill_then_set_single_voxel/8/batch
                        time:   [268.79 ns 270.11 ns 271.55 ns]
voxtree_fill_then_set_single_voxel/16/single
                        time:   [124.90 ns 125.20 ns 125.52 ns]
voxtree_fill_then_set_single_voxel/16/batch
                        time:   [526.97 ns 532.89 ns 538.78 ns]
voxtree_fill_then_set_single_voxel/32/single
                        time:   [149.76 ns 150.12 ns 150.49 ns]
voxtree_fill_then_set_single_voxel/32/batch
                        time:   [2.1832 µs 2.2058 µs 2.2290 µs]
voxtree_fill_then_set_single_voxel/64/single
                        time:   [179.06 ns 179.40 ns 179.77 ns]
voxtree_fill_then_set_single_voxel/64/batch
                        time:   [15.268 µs 15.830 µs 16.449 µs]
```

### voxtree_set_uniform

Sets all voxels in the octree to the same value. This represents a worst-case scenario for single operations as every voxel must be individually modified.

```rust
// Single operations
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            tree.set(
                &mut interner,
                IVec3::new(x, y, z),
                value
            );
        }
    }
}

// Batch operations
let mut batch = tree.create_batch();
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            batch.set(&mut interner, IVec3::new(x, y, z), value);
        }
    }
}
tree.apply_batch(&mut interner, &batch);
```

```accesslog
voxtree_set_uniform/8/single
                        time:   [45.188 µs 45.303 µs 45.435 µs]
voxtree_set_uniform/8/batch
                        time:   [520.37 ns 521.65 ns 522.93 ns]
voxtree_set_uniform/16/single
                        time:   [499.48 µs 501.05 µs 502.91 µs]
voxtree_set_uniform/16/batch
                        time:   [2.9751 µs 2.9809 µs 2.9869 µs]
voxtree_set_uniform/32/single
                        time:   [5.1634 ms 5.1711 ms 5.1798 ms]
voxtree_set_uniform/32/batch
                        time:   [23.059 µs 23.110 µs 23.164 µs]
voxtree_set_uniform/64/single
                        time:   [48.052 ms 48.137 ms 48.235 ms]
voxtree_set_uniform/64/batch
                        time:   [189.87 µs 191.93 µs 194.29 µs]
```

### voxtree_set_uniform_half

Similar to uniform fill but only fills half of the octree (along the Y-axis). Tests partial volume modification.

```rust
// Example code
let half_size = size / 2;
for y in 0..half_size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            tree.set(&mut interner, IVec3::new(x, y, z), value);
        }
    }
}
```

```accesslog
voxtree_set_uniform_half/8/single
                        time:   [22.526 µs 22.594 µs 22.663 µs]
voxtree_set_uniform_half/8/batch
                        time:   [397.83 ns 400.15 ns 402.62 ns]
voxtree_set_uniform_half/16/single
                        time:   [250.61 µs 251.19 µs 251.83 µs]
voxtree_set_uniform_half/16/batch
                        time:   [1.9322 µs 1.9543 µs 1.9791 µs]
voxtree_set_uniform_half/32/single
                        time:   [2.6686 ms 2.6728 ms 2.6775 ms]
voxtree_set_uniform_half/32/batch
                        time:   [13.469 µs 13.547 µs 13.622 µs]
voxtree_set_uniform_half/64/single
                        time:   [25.000 ms 25.078 ms 25.156 ms]
voxtree_set_uniform_half/64/batch
                        time:   [114.45 µs 116.11 µs 117.79 µs]
```

### voxtree_set_sum

Sets each voxel's value to the sum of its coordinates (x+y+z). This creates high entropy data with minimal node sharing.

```rust
// Single operations
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            tree.set(
                &mut interner,
                IVec3::new(x, y, z),
                (x + y + z + v) % i32::MAX
            );
        }
    }
}
```

```accesslog
voxtree_set_sum/8/single time:   [54.843 µs 55.270 µs 55.782 µs]
voxtree_set_sum/8/batch  time:   [3.3396 µs 3.3511 µs 3.3636 µs]
voxtree_set_sum/16/single
                        time:   [594.97 µs 595.90 µs 597.02 µs]
voxtree_set_sum/16/batch time:   [24.301 µs 24.350 µs 24.405 µs]
voxtree_set_sum/32/single
                        time:   [5.9124 ms 5.9210 ms 5.9306 ms]
voxtree_set_sum/32/batch time:   [193.91 µs 194.18 µs 194.46 µs]
voxtree_set_sum/64/single
                        time:   [52.772 ms 52.862 ms 52.963 ms]
voxtree_set_sum/64/batch time:   [1.5296 ms 1.5363 ms 1.5443 ms]
```

### voxtree_set_checkerboard

Creates a 3D checkerboard pattern by setting every other voxel (based on coordinate parity).

```rust
// Example code
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            if (x + y + z) % 2 == 0 {
                tree.set(&mut interner, IVec3::new(x, y, z), value);
            }
        }
    }
}
```

```accesslog
voxtree_set_checkerboard/8/single
                        time:   [23.138 µs 23.187 µs 23.240 µs]
voxtree_set_checkerboard/8/batch
                        time:   [2.0223 µs 2.0274 µs 2.0328 µs]
voxtree_set_checkerboard/16/single
                        time:   [245.74 µs 246.27 µs 246.87 µs]
voxtree_set_checkerboard/16/batch
                        time:   [14.060 µs 14.095 µs 14.129 µs]
voxtree_set_checkerboard/32/single
                        time:   [2.5199 ms 2.5244 ms 2.5294 ms]
voxtree_set_checkerboard/32/batch
                        time:   [115.67 µs 116.68 µs 117.92 µs]
voxtree_set_checkerboard/64/single
                        time:   [24.209 ms 24.263 ms 24.321 ms]
voxtree_set_checkerboard/64/batch
                        time:   [933.14 µs 937.43 µs 941.88 µs]
```

### voxtree_set_sparse_fill

Sets values at regular intervals (every 4 voxels), creating a sparse tree. Tests performance with highly sparse modifications.

```rust
// Example code
for y in (0..size as i32).step_by(4) {
    for z in (0..size as i32).step_by(4) {
        for x in (0..size as i32).step_by(4) {
            tree.set(&mut interner, IVec3::new(x, y, z), value);
        }
    }
}
```

```accesslog
voxtree_set_sparse_fill/8/single
                        time:   [600.98 ns 602.26 ns 603.65 ns]
voxtree_set_sparse_fill/8/batch
                        time:   [454.79 ns 456.16 ns 457.77 ns]
voxtree_set_sparse_fill/16/single
                        time:   [6.4702 µs 6.4828 µs 6.4971 µs]
voxtree_set_sparse_fill/16/batch
                        time:   [2.3202 µs 2.3368 µs 2.3581 µs]
voxtree_set_sparse_fill/32/single
                        time:   [65.867 µs 65.988 µs 66.112 µs]
voxtree_set_sparse_fill/32/batch
                        time:   [16.853 µs 16.889 µs 16.930 µs]
voxtree_set_sparse_fill/64/single
                        time:   [649.08 µs 651.14 µs 653.58 µs]
voxtree_set_sparse_fill/64/batch
                        time:   [147.27 µs 148.79 µs 150.42 µs]
```

### voxtree_set_gradient_fill

Creates a gradient along the X-axis by setting all voxels in each X-slice to the same value.

```rust
// Example code
for x in 0..size as i32 {
    let value = (x + v) % 256;
    for y in 0..size as i32 {
        for z in 0..size as i32 {
            tree.set(&mut interner, IVec3::new(x, y, z), value);
        }
    }
}
```

```accesslog
voxtree_set_gradient_fill/8/single
                        time:   [48.094 µs 48.194 µs 48.315 µs]
voxtree_set_gradient_fill/8/batch
                        time:   [3.3382 µs 3.3582 µs 3.3835 µs]
voxtree_set_gradient_fill/16/single
                        time:   [539.41 µs 544.38 µs 551.02 µs]
voxtree_set_gradient_fill/16/batch
                        time:   [25.662 µs 25.736 µs 25.814 µs]
voxtree_set_gradient_fill/32/single
                        time:   [5.4438 ms 5.4522 ms 5.4620 ms]
voxtree_set_gradient_fill/32/batch
                        time:   [208.60 µs 208.93 µs 209.27 µs]
voxtree_set_gradient_fill/64/single
                        time:   [51.145 ms 51.222 ms 51.317 ms]
voxtree_set_gradient_fill/64/batch
                        time:   [1.6997 ms 1.7050 ms 1.7105 ms]
```

### voxtree_set_hollow_cube

Sets only the outer shell/surface of the octree, leaving the interior empty. This simulates creating a hollow 3D structure.

```rust
// Example code
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            let is_edge = x == 0 || x == (size as i32) - 1 ||
                          y == 0 || y == (size as i32) - 1 ||
                          z == 0 || z == (size as i32) - 1;
            if is_edge {
                tree.set(&mut interner, IVec3::new(x, y, z), value);
            }
        }
    }
}
```

```accesslog
voxtree_set_hollow_cube/8/single
                        time:   [27.484 µs 27.540 µs 27.601 µs]
voxtree_set_hollow_cube/8/batch
                        time:   [2.5460 µs 2.5514 µs 2.5572 µs]
voxtree_set_hollow_cube/16/single
                        time:   [166.21 µs 166.74 µs 167.40 µs]
voxtree_set_hollow_cube/16/batch
                        time:   [11.249 µs 11.272 µs 11.298 µs]
voxtree_set_hollow_cube/32/single
                        time:   [973.29 µs 979.59 µs 987.18 µs]
voxtree_set_hollow_cube/32/batch
                        time:   [49.146 µs 49.305 µs 49.483 µs]
voxtree_set_hollow_cube/64/single
                        time:   [4.9734 ms 4.9827 ms 4.9933 ms]
voxtree_set_hollow_cube/64/batch
                        time:   [220.71 µs 221.58 µs 222.55 µs]
```

### voxtree_set_diagonal

Sets only voxels along the main diagonal (where x=y=z). Tests performance with minimal, structured modifications.

```rust
// Example code
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            if x == y && x == z {
                tree.set(&mut interner, IVec3::new(x, y, z), value);
            }
        }
    }
}
```

```accesslog
voxtree_set_diagonal/8/single
                        time:   [858.90 ns 862.10 ns 865.30 ns]
voxtree_set_diagonal/8/batch
                        time:   [341.12 ns 342.05 ns 343.02 ns]
voxtree_set_diagonal/16/single
                        time:   [3.3219 µs 3.3581 µs 3.4008 µs]
voxtree_set_diagonal/16/batch
                        time:   [704.96 ns 710.51 ns 716.53 ns]
voxtree_set_diagonal/32/single
                        time:   [17.544 µs 17.597 µs 17.658 µs]
voxtree_set_diagonal/32/batch
                        time:   [2.6669 µs 2.6788 µs 2.6930 µs]
voxtree_set_diagonal/64/single
                        time:   [96.842 µs 97.125 µs 97.422 µs]
voxtree_set_diagonal/64/batch
                        time:   [18.566 µs 19.850 µs 21.187 µs]
```

### voxtree_set_sphere

Creates a sphere by setting all voxels within a specified radius from the center. Tests performance with a common 3D shape pattern.

```rust
let radius = (size / 2) as i32;
let r1 = radius - 1;
let (cx, cy, cz) = (r1, r1, r1);
let radius_squared = radius * radius;

for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            let dx = (x - cx).abs();
            let dy = (y - cy).abs();
            let dz = (z - cz).abs();
            let distance_squared = dx * dx + dy * dy + dz * dz;

            if distance_squared <= radius_squared {
                tree.set(&mut interner, IVec3::new(x, y, z), value);
            }
        }
    }
}
```

```accesslog
voxtree_set_sphere/8/single
                        time:   [24.023 µs 24.082 µs 24.157 µs]
voxtree_set_sphere/8/batch
                        time:   [1.6889 µs 1.6933 µs 1.6978 µs]
voxtree_set_sphere/16/single
                        time:   [266.11 µs 267.03 µs 268.02 µs]
voxtree_set_sphere/16/batch
                        time:   [8.0822 µs 8.0969 µs 8.1135 µs]
voxtree_set_sphere/32/single
                        time:   [2.9858 ms 2.9945 ms 3.0045 ms]
voxtree_set_sphere/32/batch
                        time:   [42.794 µs 42.877 µs 42.975 µs]
voxtree_set_sphere/64/single
                        time:   [27.390 ms 27.448 ms 27.510 ms]
voxtree_set_sphere/64/batch
                        time:   [249.20 µs 252.59 µs 256.67 µs]
```

### voxtree_set_terrain_surface_only

Creates a terrain-like surface using noise functions, setting only the surface voxels. Simulates terrain generation common in voxel engines.

```rust
// Example code
let mut noise = fastnoise_lite::FastNoiseLite::new();
noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

for z in 0..size as i32 {
    for x in 0..size as i32 {
        let y = ((noise.get_noise_2d(
            x as f32 / size as f32,
            z as f32 / size as f32,
        ) + 1.0) / 2.0) * size as f32;
        let y = y as i32;
        tree.set(&mut interner, IVec3::new(x, y, z), value);
    }
}
```

```accesslog
voxtree_set_terrain_surface_only/8/single
                        time:   [7.3715 µs 7.3887 µs 7.4043 µs]
voxtree_set_terrain_surface_only/8/batch
                        time:   [701.19 ns 706.21 ns 712.36 ns]
voxtree_set_terrain_surface_only/16/single
                        time:   [36.946 µs 37.146 µs 37.371 µs]
voxtree_set_terrain_surface_only/16/batch
                        time:   [2.4064 µs 2.4150 µs 2.4251 µs]
voxtree_set_terrain_surface_only/32/single
                        time:   [180.59 µs 181.04 µs 181.56 µs]
voxtree_set_terrain_surface_only/32/batch
                        time:   [10.851 µs 10.888 µs 10.931 µs]
voxtree_set_terrain_surface_only/64/single
                        time:   [865.99 µs 867.21 µs 868.55 µs]
voxtree_set_terrain_surface_only/64/batch
                        time:   [60.955 µs 61.487 µs 62.042 µs]
```

### voxtree_set_terrain_surface_and_below

Similar to terrain surface, but fills all voxels from the surface down to the bottom. Simulates solid terrain with a varied surface.

```rust
// Example code
let mut noise = fastnoise_lite::FastNoiseLite::new();
noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

for z in 0..size as i32 {
    for x in 0..size as i32 {
        let height = ((noise.get_noise_2d(
            x as f32 / size as f32,
            z as f32 / size as f32,
        ) + 1.0) / 2.0) * size as f32;
        let height = height as i32;

        for y in 0..=height {
            tree.set(&mut interner, IVec3::new(x, y, z), value);
        }
    }
}
```

```accesslog
voxtree_set_terrain_surface_and_below/8/single
                        time:   [30.142 µs 30.227 µs 30.315 µs]
voxtree_set_terrain_surface_and_below/8/batch
                        time:   [916.33 ns 918.53 ns 921.36 ns]
voxtree_set_terrain_surface_and_below/16/single
                        time:   [289.45 µs 290.61 µs 291.76 µs]
voxtree_set_terrain_surface_and_below/16/batch
                        time:   [3.8895 µs 3.9156 µs 3.9500 µs]
voxtree_set_terrain_surface_and_below/32/single
                        time:   [2.8946 ms 2.9005 ms 2.9070 ms]
voxtree_set_terrain_surface_and_below/32/batch
                        time:   [21.865 µs 21.918 µs 21.975 µs]
voxtree_set_terrain_surface_and_below/64/single
                        time:   [26.505 ms 26.574 ms 26.651 ms]
voxtree_set_terrain_surface_and_below/64/batch
                        time:   [140.46 µs 141.87 µs 143.43 µs]
```

### voxtree_set_random_position_same_value

Sets a single voxel at a random position to a fixed value. Tests performance of random access patterns.

```rust
// Example code
let mut rng = rand::rng();
let x = rng.random_range(0..size as i32);
let y = rng.random_range(0..size as i32);
let z = rng.random_range(0..size as i32);

tree.set(&mut interner, IVec3::new(x, y, z), value);
```

```accesslog
voxtree_set_random_position_same_value/8/single
                        time:   [16.119 ns 16.172 ns 16.222 ns]
voxtree_set_random_position_same_value/8/batch
                        time:   [236.93 ns 238.42 ns 240.21 ns]
voxtree_set_random_position_same_value/16/single
                        time:   [16.452 ns 16.496 ns 16.543 ns]
voxtree_set_random_position_same_value/16/batch
                        time:   [638.38 ns 642.81 ns 647.55 ns]
voxtree_set_random_position_same_value/32/single
                        time:   [18.380 ns 18.528 ns 18.710 ns]
voxtree_set_random_position_same_value/32/batch
                        time:   [3.7039 µs 3.7423 µs 3.7812 µs]
voxtree_set_random_position_same_value/64/single
                        time:   [47.907 ns 49.631 ns 51.846 ns]
voxtree_set_random_position_same_value/64/batch
                        time:   [29.488 µs 30.941 µs 32.666 µs]
```

### voxtree_set_random_position_and_value

Sets a single voxel at a random position to a random value. Tests combined random access and value patterns.

```rust
// Example code
let mut rng = rand::rng();
let x = rng.random_range(0..size as i32);
let y = rng.random_range(0..size as i32);
let z = rng.random_range(0..size as i32);
let value = rng.random_range(1..i32::MAX);

tree.set(&mut interner, IVec3::new(x, y, z), value);
```

```accesslog
voxtree_set_random_position_and_value/8/single
                        time:   [160.17 ns 160.69 ns 161.28 ns]
voxtree_set_random_position_and_value/8/batch
                        time:   [350.71 ns 351.91 ns 353.27 ns]
voxtree_set_random_position_and_value/16/single
                        time:   [211.11 ns 212.19 ns 213.29 ns]
voxtree_set_random_position_and_value/16/batch
                        time:   [645.62 ns 647.84 ns 650.53 ns]
voxtree_set_random_position_and_value/32/single
                        time:   [321.36 ns 329.21 ns 337.56 ns]
voxtree_set_random_position_and_value/32/batch
                        time:   [2.3901 µs 2.4064 µs 2.4263 µs]
voxtree_set_random_position_and_value/64/single
                        time:   [400.87 ns 407.88 ns 415.70 ns]
voxtree_set_random_position_and_value/64/batch
                        time:   [26.364 µs 27.508 µs 28.700 µs]
```

## Read Operations

### voxtree_get_empty

Retrieves values from every position in an empty tree. Tests baseline traversal performance.

```rust
// Example code
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            let _ = tree.get(&interner, IVec3::new(x, y, z));
        }
    }
}
```

```accesslog
voxtree_get_empty/8      time:   [673.56 ns 676.16 ns 679.09 ns]
voxtree_get_empty/16     time:   [5.3680 µs 5.3826 µs 5.3963 µs]
voxtree_get_empty/32     time:   [42.193 µs 42.426 µs 42.737 µs]
voxtree_get_empty/64     time:   [346.43 µs 349.15 µs 352.25 µs]
```

### voxtree_get_sphere_uniform

Retrieves values from every position in a octree containing a uniform-valued sphere. Tests read performance with a common pattern.

```rust
// Example code
// First generate a sphere with uniform values
generate_test_sphere(&mut octree, &mut interner, size, 1);

// Then benchmark reading all values
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            let _ = tree.get(&interner, IVec3::new(x, y, z));
        }
    }
}
```

```accesslog
voxtree_get_sphere_uniform/8
                        time:   [1.8093 µs 1.8196 µs 1.8290 µs]
voxtree_get_sphere_uniform/16
                        time:   [16.355 µs 16.441 µs 16.543 µs]
voxtree_get_sphere_uniform/32
                        time:   [141.41 µs 141.74 µs 142.09 µs]
voxtree_get_sphere_uniform/64
                        time:   [1.2057 ms 1.2089 ms 1.2124 ms]
```

### voxtree_get_sphere_sum

Retrieves values from every position in a octree containing a sphere with sum-based values. Tests read performance with high-entropy data.

```rust
// Example code
// First generate a sphere with sum-based values
generate_test_sphere_sum(&mut octree, &mut interner, size);

// Then benchmark reading all values
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            let _ = tree.get(&interner, IVec3::new(x, y, z));
        }
    }
}
```

```accesslog
voxtree_get_sphere_sum/8 time:   [2.0344 µs 2.0475 µs 2.0595 µs]
voxtree_get_sphere_sum/16
                        time:   [17.460 µs 17.498 µs 17.539 µs]
voxtree_get_sphere_sum/32
                        time:   [168.63 µs 169.48 µs 170.38 µs]
voxtree_get_sphere_sum/64
                        time:   [1.3968 ms 1.4016 ms 1.4067 ms]
```

### voxtree_get_full_uniform

Retrieves values from every position in a octree filled with a uniform value. Tests read performance with maximally simple data.

```rust
// Example code
// First fill the octree with a uniform value
tree.fill(&mut interner, 1);

// Then benchmark reading all values
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            let _ = tree.get(&interner, IVec3::new(x, y, z));
        }
    }
}
```

```accesslog
voxtree_get_full_uniform/8
                        time:   [746.22 ns 747.82 ns 749.65 ns]
voxtree_get_full_uniform/16
                        time:   [6.0407 µs 6.0876 µs 6.1461 µs]
voxtree_get_full_uniform/32
                        time:   [47.054 µs 47.381 µs 47.694 µs]
voxtree_get_full_uniform/64
                        time:   [392.92 µs 394.57 µs 396.33 µs]
```

### voxtree_get_full_sum

Retrieves values from every position in a octree filled with sum-based values. Tests read performance with high-entropy data.

```rust
// Example code using the fill_sum! macro
fill_sum!(size, octree, interner);

// Then benchmark reading all values
for y in 0..size as i32 {
    for z in 0..size as i32 {
        for x in 0..size as i32 {
            let _ = tree.get(&interner, IVec3::new(x, y, z));
        }
    }
}
```

```accesslog
voxtree_get_full_sum/8   time:   [2.3402 µs 2.3486 µs 2.3576 µs]
voxtree_get_full_sum/16  time:   [20.581 µs 20.634 µs 20.688 µs]
voxtree_get_full_sum/32  time:   [182.84 µs 183.60 µs 184.53 µs]
voxtree_get_full_sum/64  time:   [1.6238 ms 1.6531 ms 1.6966 ms]
```

## State Operations

### voxtree_is_empty_empty

Checks if an empty octree is empty. Tests baseline state query performance.

```rust
// Example code
let tree = VoxTree::new(depth);
let _ = tree.is_empty();
```

```accesslog
voxtree_is_empty_empty   time:   [271.79 ps 272.60 ps 273.55 ps]
```

### voxtree_is_empty_not_empty

Checks if a non-empty octree is empty. Tests state query with populated data.

```rust
// Example code
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024);
tree.fill(&mut interner, 1);
let _ = tree.is_empty();
```

```accesslog
voxtree_is_empty_not_empty
                        time:   [278.57 ps 279.91 ps 281.29 ps]
```

### voxtree_clear_empty

Clears an already empty tree. Tests baseline clear performance.

```rust
// Example code
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024);
tree.clear(&mut interner);
```

```accesslog
voxtree_clear_empty/8    time:   [272.80 ps 274.14 ps 275.61 ps]
voxtree_clear_empty/16   time:   [272.67 ps 273.25 ps 273.79 ps]
voxtree_clear_empty/32   time:   [272.73 ps 273.31 ps 273.89 ps]
voxtree_clear_empty/64   time:   [274.86 ps 276.43 ps 278.37 ps]
```

### voxtree_clear_sphere

Clears a octree containing a sphere. Tests clear performance with typical data.

```rust
// Example code
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 22);

// First create and apply a batch containing a sphere
let mut batch = tree.create_batch();
generate_test_sphere_for_batch(&mut batch, &mut interner, size, 1);
tree.apply_batch(&mut interner, &batch);

// Then benchmark clearing
tree.clear(&mut interner);
```

```accesslog
voxtree_clear_sphere/8   time:   [1.6452 µs 1.6512 µs 1.6579 µs]
voxtree_clear_sphere/16  time:   [7.5911 µs 7.6197 µs 7.6475 µs]
voxtree_clear_sphere/32  time:   [39.158 µs 39.299 µs 39.443 µs]
voxtree_clear_sphere/64  time:   [227.02 µs 229.80 µs 232.64 µs]
```

### voxtree_clear_filled

Clears a completely filled tree. Tests clear performance with maximally populated data.

```rust
// Example code
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

// Fill the octree first
tree.fill(&mut interner, 1);

// Then benchmark clearing
tree.clear(&mut interner);
```

```accesslog
voxtree_clear_filled/8   time:   [13.595 ns 13.700 ns 13.797 ns]
voxtree_clear_filled/16  time:   [13.671 ns 13.831 ns 14.020 ns]
voxtree_clear_filled/32  time:   [13.463 ns 13.536 ns 13.609 ns]
voxtree_clear_filled/64  time:   [13.450 ns 13.523 ns 13.590 ns]
```

## LOD Operations

### voxtree_to_vec_empty

Converts an empty octree to a vector representation at different LOD levels. Tests baseline LOD extraction performance.

```rust
// Example code
let tree = VoxTree::new(depth);
let interner = VoxInterner::<i32>::with_memory_budget(1024);
let lod = Lod::new(lod_level);
let _ = tree.to_vec(&interner, lod);
```

```accesslog
voxtree_to_vec_empty/8/LOD_0
                        time:   [31.703 ns 31.804 ns 31.917 ns]
voxtree_to_vec_empty/8/LOD_1
                        time:   [13.829 ns 13.880 ns 13.937 ns]
voxtree_to_vec_empty/8/LOD_2
                        time:   [13.755 ns 13.798 ns 13.842 ns]
voxtree_to_vec_empty/16/LOD_0
                        time:   [231.85 ns 236.58 ns 241.46 ns]
voxtree_to_vec_empty/16/LOD_1
                        time:   [31.786 ns 32.107 ns 32.489 ns]
voxtree_to_vec_empty/16/LOD_2
                        time:   [14.156 ns 14.242 ns 14.320 ns]
voxtree_to_vec_empty/16/LOD_3
                        time:   [13.873 ns 13.916 ns 13.957 ns]
voxtree_to_vec_empty/32/LOD_0
                        time:   [870.77 ns 887.50 ns 903.88 ns]
voxtree_to_vec_empty/32/LOD_1
                        time:   [226.36 ns 231.34 ns 236.61 ns]
voxtree_to_vec_empty/32/LOD_2
                        time:   [31.166 ns 31.227 ns 31.296 ns]
voxtree_to_vec_empty/32/LOD_3
                        time:   [13.756 ns 13.784 ns 13.818 ns]
voxtree_to_vec_empty/32/LOD_4
                        time:   [13.896 ns 13.965 ns 14.048 ns]
voxtree_to_vec_empty/64/LOD_0
                        time:   [4.6677 µs 4.7132 µs 4.7657 µs]
voxtree_to_vec_empty/64/LOD_1
                        time:   [828.96 ns 841.57 ns 856.63 ns]
voxtree_to_vec_empty/64/LOD_2
                        time:   [222.81 ns 226.17 ns 229.50 ns]
voxtree_to_vec_empty/64/LOD_3
                        time:   [31.279 ns 31.354 ns 31.435 ns]
voxtree_to_vec_empty/64/LOD_4
                        time:   [13.783 ns 13.837 ns 13.895 ns]
voxtree_to_vec_empty/64/LOD_5
                        time:   [13.659 ns 13.701 ns 13.744 ns]
```

### voxtree_to_vec_sphere

Converts a octree containing a sphere to a vector representation at different LOD levels. Tests LOD with a common shape.

```rust
// Example code
// First create a sphere
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 14);
let mut batch = tree.create_batch();
generate_test_sphere_for_batch(&mut batch, &mut interner, size, 1);
tree.apply_batch(&mut interner, &batch);

// Then benchmark LOD extraction at different levels
let lod = Lod::new(lod_level);
let _ = tree.to_vec(&interner, lod);
```

```accesslog
voxtree_to_vec_sphere/8/LOD_0
                        time:   [1.5369 µs 1.5449 µs 1.5532 µs]
voxtree_to_vec_sphere/8/LOD_1
                        time:   [201.81 ns 205.17 ns 209.72 ns]
voxtree_to_vec_sphere/8/LOD_2
                        time:   [31.124 ns 31.224 ns 31.343 ns]
voxtree_to_vec_sphere/16/LOD_0
                        time:   [14.330 µs 14.385 µs 14.444 µs]
voxtree_to_vec_sphere/16/LOD_1
                        time:   [1.8859 µs 1.8906 µs 1.8961 µs]
voxtree_to_vec_sphere/16/LOD_2
                        time:   [199.51 ns 200.52 ns 201.58 ns]
voxtree_to_vec_sphere/16/LOD_3
                        time:   [30.921 ns 31.240 ns 31.599 ns]
voxtree_to_vec_sphere/32/LOD_0
                        time:   [125.31 µs 125.59 µs 125.90 µs]
voxtree_to_vec_sphere/32/LOD_1
                        time:   [15.698 µs 15.736 µs 15.776 µs]
voxtree_to_vec_sphere/32/LOD_2
                        time:   [2.0035 µs 2.0079 µs 2.0131 µs]
voxtree_to_vec_sphere/32/LOD_3
                        time:   [198.58 ns 199.13 ns 199.74 ns]
voxtree_to_vec_sphere/32/LOD_4
                        time:   [30.332 ns 30.408 ns 30.499 ns]
voxtree_to_vec_sphere/64/LOD_0
                        time:   [1.0756 ms 1.0788 ms 1.0827 ms]
voxtree_to_vec_sphere/64/LOD_1
                        time:   [132.52 µs 133.29 µs 134.28 µs]
voxtree_to_vec_sphere/64/LOD_2
                        time:   [15.708 µs 15.761 µs 15.821 µs]
voxtree_to_vec_sphere/64/LOD_3
                        time:   [2.0202 µs 2.0253 µs 2.0310 µs]
voxtree_to_vec_sphere/64/LOD_4
                        time:   [201.95 ns 202.90 ns 203.77 ns]
voxtree_to_vec_sphere/64/LOD_5
                        time:   [30.716 ns 30.814 ns 30.911 ns]
```

### voxtree_to_vec_uniform

Converts a uniformly filled octree to a vector representation at different LOD levels. Tests LOD with simple data.

```rust
// Example code
// First fill the octree with a uniform value
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);
tree.fill(&mut interner, 1);

// Then benchmark LOD extraction at different levels
let lod = Lod::new(lod_level);
let _ = tree.to_vec(&interner, lod);
```

```accesslog
voxtree_to_vec_uniform/8/LOD_0
                        time:   [31.689 ns 31.765 ns 31.850 ns]
voxtree_to_vec_uniform/8/LOD_1
                        time:   [19.165 ns 19.363 ns 19.584 ns]
voxtree_to_vec_uniform/8/LOD_2
                        time:   [14.951 ns 14.999 ns 15.055 ns]
voxtree_to_vec_uniform/16/LOD_0
                        time:   [229.08 ns 234.31 ns 239.44 ns]
voxtree_to_vec_uniform/16/LOD_1
                        time:   [31.763 ns 31.832 ns 31.913 ns]
voxtree_to_vec_uniform/16/LOD_2
                        time:   [19.080 ns 19.169 ns 19.271 ns]
voxtree_to_vec_uniform/16/LOD_3
                        time:   [15.078 ns 15.134 ns 15.193 ns]
voxtree_to_vec_uniform/32/LOD_0
                        time:   [1.9131 µs 2.0777 µs 2.2424 µs]
voxtree_to_vec_uniform/32/LOD_1
                        time:   [242.04 ns 246.77 ns 251.42 ns]
voxtree_to_vec_uniform/32/LOD_2
                        time:   [32.013 ns 32.112 ns 32.220 ns]
voxtree_to_vec_uniform/32/LOD_3
                        time:   [19.271 ns 19.350 ns 19.431 ns]
voxtree_to_vec_uniform/32/LOD_4
                        time:   [15.153 ns 15.205 ns 15.256 ns]
voxtree_to_vec_uniform/64/LOD_0
                        time:   [11.841 µs 12.492 µs 13.175 µs]
voxtree_to_vec_uniform/64/LOD_1
                        time:   [1.9686 µs 2.0717 µs 2.1835 µs]
voxtree_to_vec_uniform/64/LOD_2
                        time:   [222.33 ns 226.32 ns 230.05 ns]
voxtree_to_vec_uniform/64/LOD_3
                        time:   [31.683 ns 31.756 ns 31.833 ns]
voxtree_to_vec_uniform/64/LOD_4
                        time:   [19.082 ns 19.146 ns 19.211 ns]
voxtree_to_vec_uniform/64/LOD_5
                        time:   [14.964 ns 15.000 ns 15.041 ns]
```

### voxtree_to_vec_sum

Converts a octree with sum-based values to a vector representation at different LOD levels. Tests LOD with high-entropy data.

```rust
// Example code
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 24);

// Fill with sum-based values
fill_sum!(size, octree, interner);

// Then benchmark LOD extraction at different levels
let lod = Lod::new(lod_level);
let _ = tree.to_vec(&interner, lod);
```

```accesslog
voxtree_to_vec_sum/8/LOD_0
                        time:   [2.1706 µs 2.1787 µs 2.1868 µs]
voxtree_to_vec_sum/8/LOD_1
                        time:   [199.01 ns 199.62 ns 200.28 ns]
voxtree_to_vec_sum/8/LOD_2
                        time:   [30.865 ns 30.982 ns 31.110 ns]
voxtree_to_vec_sum/16/LOD_0
                        time:   [18.770 µs 18.824 µs 18.890 µs]
voxtree_to_vec_sum/16/LOD_1
                        time:   [2.1807 µs 2.1869 µs 2.1940 µs]
voxtree_to_vec_sum/16/LOD_2
                        time:   [199.22 ns 199.85 ns 200.43 ns]
voxtree_to_vec_sum/16/LOD_3
                        time:   [30.984 ns 31.123 ns 31.262 ns]
voxtree_to_vec_sum/32/LOD_0
                        time:   [167.17 µs 167.65 µs 168.24 µs]
voxtree_to_vec_sum/32/LOD_1
                        time:   [19.309 µs 19.466 µs 19.647 µs]
voxtree_to_vec_sum/32/LOD_2
                        time:   [2.2240 µs 2.2516 µs 2.2911 µs]
voxtree_to_vec_sum/32/LOD_3
                        time:   [206.70 ns 207.45 ns 208.20 ns]
voxtree_to_vec_sum/32/LOD_4
                        time:   [30.827 ns 31.063 ns 31.295 ns]
voxtree_to_vec_sum/64/LOD_0
                        time:   [1.4821 ms 1.4870 ms 1.4919 ms]
voxtree_to_vec_sum/64/LOD_1
                        time:   [168.28 µs 168.77 µs 169.23 µs]
voxtree_to_vec_sum/64/LOD_2
                        time:   [19.048 µs 19.116 µs 19.184 µs]
voxtree_to_vec_sum/64/LOD_3
                        time:   [2.2125 µs 2.2283 µs 2.2456 µs]
voxtree_to_vec_sum/64/LOD_4
                        time:   [201.22 ns 202.31 ns 203.34 ns]
voxtree_to_vec_sum/64/LOD_5
                        time:   [31.539 ns 31.806 ns 32.065 ns]
```

### voxtree_to_vec_terrain

Converts a terrain-like octree to a vector representation at different LOD levels. Tests LOD with a typical game world pattern.

```rust
// Example code
let mut tree = VoxTree::new(depth);
let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

// Generate terrain with noise
let mut noise = fastnoise_lite::FastNoiseLite::new();
noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

let mut batch = tree.create_batch();
for x in 0..size as i32 {
    for z in 0..size as i32 {
        let y = ((noise.get_noise_2d(x as f32 / size as f32, z as f32 / size as f32) + 1.0) / 2.0) * size as f32;
        let y = y as i32;
        batch.set(&mut interner, IVec3::new(x, y, z), 1);
    }
}
tree.apply_batch(&mut interner, &batch);

// Then benchmark LOD extraction at different levels
let lod = Lod::new(lod_level);
let _ = tree.to_vec(&interner, lod);
```

```accesslog
voxtree_to_vec_terrain/8/LOD_0
                        time:   [898.56 ns 902.47 ns 906.71 ns]
voxtree_to_vec_terrain/8/LOD_1
                        time:   [146.69 ns 147.23 ns 147.82 ns]
voxtree_to_vec_terrain/8/LOD_2
                        time:   [30.975 ns 31.124 ns 31.271 ns]
voxtree_to_vec_terrain/16/LOD_0
                        time:   [6.6373 µs 6.6584 µs 6.6820 µs]
voxtree_to_vec_terrain/16/LOD_1
                        time:   [907.29 ns 910.50 ns 914.44 ns]
voxtree_to_vec_terrain/16/LOD_2
                        time:   [147.50 ns 148.26 ns 149.12 ns]
voxtree_to_vec_terrain/16/LOD_3
                        time:   [31.110 ns 31.214 ns 31.329 ns]
voxtree_to_vec_terrain/32/LOD_0
                        time:   [51.635 µs 51.928 µs 52.221 µs]
voxtree_to_vec_terrain/32/LOD_1
                        time:   [6.6271 µs 6.6426 µs 6.6604 µs]
voxtree_to_vec_terrain/32/LOD_2
                        time:   [898.81 ns 901.15 ns 903.58 ns]
voxtree_to_vec_terrain/32/LOD_3
                        time:   [148.10 ns 149.05 ns 150.17 ns]
voxtree_to_vec_terrain/32/LOD_4
                        time:   [30.998 ns 31.154 ns 31.313 ns]
voxtree_to_vec_terrain/64/LOD_0
                        time:   [407.46 µs 408.04 µs 408.70 µs]
voxtree_to_vec_terrain/64/LOD_1
                        time:   [50.955 µs 51.036 µs 51.143 µs]
voxtree_to_vec_terrain/64/LOD_2
                        time:   [6.6227 µs 6.6438 µs 6.6705 µs]
voxtree_to_vec_terrain/64/LOD_3
                        time:   [911.40 ns 914.14 ns 916.93 ns]
voxtree_to_vec_terrain/64/LOD_4
                        time:   [147.10 ns 147.65 ns 148.28 ns]
voxtree_to_vec_terrain/64/LOD_5
                        time:   [31.225 ns 31.348 ns 31.487 ns]
```
