# Voxelis Benchmarks - Enhanced Comparison Tables

## Batch vs Single Operations - Key Insights

This table provides a comparison of single vs batch operations across octree sizes, with calculated speedup factors and color-coding to highlight performance characteristics.

### Operations Where Batch is Most Beneficial (32³ octree)

| Operation | Single (µs) | Batch (µs) | Speedup |
|-----------|------------|-----------|---------|
| set_uniform | 5,170.00 | 23.11 | 🟢 **224x** |
| set_uniform_half | 2,670.00 | 13.55 | 🟢 **197x** |
| set_terrain_surface_and_below | 2,900.00 | 21.92 | 🟢 **132x** |
| set_sphere | 2,990.00 | 42.88 | 🟢 **70x** |
| set_sum | 5,920.00 | 194.18 | 🟢 **30x** |
| set_checkerboard | 2,520.00 | 116.68 | 🟢 **22x** |
| set_terrain_surface_only | 181.04 | 10.89 | 🟢 **17x** |
| set_diagonal | 17.60 | 2.68 | 🟢 **6.6x** |
| set_sparse_fill | 65.99 | 16.89 | 🟢 **3.9x** |
| fill | 0.0090 | 0.0106 | 🔴 **0.85x** |
| set_single_voxel | 0.1139 | 2.19 | 🔴 **0.05x** |
| set_random_position_same_value | 0.0185 | 3.74 | 🔴 **0.005x** |

### Fill Operations (All Values in ns)

| Operation | Size | Single | Batch | Speedup |
|-----------|------|--------|-------|---------|
| voxtree_fill | 8³ | 9.21 | 10.46 | 🔴 0.88x |
| voxtree_fill | 16³ | 8.92 | 10.43 | 🔴 0.86x |
| voxtree_fill | 32³ | 8.98 | 10.60 | 🔴 0.85x |
| voxtree_fill | 64³ | 8.93 | 10.31 | 🔴 0.87x |

### Single Voxel & Random Operations (All Values in ns)

| Operation | Size | Single | Batch | Speedup |
|-----------|------|--------|-------|---------|
| set_single_voxel | 8³ | 75.51 | 245.83 | 🔴 0.31x |
| set_single_voxel | 16³ | 92.99 | 478.77 | 🔴 0.19x |
| set_single_voxel | 32³ | 113.89 | 2,190.00 | 🔴 0.05x |
| set_single_voxel | 64³ | 131.24 | 15,160.00 | 🔴 0.01x |
| set_random_position_same_value | 8³ | 16.17 | 238.42 | 🔴 0.07x |
| set_random_position_same_value | 16³ | 16.50 | 642.81 | 🔴 0.03x |
| set_random_position_same_value | 32³ | 18.53 | 3,740.00 | 🔴 0.005x |
| set_random_position_same_value | 64³ | 49.63 | 30,940.00 | 🔴 0.002x |

### Bulk Operations (All Values in µs)

<details>
<summary>Click to expand complete bulk operations table</summary>

| Operation | Size | Single | Batch | Speedup |
|-----------|------|--------|-------|---------|
| set_uniform | 8³ | 45.30 | 0.52 | 🟢 86.8x |
| set_uniform | 16³ | 501.05 | 2.98 | 🟢 168.1x |
| set_uniform | 32³ | 5,170.00 | 23.11 | 🟢 223.7x |
| set_uniform | 64³ | 48,140.00 | 192.93 | 🟢 249.5x |
| set_uniform_half | 8³ | 22.59 | 0.40 | 🟢 56.5x |
| set_uniform_half | 16³ | 251.19 | 1.95 | 🟢 128.8x |
| set_uniform_half | 32³ | 2,670.00 | 13.55 | 🟢 197.0x |
| set_uniform_half | 64³ | 25,080.00 | 116.11 | 🟢 216.0x |
| set_sum | 8³ | 55.27 | 3.35 | 🟢 16.5x |
| set_sum | 16³ | 595.90 | 24.35 | 🟢 24.5x |
| set_sum | 32³ | 5,920.00 | 194.18 | 🟢 30.5x |
| set_sum | 64³ | 52,860.00 | 1,540.00 | 🟢 34.3x |
| set_checkerboard | 8³ | 23.19 | 2.03 | 🟢 11.4x |
| set_checkerboard | 16³ | 246.27 | 14.10 | 🟢 17.5x |
| set_checkerboard | 32³ | 2,520.00 | 116.68 | 🟢 21.6x |
| set_checkerboard | 64³ | 24,260.00 | 937.43 | 🟢 25.9x |
| set_sparse_fill | 8³ | 0.60 | 0.46 | 🟢 1.3x |
| set_sparse_fill | 16³ | 6.48 | 2.34 | 🟢 2.8x |
| set_sparse_fill | 32³ | 65.99 | 16.89 | 🟢 3.9x |
| set_sparse_fill | 64³ | 651.14 | 148.79 | 🟢 4.4x |
| set_gradient_fill | 8³ | 48.19 | 3.36 | 🟢 14.3x |
| set_gradient_fill | 16³ | 544.38 | 25.74 | 🟢 21.1x |
| set_gradient_fill | 32³ | 5,450.00 | 208.93 | 🟢 26.1x |
| set_gradient_fill | 64³ | 51,220.00 | 1,710.00 | 🟢 30.0x |
| set_hollow_cube | 8³ | 27.54 | 2.55 | 🟢 10.8x |
| set_hollow_cube | 16³ | 166.74 | 11.27 | 🟢 14.8x |
| set_hollow_cube | 32³ | 979.59 | 49.31 | 🟢 19.9x |
| set_hollow_cube | 64³ | 4,980.00 | 221.58 | 🟢 22.5x |
| set_diagonal | 8³ | 0.86 | 0.34 | 🟢 2.5x |
| set_diagonal | 16³ | 3.36 | 0.71 | 🟢 4.7x |
| set_diagonal | 32³ | 17.60 | 2.68 | 🟢 6.6x |
| set_diagonal | 64³ | 97.13 | 19.85 | 🟢 4.9x |
| set_sphere | 8³ | 24.08 | 1.69 | 🟢 14.2x |
| set_sphere | 16³ | 267.03 | 8.10 | 🟢 33.0x |
| set_sphere | 32³ | 2,990.00 | 42.88 | 🟢 69.7x |
| set_sphere | 64³ | 27,450.00 | 252.59 | 🟢 108.7x |
| set_terrain_surface_only | 8³ | 7.39 | 0.71 | 🟢 10.5x |
| set_terrain_surface_only | 16³ | 37.15 | 2.42 | 🟢 15.4x |
| set_terrain_surface_only | 32³ | 181.04 | 10.89 | 🟢 16.6x |
| set_terrain_surface_only | 64³ | 867.21 | 61.49 | 🟢 14.1x |
| set_terrain_surface_and_below | 8³ | 30.23 | 0.92 | 🟢 32.9x |
| set_terrain_surface_and_below | 16³ | 290.61 | 3.92 | 🟢 74.1x |
| set_terrain_surface_and_below | 32³ | 2,900.00 | 21.92 | 🟢 132.3x |
| set_terrain_surface_and_below | 64³ | 26,570.00 | 141.87 | 🟢 187.3x |

</details>

## LOD Performance Scaling Analysis

### LOD Performance Scaling for 64³ Sphere

| LOD Level | Time | Speedup vs LOD 0 | Theoretical (8ˣ) |
|-----------|------|------------------|------------------|
| 0 | 1,078.8 µs | 1.0x | 1.0x |
| 1 | 133.29 µs | 🟢 8.1x | 8.0x |
| 2 | 15.76 µs | 🟢 68.5x | 64.0x |
| 3 | 2.03 µs | 🟢 531.4x | 512.0x |
| 4 | 202.90 ns | 🟢 5,316.4x | 4,096.0x |
| 5 | 30.81 ns | 🟢 35,014.9x | 32,768.0x |

### Empty Octree LOD Performance (µs)

| Size | LOD 0 | LOD 1 | LOD 2 | LOD 3 | LOD 4 | LOD 5 |
|------|-------|-------|-------|-------|-------|-------|
| 8³ | 0.032 | 0.014 | 0.014 | - | - | - |
| 16³ | 0.237 | 0.032 | 0.014 | 0.014 | - | - |
| 32³ | 0.888 | 0.231 | 0.031 | 0.014 | 0.014 | - |
| 64³ | 4.713 | 0.842 | 0.226 | 0.031 | 0.014 | 0.014 |

### Ratio to LOD 0 (Empty Octree)

| Size | LOD 0 | LOD 1 | LOD 2 | LOD 3 | LOD 4 | LOD 5 |
|------|-------|-------|-------|-------|-------|-------|
| 8³ | 1.0x | 0.44x | 0.44x | - | - | - |
| 16³ | 1.0x | 0.14x | 0.06x | 0.06x | - | - |
| 32³ | 1.0x | 0.26x | 0.03x | 0.02x | 0.02x | - |
| 64³ | 1.0x | 0.18x | 0.05x | 0.01x | 0.003x | 0.003x |

### Sphere LOD Performance (µs)

| Size | LOD 0 | LOD 1 | LOD 2 | LOD 3 | LOD 4 | LOD 5 |
|------|-------|-------|-------|-------|-------|-------|
| 8³ | 1.545 | 0.205 | 0.031 | - | - | - |
| 16³ | 14.385 | 1.891 | 0.201 | 0.031 | - | - |
| 32³ | 125.590 | 15.736 | 2.008 | 0.199 | 0.030 | - |
| 64³ | 1,078.800 | 133.290 | 15.761 | 2.025 | 0.203 | 0.031 |

### Ratio to LOD 0 (Sphere)

| Size | LOD 0 | LOD 1 | LOD 2 | LOD 3 | LOD 4 | LOD 5 |
|------|-------|-------|-------|-------|-------|-------|
| 8³ | 1.0x | 0.13x | 0.02x | - | - | - |
| 16³ | 1.0x | 0.13x | 0.01x | 0.002x | - | - |
| 32³ | 1.0x | 0.13x | 0.02x | 0.002x | 0.0002x | - |
| 64³ | 1.0x | 0.12x | 0.01x | 0.002x | 0.0002x | 0.00003x |

### Uniform Fill LOD Performance (µs)

| Size | LOD 0 | LOD 1 | LOD 2 | LOD 3 | LOD 4 | LOD 5 |
|------|-------|-------|-------|-------|-------|-------|
| 8³ | 0.032 | 0.019 | 0.015 | - | - | - |
| 16³ | 0.234 | 0.032 | 0.019 | 0.015 | - | - |
| 32³ | 2.078 | 0.247 | 0.032 | 0.019 | 0.015 | - |
| 64³ | 12.492 | 2.072 | 0.226 | 0.032 | 0.019 | 0.015 |

### Sum-based LOD Performance (µs)

| Size | LOD 0 | LOD 1 | LOD 2 | LOD 3 | LOD 4 | LOD 5 |
|------|-------|-------|-------|-------|-------|-------|
| 8³ | 2.179 | 0.200 | 0.031 | - | - | - |
| 16³ | 18.824 | 2.187 | 0.200 | 0.031 | - | - |
| 32³ | 167.650 | 19.466 | 2.252 | 0.207 | 0.031 | - |
| 64³ | 1,487.000 | 168.770 | 19.116 | 2.228 | 0.202 | 0.032 |

## Size Scaling Analysis

This table shows how performance scales as octree size increases (using batch operations):

| Operation | 8³ | 16³ | 32³ | 64³ | Scaling Factor (8³→64³) | Theoretical (Volume) |
|-----------|-------|--------|---------|----------|------------|------------|
| fill | 10.46 ns | 10.43 ns | 10.60 ns | 10.31 ns | 🟢 0.99x | 512x |
| set_uniform | 0.52 µs | 2.98 µs | 23.11 µs | 192.93 µs | 🔴 370x | 512x |
| set_sum | 3.35 µs | 24.35 µs | 194.18 µs | 1.54 ms | 🔴 460x | 512x |
| set_sphere | 1.69 µs | 8.10 µs | 42.88 µs | 252.59 µs | 🔴 149x | 512x |
| set_terrain_surface_only | 0.71 µs | 2.42 µs | 10.89 µs | 61.49 µs | 🔴 87x | 64x (surface) |
| get_sphere_uniform | 1.82 µs | 16.44 µs | 141.74 µs | 1.21 ms | 🔴 665x | 512x |

### Scaling Visual Comparison (64³ vs 8³) for Batch Operations

| Operation | Better than Theoretical | Close to Theoretical | Worse than Theoretical |
|-----------|-------------------------|----------------------|------------------------|
| fill | 🟢 0.99x |  |  |
| set_terrain_surface_only |  | 🟡 87x (vs 64x) |  |
| set_sphere |  |  | 🔴 149x |
| set_uniform |  |  | 🔴 370x |
| set_sum |  |  | 🔴 460x |
| get_sphere_uniform |  |  | 🔴 665x |

## Performance Impacts of Data Patterns

This table compares the performance of different data patterns (at 32³, batch mode):

| Data Pattern | Batch Time (32³) | Relative to Uniform | DAG Efficiency |
|--------------|------------------|---------------------|----------------|
| Diagonal | 2.68 µs | 0.12x | 🟢 High |
| Sparse Fill | 16.89 µs | 0.73x | 🟢 High |
| Terrain Surface | 10.89 µs | 0.47x | 🟢 High |
| Terrain Volume | 21.92 µs | 0.95x | 🟢 Medium |
| Uniform Fill | 23.11 µs | 1.0x | 🟡 Medium |
| Sphere | 42.88 µs | 1.9x | 🟡 Medium |
| Hollow Cube | 49.31 µs | 2.1x | 🟡 Medium |
| Checkerboard | 116.68 µs | 5.0x | 🔴 Low |
| Sum-based | 194.18 µs | 8.4x | 🔴 Low |

## Key Observations and Insights

1. **Single vs Batch Performance**:
   - Batch operations provide massive speedups (up to 250x) for operations that modify many voxels
   - Small, targeted operations (single voxel, random access) are faster with single operations

2. **LOD Scaling**:
   - Each LOD level provides approximately 8x speedup, matching the theoretical octree reduction (8^n)
   - LOD performance closely follows the expected octree reduction curve, confirming the efficiency of the zero-cost LOD implementation

3. **Size Scaling**:
   - The `fill` operation shows near-constant time regardless of octree size - impressive O(1) performance
   - Surface-based operations scale closer to O(n²) than O(n³), showing the VoxTree's efficiency with surface data
   - High-entropy operations (set_sum) scale nearly linearly with volume, approaching O(n³)

4. **Data Pattern Impact**:
   - Patterns with high spatial coherence (diagonal, sparse fill) perform significantly better than high-entropy patterns
   - The VoxTree structure is most efficient with patterns that can leverage structural sharing

5. **Notable Anomalies**:
   - The `get_sphere_uniform` operation scales worse than theoretical O(n³), suggesting room for optimization
   - Surface operations scale slightly worse than the theoretical O(n²), possibly due to traversal overhead
