# Pinned CGAL EPECK Boolean adapter

This benchmark-only adapter consumes two exact triangular OFF files, runs CGAL
6.0.3 `Exact_predicates_exact_constructions_kernel`, and writes one JSON record
per repetition. Coordinate tokens may be integers or exact `numerator/denominator`
rationals; they are parsed through `CGAL::Gmpq`, not binary floating point.

Build:

```sh
cmake -S competitive/cgal-epeck \
  -B target/competitive/cgal-epeck \
  -DCMAKE_BUILD_TYPE=Release
cmake --build target/competitive/cgal-epeck --parallel
```

Run:

```sh
target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  benchmarks/corpus/exact/overlapping-boxes-left.off \
  benchmarks/corpus/exact/overlapping-boxes-right.off \
  all 11 outside
```

Generated competitive fixtures use the Rust exact-OFF exporter so CGAL sees
the same rational value that Hyperreal imports from each binary64 coordinate,
including values whose shortest decimal spelling would not be the identical
rational:

```sh
cargo build --release --example export_cgal_exact_off
target/release/examples/export_cgal_exact_off \
  clipped_voxel_torus_65 /tmp/hypermesh-cgal
target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  /tmp/hypermesh-cgal/clipped_voxel_torus_65-left.off \
  /tmp/hypermesh-cgal/clipped_voxel_torus_65-right.off \
  intersection 21 outside
```

The same exporter accepts `dense_coplanar_boxes_4`,
`dense_coplanar_boxes_16`, and `dense_coplanar_boxes_32`, whose opposite face
diagonals provide the fixed-coordinate-complexity coplanar scaling family.

It also accepts `sparse_multishell_tetrahedra_8`,
`sparse_multishell_tetrahedra_64`, and
`sparse_multishell_tetrahedra_512`. Each input mesh contains many disconnected
closed tetrahedral shells; corresponding shell pairs intersect and distinct
grid cells are exactly disjoint. This is the sparse component-scaling family.

It also accepts `wide_rational_boxes_64`, `wide_rational_boxes_512`, and
`wide_rational_boxes_2048`. Those inputs preserve one fixed topology while
growing the exact similarity numerator and denominator through 65, 513, and
2,049 bits; the exporter retains the exact rational rather than its binary64
approximation of one.

It also accepts `thin_dyadic_boxes_64`, `thin_dyadic_boxes_512`, and
`thin_dyadic_boxes_2048`. They retain one fixed 6,144-triangle topology under
the exact affine map `(x, y, z) -> (x + z, y, z / 2^shift)`, so the last member
has an exact nonzero thin coordinate even though its binary64 approximation is
zero.

The exporter writes reduced `numerator/denominator` tokens from
`Real::exact_rational`; it does not round through a display approximation.

The operation is `union`, `intersection`, `difference`,
`reverse-difference`, `xor`, or `all`. The final argument selects whether the
input copies required by CGAL's mutating API are made `inside` or `outside` the
timed interval. Authoritative reports include both modes and measure peak RSS by
running this executable in a fresh `/usr/bin/time -v` process.

CGAL is a competitive/development dependency only. It is not linked into
Hypermesh and is not a correctness oracle by itself.
