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

The exporter writes reduced `numerator/denominator` tokens from
`Real::exact_rational`; it does not round through a display approximation.

The operation is `union`, `intersection`, `difference`,
`reverse-difference`, `xor`, or `all`. The final argument selects whether the
input copies required by CGAL's mutating API are made `inside` or `outside` the
timed interval. Authoritative reports include both modes and measure peak RSS by
running this executable in a fresh `/usr/bin/time -v` process.

CGAL is a competitive/development dependency only. It is not linked into
Hypermesh and is not a correctness oracle by itself.
