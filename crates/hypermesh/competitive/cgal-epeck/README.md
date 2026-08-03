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

The operation is `union`, `intersection`, `difference`,
`reverse-difference`, `xor`, or `all`. The final argument selects whether the
input copies required by CGAL's mutating API are made `inside` or `outside` the
timed interval. Authoritative reports include both modes and measure peak RSS by
running this executable in a fresh `/usr/bin/time -v` process.

CGAL is a competitive/development dependency only. It is not linked into
Hypermesh and is not a correctness oracle by itself.
