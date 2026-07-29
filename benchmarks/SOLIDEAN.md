# Solidean published Boolean benchmarks

`solidean_comparison` replays the public request files used in Solidean's 2026
Boolean-kernel articles. The fixture loader pins
[`solidean/bench-blog-data`](https://github.com/solidean/bench-blog-data) at
commit `80900cf867fb690e6c7571880857e82f6f45ae34`, verifies the downloaded archive
and request files by SHA-256, and caches them below
`target/benchmark-fixtures/solidean`. Dataset assets remain under their
published CC BY 4.0 terms and are not vendored into csgrs.

The measured boundary follows the
[Open Boolean Benchmark runners](https://github.com/Open-Boolean-Benchmark/boolean-benchmark-runners):
request and OBJ parsing and disk I/O are excluded; conversion from indexed
double-precision triangles, all ordered Boolean operations, and final result
materialization are included. csgrs executes each request serially. The final
area and volume check, or the required empty-result check for the cube grid,
runs after the per-sample timer stops.

## Published reference times

These values are transcribed from the linked Solidean articles. They were
recorded on the publisher's hardware—principally a Ryzen 9 5900X, 64 GiB
DDR4-2666, and Windows 11—and may include each kernel's supported parallelism.
Local csgrs CSV rows therefore provide a reproducible workload comparison, not
a same-machine speed ratio.

### First iterated CSG

Nine differences; canonical area `13.4224`, volume `2.3513`.
[Published article and methodology](https://solidean.com/blog/2026/first-benchmark-results-iterated-csg/).

| Kernel | Published time |
|---|---:|
| Solidean | 27 ms |
| Trueform | 85 ms |
| MeshLib | 109 ms |
| Manifold | 332 ms |
| CGAL corefine | 507 ms |
| Mesh Arrangements | 5.6 s |
| Geogram | 32.4 s |
| CGAL Nef | 477.7 s |

### Terrain carve

The pinned request contains 224 ordered differences; canonical area `6.1648`,
volume `0.5477`.
[Published article and methodology](https://solidean.com/blog/2026/terrain-carve-benchmark/).

| Kernel | Published time |
|---|---:|
| Solidean | 205 ms |
| Manifold | 525 ms |
| CGAL corefine | 11.0 s |
| CGAL Nef | 330 s |

The article reports these four as producing the canonical result.
It describes the workload as 223 subtractions even though its current public
version-1 request contains 224 difference operations (tools `0.obj` through
`223.obj`). The csgrs row follows the request file exactly and carries `_224`
in its case name so this upstream off-by-one discrepancy is visible when
comparing the published time.

### Iterated cube grid

One thousand cube loads, 999 ordered unions, and 1,000 ordered differences;
the final result is empty.
[Published article and per-step correctness discussion](https://solidean.com/blog/2026/iterated-cube-grid-benchmark/).

| Kernel | Published time |
|---|---:|
| Solidean | 980 ms |
| Trueform | 4.1 s |
| Manifold | 5.3 s |
| Carve | 23 s |
| Mesh Arrangements | 62 s |
| Geogram 1.10 | 112 s |
| CGAL Nef | 280 s |
| Blender exact | 294 s |
| Geogram 1.9.8 | about 37 min |

CGAL corefine correctly declines the early non-manifold intermediate produced
by this request and consequently has no completion time.

### Dome carve

One block followed by an ordered chain of capsule-tool differences.
[Published article and scaling discussion](https://solidean.com/blog/2026/iterated-dome-carve-benchmark/).

| Tools | Canonical area | Canonical volume | Solidean | Manifold | CGAL corefine | MeshLib |
|---:|---:|---:|---:|---:|---:|---:|
| 100 | 64106.5450 | 274099.8661 | 73 ms | 1.1 s | 3.3 s | 957 ms |
| 250 | 25505.1990 | 197234.4605 | 247 ms | 5.5 s | 13.6 s | 4.6 s |
| 1,000 | 23790.0451 | 192212.0330 | 1.3 s | 34 s | 157 s | 219 s |
| 5,000 | 23041.8864 | 189335.5688 | 12.4 s | 442 s | about 44 min | — |

The public data repository contains the 10, 100, 250, 1,000, and 5,000 tool
requests. The default opt-in csgrs run executes 10 and 100; stress enables 250,
and the dangerous gate enables 1,000 and 5,000.

## Run and interpret

```sh
SOLIDEAN_BENCH=1 benchmarks/run.sh
```

The local rows appear in `solidean-comparison.csv` and the generated summary.
Record host, compiler, commit, sibling Hyper crate commits, sample controls,
and enabled dome levels when publishing a comparison. Do not divide a local
time by the tables above without explicitly labeling the result as
cross-machine.
