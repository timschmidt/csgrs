# Solidean published Boolean benchmarks

`solidean_comparison` replays the public request files used in Solidean's 2026
Boolean-kernel articles. The fixture loader pins
[`solidean/bench-blog-data`](https://github.com/solidean/bench-blog-data) at
commit `80900cf867fb690e6c7571880857e82f6f45ae34`, verifies the downloaded archive
and selected request files by SHA-256, and caches them below
`target/benchmark-fixtures/solidean` (or the selected Cargo target directory).
Reused extracted OBJs are not individually hashed; derived dome caches check only
three filenames for presence. The
[pinned upstream license notice](https://github.com/solidean/bench-blog-data/blob/80900cf867fb690e6c7571880857e82f6f45ae34/README.md)
defaults non-code assets to CC BY 4.0, subject to asset-specific overrides.
Dataset assets are not vendored into csgrs.

The local boundary follows the broad import-plus-operation scope described in the
[publisher's measurement notes](https://solidean.com/blog/2026/first-benchmark-results-iterated-csg/#measurement):
request and OBJ parsing and disk I/O are excluded; conversion from indexed
double-precision triangles, all ordered Boolean operations, and final result
materialization are included. csgrs executes each request serially. The final
projected area and absolute signed volume check runs after the per-sample timer
stops. Optional per-operation diagnostics and intermediate destruction are inside
execution timing; final-output destruction is outside it. OBJ decimals pass through
binary64 before exact native ingress, and local certainty receipts are not emitted.

The cube grid is intended to become empty, but the local runner only checks
area/volume against zero with absolute tolerance `5e-4`. It neither asserts a
canonical empty carrier nor checks every intermediate shape, unlike the publisher's
[per-step grid validation](https://solidean.com/blog/2026/iterated-cube-grid-benchmark/).
Other references use `max(5e-4, abs(expected) * 1e-8)` after checking finite measures.
These checks are not independent topology or exact-set oracles. A successful local
CSV row therefore must not be equated with the publisher's full correctness verdict.

## Published reference times

These values are transcribed from the linked Solidean articles. They were
recorded on the publisher's hardware—principally a Ryzen 9 5900X, 64 GiB
DDR4-2666, and Windows 11—and may include each kernel's supported parallelism.
Local csgrs CSV rows describe local attempts; their execution, validation scope and
host must be recorded separately before comparing them with these historical times.

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
It describes the workload as 223 subtractions even though the
[pinned version-one request](https://github.com/solidean/bench-blog-data/blob/80900cf867fb690e6c7571880857e82f6f45ae34/blog/2026/terrain-carve-benchmark/0.2_-y.request.json)
contains 224 difference operations (tools `0.obj` through `223.obj`). The csgrs
row follows those instructions and carries `_224`
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

The publisher records CGAL corefine declining an early non-manifold intermediate;
its partial timing is not a completed-run time.

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
