# csgrs benchmark suite

This suite provides two complementary optimization anchors:

- `kernel_comparison` runs the same named CAD scenarios in csgrs, CGAL, and
  OpenCascade (OCCT).
- `feature_pipeline` covers csgrs-specific end-to-end paths and the underlying
  Hyper geometry stack, optional mesh/profile features, adapters, and I/O.

Every runner emits the same CSV schema. Raw samples are intentionally retained;
`summarize.py` validates them and reports median nanoseconds per operation plus
the ratio to csgrs. Full native runs also enforce that every `kernel` and
`precision` workload has exactly one implementation in each of the three
engines. See [the cross-kernel coverage matrix](CROSS_KERNEL_COVERAGE.md) for
the portable API boundary and explicitly excluded nonportable surfaces.

## Run

Install CGAL and OpenCascade development packages so their CMake package files
are discoverable, then run:

```sh
benchmarks/run.sh
```

For a one-sample build and corpus check:

```sh
benchmarks/run.sh --quick
```

To record cold and warm cross-kernel results together:

```sh
benchmarks/run.sh --cold-warm
```

Cold samples launch each runner in a fresh process, disable warmups, and force
one operation per case. Warm samples retain the normal warmup and iteration
settings. Input construction remains outside the timed region in both modes.

To exercise only csgrs on a host without the native kernels:

```sh
benchmarks/run.sh --quick --csgrs-only
```

Results are written below `benchmarks/results/` by default and ignored by Git.
Pass `--output DIRECTORY` to select a durable result location. Native discovery
honors normal CMake controls such as `CMAKE_PREFIX_PATH` and the package-specific
`CGAL_DIR` or `OpenCASCADE_DIR` environment/toolchain configuration.

The runners share these environment variables:

| Variable | Default | Meaning |
|---|---:|---|
| `CSGRS_BENCH_SAMPLES` | 10 | Independently timed samples per case |
| `CSGRS_BENCH_WARMUP` | 2 | Untimed warmup batches per case |
| `CSGRS_BENCH_SCALE` | 1 | Multiplier for iterations within each sample |
| `CSGRS_BENCH_ITERATIONS` | unset | Override iterations per sample; the cold runner sets this to 1 |
| `CSGRS_BENCH_SAMPLE_OFFSET` | 0 | First emitted sample number |
| `CSGRS_BENCH_TEMPERATURE` | `warm` | CSV temperature label: `cold` or `warm` |
| `CSGRS_BENCH_COLD_SAMPLES` | 5 | Fresh-process samples used by `--cold-warm` |
| `CSGRS_BENCH_FILTER` | unset | Comma-separated substrings matched against `suite/benchmark/case` |

For example, this runs only Boolean cases with five samples:

```sh
CSGRS_BENCH_FILTER=boolean CSGRS_BENCH_SAMPLES=5 benchmarks/run.sh
```

## Comparison protocol

The comparison corpus fixes dimensions, tessellation counts, transforms, and
operation names in source. Timed work includes result materialization and the
lightweight output-complexity/checksum traversal. Transform rows fingerprint
coordinates and face order/orientation so the result cannot collapse into a
topology-only no-op. Setup objects that can be
reused without changing operation semantics are built outside the timed region.
Failed Booleans, invalid mass properties, or failed serialization abort a run
instead of producing deceptively fast rows.

The engines use their natural representations:

- csgrs uses Hyper-backed exact-aware mesh/profile carriers.
- `cgal-epeck` uses `Exact_predicates_exact_constructions_kernel` (EPECK) and
  triangulated `Surface_mesh` inputs.
- `opencascade-double-tight` uses analytic B-reps, no added Boolean fuzzy
  tolerance, and tessellates results for comparable output-size reporting.
  Construction, isolated translation/rotation/scaling/affine/mirror/inverse
  transforms, all four Booleans, and extrusion therefore include OCCT
  result meshing.

### High-precision policy

CGAL EPECK supplies exact predicates and exact constructions through its lazy
exact field type. Sampled round primitives necessarily begin with the fixed
binary floating-point trigonometric samples shared by the corpus, but EPECK
promotes those values exactly and performs subsequent topology and construction
decisions in the exact kernel. The runner has a compile-time guard preventing a
floating-point `Kernel::FT` from being substituted accidentally.
Translation, non-uniform scaling, plane reflection, and affine shear use
integer or rational coefficients in csgrs and CGAL, so those transform paths
remain exact after construction. The rotation row is called out separately
because its fixed sine/cosine samples originate in floating-point before EPECK
promotes them exactly.

OCCT does not expose an arbitrary-precision scalar mode: `Standard_Real` is an
IEEE double and accuracy is managed by algorithm and topology tolerances. The
explicit tight profile therefore:

- sets Boolean fuzzy tolerance to zero, so no tolerance is added beyond the
  values stored on OCCT topology;
- keeps deterministic, non-parallel Boolean execution for repeatability;
- exercises a `1e-6` thin overlap, ten times OCCT's documented
  `Precision::Confusion()` value of `1e-7`;
- includes a dedicated high-resolution sphere mesh using linear deflection
  `0.005` and angular deflection `0.07` radians, safely above OCCT's minimum
  `Precision::Confusion()` and `Precision::Angular()` contracts.

The OCCT rows are consequently labeled `opencascade-double-tight`, not
"exact" or "arbitrary precision." This is the highest-accuracy mode represented
by these native B-rep workloads without mischaracterizing OCCT's numeric model.

This makes the results representative end-to-end CAD comparisons, not claims
that the internal algorithms or representations are identical. `work_units`
records input facets when a facet stream exists, while `output_size` records
output triangles/facets. OCCT's analytic analysis cases report B-rep input faces.
The fixed sphere/box Boolean uses a deliberately moderate 12×6 mesh on the
polygon kernels because exact topology dominates runtime; larger 32×16 and
64×32 meshes remain in construction, transformation, analysis, and I/O cases.
The `precision` suite adds a 128×64 sphere and the thin-overlap Boolean to make
exact/tight behavior visible independently of ordinary throughput cases.

OCCT's stable STL writer is file-oriented. Its `stl_write` row therefore
includes temporary-file I/O, while the csgrs and CGAL rows use in-memory
serialization. The row remains useful as an end-to-end public API comparison,
but serialization-only conclusions should account for that boundary difference.

## Coverage

| Area | Representative cases | Principal implementation/dependencies |
|---|---|---|
| Exact scalar/lattice | expression evaluation; isolated translation, rotation, non-uniform scale, affine shear, mirror, and inverse | `hyperreal`, `hyperlattice` |
| 3D construction | box, sphere, cylinder, torus, ellipsoid, icosahedron | csgrs mesh shapes |
| Mesh topology | union, difference, intersection, XOR, inverse orientation, connectivity, manifold test, triangulation | `hypermesh`, `hypertri`, `hyperlimit` |
| Geometry algorithms | hull, Minkowski sum, subdivision, smoothing | `hypermesh`, csgrs mesh pipeline |
| Analysis/query | bounds, rays, exact mass properties, graphics buffers | `hyperlimit`, `hyperphysics` |
| 2D profiles | shape catalog, all Booleans, all CSG transform helpers, triangulation, offsets | `hypercurve`, `hypertri`, `geo` |
| 2D→3D | extrusion, revolution, twist, sweep, loft | `hypercurve`, csgrs mesh lowering |
| Implicit geometry | retained SDF, 2D/3D metaballs, TPMS catalog | `hypersdf`, `fast-surface-nets` |
| Projection | slice and flatten | `hypercurve`, `hyperlimit` |
| Raster/text | image contours, TrueType outlines, Hershey strokes | `image`, `contour_tracing`, `ttf-parser`, `hershey` |
| Mesh I/O | STL, DXF, OBJ, PLY, AMF, glTF; STL/OBJ import | format crates, `serde_json`, `base64` |
| Profile I/O | SVG and Gerber round trips | `svg`, `svgtypes`, `gerber-types`, `gerber_parser` |
| Adapters/parts | Bevy conversion, AABB blueprint extraction | `bevy_mesh`, csgrs parts |

`voxels` currently exposes no executable public operation. WASM, language FFI,
and packaging are target/binding layers rather than geometry kernels; keep their
existing compile/test workflows alongside these runtime benchmarks.

## Interpreting and preserving results

Compare like-for-like builds on an idle, fixed-frequency machine. Record the
CPU, operating system, compiler versions, csgrs commit, sibling Hyper commits,
CGAL/OCCT versions, and relevant environment values with any published result.
Do not compare a `--quick` run to a full run.

`checksum` is a repeatability guard within one engine and case; it is not a
cross-kernel geometry hash. Transform-coordinate fingerprints are quantized to
one-billionth of a model unit at the primitive export boundary so equivalent
symbolic `Real` graphs do not differ only because of adaptive f64 approximation
state. `summarize.py` rejects changing checksums across
samples. Output-size differences across engines should be reviewed before
attributing a speed ratio solely to implementation quality.
