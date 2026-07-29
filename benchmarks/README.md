# csgrs benchmark suite

This suite provides three complementary optimization anchors:

- `kernel_comparison` runs the same named CAD scenarios in csgrs, CGAL, and
  OpenCascade (OCCT).
- `feature_pipeline` covers csgrs-specific end-to-end paths and the underlying
  Hyper geometry stack, optional solid/curve-region features, adapters, and I/O.
- `competitive` compares the public csgrs mesh path against pinned `boolmesh`
  and pure-Rust Manifold versions. Since `tri-mesh` is a topology carrier
  rather than a Boolean kernel, it is compared on mesh import and half-edge
  validation. A dedicated large workload runs union, intersection, and
  difference on two 3,072-triangle closed meshes.
- `solidean_comparison` replays Solidean's published ordered Boolean requests
  in csgrs, preserving the published timing boundary and correctness oracles.
  It is opt-in because the datasets are downloaded and the larger requests are
  intentionally expensive.

Everything needed to understand or reproduce a measurement lives in this
directory:

- `rust/` contains the explicitly registered Cargo benchmark targets.
- `support/` contains the shared CSV harness and competitive correctness corpus.
- The shared corpus deterministically serializes a concave labyrinth and a
  level-3 Sierpiński foam below `target/benchmark-fixtures/generated`.
- `native/` contains the CGAL and OpenCascade counterparts.
- `results/` receives generated samples and summaries.
- `run.sh` orchestrates serialized runs; `summarize.py` validates and compares
  their output.

Every runner emits the same CSV schema. Raw samples are intentionally retained;
`summarize.py` validates them and reports median nanoseconds per operation plus
the ratio to csgrs. Full native runs also enforce that every `kernel`,
`precision`, and `corpus` workload has exactly one implementation in each of
the three engines. Opt-in `stress` and `dangerous` workloads are diagnostic
ceilings and are not part of the portable parity contract.
See [the cross-kernel coverage matrix](CROSS_KERNEL_COVERAGE.md) for
the portable API boundary and explicitly excluded nonportable surfaces.

## Run

Install CGAL and OpenCascade development packages so their CMake package files
are discoverable, then run:

```sh
benchmarks/run.sh
```

The standard run includes the Rust-only competitive matrix in
`rust-competitive.csv` and the generated summary.

For a one-sample build and corpus check:

```sh
benchmarks/run.sh --quick
```

Quick runs skip the long combined corpus Boolean and opt-in stress workloads.

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

The Rust-only competitive correctness corpus and release benchmark need no
native packages:

```sh
cargo test --test competitive
cargo bench --bench competitive
```

To download the pinned Solidean corpus and append its csgrs measurements to a
normal serialized run:

```sh
SOLIDEAN_BENCH=1 benchmarks/run.sh
```

This runs the first iterated CSG, terrain carve, 10×10×10 cube grid, and
10/100-tool dome requests. Add `CSGRS_BENCH_ENABLE_STRESS=1` for the 250-tool
dome. `CSGRS_BENCH_ENABLE_DANGEROUS=1` additionally enables the published
1,000- and 5,000-tool domes. The dome archive requires `7z`; acquisition also
uses `curl`, `tar`, and `sha256sum`. See
[Solidean protocol and published references](SOLIDEAN.md) before comparing
times recorded on different hardware.

Results are written below `benchmarks/results/` by default and ignored by Git.
Pass `--output DIRECTORY` to select a durable result location. Native discovery
honors normal CMake controls such as `CMAKE_PREFIX_PATH` and the package-specific
`CGAL_DIR` or `OpenCASCADE_DIR` environment/toolchain configuration.

The runners share these environment variables:

| Variable | Default | Meaning |
|---|---:|---|
| `CSGRS_BENCH_SAMPLES` | 10 | Independently timed samples per case |
| `CSGRS_BENCH_WARMUP` | 2 | Untimed warmup batches per case |
| `CSGRS_BENCH_STRESS_SAMPLES` | 1 | Samples for `corpus`, `competitive-large`, `competitive-yeahright`, `stress`, and `dangerous` cases |
| `CSGRS_BENCH_STRESS_WARMUP` | 0 | Untimed warmup batches for `corpus`, `competitive-large`, `competitive-yeahright`, `stress`, and `dangerous` cases |
| `CSGRS_BENCH_SCALE` | 1 | Multiplier for iterations within each sample |
| `CSGRS_BENCH_ITERATIONS` | unset | Override iterations per sample; the cold runner sets this to 1 |
| `CSGRS_BENCH_SAMPLE_OFFSET` | 0 | First emitted sample number |
| `CSGRS_BENCH_TEMPERATURE` | `warm` | CSV temperature label: `cold` or `warm` |
| `CSGRS_BENCH_COLD_SAMPLES` | 5 | Fresh-process samples used by `--cold-warm` |
| `CSGRS_BENCH_FILTER` | unset | Comma-separated substrings matched against `suite/benchmark/case` |
| `CSGRS_BENCH_SKIP_STRESS` | unset | Set to a nonzero value to skip normal stress cases; `--quick` sets this |
| `CSGRS_BENCH_SKIP_CORPUS_BOOLEANS` | unset | Set to a nonzero value to skip the combined corpus Boolean; `--quick` sets this |
| `CSGRS_BENCH_ENABLE_STRESS` | unset | Set to exactly `1` to enable rotated-copy stress cases that may consume tens of GiB |
| `CSGRS_BENCH_ENABLE_DANGEROUS` | unset | Set to exactly `1` to enable workloads known to risk exhausting host memory |
| `YEAHRIGHT_BENCH` | unset | Set to `1` to download and run the optional public-domain mesh corpus from `target/benchmark-fixtures/yeahright` |
| `SOLIDEAN_BENCH` | unset | Set to `1` to download and replay the pinned Solidean published benchmark requests |

For example, this runs only Boolean cases with five samples:

```sh
CSGRS_BENCH_FILTER=boolean CSGRS_BENCH_SAMPLES=5 benchmarks/run.sh
```

## Runtime path tracing

Enable `dispatch-trace` to record the exact-computation path selected by every
timed `feature_pipeline` workload, including public mesh paths through
`hypermesh` and public curve-region paths through `hypercurve`:

```sh
cargo test --lib --all-features dispatch_trace_tests
cargo bench --all-features --bench feature_pipeline
```

The focused tests fail if representative public mesh or curve-region Booleans emit
neither a dispatch event nor rational-reducer evidence. Every benchmark window
also records a `csgrs-benchmark/entry/recorded-workload` marker and fails on an empty trace, so
finite adapters and serialization paths remain correlated even when they do
not enter exact arithmetic. The benchmark prints a correlation summary and
per-operation trace for each independently recorded workload.

## Comparison protocol

The comparison corpus fixes dimensions, tessellation counts, transforms, and
operation names in source. Timed work includes result materialization and the
lightweight output-complexity/checksum traversal. Transform rows fingerprint
coordinates and face order/orientation so the result cannot collapse into a
topology-only no-op. Setup objects that can be
reused without changing operation semantics are built outside the timed region.
Failed Booleans, invalid mass properties, or failed serialization abort a run
instead of producing deceptively fast rows.

The Solidean runner follows the request files rather than approximating their
scenario in code. OBJ parsing and request decoding occur outside the timed
window. Timed work begins with conversion from indexed `f64` triangles into
the native Hypermesh carrier, includes every ordered Boolean, and ends once
the final mesh is materialized. The same execution then checks the published
area/volume oracle outside the recorded interval where one is available; the
cube grid must finish empty. This matches the published “import conversion plus
Booleans, excluding file I/O and export” boundary without performing each
expensive request twice.

The generated corpus contributes two closed native meshes to all three
engines: an 8,020-triangle orthogonal concave labyrinth and a 36,096-triangle
level-3 Sierpiński foam. Their position and triangle counts, manifold status,
and serialized bytes are hard regression checks.

The engines use their natural representations:

- csgrs uses native Hypermesh triangle meshes and Hypercurve regions.
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

Coverage is audited by executable public family. Data-only types, constants,
report accessors, and binding wrappers are validated through their producing
operation or their compile/test workflow rather than assigned meaningless
standalone timings. Each runtime-bearing family in the table has semantic
tests and a `feature_pipeline` or focused release benchmark. With
`dispatch-trace`, every `feature_pipeline` row is independently recorded;
mesh and curve-region trace integration is additionally enforced by
`dispatch_trace_tests`.

| Area | Representative cases | Principal implementation/dependencies |
|---|---|---|
| Exact scalar/lattice | expression evaluation; isolated translation, rotation, non-uniform scale, affine shear, mirror, and inverse | `hyperreal`, `hyperlattice` |
| 3D construction | every public mesh constructor: boxes, round/frustum solids, polyhedron, revolved/extruded specialty solids, Platonic solids, torus, arrow, and gear variants | csgrs mesh shapes |
| TriangleMesh topology | union, difference, intersection, XOR, inverse orientation, connectivity, manifold test, triangulation | `hypermesh`, `hypertri`, `hyperlimit` |
| Geometry algorithms | hull, Minkowski sum, subdivision, smoothing | `hypermesh`, csgrs mesh pipeline |
| Analysis/query | bounds, rays, exact mass properties, graphics buffers | `hyperlimit`, `hyperphysics` |
| 2D curve regions | every public region constructor (including polygon, Bezier/B-spline, gear/rack, airfoil, and Hilbert paths), all Booleans, all CSG transform helpers, triangulation, offsets | `hypercurve`, `hypertri` |
| 2D→3D | extrusion, revolution, twist, sweep, loft | `hypercurve`, csgrs mesh lowering |
| Implicit geometry | retained SDF, 2D/3D metaballs, TPMS catalog | `hypersdf`, `fast-surface-nets` |
| Projection | slice and flatten | `hypercurve`, `hyperlimit` |
| Raster/text | direct integer-grid image contours, TrueType outlines, Hershey strokes | `image`, `hypercurve`, `ttf-parser`, `hershey` |
| TriangleMesh I/O | STL, DXF, OBJ, PLY, AMF, glTF; STL/OBJ import, including exact scientific-notation OBJ coordinates | format crates, `serde_json`, `base64` |
| CurveRegion2 I/O | SVG and Gerber round trips | `svg`, `svgtypes`, `gerber-types`, `gerber_parser` |
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
