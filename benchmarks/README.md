# csgrs benchmark suite

This suite provides four complementary benchmark groups:

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
  in csgrs, with conversion-plus-operation timing and local final mass checks.
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

Every runner emits the same CSV schema. Raw samples are retained;
`summarize.py` checks headers, numeric fields, positive iterations and within-group
checksum stability, then reports median nanoseconds per iteration plus the ratio
to csgrs. One iteration can contain several operations. Native-run parity requires
the three engine labels for each observed `kernel`, `precision` and `corpus` key;
it cannot detect workloads absent from every engine or establish geometric equality.
Opt-in `stress` and `dangerous` workloads are diagnostic
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

The driver contains a cold/warm mode:

```sh
benchmarks/run.sh --cold-warm
```

The current cold/warm branch omits the `attributed` feature required by
`kernel_comparison`, so Cargo target selection fails until that configuration gap
is repaired. Its intended cold sampling launches a fresh runner process and sets
ordinary warmups to zero and iterations to one. Stress-family sample settings are
separate, and setup/earlier cases can warm caches inside each process. Only setup
outside a closure is excluded from timing; constructor cases time their construction.

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
Pass an absolute `--output DIRECTORY` when invoking outside the repository root;
relative paths are used before and after the driver's directory change. Failures
leave partial files; no completion manifest is written. Native discovery
honors normal CMake controls such as `CMAKE_PREFIX_PATH` and the package-specific
`CGAL_DIR` or `OpenCASCADE_DIR` environment/toolchain configuration.

The runners share these environment variables:

| Variable | Default | Meaning |
|---|---:|---|
| `CSGRS_BENCH_SAMPLES` | 10 | Independently timed samples per case |
| `CSGRS_BENCH_WARMUP` | 2 | Untimed warmup batches per case |
| `CSGRS_BENCH_STRESS_SAMPLES` | 1 | Rust samples for `corpus`, `competitive-large`, `competitive-yeahright`, `competitive-full`, `published-solidean`, `stress`, and `dangerous`; native runners use it for `corpus`, `stress`, `dangerous` |
| `CSGRS_BENCH_STRESS_WARMUP` | 0 | Untimed batches for the same stress-family suites |
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

Parsing is not identical across runners. Rust trims filter tokens and an empty
filter selects nothing; native filters preserve whitespace and an empty value
selects all gated cases. Native integer parsing accepts prefixes and uses wrapping
arithmetic where the Rust harness uses whole-value parsing and saturation.

For example, this selects Boolean cases with five ordinary samples:

```sh
CSGRS_BENCH_FILTER=boolean CSGRS_BENCH_SAMPLES=5 benchmarks/run.sh
```

The focused mesh context benchmark subtracts 16- and 32-segment drill cylinders
from a 12×12×4 body under both `GeometryContext` policies:

```sh
cargo bench --locked --no-default-features --bench mesh_boolean_context
```

It reports completed operations, consumed approximation, undecided predicates,
triangle counts, and first/median operation times in microseconds. Construction
is excluded. Each policy starts with fresh mesh storage; later samples reuse
those inputs and their retained facts. Set `CSGRS_BOOLEAN_CONTEXT_BENCH_ITERS`
to change the default five samples. Scalar expression caches may already be
warm, so the first sample is not a cold-process measurement.
The retained assertions expect STRICT to be undecided and APPROXIMATE_512 to
complete using approximation. Those are different completion contracts, and newly
certified completion would fail these historical assertions.
The [2026-09-05 measurement](mesh-boolean-context-2026-09-05.md) records the
workload, local timings, output counts, and correctness checks.

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

The comparison corpus fixes nominal dimensions and labels in source. The current
runners do **not** establish matched geometric inputs or completed work for every
same-named row. CSGRS boxes span from the origin; native boxes are centered. Thus
the sphere/box and contained-box scenarios differ geometrically. CSGRS sphere
samples have Y-axis poles and native exact trigonometric coordinates; CGAL uses
Z-axis poles and double samples; OCCT uses an analytic sphere.

Timing includes each closure's implemented work and destruction, which varies by
engine. For example, native graphics cases count corners rather than exporting
graphics buffers, CGAL mass-properties computes volume alone, and OCCT subdivision
is finer remeshing. Identical-box intersection in both native runners uses an
identity shortcut. Checksums and engine-label parity cannot certify equivalence.
See [workload definitions and differences](WORKLOADS.md) before interpreting ratios.
Failed calls with explicit success checks abort; most measured rows do not contain
an independent geometric oracle or expose certainty receipts.

The Solidean runner follows the request files rather than approximating their
scenario in code. OBJ parsing and request decoding occur outside the timed
window. Timed work begins with conversion from indexed `f64` triangles into
the native Hypermesh carrier, includes every ordered Boolean, and ends once
the final mesh is materialized. The same execution then checks the published
area/volume reference outside the recorded interval where one is available. The
cube grid's intended result is empty, but the local check only requires projected
area/volume near zero; it does not assert canonical emptiness or check every
intermediate shape. Optional diagnostics and intermediate destruction are included
in execution timing. This follows the broad conversion-plus-Booleans timing scope,
not necessarily every publisher implementation's detailed boundary or oracle.

The generated corpus contributes two closed native meshes to all three
engines: an 8,020-triangle orthogonal concave labyrinth and a 36,096-triangle
level-3 Sierpiński-derived foam with diagonal edge contacts filled by the generator.
The local regression checks fix position/triangle counts, indexed edge pairing and
serialized-byte stability. Those checks alone do not prove vertex-link manifoldness,
embeddedness or identical post-import geometry in all three engines.

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

CGAL EPECK supplies exact predicates and constructions through its lazy exact
field type. This runner's sampled round primitives begin with binary floating-point
trigonometric coordinates; EPECK promotes those values exactly. These are not the
same coordinate construction as CSGRS native exact expressions. A compile-time guard
prevents a built-in floating-point `Kernel::FT` from being substituted accidentally.
Translation, non-uniform scaling, plane reflection, and affine shear use
integer or rational coefficients in csgrs and CGAL, so those transform paths
remain exact after construction. The rotation row is called out separately
because it projects points to double, performs the entire rotation in double,
then promotes the resulting coordinates.

OCCT does not expose an arbitrary-precision scalar mode: `Standard_Real` is an
IEEE double and accuracy is managed by algorithm and topology tolerances. The
explicit tight profile therefore:

- sets Boolean fuzzy tolerance to zero, so no tolerance is added beyond the
  values stored on OCCT topology;
- requests non-parallel execution for the explicit Boolean Build call;
- exercises a nominal `1e-6` thin overlap, with a compile-time assertion that this
  exceeds the installed `Precision::Confusion()` value;
- includes a dedicated high-resolution sphere mesh using linear deflection
  `0.005` and angular deflection `0.07` radians. Meshing itself requests parallel
  execution and does not return a source-fidelity receipt.

The OCCT rows are consequently labeled `opencascade-double-tight`, not
"exact" or "arbitrary precision." These are selected runner settings, not a proof
of the highest achievable accuracy.

Comparisons need per-row review of inputs, operations and validated outputs.
`work_units` and `output_size` are case-specific nominal counters, not uniform
complexity measures. Some OCCT inputs are hard-coded polygon-counterpart counts;
analytic query rows can use one. CSV ratios alone do not compare matched contracts.
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

This table indexes representative benchmark families; it is not a complete public
API inventory or proof that every listed path currently completes. Data-only types,
accessors and bindings require their own producing-operation/consumer validation.
With `dispatch-trace`, every `feature_pipeline` row is independently recorded;
mesh and curve-region trace integration is additionally enforced by
`dispatch_trace_tests`.

| Area | Representative cases | Principal implementation/dependencies |
|---|---|---|
| Exact scalar/lattice | expression evaluation; isolated translation, rotation, non-uniform scale, affine shear, mirror, and inverse | `hyperreal`, `hyperlattice` |
| 3D construction | selected boxes, round/frustum solids, polyhedron, extruded gears, Platonic solids, torus and arrow | csgrs mesh shapes |
| TriangleMesh topology | union, difference, intersection, XOR, inverse orientation, connectivity, manifold test, triangulation | `hypermesh`, `hypertri`, `hyperlimit` |
| Geometry algorithms | hull, Minkowski sum, subdivision, smoothing | `hypermesh`, csgrs mesh pipeline |
| Analysis/query | bounds, rays, exact mass properties, graphics buffers | `hyperlimit`, `hyperphysics` |
| 2D curve regions | selected polygon/Bezier/B-spline/gear/rack/airfoil/Hilbert cases, four Booleans, selected transforms, triangulation and offsets | `hypercurve`, `hypertri` |
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
cross-kernel geometry hash. CSGRS transform fingerprints quantize projected
coordinates to a nominal `1e-9` grid; CGAL hashes projected double bits, and OCCT
hashes bounds/orientation. None proves exact equality. `summarize.py` rejects
changing raw aggregate checksums within a group, so differing iteration counts can
also trigger rejection. Review geometric and measurement differences before making
performance claims, even when counts and checksums are stable.
