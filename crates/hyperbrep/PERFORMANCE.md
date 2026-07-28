# HyperBrep reference and performance audit

This audit maps every item in the README reference list to the retained BREP
implementation. Optimizations were accepted only when Criterion measurements
showed a repeatable improvement and the exact report contents and validation
gates remained unchanged.

## Retained optimizations

The measurements below are Criterion steady-state times from the same release
build and machine. Each row compares the implementation immediately before and
after the named change.

| Benchmark | Before | After | Change |
| --- | ---: | ---: | ---: |
| 1,024-edge agreement report | 9.680 ms | 3.885 ms | 59.9% faster |
| 1,024-edge face validation report | 988.1 us | 897.8 us | 9.1% faster |
| 128-loop trim-set report | 1.109 ms | 25.98 us | 97.7% faster |
| solid readiness report | 5.075 ms | 3.880 ms | 23.6% faster |

The edge-agreement report now builds its vertex, face, and surface identity maps
once per shell report instead of once per edge. Face validation passes its
already-built trim report into geometry validation. A face trim-set report
similarly shares one shell lookup across its outer and inner loops. Finally,
solid readiness passes its closure and face-validation evidence into volume
replay instead of rebuilding both report families.

These are evidence-reuse changes, not relaxed checks. The public standalone
entry points still build the evidence they require, while composed reports can
reuse immutable evidence from the same shell snapshot. The 128-loop benchmark
is deliberately a report-pressure sentinel made from many disjoint loops; it
does not claim that the resulting outer/inner-loop geometry is a valid material
region.

## Immediate AABB API gate

Face and shell bounds reports now expose their retained exact corners directly
through `exact_bounds`. The former prepared bounds wrappers only borrowed those
same corners and forwarded each query, so face AABB preflight now calls the
immediate `hyperlimit` predicate without an intermediate carrier.

The focused face-preflight benchmark measured 414.40 us before and 413.79 us
after. Criterion found no performance change (`p = 0.91`), with a
-0.37% to +0.35% confidence interval. Exact reports, blockers, and
narrow-phase scheduling rules are unchanged.

## Immediate plane-evidence API gate

Surface and face-query reuse now retains `Plane3Evidence` and calls HyperLimit's
immediate classifiers. `BrepSurfaceEvidence` and `BrepFaceQueryEvidence`
describe the reusable data directly; the old prepared wrappers, readiness
names, and forwarding plane methods are gone.

The old face-query source was reconstructed from the committed revisions in an
isolated sibling tree so the serialized 100-sample Criterion comparison used a
genuine pre-change implementation:

| Benchmark | Before median | After median | Change |
| --- | ---: | ---: | ---: |
| Face-query evidence derivation | 207.01 us | 207.06 us | +0.02% |
| 1,024 point plus 128 segment face-query batch | 188.25 us | 166.73 us | -11.43% |
| 1,024 plane-surface point reports | 28.980 us | 15.654 us | -45.98% |

Criterion found no construction change and measured both query improvements as
significant. The planar hot path resolves the surface family once and inlines
the evidence classifier across the crate boundary; the generic function still
preserves explicit unsupported surface reports. Exact report contents,
blockers, and narrow-phase rules remain unchanged.

## Immediate voxel-geometry API gate

`BrepShell::voxel_geometry` now constructs HyperVoxel's validated
`ExactTriangleSolid` directly. The handoff no longer exposes a preparation
verb, a partly usable triangle wrapper, or a separate readiness report.

The affected HyperVoxel comparisons improved exact-solid construction from
8.313 us to 7.148 us (14.0%) and depth-three tetrahedron voxelization from
5.388 ms to 5.298 ms (1.7%). HyperBrep's retained cube sentinels measured
247.8 us for voxel-geometry construction and 376.4 us for voxel
materialization after the migration. Exact AABB, triangle-source, policy, and
predicate-certificate semantics are unchanged.

## Immediate planar-construction API gate

The public planar constructors now return a finished `BrepShell` or a typed
multi-blocker error. This removes the former construction carriers, which
combined an optional shell, a redundant readiness boolean, duplicate output
counts, and blockers in one publicly constructible state.

The existing Criterion construction sentinels were measured serially before
and after the change. Initial baselines were 28.834--29.043 us for a planar
face, 136.50--138.34 us for a simple prism, and 300.83--304.06 us for a holed
prism. Clean post-change runs measured 28.684--28.810, 136.38--136.87, and
303.02--304.66 us respectively; the holed intervals overlap. A longer,
immediately sequential old/new A/B confirmation measured 308.56--313.04 us
for the committed API and 304.41--305.26 us for the immediate API, about 1.9%
faster at the midpoint. An earlier post-change sample was discarded because
the host's scheduled rootkit scan was active during that measurement.

## Reference disposition

### Yap: exact geometric computation

Yap's separation between exact decisions and approximation is the crate's
central contract. Coordinates and determinant-based decisions remain
`hyperreal::Real`; unknown predicate outcomes become typed blockers. None of
the retained changes substitutes a float comparison or cached approximate
answer for exact replay.

### Mantyla: solid modeling and half-edge structure

The stable vertex, edge, coedge, loop, face, and shell identities implement the
book's incidence-oriented view of a boundary representation. The shared edge
and trim lookup tables are a direct performance application: incidence is
indexed once at shell or face scope, then replayed through the existing
manifold and orientation checks. Mutable Euler operators are not introduced
because the current API is an immutable retained record with explicit
construction reports.

### Piegl and Tiller: NURBS

Spatial NURBS already validate knot/control/weight relationships, evaluate in
homogeneous coordinates, and project only at the end. Homogeneous control nets
are cached with `OnceCell`. Moving that machinery into topology reports would
duplicate `BrepCurve3` ownership, so the existing curve-local cache is retained
unchanged.

### Farin: CAGD curves and surfaces

Bezier and spline evaluation already follows the affine/homogeneous algorithms
owned by `hypercurve` and the spatial curve module. Native and reversed pcurve
images are cached at the pcurve carrier. No second BREP-specific evaluator was
added; keeping one exact curve representation avoids disagreement between
topology and geometry layers.

The public API exposes the exact operations and their results, not whether an
internal `OnceCell` happens to be populated. Removing the four cache-state
queries leaves homogeneous controls, reversed pcurves, and native segment views
retained exactly as before. Before that API-only change, serialized Criterion
ranges were 1.6070–1.6548 us for a rational Bezier point, 1.8464–1.8591 us for
a NURBS point, 288.40–289.52 ns for pcurve image equality, and
184.65–186.12 ns for a face edge-use query. Final serialized measurements were
1.5952–1.6087 us, 1.8122–1.8730 us, 285.09–286.30 ns, and
183.66–184.12 ns respectively. The retained native-segment accessor now takes
an explicit already-initialized fast path, which preserves lazy first use while
making the repeated immediate edge-use query slightly faster.

### Meisters: polygon ears

Exact planar tessellation delegates polygon triangulation to `hypertri`, whose
own reference audit covers ear clipping and exact orientation predicates.
Duplicating an ear-clipping implementation inside HyperBrep would add a second
topological decision path without improving the retained representation.

### Mirtich: polyhedral mass properties

HyperBrep certifies closed, oriented, exact planar shells and hands exact
triangles to `hyperphysics`, the owner of mass-property accumulation. Its local
volume report keeps exact determinant accumulation as a readiness certificate.
Higher moments and projection-axis accumulation belong in HyperPhysics and are
audited there rather than duplicated at the BREP boundary.

### Requicha and Voelcker: representation completeness

The retained BREP remains source truth; tessellations, physics shapes, voxels,
and exports are derived handoffs with provenance and blockers. This follows the
reference's distinction between a solid representation and downstream display
or analysis artifacts. The performance changes preserve that boundary and
reuse only evidence from the same source snapshot.

### Weiler: topological structures

Weiler's explicit adjacency and radial-incidence model motivates the edge-use
agreement reports and the shell-scoped identity maps. The retained lookup
changes remove repeated reconstruction of those incidence indexes. A mutable
radial-edge structure, non-manifold editing, sewing, and boolean operators are
larger representation changes and remain deferred until they can carry the
same exact report and provenance contracts.

## Deferred ideas

- A persistent prepared-shell identity context could share maps across every
  report family, but it needs an explicit snapshot/freshness API to prevent
  evidence from being replayed against a different shell value.
- General analytic surfaces, curved trim-edge reconstruction, periodic NURBS,
  and geometric equality across differently partitioned pcurves remain named
  capability gaps rather than approximated operations.
- Mutable radial-edge editing, Euler operators, sewing, and BREP booleans need
  transactional topology validation and construction provenance.
- Cross-thread curve caches would require changing the current `Rc`/`OnceCell`
  ownership model and have no measured workload justifying that cost.
- Polyhedral centers of mass and inertia tensors are owned by HyperPhysics;
  BREP should provide certified geometry and reuse its returned evidence.

No measured local experiment in this audit regressed or failed its semantic
checks; all four bounded evidence-reuse experiments were retained. Architectural
ideas without a safe local implementation or representative benchmark were
deferred rather than treated as optimizations.

## Validation protocol

Run from the crate root:

```sh
cargo fmt --all -- --check
cargo test --all-targets --locked
cargo clippy --all-targets --locked -- -D warnings
cargo check --all-targets --locked
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
cargo bench --bench shell_audit --locked
```

The benchmark targets include the large edge, face, trim-set, and composed
solid-report sentinels used above. Unit and integration tests cover blocker
contents as well as readiness booleans, so evidence reuse cannot silently turn
a previously blocked case into an accepted one.
