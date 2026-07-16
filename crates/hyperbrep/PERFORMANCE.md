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
