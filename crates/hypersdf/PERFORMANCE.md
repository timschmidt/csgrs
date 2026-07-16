# HyperSDF performance and reference audit

This audit maps every reference in the crate README to the implementation surface that
can use it without weakening Hyper's exact-evidence boundary. Timings are Criterion
estimates from an optimized local build on 2026-07-15; they are comparative evidence,
not portable latency promises.

## Retained changes

| Path | Before | After | Criterion result |
|---|---:|---:|---:|
| affine cell interval | 21.90 us | 1.30 us | 94.1% faster |
| sphere point report | 752 ns | 510 ns | 32-34% faster |
| prepared six-point CSG batch | 8.22 us | 5.40 us | 34-38% faster |
| 4 x 4 x 4 exact-grid preview | 70.20 us | 58.92 us | 15.3% faster |
| mesh preview including that grid | 79.79 us | 60.87 us | 19.7% faster |
| sphere cell interval | 6.56 us | 1.47 us | 75.9% faster |
| sphere cell classification | 3.97 us | 1.03 us | 74.3% faster |
| sphere cell Lipschitz bound | 5.48 us | 1.28 us | 76.7% faster |
| AABB cell classification | 971 ns | 333 ns | 65.6% faster |

The retained implementations are:

- Arvo-style per-output-axis transformed-AABB accumulation instead of transforming
  eight corners. The same Moore interval form computes exact plane and linear ranges.
- A one-pass prepared point evaluator that retains each already-computed exact scalar
  while preserving the existing CSG location-composition rules.
- Per-axis exact grid-coordinate schedules, so an `x` coordinate is constructed once
  and reused for every `y,z` combination rather than rebuilt at every grid point.
- A shared separable farthest-squared-distance bound for sphere cell classification,
  intervals, and local Lipschitz reports.
- The axis-aligned containment observation that two opposite cell corners collectively
  cover both endpoints of every coordinate, replacing six redundant AABB point tests.

Generated properties cover arbitrary signed linear coefficients, reversed cell
endpoints, and sheared affine frames. The optional `dispatch-trace` test confirms that
exact point and affine-interval replay records sign/dispatch activity with zero
approximation and unknown-fact events.

## Reference mapping

### Gibson, Constrained Elastic Surface Nets

The existing `fast-surface-nets` adapter, crossing diagnostics, preview-only status,
and constrained cell ownership match the paper's surface-net role. Axis-coordinate
reuse accelerates its sampled-grid input. Elastic vertex relaxation was not moved into
the exact field layer: it changes proposal geometry, requires an application-owned
smoothing objective, and cannot certify topology from binary or lossy samples.

The README DOI was corrected from `10.1007/BFb0056308` to the published
`10.1007/BFb0056277` during this audit.

### Arvo, Transforming Axis-Aligned Bounding Boxes

Retained directly. An affine row is bounded by adding the smaller and larger products
of each coefficient with that axis's two endpoints. This gives exactly the same AABB
as eight-corner enumeration, including reversed endpoint input, with six products and
three comparisons per output coordinate instead of repeated 4D point transforms.

### Hart, Sphere Tracing

Hart's safe stepping requires a true distance or a certified Lipschitz bound. HyperSDF
already exposes local Lipschitz reports, and the shared sphere/AABB extremum makes that
report substantially cheaper. A ray marcher was deliberately not added: many retained
expressions are only `SignEquivalent`, and stepping by their raw scalar would violate
Hart's non-penetration premise. Ray candidates remain external proposals that must be
replayed against exact classification.

### Ju et al., Dual Contouring of Hermite Data

The current report already retains active edge ownership, exact affine roots, exact
normal directions, QEF rows, degeneracy blockers, and sampled-versus-exact provenance.
General floating QEF minimization was not promoted to certified geometry; nonlinear
roots and numerical minimizers remain proposals until exact replay. The grid cache
reduces report input construction, while the tiny 2 x 2 x 2 affine fixture was dominated
by report bookkeeping and showed no statistically significant end-to-end change.

### Lorensen and Cline, Marching Cubes

The paper's edge-crossing and regular-grid concepts are represented in mesh diagnostics,
but adding a second sampled mesh backend would not improve certified classification.
The original case-table route also does not resolve sampled face/interior ambiguities.
HyperSDF therefore keeps Surface Nets and dual/gradient contouring as explicit preview
or validation proposals rather than treating a Marching Cubes table as topology proof.

### Moore, Interval Analysis

Retained directly. Natural interval extensions cover arithmetic and CSG nodes, while
affine forms use dependency-free per-axis accumulation for tight exact bounds. The
shared sphere extremum is the corresponding separable range calculation for squared
distance. Unsupported trigonometric ranges remain unknown instead of receiving an
uncertified sampled enclosure.

### Frisken et al., Adaptively Sampled Distance Fields

Per-axis coordinate reuse improves regular sampling and downstream mesh previews.
Adaptive subdivision itself remains with the voxel/octree owner: HyperSDF produces
certified cell classifications and an explicit `hypervoxel` handoff, avoiding a second
competing tree and preserving one source of frame/indexing truth.

### Yap, Towards Exact Geometric Computation

Retained through object preparation, square-root-free predicates, exact interval and
sign decisions, and the one-pass report evaluator. The evaluator now exploits context
across what were formerly separate classification and scalar calls, matching Yap's
recommendation that expression/object packages expose cross-call optimization. Lossy
sampling, finite-difference normals, and mesh/QEF candidates remain named adapters and
never become topology evidence without exact replay.

## Considered but not retained

- Elastic smoothing, general numerical QEF minimization, and Marching Cubes case tables
  belong to proposal geometry and offered no stronger exact evidence contract here.
- Raw sphere-tracing steps are unsafe for `SignEquivalent` fields; only certified
  Lipschitz information is exposed.
- A second adaptive octree inside HyperSDF would duplicate `hypervoxel` ownership and
  invite frame/version divergence.
- The small dual-contouring affine benchmark did not materially benefit from the grid
  coordinate cache; the cache was retained because the direct grid and mesh benchmarks
  improved significantly and the formula is identical.
