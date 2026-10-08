# Stack Predicate-Lift Audit

Audit date: 2026-07-28

## Scope

This audit followed exact or predicate-like decisions from CSGRS through
Hypermesh, Hypercurve, Hypertri, Hypersolve, Hyperphysics, Hypersdf, Hyperbrep,
Hyperpath, and Hyperdrc. Hypervoxel was included after the dependency scan
showed that it directly consumes Hyperlimit's public 3D AABB classifiers.

The review looked for:

- determinant, sign, equality, incidence, interval, AABB, collinearity, and
  intersection decisions implemented above Hyperlimit;
- local primitive or exact-rational fast paths that could become an earlier
  stage of a canonical Hyperlimit cascade;
- semantic boundaries that make a local test intentionally different, such as
  preview tolerances, solver-selected refinement precision, or conservative
  binary broad phases.

## Retained lifts

### Projected 3D triangle degeneracy

Hypermesh's plane validator owned a separate 80-line projected determinant
cascade. Hyperlimit's canonical `classify_triangle3_degeneracy` now borrows the
original coordinates and advances all XY/XZ/YZ projections together through
certified primitive, word-sized rational, arbitrary rational, and general Real
stages. Hypermesh consumes the explicit result.

The direct common-case row improved from 436.90 ns to 33.72 ns (92.26%).
Hypermesh's 6,144-triangle soup improved 1.23%; its small cube-pair row was
within noise. CSGRS medium-sphere construction stayed at 75.3--75.6 us warm.

Commits:

- Hyperlimit `03aaf30` (`Cascade triangle degeneracy predicates`)
- Hypermesh `c6b85b15` (`Use canonical triangle degeneracy predicate`)

### Ordered exact AABB decisions

Hypermesh had separate exact overlap, containment, and relative-interior loops.
CSGRS also rebuilt its mesh broad-phase disjoint and point-outside decisions
from scalar comparisons. Hyperlimit now owns ordered 3D overlap, containment,
and relative-interior predicates with a borrowed exact-rational batch stage and
a general Real fallback. The same batch stage accelerates the existing
endpoint-normalizing 3D point and intersection classifiers.

The general exact intersection row improved from 113.18 ns to 86.60 ns
(23.56%), and point classification improved from 58.95 ns to 54.98 ns (6.48%).
Ordered full-axis overlap, containment, and relative-interior rows measured
24.50 ns, 24.12 ns, and 29.94 ns.

Commits:

- Hyperlimit `3ee4350` (`Batch ordered AABB predicates`)
- Hypermesh `041d5c00` (`Use canonical ordered AABB predicates`)
- CSGRS `05c6221` (`Use canonical exact AABB predicates`)

## Serialized performance gate

| Affected crate | Serial workload | Result |
|---|---|---:|
| Hyperlimit | general 3D AABB intersection | 23.56% faster |
| Hyperlimit | general 3D AABB point | 6.48% faster |
| Hyperlimit | ordered overlap / contains / relative interior | 24.50 / 24.12 / 29.94 ns |
| Hypermesh | polygon soup, 6,144 triangles | +0.68%, no detected change (`p = 0.22`) |
| Hypermesh | subdivided-cube union, 192 cells | -0.34%, no detected change (`p = 0.69`) |
| Hypervoxel | exact box voxelization | 4.89% faster |
| Hypervoxel | triangle-solid voxelization | 3.89% faster |
| Hypersdf | AABB point / cell classification | 238.72 / 252.83 ns |
| Hyperbrep | face-AABB preflight | 417.26 us |
| Hyperphysics | exact AABB contact replay | 535 ns |
| CSGRS | box / medium sphere, isolated warm | 1.019 / 75.633 us |
| CSGRS | disjoint union / intersection / difference | 25.132 / 0.253 / 1.083 us |

All output sizes and checksums were preserved. The CSGRS cross-kernel
fresh-process gate measured 1.247 us/81.478 us warm and 77.455 us/1.445 ms cold
for box/sphere construction; a 20-sample isolated repeat returned
1.019 us/75.633 us, identifying harness/process drift rather than an
attributable steady-state regression.

## Remaining candidates

### Coordinate-borrowed 3D segment construction

`hypermesh/src/output.rs` still contains
`proper_segment_intersection_after_bounds_overlap`,
`point_on_segment_exact`, and a local cross-product collinearity test.
Hyperlimit already classifies 3D segment relations, but it does not construct a
proper 3D crossing point and its public carrier is `Point3`, while Hypermesh
uses `OutputVertex`.

This is a valid future lift only if Hyperlimit gains a borrowed-coordinate
construction that reuses the classifier's certified nonparallel component and
parameter numerators. Replacing the local code with cloned `Point3` carriers
would add allocation and repeat work. Retention must be gated on an exact
boundary T-junction fixture, not only a cube Boolean that rarely enters this
repair path.

### Ordered 2D curve AABBs

`hypercurve/src/bbox.rs` has high-frequency ordered overlap and point-membership
loops. Its exact mode could use a 2D counterpart of the new batch cascade.
However, `NumericMode::EdgePreview` deliberately applies a tolerance, and
`region_nesting::aabb_may_contain` orders predicates using lossy separation
only to choose the first exact rejection.

A future lift should route only the non-preview exact branch to ordered
Hyperlimit AABB2 predicates and retain the preview and comparison-order
heuristics locally. The gates should include sparse curve-string intersection,
unordered native-segment region construction, region Boolean, and the CSGRS
profile-primitive suite.

## Reviewed and retained above Hyperlimit

| Layer | Predicate-like code | Disposition |
|---|---|---|
| Hypertri | orientation, incircle/insphere, and scalar-sign kernel adapters | Already route exact decisions through Hyperlimit; local code maps domain enums and generic kernels. |
| Hypersolve | certified zero/sign, polynomial trimming, algebraic interval containment | Keep local: callers select solver precision and distinguish algebraic/linear-algebra failure modes from topology predicates. |
| Hyperphysics | shape/contact sign helpers | Already delegate to Hyperlimit and only map `PredicateOutcome` into physics errors and `RealSign`. |
| Hypersdf | exact point/cell classifiers | Already use Hyperlimit. Remaining raw `f64` sign and bounds tests are preview meshing proposals, not topology evidence. |
| Hyperbrep | face AABB, plane, segment, surface, and trim preflights | Already compose Hyperlimit reports; local code packages retained B-rep evidence and readiness blockers. |
| Hyperpath | PCB clearance distances, tangent joins, curve-cell equality | Keep local: most construct exact distance/margin values or preserve routing-specific acceptance classes. Scalar equality adapters already call Hyperlimit ordering. |
| Hyperdrc | board margins and finite bounds | Keep local: these operate on primitive-float source grids, outward-conservative boxes, or DRC policy margins. Exact arc orientation already uses Hyperlimit. |
| Hypercurve | algebraic-root equality, weight-sign, preview bounds | Keep local except the exact ordered-AABB2 candidate above; these retain curve policy, algebraic witnesses, and explicit uncertainty reasons. |
| CSGRS | outward-rounded `f64` AABB rejection and bounded scalar helpers | Keep local: the former is a conservative compatibility accelerator; the latter intentionally use operation-specific bounded refinement or input validation. |
| Hypermesh | plane/halfspace evidence, projective classification, cached probes | Already route final decisions through Hyperlimit or retain cache/evidence ownership that a stateless predicate cannot represent. |

## Audit boundary

A test belongs in Hyperlimit when it decides reusable geometric or scalar
semantics over exact carriers and can expose uncertainty without importing a
downstream error type. It stays above Hyperlimit when it:

- only chooses work using conservative primitive-float data;
- applies a domain tolerance or caller-selected refinement budget;
- constructs solver, routing, B-rep, or cache evidence rather than deciding a
  reusable predicate;
- depends on retained topology identities or operation-specific acceptance
  policy.

Under that boundary, the duplicate triangle and ordered-AABB decisions were
lifted, the two remaining reusable candidates are explicit, and the other
reviewed helpers are intentional adapters or domain logic rather than missing
Hyperlimit predicates.
