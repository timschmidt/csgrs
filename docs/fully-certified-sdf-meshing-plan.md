# Fully Certified SDF-to-Hypermesh Meshing Plan

## Status and purpose

CSGRS now has a native-`Real` regular-grid Surface Nets path. Exact shifted
samples are passed to Hypermesh, which performs sign decisions, edge
interpolation, cell centroids, diagonal selection, and triangle-degeneracy
checks without lowering construction data to `f32` or `f64`. The result is exact
triangle geometry for a sampled topology proposal.

That removes the primitive-float precision ceiling. It does **not** prove that a
finite regular grid captured the zero set of the continuous field. This plan
defines the additional work required for a certified path that either returns a
proof-bearing mesh or an explicit blocker.

## Certification target

The certified API must make two independent claims:

1. **Mesh validity:** every emitted coordinate and predicate remains in the
   Hyper exact-computation boundary, and Hypermesh certifies nondegenerate,
   consistently oriented, closed PWN geometry when a closed solid is requested.
2. **Field correspondence:** the retained `SdfExpr`, selected iso value, domain,
   subdivision, root evidence, and topology decisions prove that the mesh has
   the intended relationship to the continuous level set.

For the initial certified subset, field correspondence should mean an ambient
isotopic approximation of the regular level set inside the requested domain,
plus an exact conservative geometric-error bound. If the proof obligations are
not met, the operation returns no certified mesh. It may return a separately
labelled proposal mesh for diagnostics, but proposal data must never inherit a
certified status.

“Fully certified” does not mean that every possible `SdfExpr` is guaranteed to
terminate successfully. Expressions with unsupported intervals, undecidable
predicates, singular zero sets, coincident zero regions, or exhausted refinement
budgets must return typed blockers.

## Ownership and dependency boundaries

The implementation should preserve the existing stack boundaries:

- **CSGRS** owns the solid-modeling API, options, backward compatibility, and
  orchestration.
- **Hypersdf** owns the retained shifted field `f(point) - iso`, exact point and
  cell classification, scalar intervals, gradients, Lipschitz evidence, edge
  root reports, and local topology evidence.
- **Hypervoxel** owns adaptive octree storage, neighbor balancing, leaf identity,
  and deterministic refinement order.
- **Hypermesh** owns exact vertex/triangle geometry, shared vertex identity,
  degeneracy and duplicate rejection, orientation, closure, and PWN validation.

The first bridge should live in `csgrs::implicit` because CSGRS already depends
on all required layers. Hypermesh must not depend on Hypersdf. If a second
independent consumer appears, extract the orchestration and certificate schema
into a dedicated adapter crate rather than reversing that dependency.

## Proposed API contract

Add a fallible API alongside the existing sampled API. A representative shape
is:

```rust,ignore
pub fn try_sdf_expr_certified(
    expression: SdfExpr,
    domain: Aabb,
    iso: Real,
    options: CertifiedSdfMeshingOptions,
) -> Result<CertifiedSdfMeshOutcome, CertifiedSdfMeshingError>;
```

`CertifiedSdfMeshingOptions` should include:

- strict or explicitly selected predicate policy;
- maximum octree depth and work/memory budgets;
- required exact geometric-error bound;
- boundary policy;
- supported regularity policy;
- optional proposal generation for diagnostics;
- deterministic parallelism controls.

The initial boundary policy should be `RequireEnclosed`: the field interval on
every domain-boundary cell must exclude zero. Later policies may add
`CertifiedOpenSurface` and `ClipAndCap`, but they need separate contracts.

`CertifiedSdfMeshOutcome` should contain:

- the `TriangleMesh`;
- Hypermesh predicate certainty;
- a field-topology certification status distinct from mesh certainty;
- exact domain and iso value;
- exact geometric-error bound;
- counts and provenance for cells, roots, topology decisions, and refinement;
- retained certificate or a stable certificate digest suitable for replay;
- an optional sampled proposal report kept explicitly separate.

Blockers should identify at least unsupported intervals, unknown sample or cell
signs, non-regular cells, nonsmooth CSG ties, zero regions, multiple or
unisolated edge roots, ambiguous face/interior topology, boundary intersection,
refinement exhaustion, predicate exhaustion, invalid contour complexes, and
Hypermesh validation failure.

Closure-field `solid::sdf` inputs cannot be certified because their computation
cannot be structurally replayed. Certification initially applies only to
retained `SdfExpr` inputs.

## Required evidence pipeline

### 1. Exact iso shifting

Construct and retain the expression for `f - iso` once. All point signs, cell
intervals, gradients, roots, diagnostics, and replay must refer to that same
object. The iso shift must never be applied only to a preview sample buffer.

### 2. Conservative adaptive cell classification

Classify every candidate cell using Hypersdf interval and cell reports:

- an interval wholly below zero is inside;
- an interval wholly above zero is outside;
- an interval containing zero is active or requires refinement;
- missing or undecidable interval evidence is a blocker unless subdivision
  produces supported children within the configured budget.

Corner signs alone are insufficient: a closed component or thin feature can be
entirely contained in a cell whose corners all have the same sign.

Hypervoxel should retain the adaptive tree. Neighbor leaves must be balanced to
the transition rule required by the contour complex, and traversal/identity
must be deterministic.

### 3. Regularity and local topology proof

Adopt and document an interval-subdivision theorem for implicit surfaces rather
than promoting heuristic case tables. Each accepted active leaf must prove the
local hypotheses used by that theorem, such as:

- zero is in the scalar interval;
- the zero set is not singular in the cell;
- a gradient component or equivalent monotonic direction is bounded away from
  zero where required;
- the leaf contains one certified sheet with a determined face connection;
- neighboring leaf decisions agree on shared faces.

The first implementation should support smooth regular fields. Nonsmooth CSG
ties and intentional sharp features need a later exact branch-decomposition
proof; until then they remain explicit blockers rather than being smoothed or
guessed.

### 4. Certified edge-root isolation

Extend the current Hypersdf dual-contouring evidence beyond affine edges:

- retain the exact one-variable edge restriction;
- count and isolate roots on the closed edge;
- distinguish no root, one simple root, endpoint root, tangent root, multiple
  roots, and a coincident-zero edge;
- prove the ordering and ownership of roots shared by adjacent cells;
- retain an exact isolating interval and the evidence used to establish it.

Hypersolve can serve polynomial restrictions. General supported expressions need
interval subdivision or interval-Newton/Krawczyk replay. Unsupported
transcendental ranges must remain blocked.

An exact on-surface coordinate is not required for the first certified mesh.
The builder may select an exact dyadic point inside a certified root or vertex
box and carry the box as its error certificate. Hypermesh then operates on
exact coordinates, while field correspondence comes from the retained
isolation evidence. Requiring arbitrary nonlinear roots themselves to become
`Real` values would instead require a new algebraic/computable-root scalar
representation and should be treated as a separate project.

### 5. Certified contour complex

Build a topology-first intermediate complex before triangulation:

- stable adaptive-cell and shared-face identifiers;
- one or more certified surface patches per leaf as allowed by the theorem;
- shared edge-root identities and face arcs;
- oriented polygon loops derived from exact inside/outside decisions;
- transition elements for unequal neighboring leaf levels;
- provenance from every complex element back to its field evidence.

Ambiguous cells must subdivide or fail. Exact-zero vertices and edges require a
documented symbolic ownership rule; they must not be resolved with a tolerance.

### 6. Vertex placement and quality

Topology certification must not depend on a numerical QEF minimizer. Begin with
an exact dyadic representative inside each certified vertex box or a centroid of
certified edge representatives. Record its conservative distance bound.

QEF placement can be added as a quality optimization when:

- every Hermite row has certified provenance;
- rank-deficient systems have an exact, deterministic fallback;
- the result is constrained to the certified placement region;
- moving the vertex cannot change the certified topology or exceed the error
  bound.

### 7. Hypermesh assembly and validation

Translate the contour complex into shared `Point3<Real>` positions and indexed
triangles. Hypermesh must then verify:

- indices and shared identities are valid;
- triangles are exact-nondegenerate and unique;
- directed edge use is balanced;
- the result is closed when `RequireEnclosed` is selected;
- orientation agrees with the field’s negative-inside convention;
- the result passes strict PWN intake via `polygon_soup`.

Hypermesh validation certifies the emitted mesh, but it does not replace the
field-correspondence certificate. Both statuses must survive in the public
outcome.

## Delivery milestones

### Milestone 0: contracts and replay fixtures

- Define certification statuses, blockers, options, and report serialization.
- Specify the exact meaning of the geometric-error bound and topology claim.
- Add minimal certificate replay fixtures before implementing extraction.
- Ensure sampled APIs and the new native-`Real` Surface Nets path remain
  explicitly proposal-only.

### Milestone 1: regular-grid validation-ready subset

- Consume the existing affine dual-contouring handoff.
- Add exact shared-edge ownership and oriented quad/triangle connectivity.
- Support exact-zero-free, unambiguous regular-grid cases.
- Produce open certified patches for unit tests and require enclosure for public
  solid output.
- Pass Hypermesh exact degeneracy and connectivity validation.

### Milestone 2: smooth algebraic primitives

- Add unique root isolation for plane, sphere/quadric, cylinder, capsule, and
  supported polynomial edge restrictions.
- Add interval-gradient regularity proofs and exact root ordering.
- Certify enclosed sphere and transformed ellipsoid meshes end to end.
- Introduce exact dyadic placement and geometric-error certificates.

### Milestone 3: adaptive certified extraction

- Integrate Hypervoxel octree ownership and balanced refinement.
- Prove active-leaf local topology and shared-face agreement.
- Add crack-free adaptive transition elements.
- Demonstrate thin and fully cell-contained features that regular corner
  sampling misses.
- Establish deterministic results across thread counts.

### Milestone 4: CSG and degeneracies

- Decompose min/max CSG ties into retained exact branches.
- Certify supported sharp intersections and reject unsupported singularities.
- Add endpoint, tangent, multiple-root, and coincident-zero ownership rules.
- Add certified clipping/capping only after the enclosed contract is complete.

### Milestone 5: hardening and rollout

- Add bounded caches for point, interval, gradient, root, and face evidence.
- Use primitive proposals only to prioritize work; replay every accepted fact.
- Add certificate replay, fuzzing, performance budgets, and adversarial corpus
  minimization.
- Expose the certified API as opt-in until its supported-expression matrix and
  failure reporting are stable.
- Consider extraction into a standalone adapter crate only after a second
  consumer exists.

## Verification matrix

The test suite must cover:

- affine planes and nonzero iso shifts;
- spheres, ellipsoids, cylinders, capsules, tori, and transformed primitives;
- disconnected components, nested shells, and thin enclosed features;
- unions, intersections, complements, offsets, and branch ties;
- exact-zero corners, endpoint roots, tangent roots, multiple roots, and
  coincident-zero edges;
- checkerboard faces and interior ambiguous configurations;
- reversed bounds, anisotropic cells, extreme `Real` magnitudes, and coordinates
  below primitive-float spacing;
- boundary-free domains, deliberate boundary intersections, and later caps;
- refinement invariance, component/Euler-characteristic expectations, and
  deterministic serialization;
- strict Hypermesh uniqueness, closure, orientation, PWN intake, and a Boolean
  operation using the certified result;
- property tests asserting that every accepted local decision has retained
  evidence and every unsupported case has a blocker;
- fuzz targets for expression trees, grids, budgets, certificate replay, and the
  rule that proposal-only evidence can never produce a certified status.

Known analytic fixtures should check both topology and exact error bounds.
Adversarial fixtures should ensure that exhaustion produces a stable error, not
an empty mesh or a downgraded success.

## Performance rules

- Cache exact grid coordinates and shared point/cell evaluations.
- Evaluate intervals before corners so inactive regions are pruned cheaply.
- Isolate roots only on edges owned by active leaves.
- Allow an f32/f64 proposal to order refinement work, but never consume it as
  proof.
- Parallelize independent classification and root work while sorting all
  retained identities deterministically before assembly.
- Bound depth, root refinement, predicate work, memory, and certificate size
  independently; report which budget was exhausted.
- Benchmark proposal generation, exact replay, Hypermesh validation, peak
  memory, and certificate size separately.

## Completion criteria

The certified path is ready for general use when:

- no primitive-float value participates in an accepted topology or mesh
  construction decision;
- every successful mesh carries replayable field and mesh evidence;
- every unsupported or exhausted case returns a typed blocker;
- the supported smooth-expression subset has adaptive topology and geometric
  error guarantees documented against a named interval-subdivision theorem;
- strict Hypermesh validation and end-to-end Boolean consumption pass;
- adversarial, property, fuzz, determinism, and performance gates pass in CI;
- sampled and certified APIs remain visibly distinct in names, documentation,
  diagnostics, and types.
