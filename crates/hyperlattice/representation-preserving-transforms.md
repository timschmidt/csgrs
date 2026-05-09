# Representation-Preserving Transform Plan

## Goal

Make rigid transforms, affine transforms, and coordinate changes preserve
`hyperreal`/`realistic_blas` representation instead of eagerly expanding every
coordinate into independent scalar expression trees.

The central rule is:

> A transform should mostly rewrite structure. It should not force numeric
> approximation, flatten shared factors, or discard facts that were already
> known before the transform.

This plan is cross-crate:

- `hyperreal` owns scalar representation, exact facts, planning facts, symbolic
  affine scalar forms, and demand-driven approximation.
- `realistic_blas` owns vector/matrix APIs, fixed-size affine kernels, and
  opportunities to share transform state across many output coordinates.

## Current Friction

`realistic_blas` already routes fixed-size dot products through backend hooks,
and the `HyperrealBackend` delegates to `Real::dot3_refs` / `Real::dot4_refs`.
That is the right direction: a matrix row times a vector is not just arbitrary
arithmetic; it is an affine combination with repeated structure.

The missing layer is representation-preserving transform structure. Today, a
coordinate transform tends to become:

```text
x' = m00*x + m01*y + m02*z + tx
y' = m10*x + m11*y + m12*z + ty
z' = m20*x + m21*y + m22*z + tz
```

as separate scalar expressions. That loses information the crates could have
kept:

- all coordinates were transformed by the same matrix
- each output is an affine combination, not an arbitrary expression
- many terms share scales, symbolic factors, or exact-rational denominators
- zeros, signs, exactness, rational-only status, and rough magnitudes may be
  derivable without approximation
- approximation precision for the object can often be planned as a group

## Design Principles

### 1. Transforms Preserve Representation

A rigid transform, affine transform, or coordinate change should rewrite the
representation whenever possible.

Examples:

- multiplying by an identity or permutation matrix should rename/reorder
  coordinates
- multiplying by a diagonal scale should attach shared scale structure rather
  than expand every lane independently
- translating by an exact vector should produce offset/affine nodes, not force
  approximate coordinates
- applying the same matrix to many points should be represented as a shared
  transform context plus per-point inputs when that is cheaper than expansion

The expanded arithmetic result is still the semantic fallback, but it should not
be the default internal shape for common geometry transforms.

### 2. Semantic Facts Propagate Through Transforms

Transforms should update cheap facts exactly and conservatively:

- zero / nonzero status
- sign when implied by structure
- exact-rational availability
- rational-only status for whole expressions
- shared scale factors
- conservative magnitude bounds

No fact propagation rule may turn a planning heuristic into a public semantic
answer. This should follow the existing `hyperreal` split between conservative
public structural facts and internal planning facts.

Examples:

- `0` transformed by any linear map remains `0`
- a coordinate copied by a permutation keeps its exact rational status, sign,
  and magnitude facts
- a positive uniform scale preserves signs; a negative uniform scale flips them
- an exact-rational matrix applied to an exact-rational vector stays exact
  rational
- a shared nonzero scale can be carried as object-level metadata and combined
  with per-coordinate facts only when answering a coordinate query

### 3. Approximation Is Demand-Driven

No coordinate should be approximated merely because a transform was applied.

Transform construction should produce a lazy representation. Approximation is
requested only when a caller asks for:

- an `f32`/`f64` approximation
- bounded sign refinement
- a predicate/filter requiring numeric intervals
- formatting that explicitly asks for decimal output

Even then, approximation should be scheduled from the transform structure. A
whole transformed point, vector, matrix, or batch can share input approximations,
matrix approximations, and magnitude planning rather than asking each scalar
coordinate to rediscover the same facts.

### 4. Common Affine Patterns Get Internal Forms

`hyperreal` has already benefited from first-class special forms such as
`SinPi`, `TanPi`, selected logs, roots, factored products, and rational scales.
Affine geometry deserves the same treatment.

Candidate scalar-level internal forms:

- `LinearCombination`: `sum_i coeff_i * value_i`
- `AffineCombination`: `offset + sum_i coeff_i * value_i`
- `SharedScale`: `scale * value` where `scale` may be exact rational,
  symbolic, or object-level
- fixed arity variants for hot paths: 2, 3, and 4 lane combinations
- signed product-sum variants that generalize current determinant/cofactor
  fusion

Candidate object-level internal forms in `realistic_blas`:

- `TransformedVector3`
- `TransformedVector4`
- `AffinePoint3`
- `AffinePoint4`
- `TransformedMatrix3`
- `TransformedMatrix4`
- batch transform handles for many vectors/points using the same matrix

These do not need to become public types immediately. The first useful version
can be backend hooks and private constructors that produce optimized scalar
forms while preserving existing public APIs.

### 5. Group Operations Share Factors Aggressively

When a whole object is transformed by one matrix, common structure should be
kept at the group level as long as possible.

Useful shared state:

- matrix entries
- translation entries
- common rational denominator
- common dyadic shift
- uniform scale
- determinant or orientation scale for predicate-facing transforms
- symbolic factors such as `pi`, `e`, `sqrt(n)`, or shared computable nodes
- cached input approximations for a batch
- conservative row/column magnitude summaries

This is especially important for geometry. Rigid and affine transforms often
apply to thousands of coordinates where every lane repeats the same matrix
terms. Expanding each coordinate independently makes later sign, magnitude, and
approximation work repeat itself.

## Proposed Architecture

### `hyperreal`: Scalar Affine Forms

Add internal representation for affine scalar expressions before generic
`Computable` expansion.

Initial shape:

```text
AffineCombination {
    offset: Real,
    terms: smallvec[(Real coeff, Real value)],
    facts: cached exact/planning facts,
}
```

The implementation does not need to literally use this struct name or layout.
The important properties are:

- fixed-size hot paths avoid heap work where possible
- exact-rational terms are fused through shared-denominator arithmetic
- zero terms and identity coefficients are removed structurally
- common symbolic factors are carried instead of multiplied into every term
- public facts are derived conservatively without approximation
- fallback conversion to existing `Real` arithmetic remains available

Fact propagation should distinguish:

- exact/public facts: safe for `structural_facts()`, `zero_status()`,
  `sign()`, constructor rewrites, and checked APIs
- planning facts: safe for precision selection, scheduling, and internal
  approximation only

### `realistic_blas`: Transform-Aware Kernels

Add backend-neutral transform hooks that can preserve structure for exact
backends and remain cheap for approximate backends.

Candidate hooks:

```text
affine3(matrix: [[&Scalar; 3]; 3], translation: [&Scalar; 3], point: [&Scalar; 3])
linear3(matrix: [[&Scalar; 3]; 3], vector: [&Scalar; 3])
affine4(matrix: [[&Scalar; 4]; 4], vector: [&Scalar; 4])
batch_affine3(matrix, translation, points)
```

For `ApproxBackend`, these can initially lower to existing arithmetic. For
`HyperrealBackend`, they should call `hyperreal` constructors that preserve
affine combination structure and shared matrix state where possible.

Existing `dot3`, `dot4`, and `signed_product_sum2` specializations are the
starting point, not the destination.

## Fact Propagation Rules

Start with cheap exact rules only.

Zero:

- zero coefficient removes a term
- zero value removes a term
- empty affine sum becomes the offset
- linear transform of a zero vector is zero
- translation-only result keeps translation facts

Sign:

- copied coordinates keep sign
- positive exact scale preserves sign
- negative exact scale flips sign
- sums get sign only when all nonzero terms have a provably common sign and no
  cancellation is possible under the rule being used

Exactness:

- exact-rational coefficients plus exact-rational values plus exact-rational
  offset produce exact rational output
- rational-only affine forms stay rational-only until a non-rational term is
  introduced
- symbolic non-rational terms keep their symbolic certificate rather than
  becoming generic computable values

Magnitude:

- pure scale adjusts magnitude by exact scale bits when known
- dyadic scales update `msd` exactly
- non-dyadic exact-rational scales can update a conservative bound without
  decimal approximation
- affine sums use conservative max-plus-one style bounds unless exact
  cancellation is structurally proven

Shared scale:

- object-level uniform scale should remain object-level
- row-level common factors should remain row-level
- common denominator extraction should happen once for the group, not per
  coordinate

## Approximation Scheduling

Affine forms should approximate only under request. The scheduler should:

1. inspect exact facts first
2. use planning facts to choose working precision
3. approximate shared matrix/translation/input terms once
4. evaluate all requested coordinates against those shared approximations
5. cache coordinate results and group-level planning summaries separately

Important non-goals:

- do not approximate all coordinates when only one coordinate is requested
- do not refine sign for every lane when a caller asked for an approximate
  vector
- do not expose planning-only sign or magnitude facts through public APIs

## Staged Plan

### Stage 1: Document and Trace Existing Transform Shapes

Deliverables:

- this planning document
- dispatch traces for matrix-vector, matrix-matrix, point transform, and
  predicate workloads
- benchmarks that separate exact-rational, symbolic, mixed symbolic, and
  approximate transform cases

### Stage 2: Scalar Affine Constructors in `hyperreal`

Add private constructors for fixed-size affine combinations:

- `Real::linear_combination3_refs`
- `Real::linear_combination4_refs`
- `Real::affine_combination3_refs`
- `Real::affine_combination4_refs`

Initially, these can wrap existing `dot3_refs` / `dot4_refs` plus offset, but
the call sites should move to the affine names so later representation changes
do not require another API sweep.

### Stage 3: Fact Propagation for Affine Forms

Add exact/public fact propagation for the simple cases:

- zero lanes
- copied/permuted lanes
- exact-rational-only affine sums
- dyadic and sign-known scales
- conservative magnitude bounds for fixed-size sums

Keep planning facts separate from public facts.

### Stage 4: Backend Transform Hooks in `realistic_blas`

Introduce backend hooks for fixed-size linear and affine transforms.

Initial implementation:

- `ApproxBackend`: existing arithmetic
- `HyperrealBackend`: scalar affine constructors
- public matrix/vector APIs remain source-compatible

Then update matrix-vector and homogeneous-coordinate paths to call these hooks
instead of building independent coordinate expressions.

### Stage 5: Shared Group Representation

Add optional private object-level transform wrappers for repeated transforms:

- transformed vector/point views
- transformed matrix views
- batch transform handles

These should be introduced only where traces show repeated shared factors or
approximation requests. The purpose is to share matrix terms, common scales,
and approximation caches across coordinates.

### Stage 6: Demand-Driven Group Approximation

Teach `hyperreal`/`realistic_blas` approximation paths to evaluate affine
groups as groups:

- one precision plan per shared transform
- one approximation per shared matrix/input term
- coordinate results derived from shared work
- abort-aware scheduling for long refinements

This stage should build on the evaluator refactor direction: explicit fact
propagation first, approximation second.

## Tests

Add targeted tests for:

- identity, permutation, diagonal, and translation transforms
- exact-rational transform preserving exact rational output
- symbolic coordinates preserving symbolic classes through transform
- zero/sign propagation through positive and negative scales
- dyadic scale magnitude updates
- no approximation triggered during transform construction
- one-coordinate approximation not forcing all sibling coordinates
- batch transform sharing matrix/input approximation work

Regression tests should include both `HyperrealBackend` and `ApproxBackend`,
with backend-specific expectations only where representation differs.

## Benchmarks

Add benchmark groups for:

- matrix-vector transform over exact rationals
- matrix-vector transform over `pi`, `e`, `sqrt` symbolic values
- affine point transform with exact translation
- repeated transform of many points by one matrix
- determinant/orientation workloads after a common affine transform
- approximation of one coordinate vs all coordinates
- sign refinement after transform

Useful measurements:

- constructor count from dispatch tracing
- cloned `Real`/`Computable` graph count where traceable
- exact-rational denominator reductions
- approximation calls per transformed object
- peak expression depth
- wall time for construction, fact query, and approximation separately

## First Increment

The first implementation step should be deliberately small:

1. add affine-named scalar constructors in `hyperreal` that delegate to current
   fixed-size dot-product machinery
2. add backend hooks in `realistic_blas` for linear/affine 3D and 4D operations
   that delegate to existing arithmetic
3. route one hot matrix-vector path through the new hook
4. add dispatch trace labels and exact-rational/symbolic benchmarks

That creates the architectural slot without changing public behavior. Later
stages can make the constructors preserve richer affine forms and shared group
state behind the same internal API.

## Progress Log (2026-05-09)

### Completed

- Added fixed-arity linear/affine backend hooks in `src/backend/hyperreal/mod.rs` and
  `src/backend/approx/mod.rs`, including:
  - `linear_combination3` / `linear_combination4`
  - `affine_combination3` / `affine_combination4`
- Updated `src/lib.rs` scalar fast-path routing to call
  `linear_combination*` / `affine_combination*` with transform-oriented trace
  labels.
- Route `Matrix3/4 * Vector3/4` through the new fast paths in
  `src/matrix.rs` by:
  - replacing the ad-hoc owned-owned branch with shared helper usage, and
  - splitting homogeneous 4x4 form into 3-term linear combination + offset
    (`Scalar::affine_combination3`) so the backend can preserve affine shape.
- Extended dispatch trace accounting (`benches/mathbench/dispatch_trace.rs`) for
  the new fast-path labels and updated the profile explanation text.
- Added hyperreal matrix-transform fact-preservation tests in `tests/matrix.rs` for
  permutation, homogeneous point/direction behavior, and scaled+translated points.
- Routed hyperreal matrix hot paths through local `*_refs` naming shims for
  linear/affine constructors (`Real::linear_combination*_refs`,
  `Real::affine_combination*_refs`), preserving Stage-2 naming while staying on
  the pinned `hyperreal = "0.10.6"` surface.
- The public hyperreal backend no longer has to be bumped yet; the compatibility
  shims keep the transform constructors stable until a staged dependency upgrade
  is explicitly taken.
- Removed the attempted prewarmed constants path from benchmark setup after it showed
  no meaningful gain at this stage; constant/context initialization now remains
  demand-driven per process.
- Added targeted transform benchmark/trace coverage in
  `benches/mathbench/comparisons.rs` for exact-rational `Matrix3/4 * Vector3/4`
  paths on:
  - `matrix3/hyperreal-rational/mat3 transform vec3 one-coord approx`
  - `matrix3/hyperreal-rational/mat3 transform vec3 all-coord approx`
  - `matrix4/hyperreal-rational/mat4 transform vec4 one-coord approx`
  - `matrix4/hyperreal-rational/mat4 transform vec4 all-coord approx`
- Added focused CLI support in `benches/mathbench.rs` for comma-separated
  `--trace-dispatch-filter` arguments so dispatch tracing can run only the rows
  under inspection.
- Ran focused regression measurements (2026-05-09):
  - Dispatch trace confirms one-coordinate demand now emits 1 `to-f64-approx` per
    transform for both mat3/mat4, while all-coordinate demand emits 3/4
    respectively.
- `cargo bench ... --filter "one-coord approx"` results:
    - `matrix3/hyperreal-rational/mat3 transform vec3 one-coord approx`: ~4.68 µs
    - `matrix4/hyperreal-rational/mat4 transform vec4 one-coord approx`: ~1.75 µs
- `cargo bench ... --filter "all-coord approx"` results:
    - `matrix3/hyperreal-rational/mat3 transform vec3 all-coord approx`: ~4.83 µs
    - `matrix4/hyperreal-rational/mat4 transform vec4 all-coord approx`: ~1.83 µs
- Added code comments near the new comparison rows explaining the performance
  intent (demand-driven approximation and avoiding eager per-coordinate
  refinement).
- Added symbolic and predicate-oriented transform workloads in
  `benches/mathbench/comparisons.rs`:
  - `matrix3/hyperreal-symbolic/mat3 transform vec3`
  - `matrix3/hyperreal-symbolic/mat3 transform vec3 one-coord approx`
  - `matrix3/hyperreal-symbolic/mat3 transform vec3 all-coord approx`
  - `matrix4/hyperreal-symbolic/mat4 transform vec4`
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 one-coord approx`
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 all-coord approx`
  - `matrix3/hyperreal-rational/mat3 transform vec3 sign/zero facts`
  - `matrix4/hyperreal-rational/mat4 transform point vec4`
  - `matrix4/hyperreal-rational/mat4 transform direction vec4`
- Ran focused symbolic/predicate trace and timing sweeps:
  - Trace confirms one-coordinate symbolic demand now emits `to-f64-approx` only
    for the requested coordinate (plus one extra `refine_sign_until` for symbol
    planning in hyperreal) while all-coordinate demand emits all coordinates.
  - `cargo bench ... -- \"hyperreal-symbolic/mat3 transform vec3\"` results:
    - `matrix3/hyperreal-symbolic/mat3 transform vec3`: ~6.0 µs
    - `hyperreal-symbolic/mat3 transform vec3 one-coord approx`: ~8.7 µs
    - `hyperreal-symbolic/mat3 transform vec3 all-coord approx`: ~13.8 µs
  - `cargo bench ... -- \"hyperreal-symbolic/mat4 transform vec4|hyperreal-rational/mat4 transform point vec4|hyperreal-rational/mat4 transform direction vec4\"` results:
    - `matrix4/hyperreal-symbolic/mat4 transform vec4`: ~17.2 µs
    - `matrix4/hyperreal-symbolic/mat4 transform vec4 one-coord approx`: ~20.5 µs
    - `matrix4/hyperreal-symbolic/mat4 transform vec4 all-coord approx`: ~29.5 µs
    - `matrix4/hyperreal-rational/mat4 transform point vec4`: ~13.3 µs
    - `matrix4/hyperreal-rational/mat4 transform direction vec4`: ~4.05 µs
- Added Stage-2 constructor naming shims in `src/lib.rs`:
  - `Scalar::linear_combination3_refs`
  - `Scalar::linear_combination4_refs`
  - `Scalar::affine_combination3_refs`
  - `Scalar::affine_combination4_refs`
  and routed matrix-vector transform hotspots in `src/matrix.rs` to those names for
  forward-compat with the future `*_refs` backend API.
- Re-ran focused benchmark after migration:
  - `cargo bench ... -- \"hyperreal-symbolic/mat3 transform vec3 one-coord approx\"`
    returned ~8.73 µs (within noise of the prior value, no meaningful regression).
  - `cargo bench ... -- \"hyperreal-symbolic/mat4 transform vec4|hyperreal-symbolic/mat4 transform vec4 one-coord approx|hyperreal-symbolic/mat4 transform vec4 all-coord approx\"` results:
    - `matrix4/hyperreal-symbolic/mat4 transform vec4`: ~16.94 µs (small improvement versus prior run, likely noise-favorable)
    - `matrix4/hyperreal-symbolic/mat4 transform vec4 one-coord approx`: ~20.50 µs (within noise)
    - `matrix4/hyperreal-symbolic/mat4 transform vec4 all-coord approx`: ~29.28 µs (within noise)
- Added focused directional path verification on 2026-05-09:
  - `cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --trace-dispatch-filter=\"matrix4/hyperreal-rational/mat4 transform point vec4,matrix4/hyperreal-rational/mat4 transform direction vec4\" --dispatch-trace-only --write-dispatch-trace-md`
    - Directional transform row now records `realistic_blas` `scalar_fast_path` `linear-combination3-specialized` and backend `linear-combination3-specialized`.
    - Point-row trace unchanged: `realistic_blas` `scalar_fast_path` `linear-combination4-specialized`.
  - `cargo bench --bench mathbench -- \"hyperreal-rational/mat4 transform direction vec4\"` result: `4.0137 µs..4.0515 µs` (no significant change; Criterion reported no detectable performance change versus prior directional baseline).
- Added symbolic direction rows (full + one/all coordinate approximation demand) on 2026-05-09:
  - New trace rows in `benches/mathbench/comparisons.rs`:
    - `matrix4/hyperreal-symbolic/mat4 transform direction vec4`
    - `matrix4/hyperreal-symbolic/mat4 transform direction vec4 one-coord approx`
    - `matrix4/hyperreal-symbolic/mat4 transform direction vec4 all-coord approx`
  - Trace (`--trace-dispatch-filter` for the three new rows) shows 3-term behavior:
    - `realistic_blas` `scalar_fast_path` `linear-combination3-specialized` for all three symbolic direction rows.
    - One-coord approx row emits exactly 1 `realistic_blas` `scalar_query` `to-f64-approx` versus 4 in all-coord row.
  - Added cached direction-fact check (`right[3].zero_status()`) outside the row loop in
    `src/matrix.rs`; trace now shows `scalar_query` `zero-status` count dropped from 4 → 1 per
    benchmark row (one decision reused across all output coordinates).
  - Focused timing (`cargo bench --bench mathbench -- \"hyperreal-symbolic/mat4 transform direction vec4|hyperreal-symbolic/mat4 transform direction vec4 one-coord approx|hyperreal-symbolic/mat4 transform direction vec4 all-coord approx|hyperreal-rational/mat4 transform direction vec4\"`) results:
    - `matrix4/hyperreal-symbolic/mat4 transform direction vec4`: ~9.30 µs
    - `matrix4/hyperreal-symbolic/mat4 transform direction vec4 one-coord approx`: ~13.74 µs
    - `matrix4/hyperreal-symbolic/mat4 transform direction vec4 all-coord approx`: ~16.83 µs
    - `matrix4/hyperreal-rational/mat4 transform direction vec4`: ~3.88 µs
- Added one focused regression test on 2026-05-09 for Stage-3-style zero-lane fact preservation on direction transforms:
  - `hyperreal_matrix_transform_direction_zero_lane_facts` in `tests/matrix.rs`
  - Asserts expected zero/nonnull `ZeroStatus` outputs when direction inputs and
    sparse linear structure force zero lanes under the `w == 0` path.
- Added point-branch zero-coefficient fact pruning on 2026-05-09:
  - New benchmark row `matrix4/hyperreal-rational/mat4 transform vec4 no-translation` in
    `benches/mathbench/comparisons.rs`.
  - `transform_vector_rhs_ref` now uses `linear_combination3_refs` when either:
    - input `w` is known-zero (direction), or
    - matrix translation coefficient `left[row][3]` is known-zero.
  - Targeted trace (`matrix4/hyperreal-rational/mat4 transform vec4 no-translation`) shows
    `realistic_blas` `scalar_fast_path` `linear-combination3-specialized` with `scalar_query`
    `zero-status` overhead from fact checks on coefficient path.
  - Timed result:
  - `cargo bench --bench mathbench -- \"matrix4/hyperreal-rational/mat4 transform vec4 no-translation\"` → ~3.33 µs.
- Added explicit demand-demotion rows for no-translation affine loads to compare one-coordinate vs all-coordinate approximation:
  - `matrix4/hyperreal-rational/mat4 transform vec4 no-translation one-coord approx`
  - `matrix4/hyperreal-rational/mat4 transform vec4 no-translation all-coord approx`
- Ran focused timing (`cargo bench --bench mathbench -- \"matrix4/hyperreal-rational/mat4 transform vec4 no-translation|matrix4/hyperreal-rational/mat4 transform vec4 no-translation one-coord approx|matrix4/hyperreal-rational/mat4 transform vec4 no-translation all-coord approx\"`) and recorded:
  - `no-translation`: ~3.33 µs
  - `no-translation one-coord approx`: ~3.22 µs
  - `no-translation all-coord approx`: ~3.41 µs
- Captured fresh dispatch trace for the no-translation rows. All three now consistently route through `realistic_blas` `scalar_fast_path` `linear-combination3-specialized` (not 4-term); one-coord demand reports exactly 1 `to-f64-approx` and all-coord demand reports 4.
- Removed an accidental duplicate of
  `"matrix4/hyperreal-rational/mat4 transform vec4 one-coord approx"` in
  `benches/mathbench/comparisons.rs` after a failed duplication attempt in the
  previous edit pass.
- Added a dedicated no-translation structural-facts row to enforce demand-driven
  behavior:
  - `matrix4/hyperreal-rational/mat4 transform vec4 no-translation structural facts`
- Trace confirms this row:
  - routes through `linear-combination3-specialized`,
  - performs only `scalar_query` `structural-facts`,
  - does **not** emit `to-f64-approx` in dispatch accounting.
- Timed result for the structural-facts row:
  - `matrix4/hyperreal-rational/mat4 transform vec4 no-translation structural facts`: ~3.18 µs.
- Added a symbolic mirror:
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation structural facts`
- Trace confirms symbolic structural-facts probing also stays on 3-term fast path with
  `scalar_query` `structural-facts`/`zero-status` only and no explicit `to-f64-approx`.
- Timed result for the symbolic structural-facts row:
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation structural facts`: ~10.37 µs.
- Added a focused symbolic no-translation Stage-3 test:
  - `hyperreal_matrix_transform_symbolic_no_translation_zero_lane_facts` in
    `tests/matrix.rs`
  - Uses `Scalar::pi()`/`Scalar::e()` in the matrix terms, a direction input
    (`w == 0`), and explicit assertions for lane `zero_status` plus
    `structural_facts().sign` and `exact_rational` preservation.
- Added missing no-translation symbolic timing rows for mat4:
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation`
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation one-coord approx`
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation all-coord approx`
- Focused timing for the above rows:
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation`: ~9.02 µs
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation one-coord approx`:
    ~11.84 µs
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation all-coord approx`:
    ~19.17 µs
  - `matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation structural facts`:
    ~10.19 µs
- Separate dispatch captures for those symbolic no-translation rows confirm demand behavior:
  - one-coord approx: `scalar_query` `to-f64-approx` = 1
  - all-coord approx: `scalar_query` `to-f64-approx` = 4
  - structural facts: no `to-f64-approx`, only `structural-facts` + `zero-status`
- Command note:
  - `--trace-dispatch-filter` parsing is comma-delimited exact matches in this CLI path; using `|` regex alternation did not match all intended rows, so I switched to comma-separated filter values.
- Re-ran directional symbolic and rational timing to check post-edit stability:
  - `matrix4/hyperreal-symbolic/mat4 transform direction vec4`: ~9.46 µs
  - `matrix4/hyperreal-symbolic/mat4 transform direction vec4 one-coord approx`: ~13.53 µs
  - `matrix4/hyperreal-symbolic/mat4 transform direction vec4 all-coord approx`: ~16.83 µs
  - `matrix4/hyperreal-rational/mat4 transform direction vec4`: ~4.09 µs
- Reworked nearby comments near transform dispatch branching in `src/matrix.rs` to record the decision rationale:
  direction vectors can drop the translation term, and points are reduced to 3-term
  form when translation coefficients are proven zero.
- Added directional structural-facts coverage in `benches/mathbench/comparisons.rs`:
  - `matrix4/hyperreal-rational/mat4 transform direction vec4 structural facts`
  - `matrix4/hyperreal-rational/mat4 transform direction vec4` in trace mode now
    appears with structural-facts queries and no `to-f64-approx` emission for the
    structural row.
- Focused regression validation for that addition:
  - `cargo bench --bench mathbench -- --trace-dispatch-filter=\"matrix4/hyperreal-rational/mat4 transform direction vec4,matrix4/hyperreal-rational/mat4 transform direction vec4 structural facts\" --dispatch-trace-only --write-dispatch-trace-md`
  - `cargo bench --bench mathbench -- \"matrix4/hyperreal-rational/mat4 transform direction vec4|matrix4/hyperreal-rational/mat4 transform direction vec4 structural facts\"`
  - Initial short-run capture looked like a directional regression:
    `~4.4383 µs..5.0036 µs` vs prior baseline.
  - Follow-up long-sample rerun with shared baseline (`matrix4/hyperreal-rational/mat4 transform vec4|matrix4/hyperreal-rational/mat4 transform direction vec4|matrix4/hyperreal-rational/mat4 transform direction vec4 structural facts`, 500 samples) showed `~3.80..3.81 µs` for the `transform vec4` row and `~3.80..3.97 µs` for directional/structural rows, indicating improved throughput and ruling out the earlier regression concern.
- Routed full `N=4` point transform rows through `Scalar::affine_combination4_refs` with a
  zero offset sentinel, while `Scalar::affine_combination4` collapses that zero-offset
  case back to `linear_combination4_refs` internally.
- This also removed the previously observed unused constructor warning by exercising
  `linear_combination4_refs` through the affine-zero fast path; confirmed by `cargo check` warning-free.
- Re-ran targeted benchmarks for the affected direction + structural-facts rows after the
  constructor reroute and captured stable improvement in measured rows rather than a
  regression.
  - `matrix4/hyperreal-rational/mat4 transform vec4`: ~1.7768..1.7823 µs
  - `matrix4/hyperreal-rational/mat4 transform direction vec4`: ~3.8018..3.8077 µs
  - `matrix4/hyperreal-rational/mat4 transform direction vec4 structural facts`: ~3.9619..3.9724 µs

### Unsuccessful / Deferred

- Did not introduce the optional shared transform object layer (`TransformedVector`,
  batch transform handles) yet; that was deliberately deferred to keep this
  change limited to Stage 1.
- Did not implement new cache-sharing strategies yet (`-128`, `-256`, `-512`
  matrix-level cache options) because this pass is focused on internal affine-form
  plumbing and regression safety.
- Could not combine trace-only dispatch capture with measured benchmark timing in the same command because trace mode still forces Criterion to `$^`; collected trace and timing in separate invocations.
- Initial attempt to add the one-vs-all coordinate comparison rows in
  `benches/mathbench/comparisons.rs` failed from stale patch context and was
  retried successfully after re-reading the current file structure.
- Initial attempt at symbolic closure tracing failed with a tuple-return mismatch in
  `black_box` calls until I added explicit statement terminators (`;`) in those
  trace closures.
- Criterion invocation initially failed with `--filter` passed as an option in
  the bench command; switched to positional regex filters after `--bench` command
  parsing rejected the former form.
- Added `matrix4/hyperreal-rational/mat4 transform vec4 no-translation structural facts` row and initially hit a
  trace-closure typing mismatch (`black_box(tuple)` returning non-`()`), then
  corrected it by binding the tuple result to `_` so the closure still compiles as
  a statement-only trace probe.
- Initial no-translation symbolic timing command accidentally used a loose regex and
  captured additional `direction vec4` rows (one/all-coordinate) in addition to the
  intended labels; reran with anchored regex filters to get the exact set.
