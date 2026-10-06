# HyperGraphics performance and reference audit

This audit covers every source in the README reference section. Criterion timings
were collected from an optimized local build on 2026-07-15 and are comparative,
not portable latency guarantees.

## Retained results

| Path | Baseline | Retained | Result |
|---|---:|---:|---:|
| repeated triangle orientation (2026-07-15) | 279 ns direct | 122 ns caller-prepared | about 56% faster |
| exact 201-line-per-axis grid construction | 133 us | 123 us | 8.2% faster |

The 2026-07-27 immediate-API migration used saved Criterion baselines with 100
samples per path:

| Path | Baseline estimate | Retained estimate | Result |
|---|---:|---:|---:|
| repeated triangle orientation | 68.67 ns direct | 40.80 ns immediate | 40.6% faster |
| clone and query after orientation reuse | 169.73 ns | 78.76 ns | 53.6% faster |
| exact 201-line-per-axis grid construction | 53.379 us | 52.055 us | 2.5% faster |

`ExactMesh::triangle_orientation_against` admits repeated triangles to a
small thread-safe cache owned by the mesh. The first query still takes the
ordinary immediate predicate path; reuse promotes the triangle internally.
Clones share the cache while their vertices remain identical, and mutable
vertex access detaches and invalidates it. The public prepared handle and its
manual lifecycle remain removed.

The stack-wide plane-evidence rename retained the same cache and immediate
query path. A second serialized 100-sample gate measured 40.70 to 39.86 ns for
the repeated query and 79.31 to 78.08 ns for clone-and-query. Criterion measured
the first improvement as significant and classified the second within its
noise threshold.

The 2026-08-02 replacement-aware cache was checked with Criterion's quick mode:

| Path | Estimate |
|---|---:|
| repeated triangle orientation | 44.33–45.62 ns |
| clone and query after orientation reuse | 96.12–99.69 ns |
| four-query collision/promotion sequence | 2.187–2.267 us |

The cache remains a fixed four-slot structure, but a repeatedly used triangle
may now replace a colliding entry. This avoids permanently pinning the first
triangle mapped to a slot. The small synchronization cost on a cache hit is the
tradeoff for safe replacement shared across clones; rebuilding exact plane
evidence dominates the deliberately adversarial collision sequence.

Grid construction computes each exact coordinate once and reuses it for the
vertical and horizontal line with the same coordinate while preserving vertex
order. Its cardinality is unsigned, count arithmetic is checked, and allocation
failure is returned as an error instead of relying on a potentially panicking
reserve.

An optional `dispatch-trace` test uses a rational point only `1/10^12` away from a
plane whose coordinates are of order `10^6`. Classification remains decided with
predicate activity and no approximation or unknown-fact events.

## OpenGL 3.3 Core Specification

The backend retains the specified interleaved `xyz rgb` layout, 24-byte stride,
attribute offsets, column-major matrix upload, signed draw count, and shader version
directives. The audit added checked conversion to OpenGL's signed vertex-count field,
explicit buffer/VAO/program destruction, and cleanup on every partially constructed
shader or object failure path. Viewport dimensions and perspective parameters are
validated before they can create infinities or undefined screen transforms.

## WebGL specifications

Desktop GLSL 330, GLSL ES 300, and GLSL ES 100 remain separate sources selected from
the context's shading-language string. The ES 100 path is accepted only when a vertex
array object extension is advertised, avoiding `glow`'s unsupported WebGL 1 VAO path,
and both vertex attributes are bound to the layout used by the mesh uploader before
linking. WebGL resource handles continue to be used only through the owning
`glow::Context`; consuming destruction makes that ownership boundary explicit. No
sampled render value is promoted to geometric evidence.

## `glow`

The local `glow` 0.16 source confirms that native objects are integer names while the
web backend uses context-managed keys/DOM handles. The wrappers therefore keep their
existing unsafe current-context contract rather than inventing a second context owner.
The manual `Send`/`Sync` declarations were reviewed but not changed: every operation is
unsafe and requires the caller to make the owning context current and serialize access,
matching `glow::Context`'s own cross-thread type contract. Explicit `destroy` methods
address lifetime cleanup without storing or guessing a context in `Drop`.

## nalgebra documentation

Projection matrices remain nalgebra column-major slices, which can be passed directly
to `uniform_matrix_4_f32_slice` with transpose disabled. The camera uses
`Perspective3`, quaternion rotation, and homogeneous translation in their documented
roles. Exact camera parameters are validated both before and after lowering so an
accepted exact value cannot become a degenerate `f64` projection. Unprojection checks
the inverse's finite condition estimate and near-parallel rays, remains explicitly
approximate, and returns `ApproxPoint3` rather than exact geometry. A cache for
`Matrix4::try_inverse` was implemented and benchmarked because nalgebra recommends
specialized/reused projection inverses; in this complete path the atomic cache lookup
increased repeated unprojection time by 21.2% (2.64 to 2.95 us), so the cache was
removed.

## Shewchuk, adaptive robust predicates

The first orientation delegates to HyperLimit's exact/adaptive `orient3`
ladder. Repeated fixed-triangle queries use an internal oriented plane retaining
the certified determinant filter and exact fallback representation. The one
public immediate method maps both routes into the same `TriangleOrientation`;
tests compare automatic reuse with direct answers on both sides and on the
plane, and verify invalidation after vertex mutation.

## Considered but not retained

- Caching the general 4x4 inverse regressed the measured end-to-end unprojection path
  and was fully reverted.
- Exact-to-`f64` exports measured about 50 us for 600 vertices, but the cost is the
  required HyperReal lowering. Bypassing it with primitive-float geometry would break
  the crate's evidence boundary and was not attempted.
- Interleaving vertical and horizontal grid output could avoid the coordinate vector,
  but would change observable vertex order. The retained coordinate cache preserves
  order and still improves construction.
