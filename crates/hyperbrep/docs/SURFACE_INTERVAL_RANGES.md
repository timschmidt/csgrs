# Native surface interval ranges

Enable `hyperbrep/interval-ranges` to use
`interval_ranges::prepare_rational_bezier_surface_ranges`. The constructor
retains the immutable native `Surface` and prepares one shared coordinate and
Jacobian program. Each `enclose` call consumes a `BoxN<DyadicInterval, 2>`, an
explicit precision, an `ExpressionBudget` and a `DyadicBudget`. The runnable
[surface example](../examples/surface_ranges.rs) evaluates sixteen adjoining
UV cells and consumes their evidence through native spatial exclusion.

## Domain and authority

The current adapter accepts admitted positive-weight native rational Bézier
tensor surfaces. Both control-net dimensions may be arbitrary finite sizes,
subject to declared limits. Native exact coefficients remain authoritative;
generated algebraic subdivisions and native affine transforms retain their
definitions. NURBS, implicit and analytic-family adapters remain implementation
gaps and return `UnsupportedSurface`; this does not narrow the general algebraic
target. Existing native evaluators and default features are unchanged.

The native tensor Bernstein conversion is reused. Hypersolve compiles its
existing bivariate coefficient grids with ordered bindings and one shared
original denominator, then differentiates and certifies the resulting program.
No fitted surface, tessellation or rounded control net replaces the source.
The source's homogeneous-control cache is not populated during preparation:
cold sources use a temporary native carrier, and an existing cache is borrowed.
Shared scalar facts may warm during these exact calculations.

Queries accept a canonical empty Cartesian set or a finite dyadic box contained
in the native UV chart. Native strict comparisons validate chart membership.
The returned `SurfaceRangeEnclosure` retains the source, exact parameter box,
three coordinate enclosures, six partial enclosures and original regularity
evidence. `partials()[0]` is the u derivative and `partials()[1]` the v derivative;
each vector is in x,y,z order. Precision chooses an enclosure grid, not a bound
on the final width. Repeated queries can reuse the immutable prepared program
with separate caller-owned budgets, including concurrent queries.

## What the certificate establishes

Successful C1 certification encloses the coordinates and both local partials
throughout the retained parameter set. It does not imply full rank, an injective
chart, trimmed-face membership or valid face/solid topology. Constant and
rank-one patches can therefore have valid C1 range evidence while native normal
construction reports a singular parameter.

`native_bounds` converts outward endpoints exactly to the existing `Aabb` and
returns a wrapper borrowing the range evidence. Empty parameter sets have no
finite box. The native Hyperlimit `ordered_aabb3s_intersect` predicate can use
non-overlap as an exact exclusion; overlap alone does not certify incidence.
The example and corpus consume the certainty receipt rather than a guessed
classification. Touching candidates and exact 2^-80 gaps remain distinct.

The original denominator guard is retained. An interval containing zero refuses
certification and does not prove a pole: dependency overestimation can occur
even with positive native weights. Smaller subboxes can succeed on retry.
Unsupported families, outside-domain inputs, scalar/predicate failures, graph
exhaustion, interval exhaustion and cancellation retain typed diagnostics and
their original error sources. Failed attempts publish no partial success.

## Resources and lifetimes

For U columns, V rows and N=U*V, preparation checks 64*N+2 structural slots and
reserves 32*N*(U*U+V*V+1) conversion units before native conversion. Graph
emission, validation, differentiation, evidence retention and native projection
also charge their supplied budgets. Interval precision/size/work are accounted
separately. Workers must receive explicit allowances; no implicit shared worker
budget is provided.

These are structural reservations, not total CPU or memory bounds. Scalar
refinement, coefficient growth, payload cloning/destruction and total heap
remain exclusions. Cancellation is checked around native conversion and before
publication; it cannot interrupt an ongoing native conversion or scalar call.
An exhausted or cancelled attempt does not become negative geometric evidence
and does not poison a larger-budget retry.

Native caches and shared source identity are checked independently of scalar
cache warming. Complete trace snapshots cover cold/warm native caches, success,
post-conversion exhaustion and pre-cancellation under both policies. Preparation
changes one scalar first-observation count in the maintained cold lifecycle;
no claim of invisible scalar-cache warming is made. Cold allocation observations
retain storage after output/source destruction, and repeated fresh-source
queries reach a plateau in the maintained corpus; see the
[measured resource record](../PERFORMANCE.md#native-surface-interval-ranges).

## Evidence and remaining qualification

Focused tests cover asymmetric/ragged coefficient grids, arbitrary-degree
examples, algebraic coefficients and subdivisions, independent exact derivative
expectations, nonuniform weights, denominator overestimation, empty/outside
domains, small budgets, cancellation/retry, precision changes, concurrent
sharing, affine transport, reconstruction and fresh-process native archive
replay. Native bytes and mathematical expectations are checked separately.

An additional scalar oracle exposed a bounded-recognizer limitation: comparing
one fully expanded algebraic partial against its closed form can remain
undecided. The range certificate succeeds; an exact per-node quadratic-field
oracle checks all nine outputs without modifying production normalization or
claiming general scalar-decision closure.

The [completed workload](../benches/surface_ranges.rs) compares independent and
shared preparation with exact analytic extrema, all output evidence, native
exclusion and destruction. Expanded acceptance records 52 focused passes. Full Rust 1.95 release owner
and downstream runs report 2399 and 127 passes across
147 and 28 harness summaries, respectively, with no failures.
These are reported test/doctest results, not a unique coverage denominator.
Strict all-feature/all-target Clippy, strict documentation, the actual example,
disabled-feature checks, WASM library compilation and stable all-target checks
also pass on the fixed implementation revision.

Nine archives are verified. An external consumer builds, runs the unchanged
example and passes strict Clippy using only extracted Hyper archives. The same
archived example completes three sixteen-cell portfolios in WebAssembly with
zero host imports and the default linker stack. This debug Rust test export does
not qualify a public JavaScript binding. A metadata-only followup enables all
features for docs.rs; strict packaged documentation and the rendered API pages
are verified locally. Hosted docs.rs execution is not claimed.

The [existing implementation ledger](IMPLEMENTATION_STATUS.json) and
[interval refactor progress](../../../INTERVAL_REFACTOR_PROGRESS.md) identify
source-specific evidence. Full pre-push workflows, stable runtime suites,
hosted platforms, public bindings and universal algebraic closure remain open.
Local archive qualification is not a published release.
