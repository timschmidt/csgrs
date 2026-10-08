**Hypercurve API and representation review — 12 September 2026**

This document records the initial audit. Its concrete failures describe that
baseline, rather than the current working tree. The ongoing implementation and
verified commits are recorded in [implementation.md](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/implementation.md).
The cap offset, several recursive contact compositions, shared scalar replay,
and topology-policy reuse have since been repaired. General curve and point
values now retain generated evidence, and lossless boundary paths replace the
materialization-only export. Current completion status, remaining API migrations,
and bounded-workload failures are tracked in `implementation.md`; the full
implementation goal remains active.

The best simplification is a small, closed vocabulary of exact geometry, backed by shared algebraic evidence and one arrangement engine. Authored curves and operation-generated curves should use that vocabulary equally. Lines, arcs, conics, Béziers, B-splines, and NURBS can have convenient constructors and specialized execution paths without requiring separate public topology representations.

The present implementation contains much of the necessary mathematics. Its principal difficulty is that mathematical geometry, construction history, parameter representation, proof availability, and predicate policy frequently become different carrier variants or different execution paths. An operation can therefore produce an exact object that the next operation recognizes through a less capable interface. Simplification should remove those accidental boundaries while retaining the evidence and specialized arithmetic that make exact computation practical.

This review uses the stated order **exactness > completeness > performance > memory size > binary size > code size**. Completeness means the required families and their operation results remain usable through arbitrary finite compositions of the supported operations. Preserving existing names, enums, constructors, or intermediate reports is not a completeness requirement.

All callers in this young workspace are controlled. API changes should therefore update callers directly and remove the superseded interfaces in the same change. Do not introduce compatibility shims, forwarding aliases, deprecated interfaces, or parallel legacy entry points. Remove existing compatibility layers encountered during the work and migrate their callers. Staging the implementation means completing coherent changes across the affected crates; each completed stage should have one current interface for its migrated capability. Family constructors, authoring tools, and format conversions remain justified when they provide a distinct mathematical or product capability.

I inspected the current working trees, concentrating on the public surface, representations, region admission, Boolean arrangements, offsets, corner editing, selected algebraic fields, policy caches, downstream consumers, regression tests, and recorded performance qualifications. This is a targeted architectural audit, not a claim to have verified every source line. The source state and existing uncommitted edits are recorded in [source-state.json](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/source-state.json). Hypercurve HEAD was `7f738389b4f2`; the local Hypercurve, Hyperreal, and Hypersolve edits present before this review were included in the builds and left intact.

**The four closures should be explicit representation and operation invariants.**

| Closure | Required invariant | Architectural consequence |
| --- | --- | --- |
| Semantic | An operation has one mathematical interpretation for winding, ownership, coincidence, tangency, singularities, and degeneracies. Region results represent the intended regularized set. | Normalize region topology through one authority. Preserve path traversal semantics separately from region fill semantics. |
| Representational | Every exact result needed by subsequent operations can be stored and passed onward, including generated curves, selected endpoints, and arbitrary finite island nesting. | The general curve and point interfaces must accept retained exact values; conversion to a narrower authoring form is optional. |
| Evidentiary | Selected roots, domains, branches, incidence, signs, overlap maps, and connectivity remain identifiable and reusable. | Retain a shared graph of construction dependencies and certificates. Distinguish structural identity from decided geometric equality. |
| Computational | Composition does not repeatedly reconstruct equivalent expressions, fields, roots, or arrangements, or accumulate avoidable history and allocations. | Compose charts, share supports and fields, retain useful proofs, and evaluate only the expressions a query needs. |

Computational closure cannot mean a uniform bound on the cost of every exact construction sequence. A sequence can create genuinely more boundary components or algebraic numbers of increasing degree. The useful requirement is that representation and computation track the surviving geometry and necessary algebraic dependencies, with avoidable history inflation removed. A fixed degree, field-depth, or iteration cap must not silently become a restriction on representable results.

For open paths, the semantic contract also needs a dimensional distinction. An ordered path carries traversal, parameterization, endpoints, and possibly repeated or overlapping traces. Taking its two-dimensional regularized interior would erase it. Region Booleans regularize two-dimensional filled sets; path edits preserve their specified path semantics, path intersection can report isolated contacts and overlaps, and stroking constructs a filled region. Empty results and singular or collapsed traces need operation-specific meanings.

**A new concrete example exposes both an API-state problem and a closure defect.**

The standalone [probe](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/probe.rs) constructs this degree-five rational Bézier and closes it with a line:

```text
controls: (0,0), (1,1), (2,2), (3,2), (4,1), (5,0)
weights:       1,     2,     3,     4,     5,     7
closing line: (5,0) -> (0,0)
denominator: W(t) = 1 + 5t + t^5, 0 <= t <= 1
```

All inputs are rational and all weights are positive. An independent calculation using Python's exact fractions certifies a strictly positive x-derivative numerator and a nonpositive curvature numerator, strictly negative in the interior. The curve stays above its closing line and has no source singularity. The region is a simple convex cap. Its outward round offset is representable by the existing rational-source analytic parallel, an offset line, and circular joins. The certificates and calculation are in [cap-certificate.txt](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/cap-certificate.txt) and [prove_cap.py](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/prove_cap.py).

The observed behavior is:

| Query on this cap | Result under `STRICT` |
| --- | --- |
| Construct with material role and nonzero fill rule | `Certified` |
| Request signed area | `Decided(None)` |
| Request filled side on the authored representation | `Uncertain(Unsupported)` |
| Offset that representation by `1/10`, round joins | `Blocked(Offset, Line, Unsupported)` |
| Explicitly regularize the region | `Certified`, one boundary loop |
| Request the normalized filled side | `Decided([true])` |
| Request the normalized area | Still `Decided(None)` |
| Offset the normalized result by `1/10`, round joins | `Blocked(Offset, Line, Predicate)` |

The first problem is avoidable coupling between topology admission and measurement. [`compute_filled_side_is_left_with_area_cache`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_region.rs:11198) tries signed areas and area-based nesting. Binary Boolean carrier construction requires these sides, and the offset implementation requests them before constructing its boundary. Unary regularization can already obtain the required topology without the area integral: it builds carriers with `require_filled_sides = false`. See [Boolean construction](/home/tim/Documents/GitHub/workspace/hypercurve/src/curve_region_boolean.rs:1402), [unary construction](/home/tim/Documents/GitHub/workspace/hypercurve/src/curve_region_boolean.rs:1511), and [carrier admission](/home/tim/Documents/GitHub/workspace/hypercurve/src/curve_region_boolean.rs:12768).

The second problem is more specific than an insufficient general algebraic solver. The retained [dispatch trace](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/normalization-trace.txt) localizes the failure to an outward round join. The source contains a concrete contract mismatch: [`exact_offset_spans_from_materialized_curve`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_region.rs:4856) stores `parallel.derivative_at(...)` in `ExactOffsetTangent2::Vector`, while [`append_selected_chord_normal_round_join`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_region.rs:8103) requires that vector to have squared length one and otherwise returns `Predicate` uncertainty. The dispatch into that helper is visible [here](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_region.rs:7643).

For this cap the endpoint squared speeds are exactly `200` and `1250/49`. Endpoint curvature is zero, so the regular parallel has those same endpoint speeds; reversing traversal does not change them. An ordinary derivative is therefore not a unit direction. The source-level mismatch matches the observed blocker. I have not patched the kernel or claimed to isolate every possible later failure after that mismatch is repaired.

This example should be retained as a completion regression. It reaches a successfully normalized exact region, then fails an operation whose result has an existing exact representation. The current failure preserves exactness but fails the requested completeness contract. It also illustrates why extending root isolation indiscriminately would miss some of the most useful simplifications.

**The most important public boundary to remove is the difference between authored and generated exact curves.**

[`CurveGeometry2`](/home/tim/Documents/GitHub/workspace/hypercurve/src/curve.rs:149) contains the eight authored families. Generated analytic parallels, algebraic chords, selected-circle fragments, and selected-fiber cuts live in [`BezierSplitFragment2`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_split.rs:1002). Their parameters have another vocabulary in [`CurveRegionParameter2`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_split.rs:44). This is a genuine representational split even though `CurveRegion2` itself is already unified.

The probe offsets a smooth four-quadratic region successfully under `STRICT`. Its resulting exact analytic boundary cannot pass through `materialized_boundary_paths`: that query returns `Unsupported`. A nonsingular shear of that same region also returns `Unsupported`. The first result is the documented public-carrier limitation; the second follows from requiring a similarity for retained parallels in [`transform_retained_region_fragment`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_region.rs:17562). The general affine case is an extension opportunity, not evidence that an already-supported similarity operation regressed. See [carrier results](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/carriers.txt).

The same split affects open paths. The public path chamfer documents that nonmaterializable algebraic trims block even when retained-region topology can store them. Path fillet reconstruction explicitly asks its contacts and center for represented `Point2` values. The shared corner solver can know a valid exact answer which the public path cannot retain. See [path chamfer contract](/home/tim/Documents/GitHub/workspace/hypercurve/src/curve.rs:1764) and [fillet reconstruction](/home/tim/Documents/GitHub/workspace/hypercurve/src/curve.rs:2013).

These restrictions also reach consumers. Hypergraphics' region line-mesh adapter begins with `materialized_boundary_paths`, and Synaps CAD's linear hull adapter inspects `BezierSplitFragment2::Materialized` directly. Update those callers to the general exact boundary interface when it replaces the current carriers. Remove obsolete exports and any compatibility adapters as part of that migration. See [Hypergraphics](/home/tim/Documents/GitHub/workspace/hypergraphics/src/scene.rs:191) and [Synaps CAD](/home/tim/Documents/GitHub/workspace/synaps-cad/src/compiler/evaluator/booleans.rs:260).

I recommend one public exact curve value that can hold every supported authored and generated curve, together with one path value and one filled-region value. Specialized authoring constructors can return that curve value. A specialized spline definition API may still be justified where knots and control-net editing are themselves user-visible semantics. Multiple public spline carriers whose distinction is extraction or cache state are much less useful: the current lower spline carriers and their shared wrappers show that opportunity. See [spline carriers](/home/tim/Documents/GitHub/workspace/hypercurve/src/bspline.rs:21), [polynomial spline wrapper](/home/tim/Documents/GitHub/workspace/hypercurve/src/polynomial_spline.rs:1), and [NURBS wrapper](/home/tim/Documents/GitHub/workspace/hypercurve/src/nurbs.rs:26).

The following is a proposed conceptual representation, not an implemented API:

```text
Curve2         = shared support + local domain + parameter chart + traversal
Point2         = represented coordinates or a projection from shared exact evidence
CurveLocation2 = support/chart identity + selected parameter or endpoint identity
CurvePath2     = ordered Curve2 values with certified connectivity
CurveRegion2   = normalized oriented boundary graph over the same Curve2 values
```

`Point2::new(Real, Real)` can remain a convenient constructor. Obtaining two directly represented `Real` coordinates from a general point is a narrower query, with a name such as `represented_coordinates`. It must not determine whether the point itself is exact. Likewise, a selected algebraic parameter is exact even when `as_exact()` currently returns `None`. Names such as `is_exact` should not mean “already materialized in this particular scalar carrier.” This extends the exact-point versus rational-payload cleanup consistently; rational arithmetic and reconstruction theorems must still retain their explicit rational guards.

A general boundary iterator should return exact curves directly. Producing an authored spline, an ordinary rational Bézier control net, or an SVG-native command can remain an optional conversion. Lossless exchange of generated geometry needs the support, charts, selected roots, branch constraints, and dependency references; lazy caches and process-local identities are not interchange data.

**A small geometry grammar can cover much more than a small list of authoring types.**

For the requested Boolean, offset, circular fillet, and chord-setback chamfer operations, the central geometric supports are rational curve spans, their signed normal offsets, lines, and circles over retained exact coefficient constructions. Splines are piecewise rational spans with knot-domain charts. Polynomial curves are unit-weight rational spans mathematically, while compact polynomial kernels can remain private optimizations. Lines and circles deserve similarly cheap internal paths.

| Authored family | Shared exact representation | Useful specialization to preserve |
| --- | --- | --- |
| Line segments | Degree-one rational span or line support with a domain | Direct orientation, intersection, and distance predicates. |
| Circular arcs | Circle support or rational quadratic charts | Center/radius identities, angular ordering, and exact circle incidence. |
| General conic arcs | Rational quadratic charts over exact coefficients | Conic classification and degree-two predicates where cheaper. |
| Polynomial or rational Béziers | Homogeneous polynomial spans with certified denominator domains | Compact low-degree and unit-weight evaluation. |
| Polynomial B-splines | Shared spline definition with exact knot-domain charts and lazy span extraction | Knot locality, continuity metadata, and control-net editing. |
| NURBS | The same spline mechanism with homogeneous weighted data | Shared weights, extracted spans, and projective evaluation. |

These are mathematical representations, not requirements to allocate homogeneous control nets for every line or extract every spline span eagerly. Conics may require several charts, and denominator zeros, removable singularities, and domain endpoints need explicit treatment. A weight-sign restriction or a single finite chart must not accidentally redefine the desired curve family. Degenerate conics reduce to their appropriate components; region regularization and open-path semantics determine which components survive. The contract should explicitly state whether unbounded components and infinite endpoints are included, rather than confusing a support extending to infinity with an admissible finite path segment.

The closure argument is useful because it identifies the irreducible machinery. Let a rational source be `P(t) = (X(t)/W(t), Y(t)/W(t))`, and let

```text
vx = X'W - XW'
vy = Y'W - YW'
s^2 = vx^2 + vy^2, with s > 0
Q(t) = P(t) + d * (-vy, vx) / s
```

On a pole-free regular domain, this is an exact algebraic construction over the source coefficients and distance. A general offset should remain such a selected construction; it need not be rationally parameterizable. The implicit offset equations can include both normal sheets, making the positive-speed and signed-distance constraints essential. Farouki and Neff describe algebraic offset equations, their singularities, and the role of root isolation in trimming. [Algebraic properties of plane offset curves](https://research.ibm.com/publications/algebraic-properties-of-plane-offset-curves).

A Boolean cuts existing supports at selected contacts and overlaps. A chord-setback chamfer selects endpoints from distance equations and constructs a line between them. A circular fillet selects a center from the appropriate signed-offset incidences, retains the contact parameters, and inserts a circle arc. Subsequent constructions require further selected algebraic extensions of the coefficient domain. They do not intrinsically require another public point or curve type named after the exact sequence that produced it.

This is an architectural argument, not a proof that the current fallback algorithms decide every such system. A complete implementation still needs positive-dimensional component handling, pole exclusion, singular branch decomposition, multiplicities, and selected-sheet replay. A general callable `point_at(t)` with numerical bounds is insufficient: arrangement requires evidence about zero sets and local branches, not merely an evaluation oracle.

General affine images of analytic parallels require an additional exact mapped-support form or a more general lifted algebraic relation. A shear does not commute with Euclidean normal offset. Therefore `AffineImage(Parallel(P,d))` must not be rewritten as `Parallel(AffineImage(P),d)`. Similarity transforms have the familiar scale and orientation rules; nonsimilar transforms require their own mathematics.

The scalar decision domain also needs an honest contract. The geometry can be algebraic over exact authored constants without those constants being rational or algebraic over the rationals. General `Real` expressions involving transcendental functions do not automatically provide a complete zero-decision procedure. Increasing numerical precision alone cannot establish every exact equality. This distinction is fundamental to exact geometric computation. [Yap, Is It Really Zero?](https://cs.nyu.edu/exact/doc/isItZero.pdf).

Preserve every supported exact scalar representation. State the domain for which total algebraic decision procedures are available, and distinguish a base-scalar identity that remains unresolved from a missing geometric case, a missing replay route, and a resource limit. None of those distinctions excuses the cap example: its source and design data are rational, and its immediate failure is a tangent-contract mismatch.

**`CurveRegion2` should have one filled-set invariant, including arbitrary finite nesting.**

The current [`CurveRegionData2`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_region.rs:163) combines boundary loops, optional roles, optional fill rules, signed loop composition, regularized-topology flags, connectivity flags, filled-side caches, native views, and measurement caches. These do not all describe the same semantic state. An independently authored collection of signed or self-crossing loops is different from the normalized boundary of a regularized filled set.

Use a builder or input description for authored winding and signed compositions. Its evaluation constructs the one region type. The normalized region should retain oriented boundary incidence and filled-side ownership, while source roles, original winding multiplicities, and authoring provenance remain available where useful. They should not be repeatedly rediscovered to decide how to operate on a completed result.

An island inside a hole inside another island needs no new curve carrier or fixed-depth special case. The arrangement labels faces; the surviving boundary separates filled from empty faces. A containment hierarchy can be derived for interfaces that need nested components. The authoritative structure must also handle point-touching components and higher-valence boundary vertices, where a simple nesting tree and the assumption that every vertex has degree two are insufficient.

This is consistent with the arrangement-based model used by CGAL's general polygon sets: geometric traits provide curves and points, while a DCEL stores the set topology. The relevant precedent is the separation of geometry from ownership, not adoption of CGAL's public API or coefficient restrictions. [General_polygon_set_2](https://doc.cgal.org/latest/Boolean_set_operations_2/classCGAL_1_1General__polygon__set__2.html). Regularized region operations deliberately eliminate isolated lower-dimensional set features while retaining the boundaries of the resulting filled set. [CGAL Boolean set-operation semantics](https://doc.cgal.org/latest/Boolean_set_operations_2/index.html).

Hypercurve already has face-winding propagation, shared split topology, overlap correspondence, certified successors, and an authoritative unary arrangement. The recommendation is to make those existing results the normal region invariant and consolidate their use. It does not require every operation to retain a complete interior arrangement permanently. The surviving boundary graph can remain compact, with optional derived indices for nesting, location, or repeated queries.

Measurement must be downstream of this invariant. Filled side, orientation, ownership, and nesting should use topology and certified local predicates. An unavailable closed-form area or length representation must not block a Boolean or offset whose boundary is representable. Exact measurements and certified numerical measurement adapters can evolve independently. The cap's successful area-free normalization demonstrates that this is a practical refactoring direction in the current code.

**Retain mathematical distinctions that prevent errors; hide distinctions that merely describe construction history.**

The offset failure is a reason to distinguish an actual derivative, an oriented tangent direction, and a certified unit direction. Most ordering and incidence predicates need only a tangent direction and should avoid normalization. A metric construction can request a unit normal and retain the positive norm and branch certificate once. Passing a raw `(Real, Real)` between those roles makes an essential precondition implicit and encourages repeated attempts to rediscover it from scalar expressions.

Conversely, [`RationalBezierIntersectionPointEvidence2`](/home/tim/Documents/GitHub/workspace/hypercurve/src/rational_bezier_general.rs:158) now includes chord-pair points, circle/chord-derived points, chord parallels, analytic-parallel points, and similarity images. These are exact point representations, many unrelated to a rational Bézier intersection. The public name and exposed variants make history look like mathematical capability. One exact point interface with a private projection representation would be more accurate and easier to extend.

The same separation applies to locations. A location needs the support and chart in which it is ordered, its selected root or endpoint identity, and the evidence required to compare or transport it. It should not require a separate public “region parameter” solely because its root is represented in a selected fiber. Ordering locations on unrelated supports is generally not a meaningful scalar operation; the API should expose that domain explicitly.

Path/region corner selection should use boundary-location or vertex handles valid for a particular result, rather than requiring consumers to infer a meaningful corner from `(loop_index, vertex_index)` after spline decomposition and arrangement splitting. Such handles need an explicit validity and correspondence contract across edits. Arbitrary decomposition boundaries are not automatically authored corners or geometric corners.

**General algebraic machinery belongs at the shared algebraic layer, with geometry-specific equations above it.**

The current offset module contains generic-looking selected-fiber scalar machinery, recursive quadratic fields, projective scalars, field embeddings, and polynomial arithmetic alongside circle/chord geometry and approximation algorithms. Relevant starting points are the [selected-fiber authority](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_offset.rs:4079), [recursive field](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_offset.rs:6366), and [projective scalar](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_offset.rs:6430). Hypersolve already supplies [fiber replay](/home/tim/Documents/GitHub/workspace/hypersolve/src/algebraic_fiber.rs:1) and an [ordered-field polynomial interface](/home/tim/Documents/GitHub/workspace/hypersolve/src/ordered_field_roots.rs:1).

| Layer | Responsibility in the simplified design |
| --- | --- |
| Hyperreal | Exact scalar meaning and arithmetic, scalar structural facts, and certified refinement of its representations. |
| Hyperlattice | Geometric structure and reusable scheduling facts for fixed geometric data. |
| Hyperlimit | Certified geometric decisions, certainty, and unresolved predicate outcomes. |
| Hypersolve | Selected roots and fibers, exact algebraic replay, field extensions and embeddings, polynomial signs and root certificates. |
| Hypercurve | Supports and parameter charts, curve-specific incidence equations, local branch geometry, path connectivity, arrangements, and region ownership. |

The useful extraction is one reusable selected-algebraic authority supporting general finite algebraic extensions as well as cheap quadratic and rational cases. Moving only square-root recursion would leave higher-degree selected fibers as another competing mechanism. The authority should share selected generators and preserve correlations between coordinates. It should permit projective expressions with certified denominator signs and evaluate predicates in the smallest sufficient existing field.

Do not convert every coordinate to an independent minimal polynomial or merge every endpoint into a global primitive element. Those operations can multiply degrees and discard inexpensive shared identities. Do not move source-curve incidence semantics into Hypersolve merely because the equations are polynomial. Extract the reusable arithmetic and replay; keep geometric construction and branch ownership in Hypercurve.

An extraction can start internally and preserve the existing specialized layouts. It need not enlarge `Real`, place an algebraic solver dependency below its own scalar substrate, introduce a public tower of generic type parameters, or replace every hot path with dynamic dispatch.

**Evidentiary closure should be carried by a small set of reusable contracts.**

For a point contact, retain the shared point identity, both support locations, the selected branch, and certified local crossing or tangent information. For an overlap, retain an actual parameter correspondence with orientation, domains, and multiplicity. Two unrelated parameter intervals are not sufficient evidence of an overlap. For a root, retain the defining relation, selected fiber or field, isolation/domain convention, and uniqueness or component evidence. For a normalized boundary edge, retain its endpoint incidence and filled-side ownership.

These obligations are more stable than the algorithm that first proved them. Their implementations can have several fast paths behind one private interface. Selected-sheet conditions, source-pole exclusions, endpoint ownership, and exact overlap transport must survive that consolidation. Squaring equations or taking a resultant creates candidates; it does not itself certify the authored geometric branch.

Cache ownership should follow the existing stack retention rule: intrinsic scalar facts stay with scalars; fixed-input preparation belongs with reusable evidence; query results and traces are reports; larger aggregate caches require demonstrated reuse. [Stack retained-fact audit](/home/tim/Documents/GitHub/workspace/hyperlimit/STACK_RETAINED_FACT_AUDIT.md:31). Construction evidence which is part of an output's meaning must survive with that output, even when the operation's full report is discarded.

The two policy cache types are not redundant just because both use `OnceLock`. [`PolicyClassificationCache`](/home/tim/Documents/GitHub/workspace/hypercurve/src/policy.rs:564) allows independent certified evidence to supersede approximate evidence; [`PolicyEvaluationCache`](/home/tim/Documents/GitHub/workspace/hypercurve/src/policy.rs:600) has a single retained result. Consolidation requires preserving those differing upgrade contracts. A cached lack of separation is not a permanent proof that separation is impossible.

**An exact topology API can also be substantially smaller than the current policy/result vocabulary.**

The present surface mixes `CurveResult`, `ExactCurveResult`, `Classification`, `CurveOutcome`, optional materializations, retained statuses, and operation-specific reports. Some combinations are warranted, but a nested `Result<CurveOutcome<Classification<Option<T>>>>` asks the user to understand implementation stages before obtaining an ordinary geometric result.

For the principal exact API I recommend successful return values with exact semantic and evidentiary contracts, plus one contextual failure vocabulary. An execution context can carry caches, cancellation, and resource budgets without changing the meaning of a topology predicate. Distinguish invalid input, undefined construction or no solution, missing capability, unresolved base-scalar predicate, incomplete algebraic replay, and resource exhaustion. Optional materialization is a separate query, not a second kind of geometric exactness. Diagnostic or resumable reports can remain available without being mandatory intermediate steps.

The cap currently reports its failing family as `Line` even though the relevant problem spans a rational curve and a retained chord. A useful blocker would identify the operation, participating supports or locations, unmet predicate/precondition, and attempted authority. Rich diagnostics do not require a separate public report type for every internal stage.

There is also a subtle certainty distinction worth making impossible to overlook. The [policy probe](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/policy.txt) constructs a loop using `APPROXIMATE_512`, discards the wrapper, and unions the region with empty under `STRICT`. The identity union returns `Certified`, while a strict boundary materialization still cannot certify the original seam. This is consistent with the documented operation-scoped meaning of `CurveOutcome`: the identity operation consumed no approximate predicate. It is not a certificate that all retained input topology has been independently certified. See [outcome aggregation](/home/tim/Documents/GitHub/workspace/hypercurve/src/policy.rs:772) and [trivial Boolean returns](/home/tim/Documents/GitHub/workspace/hypercurve/src/curve_region_boolean.rs:12676).

Given the requested exactness priority, the simplest principal topology surface would admit only exact/certified region states. If approximate topology is retained as a feature, make it a separate provisional value or explicitly qualified wrapper whose promotion requires certification. Rendering approximation can remain an ordinary output adapter. Thread-local certainty aggregation should not be the only carrier of a value's evidentiary obligations; an explicit internal query context will also compose more naturally with future parallel evaluation.

**Computational closure requires a normal form for construction dependencies, not universal eager algebraic normalization.**

| Repeated construction | Preferred representation rule | Condition that must remain explicit |
| --- | --- | --- |
| Repeated trim/split | One shared support with composed domain/chart restrictions | Preserve source parameter mapping, endpoint selection, and traversal. |
| Reversal | Compose traversal/chart orientation without copying control nets unnecessarily | Parameter derivatives change sign according to order; local branch direction must remain correct. |
| Repeated affine or similarity transport | Compose the maps and retain the original support/evidence | Transport orientation, metric, and branch conditions correctly. |
| Repeated parallel on a regular cell | Share the original source and combine signed distances | The derivative-scale sign and traversal determine the sign of composition; split at singularities. |
| Repeated point/tangent queries | Reuse selected roots, shared fields, and requested projections | Avoid constructing unused coordinates, higher derivatives, or global norms. |
| Repeated region operations | Reuse immutable source and boundary facts; build one arrangement for the requested group of results | Do not retain unlimited obsolete pair reports or treat old adjacency as proof after reconnection. |
| Repeated algebraic extensions | Share parents, eliminate unused dependencies, reuse existing generators and embeddings | New genuine algebraic dependencies can still increase necessary complexity. |

The offset-distance rule is already implemented for analytic fragments with a certified derivative-scale sign in [`exact_offset_span_from_analytic_parallel`](/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_region.rs:5201), and a fresh regression run confirmed repeated offsets on a regular smooth fixture. It must not be generalized into an unconditional region identity `offset(offset(R,a),b) = offset(R,a+b)`: intervening regularization, erosion collapse, changed joins, and traversal changes matter.

The same principle applies to apparent equality. Shared support or contact identity is a cheap positive certificate. Different storage does not prove different geometry. A new common representation should not force expensive equality on every clone or lookup, nor turn a failed structural comparison into a geometric inequality. The [Point2 implementation](/home/tim/Documents/GitHub/workspace/hypercurve/src/point.rs:24) illustrates the existing distinction between shared identity and what its scalar zero-status query can establish.

Numerical scheduling also needs to remain demand-driven. A containment predicate often needs signs and certified bounds, not exact coordinate materialization. Tangent ordering often needs no unit vector and usually needs no higher derivatives when connectivity already gives a unique successor. Bounding-box uncertainty should retain a candidate. A bounded exact filter should escalate to its complete authority rather than make its depth limit a new family restriction. Hypercurve already follows that pattern for resultant degree bounds: the [continuation](/home/tim/Documents/GitHub/workspace/hypercurve/src/rational_bezier_general.rs:1418) retries the complete degree setting.

**The recorded performance evidence supports consolidation, with important qualifications.**

The September 8 selected-radial change reused a recursive field for affine signs. Its documented affine workload fell from `63.65 s` to `3.14 s`, and allocation count from about `659 million` to `24 million`. But the tangent workload rose from `0.33 s` to `0.57 s`, with higher allocation traffic and instruction count. These are recorded matched workload measurements, not fresh timings from this review and not a uniform speedup. [Recorded qualification](/home/tim/Documents/GitHub/workspace/hypercurve/PERFORMANCE.md:8926).

That is strong evidence for sharing field meaning while specializing what each query evaluates. It is weak evidence for routing every query through the same heavyweight construction. A smaller implementation that makes simple tangent work build an unnecessary field is not automatically a better implementation under the stated priorities.

The working-tree inventory contains 337,345 Rust source lines across 63 files, including comments, embedded tests, and generated data. `bezier_offset.rs` has 168,554 lines; `bezier_region.rs` 34,361; `curve_region_boolean.rs` 22,110. A source-text scan found 327 `pub` type declarations and 1,584 `pub` function declarations, including declarations inside private modules; these are not exact counts of externally reachable API items. The three large files account for about two-thirds of raw source lines. These figures locate concentrated responsibilities; they are not a reason to remove necessary algorithms or a prediction of binary-size savings.

Split those files by mathematical responsibility as extraction proceeds: support construction, selected algebraic values, incidence, overlap, parameter transport, local topology, and approximation. A file split alone does not reduce machinery. Likewise, hiding public types alone does not reduce runtime work. The useful deletions come when several history-specific implementations can delegate to one correctly specified operation.

Note that the workspace includes a rust function call graph utility which may be of use during refactoring.

**Completion should be judged with composed workloads, not only a family-pair matrix.**

The existing corpus is substantial. It includes authored-family Boolean matrices, selected-circle fillets, algebraic chamfers, repeated offsets, Boolean-to-offset reentry, policy replay, high-degree elevations, and recursive generated geometry. Those are valuable assets to preserve. This review freshly ran six existing promotion regressions successfully, including non-PH Bézier-pair fillets and cusp/chord Boolean results reentering bevel offsets.

A source-only reading also suggested that the older second/third-order tangent helpers might bound general Boolean completeness. I tested that hypothesis using two lenses bounded by `y = k*x^n`, for `n = 4` and `n = 8`, touching only at the origin. All four Booleans returned the expected loop counts and passed five exact membership checks per result against the defining equations. Thus this review does **not** claim a third-order limit in the public Boolean engine. The low-level helper's limits are not the limits of every public dispatch path. [Fourth-order results](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/tangent4.txt), [eighth-order results](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/tangent8.txt).

The additional validation target should be a stateful generator over exact objects and operation sequences. Reuse generated outputs as subsequent operands and grow both the boundary topology and coefficient dependencies. Include open-path edits and their region/stroke uses; all Boolean operators; inward/outward offsets and all join styles; fillet and chord-setback chamfer choices; selected-root trims; reversal and supported transforms; several levels of islands and holes; and independent fields meeting after different histories.

The generator must exercise transverse crossings, high-order tangencies, singular endpoints, partial/reversed overlaps, coincident components, rational poles at domain boundaries, zero-length or collapsed features, and multiple valid corner solutions. Degree and field depth should be varied independently. No-solution outcomes need independent geometric reasons; a missing replay route must not masquerade as no solution.

Required exact cases should assert completion. For example, the current [libFuzzer region target](/home/tim/Documents/GitHub/workspace/hypercurve/fuzz/fuzz_targets/region_boolean.rs:84) conditionally checks correctness only when a Boolean returns `Ok`; it can miss a completeness regression even on its rectangle inputs. That observation does not negate the stricter integration/property corpora, but it is an inexpensive gate to strengthen.

Use independent exact oracles where possible: analytic membership for designed families, exact rational fixtures, known topology counts and incidences, and certificates such as the cap proof. Metamorphic identities are useful supplements, but two results produced by the same flawed kernel can agree. Record completion, topology validity, and evidence reuse separately from speed. A timeout is not a mathematical disproof; for a qualified complete domain, however, recurring resource exhaustion still matters to the computational contract.

For computational closure, measure the number of retained supports and selected generators, chart/composition depth, coefficient sizes, repeated root refinements, coordinate reconstructions, allocations and bytes allocated, and peak live memory along increasing sequence lengths. Compare both first use and reuse. The critical performance metric is the end-to-end composed workload with the same guarantees; smaller structs, fewer lines, or one fast scalar microbenchmark are secondary evidence.

**I would implement the simplification in this order.**

1. Establish the normalized-region and path semantics, retain the new cap regression, and repair the derivative/direction/unit-normal contract. Separate contextual missing-capability and predicate-precondition failures. Make area-free region admission the normal path where existing topology can provide the proof.
2. Replace the public curve, point, and location interfaces with values that carry every existing exact internal result. Update all affected workspace callers, including open paths and region consumers, and delete the superseded interfaces in the same changes. Replace mandatory materialization with explicit optional conversions where a consumer actually needs them, and keep arbitrary finite nesting in the region boundary topology.
3. Consolidate selected algebraic values, fibers, embeddings, and predicate replay through Hypersolve while preserving native rational, polynomial, line, and circle execution paths. Complete each vertical slice—construction, split, query, Boolean, edit, and reentry—with its callers migrated and its redundant history-specific routes removed.
4. Consolidate region admission, unary regularization, Boolean ownership, and offset/edit reconstruction around the same arrangement contracts. Retain the existing batched Boolean work-sharing and use common multioperand selection where it can eliminate intermediate arrangements. Preserve all overlap and degenerate-contact obligations.
5. Enforce computational normalization rules and expand composition-based qualification. Audit the completed migrations for remaining redundant carriers, reports, aliases, and adapters; remove them and update any remaining callers. Split remaining large modules by responsibility and measure performance, memory, linked binary size, and source reduction in the stated priority order.

The migration should preserve every mathematical regression even when the old test asserted an enum layout or construction path. Rewrite such tests around the exact geometry and evidence obligation, with targeted assertions for deliberately retained fast paths. Do not retain an obsolete public carrier merely to keep a representation-sensitive assertion compiling.

A migration step is complete when its production callers, tests, examples, benchmarks, fuzz targets, and documentation use the replacement; its superseded exports and compatibility layers are removed; and the affected dependent crates pass appropriate verification. Caller migration and interface removal are part of the implementation step, with no separate compatibility phase.

The intended result is a small public API whose values are closed under the required operations, a compact internal grammar that separates geometry from its proof representation, and shared machinery that evaluates only the evidence needed by each query. The cap example is a particularly useful first task because it tests semantic typing, normalized-state admission, evidentiary reuse, and actual operation closure with entirely rational authored data.

**Validation and reproduction.** The kernel was not modified by this review. Six selected existing release integration tests passed; standalone probes reproduced the carrier, area-admission, normalized-offset, and operation-certainty observations; the degree-four and degree-eight Boolean probes checked 40 exact membership results in addition to loop counts; and the independent cap certificates passed. Full-suite, fuzz-campaign, release-readiness, and new performance claims are outside this review's validation scope. The historical performance qualification is cited as historical evidence.

The probe package is next to Hypercurve and uses sibling path dependencies. Its lockfile was seeded from Hypercurve's lockfile so the final recorded Cargo runs retain the matching numeric dependency versions. Reproduce the principal failure with:

```bash
cd /home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12
cargo run --offline --release --features dispatch-trace --target-dir ../hypercurve/target -- normalization offset
python3 prove_cap.py
```

Other probe arguments are `orientation`, `carriers`, `policy`, `tangent 4`, and `tangent 8`. The exact existing-test selections and their observed results are recorded in [validation.txt](/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12/validation.txt). These artifacts make the proposed changes reviewable without requiring a production rewrite as part of the architectural examination.
