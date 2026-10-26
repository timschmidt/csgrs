# HyperBRep contributor requirements

Follow the repository [agent rules](../../AGENTS.md),
[properties](../../properties.md), and [admission protocol](../../docs/admission.md).
This file specializes those requirements for HyperBRep and affected consumers.
Use the existing [development plan](docs/DEVELOPMENT_PLAN.md) and the focused
[curve and surface closure plan](docs/CURVE_SURFACE_CLOSURE_PLAN.md) as execution
references. Maintain one linked capability and implementation record; do not
create competing project status ledgers.

These are target requirements, not assertions that every capability is
implemented. Reproduce findings on the revision being changed. Keep current
blockers, measurements, and completed increments in the support matrix and
execution ledger, rather than weakening this contract or embedding a transient
implementation inventory here.

**Generated exact geometry is first-class geometry. An operation is not complete
merely because its output can be evaluated or drawn. Every applicable subsequent
operation must consume the result without changing its meaning, losing required
evidence, or escaping its declared resource contract.**

## 1. Ownership and change discipline

Keep the production kernel and interchange implementation pure Rust and usable
on native platforms and WebAssembly. External kernels and computer algebra
systems may provide independent test oracles; they must not become undeclared
production fallbacks. Preserve a serial implementation with the same exact
semantics when adding optional parallel execution.

Hyperreal owns authored scalars; Hypersolve owns algebraic solving and selected
fields; Hyperlimit owns predicate policy; Hypercurve owns planar curve geometry
and arrangements. HyperBRep owns spatial geometry, chart transport, topology,
solid admission, and modeling composition. HyperBRep IO owns format syntax,
mapping, and exchange policy. Fix each invariant in its lowest sufficient layer.
Adapters must not duplicate a geometry kernel or reconstruct discarded proofs.

Follow the existing greenfield direction: update callers directly when public
APIs or archives change, and document intentional migrations. Do not introduce
parallel legacy interfaces or compatibility shims by default. Permission to
change a format does not authorize losing an existing exact capability, source
identity, or regression fixture. Apply all repository validation and pre-push
rules; this file does not replace them.

## 2. Required mathematical domain and operations

The general algebraic target comprises finite descriptions of real algebraic
points, curve pieces, surface pieces, and bounded regularized semialgebraic
volume sets, with rational or real-algebraic geometric data. Permit arbitrary
finite degree and algebraic coefficient fields in the mathematical model;
rational coordinates, radicals, or low degree are not universal prerequisites.
Publish implemented subdomains and resource limits separately. Preserve existing
supported symbolic inputs. Broader analytic or transcendental inputs require
explicit representation and decision contracts; no universal decision procedure
for arbitrary computable real expressions is implied.

The core operation target includes regularized Boolean operations; exact
section, split, imprint, restriction, reversal, and reconstruction;
constant-distance Euclidean surface and solid offsets; shelling and sheet
thickening with stated opening and join laws; constant-radius rolling-ball
fillets; and chamfers with explicit distance, angle, rail, and correspondence
semantics. Include finite compositions, supported transforms, and native exact
save/reload between operations. Existing extrusion, revolution, sweep, and loft
workflows retain their documented domains; each broader construction law must
earn its own closure and admission evidence.

Valid inputs can have algebraic endpoints, mixed families, holes, cavities,
disconnected components, and canonical empty results. Constructions need not
exist or be unique. Distinguish proved absence of solutions, multiple solutions,
and underconstrained continuous families from invalid input or algorithmic
incompleteness. Never select an arbitrary numerical sample as an exact answer.

## 3. Minimum exact and practical representations

Distinguish mathematical families, source definitions, parameter charts, storage
variants, and topology. There is no uniquely smallest enum or universally fastest
representation. Minimize duplicated authority and avoidable growth.

A sufficient general exact foundation is selected algebraic systems and
semialgebraic cells, with exact coefficient fields, branch selectors, domains,
orientation, and incidence. A polynomial equation alone does not select a
surface patch or curve branch. A spatial curve can require several equations
and branch data; a solid additionally requires material membership and boundary
topology. Exact points may be jointly selected solution fibers whose individual
coordinates have no convenient scalar view.

Maintain these practical functional roles, without requiring one public type
per row:

| Role | Preserved structure and purpose |
| --- | --- |
| Native analytic curves and surfaces | Cheap exact predicates and constructions for lines, conics, planes, quadrics, tori, and applicable specializations |
| Rational curve and tensor-surface spans | Exact homogeneous controls, knot charts, denominator guards, and useful polynomial or low-degree structure |
| Retained source and procedural definitions | Compact transformed geometry, extrusion/revolution definitions, surface parallels, and admitted blend or envelope relations |
| Selected algebraic curves and surface patches | Nonrational intersections, offsets, and generated branches without mandatory rational conversion |
| Correlated points, restrictions, and chart maps | Shared selected roots, exact trims, spatial/pcurve correspondence, and reusable incidence evidence |
| Material topology and singular strata | Faces, edges, vertices, chart gluing, shell roles, and faithful handling of the declared result domain |

Keep useful exact specializations when completed-workload evidence supports
them. Avoid mandatory global implicitization, large eliminants, or independent
coordinate materialization when a compact source definition suffices. Conversely,
special-case recognizers must not replace completion of the general admitted
domain. Polynomial curves, fixed-degree splines, and NURBS are not universal
exact fallbacks. Fitting a generated curve or surface does not establish closure.

The minimum performant set is the smallest coherent implementation set meeting
the declared workload envelope with evidence. Add, merge, or remove
specializations only after checking exactness, composition, maintained proof
structure, and cost. General and specialized paths must agree on their claims.

## 4. World geometry, pcurves, and chart laws

Algebraic world geometry does not imply algebraic coordinates in every chart.
The algebraic core concerns geometric coefficients and world selections, not
arbitrary algebraic-valued angle inputs: a one-radian point on a circle need
not have algebraic coordinates. Preserve exact angular definitions as qualified
extensions or certify equivalent algebraic direction/endpoint relations.
For a cylinder `S(u,v) = (R cos(u), R sin(u), v)`, intersection with `z = a x`
has the angular-chart pcurve `v = a R cos(u)`, although the spatial intersection
is algebraic. Retain a certified chart relation or use compatible algebraic
charts; do not require that angular pcurve to become a polynomial or NURBS curve.

The converse also fails: lifting the UV line `(u,v) = (t,t)` through a unit
cylinder gives `(cos(t), sin(t), t)`, a helix outside the algebraic curve domain.
Do not promise algebraic closure for arbitrary pcurve lifts, arbitrary surface
parameterizations, or unrestricted composition of analytic chart expressions.
Admit the actual image and construction law, not merely the input type tags.

Every edge has one canonical spatial identity. Every incident use retains its
own pcurve, traversal, and complete parameter correspondence. Prove throughout
the active domain that the spatial curve equals each surface/pcurve image. For
an intersection edge, both incident surfaces and both pcurves must refer to the
same selected component. Endpoints, sampled coincidence, or overlapping numeric
enclosures do not establish this equality.

Keep periodic quotient identity distinct from equality of lifted parameters.
Seams can have distinct pcurves for one spatial edge, including two uses by the
same face. Preserve selected lifts, period counts, branch choices, and chart
transitions through splits, reversal, transforms, and replay. A chart boundary
need not be a physical edge. Preserve legal boundaryless faces and explicit
pole, apex, or collapsed-chart strata; do not fabricate ordinary zero-length
edges to satisfy a wire invariant.

UV distance and angle are not world-space distance and angle. Use the induced
metric, an isometric chart, or a certified spatial construction for metric
operations. Offsetting a pcurve in its parameter plane is not generally a
physical constant-distance offset on the surface.

## 5. Complete intersections and reusable arrangements

Complete point/curve, curve/curve, curve/surface, and surface/surface incidence
for each admitted domain, including native and generated carriers. A newly
generated intersection edge must remain intersectable with a third surface and
splittable together with every incident pcurve. Retain exact contained and
overlap domains and all participating parameter correspondences.

An intersection result must distinguish isolated contacts, curves, overlapping
patches, and other positive-dimensional components. Preserve multiplicity,
tangent contact, branch incidence, and parameter correspondences where needed
for later material selection. Contact existence is not completeness of the
contact set. Local continuation, Newton convergence, or boundary crossings alone
cannot rule out interior loops, disconnected branches, or singular components.

Provide a complete component census for each admitted domain. Handle tangencies,
common factors, coincident patches, projection exceptions, and singular events.
Exclude the parameter diagonal and authorized atlas identifications exactly
when checking self-intersections; never discard a numerical neighborhood of
them. Validate candidates from squaring or elimination against original
equations, selectors, domains, nonzero denominators, and sign conditions.

Build shared arrangements from original trims and all relevant intersection
traces. Atomize crossings and overlaps, retain directed incidence, and certify
connected cells. One witness classifies a cell only after its arrangement proves
that no unresolved material boundary crosses it. Include wholly interior loops,
nested loops, and periodic gluing. Share one canonical intersection edge between
operands while retaining distinct face pcurves.

Reuse complete contacts and arrangements across Boolean selections and
expression roots. Additional selections must not needlessly rebuild the same
intersection computation. Preserve disjoint, identical, empty, and supported
specialized paths without narrowing the general operation's domain.

## 6. Topology, material, and publication

Preserve the immutable model and scoped ownership rules. Publication requires
the applicable geometric and topological proofs, including valid references,
ordered uses, full image agreement, face validity, shell embedding, material
orientation, and cavity nesting. A failed edit must not mutate its source or
publish a partially completed ordinary result.

For manifold volume bodies, check two opposite uses at each regular edge and
the required vertex-link cycles. Edge-use counts alone do not exclude pinches.
Check self-overlap within a face and intersections between nonincident faces.
Adjacent faces must also be checked away from their authorized shared boundary.
Distinguish topological identity, scalar equality, curve/surface image equality,
parameter correspondence, and equality of material sets.

Material orientation follows the represented interior, including reflected
models, reversed boundaries, void shells, and coincident faces. Prove nesting
and boundary ownership. A closed-form area or volume integral is not a
prerequisite for valid topology; unavailable metrics remain separate queries.
Construction-family certificates are efficient sufficient proofs, not the only
admission route for an otherwise valid mixed curved shell.

Regularization removes lower-dimensional remnants from volume results but does
not guarantee a manifold boundary. Keep the current manifold `Solid`/body API
explicitly qualified. Representing all regularized algebraic sets requires a
faithful contact-aware decomposition or stratified extension, with an explicit
API/model migration when necessary. Do not silently change `Model` semantics,
duplicate singular geometry and call it manifold, or hide nonmanifold results
as empty. Separate sheet, wire, contact, and volume meanings. Body selections
denote their documented set union; counts and summed component measures need
not describe the normalized union.

## 7. Offsets, shelling, fillets, and chamfers

Separate a surface parallel, regularized solid offset, hollowing/shelling, and
sheet thickening. Round Euclidean solid offsets use regularized ball dilation
and erosion with exact distance semantics.
Extended-surface or miter joins have a different contract. A displaced face
collection alone is not a completed solid offset.

A surface parallel retains its selected normal branch, positive normal-length
root, domain, and regularity conditions. Repeated normal displacements may be
simplified only after certifying the required branch, orientation, and regular
domain. Local support simplification must not erase intermediate topology
changes or imply that dilation and erosion undo each other. At zero distance,
preserve the source without unnecessarily requiring a normal.

Generate and globally select displaced faces, edge transition envelopes, and
vertex patches. Account for focal singularities, collapsing walls, vanishing
components, self-contact, and closest-feature changes across the complete
source. Shelling specifies openings, thickness side, and rim law; prove the
complete wall and its claimed thickness relation. Local curvature regularity
does not prove global separation. Reject invalid radius or thickness values;
silently taking an absolute value or clamping them changes the construction.

Rolling-ball fillets require a selected center spine, contact rails, envelope
branch, end trims, corner patches, and final material validation. Certify radius
and required continuity over complete rails. A successful edge strip with open
ends is not a completed fillet. General curved-face chamfers require stated
rail and correspondence laws and need not be planar. Account for every requested
edge and corner as completed, rejected, or explicitly superseded. Ordinary
operations are atomic; partial previews identify every omission.

A general spatial curve has no canonical unique normal. Tube, ribbon, sweep,
frame-transport, and normal-field operations require an explicit law and its
singularity contract. Variable-radius or higher-continuity blends, geodesics,
true arc-length setbacks, and general sweep/loft laws require separate scoped
closure and decision guarantees. Parameter distance, chord distance, geodesic
distance, and arc length must not be silently substituted for one another.

## 8. Proof transport, exactness, and transformations

Preserve or re-establish defining relations, correlated selected fields, branch
and domain selectors, contacts, overlap maps, tangencies, and source-to-result
identity. Do not discard a construction proof and force consumers to recover it
through rounded coordinates or repeated global elimination. Allocation identity
may accelerate a proof; it cannot define mathematical equality.

Every cached certificate or proof flag must name its premises and scope. A
restriction, reversal, transform, or edit must either transport those premises
validly or invalidate the affected proof. Geometry/proof graphs must not create
ownership cycles. Cold replay independently reconstructs their authority.

Detect removable rational factors before declaring a genuine pole. Cancellation
must preserve authored exclusions and original denominator guards. Admitting a
formerly excluded parameter requires an explicit extension contract and a
finite branch-limit proof. Preserve one-sided singular limits or alternative
charts when regular formulas fail; do not discard meaningful multiplicity.

Similarities and general affine maps have different metric effects. Transport
orientation under reflection. An affine image of an analytic primitive may
require a different exact carrier. General affine maps do not commute with
Euclidean offsets. Singular maps require explicit lower-dimensional semantics
or a typed refusal before solid publication.

Keep numerical certainty, topological validity, and source fidelity separate.
Geometric approximation or scalar demotion that changes authoritative geometry
requires an explicit boundary with a loss model and stated guarantees. Certified
internal filters and enclosures are allowed when they justify every consumed
decision and preserve exact authority. Display meshes, fitted NURBS, and sampled
pcurves must not become hidden exact-operation inputs.

## 9. Persistence, exchange, and resources

Native exact replay retains shared sources, selected roots/fibers, charts,
correspondences, topology, and required provenance. Validate definitions and
selectors; serialized certainty flags and live caches are not proof. Use
deterministic graph references without claiming a universal canonical form for
equivalent geometry. Preserve semantic equality separately from serialized-byte
stability, and document deliberate format changes.

STEP is a format-specific mapping with entity, chart, coordinate, unit, and
representation limits. Native exact support does not establish exact STEP
encodability. Report unsupported or omitted content explicitly; preserve source
accounting. Approximate export must be intentional and separately qualified.
Validating a quantized reimport does not certify fidelity to the original model.

Distinguish precision, total work, coefficient growth, expression/system size,
temporary and retained memory, and cancellation. Share accounting across nested
operations and allocate worker budgets explicitly. Degree or refinement limits
and cache-entry counts alone do not establish total time or memory bounds.
Archive syntax/structure limits likewise do not bound proof reconstruction.

Exhaustion requires an actually reached declared limit and must occur before
publishing a complete result. Cancellation releases owned temporary state.
Exhaustion cannot poison a larger-budget retry or become permanent proof of
undecidability. Publish measured ordinary and stress envelopes; no uniform
practical bound is promised for arbitrary algebraic degree or operation depth.

## 10. Admission and regression evidence

Before changing representations, algorithms, conversions, or consumers, identify
the owning layer, affected closure relationships, and a failing or missing
acceptance witness. Maintain separate coverage for representation, evaluation,
complete intersections, chart arrangements, stitching, solid admission, later
operations, and replay. A typed blocker is honest reporting of an open gap,
not completed support. Preserve existing successes and use explicit migrations
for intentional contract changes.

Test outputs as subsequent inputs through applicable construction, intersection,
Boolean, offset or corner editing, reconstruction, transformation, cold replay,
and further modeling. Cover mixed/generated carriers, selected parameters
without scalar views, interior loops, coincidences, tangencies, singularities,
seams/poles, cavities, empty results, nonmanifold contacts, tiny distinct features,
invalid archives, cache/policy transitions, budget exhaustion, and recovery.
Apply the existing affected-contract validation rules; unrelated changes do
not acquire a new unconditional full-matrix requirement here.

Use independent exact expectations, full-image identities, component counts
with completeness proofs, topology invariants, and representation-aware
generators. Sampled membership, area/volume, checksums, or agreement between
wrappers of one kernel cannot alone prove the intended result. Keep hard
original cases beside minimized regressions. Native and WASM qualification,
format interoperability, and shipped consumers have distinct evidence gates.

Report certified success, qualified approximation, unsupported, undecided,
invalid, nonmanifold, exhausted, and incorrect outcomes separately. Conformance
fixtures must complete within their published envelope; controlled exhaustion
tests resource handling. Compare performance on the same completed contract,
including required output materialization and evidence. A finite suite is not
a universal closure proof: claims require the domain, representation argument,
algorithmic coverage, implementation review, and maintained qualification corpus.

