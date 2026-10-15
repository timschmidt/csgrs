# HyperBRep algorithm feasibility and performance research

Research date: 2026-10-14. Inspected repository revision:
`4529450d6` on `brep`. This supplements the
[development plan](DEVELOPMENT_PLAN.md) and
[repository audit](REPOSITORY_AUDIT.md). It records algorithm choices and
experiments to implement, not newly implemented capabilities or measured
speedups. No external implementation was built or benchmarked for this
research. The earlier native test result belongs to the audit's baseline.

The governing order remains [properties.md](../../../properties.md):
**exactness > completeness > performance > memory > binary size > source
size**. In particular, a published algorithm's use of “topology guaranteed,”
“exact,” or “watertight” does not establish that its arithmetic, hypotheses,
output representation and degeneracy handling meet this repository's
contract. Source findings and proposed applications are distinguished below.
The [admission protocol](../../../docs/admission.md) remains normative;
this document does not create another property-status ledger.

## Feasibility assessment

Exact curved boundary modeling has substantial precedent, but general,
practical, degeneration-complete NURBS Booleans are still a major engineering
and algorithmic undertaking. The strongest route is a portfolio of certified
reductions sharing retained geometry and proof infrastructure. A numerical
tracer, a generic elimination command, or a mesh Boolean cannot alone supply
that infrastructure.

The distinctions in the following table matter for release planning.
“Prototype required” means the mathematical route is credible but its cost
and coverage have not been established in HyperBRep. It does not authorize
approximate topology or removal of an existing exact capability.

| Domain | Assessment | Required gate and owner |
| --- | --- | --- |
| Rational-coefficient planes and quadrics | Strong exact algorithm precedent, including non-rational intersection curves; finite trimmed-domain integration remains work | Quadric classification, branch identity and both pcurves in HyperBRep; exact polynomial operations in Hypersolve; WP04 |
| Plane/quadric against rational patches | Prefer reduction to a bivariate equation; practical prototype required for branch topology and inverse-chart clipping | Complete interior/boundary event census and retained trims; WP05a–WP05b |
| Regular rational patch pairs | Credible exact/hybrid routes; choose between certified implicitization and four-parameter equations by measured cost | Complete component discovery, regular continuation and replay, including closed interior loops; WP05b |
| Singular, tangent and shared-component intersections | Separate substantial work; regular-root interval methods are insufficient | Local branch incidence, multiplicity, dimensional decomposition and exact overlap maps; WP05c |
| Finite semialgebraic inputs over rational or algebraic coefficients | General real-algebraic decision methods provide a theoretical route; they do not imply usable CAD latency or memory | Degree/bit/work budgets and typed blockers; no unrestricted practical-completeness claim |
| Arbitrary computable `Real` coefficients or analytic laws | No universal equality/zero-decision guarantee follows from more precision | Preserve `Undecided` and admitted specialized certificates in Hyperreal/Hyperlimit/Hypersolve |
| General rational-surface parallels and constant-radius blends | Exact algebraic relations are possible; rational/NURBS representational closure is not general | Retained relations, branch guards, global trimming and downstream composition; WP10–WP11 |
| General sweeps, rotation-minimizing frames and arc-length laws | Prescribed rational/algebraic frames are a tractable first domain; arbitrary transport or arc-length inversion is not automatically algebraic | Declare frame/law domain, regularity, closure and solver support; WP09/WP19 |
| Curved measurements and tessellation | Certified enclosures and regular-domain topology algorithms exist; trimmed singular bodies need additional integration | Separate geometric error, topology and conformity proofs; WP12/WP15 |

ESOLID is important evidence for feasibility, not a general completion claim:
its reported practical domain was low-degree geometry, and its implementation
excluded important degeneracies. QI supplies stronger specialized precedent
for quadric pairs. These are reasons to implement analytic and low-dimensional
paths early, while retaining the general intersection work as an explicit
release dependency. [ESOLID](https://gamma-web.iacs.umd.edu/ESOLID/keyser02.pdf),
[QI](https://gamble.loria.fr/qi/).

## Intersection strategy

### Reduce dimension before increasing arithmetic precision

The following is the proposed HyperBRep design, informed by the sources below.
All routes publish the same `IntersectionSet` contract. A route is selected
by certified representation facts and measured cost, not by changing the
meaning of an uncertain answer.

1. Traverse conservative face and patch hierarchies. Missing or undecided
   bounds retain candidates. Cache exact knot-span decomposition and bounds
   on immutable geometry, with instance transforms and active trims accounted
   for separately.
2. Use construction identity and existing analytic formulas first. Extend
   arbitrary quadric pairs with pencil/type analysis and retained branches;
   do not assume every such curve is rational. A non-rational quartic must
   remain available to trimming and subsequent operations.
3. When one carrier has a cheap exact implicit equation, pull it back to the
   other carrier's parameter domain. This reduces topology discovery to a
   bivariate zero set. Recover and certify the other chart coordinates over
   each whole branch, not only at endpoints or sample points.
4. For general rational patches, compare a reusable implicit/matrix
   representation against direct four-parameter equations. Explicit
   implicitization can be valuable when reused, but avoid unconditionally
   expanding a large implicit polynomial for every pair.
5. Use interval/Bernstein exclusion and certified local continuation where
   their hypotheses hold. Route unresolved singular or common components
   to dimensional decomposition and specialized exact analysis.

The degree saving in step 3 can be estimated before doing elimination.
If `S = (A/W, B/W, C/W)` has tensor bidegree at most `(m,n)` and
`F_h(X,Y,Z,W)` is homogeneous of degree `d`, then
`F_h(A,B,C,W)` has bidegree at most `(dm,dn)`. This follows by multiplying
the degrees of each monomial; cancellation can lower the actual degree.
For a bicubic patch, a plane therefore gives at most `(3,3)` and a quadric
at most `(6,6)`. These are equation bounds, not runtime bounds or bounds on
the number of trimmed components. Record actual degree, support and
coefficient bit sizes after exact normalization.

For two rational patches use the original three equalities
`A_i(u,v) W_B(s,t) - B_i(s,t) W_A(u,v) = 0`, together with both nonzero
denominator guards and the two active parameter domains. Clearing
denominators, taking resultants or squaring radicals must retain these
guards. Root candidates need replay against the original relations.
Implicitization can include points outside the authored patch image,
extraneous components or exceptional fibers. A vanishing implicit equation
is therefore insufficient to prove membership in a trimmed face.

The 2023 Yang/Jia/Yan presentation uses moving planes/Dixon matrices,
implicit pullback, critical events and parameter-domain clipping. Its
matrix representation and inversion step are promising prototype candidates.
The slides also explicitly distinguish the implicit surface from the
bounded parameterized patch. The full theorem and base-point assumptions
must be reviewed before admitting a Rust implementation; slides alone are
not that review.
[Author presentation](https://yangjieyin.github.io/homepage/ACMTOG2023/SIGA23-slides.pdf).

### Global discovery and local continuation have different obligations

A complete result needs a covering proof for the original query domain.
Partition it into certified empty cells, regular branch cells, event cells
and positive-dimensional overlap cells. Every subdivision child is accounted
for. Boundary ownership prevents double counting without losing a root on
a subdivision plane. Overlapping isolating boxes may help verification, but
do not authorize merging nearby geometric entities.

For a regular one-dimensional solution of three equations in four
parameters, choose a local projection and prove an appropriate Jacobian
minor nonzero. Parameterized interval methods can then certify a branch
over a parameter interval. A square-system Krawczyk test at one fixed slice
does not prove a whole curve segment, uniqueness in every neighboring
slice, or absence of another component elsewhere in the patch pair.
Include entry/exit and event connectivity in the certificate.

The 2025 Zhang/Cheng/Liu/Zhang method explicitly assumes a starting point
and stopping event set. It addresses local tracing and controls behavior
in both parameter and model space. Use it as a candidate local accelerator
after global discovery; its stated problem does not supply the missing
starting point on every interior loop.
[Publisher introduction](https://www.sciencedirect.com/science/article/pii/S0167839625000214).

Generate the global census using projection-critical events, domain and
trim boundaries, singularities and shared components. A compact interior
loop has extrema under a suitable projection even when it has no boundary
crossings. Prove projection suitability, detect exceptional/vertical
components, and retry an exact coordinate change when appropriate. A
coordinate change must preserve the original geometry and be retained in
the proof. Perturbing authored geometry to make the case generic is an
explicit approximation operation, never a strict solver fallback.

### Singularities and coincidence need their own algorithms

Use subresultants, square-free factors, selected fibers and local incidence
counts to distinguish isolated points, crossings, tangency, cusps and
multiple/coincident branches. Multiplicity can determine material crossing;
two faces meeting geometrically need not separate different material cells.
Separate singularities of the spatial curve from singularities introduced
only by its projection. A tiny box around a singularity does not establish
its incident branches or exclude a hidden loop inside that box.

Imbach/Moroz/Pouget give useful certified subresultant/interval machinery,
but require genericity hypotheses: a smooth lifted curve, controlled fiber
multiplicity, leading-coefficient conditions and restricted projected
singularities. Their assumption checker is a semi-algorithm. Some
resultant-cusp decisions cannot be certified by their numerical route;
the discriminant case differs. Record and prove applicability before using
this fast path. Failure to establish it is not disjointness.
[Paper, §§2–4](https://arxiv.org/pdf/1412.3290).

An identically zero resultant requires diagnosis: shared factors,
dimension change, an invalid projection or leading-coefficient degeneration
are possible. For two-dimensional coincidence, retain a certified chart
relation, its exceptional set and the intersection of active trim regions.
Identical control nets after exact operations can provide cheap sufficient
proofs; different control nets do not prove different surfaces. Shared
carrier equality still does not imply equality of the two trimmed faces.

The 2025 Yang/Jia overlap work defines a tolerance-based proximity region
and uses floating-point optimization. Its author presentation reports
misclassification limitations. It is useful for an explicitly approximate
proximity/healing layer, not an exact coincidence test. Setting its tolerance
to zero does not turn the method into an algebraic identity proof.
[Author presentation, slides 8, 10 and 35](https://yangjieyin.github.io/homepage/SIGA2025/SigA25-slides.pdf).

General cylindrical algebraic decomposition is a possible completeness
backstop for finite algebraic cases, not the default production path.
Equational constraints can reduce the decomposition work. Reduced
projection methods have applicability conditions, including nullification
handling; do not silently continue when those conditions fail. Full CAD's
worst-case complexity is not a complexity claim about every specialized
surface-intersection algorithm.
[England/Bradford/Davenport](https://arxiv.org/pdf/1903.08999).

### Retained certificates and ownership

Extend the plan's private intersection proof, rather than introducing a
second public solver/carrier abstraction. The proposed proof contains:

- source carrier/domain identities, coefficient field and exact coordinate
  changes;
- original defining equations, denominator/radical guards and multiplicity
  information;
- a complete domain cover with replayable exclusions and event incidence;
- branch selectors, local projection choices and certified correspondence
  between the spatial carrier and both pcurves;
- certified overlap domains and their chart relations;
- the policy, algorithm/certificate schema version and resource receipt.

All fields that establish validity are constructed through checked private
builders. Candidate boxes, numerical traces and partial covers remain
unpublished solver state. Cancellation or exhaustion returns a typed blocker
with diagnostics, never a partially complete successful intersection set.
Persistent geometry retains enough information to replay the result even
if performance caches are evicted.

| Responsibility | Intended location |
| --- | --- |
| Primitive polynomial normalization, modular operations, root counts, selected fields, algebraic decomposition and replay | Extend `hypersolve`; split new decomposition/continuation modules by mathematical responsibility |
| Planar retained traces, critical fibers, curve arrangements and exact trimming | Extend `hypercurve`; reuse its root and selected-fiber identities |
| Surface equations, chart maps, geometric applicability and branch lifting | Proposed HyperBRep `geometry/algebraic.rs`, `geometry/correspondence.rs`, `intersection/algebraic.rs` and `intersection/branch.rs` |
| Patch bounds, pair traversal and immutable preparation | Proposed HyperBRep `geometry/bounds.rs` and `intersection/broad_phase.rs` |
| Coincident trimmed regions and oriented boundary ownership | Proposed HyperBRep `intersection/coincidence.rs` and arrangement modules |
| Primitive fact/sign/enclosure costs and sound filters | Existing Hyperreal/Hyperlimit ownership; no alternate scalar engine inside BREP |

CGAL's algebraic kernel is a useful representation precedent: a planar
algebraic point retains an x-root, a curve and a fiber index, with expensive
y-coordinate materialization deferred. It also caches curve and curve-pair
analyses. This supports keeping Hyper's selected-fiber identity through
operations, rather than eliminating each coordinate repeatedly.
[CGAL algebraic-kernel manual, Implementation](https://doc.cgal.org/latest/Algebraic_kernel_d/index.html).

### Selected spatial evaluation follow-up — 2026-10-15

CGAL's Bézier point interface also constructs a point from a curve and an
algebraic parameter, and distinguishes available exact coordinates from a
deferred representation. This is a representation precedent, not evidence
that its approximation policy matches Hyper's contract.
[Bézier point reference](https://doc.cgal.org/latest/Arrangement_on_surface_2/classCGAL_1_1Arr__Bezier__curve__traits__2_1_1Point__2.html).

The HyperBRep implementation now retains a spatial curve plus one selector,
reusing Hypercurve's XY/XZ image predicates. This preserves correlation
without independent coordinate elimination. Both image preparations are
demand-driven and share the source's lifetime. This is our implementation
choice; the literature does not establish its cost. The
[qualification](evidence/2026-10-15-retained-spatial-points/README.md) measures
it and keeps the selected edge/pcurve/archive gate open. Generic geometric
DAG laziness motivates such sharing, but does not justify accumulating a new
transform wrapper for each edit.
[Pion/Fabri](https://arxiv.org/abs/cs/0608063).

## Offsets, shelling, fillets and sweeps

### Preserve exact relations rather than promising rational closure

For a regular rational surface `S(u,v)`, let
`N = S_u × S_v`. A signed parallel at distance `d` can be retained using

```text
q² = N · N
q > 0
q (X - S) = d N
```

with the original chart/denominator guards and `N != 0`. The positive-root
condition selects the oriented normal. Without it, squaring admits the
opposite parallel. At singular chart points this relation does not supply
a normal; use an admitted alternate chart or singular-stratum construction.
These equations are a proposed representation, not an implemented solver.

The offset literature identifies special rational-normal surface families
with rational offsets; it does not make arbitrary rational surfaces closed
under rational offsetting. Keep recognized rational cases as accelerators,
and retain algebraic parallels for the general admitted input domain.
Consequently, exact native persistence and explicit STEP export
representability are architectural requirements, not optional cleanup after
shelling.
[Krasauskas/Peternell, rational offset surfaces](https://www.geometrie.tuwien.ac.at/geom/ig/peternell/Krasauskas_Peternell.pdf).

### Global solid offsets require more than face parallels

The round solid contracts are dilation `K ⊕ B_d` and erosion `K ⊖ B_d` for
nonnegative `d`; signed API requests select the appropriate operation.
Regularize volume-body results by taking the closure of the interior;
lower-dimensional collapse events can be retained as contact diagnostics.
Construct candidate face, edge and vertex envelopes, then determine the
boundary of the requested material set. Boundary selection needs global
distance/material evidence. Being at distance `d` from one selected face is
insufficient if another feature is closer or the candidate lies inside the
result. Trim domains and nearest boundary strata participate in this test.

Local offset regularity and global embedding are different checks.
Principal-curvature conditions detect local focal behavior, while distinct
source points can generate a global collision before a local test detects
trouble. The MIT author text treats local and global offset
self-intersections separately. Shelling needs both, plus openings and rim
closure; an offset that changes topology can still be a legitimate result
under its stated regularized-set contract.
[Surface offset singularities](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node225.html),
[curve local/global offset behavior](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node220.html).

For self-intersection, solve `S(u,v) = S(s,t)` away from the parameter
diagonal and authorized atlas identifications. The diagonal is an inherent
positive-dimensional solution, not evidence of self-intersection. Use exact
domain ordering/decomposition or a justified algebraic exclusion; never
remove a numerical band around the diagonal. Periodic seam duplicates and
real self-contact must remain distinguishable.

Control-point exclusion tests from recent NURBS self-intersection research
are worth a separate prototype. A sufficient exclusion certificate can
save expensive pair solving, but failure of that certificate does not prove
a self-intersection. This review reached the authors' institutional
description and bibliographic entry, not a full proof audit; no new
signature formula is admitted on that evidence.
[Author institute report](https://www.amss.ac.cn/kyjz1/202501/t20250123_7522288.html),
[paper record](https://team.ustc.edu.cn/Chen_Reaserch_Lab/zh_CN/lwcg/1025581/content/63539.htm).

### Fillet feasibility and frame restrictions

For a proposed rolling-ball spine `C(t)` and radius `r(t)`, the sphere
envelope satisfies

```text
(X - C(t)) · (X - C(t)) = r(t)²
(X - C(t)) · C'(t) + r(t) r'(t) = 0
```

The second equation is obtained by differentiating the first at fixed `X`.
These relations alone do not select the intended envelope branch, prove
regularity, provide all contact rails or close a corner network. Retain
those obligations separately. Constant-radius spines from intersecting
offset supports are a useful first construction; finite face trims and
collisions with unselected features still govern the completed fillet.
The rolling-ball construction has analytic precedent for double contact
with quadrics, not a universal arbitrary-corner solution.
[MIT rolling-ball discussion](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node227.html).

Admit corner families with explicit boundary and continuity contracts.
Use exact analytic corners first; require a constructive patch and a
full-boundary proof before expanding to other networks. Smooth-looking
samples or a fitted NURBS corner do not close WP11. Variable radius/G2
extensions must state the law's parameter, positivity, admissible
derivatives and singularity handling.

Rational rotation-minimizing frames exist for special polynomial curves;
the classification is a proper restriction even within Pythagorean-hodograph
curves. It does not justify a rational exact frame for every NURBS spine.
Implement prescribed frames first, optionally add a recognized RRMF family,
and treat general transport as its own admitted analytic relation. Arc-length
inversion likewise requires its own proof and solver support.
[Farouki et al., RRMF characterization](https://arxiv.org/abs/1604.07008).

## Performance opportunities grounded in the existing implementation

### Existing machinery to extend

The inspected source already contains important accelerators:

- [modular_gcd.rs](../../hypersolve/src/modular_gcd.rs) has one-sided
  rational modular coprimality proofs and algebraic-extension checks.
  Admitted primes preserve the required degrees and nonzero denominators;
  these good-prime checks are part of the proof. A nonconstant
  modular GCD is not by itself an exact common-factor certificate.
- [integer_interpolation.rs](../../hypersolve/src/integer_interpolation.rs)
  already uses modular reconstruction and exact replay for rational
  polynomial operations. Extend that machinery before adding a competing
  CRT engine.
- [bareiss.rs](../../hypersolve/src/bareiss.rs),
  [resultant.rs](../../hypersolve/src/resultant.rs) and
  [tensor_resultant.rs](../../hypersolve/src/tensor_resultant.rs) provide
  fraction-free/resultant and tensor reconstruction paths. Sparse-input
  support does not establish that every subsequent matrix or tensor stays
  sparse; measure densification and peak coefficient storage.
- [root_isolation.rs](../../hypersolve/src/root_isolation.rs) already has
  reusable `UnivariateSturmSequence`, square-free processing,
  Descartes/Bernstein count interfaces and sign bisection for owned isolated
  roots. “Avoid rebuilding Sturm for every bisection” is partly satisfied,
  not a new optimization to claim wholesale.
- [boolean_kernel.rs](../src/boolean_kernel.rs), `intersection_graph`, already performs
  certified bound rejection but visits the Cartesian product of face
  lists. A hierarchy should reduce pair visits, not merely add another
  bound test inside the same quadratic loop.

### Proposed experiment order

These rows are hypotheses, not retained optimizations. Record their eventual
`kept`, `rejected`, `already-satisfied` or `architecture-inapplicable`
disposition in the existing performance ledgers with decisive evidence.
The early rows reduce redundant work before introducing a new solver.

| ID | Experiment and location | Measure and gate |
| --- | --- | --- |
| E01 | Dual face/patch BVH traversal in BREP; conservative exact bounds, prepared immutable operands | Visited node/leaf pairs, actual narrow-phase calls and build/refit cost; sparse and dense cases; checked count arithmetic; preserve deterministic graph/output ordering |
| E02 | Transactional topology edits and dependency-scoped revalidation in BREP | Revalidation count, touched topology and retained certificates; unchanged shells reusable only with complete dependency evidence; final atomic publish still validates the result |
| E03 | Cache active knot spans, homogeneous basis/derivative windows and denominator work in BREP/Hypercurve | Control points touched, evaluations, allocations and retained bytes; repeated knots, mixed weights, exact transforms and edited trims |
| E04 | Geometry-level prepared relations and shared selected-field proofs in Hypersolve/Hypercurve/BREP | Repeated eliminations, materializations, root refinements and live DAG bytes; cold/warm/evicted states and save/load composition |
| E05 | One exact curved arrangement for multiple Boolean roots; classify certified cell seeds then propagate oriented material labels | Ray/classification calls and extra-root cost; nested disconnected components, cavities, multiplicity, consistency around cycles and coincident ownership |
| E06 | Local exact coordinate normalization and homogeneous/indirect predicates for supported analytic constructions | Coefficient growth and filter pass rates; tiny features far from origin; exact transform replay, denominator sign and zero handling |
| E07 | Certified Newton/Descartes acceleration of isolation/refinement in Hypersolve | Clustered roots versus simple low-degree cohorts; proof cost, precision and subdivisions; retain current cheap paths and half-open endpoint ownership |
| E08 | Sparse/degree-aware elimination, projection choice, modular reconstruction and matrix implicitization | Time, peak bits, nonzero support, matrix dimensions and replay time; bad primes, base points and exceptional fibers; complete original-equation replay |
| E09 | Generated error-bounded filters for measured hot polynomial predicates in Hyperlimit | Success rate and total cascade cost, including conversion enclosure and fallback; native/WASM arithmetic assumptions and exact degeneracies |
| E10 | Certified local continuation after a complete event census | Boxes, continuation steps and complete-model latency; no branch jumping, hidden loops or loss at chart transitions; compare parameter and model-space bounds |
| E11 | Certified injectivity/normal-cone exclusion for self/pair intersections | Exclusions versus certificate cost and pathological fallback cost; failure of a sufficient test remains unknown |
| E12 | Deterministic parallelism and `fearless_simd` in verified filters, pair proposals and independent algebra work | End-to-end time, scheduling overhead and total memory; serial/native/WASM parity, cancellation and fixed global budgets |
| E13 | Linear-operation Bézier point evaluation versus de Casteljau in Hypercurve/BREP | Exact scalar/control cohorts, endpoint guards, division cost, expression/coefficient growth and scratch allocations; preserve splitting and derivative certificates |

E01–E06 can begin against existing supported families. E07–E11 should use
the intersection prototype corpus rather than wait for a large completed
kernel. E12 follows stable proof sharing and resource accounting. Do not
hold architectural progress hostage to an asymptotically interesting
optimization that loses on the actual low-degree workload.

For E13, Woźny/Chudy provide a convex-combination evaluator for positive-weight
polynomial/rational Béziers with linear arithmetic-operation count and constant
auxiliary storage. Their theorem requires nonnegative normalized basis
functions and positive weights; zero basis values need separate handling.
This fits the current positive-weight native point-evaluation domain as a
prototype. The stated complexity is not an exact-arithmetic bit-cost bound:
additional divisions and intermediate expression growth may erase the gain.
It does not replace subdivision or selected-root sign proofs. Measure native
degrees 1–32, tiny endpoint parameters and mixed `Real` representations before
changing either geometric owner; retain de Casteljau where its intermediate
controls are required.
[Paper, §§1–2](https://arxiv.org/pdf/1803.06843).

Lazy exact geometric computation supports deferring an exact construction
until a downstream predicate needs it, with geometric DAG nodes rather than
unbounded scalar-expression expansion. Apply that idea by extending existing
sharing and proof identities; do not stack a second opaque lazy engine over
`Real`.
[Pion/Fabri](https://arxiv.org/abs/cs/0608063).

Indirect predicates evaluate a predicate through the exact construction
inputs instead of a rounded intermediate point. Attene's examples support
analytic rational construction paths; they are not a ready-made solution
for every selected algebraic fiber. Preserve numerator/denominator sign
guards and retain the general exact fallback.
[Attene](https://arxiv.org/abs/2105.09772).

Shewchuk-style expansion predicates address exact decisions for their
floating-point inputs under stated arithmetic assumptions. They do not
automatically decide a predicate on arbitrary `Real` values rounded to those
inputs. New polynomial filters need bounds for source conversion and every
arithmetic step; overflow, underflow and compiler contraction must be
covered or return inconclusive. Generated filters are a candidate after
hot paths are measured, with native and WASM proof assumptions recorded.
[Shewchuk](https://people.eecs.berkeley.edu/~jrs/papers/robust-predicates.pdf),
[Fast Floating-Point Filters for Robust Predicates](https://arxiv.org/abs/2208.00497).

ANewDsc combines certified Descartes decisions with Newton acceleration and
adaptive precision. Its square-free hypothesis matters; repeated-root
multiplicity must remain available after square-free reduction. Prototype
cluster acceleration and admissible subdivision points against the existing
Sturm/reference paths. The implementation paper also measures small-case
overhead; its reported speedups are not forecasts for Hyper's coefficient
types, budgets or workloads.
[Sagraloff/Mehlhorn](https://arxiv.org/abs/1308.4088),
[Kobel/Rouillier/Sagraloff](https://who.paris.inria.fr/Fabrice.Rouillier/AnewDesc/ANewDsc-final.pdf).

The mesh-arrangement literature supports reusing a common arrangement and
propagating winding information for Boolean selection. Transfer the
architecture to certified curved cells only after their adjacency and
crossing semantics are established. A mesh proof is not a proof that a
curved arrangement was complete.
[Zhou et al.](https://www.cs.columbia.edu/cg/mesh-arrangements/).

## Derived approximations and measurements

Use numerical meshes and solvers to propose work order or candidates, then
verify against retained exact carriers. The 2025 hybrid BREP Boolean paper
uses mesh correspondence and numerical optimization. Its indexed primary
text is useful for candidate-generation and comparison ideas, but is not
an exact-Hyperreal proof. No performance claim from it is adopted here.
An authoritative mesh-guided shortcut would additionally need a certificate
preserving the *joint* intersection arrangement: separate approximations of
two bodies can each preserve their own topology yet change their contact.
[Yang et al., primary paper](https://publications.rwth-aachen.de/record/1016540/files/1016540.pdf).

For tessellation, keep three independent obligations: source-to-mesh
geometric error, topology preservation and shared-boundary conformity.
Plantinga/Vegter provide interval-based isotopic approximation for regular
implicit curves/surfaces. Their regularity hypotheses do not cover every
trimmed BREP seam, pole or singularity. Begin with regular patches and exact
shared-edge sampling, then prove how the stratified patches glue together.
Hausdorff error alone is not a topology certificate.
[Plantinga/Vegter](https://www.math.rug.nl/~gert/documents/2004/pv-sgp-2004.pdf).

For measurements, a conservative interval/Bernstein subdivision integrator
is a useful initial reference on compact regular chart domains. Add faster
validated quadrature where the pulled-back integrand satisfies its
hypotheses. Johansson's Petras-based integration work concerns analytic
functions with rigorous complex bounds; it is not a drop-in proof for a
singular trimmed two-dimensional integral. Account for trim-domain area
uncertainty, denominator guards and singular endpoints explicitly. The
integral may be a certified enclosure even when the solid topology is exact.
[Johansson](https://arxiv.org/abs/1802.07942).

## Prototype gates and decisive fixtures

These gates refine the existing work packages; they are not a second
schedule. Every successful geometry prototype must support persistence and
another operation. An unsupported or exhausted case remains visible in
results and cannot be counted as a passing exact solution.

| Gate | Required experiment | Exit evidence |
| --- | --- | --- |
| G1 — retained representation, WP02 | Algebraic endpoint/branch not expressible through the current materialized parameter path | Split both carriers, validate, persist/reload and split/classify again; stable source identity and bounded reconstruction growth |
| G2 — analytic reduction, WP04/WP05a | General quadric-pair type/branch analysis plus plane/quadric pullback into a rational patch | Exact spatial and dual-chart relations, all real components clipped to active domains using G3 where needed, alternate-frame agreement and measured costs |
| G3 — bivariate topology, WP05a | Interior loops, isolated contacts, trim crossings, repeated roots and bad projections | Independent event/root counts and exact connectivity replay; no reliance on boundary-only seeds |
| G4 — regular patch pairs, WP05b | Compare guarded matrix/implicit reduction with direct four-parameter cover and continuation | Complete component census and cover; whole-branch chart agreement; degree/bit/memory/runtime comparison on the same corpus |
| G5 — singular/shared components, WP05c | Nodes, cusps, tangent curves, common factors, rank loss and different parameterizations of the same trimmed carrier | Correct dimension, multiplicity, local incidence and overlap region; alternate projection handles or diagnoses each exception |
| G6 — offsets/fillets, WP10–WP11 | Analytic cases followed by one declared non-rational parallel, self-contact and a completed corner network | Intended distance/material set, complete rails/rims/corners, composition/save-load/query/meshing closure and typed excessive-radius outcomes |
| G7 — derived output, WP12/WP15 | Regular trimmed mixed-carrier body, near-contact bodies and chart singularities | Separate integration bounds, geometric approximation bounds, joint-contact limits and mesh conformity; source topology unaffected by display resolution |

The [2026-10-15 G1 prerequisite probe](evidence/2026-10-15-retained-parameter-feasibility/README.md)
exercises the retained quintic parameter in Hypercurve and records BREP's
`UnrepresentableParameter` boundary at revision `678c8072e`. Planar split/reverse/transform
composition passes without a stored scalar view. Spatial topology, dual
pcurves and graph persistence/replay remain unimplemented by that probe;
G1 remains open.

The subsequent [retained inverse-query increment](evidence/2026-10-15-retained-inverse-parameters/README.md)
removes that query boundary: spatial Bézier/NURBS queries retain selected
roots, certify the remaining coordinate at the same root, and preserve authored
knot charts. It reuses the existing Hypercurve affine chart implementation.
Topology transfer still reports unavailable materialization explicitly; this
does not complete the G1 edge/pcurve/archive composition gate.

The [spatial-point increment](evidence/2026-10-15-retained-spatial-points/README.md)
adds selected evaluation, exact spatial comparisons, bounds and affine
composition. Vertices, edge domains, dual-pcurve subdivision and selected
archive graph replay remained that increment's next G1 obligations.

The [selected-topology increment](evidence/2026-10-15-selected-topology/README.md)
now carries those parameters into vertices, trim ranges and both incident
pcurve uses. Repeated subdivision retains the source chart; reversal changes
traversal independently. A reflection regression caught the distinction between
an increasing interval and directed endpoints. This is a local implementation
result, not a claim that the general intersection pipeline is complete.
Selected graph persistence and recertification after load still keep G1 open.

Small mathematical fixtures should precede large imported examples. The
following are proposed exact counterexamples and proof obligations, not
results of running HyperBRep:

| Fixture | Exact construction and expected distinction |
| --- | --- |
| Interior loop with no boundary seeds | Intersect `z=0` with `z=u²+v²-1/16` on `[-1,1]²`; the circle of radius `1/4` is wholly interior |
| Isolated contact versus empty versus tiny loop | Replace the constant by `δ`, using `δ=0`, positive rational `δ`, and negative rational `δ` in `z=u²+v²+δ`; require three different outcomes |
| Pole introduced by clearing a denominator | Solve `(u²-1)/(u-1)=2` with `u!=1`; the cleared equation has only the excluded root; cancellation cannot discard the original domain guard |
| Lost orientation after squaring | A plane with normal `(0,0,1)` and signed distance `2` must choose `z=2`; `q²=1` without `q>0` also admits `z=-2` |
| Persistent shared component | `f(u,v)=v` and `g(u,v)=uv` share `v=0`; an identically zero resultant cannot mean empty |
| Multiple root without sign crossing | `(t-1)²` on `[0,2]` has a root despite equal positive endpoint signs; multiplicity and square-free evidence matter |
| Local regularity without global separation | Two unit balls with surface gap `g>0` have regular outward sphere parallels that contact at offset `g/2`; local focal tests cannot certify global separation |
| Exact input lost before a filter | The integers `2^53` and `2^53+1` are distinct although their binary64 conversions can agree; source conversion error belongs to the filter proof |
| Parameter-space error amplified | Under `S(u,v)=(2^40 u,v,0)`, a parameter error `2^-20` becomes spatial error `2^20`; world-space guarantees need derivative/image bounds |
| Near contact changes under separate tessellation | Use two externally tangent spheres and varied mesh sample phases; individually valid geometric approximations do not establish their joint contact arrangement |

Extend these with full-resolution sourced cases: mixed-degree/weight spline
pairs; high-bit rational inputs; rotated/reflected frames; repeated knots;
periodic seams and poles; overlap with different parameterizations; base
points; interior and boundary singular events; disconnected/cavity solids;
tiny features at large translations; and long chains of Booleans and edits.
Keep coefficient representations as independent lexicase buckets. A rational
prototype passing does not establish all computable-coefficient cases.

## Measurement and implementation decisions

First capture a frozen baseline on both existing supported workloads and
the prototype corpus. Record source revision, fixture/license/hash,
operation recipe, coefficient tier, degrees, active spans, compiler/profile,
platform/features, policy and work limits. Retain exact output rows,
provenance, certainty, serialized size and checksums. Report rejected,
unsupported, undecided, cancelled and successful cases separately.

Measure wall time distributions and total work, not just successful tracing
time. Useful counters include pair visits, certified exclusions, source
evaluations, eliminations, matrix dimensions, support density, largest
coefficient bit size, root refinements, precision promotions, cache hits,
proof replay, allocations and peak live/retained bytes. Charge expensive
steps before starting them and propagate a single operation budget across
workers. More workers must not multiply the caller's work limit.

Run ablations for individual experiments and useful combinations. In
particular compare E04+E07, E06+E09 and E01+E10: caching, coordinate
normalization and candidate reduction can change which later optimization
is worthwhile. Include tiny easy cases, adversarial clustered cases, cold
and warm caches, persistent shared assemblies and dense intersections.
Use the existing [performance contract](../PERFORMANCE.md), including its
established-cohort regression rule; do not invent a universal latency or
speedup target before measurements exist.

A performance-only change must preserve promised rows, output size and
checksums, not merely an approximately similar shape. Proof scheduling
telemetry may differ, but is kept separate from deterministic model evidence.
For deliberate representation/capability changes, establish a reviewed new
baseline and preserve old supported semantics. Cross-kernel geometric
comparisons may allow different valid decompositions; that is a separate
comparison from this implementation's optimization gate.

The immediate implementation order is therefore: retained-geometry closure
and baseline instrumentation; E01–E04 on existing families; analytic and
bivariate intersection prototypes; regular patch-pair comparison; singular
and coincident completion; global offset/fillet completion; derived-output
and release evidence. STEP syntax and basic exchange can advance alongside
this sequence. No downstream feature should force a premature promise of
general solver completeness.

## Source review limits and follow-up

The cited evidence ranges from paper sections and manuals to abstracts and
author presentations. In particular, ESOLID's architecture/limitations,
CGAL's implementation notes, the resultant/discriminant hypotheses and
singularity tests, CAD's nullification conditions, and ANewDsc's algorithm
and implementation discussion were examined. The RRMF and validated
integration recommendations use the scope stated in their abstracts;
implementation requires further theorem-level review. This is a feasibility
review, not a formal verification of these papers or a reproduction of
their experiments. QI's primary project description was reviewed; its
complete implementation was not audited.

Access was narrower for several recent results:

- **Topology driven approximation to rational surface-surface intersection
  via interval algebraic topology analysis**, Cheng et al., TOG 2023:
  [paper](https://doi.org/10.1145/3592452) and
  [author-posted abstract](https://independent.academia.edu/ZhangBingwei).
  Full text was not retrieved. Its four-dimensional topology analysis is a
  candidate for G4/G5; classify it as awaiting theorem/implementation review.
- **Topology Guaranteed B-Spline Surface/Surface Intersection**, Yang et al.,
  2023: author slides were read; complete paper assumptions remain to review.
- **Topology guaranteed and error controlled curve tracing**, Zhang et al.,
  2025: publisher abstract/introduction were read, including the stated
  starting/stopping-point assumption; no implementation was evaluated.
- **Fast Determination and Computation of Self-intersections for NURBS
  Surfaces**, Li et al., 2025: institutional summary and paper record were
  available; linked full text could not be retrieved. E11 remains conditional.
- **Overlap Region Extraction of Two NURBS Surfaces**, Yang/Jia, 2025:
  author slides were read. The tolerance-based objective is already enough
  to exclude it as an exact coincidence replacement.
- **Boolean Operation for CAD Models Using a Hybrid Representation**,
  Yang et al., 2025: indexed passages from the primary paper were inspected;
  the browser could not retrieve the complete 42 MB PDF. No complete-method
  correctness or performance conclusion is claimed.
- The author's [publication list](https://yangjieyin.github.io/publications/)
  lists **A Robust and Efficient Intersection Algorithm for NURBS Surfaces:
  Handling Small Loops and Tangent Intersections** as accepted in 2026.
  No accessible paper was linked there at review time. Track it as follow-up,
  not as an already reviewed solution to WP05.

Before importing code, datasets or fixtures, record their revision and
license. For example, QI's project page states non-commercial software
terms; its existence does not authorize copying it into the production
kernel. Implement mathematical methods in Rust under the project's normal
admission process and keep optional external oracles outside production
dependencies. No new dependency or license decision is required merely to
retain these research references.

## Selected-root archive replay implementation note (2026-10-15)

The [CGAL Algebraic Kernel manual](https://doc.cgal.org/latest/Algebraic_kernel_d/index.html)
describes polynomial/isolating-interval root construction and root counting.
This supports retaining selected-root definitions instead of materializing
independent scalar coordinates. HyperBRep format 9 reconstructs those premises
through Hypercurve, retaining flattened chart relations and shared curve sources.
The implementation, exact boundary-reassembly conditions and remaining selected
solid/resource limits are in the [archive contract](SELECTED_ARCHIVE.md).
[Measured evidence](evidence/2026-10-15-selected-archive/README.md) covers selected
sheet replay and composition; it does not close G1 or establish general surface
intersection feasibility.
