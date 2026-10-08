# exactCorelib vs. Hyper: exhaustive comparison report

**Audit date:** 2026-08-24
**exactCore subject:** the local `exactCorelib-main/trunk` checkout, which identifies
itself as version 2.1.0 in its Makefile and as the July 2010 version 2.1 release in
its README. The tree contains later edits, so “2.1.0” is a source-tree identity,
not an immutable revision. No repository metadata accompanies this checkout.
**Hyper subject:** all 18 sibling `hyper*` Cargo packages present in the workspace
on the audit date. Versions are listed below.
**Harness:** [`exactcore-hyper-comparison`](README.md), a same-process differential
test and Criterion benchmark suite.

## Executive conclusion

There is a substantial, useful common kernel. exactCore and Hyper agree throughout
the tested domains for exact rational arithmetic, the shared elementary real
functions, complex arithmetic, most vector and matrix operations, core 2D and 3D
predicates and distances, most polynomial operations, and generic-position
Delaunay triangulation. Of 65 distinct semantic comparison families, 49 match
directly, 4 match after an explicit convention adapter, 11 have a preserved
counterexample, and 1 is blocked because exercising the exactCore path is not
memory-safe.

The strongest exactCore capabilities are its compact arbitrary-precision number
tower, lazy `Expr` algebraic-real model, univariate/bivariate polynomial package,
and exact-predicate geometry primitives. The Hyper workspace has a much broader
modern application surface: richer computable-real functions, fixed-size exact
linear algebra with structural facts, policy-aware predicate reports, CAD curves,
mesh booleans, paths, B-reps, SDFs, voxels, physics, packing, circuits/PCB tools,
and solver/application layers.

The main risks in exactCore's compared surface are not rounding errors; they are
topological, convention, lifetime, and concurrency defects in legacy geometry
and a few algebraic algorithms. The most serious is unsafe lifetime behavior in
coplanar 3D clipping. Independent native geometry calls also have to be serialized:
parallel test workers reproducibly trigger invalid native frees. The most
consequential wrong-answer cases are Bareiss determinant pivoting, plane
coincidence, triangle boolean intersections/containment, and a Sturm root-count
failure. All safe counterexamples are retained as named ignored tests rather than
hidden from the passing result.

### Headline numbers

| Measure | Result |
|---|---:|
| Hyper crates audited | 18 / 18 |
| Crates with differential suites | 8 |
| Compatible crates awaiting a differential suite | 1 (`hyperbrep`) |
| Crates with no same-domain exactCore contract | 9 |
| Semantic comparison families | 65 |
| Direct matches | 49 (75.4% of all rows) |
| Adapted/convention matches | 4 (6.2%) |
| Known divergences | 11 (16.9%) |
| Unsafe blocked family | 1 (1.5%) |
| Native exactCore C ABI entry points | 57 (54 oracle + 3 benchmark-fixture) |
| Passing tests after report validation | 71 (serialized native mode) |
| Ignored counterexample tests | 19 |
| Paired benchmark cases | 160 (480 Criterion functions across three tiers) |

“Matched” means the two public operations implement the same contract on the
tables, property domains, degeneracies, and witnesses actually exercised. It is
strong differential evidence, not a proof for every possible input. “Adapted”
means the mathematical information overlaps but an intentional convention,
precondition, or output normalization differs. “Divergent” means a concrete safe
counterexample is preserved. “Blocked” means running the pair would be unsound or
the crate cannot currently be built without changing the subjects under audit.

## Scope: what “exhaustive” means

This report is exhaustive over the *directly comparable semantic public surface*
found in the two local source trees. The canonical inventory is
[`coverage/comparable-api.tsv`](coverage/comparable-api.tsv); every one of its 65
rows appears in this report. The independent crate disposition is
[`coverage/hyper-crates.tsv`](coverage/hyper-crates.tsv); every sibling Hyper crate
appears exactly once below. A manifest test discovers sibling `hyper*` packages
and fails if either inventory becomes incomplete.

The unit of comparison is a mathematical operation, not a spelling. Constructors,
accessors, aliases, trait implementations, policy/evidence variants, batch forms,
and re-exports are assigned to the operation they serve. Conversely, a high-level
domain API is not called a direct counterpart merely because it delegates to an
already-compared primitive. For example, `hyperphysics` shapes consume exact
geometry and `hypersdf` has plane/sphere expressions, but exactCore has neither a
physical-simulation contract nor an SDF-field contract.

This is not an exhaustive line-by-line security audit, an equivalence proof, or a
claim that every demo program is a supported library API. Source-only hazards
noticed while tracing the compared paths are reported separately and are excluded
from the 65-row statistics.

## Reproducible configuration

The build script compiles exactCore with C++11, `-O3`, `-DNDEBUG`,
`-DCORE_DEBUG`, and `-DCORE_LEVEL=3`; it links GMP, MPFR, `libstdc++`, and
`libm`. The narrow wrapper in [`cpp/exactcore_oracle.cpp`](cpp/exactcore_oracle.cpp)
exposes 54 `ec_*` calls declared in
[`cpp/exactcore_oracle.h`](cpp/exactcore_oracle.h). Rust and C++ receive the same
integer or rational input in the same process.

The final verification environment was:

| Component | Version |
|---|---|
| OS context | Linux, local workspace |
| Rust compiler | `rustc 1.97.0 (2d8144b78 2026-07-07)` |
| Cargo | `cargo 1.97.0 (c980f4866 2026-06-30)` |
| C++ compiler | GCC 15.3.1 |
| GMP | 6.3.0 |
| MPFR | 4.2.2 |
| exactCore compilation level | `CORE_LEVEL=3` |
| Correctness-test concurrency | one Rust test worker per executable |

Rational results are compared in canonical exact form. Algebraic/transcendental
values and Euclidean distances are numerically evaluated at an explicit common
tolerance; transcendental `Expr` values should not be mistaken for finite symbolic
identities. Predicate signs use certified/refined sign paths where both APIs offer
them. Root isolators are compared by root count, ordering, interval disjointness,
root containment, and endpoint signs rather than demanding identical interval
endpoints.

Parallel invocation is not a supported harness mode. Repeated
`cargo test --test geometry_3d` runs with the default worker pool abort in native
deallocation with `free(): invalid size`; the same seven live tests repeatedly
pass with `--test-threads=1`. The evidence localizes the failure to concurrent use
of the configured native exactCore path, but this audit did not use a sanitizer to
attribute it to a particular global, reference counter, or allocator. All headline
correctness results therefore use the serialized command shown below.

## What exactCore supports

### Number systems and elementary mathematics

| Facility | exactCore surface | Comparison / limitation |
|---|---|---|
| Integers | `BigInt`, GMP-backed arbitrary precision | Foundational carrier; no separate row for constructors/formatting |
| Rationals | `BigRat`, canonicalization, arithmetic, inverse, shifts, comparison, numerator/denominator/sign/bit queries | Direct overlap with `hyperreal::Rational`; core arithmetic and representation all match |
| Arbitrary-precision floats | `BigFloat`, `BigFloat2`, MPFR wrappers and directed/precision-aware operations | Used beneath evaluation and isolation; not treated as a direct replacement for Hyper's lazy `Real` contract |
| Intervals and filters | `BFInterval`, `IntervalT`, filters, root bounds and approximation policies | Infrastructure for exact decisions/refinement; Hyper expresses related knowledge through `Real` facts and predicate policies |
| Lazy reals | `Expr`, expression nodes, exact algebraic roots, approximation and certified sign | Directly compared with `hyperreal::Real` for the shared function subset |
| Constants | `pi`, `e` | Match numerically at common tolerance |
| Arithmetic | `+`, `-`, `*`, `/`, negation, absolute value, integer power | Match on shared domains |
| Roots | `sqrt`, `cbrt`, `dbrt`, general `root`/`radical`, polynomial `rootOf` | Shared square/cube/nth-root paths match; `rootOf` underpins algebraic values |
| Exponentials/logs | `exp`, `exp2`, `exp10`, `log`, `log2`, `log10` | Shared operations match |
| Trigonometry | `sin`, `cos`, `tan`, `cot`, `asin`, `acos`, `atan` | Shared operations match with domain checks |
| Integral queries | `floor`, `ceil` | Match certified Hyper results |

The inspected exactCore `Expr` surface does not expose direct named counterparts
for Hyper's `atan2`, hyperbolic/inverse-hyperbolic family, `expm1`, `ln_1p`, error
functions, `sinc` variants, or several other stability/special-function APIs.
Those are Hyper-only rather than failed comparisons.

### Algebra and linear algebra

| Facility | exactCore support | Hyper relationship |
|---|---|---|
| Complex numbers | Generic `ComplexT`, Cartesian and polar construction, `+ - * /`, equality, real/imaginary parts, argument and modulus | Arithmetic and squared norm match `hyperlattice::Complex`; Hyper convenience operations composed from these are not double-counted |
| Vectors | Dynamic `VectorT`, arithmetic, dot product, norm, and 3D cross product | Compared with fixed `Vector2`, `Vector3`, and `Vector4`; common operations match |
| Matrices | Dynamic `MatrixT`, arithmetic, transpose, fraction-free determinant, adjugate and inverse | Compared with `Matrix3`/`Matrix4`; all rows match except pivot-requiring Bareiss determinants |
| Univariate polynomials | `Polynomial`: arithmetic, composition, differentiation, pseudo-remainder, resultants, discriminant helper, GCD, square-free helpers | Broad direct overlap with `hypersolve`; three standard-convention defects and one root-count defect are recorded |
| Root algorithms | Sturm and Descartes counting/isolation, refinement, root bounds | Isolation cases match; one simple cubic exposes a Sturm total-count failure |
| Bivariate polynomials | `BiPoly`/`Curves`: evaluation, specialization, arithmetic, resultants/elimination, algebraic-curve infrastructure | Evaluation and both resultant elimination directions match the Hyper polynomial system API |

Hyper additionally exposes fixed-size transformation helpers, normalization and
structural facts in `hyperlattice`, and a large solver layer in `hypersolve`
(nonlinear solving, diagnostics, constraint-domain certification, sparse analysis,
CAD substitutions, algebraic images, and sketch constraints). Those have no
same-contract exactCore public API in this checkout.

### Exact 2D geometry

The extension headers under `inc/CORE/geom2d` provide:

| Type / free function | Supported operations |
|---|---|
| `Point2d` | Coordinate construction/access/mutation, vector conversion, point subtraction/vector addition, equality, Euclidean distance, 90° rotation |
| Point predicates/constructions | Midpoint, affine center, doubled signed area, orientation, left/right turn, collinearity, strict betweenness and a dot-product betweenness variant |
| `Line2d` | Point/vector or two-point construction, implicit coefficients, direction/endpoints, distance, projection, orientation, sine/cosine/slope/intercept, vertical/horizontal/trivial, containment, coincidence, parallelism, intersection dimension and witness, perpendicular bisector |
| `Segment2d` | Endpoints, reversal, supporting line, length, point distance/nearest point, open/directed flags, orientation, trivial/axis/collinear/coincident/parallel, point/segment containment, line/segment intersection dimension and witness |
| `Circle2d` | Three-point, center/point, center/radius and trivial forms; center/radius/defining points; orientation/degeneracy; point side/inside/outside/on-boundary; point/line/circle distance |
| Predicate | `incircle` for a point against an oriented circle through three points |

The direct Hyper overlap is mainly `hyperlimit` plus the line/circle supporting
geometry exposed by `hypercurve` and `hyperpath`. Hyper adds policy-aware evidence,
N-dimensional predicates, rings/polygons, rays, sphere/AABB classification, and
many degeneracy reports not represented by an equivalent exactCore contract.

### Exact 3D geometry

The extension headers under `inc/CORE/geom3d` provide:

| Type / free function | Supported operations |
|---|---|
| `Point3d` | Coordinate/vector construction, arithmetic/equality, Euclidean distance and basic transformations |
| 3D predicates | Tetrahedral orientation and signed six-times-volume, collinearity/coplanarity helpers |
| `Line3d` | Direction/endpoints, incidence, parallel/coplanar/skew relations, distance/projection, intersection dimension and witness |
| `Segment3d` | Endpoint/support operations, containment, coincidence/coplanarity, nearest-point distance, line/segment intersection topology and witness |
| `Plane3d` | Point/normal, three-point, point/line, point/segment, normal/displacement and coefficient construction; expression/normal; containment, projection, distance, line/segment/plane relation and witness |
| `Triangle3d` | Vertices, normal/support plane, containment/inside/edge tests, line/segment/plane/triangle relations and intersection objects, coplanar clipping |
| `Polygon3d` | Linked-list point carrier, copying, point add/remove/search, coplanarity and edge queries, basic verification |

The direct comparison is against `hyperlimit` classifiers and `hypermesh` plane
and convex-triangle APIs. exactCore's nominal breadth here is real, but its plane,
triangle, polygon, and ownership code contains the audit's highest-risk defects.

### Algorithms and demonstrations

The checkout contains programs for convex hulls, lower-hull and incremental
Delaunay triangulation, constrained Delaunay triangulation, and Fortune-style
Voronoi construction, among many research/demo programs. Only `dt4` is included
in the differential manifest because it is a stable, directly comparable
2D Delaunay implementation. The oracle ports its O(n^4) lifted lower-paraboloid
lower-hull test; it does not benchmark an unmodified demo executable.

## Complete Hyper crate disposition

These are local package versions, not claims about crates.io releases. “No direct
counterpart” still allows the crate to consume a lower-level primitive already
compared elsewhere. “Comparison pending” means the crate builds against its
current dependencies and has direct mathematical overlap, but does not yet have
its own differential test and benchmark suite.

| Crate | Version | Disposition | Direct overlap or reason |
|---|---:|---|---|
| `hyperreal` | 0.13.1 | compared | Rational and computable-real arithmetic |
| `hyperlattice` | 0.6.1 | compared | Complex numbers, vectors, matrices |
| `hyperlimit` | 0.4.1 | compared | Exact 2D/3D predicates, constructions, topology and distances |
| `hypersolve` | 0.3.1 | compared | Univariate/bivariate polynomials, resultants and root isolation |
| `hypertri` | 0.4.1 | compared | 2D Delaunay complex/triangulation against exactCore `dt4` |
| `hypercurve` | 0.3.1 | compared, feature-gated | Finite lines and supporting-circle semantics; no exactCore Bezier/B-spline/NURBS counterpart |
| `hypermesh` | 0.1.0 | compared, feature-gated | Planes, convex triangles and pairwise triangle contacts; no exactCore mesh-boolean system counterpart |
| `hyperpath` | 0.3.0 | compared, feature-gated | Line-path metrics/order and explicit full-circle boundary predicates |
| `hyperbrep` | 0.2.0 | comparison pending | Finite `Curve3` lines and planar surfaces overlap and the crate builds against the current `hypercurve` context/outcome API; a dedicated exactCore differential suite remains to be added |
| `hypercircuit` | 0.3.0 | no direct counterpart | Circuit MNA, event routing, board and interchange domain APIs; exactCore has no circuit model |
| `hyperdrc` | 0.3.0 | no direct counterpart | Gerber parsing, spatial indexing and PCB design-rule checking |
| `hyperevolution` | 0.3.0 | no direct counterpart | Evolutionary proposal/fitness/search carriers |
| `hypergraphics` | 0.1.0 | no direct counterpart | Scene and rendering-boundary carriers; primitive math delegates to already-compared crates |
| `hyperpack` | 0.3.0 | no direct counterpart | Packing feasibility and replay contracts |
| `hyperparts` | 0.3.0 | no direct counterpart | Source-attributed part knowledge graph |
| `hyperphysics` | 0.3.0 | no direct counterpart | Physical simulation and mass-property contracts; shape primitives do not make exactCore a physics engine |
| `hypersdf` | 0.2.0 | no direct counterpart | Signed-distance/implicit-field composition contract; isolated exactCore distance methods are lower-level |
| `hypervoxel` | 0.3.0 | no direct counterpart | Voxel frames, sparse grids, aggregate facts and reports |

## Complete semantic comparison matrix

The following tables contain all 65 manifest families. The canonical machine-readable
row for each family also names its exact `ec_*` symbol(s), test source, benchmark
source, participating crate(s), and audit note in
[`coverage/comparable-api.tsv`](coverage/comparable-api.tsv).

### Scalars and elementary real functions (9 families)

Evidence: [`tests/scalars.rs`](tests/scalars.rs) and the scalar groups in
[`benches/comparable.rs`](benches/comparable.rs).

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `rational-binary` | matched | `BigRat + - * /` | `Rational` operators | Canonical reduced outputs agree for signed tables and 256 randomized fraction cases; division-by-zero domains agree |
| `rational-unary` | matched | negation, absolute value, inverse | `Rational` negation, `abs`, inverse | Exact outputs and zero inverse rejection agree |
| `rational-order` | matched | `BigRat` comparison | `Rational::partial_cmp` | Exact trichotomy agrees |
| `rational-representation` | matched | canonical numerator, denominator, sign, zero/one queries | normalized parts and queries | Reduction, denominator sign and large integer parts agree |
| `real-unary` | matched | negation, absolute value, roots, exponentials, logarithms, trigonometry and inverse trigonometry | shared `Real` elementary functions | Shared valid-domain values agree numerically; shared invalid domains reject |
| `real-binary` | matched | `Expr + - * /`, integer `pow` | `Real` arithmetic, `powi_i64` | Rational inputs and algebraic results agree on the shared domain |
| `real-constants` | matched | `CORE::pi`, `CORE::e` | `Real::pi`, `Real::e` | Numeric evaluations agree at the suite tolerance |
| `real-sign` | matched | sign of refined `Expr` | `Real::refine_sign_until` | Certified sign decisions agree for composed expressions |
| `real-integer-rounding` | matched | `floor`, `ceil` | `floor_certified`, `ceil_certified` | Exact integer output agrees, including negative nonintegers and large rationals |

Not counted as separate algorithms: constructors, parsing, display, aliases, cached
facts, evidence/policy overloads, and Hyper elementary functions with no named
exactCore counterpart.

### Complex, vector and matrix operations (10 families)

Evidence: [`tests/linear_algebra.rs`](tests/linear_algebra.rs) and the linear-algebra
groups in [`benches/comparable.rs`](benches/comparable.rs).

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `complex-binary` | matched | `ComplexT + - * /` | `Complex` operators | Exact rational real and imaginary components agree |
| `complex-norm` | matched | squared modulus | `Complex::norm_squared` | Exact rational result agrees |
| `vector-dot` | matched | dynamic-vector dot product | `Vector2/3/4::dot` | Dimensions 2, 3 and 4 agree |
| `vector-binary` | matched | vector addition/subtraction | fixed-vector operators | Components agree in dimensions 2, 3 and 4 |
| `vector-cross` | matched | 3D cross product | `Vector3::cross` | Exact integer components agree |
| `vector-norm` | matched | norm and squared norm | fixed-vector norm APIs | Zero and nonzero vectors agree; 2D wedge is cross-checked through signed area |
| `matrix-determinant` | divergent | fraction-free Bareiss determinant | `Matrix3/4::determinant` | Ordinary samples agree, but exactCore returns the wrong result when a required column pivot is not the current column |
| `matrix-binary` | matched | matrix addition/subtraction/multiplication | `Matrix3/4` operators | Row-major 3×3 and 4×4 results agree |
| `matrix-transpose` | matched | matrix transpose | `Matrix3/4::transpose` | Exact entries agree |
| `matrix-adjugate-inverse` | matched | fraction-free adjugate, determinant and inverse | `Matrix3/4::inverse` | Nonsingular, non-triggering samples agree; the claim explicitly excludes the known pivot counterexample |

The determinant regression uses the matrix `[0,0,-1; 0,1,0; 1,0,0]`. exactCore's
pivot permutation is used during elimination but the final determinant is read from
the unpermuted lower-right entry and no permutation sign is applied.

### Two-dimensional geometry (13 families)

Evidence: [`tests/geometry_2d.rs`](tests/geometry_2d.rs), with supporting high-level
checks in the feature suites.

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `orientation-2d` | matched | free `orientation2d(a,b,c)` | `orient2` | Both windings, collinearity and 256 randomized integer triples agree |
| `signed-area-2d` | matched | `area`/`area2` | `orient2d_value` | Both use doubled signed triangle area and agree exactly |
| `between-2d` | matched | strict `between(a,b,c)` | `classify_point_segment` | Strict interior and endpoint handling agree after selecting the corresponding closed-segment class |
| `line-relations-2d` | divergent | member orientation, incidence, parallel/coincident and axis predicates | point/line and vector classifiers | Incidence and ordinary relations agree; exactCore member orientation is reversed and coincident lines report disjoint intersection dimension |
| `line-intersection-2d` | matched | `Line2d::intersection` | `construct_line_intersection_point` | Unique intersection witnesses agree; coincident topology is isolated in the divergent row |
| `segment-relations-2d` | matched | containment, parallel/coincident and line/segment intersections | point/segment and segment/segment classifiers | Crossings, endpoint touches, overlaps, disjoint cases and degenerate contacts agree |
| `incircle-2d` | matched | `incircle` | `incircle2` | Inside/outside/boundary signs agree for both triangle windings |
| `circle-line-segment-2d` | divergent | circle/line distance and segment-boundary relation | `classify_circle_line2`, `classify_circle_segment2` | Intersection topology agrees; exactCore clamps line overlap/secancy to zero instead of preserving a signed boundary distance |
| `circle-point-distance` | matched | center distance minus radius | Hyper center norm minus radius / supporting-circle APIs | Signed inside, boundary and outside values agree |
| `circle-pair-distance` | adapted | nonnegative clamped circle separation | circle relation plus explicitly clamped supporting distance | Results agree under clamping; exactCore cannot distinguish tangent from secant/overlapping circles by distance alone |
| `point-distance-2d` | matched | `Point2d::distance` | `Point2` displacement norm | Euclidean distances agree |
| `line-point-distance-2d` | matched | `Line2d::distance` | wedge magnitude over direction norm | Supporting-line distances agree |
| `segment-point-distance-2d` | matched | `Segment2d::distance` | projection-clamped segment metric | Endpoint, interior-projection and off-support cases agree |

The free function `orientation2d` is correct on the exercised domain; the reversal
is specifically in `Line2d::orientation`. The line intersection constructor checks
coincidence before parallelism and can construct the coincident line, while the
dimension predicate checks parallelism first and reports `-1`; those two exactCore
APIs therefore contradict one another.

### Three-dimensional geometry (10 families)

Evidence: [`tests/geometry_3d.rs`](tests/geometry_3d.rs), with plane and convex-triangle
checks in [`tests/meshes.rs`](tests/meshes.rs).

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `orientation-3d` | matched | `orientation3d` | `orient3` | Both windings, coplanarity and 256 randomized integer tetrahedra agree |
| `signed-volume-3d` | matched | `volume3` | determinant of tetrahedron offsets | Signed six-times-volume convention agrees |
| `line-relations-3d` | matched | incidence, parallel, coplanar, skew and intersection dimension | point/segment classifiers, cross products and `orient3` | Finite cases cover all relation classes and agree |
| `segment-relations-3d` | matched | containment, coincidence, coplanarity and intersections | `classify_point_segment3`, segment intersection classifier | Point, crossing, touch, overlap and disjoint topology agree |
| `plane-relations-3d` | divergent | point/line/segment/plane side, containment and intersection | `Plane3` predicates and `hypermesh::Plane` | Point/line/segment basics agree; `Plane3d::isCoincident` is inverted, breaking equal and transverse plane relations |
| `point-distance-3d` | matched | `Point3d::distance` | `Point3` displacement norm | Euclidean distances agree |
| `line-point-distance-3d` | matched | `Line3d::distance` | cross-product line metric | Supporting-line distances agree |
| `segment-point-distance-3d` | matched | `Segment3d::distance` | projection-clamped segment metric | Endpoint, interior and off-support cases agree |
| `plane-point-distance-3d` | matched | absolute plane expression over normal norm | `Plane3` expression metric | Values and zero-set classification agree |
| `triangle-relations-3d` | divergent | point/segment/line/triangle containment and intersections | triangle classifiers and `hypermesh::ConvexPolygon` | Safe ordinary contacts agree; exactCore accepts off-plane and beyond-edge points and converts disjoint dimension `-1` to boolean `true` in several paths |

### Univariate and bivariate polynomials (13 families)

Evidence: [`tests/polynomial.rs`](tests/polynomial.rs) and the polynomial groups in
[`benches/comparable.rs`](benches/comparable.rs). The randomized evaluation suite
uses 128 cases in addition to deterministic identities.

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `polynomial-evaluation` | matched | `Polynomial::eval` | `Real::eval_poly` | Exact integer/rational evaluation and randomized cases agree |
| `polynomial-binary` | matched | add, subtract, multiply, compose and pseudo-remainder | coefficient arithmetic and first subresultant remainder | Compared by exact evaluation at shared points; results agree |
| `polynomial-derivative` | matched | nth derivative | iterated coefficient differentiation | Orders through degree exhaustion agree |
| `polynomial-resultant` | adapted | `res(p,q)` | standard `resultant_univariate_polynomials` | Nonconstant results agree after reversing/documenting exactCore's argument/sign orientation |
| `constant-resultant` | divergent | exactCore constant-polynomial base case | standard resultant convention | exactCore returns zero where the standard resultant is a power of the constant |
| `polynomial-discriminant` | divergent | `disc()` cheap resultant proxy | standard normalized discriminant | exactCore omits the conventional sign and division by the leading coefficient |
| `polynomial-gcd` | matched | polynomial GCD degree | terminal subresultant-chain degree | Shared-factor and coprime cases agree |
| `polynomial-square-free` | matched | `sqFreePart` degree | `square_free_part` | Repeated-root factor cases agree |
| `polynomial-root-count` | divergent | total Sturm root count | Hyper certified isolator count | exactCore reports 2 instead of 3 for `(x-1)(x-2)(x-3)` |
| `polynomial-interval-count` | matched | Sturm count on an integer interval | isolated roots filtered to an open interval | Counts agree, including endpoint-root/open-interval conventions |
| `polynomial-root-isolation` | matched | `Sturm::isolateRoots` | `isolate_univariate_polynomial_expr` | Counts, ordering, disjointness, root containment and witness signs agree on the exercised polynomials |
| `bivariate-evaluation` | matched | `BiPoly::eval`/specialization | `BivariatePolynomial` coefficient evaluation | Exact specialization in either variable agrees |
| `bivariate-resultant` | matched | bivariate resultant in X or Y | `resultant_bivariate_polynomial_system` | Both elimination directions agree when evaluated in the retained variable |

The passing isolation row and failing total-count row are not contradictory: the
counterexample is retained separately and the passing claim is limited to the
explicit exercised polynomial set. It demonstrates why a green sample suite is
not treated as a universal proof.

### Triangulation (1 family)

Evidence: [`tests/triangulation.rs`](tests/triangulation.rs), including exhaustive
small configurations, deterministic degeneracies and 96 randomized point sets.

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `delaunay-2d` | adapted | independent O(n^4) empty-circle complex using exactCore `Expr` arithmetic and the `dt4` cell contract | `delaunay_complex` and tie-broken `delaunay` triangulation | Generic simplicial cells agree; the adapter accepts duplicate coordinates while Hyper rejects them, and cocircular adapter output contains the full empty-circle cell complex while Hyper selects a triangulation |

### Higher-level curve surface (3 families)

Enabled by feature `curve`. Evidence: [`tests/curves.rs`](tests/curves.rs) and
[`benches/curves.rs`](benches/curves.rs).

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `curve-lines` | matched | `Point2d`, `Line2d`, `Segment2d` | `LineSeg2`, `Segment2::Line` dispatch | Length, side, finite containment, topology, witnesses and enum dispatch agree |
| `curve-degenerate-line` | divergent | zero-length `Segment2d` accepted | `LineSeg2::try_new` rejects identical endpoints | Constructor contracts differ |
| `curve-circles` | adapted | `Circle2d` point/line/circle relations | `CircularArc2` supporting-circle APIs | Full supporting-circle semantics agree; a partial arc's sweep/parameterization has no exactCore counterpart |

### Higher-level mesh surface (3 families)

Enabled by feature `mesh`. Evidence: [`tests/meshes.rs`](tests/meshes.rs) and
[`benches/meshes.rs`](benches/meshes.rs).

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `mesh-planes` | matched | `Plane3d` construction, expression, sidedness and inversion | `hypermesh::Plane` | Plane support orientation, inversion and point classification agree outside the known plane/plane defect |
| `mesh-convex-triangles` | divergent | `Triangle3d` containment and pairwise relation | `ConvexPolygon`, `intersect_polygons` | Ordinary point/contact cases agree; off-plane, infinite-edge-support and boolean `-1` counterexamples do not |
| `mesh-coplanar-clipping` | blocked | coplanar `Triangle3d` clipping through temporary `Polygon3d` objects | coplanar polygon intersection | No exactCore test or benchmark is executed because the legacy clipping path contains a dangling-pointer lifetime defect and has produced process crashes |

### Higher-level path surface (3 families)

Enabled by feature `path`. Evidence: [`tests/paths.rs`](tests/paths.rs) and
[`benches/paths.rs`](benches/paths.rs).

| ID | Status | exactCore | Hyper | Result |
|---|---|---|---|---|
| `path-lines` | matched | point/line/segment metrics, incidence and betweenness | `LinePathSegment` | Length, axis, endpoints, equality and certified parameter order agree |
| `path-circles` | matched | circle signed point distance and boundary facts | `ExplicitCircularArc` full-circle APIs | Full-circle point membership and segment boundary contacts agree; partial sweep is outside the pair |
| `path-contained-segment` | divergent | signed segment-to-circle-boundary metric | `ExplicitCircularArc::intersect_segment` | A segment wholly inside the disk has a negative exactCore boundary metric but correctly has no boundary hit under Hyper's intersection contract |

## Divergence, adaptation and hazard dossier

Impact labels describe likely mathematical/API consequences, not a security score.
Source references point to the audited local checkout; line numbers are recorded so
the observations remain reviewable even though that checkout has no commit ID.

### Wrong-answer and topology defects

1. **Matrix Bareiss column pivot — high impact.** In
   `inc/CORE/linearAlgebraT.h:617-728`, elimination maintains a column permutation
   array, but `determinant()` returns `A(n-1,n-1)` without reading the permuted
   column or applying permutation parity. A matrix needing a column swap therefore
   gets a wrong determinant; inverse/adjugate users share the risky path.

2. **2D line member orientation — medium impact.**
   `ext/geom2d/line2d.cpp:65-73` implements `Line2d::orientation` with the opposite
   sign from the documented free `orientation2d`. Predicates that use the member
   method can reverse left/right while free-function users get the expected sign.

3. **Coincident 2D line dimension — medium impact.**
   `ext/geom2d/line2d.cpp:100-108` tests `isParallel` before `isCoincident`.
   Coincident lines are parallel, so `intersects()` returns disjoint (`-1`) and
   never reaches dimension 1. The separate `intersection()` method checks in the
   opposite order and returns a line, making exactCore internally inconsistent.

4. **Plane coincidence — high impact.**
   `ext/geom3d/plane3d.cpp:84-90` returns true only when normals have a *nonzero*
   cross product and the stored displacement happens to be equal. Equal planes are
   therefore not coincident, while some transverse origin planes are. Plane/plane
   intersection dimension and equality inherit the error.

5. **Triangle dimension converted to boolean — high impact.**
   `ext/geom3d/triangle3d.cpp:168-181` calls integer-valued `intersects()` inside
   boolean conditions. The disjoint value is `-1`, which C++ converts to `true`.
   The suite exhibits false positives for both a disjoint coplanar segment and
   disjoint triangles.

6. **Triangle containment lacks finite coplanar validation — high impact.**
   `ext/geom3d/triangle3d.cpp:39-57` does not first require the point to lie on the
   triangle plane, and its `s1*s2*s3 == 0` branch accepts points aligned with an
   infinite supporting edge even when outside the finite edge. `inside()` does
   assert coplanarity, but `contains()`—the method consumed by intersection
   construction—does not.

7. **Sturm total root count — high impact.** The exactCore total-count path returns
   2 for `x^3 - 6x^2 + 11x - 6`, whose three simple roots are 1, 2 and 3. Interval
   counting/isolation passes the other exercised inputs, but the concrete total
   count shows the routine cannot be treated as universally reliable.

8. **Constant resultant base case — medium impact.** The implementation and its
   comment in `inc/CORE/poly/Poly.h:1527-1562` use a nonstandard zero base case for
   one constant ordering. Standard elimination defines the result as an appropriate
   power of the constant (subject to argument order).

9. **Discriminant normalization — medium impact.** `Poly.h:1573-1576` returns the
   raw resultant with the derivative and explicitly comments that division by the
   leading coefficient and the conventional sign are omitted. It is a cheap proxy,
   not the standard discriminant advertised by most modern APIs.

### Contract/convention differences

10. **Circle-to-line separation.** `ext/geom2d/circle2d.cpp:108-112` clamps
    `line_distance(center)-radius` to zero. That is a valid nonnegative-set
    separation, but it is not a signed boundary distance and loses penetration
    depth for secants.

11. **Circle-to-circle separation.** `circle2d.cpp:114-118` similarly clamps
    center distance minus both radii. Tangency and every overlap return zero, so a
    second topological predicate is required. The comparison matches only after
    Hyper is explicitly clamped to that contract.

12. **Resultant argument orientation.** Nonconstant exactCore results consistently
    match after the documented argument/sign reversal. This is adapted rather than
    called a wrong mathematical value; the constant base case remains a separate
    true divergence.

13. **Delaunay degeneracy and output model.** The port accepts duplicate coordinates
    whereas `hypertri` rejects them. For cocircular inputs, exactCore's empty-circle
    complex contains all valid cells while `hypertri::delaunay` chooses one
    triangulation. Generic simplicial inputs match.

14. **Degenerate curve line.** exactCore explicitly permits a zero-length
    `Segment2d`; `hypercurve::LineSeg2::try_new` treats identical endpoints as an
    invalid curve. Neither convention is silently normalized.

15. **Contained segment vs. circular boundary.** A finite segment wholly inside a
    disk has negative exactCore support-boundary separation, but no intersection
    with Hyper's circular boundary. This is a query-contract mismatch, not evidence
    that either primitive computed its own contract incorrectly.

### Unsafe blocked path

16. **Coplanar 3D clipping lifetime — critical.** In
    `ext/geom3d/triangle3d.cpp:299-317`, the one-point result of coplanar
    segment/triangle clipping returns `plg2[0]`, a pointer into a local `Polygon3d`.
    `Polygon3d::~Polygon3d()` frees that point as the function returns. The shared
    clipping pipeline also allocates intermediate polygons without clear ownership.
    Because nearby triangle/triangle clipping traverses the same legacy machinery
    and the audited path has produced segmentation faults, the entire
    `mesh-coplanar-clipping` family is conservatively blocked: there is no oracle
    symbol, test, or benchmark for it.

### Source-only exactCore hazards outside the differential count

These observations were found while tracing the compared implementations. They
are not included among the 11 divergent rows because the harness has no safe,
same-contract pair for them. They have not been repaired in either subject tree.

- **`Circle2d` object lifetime and initialization.** In
  `ext/geom2d/circle2d.cpp:41-64`, the center/point and center/radius constructors
  do not initialize `orient`; the default constructor initializes neither `orient`
  nor owning pointers `cp`/`rp`, although the destructor conditionally deletes the
  pointers. The assignment operator shallow-copies those owning pointers, allowing
  double deletion. A copy constructor and point-only constructor are declared in
  the header but have no definition in the extension implementation. The oracle
  avoids copying and uses only constructor paths whose pointers are initialized.

- **`Plane3d::coeffients()` writes one slot four times.** At
  `ext/geom3d/plane3d.cpp:72-81`, all four assignments target `params[0]`; elements
  1–3 remain uninitialized. The correctly named scalar accessors `A/B/C` and
  `displacement` are separate and are used by the oracle.

- **`Segment2d::setOpen(bool)` writes the wrong member.** At
  `inc/CORE/geom2d/segment2d.h:89`, the method assigns `directed = open` and ignores
  its `_open` argument, leaving the `open` flag unchanged.

- **`Polygon3d::verify()` rejects triangles.** At
  `ext/geom3d/polygon3d.cpp:208-229`, `size <= 3` returns false even though the
  warning says “less than three”; the remaining implementation checks coplanarity
  but leaves edge validation unfinished.

- **Unreachable cleanup after returns.** Several 3D intersection functions place
  `delete` statements after unconditional `return` expressions. This leaks
  intermediate intersection objects even on paths that return a safe copied
  witness.

- **Concurrent exactCore calls corrupt native allocation state.** The live 3D
  tests pass serially but reproducibly emit several `free(): invalid size` messages
  and abort when the Rust harness invokes independent geometry cases in parallel.
  The precise exactCore internal root cause is not assigned without sanitizer
  evidence; the oracle is treated as process-global and non-thread-safe.

## The 19 ignored counterexamples

Ignored tests are deliberate executable characterizations. They assert the desired
or cross-library contract and therefore fail against the current source; they are
not flaky tests and are not included in the 70-pass headline. The unsafe clipping
case is not executable and therefore is not in this table.

| Test source | Ignored test | What it preserves |
|---|---|---|
| `tests/linear_algebra.rs` | `exactcore_matrix_determinant_loses_column_swap_sign_and_position` | Bareiss pivot counterexample |
| `tests/geometry_2d.rs` | `exactcore_coincident_lines_report_disjoint_dimension` | Coincident line dimension defect |
| `tests/geometry_2d.rs` | `exactcore_line_member_orientation_does_not_match_free_orientation2d` | Reversed member orientation |
| `tests/geometry_2d.rs` | `exactcore_circle_line_distance_is_not_a_signed_boundary_distance` | Secant distance clamping |
| `tests/geometry_3d.rs` | `exactcore_identical_planes_do_not_report_coincidence` | Equal-plane false negative |
| `tests/geometry_3d.rs` | `exactcore_transverse_origin_planes_report_wrong_dimension` | Transverse-plane false coincidence/dimension |
| `tests/geometry_3d.rs` | `exactcore_disjoint_coplanar_triangle_and_segment_report_intersection` | Integer `-1` converted to true |
| `tests/geometry_3d.rs` | `exactcore_disjoint_triangles_report_intersection` | Triangle boolean false positive |
| `tests/polynomial.rs` | `univariate_resultant_matches_the_standard_sylvester_orientation` | Resultant orientation convention |
| `tests/polynomial.rs` | `exactcore_sturm_counts_three_simple_integer_roots` | Missing cubic root count |
| `tests/polynomial.rs` | `constant_polynomial_resultant_uses_the_standard_convention` | Constant resultant base case |
| `tests/polynomial.rs` | `polynomial_discriminant_matches_the_standard_definition` | Sign/leading-coefficient normalization |
| `tests/triangulation.rs` | `duplicate_point_preconditions_match` | Duplicate-coordinate precondition mismatch |
| `tests/curves.rs` | `core_circle_distance_cannot_distinguish_secant_from_tangent` | Circle overlap information loss |
| `tests/curves.rs` | `degenerate_line_segment_constructor_contracts_match` | Zero-length segment contract |
| `tests/meshes.rs` | `off_plane_triangle_point_containment_matches` | Missing coplanarity requirement |
| `tests/meshes.rs` | `outside_point_on_edge_support_line_is_not_contained` | Infinite edge-support false containment |
| `tests/meshes.rs` | `disjoint_triangle_boolean_intersection_matches` | High-level boolean false positive |
| `tests/paths.rs` | `contained_segment_boundary_intersection_contracts_match` | Signed metric vs. boundary-hit contract |

Several manifest rows correspond to more than one ignored test because the same
root defect has distinct externally visible consequences. Conversely, adapted rows
can have an ignored test that documents the unadapted convention without making
the adapted comparison a failure.

## Test evidence inventory

| Target | Passing | Ignored | Main evidence |
|---|---:|---:|---|
| Library smoke test | 1 | 0 | Native archive links; a rational oracle call executes |
| Coverage/report manifest | 4 | 0 | All crates, rows, ABI symbols and report entries are mechanically complete |
| Scalars | 7 | 0 | Exact tables, invalid domains, 256 rational property cases |
| Linear algebra | 7 | 1 | Complex/vector/matrix tables and 128 vector/matrix property cases |
| 2D geometry | 9 | 3 | Relation tables, degeneracies, witnesses, distances, 256 orientation/segment property cases |
| 3D geometry | 7 | 4 | Relation tables, witnesses, distances, 256 orientation property cases |
| Polynomials | 10 | 4 | Identities, resultants, GCD/square-free, isolation and 128 evaluation property cases |
| Triangulation | 5 | 1 | Exhaustive small sets, degeneracies and 96 randomized sets |
| Curves feature | 7 | 2 | Line and supporting-circle public API dispatch |
| Meshes feature | 6 | 3 | Plane/convex-triangle public APIs; unsafe clip excluded |
| Paths feature | 7 | 1 | Line paths and full-circle boundary public APIs |
| **Total** | **70** | **19** | 89 named tests, plus doc-test discovery |

The suite favors small exact integer/rational cases because they make a mismatch
unambiguous and reproducible. Property generators expand sign, scale, ordering and
degeneracy coverage but do not constitute exhaustive input enumeration. The passing
test count includes the report completeness test added with this document and is
measured with one test worker because parallel exactCore calls are unsafe.

## Native oracle inventory

All 54 declared C ABI entry points are mapped to at least one nonblocked manifest
row and are referenced by both test and benchmark source. They are grouped as:

- Rational (4): `ec_rational_binary`, `ec_rational_unary`,
  `ec_rational_compare`, `ec_rational_parts`.
- `Expr` (5): `ec_expr_unary`, `ec_expr_binary`, `ec_expr_constant`,
  `ec_expr_sign_unary`, `ec_expr_floor_ceil`.
- Complex/vector/matrix (10): `ec_complex_binary`,
  `ec_complex_norm_squared`, four vector calls and four matrix calls.
- 2D geometry (12): orientation, area, betweenness, line relation/witness,
  segment relation, incircle, circle relation/distance, and three metric calls.
- 3D geometry (10): orientation, volume, line/segment/plane/triangle relations,
  and four metric calls.
- Univariate polynomial (10): evaluation, binary evaluation, derivative,
  resultant, discriminant, GCD degree, square-free degree, total/interval root
  count, and isolation.
- Bivariate polynomial (2): evaluation and resultant specialization.
- Delaunay (1): the `dt4` lower-hull port.

This ABI is intentionally narrow. It is evidence plumbing, not a proposed stable
binding for exactCore.

## Benchmark inventory and interpretation

| Benchmark target | Feature | Paired cases | Surface |
|---|---|---:|---|
| `comparable` | default | 139 | Scalars, linear algebra, 2D/3D geometry, polynomial algebra/root work, Delaunay |
| `curves` | `curve` | 9 | Finite lines and supporting circles |
| `meshes` | `mesh` | 5 | Planes and safe convex-triangle operations |
| `paths` | `path` | 7 | Line paths and full circles |
| **Total** | all features | **160** | **Each case has adapter exactCore, retained exactCore, and Hyper functions** |

After the inventory audit, all 160 pairs were measured in two pinned Criterion
campaigns. The original
[`adapter benchmark report`](BENCHMARK_REPORT.md) includes one-shot C-ABI
parsing, native input construction, serialization, and per-operation FFI. Hyper
won 122 of its 123 semantically matched rows. The upgraded
[`retained-native benchmark report`](RETAINED_BENCHMARK_REPORT.md) constructs
exactCore inputs once, batches its native operation loop, and retains native
results; its matched result is 80 Hyper wins and 43 exactCore wins. The large
change demonstrates that the original near-sweep was substantially an
integration-path result, not an intrinsic kernel ranking.

The retained tier is much closer, but a fully controlled kernel campaign must
still account for the following:

- Output value/container construction intrinsic to each native API remains
  timed, and similarly named APIs can return different amounts of evidence.
- Retained real rows compare lazy expression-graph construction. A numeric
  throughput comparison must force both sides to the same precision and
  certification target.
- C++ sources are compiled at `-O3`; Criterion builds Rust in bench/release mode.
  Ordinary `cargo test` debug timings are not comparable.
- Hyper policy/evidence APIs can return richer diagnostic objects than an integer
  exactCore relation code. Benchmarking only the reduced common answer deliberately
  excludes that extra information.
- `dt4` is O(n^4) and returns a cell complex; Hyper algorithms and tie-breaking
  may solve a different output contract. Large-n timing would not be an
  apples-to-apples implementation comparison.
- Unsafe clipping and the comparison-pending `hyperbrep` surface have no benchmark
  and are not represented as zero/slow results.

## What remains outside the pair

Notable exactCore-only or differently shaped facilities include dynamic-size
linear algebra, explicit complex polar construction/argument, the legacy general
`Polygon3d` carrier, direct `Descartes` machinery, algebraic `rootOf`, and numerous
research/demo algorithms. Some can be recreated from Hyper primitives, but no
same named public contract was found and no synthetic comparison was invented.

Notable Hyper-only facilities include its broader real special-function surface;
structural facts and abort/policy-aware operations; N-dimensional orientation and
insphere; rings, rays, spheres/AABBs, homogeneous constructions and rich topology
reports; Bezier, rational Bezier, B-spline and NURBS curves; region and mesh
booleans; path constraints; B-reps; SDF/voxel/scene/physics layers; packing and
evolution; circuit/PCB tooling; part graphs; and a large nonlinear/CAD solver
stack. These are meaningful product capabilities, but they are not evidence
against exactCore's correctness on its smaller surface.

## Recommended action order

1. **Quarantine and repair exactCore 3D ownership first.** Replace raw polymorphic
   `GeomObj*` ownership and local-polygon interior pointers with value types or
   explicit smart ownership; add sanitizers before re-enabling coplanar clipping.
2. **Fix the small, high-confidence geometry bugs.** Plane coincidence,
   `coeffients()`, `Segment2d::setOpen`, circle initialization/copy ownership,
   line predicate ordering/orientation, triangle boolean conversions and finite
   containment all have localized source evidence.
3. **Correct Bareiss pivot accounting** and add determinant/inverse tests that
   require odd/even row and column permutations.
4. **Define algebraic conventions explicitly.** Either implement standard
   resultants/discriminants or rename/document proxy conventions, then repair the
   Sturm counterexample with root-at-bound and sign-variation regression coverage.
5. **Use typed topology results.** Replacing overloaded integers and implicit
   integer-to-bool conversions with enums would eliminate an entire defect class.
6. **Preserve explicit adapters.** Circle separation, cocircular Delaunay output,
   zero-length segments and boundary-hit queries are legitimate contract choices;
   callers need named conversions, not silent equality assumptions.
7. **Add a dedicated `hyperbrep` differential suite.** Build compatibility with
   the current `hypercurve` context/outcome model is restored; finite-line and
   planar-surface tests and paired benchmarks are the remaining comparison work.

## Reproduce and inspect

From this directory:

```console
cargo test --locked --all-features -- --test-threads=1
cargo bench --locked --all-features -- --test
cargo fmt --package exactcore-hyper-comparison -- --check
cargo clippy --locked --all-targets --all-features -- -D warnings
```

To inspect one known safe counterexample, substitute its exact test name:

```console
cargo test --locked --all-features \
  exactcore_matrix_determinant_loses_column_swap_sign_and_position \
  -- --ignored --exact --test-threads=1
```

The expected outcome of a counterexample test is failure until the underlying
contract or implementation is fixed. Do not add an executable clipping test until
the exactCore lifetime defect is repaired and checked under a memory sanitizer.

### Canonical artifacts

- [`coverage/comparable-api.tsv`](coverage/comparable-api.tsv): all comparable
  semantic rows and their oracle/test/benchmark mappings.
- [`coverage/hyper-crates.tsv`](coverage/hyper-crates.tsv): all sibling crate
  dispositions.
- [`tests/coverage_manifest.rs`](tests/coverage_manifest.rs): mechanical coverage
  enforcement.
- [`cpp/exactcore_oracle.h`](cpp/exactcore_oracle.h): the complete native ABI.
- [`cpp/exactcore_oracle.cpp`](cpp/exactcore_oracle.cpp): exactCore adapters and
  operation dispatch.
- [`cpp/exactcore_bench.h`](cpp/exactcore_bench.h) and
  [`cpp/exactcore_bench.cpp`](cpp/exactcore_bench.cpp): retained native benchmark
  fixture ABI and all 160 fixed-input operation fixtures.
- [`src/lib.rs`](src/lib.rs): safe Rust-facing wrapper used by tests/benchmarks.
- [`RETAINED_BENCHMARK_REPORT.md`](RETAINED_BENCHMARK_REPORT.md): upgraded
  retained-native 160-pair comparison and full result appendix.
- [`BENCHMARK_REPORT.md`](BENCHMARK_REPORT.md): historical one-shot adapter-tier
  comparison and full result appendix.
- [`scripts/analyze-criterion.mjs`](scripts/analyze-criterion.mjs): Criterion
  result analyzer and Markdown/TSV/JSON emitter.
- [`README.md`](README.md): concise suite usage and overview.

## Bottom line

exactCore is a capable exact-computation kernel with far more mathematics than a
small predicate library: arbitrary-precision scalar types, lazy algebraic reals,
polynomial elimination/root isolation, linear algebra, exact 2D/3D primitives and
computational-geometry demos. Hyper matches most of the directly shared operations
tested here and extends them into a much larger, modern Rust application stack.

The comparison does not support a blanket “one is correct and the other is not”
judgment. It supports a narrower and more useful conclusion: the shared arithmetic
and predicate kernel is largely compatible, while exactCore's legacy matrix pivot,
polynomial conventions/root counting, and especially 3D geometry ownership and
topology code require explicit remediation before they should anchor production
CAD/mesh workflows. Hyper's differing contracts must likewise remain explicit at
circle, degeneracy, triangulation and boundary-query seams.
