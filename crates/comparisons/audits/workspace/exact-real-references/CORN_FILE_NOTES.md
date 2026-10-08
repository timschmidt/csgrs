# CoRN audit checkpoint

Repository: `rocq-community/corn`, pinned at `ada7c0b497ff15dd67cf7932c6f20e143a2aee2f`. The shallow checkout contains 446 files; the `.v`/`.ml`/`.mli` source corpus is 375 files and 189,930 physical lines. Full coverage is not claimed. The inventory is in `CORN_FILE_INVENTORY.tsv`; a combined first read was discarded because tool output truncated.

## Bounded source coverage

Read completely in bounded, non-truncated output: `reals/faster/ARArith.v` (1,173 lines), `ARbigD.v` (275), `ARbigQ.v` (105), `ARQ.v` (69), `ARexp.v` (352), and `ARsin.v` (383).

## Transfer findings

CoRN’s `AR` layer is a proof-oriented regular-function wrapper over approximate rational carriers. `ARArith` builds operations through explicitly supplied uniform-continuity proofs, uses bounded multiplication based on a first approximation (`AR_b`), and exposes constructive epsilon-sign/apartness predicates. `ARbigD` and `ARbigQ` provide exact integer/dyadic/rational carriers whose approximation is integer shift/division with proved one-unit enclosure. `ARexp` uses a small alternating factorial stream plus power-of-two range reduction and repeated bounded squaring; `ARsin` uses a bounded alternating stream on [-1,1] and repeated triple-angle-style polynomial reconstruction over a power-of-three range. `ApproximateRationals` makes the approximation contract explicit as classes for division/compression/shift/power plus order embeddings; `AQmetric` lifts a dense approximate carrier into a complete regular-function space and exposes a boolean ball test; `ARAlternatingSum` postpones division, chooses a stopping index from an exact numerator/denominator inequality, then splits the error budget between truncation and per-term division. These are semantically relevant but Hyper already has demand-driven dyadic enclosures, exact integer/dyadic carriers, argument reduction, and specialized transcendental kernels. `ARinterval` constructs compact rational endpoint partitions through a uniform grid; `ARsign` exposes tactic-level epsilon search in eight-bit decrements; `ARpi` uses Machin-like arctangent combinations with fixed integer coefficients; `ARcos` derives cosine from a bounded sine half-angle polynomial. The interval and sign layers are proof/API scaffolding rather than scalar storage improvements, and Hyper already has exact pi identities and argument-reduced trig kernels. `ARarctan` adds domain-partitioned transforms (small, midpoint, reciprocal-large) with explicit proof-carrying branch decisions. `ARroot` uses an invariant-preserving integer recurrence, power-of-two normalization, and monotonicity proofs for square roots; its recurrence shape is conceptually close to Hyper’s guarded integer sqrt seeds, but Hyper already has a demand-sized exact-seed path and stronger shared DAG/cache semantics. `ACarith` lifts complex arithmetic and square roots as paired AR values; `ARplot` demonstrates certified raster sampling from compact rational partitions; `ARabs` supplies a uniformly continuous absolute-value lift; `ARSpeedTests` is a single historical timing command; `ARtrans` is only an export aggregator. These are consumer/proof layers rather than scalar representation improvements. No production change is selected from this partial pass.

The entire `reals/faster` directory (21 files / 4,905 lines) is now covered, along with `reals/fast/CRexp.v` (1,053 lines) and `reals/fast/CRroot.v` (1,274 lines). CRroot confirms a proof-oriented square-root design: invariant-preserving Newton recurrence, exact power-of-four scaling in both directions, explicit nonnegativity/monotonicity proofs, and a uniform-continuity modulus based on squared input error. It is useful as a correctness reference, but does not expose a better Hyper scalar representation or runtime path; Hyper already has guarded exact seeds, demand-sized approximation, and shared DAG/cache evaluation. Native Coq qualification is pending until the toolchain is available. The remaining 353 CoRN source files are explicitly pending.


## Core fast-kernel pass

`reals/fast/CRexp.v` (1,053 lines) was read completely in two bounded chunks. It implements exponential through an alternating Taylor stream with exact factorial stopping, splits division error from truncation error, uses repeated compressed squaring for negative range reduction, and handles positive inputs through a certified reciprocal lower bound. It also defines a bounded exponential adapter from a first coarse approximation and a ceiling-based integer domain bound. These contracts align with Hyper’s existing expm1/exp range reduction and explicit precision budgets; the proof-oriented stream and continuity wrappers do not provide a safe scalar replacement. No production change selected.
CRsin.v (817 lines) is now fully covered. It defines sine on [-1,1] with a factorial alternating stream, extends the range through the cubic triple-angle polynomial `3x-4x^3`, chooses the reduction depth from a base-3 logarithmic bound, and applies a final period reduction using a coarse six-pi quotient. The proof explicitly supplies a global continuity modulus and odd symmetry. This is a sound range-reduction reference, but Hyper already has demand-sized trig argument reduction and shared expression evaluation; no production change selected.
CRarctan.v (377 lines) is now fully covered. It partitions nonnegative inputs into small-series, midpoint `(a-1)/(a+1)`, and reciprocal-large branches with fixed 2/5 and 5/2 thresholds, then extends by odd symmetry and supplies a derivative-based global modulus. The branch identities are a useful exactness reference, but Hyper’s arctan implementation already performs demand-driven range reduction and does not need these fixed rational thresholds; no production change selected.
CRabs.v (360 lines) is now fully covered. It lifts rational absolute value through an explicit 1-Lipschitz uniform-continuity proof, defines exact distance and ball equivalences, and uses stable/decidable-order reasoning for sign-sensitive identities. This is semantically useful for partial comparison, but Hyper already has exact abs/distance behavior and shared approximations; no production change selected.
CRarctan_small.v (344 lines) is now fully covered. It uses an alternating odd-power stream on [-1,1], with denominator/state updates carried in the stream and explicit convergence bounds from reciprocal ceilings. The source notes a larger stream state could avoid repeated denominator work; its newer `positive*Q` state already does so. Hyper’s current kernels use demand-sized precision and do not benefit from this proof-oriented stream shape; no production change selected.
CRcos.v (288 lines) is now fully covered. It derives cosine from the half-angle identity `1-2 sin(x/2)^2`, proves a bounded quadratic modulus on [-1,1], then reuses the same coarse six-pi period reduction as sine. This is mathematically clean but introduces no better Hyper scalar or scheduling strategy; no production change selected.
CRpi_fast.v (346 lines) is now fully covered. It builds π from a Machin-like fixed arctangent combination, uses rational Möbius addition with proof-checked denominator/non-domain conditions, and computes repeated angle multiples at compile time. The fixed identity is valuable for constant correctness, but Hyper already has exact π construction and cached/shared constants; no replacement or extra relation cache was justified.
CRln.v (448 lines) is now fully covered. It reduces positive logarithm to `2*artanh((a-1)/(a+1))`, scales by powers of two, and compensates with a shared `ln 2`; a positive lower-bound parameter supplies the uniform-continuity modulus and prevents invalid-domain evaluation. This domain-carrying API is relevant to completeness, but Hyper already carries positivity guards and has demand-driven logarithm reduction; no production change selected.
CRartanh_slow.v (427 lines) is now fully covered. It represents artanh as a positive-ratio geometric series rather than an alternating stream, proving convergence by a ratio test and carrying explicit domain `a²<1`; the source itself notes incomplete development outside logarithm needs. Hyper’s logarithm path already has guarded domains and demand-sized series evaluation, so no production change selected.
CRpower.v (585 lines) is now fully covered. It uses binary natural exponents, derivative-based moduli on bounded intervals, explicit `QboundAbs` clamping, and a bounded-power composition that reuses multiplication while weakening bounds safely. Hyper already uses binary exponentiation, exact structural bounds, and demand-sized evaluation; the CoRN proof wrappers do not justify a runtime change.
CRconst.v (29 lines) is now fully covered. It defines constant uniformly continuous functions with an infinite modulus; this is a proof/API wrapper and does not change Hyper’s constant-node representation. No production change selected.
CRpi_slow.v (311 lines) is now fully covered. It provides an alternate, compile-faster but runtime-slower Machin-like π identity with smaller arctangent arguments and compile-time rational angle combination. The fast/slow tradeoff is explicit, but Hyper’s existing shared exact π node and benchmarks provide no reason to adopt this alternate identity. No production change selected.
CRball.v (126 lines) is now fully covered. It defines balls with real-valued radii and proves equivalence to exact distance bounds, including rational scaling of metric errors. This is a useful API idea for limit/geometry layers, but Hyper’s current enclosure and comparison APIs already carry equivalent bounds; no scalar change selected.
CRsign.v (83 lines) is now fully covered. It provides a tactic that repeatedly halves an epsilon until a certified sign is found, with explicit nontermination for zero/undecidable cases. Hyper’s strict sign queries already fail closed and schedule demand adaptively; no change selected.
CRpi.v (12 lines) is now fully covered. It exports the fast π implementation and proves positivity through the sign tactic; no runtime representation change selected.
CRGeometricSum.v (1,122 lines) is now fully covered. Its central performance technique is `InfiniteSum_raw_N`: logarithmic unary fuel (`2^n` iterations) plus continuation composition, avoiding large normal forms in `vm_compute` while preserving the same early-stop filter as the straightforward recursive sum. It also proves geometric tail error bounds and monotone maximum iteration estimates. Hyper’s evaluator already uses explicit stacks/frames for deep expression traversal; its transcendental kernels use demand-sized iterative accumulation, so this CoRN continuation/fuel shape is a candidate reference but no measured production change is selected yet.
CRAlternatingSum.v (1,114 lines) is now fully covered. It uses alternating-term magnitude as an immediate error bound, explicit stopping filters, a three-component state carrying current term and reduced partial sum, and `AltSeries_shift` to peel terms without rebuilding the whole stream. It also imports `LazyNat` and documents logarithmic-fuel concerns in the paired geometric engine. Hyper’s evaluator already has explicit demand/error scheduling and iterative frames, but term-peeling and continuation reuse remain useful candidates for future series-kernel profiling; no unmeasured production change selected.
CRIR.v (404 lines) is now fully covered. It establishes the CR↔IR isomorphism, transports arithmetic/order/apartness and Cauchy limits, and maintains rewrite lemmas for exact normalization between representations. This is primarily semantic glue: Hyper keeps one native scalar representation and avoids such cross-model transport overhead; no production change selected.
CRGroupOps.v (725 lines) is now fully covered. It lifts rational translation/negation, defines nonnegative-based order, and implements max/min through 1-Lipschitz rational clamps with explicit bound proofs; curried and uncurried addition forms are related by metric-product maps. Hyper’s native DAG and exact sign/bound shortcuts already avoid CoRN’s repeated `Qmax/Qmin` clamping and transport wrappers, so no production change selected.

CRFieldOps.v (1,695 lines) is now fully covered. It defines the field-level constructive-real operations and proof obligations: positive lower-bound witnesses (`CRpos`), strict order/apartness, exact rational scaling moduli, bounded multiplication via `CR_b`/`QboundAbs`, inverse moduli with guarded positive branches, safe bound weakening through `QreduceApprox`/`faster`, and associativity transport for bounded products. The runtime strategy repeatedly obtains coarse first approximations, clamps magnitudes, and weakens requested errors to preserve uniform-continuity contracts. Hyper already carries structural bounds and exact rational/algebraic shortcuts in its native DAG, so importing these proof wrappers would add work without improving measured exactness or cost. The rescaling idea is already represented by Hyper’s precision-aware cache/evaluator; no production change selected.


CRArith.v (2,354 lines) is now fully covered. It establishes the CR ring/field instances, rational embedding, order/apartness and partial-decision interfaces, ball/order equivalences, sign search (`CR_epsilon_sign_dec`), monotonicity/cancellation, and positivity of products. The central runtime pattern is proof-oriented: operations are wrapped in uniform-continuity obligations, multiplication uses coarse `CR_b` bounds and `QboundAbs`, inverse uses sign witnesses and guarded lower bounds, and comparisons use DN/searchable epsilon refinement. Hyper already has native exact rational/algebraic paths, structural bounds, and fail-closed demand-driven comparison; adopting these wrappers would increase code and proof overhead without improving scalar exactness or measured runtime. The epsilon-sign thresholding and explicit partiality remain useful semantic references only; no production change selected.

CRArith_alg.v (101 lines) is now fully covered. It supplies algebraic adapters for rational products, reduced-rational apartness, strict-extensionality of the rational embedding, and the ring-homomorphism instance. These are compile-time/setoid glue around already exact rational injection; Hyper’s direct rational node constructors make an equivalent adapter unnecessary. No production change selected.

CRtrans.v (34 lines) is now fully covered. It is only a re-export aggregator for CR arithmetic, compression, powers, roots, exponentials, logarithms, trigonometric functions, π, arctangent, and absolute value. It contains no scalar or evaluator implementation; no production change selected.

Compress.v (206 lines) is now fully covered. CoRN’s `compress_raw` doubles requested precision, then rounds the resulting rational by integer division to a bounded numerator/denominator; the proof spends the extra error budget to preserve the ball contract and exposes compression as an identity-equivalent uniformly continuous function. This is useful when arbitrary rational numerators/denominators dominate storage, but Hyper’s approximation carrier is already dyadic and precision-indexed, so each result is naturally denominator-normalized. Adding a second rational compression pass would cost arithmetic and code without a demonstrated memory or binary-size win; no production change selected.

PowerBound.v (135 lines) is now fully covered. It derives logarithmic integer exponents from `Pos.size`/`Z.log2` to bound rationals by powers of 3 and 4, then proves reciprocal lower bounds for positive inputs. The approach is an exact bit-length planning primitive for range reduction, but Hyper already uses BigInt bit lengths and demand-sized power-of-two scaling; no production change selected.

CRsum.v (236 lines) is now fully covered. `CRsum_list_raw` distributes a total error budget across list terms using a denominator based on list length, then performs a left fold of independently approximated terms; proofs propagate the accumulated error and establish equivalence to ordinary CR addition. This is straightforward but has linear-depth fold structure and no coefficient/term sharing. Hyper’s explicit evaluator frames and specialized series kernels already avoid recursive list walks and schedule term precision directly; no production change selected.

uneven_CRplus.v (75 lines) is now fully covered. It allows addition to split an error budget by arbitrary positive weights (for example, proportional to interval widths) while proving equivalence to ordinary equal-split addition. This could reduce work when operand difficulty or downstream conditioning is strongly asymmetric, but Hyper’s evaluator currently evaluates both Add children at the same guarded precision and only orders them by `linear_demand`; introducing dynamic weighted precision would require a new enclosure proof and benchmark evidence. It remains a candidate, not a retained change.

LazyNat.v (119 lines) is now fully covered. It represents natural-number bounds as a thunked successor chain, with lazy predecessor, addition, and positive-times-power-of-two construction so huge bounds do not normalize eagerly. This is relevant to proof-level upper-bound generation, but Hyper’s runtime uses bounded integer precision and iterative demand frames rather than symbolic unary fuel; importing a lazy-unary type would add allocations and indirection. No production change selected.

ContinuousCorrect.v (179 lines) is now fully covered. It proves equality of an IR continuous function and a CR implementation by density of rationals in compact intervals, intersecting the function and CR moduli, and a final ball triangle argument. This is a semantic completeness/correctness theorem rather than an evaluator optimization; Hyper’s native scalar model does not carry a second IR representation, so no production change selected.

ModulusDerivative.v (293 lines) is now fully covered. It derives a uniform-continuity modulus from an absolute derivative bound on a clamped interval, using the mean-value inequality and rational-density agreement between IR and CR functions. The domain clamp and derivative-based Lipschitz bound are valuable correctness patterns, but Hyper’s elementary kernels already carry explicit moduli and guarded domains directly in their approximation schedules; no production change selected.

CRAlternatingSum_alg.v (381 lines) is now fully covered. It proves convergence and remainder bounds for alternating streams, normalizes negative-first-term cases by sign transformation, and uses explicit stream iteration with a modulus-driven stop index. The algorithm separates first-term/small-tail reasoning from the final remainder certificate, but does not add memoization or a faster numeric kernel. Hyper’s iterative demand-sized series paths already preserve these contracts without stream/list overhead; no production change selected.

CRstreams.v (820 lines) is now fully covered. It supplies coinductive stream bounds, reciprocal/factorial streams, and a binary-positive `iterate` operator that advances a state in logarithmic recursion depth. Its `iterate_stop` implementation explicitly notes a performance defect: recursive stopping repeats logarithmically many stop tests and could return `(state, stopped)` to avoid them; LazyNat fuel is another option. Hyper’s production series kernels already use iterative state and stop checks rather than this recursive stream encoding, so no direct change was selected. The pair-returning stop flag remains a candidate for any future stream/IVP subsystem.

CRcorrect.v (979 lines) is now fully covered. It constructs and proves the CR↔Cauchy-sequence isomorphism, transports equality/order/apartness, and establishes preservation of addition, bounded multiplication, and guarded inversion. The implementation repeatedly chooses max indices for multiple Cauchy witnesses and spends explicit ball budgets; this is semantic proof infrastructure rather than a faster scalar representation. Hyper’s single native DAG avoids these cross-representation max-index and transport costs, so no production change selected.

Integration.v (984 lines) is now fully covered. It constructs effective integration from balanced rational step functions, chooses sample depth from a certified sup-distance bound, distributes interval error by subrange widths, and lifts continuous composition through bounded/L-infinity metrics. The balanced tree and width-proportional error split are useful future Hyperlimit integration patterns, but Hyper has no general exact integrator consumer yet; importing this proof-heavy StepF/L-infinity stack would add substantial code and allocation overhead. No production change selected.

MultivariatePolynomials.v (1,578 lines) is now fully covered. It represents n-variable polynomials recursively over constant-polynomial rings, evaluates each variable once, bounds unit-hyperinterval values with Bernstein coefficients, derives derivative-based moduli, and uses a conservative coefficient-sum bound for recursive continuity/evaluation proofs. The Bernstein upper/lower bound and sparse recursive coefficient representation are directly relevant to `hypersolve`; Hyper already has exact integer/rational Bernstein sign counts, common-positive-scale subdivision, and persistent polynomial identities, while this CoRN layer is proof-heavy and allocates nested vectors/setoids. No additional production change selected.

RasterQ.v (343 lines) is now fully covered. It maps well-formed boolean rasters to finite rational pixel-center sets over uniform partitions, proving index bounds, membership, coordinate formulas, and metric/setoid stability. It explicitly leaves sparse-raster support as a TODO and uses list/filter/flat_map traversal; Hypercurve/Hypertri use more direct exact geometry and do not gain a scalar or memory improvement from this representation. No production change selected.
Interval.v (711 lines) is now fully covered. It constructs compact rational intervals from binary-uniform partitions, with exact rasterization bounds, endpoint handling, and located-subset proofs. The partition enumerator is balanced over binary positive indices and gives a half-step witness; compact approximations use a ceiling-derived point count and clamp approximants for correctness. This is useful as a Hyperlimit/geometry sampling reference, but Hypercurve/Hypertri already use direct exact primitives and adaptive demand-driven evaluation; replacing them with materialized uniform lists would increase memory and lose adaptivity. No production change selected.

Plot.v (438 lines) is now fully covered. It builds exact graph plots by composing clipping, compact interval approximations, uniform sampling, and rasterization; the alternate slow definition proves equivalence to the fused path. Located-subset plotting recursively scans pixels and certifies blank cells, but uses non-tail-recursive list construction and materialized rasters. Hypercurve/Hypertri already retain exact adaptive predicates and avoid raster materialization, so no scalar, performance, or memory change was worthwhile.

RasterizeQ.v (583 lines) is now fully covered. It clamps rational points into a raster using min/max index arithmetic, proves two-way Hausdorff-style inclusion, and offers a sparse raster list representation to reduce storage when lit pixels are few. However, sparse entries still allocate two positive coordinates per point and PixelizeQ2 folds through a materialized list. Hypercurve/Hypertri’s exact predicates and Hyperlimit’s adaptive evaluation do not benefit from this raster layer; no production change selected.


Small CoRN support files `CRing_as_Ring.v` (7), `N.v` (7), `Container.v` (8), `Ranges.v` (19), `Qclasses.v` (21), and `Qposclasses.v` (23) are fully covered. They provide proof/typeclass adapters, descending natural enumeration, generic membership notation, min/max range predicates, and rational positive-number instances. These are compile-time interfaces or tiny list helpers; Hyper already has native scalar traits and bounded integer/rational carriers, so no production change selected.


Compact CoRN files `Rsign.v` (25), `CornScope.v` (28), `Pair.v` (29), `PropDecid.v` (30), and example `bigD.v` (32) are fully covered. They contain tactic wrappers, notation scopes, pair/list combinators, propositional decidability, and a benchmark-only dyadic example. None changes Hyper’s runtime scalar architecture; the dyadic example reinforces that benchmark code should stay out of production. No production change selected.


Compact.v (1,824 lines) is now fully covered. It defines compact sets as completions of finite enumerations, proves equivalence with Bishop compactness, constructs geometric-improvement streams with binary-log convergence indices, and maps compact images under uniformly continuous functions. The key runtime ideas are precision-indexed finite approximants, geometric contraction, and explicit unknown/reverse-image limits; Hyperlimit already uses demand-driven approximation and Hyper’s node cache, while this implementation repeatedly materializes lists and carries proof witnesses. Compact image reverse inclusion is explicitly incomplete without sequential compactness. No production change selected, though the geometric contraction/index pattern remains a reference for future Hyperlimit compact operators.


CPolynomials.v (2,786 lines) is now fully covered. It defines recursive coefficient-first univariate polynomials, proves setoid/ring laws, Horner evaluation, derivatives, ring-hom mapping, and a `cpoly_mult_fast` constructor that trims multiplication by a zero polynomial to avoid trailing zero nodes and improve degree estimates. Hyperlimit/hypersolve currently use fixed-size lifted predicates and dense/sparse tensor structures rather than this generic recursive ring; their exact zero filters and persistent identities already avoid the relevant trailing-zero cost. No production change selected.


Series.v (1,341 lines) is now fully covered. It formalizes Cauchy/convergent and divergent series, comparison and ratio tests, alternating-series remainder bounds, geometric/power series, and constructions of e and pi. Error reasoning is semantic/proof-level over infinite sequences; it does not provide demand-driven kernels or sharing. Hyperreal’s iterative series evaluators already use explicit precision budgets, guarded domains, and abort-safe caches, so no production change selected. The alternating/geometric remainder inequalities remain useful as proof references for future kernel audits.


Exponential.v (1,351 lines) is now fully covered. It proves exact exponential/logarithm identities, positivity and monotonicity, derivatives, uniqueness, and algebraic laws using Taylor-series convergence and interval-domain continuity. The implementation is theorem-level and delegates actual approximations to earlier Taylor/CR layers; it has no demand-aware cache or argument-reduction scheduler. Hyperreal’s exp/log kernels already provide guarded domains, range reduction, precision-aware series, and cache-safe evaluation, so no production change selected.


InvTrigonom.v (1,274 lines) is now fully covered. It defines inverse sine/cosine/tangent as indefinite integrals, proves domains/ranges/inverse laws and monotonicity, and uses IVT plus elaborate interval witnesses for arctangent range existence. The construction is proof-heavy and non-constructive at runtime; it does not expose practical argument reduction or precision scheduling. Hyperreal’s inverse-trig kernels already use exact domain guards, range reduction, series thresholds, and abort-safe caching, so no production change selected.


Integral.v (1,498 lines) is now fully covered. It defines integrals as limits of even-partition Darboux sums, proves Cauchy convergence from continuity moduli, partition-join/additivity, linearity, monotonicity, norm bounds, strict positivity, and interval integral estimates. The architecture uses explicit mesh/modulus coupling and partition joining, but relies on proof-level real functions and repeated finite sums rather than executable demand-driven quadrature. Hyperlimit currently has no general integrator; these mesh/error inequalities are a useful future design reference, but no immediate production change was justified.


COrdFields.v (1,656 lines) is now fully covered. It defines constructive ordered-field axioms, strict/apartness order conversion, cancellation and shift laws, positivity/division lemmas, characteristic-zero facts, and finite-sum sign propagation. The key semantic pattern is partial order derived from apartness plus strict comparison, with explicit nonzero witnesses for division. Hyperreal’s structural sign/zero cascade and fail-closed unresolved comparisons already implement the runtime analogue; no production change selected.


Limit.v (423 lines) is now fully covered. It defines lazy existential stream termination, continuation-passing `takeUntil` (avoiding intermediate lists), length/termination proofs, and coinductive `NearBy`/`Limit` predicates stable under stream tails. The continuation-passing pattern is memory-conscious, but Hyperlimit uses imperative/iterative demand loops rather than Coq streams; no production change selected. It remains a useful reference for stack-safe early-stop combinators.


Complete.v (1,233 lines) is now fully covered. It defines regular-function completions, metric/coherence proofs, constant injection, completion map/join/bind operations, monad laws, faster approximation wrappers, and locatedness transport. The strongest implementation ideas are precision-indexed approximants, explicit half/quarter error splitting, and a `faster` wrapper that preserves metric equality while tightening approximations. Hyperreal already uses precision-indexed approximation caches and rescaling; Hyperlimit’s demand loops likewise split guarded budgets explicitly. No production change selected.
### `reals/Bridges_LUB.v` (1,282 lines)

Constructs least upper bounds for inhabited bounded-above located predicates by a bisection-like interval sequence. `dcotrans_analyze` chooses a left witness or right upper bound using locatedness; interval width shrinks geometrically by 2/3. Monotone left/right endpoints and midpoint Cauchy bounds yield the limit, with Archimedean index selection proving convergence and leastness. This is proof-level completeness/partial-comparison machinery; Hyperreal already uses executable precision refinement and Hyperlimit iterative stopping, so no worthwhile implementation change selected.
### `model/reals/Cauchy_IR.v` (51 lines)

Defines `Cauchy_IR` by instantiating the generic `R_as_CReals` construction with the rational ordered field and its Archimedean proof. The file is only a model alias; all representation and operations live in imported modules. No direct Hyperreal transfer.
### `metric2/Classification.v` (52 lines)

Defines located and decidable metric-space predicates. A decidable ball relation implies locatedness by comparing a stricter radius and weakening the ball bound. This formalizes the partial-distance decision contract but adds no executable strategy beyond Hyperreal’s interval/precision predicates; no change selected.
### `model/metric2/BoundedFunction.v` (40 lines)

Defines bounded functions as the completion of `LinfStepQ`, then lifts the supremum operator through a uniformly continuous map (`Cmap`). This is a concise completion/combinator pattern; Hyperlimit’s existing iterative convergence and Hyperreal caches provide the operational analogue, so no change selected.
### `model/metric2/IntegrableFunction.v` (46 lines)

Defines integrable functions as the completion of `L1StepQ`, lifts the integral via `Cmap`, and embeds bounded functions through the uniformly continuous `L∞→L1` map. It is completion/type glue with no new scalar algorithm or storage strategy; no change selected.
### `model/partialorder/CRpartialorder.v` (39 lines)

Packages the existing constructive-real order relation `CRle` and monotonicity/antitonicity lemmas into a generic `PartialOrder`. It contributes no representation, evaluator, or comparison scheduling mechanism beyond imported `CRGroupOps`; no Hyper change selected.
### `metric2/UCFnMonoid.v` (36 lines)

Instantiates identity and composition for uniformly continuous endomaps and proves the monoid laws through the continuity-order relation. This is algebraic typeclass infrastructure only; it suggests no scalar evaluation, caching, or precision improvement for Hyper.
### `model/semigroups/Qpossemigroup.v` (46 lines)

Builds the constructive semigroup instance for positive rationals under multiplication from the setoid and associativity proofs. It is a one-definition algebraic adapter with no exact-real runtime or performance idea; no Hyper change selected.
### `model/semigroups/QSpossemigroup.v` (47 lines)

Builds a constructive semigroup on positive rationals using the `xy/2` operation and its associativity proof. This is a scaling-specific algebraic adapter, not an exact-real evaluator or storage strategy; no Hyper change selected.
### `model/reals/CRreal.v` (222 lines)

Defines CoRN’s concrete Cauchy-real model. `CRAbsSmall_ball` bridges regular-function absolute-smallness and metric balls using `doubleSpeed` and quarter-precision approximations. `CRlim` converts a Cauchy sequence to a regular function with a `QposInfinity` sentinel and per-request convergence indices; `CRisCReals` proves completeness and Archimedeanness. The explicit precision split and lazy `Cjoin` are useful semantic references, but Hyperreal already has precision-aware node evaluation and cache rescaling; no worthwhile change selected.
### `raster/Raster.v` (224 lines)

Represents an `n×m` Boolean raster as nested lists, explicitly avoiding Coq vectors because each cons would store a slow `nat`. It proves well-formedness, indexing, and functional pixel updates with overflow/no-op behavior. The representation warning is relevant to Hyper geometry memory layout, but this file has no scalar algorithm and Hyper’s existing packed/structured geometry paths are preferable; no change selected.
### `ode/BanachFixpoint.v` (218 lines)

Develops a contraction iteration `x n = fⁿ x₀`, proving geometric distance bounds and Cauchy convergence. The stopping schedule uses an explicit rational ceiling of `d/(e(1−q)^2)` (with a separate zero-initial-distance path), then obtains the fixed point through the generic limit operator. This is a useful `hypersolve` design reference for contraction-aware iteration and certified iteration counts, but HyperSolve currently has no matching contraction API and no isolated benchmark justified adding one; no change selected.
### `metric2/UniformContinuity.v` (324 lines)

Defines metric balls over `QposInf` (with infinity as an always-true bound), explicit uniform-continuity records carrying a modulus, extensional equality, and compositional modulus propagation via `QposInf_bind`. The infinity sentinel and modulus composition are useful contracts for partial precision, but Hyperreal’s node bounds are structural and its evaluator already propagates requested precision; no measurable or exactness improvement selected.
### `reals/OddPolyRootIR.v` (328 lines)

Proves odd-degree polynomial roots over constructive reals. It establishes eventual positivity of monic polynomials, uses coefficient/sign-flipping to obtain a negative-side witness, normalizes a nonzero leading coefficient to monic form, and applies the polynomial IVT to produce a root. The result is existential/proof-level; it supplies no root-isolation algorithm or precision scheduler beyond imported IVT, so HyperSolve’s explicit algebraic isolation remains more operational and no change was selected.
### `ftc/IntegrationRules.v` (331 lines)

Proves integration by substitution and the `[0,1]` affine-rescaling rule under compact inclusion, continuity, and derivative hypotheses. The proof handles reversed/degenerate intervals explicitly and transports endpoint bounds through compact maps. It is theorem-level integration infrastructure with no executable quadrature, adaptive partition, or error estimator; Hyperlimit’s numerical integration remains operationally distinct, so no change selected.
### `model/metric2/LinfDistMonad.v` (337 lines)

Constructs a distributive map between `StepFSup (Complete X)` and `Complete (StepFSup X)` by mapping each completed value through requested approximations. It proves uniform continuity, constant/glue compatibility, and several monad-distribution laws; one difficult law is explicitly left unproved because `glue` does not compose cleanly with `join`. The repeated quarter-precision splits and structural map traversal are useful for cache/evaluation design, but Hyper’s node graph avoids this nested list/monad representation and no tested transfer was identified.
### `metric2/MetricMorphisms.v` (329 lines)

Defines metric embeddings and dense embeddings with an approximate inverse, then proves prelength transfer and an isomorphism between completions. The `app_inverse` API carries a requested positive precision; completion maps use explicit half/quarter error splits, and unary/binary function transport preserves uniform-continuity moduli. This is a strong abstraction reference for exact adapters, but Hyper’s scalar representation is not a completion wrapper and no measurable transfer was identified.
### `model/structures/Qsec.v` (357 lines)

Defines CoRN’s rational structure over integer/positive-denominator pairs, including apartness, tightness, sign and order decisions, inverse laws, integer injection, an Archimedean bound, and duplicate-aware list NoDup. Structural sign decisions and explicit Archimedean witnesses align with Hyperreal’s exact rational fast paths, which already use a more compact/canonical backend; no worthwhile change selected.
### `reals/stdlib/CMTcast.v` (353 lines)

Transports constructive-real values, partial functions, Riesz spaces, and integration spaces between real implementations through `SlowConstructiveRealsMorphism`. It proves preservation of arithmetic, absolute/min operations, limits, and integrability, but repeatedly composes morphisms and carries extensive domain proofs. Hyperreal has one canonical scalar backend, so there is no analogous cast overhead to remove and no worthwhile change selected.
### `model/Zmod/IrrCrit.v` (345 lines)

Despite its name, this file proves an integer-polynomial irreducibility criterion by reducing coefficients and products modulo a fixed prime field. It establishes well-defined coefficient translation, degree/monicity preservation, and reducibility reflection; it contains no real-number irrationality or approximation logic. The finite-field homomorphism pattern is not applicable to Hyper’s scalar runtime, so no change selected.
### `model/structures/OpenUnit.v` (205 lines)

Represents rational values strictly in `(0,1)` with proof-carrying bounds. It defines multiplication, constrained division, dual `1−a`, dual multiplication/division, and strict affine combinations, including identities for nested affine interpolation. These operations provide exact interval-splitting contracts, but Hyperlimit already uses explicit rational interval refinement and no measurable improvement was identified.
### `reals/Cesaro.v` (209 lines)

Proves that weighted Cesàro means preserve constructive-real convergence. The proof splits the error into a finite-prefix term and a tail term, uses positive weights and an infinite-sum bound, and chooses a maximum of two convergence indices. The finite-prefix/tail decomposition could inform sequence acceleration, but Hyperlimit has no weighted-series API and no benchmark supports adding one; no change selected.
### `model/structures/QnonNeg.v` (213 lines)

Defines nonnegative rationals as a proof-carrying subtype, factors addition/multiplication/min/max through a generic `binop`, and supplies inverse, coercion, case analysis, and induction helpers. The file notes that nonnegative proof terms lack the uniqueness enjoyed by strict-positive proofs, complicating dependent rewriting. Hyperreal keeps bounds as runtime data rather than proof-carrying scalar types, so no worthwhile change selected.
### `metric2/StepFunctionMonad.v` (358 lines)

Defines the step-function monad’s `bind`, `join`, and applicative operations over glued interval trees. Recursive bind pushes split maps into both branches and proves monad laws, with extensional/setoid morphisms preserving equality. The representation exposes potential recursion depth and repeated glue traversal; Hyper’s evaluator is iterative and graph-based, so no direct transfer was worthwhile. A related distribution law is explicitly omitted because glue and join interact poorly.
### `model/metric2/Qmetric.v` (508 lines)

Defines CoRN’s rational metric as `QAbsSmall e (a−b)`, proves metric/prelength/locatedness laws, and supplies exact ball arithmetic for addition, multiplication, division, and negation. It includes a boolean `Qball_ex_bool` decision over `QposInf` plus correctness and the exact-zero equivalence `Qball 0 a b ↔ Qeq a b`. Hyperreal already has structural rational filters and exact-zero handling with a richer canonical backend; no worthwhile change selected.
### `reals/IVT.v` (551 lines)

Develops constructive nested-interval and intermediate-value machinery. Monotone endpoint sequences form a Cauchy sequence; the limit lies between all endpoints. The operational bisection uses `lft=(2a+b)/3`, `rht=(a+2b)/3`, retaining a sign-changing subinterval and shrinking width by `2/3` per step. Explicit Archimedean index selection proves termination, and polynomial IVT is derived from continuity plus an apartness witness. The 2/3 split avoids requiring a decidable midpoint sign, but Hyper’s exact predicate/refinement pipeline already handles unresolved midpoint signs; no change selected without a benchmark showing improved scheduling.
### `complex/AbsCC.v` (509 lines)

Defines the constructive complex norm as `sqrt(Re²+Im²)` and proves nonnegativity, strict positivity from apartness, multiplicativity/division, conjugate identities, powers, strong extensionality, and the triangle inequality (including finite sums). It uses exact square-root side conditions and converts norm-smallness into complex equality. This is relevant only to a future complex Hyper layer; current Hyper crates use real scalars and no change was selected.
### `reals/CReals1.v` (513 lines)

Adds constructive-real Cauchy properties: absolute values preserve limits, sequences are bounded, monotone subsequences preserve limits, and powers tend to zero/infinity under rational bounds. Proofs use explicit `ε/2` splits, Archimedean ceilings, and exact positivity/apartness. These are semantic convergence lemmas rather than executable evaluators; Hyperreal already has structural absolute/power paths, so no worthwhile change selected.
### `model/structures/NNUpperR.v` (521 lines)

Defines nonnegative upper cuts over `QnonNeg`, carrying a witness bound and closedness, with order based only on non-lowest upper bounds. Generic monotone “binop” construction supports addition/multiplication and uses a `sneaky` approximation lemma to prove order compatibility; it derives a semiring and models `sqrt2` from arbitrary rational square over-approximations. This one-sided enclosure architecture is relevant to completeness, but Hyperreal stores two-sided certified intervals and exact symbolic structure; no benchmark-backed replacement selected.
### `algebra/COrdCauchy.v` (763 lines)

Defines ordered-field Cauchy sequences and proves closure under constants, addition, negation, multiplication, and guarded reciprocal sequences. Multiplication derives explicit sequence bounds and splits the target error with conservative factor-12 constants; reciprocals remain constant until a positive lower bound is established. It also proves local/global monotonicity, injectivity, and continuity moduli for field operations. Hyperreal already has guarded inverse domains and structural monotonicity, while its precision evaluator is more direct; no worthwhile change selected.
### `transc/TrigMon.v` (730 lines)

Proves exact sign, monotonicity, and derivative properties for sine, cosine, and tangent over bounded Pi intervals. It derives positivity from continuity, uses derivative-integral arguments for strict monotonicity, handles periodic nonzero criteria for zeros, and establishes tangent derivatives with explicit cosine-away-from-zero bounds. These are proof-level domain/identity results; Hyperreal’s trig kernels already perform argument reduction and directed approximation, so no worthwhile change selected.
### `metrics/CMetricSpaces.v` (759 lines)

Builds constructive metric spaces from pseudo-metric spaces by quotienting points at zero distance, and proves product/subspace constructions, metric extensionality, and unique limits. The quotient relies on apartness/tightness and transports distance operations through setoid morphisms; limit uniqueness uses an Archimedean contradiction. This is semantic metric infrastructure rather than a scalar evaluator or cache strategy, so no Hyper change selected.
### `ftc/FunctSeries.v` (774 lines)

Defines convergent series of continuous functions on compact intervals through Cauchy partial sums, then derives pointwise sums, arithmetic operations, absolute convergence, comparison, and a uniform ratio test. Error proofs use explicit finite-prefix/tail and geometric-power bounds, while continuity is transported from partial sums. This is proof-level uniform-convergence infrastructure with materialized sums; Hyperlimit has no equivalent function-series runtime, so no change selected.
### `transc/ArTanH.v` (545 lines)

Defines inverse hyperbolic tangent on `(-1,1)` as `½·log((1+x)/(1−x))`, proves exact domain equivalence, continuity, derivative `1/(1−x²)`, oddness, and zero value, then derives a power-series representation from logarithm series with compact-domain mapping proofs. Hyperreal already has dedicated inverse-hyperbolic kernels and domain guards; this identity/series formulation offers no measured improvement, so no change selected.
### `metric2/Hausdorff.v` (556 lines)

Defines weak and strong Hausdorff-ball relations over predicate subsets of a metric space. The weak form uses classical `existsC` witnesses; the strong form carries constructive witnesses with an extra positive slack, enabling triangle composition by splitting slack in half. It proves setoid respect, reflexivity, symmetry, and triangle laws, plus (mostly commented) conversions between strong/weak forms under finite enumeration and located metrics, and almost-decision procedures for finitely enumerable subsets. This is metric-set infrastructure with substantial classical/proof machinery and no scalar evaluator, cache, or approximation-kernel optimization applicable to Hyper; no change selected.
### `reals/iso_CReals.v` (1590 lines)

Constructs a canonical isomorphism between arbitrary CoRN CReals models by mapping each value through its rational approximation sequence `G` and taking `Lim` in the target model. It proves order, equality, inverse, surjectivity, and preservation of addition and multiplication, with explicit epsilon splits and sequence bounds. The bridge is semantically useful for model independence but introduces repeated Cauchy and Lim conversions and proof-heavy constants; Hyper has one canonical scalar backend, so no runtime or representation change selected.
### `algebra/CRings.v` (1390 lines)

Defines constructive commutative rings, setoid-safe arithmetic, recursive natural exponentiation, natural and integer injections, finite and infinite ring sums, distributivity, and partial-function multiplication/powers. Integer injection includes an efficient positive-bit recursion (`pring_aux`) while natural exponentiation is structurally linear in the exponent; Hyperreal’s scalar/power paths already use dedicated structural reductions and power-of-two factoring, so no replacement was justified without a measured regression-free benchmark.
### `reals/Bridges_iso.v` (1747 lines)

Implements Bridges’ constructive completeness argument: derives least-upper and greatest-lower bounds from a lub principle, bounds finite samples using explicit quadratic envelopes, and constructs a Cauchy limit as the infimum of tail suprema. It proves tail monotonicity, convergence, and a Cauchy-sequence model of the ordered field. The construction repeatedly materializes finite bounds and nested supremum/infimum witnesses (and uses linear recursive sums), so it is semantic completeness infrastructure rather than an efficient scalar evaluator; no Hyper change selected.
### `reals/Max_AbsIR.v` (1469 lines)

Defines constructive `Max`, `Min`, and absolute value by selecting between operands at precision threshold `1/(n+1)` rather than deciding exact equality. The resulting selector sequence is Cauchy, and extensive order, monotonicity, translation, scaling, and partial-function-domain lemmas establish exact lattice laws. Hyperreal already exposes precision-directed comparison and structural absolute-value handling; adding a separate lim-based max would add evaluator work and memory, so no worthwhile change selected.
### `reals/stdlib/CMTReals.v` (1468 lines)

Builds Bishop/Cheng constructive integration over uniformly continuous compact-support functions: closure under sum, absolute value, clipping, scaling, trapezoid approximants, series limits, and open/closed interval measures. The implementation uses proof-carrying support bounds and repeated interval-integral extensionality; it is an integration-space model rather than a scalar evaluator or performance kernel, so no Hyper change selected.
### `reals/stdlib/ConstructiveUniformCont.v` (1441 lines)

Defines proof-carrying uniform-continuity moduli and Lipschitz conversion, then derives moduli for constants, sums, translations, scaling, min/max, clipped Heaviside/trapezoid functions, products, and finite sums. Error composition consistently splits the requested epsilon in half; products first establish interval bounds and scale each factor’s modulus. Boundedness uses a finite grid plus `CRfloor`. This is constructive analysis and proof-level modulus plumbing; Hyper’s evaluator already propagates certified interval demand directly, so no measured scalar change selected.
### `complex/NRootCC.v` (1381 lines)

Constructs exact complex square and odd nth roots using real square roots, conjugation identities, polynomial odd-root existence, and repeated doubling from odd exponents. The formulas preserve exact algebra but rely on classical apartness branches and substantial polynomial witness proofs; Hyperreal currently targets real scalar roots and has no complex evaluator, so no direct change selected.
### `reals/stdlib/ConstructiveCauchyIntegral.v` (3015 lines)

Constructs the Cauchy integral of uniformly continuous functions from finite histogram sums over interval partitions. It defines partition mesh bounds, refinement/packet regrouping, rational partition merging with sorted-list injections, explicit error bounds `eps*(b-a)`, Cauchy convergence, translation and Chasles laws, positivity/monotonicity, constants, scaling, trapezoid integrals, and the identity integral of `x`. The evaluator materializes many partition lists and repeatedly merges them for proofs; this is rigorous quadrature/completeness infrastructure rather than a scalar kernel, so no Hyper change selected.
### `ode/AbstractIntegration.v` (1500 lines)

Defines an abstract integral over rational intervals with additive and boundedness interfaces, proves Riemann-sum approximation under local uniform continuity, and derives total signed integrals, absolute bounds, linearity, and a locally Lipschitz integral operator. The constructive schedule chooses a positive partition size from the continuity modulus and scales error by interval width; proof code repeatedly materializes sums and splits epsilon into halves/quarters. HyperSolve already uses direct demand propagation and iterative integration primitives, so no scalar or runtime change was selected. The Lipschitz bound and explicit integral error formula remain useful design references for future ODE contracts.
### `ode/SimpleIntegration.v` (779 lines)

Implements the abstract integral with midpoint Riemann sums. It chooses subdivision counts from the local-uniform-continuity modulus, samples each midpoint at a precision proportional to error/width, and proves well-definedness by common-subdivision gcd alignment plus triangle bounds. Additivity and boundedness are obtained by repeated materialized finite sums; the source explicitly calls the implementation straightforward but inefficient. HyperSolve’s evaluator already avoids this proof-oriented resampling path, so no change selected; midpoint/error scheduling is retained as a future quadrature-contract reference.
### `ode/Picard.v` (814 lines)

Builds a constructive Picard operator over bounded uniformly continuous vector fields: domain extension clamps rationals to a closed ball, integrals are represented by midpoint Riemann sums, and contraction factor is `L*rx < 1`. The proof establishes invariant output radius, uniform continuity, and Banach fixed-point existence; the concrete computation section notes iteration is prohibitively slow. HyperSolve already has iterative/fixed-point machinery with exact interval witnesses and resource limits, so no direct change was selected. The explicit invariant `rx*M ≤ ry` and contraction-radius check are useful completeness contracts.
### `ode/FromMetric2.v` (283 lines)

Bridges metric spaces to metric-ball/completion interfaces, defines regular-function completion and limits, and proves approximation-based ball equivalences. It adds generic translation/negation/sum uniform-continuity and local-Lipschitz instances, using epsilon splitting and nested-ball reasoning. These are typeclass/proof bridges with no distinct scalar representation or allocation strategy; Hyper’s concrete interval backend already avoids this indirection, so no change selected.
### `ode/metric.v` (1027 lines)

Defines the foundational metric-ball classes used by CoRN ODEs: nonnegative/closed ball laws, completion and regular-function limits, product and function-space L-infinity metrics, uniform/local continuity, Lipschitz and contraction records, and sequence/Cauchy limit transfer. Product/function metrics use pointwise conjunctions and `max` radii; continuity composition propagates `Qinf` moduli and explicitly handles infinite/zero cases. This is proof/typeclass infrastructure with substantial instance-search indirection; HyperSolve’s concrete interval domains already encode these invariants directly, so no runtime or scalar change selected.
### `ode/metric.v` (1027 lines)

Defines the foundational metric-ball classes used by CoRN ODEs: nonnegative/closed ball laws, completion and regular-function limits, product and function-space L-infinity metrics, uniform/local continuity, Lipschitz and contraction records, and sequence/Cauchy limit transfer. Product/function metrics use pointwise conjunctions and `max` radii; continuity composition propagates `Qinf` moduli and explicitly handles infinite/zero cases. This is proof/typeclass infrastructure with substantial instance-search indirection; HyperSolve’s concrete interval domains already encode these invariants directly, so no runtime or scalar change selected.
### `transc/Pi.v` (1121 lines)

Defines π as twice the limit of an increasing cosine-zero sequence `pₙ₊₁ = pₙ + cos(pₙ)`. Geometric-series bounds from `1−sin(1)` certify Cauchy convergence; subsequent lemmas establish positivity, half/quarter-period values, double-angle identities, and integer periodicity. This is a proof-level π construction with slow recursive convergence and no executable kernel; Hyperreal’s Chudnovsky/MPFR π path is substantially faster and demand-directed, so no change selected.
### `reals/CauchySeq.v` (1044 lines)

Defines the canonical constructive-real model and multiple equivalent Cauchy predicates, then proves order transfer, uniqueness, apartness eventuality, and closure of limits under addition, negation, subtraction, and multiplication. Proofs use explicit epsilon halving and `Nat.max` synchronization of sequence indices. This materializes witness thresholds and is semantic completeness infrastructure; Hyperreal’s per-node error bounds and shared DAG avoid reifying Cauchy sequences, so no scalar/runtime change selected.
### `algebra/COrdFields2.v` (1196 lines)

Adds ordered-field monotonicity, cancellation, reciprocal/division bounds, nonnegative powers, power cancellation, finite-sum monotonicity, and explicit epsilon fractions (`/2`, `/3`, `/4`). Proofs repeatedly branch on apartness and constructively avoid decidable equality. Hyperreal already has exact rational sign/scale filters and structural power factoring; these generic laws offer no faster executable path, so no change selected.
### `transc/TaylorSeries.v` (658 lines)

Defines Taylor-series functions from higher derivatives and proves convergence via a factorial/geometric majorant, then gives a remainder characterization and a criterion for equality with the original function. Error bounds use explicit radius envelopes and factorial denominators; all computation is proof-level over partial functions and compact intervals. Hyperreal’s elementary kernels already use direct reduced series with certified integer error budgets, so no replacement was justified.
### `transc/Trigonometric.v` (662 lines)

Proves sine/cosine zero values and derivatives from power-series coefficient identities, then derives addition, double-angle, half/quarter-period, tangent, and integer-periodicity laws. Compact-domain mapping and Taylor majorants are used to justify termwise differentiation and functional identities. The implementation is theorem-level over partial functions; Hyperreal already has dedicated reduced trig kernels and exact residual identities, so no replacement was selected.
### `transc/SinCos.v` (412 lines)

Proves sine/cosine/tangent addition and negation identities, the Pythagorean invariant, derivative/continuity links, and boundedness from power-series definitions. The core implementation remains partial-function power series with theorem-level domain witnesses; Hyperreal’s reduced trig residual representation and exact domain guards are more operationally direct. No change selected.
### `transc/RealPowers.v` (610 lines)

Defines positive-base real powers as `Exp(y*Log x)`, proving integer/natural/rational-power identities, reciprocal/division/root relations, monotonicity in base and exponent, and derivative/continuity rules for partial functions. Domain positivity is explicit and all algebra is proof-level over exponential/logarithmic primitives. Hyperreal already has exact rational-power/root constructors and guarded logarithm/exponential kernels; no replacement was selected.
### `transc/PowerSeries.v` (581 lines)

Defines partial-function power-series terms and convergence via ratio/Dirichlet tests, then instantiates exponential, sine, cosine, and logarithm from factorial-scaled coefficients. Coefficient bounds reduce sine/cosine convergence to the exponential majorant; logarithm is a reciprocal-domain function. This is a proof-level series framework with repeated function/compactness witnesses, while Hyperreal uses direct integer-budget kernels and residual reductions. No change selected.
### `transc/MoreArcTan.v` (631 lines)

Proves ArcTan zero/one, oddness, monotonicity, reciprocal and addition identities, then derives the alternating arctangent power series on `(-1,1)` via a geometric `1/(1+x²)` series and termwise integration. Convergence bounds use `max(|x|,|y|)` compact envelopes and ratio tests; domains and nonzero denominators are explicit. Hyperreal’s inverse-trig kernels already use certified argument reduction and residual series over exact intervals, so no direct change selected.
### `algebra/Expon.v` (826 lines)

Provides ordered-field power laws, integer exponentiation, and monotonicity/positivity lemmas, then defines arbitrary positive-base real powers through `Exp(y*Log x)` and proves root/rational-power identities plus continuity/derivatives. The implementation is algebraic and proof-oriented with explicit apartness witnesses; Hyperreal’s structural exponentiation and guarded transcendental kernels are more efficient and exact for runtime evaluation. No change selected.
### `algebra/CSums.v` (765 lines)

Defines recursive finite sums (`Sum0`, bounded `Sum`, dependent `Sumx`/`Sum2`) over constructive abelian groups, with extensionality, cancellation, shift, sparse-term, and monotonicity lemmas. Every executable sum is linear recursive/materialized and many proofs rely on repeated reassociation; Hyperreal’s fused sum and linear-combination kernels already avoid this overhead. No change selected.

### `algebra/COrdAbs.v` (680 lines)

Defines `AbsSmall`/`AbsBig` predicates and proves ordered-field error propagation for sums, products, scaling, cancellation, and threshold classification. The product bounds (including a conservative 3·e₁·e₂ estimate) are generic proof lemmas; Hyperreal’s interval kernels already carry tighter endpoint bounds and exact apartness. No change selected.

### `algebra/Bernstein.v` (522 lines)

Builds Bernstein polynomial bases, degree-raising identities, coefficient-vector conversion, partition of unity, and non-negativity on [0,1]. Bernstein coefficient convex-hull bounds could tighten interval evaluation for polynomial nodes, but Hyperreal currently evaluates general expression DAGs and has no polynomial coefficient path; adding one would increase code and binary size without a demonstrated workload win. No change selected.

### `algebra/CAbGroups.v` (534 lines)

Defines constructive abelian-group structures, subgroup construction, cancellation/apartness, and recursive natural/integer repeated addition (`nmult`/`zmult`) with algebraic laws. Hyperreal scalar multiplication is already represented by compact DAG nodes and specialized integer/rational kernels; replacing them with recursive materialization would hurt memory and speed. No change selected.

### `algebra/CAbMonoids.v` (117 lines)

Adds commutative-monoid structure, properness of commutative sums, and closed submonoids. This is typeclass/proof scaffolding with no executable scalar strategy applicable to Hyperreal. No change selected.

### `algebra/CFields.v` (920 lines)

Defines constructive fields with proof-carrying reciprocal domains, apartness-preserving multiplication/division, cancellation, reciprocal identities, and partial-function lifting. The explicit nonzero witnesses reinforce Hyperreal’s domain guards and cached sign facts, but its node metadata already carries stronger executable certificates; no worthwhile change selected.

### `algebra/CGroups.v` (630 lines)

Defines constructive groups, inverse/minus cancellation and apartness, subgroup/function-space lifting, and domain-aware partial-function negation/subtraction. Hyperreal already canonicalizes negation/subtraction in its DAG and tracks exact domain facts; no executable improvement identified. No change selected.

### `algebra/CMonoids.v` (1,041 lines)

Defines constructive monoids, identity and submonoid/intersection constructors, monoid isomorphisms, cyclic/generator laws, direct products, function/free monoids, and list-backed `cm_Sum`. The materialized list sum and recursive monoid power are less memory-efficient than Hyperreal’s fused scalar reductions and compact DAG nodes; proof-carrying identity domains are already represented by Hyperreal metadata. No change selected.

### `algebra/CPoly_ApZero.v` (633 lines)

Proves polynomial factorization at roots, Lagrange-like `poly_01` interpolation bases, nonzero evaluation witnesses, interval nonzero points, and interpolation uniqueness. These are exact algebraic witness constructions; Hyperreal’s root-isolation/resultant machinery already provides stronger direct certificates, while materialized polynomial bases would add memory/code cost. No change selected.

### `algebra/CPoly_Degree.v` (676 lines)

Defines constructive polynomial degree as a partial relation (unknown when leading coefficients are not apart from zero), degree bounds for sums/products/powers, monicness, coefficient-sum representations, and field degree cancellation. The unknown-degree semantics are important for exactness, but Hyperreal’s active polynomial/root code already keeps explicit degree and validity witnesses; no safer compact optimization was identified. No change selected.

### `algebra/CPoly_Newton.v` (487 lines)

Implements divided differences and Newton interpolation over rational abscissae, with exact interpolation/leading-coefficient proofs and a sketch of repeated-integral weights. Recursive divided differences and materialized Newton products are proof-oriented; Hyperreal’s polynomial/resultant kernels already use compact coefficient structures and exact witnesses. The unfinished integral-weight section offers no deployable optimization. No change selected.

### `algebra/CPoly_NthCoeff.v` (482 lines)

Defines coefficient extraction, extensionality/apartness, coefficient-wise arithmetic, polynomial evaluation at zero, and convolution-based coefficient multiplication. The recursive `nth_coeff_mult` proof materializes a finite sum per coefficient; Hyperreal’s polynomial kernels already use compact dense/sparse convolution paths, so no change selected.

### `algebra/CRing_Homomorphisms.v` (227 lines)

Defines proof-carrying ring homomorphisms, preservation of zero/inverse/minus/natural numerals, apartness reflection, identity, and composition. Hyperreal’s scalar constructors are concrete homomorphism-like transformations with direct canonicalization; generic records would add overhead without improving runtime behavior. No change selected.

### `algebra/CSemiGroups.v` (388 lines)

Defines constructive semigroups, partial-function addition with domain intersections, subsemigroups, direct products, function composition, and free semigroups. All operations are generic proof/typeclass wrappers; Hyperreal’s scalar DAG and domain propagation already avoid these layers at runtime. No change selected.

### `algebra/CSetoidFun.v` (1,144 lines)

Defines setoid function extensionality/apartness, composition/projection combinators, free-list setoids, proof-carrying partial functions with well-defined domains, domain extension/conjunction/disjunction, and bijection inverses. The explicit separation of proof objects from function outputs is a useful allocation principle; Hyperreal already keeps validity metadata out of scalar payloads and uses compact node caches, so no safe further change was justified.

### `algebra/CSetoidInc.v` (155 lines)

Defines predicate inclusion, conjunction/extension projection, transitivity, and domain inclusion for partial-function composition. This is proof-only domain bookkeeping; Hyperreal propagates executable validity facts directly in nodes, so no change selected.

### `algebra/CSetoids.v` (1,193 lines)

Defines constructive setoids with tight apartness, product/subsetoid constructions, well-defined and strongly-extensional predicates/relations, unary/binary operation wrappers, and coercion-preserving restrictions. The explicit apartness/cotransitivity semantics are relevant to exactness; Hyperreal’s scalar facts are stronger executable certificates, while generic wrapper records would add overhead. No change selected.

### `algebra/Cauchy_COF.v` (1,104 lines)

Constructs a complete ordered field from Cauchy sequences: threshold-based apartness, sequence arithmetic, reciprocal via eventual bounds, and order/equality characterizations. Repeated `Nat.max` stabilization and fixed epsilon splitting are semantically robust but computationally expensive; Hyperreal’s demand-driven interval/DAG evaluation is substantially more efficient. No change selected.

### `algebra/OperationClasses.v` (148 lines)

Provides generic typeclasses for operation laws (associativity, commutativity, units, inverses, distributivity) and derives dual laws through rewriting. This is compile-time proof automation; Hyperreal’s concrete arithmetic avoids dynamic typeclass dispatch and already specializes commutative/canonical operations. No change selected.

### `algebra/RSetoid.v` (193 lines)

Defines the obsolete bundled RSetoid and extensional morphism combinators (`compose`, `const`, `flip`, `join`, `bind`). It relies on nested proof-carrying function wrappers and is not a scalar execution model; Hyperreal’s direct closures/DAG nodes avoid this allocation and indirection. No change selected.

### `broken/CCayleyHamilton.v` (293 lines)

A broken ssreflect development of characteristic polynomials and Cayley–Hamilton over constructive rings, using finite big-operator sums/products and coefficient-degree proofs. The determinant/matrix identity is proof-only and Hyperreal/Hypersolve already use specialized resultant and polynomial kernels; no deployable scalar change selected.

### `broken/CPoly_Lagrange.v` (172 lines)

Broken Lagrange interpolation development using separated point lists, products of linear factors, exact degree bounds, and interpolation proofs. It materializes products and relies on fragile permutation rewrites; Hyperreal’s compact polynomial/resultant paths are more robust and cheaper. No change selected.

### `broken/CompletePointFree.v` (75 lines)

A non-buildable experiment combining point-free typeclasses, metric completion, and uniformly continuous bind/composition. Definitions are commented/test scaffolding rather than an executable scalar implementation; no Hyperreal change selected.

### `broken/IntegrationExamples.v` (68 lines)

Example-only code comparing constructive integration, midpoint, and Simpson routines, with decimal display via approximate results and an elliptic-integral composition. It exposes no new scalar representation; Hyperreal’s guarded transcendental and interval integration paths are already more exact. No change selected.

### `broken/NewAbstractIntegration.v` (401 lines)

An unfinished/admitted redesign of Riemann integration with positive rational sample counts, additive/scaled/adjacent-range laws, and continuity wrappers over uniformly continuous functions. Most implementation obligations remain admitted; no reliable optimization can be extracted. Hyperreal’s existing interval integration and error-budget scheduling are more complete. No change selected.

### `broken/SimpsonIntegration.v` (446 lines)

Broken Simpson quadrature experiments over constructive/regular reals, including fourth-root interval sizing, repeated rational approximations, list/positive-index summation, and Picard integration scaffolding. Several obligations are admitted and many paths materialize sample lists; Hyperreal’s adaptive interval integration is more complete and memory-efficient. No change selected.

### `broken/abstract_gsum.v` (136 lines)

An unfinished generalized geometric-series development proving positivity, monotonicity, and ratio bounds over ordered semirings. Key nth-term and helper lemmas are admitted; it supplies no executable summation algorithm or safe Hyperreal optimization. No change selected.

### `broken/DivDiff_RepeatedIntegral.v` (295 lines)

Entire source is commented-out experimental scaffolding for expressing divided differences as repeated integrals over weight simplices. It contains admitted continuity/integration obligations and no compiled implementation; no Hyperreal change selected.

### `broken/algebra/bigopsClass.v` (992 lines)

Broken ssreflect finite big-operator framework: filtered/reindexed sums/products, permutation and partition laws, exchange, distributivity, and finite-family relations. It is proof infrastructure over materialized sequences; the distributivity sections are commented out. Hyperreal’s fused reductions already avoid these intermediate lists, so no change selected.

### `broken/diff.v` (210 lines)

Entire source is commented-out exploratory derivative/divided-difference code, including telescoping lists and point-free combinators, with admitted proofs. It is non-buildable and provides no deployable scalar optimization. No change selected.

### `broken/lagrange.v` (4,834 lines)

Broken legacy Newton/Lagrange interpolation and differentiability development. It uses axiomatized `sq`/`unsq`, many admitted or unfinished proofs, duplicated CoRN typeclass wrappers, and proof-heavy `C_inf_ab` values carrying all derivative witnesses. Integration and continuity sections materialize product/subspace structures and leave key flattening/translation lemmas admitted. The interpolation identities are algebraically standard but not an executable scalar kernel; Hyperreal already has demand-driven DAG evaluation and interval/root machinery. No exactness, completeness, performance, memory, binary-size, or code-size change selected.

### `broken/matrixClass.v` (1,322 lines)

Generic ssreflect matrix slicing, block/paste operations, ring matrix arithmetic, traces, permutation matrices, cofactors, and Leibniz determinants. Matrix entries are functions over finite ordinals and all reductions use materialized finite big-operators; determinant multiplication is factorial/permutation based. The identities are proof infrastructure over an abstract ring, not a scalar representation or runtime evaluator. Hyperreal has no analogous dense-matrix kernel in the audited scalar path, and importing this would add code/binary weight without improving exactness or scheduling. No change selected.

### `complex/CComplex.v` (693 lines)

Constructive complex numbers as pairs of IR values, with componentwise setoid/apartness, arithmetic, reciprocal via a sum-of-squares witness, and CField instances. The representation is direct and exact for complex algebra but requires an apartness proof to construct reciprocals; it provides no new scalar scheduling or approximation mechanism. Hyperreal’s interval/DAG model already handles real transcendental evaluation, so adding this legacy pair layer would increase code without improving the target scalar architecture. No change selected.

### `complex/Complex_Exponential.v` (293 lines)

Defines complex exponential as `exp(Re z) * (cos(Im z) + i sin(Im z))`, proving addition, inverse, powers, modulus, periodicity, Euler identity, and nonzero results. This is a straightforward composition of existing real Exp/Sin/Cos kernels with pair arithmetic; no independent approximation, caching, or error-budget strategy appears. Hyperreal already has the underlying guarded transcendental paths, so no change selected.

### `coq_reals/Rreals.v` (489 lines)

Builds CoRN algebra/order/real-number structures over Coq's classical `R`, including Cauchy limits through `R_complete`, and bridges `INR` to CoRN nring. This is a nonconstructive compatibility adapter: exactness comes from Coq axioms/classical completeness rather than an executable representation, and importing it would undermine Hyperreal's constructive demand-driven semantics. No change selected.

### `coq_reals/Rreals_iso.v` (1,025 lines)

Constructs the canonical isomorphism between classical Coq `R` and CoRN `IR`, then proves transport lemmas for equality/apartness/order, arithmetic, sums, powers, Exp, Sin, Cos, Tan, Log, Pi, and rationals. The bridge is useful as a specification of cross-representation law transport, but relies on classical `R`, explicit series-limit conversions, and extensive rewrite proofs rather than runtime evaluation. Hyperreal should retain typed conversion boundaries but not this nonconstructive adapter. No change selected.

### `examples/Calculemus2011.v` (144 lines)

Benchmark/demo definitions compare ARbigD and CR evaluations for pi, nested exp, arctan, large-argument cos, nested sin, and square root. The notable implementation experiment is choosing BigD multiplication by powers of two versus exponent shifting based on operand size; this is already reflected in Hyperreal’s dyadic fast paths and dispatch traces (shift-only and word-dyadic kernels), so no duplicate change was warranted.

### `examples/Circle.v` (85 lines)

Plots circle/cosine paths using uniformly continuous compositions over rational, CR, and faster ARbigD backends. The recorded sparse-raster timings (about 3.1–3.7s for faster AR versus 16.3s for CR) demonstrate representation/backend impact, but this is an application-level raster pipeline, not a scalar algorithm. Hyperreal already separates exact scalar evaluation from downstream consumers; no change selected.

### `examples/IntegrationExamples.v` (67 lines)

Executable examples for digit-style approximation, integration of sin and bounded powers, continuous suprema, and an elliptic-integral composition. It illustrates that uniform-continuity wrappers are required for integration, but adds no new scalar representation or scheduling strategy. No change selected.

### `examples/LMCS2011.v` (57 lines)

Benchmark comparison of old CR and newer ARbigD expressions across nested transcendental functions and large arguments. It reinforces that compressed/ARbigD evaluation materially improves throughput; Hyperreal already uses compact dyadic/word kernels and retained-fact dispatch. No new change selected.

### `examples/Picard.v` (206 lines)

Unfinished Picard/Banach fixed-point example: integral operators, contraction estimates, geometric difference streams, memoized streams, and Cauchy convergence are mostly admitted/TODO. It suggests memoizing successive differences, but lacks a sound completed implementation and Hyperreal already has explicit memoized DAG evaluation. No change selected.

### `examples/PlotExamples.v` (66 lines)

Demonstrates exact plot correctness via uniformly continuous functions, then extracts raster/error witnesses with computation. This is proof/application scaffolding; no scalar exactness or evaluator optimization is introduced. No change selected.

### `examples/Circle.v` (85 lines)

Plots circle/cosine paths using uniformly continuous compositions over rational, CR, and faster ARbigD backends. Recorded sparse-raster timings (about 3.1–3.7s for faster AR versus 16.3s for CR) demonstrate representation/backend impact, but this is an application-level raster pipeline, not a scalar algorithm. Hyperreal already separates exact scalar evaluation from downstream consumers; no change selected.

### `examples/IntegrationExamples.v` (67 lines)

Executable examples for digit-style approximation, integration of sin and bounded powers, continuous suprema, and an elliptic-integral composition. It illustrates that uniform-continuity wrappers are required for integration, but adds no new scalar representation or scheduling strategy. No change selected.

### `examples/LMCS2011.v` (57 lines)

Benchmark comparison of old CR and newer ARbigD expressions across nested transcendental functions and large arguments. It reinforces that compressed/ARbigD evaluation materially improves throughput; Hyperreal already uses compact dyadic/word kernels and retained-fact dispatch. No new change selected.

### `examples/Picard.v` (206 lines)

Unfinished Picard/Banach fixed-point example: integral operators, contraction estimates, geometric difference streams, memoized streams, and Cauchy convergence are mostly admitted/TODO. It suggests memoizing successive differences, but lacks a sound completed implementation and Hyperreal already has explicit memoized DAG evaluation. No change selected.

### `examples/PlotExamples.v` (66 lines)

Demonstrates exact plot correctness via uniformly continuous functions, then extracts raster/error witnesses with computation. This is proof/application scaffolding; no scalar exactness or evaluator optimization is introduced. No change selected.

### `examples/RealFast.v` (106 lines)

CR approximation/solver benchmark suite covering nested transcendental expressions, huge arguments, and expression sharing via `compress`/multivariate polynomials. It documents severe slow cases but provides no new evaluator mechanism; Hyperreal already performs DAG sharing, retained-fact reuse, and adaptive precision. No change selected.

### `examples/RealFaster.v` (82 lines)

ARbigD benchmark suite with compact BigZ answers, compression, exact apartness proofs, and AR sign solving. The faster backend validates the value of compressed dyadic representations, which Hyperreal already uses in its word/dyadic scalar paths. No additional change selected.

### `fta/CC_Props.v` (273 lines)

Complex Cauchy-sequence and continuity properties: componentwise real/imaginary limits, norm bounds from component errors, uniqueness, continuous-function limit preservation, and geometric-series convergence. The proof splits complex convergence into two IR streams and uses coarse error partitioning; it is proof-level semantics with no executable scalar evaluator or precision scheduler. Hyperreal’s scalar interval/DAG machinery already supplies tighter demand-driven bounds. No change selected.

### `fta/CPoly_Contin1.v` (156 lines)

Proves complex multiplication and polynomial continuity using norm bounds and explicit error splitting. It is proof-only continuity infrastructure over CC polynomials; no executable evaluator or scalar representation is introduced. Hyperreal’s interval propagation is already more direct and demand-driven. No change selected.

### `fta/CPoly_Rev.v` (413 lines)

Defines monomials and coefficient-based polynomial reversal, with degree, coefficient, sum, and multiplicative reversal proofs. Implementations expand polynomials into materialized finite sums and recurse through monomial lists; this is symbolic proof infrastructure rather than a runtime scalar kernel. Hyperreal’s existing polynomial/resultant code uses more targeted coefficient operations. No change selected.

### `fta/CPoly_Shift.v` (187 lines)

Defines polynomial translation `Shift a p` by substituting `(X+a)^i`, proving evaluation, inverse shift, multiplicativity, degree, and monicity. The construction materializes coefficient sums and powers; useful algebraically but not a faster exact evaluator. Hyperreal already performs expression-level affine rewrites where profitable. No change selected.

### `fta/FTA.v` (252 lines)

Constructive Fundamental Theorem of Algebra factorization over complex polynomials, using reversal and shift operations to peel off a linear factor and recurse. It is entirely proof-level and depends on prior polynomial APIs; no root-isolation evaluator or scalar runtime optimization is implemented. Hyperreal’s root/resultant machinery is more operational and interval-driven. No change selected.

### `fta/FTAreg.v` (614 lines)

Constructive FTA regularity proof via a Kneser iterative sequence: each step contracts polynomial residuals by `q<1`, derives geometric Cauchy bounds for complex components, and obtains a limiting root. The sequence and convergence arguments are proof-level and repeatedly materialize sums/limits; they do not provide a practical root isolation or adaptive evaluator. Hyperreal’s certified interval/Sturm/resultant paths are more operational. No change selected.

### `fta/KneserLemma.v` (554 lines)

Proves Kneser’s contraction bound for monic complex polynomials. It constructs an explicit tiny contraction factor from `p3m`, recursively builds a sequence of approximate roots, and bounds residuals using complex norms and geometric sums. The result is noncomputable/proof-oriented with very conservative constants and no executable root search; Hyperreal’s interval and algebraic isolation machinery is substantially more useful operationally. No change selected.

### `fta/MainLemma.v` (483 lines)

Derives the coefficient-selection and radius bounds used by the Kneser root proof. It repeatedly rescales by powers of three, splits lower/upper coefficient sums, and obtains conservative geometric error bounds. All work is constructive inequality proof over IR with finite materialized sums; it does not expose a root isolation or scalar-evaluation algorithm. Hyperreal’s adaptive interval bounds are tighter and operational. No change selected.

### `fta/KeyLemma.v` (531 lines)

Builds the coefficient index sequence and scaling lemmas used by the FTA proof. It repeatedly rescales radii by powers of one-third, maintains a descending maximizer index, and proves geometric error bounds. The construction is an existence proof over IR with conservative constants and no executable search/state representation; Hyperreal’s adaptive interval/root machinery is more precise and efficient. No change selected.

### `ftc/COrdLemmas.v` (464 lines)

Ordered-field lemmas for merging finite point sequences and translating indexed sums. `om_fun` interleaves disjoint ordered lists while preserving monotonicity and predicates; `Sumx_Sum_Sum` and variants rewrite partitioned sums. These are proof infrastructure over finite lists with dependent proof arguments, not scalar evaluation or precision scheduling. Hyperreal’s reductions are already fused and allocation-light. No change selected.

### `ftc/Derivative.v` (427 lines)

Defines interval derivative as a relational epsilon/delta property over partial functions, proves derivative uniqueness, domain inclusion, and continuity transfer, and supports restriction to subintervals. Derivatives carry many proof witnesses and rely on approach-to-zero arguments; there is no executable automatic differentiation or scalar approximation strategy. Hyperreal’s symbolic/DAG operations remain more suitable. No change selected.

### `ftc/Derivative.v` (427 lines)

Defines interval derivative as a relational epsilon/delta property over partial functions, proves derivative uniqueness, domain inclusion, continuity transfer, and restriction to subintervals. Derivatives carry many proof witnesses and rely on approach-to-zero arguments; there is no executable automatic differentiation or scalar approximation strategy. Hyperreal’s symbolic/DAG operations remain more suitable. No change selected.

### `ftc/DerivativeOps.v` (658 lines)

Derivative rules for constants, identity, addition, negation, multiplication, reciprocal, powers, polynomials, and finite sums. Proofs derive continuity/norm bounds and split error budgets with repeated `Min`/`Max` witnesses; derivative values remain relational partial functions carrying domains and certificates. This offers no executable differentiation or scalar scheduling mechanism, and Hyperreal’s symbolic DAG can represent these operations more compactly. No change selected.

### `ftc/CalculusTheorems.v` (711 lines)

Miscellaneous calculus theorems: continuity commuting with limits, positivity/increasingness glued across intervals, zero-derivative constancy, uniqueness from initial conditions, and monotonicity from derivative lower bounds. Proofs use interval compactness, `Min`/`Max`, and mean-value estimates with explicit witnesses; no executable evaluator or scalar data-structure ideas are present. Hyperreal’s interval arithmetic is already more operational. No change selected.

### `ftc/Composition.v` (1,035 lines)

Develops composition of partial functions with explicit `maps_into_compacts` obligations, continuity and chain-rule derivative proofs, limits of composed sequences/series, and generalized interval variants. The domain/range certificate separation is conceptually relevant to Hyperreal, but this implementation carries extensive dependent proof witnesses and repeatedly computes norm bounds; Hyperreal already has explicit domain/guard metadata in its DAG nodes. No change selected.

### `ftc/Continuity.v` (1,091 lines)

Defines constructive uniform continuity on compact intervals, derives totally bounded images, least upper/lower bounds, and function norms, then proves continuity for algebraic operations, max/min, reciprocal, roots, finite sums, and restrictions. Composition and continuity proofs carry dependent domain/compactness witnesses and repeatedly split error budgets. The norm/lub abstraction is semantically useful, but its runtime would materialize proof-heavy bounds; Hyperreal already propagates interval bounds directly through DAG evaluation. No change selected.

### `ftc/Differentiability.v` (425 lines)

Defines differentiability as existence of a restricted setoid derivative witness, with domain inclusion and transfer to subintervals. Proves algebraic operations, powers, polynomials, reciprocal/division, finite sums, and differentiability-implies-continuity. The witness-heavy relational design is semantically rigorous but not a runtime AD representation; Hyperreal’s symbolic DAG can share expressions without carrying proof objects. No change selected.

### `ftc/FTC.v` (675 lines)

Defines indefinite integrals over compact intervals and proves continuity of primitives, FTC1/FTC2, Barrow's rule, convergence of integral sequences, and termwise differentiation of convergent series. The proofs use norm/lub bounds, Archimedean index selection, Cauchy limits, and materialized sequence primitives; this is proof-level integration semantics rather than a runtime scalar evaluator. Hyperreal's interval integrator and explicit error-budget scheduling are more direct and demand-driven. No change selected.

### `ftc/FunctSequence.v` (1116 lines)

Defines several equivalent Cauchy/convergence notions for sequences of continuous partial functions (pointwise, norm, fixed-reference, and epsilon forms), constructs a continuous limit function, proves proof-irrelevance, uniqueness, and closure under +, -, *, inverse. Thresholds use Archimedes and fixed `one_div_succ k`; products split error into thirds using function norms. This is proof-heavy sequence infrastructure with materialized function witnesses, not a runtime evaluator. Hyperreal already retains DAG sharing and adaptive interval/error budgets; no worthwhile change selected.

### `ftc/FunctSums.v` (531 lines)

Defines partial-function finite sums (`FSum0`, `FSum`, dependent `FSumx`) over intersections of all summand domains, with extensionality, splitting, order, scalar-commutation, and domain-characterization lemmas. `FSumx` is recursively materialized and then related to ordinary sums. The domain-intersection discipline is semantically useful, but Hyper reductions already track operation domains/definedness and use fused reductions instead of proof-carrying partial-function sums. No change selected.

### `ftc/IntervalFunct.v` (262 lines)

Wraps functions on compact intervals as setoid functions and defines constant, identity, arithmetic, powers, reciprocal/division under a nonzero-on-interval certificate, absolute value, and composition with range-inclusion proofs. This cleanly separates interval membership and operation side conditions, but Hyperreal's scalar metadata/domain certificates already provide the runtime equivalent without proof-carrying function wrappers. No change selected.

### `ftc/MoreFunSeries.v` (1123 lines)

Lifts compact-interval sequence/series convergence to arbitrary intervals by quantifying over every compact subinterval, constructs interval-wide pointwise series sums, and proves algebraic closure, comparison/ratio tests, geometric power-series convergence, and insertion-shift invariance. Runtime implications are conservative tail bounds and compact-local certificates; Hyper already uses adaptive interval tails and retained DAG nodes. The repeated re-projection to compact intervals and materialized partial sums offer no worthwhile implementation change.

### `ftc/MoreFunctions.v` (1497 lines)

Extends continuity, norms, arithmetic, reciprocal/division, finite sums, derivatives, differentiability, and nth-derivatives from compact intervals to arbitrary intervals. `FNorm` is a compact-local sup norm; `N_Deriv_fun` computes nth derivatives by selecting a compact neighborhood around each point and proving independence of the chosen neighborhood. This neighborhood re-selection is proof machinery and potentially expensive if reified, but Hyper's scalar kernels already use direct global/local interval bounds and cached metadata. No worthwhile change selected.

### `ftc/MoreIntegrals.v` (756 lines)

Generalizes integration to reversed/equal endpoints by subtracting integrals from `Min(a,b)`, proves extensionality, linearity, domain-addition and orientation laws, norm bounds, and positivity/nonzero consequences. The key bound is `|Integral| <= ||F|| * |b-a|`; proofs duplicate endpoint cases and compact inclusions. Hyper's interval integrator already represents orientation and explicit error budgets directly; no worthwhile change selected.

### `ftc/MoreIntervals.v` (1284 lines)

Defines the nine constructive interval forms and predicates, compact/proper/finite metadata, convexity and Min/Max closure, endpoint extraction, compact neighborhoods around one or two points, and equivalence between arbitrary intervals and compact endpoint intervals. It then lifts continuity, derivatives, and differentiability by quantifying over compact subintervals. The one/two-point neighborhoods use midpoint-like half-distance constructions and are repeatedly re-proved for endpoint membership. This is a strong semantic model for local certificates, but Hyper already has interval metadata and adaptive local bounds; no worthwhile change selected. The initial whole-file read exceeded the output cap, so the source was reread in three bounded chunks before marking complete.

### `ftc/NthDerivative.v` (793 lines)

Defines nth derivatives/differentiability recursively through dependent setoid-function witnesses, proves proof irrelevance, uniqueness, restriction to subintervals, derivative-order composition, and constructs `n_deriv_I` by recursively extracting witnesses. The representation is proof-carrying and repeatedly materializes `PartInt`/`CSetoid_fun` wrappers; it improves semantic completeness but would increase runtime allocation in Hyper. Hyper's scalar DAG and derivative metadata are lighter. No worthwhile change selected.

### `ftc/PartFunEquality.v` (608 lines)

Defines domain-aware extensional equality for partial functions, restriction and composition laws, arithmetic congruence, image extensionality, and bounded-away-from-zero certificates that establish reciprocal/division domains. The explicit requirement that both functions be defined on the comparison predicate avoids vacuous equality. Hyper's scalar values are total DAG nodes with definedness/domain metadata, so this proof-side discipline is already reflected without a separate `Feq` layer. No worthwhile change selected.

### `ftc/PartInterval.v` (269 lines)

Provides bidirectional conversion between partial functions defined on an interval and total setoid functions over the interval, proves inverse equivalence and preservation of arithmetic/powers/reciprocal/division. The conversion carries explicit domain proofs and extensionality witnesses. Hyper’s scalar nodes are total over their representable domain and retain domain metadata directly, avoiding repeated conversion wrappers. No worthwhile change selected.

### `ftc/Partitions.v` (883 lines)

Defines proof-carrying partitions with ordered endpoints, mesh/antimesh lists, even partitions, refinements, tagged Riemann sums, and separation positivity. Mesh is computed by recursively materializing a list of adjacent widths then `maxlist`/`minlist`; common refinements use the product of subdivision counts. This is mathematically complete but allocates O(n) lists and can cause multiplicative refinement growth. Hyper's adaptive interval integrator does not use these Riemann partitions, so importing this representation would hurt memory/performance; no change selected.

### `ftc/Taylor.v` (415 lines)

Builds Taylor polynomials from recursively extracted nth-derivative witnesses and proves a constructive Lagrange remainder bound, including an apartness argument to choose a valid intermediate point. Terms are represented as dependent `FSumx` products with factorial reciprocals and repeated extensionality/domain proofs. Hyper's series kernels already use direct coefficient/error bounds and retained DAG nodes; importing this proof-heavy construction would increase allocation and code size. No worthwhile change selected.

### `ftc/Rolle.v` (794 lines)

Constructive Rolle and mean-value theorems. It derives a local derivative modulus, samples a uniform compact grid with `compact_nat`, uses telescoping `Sumx`, and performs finite sign-case analysis to locate a point whose derivative is bounded. This is an exact existence proof, not an executable root/zero finder; the grid and dependent witnesses are materialized. Hyper's interval/root kernels already use adaptive subdivision and retained bounds, so no worthwhile change selected.

### `ftc/StrongIVT.v` (579 lines)

Constructive strong IVT for strictly monotone continuous partial functions. It iterates a ternary-style interval contraction (new endpoints at 1/3 and 2/3), retaining endpoint sign certificates and proving geometric Cauchy convergence; additional formulations handle weak/strict target intervals and decreasing functions. This is the clearest reusable completeness idea: ternary contraction avoids requiring decidable sign equality, but the implementation materializes dependent pair records and ultimately delegates exact existence to limits. Hyper’s adaptive bisection/subdivision already has retained sign facts; a ternary split could be benchmarked but is not selected without evidence of fewer evaluations.

### `ftc/TaylorLemma.v` (823 lines)

Constructive Taylor remainder proof via Rolle: builds derivative-coefficient terms, proves endpoint cancellation, differentiates a remainder auxiliary function, and obtains a bound using a point where the auxiliary derivative is small. Uses factorial reciprocals, dependent finite sums, and extensive domain/extensionality rewrites. The exact remainder inequality is useful as a specification for Hyper’s series error certificates, but the implementation is proof-only and materially heavier than existing Hyper coefficient/tail bounds. No change selected.

### `ftc/WeakIVT.v` (612 lines)

Proves approximate IVT for continuous functions using a glb of absolute residuals, compact uniform-continuity moduli, and finite sampled grids; for strictly increasing functions it uses ternary interval contraction with endpoint value certificates and geometric Cauchy limits to obtain an exact root. The constructive endpoint-sign scheduling is a useful completeness specification, but dependent records and fixed 2/3 contraction are proof-heavy. Hyper already retains sign/interval facts and adaptively subdivides; no change selected without a benchmarked evaluation reduction.

### `ftc/WeakIVTQ.v` (181 lines)

Strengthens weak IVT by selecting a rational `Q` approximation within an interval and residual tolerance, using constructive rational density plus continuity. It explicitly separates an exact interval witness from a cheaply representable rational output. This is a useful API idea for Hyper: expose rational/dyadic witness extraction from an interval when the requested tolerance is met. Hyper already has dyadic approximation paths, so no new implementation selected; add only if coverage confirms a missing public helper.

### `liouville/CPoly_Euclid.v` (166 lines)

Proves Euclidean polynomial division and uniqueness over constructive commutative rings using coefficient/degree bounds, monic divisors, and recursive coefficient cancellation. The algorithmic shape is exact but restricted to polynomial rings; Hyper’s polynomial/root code already has direct coefficient storage and degree metadata. No scalar or performance change selected.

### `liouville/CRingClass.v` (62 lines)

Adds a constructive commutative-ring instance over CoRN’s CRing and a generic sub-CRing constructor from predicates closed under operations. This is typeclass/proof scaffolding with no scalar evaluation or storage strategy. Hyper’s concrete scalar types avoid this abstraction overhead. No change selected.

### `liouville/Liouville.v` (477 lines)

Constructive Liouville irrationality bounds for rational-coefficient polynomials: computes coefficient absolute-value bounds on a finite interval, clears denominators with integer LCMs, and derives explicit lower bounds on nonzero polynomial values at irrational algebraic candidates. This is valuable as a specification for exact algebraic separation certificates, but it is specialized proof code with large conservative constants and no general scalar evaluator. Hyper’s algebraic/root metadata would need a targeted separation API before reuse; no change selected.

### `liouville/QX_ZX.v` (410 lines)

Converts rational-coefficient polynomials to integer-coefficient form by recursively collecting coefficient denominators, computing an integer LCM, and normalizing to monic form; proves evaluation and degree preservation through ring homomorphisms. The denominator list/LCM materialization is exact but can inflate intermediate integers and memory. Hyper’s polynomial representation already keeps rational/dyadic coefficients directly; denominator clearing should remain an opt-in algebraic certificate, not a default path. No change selected.

### `liouville/QX_extract_roots.v` (278 lines)

Implements rational-root extraction for rational polynomials: tests zero at 0 and a finite candidate list generated from constant/leading coefficients, then recursively divides by a linear factor while decreasing degree. It proves nonzero residuals at rational points and preserves irrational roots under denominator-cleared homomorphisms. Candidate-list materialization and repeated polynomial division are exact but potentially expensive; Hyper’s root isolation should prefer interval isolation and only invoke rational-root testing as a fast-path. No change selected.

### `liouville/QX_root_loc.v` (407 lines)

Derives rational-root localization: denominator divisibility constraints on any rational root are obtained from leading/constant coefficients after denominator clearing, then packaged as a finite candidate list. It also proves degree behavior under linear division. This is an exact algebraic fast path, but list generation and integer arithmetic are expensive; Hyper should use it only when coefficients are already rational/algebraic and interval isolation would otherwise be costly. No change selected.

### `liouville/Q_can.v` (150 lines)

Canonicalizes rationals by dividing numerator/denominator by their integer gcd, proves representation invariance, positive canonical denominators, canonical reconstruction, and coprimality. This is a compact exact normalization strategy; Hyper’s dyadic/rational scalar paths already normalize where needed, and applying gcd normalization eagerly could hurt performance. No change selected.

### `liouville/RX_deg.v` (217 lines)

Defines a decidable polynomial degree by recursively testing zero tails, proves degree witnesses, extensionality, constants/monomial degrees, inverse invariance, and max-degree behavior for sums with unequal degrees. This metadata is directly relevant to exact polynomial scheduling, but Hyper already stores degree/zero facts in its polynomial nodes; recursive decidable-tail scans should remain cached or lazy. No change selected.

### `liouville/RX_div.v` (65 lines)

Defines polynomial division by a monic linear factor via Euclidean division and proves the evaluation/remainder identity `p = q*(X-a) + p(a)`. This is an exact synthetic-division specification; Hyper’s polynomial evaluator/root code can use it as a fast path when a candidate root is known, but existing kernels already cover equivalent coefficient operations. No change selected.

### `liouville/RingClass.v` (123 lines)

Defines generic ring/subring typeclasses and lifts operation laws to dependent subtype records. This is abstraction scaffolding only; Hyper’s concrete scalar implementations avoid these runtime/typeclass layers. No change selected.

### `liouville/Zlcm.v` (255 lines)

Defines integer LCM from gcd, proves divisibility/universal properties, nonzero behavior, list-folded LCM, and exact divisibility reconstruction. It is used for denominator clearing; eager LCM growth can inflate integer intermediates, so Hyper should keep denominator clearing lazy/opt-in. No change selected.

### `liouville/nat_Q_lists.v` (226 lines)

Builds finite natural-product and rational candidate lists, proving coverage for numerator/denominator divisibility constraints and bounds. This supports rational-root enumeration but materializes Cartesian products and duplicate sign candidates. Hyper should avoid this by using bounded candidate generation only on demand. No change selected.

### `ftc/RefSepRef.v` (728 lines)

Constructs a common separated refinement of two separated partitions by merging interior points through an order-preserving interleaving function. The resulting partition has `pred (m+n)` points; mapping proofs use apartness and monotonicity witnesses. This is exact but proof-heavy and allocates a merged partition; Hyper’s adaptive integrator does not materialize Riemann partitions. No change selected.

### `ftc/RefLemma.v` (1163 lines)

Proves the first Riemann-sum refinement estimate: with a continuity modulus and `Mesh P <= d`, any tagged sum over a refinement Q differs from the tagged sum over P by at most `e*(b-a)`. The proof builds refinement index maps, nested `Sumx`/`Sum2` telescoping expressions, and repeatedly applies triangle inequalities and mesh bounds. It is mathematically exact but materializes nested sums and dependent partition/tag witnesses; Hyper’s adaptive interval integrator avoids Riemann partitions and already carries direct error budgets. No worthwhile change selected.

### `ftc/RefSeparated.v` (840 lines)

Constructs a separated common partition by scanning partition points, selecting either a nearby prior point or a midpoint offset, and lifting tags with `Max`; proves mesh/error bounds through continuity and AntiMesh. The implementation repeatedly materializes dependent partition functions, finite disjunctions, and `Sumx` bounds. Hyper’s interval/adaptive representation avoids this global partition construction; no worthwhile runtime change selected.

### `ftc/RefSeparating.v` (1235 lines)

Builds a separating refinement of a partition via a recursively scanned index map. It proves monotonicity, endpoint coverage, mesh bounds, tag lifting, and Riemann-sum error estimates using nested `Sum2`/`Sumx`, finite disjunctions, and dependent natural-number bounds. Exactness is proof-level and the construction is O(n) materialized partition state with costly dependent witnesses; Hyper’s adaptive interval subdivision is more direct and avoids this global refinement. No worthwhile change selected.

### `logic/CLogic.v` (1662 lines)

Defines CoRN’s computational propositions (`CProp`), dependent computational conjunction/disjunction/existentials, setoid-aware relations, decidable/order encodings, finite-choice eliminators, natural/integer induction and monotonicity lemmas, and list-wide computational predicates (`CForall`, `CNoDup`). These are proof-language foundations rather than scalar algorithms. The key architectural lesson—keep computationally relevant branch witnesses explicit and avoid false decidability—is already reflected in Hyper’s certified/unknown comparison APIs. No worthwhile production change selected.

### `logic/Classic.v` (173 lines)

Defines double-negation-stable classical disjunction/existential encodings and a pigeonhole principle without classical axioms. These are Prop-level proof conveniences with no executable scalar representation; Hyper’s explicit `Unknown`/partial comparison semantics remain preferable for computation. No worthwhile change selected.

### `logic/CornBasics.v` (969 lines)

Provides CoRN foundational nat/Z arithmetic lemmas, coercion and sign conversions, computational case splits, well-founded induction in `Type`, positive-number recursion, and list iteration helpers. These support proof extraction and representation plumbing but do not add scalar algorithms; Hyper already uses direct Rust integer/dyadic primitives and explicit recursion. No worthwhile change selected.

### `logic/Stability.v` (158 lines)

Defines a double-negation monad, stable propositions, decidability wrappers, and stability instances for constructive reals/nonnegativity/equality. The useful semantic point is explicit separation between semidecision and stable propositions; Hyper’s `Certified`/`Unknown` results already preserve this boundary. No worthwhile change selected.

### `metric2/CompleteProduct.v` (172 lines)

Defines uniformly continuous projections and pairing between completed product spaces, with regular-function proofs and extensional correctness. This is completion/product interface glue; Hyperlimit’s immutable approximant model already avoids rebuilding paired completion witnesses. No worthwhile change selected.

### `metric2/DistanceMetricSpace.v` (167 lines)

Builds a metric-space interface from a CR-valued distance, proving ball well-definedness, closure, symmetry, triangle, and equality. It bridges finite-distance metrics to CoRN’s ball model; Hyper’s interval distances and explicit radii already encode the stronger operational contract. No worthwhile change selected.

### `metric2/FinEnum.v` (1000 lines)

Defines finite enumerations as lists under a double-negated Hausdorff-ball membership predicate, proving metric, locatedness, prelength, map, reverse, and completion embedding properties. The construction repeatedly traverses/materializes lists and uses classical pigeonhole extraction to remove double negation. Hyper’s interval sets and immutable DAGs avoid list-level Hausdorff enumeration; no worthwhile scalar/performance change selected.

### `metric2/LocatedSubset.v` (248 lines)

Defines constructive located subsets via near/far sumbool separation, with closure, unions, finite/compact approximations, and completion transport. The explicit locatedness contract parallels Hyper’s certified interval predicates, but its finite-list/compact witness machinery is not a scalar improvement. No change selected.

### `metric2/Metric.v` (248 lines)

Defines CoRN metric spaces by rational ball relations, including closure, stability, setoid equality, weakening, and infinite-distance behavior. This ball-based partial-distance model is semantically relevant to Hyper’s interval bounds, but the existing Hyper scalar contract is stronger and operationally richer. No change selected.

### `metric2/ProductMetric.v` (385 lines)

Builds product metrics, located/decidable/prelength lifting, uniformly continuous tensor/curry/flip/association maps, and completion distribution over products. These are generic metric/completion combinators; Hyper’s typed lattice/limit layers already specialize such composition without materialized completion witnesses. No change selected.

### `metric2/Prelength.v` (664 lines)

Defines prelength spaces via constructive midpoint/trail witnesses, proves finite modulus summation, and introduces faster completion combinators (`Cmap`, `Cbind`, `Cap`, `Cmap2`) with monad laws and completion prelength. The key performance idea is modulus-aware approximation that avoids repeated slow completion maps; Hyper already uses per-node precision budgets and fused evaluator kernels, while these witnesses remain proof-heavy. No worthwhile change selected.

### `metric2/list_separates.v` (54 lines)

Generates each list element paired with its complement and proves length/permutation invariants. This is finite-enumeration support only; no scalar or Hyper performance transfer.

### `metric2/Graph.v` (665 lines)

Constructs compact graphs of uniformly continuous functions and completed binds using product metrics, `Cmap`/`Cbind`, finite enumeration approximations, and Hausdorff locatedness. The graph correctness proofs repeatedly materialize compact approximants and nested ball witnesses; Hyper’s adaptive interval/domain machinery is more direct. No worthwhile change selected.

### `metric2/Classified.v` (1187 lines)

Introduces a Qinf-radius metric class with explicit infinite/negative radius behavior, smart lifting from positive-radius balls, typeclass-based uniform continuity/moduli, function-space metrics, vectors, products, sigmas, and composition/currying helpers. The important design is a single generalized ball relation that reduces cleanly for products and avoids parallel “basic/generalized” APIs. Hyper’s scalar bounds similarly normalize uncertainty into one interval contract; this suggests keeping one canonical bound type rather than proliferating wrappers, but current Hyper already follows that direction. No code change selected.

### `metric2/StepFunction.v` (890 lines)

Represents rational-cut step functions as binary trees, with `Split`, `Mirror`, setoid equality modulo rational cuts, and applicative `Map`/`Ap` composition. Split laws preserve exact interval geometry, but repeated recursive tree splitting and equality proofs can duplicate structure. Hyper’s interval subdivision and expression DAGs already share nodes and avoid function-tree materialization; no worthwhile scalar change selected.

### `metric2/StepFunctionSetoid.v` (908 lines)

Lifts step functions from plain values to setoids: characteristic-function equality, glue/split decomposition, mirror and applicative laws, morphisms, common-subdivision induction, and BCKW combinator evaluation. The setoid layer duplicates tree traversals and relies on proof-level extensionality; Hyper’s canonical node identity and immutable DAG sharing are more compact. No worthwhile change selected.

### `model/Zmod/Cmod.v` (135 lines)

Adds computational-positive modulo decomposition and nat/Z division correctness lemmas. These are integer proof/conversion helpers; Hyper’s bigint/dyadic kernels already use direct Euclidean arithmetic. No worthwhile change selected.

### `metrics/Equiv.v` (546 lines)

Defines equivalence of pseudo-metric spaces via bijections and bounded distortion, with composition/inverse closure and a separation counterexample. This is structural metric theory rather than scalar approximation; no Hyper change selected.

### `metrics/CPseudoMSpaces.v` (340 lines)

Defines traditional CR-valued pseudo-metric spaces, symmetry/nonnegativity/triangle axioms, zero metrics, and derived diagonal/apartness lemmas. This older real-valued distance layer is less operational than Hyper’s interval bounds and adds no scalar improvement.

### `metrics/LipExt.v` (446 lines)

Formalizes McShane/Kirszbraun Lipschitz extension over totally bounded subsets using an infimum of `f(x)+C*d(x,y)`, with exact extension and Lipschitz proofs. The infimum construction is proof-heavy and requires boundedness/nonempty witnesses; Hyper’s interval optimization/solver layers would need a separate consumer, so no scalar change selected.

### `metrics/Prod_Sub.v` (412 lines)

Defines additive product pseudo-metrics and restricted/subspace metrics, with symmetry, nonnegativity, apartness, triangle, and distance-to-subspace continuity proofs. Hyper’s lattice/product bounds already use explicit component-wise interval propagation; no worthwhile change selected.

### `metrics/CPMSTheory.v` (784 lines)

Develops total boundedness, located subsets, exact infimum/supremum existence for uniformly continuous images, approximate nearest-point selectors, Cauchy completeness, compactness, and open/well-contained predicates over CR-valued pseudo-metrics. The semantic pattern of explicit approximate selectors and infimum error margins is relevant to Hyperlimit/solver contracts, but this implementation materializes lists and proof witnesses; no worthwhile scalar change selected.

### `metrics/ContFunctions.v` (712 lines)

Defines constructive continuous, uniformly continuous, and Lipschitz predicates over CR pseudo-metrics, proves conversions between exponential and reciprocal error scales, and closure under identity/constants/composition and sequence limits. Hyper’s integer error budgets and direct interval widths already avoid repeated base-2 witness conversions; no worthwhile change selected.

### `metrics/IR_CPMSpace.v` (498 lines)

Builds the CR-valued real pseudo-metric `dIR = |x-y|`, proves metric/triangle and distance-function continuity, and addition closure for Lipschitz/uniformly continuous/continuous functions. These proofs are foundational and proof-heavy; Hyper’s dyadic interval arithmetic subsumes the operational bounds. No worthwhile change selected.

### `model/Zmod/ZBasics.v` (931 lines)

Provides foundational nat/Z arithmetic, signed comparisons, multiplication monotonicity/cancellation, division-sign lemmas, absolute-value/sign identities, and integer triangle inequalities. These are proof and normalization helpers; Hyper’s bigint/dyadic backend already supplies direct operations, so no worthwhile change selected.

### `model/Zmod/ZDivides.v` (1017 lines)

Defines integer divisibility, cancellation, sign/absolute-value transport, quotient/remainder uniqueness, modulo-zero equivalences, exact division identities, and decidability. These are theorem-level bigint normalization facts; Hyper’s integer/dyadic kernels already rely on lower-level Euclidean operations and do not need this proof scaffolding. No worthwhile change selected.

### `model/Zmod/ZMod.v` (755 lines)

Defines exact modular arithmetic over positive integer moduli: normalization/idempotence, additive/multiplicative compatibility, negation, small representatives, cancellation under relative primality, linear-combination gcd identities, modular inverses, and then computational congruence/equivalence relations. This is bigint theorem infrastructure; Hyper’s integer/dyadic kernels already use direct Euclidean arithmetic, and no scalar change is justified.

### `model/Zmod/ZGcd.v` (1785 lines)

Implements positive and signed-integer Euclidean gcd with Bezout coefficients via well-founded remainder recursion, then proves divisor, symmetry, normalization, modular, quotient, relative-prime, and prime lemmas. The architecture is proof-oriented and recomputes coefficient projections from a dependent pair; Hyper’s runtime bigint paths should continue using native gcd/extended-gcd primitives, so no worthwhile scalar change is justified.

### `model/Zmod/Zm.v` (729 lines)

Builds integers modulo a positive modulus as a constructive setoid, abelian group, commutative ring, and (for prime modulus) field with gcd-based inverses. Operations are quotient-ring wrappers over ordinary integer arithmetic; Hyper has no matching finite-field scalar path, and the dependent proof scaffolding offers no worthwhile runtime change.

### `model/abgroups/CRabgroup.v` (49 lines)

Lifts CR (constructive reals) addition to a commutative abelian-group structure, proving commutativity through its Cauchy/IR model. Pure typeclass glue; no runtime change.

### `model/abgroups/QSposabgroup.v` (54 lines)

Proves positive rationals under the scaled operation x*y/2 form a constructive abelian group. Algebraic structure only; no Hyper impact.

### `model/abgroups/Qabgroup.v` (56 lines)

Lifts rational addition into a constructive abelian-group structure using the existing commutativity theorem. Typeclass glue only; no runtime impact.

### `model/abgroups/Qposabgroup.v` (53 lines)

Lifts positive-rational multiplication into a constructive abelian-group structure. The proof is ring automation over existing rationals; no worthwhile Hyper change.

### CoRN group/field wrappers (8 files, 479 lines)

`Zabgroup.v`, `CRfield.v`, `Qfield.v`, `CRgroup.v`, `QSposgroup.v`, `Qgroup.v`, `Qposgroup.v`, and `Zgroup.v` lift existing integer/rational/constructive-real operations into constructive group, abelian-group, and field typeclasses. Proofs use previously established laws and apartness; they add no new scalar algorithms or worthwhile Hyper changes.

### CoRN lattice/metric/monoid models (5 files, 547 lines)

`CRlattice.v` lifts constructive-real min/max into a lattice and records monotonicity/distributivity laws; `CRmetric.v` defines CR as the metric completion of rationals and injects rationals; `CRmonoid.v` supplies additive monoid structure; `Nm_to_cycm.v` builds a surjective natural-power morphism into generated cyclic monoids; `Nm_to_freem.v` maps naturals bijectively into the free one-letter monoid. These are structural/proof abstractions, not scalar kernels; no worthwhile Hyper change identified.

### `model/metric2/LinfMetric.v` (296 lines)

Defines sup aggregation for dyadic/rational step functions, proves its setoid and monotonicity laws, constructs the Linf step-function metric, and supplies uniformly continuous sup and Linf→L1 maps. The key reusable idea is structural: fold-based sup with compositional glue laws; Hyper’s lattice/interval code already has direct max/min reductions, so no worthwhile change was selected.

### `model/metric2/LinfMetricMonad.v` (469 lines)

Lifts a base metric ball predicate pointwise over step functions, proving glue decomposition, metric axioms, closure/stability, prelength, and bind/map uniform-continuity laws. This compositional metric-lifting pattern resembles Hyper’s interval-tree propagation, but existing Hyper code already performs direct structural traversal; no worthwhile change was selected.

### `model/metric2/L1metric.v` (560 lines)

Defines rational step-function integration, L1 norm/distance/balls, metric and prelength laws, scaling/triangle/zero lemmas, and a uniformly continuous integral map. The weighted glue recursion and norm scaling are structurally analogous to Hyper’s interval aggregation, but Hyper already uses specialized exact dyadic reductions; no worthwhile change selected.

### CoRN basic monoids (4 files, 309 lines)

`Nmonoid.v`, `Nposmonoid.v`, `QSposmonoid.v`, and `Qmonoid.v` package existing natural, positive-natural, rational, and scaled-positive-rational operations into constructive monoid structures with explicit units. These are algebraic typeclass wrappers only; no runtime scalar or performance ideas apply to Hyper.

### CoRN monoid/order-field wrappers (5 files, 443 lines)

`Qposmonoid.v`, `Zmonoid.v`, and `freem_to_Nm.v` package positive-rational/integer monoids and free-monoid length morphisms; `CRordfield.v` and `Qordfield.v` lift constructive reals/rationals into ordered-field structures with strict-order/apartness laws. All are typeclass/proof wrappers over existing arithmetic, with no worthwhile Hyper runtime change.

### CoRN ring/semigroup wrappers (5 files, 445 lines)

`CRring.v`, `Qring.v`, and `Zring.v` package constructive-real/rational/integer arithmetic into ring structures; `CRsemigroup.v` and `Npossemigroup.v` package addition/multiplication into semigroups. These prove algebraic laws over existing operations and provide no new scalar kernels or worthwhile Hyper changes.

### CoRN primitive semigroup wrappers (3 files, 185 lines)

`Nsemigroup.v`, `Qsemigroup.v`, and `Zsemigroup.v` package native natural/rational/integer addition and multiplication with associativity proofs. They are algebraic wrappers over primitive operations and offer no worthwhile Hyper change.

### CoRN setoid models (3 files, 323 lines)

`CRsetoid.v` packages constructive reals with metric equality/apartness; `Nfinsetoid.v` defines finite naturals with bounded proofs and decidable apartness; `Npossetoid.v` defines positive naturals as a subsetoid and lifts addition/multiplication while proving missing units/inverses. These are representation/typeclass layers, not Hyper scalar algorithms; no worthwhile change.

### CoRN natural/positive-rational setoids (2 files, 441 lines)

`Nsetoid.v` builds the natural-number constructive setoid and lifts addition/multiplication plus higher-arity operations; `Qpossetoid.v` builds positive-rational equality/apartness, arithmetic, inversion, and scaled multiplication operations. These are primitive wrappers and proof obligations, with no worthwhile Hyper scalar change.

### CoRN rational/integer finite setoids (2 files, 278 lines)

`Qsetoid.v` packages rational equality/apartness, arithmetic, and ordering into constructive setoid operations; `Zfinsetoid.v` defines bounded nonnegative integer subsetoids with decidable apartness. Both are wrappers around primitive arithmetic and do not suggest worthwhile Hyper changes.

### CoRN integer and decidable-setoid infrastructure (2 files, 308 lines)

`Zsetoid.v` packages signed-integer equality/apartness and arithmetic operations into constructive setoids; `decsetoid.v` generically derives constructive apartness, setoids, and function/relations’ strong extensionality from decidable standard setoids. These are proof/typeclass infrastructure and yield no worthwhile Hyper runtime change.

### CoRN natural section structures (2 files, 367 lines)

`Nsec.v` establishes natural-number apartness, extensionality, arithmetic separation, and small decidability lemmas; `Npossec.v` proves positivity preservation for natural addition/multiplication. These are foundational proofs over machine naturals, with no worthwhile Hyper scalar change.

### CoRN extended rational/error-bound structures (3 files, 415 lines)

`QposInf.v` introduces positive rationals plus an infinity token with bind, equality, addition, multiplication, order, and minimum; `Qinf.v` extends rationals with positive infinity and ordered addition; `QnnInf.v` extends nonnegative rationals similarly with lifted bind/map operations. The explicit tagged-union infinity handling is relevant to robust bound APIs, but Hyper already models unbounded/unknown outcomes separately; no worthwhile scalar change selected.

### `model/structures/Qpossec.v` (480 lines)

Defines positive rationals as proof-carrying values, with canonical coercions, arithmetic/inverse/power, positivity lemmas, ring/field tactics, list sums, positive reduction, ceilings, and absolute-value conversion. Proof-carrying positivity and canonical reduction are useful API patterns, but Hyper’s scalar already tracks sign/normalization directly; no worthwhile change selected.

### `model/structures/StepQsec.v` (372 lines)

Defines rational setoid-valued step-function maps (abs, +, -, *, order), a ring structure, fold-based equality/zero booleans, and absolute-value/triangle laws. The direct structural lifting and fold aggregation parallel Hyper’s interval trees; Hyper already has specialized exact dyadic nodes and avoids boolean equality as a proof oracle, so no worthwhile change selected.

### `model/structures/Zsec.v` (379 lines)

Defines signed-integer apartness/extensionality and arithmetic separation, positivity/sign helpers, and several divisibility/inequality algebra lemmas used by later constructions. It relies on Coq’s signed-binary `Z`; no new scalar algorithm or worthwhile Hyper optimization is present.

### `model/totalorder/QMinMax.v` (251 lines)

Builds a rational total order with min/max, monotonicity, distributivity, De Morgan, and sign-aware multiplication laws. The explicit positive-multiplier min/max distribution is a useful exact-bound invariant; Hyper’s interval arithmetic already applies equivalent sign-split endpoint rules, so no change was retained.

### `model/totalorder/QposMinMax.v` (444 lines)

Defines proof-carrying positive rationals, exact reduction/power/ceiling helpers, a positive total order with min/max, and sign-preserving min/max distribution under addition and multiplication. Hyper already performs equivalent sign-aware interval endpoint operations; proof-carrying wrappers would add allocations, so no change selected.

### CoRN order/algebra infrastructure (5 files, 678 lines)

`ZMinMax.v` builds signed-integer total-order min/max and extensive monotonicity/distribution laws; `Lattice.v` and `PartialOrder.v` define generic order records, duals, and compatibility lemmas; `Transparent_algebra.v`/`Opaque_algebra.v` toggle reduction visibility for bundled algebra projections. Hyper’s endpoint logic already implements these order laws, while transparency is a build-time concern; no worthwhile runtime change selected.

### `order/TotalOrder.v` (329 lines)

Defines total-order extensions over lattices, generic monotone/antitone min/max distribution, modular/disassociation laws, dual orders, and constructors from comparison decision procedures. The default min/max branches on a supplied total-order decision; Hyper’s interval extrema already branch on exact sign/comparison outcomes, so no worthwhile change selected.

### CoRN real/metric/order infrastructure (4 files, 599 lines)

`CMetricFields.v` defines metric-field absolute values and Cauchy completeness contracts; `CPoly_Contin.v` proves continuity closure for polynomial operations; `CReals.v` specifies Cauchy-complete Archimedean ordered fields; `CSumsReals.v` proves geometric-series and finite-sum/scaling identities. These are semantic contracts and theorem layers, not runtime kernels; no worthwhile Hyper change selected.

### CoRN real/metric/order infrastructure (4 files, 599 lines)

`CMetricFields.v` defines metric-field absolute values and Cauchy completeness contracts; `CPoly_Contin.v` proves continuity closure for polynomial operations; `CReals.v` specifies Cauchy-complete Archimedean ordered fields; `CSumsReals.v` proves geometric-series and finite-sum/scaling identities. These are semantic contracts and theorem layers, not runtime kernels; no worthwhile Hyper change selected.

### `reals/Cauchy_CReals.v` (850 lines)

Constructs reals as Cauchy sequences over an Archimedean ordered field, with injection of field values, algebra/order cancellation, explicit convergence moduli, diagonal sequence completion, density, and completeness proofs. The repeated safety-factor splits (1/3, 1/4, 1/6, 1/8) are proof scaffolding; Hyper’s integer error budgets encode equivalent margins without dependent sequence wrappers. No worthwhile runtime change selected.

### `reals/Intervals.v` (1015 lines)

Defines compact intervals, restrictions/inclusions, constructive total-boundedness, finite ε-net representatives, lub/glb construction by Cauchy limits, and uniform interval partitioning with explicit mesh bounds. The representative-selection/error-budget pattern is conceptually relevant to Hyper’s refinement scheduler, but Hyper already uses demand-driven interval tightening and immutable certificates; adopting CoRN’s list-heavy witnesses would increase allocation, so no change selected.

### `reals/NRootIR.v` (880 lines)

Constructs nonnegative nth roots via polynomial IVT, proves uniqueness/monotonicity/continuity, square-root and absolute-value identities, norm/triangle bounds, reciprocal Cauchy convergence, and partial-function lifting. Root selection is existence/proof based rather than a practical iteration kernel; Hyper’s integer-budget Newton/bisection kernels are more operational and efficient. No worthwhile change selected.

### `reals/Q_dense.v` (918 lines)

Implements constructive rational density using shrinking rational intervals: trichotomy chooses a subinterval by a 1/3–2/3 split, `Intrvl` recursively narrows around a real, interval lengths contract by 2/3, and center sequence `G` converges with explicit rates. This is a strong scheduling/completeness pattern; Hyper’s adaptive interval refinement already centralizes exact sign/interval decisions, while fixed ternary splits would add work versus demand-driven precision, so no change selected.

### `reals/Q_in_CReals.v` (996 lines)

Defines rational injection into an arbitrary CReals structure, proves denominator/nonzero and nring/zring extensionality/order, exact preservation of arithmetic/inverse/division and AbsSmall, and preservation of Cauchy sequences. It is proof-heavy embedding infrastructure; Hyper’s scalar already has direct rational/dyadic constructors and exact arithmetic, so no worthwhile runtime change selected.

### `order/SemiLattice.v` (163 lines)

Defines meet-semilattice laws, extensional compatibility, commutativity/associativity/idempotence, order characterizations, and monotonicity. Pure proof-level order infrastructure; Hyper’s lattice helpers already encode needed scalar bounds, so no runtime change.

### `reals/PosSeq.v` (168 lines)

Proves positivity and divergent-sum properties for positive sequences, positivity/apartness of partial sums, and ratio tail bounds used in series convergence. The explicit witness/certificate style is useful semantically but duplicates Hyper’s adaptive error budgets; no worthwhile change selected.

### `reals/R_morphism.v` (673 lines)

Defines CReals homomorphisms preserving apartness, order, addition, multiplication, and Cauchy limits; derives preservation of zero/one/minus/inverse, composition/isomorphism/surjectivity, and simplification lemmas. The explicit structure is a useful semantic contract, but Hyper already centralizes scalar operation traits and certified approximants; no worthwhile change selected.

### `reals/RealCount.v` (368 lines)

Cantor-style nested interval construction proving reals are uncountable, with fixed 2/3 shrink and constructive cotransitivity branching. Conceptually reinforces adaptive exclusion certificates, but Hyper already has demand-driven refinement and no change is justified.

### `reals/RealFuncts.v` (260 lines)

Defines interval predicates and epsilon-delta/Cauchy notions of limits and continuity, plus monotonicity/IVT specifications (some historical axioms). This is semantic API material; Hyper’s limit engine is operationally stronger, so no runtime change.

### `reals/RealLists.v` (438 lines)

Provides max/min over nonempty real lists, membership, length-preserving map, partial-function map2, and epsilon-approximate extrema. List witnesses are allocation-heavy compared with Hyper’s scalar certificates; no worthwhile change selected.

### `reals/stdlib/CMTDirac.v` (85 lines)

Builds a Dirac integration space over constructive reals, including partial evaluation at zero and integral bounds. Domain/proof scaffolding only; no scalar-runtime change.

### `reals/stdlib/CMTFullSets.v` (1982 lines)

Develops almost-everywhere/full-set integration, diagonalization of countable families, representation completion, monotone convergence, truncation limits, woven sequence continuity, and integration-space completion. Strong evidence for explicit convergence witnesses and diagonal scheduling, but this is domain-specific measure theory and would add substantial machinery to Hyper; no worthwhile change selected.

### `reals/stdlib/ConstructiveFastReals.v` (374 lines)

Adapts CoRN fast regular-function reals to the standard constructive-real interface. Key mechanisms are approximate-order characterization with a 2/n slack, constructive indefinite search for disjunctions, explicit Archimedean upper bounds, and Cauchy completion via a regular-function join. Hyper already uses certified interval/order bounds; the 2/n search pattern is less precise than current adaptive budgets, so no change.

### `reals/stdlib/ConstructiveFasterReals.v` (419 lines)

Lifts the same interface to faster algebraic reals through an order embedding into fast reals, preserving arithmetic/order/abs/inverse and Cauchy completeness. The cast-based architecture demonstrates a clean compatibility layer, but Hyper’s scalar tower is native and avoids duplicate casts; no worthwhile change.

### `reals/stdlib/CMTbase.v` (586 lines)

Defines constructive-measure-theory function Riesz/integration-space interfaces, stability under lattice operations, additive/homogeneous integrals, monotonicity/extensionality, and truncation limits. The explicit type-level closure and nonnegative integral bounds are valuable API guidance, but measure-specific and allocation-heavy; no worthwhile Hyper scalar change.

### `reals/stdlib/CMTPositivity.v` (456 lines)

Provides constructive continuity-at-zero scaling, increasing subsequence control, prepended-series identities, nonnegative sum bounds, and a classical-style integral continuity construction using geometric rescaling and weaving. The geometric tail scheduling is interesting but integration-specific; Hyper’s certified series already uses tighter adaptive budgets, so no change selected.

### `reals/stdlib/ConstructiveDiagonal.v` (1016 lines)

Implements a triangular nat²↔nat diagonal bijection, subsequence inversion/filling with zeros, preservation of Cauchy and nonnegative series convergence under subsequences, triangle/square majorants, truncation rectangles, and rigorous double-series rearrangement (including positivity requirement). The diagonal scheduling and explicit tail majorants are relevant to Hyper’s series engine, but its fixed geometric constants and allocation-heavy witness proofs are less efficient than current adaptive DAG evaluation; no worthwhile change selected.

### `reals/stdlib/ConstructivePartialFunctions.v` (861 lines)

Defines proof-carrying partial functions, pointwise limits, absolutely convergent infinite sums, domain inclusions/restrictions, arithmetic/lattice combinators, finite sums, positive/negative parts, min/max, projections, and partial division. The domain-as-certificate design is directly relevant to exactness, but Hyper’s scalar DAG nodes avoid per-value dependent witnesses and are substantially lighter; no change selected.

### `reals/stdlib/Markov.v` (131 lines)

Formalizes Markov’s principle as unbounded decidable search and derives double-negation elimination for strict order/apartness. This is noncomputable/axiomatic relative to Hyper’s certified semantics and is not suitable for adoption.

### `stdlib_omissions/List.v` (155 lines)

Adds list zip/permutation/membership/extensionality and indexed NoDup utilities. Generic proof support only; no scalar or performance improvement.

### `stdlib_omissions/P.v` (53 lines)

Adds positive↔nat conversion and order equivalence lemmas. Representation glue only; no Hyper change.

### `stdlib_omissions/Q.v` (547 lines)

Adds rational numerator/sign, injection homomorphisms, floor/ceiling, inverse/division/order/abs lemmas, and canonicalized Q list support. Hyper’s dyadic/rational kernels already provide equivalent operations with certified bounds; no worthwhile change.

### `stdlib_omissions/Z.v` (110 lines)

Adds signed-integer to nat/N conversions and order/addition conversion lemmas. Generic interop only; no scalar change.

### `tactics/CornTac.v` (59 lines)

Defines generic setoid-aware LHS/RHS replacement tacticals. Proof automation only; no scalar runtime impact.

### `tactics/DiffTactics1.v` (49 lines)

Defines small continuity/derivative Ltac wrappers around hint databases. Build-time automation only; no Hyper change.

### `tactics/DiffTactics2.v` (357 lines)

Encodes typed symbolic continuity and derivative ASTs, recursive derivative construction, and syntax-directed tactics for arithmetic, powers, reciprocals, division, and composition. The symbolic-AST approach could reduce repeated proof search, but Hyper’s runtime expression DAG is separate and this Coq tactic machinery does not transfer safely; no change.

### `tactics/DiffTactics3.v` (176 lines)

Extends derivative automation with interval-aware symbolic ASTs and syntax-directed derivative extraction. Useful as proof-generation architecture, not scalar runtime; no worthwhile change.

### `tactics/AlgReflection.v` (524 lines)

Implements a reflected expression language and normalization passes for integer/rational polynomial expressions, including monomial ordering, constant folding, rational denominator normalization, and correctness lemmas. The canonical AST/normal-form strategy could reduce repeated symbolic work, but this is Coq tactic code and Hyper’s runtime already has canonical DAG/hash-consing; no worthwhile change selected.

### `reals/stdlib/CMTIntegrableFunctions.v` (2373 lines)

Defines constructive integrable-function representations as absolutely convergent partial-function series, with representation-independent integrals, weaving/interleaving, scaling (including zero-safe restriction), addition/minus, positivity, absolute-value contraction, integral distance, truncation, and completion into a Riesz space. Strong ideas include contractive telescoping operators and explicit domain certificates, but the implementation is heavily dependent on proof-carrying partial functions and fixed series witnesses; Hyper’s shared immutable DAG and adaptive interval budgets are lighter and more precise. No worthwhile production change selected.


### `reals/stdlib/CMTIntegrableSets.v` (1170 lines)

Builds characteristic-function integrable sets, extensionality, measures, finite/countable unions and intersections, differences, monotone continuity, restricted integrals, and constructive slice constructions. It reinforces domain-as-certificate and geometric subsequence scheduling, but is measure-specific and witness/allocation-heavy; no worthwhile Hyper scalar change selected.

### `reals/stdlib/CMTMeasurableFunctions.v` (2034 lines)

Defines measurable partial functions/sets over integrable rectangles, closure under complements/unions/intersections, truncation and positive/negative parts, support approximations, generator-based measurable approximation, convergence in measure, and dominated convergence. The recurring ideas are explicit support/domain certificates and geometric (half-power) error schedules; these are measure-theoretic and witness/allocation-heavy, while Hyper already uses lighter DAG certificates and adaptive budgets, so no worthwhile scalar/runtime change was selected.

### `reals/stdlib/CMTProductIntegral.v` (2323 lines)

Constructs product integration spaces and Fubini via finite lists of rectangle terms, boolean subset expansion (`FreeSubsets`), disjoint-grid simplification, coefficient mapping, Riesz closure, product integrals, and monotone/min-limit continuity. It offers explicit disjointization and sparse single-active-term evaluation, but the 2^n list expansion and proof-carrying domains are allocation-heavy; Hyper DAG sharing/adaptive evaluation is preferable, so no worthwhile scalar/runtime change selected.

### `reals/stdlib/CMTprofile.v` (2239 lines)

Develops constructive measure profiles: interval step approximations, monotone bounds, binary subdivision/refinement, crossing/jump-point searches, countability of discontinuities, inverse-image integrability almost everywhere, and positive-measure subset extraction. It uses geometric half-power schedules and extensive proof-carrying monotone searches; these are semantically useful for partial decisions but too witness-heavy for Hyper’s scalar runtime, so no worthwhile change selected.

### `tactics/FieldReflection.v` (958 lines)

Implements reflected field-expression ASTs, typed interpretation/well-formedness, normalization correctness for rational polynomial forms, and Ltac quotation/variable indexing. Canonical expression reflection is useful for proof-time normalization, but Hyper already hash-conses runtime DAGs and does not need Coq tactic machinery; no scalar/runtime change selected.

### `tactics/Qauto.v` (51 lines)

Ltac automation for rational positivity, nonnegativity, and inequalities via normalization; proof-time only, no Hyper change.

### `tactics/Rational.v` (77 lines)

Dispatches rational normalization to field/ring reflection tactics and provides rewrite tacticals; proof-time only, no Hyper change.

### `tactics/Step.v` (47 lines)

Small algebraic Ltac wrappers (`algebra`, `astepl/astepr`, inclusion search); proof-time only, no Hyper change.

### `util/PointFree.v` (74 lines)

Typeclass-driven point-free conversion for lambdas, composition, pairs, and binary applications. It can reduce proof-term boilerplate but has no scalar/runtime impact; no Hyper change.

### `tactics/RingReflection.v` (890 lines)

Reflected commutative-ring ASTs, normalization correctness, and quotation tactics parallel FieldReflection without division. Proof-time only; no Hyper runtime change.

### `util/Extract.v` (111 lines)

Coq extraction mappings to Haskell primitives for naturals, integers, rationals, lists, streams, and comparisons. It highlights replacing unary numerals with machine integers, but Hyper is native Rust and already uses compact integer kernels; no direct change.

### `util/Qdlog.v` (288 lines)

Exact rational discrete-log bounds for bases 2/4, bounded repeated division, monotonicity, and scaling identities. The logarithm bounds could inform exponent seeding, but Hyper already has demand-sized integer-log helpers and no measured gap justified a change.

### `util/Qgcd.v` (104 lines)

Rational gcd/canonical positive gcd and divisibility lemmas. Useful for rational normalization semantics, but Hyper’s dyadic kernels do not need general rational gcd; no change.

### `util/Qsums.v` (241 lines)

Finite rational sum identities, error-ball bounds, heterogeneous sum comparison, and a `fastΣ` accumulator with immediate `Qred` reduction to avoid list allocation. The accumulator/early reduction idea is potentially applicable to Hyper finite rational reductions, but existing kernels already normalize inline and benchmarks show no worthwhile change.

### `util/SetoidPermutation.v` (111 lines)

Setoid-aware list permutation relation and map properness. Proof/list infrastructure only; no Hyper change.

### `write_image/WritePPM.v` (168 lines)

Sparse-raster sorting, duplicate removal, row rasterization, and Elpi file output. Visualization-specific and unrelated to scalar architecture; no change.

### `tactics/csetoid_rewrite.v` (1505 lines)

Implements reflective setoid rewriting for total and partial functions, typed expression interpreters, replacement correctness, formula traversal over Prop/CProp connectives, and automatic detection/folding of partial-function domains. The total/partial split and domain folding are semantically careful but proof/tactic-only; Hyper’s runtime DAG already preserves domains explicitly, so no worthwhile change selected.
