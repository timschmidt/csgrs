# Changelog

## [0.4.0] - Unreleased

- Reject non-finite linear proposals. Normal equations and BFGS could return
  successful NaN steps from finite overflowing inputs, and normal equations
  accepted NaN input. `LinearSolveError` and `ConvergenceReason` gain typed
  non-finite variants. See [the migration note](../../docs/MIGRATIONS.md).

- Keep approximate residual-ball receipts out of exact satisfaction summaries.
  The affine interval-box facade previously promoted approximate receipts,
  including nonzero equality residuals, to `Certified`. Failed-constraint
  diagnostics now retain their qualification.

- Add `DomainCheckStatus::Approximate { valid, stage }`. The domain preflight
  previously counted approximate predicate answers as exact domain
  certificates; a negative square-root operand could be `CertifiedValid`.
  See [the migration note](../../docs/MIGRATIONS.md).

- Evaluate each `finite_difference_jacobian` column at the variable's context
  binding. A rebound free variable previously stepped from its model value, so
  `x²` bound at 3 with model value 1 and step 1 gave −5 instead of 7.

- Require certified comparisons for pivot selection, elimination and
  containment in `certify_affine_krawczyk_box` under both predicate policies.
  Under `APPROXIMATE_512` a terminal approximation could previously certify a
  rootless box, report uniqueness for dependent rows, or accept a root that
  fails an original equation. Such cases now return their unresolved status.

- Report a lower bound when both coefficient-growth counts reach the 32-bit
  reporting ceiling. The diagnostic now preserves the strict inequality
  between the observed bit length and its limit.

- Require certified comparisons when classifying a univariate quadratic
  Krawczyk row. An unresolved containment or contraction comparison can now
  return `Undecided` under `APPROXIMATE_512`.

- Require certified comparisons for univariate quadratic alpha reports under
  both predicate policies. A zero residual with an unresolved derivative now
  returns `Undecided`. An approximate comparison can no longer classify a
  nonroot as an exact simple root.

- Require a certified nonnegative radius for interval-box and Krawczyk
  certificates. An unresolved radius sign returns the existing typed error
  even when the caller permits approximate terminal predicates.

- Compile native bivariate polynomial quotients into shared expression/Jacobian
  programs with exact coefficients, ordered bindings, original denominator guards
  and explicit graph/work budgets.

- Add opt-in sensitivity-retained Taylor noise enclosures for repeated affine
  observations, retaining exact sources, shared selected monomials and every
  omitted contribution in distinct residual bounds.


- Add opt-in certified Taylor model composition, preserving both source proofs,
  full outer-domain inclusion, generated polynomial/cross-remainder evidence and
  the complete outer residual. Generated results accept subsequent model maps.

- Add opt-in exact QR coordinate construction with orthogonality and reconstruction evidence.

- Add opt-in exact source-bound coordinate transitions for direct chart transport.

- Add opt-in polynomial output maps over existing Taylor models, retaining exact
  coefficients, every truncated term and residual cross-product, shared proof
  stages, checked subdomain reuse and source/map replay. `taylor-polynomial`
  leaves established feature sets unchanged. `TaylorRange` reuses the existing
  Cartesian range carrier, including empty/unit distinctions.

- Add source-bound nonlinear program composition with explicit binding selection,
  shared nodes, retained inner domain checks and original-source replay. Existing
  Jacobian/Taylor consumers retain the guard prefix when certifying final values.

- Add opt-in fixed-size Jacobian projections retaining source/domain evidence,
  checked rectangular shapes and shared graph/native copy budgets.

- Add opt-in exact coordinate programs with STRICT inverse proofs, shared graph substitution and replayable source association.
- Add source-bound affine Taylor forms with exact coefficient combination, retained original residual coordinates, subdomain reuse and composed-map replay.
- Add optional demanded sparse normalized partials and exact-coefficient Taylor
  models with explicit certified remainders, source guards and analytic-domain
  evidence. Subdomain reuse preserves the global remainder; replay rebuilds
  coefficients and proof from original definitions. Graph/native budgets retain
  scalar-refinement and heap-byte exclusions.

- Add graph-bounded `ExpressionProgram::jacobian` construction through the
  existing symbolic derivative rules and shared source references. Retain source
  guards, ordered variables and replay by regeneration. This does not establish
  global smoothness or bound all scalar arithmetic and payload costs.
- Correct identity-power derivatives at a zero base while retaining the base's
  original domain guards; `x^1` no longer introduces an artificial `x != 0` guard.

- Add experimental `interval-program` evaluation of owned expressions over
  dyadic input boxes. Preserve
  source guards, including Expr's exclusion of zero-to-zero. Refuse partial
  domains instead of treating their images as complete field evidence. Graph
  and dyadic budgets remain separate; Hyperreal constant refinement and symbolic
  derivative construction still lack whole-operation resource accounting.

- Add experimental `expression-program` support for immutable flat symbolic
  definitions, explicit ordered bindings, validated DAG replay, and exact scalar
  evaluation. Preserve source guards and typed scalar failures. Structural work
  and entry limits include cancellation and retry; they do not bound scalar
  arithmetic or heap bytes. Optional `serde` retains validated Hyperreal scalar
  definitions. This is not an interval field or validated ODE implementation.

- Preserve the exact integer-power derivative at `i64::MIN` using a quotient
  whose exponent remains representable, including repeated differentiation and
  the nonzero-base domain. Report an unavailable dense symbolic-Jacobian
  conversion through `DenseSolverEstimateUnavailable` instead of a zero entry.

- Add `resultant_bivariate_perturbation_ordered_field` and
  `OrderedFieldResultantPerturbation` to retain finite-contact projections beside
  common supports. Reuse the existing ordered-field determinant authority and
  count original coefficient-context calls across all lifted work. A zero
  formal resultant remains explicit; candidate evidence still requires branch,
  domain and denominator validation.
- Complete an exact selected-field sign through the strict counterpart after
  the approximate-selected whole-value attempt declines. Preserve successful
  predicate order, shared work scopes and bounded speculative declines. See the
  [admission](../../docs/research/implicit-shared-sections-2026-11-08.md).

- Expose `bivariate_topology::bivariate_polynomial_gcd` through the checked
  rational-coefficient factor reducer used by curve topology. Hypercurve reuses
  it to distinguish selected and unselected denominator components. Projection
  limits and counted work remain separate from total arithmetic and memory.

- Add `ordered_field_resultant_bivariate` over an existing exact ordered
  coefficient context, including correlated selected algebraic values. Share
  the pivot-free determinant recurrence, preserve original polynomial scale,
  use compact constant-resultant powers and ragged scans, and propagate typed
  context/storage/resource failures. Native topology and rational-map preimage
  integration remain open. See the [qualification](PERFORMANCE.md#r06-selected-coefficient-bivariate-resultants-2026-11-04).

- Decompose rational vertical polynomial content into selected closed interval
  components with defining exponents and residual graph attachments. Preserve
  exact content/quotient reconstruction. Event point roots describe the
  nonvertical part; consult `vertical_components()` for whole intervals.
  `graph_equation()` distinguishes residual support from the full geometry.

- Add `certify_univariate_polynomial_nonnegative` with an explicit policy
  restricted to certified stages. Shared Sturm counts and inward Taylor signs
  cover repeated and endpoint roots; unresolved decisions remain `None`.
  Sign-cell subdivision and coefficient arithmetic have no total-work bound.

- Add `with_coefficient_bit_limit` and `observe_coefficient_bits`. Integer
  polynomial Bareiss entries, pseudo-quotient multiplication matrix entries and
  exact rational dense tensor products report their bit lengths, and one above
  an active limit fails the scope with `WorkKind::CoefficientGrowth` instead of
  being used. The bound is deterministic but covers only these constructions;
  it is not a memory bound.

- Add cooperative cancellation to work-budget scopes. `CancellationToken` can be
  cancelled from any thread; `with_cancellable_budget` then fails the next
  instrumented unit or exhaustion check with `WorkKind::Cancellation` and
  publishes no value. Outer tokens stop nested scopes; inner tokens do not
  poison their parents. `WorkBudget::UNLIMITED` bounds a scope by cancellation
  alone. Cancellation is not a wall-clock deadline: uninstrumented arithmetic
  between checks still runs to completion.

- Reuse one prepared Sturm chain for complete root-isolation counts until the
  temporary polynomial changes by exact deflation, preserving remainder and
  refinement limits.

- Keep selected polynomial field/value diagnostics shallow so shared coefficient
  towers do not expand into diagnostic trees.

- Reduce repeated rational bivariate factors for finite-fiber topology while
  retaining the original polynomial, its exact scalar and factor exponents.
  Checked selected-root and open-branch memberships preserve multiplicities
  separately from the reduced support. Native consumers replay the original
  definition; vertical components and algebraic-coefficient admission remain
  separate capabilities. Configured degree and shared work limits still apply.

- Admit empty topology for first-coordinate-only rational polynomials whose
  complete closed-domain projection has no source roots. Retain original
  equations and degree/work controls; actual vertical components keep their
  typed outcome. Empty fiber queries and native consumers remain exact.

- Normalize invertible native rational quotients and selected-field denominators
  with verified modular or Bezout inverses, retaining nonunit local quotients.
  Proved parent images keep their original relations, multiplicity and selectors.
  Strict sign filters refine selected roots over guarded coefficient enclosures;
  rational coefficients keep their existing exact remainder path and work cost.
  Original source graphs survive enclosure, repeated reciprocals and fresh replay.

- Prove correspondence between independently selected equal generators before
  adjoining an independent coordinate. Retain the original relation, selector,
  multiplicity and source origin while using a certified linear reduction.
  Existing join recipes re-establish the relation during fresh replay; serialized
  coordinate expressions may change. Promotion accounting and typed retry remain.

- Exact scalar witnesses ignore structurally zero tensor padding after selected
  source reduction. Constants and mixed tensors no longer require witnesses
  for unused generators; their native fields and source selections stay intact.

- Evaluate polynomial coefficients from admitted selected fields at represented
  scalars without projecting their dependencies. Exact arithmetic reuses stored
  zero and unit identities after field alignment; division still validates its
  denominator before publishing an admitted value.

- Add configured-depth use of the complete ordered-field Sturm isolator.
  Separation and singleton refinement share the same implementation as the
  unrestricted API. Reached local depth limits return `BudgetExhausted` with
  no partial roots, while shared work accounting remains active.

- Selected-field joins retain validated source-generator correspondences.
  Reusing either original operand or a corresponding generator from another
  join preserves its existing authority instead of adjoining a duplicate root.
  Definition graphs retain the join recipe and re-establish these relations
  through strict source and selection replay.
  Replay validates original rational denominators before retaining shared
  coordinate nodes, preserving native re-export structure. Negative
  denominators remain valid after exact sign normalization; zero stays invalid.

- Selected polynomial joins retain the first native base when the ordered
  source union adds no axes and the existing foreign-field embedding adds no
  radical. Subsequent joins preserve polynomial ancestors instead of appending
  those generators again. New radical extensions keep the existing replay path.

### Breaking

- `SelectedPolynomialDefinitionNode` gains `JoinedField`; exhaustive matches
  and graph format owners must handle the new operation record. Existing
  records retain their meanings. Joined fields intentionally export their
  construction dependencies instead of an unrelated polynomial selection.

- Prepared problem objects are replaced by immediate analysis. The `prepared`
  module and `PreparedProblem`, `PreparedProblemFacts` and
  `PreparedConstraintFacts` become `ProblemAnalysis` (returned by the new
  `Problem::analyze`), `ProblemFacts` and `ConstraintFacts` in the `analysis`
  module; `facts_depend_on_symbol` stays at the crate root.
  `symbolic_jacobian_prepared` becomes `symbolic_jacobian_with_analysis`.
- Prepared residual and block types are renamed: `PreparedAffineResidual`,
  `PreparedQuadraticResidual`, `PreparedUnivariateQuadraticResidual`,
  `PreparedLinearTerm`, `PreparedQuadraticTerm`, `PreparedSolverBlock` and
  `PreparedSolverBlockFacts` become `AffineResidual`, `QuadraticResidual`,
  `UnivariateQuadraticResidual`, `QuadraticLinearTerm`, `QuadraticTerm`,
  `SolverBlock` and `SolverBlockFacts`. Replace `prepare_affine_residual`,
  `prepare_quadratic_residual` and `prepare_univariate_quadratic_residual`
  with each type's `from_expr`. `SolverBlockRowKind::PreparedAffine` and
  `AffineUnprepared` become `AffineForm` and `AffineWithoutForm`, and
  `ExprEvalError::PreparedShapeMismatch` becomes `ResidualShapeMismatch`.
- `PreparedSparseLinearSystem`, `prepare_sparse_linear_residual_system` and
  `replay_sparse_linear_residual_batch_prepared` are replaced by
  `SparseLinearSystem::from_terms` and its `replay_candidate` and
  `replay_batch` methods.
- `build_equality_substitution_classes`, `build_sketch_workplane_frame` and
  `build_sketch_workplane_frame_with_policy` are renamed
  `equality_substitution_classes`, `sketch_workplane_frame` and
  `sketch_workplane_frame_with_policy`.
- `solve_dense_linear_system_bareiss`, `replay_dense_linear_residuals` and
  `arithmetic_algebraic_root_representations` take an explicit
  `PredicatePolicy`. Reports record the evidence consumed:
  `BareissSolveReport::certainty`, `BareissDeterminantReport::method`,
  `DenseResidualReplayReport::certainty` and
  `DenseResidualReplayRow::certainty` are new, dense replay rows no longer
  store `residual`, and `CertifiedCandidateStatus::BallCertified` gains
  `certainty` and `satisfied`.
- `AlgebraicRootKind` and `AlgebraicRootRepresentation::kind` are removed.
  `exact_rational_witness` is replaced by `exact_point_witness`, which can
  return any exact `Real`, not only a rational.
  `CurveIntersectionResultantSample` and
  `CurveIntersectionResultantReport::samples` are removed; reports retain the
  resultant coefficients without reconstruction samples.
- Exhaustive status and error enums gained variants, which also shifts their
  implicit discriminants. Algebraic-root arithmetic, binary-transform,
  polynomial-image and rational-image statuses and
  `CurveIntersectionResultantStatus` gain `BudgetExhausted`; arithmetic and
  evaluation statuses gain `ComputedExactRealWitness` or
  `EvaluatedExactRealWitness` for exact `Real` results without a rational
  payload; `AffineIntervalError` and `QuadraticIntervalError` gain
  `UnknownVariableRadiusSign`; `AlgebraicRootRationalImageStatus` gains
  `ImageIntervalDisjoint`.
- `AlgebraicRootPolynomialImageStatus::NonMonotoneImage` is replaced by
  `ImageIsolationFailed`. A polynomial image no longer has to be monotone on
  the source interval; it fails only when bounded refinement cannot certify
  one image root.
- Requires `hyperreal` 0.14, `hyperlattice` 0.7 and `hyperlimit` 0.5; `Real`
  and the re-exported Hyperlimit predicate types come from those releases.

### Added

- Complete ordered-field root isolation reuses exact Sturm counts for simple
  and repeated roots, retaining original-equation selectors and shared work
  accounting. The bounded Bernstein entry point keeps its fallback contract.

- Shared selected-polynomial field/value definitions retain original selectors,
  native radicals and rational coordinates. Strict replay validates every record
  and denominator before publishing outputs, with an explicit graph-node limit.
  The scalar graph supplies format owners with exact data; geometric native
  interchange still requires its own integration.

- Certified closed bounds for retained selected-polynomial values validate
  containment and refine by exact comparisons without independent root images.
  Shared refinement limits preserve typed refusal and larger-budget retry.

- Exact joins of independently selected polynomial fields retain native and
  polynomial generator embeddings, original singleton selectors, and finite
  denominator proofs for further arithmetic and selected constructions.

- Retain arbitrary finite-degree polynomial extensions and dependent rational
  values over selected coefficient fields. Exact root counting validates open
  singleton selectors, including repeated roots; denominator guards are checked
  at the selected root. Native quadratic fields remain shared dependencies.
- Compose bivariate equations with polynomial homogeneous coordinates in one
  retained recursive coefficient field. The result preserves selected algebraic
  dependencies and denominator factors for subsequent exact root isolation;
  it does not select a geometric branch or extend a rational chart domain.
- `PredicatePolicy`, `PredicateOutcome`, `PredicateCertainty`,
  `PredicateSign`, `classify_real_sign_predicate` and
  `compare_real_predicate` are re-exported from Hyperlimit.
- The `work_budget` module (`with_budget`, `WorkBudget`, `BudgetOutcome`,
  `BudgetExceeded`) limits instrumented root-subdivision and
  algebraic-promotion attempts across nested synchronous calls and reports
  exhaustion as a typed failure. It does not bound every solver path, scalar
  arithmetic, memory or elapsed time.
- Exact multivariate polynomials: `BivariatePolynomial`,
  `TrivariatePolynomial`, `QuadrivariatePolynomial` and
  `DenseTensorPolynomial`, with exact division
  (`divide_univariate_polynomial_exact`, `divide_bivariate_polynomial_exact`),
  `greatest_common_divisor_univariate_polynomials_exact`, `square_free_part`,
  `univariate_polynomials_modular_coprimality`, and the `exact_factor`,
  `quotient_ring`, `bivariate_arithmetic` and `trivariate_arithmetic`
  modules.
- Exact eliminations: `resultant_bivariate_polynomial_system`, trivariate,
  quadrivariate and tensor constraint resultants and subresultants (for
  example `resultant_trivariate_polynomial_univariate_constraint` and
  `subresultant_trivariate_polynomial_system`), bivariate component and
  axis-factor extraction, and parameter lifts
  (`linear_parameter_lifts_bivariate_polynomial_system`).
- Algebraic fibers: count, isolate and project the roots of a bivariate
  polynomial at an algebraic parameter
  (`count_bivariate_fiber_roots_at_algebraic_parameter`,
  `isolate_bivariate_fiber_roots_at_algebraic_parameter`,
  `project_bivariate_fiber_at_algebraic_parameter`).
- Represented algebraic roots: the `represented_root` module for exact values,
  intervals and ratios built from isolated roots, `RealInterval`,
  `square_root_algebraic_root_representation`,
  `represent_algebraic_tensor_image`,
  `transform_algebraic_root_rational_images`,
  `algebraic_root_affine_relation`, `translated_algebraic_root_difference`
  and `compose_univariate_polynomial_linear_fractional`.
- Ordered-field root tools (`isolate_ordered_field_polynomial_roots`,
  `ordered_field_sign_at_selected_root`, `sign_at_selected_root`,
  `sign_at_selected_tuple`) and the `recursive_quadratic_field` and
  `radical_expression` modules.
- `solve_dense_linear_system_bareiss_multi_rhs`,
  `solve_sparse_linear_system_bareiss_minimum_degree`, and
  `project_origin_onto_simplex` (also
  `project_origin_onto_simplex_with_config`), which returns the exact closest
  point, squared distance and barycentric evidence for small convex hulls.

### Changed

- License is now `MIT OR Apache-2.0` (was `Apache-2.0`).
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.3.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.
- `ordered_field_polynomial_sign_remainder` reduces a dividend at least four
  degrees above the modulus along its Horner sequence. The remainder is the
  same polynomial; a degree-192 dividend by a degree-6 modulus takes 2,430
  field products instead of 19,635.
- Quotient-ring reductions of exact rational polynomials, including tensor
  reduction at selected source tuples and local algebraic field reduction,
  use integer pseudo-division without first removing the dividend's integer
  content. Results are unchanged. With the previous item, Hypercurve's
  chamfered bevel stroke round trip takes about 15 s instead of about 60 s.

### Fixed

- `certify_multivariate_quadratic_krawczyk_box` no longer scales its
  contraction bound by the variable radii. The old bound could report
  `CertifiedUniqueRoot` for a box that contains two roots.
- `certify_candidate_domains` checks that the base of `powi(0)` is nonzero
  (`DomainCheckKind::ZeroPowerBaseNonZero`), because Hyperreal leaves `0^0`
  undefined. Previously only negative exponents were checked.
- Curve-intersection resultants report an undecided vanishing top coefficient
  as `UndecidedCoefficient` instead of `InterpolationDivisionFailed`.

### Removed

- The placeholder `hypersolve` binary target.

For 0.3.0 and earlier, see [crates.io](https://crates.io/crates/hypersolve/versions).
