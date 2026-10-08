# Stack Exactness-Policy File Audit

Generated from the live worktree by `scripts/generate_stack_exactness_file_audit.sh`.
The audit universe is every Rust source, Cargo manifest, and build script in each
crate that implements or consumes the Hyper exact-arithmetic stack. Generated
artifacts, vendored dependencies, binary assets, and prose-only files are excluded.
Tests, examples, benches, and fuzz targets are included because they can encode or
conceal incorrect predicate assumptions.

`hypervoxel`, `hyperbrep`, and Hypersdf's optional Hypervoxel adapter are
intentionally deferred from this pass at the user's request. Their entries remain unchecked
and do not count toward this pass's completion.

## Status legend

- `[ ]`: not yet manually reviewed.
- `[x]`: reviewed against the centralized Hyperlimit policy and predicate boundary.
- Signal tags are search aids, not findings. `policy` means direct Hyperlimit policy
  or predicate use; `scalar` means local sign/order/equality or approximation code;
  `none` means no lexical signal and still requires a manual applicability check.

## Accepted fast-path lifts

### Borrowed ordered 2D AABBs

Hypercurve's certified `Aabb2::contains_point` and `Aabb2::overlaps` paths now
delegate to coordinate-borrowed Hyperlimit predicates. Preview mode retains its
explicit tolerance behavior, and builds without Hyperlimit predicates retain
the local exact fallback.

The gate compared clean `HEAD` checkouts with the candidate, serially and pinned
to one CPU. All checksums and classification counts matched.

- Hyperlimit ordered intersection: 13.50 ns.
- Hyperlimit ordered point membership: 13.39 ns.
- Hypercurve containment rows: 2.0% to 8.5% faster.
- Hypercurve intersection rows: 0.8% to 18.3% faster.
- CSGRS rectangle construction: 386.04 to 386.54 ns/op (+0.13%, unchanged).
- CSGRS polygon construction: 291.97 to 282.20 ns/op (3.3% faster).

The CSGRS `curved_line_arc_union` profile row is not a valid gate: the clean
baseline currently fails with `ExactCurve(Invalid { cause: ZeroLengthLine })`.
That pre-existing failure remains an open audit finding.

### Hyperphysics scalar comparisons

Hyperphysics contact and ordered-shape comparisons now pass both operands
directly to Hyperlimit's exact-rational-first comparison cascade instead of
constructing a temporary `left - right` expression. Its default GJK and mass
certificate refinement ceilings now use
`PredicatePolicy::MAX_REFINEMENT_PRECISION`.

The full Hyperphysics benchmark executable was run serially and pinned to one
CPU before and after the change. Checksums matched. The directly affected AABB
contact row improved from 515 ns to 366-372 ns (about 28%); the mixed shape row
improved from 1.209 us to 1.150-1.166 us. Other rows varied within run-to-run
noise. A proposed direct photochemistry comparison was rejected and reverted
after its row slowed from 754 ns to 773 ns.

### Paired scalar-sign cascade

Hyperlimit now batches two scalar sign queries through one exact-rational fast
path while retaining the weakest certainty and latest escalation stage on the
general path. Hypersdf uses it for cylinder, capsule, and torus domain checks.

Serial CPU-pinned Criterion gates measured 11.9 ns for the paired cascade
versus 31.3 ns for two composed scalar cascades (about 62% faster). Hypersdf
point classification improved to 234.9 ns for cylinders, 244.8 ns for capsules,
and 370.2 ns for tori. Capsule intervals improved to 818.1 ns, torus intervals
to 1.002 us, and the cylinder interval row was statistically unchanged.

## `hyperreal`

Hyperreal cannot depend on Hyperlimit without introducing a dependency cycle,
so its responsibility is to expose exact structural facts, certificate-bearing
bounded-refinement queries, and explicit uncertainty. The file-by-file review
used these dispositions:

- The manifest and module/CLI/error wiring contain no scalar decisions.
- Rational implementation files use exact integer/rational arithmetic only;
  algorithm dispatch changes cost, not value or predicate semantics.
- Computable constructors and approximation kernels preserve the exact
  expression graph and stated integer error bounds. Rough probes select only
  value-equivalent evaluation schedules. Presentation uses the documented
  lossy sign/approximation boundary.
- Real arithmetic now uses certified sign/order/zero queries for domain,
  endpoint, reciprocal, and quadrant decisions. Optimization-only unknown
  signs take branch-independent exact constructions. Primitive floats occur
  only at exact IEEE import, explicit lossy export, display, or numerical seed
  boundaries.
- Benches, examples, tests, and fuzz targets were checked for semantic misuse.
  Their floating tolerances are oracle/quality checks, and the refinement
  example labels `compare_absolute` equality as “within tolerance.”
- The all-feature test suite, doc tests, clippy with warnings denied, formatting,
  and diff checks pass. Newly added proof-state APIs are explicitly classified
  by the GMP API coverage guard.

- [x] `hyperreal/Cargo.toml` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/adversarial_library.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/adversarial_transcendentals.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/borrowed_ops.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/dispatch_trace.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/float_convert.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/gmp_api.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/library_perf.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/numerical_micro.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/scalar_micro.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/benches/support/bench_docs.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/examples/computable_graphs.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/examples/computable_refinement_steps.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/examples/readme_quickstart.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/fuzz/Cargo.toml` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/fuzz/fuzz_targets/computable_approximation.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/fuzz/fuzz_targets/rational_arithmetic.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/fuzz/fuzz_targets/real_elementary.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/fuzz/fuzz_targets/real_exact.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/fuzz/fuzz_targets/structural_representations.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/approximation.rs` — signals: `scalar`; disposition: module wiring only; approximation kernels retain BigInt error-bounded semantics.
- [x] `hyperreal/src/computable/approximation/arithmetic_kernels.rs` — signals: `scalar`; disposition: exact-real approximation planning preserves stated integer error bounds; inconclusive magnitude probes refine or return approximation-scale zero.
- [x] `hyperreal/src/computable/approximation/constants.rs` — signals: `scalar`; disposition: constants use rational identities and rounded BigInt evaluation; primitive floats only size iteration work.
- [x] `hyperreal/src/computable/approximation/dispatch.rs` — signals: `scalar`; disposition: exhaustive kernel dispatch only; no independent decisions.
- [x] `hyperreal/src/computable/approximation/exp_sqrt.rs` — signals: `scalar`; disposition: guarded BigInt series/Newton approximation kernels; no primitive-float predicate decisions.
- [x] `hyperreal/src/computable/approximation/inverse_hyperbolic.rs` — signals: `scalar`; disposition: deferred exact identities and guarded BigInt series; domain decisions remain in certified Real APIs.
- [x] `hyperreal/src/computable/approximation/inverse_trig.rs` — signals: `scalar`; disposition: exact-rational range reductions and error-budgeted BigInt series; no lossy topology/equality decisions.
- [x] `hyperreal/src/computable/approximation/logarithms.rs` — signals: `scalar`; disposition: exact-rational reductions and guarded atanh series preserve approximation contract.
- [x] `hyperreal/src/computable/approximation/representation.rs` — signals: `scalar`; disposition: expression-kernel representation only; exact rational payloads are retained.
- [x] `hyperreal/src/computable/approximation/statistics.rs` — signals: `scalar`; disposition: demand-driven BigInt approximations; public certified domain checks precede these kernels.
- [x] `hyperreal/src/computable/approximation/trig.rs` — signals: `scalar`; disposition: exact-rational sector filters retain safe reduced ranges and BigInt error budgets; uncertain general reductions refine.
- [x] `hyperreal/src/computable/constants.rs` — signals: `scalar`; disposition: documentation facade only.
- [x] `hyperreal/src/computable/format.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/mod.rs` — signals: `scalar`; disposition: module/export wiring only.
- [x] `hyperreal/src/computable/node.rs` — signals: `scalar`; disposition: shared imports and implementation includes only; decision logic is audited in included files.
- [x] `hyperreal/src/computable/node/algebra.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/approximation_queries.rs` — signals: `scalar`; disposition: bounded sign/order queries preserve explicit uncertainty; fixed mixed exact-leaf signed-order bug in `compare_absolute`.
- [x] `hyperreal/src/computable/node/bounds.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/exp_trig.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/logarithms.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/primitive_constructors.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/representation.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/roots_inverse_hyperbolic.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/scale.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/structural_analysis.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/computable/node/tests.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/dispatch_trace.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/lib.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/main.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/problem.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/aggregate_products.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/algorithm_dispatch.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/as_ref.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/barrett_division.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/comparison.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/construction.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/format_parse.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/ntt_multiplication.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/ops.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/queries_conversion.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/representation.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/squares_powers.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/tests.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/toom4_multiplication.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/toom6_multiplication.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/arithmetic/toom8_multiplication.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/convert.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/mod.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/rational/parse.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/approximation.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/add_sub.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/canonical_constants.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/classification.rs` — signals: `scalar`; disposition: exact symbolic-class certificates use rational inequalities only; primitive approximation cache is non-semantic.
- [x] `hyperreal/src/real/arithmetic/comparison.rs` — signals: `scalar`; disposition: `PartialOrd` delegates to certified bounded comparison and returns `None` on uncertainty; primitive inputs are first imported exactly.
- [x] `hyperreal/src/real/arithmetic/elementary_functions.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/facts.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/format_parse.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/inversion.rs` — signals: `scalar`; disposition: reviewed; low-level preclassified reciprocal hook avoids repeating a policy decision while retaining exact arithmetic.
- [x] `hyperreal/src/real/arithmetic/linear_algebra.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/mul_div.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/representation.rs` — signals: `scalar`; disposition: certified rounding/aggregate helpers propagate `Exhausted` on unresolved decisions; `abs` uses an exact sqrt-square fallback.
- [x] `hyperreal/src/real/arithmetic/structural_helpers.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/arithmetic/tests.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/constructors.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/convert.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/exact_set.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/facts.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/linear_combination.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/mod.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/normal_reference.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/real/tests.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/serde.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/simple.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/src/structural.rs` — signals: `policy,scalar`; disposition: certificate/result vocabulary explicitly separates known sign/order/equality from bounded-refinement uncertainty.
- [x] `hyperreal/src/trace.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/adversarial_props.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/adversarial_regressions.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/adversarial_semantics.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/gmp_api_coverage.rs` — signals: `policy,scalar`; disposition: reviewed; the preclassified reciprocal hook is explicitly classified as a policy-wrapper-only API.
- [x] `hyperreal/tests/numeric_roundtrip_formats.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/numerical_cross_reference.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/public_numeric_semantics.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/rational_oracle.rs` — signals: `scalar`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/readme.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.
- [x] `hyperreal/tests/readme_metadata.rs` — signals: `none`; disposition: reviewed; see the Hyperreal file-group dispositions above the checklist.

## `hyperlattice`

Hyperlattice sits below Hyperlimit, so it must preserve exact expressions,
publish conservative structural facts, and propagate uncertainty instead of
making policy decisions. The file-by-file review used these dispositions:

- Manifests, facades, containers, tracing, arbitrary-data support, and module
  wiring contain no semantic scalar decisions.
- Vector, matrix, projective, and algebra kernels retain `Real` expressions.
  Sparse/factor fast paths prune only values proved zero; unknown facts take
  the generic exact path. Checked division and inversion propagate
  `UnknownZero`.
- The bounded rotation-sign probe consumes only a returned certificate;
  unresolved probes use the checked generic construction. Primitive floats
  occur only at exact imports, explicit lossy exports, benchmark oracles, and
  fuzz/test quality checks.
- Complex integer powers now classify `0^0` conservatively: proven zero is
  `NotANumber`, unresolved zero is `UnknownZero`, and only proven nonzero input
  yields one. An exact-rational real-component fast path avoids adding overhead
  to the common case.
- Benches, examples, tests, and fuzz targets were checked for semantic misuse.
  The all-feature tests and doc test, clippy with warnings denied, formatting,
  and diff checks pass. A serial CPU-pinned paired sentinel measured the fixed
  known-nonzero `powi(0)` path at 33.458 ns versus 33.855 ns clean baseline.

- [x] `hyperlattice/Cargo.toml` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/api_dispatch_trace.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/borrowed_ops.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/comparisons.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/complex_ops.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/dispatch_trace.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/engines.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/fixtures.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/matrix_ops.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/report.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/scalar_ops.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/mathbench/vector_ops.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/benches/regression_sentinels.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/examples/readme_quickstart.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/fuzz/Cargo.toml` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/fuzz/fuzz_targets/complex_ops.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/fuzz/fuzz_targets/matrix_ops.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/fuzz/fuzz_targets/scalar_ops.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/fuzz/fuzz_targets/vector_ops.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/aabb.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/algebra2.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/arbitrary_impls.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/complex.rs` — signals: `scalar`; disposition: conservative complex zero classification fixed and benchmark-gated.
- [x] `hyperlattice/src/error.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/kernels.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/lib.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/batch.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/core.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/determinant.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/inverse.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/mod.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/ops.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/transform.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/matrix/types.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/point.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/projective.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/scalar.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/trace.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/src/vector.rs` — signals: `policy,scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/adversarial_borrowed_owned.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/adversarial_constraint_props.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/adversarial_matrix_invariants.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/adversarial_props.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/adversarial_regressions.rs` — signals: `scalar`; disposition: regression coverage for unresolved complex `0^0` classification.
- [x] `hyperlattice/tests/adversarial_scalar_semantics.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/borrowed_ops.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/common/mod.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/complex.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/geometric.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/gmp_benchmark_coverage.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/matrix.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/readme.rs` — signals: `none`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/scalar.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.
- [x] `hyperlattice/tests/vector.rs` — signals: `scalar`; disposition: reviewed; see the Hyperlattice file-group dispositions above the checklist.

## `hyperlimit`

- [x] `hyperlimit/Cargo.toml` — signals: `policy`; disposition: dependency/features and serial Criterion gates reviewed; no policy bypass.
- [x] `hyperlimit/benches/benchmark_report.rs` — signals: `policy`; disposition: report generation only; consumes Criterion estimates without geometric decisions
- [x] `hyperlimit/benches/dispatch_trace.rs` — signals: `policy,scalar`; disposition: exact/filtered/refined dispatch counters reviewed; no certainty relabeling
- [x] `hyperlimit/benches/predicates.rs` — signals: `policy,scalar`; disposition: predicate provenance and AABB controls reviewed; added ordered-coordinate and paired-sign fast-path gates.
- [x] `hyperlimit/examples/readme_quickstart.rs` — signals: `policy`; disposition: centralized policy example and Unknown handling reviewed
- [x] `hyperlimit/examples/write_benchmarks_md.rs` — signals: `none`; disposition: benchmark Markdown writer only; no Real decisions
- [x] `hyperlimit/fuzz/Cargo.toml` — signals: `policy`; disposition: both declared fuzz binaries compile with current APIs
- [x] `hyperlimit/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `policy`; disposition: strict/default representation-equivalence behavior and certainty invariants reviewed
- [x] `hyperlimit/fuzz/fuzz_targets/predicate_invariants.rs` — signals: `policy,scalar`; disposition: exact symmetry/permutation/incidence invariants and Unknown handling reviewed
- [x] `hyperlimit/src/batch.rs` — signals: `policy,scalar`; disposition: sequential/parallel batches forward one explicit policy to every scalar predicate
- [x] `hyperlimit/src/classify.rs` — signals: `policy`; disposition: discrete relation helpers only; no independent numeric threshold
- [x] `hyperlimit/src/error.rs` — signals: `none`; disposition: error types only
- [x] `hyperlimit/src/geometry/facts.rs` — signals: `policy,scalar`; disposition: structural facts are advisory metadata and never terminal approximate decisions
- [x] `hyperlimit/src/geometry/homogeneous.rs` — signals: `policy`; disposition: exact projective constructions retain points at infinity; incidence uses predicate cascade
- [x] `hyperlimit/src/geometry/mod.rs` — signals: `none`; disposition: module/re-export surface only
- [x] `hyperlimit/src/geometry/plane.rs` — signals: `policy,scalar`; disposition: plane evidence, side, segment, triangle, and AABB cascades preserve certainty and Unknown
- [x] `hyperlimit/src/geometry/point.rs` — signals: `policy`; disposition: exact point carriers and arithmetic only; no local equality threshold
- [x] `hyperlimit/src/lib.rs` — signals: `policy`; disposition: public predicate/policy boundary reviewed; canonical comparison, reciprocal, borrowed AABB, and explicit-policy 3D orientation cascades are exported.
- [x] `hyperlimit/src/orient.rs` — signals: `policy`; disposition: public compatibility re-exports include both centralized-default and explicit-policy 3D orientation.
- [x] `hyperlimit/src/plane.rs` — signals: `policy`; disposition: public compatibility re-exports only
- [x] `hyperlimit/src/predicate.rs` — signals: `policy,scalar`; disposition: centralized STRICT/APPROXIMATE_512 policy and certainty provenance reviewed; retained.
- [x] `hyperlimit/src/predicates/aabb.rs` — signals: `policy,scalar`; disposition: policy forwarding, Unknown propagation, exact-rational stages, and ordered box semantics reviewed; added borrowed ordered-AABB2 cascade.
- [x] `hyperlimit/src/predicates/convex.rs` — signals: `policy,scalar`; disposition: halfspace composition merges weakest certainty/latest stage and propagates Unknown
- [x] `hyperlimit/src/predicates/coplanar.rs` — signals: `policy,scalar`; disposition: projection is certified by exact normal components; no epsilon projection choice
- [x] `hyperlimit/src/predicates/distance.rs` — signals: `policy,scalar`; disposition: squared/scaled distance predicates avoid roots/division where possible and forward explicit policy
- [x] `hyperlimit/src/predicates/dop.rs` — signals: `policy,scalar`; disposition: slab construction/classification and validation fail closed on Unknown; retained witnesses replay exactly
- [x] `hyperlimit/src/predicates/exact.rs` — signals: `policy`; disposition: compact exact-rational determinant kernels only
- [x] `hyperlimit/src/predicates/filters.rs` — signals: `policy,scalar`; disposition: interval/ball filters preserve endpoint/radius certainty provenance (including policy-authorized Approximate), reject unresolved/invalid radii, and escalate crossing enclosures; paired pinned ball-sign benchmark improved from 36.15 µs to 34.93 µs.
- [x] `hyperlimit/src/predicates/halfspace.rs` — signals: `policy,scalar`; disposition: feasibility/certificate search forwards policy, merges provenance, and rejects unresolved validation
- [x] `hyperlimit/src/predicates/interval.rs` — signals: `policy,scalar`; disposition: endpoint/order/intersection cascades preserve explicit policy and Unknown
- [x] `hyperlimit/src/predicates/mod.rs` — signals: `policy`; disposition: module surface only
- [x] `hyperlimit/src/predicates/nd.rs` — signals: `policy,scalar`; disposition: arity failures are explicit Unknown; exact-rational determinant and Real fallback preserve policy
- [x] `hyperlimit/src/predicates/order.rs` — signals: `policy,scalar`; disposition: canonical scalar sign/order/reciprocal entry points classify once, forward explicit policy, and preserve paired exact-rational evidence.
- [x] `hyperlimit/src/predicates/orient.rs` — signals: `policy,scalar`; disposition: structural/word/rational/Real orientation cascades retain exact evidence and policy provenance; the existing 3D policy entry point is now public.
- [x] `hyperlimit/src/predicates/ring.rs` — signals: `policy,scalar`; disposition: area/convexity/even-odd decisions use explicit cascades; structural zero facts remain advisory counters
- [x] `hyperlimit/src/predicates/segment.rs` — signals: `policy,scalar`; disposition: point/segment/compound intersection policy variants exported; decision traces preserve weakest certainty
- [x] `hyperlimit/src/predicates/segment_plane.rs` — signals: `policy`; disposition: exact ratio construction and explicit construction-failure states reviewed
- [x] `hyperlimit/src/predicates/triangle.rs` — signals: `policy,scalar`; disposition: point/segment/ray/tetrahedron predicates merge all component certainty and fail closed on unresolved construction
- [x] `hyperlimit/src/predicates/triangle_triangle.rs` — signals: `policy,scalar`; disposition: plane separation, noncoplanar edges, and coplanar fallback propagate caller policy and Unknown
- [x] `hyperlimit/src/real.rs` — signals: `policy,scalar`; disposition: structural/certified sign knowledge mapping reviewed; it supplies evidence and does not make terminal topology decisions.
- [x] `hyperlimit/src/resolve.rs` — signals: `policy,scalar`; disposition: full structural/filter/exact/refined/policy-authorized approximate cascade reviewed; Approximate and Unknown provenance preserved.
- [x] `hyperlimit/src/trace.rs` — signals: `none`; disposition: optional dispatch instrumentation only; does not affect outcomes
- [x] `hyperlimit/tests/adversarial_degeneracies.rs` — signals: `policy`; disposition: exact dyadic/subnormal, near-degenerate, and boundary cases pass
- [x] `hyperlimit/tests/adversarial_predicates.rs` — signals: `policy`; disposition: strict sign/order, structural facts, and batch/scalar hostile cases pass
- [x] `hyperlimit/tests/adversarial_props.rs` — signals: `policy,scalar`; disposition: generated invariance/incidence/clearance properties pass with exact integer oracles
- [x] `hyperlimit/tests/batch.rs` — signals: `policy,scalar`; disposition: sequential/parallel batches match scalar predicates and strict policy
- [x] `hyperlimit/tests/halfspace_feasibility.rs` — signals: `policy`; disposition: feasible witnesses and exact Farkas certificates pass
- [x] `hyperlimit/tests/nd_predicates.rs` — signals: `policy`; disposition: exact N-D orientation/insphere and invalid-arity Unknown cases pass
- [x] `hyperlimit/tests/promoted_mesh_helpers.rs` — signals: `policy`; disposition: retained report/source replay and support-DOP refresh tests pass
- [x] `hyperlimit/tests/readme.rs` — signals: `none`; disposition: example/release metadata checks pass
- [x] `hyperlimit/tests/robust_parity.rs` — signals: `policy,scalar`; disposition: 2D/3D orientation and circle/sphere predicates match Shewchuk adaptive references
- [x] `hyperlimit/tests/support_dop.rs` — signals: `policy`; disposition: witness, slab, AABB, plane, degeneracy, and forged-evidence validation tests pass

## `hypersolve`

- [x] `hypersolve/Cargo.toml` — signals: `policy`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/benches/certification.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/benches/dispatch_trace.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/examples/basic.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/Cargo.toml` — signals: `policy`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/active_set_quadratic_regeneration.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/algebraic_binary.rs` — signals: `policy`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/algebraic_difference_comparison.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/algebraic_mobius.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/algebraic_polynomial_image.rs` — signals: `policy`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/algebraic_rational_image.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/curve_resultant.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/curve_substitution.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/dense_bareiss_multi_rhs.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/failed_constraint_minimal_removals.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/modified_newton_bounded_quadratic_seed.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/modified_newton_bounded_substitution_seed.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_arc_arc_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_arc_cubic_second_order.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_arc_cubic_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_arc_line_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_circle_circle_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_concentric.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_cubic_cubic_c2.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_cubic_cubic_g2.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_cubic_cubic_tangent.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_cubic_line_tangent.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_failed_constraints.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_line_arc_length.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_line_arc_sweep_length.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_oriented_angle.rs` — signals: `policy`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_point_line_incidence.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_point_on_arc.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_point_on_cubic.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_arc_cubic_curve_second_order_contact.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_arc_cubic_curve_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_arc_line_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_cubic_curve_cubic_curve_c2.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_cubic_curve_cubic_curve_g2.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_cubic_curve_cubic_curve_tangent.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_cubic_curve_line_tangent.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_cubic_line_tangent.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_distance.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_distance_range.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_equal_length.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_equal_point_distance_point_line_distance.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_equal_point_line_distances.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_equal_point_point_distances.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_length_difference.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_length_point_line_distance.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_length_ratio.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_line_arc_sweep_length.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_line_circle_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_line_length_range.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_line_orientation.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_line_radius_equality.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_line_symmetry.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_oriented_angle.rs` — signals: `policy`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_concentric.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_distance_difference.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_distance_ratio.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_line_distance.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_line_distance_range.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_line_radius_equality.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_on_arc.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_on_circle.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_on_cubic.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_on_cubic_curve.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_projected_point_radius_equality.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sketch_workplane_symmetry.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/fuzz/fuzz_targets/sparse_pattern_preserving_bareiss.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/active_set.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/affine.rs` — signals: `scalar`; disposition: retained exact affine evaluation reviewed; a remaining `x^0` is no longer collapsed to a total constant, preserving its nonzero-base domain obligation.
- [x] `hypersolve/src/algebraic.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/algebraic_binary.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/algebraic_mobius.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/algebraic_polynomial_image.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/algebraic_rational_image.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/alpha.rs` — signals: `policy,scalar`; disposition: alpha bounds now use branch-independent exact Hyperreal absolute values and explicit policy outcomes; no structural-sign-only magnitude failure remains.
- [x] `hypersolve/src/analysis.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/bareiss.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/batch.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/branches.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/certification.rs` — signals: `policy,scalar`; disposition: inequality orientation/satisfaction semantics corrected; ball certificates retain certainty and satisfaction separately, approximate provenance is preserved, and invalid radii count as domain failures.
- [x] `hypersolve/src/curve_resultant.rs` — signals: `scalar`; disposition: exact resultant validation is policy-driven; the default refinement floor now comes from the centralized policy constant.
- [x] `hypersolve/src/curve_substitution.rs` — signals: `scalar`; disposition: exact B-spline/NURBS span validation is policy-driven; the default refinement floor now comes from the centralized policy constant.
- [x] `hypersolve/src/diagnostics.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/direct.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/domain/geometry.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/domain/mod.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/domain_certification.rs` — signals: `policy,scalar`; disposition: explicit zero-power nonzero-base checks added alongside division/negative-power domains; all checks preserve CertifiedInvalid, Unknown, and evaluation-failure distinctions.
- [x] `hypersolve/src/eval.rs` — signals: `scalar`; disposition: proposal hinge evaluation now uses branch-independent exact positive-part construction for unresolved signs; lossy conversion remains proposal-only.
- [x] `hypersolve/src/failed_constraints.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/integer_interpolation.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/interval.rs` — signals: `policy,scalar`; disposition: variable radii are certified with the caller policy (negative and unresolved are distinct failures); exact abs removes obsolete magnitude-Unknown branches and accelerates the quadratic interval benchmark.
- [x] `hypersolve/src/jacobian.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/lib.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/linalg.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/model.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/polynomial.rs` — signals: `scalar`; disposition: univariate and multivariate collectors reject retained `x^0`, preventing domain-sensitive expressions from masquerading as total polynomial constants.
- [x] `hypersolve/src/predicates.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/rank.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/residual_replay.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/resultant.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/root_isolation.rs` — signals: `policy,scalar`; disposition: exact Hyperreal absolute value replaces structural-sign magnitude branching; isolation/refinement continues to return explicit bounded Unknown/error states.
- [x] `hypersolve/src/simplex_projection.rs` — signals: `scalar`; disposition: face feasibility and ordering preserve explicit Unknown; the default refinement floor now comes from the centralized policy constant.
- [x] `hypersolve/src/sketch.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_arc_incidence.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_arc_length.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_arc_tangent.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_builders.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_certificates.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_circle_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_cubic_tangent.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_degeneracy.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_domains.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_entity_domains.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_failed_constraints.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_fixtures.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_oriented_angle.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_projection.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_units.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_workplane_symmetry.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sketch_workplanes.rs` — signals: `policy`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/solver.rs` — signals: `policy,scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/solver_block.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/sparse_pattern.rs` — signals: `scalar`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/src/symbolic.rs` — signals: `scalar`; disposition: simplification no longer hides `0/0`, `0^0`, failed constant powers, or domain-sensitive operands annihilated by zero; totality-aware fast paths remain.
- [x] `hypersolve/tests/readme.rs` — signals: `none`; disposition: reviewed for exact-policy ownership, conservative Unknown/domain handling, and separation of proposal-only lossy arithmetic from terminal certification; no uncentralized approximate terminal decision remains
- [x] `hypersolve/tests/residual_props.rs` — signals: `policy,scalar`; disposition: generated exact/integer oracles and report-shape assertions reviewed; updated ball-certificate matching preserves the expanded certainty payload.
- [x] `hypersolve/tests/smoke.rs` — signals: `policy,scalar`; disposition: adversarial regressions cover nonstructural satisfied inequalities, approximate ball certainty, negative/unknown radii, and `0/0`/`0^0` simplification/collector leaks; all pass.

Hypersolve audit result: every scoped manifest, benchmark, example, fuzz target,
source module, and test listed above was reviewed. The file list exactly matches
the current scoped tree. Verification passed: 503 all-feature tests, all-target
clippy with warnings denied, formatting, diff hygiene, and compilation of every
fuzz target. Paired serial CPU-pinned benchmarks found no material regressions:
affine candidate certification was 2.6050 µs versus 2.5786 µs clean baseline
(+1.0%, within noise), while quadratic interval certification was 9.5619 µs
versus 9.7407 µs clean baseline (-1.8%). Other affected exact routes were also
flat or faster in their paired gates (curve resultant, curve substitution,
root isolation, alpha bounds, quadratic row extraction, LM solve, domain
certification, and symbolic simplification).

## `hypercurve`

- [x] `hypercurve/Cargo.toml` — signals: `policy`; disposition: optional predicate feature boundary reviewed; exact integration remains feature-gated with a no-feature fallback.
- [x] `hypercurve/benches/api_surface.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/arc.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bezier_algebraic_parameter.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bezier_arrangement.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bezier_evaluation.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bezier_rational_overlap.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bezier_region.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bezier_split_materialization.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bezier_tangent_order.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/bspline.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/common/pathological.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/comparative.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/containment.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/curve_path.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/dispatch_trace.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/editing.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/intersection.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/intersection_sweep.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/offset.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/pathological_regions.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/rational_bezier.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/reconstruction.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/benches/straight_skeleton.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/arrangement.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/basic.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/Cargo.toml` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/app.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/corner_scenes.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/editor.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/geometry.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/main.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/plotting.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/scenes.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/share.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/theme.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/examples/hypercurve_ui/src/torture_scene.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/Cargo.toml` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/bezier_algebraic_image.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/bezier_algebraic_parameter.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/bezier_arrangement.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/bezier_region.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/bezier_split_materialization.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/bezier_tangent_order.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/bspline.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/curve_string_editing.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/region_boolean.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/retained_import.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/straight_skeleton.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/fuzz/fuzz_targets/svg.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/arc_bezier.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bbox.rs` — signals: `policy,scalar`; disposition: preview tolerances remain mode-local; certified overlap/containment now use the borrowed Hyperlimit AABB2 cascade.
- [x] `hypercurve/src/bezier.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_algebraic_image.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_arrangement.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_fit.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_flatten.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_metric.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_moment.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_offset.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_parameter.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_region.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_retained_measure.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_retained_overlap.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_split.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_split_endpoint.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_tangent_order.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bezier_topology.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/boolean.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/boolean_boundary.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bspline.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/bulge.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/classify.rs` — signals: `policy,scalar`; disposition: certified mode delegates sign/order to Hyperlimit; preview-only lossy decisions are explicit; no-predicate refinement remains certified and returns Unknown on exhaustion.
- [x] `hypercurve/src/contour.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/contour_regularize.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/curve.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/curve_intersection.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/curve_path_intersection.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/curve_region_boolean.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/curve_region_trim.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/curve_string.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/error.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/events.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/facts.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/finite_projection.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/fragment.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/hershey.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/hershey_data.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/intersect.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/lib.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/nurbs.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/nurbs_interpolation.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/offset.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/point.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/policy.rs` — signals: `policy,scalar`; disposition: fixed constructors that hardcoded APPROXIMATE_512; all curve modes now inherit Hyperlimit's centralized workspace policy, with a regression test.
- [x] `hypercurve/src/polynomial_spline.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/prepared.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/rational_bezier.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/rational_bezier_general.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/reconstruct.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/region.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/region_boolean.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/region_crossing_winding.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/region_events.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/region_fragments.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/region_nesting.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/retained_status.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/segment.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/self_intersect.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/spline_periodic.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/split.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/straight_skeleton.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/svg.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/svg/exact.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/transform.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/translation_obstacle.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/src/triangulation.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/exact_structural_facts.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_adversarial_polygons.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_arc_bezier.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bbox.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_algebraic_image.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_algebraic_parameter.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_arrangement.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_evaluation.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_fit_offset.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_region.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_split_materialization.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bezier_tangent_order.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_boolean.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_bspline.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_circle_predicates.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_contour.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_curve.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_curve_intersection.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_curve_region_boolean.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_curve_region_boolean_fuzz.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_curve_region_promotion.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_curve_string.rs` — signals: `policy`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_dispatch_trace.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_events.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_fragments.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_fuzz_regressions.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_geo_regressions.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_nurbs.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_nurbs_interpolation.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_offset.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_pcb_boolean_regressions.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_polynomial_spline.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_pr59_shape_regressions.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_rational_bezier.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_reconstruct.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_region.rs` — signals: `policy,scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_region_events.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_region_fragments.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_self_contacts.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_split.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_svg.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/hypercurve_triangulation.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/pathological_benchmark_fixture.rs` — signals: `scalar`; disposition: reviewed in Hypercurve audit; see crate summary below.
- [x] `hypercurve/tests/readme.rs` — signals: `none`; disposition: reviewed in Hypercurve audit; see crate summary below.

Hypercurve audit result: all 162 scoped manifest, benchmark, example, fuzz,
source, and test files were reviewed against the centralized Hyperlimit
certainty policy. Certified topology routes sign/order decisions through the
policy and preserves explicit uncertainty; lossy projection, SVG/import
sampling, reconstruction, and edge-preview behavior remain named finite
boundaries. The no-predicate build retains its documented bounded 512-bit
fallback and exposes unresolved transcendental boxes instead of claiming
incomplete envelopes. Exact contact deduplication, connectivity filtering,
retained algebraic Boolean contacts, trim materialization cardinality, and
triangulation index validation were made fail-closed. The newly present exact
contour first-moment path was also reviewed and validated without sampling.

Verification passed on the final source: 224 all-feature library tests, the
complete all-feature integration/doc suite (including seven expensive retained
Boolean fuzz regressions and the pathological corpus), minimal-feature
compilation, all 13 fuzz targets, the UI example, all-target/all-feature Clippy
with warnings denied, formatting, and diff hygiene. Paired serial CPU-pinned
benchmarks found no material regression: retained algebraic region Booleans
improved from 138.0–138.9 µs to 136.7–137.3 µs; self-contact, curve-string
intersection, and region-event rows stayed within roughly 1%; and rational
Bezier replay, conic, immediate-contact, and topology rows were flat or faster.

## `hypertri`

- [x] `hypertri/Cargo.toml` — signals: `policy`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/benches/delaunay.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/benches/earcut.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/benches/exact.rs` — signals: `policy,scalar`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/examples/basic.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/examples/cdt.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/examples/hypertri_ui/Cargo.toml` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/examples/hypertri_ui/src/app.rs` — signals: `scalar`; disposition: reviewed in Hypertri audit; lossy conversion is display-only.
- [x] `hypertri/examples/hypertri_ui/src/main.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/examples/hypertri_ui/src/share.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/examples/nd.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/fuzz/Cargo.toml` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/fuzz/fuzz_targets/topology_invariants.rs` — signals: `none`; disposition: reviewed in Hypertri audit; see crate summary below.
- [x] `hypertri/src/cdt.rs` — signals: `policy,scalar`; disposition: reviewed in Hypertri audit; numeric point identity is certified and malformed triangulation buffers fail closed.
- [x] `hypertri/src/cdt_constraints.rs` — signals: `policy,scalar`; disposition: reviewed in Hypertri audit; all geometric decisions use exact kernel predicates.
- [x] `hypertri/src/cdt_insert.rs` — signals: `policy,scalar`; disposition: reviewed in Hypertri audit; Steiner-point identity is certified and uncertainty propagates.
- [x] `hypertri/src/cdt_validate.rs` — signals: `policy`; disposition: reviewed in Hypertri audit; topology legality delegates to exact kernel predicates.
- [x] `hypertri/src/earcut.rs` — signals: `policy,scalar`; disposition: reviewed in Hypertri audit; point identity, rectangle dispatch, topology, and conformity checks are exact and fail closed.
- [x] `hypertri/src/error.rs` — signals: `none`; disposition: reviewed in Hypertri audit; predicate uncertainty remains a typed public error.
- [x] `hypertri/src/f64.rs` — signals: `none`; disposition: finite IEEE inputs are lifted exactly before triangulation.
- [x] `hypertri/src/kernel.rs` — signals: `policy`; disposition: exact structural sign facts are valid early certificates; remaining sign/orientation/incircle decisions delegate to Hyperlimit and Unknown maps to PredicateUndecided.
- [x] `hypertri/src/lib.rs` — signals: `none`; disposition: reviewed in Hypertri audit; exports expose no alternate scalar decision path.
- [x] `hypertri/src/nd.rs` — signals: `policy,scalar`; disposition: reviewed in Hypertri audit; D-dimensional identity and predicates are certified and uncertainty is preserved in errors and reports.
- [x] `hypertri/src/polygon.rs` — signals: `scalar`; disposition: reviewed in Hypertri audit; duplicate closure detection uses certified numeric equality.
- [x] `hypertri/src/predicates.rs` — signals: `policy`; disposition: exact-rational and structural facts are early certificates; remaining decisions delegate to Hyperlimit.
- [x] `hypertri/src/runtime.rs` — signals: `policy,scalar`; disposition: structural facts select exact algorithms but never replace geometric predicates.
- [x] `hypertri/src/types.rs` — signals: `policy,scalar`; disposition: cached facts are exact structural summaries and are documented not to replace validity proofs.
- [x] `hypertri/tests/adversarial.rs` — signals: `policy`; disposition: reviewed in Hypertri audit; exact boundary, representation, topology, and D-dimensional regressions pass.
- [x] `hypertri/tests/earcutr_differential.rs` — signals: `scalar`; disposition: floating tolerance is confined to a differential-test area oracle.
- [x] `hypertri/tests/fuzz_properties.rs` — signals: `policy,scalar`; disposition: reviewed in Hypertri audit; generated exact topology invariants pass.
- [x] `hypertri/tests/readme.rs` — signals: `none`; disposition: documentation synchronization and release metadata only.

Hypertri audit result: all 32 scoped manifest, benchmark, example, fuzz,
source, and test files were reviewed against the centralized Hyperlimit
certainty policy. Exact-rational and representation-equality facts are used
only as sound early certificates; all remaining equality, order, orientation,
in-circle, and in-sphere decisions delegate to Hyperlimit. Numeric duplicate
points represented by distinct expression trees are now detected throughout
2D, constrained-insertion, ring-closure, and D-dimensional paths. Predicate
exhaustion propagates as `PredicateUndecided`, including geometric TDS and
bistellar-flip reports, rather than being collapsed into invalid input.
Malformed triangle buffers now fail closed before indexing or certification.

Verification passed on the final source: 60 all-feature library tests, 46
integration/property/README tests, four doctests, the default feature suite,
both fuzz targets, the UI example, all-target/all-feature Clippy with warnings
denied, formatting, and diff hygiene. Paired serial CPU-pinned benchmarks found
no material regression: exact earcut, 2D Delaunay, constrained intersection,
4D Delaunay, insertion, flip validation/application, and geometric-report rows
were flat or faster overall. The lifted exact-rational equality fast paths
improved the representative exact earcut row from about 2.79 µs to 2.76 µs and
the 4D Delaunay row from about 21.30 µs to 20.99 µs.

## `hypermesh`

- [x] `hypermesh/Cargo.toml` — signals: `policy,scalar`; disposition: reviewed in Hypermesh audit; dependency features preserve the centralized Hyperlimit policy.
- [x] `hypermesh/benches/common/mod.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; see crate summary below.
- [x] `hypermesh/benches/competitive.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; see crate summary below.
- [x] `hypermesh/benches/dispatch_trace.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; see crate summary below.
- [x] `hypermesh/benches/end_to_end.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; serial baseline/candidate gates cover affected hot paths.
- [x] `hypermesh/competitive/support.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; floating tolerances are confined to competitor metric validation.
- [x] `hypermesh/competitive/yeahright.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; external fixture acquisition is opt-in, hash-checked, and target-local.
- [x] `hypermesh/examples/basic.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; see crate summary below.
- [x] `hypermesh/examples/hypermesh_ui/Cargo.toml` — signals: `policy`; disposition: reviewed in Hypermesh audit; sibling exact-stack patches preserve one policy implementation.
- [x] `hypermesh/examples/hypermesh_ui/src/app.rs` — signals: `policy,scalar`; disposition: lossy conversion is display-only; current Arc-backed triangle iteration is compatible with current dependencies.
- [x] `hypermesh/examples/hypermesh_ui/src/main.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; see crate summary below.
- [x] `hypermesh/fuzz/Cargo.toml` — signals: `none`; disposition: reviewed in Hypermesh audit; all eight fuzz targets compile against the final source.
- [x] `hypermesh/fuzz/fuzz_targets/boolean_box_oracle.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; exact integer-box oracle checks certified output.
- [x] `hypermesh/fuzz/fuzz_targets/boolean_hyperreal_representations.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; representation changes exercise policy exhaustion and topology invariants.
- [x] `hypermesh/fuzz/fuzz_targets/boolean_input_validation.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; malformed and uncertain inputs fail closed.
- [x] `hypermesh/fuzz/fuzz_targets/boolean_pipeline.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; all Boolean APIs and certification paths are cross-checked.
- [x] `hypermesh/fuzz/fuzz_targets/boolean_transformations.rs` — signals: `policy,scalar`; disposition: reviewed in Hypermesh audit; exact transformed representations preserve certified outcomes.
- [x] `hypermesh/fuzz/fuzz_targets/bvh_queries.rs` — signals: `policy,scalar`; disposition: reviewed in Hypermesh audit; accelerated candidates are checked against exact brute force.
- [x] `hypermesh/fuzz/fuzz_targets/mesh_and_hull.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; mesh, hull, and GPU adapter invariants are covered.
- [x] `hypermesh/fuzz/fuzz_targets/polygon_predicates.rs` — signals: `scalar`; disposition: reviewed in Hypermesh audit; clipping and containment predicates remain exact.
- [x] `hypermesh/fuzz/fuzz_targets/support/mod.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; shared fuzz assertions require certified closed output.
- [x] `hypermesh/src/bvh.rs` — signals: `policy,scalar`; disposition: exact classifications propagate uncertainty; lossy coordinates only order traversal and failed bound certificates broaden candidate search.
- [x] `hypermesh/src/clip.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; clipping decisions delegate to fallible exact classifications.
- [x] `hypermesh/src/convex_hull.rs` — signals: `policy,scalar`; disposition: numeric duplicate points and seed degeneracy use centralized exact predicates; lossy distance only orders work.
- [x] `hypermesh/src/error.rs` — signals: `policy,scalar`; disposition: explicit errors preserve predicate exhaustion and malformed topology.
- [x] `hypermesh/src/geometry.rs` — signals: `policy,scalar`; disposition: fallible plane/triangle validity and collinearity decisions preserve Unknown in geometry-producing paths.
- [x] `hypermesh/src/gpu.rs` — signals: `scalar`; disposition: approximation is an explicitly named rendering/export boundary with strict and zero-fallback policies.
- [x] `hypermesh/src/halfspace.rs` — signals: `policy`; disposition: reviewed in Hypermesh audit; exact halfspace feasibility and containment errors propagate.
- [x] `hypermesh/src/intersection.rs` — signals: `scalar`; disposition: alternative exact certificates backtrack while unresolved final relations propagate Unknown.
- [x] `hypermesh/src/lib.rs` — signals: `policy,scalar`; disposition: public documentation and exports expose the exact carrier and explicit uncertainty contract.
- [x] `hypermesh/src/local_bsp.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; BSP topology changes require certified classifications.
- [x] `hypermesh/src/mesh.rs` — signals: `policy,scalar`; disposition: numeric vertex/edge identity is certified; dyadic interval/grid candidate filters reject only provably distinct points; malformed indices fail closed; exact quality facts are cached.
- [x] `hypermesh/src/operations.rs` — signals: `policy,scalar`; disposition: fixed tolerance gates were removed from topology; bounded finite hints remain proposal-only; compact plane-triple classification and one-shot policy reciprocals are replayed exactly.
- [x] `hypermesh/src/output.rs` — signals: `policy,scalar`; disposition: vertex identity, degeneracy, T-junction, crossing, sorting, and cleanup paths are fallible exact decisions; uncertain geometry is never silently deleted.
- [x] `hypermesh/src/polygon.rs` — signals: `policy,scalar`; disposition: exact planes and retained construction identities are used without lossy topology decisions.
- [x] `hypermesh/src/predicate.rs` — signals: `policy,scalar`; disposition: exact-rational and structural certificates precede the centralized Hyperlimit cascade; Unknown maps to `UnknownClassification`.
- [x] `hypermesh/src/segment_trace.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; see crate summary below.
- [x] `hypermesh/src/segment_trace/halfspace_witness.rs` — signals: `policy`; disposition: candidate backtracking tracks uncertainty and only certified witnesses are accepted.
- [x] `hypermesh/src/segment_trace/leaf_probe.rs` — signals: `policy,scalar`; disposition: candidate-family uncertainty is retained through fallback searches.
- [x] `hypermesh/src/segment_trace/path.rs` — signals: `policy,scalar`; disposition: exact event ordering and classifications propagate unresolved final paths.
- [x] `hypermesh/src/segment_trace/probe_cache.rs` — signals: `policy`; disposition: structural equality only keys conservative cache reuse and does not certify geometry.
- [x] `hypermesh/src/segment_trace/probe_geometry.rs` — signals: `policy,scalar`; disposition: nearest-stop ordering now propagates uncertainty; proposal families retain hard-unknown state.
- [x] `hypermesh/src/segment_trace/probe_reachability.rs` — signals: `scalar`; disposition: exact reachability certificates and cycle guards fail closed.
- [x] `hypermesh/src/segment_trace/tests.rs` — signals: `policy,scalar`; disposition: exhaustive uncertainty, boundary, replay, cache, and fallback regressions pass.
- [x] `hypermesh/src/segment_trace/witness.rs` — signals: `policy,scalar`; disposition: exact witness construction preserves partial-family uncertainty.
- [x] `hypermesh/src/storage_hash.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; storage hashing is never a geometric equality certificate.
- [x] `hypermesh/src/subdivision.rs` — signals: `policy,scalar`; disposition: reference and leaf fallback cascades track Unknown; closed-family edge pairing now uses certified numeric equality.
- [x] `hypermesh/src/subdivision/split.rs` — signals: `scalar`; disposition: lossy-free split ranking uses exact values; conservative unknown ranking cannot certify a split.
- [x] `hypermesh/src/subdivision/tests.rs` — signals: `policy,scalar`; disposition: exhaustive subdivision uncertainty, cache, closure, and reference-search regressions pass.
- [x] `hypermesh/src/trace.rs` — signals: `none`; disposition: reviewed in Hypermesh audit; tracing is observational only.
- [x] `hypermesh/src/winding.rs` — signals: `scalar`; disposition: integer reachability is exact and overflow/dimension failures propagate explicitly.
- [x] `hypermesh/tests/competitive.rs` — signals: `scalar`; disposition: shared-corpus topology and metric validation pass; external YeahRight cases remain explicitly opt-in.
- [x] `hypermesh/tests/core.rs` — signals: `policy,scalar`; disposition: exact predicates, Boolean semantics, input validation, and uncertainty regressions pass.
- [x] `hypermesh/tests/readme.rs` — signals: `none`; disposition: README examples, metadata, and brief optional-fixture documentation remain synchronized.
- [x] `hypermesh/tests/regression.rs` — signals: `scalar`; disposition: exact Boolean, boundary-contact, containment, generated-sphere, and roundtrip regressions pass.

Hypermesh audit result: all 55 scoped manifest, benchmark, example, fuzz,
source, and test files were reviewed against the centralized Hyperlimit
certainty policy. Numeric equality now covers convex-hull deduplication,
independently indexed mesh closure, Boolean output vertices, and closed-family
edge pairing across distinct exact expression representations. Plane,
triangle, collinearity, segment-stop, and output-cleanup decisions preserve
predicate exhaustion instead of treating Unknown as false, degenerate, or
disposable geometry. Fixed floating tolerances were removed from topology and
recovery; remaining lossy values only order or propose candidates that are
replayed exactly. Malformed output indices now return typed errors.

Verification passed on the final source: 1,022 all-feature library tests, the
complete all-feature integration/README/doc suite (60 core, 48 regression, and
four enabled competitive tests), the default suite, all eight fuzz targets,
the standalone UI crate, all-target/all-feature Clippy plus UI Clippy with
warnings denied, formatting, and diff hygiene. The optional YeahRight fixture
is downloaded only when requested, hash-checked, and stored under `target`.
Paired serial CPU-pinned benchmarks found no material regression: the 192-face
subdivided Boolean union held at about 84.4 ms and remained faster than the
pristine baseline; the 3,072-face-per-input soup build held near 13.74 ms; cube
output certification improved to about 184.6 µs from about 186.0 µs; and hull,
cube-soup, and representative Boolean rows were flat or faster.

## `hyperphysics`

- [x] `hyperphysics/Cargo.toml` — signals: `policy`; disposition: dependency and dispatch-trace feature wiring preserves one Hyperlimit policy instance.
- [x] `hyperphysics/benches/mass_properties.rs` — signals: `scalar`; disposition: serial gate covers exact reports and the lifted contact/shape comparisons with checksums.
- [x] `hyperphysics/examples/basic.rs` — signals: `none`; disposition: construction-only example exercises certified material and ordered-box validation.
- [x] `hyperphysics/fuzz/Cargo.toml` — signals: `policy`; disposition: fuzz dependencies patch every exact-stack crate to the local unified implementation.
- [x] `hyperphysics/fuzz/fuzz_targets/physics_invariants.rs` — signals: `none`; disposition: representation-crossing invariants require certified results and retain bounded GJK uncertainty.
- [x] `hyperphysics/src/body.rs` — signals: `scalar`; disposition: identifier and exact-carrier code makes no scalar or topology decisions.
- [x] `hyperphysics/src/contact.rs` — signals: `policy,scalar`; disposition: comparisons use Hyperlimit's centralized exact-rational-first cascade and propagate Unknown as a physics query error.
- [x] `hyperphysics/src/em.rs` — signals: `scalar`; disposition: exact report construction uses STRICT domain checks and never promotes a lossy proposal.
- [x] `hyperphysics/src/error.rs` — signals: `policy`; disposition: explicit error variants preserve undecidable signs, diagnostics, and shape queries.
- [x] `hyperphysics/src/fluid.rs` — signals: `scalar`; disposition: physical domains use STRICT signs; conservation reports are exact algebra without predicate decisions.
- [x] `hyperphysics/src/gjk.rs` — signals: `scalar`; disposition: caller-owned bounded refinement returns explicit Unknown, while its default ceiling follows Hyperlimit's centralized maximum.
- [x] `hyperphysics/src/integration.rs` — signals: `scalar`; disposition: positive-domain decisions use STRICT and replay diagnostics retain explicit provenance.
- [x] `hyperphysics/src/lib.rs` — signals: `policy,scalar`; disposition: report paths that promise certified provenance explicitly select STRICT and preserve Unknown.
- [x] `hyperphysics/src/mass.rs` — signals: `policy`; disposition: signed-volume certificates refine to the centralized maximum and unresolved orientation remains an error.
- [x] `hyperphysics/src/material.rs` — signals: `none`; disposition: exact material construction requires a STRICT positive-density decision.
- [x] `hyperphysics/src/optics.rs` — signals: `scalar`; disposition: interface topology and physical domains use STRICT signs and propagate uncertainty.
- [x] `hyperphysics/src/photochemistry.rs` — signals: `scalar`; disposition: cure and fraction decisions use STRICT comparisons; unknown cure margins remain absent.
- [x] `hyperphysics/src/property.rs` — signals: `scalar`; disposition: intervals and domains use STRICT; structural Real inequality no longer masquerades as a certified source conflict.
- [x] `hyperphysics/src/residual.rs` — signals: `scalar`; disposition: primitive estimates are diagnostic-only and exact zero replay depends solely on certified residual signs.
- [x] `hyperphysics/src/shape.rs` — signals: `policy`; disposition: support/shape decisions use centralized predicates, lifted direct comparisons, and explicit Unknown errors.
- [x] `hyperphysics/src/thermal.rs` — signals: `scalar`; disposition: every physical-domain decision uses STRICT; exact report algebra contains no tolerance decisions.
- [x] `hyperphysics/tests/contact.rs` — signals: `policy`; disposition: regression and generated cases cross-check contact classifications against Hyperlimit.
- [x] `hyperphysics/tests/dispatch_trace.rs` — signals: `scalar`; disposition: asserts certified query/replay paths emit no approximation or unknown-fact events.
- [x] `hyperphysics/tests/em.rs` — signals: `scalar`; disposition: exact constitutive algebra and invalid-domain cases are covered.
- [x] `hyperphysics/tests/fluid.rs` — signals: `none`; disposition: generated exact conservation and physical-domain invariants are covered.
- [x] `hyperphysics/tests/integration.rs` — signals: `none`; disposition: generated exact step and diagnostic identities are covered.
- [x] `hyperphysics/tests/mass_properties.rs` — signals: `policy`; disposition: orientation certificates, inward winding, invalid domains, and generated scaling are covered.
- [x] `hyperphysics/tests/optics.rs` — signals: `none`; disposition: exact interface, attenuation, and generated domain invariants are covered.
- [x] `hyperphysics/tests/photochemistry.rs` — signals: `none`; disposition: threshold, invalid-domain, and generated diffusion invariants are covered.
- [x] `hyperphysics/tests/property.rs` — signals: `none`; disposition: certified conflicts, unresolved structural differences, intervals, and generated agreement are covered.
- [x] `hyperphysics/tests/readme.rs` — signals: `none`; disposition: documentation example compilation has no independent predicate decisions.
- [x] `hyperphysics/tests/residual.rs` — signals: `none`; disposition: exact zero/nonzero and generated residual replay status are covered.
- [x] `hyperphysics/tests/shape_queries.rs` — signals: `policy`; disposition: generated shape reports cross-check Hyperlimit and preserve exact boundaries.
- [x] `hyperphysics/tests/thermal.rs` — signals: `none`; disposition: exact thermal identities and generated positive-domain cases are covered.

## `hypersdf`

- [x] `hypersdf/Cargo.toml` — signals: `policy`; disposition: exact-stack dependencies and optional adapter boundary reviewed; no independent decisions.
- [x] `hypersdf/benches/classification.rs` — signals: `policy`; disposition: every classification, interval, sampling, contouring, transform, and voxel hot path is covered by serial Criterion gates.
- [x] `hypersdf/examples/basic.rs` — signals: `policy`; disposition: public report-bearing classification path only; no local scalar decisions.
- [x] `hypersdf/fuzz/Cargo.toml` — signals: `policy`; disposition: all three fuzz targets build against the audited default feature set.
- [x] `hypersdf/fuzz/fuzz_targets/dual_contouring.rs` — signals: `policy`; disposition: exact replay readiness, unknown signs, degenerate roots, and lossy signed-grid blockers are asserted.
- [x] `hypersdf/fuzz/fuzz_targets/gradient_contouring.rs` — signals: `policy`; disposition: primitive gradients remain proposal-only and filter/report invariants are asserted.
- [x] `hypersdf/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `policy`; disposition: retained expression dispatch covers all representative Hyperreal structural families.
- [x] `hypersdf/src/dual_contour.rs` — signals: `policy,scalar`; disposition: retained-scalar signs use the strict construction policy; primitive signs remain explicitly lossy and blocked from validation.
- [x] `hypersdf/src/expr.rs` — signals: `policy,scalar`; disposition: retained carrier and metric-status propagation only; scalar decisions are delegated.
- [x] `hypersdf/src/facts.rs` — signals: `policy`; disposition: domain facts consume strict predicate outcomes; unbounded polynomial primitives are correctly LocalOnly for Lipschitz evidence.
- [x] `hypersdf/src/gradient.rs` — signals: `policy`; disposition: piecewise branch selection is strict; approximate terminal norm decisions never expose a certified normal.
- [x] `hypersdf/src/gradient_contour.rs` — signals: `policy,scalar`; disposition: all primitive-float comparisons are confined to a permanently blocked proposal-only report.
- [ ] `hypersdf/src/hypervoxel_adapter.rs` — signals: `none`; disposition: deferred with Hypervoxel at user request.
- [x] `hypersdf/src/interval.rs` — signals: `policy`; disposition: endpoint ordering, min/max, clamp, absolute-value, and domain construction all use the strict policy and propagate provenance.
- [x] `hypersdf/src/lib.rs` — signals: `policy,scalar`; disposition: exports and 57 unit regressions reviewed; no alternate unreported decision path.
- [x] `hypersdf/src/lipschitz.rs` — signals: `policy`; disposition: conservative-bound construction is strict and composed prerequisite provenance is retained.
- [x] `hypersdf/src/mesh.rs` — signals: `scalar`; disposition: float lowering, zero crossings, Surface Nets topology, and normals are explicitly PreviewOnly.
- [x] `hypersdf/src/policy.rs` — signals: `policy`; disposition: centralized strict-construction versus report-bearing terminal-decision boundary.
- [x] `hypersdf/src/primitive.rs` — signals: `policy`; disposition: domains and scalar construction use strict or paired Hyperlimit cascades; only the final reported sign may use terminal approximation.
- [x] `hypersdf/src/sampling.rs` — signals: `policy,scalar`; disposition: retained min/max/abs branches are strict; approximate sign buckets are limited to PreviewOnly reports.
- [x] `hypersdf/src/sdf.rs` — signals: `policy,scalar`; disposition: terminal decisions retain Approximate separately, composed certainty is never upgraded, decisive CSG children dominate Unknown only when logically sound.
- [x] `hypersdf/src/shader.rs` — signals: `policy,scalar`; disposition: all float lowering and generated comparisons are explicitly PreviewOnly adapter output.
- [x] `hypersdf/src/solver.rs` — signals: `policy`; disposition: boundary replay is accepted only when evidence is certified; approximate boundary decisions remain Unknown.
- [x] `hypersdf/src/status.rs` — signals: `policy,scalar`; disposition: Approximate is distinct from Certified; public malformed Certified/Approximate combinations fail self-consistency.
- [x] `hypersdf/src/transform.rs` — signals: `policy`; disposition: affine interval branch selection is strict and unsupported inverses remain explicit.
- [x] `hypersdf/src/voxel.rs` — signals: `policy`; disposition: grid-step validation is strict and uncertified cell evidence maps to Unknown occupancy.
- [x] `hypersdf/tests/adversarial.rs` — signals: `policy,scalar`; disposition: 45 generated/adversarial tests cover invariance, exact boundaries, invalid domains, previews, contours, and voxel reports.
- [x] `hypersdf/tests/dispatch_trace.rs` — signals: `policy,scalar`; disposition: dispatch trace asserts predicate activity without defining alternate semantics.
- [x] `hypersdf/tests/readme.rs` — signals: `none`; disposition: documentation synchronization and release metadata only.

## `hyperbrep`

- [ ] `hyperbrep/Cargo.toml` — signals: `policy`; disposition: deferred at user request
- [ ] `hyperbrep/benches/kernel.rs` — signals: `policy,scalar`; disposition: deferred at user request
- [ ] `hyperbrep/examples/basic.rs` — signals: `none`; disposition: deferred at user request
- [ ] `hyperbrep/fuzz/Cargo.toml` — signals: `policy`; disposition: deferred at user request
- [ ] `hyperbrep/fuzz/fuzz_targets/analytic_solid_roundtrip.rs` — signals: `policy`; disposition: deferred at user request
- [ ] `hyperbrep/fuzz/fuzz_targets/model_builder.rs` — signals: `none`; disposition: deferred at user request
- [ ] `hyperbrep/fuzz/fuzz_targets/topology_edit_roundtrip.rs` — signals: `scalar`; disposition: deferred at user request
- [ ] `hyperbrep/src/boolean.rs` — signals: `policy`; disposition: deferred at user request
- [ ] `hyperbrep/src/builder.rs` — signals: `policy,scalar`; disposition: deferred at user request
- [ ] `hyperbrep/src/error.rs` — signals: `policy`; disposition: deferred at user request
- [ ] `hyperbrep/src/geometry.rs` — signals: `policy,scalar`; disposition: deferred at user request
- [ ] `hyperbrep/src/lib.rs` — signals: `none`; disposition: deferred at user request
- [ ] `hyperbrep/src/model.rs` — signals: `policy,scalar`; disposition: deferred at user request
- [ ] `hyperbrep/src/persistence.rs` — signals: `policy`; disposition: deferred at user request
- [ ] `hyperbrep/src/tessellation.rs` — signals: `policy,scalar`; disposition: deferred at user request

## `hyperpath`

- [x] `hyperpath/Cargo.toml` — signals: `policy`; disposition: dependency paths/features and full-root crate links checked; no alternate predicate implementation
- [x] `hyperpath/benches/path_predicates.rs` — signals: `policy`; disposition: affected cascades covered and serial CPU-pinned gate retained no regressions
- [x] `hyperpath/examples/basic.rs` — signals: `policy`; disposition: example uses the centralized workspace policy; no hidden scalar decisions
- [x] `hyperpath/fuzz/Cargo.toml` — signals: `policy`; disposition: all declared fuzz binaries compile against current exact-policy APIs
- [x] `hyperpath/fuzz/fuzz_targets/bezier_arrangement.rs` — signals: `policy`; disposition: centralized policy and exact arrangement invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/cam_pocket_link.rs` — signals: `policy`; disposition: centralized policy and exact link continuity invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/cam_rest_material.rs` — signals: `policy`; disposition: centralized policy and exact area/set invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/explicit_arc_arrangement.rs` — signals: `policy`; disposition: centralized policy and exact arc-event invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `policy,scalar`; disposition: representation-equivalence oracle intentionally exercises policy escalation
- [x] `hyperpath/fuzz/fuzz_targets/line_arc_arrangement.rs` — signals: `policy`; disposition: centralized policy and exact mixed-event invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/line_arrangement.rs` — signals: `policy`; disposition: centralized policy and exact split/cell invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/pcb_circular_board.rs` — signals: `policy`; disposition: centralized policy and exact clearance invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/pcb_convex_pad.rs` — signals: `policy,scalar`; disposition: centralized policy; integer/rational oracle comparisons are non-lossy
- [x] `hyperpath/fuzz/fuzz_targets/pcb_obround_board.rs` — signals: `policy`; disposition: centralized policy and exact capsule-clearance invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/pcb_obround_pad.rs` — signals: `policy,scalar`; disposition: centralized policy; scalar oracle remains exact integer/rational arithmetic
- [x] `hyperpath/fuzz/fuzz_targets/pcb_oriented_rect_pad.rs` — signals: `policy,scalar`; disposition: centralized policy; Pythagorean/integer oracle is exact
- [x] `hyperpath/fuzz/fuzz_targets/pcb_orthogonal_pad.rs` — signals: `policy,scalar`; disposition: centralized policy and exact polygon/clearance invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/pcb_rounded_rect_pad.rs` — signals: `policy,scalar`; disposition: centralized policy and exact rounded-pad equivalence invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/pcb_via_fabrication.rs` — signals: `policy`; disposition: centralized policy and exact fabrication-rule invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/routing_feed.rs` — signals: `policy`; disposition: centralized policy and exact feed residual invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/routing_keepout.rs` — signals: `policy,scalar`; disposition: centralized policy and exact obstacle/length invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/specctra_route_rule_audit.rs` — signals: `policy`; disposition: centralized policy and exact rule-selection invariants checked
- [x] `hyperpath/fuzz/fuzz_targets/specctra_route_rules.rs` — signals: `scalar`; disposition: fixed-grid parser/round-trip fuzz uses exact integers; no Real predicate
- [x] `hyperpath/fuzz/fuzz_targets/specctra_route_text.rs` — signals: `scalar`; disposition: syntax/round-trip fuzz uses exact fixed-grid tokens; no lossy decision
- [x] `hyperpath/fuzz/fuzz_targets/specctra_trace_rule_clearance.rs` — signals: `policy,scalar`; disposition: centralized policy and exact clearance/rule invariants checked
- [x] `hyperpath/src/arc.rs` — signals: `policy,scalar`; disposition: constructors/incidence/equality now policy-aware; sole structural sign use is a non-authoritative cached Unknown-capable sweep hint
- [x] `hyperpath/src/arrangement.rs` — signals: `policy,scalar`; disposition: point/segment/intersection/construction cascades now preserve the caller policy end-to-end
- [x] `hyperpath/src/bezier.rs` — signals: `policy,scalar`; disposition: rational-weight construction guard uses centralized sign cascade; polynomial evaluation is exact
- [x] `hyperpath/src/bezier_arrangement.rs` — signals: `policy,scalar`; disposition: all ordering/equality/domain decisions use explicit policy and exact represented-root evidence
- [x] `hyperpath/src/cam.rs` — signals: `policy,scalar`; disposition: rectangular constructors and generated bead/link segments propagate explicit policy; unknowns remain errors/statuses
- [x] `hyperpath/src/cam/pocket_link.rs` — signals: `policy`; disposition: boundary/link construction and point equality now preserve explicit policy
- [x] `hyperpath/src/cam/rest.rs` — signals: `policy`; disposition: rectangular set/area decisions propagate caller policy and unknown errors
- [x] `hyperpath/src/curve_cell.rs` — signals: `policy,scalar`; disposition: curve-cell ordering, equality, root-domain, ray, and area decisions all use explicit policy
- [x] `hyperpath/src/lib.rs` — signals: `policy`; disposition: public explicit-policy constructors/certifiers and typed unresolved errors exported consistently
- [x] `hyperpath/src/mixed_bezier_arrangement.rs` — signals: `policy`; disposition: line-fragment reconstruction now preserves caller policy; unknown ordering/equality remains typed
- [x] `hyperpath/src/mixed_conic_arrangement.rs` — signals: `policy,scalar`; disposition: represented conic roots and generated line fragments use explicit policy throughout
- [x] `hyperpath/src/mixed_cubic_arrangement.rs` — signals: `policy,scalar`; disposition: represented cubic roots and generated line fragments use explicit policy throughout
- [x] `hyperpath/src/mixed_curve_arrangement.rs` — signals: `policy,scalar`; disposition: merged-family scheduling and fragment construction preserve caller policy
- [x] `hyperpath/src/offset.rs` — signals: `policy`; disposition: distance/radius guards and generated segment/arc construction preserve caller policy; unresolved is typed
- [x] `hyperpath/src/pcb.rs` — signals: `policy,scalar`; disposition: board/pad/via constructors, polygon validation, compound intersections, and generated edges now preserve explicit policy
- [x] `hyperpath/src/pcb/drill_policy.rs` — signals: `policy,scalar`; disposition: sign/order rule decisions use explicit cascades and retain Unknown acceptance state
- [x] `hyperpath/src/pcb_circular_board.rs` — signals: `policy,scalar`; disposition: radius guard and squared-distance containment use explicit policy with Unknown status
- [x] `hyperpath/src/pcb_convex_pad.rs` — signals: `policy`; disposition: policy-aware constructor/convexity validation and compound edge predicates added
- [x] `hyperpath/src/pcb_obround_board.rs` — signals: `policy,scalar`; disposition: diameter guard and capsule predicates use explicit policy with Unknown status
- [x] `hyperpath/src/pcb_obround_pad.rs` — signals: `policy,scalar`; disposition: constructor, compound segment predicates, and distance ordering preserve explicit policy
- [x] `hyperpath/src/pcb_oriented.rs` — signals: `policy`; disposition: extent/unit-axis guards and generated edge/intersection predicates preserve explicit policy
- [x] `hyperpath/src/pcb_orthogonal_pad.rs` — signals: `policy,scalar`; disposition: policy-aware constructor, edge construction, simplicity, and clearance predicates added
- [x] `hyperpath/src/ph.rs` — signals: `policy,scalar`; disposition: PH construction and inverse-length guards expose explicit-policy variants; residual replay remains exact
- [x] `hyperpath/src/ph_smoothing.rs` — signals: `policy,scalar`; disposition: tangent nondegeneracy guards expose and preserve explicit-policy variants
- [x] `hyperpath/src/routing.rs` — signals: `policy,scalar`; disposition: all meander guards, generated segments, obstacle predicates, and length decisions preserve caller policy
- [x] `hyperpath/src/routing/feed.rs` — signals: `policy,scalar`; disposition: process guards and exact residual certification use explicit policy; unresolved is typed
- [x] `hyperpath/src/routing/jerk_schedule.rs` — signals: `policy,scalar`; disposition: jerk/feed/time guards use explicit policy and preserve certification failures
- [x] `hyperpath/src/routing/lookahead.rs` — signals: `policy,scalar`; disposition: feed/radius guards and schedule decisions use explicit policy with typed unresolved results
- [x] `hyperpath/src/routing/orthogonal_keepout.rs` — signals: `policy,scalar`; disposition: generated edges and compound intersection predicates now preserve caller policy
- [x] `hyperpath/src/segment.rs` — signals: `policy,scalar`; disposition: fallible policy-aware construction prevents invalid cached bounds; endpoint equality preserves certainty/provenance
- [x] `hyperpath/src/solve.rs` — signals: `policy,scalar`; disposition: exact Hypersolve residual construction only; no local inequality or tolerance decision
- [x] `hyperpath/src/specctra.rs` — signals: `policy,scalar`; disposition: exact-record imports expose explicit-policy variants; fixed-grid lowering remains exact rational
- [x] `hyperpath/src/specctra/rule_audit.rs` — signals: `policy,scalar`; disposition: rule sign/order/clearance decisions use explicit policy and retain Unknown statuses
- [x] `hyperpath/src/specctra_syntax.rs` — signals: `none`; disposition: tokenizer/parser control logic only; no Real predicate
- [x] `hyperpath/src/swept.rs` — signals: `policy`; disposition: width construction guard uses explicit sign cascade and fails on unresolved input
- [x] `hyperpath/src/tangent.rs` — signals: `policy,scalar`; disposition: endpoint equality, zero-vector, cross/dot, and join decisions use explicit policy
- [x] `hyperpath/tests/dispatch_trace.rs` — signals: `policy,scalar`; disposition: verifies exact fast path does not request terminal approximation
- [x] `hyperpath/tests/path_primitives.rs` — signals: `policy,scalar`; disposition: 421 exact/property tests pass, including strict propagation and approximate-provenance regressions
- [x] `hyperpath/tests/readme.rs` — signals: `none`; disposition: release/example metadata checks pass; no scalar decisions

## `hyperdrc`

- [x] `hyperdrc/Cargo.toml` — signals: `policy`; disposition: exact-stack dependencies are current workspace crates; Geo is absent and repository documentation links use crate roots.
- [x] `hyperdrc/benches/fixture_smoke.rs` — signals: `scalar`; disposition: fixture counts and elapsed-time smoke gate only; no Real predicate.
- [x] `hyperdrc/benches/parser_geometry_smoke.rs` — signals: `scalar`; disposition: audited workload generator; exact inputs feed public checks and the full serial gate covers affected paths.
- [x] `hyperdrc/benches/spatial_index_audit.rs` — signals: `scalar`; disposition: finite conservative index workload with exact narrow phase; serial gate retained.
- [x] `hyperdrc/examples/basic.rs` — signals: `none`; disposition: public API example only; no independent numerical decision.
- [x] `hyperdrc/src/app.rs` — signals: `policy,scalar`; disposition: CLI orchestration delegates numerical checks; scalar configuration reaches centralized helpers.
- [x] `hyperdrc/src/arrow_report.rs` — signals: `none`; disposition: report serialization only; no predicate.
- [x] `hyperdrc/src/assembly_policy.rs` — signals: `none`; disposition: exact scalar configuration composition only; no ordering decision.
- [x] `hyperdrc/src/authoring_intent.rs` — signals: `scalar`; disposition: exact spacing/keepout decisions use centralized scalar helpers and typed Boolean gateways.
- [x] `hyperdrc/src/baseline.rs` — signals: `scalar`; disposition: finding identity and report-state comparison only; no Real predicate.
- [x] `hyperdrc/src/capability.rs` — signals: `none`; disposition: profile validation routes scalar bounds through centralized decisions; remaining equality is schema/data identity.
- [x] `hyperdrc/src/checks/artifact_handoff.rs` — signals: `none`; disposition: textual artifact evidence only.
- [x] `hyperdrc/src/checks/artifact_table.rs` — signals: `none`; disposition: table parsing only.
- [x] `hyperdrc/src/checks/artifacts.rs` — signals: `policy,scalar`; disposition: exact numeric artifact rules use centralized scalar decisions; other comparisons are textual/count checks.
- [x] `hyperdrc/src/checks/assembly.rs` — signals: `policy,scalar`; disposition: exact narrow phases and thresholds use Hyperlimit policy; finite AABBs remain conservative broad phases and unresolved sorts are explicit invariants.
- [x] `hyperdrc/src/checks/board.rs` — signals: `policy,scalar`; disposition: exact distances, areas, angles, equality, and extrema use centralized policy; finite indices remain conservative.
- [x] `hyperdrc/src/checks/constraints.rs` — signals: `scalar`; disposition: every rule threshold, extrema, bounds order, and route metric uses centralized policy; uncertain contour length falls back to certified exact bounds.
- [x] `hyperdrc/src/checks/continuity.rs` — signals: `scalar`; disposition: finite rectangles only cull candidates; exact Boolean gateway decides overlap.
- [x] `hyperdrc/src/checks/dense_pad.rs` — signals: `scalar`; disposition: exact distances and clearances use centralized policy; finite grid comparisons are conservative.
- [x] `hyperdrc/src/checks/differential.rs` — signals: `scalar`; disposition: exact width/length/spacing/skew decisions are centralized; uncertain contour extraction uses certified exact bounding dimensions.
- [x] `hyperdrc/src/checks/distance.rs` — signals: `policy,scalar`; disposition: circle/segment/equality/intersection and lifted interval ordering use explicit workspace policy; finite envelopes only reject certified-disjoint candidates.
- [x] `hyperdrc/src/checks/drill.rs` — signals: `policy,scalar`; disposition: exact drill/copper comparisons and grid lifts use explicit workspace policy; candidate indexing is conservative.
- [x] `hyperdrc/src/checks/excellon.rs` — signals: `none`; disposition: exact diameter ordering uses centralized policy with explicit comparability invariant.
- [x] `hyperdrc/src/checks/impedance.rs` — signals: `scalar`; disposition: transcendental impedance/domain comparisons route through centralized policy; stack-layer indexes are primitive integers.
- [x] `hyperdrc/src/checks/layer.rs` — signals: `policy,scalar`; disposition: topology, orientation, areas, density, bounds, angles, intersections, and equality use explicit policy; interval/finite paths are conservative or report-only.
- [x] `hyperdrc/src/checks/manifest.rs` — signals: `scalar`; disposition: manifest counts and textual metadata only; no Real decision.
- [x] `hyperdrc/src/checks/mechanical.rs` — signals: `scalar`; disposition: exact distance/hull/keepout decisions use centralized policy; finite rectangular fast paths are conservative and exact division cannot silently become zero.
- [x] `hyperdrc/src/checks/mod.rs` — signals: `policy`; disposition: all exact offset/Boolean operations use typed uncertainty gateways; construction failures have a regression test.
- [x] `hyperdrc/src/checks/net_class.rs` — signals: `scalar`; disposition: class selection is textual; scalar values are carried without local predicates.
- [x] `hyperdrc/src/checks/net_scope.rs` — signals: `scalar`; disposition: exact region-bound validation and membership now use centralized scalar decisions.
- [x] `hyperdrc/src/checks/outline.rs` — signals: `policy,scalar`; disposition: lifted exact comparisons use workspace policy; finite geometry is compatibility input only.
- [x] `hyperdrc/src/checks/power.rs` — signals: `scalar`; disposition: exact clearance predicates use centralized helpers after conservative indexing.
- [x] `hyperdrc/src/checks/power_integrity.rs` — signals: `scalar`; disposition: exact pad/via dimensions and distances use centralized helpers; unavailable finite projections fall back to exact region bounds.
- [x] `hyperdrc/src/checks/return_path.rs` — signals: `scalar`; disposition: exact reference-distance extrema and thresholds use centralized policy; finite AABBs only cull.
- [x] `hyperdrc/src/checks/rf.rs` — signals: `scalar`; disposition: exact RF clearances use centralized policy; finite index comparisons remain conservative.
- [x] `hyperdrc/src/checks/safety.rs` — signals: `scalar`; disposition: exact isolation/return distances and thresholds use centralized policy; finite broad phase cannot accept a violation.
- [x] `hyperdrc/src/checks/signal.rs` — signals: `scalar`; disposition: exact mixed-signal distances use centralized policy; finite grid is candidate-only.
- [x] `hyperdrc/src/checks/spatial.rs` — signals: `scalar`; disposition: explicitly finite conservative broad-phase grid; no authoritative scalar decision.
- [x] `hyperdrc/src/checks/spread.rs` — signals: `policy,scalar`; disposition: exact hull ordering, orientation, equality, and diameter use centralized policy with explicit comparability invariants.
- [x] `hyperdrc/src/checks/stencil.rs` — signals: `scalar`; disposition: exact aperture areas, ratios, and distances use centralized helpers; Boolean uncertainty propagates.
- [x] `hyperdrc/src/checks/surface_finish.rs` — signals: `none`; disposition: textual finish compatibility only.
- [x] `hyperdrc/src/checks/thermal.rs` — signals: `scalar`; disposition: exact via spread, touch, distance, and dimensions use centralized policy; finite spatial tests are conservative.
- [x] `hyperdrc/src/cli.rs` — signals: `none`; disposition: exact decimal parsing adapter only; no scalar predicate.
- [x] `hyperdrc/src/config.rs` — signals: `none`; disposition: configuration transport only.
- [x] `hyperdrc/src/constraint_policy.rs` — signals: `none`; disposition: exact scalar policy data model only; checks own decisions.
- [x] `hyperdrc/src/conversion.rs` — signals: `scalar`; disposition: conversion process/status and file metadata only; no Real predicate.
- [x] `hyperdrc/src/date.rs` — signals: `scalar`; disposition: calendar integer validation only.
- [x] `hyperdrc/src/dxf_overlay.rs` — signals: `none`; disposition: finite report rendering bounds only.
- [x] `hyperdrc/src/exact_path_rules.rs` — signals: `policy`; disposition: caller policy now propagates through sign and comparison decisions; strict versus terminal-512 regression included.
- [x] `hyperdrc/src/excellon.rs` — signals: `scalar`; disposition: retained decimal coordinates and exact diameter extrema use workspace-policy Hyperlimit calls; syntax decisions are primitive.
- [x] `hyperdrc/src/excellon_overlay.rs` — signals: `scalar`; disposition: finite report overlay bounds only.
- [x] `hyperdrc/src/gencad_review.rs` — signals: `scalar`; disposition: finite parser/report normalization only; tiny-value cleanup is not topology.
- [x] `hyperdrc/src/geometry.rs` — signals: `scalar`; disposition: exact-aware public geometry exports and regressions reviewed; invalid transforms retain explicit construction failure.
- [x] `hyperdrc/src/geometry/primitives.rs` — signals: `scalar`; disposition: finite parser adapters construct exact regions; transform failure is explicit and never silently unchanged.
- [x] `hyperdrc/src/geometry/region.rs` — signals: `none`; disposition: exact polygon composition propagates contour/kernel uncertainty instead of substituting empty geometry.
- [x] `hyperdrc/src/geometry/source_units.rs` — signals: `policy,scalar`; disposition: source decimal/grid provenance is exact integer/rational arithmetic; compatibility merging is non-lossy.
- [x] `hyperdrc/src/geometry/types.rs` — signals: `policy,scalar`; disposition: polygon contour orientation uses explicit workspace policy and exact construction failure is retained.
- [x] `hyperdrc/src/geometry/violations.rs` — signals: `policy`; disposition: exact area gates use explicit workspace policy; finite shapes are report output.
- [x] `hyperdrc/src/gerber_metadata.rs` — signals: `scalar`; disposition: Gerber syntax/metadata plus centralized exact macro-parameter comparisons; no float topology predicate.
- [x] `hyperdrc/src/gerber_overlay.rs` — signals: `none`; disposition: finite report overlay bounds only.
- [x] `hyperdrc/src/github_annotations.rs` — signals: `none`; disposition: report formatting only.
- [x] `hyperdrc/src/html_report.rs` — signals: `scalar`; disposition: report serialization and browser-side display filtering only.
- [x] `hyperdrc/src/io.rs` — signals: `scalar`; disposition: file discovery and exact scalar serialization transport only.
- [x] `hyperdrc/src/ipc2581_review.rs` — signals: `scalar`; disposition: finite review-summary bounds only; no exact model decision.
- [x] `hyperdrc/src/ipc356.rs` — signals: `scalar`; disposition: retained exact coordinates/diameters and extrema use centralized policy; grammar control is primitive.
- [x] `hyperdrc/src/ipc356_review.rs` — signals: `policy,scalar`; disposition: exact IPC evidence comparisons use workspace policy; report-only finite locations remain adapters.
- [x] `hyperdrc/src/jsonl.rs` — signals: `none`; disposition: report serialization only.
- [x] `hyperdrc/src/junit.rs` — signals: `none`; disposition: report serialization only.
- [x] `hyperdrc/src/kicad.rs` — signals: `scalar`; disposition: retained decimal coordinates use exact arithmetic; exact offset and area decisions route through centralized helpers.
- [x] `hyperdrc/src/kicad/arcs.rs` — signals: `policy,scalar`; disposition: exact arc orientation uses explicit workspace policy; sampling remains a named finite construction adapter.
- [x] `hyperdrc/src/kicad/custom_pad.rs` — signals: `scalar`; disposition: exact source points feed policy-audited primitive construction; sampling is conservative parser adaptation.
- [x] `hyperdrc/src/kicad/footprint_graphics.rs` — signals: `scalar`; disposition: retained exact source coordinates feed exact-backed geometry; finite sampling is compatibility construction.
- [x] `hyperdrc/src/kicad/graphic_primitives.rs` — signals: `scalar`; disposition: finite KiCad drawing adapter only; results are promoted to exact-backed polygons before checks.
- [x] `hyperdrc/src/kicad/graphics.rs` — signals: `policy,scalar`; disposition: exact endpoint closure uses explicit workspace point equality; stroke sampling is parser adaptation.
- [x] `hyperdrc/src/kicad/model.rs` — signals: `scalar`; disposition: exact-aware board data model only; no numerical predicate.
- [x] `hyperdrc/src/kicad/text.rs` — signals: `scalar`; disposition: finite text-envelope adapter only; no authoritative exact decision.
- [x] `hyperdrc/src/kicad_dru.rs` — signals: `scalar`; disposition: exact configured threshold validity uses centralized policy; emitted text is not reinterpreted locally.
- [x] `hyperdrc/src/kicad_markers.rs` — signals: `scalar`; disposition: marker report parsing/render bounds only; no exact model predicate.
- [x] `hyperdrc/src/lib.rs` — signals: `scalar`; disposition: `PcbRegion` owns exact curve topology; construction/Boolean/offset uncertainty is typed and finite projections are report-only.
- [x] `hyperdrc/src/main.rs` — signals: `none`; disposition: CLI entry point only.
- [x] `hyperdrc/src/package_archive.rs` — signals: `scalar`; disposition: archive metadata/limits use primitive counts and sizes only.
- [x] `hyperdrc/src/package_policy.rs` — signals: `none`; disposition: package policy enum/data only.
- [x] `hyperdrc/src/parquet_report.rs` — signals: `none`; disposition: report serialization only.
- [x] `hyperdrc/src/pdf_overlay.rs` — signals: `scalar`; disposition: finite report rendering and page bounds only.
- [x] `hyperdrc/src/process_lifecycle.rs` — signals: `scalar`; disposition: OS process/status integer logic only.
- [x] `hyperdrc/src/readiness.rs` — signals: `policy,scalar`; disposition: execution certainty/status transport preserves uncertain versus failed outcomes; no independent Real predicate.
- [x] `hyperdrc/src/report.rs` — signals: `scalar`; disposition: exact scalar values are serialized/displayed; identity hashing uses report projections intentionally.
- [x] `hyperdrc/src/sarif.rs` — signals: `none`; disposition: report serialization only; crate link is the repository root.
- [x] `hyperdrc/src/scalar.rs` — signals: `policy,scalar`; disposition: sole default comparison/sign boundary delegates to Hyperlimit's centralized Approximate-512 workspace policy; explicit-policy variants are available internally.
- [x] `hyperdrc/src/sexp.rs` — signals: `scalar`; disposition: token parser only; exact decimal interpretation occurs in source-unit adapters.
- [x] `hyperdrc/src/sqlite_report.rs` — signals: `scalar`; disposition: report persistence only.
- [x] `hyperdrc/src/svg_overlay.rs` — signals: `none`; disposition: finite report rendering only.
- [x] `hyperdrc/src/test_intent.rs` — signals: `scalar`; disposition: exact locations are carried as evidence; decisions are enum/text/count based.
- [x] `hyperdrc/src/waiver.rs` — signals: `none`; disposition: waiver identity/governance text only.
- [x] `hyperdrc/tests/readme.rs` — signals: `none`; disposition: documentation and release metadata checks only.

## `hypervoxel`

- [ ] `hypervoxel/Cargo.toml` — signals: `policy`; disposition: pending
- [ ] `hypervoxel/benches/grid_frame.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/examples/exact_box.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/examples/sparse_grid.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/fuzz/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/fuzz/fuzz_targets/continuous_field_materialization.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/fuzz/fuzz_targets/grid_address.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/fuzz/fuzz_targets/hypermesh_adapter.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/fuzz/fuzz_targets/triangle_solid_voxelization.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/fuzz/fuzz_targets/triangle_surface_voxelization.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/aabb.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/address.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/affine.rs` — signals: `policy`; disposition: pending
- [ ] `hypervoxel/src/aggregate.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/src/batch.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/cell.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/chunk.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/chunk_diff.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/chunk_faces.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/chunk_storage.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/src/chunk_support.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/chunk_surface_mesh.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/component_row_plan.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/continuous.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/differential.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/distance.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/error.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/field.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/frame.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/halfspace.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/src/hypermesh_adapter.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/legacy.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/lib.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/lod.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/material.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/mesh.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/path.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/query.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/src/ray_schedule.rs` — signals: `policy`; disposition: pending
- [ ] `hypervoxel/src/report.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/serialize.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/side_table.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/solid.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/sparse_surface_mesh.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/spatial.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/storage.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/support.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/surface_mesh.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/surface_topology.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/svo.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/svo_surface.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/src/transform.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/triangle_mesh.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/src/triangle_row_cache.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/triangle_solid.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/src/voxelis_adapter.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/src/voxelize.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/tests/antagonistic_inputs.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/tests/box_voxelization.rs` — signals: `policy,scalar`; disposition: pending
- [ ] `hypervoxel/tests/continuous_field_materialization.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/tests/dispatch_trace.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/tests/field_path_batch.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/tests/grid_semantics.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/tests/hypermesh_adapter.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/tests/legacy_voxelis_adapter.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/tests/readme.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/tests/triangle_mesh_voxelization.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis-bevy/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-bevy/examples/greedy_meshing.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-bevy/examples/lod.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-bevy/src/lib.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-bevy/src/mesh.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-math/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-math/benches/overlap_audit.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis-math/src/lib.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis-memory/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-memory/src/allocator_stats.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-memory/src/lib.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-memory/src/pool_allocator.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis-memory/src/pool_allocator_lite.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis-voxelize/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis-voxelize/benches/voxelize.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis-voxelize/src/lib.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/benches/voxtree_bench.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/core/batch.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/core/block_id.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/core/lod.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/core/max_depth.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/core/mod.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/core/traversal_depth.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/core/voxel.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/interner/consts.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/interner/hash.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/interner/macros.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/interner/mod.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/interner/stats.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/io/consts.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/io/export.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/io/flags.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/io/import.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/io/mod.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/io/obj_reader.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/io/varint.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/lib.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/spatial/aabb2d.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/spatial/mod.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/spatial/voxops.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/spatial/voxtree.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/utils/common.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/utils/mesh.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/utils/mod.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/utils/shapes.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/world/mod.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/voxelis/src/world/voxchunk.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/world/voxmodel.rs` — signals: `scalar`; disposition: pending
- [ ] `hypervoxel/voxelis/src/world/voxworld.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/vtm-export/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/vtm-export/src/main.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/vtm-viewer/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/vtm-viewer/src/main.rs` — signals: `none`; disposition: pending
- [ ] `hypervoxel/vtm-voxelize/Cargo.toml` — signals: `none`; disposition: pending
- [ ] `hypervoxel/vtm-voxelize/src/main.rs` — signals: `none`; disposition: pending

## `hyperpack`

- [x] `hyperpack/Cargo.toml` — signals: `policy`; disposition: Hyperlimit is a direct unified dependency and dispatch tracing follows its centralized certainty policy.
- [x] `hyperpack/benches/feasibility.rs` — signals: `policy,scalar`; disposition: broad serial gate covers replay, support/load, model export, objectives, search, and adapters.
- [x] `hyperpack/benches/replay_micro.rs` — signals: `scalar`; disposition: CPU-pinned micro gate covers 1D/2D/3D replay, clearance, irregular packing, analysis, and bounded search.
- [x] `hyperpack/examples/basic.rs` — signals: `none`; disposition: reviewed public API example; all acceptance delegates to policy-aware replay.
- [x] `hyperpack/fuzz/Cargo.toml` — signals: `policy`; disposition: local exact-stack patches resolve the same Hyperlimit implementation as the library.
- [x] `hyperpack/fuzz/fuzz_targets/packing_invariants.rs` — signals: `none`; disposition: generated containment, overlap, accounting, and exact-objective invariants compile against the final policy-aware library.
- [x] `hyperpack/src/analysis.rs` — signals: `scalar`; disposition: ordering and extrema use centralized comparison; uncertainty in a maximum is sticky and cannot be overwritten by a later item.
- [x] `hyperpack/src/bounds.rs` — signals: `policy,scalar`; disposition: capacity and pair-incompatibility certificates use centralized decisions and retain Unknown when no decisive bound exists.
- [x] `hyperpack/src/clearance.rs` — signals: `scalar`; disposition: every separating gap is checked independently; any certified sufficient gap wins, all certified failures violate, and mixed exhaustion is Unknown.
- [x] `hyperpack/src/domain.rs` — signals: `scalar`; disposition: exact imports fail closed on uncertified dimensions while conservative/lossy/unknown domain facts remain explicit evidence.
- [x] `hyperpack/src/error.rs` — signals: `none`; disposition: validation errors document fail-closed nonnegative certification rather than claiming an uncertified value was negative.
- [x] `hyperpack/src/heuristic2d.rs` — signals: `policy,scalar`; disposition: proposal comparisons use the central cascade; tri-state separation preserves decisive axes and final layouts are replay-gated.
- [x] `hyperpack/src/heuristic3d.rs` — signals: `policy,scalar`; disposition: proposal fits, ordering, and residual choices use central decisions; emitted layouts remain exact-replay gated.
- [x] `hyperpack/src/irregular2d.rs` — signals: `policy,scalar`; disposition: Hypercurve certified policy shares the Hyperlimit workspace policy; unavailable no-fit, containment, area, and ordering evidence stays Unknown.
- [x] `hyperpack/src/lib.rs` — signals: `policy`; disposition: module wiring centralizes crate-internal scalar decisions and public exports preserve report-bearing APIs.
- [x] `hyperpack/src/model.rs` — signals: `scalar`; disposition: exact carrier dimensions require a centralized certified positive sign and fail closed otherwise.
- [x] `hyperpack/src/model_export.rs` — signals: `scalar`; disposition: unknown lower bounds and axis extents can no longer become Ready or false Infeasible model reports.
- [x] `hyperpack/src/multibin.rs` — signals: `scalar`; disposition: costs fail closed through central signs and per-bin Unknown replay status propagates to the aggregate.
- [x] `hyperpack/src/objective.rs` — signals: `scalar`; disposition: maximum uncertainty is sticky and an unknown higher-priority lexicographic term can no longer be overruled by a lower-priority term.
- [x] `hyperpack/src/orientation.rs` — signals: `policy,scalar`; disposition: rotations are exact permutations and every legal lowering is checked by the centralized replay layer.
- [x] `hyperpack/src/portfolio.rs` — signals: `policy,scalar`; disposition: feasibility rank is authoritative; unresolved scalar tie-breaks conservatively retain deterministic first-seen reports.
- [x] `hyperpack/src/predicate.rs` — signals: `policy,scalar`; disposition: sole scalar adapter uses Hyperlimit sign/compare cascades and truth-dominant three-valued conjunction/disjunction; regression exceeds the removed 64-bit cutoff.
- [x] `hyperpack/src/replay.rs` — signals: `scalar`; disposition: containment and separation use lazy truth-dominant tri-state cascades, so decisive evidence overrides earlier Unknown without losing fast paths.
- [x] `hyperpack/src/search.rs` — signals: `policy,scalar`; disposition: local-search geometry proposals and objective ranking use centralized predicates and all retained candidates are replayed.
- [x] `hyperpack/src/sheet.rs` — signals: `policy,scalar`; disposition: authoritative 2D containment/no-overlap uses centralized truth-dominant tri-state decisions.
- [x] `hyperpack/src/snapshot.rs` — signals: `scalar`; disposition: exact rationals or full structural JSON are serialized without primitive-float lowering.
- [x] `hyperpack/src/solver.rs` — signals: `policy,scalar`; disposition: bounded exhaustive search records predicate uncertainty, so skipped undecidable branches can never yield a false Infeasible proof.
- [x] `hyperpack/src/stock.rs` — signals: `scalar`; disposition: authoritative 1D containment/no-overlap uses centralized truth-dominant tri-state decisions.
- [x] `hyperpack/src/support.rs` — signals: `scalar`; disposition: contact uncertainty propagates, supported area is an exact rectangle union, and uncertain carried contact cannot undercount load into a false pass.
- [x] `hyperpack/tests/dispatch_trace.rs` — signals: `scalar`; disposition: all-feature trace regression confirms exact benchmark-shaped inputs avoid terminal approximation.
- [x] `hyperpack/tests/feasibility.rs` — signals: `policy,scalar`; disposition: 146 integration/property regressions pass, including exact support-union, clearance, replay, bounds, export, objective, and search cases.
- [x] `hyperpack/tests/readme.rs` — signals: `none`; disposition: quickstart and release metadata remain synchronized with the final manifest and example.

Hyperpack audit result: all 32 scoped manifest, benchmark, example, fuzz,
source, and test files were reviewed against the centralized Hyperlimit
certainty policy. Every production `Real` sign and ordering decision now routes
through one crate-internal Hyperlimit adapter; the former hard-coded 64-bit
refinement ceiling is absent. Truth-dominant tri-state combinators retain
decisive containment/separation evidence, while clearance, support/load, model
export, extrema, lexicographic objectives, and exhaustive search propagate
uncertainty instead of silently selecting false, omitting load, or proving
infeasibility. Support contact area is now an exact rectangle union.

Verification passed on the final source: six library tests, 146
integration/property tests, dispatch-trace and README tests, every all-feature
bench/example target, the fuzz target, all-target/all-feature Clippy with
warnings denied, formatting, and diff hygiene. Paired serial CPU-pinned release
benchmarks found no regression: broad feasibility improved from about 298.87 ms
to 148.11 ms per iteration; 3D/2D/1D replay improved from roughly
633.52/621.98/612.17 µs to 243.58/214.83/227.80 µs; clearance improved from
3.37 ms to 2.34 ms; and all irregular, analysis, and bounded-search rows were
flat or faster.

## `hyperparts`

- [x] `hyperparts/Cargo.toml` — signals: `policy`; disposition: Hyperlimit is a direct unified dependency and dispatch tracing follows the centralized policy.
- [x] `hyperparts/benches/queries.rs` — signals: `scalar`; disposition: serial gate covers interval validation, safe voltage-envelope queries, EDA intake, and nonnumeric catalog baselines.
- [x] `hyperparts/examples/basic.rs` — signals: `none`; disposition: reviewed public graph/query example; no independent scalar predicate.
- [x] `hyperparts/fuzz/Cargo.toml` — signals: `policy`; disposition: fuzz build resolves the same local Hyperlimit and Hyperreal implementations.
- [x] `hyperparts/fuzz/fuzz_targets/eda_authoring_intake.rs` — signals: `scalar`; disposition: generated exact-text intake and explicit issue/status invariants compile against the final policy-aware crate.
- [x] `hyperparts/fuzz/fuzz_targets/hyperreal_representations.rs` — signals: `none`; disposition: representation-preservation fuzz target performs no local inequality decision.
- [x] `hyperparts/src/assertion.rs` — signals: `scalar`; disposition: exact interval ordering now uses the central Hyperlimit comparison and fails closed on Unknown.
- [x] `hyperparts/src/compatibility.rs` — signals: `none`; disposition: relationship and decision carriers only; no scalar predicate.
- [x] `hyperparts/src/eda_intake.rs` — signals: `scalar`; disposition: numeric source text parses directly to exact Real values; parse failures remain structured issues without numeric fallback.
- [x] `hyperparts/src/electronics.rs` — signals: `scalar`; disposition: voltage range construction and overlap use centralized ordering with truth-dominant disjointness.
- [x] `hyperparts/src/error.rs` — signals: `none`; disposition: range validation errors explicitly describe failure to certify ordered bounds.
- [x] `hyperparts/src/geometry.rs` — signals: `scalar`; disposition: geometry handles/status/provenance only; no Real decision.
- [x] `hyperparts/src/graph.rs` — signals: `scalar`; disposition: safe power-terminal compatibility consumes the policy-aware tri-state voltage overlap; other ordering is primitive rank/text.
- [x] `hyperparts/src/identity.rs` — signals: `none`; disposition: stable textual identity validation only.
- [x] `hyperparts/src/interface.rs` — signals: `scalar`; disposition: terminal voltage envelope construction and overlap use centralized ordering with explicit Unknown.
- [x] `hyperparts/src/lib.rs` — signals: `policy`; disposition: crate-internal scalar adapter is wired without expanding the public API; exact carriers remain re-exported.
- [x] `hyperparts/src/physics.rs` — signals: `scalar`; disposition: physical values are provenance/status handoffs; readiness is categorical and makes no scalar decision.
- [x] `hyperparts/src/predicate.rs` — signals: `policy,scalar`; disposition: sole scalar adapter uses Hyperlimit comparison and truth-dominant closed-interval overlap; regression exceeds the removed local 64-bit horizon.
- [x] `hyperparts/src/process.rs` — signals: `none`; disposition: process/tolerance assertions are carried without locally interpreting numerical limits.
- [x] `hyperparts/src/query.rs` — signals: `none`; disposition: query constraints and evidence carriers use textual/categorical matching only.
- [x] `hyperparts/tests/dispatch_trace.rs` — signals: `scalar`; disposition: exact EDA numeric intake trace confirms no terminal approximation or unknown-fact event.
- [x] `hyperparts/tests/part_graph.rs` — signals: `policy,scalar`; disposition: all 19 graph/intake/handoff tests pass, including exact voltage compatibility and unknown evidence.
- [x] `hyperparts/tests/readme.rs` — signals: `none`; disposition: quickstart and release metadata remain synchronized.

Hyperparts audit result: all 23 scoped manifest, benchmark, example, fuzz,
source, and test files were reviewed. The only authoritative Real decisions
are assertion/range ordering and voltage-envelope overlap; both now route
through a single Hyperlimit adapter. A certified disjoint bound dominates an
unknown opposite comparison, while construction still fails closed when bound
ordering cannot be certified. All other numeric fields are exact/provenance
carriers or explicit lossy/unknown handoffs.

Verification passed on the final source: two library policy regressions, 19
graph/intake tests, dispatch-trace and README tests, the benchmark/example
targets, both fuzz targets, all-target/all-feature Clippy with warnings denied,
formatting, and diff hygiene. Serial CPU-pinned performance passed after making
Hyperlimit's existing exact-rational compare entry and outcome extraction
cross-crate inlineable: safe connection improved from about 105 ns to 97 ns,
assertion interval validation held at about 666 ns versus 669 ns, EDA intake
improved from about 10.25 µs to 9.78 µs, and remaining rows were flat or faster.

## `hyperevolution`

- [x] `hyperevolution/Cargo.toml` — signals: `policy`; disposition: Hyperlimit is a direct unified dependency and dispatch tracing follows its central policy.
- [x] `hyperevolution/benches/fitness.rs` — signals: `scalar`; disposition: serial gate covers scalar/lexicographic/interval fitness, hill climbing, selection, GP, oracle, and wide-neighborhood paths.
- [x] `hyperevolution/examples/basic.rs` — signals: `none`; disposition: reviewed public proposal/replay example; no independent numeric decision.
- [x] `hyperevolution/fuzz/Cargo.toml` — signals: `policy`; disposition: fuzz build resolves the same local Hyperlimit/Hyperreal stack.
- [x] `hyperevolution/fuzz/fuzz_targets/evolution_invariants.rs` — signals: `scalar`; disposition: representation-spanning ordering now exercises the policy-aware fitness API rather than Real partial ordering.
- [x] `hyperevolution/src/domain.rs` — signals: `scalar`; disposition: replay manifests carry categorical owner/status/evidence only; no local scalar decision.
- [x] `hyperevolution/src/fitness.rs` — signals: `scalar`; disposition: scalar, lexicographic, Pareto, interval validity, separation, and numeric endpoint equality all use centralized comparison.
- [x] `hyperevolution/src/gp.rs` — signals: `none`; disposition: structural-zero validation is explicitly conservative and exact evaluation still returns checked division failure.
- [x] `hyperevolution/src/identity.rs` — signals: `none`; disposition: stable textual candidate identity validation only.
- [x] `hyperevolution/src/lib.rs` — signals: `policy`; disposition: central predicate adapter is crate-internal and report-bearing public APIs remain intact.
- [x] `hyperevolution/src/oracle.rs` — signals: `scalar`; disposition: opaque fitness and surrogate stages carry evidence/status without interpreting scalar values.
- [x] `hyperevolution/src/predicate.rs` — signals: `policy,scalar`; disposition: sole Real sign/order/equality adapter uses Hyperlimit; regression exceeds the removed local 64-bit horizon.
- [x] `hyperevolution/src/search.rs` — signals: `scalar`; disposition: selection/hill-climb ranking uses policy-aware fitness; annealing schedule guards now distinguish InvalidSchedule from UnknownSchedule.
- [x] `hyperevolution/tests/dispatch_trace.rs` — signals: `scalar`; disposition: exact GP and interval workloads confirm no terminal approximation.
- [x] `hyperevolution/tests/fitness.rs` — signals: `scalar`; disposition: all 22 exact/property search, interval, GP, oracle, archive, and replay tests pass.
- [x] `hyperevolution/tests/readme.rs` — signals: `none`; disposition: quickstart and release metadata remain synchronized.

Hyperevolution audit result: all 16 scoped manifest, benchmark, example, fuzz,
source, and test files were reviewed. Fitness comparison, interval validity,
interval separation, and endpoint equality now route through a single
Hyperlimit adapter. Simulated-annealing schedule inequalities no longer use
overloaded partial-order booleans that can turn Unknown into false; uncertified
schedules return `UnknownSchedule`, while certified domain violations remain
`InvalidSchedule`.

Verification passed on the final source: one library policy regression, 22
integration/property tests, dispatch-trace and README tests, benchmark/example
targets, the fuzz target, all-target/all-feature Clippy with warnings denied,
formatting, link checks, and diff hygiene. Serial CPU-pinned performance was
flat or faster: the broad workload improved from about 4.17 µs to 3.74 µs per
iteration, interval comparison improved from about 61 ns to 34 ns, and GP plus
wide-neighborhood rows held near 13 ns and 2.59 µs.

## `hypergraphics`

- [x] `hypergraphics/Cargo.toml` — signals: `policy`; disposition: the exact feature owns the direct Hyperlimit dependency and dispatch tracing; crate metadata links to the crate-level GitHub URL.
- [x] `hypergraphics/benches/graphics.rs` — signals: `policy`; disposition: serial CPU-pinned export, grid, default orientation, retained-evidence, and unprojection gates cover every affected hot path.
- [x] `hypergraphics/examples/readme_quickstart.rs` — signals: `none`; disposition: reviewed; it crosses only the explicit finite render-export boundary.
- [x] `hypergraphics/fuzz/Cargo.toml` — signals: `policy`; disposition: reviewed; the fuzz package resolves the exact Hypergraphics feature and unified sibling dependencies.
- [x] `hypergraphics/fuzz/fuzz_targets/scene_invariants.rs` — signals: `policy`; disposition: now exercises both the centralized default and strict explicit orientation cascades across representative Real structures.
- [x] `hypergraphics/src/backend.rs` — signals: `scalar`; disposition: all primitive-float tests and narrowing are confined to the documented GPU presentation boundary and reject non-finite/overflowing data.
- [x] `hypergraphics/src/camera.rs` — signals: `policy,scalar`; disposition: exact camera domains and unprojection-ray degeneracy now use policy-aware Hyperlimit ordering; Unknown is an error, the fixed 1e-12 test is gone, and every primitive projection boundary rejects invalid/non-finite data.
- [x] `hypergraphics/src/error.rs` — signals: `none`; disposition: added an explicit indeterminate-predicate error retaining Hyperlimit refinement need and escalation stage.
- [x] `hypergraphics/src/geometry.rs` — signals: `policy,scalar`; disposition: triangle orientation now has an explicit-policy API while the default retains its benchmarked direct centralized cascade and evidence cache; Unknown remains first-class.
- [x] `hypergraphics/src/lib.rs` — signals: `policy,scalar`; disposition: reviewed and re-exports the policy/certainty vocabulary required by the public exact APIs.
- [x] `hypergraphics/src/render.rs` — signals: `scalar`; disposition: reviewed; f32/f64 values are finite checked color and presentation attributes, never topology certificates.
- [x] `hypergraphics/src/scene.rs` — signals: `scalar`; disposition: reviewed; geometry remains Real-owned and polygon topology delegates to Hypertri, while integer loop cardinality does not make scalar predicates.
- [x] `hypergraphics/tests/dispatch_trace.rs` — signals: `policy,scalar`; disposition: verifies near-coplanar rational orientation stays on certified predicate dispatch without approximation or unknown facts.
- [x] `hypergraphics/tests/readme.rs` — signals: `none`; disposition: validates synchronized runnable documentation and release metadata.

Hypergraphics is complete at 14/14 policy-relevant files. The audit exported
Hyperlimit's existing policy-aware 3D orientation cascade, added explicit
policy variants for mesh orientation and exact-camera domain decisions, and
removed a fixed unprojection epsilon. Primitive render boundaries now reject
invalid public-field inputs and non-finite matrices/results instead of
returning misleading projected points. Validation covers all-feature and
renderer-only tests, dispatch tracing, the release fuzz build, all-target
Clippy with warnings denied, documentation/link checks, formatting, and diff
hygiene. Serial CPU-4 measurements were flat: default orientation measured
about 41.2 ns versus 41.4 ns baseline, while unprojection's 1.845–1.860 us
interval remained inside the baseline's 1.806–1.912 us interval.

## `hypercircuit`

Checkpoint: Hypercircuit is complete. Hyperlimit is a non-optional dependency,
and one crate-local extension routes comparison, equality, inequality, and
sign classification through the centralized default decision cascade. Every
listed source, benchmark, example, and test file has been checked; direct
`Real::partial_cmp`, structural-sign, fixed-refinement, and overloaded Real
ordering bypasses are absent.

- [x] `hypercircuit/Cargo.toml` — signals: `policy`; disposition: Hyperlimit is now a direct dependency for base simulation as well as layout, with the crate-level GitHub URL retained.
- [x] `hypercircuit/benches/easyduino_full_pipeline.rs` — signals: `scalar`; disposition: reviewed; benchmark harness only; exact quantities are inputs or reported results and no acceptance predicate is implemented here.
- [x] `hypercircuit/benches/event_agenda.rs` — signals: `none`; disposition: reviewed; benchmark harness only; exact quantities are inputs or reported results and no acceptance predicate is implemented here.
- [x] `hypercircuit/benches/mna.rs` — signals: `scalar`; disposition: reviewed; benchmark harness only; exact quantities are inputs or reported results and no acceptance predicate is implemented here.
- [x] `hypercircuit/benches/routing.rs` — signals: `none`; disposition: reviewed; benchmark harness only; exact quantities are inputs or reported results and no acceptance predicate is implemented here.
- [x] `hypercircuit/examples/ac_sweep.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/advanced_routing.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/analytic_sources.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/auto_schematic.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/basic.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/curved_fabrication.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/declarative_board.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/diode_transient.rs` — signals: `scalar`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_esp32.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_esp32s3.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_fixture_generator.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_nano.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_native/mod.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_rp2040.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_stm32f103.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/easyduino_uno.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/fluent_design.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/hierarchical_schematic.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/kicad_library_import.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/kicad_roundtrip.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/kicad_schematic.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/kicad_stackup_rules.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/layout_module.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/lceda_pro_export.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/mixed_signal_session.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/mosfet_dc.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/multipart_symbol.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/nonlinear_ac.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/parameterized_module.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/part_library.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/phase_tuning.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/reusable_parts.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/review_bundle.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/semantic_edit.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/signal_bundles.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/source_diagnostics.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/transient_run.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/examples/tscircuit_router_handoff.rs` — signals: `none`; disposition: reviewed; authored exact fixture only; no independent acceptance predicate is implemented here.
- [x] `hypercircuit/src/ac.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/adapter.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/assembly.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/authoring.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/autoroute.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/behavior.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/bin/hypercircuit.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/coordinates.rs` — signals: `policy`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/coupling.rs` — signals: `scalar`; disposition: electrothermal sign certification now uses the centralized Hyperlimit cascade and preserves indeterminate replay as an error.
- [x] `hypercircuit/src/drc.rs` — signals: `scalar`; disposition: process-area sign classification now uses the centralized policy; the concurrent region-terminology migration is preserved.
- [x] `hypercircuit/src/edit.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/erc.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/error.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/event_simulation.rs` — signals: `scalar`; disposition: all Real clock, agenda, breakpoint, and stochastic-stream ordering now goes through the centralized cascade with existing explicit agenda errors on Unknown.
- [x] `hypercircuit/src/fabrication.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/hierarchy.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/identity.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/intelligent_exchange.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/intent.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/interchange.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/interface.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/kicad.rs` — signals: `policy,scalar`; disposition: reviewed; primitive conversion is confined to checked KiCad decimal serialization and does not certify retained topology.
- [x] `hypercircuit/src/kicad_import.rs` — signals: `policy,scalar`; disposition: fixed refinements use the centralized cascade; board exterior selection now ranks exact Real shoelace-area magnitudes and returns an explicit import error on Unknown instead of treating failed f64 projection as zero.
- [x] `hypercircuit/src/kicad_library.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/kicad_schematic.rs` — signals: `policy,scalar`; disposition: schematic geometry sign/incidence decisions now use the centralized cascade; finite conversion remains checked serialization only.
- [x] `hypercircuit/src/layout.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/layout_module.rs` — signals: `policy`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/lceda.rs` — signals: `scalar`; disposition: retained range checks now use the centralized policy while checked f64 conversion remains at the LCEDA interchange boundary.
- [x] `hypercircuit/src/legacy_csgrs.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/lib.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/manufacturing_release.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/materialize.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/mna.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/model.rs` — signals: `policy,scalar`; disposition: source-waveform validation and exact phase/time evaluation now share the centralized comparison cascade.
- [x] `hypercircuit/src/mosfet.rs` — signals: `policy,scalar`; disposition: Newton policy validation, interval selection, maxima, and tolerance replay now use centralized comparisons and retain Unknown as the existing indeterminate solve error.
- [x] `hypercircuit/src/nonlinear.rs` — signals: `scalar`; disposition: diode/switch Newton validation, interval selection, maxima, and authored tolerance replay now use centralized comparisons and explicit indeterminate errors.
- [x] `hypercircuit/src/package.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/panel.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/placement.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/predicate.rs` — signals: `policy,scalar`; disposition: new crate-local adapter routes Real comparison and sign classification through Hyperlimit's centralized default policy and preserves Unknown as `None` for call-site-specific errors or conservative rejection.
- [x] `hypercircuit/src/preview.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/project.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/project_manifest.rs` — signals: `scalar`; disposition: material-property sign validation now uses centralized predicate classification.
- [x] `hypercircuit/src/release_artifact.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/route_constraints.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/routing.rs` — signals: `scalar`; disposition: exact route candidate minima and denominator-domain checks now use the centralized comparison cascade; cardinality comparisons remain integer-only.
- [x] `hypercircuit/src/schematic.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/schematic_auto.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/sexp.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/simulation.rs` — signals: `scalar`; disposition: all exact waveform, event, timestep, tolerance, and adaptive-controller decisions now use centralized comparisons and preserve explicit indeterminate run errors.
- [x] `hypercircuit/src/stitching.rs` — signals: `policy,scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/test_intent.rs` — signals: `none`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/tscircuit_routing.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/src/workflow.rs` — signals: `scalar`; disposition: reviewed; no direct exactness-policy bypass remains; numeric decisions use the centralized cascade or reject/report Unknown conservatively.
- [x] `hypercircuit/tests/ac.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/authoring.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/autoroute.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/behavior.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/cli.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/coppertrace_erc.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/coppertrace_examples.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/curved_outline.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/curved_routing.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/declarative.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/dispatch_trace.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/easyduino.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/edit.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/erc.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/event_interchange.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/event_simulation.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/fabrication.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/hierarchy.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/intent.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/interchange.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/interface.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/kicad_library.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/kicad_schematic.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/layout.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/layout_module.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/lceda.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/legacy_csgrs.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/materialize.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/mna.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/mosfet.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/nonlinear.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/package.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/placement.rs` — signals: `policy`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/preview.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/project.rs` — signals: `policy`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/project_manifest.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/readme.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/release_workflow.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/routing_corpus.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/schematic.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/schematic_auto.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/simulation.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/stitching.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/support/routing_corpus.rs` — signals: `scalar`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.
- [x] `hypercircuit/tests/workflow.rs` — signals: `none`; disposition: reviewed; assertions exercise production policy; Real ordering uses the centralized comparator where applicable and no production decision is certified here.

Hypercircuit is complete at 142/142 policy-relevant files. Import, layout,
materialization, placement, autorouting, phase tuning, simulation, schematic,
fabrication, panelization, and edit/replay decisions now either receive a
centralized Hyperlimit result or retain/report Unknown explicitly. In
particular, indeterminate coordinate incidence, sorting, grid membership,
candidate collision, transformed bounds, skew/coupling, KiCad extrema,
outline stitching, and session-time comparisons can no longer fall through as
successful geometry or simulation. Integration-test ordering of transcendental
and other nontrivial Reals also uses the centralized comparator.

Validation passed the complete all-feature library/integration suite, a
focused 77-test rerun after the test-policy cleanup, all-feature library/test
Clippy with warnings denied, formatting, direct-bypass scans, and diff hygiene.
Serial CPU-4 benchmark medians showed no regression. Event agenda ingest was
92.771 ms baseline versus 92.787 ms candidate and delivery improved from
26.622 ms to 25.391 ms. Routing improved from 451.495/94.864/64.259 ms to
446.051/93.090/61.158 ms for parallel-bus, crossing, and any-angle fixtures,
with identical work and result metrics. A nine-pair, single-codegen-unit MNA
cross-check measured 561 ns versus 563 ns for stamp/replay, 7.525 us versus
7.537 us for multi-node assembly, and improvements for carrier,
electrothermal, and nonlinear-report paths. The baseline was committed
Hypercircuit HEAD against the same current sibling crates, with only the
disposable baseline manifest's retired CSGRS feature names updated so Cargo
could resolve it.

## `csgrs`

- [x] `csgrs/Cargo.toml` — signals: `policy,scalar`; disposition: exact-stack dependency surface audited; attributed-only benchmark now declares its required feature.
- [x] `csgrs/benchmarks/rust/adapter_graphics_export.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/rust/competitive.rs` — signals: `none`; disposition: audited
- [x] `csgrs/benchmarks/rust/feature_pipeline.rs` — signals: `policy,scalar`; disposition: audited
- [x] `csgrs/benchmarks/rust/kernel_comparison.rs` — signals: `policy,scalar`; disposition: exact fixture orientation and zero decisions use Hyperlimit
- [x] `csgrs/benchmarks/rust/mesh_pipeline.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/rust/part_blueprint.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/rust/profile_primitives.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/rust/solidean_comparison.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/support/competitive.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/support/generated_corpus.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/support/harness.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/support/solidean.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/benchmarks/support/yeahright.rs` — signals: `none`; disposition: optional verified target-directory download
- [x] `csgrs/examples/adjacency_demo.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/basic.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/basic2d_shapes_and_offsetting.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/basic_shapes.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/boolean_operations.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/convex_hull.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/extrude.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/minkowski_sum.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/multi_format_export.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/examples/readme_renders.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/examples/scalar_adapter.rs` — signals: `none`; disposition: audited
- [x] `csgrs/examples/transformations.rs` — signals: `none`; disposition: audited
- [x] `csgrs/fuzz/Cargo.toml` — signals: `policy`; disposition: all 18 fuzz binaries compile
- [x] `csgrs/fuzz/fuzz_targets/fuzz_curve_boolean_pair.rs` — signals: `scalar`; disposition: exact outcome invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_curve_extrude_revolve_sweep.rs` — signals: `scalar`; disposition: fallible construction and manifold invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_curve_polygon_triangulate.rs` — signals: `none`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_curve_shape_catalog.rs` — signals: `scalar`; disposition: exact catalog invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_dxf_import.rs` — signals: `none`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_export_names.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_gerber_import.rs` — signals: `none`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_mesh_boolean_pair.rs` — signals: `scalar`; disposition: exact Boolean invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_mesh_bytecode.rs` — signals: `scalar`; disposition: exact transform and failure invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_mesh_hypermesh_adapter.rs` — signals: `scalar`; disposition: adapter invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_mesh_polyhedron_constructor.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_mesh_primitive_catalog.rs` — signals: `scalar`; disposition: exact primitive invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_metaballs_boundary.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_obj_import.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_part_blueprint.rs` — signals: `scalar`; disposition: failure and exact-bounds invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_sdf_tpms.rs` — signals: `policy,scalar`; disposition: whole-operation failure invariants added
- [x] `csgrs/fuzz/fuzz_targets/fuzz_svg_import.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/fuzz_transform_matrix.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/fuzz/fuzz_targets/support/mod.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/adapter/curve.rs` — signals: `none`; disposition: fallible exact projection, extrusion, and bounds paths
- [x] `csgrs/src/adapter/mesh.rs` — signals: `none`; disposition: fallible transform and bounds paths
- [x] `csgrs/src/adapter/mod.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/adapter/scalar.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/src/attributed.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/curve/image.rs` — signals: `policy,scalar`; disposition: audited
- [x] `csgrs/src/curve/metaballs.rs` — signals: `policy,scalar`; disposition: failed samples abort the whole extraction
- [x] `csgrs/src/curve/mod.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/curve/native.rs` — signals: `policy,scalar`; disposition: exact comparisons, strict profiles, fallible mesh builders, OpenSCAD-compatible axis collapse, and explicit indexed revolve grids avoid quadratic equality scans.
- [x] `csgrs/src/curve/truetype.rs` — signals: `policy,scalar`; disposition: uncertain curve conversion aborts the whole glyph result
- [x] `csgrs/src/errors.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/hyper_math.rs` — signals: `policy,scalar`; disposition: scalar and triangle decisions route through Hyperlimit
- [x] `csgrs/src/implicit/metaballs.rs` — signals: `policy,scalar`; disposition: failed samples abort the whole extraction
- [x] `csgrs/src/implicit/mod.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/implicit/sdf.rs` — signals: `policy,scalar`; disposition: failed samples abort without fabricated sentinel values
- [x] `csgrs/src/implicit/tpms.rs` — signals: `policy,scalar`; disposition: exact sign and domain validation
- [x] `csgrs/src/io/amf.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/io/dxf.rs` — signals: `scalar`; disposition: checked finite interchange boundary
- [x] `csgrs/src/io/gerber.rs` — signals: `policy,scalar`; disposition: exact topology decisions; documented ULP parser boundary only
- [x] `csgrs/src/io/gltf.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/src/io/mod.rs` — signals: `policy,scalar`; disposition: finite conversions are checked interchange boundaries
- [x] `csgrs/src/io/obj.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/src/io/ply.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/io/stl.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/io/svg.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/io/vrml.rs` — signals: `policy,scalar`; disposition: explicit whole-import validation and exact transforms
- [x] `csgrs/src/lib.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/src/parts/blueprint.rs` — signals: `policy`; disposition: missing metadata or uncertain exact bounds block the whole report
- [x] `csgrs/src/parts/metadata.rs` — signals: `policy,scalar`; disposition: exact bounds and explicit uncertainty
- [x] `csgrs/src/parts/mod.rs` — signals: `none`; disposition: audited
- [x] `csgrs/src/solid.rs` — signals: `policy,scalar`; disposition: fallible transforms/bounds and known-orientation exact fast paths
- [x] `csgrs/src/voxels/mod.rs` — signals: `none`; disposition: audited; no Hypervoxel source was inspected or changed
- [x] `csgrs/src/wasm/curve_js.rs` — signals: `none`; disposition: JS failures are explicit Results
- [x] `csgrs/src/wasm/matrix_js.rs` — signals: `none`; disposition: nonfinite inputs are rejected
- [x] `csgrs/src/wasm/mesh_js.rs` — signals: `scalar`; disposition: JS failures are explicit Results
- [x] `csgrs/src/wasm/mod.rs` — signals: `scalar`; disposition: checked finite boundary helpers
- [x] `csgrs/src/wasm/plane_js.rs` — signals: `none`; disposition: invalid planes are rejected
- [x] `csgrs/src/wasm/point_js.rs` — signals: `none`; disposition: nonfinite inputs are rejected
- [x] `csgrs/src/wasm/vector_js.rs` — signals: `policy,scalar`; disposition: exact vector decisions and explicit errors
- [x] `csgrs/tests/adversarial.rs` — signals: `scalar`; disposition: exact boundary and failure regressions covered
- [x] `csgrs/tests/adversarial_deep.rs` — signals: `none`; disposition: repeated exact operations covered
- [x] `csgrs/tests/adversarial_extrusions.rs` — signals: `none`; disposition: OpenSCAD-compatible extrusion/revolve/sweep behavior covered
- [x] `csgrs/tests/adversarial_fixtures.rs` — signals: `none`; disposition: audited
- [x] `csgrs/tests/adversarial_stress.rs` — signals: `none`; disposition: audited
- [x] `csgrs/tests/competitive.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/tests/generated_benchmark_corpus.rs` — signals: `none`; disposition: audited
- [x] `csgrs/tests/metaballs_diagnostics.rs` — signals: `scalar`; disposition: sample-failure diagnostics covered
- [x] `csgrs/tests/native_queries.rs` — signals: `scalar`; disposition: exact query regressions covered
- [x] `csgrs/tests/readme.rs` — signals: `none`; disposition: audited
- [x] `csgrs/tests/readme_render_regression.rs` — signals: `scalar`; disposition: audited
- [x] `csgrs/tests/sdf_tpms_diagnostics.rs` — signals: `scalar`; disposition: exact sampling-boundary diagnostics covered

CSGRS is complete at 97/97 policy-relevant Rust and Cargo files. Numeric
topology, domain, winding, degeneracy, and bounds decisions now use the
centralized Hyperlimit cascade or preserve explicit uncertainty. Finite
conversion remains confined to checked projection, rendering, WASM, and file
interchange boundaries. Failed implicit samples, missing part metadata,
uncertain curve ownership, invalid transforms, and unrepresentable bounds no
longer become partial geometry, fabricated scalar sentinels, unchanged input,
or successful empty results on fallible APIs. Compatibility wrappers fail
whole-empty where their signatures cannot report errors.

Validation passed formatting, spelling, all three CI check configurations,
default and all-feature unit/integration/doc suites, Clippy over all targets
and features with warnings denied, both adversarial scripts, and compilation
of all 18 fuzz binaries. Serial pinned-CPU benchmark gates covered profile,
mesh, kernel, and broad feature pipelines with identical work/output
checksums. Feature-matched transform paths were flat (rotation +0.05%;
translation, scale, mirror, and generic affine between -1.27% and +0.04%),
the profile suite improved 5.24% overall, and the broad affected path remained
within the 3-7% control-run drift after redundant certification was lifted
into centralized/whole-operation exact cascades.

## `csgrs-ffi`

- [x] `csgrs-ffi/Cargo.toml` — signals: `policy`; disposition: current exact-stack dependencies and crate-level repository URL reviewed; no alternate policy implementation or Geo dependency.
- [x] `csgrs-ffi/src/lib.rs` — signals: `scalar`; disposition: fallible mesh/curve operations propagate exact failures through status values; zero-length null buffers avoid invalid Rust slices; polyhedron offsets cannot discard input indices.

CSGRS FFI is complete at 2/2 policy-relevant Rust and Cargo files. Its C ABI
now compiles against the current fallible CSGRS adapters and preserves
transform, centering, floating, and curve-transform failures instead of
constructing handles from invalid results. The raw-slice boundary accepts
`(NULL, 0)` without invoking `slice::from_raw_parts`, while nonempty null
buffers fail explicitly; polyhedron offsets must cover the complete index
stream so no leading or trailing geometry is silently ignored.

Formatting, locked checks/tests, serial execution of all seven boundary
regressions, Clippy over all targets with warnings denied, C header syntax,
link hygiene, and `git diff --check` passed. A before/after runtime benchmark
is not meaningful for this compatibility repair because the prior source does
not compile against the current adapters; successful paths add only normal
`Result` propagation, while the new slice/offset work is confined to ABI input
validation.

## `openscad-rs`

This parser is below any geometric predicate layer: it preserves numeric
literals as exact `Real` values and does not make Hyperlimit policy decisions.
Every listed manifest, benchmark, example, source, test, and fuzz target was
reviewed. Invalid lexer tokens now fail parsing instead of being deleted;
numeric callbacks propagate failures; malformed include slices cannot invert
a byte range; empty child bodies, grouping, unary plus, and expression
delimiters carry sound spans; and modifiers on `if` statements are retained.

Four sanitizer campaigns cover arbitrary lexing, arbitrary parsing and
diagnostics, generated-valid grammar, and exact decimal/hex/scientific
literals. Their AST oracle checks UTF-8 bounds, containment, sibling ordering,
parameters, arguments, and defaults. Each campaign passed 10,000 serial
executions. The numeric campaign found a shared eager-power resource leak;
Hyperreal now caps eager rational result growth and retains oversized powers
as exact lazy computable expressions. The saved 47.6-second ASan reproducer
now completes in 8 ms.

Formatting, locked all-target checks, 40 unit tests, compatibility and doc
tests, strict Clippy, all fuzz-bin builds, and diff checks pass against the
declared published dependencies. A serial Criterion comparison against an
isolated HEAD archive improved `parse_sample` by 1.9–2.8% and kept the 37 KiB
case within noise (-0.88% to -0.04%).

- [x] `openscad-rs/Cargo.toml` — signals: `none`; disposition: reviewed; exact-number dependency and full crate-root repository metadata are appropriate.
- [x] `openscad-rs/benches/parse_bench.rs` — signals: `scalar`; disposition: reviewed; timing only, with parser results consumed rather than converted into semantic decisions.
- [x] `openscad-rs/examples/bench_file.rs` — signals: `scalar`; disposition: reviewed; lossy text decoding and floating throughput arithmetic are explicitly benchmark-only.
- [x] `openscad-rs/fuzz/Cargo.toml` — signals: `none`; disposition: reviewed; four bounded sanitizer targets are registered.
- [x] `openscad-rs/fuzz/fuzz_targets/fuzz_lexer.rs` — signals: `scalar`; disposition: reviewed; validates token/path spans and error reporting without semantic float decisions.
- [x] `openscad-rs/fuzz/fuzz_targets/fuzz_numeric_literals.rs` — signals: `scalar`; disposition: reviewed; generates bounded exact decimal, hexadecimal, and scientific literals and checks lex/parse agreement.
- [x] `openscad-rs/fuzz/fuzz_targets/fuzz_parser.rs` — signals: `none`; disposition: reviewed; arbitrary input checks deterministic ASTs and bounded diagnostics.
- [x] `openscad-rs/fuzz/fuzz_targets/fuzz_structured_parser.rs` — signals: `none`; disposition: reviewed; generated-valid programs must parse and satisfy strengthened AST invariants.
- [x] `openscad-rs/fuzz/fuzz_targets/support.rs` — signals: `scalar`; disposition: reviewed; input/depth bounds and comprehensive span containment/order checks are non-semantic safety oracles.
- [x] `openscad-rs/src/ast.rs` — signals: `none`; disposition: reviewed; exact `Real` literals and `Known` syntax structure are preserved without predicate decisions.
- [x] `openscad-rs/src/error.rs` — signals: `none`; disposition: reviewed; invalid-token diagnostics preserve exact byte ranges.
- [x] `openscad-rs/src/lexer.rs` — signals: `none`; disposition: reviewed; invalid tokens remain observable and include-path extraction cannot construct an inverted range.
- [x] `openscad-rs/src/lib.rs` — signals: `none`; disposition: reviewed; facade exports exact-number AST and traversal APIs only.
- [x] `openscad-rs/src/parser.rs` — signals: `scalar`; disposition: reviewed; exact numeric tokens are retained, lexer errors propagate, syntax modifiers are not discarded, and all AST spans cover only their own syntax.
- [x] `openscad-rs/src/span.rs` — signals: `none`; disposition: reviewed; public malformed spans have saturating length while parser-produced spans retain checked ordering.
- [x] `openscad-rs/src/token.rs` — signals: `scalar`; disposition: reviewed; decimal/hex parsing is exact and fallible; oversized eager powers use Hyperreal's exact lazy fallback.
- [x] `openscad-rs/src/visit.rs` — signals: `none`; disposition: reviewed; traversal covers all statement and expression descendants, including modified conditionals.
- [x] `openscad-rs/tests/openscad_compat.rs` — signals: `scalar`; disposition: reviewed; floating arithmetic is confined to non-semantic corpus pass-rate reporting.

## `synaps-cad`

The evaluator now carries exact `Real` literals and curve/triangle carriers
through all semantic operations. Comparisons, truthiness, ranges, extrema,
lookup ordering, signs, tessellation controls, hull orientation, equality, and
zero tests route through Hyperlimit; uncertainty becomes `undef`, a warning,
or an explicit failed shape. There is no fixed planarity tolerance, implicit
2D extrusion, failed-to-empty conversion, or unchanged-input offset fallback.
Finite values appear only at renderer/export/UI boundaries and in
compatibility-test measurements.

Eight sanitizer targets cover arbitrary and structured compilation,
expression control, shape operations/catalogs, mesh conversion, text, and the
full pipeline. Saved timeout reproduction improved from over 120 seconds to
7.2 seconds under ASan; bounded campaigns completed without a crash. OpenSCAD
visual and semantic fixtures cover mixed dimensions, flat 2D presentation,
extrusion, text, offsets, hulls, transforms, and exact literals.

- [x] `synaps-cad/Cargo.toml` — signals: `policy,scalar`; disposition: current exact-stack dependencies and crate-root repository links reviewed; fuzz/benchmark features resolve the centralized policy.
- [x] `synaps-cad/benches/compile_default.rs` — signals: `scalar`; disposition: benchmark preserves exact shapes until explicit renderer conversion and rejects failed geometry.
- [x] `synaps-cad/build.rs` — signals: `none`; disposition: platform resource metadata only.
- [x] `synaps-cad/fuzz/Cargo.toml` — signals: `policy`; disposition: eight bounded sanitizer binaries share the workspace Hyperlimit implementation.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_compile_arbitrary.rs` — signals: `none`; disposition: arbitrary-source compile result and warning invariants audited.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_compile_structured.rs` — signals: `none`; disposition: generated-valid OpenSCAD compilation and output invariants audited.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_expression_control.rs` — signals: `scalar`; disposition: exact expression, truthiness, range, assertion, and control-flow cascade covered.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_full_pipeline.rs` — signals: `none`; disposition: parse/evaluate/exact geometry/render pipeline invariants covered.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_mesh_conversion.rs` — signals: `none`; disposition: strict exact-to-renderer conversion, indices, finiteness, and nondegeneracy covered.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_shape_catalog.rs` — signals: `scalar`; disposition: native curve/mesh catalog, extrusion, revolve, and explicit failure invariants covered.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_shape_operations.rs` — signals: `scalar`; disposition: exact transforms and dimension-preserving operations covered without implicit coercion.
- [x] `synaps-cad/fuzz/fuzz_targets/fuzz_text_pipeline.rs` — signals: `none`; disposition: font selection, direction, alignment, extrusion, and rendering failure invariants covered.
- [x] `synaps-cad/fuzz/fuzz_targets/support.rs` — signals: `scalar`; disposition: bounded exact inputs and shared shape/render invariants reject malformed or nonfinite output.
- [x] `synaps-cad/src/app_config.rs` — signals: `none`; disposition: application configuration only.
- [x] `synaps-cad/src/compiler/evaluator/booleans.rs` — signals: `scalar`; disposition: exact 2D/3D operations propagate failure; mixed dimensions match OpenSCAD; planar hull orientation and ordering use Hyperlimit.
- [x] `synaps-cad/src/compiler/evaluator/builtins.rs` — signals: `scalar`; disposition: comparisons, equality, extrema, signs, and lookup ordering use centralized decisions and return `undef` on unresolved evidence.
- [x] `synaps-cad/src/compiler/evaluator/mod.rs` — signals: `scalar`; disposition: exact tessellation controls use Hyperlimit extrema; undecided or unrepresentable counts are explicit warnings with OpenSCAD-compatible fallback.
- [x] `synaps-cad/src/compiler/evaluator/primitives.rs` — signals: `none`; disposition: curve/mesh constructors validate complete numeric vectors and retain construction failures.
- [x] `synaps-cad/src/compiler/evaluator/tests.rs` — signals: `scalar`; disposition: exactness, OpenSCAD behavior, topology, and failure regressions audited; finite tolerances are compatibility-output measurements only.
- [x] `synaps-cad/src/compiler/evaluator/transformations.rs` — signals: `none`; disposition: extrusion/revolve and curve unions are fallible; 3D children are skipped with OpenSCAD warnings rather than coerced.
- [x] `synaps-cad/src/compiler/evaluator/value.rs` — signals: `scalar`; disposition: one centralized comparison cascade owns equality, truthiness, and bounded exact ranges; lossy conversion is explicitly interoperability-only.
- [x] `synaps-cad/src/compiler/geometry/conversions.rs` — signals: `scalar`; disposition: exact triangle uniqueness/nondegeneracy is certified before checked finite renderer projection.
- [x] `synaps-cad/src/compiler/geometry/mod.rs` — signals: `policy`; disposition: curve/triangle terminology is native; exact transforms and booleans propagate failure; zero/axis decisions use Hyperlimit; no tolerance or invented thickness remains.
- [x] `synaps-cad/src/compiler/mod.rs` — signals: `scalar`; disposition: exact shapes remain native until renderer projection; failed and canceled compilation remains observable.
- [x] `synaps-cad/src/compiler/rendering/colors.rs` — signals: `none`; disposition: named-color parsing is renderer metadata only.
- [x] `synaps-cad/src/compiler/rendering/fonts.rs` — signals: `none`; disposition: exact glyph-region union, bounds, placement, and alignment are fallible and uncertainty is retained.
- [x] `synaps-cad/src/compiler/rendering/mod.rs` — signals: `scalar`; disposition: finite tolerances, depth sorting, and normalization are confined to raster preview output.
- [x] `synaps-cad/src/compiler/types.rs` — signals: `none`; disposition: finite renderer/result transport only.
- [x] `synaps-cad/src/export.rs` — signals: `scalar`; disposition: finite mesh serialization and material deduplication only; exact topology is validated before this boundary.
- [x] `synaps-cad/src/lib.rs` — signals: `none`; disposition: application facade exports compiler/plugin surfaces without alternate decisions.
- [x] `synaps-cad/src/main.rs` — signals: `none`; disposition: application startup only.
- [x] `synaps-cad/src/plugins/ai_chat.rs` — signals: `scalar`; disposition: network/UI configuration, text patching, and message state only.
- [x] `synaps-cad/src/plugins/camera.rs` — signals: `scalar`; disposition: finite window/camera projection only.
- [x] `synaps-cad/src/plugins/code_editor.rs` — signals: `scalar`; disposition: source-text/view parsing only.
- [x] `synaps-cad/src/plugins/compilation.rs` — signals: `none`; disposition: background compilation state, cancellation, and result transport preserve explicit outcomes.
- [x] `synaps-cad/src/plugins/mod.rs` — signals: `none`; disposition: plugin composition only.
- [x] `synaps-cad/src/plugins/persistence.rs` — signals: `scalar`; disposition: serialized application state and file-system status only.
- [x] `synaps-cad/src/plugins/scene.rs` — signals: `policy,scalar`; disposition: finite Bevy mesh/grid construction occurs after exact compiler validation; no model predicate is decided here.
- [x] `synaps-cad/src/plugins/ui/chat.rs` — signals: `none`; disposition: chat markup presentation only.
- [x] `synaps-cad/src/plugins/ui/editor.rs` — signals: `none`; disposition: editor widgets only.
- [x] `synaps-cad/src/plugins/ui/layout.rs` — signals: `scalar`; disposition: animation timing, image indices, and configuration controls only.
- [x] `synaps-cad/src/plugins/ui/mod.rs` — signals: `none`; disposition: UI module composition only.
- [x] `synaps-cad/src/plugins/ui/resources.rs` — signals: `scalar`; disposition: finite frame-time monitoring only.
- [x] `synaps-cad/src/plugins/ui/systems.rs` — signals: `scalar`; disposition: finite image/performance graph presentation only.
- [x] `synaps-cad/src/plugins/ui/theme.rs` — signals: `scalar`; disposition: display colors and spacing only.
- [x] `synaps-cad/src/plugins/ui/utils.rs` — signals: `scalar`; disposition: image resizing/compression and UI texture caches only.
- [x] `synaps-cad/src/plugins/ui/viewport.rs` — signals: `scalar`; disposition: finite screen projection and depth ordering only.
