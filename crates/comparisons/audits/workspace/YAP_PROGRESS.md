# Yap exact-geometric-computation optimization audit

Source: `yap.pdf`, Chee-Keng Yap, “Towards Exact Geometric Computation,”
*Computational Geometry* 7 (1997), pp. 3–23.

## Marker

- Current reading marker: end of the paper, including §7 and Appendix A, PDF
  page 21 of 21 (journal page 21).
- Next experiment: none from the unread text. Future common-representation work
  remains benchmark-driven and must preserve every existing control workload.
- Completion rule: advance the marker only after each concrete suggestion is
  mapped to existing code, attempted with a benchmark when feasible, and
  recorded as kept, rejected, already satisfied, or architecture-inapplicable.

## Audit through the marker

| Paper location | Suggestion | Current evidence/status |
| --- | --- | --- |
| §§2–3, pp. 4–11 | Exact object representation and error-free decisions; internal approximation is permitted. | Already satisfied by `hyperreal::Real`, explicit uncertainty/certificates in `hyperlimit`, and exact imported rational/dyadic values. Conceptual requirement, not a standalone optimization. |
| §3.2, p. 11 | Replace disguised transcendental decisions by algebraic relations where possible. | Partially satisfied by exact symbolic identities and polynomial geometric predicates. Keep as an ongoing design constraint; no isolated benchmark candidate in this passage. |
| §4, p. 12 | Exploit rational bounded depth and predictable intermediate size. | Already represented by exact-rational structural facts, storage classes, bounded fixed-dimensional predicates, and integer/rational fast paths. |
| §4 example, p. 12 | Use Gaussian/Bareiss fraction-free determinant evaluation; retain the tighter `D = k` growth bound. | Kept: exact fixed-dimensional reducers and the exact-rational ND Bareiss path are present in `hyperlimit`. |
| §5, p. 15 | Avoid general BigNumber overhead when required precision/size is predictable; specialize small `D`. | Kept: word-sized rational arithmetic, small-storage dispatch, delayed reduction, dyadic shift schedules, and fallback to arbitrary precision. |
| §6, p. 15 | Provide a determinant/sign package above the scalar number package. | Kept: `hyperlimit` owns determinant predicates and sign outcomes; scalar reducers remain in `hyperreal`. |
| §6.1, p. 16 | Per-value arbitrary precision, dynamically refined only enough for an error-free decision; decouple precision from magnitude. | Kept: `Computable` refinement, certified dyadic intervals/balls, sign-until APIs, and predicate escalation. |
| §6.2, pp. 16–17 | Compile recurring expressions; preprocess static bounds and structure; use floating filters, dynamic bounds, incremental/lazy evaluation at runtime. | Partially kept: one-shot and prepared certified determinant/incircle/insphere/linear-form filters, prepared geometric objects, and lazy real expressions. Current work extends compilation to exact non-dyadic repeated queries. |
| §6.2, p. 17 | Generalized product/sum, constant multiply/add, fixed powers, and sign-classified sums. | Kept: fixed-size dot/product-sum reducers, constant/identity shortcuts, power specializations, and positive/negative accumulator separation. |
| §6.2, p. 17 | Expression variables retain a defining expression, approximation, and error bound and can be reevaluated. | Kept in the heterogeneous `Real`/`Computable` expression and refinement architecture. |
| §6.3, p. 17 | Encapsulate points, hyperplanes, incidence, and intersection in geometric object packages. | Kept across `hyperlattice` object carriers and `hyperlimit` predicates/reports. |
| §6.3, p. 18 | Convert rational vectors to a common homogeneous representation; compute complementary denominator products with `2k-2` multiplications and avoid per-coordinate GCD reduction. | Kept for repeated 2D line and 3D oriented-plane queries. Fixed rational points are converted once to homogeneous word-sized coefficients without GCD reduction; queries use checked `i128` and retain arbitrary-precision fallback. Value-gated layouts preserve the existing dyadic paths. Earlier same-denominator determinant expansions remain rejected because they regressed compact rational reducers. |
| §6.4, p. 19 | Permit heterogeneous internal number representations and seamless conversion rather than requiring one positional BigNum representation. | Already satisfied: `Real` ties exact rationals, symbolic constants/expressions, computable refinement, dyadic views, and word/arbitrary-precision storage together behind one API with explicit conversion and fallback. |
| §7, p. 19 | Treat exact computation as a layered software discipline and exploit predictable rational bounded-depth work. | Summary of the audited design constraints; no additional isolated optimization. |
| Appendix A, pp. 20–21 | Semialgebraic finiteness argument for Euclidean shortest paths. | Proof material, architecture-inapplicable to the arithmetic and predicate benchmark surface. |

## Experiment log

| Experiment | Result | Decision |
| --- | --- | --- |
| Prepared certified filters for line, oriented plane, explicit plane, incircle, and insphere. | Large repeated-query wins; construction cost recovered quickly. | Kept. |
| Ordinary explicit-plane one-shot certified linear form. | Direct point/plane improved; failed-filter cost regressed exact-rational composers. | Kept on direct/batch classification. Composed halfspace, convex, and segment routes retain their unfiltered ladder; triangle classification now prepares once for its three tests. |
| Shared/common-denominator determinant expansions ahead of compact rational kernels. | Regressed the affected exact-rational predicate rows. | Rejected; compact rational kernels retained. |
| Prepared exact-rational homogeneous 2D line filter. | Transformed prepared line: 238.24 µs to 11.63 µs per 512 queries (-95.3%). Filter construction is about 35.8 ns, so the first query repays it. Separate optional fields preserve `PreparedLine2: Copy`; saved dyadic controls improved about 3.3% and 6.7%. | Kept, automatically selected at construction when the dyadic filter is ineligible. |
| Prepared exact-rational homogeneous 3D oriented-plane filter. | Transformed prepared plane: 144.46 µs to 20.78 µs per 512 queries (-85.6%). Exact filter construction is about 171.7 ns; the gated boxed layout adds about 115 ns to full rational preparation, less than the roughly 242 ns saved by its first query. A direct A/B found no dyadic regression. | Kept behind a construction-time value gate; the legacy dyadic field and hot branch remain unchanged. |
| Reconsider removed one-shot point/plane filtering inside composed predicates. | Rebuilding the one-shot filter for every segment/triangle point regressed dyadic rows 39.7%/46.8%. Preparing once per segment was neutral for dyadics and regressed exact rationals 5.9%. Preparing once per triangle improved dyadics 4.5% and was neutral for exact rationals. | Restore only prepare-once triangle reuse. Keep segment, convex, and halfspace single-comparison routes unfiltered. |
