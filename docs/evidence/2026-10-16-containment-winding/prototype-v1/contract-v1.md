# Nonzero-winding solid containment

Base revision: `d4eee9a675fa51d84eb0eb29503dd802bdcdbd19`.

The owning layer is Hypermesh's winding machinery and native surface
arrangement. P6/P8/P9/P17/P18/P21/P22/P24/P25/P30/P36 apply. CSGRS and WASM
consume the native result; they must not implement another classifier.

`contains_point` must decide strict interior of the regularized set whose
source winding is nonzero. Global orientation reversal, positive winding
multiplicity, disconnected components, nested material, holes, coincident
opposite shells and internal source faces must follow that same set meaning.
The external set boundary returns false; an internal source face can be inside.
Canonical empty and invalid-input behavior from the preceding admission stays
intact. Policy-qualified outcomes must keep every consumed certainty fact.

The correction deliberately replaces the old documented ray-parity/source-face
exclusion behavior. No supported set operation uses that parity interpretation
for its individual source operands. Wrong definite results must become the
independently expected values, not new Unknowns, errors or lower-resolution
fixtures. Actual unresolved predicate decisions remain typed; approximate
policy selection alone does not qualify an otherwise exact answer.

Before implementation, run the unchanged committed native and actual WASM
libraries against new tests. The independent oracle sums signed box membership
in each of eight adjacent open octants around half-integer queries. Integer
source planes and an offset of 1/8 prove that this samples every local cell;
strict interior requires nonzero winding in all octants. Include exact affine
invariance, tangent-ray contacts and unary regularization as additional checks.
Agreement between wrappers or Boolean paths is not the sole oracle.

Reuse the existing signed ray/triangle primitive and finite direction family
already used by Minkowski cell winding. Boundary queries must distinguish
source faces from the selected regularized boundary through the existing
native arrangement. Preserve Minkowski cancellation, typed outcomes, input
validation, public mesh storage and source coordinates; any diagnostic wording
change must be identified. Do not replace authoritative geometry with a
normalized rendering/export approximation or retain an unbounded query cache.

The finite ray-family count is not a total-work, heap, expression-growth or
cancellation bound. Boundary classification may require native arrangement
work. These limitations must be documented and dangerous validation externally
contained. Measure the full completed contract, cold and warm resource use,
allocations, peak requested bytes and artifacts; distinguish baseline wrong
answers from completed correct work. This is a correctness migration, not a
performance-only optimization.

Acceptance includes new native/modeling and real WASM consumers, existing
source-validation regressions, full affected native suites and feature
configurations, Clippy, docs, formatting, immutable source/lock/toolchain pins,
and raw cost observations. The complete properties plan, historical artifact
audit, external distribution, coverage and pre-push matrix remain open.
