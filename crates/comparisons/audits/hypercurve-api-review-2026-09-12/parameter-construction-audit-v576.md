# Remaining parameter construction split (V576, read-only)

The V572 interval migration is under V577 qualification after the V573 fixture
build error was repaired. No further production API changes during that run.

The 2,047-file workspace inventory contains 24 explicit
`BezierParameter2::exact` calls and 55 `BezierParameter2::algebraic` calls, all
in Hypercurve. Locations are recorded in parameter-construction-uses-v576.json.
The three `Self::exact` calls in the root importer are additional and must be
included in a semantic migration. This is an inventory, not a completed audit
of every indirect wrapper or imported alias.

`BezierParameter2` already admits an arbitrary exact scalar through its public
Exact variant, but the public `exact(value, policy)` convenience constructor
silently enforces [0,1]. Its `algebraic` counterpart is a plain enum wrapper.
This is another opportunity to remove ambiguous parallel interfaces instead
of preserving them as compatibility entry points. The general CurveParameter2
interface already separates exact value construction from operation-domain
admission.

The direct production exact-constructor uses have real domain semantics:

- `real_coefficient_rational_image_parameter` rejects a represented target
  outside the native conic chart as no parameter.
- `exact_rational_parameter_image` does this only when its unit_domain flag is
  set; the other branch already uses the generic Exact variant.
- `supporting_line_contact_evidence_impl` clips the contact's line parameter
  only when affine_line_parameter is false. Its affine branch already retains
  arbitrary exact values. Preserve the distinction between finite membership
  and an infinite supporting-line witness.
- The three root importer branches require unit checks for exact point,
  represented upper endpoint and linear polynomial results when importing to
  a native unit domain. The V572 nonlinear branch now checks its bounds
  explicitly.

A future cleanup can replace the wrapper constructors with direct value
construction and explicit membership predicates at these operations, updating
tests, benchmarks and all three affected fuzz targets. Do not simply remove
the checks or introduce renamed forwarding aliases. Prioritize the unrun V575
recursive point-constraint closure probe before this optional API cleanup.

Related documentation observation: Hypersolve's AlgebraicRootRepresentation
interval field still says "Certified unit isolating interval", although the
same carrier represents arbitrary exact point roots and retained exterior
finite roots. Correct the owning-layer documentation when that API is next
edited; it should not imply a false representational restriction.
