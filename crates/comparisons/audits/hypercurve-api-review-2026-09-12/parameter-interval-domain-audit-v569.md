# Ordered finite parameter intervals (continuation audit V569)

Read-only source audit during V567. No production change or new build. V567
owns the workspace, mirror, and live archive until exact outer 46660 is reaped.
The primitive tangent candidate V568 is the next prepared implementation probe.

The interval type already stores generic ordered exact bounds. Its private
`try_new_ordered` constructor is used by exterior selected fibers, root imports,
refinement, and incident bridges. Its public `try_new` adds a unit-domain check
which does not belong to this representation. Remove the private alternate
constructor and migrate callers directly; do not add an alias. Correct the
interval and oriented-range documentation to describe their actual domains.

Production ordinary-constructor call-site audit:

- `exact_nonrational_bernstein_unit_roots` and
  `exact_rational_square_free_bernstein_unit_roots` begin at [0,1] and subdivide
  that bracket. Their unit-domain proof is independent of the constructor.
- `refine_algebraic_cusp_semicircle_parameter_bracket` begins at [0,1] or a
  preceding certified bracket and bisects it. `mapped_parameter_bracket`
  combines bounds from that same refinement routine (or complements its
  original bracket). `complement_cusp_parameter_bracket` implements 1-t.
- `cusp_chamfer_parameter_bracket` explicitly clips the transported bounds to
  [0,1], using the selected target's existing chart-membership certificate.
- `represented_parallel_endpoint_oblique_chord_intersections_in_domain`
  initializes the direct selected chord bracket at [0,1] and refines inside
  it. Its defining relation and sign-change tests own the root evidence.
- `locally_certified_rational_image_parameter` explicitly clips its rational
  enclosure to [0,1] before the Bernstein singleton test.
- `from_monotone_span` merely adapts `BezierMonotoneSpan`. That source type's
  public constructor proves ordering, not unit membership; broadening this
  adapter preserves the original exact span rather than imposing a second
  domain interpretation.
- `from_algebraic_root_representation_with_domain` is the identified caller
  whose unit-mode path currently relies on `try_new` to validate both bounds.
  Preserve this check explicitly at that importer when consolidating interval
  construction. Its exact-point, represented upper-root, and linear-root paths
  separately use the existing unit-validated `BezierParameter2::exact`.

Other production interval construction uses `try_new_ordered` already. The
remaining explicit calls in the V558 inventory are test/benchmark fixtures
with known endpoints; update all private-constructor uses along with production.
Text inventory alone is not a proof of every indirect consumer invariant.

Native evaluation and rational-pole audit:

- `Curve2::point_at_selected_native_parameter` orders the selected parameter
  against each authored fragment's start/end before constructing its local
  source parameter. Exterior roots fail native admission independently.
- `validate_rational_point` uses common-sign control weights only after proving
  that the selected parameter is in the closed unit span. Otherwise it signs
  the actual denominator polynomial at that parameter.
- `RationalBezier2::point_at_classified` explicitly checks unit membership;
  its internal affine counterpart evaluates and projects the actual weight.
- Raw quadratic/cubic polynomial algebraic image methods evaluate their
  polynomial images without a unit assumption. The raw rational algebraic
  methods call `rational_point_image_from_power_basis`, reduce in the selected
  parameter field, and validate the denominator through Hypersolve rational
  images or exact polynomial sign replay. They do not use positive Bernstein
  weights to justify an exterior parameter.
- `RationalBezierAlgebraicPointImage2::from_parametric_source` retains its
  authority lazily; resolution uses the same rational-image method. Admission
  through the general curve interface has the preceding actual-domain check.

Planned contract tests for the migration: isolate both roots of t²-2 on
negative/exterior brackets; retain their equations and order under both
policies; reject reversed intervals; preserve native curve-domain rejection;
reject an exterior rational pole even when all unit-span control weights have
one sign. Include importer unit-mode coverage and the monotone-span adapter.
These tests and the migration have not been written or run yet.


V577 follow-through (production frozen): all three new contract regressions
pass in the normal nine-binary build. The complete qualification is still
running. The first V573 build failed only because the importer fixture omitted
constraint_index; exact outer 3438 was reaped before that test-only repair.

Additional native subdivision audit: the public quadratic/cubic/rational
split methods pass an explicit [0,1] range to split_curve_at_parameters. That
shared routine sorts the cuts and requires the first and last boundaries to
remain exactly equal to the requested range endpoints before materializing any
fragments. An exterior cut therefore cannot silently extend a native split.
validate_exact_range also checks its actual unit bounds. The internal refined
split path already distinguishes unit subcurves from exterior affine charts.
These are operation-owned checks independent of interval construction.
