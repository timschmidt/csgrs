# Tangent parameter follow-up (read-only design)

CurveTangent2::RetainedParallel still stores both a BezierParameter2 and an
optional original selected-fiber parameter. exact_parallel_region_endpoint_tangent
promotes the selected fiber before admitting this carrier. This is an internal
representation split left after the public CurveParameter2 migration.

The original selected authority is semantically useful: source-circle/parallel
overlap must compare the compact selected identity instead of reproving equality
against a global projected root. A single CurveParameter2 can retain that meaning.
The promoted native parameter is derived evidence and should be requested only
by a theorem that actually needs its univariate payload, using the existing
selected authority cache. The field should not store two competing parameters.

This is not a mechanical enum-field substitution. Consumers include round joins,
parallel/parallel cross and dot signs, selected-circle tangent comparisons and
source-overlap recovery. The selected-circle source-overlap routine in
bezier_offset.rs currently takes both parameters; its selected and mapped arms
can instead compare their CurveParameter2 values directly. The other circle
routines still accept only BezierParameter2 and require either migration to the
common parameter or a guarded native fast path plus their existing general
chord-tangent proof. Merely deleting promotion would lose those capabilities.

Preserve source_parallel, composed parallel, finite source_range and orientation:
they own the offset anchor, pole-free domain and tangent direction semantics.
Do not replace them with an unrelated Cartesian vector or globally promoted
endpoint. Removing duplicate parameter storage must preserve local overlap
proofs and the complete existing fallback, with both policy modes and traversal
reversal. The current mixed-carrier offsets, selected-companion fillets,
recursive selected-radial chamfers and homogeneous composition tests are
necessary qualification targets. Duplicate tangent-parameter storage has not
yet been migrated.

The first bounded migration is complete in Hypercurve c13778e: all three
vector_tangent_cross_and_dot_signs methods and the source-tangent linear form
now accept CurveParameter2 and evaluate through its polynomial-sign authority.
All callers were migrated directly. Independent selected and recursive
parameter tests, regularized stationary tangents, both policies and traversal
directions pass; the projected Bezier cache remains empty. The regular-range
path retains its one-sided cancelled tangent and certified interior derivative
scale. See general-vector-tangents-20260924-proof.md and its qualification.
exact_retained_parallel_represented_tangent only needs scalar(), which the
common parameter already provides.

The pair-tangent methods are different: their bivariate predicate combines two
parameter authorities, including possible shared sources. Do not substitute
independent global roots mechanically. Preserve the shared-source speed proof
and native bivariate specialization, then use the common retained chord/field
authority for other scalar pairs. Round joins and selected-circle predicates
also require the explicit guarded native theorem plus existing general proof.
The complete promotion method on CurveParameter2 is available, but making it
unconditional here would undo the retained-field scheduling improvements.

Concrete shared-field building blocks already exist in bezier_offset.rs:
recursive_projective_bivariate_first_parameter_polynomial substitutes the
first scalar using one common homogeneous degree, so its positive denominator
does not change a sign. recursive_polynomial_sign_joined can replay the second
root over the least shared coefficient field without erasing its defining
correlations. Any general parameter-pair predicate should try structural
same-parameter reduction and represented-axis specialization first, keep
signed_bivariate_at_parameter_pair for two native parameters, and then reuse
these retained-field authorities where their preconditions hold. Two local
polynomial roots do not necessarily expose explicit projective scalars; the
substitution helper can decline and must leave the complete fallback intact.

The selected-circle layer also has a bounded migration path. Its existing
endpoint_tangent_cross_dot_retained_parallel_by_chords already accepts
CurveParameter2. The cross and linear-combination entry points still take
BezierParameter2 chiefly for exact_endpoint_tangent_cross_dot_retained_parallel
(a native parameter-pair theorem), plus equality against mapped contacts.
Mapped-contact equality and source-overlap equality can use same_value on the
single general parameter directly; no projected parameter is needed there.
Do not force projection before consulting those existing identity proofs.
The normal-circle constructor still stores a native center parameter in its
frame. A round join requiring that specialized frame must request its native
evidence locally or use the existing retained-center/chord-normal frame.

The overlap entry point can lose its two-parameter signature independently:
endpoint_tangent_dot_retained_parallel_source_overlap currently branches
between candidate.cmp_by_refinement(selected_source_parameter) and
candidate.cmp_bezier_parameter(parameter). Both can become the existing
CurveParameter2::same_value predicate on the one caller-retained parameter.
The mapped-overlap arm already uses that authority. Its region caller,
exact_selected_circle_retained_parallel_tangent_cross_and_dot, also has an
unused `_parallel` argument to remove with all callers. Preserve source_parallel,
source_range, source_direction and the authored source fragment: those carry
the exact positive-length overlap and orientation proof.

For the circle cross/dot entry points, a native compact theorem remains a
valid specialization, selected by as_bezier_parameter without promotion.
The existing general by-chords fallback already accepts CurveParameter2.
Mapped-contact same-value checks and derivative-scale checks can consume the
general parameter directly. Reuse construction identity before asking either
fallback to rebuild a common tangent field, and retain a complete fallback
when a bounded local comparison declines.

Another current accidental restriction is in the oriented tangent-chord
constructor itself: from_certified_retained_parallel_oriented_unit_tangent
accepts a CurveParameter2 but admits only selected and recursive variants.
Its endpoint representation already supports ordinary Bezier parameters.
The component-sign dispatch can consume the common polynomial-sign authority,
and native admission can make this one fallback usable by mixed parameter
pairs as well. Preserve its strict nonzero-component certificate, demand-driven
choice of monotone axis, traversal direction and unoffset source anchor. A
stationary endpoint needs the regularized one-sided tangent on its certified
source range; a vanishing raw derivative is not that certificate.

Further caller review during retained-root qualification identifies two more
migration obligations. The miter support builder constructs a displaced
BezierAnalyticParallelPoint2 with the native-only constructor; it must use the
existing general-parameter construction while keeping the actual offset anchor.
The selected normal-circle round-join helper currently receives both parameter
copies, promotes the native copy into its specialized frame, then switches back
to the original selected parameter for terminal contact identity. A single
CurveParameter2 should remain authoritative throughout. Request a native frame
only within the specialized circle theorem, and allow the already available
retained-center/chord-normal construction when that theorem cannot admit the
parameter. Returning an early uncertainty from the specialized helper must not
prevent the complete general round-join path from running.

The ordinary endpoint tangent constructor has one caller, the general endpoint
constructor. After migration, their duplicate direction calculation and native
wrapper boundary can disappear rather than remain as forwarding interfaces.
Keep scalar-domain admission and source-range regularity explicit; a circle's
local parameter is not automatically a rational source parameter merely because
both use CurveParameter2.

The current local-chord candidate also removes the native-only normal-circle
frame restriction: ParallelNormal and selected parallel contacts retain
CurveParameter2, including affine canonical tangent authority. The remaining
offset tangent storage migration can call that constructor directly; it no
longer needs to manufacture a chord-normal frame solely for a local parameter.

Local polynomial parameters now retain one normalized projective chart to the
original selected root. A subsequent pair-predicate migration can exploit two
charts of that same selection before treating them as independent field axes.
Pull back the bivariate expression onto the common source root and replay its
polynomial sign locally. Preserve denominator signs: these charts are invertible
at the selected root, but their denominators are not universally positive.
Even homogeneous clearing degrees provide a positive nonzero denominator
without requiring a represented scalar, or signs can be certified explicitly.
Mere overlapping enclosures never establish a common selection. Keep native
bivariate specialization and the complete existing fallback for unrelated roots.
