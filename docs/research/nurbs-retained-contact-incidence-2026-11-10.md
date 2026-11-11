# Retained NURBS fillet contact incidence (2026-11-10)

Hypercurve owns complete inverse incidence on each rational span under P15,
P17, P18, P19 and P20. A generated point can retain its original parameter
without storing independent scalar coordinates. The caller still needs all
visits on the target curve, including other spans and selected endpoints.

The original hard NURBS pair has one trim-only fillet of radius `1/2`.
Independent exact certificates bound both contact parameters and prove that
each original span is injective. On `30ea60023a`, the generated fillet's
endpoints replay as strict zero-distance evaluations of the original rational
spans, with exact matching parameters. An inverse diagnostic without an
internal work allowance reached its external wall limit. The bounded regression reported
`RootRefinement` after all 64 admitted units. Its promotion allowance was zero.

The inverse query now uses the retained parameter for a matching native
zero-distance evaluation. The existing point accessor checks the retained
policy and rejects replacement tangent frames, tangent displacement and
translation. The span proves denominator nonvanishing and injectivity on its
native unit chart. The known parameter and the entire requested range must lie
in that chart. Strict clipping then determines whether the unique visit belongs
to the requested span. Inconclusive or inapplicable cases use the general
homogeneous preimage solver. Enumeration across other spans still runs.

The original hard seed now locates both contacts in the bounded query under
both policies, compares them against the independent parameter enclosures and
evaluates the original curves at those exact parameters. Additional witnesses
cover a selected trim endpoint, reversed traversal, an omitted contact, two
visits on a folded span, two visits across a retraced spline, and a valid
exterior visit beyond the certified unit chart. The retraced spline uses the
exact unit-weight rational embedding of its polynomial quadratic span.

The allowance covers instrumented refinement and promotion. The round stroke
of the original filleted pair and total resource bounds remain open.

The final source qualification passes the individual hard regression and all
nine selected rational location tests. The five affected Hypercurve harnesses
record 101 passes and 13 existing ignores. Workspace formatting, all-target
and all-feature Hypercurve Clippy with `-D warnings`, and the four new
capability witnesses pass. The all-feature release BRep and BRep IO run
records 996 passes across 110 harnesses, including doctests.

An earlier test used a retraced NURBS span that specialized to a polynomial
quadratic. Its exact unit-weight embedding exercises the intended rational
path with the same visits. The original failure and the corrected parent
controls are retained in the local qualification evidence. A separate
round-stroke diagnostic generated the unique fillet and reached its
900-second external watchdog before it returned a region.
