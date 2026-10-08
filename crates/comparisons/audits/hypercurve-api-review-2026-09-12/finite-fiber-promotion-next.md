# Finite selected-fiber projection and promotion

The finite circle/rational audit at Hypercurve 63dc97aba4fe578ceb11dee3bdc50ee2307f3884 found shared selected-fiber assumptions that must be corrected before widening geometric dispatch:

- A general resultant fallback accepts an optional finite range but counts a degenerate projection only in the unit interval.
- High-degree norm replay uses unit-constrained scalar and interval constructors even for exterior ranges.
- Scalar promotion schedules the unit interval or an incident ray, and its complete continuation builds a compact affine chart and maps the result back. A root already owns a finite certified isolator that can schedule this projection directly.

The implementation will make finite ranges explicit in the common fiber projection helpers, use the existing CurveParameterDomain2 envelope authority for scheduling, preserve original selected boundaries for membership, and project scalar promotion in its existing isolator. The complete continuation retains its uncapped degree schedule. Cached representations and local selected-root identities remain authoritative. Remove the complete-unit helper and the promotion chart construction; migrate callers directly.

Baseline regressions are recorded before production edits. Qualification must cover exterior represented and algebraic roots, reversed and selected finite bounds, policy certainty, imported/refined cached identities, conjugate rejection, and existing corner/region/Boolean callers. The nonlinear circle dispatcher remains guarded until its geometric range and denominator contracts are migrated separately. No performance claim without a controlled comparison.
