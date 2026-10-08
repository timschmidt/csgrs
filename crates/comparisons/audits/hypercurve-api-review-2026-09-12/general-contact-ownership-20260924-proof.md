# General parameter authority for owned chord contacts

Parent: Hypercurve `20f15803ebaf16b702af07a2fb3ab6f89c1f8fc6`.

Seven chord/rational contact helpers accepted only `BezierParameter2` for an
already-owned source contact. Three Boolean callers consequently discarded a
selected-fiber or recursive parameter with `as_bezier_parameter()`. This was an
API restriction on contact ownership, not a mathematical requirement. The
candidate carries `CurveParameter2` through those helpers and callers directly.
Ordinary scalar factorization and univariate algebraic diagonal deflation retain
their explicit narrow views. No projection or compatibility overload is added.

The collinear point-contact helper now also accepts and publishes the general
source parameter directly. Redundant wrapping/copying is removed. An existing
retraced-curve regression passes its returned contact parameter back directly,
removing a forced global promotion; its requirement to preserve the distinct
unowned preimage is unchanged.

The new regression uses P(t)=(t,(2t-1)^2) and the horizontal chord y=beta, where
beta=sqrt(1/2). The two parameters (1±sqrt(beta))/2 are selected independently by
singleton intervals in the fiber `(2t-1)^2-beta=0`. Each exclusion must preserve
exactly the other contact. The selection starts without a native Bezier view.
Both policies, chord directions, and homogeneous gauges 1 and -3 are exercised.
No recursive scalar or geometry Debug output is added.

The first frozen candidate's all-target build exposed the third Boolean caller
at the explicit endpoint-contact append path. V1 executed no tests and all
processes were reaped before that caller was updated. V2 passes both all-target
Clippy configurations with warnings denied. Its new regression passes, as do
all 121 public integration tests. The 300-case release qualification is still
running; it has so far reproduced the two already-recorded corner timeouts.

V1/V2 manifests, logs, binaries, case results, and terminal reports are recorded
under `general-contact-ownership-20260924-v{1,2}`. The candidate is not yet
committed. The selected-chamfer projection cost remains open.

## Next investigation: reuse existing projective root replay

`compute_recursive_projective_point` globally promotes every selected-fiber
parameter before importing its coordinates. Existing
`recursive_quadratic_polynomial_projective_roots` already constructs linear and
quadratic roots over a retained recursive coefficient field, and
`recursive_projective_point_from_recursive_parameter` already evaluates analytic
points in that field. A low-degree selected fiber can potentially connect these
two authorities by selecting the projective root through its certified original
singleton interval. A successful representation should be shared across clones
and refinements; unsupported or higher-degree cases must retain the complete
fallback. This is a hypothesis, not implemented or qualified behavior.

For the outstanding nonlinear selected chamfer, the retained center satisfies
`4u²-alpha=0` over a degree-65 alpha. A projective quadratic representation can
retain this relation without first constructing the degree-130 global u
polynomial. The newly generated chamfer cut may have higher local degree, so this
import alone does not establish completion of that workload.

The known boundary-root evidence also permits a separate sufficient conic contact
certificate: for a pole-free rational curve of homogeneous degree at most two,
one owned boundary root plus a certified strict incidence sign near that boundary
and the same strict incidence side at the opposite endpoint exclude every other
root. Another interior root would have to change that side; an even-multiplicity
interior root together with the boundary root exceeds degree two. The degree,
range, ownership, pole, and sign hypotheses all need executable controls before
this could replace any intersection work. No such shortcut is implemented yet.

## Qualified and committed

Hypercurve `8f0ff9a` commits the completed migration. V2 finished 300 release
attempts: 295 pass, two match the existing ignored tests, and three match the
existing 75-second corner timeouts. All 121 public integration cases pass. The
previously long passing selected-circle fillet kernel passes in 150.185 seconds
under its existing 180-second limit. Both all-target Clippy configurations and
Rustfmt checks pass. There are no new nonpasses.

The qualification, staged bytes, final HEAD, and all 2,044 source bindings are
recorded in `general-contact-ownership-20260924-{qualification,staged,post-commit}.json`.
Every owned process was reaped before staging and committing. All thirty workspace
repositories are clean. Nothing was pushed. The larger implementation goal and
selected-chamfer projection work remain active.
