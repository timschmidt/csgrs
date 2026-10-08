# Local equality of retained analytic point frames

Committed Hypercurve `5c115f5171c0c90623cddaa7c268bac4d316d758`.

Parent: Hypercurve `3eabc047e4e826385aa6ab7720cdc966dc06f1a1`. HyperBREP remains `878fcb441b281ab1b44051ca4420b28092c3c2ed`. This step addresses computational and evidentiary closure within the active architecture goal.

## Defect and independent geometry

The public circle fixture intersects an exact retained radius-1/4 fillet with the left distance-1/64 parallel of `P(t)=(-1/8+t^2,t)`, `t in [0,1/8]`. The independent sign inequalities proving a unique transverse contact are recorded in `finite-circle-analytic-domains-certificate.md`. The rational chart `t=u/(2u+6)`, `u in [0,1]`, has the same oriented image. Its controls are `(-1/8,0),(-1/8,1/16),(-7/64,1/8)`, weights `9,12,16`.

The parent constructed this curve successfully, then exceeded 120 seconds in the combined public contact/replay matrix and 60 seconds in a separate native-chart process. The frozen parent sources, executable and library hashes are recorded in `finite-circle-native-poles-baseline.json` and `finite-circle-native-poles-charts-baseline.json`.

An approved debugger run interrupted the same parent executable after five seconds. `selected-point-frame-replay-parent-stack.json` binds the trace to its artifacts. The contact had already been found: public point coincidence had reached selected-fiber global promotion through `BezierAnalyticParallelPoint2::represented_coordinates`, spending time in a local Sturm sequence. The earlier phase logs did not distinguish intersection from the subsequent public point replay; the stack resolves that ambiguity.

## Change and proof

Contact evidence can retain the raw homogeneous tangent, while evaluation on a regular fragment retains a tangent with a common hodograph factor removed. Both are exact representations of the same unit frame, but their structures differ. Structural equality therefore fell through to Cartesian reconstruction.

For a shared source point and translation, with identical tangent displacement `a` and normal displacement `d`, the point is `P + (a I + d J) U + T`, where `U` is the unit tangent. If both displacements vanish, the frame is unnecessary. Otherwise the displacement map has determinant `a^2+d^2>0`, and equality is equivalent to equality of unit directions. Zero cross product and positive dot product prove equal directions; nonzero cross product or negative dot product prove distinct directions. A zero frame is left to its existing one-sided/degeneracy authority.

The analytic-point equality path now signs these polynomials in the existing shared parameter field. All decisions run in strict predicate scope under the original retained policy. Unrelated parameters, supports or displacements retain the existing complete equality path. The duplicate structural `shares_carrier` helper is removed and its callers use `PartialEq` directly. No new stored field, public interface, compatibility wrapper or dependency is introduced.

## Regression scope

The new unit regression derives the native chart's tangent independently: `H=(3/2)(u+3)*(u,u+3)`. It compares the raw frame against `(u,u+3)`, its negative and its quarter-turn. Both policies, three normal displacements, two tangent displacements and both operand orders give 72 certified comparisons. The selected parameter's global representation cache and both recursive Cartesian caches must remain empty.

The existing public integration regression now includes both polynomial charts and the native rational chart: 48 contact queries and 96 exact location/point replays, covering both policies, traversal reversals and operand orders. Its focused run completes in approximately 4.4 seconds. The separate unchanged 64-case public source now completes in approximately 4.2 seconds with 48 contacts, 96 point replays and 16 remaining rational-exterior Boundary blockers. These timings are local observations under uncontrolled other-session system load, not a general throughput claim.

Six focused tests pass, including independent algebraic point equality, reduced-frame axis ordering, retained policy identity and the explicit cold global-materialization boundary. The latter confirms that complete reconstruction remains available when needed. Final qualification records 2,300 passes across 49 targets (2,068 Hypercurve and 232 HyperBREP), five unchanged known failures, nine ignored tests, eight existing expensive exclusions and no new failures or broad-run timeouts. All 36 public probes and six focused tests pass. Both no-default-feature caller checks and all-feature release builds pass. Formatting, toolchain, 396 working-source hashes, 1,005 isolated-source hashes and artifact hashes verify; see `selected-point-frame-replay-qualification.json`. The successful qualification writer must not be rerun against changed sources. The final archived normal library has SHA-256 `476ce4085af4463f86d6182e476e0fa13af9266727aaefa54be4b0293d704b2a`; the libtest SHA-256 is `e90f7bb5aac4c6ca2cd8b9e7f629bd71bc410f3f7023f22c4cd3a28268c4894f`, under `selected-point-frame-replay-libraries/`.

## Remaining scope and ownership

The rational exterior source still has an unused genuine pole at parameter 1/2, outside its active `[2,5/2]` range. Lower circle-equation builders continue to require native whole-unit finiteness/regularity. This remains a required next migration, recorded in `finite-circle-native-poles-next.md`; the 16 expected remaining blockers are not counted among passing public probes.

Selected-point finite incidence, source-cusp frames, normalized public-region construction, independent inverse replay, the five known failures/eight expensive exclusions and broader algebraic/computational consolidation remain unfinished. Builds use pinned committed dependencies, and the other session's Hyperreal work is untouched. No global workspace-clean claim is made.
