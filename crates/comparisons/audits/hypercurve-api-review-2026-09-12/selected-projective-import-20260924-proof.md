# Pending selected-fiber projective import

Parent: Hypercurve `8f0ff9ad770f2596b85fba3897733a4460e92c2e`.

The current point importer globally promotes selected-fiber parameters before
building a recursive projective point. That duplicates an existing capability:
`recursive_quadratic_polynomial_projective_roots` solves linear and quadratic
polynomials in their retained coefficient fields, while
`recursive_projective_point_from_recursive_parameter` evaluates a source point
in that same projective field.

The candidate connects those existing authorities. It transposes the selected
fiber's coefficients into the retained base, invokes the existing projective
root constructor, and selects exactly one returned root using strict comparisons
against the original certified singleton interval. The constructor keeps positive
denominators; the singleton chooses the actual radical branch. Only a successful
strict representation is cached. Clones and interval refinements share one cache
owner with separate native-Bezier and projective representations. Equality still
depends on the original authority and selected root, never cache state.

The attempt runs under the existing bounded exact policy. A declined or failed
optional import leaves the previous complete projection path available. Point
evaluation in the new representation must itself succeed before it replaces that
path. No approximate decision selects a representation, no new public interface
or compatibility wrapper is added, and no global projection degree is tightened.

The new point regression uses P(t)=(2t²,t), a degree-65 retained alpha, both signs
of `4t²-alpha=0`, and the linear relation `sqrt(2)*t-alpha=0`. It checks both
policies and equation gauges 1 and -3. It requires a bounded point import, checks
that the global Bezier representation remains absent, that clones/refinements
share the cached projective parameter, and that the field retains one original
degree-65 source. Exact source/fiber equations and denominator/branch signs
independently check the resulting point. A separate cubic-fiber test checks that
declining this optional representation preserves replay of the defining equation.

The first driver invocation failed at Python parsing before any build started;
the missing newline was corrected only in the inactive driver. The first frozen
source candidate then failed compilation because the new test used a point's
`lifted_to` helper on a scalar value. V1 executed no tests, and every process was
reaped before correcting that test call to the field's existing `lift` method.
V2 now passes both all-target Clippy feature configurations with warnings denied.
The release build completed. Eight focused cases pass, including both new
regressions; the two existing selected-chamfer and extended-fillet cases still
exceed their unchanged 75-second limits. The broader qualification reuses that
exact copied libtest and builds five public suites from the same frozen inputs.
No candidate is committed yet.

Artifacts use the `selected-projective-import-20260924-v{1,2}` prefixes; their
source manifests bind 2,044 workspace and snapshot inputs. Existing immutable
snapshots are unchanged. The full implementation goal remains active.

The next bounded stack capture is `selected-projective-import-20260924-stack-
{terminal.json,log}`, bound to libtest SHA
`4ad89e7f3c11ce9843bbe67aaf84388dcaa0b6cc16be2e8534b1008be916d776`.
At eight seconds the selected chamfer is again in complete selected-fiber
projection beneath chord/rational incidence. The capture does not identify the
particular endpoint or fiber degree. It proves that the successful low-degree
import does not eliminate every later global projection. The debugger killed
and reaped its own inferior. Source bytes stayed frozen throughout qualification
and this capture.

## Qualified increment

Committed Hypercurve `45c1e7f174917f58bdd6b4ea3bcc833112cc79ab`. The broader frozen qualification has 424
passes, three unchanged ignored cases, and four unchanged baseline nonpasses
in 431 attempts, including all 121 public integration passes. Eight focused
cases also pass. Both all-target Clippy configurations deny warnings; Rustfmt
and whitespace checks pass. All 2,044 source bindings, copied executables,
staged bytes and HEAD match. All processes are terminal/reaped and all thirty
repositories are clean. Qualification and commit bindings use this prefix's
`qualification`, `staged`, and `post-commit` JSON reports. Nothing was pushed.
